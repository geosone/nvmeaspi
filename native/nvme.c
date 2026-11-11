/* SPDX-License-Identifier: GPL-2.0-or-later */
/* NVMe transport for OpenASPI's native layer (polled).
 *
 * Bring-up per NVMe 1.4 section 7.6.1: disable the controller, set up the
 * admin queue pair, enable, IDENTIFY the controller and its namespaces,
 * create one I/O completion and submission queue without interrupts.
 * SCSI requests go through OpenASPI's block device emulation; this file
 * only reads, writes and flushes blocks and maps NVMe status codes to SCSI
 * sense data (as in the NVM Express SCSI translation reference).
 *
 * Data buffers are described with PRPs page by page (oa_host_virt_to_bus
 * for each page); a buffer that is not dword aligned goes through a bounce
 * buffer. 32/64-bit only: registers and queues need a flat address space.
 */

#include <string.h>

#include "nvmeaspi/native.h"
#include "openaspi/native/blockemu.h"
#include "openaspi/native/hba.h"
#include "openaspi/native/host.h"
#include "openaspi/scsi.h"

#define MAX_CONTROLLERS 4
#define MAX_NAMESPACES  16
#define QUEUE_ENTRIES   16u
#define PAGE            4096u
#define BOUNCE_SIZE     65536u
#define PRP_ENTRIES     (PAGE / 8u)

/* controller registers */
#define REG_CAP   0x00u
#define REG_VS    0x08u
#define REG_CC    0x14u
#define REG_CSTS  0x1Cu
#define REG_AQA   0x24u
#define REG_ASQ   0x28u
#define REG_ACQ   0x30u
#define REG_DOORBELL 0x1000u

#define CC_EN      0x00000001ul
#define CC_IOSQES  (6ul << 16) /* 64-byte submission entries */
#define CC_IOCQES  (4ul << 20) /* 16-byte completion entries */
#define CSTS_RDY   0x00000001ul
#define CSTS_CFS   0x00000002ul

/* admin opcodes */
#define ADM_CREATE_SQ 0x01u
#define ADM_CREATE_CQ 0x05u
#define ADM_IDENTIFY  0x06u
/* NVM opcodes */
#define NVM_FLUSH 0x00u
#define NVM_WRITE 0x01u
#define NVM_READ  0x02u

struct queue {
    volatile oa_u32 *sq; /* QUEUE_ENTRIES x 16 dwords */
    volatile oa_u32 *cq; /* QUEUE_ENTRIES x 4 dwords */
    oa_u64 sq_bus, cq_bus;
    oa_u16 qid;
    oa_u16 sq_tail, cq_head;
    oa_u8 phase;
    oa_host_event *ev;   /* completion event, NULL = poll this queue */
};

struct ctrl;

struct ns {
    struct ctrl *c;
    oa_u32 nsid;
    struct oa_blockemu emu;
};

struct ctrl {
    struct oa_hba hba;
    char name[32];
    volatile oa_u8 *regs;
    oa_u32 dstrd;           /* doorbell stride in bytes */
    oa_u32 timeout_ms;      /* CAP.TO */
    oa_u32 max_bytes;       /* per I/O command */
    oa_u16 cid;
    int vwc;                /* volatile write cache present */
    struct queue admin, io;
    oa_u8 *ident;           /* 4 KiB IDENTIFY buffer */
    oa_u64 ident_bus;
    oa_u64 *prp_list;       /* one page of PRP entries */
    oa_u64 prp_list_bus;
    oa_u8 *bounce;
    oa_u64 bounce_bus;
    char model[41], serial[21], firmware[9];
    unsigned nns;
    struct ns ns[MAX_NAMESPACES];
};

static struct ctrl ctrls[MAX_CONTROLLERS];
static unsigned nctrls;

static oa_u32 rd32(struct ctrl *c, unsigned off)
{
    return *(volatile oa_u32 *)(c->regs + off);
}

static oa_u64 rd64(struct ctrl *c, unsigned off)
{
    return (oa_u64)rd32(c, off) | ((oa_u64)rd32(c, off + 4u) << 32);
}

static void wr32(struct ctrl *c, unsigned off, oa_u32 v)
{
    *(volatile oa_u32 *)(c->regs + off) = v;
}

static void wr64(struct ctrl *c, unsigned off, oa_u64 v)
{
    wr32(c, off, (oa_u32)v);
    wr32(c, off + 4u, (oa_u32)(v >> 32));
}

static int wait_rdy(struct ctrl *c, oa_u32 want)
{
    oa_u64 end = oa_host_time_us() + (oa_u64)c->timeout_ms * 1000u;
    oa_u32 st;

    for (;;) {
        st = rd32(c, REG_CSTS);
        if (st == 0xFFFFFFFFul || (want && (st & CSTS_CFS))) {
            return -1;
        }
        if ((st & CSTS_RDY) == want) {
            return 0;
        }
        if (oa_host_time_us() > end) {
            return -1;
        }
        oa_host_delay_us(100);
    }
}

static int queue_alloc(struct queue *q, oa_u16 qid)
{
    void *sq, *cq;

    sq = oa_host_dma_alloc(PAGE, PAGE, 0, &q->sq_bus);
    cq = oa_host_dma_alloc(PAGE, PAGE, 0, &q->cq_bus);
    if (sq == 0 || cq == 0) {
        return -1;
    }
    memset(sq, 0, PAGE);
    memset(cq, 0, PAGE);
    q->sq = sq;
    q->cq = cq;
    q->qid = qid;
    q->sq_tail = q->cq_head = 0;
    q->phase = 1;
    return 0;
}

/* An I/O completion queue's MSI-X vector. One controller runs one I/O command
 * at a time, so the handler just wakes the submitter, which re-checks the phase
 * bit. MSI-X is an edge message -- nothing to clear here. */
static volatile int nvme_irq_seen;  /* an NVMe interrupt was delivered + handled */
static int nvme_irq_logged;
static void nvme_irq(void *arg)
{
    struct queue *q = arg;
    nvme_irq_seen = 1;
    oa_host_event_signal(q->ev);
}

/* Submit one command (16 dwords, the command id is filled in) and wait for its
 * completion -- blocking on the queue's MSI-X event when it has one, else
 * polling the phase bit. Returns OA_HOST_OK with the NVMe status (0: success)
 * in *status, else OA_HOST_TIMEOUT. */
static int submit(struct ctrl *c, struct queue *q, oa_u32 cmd[16], oa_u32 timeout_ms,
                  oa_u16 *status, oa_u32 *result)
{
    volatile oa_u32 *e;
    oa_u64 end;
    oa_u16 cid = ++c->cid;
    unsigned i;
    oa_u32 dw3;

    cmd[0] = (cmd[0] & 0xFFFFu) | ((oa_u32)cid << 16);
    e = q->sq + 16u * q->sq_tail;
    for (i = 0; i < 16u; i++) {
        e[i] = cmd[i];
    }
    q->sq_tail = (oa_u16)((q->sq_tail + 1u) % QUEUE_ENTRIES);
    oa_host_mb(); /* the entry before the doorbell */
    if (q->ev)
        oa_host_event_reset(q->ev); /* arm before ringing, so no completion is missed */
    wr32(c, REG_DOORBELL + (2u * q->qid) * c->dstrd, q->sq_tail);

    end = oa_host_time_us() + (oa_u64)(timeout_ms ? timeout_ms : 30000u) * 1000u;
    for (;;) {
        e = q->cq + 4u * q->cq_head;
        dw3 = e[3];
        if (((dw3 >> 16) & 1u) == q->phase) {
            if (q->ev && nvme_irq_seen && !nvme_irq_logged) {
                nvme_irq_logged = 1;
                oa_host_log(OA_LOG_INFO, "NVMe: interrupt-driven completion\n");
            }
            break;
        }
        if (oa_host_time_us() > end) {
            return OA_HOST_TIMEOUT;
        }
        if (q->ev)
            oa_host_event_wait(q->ev, 1000u); /* sleep on the IRQ, re-check on wake */
    }
    oa_host_mb(); /* the completion before the data */
    if (result) {
        *result = e[0];
    }
    *status = (oa_u16)(dw3 >> 17);
    q->cq_head = (oa_u16)((q->cq_head + 1u) % QUEUE_ENTRIES);
    if (q->cq_head == 0u) {
        q->phase ^= 1u;
    }
    wr32(c, REG_DOORBELL + (2u * q->qid + 1u) * c->dstrd, q->cq_head);
    if ((dw3 & 0xFFFFu) != cid) {
        return OA_HOST_BUS_ERROR;
    }
    return OA_HOST_OK;
}

static int identify(struct ctrl *c, oa_u32 cns, oa_u32 nsid)
{
    oa_u32 cmd[16];
    oa_u16 status;
    int r;

    memset(cmd, 0, sizeof(cmd));
    cmd[0] = ADM_IDENTIFY;
    cmd[1] = nsid;
    cmd[6] = (oa_u32)c->ident_bus;
    cmd[7] = (oa_u32)(c->ident_bus >> 32);
    cmd[10] = cns;
    r = submit(c, &c->admin, cmd, 5000, &status, 0);
    if (r != OA_HOST_OK) {
        return r;
    }
    return status ? (int)OA_HOST_BUS_ERROR : (int)OA_HOST_OK;
}

/* --- NVMe status to SCSI sense ---------------------------------------- */

static void status_sense(oa_u16 status, struct oa_blk_sense *s)
{
    oa_u8 sct = (oa_u8)((status >> 8) & 7u), sc = (oa_u8)status;

    s->ascq = 0;
    if (sct == 0u) { /* generic */
        switch (sc) {
        case 0x02: /* invalid field */
            s->key = OA_SENSE_ILLEGAL_REQUEST;
            s->asc = OA_ASC_INVALID_FIELD;
            return;
        case 0x80: /* LBA out of range */
        case 0x81: /* capacity exceeded */
            s->key = OA_SENSE_ILLEGAL_REQUEST;
            s->asc = OA_ASC_LBA_OUT_OF_RANGE;
            return;
        case 0x82: /* namespace not ready */
            s->key = OA_SENSE_NOT_READY;
            s->asc = 0x04u;
            return;
        case 0x06: /* internal error */
            s->key = OA_SENSE_HARDWARE_ERROR;
            s->asc = 0x44u;
            return;
        }
    } else if (sct == 2u) { /* media and data integrity */
        switch (sc) {
        case 0x80: /* write fault */
            s->key = OA_SENSE_MEDIUM_ERROR;
            s->asc = 0x03u;
            return;
        case 0x81: /* unrecovered read error */
            s->key = OA_SENSE_MEDIUM_ERROR;
            s->asc = 0x11u;
            return;
        case 0x82: case 0x83: case 0x84: /* end-to-end guard/tag/ref */
            s->key = OA_SENSE_MEDIUM_ERROR;
            s->asc = 0x10u;
            s->ascq = (oa_u8)(sc - 0x81u);
            return;
        case 0x86: /* access denied */
            s->key = OA_SENSE_DATA_PROTECT;
            s->asc = 0x20u;
            s->ascq = 0x02u;
            return;
        }
    } else if (sct == 1u && sc == 0x80u) { /* conflicting attributes */
        s->key = OA_SENSE_ILLEGAL_REQUEST;
        s->asc = OA_ASC_INVALID_FIELD;
        return;
    }
    s->key = OA_SENSE_ABORTED_COMMAND;
    s->asc = 0;
}

/* --- block I/O for the emulation -------------------------------------- */

/* PRP1/PRP2 for `len` bytes at `buf` (dword aligned). */
static void build_prps(struct ctrl *c, const oa_u8 *buf, oa_u32 len, oa_u32 cmd[16])
{
    oa_u64 first = oa_host_virt_to_bus(buf), prp2 = 0;
    oa_u32 off = (oa_u32)(first & (PAGE - 1u));
    oa_u32 pages = (off + len + PAGE - 1u) / PAGE, i;
    const oa_u8 *page = buf - off;

    if (pages == 2u) {
        prp2 = oa_host_virt_to_bus(page + PAGE);
    } else if (pages > 2u) {
        for (i = 1; i < pages; i++) {
            c->prp_list[i - 1u] = oa_host_virt_to_bus(page + (oa_u32)i * PAGE);
        }
        prp2 = c->prp_list_bus;
    }
    cmd[6] = (oa_u32)first;
    cmd[7] = (oa_u32)(first >> 32);
    cmd[8] = (oa_u32)prp2;
    cmd[9] = (oa_u32)(prp2 >> 32);
}

static int ns_rw(void *ctx, oa_u64 lba, oa_u32 count, int write, void *buf, oa_u32 timeout_ms,
                 struct oa_blk_sense *s)
{
    struct ns *n = ctx;
    struct ctrl *c = n->c;
    oa_u8 *p = buf, *data;
    oa_u32 bs = n->emu.block_size, max, now, bytes, cmd[16];
    oa_u16 status;
    int bounce = (oa_host_virt_to_bus(buf) & 3u) != 0u, r;

    max = (bounce ? BOUNCE_SIZE : c->max_bytes) / bs;
    if (max == 0u) {
        max = 1;
    }
    while (count > 0u) {
        now = count < max ? count : max;
        if (now > 65536ul) {
            now = 65536ul;
        }
        bytes = now * bs;
        data = bounce ? c->bounce : p;
        if (bounce && write) {
            memcpy(c->bounce, p, bytes);
        }
        memset(cmd, 0, sizeof(cmd));
        cmd[0] = write ? NVM_WRITE : NVM_READ;
        cmd[1] = n->nsid;
        build_prps(c, data, bytes, cmd);
        cmd[10] = (oa_u32)lba;
        cmd[11] = (oa_u32)(lba >> 32);
        cmd[12] = now - 1u;
        r = submit(c, &c->io, cmd, timeout_ms, &status, 0);
        if (r != OA_HOST_OK) {
            return r;
        }
        if (status != 0u) {
            status_sense(status, s);
            return OA_HOST_OK;
        }
        if (bounce && !write) {
            memcpy(p, c->bounce, bytes);
        }
        p += bytes;
        lba += now;
        count -= now;
    }
    return OA_HOST_OK;
}

static int ns_flush(void *ctx, struct oa_blk_sense *s)
{
    struct ns *n = ctx;
    oa_u32 cmd[16];
    oa_u16 status;
    int r;

    memset(cmd, 0, sizeof(cmd));
    cmd[0] = NVM_FLUSH;
    cmd[1] = n->nsid;
    r = submit(n->c, &n->c->io, cmd, 60000ul, &status, 0);
    if (r == OA_HOST_OK && status != 0u) {
        status_sense(status, s);
    }
    return r;
}

/* --- the adapter ------------------------------------------------------ */

static void nvme_execute(struct oa_hba *hba, struct oa_srb *srb)
{
    struct ctrl *c = (struct ctrl *)hba;

    if (srb->target >= c->nns) {
        srb->host_status = OA_HOST_NO_DEVICE;
        return;
    }
    oa_blockemu_execute(&c->ns[srb->target].emu, srb);
}

static const struct oa_hba_ops nvme_ops = { nvme_execute, 0, 0 };

/* Space-padded identify string to a C string. */
static void ident_string(char *out, const oa_u8 *in, unsigned size)
{
    oa_scsi_identity(out, in, (oa_u16)size);
}

static void hex(char *out, oa_u32 v, unsigned digits)
{
    while (digits--) {
        out[digits] = "0123456789abcdef"[v & 15u];
        v >>= 4;
    }
}

static int create_io_queues(struct ctrl *c)
{
    oa_u32 cmd[16];
    oa_u16 status;

    if (queue_alloc(&c->io, 1) < 0) {
        return -1;
    }
    memset(cmd, 0, sizeof(cmd));
    cmd[0] = ADM_CREATE_CQ;
    cmd[6] = (oa_u32)c->io.cq_bus;
    cmd[7] = (oa_u32)(c->io.cq_bus >> 32);
    cmd[10] = ((QUEUE_ENTRIES - 1u) << 16) | 1u; /* size, qid 1 */
    /* physically contiguous; when the I/O queue has an MSI-X event, enable
     * interrupts (IEN) on vector 0 (the table entry oa_host_pci_msix bound) */
    cmd[11] = c->io.ev ? 0x3u : 0x1u;
    if (submit(c, &c->admin, cmd, 5000, &status, 0) != OA_HOST_OK || status) {
        return -1;
    }
    memset(cmd, 0, sizeof(cmd));
    cmd[0] = ADM_CREATE_SQ;
    cmd[6] = (oa_u32)c->io.sq_bus;
    cmd[7] = (oa_u32)(c->io.sq_bus >> 32);
    cmd[10] = ((QUEUE_ENTRIES - 1u) << 16) | 1u;
    cmd[11] = (1ul << 16) | 1u; /* CQ 1, physically contiguous */
    if (submit(c, &c->admin, cmd, 5000, &status, 0) != OA_HOST_OK || status) {
        return -1;
    }
    return 0;
}

static void add_namespace(struct ctrl *c, oa_u32 nsid)
{
    const oa_u8 *id = c->ident;
    struct ns *n;
    oa_u64 nsze;
    oa_u32 lbaf;
    unsigned lbads;

    if (c->nns == MAX_NAMESPACES || identify(c, 0, nsid) != OA_HOST_OK) {
        return;
    }
    nsze = oa_get_le64(id);
    lbaf = oa_get_le32(id + 128u + 4u * (id[26] & 0x0Fu));
    lbads = (lbaf >> 16) & 0xFFu;
    if (nsze == 0u || lbads < 9u || lbads > 16u) {
        return; /* inactive, or a block size we do not offer */
    }
    n = &c->ns[c->nns++];
    memset(n, 0, sizeof(*n));
    n->c = c;
    n->nsid = nsid;
    n->emu.rw = ns_rw;
    n->emu.flush = c->vwc ? ns_flush : 0;
    n->emu.ctx = n;
    n->emu.blocks = nsze;
    n->emu.block_size = 1ul << lbads;
    memcpy(n->emu.vendor, "NVMe", 5);
    memcpy(n->emu.model, c->model, sizeof(c->model));
    memcpy(n->emu.serial, c->serial, sizeof(c->serial));
    memcpy(n->emu.firmware, c->firmware, sizeof(c->firmware));
}

int nvmeaspi_probe(oa_u32 pci)
{
    struct ctrl *c;
    oa_u32 bar_lo, bar_hi = 0, cmd, nn, i, mpsmin;
    oa_u64 bar, cap;
    oa_u8 mdts;

    if (nctrls == MAX_CONTROLLERS ||
        (oa_host_pci_read32(pci, 0x08) >> 8) != NVMEASPI_PCI_CLASS_NVME) {
        return -1;
    }
    bar_lo = oa_host_pci_read32(pci, 0x10);
    if ((bar_lo & 0x6u) == 0x4u) {
        bar_hi = oa_host_pci_read32(pci, 0x14);
    }
    bar = ((oa_u64)bar_hi << 32) | (bar_lo & 0xFFFFF000ul);
    if (bar == 0u || (bar_lo & 1u)) {
        return -1;
    }
    cmd = oa_host_pci_read32(pci, 0x04);
    oa_host_pci_write32(pci, 0x04, (cmd & 0xFFFFu) | 0x0006u); /* memory, bus master */

    c = &ctrls[nctrls];
    memset(c, 0, sizeof(*c));
    c->regs = oa_host_map_mmio(bar, 0x2000);
    if (c->regs == 0) {
        return -1;
    }
    cap = rd64(c, REG_CAP);
    c->dstrd = 4u << ((cap >> 32) & 0xFu);
    c->timeout_ms = (oa_u32)((cap >> 24) & 0xFFu) * 500u;
    if (c->timeout_ms == 0u) {
        c->timeout_ms = 500;
    }
    mpsmin = (oa_u32)((cap >> 48) & 0xFu);
    if (mpsmin != 0u || !((cap >> 37) & 1u)) {
        oa_host_log(OA_LOG_WARN, "NVMe: 4 KiB pages or NVM command set not supported\n");
        return -1;
    }

    /* disable, admin queues, enable */
    if (rd32(c, REG_CC) & CC_EN) {
        wr32(c, REG_CC, rd32(c, REG_CC) & ~CC_EN);
    }
    if (wait_rdy(c, 0) < 0 || queue_alloc(&c->admin, 0) < 0) {
        return -1;
    }
    c->ident = oa_host_dma_alloc(PAGE, PAGE, 0, &c->ident_bus);
    c->prp_list = oa_host_dma_alloc(PAGE, PAGE, 0, &c->prp_list_bus);
    c->bounce = oa_host_dma_alloc(BOUNCE_SIZE, PAGE, 0, &c->bounce_bus);
    if (c->ident == 0 || c->prp_list == 0 || c->bounce == 0) {
        oa_host_log(OA_LOG_ERROR, "NVMe: out of DMA memory\n");
        return -1;
    }
    wr32(c, REG_AQA, ((QUEUE_ENTRIES - 1u) << 16) | (QUEUE_ENTRIES - 1u));
    wr64(c, REG_ASQ, c->admin.sq_bus);
    wr64(c, REG_ACQ, c->admin.cq_bus);
    wr32(c, REG_CC, CC_IOSQES | CC_IOCQES | CC_EN); /* NVM set, 4 KiB pages, round robin */
    if (wait_rdy(c, CSTS_RDY) < 0) {
        oa_host_log(OA_LOG_WARN, "NVMe: controller does not become ready\n");
        return -1;
    }

    /* the controller */
    if (identify(c, 1, 0) != OA_HOST_OK) {
        return -1;
    }
    ident_string(c->serial, c->ident + 4, 20);
    ident_string(c->model, c->ident + 24, 40);
    ident_string(c->firmware, c->ident + 64, 8);
    mdts = c->ident[77];
    c->vwc = c->ident[525] & 1u;
    nn = oa_get_le32(c->ident + 516);
    /* per command: MDTS (in pages), the PRP list page and 2 MiB at most */
    c->max_bytes = PRP_ENTRIES * PAGE;
    if (mdts != 0u && mdts < 9u && (PAGE << mdts) < c->max_bytes) {
        c->max_bytes = PAGE << mdts;
    }
    /* Interrupt-driven I/O completions: bind one MSI-X vector (table entry 0)
     * to the I/O completion queue, which is then created with IEN on vector 0.
     * The admin queue stays polled -- it is used only during this init. On any
     * failure the I/O queue keeps polling (ev stays NULL), so a controller
     * without working MSI-X never regresses. */
    c->io.ev = oa_host_event_create();
    if (c->io.ev && oa_host_pci_msix(pci, nvme_irq, &c->io) < 0) {
        oa_host_event_destroy(c->io.ev);
        c->io.ev = 0;
    }
    if (create_io_queues(c) < 0) {
        oa_host_log(OA_LOG_WARN, "NVMe: cannot create the I/O queues\n");
        return -1;
    }
    for (i = 1; i <= nn && i <= 1024u && c->nns < MAX_NAMESPACES; i++) {
        add_namespace(c, i);
    }

    memcpy(c->name, "NVMe 0000:00:00.0", 18);
    hex(c->name + 5, pci >> 16, 4);
    hex(c->name + 10, (pci >> 8) & 0xFFu, 2);
    hex(c->name + 13, (pci >> 3) & 0x1Fu, 2);
    c->name[16] = (char)('0' + (pci & 7u));
    c->hba.name = c->name;
    c->hba.ops = &nvme_ops;
    c->hba.priv = c;
    c->hba.max_target = (oa_u8)(c->nns ? c->nns - 1u : 0u);
    c->hba.max_lun = 0;
    nctrls++;
    if (c->nns) {
        oa_hba_register(&c->hba);
    }
    return (int)c->nns;
}
