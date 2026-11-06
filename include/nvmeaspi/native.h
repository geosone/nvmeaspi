/* SPDX-License-Identifier: GPL-2.0-or-later */
/* NVMEASPI: NVMe transport for OpenASPI's native layer. Each active
 * namespace of a controller is one SCSI direct-access target (LUN 0),
 * translated by OpenASPI's block device emulation. Polled, one I/O queue;
 * the embedding system owns the controller. */

#ifndef NVMEASPI_NATIVE_H
#define NVMEASPI_NATIVE_H

#include "openaspi/types.h"

/* PCI class of an NVMe controller (mass storage, NVM, NVMe). */
#define NVMEASPI_PCI_CLASS_NVME 0x010802ul

/* Take over the NVMe controller at PCI address `pci` (OA_PCI_ADDR) and
 * register it as one OpenASPI adapter. Returns the number of namespaces
 * offered, or -1 if the controller is not usable. */
int nvmeaspi_probe(oa_u32 pci);

#endif
