# NVMEASPI - NVMe for the OpenASPI native layer

NVMEASPI is the NVMe transport of [OpenASPI](https://github.com/geosone/OpenAspi)'s native ASPI layer, written in C for 32- and 64-bit systems that drive the controller themselves, such as the GDOS kernel of [GonWare](https://gitea.disconnected-by-peer.at/GonSoft/GonWare). It sits next to [AHCIASPI](https://github.com/geosone/ahciaspi) (AHCI and IDE) and [EUSBASPI](https://github.com/geosone/eusbaspi) (USB) and, like them, offers its drives as SCSI devices through the ASPI interface.

**Status:** first version, C only (no DOS manager yet). Tested in QEMU (`-device nvme`) on x86-64, i686 and AArch64, through UEFI and, on x86, through OpenDuet on a BIOS.

## What it does

- Takes over an NVMe controller found by PCI class 010802h: disables it, sets up the admin queue pair, enables it (NVMe 1.4 section 7.6.1), identifies the controller and its namespaces and creates one I/O queue pair without interrupts. Commands are polled.
- Offers each active namespace as one SCSI direct-access target (LUN 0). The SCSI side - INQUIRY with the serial number page, READ CAPACITY, MODE SENSE, REQUEST SENSE, READ/WRITE (6/10/12/16), SYNCHRONIZE CACHE - is OpenASPI's shared block device emulation, the same that answers for ATA disks through SAT; NVMEASPI only reads, writes and flushes and maps NVMe status codes to SCSI sense data, following the NVM Express SCSI translation reference.
- Describes data with PRPs page by page and limits a command to MDTS and one PRP list page (2 MiB); buffers that are not dword aligned go through a 64 KiB bounce buffer.

Namespaces with a block size from 512 bytes to 64 KiB are offered; protection information and metadata are not used.

## Building

CMake, as `NVMEASPI::native`, on top of the `OpenASPI::native` target of a parent project or of an OpenASPI checkout next to this one (`OPENASPI_SOURCE_DIR`):

```sh
cmake -S . -B build -G Ninja && ninja -C build
```

The code is C90. The embedding system provides the host services of `openaspi/native/host.h` (time, DMA memory, MMIO mapping, PCI configuration, the DMA barrier `oa_host_mb()`) and calls `nvmeaspi_probe()` for each controller of PCI class 010802h. It needs a flat address space for the registers and queues, so there is no 16-bit DOS build.

## License

GPLv2 or later, see [LICENSE](LICENSE).
