Linux Kernel Driver package for SPAcc/SPAcc-PDU with support for
PKA, TRNG, RE, EA, KEP, CLP-30/36, and MPM modules

------------------------------------------------------------------------
This Synopsys software and associated documentation (hereinafter the
"Software") is an unsupported proprietary work of Synopsys, Inc. unless
otherwise expressly agreed to in writing between Synopsys and you. The
Software IS NOT an item of Licensed Software or a Licensed Product under
any End User Software License Agreement or Agreement for Licensed Products
with Synopsys or any supplement thereto. Synopsys is a registered trademark
of Synopsys, Inc. Other names included in the SOFTWARE may be the
trademarks of their respective owners.

The contents of this file are dual-licensed; you may select either version
2 of the GNU General Public License ("GPL") or the BSD-3-Clause license
("BSD-3-Clause"). The GPL is included in the COPYING file accompanying the
SOFTWARE. The BSD License is copied below.

BSD-3-Clause License:
Copyright (c) 2011-2015 Synopsys, Inc. and/or its affiliates.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions, and the following disclaimer, without
   modification.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. The names of the above-listed copyright holders may not be used to
   endorse or promote products derived from this software without specific
   prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------

This package includes Linux 3.14+ drivers for the Elliptic CLP-30, SPAcc,
SPAcc-PDU, CLP-800, and PKA related devices.  The drivers were tested on
little endian 32-bit hosts and should be portable to both x86 and ARM
platforms.

The package has the following directory structure:

synopsys
|-- doc                       <--- SDK Documentation (EDN-0510 located here)
`-- driver
    |-- bin                   <--- Destination for compiled libraries, applications, and kernel modules
    |-- patches
    `-- src
        |-- clp30             <--- CLP-30/36 SDK
        |   |-- clp30
        |   |-- include
        |   `-- kernel        <--- Linux Kernel modules and integration
        |-- core              <--- SPAcc SDK
        |   |-- include
        |   |-- kernel        <--- Linux Kernel modules and integration
        |   |-- spacc         <--- Portable SPAcc control SDK
        |   `-- user          <--- Userspace library for Linux userspace driver
        |-- ea                <--- SPAcc-EA (SPP-230/330) IPsec SDK
        |   |-- ea            <--- Portable EA control SDK
        |   |-- include
        |   `-- kernel        <--- Linux Kernel modules and integration
        |-- include
        |   `-- crypto
        |-- mpm               <--- SPAcc-MPM SDK
        |   |-- include
        |   |-- kernel        <--- Linux Kernel modules
        |   `-- mpm           <--- Portable MPM control SDK
        |-- pdu               <--- Platform Device SDK
        |   |-- bare          <--- Bare metal platform template
        |   |   |-- include
        |   |   |-- kernel
        |   |   `-- pdu
        |   `-- linux         <--- Linux platform template
        |       |-- include
        |       |-- kernel
        |       `-- pdu
        |-- pka               <--- PKA SDK
        |   |-- include
        |   |-- kernel        <--- Linux Kernel modules
        |   |-- pka           <--- Portable PKA control SDK
        |   `-- user          <--- Userspace Library
        |       |-- diag
        |       `-- engine
        |           |-- m4
        |           `-- src
        |-- re                <--- SPAcc-RE Record Engine SDK
        |   |-- include
        |   |-- kep           <--- Portable KEP control SDK
        |   |-- kernel        <--- Linux Kernel modules for RE and KEP
        |   |-- re            <--- Portable RE control SDK
        |   `-- user          <--- Userspace Library
        |-- saspa             <--- SASPA SDK
        |   |-- include
        |   |-- kernel        <--- Linux Kernel Modules
        |   `-- saspa         <--- Portable SASPA control SDK
        |-- test
        |   `-- offload
        `-- trng              <--- CLP-800 (TRNG3) SDK
            |-- include
            |-- kernel        <--- Linux Kernel modules and integration
            `-- trng          <--- Portable TRNG SDK


---- Building Drivers ----

The top level makefile will build all of the drivers and place them in
synopsys/driver/bin by default.  Issuing "make" is sufficient to build
all of the drivers.  Minor changes may be required for cross compiling.

The modules names are

elppdu.ko        - The PDU platform_device module
elpspacc.ko      - Provides the SPAcc SDK in Kernel space (requires elppdu)
elpspaccusr.ko   - Provides userspace interface to the SPAcc (req: elpspacc, elppdu)
elpspacccrypt.ko - Provides Linux crypto API hash/cipher drivers.
elpre.ko         - Provides the RE SDK in Kernel space (req: elpspacc, elppdu)
elpreusr.ko      - Provides the /dev/spaccre interface to use the RE from userspace (req: elpre, elpspacc, elppdu)
elpkep.ko        - Provides the KEP SDK in Kernel space (req: elpspacc, elppdu)
elpkepdiag.ko    - Provides the KEP diagnostic interface /proc/kepdiag (req: elpkep, elpspacc, elppdu)
elpkepdev.ko     - Provides the /dev/spacckep interface to use the KEP from userspace (req: elpkep, elpspacc, elppdu)
elpmpm.ko        - Provides the MPM SDK in Kernel space (req: elpspacc, elppdu)
elpmpmdiag.ko    - Provides the MPM diagnostic interface /proc/mpmdiag (req: elpmpm, elpspacc, elppdu)
elpea.ko         - Provides the EA SDK in Kernel space (req: elpspacc, elppdu)
elpeaxfrm.ko     - Provides the EA Linux integration for IPSec (req: elpea, elpspacc, elppdu)
elpclp800.ko     - Provides the CLP-800 (TRNG3) kernel interfaces and /dev/hwrng usage.
elpclp890.ko     - Provides the CLP-890 kernel interfaces and /dev/hwrng usage.

The "diag" modules are not required to be loaded for use of the drivers.  They
are only provided to run vectors which would be provided to customers on a per
order basis.

--- Using Crypto API ---

The Linux crypto API driver (elpspacccrypt.ko) is used to integrate the SPAcc
with other Linux subsystems (e.g., IPsec).  After registering the SPAcc
platform_device with the kernel, simply load the module to hook into the
cipher/hash systems.  There are, however, some limitations to note:

  * Due to some bugs/limitations with DMA mapping and crypto API in current
    versions of Linux, SG debugging must be disabled in the kernel config
    (CONFIG_DEBUG_SG=n).

  * For modes requiring separate encrypt/decrypt contexts (e.g., AES-ECB or
    AES-CBC), encrypt and decrypt must be done using separate handles.

--- Using PKA Linux driver ---

The PKA firmware requires processing before it can be loaded by the Linux
driver.  Firmware is generated from the hardware configuration as one or
more Verilog $readmemh memory images (.hex files), together with an entry
point list (.ent).  The pka-mkimage tool takes these inputs and and produces
a combined binary output file (.elf).  This file is to be installed into
/lib/firmware to be loaded by the driver.

The tool is located in src/pka/user/mkimage.  To build:

  % make -C src/pka/user
  % make -C src/pka/user/mkimage

You may need to install development headers for the libelf / elfutils package
in ordre to build pka-mkimage; the package may be called elfutils-devel or
libelf-dev or similar.

Usage depends on the target memory configuration:

  - Prepare a RAM-only image:

      pka-mkimage -o elppka.elf -e firmware.ent ram.hex

  - Prepare a ROM-only image:

      pka-mkimage --rom-only -o elppka.elf -e firmware.ent rom.hex

  - Prepare a split RAM/ROM image:

      pka-mkimage --rom-base=2048 -o elppka.elf -e firmware.ent ram.hex rom.hex

For more usage details, see the man page:

  man src/pka/user/mkimage/pka-mkimage.1

To load the firmware, copy the .elf file into /lib/firmware.  After the driver
is installed, you can (re-)load the firmware by writing its filename to

  /sys/bus/platform/devices/pka*/load_firmware

e.g.,

  echo elppka.elf > /sys/bus/platform/devices/pka*/load_firmware

If successful, information about the firmware will be printed to the kernel
log.
