# ------------------------------------------------------------------------
# This Synopsys software and associated documentation (hereinafter the
# "Software") is an unsupported proprietary work of Synopsys, Inc. unless
# otherwise expressly agreed to in writing between Synopsys and you. The
# Software IS NOT an item of Licensed Software or a Licensed Product under
# any End User Software License Agreement or Agreement for Licensed Products
# with Synopsys or any supplement thereto. Synopsys is a registered trademark
# of Synopsys, Inc. Other names included in the SOFTWARE may be the
# trademarks of their respective owners.
#
# The contents of this file are dual-licensed; you may select either version
# 2 of the GNU General Public License ("GPL") or the BSD-3-Clause license
# ("BSD-3-Clause"). The GPL is included in the COPYING file accompanying the
# SOFTWARE. The BSD License is copied below.
#
# BSD-3-Clause License:
# Copyright (c) 2011-2017 Synopsys, Inc. and/or its affiliates.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions, and the following disclaimer, without
#    modification.
#
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
#
# 3. The names of the above-listed copyright holders may not be used to
#    endorse or promote products derived from this software without specific
#    prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# ------------------------------------------------------------------------

#if KERNELDIR is not defined, set it to the modules directory
KERNELDIR ?= /lib/modules/$(shell uname -r)/buil
ifndef RD
RD       := $(shell pwd)
endif

include ${RD}/Makefile.opts

ifdef PDU_MAX_DDT
EXTRA_CFLAGS += -DPDU_MAX_DDT=${PDU_MAX_DDT}
endif

ifdef PDU_DUAL_MPM
EXTRA_CFLAGS += -DPDU_DUAL_MPM
endif

ifdef PDU_BASE_ADDR
EXTRA_CFLAGS += -DPDU_BASE_ADDR=${PDU_BASE_ADDR}
endif

ifdef PDU_BASE_IRQ
EXTRA_CFLAGS += -DPDU_BASE_IRQ=${PDU_BASE_IRQ}
endif

ifdef PDU_ZYNQ_RESET
EXTRA_CFLAGS += -DPDU_ZYNQ_RESET
endif

EXTRA_CFLAGS += -I${RD}/../include -I${RD}/../../../../common_sdk/include/
EXTRA_CFLAGS += -I${RD}/../../../trng/include
EXTRA_CFLAGS += -I${RD}/../../../saspa/include

#build up the object list depending on options
OBJS-PDU += pdu.o ../../common/pdu/pdu_dev32.o \
	../../common/pdu/pdu_get_version.o

ifdef PDU_SINGLE_CORE
EXTRA_CFLAGS += -DPDU_SINGLE_CORE
endif
ifdef PDU_SINGLE_CLP800
EXTRA_CFLAGS += -DPDU_SINGLE_CLP800
endif
ifdef PDU_SINGLE_CLP850
EXTRA_CFLAGS += -DPDU_SINGLE_CLP850
endif
ifdef PDU_SINGLE_CLP890
EXTRA_CFLAGS += -DPDU_SINGLE_CLP890
endif
ifdef PDU_SINGLE_EAPE
EXTRA_CFLAGS += -DPDU_SINGLE_EAPE
endif
ifdef PDU_SINGLE_CLP30
EXTRA_CFLAGS += -DPDU_SINGLE_CLP30
endif
ifdef PDU_SINGLE_CLP36
EXTRA_CFLAGS += -DPDU_SINGLE_CLP36
endif
ifdef CLP36_ENABLED
EXTRA_CFLAGS += -DCLP36_ENABLED=1
endif
ifdef PDU_SINGLE_PKA
EXTRA_CFLAGS += -DPDU_SINGLE_PKA
endif
ifdef PCI_INDIRECT
EXTRA_CFLAGS += -DPCI_INDIRECT
endif

# creates the actual kernel module name based on input or uses a default name
module-pdu=elppdu
module-pci=elppci
module-mem=elpmem
module-standalone=elppci_standalone

ifdef PDU_USE_PCI
OUTPUT-PCI = ${module-pci}.o ${module-standalone}.o
endif

ifndef PDU_USE_PCI
OUTPUT-MEM = ${module-mem}.o
endif

OUTPUT-PDU = ${module-pdu}.o 
${module-pdu}-objs := $(OBJS-PDU)
ifndef PDU_USE_SASPA
${module-pci}-objs := spacc_pci.o
${module-mem}-objs := spacc_mem.o
else
${module-pci}-objs := saspa_pci.o
${module-mem}-objs := saspa_mem.o
endif

obj-m := ${OUTPUT-PDU} ${OUTPUT-PCI} ${OUTPUT-MEM}

${module-standalone}-objs := standalone_pci.o

all: 
	RD=${RD} CFLAGS="${CFLAGS}" $(MAKE) -C $(KERNELDIR) SUBDIRS=$(RD) modules

clean:
	rm -rf *.o *.ko *.mod.? Module.* modules.* .*.cmd .tmp*
