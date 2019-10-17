# ------------------------------------------------------------------------
# This Synopsys software and associated documentation (hereinafter the
# "Software") is an unsupported proprietary work of Synopsys, Inc. unless
# oMherwise expressly agreed to in writing between Synopsys and you. The
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
# Copyright (c) 2012-2015 Synopsys, Inc. and/or its affiliates.
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

EXTRA_CFLAGS += -DPDU_USE_KERNEL=1

#if KERNELDIR is not defined, set it to the modules directory
KERNELDIR ?= /lib/modules/$(shell uname -r)/build
RD ?= $(shell pwd)

include ${RD}/../../pdu/${PDU_ARCH}/kernel/Makefile.opts
include ${RD}/Makefile.opts

EXTRA_CFLAGS += -I${RD}/../include -I${RD}/../../pdu/${PDU_ARCH}/include -I${RD}/../../../../common_sdk/include -I${RD}/../../core/include
#export KBUILD_EXTRA_SYMBOLS = ${RD}/../../pdu/kernel/Module.symvers

ifdef PCI_INDIRECT
EXTRA_CFLAGS += -DPCI_INDIRECT
endif

OBJS-PKA += pka.o class.o ../pka/elppka.o ../pka/firmware.o

module-pka=elppka
OUTPUT-PKA = ${module-pka}.o
${module-pka}-objs := ${OBJS-PKA}
obj-m := ${OUTPUT-PKA}
ccflags-$(CONFIG_PKA_DEBUG) := -DDEBUG

all: 
	RD=${RD} CFLAGS="${CFLAGS}" $(MAKE) -C $(KERNELDIR) SUBDIRS=$(RD) modules

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions *.symvers Module.* modules.*

