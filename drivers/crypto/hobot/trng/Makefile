EXTRA_CFLAGS += -DPDU_USE_KERNEL=1

#if KERNELDIR is not defined, set it to the modules directory
KERNELDIR ?= /lib/modules/$(shell uname -r)/build
RD ?= $(shell pwd)

include ${RD}/../../pdu/${PDU_ARCH}/kernel/Makefile.opts
include ${RD}/Makefile.opts

ifdef PCI_INDIRECT
EXTRA_CFLAGS += -DPCI_INDIRECT=1
endif

EXTRA_CFLAGS += -Wall -I${RD}/../include -I${RD}/../../pdu/${PDU_ARCH}/include -I${RD}/../../core/include -I${RD}/../../include

obj-m += elpclp800.o
elpclp800-objs := clp800.o clp800_debug.o ../trng/clp800.o

obj-m += elpclp850.o
elpclp850-objs := clp850.o ../trng/clp850.o

obj-m += elpclp890.o
elpclp890-objs := clp890.o ../trng/clp890.o ../trng/clp890_private.o

all: 
	RD=${RD} CFLAGS="${CFLAGS}" $(MAKE) -C $(KERNELDIR) SUBDIRS=$(RD) modules

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions *.symvers Module.* modules.*

