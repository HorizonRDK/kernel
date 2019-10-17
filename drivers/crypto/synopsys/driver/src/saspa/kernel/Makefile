EXTRA_CFLAGS += -DPDU_USE_KERNEL=1

#if KERNELDIR is not defined, set it to the modules directory
KERNELDIR ?= /lib/modules/$(shell uname -r)/build
RD ?= $(shell pwd)

include ${RD}/../../pdu/${PDU_ARCH}/kernel/Makefile.opts
include ${RD}/Makefile.opts

EXTRA_CFLAGS += -I${RD}/../include -I${RD}/../../pdu/${PDU_ARCH}/include -I${RD}/../../core/include -I${RD}/../../core/kernel/

ifdef MAKEAVECTOR
EXTRA_CFLAGS += -DMAKEAVECTOR
endif

ifdef PCI_INDIRECT
EXTRA_CFLAGS += -DPCI_INDIRECT
endif

OBJS-SASPA += saspa.o aes.o des.o hash.o ../saspa/elpsaspa.o ../saspa/elpaes.o ../saspa/elpdes.o ../saspa/elphash.o
OBJS-SASPADIAG += saspadiag.o ../../core/kernel/elpparser.o

module-saspa=elpsaspa
OUTPUT-SASPA = ${module-saspa}.o
${module-saspa}-objs := ${OBJS-SASPA}

module-saspadiag = elpsaspadiag
OUTPUT-SASPADIAG = ${module-saspadiag}.o
${module-saspadiag}-objs := ${OBJS-SASPADIAG}

obj-m := ${OUTPUT-SASPA} ${OUTPUT-SASPADIAG}

all: 
	RD=${RD} CFLAGS="${CFLAGS}" $(MAKE) -C $(KERNELDIR) SUBDIRS=$(RD) modules

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions *.symvers Module.* modules.*
