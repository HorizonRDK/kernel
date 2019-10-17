EXTRA_CFLAGS += -DPDU_USE_KERNEL=1

ifdef USE_ZYNQ_OCM
EXTRA_CFLAGS += -DUSE_ZYNQ_OCM
endif

#if KERNELDIR is not defined, set it to the modules directory
KERNELDIR ?= /lib/modules/$(shell uname -r)/build
RD ?= $(shell pwd)

include ${RD}/../../pdu/${PDU_ARCH}/kernel/Makefile.opts
include ${RD}/Makefile.opts

EXTRA_CFLAGS += -I${RD}/../include -I${RD}/../../pdu/${PDU_ARCH}/include -I${RD}/../../core/include -I${RD}/../../xfrm/ -DXFRM_USE_EA

ifdef PCI_INDIRECT
EXTRA_CFLAGS += -DPCI_INDIRECT
endif

OBJS-EA += ea.o ../ea/ea_build_sa.o ../ea/ea_deinit.o ../ea/ea_go.o ../ea/ea_init.o ../ea/ea_open.o ../ea/ea_close.o ../ea/ea_done.o ../ea/ea_pop_packets.o ../ea/ea_set_mode.o 

module-ea=elpea
OUTPUT-EA = ${module-ea}.o
${module-ea}-objs := ${OBJS-EA}
obj-m := ${OUTPUT-EA}

OBJS-EADIAG += eadiag.o
module-eadiag=elpeadiag
OUTPUT-EADIAG = ${module-eadiag}.o
${module-eadiag}-objs := ${OBJS-EADIAG}
obj-m += ${OUTPUT-EADIAG}

OBJS-EAOFF += eaoff.o
module-eaoff=elpeaoff
OUTPUT-EAOFF = ${module-eaoff}.o
${module-eaoff}-objs := ${OBJS-EAOFF}
obj-m += ${OUTPUT-EAOFF}

obj-m += elpxfrm.o
elpxfrm-objs := ../../xfrm/xfrm.o

all: 
	RD=${RD} CFLAGS="${CFLAGS}" $(MAKE) -C $(KERNELDIR) SUBDIRS=$(RD) modules

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions *.symvers Module.* modules.*
