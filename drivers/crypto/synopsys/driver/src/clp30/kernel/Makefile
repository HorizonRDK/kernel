EXTRA_CFLAGS += -DPDU_USE_KERNEL=1

#if KERNELDIR is not defined, set it to the modules directory
KERNELDIR ?= /lib/modules/$(shell uname -r)/build
RD ?= $(shell pwd)

include ${RD}/../../pdu/${PDU_ARCH}/kernel/Makefile.opts
include ${RD}/Makefile.opts

EXTRA_CFLAGS += -I${RD}/../include -I${RD}/../../pdu/${PDU_ARCH}/include -I${RD}/../../core/include -I${RD}/../../xfrm/ -DXFRM_USE_CLP30

ifdef CLP36_ENABLED
EXTRA_CFLAGS += -DCLP36_ENABLED
endif

ifdef PCI_INDIRECT
EXTRA_CFLAGS += -DPCI_INDIRECT
endif

OBJS-clp30 += clp30.o ../clp30/clp30_build_sa.o ../clp30/clp30_deinit.o ../clp30/clp30_go.o \
../clp30/clp30_init.o ../clp30/clp30_open.o ../clp30/clp30_close.o ../clp30/clp30_done.o ../clp30/clp30_pop_packets.o

module-clp30=elpclp30
OUTPUT-clp30 = ${module-clp30}.o
${module-clp30}-objs := ${OBJS-clp30}
obj-m := ${OUTPUT-clp30}

OBJS-CLP30DIAG += clp30diag.o
module-clp30diag=elpclp30diag
OUTPUT-CLP30DIAG = ${module-clp30diag}.o
${module-clp30diag}-objs := ${OBJS-CLP30DIAG}
obj-m += ${OUTPUT-CLP30DIAG}

#OBJS-CLP30OFF += clp30off.o
#module-clp30off=elpclp30off
#OUTPUT-CLP30OFF = ${module-clp30off}.o
#${module-clp30off}-objs := ${OBJS-CLP30OFF}
#obj-m += ${OUTPUT-CLP30OFF}

obj-m += elpxfrm.o
elpxfrm-objs := ../../xfrm/xfrm.o

all: 
	RD=${RD} CFLAGS="${CFLAGS}" $(MAKE) -C $(KERNELDIR) SUBDIRS=$(RD) modules

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions *.symvers Module.* modules.*
