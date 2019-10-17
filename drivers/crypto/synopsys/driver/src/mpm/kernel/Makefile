EXTRA_CFLAGS += -DPDU_USE_KERNEL=1

#if KERNELDIR is not defined, set it to the modules directory
KERNELDIR ?= /lib/modules/$(shell uname -r)/build
RD ?= $(shell pwd)

include ${RD}/../../pdu/${PDU_ARCH}/kernel/Makefile.opts
include ${RD}/Makefile.opts

ifdef PCI_INDIRECT
EXTRA_CFLAGS += -DPCI_INDIRECT
endif

ifdef MPM_TIMING_X86
EXTRA_CFLAGS += -DMPM_TIMING_X86
endif

ifdef MPM_PERF_MON
EXTRA_CFLAGS += -DMPM_PERF_MON
endif

EXTRA_CFLAGS += -I${RD}/../include -I${RD}/../../pdu/${PDU_ARCH}/include -I${RD}/../../mpm/include -I${RD}/../../core/include

OBJS-MPM += mpm.o ../mpm/mpm_alloc_key.o ../mpm/mpm_alloc_pdu.o ../mpm/mpm_clear_chains.o \
../mpm/mpm_deinit.o ../mpm/mpm_enqueue_chain.o ../mpm/mpm_free_key.o ../mpm/mpm_free_spacc_ctx.o \
../mpm/mpm_init.o ../mpm/mpm_insert_pdu.o ../mpm/mpm_req_spacc_ctx.o ../mpm/mpm_set_key.o \
../mpm/mpm_get_status_word.o ../mpm/mpm_cleanup.o ../mpm/mpm_stats.o ../mpm/mpm_process_int.o


module-mpm=elpmpm
OUTPUT-MPM = ${module-mpm}.o
${module-mpm}-objs := ${OBJS-MPM}
obj-m := ${OUTPUT-MPM}

EXTRA_CFLAGS += -I${RD}/../../core/kernel/
OBJS-MPMDIAG += mpmdiag.o ../../core/kernel/elpparser.o
module-mpmdiag=elpmpmdiag
OUTPUT-MPMDIAG= ${module-mpmdiag}.o
${module-mpmdiag}-objs := ${OBJS-MPMDIAG}
obj-m += ${OUTPUT-MPMDIAG}

#traffic module not yet ready for prime time
ifndef NOPE
OBJS-MPMTRAFFIC += mpmtraffic.o
module-mpmtraffic=elpmpmtraffic
OUTPUT-MPMTRAFFIC= ${module-mpmtraffic}.o
${module-mpmtraffic}-objs := ${OBJS-MPMTRAFFIC}
obj-m += ${OUTPUT-MPMTRAFFIC}
endif

all: 
	RD=${RD} CFLAGS="${CFLAGS}" $(MAKE) -C $(KERNELDIR) SUBDIRS=$(RD) modules

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions *.symvers Module.* modules.*

