EXTRA_CFLAGS += -DPDU_USE_KERNEL=1

#if KERNELDIR is not defined, set it to the modules directory
KERNELDIR ?= /lib/modules/$(shell uname -r)/build
RD ?= $(shell pwd)

include ${RD}/../../pdu/${PDU_ARCH}/kernel/Makefile.opts
include ${RD}/Makefile.opts

EXTRA_CFLAGS += -I${RD}/../include -I${RD}/../../pdu/${PDU_ARCH}/include

ifdef MAKEAVECTOR
EXTRA_CFLAGS += -DMAKEAVECTOR
endif

ifdef PCI_INDIRECT
EXTRA_CFLAGS += -DPCI_INDIRECT
endif

ifdef SPACC_MAX_PARTICLE_SIZE
EXTRA_CFLAGS += -DSPACC_MAX_PARTICLE_SIZE=${SPACC_MAX_PARTICLE_SIZE}
endif

ifdef _SPACC_MAX_MSG_MALLOC_SIZE
EXTRA_CFLAGS += -D_SPACC_MAX_MSG_MALLOC_SIZE=${_SPACC_MAX_MSG_MALLOC_SIZE}
endif

ifdef SPACC_DEFAULT_DDT_LENGTH
EXTRA_CFLAGS += -DSPACC_DEFAULT_DDT_LENGTH=${SPACC_DEFAULT_DDT_LENGTH}
endif

OBJS-SPACC += spacc.o ../spacc/elpspacc.o ../spacc/spacc_open.o ../spacc/spacc_close.o  \
../spacc/spacc_get_context.o ../spacc/spacc_get_rc4_context.o \
../spacc/spacc_packet_dequeue.o ../spacc/spacc_packet_enqueue_ddt.o \
../spacc/spacc_set_context.o ../spacc/spacc_set_operation.o ../spacc/spacc_set_rc4_context.o  \
../spacc/spacc_set_auxinfo.o ../spacc/spacc_virtual_request_rc4.o \
../spacc/spacc_virtual_set_weight.o ../spacc/spacc_error_msg.o \
../spacc/spacc_ctx_manager.o ../spacc/spacc_job_manager.o \
../spacc/spacc_enable_int.o ../spacc/spacc_process_irq.o \
../spacc/spacc_load_skp.o ../spacc/spacc_dump_ctx.o ../spacc/spacc_hsm_semaphore.o \
../spacc/spacc_set_key_exp.o ../spacc/spacc_set_wd_count.o \
../spacc/spacc_autodetect.o ../spacc/spacc_isenabled.o \
../spacc/spacc_compute_xcbc_key.o ../spacc/spacc_process_jb.o

module-spacc=elpspacc
OUTPUT-SPACC = ${module-spacc}.o
${module-spacc}-objs := ${OBJS-SPACC}
obj-m := ${OUTPUT-SPACC}

ifdef MAKEAVECTOR1
OBJS-SPACCDIAG += spaccdiag.o elpparser.o
module-spaccdiag=elpspaccdiag
OUTPUT-SPACCDIAG= ${module-spaccdiag}.o
${module-spaccdiag}-objs := ${OBJS-SPACCDIAG}
obj-m += ${OUTPUT-SPACCDIAG}
endif

OBJS-SPACCTRAFFIC += spacctraffic.o
module-spacctraffic=spacctraffic
OUTPUT-SPACCTRAFFIC=${module-spacctraffic}.o
${module-spacctraffic]-objs := ${OBJS-SPACCTRAFFIC}
obj-m += ${OUTPUT-SPACCTRAFFIC}

OBJS-SPACCEXAMPLE += example.o
module-spaccexample=spaccexample
OUTPUT-SPACCEXAMPLE=${module-spaccexample}.o
${module-spaccexample}-objs := ${OBJS-SPACCEXAMPLE}
obj-m += ${OUTPUT-SPACCEXAMPLE}

OBJS-SPACCUSR += spacc_dev.o
module-spaccusr=elpspaccusr
OUTPUT-SPACCUSR=${module-spaccusr}.o
${module-spaccusr}-objs := ${OBJS-SPACCUSR}
obj-m += ${OUTPUT-SPACCUSR}

OBJS-SPACCRC4 += spacc_rc4test.o
module-spaccrc4 = spaccrc4test
OUTPUT-SPACCRC4=${module-spaccrc4}.o
${module-spaccrc4}-objs := ${OBJS-SPACCRC4}
obj-m += ${OUTPUT-SPACCRC4}

#obj-m += elpspacccrypt.o
#elpspacccrypt-objs := cryptoapi.o cryptoapi-ahash.o  cryptoapi-aead.o

ccflags-$(CONFIG_SPACC_DEBUG) := -DDEBUG

all: 
	RD=${RD} CFLAGS="${CFLAGS}" $(MAKE) -C $(KERNELDIR) SUBDIRS=$(RD) modules

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions *.symvers Module.* modules.*
