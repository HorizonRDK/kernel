EXTRA_CFLAGS += -DPDU_USE_KERNEL=1

ifdef RE_DEFAULT_DDT_LENGTH
EXTRA_CFLAGS += -DRE_DEFAULT_DDT_LENGTH=${RE_DEFAULT_DDT_LENGTH}
endif

#if KERNELDIR is not defined, set it to the modules directory
KERNELDIR ?= /lib/modules/$(shell uname -r)/build
RD ?= $(shell pwd)

include ${RD}/../../pdu/${PDU_ARCH}/kernel/Makefile.opts
include ${RD}/Makefile.opts

ifdef PCI_INDIRECT
EXTRA_CFLAGS += -DPCI_INDIRECT
endif

EXTRA_CFLAGS += -I${RD}/../include -I${RD}/../../pdu/${PDU_ARCH}/include -I${RD}/../../core/include

OBJS-RE += re.o \
../re/elpre.o ../re/re_error_msg.o ../re/re_fini.o ../re/re_finish_operation.o \
../re/re_get_context.o ../re/re_init_context.o ../re/re_packet_dequeue.o ../re/re_print_diag.o \
../re/re_release_context.o ../re/re_reset_sa.o ../re/re_retrieve_sa.o ../re/re_set_next_read.o \
../re/re_set_next_read_iv.o ../re/re_set_next_read_key.o ../re/re_set_next_read_mackey.o \
../re/re_set_next_read_params.o ../re/re_set_next_read_sequence_number.o ../re/re_set_next_write.o \
../re/re_set_next_write_iv.o ../re/re_set_next_write_key.o ../re/re_set_next_write_mackey.o \
../re/re_set_next_write_params.o ../re/re_set_next_write_sequence_number.o ../re/re_start_operation.o ../re/re_write_sa.o

module-re=elpre
OUTPUT-RE = ${module-re}.o
${module-re}-objs := ${OBJS-RE}
obj-m := ${OUTPUT-RE}

OBJS-REUSR += re_dev.o
module-reusr=elpreusr
OUTPUT-REUSR= ${module-reusr}.o
${module-reusr}-objs := ${OBJS-REUSR}
obj-m += ${OUTPUT-REUSR}

OBJS-KEP += kep.o ../kep/kep_close.o ../kep/kep_done.o ../kep/kep_fini.o ../kep/kep_go.o ../kep/kep_init.o ../kep/kep_load_keys.o ../kep/kep_open.o ../kep/kep_is_valid.o

module-kep=elpkep
OUTPUT-KEP= ${module-kep}.o
${module-kep}-objs := ${OBJS-KEP}
obj-m += ${OUTPUT-KEP}

OBJS-KEPDEV += kep_dev.o
module-kepdev=elpkepdev
OUTPUT-KEPDEV= ${module-kepdev}.o
${module-kepdev}-objs := ${OBJS-KEPDEV}
obj-m += ${OUTPUT-KEPDEV}

all: 
	RD=${RD} CFLAGS="${CFLAGS}" $(MAKE) -C $(KERNELDIR) SUBDIRS=$(RD) modules

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions *.symvers Module.* modules.*

