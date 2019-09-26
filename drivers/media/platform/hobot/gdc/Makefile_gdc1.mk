CONFIG_MODULE_SIG=n


#redefine ARCH
ifeq ($(ARCH),)
    _ARCH=arm64
    $(info "set default ARCH=${_ARCH}")
else
    _ARCH=${ARCH}
endif


#redefine CROSS_COMPILE
ifeq ($(CROSS_COMPILE),)
    _CROSS_COMPILE=aarch64-linux-gnu-
    $(info "set default toolachains ${_CROSS_COMPILE}")
else
    _CROSS_COMPILE=${CROSS_COMPILE}
endif

#redefine KDIR
ifeq ($(KDIR),)
	_KDIR= $(PWD)/../../../../../
    $(info "set default kernel path ${_KDIR}")
else
    _KDIR=${KDIR}
endif


obj-m += gdc1.o

ifeq ($(FW_SRC_OBJ),)
	FW_SRC := $(wildcard src/*.c src/*/*.c src/*/*/*.c app/*.c app/*/*.c ../bsp/*.c)
	export FW_SRC_OBJ := $(FW_SRC:.c=.o)
endif


gdc1-objs += $(FW_SRC_OBJ)

INCLUDE_DIRS := $(addprefix -I,$(shell find ../ -type d ))

ccflags-y:=-I$(PWD)/app -I$(PWD)/inc -I$(PWD)/app/control -I$(PWD)/inc/api -I$(PWD)/inc/gdc -I$(PWD)/inc/sys -I$(PWD)/src/platform -I$(PWD)/src/fw  -I$(PWD)/src/fw_lib  -I$(PWD)/src/driver/lens -I$(_KDIR)/include/linux/

ccflags-y += -Wno-declaration-after-statement

$(info "make driver for GDC1")

all:
		CROSS_COMPILE=${_CROSS_COMPILE} make ARCH=${_ARCH} -C $(_KDIR) M=$(PWD) modules

clean:
		CROSS_COMPILE=${_CROSS_COMPILE} make ARCH=${_ARCH} -C $(_KDIR) M=$(PWD) clean

