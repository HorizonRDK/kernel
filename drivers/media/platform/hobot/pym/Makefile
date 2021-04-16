#
# Makefile for the Linux Kernel IPS specific device drivers.
#
ifneq ($(KERNELRELEASE),)
obj-$(CONFIG_HOBOT_VIO) += hobot_pym.o
hobot_pym-objs := hobot_pym_hw_reg.o hobot_dev_pym.o

EXTRA_CFLAGS += -Idrivers/media/platform/hobot/common_api/

ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif

else

TARGET := hobot_pym

#FW_SRC := $(wildcard ./*.c ../common_api/*.c)
FW_SRC := ./hobot_pym_hw_reg.c ./hobot_dev_pym.c ../common_api/vio_framemgr.c ../common_api/vio_hw_common_api.c ../common_api/vio_group_api.c

export FW_SRC_OBJ := $(FW_SRC:.c=.o)

ccflags-y:= -I./ -I../
EXTRA_CFLAGS += -Idrivers/media/platform/hobot/common_api/

obj-$(CONFIG_HOBOT_VIO) += $(TARGET).o
$(TARGET)-objs += $(FW_SRC_OBJ)

ifeq ($(CROSS_COMPILE),)
    _CROSS_COMPILE = ~/codes/testbench-master/hbtdisk/toolchain/aarch64-gcc-linaro-7.2.1-2017.11/bin/aarch64-linux-gnu-
    $(info "set default toolachains ${_CROSS_COMPILE}")
else
    _CROSS_COMPILE=${CROSS_COMPILE}
endif

ifeq ($(ARCH),)
    _ARCH=arm64
    $(info "set default ARCH=${_ARCH}")
else
    _ARCH=${ARCH}
endif

#redefine KDIR
ifeq ($(KDIR),)
        _KDIR= $(PWD)/../../../../../
    $(info "set default kernel path ${_KDIR}")
else
    _KDIR=${KDIR}
endif

default:
	CROSS_COMPILE=${_CROSS_COMPILE} make ARCH=${_ARCH} -C $(_KDIR) M=$(PWD) modules

clean:
	CROSS_COMPILE=${_CROSS_COMPILE} make ARCH=${_ARCH} -C $(_KDIR) M=$(PWD) clean
endif
