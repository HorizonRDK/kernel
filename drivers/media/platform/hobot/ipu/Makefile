#
# Makefile for the Linux Kernel IPS specific device drivers.
#
ifneq ($(KERNELRELEASE),)
obj-$(CONFIG_HOBOT_VIO) += hobot_ipu.o
hobot_ipu-objs := hobot_ipu_hw_reg.o ./hobot_dev_ipu.o

EXTRA_CFLAGS += -Idrivers/media/platform/hobot/common_api/
else

TARGET := hobot_ipu

#FW_SRC := $(wildcard ./*.c ../common_api/*.c)
FW_SRC := ./hobot_ipu_hw_reg.c ./hobot_dev_ipu.c ../common_api/vio_framemgr.c ../common_api/vio_hw_common_api.c ../common_api/vio_group_api.c

export FW_SRC_OBJ := $(FW_SRC:.c=.o)

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
