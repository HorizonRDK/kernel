#
# Makefile for the Linux Kernel IPS specific device drivers.
#
ifneq ($(KERNELRELEASE),)
obj-y += hobot_common_api.o
hobot_common_api-objs := vio_framemgr.o vio_hw_common_api.o vio_group_api.o
ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif

else

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
