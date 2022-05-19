
# Makefile for the Linux Kernel IPS specific device drivers.

ifneq ($(KERNELRELEASE),)
obj-m += hobot_osd.o
hobot_osd-objs := ./hobot_dev_osd.o ./hobot_osd_mem.o ./hobot_osd_process.o
CFLAGS_MODULE += -O3

EXTRA_CFLAGS += -Idrivers/media/platform/hobot/common_api/

ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif

else

TARGET := hobot_osd

FW_SRC := ./hobot_dev_osd.c ./hobot_osd_mem.c ./hobot_osd_process.c ../common_api/vio_framemgr.c ../common_api/vio_hw_common_api.c ../common_api/vio_group_api.c

export FW_SRC_OBJ := $(FW_SRC:.c=.o)

EXTRA_CFLAGS += -Idrivers/media/platform/hobot/common_api/

obj-$(CONFIG_HOBOT_VIO) += $(TARGET).o
$(TARGET)-objs += $(FW_SRC_OBJ)

# redefine KDIR
ifeq ($(KDIR),)
        _KDIR= $(PWD)/../../../../../
    $(info "set default kernel path ${_KDIR}")
else
    _KDIR=${KDIR}
endif

.PHONY:modules
modules:
	CROSS_COMPILE=${CROSS_COMPILE} make -C $(_KDIR) M=$(PWD) modules

clean:
	CROSS_COMPILE=${CROSS_COMPILE} make -C $(_KDIR) M=$(PWD) clean
endif
