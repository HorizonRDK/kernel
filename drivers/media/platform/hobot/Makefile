#
# Makefile for the Linux Kernel IPS specific device drivers.
#
subdir-ccflags-y := -Idrivers/media/platform/hobot/
subdir-ccflags-y += -Idrivers/media/platform/hobot/sif/
subdir-ccflags-y += -Idrivers/media/platform/hobot/ipu/
subdir-ccflags-y += -Idrivers/media/platform/hobot/pym/
subdir-ccflags-y += -Idrivers/media/platform/hobot/gdc/

vio-objs := common_api/vio_framemgr.o \
		common_api/vio_hw_common_api.o \
		common_api/vio_group_api.o \

obj-y += vio.o
obj-$(CONFIG_HOBOT_VIO) += ips/
obj-$(CONFIG_HOBOT_VIO) += sif/
obj-$(CONFIG_HOBOT_VIO) += ipu/
obj-$(CONFIG_HOBOT_VIO) += pym/
#obj-$(CONFIG_HOBOT_VIO) += gdc/

obj-y += mipi/
