#
# Makefile for the Linux Kernel IPS specific device drivers.
#
vio-objs := vio_framemgr.o \
		vio_hw_common_api.o \
		vio_group_api.o \

obj-y += vio.o
obj-$(CONFIG_HOBOT_VIO) += ips/
obj-$(CONFIG_HOBOT_VIO) += sif/
obj-$(CONFIG_HOBOT_VIO) += ipu/
obj-$(CONFIG_HOBOT_VIO) += pym/
#obj-$(CONFIG_HOBOT_VIO) += gdc/

subdir-ccflags-y := -Idrivers/media/platform/hobot/vio/
subdir-ccflags-y += -Idrivers/media/platform/hobot/vio/sif/
subdir-ccflags-y += -Idrivers/media/platform/hobot/vio/ipu/
subdir-ccflags-y += -Idrivers/media/platform/hobot/vio/pym/
subdir-ccflags-y += -Idrivers/media/platform/hobot/vio/gdc/
