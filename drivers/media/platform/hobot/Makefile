#
# Makefile for the Linux Kernel IPS specific device drivers.
#
subdir-ccflags-y := -Idrivers/media/platform/hobot/
subdir-ccflags-y += -Idrivers/media/platform/hobot/sif/
subdir-ccflags-y += -Idrivers/media/platform/hobot/ipu/
subdir-ccflags-y += -Idrivers/media/platform/hobot/pym/
subdir-ccflags-y += -Idrivers/media/platform/hobot/gdc/
subdir-ccflags-y += -Idrivers/media/platform/hobot/osd/
subdir-ccflags-y += -Idrivers/media/platform/hobot/common_api/

# obj-m += osd/
obj-y += common_api/
obj-y += ips/
obj-$(CONFIG_HOBOT_VIO) += sif/
obj-$(CONFIG_HOBOT_VIO) += isp/
obj-$(CONFIG_HOBOT_VIO) += ipu/
obj-$(CONFIG_HOBOT_VIO) += pym/
obj-$(CONFIG_HOBOT_VIO) += gdc/
obj-$(CONFIG_HOBOT_VIO) += osd/
obj-y += hmipc/

obj-$(CONFIG_HOBOT_MIPI_CSI) += mipi/

EXTRA_CFLAGS += -Idrivers/media/platform/hobot/ips/

