#
# Makefile for the Linux Kernel IPS specific device drivers.
#
obj-$(CONFIG_HOBOT_IAR) += iar.o
obj-$(CONFIG_HOBOT_IAR) += iar_cdev.o
obj-$(CONFIG_HOBOT_IAR) += iar_mmap.o
iar-objs += mipi_dsi.o
iar-objs += hobot_rotation_neon64.o
iar-objs += hobot_iar.o
iar_cdev-objs := hobot_iar_cdev.o
iar_mmap-objs := hobot_iar_mmap.o

ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif

