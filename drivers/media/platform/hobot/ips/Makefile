#
# Makefile for the Linux Kernel IPS specific device drivers.
#
obj-y += hobot_ips_hw_reg.o
obj-y += hobot_dev_ips.o

EXTRA_CFLAGS += -Idrivers/media/platform/hobot/common_api/
ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif
