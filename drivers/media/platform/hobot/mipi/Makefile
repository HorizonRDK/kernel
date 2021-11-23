ccflags-$(CONFIG_HOBOT_MIPI_CLK_RECALCULATION) += -DADJUST_CLK_RECALCULATION
ccflags-y += -std=gnu11 -Wno-declaration-after-statement
obj-$(CONFIG_HOBOT_MIPI_HOST) += hobot_mipi_host.o
obj-$(CONFIG_HOBOT_MIPI_DEV) += hobot_mipi_dev.o
obj-$(CONFIG_HOBOT_MIPI_PHY) += hobot_mipi_dphy.o

ifeq ($(CONFIG_HOBOT_GCOV_AVIO),y)
GCOV_PROFILE := y
endif
