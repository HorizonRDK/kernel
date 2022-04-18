dtb-$(CONFIG_HOBOT_XJ2)  += hobot-x2-soc.dtb
dtb-$(CONFIG_HOBOT_XJ2)  += hobot-x2-som.dtb
dtb-$(CONFIG_HOBOT_XJ2)  += hobot-x2-dev.dtb
dtb-$(CONFIG_HOBOT_XJ2)  += hobot-x2-dev-512m.dtb
dtb-$(CONFIG_HOBOT_XJ2)  += hobot-j2-som.dtb
dtb-$(CONFIG_HOBOT_XJ2)  += hobot-x2-som-ipc.dtb
dtb-$(CONFIG_HOBOT_XJ2)  += hobot-j2-mono.dtb
dtb-$(CONFIG_HOBOT_XJ2)  += hobot-j2-dev.dtb
dtb-$(CONFIG_HOBOT_XJ2)  += hobot-j2-quad.dtb
dtb-$(CONFIG_HOBOT_XJ2)  += hobot-j2-mm.dtb
dtb-$(CONFIG_HOBOT_XJ2)  += hobot-j2-mm-s202.dtb
dtb-$(CONFIG_HOBOT_XJ2)  += hobot-j2-sk.dtb
dtb-$(CONFIG_HOBOT_XJ2)  += hobot-j2-sam.dtb
dtb-$(CONFIG_HOBOT_XJ2)  += hobot-x2-96board.dtb
dtb-$(CONFIG_HOBOT_XJ2)  += hobot-j2-mono-lite.dtb
dtb-$(CONFIG_HOBOT_XJ2)  += hobot-x2-xiaomi.dtb
dtb-$(CONFIG_HOBOT_FPGA_X3) += hobot-xj3-fpga.dtb
dtb-$(CONFIG_HOBOT_FPGA_X3) += hobot-xj3-fpga-haps.dtb
dtb-$(CONFIG_HOBOT_XJ3)  += hobot-x3-dvb.dtb
dtb-$(CONFIG_HOBOT_XJ3)  += hobot-j3-dvb.dtb
dtb-$(CONFIG_HOBOT_XJ3)  += hobot-xj3-cvb.dtb
dtb-$(CONFIG_HOBOT_XJ3)  += hobot-x3-sdb.dtb
dtb-$(CONFIG_HOBOT_XJ3)  += hobot-x3-sdb_v4.dtb
dtb-$(CONFIG_HOBOT_XJ3)  += hobot-x3-pi.dtb

ifeq ($(CONFIG_HOBOT_XJ3_CV_ADDITION),y)
	dtb-$(CONFIG_HOBOT_XJ3)  += hobot-xj3-soc-cv.dtb
	dtb-$(CONFIG_HOBOT_XJ3)  += hobot-xj3-fpga-cv.dtb
endif


always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
