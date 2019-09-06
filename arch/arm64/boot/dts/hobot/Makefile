dtb-$(CONFIG_X2_FPGA) += hobot-x2-fpga.dtb
dtb-$(CONFIG_X2_SOC)  += hobot-x2-soc.dtb
dtb-$(CONFIG_X2_SOC)  += hobot-x2-som.dtb
dtb-$(CONFIG_X2_SOC)  += hobot-x2-dev.dtb
dtb-$(CONFIG_X2_SOC)  += hobot-x2-dev-512m.dtb
dtb-$(CONFIG_X2_SOC)  += hobot-j2-som.dtb
dtb-$(CONFIG_X2_SOC)  += hobot-x2-som-ipc.dtb
dtb-$(CONFIG_X2_SOC)  += hobot-j2-mono.dtb
dtb-$(CONFIG_X2_SOC)  += hobot-j2-dev.dtb
dtb-$(CONFIG_X2_SOC)  += hobot-j2-quad.dtb
dtb-$(CONFIG_X2_SOC)  += hobot-j2-mm.dtb
dtb-$(CONFIG_X2_SOC)  += hobot-j2-mm-s202.dtb
dtb-$(CONFIG_X2_SOC)  += hobot-j2-sk.dtb
dtb-$(CONFIG_X2_SOC)  += hobot-j2-sam.dtb
dtb-$(CONFIG_X2_SOC)  += hobot-x2-96board.dtb
dtb-$(CONFIG_X2_SOC)  += hobot-j2-mono-lite.dtb
dtb-$(CONFIG_X2_SOC)  += hobot-x2-xiaomi.dtb
dtb-$(CONFIG_X2A_FPGA) += hobot-x2aj2a-fpga.dtb

ifeq ($(CONFIG_X2A_FPGA),y)
	ifeq ($(CONFIG_USB_DWC3_HOST),y)
		dtb-y += hobot-x2aj2a-fpga-usb-host.dtb
	else ifeq ($(CONFIG_USB_DWC3_GADGET),y)
		dtb-y += hobot-x2aj2a-fpga-usb-device.dtb
	else
		dtb-y += hobot-x2aj2a-fpga.dtb
	endif
endif

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
