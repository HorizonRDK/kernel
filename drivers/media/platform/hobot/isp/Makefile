src_v4l2 := drivers/media/platform/hobot/isp/v4l2_dev
src_subdev := drivers/media/platform/hobot/isp/subdev
ifeq ($(KBUILD_SRC),)
	KBUILD_SRC := .
endif
export src_v4l2 src_subdev KBUILD_SRC
ifeq ($(CONFIG_VIDEO_HOBOT_ISP), m)
	subdir-ccflags-y += -DBUILD_MODULE
endif

subdir-ccflags-y += -Wno-declaration-after-statement	\
	-Idrivers/media/platform/hobot/common_api/	\
	-I$(src_v4l2)/inc/		\
	-I$(src_v4l2)/inc/api/	\
	-I$(src_v4l2)/inc/sys/	\
	-I$(src_v4l2)/src/fw/

obj-$(CONFIG_VIDEO_HOBOT_ISP) += subdev/log_list/
obj-$(CONFIG_VIDEO_HOBOT_ISP) += subdev/iq/
obj-$(CONFIG_VIDEO_HOBOT_ISP) += subdev/sensor/
obj-$(CONFIG_VIDEO_HOBOT_ISP) += subdev/common/
obj-$(CONFIG_VIDEO_HOBOT_ISP) += subdev/lens/
obj-$(CONFIG_VIDEO_HOBOT_ISP) += subdev/dwe/
obj-$(CONFIG_VIDEO_HOBOT_ISP) += v4l2_dev/
