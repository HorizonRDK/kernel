src_v4l2 := drivers/media/platform/hobot/isp/v4l2_dev
src_subdev := drivers/media/platform/hobot/isp/subdev
export src_v4l2 src_subdev
ifeq ($(CONFIG_VIDEO_HOBOT_ISP), m)
	subdir-ccflags-y += -DBUILD_MODULE
endif

subdir-ccflags-y += -Wno-declaration-after-statement

obj-$(CONFIG_VIDEO_HOBOT_ISP) += subdev/log_list/
obj-$(CONFIG_VIDEO_HOBOT_ISP) += subdev/iq/
obj-$(CONFIG_VIDEO_HOBOT_ISP) += subdev/lens/
obj-$(CONFIG_VIDEO_HOBOT_ISP) += subdev/sensor/
obj-$(CONFIG_VIDEO_HOBOT_ISP) += subdev/dwe/
obj-$(CONFIG_VIDEO_HOBOT_ISP) += subdev/common/
obj-$(CONFIG_VIDEO_HOBOT_ISP) += v4l2_dev/
