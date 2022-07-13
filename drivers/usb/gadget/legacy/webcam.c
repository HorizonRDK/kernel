// SPDX-License-Identifier: GPL-2.0+
/*
 *	webcam.c -- USB webcam gadget driver
 *
 *	Copyright (C) 2009-2010
 *	    Laurent Pinchart (laurent.pinchart@ideasonboard.com)
 */

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/usb/video.h>

#include "u_uvc.h"

USB_GADGET_COMPOSITE_OPTIONS();

/*-------------------------------------------------------------------------*/

/* module parameters specific to the Video streaming endpoint */
static unsigned int streaming_interval = 1;
module_param(streaming_interval, uint, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(streaming_interval, "1 - 16");

/* streaming_maxpacket: usb2.0 isoc high-bandwidht transfer(192Mbps, 24MBps) */
// static unsigned int streaming_maxpacket = 1024 * 3;
static unsigned int streaming_maxpacket = 1024;
module_param(streaming_maxpacket, uint, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(streaming_maxpacket, "ISOC: 1 - 1023 (FS), 1 - 3072 (hs/ss) / "
		"BULK: 1 - 64 (FS), 1 - 512 (HS), 1 - 1024 (SS)");

static unsigned int streaming_maxburst;
module_param(streaming_maxburst, uint, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(streaming_maxburst, "0 - 15 (ss only)");

static unsigned int streaming_bulk = 0;
module_param(streaming_bulk, uint, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(streaming_bulk, "0 (use ISOC video streaming ep) / "
		"1 (use BULK video streaming ep)");

static unsigned int trace;
module_param(trace, uint, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(trace, "Trace level bitmask");
/* --------------------------------------------------------------------------
 * Device descriptor
 */

#define WEBCAM_VENDOR_ID		0x1d6b	/* Linux Foundation */
#define WEBCAM_PRODUCT_ID		0x0102	/* Webcam A/V gadget */
#define WEBCAM_DEVICE_BCD		0x0010	/* 0.10 */

static char webcam_vendor_label[] = "Linux Foundation";
static char webcam_product_label[] = "Webcam gadget";
static char webcam_config_label[] = "Video";

/* string IDs are assigned dynamically */

#define STRING_DESCRIPTION_IDX		USB_GADGET_FIRST_AVAIL_IDX

static struct usb_string webcam_strings[] = {
	[USB_GADGET_MANUFACTURER_IDX].s = webcam_vendor_label,
	[USB_GADGET_PRODUCT_IDX].s = webcam_product_label,
	[USB_GADGET_SERIAL_IDX].s = "",
	[STRING_DESCRIPTION_IDX].s = webcam_config_label,
	{  }
};

static struct usb_gadget_strings webcam_stringtab = {
	.language = 0x0409,	/* en-us */
	.strings = webcam_strings,
};

static struct usb_gadget_strings *webcam_device_strings[] = {
	&webcam_stringtab,
	NULL,
};

static struct usb_function_instance *fi_uvc;
static struct usb_function *f_uvc;

static struct usb_device_descriptor webcam_device_descriptor = {
	.bLength		= USB_DT_DEVICE_SIZE,
	.bDescriptorType	= USB_DT_DEVICE,
	/* .bcdUSB = DYNAMIC */
	.bDeviceClass		= USB_CLASS_MISC,
	.bDeviceSubClass	= 0x02,
	.bDeviceProtocol	= 0x01,
	.bMaxPacketSize0	= 0, /* dynamic */
	.idVendor		= cpu_to_le16(WEBCAM_VENDOR_ID),
	.idProduct		= cpu_to_le16(WEBCAM_PRODUCT_ID),
	.bcdDevice		= cpu_to_le16(WEBCAM_DEVICE_BCD),
	.iManufacturer		= 0, /* dynamic */
	.iProduct		= 0, /* dynamic */
	.iSerialNumber		= 0, /* dynamic */
	.bNumConfigurations	= 0, /* dynamic */
};

DECLARE_UVC_HEADER_DESCRIPTOR(1);

static const struct UVC_HEADER_DESCRIPTOR(1) uvc_control_header = {
	.bLength		= UVC_DT_HEADER_SIZE(1),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VC_HEADER,
#if (USB_VIDEO_CLASS_VERSION == 0x150)
	.bcdUVC			= cpu_to_le16(0x0150),
#else
	.bcdUVC			= cpu_to_le16(0x0100),
#endif
	.wTotalLength		= 0, /* dynamic */
	.dwClockFrequency	= cpu_to_le32(48000000),
	.bInCollection		= 0, /* dynamic */
	.baInterfaceNr[0]	= 0, /* dynamic */
};

static const struct uvc_camera_terminal_descriptor uvc_camera_terminal = {
	.bLength		= UVC_DT_CAMERA_TERMINAL_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VC_INPUT_TERMINAL,
	.bTerminalID		= 1,
	.wTerminalType		= cpu_to_le16(0x0201),
	.bAssocTerminal		= 0,
	.iTerminal		= 0,
	.wObjectiveFocalLengthMin	= cpu_to_le16(0),
	.wObjectiveFocalLengthMax	= cpu_to_le16(0),
	.wOcularFocalLength		= cpu_to_le16(0),
	.bControlSize		= 3,
	.bmControls[0]		= 2,
	.bmControls[1]		= 0,
	.bmControls[2]		= 0,
};

static const struct uvc_processing_unit_descriptor uvc_processing = {
	.bLength		= UVC_DT_PROCESSING_UNIT_SIZE(2),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VC_PROCESSING_UNIT,
	.bUnitID		= 2,
	.bSourceID		= 1,
	.wMaxMultiplier		= cpu_to_le16(16*1024),
	.bControlSize		= 2,
	.bmControls[0]		= 1,
	.bmControls[1]		= 0,
#if (USB_VIDEO_CLASS_VERSION == 0x150)
	.bmControls[2]		= 0,
#endif
	.iProcessing		= 0,
};

static const struct uvc_output_terminal_descriptor uvc_output_terminal = {
	.bLength		= UVC_DT_OUTPUT_TERMINAL_SIZE,
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VC_OUTPUT_TERMINAL,
	.bTerminalID		= 3,
	.wTerminalType		= cpu_to_le16(0x0101),
	.bAssocTerminal		= 0,
	.bSourceID		= 2,
	.iTerminal		= 0,
};

DECLARE_UVC_INPUT_HEADER_DESCRIPTOR(1, 4);
DECLARE_UVC_FRAME_UNCOMPRESSED(3);
DECLARE_UVC_FRAME_MJPEG(3);
DECLARE_UVC_FRAME_FRAMEBASED(3);
#if (USB_VIDEO_CLASS_VERSION == 0x150)
DECLARE_UVC_FRAME_H264(3);
#endif

// #define SUPPORT_YUY2
#define SUPPORT_NV12
#define SUPPORT_MJPG
#define SUPPORT_H264
#define SUPPORT_H265

static const struct UVC_INPUT_HEADER_DESCRIPTOR(1, 4) uvc_input_header = {
	.bLength		= UVC_DT_INPUT_HEADER_SIZE(1, 4),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_INPUT_HEADER,
	.bNumFormats		= 4, /* nv12 , mjpg, h264, h265 */
	.wTotalLength		= 0, /* dynamic */
	.bEndpointAddress	= 0, /* dynamic */
	.bmInfo			= 0,
	.bTerminalLink		= 3,
	.bStillCaptureMethod	= 0,
	.bTriggerSupport	= 0,
	.bTriggerUsage		= 0,
	.bControlSize		= 1,
	.bmaControls[0][0]	= 0,
	.bmaControls[1][0]	= 4,
	.bmaControls[2][0]	= 4,
	.bmaControls[3][0]	= 4,
};

#ifdef SUPPORT_YUY2
static const struct uvc_format_uncompressed uvc_format_yuy2 = {
	.bLength		= UVC_DT_FORMAT_UNCOMPRESSED_SIZE,
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FORMAT_UNCOMPRESSED,
	.bFormatIndex		= 1,
	.bNumFrameDescriptors	= 2,
	.guidFormat		=
		{ 'Y',  'U',  'Y',  '2', 0x00, 0x00, 0x10, 0x00,
		 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71},
	.bBitsPerPixel		= 16,
	.bDefaultFrameIndex	= 1,
	.bAspectRatioX		= 0,
	.bAspectRatioY		= 0,
	.bmInterfaceFlags	= 0,
	.bCopyProtect		= 0,
};

static const struct UVC_FRAME_UNCOMPRESSED(3) uvc_frame_yuy2_720p = {
	.bLength		= UVC_DT_FRAME_UNCOMPRESSED_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_UNCOMPRESSED,
	.bFrameIndex		= 1,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(1280),
	.wHeight		= cpu_to_le16(720),
	.dwMinBitRate		= cpu_to_le32(1280*720*2*8*10),
	.dwMaxBitRate		= cpu_to_le32(1280*720*2*8*30),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(1280*720*2),
	.dwDefaultFrameInterval	= cpu_to_le32(333333),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
	.dwFrameInterval[2]	= cpu_to_le32(1000000),
};

static const struct UVC_FRAME_UNCOMPRESSED(3) uvc_frame_yuy2_1080p = {
	.bLength		= UVC_DT_FRAME_UNCOMPRESSED_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_UNCOMPRESSED,
	.bFrameIndex		= 2,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(1920),
	.wHeight		= cpu_to_le16(1080),
	.dwMinBitRate		= cpu_to_le32(1920*1080*2*8*10),
	.dwMaxBitRate		= cpu_to_le32(1920*1080*2*8*30),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(1920*1080*2),
	.dwDefaultFrameInterval	= cpu_to_le32(333333),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
	.dwFrameInterval[2]	= cpu_to_le32(1000000),
};

#define UVC_DESCRIPTOR_HEADERS_OF_YUY2_FRAME \
	(const struct uvc_descriptor_header *) &uvc_format_yuy2, \
	(const struct uvc_descriptor_header *) &uvc_frame_yuy2_720p, \
	(const struct uvc_descriptor_header *) &uvc_frame_yuy2_1080p,
#else
#define UVC_DESCRIPTOR_HEADERS_OF_YUY2_FRAME
#endif

#ifdef SUPPORT_NV12
static const struct uvc_format_uncompressed uvc_format_nv12 = {
	.bLength		= UVC_DT_FORMAT_UNCOMPRESSED_SIZE,
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FORMAT_UNCOMPRESSED,
	.bFormatIndex		= 1,
	.bNumFrameDescriptors	= 3,
	.guidFormat		=
		{ 'N',  'V',  '1',  '2', 0x00, 0x00, 0x10, 0x00,
		 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71},
	.bBitsPerPixel		= 12,
	.bDefaultFrameIndex	= 1,
	.bAspectRatioX		= 0,
	.bAspectRatioY		= 0,
	.bmInterfaceFlags	= 0,
	.bCopyProtect		= 0,
};

static const struct UVC_FRAME_UNCOMPRESSED(3) uvc_frame_nv12_720p = {
	.bLength		= UVC_DT_FRAME_UNCOMPRESSED_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_UNCOMPRESSED,
	.bFrameIndex		= 1,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(1280),
	.wHeight		= cpu_to_le16(720),
	.dwMinBitRate		= cpu_to_le32(1280*720*1.5*8*10),
	.dwMaxBitRate		= cpu_to_le32(1280*720*1.5*8*30),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(1280*720*1.5),
	.dwDefaultFrameInterval	= cpu_to_le32(333333),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
	.dwFrameInterval[2]	= cpu_to_le32(1000000),
};

static const struct UVC_FRAME_UNCOMPRESSED(3) uvc_frame_nv12_1080p = {
	.bLength		= UVC_DT_FRAME_UNCOMPRESSED_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_UNCOMPRESSED,
	.bFrameIndex		= 2,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(1920),
	.wHeight		= cpu_to_le16(1080),
	.dwMinBitRate		= cpu_to_le32(1920*1080*1.5*8*10),
	.dwMaxBitRate		= cpu_to_le32(1920*1080*1.5*8*30),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(1920*1080*1.5),
	.dwDefaultFrameInterval	= cpu_to_le32(333333),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
	.dwFrameInterval[2]	= cpu_to_le32(1000000),
};

static const struct UVC_FRAME_UNCOMPRESSED(3) uvc_frame_nv12_2160p = {
	.bLength		= UVC_DT_FRAME_UNCOMPRESSED_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_UNCOMPRESSED,
	.bFrameIndex		= 3,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(3840),
	.wHeight		= cpu_to_le16(2160),
	.dwMinBitRate		= cpu_to_le32(3840*2160*1.5*8*10),
	.dwMaxBitRate		= cpu_to_le32(3840*2160*1.5*8*30),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(3840*2160*1.5),
	.dwDefaultFrameInterval	= cpu_to_le32(333333),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
	.dwFrameInterval[2]	= cpu_to_le32(1000000),
};
#define UVC_DESCRIPTOR_HEADERS_OF_NV12_FRAME \
	(const struct uvc_descriptor_header *) &uvc_format_nv12, \
	(const struct uvc_descriptor_header *) &uvc_frame_nv12_720p, \
	(const struct uvc_descriptor_header *) &uvc_frame_nv12_1080p, \
	(const struct uvc_descriptor_header *) &uvc_frame_nv12_2160p,
#else
#define UVC_DESCRIPTOR_HEADERS_OF_NV12_FRAME
#endif

#ifdef SUPPORT_MJPG
static const struct uvc_format_mjpeg uvc_format_mjpg = {
	.bLength		= UVC_DT_FORMAT_MJPEG_SIZE,
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FORMAT_MJPEG,
	.bFormatIndex		= 2,
	.bNumFrameDescriptors	= 3,
	.bmFlags		= 0,
	.bDefaultFrameIndex	= 1,
	.bAspectRatioX		= 0,
	.bAspectRatioY		= 0,
	.bmInterfaceFlags	= 0,
	.bCopyProtect		= 0,
};

static const struct UVC_FRAME_MJPEG(3) uvc_frame_mjpg_720p = {
	.bLength		= UVC_DT_FRAME_MJPEG_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_MJPEG,
	.bFrameIndex		= 1,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(1280),
	.wHeight		= cpu_to_le16(720),
	.dwMinBitRate		= cpu_to_le32(1280*720*2*8*10),
	.dwMaxBitRate		= cpu_to_le32(1280*720*2*8*30),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(1280*720*2),
	.dwDefaultFrameInterval	= cpu_to_le32(333333),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
	.dwFrameInterval[2]	= cpu_to_le32(1000000),
};

static const struct UVC_FRAME_MJPEG(3) uvc_frame_mjpg_1080p = {
	.bLength		= UVC_DT_FRAME_MJPEG_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_MJPEG,
	.bFrameIndex		= 2,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(1920),
	.wHeight		= cpu_to_le16(1080),
	.dwMinBitRate		= cpu_to_le32(1920*1080*2*8*10),
	.dwMaxBitRate		= cpu_to_le32(1920*1080*2*8*30),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(1920*1080*2),
	.dwDefaultFrameInterval	= cpu_to_le32(333333),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
	.dwFrameInterval[2]	= cpu_to_le32(1000000),
};

static const struct UVC_FRAME_MJPEG(3) uvc_frame_mjpg_2160p = {
	.bLength		= UVC_DT_FRAME_MJPEG_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_MJPEG,
	.bFrameIndex		= 3,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(3840),
	.wHeight		= cpu_to_le16(2160),
	.dwMinBitRate		= cpu_to_le32((__u32)3840*2160*2*8*10),
	.dwMaxBitRate		= cpu_to_le32((__u32)3840*2160*2*8*30),
	.dwMaxVideoFrameBufferSize	= cpu_to_le32(3840*2160*2),
	.dwDefaultFrameInterval	= cpu_to_le32(333333),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
	.dwFrameInterval[2]	= cpu_to_le32(1000000),
};

#define UVC_DESCRIPTOR_HEADERS_OF_MJPG_FRAME \
	(const struct uvc_descriptor_header *) &uvc_format_mjpg, \
	(const struct uvc_descriptor_header *) &uvc_frame_mjpg_720p, \
	(const struct uvc_descriptor_header *) &uvc_frame_mjpg_1080p, \
	(const struct uvc_descriptor_header *) &uvc_frame_mjpg_2160p,
#else
#define UVC_DESCRIPTOR_HEADERS_OF_MJPG_FRAME
#endif

#ifdef SUPPORT_H264
#if (USB_VIDEO_CLASS_VERSION == 0x150)
static const struct uvc_format_h264 uvc_format_h264 = {
	.bLength				= UVC_DT_FORMAT_H264_SIZE,
	.bDescriptorType			= USB_DT_CS_INTERFACE,
	.bDescriptorSubType			= UVC_VS_FORMAT_H264,
	.bFormatIndex				= 3,
	.bNumFrameDescriptors			= 3,
	.bDefaultFrameIndex			= 1,
	.bMaxCodecConfigDelay			= 0x4,
	.bmSupportedSliceModes			= 0,
	.bmSupportedSyncFrameTypes		= 0x76,
	.bResolutionScaling			= 0,
	.Reserved1				= 0,
	.bmSupportedRateControlModes		= 0x3F,
	.wMaxMBperSecOneResNoScalability	= cpu_to_le16(972),
	.wMaxMBperSecTwoResNoScalability	= 0,
	.wMaxMBperSecThreeResNoScalability	= 0,
	.wMaxMBperSecFourResNoScalability	= 0,
	.wMaxMBperSecOneResTemporalScalability	= cpu_to_le16(972),
	.wMaxMBperSecTwoResTemporalScalability	= 0,
	.wMaxMBperSecThreeResTemporalScalability	= 0,
	.wMaxMBperSecFourResTemporalScalability		= 0,
	.wMaxMBperSecOneResTemporalQualityScalability	= cpu_to_le16(972),
	.wMaxMBperSecTwoResTemporalQualityScalability	= 0,
	.wMaxMBperSecThreeResTemporalQualityScalability	= 0,
	.wMaxMBperSecFourResTemporalQualityScalability	= 0,
	.wMaxMBperSecOneResTemporalSpatialScalability	= 0,
	.wMaxMBperSecTwoResTemporalSpatialScalability	= 0,
	.wMaxMBperSecThreeResTemporalSpatialScalability	= 0,
	.wMaxMBperSecFourResTemporalSpatialScalability	= 0,
	.wMaxMBperSecOneResFullScalability		= 0,
	.wMaxMBperSecTwoResFullScalability		= 0,
	.wMaxMBperSecThreeResFullScalability	= 0,
	.wMaxMBperSecFourResFullScalability		= 0,
};

static const struct UVC_FRAME_H264(3) uvc_frame_h264_720p = {
	.bLength		= UVC_DT_FRAME_H264_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_H264,
	.bFrameIndex		= 1,
	.wWidth			= cpu_to_le16(1280),
	.wHeight		= cpu_to_le16(720),
	.wSARwidth		= 1,
	.wSARheight		= 1,
	.wProfile		= 0x6400,
	.bLevelIDC		= 0x33,
	.bmSupportedUsages	= 0x70003,
	.wConstrainedToolset	= cpu_to_le16(0),
	.bmCapabilities		= 0x47,
	.bmSVCCapabilities	= 0x4,
	.bmMVCCapabilities	= 0,
	.dwMinBitRate		= cpu_to_le32(1280*720*2*8*10),
	.dwMaxBitRate		= cpu_to_le32(1280*720*2*8*30),
	.dwDefaultFrameInterval	= cpu_to_le32(333333),
	.bNumFrameIntervals	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
	.dwFrameInterval[2]	= cpu_to_le32(1000000),
};

static const struct UVC_FRAME_H264(3) uvc_frame_h264_1080p = {
	.bLength		= UVC_DT_FRAME_H264_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_H264,
	.bFrameIndex		= 2,
	.wWidth			= cpu_to_le16(1920),
	.wHeight		= cpu_to_le16(1080),
	.wSARwidth		= 1,
	.wSARheight		= 1,
	.wProfile		= 0x6400,
	.bLevelIDC		= 0x33,
	.bmSupportedUsages	= 0x70003,
	.wConstrainedToolset	= cpu_to_le16(0),
	.bmCapabilities		= 0x47,
	.bmSVCCapabilities	= 0x4,
	.bmMVCCapabilities	= 0,
	.dwMinBitRate		= cpu_to_le32(1920*1080*2*8*10),
	.dwMaxBitRate		= cpu_to_le32(1920*1080*2*8*30),
	.dwDefaultFrameInterval	= cpu_to_le32(333333),
	.bNumFrameIntervals	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
	.dwFrameInterval[2]	= cpu_to_le32(1000000),
};

static const struct UVC_FRAME_H264(3) uvc_frame_h264_2160p = {
	.bLength		= UVC_DT_FRAME_H264_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_H264,
	.bFrameIndex		= 1,
	.wWidth			= cpu_to_le16(3840),
	.wHeight		= cpu_to_le16(2160),
	.wSARwidth		= 1,
	.wSARheight		= 1,
	.wProfile		= 0x6400,
	.bLevelIDC		= 0x33,
	.bmSupportedUsages	= 0x70003,
	.wConstrainedToolset	= cpu_to_le16(0),
	.bmCapabilities		= 0x47,
	.bmSVCCapabilities	= 0x4,
	.bmMVCCapabilities	= 0,
	.dwMinBitRate		= cpu_to_le32(3840*2160*2*8*10),
	.dwMaxBitRate		= cpu_to_le32(3840*2160*2*8*30),
	.dwDefaultFrameInterval	= cpu_to_le32(333333),
	.bNumFrameIntervals	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
	.dwFrameInterval[2]	= cpu_to_le32(1000000),
};
#else

static const struct uvc_format_framebased uvc_format_h264 = {
	.bLength		= UVC_DT_FORMAT_FRAMEBASED_SIZE,
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FORMAT_FRAME_BASED,
	.bFormatIndex		= 3,
	.bNumFrameDescriptors	= 3,
	.guidFormat		=
		{ 'H', '2',  '6', '4', 0x00, 0x00, 0x10, 0x00,
		 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71},
	.bBitsPerPixel		= 16,
	.bDefaultFrameIndex	= 1,
	.bAspectRatioX		= 0,
	.bAspectRatioY		= 0,
	.bmInterfaceFlags	= 0,
	.bCopyProtect		= 0,
	.bVariableSize		= 1
};

static const struct UVC_FRAME_FRAMEBASED(3) uvc_frame_h264_720p = {
	.bLength		= UVC_DT_FRAME_FRAMEBASED_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_FRAME_BASED,
	.bFrameIndex		= 1,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(1280),
	.wHeight		= cpu_to_le16(720),
	.dwMinBitRate		= cpu_to_le32(1280*720*2*8*10),
	.dwMaxBitRate		= cpu_to_le32(1280*720*2*8*30),
	.dwDefaultFrameInterval = cpu_to_le32(333333),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
	.dwFrameInterval[2]	= cpu_to_le32(1000000),
};

static const struct UVC_FRAME_FRAMEBASED(3) uvc_frame_h264_1080p = {
	.bLength		= UVC_DT_FRAME_FRAMEBASED_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_FRAME_BASED,
	.bFrameIndex		= 2,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(1920),
	.wHeight		= cpu_to_le16(1080),
	.dwMinBitRate		= cpu_to_le32(1920*1080*2*8*10),
	.dwMaxBitRate		= cpu_to_le32(1920*1080*2*8*30), // int32_t overflow
	.dwDefaultFrameInterval = cpu_to_le32(333333),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
	.dwFrameInterval[2]	= cpu_to_le32(1000000),
};

static const struct UVC_FRAME_FRAMEBASED(3) uvc_frame_h264_2160p = {
	.bLength		= UVC_DT_FRAME_FRAMEBASED_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_FRAME_BASED,
	.bFrameIndex		= 3,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(3840),
	.wHeight		= cpu_to_le16(2160),
	.dwMinBitRate		= cpu_to_le32((__u32)3840*2160*2*8*10),
	.dwMaxBitRate		= cpu_to_le32((__u32)3840*2160*2*8*30), // int32_t overflow
	.dwDefaultFrameInterval = cpu_to_le32(333333),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
	.dwFrameInterval[2]	= cpu_to_le32(1000000),
};

#endif
#define UVC_DESCRIPTOR_HEADERS_OF_H264_FRAME \
	(const struct uvc_descriptor_header *) &uvc_format_h264, \
	(const struct uvc_descriptor_header *) &uvc_frame_h264_720p, \
	(const struct uvc_descriptor_header *) &uvc_frame_h264_1080p, \
	(const struct uvc_descriptor_header *) &uvc_frame_h264_2160p,
#else
#define UVC_DESCRIPTOR_HEADERS_OF_H264_FRAME
#endif

#ifdef SUPPORT_H265
static const struct uvc_format_framebased uvc_format_h265 = {
	.bLength		= UVC_DT_FORMAT_FRAMEBASED_SIZE,
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FORMAT_FRAME_BASED,
	.bFormatIndex		= 4,
	.bNumFrameDescriptors	= 3,
	.guidFormat		=
		{ 'H', '2',  '6', '5', 0x00, 0x00, 0x10, 0x00,
		 0x80, 0x00, 0x00, 0xaa, 0x00, 0x38, 0x9b, 0x71},
	.bBitsPerPixel		= 16,
	.bDefaultFrameIndex	= 1,
	.bAspectRatioX		= 0,
	.bAspectRatioY		= 0,
	.bmInterfaceFlags	= 0,
	.bCopyProtect		= 0,
	.bVariableSize		= 1
};

static const struct UVC_FRAME_FRAMEBASED(3) uvc_frame_h265_720p = {
	.bLength		= UVC_DT_FRAME_FRAMEBASED_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_FRAME_BASED,
	.bFrameIndex		= 1,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(1280),
	.wHeight		= cpu_to_le16(720),
	.dwMinBitRate		= cpu_to_le32(1280*720*2*8*10),
	.dwMaxBitRate		= cpu_to_le32(1280*720*2*8*30),
	.dwDefaultFrameInterval = cpu_to_le32(333333),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
	.dwFrameInterval[2]	= cpu_to_le32(1000000),
};

static const struct UVC_FRAME_FRAMEBASED(3) uvc_frame_h265_1080p = {
	.bLength		= UVC_DT_FRAME_FRAMEBASED_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_FRAME_BASED,
	.bFrameIndex		= 2,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(1920),
	.wHeight		= cpu_to_le16(1080),
	.dwMinBitRate		= cpu_to_le32(1920*1080*2*8*10),
	.dwMaxBitRate		= cpu_to_le32(1920*1080*2*8*30), // int32_t overflow
	.dwDefaultFrameInterval = cpu_to_le32(333333),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
	.dwFrameInterval[2]	= cpu_to_le32(1000000),
};

static const struct UVC_FRAME_FRAMEBASED(3) uvc_frame_h265_2160p = {
	.bLength		= UVC_DT_FRAME_FRAMEBASED_SIZE(3),
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_FRAME_FRAME_BASED,
	.bFrameIndex		= 3,
	.bmCapabilities		= 0,
	.wWidth			= cpu_to_le16(3840),
	.wHeight		= cpu_to_le16(2160),
	.dwMinBitRate		= cpu_to_le32((__u32)3840*2160*2*8*10),
	.dwMaxBitRate		= cpu_to_le32((__u32)3840*2160*2*8*30), // int32_t overflow
	.dwDefaultFrameInterval = cpu_to_le32(333333),
	.bFrameIntervalType	= 3,
	.dwFrameInterval[0]	= cpu_to_le32(333333),
	.dwFrameInterval[1]	= cpu_to_le32(666666),
	.dwFrameInterval[2]	= cpu_to_le32(1000000),
};

#define UVC_DESCRIPTOR_HEADERS_OF_H265_FRAME \
	(const struct uvc_descriptor_header *) &uvc_format_h265, \
	(const struct uvc_descriptor_header *) &uvc_frame_h265_720p, \
	(const struct uvc_descriptor_header *) &uvc_frame_h265_1080p, \
	(const struct uvc_descriptor_header *) &uvc_frame_h265_2160p, \
	(const struct uvc_descriptor_header *) &uvc_color_matching,
#else
#define UVC_DESCRIPTOR_HEADERS_OF_H265_FRAME
#endif

static const struct uvc_color_matching_descriptor uvc_color_matching = {
	.bLength		= UVC_DT_COLOR_MATCHING_SIZE,
	.bDescriptorType	= USB_DT_CS_INTERFACE,
	.bDescriptorSubType	= UVC_VS_COLORFORMAT,
	.bColorPrimaries	= 1,
	.bTransferCharacteristics	= 1,
	.bMatrixCoefficients	= 4,
};

static const struct uvc_descriptor_header * const uvc_fs_control_cls[] = {
	(const struct uvc_descriptor_header *) &uvc_control_header,
	(const struct uvc_descriptor_header *) &uvc_camera_terminal,
	(const struct uvc_descriptor_header *) &uvc_processing,
	(const struct uvc_descriptor_header *) &uvc_output_terminal,
	NULL,
};

static const struct uvc_descriptor_header * const uvc_ss_control_cls[] = {
	(const struct uvc_descriptor_header *) &uvc_control_header,
	(const struct uvc_descriptor_header *) &uvc_camera_terminal,
	(const struct uvc_descriptor_header *) &uvc_processing,
	(const struct uvc_descriptor_header *) &uvc_output_terminal,
	NULL,
};

static const struct uvc_descriptor_header * const uvc_fs_streaming_cls[] = {
	(const struct uvc_descriptor_header *) &uvc_input_header,
	UVC_DESCRIPTOR_HEADERS_OF_YUY2_FRAME
	UVC_DESCRIPTOR_HEADERS_OF_NV12_FRAME
	UVC_DESCRIPTOR_HEADERS_OF_MJPG_FRAME
	UVC_DESCRIPTOR_HEADERS_OF_H264_FRAME
	UVC_DESCRIPTOR_HEADERS_OF_H265_FRAME
	NULL,
};

static const struct uvc_descriptor_header * const uvc_hs_streaming_cls[] = {
	(const struct uvc_descriptor_header *) &uvc_input_header,
	UVC_DESCRIPTOR_HEADERS_OF_YUY2_FRAME
	UVC_DESCRIPTOR_HEADERS_OF_NV12_FRAME
	UVC_DESCRIPTOR_HEADERS_OF_MJPG_FRAME
	UVC_DESCRIPTOR_HEADERS_OF_H264_FRAME
	UVC_DESCRIPTOR_HEADERS_OF_H265_FRAME
	NULL,
};

static const struct uvc_descriptor_header * const uvc_ss_streaming_cls[] = {
	(const struct uvc_descriptor_header *) &uvc_input_header,
	UVC_DESCRIPTOR_HEADERS_OF_YUY2_FRAME
	UVC_DESCRIPTOR_HEADERS_OF_NV12_FRAME
	UVC_DESCRIPTOR_HEADERS_OF_MJPG_FRAME
	UVC_DESCRIPTOR_HEADERS_OF_H264_FRAME
	UVC_DESCRIPTOR_HEADERS_OF_H265_FRAME
	NULL,
};

/* --------------------------------------------------------------------------
 * USB configuration
 */

static int
webcam_config_bind(struct usb_configuration *c)
{
	int status = 0;

	f_uvc = usb_get_function(fi_uvc);
	if (IS_ERR(f_uvc))
		return PTR_ERR(f_uvc);

	status = usb_add_function(c, f_uvc);
	if (status < 0)
		usb_put_function(f_uvc);

	return status;
}

static struct usb_configuration webcam_config_driver = {
	.label			= webcam_config_label,
	.bConfigurationValue	= 1,
	.iConfiguration		= 0, /* dynamic */
	.bmAttributes		= USB_CONFIG_ATT_SELFPOWER,
	.MaxPower		= CONFIG_USB_GADGET_VBUS_DRAW,
};

static int
webcam_unbind(struct usb_composite_dev *cdev)
{
	if (!IS_ERR_OR_NULL(f_uvc))
		usb_put_function(f_uvc);
	if (!IS_ERR_OR_NULL(fi_uvc))
		usb_put_function_instance(fi_uvc);
	return 0;
}

static int
webcam_bind(struct usb_composite_dev *cdev)
{
	struct f_uvc_opts *uvc_opts;
	int ret;

	fi_uvc = usb_get_function_instance("uvc");
	if (IS_ERR(fi_uvc))
		return PTR_ERR(fi_uvc);

	uvc_opts = container_of(fi_uvc, struct f_uvc_opts, func_inst);

	uvc_opts->streaming_interval = streaming_interval;
	uvc_opts->streaming_maxpacket = streaming_maxpacket;
	uvc_opts->streaming_maxburst = streaming_maxburst;
	uvc_opts->streaming_bulk = streaming_bulk;

	uvc_opts->fs_control = uvc_fs_control_cls;
	uvc_opts->ss_control = uvc_ss_control_cls;
	uvc_opts->fs_streaming = uvc_fs_streaming_cls;
	uvc_opts->hs_streaming = uvc_hs_streaming_cls;
	uvc_opts->ss_streaming = uvc_ss_streaming_cls;

	/* Allocate string descriptor numbers ... note that string contents
	 * can be overridden by the composite_dev glue.
	 */
	ret = usb_string_ids_tab(cdev, webcam_strings);
	if (ret < 0)
		goto error;
	webcam_device_descriptor.iManufacturer =
		webcam_strings[USB_GADGET_MANUFACTURER_IDX].id;
	webcam_device_descriptor.iProduct =
		webcam_strings[USB_GADGET_PRODUCT_IDX].id;
	webcam_config_driver.iConfiguration =
		webcam_strings[STRING_DESCRIPTION_IDX].id;

	/* Register our configuration. */
	if ((ret = usb_add_config(cdev, &webcam_config_driver,
					webcam_config_bind)) < 0)
		goto error;

	usb_composite_overwrite_options(cdev, &coverwrite);
	INFO(cdev, "Webcam Video Gadget\n");
	return 0;

error:
	usb_put_function_instance(fi_uvc);
	return ret;
}

/* --------------------------------------------------------------------------
 * Driver
 */

static struct usb_composite_driver webcam_driver = {
	.name		= "g_webcam",
	.dev		= &webcam_device_descriptor,
	.strings	= webcam_device_strings,
	.max_speed	= USB_SPEED_SUPER,
	.bind		= webcam_bind,
	.unbind		= webcam_unbind,
};

module_usb_composite_driver(webcam_driver);

MODULE_AUTHOR("Laurent Pinchart");
MODULE_DESCRIPTION("Webcam Video Gadget");
MODULE_LICENSE("GPL");

