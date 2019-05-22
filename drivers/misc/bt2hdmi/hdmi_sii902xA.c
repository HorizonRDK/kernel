/***************************************************************************
 *
 * SIMG PART NUMBER - HDMI Transmitter Driver
 *
 * Copyright (C) (2011, Silicon Image)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 *****************************************************************************/

#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/console.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

#include "siHdmiTx_902x_TPI.h"


#define DEVICE_NAME	"sii902xA"

#define X2_HDMI_HOTPOLL_MIN		(20)
#define X2_HDMI_HOTPOLL_DEF		(500)

#define X2_HDMI_VMODE_DEF		(HDMI_1080P60)
#define X2_HDMI_VFORMAT_DEF		(VMD_HDMIFORMAT_HB)
#define X2_HDMI_AFS_DEF			(AFS_48K)

static unsigned int hotpoll_en;
static unsigned int hotpoll_ms;
static unsigned int vmode;
static unsigned int vformat;
static unsigned int afs;
static spinlock_t sii902xA_lock;
static struct work_struct	*sii902xAwork;

// hotplug service with timer poll.
static void x2_hdmi_timer(unsigned long dontcare);

static DEFINE_TIMER(x2hdmitimer, x2_hdmi_timer, 0, 0);
static void x2_hdmi_timer(unsigned long dontcare)
{
	if (gpio_get_value(Si9022A_irq_pin) == 0) {
		schedule_work(sii902xAwork);
	} else {
		mod_timer(&x2hdmitimer, jiffies + msecs_to_jiffies(hotpoll_ms));
	}
}

// hotplug service with interrupt(X2J2 not support LEVEL gpio irq).
static irqreturn_t sii902xA_interrupt(int irq, void *dev_id)
{
	unsigned long lock_flags = 0;

	disable_irq_nosync(irq);
	spin_lock_irqsave(&sii902xA_lock, lock_flags);
	//printk("The sii902xA interrupt handeler is working..\n");
	//printk("The most of sii902xA interrupt work will be done
	//by following tasklet..\n");

	schedule_work(sii902xAwork);

	spin_unlock_irqrestore(&sii902xA_lock, lock_flags);
	return IRQ_HANDLED;
}

// hotplug service work with timer pool / interrupt.
static void work_queue(struct work_struct *work)
{
	siHdmiTx_TPI_Poll();

	if (hotpoll_en)
		mod_timer(&x2hdmitimer, jiffies + msecs_to_jiffies(hotpoll_ms));
	else
		enable_irq(sii902xA->irq);
}

static const struct i2c_device_id hmdi_sii_id[] = {
	{ "sii902xA", 0 },
	{ "siiEDID", 0 },
	{ "siiSegEDID", 0 },
	{ "siiHDCP", 0 },
	{ },
};

static int match_id(const struct i2c_device_id *id,
		const struct i2c_client *client)
{
	if (strcmp(client->name, id->name) == 0)
		return true;

	return false;
}

static int hdmi_sii_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;

	dev_info(&client->adapter->dev, "hdmi %s i2c 0x%02X probe ...\n",
			 id->name, client->addr);
	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_SMBUS_BYTE | I2C_FUNC_I2C))
		return -ENODEV;

	if (match_id(&hmdi_sii_id[1], client)) {
		siiEDID = client;
		dev_dbg(&client->dev, "hdmi EDID: 0x%02X attached\n",
				 client->addr);
	} else if (match_id(&hmdi_sii_id[2], client)) {
		siiSegEDID = client;
		dev_dbg(&client->dev, "hdmi SegEDID: 0x%02X attached\n",
				 client->addr);
	} else if (match_id(&hmdi_sii_id[3], client)) {
		siiHDCP = client;
		dev_dbg(&client->dev, "hdmi HDCP: 0x%02X attached\n",
				 client->addr);
	} else if (match_id(&hmdi_sii_id[0], client)) {
		sii902xA = client;
		dev_dbg(&client->dev, "hdmi 902xA: 0x%02X attached\n",
				 client->addr);

		pr_debug("\n=========================\n");
		pr_debug("SiI-902xA Driver Version 1.4\n");
		pr_debug("===========================\n");

		// dts: hotpoll - 0-irq mode, >0-poll mode ms
		ret = of_property_read_u32(client->dev.of_node, "hotpoll",
				&hotpoll_ms);
		if (ret) {
			hotpoll_en = 1;
			hotpoll_ms = X2_HDMI_HOTPOLL_DEF;
		} else {
			if (hotpoll_ms) {
				hotpoll_en = 1;
				if (hotpoll_ms < X2_HDMI_HOTPOLL_MIN)
					hotpoll_ms = X2_HDMI_HOTPOLL_MIN;
			} else {
				hotpoll_en = 0;
			}
		}

		// dts: rst_pin - reset pin number.
		ret = of_property_read_u32(client->dev.of_node, "rst_pin",
				&Si9022A_rst_pin);
		if (ret) {
			dev_err(&client->dev, "Filed to get rst_pin %d\n", ret);
			return ret;
		}
		ret = gpio_request(Si9022A_rst_pin, "rst_pin");
		if (ret < 0) {
			dev_err(&client->dev, "Filed to request rst_pin-%d %d\n",
					Si9022A_rst_pin, ret);
			return ret;
		}
		dev_dbg(&client->dev, "rst_pin=%d, init out 1\n",
				 Si9022A_rst_pin);
		gpio_direction_output(Si9022A_rst_pin, 1);

		// dts: irq_pin - irq pin number(poll if irq requeset failed).
		ret = of_property_read_u32(client->dev.of_node, "irq_pin",
				&Si9022A_irq_pin);
		if (ret) {
			dev_err(&client->dev, "Filed to get irq_pin %d\n", ret);
			gpio_free(Si9022A_rst_pin);
			return ret;
		}
		ret = gpio_request(Si9022A_irq_pin, "irq_pin");
		if (ret < 0) {
			dev_err(&client->dev, "Filed to request irq_pin-%d %d\n",
					Si9022A_irq_pin, ret);
			gpio_free(Si9022A_rst_pin);
			return ret;
		}

		// dts: vmode - vmode init.
		ret = of_property_read_u32(client->dev.of_node, "vmode",
				&vmode);
		if (ret || HDMI_VMODE_SIZE <= vmode)
			vmode = X2_HDMI_VMODE_DEF;

		// dts: vformat - vformat init.
		ret = of_property_read_u32(client->dev.of_node, "vformat",
				&vformat);
		if (ret || VMD_HDMIFORMAT_HB < vformat)
			vformat = X2_HDMI_VFORMAT_DEF;

		// dts: afs - afs init.
		ret = of_property_read_u32(client->dev.of_node, "afs",
				&afs);
		if (ret || AFS_192K < afs)
			afs = X2_HDMI_AFS_DEF;

		// Initialize the registers as required. Setup firmware vars.
		dev_dbg(&client->dev, "hdmi video vmode=%d vformat=%d, audio afs=%d\n",
				 vmode, vformat, afs);
		ret = siHdmiTx_ReConfig(vmode, vformat, afs);
		if (ret < 0) {
			pr_info("bt1120 to HDMI device:sii9022a is not exist!\n");
			return ret;
		}
		// init hotplug service.
		sii902xAwork = kmalloc(sizeof(*sii902xAwork),
				GFP_ATOMIC);
		INIT_WORK(sii902xAwork, work_queue);

		if (!hotpoll_en) {
			ret = gpio_to_irq(Si9022A_irq_pin);
			if (ret >= 0) {
				client->irq = ret;
				ret = request_irq(client->irq,
						sii902xA_interrupt,
						IRQ_TYPE_LEVEL_LOW,
						client->name, client);
				if (ret == 0) {
					dev_dbg(&client->dev,
						"irq_pin=%d, irq=%d enable\n",
						 Si9022A_irq_pin, client->irq);
					enable_irq_wake(client->irq);
				} else {
					dev_warn(&client->dev,
						"Failed to request_irq %d %d\n",
						 client->irq, ret);
				}
			} else {
				dev_warn(&client->dev,
					"Failed to get irq_pin-%d irq %d\n",
					Si9022A_irq_pin, ret);
			}
			if (ret < 0) {
				ret = 0;
				hotpoll_en = 1;
				hotpoll_ms = X2_HDMI_HOTPOLL_DEF;
			}
		}
		if (hotpoll_en) {
			dev_dbg(&client->dev, "irq_pin=%d, init in for poll %dms\n",
					 Si9022A_irq_pin, hotpoll_ms);
			gpio_direction_input(Si9022A_irq_pin);
			mod_timer(&x2hdmitimer,
				jiffies + msecs_to_jiffies(hotpoll_ms));
		}
		ret = 0;

	} else {
		dev_err(&client->adapter->dev, "can not found dev_id %s matched\n",
				client->name);
		return -EIO;
	}
	return ret;
}


static int hdmi_sii_remove(struct i2c_client *client)
{
	if (!hotpoll_en)
		free_irq(sii902xA->irq, &sii902xA);
	del_timer(&x2hdmitimer);
	gpio_free(Si9022A_irq_pin);
	gpio_free(Si9022A_rst_pin);
	kfree(sii902xAwork);
	dev_info(&client->dev, "detached successfully\n");

	return 0;
}

extern SIHDMITX_CONFIG siHdmiTx;
static ssize_t sii902x_hdmi_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	int l;
	char *s = buf;
	#define SHOW_HDMI_ATTR(f, a) \
		do { l = sprintf(s, " %-20s: " f "\n", \
			__stringify(a), siHdmiTx.a); \
			s += l; \
		} while (0)

	s += sprintf(s, "sii902x %s:\n", attr->attr.name);
	SHOW_HDMI_ATTR("%d", HDMIVideoFormat);
	SHOW_HDMI_ATTR("%d", VIC);
	SHOW_HDMI_ATTR("%d", HbVIC);
	SHOW_HDMI_ATTR("%d", AspectRatio);
	SHOW_HDMI_ATTR("%d", ColorSpace);
	SHOW_HDMI_ATTR("%d", ColorDepth);
	SHOW_HDMI_ATTR("%d", Colorimetry);
	SHOW_HDMI_ATTR("%d", SyncMode);
	SHOW_HDMI_ATTR("%d", TclkSel);
	SHOW_HDMI_ATTR("%d", ThreeDStructure);
	SHOW_HDMI_ATTR("%d", ThreeDExtData);
	SHOW_HDMI_ATTR("%d", AudioMode);
	SHOW_HDMI_ATTR("%d", AudioChannels);
	SHOW_HDMI_ATTR("%d", AudioFs);
	SHOW_HDMI_ATTR("%d", AudioWordLength);
	SHOW_HDMI_ATTR("%d", AudioI2SFormat);

	return (s - buf);
}
static ssize_t sii902x_hdmi_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t n)
{
	int error = -EINVAL;
	char *tmpx, *tmpv, *tmpy;
	int done = 0, err = 0, len = 0, ret;
	unsigned long value;
	#define STORE_HDMI_ATTR(a) \
		do { if (!done && 0 == memcmp(tmpx, __stringify(a), len)) { \
			siHdmiTx.a = value;	\
			done = 1; } \
		} while (0)

	tmpx = (char *)buf;
	do {
		tmpv = memchr(tmpx, '=', n);
		if (tmpv) {
			len = tmpv - tmpx;
			while (' ' == *tmpv || '=' == *tmpv)
				tmpv++;
			tmpy = tmpv;
			while (' ' != *tmpy && ',' != *tmpy)
				tmpy++;
			*tmpy = '\0';
			tmpy++;
			ret = kstrtoul(tmpv, 0, &value);
			done = 0;
			if (ret == 0) {
				STORE_HDMI_ATTR(HDMIVideoFormat);
				STORE_HDMI_ATTR(VIC);
				STORE_HDMI_ATTR(HbVIC);
				STORE_HDMI_ATTR(AspectRatio);
				STORE_HDMI_ATTR(ColorSpace);
				STORE_HDMI_ATTR(ColorDepth);
				STORE_HDMI_ATTR(Colorimetry);
				STORE_HDMI_ATTR(SyncMode);
				STORE_HDMI_ATTR(TclkSel);
				STORE_HDMI_ATTR(ThreeDStructure);
				STORE_HDMI_ATTR(ThreeDExtData);
				STORE_HDMI_ATTR(AudioMode);
				STORE_HDMI_ATTR(AudioChannels);
				STORE_HDMI_ATTR(AudioFs);
				STORE_HDMI_ATTR(AudioWordLength);
				STORE_HDMI_ATTR(AudioI2SFormat);
			}
			if (!done)
				err++;
		} else {
			tmpy = NULL;
		}
		if (tmpy) {
			while (',' == *tmpy || ' ' == *tmpy)
				tmpy++;
			tmpx = tmpy;
		}
	} while (tmpy && (tmpx < (buf + n)));

	error = (err) ? error : 0;
	return error ? error : n;
}

extern GLOBAL_SYSTEM g_sys;
static ssize_t sii902x_sys_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	int l;
	char *s = buf;
	#define SHOW_SYS_ATTR(f, a) \
		do { l = sprintf(s, " %-20s: " f "\n", \
			__stringify(a), (g_sys.a)); \
			s += l; \
		} while (0)

	s += sprintf(s, "sii902x %s:\n", attr->attr.name);
	SHOW_SYS_ATTR("%d", txPowerState);
	SHOW_SYS_ATTR("%d", tmdsPoweredUp);
	SHOW_SYS_ATTR("%d", hdmiCableConnected);
	SHOW_SYS_ATTR("%d", dsRxPoweredUp);

	return (s - buf);
}

extern GLOBAL_HDCP g_hdcp;
static ssize_t sii902x_hdcp_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	int l;
	char *s = buf;
	#define SHOW_DHCP_ATTR(f, a) \
		do { l = sprintf(s, " %-20s: " f "\n", \
			__stringify(a), (g_hdcp.a)); \
			s += l; \
		} while (0)

	s += sprintf(s, "sii902x %s:\n", attr->attr.name);
	SHOW_DHCP_ATTR("%d", HDCP_TxSupports);
	SHOW_DHCP_ATTR("%d", HDCP_AksvValid);
	SHOW_DHCP_ATTR("%d", HDCP_Started);
	SHOW_DHCP_ATTR("%d", HDCP_LinkProtectionLevel);
	SHOW_DHCP_ATTR("%d", HDCP_Override);
	SHOW_DHCP_ATTR("%d", HDCPAuthenticated);

	return (s - buf);
}

extern GLOBAL_EDID g_edid;
static ssize_t sii902x_edid_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	int i, l;
	char *s = buf;
	#define SHOW_EDID_ATTR(f, a) \
		do { l = sprintf(s, " %-20s: " f "\n", \
				__stringify(a), g_edid.a); \
			s += l; \
		} while (0)
	#define SHOW_EDID_ATTRS(f, a, n) \
		do { s += sprintf(s, " %-20s: ", __stringify(a)); \
			for (i = 0; i < (n); i++) { \
				s += sprintf(s, f " ", \
					((byte *)(g_edid.a))[i]); \
			} \
			s += sprintf(s, "\n"); \
		} while (0)

	s += sprintf(s, "sii902x %s:\n", attr->attr.name);
	if (g_edid.edidDataValid) {
		s += sprintf(s, " %-20s:\n", "EDID-data");
		for (i = 0; i < EDID_BLOCK_SIZE; i++) {
			if (i % 16 == 0)
				s += sprintf(s, "  %02X:", i);
			s += sprintf(s, " %02X", g_EdidData[i]);
			if (i % 16 == 15)
				s += sprintf(s, "\n");
		}
		if (g_EdidData[NUM_OF_EXTEN_ADDR]) {
			s += sprintf(s, " %-20s:\n", "SegEDID-data");
			for (i = 0; i < EDID_BLOCK_SIZE; i++) {
				if (i % 16 == 0)
					s += sprintf(s, "  %02X:", i);
				s += sprintf(s, " %02X", g_SEdidData[i]);
				if (i % 16 == 15)
					s += sprintf(s, "\n");
			}
		}
	}

	SHOW_EDID_ATTR("%d", edidDataValid);
	SHOW_EDID_ATTRS("%02X", VideoDescriptor, MAX_V_DESCRIPTORS);
	SHOW_EDID_ATTRS("%02X", AudioDescriptor, MAX_A_DESCRIPTORS * 3);
	SHOW_EDID_ATTRS("%02X", SpkrAlloc, MAX_SPEAKER_CONFIGURATIONS);
	SHOW_EDID_ATTR("%d", UnderScan);
	SHOW_EDID_ATTR("%d", BasicAudio);
	SHOW_EDID_ATTR("%d", YCbCr_4_4_4);
	SHOW_EDID_ATTR("%d", YCbCr_4_2_2);
	SHOW_EDID_ATTR("%d", HDMI_Sink);
	SHOW_EDID_ATTR("%d", CEC_A_B);
	SHOW_EDID_ATTR("%d", CEC_C_D);
	SHOW_EDID_ATTR("%d", ColorimetrySupportFlags);
	SHOW_EDID_ATTR("%d", MetadataProfile);
	SHOW_EDID_ATTR("%d", _3D_Supported);

	return (s - buf);
}
static ssize_t sii902x_edid_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t n)
{
	int error = -EINVAL;

	#ifdef DEV_SUPPORT_EDID
	extern byte DoEdidRead(void);
	g_edid.edidDataValid = FALSE;
	DoEdidRead();
	#endif

	return error ? error : n;
}

static ssize_t sii902x_regs_show(struct kobject *kobj,
				 struct kobj_attribute *attr, char *buf)
{
	int i;
	char *s = buf;
	char regs[256];
	int c, f, p, l;
	#define SHOW_REGS_ATTRS(f, a, b, n) \
		do { s += sprintf(s, " %-20s: [%02X] ", __stringify(a), b); \
			for (i = 0; i < (n); i++) { \
				s += sprintf(s, f" ", regs[b + i]); \
			} \
			s += sprintf(s, "\n"); \
		} while (0)


	for (i = 0; i < 256; i += 16)
		ReadBlockTPI(i, 16, &regs[i]);

	s += sprintf(s, "sii902x %s:\n", attr->attr.name);
	for (i = 0; i < 256; i++) {
		if (i % 16 == 0)
			s += sprintf(s, " %02X:", i);
		s += sprintf(s, " %02X", regs[i]);
		if (i % 16 == 15)
			s += sprintf(s, "\n");
	}
	SHOW_REGS_ATTRS("%02X", Identification, 0x1B, 3);
	SHOW_REGS_ATTRS("%02X", Input-Bus, 0x08, 1);
	s += sprintf(s, " %-20s:     >TClkSel=%d Bus=%s Edge=%s PR=%d\n",
				 "Input-BusDespM",
				 (regs[0x08] >> 6) & 0x3,
				 (regs[0x08] & 0x20) ? "Full" : "Half",
				 (regs[0x08] & 0x10) ? "Rising" : "Falling",
				 regs[0x08] & 0xF);
	SHOW_REGS_ATTRS("%02X", Input-YcMode, 0x0B, 1);
	s += sprintf(s, " %-20s:     >Swap=%s IDDR=%s NonGap=%s YcInput=%d\n",
				 "Input-YcModeDesp",
				 (regs[0x0B] & 0x80) ? "Enable" : "Disable",
				 (regs[0x0B] & 0x40) ? "Up" : "Low",
				 (regs[0x0B] & 0x08) ? "Enable" : "Disable",
				 regs[0x0B] & 0x7);
	SHOW_REGS_ATTRS("%02X", Input-Sync, 0x60, 2);
	s += sprintf(s, " %-20s:     >Sy=%s YM=%s Iv=%s Idet=%s Pv=%s Ph=%s\n",
				 "Input-SyncDesp",
				 (regs[0x60] & 0x80) ? "Embed" : "Exten",
				 (regs[0x60] & 0x40) ? "Enable" : "Disable",
				 (regs[0x60] & 0x10) ? "Yes" : "No",
				 (regs[0x61] & 0x04) ? "Yes" : "No",
				 (regs[0x61] & 0x02) ? "Low" : "High",
				 (regs[0x61] & 0x01) ? "Low" : "High");
	SHOW_REGS_ATTRS("%02X", Input-SyncDE, 0x62, 14);
	if (regs[0x60] & 0x80) {
		c = regs[0x62] + ((regs[0x63] & 0x3) << 8);
		f = regs[0x64] + ((regs[0x65] & 0x1F) << 8);
		p = regs[0x66] + ((regs[0x67] & 0x3) << 8);
		s += sprintf(s,
				" %-20s:     >HB=%d FLD2=%d HW=%d VB=%d VW=%d\n",
				"Input-SyncDEDesp",
				c, f, p, regs[0x68] & 0x1F, regs[0x69] & 0x1F);
	} else {
		c = regs[0x62] + ((regs[0x63] & 0x3) << 8);
		f = regs[0x66] + ((regs[0x67] & 0xF) << 8);
		p = regs[0x68] + ((regs[0x69] & 0x7) << 8);
		l = regs[0x6A] + ((regs[0x6B] & 0xF) << 8);
		i = regs[0x6C] + ((regs[0x6D] & 0xF) << 8);
		s += sprintf(s,
		" %-20s:     >DLY=%d TOP=%d CNT=%d LIN=%d VRES=%d HRES=%d\n",
				"Input-SyncDEDesp",
				c, regs[0x64] & 0x3F, f, p, l, i);
	}
	SHOW_REGS_ATTRS("%02X", Audio, 0x1F, 10);
	SHOW_REGS_ATTRS("%02X", Video-Mode, 0x00, 8);
	c = regs[0x00] + (regs[0x01] << 8);
	f = regs[0x02] + (regs[0x03] << 8);
	p = regs[0x04] + (regs[0x05] << 8);
	l = regs[0x06] + (regs[0x07] << 8);
	s += sprintf(s, " %-20s:     >Clk=%d Vfreq=%d Pline=%d Lines=%d\n",
				 "Video-ModeDesp", c, f, p, l);
	SHOW_REGS_ATTRS("%02X", Video-Format, 0x09, 2);
	s += sprintf(s, " %-20s:     >ICdepth=%d IRange=%d ICspace=%d\n",
				 "Video-FormateDespI",
				 (regs[0x09] >> 6) & 0x3,
				 (regs[0x09] >> 2) & 0x3,
				 regs[0x09] & 0x3);
	s += sprintf(s, " %-20s:     >OCspace=%s ORange=%d OFormat=%d\n",
				 "Video-FormateDespO",
				 (regs[0x0A] & 0x10) ? "BT.709" : "BT.601",
				 (regs[0x0A] >> 2) & 0x3,
				 regs[0x0A] & 0x3);
	SHOW_REGS_ATTRS("%02X", Video-AviInfo, 0x0C, 14);
	s += sprintf(s, " %-20s:     >Y=%d A=%d B=%d S=%d C=%d M=%d R=%d\n",
				 "Video-AviInfo1",
				 (regs[0x0D] >> 5) & 0x3,
				 (regs[0x0D] >> 4) & 0x1,
				 (regs[0x0D] >> 2) & 0x3,
				 regs[0x0D] & 0x3,
				 (regs[0x0E] >> 6) & 0x3,
				 (regs[0x0E] >> 4) & 0x3,
				 regs[0x0E] & 0xF);
	s += sprintf(s, " %-20s:     >ITC=%d EC=%d Q=%d SC=%d\n",
				 "Video-AviInfo2",
				 (regs[0x0F] >> 7) & 0x1,
				 (regs[0x0F] >> 4) & 0x7,
				 (regs[0x0F] >> 2) & 0x3,
				 regs[0x0F] & 0x3);
	s += sprintf(s, " %-20s:     >VIC=%d YQ=%d CN=%d PR=%d\n",
				 "Video-AviInfo3",
				 regs[0x10] & 0x7F,
				 (regs[0x11] >> 6) & 0x3,
				 (regs[0x11] >> 4) & 0x3,
				 regs[0x11] & 0xF);
	c = regs[0x12] + (regs[0x13] << 8);
	f = regs[0x14] + (regs[0x15] << 8);
	p = regs[0x16] + (regs[0x17] << 8);
	l = regs[0x18] + (regs[0x19] << 8);
	s += sprintf(s, " %-20s:     >ETB=%d SBB=%d ELB=%d ERB=%d\n",
				 "Video-AviInfo4",
				 c, f, p, l);
	SHOW_REGS_ATTRS("%02X", Video-OthInfo, 0xBF, 32);
	SHOW_REGS_ATTRS("%02X", Video-YcMux, 0x60, 1);
	SHOW_REGS_ATTRS("%02X", SysCtrl, 0x1A, 1);
	SHOW_REGS_ATTRS("%02X", Interrupt, 0x3C, 2);
	SHOW_REGS_ATTRS("%02X", PowerCtrl, 0x1E, 1);
	SHOW_REGS_ATTRS("%02X", HDCP, 0x29, 18);

	return (s - buf);
}

static struct kobj_attribute hdmi_attr =
	__ATTR(hdmi, 0644, sii902x_hdmi_show, sii902x_hdmi_store);
static struct kobj_attribute sys_attr =
	__ATTR(sys, 0444, sii902x_sys_show, NULL);
static struct kobj_attribute hdcp_attr =
	__ATTR(hdcp, 0444, sii902x_hdcp_show, NULL);
static struct kobj_attribute edid_attr =
	__ATTR(edid, 0644, sii902x_edid_show, sii902x_edid_store);
static struct kobj_attribute regs_attr =
	__ATTR(regs, 0444, sii902x_regs_show, NULL);

static ssize_t sii902x_cfgs_show(struct kobject *kobj,
				struct kobj_attribute *attr, char *buf)
{
	char *s = buf;

	s += sii902x_hdmi_show(kobj, &hdmi_attr, s);
	s += sprintf(s, "\n");
	s += sii902x_sys_show(kobj, &sys_attr, s);
	s += sprintf(s, "\n");
	s += sii902x_hdcp_show(kobj, &hdcp_attr, s);
	s += sprintf(s, "\n");
	s += sii902x_edid_show(kobj, &edid_attr, s);
	s += sprintf(s, "\n");

	s += sprintf(s, "sii902x %s:\n", attr->attr.name);
	s += sprintf(s, " %-20s: %02X %02X %02X\n", "tpivmode",
				 tpivmode[0], tpivmode[1], tpivmode[2]);
	s += sprintf(s, " %-20s: %d\n", "hotpoll_en", hotpoll_en);
	s += sprintf(s, " %-20s: %d\n", "hotpoll_ms", hotpoll_ms);
	s += sprintf(s, " %-20s: %d\n", "vmode", vmode);
	s += sprintf(s, " %-20s: %d\n", "vformat", vformat);
#ifdef HDMI_AUDIO_MUTE
	s += sprintf(s, " %-20s: %d (MUTE)\n", "afs", afs);
#else
	s += sprintf(s, " %-20s: %d\n", "afs", afs);
#endif
	s += sprintf(s, " %-20s: %d\n", "reconfig", 0);

	return (s - buf);
}
static ssize_t sii902x_cfgs_store(struct kobject *kobj,
				struct kobj_attribute *attr,
				const char *buf, size_t n)
{
	int error = -EINVAL;
	char *tmpv, *tmpx, *tmpy;
	int done = 0, err = 0, len = 0, ret;
	unsigned long value;
	unsigned int reconfig = 0;
	#define STORE_CFGS_ATTR(a) \
		do { if (!done && 0 == memcmp(tmpx, __stringify(a), len)) { \
			a = value;	\
			done = 1; } \
		} while (0)

	tmpx = (char *)buf;
	do {
		tmpv = memchr(tmpx, '=', n);
		if (tmpv) {
			len = tmpv - tmpx;
			while (' ' == *tmpv || '=' == *tmpv)
				tmpv++;
			tmpy = tmpv;
			while (' ' != *tmpy && ',' != *tmpy)
				tmpy++;
			*tmpy = '\0';
			tmpy++;
			ret = kstrtoul(tmpv, 0, &value);
			done = 0;
			if (ret == 0) {
				STORE_CFGS_ATTR(hotpoll_ms);
				STORE_CFGS_ATTR(vmode);
				STORE_CFGS_ATTR(vformat);
				STORE_CFGS_ATTR(afs);
				STORE_CFGS_ATTR(reconfig);
			}
			if (!done)
				err++;
		} else {
			tmpy = NULL;
		}
		if (tmpy) {
			while (',' == *tmpy || ' ' == *tmpy)
				tmpy++;
			tmpx = tmpy;
		}
	} while (tmpy && (tmpx < (buf + n)));

	if (reconfig) {
		if (vmode >= HDMI_VMODE_SIZE) {
			pr_err("vmode=%d >= %d, error.\n",
				   vmode, HDMI_VMODE_SIZE);
			reconfig = 0;
		}
		if (vformat > VMD_HDMIFORMAT_HB) {
			pr_err("vmode=%d > %d, error.\n",
				   vformat, VMD_HDMIFORMAT_HB);
			reconfig = 0;
		}
#ifndef HDMI_AUDIO_MUTE
		if (afs > AFS_192K) {
			pr_err("afs=%d > %d, error.\n", afs, AFS_192K);
			reconfig = 0;
		}
#endif
		if (reconfig) {
			pr_info("hdmi video vmode=%d vformat=%d, audio afs=%d.\n",
					vmode, vformat, afs);
			siHdmiTx_ReConfig(vmode, vformat, afs);
		}
	}

	error = (err) ? error : 0;
	return error ? error : n;
}

static struct kobj_attribute cfgs_attr =
	__ATTR(cfgs, 0644, sii902x_cfgs_show, sii902x_cfgs_store);

static struct attribute *attributes[] = {
	&hdmi_attr.attr,
	&sys_attr.attr,
	&hdcp_attr.attr,
	&edid_attr.attr,
	&regs_attr.attr,
	&cfgs_attr.attr,
	NULL,
};

static struct attribute_group attr_group = {
	.attrs = attributes,
};

static struct i2c_driver hdmi_sii_i2c_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
	},
	.probe = hdmi_sii_probe,
	.remove =  hdmi_sii_remove,
	.id_table = hmdi_sii_id,
};

struct kobject *sii902x_kobj;
static int __init hdmi_sii_init(void)
{
	int ret;

	pr_info("hdmi: add %s driver\n", DEVICE_NAME);
	ret = i2c_add_driver(&hdmi_sii_i2c_driver);
	if (ret)
		pr_err("%s: failed to add sii902xA i2c driver\n",
				__func__);

	sii902x_kobj = kobject_create_and_add(DEVICE_NAME, NULL);
	if (!sii902x_kobj)
		return -ENOMEM;

	return sysfs_create_group(sii902x_kobj, &attr_group);
}

static void __exit hdmi_sii_exit(void)
{
	i2c_del_driver(&hdmi_sii_i2c_driver);

	if (sii902x_kobj) {
		sysfs_remove_group(sii902x_kobj, &attr_group);
		kobject_del(sii902x_kobj);
	}

	pr_info("Module is leaving..\n");

}

late_initcall(hdmi_sii_init);
module_exit(hdmi_sii_exit);
MODULE_VERSION("1.4");
MODULE_AUTHOR("Silicon image SZ office, Inc.");
MODULE_DESCRIPTION("sii902xA HDMI driver");
MODULE_ALIAS("platform:hdmi-sii902xA");
