/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/iio/iio.h>
#include <linux/iio/driver.h>
#include <linux/iio/machine.h>
#include "hobot-pvt.h"

struct hobot_vm_data_t {
    struct device *dev;
    void __iomem *reg_base;
    void __iomem *efuse_base;
    int vm_k3;
    int vm_n0;
    int vm_mode;
    int num_channels;
    int last_smpl[PVT_VM_NUM];
    struct iio_chan_spec *channels;
    void *pvt_data;
};

static inline u32 pvt_reg_rd(struct hobot_vm_data_t *dev, u32 reg)
{
    return ioread32(dev->reg_base + reg);
}

static inline void pvt_reg_wr(struct hobot_vm_data_t *dev, u32 reg, u32 val)
{
    iowrite32(val, dev->reg_base + reg);
}

static int hobot_vm_read_channel(struct hobot_vm_data_t *info, int channel)
{
    u32 val = 0;

    pvt_reg_wr(info, VM_CMN_SDIF_ADDR, (u32)(0x89001000 | (channel << 16)));
    msleep(1);
    if (!pvt_reg_rd(info, VM_CMN_SDIF_STATUS_ADDR)) {
        dev_warn(info->dev, "%s %d TIMEOUT", __func__, __LINE__);
        return -ETIMEDOUT;
    }

    pvt_reg_wr(info, VM_CMN_SDIF_ADDR, 0x8d000040);
    msleep(1);
    if (!pvt_reg_rd(info, VM_CMN_SDIF_STATUS_ADDR)) {
        dev_warn(info->dev, "%s %d TIMEOUT", __func__, __LINE__);
        return -ETIMEDOUT;
    }

    pvt_reg_wr(info, VM_CMN_SDIF_ADDR, 0x8a000000);
    msleep(1);
    if (!pvt_reg_rd(info, VM_CMN_SDIF_STATUS_ADDR)) {
        dev_warn(info->dev, "%s %d TIMEOUT", __func__, __LINE__);
        return -ETIMEDOUT;
    }

    pvt_reg_wr(info, VM_CMN_SDIF_ADDR, 0x8c00ffff);
    msleep(1);
    if (!pvt_reg_rd(info, VM_CMN_SDIF_STATUS_ADDR)) {
        dev_warn(info->dev, "%s %d TIMEOUT", __func__, __LINE__);
        return -ETIMEDOUT;
    }

    pvt_reg_wr(info, VM_CMN_SDIF_ADDR, 0x88000508);
    msleep(1);
    if (!pvt_reg_rd(info, VM_CMN_SDIF_STATUS_ADDR)) {
        dev_warn(info->dev, "%s %d TIMEOUT", __func__, __LINE__);
        return -ETIMEDOUT;
    }

    val = pvt_reg_rd(info, VM_CH_n_SDIF_DATA_ADDR + channel * 0x4);
    info->last_smpl[channel] = val;
    return 0;
}

static int hobot_vm_read_raw(struct iio_dev *indio_dev,
                struct iio_chan_spec const *chan,
                int *val, int *val2, long mask) {
    struct hobot_vm_data_t *info = iio_priv(indio_dev);
    int volt = 0;
    s64 k = 0, offset = 0;

    switch (mask) {
    case IIO_CHAN_INFO_RAW:
        mutex_lock(&indio_dev->mlock);
        if (chan->channel >= 0 && chan->channel < PVT_VM_NUM) {
            if (hobot_vm_read_channel(info, chan->channel)) {
                mutex_unlock(&indio_dev->mlock);
                return -ETIMEDOUT;
            }
            if (info->vm_mode == 0) {
                k = (s64)info->vm_k3 * 25;
                offset = info->vm_n0 * k;
                volt = (int)((k * (s64)(info->last_smpl[chan->channel]) \
                       - offset) >> 12);
            } else {
                /* (0.24 * (6 * sapmle - 16387)) * 1000 / 16384 */
                volt = (90000 * info->last_smpl[chan->channel] - 245805000) \
                        >> 10;
            }
            *val = volt;
            dev_dbg(info->dev, "%s reg_val=%d\n", __func__,
                    info->last_smpl[chan->channel]);
            dev_dbg(info->dev, "%s reg_volt=%d\n", __func__, volt);
        } else {
            mutex_unlock(&indio_dev->mlock);
            return -ENAVAIL;
        }
        mutex_unlock(&indio_dev->mlock);
        return IIO_VAL_INT;
    default:
        return -EINVAL;
    }
}

static const struct iio_info hobot_vm_iio_info = {
    .read_raw = hobot_vm_read_raw,
};

#define VM_CHANNEL(_index, _id) {       \
    .type = IIO_VOLTAGE,		        \
    .indexed = 1,						\
    .channel = _index,					\
    .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),		    \
    .info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),	\
    .datasheet_name = _id,					\
}

static struct iio_chan_spec hobot_vm_iio_channels[] = {
    VM_CHANNEL(0,  "Lite adc"),
    VM_CHANNEL(1,  "VDD CORE A0"),
    VM_CHANNEL(2,  "VDD CNN0"),
    VM_CHANNEL(3,  "VDD CNN0"),
    VM_CHANNEL(4,  "VDD CNN0"),
    VM_CHANNEL(5,  "VDD CPU OFF 1"),
    VM_CHANNEL(6,  "VDD CPU"),
    VM_CHANNEL(7,  "VDD CPU OFF 0"),
    VM_CHANNEL(8,  "VDD DDR"),
    VM_CHANNEL(9,  "VDD DDR"),
    VM_CHANNEL(10, "VDD CNN 1"),
    VM_CHANNEL(11, "VDD CNN 1"),
    VM_CHANNEL(12, "VDD CORE PD"),
    VM_CHANNEL(13, "VDD CORE PD"),
    VM_CHANNEL(14, "VDD CORE PD"),
    VM_CHANNEL(15, "VDD CORE PD"),
};

#define HB_VM_MAP(_consumer_dev_name, _consumer_channel, _adc_channel_label) \
{ \
    .consumer_dev_name = _consumer_dev_name,	\
	.consumer_channel = _consumer_channel,		\
	.adc_channel_label = _adc_channel_label,	\
}

/* default maps used by iio consumer (lp8788-charger driver) */
static struct iio_map hb_vm_default_iio_maps[] = {
	HB_VM_MAP("hobot-pvt-vm", "hobot-vm-ch0",  "Lite adc"),
	HB_VM_MAP("hobot-pvt-vm", "hobot-vm-ch1",  "VDD CORE A0"),
	HB_VM_MAP("hobot-pvt-vm", "hobot-vm-ch2",  "VDD CNN0"),
	HB_VM_MAP("hobot-pvt-vm", "hobot-vm-ch3",  "VDD CNN0"),
	HB_VM_MAP("hobot-pvt-vm", "hobot-vm-ch4",  "VDD CNN0"),
	HB_VM_MAP("hobot-pvt-vm", "hobot-vm-ch5",  "VDD CPU OFF 1"),
	HB_VM_MAP("hobot-pvt-vm", "hobot-vm-ch6",  "VDD CPU"),
	HB_VM_MAP("hobot-pvt-vm", "hobot-vm-ch7",  "VDD CPU OFF 0"),
	HB_VM_MAP("hobot-pvt-vm", "hobot-vm-ch8",  "VDD DDR"),
	HB_VM_MAP("hobot-pvt-vm", "hobot-vm-ch9",  "VDD DDR"),
	HB_VM_MAP("hobot-pvt-vm", "hobot-vm-ch10", "VDD CNN 1"),
	HB_VM_MAP("hobot-pvt-vm", "hobot-vm-ch11", "VDD CNN 1"),
	HB_VM_MAP("hobot-pvt-vm", "hobot-vm-ch12", "VDD CORE PD"),
	HB_VM_MAP("hobot-pvt-vm", "hobot-vm-ch13", "VDD CORE PD"),
	HB_VM_MAP("hobot-pvt-vm", "hobot-vm-ch14", "VDD CORE PD"),
	HB_VM_MAP("hobot-pvt-vm", "hobot-vm-ch15", "VDD CORE PD"),
	{},
};


static int hobot_vm_init_hw(struct hobot_vm_data_t *info)
{
    u32 val;
    int cnt = 0;

    /* Configure Clock Synthesizers from pvt spec, 240MHz to 4MHz  */
    pvt_reg_wr(info, VM_CMN_CLK_SYNTH_ADDR, 0x01011D1D);

    dev_dbg(info->dev, "VM_CLK_SYN: %08x\n",
                pvt_reg_rd(info, VM_CMN_CLK_SYNTH_ADDR));
    dev_dbg(info->dev, "VM_SDIF_STATUS: 0x%08x\n",
                pvt_reg_rd(info, VM_CMN_SDIF_STATUS_ADDR));

    /* enable continue mode, 488 samples/s */
    val = pvt_reg_rd(info, VM_CMN_SDIF_STATUS_ADDR);
    while (val & (PVT_SDIF_BUSY_BIT | PVT_SDIF_LOCK_BIT)) {
        val = pvt_reg_rd(info, VM_CMN_SDIF_STATUS_ADDR);
        if (cnt++ > 100) {
            dev_warn(info->dev, "SDIF status busy or lock, status 0x%08x", val);
            break;
        }
        udelay(10);
    }

    /* set temperature sensor mode via SDIF */
    pvt_reg_wr(info, VM_CMN_SDIF_ADDR, 0x89001000 | (0x1 << 16));
    udelay(10);

    /* read back the temperature sensor mode register */
    pvt_reg_wr(info, VM_CMN_SDIF_ADDR, 0x81000000);
    udelay(10);

    val = pvt_reg_rd(info, VM_SDIF_RDATA_ADDR);
    dev_dbg(info->dev, "get ip_cfg: VM_SDIF_RDATA_ADDR: 0x%08x\n", val);

    /* set ip_ctrl, 0x88000104 is self-clear run once,
     * 0x88000108 run continuously
     */
    pvt_reg_wr(info, VM_CMN_SDIF_ADDR, 0x88000508);
    return 0;
}

int hobot_vm_probe(struct device *dev, void __iomem *reg_base,
            void __iomem *efuse_base)
{
    struct hobot_vm_data_t *info = NULL;
    struct iio_dev *indio_dev = NULL;
    struct device_node *np = dev->of_node;
    struct device_node *of_node_vm = NULL;
    u32 val;
    int ret;

    of_node_vm = of_get_child_by_name(np, "voltage_monitor");
    if (!of_node_vm) {
        dev_err(dev, "voltage monitor is disabled");
        return -ENODEV;
    }

    indio_dev = devm_iio_device_alloc(dev, sizeof(*info));
    if (!indio_dev) {
        dev_err(dev, "failed allocating iio device\n");
        return -ENOMEM;
    }

    info = iio_priv(indio_dev);
    info->dev = dev;
    info->reg_base = reg_base;
    info->efuse_base = efuse_base;


    info->channels = hobot_vm_iio_channels;
    info->num_channels = ARRAY_SIZE(hobot_vm_iio_channels);

    /* use uncalibrated mode by default */
    info->vm_mode = 0;

    val = ioread32(info->efuse_base + 4 * 0x4);
    info->vm_k3 = ((val & PVT_VM_K3_MASK) >> 16);
    info->vm_n0 = (val & PVT_VM_N0_MASK);
    if (info->vm_k3 > 0x4000 || info->vm_k3 < 0x3000 ||
        info->vm_n0 > 0xb00 || info->vm_n0 < 0xa00 ) {
        dev_info(info->dev, "VM Calibrated data Invalid\n");
        info->vm_mode = 1;
    }
    if (info->vm_mode == 0) {
        dev_info(info->dev, "VM Calibrated data detected\n");
        dev_dbg(info->dev, "val:%08x, k3:%d, n0:%d， mode:%d\n",
                    val, info->vm_k3, info->vm_n0, info->vm_mode);
    }

    indio_dev->name = dev_name(info->dev);
    indio_dev->dev.parent = info->dev;
    indio_dev->dev.of_node = of_node_vm;
    indio_dev->info = &hobot_vm_iio_info;
    indio_dev->modes = INDIO_DIRECT_MODE;

    indio_dev->channels = info->channels;
    indio_dev->num_channels = info->num_channels;

	ret = iio_map_array_register(indio_dev, hb_vm_default_iio_maps);
	if (ret) {
		dev_err(info->dev, "iio map_array register failed\n");
        return ret;
	}
    ret = iio_device_register(indio_dev);
    if (ret) {
        dev_err(info->dev, "iio device register failed\n");
        return ret;
    }

    hobot_vm_init_hw(info);
    return 0;
}
