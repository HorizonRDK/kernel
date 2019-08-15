/*
 *    COPYRIGHT NOTICE
 *   Copyright 2019 Horizon Robotics, Inc.
 *    All rights reserved.
 */

/*
 * ti1864.c  --  ti1864 ALSA Soc Audio driver
 *
 * Author: horizon
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/firmware.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/debugfs.h>
#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/initval.h>


#include <linux/of.h>
#include <sound/tlv.h>
#include <linux/regulator/consumer.h>
#include <linux/io.h>

#define AC108_CHANNELS_MAX		4        //  range[1, 16]
#define AC108_SLOT_WIDTH		32
#define AC108_PGA_GAIN			0x3c	//  range[0, 0x3f], eg: 0->0dB, 0x3d->30.5dB
#define AC108_DMIC_EN			0		//  0:ADC	 1:DMIC
#define AC108_ENCODING_EN		0		//  TX Encoding mode enable
#define AC108_ENCODING_CH_NUMS 	8
#define AC108_LRCK_PERIOD		(AC108_ENCODING_EN ? 32 : 256)	//  range[1, 1024]

#define AC108_MATCH_DTS_EN		0
#define AC108_I2C_BUS_NUM 		1
#define AC108_REGULATOR_NAME	"voltage_enable"

#define AC108_RATES 			(SNDRV_PCM_RATE_8000_96000 | SNDRV_PCM_RATE_KNOT)
#define AC108_FORMATS			(SNDRV_PCM_FMTBIT_S16_LE | \
								SNDRV_PCM_FMTBIT_S20_3LE | \
								SNDRV_PCM_FMTBIT_S24_LE | \
								SNDRV_PCM_FMTBIT_S32_LE)

#define TI1864_I2C_ADDR 0x4a

struct i2c_adapter *i2c_adap = NULL;

static void ac108_hw_init(struct i2c_client *i2c)
{
	/*** Chip reset ***/
	int ret = 0;
}


static int ac108_set_sysclk(struct snd_soc_dai *dai,
	int clk_id, unsigned int freq, int dir)
{
	return 0;
}

static int ac108_set_pll(struct snd_soc_dai *dai, int pll_id,
	int source, unsigned int freq_in, unsigned int freq_out)
{
	return 0;
}


static int ac108_set_clkdiv(struct snd_soc_dai *dai, int div_id, int div)
{
	return 0;
}

static int ac108_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	return 0;
}

static int ac108_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	return 0;
}

static int ac108_trigger(struct snd_pcm_substream *substream,
	int cmd, struct snd_soc_dai *dai)
{
	return 0;
}

static int ac108_hw_free(struct snd_pcm_substream *substream,
	struct snd_soc_dai *dai)
{
	return 0;
}


/*** define  ac108  dai_ops  struct ***/
static const struct snd_soc_dai_ops ac108_dai_ops = {
	/*DAI clocking configuration*/
	.set_sysclk = ac108_set_sysclk,
	.set_pll = ac108_set_pll,
	.set_clkdiv = ac108_set_clkdiv,

	/*ALSA PCM audio operations*/
	.hw_params = ac108_hw_params,
	.trigger = ac108_trigger,
	// .hw_free = ac108_hw_free,

	/*DAI format configuration*/
	.set_fmt = ac108_set_fmt,
};

/*** define  ac108  dai_driver struct ***/
static const struct snd_soc_dai_driver ac108_dai = {
	.name = "ti1864-pcm",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = AC108_CHANNELS_MAX,
		.rates = AC108_RATES,
		.formats = AC108_FORMATS,
	},
	.ops = &ac108_dai_ops,
};

static int ac108_probe(struct snd_soc_codec *codec)
{
	return 0;
}

static int ac108_remove(struct snd_soc_codec *codec)
{
	return 0;
}

static int ac108_suspend(struct snd_soc_codec *codec)
{
	return 0;
}

static int ac108_resume(struct snd_soc_codec *codec)
{
	return 0;
}


/*** define  ac108  codec_driver struct ***/
static const struct snd_soc_codec_driver ac108_soc_codec_driver = {
	.probe = ac108_probe,
	.remove = ac108_remove,
	.suspend = ac108_suspend,
	.resume = ac108_resume,
};

static int ti1864_i2c_write(unsigned char ucAddress, unsigned char ucReg,
	unsigned char ucValue )
{
	int ret = 0;
	size_t count;
	uint8_t out_buf[3];
	struct i2c_msg msg;
	unsigned int i;
	memset(out_buf, 0x00, 3);

  out_buf[0] = ucAddress;
  out_buf[1] = ucReg;
  out_buf[2] = ucValue;

	msg.addr = out_buf[0]; // slave address.
	msg.flags = !I2C_M_RD; // not read, is write.
	msg.len = 2; // address, reg address, reg val.
	msg.buf = out_buf + 1;

	if (!i2c_adap) {
		printk(KERN_ERR "i2c_adap not found\n");
		ret = -EFAULT;
		goto error;
	}

	printk("i2c address: 0x%x, reg addr: 0x%x, reg val: 0x%x\n",
		msg.addr, out_buf[1], out_buf[2]);
	if (i2c_transfer(i2c_adap, &msg, 1) != 1) {
		printk(KERN_ERR "i2c write failed\n");
		ret = -EINVAL;
		goto error;
	} else {
		printk(KERN_NOTICE "i2c write OK\n");
	}
error:
	return ret;
}

static int ti1864_i2c_probe(struct i2c_client *i2c,
	const struct i2c_device_id *i2c_id)
{
  int ret = 0;
  printk("-----------------ti1864_i2c_probe--------------------\n");

  i2c_adap = i2c_get_adapter(0);
	if (IS_ERR(i2c_adap)) {
		printk(KERN_ERR  "[%s:%d] get i2c adapter error\n",
			__func__, __LINE__);
		ret = PTR_ERR(i2c_adap);
    return ret;
	}

#if 1
//  ret += ti1864_i2c_write(TI1864_I2C_ADDR, 0x0b, 0xDF);
//  ret += ti1864_i2c_write(TI1864_I2C_ADDR, 0x0c, 0x01);

  /*not 0x2F, block other reg*/
//  ret += ti1864_i2c_write(TI1864_I2C_ADDR, 0x20, 0x2E);

  ret += ti1864_i2c_write(TI1864_I2C_ADDR, 0x00, 0x00);
  ret += ti1864_i2c_write(TI1864_I2C_ADDR, 0x01, 0x00);
  ret += ti1864_i2c_write(TI1864_I2C_ADDR, 0x02, 0x00);
  ret += ti1864_i2c_write(TI1864_I2C_ADDR, 0x03, 0x00);
  ret += ti1864_i2c_write(TI1864_I2C_ADDR, 0x04, 0x00);
  ret += ti1864_i2c_write(TI1864_I2C_ADDR, 0x05, 0x87);
  ret += ti1864_i2c_write(TI1864_I2C_ADDR, 0x06, 0x41);
  ret += ti1864_i2c_write(TI1864_I2C_ADDR, 0x07, 0x41);
  ret += ti1864_i2c_write(TI1864_I2C_ADDR, 0x08, 0x44);
  ret += ti1864_i2c_write(TI1864_I2C_ADDR, 0x09, 0x44);
  ret += ti1864_i2c_write(TI1864_I2C_ADDR, 0x0a, 0x00);
  ret += ti1864_i2c_write(TI1864_I2C_ADDR, 0x0b, 0xdf);
  ret += ti1864_i2c_write(TI1864_I2C_ADDR, 0x0c, 0x01);
  ret += ti1864_i2c_write(TI1864_I2C_ADDR, 0x10, 0x00);
  ret += ti1864_i2c_write(TI1864_I2C_ADDR, 0x11, 0x50);
  ret += ti1864_i2c_write(TI1864_I2C_ADDR, 0x12, 0x00);
  ret += ti1864_i2c_write(TI1864_I2C_ADDR, 0x13, 0x40);
  ret += ti1864_i2c_write(TI1864_I2C_ADDR, 0x20, 0x61);

#endif

  printk("-----------------i2c dev %s--------------------\n",
	dev_name(&i2c->dev));

  ret = snd_soc_register_codec(&i2c->dev,
	&ac108_soc_codec_driver, &ac108_dai, 1);
	if (ret < 0) {
		dev_err(&i2c->dev, "Failed to register ac108 codec: %d\n", ret);
  }
  printk("-----------------ti1864_i2c_probe end--------------------\n");
	return ret;
}

static int ti1864_i2c_remove(struct i2c_client *i2c)
{
	snd_soc_unregister_codec(&i2c->dev);
	return 0;
}

static struct of_device_id ti1864_match_table[] = {
        { .compatible = "ti1864", },
        { },
};


static struct i2c_driver ti1864_i2c_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = "ti1864",
		.owner = THIS_MODULE,
    .of_match_table = of_match_ptr(ti1864_match_table),
	},
	.probe = ti1864_i2c_probe,
	.remove = ti1864_i2c_remove,
};

module_i2c_driver(ti1864_i2c_driver);

MODULE_DESCRIPTION("ASoC ti1864 codec driver");
MODULE_AUTHOR("horizon");
MODULE_LICENSE("GPL");
