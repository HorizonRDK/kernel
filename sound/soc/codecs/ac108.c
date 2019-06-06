/*
 * ac108.c  --  ac108 ALSA Soc Audio driver
 *
 * Author: panjunwen
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

#include "ac108.h"

#define AC108_DEBUG_EN			1

#if AC108_DEBUG_EN
#define AC108_DEBUG(...)	printk(__VA_ARGS__)
#else
#define AC108_DEBUG(...)
#endif


//test enable config
#define AC108_DAPM_TEST_EN			0
#define AC108_CODEC_RW_TEST_EN		1


//AC108 config
#define AC108_CHANNELS_MAX		4		//range[1, 16]
#define AC108_SLOT_WIDTH		32
#define AC108_PGA_GAIN			0x3c
#define AC108_DMIC_EN			0
#define AC108_ENCODING_EN		0
#define AC108_ENCODING_CH_NUMS	8
#define AC108_LRCK_PERIOD		(AC108_ENCODING_EN ? 32 : 256)

#define AC108_MATCH_DTS_EN		0
#define AC108_I2C_BUS_NUM		1
#define AC108_REGULATOR_NAME	"voltage_enable"

#define AC108_RATES		SNDRV_PCM_RATE_8000_96000
#define AC108_FORMATS	SNDRV_PCM_FMTBIT_S16_LE


struct i2c_client *i2c_clt[(AC108_CHANNELS_MAX+3)/4];
int	regulator_en;
unsigned int ac108_bclk_div_val;



struct voltage_supply {
	struct regulator *vcc3v3;
};

struct ac108_priv {
	struct i2c_client *i2c;
	struct snd_soc_codec *codec;
	struct voltage_supply vol_supply;
};

static const struct regmap_config ac108_regmap_config = {
	.reg_bits = 8,	//Number of bits in a register address
	.val_bits = 8,	//Number of bits in a register value
};


struct real_val_to_reg_val {
	unsigned int real_val;
	unsigned int reg_val;
};

struct pll_div {
	u32 freq_in;
	u32 freq_out;
	u32 m1;
	u32 m2;
	u32 n;
	u32 k1;
	u32 k2;
};


//ac108 dapm routes


static int ac108_read(u8 reg, u8 *rt_value, struct i2c_client *client)
{
	return 0;
}


static int ac108_set_sysclk(struct snd_soc_dai *dai, int clk_id,
	unsigned int freq, int dir)
{
	return 0;
}

static int ac108_set_pll(struct snd_soc_dai *dai, int pll_id, int source,
	unsigned int freq_in, unsigned int freq_out)
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


/*** define  ac108  dai_ops  struct ***/
static const struct snd_soc_dai_ops ac108_dai_ops = {
	/*DAI clocking configuration*/
	.set_sysclk = ac108_set_sysclk,
	.set_pll = ac108_set_pll,
	.set_clkdiv = ac108_set_clkdiv,

	/*ALSA PCM audio operations*/
	.hw_params = ac108_hw_params,
	.trigger = ac108_trigger,
	//.hw_free = ac108_hw_free,

	/*DAI format configuration*/
	.set_fmt = ac108_set_fmt,
};

/*** define  ac108  dai_driver struct ***/
static struct snd_soc_dai_driver ac108_dai0 = {
	.name = "ac108-pcm0",
	.capture = {
		.stream_name = "Capture",
		.channels_min = 1,
		.channels_max = AC108_CHANNELS_MAX,
		.rates = AC108_RATES,
		.formats = AC108_FORMATS,
	},
	.ops = &ac108_dai_ops,
};

static struct snd_soc_dai_driver *ac108_dai[] = {
#if AC108_CHANNELS_MAX > 0
	&ac108_dai0,
#endif
};

static int ac108_probe(struct snd_soc_codec *codec)
{
	return 0;
}

static int ac108_remove(struct snd_soc_codec *codec)
{
	return 0;
}


#ifdef CONFIG_PM
static int ac108_suspend(struct snd_soc_codec *codec)
{
	return 0;
}

static int ac108_resume(struct snd_soc_codec *codec)
{
	return 0;
}

#else

#define ac108_suspend	NULL
#define ac108_resume	NULL

#endif

static unsigned int ac108_codec_read(struct snd_soc_codec *codec,
	unsigned int reg)
{
	//AC108_DEBUG("\n--->%s\n",__FUNCTION__);
	u8 val_r = 0;
	struct ac108_priv *ac108 = dev_get_drvdata(codec->dev);

	ac108_read(reg, &val_r, ac108->i2c);
	return val_r;
}
static int ac108_multi_chips_write(u8 reg, unsigned char value)
{
	int ret = 0;
	return ret;
}

static int ac108_codec_write(struct snd_soc_codec *codec,
	unsigned int reg, unsigned int value)
{
	//AC108_DEBUG("\n--->%s\n",__FUNCTION__);
	ac108_multi_chips_write(reg, value);
	return 0;
}


/*** define  ac108  codec_driver struct ***/
static const struct snd_soc_codec_driver ac108_soc_codec_driver = {
	.probe = ac108_probe,
	.remove = ac108_remove,
	.suspend = ac108_suspend,
	.resume = ac108_resume,

#if AC108_CODEC_RW_TEST_EN
	.read = ac108_codec_read,
	.write = ac108_codec_write,
#endif
#if AC108_DAPM_TEST_EN
	.component_driver = {
		.controls = ac108_controls,
		.num_controls = ARRAY_SIZE(ac108_controls),
		},

#endif
};


static ssize_t ac108_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t ac108_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return 0;
}

static DEVICE_ATTR(ac108, 0644, ac108_show, ac108_store);

static struct attribute *ac108_debug_attrs[] = {
	&dev_attr_ac108.attr,
	NULL,
};

static struct attribute_group ac108_debug_attr_group = {
	.name   = "ac108_debug",
	.attrs  = ac108_debug_attrs,
};


static int ac108_i2c_probe(struct i2c_client *i2c,
	const struct i2c_device_id *i2c_id)
{
	struct ac108_priv *ac108;
	int ret = 0;

	ac108 = devm_kzalloc(&i2c->dev, sizeof(struct ac108_priv), GFP_KERNEL);
	if (ac108 == NULL)
		return -ENOMEM;

	ac108->i2c = i2c;
	dev_set_drvdata(&i2c->dev, ac108);
	if (i2c_id->driver_data < (AC108_CHANNELS_MAX+3)/4) {
		i2c_clt[i2c_id->driver_data] = i2c;
		ret = snd_soc_register_codec(&i2c->dev,
			&ac108_soc_codec_driver,
				ac108_dai[i2c_id->driver_data], 1);
		if (ret < 0)
			dev_err(&i2c->dev, "Failed to register ac108 codec: %d\n", ret);
	} else {
		pr_err("The wrong i2c_id number :%d\n",
			(int)(i2c_id->driver_data));
	}

	pr_err("ac108 init ret is %d\n", ret);

	ret = sysfs_create_group(&i2c->dev.kobj, &ac108_debug_attr_group);
	//ac108_hw_init(i2c_clt[0]);
	return ret;
}

static int ac108_i2c_remove(struct i2c_client *i2c)
{
	snd_soc_unregister_codec(&i2c->dev);
	return 0;
}


static const struct i2c_device_id ac108_i2c_id[] = {
#if AC108_CHANNELS_MAX > 0
	{ "ac108_0", 0 },//ac108_0
#endif

#if AC108_CHANNELS_MAX > 4
	{ "ac108_1", 1 },//ac108_1
#endif

#if AC108_CHANNELS_MAX > 8
	{ "ac108_2", 2 },//ac108_2
#endif

#if AC108_CHANNELS_MAX > 12
	{ "ac108_3", 3 },//ac108_3
#endif
	{ }
};
MODULE_DEVICE_TABLE(i2c, ac108_i2c_id);

static const struct of_device_id ac108_dt_ids[] = {
#if AC108_CHANNELS_MAX > 0
	{ .compatible = "ac108_0", .data = NULL},//ac108_0
#endif
};
MODULE_DEVICE_TABLE(of, ac108_dt_ids);

static struct i2c_driver ac108_i2c_driver = {
	.class = I2C_CLASS_HWMON,
	.driver = {
		.name = "ac108",
		.owner = THIS_MODULE,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(ac108_dt_ids),
#endif

	},
	.probe = ac108_i2c_probe,
	.remove = ac108_i2c_remove,
	.id_table = ac108_i2c_id,
};
module_i2c_driver(ac108_i2c_driver);


MODULE_DESCRIPTION("ASoC ac108 codec driver");
MODULE_AUTHOR("panjunwen");
MODULE_LICENSE("GPL");

