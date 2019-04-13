/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2018 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/
#include <linux/module.h>

#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/soc-dapm.h>
#include <linux/io.h>

#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pm_runtime.h>

#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/dmaengine_pcm.h>

#include "x2-i2s.h"

struct s2_snd_config_s {
	u32 i2s_mode;
	u32 master;
	u32 channel_max;

};
static struct s2_snd_config_s x2_snd_config[2] = {

	{
		.i2s_mode = 0,
		.master = 0,
		.channel_max = 0,
	},
	{
		.i2s_mode = 0,
		.master = 0,
		.channel_max = 0,
	}
};



static int x2_snd0_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{

	int ret = 0;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned long sample_rate = params_rate(params);

	/* config codec */
	ret =
	    snd_soc_dai_set_fmt(codec_dai,
				SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0) {
		pr_err("%s, line:%d\n", __func__, __LINE__);
		return ret;
	}

	ret = snd_soc_dai_set_fmt(cpu_dai, (x2_snd_config[0].i2s_mode |
		(x2_snd_config[0].master << 12)));
	if (ret < 0) {
		pr_err("%s, line:%d\n", __func__, __LINE__);
		return ret;
	}

	ret = snd_soc_dai_set_clkdiv(codec_dai, 0, sample_rate);
	if (ret < 0) {
		pr_err("%s, line:%d\n", __func__, __LINE__);
		return ret;

	}
	/* below div is getted by sample_rate */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, 0, sample_rate);
	if (ret < 0) {
		pr_err("%s, line:%d\n", __func__, __LINE__);
		return ret;
	}

	return 0;

}

static int x2_snd1_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params)

{

		int ret = 0;
		struct snd_soc_pcm_runtime *rtd = substream->private_data;
		struct snd_soc_dai *codec_dai = rtd->codec_dai;
		struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
		unsigned long sample_rate = params_rate(params);

					 /* config codec */
		ret =
			snd_soc_dai_set_fmt(codec_dai,
				SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
				SND_SOC_DAIFMT_CBS_CFS);
			if (ret < 0) {
				pr_err("%s, line:%d\n", __func__, __LINE__);
				return ret;
			}

		ret = snd_soc_dai_set_fmt(cpu_dai, (x2_snd_config[1].i2s_mode
			|(x2_snd_config[1].master << 12)));
		if (ret < 0) {
			pr_err("%s, line:%d\n", __func__, __LINE__);
			return ret;
		}

		ret = snd_soc_dai_set_clkdiv(codec_dai, 0, sample_rate);
		if (ret < 0) {
			pr_err("%s, line:%d\n", __func__, __LINE__);
			return ret;

		}
			/* below div is getted by sample_rate */
		ret = snd_soc_dai_set_clkdiv(cpu_dai, 0, sample_rate);
		if (ret < 0) {
			pr_err("%s, line:%d\n", __func__, __LINE__);
				return ret;
		}

			 return 0;

}



static int x2_snd0_init(struct snd_soc_pcm_runtime *rtd)
{
	/* struct snd_soc_codec *codec = rtd->codec; */
	/* struct snd_soc_card *card = rtd->card; */
	return 0;
}
static int x2_snd1_init(struct snd_soc_pcm_runtime *rtd)
{

	return 0;
}

static struct snd_soc_ops x2_snd0_ops = {
	.hw_params = x2_snd0_hw_params,
};

static struct snd_soc_ops x2_snd1_ops = {
	.hw_params = x2_snd1_hw_params,
};

static struct snd_soc_dai_link x2_snd0_dai_link = {
	.name = "x2dailink0",
	.stream_name = "x2-stream",
	.cpu_dai_name = "x2-i2s0",

	.codec_dai_name = "snd-soc-dummy-dai",
	 .codec_name   = "snd-soc-dummy",
	.init = x2_snd0_init,
	.platform_name = "a5007000.idma",
	.ops = &x2_snd0_ops,
};

static struct snd_soc_dai_link x2_snd1_dai_link = {
	.name = "x2dailink1",
	.stream_name = "x2-stream",
	.cpu_dai_name = "x2-i2s1",

	 .codec_dai_name = "snd-soc-dummy-dai",
	 .codec_name   = "snd-soc-dummy",
	.init = x2_snd1_init,
	.platform_name = "a5008000.idma",
	.ops = &x2_snd1_ops,
};

static struct snd_soc_card x2_soc_snd[2] = {

	{
		.name = "x2snd0",
		.owner = THIS_MODULE,
		.dai_link = &x2_snd0_dai_link,
		.num_links = 1,
	},
	{
		.name = "x2snd1",
		.owner = THIS_MODULE,
		.dai_link = &x2_snd1_dai_link,
		.num_links = 1,
	}
};



static int x2_snd_get_dt_data(struct platform_device *pdev, int id)
{
	int rc;

	rc = of_property_read_u32(pdev->dev.of_node, "i2s_mode",
				  &x2_snd_config[id].i2s_mode);
	if (rc < 0) {
		pr_err("failed: i2s_mode rc %d", rc);
		return rc;
	}
	pr_err("current snd card id is %d, i2s_mode is %d\n",
		id, x2_snd_config[id].i2s_mode);
	rc = of_property_read_u32(pdev->dev.of_node, "master",
				  &x2_snd_config[id].master);
	if (rc < 0) {
		pr_err("failed: master rc %d", rc);
		return rc;
	}
	pr_err("current snd card id is %d, master flag is %d\n",
		id, x2_snd_config[id].master);

	rc = of_property_read_u32(pdev->dev.of_node, "channel_max",
				  &x2_snd_config[id].channel_max);
	if (rc < 0) {
		pr_err("failed: channel_max rc %d", rc);
		return rc;
	}
	pr_err("current snd card id is %d, channel_max is %d\n",
		id, x2_snd_config[id].channel_max);

	return rc;

}

static int x2_snd_probe(struct platform_device *pdev)
{
	int ret = 0;
	int id = 0;
	struct snd_soc_card *card;

	id = of_alias_get_id(pdev->dev.of_node, "sndcard");
	if (id < 0)
		id = 0;

	card = &x2_soc_snd[id];

	ret = x2_snd_get_dt_data(pdev, id);

	/* platform device is set to snd_soc_card->dev */
	card->dev = &pdev->dev;

	ret = snd_soc_register_card(card);
	if (ret) {
		dev_err(&pdev->dev, "snd_soc_register_card() failed: %d\n",
				ret);
		return -ENOMEM;
	}

	pr_err("register success for x2snd%d\n", id);

	return ret;
}

static int x2_snd_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id x2_snd_of_match[] = {
	{.compatible = "hobot,x2-snd0",},
	{.compatible = "hobot,x2-snd1",},
	{}
};


MODULE_DEVICE_TABLE(of, x2_snd0_of_match);
#endif

static struct platform_driver x2_snd_driver = {
	.probe = x2_snd_probe,
	.remove = x2_snd_remove,
	.driver = {
		   .name = "x2-snd",
		   .of_match_table = x2_snd_of_match,
		   },
};

module_platform_driver(x2_snd_driver);

/* Module information */
MODULE_AUTHOR("Jxy");
MODULE_DESCRIPTION("X2 snd Interface");
MODULE_ALIAS("platform:x2-snd");
MODULE_LICENSE("GPL");
