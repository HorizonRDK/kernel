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

#include <linux/module.h>
#include <sound/soc.h>
#include <sound/dmaengine_pcm.h>
#include "./hobot-i2s.h"

#define HOBOT_DEF_SND_CARD 0;
static int snd_card = HOBOT_DEF_SND_CARD;
module_param(snd_card, uint, S_IRUGO);
MODULE_PARM_DESC(snd_card, "Hobot Sound card");

struct hobot_snd {
    uint32_t work_mode;
};

static int hobot_snd_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	unsigned int sample_rate = params_rate(params);
	struct hobot_i2s *i2s = snd_soc_dai_get_drvdata(rtd->cpu_dai);
	struct hobot_snd *snd_data = snd_soc_card_get_drvdata(rtd->card);
	int ret = 0;

	if (!i2s || !snd_data)
		return -ENOMEM;
	i2s->work_mode = snd_data->work_mode;
	if(rtd->dai_link){
		if(rtd->dai_link->dai_fmt){
			ret = snd_soc_runtime_set_dai_fmt(rtd, rtd->dai_link->dai_fmt);
			if(0 > ret)
				return ret;
		}
		if(rtd->dai_link->dai_fmt & SND_SOC_DAIFMT_CBS_CFS){
			/* below div is getted by sample_rate */
			/*
			ret = snd_soc_dai_set_clkdiv(rtd->codec_dai, 0, sample_rate);
			if (ret < 0) {
				pr_err("%s, line:%d\n", __func__, __LINE__);
				return ret;
			}
			*/
		} else {
			ret = snd_soc_dai_set_clkdiv(rtd->cpu_dai, 0, sample_rate);
			if (ret < 0) {
				pr_err("%s, line:%d\n", __func__, __LINE__);
				return ret;
			}
		}
	}

	return ret;
}

static struct snd_soc_ops hobot_snd_ops = {
	.hw_params = hobot_snd_hw_params,
};

static int hobot_snd_probe(struct platform_device *pdev)
{
	int ret = 0, id = 0, num = 0, idx = 0;
	struct snd_soc_card *card = NULL;
	struct snd_soc_dai_link *link = NULL, *links = NULL;
	struct device *dev = &pdev->dev;
	struct device_node *np, *codec, *cpu, *platform, *node = dev->of_node;
	struct hobot_snd *snd_data;

	card = devm_kzalloc(dev, sizeof(*card), GFP_KERNEL);
	if (!card) {
		pr_err("No memory for snd_soc_card alloc for card:%d\n", id);
		return -ENOMEM;
	}
	card->owner = THIS_MODULE;
	card->dev = dev;
	ret = snd_soc_of_parse_card_name(card, "model");
	if (ret < 0)
		return ret;

	if (!card->name) {
		pr_err("Error: sound card name is NULL!\n");
		return -EINVAL;
	}

	pr_debug("Create sound card: %s\n", card->name);
	if (node && of_get_child_by_name(node, "dai-link"))
		num = of_get_child_count(node);
	else
		num = 1;
	pr_debug("Number dai_link: %d\n", num);

	links = devm_kzalloc(dev, sizeof(*link) * num, GFP_KERNEL);
	if (!links) {
		pr_err("No memory for snd_soc_dai_link alloc for card:%d\n", id);
		return -ENOMEM;
	}
	card->dai_link = links;
	id = of_alias_get_id(pdev->dev.of_node, "sndcard");
	if (id < 0) {
		pr_debug("id: %d\n", id);
		if (!strcmp(card->name, "hobotsnd3"))
			id = 3;
	}
	snd_data = devm_kzalloc(dev, sizeof(*snd_data), GFP_KERNEL);
	if (!snd_data)
			return -ENOMEM;
	ret = of_property_read_u32(pdev->dev.of_node, "work_mode",
		&snd_data->work_mode);
	if (ret < 0) {
			pr_err("failed get work_mode %d\n", ret);
			return ret;
    }
		for_each_child_of_node(node, np) {
			link = links + idx;
			link->ops = &hobot_snd_ops;
			cpu = of_get_child_by_name(np, "cpu");
			codec = of_get_child_by_name(np, "codec");
			if (!cpu || !codec) {
				dev_err(dev, "Can't find cpu(%p)/codec(%p) DT node\n",
					cpu, codec);
				return -EINVAL;
			}
			link->cpu_of_node = of_parse_phandle(cpu, "sound-dai", 0);
			if (!link->cpu_of_node) {
				dev_err(dev, "error getting cpu phandle\n");
				return ret;
			}

			pr_debug("Name of link->cpu_of_node : %s\n", link->cpu_of_node->name);
		
			ret = snd_soc_of_get_dai_link_codecs(dev, codec, link);
			if (!link->codecs) {
				dev_err(dev, "error getting codec\n");
				return -EINVAL;
			}
			if (ret < 0) {
				dev_err(dev, "error getting codec dai name(%d)\n", ret);
				return ret;
			}

			platform = of_get_child_by_name(np, "platform");
			if (platform) {
				link->platform_of_node = of_parse_phandle(platform, "sound-dai", 0);
				if (!link->platform_of_node) {
					dev_err(dev, "error getting platform phandle\n");
					return ret;
				}
				pr_debug("Name of link->platform_of_node : %s\n",
					link->platform_of_node->name);
			}

			ret = of_property_read_string(np, "link-name", &link->name);
			if (ret) {
				dev_err(dev, "error getting codec dai_link name\n");
				return ret;
			}
			link->dai_fmt = snd_soc_of_parse_daifmt(np, NULL, NULL, NULL);
			pr_debug("Data of link->dai_fmt: 0x%08X\n", link->dai_fmt);
			card->num_links++;
			idx++;
		}

	snd_soc_card_set_drvdata(card, snd_data);
	ret = snd_soc_register_card(card);
	if (ret) {
 		dev_err(dev, "snd_soc_register_card() failed: %d\n", ret);
		return -ENOMEM;
	}

	pr_info("Register success for hobotsnd%d\n", id);

	return ret;
}


static int hobot_snd_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	if(card){
		snd_soc_unregister_card(card);
	}
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id hobot_snd_of_match[] = {
	{.compatible = "hobot, hobot-snd3", },
	{}
};


MODULE_DEVICE_TABLE(of, hobot_snd_of_match);
#endif

static struct platform_driver hobot_snd_driver = {
	.probe = hobot_snd_probe,
	.remove = hobot_snd_remove,
	.driver = {
	   .name = "hobot-snd",
#ifdef CONFIG_OF
	   .of_match_table = hobot_snd_of_match,
#endif
   },
};

int __init hobot_snd_init(void)
{
	return platform_driver_register(&hobot_snd_driver);
}

static void __exit hobot_snd_cleanup(void)
{
	platform_driver_unregister(&hobot_snd_driver);
}

late_initcall(hobot_snd_init);
module_exit(hobot_snd_cleanup);

/* Module information */
MODULE_AUTHOR("Li Yang");
MODULE_DESCRIPTION("Hobot snd Interface");
MODULE_ALIAS("platform:hobot-snd-96boards");
MODULE_LICENSE("GPL");
