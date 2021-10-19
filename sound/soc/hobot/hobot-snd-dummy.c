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

/*pll source*/
#define AC101_MCLK1 1
#define AC101_MCLK2 2
#define AC101_BCLK1 3
#define AC101_BCLK2 4

#define FREQ_OUT 24576000
#define HOBOT_DEF_SND_CARD 0;
static int snd_card = HOBOT_DEF_SND_CARD;
module_param(snd_card, uint, S_IRUGO);
MODULE_PARM_DESC(snd_card, "Hobot Sound card");

struct dummy_data {
	struct snd_soc_dai_link dai_link[3];
};

static int hobot_snd_probe(struct platform_device *pdev)
{
	int ret = 0, id = 0, num = 0, idx = 0;
	struct snd_soc_card *card = NULL;
	struct snd_soc_dai_link *link = NULL, *links = NULL;
	struct device *dev = &pdev->dev;
	struct device_node *np, *codec, *cpu, *platform, *node = dev->of_node;
	struct dummy_data *data;

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

	data = devm_kzalloc(dev, sizeof(*data) + sizeof(*link) * num,
            GFP_KERNEL);
	 if (!data)
        return ERR_PTR(-ENOMEM);

	card->dai_link = &data->dai_link[0];
	card->num_links = num;

	link = data->dai_link;

	id = of_alias_get_id(pdev->dev.of_node, "sndcard");
	if (id < 0) {
		pr_debug("id: %d\n", id);
		if (!strcmp(card->name, "hobotsnd4"))
			id = 4;
		else if (!strcmp(card->name, "hobotsnd5"))
                        id = 5;
	}
	if (id == snd_card) {
		for_each_child_of_node(node, np) {
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

			ret = snd_soc_of_get_dai_name(cpu, &link->cpu_dai_name);
			if (ret) {
					dev_err(card->dev, "error getting cpu dai name\n");
					return ERR_PTR(ret);
			}
			pr_debug("cpu_dai_name: %s\n", link->cpu_dai_name);
		
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
			link++;
		}
	} else {
		if (data)
			devm_kfree(dev, data);
		if (card)
			devm_kfree(dev, card);
		return 0;
	}
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
	{.compatible = "hobot, hobot-snd4", },
	{.compatible = "hobot, hobot-snd5", },
	{}
};


MODULE_DEVICE_TABLE(of, hobot_snd_of_match);
#endif

static struct platform_driver hobot_snd_driver = {
	.probe = hobot_snd_probe,
	.remove = hobot_snd_remove,
	.driver = {
	   .name = "hobot-snd-dummy",
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
