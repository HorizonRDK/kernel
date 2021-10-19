/*
 * Horizon Robotics
 *
 *  Copyright (C) 2020 Horizon Robotics Inc.
 *  All rights reserved.
 *
 *  hobot_codec.c  --  dummy audio codec for hobot
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <sound/soc.h>
#include <sound/pcm.h>
#include <sound/initval.h>

struct snd_soc_dai_driver dummy_dai[] = {
	{
		.name = "hobot-codec0",
		.capture = {
			.stream_name = "Dummy Capture",
			.channels_min = 1,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S20_3LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE),
		},
		.playback = {
                        .stream_name = "Dummy Playback",
                        .channels_min = 1,
                        .channels_max = 8,
                        .rates = SNDRV_PCM_RATE_8000_192000,
                        .formats = (SNDRV_PCM_FMTBIT_S16_LE |
                                        SNDRV_PCM_FMTBIT_S20_3LE |
                                        SNDRV_PCM_FMTBIT_S24_LE |
                                        SNDRV_PCM_FMTBIT_S32_LE),
                },
	},
	{
		.name = "hobot-codec1",
		.capture = {
			.stream_name = "Dummy Capture",
			.channels_min = 1,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S20_3LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE),
		},
		.playback = {
                        .stream_name = "Dummy Playback",
                        .channels_min = 1,
                        .channels_max = 8,
                        .rates = SNDRV_PCM_RATE_8000_192000,
                        .formats = (SNDRV_PCM_FMTBIT_S16_LE |
                                        SNDRV_PCM_FMTBIT_S20_3LE |
                                        SNDRV_PCM_FMTBIT_S24_LE |
                                        SNDRV_PCM_FMTBIT_S32_LE),
                },
	},
	{
		.name = "hobot-codec2",
		.capture = {
			.stream_name = "Dummy Capture",
			.channels_min = 1,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S20_3LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE),
		},
	},
	{
		.name = "hobot-codec3",
		.capture = {
			.stream_name = "Dummy Capture",
			.channels_min = 1,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S20_3LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE),
		},
	},
	{
		.name = "hobot-codec4",
		.capture = {
			.stream_name = "Dummy Capture",
			.channels_min = 1,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S20_3LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE),
		},
	},
	{
		.name = "hobot-codec5",
		.capture = {
			.stream_name = "Dummy Capture",
			.channels_min = 1,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S20_3LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE),
		},
	},
	{
		.name = "hobot-codec6",
		.capture = {
			.stream_name = "Dummy Capture",
			.channels_min = 1,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S20_3LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE),
		},
	},
	{
		.name = "hobot-codec7",
		.capture = {
			.stream_name = "Dummy Capture",
			.channels_min = 1,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_8000_192000,
			.formats = (SNDRV_PCM_FMTBIT_S16_LE |
					SNDRV_PCM_FMTBIT_S20_3LE |
					SNDRV_PCM_FMTBIT_S24_LE |
					SNDRV_PCM_FMTBIT_S32_LE),
		},
	},
};

static const struct snd_soc_codec_driver soc_dummy_codec;

static int hobot_codec_probe(struct platform_device *pdev)
{
	int ret;
	ret = snd_soc_register_codec(&pdev->dev, &soc_dummy_codec,
		dummy_dai, ARRAY_SIZE(dummy_dai));

	return ret;
}

static int hobot_codec_remove(struct platform_device *pdev)
{
	snd_soc_unregister_codec(&pdev->dev);

	return 0;
}

static const struct of_device_id hobot_codec_of_match[] = {
	{ .compatible = "hobot,hobot-codec0", },
	{},
};
MODULE_DEVICE_TABLE(of, hobot_codec_of_match);

static struct platform_driver hobot_codec_driver = {
	.driver = {
		.name = "hobot-codec",
		.of_match_table = hobot_codec_of_match,
	},
	.probe = hobot_codec_probe,
	.remove = hobot_codec_remove,
};

module_platform_driver(hobot_codec_driver);

MODULE_AUTHOR("sunlf");
MODULE_DESCRIPTION("Hobot Codec Driver");
MODULE_LICENSE("GPL v2");
