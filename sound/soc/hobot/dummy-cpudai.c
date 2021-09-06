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
#include <linux/reset.h>

#include "./hobot-i2s.h"

#define msecs_to_loops(t) (loops_per_jiffy / 1000 * HZ * t)

#define HOBOT_I2S_RATES    SNDRV_PCM_RATE_8000_96000

#define HOBOT_I2S_FMTS (SNDRV_PCM_FMTBIT_S8 | SNDRV_PCM_FMTBIT_S16_LE)

static int hobot_dummy_i2s_dai_probe(struct snd_soc_dai *dai)
{
        return 0;
}

static int hobot_dummy_i2s_dai_remove(struct snd_soc_dai *dai)
{
        return 0;
}

static struct snd_soc_dai_driver hobot_dummy_i2s_dai_drv[] = {
    {
            .probe = hobot_dummy_i2s_dai_probe,
            .remove = hobot_dummy_i2s_dai_remove,
            .capture = {
                            .stream_name = "i2s-dummy-Capture",
                            .channels_min = 1,
                            .channels_max = 8,
                            .rates = HOBOT_I2S_RATES,
                            .formats = HOBOT_I2S_FMTS,
                            },

            .symmetric_rates = 1,
            .name = "hobot-dummy-i2s0",
    },
    {
            .probe = hobot_dummy_i2s_dai_probe,
            .remove = hobot_dummy_i2s_dai_remove,
            .capture = {
                            .stream_name = "i2s-dummy-Capture",
                            .channels_min = 1,
                            .channels_max = 8,
                            .rates = HOBOT_I2S_RATES,
                            .formats = HOBOT_I2S_FMTS,
                            },

            .symmetric_rates = 1,
            .name = "hobot-dummy-i2s1",
    },
    {
            .probe = hobot_dummy_i2s_dai_probe,
            .remove = hobot_dummy_i2s_dai_remove,
            .capture = {
                            .stream_name = "i2s-dummy-Capture",
                            .channels_min = 1,
                            .channels_max = 8,
                            .rates = HOBOT_I2S_RATES,
                            .formats = HOBOT_I2S_FMTS,
                            },

            .symmetric_rates = 1,
            .name = "hobot-dummy-i2s2",
    },
    {
            .probe = hobot_dummy_i2s_dai_probe,
            .remove = hobot_dummy_i2s_dai_remove,
            .capture = {
                            .stream_name = "i2s-dummy-Capture",
                            .channels_min = 1,
                            .channels_max = 8,
                            .rates = HOBOT_I2S_RATES,
                            .formats = HOBOT_I2S_FMTS,
                            },

            .symmetric_rates = 1,
            .name = "hobot-dummy-i2s3",
    },
    {
            .probe = hobot_dummy_i2s_dai_probe,
            .remove = hobot_dummy_i2s_dai_remove,
            .capture = {
                            .stream_name = "i2s-dummy-Capture",
                            .channels_min = 1,
                            .channels_max = 8,
                            .rates = HOBOT_I2S_RATES,
                            .formats = HOBOT_I2S_FMTS,
                            },

            .symmetric_rates = 1,
            .name = "hobot-dummy-i2s4",
    },
    {
            .probe = hobot_dummy_i2s_dai_probe,
            .remove = hobot_dummy_i2s_dai_remove,
            .capture = {
                            .stream_name = "i2s-dummy-Capture",
                            .channels_min = 1,
                            .channels_max = 8,
                            .rates = HOBOT_I2S_RATES,
                            .formats = HOBOT_I2S_FMTS,
                            },

            .symmetric_rates = 1,
            .name = "hobot-dummy-i2s5",
    },
    {
            .probe = hobot_dummy_i2s_dai_probe,
            .remove = hobot_dummy_i2s_dai_remove,
            .capture = {
                            .stream_name = "i2s-dummy-Capture",
                            .channels_min = 1,
                            .channels_max = 8,
                            .rates = HOBOT_I2S_RATES,
                            .formats = HOBOT_I2S_FMTS,
                            },

            .symmetric_rates = 1,
            .name = "hobot-dummy-i2s6",
    },
    {
            .probe = hobot_dummy_i2s_dai_probe,
            .remove = hobot_dummy_i2s_dai_remove,
            .capture = {
                            .stream_name = "i2s-dummy-Capture",
                            .channels_min = 1,
                            .channels_max = 8,
                            .rates = HOBOT_I2S_RATES,
                            .formats = HOBOT_I2S_FMTS,
                            },

            .symmetric_rates = 1,
            .name = "hobot-dummy-i2s7",
    },
};

static const struct snd_soc_component_driver hobot_dummy_i2s_component_drv = {};
static int hobot_dummy_i2s_probe(struct platform_device *pdev) {
    int ret;

    pr_err("%s start\n", __func__);

    ret = devm_snd_soc_register_component(&pdev->dev,
        &hobot_dummy_i2s_component_drv, hobot_dummy_i2s_dai_drv,
        ARRAY_SIZE(hobot_dummy_i2s_dai_drv));
    if (ret) {
        dev_err(&pdev->dev, "Could not register DAI\n");
        return ret;
    }

    pr_err("success register dummy cpu dai driver\n");
    return 0;
}

static int hobot_dummy_i2s_remove(struct platform_device *pdev) {
    snd_soc_unregister_component(&pdev->dev);
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id hobot_dummy_i2s_match[] = {
    { .compatible = "hobot, hobot-dummy-i2s0"},
    {},
};
MODULE_DEVICE_TABLE(of, hobot_dummy_i2s_match);
#endif

static struct platform_driver hobot_dummy_i2s_driver = {
    .driver = {
        .name = "hobot-dummy-i2s",
        .of_match_table = hobot_dummy_i2s_match,
    },
    .probe = hobot_dummy_i2s_probe,
    .remove = hobot_dummy_i2s_remove,
};

module_platform_driver(hobot_dummy_i2s_driver);

/* Module information */
MODULE_AUTHOR("lingfeng.sun <v-lingfeng.sun@horizon.ai");
MODULE_DESCRIPTION("Hobot I2S Soc Interface");
MODULE_ALIAS("cpu dai driver:hobot-dummy-i2s");
MODULE_LICENSE("GPL");
