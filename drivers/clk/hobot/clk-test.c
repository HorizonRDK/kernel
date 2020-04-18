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

#include <linux/init.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/clk.h>
#include <linux/printk.h>

typedef struct _clk_test{
	struct clk *laintpll;
	struct clk *fracpll;
	struct clk *mux;
	struct clk *divider;
	struct clk *gate;
	struct clk *osc;
} clk_test;

static clk_test *g_clk_test;

static int clk_test_open(struct inode *inode, struct file *file)
{
	clk_test *clk;
	clk = g_clk_test;
	file->private_data = clk;

	pr_info("%s clk test open.\n", __func__);
	return 0;
}

void clk_gate_enable_test(struct clk *clk)
{
	pr_info("%s: clk gate enable test start\n", __func__);

	clk_enable(clk);

	pr_info("%s: clk gate enable test done, pls check\n", __func__);
	return;
}

void clk_gate_disable_test(struct clk *clk)
{
	pr_info("%s: clk gate disable test start\n", __func__);

	clk_disable(clk);

	pr_info("%s: clk gate disable test done, pls check\n", __func__);
	return;
}

void clk_divider_test(struct clk *clk)
{
	unsigned long rate, target_rate;
	pr_info("%s: clk divider test start\n", __func__);

	rate = clk_get_rate(clk);
	target_rate = rate / 2;
	rate = clk_round_rate(clk, target_rate);
	clk_set_rate(clk, rate);

	pr_info("%s: clk divider test done, pls check\n", __func__);
	return;
}

void clk_mux_test(struct clk *clk, struct clk *new)
{
	pr_info("%s: clk mux test start\n", __func__);

	clk_set_parent(clk, new);

	pr_info("%s: clk mux test done, pls check\n", __func__);
	return;
}

void clk_pll_test(struct clk *clk)
{
	unsigned long rate, target_rate;
	pr_info("%s: clk pll test start\n", __func__);

	rate = clk_get_rate(clk);
	if(rate >= 0){
		pr_info("%s: get current rate %lu\n", __func__, rate);
	}else{
		pr_info("%s: current rate invalid\n", __func__);
		return;
	}
	target_rate = rate / 2;
	rate = clk_round_rate(clk, target_rate);
	if(rate >= 0){
		pr_info("%s: get round rate %lu\n", __func__, rate);
	}else{
		pr_info("%s: round rate invalid\n", __func__);
		return;
	}

	clk_set_rate(clk, rate);
	pr_info("%s: clk pll test done, pls check\n", __func__);
	return;
}

void clk_pll_enable_test(struct clk *clk)
{
	pr_info("%s: clk pll enable test enable start\n", __func__);

	clk_enable(clk);

	pr_info("%s: clk pll enable test done, pls check\n", __func__);
	return;
}

void clk_pll_disable_test(struct clk *clk)
{
	pr_info("%s: clk pll disable test enable start\n", __func__);

	clk_disable(clk);

	pr_info("%s: clk pll disable test done, pls check\n", __func__);
	return;
}

static ssize_t clk_test_write(struct file *file, const char *data,
				size_t len, loff_t *ppos)
{
	clk_test *clk;
	char user[512];
	int ret;
	int cmd;

	clk = file->private_data;

	memset(user, 0x00, sizeof(user));
	ret = copy_from_user(user, data, len);
	cmd = user[0] - 0x30;
	pr_info("%s: valid cmd 0x%x\n", __func__, cmd);

	switch(cmd){
		case 0:
			clk_gate_enable_test(clk->gate);
			break;
		case 1:
			clk_gate_disable_test(clk->gate);
			break;
		case 2:
			clk_divider_test(clk->divider);
			break;
		case 3:
			clk_mux_test(clk->mux, clk->osc);
			break;
		case 4:
			clk_pll_test(clk->laintpll);
			break;
		case 5:
			clk_pll_enable_test(clk->laintpll);
			break;
		case 6:
			clk_pll_disable_test(clk->laintpll);
			break;
		case 7:
			clk_pll_test(clk->fracpll);
			break;
		case 8:
			clk_pll_enable_test(clk->fracpll);
			break;
		case 9:
			clk_pll_disable_test(clk->fracpll);
			break;
		default:
			pr_info("%s: command not supported\n", __func__);
			break;
	}
	return len;
}

static const struct file_operations clk_test_fops = {
	.owner = THIS_MODULE,
	.open  = clk_test_open,
	.write = clk_test_write,
};

static struct miscdevice clk_test_miscdev = {
	.minor	= MISC_DYNAMIC_MINOR,
	.name	= "clk_test",
	.fops	= &clk_test_fops,
};

static int clk_test_probe(struct platform_device *pdev)
{
	int ret;
	clk_test *clk;

	clk = devm_kzalloc(&pdev->dev, sizeof(clk_test), GFP_KERNEL);
	if(!clk){
		pr_err("%s: failed to alloc clk structure\n", __func__);
		return -ENOMEM;
	}
	dev_set_drvdata(&pdev->dev, clk);

	clk->laintpll = devm_clk_get(&pdev->dev, "laintpll");
	if(IS_ERR(clk->laintpll)){
		pr_err("%s: failed to get pll clk", __func__);
		return PTR_ERR(clk->laintpll);
	}
	ret = clk_prepare(clk->laintpll);
	if(ret != 0){
		pr_err("%s: failed to prepare the laintpll clk!\n", __func__);
		return -1;
	}

	clk->fracpll = devm_clk_get(&pdev->dev, "fracpll");
	if(IS_ERR(clk->fracpll)){
		pr_err("%s: failed to get fracpll clk", __func__);
		return PTR_ERR(clk->fracpll);
	}
	ret = clk_prepare(clk->fracpll);
	if(ret != 0){
		pr_err("%s: failed to prepare the fracpll clk!\n", __func__);
		return -1;
	}

	clk->mux = devm_clk_get(&pdev->dev, "mux");
	if(IS_ERR(clk->mux)){
		pr_err("%s: failed to get mux clk", __func__);
		return PTR_ERR(clk->mux);
	}
	ret = clk_prepare(clk->mux);
	if(ret != 0){
		pr_err("%s: failed to prepare the mux clk!\n", __func__);
		return -1;
	}

	clk->divider = devm_clk_get(&pdev->dev, "divider");
	if(IS_ERR(clk->divider)){
		pr_err("%s: failed to get divider clk", __func__);
		return PTR_ERR(clk->divider);
	}
	ret = clk_prepare(clk->divider);
	if(ret != 0){
		pr_err("%s: failed to prepare the divider clk!\n", __func__);
		return -1;
	}

	clk->gate = devm_clk_get(&pdev->dev, "gate");
	if(IS_ERR(clk->gate)){
		pr_err("%s: failed to get gate clk", __func__);
		return PTR_ERR(clk->gate);
	}
	ret = clk_prepare(clk->gate);
	if(ret != 0){
		pr_err("%s: failed to prepare the gate clk!\n", __func__);
		return -1;
	}

	clk->osc = devm_clk_get(&pdev->dev, "osc");
	if(IS_ERR(clk->osc)){
		pr_err("%s: failed to get osc clk", __func__);
		return PTR_ERR(clk->osc);
	}
	ret = clk_prepare(clk->osc);
	if(ret != 0){
		pr_err("%s: failed to prepare the osc clk!\n", __func__);
		return -1;
	}

	g_clk_test = clk;

	ret = misc_register(&clk_test_miscdev);
	if (ret < 0) {
		pr_err("%s: failed to register clk_test device\n", __func__);
		return -1;
	}

	pr_info("%s: clk test probe done\n", __func__);
	return 0;
}

static int clk_test_remove(struct platform_device *pdev)
{
	misc_deregister(&clk_test_miscdev);
	return 0;
}

static const struct of_device_id x2_clk_test_dt_ids[] = {
	{ .compatible = "hobot,clk-test", },
	{},
};

MODULE_DEVICE_TABLE(of, x2_clk_test_dt_ids);

static struct platform_driver x2_clk_test_driver = {
	.probe	= clk_test_probe,
	.remove = clk_test_remove,
	.driver = {
		.name  = "clk_test",
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(x2_clk_test_dt_ids),
	},
};

static int __init clk_test_init(void)
{
	int ret;

	ret = platform_driver_register(&x2_clk_test_driver);
	if (ret)
		pr_err("%s: clk test probe failed: %d\n", __func__, ret);

	return ret;
}
module_init(clk_test_init);

static void __exit clk_test_exit(void)
{
	platform_driver_unregister(&x2_clk_test_driver);
}
module_exit(clk_test_exit);

MODULE_LICENSE("GPL");
