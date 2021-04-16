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

#include <linux/of.h>
#include <linux/printk.h>
#include <linux/of_address.h>

void __iomem *clk_reg_base = NULL;
void __iomem *clk_ipsreg_base = NULL;

void __iomem * clk_get_register_base(struct device_node *np)
{
	struct device_node *pnode;

	if(clk_reg_base == NULL){
		pnode = of_get_parent(np);
		if(!pnode){
			pr_err("%s: %s failed to get parent node!\n", __func__, np->name);
			return NULL;
		}
		clk_reg_base = of_iomap(pnode, 0);
		if(!clk_reg_base){
			pr_err("%s: %s faield to remap!\n", __func__, np->name);
			return ERR_PTR(-ENOMEM);
		}
		of_node_put(pnode);
	}
	return clk_reg_base;
}

void __iomem * clk_get_ipsregister_base(struct device_node *np)
{
	struct device_node *pnode;

	if(clk_ipsreg_base == NULL) {
		pnode = of_get_parent(np);
		if(!pnode) {
			pr_err("%s: %s failed to get parent node!\n", __func__, np->name);
			return NULL;
		}
		clk_ipsreg_base = of_iomap(pnode, 1);
		if(!clk_ipsreg_base) {
			pr_err("%s: %s faield to remap!\n", __func__, np->name);
			return ERR_PTR(-ENOMEM);
		}
		of_node_put(pnode);
	}
	return clk_ipsreg_base;
}

