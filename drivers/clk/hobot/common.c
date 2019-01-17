#include <linux/of.h>
#include <linux/printk.h>
#include <linux/of_address.h>

static void __iomem *clk_reg_base = NULL;

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
			return -ENOMEM;
		}
		of_node_put(pnode);
	}
	return clk_reg_base;
}
