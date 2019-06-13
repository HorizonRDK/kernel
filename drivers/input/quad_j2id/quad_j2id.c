#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>

#define J2ID_PROC_FILE "j2id"

enum j2id {
	J2A,
	J2C,
	J2B,
	J2D,
};

char *j2id_name[] = {
	"j2a",
	"j2c",
	"j2b",
	"j2d",
};


struct quad_j2id_dev {
	unsigned int eth_mac_cfg0;
	unsigned int eth_mac_cfg1;
	int id;
	struct proc_dir_entry *j2id_proc_entry;
	struct device *dev;
};

struct quad_j2id_dev *gquad_j2id;

static int j2id_proc_show(struct seq_file *seq, void *v)
{
	unsigned int *ptr_var = seq->private;

	if ((*ptr_var < J2A) || (*ptr_var > J2D))
		return -1;
	seq_printf(seq, "%s\n", j2id_name[*ptr_var]);

	return 0;
}

static ssize_t j2id_proc_write(struct file *file,
					const char __user *buffer, size_t count, loff_t *ppos)
{
	struct seq_file *seq = file->private_data;
	unsigned int *ptr_var = seq->private;

	*ptr_var = simple_strtoul(buffer, NULL, 10);
	return count;
}

static int test_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, j2id_proc_show, PDE_DATA(inode));
}

static const struct file_operations j2id_proc_fops = {
	.owner = THIS_MODULE,
	.open = test_proc_open,
	.read = seq_read,
	.write = j2id_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

#ifdef CONFIG_OF
static inline int j2id_get_devtree_pdata(struct quad_j2id_dev *j2id_dev)
{
	struct device *dev = j2id_dev->dev;
	int ret;

	if (of_property_read_u32(dev->of_node, "eth_mac_cfg0_gpio",
		&j2id_dev->eth_mac_cfg0)) {
		dev_err(dev, "get eth_mac_cfg0_gpio proterty err\n");
		return  -EINVAL;
	}

	if (of_property_read_u32(dev->of_node, "eth_mac_cfg1_gpio",
		&j2id_dev->eth_mac_cfg1)) {
		dev_err(dev, "get eth_mac_cfg0_gpio proterty err\n");
		return -EINVAL;
	}

	ret = gpio_request(j2id_dev->eth_mac_cfg0, "eth-mac-cfg0-gpio");
	if (ret) {
		dev_err(dev, "request eth-mac-cfg0-gpio fail,(%d)\n",
			j2id_dev->eth_mac_cfg0);
		return ret;
	}

	ret = gpio_request(j2id_dev->eth_mac_cfg1, "eth-mac-cfg1-gpio");
	if (ret) {
		dev_err(dev, "request eth-mac-cfg1-gpio fail,(%d)\n",
			j2id_dev->eth_mac_cfg1);
		goto fail_gpio_req0;
	}

	ret = gpio_direction_input(j2id_dev->eth_mac_cfg0);
	if (ret) {
		dev_err(dev, "set eth-mac-cfg0-gpio direction err,rc(%d)\n",
			ret);
		goto fail_gpio_req1;
	}

	ret = gpio_direction_input(j2id_dev->eth_mac_cfg1);
	if (ret) {
		dev_err(dev, "set eth-mac-cfg1-gpio direction err,rc(%d)\n",
			ret);
		goto fail_gpio_req1;
	}

	return 0;
fail_gpio_req1:
	gpio_free(j2id_dev->eth_mac_cfg1);
fail_gpio_req0:
	gpio_free(j2id_dev->eth_mac_cfg0);

	return ret;
}

static const struct of_device_id j2id_of_match[] = {
	{ .compatible = "hobot,matrix2v0-quad-j2id", },
	{ },
};
#else
static inline int j2id_get_devtree_pdata(struct quad_j2id_dev *j2id_dev)
{
	return ERR_PTR(-ENODEV);
}
#endif

static int j2id_get_value(struct quad_j2id_dev *j2id_dev)
{
	int ret;
	struct device *dev = j2id_dev->dev;

	ret = gpio_get_value(j2id_dev->eth_mac_cfg0);
	if (ret < 0) {
		dev_err(dev, "get eth_mac_cfg0 value failed,rc(%d)\n", ret);
		return ret;
	}
	j2id_dev->id = ret;

	ret = gpio_get_value(j2id_dev->eth_mac_cfg1);
	if (ret < 0) {
		dev_err(dev, "get eth_mac_cfg1 value failed,rc(%d)\n", ret);
		return ret;
	}
	j2id_dev->id |= (ret << 1);

	return 0;
}

static int j2id_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;
	struct quad_j2id_dev *quad_j2id = NULL;

	quad_j2id = devm_kzalloc(dev, sizeof(*quad_j2id), GFP_KERNEL);
	if (!quad_j2id) {
		dev_err(dev, "no more memery\n");
		ret = -ENOMEM;
		return ret;
	}
	quad_j2id->dev = dev;

	ret = j2id_get_devtree_pdata(quad_j2id);
	if (ret) {
		dev_err(dev, "get device tree data error(%d)\n", ret);
		return ret;
	}

	ret = j2id_get_value(quad_j2id);
	if (ret) {
		dev_err(dev, "get j2id error,rc(%d)\n", ret);
		return ret;
	}

	quad_j2id->j2id_proc_entry = proc_create_data(J2ID_PROC_FILE, 0666, NULL, &j2id_proc_fops, &quad_j2id->id);
	if (!quad_j2id->j2id_proc_entry) {
		dev_err(dev, "create proc entry error\n");
		return -ENOMEM;
	}
	gquad_j2id = quad_j2id;

	return 0;
}

static int j2id_remove(struct platform_device *pdev)
{
	if (gquad_j2id) {
		if (gquad_j2id->j2id_proc_entry)
			remove_proc_entry(J2ID_PROC_FILE, NULL);

		gpio_free(gquad_j2id->eth_mac_cfg1);
		gpio_free(gquad_j2id->eth_mac_cfg0);
		return 0;
	}

	return -EINVAL;
}

static struct platform_driver j2id_device_driver = {
	.probe		= j2id_probe,
	.remove		= j2id_remove,
	.driver		= {
		.name	= "j2id",
		.of_match_table = of_match_ptr(j2id_of_match),
	}
};

static int __init j2id_proc_init(void)
{
	return platform_driver_register(&j2id_device_driver);
}

static void __exit j2id_proc_exit(void)
{
	platform_driver_unregister(&j2id_device_driver);
}

module_init(j2id_proc_init);
module_exit(j2id_proc_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("horizon robot,donghe.zhang");
MODULE_DESCRIPTION("driver for j2 identify on quad");






