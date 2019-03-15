/*************************************************************
 ****			 COPYRIGHT NOTICE			 ****
 ****		 Copyright	2018 Horizon Robotics, Inc.		 ****
 ****			 All rights reserved.			 ****
 *************************************************************/
/**
 * BIF base driver for memory and irq management
 * @author		xiaofeng.ling(xiaofeng.ling@horizon.ai)
 * @date		2018/12/21
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/gpio/consumer.h>
#include <linux/delay.h>
#include "bif_base.h"
#include "bif_api.h"

//#define X2_LINUX_VERSION_CODE		(((4) << 16) + ((9) << 8) + (0))
#define X2_LINUX_VERSION_CODE		(((4) << 16) + ((14) << 8) + (74))
#define KERNEL_VERSION(a, b, c)		(((a) << 16) + ((b) << 8) + (c))
#if KERNEL_VERSION(4, 14, 14) <= X2_LINUX_VERSION_CODE
#include <linux/sched/signal.h>
#endif

#define BASE_PHY_SIZE	512
#ifdef CONFIG_HOBOT_BIF_AP
#define BIF_BASE_MAGIC	"BIFA"
wait_queue_head_t bif_ap_wq;
static int bif_ap_wq_flg;
#else
#define BIF_BASE_MAGIC	"BIFB"
static unsigned int base_phy_addr_offset;
static unsigned int base_phy_addr_maxsize;
#endif

static struct bif_base_info *bif_base, *bif_ap, *bif_self, *bif_other;
/*0x7ff00000 vmware reversed address*/
unsigned long base_phy_addr = 0x7ff00000;
void *bif_vir_addr;
wait_queue_head_t bif_irq_wq;
static int bif_irq_wq_flg;
struct work_struct bif_irq_work;	/* irq work */
static int irq_pin = -1;
static int tri_pin = -1;
static int irq_num = -1;
static int tri_val;
static int bif_base_start;
static struct device *pbdev;
static irq_handler_t irq_func[BUFF_MAX];
static int irq_pin_absent;
/*
 *fpga(x2)	hogu(AP)
 *84-->		54(960)	 GPIO_EMIO_0
 *85<--		55(961)	 GPIO_EMIO_1
 *arch/arm64/boot/dts/hobot-x2-fpga.dts
 *#ifndef CONFIG_HOBOT_BIF_AP
 *bif_base_reserved: bif_base_reserved@0x20000000 {
 *	reg = <0x0 0x20000000 0x0 0x00100000>;
 *	no-map;
 *};
 *#endif
 *bif_base {
 *	compatible = "hobot,bif_base";
 *#ifdef CONFIG_HOBOT_BIF_AP
 *	bif_base_phy_add = <0x20000000>;
 *#else
 *	memory-region = <&bif_base_reserved>;
 *#endif
 *	bif_base_irq_pin = <85>;
 *	bif_base_tri_pin = <84>;
 *};

 *cp side
 *hobot-pinctrl.dtsi
 *gpioirq-bank-cfg = <85 0 0 0>;

 *bif_base_reserved: bif_base_reserved@0x20000000 {
 *	reg = <0x0 0x20000000 0x0 0x00100000>;
 *	no-map;
 *};
 *bif_base {
 *	compatible = "hobot,bif_base";
 *	memory-region = <&bif_base_reserved>;
 *	bif_base_irq_pin = <85>;
 *	bif_base_tri_pin = <84>;
 *};

 *Ap side
 *bif_base {
 *	compatible = "hobot,bif_base";
 *	bif_base_phy_add = <0x20000000>;
 *	bif_base_irq_pin = <960>;
 *	bif_base_tri_pin = <961>;
 *};
 */

static void gpio_trigger_irq(int irq)
{
#ifdef CONFIG_HOBOT_BIF_TEST
	t_bif_send_irq(BUFF_BASE);
#else
	if (tri_val)
		tri_val = 0;
	else
		tri_val = 1;
	gpio_direction_output(irq, tri_val);
#endif
}

static int gpio_init(void)
{
	int ret;

	ret = gpio_request(tri_pin, "tri_pin");
	if (ret < 0) {
		pr_err("%s() Err get trigger pin ret= %d\n", __func__, ret);
		goto exit_1;
	}
	gpio_direction_output(tri_pin, tri_val);
	gpio_trigger_irq(tri_pin);

	ret = gpio_request(irq_pin, "irq_pin");
	if (ret < 0) {
		pr_err("%s() Err get irq pin ret= %d\n", __func__, ret);
		gpio_free(tri_pin);
		goto exit_1;
	}
	irq_num = gpio_to_irq(irq_pin);
	if (irq_num < 0) {
		pr_err("%s() Err get irq num\n", __func__);
		gpio_free(tri_pin);
		gpio_free(irq_pin);
		ret = -ENODEV;
	}

exit_1:
	return ret;
}

static void gpio_deinit(void)
{
	gpio_free(tri_pin);
	gpio_free(irq_pin);
}

#ifdef CONFIG_HOBOT_BIF_AP
static int bif_sync_ap(void)
{
	unsigned char tbuf[521];
	unsigned long cur_phy = base_phy_addr + BASE_PHY_SIZE;

	if (!bif_base_start)
		return -1;
	memcpy(tbuf, (unsigned char *)bif_ap, BASE_PHY_SIZE);

#ifdef CONFIG_HOBOT_BIFSD
	if (bif_sd_write((void *)cur_phy, BASE_PHY_SIZE, tbuf))
		return -1;
#else
#ifdef CONFIG_HOBOT_BIFSPI
	if (bif_spi_write((void *)cur_phy, BASE_PHY_SIZE, tbuf))
		return -1;
#endif
#endif

	return 0;
}

static int bif_sync_base(void)
{
	unsigned char tbuf[521];
	unsigned long cur_phy = base_phy_addr;

	if (!bif_base_start)
		return -1;
#ifdef CONFIG_HOBOT_BIFSD
	if (bif_sd_read((void *)cur_phy, BASE_PHY_SIZE, tbuf))
		return -1;
#else
#ifdef CONFIG_HOBOT_BIFSPI
	if (bif_spi_read((void *)cur_phy, BASE_PHY_SIZE, tbuf))
		return -1;
#endif
#endif
	memcpy((unsigned char *)bif_base, tbuf, BASE_PHY_SIZE);

	return 0;
}
#endif

static irqreturn_t bif_base_irq_handler(int irq, void *data)
{
#ifdef CONFIG_HOBOT_BIF_AP
	wake_up_interruptible(&bif_ap_wq);
#endif
	return IRQ_HANDLED;
}

static void work_bif_irq(struct work_struct *work)
{
	int birq;
	int irq_full = 0;

#ifdef CONFIG_HOBOT_BIF_AP
	if (bif_sync_base() != 0)
		pr_err("%s() Err sync base\n", __func__);
#endif

	if ((bif_other->send_irq_tail + 1) % IRQ_QUEUE_SIZE ==
		bif_self->read_irq_head)
		irq_full = 1;

	while (bif_self->read_irq_head != bif_other->send_irq_tail) {
		if (!bif_base_start)
			return;

		birq = bif_other->irq[(bif_self->read_irq_head) %
			IRQ_QUEUE_SIZE];
		if (birq < BUFF_MAX && irq_func[birq % BUFF_MAX])
			irq_func[birq % BUFF_MAX] (birq, NULL);
		else
			pr_warn("%s() Warn irq %d not register\n",
				__func__, birq);

		bif_other->irq[(bif_self->read_irq_head) % IRQ_QUEUE_SIZE] = -1;
		bif_self->read_irq_head =
			(bif_self->read_irq_head + 1) % IRQ_QUEUE_SIZE;
	}

#ifdef CONFIG_HOBOT_BIF_AP
	if (bif_ap_wq_flg)
		wake_up_interruptible(&bif_ap_wq);
	if (bif_sync_ap() != 0)
		pr_err("%s() Err bif sync ap\n", __func__);
#endif
	if (irq_full)
		bif_send_irq(BUFF_BASE);
	if (bif_irq_wq_flg)
		wake_up_interruptible(&bif_irq_wq);
}

static irqreturn_t bif_irq_handler(int irq, void *data)
{
	if (!bif_base_start)
		return IRQ_NONE;
	schedule_work(&bif_irq_work);
	return IRQ_HANDLED;
}

int bif_send_irq(int irq)
{
	if (!bif_base_start || bif_self == NULL || bif_other == NULL)
		return 0;

	while ((bif_self->send_irq_tail + 1) % IRQ_QUEUE_SIZE ==
	       bif_other->read_irq_head) {
		if (bif_base_start) {
#ifdef CONFIG_HOBOT_BIF_AP
			if (bif_sync_ap() != 0)
				pr_err("%s() Err bif sync ap\n", __func__);
#endif
			gpio_trigger_irq(tri_pin);
			if (wait_event_interruptible_timeout(bif_irq_wq,
				(bif_self->send_irq_tail + 1) %	IRQ_QUEUE_SIZE
				!= bif_other->read_irq_head,
				msecs_to_jiffies(2000)) == 0)
				goto try_send_irq;
		} else
			return 0;
	}

	bif_self->irq[(bif_self->send_irq_tail) % IRQ_QUEUE_SIZE] = irq;
	bif_self->send_irq_tail =
	    (bif_self->send_irq_tail + 1) % IRQ_QUEUE_SIZE;

try_send_irq:
#ifdef CONFIG_HOBOT_BIF_AP
	if (bif_sync_ap() != 0)
		pr_err("%s() Err sync ap\n", __func__);
#endif
	gpio_trigger_irq(tri_pin);

	return 0;
}
EXPORT_SYMBOL(bif_send_irq);

int bif_register_address(enum BUFF_ID buffer_id, void *address)
{
	unsigned long addr = (unsigned long)address;

	if (buffer_id >= BUFF_MAX || bif_base == NULL || !bif_base_start)
		return -1;
	bif_base->address_list[buffer_id] = addr;
	bif_base->buffer_count++;
	if (!irq_pin_absent)
		bif_send_irq(BUFF_BASE);
	return 0;
}
EXPORT_SYMBOL(bif_register_address);

int bif_register_irq(enum BUFF_ID buffer_id, irq_handler_t irq_handler)
{
	if (buffer_id >= BUFF_MAX || !bif_base_start)
		return -1;
	irq_func[buffer_id] = irq_handler;
	return buffer_id;
}
EXPORT_SYMBOL(bif_register_irq);

void *bif_query_address_wait(enum BUFF_ID buffer_id)
{
	unsigned long addr = 0;

	if (buffer_id >= BUFF_MAX || bif_base == NULL || !bif_base_start)
		return (void *)-1;

#ifdef CONFIG_HOBOT_BIF_AP
	while (bif_base->address_list[buffer_id] == 0) {
		bif_irq_wq_flg = 1;
		wait_event_interruptible_timeout(bif_ap_wq,
		bif_base->address_list[buffer_id] != 0, msecs_to_jiffies(2000));
		if (signal_pending(current))
			return (void *)-ERESTARTSYS;
	}
	addr = bif_base->address_list[buffer_id];
	bif_irq_wq_flg = 0;
#endif

	return (void *)addr;
}
EXPORT_SYMBOL(bif_query_address_wait);

void *bif_query_address(enum BUFF_ID buffer_id)
{
	unsigned long addr = 0;

	if (buffer_id >= BUFF_MAX || bif_base == NULL || !bif_base_start)
		return (void *)-1;

	addr = bif_base->address_list[buffer_id];
	if (addr != 0)
		return (void *)addr;
	return (void *)-1;
}
EXPORT_SYMBOL(bif_query_address);

#ifndef CONFIG_HOBOT_BIF_AP
void *bif_alloc_cp(enum BUFF_ID buffer_id, int size, unsigned long *phyaddr)
{
	unsigned int basemaxsize = base_phy_addr_maxsize;
	unsigned int baseoffset = base_phy_addr_offset;
	unsigned int basephy = base_phy_addr;
	unsigned int len;
	void *addr = bif_vir_addr;

	pr_info("basemaxsize=%x,baseoffset=%x,basephy=%x,basevir=%lx,size=%x\n",
	    basemaxsize, baseoffset, basephy, (unsigned long)addr, size);

	*phyaddr = 0;

	if (buffer_id >= BUFF_MAX || bif_vir_addr == NULL || !bif_base_start)
		return (void *)-1;

	if (baseoffset < 2*BASE_PHY_SIZE)
		baseoffset = 2*BASE_PHY_SIZE;

	if (size % 512)
		len = (size/512 + 1)*512;
	else
		len = size;

	if (baseoffset + len > basemaxsize)
		return (void *)-1;

	*phyaddr = basephy + baseoffset;
	addr = addr + baseoffset;
	base_phy_addr_offset = baseoffset + len;

	return addr;
}
EXPORT_SYMBOL(bif_alloc_cp);
#endif

void *bif_alloc_base(enum BUFF_ID buffer_id, int size)
{
	int offset;
	void *addr;

	if (buffer_id >= BUFF_MAX || bif_self == NULL || !bif_base_start)
		return (void *)-1;

	if ((bif_self->next_offset + size) > BASE_PHY_SIZE)
		return (void *)-1;

	if (bif_self->offset_list[buffer_id] == 0) {
		offset = bif_self->next_offset;
		bif_self->offset_list[buffer_id] = offset;
		bif_self->next_offset += size;
	} else
		offset = bif_self->offset_list[buffer_id];

	addr = (void *)bif_self + offset;

	bif_send_irq(BUFF_BASE);

	return addr;
}
EXPORT_SYMBOL(bif_alloc_base);

void *bif_query_otherbase_wait(enum BUFF_ID buffer_id)
{
	unsigned int offset = 0;
	void *addr;

	if (buffer_id >= BUFF_MAX || bif_other == NULL || !bif_base_start)
		return (void *)-1;

	while (bif_other->offset_list[buffer_id] == 0) {
		bif_irq_wq_flg = 1;
		wait_event_interruptible_timeout(bif_irq_wq,
			bif_other->offset_list[buffer_id] != 0,
			msecs_to_jiffies(2000));
		if (signal_pending(current))
			return (void *)-ERESTARTSYS;
	}
	bif_irq_wq_flg = 0;

	offset = bif_other->offset_list[buffer_id];
	addr = (void *)bif_other + offset;

	if (offset != 0)
		return addr;

	return (void *)-1;
}
EXPORT_SYMBOL(bif_query_otherbase_wait);

void *bif_query_otherbase(enum BUFF_ID buffer_id)
{
	int offset = 0;

	if (buffer_id >= BUFF_MAX || bif_other == NULL || !bif_base_start)
		return (void *)-1;

	offset = bif_other->offset_list[buffer_id];
	if (offset != 0)
		return (void *)bif_other + offset;

	return (void *)-1;
}
EXPORT_SYMBOL(bif_query_otherbase);

int _bif_base_init(void)
{
	int ret;
	unsigned char *ptr = NULL;

	bif_base_start = 0;
#ifdef CONFIG_HOBOT_BIF_TEST
	bif_netlink_init();
	ret = t_bif_register_irq(BUFF_BASE, bif_irq_handler);
#else
	if (!irq_pin_absent) {
		ret = gpio_init();
		if (ret)
			goto exit_1;
		ret = devm_request_irq(pbdev, irq_num, bif_irq_handler,
			IRQ_TYPE_EDGE_BOTH, "bif_base", NULL);
		if (ret) {
			dev_err(pbdev, "Err request irq fail! irq_num=%d, ret=%d\n",
				irq_num, ret);
			goto exit_1;
		}
	}
#endif

#ifdef CONFIG_HOBOT_BIF_AP
	init_waitqueue_head(&bif_ap_wq);

	bif_vir_addr = kmalloc(2*BASE_PHY_SIZE, GFP_ATOMIC | __GFP_ZERO);
	if (bif_vir_addr == NULL) {
		ret = -ENOMEM;
		goto exit_1;
	}
	bif_base = (struct bif_base_info *)(bif_vir_addr + 0);
	bif_ap = (struct bif_base_info *)(bif_vir_addr + BASE_PHY_SIZE);
	bif_self = bif_ap;
	bif_other = bif_base;
#else
#ifdef CONFIG_HOBOT_BIF_TEST
	bif_vir_addr = (void *)ioremap(base_phy_addr, 0x00100000);
	base_phy_addr_maxsize = 0x00100000;
#endif
	memset(bif_vir_addr, 0, 1024);
	bif_base = (struct bif_base_info *)(bif_vir_addr + 0);
	bif_ap = (struct bif_base_info *)(bif_vir_addr + BASE_PHY_SIZE);
	bif_self = bif_base;
	bif_other = bif_ap;
	base_phy_addr_offset = 1024;
#endif
	sprintf(bif_self->magic, "%s", BIF_BASE_MAGIC);
	memcpy(bif_self->magic, BIF_BASE_MAGIC, 4);
	memset(bif_self->irq, -1, IRQ_QUEUE_SIZE);
	ptr = (unsigned char *)bif_self;

	pr_info("bif_base: magic=%c%c%c%c,size=%x\n", ptr[0], ptr[1],
		ptr[2], ptr[3], (unsigned int)sizeof(struct bif_base_info));
#ifdef CONFIG_HOBOT_BIF_AP
	pr_info("bif_base=%lx,bif_ap=%lx,phy_add=%lx,vir_add=%lx\n",
	    (unsigned long)bif_base, (unsigned long)bif_ap,
	    base_phy_addr, (unsigned long)bif_vir_addr);
#else
	pr_info("bif_base=%lx,bif_ap=%lx,phy_add=%lx,vir_add=%lx,maxsize=%x\n",
		(unsigned long)bif_base, (unsigned long)bif_ap,
		base_phy_addr, (unsigned long)bif_vir_addr,
		base_phy_addr_maxsize);
#endif
	if (!irq_pin_absent) {
		bif_register_irq(BUFF_BASE, bif_base_irq_handler);
		bif_self->next_offset = sizeof(struct bif_base_info);
		init_waitqueue_head(&bif_irq_wq);
		INIT_WORK(&bif_irq_work, work_bif_irq);
	}
	bif_base_start = 1;

exit_1:
	return ret;
}

void _bif_base_exit(void)
{
	pr_info("bif_base: exit begin...\n");
	wake_up_all(&bif_irq_wq);
#ifdef CONFIG_HOBOT_BIF_AP
	wake_up_all(&bif_ap_wq);
	kfree(bif_vir_addr);
#else
	if (bif_vir_addr)
#ifdef CONFIG_HOBOT_BIF_TEST
		iounmap(bif_vir_addr);
#else
		memunmap(bif_vir_addr);
#endif
#endif

#ifdef CONFIG_HOBOT_BIF_TEST
	pr_info("bif_base: exit bif netlink...\n");
	bif_netlink_exit();
#else
	devm_free_irq(pbdev, irq_num, NULL);
	gpio_deinit();
#endif

	pr_info("bif_base: exit end...\n");
}

static int bif_base_probe(struct platform_device *pdev)
{
#ifdef CONFIG_HOBOT_BIF_AP
	unsigned int bif_base_phy_r = 0;
#else
	struct device_node *np = NULL;
	struct resource mem_reserved;
#endif
	int ret;

	dev_info(&pdev->dev, "probe begin...\n");
	pbdev = &pdev->dev;
	ret = of_property_read_u32(pdev->dev.of_node, "bif_base_irq_pin",
		&irq_pin);
	if (ret) {
		dev_err(&pdev->dev, "Filed to get bif_base_irq_pin\n");
		irq_pin_absent = 1;
	}
	ret = of_property_read_u32(pdev->dev.of_node, "bif_base_tri_pin",
		&tri_pin);
	if (ret) {
		dev_err(&pdev->dev, "Filed to get bif_base_tri_pin\n");
		irq_pin_absent = 1;
	}
#ifdef CONFIG_HOBOT_BIF_AP
	ret = of_property_read_u32(pdev->dev.of_node, "bif_base_phy_add",
		&bif_base_phy_r);
	if (ret) {
		dev_err(&pdev->dev, "Filed to get bif_base_phy_add\n");
		goto out;
	} else
		base_phy_addr = bif_base_phy_r;

	dev_info(&pdev->dev,
		 "Get CP Allocate reserved memory paddr=%lx,irq_pin=%d,tri_pin=%d",
		 base_phy_addr, irq_pin, tri_pin);
#else
	/* request memory address */
	np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!np) {
		dev_err(&pdev->dev, "No %s specified\n", "memory-region");
		goto out;
	}
	ret = of_address_to_resource(np, 0, &mem_reserved);
	if (ret) {
		dev_err(&pdev->dev, "No memory address assigned to the region\n");
		goto out;
	} else {
		base_phy_addr = mem_reserved.start;
		base_phy_addr_maxsize = resource_size(&mem_reserved);
		bif_vir_addr = (void *)memremap(mem_reserved.start,
			base_phy_addr_maxsize, MEMREMAP_WC); /*MEMREMAP_WB*/
		memset(bif_vir_addr, 0, base_phy_addr_maxsize);
	}
	dev_info(&pdev->dev,
		 "Allocate reserved memory,vaddr=%lx,paddr=%lx,irq_pin=%d,tri_pin=%d",
		 (unsigned long)bif_vir_addr, base_phy_addr, irq_pin, tri_pin);
#endif

	ret = _bif_base_init();
out:
	return ret;
}

static const struct of_device_id bif_base_of_match[] = {
	{.compatible = "hobot,bif_base"},
	{},
};

static int bif_base_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver bif_base_driver = {
	.driver = {
		   .name = "bif_base",
		   .of_match_table = bif_base_of_match,
		   },
	.probe = bif_base_probe,
	.remove = bif_base_remove,
};

static int __init bif_base_init(void)
{
	int ret;

	ret = platform_driver_register(&bif_base_driver);
	if (ret)
		pr_err("bif_base: Err bif base driver register failed.\n");
	else
		pr_info("bif_base: Suc bif base driver register success.\n");

#ifdef CONFIG_HOBOT_BIF_TEST
	_bif_base_init();
#endif
	return ret;
}

static void __exit bif_base_exit(void)
{
	bif_base_start = 0;
	_bif_base_exit();
	platform_driver_unregister(&bif_base_driver);
}

#ifdef CONFIG_X2_FPGA
late_initcall(bif_base_init);
#else
module_init(bif_base_init);
#endif
module_exit(bif_base_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("bif base module");
