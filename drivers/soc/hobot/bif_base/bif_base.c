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
#include <linux/sched/signal.h>
#include <linux/signal.h>
#include <linux/errno.h>

#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>

#include <linux/gpio.h>
#include <linux/io.h>
#include <linux/gpio/consumer.h>

#include "bif_base.h"
#include "bif_api.h"

#ifdef CONFIG_HOBOT_BIF_AP
#define BIF_BASE_MAGIC	"BIFA"
wait_queue_head_t bif_ap_wq;
#else
#define BIF_BASE_MAGIC	"BIFB"
#endif
#define pagesize (4*1024)
static struct bif_base_info *bif_base, *bif_ap, *bif_self, *bif_other;
/*0x7ff00000 vmware reversed address*/
unsigned long base_phy_addr = 0x7ff00000;
void *bif_vir_addr;
wait_queue_head_t bif_irq_wq;
struct work_struct bif_irq_work;	/* irq work */
static int irq_pin = 30;	//GPIO3_6 3*8+6=30
static int tri_pin = 31;
static int irq_num = -1;
static int bif_base_start;
static struct device *pbdev;

static int gpio_init(void)
{
	int ret;

	ret = gpio_request(tri_pin, "tri_pin");
	if (ret < 0) {
		pr_err("%s() Err get trigger pin ret= %d\n", __func__, ret);
		return ret;
	}
	gpio_direction_output(tri_pin, 0);

	ret = gpio_request(irq_pin, "irq_pin");
	if (ret < 0) {
		pr_err("%s() Err get irq pin ret= %d\n", __func__, ret);
		gpio_free(tri_pin);
		return ret;
	}
	irq_num = gpio_to_irq(irq_pin);
	if (irq_num < 0) {
		pr_err("%s() Err get irq num\n", __func__);
		gpio_free(tri_pin);
		gpio_free(irq_pin);
		ret = -ENODEV;
	}
	/*pr_info("%s() irq_num=%d ret=%d\n",__func__,irq_num,ret); */

	return ret;
}

static void gpio_deinit(void)
{
	gpio_free(tri_pin);
	gpio_free(irq_pin);
}

static void gpio_trigger_irq(int irq)
{
#ifdef CONFIG_HOBOT_BIF_TEST
	t_bif_send_irq(BUFF_BASE);
#else
	int val = 0;

	gpiod_direction_input(gpio_to_desc(irq));
	val = gpio_get_value(irq) & 0x01;
	val = ~val;
	gpio_direction_output(irq, val);
#endif
}

#ifdef CONFIG_HOBOT_BIF_AP
static int bif_sync_ap(void)
{
	char tbuf[521] = { 0 };

	if (!bif_base_start)
		return -1;
	memcpy(tbuf, (char *)bif_ap, 512);
#ifdef CONFIG_HOBOT_BIFSD
	if (!bif_sd_write((void *)(base_phy_addr + 512), 512, tbuf)) {
		pr_err("%s() Err Write by BIFSD for bif_ap fail\n", __func__);
		return -1;
	}
#else
#ifdef CONFIG_HOBOT_BIFSPI
	if (!bif_spi_write((void *)(base_phy_addr + 512), 512, tbuf)) {
		pr_err("%s() Err Write by BIFSPI for bif_ap fail\n", __func__);
		return -1;
	}
#endif
#endif
	return 0;
}

static int bif_sync_base(void)
{
	char tbuf[521] = { 0 };

	if (!bif_base_start)
		return -1;
#ifdef CONFIG_HOBOT_BIFSD
	if (!bif_sd_read((void *)base_phy_addr, 512, tbuf)) {
		pr_err("%s() Err Read by BIFSD for bif_base fail\n", __func__);
		return -1;
	}
#else
#ifdef CONFIG_HOBOT_BIFSPI
	if (!bif_spi_read((void *)base_phy_addr, 512, tbuf)) {
		pr_err("%s() Err Read by BIFSPI for bif_base fail\n", __func__);
		return -1;
	}
#endif
#endif
	memcpy((char *)bif_base, tbuf, 512);
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
	if ((bif_other->send_irq_tail + 1)
	    % IRQ_QUEUE_SIZE == bif_self->read_irq_head)
		irq_full = 1;

	while (bif_self->read_irq_head != bif_other->send_irq_tail) {
		if (!bif_base_start)
			return;

		birq =
		    bif_other->irq[(bif_self->read_irq_head) % IRQ_QUEUE_SIZE];
		if (birq < BUFF_MAX && bif_self->irq_func[birq % BUFF_MAX])
			bif_self->irq_func[birq % BUFF_MAX] (birq, NULL);
		else
			pr_err("%s() Err irq %d not register\n",
			       __func__, birq);

		bif_other->irq[(bif_self->read_irq_head) % IRQ_QUEUE_SIZE] = -1;
		bif_self->read_irq_head =
		    (bif_self->read_irq_head + 1) % IRQ_QUEUE_SIZE;
	}

#ifdef CONFIG_HOBOT_BIF_AP
	if (bif_sync_ap() != 0)
		pr_err("%s() Err sync ap\n", __func__);
#endif
	if (irq_full)
		bif_send_irq(BUFF_BASE);
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
	if (!bif_base_start)
		return 0;

	while ((bif_self->send_irq_tail + 1)
	       % IRQ_QUEUE_SIZE == bif_other->read_irq_head) {
		if (bif_base_start) {
			if (wait_event_interruptible_timeout(bif_irq_wq,
			     (bif_self->send_irq_tail + 1) % IRQ_QUEUE_SIZE !=
			     bif_other->read_irq_head, HZ * 10) == 0)
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
	if (buffer_id >= BUFF_MAX)
		return -1;
	bif_base->address_list[buffer_id] = address;
	bif_base->buffer_count++;
	bif_send_irq(BUFF_BASE);
	return 0;
}
EXPORT_SYMBOL(bif_register_address);

int bif_register_irq(enum BUFF_ID buffer_id, irq_handler_t irq_handler)
{
	if (buffer_id >= BUFF_MAX)
		return -1;
	bif_self->irq_func[buffer_id] = irq_handler;
	return buffer_id;
}
EXPORT_SYMBOL(bif_register_irq);

void *bif_query_address_wait(enum BUFF_ID buffer_id)
{
	void *ret = NULL;
	if (buffer_id >= BUFF_MAX)
		return (void *)-1;

#ifdef CONFIG_HOBOT_BIF_AP
	while (bif_base->address_list[buffer_id] == 0) {
		wait_event_interruptible(bif_ap_wq,
					 bif_base->address_list[buffer_id] !=
					 0);
		if (signal_pending(current))
			return (void *)-ERESTARTSYS;
	}
	ret = bif_base->address_list[buffer_id];
#endif

	return ret;
}
EXPORT_SYMBOL(bif_query_address_wait);

void *bif_query_address(enum BUFF_ID buffer_id)
{
	void *addr = NULL;

	if (buffer_id >= BUFF_MAX)
		return (void *)-1;

	addr = bif_base->address_list[buffer_id];
	if (addr != 0)
		return addr;
	return (void *)-1;
}
EXPORT_SYMBOL(bif_query_address);

void *bif_alloc_base(enum BUFF_ID buffer_id, int size)
{
	int offset;
	void *addr;

	if (buffer_id >= BUFF_MAX)
		return (void *)-1;

	if ((bif_self->next_offset + size) > 512)
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
	int offset = 0;
	void *addr;

	if (buffer_id >= BUFF_MAX)
		return (void *)-1;

	while (bif_other->offset_list[buffer_id] == 0) {
		wait_event_interruptible(bif_irq_wq,
					 bif_other->offset_list[buffer_id] !=
					 0);
		if (signal_pending(current))
			return (void *)-ERESTARTSYS;
	}

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

	if (buffer_id >= BUFF_MAX)
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
	gpio_init();
	ret = devm_request_irq(pbdev, irq_num, bif_irq_handler,
			       IRQ_TYPE_EDGE_BOTH, "bif_base", NULL);
	if (ret) {
		dev_err(pbdev,
			"Err request_irq for bif GPIO fail! irq_num=%d, ret=%d\n",
			irq_num, ret);
	}
#endif

#ifdef CONFIG_HOBOT_BIF_AP
	init_waitqueue_head(&bif_ap_wq);
	bif_vir_addr = kmalloc(1024, GFP_ATOMIC | __GFP_ZERO);
	if (!bif_vir_addr)
		return -ENOMEM;
	bif_base = (struct bif_base_info *)(bif_vir_addr + 0);
	bif_ap = (struct bif_base_info *)(bif_vir_addr + 512);
	bif_self = bif_ap;
	bif_other = bif_base;
#else
#ifdef CONFIG_HOBOT_BIF_TEST
	bif_vir_addr = (void *)ioremap(base_phy_addr, 0x00100000);
#endif
	memset(bif_vir_addr, 0, 1024);
	bif_base = (struct bif_base_info *)(bif_vir_addr + 0);
	bif_ap = (struct bif_base_info *)(bif_vir_addr + 512);
	bif_self = bif_base;
	bif_other = bif_ap;
#endif
	sprintf(bif_self->magic, "%s", BIF_BASE_MAGIC);
	memset(bif_self->irq, -1, IRQ_QUEUE_SIZE);
	ptr = (unsigned char *)bif_self;
	pr_info
	    ("magic=%c%c%c%c,size=0x%lx,base=%lx,ap=%lx,phy=0x%lx,vir=%lx\n",
	     ptr[0], ptr[1], ptr[2], ptr[3], sizeof(struct bif_base_info),
	     (unsigned long)bif_base, (unsigned long)bif_ap, base_phy_addr,
	     (unsigned long)bif_vir_addr);

	bif_register_irq(BUFF_BASE, bif_base_irq_handler);
	bif_self->next_offset = sizeof(struct bif_base_info);
	init_waitqueue_head(&bif_irq_wq);
	INIT_WORK(&bif_irq_work, work_bif_irq);

	bif_base_start = 1;
	return ret;
}

void _bif_base_exit(void)
{
	pr_info("%s() exit, begin...\n", __func__);
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
	pr_info("%s() exit, bif_netlink_exit\n", __func__);
	bif_netlink_exit();
#else
	//free_irq(irq_num,NULL);
	devm_free_irq(pbdev, irq_num, NULL);
	gpio_deinit();
#endif

	pr_info("%s() exit, end\n", __func__);
}

static int bif_base_probe(struct platform_device *pdev)
{
	struct device_node *np = NULL;
	int ret;
#ifdef CONFIG_HOBOT_BIF_AP
	unsigned int bif_base_phy_r = 0;
#else
	struct resource mem_reserved;
#endif

	/*dev_info(&pdev->dev,"probe, begin...\n"); */
	pbdev = &pdev->dev;

/*      match = of_match_node(bif_base_of_match, pdev->dev.of_node);*/
	ret =
	    of_property_read_u32(pdev->dev.of_node, "bif_base_irq_pin",
				 &irq_pin);
	if (ret) {
		dev_err(&pdev->dev, "Filed to get bif_base_irq_pin\n");
		goto out;
	}
	ret =
	    of_property_read_u32(pdev->dev.of_node, "bif_base_tri_pin",
				 &tri_pin);
	if (ret) {
		dev_err(&pdev->dev, "Filed to get bif_base_tri_pin\n");
		goto out;
	}

	/* request memory address */
	np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
	if (!np) {
		dev_err(&pdev->dev, "No %s specified\n", "memory-region");
		goto out;
	}
#ifdef CONFIG_HOBOT_BIF_AP
	ret =
	    of_property_read_u32(pdev->dev.of_node, "bif_base_phy_add",
				 &bif_base_phy_r);
	if (ret) {
		dev_err(&pdev->dev, "Filed to get bif_base_phy_add\n");
		goto out;
	} else
		base_phy_addr = bif_base_phy_r;
	dev_info(&pdev->dev,
		 "CP Allocate reserved memory,paddr=0x%0llx,irq_pin=%d,tri_pin=%d",
		 (uint64_t) base_phy_addr, irq_pin, tri_pin);
#else
	ret = of_address_to_resource(np, 0, &mem_reserved);
	if (ret) {
		dev_err(&pdev->dev,
			"No memory address assigned to the region\n");
		goto out;
	} else {
		base_phy_addr = mem_reserved.start;
		bif_vir_addr =
		    (void *)memremap(mem_reserved.start,
				     resource_size(&mem_reserved), MEMREMAP_WB);
	}
	dev_info(&pdev->dev,
		 "Rev memory,vaddr=0x%0llx,paddr=0x%0llx,irq_pin=%d,tri_pin=%d",
		 (uint64_t) bif_vir_addr, (uint64_t) base_phy_addr, irq_pin,
		 tri_pin);
#endif

	ret = _bif_base_init();
	/*dev_info(&pdev->dev, "probe, end... ret=%d",ret); */
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

module_init(bif_base_init);
module_exit(bif_base_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("By:hobot, 2018 horizon robotics.");
