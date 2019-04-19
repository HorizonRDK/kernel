/*************************************************************
 ****			 COPYRIGHT NOTICE
 ****		 Copyright	2019 Horizon Robotics, Inc.
 ****			 All rights reserved.
 *************************************************************/
/**
 * BIF base driver for memory and irq management
 * @version	2.0
 * @author	haibo.guo(haibo.guo@horizon.ai)
 * @date	2019/04/01
 */

/************************************************************
 *fpga(x2)	hogu(AP)
 *84-->		54(960)	 GPIO_EMIO_0
 *85<--		55(961)	 GPIO_EMIO_1
 *cp side
 *hobot-pinctrl.dtsi
 *gpioirq-bank-cfg = <85 0 0 0>;
 *arch/arm64/boot/dts/hobot-x2-fpga.dts
 *bifbase_reserved: bifbase_reserved@0x04000000 {
 *	reg = <0x0 0x04000000 0x0 0x00100000>;
 *	no-map;
 *};
 *bifbase {
 *	compatible = "hobot,bifbase";
 *	memory-region = <&bifbase_reserved>;
 *	bifbase_irq_pin = <85>;
 *	bifbase_tri_pin = <84>;
 *};

 *Ap side
 *bifbase {
 *	compatible = "hobot,bifbase";
 *	bifbase_phyaddr = <0x20000000>;
 *	bifbase_irq_pin = <960>;
 *	bifbase_tri_pin = <961>;
 *	bifspi = "yes";
 *	bifsd = "no";
 *	bifbase = "bifspi";
 *	bifeth = "bifsd";
 *	bifsmd = "bifspi";
 *	bifsio = "bifspi";
 *	biflite = "bifspi";
 *};
 ************************************************************/

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_address.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/vmalloc.h>

//#define CURR_LINUX_VERSION	(((3) << 16) + ((18) << 8) + (0))	//hi3519
//#define CURR_LINUX_VERSION	(((4) << 16) + ((9) << 8) + (0))	//xilinx
#define CURR_LINUX_VERSION	(((4) << 16) + ((14) << 8) + (74))	//x2
#define KERNEL_VERSION(a, b, c)	(((a) << 16) + ((b) << 8) + (c))
#if KERNEL_VERSION(4, 14, 14) <= CURR_LINUX_VERSION
#include <linux/sched/signal.h>
#endif

#include "bif_base.h"
#include "bif_api.h"

#define BIFBASE_APMAGIC		"BIFA"
#define BIFBASE_CPMAGIC		"BIFC"
#define BIFBASE_VER		"BIFBASE_V20"
#define BIFBASE_MAJOR		(123)
#define BIFBASE_BLOCK		(512)
#define BIFBASE_VER_SIZE	(16)
#define BIFBASE_PHYDDR_SIZE	(0x00100000)
#define BIFBASE_UPDATE_TIME	(5)
#define BIFBASE_CHECKPARAM	(0)
#define BIFBASE_UPDATEPARAM	(1)
#define BIFBASE_MEM_NC

static ulong bifbase_phyaddr;
static char *bifspi;
static char *bifsd;
char *bifbase;
char *bifeth;
char *bifsmd;
char *bifsio;
char *biflite;

module_param(bifbase_phyaddr, ulong, 0444);
module_param(bifspi, charp, 0644);
module_param(bifsd, charp, 0644);
module_param(bifbase, charp, 0644);
module_param(bifeth, charp, 0644);
module_param(bifsmd, charp, 0644);
module_param(bifsio, charp, 0644);
module_param(biflite, charp, 0644);
MODULE_PARM_DESC(bifbase_phyaddr, "bifbase: cp side reserved ddr memory for bifbase");
MODULE_PARM_DESC(bifspi, "bifbase: ap side whether support bifspi driver? no or yes");
MODULE_PARM_DESC(bifsd, "bifbase: ap side whether support bifsd driver? no or yes");
MODULE_PARM_DESC(bifbase, "bifbase: ap side, Which channel does bifbase support? bifno, bifspi or bifsd");
MODULE_PARM_DESC(bifeth, "bifbase: ap side, Which channel does bifeth support? bifno, bifspi or bifsd");
MODULE_PARM_DESC(bifsmd, "bifbase: ap side, Which channel does bifsmd support? bifno, bifspi or bifsd");
MODULE_PARM_DESC(bifsio, "bifbase: ap side, Which channel does bifsio support? bifno, bifspi or bifsd");
MODULE_PARM_DESC(biflite, "bifbase: ap side, Which channel does biflite support? bifno, bifspi or bifsd");

struct bifbase_local {
	char ver[BIFBASE_VER_SIZE];
	int start;
	struct bif_base_info *cp, *ap, *self, *other;
	struct work_struct base_irq_work;
	irq_handler_t irq_func[BUFF_MAX];

	wait_queue_head_t base_ap_wq;
	wait_queue_head_t base_irq_wq;
	int base_ap_wq_flg;
	int base_irq_wq_flg;

	void *bifbase_viraddr;
	int bifbase_phyaddroffset;
	int bifbase_phyaddrsize;

	struct bifplat_info *plat;
	enum BUFF_ID bifbase_id;
	int bifbase_channel;

	struct platform_device *pdev;
	struct class *class;
	struct device *dev;

	struct timer_list bifbase_timer;
//bifbase param
	ulong bifbase_phyaddr;
	char *bifspi;
	char *bifsd;
	char *bifbase;
	char *bifeth;
	char *bifsmd;
	char *bifsio;
	char *biflite;
};

struct bifbase_local *pbl;
struct bifplat_info bifplat;

extern int bifget_supportbus(char *str_bus);
extern int bifget_bifbustype(char *str_bustype);
extern void bifplat_print_info(void *p);
extern void bifplat_trigger_irq(void *p);
extern int bifplat_gpio_init(void *p);
extern void bifplat_gpio_deinit(void *p);
extern int bifplat_register_irq(enum BUFF_ID buffer_id,
	irq_handler_t irq_handler);
extern int bifplat_config(void *p);
extern void bifplat_unconfig(void *p);
//pr_bif
extern int bifdebug;
#define pr_bif(fmt, args...) do {if (bifdebug) pr_err(fmt, ##args); } while (0)

static struct bifbase_local *get_bifbase_local(void)
{
	return pbl;
}

static int get_bifbase_channel(void *p)
{
	struct bifbase_local *pl = (struct bifbase_local *)p;

	if (!pl || !pl->plat)
		return BIFBUS_NO;

	return bifget_bifbustype(pl->bifbase);
}

static void bifbase_print_param(void)
{
	pr_info("bifbase phyaddr: 0x%lx\n", bifbase_phyaddr);
	pr_info("bifspi: %s\n", bifspi);
	pr_info("bifsd: %s\n", bifsd);
	pr_info("bifbase: %s\n", bifbase);
	pr_info("bifeth: %s\n", bifeth);
	pr_info("bifsmd: %s\n", bifsmd);
	pr_info("bifsio: %s\n", bifsio);
	pr_info("biflite: %s\n", biflite);
}

static void bifbase_check_bus(void *p, int flag)
{
	struct bifbase_local *pl = (struct bifbase_local *)p;

	if (!pl || !pl->plat)
		return;

	if (pl->plat->plat_type == PLAT_AP) {
		if (pl->plat->bifspi == SUPPORT_NO) {
			bifspi = STR_SUPPORT_NO;
			pl->bifspi = STR_SUPPORT_NO;
		} else {
			if (bifget_supportbus(bifspi) == SUPPORT_MAX) {
				if (flag == BIFBASE_CHECKPARAM) {
					if (bifget_supportbus(pl->bifspi)
						== SUPPORT_MAX) {
						bifspi = STR_SUPPORT_YES;
						pl->bifspi = STR_SUPPORT_YES;
					} else
						bifspi = pl->bifspi;
				} else {
					pl->bifspi = bifspi;
				}
			} else
				pl->bifspi = bifspi;
		}

		if (pl->plat->bifsd == SUPPORT_NO) {
			bifsd = STR_SUPPORT_NO;
			pl->bifsd = STR_SUPPORT_NO;
		} else {
			if (bifget_supportbus(bifsd) == SUPPORT_MAX) {
				if (flag == BIFBASE_CHECKPARAM) {
					if (bifget_supportbus(pl->bifsd)
						== SUPPORT_MAX) {
						bifsd = STR_SUPPORT_YES;
						pl->bifsd = STR_SUPPORT_YES;
					} else
						bifsd = pl->bifsd;
				} else {
					pl->bifsd = bifsd;
				}
			} else
				pl->bifsd = bifsd;
		}
	} else {
		bifspi = STR_SUPPORT_NO;
		bifsd = STR_SUPPORT_NO;
		pl->bifspi = STR_SUPPORT_NO;
		pl->bifsd = STR_SUPPORT_NO;
	}

	pr_bif("bifbase: %s()-%d bifspi=%s, bifsd=%s\n",
		__func__, __LINE__, bifspi, bifsd);
}

static void bifbase_check_type(
	void *p, int flag, char **str_type1, char **str_type2)
{
	int bustype;
	struct bifbase_local *pl = (struct bifbase_local *)p;

	if (!pl || !pl->plat)
		return;

	if (pl->plat->plat_type == PLAT_AP) {
		bustype = bifget_bifbustype(*str_type1);
		if (flag == BIFBASE_CHECKPARAM) {
			if (bustype == BIFBUS_NO || bustype == BIFBUS_MAX)
				bustype = bifget_bifbustype(*str_type2);
		}
		switch (bustype) {
		case BIFBUS_NO:
		case BIFBUS_MAX:
			//default, bifsd first
			if (pl->plat->bifsd == SUPPORT_YES
				&& bifget_supportbus(bifsd) == SUPPORT_YES) {
				*str_type1 = STR_BIFBUS_SD;
				*str_type2 = STR_BIFBUS_SD;
			} else if (pl->plat->bifspi == SUPPORT_YES
				&& bifget_supportbus(bifspi) == SUPPORT_YES) {
				*str_type1 = STR_BIFBUS_SPI;
				*str_type2 = STR_BIFBUS_SPI;
			} else {
				*str_type1 = STR_BIFBUS_NO;
				*str_type1 = STR_BIFBUS_NO;
			}
			break;
		case BIFBUS_SPI:
			if (pl->plat->bifspi == SUPPORT_YES
				&& bifget_supportbus(bifspi) == SUPPORT_YES) {
				*str_type1 = STR_BIFBUS_SPI;
				*str_type2 = STR_BIFBUS_SPI;
			} else if (pl->plat->bifsd == SUPPORT_YES
				&& bifget_supportbus(bifsd) == SUPPORT_YES) {
				*str_type1 = STR_BIFBUS_SD;
				*str_type2 = STR_BIFBUS_SD;
			} else {
				*str_type1 = STR_BIFBUS_NO;
				*str_type1 = STR_BIFBUS_NO;
			}
			break;
		case BIFBUS_SD:
			if (pl->plat->bifsd == SUPPORT_YES
				&& bifget_supportbus(bifsd) == SUPPORT_YES) {
				*str_type1 = STR_BIFBUS_SD;
				*str_type2 = STR_BIFBUS_SD;
			} else if (pl->plat->bifspi == SUPPORT_YES
				&& bifget_supportbus(bifspi) == SUPPORT_YES) {
				*str_type1 = STR_BIFBUS_SPI;
				*str_type2 = STR_BIFBUS_SPI;
			} else {
				*str_type1 = STR_BIFBUS_NO;
				*str_type1 = STR_BIFBUS_NO;
			}
			break;
		}
	} else {
		*str_type1 = STR_BIFBUS_NO;
		*str_type1 = STR_BIFBUS_NO;
	}

	pr_bif("bifbase: %s()-%d str_type1=%s, str_type2=%s\n",
		__func__, __LINE__, *str_type1, *str_type2);
}

static void bifbase_check_param(void *p)
{
	struct bifbase_local *pl = (struct bifbase_local *)p;

	if (!pl || !pl->plat)
		return;

	if (bifbase_phyaddr) {
		pl->bifbase_phyaddr = bifbase_phyaddr;
		pl->plat->bifbase_phyaddr = bifbase_phyaddr;
	} else {
		pl->bifbase_phyaddr = pl->plat->bifbase_phyaddr;
		bifbase_phyaddr = pl->plat->bifbase_phyaddr;
	}

	bifbase_check_bus((void *)pl, BIFBASE_CHECKPARAM);
	bifbase_check_type((void *)pl, BIFBASE_CHECKPARAM,
		&bifbase, &pl->bifbase);
	bifbase_check_type((void *)pl, BIFBASE_CHECKPARAM,
		&bifeth, &pl->bifeth);
	bifbase_check_type((void *)pl, BIFBASE_CHECKPARAM,
		&bifsmd, &pl->bifsmd);
	bifbase_check_type((void *)pl, BIFBASE_CHECKPARAM,
		&bifsio, &pl->bifsio);
	bifbase_check_type((void *)pl, BIFBASE_CHECKPARAM,
		&biflite, &pl->biflite);
}

static void bifbase_update_param_func(unsigned long data)
{
	struct bifbase_local *pl = (struct bifbase_local *)data;

	if (!pl || !pl->plat)
		return;

	bifbase_check_bus((void *)pl, BIFBASE_UPDATEPARAM);
	bifbase_check_type((void *)pl, BIFBASE_UPDATEPARAM,
		&bifbase, &pl->bifbase);
	bifbase_check_type((void *)pl, BIFBASE_UPDATEPARAM,
		&bifeth, &pl->bifeth);
	bifbase_check_type((void *)pl, BIFBASE_UPDATEPARAM,
		&bifsmd, &pl->bifsmd);
	bifbase_check_type((void *)pl, BIFBASE_UPDATEPARAM,
		&bifsio, &pl->bifsio);
	bifbase_check_type((void *)pl, BIFBASE_UPDATEPARAM,
		&biflite, &pl->biflite);

	mod_timer(&pl->bifbase_timer, jiffies + BIFBASE_UPDATE_TIME * HZ);
}

static void bifbase_tri_irq(void *p)
{
	struct bifbase_local *pl = (struct bifbase_local *)p;

	if (!pl || !pl->start || !pl->plat || !pl->self || !pl->other)
		return;

	bifplat_trigger_irq((void *)pl->plat);
}

static int bifbase_sync_ap(void *p)
{
	unsigned char tbuf[BIFBASE_BLOCK + 16];
	unsigned long cur_phy;
	unsigned int cur_len;
	struct bifbase_local *pl = (struct bifbase_local *)p;

	if (!pl || !pl->start || !pl->plat || !pl->self || !pl->other)
		return -3;

	if (pl->plat->plat_type == PLAT_AP) {
		pl->bifbase_channel = get_bifbase_channel((void *)pl);
		if (pl->bifbase_channel == BIFBUS_SD)
			cur_len = BIFBASE_BLOCK;
		else if (pl->bifbase_channel == BIFBUS_SPI)
			cur_len = MULTI(pl->self->next_offset, 16);
		else
			return -2;

		cur_phy = pl->bifbase_phyaddr + BIFBASE_BLOCK;
		memcpy(tbuf, (unsigned char *)pl->ap, cur_len);
		if (bifwrite(pl->bifbase_channel,
			(void *)cur_phy, cur_len, tbuf))
			return -1;
	}

	return 0;
}

int bifbase_sync_cp(void *p)
{
	unsigned char tbuf[BIFBASE_BLOCK + 16];
	unsigned long cur_phy;
	unsigned int cur_len;
	struct bifbase_local *pl = (struct bifbase_local *)p;

	if (!pl || !pl->start || !pl->plat || !pl->self || !pl->other)
		return -3;

	if (pl->plat->plat_type == PLAT_AP) {
		pl->bifbase_channel = get_bifbase_channel((void *)pl);
		if (pl->bifbase_channel == BIFBUS_SD)
			cur_len = BIFBASE_BLOCK;
		else if (pl->bifbase_channel == BIFBUS_SPI)
			cur_len = MULTI(pl->self->next_offset, 16);
		else
			return -2;

		cur_phy = pl->bifbase_phyaddr;
		if (bifread(pl->bifbase_channel,
			(void *)cur_phy, cur_len, tbuf))
			return -1;

		memcpy((unsigned char *)pl->cp, tbuf, cur_len);
	}

	return 0;
}

static irqreturn_t bifbase_baseirq_handler(int irq, void *data)
{
	struct bifbase_local *pl = get_bifbase_local();

	if (!pl || !pl->start || !pl->plat || !pl->self || !pl->other)
		return IRQ_NONE;

	if (pl->plat->plat_type == PLAT_AP)
		wake_up_interruptible(&pl->base_ap_wq);

	return IRQ_HANDLED;
}

static void bifbase_irq_work(struct work_struct *work)
{
	int birq;
	int irq_full = 0;
	struct bifbase_local *pl =
		container_of(work, struct bifbase_local, base_irq_work);

	if (!pl || !pl->start || !pl->plat || !pl->self || !pl->other)
		return;

	if (bifbase_sync_cp((void *)pl) != 0)
		pr_err("bifbase: %s() Err sync base\n", __func__);

	if ((pl->other->send_irq_tail + 1) % IRQ_QUEUE_SIZE ==
		pl->self->read_irq_head)
		irq_full = 1;

	while (pl->self->read_irq_head != pl->other->send_irq_tail) {
		if (!pl->start)
			return;

		birq = pl->other->irq[(pl->self->read_irq_head) %
			IRQ_QUEUE_SIZE];
		if (birq < BUFF_MAX && pl->irq_func[birq % BUFF_MAX])
			pl->irq_func[birq % BUFF_MAX] (birq, NULL);
		else
			pr_bif("bifbase: %s() Warn irq %d no register\n",
				__func__, birq);

		pl->other->irq[(pl->self->read_irq_head) % IRQ_QUEUE_SIZE] = -1;
		pl->self->read_irq_head =
			(pl->self->read_irq_head + 1) % IRQ_QUEUE_SIZE;
	}

	if (pl->base_ap_wq_flg)
		wake_up_interruptible(&pl->base_ap_wq);
	if (bifbase_sync_ap((void *)pl) != 0)
		pr_err("bifbase: %s() Err sync ap\n", __func__);

	if (irq_full)
		bif_send_irq(pl->bifbase_id);
	if (pl->base_irq_wq_flg)
		wake_up_interruptible(&pl->base_irq_wq);
}

static irqreturn_t bifbase_irq_handler(int irq, void *data)
{
	struct bifbase_local *pl = (struct bifbase_local *)data;
	//struct bifbase_local *pl = get_bifbase_local();

	if (!pl || !pl->start || !pl->plat || !pl->self || !pl->other)
		return IRQ_NONE;

	schedule_work(&pl->base_irq_work);

	return IRQ_HANDLED;
}

static void *bif_memset(void *s, int c, size_t count)
{
	char *xs = s;

	while (count--)
		*xs++ = c;
	return s;
}

static int bifbase_pre_init(void *p)
{
	int ret;
	unsigned char *ptr = NULL;
	struct bifbase_local *pl = (struct bifbase_local *)p;

	if (!pl || !pl->plat) {
		ret = -1;
		goto exit_1;
	}

	ret = bifplat_gpio_init((void *)pl->plat);
	if (ret)
		goto exit_1;

	bifbase_check_param((void *)pl);
	bifbase_print_param();

	ret = bifplat_register_irq(pl->bifbase_id, bifbase_irq_handler);
	if (ret == -BIFERR && !pl->plat->irq_pin_absent) {
		ret = devm_request_irq(pl->dev, pl->plat->irq_num,
			bifbase_irq_handler, IRQ_TYPE_EDGE_BOTH, "bifbase",
			(void *)pl);
		if (ret) {
			dev_err(pl->dev, "Err request irq_num=%d, ret=%d\n",
				pl->plat->irq_num, ret);
			goto exit_2;
		}
	}

	if (pl->plat->plat_type == PLAT_AP) {
		init_timer(&pl->bifbase_timer);
		pl->bifbase_timer.function = bifbase_update_param_func;
		pl->bifbase_timer.expires = jiffies + BIFBASE_UPDATE_TIME * HZ;
		pl->bifbase_timer.data = (unsigned long)pl;
		add_timer(&pl->bifbase_timer);

		init_waitqueue_head(&pl->base_ap_wq);
		pl->bifbase_phyaddrsize = 2 * BIFBASE_BLOCK;
		pl->bifbase_viraddr =
			kzalloc(pl->bifbase_phyaddrsize, GFP_KERNEL);
		if (pl->bifbase_viraddr == NULL) {
			ret = -ENOMEM;
			goto exit_3;
		}
		pl->cp = (struct bif_base_info *)(pl->bifbase_viraddr + 0);
		pl->ap = (struct bif_base_info *)(pl->bifbase_viraddr +
			BIFBASE_BLOCK);
		pl->self = pl->ap;
		pl->other = pl->cp;
		pl->bifbase_phyaddroffset = 2 * BIFBASE_BLOCK;
		memcpy(pl->self->magic, BIFBASE_APMAGIC, 4);
	} else {
		if (!pl->bifbase_phyaddrsize)
			pl->bifbase_phyaddrsize = BIFBASE_PHYDDR_SIZE;

		pr_info("bifbase: phy=0x%lx,vir=0x%lx,size=0x%x\n",
			pl->bifbase_phyaddr, (ulong)pl->bifbase_viraddr,
			pl->bifbase_phyaddrsize);

		//pl->bifbase_viraddr = bif_ram_vmap(pl->bifbase_phyaddr,
		//	pl->bifbase_phyaddrsize, BIF_MT_WC);//no data update
		//pl->bifbase_viraddr = (void *)memremap(pl->bifbase_phyaddr,
		//	pl->bifbase_phyaddrsize, MEMREMAP_WC);//ok,hisi?
#ifdef BIFBASE_MEM_NC
		pr_info("bifbase: call ioremap_nocache()\n");
		pl->bifbase_viraddr = (void *)ioremap_nocache(
			pl->bifbase_phyaddr, pl->bifbase_phyaddrsize);
#else
		pr_info("bifbase: call ioremap_wc()\n");
		pl->bifbase_viraddr = (void *)ioremap_wc(
			pl->bifbase_phyaddr, pl->bifbase_phyaddrsize);
#endif
		bif_memset(pl->bifbase_viraddr, 0, pl->bifbase_phyaddrsize);

		pl->cp = (struct bif_base_info *)(pl->bifbase_viraddr + 0);
		pl->ap = (struct bif_base_info *)(pl->bifbase_viraddr +
			BIFBASE_BLOCK);
		pl->self = pl->cp;
		pl->other = pl->ap;
		pl->bifbase_phyaddroffset = 2 * BIFBASE_BLOCK;
		memcpy(pl->self->magic, BIFBASE_CPMAGIC, 4);
	}

	bif_memset(pl->self->irq, -1, IRQ_QUEUE_SIZE);
	ptr = (unsigned char *)pl->self;
	pr_info("bifbase: magic=%c%c%c%c,size=0x%x\n", ptr[0], ptr[1],
		ptr[2], ptr[3], (unsigned int)sizeof(struct bif_base_info));

	pr_info("bifbase: cp=0x%lx,ap=0x%lx,phy=0x%lx,vir=0x%lx,size=0x%x\n",
		(ulong)pl->cp, (ulong)pl->ap, pl->bifbase_phyaddr,
		(ulong)pl->bifbase_viraddr, pl->bifbase_phyaddrsize);

	pl->self->next_offset = sizeof(struct bif_base_info);
	if (!pl->plat->irq_pin_absent) {
		bif_register_irq(pl->bifbase_id, bifbase_baseirq_handler);
		init_waitqueue_head(&pl->base_irq_wq);
		INIT_WORK(&pl->base_irq_work, bifbase_irq_work);
	}
	pl->start = 1;
	return 0;
exit_3:
	devm_free_irq(pl->dev, pl->plat->irq_num, (void *)pl);
exit_2:
	bifplat_gpio_deinit((void *)pl->plat);
exit_1:
	return ret;
}

void bifbase_pre_exit(void *p)
{
	struct bifbase_local *pl = (struct bifbase_local *)p;

	if (!pl || !pl->plat)
		return;

	pr_info("bifbase: pre exit begin...\n");

	if (!pl->plat->irq_pin_absent)
		wake_up_all(&pl->base_irq_wq);

	if (pl->plat->plat_type == PLAT_AP) {
		del_timer(&pl->bifbase_timer);
		wake_up_all(&pl->base_ap_wq);
		kfree(pl->bifbase_viraddr);
	} else {
		//bif_ram_vunmap(pl->bifbase_viraddr, pl->bifbase_phyaddrsize);
		//memunmap(pl->bifbase_viraddr);
		iounmap(pl->bifbase_viraddr);
	}

	if (!pl->plat->irq_pin_absent)
		devm_free_irq(pl->dev, pl->plat->irq_num, (void *)pl);

	bifplat_gpio_deinit((void *)pl->plat);
}


static struct file_operations bifbase_fops = {
	.owner = THIS_MODULE,
};

static int bifbase_driver_register(void *p)
{
	int ret;
	struct bifbase_local *pl = (struct bifbase_local *)p;

	if (!pl || !pl->plat) {
		ret = -1;
		goto exit_1;
	}

	pl->class = class_create(THIS_MODULE, "bifbase");
	if (IS_ERR(pl->class)) {
		pr_err("bifbase: Err class create\n");
		ret = PTR_ERR(pl->class);
		goto exit_1;
	}

	ret = register_chrdev(BIFBASE_MAJOR, "bifbase", &bifbase_fops);
	if (ret < 0) {
		pr_err("bifbase: Err register chrdev\n");
		class_destroy(pl->class);
		goto exit_1;
	}

	pl->dev = device_create(pl->class, NULL, MKDEV(BIFBASE_MAJOR, 0),
		NULL, "bifbase");
	if (IS_ERR(pl->dev)) {
		pr_err("bifbase: Err device create\n");
		unregister_chrdev(BIFBASE_MAJOR, "bifbase");
		class_destroy(pl->class);
		ret = -1;
		goto exit_1;
	}

	ret = bifbase_pre_init((void *)pl);
exit_1:
	return ret;
}

static void bifbase_driver_unregister(void *p)
{
	struct bifbase_local *pl = (struct bifbase_local *)p;

	if (!pl || !pl->plat)
		return;

	pr_info("bifbase: driver unregister begin...\n");

	device_destroy(pl->class, MKDEV(BIFBASE_MAJOR, 0));
	class_destroy(pl->class);
	unregister_chrdev(BIFBASE_MAJOR, "bifbase");
}

static int bifbase_probe(struct platform_device *pdev)
{
	int ret;
	unsigned int tmp_phyaddr = 0;
	struct device_node *np = NULL;
	struct resource mem_reserved;
	struct bifbase_local *pl = get_bifbase_local();

	dev_info(&pdev->dev, "probe begin...\n");

	if (!pl || !pl->plat) {
		ret = -1;
		dev_info(&pdev->dev, "Err probe pl=NULL or pl->plat=NULL\n");
		goto exit_1;
	}

	pl->pdev = pdev;
	pl->dev = &pdev->dev;
	ret = of_property_read_u32(pdev->dev.of_node, "bifbase_irq_pin",
		&pl->plat->irq_pin);
	if (ret) {
		dev_err(&pdev->dev, "Filed to get bifbase_irq_pin\n");
		pl->plat->irq_pin_absent = 1;
	}
	ret = of_property_read_u32(pdev->dev.of_node, "bifbase_tri_pin",
		&pl->plat->tri_pin);
	if (ret) {
		dev_err(&pdev->dev, "Filed to get bifbase_tri_pin\n");
		pl->plat->irq_pin_absent = 1;
	}

	if (pl->plat->plat_type == PLAT_AP) {
		ret = of_property_read_u32(pdev->dev.of_node,
			"bifbase_phyaddr", &tmp_phyaddr);
		if (ret) {
			dev_err(&pdev->dev, "Filed to get bifbase_phyaddr\n");
			goto exit_1;
		} else
			pl->plat->bifbase_phyaddr = tmp_phyaddr;

		if (of_property_read_string(pdev->dev.of_node, "bifspi",
			(const char **)&pl->bifspi))
			dev_err(&pdev->dev, "Filed to get bifspi\n");
		if (of_property_read_string(pdev->dev.of_node, "bifsd",
			(const char **)&pl->bifsd))
			dev_err(&pdev->dev, "Filed to get bifsd\n");
		if (of_property_read_string(pdev->dev.of_node, "bifbase",
			(const char **)&pl->bifbase))
			dev_err(&pdev->dev, "Filed to get bifbase\n");
		if (of_property_read_string(pdev->dev.of_node, "bifeth",
			(const char **)&pl->bifeth))
			dev_err(&pdev->dev, "Filed to get bifeth\n");
		if (of_property_read_string(pdev->dev.of_node, "bifsmd",
			(const char **)&pl->bifsmd))
			dev_err(&pdev->dev, "Filed to get bifsmd\n");
		if (of_property_read_string(pdev->dev.of_node, "bifsio",
			(const char **)&pl->bifsio))
			dev_err(&pdev->dev, "Filed to get bifsio\n");
		if (of_property_read_string(pdev->dev.of_node, "biflite",
			(const char **)&pl->biflite))
			dev_err(&pdev->dev, "Filed to get biflite\n");
	} else {
		/* request memory address */
		np = of_parse_phandle(pdev->dev.of_node, "memory-region", 0);
		if (!np) {
			dev_err(&pdev->dev,
				"No %s specified\n", "memory-region");
			goto exit_1;
		}
		ret = of_address_to_resource(np, 0, &mem_reserved);
		if (ret) {
			dev_err(&pdev->dev,
				"No memory address assigned to the region\n");
			goto exit_1;
		} else {
			pl->plat->bifbase_phyaddr = mem_reserved.start;
			pl->plat->bifbase_phyaddrsize =
				resource_size(&mem_reserved);
		}
	}

	dev_info(&pdev->dev,
		"CP reserved mem paddr=0x%lx,size=0x%x,irq_pin=%d,tri_pin=%d",
		pl->plat->bifbase_phyaddr, pl->plat->bifbase_phyaddrsize,
		pl->plat->irq_pin, pl->plat->tri_pin);

	ret = bifbase_pre_init((void *)pl);

exit_1:
	return ret;
}

static int bifbase_remove(struct platform_device *pdev)
{
	pr_info("bifbase: bifbase remove begin...\n");

	return 0;
}

static const struct of_device_id bifbase_of_match[] = {
	{.compatible = "hobot,bifbase"},
	{},
};

static struct platform_driver bifbase_driver = {
	.driver = {
		   .name = "bifbase",
		   .of_match_table = bifbase_of_match,
		   },
	.probe = bifbase_probe,
	.remove = bifbase_remove,
};

static int __init bifbase_init(void)
{
	int ret;
	struct bifbase_local *pl;

	pr_info("bifbase: init begin\n");

	pl = kzalloc(sizeof(struct bifbase_local), GFP_KERNEL);
	if (pl == NULL) {
		ret = -ENOMEM;
		goto exit_1;
	}

	pl->start = 0;
	pl->bifbase_id = BUFF_BASE;
	pl->plat = &bifplat;
	pbl = pl;
	sprintf(pl->ver, "%s", BIFBASE_VER);
	pr_info("bifbase: %s\n", pl->ver);

	ret = bifplat_config((void *)pl->plat);
	if (ret)
		goto exit_1;

	//bifplat_print_info((void *)pl->plat);

	if (pl->plat->param == PARAM_MODULE)
		ret = bifbase_driver_register((void *)pl);
	else
		ret = platform_driver_register(&bifbase_driver);

	if (ret) {
		bifplat_unconfig((void *)pl->plat);
		kfree(pl);
exit_1:
		pr_err("bifbase: Err bifbase driver register failed.\n");
	} else {
		pr_info("bifbase: Suc bifbase driver register success.\n");
	}
	return ret;
}

static void __exit bifbase_exit(void)
{
	struct bifbase_local *pl = get_bifbase_local();

	if (!pl || !pl->start || !pl->plat || !pl->self || !pl->other)
		return;

	pr_info("bifbase: exit begin...\n");

	pl->start = 0;
	bifbase_pre_exit((void *)pl);

	if (pl->plat->param == PARAM_MODULE)
		bifbase_driver_unregister((void *)pl);
	else
		platform_driver_unregister(&bifbase_driver);

	bifplat_unconfig((void *)pl->plat);

	kfree(pl);

	pr_info("bifbase: exit end...\n");
}

void *bif_ram_vmap(phys_addr_t start, size_t size, unsigned int memtype)
{
	struct page **pages;
	phys_addr_t page_start;
	unsigned int page_count;
	pgprot_t prot;
	unsigned int i;
	void *vaddr;

	page_start = start - offset_in_page(start);
	page_count = DIV_ROUND_UP(size + offset_in_page(start), PAGE_SIZE);

	switch (memtype) {
	case BIF_MT_WB:
		prot = PAGE_KERNEL;
		break;
	case BIF_MT_UC:
		prot = pgprot_noncached(PAGE_KERNEL);
		break;
	case BIF_MT_WC:
		prot = pgprot_writecombine(PAGE_KERNEL);
		break;
//	case BIF_MT_WT:
//		prot = __pgprot(PROT_NORMAL_WT);
//		break;
	default:
		/* Default set normal memory(cacheable) */
		prot = PAGE_KERNEL;
	}

	pages = kmalloc_array(page_count, sizeof(struct page *), GFP_KERNEL);
	if (!pages) {
		pr_err("%s: Failed to allocate array for %u pages\n",
			 __func__, page_count);
		return NULL;
	}

	for (i = 0; i < page_count; i++) {
		phys_addr_t addr = page_start + i * PAGE_SIZE;
		pages[i] = pfn_to_page(addr >> PAGE_SHIFT);
	}
	vaddr = vm_map_ram(pages, page_count, -1, prot);
	kfree(pages);

	return vaddr;
}

void bif_ram_vunmap(const void *mem, size_t size)
{
	vm_unmap_ram(mem, size / PAGE_SIZE);
}

int bif_send_irq(int irq)
{
	struct bifbase_local *pl = get_bifbase_local();

	if (!pl || !pl->start || !pl->plat || !pl->self || !pl->other)
		return 0;

	while ((pl->self->send_irq_tail + 1) % IRQ_QUEUE_SIZE ==
	       pl->other->read_irq_head) {
		if (pl->start) {
			if (bifbase_sync_ap((void *)pl) != 0)
				pr_err("bifbase: %s() Err sync ap\n", __func__);
			bifbase_tri_irq((void *)pl);
			if (wait_event_interruptible_timeout(pl->base_irq_wq,
				(pl->self->send_irq_tail + 1) %	IRQ_QUEUE_SIZE
				!= pl->other->read_irq_head,
				msecs_to_jiffies(2000)) == 0)
				goto try_send_irq;
		} else
			return 0;
	}

	pl->self->irq[(pl->self->send_irq_tail) % IRQ_QUEUE_SIZE] = irq;
	pl->self->send_irq_tail =
		(pl->self->send_irq_tail + 1) % IRQ_QUEUE_SIZE;

try_send_irq:
	if (bifbase_sync_ap((void *)pl) != 0)
		pr_err("bifbase: %s() Err sync ap\n", __func__);

	bifbase_tri_irq((void *)pl);

	return 0;
}
EXPORT_SYMBOL(bif_send_irq);

int bif_register_address(enum BUFF_ID buffer_id, void *address)
{
	unsigned long addr = (unsigned long)address;
	struct bifbase_local *pl = get_bifbase_local();

	if (!pl || !pl->start || !pl->plat || !pl->cp || buffer_id >= BUFF_MAX)
		return -1;

	if (pl->plat->plat_type == PLAT_CP) {
		pl->cp->address_list[buffer_id] = addr;
		pl->cp->buffer_count++;
		if (!pl->plat->irq_pin_absent)
			bif_send_irq(pl->bifbase_id);
	}

	return 0;
}
EXPORT_SYMBOL(bif_register_address);

int bif_register_irq(enum BUFF_ID buffer_id, irq_handler_t irq_handler)
{
	struct bifbase_local *pl = get_bifbase_local();

	if (!pl || !pl->start || !pl->plat || buffer_id >= BUFF_MAX)
		return -1;

	pl->irq_func[buffer_id] = irq_handler;

	return buffer_id;
}
EXPORT_SYMBOL(bif_register_irq);

void *bif_query_address_wait(enum BUFF_ID buffer_id)
{
	unsigned long addr = 0;
	struct bifbase_local *pl = get_bifbase_local();

	if (!pl || !pl->start || !pl->plat || !pl->cp || buffer_id >= BUFF_MAX)
		return (void *)-1;

	if (pl->plat->plat_type == PLAT_AP) {
		while (pl->cp->address_list[buffer_id] == 0) {
			pl->base_irq_wq_flg = 1;
			wait_event_interruptible_timeout(pl->base_ap_wq,
				pl->cp->address_list[buffer_id] != 0,
				msecs_to_jiffies(2000));
			if (signal_pending(current))
				return (void *)-ERESTARTSYS;
		}
		addr = pl->cp->address_list[buffer_id];
		pl->base_irq_wq_flg = 0;
	}

	return (void *)addr;
}
EXPORT_SYMBOL(bif_query_address_wait);

void *bif_query_address(enum BUFF_ID buffer_id)
{
	unsigned long addr = 0;
	struct bifbase_local *pl = get_bifbase_local();

	if (!pl || !pl->start || !pl->plat || !pl->cp || buffer_id >= BUFF_MAX)
		return (void *)-1;

	addr = pl->cp->address_list[buffer_id];
	if (addr != 0)
		return (void *)addr;
	return (void *)-1;
}
EXPORT_SYMBOL(bif_query_address);

void *bif_alloc_cp(enum BUFF_ID buffer_id, int size, ulong *phyaddr)
{
	int basemaxsize;
	int baseoffset;
	int basephy;
	int len;
	void *addr;
	struct bifbase_local *pl = get_bifbase_local();

	if (!pl || !pl->start || !pl->plat || !pl->cp || buffer_id >= BUFF_MAX
		|| !pl->bifbase_viraddr)
		return (void *)-1;

	if (pl->plat->plat_type == PLAT_AP)
		return (void *)-1;

	basemaxsize = pl->bifbase_phyaddrsize;
	baseoffset = pl->bifbase_phyaddroffset;
	basephy = pl->bifbase_phyaddr;
	addr = pl->bifbase_viraddr;
	*phyaddr = 0;

	pr_info("bifbase: max=0x%x,offset=0x%x,phy=0x%x,vir=0x%lx,size=0x%x\n",
	    basemaxsize, baseoffset, basephy, (unsigned long)addr, size);

	if (baseoffset < 2 * BIFBASE_BLOCK)
		baseoffset = 2 * BIFBASE_BLOCK;

	if (size % BIFBASE_BLOCK)
		len = (size / BIFBASE_BLOCK + 1)*BIFBASE_BLOCK;
	else
		len = size;

	if (baseoffset + len > basemaxsize)
		return (void *)-1;

	*phyaddr = basephy + baseoffset;
	addr = addr + baseoffset;
	pl->bifbase_phyaddroffset = baseoffset + len;

	return addr;
}
EXPORT_SYMBOL(bif_alloc_cp);

void *bif_alloc_base(enum BUFF_ID buffer_id, int size)
{
	int offset;
	void *addr;
	struct bifbase_local *pl = get_bifbase_local();

	if (!pl || !pl->start || !pl->plat || !pl->self ||
		buffer_id >= BUFF_MAX)
		return (void *)-1;

	if ((pl->self->next_offset + size) > BIFBASE_BLOCK)
		return (void *)-1;

	if (pl->self->offset_list[buffer_id] == 0) {
		offset = pl->self->next_offset;
		pl->self->offset_list[buffer_id] = offset;
		pl->self->next_offset += size;
	} else
		offset = pl->self->offset_list[buffer_id];

	addr = (void *)pl->self + offset;
	if (!pl->plat->irq_pin_absent)
		bif_send_irq(pl->bifbase_id);

	return addr;
}
EXPORT_SYMBOL(bif_alloc_base);

void *bif_query_otherbase_wait(enum BUFF_ID buffer_id)
{
	unsigned int offset = 0;
	void *addr;
	struct bifbase_local *pl = get_bifbase_local();

	if (!pl || !pl->start || !pl->plat || !pl->other ||
		buffer_id >= BUFF_MAX)
		return (void *)-1;

	while (pl->other->offset_list[buffer_id] == 0) {
		pl->base_irq_wq_flg = 1;
		wait_event_interruptible_timeout(pl->base_irq_wq,
			pl->other->offset_list[buffer_id] != 0,
			msecs_to_jiffies(2000));
		if (signal_pending(current))
			return (void *)-ERESTARTSYS;
	}
	pl->base_irq_wq_flg = 0;

	offset = pl->other->offset_list[buffer_id];
	addr = (void *)pl->other + offset;

	if (offset != 0)
		return addr;

	return (void *)-1;
}
EXPORT_SYMBOL(bif_query_otherbase_wait);

void *bif_query_otherbase(enum BUFF_ID buffer_id)
{
	int offset = 0;
	struct bifbase_local *pl = get_bifbase_local();

	if (!pl || !pl->start || !pl->plat || !pl->other ||
		buffer_id >= BUFF_MAX)
		return (void *)-1;

	offset = pl->other->offset_list[buffer_id];
	if (offset != 0)
		return (void *)pl->other + offset;

	return (void *)-1;
}
EXPORT_SYMBOL(bif_query_otherbase);

int bif_sync_base(void)
{
	struct bifbase_local *pl = get_bifbase_local();

	return bifbase_sync_cp((void *)pl);
}
EXPORT_SYMBOL(bif_sync_base);

void *bif_dma_alloc(size_t size, dma_addr_t *dma_addr,
	gfp_t gfp, unsigned long attrs)
{
	struct bifbase_local *pl = get_bifbase_local();

	if (!pl || !pl->start || !pl->plat || !pl->pdev)
		return (void *)-1;

	if (pl->plat->plat_type == PLAT_AP)
		return (void *)-1;

	return dma_alloc_attrs(&pl->pdev->dev, size, dma_addr, gfp, attrs);
}
EXPORT_SYMBOL(bif_dma_alloc);

void bif_dma_free(size_t size, dma_addr_t *dma_addr,
	gfp_t gfp, unsigned long attrs)
{
	struct bifbase_local *pl = get_bifbase_local();

	if (!pl || !pl->start || !pl->plat || !pl->pdev)
		return;

	if (pl->plat->plat_type == PLAT_AP)
		return;

	dma_free_attrs(&pl->pdev->dev, size, dma_addr, gfp, attrs);
}
EXPORT_SYMBOL(bif_dma_free);

void *bif_get_plat_info(void)
{
	return (void *)&bifplat;
}
EXPORT_SYMBOL(bif_get_plat_info);

char *bif_get_str_bus(enum BUFF_ID buffer_id)
{
	char *p = NULL;

	switch (buffer_id) {
	case BUFF_BASE:
		p = bifbase;
		break;
	case BUFF_ETH:
		p = bifeth;
		break;
	case BUFF_SMD:
		p = bifsmd;
		break;
	case BUFF_SIO:
		p = bifsio;
		break;
	case BUFF_LITE:
		p = biflite;
		break;
	case BUFF_MAX:
		break;
	}

	return p;
}
EXPORT_SYMBOL(bif_get_str_bus);

late_initcall(bifbase_init);
//module_init(bif_base_init);
module_exit(bifbase_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Horizon Inc.");
MODULE_DESCRIPTION("bifbase module");
