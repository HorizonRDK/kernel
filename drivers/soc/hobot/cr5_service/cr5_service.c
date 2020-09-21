// SPDX-License-Identifier: GPL-2.0
/*
 *
 * Copyright (c) 2020-21
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/cdev.h>

#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#include <linux/mm.h>

#include <linux/smp.h>
#include <linux/irqchip/arm-gic.h>

#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/delay.h>

#include <soc/hobot/cr5_service.h>

#define DRIVER_NAME "cr5_service"

#define CR5_SERVICE_DEV_MAX_MINORS 1

#define CR5_LOAD_START_IMAGE _IO('A', 0x1)
#define CR5_SERVICE_REQ _IO('A', 0x2)
#define IPI_TEST_CMD _IO('A', 0x3)

#define IMAGE_NAME_LEN 128

#define CR5_IMAGE_REGION_SIZE  0xFF000
#define CR5_MSG_REGION_SIZE    0x1000

typedef struct cr5_image_info {
	char image_name[IMAGE_NAME_LEN];
}cr5_image_info_t;

#define SERVICE_START_FLAG 0xA5A5A5A5
#define SERVICE_STOP_FLAG 0x5A5A5A5A

enum CR5_SERVICE_CMD_TYPE {
	OSD_ASM_SERVICE,
	OSD_C_SERVICE,
	YUV422_TO_420_SERVICE
};

enum CR5_IMAGE_FORMAT {
	IMAGE_YUV420_NV12,
	IMAGE_YUYV422
};

/*
 * cr5 image occupied reserved memory in device tree
 * addr = 0x03E00000, size = 0x100000
 */

typedef struct cr5_msg_info {
	unsigned int service_flags;
	unsigned int service_cmd;
	unsigned int service_info_paddr;
	unsigned int service_info_size;
}cr5_msg_info_t;

typedef struct osd_service_info {
	unsigned int src_phy_addr;
	unsigned int src_img_height;
	unsigned int src_img_width;
	unsigned int tmp_phy_addr;
	unsigned int tmp_size;
}osd_service_info_t;

typedef struct yuv422_to_420_service {
	unsigned int src_phy_addr;
	unsigned int src_img_height;
	unsigned int src_img_width;
	unsigned int dst_phy_addr;
	unsigned int dst_size;
}yuv422to420_service_info_t;

typedef struct cr5_mem_info {
	void __iomem *image_vaddr;
	unsigned long image_phy_addr;
	unsigned long image_region_size;
	void __iomem *msg_vaddr;
	unsigned long msg_phy_addr;
	unsigned long msg_region_size;
} cr5_mem_info_t;

typedef struct ipi_info {
	int ipi_apu_resp_irq;
	void __iomem *ipi_apu_base;

	spinlock_t irq_lock;
	int irq_triggered;
	wait_queue_head_t wait_resp_queue;
}ipi_info_t;

typedef struct sys_ctrl_info {
	void __iomem *reg_base;
}sys_ctrl_t;

typedef struct cr5_dev_info {
	sys_ctrl_t sys_ctrl;
	ipi_info_t ipi;
	cr5_mem_info_t mem;

	int major;
	struct device *dev;
	struct class *class;
	struct cdev cdev;

	struct mutex io_mutex;
	struct mutex service_mutex;
}cr5_dev_info_t;
cr5_dev_info_t *global_cr5_device;

unsigned int find_firstzero_bit(unsigned int* bitmask)
{
	unsigned int i = 0;
	unsigned int bitcheck = *bitmask;
	if (bitcheck == IPI_CMDBUF_FULLUSE) {
		*bitmask = 0;
		return i;
	}

	while (bitcheck) {
		bitcheck &= ~(1 << i);
		i++;
	}
	return i;
}

void mbox_ca53_transmit(cr5_dev_info_t *cr5_dev, unsigned int msg)
{
	static unsigned int bitmask = 0;
	unsigned int bufidx;
	void __iomem *ipi_apu_base = cr5_dev->ipi.ipi_apu_base;

	/* wait for CR5 response */
	bufidx = find_firstzero_bit(&bitmask);
	/* write msg */
	writel(msg, ipi_apu_base + IPI_APU_REQ_BUFF(bufidx));
	/* trigger CR5 interrupt */
	writel(MBOX_IRQ_RPU, ipi_apu_base + IPI_APU_TRIG_REG);
	bitmask |= 1 << bufidx;
}

unsigned int mbox_ca53_get_resp(cr5_dev_info_t *cr5_dev, unsigned int irq_type)
{
	/* read from head of RPU response buffer  */
	unsigned int respdata;
	unsigned int bufidx;
	static unsigned int bitmask = 0;
	void __iomem *ipi_apu_base = cr5_dev->ipi.ipi_apu_base;

	/* mask CR5(RX) interrupt */
	writel(GET_IPI_IRQ_MSK(irq_type), ipi_apu_base + IPI_APU_IER_REG);
	bufidx = find_firstzero_bit(&bitmask);
	respdata = readl(ipi_apu_base + IPI_RPU_RESP_BUFF(bufidx));
	bitmask |= 1 << bufidx;

	return respdata;
}

void ack_mbox_irq(cr5_dev_info_t *cr5_dev, unsigned int irq_type)
{
	void __iomem *ipi_apu_base = cr5_dev->ipi.ipi_apu_base;

	writel(GET_IPI_IRQ_MSK(irq_type), ipi_apu_base + IPI_APU_ISR_REG);
	readl(ipi_apu_base + IPI_APU_ISR_REG); /* flush posted write */

	/* unmask CR5 interrupt */
	writel(GET_IPI_IRQ_MSK(irq_type), ipi_apu_base + IPI_APU_IDR_REG);
}

int is_mbox_irq(cr5_dev_info_t *cr5_dev, unsigned int irq_type)
{
	void __iomem *ipi_apu_base = cr5_dev->ipi.ipi_apu_base;
	unsigned int state = 0xff, mask = 0xff, enable = 0xff;

	if((irq_type != MBOX_RX) && (irq_type != MBOX_TX))
		return 0;

	if(irq_type == MBOX_IRQ_APU) {
		state = readl(ipi_apu_base + IPI_APU_ISR_REG);
		mask = readl(ipi_apu_base + IPI_APU_IMR_REG);
		enable = ~mask & 0x3;
	} else {
		enable = 0;
	}

	return (int)(state & enable & GET_IPI_IRQ_MSK(irq_type));
}

static irqreturn_t ipi_apu_resp_irq_handle(int irq, void *dev_id)
{
	cr5_dev_info_t *cr5_dev = (cr5_dev_info_t *)dev_id;

	if(is_mbox_irq(cr5_dev, MBOX_TX)) {
		/* TODO : check CR5 response */
		mbox_ca53_get_resp(cr5_dev, MBOX_TX);
		//printk("get cr5 respond\n");
		ack_mbox_irq(cr5_dev, MBOX_TX);

		spin_lock(&cr5_dev->ipi.irq_lock);
		cr5_dev->ipi.irq_triggered = 1;
		spin_unlock(&cr5_dev->ipi.irq_lock);
		wake_up_interruptible(&cr5_dev->ipi.wait_resp_queue);
	}

	return IRQ_HANDLED;
}

static int request_ipi_irq(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	cr5_dev_info_t *cr5_dev = dev_get_drvdata(dev);

	ret = platform_get_irq(pdev, 0);
	if (ret < 0) {
		dev_err(&pdev->dev, "unable to find IPI IRQ\n");
		return ret;
	}
	cr5_dev->ipi.ipi_apu_resp_irq = ret;

	ret = devm_request_irq(&pdev->dev, cr5_dev->ipi.ipi_apu_resp_irq,
					ipi_apu_resp_irq_handle,
					IRQF_NO_THREAD, dev_name(&pdev->dev), cr5_dev);
	if (ret) {
		dev_err(&pdev->dev, "IRQ %d already allocated\n",
					cr5_dev->ipi.ipi_apu_resp_irq);
		return ret;
	}

    return 0;
}

static int cr5_load_start_image(cr5_dev_info_t *cr5_dev, cr5_image_info_t *info)
{
	int ret = 0;
	struct file *fp;
	void *image_data;
	loff_t image_size;
	struct device *dev = cr5_dev->dev;
	unsigned char *name = info->image_name;
	int val;

	/*open cr5 file*/
	fp = filp_open(name, O_RDONLY, 0);
	if (IS_ERR(fp)) {
		dev_err(dev, "open file error\n");
		return -ENODEV;
	}

	ret = kernel_read_file(fp, &image_data, &image_size,
					INT_MAX, READING_KEXEC_IMAGE);
	if (ret) {
		dev_err(dev, "kernel read file, 0x%x\n", ret);
		goto err1;
	}

	/*load image to cr5 reserve memory*/
	memcpy(cr5_dev->mem.image_vaddr, image_data, image_size);
	msleep(50);

	/*start cr5
	 *0.enable cr5 clock
	 *1.keep cr5 reset state
	 *2.set cr5 start addr
	 *3.let cr5 run
	 */
	val = readl(cr5_dev->sys_ctrl.reg_base + 0x104);
	val |= (0x1 << 14);
	writel(val, cr5_dev->sys_ctrl.reg_base + 0x104);
	msleep(10);
	writel(0x2800, cr5_dev->sys_ctrl.reg_base + 0x400);
	msleep(10);
	writel(cr5_dev->mem.image_phy_addr, cr5_dev->sys_ctrl.reg_base + 0x600);
	msleep(10);
	writel(0x800, cr5_dev->sys_ctrl.reg_base + 0x400);
	msleep(10);

	filp_close(fp, NULL);
	return 0;

err1:
	filp_close(fp, NULL);
	return ret;
}

typedef void *(*CR5_GET_INFO_HANDLE_T)(void);
extern void register_cr5_get_msginfo_base(CR5_GET_INFO_HANDLE_T func);
void __iomem *cr5_get_msginfo_base(void)
{
	return global_cr5_device->mem.msg_vaddr;
}

typedef void (*CR5_SEND_IPI_HANDLE_T)(void);
extern void register_cr5_send_ipi_func(CR5_SEND_IPI_HANDLE_T func);
void send_cr5_ipi(void)
{
	mbox_ca53_transmit(global_cr5_device, 0);
}

static long cr5_service_dev_ioctl(struct file *file,
							unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	cr5_image_info_t info;
	cr5_dev_info_t *cr5_dev = file->private_data;
	void *ptr;

	mutex_lock(&cr5_dev->io_mutex);
	switch (cmd) {
		case CR5_LOAD_START_IMAGE:
			if (!copy_from_user(&info, (void __user *)arg, sizeof (cr5_image_info_t))) {
				ret = cr5_load_start_image(cr5_dev, &info);
			} else {
				ret = -ENOTTY;
			}

			register_cr5_get_msginfo_base(cr5_get_msginfo_base);
			register_cr5_send_ipi_func(send_cr5_ipi);
			break;
		case CR5_SERVICE_REQ:
			if (mutex_lock_interruptible(&cr5_dev->service_mutex)) {
				ret = -ERESTARTSYS;
				break;
			}

			ptr = cr5_dev->mem.msg_vaddr;
			if (!copy_from_user(ptr, (void __user *)arg, sizeof (cr5_msg_info_t))) {
				cr5_msg_info_t *p = (cr5_msg_info_t *)ptr;
				p->service_flags = SERVICE_START_FLAG;
				mbox_ca53_transmit(cr5_dev, 0);
			} else {
				ret = -ENOTTY;
			}
			break;
		case IPI_TEST_CMD:
			mbox_ca53_transmit(cr5_dev, 0);
			break;
		default:
			ret = -ENOTTY;
	}
	mutex_unlock(&cr5_dev->io_mutex);

	return ret;
}

static int cr5_service_dev_open(struct inode *inode, struct file *file)
{
	file->private_data = global_cr5_device;

	mutex_init(&global_cr5_device->io_mutex);
	mutex_init(&global_cr5_device->service_mutex);

	return 0;
}

static int cr5_service_dev_close(struct inode *inode, struct file *file)
{
	file->private_data = NULL;
	return 0;
}

static unsigned int cr5_service_dev_poll(struct file *file, poll_table *wait)
{
	unsigned int mask = 0;
	cr5_dev_info_t *cr5_dev = file->private_data;
	unsigned long flags;
	unsigned int valid_wake = 0;

	poll_wait(file, &cr5_dev->ipi.wait_resp_queue, wait);

	spin_lock_irqsave(&cr5_dev->ipi.irq_lock, flags);
	if (cr5_dev->ipi.irq_triggered) {
		mask |= (POLLIN | POLLRDNORM);
		valid_wake = 1;
	}
	cr5_dev->ipi.irq_triggered = 0;
	spin_unlock_irqrestore(&cr5_dev->ipi.irq_lock, flags);

	if (valid_wake)
		mutex_unlock(&cr5_dev->service_mutex);

	return mask;
}

static const struct file_operations cr5_service_dev_fops = {
	.owner = THIS_MODULE,
	.open = cr5_service_dev_open,
	.unlocked_ioctl = cr5_service_dev_ioctl,
	.release = cr5_service_dev_close,
	.poll = cr5_service_dev_poll,
};

static void release_ipi_irq(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    cr5_dev_info_t *cr5_dev = dev_get_drvdata(dev);

    devm_free_irq(&pdev->dev, cr5_dev->ipi.ipi_apu_resp_irq, cr5_dev);
}

static int map_cr5_service_reg_addr(struct platform_device *pdev)
{
	int ret;
	struct device_node *np = NULL;
	unsigned long cr5_image_phy_base;
	unsigned long cr5_image_total_size;
	void __iomem *cr5_image_virt_base;
	unsigned long cr5_msg_phy_base;
	unsigned long cr5_msg_total_size;
	void __iomem *cr5_msg_virt_base;
	struct device *dev = &pdev->dev;
	cr5_dev_info_t *cr5_dev = dev_get_drvdata(dev);
	struct resource *res;
	struct resource mem_reserved;

	/*get image reserve path*/
	np = of_parse_phandle(pdev->dev.of_node, "image-memory-region", 0);
	if (!np) {
		dev_err(&pdev->dev, "No %s specified\n", "image-memory-region");
	}

	ret = of_address_to_resource(np, 0, &mem_reserved);
	if (ret) {
		dev_err(&pdev->dev,
			"No memory address assigned to the region\n");
		return -ENOMEM;
	} else {
		cr5_image_phy_base = (unsigned long)mem_reserved.start;
		cr5_image_total_size = resource_size(&mem_reserved);
		cr5_image_virt_base = (void __iomem *)ioremap_nocache(cr5_image_phy_base,
									cr5_image_total_size);
		dev_info(&pdev->dev,
			"Allocate image reserved memory, paddr: 0x%0llx, vaddr: 0x%0llx, size = 0x%llx\n",
			(uint64_t) cr5_image_phy_base, (uint64_t)cr5_image_virt_base, (uint64_t)cr5_image_total_size);
	}

	/*get msg reserve path*/
	np = of_parse_phandle(pdev->dev.of_node, "msg-memory-region", 0);
	if (!np) {
		dev_err(&pdev->dev, "No %s specified\n", "msg-memory-region");
	}

	memset (&mem_reserved, 0, sizeof (struct resource));
	ret = of_address_to_resource(np, 0, &mem_reserved);
	if (ret) {
		dev_err(&pdev->dev,
			"No memory address assigned to the region\n");
		return -ENOMEM;
	} else {
		cr5_msg_phy_base = (unsigned long)mem_reserved.start;
		cr5_msg_total_size = resource_size(&mem_reserved);
		cr5_msg_virt_base = (void __iomem *)ioremap_nocache(cr5_msg_phy_base,
									cr5_msg_total_size);
		dev_info(&pdev->dev,
			"Allocate reserved memory, paddr: 0x%0llx, vaddr: 0x%0llx, size = 0x%llx\n",
			(uint64_t) cr5_msg_phy_base, (uint64_t)cr5_msg_virt_base, (uint64_t)cr5_msg_total_size);
	}

	cr5_dev->mem.image_phy_addr = cr5_image_phy_base;
	cr5_dev->mem.image_vaddr = cr5_image_virt_base;
	cr5_dev->mem.image_region_size = cr5_image_total_size;
	cr5_dev->mem.msg_phy_addr = cr5_msg_phy_base;
	cr5_dev->mem.msg_vaddr = cr5_msg_virt_base;
	cr5_dev->mem.msg_region_size = cr5_msg_total_size;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ipi_reg");
	if (!res) {
		dev_err(&pdev->dev, "invalid ipi_apu_reg address\n");
		ret = -ENODEV;
		goto err1;
	}

	cr5_dev->ipi.ipi_apu_base = devm_ioremap_nocache(&pdev->dev,
    						res->start, resource_size(res));
	if (cr5_dev->ipi.ipi_apu_base == NULL) {
		dev_err(&pdev->dev, "faile to ioremap ipi_apu_reg address\n");
		ret = -ENOMEM;
		goto err1;
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "sys_ctrl");
	if (!res) {
		dev_err(&pdev->dev, "invalid sys_ctrl_reg address\n");
		ret = -ENODEV;
		goto err2;
	}

	cr5_dev->sys_ctrl.reg_base = devm_ioremap_nocache(&pdev->dev,
    						res->start, resource_size(res));
	if (cr5_dev->sys_ctrl.reg_base == NULL) {
		dev_err(&pdev->dev, "faile to ioremap sys_ctrl_reg address\n");
		ret = -ENOMEM;
		goto err2;
	}

	return 0;

err2:
	devm_iounmap(&pdev->dev, cr5_dev->ipi.ipi_apu_base);
	cr5_dev->ipi.ipi_apu_base = NULL;
err1:
	iounmap(cr5_msg_virt_base);
	iounmap(cr5_image_virt_base);
	cr5_dev->mem.image_phy_addr = 0;
	cr5_dev->mem.image_vaddr = NULL;
	cr5_dev->mem.image_region_size = 0;
	cr5_dev->mem.msg_phy_addr = 0;
	cr5_dev->mem.msg_vaddr = NULL;
	cr5_dev->mem.msg_region_size = 0;
	return ret;
}

static void unmap_cr5_service_reg_addr(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	cr5_dev_info_t *cr5_dev = dev_get_drvdata(dev);

	if (cr5_dev->sys_ctrl.reg_base != NULL) {
		devm_iounmap(&pdev->dev, cr5_dev->sys_ctrl.reg_base);
		cr5_dev->sys_ctrl.reg_base = NULL;
	}

	if (cr5_dev->ipi.ipi_apu_base != NULL) {
		devm_iounmap(&pdev->dev, cr5_dev->ipi.ipi_apu_base);
		cr5_dev->ipi.ipi_apu_base = NULL;
	}

	if (cr5_dev->mem.image_vaddr != NULL) {
		iounmap(cr5_dev->mem.image_vaddr);
	}

	if (cr5_dev->mem.msg_vaddr != NULL) {
		iounmap(cr5_dev->mem.msg_vaddr);
	}

	cr5_dev->mem.image_phy_addr = 0;
	cr5_dev->mem.image_vaddr = NULL;
	cr5_dev->mem.image_region_size = 0;
	cr5_dev->mem.msg_phy_addr = 0;
	cr5_dev->mem.msg_vaddr = NULL;
	cr5_dev->mem.msg_region_size = 0;
}

static int alloc_cr5_service_cdev(struct platform_device *pdev)
{
	int ret;
	dev_t devno;
	struct device *dev = &pdev->dev;
	cr5_dev_info_t *cr5_dev = dev_get_drvdata(dev);

	cr5_dev->class = class_create(THIS_MODULE, "cr5_service_class");
	if (cr5_dev->class == NULL) {
		dev_err(dev, "Failed to register cr5 device class\n");
		return -ENOMEM;
	}

	/* Allocate char device for this r5 service driver */
	if (alloc_chrdev_region(&devno, 0,
		CR5_SERVICE_DEV_MAX_MINORS, "cr5_service_dev") < 0) {
		dev_err(dev, "Failed to register cr5 service char dev\n");
		ret = -ENODEV;
		goto err1;
	}
	cr5_dev->major = MAJOR(devno);

	/* Initialize character device */
	cdev_init(&cr5_dev->cdev, &cr5_service_dev_fops);
	cr5_dev->cdev.owner = THIS_MODULE;
	if (cdev_add(&cr5_dev->cdev, MKDEV(cr5_dev->major, 0), 1)) {
		dev_err(dev, "CR5 chardev registration failed.\n");
		ret = -ENODEV;
		goto err2;
	}
	/* Create device */
	cr5_dev->dev = device_create(cr5_dev->class, dev, devno,
                                NULL, "cr5_service");
	dev_info(dev, "cr5_service registered!\n");
	if (cr5_dev->dev == NULL) {
		dev_err(dev, "Cannot create device file.\n");
		goto err3;
	}

	return 0;

err3:
	cdev_del(&cr5_dev->cdev);
err2:
	unregister_chrdev_region(devno, CR5_SERVICE_DEV_MAX_MINORS);
err1:
	class_destroy(cr5_dev->class);
    return ret;
}

static void release_cr5_service_cdev(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	cr5_dev_info_t *cr5_dev = dev_get_drvdata(dev);

	device_unregister(cr5_dev->dev);
	cdev_del(&cr5_dev->cdev);
	unregister_chrdev_region(MKDEV(cr5_dev->major, 0),
				CR5_SERVICE_DEV_MAX_MINORS);
	class_destroy(cr5_dev->class);
}

static int init_cr5_service_ipi_info(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	cr5_dev_info_t *cr5_dev = dev_get_drvdata(dev);

	cr5_dev->ipi.irq_triggered = 0;
	spin_lock_init(&cr5_dev->ipi.irq_lock);
	init_waitqueue_head(&cr5_dev->ipi.wait_resp_queue);

	ret = request_ipi_irq(pdev);
	if (ret < 0) {
		dev_err(dev, "Cound not request ipi irq\n");
		return ret;
	}

	return ret;
}

static void release_cr5_service_ipi_info(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	cr5_dev_info_t *cr5_dev = dev_get_drvdata(dev);

	cr5_dev->ipi.irq_triggered = 0;
	release_ipi_irq(pdev);
}


static int cr5_service_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;

	global_cr5_device = (cr5_dev_info_t *) kzalloc(sizeof(cr5_dev_info_t),
										GFP_KERNEL);
	if (!global_cr5_device) {
		dev_err(dev, "Cound not allocate cr5 device info\n");
		return -ENOMEM;
	}
	dev_set_drvdata(dev, global_cr5_device);

	ret = map_cr5_service_reg_addr(pdev);
	if (ret < 0) {
		dev_err(dev, "Cound not get cr5 service reg info\n");
		goto err1;
	}

	ret = init_cr5_service_ipi_info(pdev);
	if (ret < 0) {
		dev_err(dev, "Cound not init cr5 service ipi info\n");
		goto err2;
	}
	mutex_init(&global_cr5_device->io_mutex);
	mutex_init(&global_cr5_device->service_mutex);

	ret = alloc_cr5_service_cdev(pdev);
	if (ret < 0) {
		dev_err(dev, "Cound not alloc cr5 service cdev\n");
		goto err3;
	}

	return 0;
err3:
	release_cr5_service_ipi_info(pdev);
err2:
	unmap_cr5_service_reg_addr(pdev);
err1:
	kfree(global_cr5_device);
	global_cr5_device = NULL;
	return ret;
}

static int cr5_service_remove(struct platform_device *pdev)
{
	release_cr5_service_cdev(pdev);
	release_cr5_service_ipi_info(pdev);
	unmap_cr5_service_reg_addr(pdev);
	kfree(global_cr5_device);
	global_cr5_device = NULL;

	return 0;
}

#ifdef CONFIG_OF
static struct of_device_id cr5_service_of_match[] = {
    { .compatible = "hobot,cr5_service", },
    { /* end of list */ },
};
MODULE_DEVICE_TABLE(of, cr5_service_of_match);
#else
#define cr5_service_of_match
#endif

static struct platform_driver cr5_service_driver = {
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
        .of_match_table = cr5_service_of_match,
    },
    .probe      = cr5_service_probe,
    .remove     = cr5_service_remove,
};

static int __init cr5_service_init(void)
{
    return platform_driver_register(&cr5_service_driver);
}

static void __exit cr5_service_exit(void)
{
    platform_driver_unregister(&cr5_service_driver);
}

module_init(cr5_service_init);
module_exit(cr5_service_exit);
MODULE_LICENSE("GPL");

