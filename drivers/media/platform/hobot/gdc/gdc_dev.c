/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <asm/cacheflush.h>
#include <asm/pgtable.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/scatterlist.h>
#include <linux/bug.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <asm/cacheflush.h>
#include <linux/io.h>

#include "gdc_dev.h"
#include "gdc_hw_api.h"

#define MODULE_NAME "X3 GDC"

extern void write_gdc_mask(uint32_t model, uint32_t *enable);
extern void write_gdc_status(uint32_t model, uint32_t *enable);

typedef void(*rst_func)(void);
extern void gdc_call_install(rst_func p);

struct x3_gdc_dev *g_gdc_dev = NULL;

struct gdc_group *gdc_get_group(struct x3_gdc_dev *gdc)
{
	u32 stream;
	struct gdc_group *group;
	for (stream = 0; stream < VIO_MAX_STREAM; ++stream) {
		group = &gdc->group[stream];
		if (!test_bit(GDC_GROUP_OPEN, &group->state)) {
			group->instance = stream;
			return group;
		}
	}
	return NULL;
}

static int x3_gdc_open(struct inode *inode, struct file *file)
{
	struct gdc_video_ctx *gdc_ctx;
	struct x3_gdc_dev *gdc;
	struct gdc_group *group;
	int ret = 0;
	int enbale = 1;

	gdc = container_of(inode->i_cdev, struct x3_gdc_dev, cdev);
	gdc_ctx = kzalloc(sizeof(struct gdc_video_ctx), GFP_KERNEL);
	if (gdc_ctx == NULL) {
		vio_err("kzalloc is fail");
		ret = -ENOMEM;
		goto p_err;
	}

	group = gdc_get_group(gdc);
	if (group == NULL) {
		ret = -EBUSY;
		vio_err("can get free group\n");
		goto p_err;
	}

	group->sub_ctx[0] = gdc_ctx;
	gdc_ctx->group = group;
	set_bit(GDC_GROUP_OPEN, &group->state);

	init_waitqueue_head(&gdc_ctx->done_wq);

	gdc_ctx->gdc_dev = gdc;
	file->private_data = gdc_ctx;

	vio_dwe_clk_enable();
	write_gdc_mask(gdc->hw_id, &enbale);

	vio_info("GDC%d open node\n", gdc->hw_id);
p_err:
	return ret;
}

static ssize_t x3_gdc_write(struct file *file, const char __user * buf,
			     size_t count, loff_t * ppos)
{
	return 0;
}

static ssize_t x3_gdc_read(struct file *file, char __user * buf, size_t size,
			    loff_t * ppos)
{
	return 0;

}

static int x3_gdc_close(struct inode *inode, struct file *file)
{
	struct gdc_video_ctx *gdc_ctx;
	struct gdc_group *group;
	struct x3_gdc_dev *gdc;
	int enbale = 0;

	gdc_ctx = file->private_data;
	group = gdc_ctx->group;

	clear_bit(GDC_GROUP_OPEN, &group->state);

	gdc = gdc_ctx->gdc_dev;

	write_gdc_mask(gdc->hw_id, &enbale);
	vio_dwe_clk_disable();

	kfree(gdc_ctx);

	return 0;
}

void dwe0_reset_control(void)
{
	u32 cnt = 20;
	unsigned long flag;

	spin_lock_irqsave(&g_gdc_dev->shared_slock, flag);
	while (1) {
		if (g_gdc_dev->state != GDC_DEV_PROCESS) {
			ips_set_module_reset(DWE0_RST);
			break;
		}

		mdelay(1);
		cnt--;
		if (cnt == 0) {
			vio_err("%s timeout\n", __func__);
			break;
		}
	}
	spin_unlock_irqrestore(&g_gdc_dev->shared_slock, flag);
}

/**
 *   This function starts the gdc block
 *
 *   Writing 0->1 transition is necessary for trigger
 *
 *   @param  gdc_settings - overall gdc settings and state
 *
 */
void gdc_start(struct x3_gdc_dev *gdc_dev)
{
	gdc_process_enable(gdc_dev->base_reg, 0);
	gdc_process_enable(gdc_dev->base_reg, 1);
	vio_dbg("%s\n", __func__);
}

/**
 *   Configure the output gdc configuration address/size and buffer address/size; and resolution.
 *
 *   More than one gdc settings can be accessed by index to a gdc_config_t.
 *
 *   @return 0 - success
 *           -1
 */

void gdc_init(struct x3_gdc_dev *gdc_dev, gdc_settings_t *gdc_settings)
{
	void __iomem *base_addr;

	base_addr = gdc_dev->base_reg;

	gdc_process_enable(base_addr, 0);
	gdc_set_config_addr(base_addr, gdc_settings->gdc_config.config_addr);
	gdc_set_config_size(base_addr,
			    gdc_settings->gdc_config.config_size / 4);
	gdc_set_rdma_img_width(base_addr, gdc_settings->gdc_config.input_width);
	gdc_set_rdma_img_height(base_addr,
				gdc_settings->gdc_config.input_height);
	gdc_set_wdma_img_width(base_addr,
			       gdc_settings->gdc_config.output_width);
	gdc_set_wdma_img_height(base_addr,
				gdc_settings->gdc_config.output_height);
	vio_dbg("GDC config_addr:%x,config_size:%x\n",
		 gdc_settings->gdc_config.config_addr,
		 gdc_settings->gdc_config.config_size);
	vio_dbg("gdc in w:%d, h:%d\n", gdc_settings->gdc_config.input_width,
		 gdc_settings->gdc_config.input_height);
	vio_dbg("gdc out w:%d, h:%d\n", gdc_settings->gdc_config.output_width,
		 gdc_settings->gdc_config.output_height);
}

/**
 *   This function points gdc to its input resolution and yuv address and offsets
 *
 *   Shown inputs to GDC are Y and UV plane address and offsets
 *
 *
 *   @return 0 - success
 *           -1 - no interrupt from GDC.
 */

int gdc_process(struct x3_gdc_dev *gdc_dev, gdc_settings_t *gdc_settings)
{
	void __iomem *base_addr;

	u32 lineoffset, height;	//, size;
	u32 num_input = 0;
	u32 *input_addr;

	num_input = gdc_settings->total_planes;
	input_addr = gdc_settings->In_buffer_addr;
	base_addr = gdc_dev->base_reg;

	//process input addresses
	if (num_input >= 1) {
		lineoffset = gdc_settings->gdc_config.input_stride;
		height = gdc_settings->gdc_config.input_height;
		if (gdc_settings->gdc_config.sequential_mode == 1){
			gdc_set_rdma0_img_addr(base_addr,
				input_addr[gdc_settings->seq_planes_pos]);
			if (gdc_settings->seq_planes_pos > 0){	//UV planes
				lineoffset = gdc_settings->gdc_config.output_width
				    >> gdc_settings->gdc_config.div_width;
				height = gdc_settings->gdc_config.output_height
				    >> gdc_settings->gdc_config.div_height;
			}
		} else{
			gdc_set_rdma0_img_addr(base_addr, input_addr[0]);
			vio_dbg("GDC input1 addr:%x\n", input_addr[0]);
		}
		gdc_set_rdma0_line_offset(base_addr, lineoffset);
		vio_dbg("GDC data1in lineoffset:%d, height:%d\n", lineoffset,
			 height);
	}
	if (num_input >= 2 && gdc_settings->gdc_config.sequential_mode == 0) {			//only processed if not in toggle mode
		vio_dbg("GDC plain_num:%d case\n", num_input);
		lineoffset = gdc_settings->gdc_config.input_stride
			>> gdc_settings->gdc_config.div_width;
		height = gdc_settings->gdc_config.input_height
			>> gdc_settings->gdc_config.div_height;

		gdc_set_rdma1_img_addr(base_addr, input_addr[1]);
		gdc_set_rdma1_line_offset(base_addr, lineoffset);
		vio_dbg("GDC input2 addr:%x\n", input_addr[1]);
		vio_dbg("GDC data2in lineoffset:%d, height:%d\n", lineoffset,
			 height);
	}
	if (num_input >= 3 && gdc_settings->gdc_config.sequential_mode == 0) {			//only processed if not in toggle mode
		lineoffset = gdc_settings->gdc_config.input_stride
			>> gdc_settings->gdc_config.div_width;
		height = gdc_settings->gdc_config.input_height
			>> gdc_settings->gdc_config.div_height;
		gdc_set_rdma2_img_addr(base_addr, input_addr[2]);
		gdc_set_rdma2_line_offset(base_addr, lineoffset);
	}
	//outputs
	if (num_input >= 1) {
		lineoffset = gdc_settings->gdc_config.output_stride;
		height = gdc_settings->gdc_config.output_height;
		vio_dbg("GDC out1: plain_num:%d case, lineoffset:%d, height:%d\n",
			 num_input, lineoffset, height);
		if (gdc_settings->gdc_config.sequential_mode == 1
		    && gdc_settings->seq_planes_pos > 0){		//UV planes
			lineoffset = gdc_settings->gdc_config.output_stride
				>> gdc_settings->gdc_config.div_width;
			height = gdc_settings->gdc_config.output_height
				>> gdc_settings->gdc_config.div_height;
		}
		gdc_set_wdma0_img_addr(base_addr,
				       gdc_settings->Out_buffer_addr[0]);
		gdc_set_wdma0_line_offset(base_addr, lineoffset);
		vio_dbg("GDC out1: data1out_addr_write:%x\n",
			 gdc_settings->Out_buffer_addr[0]);
	}

	if (num_input >= 2 && gdc_settings->gdc_config.sequential_mode == 0) {
		lineoffset = gdc_settings->gdc_config.output_stride
			>> gdc_settings->gdc_config.div_width;
		height = gdc_settings->gdc_config.output_height
			>> gdc_settings->gdc_config.div_height;
		vio_dbg("GDC out2: plain_num:%d case, lineoffset:%d, height:%d\n",
			 num_input, lineoffset, height);
		gdc_set_wdma1_img_addr(base_addr,
				       gdc_settings->Out_buffer_addr[1]);
		gdc_set_wdma1_line_offset(base_addr, lineoffset);
		vio_dbg("GDC out2: data2out_addr_write:%x\n",
			 gdc_settings->Out_buffer_addr[1]);
	}

	if (num_input >= 3 && gdc_settings->gdc_config.sequential_mode == 0) {
		lineoffset = gdc_settings->gdc_config.output_width
			>> gdc_settings->gdc_config.div_width;
		height = gdc_settings->gdc_config.output_height
			>> gdc_settings->gdc_config.div_height;
		gdc_set_wdma2_img_addr(base_addr,
				       gdc_settings->Out_buffer_addr[1]);
		gdc_set_wdma2_line_offset(base_addr, lineoffset);
		vio_dbg("out2: data2out_addr_write:%x\n",
			 gdc_settings->Out_buffer_addr[1]);
	}

	if (gdc_settings->gdc_config.sequential_mode == 1) {			//update the planes position
		if (++(gdc_settings->seq_planes_pos) >=
		    gdc_settings->gdc_config.total_planes)
			gdc_settings->seq_planes_pos = 0;
	}

	return 0;

}

void gdc_set_iar_output(struct x3_gdc_dev *gdc_dev,
		gdc_settings_t *gdc_settings)
{
#ifdef X3_IAR_INTERFACE
	u8 dis_instance[2] = {0, 0};
	u8 display_layer[2] ={0, 0};
	int ret = 0;

	ret = ipu_get_iar_display_type(dis_instance, display_layer);
	if (!ret) {
		if (gdc_dev->hw_id == (display_layer[0] - 37) ||
				gdc_dev->hw_id == (display_layer[1] - 37))
			ipu_set_display_addr(0, gdc_settings->Out_buffer_addr[0],
				gdc_settings->Out_buffer_addr[1]);
	}
	vio_dbg("GDC display_layer = %d, dis_instance = %d, ret(%d)",
		display_layer[0], dis_instance[0], ret);
#endif
}

int gdc_video_process(struct gdc_video_ctx *gdc_ctx, unsigned long arg)
{
	int ret = 0;
	int timeout = 0;
	unsigned long flag;
	gdc_settings_t gdc_settings;
	struct x3_gdc_dev *gdc_dev;

	ret = copy_from_user(&gdc_settings, (gdc_settings_t *) arg,
			   sizeof(gdc_settings_t));
	if (ret) {
		vio_err("GDC_IOCTL_PROCESS copy from user failed (ret=%d)\n",
			 ret);
		return -EFAULT;
	}

	gdc_dev = gdc_ctx->gdc_dev;

	ret = down_interruptible(&gdc_dev->smp_gdc_enable);
	if (ret) {
		vio_err("%s down fail(%d)", __func__, ret);
		goto p_err_ignore;
	}

	spin_lock_irqsave(&gdc_dev->shared_slock, flag);
	gdc_dev->state = GDC_DEV_PROCESS;
	gdc_ctx->event = 0;
	atomic_set(&gdc_dev->instance, gdc_ctx->group->instance);

	gdc_init(gdc_dev, &gdc_settings);
	ret = gdc_process(gdc_dev, &gdc_settings);
	gdc_start(gdc_dev);
	spin_unlock_irqrestore(&gdc_dev->shared_slock, flag);

	timeout = wait_event_interruptible_timeout(gdc_ctx->done_wq,
				gdc_ctx->event,
				msecs_to_jiffies(GDC_PROCESS_TIMEOUT));
	if (timeout == 0) {
		up(&gdc_dev->smp_gdc_enable);
		vio_err("GDC process timeout;\n");
		return -ETIMEDOUT;
	}
	gdc_dev->state = GDC_DEV_FREE;

	if (gdc_ctx->event == VIO_FRAME_DONE) {
		gdc_set_iar_output(gdc_dev, &gdc_settings);
	} else {
		vio_err("GDC process failed\n");
		ret = -VIO_FRAME_NDONE;
	}

	vio_dbg("%s done: ret(%d), timeout %d\n", __func__, ret, timeout);
p_err_ignore:
	return ret;
}

static long x3_gdc_ioctl(struct file *file, unsigned int cmd,
			  unsigned long arg)
{
	struct gdc_video_ctx *gdc_ctx;
	struct gdc_group *group;
	int ret = 0;

	gdc_ctx = file->private_data;
	group = gdc_ctx->group;

	if (_IOC_TYPE(cmd) != GDC_IOC_MAGIC)
		return -ENOTTY;

	switch (cmd) {
	case GDC_IOC_PROCESS:
		ret = gdc_video_process(gdc_ctx, arg);
		break;
	default:
		vio_err("wrong ioctl cmd for GDC\n");
		break;
	}

	return ret;
}

static irqreturn_t gdc_isr(int irq, void *data)
{
	u32 status;
	u32 dwe_status = 1;
	u32 instance;
	struct x3_gdc_dev *gdc;
	struct gdc_group *gdc_group;
	struct gdc_video_ctx *gdc_ctx;

	gdc = data;
	instance = atomic_read(&gdc->instance);
	gdc_group = &gdc->group[instance];
	gdc_ctx = gdc_group->sub_ctx[0];
	gdc_ctx->event = VIO_FRAME_DONE;

	status = gdc_get_intr_status(gdc->base_reg);
	write_gdc_status(gdc->hw_id, &dwe_status);

	vio_dbg("%s:status = 0x%x, dwe_status = 0x%x\n", __func__, status,
		 dwe_status);

	if (status & 1 << INTR_GDC_BUSY) {
		vio_info("GDC current frame is processing\n");
	} else if (status & 1 << INTR_GDC_ERROR) {
		if (status & 1 << INTR_GDC_CONF_ERROR)
			vio_err("GDC configuration error\n");

		if (status & 1 << INTR_GDC_USER_ABORT)
			vio_err("GDC user abort(stop/reset command)\n");

		if (status & 1 << INTR_GDC_AXI_READER_ERROR)
			vio_err("GDC AXI reader error\n");

		if (status & 1 << INTR_GDC_AXI_WRITER_ERROR)
			vio_err("GDC AXI writer error\n");

		if (status & 1 << INTR_GDC_UNALIGNED_ACCESS)
			vio_err("GDC address pionter is not aligned\n");

		if (status & 1 << INTR_GDC_INCOMPATIBLE_CONF)
			vio_err("GDC incopatible configuration\n");

		gdc_ctx->event = VIO_FRAME_NDONE;
	}

	if (gdc_ctx->event) {
		up(&gdc->smp_gdc_enable);
		wake_up(&gdc_ctx->done_wq);
	}

	return IRQ_HANDLED;
}

static struct file_operations x3_gdc_fops = {
	.owner = THIS_MODULE,
	.open = x3_gdc_open,
	.write = x3_gdc_write,
	.read = x3_gdc_read,
	.release = x3_gdc_close,
	.unlocked_ioctl = x3_gdc_ioctl,
	.compat_ioctl = x3_gdc_ioctl,
};

static int x3_gdc_suspend(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x3_gdc_resume(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x3_gdc_runtime_suspend(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static int x3_gdc_runtime_resume(struct device *dev)
{
	int ret = 0;

	vio_info("%s\n", __func__);

	return ret;
}

static const struct dev_pm_ops x3_gdc_pm_ops = {
	.suspend = x3_gdc_suspend,
	.resume = x3_gdc_resume,
	.runtime_suspend = x3_gdc_runtime_suspend,
	.runtime_resume = x3_gdc_runtime_resume,
};

int x3_gdc_device_node_init(struct x3_gdc_dev *gdc)
{
	int ret = 0;
	struct device *dev = NULL;
	char name[32];

	snprintf(name, 32, "gdc%d", gdc->hw_id);

	ret = alloc_chrdev_region(&gdc->devno, 0, GDC_MAX_DEVICE, name);
	if (ret < 0){
		vio_err("Error %d while alloc chrdev gdc", ret);
		goto err_req_cdev;
	}

	cdev_init(&gdc->cdev, &x3_gdc_fops);
	gdc->cdev.owner = THIS_MODULE;
	ret = cdev_add(&gdc->cdev, gdc->devno, GDC_MAX_DEVICE);
	if (ret){
		vio_err("Error %d while adding x2 gdc cdev", ret);
		goto err;
	}

	if (vps_class)
		gdc->class = vps_class;
	else
		gdc->class = class_create(THIS_MODULE, name);

	dev = device_create(gdc->class, NULL, MKDEV(MAJOR(gdc->devno), 0),
		NULL, name);
	if (IS_ERR(dev)){
		ret = -EINVAL;
		vio_err("gdc device create fail\n");
		goto err;
	}

	return ret;
err:
	class_destroy(gdc->class);
err_req_cdev:
	unregister_chrdev_region(gdc->devno, GDC_MAX_DEVICE);
	return ret;
}

static ssize_t gdc_reg_dump(struct device *dev,struct device_attribute *attr, char* buf)
{
	struct x3_gdc_dev *gdc;

	gdc = dev_get_drvdata(dev);

	gdc_hw_dump(gdc->base_reg);

	return 0;
}

static DEVICE_ATTR(regdump, 0444, gdc_reg_dump, NULL);

static int x3_gdc_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct x3_gdc_dev *gdc;
	struct resource *mem_res;
	struct device_node *dnode;
	struct device *dev;

	BUG_ON(!pdev);

	dev = &pdev->dev;
	dnode = dev->of_node;

	gdc = kzalloc(sizeof(struct x3_gdc_dev), GFP_KERNEL);
	if (!gdc) {
		vio_err("gdc is NULL");
		ret = -ENOMEM;
		goto p_err;
	}

	mem_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem_res) {
		vio_err("Failed to get io memory region(%p)", mem_res);
		ret = -EBUSY;
		goto err_get_resource;
	}

	gdc->regs_start = mem_res->start;
	gdc->regs_end = mem_res->end;
	gdc->base_reg =devm_ioremap_nocache(&pdev->dev,
		mem_res->start, resource_size(mem_res));
	if (!gdc->base_reg) {
		vio_err("Failed to remap io region(%p)", gdc->base_reg);
		ret = -ENOMEM;
		goto err_get_resource;
	}

	/* Get IRQ SPI number */
	gdc->irq = platform_get_irq(pdev, 0);
	if (gdc->irq < 0) {
		vio_err("Failed to get gdc_irq(%d)", gdc->irq);
		ret = -EBUSY;
		goto err_get_irq;
	}

	ret = request_irq(gdc->irq, gdc_isr, IRQF_TRIGGER_HIGH, "gdc", gdc);
	if (ret) {
		vio_err("request_irq(IRQ_GDC %d) is fail(%d)", gdc->irq, ret);
		goto err_get_irq;
	}

#ifdef CONFIG_OF
	ret = of_property_read_u32(dnode, "id", &gdc->hw_id);
	if (ret){
		vio_err("id read is fail(%d)", ret);
	}
#endif

	x3_gdc_device_node_init(gdc);

	ret = device_create_file(dev, &dev_attr_regdump);
	if(ret < 0) {
		vio_err("create regdump failed (%d)\n",ret);
		goto p_err;
	}

	platform_set_drvdata(pdev, gdc);

	sema_init(&gdc->smp_gdc_enable, 1);
	spin_lock_init(&gdc->shared_slock);

	if (gdc->hw_id == 0)
		g_gdc_dev = gdc;

	gdc_call_install(dwe0_reset_control);
	vio_info("[FRT:D] %s(%d)\n", __func__, ret);

	return 0;

err_get_irq:
	iounmap(gdc->base_reg);

err_get_resource:
	kfree(gdc);
p_err:
	vio_err("[FRT:D] %s(%d)\n", __func__, ret);
	return ret;

}

static int x3_gdc_remove(struct platform_device *pdev)
{
	int ret = 0;
	struct x3_gdc_dev *gdc;

	BUG_ON(!pdev);

	gdc = platform_get_drvdata(pdev);

	device_remove_file(&pdev->dev, &dev_attr_regdump);

	free_irq(gdc->irq, gdc);

	device_destroy(gdc->class, gdc->devno);
	if (!vps_class)
		class_destroy(gdc->class);
	cdev_del(&gdc->cdev);
	unregister_chrdev_region(gdc->devno, GDC_MAX_DEVICE);
	kfree(gdc);

	vio_info("%s\n", __func__);

	return ret;
}

#ifdef CONFIG_OF
static const struct of_device_id x3_gdc_match[] = {
	{
	 .compatible = "hobot,gdc",
	 },
	{},
};

MODULE_DEVICE_TABLE(of, x3_gdc_match);

static struct platform_driver x3_gdc_driver = {
	.probe = x3_gdc_probe,
	.remove = x3_gdc_remove,
	.driver = {
		   .name = MODULE_NAME,
		   .owner = THIS_MODULE,
		   .pm = &x3_gdc_pm_ops,
		   .of_match_table = x3_gdc_match,
		   }
};

#else
static struct platform_device_id x3_gdc_driver_ids[] = {
	{
	 .name = MODULE_NAME,
	 .driver_data = 0,
	 },
	{},
};

MODULE_DEVICE_TABLE(platform, x3_gdc_driver_ids);

static struct platform_driver x3_gdc_driver = {
	.probe = x3_gdc_probe,
	.remove = __devexit_p(x3_gdc_remove),
	.id_table = x3_gdc_driver_ids,
	.driver = {
		   .name = MODULE_NAME,
		   .owner = THIS_MODULE,
		   .pm = &x3_gdc_pm_ops,
		   }
};
#endif

static int __init x3_gdc_init(void)
{
	int ret = platform_driver_register(&x3_gdc_driver);
	if (ret)
		vio_err("platform_driver_register failed: %d\n", ret);

	return ret;
}

late_initcall(x3_gdc_init);

static void __exit x3_gdc_exit(void)
{
	platform_driver_unregister(&x3_gdc_driver);
}

module_exit(x3_gdc_exit);

MODULE_AUTHOR("Sun Kaikai<kaikai.sun@horizon.com>");
MODULE_DESCRIPTION("X3 GDC driver");
MODULE_LICENSE("GPL v2");
