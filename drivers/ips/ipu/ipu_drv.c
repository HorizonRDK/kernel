#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/device.h>
#include <linux/compiler.h>
#include <linux/slab.h>
#include <asm-generic/uaccess.h>
#include <x2/x2_ips.h>
#include "ipu_slot.h"
#include "ipu_dev.h"
#include "ipu_drv.h"
#include "ipu_message.h"

#define X2_IPU_NAME         "x2-ipu"

struct x2_ipu_data {
	unsigned char __iomem *regbase;	/* read/write[bwl] */
	unsigned char __iomem *paddr;
	unsigned char __iomem *vaddr;
	struct task_struct *ipu_task;
	struct class *class;
	struct completion comp;
	struct dma_chan *dma_chan;
	dma_addr_t io_phys;
	wait_queue_head_t wq_head;
	spinlock_t slock;
	bool trigger_isr;
	uint32_t isr_data;
	int32_t major;
	ipu_info_t *info;
	void *private;
	struct resource *io_r;
};

static struct x2_ipu_data *g_ipu = NULL;

static int8_t ipu_set_ddr(ipu_info_t * ipu, uint64_t ddrbase)
{
	uint32_t w = 0, h = 0;
	uint32_t size = 0;
	uint32_t i = 0;

	/* step1. calculate crop space */
	if (ipu->ctrl.crop_ddr_en == 1) {
		w = ALIGN_16(ipu->crop.crop_ed.w - ipu->crop.crop_st.w);
		h = ipu->crop.crop_ed.h - ipu->crop.crop_st.h;
		size = w * h;
		ipu->crop_ddr.y_addr = ddrbase;
		ipu->crop_ddr.c_addr = ddrbase + size;
		ipu->crop_ddr.y_size = size;
		ipu->crop_ddr.c_size = size >> 1;
		ddrbase += size * 3 >> 1;
		ddrbase = ALIGN_16(ddrbase);
		set_ipu_addr(0, ipu->crop_ddr.y_addr, ipu->crop_ddr.c_addr);
	}

	/* step2. calculate scale space */
	if (ipu->ctrl.scale_ddr_en == 1) {
		w = ALIGN_16(ipu->scale.scale_tgt.w);
		h = ipu->scale.scale_tgt.h;
		size = w * h;
		ipu->scale_ddr.y_addr = ddrbase;
		ipu->scale_ddr.c_addr = ddrbase + size;
		ipu->scale_ddr.y_size = size;
		ipu->scale_ddr.c_size = size >> 1;
		ddrbase += size * 3 >> 1;
		ddrbase = ALIGN_16(ddrbase);
		set_ipu_addr(1, ipu->scale_ddr.y_addr, ipu->scale_ddr.c_addr);
	}

	/* step3. calculate pymid ds space */
	if (ipu->pymid.pymid_en == 1) {
		for (i = 0; i < ipu->pymid.ds_layer_en; i++) {
			if (ipu->pymid.ds_factor[i] == 0) {
				/* if factor == 0, bypass layer */
				ipu->ds_ddr[i].y_addr = 0;
				ipu->ds_ddr[i].y_size = 0;
				ipu->ds_ddr[i].c_addr = 0;
				ipu->ds_ddr[i].c_size = 0;
				continue;
			}

			w = ALIGN_16(ipu->pymid.ds_roi[i].w);
			h = ipu->pymid.ds_roi[i].h;
			size = w * h;
			ipu->ds_ddr[i].y_addr = ddrbase;
			ipu->ds_ddr[i].y_size = size;
			if (ipu->pymid.ds_uv_bypass & (1 << i)) {
				/* uv bypass layer won't write to ddr */
				ipu->ds_ddr[i].c_addr = 0;
				ipu->ds_ddr[i].c_size = 0;
				ddrbase += size;
			} else {
				ipu->ds_ddr[i].c_addr = ddrbase + size;
				ipu->ds_ddr[i].c_size = size >> 1;
				ddrbase += size * 3 >> 1;
			}
			ddrbase = ALIGN_16(ddrbase);
			set_ds_layer_addr(i, ipu->ds_ddr[i].y_addr,
					  ipu->ds_ddr[i].c_addr);
		}

		/* step3.1. calculate pymid us space */
		for (i = 0; i < 6; i++) {
			if (!(ipu->pymid.us_layer_en & 1 << i)) {
				/* layer disable */
				ipu->us_ddr[i].y_addr = 0;
				ipu->us_ddr[i].y_size = 0;
				ipu->us_ddr[i].c_addr = 0;
				ipu->us_ddr[i].c_size = 0;
				continue;
			}
			w = ALIGN_16(ipu->pymid.us_roi[i].w);
			h = ipu->pymid.us_roi[i].h;
			size = w * h;
			ipu->us_ddr[i].y_addr = ddrbase;
			ipu->us_ddr[i].y_size = size;
			if (ipu->pymid.us_uv_bypass & 1 << i) {
				ipu->us_ddr[i].c_addr = ddrbase + size;
				ipu->us_ddr[i].c_size = size >> 1;
				ddrbase += size * 3 >> 1;
			} else {
				/* uv bypass layer won't write to ddr */
				ipu->us_ddr[i].c_addr = 0;
				ipu->us_ddr[i].c_size = 0;
				ddrbase += size;
			}
			ddrbase = ALIGN_16(ddrbase);
			set_us_layer_addr(i, ipu->us_ddr[i].y_addr,
					  ipu->us_ddr[i].c_addr);
		}
	}

	return 0;
}

int8_t ipu_set(ipu_cmd_e cmd, ipu_info_t * ipu_info, uint64_t data)
{
	switch (cmd) {
	case IPUC_SET_DDR:
		ipu_set_ddr(ipu_info, data);
		break;
	case IPUC_SET_BASE:
		set_ipu_ctrl(&ipu_info->ctrl);
		set_ipu_video_size(&ipu_info->video_in);
		break;
	case IPUC_SET_CROP:
		set_ipu_crop(&ipu_info->crop);
		break;
	case IPUC_SET_SCALE:
		set_ipu_scale(&ipu_info->scale);
		break;
	case IPUC_SET_FRAME_ID:
		set_ipu_frame_id(&ipu_info->frame_id);
		break;
	case IPUC_SET_PYMID:
		set_ipu_pymid(&ipu_info->pymid);
		break;
	default:
		break;
	}

	return 0;
}

static int ipu_thread(void *data)
{
	uint32_t status = 0;
	unsigned long flags = 0;
	struct x2_ipu_data *ipu = (struct x2_ipu_data *)data;
	ipu_info_t *ipu_info = (ipu_info_t *) ipu->private;
	ipu_slot_h_t *slot_h = NULL;
	ipu_msg_t msg;

	do {
		wait_event_interruptible(ipu->wq_head,
					 ipu->trigger_isr == true);
		status = ipu->isr_data;

		spin_lock_irqsave(&ipu->slock, flags);
		if (status & IPU_BUS01_TRANSMIT_ERRORS) {
			ipu_err("ipu bus01 error\n");
		}
		if (status & IPU_BUS23_TRANSMIT_ERRORS) {
			ipu_err("ipu bus23 error\n");
		}
		if (status & PYM_DS_FRAME_DROP) {
			ipu_err("pymid ds drop\n");
		}
		if (status & PYM_US_FRAME_DROP) {
			ipu_err("pymid us drop\n");
		}

		if (status & IPU_FRAME_START) {
			slot_h = ipu_get_free_slot();
			if (!slot_h) {
				/* TODO free list empty */
				continue;
			}
			slot_to_busy_list(slot_h);
			ipu_set(IPUC_SET_DDR, ipu_info,
				IPU_GET_SLOT(slot_h->slot_id,
					     (uint64_t) ipu->paddr));
		}

		if (status & IPU_FRAME_DONE) {
		}

		if (status & PYM_FRAME_START) {
		}

		if (status & PYM_FRAME_DONE) {
			slot_h = ipu_get_busy_slot();
			if (!slot_h) {
				/* busy list empty */
				continue;
			}
			slot_to_done_list(slot_h);
			/* TODO need flash cache? */
			/* send message pymid done */
			msg.addr = (msg_ddr_info *) & slot_h->ddr_info;
			msg.slot_phys = virt_to_phys((void *)slot_h->membase);
			msg.slot_id = slot_h->slot_id;
			send_to_usr((char *)&msg, sizeof(msg));
		}
		ipu->trigger_isr = false;
		ipu->isr_data = 0;
		spin_unlock_irqrestore(&ipu->slock, flags);
	} while (!kthread_should_stop());

	return 0;
}

void x2_ipu_isr(unsigned int status, void *data)
{
	unsigned long flags = 0;
	struct x2_ipu_data *ipu = (struct x2_ipu_data *)data;

	ipu->isr_data = status;
	spin_lock_irqsave(&ipu->slock, flags);
	ipu->trigger_isr = true;
	spin_unlock_irqrestore(&ipu->slock, flags);
	wake_up(&ipu->wq_head);
}

static int8_t ipu_stop_thread(struct x2_ipu_data *ipu)
{
	if (!IS_ERR(ipu->ipu_task))
		kthread_stop(ipu->ipu_task);
	ipu->ipu_task = NULL;
	return 0;
}

/* Match table for of_platform binding */
static const struct of_device_id x2_ipu_of_match[] = {
	{.compatible = "hobot,x2-ipu",},
	{}
};

MODULE_DEVICE_TABLE(of, x2_ipu_of_match);

static int8_t ipu_core_init(ipu_info_t * ipu_info)
{
	slot_ddr_info_t s_info;
	uint64_t pbase = (uint64_t) g_ipu->paddr;
	uint8_t i = 0;

	memset(&s_info, 0, sizeof(slot_ddr_info_t));

	ipu_set(IPUC_SET_BASE, ipu_info, 0);
	ipu_set(IPUC_SET_CROP, ipu_info, 0);
	ipu_set(IPUC_SET_SCALE, ipu_info, 0);
	ipu_set(IPUC_SET_PYMID, ipu_info, 0);
	ipu_set(IPUC_SET_FRAME_ID, ipu_info, 0);
	ipu_set(IPUC_SET_DDR, ipu_info, pbase);

	s_info.crop.offset = ipu_info->crop_ddr.y_addr - pbase;
	s_info.crop.size =
	    ipu_info->crop_ddr.y_size + ipu_info->crop_ddr.c_size;
	s_info.scale.offset = ipu_info->scale_ddr.y_addr - pbase;
	s_info.scale.size =
	    ipu_info->scale_ddr.y_size + ipu_info->scale_ddr.c_size;
	for (i = 0; i < 24; i++) {
		s_info.ds[i].offset = ipu_info->ds_ddr[i].y_addr - pbase;
		s_info.ds[i].size =
		    ipu_info->ds_ddr[i].y_size + ipu_info->ds_ddr[i].c_size;
	}
	for (i = 0; i < 6; i++) {
		s_info.us[i].offset = ipu_info->us_ddr[i].y_addr - pbase;
		s_info.us[i].size =
		    ipu_info->us_ddr[i].y_size + ipu_info->us_ddr[i].c_size;
	}
	init_ipu_slot((uint64_t) g_ipu->vaddr, &s_info);

	g_ipu->ipu_task = kthread_run(ipu_thread, (void *)g_ipu, "ipu_daemon");
	if (IS_ERR(g_ipu->ipu_task)) {
		ipu_err("thread create fail\n");
		return -1;
	}

	ips_register_irqhandle(IPU_INT, x2_ipu_isr, (void *)g_ipu);

	return 0;
}

int ipu_open(struct inode *node, struct file *filp)
{
	return 0;
}

static void dma_comp_func(void *completion)
{
	complete(completion);
}

int dma_transfer(void *buf, int len)
{
	struct dma_device *dma_dev;
	struct dma_async_tx_descriptor *tx = NULL;
	dma_addr_t dma_src_addr, dma_dst_addr, phys_addr;
	dma_cap_mask_t mask;
	ipu_slot_h_t *slot_h;
	dma_cookie_t cookie;
	enum dma_status status;
	int rc = 0;

	/* request dma */
	dma_cap_zero(mask);
	dma_cap_set(DMA_MEMCPY, mask);
	g_ipu->dma_chan = dma_request_channel(mask, NULL, NULL);
	if (!g_ipu->dma_chan) {
		ipu_err("fail to request dma\n");
		return -EFAULT;
	} else {
		ipu_err("Using %s for DMA transfer\n",
			dma_chan_name(ipu->dma_chan));
	}

	dma_dev = g_ipu->dma_chan->device;
	phys_addr = dma_map_single(dma_dev->dev, buf, len, DMA_BIDIRECTIONAL);
	if (dma_mapping_error(dma_dev->dev, phys_addr)) {
		ipu_err("fail to dma_map_single\n");
		rc = -EFAULT;
		goto err_out;
	}

	dma_dst_addr = phys_addr;
	slot_h = ipu_get_done_slot();
	if (!slot_h) {
		rc = -ENODATA;
		goto err_out1;
	}
	dma_src_addr = virt_to_phys((void *)slot_h->membase);

	tx = dma_dev->device_prep_dma_memcpy(g_ipu->dma_chan, dma_dst_addr,
					     dma_src_addr, len,
					     DMA_CTRL_ACK | DMA_PREP_INTERRUPT);
	if (!tx) {
		ipu_err("fail to prepare dma memory\n");
	}
	init_completion(&g_ipu->comp);
	tx->callback = dma_comp_func;
	tx->callback_param = &g_ipu->comp;

	cookie = tx->tx_submit(tx);
	if (dma_submit_error(cookie)) {
		ipu_err("fail to do dma tx\n");
		rc = -EFAULT;
		goto err_out1;
	}
	dma_async_issue_pending(g_ipu->dma_chan);
	wait_for_completion(&g_ipu->comp);

	status = dma_async_is_tx_complete(g_ipu->dma_chan, cookie, NULL, NULL);
	if (status != DMA_COMPLETE) {
		ipu_err("dma tx complete check fail\n");
		rc = -EFAULT;
		goto err_out1;
	}

err_out1:
	dma_unmap_single(dma_dev->dev, phys_addr, len, DMA_BIDIRECTIONAL);
err_out:
	dma_release_channel(g_ipu->dma_chan);
	return rc;
}

long ipu_ioctl(struct file *filp, unsigned int cmd, unsigned long data)
{
	ipu_info_t *ipu_info = (ipu_info_t *) g_ipu->private;
	ipu_slot_h_t *slot_h = NULL;
	int ret = 0;

	switch (cmd) {
	case IPUC_INIT:
		/* TODO need confirm init file format */
		ret =
		    copy_from_user(ipu_info, (const void __user *)data,
				   sizeof(data));
		if (ret) {
			ipu_err("ioctl init fail\n");
			return -EFAULT;
		}
		ret = ipu_core_init(ipu_info);
		if (ret < 0) {
			ipu_err("ioctl init fail\n");
			return -EFAULT;
		}
		ret = 0;
		break;
	case IPUC_GET_IMG:
		/* TODO get description */
		/* could be used read api, caused user can define length */
		dma_transfer((void *)data, IPU_SLOT_SIZE);
		ret = IPU_SLOT_SIZE;
		break;
	case IPUC_CNN_DONE:
		/* step 1 mv slot from done to free */
		slot_h = ipu_get_done_slot();
		if (!slot_h) {
			return -EFAULT;
		}
		slot_to_free_list(slot_h);
		ret = 0;
		break;
	default:
		break;
	}

	return ret;
}

int ipu_mmap(struct file *filp, struct vm_area_struct *vma)
{
	/*
	   int8_t ret = 0;
	   ret = remap_pfn_range(vma, vma->vm_start, ,
	   vma->vm_end - vma->vm_start, vma->vm_page_prot);
	   if (ret)
	   return -EAGAIN;
	 */
	return 0;
}

static const struct file_operations ipu_fops = {
	.owner = THIS_MODULE,
	.mmap = ipu_mmap,
	.open = ipu_open,
	.unlocked_ioctl = ipu_ioctl,
};

static int x2_ipu_probe(struct platform_device *pdev)
{
	int rc = 0;
	const struct of_device_id *match;
	struct resource *res;
	struct resource r;
	struct x2_ipu_data *ipu = NULL;
	struct device_node *np = NULL;
	ipu_info_t *ipu_info = NULL;

	/* rc = proc_create("ipu", 0, NULL, &ipu_fops); */

	ipu = devm_kzalloc(&pdev->dev, sizeof(*ipu), GFP_KERNEL);
	if (!ipu) {
		return -ENOMEM;
	}
	ipu->trigger_isr = false;
	ipu->isr_data = 0;
	spin_lock_init(&ipu->slock);
	init_waitqueue_head(&ipu->wq_head);

	ipu_info = kzalloc(sizeof(ipu_info_t), GFP_KERNEL);
	if (!ipu_info) {
		rc = -ENOMEM;
		goto err_out;
	}

	ipu->info = ipu_info;

	/* get driver info from dts */
	match = of_match_node(x2_ipu_of_match, pdev->dev.of_node);
	if (match && match->data) {
		/* Nothing to do, maybe can be used in future */
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		rc = -ENODEV;
		goto err_out;
	}
	if (!request_mem_region(res->start, resource_size(res), X2_IPU_NAME)) {
		rc = -ENOMEM;
		goto err_out;
	}
	ipu->regbase = ioremap(res->start, resource_size(res));
	if (!ipu->regbase) {
		dev_err(&pdev->dev, "unable to map registers\n");
		rc = -ENOMEM;
		goto err_out1;
	}
	set_ipu_regbase(ipu->regbase);
	ipu->io_r = res;

	/* request memory address */
	np = of_parse_phandle(pdev->dev.of_node, "memory_region", 0);
	if (!np) {
		dev_err(&pdev->dev, "No %s specified\n", "memory-region");
		rc = -ENODEV;
		goto err_out2;
	}

	rc = of_address_to_resource(np, 0, &r);
	if (rc) {
		dev_err(&pdev->dev,
			"No memory address assigned to the region\n");
		goto err_out2;
	}
	ipu->paddr = (unsigned char __iomem *)r.start;
	ipu->vaddr =
	    (unsigned char __iomem *)memremap(r.start, resource_size(&r),
					      MEMREMAP_WB);
	dev_info(&pdev->dev,
		 "Allocate reserved memory, vaddr: 0x%0llx, paddr: 0x%0llx\n",
		 (uint64_t) ipu->paddr, (uint64_t) ipu->vaddr);

	platform_set_drvdata(pdev, ipu);

	/* create device node */
	ipu->class = class_create(THIS_MODULE, X2_IPU_NAME);
	ipu->major = register_chrdev(0, X2_IPU_NAME, &ipu_fops);
	if (ipu->major < 0) {
		dev_err(&pdev->dev, "No major\n");
		rc = -ENODEV;
		goto err_out3;
	}
	device_create(ipu->class, NULL, MKDEV(ipu->major, 0), NULL,
		      X2_IPU_NAME);

	g_ipu = ipu;

	return 0;

err_out3:
	class_destroy(ipu->class);
err_out2:
	clr_ipu_regbase();
	iounmap(ipu->regbase);
	ipu->io_r = NULL;
err_out1:
	release_mem_region(res->start, resource_size(res));
err_out:
	kfree(ipu);
	return rc;
}

static int x2_ipu_remove(struct platform_device *pdev)
{
	struct x2_ipu_data *ipu = platform_get_drvdata(pdev);

	ipu_stop_thread(ipu);
	release_mem_region(ipu->io_r->start, resource_size(ipu->io_r));
	clr_ipu_regbase();
	iounmap(ipu->regbase);
	ipu->regbase = NULL;
	ipu->io_r = NULL;

	if (ipu->info)
		kfree(ipu->info);

	//device_destory(ipu->class, MKDEV(ipu->major, 0));
	unregister_chrdev(ipu->major, X2_IPU_NAME);
	class_destroy(ipu->class);

	if (ipu)
		devm_kfree(&pdev->dev, ipu);

	g_ipu = NULL;

	return 0;
}

static struct platform_driver x2_ipu_platform_driver = {
	.probe = x2_ipu_probe,
	.remove = x2_ipu_remove,
	.driver = {
		   .name = X2_IPU_NAME,
		   .of_match_table = x2_ipu_of_match,
		   },
};

static int __init x2_ipu_init(void)
{
	int ret = 0;

	/* Register the platform driver */
	ret = platform_driver_register(&x2_ipu_platform_driver);
	return ret;
}

static void __exit x2_ipu_exit(void)
{
	/* Unregister the platform driver */
	platform_driver_unregister(&x2_ipu_platform_driver);
}

module_init(x2_ipu_init);
module_exit(x2_ipu_exit);

MODULE_DESCRIPTION("Driver for X2 IPU Dev");
MODULE_AUTHOR("Horizon Inc.");
MODULE_LICENSE("GPL");
