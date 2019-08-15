#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/init.h>
#include <linux/wait.h>
#include <linux/kfifo.h>
#include <linux/kthread.h>
#include "ipu_slot_dual.h"
#include "ipu_dev.h"
#include "ipu_dual.h"
#include "ipu_drv.h"
#include "ipu_common.h"
#include "ipu_pym.h"

#define IPU_PYM_BUF_LEN 6

#define IPU_GET_SLOT(id, base)		((base) + (id) * IPU_SLOT_SIZE)

extern struct x2_ipu_data *g_ipu;
extern int g_pym_from;
struct ipu_pym *g_ipu_pym;

int ipu_pym_to_process(void *img_info,
		ipu_cfg_t *ipu_cfg, ipu_pym_slot_type type,
		ipu_pym_process_type process_type)
{
	struct pym_slot_info tmp_pym_slot;
	struct src_img_info_t *single_img_info;
	struct mult_img_info_t *mult_img_info;
	int all_fifo_len;
	unsigned long flags;
	int i, ret;

	if (!g_ipu_pym) {
		pr_err("IPU pym not init!\n");
		return -ENODEV;
	}

	if (!img_info) {
		pr_err("IPU pym invalid slot!\n");
		return -EINVAL;
	}

	spin_lock_irqsave(&g_ipu_pym->slock, flags);

	if ((g_ipu->ipu_mode != IPU_DDR_SIGNLE)
			&& (g_ipu->ipu_mode != IPU_DDR_DUAL)
			&& (g_ipu->ipu_mode != IPU_FEED_BACK)) {

		pr_err("IPU unmatched working mode\n");
		spin_unlock_irqrestore(&g_ipu_pym->slock, flags);
		return -EINVAL;
	}

	all_fifo_len = kfifo_len(&g_ipu_pym->pym_slots)
			+ kfifo_len(&g_ipu_pym->done_inline_pym_slots)
			+ kfifo_len(&g_ipu_pym->done_offline_pym_slots);

	if (all_fifo_len >= IPU_PYM_BUF_LEN) {
		spin_unlock_irqrestore(&g_ipu_pym->slock, flags);
		pr_err("IPU pym busy\n");
		return -EBUSY;
	}

	if (type == PYM_SLOT_SINGLE) {
		single_img_info = (struct src_img_info_t *)img_info;
		if ((all_fifo_len > 0)
				&& (g_ipu_pym->new_slot_id == single_img_info->slot_id)) {
			spin_unlock_irqrestore(&g_ipu_pym->slock, flags);
			pr_err("IPU pym to process the processing slot\n");
			return -EBUSY;
		}
		g_ipu_pym->new_slot_id = single_img_info->slot_id;

		memcpy(&tmp_pym_slot.img_info.src_img_info,
				single_img_info, sizeof(struct src_img_info_t));
		tmp_pym_slot.slot_type = type;
		tmp_pym_slot.process_type = process_type;
		tmp_pym_slot.pym_left_num = 0;
		tmp_pym_slot.cfg = ipu_cfg;

		ret = kfifo_in(&g_ipu_pym->pym_slots, &tmp_pym_slot, 1);
		if (ret < 1) {
			spin_unlock_irqrestore(&g_ipu_pym->slock, flags);
			pr_err("IPU slot to pym error!\n");
			return ret;
		}
	} else if (type == PYM_SLOT_MULT) {
		mult_img_info = (struct mult_img_info_t *)img_info;
		if ((all_fifo_len > 0)
				&& (g_ipu_pym->new_slot_id
					== mult_img_info->src_img_info[0].slot_id)) {
			spin_unlock_irqrestore(&g_ipu_pym->slock, flags);
			pr_err("IPU pym to process the processing slot\n");
			return -EBUSY;
		}
		g_ipu_pym->new_slot_id = mult_img_info->src_img_info[0].slot_id;

		for (i = 0; i < mult_img_info->src_num; i++) {
			memcpy(&tmp_pym_slot.img_info.mult_img_info,
					mult_img_info, sizeof(struct mult_img_info_t));
			tmp_pym_slot.slot_type = type;
			tmp_pym_slot.process_type = process_type;
			tmp_pym_slot.pym_left_num = mult_img_info->src_num - i - 1;
			tmp_pym_slot.cfg = ipu_cfg;

			ret = kfifo_in(&g_ipu_pym->pym_slots, &tmp_pym_slot, 1);
			if (ret < 1) {
				spin_unlock_irqrestore(&g_ipu_pym->slock, flags);
				pr_err("IPU slot to pym error!\n");
				return ret;
			}
		}
	} else {
		pr_err("IPU pym invalid slot type\n");
		spin_unlock_irqrestore(&g_ipu_pym->slock, flags);
		return -EINVAL;
	}

	spin_unlock_irqrestore(&g_ipu_pym->slock, flags);

	wake_up_interruptible(&g_ipu_pym->process_wait);

	return 0;
}

static int ipu_pym_do_process(struct pym_slot_info *pym_slot_info)
{
	unsigned long flags;
	struct src_img_info_t *process_img_info;

	if (!g_ipu_pym) {
		pr_err("IPU pym not init!\n");
		return -ENODEV;
	}

	spin_lock_irqsave(&g_ipu_pym->slock, flags);

	if (pym_slot_info->slot_type == PYM_SLOT_SINGLE) {
		process_img_info = &pym_slot_info->img_info.src_img_info;
		if (g_pym_from == PYM_SRC_FROM_CROP) {
			set_ds_src_addr(process_img_info->src_img.y_paddr,
					process_img_info->src_img.c_paddr);
		} else if (g_pym_from == PYM_SRC_FROM_SCALE) {
			set_ds_src_addr(process_img_info->scaler_img.y_paddr,
					process_img_info->scaler_img.c_paddr);
		} 
		ipu_set(IPUC_SET_PYM_DDR, pym_slot_info->cfg,
				IPU_GET_SLOT(process_img_info->slot_id,
				g_ipu->paddr));
	} else if (pym_slot_info->slot_type == PYM_SLOT_MULT) {
		/* here should use num to get mult offset */
		if (pym_slot_info->pym_left_num || (pym_slot_info->img_info.mult_img_info.src_num == 1)) {
			process_img_info = &pym_slot_info->img_info.mult_img_info.src_img_info[0];

			set_ds_src_addr(process_img_info->src_img.y_paddr,
					process_img_info->src_img.c_paddr);
			ipu_set(IPUC_SET_PYM_DDR, pym_slot_info->cfg,
					IPU_GET_FST_PYM_OF_SLOT(process_img_info->slot_id,
					g_ipu->paddr));
		} else {
			process_img_info = &pym_slot_info->img_info.mult_img_info.src_img_info[1];
			set_ds_src_addr(process_img_info->src_img.y_paddr,
					process_img_info->src_img.c_paddr);
			ipu_set(IPUC_SET_PYM_DDR, pym_slot_info->cfg,
					IPU_GET_SEC_PYM_OF_SLOT(process_img_info->slot_id,
					g_ipu->paddr));
		}
	} else {
		pr_err("IPU invalid Process slot type\n");
		return -EINVAL;
	}

	pym_manual_start();

	g_ipu_pym->processing = 1;
	spin_unlock_irqrestore(&g_ipu_pym->slock, flags);

	return 0;
}

/* irq to trigger ipu pym frame process done */
int ipu_pym_process_done(void)
{
	unsigned long flags;
	int ret;
	int process_type;

	if (!g_ipu_pym) {
		pr_err("IPU pym not init!\n");
		return -ENODEV;
	}

	spin_lock_irqsave(&g_ipu_pym->slock, flags);
	if (!g_ipu_pym->pyming_slot_info) {
		spin_unlock_irqrestore(&g_ipu_pym->slock, flags);
		pr_err("IPU pym not inited\n");
		return -ENODEV;
	}

	if (g_ipu_pym->pyming_slot_info->slot_type >= PYM_SLOT_TYPE_END) {
		spin_unlock_irqrestore(&g_ipu_pym->slock, flags);
		pr_err("IPU pym invalid slot type\n");
		return -EINVAL;
	}

	if (g_ipu_pym->pyming_slot_info->slot_type == PYM_SLOT_MULT) {
		if (g_ipu_pym->pyming_slot_info->pym_left_num) {
			/* not the end of the mult slot, not need report */
			g_ipu_pym->processing = 0;
			spin_unlock_irqrestore(&g_ipu_pym->slock, flags);
			wake_up_interruptible(&g_ipu_pym->process_wait);
			return 0;
		}
	}

	process_type = g_ipu_pym->pyming_slot_info->process_type;
	if (process_type == PYM_INLINE)
		ret = kfifo_in(&g_ipu_pym->done_inline_pym_slots,
				g_ipu_pym->pyming_slot_info, 1);
	else 
		ret = kfifo_in(&g_ipu_pym->done_offline_pym_slots,
				g_ipu_pym->pyming_slot_info, 1);
	if (ret < 1) {
		g_ipu_pym->processing = 0;
		spin_unlock_irqrestore(&g_ipu_pym->slock, flags);
		pr_err("IPU slot to pym error!\n");
		return ret;
	}
	g_ipu_pym->processing = 0;
	spin_unlock_irqrestore(&g_ipu_pym->slock, flags);

	wake_up_interruptible(&g_ipu_pym->process_wait);
	wake_up_interruptible(
			&g_ipu_pym->done_wait[process_type]);

	return 1;
}

int ipu_pym_wait_process_done(void *data, int len, ipu_pym_process_type process_type, int timeout)
{
	struct pym_slot_info tmp_pym_slot;
	unsigned long flags;
	int ret;

	if (!g_ipu_pym) {
		pr_err("IPU pym not init!\n");
		return -ENODEV;
	}

	if (!data) {
		pr_err("IPU pym not init!\n");
		return -EINVAL;
	}
	if (len > sizeof(struct mult_img_info_t))
		len = sizeof(struct mult_img_info_t);

	if (timeout == 0) {
		spin_lock_irqsave(&g_ipu_pym->slock, flags);
		if (process_type == PYM_INLINE)
			ret = kfifo_len(&g_ipu_pym->done_inline_pym_slots);
		else
			ret = kfifo_len(&g_ipu_pym->done_offline_pym_slots);
		if (ret) {
			goto slot_pop;
		} else {
			spin_unlock_irqrestore(&g_ipu_pym->slock, flags);
			return -EAGAIN;
		}
	} else if (timeout < 0) {
		if (process_type == PYM_INLINE)
			ret = wait_event_interruptible(
					g_ipu_pym->done_wait[process_type],
					kfifo_len(&g_ipu_pym->done_inline_pym_slots));
		else
			ret = wait_event_interruptible(
					g_ipu_pym->done_wait[process_type],
					kfifo_len(&g_ipu_pym->done_offline_pym_slots));

	} else if (timeout > 0) {
		if (process_type == PYM_INLINE)
			ret = wait_event_interruptible_timeout(
					g_ipu_pym->done_wait[process_type],
					kfifo_len(&g_ipu_pym->done_inline_pym_slots),
					msecs_to_jiffies(timeout));
		else
			ret = wait_event_interruptible_timeout(
					g_ipu_pym->done_wait[process_type],
					kfifo_len(&g_ipu_pym->done_offline_pym_slots),
					msecs_to_jiffies(timeout));
	}

	if (ret < 0) {
		pr_err("IPU wait pym error %d, %d\n", __LINE__, ret);
		return ret;
	} else if (ret == 0) {
		pr_debug("IPU wait pym done timeout %d\n", __LINE__);
		return -EFAULT;
	}

	spin_lock_irqsave(&g_ipu_pym->slock, flags);
slot_pop:
	if (process_type == PYM_INLINE)
		ret = kfifo_out(&g_ipu_pym->done_inline_pym_slots, &tmp_pym_slot, 1);
	else
		ret = kfifo_out(&g_ipu_pym->done_offline_pym_slots, &tmp_pym_slot, 1);
	if (ret != 1) {
		pr_err("Get ipu slot from fifo error!\n");
		spin_unlock_irqrestore(&g_ipu_pym->slock, flags);
		return ERR_PTR(-EFAULT);
	}
	spin_unlock_irqrestore(&g_ipu_pym->slock, flags);

	memcpy(data, &tmp_pym_slot.img_info, len);

	return 0;
}

static int ipu_pym_process_thread(void *data)
{
	struct ipu_pym *ipu_pym = (struct ipu_pym *)data;
	struct pym_slot_info tmp_pym_slot;
	unsigned long flags;
	int ret;

	if (!ipu_pym) {
		pr_err("IPU pym not init!\n");
		return -ENODEV;
	}

	while (ipu_pym->inited) {
		ret = wait_event_interruptible(ipu_pym->process_wait,
				kfifo_len(&ipu_pym->pym_slots) && (!ipu_pym->processing));
		if (ret < 0)
			continue;

		spin_lock_irqsave(&ipu_pym->slock, flags);
		ret = kfifo_out(&ipu_pym->pym_slots, &tmp_pym_slot, 1);
		if (ret != 1) {
			spin_unlock_irqrestore(&ipu_pym->slock, flags);
			pr_err("Get ipu slot from fifo error!\n");
			continue;
		}
		spin_unlock_irqrestore(&ipu_pym->slock, flags);
		/* parse the slot info to real info to process */
		memcpy(ipu_pym->pyming_slot_info,
				&tmp_pym_slot, sizeof(struct pym_slot_info));

		if (ipu_pym_do_process(&tmp_pym_slot)) {
			pr_debug("IPU pym process error!\n");
			continue;
		}
	}

	return 0;
}

void ipu_pym_clear(void)
{
	unsigned long flags;

	spin_lock_irqsave(&g_ipu_pym->slock, flags);
	g_ipu_pym->processing = 0;
	kfifo_reset(&g_ipu_pym->pym_slots);
	kfifo_reset(&g_ipu_pym->done_inline_pym_slots);
	kfifo_reset(&g_ipu_pym->done_offline_pym_slots);
	spin_unlock_irqrestore(&g_ipu_pym->slock, flags);
}

int ipu_pym_init(void)
{
	struct ipu_pym *pym;
	struct pym_slot_info tmp_pym_slot;
	int i, ret;
	if (g_ipu_pym)
		return 0;

	pym = kmalloc(sizeof(struct ipu_pym), GFP_KERNEL);
	if (!pym) {
		pr_err("IPU pym init failed!!\n");
		return -ENOMEM;
	}

	pym->pyming_slot_info = kzalloc(sizeof(struct pym_slot_info), GFP_KERNEL);
	if (!pym->pyming_slot_info) {
		kfree(pym);
		pr_err("IPU pym processing buf init failed!!\n");
		return -ENOMEM;
	}

	pym->pym_slots.type= &tmp_pym_slot;
	ret = kfifo_alloc(&pym->pym_slots, IPU_PYM_BUF_LEN, GFP_KERNEL);
	if (ret) {
		kfree(pym->pyming_slot_info);
		kfree(pym);
		pr_err("Can't alloc pym fifo buffer\n");
		return ret;
	}

	ret = kfifo_alloc(&pym->done_inline_pym_slots, IPU_PYM_BUF_LEN, GFP_KERNEL);
	if (ret) {
		kfifo_free(&pym->pym_slots);
		kfree(pym->pyming_slot_info);
		kfree(pym);
		pr_err("Can't alloc pym done fifo buffer\n");
		return ret;
	}

	ret = kfifo_alloc(&pym->done_offline_pym_slots, IPU_PYM_BUF_LEN, GFP_KERNEL);
	if (ret) {
		kfifo_free(&pym->done_inline_pym_slots);
		kfifo_free(&pym->pym_slots);
		kfree(pym->pyming_slot_info);
		kfree(pym);
		pr_err("Can't alloc pym done fifo buffer\n");
		return ret;
	}

	init_waitqueue_head(&pym->process_wait);

	for (i = 0; i < PYM_PROCESS_TYPE_END; i++)
		init_waitqueue_head(&pym->done_wait[i]);

	spin_lock_init(&pym->slock);
	pym->inited = 1;

	pym->process_task = kthread_run(ipu_pym_process_thread, pym,
				 "%s", "ipu_pym");
	if (IS_ERR(pym->process_task)) {
		pym->inited = 0;
		kfifo_free(&pym->done_offline_pym_slots);
		kfifo_free(&pym->done_inline_pym_slots);
		kfifo_free(&pym->pym_slots);
		kfree(pym->pyming_slot_info);
		kfree(pym);
		pr_err("%s: Creat IPU pym process thread failed\n", __func__);
		return PTR_ERR_OR_ZERO(pym->process_task);
	}
	pym->processing = 0;
	pym->new_slot_id = -1;

	g_ipu_pym = pym;

	printk("!!!!ipu pym inited \n");
	return 0;
}

void ipu_pym_exit(void)
{
	if (!g_ipu_pym)
		return;

	g_ipu_pym->inited = 0;
	wake_up_interruptible(&g_ipu_pym->process_wait);
	kthread_stop(g_ipu_pym->process_task);

	kfifo_free(&g_ipu_pym->done_offline_pym_slots);
	kfifo_free(&g_ipu_pym->done_inline_pym_slots);
	kfifo_free(&g_ipu_pym->pym_slots);

	kfree(g_ipu_pym->pyming_slot_info);
	kfree(g_ipu_pym);

	g_ipu_pym = NULL;
}
