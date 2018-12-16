/******************************************************************************
 ****			 COPYRIGHT NOTICE			 ****
 ****		 Copyright	2017 Horizon Robotics, Inc.		 ****
 ****			 All rights reserved.			 ****
 *****************************************************************************/
/**
 * BIF base driver for memory and irq management
 * @author		xiaofeng.ling(xiaofeng.ling@horizon.ai)
 * @date		2018/11/16
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include "bif_base.h"

#define BIF_BASE_ADDR ((void *)0x100000)
#define CONFIG_HOBOT_BIF_AP

#ifdef CONFIG_HOBOT_BIF_AP
struct bif_base_info bif_base_buf = {
	.magic = {'B', 'I', 'F', 'B'}
};

struct bif_base_info bif_ap_buf = {
	.magic = {'B', 'I', 'F', 'A'}
};

wait_queue_head_t bif_ap_wq;
#else

#endif

struct bif_base_info *bif_base, *bif_ap, *bif_self, *bif_other;
wait_queue_head_t bif_irq_wq;

void gpio_trigger_irq(int irq);

int bif_spi_read(void *addr, unsigned int count, char *buf)
{
	return 0;
}

int bif_spi_write(void *addr, unsigned int count, char *buf)
{
	return 0;
}

int bif_sd_read(unsigned int sector, unsigned int count, char *buf)
{

	return 0;
}

int bif_sd_write(unsigned int sector, unsigned int count, char *buf)
{
	return 0;
}

static irqreturn_t bif_base_irq_handler(int irq, void *data)
{

#ifdef CONFIG_HOTBOT_BIF_AP
	wake_up(&bif_ap_wq);
#endif
	return IRQ_HANDLED;
}

static irqreturn_t bif_irq_handler(int irq, void *data)
{
	int birq;
	int irq_full = 0;
	irqreturn_t ret = IRQ_NONE;

#ifdef CONFIG_HOTBOT_BIF_AP
	bif_sync_base();
#endif

	if ((bif_other->send_irq_tail + 1)
	    % IRQ_QUEUE_SIZE == bif_self->read_irq_head)
		irq_full = 1;
	while (bif_self->read_irq_head != bif_other->send_irq_tail) {
		birq = bif_self->irq[bif_self->read_irq_head];
		if (birq < BUFF_MAX && bif_self->irq_func[irq])
			ret = bif_self->irq_func[birq] (birq, NULL);
		else
			pr_err("irq %d not register\n", birq);
		bif_self->irq[bif_self->read_irq_head] = 0;
		bif_self->read_irq_head++;
		bif_self->read_irq_head %= BUFF_MAX;
	}

#ifdef CONFIG_HOTBOT_BIF_AP
	bif_sync_ap();
#endif
	if (irq_full)
		bif_send_irq(BUFF_BASE);
	wake_up(&bif_irq_wq);
	return ret;
}

void bif_init(void)
{
	int ret;

#ifdef CONFIG_HOTBOT_BIF_AP
	bif_base = bif_base_buf;
	bif_ap = bif_ap_buf;
	init_waitqueue_head(&bif_ap_wq);
	bif_sync_base();
	bif_self = bif_ap;
	bif_other = bif_base;
#else /*CP side */
	/*resever fix address space for bif base
	 *AP side know this hardcode
	 */
	bif_base = BIF_BASE_ADDR;
	bif_ap = BIF_BASE_ADDR + 512;
	bif_base->magic[0] = 'B';
	bif_base->magic[1] = 'I';
	bif_base->magic[2] = 'F';
	bif_base->magic[3] = 'B';
	bif_self = bif_base;
	bif_other = bif_ap;
#endif
	bif_register_irq(BUFF_BASE, bif_base_irq_handler);
	bif_self->next_offset = sizeof(struct bif_base_info);
	init_waitqueue_head(&bif_irq_wq);
#ifdef CONFIG_HOTBOT_BIF_AP
	ret = devm_request_irq(NULL, 1, bif_irq_handler, IRQF_SHARED, "bif_base", NULL);
#else
	ret = devm_request_irq(NULL, 1, bif_irq_handler, IRQF_SHARED, "bif_base", NULL);
#endif
	if (!ret)
		panic("request_irq for bif GPIO fail");
}

#ifdef CONFIG_HOBOT_BIF_AP
void bif_sync_ap(void)
{
#ifdef CONFIG_HOBOT_BIFSD
	if (!bif_sd_write(BIF_BASE_ADDR / 512 + 1, 1, &bif_ap_buf)) {
		pr_crit("Write by BIFSD for bif_ap fail");
		return;
	}
#else
	if (!bif_spi_write(BIF_BASE_ADDR,
			   sizeof(struct bif_base_info),
			   (char *)&bif_base_buf)) {
		pr_crit("Write by BIFSPI for bif_ap fail");
		return;
	}
#endif
}

void bif_sync_base(void)
{
#ifdef CONFIG_HOBOT_BIFSD
	if (!bif_sd_read(BIF_BASE_ADDR / 512, 1, &bif_base_buf)) {
		pr_crit("Read by BIFSD for bif_base fail");
		return;
	}
#else
#ifdef CONFIG_HOBOT_BIFSPI
	if (!bif_spi_read(BIF_BASE_ADDR, sizeof(bif_base_buf),
			  (char *)&bif_base_buf)) {
		pr_crit("Read by BIFSPI for bif_base fail");
		return;
	}
#endif
#endif
}
#endif

int bif_send_irq(int irq)
{
#ifdef CONFIG_HOTBOT_BIF_AP
	bif_self = bif_base;
	bif_other = bif_ap;
#else
	bif_self = bif_ap;
	bif_other = bif_base;
#endif
	while ((bif_self->send_irq_tail + 1) % IRQ_QUEUE_SIZE ==
		bif_other->read_irq_head) {
		wait_event(bif_irq_wq,
			   (bif_self->send_irq_tail + 1)
			   % IRQ_QUEUE_SIZE != bif_other->read_irq_head);
	}

	bif_self->irq[bif_self->send_irq_tail] = irq;
	bif_self->send_irq_tail++;
	bif_self->send_irq_tail %= IRQ_QUEUE_SIZE;
#ifdef CONFIG_HOTBOT_BIF_AP
	bif_sync_ap();
#endif
	gpio_trigger_irq(1);
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
	bif_self->irq_func[buffer_id] = irq_handler;
	return buffer_id;
}
EXPORT_SYMBOL(bif_register_irq);

void *bif_query_address_wait(enum BUFF_ID buffer_id)
{
	void *ret = NULL;

#ifdef CONFIG_HOTBOT_BIF_AP
	while (bif_base->address_list[buffer_id] == 0) {
		wait_event_interruptible(bif_ap_wq,
			bif_base->address_list[buffer_id] != 0);
		if (signal_pending(current)) {
			/*need consider duplicate with address? */
			ret = (void *)-ERESTARTSYS
			goto out;
		}
	}
	ret = bif_base->address_list[buffer_id];
out:
#endif
	return ret;
}
EXPORT_SYMBOL(bif_query_address_wait);

void *bif_query_address(enum BUFF_ID buffer_id)
{
	void *addr = bif_base->address_list[buffer_id];

	if (addr != 0)
		return addr;
	return (void *)-1;
}
EXPORT_SYMBOL(bif_query_address);


void *bif_alloc_baseoffset(enum BUFF_ID buffer_id, int size)
{
	int offset;

	if ((bif_self->next_offset + size) > 512)
		return (void *)-1;
	offset = bif_self->next_offset;
	bif_self->offset_list[buffer_id] = offset;
	bif_self->next_offset += size;
	return bif_self + offset;
}
EXPORT_SYMBOL(bif_alloc_baseoffset);

void *bif_query_otherbase_wait(enum BUFF_ID buffer_id)
{
	void *ret = NULL;
	int off;

	while (bif_other->offset_list[buffer_id] == 0) {
		wait_event_interruptible(bif_irq_wq,
			bif_other->offset_list[buffer_id] != 0);
		if (signal_pending(current)) {
			/*need consider duplicate with address?*/
			ret = (void *)-ERESTARTSYS;
			goto out;
		}
	}
	off = bif_other->offset_list[buffer_id];
#ifdef CONFIG_HOBOT_BIF_AP
	ret = bif_base + off;
#else
	ret = bif_ap + off;
#endif
out:
	return ret;
}
EXPORT_SYMBOL(bif_query_otherbase_wait);

void *bif_query_otherbase(enum BUFF_ID buffer_id)
{
	int offset = bif_other->offset_list[buffer_id];

	if (offset != 0)
		return bif_other + offset;
	return (void *)-1;
}
EXPORT_SYMBOL(bif_query_otherbase);
