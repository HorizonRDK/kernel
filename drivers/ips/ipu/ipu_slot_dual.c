#include <linux/spinlock.h>
#include <asm-generic/io.h>
#include "ipu_slot_dual.h"
#include "ipu_common.h"

//dual mode
ipu_slot_dual_h_t g_ipu_slot_dual[IPU_MAX_SLOT_DUAL];
slot_queue_t g_free_slot_queue;
slot_queue_t g_recv_slot_queue;
slot_queue_t g_recvdone_slot_queue;
slot_queue_t g_pym_slot_queue;
slot_queue_t g_pymdone_slot_queue;

//dual mode below
void enqueue_slot(slot_queue_t * slot_queue, ipu_slot_dual_h_t * slot_h)
{
	struct list_head *phead;
	phead = &slot_queue->queue;
	spin_lock(&slot_queue->lock);
	list_add_tail(&slot_h->list, phead);
	slot_queue->qlen++;
	spin_unlock(&slot_queue->lock);
}

ipu_slot_dual_h_t *dequeue_slot(slot_queue_t * slot_queue)
{
	ipu_slot_dual_h_t *slot_h;
	struct list_head *phead, *pnode;
	phead = &slot_queue->queue;
	spin_lock(&slot_queue->lock);
	if (list_empty(phead)) {
		spin_unlock(&slot_queue->lock);
		return NULL;
	}
	pnode = phead->next;
	list_del_init(pnode);
	slot_h = (ipu_slot_dual_h_t *) pnode;
	slot_queue->qlen--;
	spin_unlock(&slot_queue->lock);
	return slot_h;
}

ipu_slot_dual_h_t *get_first_of_queue(slot_queue_t * slot_queue)
{
	ipu_slot_dual_h_t *slot_h;
	struct list_head *phead, *pnode;
	phead = &slot_queue->queue;
	spin_lock(&slot_queue->lock);
	if (list_empty(phead)) {
		spin_unlock(&slot_queue->lock);
		return NULL;
	}
	pnode = phead->next;
	slot_h = (ipu_slot_dual_h_t *) pnode;
	spin_unlock(&slot_queue->lock);
	return slot_h;
}

int8_t init_ipu_slot_dual(uint64_t base, slot_ddr_info_dual_t * data)
{
	int8_t i = 0;

	INIT_LIST_HEAD(&g_free_slot_queue.queue);
	INIT_LIST_HEAD(&g_recv_slot_queue.queue);
	INIT_LIST_HEAD(&g_recvdone_slot_queue.queue);
	INIT_LIST_HEAD(&g_pym_slot_queue.queue);
	INIT_LIST_HEAD(&g_pymdone_slot_queue.queue);
	spin_lock_init(&g_free_slot_queue.lock);
	spin_lock_init(&g_recv_slot_queue.lock);
	spin_lock_init(&g_recvdone_slot_queue.lock);
	spin_lock_init(&g_pym_slot_queue.lock);
	spin_lock_init(&g_pymdone_slot_queue.lock);
	g_free_slot_queue.qlen = 0;
	g_recvdone_slot_queue.qlen = 0;
	g_recv_slot_queue.qlen = 0;
	g_pym_slot_queue.qlen = 0;
	g_pymdone_slot_queue.qlen = 0;

	for (i = 0; i < IPU_MAX_SLOT_DUAL; i++) {
		uint64_t slot_vaddr = IPU_GET_DUAL_SLOT(i, base);
		memset(&g_ipu_slot_dual[i], 0, sizeof(ipu_slot_dual_h_t));
		INIT_LIST_HEAD(&g_ipu_slot_dual[i].list);
		ipu_dbg("slot-%d, vaddr=%llx\n", i, slot_vaddr);

		enqueue_slot(&g_free_slot_queue, &g_ipu_slot_dual[i]);
		g_ipu_slot_dual[i].info_h.slot_id = i;

		memcpy(&g_ipu_slot_dual[i].info_h.ddr_info, data,
		       sizeof(slot_ddr_info_dual_t));
		ipu_dbg("ds[%d].y_offset=0x%x\n", i,
			g_ipu_slot_dual[i].info_h.ddr_info.ds_1st[i].y_offset);
	}
	return 0;
}

int8_t ipu_clean_slot_queue(void)
{
	ipu_slot_dual_h_t *slot_h = NULL;
	while (g_recv_slot_queue.qlen > 0) {
		slot_h = dequeue_slot(&g_recv_slot_queue);
		enqueue_slot(&g_free_slot_queue, slot_h);
	}
	while (g_recvdone_slot_queue.qlen > 0) {
		slot_h = dequeue_slot(&g_recvdone_slot_queue);
		enqueue_slot(&g_free_slot_queue, slot_h);
	}
	while (g_pym_slot_queue.qlen > 0) {
		slot_h = dequeue_slot(&g_pym_slot_queue);
		enqueue_slot(&g_free_slot_queue, slot_h);
	}
	while (g_pymdone_slot_queue.qlen > 0) {
		slot_h = dequeue_slot(&g_pymdone_slot_queue);
		enqueue_slot(&g_free_slot_queue, slot_h);
	}
	return 0;
}
