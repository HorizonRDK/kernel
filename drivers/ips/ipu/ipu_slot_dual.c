#include <linux/spinlock.h>
#include <asm-generic/io.h>
#include "ipu_drv.h"
#include "ipu_slot_dual.h"
#include "ipu_common.h"


ipu_slot_dual_h_t	g_ipu_slot_dual[IPU_MAX_SLOT_DUAL];
slot_queue_t g_dual_slot_queue[SLOT_QUEUE_MAX];
static DECLARE_BITMAP(slot_init_mask, IPU_MAX_SLOT_DUAL);
extern struct x2_ipu_data *g_ipu;

int slot_alive(int type)
{
	int q_cnt = 0;

	q_cnt = g_dual_slot_queue[type].qlen;
	ipu_dbg("q_cnt = %d\n", q_cnt);
	return q_cnt;
}
void enqueue_slot(slot_queue_t *slot_queue, ipu_slot_dual_h_t *slot_h)
{
	struct list_head *phead;
	phead = &slot_queue->queue;
	spin_lock(&slot_queue->lock);
	list_add_tail(&slot_h->list, phead);
	slot_queue->qlen++;
	spin_unlock(&slot_queue->lock);
}

ipu_slot_dual_h_t* dequeue_slot(slot_queue_t *slot_queue)
{
	ipu_slot_dual_h_t *slot_h;
	struct list_head *phead,*pnode;
	phead = &slot_queue->queue;
	spin_lock(&slot_queue->lock);
	if (list_empty(phead)) {
		spin_unlock(&slot_queue->lock);
		return NULL;
	}
	pnode = phead->next;
	list_del_init(pnode);
	slot_h = (ipu_slot_dual_h_t *)pnode;
	slot_queue->qlen--;
	spin_unlock(&slot_queue->lock);
	return slot_h;
}

int insert_dual_slot_to_free(int slot_id, slot_ddr_info_dual_t *data)
{
	ipu_slot_dual_h_t *slot_h = NULL;
	if (slot_id < 0 || slot_id >= g_ipu->slot_num / 2) {
		ipu_err("invalid slot id when free to done\n");
		return -1;
	}

	ipu_info("insert slot-%d \n", slot_id);
	slot_h = &g_ipu_slot_dual[slot_id];
	slot_h->info_h.cnn_flag = 0;
	slot_h->info_h.slot_flag = SLOT_FREE;
	slot_h->slot_get = 0;
	enqueue_slot(&g_dual_slot_queue[FREE_SLOT_QUEUE], slot_h);
	return 0;
}

ipu_slot_dual_h_t* get_cur_pym_slot(void)
{
	ipu_slot_dual_h_t *slot_h;
	struct list_head *phead,*pnode;
	slot_queue_t *slot_queue = &g_dual_slot_queue[PYMING_SLOT_QUEUE];
	phead = &slot_queue->queue;
	spin_lock(&slot_queue->lock);
	if (list_empty(phead)) {
		spin_unlock(&slot_queue->lock);
		return NULL;
	}
	pnode = phead->next;
	slot_h = (ipu_slot_dual_h_t *)pnode;
	spin_unlock(&slot_queue->lock);
	return slot_h;
}

ipu_slot_dual_h_t* get_last_pym_slot(void)
{
	ipu_slot_dual_h_t *slot_h;
	struct list_head *phead,*pnode;
	slot_queue_t *slot_queue = &g_dual_slot_queue[PYMING_SLOT_QUEUE];
	phead = &slot_queue->queue;
	spin_lock(&slot_queue->lock);
	if (list_empty(phead)) {
		spin_unlock(&slot_queue->lock);
		return NULL;
	}
	pnode = phead->prev;
	slot_h = (ipu_slot_dual_h_t *)pnode;
	spin_unlock(&slot_queue->lock);
	return slot_h;
}

ipu_slot_dual_h_t* ipu_get_pym_free_slot(void)
{
	return dequeue_slot(&g_dual_slot_queue[FREE_SLOT_QUEUE]);
}

ipu_slot_dual_h_t* ipu_get_recv_done_slot(void)
{
	return dequeue_slot(&g_dual_slot_queue[RECVDONE_SLOT_QUEUE]);
}

ipu_slot_dual_h_t* ipu_get_pym_done_slot(void)
{
	return dequeue_slot(&g_dual_slot_queue[PYMDONE_SLOT_QUEUE]);
}

bool ipu_is_pym_done_empty(void)
{
	if (g_dual_slot_queue[PYMDONE_SLOT_QUEUE].qlen == 0)
		return true;
	return false;
}

bool ipu_is_pym_busy_empty(void)
{
	if (g_dual_slot_queue[PYMING_SLOT_QUEUE].qlen == 0)
		return true;
	return false;
}

ipu_slot_dual_h_t* recv_slot_free_to_busy(void)
{
	ipu_slot_dual_h_t *slot_h = NULL;
	slot_h = dequeue_slot(&g_dual_slot_queue[FREE_SLOT_QUEUE]);
	if (slot_h)
		enqueue_slot(&g_dual_slot_queue[RECVING_SLOT_QUEUE], slot_h);
	else
		ipu_info("free slot empty\n");
	return slot_h;
}

ipu_slot_dual_h_t* recv_slot_busy_to_done(void)
{
	ipu_slot_dual_h_t *slot_h = NULL;
	slot_h = dequeue_slot(&g_dual_slot_queue[RECVING_SLOT_QUEUE]);
	if (slot_h)
		enqueue_slot(&g_dual_slot_queue[RECVDONE_SLOT_QUEUE], slot_h);
	return slot_h;
}

ipu_slot_dual_h_t* pym_slot_free_to_busy(void)
{
	ipu_slot_dual_h_t *slot_h = NULL;
	slot_h = dequeue_slot(&g_dual_slot_queue[RECVDONE_SLOT_QUEUE]);
	if (slot_h)
		enqueue_slot(&g_dual_slot_queue[PYMING_SLOT_QUEUE], slot_h);
	else
		ipu_info("free pym slot empty\n");
	return slot_h;
}

ipu_slot_dual_h_t* pym_slot_busy_to_done(void)
{
	ipu_slot_dual_h_t *slot_h = NULL;
	slot_h = dequeue_slot(&g_dual_slot_queue[PYMING_SLOT_QUEUE]);
	if (slot_h)
		enqueue_slot(&g_dual_slot_queue[PYMDONE_SLOT_QUEUE], slot_h);
	return slot_h;
}

ipu_slot_dual_h_t* pym_slot_busy_to_free(void)
{
	ipu_slot_dual_h_t *slot_h = NULL;
	slot_h = dequeue_slot(&g_dual_slot_queue[PYMING_SLOT_QUEUE]);
	if (slot_h) {
		slot_h->info_h.cnn_flag = 0;
		slot_h->info_h.slot_flag = SLOT_FREE;
		slot_h->slot_get = 0;
		enqueue_slot(&g_dual_slot_queue[FREE_SLOT_QUEUE], slot_h);
	}
	return slot_h;
}

int8_t init_ipu_slot_dual(uint64_t base, slot_ddr_info_dual_t *data)
{
	int8_t i = 0;
	bitmap_zero(slot_init_mask, g_ipu->slot_num / 2);
	for (i = 0; i < SLOT_QUEUE_MAX; i++) {
		INIT_LIST_HEAD(&g_dual_slot_queue[i].queue);
		spin_lock_init(&g_dual_slot_queue[i].lock);
		g_dual_slot_queue[i].qlen = 0;
	}

	for (i = 0; i < g_ipu->slot_num / 2; i++) {
		uint64_t  slot_vaddr = IPU_GET_DUAL_SLOT(i, base);
		memset(&g_ipu_slot_dual[i], 0, sizeof(ipu_slot_dual_h_t));
		INIT_LIST_HEAD(&g_ipu_slot_dual[i].list);
		ipu_dbg("slot-%d, vaddr=%llx\n", i, slot_vaddr);

		enqueue_slot(&g_dual_slot_queue[FREE_SLOT_QUEUE], &g_ipu_slot_dual[i]);
		g_ipu_slot_dual[i].info_h.slot_id = i;
		memcpy(&g_ipu_slot_dual[i].info_h.dual_ddr_info, data, sizeof(slot_ddr_info_dual_t));

		ipu_dbg("slot-%d, crop=0x%x\n", i, g_ipu_slot_dual[i].info_h.dual_ddr_info.crop.y_offset);
		ipu_dbg("slot-%d, scale=0x%x\n", i, g_ipu_slot_dual[i].info_h.dual_ddr_info.scale.y_offset);
		ipu_dbg("ds1st[%d].y_offset=0x%x\n", i, g_ipu_slot_dual[i].info_h.dual_ddr_info.ds_1st[i].y_offset);
		ipu_dbg("ds2nd[%d].y_offset=0x%x\n", i, g_ipu_slot_dual[i].info_h.dual_ddr_info.ds_2nd[i].y_offset);
	}
	return 0;
}

int8_t ipu_slot_dual_recfg(slot_ddr_info_dual_t *data)
{
	bitmap_fill(slot_init_mask, g_ipu->slot_num / 2);
	ipu_clean_slot_queue(data);
	return 0;
}

int8_t ipu_clean_slot_queue(slot_ddr_info_dual_t *data)
{
	ipu_slot_dual_h_t	*slot_h = NULL;
	while (g_dual_slot_queue[RECVING_SLOT_QUEUE].qlen > 0) {
		slot_h = dequeue_slot(&g_dual_slot_queue[RECVING_SLOT_QUEUE]);
		insert_dual_slot_to_free(slot_h->info_h.slot_id, data);
	}
	while (g_dual_slot_queue[RECVDONE_SLOT_QUEUE].qlen > 0) {
		slot_h = dequeue_slot(&g_dual_slot_queue[RECVDONE_SLOT_QUEUE]);
		insert_dual_slot_to_free(slot_h->info_h.slot_id, data);
	}
	while (g_dual_slot_queue[PYMING_SLOT_QUEUE].qlen > 0) {
		slot_h = dequeue_slot(&g_dual_slot_queue[PYMING_SLOT_QUEUE]);
		insert_dual_slot_to_free(slot_h->info_h.slot_id, data);
	}
#if 1
	while (g_dual_slot_queue[PYMDONE_SLOT_QUEUE].qlen > 0) {
		slot_h = dequeue_slot(&g_dual_slot_queue[PYMDONE_SLOT_QUEUE]);
		insert_dual_slot_to_free(slot_h->info_h.slot_id, data);
	}
#endif
	return 0;
}
