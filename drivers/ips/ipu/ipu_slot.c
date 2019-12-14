#include <linux/spinlock.h>
#include <asm-generic/io.h>
#include "ipu_drv.h"
#include "ipu_slot.h"
#include "ipu_common.h"
#include <linux/ktime.h>
static ipu_slot_h_t 	g_ipu_slot_[IPU_MAX_SLOT];
static struct list_head g_ipu_slot_list[SLOT_LIST_NUM];
static DECLARE_BITMAP(slot_init_mask, IPU_MAX_SLOT);
extern struct x2_ipu_data *g_ipu;
extern unsigned int queue_free_cnt;
extern unsigned int queue_busy_cnt;
extern unsigned int queue_done_cnt;

/********************************************************************
 * @brief init_ipu_slot
 *
 * @param base virtual address
 * @param data
 *
 * @return
 ********************************************************************/
int8_t init_ipu_slot(uint64_t base, slot_ddr_info_t *data)
{
	int8_t i = 0;
	bitmap_zero(slot_init_mask, g_ipu->slot_num);
	INIT_LIST_HEAD(&g_ipu_slot_list[FREE_SLOT_LIST]);
	INIT_LIST_HEAD(&g_ipu_slot_list[BUSY_SLOT_LIST]);
	INIT_LIST_HEAD(&g_ipu_slot_list[DONE_SLOT_LIST]);
	for (i = 0; i < g_ipu->slot_num; i++) {
		uint64_t  slot_vaddr = IPU_GET_SLOT(i, base);
		INIT_LIST_HEAD(&g_ipu_slot_[i].list);
		ipu_dbg("slot-%d, vaddr=%llx\n", i, slot_vaddr);
		list_add_tail(&g_ipu_slot_[i].list, &g_ipu_slot_list[FREE_SLOT_LIST]);

		g_ipu_slot_[i].slot_get = 0;
		g_ipu_slot_[i].slot_cnt = 0;
		memset(&g_ipu_slot_[i].info_h, 0, sizeof(info_h_t));
		g_ipu_slot_[i].info_h.slot_id = i;
		memcpy(&g_ipu_slot_[i].info_h.ddr_info, data, sizeof(slot_ddr_info_t));
		ipu_dbg("ds[%d].y_offset=0x%x\n", i, g_ipu_slot_[i].info_h.ddr_info.ds[i].y_offset);
	}
	return 0;
}

int8_t ipu_slot_recfg(slot_ddr_info_t *data)
{
	bitmap_fill(slot_init_mask, g_ipu->slot_num);
	ipu_clean_slot(data);
	return 0;
}

int8_t ipu_clean_slot(slot_ddr_info_t *data)
{
	while (!list_empty(&g_ipu_slot_list[DONE_SLOT_LIST])) {
		if (NULL == slot_done_to_free(data))
			break;
	}
	while (!list_empty(&g_ipu_slot_list[BUSY_SLOT_LIST])) {
		if (NULL == slot_busy_to_free(data))
			break;
	}
	return 0;
}

ipu_slot_h_t* ipu_get_done_slot()
{
	struct list_head *node = NULL;
	struct list_head *head = NULL;
	ipu_slot_h_t	 *slot_h = NULL;

	if (list_empty(&g_ipu_slot_list[DONE_SLOT_LIST])) {
		return NULL;
	}
	head = &g_ipu_slot_list[DONE_SLOT_LIST];
	node = head->next;
	slot_h = (ipu_slot_h_t *)node;
	list_del_init(node);
	return slot_h;
}

ipu_slot_h_t *ipu_read_done_slot(void)
{
	struct list_head *node = NULL;
	struct list_head *head = NULL;
	ipu_slot_h_t	 *slot_h = NULL;

	if (list_empty(&g_ipu_slot_list[DONE_SLOT_LIST])) {
		ipu_info("done slot empty\n");
		return NULL;
	}
	head = &g_ipu_slot_list[DONE_SLOT_LIST];
	node = head->next;
	slot_h = (ipu_slot_h_t *)node;
	return slot_h;
}

ipu_slot_h_t* slot_free_to_busy(void)
{
	int busy_cnt = 0, done_cnt = 0;
	struct list_head *node = NULL;
	ipu_slot_h_t	 *slot_h = NULL;

	busy_cnt = slot_left_num(BUSY_SLOT_LIST);
	if (busy_cnt >= 2) {
		ipu_info("busy list already have 2 nodes\n");
		goto out;
	}
	/* get node from done list */
	if (list_empty(&g_ipu_slot_list[FREE_SLOT_LIST])) {
		done_cnt = slot_left_num(DONE_SLOT_LIST);
		if (done_cnt > 1) {
			slot_h =
			list_last_entry(&g_ipu_slot_list[DONE_SLOT_LIST],
				ipu_slot_h_t, list);
		}
	/* get node from free list */
	} else {
		node = g_ipu_slot_list[FREE_SLOT_LIST].next;
		slot_h = (ipu_slot_h_t *)node;
	}
	if (slot_h) {
		slot_h->info_h.slot_flag = SLOT_BUSY;
		slot_h->slot_cnt++;
		list_move_tail(&slot_h->list, &g_ipu_slot_list[BUSY_SLOT_LIST]);
	}
out:
	return slot_h;
}

ipu_slot_h_t *ipu_get_free_slot(void)
{
	struct list_head *node = NULL;
	struct list_head *head = NULL;
	ipu_slot_h_t	 *slot_h = NULL;
	if (list_empty(&g_ipu_slot_list[FREE_SLOT_LIST])) {
		return NULL;
	}
	head = &g_ipu_slot_list[FREE_SLOT_LIST];
	node = head->next;
	slot_h = (ipu_slot_h_t *)node;
	list_del_init(node);
	return slot_h;
}

int ipu_slot_id_check(int type, int slot_id)
{
	struct list_head *pos = NULL;
	struct list_head *n = NULL;
	ipu_slot_h_t	 *slot_h = NULL;

	list_for_each_safe(pos, n, &g_ipu_slot_list[type]) {
		slot_h = (ipu_slot_h_t *)pos;
		if (slot_h == NULL) {
			pr_err("check slot is NULL\n");
			return -ENOMEM;
		}
		if (slot_id == slot_h->info_h.slot_id) {
			pr_err("slot %d have have been in existence\n", slot_id);
			return -EFAULT;
		}
	}
	pr_debug("[%s] slot type = %d\n", __func__, type);
	return slot_id;
}

int slot_left_num(int type)
{
	struct list_head *pos = NULL;
	struct list_head *n = NULL;
	int count = 0;

	list_for_each_safe(pos, n, &g_ipu_slot_list[type])
		count++;
	pr_debug("[%s] slot type = %d count = %d\n", __func__, type, count);
	return count;
}

ipu_slot_h_t* slot_busy_to_done(void)
{
	int cnt = 0;
	struct list_head *node = NULL;
	ipu_slot_h_t	 *slot_h = NULL;

	cnt = slot_left_num(BUSY_SLOT_LIST);
	if (cnt < 2) {
		ipu_info("busy list nodes < 2\n");
		return NULL;
	}

	node = g_ipu_slot_list[BUSY_SLOT_LIST].next;
	list_move_tail(node, &g_ipu_slot_list[DONE_SLOT_LIST]);

	slot_h = (ipu_slot_h_t *)node;
	do_gettimeofday(&slot_h->tv);
	slot_h->info_h.slot_flag = SLOT_DONE;
	slot_h->slot_get = 0;
	return slot_h;
}

ipu_slot_h_t* slot_busy_to_free(slot_ddr_info_t *data)
{
	struct list_head *node = NULL;
	ipu_slot_h_t	 *slot_h = NULL;

	if (list_empty(&g_ipu_slot_list[BUSY_SLOT_LIST])) {
		return NULL;
	}
	node = g_ipu_slot_list[BUSY_SLOT_LIST].next;
	list_move_tail(node, &g_ipu_slot_list[FREE_SLOT_LIST]);
	slot_h = (ipu_slot_h_t *)node;
	if (test_and_clear_bit(slot_h->info_h.slot_id, slot_init_mask))
		memcpy(&slot_h->info_h.dual_ddr_info, data, sizeof(slot_ddr_info_dual_t));
	slot_h->info_h.cnn_flag = 0;
	slot_h->info_h.slot_flag = SLOT_FREE;
	return slot_h;
}

ipu_slot_h_t* slot_done_to_free(slot_ddr_info_t *data)
{
	struct list_head *node = NULL;
	ipu_slot_h_t	 *slot_h = NULL;

	if (list_empty(&g_ipu_slot_list[DONE_SLOT_LIST])) {
		return NULL;
	}
	node = g_ipu_slot_list[DONE_SLOT_LIST].next;
	list_move_tail(node, &g_ipu_slot_list[FREE_SLOT_LIST]);
	slot_h = (ipu_slot_h_t *)node;
	if (test_and_clear_bit(slot_h->info_h.slot_id, slot_init_mask))
		memcpy(&slot_h->info_h.dual_ddr_info, data, sizeof(slot_ddr_info_dual_t));
	slot_h->info_h.cnn_flag = 0;
	slot_h->info_h.slot_flag = SLOT_FREE;
	slot_h->slot_get = 0;
	return slot_h;
}

int insert_slot_to_free(int slot_id)
{
	ipu_slot_h_t *slot_h = NULL;
	if (slot_id < 0 || slot_id >= g_ipu->slot_num) {
		ipu_err("invalid slot id when free to done, %d\n", slot_id);
		return -1;
	}
	if ((ipu_slot_id_check(FREE_SLOT_LIST, slot_id) < 0) ||
		(ipu_slot_id_check(BUSY_SLOT_LIST, slot_id) < 0) ||
		(ipu_slot_id_check(DONE_SLOT_LIST, slot_id) < 0)) {
		pr_err("slot %d have been free !!\n", slot_id);
		return 0;
	}
	slot_h = &g_ipu_slot_[slot_id];
	list_move_tail(&slot_h->list, &g_ipu_slot_list[FREE_SLOT_LIST]);
	slot_h->info_h.cnn_flag = 0;
	slot_h->info_h.slot_flag = SLOT_FREE;
	slot_h->slot_get = 0;
	return 0;
}

bool is_slot_busy_empty(void)
{
	if (list_empty(&g_ipu_slot_list[BUSY_SLOT_LIST])) {
		return true;
	}
	return false;
}

bool is_slot_free_empty(void)
{
	if (list_empty(&g_ipu_slot_list[FREE_SLOT_LIST])) {
		return true;
	}
	return false;
}
bool is_slot_done_empty(void)
{
	if (list_empty(&g_ipu_slot_list[DONE_SLOT_LIST])) {
		return true;
	}
	return false;
}
void dump_slot_state(void)
{
	queue_free_cnt = 0;
	queue_busy_cnt = 0;
	queue_done_cnt = 0;
	struct list_head *head = NULL;
	struct list_head *node = NULL;
	int count = 0;
	head = &g_ipu_slot_list[FREE_SLOT_LIST];
	node = head->next;
	if (!list_empty(head)) {
		while (node != head) {
			count++;
			queue_free_cnt++;
			node = node->next;
		}
	}
	printk("g_ipu_slot_list[FREE_SLOT_LIST] :%d\n", count);

	head = &g_ipu_slot_list[BUSY_SLOT_LIST];
	count = 0;
	node = head->next;
	if (!list_empty(head)) {
		while (node != head) {
			count++;
			queue_busy_cnt++;
			node = node->next;
		}
	}
	printk("g_ipu_slot_list[BUSY_SLOT_LIST] :%d\n", count);

	head = &g_ipu_slot_list[DONE_SLOT_LIST];
	node = head->next;
	count = 0;
	if (!list_empty(head)) {
		while (node != head) {
			count++;
			queue_done_cnt++;
			node = node->next;
		}
	}
	printk("g_ipu_slot_list[DONE_SLOT_LIST] :%d\n", count);
}
