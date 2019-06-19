#include <linux/spinlock.h>
#include <asm-generic/io.h>
#include "ipu_slot.h"
#include "ipu_common.h"

enum {
	FREE_SLOT_LIST,
	BUSY_SLOT_LIST,
	DONE_SLOT_LIST,
	SLOT_LIST_NUM,
};

static ipu_slot_h_t 	g_ipu_slot_[IPU_MAX_SLOT];
static struct list_head g_ipu_slot_list[SLOT_LIST_NUM];
static DECLARE_BITMAP(slot_init_mask, IPU_MAX_SLOT);

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
	bitmap_zero(slot_init_mask, IPU_MAX_SLOT);
	INIT_LIST_HEAD(&g_ipu_slot_list[FREE_SLOT_LIST]);
	INIT_LIST_HEAD(&g_ipu_slot_list[BUSY_SLOT_LIST]);
	INIT_LIST_HEAD(&g_ipu_slot_list[DONE_SLOT_LIST]);
	for (i = 0; i < IPU_MAX_SLOT; i++) {
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
	bitmap_fill(slot_init_mask, IPU_MAX_SLOT);
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
	struct list_head *node = NULL;
	ipu_slot_h_t	 *slot_h = NULL;

	if (list_empty(&g_ipu_slot_list[FREE_SLOT_LIST])) {
		ipu_info("free slot empty\n");
		return NULL;
	}
	node = g_ipu_slot_list[FREE_SLOT_LIST].next;
	list_move_tail(node, &g_ipu_slot_list[BUSY_SLOT_LIST]);
	slot_h = (ipu_slot_h_t *)node;
	slot_h->info_h.slot_flag = SLOT_BUSY;
	slot_h->slot_cnt++;
	return slot_h;
}


ipu_slot_h_t *slot_free_to_done(void)
{
	struct list_head *node = NULL;
	ipu_slot_h_t	 *slot_h = NULL;

	if (list_empty(&g_ipu_slot_list[FREE_SLOT_LIST])) {
		ipu_info("free slot empty\n");
		return NULL;
	}
	node = g_ipu_slot_list[FREE_SLOT_LIST].next;
	list_move_tail(node, &g_ipu_slot_list[DONE_SLOT_LIST]);
	slot_h = (ipu_slot_h_t *)node;
	slot_h->info_h.slot_flag = SLOT_DONE;
	slot_h->slot_cnt++;
	return slot_h;
}


ipu_slot_h_t* slot_busy_to_done(void)
{
	int v = 0;
	struct list_head *this, *next;
	struct list_head *node = NULL;
	ipu_slot_h_t	 *slot_h = NULL;

	list_for_each_safe(this, next, &g_ipu_slot_list[BUSY_SLOT_LIST])
		v++;
	if (v < 2) {
		ipu_err("busy slot < 2\n");
		return NULL;
	}

	node = g_ipu_slot_list[BUSY_SLOT_LIST].next;
	list_move_tail(node, &g_ipu_slot_list[DONE_SLOT_LIST]);
	slot_h = (ipu_slot_h_t *)node;
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
	if (slot_id < 0 || slot_id >= IPU_MAX_SLOT) {
		ipu_err("invalid slot id when free to done\n");
		return -1;
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
	struct list_head *head = NULL;
	struct list_head *node = NULL;
	int count = 0;
	head = &g_ipu_slot_list[FREE_SLOT_LIST];
	node = head->next;
	if (!list_empty(head)) {
		while (node != head) {
			count++;
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
			node = node->next;
		}
	}
	printk("g_ipu_slot_list[DONE_SLOT_LIST] :%d\n", count);
}
