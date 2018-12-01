#include <linux/spinlock.h>
#include "ipu_slot.h"
#include "ipu_common.h"

static spinlock_t g_listlock;

static ipu_slot_h_t *g_ipu_slot_[IPU_MAX_SLOT];
static struct list_head g_free_list;
static struct list_head g_busy_list;
static struct list_head g_done_list;

/********************************************************************
 * @brief init_ipu_slot
 *
 * @param base virtual address
 * @param data
 *
 * @return
 ********************************************************************/
int8_t init_ipu_slot(uint64_t base, slot_ddr_info_t * data)
{
	int8_t i = 0;

	INIT_LIST_HEAD(&g_free_list);
	INIT_LIST_HEAD(&g_busy_list);
	INIT_LIST_HEAD(&g_done_list);

	for (i = 0; i < IPU_MAX_SLOT; i++) {
		g_ipu_slot_[i] = (ipu_slot_h_t *) IPU_GET_SLOT(i, base);
		ipu_dbg("slot-%d, vaddr=%llx\n", i, (uint64_t) g_ipu_slot_[i]);
		INIT_LIST_HEAD(&g_ipu_slot_[i]->list);
		list_add_tail(&g_ipu_slot_[i]->list, &g_free_list);

		g_ipu_slot_[i]->slot_id = i;
#if 0
		g_ipu_slot_[i]->ddr_info.crop.y_offset = data->crop.y_offset;
		g_ipu_slot_[i]->ddr_info.scale.offset = data->scale.offset;
		g_ipu_slot_[i]->ddr_info.scale.size = data->scale.size;
		for (j = 0; j < 23; j++) {
			g_ipu_slot_[i]->ddr_info.ds[j].offset =
			    data->ds[j].offset;
			g_ipu_slot_[i]->ddr_info.ds[j].size = data->ds[j].size;
		}
		for (j = 0; j < 6; j++) {
			g_ipu_slot_[i]->ddr_info.us[j].offset =
			    data->us[j].offset;
			g_ipu_slot_[i]->ddr_info.us[j].size = data->us[j].size;
		}
#else
		memcpy(&g_ipu_slot_[i]->ddr_info, data,
		       sizeof(slot_ddr_info_t));
		ipu_dbg("ds[%d].y_offset=0x%x\n", i,
			g_ipu_slot_[i]->ddr_info.ds[i].y_offset);
#endif
		g_ipu_slot_[i]->slot_flag = 0;
		g_ipu_slot_[i]->slot_get = 0;
		g_ipu_slot_[i]->ipu_flag = 0;
		g_ipu_slot_[i]->cnn_flag = 0;
	}

	spin_lock_init(&g_listlock);
	return 0;
}

int8_t ipu_clean_slot(void)
{
	while (!list_empty(&g_busy_list)) {
		if (NULL == slot_busy_to_free())
			break;
	}

	while (!list_empty(&g_done_list)) {
		if (NULL == slot_done_to_free())
			break;
	}
	return 0;
}

ipu_slot_h_t *ipu_get_done_slot()
{
	struct list_head *node = NULL;
	ipu_slot_h_t *slot_h = NULL;

	spin_lock(&g_listlock);
	if (list_empty(&g_done_list)) {
		spin_unlock(&g_listlock);
		return NULL;
	}
	node = g_done_list.next;
	while (NULL != node) {
		slot_h = (ipu_slot_h_t *) node;
		if (!slot_h->slot_get) {
			slot_h->slot_get = 1;
			break;
		}
		slot_h = NULL;
		node = node->next;
	}
	spin_unlock(&g_listlock);

	return slot_h;
}

ipu_slot_h_t *slot_free_to_busy(void)
{
	struct list_head *node = NULL;
	ipu_slot_h_t *slot_h = NULL;
	spin_lock(&g_listlock);
	if (list_empty(&g_free_list)) {
		spin_unlock(&g_listlock);
		return NULL;
	}
	node = g_free_list.next;
	list_move_tail(node, &g_busy_list);
	slot_h = (ipu_slot_h_t *) node;
	slot_h->slot_flag = SLOT_BUSY;
	spin_unlock(&g_listlock);
	return slot_h;
}

ipu_slot_h_t *slot_busy_to_done(void)
{
	struct list_head *node = NULL;
	ipu_slot_h_t *slot_h = NULL;
	spin_lock(&g_listlock);
	if (list_empty(&g_busy_list)) {
		spin_unlock(&g_listlock);
		return NULL;
	}
	node = g_busy_list.next;
	list_move_tail(node, &g_done_list);
	slot_h = (ipu_slot_h_t *) node;
	slot_h->slot_flag = SLOT_DONE;
	slot_h->slot_get = 0;
	spin_unlock(&g_listlock);
	return slot_h;
}

ipu_slot_h_t *slot_busy_to_free(void)
{
	struct list_head *node = NULL;
	ipu_slot_h_t *slot_h = NULL;
	spin_lock(&g_listlock);
	if (list_empty(&g_busy_list)) {
		spin_unlock(&g_listlock);
		return NULL;
	}
	node = g_busy_list.next;
	list_move_tail(node, &g_free_list);
	slot_h = (ipu_slot_h_t *) node;
	slot_h->cnn_flag = 0;
	slot_h->slot_flag = SLOT_FREE;
	spin_unlock(&g_listlock);
	return slot_h;
}

ipu_slot_h_t *slot_done_to_free(void)
{
	struct list_head *node = NULL;
	ipu_slot_h_t *slot_h = NULL;
	spin_lock(&g_listlock);
	if (list_empty(&g_done_list)) {
		spin_unlock(&g_listlock);
		return NULL;
	}
	node = g_done_list.next;
	list_move_tail(node, &g_free_list);
	slot_h = (ipu_slot_h_t *) node;
	slot_h->cnn_flag = 0;
	slot_h->slot_get = 0;
	slot_h->slot_flag = SLOT_FREE;
	spin_unlock(&g_listlock);
	return slot_h;
}
