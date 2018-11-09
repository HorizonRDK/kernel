#include "ipu_slot.h"
#include "ipu_common.h"

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
	int8_t i = 0, j = 0;

	INIT_LIST_HEAD(&g_free_list);
	INIT_LIST_HEAD(&g_busy_list);
	INIT_LIST_HEAD(&g_done_list);

	for (i = 0; i < IPU_MAX_SLOT; i++) {
		g_ipu_slot_[i] = (ipu_slot_h_t *) IPU_GET_SLOT(i, base);
		ipu_dbg("slot-%d, vaddr=%llx\n", i, (uint64_t) g_ipu_slot_[i]);
		INIT_LIST_HEAD(&g_ipu_slot_[i]->list);
		list_add_tail(&g_ipu_slot_[i]->list, &g_free_list);

		g_ipu_slot_[i]->slot_id = i;
		g_ipu_slot_[i]->ddr_info.crop.offset = data->crop.offset;
		g_ipu_slot_[i]->ddr_info.crop.size = data->crop.size;
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
		g_ipu_slot_[i]->slot_flag = 0;
		g_ipu_slot_[i]->ipu_flag = 0;
		g_ipu_slot_[i]->cnn_flag = 0;
	}

	return 0;
}

ipu_slot_h_t *ipu_get_free_slot()
{
	ipu_slot_h_t *slot;

	if (list_empty(&g_free_list)) {
		return NULL;
	}
	slot = (ipu_slot_h_t *) g_free_list.next;
	ipu_dbg("get-free-slot, vaddr=%llx\n", (uint64_t) slot);

	return slot;
}

int8_t slot_to_busy_list(ipu_slot_h_t * slot_h)
{
	struct list_head *node = (struct list_head *)slot_h;
	list_del(node);
	list_add_tail(node, &g_busy_list);
	slot_h->slot_flag = SLOT_BUSY;

	return 0;
}

ipu_slot_h_t *ipu_get_busy_slot()
{
	struct list_head *node;
	ipu_slot_h_t *slot;

	if (list_empty(&g_busy_list)) {
		return NULL;
	}
	node = g_busy_list.next;
	slot = (ipu_slot_h_t *) node;

	return slot;
}

int8_t slot_to_done_list(ipu_slot_h_t * slot_h)
{
	struct list_head *node = (struct list_head *)slot_h;
	list_del(node);
	list_add_tail(node, &g_done_list);
	slot_h->slot_flag = SLOT_DONE;

	return 0;
}

ipu_slot_h_t *ipu_get_done_slot()
{
	struct list_head *node;
	ipu_slot_h_t *slot;

	if (list_empty(&g_done_list)) {
		return NULL;
	}
	node = g_done_list.next;
	slot = (ipu_slot_h_t *) node;

	return slot;
}

int8_t slot_to_free_list(ipu_slot_h_t * slot_h)
{
	struct list_head *node = (struct list_head *)slot_h;
	list_del(node);
	list_add_tail(node, &g_free_list);
	slot_h->cnn_flag = 0;
	slot_h->slot_flag = SLOT_FREE;

	return 0;
}
