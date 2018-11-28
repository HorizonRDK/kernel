#include <linux/spinlock.h>
#include "ipu_slot.h"
#include "ipu_common.h"

typedef struct {
	spinlock_t sl_free;
	spinlock_t sl_busy;
	spinlock_t sl_done;
} ipu_slot_private_data;
ipu_slot_private_data g_slot_p;

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
#endif
		g_ipu_slot_[i]->slot_flag = 0;
		g_ipu_slot_[i]->ipu_flag = 0;
		g_ipu_slot_[i]->cnn_flag = 0;
	}

	spin_lock_init(&g_slot_p.sl_free);
	spin_lock_init(&g_slot_p.sl_busy);
	spin_lock_init(&g_slot_p.sl_done);

	return 0;
}

int8_t ipu_clean_slot(void)
{
	ipu_slot_h_t *tmp = NULL;

	while (!list_empty(&g_busy_list)) {
		tmp = ipu_get_busy_slot();
		slot_to_free_list(tmp);
	}

	while (!list_empty(&g_done_list)) {
		tmp = ipu_get_done_slot();
		slot_to_free_list(tmp);
	}
	return 0;
}

ipu_slot_h_t *ipu_get_free_slot()
{
	ipu_slot_h_t *slot;

	spin_lock(&g_slot_p.sl_free);
	if (list_empty(&g_free_list)) {
		spin_unlock(&g_slot_p.sl_free);
		return NULL;
	}
	slot = (ipu_slot_h_t *) g_free_list.next;
	spin_unlock(&g_slot_p.sl_free);
	ipu_dbg("get-free-slot, vaddr=%llx\n", (uint64_t) slot);

	return slot;
}

int8_t slot_to_busy_list(ipu_slot_h_t * slot_h)
{
	struct list_head *node = (struct list_head *)slot_h;

	//spin_lock(&g_slot_p.sl_free);
	list_del(node);
	list_add_tail(node, &g_busy_list);
	slot_h->slot_flag = SLOT_BUSY;
	//spin_unlock(&g_slot_p.sl_free);

	return 0;
}

ipu_slot_h_t *ipu_get_busy_slot()
{
	struct list_head *node;
	ipu_slot_h_t *slot;

	spin_lock(&g_slot_p.sl_busy);
	if (list_empty(&g_busy_list)) {
		spin_unlock(&g_slot_p.sl_busy);
		return NULL;
	}
	node = g_busy_list.next;
	slot = (ipu_slot_h_t *) node;
	spin_unlock(&g_slot_p.sl_busy);

	return slot;
}

int8_t slot_to_done_list(ipu_slot_h_t * slot_h)
{
	struct list_head *node = (struct list_head *)slot_h;

	//spin_lock(&g_slot_p.sl_busy);
	list_del(node);
	list_add_tail(node, &g_done_list);
	slot_h->slot_flag = SLOT_DONE;
	//spin_unlock(&g_slot_p.sl_busy);

	return 0;
}

ipu_slot_h_t *ipu_get_done_slot()
{
	struct list_head *node;
	ipu_slot_h_t *slot;

	spin_lock(&g_slot_p.sl_done);
	if (list_empty(&g_done_list)) {
		spin_unlock(&g_slot_p.sl_done);
		return NULL;
	}
	node = g_done_list.next;
	slot = (ipu_slot_h_t *) node;
	spin_unlock(&g_slot_p.sl_done);

	return slot;
}

int8_t slot_to_free_list(ipu_slot_h_t * slot_h)
{
	struct list_head *node = (struct list_head *)slot_h;

	//spin_lock(&g_slot_p.sl_done);
	list_del(node);
	list_add_tail(node, &g_free_list);
	slot_h->cnn_flag = 0;
	slot_h->slot_flag = SLOT_FREE;
	//spin_unlock(&g_slot_p.sl_done);

	return 0;
}
