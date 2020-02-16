
#define pr_fmt(fmt) "[isp_ctxsv]: %s: " fmt, __func__

#include <linux/types.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/moduleparam.h>
#include "isp_ctxsv.h"
#include "acamera_firmware_config.h"

static spinlock_t lock;
static isp_ctx_node_t ctx_node[PER_ZONE_NODES*ZONE_MAX];
static struct list_head ctx_queue[FIRMWARE_CONTEXT_NUMBER][BUTT];

extern void *isp_dev_get_vir_addr(void);
void isp_ctx_queue_state(char *tags);

static int ctx_max;
module_param(ctx_max, int, 0644);

isp_ctx_node_t *isp_ctx_get_node(int ctx_id, isp_ctx_queue_type_e type)
{
	struct list_head *node;
	isp_ctx_node_t *cn = NULL;

	spin_lock(&lock);
	if (!list_empty(&ctx_queue[ctx_id][type])) {
		node = ctx_queue[ctx_id][type].next;
		cn = (isp_ctx_node_t *)node;
		list_del_init(node);
	}
	spin_unlock(&lock);

	isp_ctx_queue_state("get");

	return cn;
}

void isp_ctx_put_node(int ctx_id, isp_ctx_node_t *cn, isp_ctx_queue_type_e type)
{
	spin_lock(&lock);
	list_move_tail(&cn->node, &ctx_queue[ctx_id][type]);
	spin_unlock(&lock);

	isp_ctx_queue_state("put");
}

void isp_ctx_put(int ctx_id, uint8_t idx)
{
	isp_ctx_put_node(ctx_id, &ctx_node[idx], FREEQ);
}

int isp_ctx_queue_init(void)
{
	int i, j, k;
	void *base = NULL;

	base = isp_dev_get_vir_addr();

	for (i = 0; i < FIRMWARE_CONTEXT_NUMBER; i++) {
		INIT_LIST_HEAD(&ctx_queue[i][FREEQ]);
		INIT_LIST_HEAD(&ctx_queue[i][BUSYQ]);
		INIT_LIST_HEAD(&ctx_queue[i][DONEQ]);
	}

        for (i = 0; i < ZONE_MAX; i++) {
        	for (j = 0; j < PER_ZONE_NODES; j++) {
			k = i * PER_ZONE_NODES + j;
			ctx_node[k].base = base + k * NODE_SIZE;
			ctx_node[k].ctx.idx = k;
			INIT_LIST_HEAD(&ctx_node[k].node);
			list_add_tail(&ctx_node[k].node, &ctx_queue[i][FREEQ]);
		}
	}

	spin_lock_init(&lock);

	pr_info("init done\n");

	return 0;
}

void isp_ctx_queue_state(char *tags)
{
	int i, k1, k2, k3; 
	struct list_head *this, *next;

	if (ctx_max > FIRMWARE_CONTEXT_NUMBER)
		ctx_max = FIRMWARE_CONTEXT_NUMBER;

	pr_debug("--op %s--\n", tags);
	for (i = 0; i < ctx_max; i++) {
		k1 = 0, k2 = 0, k3 = 0; 
		list_for_each_safe(this, next, &ctx_queue[i][FREEQ])
			k1++;

		list_for_each_safe(this, next, &ctx_queue[i][BUSYQ])
			k2++;

		list_for_each_safe(this, next, &ctx_queue[i][DONEQ])
			k3++;

		pr_debug("ctx[%d] free queue count %d\n", i, k1);
		pr_debug("ctx[%d] busy queue count %d\n", i, k2);
		pr_debug("ctx[%d] done queue count %d\n", i, k3);
	}

}
