
#define pr_fmt(fmt) "[isp_ctxsv]: %s: " fmt, __func__

#include <linux/types.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/moduleparam.h>
#include <linux/completion.h>
#include "isp_ctxsv.h"

/*
one single context memory layout
====================================
    |--------|<---------------
    |  buf0  |---            |
    |--------|  |            |
    |  buf1  |  |            |
    |--------|cfg area       |
    |  ....  |  |            |
    |--------|  |            |
    |  buf5  |<--            |
    |--------|               |
    |  buf0  |---            |
    |--------|  |            |
    |  buf1  |  |            |
    |--------|ae area     one zone
    |  ....  |  |            |
    |--------|  |            |
    |  buf5  |<--            |
    |--------|               |
    |  buf0  |---            |
    |--------|  |            |
    |  buf1  |  |            |
    |--------|awb area       |
    |  ....  |  |            |
    |--------|  |            |
    |  buf5  |<--            |
    |--------|               |
    |  buf0  |---            |
    |--------|  |            |
    |  buf1  |  |            |
    |--------|af area        |
    |  ....  |  |            |
    |--------|  |            |
    |  buf5  |<--            |
    |--------|               |
    |  buf0  |---            |
    |--------|  |            |
    |  buf1  |  |            |
    |--------|ae_5bin area   |
    |  ....  |  |            |
    |--------|  |            |
    |  buf5  |<--            |
    |--------|<---------------

cfg, ae, awb, af, ae_5bin each have two queues: FREEQ, DONEQ
*/
enum interrupt_type {
	ISP_FS_INT = 0,
	ISP_FE_INT,
	MAX_INT_TYPE,
};

static spinlock_t lock;
static isp_ctx_node_t ctx_node[FIRMWARE_CONTEXT_NUMBER][TYPE_MAX][PER_ZONE_NODES];
static struct list_head ctx_queue[FIRMWARE_CONTEXT_NUMBER][TYPE_MAX][Q_MAX];
static struct completion irq_completion[FIRMWARE_CONTEXT_NUMBER][MAX_INT_TYPE];

extern void *isp_dev_get_vir_addr(void);
void isp_ctx_queue_state(char *tags);

static int ctx_max;
module_param(ctx_max, int, 0644);

isp_ctx_node_t *isp_ctx_get_node(int ctx_id, isp_info_type_e it, isp_ctx_queue_type_e qt)
{
	struct list_head *node;
	isp_ctx_node_t *cn = NULL;

	spin_lock(&lock);
	if (!list_empty(&ctx_queue[ctx_id][it][qt])) {

		node = ctx_queue[ctx_id][it][qt].next;
		cn = (isp_ctx_node_t *)node;
		list_del_init(node);
	}
	spin_unlock(&lock);

	isp_ctx_queue_state("get");

	return cn;
}

void isp_ctx_put_node(int ctx_id, isp_ctx_node_t *cn, isp_info_type_e it, isp_ctx_queue_type_e qt)
{
	spin_lock(&lock);
	list_move_tail(&cn->node, &ctx_queue[ctx_id][it][qt]);
	spin_unlock(&lock);

	isp_ctx_queue_state("put");
}

void isp_ctx_put(int ctx_id, isp_info_type_e type, uint8_t idx)
{
	isp_ctx_put_node(ctx_id, &ctx_node[ctx_id][type][idx], type, FREEQ);
}

void isp_ctx_done_queue_clear(int ctx_id)
{
	int k = 0;
 
	if (!ctx_queue[ctx_id][k][DONEQ].next)
		return;

	for (k = 0; k < TYPE_MAX; k++) {
		if (!list_empty(&ctx_queue[ctx_id][k][DONEQ]))
			list_splice_tail_init(&ctx_queue[ctx_id][k][DONEQ], &ctx_queue[ctx_id][k][FREEQ]);
	}
}

int isp_ctx_queue_init(void)
{
	int i, j, k;
	void *base = NULL;
	void *cfg_base = NULL;
	void *ae_base = NULL;
	void *awb_base = NULL;
	void *af_base = NULL;
	void *ae_5bin_base = NULL;

	base = isp_dev_get_vir_addr();

	for (i = 0; i < FIRMWARE_CONTEXT_NUMBER; i++) {
		for (j = 0; j < TYPE_MAX; j++) {
			INIT_LIST_HEAD(&ctx_queue[i][j][FREEQ]);
			INIT_LIST_HEAD(&ctx_queue[i][j][DONEQ]);
		}
	}

        for (i = 0; i < FIRMWARE_CONTEXT_NUMBER; i++) {
		for (j = 0; j < PER_ZONE_NODES; j++) {
			cfg_base = base + i * ONE_ZONE_SIZE;
			ae_base = cfg_base + CFG_SIZE_IN_ONE_ZONE;
			awb_base = ae_base + AE_SIZE_IN_ONE_ZONE;
			af_base = awb_base + AWB_SIZE_IN_ONE_ZONE;
			ae_5bin_base = af_base + AF_SIZE_IN_ONE_ZONE;
			ctx_node[i][ISP_CTX][j].base = cfg_base + j * CFG_NODE_SIZE;
			ctx_node[i][ISP_AE][j].base = ae_base + j * AE_NODE_SIZE;
			ctx_node[i][ISP_AWB][j].base = awb_base + j * AWB_NODE_SIZE;
			ctx_node[i][ISP_AF][j].base = af_base + j * AF_NODE_SIZE;
			ctx_node[i][ISP_AE_5BIN][j].base = ae_5bin_base + j * AE_5BIN_NODE_SIZE;

			for (k = 0; k < TYPE_MAX; k++) {
				ctx_node[i][k][j].ctx.idx = j;
				INIT_LIST_HEAD(&ctx_node[i][k][j].node);
				list_add_tail(&ctx_node[i][k][j].node, &ctx_queue[i][k][FREEQ]);
			}
		}
	}

	spin_lock_init(&lock);

	// init isp interrupt completion
        for (i = 0; i < FIRMWARE_CONTEXT_NUMBER; i++) {
		for (j = 0; j < MAX_INT_TYPE; j++) {
			init_completion(&irq_completion[i][j]);
		}
	}

	pr_debug("init done\n");

	return 0;
}

void isp_ctx_queue_state(char *tags)
{
	int i, j, k1, k2;
	struct list_head *this, *next;

	if (ctx_max > FIRMWARE_CONTEXT_NUMBER)
		ctx_max = FIRMWARE_CONTEXT_NUMBER;

	pr_debug("--op %s--\n", tags);
	for (i = 0; i < ctx_max; i++) {
		for (j = 0; j < TYPE_MAX; j++) {
			k1 = 0, k2 = 0;
			list_for_each_safe(this, next, &ctx_queue[i][j][FREEQ])
				k1++;

			list_for_each_safe(this, next, &ctx_queue[i][j][DONEQ])
				k2++;

			pr_debug("ctx[%d] type[%d] free queue count %d\n", i, j, k1);
			pr_debug("ctx[%d] type[%d] done queue count %d\n", i, j, k2);
		}
	}

}

int isp_irq_wait_for_completion(int ctx_id, uint8_t irq_type, unsigned long timeout)
{
	int ret = 0;
	unsigned long td = 0;
	if ((ctx_id >= FIRMWARE_CONTEXT_NUMBER) || (irq_type >= MAX_INT_TYPE)) {
		pr_err("param is err, ctx[%d] or irq_type[%d] is err!\n", ctx_id, irq_type);
		return -1;
	}
	td = wait_for_completion_timeout(&irq_completion[ctx_id][irq_type], timeout);
	if (!td) {
		pr_debug("ctx[%d] irq_type[%d] is time_out\n", ctx_id, irq_type);
		ret = -1;
	}
	// pr_info("ctx %d, irq_type %d is require, time out is %d!\n", ctx_id, irq_type, td);
	return ret;
}

void isp_irq_completion(int ctx_id, uint8_t irq_type)
{
	if ((ctx_id >= FIRMWARE_CONTEXT_NUMBER) || (irq_type >= MAX_INT_TYPE)) {
		pr_err("param is err, ctx[%d] or irq_type[%d] is err!\n", ctx_id, irq_type);
		return;
	}
	//complete_all(&irq_completion[ctx_id][irq_type]);
}

