
#define pr_fmt(fmt) "[isp_ctxsv]: %s: " fmt, __func__

#include <linux/types.h>
#include <linux/list.h>
#include <linux/spinlock.h>
#include <linux/semaphore.h>
#include <linux/moduleparam.h>
#include <linux/completion.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include "isp_ctxsv.h"

// PRQA S 0844,0497,0685,0636,0605 ++

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

typedef struct isp_ctx_head_s {
	struct list_head ctx_node_head[Q_MAX];
	struct semaphore sem;
}isp_ctx_head;

static isp_ctx_node_t ctx_node[FIRMWARE_CONTEXT_NUMBER][TYPE_MAX][PER_ZONE_NODES];
static isp_ctx_head ctx_queue[FIRMWARE_CONTEXT_NUMBER][TYPE_MAX];
static struct completion irq_completion[FIRMWARE_CONTEXT_NUMBER][MAX_INT_TYPE];
static uint8_t completion_flag = 0;

extern void *isp_dev_get_vir_addr(void);
void isp_ctx_queue_state(char *tags);

int ctx_max;
module_param(ctx_max, int, 0644);

isp_ctx_node_t *isp_ctx_get_node(int ctx_id, isp_info_type_e it, isp_ctx_queue_type_e qt)
{
	struct list_head *node;
	isp_ctx_node_t *cn = NULL;

	spin_lock(&lock);
	if (!list_empty(&ctx_queue[ctx_id][it].ctx_node_head[qt])) {
		node = ctx_queue[ctx_id][it].ctx_node_head[qt].next;
		cn = (isp_ctx_node_t *)node;
		list_del_init(node);
	} else if (qt == FREEQ) {
		node = ctx_queue[ctx_id][it].ctx_node_head[DONEQ].next;
		if (!node) {
			pr_err("DONEQ node is null\n");
			return NULL;
		}
		cn = (isp_ctx_node_t *)node;
		list_del_init(node);
	}
	spin_unlock(&lock);

	isp_ctx_queue_state("get");

	return cn;
}


int isp_ctx_flush_queueto(int ctx_id, isp_info_type_e it,
		isp_ctx_queue_type_e src_queue,
		isp_ctx_queue_type_e dst_queue)
{
	int ret = 0;
	struct list_head *node_pos, *node_next;

	list_for_each_safe(node_pos, node_next, &ctx_queue[ctx_id][it].ctx_node_head[src_queue]) {
		if (DONEQ == src_queue) {
			ret = down_trylock(&ctx_queue[ctx_id][it].sem);
		}
	}

	list_splice_tail_init(&ctx_queue[ctx_id][it].ctx_node_head[src_queue],
			&ctx_queue[ctx_id][it].ctx_node_head[dst_queue]);

	return ret;
}

isp_ctx_node_t *isp_ctx_get_node_timeout(int ctx_id, isp_info_type_e it,
				isp_ctx_queue_type_e qt, int32_t timeout, int latest_flag)
{
	struct list_head *node;
	isp_ctx_node_t *cn = NULL;
	int ret = 0;
	int ret1 = 0;
	int is_empty;

	spin_lock(&lock);
	is_empty = list_empty(&ctx_queue[ctx_id][it].ctx_node_head[qt]);
	spin_unlock(&lock);
	if (unlikely(is_empty == 1)) {
		system_chardev_unlock();
		ret = down_timeout(&ctx_queue[ctx_id][it].sem,
						msecs_to_jiffies(timeout));
		system_chardev_lock();
	} else {
		if(latest_flag == 1) {
			spin_lock(&lock);
			isp_ctx_flush_queueto(ctx_id, it, DONEQ, FREEQ);
			spin_unlock(&lock);
			system_chardev_unlock();
			ret = down_timeout(&ctx_queue[ctx_id][it].sem,
							msecs_to_jiffies(timeout));
			system_chardev_lock();
		} else {
			ret1 = down_trylock(&ctx_queue[ctx_id][it].sem);
		}
	}
	if (ret >= 0) {
		spin_lock(&lock);
		if (!list_empty(&ctx_queue[ctx_id][it].ctx_node_head[qt])) {
			node = ctx_queue[ctx_id][it].ctx_node_head[qt].next;
			cn = (isp_ctx_node_t *)node;
			list_del_init(node);
		}
		spin_unlock(&lock);
		isp_ctx_queue_state("get");
	}
	return cn;
}

isp_ctx_node_t *isp_ctx_get_node_timeout_conditional(int ctx_id, isp_info_type_e it,
				int frame_id, isp_ctx_queue_type_e qt, int32_t timeout)
{
	struct list_head *node, *next;
	isp_ctx_node_t *cn = NULL;
	int ret = 0;
	int ret1 = 0;
	int is_empty;
	spin_lock(&lock);
	is_empty = list_empty(&ctx_queue[ctx_id][it].ctx_node_head[qt]);
	spin_unlock(&lock);
	if (unlikely(is_empty == 1)) {
		system_chardev_unlock();
		ret = down_timeout(&ctx_queue[ctx_id][it].sem,
						msecs_to_jiffies(timeout));
		system_chardev_lock();
	} else {
		ret1 = down_trylock(&ctx_queue[ctx_id][it].sem);
	}

	if (ret >= 0) {
		spin_lock(&lock);
		if (!list_empty(&ctx_queue[ctx_id][it].ctx_node_head[qt])) {
			list_for_each_safe(node, next, &ctx_queue[ctx_id][it].ctx_node_head[qt]) {
				cn = (isp_ctx_node_t *)node;
				if (cn->ctx.frame_id == frame_id) {
					list_del_init(node);
					break;
				} else if (cn->ctx.frame_id < frame_id) {
					list_move_tail(&cn->node, &ctx_queue[ctx_id][it].ctx_node_head[FREEQ]);
					cn = NULL;
				}
			}
		}
		spin_unlock(&lock);
		isp_ctx_queue_state("get");
	}
	if (!cn) {
		pr_err("can't get related statistics of type %d for frame_id %d",
		it, frame_id);
	}
	return cn;
}

void isp_ctx_put_node(int ctx_id, isp_ctx_node_t *cn, isp_info_type_e it, isp_ctx_queue_type_e qt)
{
	spin_lock(&lock);
	list_move_tail(&cn->node, &ctx_queue[ctx_id][it].ctx_node_head[qt]);
	spin_unlock(&lock);
	isp_ctx_queue_state("put");

	if(qt == DONEQ) {
		up(&ctx_queue[ctx_id][it].sem);
	}
}

isp_ctx_node_t * isp_ctx_get(int ctx_id, isp_info_type_e it, int32_t timeout, int32_t latest_flag)
{
	return isp_ctx_get_node_timeout(ctx_id, it, DONEQ, timeout, latest_flag);
}

isp_ctx_node_t *isp_ctx_get_conditional(int ctx_id, isp_info_type_e it, int frame_id, int32_t timeout)
{
	return isp_ctx_get_node_timeout_conditional(ctx_id, it, frame_id, DONEQ, timeout);
}

void isp_ctx_put(int ctx_id, isp_info_type_e type, uint8_t idx)
{
	isp_ctx_put_node(ctx_id, &ctx_node[ctx_id][type][idx], type, FREEQ);
}

void isp_ctx_done_queue_clear(int ctx_id)
{
	int k = 0;

	if (!ctx_queue[ctx_id][k].ctx_node_head[DONEQ].next)
		return;

	for (k = 0; k < TYPE_MAX; k++) {
		if (!list_empty(&ctx_queue[ctx_id][k].ctx_node_head[DONEQ]))
			list_splice_tail_init(&ctx_queue[ctx_id][k].ctx_node_head[DONEQ],
			&ctx_queue[ctx_id][k].ctx_node_head[FREEQ]);
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
	void *lumvar_base = NULL;

	base = isp_dev_get_vir_addr();

	for (i = 0; i < FIRMWARE_CONTEXT_NUMBER; i++) {
		for (j = 0; j < TYPE_MAX; j++) {
			sema_init(&ctx_queue[i][j].sem, 0);
			INIT_LIST_HEAD(&ctx_queue[i][j].ctx_node_head[FREEQ]);
			INIT_LIST_HEAD(&ctx_queue[i][j].ctx_node_head[DONEQ]);
		}
	}

        for (i = 0; i < FIRMWARE_CONTEXT_NUMBER; i++) {
		for (j = 0; j < PER_ZONE_NODES; j++) {
			cfg_base = base + i * ONE_ZONE_SIZE;
			ae_base = cfg_base + CFG_SIZE_IN_ONE_ZONE;
			awb_base = ae_base + AE_SIZE_IN_ONE_ZONE;
			af_base = awb_base + AWB_SIZE_IN_ONE_ZONE;
			ae_5bin_base = af_base + AF_SIZE_IN_ONE_ZONE;
			lumvar_base = ae_5bin_base + AE_5BIN_SIZE_IN_ONE_ZONE;
			ctx_node[i][ISP_CTX][j].base = cfg_base + j * CFG_NODE_SIZE;
			ctx_node[i][ISP_AE][j].base = ae_base + j * AE_NODE_SIZE;
			ctx_node[i][ISP_AWB][j].base = awb_base + j * AWB_NODE_SIZE;
			ctx_node[i][ISP_AF][j].base = af_base + j * AF_NODE_SIZE;
			ctx_node[i][ISP_AE_5BIN][j].base = ae_5bin_base + j * AE_5BIN_NODE_SIZE;
			ctx_node[i][ISP_LUMVAR][j].base = lumvar_base + j * LUMVAR_NODE_SIZE;

			ctx_node[i][ISP_CTX][j].ctx.ctx_id = i;
			ctx_node[i][ISP_CTX][j].ctx.type = ISP_CTX;
			ctx_node[i][ISP_AE][j].ctx.ctx_id = i;
			ctx_node[i][ISP_AE][j].ctx.type = ISP_AE;
			ctx_node[i][ISP_AWB][j].ctx.ctx_id = i;
			ctx_node[i][ISP_AWB][j].ctx.type = ISP_AWB;
			ctx_node[i][ISP_AF][j].ctx.ctx_id = i;
			ctx_node[i][ISP_AF][j].ctx.type = ISP_AF;
			ctx_node[i][ISP_AE_5BIN][j].ctx.ctx_id = i;
			ctx_node[i][ISP_AE_5BIN][j].ctx.type = ISP_AE_5BIN;
			ctx_node[i][ISP_LUMVAR][j].ctx.ctx_id = i;
			ctx_node[i][ISP_LUMVAR][j].ctx.type = ISP_LUMVAR;

			for (k = 0; k < TYPE_MAX; k++) {
				ctx_node[i][k][j].ctx.idx = j;
				INIT_LIST_HEAD(&ctx_node[i][k][j].node);
				list_add_tail(&ctx_node[i][k][j].node,
							&ctx_queue[i][k].ctx_node_head[FREEQ]);
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
	completion_flag = 1;

	pr_debug("init done\n"); /* PRQA S ALL */

	return 0;
}

void isp_ctx_queue_state(char *tags)
{
	int i, j, k1, k2;
	struct list_head *this, *next;

	if (ctx_max > FIRMWARE_CONTEXT_NUMBER)
		ctx_max = FIRMWARE_CONTEXT_NUMBER;

	pr_debug("--op %s--\n", tags); /* PRQA S ALL */
	for (i = 0; i < ctx_max; i++) {
		for (j = 0; j < TYPE_MAX; j++) {
			k1 = 0, k2 = 0;
			spin_lock(&lock);
			list_for_each_safe(this, next, &ctx_queue[i][j].ctx_node_head[FREEQ])
				k1++;

			list_for_each_safe(this, next, &ctx_queue[i][j].ctx_node_head[DONEQ])
				k2++;
			spin_unlock(&lock);
			pr_debug("ctx[%d] type[%d] free queue count %d\n", i, j, k1); /* PRQA S ALL */
			pr_debug("ctx[%d] type[%d] done queue count %d\n", i, j, k2); /* PRQA S ALL */
		}
	}

}

static DECLARE_WAIT_QUEUE_HEAD(frame_start);
static DECLARE_WAIT_QUEUE_HEAD(frame_end);

static uint32_t frame_start_cond[8];
static uint32_t frame_end_cond[8];

int isp_irq_wait_for_completion(int ctx_id, uint8_t irq_type, unsigned long timeout)
{
	int ret = 0;
	unsigned long td = 0;
	if ((ctx_id >= FIRMWARE_CONTEXT_NUMBER) || (irq_type >= MAX_INT_TYPE)) {
		pr_err("param is err, ctx[%d] or irq_type[%d] is err!\n", ctx_id, irq_type); /* PRQA S ALL */
		return -1;
	}
	if (irq_type == 0) {
		td = wait_event_timeout(frame_start, frame_start_cond[ctx_id], msecs_to_jiffies(timeout)); /* PRQA S ALL */
		frame_start_cond[ctx_id] = 0;
	} else {
		td = wait_event_timeout(frame_end, frame_end_cond[ctx_id], msecs_to_jiffies(timeout)); /* PRQA S ALL */
		frame_end_cond[ctx_id] = 0;
	}
	// td = wait_for_completion_timeout(&irq_completion[ctx_id][irq_type], msecs_to_jiffies(timeout));
	if (!td) {
		pr_debug("ctx[%d] irq_type[%d] is time_out\n", ctx_id, irq_type); /* PRQA S ALL */ /* print api */
		ret = -1;
	}
	pr_debug("ctx %d, irq_type %d is require, time is %lu!\n", ctx_id, irq_type, td); /* PRQA S ALL */ /* print api */
	return ret;
}

void isp_irq_completion(int ctx_id, uint8_t irq_type)
{
	if ((ctx_id >= FIRMWARE_CONTEXT_NUMBER) || (irq_type >= MAX_INT_TYPE)) {
		pr_err("param is err, ctx[%d] or irq_type[%d] is err!\n", ctx_id, irq_type);
		return;
	}
	if (completion_flag) {
		if (irq_type == 0) {
			wake_up(&frame_start);
			frame_start_cond[ctx_id] = 1;
			frame_end_cond[ctx_id] = 0;
		} else {
			wake_up(&frame_end);
			frame_start_cond[ctx_id] = 0;
			frame_end_cond[ctx_id] = 1;
		}
		// complete_all(&irq_completion[ctx_id][irq_type]);
		// complete(&irq_completion[ctx_id][irq_type]);
	}
}

// PRQA S --
