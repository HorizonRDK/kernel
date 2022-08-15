/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#define pr_fmt(fmt) "vio_group_api: " fmt
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/interrupt.h>
#include <uapi/linux/sched/types.h>
#include <linux/stacktrace.h>
#include "vio_group_api.h"

static struct vio_core iscore;
u32 ldc_reset_flag, sif_module_exit;

/*
 * ldc_access_mutex not only protect access of ldc_reset_flag,
 * also protect the entire process of starting read data of sifddrin
 */
struct mutex ldc_access_mutex;
struct mutex rst_mutex;
static u32 stat_info_update = 1;
struct vio_frame_id  sif_frame_info[VIO_MAX_STREAM];
struct vio_frame_id  ipu_frame_info[VIO_MAX_STREAM];
struct vio_osd_info *osd_info[VIO_MAX_STREAM][MAX_SUB_DEVICE];

EXPORT_SYMBOL(sif_frame_info);
EXPORT_SYMBOL(ipu_frame_info);

void vio_ldc_access_mutex_lock(void)
{
	mutex_lock(&ldc_access_mutex);
}
EXPORT_SYMBOL(vio_ldc_access_mutex_lock);

void vio_ldc_access_mutex_unlock(void)
{
	mutex_unlock(&ldc_access_mutex);
}
EXPORT_SYMBOL(vio_ldc_access_mutex_unlock); /*PRQA S ALL*/

void vio_init_ldc_access_mutex()
{
	mutex_init(&ldc_access_mutex);
}
EXPORT_SYMBOL(vio_init_ldc_access_mutex);

void vio_get_ldc_rst_flag(u32 *ldc_rst_flag)
{
	*ldc_rst_flag = ldc_reset_flag;
}
EXPORT_SYMBOL(vio_get_ldc_rst_flag);

void vio_set_ldc_rst_flag(u32 ldc_rst_flag)
{
	ldc_reset_flag = ldc_rst_flag;
}
EXPORT_SYMBOL(vio_set_ldc_rst_flag);

void vio_get_sif_exit_flag(u32 *sif_exit)
{
	*sif_exit = sif_module_exit;
}
EXPORT_SYMBOL(vio_get_sif_exit_flag);

void vio_set_sif_exit_flag(u32 sif_exit)
{
	sif_module_exit = sif_exit;
}
EXPORT_SYMBOL(vio_set_sif_exit_flag);

void vio_rst_mutex_init(void)
{
	mutex_init(&rst_mutex);
}
EXPORT_SYMBOL(vio_rst_mutex_init);

void vio_rst_mutex_lock(void)
{
	mutex_lock(&rst_mutex);
}
EXPORT_SYMBOL(vio_rst_mutex_lock);

void vio_rst_mutex_unlock(void)
{
	mutex_unlock(&rst_mutex);
}
EXPORT_SYMBOL(vio_rst_mutex_unlock);

isp_callback sif_isp_ctx_sync;
EXPORT_SYMBOL(sif_isp_ctx_sync);

void isp_register_callback(isp_callback func)
{
	sif_isp_ctx_sync = func;
}
EXPORT_SYMBOL(isp_register_callback);

iar_get_type_callback iar_get_type;
EXPORT_SYMBOL(iar_get_type);

void iar_register_get_callback(iar_get_type_callback func)
{
       iar_get_type = func;
}
EXPORT_SYMBOL(iar_register_get_callback);

iar_set_addr_callback iar_set_addr;
EXPORT_SYMBOL(iar_set_addr);

void iar_register_set_callback(iar_set_addr_callback func)
{
       iar_set_addr = func;
}
EXPORT_SYMBOL(iar_register_set_callback);

void osd_set_info(int32_t instance, int32_t chn, struct vio_osd_info *info)
{
    osd_info[instance][chn] = info;
}
EXPORT_SYMBOL(osd_set_info);

struct vio_osd_info* osd_get_info(int32_t instance, int32_t chn)
{
    return osd_info[instance][chn];
}
EXPORT_SYMBOL(osd_get_info);

osd_send_frame_callback osd_send_frame;
EXPORT_SYMBOL(osd_send_frame);

void osd_send_callback(osd_send_frame_callback func)
{
       osd_send_frame = func;
}
EXPORT_SYMBOL(osd_send_callback);

osd_get_sta_bin_callback osd_get_sta_bin;
EXPORT_SYMBOL(osd_get_sta_bin);

void osd_get_sta_callback(osd_get_sta_bin_callback func)
{
       osd_get_sta_bin = func;
}
EXPORT_SYMBOL(osd_get_sta_callback);

void vio_irq_affinity_set(int irq, enum MOD_ID id, int suspend,
		int input_online)
{
	if (suspend) {
		irq_set_affinity_hint(irq, NULL);
	} else {
		switch(id) {
		case MOD_IDMA:
			if (nr_cpu_ids == 2)
				irq_set_affinity_hint(irq, get_cpu_mask(1));
			else
				irq_set_affinity_hint(irq, get_cpu_mask(3));
			break;
		case MOD_ISP:
			if (nr_cpu_ids == 2 || input_online)
				irq_set_affinity_hint(irq, get_cpu_mask(1));
			else
				irq_set_affinity_hint(irq, get_cpu_mask(2));
			break;
		default:
			irq_set_affinity_hint(irq, get_cpu_mask(1));
			break;
		}
	}
}
EXPORT_SYMBOL(vio_irq_affinity_set);

int vio_group_task_start(struct vio_group_task *group_task)
{
	int ret = 0;
	char name[30];
	struct sched_param param = {0};

	if(group_task->id == GROUP_ID_SIF_OUT) {
		param.sched_priority = SIF_OUT_TASK_PRIORITY;
	} else if (group_task->id == GROUP_ID_SIF_IN) {
		param.sched_priority = SIF_DDRIN_TASK_PRIORITY;
	} else if (group_task->id == GROUP_ID_IPU) {
		param.sched_priority = IPU_TASK_PRIORITY;
	} else if (group_task->id == GROUP_ID_PYM) {
		param.sched_priority = PYM_TASK_PRIORITY;
	}

	BUG_ON(!group_task);

	if (test_bit(VIO_GTASK_START, &group_task->state))
		goto p_work;

	kthread_init_worker(&group_task->worker);
	snprintf(name, sizeof(name), "vio_gw%d", group_task->id);
	group_task->task = kthread_run(kthread_worker_fn, &group_task->worker, name); /*PRQA S ALL*/
	if (IS_ERR(group_task->task)) {
		vio_err("failed to create buffer task, err(%ld)\n",PTR_ERR(group_task->task));
		ret = (int)PTR_ERR(group_task->task);
		goto p_err;
	}

	ret = sched_setscheduler_nocheck(group_task->task, SCHED_FIFO, &param);
	if (ret) {
		vio_err("sched_setscheduler_nocheck is fail(%d)", ret);
		goto p_err;
	}
#ifdef SET_CPU_AFFINITY
	ret = set_cpus_allowed_ptr(group_task->task, cpumask_of(1));
#endif

	sema_init(&group_task->hw_resource, 1);

	set_bit(VIO_GTASK_START, &group_task->state);
p_work:
	atomic_inc(&group_task->refcount);
p_err:
	return ret;
}
EXPORT_SYMBOL(vio_group_task_start);

int vio_group_task_stop(struct vio_group_task *group_task)
{
	int ret = 0;
	int refcount;

	BUG_ON(!group_task);
	if (!test_bit(VIO_GTASK_START, &group_task->state)) {
		vio_err("gtask(%d) is not started", group_task->id);
		ret = -EINVAL;
		goto p_err;
	}

	if (IS_ERR_OR_NULL(group_task->task)) {
		vio_err("task of group_task(%d) is invalid(%p)", group_task->id, group_task->task);
		ret = -EINVAL;
		goto p_err;
	}

	refcount = atomic_dec_return(&group_task->refcount);
	if (refcount > 0)
		goto p_err;

	vio_dbg("real task stop %s\n", group_task->task->comm); /*PRQA S ALL*/
	set_bit(VIO_GTASK_REQUEST_STOP, &group_task->state);

	if(test_bit(VIO_GTASK_SHOT, &group_task->state)){
		set_bit(VIO_GTASK_SHOT_STOP, &group_task->state);
		up(&group_task->hw_resource);
	}

	kthread_stop(group_task->task);

	clear_bit(VIO_GTASK_SHOT_STOP, &group_task->state);
	clear_bit(VIO_GTASK_REQUEST_STOP, &group_task->state);
	clear_bit(VIO_GTASK_START, &group_task->state);

p_err:
	return ret;

}
EXPORT_SYMBOL(vio_group_task_stop);

void vio_group_start_trigger(struct vio_group *group, struct vio_frame *frame)
{
	struct vio_group_task *group_task;
	bool trigger;
	group_task = group->gtask;
	atomic_inc(&group->rcount);
	trigger = kthread_queue_work(&group_task->worker, &frame->work);
	if (trigger == false)
		vio_err("vio_group_start_trigger failed\n");
}
EXPORT_SYMBOL(vio_group_start_trigger);

void vio_group_start_trigger_mp(struct vio_group *group, struct vio_frame *frame)
{
	struct vio_group_task *group_task;

	group_task = group->gtask;
	atomic_inc(&group->rcount);
	kthread_queue_work(&group_task->worker, frame->mp_work);
}
EXPORT_SYMBOL(vio_group_start_trigger_mp); /*PRQA S ALL*/

void vio_group_insert_work(struct vio_group *group, struct kthread_work *work)
{
	struct vio_group_task *group_task;

	group_task = group->gtask;
	atomic_inc(&group->rcount);
	kthread_queue_work(&group_task->worker, work);
}
EXPORT_SYMBOL(vio_group_insert_work);


int vio_group_init_mp(u32 group_id)
{
	struct vio_group *group = NULL;
	u8 i;

	if (group_id >= GROUP_ID_NUMBER) {
		vio_err("[%s]wrong group_id (%d)\n", __func__, group_id);
		return -EINVAL;
	}

	for (i = 0; i < VIO_MAX_STREAM; i++) {
		group = &iscore.chain[i].group[group_id];
		spin_lock_init(&group->slock);
	}
	return 0;
}
EXPORT_SYMBOL(vio_group_init_mp);

struct vio_group *vio_get_chain_group(int instance, u32 group_id)
{
	struct vio_group *group = NULL;
	struct vio_chain *ischain;

	if (instance < 0 || instance >= VIO_MAX_STREAM) {
		vio_err("can't support instance %d\n", instance);
		return NULL;
	}

	vio_init_chain(instance);

	if (group_id >= GROUP_ID_NUMBER) {
		vio_err("[%s]wrong group id (%d)\n", __func__, group_id);
		return NULL;
	}

	ischain = &iscore.chain[instance];
	group = &ischain->group[group_id];

	return group;
}
EXPORT_SYMBOL(vio_get_chain_group);

void vio_group_init(struct vio_group *group)
{
	int i = 0;

	BUG_ON(!group);

	group->state = 0;
	group->leader = 0;
	group->gtask = NULL;
	group->frame_work = NULL;
	group->next = NULL;
	group->prev = group;
	group->head = group;
	group->frameid.frame_id = 0;
	group->target_sema = 0x3;
	group->sema_flag = 0x00;
	group->group_scenario = -1;
	group->shadow_reuse_check = 0;
	atomic_set(&group->rcount, 0); /*PRQA S ALL*/
	atomic_set(&group->node_refcount, 0); /*PRQA S ALL*/
	atomic_set(&group->work_insert, 0); /*PRQA S ALL*/
	for(i = 0; i < MAX_SUB_DEVICE; i++)
		group->sub_ctx[i] = NULL;

	vio_dbg("%s : %d\n", __func__, group->id); /*PRQA S ALL*/
}
EXPORT_SYMBOL(vio_group_init);

int vio_init_chain(int instance)
{
	struct vio_chain *ischain;
	struct vio_group *group;
	unsigned long flags;
	int i = 0;

	ischain = &iscore.chain[instance];

	for (i = 0; i < GROUP_ID_NUMBER; i++) {
		group = &ischain->group[i];
		spin_lock_irqsave(&group->slock, flags); /*PRQA S ALL*/
		if (!test_bit(VIO_GROUP_INIT, &group->state)) {
			group->chain = ischain;
			group->instance= instance;
			group->id = i;
			vio_group_init(group);
			set_bit(VIO_GROUP_INIT, &group->state);
		}
		spin_unlock_irqrestore(&group->slock, flags); /*PRQA S ALL*/
	}

	return 0;
}
EXPORT_SYMBOL(vio_init_chain);

int vio_bind_chain_groups(struct vio_group *src_group, struct vio_group *dts_group)
{
	struct vio_group *group;

	BUG_ON(!src_group);
	BUG_ON(!dts_group);

	src_group->next = dts_group;
	dts_group->prev = src_group;

	if (!test_bit(VIO_GROUP_DMA_OUTPUT, &dts_group->state))
		dts_group->leader = false;

	group = src_group;
	while (group != group->prev) {
		group = group->prev;
	}
	dts_group->head = group;

	if (src_group->id == GROUP_ID_IPU && src_group->get_timestamps)
		dts_group->get_timestamps = true;

	return 0;
}
EXPORT_SYMBOL(vio_bind_chain_groups);

/*
 * VIO modules in pipeline will run into this when init
 * VIN/VPS will initialized randomly,
 * but modules in VIN/VPS is sequential
 */
// PRQA S ALL ++
void vio_bind_group_done(int instance)
{
	int i = 0;
	char stream[64] = {'\0'};
	int offset = 0;
	struct vio_chain *ischain;
	struct vio_group *group;
	struct vio_group *ipu_group, *pym_group;
	int every_group_input[GROUP_ID_NUMBER] = {-1, -1, -1, -1};

	ischain = &iscore.chain[instance];
	for (i = 0; i < GROUP_ID_NUMBER; i++) {
		group = &ischain->group[i];
		if (test_bit(VIO_GROUP_DMA_OUTPUT, &group->state)) {
			group->get_timestamps = true;
			break;
		}
	}

	/*
	 * whenerver vio_bind_group_done called,
	 * all the groups in pipeline will be rechecked
	 * and group bind relationship will be rewrite
	 * 1. if module is DDR input, this module will be group leader
	 * 2. if module is ONLINE input,
	 *		this module is connected to previous group
	 */
	for (i = 0; i < GROUP_ID_NUMBER; i++) {
		group = &ischain->group[i];
		group->next = NULL;
		if (test_bit(VIO_GROUP_DMA_INPUT, &group->state)) {
			group->leader = true;
			snprintf(&stream[offset], sizeof(stream) - offset,
					"=>G%d", group->id);
			offset = (int)strlen(stream);
			every_group_input[i] = 1;
		} else if (test_bit(VIO_GROUP_OTF_INPUT, &group->state)) {
			vio_bind_chain_groups(&ischain->group[i - 1], group);
			snprintf(&stream[offset], sizeof(stream) - offset,
					"->G%d", group->id);
			offset = (int)strlen(stream);
			every_group_input[i] = 0;
		}
	}

	/*
	 * this logic is for frameid judge
	 */
	ipu_group = &iscore.chain[instance].group[GROUP_ID_IPU];
	pym_group = &iscore.chain[instance].group[GROUP_ID_PYM];

	if (every_group_input[GROUP_ID_SIF_IN] == 1
		 && every_group_input[GROUP_ID_IPU] == 0 &&
		 every_group_input[GROUP_ID_PYM] == 1) {
		// sif-offline-isp-online-ipu-offline-pym G0=>G1->G2=>G3
		ipu_group->group_scenario = VIO_GROUP_SIF_OFF_ISP_ON_IPU_OFF_PYM;
		pym_group->group_scenario = VIO_GROUP_SIF_OFF_ISP_ON_IPU_OFF_PYM;
	}  else if (every_group_input[GROUP_ID_SIF_IN] == 1
		 && every_group_input[GROUP_ID_IPU] == 0 &&
		 every_group_input[GROUP_ID_PYM] == 0) {
		// sif-offline-isp-online-ipu-online-pym G0=>G1->G2->G3
		ipu_group->group_scenario = VIO_GROUP_SIF_OFF_ISP_ON_IPU_ON_PYM;
		pym_group->group_scenario = VIO_GROUP_SIF_OFF_ISP_ON_IPU_ON_PYM;
	}

	for (i = 0; i < GROUP_ID_NUMBER; i++) {
		group = &ischain->group[i];
		if (group->leader) {
			/* make sure pym node is group leader in such case:
			 * 1.ioctl EOS is called more than one time
			 * 2.multi-process sharing data
			 */
			if (test_bit(VIO_GROUP_OTF_INPUT, &group->state)
				&& (i >= GROUP_ID_IPU)
				&& test_bit(VIO_GROUP_DMA_OUTPUT, &group->state)) {
				group->head = group;
				group->next = NULL;
				continue;
			}
			/*
			 * clear leader flag make sure only one leader
			 */
			while (group->next) {
				group->next->leader = false;
				group = group->next;
			}
			break;
		} else if (i >= GROUP_ID_IPU) {
			if (test_bit(VIO_GROUP_DMA_OUTPUT, &group->state)) {
				group->leader = true;
				group->head = group;
				group->next = NULL;
				vio_info("change group %d to leader\n", i);
			}
		}
	}

	group = &ischain->group[GROUP_ID_SIF_IN];

	/* ddr-isp-otf-ipu */
	if (group->next &&
		test_bit(VIO_GROUP_DMA_OUTPUT, &group->next->state)) {
		group->sema_flag = 0x00;
		vio_info("[S%d]G1->G2 case\n", group->instance);
	}

	/* ddr-isp-otf-ipu-otf-pym */
	if (group->next && group->next->next &&
		test_bit(VIO_GROUP_DMA_OUTPUT, &group->next->state)) {
		group->target_sema = 0x7;
		group->sema_flag = 0x00;
		vio_info("[S%d]G1->G2->G3 case\n", group->instance);
	}

	vio_info("Stream%d path: G0%s\n", group->instance, stream);
}
// PRQA S ALL --
EXPORT_SYMBOL(vio_bind_group_done);

void vio_get_sif_frame_info(u32 instance, struct vio_frame_id *frame_info)
{
	frame_info->frame_id = sif_frame_info[instance].frame_id;
	frame_info->timestamps = sif_frame_info[instance].timestamps;
	frame_info->tv = sif_frame_info[instance].tv;
}
EXPORT_SYMBOL(vio_get_sif_frame_info);

void vio_get_ipu_frame_info(u32 instance, struct vio_frame_id *frame_info)
{
	frame_info->frame_id = ipu_frame_info[instance].frame_id;
	frame_info->timestamps = ipu_frame_info[instance].timestamps;
	frame_info->tv = ipu_frame_info[instance].tv;
}
EXPORT_SYMBOL(vio_get_ipu_frame_info);

void vio_get_frame_id(struct vio_group *group)
{
	struct vio_chain *ischain;
	struct vio_group *sif_group;

	ischain = group->chain;
	sif_group = &ischain->group[GROUP_ID_SIF_OUT];

	memcpy(&group->frameid, &sif_group->frameid, sizeof(struct frame_id));
}
EXPORT_SYMBOL(vio_get_frame_id);

void vio_get_sif_frame_id(struct vio_group *group)
{
	struct vio_chain *ischain;
	struct vio_group *sif_group;

	ischain = group->chain;
	sif_group = &ischain->group[GROUP_ID_SIF_IN];

	memcpy(&group->frameid, &sif_group->frameid, sizeof(struct frame_id));
}

EXPORT_SYMBOL(vio_get_sif_frame_id);

void vio_get_ipu_frame_id(struct vio_group *group)
{
	struct vio_chain *ischain;
	struct vio_group *ipu_group;

	ischain = group->chain;
	ipu_group = &ischain->group[GROUP_ID_IPU];

	memcpy(&group->frameid, &ipu_group->frameid, sizeof(struct frame_id));
}
EXPORT_SYMBOL(vio_get_ipu_frame_id);

int vio_check_all_online_state(struct vio_group *group)
{
	struct vio_chain *ischain;
	struct vio_group *sif_group;
	struct vio_group *ipu_group;

	ischain = group->chain;
	sif_group = &ischain->group[GROUP_ID_SIF_OUT];
	ipu_group = &ischain->group[GROUP_ID_IPU];

	if (test_bit(VIO_GROUP_OTF_OUTPUT, &sif_group->state) &&
			test_bit(VIO_GROUP_OTF_INPUT, &ipu_group->state) &&
			test_bit(VIO_GROUP_OTF_INPUT, &group->state))
		return 1;
	else
		return 0;
}
EXPORT_SYMBOL(vio_check_all_online_state);

void vio_reset_module(u32 module)
{
	u32 cfg = 0;
	u32 cnt = VIO_RETRY_100;
	u32 value = 0;
	u32 bit = 0;
	u32 reset = 0;

	if (module == GROUP_ID_IPU) {
		bit = IPU0_IDLE;
		reset = IPU0_RST;
	} else if (module == GROUP_ID_PYM) {
		bit = PYM_IDLE;
		reset = PYM_RST;
	}

	cfg = ips_get_bus_ctrl() | bit;
	ips_set_bus_ctrl(cfg);

	while(1) {
		value = ips_get_bus_status();
		if (value & bit << 16)
			break;

		msleep(1);
		cnt--;
		if (cnt == 0) {
			vio_info("%s timeout\n", __func__);
			break;
		}
	}

	ips_set_module_reset(reset);
	if (module == GROUP_ID_PYM)
		ips_set_module_reset(IPU_PYM_RST);

	cfg = ips_get_bus_ctrl() & ~bit;
	ips_set_bus_ctrl(cfg);
}
EXPORT_SYMBOL(vio_reset_module);

/**
 * @brief: Notify the leader that this frame is complete
 * @param group: cur module
 */
void vio_group_done(struct vio_group *group)
{
	struct vio_group_task *group_task;
	struct vio_group *group_leader;
	unsigned long flags;
	u32 sema_flag;

	group_leader = group->head;
	group_task = group_leader->gtask;
	/* all online scenario, ipu will enter this condition */
	if (group_task == NULL) {
		vio_dbg("WARN: group%d's group leader(%d) task is NULL\n",
				group->id, group_leader->id);
		return;
	}

	spin_lock_irqsave(&group_leader->slock, flags); /*PRQA S ALL*/
	if (group->next && group->head != group &&
		test_bit(VIO_GROUP_DMA_OUTPUT, &group->state)) {
		group_leader->sema_flag |= 1 << 2;
	}

	if (group->next == NULL) {
		group_leader->sema_flag |= 1 << 0;
	}

	if (group->head == group) {
		group_leader->sema_flag |= 1 << 1;
	}

	sema_flag = group_leader->sema_flag;

	if (group_leader->sema_flag == group_leader->target_sema) {
		group_leader->sema_flag = 0;
		up(&group_task->hw_resource);
		vio_dbg("group%d,leader%d,sema_flag=%d,target_sema=%d", /*PRQA S ALL*/
			group->id, group_leader->id, sema_flag,
			group_leader->target_sema);
		vio_dbg("[S%d][G%d]up hw_resource G%d\n", group->instance, group->id, /*PRQA S ALL*/
		group_leader->id);
	} else {
		vio_dbg("group%d,leader%d,sema_flag=%d,target_sema=%d", /*PRQA S ALL*/
			group->id, group_leader->id, sema_flag,
			group_leader->target_sema);
	}
	spin_unlock_irqrestore(&group_leader->slock, flags); /*PRQA S ALL*/
}
EXPORT_SYMBOL(vio_group_done);

void vio_dwe_clk_enable(void)
{
	struct vio_core *core;
	bool enable = true;

	core = &iscore;
	if (atomic_read(&core->rsccount) == 0) { /*PRQA S ALL*/
		if (sif_mclk_freq)
			vio_set_clk_rate("sif_mclk", sif_mclk_freq);
		ips_set_clk_ctrl(DWE0_CLOCK_GATE, enable);
		vio_info("%s done", __func__);
	}
	atomic_inc(&core->rsccount);
}
EXPORT_SYMBOL(vio_dwe_clk_enable);

void vio_dwe_clk_disable(void)
{
	struct vio_core *core;
	bool enable = false;

	core = &iscore;
	if (atomic_dec_return(&core->rsccount) == 0) {
		ips_set_clk_ctrl(DWE0_CLOCK_GATE, enable);
		vio_info("%s done", __func__);
	}
}
EXPORT_SYMBOL(vio_dwe_clk_disable);

void vio_gdc_clk_enable(u32 hw_id)
{
	struct vio_core *core;
	bool enable = true;

	core = &iscore;
	if (hw_id == 0) {
		if (atomic_read(&core->gdc0_rsccount) == 0) { /*PRQA S ALL*/
			if (sif_mclk_freq)
				vio_set_clk_rate("sif_mclk", sif_mclk_freq);
			ips_set_clk_ctrl(GDC0_CLOCK_GATE, enable);
			vio_info("%s hw_id:%d done", __func__, hw_id);
		}
		atomic_inc(&core->gdc0_rsccount);
	} else if (hw_id == 1) {
		if (atomic_read(&core->gdc1_rsccount) == 0) { /*PRQA S ALL*/
                        if (sif_mclk_freq)
                                vio_set_clk_rate("sif_mclk", sif_mclk_freq);
                        ips_set_clk_ctrl(GDC1_CLOCK_GATE, enable);
                        vio_info("%s hw_id:%d done", __func__, hw_id);
                }
		atomic_inc(&core->gdc1_rsccount);
	} else {
		vio_err("%s error hw_id:%d\n", __func__, hw_id);
	}
}
EXPORT_SYMBOL(vio_gdc_clk_enable);

void vio_gdc_clk_disable(u32 hw_id)
{
	struct vio_core *core;
	bool enable = false;

	core = &iscore;
	if (hw_id == 0) {
		if (atomic_dec_return(&core->gdc0_rsccount) == 0) {
			ips_set_clk_ctrl(GDC0_CLOCK_GATE, enable);
			vio_info("%s hw_id:%d done", __func__, hw_id);
		}
	} else if (hw_id == 1) {
		if (atomic_dec_return(&core->gdc1_rsccount) == 0) {
                        ips_set_clk_ctrl(GDC1_CLOCK_GATE, enable);
                        vio_info("%s hw_id:%d done", __func__, hw_id);
                }
	} else {
		vio_err("%s error hw_id:%d\n", __func__, hw_id);
	}
}
EXPORT_SYMBOL(vio_gdc_clk_disable);

void* vio_get_stat_info_ptr(u32 instance)
{
	return &iscore.chain[instance];
}
EXPORT_SYMBOL(vio_get_stat_info_ptr);

void voi_set_stat_info_update(s32 update)
{
	stat_info_update = update;
}
EXPORT_SYMBOL(voi_set_stat_info_update);

void vio_set_stat_info(u32 instance, u32 stat_type, u32 event, u32 frameid,
	u32 addr, u32 *queued_count)
{
	struct vio_chain *chain;
	struct statinfo *stat;
	int i = 0;

	if (stat_info_update) {
		chain = &iscore.chain[instance];
		if (chain->statinfoidx[stat_type] >= MAX_DELAY_FRAMES)
			chain->statinfoidx[stat_type] = 0;
		stat = &chain->statinfo[chain->statinfoidx[stat_type]][stat_type];
		stat->framid = frameid;
		do_gettimeofday(&stat->g_tv);
		stat->event = event;
		stat->addr  = addr;
		if (queued_count != NULL) {
			for (i = 0; i < NR_FRAME_STATE; i++)
				stat->queued_count[i] = (u8)queued_count[i];
		}
		chain->statinfoidx[stat_type]++;
	}
}
EXPORT_SYMBOL(vio_set_stat_info);

void vio_clear_stat_info(u32 instance)
{
	struct vio_chain *chain;
	chain = &iscore.chain[instance];
	memset(chain->statinfo, 0, sizeof(chain->statinfo));
	memset(chain->statinfoidx, 0, sizeof(chain->statinfoidx));
}
EXPORT_SYMBOL(vio_clear_stat_info);

int vio_print_delay(s32 instance, u8* buf, u32 size)
{
	struct vio_chain *chain;
	struct statinfo *stat;
	int i = 0;
	int modidx = 0;
	u32 offset = 0;
	int len = 0;
	int idx = 0;
	int showidx = 0;
	char *event[] = {
		"event_none",
		"event_sif_cap_fs",
		"event_sif_cap_fe",
		"event_sif_in_fs",
		"event_sif_in_fe",
		"event_isp_fs",
		"event_isp_fe",
		"event_ipu_fs",
		"event_ipu_us_fe",
		"event_ipu_ds0_fe",
		"event_ipu_ds1_fe",
		"event_ipu_ds2_fe",
		"event_ipu_ds3_fe",
		"event_ipu_ds4_fe",
		"event_pym_fs",
		"event_pym_fe",
		"event_gdc_fs",
		"event_gdc_fe"
	};
	int events[VIO_MOD_NUM] = {
		event_sif_cap_fs,
		event_isp_fs,
		event_ipu_fs,
		event_pym_fs,
		event_gdc_fs
	};
	int evente[VIO_MOD_NUM] = {
		event_sif_in_fe,
		event_isp_fe,
		event_ipu_ds4_fe,
		event_pym_fe,
		event_gdc_fe
	};
	chain = &iscore.chain[instance];
	len = snprintf(&buf[offset], size - offset,
			"*******pipe %d vio info:************\n", instance);
	offset += len;
	for (modidx = 0; modidx < VIO_MOD_NUM; modidx++) {
		showidx = 0;
		idx = (int)chain->statinfoidx[modidx];
		for (i = 0; i < MAX_DELAY_FRAMES; i++) {
			stat = chain->statinfo[(idx + i)%MAX_DELAY_FRAMES];
			if (stat[modidx].event >= events[modidx] &&
				stat[modidx].event <= evente[modidx]) {
				len = snprintf(&buf[offset], size - offset,
					"[F%07d] %s (FS %ld.%06ld)\n",
					stat[modidx].framid, event[stat[modidx].event],
					stat[modidx].g_tv.tv_sec, stat[modidx].g_tv.tv_usec);
				offset += len;
				showidx++;
				if ((modidx == (int)IPU_MOD) && (showidx >= 60)) {
					break;
				} else if (showidx >= 10) {
					break;
				}
			}
		}
		len = snprintf(&buf[offset], size - offset, "\n");
		offset += len;
	}

	return offset;
}
EXPORT_SYMBOL(vio_print_delay);

void vio_print_stat_info(u32 instance)
{
	struct vio_chain *chain;
	struct statinfo *stat;

	chain = &iscore.chain[instance];
	stat = chain->statinfo[chain->statinfoidx[0]];
	vio_info("[F%d]sif_cap(FS %ld.%06ld|FE %ld.%06ld)\n",
		stat[SIF_CAP_FS].framid,
		stat[SIF_CAP_FS].g_tv.tv_sec, stat[SIF_CAP_FS].g_tv.tv_usec,
		stat[SIF_CAP_FE].g_tv.tv_sec, stat[SIF_CAP_FE].g_tv.tv_usec);

	vio_info("[F%d]sif_in(FS %ld.%06ld|FE %ld.%06ld)\n",
		stat[SIF_IN_FS].framid,
		stat[SIF_IN_FS].g_tv.tv_sec, stat[SIF_IN_FS].g_tv.tv_usec,
		stat[SIF_IN_FE].g_tv.tv_sec, stat[SIF_IN_FE].g_tv.tv_usec);

	vio_info("[F%d]ipu(FS %ld.%06ld|FE US %ld.%06ld|ds0 %ld.%06ld|", /*PRQA S ALL*/
		stat[IPU_FS].framid,
		stat[IPU_FS].g_tv.tv_sec, stat[IPU_FS].g_tv.tv_usec,
		stat[IPU_US_FE].g_tv.tv_sec, stat[IPU_US_FE].g_tv.tv_usec,
		stat[IPU_DS0_FE].g_tv.tv_sec, stat[IPU_DS0_FE].g_tv.tv_usec);

	vio_info("ds1 %ld.%06ld|ds2 %ld.%06ld|ds3 %ld.%06ld|ds4 %ld.%06ld)\n", /*PRQA S ALL*/
		stat[IPU_DS1_FE].g_tv.tv_sec, stat[IPU_DS1_FE].g_tv.tv_usec,
		stat[IPU_DS2_FE].g_tv.tv_sec, stat[IPU_DS2_FE].g_tv.tv_usec,
		stat[IPU_DS3_FE].g_tv.tv_sec, stat[IPU_DS3_FE].g_tv.tv_usec,
		stat[IPU_DS4_FE].g_tv.tv_sec, stat[IPU_DS4_FE].g_tv.tv_usec);

	vio_info("[F%d]pym(FS %ld.%06ld|FE %ld.%06ld)\n", /*PRQA S ALL*/
		stat[PYM_FS].framid,
		stat[PYM_FS].g_tv.tv_sec, stat[PYM_FS].g_tv.tv_usec,
		stat[PYM_FE].g_tv.tv_sec, stat[PYM_FE].g_tv.tv_usec);

	vio_info("[F%d]gdc(FS %ld.%06ld|FE %ld.%06ld)\n", /*PRQA S ALL*/
		stat[GDC_FS].framid,
		stat[GDC_FS].g_tv.tv_sec, stat[GDC_FS].g_tv.tv_usec,
		stat[GDC_FE].g_tv.tv_sec, stat[GDC_FE].g_tv.tv_usec);
}
EXPORT_SYMBOL(vio_print_stat_info);
// PRQA S ALL ++
void vio_print_stack_by_name(char *name)
{
#define TRACE_DEPTH 20

#ifdef CONFIG_STACK_TRACER
    struct task_struct *p = NULL;
    unsigned long backtrace[TRACE_DEPTH];
    struct stack_trace trace;

    memset(&trace, 0, sizeof(trace));
    memset(backtrace, 0, sizeof(unsigned long) * TRACE_DEPTH);
    trace.max_entries = TRACE_DEPTH;
    trace.entries = backtrace;

    for_each_process(p) {
        if (!strcmp(p->comm, name)) {
            pr_info("find %s.\n", name);
            save_stack_trace_tsk(p, &trace);
            print_stack_trace(&trace, 0);
        }
    }
#endif
}
EXPORT_SYMBOL(vio_print_stack_by_name);
// PRQA S ALL --
