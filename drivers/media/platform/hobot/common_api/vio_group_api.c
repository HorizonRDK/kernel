/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2019 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

#include <linux/delay.h>
#include <linux/sched.h>
#include <uapi/linux/sched/types.h>
#include "vio_group_api.h"

static struct vio_core iscore;

struct vio_frame_id  sif_frame_info;
struct vio_frame_id  ipu_frame_info;

EXPORT_SYMBOL(sif_frame_info);
EXPORT_SYMBOL(ipu_frame_info);

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

int vio_group_task_start(struct vio_group_task *group_task)
{
	int ret = 0;
	char name[30];
	struct sched_param param = { .sched_priority = MAX_RT_PRIO - 1 };

	BUG_ON(!group_task);

	if (test_bit(VIO_GTASK_START, &group_task->state))
		goto p_work;

	kthread_init_worker(&group_task->worker);
	snprintf(name, sizeof(name), "vio_gw%d", group_task->id);
	group_task->task = kthread_run(kthread_worker_fn, &group_task->worker, name);
	if (IS_ERR(group_task->task)) {
		vio_err("failed to create buffer task, err(%ld)\n",PTR_ERR(group_task->task));
		ret = PTR_ERR(group_task->task);
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
EXPORT_SYMBOL(vio_group_start_trigger_mp);

int vio_group_init_mp(u32 group_id)
{
	struct vio_group *group = NULL;
	u8 i;

	if (group_id > GROUP_ID_NUMBER) {
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

	if (group_id > GROUP_ID_NUMBER) {
		vio_err("[%s]wrong group id (%d)\n", __func__, group_id);
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
	atomic_set(&group->rcount, 0);
	atomic_set(&group->node_refcount, 0);
	for(i = 0; i < MAX_SUB_DEVICE; i++)
		group->sub_ctx[i] = NULL;

	vio_dbg("%s : %d\n", __func__, group->id);
}
EXPORT_SYMBOL(vio_group_init);

int vio_init_chain(int instance)
{
	struct vio_chain *ischain;
	struct vio_group *group;
	int i = 0;

	ischain = &iscore.chain[instance];

	for (i = 0; i < GROUP_ID_NUMBER; i++) {
		group = &ischain->group[i];
		spin_lock(&group->slock);
		if (!test_bit(VIO_GROUP_INIT, &group->state)) {
			group->chain = ischain;
			group->instance= instance;
			group->id = i;
			vio_group_init(group);
			set_bit(VIO_GROUP_INIT, &group->state);
		}
		spin_unlock(&group->slock);
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

	// since this func run in
	// the dts_group is always OTF_INPUT
	// it's not the leader
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
void vio_bind_group_done(int instance)
{
	int i = 0;
	char stream[64] = {'\0'};
	int offset = 0;
	struct vio_chain *ischain;
	struct vio_group *group;
	struct vio_group *ipu_group;
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
			offset = strlen(stream);
			every_group_input[i] = 1;
		} else if (test_bit(VIO_GROUP_OTF_INPUT, &group->state)) {
			vio_bind_chain_groups(&ischain->group[i - 1], group);
			snprintf(&stream[offset], sizeof(stream) - offset,
					"->G%d", group->id);
			offset = strlen(stream);
			every_group_input[i] = 0;
		}
	}

	/*
	 * this logic is for frameid judge
	 */
	ipu_group = &iscore.chain[instance].group[GROUP_ID_IPU];
	//  offline to ipu
	if ((every_group_input[GROUP_ID_SIF_IN] == 1)
		 && (every_group_input[GROUP_ID_IPU] == 1 ||
		 every_group_input[GROUP_ID_IPU] == 0)) {
		// sif->offline->isp->offline->ipu  G0=>G1=>G2
		// sif-offline-isp-online-ipu G0=>G1->G2
		ipu_group->group_scenario = 2;
	} else if (every_group_input[GROUP_ID_SIF_IN] == -1
		 && every_group_input[GROUP_ID_IPU] == 1) {
		// sif->offline->ipu or sif-online-isp-offline-ipu
		// G0=>G2 or G0->G2=>G3
		ipu_group->group_scenario = 1;
	} else if (every_group_input[GROUP_ID_SIF_IN] == -1
		 && every_group_input[GROUP_ID_IPU] == 0) {
		// sif->online->ipu or sif-online-isp-online-ipu
		// G0->G2
		ipu_group->group_scenario = 0;
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
			break;
		} else if (i >= GROUP_ID_IPU) {
			if (test_bit(VIO_GROUP_DMA_OUTPUT, &group->state)) {
				group->leader = true;
				group->head = group;
				group->next = NULL;
			}
		}
	}

	group = &ischain->group[GROUP_ID_SIF_IN];
	if (group->next && group->next->next &&
		test_bit(VIO_GROUP_DMA_OUTPUT, &group->next->state)) {
		group->target_sema = 0x7;
		vio_info("G1->G2->G3 case\n");
	}

	vio_info("Stream%d path: G0%s\n", group->instance, stream);
}
EXPORT_SYMBOL(vio_bind_group_done);

void vio_get_sif_frame_info(struct vio_frame_id *frame_info)
{
	frame_info->frame_id = sif_frame_info.frame_id;
	frame_info->timestamps = sif_frame_info.timestamps;
}
EXPORT_SYMBOL(vio_get_sif_frame_info);

void vio_get_ipu_frame_info(struct vio_frame_id *frame_info)
{
	frame_info->frame_id = ipu_frame_info.frame_id;
	frame_info->timestamps = ipu_frame_info.timestamps;
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

void vio_reset_module(u32 module)
{
	u32 cfg = 0;
	u32 cnt = 20;
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

		msleep(5);
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

void vio_group_done(struct vio_group *group)
{
	struct vio_group_task *group_task;
	struct vio_group *group_leader;

	group_leader = group->head;
	group_task = group_leader->gtask;

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

	vio_dbg("group%d,leader%d,sema_flag=%d,target_sema=%d",
		group->id, group_leader->id, group_leader->sema_flag, group_leader->target_sema);
	if(group_leader->sema_flag == group_leader->target_sema) {
		up(&group_task->hw_resource);
		group_leader->sema_flag = 0;
		vio_dbg("[S%d][G%d]up hw_resource G%d\n", group->instance, group->id,
		group_leader->id);
	}
}
EXPORT_SYMBOL(vio_group_done);

void vio_dwe_clk_enable(void)
{
	struct vio_core *core;
	bool enable = true;

	core = &iscore;
	if (atomic_read(&core->rsccount) == 0) {
		if (sif_mclk_freq)
			vio_set_clk_rate("sif_mclk", sif_mclk_freq);
		ips_set_clk_ctrl(GDC0_CLOCK_GATE, enable);
		ips_set_clk_ctrl(GDC1_CLOCK_GATE, enable);
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
		ips_set_clk_ctrl(GDC0_CLOCK_GATE, enable);
		ips_set_clk_ctrl(GDC1_CLOCK_GATE, enable);
		ips_set_clk_ctrl(DWE0_CLOCK_GATE, enable);
		vio_info("%s done", __func__);
	}
}
EXPORT_SYMBOL(vio_dwe_clk_disable);

void vio_set_stat_info(u32 instance, u32 stat_type, u16 frameid)
{
	struct vio_chain *chain;
	struct statinfo *stat;

	chain = &iscore.chain[instance];
	if (chain->statinfoidx < 0)
		chain->statinfoidx = 0;
	else if (chain->statinfoidx >= MAX_DELAY_FRAMES)
		chain->statinfoidx = 0;
	stat = &chain->statinfo[chain->statinfoidx][stat_type];
	stat->framid = frameid;
	do_gettimeofday(&stat->g_tv);
}
EXPORT_SYMBOL(vio_set_stat_info);

void vio_clear_stat_info(u32 instance)
{
	struct vio_chain *chain;
	chain = &iscore.chain[instance];
	memset(chain->statinfo, 0, sizeof(chain->statinfo));
	chain->statinfoidx = 0;
}
EXPORT_SYMBOL(vio_clear_stat_info);

void vio_exchanage_stat_info(u32 instance)
{
	struct vio_chain *chain;

	chain = &iscore.chain[instance];
	chain->statinfoidx++;
	if (chain->statinfoidx >= MAX_DELAY_FRAMES)
		chain->statinfoidx = 0;
	memset(&chain->statinfo[chain->statinfoidx], 0, sizeof(struct statinfo));
}
EXPORT_SYMBOL(vio_exchanage_stat_info);

int vio_print_delay(s32 instance, s8* buf, u32 size)
{
	struct vio_chain *chain;
	struct statinfo *stat;
	int i = 0;
	u32 offset = 0;
	int len = 0;
	int idx = 0;

	chain = &iscore.chain[instance];
	idx = chain->statinfoidx;
	len = snprintf(&buf[offset], size - offset,
			"*******pipe %d vio info:************\n", instance);
	offset += len;
	for (i = 1; i < MAX_DELAY_FRAMES; i++) {
		stat = chain->statinfo[(idx + i)%MAX_DELAY_FRAMES];
		len = snprintf(&buf[offset], size - offset,
			"*******frame %d vio info:******\n", i);
		offset += len;
		len = snprintf(&buf[offset], size - offset,
			"[F%07d]sif_cap(FS %ld.%06ld)",
			stat[SIF_CAP_FS].framid,
			stat[SIF_CAP_FS].g_tv.tv_sec, stat[SIF_CAP_FS].g_tv.tv_usec);
		offset += len;
		if (stat[SIF_CAP_FS].framid == stat[SIF_CAP_FE].framid) {
			len = snprintf(&buf[offset], size - offset,
				" sif_cap(FE %ld.%06ld) sif use time: %ldus\n",
				stat[SIF_CAP_FE].g_tv.tv_sec,
				stat[SIF_CAP_FE].g_tv.tv_usec,
				(stat[SIF_CAP_FE].g_tv.tv_sec - stat[SIF_CAP_FS].g_tv.tv_sec) *
				1000000 + stat[SIF_CAP_FE].g_tv.tv_usec -
				stat[SIF_CAP_FS].g_tv.tv_usec);
			offset += len;
		} else {
			len = snprintf(&buf[offset], size - offset,
				"\n[F%07d]sif_cap(FE %ld.%06ld)\n",
				stat[SIF_CAP_FE].framid,
				stat[SIF_CAP_FE].g_tv.tv_sec, stat[SIF_CAP_FE].g_tv.tv_usec);
			offset += len;
		}
		len = snprintf(&buf[offset], size - offset,
			"[F%07d] sif_in(FS %ld.%06ld)\n",
			stat[SIF_IN_FS].framid,
			stat[SIF_IN_FS].g_tv.tv_sec, stat[SIF_IN_FS].g_tv.tv_usec);
		offset += len;

		len = snprintf(&buf[offset], size - offset,
			"[F%07d] sif_in(FE %ld.%06ld)\n",
			stat[SIF_IN_FE].framid,
			stat[SIF_IN_FE].g_tv.tv_sec, stat[SIF_IN_FE].g_tv.tv_usec);
		offset += len;

		len = snprintf(&buf[offset], size - offset,
			"[F%07d]    isp(FS %ld.%06ld)\n",
			stat[ISP_FS].framid,
			stat[ISP_FS].g_tv.tv_sec, stat[ISP_FS].g_tv.tv_usec);
		offset += len;

		len = snprintf(&buf[offset], size - offset,
			"[F%07d]    isp(FE %ld.%06ld)\n",
			stat[ISP_FE].framid,
			stat[ISP_FE].g_tv.tv_sec, stat[ISP_FE].g_tv.tv_usec);
		offset += len;

		len = snprintf(&buf[offset], size - offset,
			"[F%07d]    ipu(FS %ld.%06ld)\n",
			stat[IPU_FS].framid,
			stat[IPU_FS].g_tv.tv_sec, stat[IPU_FS].g_tv.tv_usec);
		offset += len;

		len = snprintf(&buf[offset], size - offset,
			"[F%07d]    ipu(FE US %ld.%06ld|ds0 %ld.%06ld|",
			stat[IPU_FS].framid,
			stat[IPU_US_FE].g_tv.tv_sec, stat[IPU_US_FE].g_tv.tv_usec,
			stat[IPU_DS0_FE].g_tv.tv_sec, stat[IPU_DS0_FE].g_tv.tv_usec);
		offset += len;

		len = snprintf(&buf[offset], size - offset,
			"ds1 %ld.%06ld|ds2 %ld.%06ld|ds3 %ld.%06ld|ds4 %ld.%06ld)\n",
			stat[IPU_DS1_FE].g_tv.tv_sec, stat[IPU_DS1_FE].g_tv.tv_usec,
			stat[IPU_DS2_FE].g_tv.tv_sec, stat[IPU_DS2_FE].g_tv.tv_usec,
			stat[IPU_DS3_FE].g_tv.tv_sec, stat[IPU_DS3_FE].g_tv.tv_usec,
			stat[IPU_DS4_FE].g_tv.tv_sec, stat[IPU_DS4_FE].g_tv.tv_usec);
		offset += len;

		len = snprintf(&buf[offset], size - offset,
			"[F%07d]    pym(FS %ld.%06ld)\n",
			stat[PYM_FS].framid,
			stat[PYM_FS].g_tv.tv_sec, stat[PYM_FS].g_tv.tv_usec);
		offset += len;

		len = snprintf(&buf[offset], size - offset,
			"[F%07d]    pym(FE %ld.%06ld)\n",
			stat[PYM_FE].framid,
			stat[PYM_FE].g_tv.tv_sec, stat[PYM_FE].g_tv.tv_usec);
		offset += len;

		len = snprintf(&buf[offset], size - offset,
			"[F%07d]    gdc(FS %ld.%06ld)\n",
			stat[GDC_FS].framid,
			stat[GDC_FS].g_tv.tv_sec, stat[GDC_FS].g_tv.tv_usec);
		offset += len;

		len = snprintf(&buf[offset], size - offset,
			"[F%07d]    gdc(FE %ld.%06ld)\n",
			stat[GDC_FE].framid,
			stat[GDC_FE].g_tv.tv_sec, stat[GDC_FE].g_tv.tv_usec);
		offset += len;
	}
	len = snprintf(&buf[offset], size - offset, "\n");
	offset += len;

	return offset;
}
EXPORT_SYMBOL(vio_print_delay);

void vio_print_stat_info(u32 instance)
{
	struct vio_chain *chain;
	struct statinfo *stat;

	chain = &iscore.chain[instance];
	stat = chain->statinfo[chain->statinfoidx];
	vio_info("[F%d]sif_cap(FS %ld.%06ld|FE %ld.%06ld)\n",
		stat[SIF_CAP_FS].framid,
		stat[SIF_CAP_FS].g_tv.tv_sec, stat[SIF_CAP_FS].g_tv.tv_usec,
		stat[SIF_CAP_FE].g_tv.tv_sec, stat[SIF_CAP_FE].g_tv.tv_usec);

	vio_info("[F%d]sif_in(FS %ld.%06ld|FE %ld.%06ld)\n",
		stat[SIF_IN_FS].framid,
		stat[SIF_IN_FS].g_tv.tv_sec, stat[SIF_IN_FS].g_tv.tv_usec,
		stat[SIF_IN_FE].g_tv.tv_sec, stat[SIF_IN_FE].g_tv.tv_usec);

	vio_info("[F%d]ipu(FS %ld.%06ld|FE US %ld.%06ld|ds0 %ld.%06ld|",
		stat[IPU_FS].framid,
		stat[IPU_FS].g_tv.tv_sec, stat[IPU_FS].g_tv.tv_usec,
		stat[IPU_US_FE].g_tv.tv_sec, stat[IPU_US_FE].g_tv.tv_usec,
		stat[IPU_DS0_FE].g_tv.tv_sec, stat[IPU_DS0_FE].g_tv.tv_usec);

	vio_info("ds1 %ld.%06ld|ds2 %ld.%06ld|ds3 %ld.%06ld|ds4 %ld.%06ld)\n",
		stat[IPU_DS1_FE].g_tv.tv_sec, stat[IPU_DS1_FE].g_tv.tv_usec,
		stat[IPU_DS2_FE].g_tv.tv_sec, stat[IPU_DS2_FE].g_tv.tv_usec,
		stat[IPU_DS3_FE].g_tv.tv_sec, stat[IPU_DS3_FE].g_tv.tv_usec,
		stat[IPU_DS4_FE].g_tv.tv_sec, stat[IPU_DS4_FE].g_tv.tv_usec);

	vio_info("[F%d]pym(FS %ld.%06ld|FE %ld.%06ld)\n",
		stat[PYM_FS].framid,
		stat[PYM_FS].g_tv.tv_sec, stat[PYM_FS].g_tv.tv_usec,
		stat[PYM_FE].g_tv.tv_sec, stat[PYM_FE].g_tv.tv_usec);

	vio_info("[F%d]gdc(FS %ld.%06ld|FE %ld.%06ld)\n",
		stat[GDC_FS].framid,
		stat[GDC_FS].g_tv.tv_sec, stat[GDC_FS].g_tv.tv_usec,
		stat[GDC_FE].g_tv.tv_sec, stat[GDC_FE].g_tv.tv_usec);
}
EXPORT_SYMBOL(vio_print_stat_info);
