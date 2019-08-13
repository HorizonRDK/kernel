#ifndef __IPU_PYM_H__
#define __IPU_PYM_H__

#include <linux/kfifo.h>
#include "ipu_ddr.h"

#define IPU_PYM_BUF_LEN 6

typedef enum {
	PYM_INLINE,
	PYM_OFFLINE,
	PYM_PROCESS_TYPE_END,
} ipu_pym_process_type;

typedef enum {
	PYM_SLOT_SINGLE,
	PYM_SLOT_MULT,
	PYM_SLOT_TYPE_END,
} ipu_pym_slot_type;

struct pym_slot_info {
	ipu_pym_slot_type slot_type;
	ipu_pym_process_type process_type;
	/* if pym slot type is mult, use pym_left_num*/
	int pym_left_num;
	union {
		struct src_img_info_t src_img_info;
		struct mult_img_info_t mult_img_info;
	} img_info;
	ipu_cfg_t *cfg;
};

struct ipu_pym {
	struct task_struct *process_task;
	wait_queue_head_t process_wait;
	wait_queue_head_t done_wait[PYM_PROCESS_TYPE_END];
	DECLARE_KFIFO_PTR(pym_slots, struct pym_slot_info);
	DECLARE_KFIFO_PTR(done_inline_pym_slots, struct pym_slot_info);
	DECLARE_KFIFO_PTR(done_offline_pym_slots, struct pym_slot_info);
	/* now pym can only processing one frame a time */
	struct pym_slot_info *pyming_slot_info;
	spinlock_t slock;
	int processing;
	int inited;
};


extern struct ipu_pym *g_ipu_pym;

int ipu_pym_init(void);
void ipu_pym_exit(void);
void ipu_pym_clear(void);
int ipu_pym_wait_process_done(void *data, int len,
		ipu_pym_process_type process_type, int timeout);
int ipu_pym_process_done(void);
int ipu_pym_to_process(void *img_info, ipu_cfg_t *ipu_cfg,
		ipu_pym_slot_type type, ipu_pym_process_type process_type);

#endif
