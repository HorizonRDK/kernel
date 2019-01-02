#ifndef __IPU_SLOT_DUAL_H__
#define __IPU_SLOT_DUAL_H__

#include <linux/types.h>
#include <linux/list.h>

/* 64M per slot
 * 1080p 28M for crop, scale, pym_ds, 36M for pym_us
 */
#define IPU_MAX_SLOT_DUAL			4
#define IPU_SLOT_DAUL_SIZE			0x8000000
#define IPU_GET_DUAL_SLOT(id, base)		((base) + (id) * IPU_SLOT_DAUL_SIZE)
#define IPU_GET_FST_PYM_OF_SLOT(id, base)	   ((base) + (id) * IPU_SLOT_DAUL_SIZE)
#define IPU_GET_SEC_PYM_OF_SLOT(id, base)	   ((base) + (id) * IPU_SLOT_DAUL_SIZE + IPU_SLOT_DAUL_SIZE/2)

typedef struct {
	uint32_t y_offset;
	uint32_t c_offset;
	uint16_t y_width;
	uint16_t y_height;
	uint16_t y_stride;	/// contain blanking data
	uint16_t c_width;
	uint16_t c_height;
	uint16_t c_stride;	/// contain blanking data
} slot_ddr_t;

//dual mode below
typedef struct {
	slot_ddr_t crop;
	slot_ddr_t scale;
	slot_ddr_t ds_1st[24];
	slot_ddr_t ds_2nd[24];
	slot_ddr_t us_1st[6];
	slot_ddr_t us_2nd[6];
} slot_ddr_info_dual_t;

typedef struct {
	uint8_t slot_id;
	uint8_t slot_flag;	/// busy, free, done
	uint8_t ipu_flag;	/// start, done, pym start, done
	uint8_t cnn_flag;	/// start, done
	uint16_t cf_id;
	uint16_t sf_id;
	uint64_t base;
	slot_ddr_info_dual_t ddr_info;
} info_dual_h_t;

typedef struct {
	struct list_head list;
	uint8_t slot_get;
	uint8_t slot_cnt;
	info_dual_h_t info_h;
} ipu_slot_dual_h_t;

typedef enum {
	ELIST_EMPTY = 1,
} slot_err_e;

typedef enum {
	SLOT_FREE = 0,
	SLOT_BUSY = 1,
	SLOT_DONE = 2,
} slot_flag_e;

typedef enum {
	reserved = 1,
} slot_cnn_flag_e;

typedef struct {
	struct list_head queue;
	int qlen;
	spinlock_t lock;
} slot_queue_t;

//dual below
ipu_slot_dual_h_t *dequeue_slot(slot_queue_t * slot_queue);
void enqueue_slot(slot_queue_t * slot_queue, ipu_slot_dual_h_t * slot_h);
ipu_slot_dual_h_t *get_first_of_queue(slot_queue_t * slot_queue);
int8_t init_ipu_slot_dual(uint64_t base, slot_ddr_info_dual_t * data);
int8_t ipu_clean_slot_queue(void);

#endif
