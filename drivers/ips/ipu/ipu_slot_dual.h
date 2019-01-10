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
	uint16_t y_stride;          /// contain blanking data
	uint16_t c_width;
	uint16_t c_height;
	uint16_t c_stride;          /// contain blanking data
} slot_ddr_t;

typedef struct {
	slot_ddr_t crop;
	slot_ddr_t scale;
	slot_ddr_t ds[24];
	slot_ddr_t us[6];
} slot_ddr_info_t;

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
	uint8_t slot_flag;          /// busy, free, done
	uint8_t ipu_flag;           /// start, done, pym start, done
	uint8_t cnn_flag;           /// start, done
	uint16_t cf_id;
	uint16_t sf_id;
	uint64_t base;
	union {
		slot_ddr_info_dual_t dual_ddr_info;
		slot_ddr_info_t ddr_info;
	};
} info_dual_h_t;

typedef struct {
	struct list_head list;
	uint8_t  slot_get;
	uint8_t  slot_cnt;
	info_dual_h_t info_h;
} ipu_slot_dual_h_t;


typedef enum {
	ELIST_EMPTY = 1,
} slot_err_e;

typedef enum {
	SLOT_FREE,
	SLOT_RECVING,
	SLOT_PYM_1ST,
	SLOT_PYM_2ND,
	SLOT_DONE,
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
ipu_slot_dual_h_t* dequeue_slot(slot_queue_t *slot_queue);
void enqueue_slot(slot_queue_t *slot_queue, ipu_slot_dual_h_t *slot_h);
int insert_dual_slot_to_free(int slot_id);
ipu_slot_dual_h_t* ipu_get_pym_done_slot(void);
bool ipu_is_pym_done_empty(void);
ipu_slot_dual_h_t* get_cur_pym_slot(void);
ipu_slot_dual_h_t* get_last_pym_slot(void);
ipu_slot_dual_h_t* recv_slot_free_to_busy(void);
ipu_slot_dual_h_t* recv_slot_busy_to_done(void);
ipu_slot_dual_h_t* pym_slot_free_to_busy(void);
ipu_slot_dual_h_t* pym_slot_busy_to_done(void);
int8_t init_ipu_slot_dual(uint64_t base, slot_ddr_info_dual_t *data);
int8_t ipu_clean_slot_queue(void);
bool ipu_is_pym_busy_empty(void);
ipu_slot_dual_h_t* pym_slot_busy_to_free(void);

#endif
