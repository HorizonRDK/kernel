#ifndef __IPU_SLOT_H__
#define __IPU_SLOT_H__

#include <linux/types.h>
#include <linux/list.h>
#include <linux/time.h>
#include "ipu_common.h"

enum {
	FREE_SLOT_LIST,
	BUSY_SLOT_LIST,
	DONE_SLOT_LIST,
	SLOT_LIST_NUM,
};

/* 64M per slot
 * 1080p 28M for crop, scale, pym_ds, 36M for pym_us
 */
// #define IPU_MAX_SLOT				8
// #define IPU_SLOT_SIZE				0x1000000
#define IPU_GET_SLOT(id, base)		((base) + (id) * IPU_SLOT_SIZE)

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

#define CHANNEL_MAX		2
#define CAM_0			0
#define CAM_1			1
#define CAM_FB_0		0xFB0
#define CAM_FB_1		0xFB1

typedef struct {
	uint8_t slot_id;
	uint8_t slot_flag;          /// busy, free, done
	uint8_t ipu_flag;           /// start, done, pym start, done
	uint8_t cnn_flag;           /// start, done
	uint16_t cf_id;
	uint16_t sf_id;
	int64_t cf_timestamp;
	int64_t sf_timestamp;
	uint64_t base;
	union {
		slot_ddr_info_dual_t dual_ddr_info;
		slot_ddr_info_t ddr_info;
	};
	int cam_id[CHANNEL_MAX];
} info_h_t;

typedef struct {
	struct list_head list;
	uint8_t  slot_get;
	uint8_t  slot_cnt;
	info_h_t info_h;
	struct timeval tv;
} ipu_slot_h_t;

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
        info_h_t info;
        int status;
        int time;
}vio_ipu_info_t;

int slot_left_num(int type);
int8_t init_ipu_slot(uint64_t base, slot_ddr_info_t *data);
int8_t ipu_clean_slot(slot_ddr_info_t *data);
ipu_slot_h_t *ipu_get_free_slot(void);
ipu_slot_h_t *ipu_get_done_slot(void);
ipu_slot_h_t* ipu_get_done_slot_from_tail(void);
ipu_slot_h_t *slot_free_to_busy(void);
ipu_slot_h_t *ipu_read_done_slot(void);
ipu_slot_h_t *slot_busy_to_done(void);
ipu_slot_h_t *slot_done_to_free(slot_ddr_info_t *data);
ipu_slot_h_t *slot_busy_to_free(slot_ddr_info_t *data);
int insert_slot_to_free(int slot_id);
bool is_slot_busy_empty(void);
bool is_slot_free_empty(void);
bool is_slot_done_empty(void);
void dump_slot_state(void);
int8_t ipu_slot_recfg(slot_ddr_info_t *data);
#endif
