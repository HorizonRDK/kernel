#ifndef __IPU_SLOT_H__
#define __IPU_SLOT_H__

#include <linux/types.h>
#include <linux/list.h>

#define IPU_MAX_SLOT                8
/* 64M per slot
 * 1080p 28M for crop, scale, pym_ds, 36M for pym_us
 */
#define IPU_SLOT_SIZE               0x4000000
#define IPU_GET_SLOT(id, base)      ((base) + (id) * IPU_SLOT_SIZE)

typedef struct {
	uint32_t offset;
	uint32_t size;
} slot_ddr_t;

typedef struct {
	slot_ddr_t crop;
	slot_ddr_t scale;
	slot_ddr_t ds[24];
	slot_ddr_t us[6];
} slot_ddr_info_t;

typedef struct {
	struct list_head list;
	uint8_t slot_id;
	uint8_t slot_flag;
	uint8_t ipu_flag;
	uint8_t cnn_flag;
	slot_ddr_info_t ddr_info;
} ipu_slot_h_t;

typedef struct {
	uint8_t slot_id;
	uint8_t slot_flag;
	uint8_t ipu_flag;
	uint8_t cnn_flag;
	uint64_t base;
	slot_ddr_info_t ddr_info;
} info_h_t;

typedef enum {
	ELIST_EMPTY = 1,
} slot_err_e;

typedef enum {
	SLOT_FREE = 1,
	SLOT_BUSY = 2,
	SLOT_DONE = 3,
} slot_flag_e;

typedef enum {
	reserved = 1,
} slot_cnn_flag_e;

int8_t init_ipu_slot(uint64_t base, slot_ddr_info_t * data);
ipu_slot_h_t *ipu_get_free_slot(void);
ipu_slot_h_t *ipu_get_busy_slot(void);
ipu_slot_h_t *ipu_get_done_slot(void);
int8_t slot_to_busy_list(ipu_slot_h_t * slot_h);
int8_t slot_to_done_list(ipu_slot_h_t * slot_h);
int8_t slot_to_free_list(ipu_slot_h_t * slot_h);

#endif
