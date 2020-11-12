#ifndef __ISP_CTX_SAVE_H__
#define __ISP_CTX_SAVE_H__

#include "acamera_firmware_config.h"

// for ctx save
//(ACAMERA_ISP1_BASE_ADDR - ACAMERA_DECOMPANDER0_MEM_BASE_ADDR + ACAMERA_ISP1_SIZE)

// for stats
//#define MAX_AWB_ZONES	(33 * 33)

#define CFG_NODE_SIZE	(0x18e88 - 0xab6c + 0x4000) //74524
#define CTX_SIZE	CFG_NODE_SIZE
#define CTX_OFFSET      0xab6c

#define AWB_NODE_SIZE	(33 * 33 * 8) //8712
#define AE_NODE_SIZE	(ISP_FULL_HISTOGRAM_SIZE * 4) //4096
#define AF_NODE_SIZE	(33 * 33 * 8) //8712
#define AE_5BIN_NODE_SIZE	(33 * 33 * 8) //8712
#define LUMVAR_NODE_SIZE (32 * 16 * 4) // 512*4

#define PER_ZONE_NODES	6

#define CFG_SIZE_IN_ONE_ZONE    (CFG_NODE_SIZE * PER_ZONE_NODES)
#define AE_SIZE_IN_ONE_ZONE     (AE_NODE_SIZE * PER_ZONE_NODES)
#define AWB_SIZE_IN_ONE_ZONE    (AWB_NODE_SIZE * PER_ZONE_NODES)
#define AF_SIZE_IN_ONE_ZONE    (AF_NODE_SIZE * PER_ZONE_NODES)
#define AE_5BIN_SIZE_IN_ONE_ZONE (AE_5BIN_NODE_SIZE * PER_ZONE_NODES)
#define ONE_ZONE_SIZE	(PER_ZONE_NODES * (CFG_NODE_SIZE + AWB_NODE_SIZE + AE_NODE_SIZE + AF_NODE_SIZE + AE_5BIN_NODE_SIZE + LUMVAR_NODE_SIZE)) //87332x6=523992
#define TOTAL_MEM_SIZE	(ONE_ZONE_SIZE * FIRMWARE_CONTEXT_NUMBER) //523992x4=2095968=2M

typedef enum {
	FREEQ = 0,
	DONEQ,
	Q_MAX,
} isp_ctx_queue_type_e;

typedef enum {
	ISP_CTX = 0,
	ISP_AE,
	ISP_AWB,
	ISP_AF,
	ISP_AE_5BIN,
	ISP_LUMVAR,
	TYPE_MAX,
} isp_info_type_e;

// isp ctx get
typedef struct {
        uint8_t ctx_id;
        uint8_t idx;
	uint8_t type;
        uint32_t frame_id;
        uint64_t timestamps;
        uint32_t crc16;
		int32_t time_out;
} isp_ctx_r_t;

// isp ctx set
typedef struct {
        uint8_t ctx_id;
        uint32_t crc16;
        void *ptr;
} isp_ctx_w_t;

// context queue node
typedef struct {
	struct list_head node;
	void *base;
	isp_ctx_r_t ctx;
} isp_ctx_node_t;

extern isp_ctx_node_t *isp_ctx_get_node(int ctx_id, isp_info_type_e it, isp_ctx_queue_type_e qt);
extern isp_ctx_node_t *isp_ctx_get(int ctx_id, isp_info_type_e it, int32_t timeout);
extern void isp_ctx_put_node(int ctx_id, isp_ctx_node_t *cn, isp_info_type_e it, isp_ctx_queue_type_e qt);
extern void isp_ctx_put(int ctx_id, isp_info_type_e type, uint8_t idx);
extern void isp_ctx_done_queue_clear(int ctx_id);
extern int isp_ctx_queue_init(void);
extern int system_chardev_lock(void);
extern void system_chardev_unlock(void);
extern int isp_irq_wait_for_completion(int ctx_id, uint8_t irq_type, unsigned long timeout);
extern void isp_irq_completion(int ctx_id, uint8_t irq_type);

#endif
