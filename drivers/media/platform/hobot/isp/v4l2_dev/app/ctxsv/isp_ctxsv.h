#ifndef __ISP_CTX_SAVE_H__
#define __ISP_CTX_SAVE_H__

// for ctx save
//(ACAMERA_ISP1_BASE_ADDR - ACAMERA_DECOMPANDER0_MEM_BASE_ADDR + ACAMERA_ISP1_SIZE)
#define PER_ZONE_NODES	6
#define ZONE_MAX	4
#define NODE_SIZE	(0x18e88 - 0xab6c + 0x4000)
#define MEM_SIZE	(NODE_SIZE * PER_ZONE_NODES * ZONE_MAX)
#define CTX_SIZE	NODE_SIZE
#define CTX_OFFSET      0xab6c

typedef enum {
	FREEQ = 0,
	BUSYQ,
	DONEQ,
	BUTT,
} isp_ctx_queue_type_e;

// isp ctx get
typedef struct {
        uint8_t ctx_id;
        uint8_t idx;
        uint32_t frame_id;
        uint64_t timestamps;
        uint32_t crc16;
} isp_ctx_r_t;

// isp ctx set
typedef struct {
        uint8_t ctx_id;
        uint32_t crc16;
        void *ptr;
} isp_ctx_w_t;

// queue node
typedef struct {
	struct list_head node;
	void *base;
	isp_ctx_r_t ctx;
} isp_ctx_node_t;

extern isp_ctx_node_t *isp_ctx_get_node(int ctx_id, isp_ctx_queue_type_e type);
extern void isp_ctx_put_node(int ctx_id, isp_ctx_node_t *cn, isp_ctx_queue_type_e type);
extern void isp_ctx_put(int ctx_id, uint8_t idx);
extern int isp_ctx_queue_init(void);
#endif
