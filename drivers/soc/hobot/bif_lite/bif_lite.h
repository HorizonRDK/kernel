#ifndef _BIF_LITE_H_
#define _BIF_LITE_H_
#include "bif_platform.h"

//#define FRAME_LEN_MAX (1024*16)
#define FRAME_LEN_MAX (1024*128)

// do not bigger than 1024
#define BUFFER_LEN  512
//#define BUFFER_NUM  ((FRAME_LEN_MAX/BUFFER_LEN)*4)
#define BUFFER_NUM  ((FRAME_LEN_MAX/BUFFER_LEN)*2)
#define CP_TO_AP_BUFFER_LEN   (BUFFER_NUM*BUFFER_LEN)
#define AP_TO_CP_BUFFER_LEN   (BUFFER_NUM*BUFFER_LEN)
#define BIF_CACHE_NUM BUFFER_NUM
#define FRAME_CACHE_MAX 4
#define BIFSPI_LEN_ALIGN (16)

struct bif_frame_cache {
	unsigned short  framelen;
	struct list_head frame_cache_list;
	unsigned char framecache[];
};

struct frag_info {
	unsigned int start : 1;
	unsigned int end : 1;
	unsigned int id : 12;
	unsigned int len : 12;
	unsigned int crc12 : 12;
};
#define FRAG_INFO_LEN (sizeof(struct frag_info))
#define VALID_BUFFER_LEN (BUFFER_LEN - FRAG_INFO_LEN)

struct bif_tx_ring_info {
	//struct frag_info  fragment_info[BUFFER_NUM];
	int  send_tail;
	//unsigned short  check_sum;
};

struct bif_rx_ring_info {
	 int  recv_head;
};

struct bif_rx_cache {
	unsigned short  datalen;
	unsigned char datacache[BUFFER_LEN];
};

#define CP_TO_AP_RING_AP_INFO_LEN (ALIGN(sizeof(struct bif_rx_ring_info), \
BUFFER_LEN))
#define AP_TO_CP_RING_AP_INFO_LEN (ALIGN(sizeof(struct bif_tx_ring_info), \
BUFFER_LEN))
#define CP_TO_AP_RING_CP_INFO_LEN (ALIGN(sizeof(struct bif_tx_ring_info), \
BUFFER_LEN))
#define AP_TO_CP_RING_CP_INFO_LEN (ALIGN(sizeof(struct bif_rx_ring_info), \
BUFFER_LEN))

#define CP_TO_AP_RING_AP_INFO_OFFSET 0

#define AP_TO_CP_RING_AP_INFO_OFFSET \
	(CP_TO_AP_RING_AP_INFO_OFFSET \
	+CP_TO_AP_RING_AP_INFO_LEN)

#define CP_TO_AP_RING_CP_INFO_OFFSET \
	(AP_TO_CP_RING_AP_INFO_OFFSET \
	+AP_TO_CP_RING_AP_INFO_LEN)

#define AP_TO_CP_RING_CP_INFO_OFFSET \
	(CP_TO_AP_RING_CP_INFO_OFFSET \
	+CP_TO_AP_RING_CP_INFO_LEN)

#define CP_TO_AP_BUFFER_OFFSET  \
	(AP_TO_CP_RING_CP_INFO_OFFSET \
	+AP_TO_CP_RING_CP_INFO_LEN)

#define AP_TO_CP_BUFFER_OFFSET  \
	(CP_TO_AP_BUFFER_OFFSET \
	+CP_TO_AP_BUFFER_LEN)

#define TOTAL_MEM_SIZE \
	(CP_TO_AP_RING_AP_INFO_LEN \
	+AP_TO_CP_RING_AP_INFO_LEN \
	+CP_TO_AP_RING_CP_INFO_LEN \
	+AP_TO_CP_RING_CP_INFO_LEN \
	+CP_TO_AP_BUFFER_LEN \
	+AP_TO_CP_BUFFER_LEN)

/*
 * memory details:
 * ============================ CP_TO_AP_RING_AP_INFO_OFFSET
 * bif_rx_ring_info_t  for CP_TO_AP ,   the info is changed by AP
 * ( CP_TO_AP_RING_AP_INFO_LEN bytes)
 * ============================ AP_TO_CP_RING_AP_INFO_OFFSET
 * bif_tx_ring_info_t  for AP_TO_CP,    the info is changed by AP
 * ( AP_TO_CP_RING_AP_INFO_LEN bytes)
 * ============================ CP_TO_AP_RING_CP_INFO_OFFSET
 * bif_tx_ring_info_t for CP_TO_AP ,    the info is changed by CP
 * ( CP_TO_AP_RING_CP_INFO_LEN bytes)
 * ============================ AP_TO_CP_RING_CP_INFO_OFFSET
 * bif_rx_ring_info_t for AP_TO_CP ,    the info is changed by CP
 * ( AP_TO_CP_RING_CP_INFO_LEN bytes)
 * ============================ CP_TO_AP_BUFFER_OFFSET
 * data buffers for CP_TO_AP
 * (CP_TO_AP_BUFFER_LEN bytes )
 * ============================ AP_TO_CP_BUFFER_OFFSET
 * data buffers for AP_TO_CP
 * (AP_TO_CP_BUFFER_LEN bytes )
 * ============================
 */

#endif
