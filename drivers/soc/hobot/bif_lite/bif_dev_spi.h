#ifndef _BIF_DEV_H_
#define _BIF_DEV_H_

static int frame_len_max_g = 256 * 1024;
static int frag_len_max_g = 1024;

#define RING_INFO_ALIGN            (512)
#define FRAME_LEN_MAX              (frame_len_max_g)
#define FRAG_LEN_MAX               (frag_len_max_g)
#define FRAG_NUM                   ((FRAME_LEN_MAX / FRAG_LEN_MAX) * 4)
#define CP2AP_RING_BUFFER_LEN      (FRAG_NUM * FRAG_LEN_MAX)
#define AP2CP_RING_BUFFER_LEN      (FRAG_NUM * FRAG_LEN_MAX)
#define FRAME_CACHE_MAX            (50)

#define CP2AP_RING_AP_INFO_LEN (ALIGN(sizeof(struct bif_rx_ring_info), \
RING_INFO_ALIGN))
#define AP2CP_RING_AP_INFO_LEN (ALIGN(sizeof(struct bif_tx_ring_info), \
RING_INFO_ALIGN))
#define CP2AP_RING_CP_INFO_LEN (ALIGN(sizeof(struct bif_tx_ring_info), \
RING_INFO_ALIGN))
#define AP2CP_RING_CP_INFO_LEN (ALIGN(sizeof(struct bif_rx_ring_info), \
RING_INFO_ALIGN))

#define CP2AP_RING_AP_INFO_OFFSET 0

#define AP2CP_RING_AP_INFO_OFFSET \
	(CP2AP_RING_AP_INFO_OFFSET \
	+ CP2AP_RING_AP_INFO_LEN)

#define CP2AP_RING_CP_INFO_OFFSET \
	(AP2CP_RING_AP_INFO_OFFSET \
	+ AP2CP_RING_AP_INFO_LEN)

#define AP2CP_RING_CP_INFO_OFFSET \
	(CP2AP_RING_CP_INFO_OFFSET \
	+ CP2AP_RING_CP_INFO_LEN)

#define CP2AP_BUFFER_OFFSET  \
	(AP2CP_RING_CP_INFO_OFFSET \
	+ AP2CP_RING_CP_INFO_LEN)

#define AP2CP_BUFFER_OFFSET  \
	(CP2AP_BUFFER_OFFSET \
	+ CP2AP_RING_BUFFER_LEN)

#define TOTAL_MEM_SIZE \
	(CP2AP_RING_AP_INFO_LEN \
	+ AP2CP_RING_AP_INFO_LEN \
	+ CP2AP_RING_CP_INFO_LEN \
	+ AP2CP_RING_CP_INFO_LEN \
	+ CP2AP_RING_BUFFER_LEN \
	+ AP2CP_RING_BUFFER_LEN)

#ifndef CONFIG_HOBOT_BIF_AP
#define RX_LOCAL_INFO_OFFSET     AP2CP_RING_CP_INFO_OFFSET
#define RX_REMOTE_INFO_OFFSET    AP2CP_RING_AP_INFO_OFFSET
#define TX_LOCAL_INFO_OFFSET     CP2AP_RING_CP_INFO_OFFSET
#define TX_REMOTE_INFO_OFFSET    CP2AP_RING_AP_INFO_OFFSET
#define RX_BUFFER_OFFSET         AP2CP_BUFFER_OFFSET
#define TX_BUFFER_OFFSET         CP2AP_BUFFER_OFFSET
#else
#define RX_LOCAL_INFO_OFFSET     CP2AP_RING_AP_INFO_OFFSET
#define RX_REMOTE_INFO_OFFSET    CP2AP_RING_CP_INFO_OFFSET
#define TX_LOCAL_INFO_OFFSET     AP2CP_RING_AP_INFO_OFFSET
#define TX_REMOTE_INFO_OFFSET    AP2CP_RING_CP_INFO_OFFSET
#define RX_BUFFER_OFFSET         CP2AP_BUFFER_OFFSET
#define TX_BUFFER_OFFSET         AP2CP_BUFFER_OFFSET
#endif

#endif
