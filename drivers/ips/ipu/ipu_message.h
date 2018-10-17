#ifndef __IPU_MESSAGE_H__
#define __IPU_MESSAGE_H__

typedef struct {
	uint32_t offset;
	uint32_t size;
} msg_ddr_t;

typedef struct {
	msg_ddr_t crop;
	msg_ddr_t scale;
	msg_ddr_t ds[24];
	msg_ddr_t us[6];
} msg_ddr_info;

typedef struct {
	uint16_t slot_id;
	uint64_t slot_phys;
	msg_ddr_info *addr;
	uint16_t reserved;
} ipu_msg_t;

int send_to_usr(char *pbuf, uint16_t len);
#endif
