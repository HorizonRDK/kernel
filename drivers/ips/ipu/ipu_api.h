#ifndef __IPU_API_H__
#define __IPU_API_H__

typedef enum {
	IPUC_INIT = 0,
	IPUC_GET_IMG = 1,
	IPUC_CNN_DONE = 2,
	IPUC_GET_DONE_INFO = 3,
} ipu_cmd_e;

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
	struct nlmsghdr hdr;
	uint16_t slot_id;
	uint64_t slot_phys;
	msg_ddr_info *addr;
	uint16_t reserved;
} msg_t;

int ipu_open(char *args);
int ipu_close(int fd);
int inform_ipu_cnn_done(int fd);
int inform_ipu_get_imgae(int fd, char *buf);

#endif
