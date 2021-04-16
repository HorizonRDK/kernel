#ifndef __LINUX_AVB_H__
#define __LINUX_AVB_H__

#include <linux/tsn.h>

#define AVB_TEST_HEADER_MAGIC		0xdeadbeef
#define AVB_VIDEO_XRES			640
#define AVB_VIDEO_YRES			480

struct avb_test_header {
	u32 id;
	u32 size;
	u8 payload[0];
} __packed;

static inline size_t avb_hdr_size(void)
{
	return sizeof(struct avb_test_header);
}

static inline void avb_assemble_header(struct avtpdu_header *header,
		size_t bytes)
{
	struct avb_test_header *dh;

	dh = (struct avb_test_header *)&header->data;
	dh->id = AVB_TEST_HEADER_MAGIC;
	dh->size = bytes;
}

static inline int avb_validate_header(struct avtpdu_header *header)
{
	struct avb_test_header *dh;

	dh = (struct avb_test_header *)&header->data;
	if (dh->id != AVB_TEST_HEADER_MAGIC)
		return -EINVAL;
	return 0;
}

static inline void *avb_get_payload_data(struct avtpdu_header *header)
{
	struct avb_test_header *dh;

	dh = (struct avb_test_header *)&header->data;
	return &dh->payload;
}

#endif /* __LINUX_AVB_H__ */
