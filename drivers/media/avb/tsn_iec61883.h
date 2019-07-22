#ifndef TSN_IEC61883_H
#define TSN_IEC61883_H
#include <linux/tsn.h>



struct iec61883_tag {
#if defined(__LITTLE_ENDIAN_BITFIELD)
	u8 tag:2;
	u8 channel:6;
	u8 tcode:4;
	u8 sy:4;
#elif defined(__BIG_ENDIAN_BITFIELD)
	u8 channel:6;
	u8 tag:2;
	u8 sy:4;
	u8 sy:4;
#else
#error "Unknown Endianness, cannot determine bitfield ordering"
#endif
} __packed;






struct iec61883_audio_header {
#if defined(__LITTLE_ENDIAN_BITFIELD)
	u8 sid:6;
	u8 cip_1:2;

	u8 dbs:8;

	u8 rsv:2;		/* reserved */
	u8 sph:1;
	u8 qpc:3;
	u8 fn:2;

	u8 dbc;

	u8 fmt:6;
	u8 cip_2:2;
#elif defined(__BIG_ENDIAN_BITFIELD)
	u8 cip_1:2;
	u8 sid:6;

	u8 dbs:8;

	u8 fn:2;
	u8 qpc:3;
	u8 sph:1;
	u8 rsv:2;		/* reserved */

	u8 dbc;

	u8 cip_2:2;
	u8 fmt:6;
#else
#error "Unknown Endianness, cannot determine bitfield ordering"
#endif
	u8 fdf;
	u16 syt;
	u8 payload[0];
} __packed;





static inline size_t _iec61883_hdr_len(void)
{
	return sizeof(struct iec61883_audio_header);
}





static inline int _iec61883_hdr_verify(struct avtpdu_header *hdr)
{
	struct iec61883_audio_header *dh;
	struct iec61883_tag *psh;

	if (hdr->subtype != TSN_61883_IIDC)
		return -EINVAL;
	dh  = (struct iec61883_audio_header *)&hdr->data;
	psh = (struct iec61883_tag *)&hdr->psh;

	/* Verify 61883 header */
	if (psh->tag != 1 || psh->channel != 31 ||
		psh->tcode != 0xA || psh->sy != 0)
		return -EINVAL;

	/* check flags that should be static from frame to frame */
	if (dh->cip_1 != 0 || dh->sid != 0x3f || dh->qpc != 0 || dh->fn != 0 ||
		dh->sph != 0 || dh->cip_2 != 2)
		return -EINVAL;

	if (dh->dbs != ntohs(hdr->sd_len)*2 || dh->dbc != hdr->seqnr)
		return -EINVAL;

	return 0;
}




static inline void _iec61883_hdr_assemble(struct avtpdu_header *hdr,
					  size_t bytes)
{
	struct iec61883_tag *psh;
	struct iec61883_audio_header *dh;

	if (bytes > 0x7f)
		pr_warn("%s: hdr->dbs will overflow, malformed frame will be the result\n",
			__func__);


	hdr->subtype = TSN_61883_IIDC;

	/* IIDC 61883 header */
	psh = (struct iec61883_tag *)&hdr->psh;
	psh->tag = 1;
	psh->channel = 31;      /* 0x1f */
	psh->tcode = 0xA;
	psh->sy = 0;

	dh = (struct iec61883_audio_header *)&hdr->data;
	dh->cip_1 = 0;
	dh->sid = 63;           /* 0x3f */
	dh->dbs = (u8)(bytes*2); /* number of quadlets of data in AVTPDU */
	dh->qpc = 0;
	dh->fn = 0;
	dh->sph = 0;
	dh->dbc = hdr->seqnr;
	dh->cip_2 = 2;

	/*
	 * FMT (Format ID): same as specified in iec 61883-1:2003
	 *
	 * For IEC 61883-6, it shall be 0x10 (16) to define Audio and
	 * Music data
	 */
	dh->fmt = 0x10;

	/* FIXME: find value
	 * Could be sampling-freq, but 8 bits give 0 - 65kHz sampling.
	 */
	dh->fdf = 0;

	dh->syt = 0xFFFF;
}




static inline void *_iec61883_payload(struct avtpdu_header *hdr)
{
	struct iec61883_audio_header *dh = (struct iec61883_audio_header *)&hdr->data;
	/* TODO: add some basic checks before returning payload ? */
	return &dh->payload;
}

#endif	/* TSN_IEC61883_H */
