
#ifndef _BIF_ETHER_H_
#define _BIF_ETHER_H_

#define ETHER_QUERE_SIZE	(10)
struct bif_ether_info {
	unsigned char send_tail;
	unsigned char recv_head;
	unsigned char queue_full;
	unsigned short elen[ETHER_QUERE_SIZE];
};

#endif /* _BIF_ETHER_H_ */
