#ifndef _BIF_BASE_H_
#define _BIF_BASE_H_
#include <linux/interrupt.h>

enum BUFF_ID {
	BUFF_BASE,
	BUFF_ETH,
	BUFF_SMD,
	BUFF_MAX,
};

#define IRQ_QUEUE_SIZE 16
/*must be 512 padding for BIFSD access*/
struct bif_base_info {
	unsigned char magic[4];
	unsigned char irq[IRQ_QUEUE_SIZE];
	unsigned char send_irq_tail;
	unsigned char read_irq_head;
	unsigned char register_irqs;
	unsigned char buffer_count;
	unsigned int address_list[BUFF_MAX];
//	irq_handler_t irq_func[BUFF_MAX];
	unsigned short offset_list[BUFF_MAX];
	unsigned short next_offset;
};

int bif_register_irq(enum BUFF_ID buffer_id, irq_handler_t handler);
int bif_send_irq(int irq);

int bif_register_address(enum BUFF_ID buffer_id, void *address);
void *bif_query_address(enum BUFF_ID buffer_id);
void *bif_query_address_wait(enum BUFF_ID buffer_id);

void *bif_alloc_base(enum BUFF_ID buffer_id, int size);
void *bif_query_otherbase_wait(enum BUFF_ID buffer_id);
void *bif_query_otherbase(enum BUFF_ID buffer_id);
#ifndef CONFIG_HOBOT_BIF_AP
void *bif_alloc_cp(enum BUFF_ID buffer_id, int size, unsigned long *phyaddr);
#endif
#endif
