#ifndef _BIF_BASE_H_
#define _BIF_BASE_H_
#include <linux/interrupt.h>

enum BUFF_ID {
	BUFF_BASE,
	BUFF_ETH,	//bifeth0
	BUFF_SMD,	//biftty0
	BUFF_SIO,	//biftty1
	BUFF_LITE,	//biflite0
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
	unsigned short offset_list[BUFF_MAX];
	unsigned short next_offset;
};

#define BIF_MT_WB 0x1
#define BIF_MT_UC 0x2
#define BIF_MT_WC 0x3
#define BIF_MT_WT 0x4

//cp,ap register irq
int bif_register_irq(enum BUFF_ID buffer_id, irq_handler_t handler);
int bif_send_irq(int irq);
//cp register self phyaddr to base
int bif_register_address(enum BUFF_ID buffer_id, void *address);
//ap query cp register phyaddr from base
void *bif_query_address(enum BUFF_ID buffer_id);
void *bif_query_address_wait(enum BUFF_ID buffer_id);
//cp,ap alloc buff id control memory from base memory
void *bif_alloc_base(enum BUFF_ID buffer_id, int size);
//cp,ap query other sides control memory from base
void *bif_query_otherbase_wait(enum BUFF_ID buffer_id);
void *bif_query_otherbase(enum BUFF_ID buffer_id);
//cp alloc alloc buff id buff memory from base memory, return phyaddr
void *bif_alloc_cp(enum BUFF_ID buffer_id, int size, ulong *phyaddr);
// ap sync cp side info structure
//int bifbase_sync_cp(void *p);
int bif_sync_base(void);

#endif
