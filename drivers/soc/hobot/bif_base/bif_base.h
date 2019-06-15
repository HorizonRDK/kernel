#ifndef _BIF_BASE_H_
#define _BIF_BASE_H_
#include <linux/interrupt.h>

enum BUFF_ID {
	BUFF_BASE = 0,
	BUFF_ETH,	//bifeth0
	BUFF_SMD,	//biftty0
	BUFF_SIO,	//biftty1
	BUFF_LITE,	//biflite0
	BUFF_MAX,
	BUFF_POSTAKEN = 8 //it seems enough in near future
};

#define IRQ_QUEUE_SIZE 64 /*16->64*/
/*must be 512 padding for BIFSD access*/
struct bif_base_info {
	unsigned char magic[4];
	/*exclusive(one node):1~BUFF_MAX ; share(multi nodes):0*/
	unsigned char running_mode;
	unsigned char irq_queue_size;/*typically==IRQ_QUEUE_SIZE*/
	unsigned char send_irq_tail;
	unsigned char read_irq_head;
	unsigned char register_irqs;
	unsigned char buffer_count;
	unsigned char reserved0[2];
	unsigned int  address_list[BUFF_POSTAKEN];
	unsigned short offset_list[BUFF_POSTAKEN];
	unsigned short reserved1;
	unsigned short next_offset;
	/*64 Byte Taken To Here.*/
	unsigned char  irq[0];
	unsigned char  reserved2[128];/*max 128 interrupt msg*/
	/*192 Byte Taken To Here. for next_offset be used like bif_eth*/
	/*end*/
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
//cp alloc alloc buff id buff memory from base memory, return phyaddr.!discard!
void *bif_alloc_cp(enum BUFF_ID buffer_id, int size, ulong *phyaddr);
// ap sync 'cp side side base info' to ap
int bif_sync_base(void);
// ap sync 'ap side base info' to cp
int bif_sync_ap(void);
// cp dma alloc memory
void *bif_dma_alloc(size_t size, dma_addr_t *dma_addr,
	gfp_t gfp, unsigned long attrs);
void bif_dma_free(size_t size, dma_addr_t *dma_addr,
	gfp_t gfp, unsigned long attrs);

void *bif_get_plat_info(void);
char *bif_get_str_bus(enum BUFF_ID buffer_id);

/*request exclusive mode*/
int bif_excmode_request(enum BUFF_ID buffer_id);
/*leave excludesive mode into share mode*/
int bif_excmode_release(void);
int bif_get_rmode(void);

//wait_type: 0, bif_query_address_wait; 1, bif_query_otherbase_wait
int bif_query_wait_exit(enum BUFF_ID buffer_id, int wait_type);

#endif
