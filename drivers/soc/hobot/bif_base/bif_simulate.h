
#ifndef _BIF_BASE_TEST_H_
#define _BIF_BASE_TEST_H_
#include <linux/interrupt.h>
#include "bif_base.h"

#ifdef CONFIG_HOBOT_BIF_TEST
#define BIF_BASE_ADDR	0x7ff00000
#endif

void pr_buff(unsigned char *buff, int len);
void pr_buff_16(unsigned char *buff, int len_16);

int bif_netlink_init(void);
void bif_netlink_exit(void);

int t_bif_register_address(enum BUFF_ID buffer_id, void *address);
int t_bif_register_irq(enum BUFF_ID buffer_id, irq_handler_t irq_handler);
int t_bif_send_irq(int irq);
void *t_bif_query_address(enum BUFF_ID buffer_id);
void *t_bif_query_address_wait(enum BUFF_ID buffer_id);

int t_bif_sd_read(void *addr, unsigned int count, unsigned char *buf);
int t_bif_sd_write(void *addr, unsigned int count, unsigned char *buf);
int t_bif_spi_read(void *addr, unsigned int count, unsigned char *buf);
int t_bif_spi_write(void *addr, unsigned int count, unsigned char *buf);

#endif
