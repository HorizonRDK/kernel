
#ifndef _BIF_API_H_
#define _BIF_API_H_
#include <linux/interrupt.h>

#ifdef CONFIG_HOBOT_BIF_TEST
extern int bif_netlink_init(void);
extern void bif_netlink_exit(void);
/*extern int t_bif_register_address(BUFF_ID buffer_id, void *address);*/
extern int t_bif_register_irq(BUFF_ID buffer_id, irq_handler_t irq_handler);
extern int t_bif_send_irq(int irq);
/*extern void *t_bif_query_address(BUFF_ID buffer_id);*/
/*extern void *t_bif_query_address_wait(BUFF_ID buffer_id);*/
extern int t_bif_sd_read(void *addr, unsigned int count, char *buf);
extern int t_bif_sd_write(void *addr, unsigned int count, char *buf);
extern int t_bif_spi_read(void *addr, unsigned int count, char *buf);
extern int t_bif_spi_write(void *addr, unsigned int count, char *buf);
#else
#ifdef CONFIG_HOBOT_BIF_AP
#ifdef CONFIG_HOBOT_BIFSD
extern int bifsd_read(void *addr, unsigned int count, char *buf);
extern int bifsd_write(void *addr, unsigned int count, char *buf);
#else
#ifdef CONFIG_HOBOT_BIFSPI
extern int bifspi_read(void *addr, unsigned int count, char *buf);
extern int bifspi_write(void *addr, unsigned int count, char *buf);
#endif
#endif
#endif
#endif

#ifdef CONFIG_HOBOT_BIF_AP
#ifdef CONFIG_HOBOT_BIFSD
int bif_sd_read(void *addr, unsigned int count, char *buf);
int bif_sd_write(void *addr, unsigned int count, char *buf);
#else
#ifdef CONFIG_HOBOT_BIFSPI
int bif_spi_read(void *addr, unsigned int count, char *buf);
int bif_spi_write(void *addr, unsigned int count, char *buf);
#endif
#endif
#endif

#endif
