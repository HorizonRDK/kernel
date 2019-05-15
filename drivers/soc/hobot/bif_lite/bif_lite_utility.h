#ifndef _BIF_LITE_UTILITY_H_
#define _BIF_LITE_UTILITY_H_

#include "bif_lite.h"

int bif_rx_get_frame(void *data);
int bif_rx_get_stock_frame(struct bif_frame_cache   **frame);
void bif_del_frame_from_list(struct bif_frame_cache  *frame);
int bif_tx_put_frame(void *data, int len);
int bif_start(void);
int bif_lite_init(void);
void bif_lite_exit(void);
int bif_lite_register_irq(irq_handler_t handler);

#endif
