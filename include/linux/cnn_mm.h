#ifndef _CNN_MM_H
#define _CNN_MM_H

//#include <uapi/linux/x2_cnn_mm.h>
#include <linux/dma-buf.h>
#include <linux/scatterlist.h>

struct cnn_client *x2_cnn_client_create(const char *name);

#endif
