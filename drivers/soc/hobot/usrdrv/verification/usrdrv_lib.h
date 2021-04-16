#ifndef USRDRV_LIB_H
#define USRDRV_LIB_H
#include "../usrdrv.h"
#define USRDRV_DEVICE_NAME  "/dev/usrdrv"

typedef uint32_t(*INTR_CALLBACK)(void *);

typedef struct interrupt_info_s {
	pthread_t thread_handle;
	int thread_run;
	int fd;
	INTR_CALLBACK cb;
	void *cb_data;
} interrupt_info_t;

uint32_t usr_readl(volatile uint8_t *base, uint32_t reg);
void usr_writel(volatile uint8_t *base, uint32_t val, uint32_t reg);
int usr_open(int *fd);
int usr_close(int fd);
int usr_map_phys_addr(int fd, void **virt_io, void **virt_mem_cached,
		      void **virt_mem_ncached);
int usr_request_irq(int fd, uint32_t irq_index, uint8_t *irq_name,
		    interrupt_info_t *intr_info, INTR_CALLBACK cb,
		    void *cb_data);
int usr_auto_fire(int fd);
int usr_free_irq(int fd);

#endif				//USRDRV_LIB_H
/********************in DTS**********************
usedrv: usrdrv@0xA700C000 {
			compatible = "hobot,usrdrv";
			reg = <0 0xA500C000 0 0x100>;
			interrupt-parent = <&gic>;
			interrupts = <0 42 4>, <0 41 4>, <0 40 4>;

************************************************/
