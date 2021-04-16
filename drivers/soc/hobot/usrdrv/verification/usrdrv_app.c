#include <unistd.h>
#include <fcntl.h>
#include <stdlib.h>
#include <linux/netlink.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/epoll.h>
#include <linux/socket.h>

#include "usrdrv_lib.h"

#define MAX_IRQ_TEST 3
typedef struct my_info_s {
	uint8_t index;
	uint32_t cnt;
	int fd;
	int running;
} my_info_t;

my_info_t my_info[MAX_IRQ_TEST];

volatile uint8_t *g_virt_io;
volatile uint8_t *g_virt_mem_cached;
volatile uint8_t *g_virt_mem_ncached;
interrupt_info_t g_intr_info[MAX_IRQ_TEST];
uint32_t intr_test_cnt = 1000;

uint32_t my_handler(void *data)
{
	my_info_t *p_my_info = (my_info_t *) data;
	p_my_info->cnt++;
	if (p_my_info->cnt % 200 == 0)
		printf("cnt=%d (intr index %d)\n", p_my_info->cnt, p_my_info->index);

	if (p_my_info->cnt >= intr_test_cnt) {
		p_my_info->running = 0;
		printf("my handler %d exit, cnt=%d\n", p_my_info->index, p_my_info->cnt);
		//info usrdrv_lib irq_thread goto exit
		return 0;
	}

	//info usrdrv_lib irq_thread keep running
	return 1;
}

int main(int argc, char *argv[])
{
	int fd;
	int irq_index = 0;
	int max = MAX_IRQ_TEST;
	usr_open(&fd);
	int running = 1;

	usr_map_phys_addr(fd, (void **)&g_virt_io, (void **)&g_virt_mem_cached,
			(void **)&g_virt_mem_ncached);
	printf("g_virt_io=%p\n", g_virt_io);

	printf("\n==== x2 test bif offset 31 ====\n");
	printf("bif offset 31=%x, (expect val = 5a5a0200)\n",
			usr_readl(g_virt_io, 0x01006000 + 31 * 4));
	printf("\n==== x2 test gpio interrupt ====\n");

	if (argc > 1)
		intr_test_cnt = atoi(argv[1]);

	for (irq_index = 0; irq_index < max; irq_index++) {
		my_info[irq_index].cnt = 0;
		my_info[irq_index].index = irq_index;
		my_info[irq_index].running = 1;
		usr_open(&my_info[irq_index].fd);
		usr_auto_fire(my_info[irq_index].fd);
		usr_request_irq(my_info[irq_index].fd, irq_index, "usrdrv irq",
				&g_intr_info[irq_index], my_handler,
				&my_info[irq_index]);
	}

	while (running) {
		running = 0;
		for (irq_index = 0; irq_index < max; irq_index++)
			running += my_info[irq_index].running;
		sleep(1);
		printf("running = %d\n", running);
	}

	printf("==== all interrupt thread exit=====\n");
	for (irq_index = 0; irq_index < max; irq_index++)
		usr_free_irq(my_info[irq_index].fd);

}
