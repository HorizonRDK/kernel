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

/******************************************************************************
* Define:
*******************************************************************************/

#define MEM_PHYS_START      0x00000000
#define MEM_LENGTH          (2*1024*1024*1024L)
#define IOMEM_PHYS_START    0xA0000000
#define IOMEM_LENGTH        (256*1024*1024L)
#define IRQ_NUM             0
#define IRQ_NAME            "interrupt test"

#define EPOLL_MAX_FD         1

#define PRINT_CNT 100

uint32_t usr_readl(volatile uint8_t *base, uint32_t reg)
{
	uint32_t val;
	val = *((uint32_t *)(base + (reg)));
	return val;
}

void writel(volatile uint8_t *base, uint32_t val, uint32_t reg)
{
	*((uint32_t *)(base + (reg))) = val;
}

static void *irq_thread(void *arg)
{
	interrupt_info_t *intr_info = (interrupt_info_t *) arg;
	int ret = 0;
	int epfd = 0;
	struct epoll_event ev;
	uint32_t count = 0;
	uint32_t count_timeout = 0,
			 count_in = 0,
			 count_err = 0,
			 count_pri = 0,
			 count_hup = 0,
			 count_unknown = 0;

	printf("usrdrv msg thread start\n");

	epfd = epoll_create(EPOLL_MAX_FD);
	ev.data.fd = intr_info->fd;
	ev.events = EPOLLIN | EPOLLHUP | EPOLLET;
	epoll_ctl(epfd, EPOLL_CTL_ADD, intr_info->fd, &ev);

	intr_info->thread_run = 1;
	while (intr_info->thread_run) {
		struct epoll_event event;
		uint32_t enable = 1;
		ret = epoll_wait(epfd, &event, EPOLL_MAX_FD, 5000);
		if (!ret) {
			count_timeout++;
		} else {
			if (event.events & EPOLLIN) {
				count_in++;
				intr_info->thread_run =
					intr_info->cb(intr_info->cb_data);
			} else if (event.events & EPOLLERR) {
				count_err++;
			} else if (event.events & EPOLLPRI) {
				count_pri++;
			} else if (event.events & EPOLLHUP) {
				count_hup++;
				break;
			} else {
				count_unknown++;
			}
		}
#if 0
		//this maybe not need
		ev.data.fd = intr_info->fd;
		ev.events = EPOLLIN | EPOLLHUP | EPOLLET;
		epoll_ctl(epfd, EPOLL_CTL_MOD, intr_info->fd, &ev);
#endif

		count++;
	}
	printf("usrdrv msg thread exit\n");
	printf("count %d: timeout=%d,in=%d,err=%d,pri=%d,hup=%d,unknow=%d\n",
			count, count_timeout, count_in, count_err, count_pri, count_hup,
			count_unknown);

	intr_info->thread_run = 0;
	pthread_exit(NULL);
	return NULL;
}

int usr_request_irq(int fd, uint32_t irq_index, uint8_t *irq_name,
		interrupt_info_t *intr_info, INTR_CALLBACK cb,
		void *cb_data)
{
	int ret;
	intr_info->fd = fd;
	intr_info->cb = cb;
	intr_info->cb_data = cb_data;
	ioctl_irq_info_t irq_info;

	irq_info.irq_index = irq_index;
	strncpy(irq_info.irq_name, irq_name, USRDRV_IRQNAME_MAX);

	ioctl(fd, USRDRV_IRQ_REGISTER, &irq_info);

	//interrupt will be auto enable when polling
	ret =
		pthread_create(&intr_info->thread_handle, NULL, irq_thread,
			       intr_info);
	if (ret < 0) {
		printf("create irq thread fail");
		return -1;
	}

}

int usr_free_irq(int fd)
{
	ioctl(fd, USRDRV_IRQ_UNREGISTER, NULL);
}

int usr_auto_fire(int fd)
{
	uint32_t irq_auto_fire = 1;
	ioctl(fd, USRDRV_IRQ_AUTO_FIRE, &irq_auto_fire);
}

int usr_map_phys_addr(int fd, void **virt_io, void **virt_mem_cached,
		void **virt_mem_ncached)
{
	int ret;

	*virt_io = mmap(NULL, IOMEM_LENGTH, PROT_READ | PROT_WRITE,
			MAP_SHARED, fd, IOMEM_PHYS_START);
	if (virt_io == MAP_FAILED) {
		printf("map virt_io fail \n");
		return -1;
	}

	*virt_mem_ncached = mmap(NULL, MEM_LENGTH, PROT_READ | PROT_WRITE,
			MAP_SHARED, fd, MEM_PHYS_START);
	if (virt_mem_ncached == MAP_FAILED) {
		printf("map virt_mem_ncached fail \n");
		return -1;
	}

	*virt_mem_cached = NULL;
	printf("not support virt_mem_cached\n");
}

int usr_open(int *fd)
{
	*fd = open(USRDRV_DEVICE_NAME, O_RDWR);
	return 0;
}

int usr_close(int fd)
{
	close(fd);
	return 0;
}
