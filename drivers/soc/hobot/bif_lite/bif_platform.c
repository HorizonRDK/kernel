#include "bif_platform.h"
#include "../bif_base/bif_api.h"

static addr_t base_addr;

void *bif_memset(void *s, int c, size_t n)
{
	int i;
	char *ss = (char *)s;

	for (i = 0; i < n; i++)
		ss[i] = c;
	return s;
}
void *bif_memcpy(void *dest, const void *src, size_t n)
{
	int i;
	char *d;
	char *s;

	d = (char *)dest;
	s = (char *)src;

	for (i = 0; i < n; i++)
		d[i] = s[i];
	return dest;
}
void  bif_rx_sleep(unsigned int msec)
{
	msleep(msec);
}
void  bif_lock(void)
{
}
void  bif_unlock(void)
{
}
void *bif_malloc(size_t size)
{	static int count;
	void *p;

	p = kmalloc(size, GFP_KERNEL);
	if (p)
		count++;
	//bif_debug("malloc count %d size %d  p  %p\n", count, size, p);
	return p;
}
void  bif_free(void *p)
{
	static int count;

	kfree(p);
	count++;
	//bif_debug("free count %d  p  %p\n", count, p );
}

void bif_set_base_addr(addr_t bif_base_addr)
{
	base_addr = bif_base_addr;
}

int  bif_read_cp_ddr(void *dst, addr_t offset, int len)
{
	void *src;

	if (!dst) {
		bif_debug("dst NULL\n");
		return  -1;
	}
	src = (void *)(offset+base_addr);
#ifndef CONFIG_HOBOT_BIF_AP
	bif_memcpy(dst, src, len);
#else
#ifdef CONFIG_HOBOT_BIFSPI
	if (bif_spi_read(src, len, dst))
		return -1;
#endif
#endif
#if 0
	int i;

	for (i = 0; i < len; i++)
		bif_debug(" %02x\n", *((unsigned char *)src+i));
#endif
	return 0;
}

int  bif_write_cp_ddr(void *src, addr_t offset, int len)
{
	void *dst;

	if (!src) {
		bif_debug("src NULL\n");
		return  -1;
	}
	dst = (void *)(offset+base_addr);
#ifndef CONFIG_HOBOT_BIF_AP
	bif_memcpy(dst, src, len);
#else
#ifdef CONFIG_HOBOT_BIFSPI
	if (bif_spi_write(dst, len, src))
		return -1;
#endif
#endif
#if 0
	int i;

	bif_debug("dst %p\n", dst);
	for (i = 0; i < len; i++)
		bif_debug(" %02x\n", *((unsigned char *)dst+i));
#endif
	return 0;
}
