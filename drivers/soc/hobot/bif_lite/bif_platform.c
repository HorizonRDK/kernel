#include "bif_platform.h"

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

unsigned long bif_sleep(unsigned int msec)
{
	return msleep_interruptible(msec);
}

void bif_lock(void)
{

}

void bif_unlock(void)
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

void bif_free(void *p)
{
	static int count;

	kfree(p);
	count++;
	//bif_debug("free count %d  p  %p\n", count, p );
}
