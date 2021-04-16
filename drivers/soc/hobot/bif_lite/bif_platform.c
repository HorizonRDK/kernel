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

#define MEM_THRESHOLD (64 * 1024)
#define MEM_ALLOC_TYPE_SIZE (4)
#define MEM_ALLOC_TYPE_VM (0)
#define MEM_ALLOC_TYPE_KM (1)
void *bif_malloc(size_t size)
{
	void *p = NULL;
	int *alloc_type = NULL;

	if (size > MEM_THRESHOLD) {
		p = vmalloc(MEM_ALLOC_TYPE_SIZE + size);
		if (p) {
			alloc_type = p;
			*alloc_type = MEM_ALLOC_TYPE_VM;
		}
	} else {
		p = kmalloc(MEM_ALLOC_TYPE_SIZE + size, GFP_KERNEL);
		if (p) {
			alloc_type = p;
			*alloc_type = MEM_ALLOC_TYPE_KM;
		}
	}

	// return userdata position
	return p + MEM_ALLOC_TYPE_SIZE;
}

void bif_free(void *p)
{
	// alloc_type point to start position of allocated memory
	int *alloc_type = p - MEM_ALLOC_TYPE_SIZE;

	if (!p)
		return;
	if (*alloc_type == MEM_ALLOC_TYPE_VM)
		vfree(alloc_type);
	else if (*alloc_type == MEM_ALLOC_TYPE_KM)
		kfree(alloc_type);
	else
		pr_err("error mem type\n");
}
