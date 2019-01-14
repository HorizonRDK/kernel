/*
 * ringbuf.c - C ring buffer (FIFO) implementation.
 *
 * Written in 2011 by Drew Hess <dhess-src@bothan.net>.
 *
 * To the extent possible under law, the author(s) have dedicated all
 * copyright and related and neighboring rights to this software to
 * the public domain worldwide. This software is distributed without
 * any warranty.
 *
 * You should have received a copy of the CC0 Public Domain Dedication
 * along with this software. If not, see
 * <http://creativecommons.org/publicdomain/zero/1.0/>.
 */

#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include "ringbuf.h"

#define CONFIG_RINGBUF_DEBUG

#ifdef CONFIG_RINGBUF_DEBUG
#define assert(expr) \
	do { \
		if (!(expr)) { \
			pr_err("Assertion failed! %s,%s,%s,line=%d\n", \
			#expr, __FILE__, __func__, __LINE__); \
		} \
	} while (0)

#define rb_dbg_log(...) pr_err(__VA_ARGS__)
#else
#define assert(expr)
#define rb_dbg_log(...)
#endif

#define	MIN(a, b) (((a) < (b)) ? (a) : (b))

/*
 * The code is written for clarity, not cleverness or performance, and
 * contains many assert()s to enforce invariant assumptions and catch
 * bugs. Feel free to optimize the code and to remove asserts for use
 * in your own projects, once you're comfortable that it functions as
 * intended.
 */
struct ringbuf_t *ringbuf_new(unsigned int capacity, char *rbuf, char *wbuf)
{
	struct ringbuf_t *rb = kzalloc(sizeof(struct ringbuf_t),
				       GFP_KERNEL);
	if (rb) {
		/* One byte is used for detecting the full condition. */
		rb->size = capacity + 1;
		rb->rbuf = rbuf;
		rb->wbuf = wbuf;
	}
	return rb;
}

unsigned int ringbuf_buffer_size(const struct ringbuf_t *rb)
{
	return rb->size;
}

void ringbuf_reset(struct ringbuf_t *rb)
{
	rb->head = 0;
	rb->tail = 0;
}

void ringbuf_free(struct ringbuf_t *rb)
{
	kfree(rb);
}

unsigned int ringbuf_capacity(const struct ringbuf_t *rb)
{
	return ringbuf_buffer_size(rb) - 1;
}

const unsigned int ringbuf_tail(const struct ringbuf_t *rb)
{
	return rb->tail;
}

const unsigned int ringbuf_head(const struct ringbuf_t *rb)
{
	return rb->head;
}

unsigned int ringbuf_write(struct ringbuf_t *rb,
			   const char *buff, unsigned int count)
{
	const unsigned int size = rb->size;
	unsigned int len;

	if (size <= rb->head) {
		rb_dbg_log("%s(%d) bufend <= rb->head\n", __func__, __LINE__);
		return -1;
	}
	len = MIN(size - rb->head, count);
	if (copy_from_user(rb->wbuf + rb->head, buff, len)) {
		rb_dbg_log("%s(%d)...\n", __func__, __LINE__);
		return -2;
	}
	if (len < count) {
		if (copy_from_user(rb->wbuf, buff + len, count - len)) {
			rb_dbg_log("%s(%d)...\n", __func__, __LINE__);
			return -3;
		}
	}
	rb->head += count;
	rb->head %= size;

	return count;
}

unsigned int ringbuf_read(struct ringbuf_t *rb, char *buff, unsigned int count)
{
	unsigned int size = rb->size;
	unsigned int len;

	if (size <= rb->tail) {
		rb_dbg_log("%s(%d) bufend <= rb->tail\n", __func__, __LINE__);
		return -1;
	}
	len = MIN(size - rb->tail, count);
	if (copy_to_user(buff, rb->rbuf + rb->tail, len)) {
		rb_dbg_log("%s(%d)\n", __func__, __LINE__);
		return -2;
	}
	if (len < count) {
		if (copy_to_user(buff + len, rb->rbuf, count - len)) {
			rb_dbg_log("%s(%d)\n", __func__, __LINE__);
			return -3;
		}
	}
	rb->tail += count;
	rb->tail %= size;

	return count;
}
