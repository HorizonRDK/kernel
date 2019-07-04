#ifndef INCLUDED_RINGBUF_H
#define INCLUDED_RINGBUF_H

/*
 * ringbuf.h - C ring buffer (FIFO) interface.
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

/*
 * A byte-addressable ring buffer FIFO implementation.
 *
 * The ring buffer's head pointer points to the starting location
 * where data should be written when copying data *into* the buffer
 * (e.g., with ringbuf_read). The ring buffer's tail pointer points to
 * the starting location where data should be read when copying data
 * *from* the buffer (e.g., with ringbuf_write).
 */

#include <linux/kernel.h>

struct ringbuf_t {
	unsigned int head;
	unsigned int tail;
	unsigned int size;
	unsigned int reserve;
};

/*
 * Create a new ring buffer with the given capacity (usable
 * bytes). Note that the actual internal buffer size may be one or
 * more bytes larger than the usable capacity, for bookkeeping.
 *
 * Returns the new ring buffer object, or 0 if there's not enough
 * memory to fulfill the request for the given capacity.
 */
struct ringbuf_t *ringbuf_new(unsigned int capacity, char *rbuf, char *wbuf);

/*
 * The size of the internal buffer, in bytes. One or more bytes may be
 * unusable in order to distinguish the "buffer full" state from the
 * "buffer empty" state.
 *
 * For the usable capacity of the ring buffer, use the
 * ringbuf_capacity function.
 */
unsigned int ringbuf_buffer_size(const struct ringbuf_t *rb);

/*
 * Deallocate a ring buffer, and, as a side effect, set the pointer to
 * 0.
 */
void ringbuf_free(struct ringbuf_t *rb);

/*
 * Reset a ring buffer to its initial state (empty).
 */
void ringbuf_reset(struct ringbuf_t *rb);

/*
 * The usable capacity of the ring buffer, in bytes. Note that this
 * value may be less than the ring buffer's internal buffer size, as
 * returned by ringbuf_buffer_size.
 */
unsigned int ringbuf_capacity(const struct ringbuf_t *rb);
/*
 * Const access to the head and tail pointers of the ring buffer.
 */

const unsigned int ringbuf_tail(const struct ringbuf_t *rb);

const unsigned int ringbuf_head(const struct ringbuf_t *rb);

/*
 * This convenience function calls read(2) on the file descriptor fd,
 * using the ring buffer rb as the destination buffer for the read,
 * and returns the value returned by read(2). It will only call
 * read(2) once, and may return a short count.
 *
 * It is possible to read more data from the file descriptor than is
 * available in the buffer; i.e., it's possible to overflow the ring
 * buffer using this function. When an overflow occurs, the state of
 * the ring buffer is guaranteed to be consistent, including the head
 * and tail pointers: old data will simply be overwritten in FIFO
 * fashion, as needed. However, note that, if calling the function
 * results in an overflow, the value of the ring buffer's tail pointer
 * may be different than it was before the function was called.
 */
unsigned int ringbuf_read(struct ringbuf_t *rb, char *dst,
			  char *src, unsigned int count);

/*
 * This convenience function calls write(2) on the file descriptor fd,
 * using the ring buffer rb as the source buffer for writing (starting
 * at the ring buffer's tail pointer), and returns the value returned
 * by write(2). It will only call write(2) once, and may return a
 * short count.
 *
 * Note that this copy is destructive with respect to the ring buffer:
 * any bytes written from the ring buffer to the file descriptor are
 * no longer available in the ring buffer after the copy is complete,
 * and the ring buffer will have N more free bytes than it did before
 * the function was called, where N is the value returned by the
 * function (unless N is < 0, in which case an error occurred and no
 * bytes were written).
 *
 * This function will *not* allow the ring buffer to underflow. If
 * count is greater than the number of bytes used in the ring buffer,
 * no bytes are written to the file descriptor, and the function will
 * return 0.
 */
unsigned int ringbuf_write(struct ringbuf_t *rb, char *dst,
			   const char *src, unsigned int count);

#endif /* INCLUDED_RINGBUF_H */
