/*Copyright 2011 The LibYuv Project Authors. All rights reserved.
 *
 *Redistribution and use in source and binary forms, with or without
 *modification, are permitted provided that the following conditions are
 *met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *
 *  * Neither the name of Google nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 *THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <asm/neon.h>

static const uint8_t kVTbl4x4Transpose[16] = {
	0, 4, 8,  12, 1, 5, 9,  13,
	2, 6, 10, 14, 3, 7, 11, 15
};

static void TransposeWx8_NEON(const uint8_t *src, int src_stride,
		uint8_t *dst, int dst_stride, int width)
{
	const uint8_t *src_temp;

	kernel_neon_begin();
	asm volatile (
		// loops are on blocks of 8. loop will stop when
		// counter gets to or below 0. starting the counter
		// at w-8 allow for this
		"sub	%w3, %w3, #8\n"

		// handle 8x8 blocks. this should be the
		// majority of the plane
		"1:\n"
		"mov	%0, %1\n"

		"ld1	{v0.8b}, [%0], %5\n"
		"ld1	{v1.8b}, [%0], %5\n"
		"ld1	{v2.8b}, [%0], %5\n"
		"ld1	{v3.8b}, [%0], %5\n"
		"ld1	{v4.8b}, [%0], %5\n"
		"ld1	{v5.8b}, [%0], %5\n"
		"ld1	{v6.8b}, [%0], %5\n"
		"ld1	{v7.8b}, [%0]\n"

		"trn2	v16.8b, v0.8b, v1.8b\n"
		"trn1	v17.8b, v0.8b, v1.8b\n"
		"trn2	v18.8b, v2.8b, v3.8b\n"
		"trn1	v19.8b, v2.8b, v3.8b\n"
		"trn2	v20.8b, v4.8b, v5.8b\n"
		"trn1	v21.8b, v4.8b, v5.8b\n"
		"trn2	v22.8b, v6.8b, v7.8b\n"
		"trn1	v23.8b, v6.8b, v7.8b\n"

		"trn2	v3.4h, v17.4h, v19.4h\n"
		"trn1	v1.4h, v17.4h, v19.4h\n"
		"trn2	v2.4h, v16.4h, v18.4h\n"
		"trn1	v0.4h, v16.4h, v18.4h\n"
		"trn2	v7.4h, v21.4h, v23.4h\n"
		"trn1	v5.4h, v21.4h, v23.4h\n"
		"trn2	v6.4h, v20.4h, v22.4h\n"
		"trn1	v4.4h, v20.4h, v22.4h\n"

		"trn2	v21.2s, v1.2s, v5.2s\n"
		"trn1	v17.2s, v1.2s, v5.2s\n"
		"trn2	v20.2s, v0.2s, v4.2s\n"
		"trn1	v16.2s, v0.2s, v4.2s\n"
		"trn2	v23.2s, v3.2s, v7.2s\n"
		"trn1	v19.2s, v3.2s, v7.2s\n"
		"trn2	v22.2s, v2.2s, v6.2s\n"
		"trn1	v18.2s, v2.2s, v6.2s\n"

		"mov	%0, %2\n"

		"st1	{v17.8b}, [%0], %6\n"
		"st1	{v16.8b}, [%0], %6\n"
		"st1	{v19.8b}, [%0], %6\n"
		"st1	{v18.8b}, [%0], %6\n"
		"st1	{v21.8b}, [%0], %6\n"
		"st1	{v20.8b}, [%0], %6\n"
		"st1	{v23.8b}, [%0], %6\n"
		"st1	{v22.8b}, [%0]\n"

		"add	%1, %1, #8\n"  // src += 8
		"add	%2, %2, %6, lsl #3\n"  // dst += 8 * dst_stride
		"subs	%w3, %w3, #8\n"  // w   -= 8
		"b.ge	1b\n"

		// add 8 back to counter. if the result is 0 there are
		// no residuals.
		"adds	%w3, %w3, #8\n"
		"b.eq	4f\n"

		// some residual, so between 1 and 7 lines left to transpose
		"cmp	%w3, #2\n"
		"b.lt	3f\n"

		"cmp	%w3, #4\n"
		"b.lt	2f\n"

		// 4x8 block
		"mov	%0, %1\n"
		"ld1	{v0.s}[0], [%0], %5\n"
		"ld1	{v0.s}[1], [%0], %5\n"
		"ld1	{v0.s}[2], [%0], %5\n"
		"ld1	{v0.s}[3], [%0], %5\n"
		"ld1	{v1.s}[0], [%0], %5\n"
		"ld1	{v1.s}[1], [%0], %5\n"
		"ld1	{v1.s}[2], [%0], %5\n"
		"ld1	{v1.s}[3], [%0]\n"

		"mov	%0, %2\n"

		"ld1	{v2.16b}, [%4]\n"

		"tbl	v3.16b, {v0.16b}, v2.16b\n"
		"tbl	v0.16b, {v1.16b}, v2.16b\n"

		// TODO(frkoenig): Rework shuffle above to
		// write out with 4 instead of 8 writes.
		"st1	{v3.s}[0], [%0], %6\n"
		"st1	{v3.s}[1], [%0], %6\n"
		"st1	{v3.s}[2], [%0], %6\n"
		"st1	{v3.s}[3], [%0]\n"

		"add	%0, %2, #4\n"
		"st1	{v0.s}[0], [%0], %6\n"
		"st1	{v0.s}[1], [%0], %6\n"
		"st1	{v0.s}[2], [%0], %6\n"
		"st1	{v0.s}[3], [%0]\n"

		"add	%1, %1, #4\n"  // src += 4
		"add	%2, %2, %6, lsl #2\n"  // dst += 4 * dst_stride
		"subs	%w3, %w3, #4\n"  // w   -= 4
		"b.eq	4f\n"

		// some residual, check to see if it includes a 2x8 block,
		// or less
		"cmp	%w3, #2\n"
		"b.lt	3f\n"

		// 2x8 block
		"2:\n"
		"mov	%0, %1\n"
		"ld1	{v0.h}[0], [%0], %5\n"
		"ld1	{v1.h}[0], [%0], %5\n"
		"ld1	{v0.h}[1], [%0], %5\n"
		"ld1	{v1.h}[1], [%0], %5\n"
		"ld1	{v0.h}[2], [%0], %5\n"
		"ld1	{v1.h}[2], [%0], %5\n"
		"ld1	{v0.h}[3], [%0], %5\n"
		"ld1	{v1.h}[3], [%0]\n"

		"trn2	v2.8b, v0.8b, v1.8b\n"
		"trn1	v3.8b, v0.8b, v1.8b\n"

		"mov	%0, %2\n"

		"st1	{v3.8b}, [%0], %6\n"
		"st1	{v2.8b}, [%0]\n"

		"add	%1, %1, #2\n"  // src += 2
		"add	%2, %2, %6, lsl #1\n"  // dst += 2 * dst_stride
		"subs	%w3, %w3,  #2\n"  // w   -= 2
		"b.eq	4f\n"

		// 1x8 block
		"3:\n"
		"ld1	{v0.b}[0], [%1], %5\n"
		"ld1	{v0.b}[1], [%1], %5\n"
		"ld1	{v0.b}[2], [%1], %5\n"
		"ld1	{v0.b}[3], [%1], %5\n"
		"ld1	{v0.b}[4], [%1], %5\n"
		"ld1	{v0.b}[5], [%1], %5\n"
		"ld1	{v0.b}[6], [%1], %5\n"
		"ld1	{v0.b}[7], [%1]\n"

		"st1	{v0.8b}, [%2]\n"

		"4:\n"

		: "=&r"(src_temp),		// %0
			"+r"(src),		// %1
			"+r"(dst),		// %2
			"+r"(width)		// %3
		: "r"(&kVTbl4x4Transpose),	// %4
			"r"((ptrdiff_t)(src_stride)),	// %5
			"r"((ptrdiff_t)(dst_stride))	// %6
		: "memory", "cc", "v0", "v1", "v2", "v3", "v4",
		"v5", "v6", "v7", "v16", "v17", "v18", "v19", "v20",
		"v21", "v22", "v23"
	);
	kernel_neon_end();
}


static const uint8_t kVTbl4x4TransposeDi[32] = {
	0, 16, 32, 48, 2, 18, 34, 50, 4, 20, 36, 52, 6, 22, 38, 54,
	1, 17, 33, 49, 3, 19, 35, 51, 5, 21, 37, 53, 7, 23, 39, 55
};


static void TransposeUVWx8_NEON(const uint8_t *src,
		int src_stride, uint8_t *dst_a, int dst_stride_a,
		uint8_t *dst_b, int dst_stride_b, int width)
{
	const uint8_t *src_temp;

	kernel_neon_begin();
	asm volatile (
	// loops are on blocks of 8. loop will stop when
	// counter gets to or below 0. starting the counter
	// at w-8 allow for this
	"sub	%w4, %w4, #8\n"

	// handle 8x8 blocks. this should be the majority of the plane
	"1:\n"
	"mov	%0, %1\n"

	"ld1	{v0.16b}, [%0], %5\n"
	"ld1	{v1.16b}, [%0], %5\n"
	"ld1	{v2.16b}, [%0], %5\n"
	"ld1	{v3.16b}, [%0], %5\n"
	"ld1	{v4.16b}, [%0], %5\n"
	"ld1	{v5.16b}, [%0], %5\n"
	"ld1	{v6.16b}, [%0], %5\n"
	"ld1	{v7.16b}, [%0]\n"

	"trn1	v16.16b, v0.16b, v1.16b\n"
	"trn2	v17.16b, v0.16b, v1.16b\n"
	"trn1	v18.16b, v2.16b, v3.16b\n"
	"trn2	v19.16b, v2.16b, v3.16b\n"
	"trn1	v20.16b, v4.16b, v5.16b\n"
	"trn2	v21.16b, v4.16b, v5.16b\n"
	"trn1	v22.16b, v6.16b, v7.16b\n"
	"trn2	v23.16b, v6.16b, v7.16b\n"

	"trn1	v0.8h, v16.8h, v18.8h\n"
	"trn2	v1.8h, v16.8h, v18.8h\n"
	"trn1	v2.8h, v20.8h, v22.8h\n"
	"trn2	v3.8h, v20.8h, v22.8h\n"
	"trn1	v4.8h, v17.8h, v19.8h\n"
	"trn2	v5.8h, v17.8h, v19.8h\n"
	"trn1	v6.8h, v21.8h, v23.8h\n"
	"trn2	v7.8h, v21.8h, v23.8h\n"

	"trn1	v16.4s, v0.4s, v2.4s\n"
	"trn2	v17.4s, v0.4s, v2.4s\n"
	"trn1	v18.4s, v1.4s, v3.4s\n"
	"trn2	v19.4s, v1.4s, v3.4s\n"
	"trn1	v20.4s, v4.4s, v6.4s\n"
	"trn2	v21.4s, v4.4s, v6.4s\n"
	"trn1	v22.4s, v5.4s, v7.4s\n"
	"trn2	v23.4s, v5.4s, v7.4s\n"

	"mov	%0, %2\n"

	"st1	{v16.d}[0], [%0], %6\n"
	"st1	{v18.d}[0], [%0], %6\n"
	"st1	{v17.d}[0], [%0], %6\n"
	"st1	{v19.d}[0], [%0], %6\n"
	"st1	{v16.d}[1], [%0], %6\n"
	"st1	{v18.d}[1], [%0], %6\n"
	"st1	{v17.d}[1], [%0], %6\n"
	"st1	{v19.d}[1], [%0]\n"

	"mov	%0, %3\n"

	"st1	{v20.d}[0], [%0], %7\n"
	"st1	{v22.d}[0], [%0], %7\n"
	"st1	{v21.d}[0], [%0], %7\n"
	"st1	{v23.d}[0], [%0], %7\n"
	"st1	{v20.d}[1], [%0], %7\n"
	"st1	{v22.d}[1], [%0], %7\n"
	"st1	{v21.d}[1], [%0], %7\n"
	"st1	{v23.d}[1], [%0]\n"

	"add	%1, %1, #16\n"// src   += 8*2
	"add	%2, %2, %6, lsl #3\n"// dst_a += 8 *
	// dst_stride_a
	"add	%3, %3, %7, lsl #3\n"// dst_b += 8 *
	// dst_stride_b
	"subs	%w4, %w4,  #8\n"// w     -= 8
	"b.ge	1b\n"

	// add 8 back to counter. if the result is 0 there are
	// no residuals.
	"adds	%w4, %w4, #8\n"
	"b.eq	4f\n"

	// some residual, so between 1 and 7 lines left to transpose
	"cmp	%w4, #2\n"
	"b.lt	3f\n"

	"cmp	%w4, #4\n"
	"b.lt	2f\n"

	// TODO(frkoenig): Clean this up
	// 4x8 block
	"mov	%0, %1\n"
	"ld1	{v0.8b}, [%0], %5\n"
	"ld1	{v1.8b}, [%0], %5\n"
	"ld1	{v2.8b}, [%0], %5\n"
	"ld1	{v3.8b}, [%0], %5\n"
	"ld1	{v4.8b}, [%0], %5\n"
	"ld1	{v5.8b}, [%0], %5\n"
	"ld1	{v6.8b}, [%0], %5\n"
	"ld1	{v7.8b}, [%0]\n"

	"ld1	{v30.16b}, [%8], #16\n"
	"ld1	{v31.16b}, [%8]\n"

	"tbl	v16.16b, {v0.16b, v1.16b, v2.16b, v3.16b}, v30.16b\n"
	"tbl	v17.16b, {v0.16b, v1.16b, v2.16b, v3.16b}, v31.16b\n"
	"tbl	v18.16b, {v4.16b, v5.16b, v6.16b, v7.16b}, v30.16b\n"
	"tbl	v19.16b, {v4.16b, v5.16b, v6.16b, v7.16b}, v31.16b\n"

	"mov	%0, %2\n"

	"st1	{v16.s}[0],  [%0], %6\n"
	"st1	{v16.s}[1],  [%0], %6\n"
	"st1	{v16.s}[2],  [%0], %6\n"
	"st1	{v16.s}[3],  [%0], %6\n"

	"add	%0, %2, #4\n"
	"st1	{v18.s}[0], [%0], %6\n"
	"st1	{v18.s}[1], [%0], %6\n"
	"st1	{v18.s}[2], [%0], %6\n"
	"st1	{v18.s}[3], [%0]\n"

	"mov	%0, %3\n"

	"st1	{v17.s}[0], [%0], %7\n"
	"st1	{v17.s}[1], [%0], %7\n"
	"st1	{v17.s}[2], [%0], %7\n"
	"st1	{v17.s}[3], [%0], %7\n"

	"add	%0, %3, #4\n"
	"st1	{v19.s}[0],  [%0], %7\n"
	"st1	{v19.s}[1],  [%0], %7\n"
	"st1	{v19.s}[2],  [%0], %7\n"
	"st1	{v19.s}[3],  [%0]\n"

	"add	%1, %1, #8\n"// src   += 4 * 2
	"add	%2, %2, %6, lsl #2\n"// dst_a += 4 *
	// dst_stride_a
	"add	%3, %3, %7, lsl #2\n"// dst_b += 4 *
	// dst_stride_b
	"subs	%w4,  %w4,  #4\n"// w     -= 4
	"b.eq	4f\n"

	// some residual, check to see if it includes a 2x8 block,
	// or less
	"cmp	%w4, #2\n"
	"b.lt	3f\n"

	// 2x8 block
	"2:\n"
	"mov	%0, %1\n"
	"ld2	{v0.h, v1.h}[0], [%0], %5\n"
	"ld2	{v2.h, v3.h}[0], [%0], %5\n"
	"ld2	{v0.h, v1.h}[1], [%0], %5\n"
	"ld2	{v2.h, v3.h}[1], [%0], %5\n"
	"ld2	{v0.h, v1.h}[2], [%0], %5\n"
	"ld2	{v2.h, v3.h}[2], [%0], %5\n"
	"ld2	{v0.h, v1.h}[3], [%0], %5\n"
	"ld2	{v2.h, v3.h}[3], [%0]\n"

	"trn1	v4.8b, v0.8b, v2.8b\n"
	"trn2	v5.8b, v0.8b, v2.8b\n"
	"trn1	v6.8b, v1.8b, v3.8b\n"
	"trn2	v7.8b, v1.8b, v3.8b\n"

	"mov	%0, %2\n"

	"st1	{v4.d}[0], [%0], %6\n"
	"st1	{v6.d}[0], [%0]\n"

	"mov	%0, %3\n"

	"st1	{v5.d}[0], [%0], %7\n"
	"st1	{v7.d}[0], [%0]\n"

	"add	%1, %1, #4\n"// src   += 2 * 2
	"add	%2, %2, %6, lsl #1\n"// dst_a += 2 *
	// dst_stride_a
	"add	%3, %3, %7, lsl #1\n"// dst_b += 2 *
	// dst_stride_b
	"subs	%w4,  %w4,  #2\n"// w -= 2
	"b.eq	4f\n"

	// 1x8 block
	"3:\n"
	"ld2	{v0.b, v1.b}[0], [%1], %5\n"
	"ld2	{v0.b, v1.b}[1], [%1], %5\n"
	"ld2	{v0.b, v1.b}[2], [%1], %5\n"
	"ld2	{v0.b, v1.b}[3], [%1], %5\n"
	"ld2	{v0.b, v1.b}[4], [%1], %5\n"
	"ld2	{v0.b, v1.b}[5], [%1], %5\n"
	"ld2	{v0.b, v1.b}[6], [%1], %5\n"
	"ld2	{v0.b, v1.b}[7], [%1]\n"

	"st1	{v0.d}[0], [%2]\n"
	"st1	{v1.d}[0], [%3]\n"

	"4:\n"

	: "=&r"(src_temp),	// %0
		"+r"(src),	// %1
		"+r"(dst_a),	// %2
		"+r"(dst_b),	// %3
		"+r"(width)	// %4
	: "r"((ptrdiff_t)(src_stride)),		// %5
		"r"((ptrdiff_t)(dst_stride_a)),	// %6
		"r"((ptrdiff_t)(dst_stride_b)),	// %7
		"r"(&kVTbl4x4TransposeDi)	// %8
	: "memory", "cc", "v0", "v1", "v2", "v3", "v4",
	"v5", "v6", "v7", "v16", "v17", "v18", "v19",
	"v20", "v21", "v22", "v23", "v30", "v31");
	kernel_neon_end();
}


static void TransposeWxH_C(const uint8_t *src, int src_stride,
		uint8_t *dst, int dst_stride, int width, int height)
{
	int i;

	for (i = 0; i < width; ++i) {
		int j;

		for (j = 0; j < height; ++j)
			dst[i * dst_stride + j] = src[j * src_stride + i];
	}
}

void TransposeUVWxH_C(const uint8_t *src, int src_stride,
		uint8_t *dst_a, int dst_stride_a,
		uint8_t *dst_b, int dst_stride_b,
		int width, int height)
{
	int i;

	for (i = 0; i < width * 2; i += 2) {
		int j;

		for (j = 0; j < height; ++j) {
			dst_a[j + ((i >> 1) * dst_stride_a)] =
				src[i + (j * src_stride)];
			dst_b[j + ((i >> 1) * dst_stride_b)] =
				src[i + (j * src_stride) + 1];
		}
	}
}


void TransposePlane(const uint8_t *src, int src_stride,
		uint8_t *dst, int dst_stride, int width, int height)
{
	int i = height;
	// Work across the source in 8x8 tiles
	while (i >= 8) {
		TransposeWx8_NEON(src, src_stride, dst, dst_stride, width);
		src += 8 * src_stride;  // Go down 8 rows.
		dst += 8;               // Move over 8 columns.
		i -= 8;
	}

	if (i > 0)
		TransposeWxH_C(src, src_stride, dst, dst_stride, width, i);
}

void RotatePlane90(const uint8_t *src, int src_stride,
		uint8_t *dst, int dst_stride, int width,
		int height)
{
	// Rotate by 90 is a transpose with the source read
	// from bottom to top. So set the source pointer to the end
	// of the buffer and flip the sign of the source stride.
	src += src_stride * (height - 1);
	src_stride = -src_stride;
	TransposePlane(src, src_stride, dst, dst_stride, width, height);
}

void RotatePlane270(const uint8_t *src, int src_stride,
		uint8_t *dst, int dst_stride, int width,
		int height)
{
	// Rotate by 270 is a transpose with the destination written
	// from bottom to top. So set the destination pointer to the end
	// of the buffer and flip the sign of the destination stride.
	dst += dst_stride * (width - 1);
	dst_stride = -dst_stride;
	TransposePlane(src, src_stride, dst, dst_stride, width, height);
}

void TransposeUV(const uint8_t *src, int src_stride,
		uint8_t *dst_a, int dst_stride_a, uint8_t *dst_b,
		int dst_stride_b, int width, int height)
{
	int i = height;
	// Work through the source in 8x8 tiles.
	while (i >= 8) {
		//preempt_disable();
		TransposeUVWx8_NEON(src, src_stride, dst_a, dst_stride_a,
				dst_b, dst_stride_b, width);
		//preempt_enable();
		src += 8 * src_stride;  // Go down 8 rows.
		dst_a += 8;             // Move over 8 columns.
		dst_b += 8;             // Move over 8 columns.
		i -= 8;
	}

	if (i > 0) {
		TransposeUVWxH_C(src, src_stride, dst_a,
			dst_stride_a, dst_b, dst_stride_b, width, i);
	}
}

void RotateUV90(const uint8_t *src, int src_stride, uint8_t *dst_a,
		int dst_stride_a, uint8_t *dst_b, int dst_stride_b,
		int width, int height)
{
	src += src_stride * (height - 1);
	src_stride = -src_stride;

	TransposeUV(src, src_stride, dst_a, dst_stride_a,
			dst_b, dst_stride_b, width, height);
}

void RotateUV270(const uint8_t *src, int src_stride, uint8_t *dst_a,
		int dst_stride_a, uint8_t *dst_b, int dst_stride_b,
		int width, int height)
{
	dst_a += dst_stride_a * (width - 1);
	dst_b += dst_stride_b * (width - 1);
	dst_stride_a = -dst_stride_a;
	dst_stride_b = -dst_stride_b;

	TransposeUV(src, src_stride, dst_a, dst_stride_a,
			dst_b, dst_stride_b, width, height);
}

// Supported rotation.
enum RotationMode {
	kRotate0 = 0,		// No rotation.
	kRotate90 = 90,		// Rotate 90 degrees clockwise.
	kRotate180 = 180,	// Rotate 180 degrees.
	kRotate270 = 270,	// Rotate 270 degrees clockwise.
	// Deprecated.
	kRotateNone = 0,
	kRotateClockwise = 90,
	kRotateCounterClockwise = 270,
};

int I420Rotate(const uint8_t *src_y, int src_stride_y,
		const uint8_t *src_u, int src_stride_u,
		const uint8_t *src_v, int src_stride_v,
		uint8_t *dst_y, int dst_stride_y, uint8_t *dst_u,
		int dst_stride_u, uint8_t *dst_v, int dst_stride_v,
		int width, int height, enum RotationMode mode)
{
	int halfwidth = (width + 1) >> 1;
	int halfheight = (height + 1) >> 1;

	if (!src_y || !src_u || !src_v || width <= 0 ||
		height == 0 || !dst_y || !dst_u || !dst_v)
		return -1;

	// Negative height means invert the image.
	if (height < 0) {
		height = -height;
		halfheight = (height + 1) >> 1;
		src_y = src_y + (height - 1) * src_stride_y;
		src_u = src_u + (halfheight - 1) * src_stride_u;
		src_v = src_v + (halfheight - 1) * src_stride_v;
		src_stride_y = -src_stride_y;
		src_stride_u = -src_stride_u;
		src_stride_v = -src_stride_v;
	}

	switch (mode) {
	case kRotate90:
		RotatePlane90(src_y, src_stride_y, dst_y,
				dst_stride_y, width, height);
		RotatePlane90(src_u, src_stride_u, dst_u,
			dst_stride_u, halfwidth, halfheight);
		RotatePlane90(src_v, src_stride_v, dst_v,
				dst_stride_v, halfwidth, halfheight);
		return 0;
	case kRotate270:
		RotatePlane270(src_y, src_stride_y, dst_y,
				dst_stride_y, width, height);
		RotatePlane270(src_u, src_stride_u, dst_u,
				dst_stride_u, halfwidth, halfheight);
		RotatePlane270(src_v, src_stride_v, dst_v,
				dst_stride_v, halfwidth, halfheight);
		return 0;
	default:
		break;
	}
	return -1;
}

int I444Rotate(const uint8_t *src_y, int src_stride_y,
		const uint8_t *src_u, int src_stride_u,
		const uint8_t *src_v, int src_stride_v,
		uint8_t *dst_y, int dst_stride_y, uint8_t *dst_u,
		int dst_stride_u, uint8_t *dst_v, int dst_stride_v,
		int width, int height, enum RotationMode mode)
{
	if (!src_y || !src_u || !src_v || width <= 0 ||
			height == 0 || !dst_y || !dst_u || !dst_v)
		return -1;

	// Negative height means invert the image.
	if (height < 0) {
		height = -height;
		src_y = src_y + (height - 1) * src_stride_y;
		src_u = src_u + (height - 1) * src_stride_u;
		src_v = src_v + (height - 1) * src_stride_v;
		src_stride_y = -src_stride_y;
		src_stride_u = -src_stride_u;
		src_stride_v = -src_stride_v;
	}

	switch (mode) {
	case kRotate90:
		RotatePlane90(src_y, src_stride_y, dst_y,
				dst_stride_y, width, height);
		RotatePlane90(src_u, src_stride_u, dst_u,
				dst_stride_u, width, height);
		RotatePlane90(src_v, src_stride_v, dst_v,
				dst_stride_v, width, height);
		return 0;
	case kRotate270:
		RotatePlane270(src_y, src_stride_y, dst_y,
				dst_stride_y, width, height);
		RotatePlane270(src_u, src_stride_u, dst_u,
				dst_stride_u, width, height);
		RotatePlane270(src_v, src_stride_v, dst_v,
				dst_stride_v, width, height);
		return 0;
	default:
		break;
	}
	return -1;
}

int NV12ToI420Rotate(const uint8_t *src_y, int src_stride_y,
		const uint8_t *src_uv, int src_stride_uv,
		uint8_t *dst_y, int dst_stride_y, uint8_t *dst_u,
		int dst_stride_u, uint8_t *dst_v, int dst_stride_v,
		int width, int height, enum RotationMode mode)
{
	int halfwidth = (width + 1) >> 1;
	int halfheight = (height + 1) >> 1;

	if (!src_y || !src_uv || width <= 0 || height == 0 ||
			!dst_y || !dst_u || !dst_v)
		return -1;

	// Negative height means invert the image.
	if (height < 0) {
		height = -height;
		halfheight = (height + 1) >> 1;
		src_y = src_y + (height - 1) * src_stride_y;
		src_uv = src_uv + (halfheight - 1) * src_stride_uv;
		src_stride_y = -src_stride_y;
		src_stride_uv = -src_stride_uv;
	}

	switch (mode) {
	case kRotate90:
		RotatePlane90(src_y, src_stride_y, dst_y,
				dst_stride_y, width, height);
		RotateUV90(src_uv, src_stride_uv, dst_u,
				dst_stride_u, dst_v, dst_stride_v,
				halfwidth, halfheight);
		return 0;
	case kRotate270:
		RotatePlane270(src_y, src_stride_y, dst_y,
				dst_stride_y, width, height);
		RotateUV270(src_uv, src_stride_uv, dst_u, dst_stride_u,
				dst_v, dst_stride_v,
				halfwidth, halfheight);
		return 0;
	default:
		break;
	}
	return -1;
}

// Reads 16 U's and V's and writes out 16 pairs of UV.
void MergeUVRow_NEON(const uint8_t *src_u, const uint8_t *src_v,
		uint8_t *dst_uv, int width)
{
	kernel_neon_begin();
	asm volatile (
		"1:\n"
		"ld1	{v0.16b}, [%0], #16\n"  // load U
		"ld1	{v1.16b}, [%1], #16\n"  // load V
		"subs	%w3, %w3, #16\n"  // 16 processed per loop
		"st2	{v0.16b,v1.16b}, [%2], #32\n"  // store 16 pairs of UV
		"b.gt	1b\n"
		: "+r"(src_u),			// %0
			"+r"(src_v),		// %1
			"+r"(dst_uv),		// %2
			"+r"(width)		// %3// Output registers
		:				// Input registers
		: "cc", "memory", "v0", "v1"	// Clobber List
	);
	kernel_neon_end();
}

void MergeUVRow_C(const uint8_t *src_u, const uint8_t *src_v,
		uint8_t *dst_uv, int width)
{
	int x;

	for (x = 0; x < width - 1; x += 2) {
		dst_uv[0] = src_u[x];
		dst_uv[1] = src_v[x];
		dst_uv[2] = src_u[x + 1];
		dst_uv[3] = src_v[x + 1];
		dst_uv += 4;
	}
	if (width & 1) {
		dst_uv[0] = src_u[width - 1];
		dst_uv[1] = src_v[width - 1];
	}
}

void MergeUVPlane(const uint8_t *src_u, int src_stride_u,
		const uint8_t *src_v, int src_stride_v,
		uint8_t *dst_uv, int dst_stride_uv,
		int width, int height)
{
	int y;
	// Negative height means invert the image.
	if (height < 0) {
		height = -height;
		dst_uv = dst_uv + (height - 1) * dst_stride_uv;
		dst_stride_uv = -dst_stride_uv;
	}
	// Coalesce rows.
	if (src_stride_u == width && src_stride_v == width &&
			dst_stride_uv == width * 2) {
		width *= height;
		height = 1;
		src_stride_u = src_stride_v = dst_stride_uv = 0;
	}

	for (y = 0; y < height; ++y) {
		// Merge a row of U and V into a row of UV.
		MergeUVRow_NEON(src_u, src_v, dst_uv, width);
		// MergeUVRow_C(src_u, src_v, dst_uv, width);
		src_u += src_stride_u;
		src_v += src_stride_v;
		dst_uv += dst_stride_uv;
	}
}
/*
 *void MergeUVRow_C (const uint8_t *src_u, const uint8_t *src_v,
 *		uint8_t *dst_uv, int width)
 *{
 *	int x;
 *	for (x = 0; x < width - 1; x += 2) {
 *		dst_uv[0] = src_u[x];
 *		dst_uv[1] = src_v[x];
 *		dst_uv[2] = src_u[x + 1];
 *		dst_uv[3] = src_v[x + 1];
 *		dst_uv += 4;
 *	}
 *	if (width & 1) {
 *		dst_uv[0] = src_u[width - 1];
 *		dst_uv[1] = src_v[width - 1];
 *	}
 *}
 */
#if 0
static int __init rotation_neon64_init(void)
{
	struct file *fp;
	mm_segment_t fs;
	loff_t pos;
	char *file_buf = NULL;
	char *file_buf_out = NULL;
	int file_size = 0;

	int width = 720;
	int height = 1280;

	struct timeval tv_s;
	struct timeval tv_e;
	int time_cost = 0;

	fp = filp_open("/userdata/libyuv/test.yuv", O_RDWR | O_CREAT, 0644);
	if (IS_ERR(fp)) {
		pr_err("create file error/n");
		return -1;
	}

	pr_info("file size %d %d\n",
			fp->f_inode->i_blocks, fp->f_inode->i_size);

	file_size = fp->f_inode->i_size;

	file_buf = kmalloc(file_size, GFP_KERNEL);
	file_buf_out = kmalloc(file_size, GFP_KERNEL);

	pos = 0;
	kernel_read(fp, file_buf, file_size, &pos);
	pr_info("pos %d\n", pos);
	filp_close(fp, NULL);
	do_gettimeofday(&tv_s);

#if 0
I420Rotate(file_buf, width, file_buf+width*height, width/2,
		file_buf+width*height*5/4, width/2,
		file_buf_out, height,
		file_buf_out+width*height, height/2,
		file_buf_out+width*height*5/4, height/2,
		width, height, kRotate90);
#else
NV12ToI420Rotate(file_buf, width, file_buf+width*height,
		width, file_buf_out, height,
		file_buf_out+width*height, height/2,
		file_buf_out+width*height*5/4, height/2,
		width, height, kRotate90);
#endif
	do_gettimeofday(&tv_e);

	time_cost = (tv_e.tv_sec*1000 + tv_e.tv_usec/1000) -
		(tv_s.tv_sec*1000 + tv_s.tv_usec/1000);
	pr_info("time cost %dms\n", time_cost);

	fp = filp_open("/userdata/libyuv/test-out.yuv", O_RDWR | O_CREAT, 0644);
	if (IS_ERR(fp)) {
		pr_err("create file error/n");
		return -1;
	}

	pos = 0;
	kernel_write(fp, file_buf_out, file_size, &pos);
	pr_info("pos %d\n", pos);
	filp_close(fp, NULL);

	kfree(file_buf);
	kfree(file_buf_out);

	return 0;
}

static void __exit rotation_neon64_exit(void)
{
}

module_init(rotation_neon64_init);
module_exit(rotation_neon64_exit);
#endif

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Libyuv rotation neon64");
