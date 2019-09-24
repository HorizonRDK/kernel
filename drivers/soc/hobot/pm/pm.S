
/*
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */
#include <linux/linkage.h>
#include "x2_pm.h"

ENTRY(__asm_dcache_level)
	lsl	x12, x0, #1
	msr	csselr_el1, x12		/* select cache level */
	isb				/* sync change of cssidr_el1 */
	mrs	x6, ccsidr_el1		/* read the new cssidr_el1 */
	and	x2, x6, #7		/* x2 <- log2(cache line size)-4 */
	add	x2, x2, #4		/* x2 <- log2(cache line size) */
	mov	x3, #0x3ff
	and	x3, x3, x6, lsr #3	/* x3 <- max number of #ways */
	clz	w5, w3			/* bit position of #ways */
	mov	x4, #0x7fff
	and	x4, x4, x6, lsr #13	/* x4 <- max number of #sets */
	/* x12 <- cache level << 1 */
	/* x2 <- line length offset */
	/* x3 <- number of cache ways - 1 */
	/* x4 <- number of cache sets - 1 */
	/* x5 <- bit position of #ways */

loop_set:
	mov	x6, x3			/* x6 <- working copy of #ways */
loop_way:
	lsl	x7, x6, x5
	orr	x9, x12, x7		/* map way and level to cisw value */
	lsl	x7, x4, x2
	orr	x9, x9, x7		/* map set number to cisw value */
	tbz	w1, #0, 1f
	dc	isw, x9
	b	2f
1:	dc	cisw, x9		/* clean & invalidate by set/way */
2:	subs	x6, x6, #1		/* decrement the way */
	b.ge	loop_way
	subs	x4, x4, #1		/* decrement the set */
	b.ge	loop_set

	ret
ENDPROC(__asm_dcache_level)

/*
 * void __asm_flush_dcache_all(int invalidate_only)
 *
 * x0: 0 clean & invalidate, 1 invalidate only
 *
 * flush or invalidate all data cache by SET/WAY.
 */
ENTRY(__asm_dcache_all)
	mov	x1, x0
	dsb	sy
	mrs	x10, clidr_el1		/* read clidr_el1 */
	lsr	x11, x10, #24
	and	x11, x11, #0x7		/* x11 <- loc */
	cbz	x11, finished		/* if loc is 0, exit */
	mov	x15, x30
	mov	x0, #0			/* start flush at cache level 0 */
	/* x0  <- cache level */
	/* x10 <- clidr_el1 */
	/* x11 <- loc */
	/* x15 <- return address */

loop_level:
	lsl	x12, x0, #1
	add	x12, x12, x0		/* x0 <- tripled cache level */
	lsr	x12, x10, x12
	and	x12, x12, #7		/* x12 <- cache type */
	cmp	x12, #2
	b.lt	skip			/* skip if no cache or icache */
	bl	__asm_dcache_level	/* x1 = 0 flush, 1 invalidate */
skip:
	add	x0, x0, #1		/* increment cache level */
	cmp	x11, x0
	b.gt	loop_level

	mov	x0, #0
	msr	csselr_el1, x0		/* restore csselr_el1 */
	dsb	sy
	isb
	mov	x30, x15

finished:
	ret
ENDPROC(__asm_dcache_all)

ENTRY(__asm_flush_dcache_all)
	mov	x0, #0
	b	__asm_dcache_all
ENDPROC(__asm_flush_dcache_all)
	.text
/*
 * void x2_suspend_sram_fn(struct x2_suspend_data*)
 * @input param:
 * 	@r0: base address of struct x2_suspend_data
 */
/* x2_suspend_in_sram must be 8-byte aligned per the requirements of fncpy() */
	.align 3
ENTRY(x2_suspend_in_sram)
	isb
	ldr	x10, [x0, #32]
	ldp	x4, x1, [x0]
	ldp	x2, x6, [x0, #16]

	ldr	x15, [x1]
	ldr	x15, [x4]
	ldr	x15, [x6]
	ldr	x15, [x2]
	ldr	x15, [x10]

	add	x3, x2, #0x340, lsl #12
	ldr	x15, [x3]

	add	x5, x2, #0x300, lsl #12
	add	x5, x5, #0x200
	ldr	x15, [x5]

	add	x5, x2, #0x240, lsl #12
	add	x5, x5, #0xa0
	ldr	x15, [x5]
	isb

	//writel(0x0, ddrc + DDRC_PCTRL_0);
	add	x0, x1, #0x490
	str	wzr, [x0]
	//writel(0x0, ddrc + DDRC_PCTRL_1);
	dsb	st
	add	x0, x1, #0x540
	str	wzr, [x0]
	//writel(0x0, ddrc + DDRC_PCTRL_2);
	dsb	st
	add	x0, x1, #0x5f0
	str	wzr, [x0]
	//writel(0x0, ddrc + DDRC_PCTRL_3);
	dsb	st
	add	x0, x1, #0x6a0
	str	wzr, [x0]
	//writel(0x0, ddrc + DDRC_PCTRL_4);
	dsb	st
	add	x0, x1, #0x750
	str	wzr, [x0]
	//writel(0x0, ddrc + DDRC_PCTRL_5);
	dsb	st
	add	x0, x1, #0x800
	str	wzr, [x0]
	dsb	st
	add	x3, x1, #0x3fc
	b	2f
	nop
1:	yield
2: 	ldr	w0, [x3]
	dsb	ld
	cbnz	w0, 1b
	add	x3, x1, #0x20
	ldr	w0, [x3]
	and	w0, w0, #0xfffffffe
	str	w0, [x3]
	dsb	st
	mov	w0, #0x2                   	// #2
	add	x3, x1, #0x304
	str	w0, [x3]
	dsb	st
	add	x3, x1, #0x308
	b	4f
3:	yield
4:	ldr	w0, [x3]
	dsb	ld
	tst	w0, #0x60000000
	b.eq	3b   // b.none
	add	x3, x1, #0x198
	ldr	w0, [x3]
	mov	w5, #0xfefe                	// #65278
	movk	w5, #0xfffe, lsl #16
	and	w0, w0, w5
	str	w0, [x3]
	dsb	st
	add	x3, x1, #0x19c
	ldr	w0, [x3]
	and	w0, w0, #0xfffffffe
	str	w0, [x3]
	dsb	st
	add	x3, x1, #0x1bc
	b	6f
	nop
5:	yield
6:	ldr	w0, [x3]
	dsb	ld
	tbnz	w0, #1, 5b
	add	x3, x1, #0x1c4
	ldr	w0, [x3]
	and	w0, w0, #0xfffffffe
	str	w0, [x3]
	dsb	st
	add	x3, x1, #0x30
	ldr	w0, [x3]
	orr	w0, w0, #0x20
	str	w0, [x3]
	dsb	st
	add	x3, x1, #0x4
	b	8f
7:	yield
8:	ldr	w0, [x3]
	dsb	ld
	and	w0, w0, #0x7
	cmp	w0, #0x3
	b.ne	7b
	add	x3, x1, #0x320
	str	wzr, [x3]
	dsb	st
	add	x5, x1, #0x1b0
	ldr	w0, [x5]
	and	w0, w0, #0xfffffffe
	str	w0, [x5]
	dsb	st
	mov	w0, #0x1                   	// #1
	str	w0, [x3]
	dsb	st
	add	x0, x1, #0x324
	b	10f
9:     yield
10:    ldr	w1, [x0]

	dsb	ld
	tbz	w1, #0, 9b

	str	wzr, [x3]
	dsb	st
	ldr	w1, [x5]
	mov	w7, #0x1f20                	// #7968
	orr	w1, w1, w7
	str	w1, [x5]
	dsb	st
	mov	w1, #0x1                   	// #1
	str	w1, [x3]
	dsb	st
	b	12f
11:	yield
12:	ldr	w1, [x0]
	dsb	ld
	tbz	w1, #0, 11b

	add	x3, x2, #0x340, lsl #12
	str	wzr, [x3]
	dsb	st

	add	x5, x2, #0x300, lsl #12
	mov	w0, #0x3                   	// #3
	add	x5, x5, #0x200
	str	w0, [x5]
	dsb	st

	add	x1, x2, #0x240, lsl #12
	add	x1, x1, #0xa0
	b	14f
	nop
13:	yield
14:	ldr	w0, [x1]
	dsb	ld
	tbz	w0, #0, 13b
	mov	w0, #0x2                   	// #2
	str	w0, [x5]
	dsb	st
	mov	w0, #0x1                   	// #1
	str	w0, [x3]
	dsb	st

        add	x1, x4, #0x14
	str	wzr, [x1]
	dsb	st
	add	x1, x6, #0x138
	str	w0, [x1]
	dsb	st

	add     x0, x10, #0x70
	ldr     w3, [x0]
	and     w3, w3, #0xffff3fff
	str     w3, [x0]
	dsb	st

/*
*	add	x2, x4, #0xc
*	str	wzr, [x2]
*	dsb	st
*	mov	w0, #0x1
*	add	x4, x4, #0x4
*	str	w0, [x4]
*	dsb	st
*/

pen:	wfi
	b	pen
	nop
ENDPROC(x2_suspend_in_sram)

ENTRY(x2_suspend_in_sram_sz)
	.word .-x2_suspend_in_sram
