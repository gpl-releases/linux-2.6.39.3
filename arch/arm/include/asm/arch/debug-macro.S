/* linux/include/asm-arm/arch-puma5/debug-macro.S
 *
 * Debugging macro include header
 *
 *  Copyright (C) 1994-1999 Russell King
 *  Moved and updated from linux/arch/arm/kernel/debug.S by Steve Chen
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
*/

/* Copyright 2008, Texas Instruments Incorporated
 *
 * This program has been modified from its original operation by Texas Instruments
 * to do the following:
 * Explanation of modification.
 *  used avalanche base address macros
 *
 *
 * THIS MODIFIED SOFTWARE AND DOCUMENTATION ARE PROVIDED
 * "AS IS," AND TEXAS INSTRUMENTS MAKES NO REPRESENTATIONS
 * OR WARRENTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO, WARRANTIES OF MERCHANTABILITY OR FITNESS FOR ANY
 * PARTICULAR PURPOSE OR THAT THE USE OF THE SOFTWARE OR
 * DOCUMENTATION WILL NOT INFRINGE ANY THIRD PARTY PATENTS,
 * COPYRIGHTS, TRADEMARKS OR OTHER RIGHTS.
 * See The GNU General Public License for more details.
 *
 * These changes are covered under version 2 of the GNU General Public License,
 * dated June 1991.
*/

#include <mach/hardware.h>

		.macro	addruart,rx,tmp
		mrc	p15, 0, \rx, c1, c0
		tst	\rx, #1			@ MMU enabled?
		moveq	\rx, #IO_START		@ physical base address
		movne	\rx, #IO_RELOC_ADDR	@ virtual base
		orr	\rx, #IO_START		@ virtual base
		orr	\rx, \rx, #0x08600000	@ UART 0
		orr	\rx, \rx, #0x00010000	@ UART 0
#if (CONFIG_AVALANCHE_CONSOLE_PORT > 0)
		orr	\rx, \rx, #0x00000f00	@ UART 1
#else
		orr	\rx, \rx, #0x00000e00	@ UART 0
#endif

#if defined(CONFIG_CPU_BIG_ENDIAN)
		orr	\rx, \rx, #0x00000003	@ be32
#endif

		.endm

		.macro	senduart,rd,rx
		strb	\rd, [\rx]
		.endm


        .macro  busyuart,rd,rx
1001:   ldrb \rd, [\rx, #0x14]  /* 0x5 << 2 (register size is 4 byte) = 0x14 */
		and \rd, \rd, #0x20 /* check for THRE (Transmit Hold Register Empty) bit */
        teq \rd, #0x20  
        bne 1001b
        .endm

#if 0 /* not used */
		.macro	busyuart,rd,rx
		mov	\rd, #0
1001:		add	\rd, \rd, #1
		teq	\rd, #0x10000 
		bne	1001b
		.endm
#endif

		.macro	waituart,rd,rx
		.endm
