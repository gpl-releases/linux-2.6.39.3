/*
 *
 * pal_vlynqOs.h 
 * Description:
 *
 *
 *
 * Copyright (C) 2008, Texas Instruments, Incorporated
 *
 *  This program is free software; you can distribute it and/or modify it
 *  under the terms of the GNU General Public License (Version 2) as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 *  for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
 *
 */

#ifndef __PAL_VLYNQ_OS_H__
#define __PAL_VLYNQ_OS_H__

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <asm/string.h>
#include <asm/arch/generic/pal.h>

/* To keep the ISR dispatch simple we align the signature of the ISR
 * to that stipulated by the OS.
 */
typedef irqreturn_t (*PAL_VLYNQ_DEV_ISR_FN)(int, void*, struct pt_regs*);
typedef struct pal_vlynq_dev_isr_param_grp_t
{
    int  arg1;
    void *arg2;
    void *arg3;

} PAL_VLYNQ_DEV_ISR_PARAM_GRP_T;

#define PAL_VLYNQ_DEV_ISR_PARM_NUM 3

#ifdef AVALANCHE_LOW_VLYNQ_CONTROL_BASE
#define LOW_VLYNQ_COUNT 1
#else
#define LOW_VLYNQ_COUNT 0
#endif

#ifdef AVALANCHE_HIGH_VLYNQ_CONTROL_BASE
#define HIGH_VLYNQ_COUNT 1
#else
#define HIGH_VLYNQ_COUNT 0
#endif

#define MAX_ROOT_VLYNQ (LOW_VLYNQ_COUNT + HIGH_VLYNQ_COUNT)

/* Board dependent, to be moved to board configuration, but later. */
#define MAX_VLYNQ_COUNT 4 

#endif
