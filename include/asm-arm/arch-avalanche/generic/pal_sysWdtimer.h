/*
 *
 * pal_sysWdTimer.h 
 * Description:
 * see below
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

/******************************************************************************
 * FILE PURPOSE:    Watchdog Timer Module Header
 ********************************************************************************
 * FILE NAME:       wdtimer.h
 *
 * DESCRIPTION:     Platform and OS independent API for watchdog timer module
 *
 * REVISION HISTORY:
 * 27 Nov 02 - PSP TII  
 * 
 *******************************************************************************/


#ifndef __WDTIMER_H__
#define __WDTIMER_H__

/* Return Codes */
#define WDTIMER_RET_OK			0
#define WDTIMER_RET_ERR			-1
#define WDTIMER_ERR_INVAL		-2

/****************************************************************************
 * Type:        PLA_SYS_WDTIMER_STRUCT_T
 ****************************************************************************
 * Description: This type defines the hardware configuration of the 
 *              watchdog timer
 ***************************************************************************/
typedef struct PAL_SYS_WDTIMER_STRUCT_tag
{
    UINT32  kick_lock;
    UINT32  kick;
    UINT32  change_lock;
    UINT32  change ; 
    UINT32  disable_lock;
    UINT32  disable;
    UINT32  prescale_lock;
    UINT32  prescale;
} PAL_SYS_WDTIMER_STRUCT_T;


/****************************************************************************
 * Type:        PAL_SYS_WDTIMER_CTRL_T
 ****************************************************************************
 * Description: This type defines start and stop values for the timer. 
 *              
 ***************************************************************************/
typedef enum PAL_SYS_WDTIMER_CTRL_tag
{
    WDTIMER_CTRL_DISABLE = 0,
    WDTIMER_CTRL_ENABLE
} PAL_SYS_WDTIMER_CTRL_T ;

void PAL_sysWdtimerInit(UINT32 base_addr, UINT32 clk_freq);
INT32 PAL_sysWdtimerSetPeriod( UINT32 msec );
INT32 PAL_sysWdtimerCtrl(PAL_SYS_WDTIMER_CTRL_T wd_ctrl);
INT32 PAL_sysWdtimerKick(void);

#endif /* __WDTIMER_H__ */
