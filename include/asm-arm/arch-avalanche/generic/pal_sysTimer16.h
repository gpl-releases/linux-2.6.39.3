/*
 *
 * pal_sysTimer16.h
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
 * FILE PURPOSE:    16 bit Timer Module Header
 ********************************************************************************
 * FILE NAME:       timer16.h
 *
 * DESCRIPTION:     Platform and OS independent API for 16 bit timer module
 *
 * REVISION HISTORY:
 * 27 Nov 02 - PSP TII  
 * 
 *******************************************************************************/

#ifndef __TIMER16_H__
#define __TIMER16_H__

/****************************************************************************
 * Type:        PAL_SYS_TIMER16_STRUCT_T
 ****************************************************************************
 * Description: This type defines the hardware configuration of the timer
 *              
 ***************************************************************************/
typedef struct PAL_SYS_TIMER16_STRUCT_tag
{
    UINT32 ctrl_reg;
    UINT32 load_reg;
    UINT32 count_reg;
    UINT32 intr_reg;
} PAL_SYS_TIMER16_STRUCT_T;


/****************************************************************************
 * Type:        PAL_SYS_TIMER16_MODE_T
 ****************************************************************************
 * Description: This type defines different timer modes. 
 *              
 ***************************************************************************/
typedef enum PAL_SYS_TIMER16_MODE_tag
{
    TIMER16_CNTRL_ONESHOT  = 0,
    TIMER16_CNTRL_AUTOLOAD = 2
} PAL_SYS_TIMER16_MODE_T;



/****************************************************************************
 * Type:        PAL_SYS_TIMER16_CTRL_T
 ****************************************************************************
 * Description: This type defines start and stop values for the timer. 
 *              
 ***************************************************************************/
typedef enum PAL_SYS_TIMER16_CTRL_tag
{
    TIMER16_CTRL_STOP = 0,
    TIMER16_CTRL_START
} PAL_SYS_TIMER16_CTRL_T ;


void PAL_sysTimer16GetFreqRange(UINT32  refclk_freq,
                            UINT32 *p_max_usec,
                            UINT32 *p_min_usec);                                

INT32 PAL_sysTimer16SetParams(UINT32 base_address,
                         UINT32 refclk_freq,
                         PAL_SYS_TIMER16_MODE_T mode,
                         UINT32 usec);

void PAL_sysTimer16Ctrl(UINT32 base_address, PAL_SYS_TIMER16_CTRL_T status);

#endif /* __TIMER16_H__ */
