/*
 * mcdma.h
 * Description:
 * See below.
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
 */

/******************************************************************************
 * FILE PURPOSE:    Multi Channel Direct Memory Access (MC-DMA) Header
 ********************************************************************************
 * FILE NAME:       mcdma.h
 *
 * DESCRIPTION:     Platform and OS independent API for MCDMA Controller
 *
 * REVISION HISTORY:
 * 27 Nov 02 - PSP TII  
 *
 *******************************************************************************/

#ifndef __MCDMA_H__
#define __MCDMA_H__


typedef enum MCDMA_CHANNEL_tag
{
    MCDMA_CHANNEL_0 = 0,
    MCDMA_CHANNEL_1,
    MCDMA_CHANNEL_2,
    MCDMA_CHANNEL_3,
    
} MCDMA_CHANNEL_T;

typedef enum MCDMA_CTRL_tag
{
    MCDMA_STOP = 0,
    MCDMA_START
    
} MCDMA_CTRL_T;

typedef enum MCDMA_ADDR_MODE_tag
{
    MCDMA_INCREMENTING = 0,
    MCDMA_FIXED = 2
    
} MCDMA_ADDR_MODE_T;

typedef enum MCDMA_BURST_MODE_tag
{
    MCDMA_1_WORD_BURST = 0,
    MCDMA_2_WORD_BURST = 1,
    MCDMA_4_WORD_BURST = 2,
    
} MCDMA_BURST_MODE_T;


void mcdma_init(UINT32 base_addr);
void mcdma_control(MCDMA_CHANNEL_T mcdma_ch, MCDMA_CTRL_T mcdma_ctrl);
INT32 mcdma_setdmaparams (MCDMA_CHANNEL_T mcdma_ch,
                          UINT32 src_addr, 
                          UINT32 dst_addr, 
                          UINT32 length, 
                          MCDMA_BURST_MODE_T burst_mode, 
                          MCDMA_ADDR_MODE_T src_addr_mode,
                          MCDMA_ADDR_MODE_T dst_addr_mode);

#endif /* __MCDMA_H__ */
