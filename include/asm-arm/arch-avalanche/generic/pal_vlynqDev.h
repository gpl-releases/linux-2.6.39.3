/*
 * pal_vlynqDev.h
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
 * FILE PURPOSE:    PAL VLYNQ Device Header File
 ******************************************************************************
 * FILE NAME:       pal_vlynqDev.h
 *
 * DESCRIPTION:     PAL VLYNQ Device Header File
 *
 *******************************************************************************/

#ifndef __PAL_VLYNQ_DEV_H__
#define __PAL_VLYNQ_DEV_H__

/* The vlynq dev handle. */
typedef void PAL_VLYNQ_DEV_HND;

#include "pal_vlynq.h"

#define PAL_VLYNQ_EVENT_LOCAL_ERROR 0
#define PAL_VLYNQ_EVENT_PEER_ERROR  1

#define PAL_VLYNQ_DEV_ADD_IRQ       0
#define PAL_VLYNQ_DEV_REMOVE_IRQ    1

typedef Int32 (*PAL_VLYNQ_DEV_CB_FN)(void*, Uint32 condition, Uint32 value);

PAL_Result PAL_vlynqDevCbRegister(PAL_VLYNQ_DEV_HND   *vlynq_dev, 
	   	                     PAL_VLYNQ_DEV_CB_FN cb_fn,
				     void                *this_driver);

PAL_Result PAL_vlynqDevCbUnregister(PAL_VLYNQ_DEV_HND *vlynq_dev,
     		                       void              *this_driver);

PAL_VLYNQ_DEV_HND *PAL_vlynqDevFind(char *name, Uint8 instance);

PAL_VLYNQ_HND *PAL_vlynqDevGetVlynq(PAL_VLYNQ_DEV_HND *vlynq_dev);

PAL_Result PAL_vlynqDevFindIrq(PAL_VLYNQ_DEV_HND *vlynq_dev, Uint8 *irq, 
		                  Uint32 num_irq);

PAL_Result PAL_vlynqDevGetResetBit(PAL_VLYNQ_DEV_HND *vlynq_dev, 
                                   Uint32 *reset_bit);
           
PAL_VLYNQ_DEV_HND* PAL_vlynqDevCreate(PAL_VLYNQ_HND *vlynq, char *name,
					Uint32 instance, Int32 reset_bit,
					Bool peer);

/* Protected API(s) to be used only by pal_vlynq.c */
PAL_Result pal_vlynq_dev_init(void);

PAL_Result pal_vlynq_dev_handle_event(PAL_VLYNQ_DEV_HND *vlynq_dev,
                                       Uint32 cmd, Uint32 val);

PAL_Result pal_vlynq_dev_ioctl(PAL_VLYNQ_DEV_HND *vlynq_dev, Uint32 cmd, 
		               Uint32 cmd_val);

PAL_Result PAL_vlynqDevDestroy(PAL_VLYNQ_DEV_HND *vlynq_dev);

#endif
