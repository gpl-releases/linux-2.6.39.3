/*
 * pal_osTimer_inline.h
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


/** \file   pal_osTimer_inline.h
    \brief  OsTIMER Services Source File

    This file implements the OsTIMER services for Linux.


    \author     PSP Architecture Team
    \version    0.1
*/

#ifndef __PAL_OSTIMER_INLINE_H__
#define __PAL_OSTIMER_INLINE_H__

#include "pal_os.h"
#include "pal_defs.h"
#include <linux/timer.h>

/**
 * \defgroup PalOSTimer PAL OS Timer Interface
 * 
 * PAL OS Timer Interface
 * @{
 */

/** \name PAL OS Timer Interface
 *  PAL OS Timer Interface
 * @{
 */

/**
 * \brief   PAL OS Timer Create 
 */
PAL_INLINE PAL_Result PAL_osTimerCreate(PAL_OsTimerFunc pfn, 
                                Uint32 arg, 
                                PAL_OsTimerHandle* phTimer)
{
    if ((*phTimer =  kmalloc(sizeof(struct timer_list),GFP_KERNEL)) == NULL)
	{
        return PAL_OS_ERROR_NO_RESOURCES;
	}

    setup_timer((struct timer_list *) *phTimer, pfn, arg);
    return PAL_SOK;
}

/**
 * \brief   PAL OS Timer Destroy
 */
PAL_INLINE PAL_Result PAL_osTimerDestroy(PAL_OsTimerHandle hTimer)
{
    del_timer((struct timer_list*) hTimer); 
    kfree(hTimer);
    return PAL_SOK;
}

/**
 * \brief   PAL OS Timer Activate
 */
PAL_INLINE PAL_Result PAL_osTimerStart(PAL_OsTimerHandle hTimer, Uint32 msec)
{
    mod_timer((struct timer_list*) hTimer, msecs_to_jiffies(msec) + jiffies); 
    return 0;
}

/**
 * \brief   PAL OS Timer Deactivate
 */
PAL_INLINE PAL_Result PAL_osTimerStop(PAL_OsTimerHandle hTimer)
{
    del_timer((struct timer_list*) hTimer); 
    return 0;
}

/*@}*/
/*@}*/
#endif /* !__PAL_OSTIMER_INLINE_H__ */ 

