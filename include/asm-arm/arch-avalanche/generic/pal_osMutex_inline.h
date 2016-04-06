/*
 *
 * pal_osMutex_inline.h
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


/** \file   pal_osMutex_inline.h
    \brief  OsMUTEX Services Source File

    This file declares OS abstraction services for mutually exclusive
    locks or binary semaphores. All services run in the context of the
    calling thread or program. OsMUTEX does not spawn a thread of its own 
    to implement the APIs declared here.

 
    \author     PSP Architecture Team
    \version    1.0
 */

#ifndef __PAL_OSMUTEX_INLINE_H__
#define __PAL_OSMUTEX_INLINE_H__

#include "pal_os.h"
#include "pal_defs.h"
#include "pal_osCfg.h"
#include <mach/semaphore.h>
#include <linux/slab.h>

/**
 * \defgroup PalOSMutex PAL OS Mutex Interface
 * 
 * PAL OS Mutex Interface
 * \{
 */

/** \name PAL OS Mutex Interface
 *  PAL OS Mutex Interface
 * \{
 */

/**
 * \brief   PAL OS Mutex Init
 * Nothing to initialize for Linux.
 */
PAL_INLINE PAL_Result PAL_osMutexInit(Ptr param)
{
	return PAL_SOK;
}

/**
 * \brief   PAL OS Mutex Create
 */
PAL_INLINE PAL_Result PAL_osMutexCreate(	const Char* name,
				PAL_OsMutexAttrs *attrs, 
				PAL_OsMutexHandle* hMutex)
{
    *hMutex = kmalloc(sizeof(struct semaphore), GFP_KERNEL);
    if(*hMutex == NULL) 
	{
        return PAL_OS_ERROR_NO_RESOURCES;
	}

    sema_init((struct semaphore *)*hMutex,1);
    
    return PAL_SOK;
}

/**
 * \brief   PAL OS Mutex Delete
 * This does not care about any pending threads. The onus is 
 * on the user.
 */
PAL_INLINE PAL_Result PAL_osMutexDelete(PAL_OsMutexHandle hMutex)
{
    kfree(hMutex);
    return PAL_SOK;
}

/**
 * \brief   PAL OS Mutex Lock
 * The wait should always be infinite for Linux else error is returned.
 */
PAL_INLINE PAL_Result PAL_osMutexLock(PAL_OsMutexHandle hMutex, Int32 mSecTimeout)
{
	if(mSecTimeout != PAL_OSMUTEX_NO_TIMEOUT)
		return PAL_OS_ERROR_NOT_SUPPORTED;
	
    down_interruptible((struct semaphore*)hMutex);
    return PAL_SOK;
}

/**
 * \brief   PAL OS Mutex Unlock
 */
PAL_INLINE PAL_Result PAL_osMutexUnlock(PAL_OsMutexHandle hMutex)
{
    up((struct semaphore*)hMutex);
    return PAL_SOK;
}

/**
 * \brief   PAL OS Mutex Report
 * No reporting supported by the kernel.
 */
PAL_INLINE PAL_Result PAL_osMutexReport(PAL_OsMutexHandle hMutex, PAL_OsMutexReport *report, Char* buf)
{
    return PAL_OS_ERROR_NOT_SUPPORTED;
}
		
/*\}*/
/*\}*/
#endif

