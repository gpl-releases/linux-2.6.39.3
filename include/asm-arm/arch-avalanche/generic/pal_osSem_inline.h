/*
 *
 * pal_osSem_inline.h
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


/** \file   pal_osSem_inline.h
    \brief  OsSEM Services Source File

    
    \author     PSP Architecture Team
    \version    1.0
 */

#ifndef __PAL_OSSEM_INLINE_H__
#define __PAL_OSSEM_INLINE_H__

#include "pal_os.h"
#include "pal_defs.h"
#include "pal_osCfg.h"
#include <mach/semaphore.h>
#include <linux/slab.h>

/**
 * \defgroup PalOSSem PAL OS Semaphore Interface
 * 
 * PAL OS Semaphore Interface
 * @{
 */

/** \name PAL OS Semaphore Interface
 *  PAL OS Semaphore Interface
 * @{
 */

/**
 * \brief   PAL OS Semaphore Init
 * Nothing to init for Linux
 */
PAL_INLINE PAL_Result PAL_osSemInit(Ptr param)
{
    return PAL_SOK;
}

/**
 * \brief   PAL OS Semaphore Create
 */
PAL_INLINE PAL_Result PAL_osSemCreate(const char* name,
                                  Int32 initVal,
                                  PAL_OsSemAttrs *attrs, 
                                  PAL_OsSemHandle* hSem)
{
    if ((*hSem = kmalloc(sizeof(struct semaphore), GFP_KERNEL)) == NULL) 
	{
        return PAL_OS_ERROR_NO_RESOURCES;
	}

    sema_init((struct semaphore*)*hSem, initVal); 
    return PAL_SOK;
}

/**
 * \brief   PAL OS Semaphore Delete
 */
PAL_INLINE PAL_Result PAL_osSemDelete(PAL_OsSemHandle hSem)
{
    kfree(hSem);
    return PAL_SOK;    
}

/**
 * \brief   PAL OS Semaphore Take
 */

PAL_INLINE PAL_Result PAL_osSemTake(PAL_OsSemHandle hSem, Int32 mSecTimeout)
{
	if (mSecTimeout != PAL_OSSEM_NO_TIMEOUT)
		return PAL_OS_ERROR_NOT_SUPPORTED;
				
    down_interruptible((struct semaphore*)hSem);
    return PAL_SOK;
}

/**
 * \brief   PAL OS Semaphore Give
 */
PAL_INLINE PAL_Result PAL_osSemGive(PAL_OsSemHandle hSem)
{
    up((struct semaphore*)hSem);
    return PAL_SOK;
}

/**
 * \brief   PAL OS Semaphore Report
 */
PAL_INLINE PAL_Result PAL_osSemReport(PAL_OsSemHandle hSem, PAL_OsSemReport *report, Char* buf)
{
    return PAL_OS_ERROR_NOT_SUPPORTED;
}

/*@}*/
/*@}*/
#endif

