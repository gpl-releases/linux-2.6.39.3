/*
 *
 * pal_osBuf_inline.h
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


/** \file   pal_osBuf_inline.h
    \brief  OsBUF Services Header File

    This file defines OS abstraction services for fixed length buffer
    memory management operations. The PAL module implementing these APIs
    is called OsBUF.

    In this implementaion of PAL OS for Linux, the fixed size buffer pool
    is implemented using the linux slab allocator. The slab allocator 
    allows creation of "slab caches" of fixed size. There is however no
    restriction on the number of buffers that can be allocated from a 
    particular cache. The slab allocator will attempt to fulfill a request
    for a buffer till the time the system has memory available.

 
    \author     PSP Architecture Team
    \version    1.0
 */

#ifndef __PAL_OSBUF_INLINE_H__
#define __PAL_OSBUF_INLINE_H__

#include "pal_os.h"
#include "pal_defs.h"
#include "pal_osCfg.h"
#include <asm/page.h>
#include <linux/slab.h>
#include <linux/mm.h>

typedef struct kmem_cache kmem_cache_t;

/**
 * \defgroup PalOSBuf PAL OS Buffer Interface
 * 
 * PAL OS Buffer Interface
 * @{
 */

/**
 * \brief   PAL OS Buffer Init.
 * Linux implementation leaves this as an empty function. The slab allocator
 * is initialized by the kernel itself. Nothing to do here.
 */
PAL_INLINE PAL_Result PAL_osBufInit(Ptr param)
{
   return PAL_SOK;
}


/**
 * \brief   PAL OS Buffer Pool Define
 * 
 * Only segment Id 0 is recognized. Please refer to documentaion in OsMEM 
 * module.
 * The parameter maxBufs (maximum number of buffers that can be allocated) is 
 * not honored. The allocate function will keep returning memory till it is
 * available. This constraint is because of the way slab allocator is
 * implemented in linux.
 * Alignment to Hardware cache line is only supported. No other alignment
 * is guaranteed. If the user asks for any alignment 
 * (by passing a non-NULL attrs) hardware cache alignment is assumed.
 */
PAL_INLINE PAL_Result PAL_osBufPoolDefine(   const char* name,
                                         Uint32 segId,
                                         Uint maxBufs,
                                         Uint32 bytesPerBuf,
                                         PAL_OsBufAttrs *attrs,
                                         Uint32 *pBufPoolId)
{
    Uint flags = 0;
        
    if(segId != 0)
	{
        return PAL_OS_ERROR_INVALID_PARAM;
	}

    /* if user asks for some alignment, give him Hardware cache alignmen.
     * This is the only alignment supported by Linux kernel 
     */
    if(attrs)
	{
        flags = SLAB_HWCACHE_ALIGN;
	}
		
    *pBufPoolId = (Uint32) kmem_cache_create(name, bytesPerBuf, 0, flags, 
                                             NULL);
            
    if(!(*pBufPoolId)) 
	{
        return PAL_OS_ERROR_NO_RESOURCES;
	}
		
    return PAL_SOK;
}

/**
 * \brief   PAL OS Buffer Pool Undefine
 */
PAL_INLINE PAL_Result PAL_osBufPoolUndefine (Uint32 bufPoolId)
{
    if(!bufPoolId)
	{
		return PAL_OS_ERROR_INVALID_PARAM;
	}
	
   kmem_cache_destroy((kmem_cache_t*)bufPoolId);	
   return PAL_SOK;
}

/**
 * \brief   PAL OS Buffer Alloc
 */
PAL_INLINE PAL_Result PAL_osBufAlloc(Uint32 bufPoolId, Ptr *bufAddr)
{
    *bufAddr = kmem_cache_alloc((kmem_cache_t*)bufPoolId, GFP_KERNEL);

    if(*bufAddr == NULL)
	{
        return PAL_OS_ERROR_NO_RESOURCES;
	}

    return PAL_SOK;
}

/**
 * \brief   PAL OS Buffer Free
 */
PAL_INLINE PAL_Result PAL_osBufFree(Uint32 bufPoolId, Ptr bufAddr)
{
    kmem_cache_free((kmem_cache_t*)bufPoolId, bufAddr);
   
    return PAL_SOK;
}


/**
 * \brief   PAL OS Buffer Report
 */
PAL_INLINE PAL_Result PAL_osBufReport(Uint32 bufPoolId, PAL_OsBufReport *report, Char* buf)
{
   return PAL_OS_ERROR_NOT_SUPPORTED;
}
/*@}*/
/*@}*/
#endif

