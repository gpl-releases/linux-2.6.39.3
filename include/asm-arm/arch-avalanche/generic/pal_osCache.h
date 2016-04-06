/*
 *
 * pal_osCache.h
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


/** \file   pal_osCache.h
    \brief  OsCACHE Services Header File
    

    \author     PSP Architecture Team
    \version    1.0
 */

#ifndef __PAL_OSCACHE_H__
#define __PAL_OSCACHE_H__

#include "pal_defs.h"
#include "pal_osMem.h"

/**
 * \defgroup PalOSCache PAL OS Cache Interface
 * 
 * PAL OS Cache Interface
 * \{
 */

/** \name PAL OS Cache Interface
 *  PAL OS Cache Interface
 * \{
 */

/**
 * \brief   PAL OS Cache Invalidate
 * 
 *      This function invalidates the cache region. 
 * \param   type is cache type viz. data or instruction cache.
 * \param   start is start address of the memory region.
 * \param   size is size of memory region 
 * \return  PAL_Result
 */
PAL_INLINE PAL_Result PAL_osCacheInvalidate(PAL_OsMemAddrSpace type, Uint32 start, Uint32 size);

/**
 * \brief   PAL OS Cache Flush
 * 
 *      This function flushes the cache content to the memory.
 * \param   type is cache type viz. data or instruction cache.
 * \param   start is start address of the memory region.
 * \param   size is size of memory region 
 * \return  PAL_Result
 */
PAL_INLINE PAL_Result PAL_osCacheFlush(PAL_OsMemAddrSpace type, Uint32 start, Uint32 size);

/*\}*/
/*\}*/

#endif /* _PAL_OSCACHE_H_ */
