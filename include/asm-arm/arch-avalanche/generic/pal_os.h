/*
 *
 * pal_os.h
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


/** \file   pal_os.h
    \brief  OS Abstraction Header File

    This file provides visibility to OS abstraction APIs by including
    only the configured service modules interface files.

 
    \author     PSP Architecture Team
    \version    1.0
 */

#ifndef __PAL_OS_H__
#define __PAL_OS_H__

#include "pal_osCfg.h"


/* Added:
 * PAL OS module type to be placed in section D of the final "ERROR CODE"
 * as described in pal_defs.h
 */
#define PAL_OS_COMMON_ERR   (0)
#define PAL_OSMEM_ERR       (1)
#define PAL_OSBUF_ERR       (2)
#define PAL_OSSEM_ERR       (3)
#define PAL_OSMUTEX_ERR     (4)
#define PAL_OSWAIT_ERR      (5)
#define PAL_OSLIST_ERR      (6)
#define PAL_OSPROTECT_ERR   (7)
#define PAL_OSTIMER_ERR     (8)

/* Common error codes for ALL PAL OS modules */
#define PAL_OS_COMMON_ERROR_CREATE(x)   (PAL_ERROR(PAL_CRITICAL_ERROR, PAL_OS_COMMON_ERR, 0, (x)))

/* Invalid parameter passed to the function error */
#define PAL_OS_ERROR_INVALID_PARAM      (PAL_OS_COMMON_ERROR_CREATE(1))

/* Feature not supported error */
#define PAL_OS_ERROR_NOT_SUPPORTED      (PAL_OS_COMMON_ERROR_CREATE(2))

/* No resources available error */
#define PAL_OS_ERROR_NO_RESOURCES       (PAL_OS_COMMON_ERROR_CREATE(3))

/* OS specific error */
#define PAL_OS_ERROR_OS_SPECIFIC        (PAL_OS_COMMON_ERROR_CREATE(4))


/* Default (memory) segment Id - Many of the modules (like OSSEM, OSBUF), 
 * depend upon a segment id to be passed in the API's. The macro below 
 * defines a default segment Id that can be used in these API's
 */
#define PAL_OSMEM_DEFAULT_SEGID         0


#ifdef INLINE
#define PAL_INLINE static inline
#else
#define PAL_INLINE
#endif

#ifdef PAL_INCLUDE_OSMEM
#include "pal_osMem.h"          /* OsMEM Services */
#endif /* PAL_INCLUDE_OSMEM */

#ifdef PAL_INCLUDE_OSBUF
#include "pal_osBuf.h"          /* OsBUF Services */
#endif /* PAL_INCLUDE_OSBUF */

#ifdef PAL_INCLUDE_OSSEM
#include "pal_osSem.h"          /* OsSEM Services */
#endif /* PAL_INCLUDE_OSSEM */

#ifdef PAL_INCLUDE_OSMUTEX
#include "pal_osMutex.h"        /* OsMUTEX Services */
#endif /* PAL_INCLUDE_OSMUTEX */

#ifdef PAL_INCLUDE_OSWAIT
#include "pal_osWait.h"        /* OsWAIT Services */
#endif /* PAL_INCLUDE_OSWAIT */

#ifdef PAL_INCLUDE_OSLIST
#include "pal_osList.h"         /* OsLIST Services */
#endif /* PAL_INCLUDE_OSLIST */

#ifdef PAL_INCLUDE_OSPROTECT
#include "pal_osProtect.h"      /* OsPROTECT Services */
#endif /* PAL_INCLUDE_OSPROTECT */

#ifdef PAL_INCLUDE_OSCACHE
#include "pal_osCache.h"      /* OsCACHE Services */
#endif /* PAL_INCLUDE_OSCACHE */

#ifdef PAL_INCLUDE_OSTIMER
#include "pal_osTimer.h"        /* OsSEM Services */
#endif /* PAL_INCLUDE_OSTIMER */

#ifdef INLINE

#ifdef PAL_INCLUDE_OSBUF
#include "pal_osBuf_inline.h"
#endif /* PAL_INCLUDE_OSBUF */

#ifdef PAL_INCLUDE_OSMEM
#include "pal_osMem_inline.h"
#endif /* PAL_INCLUDE_OSMEM */

#ifdef PAL_INCLUDE_OSMUTEX
#include "pal_osMutex_inline.h"
#endif /* PAL_INCLUDE_OSMUTEX */

#ifdef PAL_INCLUDE_OSSEM
#include "pal_osSem_inline.h"
#endif /* PAL_INCLUDE_OSSEM */

#ifdef PAL_INCLUDE_OSWAIT
#include "pal_osWait_inline.h"
#endif /* PAL_INCLUDE_OSWAIT */

#ifdef PAL_INCLUDE_OSPROTECT
#include "pal_osProtect_inline.h"
#endif /* PAL_INCLUDE_OSPROTECT */

#ifdef PAL_INCLUDE_OSLIST
#include "pal_osList_inline.h"
#endif /* PAL_INCLUDE_OSLIST */

#ifdef PAL_INCLUDE_OSCACHE
#include "pal_osCache_inline.h"      /* OsCACHE Services */
#endif /* PAL_INCLUDE_OSCACHE */

#ifdef PAL_INCLUDE_OSTIMER
#include "pal_osTimer_inline.h"
#endif /* PAL_INCLUDE_OSTIMER */

#endif /* INLINE */

#endif /* _PAL_OS_H_ */

