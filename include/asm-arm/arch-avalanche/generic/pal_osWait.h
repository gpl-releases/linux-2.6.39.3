/*
 *
 * pal_osWait.h
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


/** \file   pal_osWait.h
    \brief  OsWAIT Services Header File

    This file declares OS abstraction services for programmed interval-waits.
    

    \author     PSP Architecture Team
    \version    1.0
 */

#ifndef __PAL_OSWAIT_H__
#define __PAL_OSWAIT_H__

#include "pal_defs.h"
#include "pal_os.h"

/**
 * \defgroup PalOSWait PAL OS Wait Interface
 * 
 * PAL OS Wait Interface
 * \{
 */

/** \name PAL OS Wait Interface
 *  PAL OS Wait Interface
 * \{
 */

/**
 * \brief   PAL_osWaitMsecs()
 *
 *      This function leverages OS implemented "wait" to delay
 *      further execution of current thread for specified milliseconds
 *      period of time.
 * \note    The currently running thread might be preempted and placed
 *      on scheduler's wait queue for the specified duration of time.
 *      If a more effecient (w/o context switch overhead) wait is required
 *      at finer time granularity (order of microseconds), please use
 *      SysWAIT services defined in pal_sys.h file
 * \note    The function will do any milliseconds-to-ticks conversion
 *      as appropriate for implementing the wait using underlying
 *      OS supported APIs.
 * \param   mSecs [IN] is the duration in milliseconds to wait for
 * \return  PAL_SOK if succesful else a suitable error code
 */
PAL_INLINE PAL_Result PAL_osWaitMsecs(Uint32 mSecs);

/**
 * \brief   PAL_osWaitTicks()
 *
 *      This function leverages OS implemented "wait" to delay
 *      further execution of current thread for specified number
 *      of operating system ticks.
 * \note    The currently running thread might be preempted and placed
 *      on scheduler's wait queue for the specified duration of time.
 *      If a more effecient (w/o context switch overhead) wait is required
 *      at finer time granularity (order of microseconds), please use
 *      SysWAIT services defined in pal_sys.h file
 * \param   ticks [IN] is the number of operating system ticks to wait for
 * \return  PAL_SOK if succesful else a suitable error code
 */
PAL_INLINE PAL_Result PAL_osWaitTicks(Uint32 ticks);

/*\}*/
/*\}*/

#endif /* _PAL_OSWAIT_H_ */
