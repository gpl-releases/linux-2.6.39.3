/*
 *
 * pal_osProtect.h
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


/** \file   pal_osProtect.h
    \brief  OsPROTECT Services Header File
    ===================================================================
    The OsPROTECT models various levels of reentrancy protection
    for use while writing critical sections in user code. critical
    sections are those parts of user code that needs to run atomically
    in some sense. Meaning, single threading is called for. However,
    the degree of protection sought by user varies based on nature
    of code he is writing.

    It is possible that for some regions of code, user needs ultimate
    degree of protection where all external interrupts are blocked,
    essentially locking out the CPU exclusively for the critical
    section of code. On the other hand user may wish to merely avoid
    thread or task switch from occuring inside said region of code,
    but he may wish to entertain ISRs to run if so required.

    Depending on the underlying OS, the number of levels of protection
    offered may vary. At the least, two basic levels of protection are
    supported --

    - PAL_OSPROTECT_INTERRUPTS - Mask interrupts globally. This has
      real-time implications and must be used with descretion.
      If blocking/unblocking of specific interrupt lines is desired,
      one is reffered to APIs listed in pal_sys.h file.

    - PAL_OSPROTECT_SCHEDULER - Only turns off Kernel scheduler 
      completely, but still allows h/w interrupts from being serviced.

    Protection levels 0 to N (max positive Int) are platform specific
    ===================================================================
    

    \author     PSP Architecture Team
    \version    1.0
 */

#ifndef __PAL_OSPROTECT_H__
#define __PAL_OSPROTECT_H__

#include "pal_defs.h"
#include "pal_os.h"

/**
 * \defgroup PalOSProtect PAL OS Protect Interface
 * 
 * PAL OS Protect Interface
 * \{
 */

/** \name PAL OS Protect Interface
 *  PAL OS Protect Interface
 * \{
 */

#define PAL_OSPROTECT_INTERRUPT (-1)
#define PAL_OSPROTECT_SCHEDULER (-2)

/**
 * \brief   PAL OS Protect Entry
 * 
 *      This function saves the current state of protection in cookie
 *      variable passed by caller. It then applies the requested level
 *      of protection
 * \param   level is numeric identifier of the desired degree of protection.
 * \param   cookie is memory location where current state of protection is
 *      saved for future use while restoring it via PAL_osProtectExit()
 * \note    user is not expected to interpret the cookie in any manner. It
 *      is intended for use in terminating the presently enforced
 *      protection via a matching PAL_osProtectExit() call discssed
 *      later in this file.
 * \return  None
 */
PAL_INLINE void PAL_osProtectEntry(Int level, Uint32* cookie);

/**
 * \brief   PAL OS Protect Exit
 * 
 *      This function undoes the protection enforced to original state
 *      as is specified by the cookie passed.
 * \param   level is numeric identifier of the desired degree of protection.    
 * \param   cookie is original state of protection at time when the
 *      corresponding PAL_osProtectEnter() was called.
 * \return  None
 */
PAL_INLINE void PAL_osProtectExit(Int level, Uint32 cookie);

/*\}*/
/*\}*/

#endif /* _PAL_OSPROTECT_H_ */
