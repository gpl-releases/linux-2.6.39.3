/*
 *
 * pal_osProtect_inline.h
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

/** \file   pal_osProtect_inline.h
    \brief  OsPROTECT Services Source File
    

    \author     PSP Architecture Team
    \version    1.0
 */

#ifndef __PAL_OSPROTECT_INLINE_H__
#define __PAL_OSPROTECT_INLINE_H__

#include "pal_os.h"
#include "pal_defs.h"
#include "pal_osCfg.h"
#include <linux/spinlock.h>
#include <linux/slab.h>

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


/**
 * \brief   PAL OS Protect Entry
 */
PAL_INLINE void PAL_osProtectEntry(Int level, Uint32* cookie)
{
	Ulong flags;
    if(level == PAL_OSPROTECT_INTERRUPT) 
	{
        local_irq_save(flags);  
		*cookie = flags;
	}
}
/**
 * \brief   PAL OS Protect Exit
 */
PAL_INLINE void PAL_osProtectExit(Int level, Uint32 cookie)
{
    if(level == PAL_OSPROTECT_INTERRUPT) 
	{
        local_irq_restore((Ulong)cookie);
	}
}

/*\}*/
/*\}*/

#endif

