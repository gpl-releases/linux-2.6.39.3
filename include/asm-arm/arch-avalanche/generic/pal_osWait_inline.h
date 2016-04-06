/*
 *
 * pal_osWait_inline.h 
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


/** \file   pal_osWait_inline.h
    \brief  OsWAIT Services Soource File

    This file defines OS abstraction services for programmed interval-waits.
    

    \author     PSP Architecture Team
    \version    1.0
 */

#ifndef __PAL_OSWAIT_INLINE_H__
#define __PAL_OSWAIT_INLINE_H__

#include "pal_os.h"
#include "pal_defs.h"
#include "pal_osCfg.h"
#include <linux/delay.h> 
#include <linux/sched.h> 

/**
 * \defgroup PalOSWait PAL OS Wait Interface
 * 
 * PAL OS Wait Interface
 * \{
 */

/** \name PAL OS Wait Interface
 * \{
 */

/**
 * \brief   PAL_osWaitMsecs()
 * PAL_osWaitTicks
 */
PAL_INLINE PAL_Result PAL_osWaitMsecs(Uint32 mSecs)
{
    Uint32 ticks = (HZ * mSecs)/1000;

    /*  If the wait period is less than 1 tick then busyloop */
    if(ticks)
	{
    	set_current_state(TASK_INTERRUPTIBLE);
    	schedule_timeout(ticks);
	}
    else
	{
        mdelay(mSecs); 
	}
		
    return PAL_SOK;
}

/**
 * \brief   PAL_osWaitTicks()
 */
PAL_INLINE PAL_Result PAL_osWaitTicks(Uint32 ticks)
{
    set_current_state(TASK_INTERRUPTIBLE);
    schedule_timeout(ticks);
    return PAL_SOK;
}

/*\}*/
/*\}*/

#endif

