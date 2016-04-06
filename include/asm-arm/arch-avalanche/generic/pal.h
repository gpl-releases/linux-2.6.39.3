/*
 *
 * pal.h
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


/** \file   pal.h
    \brief  Platform Include file

    This file includes all platform abstraction related headers.
    

    \author     PSP Architecture Team
    \version    1.0
 */

#ifndef _PAL_H_
#define _PAL_H_

#include "pal_defs.h"   /* Platform definitions - basic types - includes "_tistdtypes.h" */
#include "pformCfg.h"   /* Platform specific file - includes SOC specific header */
#include "csl_defs.h"   /* CSL definitions - basic types and macros */

#include "pal_os.h"     /* PAL OS interfaces file - includes selected OS abstraction headers */

#include "pal_sys.h"    /* Platform System Interfaces file */

#endif /* _PAL_H_ */
