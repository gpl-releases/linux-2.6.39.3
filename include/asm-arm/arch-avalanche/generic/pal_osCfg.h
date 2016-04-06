/*
 *
 * pal_osCfg.h
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


/** \file   pal_osCfg.h
    \brief  OS Configuration Header File

    This file provides the OS configuration.


    \author     Ajay Singh
    \version    0.1
 */

#ifndef __PAL_OSCFG_H__
#define __PAL_OSCFG_H__

#define INLINE 

#define PAL_INCLUDE_OSMEM
#define PAL_INCLUDE_OSBUF
#define PAL_INCLUDE_OSSEM
#define PAL_INCLUDE_OSMUTEX
#define PAL_INCLUDE_OSWAIT
#define PAL_INCLUDE_OSLIST
#define PAL_INCLUDE_OSPROTECT
#define PAL_INCLUDE_OSCACHE
#define PAL_INCLUDE_OSTIMER

#endif
