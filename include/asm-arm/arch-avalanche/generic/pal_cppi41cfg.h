/*
 *
 * pal_cppi41cfg.h
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

/** \file   pal_cppi41cfg.h
    \brief  PAL CPPI4 Configuration header file

    This file has the static (compile time) configuration parameters for the
    CPPI4 PAL driver.

    @author     Greg Guyotte
 */

#ifndef __PAL_CPPI4_CFG_H__
#define __PAL_CPPI4_CFG_H__


/* CPPI 4.1 hardware specific */

/**
 * CPPI4 hardware specific configuration 
 */
#define CPPI4_RESET_WAIT                1     /**< Msecs to wait after reset op */

/**
 * CPPI4 Generic configuration 
 */
#define PAL_CPPI41_NUM_TD_DESC               ((PAL_CPPI41_NUM_TOTAL_CHAN) * 2)  /**< Total number of teardown desc. Just a ballpark, no science */

/**
   \brief Macros for Address conversions 
   \note: These need to be ported for a different platform other than MIPS 
 */
#define PAL_CPPI4_VIRT_2_PHYS(addr)     PAL_osMemVirt2Phy((Ptr)(addr))
#define PAL_CPPI4_PHYS_2_VIRT(addr)     PAL_osMemPhy2Virt((Uint32)(addr))
#define PAL_CPPI4_VIRT_NOCACHE(addr)    PAL_osMemVirt2VirtNoCache((Ptr)(addr))

#endif /* __PAL_CPPI4_CFG_H__ */
