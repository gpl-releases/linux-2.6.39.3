/*
 *
 * puma5_intd.c 
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


/** \file   puma5_intd.c
    \brief  puma5 SoC related info required by alalanche interrupt distributor.

    \author     PSP TII
    \version    0.1
 */

#include <asm-arm/arch-avalanche/generic/pal.h>

#define AVALANCHE_SOC_NAME         "PUMA5"   
#define AVALANCHE_SOC_MAX_HOSTS    (2)
#define ARM11_MAX_IP_INTS_MAPPED   (32)
#define C55x_MAX_IP_INTS_MAPPED    (00)
#define AVALANCHE_SOC_TOTAL_IP_INTS_MAPPED  ( ARM11_MAX_IP_INTS_MAPPED +\
                                              C55x_MAX_IP_INTS_MAPPED )

/* NOTE: In the below enum populate  the hosts 
 * alphabetically for  proper interrupt  mappings 
 */
enum puma5_hosts {ARM11, C55X };

/*
 *Please name the structure as soc_info so that generic distributor layer works. 
 */

AVALANCHE_SOC_INFO soc_info = {
	AVALANCHE_SOC_NAME,
	AVALANCHE_SOC_MAX_HOSTS,
	AVALANCHE_SOC_TOTAL_IP_INTS_MAPPED,
        {
        {ARM11,ARM11_MAX_IP_INTS_MAPPED },
        {C55X, C55x_MAX_IP_INTS_MAPPED }
        }
       };

