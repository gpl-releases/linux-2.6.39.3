/*
 *
 * avalanche_vlynq_config.h
 * Description:
 * vlynq platform configuration header file
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


#ifndef _AV_VL_CFG_H_
#define _AV_VL_CFG_H_

#include <mach/generic/pal.h>

typedef char* (*AV_VL_GETENV)(char *env);
typedef void  (*AV_VL_PRINTF)(char *format, ...);

Int32 avalanche_vlynq_set_hw_reset_info(Uint32 root_base_addr, Uint32 zero_based_hop,
                                        Uint32 gpio_bit,       Bool   gpio_pol);

Int32 avalanche_vlynq_enumerate(Uint32 cvr, Uint32 root_vlynq_virt_base,
                                Uint32 tx_virt_portal);

#endif
