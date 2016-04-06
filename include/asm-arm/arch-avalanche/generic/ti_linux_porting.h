/*
 *
 * ti_linux_porting.h
 * Description:
 *
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
#ifndef __TI_LINUX_PORTING_H__
#define __TI_LINUX_PORTING_H__

#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
#define NETDEV_PRIV(net_dev)  (net_dev)->priv
#else
#define NETDEV_PRIV(net_dev)  netdev_priv(net_dev)
#endif


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,0)
#define FREE_NETDEV(net_dev)  kfree(net_dev)
#else
#define FREE_NETDEV(net_dev)  free_netdev(net_dev)
#endif

#endif /* __TI_LINUX_PORTING_H__ */
