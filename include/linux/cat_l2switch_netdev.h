/*
  GPL LICENSE SUMMARY

  Copyright(c) 2011-2012 Intel Corporation.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
  The full GNU General Public License is included in this distribution
  in the file called LICENSE.GPL.

  Contact Information:
    Intel Corporation
    2200 Mission College Blvd.
    Santa Clara, CA  97052
*/


#ifndef _LINUX_CAT_L2SWITCH_NETDEV_H
#define _LINUX_CAT_L2SWITCH_NETDEV_H

#define L2SW_NETDEV_DATA0       "l2sd0"
#define L2SW_NETDEV_MGMT0       "l2sm0"

typedef enum
{
    L2SW_NETDEV_INSTANCE_DATA0_e,
    L2SW_NETDEV_INSTANCE_MGMT_e,
    L2SW_NETDEV_NUM_INSTANCES,
}
L2SW_NETDEV_INSTANCE_e;

#endif /* _LINUX_CAT_L2SWITCH_NETDEV_H */

