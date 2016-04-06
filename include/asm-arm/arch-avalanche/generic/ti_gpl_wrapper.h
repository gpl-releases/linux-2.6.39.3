/*
 *
 * ti_gpl_wrapper.h 
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

#ifndef __TI_GPL_WRAPPER_H__
#define __TI_GPL_WRAPPER_H__

#include <linux/module.h>
#include <linux/platform_device.h>
#include <asm-arm/arch-avalanche/generic/soc.h>

extern int fastcall ti_queue_work(struct workqueue_struct *, struct work_struct *);
extern void ti_destroy_workqueue(struct workqueue_struct *);
extern struct workqueue_struct *__ti_create_workqueue(const char *, int);

extern struct bus_type* platform_bus_type_ptr;
extern int ti_driver_create_file(struct device_driver *, struct driver_attribute *);
extern void ti_driver_remove_file(struct device_driver *, struct driver_attribute *);
extern int ti_driver_register(struct device_driver * drv);
extern void ti_driver_unregister(struct device_driver * drv);
extern struct platform_device *ti_platform_device_register_simple(char *, unsigned int, struct resource *, unsigned int);
extern int ti_platform_device_register(struct platform_device *);
extern void ti_platform_device_unregister(struct platform_device *);

#endif /* __TI_GPL_WRAPPER_H__ */
