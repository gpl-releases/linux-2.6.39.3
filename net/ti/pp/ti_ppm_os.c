/*
 * ti_ppm_os.c
 *
 *  The file contains the OS abstraction layer and the System Interface model
 *  for the PPM.
 *
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/


#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/netdevice.h>
#include <linux/inetdevice.h>
#include <linux/in.h>
#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <linux/if_pppox.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/ppp_defs.h>
#include <linux/spinlock.h>
#include <linux/ti_hil.h>
#include "linux/ti_ppm.h"

/**************************************************************************
 ************************************ Local Definitions *******************
 **************************************************************************/

/* Define this symbol to allow debug support for the OS Layer. Useful
 * debugging memory leaks. */
#undef TI_PPM_OS_DEBUG

/**************************************************************************
 ************************************ Globals *****************************
 **************************************************************************/

#ifdef TI_PPM_OS_DEBUG
int num_allocations = 0;
int num_free = 0;
#endif /* TI_PPM_OS_DEBUG */

/**************************************************************************
 ******************************* Functions  *******************************
 **************************************************************************/

/**************************************************************************
 * FUNCTION NAME : ti_ppm_os_malloc
 **************************************************************************
 * DESCRIPTION   :
 *  Wrapper for OS memory allocation.
 *
 * RETURNS       :
 *  Pointer to allocated memory - Success.
 *  0                           - Error
 **************************************************************************/
void* ti_ppm_os_malloc (unsigned int size)
{
    void* ptr = kmalloc(size, GFP_ATOMIC);
#ifdef TI_PPM_OS_DEBUG
    num_allocations++;
    printk ("Memory Allocated: 0x%p\n", ptr);
#endif /* TI_PPM_OS_DEBUG */
    return ptr;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_os_free
 **************************************************************************
 * DESCRIPTION   :
 *  Wrapper for OS memory cleanup.
 **************************************************************************/
void ti_ppm_os_free (void* ptr)
{
#ifdef TI_PPM_OS_DEBUG
    printk ("Memory Cleaned: 0x%p\n", ptr);
    num_free++;
#endif /* TI_PPM_OS_DEBUG */
    kfree (ptr);
    return;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_os_memset
 **************************************************************************
 * DESCRIPTION   :
 *  Wrapper for OS memset function.
 **************************************************************************/
void ti_ppm_os_memset (void* dst, int c, unsigned int size)
{
    memset (dst, c, size);
    return;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_os_memcpy
 **************************************************************************
 * DESCRIPTION   :
 *  Wrapper for OS memcpy function.
 **************************************************************************/
void ti_ppm_os_memcpy (void* dst, void* src, unsigned int size)
{
    memcpy (dst, src, size);
    return;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_os_memcmp
 **************************************************************************
 * DESCRIPTION   :
 *  Wrapper for OS memcmp function.
 **************************************************************************/
int ti_ppm_os_memcmp (void* arg1, void* arg2, unsigned int size)
{
    return memcmp (arg1, arg2, size);
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_os_critical_start
 **************************************************************************
 * DESCRIPTION   :
 *  Wrapper for OS start critical section
 **************************************************************************/
void ti_ppm_os_critical_start (PPM_OS_CONTEXT* ptr_context)
{
    int flags;

    local_irq_save(flags);

    ptr_context->flags = flags;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_os_critical_stop
 **************************************************************************
 * DESCRIPTION   :
 *  Wrapper for OS stop critical section
 **************************************************************************/
void ti_ppm_os_critical_stop (PPM_OS_CONTEXT* ptr_context)
{
    local_irq_restore(ptr_context->flags);
}

/**************************************************************************
 * FUNCTION NAME : ti_pp_sys_initialize
 **************************************************************************
 * DESCRIPTION   :
 *  Initialization function called to initialize the PPM OS layer/PPD.
 *
 * RETURNS       :
 *  0   - Success.
 *  <0  - Error
 **************************************************************************/
int ti_pp_sys_initialize (void)
{
    TI_PPM_OS_FUNC_TABLE    os_table;

    /* Initialize the OS Function table. */
    os_table.memcpy                 = (OS_MEMCPY)memcpy;
    os_table.memcmp                 = (OS_MEMCMP)memcmp;
    os_table.memset                 = (OS_MEMSET)memset;
    os_table.malloc                 = (OS_MALLOC)ti_ppm_os_malloc;
    os_table.free                   = (OS_FREE)ti_ppm_os_free;
    os_table.critical_section_start = (OS_CRITICAL_SECTION_START)ti_ppm_os_critical_start;
    os_table.critical_section_end   = (OS_CRITICAL_SECTION_END)ti_ppm_os_critical_stop;

    /* Initialize the PPM, PPD and the PDSP. */
    if (ti_ppm_initialize (&os_table) < 0)
    {
        printk ("Error: Failed to initialize the PPM\n");
        return -1;
    }

    /* Successfully initialized the OS Layer. */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_pp_sys_deinit
 **************************************************************************
 * DESCRIPTION   :
 *  DeInitialization function called to deinitialize the PPM OS layer.
 *
 * RETURNS       :
 *  0   - Success.
 *  <0  - Error
 **************************************************************************/
int ti_pp_sys_deinit (void)
{
    /* Deinitialize the HIL Core Layer. */
    ti_hil_deinitialize ();

    /* Deinitialize PPM which in turn closes the PPD/PDSP */
    if(ti_ppm_deinitialize() < 0)
    {
        printk("PPM deinitialize failed \n");
        return -1;
    }

#ifdef TI_PPM_OS_DEBUG
    /* Basic Testing for memory leaks. */
    if (num_allocations != num_free)
        printk ("Error: Memory leak detected\n");
    else
        printk ("Success: No memory leak\n");
#endif /* TI_PPM_OS_DEBUG */

    /* Succesfully cleaned up the OS Layer. */
    return 0;
}

/* Export all the Symbols for Linux; so that these can be called from modules. */
EXPORT_SYMBOL(ti_ppm_initialize);
EXPORT_SYMBOL(ti_ppm_deinitialize);

/* PID and VPID Management API */
EXPORT_SYMBOL(ti_ppm_create_pid);
EXPORT_SYMBOL(ti_ppm_delete_pid);
EXPORT_SYMBOL(ti_ppm_config_pid_range);
EXPORT_SYMBOL(ti_ppm_remove_pid_range);
EXPORT_SYMBOL(ti_ppm_set_pid_flags);
EXPORT_SYMBOL(ti_ppm_create_vpid);
EXPORT_SYMBOL(ti_ppm_delete_vpid);
EXPORT_SYMBOL(ti_ppm_set_vpid_flags);
EXPORT_SYMBOL(ti_ppm_get_pid);
EXPORT_SYMBOL(ti_ppm_get_vpid);

/* Session Management API */
EXPORT_SYMBOL(ti_ppm_flush_sessions);
EXPORT_SYMBOL(ti_ppm_create_session);
EXPORT_SYMBOL(ti_ppm_delete_session);
EXPORT_SYMBOL(ti_ppm_modify_session);
EXPORT_SYMBOL(ti_ppm_check_session);
EXPORT_SYMBOL(ti_ppm_lookup_session);
EXPORT_SYMBOL(ti_ppm_get_session);

/* Statistics API */
EXPORT_SYMBOL(ti_ppm_get_session_stats);
EXPORT_SYMBOL(ti_ppm_clear_session_stats);
EXPORT_SYMBOL(ti_ppm_get_vpid_stats);
EXPORT_SYMBOL(ti_ppm_clear_vpid_stats);
EXPORT_SYMBOL(ti_ppm_get_global_stats);
EXPORT_SYMBOL(ti_ppm_clear_global_stats);

/* Generic API. */
EXPORT_SYMBOL(ti_ppm_get_version);
EXPORT_SYMBOL(ti_ppm_health_check);

/* Event Handler Framework API */
EXPORT_SYMBOL(ti_ppm_register_event_handler);
EXPORT_SYMBOL(ti_ppm_unregister_event_handler);

/* QoS Statistics API */
EXPORT_SYMBOL(ti_ppm_get_qos_q_stats);
EXPORT_SYMBOL(ti_ppm_get_n_clear_qos_q_stats);

/* QoS API */
EXPORT_SYMBOL(ti_ppm_qos_cluster_setup);
EXPORT_SYMBOL(ti_ppm_qos_cluster_enable);
EXPORT_SYMBOL(ti_ppm_qos_cluster_disable);
