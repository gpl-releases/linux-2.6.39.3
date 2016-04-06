/*
 *
 * led_platform.h
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


/******************************************************************************    
 * FILE PURPOSE:     - LED Platform specific Header file                                      
 ******************************************************************************    
 * FILE NAME:     led_platform.h                                                        
 *                                                                                 
 * DESCRIPTION:   Linux specific implementation for OS abstracted function calls
 *                made by LED HAL module. This file has functions defined for
 *                Memory allocation calls, Mutex calls, String and Timer 
 *                operations.
 *                                                                 
 * REVISION HISTORY:                                                               
 * 11 Oct 03 - PSP TII                                                             
 *                                                                                 
 *******************************************************************************/  

                                                                                   
#ifndef __LED_PLATFORM__
#define __LED_PLATFORM__

#include <linux/slab.h>
#include <asm-arm/arch-avalanche/generic/pal.h>

#define os_strcmp            os_strcasecmp
#define os_strcpy            strcpy

/* #define LED_HAL_DEBUG */

#if defined(LED_HAL_DEBUG)
#define log_msg(...)              printk(__VA_ARGS__)
#else
#define log_msg(...)
#endif

/* String handling functions  not defined in asm/string.h */
static __inline__ int os_strlen(char *str)
{
    int i;
    for(i=0;str[i];i++);
    return i;
}


#define LOWER(x)    ((x < 'a') ? (x - 'A' + 'a'):(x))                                    
#define ISALPHA(x)   ((( x >= 'a') && (x <= 'z')) || (( x >= 'A') && (x <= 'Z')))        
#define COMP(x,y)   ((x == y) || ((ISALPHA(x) && ISALPHA(y)) && (LOWER(x) == LOWER(y)))) 

#define OS_TIMER_ADD( func, handle, delay, arg )  os_timer_add( handle, delay, arg )

static __inline__ int os_strcasecmp(char *str1, char *str2)
{                                         
    int i;                            
    
    for(i=0;str1[i] && str2[i];i++)
    {
        char x,y;
        
        x = str1[i];
        y = str2[i];

        if(!COMP(x,y))
            break;
    }              

    return(str1[i] || str2[i]);
}                                                                                  

/* Functions for timer related operations */
static __inline__ void * os_timer_init(void (*func)(int))
{
    struct timer_list *ptimer;
    ptimer = (struct timer_list *) kmalloc(sizeof(struct timer_list),GFP_KERNEL);
    init_timer( ptimer );
    ptimer->function = (void *)func;
    return (void *)ptimer;
}

static __inline__ int os_timer_add(void *timer_handle,int milisec,int arg)
{
    struct timer_list *ptimer=timer_handle;
    ptimer->expires = ((HZ * milisec)/1000) + jiffies;
    ptimer->data = arg;
    add_timer(ptimer);	
    return 0;
}

static __inline__ int os_timer_delete(void *timer_handle)
{
    struct timer_list *ptimer=timer_handle;
    del_timer(ptimer);
    return 0;
}

static __inline__ int os_timer_destroy(void *timer_handle)
{
    struct timer_list *ptimer=timer_handle;
    del_timer_sync(ptimer);
    kfree(ptimer);
    return 0;
}
#endif /* __LED_PLATFORM__ */

