/*
 *
 * led_manager.h 
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
 * FILE PURPOSE:     - LED manager Header                                       
 ******************************************************************************    
 * FILE NAME:     led_manager.h                                                        
 *                                                                                 
 * DESCRIPTION:  Header file for LED manager                                        
 *                                                                                 
   
******************************************************************************/    
                                                                                   
#ifndef __LED_MANAGER__
#define __LED_MANAGER__


#include <led_hal.h>


struct led_manager_user_module		                
{
	unsigned char *name;			
	unsigned int instance;	    	
	unsigned int handle;	
};

struct led_manager_user_action
{
	unsigned int handle;						 
	unsigned int state_id;						
};

#define LED_MANAGER_MAGIC			0xD1

#define LED_MANAGER_CONFIG              _IOW(LED_MANAGER_MAGIC, 1, MOD_CFG_T)
#define LED_MANAGER_REGISTER	        _IOWR(LED_MANAGER_MAGIC, 2, struct led_manager_user_module)
#define LED_MANAGER_UNREGISTER	        _IOW(LED_MANAGER_MAGIC, 3, void *)
#define LED_MANAGER_ACTION		_IOW(LED_MANAGER_MAGIC, 4, struct led_manager_user_action)

typedef struct led_manager_user_module LED_MODULE_T;
typedef struct led_manager_user_action LED_STATE_T;

#define LED_CONFIG			LED_MANAGER_CONFIG 		
#define LED_GET_HANDLE			LED_MANAGER_REGISTER 	
#define LED_ACTION			LED_MANAGER_ACTION 		
#define LED_RELEASE_HANDLE		LED_MANAGER_UNREGISTER

#define avalanche_led_register		led_manager_register_module
#define avalanche_led_action		led_manager_led_action
#define avalanche_led_unregister	led_manager_unregister_module


MOD_OBJ_HND *led_manager_register_module(char *module_name, int instance );
void led_manager_unregister_module( void *module_handle );
LED_OBJ_HND * led_manager_install_callbacks(LED_FUNCS_T * funcs);
int led_manager_led_action( void *module_handle, int state_id );
int led_manager_uninstall_callbacks( void *module_handle);
int led_manager_cfg_mod( MOD_CFG_T *mod_cfg);

#endif
