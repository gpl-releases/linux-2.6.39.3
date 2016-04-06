/*
  GPL LICENSE SUMMARY

  Copyright(c) 2008-2012 Intel Corporation.

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

/******************************************************************************
 * FILE PURPOSE:     - LED kernel Header
 ******************************************************************************
 * FILE NAME:     led_hal.h
 *
 * DESCRIPTION:   Header file defining HAL types and functions.
 *
 *
 *******************************************************************************/


#ifndef _LED_HAL_H_
#define _LED_HAL_H_

#include "_tistdtypes.h"

#define LED_ARR_LEN(v)  (sizeof(v) / sizeof(v[0]))

#define MAX_STATES_PER_MOD 25


#define LED_HAL_BITMASK_WIDTH       (128)
#define LED_HAL_BITMASK_BYTE_WIDTH  (LED_HAL_BITMASK_WIDTH/8)
#define LED_HAL_BITMASK_REG_WIDTH   (LED_HAL_BITMASK_WIDTH/32)


/*
 * NOTE: The LED defines that appear bellow, must match
 * the location of the modules inside the array named "modules"
*/
typedef enum
{
    PUMA_LED_ID_POWER,
    PUMA_LED_ID_DS,
    PUMA_LED_ID_US,
    PUMA_LED_ID_ONLINE,
    PUMA_LED_ID_WPS,    //CISCO MODIFY
    PUMA_LED_ID_LINE1,
    PUMA_LED_ID_LINE2,
    PUMA_LED_ID_DECT,   //CISCO MODIFY
    PUMA_LED_ID_WIFI,   //CISCO MODIFY
    PUMA_LED_ID_BATTERY,
    PUMA_LED_ID_MOCA,

    PUMA_LED_ID_NUM_LEDS
}
PUMA_LED_ID_e;

/*! \var typedef enum ledState_e
    \brief LED states.
*/
typedef enum
{
    PUMA_LED_STATE_OFF,
    PUMA_LED_STATE_ON,
    PUMA_LED_STATE_FLASH,
    PUMA_LED_STATE_FLASH_SLOW,  //CISCO ADD
    PUMA_LED_STATE_FLASH_FAST,  //CISCO ADD
    PUMA_LED_STATE_NUM_STATES,

    PUMA_LED_STATE_INVALID = 0xFFFFFFFF, //CISCO ADD
}
PUMA_LED_STATE_e;





typedef enum
{
    LED_HAL_MODE_LED_OFF           ,
    LED_HAL_MODE_LED_ON            ,
    LED_HAL_MODE_LED_ONESHOT_BACK  ,
    LED_HAL_MODE_LED_ONESHOT_ON    ,
    LED_HAL_MODE_LED_FLASH         ,
    LED_HAL_MODE_LED_ONESHOT_OFF   ,
    LED_HAL_MODE_LED_FLASH_BACK    ,
}
STATE_CFG_MODE_T;

/*---------------------------------------------------------------------------
 * Configuration Support.
 *-------------------------------------------------------------------------*/
typedef struct led_cfg
{
    Uint32 domain;
    Uint32 pos_map[ LED_HAL_BITMASK_REG_WIDTH ]; /* Indicates the position of LED bit; index 0 -> range of 0 - 31, index 1 -> range of 32 - 63 etc.*/
} LED_CFG_T;

typedef struct state_cfg
{
    Uint32          id;
    Uint32          mode;
    Uint32          param1;
    Uint32          param2;
    Uint32          led_val[ LED_HAL_BITMASK_REG_WIDTH ];
    LED_CFG_T       led_cfg;
} STATE_CFG_T;

typedef struct mod_cfg
{
    Int8            name[50];
    Uint32          instance;
    STATE_CFG_T     state_cfg;

} MOD_CFG_T;

typedef struct led_funcs
{
    Uint32 domain;
    Uint32 pos_map[ LED_HAL_BITMASK_REG_WIDTH ];  /* Indicates the position of LED bit; index 0 -> range of 0 - 31, index 1 -> range of 32 - 63 etc.*/
    Uint32 off_val[ LED_HAL_BITMASK_REG_WIDTH ]; /* Init values to be reflected on the LED(s). */
    Int32 (*outVal)(Uint32 led_val, Uint32 pos_map, Uint32 index);

} LED_FUNCS_T;

/*--------------------------------------------------------------------------
 * Public API(s)
 *------------------------------------------------------------------------*/
typedef void MOD_OBJ_HND;
typedef void LED_OBJ_HND;

MOD_OBJ_HND*    led_hal_register            (Int8 *name, Uint32 instance);
void            led_hal_unregister          (MOD_OBJ_HND *mod);

Int32           led_hal_action              (MOD_OBJ_HND *mod, Uint32 state_id);

LED_OBJ_HND*    led_hal_install_callbacks   (LED_FUNCS_T *led_funcs);
Int32           led_hal_uninstall_callbacks (LED_OBJ_HND *led);

Int32           led_hal_configure_mod       (MOD_CFG_T *cfg);

Int32           led_hal_dump_cfg_info       (Int8 *buf, Int32 size);

Int32           led_hal_init( void );
Int32           led_hal_exit( void );

#endif

