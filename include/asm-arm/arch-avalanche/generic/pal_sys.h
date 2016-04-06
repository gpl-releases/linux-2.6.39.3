/*
 *
 * pal_sys.h
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


/** \file   pal_sys.h
    \brief  PAL SoC level API header file

    This file defines data types and services (macros as well as functions)
    that are applicable to the abstracted h/w system (SoC/Board).

 
    \author     PSP Architecture Team
    \version    1.0
 */

#ifndef __PAL_SYS_H__
#define __PAL_SYS_H__

#define BOARD_TYPE_UNKNOWN 0xFF
/*****************************************************************************
 * Reset Control Module
 *****************************************************************************/

typedef enum PAL_SYS_RESET_CTRL_tag
{
    IN_RESET        = 0,
    OUT_OF_RESET
} PAL_SYS_RESET_CTRL_T;

typedef enum PAL_SYS_SYSTEM_RST_MODE_tag
{
    RESET_SOC_WITH_MEMCTRL      = 1,    /* SW0 bit in SWRCR register */
    RESET_SOC_WITHOUT_MEMCTRL   = 2     /* SW1 bit in SWRCR register */
} PAL_SYS_SYSTEM_RST_MODE_T;

typedef enum PAL_SYS_SYSTEM_RESET_STATUS_tag
{
    HARDWARE_RESET = 0,		/**< Power On Reset */
    SOFTWARE_RESET0,   		/**< External Warm Reset*/
    WATCHDOG_RESET,			/**< Maximum Reset - this could be from watchdog or emulation */
	SOFTWARE_RESET1,		/**< System/Chip Reset */
	RST_STAT_END
} PAL_SYS_SYSTEM_RESET_STATUS_T;

void PAL_sysResetCtrl(unsigned int reset_module,PAL_SYS_RESET_CTRL_T reset_ctrl);
PAL_SYS_RESET_CTRL_T PAL_sysGetResetStatus(unsigned int reset_module);
void PAL_sysSystemReset(PAL_SYS_SYSTEM_RST_MODE_T mode);

typedef void (*REMOTE_VLYNQ_DEV_RESET_CTRL_FN)(unsigned int reset_module, 
                                               PAL_SYS_RESET_CTRL_T reset_ctrl);


/*****************************************************************************
 * Power Control Module
 *****************************************************************************/
/** \enum PAL_SYS_POWER_CTRL_tag
	\brief Enum for power control states
	\TODO Move this to pal_sys.h
*/
typedef enum PAL_SYS_POWER_CTRL_tag
{
	/* these enums below are used for modules alone */	
	PSC_SW_RST_DISABLE=0,	/**< Completely OFF (IN RESET) state - reset asserted and clock gated */
	PSC_SYNC_RESET,			/**< Sync reset - reset asserted and clock running */
	PSC_DISABLE, 			/**< Low power mode - Reset deasserted and clock gated */
	PSC_ENABLE,				/**< Completely ON (OUT OF RESET) state - reset deasserted and clock running */

	/* These enums should be used only for domains or from pal_sysPowerCtrl.c */
    POWER_CTRL_POWER_UP,		/**< Power On */
    POWER_CTRL_POWER_DOWN,			/**< Power Off */
} PAL_SYS_POWER_CTRL_T;


typedef enum PAL_SYS_SYSTEM_POWER_MODE_tag
{
    GLOBAL_POWER_MODE_RUN       = 0,    /* All system is up */
    GLOBAL_POWER_MODE_IDLE,             /* MIPS is power down, all peripherals working */
    GLOBAL_POWER_MODE_STANDBY,          /* Chip in power down, but clock to ADSKL subsystem is running */
    GLOBAL_POWER_MODE_POWER_DOWN        /* Total chip is powered down */
} PAL_SYS_SYSTEM_POWER_MODE_T;

void PAL_sysPowerCtrl(unsigned int power_module,  PAL_SYS_POWER_CTRL_T power_ctrl);

/*****************************************************************************
 * Wakeup Control
 *****************************************************************************/

typedef enum PAL_SYS_WAKEUP_INTERRUPT_tag
{
    WAKEUP_INT0 = 1,
    WAKEUP_INT1 = 2,
    WAKEUP_INT2 = 4,
    WAKEUP_INT3 = 8
} PAL_SYS_WAKEUP_INTERRUPT_T;

typedef enum PAL_SYS_WAKEUP_CTRL_tag
{
    WAKEUP_DISABLED = 0,
    WAKEUP_ENABLED
} PAL_SYS_WAKEUP_CTRL_T;

typedef enum PAL_SYS_WAKEUP_POLARITY_tag
{
    WAKEUP_ACTIVE_HIGH = 0,
    WAKEUP_ACTIVE_LOW
} PAL_SYS_WAKEUP_POLARITY_T;


/*****************************************************************************
 * GPIO Control
 *****************************************************************************/

typedef enum PAL_SYS_GPIO_PIN_MODE_tag
{
    FUNCTIONAL_PIN = 0,
    GPIO_PIN = 1
} PAL_SYS_GPIO_PIN_MODE_T;

typedef enum PAL_SYS_GPIO_PIN_DIRECTION_tag
{
    GPIO_OUTPUT_PIN = 0,
    GPIO_INPUT_PIN = 1
} PAL_SYS_GPIO_PIN_DIRECTION_T;

typedef enum { GPIO_FALSE, GPIO_TRUE } PAL_SYS_GPIO_BOOL_T;



/*    Jay
 * From idl-29.0.12344.325129/idl/include/idl_gpio.h

* This enumeration defines the different GPIO interrupt types. <BR><BR>
* The field IDL_GPIO_ACTIVE_HIGH_LEVEL indicates an active high level interrupt type.<BR>
* The field IDL_GPIO_ACTIVE_LOW_LEVEL indicates an active low level interrupt type.<BR>
* The field IDL_GPIO_RISING_UP_EDGE indicates a rising edge interrupt type.<BR>
* The field IDL_GPIO_FALLING_DOWN_EDGE indicates an falling down edge interrupt type.<BR>
*
*/
typedef enum {
    IDL_GPIO_ACTIVE_HIGH_LEVEL      = 0x0, /**< - Active high level interrupt */
    IDL_GPIO_ACTIVE_LOW_LEVEL       = 0x1, /**< - Active low level interrupt */
    IDL_GPIO_RISING_UP_EDGE         = 0x2, /**< - Rising up edge interrupt  */
    IDL_GPIO_FALLING_DOWN_EDGE      = 0x3, /**< - Falling down edge interrupt */
} idl_gpio_interrupt_type_t;



INT32 PAL_sysGpioInBit(UINT32 gpio_pin);
INT32 PAL_sysGpioOutBit(UINT32 gpio_pin, INT32 value);

//CISCO ADD START
INT32 PAL_sysGpioCtrlSetDir(UINT32 gpio_pin, PAL_SYS_GPIO_PIN_DIRECTION_T pinDir);
INT32 PAL_sysGpioInBit_noProt(UINT32 gpio_pin);
INT32 PAL_sysGpioOutBit_noProt(UINT32 gpio_pin, INT32 value);
INT32 PAL_sysGpioCtrlSetDir_noProt(UINT32 gpio_pin, PAL_SYS_GPIO_PIN_DIRECTION_T pinDir);
//CISCO ADD END

#if defined (CONFIG_MACH_PUMA6)
PAL_SYS_GPIO_PIN_DIRECTION_T PAL_sysGpioCtrlGetDir(UINT32 gpio_pin);
INT32 PAL_sysGpioCtrlSetDir(UINT32 gpio_pin, PAL_SYS_GPIO_PIN_DIRECTION_T pin_direction);
#else
INT32 PAL_sysGpioCtrl(UINT32 gpio_pin, PAL_SYS_GPIO_PIN_MODE_T pin_mode, PAL_SYS_GPIO_PIN_DIRECTION_T pin_direction);
INT32 PAL_sysGpioOutValue(UINT32 out_val, UINT32 out_mask,UINT32 reg_index);
#endif

INT32 PAL_sysGpioCtrlGetIrqStatus(UINT32 gpio_pin);  // Jay
INT32 PAL_sysGpioCtrlEnableIrq(UINT32 gpio_pin);     // Jay
INT32 PAL_sysGpioCtrlSetIrqConfig(UINT32 gpio_pin, idl_gpio_interrupt_type_t irq_type);  // Jay
INT32 PAL_sysGpioCtrlClearIrqStatus(UINT32 gpio_pin); // Jay


/*****************************************************************************
 * CLKC Control
 *****************************************************************************/

void PAL_sysClkcInit(void* param);
int PAL_sysClkcSetFreq(PAL_SYS_CLKC_ID_T clk_id, unsigned int output_freq);
int PAL_sysClkcGetFreq(PAL_SYS_CLKC_ID_T clk_id);

/*****************************************************************************
 * MISC
 *****************************************************************************/

unsigned int PAL_sysGetChipVersionInfo(void);

typedef struct module_info {
        Uint32 version;
        Uint32 base_addr;
} MOD_INFO_T;

#if defined (CONFIG_MACH_PUMA5)
typedef struct board_info {
        MOD_INFO_T modules[MAX_MODULES];
}BOARD_INFO_T;

extern BOARD_INFO_T soc[];

PAL_Result PAL_sysProbeAndPrep(Uint32 version, Uint32 base_addr, void *param);
PAL_Result avalanche_device_prepare(Uint32 module_id, Uint32 base_addr, BOARD_ID board_variant, void *param);
#endif
/*****************************************************************************
 * CACHE
 *****************************************************************************/

int PAL_sysCacheInvalidate(PAL_OsMemAddrSpace addrSpace,
                             void *mem_start_ptr,
                             unsigned int num_bytes);

int PAL_sysCacheFlush(PAL_OsMemAddrSpace addrSpace,
                        	void *mem_start_ptr,
                        	unsigned int num_bytes);

int PAL_sysCacheFlushAndInvalidate(PAL_OsMemAddrSpace addrSpace,
                             void *mem_start_ptr,
                             unsigned int num_bytes);
#if defined (CONFIG_MACH_PUMA5)
#include <asm-arm/arch-avalanche/generic/pal_sysPsc.h>
#endif
#include "pal_sysWdtimer.h"
#include "pal_sysTimer16.h"
#endif
