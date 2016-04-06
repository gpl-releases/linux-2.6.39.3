/*
 *
 * avalanche_io_power_cntl.h 
 * Description:
 * io power control header file
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
/** \file   avalanche_io_power_cntl.h
    \brief  avalanche IO power control header file
	
    \author     Mansoor Ahamed (mansoor.ahamed@ti.com)
    \version    0.1
 */


#ifndef __AVALANCHE_IO_POWER_CNTL_H__
#define __AVALANCHE_IO_POWER_CNTL_H__
/** \enum AVALANCHE_IO_POWER_MODULE_T
	\brief Enum for modules which can do IO power management using IO PDCR
	\TODO This enum should be moved to SOC specific header file
*/
typedef enum AVALANCHE_IO_POWER_MODULE_tag 
{
	IOPM_UART = 0,
	IOPM_TDM_CODEC,
	IOPM_GPIO,
	IOPM_I2C,
	IOPM_GMII,
	IOPM_OOB,
	IOPM_EPGA,
	IOPM_BBU,
	IOPM_ASYNC_EMIF,
	IOPM_AGC,
	IOPM_TAGC,
	IOPM_EXT_INT,
	IOPM_CLKOUT0,
	IOPM_CLKOUT1,
	IOPM_VLYNQ,
	AVALANCHE_MAX_IOPM_MODULES
}AVALANCHE_IO_POWER_MODULE_T;

/** \enum AVALANCHE_IO_POWER_MODULE_T
	\brief Enum for modules which can do IO power management using IO PDCR
*/
typedef enum AVALANCHE_IO_POWER_MODE_tag 
{
	AVALANCHE_IO_POWER_DOWN = 0,
	AVALANCHE_IO_POWER_UP
}AVALANCHE_IO_POWER_MODE_T;


/*! \fn int avalanche_setIOPowerMode(AVALANCHE_IO_POWER_MODULE_T io_module_id, AVALANCHE_IO_POWER_MODE_T mode)
    \brief This API ise used to power down or power up a specific module.
    \param io_module_id Unique module id which has to be operated on.
    \param mode The new mode (AVALANCHE_IO_POWER_DOWN or AVALANCHE_IO_POWER_UP).
	\return Returns 0 on success else returns a negative value
*/
int avalanche_setIOPowerMode(AVALANCHE_IO_POWER_MODULE_T io_module_id, AVALANCHE_IO_POWER_MODE_T mode);

/*! \fn AVALANCHE_IO_POWER_MODE_T avalanche_getIOPowerMode(AVALANCHE_IO_POWER_MODULE_T io_module_id)
    \brief This API ise used to retrieve current mode (power down or up) of a module.
    \param io_module_id Unique module id which has to be operated on.
    \return mode Modules power mode (AVALANCHE_IO_POWER_DOWN or AVALANCHE_IO_POWER_UP).
*/
AVALANCHE_IO_POWER_MODE_T avalanche_getIOPowerMode(AVALANCHE_IO_POWER_MODULE_T io_module_id);

#endif /* __AVALANCHE_IO_POWER_CNTL_H__ */
