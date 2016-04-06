/*
 *
 * pformCfg.h
 * Description:
 * platform configuration file
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

#ifndef __PFORM_CFG_H__
#define __PFORM_CFG_H__

#include <linux/module.h>

#include <asm-arm/arch-avalanche/generic/soc.h>
#include <asm-arm/arch-avalanche/generic/ti_gpl_wrapper.h>
#include <asm-arm/arch-avalanche/generic/ramtest.h>

#if defined(CONFIG_MACH_PUMA5)      /* For Puma-5 SoC */
#include <asm-arm/arch-avalanche/puma5/puma5.h>
#include <asm-arm/arch-avalanche/puma5/puma5_clk_cntl.h>
#include <asm-arm/arch-avalanche/puma5/puma5_boards.h>
#include <asm-arm/arch-avalanche/generic/avalanche_io_power_cntl.h>
#if defined(CONFIG_ARM_AVALANCHE_INTC)
#include <asm-arm/arch-avalanche/puma5/puma5_intc.h>
#endif
#include <asm-arm/arch-avalanche/puma5/puma5_cppi.h>
#include <asm-arm/arch-avalanche/generic/pal_cppi41.h>
#include <asm-arm/arch-avalanche/puma5/puma5_spi.h>

#elif defined(CONFIG_MACH_PUMA6)     /* For Puma-6 SoC */

#include <asm-arm/arch-avalanche/puma6/puma6.h>
#include <asm-arm/arch-avalanche/puma6/puma6_clk_cntl.h>
#include <asm-arm/arch-avalanche/puma6/puma6_boards.h>
#include <asm-arm/arch-avalanche/generic/avalanche_io_power_cntl.h>
#if defined(CONFIG_ARM_AVALANCHE_INTC)
#include <asm-arm/arch-avalanche/puma6/puma6_intc.h>
#endif
#include <asm-arm/arch-avalanche/puma6/puma6_cppi.h>
#include <asm-arm/arch-avalanche/generic/pal_cppi41.h>
#include <asm-arm/arch-avalanche/puma6/puma6_spi.h>
#include <asm-arm/arch-avalanche/puma6/puma6_bootcfg_ctrl.h>
#endif /* Puma-6 or Puma-5 SoC includes */


#if defined(CONFIG_ARM_AVALANCHE_INTC)
#include <asm-arm/arch-avalanche/generic/avalanche_intc.h>
#include <asm-arm/arch-avalanche/generic/avalanche_intd.h>
#endif

#if defined(CONFIG_ARM_AVALANCHE_PCI)
#include <asm/arch/generic/avalanche_pci.h>
#endif

#if defined(CONFIG_ARM_AVALANCHE_QUICK_IIC)
#include <asm/arch/generic/avalanche_i2c.h>
#endif

#include <asm-arm/arch-avalanche/generic/haltypes.h>

#if defined(CONFIG_ARM_AVALANCHE_LED)
#include <asm/arch/generic/led_ioctl.h>
#include <asm/arch/generic/led_config.h>
#include <asm-arm/arch-avalanche/generic/led_hal.h>
#endif

#if defined(CONFIG_ARM_AVALANCHE_COLORED_LED)
#include <asm-arm/arch-avalanche/generic/led_hal.h>
#endif

#if defined(CONFIG_DMA_NONCOHERENT)
#ifdef CONFIG_K0_COHERENCY_ALGO_WT_WA
#define MIPS_4KC
#else
#define MIPS_4KEC
#endif
#endif

#if defined(CONFIG_CPU_BIG_ENDIAN)
#define PAL_NATIVE_ENDIAN_BIG
#endif


#if !defined(CONFIG_DMA_NONCOHERENT) && !defined(CONFIG_MIPS_UNCACHED)
#error "Error in MIPS cache configuration. PAL SYS cache APIs will not work."
#endif

#endif /* __PFORM_CFG_H__ */
