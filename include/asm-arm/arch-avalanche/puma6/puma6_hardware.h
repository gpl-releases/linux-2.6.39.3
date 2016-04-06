/* Copyright 2008, Texas Instruments Incorporated
 *
 * This program has been modified from its original operation by 
 * Texas Instruments to do the following:
 * Explanation of modification.
 * derived from:
 * linux/include/asm-arm/arch-ebsa110/hardware.h
 *  
 *  
 *
 * THIS MODIFIED SOFTWARE AND DOCUMENTATION ARE PROVIDED
 * "AS IS," AND TEXAS INSTRUMENTS MAKES NO REPRESENTATIONS
 * OR WARRENTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO, WARRANTIES OF MERCHANTABILITY OR FITNESS FOR ANY
 * PARTICULAR PURPOSE OR THAT THE USE OF THE SOFTWARE OR
 * DOCUMENTATION WILL NOT INFRINGE ANY THIRD PARTY PATENTS,
 * COPYRIGHTS, TRADEMARKS OR OTHER RIGHTS.
 * See The GNU General Public License for more details.
 *
 * These changes are covered under version 2 of the GNU General Public License,
 * dated June 1991.
 */


#ifndef __PUMA6_HARDWARE_H
#define __PUMA6_HARDWARE_H

#include <asm/arch/memory.h>
#include <asm/arch/vmalloc.h>

/*
 * Mapping areas size
 */
#define DEVICE_REGION_1_SIZE	 (0x08000000)    /* DOCSIS IP IO   */
#define SOC_DEVICE_REGION_SIZE   (0x04000000)    /* SoC IO         */
#define SOC_SPI_MEM_REGION_SIZE  (0x04000000)    /* SoC SPI Mem    */
#define INTC_DEVICE_REGION_SIZE  (0x00100000)    /* DOCSIS IP INTC */

#define DEVICE_FREE_START_1  	(0xd0000000)
#define DEVICE_FREE_START_2	    (DEVICE_FREE_START_1 + DEVICE_REGION_1_SIZE)

#define PCIO_BASE		(0)
#define IO_RELOC_ADDR	(DEVICE_FREE_START_1)	
#define IO_OFFSET		(DEVICE_FREE_START_1) /* Virtual I/O - Phys I/O */


/*-------------------------------*/
/*   Puma 6 Peripherals MMU MAP  */
/*-------------------------------*/
/* 0x0 - 0x8M   --> DOCSIS_IP IO */
/* 0x8M - 0xCM  --> SPI MM       */ 
/* 0xCM - 0x10M --> SoC IO       */ 
/* 0x20M        --> INTC         */ 
/*-------------------------------*/

/* SRAM, BootCFG, UART, WDT, etc., Region */
#define IO_PHY        	(0x0)
#define IO_START		(IO_PHY)
#define IO_SIZE         (DEVICE_REGION_1_SIZE)
#define IO_VIRT         (DEVICE_FREE_START_1 + IO_PHY)
#define IO_BASE			(IO_VIRT)
#define IO_END			(IO_VIRT + IO_SIZE - 1)

/* Memory Mapped serial flash 0 Region */
#define MM_SPI_PHY	    (0x08000000)
#define MM_SPI_SIZE	    (SOC_SPI_MEM_REGION_SIZE) 
#define MM_SPI_VIRT	    (IO_END + 1)
#define MM_SPI_END	    (MM_SPI_VIRT + MM_SPI_SIZE - 1)

/* SoC IO Region - SPI, eMMC,...  */
#define SOC_IO_PHY	    (0x0C000000)
#define SOC_IO_START	(SOC_IO_PHY) 
#define SOC_IO_SIZE     (SOC_DEVICE_REGION_SIZE)   /* 64M - SoC IO */
#define SOC_IO_VIRT	    (MM_SPI_END + 1)
#define SOC_IO_BASE		(SOC_IO_VIRT)
#define SOC_IO_END	    (SOC_IO_VIRT + SOC_IO_SIZE - 1)

/* Interrupt controller Region */
#define INTC_PHY		(0x20000000)
#define INTC_SIZE		(INTC_DEVICE_REGION_SIZE)
#define INTC_VIRT		(SOC_IO_END + 1)
#define INTC_END		(INTC_VIRT + INTC_SIZE - 1)

/* TODO : NEED to take care of Region 2 */
#define IO_ADDRESS(pa)      ((pa) + IO_OFFSET)
#define IO_PHY2VIRT(pa)   	(IO_ADDRESS(pa))
#define IO_VIRT2PHY(va)     ((va) - IO_OFFSET)

#define DOCSIS_DSGx_INTERNAL_ADDR_MASK  ((unsigned long)(1<<19)-1)
#define IO_VIRT2PHY_DSGx(va) ((va) & DOCSIS_DSGx_INTERNAL_ADDR_MASK)

#endif /* __PUMA6_HARDWARE_H */

