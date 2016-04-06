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


#ifndef __PUMA5_HARDWARE_H
#define __PUMA5_HARDWARE_H

#include <asm/arch/memory.h>
#include <asm/arch/vmalloc.h>
/*
 * Mapping areas
 */

#define DEVICE_REGION_1_SIZE	(0x10000000)

#define DEVICE_FREE_START_1	(0xd0000000)
#define DEVICE_FREE_START_2	(DEVICE_FREE_START_1 + DEVICE_REGION_1_SIZE)

#if 0
#if defined(CONFIG_MACH_PUMA5_VOLCANO)
	#define DEVICE_FREE_START_1	(0xd0000000)
	#define DEVICE_FREE_START_2	(DEVICE_FREE_START_1 + DEVICE_REGION_1_SIZE)
#else
	#define DEVICE_FREE_START_1	(VMALLOC_END)
	#define DEVICE_FREE_START_2	(VMALLOC_END + DEVICE_REGION_1_SIZE)
#endif
#endif

#define PCIO_BASE		(0)
#define IO_RELOC_ADDR	(DEVICE_FREE_START_1)	
#define IO_OFFSET		(DEVICE_FREE_START_1) /* Virtual I/O - Phys I/O */

/* BootCFG, UART, WDT, etc., Region */
#define IO_PHY        	(0x0)
#define IO_START		(IO_PHY)
#define IO_SIZE         (DEVICE_REGION_1_SIZE)
#define IO_VIRT         (DEVICE_FREE_START_1 + IO_PHY)
#define IO_BASE			(IO_VIRT)
#define IO_END			(IO_VIRT + IO_SIZE - 1)


#if 0 /* SR, Vlynq and other devices fall under IO_START (given above). Information provided below (inside #if 0) is for understanding pupose */
/* Session Router (SR) Region */
#define SR_PHY			(0x03000000)
#define SR_SIZE			(0x00500000)
#define SR_VIRT			(DEVICE_FREE_REGION_START + SR_PHY)
#define SR_END			(SR_VIRT + SR_SIZE - 1)

/* BootCFG, UART, WDT, etc., Region */
#define IO_PHY        	(0x08600000)
#define IO_START		(IO_PHY)
#define IO_SIZE         (0x00100000)
#define IO_VIRT         (SR_END + 1)
#define IO_BASE			(IO_VIRT)
#define IO_END			(IO_VIRT + IO_SIZE - 1)

/* Vlynq Region */
#define VLYNQ_PHY		(0x0c000000)
#define VLYNQ_SIZE		(0x04000000)
#define VLYNQ_VIRT		(IO_END + 1)
#define VLYNQ_END		(VLYNQ_VIRT + VLYNQ_SIZE - 1)
#endif

/* EMIF 3e Configuratio Region */
#define EMIF3E_PHY		(0x20000000)
#define EMIF3E_SIZE		(0x01000000)
#define EMIF3E_VIRT		(IO_END + 1)
#define EMIF3E_END		(EMIF3E_VIRT + EMIF3E_SIZE - 1)

/* Async EMIF 0 (Parallel Flash 0 ) Region */
#define FLASH_0_PHY		(0x38000000)
#define FLASH_0_SIZE	(0x01000000)
#define FLASH_0_VIRT	(EMIF3E_END + 1)
#define FLASH_0_END		(FLASH_0_VIRT + FLASH_0_SIZE - 1)

/* Async EMIF 1 (Parallel Flash 1 ) Region */
#define FLASH_1_PHY		(0x39000000)
#define FLASH_1_SIZE	(0x01000000)
#define FLASH_1_VIRT	(FLASH_0_END + 1)
#define FLASH_1_END		(FLASH_1_VIRT + FLASH_1_SIZE - 1)

/* Memory Mapped serial flash 0 Region */
#define MM_SPI_0_PHY	(0x48000000)
#define MM_SPI_0_SIZE	(0x01000000)
#define MM_SPI_0_VIRT	(FLASH_1_END + 1)
#define MM_SPI_0_END	(MM_SPI_0_VIRT + MM_SPI_0_SIZE - 1)

/* Memory Mapped serial flash 1 Region */
#define MM_SPI_1_PHY	(0x4c000000)
#define MM_SPI_1_SIZE	(0x01000000)
#define MM_SPI_1_VIRT	(MM_SPI_0_END + 1)
#define MM_SPI_1_END	(MM_SPI_1_VIRT + MM_SPI_1_SIZE - 1)

/* Interrupt controller Region */
#define INTC_PHY		(0x50000000)
#define INTC_SIZE		(0x00100000)
#define INTC_VIRT		(MM_SPI_1_END + 1)
#define INTC_END		(INTC_VIRT + INTC_SIZE - 1)

#if defined(CONFIG_MACH_PUMA5_VOLCANO)
/* Volcano information Region */
#define VOLCANO_PHY		(0xFFF00000)
#define VOLCANO_SIZE	(0x00100000)
#define VOLCANO_VIRT	(INTC_END + 1)
#define VOLCANO_END		(VOLCANO_VIRT + VOLCANO_SIZE - 1)
#endif

/* TODO : NEED to take care of Region 2 */
#define IO_ADDRESS(pa)  	((pa < DEVICE_REGION_1_SIZE) ? (pa + IO_OFFSET) : 0)
#define IO_PHY2VIRT(pa)   	(IO_ADDRESS(pa))
#define IO_VIRT2PHY(va) 	((va < DEVICE_FREE_START_2) ? (va - IO_OFFSET) : 0)

/* DDR CR for EMIF DDR ASYNC mode */
#define REG_DDR_CR              (0x08611B28)
#define EMIF_DDR_DUAL_MODE      (1 << 1)
#define IS_DDR_ASYNCH           ((*((volatile UINT32* )IO_PHY2VIRT(REG_DDR_CR))) & EMIF_DDR_DUAL_MODE)

#endif /* __PUMA5_HARDWARE_H */

