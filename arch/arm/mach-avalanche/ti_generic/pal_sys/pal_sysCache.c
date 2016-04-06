/*
 *
 * pal_sysCache.c 
 * Description:
 * ARM1176 cache coherncy APIs
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

/** \file   pal_sysCache.c
    \brief  ARM1176 cache coherency APIs

	FOR DOCUMENTATION OF FUNCTIONS: Refer file pal_sys.h
	
 
    \author     Mansoor Ahamed
    \version    0.1
 */

#include <asm-arm/arch-avalanche/generic/pal.h>


#if defined(CONFIG_CACHE_DEBUG)
#define DEBUG(fmt,arg...)  printk(KERN_EMERG fmt , ##arg); 
#else 
#define DEBUG(fmt,arg...) 
#endif

#define ENTER DEBUG("[Ei %s-%d] \n", __FUNCTION__, __LINE__); 
#define EXIT  DEBUG("[Ex %s-%d] \n", __FUNCTION__, __LINE__);

/* uncomment line below if you want to use OS cache functions */
//#define CONFIG_NO_PAL_SYS_CACHE 1

#ifndef CONFIG_NO_PAL_SYS_CACHE
/* Hints
 * ----
 * Flush		:	Applies to write-back data caches, and means that if the cache 
 * 					line contains stored data that has not yet been written out to
 * 					main memory, it is written to main memory now. In ARM architecture
 *					term "clean" is used for flush
 *
 * Invalidate	:	Means that the cache line is marked as invalid, so that no cache
 * 					hits occur for that line until it is re-allocated to an address.
 * 					For write back data caches, this does not include cleaning the 
 * 					cache line unless that is also started.
 *
 * Parameters	:	Address parameters passed to flush() and invalidate() functions are 
 * 					virtual addresses. Also, If a cache line is not found
 * 					then there is a chance for an interrupt. The caller and 
 * 					page fault handler should keep this in mind
 *
 * Inline asm	:	The "cc" flag used in inline assembly is used to tell GCC that I
 * 					have clobbered the conditional flags.
 */





#define CACHE_LINE_SIZE 32

/* This instruction completes when all explicit memory transactions 
 * occurring in program order before this instruction is completed. 
 * No instructions occurring in program order after this instruction
 * are executed until this instruction completes 					
 */
#define PAL_sysCacheDrainWriteBuffer() \
		{	\
			__asm__ ("mcr p15, 0, %0, c7, c10, 4" : : "r" (0) : "cc");	\
		}


/* This macro flushes the complete Branch Target Cache */
#define PAL_sysCacheFlushBTC()	\
		{	\
			__asm__ ("mcr p15, 0, %0, c7, c5, 6" : : "r" (0) : "cc");	\
		}

int PAL_sysCacheInvalidate(PAL_OsMemAddrSpace addrSpace,
                                  void *mem_start_ptr,
                                  UINT32 num_bytes)
{
    unsigned long mem_end_ptr = (unsigned long)(mem_start_ptr) + num_bytes - 1;

	if (0 == num_bytes)
	{
	    return -1;
	}
	
    if(addrSpace == PAL_OSMEM_ADDR_DAT)
    {
		/* Invalidate data cache */
		__asm__ ("mcrr   p15, 0, %1, %0, c6"
       		:
			: "r" ((unsigned long)mem_start_ptr), 
			  "r" (mem_end_ptr)
			: "cc");
	
		/* Drain write buffer */
		PAL_sysCacheDrainWriteBuffer();
    } 
	else if(addrSpace == PAL_OSMEM_ADDR_PRG) 
	{
		/* Invalidate instruction cache */
		__asm__ ("mcrr   p15, 0, %1, %0, c5"
       		:
			: "r" ((unsigned long)mem_start_ptr),
			  "r" (mem_end_ptr)
			: "cc");

		/* As per ARM11 reference manual we have to flush selective branch 
		 * target cache entries after invalidating I-CACHE : TODO
		 * For time being i'm flushing the whole branch target cache
		 */
		PAL_sysCacheFlushBTC();

		/* Drain write buffer */
		PAL_sysCacheDrainWriteBuffer();
    } 
	else 
	{
        return -1;
    }

    return 0;            
}



int PAL_sysCacheFlush(PAL_OsMemAddrSpace addrSpace,
                             void *mem_start_ptr,
                             UINT32 num_bytes)
{
    unsigned long mem_end_ptr = (unsigned long)(mem_start_ptr) + num_bytes - 1;

	
    if(addrSpace == PAL_OSMEM_ADDR_DAT)
    {
		if (0 == num_bytes)
		{
		    return -1;
		}
		
		/* Flush data cache */
		__asm__ ("mcrr   p15, 0, %1, %0, c12"
       		:
			: "r" ((unsigned long)mem_start_ptr),
			  "r" (mem_end_ptr)
			: "cc");
	
		/* Drain write buffer */
		PAL_sysCacheDrainWriteBuffer();

    } 
	else if(addrSpace == PAL_OSMEM_ADDR_PRG) 
	{
		/* First, I'm worried if we have a self modifying code :), even then 
		 * the instructions should be in the data cache
		 * Second, ARM11 doesn't support I-CACHE flush instruction (note sure)
		 * It supports only flushing of Branch Target Cache and i'm doing the same here.
		 * TODO 
		 */
		PAL_sysCacheFlushBTC();

		/* Drain write buffer */
		PAL_sysCacheDrainWriteBuffer();
    } 
	else 
	{
        return -1;
    }
	return 0;
}

int PAL_sysCacheFlushAndInvalidate(PAL_OsMemAddrSpace addrSpace,
                             void *mem_start_ptr,
                             UINT32 num_bytes)
{
    unsigned long mem_end_ptr = (unsigned long)(mem_start_ptr) + num_bytes - 1;

	if (0 == num_bytes)
	{
	    return -1;
	}
	
    if(addrSpace == PAL_OSMEM_ADDR_DAT)
    {
		/* Flush and Invalidate data cache */
		__asm__ ("mcrr   p15, 0, %1, %0, c14"
       		:
			: "r" ((unsigned long)mem_start_ptr),
			  "r" (mem_end_ptr)
			: "cc");
	
		/* Drain write buffer */
		PAL_sysCacheDrainWriteBuffer();
    } 
	else if(addrSpace == PAL_OSMEM_ADDR_PRG) 
	{
		/* First, I'm worried if we have a self modifying code :), even then 
		 * the instructions should be in the data cache
		 * Second, ARM11 doesn't support I-CACHE flush instruction (note sure)
		 * It supports only flushing of Branch Target Cache and i'm doing the same here.
		 * Also, i'm not doing a selective BTC flush : TODO 
		 */
		PAL_sysCacheFlushBTC();
	
		/* Invalidate instruction cache */
		__asm__ ("mcrr   p15, 0, %1, %0, c5"
       		:
			: "r" ((unsigned long)mem_start_ptr),
			  "r" (mem_end_ptr)
			: "cc");


		/* As per ARM11 reference manual we have to flush selective branch 
		 * target cache entries after invalidating I-CACHE. We have already
		 * flushed the BTC before invalidating but i'm doing it again under the 
		 * assumption that we might get an interrupt in-between.
		 * Also, i'm not doing a selective BTC flush : TODO
		 */
		PAL_sysCacheFlushBTC();

		/* Drain write buffer */
		PAL_sysCacheDrainWriteBuffer();
    }
	else
	{
        return -1;
    }
	return 0;
}
#else  /* CONFIG_NO_PAL_SYS_CACHE */
int PAL_sysCacheInvalidate(PAL_OsMemAddrSpace addrSpace,
                                  void *mem_start_ptr,
                                  UINT32 num_bytes)
{
	return PAL_osCacheInvalidate(addrSpace, (UINT32)mem_start_ptr, num_bytes);
}

int PAL_sysCacheFlush(PAL_OsMemAddrSpace addrSpace,
                             void *mem_start_ptr,
                             UINT32 num_bytes)
{
	return PAL_osCacheFlush(addrSpace, (UINT32)mem_start_ptr, num_bytes);
}

int PAL_sysCacheFlushAndInvalidate(PAL_OsMemAddrSpace addrSpace,
                             void *mem_start_ptr,
                             UINT32 num_bytes)
{
	return PAL_osCacheFlushAndInvalidate(addrSpace, (UINT32)mem_start_ptr, num_bytes);
}



#endif /* CONFIG_NO_PAL_SYS_CACHE */

