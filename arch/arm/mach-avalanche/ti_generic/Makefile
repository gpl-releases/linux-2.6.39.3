#/*
#*
#* Makefile
#* Description:
#*
#*
#*
#* Copyright (C) 2008, Texas Instruments, Incorporated
#*
#*  This program is free software; you can distribute it and/or modify it
#*  under the terms of the GNU General Public License (Version 2) as
#*  published by the Free Software Foundation.
#*
#*  This program is distributed in the hope it will be useful, but WITHOUT
#*  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
#*  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
#*  for more details.
#*
#*  You should have received a copy of the GNU General Public License along
#*  with this program; if not, write to the Free Software Foundation, Inc.,
#*  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.
#*
#*/

EXTRA_CFLAGS += -I$(srctree)/include/asm/arch/generic

obj-y := avalanche_generic_setup.o avalanche_misc.o avalanche_intc.o ti_gpl_wrapper.o \
		 pal_cppi41.o avalanche_intd.o ramtest.o
obj-$(CONFIG_MACH_PUMA5) += avalanche_io_power_cntl.o
obj-$(CONFIG_ARM_AVALANCHE_VLYNQ) += avalanche_vlynq_intc.o pal_vlynq.o \
                                      pal_vlynqDev.o pal_vlynqIoctl.o \
                                      avalanche_vlynq_config.o
									  
obj-y += pal_sys/
