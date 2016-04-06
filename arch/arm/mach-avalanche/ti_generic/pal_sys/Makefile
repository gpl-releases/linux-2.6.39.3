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

EXTRA_CFLAGS += -I$(TOPDIR)/include/asm/arch/generic

obj-$(CONFIG_ARM_AVALANCHE_TIMER16) += pal_sysTimer16.o 

obj-y 	+=  pal_sysCache.o \
            pal_sysWdtimer.o

obj-$(CONFIG_MACH_PUMA5) += pal_sysMisc.o
