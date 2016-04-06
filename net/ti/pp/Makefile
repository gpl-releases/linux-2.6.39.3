 # Makefile
 #
 # Description:
 # Makefile to build packet processor components.
 #
 # Copyright (C) <2008>, Texas Instruments, Incorporated
 #
 #  This program is free software; you can distribute it and/or modify it
 #  under the terms of the GNU General Public License (Version 2) as
 #  published by the Free Software Foundation.
 #
 #  This program is distributed in the hope it will be useful, but WITHOUT
 #  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 #  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 #  for more details.
 #
 #  You should have received a copy of the GNU General Public License along
 #  with this program; if not, write to the Free Software Foundation, Inc.,
 #  59 Temple Place - Suite 330, Boston MA 02111-1307, USA.

# PPM Deliverables.
obj-y := ti_ppm.o listlib.o ti_ppm_os.o hash.o ti_hil_core.o 

# Sample Profiles. 
obj-$(CONFIG_TI_HIL_PROFILE_INTRUSIVE) += hil_intrusive.o
obj-$(CONFIG_TI_HIL_PROFILE_STATIC)    += hil_static_profile.o

# Services (may be profile dependent 
obj-$(CONFIG_TI_HIL_PROFILE_INTRUSIVE) += ti_pp_path.o

#CFLAGS += -I$(srctree)/include/asm-arm/arch/generic -I$(srctree)/include/linux
