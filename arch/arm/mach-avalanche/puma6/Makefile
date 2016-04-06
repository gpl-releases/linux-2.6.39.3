#/*
#*
#* Makefile
#* Description:
#*
#*/

EXTRA_CFLAGS += -Iinclude/asm-arm/arch-avalanche/generic
EXTRA_CFLAGS += -Iinclude/linux
EXTRA_CFLAGS += -Iinclude/asm-arm/arch-avalanche/puma6

obj-y := puma6_core.o \
         puma6_board_setup.o \
         puma6_cppi.o \
         puma6_intd.o \
         puma6_clk_cntl.o \
         puma6_cru_ctrl.o \
         puma6_dect_page.o \
         puma6_bootcfg_ctrl.o

obj-$(CONFIG_AVALANCHE_GENERIC_GPIO)    += puma6_gpio_ctrl.o
obj-$(CONFIG_ARM_AVALANCHE_COLORED_LED) += puma6_board_led.o
obj-$(CONFIG_ARM_AVALANCHE_SPI) 	+= puma6_spi.o
obj-$(CONFIG_ARM_AVALANCHE_PPD) 	+= puma6_pp.o
obj-$(CONFIG_HW_MUTEXES)        	+= hw_mutex/
obj-$(CONFIG_ARM_AVALANCHE_MMC) 	+= puma6_mmc.o
obj-$(CONFIG_IOSFSB_DRV)        	+= iosfsb_drv/
obj-$(CONFIG_P_UNIT_DRV)        	+= p_unit_drv/
obj-y                           	+= arm_atom_mbx/
