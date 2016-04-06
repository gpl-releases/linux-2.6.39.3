################################################################################
#
# TBD
#
################################################################################

#
# Makefile for the CatMountain L2Switch Network Interface Driver
#

# Required definitions
ifndef TARGET_HOME
TARGET_HOME := /vobs/ua_sdk
endif

ifndef KERNELRELEASE
# Include TI build definition file (optional) only in non-kernel compilation mode
-include $(TARGET_HOME)/TI_Build.make
endif

# Kernel directory
ifndef KERNEL_DIR
KERNEL_DIR := $(TARGET_HOME)/kernel/ti/linux-2.6.39.3/src
endif

# File system path, where to install the module
ifndef TI_filesystem_path
TI_filesystem_path := $(TARGET_HOME)/build/dsdk/fs
endif

# TI include export directory
ifndef TI_include
TI_include := $(TARGET_HOME)/ti/include
endif

# Include directories
INCLUDES := \
	-I$(KERNEL_DIR)/include/asm-arm/arch-avalanche/puma6 \
	-I$(KERNEL_DIR)/include/asm-arm/arch-avalanche/generic \
	-I$(TI_include)

export EXTRA_CFLAGS += $(INCLUDES)

obj-m += cat_l2switch_netdev.o

cat_l2sw_nid-objs := cat_l2switch_netdev_main.o
