#
# linux/arch/arm/boot/compressed/Makefile
#
# create a compressed vmlinuz image from the original vmlinux
#

OBJS		=

# Ensure that mmcif loader code appears early in the image
# to minimise that number of bocks that have to be read in
# order to load it.
ifeq ($(CONFIG_ZBOOT_ROM_MMCIF),y)
ifeq ($(CONFIG_ARCH_SH7372),y)
OBJS		+= mmcif-sh7372.o
endif
endif

AFLAGS_head.o += -DTEXT_OFFSET=$(TEXT_OFFSET)
HEAD	= head.o

ifeq ($(CONFIG_KERNEL_COMPRESS_7ZIP),y)
OBJS	+= Decoder.o
else
OBJS	+= misc.o decompress.o
endif

FONTC	= $(srctree)/drivers/video/console/font_acorn_8x8.c

FONT = $(addprefix ../../../../drivers/video/console/, font_acorn_8x8.o)

#
# Architecture dependencies
#
ifeq ($(CONFIG_ARCH_ACORN),y)
OBJS		+= ll_char_wr.o font.o
endif

ifeq ($(CONFIG_ARCH_SHARK),y)
OBJS		+= head-shark.o ofw-shark.o
endif

ifeq ($(CONFIG_ARCH_L7200),y)
OBJS		+= head-l7200.o
endif

ifeq ($(CONFIG_ARCH_CLPS7500),y)
HEAD		= head-clps7500.o
endif

ifeq ($(CONFIG_ARCH_P720T),y)
# Borrow this code from SA1100
OBJS		+= head-sa1100.o
endif

ifeq ($(CONFIG_ARCH_SA1100),y)
OBJS		+= head-sa1100.o
endif

ifeq ($(CONFIG_ARCH_VT8500),y)
OBJS		+= head-vt8500.o
endif

ifeq ($(CONFIG_CPU_XSCALE),y)
OBJS		+= head-xscale.o
endif

ifeq ($(CONFIG_PXA_SHARPSL_DETECT_MACH_ID),y)
OBJS		+= head-sharpsl.o
endif
ifeq ($(CONFIG_ARCH_AT91RM9200),y)
OBJS		+= head-at91rm9200.o
endif

ifeq ($(CONFIG_MACH_OMAP_PERSEUS2),y)
OBJS		+= head-omap.o
endif

ifeq ($(CONFIG_CPU_ENDIAN_BE32),y)
ifeq ($(CONFIG_CPU_CP15),y)
OBJS		+= big-endian.o
else
# The endian should be set by h/w design.
endif
endif

ifeq ($(CONFIG_ARCH_SHMOBILE),y)
OBJS		+= head-shmobile.o
endif

#
# We now have a PIC decompressor implementation.  Decompressors running
# from RAM should not define ZTEXTADDR.  Decompressors running directly
# from ROM or Flash must define ZTEXTADDR (preferably via the config)
# FIXME: Previous assignment to ztextaddr-y is lost here. See SHARK
ifeq ($(CONFIG_ZBOOT_ROM),y)
ZTEXTADDR	:= $(CONFIG_ZBOOT_ROM_TEXT)
ZBSSADDR	:= $(CONFIG_ZBOOT_ROM_BSS)
else
ZTEXTADDR	:= 0
ZBSSADDR	:= ALIGN(4)
endif

SEDFLAGS	= s/TEXT_START/$(ZTEXTADDR)/;s/BSS_START/$(ZBSSADDR)/

suffix_$(CONFIG_KERNEL_GZIP) = gzip
suffix_$(CONFIG_KERNEL_LZO)  = lzo
suffix_$(CONFIG_KERNEL_LZMA) = lzma

ifdef ($(CONFIG_KERNEL_COMPRESS_7ZIP),y)
targets       := vmlinux vmlinux.lds piggy.gz piggy.o $(FONT) \
		 head.o Decoder.o $(OBJS)
else
targets       := vmlinux vmlinux.lds piggy.gz piggy.o $(FONT) \
		 head.o misc.o $(OBJS)
endif
EXTRA_CFLAGS  := -fpic
EXTRA_AFLAGS  :=


# Make sure files are removed during clean
extra-y       += piggy.gzip piggy.lzo piggy.lzma lib1funcs.S

ifeq ($(CONFIG_FUNCTION_TRACER),y)
ORIG_CFLAGS := $(KBUILD_CFLAGS)
KBUILD_CFLAGS = $(subst -pg, , $(ORIG_CFLAGS))
endif

ccflags-y := -fpic -fno-builtin
asflags-y := -Wa,-march=all

# Provide size of uncompressed kernel to the decompressor via a linker symbol.
LDFLAGS_vmlinux = --defsym _image_size=$(shell stat -c "%s" $(obj)/../Image)
ifneq ($(PARAMS_PHYS),)
LDFLAGS_vmlinux += --defsym params_phys=$(PARAMS_PHYS)
endif
# Don't allow any static data in misc.o, which
# would otherwise mess up our GOT table
#CFLAGS_Decoder.o := -Dstatic=
# Don't allow any static data in misc.o, which
# would otherwise mess up our GOT table
#CFLAGS_misc.o := -Dstatic=


# Supply ZRELADDR to the decompressor via a linker symbol.
ifneq ($(CONFIG_AUTO_ZRELADDR),y)
LDFLAGS_vmlinux += --defsym zreladdr=$(ZRELADDR)
endif
ifeq ($(CONFIG_CPU_ENDIAN_BE8),y)
LDFLAGS_vmlinux += --be8
endif
# ?
LDFLAGS_vmlinux += -p
# Report unresolved symbol references
LDFLAGS_vmlinux += --no-undefined
# Delete all temporary local symbols
LDFLAGS_vmlinux += -X
# Next argument is a linker script
LDFLAGS_vmlinux += -T

# For __aeabi_uidivmod
#lib1funcs = $(obj)/lib1funcs.o

$(obj)/lib1funcs.S: $(srctree)/arch/$(SRCARCH)/lib/lib1funcs.S FORCE
	$(call cmd,shipped)

$(obj)/vmlinux: $(obj)/vmlinux.lds $(obj)/$(HEAD) $(obj)/piggy.$(suffix_y).o \
	 	$(addprefix $(obj)/, $(OBJS)) $(lib1funcs) FORCE
	$(call if_changed,ld)
	@:

ifneq ($(CONFIG_KERNEL_COMPRESS_7ZIP),y)
$(obj)/piggy.$(suffix_y): $(obj)/../Image FORCE
	$(call if_changed,$(suffix_y))
else
#$(obj)/piggy.gz: FORCE
$(obj)/piggy.$(suffix_y): FORCE

ifdef SEVENZIP
	rm -f $(obj)/7zip
	cp -f $(SEVENZIP) $(obj)/7zip
else
	rm -f $(obj)/7zip
#	make -C $(objtree)/../../../tools/src/7zip/ clean
#	make -C $(objtree)/../../../tools/src/7zip/
	cp $(objtree)/../../../../tools/src/7zip/7zip $(obj)/7zip
endif
#	$(call if_changed, ../../../../../../../tools/bin/7zip)
	$(obj)/7zip $(obj)/../Image $(obj)/piggy.$(suffix_y)
	rm -f $(obj)/7zip
endif


$(obj)/piggy.$(suffix_y).o:  $(obj)/piggy.$(suffix_y) FORCE

CFLAGS_font.o := -Dstatic=

$(obj)/vmlinux.lds: $(obj)/vmlinux.lds.in arch/arm/boot/Makefile $(KCONFIG_CONFIG)
	@sed "$(SEDFLAGS)" < $< > $@

$(obj)/font.c: $(FONTC)
	$(call cmd,shipped)

