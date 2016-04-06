/*
 *  linux/drivers/mmc/core/mmc.c
 *
 *  Copyright (C) 2003-2004 Russell King, All Rights Reserved.
 *  Copyright (C) 2005-2007 Pierre Ossman, All Rights Reserved.
 *  MMCv4 support Copyright (C) 2006 Philip Langdale, All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


/******************************************************************
 
 Includes Intel Corporation's changes/modifications dated: 07/2011.
 Changed/modified portions - Copyright(c) 2011, Intel Corporation. 

******************************************************************/


#include <linux/err.h>
#include <linux/slab.h>

#include <linux/mmc/host.h>
#include <linux/mmc/card.h>
#include <linux/mmc/mmc.h>

#include "core.h"
#include "bus.h"
#include "mmc_ops.h"

static const unsigned int tran_exp[] = {
	10000,		100000,		1000000,	10000000,
	0,		0,		0,		0
};

static const unsigned char tran_mant[] = {
	0,	10,	12,	13,	15,	20,	25,	30,
	35,	40,	45,	50,	55,	60,	70,	80,
};

static const unsigned int tacc_exp[] = {
	1,	10,	100,	1000,	10000,	100000,	1000000, 10000000,
};

static const unsigned int tacc_mant[] = {
	0,	10,	12,	13,	15,	20,	25,	30,
	35,	40,	45,	50,	55,	60,	70,	80,
};

#define UNSTUFF_BITS(resp,start,size)					\
	({								\
		const int __size = size;				\
		const u32 __mask = (__size < 32 ? 1 << __size : 0) - 1;	\
		const int __off = 3 - ((start) / 32);			\
		const int __shft = (start) & 31;			\
		u32 __res;						\
									\
		__res = resp[__off] >> __shft;				\
		if (__size + __shft > 32)				\
			__res |= resp[__off-1] << ((32 - __shft) % 32);	\
		__res & __mask;						\
	})

/*
 * Given the decoded CSD structure, decode the raw CID to our CID structure.
 */
static int mmc_decode_cid(struct mmc_card *card)
{
	u32 *resp = card->raw_cid;

	/*
	 * The selection of the format here is based upon published
	 * specs from sandisk and from what people have reported.
	 */
	switch (card->csd.mmca_vsn) {
	case 0: /* MMC v1.0 - v1.2 */
	case 1: /* MMC v1.4 */
		card->cid.manfid	= UNSTUFF_BITS(resp, 104, 24);
		card->cid.prod_name[0]	= UNSTUFF_BITS(resp, 96, 8);
		card->cid.prod_name[1]	= UNSTUFF_BITS(resp, 88, 8);
		card->cid.prod_name[2]	= UNSTUFF_BITS(resp, 80, 8);
		card->cid.prod_name[3]	= UNSTUFF_BITS(resp, 72, 8);
		card->cid.prod_name[4]	= UNSTUFF_BITS(resp, 64, 8);
		card->cid.prod_name[5]	= UNSTUFF_BITS(resp, 56, 8);
		card->cid.prod_name[6]	= UNSTUFF_BITS(resp, 48, 8);
		card->cid.hwrev		= UNSTUFF_BITS(resp, 44, 4);
		card->cid.fwrev		= UNSTUFF_BITS(resp, 40, 4);
		card->cid.serial	= UNSTUFF_BITS(resp, 16, 24);
		card->cid.month		= UNSTUFF_BITS(resp, 12, 4);
		card->cid.year		= UNSTUFF_BITS(resp, 8, 4) + 1997;
		break;

	case 2: /* MMC v2.0 - v2.2 */
	case 3: /* MMC v3.1 - v3.3 */
	case 4: /* MMC v4 */
		card->cid.manfid	= UNSTUFF_BITS(resp, 120, 8);
		card->cid.oemid		= UNSTUFF_BITS(resp, 104, 16);
		card->cid.prod_name[0]	= UNSTUFF_BITS(resp, 96, 8);
		card->cid.prod_name[1]	= UNSTUFF_BITS(resp, 88, 8);
		card->cid.prod_name[2]	= UNSTUFF_BITS(resp, 80, 8);
		card->cid.prod_name[3]	= UNSTUFF_BITS(resp, 72, 8);
		card->cid.prod_name[4]	= UNSTUFF_BITS(resp, 64, 8);
		card->cid.prod_name[5]	= UNSTUFF_BITS(resp, 56, 8);
		card->cid.serial	= UNSTUFF_BITS(resp, 16, 32);
		card->cid.month		= UNSTUFF_BITS(resp, 12, 4);
		card->cid.year		= UNSTUFF_BITS(resp, 8, 4) + 1997;
		break;

	default:
		printk(KERN_ERR "%s: card has unknown MMCA version %d\n",
			mmc_hostname(card->host), card->csd.mmca_vsn);
		return -EINVAL;
	}

	return 0;
}

static void mmc_set_erase_size(struct mmc_card *card)
{
	if (card->ext_csd.erase_group_def & 1)
		card->erase_size = card->ext_csd.hc_erase_size;
	else
		card->erase_size = card->csd.erase_size;

	mmc_init_erase(card);
}

/*
 * Given a 128-bit response, decode to our card CSD structure.
 */
static int mmc_decode_csd(struct mmc_card *card)
{
	struct mmc_csd *csd = &card->csd;
	unsigned int e, m, a, b;
	u32 *resp = card->raw_csd;

	/*
	 * We only understand CSD structure v1.1 and v1.2.
	 * v1.2 has extra information in bits 15, 11 and 10.
	 * We also support eMMC v4.4 & v4.41.
	 */
	csd->structure = UNSTUFF_BITS(resp, 126, 2);
	if (csd->structure == 0) {
		printk(KERN_ERR "%s: unrecognised CSD structure version %d\n",
			mmc_hostname(card->host), csd->structure);
		return -EINVAL;
	}

	csd->mmca_vsn	 = UNSTUFF_BITS(resp, 122, 4);
	m = UNSTUFF_BITS(resp, 115, 4);
	e = UNSTUFF_BITS(resp, 112, 3);
	csd->tacc_ns	 = (tacc_exp[e] * tacc_mant[m] + 9) / 10;
	csd->tacc_clks	 = UNSTUFF_BITS(resp, 104, 8) * 100;

	m = UNSTUFF_BITS(resp, 99, 4);
	e = UNSTUFF_BITS(resp, 96, 3);
	csd->max_dtr	  = tran_exp[e] * tran_mant[m];
	csd->cmdclass	  = UNSTUFF_BITS(resp, 84, 12);

	e = UNSTUFF_BITS(resp, 47, 3);
	m = UNSTUFF_BITS(resp, 62, 12);
	csd->capacity	  = (1 + m) << (e + 2);

	csd->read_blkbits = UNSTUFF_BITS(resp, 80, 4);
	csd->read_partial = UNSTUFF_BITS(resp, 79, 1);
	csd->write_misalign = UNSTUFF_BITS(resp, 78, 1);
	csd->read_misalign = UNSTUFF_BITS(resp, 77, 1);
	csd->r2w_factor = UNSTUFF_BITS(resp, 26, 3);
	csd->write_blkbits = UNSTUFF_BITS(resp, 22, 4);
	csd->write_partial = UNSTUFF_BITS(resp, 21, 1);

	if (csd->write_blkbits >= 9) {
		a = UNSTUFF_BITS(resp, 42, 5);
		b = UNSTUFF_BITS(resp, 37, 5);
		csd->erase_size = (a + 1) * (b + 1);
		csd->erase_size <<= csd->write_blkbits - 9;
	}

	return 0;
}

/*
 * Read and decode extended CSD.
 */
static int mmc_read_ext_csd(struct mmc_card *card)
{
	int err;
	u8 *ext_csd;
#ifdef CONFIG_ARCH_GEN3
	int i;
#endif

	BUG_ON(!card);

	if (card->csd.mmca_vsn < CSD_SPEC_VER_4)
		return 0;

	/*
	 * As the ext_csd is so large and mostly unused, we don't store the
	 * raw block in mmc_card.
	 */
	ext_csd = kmalloc(512, GFP_KERNEL);
	if (!ext_csd) {
		printk(KERN_ERR "%s: could not allocate a buffer to "
			"receive the ext_csd.\n", mmc_hostname(card->host));
		return -ENOMEM;
	}

    //printk("[PowerOn] mmc_read_ext_csd: mmc_send_ext_csd.\n");
	err = mmc_send_ext_csd(card, ext_csd);
	if (err) {
		/* If the host or the card can't do the switch,
		 * fail more gracefully. */
		if ((err != -EINVAL)
		 && (err != -ENOSYS)
		 && (err != -EFAULT))
        {
            //printk("[PowerOn] mmc_read_ext_csd: return %d goto out\n",err);
			goto out;
        }

        //printk("[PowerOn] mmc_read_ext_csd: return %d (got EINVAL or ENOSYS or EFAULT)\n",err);

		/*
		 * High capacity cards should have this "magic" size
		 * stored in their CSD.
		 */
		if (card->csd.capacity == (4096 * 512)) {
			printk(KERN_ERR "%s: unable to read EXT_CSD "
				"on a possible high capacity card. "
				"Card will be ignored.\n",
				mmc_hostname(card->host));
		} else {
			printk(KERN_WARNING "%s: unable to read "
				"EXT_CSD, performance might "
				"suffer.\n",
				mmc_hostname(card->host));
			err = 0;
		}

		goto out;
	}

	/* Version is coded in the CSD_STRUCTURE byte in the EXT_CSD register */
    //printk("[PowerOn] mmc_read_ext_csd: card->csd.structure = %d\n",card->csd.structure);
	if (card->csd.structure == 3) {
        int ext_csd_struct = ext_csd[EXT_CSD_STRUCTURE];
		if (ext_csd_struct > 2) {
			printk(KERN_ERR "%s: unrecognised EXT_CSD structure "
				"version %d\n", mmc_hostname(card->host),
					ext_csd_struct);
			err = -EINVAL;
			goto out;
		}
	}

    //printk("[PowerOn] mmc_read_ext_csd: ext_csd[EXT_CSD_REV] = %d\n",ext_csd[EXT_CSD_REV]);
	card->ext_csd.rev = ext_csd[EXT_CSD_REV];
	if (card->ext_csd.rev > 5) {
		printk(KERN_ERR "%s: unrecognised EXT_CSD revision %d\n",
			mmc_hostname(card->host), card->ext_csd.rev);
		err = -EINVAL;
		goto out;
	}

	if (card->ext_csd.rev >= 2) {
        //printk("[PowerOn] mmc_read_ext_csd:card->ext_csd.rev >= 2\n");
		card->ext_csd.sectors =
			ext_csd[EXT_CSD_SEC_CNT + 0] << 0 |
			ext_csd[EXT_CSD_SEC_CNT + 1] << 8 |
			ext_csd[EXT_CSD_SEC_CNT + 2] << 16 |
			ext_csd[EXT_CSD_SEC_CNT + 3] << 24;

		/* Cards with density > 2GiB are sector addressed */
		if (card->ext_csd.sectors > (2u * 1024 * 1024 * 1024) / 512)
			mmc_card_set_blockaddr(card);
	}

#ifdef CONFIG_ARCH_GEN3
	card->ext_csd.boot_size_mult = ext_csd[EXT_CSD_BOOT_SIZE_MULT];
	card->ext_csd.boot_config = ext_csd[EXT_CSD_BOOT_CONFIG];

    //printk("[PowerOn] mmc_read_ext_csd:card->ext_csd.boot_size_mult = %d\n",card->ext_csd.boot_size_mult);
    //printk("[PowerOn] mmc_read_ext_csd:card->ext_csd.boot_config = %d\n",card->ext_csd.boot_config);

	if (ext_csd[EXT_CSD_REV] == 5) {
         //printk("[PowerOn] mmc_read_ext_csd:ext_csd[EXT_CSD_REV] == 5\n");
		for (i = 0; i < 4; i++) {
			card->ext_csd.gp_size[i] =
				(ext_csd[EXT_CSD_GP_SIZE_MULT + i * 3] + 
				ext_csd[EXT_CSD_GP_SIZE_MULT + i * 3 + 1] * 256 +
				ext_csd[EXT_CSD_GP_SIZE_MULT + i * 3 + 2] * 65536) *
				ext_csd[EXT_CSD_HC_WP_GRP_SIZE] * 
				ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE] * 1024 *
				(ext_csd[EXT_CSD_PARTITION_SUPPORT] & 0x1);
		}
	}
	else if (ext_csd[EXT_CSD_REV] == 4) {
        //printk("[PowerOn] mmc_read_ext_csd:ext_csd[EXT_CSD_REV] == 4\n");
		for (i = 0; i < 4; i++) {
			card->ext_csd.gp_size[i] =
				(ext_csd[EXT_CSD_GP_SIZE_MULT + i * 3] + 
				ext_csd[EXT_CSD_GP_SIZE_MULT + i * 3 + 1] * 8 +
				ext_csd[EXT_CSD_GP_SIZE_MULT + i * 3 + 2] * 64) *
				ext_csd[EXT_CSD_HC_WP_GRP_SIZE] * 
				ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE] * 1024 *
				(ext_csd[EXT_CSD_PARTITION_SUPPORT] & 0x1);
		}
	}
	else {
        //printk("[PowerOn] mmc_read_ext_csd:ext_csd[EXT_CSD_REV] != 4 and 5\n");
		for (i = 0; i < 4; i++) {
			card->ext_csd.gp_size[i] = 0;
		}
	}
#endif

    //printk("[PowerOn] mmc_read_ext_csd:switch (ext_csd[EXT_CSD_CARD_TYPE] & EXT_CSD_CARD_TYPE_MASK)\n");
	switch (ext_csd[EXT_CSD_CARD_TYPE] & EXT_CSD_CARD_TYPE_MASK) {
	case EXT_CSD_CARD_TYPE_DDR_52 | EXT_CSD_CARD_TYPE_52 |
	     EXT_CSD_CARD_TYPE_26:
		card->ext_csd.hs_max_dtr = 52000000;
		card->ext_csd.card_type = EXT_CSD_CARD_TYPE_DDR_52;
        //printk("[PowerOn] mmc_read_ext_csd:card->ext_csd.hs_max_dtr = %d\n",card->ext_csd.hs_max_dtr);
        //printk("[PowerOn] mmc_read_ext_csd:card->card->ext_csd.card_type = EXT_CSD_CARD_TYPE_DDR_52\n");
		break;
	case EXT_CSD_CARD_TYPE_DDR_1_2V | EXT_CSD_CARD_TYPE_52 |
	     EXT_CSD_CARD_TYPE_26:
		card->ext_csd.hs_max_dtr = 52000000;
		card->ext_csd.card_type = EXT_CSD_CARD_TYPE_DDR_1_2V;
        //printk("[PowerOn] mmc_read_ext_csd:card->ext_csd.hs_max_dtr = %d\n",card->ext_csd.hs_max_dtr);
        //printk("[PowerOn] mmc_read_ext_csd:card->card->ext_csd.card_type = EXT_CSD_CARD_TYPE_DDR_1_2V\n");
		break;
	case EXT_CSD_CARD_TYPE_DDR_1_8V | EXT_CSD_CARD_TYPE_52 |
	     EXT_CSD_CARD_TYPE_26:
		card->ext_csd.hs_max_dtr = 52000000;
		card->ext_csd.card_type = EXT_CSD_CARD_TYPE_DDR_1_8V;
        //printk("[PowerOn] mmc_read_ext_csd:card->ext_csd.hs_max_dtr = %d\n",card->ext_csd.hs_max_dtr);
        //printk("[PowerOn] mmc_read_ext_csd:card->card->ext_csd.card_type = EXT_CSD_CARD_TYPE_DDR_1_8V\n");
		break;
	case EXT_CSD_CARD_TYPE_52 | EXT_CSD_CARD_TYPE_26:
		card->ext_csd.hs_max_dtr = 52000000;
        //printk("[PowerOn] mmc_read_ext_csd:card->ext_csd.hs_max_dtr = %d\n",card->ext_csd.hs_max_dtr);
        //printk("[PowerOn] mmc_read_ext_csd:card->card->ext_csd.card_type = NULL\n");
		break;
	case EXT_CSD_CARD_TYPE_26:
		card->ext_csd.hs_max_dtr = 26000000;
        //printk("[PowerOn] mmc_read_ext_csd:card->ext_csd.hs_max_dtr = %d\n",card->ext_csd.hs_max_dtr);
        //printk("[PowerOn] mmc_read_ext_csd:card->card->ext_csd.card_type = NULL\n");
		break;
	default:
		/* MMC v4 spec says this cannot happen */
		printk(KERN_WARNING "%s: card is mmc v4 but doesn't "
			"support any high-speed modes.\n",
			mmc_hostname(card->host));
	}

	if (card->ext_csd.rev >= 3) {
        //printk("[PowerOn] mmc_read_ext_csd:card->ext_csd.rev >= 3\n");
		u8 sa_shift = ext_csd[EXT_CSD_S_A_TIMEOUT];

		/* Sleep / awake timeout in 100ns units */
		if (sa_shift > 0 && sa_shift <= 0x17)
			card->ext_csd.sa_timeout =
					1 << ext_csd[EXT_CSD_S_A_TIMEOUT];
		card->ext_csd.erase_group_def =
			ext_csd[EXT_CSD_ERASE_GROUP_DEF];
		card->ext_csd.hc_erase_timeout = 300 *
			ext_csd[EXT_CSD_ERASE_TIMEOUT_MULT];
		card->ext_csd.hc_erase_size =
			ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE] << 10;
	}

	if (card->ext_csd.rev >= 4) {
         //printk("[PowerOn] mmc_read_ext_csd:card->ext_csd.rev >= 4\n");
		/*
		 * Enhanced area feature support -- check whether the eMMC
		 * card has the Enhanced area enabled.  If so, export enhanced
		 * area offset and size to user by adding sysfs interface.
		 */
		if ((ext_csd[EXT_CSD_PARTITION_SUPPORT] & 0x2) &&
				(ext_csd[EXT_CSD_PARTITION_ATTRIBUTE] & 0x1)) {
			u8 hc_erase_grp_sz =
				ext_csd[EXT_CSD_HC_ERASE_GRP_SIZE];
			u8 hc_wp_grp_sz =
				ext_csd[EXT_CSD_HC_WP_GRP_SIZE];

			card->ext_csd.enhanced_area_en = 1;
			/*
			 * calculate the enhanced data area offset, in bytes
			 */
			card->ext_csd.enhanced_area_offset =
				(ext_csd[139] << 24) + (ext_csd[138] << 16) +
				(ext_csd[137] << 8) + ext_csd[136];
			if (mmc_card_blockaddr(card))
				card->ext_csd.enhanced_area_offset <<= 9;
			/*
			 * calculate the enhanced data area size, in kilobytes
			 */
			card->ext_csd.enhanced_area_size =
				(ext_csd[142] << 16) + (ext_csd[141] << 8) +
				ext_csd[140];
			card->ext_csd.enhanced_area_size *=
				(size_t)(hc_erase_grp_sz * hc_wp_grp_sz);
			card->ext_csd.enhanced_area_size <<= 9;
		} else {
			/*
			 * If the enhanced area is not enabled, disable these
			 * device attributes.
			 */
			card->ext_csd.enhanced_area_offset = -EINVAL;
			card->ext_csd.enhanced_area_size = -EINVAL;
		}
		card->ext_csd.sec_trim_mult =
			ext_csd[EXT_CSD_SEC_TRIM_MULT];
		card->ext_csd.sec_erase_mult =
			ext_csd[EXT_CSD_SEC_ERASE_MULT];
		card->ext_csd.sec_feature_support =
			ext_csd[EXT_CSD_SEC_FEATURE_SUPPORT];
		card->ext_csd.trim_timeout = 300 *
			ext_csd[EXT_CSD_TRIM_MULT];
	}

	if (ext_csd[EXT_CSD_ERASED_MEM_CONT])
		card->erased_byte = 0xFF;
	else
		card->erased_byte = 0x0;

out:
	kfree(ext_csd);

	return err;
}

MMC_DEV_ATTR(cid, "%08x%08x%08x%08x\n", card->raw_cid[0], card->raw_cid[1],
	card->raw_cid[2], card->raw_cid[3]);
MMC_DEV_ATTR(csd, "%08x%08x%08x%08x\n", card->raw_csd[0], card->raw_csd[1],
	card->raw_csd[2], card->raw_csd[3]);
MMC_DEV_ATTR(date, "%02d/%04d\n", card->cid.month, card->cid.year);
MMC_DEV_ATTR(erase_size, "%u\n", card->erase_size << 9);
MMC_DEV_ATTR(preferred_erase_size, "%u\n", card->pref_erase << 9);
MMC_DEV_ATTR(fwrev, "0x%x\n", card->cid.fwrev);
MMC_DEV_ATTR(hwrev, "0x%x\n", card->cid.hwrev);
MMC_DEV_ATTR(manfid, "0x%06x\n", card->cid.manfid);
MMC_DEV_ATTR(name, "%s\n", card->cid.prod_name);
MMC_DEV_ATTR(oemid, "0x%04x\n", card->cid.oemid);
MMC_DEV_ATTR(serial, "0x%08x\n", card->cid.serial);
MMC_DEV_ATTR(enhanced_area_offset, "%llu\n",
		card->ext_csd.enhanced_area_offset);
MMC_DEV_ATTR(enhanced_area_size, "%u\n", card->ext_csd.enhanced_area_size);

static struct attribute *mmc_std_attrs[] = {
	&dev_attr_cid.attr,
	&dev_attr_csd.attr,
	&dev_attr_date.attr,
	&dev_attr_erase_size.attr,
	&dev_attr_preferred_erase_size.attr,
	&dev_attr_fwrev.attr,
	&dev_attr_hwrev.attr,
	&dev_attr_manfid.attr,
	&dev_attr_name.attr,
	&dev_attr_oemid.attr,
	&dev_attr_serial.attr,
	&dev_attr_enhanced_area_offset.attr,
	&dev_attr_enhanced_area_size.attr,
	NULL,
};

static struct attribute_group mmc_std_attr_group = {
	.attrs = mmc_std_attrs,
};

static const struct attribute_group *mmc_attr_groups[] = {
	&mmc_std_attr_group,
	NULL,
};

static struct device_type mmc_type = {
	.groups = mmc_attr_groups,
};

/*
 * Handle the detection and initialisation of a card.
 *
 * In the case of a resume, "oldcard" will contain the card
 * we're trying to reinitialise.
 */
static int mmc_init_card(struct mmc_host *host, u32 ocr,
	struct mmc_card *oldcard)
{
	struct mmc_card *card;
	int err, ddr = 0;
	u32 cid[4];
	unsigned int max_dtr;
	u32 rocr;

	BUG_ON(!host);
	WARN_ON(!host->claimed);

	/*
	 * Since we're changing the OCR value, we seem to
	 * need to tell some cards to go back to the idle
	 * state.  We wait 1ms to give cards time to
	 * respond.
	 */

    //printk("[PowerOn] mmc_init_card: mmc_go_idle\n");
	mmc_go_idle(host);

	/* The extra bit indicates that we support high capacity */
    //printk("[PowerOn] mmc_init_card: mmc_send_op_cond\n");
	err = mmc_send_op_cond(host, ocr | (1 << 30), &rocr);
	if (err)
		goto err;

    //printk("[PowerOn] mmc_init_card: change ocr from 0x%0.8X to 0x%0.8X\n",ocr,rocr);
	/*
	 * For SPI, enable CRC as appropriate.
	 */
	if (mmc_host_is_spi(host)) {
		err = mmc_spi_set_crc(host, use_spi_crc);
		if (err)
			goto err;
	}

	/*
	 * Fetch CID from card.
	 */
    //printk("[PowerOn] mmc_init_card: Fetch CID from card.\n");
	if (mmc_host_is_spi(host))
		err = mmc_send_cid(host, cid);
	else
		err = mmc_all_send_cid(host, cid);
	if (err)
		goto err;

    //printk("[PowerOn] mmc_init_card: CID [%X],[%X],[%X],[%X].\n",cid[0],cid[1],cid[2],cid[3]);

	if (oldcard) {

        //printk("[PowerOn] mmc_init_card: oldcard = true.\n");
		if (memcmp(cid, oldcard->raw_cid, sizeof(cid)) != 0) {
			err = -ENOENT;
			goto err;
		}

		card = oldcard;
	} else {
		/*
		 * Allocate card structure.
		 */
        //printk("[PowerOn] mmc_init_card: mmc_alloc_card().\n");
		card = mmc_alloc_card(host, &mmc_type);
		if (IS_ERR(card)) {
			err = PTR_ERR(card);
            //printk("[PowerOn] mmc_init_card: mmc_alloc_card() ERROR %d.\n",err);
			goto err;
		}

		card->type = MMC_TYPE_MMC;
		card->rca = 1;
		memcpy(card->raw_cid, cid, sizeof(card->raw_cid));
	}

	/*
	 * For native busses:  set card RCA and quit open drain mode.
	 */
	if (!mmc_host_is_spi(host)) {
        //printk("[PowerOn] mmc_init_card: mmc_set_relative_addr().\n");
		err = mmc_set_relative_addr(card);
		if (err)
        {
            //printk("[PowerOn] mmc_init_card: mmc_set_relative_addr() return %d .\n", err);
			goto free_card;
        }

        //printk("[PowerOn] mmc_init_card: mmc_set_bus_mode().\n");
		mmc_set_bus_mode(host, MMC_BUSMODE_PUSHPULL);
	}

	if (!oldcard) {
		/*
		 * Fetch CSD from card.
		 */
        //printk("[PowerOn] mmc_init_card: Fetch CSD from card.\n");

        //printk("[PowerOn] mmc_init_card: mmc_send_csd.\n");
		err = mmc_send_csd(card, card->raw_csd);
		if (err)
        {
            //printk("[PowerOn] mmc_init_card: mmc_send_csd return %d.\n", err);
			goto free_card;
        }

        //printk("[PowerOn] mmc_init_card: mmc_decode_csd.\n");
		err = mmc_decode_csd(card);
		if (err)
        {
            //printk("[PowerOn] mmc_init_card: mmc_decode_csd return %d.\n", err);
			goto free_card;
        }


        //printk("[PowerOn] mmc_init_card: mmc_decode_cid.\n");
		err = mmc_decode_cid(card);
		if (err)
        {
            //printk("[PowerOn] mmc_init_card: mmc_decode_cid return %d.\n", err);
			goto free_card;
        }


	}

	/*
	 * Select card, as all following commands rely on that.
	 */
	if (!mmc_host_is_spi(host)) {
        //printk("[PowerOn] mmc_init_card: mmc_select_card.\n");
		err = mmc_select_card(card);
		if (err)
        {
            //printk("[PowerOn] mmc_init_card: mmc_select_card return %d.\n", err);
			goto free_card;
        }
	}

	if (!oldcard) {
		/*
		 * Fetch and process extended CSD.
		 */
        //printk("[PowerOn] mmc_init_card: Fetch and process extended CSD..\n");


        //printk("[PowerOn] mmc_init_card: mmc_read_ext_csd.\n");
		err = mmc_read_ext_csd(card);
		if (err)
        {
            //printk("[PowerOn] mmc_init_card: mmc_read_ext_csd return %d.\n", err);

#ifdef CONFIG_ARCH_GEN3 
            /* WA for eMMC errors (CRC erorr interrupt) */ 
            host->caps |= MMC_CAP_NEEDS_POLL;
#endif 

			goto free_card;
        }

		/* If doing byte addressing, check if required to do sector
		 * addressing.  Handle the case of <2GB cards needing sector
		 * addressing.  See section 8.1 JEDEC Standard JED84-A441;
		 * ocr register has bit 30 set for sector addressing.
		 */
		if (!(mmc_card_blockaddr(card)) && (rocr & (1<<30)))
        {
            //printk("[PowerOn] mmc_init_card: mmc_card_set_blockaddr.\n");
			mmc_card_set_blockaddr(card);
        }
        else
        {
             //printk("[PowerOn] mmc_init_card: SKIP mmc_card_set_blockaddr.\n");

        }

		/* Erase size depends on CSD and Extended CSD */
        //printk("[PowerOn] mmc_init_card: Erase size depends on CSD and Extended CSD.\n");

        //printk("[PowerOn]mmc_init_card:  mmc_set_erase_size.\n");
		mmc_set_erase_size(card);
	}

	/*
	 * If enhanced_area_en is TRUE, host needs to enable ERASE_GRP_DEF
	 * bit.  This bit will be lost every time after a reset or power off.
	 */
	if (card->ext_csd.enhanced_area_en) {

        //printk("[PowerOn]mmc_init_card:  mmc_switch(EXT_CSD_CMD_SET_NORMAL,EXT_CSD_ERASE_GROUP_DEF, 1).\n");
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
				EXT_CSD_ERASE_GROUP_DEF, 1);

		if (err && err != -EBADMSG)
        {
            //printk("[PowerOn] mmc_init_card: mmc_switch return %d (err != -EBADMSG).\n", err);
			goto free_card;
        }

		if (err) {
            //printk("[PowerOn] mmc_init_card: mmc_switch return %d .\n", err);
			err = 0;
			/*
			 * Just disable enhanced area off & sz
			 * will try to enable ERASE_GROUP_DEF
			 * during next time reinit
			 */
			card->ext_csd.enhanced_area_offset = -EINVAL;
			card->ext_csd.enhanced_area_size = -EINVAL;
		} else {
			card->ext_csd.erase_group_def = 1;
			/*
			 * enable ERASE_GRP_DEF successfully.
			 * This will affect the erase size, so
			 * here need to reset erase size
			 */
            //printk("[PowerOn]mmc_init_card:  mmc_set_erase_size.\n");
			mmc_set_erase_size(card);
		}
	}
    else
    {
        //printk("[PowerOn]mmc_init_card:  SKIP mmc_switch(EXT_CSD_CMD_SET_NORMAL,EXT_CSD_ERASE_GROUP_DEF, 1).\n");
    }

	/*
	 * Activate high speed (if supported)
	 */
	if ((card->ext_csd.hs_max_dtr != 0) &&
		(host->caps & MMC_CAP_MMC_HIGHSPEED)) {
        //printk("[PowerOn]mmc_init_card:  mmc_switch(EXT_CSD_CMD_SET_NORMAL,EXT_CSD_HS_TIMING, 1).\n");
		err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
			EXT_CSD_HS_TIMING, 1);
		if (err && err != -EBADMSG)
        {
            //printk("[PowerOn] mmc_init_card: mmc_switch return %d (err != -EBADMSG).\n", err);
			goto free_card;
        }

		if (err) {
			printk(KERN_WARNING "%s: switch to highspeed failed\n",
			       mmc_hostname(card->host));
			err = 0;
		} else {
            //printk("[PowerOn] mmc_init_card: mmc_card_set_highspeed.\n");
			mmc_card_set_highspeed(card);
            //printk("[PowerOn] mmc_init_card: mmc_set_timing.\n");
			mmc_set_timing(card->host, MMC_TIMING_MMC_HS);
		}
	}
    else
    {
        //printk("[PowerOn]mmc_init_card:  SKIP mmc_switch(EXT_CSD_CMD_SET_NORMAL,EXT_CSD_HS_TIMING, 1).\n");
    }


	/*
	 * Compute bus speed.
	 */
	max_dtr = (unsigned int)-1;

	if (mmc_card_highspeed(card)) {
        //printk("[PowerOn] mmc_init_card: Compute bus speed - mmc_card_highspeed().\n");
		if (max_dtr > card->ext_csd.hs_max_dtr)
        {
			max_dtr = card->ext_csd.hs_max_dtr;
            //printk("[PowerOn] mmc_init_card: Compute bus speed - max_dtr = card->ext_csd.hs_max_dtr  (%d).\n",max_dtr);
        }
	} else if (max_dtr > card->csd.max_dtr) {
        //printk("[PowerOn] mmc_init_card: Compute bus speed - max_dtr = card->csd.max_dtr. (%d)\n",max_dtr);
		max_dtr = card->csd.max_dtr;
	}
    
    //printk("[PowerOn] mmc_init_card: mmc_set_clock (max_dtr = %d).\n",max_dtr);
	mmc_set_clock(host, max_dtr);

	/*
	 * Indicate DDR mode (if supported).
	 */
	if (mmc_card_highspeed(card)) {
		if ((card->ext_csd.card_type & EXT_CSD_CARD_TYPE_DDR_1_8V)
			&& (host->caps & (MMC_CAP_1_8V_DDR)))
        {
            //printk("[PowerOn] mmc_init_card:  ddr = MMC_1_8V_DDR_MODE.\n");
		    ddr = MMC_1_8V_DDR_MODE;
        }
		else if ((card->ext_csd.card_type & EXT_CSD_CARD_TYPE_DDR_1_2V)
			&& (host->caps & (MMC_CAP_1_2V_DDR)))
        {
            //printk("[PowerOn] mmc_init_card:  ddr = MMC_1_2V_DDR_MODE.\n");
		    ddr = MMC_1_2V_DDR_MODE;
        }
	}

	/*
	 * Activate wide bus and DDR (if supported).
	 */
    //printk("[PowerOn] mmc_init_card:  Activate wide bus and DDR (if supported).\n");
	if ((card->csd.mmca_vsn >= CSD_SPEC_VER_4) &&
	    (host->caps & (MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA))) {

        //printk("[PowerOn] mmc_init_card:  (card->csd.mmca_vsn >= CSD_SPEC_VER_4) && (host->caps & (MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA)).\n");

		static unsigned ext_csd_bits[][2] = {
			{ EXT_CSD_BUS_WIDTH_8, EXT_CSD_DDR_BUS_WIDTH_8 },
			{ EXT_CSD_BUS_WIDTH_4, EXT_CSD_DDR_BUS_WIDTH_4 },
			{ EXT_CSD_BUS_WIDTH_1, EXT_CSD_BUS_WIDTH_1 },
		};
		static unsigned bus_widths[] = {
			MMC_BUS_WIDTH_8,
			MMC_BUS_WIDTH_4,
			MMC_BUS_WIDTH_1
		};
		unsigned idx, bus_width = 0;

		if (host->caps & MMC_CAP_8_BIT_DATA)
        {
            //printk("[PowerOn] mmc_init_card:  (host->caps & MMC_CAP_8_BIT_DATA) -> idx=0.\n");
			idx = 0;
        }
		else
        {
            //printk("[PowerOn] mmc_init_card:  NOT (host->caps & MMC_CAP_8_BIT_DATA) -> idx=1.\n");
			idx = 1;
        }
		for (; idx < ARRAY_SIZE(bus_widths); idx++) {
            //printk("[PowerOn] mmc_init_card:  idx=%d.\n",idx);
			bus_width = bus_widths[idx];
			if (bus_width == MMC_BUS_WIDTH_1)
            {
                //printk("[PowerOn] mmc_init_card:  ddr = 0; 'no DDR for 1-bit width'.\n");
				ddr = 0; /* no DDR for 1-bit width */
            }
            //printk("[PowerOn] mmc_init_card:  mmc_switch(EXT_CSD_CMD_SET_NORMAL,EXT_CSD_BUS_WIDTH,ext_csd_bits[%d][0]).\n",idx);
			err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
					 EXT_CSD_BUS_WIDTH,
					 ext_csd_bits[idx][0]);
			if (!err) {
                //printk("[PowerOn] mmc_init_card: mmcmmc_set_bus_width_ddr(bus_width=%d, MMC_SDR_MODE) .\n",bus_width);
				mmc_set_bus_width_ddr(card->host,
						      bus_width, MMC_SDR_MODE);
				/*
				 * If controller can't handle bus width test,
				 * use the highest bus width to maintain
				 * compatibility with previous MMC behavior.
				 */
				if (!(host->caps & MMC_CAP_BUS_WIDTH_TEST))
					break;

                //printk("[PowerOn] mmc_init_card: mmc_bus_test() .\n");
				err = mmc_bus_test(card, bus_width);
				if (!err)
                {
                    //printk("[PowerOn] mmc_init_card: mmc_bus_test return %d - BREAK .\n", err);
					break;
                }
                else
                {
                    //printk("[PowerOn] mmc_init_card: mmc_bus_test return %d .\n", err);
                }
			}
            else
            {
                //printk("[PowerOn] mmc_init_card: mmc_switch return %d .\n", err);
            }
		}

		if (!err && ddr) {
            //printk("[PowerOn] mmc_init_card: err=%d && ddr=%d.\n", err,ddr);

            //printk("[PowerOn] mmc_init_card:  mmc_switch(EXT_CSD_CMD_SET_NORMAL,EXT_CSD_BUS_WIDTH,ext_csd_bits[%d][1]).\n",idx);
			err = mmc_switch(card, EXT_CSD_CMD_SET_NORMAL,
					EXT_CSD_BUS_WIDTH,
					ext_csd_bits[idx][1]);
		}
		if (err) {
			printk(KERN_WARNING "%s: switch to bus width %d ddr %d "
				"failed\n", mmc_hostname(card->host),
				1 << bus_width, ddr);
			goto free_card;
		} else if (ddr) {
            //printk("[PowerOn] mmc_init_card: mmc_card_set_ddr_mode()\n");
			mmc_card_set_ddr_mode(card);
            //printk("[PowerOn] mmc_init_card: mmc_set_bus_width_ddr(bus_width=%d, ddr=%d)\n",bus_width, ddr);
			mmc_set_bus_width_ddr(card->host, bus_width, ddr);
		}
	}

	if (!oldcard)
    {
        //printk("[PowerOn] mmc_init_card: host->card = card\n");
		host->card = card;
    }

    //printk("[PowerOn] mmc_init_card: return 0\n"); 
	return 0;

free_card:
    //printk("[PowerOn] mmc_init_card: free_card:\n");
	if (!oldcard)
    {
        //printk("[PowerOn] mmc_init_card: mmc_remove_card\n");
		mmc_remove_card(card);
    }
err:

    //printk("[PowerOn] mmc_init_card: err: return %d\n",err);
	return err;
}

/*
 * Host is being removed. Free up the current card.
 */
static void mmc_remove(struct mmc_host *host)
{
	BUG_ON(!host);
	BUG_ON(!host->card);

	mmc_remove_card(host->card);
	host->card = NULL;
}

/*
 * Card detection callback from host.
 */
static void mmc_detect(struct mmc_host *host)
{
	int err;

	BUG_ON(!host);
	BUG_ON(!host->card);

	mmc_claim_host(host);

	/*
	 * Just check if our card has been removed.
	 */
	err = mmc_send_status(host->card, NULL);

	mmc_release_host(host);

	if (err) {
		mmc_remove(host);

		mmc_claim_host(host);
		mmc_detach_bus(host);
		mmc_release_host(host);
	}
}

/*
 * Suspend callback from host.
 */
static int mmc_suspend(struct mmc_host *host)
{
	BUG_ON(!host);
	BUG_ON(!host->card);

	mmc_claim_host(host);
	if (!mmc_host_is_spi(host))
		mmc_deselect_cards(host);
	host->card->state &= ~MMC_STATE_HIGHSPEED;
	mmc_release_host(host);

	return 0;
}

/*
 * Resume callback from host.
 *
 * This function tries to determine if the same card is still present
 * and, if so, restore all state to it.
 */
static int mmc_resume(struct mmc_host *host)
{
	int err;

	BUG_ON(!host);
	BUG_ON(!host->card);

	mmc_claim_host(host);
	err = mmc_init_card(host, host->ocr, host->card);
	mmc_release_host(host);

	return err;
}

static int mmc_power_restore(struct mmc_host *host)
{
	int ret;

	host->card->state &= ~MMC_STATE_HIGHSPEED;
	mmc_claim_host(host);
	ret = mmc_init_card(host, host->ocr, host->card);
	mmc_release_host(host);

	return ret;
}

static int mmc_sleep(struct mmc_host *host)
{
	struct mmc_card *card = host->card;
	int err = -ENOSYS;

	if (card && card->ext_csd.rev >= 3) {
		err = mmc_card_sleepawake(host, 1);
		if (err < 0)
			pr_debug("%s: Error %d while putting card into sleep",
				 mmc_hostname(host), err);
	}

	return err;
}

static int mmc_awake(struct mmc_host *host)
{
	struct mmc_card *card = host->card;
	int err = -ENOSYS;

	if (card && card->ext_csd.rev >= 3) {
		err = mmc_card_sleepawake(host, 0);
		if (err < 0)
			pr_debug("%s: Error %d while awaking sleeping card",
				 mmc_hostname(host), err);
	}

	return err;
}

static const struct mmc_bus_ops mmc_ops = {
	.awake = mmc_awake,
	.sleep = mmc_sleep,
	.remove = mmc_remove,
	.detect = mmc_detect,
	.suspend = NULL,
	.resume = NULL,
	.power_restore = mmc_power_restore,
};

static const struct mmc_bus_ops mmc_ops_unsafe = {
	.awake = mmc_awake,
	.sleep = mmc_sleep,
	.remove = mmc_remove,
	.detect = mmc_detect,
	.suspend = mmc_suspend,
	.resume = mmc_resume,
	.power_restore = mmc_power_restore,
};

static void mmc_attach_bus_ops(struct mmc_host *host)
{
	const struct mmc_bus_ops *bus_ops;

	if (!mmc_card_is_removable(host))
		bus_ops = &mmc_ops_unsafe;
	else
		bus_ops = &mmc_ops;
	mmc_attach_bus(host, bus_ops);
}

/*
 * Starting point for MMC card init.
 */
int mmc_attach_mmc(struct mmc_host *host)
{
	int err;
	u32 ocr;

	BUG_ON(!host);
	WARN_ON(!host->claimed);

    //printk("[PowerOn] mmc_attach_mmc: mmc_send_op_cond()\n");
	err = mmc_send_op_cond(host, 0, &ocr);
	if (err)
    {
        //printk("[PowerOn] mmc_attach_mmc: ERROR - mmc_send_op_cond return %d, return %d \n",err,err);
		return err;
    }

	mmc_attach_bus_ops(host);
	if (host->ocr_avail_mmc)
		host->ocr_avail = host->ocr_avail_mmc;

	/*
	 * We need to get OCR a different way for SPI.
	 */
	if (mmc_host_is_spi(host)) {
		err = mmc_spi_read_ocr(host, 1, &ocr);
		if (err)
        {
            //printk("[PowerOn] mmc_attach_mmc: ERROR - mmc_spi_read_ocr return %d, goto err\n",err);
			goto err;
        }
	}

	/*
	 * Sanity check the voltages that the card claims to
	 * support.
	 */
	if (ocr & 0x7F) {
		printk(KERN_WARNING "%s: card claims to support voltages "
		       "below the defined range. These will be ignored.\n",
		       mmc_hostname(host));
		ocr &= ~0x7F;
	}

	host->ocr = mmc_select_voltage(host, ocr);

	/*
	 * Can we support the voltage of the card?
	 */
	if (!host->ocr) {
        //printk("[PowerOn] mmc_attach_mmc: ERROR - mmc_select_voltage return null, goto err\n");
		err = -EINVAL;
		goto err;
	}

	/*
	 * Detect and init the card.
	 */
    //printk("[PowerOn] mmc_attach_mmc:  calling mmc_init_card\n");
	err = mmc_init_card(host, host->ocr, NULL);
	if (err)
    {
        //printk("[PowerOn] mmc_attach_mmc: ERROR - mmc_init_card return %d, goto err\n",err);
		goto err;
    }

    //printk("[PowerOn] mmc_attach_mmc: mmc_release_host()\n");
	mmc_release_host(host);
    //printk("[PowerOn] mmc_add_card\n");
	err = mmc_add_card(host->card);
    //printk("[PowerOn] mmc_attach_mmc: mmc_claim_host()\n");
	mmc_claim_host(host);
	if (err)
    {
        //printk("[PowerOn] mmc_attach_mmc: ERROR - mmc_add_card retutn %d, goto remove_card\n",err);
		goto remove_card;
    }

    //printk("[PowerOn] mmc_attach_mmc: return 0\n");
	return 0;

remove_card:
    //printk("[PowerOn] mmc_attach_mmc: remove_card\n");
	mmc_release_host(host);
	mmc_remove_card(host->card);
	mmc_claim_host(host);
	host->card = NULL;
err:
    //printk("[PowerOn] mmc_attach_mmc: err\n");
	mmc_detach_bus(host);

	printk(KERN_ERR "%s: error %d whilst initialising MMC card\n",
		mmc_hostname(host), err);


	return err;
}
