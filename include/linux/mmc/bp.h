/*
 *  GPL LICENSE SUMMARY
 *
 *  Copyright(c) 2011 Intel Corporation. All rights reserved.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of version 2 of the GNU General Public License as
 *  published by the Free Software Foundation.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *  The full GNU General Public License is included in this distribution
 *  in the file called LICENSE.GPL.
 *
 *  Contact Information:
 *    Intel Corporation
 *    2200 Mission College Blvd.
 *    Santa Clara, CA  97052
 *
 */



#ifndef MMC_BP_H
#define MMC_BP_H


#define BP_DIR_READ	0x0	/* Read */
#define BP_DIR_WRITE	0x1	/* Write */

#define MMC_BLK_IOCTL_BP_GETINFO	0x8820	/* Get boot partition info */
#define MMC_BLK_IOCTL_BP_RDWR		0x8821	/* Read/write boot partition */
#define MMC_BLK_IOCTL_GP_GETINFO	0x8822	/* Get general purpose partition info */
#define MMC_BLK_IOCTL_GP_RDWR		0x8823	/* Read/write general purpose partition */
#define MMC_BLK_IOCTL_ARB_CMD		0x8830	/* Arbitrary command */
#define MMC_BLK_IOCTL_CARD_INFO 	0x8831	/* Get card info, such as RCA */


#define MMC_BP_UNIT_SIZE		(128 * 1024) /*Boot partition is an integer multiple of 128 kB in size */
#define MMC_SECTOR_SIZE			512	/*A sector size is 512Byte*/
#define MAX_NUM_OF_SECTORS_TRANSFERD	128	/*Only 128 sectors can be transfered at one time*/
#define MAX_NUM_OF_BOOT_PARTITIONS	1	/*Only 2 boot partitions are supported, partition 0 & partition 1*/

#define MMC_BOOT_EN_USER	0
#define MMC_BOOT_EN_BP0		1	
#define MMC_BOOT_EN_BP1		2	
#define MMC_BOOT_EN_NONE	3	
#define MMC_BOOT_EN_RESV	4	

struct mmc_bp_info {
	unsigned long  sectors;
	unsigned long  booten;
};

struct mmc_gp_info {
	unsigned long  sectors[4];
};


struct mmc_bp_rw {
	unsigned char  which;
	unsigned char  dir;
	void           *buf;
	unsigned long  st_sector;
	unsigned long  nr_sectors; /* max 128 */
};

struct mmc_card_info {
	unsigned int  rca;
};

struct mmc_arb_cmd {
	unsigned int  opcode;    /* Command index */
	unsigned int  arg;       /* Command argument */
	unsigned int  cmdflags;  /* Command flags */
	void          *resp;     /* Command response, this buffer should be four 32 bits, i.e. 16 bytes */
	void          *databuf;  /* Data buffer for read/write */
	unsigned int  datalen;   /* Data length in byte, should be multiples of 512, max is 64K */ 
	unsigned int  datadir;   /* Data transfer direction, 0 is read, >0 is write */
	unsigned int  dataflags; /* Data transfer flags */
	unsigned int  dataready; /* Whether check for card program finish and ready for data, 0 is not check, >0 is check */
	unsigned int  stop;      /* Whether should issue stop command, 0 is not issue, >0 is issue */
};

#endif
