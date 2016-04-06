/*
 *
 * ticfg.h
 * Description:
 *
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
#ifndef _INCLUDE_TICFG_H_
#define _INCLUDE_TICFG_H_

#include <linux/ioctl.h>

#define TICFG_MTD_OFFSET     (0)

#if defined CONFIG_MIPS_TNETV1050SDB
#define TICFG_MTD_SIZE       (128*1024)
#else
#define TICFG_MTD_SIZE       (64*1024)
#endif

#define ENV_VAR_MTD          (3)
#define ENV_VAR_MTD_OFFSET   (0)
#define ENV_VAR_MTD_SIZE     (FLASH_ENV_ENTRY_SIZE * MAX_ENV_ENTRY)

#define CFGMAN_MTD           (3)
#define CFGMAN_MTD_OFFSET    (ENV_VAR_MTD_SIZE)
#define CFGMAN_MTD_SIZE      (TICFG_MTD_SIZE - ENV_VAR_MTD_SIZE)

#define TICFG_IOCTL_MAGIC	0xde

//#define TICFG_IOCTL_ERASE    0
#define TICFG_IOCTL_ERASE		_IO(TICFG_IOCTL_MAGIC, 0)
#define TICFG_IOCTL_FLAG_READ_PRIMARY	_IOWR(TICFG_IOCTL_MAGIC, 1, unsigned int)
#define TICFG_IOCTL_FLAG_SET_PRIMARY	_IOWR(TICFG_IOCTL_MAGIC, 2, unsigned int)
#define TICFG_IOCTL_FLAG_READ_SECONDARY	_IOWR(TICFG_IOCTL_MAGIC, 3, unsigned int)
#define TICFG_IOCTL_FLAG_SET_SECONDARY	_IOWR(TICFG_IOCTL_MAGIC, 4, unsigned int)

#define MAX_ENV_DATA_LEN     (FLASH_ENV_ENTRY_SIZE)

extern int ticfg_erase_cfgman(struct mtd_info *mtd);
extern int ticfg_erase_env_vars(struct mtd_info *mtd);

extern void ticfg_lock_region(void);
extern void ticfg_unlock_region(void);

#endif
