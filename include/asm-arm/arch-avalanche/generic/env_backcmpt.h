/*
 *
 * 
 * env_backcmpt.h
 * Description:
 *   This file supports the backward compatibility with Adam2 environment
 *   variables support. This is for OS co-working on boards that run Adam2 as
 *   bootloader.
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

#if 0
#include <sbl/env.h>
#include <sbl/stddef.h>
#endif

/* 
 * This array provides the mapping of the new env variables with the Adam2
 * counter parts.
 */
typedef struct ADAM2ENVDESC {
	char        *new_nm;
	char        *nm;
} ADAM2ENVDESC;
ADAM2ENVDESC env_adam2_alias[] = {
	{"CPUFREQ",      "cpufrequency",        },
	{"MEMSZ",        "memsize",             },
	{"FLASHSZ",      "flashsize",           },
	{"MODETTY0",     "modetty0",            },
	{"MODETTY1",     "modetty1",            },
	{"PROMPT",       "prompt",              },
	{"BOOTCFG",      "bootcfg",              },
	{"HWA_0",        "maca",                },
	{"HWA_1",        "macb",                },
	{"HWA_RNDIS",    "usb_board_mac",       },
	{"HWA_3",        "macc",                },
	{"IPA",          "my_ipaddress",        },
	{"IPA_SVR",      "remote_ipaddress",    },
	{"IPA_GATEWAY",  "ipa_gateway",         },
	{"SUBNET_MASK",  "subnet_mask",         },
	{"BLINE_MAC0",   "bootline1",           },
	{"BLINE_MAC1",   "bootline2",           },
	{"BLINE_RNDIS",  "rndisbootline",       },
	{"BLINE_ATM",    "atmbootline",         },
	{"USB_PID",      "usb_prod_id",         },
	{"USB_VID",      "usb_vend_id",         },
	{"USB_EPPOLLI",  "usb_ep_poll",         },
    {"USB_SERIAL",   "usb_serial",          },
   	{"HWA_HRNDIS",   "usb_rndis_mac",      }
};
