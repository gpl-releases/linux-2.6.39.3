
/*
 *  kernel/arm_atom_mbx.h
 *
 *  GPL LICENSE SUMMARY
 *
 *  Copyright(c) 2012 Intel Corporation. All rights reserved.
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
#ifndef _CPU_MBX_H_
#define _CPU_MBX_H_

#define MBX_DRIVER_DEV_NAME "/dev/arm_atom_mbx"
#define DEVICE_NAME "arm_atom_mbx"
#define DIGITS 80

/* RPC-IF structure */
struct npcpu_rpc_info
{
        unsigned int npcpu_ipv4_addr;
        unsigned int appcpu_ipv4_addr;
        unsigned int netmask;
        unsigned int vlan_id;
}__attribute__((packed));

struct arm11_atom_mbx_user
{
	unsigned short  eventId;
    unsigned short  isParamRequired;
	struct npcpu_rpc_info parameter;
	unsigned int resv[2];		/* Reserved */
}__attribute__((packed));


enum arm11_mbx_event_id
{
    ARM11_EVENT_GPIO_INIT_EXIT     = 0x0001,
    ARM11_EVENT_SPI_INIT_EXIT      = 0x0002,
    ARM11_EVENT_EMMC_INIT_EXIT     = 0x0004, 
    NPCPU_EVENT_RPC_IF_OBTAIN_ADDR = 0x0008, 
    ARM11_EVENT_EMMC_ADVANCE_INIT_EXIT = 0x0010 

};

enum atom_mbx_event_id
{
    ATOM_EVENT_RSVD   = 0x0001,
    ATOM_EVENT_SPI_ADVANCE_EXIT    = 0x0002,
    ATOM_EVENT_EMMC_ADVANCE_EXIT   = 0x0004, 
};

#define MBX_MODULE_ID 1
#define	MBX_SEND_EVENT_CMD           _IOW(MBX_MODULE_ID, 1, struct arm11_atom_mbx_user)
#define	MBX_GET_EVENT_CMD            _IOR(MBX_MODULE_ID, 2, struct arm11_atom_mbx_user)
#define	MBX_SEND_ACK_CMD             _IOW(MBX_MODULE_ID, 3, struct arm11_atom_mbx_user)
#define	MBX_RECEIVE_ACK_CMD          _IOR(MBX_MODULE_ID, 4, struct arm11_atom_mbx_user)
#define	MBX_REBOOT                   _IO (MBX_MODULE_ID, 5)

#ifdef __KERNEL__

long arm_atom_mbx_receive_event_notification(unsigned short eventId, unsigned int *param);
long arm_atom_mbx_receive_specific_ack(unsigned short eventId);
long arm_atom_mbx_send_ack(unsigned short eventID);
long arm_atom_mbx_send_notification(unsigned short eventID, unsigned int *paramPtr);


#endif
#endif	/* _CPU_MBX_H_ */
