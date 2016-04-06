/*
 * ti_hil.h - Header file for HIL
 *
 * Description:
 *  This file contains structures and definitions that are used by the
 *  HIL Framework.
 *
 * Copyright (C) <2008>, Texas Instruments, Incorporated
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
 */

#ifndef __TI_HIL_H__
#define __TI_HIL_H__
#if defined(__KERNEL__)
#ifdef CONFIG_TI_PACKET_PROCESSOR_STATS
#include <linux/skbuff.h>
#endif /* CONFIG_TI_PACKET_PROCESSOR_STATS */
#endif /* KERNEL*/
/**************************************************************************
 ****************************** Module Identifers *************************
 **************************************************************************/

/* Module Identifers: These describe the module which generated the event. */
#define TI_DEVICE   1
#define TI_INET     2
#define TI_PP       3

/**************************************************************************
 **************************** PP Event Identifiers ************************
 **************************************************************************/

/* List of event identifiers which have been added and are called from various
 * parts of the networking subsystem to generate events which should be addressed
 * by the HIL profile. 
 * To be as intutitive as possible for the HIL profile authors the following naming
 * convention has been deployed 
 *  TI_<NET-SUBSYSTEM>_<EVENTNAME>
 * For example: If the bridge generates an event when a FDB is created 
 * the name is TI_BRIDGE_FDBCREATE */

/**************************************************************************
 **************************** PP Bridge Event Identifiers *****************
 **************************************************************************/

/* Base for all bridge PP Event identifiers. */
#define TI_BRIDGE_EVENT              0x0

/* Event generated when an interface is removed from the bridge. */
#define TI_BRIDGE_PORT_DELETE        (TI_BRIDGE_EVENT + 0x1)

/* Event generated when an interface moves into the forwarding state. 
 * Note: The event needs to be handled only if the system support STP. */
#define TI_BRIDGE_PORT_FORWARD       (TI_BRIDGE_EVENT + 0x2)

/* Event generated when an interface is moved into the blocked state. 
 * Note: The event needs to be handled only if the system support STP. */
#define TI_BRIDGE_PORT_DISABLED      (TI_BRIDGE_EVENT + 0x3)

/* Event generated when a Bridge FDB entry is created */
#define TI_BRIDGE_FDB_CREATED        (TI_BRIDGE_EVENT + 0x4)

/* Event generated when a Bridge FDB entry is deleted */
#define TI_BRIDGE_FDB_DELETED        (TI_BRIDGE_EVENT + 0x5)

/* Event generated when a packet is flooded onto all interfaces */
#define TI_BRIDGE_PACKET_FLOODED     (TI_BRIDGE_EVENT + 0x6)

/**************************************************************************
 **************************** PP Routing Event Identifiers *****************
 **************************************************************************/

/* Base for all routing PP Event identifiers. */
#define TI_ROUTE_EVENT              0x10

/* Event generated when a route is added. */
#define TI_ROUTE_ADDED              (TI_ROUTE_EVENT + 0x1)

/* Event generated when a route is deleted. */
#define TI_ROUTE_DELETED            (TI_ROUTE_EVENT + 0x2)

/**************************************************************************
 **************************** PP VLAN Event Identifiers *****************
 **************************************************************************/

/* Base for all VLAN PP Event identifiers. */
#define TI_VLAN_EVENT               0x20

/* Event generated when a VLAN device is created. */
#define TI_VLAN_DEV_CREATED         (TI_VLAN_EVENT + 0x1)

/* Event generated when a VLAN device is deleted. */
#define TI_VLAN_DEV_DELETED         (TI_VLAN_EVENT + 0x2)


/**************************************************************************
 ********************* PP Connection Tracking Event Identifiers ***********
 **************************************************************************/

/* Base for all connection tracking PP Event identifiers. */
#define TI_CT_EVENT                  0x30

/* Event generated when a connection tracking entry has been created. */
#define TI_CT_ENTRY_CREATED          (TI_CT_EVENT + 0x1)

/* Event generated when a connection tracking entry has timed-out */
#define TI_CT_DEATH_BY_TIMEOUT       (TI_CT_EVENT + 0x2)

/* Event generated when a Netfilter table is being updated. */
#define TI_CT_NETFILTER_TABLE_UPDATE (TI_CT_EVENT + 0x3)

/* Event generated once Netfilter decides to drop the packet */
#define TI_CT_NETFILTER_DISCARD_PKT  (TI_CT_EVENT + 0x4)

/**************************************************************************
 ********************* PP Multicast Routing Event Identifiers *************
 **************************************************************************/

/* Base for all Multicast Routing PP Event identifiers. */
#define TI_MFC_EVENT                  0x40

/* Event generated when a multicast routing entry is created. */
#define TI_MFC_ENTRY_CREATED          (TI_MFC_EVENT + 0x1)

/* Event generated when a multicast routing entry is deleted. */
#define TI_MFC_ENTRY_DELETED          (TI_MFC_EVENT + 0x2)

#define TI_MC_SESSION_DELETED         (TI_MFC_EVENT + 0x3)

/**************************************************************************
 **************************** PP PPP Event Identifiers *****************
 **************************************************************************/

/* Base for all PPP PP Event identifiers. */
#define TI_PPP_EVENT               0x50

/* Event generated when a PPP interface is created. */
#define TI_PPP_INTERFACE_CREATED    (TI_PPP_EVENT + 0x1)

/* Event generated when a PPP interface is deleted. */
#define TI_PPP_INTERFACE_SHUTDOWN   (TI_PPP_EVENT + 0x2)


/**************************************************************************
 ************************* PP DOCSIS Event Identifiers ********************
 **************************************************************************/

/* Base for all DOCSIS PP Event identifiers. */
#define TI_DOCSIS_EVENT         0x60

/* Event generated when DOCSIS Bridge decides to drop(filter) the packet */
#define TI_DOCSIS_FLTR_DISCARD_PKT          (TI_DOCSIS_EVENT + 0x1)

/* Event generated when a new filter rule is added */
#define TI_DOCSIS_FLTR_ADD                  (TI_DOCSIS_EVENT + 0x2)

/* Event generated when a filter rule is deleted */
#define TI_DOCSIS_FLTR_DEL                  (TI_DOCSIS_EVENT + 0x3)

/* Event generated when a filter rule is being changed */
#define TI_DOCSIS_FLTR_CHG                  (TI_DOCSIS_EVENT + 0x4)

/* Event generated when a new classifier rule is added */
#define TI_DOCSIS_CLASSIFY_ADD              (TI_DOCSIS_EVENT + 0x5)

/* Event generated when a classifier rule is deleted */
#define TI_DOCSIS_CLASSIFY_DEL              (TI_DOCSIS_EVENT + 0x6)

/* Event generated when a classifier rule is being changed */
#define TI_DOCSIS_CLASSIFY_CHG              (TI_DOCSIS_EVENT + 0x7)

/* Event generated when DSID encoding is being changed, added or deleted */
#define TI_DOCSIS_DSID_CHG                  (TI_DOCSIS_EVENT + 0x8)

/* Event generated when MCAST filter is deleted */
#define TI_DOCSIS_MCAST_DEL                 (TI_DOCSIS_EVENT + 0x9)

/* Event generated when need to delete sessions from PP */
#define TI_DOCSIS_SESSIONS_DEL              (TI_DOCSIS_EVENT + 0xA)


/**************************************************************************
 ************************* PP General Event Identifiers *******************
 **************************************************************************/

/* Base for all general PP Event identifiers. */
#define TI_PP_GENERAL_EVENT         0x70
/* Event generated when create vpid */
#define TI_PP_ADD_VPID          (TI_PP_GENERAL_EVENT + 0x1)

/* Event generated when remove vpid */
#define TI_PP_REMOVE_VPID       (TI_PP_GENERAL_EVENT + 0x2)

/* DOCSIS Packet processor start/delete session notification defines */
#define TI_DOCSIS_PP_SESSION_TYPE_FORWARDING                    0x1
#define TI_DOCSIS_PP_SESSION_TYPE_DISCARDING                    0x2

typedef enum
{
    TUNNEL_TYPE_L2TPv3 = 0,
    TUNNEL_TYPE_GRE_MPLS = 1
} TUNNEL_TYPE_E;

/**************************************************************************
 **************************** HIL Profile Structures **********************
 **************************************************************************/

/* Prototype Declaration for the profile handler. */
typedef int (*TI_PROFILE_HANDLER)(unsigned int module_id, unsigned long event_id, void* ptr);
typedef int (*TI_PROFILE_INIT)(void);
typedef int (*TI_PROFILE_DEINIT)(void);

/**************************************************************************
 * STRUCTURE NAME : TI_HIL_PROFILE
 **************************************************************************
 * DESCRIPTION   :
 *  The structure describes the HIL Profile.
 **************************************************************************/
typedef struct TI_HIL_PROFILE
{
    char*               name;
    TI_PROFILE_INIT     profile_init;
    TI_PROFILE_HANDLER  profile_handler;
    TI_PROFILE_DEINIT   profile_deinit;
}TI_HIL_PROFILE;

/**************************************************************************
 ********************* PP Multicast Routing Event param *************
 **************************************************************************/
struct pp_mr_param {
	struct vif_device *vif_table;
    struct mfc_cache *cache;
};

/**************************************************************************
 **************************** HIL Core Exported API ***********************
 **************************************************************************/
#if defined(__KERNEL__)
/* Initialization and Cleanup API */
extern int ti_hil_initialize (void);
extern int ti_hil_deinitialize (void);

/* Profile Management API */
extern int ti_hil_register_profile (TI_HIL_PROFILE* ptr_profile);
extern int ti_hil_unregister_profile (void);

/* This is the API which is used for the event generation to the PP chain.
 * This should only be used in the networking subsystem to generate events 
 * which do not already exist and which are needed for the packet processor. 
 * This is needed only if the packet processor has been built into the system. */
#ifdef CONFIG_TI_PACKET_PROCESSOR
extern int ti_hil_pp_event(unsigned long val, void *v);
extern void ti_hil_clone_netdev_pp_info(struct net_device *newDev, struct net_device *origDev);
#else
#define ti_hil_pp_event(val, v)
#define ti_hil_clone_netdev_pp_info(newDev, origDev)
#endif /* CONFIG_TI_PACKET_PROCESSOR */

#ifdef CONFIG_TI_PACKET_PROCESSOR_STATS
/* DOCSIS Packet processor session notification API */
typedef int (*TI_HIL_START_SESSION)(unsigned int sessionHandle, 
                                    unsigned int sessionType,
                                    struct sk_buff* skb);
/* DOCSIS Packet processor delete session notification Callback */
typedef int (*TI_HIL_DELETE_SESSION)(unsigned int sessionHandle, 
                                     unsigned int sessionPacketsFw);

extern int ti_hil_register_start_session_notification(TI_HIL_START_SESSION ti_hil_start_session_notification);
extern int ti_hil_unregister_start_session_notification(void);
extern int ti_hil_register_delete_session_notification(TI_HIL_DELETE_SESSION ti_hil_delete_session_notification);
extern int ti_hil_unregister_delete_session_notification(void);
#endif /* CONFIG_TI_PACKET_PROCESSOR_STATS */

/* Power Saving Mode (PSM) API */
extern int ti_hil_enable_psm (void);
extern int ti_hil_disable_psm (void);
#ifdef CONFIG_INTEL_PP_TUNNEL_SUPPORT
extern int ti_hil_set_tunnel_mode(unsigned char tunnelMode);
extern int ti_hil_set_cm_mac_address(unsigned char *cmAddress);
extern int ti_hil_create_tunnel(char *tunnelHeader, unsigned char tunnelHeaderLen, unsigned char l2L3HeaderLen,
                         TUNNEL_TYPE_E tunnelType, unsigned char udpMode);
extern int ti_hil_delete_tunnel(void);
#endif /*CONFIG_INTEL_PP_TUNNEL_SUPPORT*/
#endif /*KERNEL*/
#endif /* __TI_HIL_H__ */


