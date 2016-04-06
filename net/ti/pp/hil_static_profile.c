/*
 * hil_static_profile.c
 *
 *  The file contains the implementation of the HIL Static Profile.
 *  The profile use the following kernel features
 *   - CONFIG_TI_DEVICE_PROTOCOL_HANDLING
 *  This profile uses the input from the user entered at the
 *  command prompt and validates / populates the session data
 *  structure, i.e. LUT and the modification record.
 *  This profile also demonstrates the use of BRIDGE FDB events for
 *  single session per interface.
 *
 *  The profile is provided as is and can be used as a template for
 *  development of more system specific profiles.
 *
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/netdevice.h>
#include <linux/inetdevice.h>
#include <linux/etherdevice.h>
#include <linux/in.h>
#include <linux/if_ether.h>
#include <linux/if_vlan.h>
#include <linux/if_pppox.h>
#include <linux/ip.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/inet.h>
#include <linux/ppp_defs.h>
#include <linux/ti_hil.h>
#include <linux/proc_fs.h>
#include "ti_ppm.h"
#include "../../bridge/br_private.h"

/**************************************************************************
 ************************************ Local Definitions *******************
 **************************************************************************/

/**************************************************************************
 *************************** Static Definitions ***************************
 **************************************************************************/

/* Profile Initialization/Deinitialization and networking event handlers. */
static int ti_hil_static_init (void);
static int ti_hil_netsubsystem_event_handler(unsigned int module_id, unsigned long event_id, void* ptr);
static int ti_hil_static_deinit(void);

/**************************************************************************
 ********************************* Globals ********************************
 **************************************************************************/

/* Default Profile. */
TI_HIL_PROFILE      hil_static_profile = {
    .name            = "static",
    .profile_handler = ti_hil_netsubsystem_event_handler,
    .profile_init    = ti_hil_static_init,
    .profile_deinit  = ti_hil_static_deinit,
};

/* Packet Processor Subsystem Event Handler. */
unsigned int        ppsubsystem_event_handler;

/* PP PDSP Health Timer to verify if the PP is running correctly. */
struct timer_list   pdsp_health_timer;

struct net_bridge_fdb_entry* fdb_db[TI_PP_MAX_ACCLERABLE_SESSIONS];

int session_per_fdb = 0;

/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/
#ifdef CONFIG_TI_PACKET_PROCESSOR_SIMULATION
extern int ti_pp_simulate_sr(struct sk_buff* skb);
#endif /* CONFIG_TI_PACKET_PROCESSOR_SIMULATION */

/**************************************************************************
 ******************************* Functions  *******************************
 **************************************************************************/

/**************************************************************************
 * FUNCTION NAME : ti_hil_get_macaddr
 **************************************************************************
 * DESCRIPTION   :
 *  Utility function validates the MAC entered on the console and converts
 *  it into unsigned char[ETH_ALEN] buffer.
 **************************************************************************/
static int ti_hil_get_macaddr(char *bufp, unsigned char *mac_ptr)
{
	int i, j;
	unsigned char val;
	unsigned char c;

	i = 0;
	do {
		j = val = 0;

		/* Skip semicolon. */
		if (i && (*bufp == ':')) {
			bufp++;
		}

		do {
			c = *bufp;
            /* Convert the char to int */
			if (((unsigned char)(c - '0')) <= 9) {
				c -= '0';
			} else if (((unsigned char)((c|0x20) - 'a')) <= 5) {
				c = (c|0x20) - ('a'-10);
			} else if (j && (c == ':' || c == 0)) {
				break;
			} else {
				return -1;
			}
			++bufp;
			val <<= 4;
			val += c;
		} while (++j < 2);
		*mac_ptr++ = val;
	} while (++i < ETH_ALEN);

	return (int) (*bufp);	/* Error if we don't end at end of string. */
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_device_handler
 **************************************************************************
 * DESCRIPTION   :
 *  Default HIL Device Handler. Since we create VPIDs on demand, ignore
 *  all events - do nothing.
 * RETURNS       :
 *  0   -   Success
 *  <0  -   Error
 **************************************************************************/
static int ti_hil_device_handler(unsigned long event_id, void* ptr)
{
    /* Processing is based on the Event. */
	switch (event_id)
    {
        case NETDEV_UP:
        {
            break;
        }
	    case NETDEV_DOWN:
        {
            break;
        }
        default:
        {
            /* All other events are ignored. */
            return 0;
        }
    }

    /* Work has been successfully completed! */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_inet_handler
 **************************************************************************
 * DESCRIPTION   :
 *  Default HIL INET Handler. Since we create VPIDs on demand, do nothing.
 *
 * RETURNS       :
 *  0   -   Success
 *  <0  -   Error
 **************************************************************************/
static int ti_hil_inet_handler(unsigned long event_id, void* ptr)
{
    /* Processing is based on the Event. */
	switch (event_id)
    {
        case NETDEV_UP:
        {
            break;
        }
	    case NETDEV_DOWN:
        {
            break;
        }
        default:
        {
            /* All other events are ignored. */
            return 0;
        }
    }

    /* Work has been successfully completed! */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_pp_handler
 **************************************************************************
 * DESCRIPTION   :
 *  Default HIL PP Handler. Handle all custom PP events from various
 *  networking modules in the linux stack as per system requirements.
 *
 * RETURNS       :
 *  0   -   Success
 *  <0  -   Error
 **************************************************************************/
static int ti_hil_pp_handler(unsigned long event_id, void* ptr)
{
    struct net_device*           dev = NULL;
    struct net_bridge_fdb_entry* fdb = NULL;
    TI_PP_SESSION                ppd_session;
    TI_PP_SESSION_PROPERTY*      ptr_ses_property = NULL;
    TI_PP_PACKET_DESC*           ptr_pkt_desc = NULL;

    /* Process the events. */
    switch (event_id)
    {
        case TI_BRIDGE_PORT_DELETE:
        {
            /*  Event indicates that a device has been removed from the bridge. This event is generated
             *  when the user executes the "brctl delif" command to remove a port from the bridge.
             *  This implies that the device is no longer capable of pariticipating in networking
             *  and should be removed from the packet processor. But before doing so check if the device
             *  was attached to the Packet processor or not i.e. has valid VPID handle.
             */
            break;
        }
        case TI_BRIDGE_PORT_FORWARD:
        {
            /*  Event indicates that a port attached to the bridge has moved to the FORWARDING state.
             *  The host bridge will only forward packets in this state. Thus this is the time to
             *  create the VPID.
             */
            break;
        }
        case TI_BRIDGE_PORT_DISABLED:
        {
            /*  Event indicates that a port attached to the bridge has been moved to disabled state.
             *  This event is of concern only if the bridge is running the spanning tree protocol. The event
             *  occurs if the spanning tree protocol detects a condition where the port is causing a loop and
             *  disables the port. In such a scenario the VPID and thus all sessions related to the port need
             *  to be removed from the packet processor.
             */
            break;
        }
        case TI_BRIDGE_FDB_CREATED:
        {
            /*  Event indicates that an FDB entry is being created by the bridge. This event is of concern if
             *  "Single session per interface" mode is enabled as an option for session creation (is enabled
             *  here for demonstration purposes by executing command echo "session_per_fdb 1" > /proc/ent/ti_pp"
             *  at the command prompt. If this is mode is not enabled then this event can be ignored.
             */
            if(session_per_fdb)
            {
                fdb = (struct net_bridge_fdb_entry *)ptr;

                /*  Debug Message: To indicate how to access all the information? */
                printk ("PP Static Profile -> Create Session MAC Address 0x%02x-0x%02x-0x%02x-0x%02x-0x%02x-0x%02x is"
                        " on %s VPID:%d\n",
                        fdb->addr.addr[0], fdb->addr.addr[1], fdb->addr.addr[2], fdb->addr.addr[3], fdb->addr.addr[4],
                        fdb->addr.addr[5], fdb->dst->dev->name, fdb->dst->dev->vpid_handle);

                /*  Zero out the session structure before use so that all the fields
                 *  are wild-carded in LUT and modification record except for the ones
                 *  configured below. This is very important otherwise, the session
                 *  matching, LUT lookup could fail.
                 */
                memset((void *)&ppd_session, 0, sizeof(TI_PP_SESSION));

                /**************** Configure the LUT Entry for Classification. ****************/

                /*  Configure the LUT Entry for Layer2 Classification. */
                ptr_ses_property = &ppd_session.ingress;
                ptr_pkt_desc = &ptr_ses_property->l2_packet;
                ptr_pkt_desc->packet_type = TI_PP_ETH_TYPE;

                /* Populate the destination MAC address. */
                memcpy(ptr_pkt_desc->u.eth_desc.dstmac, (void *)&fdb->addr.addr, 6);
                ptr_pkt_desc->u.eth_desc.enables  |= TI_PP_SESSION_L2_DSTMAC_VALID;

                /* Mark the session as a bridged session */
                ppd_session.is_routable_session = 0;

                /*  Configure the egress VPID */
                if(1)
                {
                    /*  Packet is being transmitted on the VPID. Record the necessary information. */
                    ppd_session.num_egress = 1;
                    ppd_session.egress[0].vpid_handle = fdb->dst->dev->vpid_handle;

                    /*  Debug Message. */
                    printk ("Interface: %s has MTU: %d bytes\n", fdb->dst->dev->name, fdb->dst->dev->mtu);
                }

                /*  Create the session using the session record just created */
                if(1)
                {
                    /*  Set the idle session timeout to 60 seconds */
                    ppd_session.session_timeout = 60;

                    fdb->ti_pp_session_handle = ti_ppm_create_session (&ppd_session, 1, 0);
                    if (fdb->ti_pp_session_handle < 0)
                    {
                        printk ("Error: Unable to create the session\n");
                        return -1;
                    }

                    /*  If we are here, the session has been successfully created. Store the FDB entry
                     *  in our local fdb_db so that we can use it to sync the bridge up when the
                     *  corresponding PP session gets deleted on timeout.
                     */
                    fdb_db[fdb->ti_pp_session_handle] = fdb;
                }
            } /* end of if(session_per_fdb) */
            break;
        }
        case TI_BRIDGE_FDB_DELETED:
        {
            /*  Event indicates that an FDB entry is being deleted. The event is of concern if the single session
             *  per interface mode is selected as an option for session creation. If this is not the case then this
             *  event can be ignored.
             */
            if(session_per_fdb)
            {
                fdb = (struct net_bridge_fdb_entry *)ptr;

                /*  Debug Message: To indicate how to access all the information? */
                printk ("PP Static Profile -> Delete Session MAC Address 0x%02x-0x%02x-0x%02x-0x%02x-0x%02x-0x%02x is "
                        "on %s VPID:%d\n",
                        fdb->addr.addr[0], fdb->addr.addr[1], fdb->addr.addr[2], fdb->addr.addr[3], fdb->addr.addr[4],
                        fdb->addr.addr[5], fdb->dst->dev->name, fdb->dst->dev->vpid_handle);

                /*  Look up the PP session database for a session matching the FDB session handle.
                 *  If there is a session matching the same - mark the FDB status to ACTIVE so that
                 *  the bridge doesnt cleanup this FDB entry from its FDB table. This is to maintain
                 *  synchronization between the PP session database and FDB entry table in "One
                 *  Session per interface" mode of the bridge.
                 */
                if (ti_ppm_lookup_session(fdb->ti_pp_session_handle) == 0)
                {
                    /*  If we are here, it implies that FDB entry has a session handle that is no longer
                     *  valid in the context of PP. So mark the FDB entry status INACTIVE so that the bridge cleans
                     *  it up in its next garbage collection cycle.
                     */
                    fdb->ti_pp_fdb_status = TI_PP_FDB_INACTIVE;
                }
                else
                {
                    /*  If session is active, do nothing; leave the FDB as it is.
                     *  To make sure the FDB entry is not cleaned up in the bridge
                     *  by it's garbage collection timer, set the status flag of FDB to ACTIVE.
                     */
                    fdb->ti_pp_fdb_status = TI_PP_FDB_ACTIVE;
                }
            } /* end of if(session_per_fdb) */
            break;
        }
        case TI_BRIDGE_PACKET_FLOODED:
        {
            /*  Event indicates that a packet is being flooded by the bridge on all
             *  ports/devices under it. Thus if BYPASS flag not set on the packet,
             *  multiple sessions could be created in PP for various egress devices
             */
            break;
        }
        case TI_ROUTE_ADDED:
        {
            /*  Event indicates that a new route is being added. A route change
             *  could affect the existing sessions in the PP. So, the HIL profile
             *  could choose to delete all sessions that are affected by the route
             *  change / flush the entire session table to make sure that there
             *  is no incosistency due to the route change.
             */
            break;
        }
        case TI_ROUTE_DELETED:
        {
            /*  Event indicates that an existing route is being added. A route change
             *  could affect the existing sessions in the PP. So, the HIL profile
             *  could choose to delete all sessions that are affected by the route
             *  change / flush the entire session table to make sure that there
             *  is no incosistency due to the route change.
             */
            break;
        }
        case TI_VLAN_DEV_CREATED:
        {
            /*  Event indicates that a new VLAN device has been created. The
             *  pointer to the VLAN device is passed here. The HIL profile could
             *  maintain a database of the VLAN devices existent on the device
             *  and use it when creating modification records based on whether
             *  the egress interface chosen by user is a VLAN device or not by
             *  checking it against this VLAN database.
             */
            dev = (struct net_device *)ptr;

            printk("VLAN dev name: %s VLAN ID: %d \n", dev->name, VLAN_DEV_INFO(dev)->vlan_id);
            break;
        }
        case TI_VLAN_DEV_DELETED:
        {
            /*  Event indicates that a VLAN device is being deleted. The
             *  pointer to this device is passed here. On this event, the
             *  HIL should clean up its reference to the VLAN device so as to
             *  make sure that its not using any stale data to create PP sessions.
             */
            dev = (struct net_device *)ptr;

            printk("VLAN dev name: %s VLAN ID: %d \n", dev->name, VLAN_DEV_INFO(dev)->vlan_id);
            break;
        }
        case TI_PPP_INTERFACE_CREATED:
        {
            /*  Event indicates that a new PPP interface is being created. The
             *  data pointer passed in this event points to the net_device
             *  structure of this new PPP device. The HIL profile could maintain
             *  a database of the PPP interfaces existing on the device and use it
             *  when creating modification records to decide whether a egress interface
             *  specified by the user is a PPP interface, and if so add the appropriate
             *  header by checking with the PPP database. Please note that the PPP session ID
             *  is stored in the "padded" field of the net_device structure, and it must be
             *  captured in this event and saved here locally and not maintain a reference
             *  since its reset on return from this event.
             */
            dev = (struct net_device *)ptr;

            printk("PPP interface name: %s Session ID: %d \n", dev->name, dev->padded);
            break;
        }
        case TI_PPP_INTERFACE_SHUTDOWN:
        {
            /*  Event indicates that a new PPP interface is being created. The
             *  data pointer passed in this event points to the net_device
             *  structure of this new PPP device. On this event, the HIL should
             *  clean up any data (session ID etc) that it might have saved
             *  locally corresponding to this PPP interface.
             */
            dev = (struct net_device *)ptr;

            printk("PPP interface name: %s is being brought down!\n", dev->name);
            break;
        }
        default:
        {
            /* Do not handle this event */
            return 0;
        }
    }

    /* Successfully handled the event. */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_netsubsystem_event_handler
 **************************************************************************
 * DESCRIPTION   :
 *  This is the HIL Intrusive event handler which will capture all events from
 *  the networking sub-system.
 *
 * RETURNS       :
 *  0   -   Success
 *  <0  -   Error
 **************************************************************************/
static int ti_hil_netsubsystem_event_handler(unsigned int module_id, unsigned long event_id, void* ptr)
{
    /* Process based on the module identifer */
    switch (module_id)
    {
        case TI_DEVICE:
        {
            /* HIL Device Handler. */
            ti_hil_device_handler (event_id, ptr);
            break;
        }
        case TI_INET:
        {
            /* HIL Inet Handler. */
            ti_hil_inet_handler (event_id, ptr);
            break;
        }
        case TI_PP:
        {
            /* HIL PP Handler. */
            ti_hil_pp_handler (event_id, ptr);
            break;
        }
    }
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_pp_ppm_event_handler
 **************************************************************************
 * DESCRIPTION   :
 *  The function handles all the events generated by the PPM part of the
 *  Packet Processor Subsystem.
 **************************************************************************/
static void ti_hil_pp_ppm_event_handler (int event, unsigned int param1, unsigned int param2)
{
    /* Process the event accordingly... */
    switch (event)
    {
        case TI_PPM_OUT_OF_MEMORY:
        {
            printk ("FATAL Error: PPM is OUT of memory\n");
            break;
        }
        case TI_PPM_INTERNAL_ERROR:
        {
            printk ("FATAL Error: PPM Internal Error\n");
            break;
        }
        case TI_PPM_CREATE_PID_FAILED:
        case TI_PPM_DELETE_PID_FAILED:
        case TI_PPM_CREATE_VPID_FAILED:
        case TI_PPM_DELETE_VPID_FAILED:
        case TI_PPM_CREATE_SESSION_FAILED:
        case TI_PPM_DELETE_SESSION_FAILED:
        case TI_PPM_MODIFY_SESSION_FAILED:
        {
            printk ("FATAL Error: PP Operation 0x%x Failed\n", event);
            break;
        }
        case TI_PPM_PID_CREATED:
        case TI_PPM_PID_DELETED:
        case TI_PPM_VPID_CREATED:
        case TI_PPM_VPID_DELETED:
        case TI_PPM_SESSION_CREATED:
        case TI_PPM_SESSION_DELETED:
        case TI_PPM_SESSION_MODIFIED:
        {
            printk ("DEBUG: PP Operation 0x%x Successful\n", event);
            break;
        }
    }

    /* Work is done! */
    return;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_pp_event_handler
 **************************************************************************
 * DESCRIPTION   :
 *  The function handles all the events generated by the PP PDSP.
 **************************************************************************/
static void ti_hil_pp_event_handler (int event, unsigned int param1, unsigned int param2)
{
    /* Process the event accordingly... */
    switch (event)
    {
        case TI_PP_SESSION_EXPIRATION:
        {
            /* Session has EXPIRED */
            TI_PP_SESSION_STATS    session_stats;

            /* Timer has expired for a session; time to delete the session. */
            if (ti_ppm_delete_session (param1, &session_stats) < 0)
            {
                printk ("Error: Unable to delete session %d\n", param1);
                return;
            }

            /* If we are here, the PP session has been deleted successfully.
               So clean up the session handle in the corresponding fdb entry
               in our fdb_db.
            */
            if(session_per_fdb)
            {
                if(fdb_db[param1] != NULL)
                {
                    fdb_db[param1]->ti_pp_session_handle = -1;
                    fdb_db[param1] = NULL;
                }
            }

            /* Print the session stats. */
            printk ("--------- Session Stats %d ---------\n", param1);
            printk ("Number of Packets Forwarded: %d\n", session_stats.packets_forwarded);
            printk ("Number of Bytes   Forwarded: %d\n", session_stats.bytes_forwarded_hi);
            printk ("Number of Bytes   Forwarded: %d\n", session_stats.bytes_forwarded_lo);
            printk ("------------------------------------\n");
            break;
        }
        default:
        {
            /* Unknown PP Event Generated! */
            printk ("PP Generated Event 0x%x which is UNHANDLED\n", event);
            break;
        }
    }

    /* Work is done! */
    return;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_ppsubsystem_event_handler
 **************************************************************************
 * DESCRIPTION   :
 *  The function is the registered event handler which listens to all events
 *  that arise from the Packet Processor Subsystem.
 **************************************************************************/
static void ti_hil_ppsubsystem_event_handler(unsigned int event_id, unsigned int param1, unsigned int param2)
{
    int module;
    int event;
    int subsystem;

    /* We should only get events generated by the PP Sub-System. */
    subsystem = event_id & TI_PP_SUBSYSTEM_BIT_MASK;
    if (subsystem != 0)
    {
        printk ("FATAL Error: Event 0x%x violates the specification\n", event_id);
        return;
    }

    /* Get information on the module which generated the event and the actual event identifier. */
    module = event_id & TI_PP_MODULE_BIT_MASK;
    event  = event_id & TI_PP_EVENT_BIT_MASK;

    /* Process each event based on the module which generated the event. */
    switch (module)
    {
        case TI_PP_MODULE:
        {
            /* Packet Processor Event */
            ti_hil_pp_event_handler (event, param1, param2);
            break;
        }
        case TI_PPM_MODULE:
        {
            /* Packet Processor Manager Event. */
            ti_hil_pp_ppm_event_handler (event, param1, param2);
            break;
        }
        case TI_PPD_MODULE:
        {
            /* Packet Processor Driver Event. */
            printk ("PPD Event 0x%x Param1: 0x%x Param2: 0x%x\n", event, param1, param2);
            break;
        }
        default:
        {
            printk ("FATAL Error: Event 0x%x violates the specification\n", event_id);
            break;
        }
    }

    /* Our work is done. */
    return;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_static_display_l2
 **************************************************************************
 * DESCRIPTION   :
 *  The function prints the Layer2 Information
 **************************************************************************/
static void ti_hil_static_display_l2 (TI_PP_ETH_DESC* ptr_eth_desc)
{
    /* Check if the Destination MAC Address has been specified. */
    if (ptr_eth_desc->enables & TI_PP_SESSION_L2_DSTMAC_VALID)
        printk ("Dst. MAC     = %02x:%02x:%02x:%02x:%02x:%02x\n", ptr_eth_desc->dstmac[0], ptr_eth_desc->dstmac[1],
                ptr_eth_desc->dstmac[2], ptr_eth_desc->dstmac[3], ptr_eth_desc->dstmac[4], ptr_eth_desc->dstmac[5]);
    else
        printk ("Dst. MAC     = XXX\n");

    /* Check if the Source MAC Address has been specified. */
    if (ptr_eth_desc->enables & TI_PP_SESSION_L2_SRCMAC_VALID)
        printk ("Src. MAC     = %02x:%02x:%02x:%02x:%02x:%02x\n", ptr_eth_desc->srcmac[0], ptr_eth_desc->srcmac[1],
                ptr_eth_desc->srcmac[2], ptr_eth_desc->srcmac[3], ptr_eth_desc->srcmac[4], ptr_eth_desc->srcmac[5]);
    else
        printk ("Src. MAC     = XXX\n");

    /* Work is completed. */
    return;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_static_display_ipv4
 **************************************************************************
 * DESCRIPTION   :
 *  The function prints the IPv4 Information
 **************************************************************************/
static void ti_hil_static_display_ipv4 (TI_PP_IPV4_DESC* ptr_ipv4_lut_entry)
{
    /* Check if the Destination IP has been specified. */
    if (ptr_ipv4_lut_entry->enables & TI_PP_SESSION_IPV4_DSTIP_VALID)
        printk ("Dst IP       = 0x%x\n", ptr_ipv4_lut_entry->dst_ip);
    else
        printk ("Dst IP       = XXX\n");

    /* Check if the Source IP has been specified. */
    if (ptr_ipv4_lut_entry->enables & TI_PP_SESSION_IPV4_SRCIP_VALID)
        printk ("Src IP       = 0x%x\n", ptr_ipv4_lut_entry->src_ip);
    else
        printk ("Src IP       = XXX\n");

    /* Check if the protocol has been specified. */
    if (ptr_ipv4_lut_entry->enables & TI_PP_SESSION_IPV4_PROTOCOL_VALID)
        printk ("Protocol     = 0x%x\n", ptr_ipv4_lut_entry->protocol);
    else
        printk ("Protocol     = XXX\n");

    /* Check if the TOS Byte has been specified. */
    if (ptr_ipv4_lut_entry->enables & TI_PP_SESSION_IPV4_TOS_VALID)
        printk ("TOS          = 0x%x\n", ptr_ipv4_lut_entry->tos);
    else
        printk ("TOS          = XXX\n");

    /* Check if the Destination Port has been specified. */
    if (ptr_ipv4_lut_entry->enables & TI_PP_SESSION_IPV4_DST_PORT_VALID)
        printk ("Dst Port     = %d\n", ptr_ipv4_lut_entry->dst_port);
    else
        printk ("Dst Port     = XXX\n");

    /* Check if the Source Port has been specified. */
    if (ptr_ipv4_lut_entry->enables & TI_PP_SESSION_IPV4_SRC_PORT_VALID)
        printk ("Src Port     = %d\n", ptr_ipv4_lut_entry->src_port);
    else
        printk ("Src Port     = XXX\n");

    /* Work is completed. */
    return;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_static_display_session
 **************************************************************************
 * DESCRIPTION   :
 *  The function displays the session properties.
 **************************************************************************/
static void ti_hil_static_display_session (int session_handle)
{
    TI_PP_SESSION    session;
    int              index;

    /* Get the pointer to the Session Information. */
    if (ti_ppm_get_session_info(session_handle, &session) < 0)
        return;

    /* Print the Session Parameters. */
    printk ("Property : %s\n", session.is_routable_session ? "Routed": "Bridged");
    printk ("Timeout  : %d\n", session.session_timeout);
    printk ("Priority : %d\n", session.priority);

    /* Print the Ingress Session Properties; which include the L2/L3 and L4 properties. */
    printk ("**** Ingress Properties ****\n");
    printk ("Ingress VPID = %d\n", session.ingress.vpid_handle);
    if (session.ingress.l2_packet.packet_type == TI_PP_ETH_TYPE)
        ti_hil_static_display_l2 (&session.ingress.l2_packet.u.eth_desc);
    else
        printk ("No L2 Properties\n");
    if (session.ingress.l3l4_packet.packet_type == TI_PP_IPV4_TYPE)
        ti_hil_static_display_ipv4 (&session.ingress.l3l4_packet.u.ipv4_desc);
    else
        printk ("No L3/L4 Properties\n");

    /* Cycle through all the Egress Session Properties. */
    for (index = 0; index < session.num_egress; index++)
    {
        printk ("**** Egress Properties %d ****\n", index+1);

        /* Print the Ingress Session Properties; which include the L2/L3 and L4 properties. */
        printk ("Egress VPID  = %d\n", session.egress[index].vpid_handle);
        if (session.egress[index].l2_packet.packet_type == TI_PP_ETH_TYPE)
            ti_hil_static_display_l2 (&session.egress[index].l2_packet.u.eth_desc);
        else
            printk ("No L2 Properties\n");
        if (session.egress[index].l3l4_packet.packet_type == TI_PP_IPV4_TYPE)
            ti_hil_static_display_ipv4 (&session.egress[index].l3l4_packet.u.ipv4_desc);
        else
            printk ("No L3/L4 Properties\n");
    }

    /* Work has been completed. */
    return;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_show_cmd_handler
 **************************************************************************
 * DESCRIPTION   :
 *  This function is the Packet Processor show command handler.
 *
 * RETURNS       :
 *  -1      - Error.
 *  0       - Success.
 ***************************************************************************/
static int ti_hil_show_cmd_handler(int argc, char* argv[])
{
    TI_PP_GLOBAL_STATS pp_stats;

    /****************************** VALIDATIONS ***************************/

    /* Validate the number of arguments that have been passed. */
    if (argc != 2)
    {
        printk ("ERROR: Incorrect Number of parameters passed.\n");
        return -1;
    }

    /* Display help options */
    if(!strcmp(argv[1], "help"))
    {
        printk("\n========= VPID menu ===========\n");
        printk("add_vpid interface_name [vlan_id <vlan_id value>] [ppp_id <ppp_id value>] \n");
        printk("Note: please note that vlan_id must be specified before ppp_id if both present \n");
        printk("\ndel_vpid interface_name \n");

        printk("\n========= SESSION menu ===========\n");
        printk("add_session ivpid <ivpid_handle> evpid <evpid_handle> {ingress_options} modify 1 {egress_modifcation_options} \n");
        printk("\nWhere ingress_options could be: \n");
        printk("[smac <source mac>] [dmac <destn mac>] [sip <source IP>] [dip <destn IP>] \n"
               "[sport <source port>] [dport <destn port>] [prot <protocol no>] [tos <tos value>] \n");
        printk("\nWhere egress_modifications_options could be: \n");
        printk("[smac <source mac>] [dmac <destn mac>] [sip <source IP>] [dip <destn IP>] \n"
               "[sport <source port>] [dport <destn port>]\n");
        printk("\nNote: Ingress Paramaters must be specified before specifying 'modify 1' and all params specified \n"
               "after 'modify 1' to add_session command are treated as egress modification record values \n");
        printk("\ndel_session <session_id> \n");

        printk("\n========= DISPLAY menu ===========\n");
        printk("show [help | global | pid | vpid | session] \n");

        /* Work is completed. */
        return 0;
    }

    /* Check if the "Global" stats were requested. */
    if (strcmp(argv[1], "global") == 0)
    {
        /* YES. Get the global stats through the PPM */
        ti_ppm_get_global_stats(&pp_stats);

        /* Print the stats on the console. */
        printk ("Packets received in the PP   : %d\n", pp_stats.packets_rxed);
        printk ("Number of search attemps     : %d\n", pp_stats.packets_searched);
        printk ("Number of matched searches   : %d\n", pp_stats.search_matched);
        printk ("Number of Synch Delays       : %d\n", pp_stats.sync_delay);
        printk ("Packet forwarded by the PP   : %d\n", pp_stats.packets_fwd);
        printk ("IPv4 Packets Forwarded       : %d\n", pp_stats.ipv4_packets_fwd);
        printk ("Descriptors Starved          : %d\n", pp_stats.desc_starved);
        printk ("Buffers Starved              : %d\n", pp_stats.buffer_starved);

        /* Work is completed. */
        return 0;
    }

    /* Check if VPID statistics were requested? */
    if (strcmp(argv[1], "vpid") == 0)
    {
        TI_PP_VPID   vpid[TI_PP_MAX_PID + 1];
        int         num_vpid;
        int         index = 0;

        /* Get a list of all VPID that exist in the System */
        num_vpid = ti_ppm_get_vpid (-1, TI_PP_MAX_PID, &vpid[0]);

        /* Cycle through all the VPID and get the stats */
        while (index < num_vpid)
        {
            TI_PP_VPID_STATS   vpid_stats;

            /* Get the VPID statistics. */
            ti_ppm_get_vpid_stats(vpid[index].vpid_handle, &vpid_stats);

            /* Print the statistics on the console. */
            printk ("---------------- VPID %d ----------------\n", vpid[index].vpid_handle);
            printk ("Rx Unicast   Packets: %d\n", vpid_stats.rx_unicast_pkt);
            printk ("Rx Broadcast Packets: %d\n", vpid_stats.rx_broadcast_pkt);
            printk ("Rx Multicast Packets: %d\n", vpid_stats.rx_multicast_pkt);
            printk ("Rx Bytes(HI)        : %d\n", vpid_stats.rx_byte_hi);
            printk ("Rx Bytes(LO)        : %d\n", vpid_stats.rx_byte_lo);
            printk ("Rx Discard          : %d\n", vpid_stats.rx_discard);
            printk ("Tx Unicast   Packets: %d\n", vpid_stats.tx_unicast_pkt);
            printk ("Tx Broadcast Packets: %d\n", vpid_stats.tx_broadcast_pkt);
            printk ("Tx Multicast Packets: %d\n", vpid_stats.tx_multicast_pkt);
            printk ("Tx Bytes(HI)        : %d\n", vpid_stats.tx_byte_hi);
            printk ("Tx Bytes(LO)        : %d\n", vpid_stats.tx_byte_lo);
            printk ("Tx Errors           : %d\n", vpid_stats.tx_error);
            printk ("Tx Discards         : %d\n", vpid_stats.tx_discard);
            printk ("-----------------------------------------\n");

            /* Goto the next VPID. */
            index = index + 1;
        }
        /* Work is completed. */
        return 0;
    }

    /* Check if PID needs to be displayed? */
    if (strcmp(argv[1], "pid") == 0)
    {
        int         num_pid;
        TI_PP_PID   pid[TI_PP_MAX_PID+1];
        int         index = 0;

        /* Get the PID Information. */
        num_pid = ti_ppm_get_pid (TI_PP_MAX_PID, &pid[0]);

        /* Cycle through all the VPID and get the stats */
        while (index < num_pid)
        {
            /* Print the PID Information on the console. */
            printk ("PID %d\n", pid[index].pid_handle);

            /* Goto the next PID. */
            index = index + 1;
        }

        /* Work is completed. */
        return 0;
    }

    /* Check if Session needs to be displayed? */
    if (strcmp(argv[1], "session") == 0)
    {
        int               num_session;
        unsigned char*    session;
        int               index = 0;
        TI_PP_SESSION_STATS session_stats;

        /* Get the number of sessions that are available. */
        num_session = ti_ppm_get_session (-1, 0, NULL);
        printk ("Detected %d sessions in packet processor\n", num_session);

        /* Allocate memory for the sessions. */
        session = (unsigned char *)kmalloc(sizeof(unsigned char)*num_session, GFP_KERNEL);
        if (session == NULL)
            return -1;

        /* Get the Session Information - array of all the session indices */
        num_session = ti_ppm_get_session (-1, num_session, session);

        /* Cycle through all the VPID and get the stats */
        while (index < num_session)
        {
            /* Print the Session Information on the console. */
            printk ("---------------- Session %d ----------------\n", session[index]);
            ti_hil_static_display_session(session[index]);

            memset((void *)&session_stats, 0, sizeof(TI_PP_SESSION_STATS));
            ti_ppm_get_session_stats(session[index], &session_stats);

            /* Print the session stats. */
            printk ("--------- Session Stats %s ---------\n", argv[1]);
            printk ("Number of Packets Forwarded: %d\n", session_stats.packets_forwarded);
            printk ("Number of Bytes   Forwarded: %d\n", session_stats.bytes_forwarded_hi);
            printk ("Number of Bytes   Forwarded: %d\n", session_stats.bytes_forwarded_lo);
            printk ("------------------------------------\n");

            /* Goto the next Session */
            index = index + 1;
        }

        /* Free the session buffer memory */
        kfree(session);

        /* Work is completed. */
        return 0;
    }

    /* Control comes here if the scope was not understood. */
    return -1;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_vpid_cmd_handler
 **************************************************************************
 * DESCRIPTION   :
 *  This function is the Packet Processor VPID command handler. Handles
 *  "add_vpid" and "del_vpid" commands.
 *
 * RETURNS       :
 *  -1      - Error.
 *  0       - Success.
 ***************************************************************************/
static int ti_hil_vpid_cmd_handler(int argc, char* argv[])
{
	struct net_device *dev = NULL;
    int i = 0;

    /****************************** VALIDATIONS ***************************/

    /* Validate the number of arguments that have been passed. */
    if (argc < 2)
    {
        printk ("ERROR: Incorrect Number of parameters passed.\n");
        return -1;
    }

    if(argv[1] != NULL && strlen(argv[1]) > 0)
    {
        dev = dev_get_by_name(argv[1]);
        if(dev)
        {
            /* Add a VPID on the device entered */
            if(!strcmp(argv[0], "add_vpid"))
            {
                /* Check if the device had a valid VPID handle. */
                if (dev->vpid_handle == -1)
                {
                    /* Scan rest of the args to check if a VLAN Id / PPP Session ID is specified */
                    for(i = 2; argc > 2 && i < argc; i++)
                    {
                        if(!strcmp(argv[i], "vlan_id"))
                        {
                            i += 1;
                            dev->vpid_block.type            = TI_PP_VLAN;
                            dev->vpid_block.vlan_identifier = (unsigned short)simple_strtol(argv[i], NULL, 0);
                            dev->vpid_block.egress_mtu      = dev->vpid_block.egress_mtu - VLAN_HLEN;
                        }
                        else if (!strcmp(argv[i], "ppp_id"))
                        {
                            i += 1;
                            if (dev->vpid_block.type == TI_PP_VLAN)
                            {
                                /* The PPP connection is initialized over a VLAN connection. Configure the Egress MTU
                                 * appropriately accounting for the PPP and VLAN headers. */
                                dev->vpid_block.type       = TI_PP_VLAN_PPPoE;
                                dev->vpid_block.egress_mtu = dev->vpid_block.egress_mtu - sizeof(struct pppoe_hdr)
                                                             - 2 - VLAN_HLEN;
                            }
                            else
                            {
                                /* The PPP Connection is being bought over a vanilla Ethernet connection. Configure
                                 * the Egress MTU appropriately accounting only for the PPP header */
                                dev->vpid_block.type        = TI_PP_PPPoE;
                                dev->vpid_block.egress_mtu  = dev->vpid_block.egress_mtu - sizeof(struct pppoe_hdr) - 2;
                            }

                            /* Extract and configure the PPP Session ID. */
                            dev->vpid_block.ppp_session_id  = (unsigned short)simple_strtol(argv[i], NULL, 0);
                        }
                        else
                        {
                            printk("Invalid paramater passed %s \n", argv[i]);
                            return -1;
                        }
                    }
                    /* NO. The device needs to be created as a VPID in the packet processor. */
                    dev->vpid_handle = ti_ppm_create_vpid (&dev->vpid_block);
                    if (dev->vpid_handle < 0)
                    {
                        printk ("Error: Unable to create VPID %d PID %d Device %s\n", dev->vpid_handle, dev->pid_handle, dev->name);
                        return -1;
                    }
                    printk ("Succesfully created VPID %d PID %d Device %s\n",  dev->vpid_handle, dev->pid_handle, dev->name);
                }
            } /* End of add_vpid */

            /* Delete the VPID installed on the device entered */
            if (!strcmp(argv[0], "del_vpid"))
            {
                /* Check if the device had a valid VPID handle. */
                if (dev->vpid_handle != -1)
                {
                    /* YES. The device needs to be removed from the packet processor. */
                    if (ti_ppm_delete_vpid (dev->vpid_handle) < 0)
                    {
                        printk ("Error: Unable to delete VPID %d PID %d Device %s\n", dev->vpid_handle, dev->pid_handle, dev->name);
                        return -1;
                    }
                    printk ("Succesfully deleted VPID %d PID %d Device %s\n",  dev->vpid_handle, dev->pid_handle, dev->name);
                    dev->vpid_handle = -1;
                }
            } /* End of del_vpid */
        } /* End of if(dev) */
        else
        {
            printk("%s is not a valid device! \n", argv[1]);
            return -1;
        }
    }
    else
    {
        printk ("ERROR: No device name passsed?\n");
        return -1;
    }

    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_session_cmd_handler
 **************************************************************************
 * DESCRIPTION   :
 *  This function is the Packet Processor Session command handler. Handles
 *  "add_session" and "del_session" commands.
 *
 * RETURNS       :
 *  -1      - Error.
 *  0       - Success.
 ***************************************************************************/
static int ti_hil_session_cmd_handler(int argc, char* argv[])
{
    int index = 1;
    TI_PP_SESSION   ptr_session;
    unsigned char mac_addr[ETH_ALEN];
    int session_handle = -1;
    int modify = 0;
    unsigned short sport = 0, dport = 0;
    unsigned int src_ip = 0, dst_ip = 0;
    TI_PP_PACKET_DESC*      ptr_pkt_desc = NULL;
    TI_PP_SESSION_PROPERTY* ptr_ses_property = NULL;
    TI_PP_SESSION_PROPERTY* ptr_egr_ses_property = NULL;

    /*  Zero out the session structure before use so that all the fields
     *  are wild-carded in LUT and modification record except for the ones
     *  configured below. This is very important otherwise, the session
     *  matching, i.e., LUT lookup could fail.
     */
    memset(&ptr_session, 0, sizeof(TI_PP_SESSION));
    ptr_ses_property = &ptr_session.ingress;

    /* Validate the number of arguments that have been passed. */
    if (argc < 2)
    {
        printk ("ERROR: Incorrect Number of parameters passed.\n");
        return -1;
    }

    /* Session to be added? */
    if(!strcmp(argv[0], "add_session"))
    {
        /* Loop through all the parameters passed and configure the LUT and the modification records. */
        while (index <= argc )
        {
            /*  Validate the <parameter,value> tuple entered.Make sure that the parameter is followed
             *  by a non-null valid value.
             */
            if((argv[index] != NULL && strlen(argv[index]) > 0) &&
               (index+1 <= argc) && (argv[index+1] != NULL && strlen(argv[index+1]) > 0))
            {
                /* Begin of modification record  configuration */
                if(!strcmp(argv[index], "modify"))
                {
                    index += 1;

                    modify = 1;
                    /* If any of L2/L3/L4 properties being modified, probably not bridged
                     * so mark the packet routed
                     */
                    ptr_session.is_routable_session = 1;
                }
                /* Source MAC configured passed for LUT/modification record configuration. */
                else if(!strcmp(argv[index], "smac"))
                {
                    index += 1;

                    /* Convert the MAC address entered on the console to standard linux format */
                    if(ti_hil_get_macaddr(argv[index], mac_addr))
                        return -1;
                    /* Validate the MAC address - Make sure its not a bunch of zeros/fs */
	                if (!is_valid_ether_addr(mac_addr))
                        return -1;

                    /* Modify not set. Configure the source MAC of the LUT data. */
                    if(!modify)
                    {
                        ptr_pkt_desc = &ptr_ses_property->l2_packet;
                    }
                    /* Modify set. Configure the modification record for source mac address */
                    else
                    {
                        ptr_pkt_desc = &ptr_egr_ses_property->l2_packet;
                    }
                    /* YES. Ethernet header was detected. Initialize the various fields. */
                    ptr_pkt_desc->packet_type = TI_PP_ETH_TYPE;

                    /* Populate the source MAC address. */
                    memcpy(ptr_pkt_desc->u.eth_desc.srcmac, (void *)&mac_addr, 6);
                    ptr_pkt_desc->u.eth_desc.enables  |= TI_PP_SESSION_L2_SRCMAC_VALID;
                }
                /* Dst MAC configured passed for LUT/modification record configuration. */
                else if(!strcmp(argv[index], "dmac"))
                {
                    index += 1;

                    /* Convert the MAC address entered on the console to standard linux format */
                    if(ti_hil_get_macaddr(argv[index], mac_addr))
                        return -1;
                    /* Validate the MAC address - Make sure its not a bunch of zeros/fs */
	                if (!is_valid_ether_addr(mac_addr))
                        return -1;

                    /* Modify not set. Configure the destination MAC of the LUT data */
                    if(!modify)
                    {
                        ptr_pkt_desc = &ptr_ses_property->l2_packet;
                    }
                    /* Modify set. Configure the modification record for destination MAC */
                    else
                    {
                        ptr_pkt_desc = &ptr_egr_ses_property->l2_packet;
                    }
                    /* YES. Ethernet header was detected. Initialize the various fields. */
                    ptr_pkt_desc->packet_type = TI_PP_ETH_TYPE;

                    /* Populate the destination MAC address. */
                    memcpy(ptr_pkt_desc->u.eth_desc.dstmac, (void *)&mac_addr, 6);
                    ptr_pkt_desc->u.eth_desc.enables  |= TI_PP_SESSION_L2_DSTMAC_VALID;
                }
                /* Layer3 Source IP address configured for LUT/modification record configuration */
                else if(!strcmp(argv[index], "sip"))
                {
                    index += 1;

                    src_ip = (unsigned int)in_aton(argv[index]);

                    /* Modify not set. Configure the source IP of LUT data */
                    if(!modify)
                    {
                        ptr_pkt_desc = &ptr_ses_property->l3l4_packet;
                    }
                    /* Modify set. Configure the source IP of the modification record */
                    else
                    {
                        ptr_pkt_desc = &ptr_egr_ses_property->l3l4_packet;
                    }
                    /* Yes. IPv4 header was detected. Initialize the various fields. */
                    ptr_pkt_desc->packet_type = TI_PP_IPV4_TYPE;

                    /* Populate the Source IP Address. */
                    ptr_pkt_desc->u.ipv4_desc.src_ip = src_ip;
                    ptr_pkt_desc->u.ipv4_desc.enables  |= TI_PP_SESSION_IPV4_SRCIP_VALID;
                }
                /* Layer3 Destination IP address configured for the LUT data/modification record */
                else if(!strcmp(argv[index], "dip"))
                {
                    index += 1;

                    dst_ip = (unsigned int)in_aton(argv[index]);

                    /* Modify not set. Configure the Destination IP of LUT data */
                    if(!modify)
                    {
                        ptr_pkt_desc = &ptr_ses_property->l3l4_packet;
                    }
                    /* Modify set. Configure the destination IP in the modification record */
                    else
                    {
                        ptr_pkt_desc = &ptr_egr_ses_property->l3l4_packet;
                    }
                    /* Yes. IPv4 header was detected. Initialize the various fields. */
                    ptr_pkt_desc->packet_type = TI_PP_IPV4_TYPE;

                    /* Populate the Destination IP Address. */
                    ptr_pkt_desc->u.ipv4_desc.dst_ip = dst_ip;
                    ptr_pkt_desc->u.ipv4_desc.enables  |= TI_PP_SESSION_IPV4_DSTIP_VALID;
                }
                /* Layer4 Protocol configured for LUT data configuration. */
                else if(!strcmp(argv[index], "prot"))
                {
                    index += 1;

                    ptr_pkt_desc = &ptr_ses_property->l3l4_packet;

                    /* Yes. IPv4 header was detected. Initialize the various fields. */
                    ptr_pkt_desc->packet_type = TI_PP_IPV4_TYPE;

                    /* Populate the Protocol */
                    ptr_pkt_desc->u.ipv4_desc.protocol = (unsigned char)simple_strtol(argv[index], NULL, 0);
                    ptr_pkt_desc->u.ipv4_desc.enables  |= TI_PP_SESSION_IPV4_PROTOCOL_VALID;
                }
                /* ToS configured for LUT data configuration */
                else if(!strcmp(argv[index], "tos"))
                {
                    index += 1;

                    ptr_pkt_desc = &ptr_ses_property->l3l4_packet;

                    /* Yes. IPv4 header was detected. Initialize the various fields. */
                    ptr_pkt_desc->packet_type = TI_PP_IPV4_TYPE;

                    /* Populate the TOS Byte */
                    ptr_pkt_desc->u.ipv4_desc.tos = (unsigned char)simple_strtol(argv[index], NULL, 0);
                    ptr_pkt_desc->u.ipv4_desc.enables  |= TI_PP_SESSION_IPV4_TOS_VALID;
                }
                /* Layer4 Source Port configured for LUT data / modification record */
                else if(!strcmp(argv[index], "sport"))
                {
                    index += 1;

                    sport = (unsigned short)simple_strtol(argv[index], NULL, 0);

                    /* Modify not set. Configure the Source port in the LUT data */
                    if(!modify)
                    {
                        ptr_pkt_desc = &ptr_ses_property->l3l4_packet;
                    }
                    /* Modify set. Configure the source port in the modification record */
                    else
                    {
                        ptr_pkt_desc = &ptr_egr_ses_property->l3l4_packet;
                    }
                    /* Yes. IPv4 header was detected. Initialize the various fields. */
                    ptr_pkt_desc->packet_type = TI_PP_IPV4_TYPE;

                    /* Populate the Source Port. */
                    ptr_pkt_desc->u.ipv4_desc.src_port = sport;
                    ptr_pkt_desc->u.ipv4_desc.enables |= TI_PP_SESSION_IPV4_SRC_PORT_VALID;
                }
                /* Layer4 Destination Port configured for LUT data / modification record */
                else if(!strcmp(argv[index], "dport"))
                {
                    index += 1;

                    dport = (unsigned short)simple_strtol(argv[index], NULL, 0);

                    /* Modify not set. Configure the destination port in LUT data */
                    if(!modify)
                    {
                        ptr_pkt_desc = &ptr_ses_property->l3l4_packet;
                    }
                    /* Modify set. Configure the destination port in the modification record */
                    else
                    {
                        ptr_pkt_desc = &ptr_egr_ses_property->l3l4_packet;
                    }
                    /* Yes. IPv4 header was detected. Initialize the various fields. */
                    ptr_pkt_desc->packet_type = TI_PP_IPV4_TYPE;

                    /* Copy the destination port */
                    ptr_pkt_desc->u.ipv4_desc.dst_port = dport;
                    ptr_pkt_desc->u.ipv4_desc.enables |= TI_PP_SESSION_IPV4_DST_PORT_VALID;
                }
                /* Ingress VPID configured for the LUT data */
                else if(!strcmp(argv[index], "ivpid"))
                {
                    index += 1;

                    ptr_ses_property->vpid_handle = (unsigned char)simple_strtol(argv[index], NULL, 0);
                }
                /*  Egress VPID configured for the modification record. Unless a valid egress VPID
                 *  is configured, the session is not created. This should be configured at the
                 *  begining of the param list.
                 *  Currently, support only for one egress vpid. Can be easily extended for multiple.
                 */
                else if(!strcmp(argv[index], "evpid"))
                {
                    index += 1;

                    ptr_session.num_egress = 1;

                    ptr_egr_ses_property = &ptr_session.egress[0];

                    ptr_egr_ses_property->vpid_handle = (unsigned char)simple_strtol(argv[index], NULL, 0);
                }
            } /* End if argv[index] != NULL .. */
            index ++;
        } /* End of while (index <= argc) */

        /* Create a static session using the session record just created */
        if(ptr_session.num_egress > 0)
        {
            ptr_session.session_timeout = 0;
            session_handle = ti_ppm_create_session (&ptr_session, 1, 0);
            if (session_handle < 0)
            {
                printk ("Error: Unable to create the session %d\n", session_handle);
                return -1;
            }
            else
                printk("Session %d successfully created \n", session_handle);
        } /* End of if(ptr_session.num_egress > 0) */
    } /* End of add_session */
    else if(!strcmp(argv[0], "del_session"))
    {
        TI_PP_SESSION_STATS    session_stats;

        if(argv[1] != NULL && strlen(argv[1]) > 0)
        {
            if (ti_ppm_delete_session ((int) simple_strtol(argv[1], NULL, 0), &session_stats) < 0)
            {
                printk ("Error: Unable to delete session %s\n", argv[1]);
                return -1;
            }

            /* Print the session stats. */
            printk ("--------- Session Stats %s ---------\n", argv[1]);
            printk ("Number of Packets Forwarded: %d\n", session_stats.packets_forwarded);
            printk ("Number of Bytes   Forwarded: %d\n", session_stats.bytes_forwarded_hi);
            printk ("Number of Bytes   Forwarded: %d\n", session_stats.bytes_forwarded_lo);
            printk ("------------------------------------\n");
        } /* End of if(argv[1] != NULL) && ... */
    } /* End of del_session */

    /* Work done! Return now! */
    return 0;
}


/**************************************************************************
 * FUNCTION NAME : ti_hil_write_cmds
 **************************************************************************
 * DESCRIPTION   :
 *  Interface for the Intrusive HIL. This is used to debug and display various
 *  packet processor entity information from the console.
 *
 * RETURNS       :
 *  -1              - Error.
 *  Non-Zero        - Success.
 ***************************************************************************/
static int ti_hil_write_cmds (struct file *file, const char *buffer, unsigned long count, void *data)
{
	char    pp_cmd[500];
	char*	argv[50];
	int		argc = 0;
	char*	ptr_cmd;
    char*   delimitters = " \n\t";
    char*   ptr_next_tok;

	/* Validate the length of data passed. */
	if (count > 500)
		count = 500;

    /* Initialize the buffer before using it. */
    memset ((void *)&pp_cmd[0], 0, sizeof(pp_cmd));
    memset ((void *)&argv[0], 0, sizeof(argv));

	/* Copy from user space. */
	if (copy_from_user (&pp_cmd, buffer, count))
		return -EFAULT;

    ptr_next_tok = &pp_cmd[0];

	/*  Tokenize the command. Check if there was a NULL entry. If so be the case the
     *  user did not know how to use the entry. Print the help screen.
     */
	ptr_cmd = strsep(&ptr_next_tok, delimitters);
    if (ptr_cmd == NULL)
        return -1;

    /* Parse all the commands typed. */
	do
	{
        /* Extract the first command. */
		argv[argc++] = ptr_cmd;

        /* Validate if the user entered more commands.*/
		if (argc >= 50)
		{
			printk ("ERROR: Incorrect too many parameters dropping the command\n");
			return -EFAULT;
		}

        /* Get the next valid command. */
		ptr_cmd = strsep(&ptr_next_tok, delimitters);
	} while (ptr_cmd != NULL);

	/* We have an extra argument when strsep is used instead of strtok */
	argc--;

    /******************************* Command Handlers *******************************/

    /* Display Command Handlers */
    if (strncmp(argv[0], "show", strlen("show")) == 0)
    {
        /* Call the Show Command Handler. */
        if (ti_hil_show_cmd_handler (argc, argv) < 0)
            return count;
    }

    /* Deinitialize Command Handlers */
    if (strncmp(argv[0], "deinit", strlen("deinit")) == 0)
    {
        /* Deinitialize all the packet processor components. */
        if (ti_ppm_deinitialize() < 0)
        {
            printk ("Error: Packet Processor Deinitialization Failed\n");
            return count;
        }
    }

    /* Sessions to be flushed? */
    if (strncmp(argv[0], "flush_all_sessions", strlen("flush_all_sessions")) == 0)
    {
        /* Call flush sessions API with -1 */
        if(ti_ppm_flush_sessions(-1) < 0)
            return count;
    }

    /* VPID Command Handlers */
    if ((strncmp(argv[0], "add_vpid", strlen("add_vpid")) == 0) ||
        (strncmp(argv[0], "del_vpid", strlen("del_vpid")) == 0))
    {
        /* Call the VPID Command Handler. */
        if (ti_hil_vpid_cmd_handler (argc, argv) < 0)
            return count;
    }

    /* Session Command Handlers */
    if ((strncmp(argv[0], "add_session", strlen("add_session")) == 0) ||
        (strncmp(argv[0], "del_session", strlen("del_session")) == 0))
    {
        /* Call the Session Command Handler. */
        if (ti_hil_session_cmd_handler (argc, argv) < 0)
            return count;
    }

    /* Enable/Disable adding static sessions based on FDB entries in the bridge */
    if ((strncmp(argv[0], "session_per_fdb", strlen("session_per_fdb")) == 0))
    {
        /* Validate the input */
        if(argc != 2)
            return count;

        if(argv[1] == NULL || strlen(argv[1]) <= 0)
            return count;

        /* Based on whether the value set is 1/0 - enable / disable this feature. */
        session_per_fdb = (int)simple_strtol(argv[1], NULL, 0);
        if(session_per_fdb != 0 && session_per_fdb != 1)
        {
            session_per_fdb = 0;
            return count;
        }
    }

    return count;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_health_timer_expired
 **************************************************************************
 * DESCRIPTION   :
 *  The function is the health timer expiration routine which is called by
 *  periodically by the HOST to verify the sanity of the PDSP.
 **************************************************************************/
static void ti_hil_health_timer_expired (unsigned long data)
{
    /* Use the PPM API to determine the health of the PDSP. */
    if (ti_ppm_health_check() < 0)
    {
        printk ("------- FATAL Error: Packet Processor PDSP is not healthy\n");
        return;
    }

    /* The PDSP passed the health check; restart the timer */
    pdsp_health_timer.expires = jiffies + 5*HZ;
    add_timer (&pdsp_health_timer);
    return;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_static_init
 **************************************************************************
 * DESCRIPTION   :
 *  Initialization function for the Static profile.
 *
 * RETURNS       :
 *  0   -   Success
 *  <0  -   Error
 **************************************************************************/
static int ti_hil_static_init (void)
{
    struct proc_dir_entry*	ptr_dir_entry;

    /* Register an event handler to listen to events. */
    ppsubsystem_event_handler = ti_ppm_register_event_handler (ti_hil_ppsubsystem_event_handler);
    if (ppsubsystem_event_handler == 0)
    {
        printk ("Error: Event Handler register failed\n");
        return -1;
    }

    /* Create the PROC Entry used by the TCA Configuration Engine. */
    ptr_dir_entry = create_proc_entry("net/ti_pp" ,0644, NULL);
    if (ptr_dir_entry == NULL)
	{
        printk ("Error: Unable to create Packet Processor proc entry.\n");
    	return -1;
    }
    ptr_dir_entry->data 	  = NULL;
	ptr_dir_entry->read_proc  = NULL;
    ptr_dir_entry->write_proc = ti_hil_write_cmds;
	ptr_dir_entry->owner 	  = THIS_MODULE;

    /* Create a timer to poll for the health */
    init_timer (&pdsp_health_timer);
    pdsp_health_timer.function = ti_hil_health_timer_expired;
    pdsp_health_timer.data     = 0;

    /* Start the timer. */
    pdsp_health_timer.expires = jiffies + 5*HZ;
    add_timer (&pdsp_health_timer);

    /* Profile has been succesfully initialized. */
	return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_static_deinit
 **************************************************************************
 * DESCRIPTION   :
 *  Deinitialization function which deinitializes and unregisters the default
 *  profile with the HIL Core.
 *
 * RETURNS       :
 *  0   -   Success
 *  <0  -   Error
 **************************************************************************/
static int ti_hil_static_deinit(void)
{
    return 0;
}
