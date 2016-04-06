/*
 * hil_intrusive.c - HIL Intrusive Mode Profile
 *
 * Description:
 *  The file contains the implementation of the HIL Intrusive Mode Profile.
 *  The profile use the following kernel features
 *   - CONFIG_TI_DEVICE_PROTOCOL_HANDLING
 *   - CONFIG_TI_EGRESS_HOOK
 *  The profile install hooks into the Ingress and Egress Data Path and
 *  populates the session structure which is stored in the SKB for the LUT
 *  and Modification record configuration. These hooks are installed on
 *  networking devices which have an PID or VPID handle associated with it.
 *
 *  The profile is provided as is and can be used as a template for
 *  development of more system specific profiles.
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
#include <linux/in.h>
#include <linux/if_ether.h>
#include "../../8021q/vlan.h"
#include <linux/if_pppox.h>
#include <linux/ip.h>
#include <linux/ipv6.h>
#include <linux/tcp.h>
#include <linux/udp.h>
#include <linux/ppp_defs.h>
#include <asm-arm/arch-avalanche/generic/pal_cppi41.h>
#include <linux/ti_hil.h>
#include <linux/mroute.h>
#include <net/ip.h>
#include <net/ipv6.h>
#include "linux/ti_ppm.h"
#include "../../bridge/br_private.h"
#include <linux/proc_fs.h>

#include <asm-arm/arch-avalanche/generic/pal.h>
#include "linux/ti_pp_path.h"

#include <mach/puma.h>
#include <mach/hardware.h>

#include <asm-arm/arch-avalanche/generic/ti_ppd.h>

#ifndef TI_MAX_DEVICE_INDEX
#define TI_MAX_DEVICE_INDEX 64
#endif




#ifdef CONFIG_NETFILTER
#include <linux/netfilter.h>
#include <net/netfilter/nf_conntrack.h>
#include <net/netfilter/nf_conntrack_helper.h>
#include <linux/netfilter_ipv4/ip_tables.h>
#endif

#if (CONFIG_MACH_PUMA6)
#include "linux/cat_l2switch_netdev.h"
#endif

/**************************************************************************
 ************************************ Local Definitions *******************
 **************************************************************************/

/* Definitons required for the HIL Analysis. */
#define HIL_SESSION_STAT_BUCKET           64
#define HIL_MAX_NUM_BUCKETS               TI_PP_MAX_ACCLERABLE_SESSIONS / HIL_SESSION_STAT_BUCKET

/* PDSP Health Timer; to periodically check if the PDSP are executing properly or not. */
#define PDSP_HEALTH_TIMER                 60

#define DUMMY_FOR_TUNNEL_1                1
#define DUMMY_FOR_TUNNEL_0                2

/**************************************************************************
 *************************** Static Definitions ***************************
 **************************************************************************/

/* Debug Dumping Functions */
static void ti_hil_intrusive_display_l2         (TI_PP_ETH_DESC*    ptr_eth_desc);
static void ti_hil_intrusive_display_ipv4       (TI_PP_IPV4_DESC*   ptr_ipv4_lut_entry);
static void ti_hil_intrusive_display_ipv6       (TI_PP_IPV6_DESC*   ptr_ipv6_lut_entry);
static const char* ti_hil_intrusive_display_ipv6_addr(const void *cp, char *buf, size_t len);
static void ti_hil_intrusive_display_session    (int session_handle);


/* Profile Initialization/Deinitialization and networking event handlers. */
static int ti_hil_intrusive_init (void);
static int ti_hil_netsubsystem_event_handler(unsigned int module_id, unsigned long event_id, void* ptr);
static int ti_hil_intrusive_deinit(void);

#ifdef CONFIG_IP_MULTICAST
static int hil_mfc_delete_session_by_mc_group(int mc_group);
static int hil_mfc_check_entry (unsigned int mcast_group);
static int hil_mfc_add_entry (unsigned int mcast_group, unsigned char session_handle);
static int hil_mfc_del_entry (unsigned int mcast_group);
static int hil_mfc_to_session(struct pp_mr_param* ptr_mr_param, TI_PP_SESSION* ptrsession);
#endif /* CONFIG_IP_MULTICAST */

#ifdef CONFIG_INTEL_PP_TUNNEL_SUPPORT
int ti_hil_delete_tunnel(void);
#endif

#ifdef HIL_EXAMPLE_DO_NOT_DELETE
void ti_hil_test_gre_session_creation(void);
#endif

/**************************************************************************
 ********************************* Globals ********************************
 **************************************************************************/

/* Default Profile. */
TI_HIL_PROFILE      hil_intrusive_profile = {
    .name            = "intrusive",
    .profile_handler = ti_hil_netsubsystem_event_handler,
    .profile_init    = ti_hil_intrusive_init,
    .profile_deinit  = ti_hil_intrusive_deinit,
};

/* Packet Processor Subsystem Event Handler. */
unsigned int        ppsubsystem_event_handler;

/* PP PDSP Health Timer to verify if the PP is running correctly. */
struct timer_list   pdsp_health_timer;

#ifdef CONFIG_NETFILTER
/* This is a global data structure which maps the session handle to corresponding
 * connection tracking entries. */
struct nf_conn* hil_session_ct_mapper [TI_PP_MAX_ACCLERABLE_SESSIONS];

#endif /* CONFIG_NETFILTER */

#ifdef CONFIG_IP_MULTICAST
unsigned int hil_mfc_session_mapper[TI_PP_MAX_ACCLERABLE_SESSIONS];
#endif /* CONFIG_IP_MULTICAST */

/* This is the default session IDLE timeout in microseconds. */
int ti_session_timeout = 30 * HZ * 1000;

#define DOCSIS_FW_PACKET_API_TCP_HIGH_PRIORITY    (1 << 10)

/* cable_pp: disable/enable capability */
#define MAX_RTT_THRESHOLD    (5*1000)
struct
{
    int                 hil_disabled;
    int                 dbg_disabled;
    int                 tdox_disabled;
    int                 qos_disabled;
    int                 tdox_RTT_threshold_ms;
#ifdef CONFIG_INTEL_PP_TUNNEL_SUPPORT
    int                 tunnelMode;
#endif

    unsigned int        num_bypassed_pkts;
    unsigned int        num_other_pkts;
    unsigned int        num_ingress_pkts;
    unsigned int        num_egress_pkts;
    unsigned int        num_null_drop_pkts;

/* Counters for keeping track of stats for the HIL Analysis. */
    unsigned int        num_total_sessions;
    unsigned int        num_error;
    unsigned int        session_bucket[HIL_MAX_NUM_BUCKETS];

}
global_ti_hil_db =
{
#ifdef CONFIG_INTEL_PP_TUNNEL_SUPPORT
    .hil_disabled       =   1,
#else
    .hil_disabled       =   0,
#endif
    .dbg_disabled       =   1,
    .tdox_disabled      =   0,
    .qos_disabled       =   0,
    .tdox_RTT_threshold_ms  =   50, // To be fine tuned (!). Currently 20 msec threshold
#ifdef CONFIG_INTEL_PP_TUNNEL_SUPPORT
    .tunnelMode         =   1,
#endif
    .num_bypassed_pkts  =   0,
    .num_other_pkts     =   0,
    .num_ingress_pkts   =   0,
    .num_egress_pkts    =   0,
    .num_null_drop_pkts =   0,
    .num_total_sessions =   0,
    .num_error          =   0
};

#ifdef CONFIG_INTEL_PP_TUNNEL_SUPPORT
static int gTunnel0Handle = -1;
static int gTunnel1Handle = -1;
#endif

/* Packet terminating VPID handle */
static int  docsis_null_vpid_handle = -1;
static int  netfilter_null_vpid_handle = -1;

/* get IPv6 traffic class */
#define IPV6TCLASS(h) (((h->priority&0x0F)<<4) | ((h->flow_lbl[0]&0xF0)>>4))

/************************************************************************/
/*                                                                      */
/*                                                                      */
/*     TDOX stuff                                                       */
/*                                                                      */
/*                                                                      */
/************************************************************************/
#define TI_HIL_TDOX_MAX_SESSIONS        16

typedef struct tdox_db_entry_t
{
    struct tdox_db_entry_t *    next;
    int                         pp_session_handle;
    int                         tdox_session_handle;
    int                         createTime;
    int                         estimated_RTT_ms;
}
tdox_db_entry_t;

static tdox_db_entry_t  tdox_db_repository[TI_HIL_TDOX_MAX_SESSIONS];

typedef struct
{
    struct tdox_db_entry_t *    activeList;
    int                         activeNum;
    struct tdox_db_entry_t *    freeList;
    int                         freeNum;
}
tdox_db_ctrl_t;

static tdox_db_ctrl_t   tdox_db;
/************************************************************************/


/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/

extern int ti_deregister_egress_hook_handler (struct net_device* dev);
extern int ti_register_egress_hook_handler (struct net_device* dev, int (*egress_hook)(struct sk_buff *skb));

#ifdef CONFIG_TI_PACKET_PROCESSOR_STATS
/* DOCSIS Packet processor start session notification Callback */
extern TI_HIL_START_SESSION ti_hil_start_session_notification_cb;
/* DOCSIS Packet processor delete session notification Callback */
extern TI_HIL_DELETE_SESSION ti_hil_delete_session_notification_cb;
#endif /* CONFIG_TI_PACKET_PROCESSOR_STATS */

/**************************************************************************
 ******************************* Functions  *******************************
 **************************************************************************/

#define TDOX_LOCK(context)
#define TDOX_UNLOCK(context)

/**************************************************************************
 * FUNCTION NAME : ti_hil_tdox_alloc_session
 **************************************************************************
 * DESCRIPTION   :
 *  The function allocates TDOX session handle.
 *
 * RETURNS:
 *   0  -  Session is allocated.
 *  -1  -  There are no TDOX IDs available at the time.
 **************************************************************************/
static int ti_hil_tdox_alloc_session (void)
{
    struct tdox_db_entry_t * tmp;

    TDOX_LOCK(os_context);

    if (0 == tdox_db.freeNum)
    {
        TDOX_UNLOCK(os_context);
        return -1;
    }

    tdox_db.freeNum--;

    tmp =                   tdox_db.freeList;
    tdox_db.freeList =      tdox_db.freeList->next;
    tmp->next =             tdox_db.activeList;
    tdox_db.activeList =    tmp;
    tdox_db.activeList->createTime = (int)AVALANCHE_PERF_MON_COUNTER_L_GET();


    tdox_db.activeNum++;

    TDOX_UNLOCK(os_context);

#ifdef CONFIG_TI_HIL_DEBUG
#ifdef TDOX_DEBUG
    if (0 == global_ti_hil_db.dbg_disabled)
    {
        printk("\n==== TDOX: Allocate Session %d ====\n",tdox_db.activeList->tdox_session_handle);
    }
#endif
#endif

    return (tdox_db.activeList->tdox_session_handle);
}


/**************************************************************************
 * FUNCTION NAME : ti_hil_tdox_associate_session
 **************************************************************************
 * DESCRIPTION   :
 *  The function associates PP session to TDOX session handle.
 *
 * RETURNS:
 *   0  -  Session is associated.
 *  -1  -  Session parameters are invalid.
 **************************************************************************/
static int ti_hil_tdox_associate_session (unsigned char tdox_handle, int pp_session_handle)
{
    if (tdox_handle >= TI_HIL_TDOX_MAX_SESSIONS)
    {
        return -1;
    }

    if ((pp_session_handle < 0) || (pp_session_handle >=TI_PP_MAX_ACCLERABLE_SESSIONS))
    {
        return -1;
    }

#ifdef CONFIG_TI_HIL_DEBUG
#ifdef TDOX_DEBUG
    if (0 == global_ti_hil_db.dbg_disabled)
    {
        printk("\n==== TDOX: Associate tdox Session %d = pp %d ====\n", tdox_handle, pp_session_handle);
    }
#endif
#endif

    tdox_db_repository[tdox_handle].pp_session_handle = pp_session_handle;
    return 0;
}


/**************************************************************************
 * FUNCTION NAME : ti_hil_tdox_free_session
 **************************************************************************
 * DESCRIPTION   :
 *  The function releases TDOX ID.
 *  The search can be done either per PP or TDOX session handle key.
 *  In case one of the search keys is unknown, the -1 should be specified.
 *
 * RETURNS:
 *   0  -  Session is freed.
 *  -1  -  Session is NOT found.
 **************************************************************************/
static int ti_hil_tdox_free_session (int pp_session_handle, int tdox_session_handle)
{
    tdox_db_entry_t *   curr;
    tdox_db_entry_t *   prev;
    tdox_db_entry_t *   tmp;

    TDOX_LOCK(os_context);

    if (0 == tdox_db.activeNum)
    {
        TDOX_UNLOCK(os_context);
        return -1;
    }

#ifdef CONFIG_TI_HIL_DEBUG
#ifdef TDOX_DEBUG
    if (0 == global_ti_hil_db.dbg_disabled)
    {
        printk("\n==== TDOX: Free Session [%d] START     ====\n",pp_session_handle);
    }
#endif
#endif

    curr = tdox_db.activeList;
    prev = NULL;

    do
    {
        if (
            ((curr->pp_session_handle != -1)   && (curr->pp_session_handle == pp_session_handle)) ||
            ((curr->tdox_session_handle != -1) && (curr->tdox_session_handle == tdox_session_handle))
           )
        {
            tdox_db.activeNum--;

            if (prev)
            {
                prev->next =         curr->next;
            }
            else
            {
                tdox_db.activeList = curr->next;
            }

            tmp =                    tdox_db.freeList;
            tdox_db.freeList =       curr;
            tdox_db.freeList->next = tmp;
            tdox_db.freeList->pp_session_handle = -1;

            tdox_db.freeNum++;
            TDOX_UNLOCK(os_context);

#ifdef CONFIG_TI_HIL_DEBUG
#ifdef TDOX_DEBUG
            if (0 == global_ti_hil_db.dbg_disabled)
            {
                printk("\n==== TDOX: Free Session [%d] OK        ====\n",pp_session_handle);
            }
#endif
#endif
            return 0;
        }

        prev = curr;
        curr = curr->next;
    }
    while (NULL != curr);

    TDOX_UNLOCK(os_context);

#ifdef CONFIG_TI_HIL_DEBUG
#ifdef TDOX_DEBUG
    if (0 == global_ti_hil_db.dbg_disabled)
    {
        printk("\n==== TDOX: Free Session [%d] Not Found ====\n",pp_session_handle);
    }
#endif
#endif

    return -1;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_tdox_get_session_entry
 **************************************************************************
 * DESCRIPTION   :
 *  The function performs search for specified PP session ID in TDOX repository.
 *  In case it finds it returns the pointer to the entry.
 *
 * RETURNS:
 *  pointer to the entry.
 *  NULL -  Session is NOT found.
 **************************************************************************/
static tdox_db_entry_t * ti_hil_tdox_get_session_entry(int pp_session_handle)
{
    int index;

    for (index=0; index<TI_HIL_TDOX_MAX_SESSIONS; index++ )
    {
        if (tdox_db_repository[index].pp_session_handle == pp_session_handle)
        {
            return &(tdox_db_repository[index]);
        }
    }

    return NULL;
}


/**************************************************************************
 * FUNCTION NAME : ti_hil_tdox_init
 **************************************************************************
 * DESCRIPTION   :
 *  The initialization function of TDOX driver internal database.
 *
 * RETURNS       :
 *  0   -   Success
 **************************************************************************/
static int ti_hil_tdox_init (void)
{
    int                 idx;
    tdox_db_entry_t *   prev = NULL;

    for (idx=0; idx < TI_HIL_TDOX_MAX_SESSIONS; idx++)
    {
        tdox_db_repository[idx].tdox_session_handle = idx;
        tdox_db_repository[idx].pp_session_handle = -1;
        tdox_db_repository[idx].next = prev;
        prev = &tdox_db_repository[idx];
    }

    tdox_db.freeList    = prev;
    tdox_db.freeNum     = TI_HIL_TDOX_MAX_SESSIONS;
    tdox_db.activeList  = NULL;
    tdox_db.activeNum   = 0;

    return 0;
}


/**************************************************************************
 * FUNCTION NAME : ti_hil_tdox_print
 **************************************************************************
 * DESCRIPTION   :
 *  Utility function prints out the TDOX driver internal database.
 *
 * RETURNS       :
 *  0   -   Success
 **************************************************************************/
static int ti_hil_tdox_print (void)
{
    unsigned char       idx;

    printk ("\n==== TDOX DB ====\n");

    printk ("  Free [%08X], num [%d]\n", (unsigned int)tdox_db.freeList, tdox_db.freeNum);
    printk ("  Act  [%08X], num [%d]\n", (unsigned int)tdox_db.activeList, tdox_db.activeNum);

    printk ("\n==== TDOX RECORDS ====\n");
    printk ("+-----+------+------------+------------+------+---------------+\n");
    printk ("| idx | sess |    addr    |    next    |  PP  | Estimated RTT |\n");
    printk ("+-----+------+------------+------------+------+---------------+\n");

    for (idx=0; idx < TI_HIL_TDOX_MAX_SESSIONS; idx++)
    {
        printk("| %3d | %4d | 0x%08X | 0x%08X | %4d | %10d ms |\n",
                    idx,
                    tdox_db_repository[idx].tdox_session_handle,
                    (unsigned int)&tdox_db_repository[idx],
                    (unsigned int)tdox_db_repository[idx].next,
                    tdox_db_repository[idx].pp_session_handle,
                    tdox_db_repository[idx].estimated_RTT_ms
                    );
    }
    printk ("+-----+------+------------+------------+------+---------------+\n");

    return 0;
}


#define TI_HIL_TDOX_ENABLED             TI_PP_SESSION_APP_RAW_INFO1_B3_VALID
#define TI_HIL_TDOX_SKIP_TIMESTAMP      TI_PP_SESSION_APP_RAW_INFO1_B3_PLUS_VALID
#define TI_HIL_TCP_SYN                  TI_PP_SESSION_APP_HIL_USED01

/************************************************************************/
/*  TCP Options Numbers                                                 */
/************************************************************************/
#define TCP_OPTION_CODE_EOL         0  // 0  End of Option List                     [RFC793]
#define TCP_OPTION_CODE_NOP         1  // 1  No-Operation                           [RFC793]
                                       // 2  Maximum Segment Size                   [RFC793]
                                       // 3  WSOPT - Window Scale                   [RFC1323]
                                       // 4  SACK Permitted                         [RFC2018]
                                       // 5  SACK                                   [RFC2018]
                                       // 6  Echo (obsoleted by option 8)           [RFC1072]
                                       // 7  Echo Reply (obsoleted by option 8)     [RFC1072]
#define TCP_OPTION_CODE_TIMESTAMP   8  // 8  TSOPT - Time Stamp Option              [RFC1323]
                                       // 9  Partial Order Connection Permitted     [RFC1693]
                                       // 10 Partial Order Service Profile          [RFC1693]
                                       // 11 CC                                     [RFC1644]
                                       // 12 CC.NEW                                 [RFC1644]
                                       // 13 CC.ECHO                                [RFC1644]
                                       // 14 TCP Alternate Checksum Request         [RFC1146]
                                       // 15 TCP Alternate Checksum Data            [RFC1146]
                                       // 16 Skeeter                                [Knowles]
                                       // 17 Bubba                                  [Knowles]
                                       // 18 Trailer Checksum Option                [Subbu & Monroe]
                                       // 19 MD5 Signature Option                   [RFC2385]
                                       // 20 SCPS Capabilities                      [Scott]
                                       // 21 Selective Negative Acknowledgements    [Scott]
                                       // 22 Record Boundaries                      [Scott]
                                       // 23 Corruption experienced                 [Scott]
                                       // 24 SNAP                                   [Sukonnik]
                                       // 25 Unassigned (released 2000-12-18)
                                       // 26 TCP Compression Filter                 [Bellovin]
                                       // 27 Quick-Start Response                   [RFC4782]
                                       // 28 User Timeout Option                    [RFC-ietf-tcpm-tcp-uto-11.txt]
/************************************************************************/


#if 0 /* TODO: Enable once router functionality is available */

/**************************************************************************
 * FUNCTION NAME : ti_hil_is_device_routed
 **************************************************************************
 * DESCRIPTION   :
 *  Utility function checks if the device is attached to the IP stack.
 *
 * RETURNS       :
 *  1   -   Device is attached to the IP stack.
 *  0   -   Device is not attached to the IP stack
 **************************************************************************/
static int ti_hil_is_device_routed (struct net_device* dev)
{
    struct in_device *in_dev = in_dev_get(dev);

    if ((in_dev == NULL) || (in_dev->ifa_list == NULL))
        return 0;

    return 1;
}

#endif

/**************************************************************************
 * FUNCTION NAME : ti_hil_ipv4_checksum
 **************************************************************************
 * DESCRIPTION   :
 *  Utility function to calculate IPv4 checksum field
 *
 * RETURNS       :
 *  ... the checksum
 **************************************************************************/
Uint16 ti_hil_ipv4_checksum(Uint8 *ipv4Header, u16 ipv4HeaderLen)
{
    Uint32 checksum;
    Uint16 i;
    
    for (i = 0, checksum = 0; i < ipv4HeaderLen ; i += 2)
    {
        checksum += (Uint32)((ipv4Header[i] << 8) & 0xFF00) + (ipv4Header[i+1] & 0xFF);
    }
    
    while (checksum & 0xFFFF0000)
    {
        checksum = (checksum >> 16) + (checksum & 0xFFFF);
    }

    return ((Uint16)~checksum);
}


/**************************************************************************
 * FUNCTION NAME : ti_hil_extract_packet_l2
 **************************************************************************
 * DESCRIPTION   :
 *  extracts HIL relevant fields from the L2 header 
 *
 * RETURNS       :
 *  0 for success
 **************************************************************************/
inline static int ti_hil_extract_packet_l2(char* ptr_data, struct ethhdr** ptr_ethhdr, struct iphdr** ptr_iphdr, struct ipv6hdr** ptr_ipv6hdr, unsigned short *protocol_type, TI_PP_SESSION_PROPERTY* ptr_ses_property)
{
    /* Get the pointer to the Ethernet header */
    *ptr_ethhdr = (struct ethhdr *)ptr_data;

    /* Skip the Ethernet header. */
    ptr_data = ptr_data + sizeof(struct ethhdr);

    /* Get the protocol type. */
    *protocol_type = (*ptr_ethhdr)->h_proto;

    /* Check the protocol field. If the protocol is VLAN or not?  */
    if (*protocol_type == __constant_htons(ETH_P_8021Q))
    {
        /* Get the VLAN header. */
        struct vlan_hdr* ptr_vlanheader = (struct vlan_hdr *)ptr_data;

        /* The new protocol is encapsulated into the VLAN header. */
        *protocol_type = ptr_vlanheader->h_vlan_encapsulated_proto;

        ptr_ses_property->l2_packet.u.eth_desc.enables  |= TI_PP_SESSION_L2_VLAN_VALID;
        ptr_ses_property->l2_packet.u.eth_desc.vlan_tag  = ptr_vlanheader->h_vlan_TCI;

        /* Skip the 4 bytes VLAN header. We have already accounted for the Ethernet header. */
        ptr_data = ptr_data + sizeof(struct vlan_hdr);
    }
    else
    {
        ptr_ses_property->l2_packet.u.eth_desc.vlan_tag  = 0;
    }

    /* We have skipped the layer2 information; so try and get the layer3 information. */
    switch (*protocol_type)
    {
        case __constant_htons(ETH_P_IP):
        {
            /* Get the pointer to the IPv4 Header. */
            *ptr_iphdr = (struct iphdr *)ptr_data;
            break;
        }
        case __constant_htons(ETH_P_IPV6):
        {
            /* Get the pointer to the IPv6 Header. */
            *ptr_ipv6hdr = (struct ipv6hdr *)ptr_data;
            break;
        }
        case __constant_htons(ETH_P_PPP_DISC):
        case __constant_htons(ETH_P_PPP_SES):
        {
            /* PPP Packet: Skip the PPP header. */
            ptr_data = ptr_data + 6;

            /* Get the PPP Protocol Information*/
            *protocol_type = *((unsigned short *)ptr_data);
            if (*protocol_type == __constant_htons(PPP_IP))
            {
                /* IP Packet. */
                *ptr_iphdr = (struct iphdr *)(ptr_data + 2);
            }
            break;
        }
        default:
        {
            /* This is a default condition to handle any packet type which is not understood. These packets are currently not accelerated by the PP. Example ARP Packets etc. */
            return -1;
        }
    }

    return 0;
}
/**************************************************************************
 * FUNCTION NAME : ti_hil_TurboDox_Check_And_Enable
 **************************************************************************
 * DESCRIPTION   : This function is called only for TCP protocol.
 * It set the Tdox Enable Flag If set of condition are filled. 
 * RETURNS:
 *  0 = OK, -1 = error.
 **************************************************************************/
int ti_hil_TurboDox_Check_And_Enable(struct tcphdr* ptr_tcphdr,char *  ptr_data_tail,TI_PP_SESSION_PROPERTY* ptr_ses_property)
{
           
    if (ptr_tcphdr->syn)
    {
      ptr_ses_property->app_specific_data.u.app_desc.enables |= TI_HIL_TCP_SYN;
    }
    else if (!global_ti_hil_db.tdox_disabled) /* if TDOX is enabled */
    {
      /* extract ack number */
      if (ptr_tcphdr->ack)
      {
          int     tcp_opt_timestamp_found = 0;
          char *  ptr_data_head   = (char *)ptr_tcphdr + ptr_tcphdr->doff*4;
          char *  ptr_tcp_options = (char *)ptr_tcphdr + sizeof(struct tcphdr);

          if (ptr_data_head > ptr_tcp_options)
          {
          /********************************************************/
          /* Go over the options and look for timestamp option    */
          /********************************************************/
            while (ptr_tcp_options < ptr_data_head)
            {
                char tcp_opt_type;
                char tcp_opt_len;

                tcp_opt_type = *(ptr_tcp_options);

                if (tcp_opt_type > TCP_OPTION_CODE_NOP)
                {
                    /************************************************/
                    /* T.L.V. styled option                         */
                    /************************************************/
                    tcp_opt_len =  *(ptr_tcp_options + 1);

                    if (TCP_OPTION_CODE_TIMESTAMP == tcp_opt_type)
                    {
                        tcp_opt_timestamp_found = 1;
                    }

                    if(tcp_opt_len != 0)
                    {
                        ptr_tcp_options += tcp_opt_len;
                    }
                    else
                    {
                        /* an illegal option length - don't accelerate this session */
                        /* Probably the server will reset the connection */
                        return -1;
                    }
                }
                else
                {
                    /************************************************/
                    /* Options EOL and NOP are without parameters   */
                    /************************************************/
                    ptr_tcp_options++;
                }
             }
             /********************************************************/
           }
           /************************************************/
           /* Options pointer ponts to the end of data -   */
           /* means NO data present in this packet.        */
           /* This packet is a potential TURBO candidate.  */
           /************************************************/
           if ((ptr_data_tail - ptr_tcp_options) < 200)
           {
               ptr_ses_property->app_specific_data.u.app_desc.raw_app_info2 = ptr_tcphdr->ack_seq;
               ptr_ses_property->app_specific_data.u.app_desc.enables |= TI_HIL_TDOX_ENABLED;
               if (tcp_opt_timestamp_found)
               {
                   ptr_ses_property->app_specific_data.u.app_desc.enables |= TI_HIL_TDOX_SKIP_TIMESTAMP;
               }
            }
            /************************************************/
           }
           else
           {
                /**************************************************/
                /* TCP with NO ACK flag set - do not accelerate   */
                /**************************************************/
                return -1;
            }
           
         
     }
    return 0;
}
/**************************************************************************
 * FUNCTION NAME : ti_hil_extract_packet_desc
 **************************************************************************
 * DESCRIPTION   :
 *  The function is called to extract the various Layer2, Layer3 and
 *  Layer4 fields and populate the packet description structure.
 *
 * RETURNS       :
 *  0   -   Success
 *  <0  -   Error
 **************************************************************************/
static int ti_hil_extract_packet_desc(char* ptr_data, struct net_device* dev, TI_PP_SESSION_PROPERTY* ptr_ses_property, Bool is_ingress)
{
    struct ethhdr*      ptr_ethhdr = NULL;
    struct iphdr*       ptr_iphdr = NULL;
    struct iphdr*       ptr_dsLiteIphdr = NULL;
    struct tcphdr*      ptr_tcphdr = NULL;
    TI_PP_PACKET_DESC*  ptr_pkt_desc;
    unsigned short      protocol_type;

    /* IPv6 params */
    struct ipv6hdr* ptr_ipv6hdr = NULL;
    struct ipv6_opt_hdr* hdr = NULL;
    unsigned char offset = sizeof(struct ipv6hdr);
    unsigned char nexthdr;
    unsigned int nextOffset;
    unsigned int hdrlen;

    /* Check if the device is a PID or not? */
    if (dev->pid_handle != -1)
    {
        /* Valid PID Handle: Packet has been received at the lowest level; at
         * this point  in time we can extract all the L2/L3/L4 information from
         * the packet. */

        if( !ptr_data )
        {
            /* Without this data we cannot continue */
            return -1;
        }

        if (ti_hil_extract_packet_l2(ptr_data, &ptr_ethhdr, &ptr_iphdr, &ptr_ipv6hdr, &protocol_type, ptr_ses_property) != 0)
        {
            return -1;
        }

        /* Step4: Extract the Layer 4 information only if the packet was an IP Packet. */
        if (ptr_iphdr != NULL)
        {
            if (is_ingress && ptr_iphdr->protocol == IPPROTO_GRE)
            {
                /* Check if we support this type of DS GRE by looking at the GRE protocol type field */
                ptr_data = (Uint8*)ptr_iphdr + (ptr_iphdr->ihl * 4);    /* Go to GRE header */
                if (*(Uint32*)ptr_data != ETH_P_TEB)  /* 0x6558 - Transparent Ethernet Bridging */
                {
                    /* unsupported GRE */
                    return -1;
                }
                
                /* Support DS GRE */
                ptr_ses_property->l2_packet.u.eth_desc.enables |= TI_PP_SESSION_L2_GRE_DS_VALID;

                /* Need to extract the internal L2/L3 packet */
                ptr_data += 4; /* Skip GRE header */
                if (ti_hil_extract_packet_l2(ptr_data, &ptr_ethhdr, &ptr_iphdr, &ptr_ipv6hdr, &protocol_type, ptr_ses_property) != 0)
                {
                    return -1;
                }
            }

            /* Currently we support only TCP & UDP. For both these protocols the PORT
             * information lies at the same location. */
            if ((ptr_iphdr->protocol == IPPROTO_TCP) || (ptr_iphdr->protocol == IPPROTO_UDP))
            {
                /* In case of IP Fragmentation - the first fragment has an offset of zero
                If this is the first fragment - the TCP/UDP header exists.
                The flags field uses one bit as the "more fragments" bit.
                This bit is turned on for each fragment comprising a datagram except the final fragment.
                frag_off field includes 3 bit flags and bit fragmant offset - Check if frag offset fiels set to zero
                this packet can be excelrated. Note that fragment offset field is 13 bits wide and we must mask the rest of the bits */
                if ( ( ptr_iphdr->frag_off & ( IP_OFFSET | IP_MF ) ) == 0 )
                {
                    ptr_tcphdr = (struct tcphdr *)((char *)ptr_iphdr + ptr_iphdr->ihl*4);
                }
                else
                {
                    return -1;
                }
            }
        }

        /* Step4: Extract the Layer 4 information only if the packet was an IPv6 Packet. */
        if (ptr_ipv6hdr != NULL)
        {
            nexthdr = ptr_ipv6hdr->nexthdr;

            /* Currently we support only TCP & UDP. For both these protocols the PORT
             * information lies at the same location. */

            /* Iterate through well-known extenstion headers till we reach the TCP/UDP header */
            while ((nexthdr != IPPROTO_TCP) && (nexthdr != IPPROTO_UDP) && (nexthdr != IPPROTO_IPIP))
            {
                /* If this is the last next header */
                if (nexthdr == NEXTHDR_NONE)
                {
                    return -1;
                }

                /* Encrypted header - cannot parse it, treat as uknown header */
                if (nexthdr == NEXTHDR_ESP)
                {
                    return -1;
                }

                hdr = (struct ipv6_opt_hdr*)((unsigned char *)ptr_ipv6hdr + offset);

                /* Calculate the extension header length */
                /* RFC 2460: Each extension header is an integer multiple of 8 octets long, in
                   order to retain 8-octet alignment for subsequent headers.  Multi-
                   octet fields within each extension header are aligned on their
                   natural boundaries, i.e., fields of width n octets are placed at an
                   integer multiple of n octets from the start of the header, for n = 1,
                   2, 4, or 8.
                */
                if (nexthdr == NEXTHDR_FRAGMENT)
                {
                    hdrlen = 8;
                }
                else if (nexthdr == NEXTHDR_AUTH)
                {
                    hdrlen = (hdr->hdrlen + 2) << 2;
                }
                else
                {
                    hdrlen = ipv6_optlen(hdr);
                }

                nextOffset = (unsigned int)offset + hdrlen;

                /* preventing a wrap - if the offset exceeds 255 bytes we exit the loop. */
                if (nextOffset > 0xFF)
                {
                    return -1;
                }
                else
                {
                    offset = (unsigned char)nextOffset;
                }

                nexthdr = hdr->nexthdr;
            }

            if (nexthdr == IPPROTO_IPIP)
            {
                ptr_dsLiteIphdr = (struct iphdr *)((unsigned char *)ptr_ipv6hdr + offset);
                if (is_ingress == TRUE)
                {
                    /* DSLite DS */
                    if ((ptr_dsLiteIphdr->protocol == IPPROTO_TCP) || (ptr_dsLiteIphdr->protocol == IPPROTO_UDP))
                    {
                        /* We take only non fragmented packets or first fragment since only these packets holds the layer 4 */
                        if ( ( ptr_dsLiteIphdr->frag_off & IP_OFFSET ) == 0 )
                        {
                            ptr_tcphdr = (struct tcphdr *)((unsigned char *)ptr_dsLiteIphdr + ptr_dsLiteIphdr->ihl * 4);
                        }
                        else
                        {
                            return -1;
                        }
                    }
                    else
                    {
                        /* Currently we support only TCP & UDP */
                        return -1;
                    }
                    
                }
                else
                {
                    /* DSLite US - Layer 4 is neglected */
                    ptr_tcphdr = (struct tcphdr *)((unsigned char *)ptr_dsLiteIphdr + ptr_dsLiteIphdr->ihl * 4);
                }
            }
            else
            {
                ptr_tcphdr = (struct tcphdr *)((unsigned char *)ptr_ipv6hdr + offset);
            }
        }

        if (ptr_ses_property->l2_packet.u.eth_desc.enables & TI_PP_SESSION_L2_GRE_DS_VALID)
        {
            /* DS GRE - L4 is neglected */
            ptr_tcphdr = NULL;
        }

        /* At this stage all the headers have been located and are pointing at the correct
         * locations. Time to start populating the Packet Properties.
         *
         * Step 1: Start with the Layer2 Header. */
        ptr_pkt_desc = &ptr_ses_property->l2_packet;

        /* Check did we have an Ethernet header. */
        if (ptr_ethhdr != NULL)
        {
            /* YES. Ethernet header was detected. Initialize the various fields. */
            ptr_pkt_desc->packet_type = TI_PP_ETH_TYPE;

            /* Populate the destination MAC address. */
            memcpy(ptr_pkt_desc->u.eth_desc.dstmac, (void *)&ptr_ethhdr->h_dest, 6);
            ptr_pkt_desc->u.eth_desc.enables  |= TI_PP_SESSION_L2_DSTMAC_VALID;

            /* Populate the source MAC address. */
            memcpy(ptr_pkt_desc->u.eth_desc.srcmac, (void *)&ptr_ethhdr->h_source, 6);
            ptr_pkt_desc->u.eth_desc.enables  |= TI_PP_SESSION_L2_SRCMAC_VALID;
        }
        else
        {
            /* No Ethernet header was present. Reset all the fields in the packet descriptor */
            memset ((void *)ptr_pkt_desc, 0, sizeof(TI_PP_PACKET_DESC));
        }

        /* Step 2: Start with the Layer3 header. */
        ptr_pkt_desc = &ptr_ses_property->l3l4_packet;

        /*  Check if an IPv4 header was detected or not? */
        if (ptr_iphdr != NULL)
        {
            if (!is_ingress && ptr_iphdr->protocol == IPPROTO_GRE)
            {
                __sum16 savedCheck;
                __be16	savedTotLen;
                __be16	savedId;
                Bool    err = false;

                /* Check if we support this type of US GRE by looking at the GRE protocol type field */
                ptr_data = (Uint8*)ptr_iphdr + (ptr_iphdr->ihl * 4);    /* Go to GRE header */
                if (*(Uint32*)ptr_data != ETH_P_TEB)  /* 0x6558 - Transparent Ethernet Bridging */
                {
                    /* unsupported GRE */
                    return -1;
                }

                /* Supported US GRE */
                ptr_ses_property->l2_packet.u.eth_desc.enables |= TI_PP_SESSION_L2_GRE_US_VALID;
                
                /* Now need to calculate checksum with payload length=0 and identification is incremented by =0x100
                   (we start with Identification high number so that host and PP packets will not have same Identification at session start) */
                /* Save the origianl values */
                savedCheck = ptr_iphdr->check;
                savedTotLen = ptr_iphdr->tot_len;
                savedId = ptr_iphdr->id;
                /* Change them to allow better PP utilization */
                ptr_iphdr->check = 0;
                ptr_iphdr->tot_len = 0;
                ptr_iphdr->id = 0xFFF;
                ptr_iphdr->check = ti_hil_ipv4_checksum((Uint8*)ptr_iphdr, ptr_iphdr->ihl * 4);
                /* Setup ipv4HdrRaw parameters */
                ptr_pkt_desc->u.ipv4_desc.ipv4HdrRawOffset = (Uint8*)ptr_iphdr - (Uint8*)ptr_ethhdr;
                ptr_pkt_desc->u.ipv4_desc.ipv4HdrRawLen = (ptr_iphdr->ihl * 4) + 4;
                if (ptr_pkt_desc->u.ipv4_desc.ipv4HdrRawLen <= TI_PP_IPV4_HEADER_RAW_SIZE_MAX)
                {
                    memcpy(ptr_pkt_desc->u.ipv4_desc.ipv4HdrRaw, ptr_iphdr, ptr_pkt_desc->u.ipv4_desc.ipv4HdrRawLen); // Save both IPv4 header and GRE header. Will be used as template
                }
                else
                {
                    /* There is not enough space to save the US GRE Encapsulation Header */
                    err = true;
                }
                /* Restore the origianl values */
                ptr_iphdr->check = savedCheck;
                ptr_iphdr->tot_len = savedTotLen;
                ptr_iphdr->id = savedId;
                if (err)
                {
                    return -1;
                }
            }

            /* Yes. IPv4 header was detected. Initialize the various fields. */
            ptr_pkt_desc->packet_type = TI_PP_IPV4_TYPE;

            /* Populate the Destination IP Address. */
            ptr_pkt_desc->u.ipv4_desc.dst_ip = ptr_iphdr->daddr;
            ptr_pkt_desc->u.ipv4_desc.enables  |= TI_PP_SESSION_IPV4_DSTIP_VALID;

            /* Populate the Source IP Address. */
            ptr_pkt_desc->u.ipv4_desc.src_ip = ptr_iphdr->saddr;
            ptr_pkt_desc->u.ipv4_desc.enables  |= TI_PP_SESSION_IPV4_SRCIP_VALID;

            /* Populate the TOS Byte */
            ptr_pkt_desc->u.ipv4_desc.tos = ptr_iphdr->tos;
            ptr_pkt_desc->u.ipv4_desc.enables  |= TI_PP_SESSION_IPV4_TOS_VALID;

            /* Populate the Protocol */
            ptr_pkt_desc->u.ipv4_desc.protocol = ptr_iphdr->protocol;
            ptr_pkt_desc->u.ipv4_desc.enables  |= TI_PP_SESSION_IPV4_PROTOCOL_VALID;

            /* Step3: Check if a Layer4 header was detected or not? */
            if (ptr_tcphdr != NULL)
            {
                /* YES. Layer4 header was present. Populate the Dst Port */
                ptr_pkt_desc->u.ipv4_desc.dst_port = ptr_tcphdr->dest;
                ptr_pkt_desc->u.ipv4_desc.enables |= TI_PP_SESSION_IPV4_DST_PORT_VALID;

                /* Populate the Source Port. */
                ptr_pkt_desc->u.ipv4_desc.src_port = ptr_tcphdr->source;
                ptr_pkt_desc->u.ipv4_desc.enables |= TI_PP_SESSION_IPV4_SRC_PORT_VALID;
                if(ptr_iphdr->protocol == IPPROTO_TCP)/* if this is TCP check if to enable Tdox */
                {
                    char *  ptr_data_tail   = (char *)ptr_iphdr  + ptr_iphdr->tot_len;
                    if(ti_hil_TurboDox_Check_And_Enable (ptr_tcphdr,ptr_data_tail ,ptr_ses_property) < 0)
                    {
                        return -1;
                    }
                }                
            }
        }
        /*  Check if an IPv6 header was detected or not? */
        else if (ptr_ipv6hdr != NULL)
        {
            /* Yes. IPv6 header was detected. Initialize the various fields. */
            ptr_pkt_desc->packet_type = TI_PP_IPV6_TYPE;

            if (ptr_dsLiteIphdr != NULL)
            {
                if (is_ingress == TRUE)
                {
                    // We would like to have the dsLite_dst_ip only for the ingress, to be able to classify according to it
                    ptr_pkt_desc->u.ipv6_desc.dsLite_dst_ip = ptr_dsLiteIphdr->daddr;
                    ptr_pkt_desc->u.ipv6_desc.enables  |= TI_PP_SESSION_IPV6_DSLITE_DSTIP_VALID;
                }
                else
                {
                    if (offset > TI_PP_IPV6_HEADER_RAW_SIZE_MAX)
                    {
                        offset = TI_PP_IPV6_HEADER_RAW_SIZE_MAX;
                    }

                    // We would like to have the whole IPv6 header only for US DsLite
                    ptr_pkt_desc->u.ipv6_desc.ipv6HdrTotalSize = offset;
                    memcpy(ptr_pkt_desc->u.ipv6_desc.ipv6HdrRaw, ptr_ipv6hdr, offset);
                    ((struct ipv6hdr*)ptr_pkt_desc->u.ipv6_desc.ipv6HdrRaw)->payload_len = 0; // reset the payload length value as it is irrelevant for the session creation and the DsLite template
                }
            }

            /* Populate the Destination IP Address. */
            memcpy(ptr_pkt_desc->u.ipv6_desc.dst_ip, ptr_ipv6hdr->daddr.s6_addr32, sizeof(ptr_pkt_desc->u.ipv6_desc.dst_ip));
            ptr_pkt_desc->u.ipv6_desc.enables  |= TI_PP_SESSION_IPV6_DSTIP_VALID;

            /* Populate the Source IP Address. */
            memcpy(ptr_pkt_desc->u.ipv6_desc.src_ip, ptr_ipv6hdr->saddr.s6_addr32, sizeof(ptr_pkt_desc->u.ipv6_desc.src_ip));
            ptr_pkt_desc->u.ipv6_desc.enables  |= TI_PP_SESSION_IPV6_SRCIP_VALID;

            /* Populate the Traffic Class Byte */
            ptr_pkt_desc->u.ipv6_desc.traffic_class = IPV6TCLASS(ptr_ipv6hdr);
            ptr_pkt_desc->u.ipv6_desc.enables  |= TI_PP_SESSION_IPV6_TRCLASS_VALID;

            /* Populate the Next Header */
            ptr_pkt_desc->u.ipv6_desc.next_header = ptr_ipv6hdr->nexthdr;
            ptr_pkt_desc->u.ipv6_desc.enables  |= TI_PP_SESSION_IPV6_NEXTHDR_VALID;

            /* Step3: Check if a Layer4 header was detected or not? */
            if (ptr_tcphdr != NULL)
            {
				//if not DsLite US
                if((ptr_dsLiteIphdr == NULL) || ((ptr_dsLiteIphdr != NULL) && (is_ingress == TRUE)))
                {
                    /* YES. Layer4 header was present. Populate the Dst Port */
                    ptr_pkt_desc->u.ipv6_desc.dst_port = ptr_tcphdr->dest;
                    ptr_pkt_desc->u.ipv6_desc.enables |= TI_PP_SESSION_IPV6_DST_PORT_VALID;

                    /* Populate the Source Port. */
                    ptr_pkt_desc->u.ipv6_desc.src_port = ptr_tcphdr->source;
                    ptr_pkt_desc->u.ipv6_desc.enables |= TI_PP_SESSION_IPV6_SRC_PORT_VALID; 
                }


                if (ptr_dsLiteIphdr != NULL)/* DSLite */
                {
                    if((ptr_dsLiteIphdr->protocol == IPPROTO_TCP) && (is_ingress == FALSE))/* if this is TCP check if to enable Tdox */
                    {
                        char *ptr_data_tail = (char *)ptr_dsLiteIphdr + ptr_dsLiteIphdr->tot_len;
                        if( ti_hil_TurboDox_Check_And_Enable (ptr_tcphdr, ptr_data_tail, ptr_ses_property)< 0)
                        {
                            return -1;
                        }
                    } 
                }
                else /* Ipv6 */
                {
                    if(ptr_ipv6hdr->nexthdr == IPPROTO_TCP)/* if this is TCP check if to enable Tdox */
                    {
                        char *ptr_data_tail = (char *)ptr_ipv6hdr + ptr_ipv6hdr->payload_len;
                        if( ti_hil_TurboDox_Check_And_Enable (ptr_tcphdr, ptr_data_tail, ptr_ses_property)< 0)
                        {
                            return -1;
                        }
                    }  
                }  
            }
        }
        else
        {
            /* No IPv4 or IPv6 header was present. Reset all the fields in the packet descriptor. */
            memset ((void *)ptr_pkt_desc, 0, sizeof(TI_PP_PACKET_DESC));
        }
    }

    /* Check if the device is a VPID handle or not? */
    if (dev->vpid_handle != -1)
    {
        /* The only missing information not available at the PID layer is the VPID handle
         * on which the packet was actually received/transmitted. */
        ptr_ses_property->vpid_handle = dev->vpid_handle;
    }

    /* All the fields have been extracted. */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_ingress_hook
 **************************************************************************
 * DESCRIPTION   :
 *  The function is registered as the device specific protocol handler for
 *  all networking devices which exist in the system which have a valid
 *  VPID. The function extracts the information pertinent to creation of
 *  the session interface and stores it in the SKB.
 *
 * RETURNS:
 *  Always returns 0.
 **************************************************************************/
int ti_hil_ingress_hook(struct sk_buff* skb)
{

    if (global_ti_hil_db.hil_disabled || (ti_pp_get_status() != ACTIVE))
    {
        return 0;
    }

    /* Extract all the fields from the packet and populate the Ingress Packet Descriptor. */
    if (ti_hil_extract_packet_desc((char *)skb_mac_header(skb), skb->dev, &skb->pp_packet_info.ti_session.ingress, TRUE) < 0)
    {
        return 0;
    }

    global_ti_hil_db.num_ingress_pkts++;

    /* Packet has passed through the Ingress Hooks. */
    skb->pp_packet_info.ti_pp_flags = skb->pp_packet_info.ti_pp_flags | TI_PPM_SESSION_INGRESS_RECORDED;
    skb->pp_packet_info.ti_session.priority = skb->ti_meta_info & 0x7;
    skb->pp_packet_info.ti_session.cluster = 0;

    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_session_intelligence
 **************************************************************************
 * DESCRIPTION   :
 *  The function is the host intelligence layer which decides if the Session
 *  is worthy of accleration or not?
 *
 * RETURNS:
 *  0   -  Session is NOT accelerated
 *  1   -  Session is accelerated
 **************************************************************************/
static int ti_hil_session_intelligence (TI_PP_SESSION* ptr_session)
{
    /* Acclerate only IPv4 or IPv6 traffic. */
    if ((ptr_session->ingress.l3l4_packet.packet_type & (TI_PP_IPV4_TYPE | TI_PP_IPV6_TYPE)) == 0)
        return 0;

    /* Dont accelerate MAC Broadcast Packets.
     *  - Check the Ingress Properties and ensure that the DST MAC Valid Bit is set.
     *  - Destination MAC Address is not a Broadcast */
    if ((ptr_session->ingress.l2_packet.u.eth_desc.enables & TI_PP_SESSION_L2_DSTMAC_VALID) &&
        (ptr_session->ingress.l2_packet.u.eth_desc.dstmac[0] == 0xFF))
        return 0;

    /* Accelerate only TCP and UDP. */
    /* IPv4 */
    if ((ptr_session->ingress.l3l4_packet.packet_type == TI_PP_IPV4_TYPE) &&
        (ptr_session->ingress.l3l4_packet.u.ipv4_desc.protocol != IPPROTO_UDP) &&
        (ptr_session->ingress.l3l4_packet.u.ipv4_desc.protocol != IPPROTO_TCP))
        return 0;

    /* IPv6 */
    else if ((ptr_session->ingress.l3l4_packet.packet_type == TI_PP_IPV6_TYPE) &&
        (ptr_session->ingress.l3l4_packet.u.ipv6_desc.next_header != IPPROTO_UDP) &&
        (ptr_session->ingress.l3l4_packet.u.ipv6_desc.next_header != IPPROTO_TCP) &&
        (ptr_session->ingress.l3l4_packet.u.ipv6_desc.next_header != IPPROTO_IPIP))
        return 0;

    /* Accelerate the session. */
    return 1;
}


/**************************************************************************
 * FUNCTION NAME : ti_hil_null_hook
 **************************************************************************
 * DESCRIPTION   :
 *  This function creates session to terminating VPID.
 *  All the packets like ingress one will be routed to it
 *  and dropped in the future.
 *  The function is very similar to ti_hil_egress_hook.
 * RETURNS       :
 *  Always returns 0.
 **************************************************************************/
int ti_hil_null_hook(struct sk_buff* skb, int null_vpid)
{
    TI_PP_SESSION*      ptr_session;
    int                 session_handle;
    struct net_device*    input_dev;

    if (global_ti_hil_db.hil_disabled || (ti_pp_get_status() != ACTIVE))
    {
        return 0;
    }

    /* These checks have the following purpose:-
     *  a) If the Packet has not HIT the ingress hook there is no point in creating the session
     *     since the packet is locally generated and these sessions cannot be acclerated.
     *  b) The Host Intelligence layers have decided not to "acclerate" this session. */
    if ((skb->pp_packet_info.ti_pp_flags & TI_PPM_SESSION_INGRESS_RECORDED) == 0)
    {
        global_ti_hil_db.num_other_pkts++;
        return 0;
    }

    if  (skb->pp_packet_info.ti_pp_flags & TI_PPM_SESSION_BYPASS)
    {
        global_ti_hil_db.num_bypassed_pkts++;
        return 0;
    }

    global_ti_hil_db.num_null_drop_pkts++;

    /* Get the pointer to the session information block. */
    ptr_session = &skb->pp_packet_info.ti_session;

    /* There can be only one egress record at this stage. */
    ptr_session->num_egress = 1;

#ifdef CONFIG_TI_META_DATA
    /* DOCSIS DSID is not supported yet... */
    ptr_session->ingress.app_specific_data.u.app_desc.enables = 0;
 /* ptr_session->ingress.app_specific_data.u.app_desc.u.raw_app_info1 = -1; */
#endif
    input_dev = dev_get_by_index(&init_net,skb->skb_iif);
    if(input_dev == NULL)
    {
         return 0;
    }
    /* Extract all the fields from the packet and populate the Egress Packet Descriptor. */
    if (ti_hil_extract_packet_desc ((char *)skb_mac_header(skb), input_dev, &ptr_session->egress[0], FALSE) < 0)
    {
        dev_put(input_dev);
        return 0;
    }

    /* We can only proceed to the next step if all the information has been extracted. This implies
     * that the Egress Hook has hit the Egress Hook on both the PID and VPID. */
    if (input_dev->pid_handle == -1)
    {
        dev_put(input_dev);
        return 0;
    }
    dev_put(input_dev);
    /* Override the VPID */
    ptr_session->egress[0].vpid_handle = null_vpid;

    /* Check if the session is ROUTABLE or not? One way of doing this is to compare the L2 destination
     * MAC Address at the Ingress and Egress and if they are not the same it is safe to assume that
     * the session was routed. There are other system wide optimizations that could be done here. For
     * example if the box is a layer2 bridge then this check is not required. Similarly if the box
     * operates only in ROUTED mode then the value can always be set. The check here is the most fail
     * proof as it handles conditions where both Bridging and Routing can coexist. */
    if (ptr_session->ingress.l2_packet.packet_type & TI_PP_ETH_TYPE)
    {
        /* OK. Ingress had recorded Layer2 properties. Does the Egress have the same */
        if (ptr_session->egress[0].l2_packet.packet_type & TI_PP_ETH_TYPE)
        {
            /* OK. Egress had also got recorded Layer2 properties. So compare the same. */
            if (memcmp ((void *)&ptr_session->ingress.l2_packet.u.eth_desc.dstmac,
                        (void *)&ptr_session->egress[0].l2_packet.u.eth_desc.dstmac, 6) != 0)
            {
                /* The destination MAC address are not the same; most definately a routed session. */
                ptr_session->is_routable_session = 1;
            }
            else
            {
                /* The destination MAC address are the same. This is bridged for sure. */
                ptr_session->is_routable_session = 0;
            }
        }
        else
        {
            /* No Egress information at layer2. We cannot take a decision now as all the information is not present.
             * Setting the flag is not correct because the framing in the PDSP will not work. So default to bridged */
            ptr_session->is_routable_session = 0;
        }
    }
    else
    {
        /* No Ingress information at layer2. We cannot take a decision now as all the information is not present.
         * Setting the flag is not correct because the framing in the PDSP will not work. So default to bridged */
        ptr_session->is_routable_session = 0;
    }

    /* Once all the fields have been extracted. Check if the session can be created or not? */
    if (ti_hil_session_intelligence (ptr_session) == 0)
        return 0;

    /* All sessions created here have a standard session timeout. */
    ptr_session->session_timeout = ti_session_timeout;
    ptr_session->priority = 0;
    ptr_session->cluster  = 0;

    /* Create the session in the Packet Processor. */
    session_handle = ti_ppm_create_session (ptr_session, (void*)skb, 0);
    if (session_handle < 0)
    {
        /* Session Creation Failed. Increment the error counter. */
        global_ti_hil_db.num_error++;
        return 0;
    }

#ifdef CONFIG_TI_HIL_DEBUG
    if (0 == global_ti_hil_db.dbg_disabled)
    {
        printk("\n ---- Session [%3d] has been created ---- Discarding ----\n", session_handle);
        ti_hil_intrusive_display_ipv4(&ptr_session->egress[0].l3l4_packet.u.ipv4_desc);
        printk(" ----------------------------------------\n");
    }
#endif

    return 0;
}


/**************************************************************************
 * FUNCTION NAME : ti_hil_egress_hook
 **************************************************************************
 * DESCRIPTION   :
 *  The function is the egress SRM hook that is called when a packet is to
 *  be transmitted on a session interfaces. The function extracts the
 *  information pertinent to creation of the session interface. It then
 *  checks if this packet was "routed/bridged". If yes then control is
 *  passed to the Plugin Logic to determine creation of the session.
 *
 * RETURNS       :
 *  Always returns 0.
 **************************************************************************/
int ti_hil_egress_hook(struct sk_buff* skb)
{
    TI_PP_SESSION*      ptr_session;
    int                 i;

    if (global_ti_hil_db.hil_disabled || (ti_pp_get_status() != ACTIVE))
    {
        return 0;
    }

    /* These checks have the following purpose:-
     *  a) If the Packet has not HIT the ingress hook there is no point in creating the session
     *     since the packet is locally generated and these sessions cannot be acclerated.
     *  b) The Host Intelligence layers have decided not to "acclerate" this session. */
    if ((skb->pp_packet_info.ti_pp_flags & TI_PPM_SESSION_INGRESS_RECORDED) == 0)
    {
        global_ti_hil_db.num_other_pkts++;
        return 0;
    }

    if  (skb->pp_packet_info.ti_pp_flags & TI_PPM_SESSION_BYPASS)
    {
        global_ti_hil_db.num_bypassed_pkts++;
        return 0;
    }

    global_ti_hil_db.num_egress_pkts++;

    /* Get the pointer to the session information block. */
    ptr_session = &skb->pp_packet_info.ti_session;

    /* There can be only one egress record at this stage. */
    ptr_session->num_egress = 1;

#ifdef CONFIG_TI_META_DATA
    for (i=0; i<ptr_session->num_egress; i++)
    {
        ptr_session->egress[i].app_specific_data.u.app_desc.u.raw_app_info1 = skb->ti_meta_info;
        ptr_session->egress[i].app_specific_data.u.app_desc.enables = TI_PP_SESSION_APP_RAW_INFO1_VALID;
    }

    /* DOCSIS DSID is not supported yet... */
    ptr_session->ingress.app_specific_data.u.app_desc.enables = 0;
 /* ptr_session->ingress.app_specific_data.u.app_desc.u.raw_app_info1 = -1; */
#endif

    /* Extract all the fields from the packet and populate the Egress Packet Descriptor. */
    if (ti_hil_extract_packet_desc ((char *)skb->data, skb->dev, &ptr_session->egress[0], FALSE) < 0)
    {
        return 0;
    }

    /* We can only proceed to the next step if all the information has been extracted. This implies
     * that the Egress Hook has hit the Egress Hook on both the PID and VPID. */
    if (skb->dev->pid_handle == -1)
        return 0;

    /* Check if the session is ROUTABLE or not? One way of doing this is to compare the L2 destination
     * MAC Address at the Ingress and Egress and if they are not the same it is safe to assume that
     * the session was routed. There are other system wide optimizations that could be done here. For
     * example if the box is a layer2 bridge then this check is not required. Similarly if the box
     * operates only in ROUTED mode then the value can always be set. The check here is the most fail
     * proof as it handles conditions where both Bridging and Routing can coexist. */
    if (ptr_session->ingress.l2_packet.packet_type & TI_PP_ETH_TYPE)
    {
        /* OK. Ingress had recorded Layer2 properties. Does the Egress have the same */
        if (ptr_session->egress[0].l2_packet.packet_type & TI_PP_ETH_TYPE)
        {
            /* OK. Egress had also got recorded Layer2 properties. So compare the same. */
            if (memcmp ((void *)&ptr_session->ingress.l2_packet.u.eth_desc.dstmac,
                        (void *)&ptr_session->egress[0].l2_packet.u.eth_desc.dstmac, 6) != 0)
            {
                /* The destination MAC address are not the same; most definately a routed session. */
                ptr_session->is_routable_session = 1;
            }
            else
            {
                /* The destination MAC address are the same. This is bridged for sure. */
                ptr_session->is_routable_session = 0;
            }
        }
        else
        {
            /* No Egress information at layer2. We cannot take a decision now as all the information is not present.
             * Setting the flag is not correct because the framing in the PDSP will not work. So default to bridged */
            ptr_session->is_routable_session = 0;
        }
    }
    else
    {
        /* No Ingress information at layer2. We cannot take a decision now as all the information is not present.
         * Setting the flag is not correct because the framing in the PDSP will not work. So default to bridged */
        ptr_session->is_routable_session = 0;
    }

    /* Once all the fields have been extracted. Check if the session can be created or not? */
    if (ti_hil_session_intelligence (ptr_session) == 0)
        return 0;

    /* All sessions created here have a standard session timeout. */
    ptr_session->session_timeout = ti_session_timeout;

    /************************************************************/
    /*                  CRITICAL SECTION START                  */
    /************************************************************/
    {
        unsigned int    lockKey;
        int             session_handle;
        int             tdox_ID = -1;

        PAL_osProtectEntry(PAL_OSPROTECT_INTERRUPT, &lockKey);

        /************************************************************************/
        /*  Scale down the priority of the egress queue                         */
        /************************************************************************/
        if ((skb->dev->vpid_block.qos_clusters_count) && (0 == global_ti_hil_db.qos_disabled))
        {
            if (NULL != skb->dev->qos_select_hook)
            {
                skb->dev->qos_select_hook(skb);
            }
            else
            {
                ptr_session->priority = 0;
            }
        }
        else
        {
            ptr_session->cluster  = 0xFF;
            ptr_session->priority = 0;
        }
        /************************************************************************/

        /* Populate the ack suppression - Tdox check and enable for DSLite */
        if (((ptr_session->egress[0].l3l4_packet.packet_type == TI_PP_IPV6_TYPE)         &&
            (ptr_session->egress[0].l3l4_packet.u.ipv6_desc.next_header == IPPROTO_IPIP) &&
            (ptr_session->ingress.l3l4_packet.packet_type == TI_PP_IPV4_TYPE)            &&
            (ptr_session->ingress.l3l4_packet.u.ipv4_desc.protocol == IPPROTO_TCP))      ||
        /* Populate the ack suppression - Tdox check and enable for IPv6 */
            ((ptr_session->egress[0].l3l4_packet.packet_type == TI_PP_IPV6_TYPE)         &&
            (ptr_session->egress[0].l3l4_packet.u.ipv6_desc.next_header == IPPROTO_TCP)) ||
        /* Populate the ack suppression - Tdox check and enable for IPv4 */
            ((ptr_session->egress[0].l3l4_packet.packet_type == TI_PP_IPV4_TYPE) &&
            (ptr_session->egress[0].l3l4_packet.u.ipv4_desc.protocol == IPPROTO_TCP)))
         {
            
            if (ptr_session->egress[0].app_specific_data.u.app_desc.enables & TI_HIL_TCP_SYN)
            {
                /* In case it's a SYN packet - do not open the session yet, but make it expedited in DOCSIS Upstream */
                skb->ti_meta_info |= DOCSIS_FW_PACKET_API_TCP_HIGH_PRIORITY;
                PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);
                return 0;
            }
            else
            {
                TI_PP_VPID vpid;
                TI_PP_PID   pid;

                ti_ppm_get_vpid_info (skb->dev->vpid_handle, &vpid);
                ti_ppm_get_pid_info  (vpid.parent_pid_handle, &pid);

                if (TI_PP_PID_TYPE_DOCSIS == pid.type)
                {
                  /************************************************************************/
                  /* DOCSIS Upstream flow case only.                                      */
                  /* The following code handles TURBO DOX feature case.                   */
                  /************************************************************************/
                    if (-1 == (session_handle = ti_ppm_check_session(ptr_session)))
                    {
                        if (ptr_session->egress[0].app_specific_data.u.app_desc.enables & TI_HIL_TDOX_ENABLED)
                        {
                            /* This is a first packet - make it expedited in DOCSIS Upstream in order to get response quickly */
                            skb->ti_meta_info |= DOCSIS_FW_PACKET_API_TCP_HIGH_PRIORITY;
                            tdox_ID = ti_hil_tdox_alloc_session();
                            ptr_session->egress[0].app_specific_data.u.app_desc.u.raw_app_info1_b3 = tdox_ID;
                            ptr_session->priority = 0;
                        }
                    }
                    else
                    {
                        tdox_db_entry_t *     tdox_Entry_p
                            = ti_hil_tdox_get_session_entry(session_handle);

                        if (tdox_Entry_p)
                        {
                            skb->ti_meta_info |= DOCSIS_FW_PACKET_API_TCP_HIGH_PRIORITY;
                            ptr_session->priority = 0;

                            if (tdox_Entry_p->createTime)
                            {
                                int delta = ( (int)AVALANCHE_PERF_MON_COUNTER_L_GET() - tdox_Entry_p->createTime );

                                delta /= 100000;  // Scale down the delta to milliseconds

                                tdox_Entry_p->estimated_RTT_ms = delta;

                                if (((delta              < global_ti_hil_db.tdox_RTT_threshold_ms) ||
                                     (MAX_RTT_THRESHOLD == global_ti_hil_db.tdox_RTT_threshold_ms)) &&
                                    (ptr_session->egress[0].app_specific_data.u.app_desc.enables & TI_HIL_TDOX_ENABLED))
                                {
                                    tdox_Entry_p->createTime = 0;
                                }
                                else
                                {
                                    ti_hil_tdox_free_session( session_handle, -1 );
                                    ptr_session->egress[0].app_specific_data.u.app_desc.enables &= ~TI_HIL_TDOX_ENABLED;
                                    ti_ppd_session_tdox_change( session_handle, ptr_session->egress[0].app_specific_data.u.app_desc.enables );
                                }
                            }

                            /* In case any of the packets coming to the host does not match the criteria for TDOX - we unset TDOX and move to low priority queue */
                            if (!(ptr_session->egress[0].app_specific_data.u.app_desc.enables & TI_HIL_TDOX_ENABLED))
                            {
                                ti_hil_tdox_free_session( session_handle, -1 );
                                ptr_session->egress[0].app_specific_data.u.app_desc.enables &= ~TI_HIL_TDOX_ENABLED;
                                ti_ppd_session_tdox_change( session_handle, ptr_session->egress[0].app_specific_data.u.app_desc.enables );
                            }
                        }
                    }
                  /************************************************************************/
                }
            }
        }

#ifdef CONFIG_TI_PACKET_PROCESSOR_EXT_SWITCH
       /************************************************************************/
        /*  The following is an application specific code sample that need      */
        /*  to be changed according to the external switch type and chosen      */
        /*  priority maintenance technique.                                     */
        /*  This case shows the sample of priority insertion using VLAN tagging */
        /*                                                                      */
        /*  Add VLAN tag with corresponding priority to the outgoing packet     */
        /*  Note that priority appears in 0(high) to 3(low) values range        */
        /************************************************************************/
#if (CONFIG_MACH_PUMA6)
        if (0 == strcmp(skb->dev->name, L2SW_NETDEV_DATA0))
#else    
        if (0 == strcmp(skb->dev->name, "eth0"))
#endif
        {
            if (ptr_session->egress[0].l2_packet.u.eth_desc.enables & TI_PP_SESSION_L2_VLAN_VALID)
            {
                PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);

                /*  The double encapsulation of VLAN tag is NOT currently supported. */
                return 0;
            }

            /* First, disable existing L2 header (assumes it was found earlier) */
            ptr_session->egress[0].l2_packet.packet_type        = TI_PP_L2_RAW_TYPE;
            ptr_session->egress[0].l2_packet.u.eth_desc.enables = 0;

            /* Second, enable raw L2 header to be inserted */
            ptr_session->egress[0].l2_raw_packet.packet_type    = TI_PP_L2_RAW_TYPE;
            ptr_session->egress[0].l2_raw_packet.u.l2raw_desc.enables = TI_PP_SESSION_L2_RAW_VALID;

            /* Third, build the L2 header that includes VLAN tag */
            {
                struct ethhdr*          ptr_ethhdr   = (struct ethhdr *)skb->data;
                struct vlan_ethhdr *    ptr_l2header = (struct vlan_ethhdr *)&ptr_session->egress[0].l2_raw_packet.u.l2raw_desc.tx_buff[0];

                memcpy( ptr_l2header->h_dest    ,ptr_ethhdr->h_dest,    ETH_ALEN );
                memcpy( ptr_l2header->h_source  ,ptr_ethhdr->h_source,  ETH_ALEN );
                ptr_l2header->h_vlan_proto = __constant_htons(ETH_P_8021Q);
                /* Insert the VLAN ID 1 together with the priority that came from DOCSIS DS (ti_meta_info) */
                ptr_l2header->h_vlan_TCI   = ( VLAN_PRIO_MASK & (((u16)(skb->ti_meta_info)) << VLAN_PRIO_SHIFT) ) | 0x01 ;

                ptr_l2header->h_vlan_encapsulated_proto = ptr_ethhdr->h_proto;
            }

            /* Finally update the length of newly built L2 header */
            ptr_session->egress[0].l2_raw_packet.u.l2raw_desc.tx_buff_len = VLAN_ETH_HLEN;
        }
        /************************************************************************/
#else
    #if (CONFIG_MACH_PUMA6)
        {
            TI_PP_VPID vpid;
            ti_ppm_get_vpid_info(skb->dev->vpid_handle, &vpid);

            /* Check if the seesion or the egress VPID has a VLAN, and if so, then update only the VLAN priority */
            if ((ptr_session->egress[0].l2_packet.u.eth_desc.enables & TI_PP_SESSION_L2_VLAN_VALID) || (vpid.type == TI_PP_VLAN))
            {
                ptr_session->egress[0].l2_packet.u.eth_desc.vlan_tag |= VLAN_PRIO_MASK & (((u16)(skb->ti_meta_info)) << VLAN_PRIO_SHIFT);
            }
            
        }
    #endif
#endif  // CONFIG_TI_PACKET_PROCESSOR_EXT_SWITCH


        /* Create the session in the Packet Processor. */
        session_handle = ti_ppm_create_session (ptr_session, (void*)skb, 0);

        if (session_handle < 0)
        {
            if (tdox_ID != -1)
            {
                ti_hil_tdox_free_session( -1, tdox_ID );
            }

            /* Session Creation Failed. Increment the error counter. */
            global_ti_hil_db.num_error++;
            PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);

            return 0;
        }

#ifdef CONFIG_TI_HIL_DEBUG
        if (0 == global_ti_hil_db.dbg_disabled)
        {
            printk("\n ---- Session [%3d] has been created ----\n", session_handle);
            ti_hil_intrusive_display_ipv4(&ptr_session->egress[0].l3l4_packet.u.ipv4_desc);
            printk(" ----------------------------------------\n");
        }
#endif

        if (tdox_ID != -1)
        {
            ti_hil_tdox_associate_session(tdox_ID, session_handle);
        }

#ifdef CONFIG_NETFILTER
        /* Once the session has been created; check if the packet had passed through the
         * connection tracking hooks and pass the session handle to that layer. */
        if (skb->nfct != NULL)
        {
            /* Session has passed through the connection tracking hooks.
             * Now get the connection tracking entry and set the hooks correctly. */

            struct nf_conn* conntrack  = (struct nf_conn *)skb->nfct;
            int*                 ct_session_handle;

            /* Check the direction of the connection tracking entry. */
            if (CTINFO2DIR(skb->nfctinfo) == IP_CT_DIR_REPLY)
                ct_session_handle = &conntrack->tuplehash[IP_CT_DIR_REPLY].ti_pp_session_handle;
            else
                ct_session_handle = &conntrack->tuplehash[IP_CT_DIR_ORIGINAL].ti_pp_session_handle;

            /* Check if the current session handle is valid or not? */
            if (!IS_TI_PP_SESSION_CT_INVALID(*ct_session_handle))
            {
                /* Handle was valid. Now we need to ensure that the current session handle matches the
                 * one we just created. */
                if (*ct_session_handle != session_handle)
                {
                    /* The existing session handle does not match the one we had.
                     *
                     * This should typically not happen because the PPM has an inbuilt duplicate session
                     * detection logic which will detect this and return the same session handle. (Example
                     * of this is the TCP Control Packets will have the same session handle passed to them
                     * at this stage so this code will never get executed)
                     *
                     * The fact that the control came here is that the PPM duplicate session detection logic
                     * failed or because there was something in the packet different which the conntrack
                     * did not care about. One of the known occurrences is the TOS Byte difference. Anyway
                     * in this case we need to handle the condition gracefully. So currently we ignore the
                     * new session handle and dont link it with the connection tracking entry.
                     *
                     * One more technique that can be used to solve this problem is to ignore the TOS byte in
                     * the LUT configuration. */
                    printk ("INFO --> Existing session %d new session %d.\n", *ct_session_handle, session_handle);

                    PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);

                    return 0;
                }
            }

            /* Remember the new session handle */
            *ct_session_handle = session_handle;

            /* Map the session handle and connection tracking entry together. */
            hil_session_ct_mapper[session_handle] = conntrack;
        }
#endif /* CONFIG_NETFILTER */

        PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);
    }
    /************************************************************/
    /*                  CRITICAL SECTION END                    */
    /************************************************************/

    return 0;
}

int ti_hil_set_mta_mac_address(unsigned char *mtaAddress)
{
    return ti_ppm_set_mta_mac_address(mtaAddress);
}

#ifdef CONFIG_INTEL_PP_TUNNEL_SUPPORT
/**************************************************************************
 * FUNCTION NAME : ti_hil_set_cm_mac_address
 **************************************************************************
 * DESCRIPTION   :
 *  Sets the CM MAC address in PP
 *
 * RETURNS       :
 *  0 = OK, other values = error
 **************************************************************************/
int ti_hil_set_cm_mac_address(unsigned char *cmAddress)
{
    return ti_ppm_set_cm_mac_address(cmAddress);
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_set_tunnel_mode
 **************************************************************************
 * DESCRIPTION   :
 *  Sets tunnelMode in PP
 *
 * RETURNS       :
 *  0 = OK, other values = error
 **************************************************************************/
int ti_hil_set_tunnel_mode(unsigned char tunnelMode)
{
    global_ti_hil_db.tunnelMode = tunnelMode;
    if (tunnelMode == 0)
    {
        ti_hil_delete_tunnel();
    }
    return ti_ppm_set_tunnel_mode(tunnelMode);
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_create_tunnel
 **************************************************************************
 * DESCRIPTION   :
 *  Creates a tunnel in PP
 *  In current implementation we have only one logical tunnel which is divided
 *  to two actual tunnels - one for US and one for DS. We will create both tunnels
 *  and then set tunnel mode (to be sure that PP will work with the configured tunnels)
 *
 * RETURNS       :
 *  0 = OK, other values = error
 **************************************************************************/
int ti_hil_create_tunnel(char *tunnelHeader, unsigned char tunnelHeaderLen, unsigned char l2L3HeaderLen,
                         TUNNEL_TYPE_E tunnelType, unsigned char udpMode)
{
    TI_PP_SESSION       *ptr_session;
    TI_PP_PACKET_DESC*  ptr_pkt_desc;
    TI_PP_VPID vpid;
    TI_PP_PID   pid;
    char cniVpid;
    char ethVpid;
    unsigned char tempVpid;
    struct sk_buff *skb;
    struct net_device *cniDev;
    struct net_device *ethDev;
    int tunnelConfig =
        (tunnelHeaderLen | (l2L3HeaderLen << 8) | (tunnelType << 16) | (udpMode << 24));
    char ipVersion;

    if (global_ti_hil_db.tunnelMode == 0)
    {
        printk("ti_hil_create_tunnel: Cannot create tunnels while not in tunnel mode\n");
        return -1;
    }

    /* Check if need to set IPv6 flag */
    if (tunnelHeader[12] == 0x86 && tunnelHeader[13] == 0xDD)
    {
        tunnelConfig |= (1 << 25);
    }

    printk("ti_hil_create_tunnel: tunnelHeaderLen=%d, l2L3HeaderLen=%d, tunnelType=%d, udpMode=%d\n",
            tunnelHeaderLen, l2L3HeaderLen, tunnelType, udpMode);

    cniVpid = -1;
    ethVpid = -1;
    for (tempVpid = 0; tempVpid < TI_PP_MAX_VPID; tempVpid++)
    {
        ti_ppm_get_vpid_info (tempVpid, &vpid);
        ti_ppm_get_pid_info  (vpid.parent_pid_handle, &pid);
        if (TI_PP_PID_TYPE_DOCSIS == pid.type)
        {
            cniVpid = tempVpid;
            if (ethVpid != 0xFF)
            {
                break;
            }
        }
        // TBD: when several ethernet VPID will be configured, need to know which is relevant
        if (TI_PP_PID_TYPE_ETHERNET == pid.type)
        {
            ethVpid = tempVpid;
            if (cniVpid != 0xFF)
            {
                break;
            }
        }
    }
    cniDev = __dev_get_by_name(&init_net, "cni0");
#if (CONFIG_MACH_PUMA6)
    ethDev = __dev_get_by_name(&init_net, L2SW_NETDEV_DATA0);
#else
    ethDev = __dev_get_by_name(&init_net, "eth0");
#endif

    if(!(skb = dev_alloc_skb(2048)))
    {
        printk("ti_hil_create_tunnel: Failed to allocate skb. Will not create a tunnel\n");
        return -1;
    }

    ptr_session = &skb->pp_packet_info.ti_session;


    /********************/
    /* WAN-->LAN tunnel */
    /********************/
    memset(&ptr_session->ingress, 0, sizeof(ptr_session->ingress));
    ptr_session->ingress.vpid_handle = cniVpid;
    ptr_session->num_egress = 1;
    ptr_session->egress[0].vpid_handle = ethVpid;

    ptr_session->is_routable_session = 0;
    ptr_session->session_timeout = tunnelConfig;

    ptr_session->egress[0].l2_packet.packet_type = TI_PP_L2_RAW_TYPE;
    ptr_pkt_desc = &ptr_session->egress[0].l2_raw_packet;
    ptr_pkt_desc->packet_type = TI_PP_L2_RAW_TYPE;
    ptr_pkt_desc->u.l2raw_desc.enables = TI_PP_SESSION_L2_RAW_VALID;

    ptr_session->ingress.l3l4_packet.packet_type = TI_PP_IPV4_TYPE;
    ptr_session->egress[0].l3l4_packet.packet_type = TI_PP_IPV4_TYPE;
    // The following two lines are done only for hash maneuver
    ptr_session->ingress.l2_packet.u.ipv4_desc.src_port = DUMMY_FOR_TUNNEL_1;
    ptr_session->ingress.l2_packet.u.ipv4_desc.enables = TI_PP_SESSION_IPV4_SRC_PORT_VALID;

    /************************************************************************/

    if(skb && cniDev)
    {
        skb->skb_iif = cniDev->ifindex;
    }


    skb->dev = ethDev;

    skb->pp_packet_info.ti_match_llc_filter = NULL;
    skb->pp_packet_info.ti_match_inbound_ip_filter = NULL;
    skb->pp_packet_info.ti_match_outbound_ip_filter = NULL;
    skb->pp_packet_info.ti_match_qos_classifier = NULL;

    /* Create the WAN-->LAN tunnel in the Packet Processor. */
    gTunnel1Handle = ti_ppm_create_session (ptr_session, (void*)skb, 1);
    if (gTunnel1Handle < 0)
    {
        printk("ti_hil_create_tunnel: ti_ppm_create_session failed\n");

        /* Session Creation Failed. Increment the error counter. */
        global_ti_hil_db.num_error++;
        dev_kfree_skb_any(skb);
        return 0;
    }

    /********************/
    /* LAN-->WAN tunnel */
    /********************/
    ptr_session->ingress.vpid_handle = ethVpid;
    ptr_session->egress[0].vpid_handle = cniVpid;

#ifdef CONFIG_TI_META_DATA
    // Configured SF=0, PHS=0xFF for US FW
    ptr_session->egress[0].app_specific_data.u.app_desc.u.raw_app_info1 = 0x00FF0000;
    ptr_session->egress[0].app_specific_data.u.app_desc.enables = TI_PP_SESSION_APP_RAW_INFO1_VALID;
#endif

    ptr_session->egress[0].l2_packet.packet_type = TI_PP_L2_RAW_TYPE;
    ptr_pkt_desc = &ptr_session->egress[0].l2_raw_packet;
    ptr_pkt_desc->packet_type = TI_PP_L2_RAW_TYPE;

    memcpy(ptr_pkt_desc->u.l2raw_desc.tx_buff, tunnelHeader, tunnelHeaderLen);
    ptr_pkt_desc->u.l2raw_desc.tx_buff_len = tunnelHeaderLen;
    ptr_pkt_desc->u.l2raw_desc.enables = TI_PP_SESSION_L2_RAW_VALID;

    // The following two lines are done only for hash maneuver
    ptr_session->ingress.l2_packet.u.ipv4_desc.src_port = DUMMY_FOR_TUNNEL_0;
    ptr_session->ingress.l2_packet.u.ipv4_desc.enables = TI_PP_SESSION_IPV4_SRC_PORT_VALID;

    /* Create the LAN-->WAN tunnel in the Packet Processor. */

    skb->skb_iif = ethDev->ifindex;

    skb->dev = cniDev;

    skb->pp_packet_info.ti_match_llc_filter = NULL;
    skb->pp_packet_info.ti_match_inbound_ip_filter = NULL;
    skb->pp_packet_info.ti_match_outbound_ip_filter = NULL;
    skb->pp_packet_info.ti_match_qos_classifier = NULL;

    gTunnel0Handle = ti_ppm_create_session (ptr_session, (void*)skb, 1);
    if (gTunnel0Handle < 0)
    {
        printk("ti_hil_create_tunnel: ti_ppm_create_session failed\n");

        /* Session Creation Failed. Increment the error counter. */
        global_ti_hil_db.num_error++;
        dev_kfree_skb_any(skb);
        return 0;
    }

    dev_kfree_skb_any(skb);
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_delete_tunnel
 **************************************************************************
 * DESCRIPTION   :
 *  Deletes a tunnel from PP
 * In current implementation we have only one logical tunnel which is divided
 * to two actual tunnels - one for US and one for DS. We will delete both tunnels
 *
 * RETURNS       :
 *  0 = OK, other values = error
 **************************************************************************/
int ti_hil_delete_tunnel(void)
{
    printk("ti_hil_delete_tunnel gTunnel0Handle=%d, gTunnel1Handle=%d\n", gTunnel0Handle, gTunnel1Handle);
    if (gTunnel0Handle >= 0)
    {
        ti_ppm_delete_session(gTunnel0Handle, NULL);
        gTunnel0Handle = -1;
    }
    if (gTunnel1Handle >= 0)
    {
        ti_ppm_delete_session(gTunnel1Handle, NULL);
        gTunnel1Handle = -1;
    }

    return 0;
}
#endif


#if 0 /* TODO: Enable once router functionality is available */

/**************************************************************************
 * FUNCTION NAME : ti_hil_device_handler
 **************************************************************************
 * DESCRIPTION   :
 *  Default HIL Device Handler.
 *
 * RETURNS       :
 *  0   -   Success
 *  <0  -   Error
 **************************************************************************/
static int ti_hil_device_handler(unsigned long event_id, void* ptr)
{
    struct net_device *dev = (struct net_device *)ptr;

    /* Dont do anything for Loopback devices; since the interface cannot participate
     * in the packet processor. */
    if (dev->flags & IFF_LOOPBACK)
    {
        /* YES. In that case there is no PID or VPID associated with the device */
        dev->pid_handle = -1;
        dev->vpid_handle= -1;
        return 0;
    }

    /* Processing is based on the Event. */
    switch (event_id)
    {
        case NETDEV_UP:
        {
            /* Network device is going UP. */
            printk ("Device %s is going UP!\n", dev->name);

            /* VPID needs to be created only if either of the following conditions are met:-
             *  a) Device is connected to the bridge.
             *     The bridge runs its internal state machine as ports move from LISTENING,
             *     LEARNING to FORWARDING state. There are events generated from the state
             *     machine which should handle the VPID creation/deletion. Thus this check
             *     is not explicitly seen here.
             *  b) Device is connected to the IP stack i.e has an IP Address. */
            if (ti_hil_is_device_routed(dev) == 1)
            {
                /* Check if the device had a valid VPID handle. */
                if (dev->vpid_handle == -1)
                {
                    /* NO. The device needs to be created as a VPID in the packet processor. */
                    dev->vpid_handle = ti_ppm_create_vpid (&dev->vpid_block);
                    if (dev->vpid_handle < 0)
                    {
                        printk ("Error: Unable to create VPID %d PID %d Device %s\n", dev->vpid_handle, dev->pid_handle, dev->name);
                        return -1;
                    }
                    printk ("Succesfully created VPID %d PID %d Device %s\n",  dev->vpid_handle, dev->pid_handle, dev->name);
                }
            }

            /* Install an Ingress Hook on the networking device. This is done if either
             * of the following conditions are met:-
             *  (a) Device has valid PID Handle.
             *  (b) Device has valid VPID Handle. */
            if ((dev->vpid_handle != -1) || (dev->pid_handle != -1))
            {
                /* Install the Ingress Hook. */
                if (ti_register_protocol_handler (dev, ti_hil_ingress_hook) == 0)
                    printk ("Ingress Hook on PID/VPID: %d/%d Name: %s\n", dev->pid_handle, dev->vpid_handle, dev->name);

                /* Install the Egress Hook. */
                if (ti_register_egress_hook_handler(dev, ti_hil_egress_hook) == 0)
                    printk ("Egress Hook on PID/VPID: %d/%d Name: %s\n", dev->pid_handle, dev->vpid_handle, dev->name);
            }
            break;
        }
        case NETDEV_DOWN:
        {
            /* Network device is going DOWN. */
            printk ("Device %s with VPID: %d is going DOWN!\n", dev->name, dev->vpid_handle);

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

            /* Uninstall the Ingress Hook; since because if the device is no longer a VPID there is no
             * point to listen for packets on it. */
            ti_deregister_protocol_handler (dev);
            ti_deregister_egress_hook_handler (dev);
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
 *  Default HIL INET Handler.
 *
 * RETURNS       :
 *  0   -   Success
 *  <0  -   Error
 **************************************************************************/
static int ti_hil_inet_handler(unsigned long event_id, void* ptr)
{
    struct in_ifaddr *ifa = (struct in_ifaddr*)ptr;
    struct in_device *in_dev = ifa->ifa_dev;
    struct net_device *dev = in_dev->dev;

    /* Dont do anything for Loopback devices; since the interface cannot participate
     * in the packet processor. */
    if (dev->flags & IFF_LOOPBACK)
    {
        /* YES. In that case there is no PID or VPID associated with the device */
        dev->pid_handle = -1;
        dev->vpid_handle= -1;
        return 0;
    }

    /* Processing is based on the Event. */
    switch (event_id)
    {
        case NETDEV_UP:
        {
            /* Network device is going UP. */
            printk ("Device %s is going UP!\n", dev->name);

            /* VPID needs to be created only if either of the following conditions are met:-
             *  a) Device is connected to the bridge.
             *     The bridge runs its internal state machine as ports move from LISTENING,
             *     LEARNING to FORWARDING state. There are events generated from the state
             *     machine which should handle the VPID creation/deletion. Thus this check
             *     is not explicitly seen here.
             *  b) Device is connected to the IP stack i.e has an IP Address. */
            if (ti_hil_is_device_routed(dev) == 1)
            {
                /* Check if the device had a valid VPID handle. */
                if (dev->vpid_handle == -1)
                {
                    /* NO. The device needs to be created as a VPID in the packet processor. */
                    dev->vpid_handle = ti_ppm_create_vpid (&dev->vpid_block);
                    if (dev->vpid_handle < 0)
                    {
                        printk ("Error: Unable to create VPID %d PID %d Device %s\n", dev->vpid_handle, dev->pid_handle, dev->name);
                        return -1;
                    }
                    printk ("Succesfully created VPID %d PID %d Device %s\n",  dev->vpid_handle, dev->pid_handle, dev->name);
                }
            }

            /* Install an Ingress Hook on the networking device. This is done if either
             * of the following conditions are met:-
             *  (a) Device has valid PID Handle.
             *  (b) Device has valid VPID Handle. */
            if ((dev->vpid_handle != -1) || (dev->pid_handle != -1))
            {
                /* Install the Ingress Hook. */
                if (ti_register_protocol_handler (dev, ti_hil_ingress_hook) == 0)
                    printk ("Ingress Hook on PID/VPID: %d/%d Name: %s\n", dev->pid_handle, dev->vpid_handle, dev->name);

                /* Install the Egress Hook. */
                if (ti_register_egress_hook_handler(dev, ti_hil_egress_hook) == 0)
                    printk ("Egress Hook on PID/VPID: %d/%d Name: %s\n", dev->pid_handle, dev->vpid_handle, dev->name);
            }
            break;
        }
        case NETDEV_DOWN:
        {
            /* Network device is going DOWN. */
            printk ("Device %s with VPID: %d is going DOWN!\n", dev->name, dev->vpid_handle);

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

            /* Uninstall the Ingress Hook; since because if the device is no longer a VPID there is no
             * point to listen for packets on it. */
            ti_deregister_protocol_handler (dev);
            ti_deregister_egress_hook_handler (dev);
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

#endif

/**************************************************************************
 * FUNCTION NAME : ti_hil_pp_handler
 **************************************************************************
 * DESCRIPTION   :
 *  Default HIL PP Handler.
 *
 * RETURNS       :
 *  0   -   Success
 *  <0  -   Error
 **************************************************************************/
static int ti_hil_pp_handler(unsigned long event_id, void* ptr)
{
    struct net_device*           dev,*input_dev;
    struct net_bridge_fdb_entry* fdb;
    struct sk_buff*              skb;
#ifdef CONFIG_NETFILTER
    struct nf_conn*             conntrack;
#endif
#ifdef CONFIG_IP_MULTICAST
    struct mfc_cache*            ptr_mfc_cache;
#endif
#ifdef CONFIG_NETFILTER
    struct xt_table*            t;
#endif

    /* Process the events. */
    switch (event_id)
    {
        case TI_DOCSIS_FLTR_DISCARD_PKT:
        {
            unsigned int lockKey;

            PAL_osProtectEntry(PAL_OSPROTECT_INTERRUPT, &lockKey);

            skb = (struct sk_buff*) ptr;
            input_dev = dev_get_by_index(&init_net,skb->skb_iif);
            if ((docsis_null_vpid_handle == -1) && (input_dev && input_dev->vpid_handle != -1))
            {
                docsis_null_vpid_handle = ti_ppm_create_vpid(&input_dev->vpid_block);

                if (docsis_null_vpid_handle < 0)
                {
                    printk ("Error: Unable to create Null VPID %d PID %d Device %s\n", docsis_null_vpid_handle, 9, input_dev->name);
                }
                else
                {
#ifdef CONFIG_TI_HIL_DEBUG
                    if (0 == global_ti_hil_db.dbg_disabled)
                    {
                        printk ("Successfully created Null VPID %d PID %d Device %s\n",  docsis_null_vpid_handle, 9, input_dev->name);
                    }
#endif /* CONFIG_TI_HIL_DEBUG */

                    ti_ppm_set_vpid_flags( docsis_null_vpid_handle,
                                           TI_PP_VPID_FLG_TX_DISBL | TI_PP_VPID_FLG_RX_DISBL );
                }
            }

            dev_put(input_dev);

            ti_hil_null_hook(skb, docsis_null_vpid_handle);
            PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);

            break;
        }

        case TI_DOCSIS_FLTR_ADD:
        case TI_DOCSIS_FLTR_DEL:
        case TI_DOCSIS_FLTR_CHG:
        {
            /* Always flush ALL sessions since we never know the behavior of the filter.
             * It may discard or allow packets. In case filter is of "allow" kind it is
             * not different from any other regular session.
             * Thus it is required to flush everything. */
            ti_ppm_flush_sessions( -1 );
            break;
        }

        case TI_DOCSIS_CLASSIFY_ADD:
        case TI_DOCSIS_CLASSIFY_DEL:
        case TI_DOCSIS_CLASSIFY_CHG:
        case TI_DOCSIS_MCAST_DEL:
        {
            ti_ppm_flush_sessions(-1);
            break;
        }

        case TI_DOCSIS_SESSIONS_DEL:
        {
            Uint32 numSessions = ((Uint32*)ptr)[0];
            Uint32 *sessList = &(((Uint32*)ptr)[1]);
            Uint32 i;
            for (i = 0; i < numSessions; i++)
            {
                ti_ppm_delete_session(sessList[i], NULL);
            }

            break;
        }
            
        case TI_DOCSIS_DSID_CHG:
        {
            /* Flush existing sessions in case of any change in DSID configuration */
            ti_ppm_flush_sessions(-1);
            break;
        }

        case TI_BRIDGE_PORT_DELETE:
        {
            /* Event indicates that a device has been removed from the bridge. This event is generated
             * when the user executes the brctl delif command to remove a port from the bridge. */
            dev = (struct net_device*)ptr;

            /* Do not touch VPIDs that do not have any parent PID defined */
            if (dev->vpid_block.parent_pid_handle == (unsigned char)(-1))
            {
                break;
            }

            /* This implies that the device is no longer capable of pariticipating
             * in the networking and should be removed from the packet processor.
             * But before doing so check if the device was attached to the Packet
             * processor or not i.e. valid VPID handle. */
            if (dev->vpid_handle != -1)
            {
                /* Delete the VPID. */
                if (ti_ppm_delete_vpid (dev->vpid_handle) < 0)
                {
                    printk ("Error: Unable to delete VPID %d PID %d Device %s\n", dev->vpid_handle, dev->pid_handle, dev->name);
                    return -1;
                }
#ifdef CONFIG_TI_HIL_DEBUG
                if (0 == global_ti_hil_db.dbg_disabled)
                {
                    printk ("Succesfully deleted VPID %d PID %d Device %s\n", dev->vpid_handle, dev->pid_handle, dev->name);
                }
#endif /* CONFIG_TI_HIL_DEBUG */
                dev->vpid_handle = -1;
            }
            /* Uninstall the Ingress Hook; since because if the device is no longer a VPID there is no
             * point to listen for packets on it. */
            ti_deregister_protocol_handler (dev);
            ti_deregister_egress_hook_handler (dev);
            break;
        }
        case TI_PP_ADD_VPID:
            /* Event to add vpid, for example when GW add vlan to LSD */
        case TI_BRIDGE_PORT_FORWARD:
        {
            /* Event indicates that a port attached to the bridge has moved to the FORWARDING state.
             * The host bridge will only forward packets in this state. Thus this is the time we
             * create the VPID.*/
            dev = (struct net_device*)ptr;

            /* Do not create any VPIDs if there is no parent PID defined */
            if (dev->vpid_block.parent_pid_handle == (unsigned char)(-1))
            {
                break;
            }

            if (dev->vpid_handle == -1)
            {
                if (0 == global_ti_hil_db.qos_disabled)
                {
                    if (NULL != dev->qos_setup_hook)
                    {
                        dev->qos_setup_hook( dev );
                    }
                }

                /* Create the VPID. */
                dev->vpid_handle = ti_ppm_create_vpid (&dev->vpid_block);
                if (dev->vpid_handle < 0)
                {
                    printk ("Error: Unable to create VPID %d PID %d Device %s\n", dev->vpid_handle, dev->pid_handle, dev->name);
                    return -1;
                }

#ifdef CONFIG_TI_HIL_DEBUG
                if (0 == global_ti_hil_db.dbg_disabled)
                {
                    printk ("Succesfully created VPID %d PID %d Device %s\n", dev->vpid_handle, dev->pid_handle, dev->name);
                }
#endif /* CONFIG_TI_HIL_DEBUG */
            }
            /* Install an Ingress Hook on the networking device. This is done if either
             * of the following conditions are met:-
             *  (a) Device has valid PID Handle.
             *  (b) Device has valid VPID Handle. */
            if ((dev->vpid_handle != -1) || (dev->pid_handle != -1))
            {
                /* Install the Ingress Hook. */
                if (ti_register_protocol_handler (dev, ti_hil_ingress_hook) == 0)
                {
#ifdef CONFIG_TI_HIL_DEBUG
                    if (0 == global_ti_hil_db.dbg_disabled)
                    {
                        printk ("Ingress Hook on PID/VPID: %d/%d Name: %s\n", dev->pid_handle, dev->vpid_handle, dev->name);
                    }
#endif /* CONFIG_TI_HIL_DEBUG */
                }

                /* Install the Egress Hook. */
                if (ti_register_egress_hook_handler(dev, ti_hil_egress_hook) == 0)
                {
#ifdef CONFIG_TI_HIL_DEBUG
                    if (0 == global_ti_hil_db.dbg_disabled)
                    {
                        printk ("Egress  Hook on PID/VPID: %d/%d Name: %s\n", dev->pid_handle, dev->vpid_handle, dev->name);
                    }
#endif /* CONFIG_TI_HIL_DEBUG */
                }
            }
            break;
        }
        case TI_PP_REMOVE_VPID:
            /* Event to remove vpid, for example when GW add vlan to LSD */
        case TI_BRIDGE_PORT_DISABLED:
        {
            /* Event indicates that a port attached to the bridge has been moved to the disabled state.
             * This event if of concern only if the bridge is running the spanning tree protocol. The event
             * occurs if the spanning tree protocol detects a condition where the port is causing a loop and
             * disables the port. In such a scenario the VPID and thus all sessions related to the port need
             * to be removed from the packet processor. */
            dev = (struct net_device*)ptr;

            /* Do not touch VPIDs that do not have any parent PID defined */
            if (dev->vpid_block.parent_pid_handle == (unsigned char)(-1))
            {
                break;
            }

            /* If the device has a VPID handle; it needs to be removed from the packet processor. */
            if (dev->vpid_handle != -1)
            {
                /* Delete the VPID. */
                if (ti_ppm_delete_vpid (dev->vpid_handle) < 0)
                {
                    printk ("Error: Unable to delete VPID %d PID %d Device %s\n", dev->vpid_handle, dev->pid_handle, dev->name);
                    return -1;
                }

                {
                    TI_PP_PID ptr_pid;
                    ti_ppm_get_pid_info (dev->vpid_block.parent_pid_handle, &ptr_pid);
                    if (ptr_pid.type == TI_PP_PID_TYPE_ETHERNET)
                    {
                        Uint32 queueIndex;
                        Uint32 divertCommand;
                        Uint32 qmgr = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
                        PAL_Handle handle = PAL_cppi4Init (NULL,(Ptr)PAL_CPPI41_QUEUE_MGR_PARTITION_SR);

                        for (queueIndex = 0; queueIndex < 1; queueIndex++)
                        {
                            divertCommand =  (PAL_CPPI41_RECYCLE_INFRA_INPUT_LOW_Q_NUM << 16);     // Setup destination Queue
                            divertCommand += ptr_pid.tx_pri_q_map[queueIndex];  // Setup source Queue

                            PAL_cppi4Control( handle, PAL_CPPI41_IOCTL_QUEUE_DIVERT, (Ptr)divertCommand, &qmgr );
                        }

                        PAL_cppi4Exit(handle, NULL);
                    }
                }

#ifdef CONFIG_TI_HIL_DEBUG
                if (0 == global_ti_hil_db.dbg_disabled)
                {
                    printk ("Succesfully deleted VPID %d PID %d Device %s\n", dev->vpid_handle, dev->pid_handle, dev->name);
                }
#endif /* CONFIG_TI_HIL_DEBUG */
                dev->vpid_handle = -1;
            }
            /* Uninstall the Ingress Hook; since because if the device is no longer a VPID there is no
             * point to listen for packets on it. */
            ti_deregister_protocol_handler (dev);
            ti_deregister_egress_hook_handler (dev);
            break;
        }
        case TI_BRIDGE_FDB_CREATED:
        {
            /* Event indicates that an FDB entry is being created. This event is of concern if the single session
             * per interface mode is selected as an option for session creation. If this is not the case then this
             * event can be ignored. */
            fdb = (struct net_bridge_fdb_entry *)ptr;

#ifdef CONFIG_TI_HIL_DEBUG
            if (0 == global_ti_hil_db.dbg_disabled)
            {
                /* Debug Message: To indicate how to access all the information? */
                printk ("Intrusive -> Create Bridge Session MAC=%02x-%02x-%02x-%02x-%02x-%02x on %s VPID:%d\n",
                    fdb->addr.addr[0], fdb->addr.addr[1], fdb->addr.addr[2], fdb->addr.addr[3], fdb->addr.addr[4],
                    fdb->addr.addr[5], fdb->dst->dev->name, fdb->dst->dev->vpid_handle);
            }
#endif /* CONFIG_TI_HIL_DEBUG */
            break;
        }
        case TI_BRIDGE_FDB_DELETED:
        {
            /* Event indicates that an FDB entry is being deleted. The event is of concern if the single session
             * per interface mode is selected as an option for session creation. If this is not the case then this
             * event can be ignored. */
            fdb = (struct net_bridge_fdb_entry *)ptr;

#ifdef CONFIG_TI_HIL_DEBUG
            if (0 == global_ti_hil_db.dbg_disabled)
            {
                /* Debug Message: To indicate how to access all the information? */
                printk ("Intrusive -> Delete Session MAC=%02x-%02x-%02x-%02x-%02x-%02x on %s VPID:%d\n",
                    fdb->addr.addr[0], fdb->addr.addr[1], fdb->addr.addr[2], fdb->addr.addr[3], fdb->addr.addr[4],
                    fdb->addr.addr[5], fdb->dst->dev->name, fdb->dst->dev->vpid_handle);
            }
#endif /* CONFIG_TI_HIL_DEBUG */
            break;
        }
        case TI_BRIDGE_PACKET_FLOODED:
        {
            /* Event indicates that the packet will now be flooded onto all interfaces. This can happen in
             * any of the following cases:-
             *  a) Unicast packet but no matching FDB entry is found.
             *  b) Broadcast packet
             *  c) Multicast packet but no layer2 extensions eg IGMP snooping exists */
            skb = (struct sk_buff*) ptr;

#ifdef CONFIG_TI_HIL_DEBUG
            if (0 == global_ti_hil_db.dbg_disabled)
            {
                printk ("Intrusive -> Packet %p is flooded on all interfaces\n", skb);
            }
#endif /* CONFIG_TI_HIL_DEBUG */

            /* In the intrusive mode profile these packets are not considered as candidates for acceleration.
             * So mark the packet BYPASS mode so that the egress hook is bypassed. */
            skb->pp_packet_info.ti_pp_flags |= TI_PPM_SESSION_BYPASS;
            break;
        }

        case TI_ROUTE_ADDED:
        {
            /*  Event indicates that a new route is being added. A route change
             *  could affect the existing sessions in the PP. So, the HIL profile
             *  could choose to delete all sessions that are affected by the route
             *  change / flush the entire session table to make sure that there
             *  is no inconsistency due to the route change.
             */
            break;
        }
        case TI_ROUTE_DELETED:
        {
            /*  Event indicates that an existing route is being deleted. A route change
             *  could affect the existing sessions in the PP. So, the HIL profile
             *  could choose to delete all sessions that are affected by the route
             *  change / flush the entire session table to make sure that there
             *  is no inconsistency due to the route change.
             */
            break;
        }

#ifdef CONFIG_NETFILTER
        case TI_CT_ENTRY_CREATED:
        {
            /* Event generated from the connection tracking layer to indicate that a connection tracking entry
             * has been created. This could be used by the system profile to analyze the connection and indicate
             * immediately if the connection is worthy of accleration or not? If the profile deems that the
             * connection can not be accelerated it sets the status flag in the connection tracking entry to BYPASS
             * mode. This will ensure that all packets matching the connection will also have the BYPASS mode
             * set. This can be used for performance optimizations as the Egress Hook and Session Intelligence
             * will be less burdened. */
            conntrack = (struct nf_conn *)ptr;

#ifdef CONFIG_TI_HIL_DEBUG
            if (0 == global_ti_hil_db.dbg_disabled)
            {
                printk ("Intrusive -> Connection Tracking Entry %p has been created\n", conntrack);
            }
#endif /* CONFIG_TI_HIL_DEBUG */

            /* Check if the conntrack was associated with an ALG? In our profile we dont want these sessions
             * to be accelerated so set the connection tracking entry to operate in BYPASS mode. */
            if ((nfct_help(conntrack)) != NULL)
            {
                conntrack->ti_pp_status_flag |= TI_PP_BYPASS;
            }

            break;
        }
        case TI_CT_DEATH_BY_TIMEOUT:
        {
            /* Event indicates that the connection tracking entry has timed out. Use this event to
             * determine if the connection tracking entry needs to be deleted or not? If the profile
             * wants to prevent the death of the connection tracking entry it should ensure that the
             * PP staus flag does NOT set the TI_PP_KILL_CONNTRACK bit and if so be the case the
             * connection tracking entry is now owned by the System Profile and it has the repsonsibility
             * of cleaning it.
             * This event needs to be handled only if CONFIG_NETFILTER is enabled else this event can
             * be safetly ignored. */
            conntrack = (struct nf_conn *)ptr;

#ifdef CONFIG_TI_HIL_DEBUG
            if (0 == global_ti_hil_db.dbg_disabled)
            {
                printk ("Intrusive -> Connection Tracking Entry %p has timed out.\n", conntrack);
            }
#endif /* CONFIG_TI_HIL_DEBUG */

            if (IS_TI_PP_SESSION_CT_INVALID(conntrack->tuplehash[IP_CT_DIR_ORIGINAL].ti_pp_session_handle) &&
                IS_TI_PP_SESSION_CT_INVALID(conntrack->tuplehash[IP_CT_DIR_REPLY   ].ti_pp_session_handle))
            {
                /* Neither of the flows in the connection tracking entry are being accelerated.
                 * This implies that we should just go ahead and delete the entry? */
                conntrack->ti_pp_status_flag |= TI_PP_KILL_CONNTRACK;
            }
            else
            {
                /* The flows are still being accelerated; so keep the connection tracking timer alive */
                conntrack->timeout.expires = ti_session_timeout + jiffies;
                add_timer(&conntrack->timeout);
            }
            break;
        }
        case TI_CT_NETFILTER_TABLE_UPDATE:
        {
            /* Get the netfilter table */
            t = (struct xt_table *)ptr;

#ifdef CONFIG_TI_HIL_DEBUG
            if (0 == global_ti_hil_db.dbg_disabled)
            {
                printk ("Intrusive -> Netfilter Table %s has been modified\n", t->name);
            }
#endif /* CONFIG_TI_HIL_DEBUG */

            /* Flush all sessions only for NAT... No need to do anything for Mangle and Firewall */
            if (strcmp (t->name, "nat") == 0 || strcmp (t->name, "filter") == 0)
            {
                if (ti_ppm_flush_sessions(-1) < 0 )
                {
                    printk ("Error: Unable to flush all sessions\n");
                    return 0;
                }
                printk ("NAT Table update all sessions flushed\n");
            }
            break;
        }

        case TI_CT_NETFILTER_DISCARD_PKT:
        {
            unsigned int lockKey;

            PAL_osProtectEntry(PAL_OSPROTECT_INTERRUPT, &lockKey);

            skb = (struct sk_buff*) ptr;

            if (!(skb->skb_iif))
            {
                PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);
                break;
            }
            input_dev = dev_get_by_index(&init_net,skb->skb_iif);
            if ((netfilter_null_vpid_handle == -1) && (input_dev && input_dev->vpid_handle != -1))
            {
                netfilter_null_vpid_handle = ti_ppm_create_vpid(&input_dev->vpid_block);

                if (netfilter_null_vpid_handle < 0)
                {
                    printk ("Error: Unable to create Null VPID %d Device %s\n", netfilter_null_vpid_handle, input_dev->name);
                }
                else
                {
#ifdef CONFIG_TI_HIL_DEBUG
                    if (0 == global_ti_hil_db.dbg_disabled)
                    {
                        printk ("Successfully created Null VPID %d Device %s\n",  netfilter_null_vpid_handle, input_dev->name);
                    }
#endif /* CONFIG_TI_HIL_DEBUG */

                    ti_ppm_set_vpid_flags( netfilter_null_vpid_handle,
                        TI_PP_VPID_FLG_TX_DISBL | TI_PP_VPID_FLG_RX_DISBL );
                }
            }

            ti_hil_null_hook(skb, netfilter_null_vpid_handle);
            dev_put(input_dev);
            PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);

            break;
        }

#endif /* CONFIG_NETFILTER */
#ifdef CONFIG_IP_MULTICAST
    case TI_MC_SESSION_DELETED:
        {
            
            if(NULL == ptr)
                {
                printk ("FATAL Error: Multicast params is NULL\n");
                break;
                }
            ptr_mfc_cache = (struct mfc_cache* )ptr;
            hil_mfc_delete_session_by_mc_group(ptr_mfc_cache->mfc_mcastgrp);
            break;
        }
        case TI_MFC_ENTRY_CREATED:
        {
            /* Event indicates that a Multicast Forwarding Cache Entry has been created. This is typically
             * the case when a proxy or multicast routing daemon has detected a multicast group to be active
             * and has created the entry which will allow multicast packets to flow through the box.
             * The parameter passed to the event is the Multicast Forwarding Cache Entry.
             *
             * This event can be ignored if the System does not support Multicast Routing. */
            TI_PP_SESSION    session;
            int              session_handle;
            unsigned int     lockKey;

            if(NULL == ptr)
            {
                printk ("FATAL Error: Multicast params is NULL\n");
                break;
            }

            PAL_osProtectEntry(PAL_OSPROTECT_INTERRUPT, &lockKey);

            /* Get the MFC Cache Entry. */

            ptr_mfc_cache =  ((struct pp_mr_param *)(ptr))->cache;

#ifdef CONFIG_TI_HIL_DEBUG
            if (0 == global_ti_hil_db.dbg_disabled)
            {
                printk ("Intrusive -> Join Multicast Group %x\n", ptr_mfc_cache->mfc_mcastgrp);
            }
#endif /* CONFIG_TI_HIL_DEBUG */

            /* Convert the MFC Cache to a session structure */
            if (hil_mfc_to_session((struct pp_mr_param *)ptr, &session) == 0)
            {
                /* Conversion was successful; so lets try and create a session */

                // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! //
                // !!!!!!!!!!!!! TBD: skb is not initialized here !!!!!!!!!!!!! //
                // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! //
                session_handle = ti_ppm_create_session(&session, (void*)skb, 0);
                if (session_handle < 0)
                {
                    printk ("FATAL Error: Multicast session creation failed\n");
                }
                else
                {
                    /* Map the session and MFC Entry together. */
                    printk ("Multicast Session %d created succesfully\n", session_handle);
                    hil_mfc_add_entry (ptr_mfc_cache->mfc_mcastgrp, session_handle);
                }
            }
            else
            {
                /* Conversion was not successful. */
                printk ("Error: MFC to session conversion failed\n");
            }

            PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);
            break;
        }
        case TI_MFC_ENTRY_DELETED:
        {
            /* Event indicates that a Multicast Forwarding Cache Entry has been deleted. This is typically
             * the case when a proxy or multicast routing daemon has detected a multicast group to be inactive
             * and has deleted the Multicast Forwarding cavhe entry
             *
             * This event can be ignored if the System does not support Multicast Routing. */
            int session_handle;
            unsigned int     lockKey;

            PAL_osProtectEntry(PAL_OSPROTECT_INTERRUPT, &lockKey);

            /* Get the MFC cache entry which needs to be deleted. */
            ptr_mfc_cache = (struct mfc_cache* )ptr;

#ifdef CONFIG_TI_HIL_DEBUG
            if (0 == global_ti_hil_db.dbg_disabled)
            {
                printk ("Intrusive -> Leave Multicast Group %x\n", ptr_mfc_cache->mfc_mcastgrp);
            }
#endif /* CONFIG_TI_HIL_DEBUG */

            /* Get the session handle and delete the MFC and Session Handle mapping */
            session_handle = hil_mfc_del_entry(ptr_mfc_cache->mfc_mcastgrp);
            if (session_handle >= 0)
            {
                /* Timer has expired for a session; time to delete the session. */
                if (ti_ppm_delete_session (session_handle, NULL) < 0)
                    printk ("Error: Unable to delete session %d\n", session_handle);
            }

            PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);
            break;
        }
#endif /* CONFIG_IP_MULTICAST */

#ifdef CONFIG_VLAN_8021Q
        case TI_VLAN_DEV_CREATED:
        {
            /* Get the pointer to the network device. */
            dev = (struct net_device*)ptr;

#ifdef CONFIG_TI_HIL_DEBUG
            if (0 == global_ti_hil_db.dbg_disabled)
            {
                printk ("Intrusive -> VLAN Device %s has been created\n", dev->name);
            }
#endif /* CONFIG_TI_HIL_DEBUG */

            /* We know for sure that this is attached to a VLAN interface.
             * So configure the VPID Information Block appropriately. The egress
             * MTU of the interface needs to be correctly handled to account for
             * the VLAN Header. */
            dev->vpid_block.type            = TI_PP_VLAN;
            dev->vpid_block.vlan_identifier = (vlan_dev_info(dev))->vlan_id;
            dev->vpid_block.egress_mtu      = dev->vpid_block.egress_mtu - VLAN_HLEN;
            break;
        }
        case TI_VLAN_DEV_DELETED:
        {
            /* Get the pointer to the network device. */
            dev = (struct net_device*)ptr;

#ifdef CONFIG_TI_HIL_DEBUG
            if (0 == global_ti_hil_db.dbg_disabled)
            {
                printk ("Intrusive -> VLAN Device %s is being removed\n", dev->name);
            }
#endif /* CONFIG_TI_HIL_DEBUG */
            break;
        }
#endif /* CONFIG_VLAN_8021Q */

#ifdef CONFIG_PPPOE
        case TI_PPP_INTERFACE_CREATED:
        {
            /* Get the pointer to the network device. */
            dev = (struct net_device*)ptr;

#ifdef CONFIG_TI_HIL_DEBUG
            if (0 == global_ti_hil_db.dbg_disabled)
            {
                printk ("Intrusive -> PPP Device %s has been created Session ID:0x%x\n", dev->name, dev->padded);
            }
#endif /* CONFIG_TI_HIL_DEBUG */

            /* PPPoE could be executed over a VLAN connection or on a vanilla Ethernet
             * connection. Configure the VPID device type appropriately. */
            if (dev->vpid_block.type == TI_PP_VLAN)
            {
                /* The PPP connection is initialized over a VLAN connection. Configure the Egress MTU
                 * appropriately accounting for the PPP and VLAN headers. */
                dev->vpid_block.type       = TI_PP_VLAN_PPPoE;
                dev->vpid_block.egress_mtu = dev->vpid_block.egress_mtu - sizeof(struct pppoe_hdr) - 2 - VLAN_HLEN;
            }
            else
            {
                /* The PPP Connection is being bought over a vanilla Ethernet connection. Configure
                 * the Egress MTU appropriately accounting only for the PPP header */
                dev->vpid_block.type        = TI_PP_PPPoE;
                dev->vpid_block.egress_mtu  = dev->vpid_block.egress_mtu - sizeof(struct pppoe_hdr) - 2;
            }

            /* Extract and configure the PPP Session ID. */
            dev->vpid_block.ppp_session_id  = dev->padded;
            break;
        }
#endif /* CONFIG_PPPOE */
        default:
        {
            printk ("Intrusive -> Does not handle event 0x%lx\n", event_id);
            break;
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
    /* Process based on the module identifier */
    switch (module_id)
    {
#if 0 /* TODO: Enable once router functionality is available */
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
#endif
        case TI_PP:
        {
            /* HIL PP Handler. */
            ti_hil_pp_handler (event_id, ptr);
            break;
        }

        default:
            break;
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
            printk("FATAL Error: PPM is OUT of memory\n");
            break;
        }
        case TI_PPM_INTERNAL_ERROR:
        {
            printk("FATAL Error: PPM Internal Error\n");
            break;
        }
        case TI_PPM_CREATE_PID_FAILED:
        {
            printk("FATAL Error: PP Operation TI_PPM_CREATE_PID_FAILED, pid_handle=%d\n", param1);
            break;
        }
        case TI_PPM_DELETE_PID_FAILED:
        {
            printk("FATAL Error: PP Operation TI_PPM_DELETE_PID_FAILED, pid_handle=%d\n", param1);
            break;
        }
        case TI_PPM_CREATE_VPID_FAILED:
        {
            printk("FATAL Error: PP Operation TI_PPM_CREATE_VPID_FAILED, vpid_handle=%d\n", param1);
            break;
        }
        case TI_PPM_DELETE_VPID_FAILED:
        {
            printk("FATAL Error: PP Operation TI_PPM_DELETE_VPID_FAILED, vpid_handle=%d\n", param1);
            break;
        }
        case TI_PPM_CREATE_SESSION_FAILED:
        {
            printk("Error: PP Operation TI_PPM_CREATE_SESSION_FAILED, session_handle=%d\n", param1);
            break;
        }
        case TI_PPM_DELETE_SESSION_FAILED:
        {
            printk("Error: PP Operation TI_PPM_DELETE_SESSION_FAILED, session_handle=%d\n", param1);
            break;
        }
        case TI_PPM_MODIFY_SESSION_FAILED:
        {
            printk ("Error: PP Operation TI_PPM_MODIFY_SESSION_FAILED, session_handle=%d\n", param1);
            break;
        }
        case TI_PPM_PID_CREATED:
        {
            printk ("PP Operation TI_PPM_PID_CREATED, pid_handle=%d\n", param1);
            break;
        }
        case TI_PPM_PID_DELETED:
        {
            printk ("PP Operation TI_PPM_PID_DELETED, pid_handle=%d\n", param1);
            break;
        }
        case TI_PPM_VPID_CREATED:
        {
            printk ("PP Operation TI_PPM_VPID_CREATED, vpid_handle=%d\n", param1);
            break;
        }
        case TI_PPM_VPID_DELETED:
        {
            printk ("PP Operation TI_PPM_VPID_DELETED, vpid_handle=%d\n", param1);
            break;
        }
        case TI_PPM_SESSION_MODIFIED:
        {
            break;
        }
        case TI_PPM_SESSION_CREATED:
        {
#ifdef CONFIG_TI_PACKET_PROCESSOR_STATS
            if (ti_hil_start_session_notification_cb)
            {
                if (((struct sk_buff*)param2)->pp_packet_info.ti_session.egress[0].vpid_handle ==
                    docsis_null_vpid_handle)
                {
                    ti_hil_start_session_notification_cb(param1, TI_DOCSIS_PP_SESSION_TYPE_DISCARDING,
                                                         ((struct sk_buff*)param2));
                }
                else
                {
                    ti_hil_start_session_notification_cb(param1, TI_DOCSIS_PP_SESSION_TYPE_FORWARDING,
                                                         ((struct sk_buff*)param2));
                }
            }
#endif /* CONFIG_TI_PACKET_PROCESSOR_STATS */

            /* Do the necessary work for the HIL Analysis. */
            {
                int                 num_current_session;
                int                 bucket;

                global_ti_hil_db.num_total_sessions++;
                num_current_session = ti_ppm_get_session (-1, 0, NULL);
                bucket = num_current_session / HIL_SESSION_STAT_BUCKET;

                /* If the number of sessions is the maximum number of sessions, then the above equation results
                * in calculating a bucket one more than the expected number leading to erroneous calculations.
                * Thus decrement the bucket number so as to increment the session counter in the last bucket.
                * For e.g.,
                *  num_current_session = 256 then, bucket = 256 / 64 = 4 so we end up incrementing the
                *  counter in bucket[4] when the valid buckets are 0,1,2,and 3. */
                if(bucket == HIL_MAX_NUM_BUCKETS)
                    bucket--;
                global_ti_hil_db.session_bucket[bucket]++;
            }
            break;
        }
        case TI_PPM_SESSION_DELETED:
        {
#ifdef CONFIG_TI_PACKET_PROCESSOR_STATS
            if (ti_hil_delete_session_notification_cb)
            {
                ti_hil_delete_session_notification_cb(param1, ((TI_PP_SESSION_STATS*)param2)->packets_forwarded);
            }
#endif /* CONFIG_TI_PACKET_PROCESSOR_STATS */
            {
                unsigned int lockKey;

                PAL_osProtectEntry(PAL_OSPROTECT_INTERRUPT, &lockKey);

                ti_hil_tdox_free_session(param1, -1);

#ifdef CONFIG_NETFILTER
                {

                    struct nf_conn* ct = hil_session_ct_mapper[param1];

                    /* Once the session has been removed; check if the session handle was present in the mapper
                     * this implies that the session handle and connection tracking entry were connected to
                     * each other. */
                    if (ct != NULL)
                    {

                        if (ct->tuplehash[IP_CT_DIR_REPLY   ].ti_pp_session_handle == param1)
                        {
                            ct->tuplehash[IP_CT_DIR_REPLY   ].ti_pp_session_handle = TI_PP_SESSION_CT_TCP_UPDATE;
                            if (IS_TI_PP_SESSION_CT_INVALID(ct->tuplehash[IP_CT_DIR_ORIGINAL].ti_pp_session_handle))
                            {
                                ct->tuplehash[IP_CT_DIR_ORIGINAL].ti_pp_session_handle = TI_PP_SESSION_CT_TCP_UPDATE;
                            }
                        }

                        if (ct->tuplehash[IP_CT_DIR_ORIGINAL].ti_pp_session_handle == param1)
                        {
                            ct->tuplehash[IP_CT_DIR_ORIGINAL].ti_pp_session_handle = TI_PP_SESSION_CT_TCP_UPDATE;
                            if (IS_TI_PP_SESSION_CT_INVALID(ct->tuplehash[IP_CT_DIR_REPLY].ti_pp_session_handle))
                            {
                                ct->tuplehash[IP_CT_DIR_REPLY].ti_pp_session_handle = TI_PP_SESSION_CT_TCP_UPDATE;
                            }
                        }

                        /* The mapper slot is free now! */
                        hil_session_ct_mapper[param1] = NULL;
                    }
                }
#endif /* CONFIG_NETFILTER */
                PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);
                break;
            }
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
            /* Session has EXPIRED; param1 is the session handle that has expired. */
            TI_PP_SESSION_STATS    session_stats;

            /* Timer has expired for a session; time to delete the session. */
            if (ti_ppm_delete_session (param1, &session_stats) < 0)
            {
                printk ("Error: Unable to delete session %d\n", param1);
                return;
            }

#ifdef CONFIG_TI_HIL_DEBUG
            if (0 == global_ti_hil_db.dbg_disabled)
            {
                /* Print the session stats. */
                printk ("--------- Session Stats %d ---------\n", param1);
                printk ("Number of Packets Forwarded: %u\n", session_stats.packets_forwarded);
                printk ("Number of Bytes   Forwarded: %u\n", session_stats.bytes_forwarded_hi);
                printk ("Number of Bytes   Forwarded: %u\n", session_stats.bytes_forwarded_lo);
                printk ("------------------------------------\n");
            }
#endif /* CONFIG_TI_HIL_DEBUG */
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

    /* Process each event based on the module which generated the event. */
    switch (module)
    {
        case TI_PP_MODULE:
        {
            /* Packet Processor Event */
            ti_hil_pp_event_handler (event_id, param1, param2);
            break;
        }
        case TI_PPM_MODULE:
        {
            /* Packet Processor Manager Event. */
            ti_hil_pp_ppm_event_handler (event_id, param1, param2);
            break;
        }
        case TI_PPD_MODULE:
        {
            /* Packet Processor Driver Event. */
            printk ("PPD Event 0x%x Param1: 0x%x Param2: 0x%x\n", event_id, param1, param2);
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

#ifdef CONFIG_IP_MULTICAST

/**************************************************************************
 * FUNCTION NAME : hil_mfc_check_entry
 **************************************************************************
 * DESCRIPTION   :
 *  The function checks the HIL MFC Database for a "session id" match for
 *  a particular mcast_group. The database keeps track of all MFC Entries
 *  that have been accelerated.
 *
 * RETURNS       :
 *  Handle of the session     - Match.
 *  -1                        - No Match.
 ***************************************************************************/
static int hil_mfc_check_entry (unsigned int mcast_group)
{
    int session_handle = 0;

    /* Cycle through all the entries */
    for (session_handle = 0; session_handle < TI_PP_MAX_ACCLERABLE_SESSIONS; session_handle++)
    {
        /* Check if there is a valid entry or not? */
        if (hil_mfc_session_mapper[session_handle] != 0)
        {
            /* YES. Match the entry to the multicast group specified. */
            if (hil_mfc_session_mapper[session_handle] == mcast_group)
            {
                /* Perfect Match: Return the PP Session handle */
                return session_handle;
            }
        }
    }

    /* Control comes here implies that there was no match! */
    return -1;
}

/**************************************************************************
 * FUNCTION NAME : hil_mfc_add_entry
 **************************************************************************
 * DESCRIPTION   :
 *  The function adds an entry to the Session to MFC Database
 *
 * RETURNS       :
 *  0     - Success.
 *  <0    - Error.
 ***************************************************************************/
static int hil_mfc_add_entry (unsigned int mcast_group, unsigned char session_handle)
{
    /* Check if the slot is free! */
    if (hil_mfc_session_mapper[session_handle] != 0)
    {
        /* No; slot is not free!*/
        printk ("Session Handle %d already mapped to group: 0x%x\n", session_handle, hil_mfc_session_mapper[session_handle]);
        return -1;
    }

    /* Map and return. */
    hil_mfc_session_mapper[session_handle] = mcast_group;
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : hil_mfc_del_entry
 **************************************************************************
 * DESCRIPTION   :
 *  The function deletes an entry from the Session to MFC Database
 *
 * RETURNS       :
 *  Session Handle     - Success.
 *  <0                 - Error.
 ***************************************************************************/
static int hil_mfc_del_entry (unsigned int mcast_group)
{
    int session_handle = hil_mfc_check_entry(mcast_group);

    /* Check if the session handle exists? */
    if (session_handle == -1)
    {
        /* No Matching entry found. */
        printk ("Multicast group 0x%x not mapped\n", mcast_group);
        return -1;
    }

    /* Unmap and return. */
    hil_mfc_session_mapper[session_handle] = 0;
    return session_handle;
}

/**************************************************************************
 * FUNCTION NAME : hil_mfc_to_session
 **************************************************************************
 * DESCRIPTION   :
 *  The function converts an MFC Entry used for multicast routing to a PP
 *  session structure.
 *
 * RETURNS       :
 *  -1      - Error.
 *  0       - Success.
 ***************************************************************************/
static int hil_mfc_to_session(struct pp_mr_param * ptr_mr_param, TI_PP_SESSION* ptr_session)
{
    int downstream_vif_index;

    /* Initialize the session structure */
    memset ((void *)ptr_session, 0, sizeof(TI_PP_SESSION));

    /* Configure the session timeout. For MFC we can create a static session; this is added till we
     * get the new header file; since we need to configure the session flag correctly. */
    ptr_session->session_timeout = 0;

    /* This is for sure a ROUTED SESSION. */
    ptr_session->is_routable_session = 1;

    /* Configure the Ingress Session Properties. We need to know from which VPID the packet will
     * be arriving. This can be retrieved from the UPSTREAM VIF Index.But we would need to convert
     * the VIF Index to VPID before we can do this. */
    if (ptr_mr_param->vif_table[ptr_mr_param->cache->mfc_parent].dev != NULL)
    {
        /* Check if the VPID has a VPID handle or not? If not then we cannot create a session in the PP
         * since the device does not belong to the Packet processor subsystem. */
        if (ptr_mr_param->vif_table[ptr_mr_param->cache->mfc_parent].dev->vpid_handle == -1)
            return -1;
    }
    else
    {
        printk ("FATAL Error: VIF Index %d does not map to a device\n", ptr_mr_param->cache->mfc_parent);
        return -1;
    }

    /* Configure the Ingress Session Properties. */
    ptr_session->ingress.vpid_handle           = ptr_mr_param->vif_table[ptr_mr_param->cache->mfc_parent].dev->vpid_handle;
    ptr_session->ingress.l2_packet.packet_type = TI_PP_ETH_TYPE;

    /* Configuration of the Destination MAC Address; we could take two approaches here!
     *  a) Have a wild carded destination MAC Address
     *  b) Convert the Multicast IPv4 address to a L2 address and configure it.
     * Approach (b) seems to be a more robust solution.
     * We make sure that the SOURCE MAC Address is WILDCARDED! Since at this time we dont
     * know from where the packet is arriving. */
    ip_eth_mc_map (ptr_mr_param->cache->mfc_mcastgrp, &ptr_session->ingress.l2_packet.u.eth_desc.dstmac[0]);
    ptr_session->ingress.l2_packet.u.eth_desc.enables = TI_PP_SESSION_L2_DSTMAC_VALID;

    /* Configure the LUT Entry for Layer3 Classification.
     * The following fields are WILDCARDED @ Layer3
     *  a) Source IP Address
     *  b) Protocol
     *  c) TOS
     *  f) Source Port
     *  g) Destination Port
     * This information is not known @ the time the MFC entry is created. */
    ptr_session->ingress.l3l4_packet.packet_type        = TI_PP_IPV4_TYPE;
    ptr_session->ingress.l3l4_packet.u.ipv4_desc.dst_ip = ptr_mr_param->cache->mfc_mcastgrp;
    ptr_session->ingress.l3l4_packet.u.eth_desc.enables = TI_PP_SESSION_IPV4_DSTIP_VALID;

    /* Configure the Egress Session Properties.
     *   - Multicast packets are not subjected to any NAT and so there are no modifications
     *     which need to be done at Layer3 or Layer4. Thus all we populate in the Egress
     *     Session Properties are the Egress VPID on which the packets need to be sent out. */
    for (downstream_vif_index=0; downstream_vif_index < MAXVIFS; downstream_vif_index++)
    {
        /* Check if a valid VIF is present or not? */
        if (ptr_mr_param->cache->mfc_un.res.ttls[downstream_vif_index] != 0xFF)
        {
            /* Found it! Get the device information from the VIF. */
            if (ptr_mr_param->vif_table[downstream_vif_index].dev != NULL)
            {
                /* Now we need to get the VPID handle. If the VPID handle does not exist then the packet
                 * is being transmitted on an interface which does not support the packet processor and
                 * thus this session is not capable of being accelerated! */
                if (ptr_mr_param->vif_table[downstream_vif_index].dev->vpid_handle == -1)
                    return -1;

                /* Remember the Egress VPID handle. */
                ptr_session->egress[ptr_session->num_egress++].vpid_handle = ptr_mr_param->vif_table[downstream_vif_index].dev->vpid_handle;
            }
            else
            {
                printk ("FATAL Error: VIF Index %d does not map to a device\n", downstream_vif_index);
                return -1;
            }
        }
    }

    /* Once we come out of the loop we should have at least one egress record. */
    if (ptr_session->num_egress == 0)
    {
        printk ("No downstream interface for group 0x%x\n", ptr_mr_param->cache->mfc_mcastgrp);
        return -1;
    }

    /* Successfully translated. */
    return 0;
}

#endif /* CONFIG_IP_MULTICAST */

/**************************************************************************
 * FUNCTION NAME : ti_hil_intrusive_display_l2
 **************************************************************************
 * DESCRIPTION   :
 *  The function prints the Layer2 Information
 **************************************************************************/
static void ti_hil_intrusive_display_l2 (TI_PP_ETH_DESC* ptr_eth_desc)
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

    if (ptr_eth_desc->enables & TI_PP_SESSION_L2_VLAN_VALID)
        printk ("VLAN TAG     = %04x\n", ptr_eth_desc->vlan_tag );

    /* Work is completed. */
    return;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_intrusive_display_ipv4
 **************************************************************************
 * DESCRIPTION   :
 *  The function prints the IPv4 Information
 **************************************************************************/
static void ti_hil_intrusive_display_ipv4 (TI_PP_IPV4_DESC* ptr_ipv4_lut_entry)
{
    /* Check if the Destination IP has been specified. */
    if (ptr_ipv4_lut_entry->enables & TI_PP_SESSION_IPV4_DSTIP_VALID)
    {
        unsigned char * ipPtr = (unsigned char *)&ptr_ipv4_lut_entry->dst_ip;
        printk ("Dst IP       = 0x%08X [ %d.%d.%d.%d ]\n", ptr_ipv4_lut_entry->dst_ip, ipPtr[0], ipPtr[1], ipPtr[2], ipPtr[3]);
    }
    else
        printk ("Dst IP       = XXX\n");

    /* Check if the Source IP has been specified. */
    if (ptr_ipv4_lut_entry->enables & TI_PP_SESSION_IPV4_SRCIP_VALID)
    {
        unsigned char * ipPtr = (unsigned char *)&ptr_ipv4_lut_entry->src_ip;
        printk ("Src IP       = 0x%08X [ %d.%d.%d.%d ]\n", ptr_ipv4_lut_entry->src_ip, ipPtr[0], ipPtr[1], ipPtr[2], ipPtr[3]);
    }
    else
        printk ("Src IP       = XXX\n");

    /* Check if the protocol has been specified. */
    if (ptr_ipv4_lut_entry->enables & TI_PP_SESSION_IPV4_PROTOCOL_VALID)
    {
        printk ("Protocol     = 0x%02X ", ptr_ipv4_lut_entry->protocol);
        switch (ptr_ipv4_lut_entry->protocol)
        {
        case IPPROTO_IPIP:
            printk("[IPinIP]\n");
            break;
        case IPPROTO_TCP:
            printk("[TCP]\n");
            break;
        case IPPROTO_UDP:
            printk("[UDP]\n");
            break;
        case IPPROTO_GRE:
            printk("[GRE]\n");
            break;
        case IPPROTO_IPV6:
            printk("[IPv6-in-IPv4 tunnelling]\n");
            break;
        default:
            printk("[???]\n");
            break;
        }
    }
    else
    {
        printk ("Protocol     = XXX\n");
    }

    /* Check if the TOS Byte has been specified. */
    if (ptr_ipv4_lut_entry->enables & TI_PP_SESSION_IPV4_TOS_VALID)
        printk ("TOS          = 0x%02X\n", ptr_ipv4_lut_entry->tos);
    else
        printk ("TOS          = XXX\n");

    /* Check if the Destination Port has been specified. */
    if (ptr_ipv4_lut_entry->enables & TI_PP_SESSION_IPV4_DST_PORT_VALID)
        printk ("Dst Port     = 0x%04X [%d]\n", ptr_ipv4_lut_entry->dst_port, ptr_ipv4_lut_entry->dst_port);
    else
        printk ("Dst Port     = XXX\n");

    /* Check if the Source Port has been specified. */
    if (ptr_ipv4_lut_entry->enables & TI_PP_SESSION_IPV4_SRC_PORT_VALID)
        printk ("Src Port     = 0x%04X [%d]\n", ptr_ipv4_lut_entry->src_port, ptr_ipv4_lut_entry->src_port);
    else
        printk ("Src Port     = XXX\n");

    /* Work is completed. */
    return;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_intrusive_display_ipv6
 **************************************************************************
 * DESCRIPTION   :
 *  The function prints the IPv4 Information
 **************************************************************************/
static void ti_hil_intrusive_display_ipv6 (TI_PP_IPV6_DESC*   ptr_ipv6_lut_entry)
{
    char ipv6addr[64];
    const char* buf;

    /* Check if the Destination IP has been specified. */
    if (ptr_ipv6_lut_entry->enables & TI_PP_SESSION_IPV6_DSTIP_VALID)
    {
        buf = ti_hil_intrusive_display_ipv6_addr(ptr_ipv6_lut_entry->dst_ip, ipv6addr, sizeof(ipv6addr));
        if (buf != NULL)
        {
            printk ("Dst IP       = %s\n", buf);
        }
        else
        {
            printk ("Dst IP       = XXX\n");
        }
    }
    else
    {
        printk ("Dst IP       = XXX\n");
    }

    /* Check if the Source IP has been specified. */
    if (ptr_ipv6_lut_entry->enables & TI_PP_SESSION_IPV6_SRCIP_VALID)
    {
        buf = ti_hil_intrusive_display_ipv6_addr(ptr_ipv6_lut_entry->src_ip, ipv6addr, sizeof(ipv6addr));
        if (buf != NULL)
        {
            printk ("Src IP       = %s\n", buf);
        }
        else
        {
            printk ("Src IP       = XXX\n");
        }
    }
    else
    {
        printk ("Src IP       = XXX\n");
    }

    /* Check if the next header has been specified. */
    if (ptr_ipv6_lut_entry->enables & TI_PP_SESSION_IPV6_NEXTHDR_VALID)
    {
        printk ("Protocol     = 0x%02X ", ptr_ipv6_lut_entry->next_header);
        switch (ptr_ipv6_lut_entry->next_header)
        {
        case IPPROTO_IPIP:
            printk("[IPinIP]\n");
            break;
        case IPPROTO_TCP:
            printk("[TCP]\n");
            break;
        case IPPROTO_UDP:
            printk("[UDP]\n");
            break;
        case IPPROTO_GRE:
            printk("[GRE]\n");
            break;
        case IPPROTO_IPV6:
            printk("[IPv6-in-IPv4 tunnelling]\n");
            break;
        default:
            printk("[???]\n");
            break;
        }
    }
    else
    {
        printk ("Protocol     = XXX\n");
    }

    /* Check if the Traffic Class Byte has been specified. */
    if (ptr_ipv6_lut_entry->enables & TI_PP_SESSION_IPV6_TRCLASS_VALID)
        printk ("TOS          = 0x%02X\n", ptr_ipv6_lut_entry->traffic_class);
    else
        printk ("TOS          = XXX\n");

    /* Check if the Destination Port has been specified. */
    if (ptr_ipv6_lut_entry->enables & TI_PP_SESSION_IPV6_DST_PORT_VALID)
        printk ("Dst Port     = 0x%04X [%d]\n", ptr_ipv6_lut_entry->dst_port, ptr_ipv6_lut_entry->dst_port);
    else
        printk ("Dst Port     = XXX\n");

    /* Check if the Source Port has been specified. */
    if (ptr_ipv6_lut_entry->enables & TI_PP_SESSION_IPV6_SRC_PORT_VALID)
        printk ("Src Port     = 0x%04X [%d]\n", ptr_ipv6_lut_entry->src_port, ptr_ipv6_lut_entry->src_port);
    else
        printk ("Src Port     = XXX\n");

    if (ptr_ipv6_lut_entry->enables & TI_PP_SESSION_IPV6_DSLITE_DSTIP_VALID)
    {
        unsigned char * ipPtr = (unsigned char *)&ptr_ipv6_lut_entry->dsLite_dst_ip;
        printk ("DsLite DstIP = 0x%08x [ %d.%d.%d.%d ]\n", ptr_ipv6_lut_entry->dsLite_dst_ip, ipPtr[0], ipPtr[1], ipPtr[2], ipPtr[3]);
    }

    /* Work is completed. */
    return;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_intrusive_display_ipv6_addr
 **************************************************************************
 * DESCRIPTION   :
 *  The function converts a numeric address into a text string suitable for presentation.
 **************************************************************************/
static const char* ti_hil_intrusive_display_ipv6_addr (const void *cp, char *buf, size_t len)
{
    size_t xlen;

    const struct in6_addr *s = (const struct in6_addr *)cp;

    xlen = snprintf(buf, len, "%x:%x:%x:%x:%x:%x:%x:%x",
                    ntohs(s->s6_addr16[0]), ntohs(s->s6_addr16[1]),
                    ntohs(s->s6_addr16[2]), ntohs(s->s6_addr16[3]),
                    ntohs(s->s6_addr16[4]), ntohs(s->s6_addr16[5]),
                    ntohs(s->s6_addr16[6]), ntohs(s->s6_addr16[7]));

    if (xlen > len)
    {
        return NULL;
    }

    return buf;
}

/**************************************************************************
 * FUNCTION NAME : hil_mfc_delete_session_by_mc_group
 **************************************************************************
 * DESCRIPTION   :
 *  The function deletes a session by a multicast group identifier
 **************************************************************************/

static int hil_mfc_delete_session_by_mc_group(int mc_group)
{
    int session_handle ;
    TI_PP_SESSION    session;
    
    /* Cycle through all the entries */
    for (session_handle = TI_PP_MAX_ACCLERABLE_SESSIONS-1; session_handle > -1; session_handle--)
    {
        /* Get the pointer to the Session Information. */
        if (ti_ppm_get_session_info(session_handle, &session) < 0)
            {
            continue;
            }
        if (session.ingress.l3l4_packet.u.ipv4_desc.dst_ip == mc_group)
        {
            ti_ppm_delete_session(session_handle, NULL);
            return 0;
        }
       
    }
    return -1;
   
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_intrusive_display_session
 **************************************************************************
 * DESCRIPTION   :
 *  The function displays the session properties.
 **************************************************************************/
static void ti_hil_intrusive_display_session (int session_handle)
{
    TI_PP_SESSION    session;
    int              index;
    TI_PP_VPID      vpid;
    TI_PP_PID       pid;

    /* Get the pointer to the Session Information. */
    if (ti_ppm_get_session_info(session_handle, &session) < 0)
    {
        return;
    }

    /* Print the Session Parameters. */
    printk ("Property : %s\n", session.is_routable_session ? "Routed": "Bridged");
    printk ("Timeout  : 0x%08X [%d usec]\n", session.session_timeout, session.session_timeout);
    printk ("Priority : %d\n", session.priority);

    /* Print the Ingress Session Properties; which include the L2/L3 and L4 properties. */
    printk ("\nIngress Properties");
    if (session.ingress.l2_packet.u.eth_desc.enables & TI_PP_SESSION_L2_GRE_DS_VALID)
    {
        printk(" (DS GRE - The ingress properties are of the encapsulated packet)");
    }
    printk ("\n------------------\n");

    ti_ppm_get_vpid_info (session.ingress.vpid_handle, &vpid);
    ti_ppm_get_pid_info  (vpid.parent_pid_handle, &pid);
    if (TI_PP_PID_TYPE_DOCSIS == pid.type)
    {
        printk ("Ingress VPID = %d [CNI]\n", session.ingress.vpid_handle);
    }
    else if (TI_PP_PID_TYPE_ETHERNET == pid.type)
    {
        printk ("Ingress VPID = %d [ETH]\n", session.ingress.vpid_handle);
    }
    else if (TI_PP_PID_TYPE_ETHERNETSWITCH == pid.type)
    {
        printk ("Ingress VPID = %d [ETH Switch]\n", session.ingress.vpid_handle);
    }
    else
    {
        printk ("Ingress VPID = %d\n", session.ingress.vpid_handle);
    }

    if (session.ingress.l2_packet.packet_type == TI_PP_ETH_TYPE)
    {
        ti_hil_intrusive_display_l2 (&session.ingress.l2_packet.u.eth_desc);
    }
    else
    {
        printk ("No L2 Properties\n");
    }
    if (session.ingress.l3l4_packet.packet_type == TI_PP_IPV4_TYPE)
    {
        printk ("IP Version   = IPv4\n");
        ti_hil_intrusive_display_ipv4 (&session.ingress.l3l4_packet.u.ipv4_desc);
    }
    else if (session.ingress.l3l4_packet.packet_type == TI_PP_IPV6_TYPE)
    {
        printk ("IP Version   = IPv6\n");
        ti_hil_intrusive_display_ipv6 (&session.ingress.l3l4_packet.u.ipv6_desc);
    }
    else
        printk ("No L3/L4 Properties\n");

    /* Cycle through all the Egress Session Properties. */
    for (index = 0; index < session.num_egress; index++)
    {
        printk ("\nEgress Properties (%d)", index+1);
        printk ("\n---------------------\n");

        /* Print the Ingress Session Properties; which include the L2/L3 and L4 properties. */
        ti_ppm_get_vpid_info (session.egress[index].vpid_handle, &vpid);
        ti_ppm_get_pid_info  (vpid.parent_pid_handle, &pid);
        if (TI_PP_PID_TYPE_DOCSIS == pid.type)
        {
            printk ("Egress VPID  = %d [CNI]\n", session.egress[index].vpid_handle);
        }
        else if (TI_PP_PID_TYPE_ETHERNET == pid.type)
        {
            printk ("Egress VPID  = %d [ETH]\n", session.egress[index].vpid_handle);
        }
        else if (TI_PP_PID_TYPE_ETHERNETSWITCH == pid.type)
        {
            printk ("Egress VPID  = %d [ETH Switch]\n", session.egress[index].vpid_handle);
        }
        else
        {
            printk ("Egress VPID  = %d\n", session.egress[index].vpid_handle);
        }

        if (session.egress[index].l2_packet.packet_type == TI_PP_ETH_TYPE)
            ti_hil_intrusive_display_l2 (&session.egress[index].l2_packet.u.eth_desc);
        else
            printk ("No L2 Properties\n");
        if (session.egress[index].l3l4_packet.packet_type == TI_PP_IPV4_TYPE)
        {
            printk ("IP Version   = IPv4\n");
            ti_hil_intrusive_display_ipv4 (&session.egress[index].l3l4_packet.u.ipv4_desc);
        }
        else if (session.egress[index].l3l4_packet.packet_type == TI_PP_IPV6_TYPE)
        {
            printk ("IP Version   = IPv6\n");
            ti_hil_intrusive_display_ipv6 (&session.egress[index].l3l4_packet.u.ipv6_desc);
        }
        else
            printk ("No L3/L4 Properties\n");
    }

    /* Work has been completed. */
    return;
}


#if (CONFIG_MACH_PUMA6)
PAL_CPPI41_SR_QMGR_QUEUES_STR(qname);
#endif

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
    /****************************** VALIDATIONS ***************************/

    /* Validate the number of arguments that have been passed. */
    if (argc != 2 && argc != 3)
    {
        printk ("ERROR: Incorrect Number of parameters passed.\n");
        return -1;
    }

    /**************************** End of VALIDATIONS ***********************/

    if (strcmp(argv[1], "ver") == 0)
    {
        TI_PP_VERSION version;

        if (0 != ti_ppm_get_version( &version ))
        {
            printk(" Version retrieve failed ... \n");
        }
        else
        {
            printk(" Packet Processor Firmware Version : %d.%d.%d.%d \n", version.v0, version.v1, version.v2, version.v3 );
        }
        return 0;
    }


    if (strcmp(argv[1], "tdox") == 0)
    {
        ti_hil_tdox_print();
        return 0;
    }

    /* Check if the "Global" stats were requested. */
    if (strcmp(argv[1], "global") == 0)
    {
        TI_PP_GLOBAL_STATS  pp_stats;

        /* YES. Get the global stats through the PPM */
        ti_ppm_get_global_stats(&pp_stats);

        /* Print the stats on the console. */
        printk ("Packets received in the PP   : %u\n", pp_stats.packets_rxed);
        printk ("Number of search attemps     : %u\n", pp_stats.packets_searched);
        printk ("Number of matched searches   : %u\n", pp_stats.search_matched);
        printk ("Number of Synch Delays       : %u\n", pp_stats.sync_delay);
        printk ("Packet forwarded by the PP   : %u\n", pp_stats.packets_fwd);
        printk ("IPv4 Packets Forwarded       : %u\n", pp_stats.ipv4_packets_fwd);
        printk ("Descriptors Starved          : %u\n", pp_stats.desc_starved);
        printk ("Buffers Starved              : %u\n", pp_stats.buffer_starved);

        /* Work is completed. */
        return 0;
    }

    /* Check if VPID statistics were requested? */
    if (strcmp(argv[1], "vpid") == 0)
    {
        TI_PP_VPID  vpid[TI_PP_MAX_PID + 1];
        int         num_vpid;
        int         index = 0;

        /* Get a list of all VPID that exist in the System */
        num_vpid = ti_ppm_get_vpid (-1, TI_PP_MAX_PID, &vpid[0]);

            printk ("-----------------------------------------\n");
            printk ("    HIL State is  : %s\n", (global_ti_hil_db.hil_disabled)?"Disabled":"Enabled" );


        /* Cycle through all the VPID and get the stats */
        while (index < num_vpid)
        {
            TI_PP_VPID_STATS   vpid_stats;
            char * vpid_type;

            switch (vpid[index].type)
            {
                case TI_PP_ETHERNET     : vpid_type = "ETHERNET   " ; break;
                case TI_PP_VLAN         : vpid_type = "VLAN       " ; break;
                case TI_PP_PPPoE        : vpid_type = "PPPoE      " ; break;
                case TI_PP_VLAN_PPPoE   : vpid_type = "VLAN_PPPoE " ; break;
                default:                  vpid_type = "UNKNOWN"     ; break;
            }
            /* Get the VPID statistics. */
            ti_ppm_get_vpid_stats(vpid[index].vpid_handle, &vpid_stats);

            /* Print the statistics on the console. */
            printk ("---------------- VPID %d (PID %d) ----------------\n", vpid[index].vpid_handle, vpid[index].parent_pid_handle );
            printk ("                 type: %s \n", vpid_type );
            printk ("Rx Unicast   Packets: %u\n",     vpid_stats.rx_unicast_pkt);
            printk ("Rx Broadcast Packets: %u\n",     vpid_stats.rx_broadcast_pkt);
            printk ("Rx Multicast Packets: %u\n",     vpid_stats.rx_multicast_pkt);
            printk ("Rx Bytes            : 0x%08X%08X\n",  vpid_stats.rx_byte_hi, vpid_stats.rx_byte_lo);
            printk ("Rx Bytes - Low (dec): %u\n",     vpid_stats.rx_byte_lo);
            printk ("Rx Discard          : %u\n",     vpid_stats.rx_discard);
            printk ("Tx Unicast   Packets: %u\n",     vpid_stats.tx_unicast_pkt);
            printk ("Tx Broadcast Packets: %u\n",     vpid_stats.tx_broadcast_pkt);
            printk ("Tx Multicast Packets: %u\n",     vpid_stats.tx_multicast_pkt);
            printk ("Tx Bytes            : 0x%08X%08X\n",  vpid_stats.tx_byte_hi, vpid_stats.tx_byte_lo);
            printk ("Tx Bytes - Low (dec): %u\n",     vpid_stats.tx_byte_lo);
            printk ("Tx Errors           : %u\n",     vpid_stats.tx_error);
            printk ("Tx Discards         : %u\n",     vpid_stats.tx_discard);
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
        int         temp;

        /* Get the PID Information. */
        num_pid = ti_ppm_get_pid (TI_PP_MAX_PID, &pid[0]);

        /* Cycle through all the VPID and get the stats */
        while (index < num_pid)
        {
            /* Print the PID Information on the console. */
            printk("##########\n");
            printk("# PID %02d #\n", pid[index].pid_handle);
            printk("##########\n\n");
            printk("Type                = %d [", pid[index].type);
            switch (pid[index].type)
            {
            case TI_PP_PID_TYPE_UNDEFINED:      printk("Undefined");        break;
            case TI_PP_PID_TYPE_ETHERNET:       printk("Ethernet");         break;
            case TI_PP_PID_TYPE_INFRASTRUCTURE: printk("Infrastructure");   break;
            case TI_PP_PID_TYPE_USBBULK:        printk("USB Bulk");         break;
            case TI_PP_PID_TYPE_CDC:            printk("CDC");              break;
            case TI_PP_PID_TYPE_DOCSIS:         printk("DOCSIS");           break;
            case TI_PP_PID_TYPE_ETHERNETSWITCH: printk("Ethernet Switch");  break;
            default:                            printk("???");              break;
            }
            printk("]\n");
            printk("PriMapping          = %d\n", pid[index].pri_mapping);
            temp = pid[index].dflt_pri_drp;
            printk("Priority            = 0x%02X [Priority=%d, DropPrecedence=%d]\n", temp, temp&0x7, (temp>>3)&0x3);
            temp = pid[index].ingress_framing;
            printk("Framing             = 0x%02X ", temp);
            if (temp)
            {
                printk("[ ");
                if (temp & TI_PP_PID_INGRESS_ETHERNET)  {printk("ETHERNET ");}
                if (temp & TI_PP_PID_INGRESS_IPV4)      {printk("IPV4 ");}
                if (temp & TI_PP_PID_INGRESS_IPV6)      {printk("IPV6 ");}
                if (temp & TI_PP_PID_INGRESS_IPOE)      {printk("IPOE ");}
                if (temp & TI_PP_PID_INGRESS_IPOA)      {printk("IPOA ");}
                if (temp & TI_PP_PID_INGRESS_PPPOE)     {printk("PPPOE ");}
                if (temp & TI_PP_PID_INGRESS_PPPOA)     {printk("PPPOA ");}
                printk("]");
            }
            printk("\n");
            temp = pid[index].priv_flags;
            printk("Flags               = 0x%02X ", temp);
            if (temp)
            {
                printk("[ ");
                if (temp & TI_PP_PID_VLAN_PRIO_MAP)         {printk("fMapVLAN ");}
                if (temp & TI_PP_PID_DIFFSRV_PRIO_MAP)      {printk("fMapDIFSRV ");}
                if (temp & TI_PP_PID_PRIO_OFF_TX_DST_TAG)   {printk("fUseDestTag ");}
                if (temp & TI_PP_PID_CLASSIFY_BYPASS)       {printk("fRxDisable ");}
                if (temp & TI_PP_PID_DISCARD_ALL_RX)        {printk("fDiscardRx ");}
                printk("]");
            }
            printk("\n");
            printk("TxDestTag           = 0x%04X\n", pid[index].dflt_dst_tag);
#if (CONFIG_MACH_PUMA6)
            printk("TxQueueBase         = %d [%s]\n\n", pid[index].dflt_fwd_q, qname[pid[index].dflt_fwd_q]);
#else
            printk("TxQueueBase         = %d\n\n", pid[index].dflt_fwd_q);
#endif
      
            /* Goto the next PID. */
            index = index + 1;
        }

        /* Work is completed. */
        return 0;
    }

    /* Check if Analysis Information needs to be displayed? */
    if (strcmp(argv[1], "stats") == 0)
    {
        int index;
        TI_PP_QOS_QUEUE_STATS   qos_stats;
        PPM_STATUS pp_status;

        /* Print the HIL Analysis report. */
        printk (" HIL  State is : %s\n", (global_ti_hil_db.hil_disabled)? "Disabled":"Enabled" );
        pp_status = ti_pp_get_status();
        printk (" PPM  State is : %s\n", (pp_status == INACTIVE) ? "Inactive" : ((pp_status == ACTIVE) ? "Active" : ((pp_status == PPM_PSM) ? "Power Save" : "Unknown")));
        printk (" DBG  State is : %s\n", (global_ti_hil_db.dbg_disabled)? "Disabled":"Enabled" );
        printk (" TDOX State is : %s\n", (global_ti_hil_db.tdox_disabled)? "Disabled":"Enabled" );
        printk (" QoS  State is : %s\n", (global_ti_hil_db.qos_disabled)? "Disabled":"Enabled" );
        printk (" TDOX Threshold : %u ms\n", global_ti_hil_db.tdox_RTT_threshold_ms );
#ifdef CONFIG_INTEL_PP_TUNNEL_SUPPORT
        printk (" Tunnel Mode   : %s\n", (global_ti_hil_db.tunnelMode)? "Enabled":"Disabled" );
#endif
        printk (" Bypassed pkts : %u\n", global_ti_hil_db.num_bypassed_pkts );
        printk (" Other    pkts : %u\n", global_ti_hil_db.num_other_pkts    );
        printk (" Ingress  pkts : %u\n", global_ti_hil_db.num_ingress_pkts  );
        printk (" Egress   pkts : %u\n", global_ti_hil_db.num_egress_pkts   );
        printk (" Null     pkts : %u\n", global_ti_hil_db.num_null_drop_pkts );
        printk (" Total Sessions: %u\n", global_ti_hil_db.num_total_sessions);
        printk (" Total Errors  : %u\n", global_ti_hil_db.num_error);
        printk (" Global Timeout: %u\n", ti_session_timeout);
        for(index = 0; index < HIL_MAX_NUM_BUCKETS; index++)
            printk("Bucket/No.Sessions %d / %d \n", index, global_ti_hil_db.session_bucket[index]);

        if (pp_status == ACTIVE)
        {
            printk ("\n QoS queues statistics:\n");
            printk (" queue | forwarded  | discarded  | owner\n");
            printk ("-------+------------+------------+--------------\n");
            for(index = 0; index <= PAL_CPPI41_SR_QPDSP_QOS_Q_LAST - PAL_CPPI41_SR_QPDSP_QOS_Q_BASE; index++)
            {
                ti_ppm_get_qos_q_stats (index, &qos_stats);
                if (qos_stats.fwd_pkts | qos_stats.drp_cnt)
                {
    #if (CONFIG_MACH_PUMA6)
                    printk (" %5u | %10u | %10u | %s\n",index, qos_stats.fwd_pkts, qos_stats.drp_cnt, qname[index + PAL_CPPI41_SR_QPDSP_QOS_Q_BASE]);
    #else
                    printk (" %5u | %10u | %10u | %s\n",index, qos_stats.fwd_pkts, qos_stats.drp_cnt, "");
    #endif
                }
            }
            printk ("-------+------------+------------+--------------\n");

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
        unsigned int    lockKey;

        PAL_osProtectEntry(PAL_OSPROTECT_INTERRUPT, &lockKey);
        /* Get the number of sessions that are available. */
        num_session = ti_ppm_get_session (-1, 0, NULL);
        printk ("Detected %d sessions in packet processor\n", num_session);

        if (num_session == 0)
        {
            PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);
            return 0;
        }

        /* Allocate memory for the sessions. */
        session = (unsigned char *)kmalloc(sizeof(unsigned char)*num_session, GFP_KERNEL);
        if (session == NULL)
        {
            PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);
            return -1;
        }

        /* Get the Session Information - array of all the session indices */
        num_session = ti_ppm_get_session (-1, num_session, session);

        /* Cycle through all the VPID and get the stats */
        while (index < num_session)
        {
            /* Print the Session Information on the console. */
            printk("\n");
            printk("####################\n");
            printk("# Session %03d info #\n", session[index]);
            printk("####################\n\n");
            ti_hil_intrusive_display_session(session[index]);

            /* Goto the next Session */
            index = index + 1;
        }

        PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);

        /* Free the session buffer memory */
        kfree(session);

        /* Work is completed. */
        return 0;
    }

    if (strcmp(argv[1], "xSession") == 0)
    {
        Uint32 session = (Uint8)simple_strtol(argv[2], NULL, 0);

        printk("\n");
        printk("#############################\n");
        printk("# Session %03d extended info #\n", session);
        printk("#############################\n\n");

       ti_ppm_dispaly_session_info(session);

        /* Work is completed. */
        return 0;
    }

    if (strcmp(argv[1], "xQQ") == 0)
    {
        Uint32 queue = (Uint8)simple_strtol(argv[2], NULL, 0);

        printk("\n");
        printk("###############################\n");
        printk("# QoS Queue %03d extended info #\n", queue);
        printk("###############################\n\n");

       ti_ppm_dispaly_qos_queue_info(queue);

        /* Work is completed. */
        return 0;
    }

    if (strcmp(argv[1], "xQC") == 0)
    {
        Uint32 cluster = (Uint8)simple_strtol(argv[2], NULL, 0);

        printk("\n");
        printk("################################\n");
        printk("# QoS Cluster %02d extended info #\n", cluster);
        printk("################################\n\n");

       ti_ppm_dispaly_qos_cluster_info(cluster);

        /* Work is completed. */
        return 0;
    }

    /* cable_pp: add brief summary printing */
    if (strcmp(argv[1], "brief") == 0)
    {
        /* GLOBAL */
        {
            TI_PP_GLOBAL_STATS  pp_stats;

            /* YES. Get the global stats through the PPM */
            ti_ppm_get_global_stats(&pp_stats);

            /* Print the stats on the console. */
            printk ("Packets received in the PP   : %u\n", pp_stats.packets_rxed);
            printk ("Number of search attemps     : %u\n", pp_stats.packets_searched);
            printk ("Number of matched searches   : %u\n", pp_stats.search_matched);
            printk ("Number of Synch Delays       : %u\n", pp_stats.sync_delay);
            printk ("Packet forwarded by the PP   : %u\n", pp_stats.packets_fwd);
            printk ("IPv4 Packets Forwarded       : %u\n", pp_stats.ipv4_packets_fwd);
        }
        /* VPIDs */
        {
            TI_PP_VPID  vpid[TI_PP_MAX_PID + 1];
            int         num_vpid;
            int         index = 0;
            unsigned int lockKey;

            PAL_osProtectEntry(PAL_OSPROTECT_INTERRUPT, &lockKey);

            /* Get a list of all VPID that exist in the System */
            num_vpid = ti_ppm_get_vpid (-1, TI_PP_MAX_PID, &vpid[0]);

            /* Cycle through all the VPID and get the stats */
            while (index < num_vpid)
            {
                TI_PP_VPID_STATS   vpid_stats;

                /* Get the VPID statistics. */
                ti_ppm_get_vpid_stats(vpid[index].vpid_handle, &vpid_stats);
                printk ("VPID index=%02d, handle=%02d: Rx=%10d, Tx=%10d\n", index, vpid[index].vpid_handle,
                    vpid_stats.rx_unicast_pkt+vpid_stats.rx_broadcast_pkt+vpid_stats.rx_multicast_pkt,
                    vpid_stats.tx_unicast_pkt+vpid_stats.tx_broadcast_pkt+vpid_stats.tx_multicast_pkt);

                /* Goto the next VPID. */
                index = index + 1;
            }
            PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);
        }
        /* SESSIONs */
        {
            int               num_session;
            unsigned char*    session;
            int               index = 0;
            unsigned int lockKey;

            PAL_osProtectEntry(PAL_OSPROTECT_INTERRUPT, &lockKey);
            /* Get the number of sessions that are available. */
            num_session = ti_ppm_get_session (-1, 0, NULL);

            /* Allocate memory for the sessions. */
            session = (unsigned char *)kmalloc(sizeof(unsigned char)*num_session, GFP_KERNEL);
            if (session == NULL)
            {
                PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);
                return -1;
            }

            /* Get the Session Information - array of all the session indices */
            num_session = ti_ppm_get_session (-1, num_session, session);

            /* Cycle through all the VPID and get the stats */
            while (index < num_session)
            {

                TI_PP_SESSION_STATS   session_stats;

                /* Get the VPID statistics. */
                ti_ppm_get_session_stats(session[index], &session_stats);

                printk ("SESSION index=%02d, handle=%02d: Fwd=%10d\n", index, session[index],
                    session_stats.packets_forwarded);
                /* Goto the next Session */
                index = index + 1;
            }

            PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);

            /* Free the session buffer memory */
            kfree(session);
        }
        return 0;
    }

    if (strcmp(argv[1], "sz") == 0)
    {
        printk (" TI_PP_ETH_DESC:       %d \n", sizeof(TI_PP_ETH_DESC));
        printk (" TI_PP_IPV4_DESC:      %d \n", sizeof(TI_PP_IPV4_DESC));
        printk (" TI_PP_IPV6_DESC:      %d \n", sizeof(TI_PP_IPV6_DESC));
        printk (" TI_PP_APP_DESC:       %d \n", sizeof(TI_PP_APP_DESC));
        printk (" TI_PP_L2_RAW_DESC:    %d \n", sizeof(TI_PP_L2_RAW_DESC));
        printk (" TI_PP_PACKET_DESC:    %d \n", sizeof(TI_PP_PACKET_DESC));
        return 0;
    }

    /* Control comes here if the scope was not understood. */
    return -1;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_set_cmd_handler
 **************************************************************************
 * DESCRIPTION   :
 *  This function is the Packet Processor set command handler.
 *
 * RETURNS       :
 *  -1      - Error.
 *  0       - Success.
 ***************************************************************************/
static int ti_hil_set_cmd_handler(int argc, char* argv[])
{
    /* Validate the number of arguments that have been passed. */
    if (argc != 3)
    {
        printk ("ERROR: Incorrect Number of parameters passed.\n");
        return -1;
    }

    if (strcmp(argv[1], "timeout") == 0)
    {
        int tmp = (int) simple_strtol(argv[2], NULL, 0);

        if (tmp < 0)
            return -1;

        /* Convert the session timeout to usec units */
        ti_session_timeout = tmp * HZ * 1000;

        /* Work is completed. */
        return 0;
    }

    if (strcmp(argv[1], "threshold") == 0)
    {
        int threshold = (int) simple_strtol(argv[2], NULL, 0);

        if (threshold < 0)
            return -1;

        global_ti_hil_db.tdox_RTT_threshold_ms = threshold;

        /* Work is completed. */
        return 0;
    }

    if (strcmp(argv[1], "mtaAddress") == 0)
    {
        char    mtaAddress[6];
        int     i;

        for (i = 0; i < 6; i++)
        {
            mtaAddress[i] = (int) simple_strtol(argv[2], NULL, 16);
            argv[2] += 3;
        }

        ti_ppm_set_mta_mac_address(mtaAddress);

        /* Work is completed. */
        return 0;
    }

#ifdef CONFIG_INTEL_PP_TUNNEL_SUPPORT
    if (strcmp(argv[1], "tunnelMode") == 0)
    {
        int tunnelMode = (int) simple_strtol(argv[2], NULL, 0);

        global_ti_hil_db.tunnelMode = tunnelMode;

        ti_ppm_set_tunnel_mode(tunnelMode);

        /* Work is completed. */
        return 0;
    }

    if (strcmp(argv[1], "cmAddress") == 0)
    {
        char    cmAddress[6];
        int     i;

        for (i = 0; i < 6; i++)
        {
            cmAddress[i] = (int) simple_strtol(argv[2], NULL, 16);
            argv[2] += 3;
        }

        ti_ppm_set_cm_mac_address(cmAddress);

        /* Work is completed. */
        return 0;
    }
#endif

    if (strcmp(argv[1], "vpid") == 0)
    {
        struct net_device * dev = dev_get_by_name (&init_net, argv[2]);     
        if(dev)
        {
            ti_hil_pp_event (TI_PP_ADD_VPID, (void *)dev);
            dev_put(dev);
        }

        /* Work is completed. */
        return 0;
    }

    /* Control comes here if the command was not understood. */
    return -1;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_reset_cmd_handler
 **************************************************************************
 * DESCRIPTION   :
 *  This function is the Packet Processor reset command handler.
 *
 * RETURNS       :
 *  -1      - Error.
 *  0       - Success.
 ***************************************************************************/
static int ti_hil_reset_cmd_handler(int argc, char* argv[])
{
    int i = 0;

    /* Validate the number of arguments that have been passed. */
    if (argc < 2)
    {
        printk ("ERROR: Incorrect Number of parameters passed.\n");
        return -1;
    }

    /* Reset the session timeout to default, i.e., 30 seconds. */
    if (strcmp(argv[1], "timeout") == 0)
    {
        ti_session_timeout = 30 * HZ * 1000;

        /* Work is completed. */
        return 0;
    }

    /* Reset the HIL Analysis stats. */
    if (strcmp(argv[1], "stats") == 0)
    {
        /* Initialize the counters for the HIL Analysis. */
        for(i = 0; i < HIL_MAX_NUM_BUCKETS; i++)
            global_ti_hil_db.session_bucket[i] = 0;

        global_ti_hil_db.num_total_sessions = 0;
        global_ti_hil_db.num_error = 0;
        global_ti_hil_db.num_bypassed_pkts = 0;
        global_ti_hil_db.num_other_pkts = 0;
        global_ti_hil_db.num_ingress_pkts = 0;
        global_ti_hil_db.num_egress_pkts  = 0;
        global_ti_hil_db.num_null_drop_pkts = 0;

        /* Work is completed. */
        return 0;
    }
    if (strcmp(argv[1], "vpid") == 0)
    {
        struct net_device * dev = dev_get_by_name (&init_net, argv[2]);     
        if(dev)
        {
            ti_hil_pp_event (TI_PP_REMOVE_VPID, (void *)dev);
            dev_put(dev);
        }

        /* Work is completed. */
        return 0;
    }

    /* Control comes here if the command was not understood. */
    return -1;
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
    char    pp_cmd[100];
    char*   argv[10];
    int     argc = 0;
    char*   ptr_cmd;
    char*   delimitters = " \n\t";
    char*   ptr_next_tok;

    /* Validate the length of data passed. */
    if (count > 100)
        count = 100;

    /* Initialize the buffer before using it. */
    memset ((void *)&pp_cmd[0], 0, sizeof(pp_cmd));
    memset ((void *)&argv[0], 0, sizeof(argv));

    /* Copy from user space. */
    if (copy_from_user (&pp_cmd, buffer, count))
        return -EFAULT;

    ptr_next_tok = &pp_cmd[0];

    /* Tokenize the command. Check if there was a NULL entry. If so be the case the
     * user did not know how to use the entry. Print the help screen. */
    ptr_cmd = strsep(&ptr_next_tok, delimitters);
    if (ptr_cmd == NULL)
        return -1;

    /* Parse all the commands typed. */
    do
    {
        /* Extract the first command. */
        argv[argc++] = ptr_cmd;

        /* Validate if the user entered more commands.*/
        if (argc >=10)
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
            return -1;
    }

    /* Deinitialize Command Handlers */
    else if (strncmp(argv[0], "deinit", strlen("deinit")) == 0)
    {
        /* Deinitialize all the packet processor components. */
        if (ti_ppm_deinitialize() < 0)
        {
            printk ("Error: Packet Processor Deinitialization Failed\n");
            return -1;
        }
    }

    /* Set Command Handlers */
    else if (strncmp(argv[0], "set", strlen("set")) == 0)
    {
        /* Call the Set Command Handler. */
        if (ti_hil_set_cmd_handler (argc, argv) < 0)
            return -1;
    }

    /* Deinitialize Command Handlers */
    else if (strncmp(argv[0], "reset", strlen("reset")) == 0)
    {
        /* Call the Reset Command Handler. */
        if (ti_hil_reset_cmd_handler (argc, argv) < 0)
            return -1;
    }

    /* cable_pp: disable/enable capability */
    else if (strcmp(argv[0], "enable") == 0)
    {
        global_ti_hil_db.hil_disabled = 0;
    }

    else if (strcmp(argv[0], "disable") == 0)
    {
        global_ti_hil_db.hil_disabled = 1;
        ti_ppm_flush_sessions(-1);
    }

    else if (strcmp(argv[0], "dbg") == 0)
    {
        global_ti_hil_db.dbg_disabled = 0;
    }

    else if (strcmp(argv[0], "nodbg") == 0)
    {
        global_ti_hil_db.dbg_disabled = 1;
    }

    else if (strcmp(argv[0], "tdox") == 0)
    {
        global_ti_hil_db.tdox_disabled = 0;
    }

    else if (strcmp(argv[0], "notdox") == 0)
    {
        global_ti_hil_db.tdox_disabled = 1;
    }
    else if (strcmp(argv[0], "ackSupp") == 0)
    {  
        char pram = *(argv[1]);
        int enDis = 0;
        if(pram == '1') 
        {
            enDis = 1;
        }
        ti_ppm_set_ack_suppression(enDis);
    }
    else if ((strcmp(argv[0], "qos") == 0) && (1 == global_ti_hil_db.qos_disabled) )
    {
        unsigned int i;
        unsigned int lockKey = 0;

        PAL_osProtectEntry(PAL_OSPROTECT_INTERRUPT, &lockKey);

        ti_ppm_flush_sessions(-1);
        // go over all devices and setup QoS
        for( i = 0; i < TI_MAX_DEVICE_INDEX; i++ )
        {
            struct net_device *cur_dev = 0;
            cur_dev = dev_get_by_index(&init_net, i);
            if( !cur_dev )
                continue;

            if (-1 == cur_dev->vpid_handle)
            {
                dev_put(cur_dev);
                continue;
            }

            if (NULL != cur_dev->qos_setup_hook)
            {
                cur_dev->qos_setup_hook( cur_dev );
            }
            dev_put(cur_dev);
        }

        PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);
    }

    else if ((strcmp(argv[0], "noqos") == 0) && (0 == global_ti_hil_db.qos_disabled) )
    {
        unsigned int i;
        unsigned int lockKey = 0;

        PAL_osProtectEntry(PAL_OSPROTECT_INTERRUPT, &lockKey);

        ti_ppm_flush_sessions(-1);
        // go over all devices and shutdown QoS
        for( i = 0; i < TI_MAX_DEVICE_INDEX; i++ )
        {
            struct net_device *cur_dev = 0;
            cur_dev = dev_get_by_index(&init_net, i);
            if( !cur_dev )
                continue;

            if (-1 == cur_dev->vpid_handle)
            {
                dev_put(cur_dev);
                continue;
            }

            if (NULL != cur_dev->qos_shutdown_hook)
            {
                cur_dev->qos_shutdown_hook( cur_dev );
            }
            dev_put(cur_dev);
        }

        PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);
    }

    else if (strcmp(argv[0], "psm") == 0)
    {
        ti_ppm_flush_sessions(-1);
        ti_ppm_enable_psm();
    }

    else if (strcmp(argv[0], "nopsm") == 0)
    {
        ti_ppm_disable_psm();
    }

    else if (strcmp(argv[0], "flush_all_sessions") == 0)
    {
        /* Call flush sessions API with -1 */
        ti_ppm_flush_sessions(-1);
    }

#ifdef HIL_EXAMPLE_DO_NOT_DELETE
    else if (strcmp(argv[0], "gre") == 0)
    {
        ti_hil_test_gre_session_creation();
    }
#endif

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
        /* PDSP dont seem to be working correctly... This is a FATAL Condition and
         * needs to be handled... We cause the system to crash here as an indication
         * that this needs to be addressed. System Profiles need to handle this in
         * a more robust manner. */
        printk ("------- FATAL Error: Packet Processor PDSP is not healthy ------- \n");
        BUG();
        return;
    }

    /* The PDSP passed the health check; restart the timer */
    pdsp_health_timer.expires = jiffies + PDSP_HEALTH_TIMER*HZ;
    add_timer (&pdsp_health_timer);
    return;
}

int ti_hil_read_devs(char* buf, char **start, off_t offset, int count,
                 int *eof, void *data)
{
    unsigned int i;
    int len=0;
    unsigned int limit = count - 80;

#ifndef TI_MAX_DEVICE_INDEX
#define TI_MAX_DEVICE_INDEX 64
#endif

    for( i = 0; i < TI_MAX_DEVICE_INDEX; i++ )
    {
         struct net_device *cur_dev = 0;
         TI_PP_VPID_STATS   vpid_stats;

         cur_dev = dev_get_by_index(&init_net, i);
         if( !cur_dev )
              continue;

         /* Get the VPID statistics. */
         if( ti_ppm_get_vpid_stats( cur_dev->vpid_handle , &vpid_stats) < 0 )
         {
              dev_put(cur_dev);
              continue;
         }

         /* Print the statistics on the console. */
         if( len < limit )
             len += sprintf(buf + len, "   /dev/%s: vpid=%d pid=%d \n", cur_dev->name, cur_dev->vpid_handle, cur_dev->pid_handle );
         if( len < limit )
             len += sprintf(buf + len, "-----------------------------------------\n");
         if( len < limit )
             len += sprintf(buf + len, "Rx Unicast   Packets: %u\n", vpid_stats.rx_unicast_pkt);
         if( len < limit )
             len += sprintf(buf + len, "Rx Broadcast Packets: %u\n", vpid_stats.rx_broadcast_pkt);
         if( len < limit )
             len += sprintf(buf + len, "Rx Multicast Packets: %u\n", vpid_stats.rx_multicast_pkt);
         if( len < limit )
             len += sprintf(buf + len, "Rx Bytes            : 0x%08X%08X\n", vpid_stats.rx_byte_hi, vpid_stats.rx_byte_lo);
         if( len < limit )
             len += sprintf(buf + len, "Rx Bytes - low      : %u\n", vpid_stats.rx_byte_lo);
         if( len < limit )
             len += sprintf(buf + len, "Rx Discard          : %u\n", vpid_stats.rx_discard);
         if( len < limit )
             len += sprintf(buf + len, "Tx Unicast   Packets: %u\n", vpid_stats.tx_unicast_pkt);
         if( len < limit )
             len += sprintf(buf + len, "Tx Broadcast Packets: %u\n", vpid_stats.tx_broadcast_pkt);
         if( len < limit )
             len += sprintf(buf + len, "Tx Multicast Packets: %u\n", vpid_stats.tx_multicast_pkt);
         if( len < limit )
             len += sprintf(buf + len, "Tx Bytes            : 0x%08X%08X\n", vpid_stats.tx_byte_hi, vpid_stats.tx_byte_lo);
         if( len < limit )
             len += sprintf(buf + len, "Tx Bytes - low      : %u\n", vpid_stats.tx_byte_lo);
         if( len < limit )
             len += sprintf(buf + len, "Tx Errors           : %u\n", vpid_stats.tx_error);
         if( len < limit )
             len += sprintf(buf + len, "Tx Discards         : %u\n\n", vpid_stats.tx_discard);
         dev_put(cur_dev);
    }

    return len;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_intrusive_init
 **************************************************************************
 * DESCRIPTION   :
 *  Initialization function for the Intrusive mode profile.
 *
 * RETURNS       :
 *  0   -   Success
 *  <0  -   Error
 **************************************************************************/
static int ti_hil_intrusive_init (void)
{
    struct proc_dir_entry*  ptr_dir_entry;
    struct proc_dir_entry*  ptr_dev_entry;
    int                     index = 0;

    /* Register an event handler to listen to events. */
    ppsubsystem_event_handler = ti_ppm_register_event_handler (ti_hil_ppsubsystem_event_handler);
    if (ppsubsystem_event_handler == 0)
    {
        printk ("Error: Event Handler register failed\n");
        return -1;
    }

    /* Create the PROC Entry used by the TCA Configuration Engine. */
    ptr_dir_entry = create_proc_entry("ti_pp" ,0644, init_net.proc_net);
    ptr_dev_entry = create_proc_entry("ti_pp_dev" ,0644, init_net.proc_net);

    if( (ptr_dir_entry == NULL) || (ptr_dev_entry == NULL ) )
    {
        printk ("Error: Unable to create Packet Processor proc entry.\n");
        return -1;
    }
    ptr_dir_entry->data      = NULL;
    ptr_dir_entry->read_proc  = NULL;
    ptr_dir_entry->write_proc = ti_hil_write_cmds;


    ptr_dev_entry->data      = NULL;
    ptr_dev_entry->read_proc  = ti_hil_read_devs;
    ptr_dev_entry->write_proc = NULL;;

    /* Create a timer to poll for the health */
    init_timer (&pdsp_health_timer);
    pdsp_health_timer.function = ti_hil_health_timer_expired;
    pdsp_health_timer.data     = 0;

    /* Start the timer. */
    pdsp_health_timer.expires = jiffies + PDSP_HEALTH_TIMER*HZ;
    add_timer (&pdsp_health_timer);

    /* Initialize the counters for the HIL Analysis. */
    for(index = 0; index < HIL_MAX_NUM_BUCKETS; index++)
        global_ti_hil_db.session_bucket[index] = 0;

#ifdef CONFIG_NETFILTER
    /* Initialize the HIL Session to Connection Tracking Mapper. */
    memset ((void *)&hil_session_ct_mapper[0], 0, sizeof(hil_session_ct_mapper));
#endif /* CONFIG_NETFILTER */

#ifdef CONFIG_IP_MULTICAST
    /* Initialize the MFC to Session Mapper. */
    memset ((void *)&hil_mfc_session_mapper[0], 0, sizeof(hil_mfc_session_mapper));;
#endif /* CONFIG_IP_MULTICAST */

    ti_hil_tdox_init();

    /* Initialize PP Path */
    PPP_Init();

    AVALANCHE_PERF_MON_COUNTER_ENABLE();

    /* Profile has been successfully initialized. */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_intrusive_deinit
 **************************************************************************
 * DESCRIPTION   :
 *  Deinitialization function which deinitializes and unregisters the default
 *  profile with the HIL Core.
 *
 * RETURNS       :
 *  0   -   Success
 *  <0  -   Error
 **************************************************************************/
static int ti_hil_intrusive_deinit(void)
{
    return 0;
}


/**************************************************************************
 * FUNCTION NAME : ti_hil_enable_psm
 **************************************************************************
 * DESCRIPTION   :
 *  Enable Power Saving Mode (PSM) API
 *
 * RETURNS       :
 *  0   -   Success
 *  <0  -   Error
 **************************************************************************/
int ti_hil_enable_psm (void)
{
    /* Delete old sessions in the system */
    printk(KERN_INFO "%s: Flush all old sessions\n", __FUNCTION__);
    if (ti_ppm_flush_sessions(-1) < 0)
    {
        printk ("Error: Unable to flush all sessions\n");
        return -1;
    }

    /* Call PPM enable PSM */
    return ti_ppm_enable_psm();
}

/**************************************************************************
 * FUNCTION NAME : ti_hil_disable_psm
 **************************************************************************
 * DESCRIPTION   :
 *  Disable Power Saving Mode (PSM) API
 *
 * RETURNS       :
 *  0   -   Success
 *  <0  -   Error
 **************************************************************************/
int ti_hil_disable_psm (void)
{
    /* Call PPM disable PSM */
    if (ti_ppm_disable_psm() < 0)
    {
        printk ("%s: Error: Unable to disable PP PSM\n", __FUNCTION__);
        return -1;
    }

    return 0;
}

#ifdef HIL_EXAMPLE_DO_NOT_DELETE /* Please do not delete - this is an example of how to open a session without sending real traffic */
Uint8 usPktDataIngress[] =
{
    0x00, 0x0D, 0xBC, 0x20, 0xB1, 0xC5, // ETH DA
    0x00, 0x0D, 0xBC, 0x20, 0xB1, 0x0E, // ETH SA
    0x08, 0x00,                         // ETH Type
    0x45, 0x00,                         // IP Version/Header Length, IP TOS
    0x03, 0xEE,                         // IP Total Length - updated per packet
    0x00, 0x00, 0x00, 0x00,             // IP Identification, IP Fragment
    0x40,                               // IP TTL
    0x11,                               // IP Protocol - UDP
    0x0D, 0xA9,                         // IP Checksum
    0x0A, 0x64, 0x2B, 0xC7,             // IP SA - 10.100.43.199
    0x0A, 0x64, 0x28, 0xC8,             // IP DA - 10.100.40.200
    0x00, 0x40, 0x07, 0xD0,             // UDP SRC Port (64), UDP DST Port (2000)
    0x03, 0xDA,                         // UDP Length
    0x00, 0x00,                         // UDP Checksum
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // Payload  
};

Uint8 usPktDataEgress[] =
{
    0x00, 0x0D, 0xBC, 0x20, 0xB1, 0xC5, // ETH DA
    0x00, 0x0D, 0xBC, 0x20, 0xB1, 0x0E, // ETH SA
    0x08, 0x00,                         // ETH Type
    0x45, 0x00,                         // IP Version/Header Length, IP TOS
    0x03, 0xEE,                         // IP Total Length - updated per packet
    0x00, 0x00, 0x00, 0x00,             // IP Identification, IP Fragment
    0x40,                               // IP TTL
    0x2F,                               // IP Protocol - UDP
    0x0D, 0x8B,                         // IP Checksum
    0x0A, 0x64, 0x2B, 0xC7,             // IP SA - 10.100.43.199
    0x0A, 0x64, 0x28, 0xC8,             // IP DA - 10.100.40.200
    0x00, 0x00, 0x65, 0x58,             // GRE header
                                        
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, // ETH DA
    0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, // ETH SA
    0x08, 0x00,                         // ETH Type
    0x45, 0x00,                         // IP Version/Header Length, IP TOS
    0x03, 0xD6,                         // IP Total Length - updated per packet
    0x00, 0x00, 0x00, 0x00,             // IP Identification, IP Fragment
    0x40,                               // IP TTL
    0x11,                               // IP Protocol - UDP
    0xE4, 0xBF,                         // IP Checksum
    0x64, 0x64, 0x64, 0xC7,             // IP SA - 100.100.100.199
    0x64, 0x64, 0x64, 0xC8,             // IP DA - 100.100.100.200
    0x00, 0x40, 0x07, 0xD0,             // UDP SRC Port (64), UDP DST Port (2000)
    0x03, 0xC2,                         // UDP Length
    0x00, 0x00,                         // UDP Checksum
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // Payload  
};

Uint8 dsPktDataIngress[] =
{
    0x00, 0x0D, 0xBC, 0x20, 0xB1, 0x0E, // ETH DA
    0x00, 0x00, 0x00, 0x06, 0x16, 0x01, // ETH SA
    0x08, 0x00,                         // ETH Type
    0x45, 0x00,                         // IP Version/Header Length, IP TOS
    0x03, 0xEE,                         // IP Total Length - updated per packet
    0x00, 0x00, 0x00, 0x00,             // IP Identification, IP Fragment
    0x40,                               // IP TTL
    0x2F,                               // IP Protocol - UDP
    0x36, 0x9F,                         // IP Checksum
    0x0A, 0x64, 0x28, 0xC8,             // IP SA - 10.100.40.200
    0x0A, 0x64, 0x2B, 0xC7,             // IP DA - 10.100.43.199
    0x00, 0x00, 0x65, 0x58,             // GRE header

    0x00, 0x0D, 0xBC, 0x20, 0xB1, 0x0E, // ETH DA
    0x00, 0x00, 0x00, 0x06, 0x16, 0x01, // ETH SA
    0x08, 0x00,                         // ETH Type
    0x45, 0x00,                         // IP Version/Header Length, IP TOS
    0x03, 0xD6,                         // IP Total Length - updated per packet
    0x00, 0x00, 0x00, 0x00,             // IP Identification, IP Fragment
    0x40,                               // IP TTL
    0x11,                               // IP Protocol - UDP
    0x0D, 0xC1,                         // IP Checksum
    0x0A, 0x64, 0x28, 0xC8,             // IP SA - 10.100.40.200
    0x0A, 0x64, 0x2B, 0xC7,             // IP DA - 10.100.43.199
    0x00, 0x40, 0x07, 0xD0,             // UDP SRC Port (40), UDP DST Port (2000)
    0x03, 0xC2,                         // UDP Length
    0x87, 0x03,                         // UDP Checksum
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // Payload  
};

Uint8 dsPktDataEgress[] =
{
    0x00, 0x0D, 0xBC, 0x20, 0xB1, 0x0E, // ETH DA
    0x00, 0x00, 0x00, 0x06, 0x16, 0x01, // ETH SA
    0x08, 0x00,                         // ETH Type
    0x45, 0x00,                         // IP Version/Header Length, IP TOS
    0x03, 0xD6,                         // IP Total Length - updated per packet
    0x00, 0x00, 0x00, 0x00,             // IP Identification, IP Fragment
    0x40,                               // IP TTL
    0x11,                               // IP Protocol - UDP
    0x0D, 0xC1,                         // IP Checksum
    0x0A, 0x64, 0x28, 0xC8,             // IP SA - 10.100.40.200
    0x0A, 0x64, 0x2B, 0xC7,             // IP DA - 10.100.43.199
    0x00, 0x40, 0x07, 0xD0,             // UDP SRC Port (40), UDP DST Port (2000)
    0x03, 0xC2,                         // UDP Length
    0x87, 0x03,                         // UDP Checksum
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // Payload  
};

void ti_hil_test_gre_session_creation(void)
{
    struct sk_buff *skb;
    struct net_device *cniDev;
    struct net_device *ethDev;
    char cniVpid;
    char ethVpid;
    unsigned char tempVpid;
    TI_PP_VPID vpid;
    TI_PP_PID   pid;
    TI_PP_SESSION       *ptr_session;

    cniVpid = -1;
    ethVpid = -1;
    for (tempVpid = 0; tempVpid < TI_PP_MAX_VPID; tempVpid++)
    {
        ti_ppm_get_vpid_info (tempVpid, &vpid);
        ti_ppm_get_pid_info  (vpid.parent_pid_handle, &pid);
        if (TI_PP_PID_TYPE_DOCSIS == pid.type)
        {
            cniVpid = tempVpid;
            if (ethVpid != 0xFF)
            {
                break;
            }
        }
        // TBD: when several ethernet VPID will be configured, need to know which is relevant
        if (TI_PP_PID_TYPE_ETHERNET == pid.type)
        {
            ethVpid = tempVpid;
            if (cniVpid != 0xFF)
            {
                break;
            }
        }
    }


    cniDev = __dev_get_by_name(&init_net, "cni0");
#if (CONFIG_MACH_PUMA6)
    ethDev = __dev_get_by_name(&init_net, L2SW_NETDEV_DATA0);
#else
    ethDev = __dev_get_by_name(&init_net, "eth0");
#endif

    if(!(skb = dev_alloc_skb(2048)))
    {
        printk("ti_hil_test_gre_session_creation: Failed to allocate skb\n");
        return;
    }

    skb->mac_header = skb->data;

    /********************/
    /* US ingress packet*/
    /********************/
    skb->skb_iif = ethDev->ifindex;
    skb->dev = ethDev;
    ptr_session = &skb->pp_packet_info.ti_session;
    ptr_session->ingress.vpid_handle = ethVpid;
    ptr_session->egress[0].vpid_handle = cniVpid;
    memcpy(skb->data, usPktDataIngress, sizeof(usPktDataIngress));
    ti_hil_ingress_hook(skb);

    /*******************/
    /* US egress packet*/
    /*******************/
    skb->skb_iif = cniDev->ifindex;
    skb->dev = cniDev;
    memcpy(skb->data, usPktDataEgress, sizeof(usPktDataEgress));
    ti_hil_egress_hook(skb);

    /********************/
    /* DS ingress packet*/
    /********************/
    skb->skb_iif = cniDev->ifindex;
    skb->dev = cniDev;
    memset((Uint8*)&skb->pp_packet_info.ti_session, 0, sizeof(skb->pp_packet_info.ti_session));
    ptr_session = &skb->pp_packet_info.ti_session;
    ptr_session->ingress.vpid_handle = cniVpid;
    ptr_session->egress[0].vpid_handle = ethVpid;
    memcpy(skb->data, dsPktDataIngress, sizeof(dsPktDataIngress));
    ti_hil_ingress_hook(skb);

    /*******************/
    /* DS egress packet*/
    /*******************/
    skb->skb_iif = ethDev->ifindex;
    skb->dev = ethDev;
    memcpy(skb->data, dsPktDataEgress, sizeof(dsPktDataEgress));
    ti_hil_egress_hook(skb);

    /* Change sessions state to forwarding and set session serial number */
#if (CONFIG_MACH_PUMA6)
    *(Uint16*)((Uint32)(((Uint32)IO_ADDRESS(0x03300900)) + (255 * 2))) = 0x40;
    *(Uint16*)((Uint32)(((Uint32)IO_ADDRESS(0x03300900)) + (254 * 2))) = 0x41;
#else
    *(Uint16*)((Uint32)(((Uint32)IO_ADDRESS(0x03100900)) + (255 * 2))) = 0x40;
    *(Uint16*)((Uint32)(((Uint32)IO_ADDRESS(0x03100900)) + (254 * 2))) = 0x41;
#endif
}
#endif
 
EXPORT_SYMBOL(ti_hil_enable_psm);
EXPORT_SYMBOL(ti_hil_disable_psm);
EXPORT_SYMBOL(ti_hil_set_mta_mac_address);
#ifdef CONFIG_INTEL_PP_TUNNEL_SUPPORT
EXPORT_SYMBOL(ti_hil_create_tunnel);
EXPORT_SYMBOL(ti_hil_delete_tunnel);
EXPORT_SYMBOL(ti_hil_set_tunnel_mode);
EXPORT_SYMBOL(ti_hil_set_cm_mac_address);
#endif

