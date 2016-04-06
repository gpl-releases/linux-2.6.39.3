/*
 * RNDIS MSG parser
 *
 * Authors:	Benedikt Spranger, Pengutronix
 *		Robert Schwebel, Pengutronix
 *
 *              This program is free software; you can redistribute it and/or
 *              modify it under the terms of the GNU General Public License
 *              version 2, as published by the Free Software Foundation.
 *
 *		This software was originally developed in conformance with
 *		Microsoft's Remote NDIS Specification License Agreement.
 *
 * 03/12/2004 Kai-Uwe Bloem <linux-development@auerswald.de>
 *		Fixed message length bug in init_response
 *
 * 03/25/2004 Kai-Uwe Bloem <linux-development@auerswald.de>
 *		Fixed rndis_rm_hdr length bug.
 *
 * Copyright (C) 2004 by David Brownell
 *		updates to merge with Linux 2.6, better match RNDIS spec
 */

/* Copyright 2008, Texas Instruments Incorporated
 *
 * This program has been modified from its original operation by 
 * Texas Instruments to do the following:
 * 
 * Explanation of modification:
 *  fixes/changes from Puma5
 *  
 *
 * THIS MODIFIED SOFTWARE AND DOCUMENTATION ARE PROVIDED
 * "AS IS," AND TEXAS INSTRUMENTS MAKES NO REPRESENTATIONS
 * OR WARRENTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO, WARRANTIES OF MERCHANTABILITY OR FITNESS FOR ANY
 * PARTICULAR PURPOSE OR THAT THE USE OF THE SOFTWARE OR
 * DOCUMENTATION WILL NOT INFRINGE ANY THIRD PARTY PATENTS,
 * COPYRIGHTS, TRADEMARKS OR OTHER RIGHTS.
 * See The GNU General Public License for more details.
 *
 * These changes are covered under version 2 of the GNU General Public License,
 * dated June 1991.
 */
 
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>
#include <linux/seq_file.h>
#include <linux/netdevice.h>

#include <asm/io.h>
#include <asm/byteorder.h>
#include <asm/system.h>
#include <asm/unaligned.h>


#undef	VERBOSE_DEBUG

#include "rndis.h"


/* The driver for your USB chip needs to support ep0 OUT to work with
 * RNDIS, plus all three CDC Ethernet endpoints (interrupt not optional).
 *
 * Windows hosts need an INF file like Documentation/usb/linux.inf
 * and will be happier if you provide the host_addr module parameter.
 */

#if 0
static int rndis_debug = 0;
module_param (rndis_debug, int, 0);
MODULE_PARM_DESC (rndis_debug, "enable debugging");
#else
#define rndis_debug		0
#endif

#define RNDIS_MAX_CONFIGS	1


static rndis_params rndis_per_dev_params[RNDIS_MAX_CONFIGS];

/* Driver Version */
static const __le32 rndis_driver_version = cpu_to_le32(1);

/* Function Prototypes */
static rndis_resp_t *rndis_add_response(int configNr, u32 length);


/* supported OIDs */
static const u32 oid_supported_list[] =
{
	/* the general stuff */
	OID_GEN_SUPPORTED_LIST,
	OID_GEN_HARDWARE_STATUS,
	OID_GEN_MEDIA_SUPPORTED,
	OID_GEN_MEDIA_IN_USE,
	OID_GEN_MAXIMUM_FRAME_SIZE,
	OID_GEN_LINK_SPEED,
	OID_GEN_TRANSMIT_BLOCK_SIZE,
	OID_GEN_RECEIVE_BLOCK_SIZE,
	OID_GEN_VENDOR_ID,
	OID_GEN_VENDOR_DESCRIPTION,
	OID_GEN_VENDOR_DRIVER_VERSION,
	OID_GEN_CURRENT_PACKET_FILTER,
	OID_GEN_MAXIMUM_TOTAL_SIZE,
	OID_GEN_MEDIA_CONNECT_STATUS,
	OID_GEN_PHYSICAL_MEDIUM,
#ifdef CONFIG_MACH_PUMA5
	OID_GEN_RNDIS_CONFIG_PARAMETER,
#endif

	/* the statistical stuff */
	OID_GEN_XMIT_OK,
	OID_GEN_RCV_OK,
	OID_GEN_XMIT_ERROR,
	OID_GEN_RCV_ERROR,
	OID_GEN_RCV_NO_BUFFER,
#ifdef	RNDIS_OPTIONAL_STATS
	OID_GEN_DIRECTED_BYTES_XMIT,
	OID_GEN_DIRECTED_FRAMES_XMIT,
	OID_GEN_MULTICAST_BYTES_XMIT,
	OID_GEN_MULTICAST_FRAMES_XMIT,
	OID_GEN_BROADCAST_BYTES_XMIT,
	OID_GEN_BROADCAST_FRAMES_XMIT,
	OID_GEN_DIRECTED_BYTES_RCV,
	OID_GEN_DIRECTED_FRAMES_RCV,
	OID_GEN_MULTICAST_BYTES_RCV,
	OID_GEN_MULTICAST_FRAMES_RCV,
	OID_GEN_BROADCAST_BYTES_RCV,
	OID_GEN_BROADCAST_FRAMES_RCV,
	OID_GEN_RCV_CRC_ERROR,
	OID_GEN_TRANSMIT_QUEUE_LENGTH,
#endif	/* RNDIS_OPTIONAL_STATS */

	/* mandatory 802.3 */
	/* the general stuff */
	OID_802_3_PERMANENT_ADDRESS,
	OID_802_3_CURRENT_ADDRESS,
	OID_802_3_MULTICAST_LIST,
	OID_802_3_MAC_OPTIONS,
	OID_802_3_MAXIMUM_LIST_SIZE,

	/* the statistical stuff */
	OID_802_3_RCV_ERROR_ALIGNMENT,
	OID_802_3_XMIT_ONE_COLLISION,
	OID_802_3_XMIT_MORE_COLLISIONS,
#ifdef	RNDIS_OPTIONAL_STATS
	OID_802_3_XMIT_DEFERRED,
	OID_802_3_XMIT_MAX_COLLISIONS,
	OID_802_3_RCV_OVERRUN,
	OID_802_3_XMIT_UNDERRUN,
	OID_802_3_XMIT_HEARTBEAT_FAILURE,
	OID_802_3_XMIT_TIMES_CRS_LOST,
	OID_802_3_XMIT_LATE_COLLISIONS,
#endif	/* RNDIS_OPTIONAL_STATS */

#ifdef	RNDIS_PM
	/* PM and wakeup are "mandatory" for USB, but the RNDIS specs
	 * don't say what they mean ... and the NDIS specs are often
	 * confusing and/or ambiguous in this context.  (That is, more
	 * so than their specs for the other OIDs.)
	 *
	 * FIXME someone who knows what these should do, please
	 * implement them!
	 */

	/* power management */
	OID_PNP_CAPABILITIES,
	OID_PNP_QUERY_POWER,
	OID_PNP_SET_POWER,

#ifdef	RNDIS_WAKEUP
	/* wake up host */
	OID_PNP_ENABLE_WAKE_UP,
	OID_PNP_ADD_WAKE_UP_PATTERN,
	OID_PNP_REMOVE_WAKE_UP_PATTERN,
#endif	/* RNDIS_WAKEUP */
#endif	/* RNDIS_PM */
};

#ifdef CONFIG_MACH_PUMA5
static void string_to_unicode (char *string, u16 *unicode_string, u16 *length);
static u8 unicode_to_macnibble (u16 value);
static int rndis_indicate_status_msg (int configNr, u32 status);
#endif

/* NDIS Functions */
static int gen_ndis_query_resp(int configNr, u32 OID, u8 *buf,
			       unsigned buf_len, rndis_resp_t *r)
{
	int retval = -ENOTSUPP;
	u32 length = 4;	/* usually */
	__le32 *outbuf;
	int i, count;
	rndis_query_cmplt_type *resp;
	struct net_device *net;
	struct rtnl_link_stats64 temp;
	const struct rtnl_link_stats64 *stats;

	if (!r) return -ENOMEM;
	resp = (rndis_query_cmplt_type *)r->buf;

	if (!resp) return -ENOMEM;

	if (buf_len && rndis_debug > 1) {
		pr_debug("query OID %08x value, len %d:\n", OID, buf_len);
		for (i = 0; i < buf_len; i += 16) {
			pr_debug("%03d: %08x %08x %08x %08x\n", i,
				get_unaligned_le32(&buf[i]),
				get_unaligned_le32(&buf[i + 4]),
				get_unaligned_le32(&buf[i + 8]),
				get_unaligned_le32(&buf[i + 12]));
		}
	}

	/* response goes here, right after the header */
	outbuf = (__le32 *)&resp[1];
	resp->InformationBufferOffset = cpu_to_le32(16);

	net = rndis_per_dev_params[configNr].dev;
	stats = dev_get_stats(net, &temp);

	switch (OID) {

	/* general oids (table 4-1) */

	/* mandatory */
	case OID_GEN_SUPPORTED_LIST:
		pr_debug("%s: OID_GEN_SUPPORTED_LIST\n", __func__);
		length = sizeof(oid_supported_list);
		count  = length / sizeof(u32);
		for (i = 0; i < count; i++)
			outbuf[i] = cpu_to_le32(oid_supported_list[i]);
		retval = 0;
		break;

	/* mandatory */
	case OID_GEN_HARDWARE_STATUS:
		pr_debug("%s: OID_GEN_HARDWARE_STATUS\n", __func__);
		/* Bogus question!
		 * Hardware must be ready to receive high level protocols.
		 * BTW:
		 * reddite ergo quae sunt Caesaris Caesari
		 * et quae sunt Dei Deo!
		 */
		*outbuf = cpu_to_le32(0);
		retval = 0;
		break;

	/* mandatory */
	case OID_GEN_MEDIA_SUPPORTED:
		pr_debug("%s: OID_GEN_MEDIA_SUPPORTED\n", __func__);
		*outbuf = cpu_to_le32(rndis_per_dev_params[configNr].medium);
		retval = 0;
		break;

	/* mandatory */
	case OID_GEN_MEDIA_IN_USE:
		pr_debug("%s: OID_GEN_MEDIA_IN_USE\n", __func__);
		/* one medium, one transport... (maybe you do it better) */
		*outbuf = cpu_to_le32(rndis_per_dev_params[configNr].medium);
		retval = 0;
		break;

	/* mandatory */
	case OID_GEN_MAXIMUM_FRAME_SIZE:
		pr_debug("%s: OID_GEN_MAXIMUM_FRAME_SIZE\n", __func__);
		if (rndis_per_dev_params[configNr].dev) {
			*outbuf = cpu_to_le32(
				rndis_per_dev_params[configNr].dev->mtu);
			retval = 0;
		}
		break;

	/* mandatory */
	case OID_GEN_LINK_SPEED:
		if (rndis_debug > 1)
			pr_debug("%s: OID_GEN_LINK_SPEED\n", __func__);
		if (rndis_per_dev_params[configNr].media_state
				== NDIS_MEDIA_STATE_DISCONNECTED)
			*outbuf = cpu_to_le32(0);
		else
			*outbuf = cpu_to_le32(
				rndis_per_dev_params[configNr].speed);
		retval = 0;
		break;

	/* mandatory */
	case OID_GEN_TRANSMIT_BLOCK_SIZE:
		pr_debug("%s: OID_GEN_TRANSMIT_BLOCK_SIZE\n", __func__);
		if (rndis_per_dev_params[configNr].dev) {
			*outbuf = cpu_to_le32(
				rndis_per_dev_params[configNr].dev->mtu);
			retval = 0;
		}
		break;

	/* mandatory */
	case OID_GEN_RECEIVE_BLOCK_SIZE:
		pr_debug("%s: OID_GEN_RECEIVE_BLOCK_SIZE\n", __func__);
		if (rndis_per_dev_params[configNr].dev) {
			*outbuf = cpu_to_le32(
				rndis_per_dev_params[configNr].dev->mtu);
			retval = 0;
		}
		break;

	/* mandatory */
	case OID_GEN_VENDOR_ID:
		pr_debug("%s: OID_GEN_VENDOR_ID\n", __func__);
		*outbuf = cpu_to_le32(
			rndis_per_dev_params[configNr].vendorID);
		retval = 0;
		break;

	/* mandatory */
	case OID_GEN_VENDOR_DESCRIPTION:
		pr_debug("%s: OID_GEN_VENDOR_DESCRIPTION\n", __func__);
		if (rndis_per_dev_params[configNr].vendorDescr) {
			length = strlen(rndis_per_dev_params[configNr].
					vendorDescr);
			memcpy(outbuf,
				rndis_per_dev_params[configNr].vendorDescr,
				length);
		} else {
			outbuf[0] = 0;
		}
		retval = 0;
		break;

	case OID_GEN_VENDOR_DRIVER_VERSION:
		pr_debug("%s: OID_GEN_VENDOR_DRIVER_VERSION\n", __func__);
		/* Created as LE */
		*outbuf = rndis_driver_version;
		retval = 0;
		break;

	/* mandatory */
	case OID_GEN_CURRENT_PACKET_FILTER:
		pr_debug("%s: OID_GEN_CURRENT_PACKET_FILTER\n", __func__);
#ifdef CONFIG_MACH_PUMA5
		*outbuf = cpu_to_le32 (*(u16 *)rndis_per_dev_params[configNr].filter);
#else
		*outbuf = cpu_to_le32(*rndis_per_dev_params[configNr].filter);
#endif
		retval = 0;
		break;

	/* mandatory */
	case OID_GEN_MAXIMUM_TOTAL_SIZE:
		pr_debug("%s: OID_GEN_MAXIMUM_TOTAL_SIZE\n", __func__);
		*outbuf = cpu_to_le32(RNDIS_MAX_TOTAL_SIZE);
		retval = 0;
		break;

	/* mandatory */
	case OID_GEN_MEDIA_CONNECT_STATUS:
		if (rndis_debug > 1)
			pr_debug("%s: OID_GEN_MEDIA_CONNECT_STATUS\n", __func__);
		*outbuf = cpu_to_le32(rndis_per_dev_params[configNr]
						.media_state);
		retval = 0;
		break;

	case OID_GEN_PHYSICAL_MEDIUM:
		pr_debug("%s: OID_GEN_PHYSICAL_MEDIUM\n", __func__);
#ifdef CONFIG_MACH_PUMA5
		*outbuf = __constant_cpu_to_le32 (2); //5
#else		
		*outbuf = cpu_to_le32(0);
#endif
		retval = 0;
		break;

	/* The RNDIS specification is incomplete/wrong.   Some versions
	 * of MS-Windows expect OIDs that aren't specified there.  Other
	 * versions emit undefined RNDIS messages. DOCUMENT ALL THESE!
	 */
	case OID_GEN_MAC_OPTIONS:		/* from WinME */
		pr_debug("%s: OID_GEN_MAC_OPTIONS\n", __func__);
		*outbuf = cpu_to_le32(
			  NDIS_MAC_OPTION_RECEIVE_SERIALIZED
			| NDIS_MAC_OPTION_FULL_DUPLEX);
		retval = 0;
		break;

	/* statistics OIDs (table 4-2) */

	/* mandatory */
	case OID_GEN_XMIT_OK:
		if (rndis_debug > 1)
			pr_debug("%s: OID_GEN_XMIT_OK\n", __func__);
		if (stats) {
			*outbuf = cpu_to_le32(stats->tx_packets
				- stats->tx_errors - stats->tx_dropped);
			retval = 0;
		}
		break;

	/* mandatory */
	case OID_GEN_RCV_OK:
		if (rndis_debug > 1)
			pr_debug("%s: OID_GEN_RCV_OK\n", __func__);
		if (stats) {
			*outbuf = cpu_to_le32(stats->rx_packets
				- stats->rx_errors - stats->rx_dropped);
			retval = 0;
		}
		break;

	/* mandatory */
	case OID_GEN_XMIT_ERROR:
		if (rndis_debug > 1)
			pr_debug("%s: OID_GEN_XMIT_ERROR\n", __func__);
		if (stats) {
			*outbuf = cpu_to_le32(stats->tx_errors);
			retval = 0;
		}
		break;

	/* mandatory */
	case OID_GEN_RCV_ERROR:
		if (rndis_debug > 1)
			pr_debug("%s: OID_GEN_RCV_ERROR\n", __func__);
		if (stats) {
			*outbuf = cpu_to_le32(stats->rx_errors);
			retval = 0;
		}
		break;

	/* mandatory */
	case OID_GEN_RCV_NO_BUFFER:
		pr_debug("%s: OID_GEN_RCV_NO_BUFFER\n", __func__);
		if (stats) {
			*outbuf = cpu_to_le32(stats->rx_dropped);
			retval = 0;
		}
		break;

	/* ieee802.3 OIDs (table 4-3) */

	/* mandatory */
	case OID_802_3_PERMANENT_ADDRESS:
		pr_debug("%s: OID_802_3_PERMANENT_ADDRESS\n", __func__);
		if (rndis_per_dev_params[configNr].dev) {
			length = ETH_ALEN;
#ifdef CONFIG_MACH_PUMA5
			 memcpy (outbuf,
                                rndis_per_dev_params [configNr].perm_mac,
                                length);
#else
			memcpy(outbuf,
				rndis_per_dev_params[configNr].host_mac,
				length);
#endif
			retval = 0;
		}
		break;

	/* mandatory */
	case OID_802_3_CURRENT_ADDRESS:
		pr_debug("%s: OID_802_3_CURRENT_ADDRESS\n", __func__);
		if (rndis_per_dev_params[configNr].dev) {
			length = ETH_ALEN;
			memcpy(outbuf,
				rndis_per_dev_params [configNr].host_mac,
				length);
			retval = 0;
		}
		break;

	/* mandatory */
	case OID_802_3_MULTICAST_LIST:
		pr_debug("%s: OID_802_3_MULTICAST_LIST\n", __func__);
		/* Multicast base address only */
		*outbuf = cpu_to_le32(0xE0000000);
		retval = 0;
		break;

	/* mandatory */
	case OID_802_3_MAXIMUM_LIST_SIZE:
		pr_debug("%s: OID_802_3_MAXIMUM_LIST_SIZE\n", __func__);
		/* Multicast base address only */
#ifdef CONFIG_MACH_PUMA5
		*outbuf = __constant_cpu_to_le32 (32); /* TAG0008 */
#else
		*outbuf = cpu_to_le32(1);
#endif
		retval = 0;
		break;

	case OID_802_3_MAC_OPTIONS:
		pr_debug("%s: OID_802_3_MAC_OPTIONS\n", __func__);
		*outbuf = cpu_to_le32(0);
		retval = 0;
		break;

	/* ieee802.3 statistics OIDs (table 4-4) */

	/* mandatory */
	case OID_802_3_RCV_ERROR_ALIGNMENT:
		pr_debug("%s: OID_802_3_RCV_ERROR_ALIGNMENT\n", __func__);
		if (stats) {
			*outbuf = cpu_to_le32(stats->rx_frame_errors);
			retval = 0;
		}
		break;

	/* mandatory */
	case OID_802_3_XMIT_ONE_COLLISION:
		pr_debug("%s: OID_802_3_XMIT_ONE_COLLISION\n", __func__);
		*outbuf = cpu_to_le32(0);
		retval = 0;
		break;

	/* mandatory */
	case OID_802_3_XMIT_MORE_COLLISIONS:
		pr_debug("%s: OID_802_3_XMIT_MORE_COLLISIONS\n", __func__);
		*outbuf = cpu_to_le32(0);
		retval = 0;
		break;

	default:
		pr_warning("%s: query unknown OID 0x%08X\n",
			 __func__, OID);
	}
	if (retval < 0)
		length = 0;

	resp->InformationBufferLength = cpu_to_le32(length);
	r->length = length + sizeof(*resp);
	resp->MessageLength = cpu_to_le32(r->length);
	return retval;
}

static int gen_ndis_set_resp(u8 configNr, u32 OID, u8 *buf, u32 buf_len,
			     rndis_resp_t *r)
{
	rndis_set_cmplt_type *resp;
	int i, retval = -ENOTSUPP;
	struct rndis_params *params;

	if (!r)
		return -ENOMEM;
	resp = (rndis_set_cmplt_type *)r->buf;
	if (!resp)
		return -ENOMEM;

	if (buf_len && rndis_debug > 1) {
		pr_debug("set OID %08x value, len %d:\n", OID, buf_len);
		for (i = 0; i < buf_len; i += 16) {
			pr_debug("%03d: %08x %08x %08x %08x\n", i,
				get_unaligned_le32(&buf[i]),
				get_unaligned_le32(&buf[i + 4]),
				get_unaligned_le32(&buf[i + 8]),
				get_unaligned_le32(&buf[i + 12]));
		}
	}

	params = &rndis_per_dev_params[configNr];
	switch (OID) {
	case OID_GEN_CURRENT_PACKET_FILTER:

		/* these NDIS_PACKET_TYPE_* bitflags are shared with
		 * cdc_filter; it's not RNDIS-specific
		 * NDIS_PACKET_TYPE_x == USB_CDC_PACKET_TYPE_x for x in:
		 *	PROMISCUOUS, DIRECTED,
		 *	MULTICAST, ALL_MULTICAST, BROADCAST
		 */
		*params->filter = (u16)get_unaligned_le32(buf);
		pr_debug("%s: OID_GEN_CURRENT_PACKET_FILTER %08x\n",
			__func__, *params->filter);

		/* this call has a significant side effect:  it's
		 * what makes the packet flow start and stop, like
		 * activating the CDC Ethernet altsetting.
		 */
		retval = 0;
		if (*params->filter) {
			params->state = RNDIS_DATA_INITIALIZED;
			netif_carrier_on(params->dev);
			if (netif_running(params->dev))
				netif_wake_queue(params->dev);
		} else {
			params->state = RNDIS_INITIALIZED;
			netif_carrier_off(params->dev);
			netif_stop_queue(params->dev);
		}
		break;

	case OID_802_3_MULTICAST_LIST:
		/* I think we can ignore this */
		pr_debug("%s: OID_802_3_MULTICAST_LIST\n", __func__);
#ifdef CONFIG_MACH_PUMA5
		/* TAG0010*/
		memset (rndis_per_dev_params [configNr].mcast_addr, 0,
                        RNDIS_MAX_MULTICAST_SIZE * 6);
                memcpy (rndis_per_dev_params [configNr].mcast_addr,
                        buf, buf_len);

		retval = 0; 

#else
		retval = 0;
#endif
		break;
#ifdef CONFIG_MACH_PUMA5
	case OID_GEN_RNDIS_CONFIG_PARAMETER:
		{
                struct rndis_config_parameter   *param;
                char   *strbuf;
                u16    param_name [80], parlen, index = 0;
                u8      first, second;


		param = (struct rndis_config_parameter *) buf;
                strbuf = buf + cpu_to_le32(param->ParameterNameOffset);

		string_to_unicode ("NetworkAddress", param_name, &parlen);
                if ((parlen == cpu_to_le32(param->ParameterNameLength)) &&
                        ((memcmp (param_name, strbuf, parlen) == 0)))
                {
                        parlen = cpu_to_le32(param->ParameterValueLength);
                        strbuf = buf + cpu_to_le32(param->ParameterValueOffset);
                        while (parlen)
                        {
                                first = unicode_to_macnibble (cpu_to_le16p((u16*)strbuf));
                                strbuf += 2;
                                second = unicode_to_macnibble (cpu_to_le16p((u16*)strbuf));
                                strbuf += 2;
                        if ((first != 0xff) && (second != 0xff))
                        {
                                rndis_per_dev_params[configNr].host_mac [index] = (first << 4) | second;
                                parlen -= 4;
                                index ++;
                        } else {
                                break;
                        }
                        }
                        retval = 0;
                }
                }

		break;
#endif
	default:
		pr_warning("%s: set unknown OID 0x%08X, size %d\n",
			 __func__, OID, buf_len);
	}

	return retval;
}

/*
 * Response Functions
 */

static int rndis_init_response(int configNr, rndis_init_msg_type *buf)
{
	rndis_init_cmplt_type *resp;
	rndis_resp_t *r;
	struct rndis_params *params = rndis_per_dev_params + configNr;

	if (!params->dev)
		return -ENOTSUPP;

	r = rndis_add_response(configNr, sizeof(rndis_init_cmplt_type));
	if (!r)
		return -ENOMEM;
	resp = (rndis_init_cmplt_type *)r->buf;

	resp->MessageType = cpu_to_le32(REMOTE_NDIS_INITIALIZE_CMPLT);
	resp->MessageLength = cpu_to_le32(52);
	resp->RequestID = buf->RequestID; /* Still LE in msg buffer */
	resp->Status = cpu_to_le32(RNDIS_STATUS_SUCCESS);
	resp->MajorVersion = cpu_to_le32(RNDIS_MAJOR_VERSION);
	resp->MinorVersion = cpu_to_le32(RNDIS_MINOR_VERSION);
	resp->DeviceFlags = cpu_to_le32(RNDIS_DF_CONNECTIONLESS);
	resp->Medium = cpu_to_le32(RNDIS_MEDIUM_802_3);
	resp->MaxPacketsPerTransfer = cpu_to_le32(1);
	resp->MaxTransferSize = cpu_to_le32(
		  params->dev->mtu
		+ sizeof(struct ethhdr)
		+ sizeof(struct rndis_packet_msg_type)
		+ 22);
	resp->PacketAlignmentFactor = cpu_to_le32(0);
	resp->AFListOffset = cpu_to_le32(0);
	resp->AFListSize = cpu_to_le32(0);

	params->resp_avail(params->v);
	return 0;
}

static int rndis_query_response(int configNr, rndis_query_msg_type *buf)
{
	rndis_query_cmplt_type *resp;
	rndis_resp_t *r;
	struct rndis_params *params = rndis_per_dev_params + configNr;

	/* pr_debug("%s: OID = %08X\n", __func__, cpu_to_le32(buf->OID)); */
	if (!params->dev)
		return -ENOTSUPP;

	/*
	 * we need more memory:
	 * gen_ndis_query_resp expects enough space for
	 * rndis_query_cmplt_type followed by data.
	 * oid_supported_list is the largest data reply
	 */
	r = rndis_add_response(configNr,
		sizeof(oid_supported_list) + sizeof(rndis_query_cmplt_type));
	if (!r)
		return -ENOMEM;
	resp = (rndis_query_cmplt_type *)r->buf;

	resp->MessageType = cpu_to_le32(REMOTE_NDIS_QUERY_CMPLT);
	resp->RequestID = buf->RequestID; /* Still LE in msg buffer */

	if (gen_ndis_query_resp(configNr, le32_to_cpu(buf->OID),
			le32_to_cpu(buf->InformationBufferOffset)
					+ 8 + (u8 *)buf,
			le32_to_cpu(buf->InformationBufferLength),
			r)) {
		/* OID not supported */
		resp->Status = cpu_to_le32(RNDIS_STATUS_NOT_SUPPORTED);
		resp->MessageLength = cpu_to_le32(sizeof *resp);
		resp->InformationBufferLength = cpu_to_le32(0);
		resp->InformationBufferOffset = cpu_to_le32(0);
	} else
		resp->Status = cpu_to_le32(RNDIS_STATUS_SUCCESS);

	params->resp_avail(params->v);
	return 0;
}

static int rndis_set_response(int configNr, rndis_set_msg_type *buf)
{
	u32 BufLength, BufOffset;
	rndis_set_cmplt_type *resp;
	rndis_resp_t *r;
	struct rndis_params *params = rndis_per_dev_params + configNr;

	r = rndis_add_response(configNr, sizeof(rndis_set_cmplt_type));
	if (!r)
		return -ENOMEM;
	resp = (rndis_set_cmplt_type *)r->buf;

	BufLength = le32_to_cpu(buf->InformationBufferLength);
	BufOffset = le32_to_cpu(buf->InformationBufferOffset);

#ifdef	VERBOSE_DEBUG
	pr_debug("%s: Length: %d\n", __func__, BufLength);
	pr_debug("%s: Offset: %d\n", __func__, BufOffset);
	pr_debug("%s: InfoBuffer: ", __func__);

	for (i = 0; i < BufLength; i++) {
		pr_debug("%02x ", *(((u8 *) buf) + i + 8 + BufOffset));
	}

	pr_debug("\n");
#endif

	resp->MessageType = cpu_to_le32(REMOTE_NDIS_SET_CMPLT);
	resp->MessageLength = cpu_to_le32(16);
	resp->RequestID = buf->RequestID; /* Still LE in msg buffer */
	if (gen_ndis_set_resp(configNr, le32_to_cpu(buf->OID),
			((u8 *)buf) + 8 + BufOffset, BufLength, r))
		resp->Status = cpu_to_le32(RNDIS_STATUS_NOT_SUPPORTED);
	else
		resp->Status = cpu_to_le32(RNDIS_STATUS_SUCCESS);

	params->resp_avail(params->v);
	return 0;
}

static int rndis_reset_response(int configNr, rndis_reset_msg_type *buf)
{
	rndis_reset_cmplt_type *resp;
	rndis_resp_t *r;
	struct rndis_params *params = rndis_per_dev_params + configNr;

	r = rndis_add_response(configNr, sizeof(rndis_reset_cmplt_type));
	if (!r)
		return -ENOMEM;
	resp = (rndis_reset_cmplt_type *)r->buf;

	resp->MessageType = cpu_to_le32(REMOTE_NDIS_RESET_CMPLT);
	resp->MessageLength = cpu_to_le32(16);
	resp->Status = cpu_to_le32(RNDIS_STATUS_SUCCESS);
	/* resent information */
	resp->AddressingReset = cpu_to_le32(1);

	params->resp_avail(params->v);
	return 0;
}

static int rndis_keepalive_response(int configNr,
				    rndis_keepalive_msg_type *buf)
{
	rndis_keepalive_cmplt_type *resp;
	rndis_resp_t *r;
	struct rndis_params *params = rndis_per_dev_params + configNr;

	/* host "should" check only in RNDIS_DATA_INITIALIZED state */

	r = rndis_add_response(configNr, sizeof(rndis_keepalive_cmplt_type));
	if (!r)
		return -ENOMEM;
	resp = (rndis_keepalive_cmplt_type *)r->buf;

	resp->MessageType = cpu_to_le32(
			REMOTE_NDIS_KEEPALIVE_CMPLT);
	resp->MessageLength = cpu_to_le32(16);
	resp->RequestID = buf->RequestID; /* Still LE in msg buffer */
	resp->Status = cpu_to_le32(RNDIS_STATUS_SUCCESS);

	params->resp_avail(params->v);
	return 0;
}


/*
 * Device to Host Comunication
 */
static int rndis_indicate_status_msg(int configNr, u32 status)
{
	rndis_indicate_status_msg_type *resp;
	rndis_resp_t *r;
	struct rndis_params *params = rndis_per_dev_params + configNr;

	if (params->state == RNDIS_UNINITIALIZED)
		return -ENOTSUPP;

	r = rndis_add_response(configNr,
				sizeof(rndis_indicate_status_msg_type));
	if (!r)
		return -ENOMEM;
	resp = (rndis_indicate_status_msg_type *)r->buf;

	resp->MessageType = cpu_to_le32(REMOTE_NDIS_INDICATE_STATUS_MSG);
	resp->MessageLength = cpu_to_le32(20);
	resp->Status = cpu_to_le32(status);
	resp->StatusBufferLength = cpu_to_le32(0);
	resp->StatusBufferOffset = cpu_to_le32(0);

	params->resp_avail(params->v);
	return 0;
}

int rndis_signal_connect(int configNr)
{
	rndis_per_dev_params[configNr].media_state
			= NDIS_MEDIA_STATE_CONNECTED;
	return rndis_indicate_status_msg(configNr,
					  RNDIS_STATUS_MEDIA_CONNECT);
}

#ifdef CONFIG_MACH_PUMA5
int rndis_signal_disconnect (int configNr, u8 logical)
#else
int rndis_signal_disconnect(int configNr)
#endif
{
#ifdef CONFIG_MACH_PUMA5
        u8      *buf;
        u32     temp;
#endif	
	rndis_per_dev_params[configNr].media_state
			= NDIS_MEDIA_STATE_DISCONNECTED;
#ifdef CONFIG_MACH_PUMA5
        if (!logical)
        {

                rndis_per_dev_params [configNr].state = RNDIS_UNINITIALIZED;
                while ((buf = rndis_get_next_response (configNr, &temp)))
                        rndis_free_response (configNr, buf);
                return 0;
        } else
#endif

	return rndis_indicate_status_msg(configNr,
					  RNDIS_STATUS_MEDIA_DISCONNECT);
}

void rndis_uninit(int configNr)
{
	u8 *buf;
	u32 length;

	if (configNr >= RNDIS_MAX_CONFIGS)
		return;
	rndis_per_dev_params[configNr].state = RNDIS_UNINITIALIZED;

	/* drain the response queue */
	while ((buf = rndis_get_next_response(configNr, &length)))
		rndis_free_response(configNr, buf);
}

void rndis_set_host_mac(int configNr, const u8 *addr)
{
	rndis_per_dev_params[configNr].host_mac = addr;
#ifdef CONFIG_MACH_PUMA5
	memcpy ((void *)rndis_per_dev_params[configNr].perm_mac, addr, 6);
#endif	
}

/*
 * Message Parser
 */
int rndis_msg_parser(u8 configNr, u8 *buf)
{
	u32 MsgType, MsgLength;
	__le32 *tmp;
	struct rndis_params *params;

	if (!buf)
		return -ENOMEM;

	tmp = (__le32 *)buf;
	MsgType   = get_unaligned_le32(tmp++);
	MsgLength = get_unaligned_le32(tmp++);

	if (configNr >= RNDIS_MAX_CONFIGS)
		return -ENOTSUPP;
	params = &rndis_per_dev_params[configNr];

	/* NOTE: RNDIS is *EXTREMELY* chatty ... Windows constantly polls for
	 * rx/tx statistics and link status, in addition to KEEPALIVE traffic
	 * and normal HC level polling to see if there's any IN traffic.
	 */

	/* For USB: responses may take up to 10 seconds */
	switch (MsgType) {
	case REMOTE_NDIS_INITIALIZE_MSG:
		pr_debug("%s: REMOTE_NDIS_INITIALIZE_MSG\n",
			__func__);
		params->state = RNDIS_INITIALIZED;
		return rndis_init_response(configNr,
					(rndis_init_msg_type *)buf);

	case REMOTE_NDIS_HALT_MSG:
		pr_debug("%s: REMOTE_NDIS_HALT_MSG\n",
			__func__);
		params->state = RNDIS_UNINITIALIZED;
		if (params->dev) {
#ifdef CONFIG_MACH_PUMA5
			 memcpy (
                (void *)rndis_per_dev_params[configNr].host_mac,
                (void *)rndis_per_dev_params[configNr].perm_mac, 6); 
#endif		
			netif_carrier_off(params->dev);
			netif_stop_queue(params->dev);
		}
		return 0;

	case REMOTE_NDIS_QUERY_MSG:
		return rndis_query_response(configNr,
					(rndis_query_msg_type *)buf);

	case REMOTE_NDIS_SET_MSG:
		return rndis_set_response(configNr,
					(rndis_set_msg_type *)buf);

	case REMOTE_NDIS_RESET_MSG:
		pr_debug("%s: REMOTE_NDIS_RESET_MSG\n",
			__func__);
		return rndis_reset_response(configNr,
					(rndis_reset_msg_type *)buf);

	case REMOTE_NDIS_KEEPALIVE_MSG:
		/* For USB: host does this every 5 seconds */
		if (rndis_debug > 1)
			pr_debug("%s: REMOTE_NDIS_KEEPALIVE_MSG\n",
				__func__);
		return rndis_keepalive_response(configNr,
						 (rndis_keepalive_msg_type *)
						 buf);

	default:
		/* At least Windows XP emits some undefined RNDIS messages.
		 * In one case those messages seemed to relate to the host
		 * suspending itself.
		 */
		pr_warning("%s: unknown RNDIS message 0x%08X len %d\n",
			__func__, MsgType, MsgLength);
		{
			unsigned i;
			for (i = 0; i < MsgLength; i += 16) {
				pr_debug("%03d: "
					" %02x %02x %02x %02x"
					" %02x %02x %02x %02x"
					" %02x %02x %02x %02x"
					" %02x %02x %02x %02x"
					"\n",
					i,
					buf[i], buf [i+1],
						buf[i+2], buf[i+3],
					buf[i+4], buf [i+5],
						buf[i+6], buf[i+7],
					buf[i+8], buf [i+9],
						buf[i+10], buf[i+11],
					buf[i+12], buf [i+13],
						buf[i+14], buf[i+15]);
			}
		}
		break;
	}

	return -ENOTSUPP;
}

int rndis_register(void (*resp_avail)(void *v), void *v)
{
	u8 i;

	if (!resp_avail)
		return -EINVAL;

	for (i = 0; i < RNDIS_MAX_CONFIGS; i++) {
		if (!rndis_per_dev_params[i].used) {
			rndis_per_dev_params[i].used = 1;
			rndis_per_dev_params[i].resp_avail = resp_avail;
			rndis_per_dev_params[i].v = v;
			pr_debug("%s: configNr = %d\n", __func__, i);
			return i;
		}
	}
	pr_debug("failed\n");

	return -ENODEV;
}

void rndis_deregister(int configNr)
{
	pr_debug("%s:\n", __func__);

	if (configNr >= RNDIS_MAX_CONFIGS) return;
	rndis_per_dev_params[configNr].used = 0;
}

int rndis_set_param_dev(u8 configNr, struct net_device *dev, u16 *cdc_filter)
{
	pr_debug("%s:\n", __func__);
	if (!dev)
		return -EINVAL;
	if (configNr >= RNDIS_MAX_CONFIGS) return -1;

	rndis_per_dev_params[configNr].dev = dev;
	rndis_per_dev_params[configNr].filter = cdc_filter;

	return 0;
}

int rndis_set_param_vendor(u8 configNr, u32 vendorID, const char *vendorDescr)
{
	pr_debug("%s:\n", __func__);
	if (!vendorDescr) return -1;
	if (configNr >= RNDIS_MAX_CONFIGS) return -1;

	rndis_per_dev_params[configNr].vendorID = vendorID;
	rndis_per_dev_params[configNr].vendorDescr = vendorDescr;

	return 0;
}

int rndis_set_param_medium(u8 configNr, u32 medium, u32 speed)
{
	pr_debug("%s: %u %u\n", __func__, medium, speed);
	if (configNr >= RNDIS_MAX_CONFIGS) return -1;

	rndis_per_dev_params[configNr].medium = medium;
	rndis_per_dev_params[configNr].speed = speed;

	return 0;
}
#ifdef CONFIG_MACH_PUMA5
u32  rndis_get_param_filter (u8 configNr)
{
        return (u32)(*rndis_per_dev_params [configNr].filter);
}
#endif

void rndis_add_hdr(struct sk_buff *skb)
{
	struct rndis_packet_msg_type *header;

	if (!skb)
		return;
	header = (void *)skb_push(skb, sizeof(*header));
	memset(header, 0, sizeof *header);
	header->MessageType = cpu_to_le32(REMOTE_NDIS_PACKET_MSG);
	header->MessageLength = cpu_to_le32(skb->len);
	header->DataOffset = cpu_to_le32(36);
	header->DataLength = cpu_to_le32(skb->len - sizeof(*header));
}

void rndis_free_response(int configNr, u8 *buf)
{
	rndis_resp_t *r;
	struct list_head *act, *tmp;

	list_for_each_safe(act, tmp,
			&(rndis_per_dev_params[configNr].resp_queue))
	{
		r = list_entry(act, rndis_resp_t, list);
		if (r && r->buf == buf) {
			list_del(&r->list);
			kfree(r);
		}
	}
}

u8 *rndis_get_next_response(int configNr, u32 *length)
{
	rndis_resp_t *r;
	struct list_head *act, *tmp;

	if (!length) return NULL;

	list_for_each_safe(act, tmp,
			&(rndis_per_dev_params[configNr].resp_queue))
	{
		r = list_entry(act, rndis_resp_t, list);
		if (!r->send) {
			r->send = 1;
			*length = r->length;
			return r->buf;
		}
	}

	return NULL;
}

static rndis_resp_t *rndis_add_response(int configNr, u32 length)
{
	rndis_resp_t *r;

	/* NOTE: this gets copied into ether.c USB_BUFSIZ bytes ... */
	r = kmalloc(sizeof(rndis_resp_t) + length, GFP_ATOMIC);
	if (!r) return NULL;

	r->buf = (u8 *)(r + 1);
	r->length = length;
	r->send = 0;

	list_add_tail(&r->list,
		&(rndis_per_dev_params[configNr].resp_queue));
	return r;
}

int rndis_rm_hdr(struct gether *port,
			struct sk_buff *skb,
			struct sk_buff_head *list)
{
	/* tmp points to a struct rndis_packet_msg_type */
	__le32 *tmp = (void *)skb->data;

	/* MessageType, MessageLength */
	if (cpu_to_le32(REMOTE_NDIS_PACKET_MSG)
			!= get_unaligned(tmp++)) {
		dev_kfree_skb_any(skb);
		return -EINVAL;
	}
	tmp++;

	/* DataOffset, DataLength */
	if (!skb_pull(skb, get_unaligned_le32(tmp++) + 8)) {
		dev_kfree_skb_any(skb);
		return -EOVERFLOW;
	}
	skb_trim(skb, get_unaligned_le32(tmp++));

	skb_queue_tail(list, skb);
	return 0;
}

#ifdef CONFIG_USB_GADGET_DEBUG_FILES

static int rndis_proc_show(struct seq_file *m, void *v)
{
	rndis_params *param = m->private;

	seq_printf(m,
			 "Config Nr. %d\n"
			 "used      : %s\n"
			 "state     : %s\n"
			 "medium    : 0x%08X\n"
			 "speed     : %d\n"
			 "cable     : %s\n"
			 "vendor ID : 0x%08X\n"
			 "vendor    : %s\n",
			 param->confignr, (param->used) ? "y" : "n",
			 ({ char *s = "?";
			 switch (param->state) {
			 case RNDIS_UNINITIALIZED:
				s = "RNDIS_UNINITIALIZED"; break;
			 case RNDIS_INITIALIZED:
				s = "RNDIS_INITIALIZED"; break;
			 case RNDIS_DATA_INITIALIZED:
				s = "RNDIS_DATA_INITIALIZED"; break;
			}; s; }),
			 param->medium,
			 (param->media_state) ? 0 : param->speed*100,
			 (param->media_state) ? "disconnected" : "connected",
			 param->vendorID, param->vendorDescr);
	return 0;
}

static ssize_t rndis_proc_write(struct file *file, const char __user *buffer,
				size_t count, loff_t *ppos)
{
	rndis_params *p = PDE(file->f_path.dentry->d_inode)->data;
	u32 speed = 0;
	int i, fl_speed = 0;

	for (i = 0; i < count; i++) {
		char c;
		if (get_user(c, buffer))
			return -EFAULT;
		switch (c) {
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
			fl_speed = 1;
			speed = speed * 10 + c - '0';
			break;
		case 'C':
		case 'c':
			rndis_signal_connect(p->confignr);
			break;
		case 'D':
		case 'd':
#ifdef CONFIG_MACH_PUMA5
			rndis_signal_disconnect(p->confignr,0);
#else			
			rndis_signal_disconnect(p->confignr);
#endif
			break;
		default:
			if (fl_speed) p->speed = speed;
			else pr_debug("%c is not valid\n", c);
			break;
		}

		buffer++;
	}

	return count;
}

static int rndis_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, rndis_proc_show, PDE(inode)->data);
}

static const struct file_operations rndis_proc_fops = {
	.owner		= THIS_MODULE,
	.open		= rndis_proc_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
	.write		= rndis_proc_write,
};

#define	NAME_TEMPLATE "driver/rndis-%03d"

static struct proc_dir_entry *rndis_connect_state [RNDIS_MAX_CONFIGS];

#endif /* CONFIG_USB_GADGET_DEBUG_FILES */


int rndis_init(void)
{
	u8 i;

	for (i = 0; i < RNDIS_MAX_CONFIGS; i++) {
#ifdef	CONFIG_USB_GADGET_DEBUG_FILES
		char name [20];

		sprintf(name, NAME_TEMPLATE, i);
		rndis_connect_state[i] = proc_create_data(name, 0660, NULL,
					&rndis_proc_fops,
					(void *)(rndis_per_dev_params + i));
		if (!rndis_connect_state[i]) {
			pr_debug("%s: remove entries", __func__);
			while (i) {
				sprintf(name, NAME_TEMPLATE, --i);
				remove_proc_entry(name, NULL);
			}
			pr_debug("\n");
			return -EIO;
		}
#endif
		rndis_per_dev_params[i].confignr = i;
		rndis_per_dev_params[i].used = 0;
		rndis_per_dev_params[i].state = RNDIS_UNINITIALIZED;
		rndis_per_dev_params[i].media_state
				= NDIS_MEDIA_STATE_DISCONNECTED;
		INIT_LIST_HEAD(&(rndis_per_dev_params[i].resp_queue));
	}

	return 0;
}

void rndis_exit(void)
{
#ifdef CONFIG_USB_GADGET_DEBUG_FILES
	u8 i;
	char name[20];

	for (i = 0; i < RNDIS_MAX_CONFIGS; i++) {
		sprintf(name, NAME_TEMPLATE, i);
		remove_proc_entry(name, NULL);
	}
#endif
}

#ifdef CONFIG_MACH_PUMA5
char  rndis_get_multicast_status (int cfg, const u8 *addr)
{
        u32     index = 0;
        char    status = 0;

        while (index < RNDIS_MAX_MULTICAST_SIZE)
        {
                if ((rndis_per_dev_params[cfg].mcast_addr[index][0] == addr[0]) &&
                (rndis_per_dev_params[cfg].mcast_addr[index][1] == addr[1]) &&
                (rndis_per_dev_params[cfg].mcast_addr[index][2] == addr[2]) &&
                (rndis_per_dev_params[cfg].mcast_addr[index][3] == addr[3]) &&
                (rndis_per_dev_params[cfg].mcast_addr[index][4] == addr[4]) &&
                (rndis_per_dev_params[cfg].mcast_addr[index][5] == addr[5]))
                {
                        status = 1;
                        break;
                }
                index++;
        }

        return status;
}
/* TAG0009 */
static void string_to_unicode (char *string, u16 *unicode_string, u16 *length)
{
        u32     index;

        /* The UNICODE string will be twice the length of the ANSI string. */
        *length = 2 * strlen (string);

        /* Convert the ANSI String to UNICODE format. */
        for (index = 0; index < strlen (string); index++)
                unicode_string[index] = cpu_to_le16((u16)(string[index]));

}

static u8 unicode_to_macnibble (u16 value)
{
        u8 retval  = 0xff;

        if ( (value >= '0') && (value <= '9') )
        {
                retval = value - '0';
        } else {
                if ( (value >= 'A') && (value <= 'F') )
                {
                        retval = value - '7';
                } else {
                        if ( (value >= 'a') && (value <= 'f') )
                        {
                                retval = value - 'W';
                        }
                }
        }

        return retval;
}
int is_rndis_configured(int configNr)
{

        if (rndis_per_dev_params [configNr].state == RNDIS_UNINITIALIZED)
                return 0;
        else
	if (rndis_per_dev_params [configNr].state == RNDIS_INITIALIZED)
                return 1;
	else
		return -1;
}
#endif
