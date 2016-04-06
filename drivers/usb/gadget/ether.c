/*
 * ether.c -- Ethernet gadget driver, with CDC and non-CDC options
 *
 * Copyright (C) 2003-2005,2008 David Brownell
 * Copyright (C) 2003-2004 Robert Schwebel, Benedikt Spranger
 * Copyright (C) 2008 Nokia Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/* Copyright 2008, Texas Instruments Incorporated
 *
 * This program has been modified from its original operation by
 * Texas Instruments to do the following:
 *
 * Explanation of modification:
 *  Puma5 changes/fixes
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

/* #define VERBOSE_DEBUG */

#include <linux/kernel.h>
#include <linux/utsname.h>


#if defined USB_ETH_RNDIS
#  undef USB_ETH_RNDIS
#endif
#ifdef CONFIG_USB_ETH_RNDIS
#  define USB_ETH_RNDIS y
#endif

#include "u_ether.h"

#define CONFIG_USB_PPD_SUPPORT
#define USB_PPD_DEBUG

#if defined(CONFIG_ARM_AVALANCHE_PPD) && defined(CONFIG_USB_PPD_SUPPORT)

#include <asm-arm/arch-avalanche/generic/csl_defs.h>

#include <asm-arm/arch-avalanche/generic/pal_cppi41.h>

#include <asm-arm/arch-avalanche/puma5/puma5_pp.h>
#include <linux/ti_ppm.h>
extern Int32 ti_ppd_create_pid(TI_PP_PID *pid_params);
extern Int32 ti_ppd_set_pid_flags(TI_PP_PID *pid_params, Uint32 new_flags);
extern Int32 ti_ppd_delete_pid(Uint8 pid_handle);
#endif

#ifdef CONFIG_MACH_PUMA5

	#include <asm-arm/arch-avalanche/puma5/usb_vendor.h>
	#define USB_MIB_SUPPORT
//	#define USB_PERFCNT_SELECT
#ifdef USB_MIB_SUPPORT
	#include <linux/proc_fs.h>
	#include "mibIoctl.h"
static int proc_read_usb_stats(char *page, char **start,
                        off_t off, int count, int *eof, void *data);
static int proc_write_usb_stats(struct file *fp, const char *buf, unsigned long count,
                    void *data);
static int init_mib_counters(struct net_device *dev);
static int proc_read_usb_link(char *page, char **start,
                        off_t off, int count, int *eof, void *data);
static int proc_read_usb_info(char *page, char **start,
                        off_t off, int count, int *eof, void *data);

#ifdef USB_PERFCNT_SELECT
static int proc_read_usb_cppi(char *page, char **start,
                        off_t off, int count, int *eof, void *data);
static int proc_write_usb_cppi(struct file *fp, const char *buf, unsigned long count,
                    void *data);
#endif
static int proc_write_usb_config_mode(struct file *fp, const char *buf, unsigned long count,
                    void *data);
static int proc_read_usb_config_mode(char *page, char **start,
                        off_t off, int count, int *eof, void *data);
/*
echo 0 > /proc/avalanche/usb_config_mode will set
rndis_default_mode = 1 ( default usb configuration mode = 'A'
echo 1 > /proc/avalanche/usb_config_mode will set
rndis_default_mode = 0 ( default usb configuration mode = 'B')
*/
int rndis_default_mode = 1; 
#endif
#endif
//#define USBDRV_DEBUG
#ifdef USBDRV_DEBUG
#define dprintk(x,...) printk(x, ## __VA_ARGS__)
#else
#define dprintk(x,...)
#endif

//#if defined(CONFIG_ARM_AVALANCHE_PPD) && defined(CONFIG_MACH_PUMA5)
#if defined(CONFIG_ARM_AVALANCHE_PPD) && defined(CONFIG_USB_PPD_SUPPORT)
#define MAX_USB_ENDPOINT            4
#define TI_PP_PID_TYPE_USBRNDIS         3
#define TI_PP_PID_TYPE_USBCDC           4

static  TI_PP_PID   pid_usb [MAX_USB_ENDPOINT];
static  TI_PP_VPID  vpid_usb[MAX_USB_ENDPOINT][TI_PP_MAX_VPID];

static int usb_add_pid      (int indx, int pid_type);
static int install_usb_pid  (struct net_device *net, int mode, bool restore_vpids);
static int uninstall_usb_pid(struct net_device *net, int mode, bool save_vpids);

extern Int32 ti_ppd_config_pid_range(TI_PP_PID_RANGE *pid_range);
extern Int32 ti_ppd_remove_pid_range(Uint32 port_num);
int created_usb_pidtype = 0;
#endif
extern int is_rndis_configured(int configNr);

/*
 * Ethernet gadget driver -- with CDC and non-CDC options
 * Builds on hardware support for a full duplex link.
 *
 * CDC Ethernet is the standard USB solution for sending Ethernet frames
 * using USB.  Real hardware tends to use the same framing protocol but look
 * different for control features.  This driver strongly prefers to use
 * this USB-IF standard as its open-systems interoperability solution;
 * most host side USB stacks (except from Microsoft) support it.
 *
 * This is sometimes called "CDC ECM" (Ethernet Control Model) to support
 * TLA-soup.  "CDC ACM" (Abstract Control Model) is for modems, and a new
 * "CDC EEM" (Ethernet Emulation Model) is starting to spread.
 *
 * There's some hardware that can't talk CDC ECM.  We make that hardware
 * implement a "minimalist" vendor-agnostic CDC core:  same framing, but
 * link-level setup only requires activating the configuration.  Only the
 * endpoint descriptors, and product/vendor IDs, are relevant; no control
 * operations are available.  Linux supports it, but other host operating
 * systems may not.  (This is a subset of CDC Ethernet.)
 *
 * It turns out that if you add a few descriptors to that "CDC Subset",
 * (Windows) host side drivers from MCCI can treat it as one submode of
 * a proprietary scheme called "SAFE" ... without needing to know about
 * specific product/vendor IDs.  So we do that, making it easier to use
 * those MS-Windows drivers.  Those added descriptors make it resemble a
 * CDC MDLM device, but they don't change device behavior at all.  (See
 * MCCI Engineering report 950198 "SAFE Networking Functions".)
 *
 * A third option is also in use.  Rather than CDC Ethernet, or something
 * simpler, Microsoft pushes their own approach: RNDIS.  The published
 * RNDIS specs are ambiguous and appear to be incomplete, and are also
 * needlessly complex.  They borrow more from CDC ACM than CDC ECM.
 */

#define DRIVER_DESC		"Ethernet Gadget"
#ifdef CONFIG_MACH_PUMA5
#define DRIVER_VERSION		"May 2007 (Sep 2011)"
#else
#define DRIVER_VERSION		"Memorial Day 2008"
#endif

#ifdef USB_ETH_RNDIS
#define PREFIX			"RNDIS/"
#else
#define PREFIX			""
#endif

/*
 * This driver aims for interoperability by using CDC ECM unless
 *
 *		can_support_ecm()
 *
 * returns false, in which case it supports the CDC Subset.  By default,
 * that returns true; most hardware has no problems with CDC ECM, that's
 * a good default.  Previous versions of this driver had no default; this
 * version changes that, removing overhead for new controller support.
 *
 *	IF YOUR HARDWARE CAN'T SUPPORT CDC ECM, UPDATE THAT ROUTINE!
 */

static inline bool has_rndis(void)
{
#ifdef	USB_ETH_RNDIS
	return true;
#else
	return false;
#endif
}

/*-------------------------------------------------------------------------*/

/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "composite.c"
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"

#include "f_ecm.c"
#include "f_subset.c"
#ifdef	USB_ETH_RNDIS
#include "f_rndis.c"
#include "rndis.c"
#endif
#include "f_eem.c"
#include "u_ether.c"

/*-------------------------------------------------------------------------*/

/* DO NOT REUSE THESE IDs with a protocol-incompatible driver!!  Ever!!
 * Instead:  allocate your own, using normal USB-IF procedures.
 */

/* Thanks to NetChip Technologies for donating this product ID.
 * It's for devices with only CDC Ethernet configurations.
 */
#define CDC_VENDOR_NUM		0x0525	/* NetChip */
#define CDC_PRODUCT_NUM		0xa4a1	/* Linux-USB Ethernet Gadget */

/* For hardware that can't talk CDC, we use the same vendor ID that
 * ARM Linux has used for ethernet-over-usb, both with sa1100 and
 * with pxa250.  We're protocol-compatible, if the host-side drivers
 * use the endpoint descriptors.  bcdDevice (version) is nonzero, so
 * drivers that need to hard-wire endpoint numbers have a hook.
 *
 * The protocol is a minimal subset of CDC Ether, which works on any bulk
 * hardware that's not deeply broken ... even on hardware that can't talk
 * RNDIS (like SA-1100, with no interrupt endpoint, or anything that
 * doesn't handle control-OUT).
 */
#define	SIMPLE_VENDOR_NUM	0x049f
#define	SIMPLE_PRODUCT_NUM	0x505a

/* For hardware that can talk RNDIS and either of the above protocols,
 * use this ID ... the windows INF files will know it.  Unless it's
 * used with CDC Ethernet, Linux 2.4 hosts will need updates to choose
 * the non-RNDIS configuration.
 */
#define RNDIS_VENDOR_NUM	0x0525	/* NetChip */
#define RNDIS_PRODUCT_NUM	0xa4a2	/* Ethernet/RNDIS Gadget */

/* For EEM gadgets */
#define EEM_VENDOR_NUM		0x1d6b	/* Linux Foundation */
#define EEM_PRODUCT_NUM		0x0102	/* EEM Gadget */

/*-------------------------------------------------------------------------*/

static struct usb_device_descriptor device_desc = {
	.bLength =		sizeof device_desc,
	.bDescriptorType =	USB_DT_DEVICE,

	.bcdUSB =		cpu_to_le16 (0x0200),

	.bDeviceClass =		USB_CLASS_COMM,
	.bDeviceSubClass =	0,
	.bDeviceProtocol =	0,
	/* .bMaxPacketSize0 = f(hardware) */

	/* Vendor and product id defaults change according to what configs
	 * we support.  (As does bNumConfigurations.)  These values can
	 * also be overridden by module parameters.
	 */
	.idVendor =		cpu_to_le16 (CDC_VENDOR_NUM),
	.idProduct =		cpu_to_le16 (CDC_PRODUCT_NUM),
	/* .bcdDevice = f(hardware) */
	/* .iManufacturer = DYNAMIC */
	/* .iProduct = DYNAMIC */
	/* NO SERIAL NUMBER */
	.bNumConfigurations =	1,
};

static struct usb_otg_descriptor otg_descriptor = {
	.bLength =		sizeof otg_descriptor,
	.bDescriptorType =	USB_DT_OTG,

	/* REVISIT SRP-only hardware is possible, although
	 * it would not be called "OTG" ...
	 */
	.bmAttributes =		USB_OTG_SRP | USB_OTG_HNP,
};

static const struct usb_descriptor_header *otg_desc[] = {
	(struct usb_descriptor_header *) &otg_descriptor,
	NULL,
};


/* string IDs are assigned dynamically */

#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1

static char manufacturer[50];

static struct usb_string strings_dev[] = {
	[STRING_MANUFACTURER_IDX].s = manufacturer,
	[STRING_PRODUCT_IDX].s = PREFIX DRIVER_DESC,
	{  } /* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static u8 hostaddr[ETH_ALEN];

/*-------------------------------------------------------------------------*/

/*
 * We may not have an RNDIS configuration, but if we do it needs to be
 * the first one present.  That's to make Microsoft's drivers happy,
 * and to follow DOCSIS 1.0 (cable modem standard).
 */
static int __init rndis_do_config(struct usb_configuration *c)
{
	/* FIXME alloc iConfiguration string, set it in c->strings */

	if (gadget_is_otg(c->cdev->gadget)) {
		c->descriptors = otg_desc;
		c->bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	}

	return rndis_bind_config(c, hostaddr);
}

static struct usb_configuration rndis_config_driver = {
	.label			= "RNDIS",
#ifdef CONFIG_MACH_PUMA5
	.bConfigurationValue = 1, //DEV_RNDIS_CONFIG_VALUE,
#else   
	.bConfigurationValue = 2,
#endif
	/* .iConfiguration = DYNAMIC */
	.bmAttributes		= USB_CONFIG_ATT_SELFPOWER,
};

/*-------------------------------------------------------------------------*/

#ifdef CONFIG_USB_ETH_EEM
static int use_eem = 1;
#else
static int use_eem;
#endif
module_param(use_eem, bool, 0);
MODULE_PARM_DESC(use_eem, "use CDC EEM mode");

/*
 * We _always_ have an ECM, CDC Subset, or EEM configuration.
 */
static int __init eth_do_config(struct usb_configuration *c)
{
	/* FIXME alloc iConfiguration string, set it in c->strings */

	if (gadget_is_otg(c->cdev->gadget)) {
		c->descriptors = otg_desc;
		c->bmAttributes |= USB_CONFIG_ATT_WAKEUP;
	}

	if (use_eem)
		return eem_bind_config(c);
	else if (can_support_ecm(c->cdev->gadget))
		return ecm_bind_config(c, hostaddr);
	else
		return geth_bind_config(c, hostaddr);
}

static struct usb_configuration eth_config_driver = {
	/* .label = f(hardware) */
	.bConfigurationValue	= 1,
	/* .iConfiguration = DYNAMIC */
	.bmAttributes		= USB_CONFIG_ATT_SELFPOWER,
};

/*-------------------------------------------------------------------------*/

static int __init eth_bind(struct usb_composite_dev *cdev)
{
	int			gcnum;
	struct usb_gadget	*gadget = cdev->gadget;
	int			status;
#if defined(CONFIG_ARM_AVALANCHE_PPD) && defined(CONFIG_USB_PPD_SUPPORT)
	int pid_type;
#endif
	/* set up network link layer */
	status = gether_setup(cdev->gadget, hostaddr);
	if (status < 0)
		return status;

	/* set up main config label and device descriptor */
	if (use_eem) {
		/* EEM */
		eth_config_driver.label = "CDC Ethernet (EEM)";
		device_desc.idVendor = cpu_to_le16(EEM_VENDOR_NUM);
		device_desc.idProduct = cpu_to_le16(EEM_PRODUCT_NUM);
	} else if (can_support_ecm(cdev->gadget)) {
		/* ECM */
		eth_config_driver.label = "CDC Ethernet (ECM)";
	} else {
		/* CDC Subset */
		eth_config_driver.label = "CDC Subset/SAFE";

		device_desc.idVendor = cpu_to_le16(SIMPLE_VENDOR_NUM);
		device_desc.idProduct = cpu_to_le16(SIMPLE_PRODUCT_NUM);
		if (!has_rndis())
			device_desc.bDeviceClass = USB_CLASS_VENDOR_SPEC;
	}

	if (has_rndis()) {
		/* RNDIS plus ECM-or-Subset */
		device_desc.idVendor = cpu_to_le16(RNDIS_VENDOR_NUM);
		device_desc.idProduct = cpu_to_le16(RNDIS_PRODUCT_NUM);
		device_desc.bNumConfigurations = 2;
	}

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0300 | gcnum);
	else {
		/* We assume that can_support_ecm() tells the truth;
		 * but if the controller isn't recognized at all then
		 * that assumption is a bit more likely to be wrong.
		 */
		dev_warn(&gadget->dev,
				"controller '%s' not recognized; trying %s\n",
				gadget->name,
				eth_config_driver.label);
		device_desc.bcdDevice =
			cpu_to_le16(0x0300 | 0x0099);
	}


	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */

	/* device descriptor strings: manufacturer, product */
	snprintf(manufacturer, sizeof manufacturer, "%s %s with %s",
		init_utsname()->sysname, init_utsname()->release,
		gadget->name);
	status = usb_string_id(cdev);
	if (status < 0)
		goto fail;
	strings_dev[STRING_MANUFACTURER_IDX].id = status;
	device_desc.iManufacturer = status;

	status = usb_string_id(cdev);
	if (status < 0)
		goto fail;
	strings_dev[STRING_PRODUCT_IDX].id = status;
	device_desc.iProduct = status;

	/* register our configuration(s); RNDIS first, if it's used */
	if (has_rndis()) {
		status = usb_add_config(cdev, &rndis_config_driver,
				rndis_do_config);
		if (status < 0)
			goto fail;
	}

	status = usb_add_config(cdev, &eth_config_driver, eth_do_config);
	if (status < 0)
		goto fail;

	dev_info(&gadget->dev, "%s, version: " DRIVER_VERSION "\n",
			DRIVER_DESC);

	return 0;

fail:
	gether_cleanup();
	return status;
}

static int __exit eth_unbind(struct usb_composite_dev *cdev)
{
	gether_cleanup();
	return 0;
}

static struct usb_composite_driver eth_driver = {
	.name		= "g_ether",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.unbind		= __exit_p(eth_unbind),
};

MODULE_DESCRIPTION(PREFIX DRIVER_DESC);
MODULE_AUTHOR("David Brownell, Benedikt Spanger");
MODULE_LICENSE("GPL");

static int __init init(void)
{
	return usb_composite_probe(&eth_driver, eth_bind);
}
module_init(init);

static void __exit cleanup(void)
{
	usb_composite_unregister(&eth_driver);
}
module_exit(cleanup);

#ifdef CONFIG_MACH_PUMA5
#ifdef USB_MIB_SUPPORT
static int init_mib_counters(struct net_device *p_dev)
{
	struct eth_dev          *dev;
	if( !p_dev )
		return -1;

	dev = netdev_priv(p_dev);

        /* clear Transmission counters. */
	dev->ifstats.broadcast_pkts_txed = 0;
	dev->ifstats.broadcast_bytes_txed = 0;
	dev->ifstats.multicast_pkts_txed = 0;
	dev->ifstats.multicast_bytes_txed = 0;
	dev->ifstats.unicast_pkts_txed = 0;
	dev->ifstats.unicast_bytes_txed = 0;
	dev->ifstats.transmit_packets_dropped = 0;
	dev->ifstats.tx_errors = 0;

	/* clear Receive counters */
	dev->ifstats.broadcast_pkts_rxed = 0;
	dev->ifstats.broadcast_bytes_rxed = 0 ;
	dev->ifstats.multicast_pkts_rxed = 0;
	dev->ifstats.multicast_bytes_rxed = 0;
	dev->ifstats.unicast_pkts_rxed = 0;
	dev->ifstats.unicast_bytes_rxed = 0;
	dev->ifstats.receive_packets_dropped = 0;
	dev->ifstats.rx_errors = 0;
	dev->ifstats.unknownProtPkts = 0;
	return 0;

}
int rndis_usb_get_mib_stats(struct net_device *p_dev, int command, void *data)
{
	int error_status = 0;
	struct eth_dev          *dev;
	dev = netdev_priv(p_dev);
	switch(command)
	{
        	case TI_SIOCGINTFCOUNTERS:
		{
		    USB_MIB2_IF_COUNTERS   *ptr_if_cntr;
	            CDC_RNDIS_STATS        *ptr_rndis_stats;

	            ptr_rndis_stats = &dev->ifstats;
	            ptr_if_cntr = (USB_MIB2_IF_COUNTERS *)data;

		    memset((void *)ptr_if_cntr, 0 , sizeof(USB_MIB2_IF_COUNTERS));

	            ptr_if_cntr->inBytesLow = ptr_rndis_stats->unicast_bytes_rxed +
                                      ptr_rndis_stats->broadcast_bytes_rxed +
                                      ptr_rndis_stats->multicast_bytes_rxed ;

	            ptr_if_cntr->inUnicastPktsLow = ptr_rndis_stats->unicast_pkts_rxed;
	            ptr_if_cntr->inBroadcastPktsLow = ptr_rndis_stats->broadcast_pkts_rxed;
        	    ptr_if_cntr->inMulticastPktsLow = ptr_rndis_stats->multicast_pkts_rxed;
	            ptr_if_cntr->inDiscardPkts = ptr_rndis_stats->receive_packets_dropped;

	            ptr_if_cntr->outBytesLow = ptr_rndis_stats->unicast_bytes_txed +
                                      ptr_rndis_stats->broadcast_bytes_txed +
                                      ptr_rndis_stats->multicast_bytes_txed ;

        	    ptr_if_cntr->outUnicastPktsLow = ptr_rndis_stats->unicast_pkts_txed;
	            ptr_if_cntr->outBroadcastPktsLow = ptr_rndis_stats->broadcast_pkts_txed;
        	    ptr_if_cntr->outMulticastPktsLow = ptr_rndis_stats->multicast_pkts_txed;
	            ptr_if_cntr->outDiscardPkts = ptr_rndis_stats->transmit_packets_dropped;

		break;
		}

/*		case TI_SIOCGUSBPARAMS:
		{
		break;
		}

	    	case TI_SIOCGETHERCOUNTERS:
	        case TI_SIOCGUSBCOUNTERS:
        	{
		break;
		}
*/
		case TI_SIOCSINTFADMINSTATUS:
	        {
#if 0
                	USB_MIB2_IF_COMMAND * ptr_if_cmd;
	                ptr_if_cmd = (USB_MIB2_IF_COMMAND *)data;

        	        if(ptr_if_cmd->ifAdminStatus == MIB2_STATUS_UP
                                || ptr_if_cmd->ifAdminStatus == MIB2_STATUS_DOWN)
			/* Only these states are supported at present */
	                {
                   	     ptr_cdc_rndis_mcb->mib2_intf_status = ptr_if_cmd->ifAdminStatus;
       		        }
			else
        	                error_status=-1;
#endif
                break;
        	}

	        case TI_SIOCGINTFPARAMS:
        	{
     	           USB_MIB2_IF_PARAMS * ptr_if_param;
        	   ptr_if_param = (USB_MIB2_IF_PARAMS *)data;

		   memset((void *)ptr_if_param, 0 , sizeof(USB_MIB2_IF_PARAMS));

	            switch (dev->gadget->speed) {
		       	    case USB_SPEED_FULL:
        	                ptr_if_param->ifSpeed = 12000000;
                		break;
#ifdef CONFIG_USB_GADGET_DUALSPEED
	        	    case USB_SPEED_HIGH:
        	                ptr_if_param->ifSpeed = 480000000;
                	        break;
#endif
	                    default:
	                    ptr_if_param->ifSpeed = 0;
	                    break;
                }

                ptr_if_param->ifHighSpeed = 0; /* TODO */
                ptr_if_param->ifOperStatus = /* TODO down, up, dormant */
                ((p_dev->flags & IFF_UP) ? MIB2_STATUS_UP : MIB2_STATUS_DOWN);

                ptr_if_param->ifPromiscuousMode =
                ((p_dev->flags & IFF_PROMISC) ? 1 : 0 );

                break;
        	}

	        default:
		{
	            break;
        	}
	}
	return error_status;
}

typedef struct {
    unsigned int cmd;       /**< Command */
    void *data;             /**< Data provided with the command - depending upon command */
} ethDrvPrivIoctl;
#define USB_SET_HOSTMAC_ADDR 1
/* Ioctl function */
/*KERNEL_PORTING_FIX_ME*/
int eth_ioctl(struct net_device *p_dev, struct ifreq *rq, int cmd)
{
	int ret = 0;
	ethDrvPrivIoctl privIoctl;
	struct eth_dev          *dev = netdev_priv(p_dev);
	char host_mac_addr[18];

	if (cmd == SIOCDEVPRIVATE) {
		if (copy_from_user((char *) &privIoctl, (char *) rq->ifr_data,sizeof(ethDrvPrivIoctl))){
	            return -EFAULT;
		}
		switch (privIoctl.cmd) {
		            /* Program Type 2/3 Address Filter */
		        case USB_SET_HOSTMAC_ADDR:
			if (copy_from_user
	                    ((char *)host_mac_addr, (char *) privIoctl.data,
        	             sizeof(host_mac_addr))){
                		return -EFAULT;
			}
			if (get_ether_addr(host_mac_addr, dev->host_mac)){
                        	printk( "invalid host mac address\n");
                	}
			printk("host_mac changed to %02x:%02x:%02x:%02x:%02x:%02x\n",dev->host_mac[0],dev->host_mac[1],
				dev->host_mac[2],dev->host_mac[3],dev->host_mac[4],dev->host_mac[5]);
            		rndis_set_host_mac (dev->rndis_configured, dev->host_mac);
			break;
			default:
		            return -EFAULT;
		            break;
		}

	} else if (cmd == SIOTIMIB2) {
        TI_SNMP_CMD_T ti_snmp_cmd;

	/* copy from user data */
	if( copy_from_user((char *)&ti_snmp_cmd, (char *)rq->ifr_data,
				sizeof(TI_SNMP_CMD_T)))
		return -EFAULT ;

	switch( ti_snmp_cmd.cmd ) {
	case TI_SIOCGINTFCOUNTERS:
	    {
  	          USB_MIB2_IF_COUNTERS localCounters;
                  ret = rndis_usb_get_mib_stats(p_dev, ti_snmp_cmd.cmd, (void *) &localCounters);

		  if( !ret ){
		        /* copy to user data */
       		       if( copy_to_user( (char *)ti_snmp_cmd.data,
                	   (char *)&localCounters, sizeof(USB_MIB2_IF_COUNTERS)))
		                   return -EFAULT;
		  }else
			return -EFAULT;

	    break;
	    }
	case TI_SIOCGINTFPARAMS:
	    {
                   USB_MIB2_IF_PARAMS localParams;
                   ret = rndis_usb_get_mib_stats(p_dev,ti_snmp_cmd.cmd, (void *) &localParams);

		   if( !ret ){
	 		     /* copy to mib counters to user data */
			   if( copy_to_user((char *)ti_snmp_cmd.data,
				 (char *)&localParams, sizeof(USB_MIB2_IF_PARAMS)))
				return -EFAULT;
		   }else
			return - EFAULT;

	    break;
	    }
	case TI_SIOCGETHERCOUNTERS:
	    {

	    break;
	    }
	case TI_SIOCGETHERPARAMS:
            {
	    break;
	    }

	case TI_SIOCSINTFADMINSTATUS:
	    {
		USB_MIB2_IF_COMMAND localParams;
                ret = rndis_usb_get_mib_stats(p_dev, ti_snmp_cmd.cmd, (void *) &localParams);

//		localParams.ifAdminStatus = (p_dev->flags & IFF_UP) ? 1 :2;

		if( !ret ){
			/* copy to user data */
			if( copy_to_user( (char *)ti_snmp_cmd.data,
			    (char *)&localParams, sizeof(USB_MIB2_IF_COMMAND)))
			   return -EFAULT;
		}else
			return -EFAULT;
		break;
	    }
	}


	}/* elseif */
	return 0;
}

static int proc_read_usb_stats(char *page, char **start,
                        off_t off, int count, int *eof, void *data)
{
        struct eth_dev          *dev;
        struct net_device       *p_dev;
        int len = 0,limit = count - 80;

        p_dev = (struct net_device *)data;
        if( !p_dev )
                goto proc_error;

        dev = netdev_priv(p_dev);

              /* clear Transmission counters. */
        len += sprintf(page+len," USB-MIB Counters - Tx \n");
        if( len <= limit )
                len += sprintf(page+len,"\t ifstats.broadcast_pkts_txed      = %u \n", dev->ifstats.broadcast_pkts_txed);
        if( len <= limit )
                len += sprintf(page+len,"\t ifstats.broadcast_bytes_txed     = %u \n",dev->ifstats.broadcast_bytes_txed) ;
        if( len <= limit )
                len += sprintf(page+len,"\t ifstats.multicast_pkts_txed      = %u \n",dev->ifstats.multicast_pkts_txed);
        if( len <= limit )
                len += sprintf(page+len,"\t ifstats.multicast_bytes_txed     = %u\n",dev->ifstats.multicast_bytes_txed);
        if( len <= limit )
                len += sprintf(page+len,"\t ifstats.unicast_pkts_txed        = %u\n",dev->ifstats.unicast_pkts_txed );
        if( len <= limit )
                len += sprintf(page+len,"\t ifstats.unicast_bytes_txed       = %u\n",dev->ifstats.unicast_bytes_txed );
        if( len <= limit )
                len += sprintf(page+len,"\t ifstats.transmit_packets_dropped = %u\n",dev->ifstats.transmit_packets_dropped );
        if( len <= limit )
                len += sprintf(page+len,"\t ifstats.tx_errors                = %u\n",dev->ifstats.tx_errors );
        /* clear Receive counters */
        if( len <= limit )
                len += sprintf(page+len," USB-MIB Counters - Rx \n");
        if( len <= limit )
                len += sprintf(page+len,"\t ifstats.broadcast_pkts_rxed      = %u\n",dev->ifstats.broadcast_pkts_rxed ) ;
        if( len <= limit )
                len += sprintf(page+len,"\t ifstats.broadcast_bytes_rxed     = %u\n",dev->ifstats.broadcast_bytes_rxed ) ;
        if( len <= limit )
                len += sprintf(page+len,"\t ifstats.multicast_pkts_rxed      = %u\n",dev->ifstats.multicast_pkts_rxed) ;
        if( len <= limit )
                len += sprintf(page+len,"\t ifstats.multicast_bytes_rxed     = %u\n",dev->ifstats.multicast_bytes_rxed) ;
        if( len <= limit )
                len += sprintf(page+len,"\t ifstats.unicast_pkts_rxed        = %u\n",dev->ifstats.unicast_pkts_rxed );
        if( len <= limit )
                len += sprintf(page+len,"\t ifstats.unicast_bytes_rxed       = %u\n",dev->ifstats.unicast_bytes_rxed );
        if( len <= limit )
                len += sprintf(page+len,"\t ifstats.receive_packets_dropped  = %u\n",dev->ifstats.receive_packets_dropped);
        if( len <= limit )
                len += sprintf(page+len,"\t ifstats.rx_errors                = %u\n",dev->ifstats.rx_errors) ;
        if( len <= limit )
                len += sprintf(page+len,"\t ifstats.unknownProtPkts          = %u\n",dev->ifstats.unknownProtPkts);

        if( len <= limit )
        	len += sprintf(page+len," mib2_ifParams \n");
        if( len <= limit )
        	len += sprintf(page+len,"\t ifSpeed              = %s\n",(dev->gadget->speed == USB_SPEED_FULL)? "12Mbps"
                :(dev->gadget->speed == USB_SPEED_HIGH)? "480Mbps" : "Unknown");
/*
        if( len <= limit )
        	len += sprintf(page+len,"\t ifHighSpeed          = \n");
        if( len <= limit )
	        len += sprintf(page+len,"\t ifOperStatus         = %s\n",(p_dev->flags & IFF_UP) ? "Up" : "Down");
        if( len <= limit )
	        len += sprintf(page+len,"\t ifPromiscuousMode    = %s\n",(p_dev->flags & IFF_PROMISC) ? "On" : "Off" );
*/
        if( len <= limit )
                len += sprintf(page+len," mib2_ifCommand \n");
        if( len <= limit )
                len += sprintf(page+len,"\t ifAdminStatus = %d\n", (p_dev->flags & IFF_UP) ? 1 :2);

        return len ;

proc_error:
	*eof = 1;
	return len;
}
static int clear_ethdev_net_stats(struct eth_dev  *dev)
{

#ifdef KERNEL_PORTING_FIX_ME
   	    dev->stats.rx_packets = 0;             /* total packets received       */
        dev->stats.tx_packets = 0;             /* total packets transmitted    */
        dev->stats.rx_bytes   = 0;               /* total bytes received         */
        dev->stats.tx_bytes   = 0;               /* total bytes transmitted      */
        dev->stats.rx_errors  = 0;              /* bad packets received         */
        dev->stats.tx_errors  = 0;              /* packet transmit problems     */
        dev->stats.rx_dropped = 0;             /* no space in linux buffers    */
        dev->stats.tx_dropped = 0;             /* no space available in linux  */
        dev->stats.multicast  = 0;              /* multicast packets received   */
        dev->stats.collisions = 0;

        /* detailed rx_errors: */
        dev->stats.rx_length_errors = 0;
        dev->stats.rx_over_errors   = 0;         /* receiver ring buff overflow  */
        dev->stats.rx_crc_errors    = 0;          /* recved pkt with crc error    */
        dev->stats.rx_frame_errors  = 0;        /* recv'd frame alignment error */
        dev->stats.rx_fifo_errors   = 0;         /* recv'r fifo overrun          */
        dev->stats.rx_missed_errors = 0;       /* receiver missed packet       */

        /* detailed tx_errors */
        dev->stats.tx_aborted_errors = 0;
        dev->stats.tx_carrier_errors = 0;
        dev->stats.tx_fifo_errors    = 0;
        dev->stats.tx_heartbeat_errors = 0;
        dev->stats.tx_window_errors = 0;

        /* for cslip etc */
        dev->stats.rx_compressed = 0;
        dev->stats.tx_compressed = 0;
#endif
	return 0;
}

/* Write stats */
static int proc_write_usb_stats(struct file *fp, const char *buf, unsigned long count,
                    void *data)
{
    char local_buf[31];
    int ret_val = 0;

   struct net_device       *p_dev;
   struct eth_dev          *dev;

    if (count > 30) {
        printk("Use \"echo 0 > usb_stats\" to reset the statistics\n");
        return -EFAULT;
    }

    copy_from_user(local_buf, buf, count);
    local_buf[count - 1] = '\0';        /* Ignoring last \n char */
    ret_val = count;

        p_dev = (struct net_device *)data;
        dev = netdev_priv(p_dev);


    if (strcmp("0", local_buf) == 0) {

        /* clear Transmission counters. */
        dev->ifstats.broadcast_pkts_txed = 0;
        dev->ifstats.broadcast_bytes_txed = 0;
        dev->ifstats.multicast_pkts_txed = 0;
        dev->ifstats.multicast_bytes_txed = 0;
        dev->ifstats.unicast_pkts_txed = 0;
        dev->ifstats.unicast_bytes_txed = 0;
        dev->ifstats.transmit_packets_dropped = 0;
        dev->ifstats.tx_errors = 0;

        /* clear Receive counters */
        dev->ifstats.broadcast_pkts_rxed = 0;
        dev->ifstats.broadcast_bytes_rxed = 0 ;
        dev->ifstats.multicast_pkts_rxed = 0;
        dev->ifstats.multicast_bytes_rxed = 0;
        dev->ifstats.unicast_pkts_rxed = 0;
        dev->ifstats.unicast_bytes_rxed = 0;
        dev->ifstats.receive_packets_dropped = 0;
        dev->ifstats.rx_errors = 0;
	dev->ifstats.unknownProtPkts = 0;

	clear_ethdev_net_stats ( dev );
        printk("Resetting statistics for usb interface\n");

    } else{
        printk("Error: Unknown operation on usb_stats statistics\n");
        printk("Use \"echo 0 > usb_stats\" to reset the statistics\n");
        return -EFAULT;
    }
    return ret_val;
}

static int proc_read_usb_link(char *page, char **start,
                        off_t off, int count, int *eof, void *data)
{

	struct eth_dev          *dev;
        struct net_device       *p_dev;
        int len = 0,limit = count - 80;

        p_dev = (struct net_device *)data;
        if( !p_dev )
                goto proc_error;

        dev = netdev_priv(p_dev);

        if( netif_carrier_ok(p_dev) && (p_dev->flags & IFF_UP) )
	{
#ifdef KERNEL_PORTING_FIX_ME	
		if( len < limit )
	          len += sprintf(page+len," usb0:Link Status: %s, speed = %s, Mode : %s\n","Up" ,
                 (dev->gadget->speed == USB_SPEED_FULL)? "Full":(dev->gadget->speed == USB_SPEED_HIGH)? "High" : "Unknown",
                cdc_active(dev)?"cdc":rndis_active(dev)?"rndis":"Unknown"
                );
#endif
	}
        else{
		if( len < limit )
	                len += sprintf(page+len," usb0:Link Status: %s\n","Down");
	}

proc_error:
	*eof = 1;
	return len;
}
static int proc_read_usb_info(char *page, char **start,
                        off_t off, int count, int *eof, void *data)
{
	struct eth_dev          *dev;
        struct net_device       *net;
        int len = 0,limit = count - 80;

        net = (struct net_device *)data;
        if( !net )
                goto proc_error;

        dev = netdev_priv(net);

        if( len < limit )
                len += sprintf(page+len, "%s, version: " DRIVER_VERSION "\n", DRIVER_DESC); 

#ifdef KERNEL_PORTING_FIX_ME
        if( len < count )

                len += sprintf(page+len, "using %s, OUT %s IN %s%s%s\n", dev->gadget->name,
                dev->out_ep->name, dev->in_ep->name,
                dev->status_ep ? " STATUS " : "",
                dev->status_ep ? dev->status_ep->name : ""
                );
        if( len < limit )
                len += sprintf(page+len, "MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
                net->dev_addr [0], net->dev_addr [1],
                net->dev_addr [2], net->dev_addr [3],
                net->dev_addr [4], net->dev_addr [5]);

        if (cdc_active(dev) || rndis_active(dev))
        if( len < limit )
                len += sprintf(page+len, "HOST MAC %02x:%02x:%02x:%02x:%02x:%02x\n",
                        dev->host_mac [0], dev->host_mac [1],
                        dev->host_mac [2], dev->host_mac [3],
                        dev->host_mac [4], dev->host_mac [5]);
        if( len < limit )
                len += sprintf(page+len,"VendorID   %04x\n", le16_to_cpu(device_desc.idVendor));
        if( len < limit )
                len += sprintf(page+len,"ProductID  %04x\n", le16_to_cpu(device_desc.idProduct));
        if( len < limit )
                len += sprintf(page+len,"Configured as %s mode\n", cdc_active(dev)?"cdc":rndis_active(dev)?"rndis":"Unknown");
        if( len < limit )
                len += sprintf(page+len,"USB CONFIGURATION = %s\n", rndis_default_mode?"A":"B");
#endif
#if defined(CONFIG_ARM_AVALANCHE_PPD) && defined(CONFIG_USB_PPD_SUPPORT)
        if( len < limit ){
	        if( created_usb_pidtype == 3 )
                	len += sprintf(page+len,"USB PID_TYPE(%d) = %s\n",created_usb_pidtype, "RNDIS");
		else
	        if( created_usb_pidtype == 4 )
                	len += sprintf(page+len,"USB PID_TYPE(%d) = %s\n",created_usb_pidtype, "CDC");
	}
#endif
proc_error:
	*eof = 1;
	return len;
}
#ifdef USB_PERFCNT_SELECT

#define MAX_COUNTER 17
struct cppi_counter {
	char name[40];
	u32 addr;
};
u32 gPktDoneCfgvalue =0, gPktDoneCntValue = 0;
struct cppi_counter cppi_cnt[MAX_COUNTER] = {
	{ "usb RxDmaCh4 pkt discard due buf/starv",IO_ADDRESS(0x08690204) },
	{ "usb RxDmaCh5 pkt discard due buf/starv",IO_ADDRESS(0x08690208) },
	{ "usb RxDmaCh6 pkt discard due buf/starv",IO_ADDRESS(0x0869020c) },
	{ "usb RxDmaCh7 pkt discard due buf/starv",IO_ADDRESS(0x08690210) },
	{ "queue pending register 0..31",IO_ADDRESS(0x0306a090)           },
	{ "queue pending register 32..63",IO_ADDRESS(0x0306a094)           },
	{ "queue pending register 64..95",IO_ADDRESS(0x0306a098)           },
	{ "queue pending register 96..127",IO_ADDRESS(0x0306a09c)           },
	{ "queue pending register 128..159",IO_ADDRESS(0x0306a0a0)           },
	{ "queue pending register 160..191",IO_ADDRESS(0x0306a0a4)           },
	{ "queue pending register 192..223",IO_ADDRESS(0x0306a0a8)           },
	{ "queue pending register 224..255",IO_ADDRESS(0x0306a0ac)           },
	{ "queue pending register 256..287",IO_ADDRESS(0x0306a0b0)           },
	{ "queue pending register 288..319",IO_ADDRESS(0x0306a0b4)           },
	{ "queue pending register 320..351",IO_ADDRESS(0x0306a0b8)           },
	{ "queue pending register 352..383",IO_ADDRESS(0x0306a0bc)           },
	{ "usb pakcet Done counter ",IO_ADDRESS(0x08690248) }
};

#if defined(CONFIG_ARM_AVALANCHE_PPD) && defined(CONFIG_USB_PPD_SUPPORT)
extern int apdsp_read_statistics(int index_code, u32 *buf, int bufLen);
extern void Debug_monitor(int debug_queue, int dispBD );
#endif
static int proc_read_usb_cppi(char *page, char **start,
                        off_t off, int count, int *eof, void *data)
{
        struct eth_dev          *dev;
        struct net_device       *p_dev;
        //int len = 0,limit = count - 80
	int i;
	int value  = gPktDoneCfgvalue;

        p_dev = (struct net_device *)data;

        dev = netdev_priv(p_dev);

              /* clear Transmission counters. */
	for(i=0; i<MAX_COUNTER-1; ++i)
                printk("\t %s [%08x]=%x\n",cppi_cnt[i].name,cppi_cnt[i].addr, *(u32 *)cppi_cnt[i].addr);
	if( value != 0 ){
	        printk("\t %s for %sDmaCh%d [%08x]=%u\n",
		cppi_cnt[i].name,(value & 1)?"Rx":"Tx",(value>>1),cppi_cnt[i].addr, *(u32 *)cppi_cnt[i].addr);
	        printk("\t %s for %sDmaCh%d after new cfg [%08x]=%u\n",
		cppi_cnt[i].name,(value & 1)?"Rx":"Tx",(value>>1),cppi_cnt[i].addr, *(u32 *)cppi_cnt[i].addr - gPktDoneCntValue);
	}

#if defined(CONFIG_ARM_AVALANCHE_PPD) && defined(CONFIG_USB_PPD_SUPPORT)
#if 0
	{
#define Cppi41QStatus(qnum)   *(volatile Uint32 *)(0xd3020000+(qnum)*16+0x00)
		printk(" Queue(204):%d\n",Cppi41QStatus(204));
		printk(" Queue(206):%d\n",Cppi41QStatus(206));
		printk(" Queue(145):%d\n",Cppi41QStatus(145));
		printk(" Queue(120):%d\n",Cppi41QStatus(120));
		printk(" Queue(57):%d\n",Cppi41QStatus(59));
	//	Debug_monitor(150,0);
/*		{
		TI_PP_GLOBAL_STATS stats;
		printk(" ret = %d=ti_ppd_get_srl_pkt_stats()\n",ti_ppd_get_srl_pkt_stats(&stats));
		printk("packets_rxed = %d\n",stats.packets_rxed);
		printk("packets_searched = %d\n",stats.packets_searched);
		printk("search_matched = %d\n",stats.search_matched);
		printk("sync_delay = %d\n",stats.sync_delay);
		printk("packets_fwd = %d\n",stats.packets_fwd);
		printk("desc_starved = %d\n",stats.desc_starved);
		printk("ipv4_packets_fwd = %d\n",stats.ipv4_packets_fwd);
		printk("buffer_starved = %d\n",stats.buffer_starved);
		}
*/		
	}
#endif
#endif
	return 0;
}
u32 cppi_txrx_done_cfg_adr = IO_ADDRESS(0x08690200);
static int proc_write_usb_cppi(struct file *fp, const char *buf, unsigned long count,
                    void *data)
{
    char local_buf[31];
    int ret_val = 0;

   struct net_device       *p_dev;
   struct eth_dev          *dev;
	u32 value =0;

    if (count > 30) {
        printk("Use \"echo 0 > usb_stats\" to reset the statistics\n");
        return -EFAULT;
    }

    copy_from_user(local_buf, buf, count);
    local_buf[count - 1] = '\0';        /* Ignoring last \n char */
    ret_val = count;

        p_dev = (struct net_device *)data;
        dev = netdev_priv(p_dev);

    if (strcmp("0", local_buf) == 0) {
	value = 0;
	*(u32 *)cppi_txrx_done_cfg_adr = 0;
	printk("written %x to Adr %x, readback val = %x",value, cppi_txrx_done_cfg_adr,*(u32 *)cppi_txrx_done_cfg_adr);
    } else
    if (strcmp("1", local_buf) == 0) {
	value = (4 <<1) | 1;
	*(u32 *)cppi_txrx_done_cfg_adr = value ;
	printk("Configuring Packet Done counter for RxDmaCh4(ep1out)\n written %x to Adr %x, readback val = %x",
		value, cppi_txrx_done_cfg_adr,*(u32 *)cppi_txrx_done_cfg_adr);
    } else
    if (strcmp("2", local_buf) == 0) {
	value = (5 <<1) | 1;
	*(u32 *)cppi_txrx_done_cfg_adr = value ;
	printk("Configuring Packet Done counter for RxDmaCh5(ep2out)\n written %x to Adr %x, readback val = %x",
		value, cppi_txrx_done_cfg_adr,*(u32 *)cppi_txrx_done_cfg_adr);
    } else
    if (strcmp("3", local_buf) == 0) {
	value = (6 <<1) | 1;
	*(u32 *)cppi_txrx_done_cfg_adr = value ;
	printk("Configuring Packet Done counter for RxDmaCh6(ep3out)\n written %x to Adr %x, readback val = %x",
		value, cppi_txrx_done_cfg_adr,*(u32 *)cppi_txrx_done_cfg_adr);
    } else
    if (strcmp("4", local_buf) == 0) {
	value = (7 <<1) | 1;
	*(u32 *)cppi_txrx_done_cfg_adr = value ;
	printk("Configuring Packet Done counter for RxDmaCh7(ep4out)\n written %x to Adr %x, readback val = %x",
		value, cppi_txrx_done_cfg_adr,*(u32 *)cppi_txrx_done_cfg_adr);
    } else
   if (strcmp("5", local_buf) == 0) {
        value = (4 <<1);
        *(u32 *)cppi_txrx_done_cfg_adr = value ;
	printk("Configuring Packet Done counter for TxDmaCh4(ep1in)\n written %x to Adr %x, readback val = %x",
		value, cppi_txrx_done_cfg_adr,*(u32 *)cppi_txrx_done_cfg_adr);
    } else
    if (strcmp("6", local_buf) == 0) {
        value = (5 <<1);
        *(u32 *)cppi_txrx_done_cfg_adr = value ;
	printk("Configuring Packet Done counter for TxDmaCh5(ep2in)\n written %x to Adr %x, readback val = %x",
		value, cppi_txrx_done_cfg_adr,*(u32 *)cppi_txrx_done_cfg_adr);
    } else
    if (strcmp("7", local_buf) == 0) {
        value = (6 <<1);
        *(u32 *)cppi_txrx_done_cfg_adr = value ;
	printk("Configuring Packet Done counter for TxDmaCh6(ep3in)\n written %x to Adr %x, readback val = %x",
		value, cppi_txrx_done_cfg_adr,*(u32 *)cppi_txrx_done_cfg_adr);
    } else
    if (strcmp("8", local_buf) == 0) {
        value = (7 <<1);
        *(u32 *)cppi_txrx_done_cfg_adr = value ;
	printk("Configuring Packet Done counter for TxDmaCh7(ep4in)\n written %x to Adr %x, readback val = %x",
		value, cppi_txrx_done_cfg_adr,*(u32 *)cppi_txrx_done_cfg_adr);
    } else{

	printk("Usage: echo 1..4 > /proc/avalanche/usb_cppi for configuring Packdone counter RxDmaCh4..7\n");
        printk("       echo 5..8 > /proc/avalanche/usb_cppi for configuring Packdone counter TxDmaCh4..7\n");
        return -EFAULT;
    }
    gPktDoneCntValue = *(u32 *)IO_ADDRESS(0x08690248);
    gPktDoneCfgvalue = value;
    printk("\n");
    return ret_val;
}
#endif
static int proc_write_usb_config_mode(struct file *fp, const char *buf, unsigned long count,
                    void *data)
{
    char local_buf[31];
    int ret_val = 0;

   struct net_device       *p_dev;
   struct eth_dev          *dev;

    if (count > 30) {
        return -EFAULT;
    }

    copy_from_user(local_buf, buf, count);
    local_buf[count - 1] = '\0';        /* Ignoring last \n char */
    ret_val = count;

        p_dev = (struct net_device *)data;
        dev = netdev_priv(p_dev);


    if (strcmp("1", local_buf) == 0) {
                rndis_default_mode = 0; /* configuration mode B */
                printk("USB CONFIGURATION = %s\n", rndis_default_mode?"A":"B");
    } else
    if (strcmp("0", local_buf) == 0) {
                rndis_default_mode = 1; /* configuration mode A */
                printk("USB CONFIGURATION = %s\n", rndis_default_mode?"A":"B");
    } else{
/*
        printk("Error: Unknown operation on usb_configuration\n");
        printk("To select configuration A :\n\techo 0 > /proc/avalanche/usb_config_mode");
        printk("To select configuration B :\n\techo 1 > /proc/avalanche/usb_config_mode");
*/
        return -EFAULT;
    }
    return ret_val;
}
static int proc_read_usb_config_mode(char *page, char **start,
                        off_t off, int count, int *eof, void *data)
{
        struct eth_dev          *dev;
        struct net_device       *p_dev;
        int len = 0,limit = count - 80;

        p_dev = (struct net_device *)data;

        dev = netdev_priv(p_dev);

        if( len < limit )
                len += sprintf(page+len, "%s\n", rndis_default_mode?"A":"B");

	return len;
}
#endif
#endif


#if defined(CONFIG_ARM_AVALANCHE_PPD) && defined(CONFIG_USB_PPD_SUPPORT)

static int usb_add_pid (int indx,int pid_type)
{
#define  TI_PP_PID_TYPE_USBRNDIS               3
#define  TI_PP_PID_TYPE_USBCDC                 4
 
    int retVal;

    /* Add USB PIDs */

    pid_usb[indx].type = pid_type;
    pid_usb[indx].ingress_framing   = TI_PP_PID_INGRESS_ETHERNET
                                            | TI_PP_PID_INGRESS_PPPOE
                                            | TI_PP_PID_INGRESS_IPV6
                                            | TI_PP_PID_INGRESS_IPV4
                                            | TI_PP_PID_INGRESS_IPOE;
    pid_usb[indx].pri_mapping     = 0;   /* Num prio Qs for fwd */
    pid_usb[indx].dflt_pri_drp    = 0;
    pid_usb[indx].dflt_dst_tag    = 0x3FFF;
    pid_usb[indx].dflt_fwd_q      = USB_CPPI4x_USB_TO_HOST_PRXY_QNUM(0);/* Queue 226. Fwd to inf0 by default */
    pid_usb[indx].tx_pri_q_map[0] = USB_CPPI4x_EP0_TX_QNUM(0);  /* Default Q used for egress rec */

    pid_usb[indx].pid_handle      = PP_USB_PID_BASE+indx;
    pid_usb[indx].priv_flags      = 0x0;
    pid_usb[indx].tx_hw_data_len  = 0;
    if( pid_type == TI_PP_PID_TYPE_USBRNDIS  )
    {
        pid_usb[indx].tx_hw_data_len  = 44;
        memset (pid_usb[indx].tx_hw_data, 0, 44);
        *(Uint32*)(pid_usb[indx].tx_hw_data) = cpu_to_le32(0x00000001U);
    }

#ifdef CONFIG_TI_PACKET_PROCESSOR
    retVal = ti_ppm_create_pid (&pid_usb[indx]);
#else
    retVal = ti_ppd_create_pid (&pid_usb[indx]);
#endif
    if( retVal < 0 ){
       printk ("usb_add_pid: failed to add PID(%d), retVal  = %d.\n",
                pid_usb[indx].pid_handle, retVal);
    }
    pid_usb[indx].pid_handle = retVal;

    return pid_usb[indx].pid_handle;
}


/************************************************************************/
/*  INSTALL PID                                                         */
/************************************************************************/
static int install_usb_pid(struct net_device *net, int mode, bool restore_vpids)
{
    int pid_type,i,retVal;
    TI_PP_PID_RANGE  pid_range_usb;
    struct eth_dev          *dev = netdev_priv(net);
    
    if( mode == 1 )
    {
        pid_type = TI_PP_PID_TYPE_USBRNDIS; 		
    }
    else
    {
        if( dev->rndis_configured == 1)
            pid_type = TI_PP_PID_TYPE_USBRNDIS;
        else
            pid_type = TI_PP_PID_TYPE_USBCDC;
    }
    
    /* install usb pid ranges */
    pid_range_usb.type        = pid_type;
    pid_range_usb.port_num    = CPPI41_SRCPORT_USBEP0;
    pid_range_usb.count       = PP_USB_PID_COUNT;
    pid_range_usb.base_index  = PP_USB_PID_BASE;
    
    /* configure the usb pid range */
#ifdef CONFIG_TI_PACKET_PROCESSOR
    retVal = ti_ppm_config_pid_range (&pid_range_usb);
#else
    retVal = ti_ppd_config_pid_range (&pid_range_usb);
#endif
    if( retVal != 0 )
    {
        printk("%s: config_pid_range failed with error code %d.\n",__FUNCTION__, retVal);
    }

    /* create the pid */
    for(i=0; i<MAX_USB_ENDPOINT; ++i)
    {
#ifdef CONFIG_TI_PACKET_PROCESSOR
        net->pid_handle                    = usb_add_pid (i,pid_type);	
        net->vpid_block.type               = TI_PP_ETHERNET;
        net->vpid_block.parent_pid_handle  = net->pid_handle;
        net->vpid_block.egress_mtu         = 0;
        net->vpid_block.priv_tx_data_len   = 0;
#else /* !CONFIG_TI_PACKET_PROCESSOR */
        retVal = usb_add_pid (i,pid_type);
#endif
    }

#ifdef CONFIG_TI_PACKET_PROCESSOR
    if (restore_vpids)
    {
        for(i=0; i<MAX_USB_ENDPOINT; ++i)
        {
            int j;
            for (j=0; j<TI_PP_MAX_VPID;j++)
            {
                if (0xFF != vpid_usb[i][j].vpid_handle)
                {
                    net->vpid_handle = ti_ppm_create_vpid (&net->vpid_block);
                }

            }
        }
    }
#endif

    created_usb_pidtype = pid_type;
    return 0;	
}


/************************************************************************/
/*  UN-INSTALL PID                                                      */
/************************************************************************/
static int uninstall_usb_pid(struct net_device *net, int mode, bool save_vpids)
{
    
    int pid_type,i;
    TI_PP_PID_RANGE  pid_range_usb;
    int retval;
    struct eth_dev          *dev = netdev_priv(net);
    
    if( mode == 1 )
    {
        pid_type = TI_PP_PID_TYPE_USBRNDIS;
    }
    else
    {
        if( dev->rndis_configured == 1)
            pid_type = TI_PP_PID_TYPE_USBRNDIS;
        else
            pid_type = TI_PP_PID_TYPE_USBCDC;
    }
    
    /* install usb pid ranges */
    pid_range_usb.type        = pid_type;
    pid_range_usb.port_num    = CPPI41_SRCPORT_USBEP0;
    pid_range_usb.count       = PP_USB_PID_COUNT;
    pid_range_usb.base_index  = PP_USB_PID_BASE;
    
    /* delete all usb pids configured */
    for(i=0; i<MAX_USB_ENDPOINT; ++i)
    {
#ifdef CONFIG_TI_PACKET_PROCESSOR
        if (save_vpids)
        {
            int j;
            for (j=0; j<TI_PP_MAX_VPID;j++)
            {
                vpid_usb[i][j].vpid_handle = 0xFF;
            }

            ti_ppm_get_vpid  ( pid_usb[i].pid_handle, TI_PP_MAX_VPID, &vpid_usb[i][0] );
        }
        retval= ti_ppm_delete_pid( pid_usb[i].pid_handle );
#else
        retval= ti_ppd_delete_pid( pid_usb[i].pid_handle );
#endif
    }
    
    /* remove pid range for usb */
#ifdef CONFIG_TI_PACKET_PROCESSOR
    retval = ti_ppm_remove_pid_range(pid_range_usb.port_num);
#else
    retval = ti_ppd_remove_pid_range(pid_range_usb.port_num);
#endif
    
    /* Temporary Fix USB stability .. This is not the final fix ! */
    net->vpid_handle = -1;
    
    created_usb_pidtype = 0;
    mdelay(100);
    return 0;
}
#endif
