/*
  GPL LICENSE SUMMARY

  Copyright(c) 2011-2012 Intel Corporation.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
  The full GNU General Public License is included in this distribution
  in the file called LICENSE.GPL.

  Contact Information:
    Intel Corporation
    2200 Mission College Blvd.
    Santa Clara, CA  97052
*/


#define DRV_NAME        "L2Switch Network Interface driver"
#define DRV_VERSION     "0.1"

/* Enable only ONE of the following */
//#define L2SW_NETDEV_USE_NAPI
/*#define L2SW_NETDEV_USE_TASKLET*/

/*Replace the driver to use workq according to Intel*/
#define L2SW_NETDEV_USE_WORKQ

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/mii.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <asm/irq.h>
#include <linux/proc_fs.h>
#include <linux/if_vlan.h>
#include <linux/inet_lro.h>
#include <linux/ethtool.h>
#include <linux/kernel.h>
#include <linux/cat_l2switch_netdev.h>

#include "pal.h"
#include "pal_cppi41.h"
#include <puma6_cppi.h>

#ifdef CONFIG_TI_PACKET_PROCESSOR
#include "linux/ti_ppm.h"
#include "ti_ppd.h"
#include <puma6_pp.h>
#endif

#ifdef L2SW_NETDEV_USE_WORKQ
#include <linux/workqueue.h>
#include <linux/sched.h>

typedef struct
{
    struct work_struct work;
    struct net_device *dev;
} l2sw_rx_work_t;
#endif

//-------------------------------------------------------------------
//
// ########  ######## ######## #### ##    ## ########  ######
// ##     ## ##       ##        ##  ###   ## ##       ##    ##
// ##     ## ##       ##        ##  ####  ## ##       ##
// ##     ## ######   ######    ##  ## ## ## ######    ######
// ##     ## ##       ##        ##  ##  #### ##             ##
// ##     ## ##       ##        ##  ##   ### ##       ##    ##
// ########  ######## ##       #### ##    ## ########  ######
//
//-------------------------------------------------------------------
/* define to enable copious debugging info */
//#define L2SW_NETDEV_DEBUG


#ifdef L2SW_NETDEV_DEBUG
/* note: prints function name for you */
#  define DPRINTK(fmt, args...) printk("%s(%d): " fmt, __FUNCTION__ , __LINE__, ## args)
#else
#  define DPRINTK(fmt, args...)
#endif
#define ERR_PRINTK(fmt, args...) printk(KERN_ERR "%s: " fmt, __FUNCTION__ , ## args)

#ifdef CONFIG_INET_LRO
#define L2SW_NETDEV_MAX_LRO_ENTRIES                         8
#endif

#define L2SW_NETDEV_RX_SERVICE_MAX                          64

/* Accumulator definitions */
#define PAL_CPPI41_L2SW_ACC_MAX_PAGE_ENTRIES                32
#define PAL_CPPI41_L2SW_ACC_LIST_NULL_TERM                  0
#define PAL_CPPI41_L2SW_ACC_PACE_MODE_LASTINTR              1
#define PAL_CPPI41_L2SW_ACC_PACE_TICK_CNT                   40
#define PAL_CPPI41_L2SW_ACC_MAX_PAGE_COUNT                  2

//----------------------------------------------------------------------------------------------
//
//  ######  ######## ########  ##     ##  ######  ######## ##     ## ########  ########  ######
// ##    ##    ##    ##     ## ##     ## ##    ##    ##    ##     ## ##     ## ##       ##    ##
// ##          ##    ##     ## ##     ## ##          ##    ##     ## ##     ## ##       ##
//  ######     ##    ########  ##     ## ##          ##    ##     ## ########  ######    ######
//       ##    ##    ##   ##   ##     ## ##          ##    ##     ## ##   ##   ##             ##
// ##    ##    ##    ##    ##  ##     ## ##    ##    ##    ##     ## ##    ##  ##       ##    ##
//  ######     ##    ##     ##  #######   ######     ##     #######  ##     ## ########  ######
//
//----------------------------------------------------------------------------------------------
typedef struct l2sw_netdev_private_t
{
    struct net_device       *netdev;
#ifdef CONFIG_INET_LRO
    struct net_lro_mgr      lro_mgr;                                /* This entry must be first */
    struct net_lro_desc     lro_arr[L2SW_NETDEV_MAX_LRO_ENTRIES];
#endif
#if defined(L2SW_NETDEV_USE_NAPI)
    struct napi_struct      napi;
#elif defined(L2SW_NETDEV_USE_TASKLET)
    struct tasklet_struct   rx_tasklet;                             /* RX completion processing tasklet */
#elif defined(L2SW_NETDEV_USE_WORKQ)
    struct workqueue_struct *l2sw_rx_wq;
    l2sw_rx_work_t           l2sw_rx_work;
#endif
    PAL_Handle              palHnd;                                 /* The handle to PAL layer */

    /* TX egress queue (ingress queue for PrxPDSP) configuration */
    PAL_Cppi4QueueHnd       egressTxQHnd;

    /* RX complete (using the accumulator) configuration */
    unsigned int            rxCompleteAccCh;                        /* The Rx accumulator channel number */
    PAL_Cppi4AccChHnd       rxCompleteAccChHnd;                     /* The Rx accumulator channel handle */
    Cppi4HostDescLinux**    rxCompleteAccChList;                    /* Rx acc channel lists */
    Ptr                     rxCompleteAccChListBase;

    struct net_device_stats stats;
    spinlock_t              devlock;
    unsigned long           state;
}
l2sw_netdev_private_t;



//-----------------------------------------------------------------------------------------------------------------------------------------------
//
// ########  ########   #######  ########  #######  ######## ##    ## ########  ########  ######
// ##     ## ##     ## ##     ##    ##    ##     ##    ##     ##  ##  ##     ## ##       ##    ##
// ##     ## ##     ## ##     ##    ##    ##     ##    ##      ####   ##     ## ##       ##
// ########  ########  ##     ##    ##    ##     ##    ##       ##    ########  ######    ######
// ##        ##   ##   ##     ##    ##    ##     ##    ##       ##    ##        ##             ##
// ##        ##    ##  ##     ##    ##    ##     ##    ##       ##    ##        ##       ##    ##
// ##        ##     ##  #######     ##     #######     ##       ##    ##        ########  ######
//
//-----------------------------------------------------------------------------------------------------------------------------------------------

/************************************************/
/* Module Functions                             */
/************************************************/
static int __init   l2sw_driver_init(void);
static int          l2sw_driver_cppi_init(void);
#ifdef CONFIG_INET_LRO
static int          get_skb_hdr(struct sk_buff *skb, void **iphdr, void **tcph, u64 *hdr_flags, void *data);
#endif
static ssize_t      l2sw_driver_show_version(struct device_driver *drv, char *buf);
static void __exit  l2sw_driver_exit(void);

/************************************************/
/* Network Device Functions                     */
/************************************************/
static int          l2sw_netdev_cppi_init(struct net_device *dev);
static void         l2sw_netdev_setup(struct net_device *dev);
#if defined(L2SW_NETDEV_USE_NAPI)
static int          l2sw_netdev_poll(struct napi_struct *napi, int budget);
#endif
static int          l2sw_netdev_open(struct net_device *dev);
static int          l2sw_netdev_close(struct net_device *dev);
static struct net_device_stats *l2sw_netdev_get_stats(struct net_device *dev);
static int __devexit l2sw_netdev_remove(struct device *dev);

/************************************************/
/* Packet Processor Functions                   */
/************************************************/
#ifdef CONFIG_TI_PACKET_PROCESSOR
static int          l2sw_netdev_pp_prepare_pid(struct net_device *dev);
static int          l2sw_netdev_pp_set_pid_flags(struct net_device *dev, int flags);
static void         l2sw_netdev_pp_prepare_vpid(struct net_device *dev);
static int          l2sw_netdev_pp_setup_qos(struct net_device *dev);
static void         l2sw_netdev_pp_prepare_qos(struct net_device *dev);
static int          l2sw_netdev_pp_select_qos(struct sk_buff *skb);
static int          l2sw_netdev_pp_shutdown_qos(struct net_device *dev);
#endif // CONFIG_TI_PACKET_PROCESSOR

/************************************************/
/* TX Flow Functions                            */
/************************************************/
static int          l2sw_netdev_init_acc_chan(PAL_Handle palHnd, int chan_num, Cppi4Queue queue, PAL_Cppi4AccChHnd* acc_ch_hnd);
static int          l2sw_netdev_tx_start_xmit(struct sk_buff *skb, struct net_device *dev);
static unsigned int l2sw_netdev_tx_link_skb_to_desc(struct net_device* dev, Cppi4HostDescLinux* hostDesc, struct sk_buff *skb);

/************************************************/
/* RX Flow Functions                            */
/************************************************/
static int          l2sw_netdev_rx_open(struct net_device* dev);
static irqreturn_t  l2sw_netdev_rx_interrupt(int irq, void *dev);
#if defined(L2SW_NETDEV_USE_NAPI)
static int          l2sw_netdev_rx_complete(struct net_device* dev, int budget);
#elif defined(L2SW_NETDEV_USE_TASKLET)
static void         l2sw_netdev_rx_complete(unsigned long data);
#elif defined(L2SW_NETDEV_USE_WORKQ)
static void l2sw_netdev_rx_complete(struct work_struct *work);
#endif
static void         l2sw_netdev_rx_link_skb_to_desc(struct net_device* dev, Cppi4HostDescLinux* hostDesc, struct sk_buff *skb);

//-----------------------------------------------------------------------------------------------------------------------------------------------
//
//  ######   ##        #######  ########     ###    ##          ##     ##    ###    ########  ####    ###    ########  ##       ########  ######
// ##    ##  ##       ##     ## ##     ##   ## ##   ##          ##     ##   ## ##   ##     ##  ##    ## ##   ##     ## ##       ##       ##    ##
// ##        ##       ##     ## ##     ##  ##   ##  ##          ##     ##  ##   ##  ##     ##  ##   ##   ##  ##     ## ##       ##       ##
// ##   #### ##       ##     ## ########  ##     ## ##          ##     ## ##     ## ########   ##  ##     ## ########  ##       ######    ######
// ##    ##  ##       ##     ## ##     ## ######### ##           ##   ##  ######### ##   ##    ##  ######### ##     ## ##       ##             ##
// ##    ##  ##       ##     ## ##     ## ##     ## ##            ## ##   ##     ## ##    ##   ##  ##     ## ##     ## ##       ##       ##    ##
//  ######   ########  #######  ########  ##     ## ########       ###    ##     ## ##     ## #### ##     ## ########  ######## ########  ######
//
//-----------------------------------------------------------------------------------------------------------------------------------------------
static unsigned char *gL2switchNetdevNames[L2SW_NETDEV_NUM_INSTANCES] =
{
    L2SW_NETDEV_DATA0,
    L2SW_NETDEV_MGMT0
};

static struct platform_device *gL2switchPlatformDev;
static unsigned char defmac[] = {0x00, 0x50, 0xF1, 0x80, 0x00, 0x00};
static char *inpmac = NULL;     /* Allocation for MAC string supplied on the command line */

/* RX Infrastructure configuration */
static PAL_Cppi4QueueHnd    l2swInfraFDHostQHnd;
/* TX Infrastructure configuration */
static PAL_Cppi4QueueHnd    hostToPpInfraInputQHnd[PAL_CPPI41_SR_HOST_TO_PP_Q_COUNT];
static PAL_Cppi4QueueHnd    hostToPpFDHostQHnd[PAL_CPPI41_SR_HOST_TO_PP_Q_COUNT];

static const struct net_device_ops gL2switchNetdevOps =
{
    .ndo_open               = l2sw_netdev_open,
    .ndo_start_xmit         = l2sw_netdev_tx_start_xmit,
    .ndo_stop               = l2sw_netdev_close,
    .ndo_get_stats          = l2sw_netdev_get_stats,
    .ndo_validate_addr      = eth_validate_addr,
    .ndo_set_mac_address    = eth_mac_addr,
};

/* structure describing the L2Switch NID */
static struct device_driver gL2switchDevDrv =
{
    .name       = "L2switchDevDrv",
    .bus        = &platform_bus_type,
    .probe      = NULL,
    .remove     = l2sw_netdev_remove,
    .suspend    = NULL,
    .resume     = NULL,
};

#ifdef CONFIG_INET_LRO
/* Very minimal ethtool support for LRO */
static const struct ethtool_ops l2sw_netdev_ethtool_ops =
{
    .get_flags = ethtool_op_get_flags,
    .set_flags = ethtool_op_set_flags
};
#endif


#ifdef CONFIG_TI_PACKET_PROCESSOR
static TI_PP_QOS_CLST_CFG  l2sw_netdev_qos_cluster_db[L2SW_NETDEV_NUM_INSTANCES];
#endif

static DRIVER_ATTR(version, S_IRUGO, l2sw_driver_show_version, NULL);

//-----------------------------------------------------------------------------------------------------------------------------------------------
//
// ##        #######   ######     ###    ##          ######## ##     ## ##    ##  ######  ######## ####  #######  ##    ##  ######
// ##       ##     ## ##    ##   ## ##   ##          ##       ##     ## ###   ## ##    ##    ##     ##  ##     ## ###   ## ##    ##
// ##       ##     ## ##        ##   ##  ##          ##       ##     ## ####  ## ##          ##     ##  ##     ## ####  ## ##
// ##       ##     ## ##       ##     ## ##          ######   ##     ## ## ## ## ##          ##     ##  ##     ## ## ## ##  ######
// ##       ##     ## ##       ######### ##          ##       ##     ## ##  #### ##          ##     ##  ##     ## ##  ####       ##
// ##       ##     ## ##    ## ##     ## ##          ##       ##     ## ##   ### ##    ##    ##     ##  ##     ## ##   ### ##    ##
// ########  #######   ######  ##     ## ########    ##        #######  ##    ##  ######     ##    ####  #######  ##    ##  ######
//
//-----------------------------------------------------------------------------------------------------------------------------------------------

/**************************************************************************/
/*! \fn         l2sw_driver_init
 **************************************************************************
 *
 *  \brief      Network driver initialization
 *
 *  \param[in]  None
 *  \return     OK or Error
 **************************************************************************/
static int __init l2sw_driver_init(void)
{
    struct net_device *netdev = NULL;
    l2sw_netdev_private_t *priv = NULL;
    int devInstance;
    int ret;

    printk(KERN_INFO "Loading " DRV_NAME "\n");

    if (sizeof(Cppi4HostDescLinux) > PAL_CPPI41_SR_L2SW_INFRA_FD_HOST_DESC_SIZE)
    {
       ERR_PRINTK( "%s fundamentally broken. Contact maintainer!\n", DRV_NAME);
       ret = -EPERM;
       goto out;
    }

    /* Initialize MAC address */
    if (inpmac && strlen(inpmac) == 17)
    {
        int i;
        int m[6];

        /* Translate MAC address from ASCII to binary */
        sscanf(inpmac, "%x:%x:%x:%x:%x:%x", &(m[0]), &(m[1]), &(m[2]), &(m[3]), &(m[4]), &(m[5]));
        for (i = 0; i < 6; i++)
        {
            defmac[i] = (Uint8)m[i];
        }
    }
    DPRINTK("Using MAC %x:%x:%x:%x:%x:%x", defmac[0], defmac[1], defmac[2], defmac[3], defmac[4], defmac[5]);

    ret = l2sw_driver_cppi_init();
    if (ret)
    {
        ERR_PRINTK("l2sw_driver_cppi_init failed, exiting\n");
        goto out;
    }

    gL2switchPlatformDev = platform_device_register_simple("L2switchDevDrv", -1, NULL, 0);
    if (IS_ERR(gL2switchPlatformDev))
    {
        ERR_PRINTK("Unable to register platform device\n");
        ret = PTR_ERR(gL2switchPlatformDev);
        goto out;
    }

    ret = driver_register(&gL2switchDevDrv);
    if (ret)
    {
        ERR_PRINTK("Unable to register driver, ret=%d\n", ret);
        goto err_driver_register;
    }

    ret = driver_create_file(&gL2switchDevDrv, &driver_attr_version);
    if (ret)
    {
        ERR_PRINTK("Unable to create file, ret=%d\n", ret);
        goto err_driver_create_file;
    }

    for (devInstance = 0; devInstance < L2SW_NETDEV_NUM_INSTANCES; devInstance++)
    {
        netdev = alloc_netdev(sizeof(l2sw_netdev_private_t), gL2switchNetdevNames[devInstance], l2sw_netdev_setup);
        if (netdev == NULL)
        {
            ERR_PRINTK("Unable to alloc new net device named %s (device instance = %d)\n", gL2switchNetdevNames[devInstance], devInstance);
            ret = -ENOMEM;
            goto err_alloc_netdev;
        }

        SET_NETDEV_DEV(netdev, &(gL2switchPlatformDev->dev));
        platform_set_drvdata(gL2switchPlatformDev, netdev);

        priv = netdev_priv(netdev);
        priv->netdev = netdev;
        netdev->devInstance = devInstance;

        // hard_addr_setup
        defmac[ETH_ALEN-1] = 0x10 + devInstance; // Make address unique per device
        memcpy(netdev->dev_addr, defmac, netdev->addr_len);
        memcpy(netdev->perm_addr, netdev->dev_addr, netdev->addr_len);

        ret = register_netdev(netdev);
        if(ret)
        {
            ERR_PRINTK("Unable to register device named %s (%p)...\n", netdev->name, netdev);
            goto err_register_netdev;
        }

#if defined(L2SW_NETDEV_USE_NAPI)
        netif_napi_add(netdev, &priv->napi, l2sw_netdev_poll, L2SW_NETDEV_RX_SERVICE_MAX);
#endif

        DPRINTK("Registered device named %s (%p)...\n", netdev->name, netdev);

        ret = l2sw_netdev_cppi_init(netdev);
        if(ret)
        {
            ERR_PRINTK("l2sw_netdev_cppi_init failed for device named %s (%p)...\n", netdev->name, netdev);
            goto err_register_netdev;
        }

#ifdef CONFIG_TI_PACKET_PROCESSOR
        l2sw_netdev_pp_prepare_pid(netdev);
        l2sw_netdev_pp_prepare_vpid(netdev);
        l2sw_netdev_pp_prepare_qos(netdev);
#endif

        spin_lock_init(&priv->devlock);

#ifdef CONFIG_INET_LRO
        /* LRO Setup */
        priv->lro_mgr.dev = netdev;
        memset(&priv->lro_mgr.stats, 0, sizeof(priv->lro_mgr.stats));
        priv->lro_mgr.features = LRO_F_NAPI;
        priv->lro_mgr.ip_summed = CHECKSUM_UNNECESSARY;
        priv->lro_mgr.ip_summed_aggr = CHECKSUM_UNNECESSARY; //CHECKSUM_NONE;
        priv->lro_mgr.max_desc = ARRAY_SIZE(priv->lro_arr);
        priv->lro_mgr.max_aggr = 32;
        priv->lro_mgr.frag_align_pad = 0;
        priv->lro_mgr.lro_arr = priv->lro_arr;
        priv->lro_mgr.get_skb_header = get_skb_hdr;
        memset(&priv->lro_arr, 0, sizeof(priv->lro_arr));

        /* Disable LRO by default */
        netdev->features &= ~NETIF_F_LRO;

        /* Init ethtool options */
        SET_ETHTOOL_OPS(netdev, (struct ethtool_ops *)&l2sw_netdev_ethtool_ops);
#endif
    }

    DPRINTK("Finished successfully\n");

    return 0;

/* Error Fallback */
err_register_netdev:
    while(devInstance-- >= 0)
    {
        netdev = dev_get_by_name(&init_net, gL2switchNetdevNames[devInstance]);
        if (netdev)
        {
            priv = netdev_priv(netdev);
            if (priv->palHnd)
            {
                PAL_cppi4Exit(priv->palHnd, NULL);
            }
#if defined(L2SW_NETDEV_USE_NAPI)
            netif_napi_del(&priv->napi);
#endif
            dev_put(netdev);
            free_netdev(netdev);
        }
    }
err_alloc_netdev:
    driver_remove_file(&gL2switchDevDrv, &driver_attr_version);
err_driver_create_file:
    driver_unregister(&gL2switchDevDrv);
err_driver_register:
    platform_device_unregister(gL2switchPlatformDev);
out:
    return ret;
}

/**************************************************************************/
/*! \fn         l2sw_driver_cppi_init
 **************************************************************************
 *
 *  \brief      L2Switch driver CPPI initialization
 *
 *  \param[in]  None
 *  \return     OK or Error
 **************************************************************************/
static int l2sw_driver_cppi_init(void)
{
    PAL_Handle          palHandle;
    Ptr                 descRegionPtr = NULL;
    Cppi4HostDescLinux* currHostDesc;
    Uint32              descCount;
    Cppi4Queue          freeHoseDescQueue;
    Cppi4Queue          queue;
    Uint32              infraChCount;
    Uint32              i;

    palHandle = PAL_cppi4Init( NULL, CPPI41_DOMAIN_PRIMARY_SR );

    /*************************************************************/
    /* Setup Free Host Descriptor Queue for L2SW Infrastructures */
    /*************************************************************/

    /* Allocate memory for the descriptors from the descriptors region */
    descRegionPtr = PAL_cppi4AllocDesc(palHandle, PAL_CPPI41_QUEUE_MGR_PARTITION_SR, PAL_CPPI41_SR_L2SW_INFRA_FD_HOST_DESC_COUNT, PAL_CPPI41_SR_L2SW_INFRA_FD_HOST_DESC_SIZE);
    if (!descRegionPtr)
    {
        ERR_PRINTK("Host descriptor region allocation FAILED\n");
        return -ENOMEM;
    }

    /* Open the free descriptors host queue */
    freeHoseDescQueue.qMgr = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
    freeHoseDescQueue.qNum = PAL_CPPI41_SR_L2SW_INFRA_FD_HOST_Q_NUM;
    if(!(l2swInfraFDHostQHnd = PAL_cppi4QueueOpen(palHandle, freeHoseDescQueue)))
    {
        ERR_PRINTK("Unable to open free desc queue #%d\n", freeHoseDescQueue.qNum);
        return -ENOMEM;
    }

    /* Format the descriptors with skb buffers and push into free queue */
    currHostDesc = (Cppi4HostDescLinux*)descRegionPtr;
    for (descCount = 0; descCount < PAL_CPPI41_SR_L2SW_INFRA_FD_HOST_DESC_COUNT; descCount++)
    {
        struct sk_buff* skb;

        PAL_osMemSet(currHostDesc, 0, PAL_CPPI41_SR_L2SW_INFRA_FD_HOST_DESC_SIZE);

        currHostDesc->hw.descInfo    = (PAL_CPPI4_HOSTDESC_DESC_TYPE_HOST << PAL_CPPI4_HOSTDESC_DESC_TYPE_SHIFT);
        currHostDesc->hw.tagInfo     = 0x3FFF;
        currHostDesc->hw.pktInfo     = (PAL_CPPI4_HOSTDESC_PKT_TYPE_ETH        << PAL_CPPI4_HOSTDESC_PKT_TYPE_SHIFT)
                                     | (PAL_CPPI4_HOSTDESC_PKT_RETPLCY_LINKED  << PAL_CPPI4_HOSTDESC_PKT_RETPLCY_SHIFT)
                                     | (PAL_CPPI4_HOSTDESC_DESC_LOC_OFFCHIP    << PAL_CPPI4_HOSTDESC_DESC_LOC_SHIFT)
                                     | (PAL_CPPI41_QUEUE_MGR_PARTITION_SR      << PAL_CPPI4_HOSTDESC_PKT_RETQMGR_SHIFT)
                                     | (PAL_CPPI41_SR_L2SW_INFRA_FD_HOST_Q_NUM << PAL_CPPI4_HOSTDESC_PKT_RETQNUM_SHIFT);


        skb = dev_alloc_skb( PAL_CPPI41_SR_L2SW_INFRA_FD_HOST_BUFFER_SIZE );

        if (NULL == skb)
        {
            ERR_PRINTK("The SKB allocation FAILED\n");
            goto out;
        }

        skb_reserve(skb, NET_IP_ALIGN);    /* 16 bit align the IP fields. */
        currHostDesc->hw.orgBuffLen  = PAL_CPPI41_SR_L2SW_INFRA_FD_HOST_BUFFER_SIZE - NET_IP_ALIGN;
        currHostDesc->hw.orgBufPtr   = PAL_CPPI4_VIRT_2_PHYS(skb->data);
        currHostDesc->skb = skb;

        PAL_CPPI4_CACHE_WRITEBACK(currHostDesc, PAL_CPPI41_SR_L2SW_INFRA_FD_HOST_DESC_SIZE);

        PAL_cppi4QueuePush (l2swInfraFDHostQHnd,
                            (Ptr)PAL_CPPI4_VIRT_2_PHYS((Uint32)currHostDesc),
                            PAL_CPPI4_DESCSIZE_2_QMGRSIZE(PAL_CPPI41_SR_L2SW_INFRA_FD_HOST_DESC_SIZE),
                            0);

        currHostDesc = (Cppi4HostDescLinux*)((Uint32)currHostDesc + PAL_CPPI41_SR_L2SW_INFRA_FD_HOST_DESC_SIZE);
    }

    /*****************************************/
    /* Open & Enable Infrastructure Channels */
    /*****************************************/
    for (infraChCount = 0; infraChCount < PAL_CPPI41_SR_L2SW_INFRA_DMA_CH_COUNT; infraChCount++)
    {
        /* Open and enable Infrastructure TX Channels */
        volatile Cppi4TxChInitCfg   infraTxChInfo;
        PAL_Cppi4TxChHnd            infraTxChHdl;
        volatile Cppi4RxChInitCfg   infraRxChInfo;
        PAL_Cppi4RxChHnd            infraRxChHdl;

        infraTxChInfo.chNum         = PAL_CPPI41_SR_L2SW_INFRA_DMA_TX_CH_NUM(infraChCount);
        infraTxChInfo.dmaNum        = PAL_CPPI41_DMA_BLOCK2;
        infraTxChInfo.tdQueue.qMgr  = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
        infraTxChInfo.tdQueue.qNum  = PAL_CPPI41_SR_DMA_FD_TEARDOWN_Q_NUM;
        infraTxChInfo.defDescType   = CPPI41_DESC_TYPE_EMBEDDED;

        DPRINTK(" Call PAL_cppi4TxChOpen channel=%d\n", infraTxChInfo.chNum);

        infraTxChHdl = PAL_cppi4TxChOpen( palHandle, (Cppi4TxChInitCfg *)(&infraTxChInfo), NULL);
        if(infraTxChHdl == NULL)
        {
            ERR_PRINTK(" Unable to open TX channel %d\n", infraTxChInfo.chNum);
            goto out;
        }
        PAL_cppi4EnableTxChannel (infraTxChHdl, NULL);


        /* Open L2Switch Host RX Queues */
        queue.qMgr = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
        queue.qNum = PAL_CPPI41_SR_L2SW_HOST_RX_Q_NUM(infraChCount);
        if (NULL == PAL_cppi4QueueOpen( palHandle, queue))
        {
            ERR_PRINTK(" Unable to open queue %d \n", queue.qNum);
            goto out;
        }

        /* Open and enable Infrastructure RX Channels */
        infraRxChInfo.chNum            = PAL_CPPI41_SR_L2SW_INFRA_DMA_RX_CH_NUM(infraChCount);
        infraRxChInfo.dmaNum           = PAL_CPPI41_DMA_BLOCK2;
        infraRxChInfo.rxCompQueue.qMgr = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
        infraRxChInfo.rxCompQueue.qNum = PAL_CPPI41_SR_L2SW_HOST_RX_Q_NUM(infraChCount);
        infraRxChInfo.sopOffset = 0;
        infraRxChInfo.defDescType = CPPI41_DESC_TYPE_HOST;
        infraRxChInfo.retryOnStarvation = 0;
        infraRxChInfo.u.hostPktCfg.fdbQueue[0] = freeHoseDescQueue;
        infraRxChInfo.u.hostPktCfg.fdbQueue[1] = freeHoseDescQueue;
        infraRxChInfo.u.hostPktCfg.fdbQueue[2] = freeHoseDescQueue;
        infraRxChInfo.u.hostPktCfg.fdbQueue[3] = freeHoseDescQueue;

        DPRINTK(" Call PAL_cppi4RxChOpen channel=%d\n", infraRxChInfo.chNum);

        infraRxChHdl = PAL_cppi4RxChOpen ( palHandle, (Cppi4RxChInitCfg *)(&infraRxChInfo), NULL);
        if(infraRxChHdl == NULL)
        {
            ERR_PRINTK(" Unable to open RX channel %d\n", infraRxChInfo.chNum);
            goto out;
        }
        PAL_cppi4EnableRxChannel (infraRxChHdl, NULL);
    }

    /****************************************************************/
    /* Get handlers for TX Infrastructure                           */
    /****************************************************************/
    for (i = 0; PAL_CPPI41_SR_HOST_TO_PP_Q_COUNT > i; i++)
    {
        /***********************************************/
        /* Initialize Host to QPDSP Priority Tx queues */
        /***********************************************/
        queue.qNum = PAL_CPPI41_SR_HOST_TO_PP_INFRA_INPUT_Q_NUM(i);

        if(!(hostToPpInfraInputQHnd[i] = PAL_cppi4QueueOpen(palHandle, queue)))
        {
            ERR_PRINTK("unable to open Tx QOS Queue %d\n", queue.qNum);
            return -ENOMEM;
        }

        /************************************************/
        /* Initialize Free Host Descriptors Tx queues   */
        /************************************************/
        queue.qNum = PAL_CPPI41_SR_HOST_TO_PP_FD_HOST_Q_NUM(i);

        if(!(hostToPpFDHostQHnd[i] = PAL_cppi4QueueOpen(palHandle, queue)))
        {
            ERR_PRINTK("unable to open Tx Queue #%d\n", queue.qNum);
            return -ENOMEM;
        }
    }

    return 0;

out:
    /* Release previous allocated skbs */
    while((currHostDesc = (Cppi4HostDescLinux *)PAL_cppi4QueuePop(l2swInfraFDHostQHnd)))
    {
        dev_kfree_skb(currHostDesc->skb);
    }

    PAL_cppi4DeallocDesc(palHandle, PAL_CPPI41_QUEUE_MGR_PARTITION_SR, descRegionPtr);

    return -ENOMEM;
}

/**************************************************************************/
/*! \fn         l2sw_netdev_cppi_init
 **************************************************************************
 *
 *  \brief      Network device CPPI initialization
 *
 *  \param[in]  None
 *  \return     OK or Error
 **************************************************************************/
static int l2sw_netdev_cppi_init(struct net_device *dev)
{
    l2sw_netdev_private_t* priv = netdev_priv(dev);
    Cppi4Queue queue;   /* used generically */

    priv->palHnd = PAL_cppi4Init(NULL, NULL);

    /************************************************/
    /* Initialize PrxPDSP ingress queue             */
    /************************************************/
    queue.qMgr = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
    queue.qNum = PAL_CPPI41_SR_L2SW_PrxPDSP_Q_NUM(dev->devInstance);

    if(!(priv->egressTxQHnd = PAL_cppi4QueueOpen(priv->palHnd, queue)))
    {
        ERR_PRINTK("unable to open Tx Queue #%d for device named %s\n", queue.qNum, dev->name);
        return -ENOMEM;
    }

    return 0;
}


#ifdef CONFIG_INET_LRO

/**************************************************************************/
/*! \fn         get_skb_hdr
 **************************************************************************
 *
 *  \brief      gets skb header
 *
 *  \param[in]  SK buff
 *  \param[out] IP header
 *  \param[out] TCP header
 *  \param[out] Header flags
 *  \return     OK or Error
 **************************************************************************/
static int get_skb_hdr(struct sk_buff *skb, void **iphdr, void **tcph, u64 *hdr_flags, void *data)
{
    /* Check that this is an ethernet packet */
    if(skb->protocol != ntohs(ETH_P_IP) )
    {
        return -1;
    }

    if( !skb->nh.iph )
    {
        if( !skb->mac.raw )
        {
            return -1;
        }
        /* In case the pointers are not initialized */
        skb->nh.iph = (struct iphdr *)(skb->mac.raw + ETH_HLEN);
        skb->h.th = (struct tcphdr *)(( (unsigned char *)(skb->nh.iph)+skb->nh.iph->ihl*4));
    }

    /* Continue only if its TCP */
    if( skb->nh.iph->protocol != IPPROTO_TCP )
    {
        return -1;
    }

    if(skb->nh.iph->version == 4)
    {
        *hdr_flags = LRO_IPV4;
    }

    *tcph = (void *)skb->h.th;
    *iphdr = (void *)skb->nh.iph;
    *hdr_flags |= LRO_TCP;

    return 0;
}

#endif // CONFIG_INET_LRO

#if defined(L2SW_NETDEV_USE_NAPI)
/**************************************************************************/
/*! \fn         l2sw_netdev_poll
 **************************************************************************
 *
 *  \brief      Network device polling function
 *
 *  \param[in]  NAPI
 *  \param[in]  Processed packets budget
 *  \return     Number of processed packets
 **************************************************************************/
static int l2sw_netdev_poll(struct napi_struct *napi, int budget)
{
    int work_done;
    unsigned long flags;
    l2sw_netdev_private_t *priv = container_of(napi, l2sw_netdev_private_t, napi);

    work_done = l2sw_netdev_rx_complete(priv->netdev, budget);

    /* order is important here. If we do EOI before calling netif_rx_complete, an interrupt
     * can occur just before we take ourselves out of the poll list; we will not
     * schedule NAPI thread on that interrupt, no further Rx interrupts and
     * Rx will stall forever. Scary...
     * */
    if (work_done < budget)
    {
        napi_complete(napi);

        /* Accumulator looks at INTD counter in order to know if it can issue another interrupt.
           Since we decrement the counter at l2sw_netdev_rx_complete it is possible that accumulator issued another interrupt.
           Due to the fact that interrupt is level and we do not want to get a false interrupt, we clear the INTC at the end of l2sw_netdev_rx_complete.
           Next time INTC will wait for INTD to become active.
           But, since INTD is level there is a possibility that INTD will remain active.
           This can happen if accumulator issues an interrupt before the host sent EOI (this is done in next line of code).
           So, in this case we have INTD status not changed - still active, while INTC now waits for it to become active.
           This can lead to not getting the interrupt forever. This is why we must check if counter>0 and if so re-schedule NAPI.
           We lock the interrupts b4 doing EOI and up until NAPI schedule in order not to get double interrupt in the case that
           an interrupt is really issued between EOI and checking INTD count - we are going to reschedule NAPI anyway... */

        spin_lock_irqsave(&priv->devlock, flags);
        ack_irq(MAP_INTD_TO_INTC(PAL_CPPI41_L2SW_ACC_INTV_NUM(priv->netdev->devInstance)));
        enable_irq(MAP_INTD_TO_INTC(PAL_CPPI41_L2SW_ACC_INTV_NUM(priv->netdev->devInstance)));

        avalanche_intd_write_eoi(PAL_CPPI41_L2SW_ACC_INTV_NUM(priv->netdev->devInstance));
        if (avalanche_intd_get_interrupt_count(0, priv->rxCompleteAccCh))
        {
            if (likely(napi_schedule_prep(napi)))
            {
                disable_irq_nosync(MAP_INTD_TO_INTC(PAL_CPPI41_L2SW_ACC_INTV_NUM(priv->netdev->devInstance)));
                __napi_schedule(napi);
            }
        }
        spin_unlock_irqrestore(&priv->devlock, flags);
    }
#ifdef CONFIG_INET_LRO
    if( dev->features & NETIF_F_LRO )
    {
        lro_flush_all(&priv->lro_mgr);
    }
#endif

    return work_done;
}
#endif

/**************************************************************************/
/*! \fn         l2sw_driver_show_version
 **************************************************************************
 *
 *  \brief      Network driver version
 *
 *  \param[in]  Network driver
 *  \param[out] Buffer holding version
 *  \return     OK or Error
 **************************************************************************/
static ssize_t l2sw_driver_show_version(struct device_driver *drv, char *buf)
{
    return sprintf(buf, "%s, version: %s", DRV_NAME, DRV_VERSION);
}

/**************************************************************************/
/*! \fn         l2sw_driver_exit
 **************************************************************************
 *
 *  \brief      Network driver exit
 *
 *  \param[in]  None
 *  \return     None
 **************************************************************************/
static void __exit l2sw_driver_exit(void)
{
    driver_remove_file(&gL2switchDevDrv, &driver_attr_version);
    driver_unregister(&gL2switchDevDrv);
}

/**************************************************************************/
/*! \fn         l2sw_netdev_setup
 **************************************************************************
 *
 *  \brief      Network device setup
 *
 *  \param[in]  Net Device
 *  \return     OK or Error
 **************************************************************************/
static void l2sw_netdev_setup(struct net_device *dev)
{

     dev->netdev_ops = &gL2switchNetdevOps;

     ether_setup(dev);
}

/**************************************************************************/
/*! \fn         l2sw_netdev_open
 **************************************************************************
 *
 *  \brief      Network device open
 *
 *  \param[in]  Net Device
 *  \return     OK or Error
 **************************************************************************/
static int l2sw_netdev_open(struct net_device *dev)
{
    int ret;
    l2sw_netdev_private_t* priv = netdev_priv(dev);

    DPRINTK("for device named %s\n", dev->name);

    /* clear the state bit. We are getting opened */
    clear_bit(0, &priv->state);

#ifdef CONFIG_TI_PACKET_PROCESSOR
    l2sw_netdev_pp_set_pid_flags(dev, 0);
#endif

   // spin_lock_irqsave(&priv->devlock, flags);

    if((ret = l2sw_netdev_rx_open(dev)))
    {
        //spin_unlock_irqrestore(&priv->devlock, flags);
        return ret;
    }

    netif_start_queue(dev);

   // spin_unlock_irqrestore(&priv->devlock, flags);

    return 0;
}

/**************************************************************************/
/*! \fn         l2sw_netdev_close
 **************************************************************************
 *
 *  \brief      Network device closure
 *
 *  \param[in]  Network device
 *  \return     OK or Error
 **************************************************************************/
static int l2sw_netdev_close(struct net_device *dev)
{
    l2sw_netdev_private_t* priv = netdev_priv(dev);

#if defined(L2SW_NETDEV_USE_NAPI)
    napi_disable(&priv->napi);
#elif defined(L2SW_NETDEV_USE_TASKLET)
    tasklet_kill(&priv->rx_tasklet);
#elif defined(L2SW_NETDEV_USE_WORKQ)
    flush_workqueue(priv->l2sw_rx_wq);
    destroy_workqueue(priv->l2sw_rx_wq);
#endif

    netif_stop_queue(dev);

#ifdef CONFIG_TI_PACKET_PROCESSOR
    l2sw_netdev_pp_set_pid_flags(dev, TI_PP_PID_DISCARD_ALL_RX);
#endif

    disable_irq(MAP_INTD_TO_INTC(PAL_CPPI41_L2SW_ACC_INTV_NUM(dev->devInstance)));

    PAL_cppi4AccChClose(priv->rxCompleteAccChHnd, NULL);

   // spin_lock_irqsave(&priv->devlock, flags);
    set_bit(0, &priv->state);

#if defined(L2SW_NETDEV_USE_NAPI)
    if(l2sw_netdev_rx_complete(dev, PAL_CPPI41_SR_L2SW_INFRA_FD_HOST_DESC_COUNT))
    {
        avalanche_intd_write_eoi(PAL_CPPI41_L2SW_ACC_INTV_NUM(dev->devInstance));
    }
#elif defined(L2SW_NETDEV_USE_TASKLET)
    l2sw_netdev_rx_complete((unsigned long)dev);
#endif

  //  spin_unlock_irqrestore(&priv->devlock, flags);

    kfree(priv->rxCompleteAccChListBase);

    free_irq(MAP_INTD_TO_INTC(PAL_CPPI41_L2SW_ACC_INTV_NUM(dev->devInstance)), dev);

    return 0;
}

/**************************************************************************/
/*! \fn         l2sw_netdev_get_stats
 **************************************************************************
 *
 *  \brief      Network device statistics
 *
 *  \param[in]  Network device
 *  \return     statistics structure
 **************************************************************************/
static struct net_device_stats *l2sw_netdev_get_stats(struct net_device *dev)
{
    l2sw_netdev_private_t* priv = netdev_priv((struct net_device*) dev);

    return &priv->stats;
}

/**************************************************************************/
/*! \fn         l2sw_netdev_remove
 **************************************************************************
 *
 *  \brief      Network device removal
 *
 *  \param[in]  Network device
 *  \return     OK or Error
 **************************************************************************/
static int __devexit l2sw_netdev_remove(struct device *dev)
{
    driver_remove_file(&gL2switchDevDrv, &driver_attr_version);
    driver_unregister(&gL2switchDevDrv);
    platform_device_unregister(gL2switchPlatformDev);

    return 0;
}

#ifdef CONFIG_TI_PACKET_PROCESSOR
/**************************************************************************/
/*! \fn         l2sw_netdev_pp_prepare_pid
 **************************************************************************
 *
 *  \brief      Packet Processor PID creation
 *
 *  \param[in]  Network Device
 *  \param[in]  PID index = Network device index
 *  \return     OK or Error
 **************************************************************************/
static int l2sw_netdev_pp_prepare_pid(struct net_device *dev)
{
    int         ret_val;
    TI_PP_PID_RANGE  pid_range_l2sw_netdev;
    TI_PP_PID   l2sw_netdev_pid;

    /* Config L2Switch Network Devices PID range */
    /* ----------------------------------------- */
    pid_range_l2sw_netdev.type        = TI_PP_PID_TYPE_ETHERNET;
    pid_range_l2sw_netdev.port_num    = dev->devInstance;
    pid_range_l2sw_netdev.count       = 1;
    pid_range_l2sw_netdev.base_index  = dev->devInstance;

    if ((ret_val = ti_ppm_config_pid_range(&pid_range_l2sw_netdev)))
    {
        ERR_PRINTK("config_pid_range failed with error code %d\n", ret_val);
        return ret_val;
    }

    /*
     * Create L2Switch NID PID
     * -----------------------
     */
    l2sw_netdev_pid.type            = TI_PP_PID_TYPE_ETHERNET;
    l2sw_netdev_pid.ingress_framing = TI_PP_PID_INGRESS_ETHERNET
                                    | TI_PP_PID_INGRESS_PPPOE
                                    | TI_PP_PID_INGRESS_IPV6
                                    | TI_PP_PID_INGRESS_IPV4
                                    | TI_PP_PID_INGRESS_IPOE;
    l2sw_netdev_pid.pri_mapping     = 1;    /* Num prio Qs for fwd */
    l2sw_netdev_pid.dflt_pri_drp    = 0;
    l2sw_netdev_pid.dflt_dst_tag    = 0x3FFF;
    l2sw_netdev_pid.dflt_fwd_q      = PAL_CPPI41_SR_L2SW_INFRA_INPUT_LOW_Q_NUM(dev->devInstance);
    l2sw_netdev_pid.tx_pri_q_map[0] = PAL_CPPI41_SR_L2SW_PrxPDSP_Q_NUM(dev->devInstance);
    l2sw_netdev_pid.tx_hw_data_len  = 0;
    l2sw_netdev_pid.pid_handle      = pid_range_l2sw_netdev.base_index;

    if ((ret_val = ti_ppm_create_pid(&l2sw_netdev_pid)) < 0)
    {
        ERR_PRINTK("create_pid for pidIndex %d failed with error code %d\n", PP_L2SW_PID_BASE + dev->devInstance, ret_val);
        l2sw_netdev_pid.pid_handle = -1;
    }
    dev->pid_handle = l2sw_netdev_pid.pid_handle;

    return ret_val;
}

/**************************************************************************/
/*! \fn         l2sw_netdev_pp_set_pid_flags
 **************************************************************************
 *
 *  \brief      Packet Processor PID flags configuration
 *
 *  \param[in]  Network Device
 *  \param[in]  PID flags
 *  \return     OK or Error
 **************************************************************************/
static int l2sw_netdev_pp_set_pid_flags(struct net_device *dev, int flags)
{
    ti_ppm_set_pid_flags(dev->pid_handle, flags);

    /* this delay is to make sure all the packets with the PID successfully egress throgh the respective ports.*/
    mdelay(200);
    return 0;
}

/**************************************************************************/
/*! \fn         l2sw_netdev_pp_prepare_vpid
 **************************************************************************
 *
 *  \brief      Packet Processor VPID preperation
 *
 *  \param[in]  Network Device
 *  \return     OK or Error
 **************************************************************************/
static void l2sw_netdev_pp_prepare_vpid(struct net_device *dev)
{
    /*
     * Create L2Switch NID VPIDs
     * -------------------------
     */
    dev->vpid_block.type               = TI_PP_ETHERNET;
    dev->vpid_block.parent_pid_handle  = dev->pid_handle;
    dev->vpid_block.egress_mtu         = 0;
    dev->vpid_block.priv_tx_data_len   = 0;
}

/**************************************************************************/
/*! \fn         l2sw_netdev_pp_select_qos
 **************************************************************************
 *
 *  \brief      Packet Processor QoS selection
 *
 *  \param[in]  Network Device
 *  \return     OK or Error
 **************************************************************************/
static int l2sw_netdev_pp_select_qos(struct sk_buff *skb)
{
    skb->pp_packet_info.ti_session.cluster = 0;
    skb->pp_packet_info.ti_session.priority = PAL_CPPI41_SR_L2SW_QPDSP_QOS_Q_PRIORITY_LOW - (skb->pp_packet_info.ti_session.priority >> 1);

    return 0;
}

/**************************************************************************/
/*! \fn         l2sw_netdev_pp_setup_qos
 **************************************************************************
 *
 *  \brief      Packet Processor QoS setup
 *
 *  \param[in]  Network Device
 *  \return     OK or Error
 **************************************************************************/
static int l2sw_netdev_pp_setup_qos(struct net_device *dev)
{
    int                 rc;
    unsigned int        devInstance = dev->devInstance;
    TI_PP_QOS_QUEUE     *qcfg;

    DPRINTK("for device %s (devInstance=%d)\n", dev->name, devInstance);

    l2sw_netdev_qos_cluster_db[devInstance].qos_q_cnt  = PAL_CPPI41_SR_L2SW_QPDSP_QOS_Q_PRIORITIES_COUNT;

    // Setup PAL_CPPI41_SR_L2SW_QPDSP_QOS_Q_PRIORITIES_COUNT(4) QOS queues (for each device), one that gets line speed and then trickles down to the other.

    // Queue 0
    qcfg = &l2sw_netdev_qos_cluster_db[devInstance].qos_q_cfg[PAL_CPPI41_SR_L2SW_QPDSP_QOS_Q_PRIORITY_HIGH];
    qcfg->q_num         = PAL_CPPI41_SR_L2SW_QPDSP_QOS_Q_OFFSET(devInstance, PAL_CPPI41_SR_L2SW_QPDSP_QOS_Q_PRIORITY_HIGH);
    qcfg->flags         = 0;
    qcfg->egr_q         = PAL_CPPI41_SR_L2SW_PrxPDSP_Q_OFFSET(devInstance);
    qcfg->it_credit     = (1000*1024*1024)/40000/8; /* <link speed> / <PP ticks per sec> / <8 bits in byte> */
    qcfg->max_credit    = MAX_IP_PACKET_SIZE * 2;
    qcfg->congst_thrsh  = MAX_IP_PACKET_SIZE * 32;
    qcfg->congst_thrsh_packets = 32;

    // Queue 1
    qcfg = &l2sw_netdev_qos_cluster_db[devInstance].qos_q_cfg[PAL_CPPI41_SR_L2SW_QPDSP_QOS_Q_PRIORITY_MEDHIGH];
    qcfg->q_num         = PAL_CPPI41_SR_L2SW_QPDSP_QOS_Q_OFFSET(devInstance, PAL_CPPI41_SR_L2SW_QPDSP_QOS_Q_PRIORITY_MEDHIGH);
    qcfg->flags         = 0;
    qcfg->egr_q         = PAL_CPPI41_SR_L2SW_PrxPDSP_Q_OFFSET(devInstance);
    qcfg->it_credit     = 0;
    qcfg->max_credit    = MAX_IP_PACKET_SIZE * 2;
    qcfg->congst_thrsh  = MAX_IP_PACKET_SIZE * 32;
    qcfg->congst_thrsh_packets = 32;

    // Queue 2
    qcfg = &l2sw_netdev_qos_cluster_db[devInstance].qos_q_cfg[PAL_CPPI41_SR_L2SW_QPDSP_QOS_Q_PRIORITY_MEDLOW];
    qcfg->q_num         = PAL_CPPI41_SR_L2SW_QPDSP_QOS_Q_OFFSET(devInstance, PAL_CPPI41_SR_L2SW_QPDSP_QOS_Q_PRIORITY_MEDLOW);
    qcfg->flags         = 0;
    qcfg->egr_q         = PAL_CPPI41_SR_L2SW_PrxPDSP_Q_OFFSET(devInstance);
    qcfg->it_credit     = 0;
    qcfg->max_credit    = MAX_IP_PACKET_SIZE * 2;
    qcfg->congst_thrsh  = MAX_IP_PACKET_SIZE * 64;
    qcfg->congst_thrsh_packets = 64;

    // Queue 3
    qcfg = &l2sw_netdev_qos_cluster_db[devInstance].qos_q_cfg[PAL_CPPI41_SR_L2SW_QPDSP_QOS_Q_PRIORITY_LOW];
    qcfg->q_num         = PAL_CPPI41_SR_L2SW_QPDSP_QOS_Q_OFFSET(devInstance, PAL_CPPI41_SR_L2SW_QPDSP_QOS_Q_PRIORITY_LOW);
    qcfg->flags         = 0;
    qcfg->egr_q         = PAL_CPPI41_SR_L2SW_PrxPDSP_Q_OFFSET(devInstance);
    qcfg->it_credit     = 0;
    qcfg->max_credit    = MAX_IP_PACKET_SIZE * 2;
    qcfg->congst_thrsh  = MAX_IP_PACKET_SIZE * 300;
    qcfg->congst_thrsh_packets = 300;

    // Cluster
    l2sw_netdev_qos_cluster_db[devInstance].global_credit      = 0;
    l2sw_netdev_qos_cluster_db[devInstance].max_global_credit  = MAX_IP_PACKET_SIZE * 2;
    l2sw_netdev_qos_cluster_db[devInstance].egr_congst_thrsh1  = MAX_IP_PACKET_SIZE * 2;
    l2sw_netdev_qos_cluster_db[devInstance].egr_congst_thrsh2  = MAX_IP_PACKET_SIZE * 28;
    l2sw_netdev_qos_cluster_db[devInstance].egr_congst_thrsh3  = MAX_IP_PACKET_SIZE * 32;
    l2sw_netdev_qos_cluster_db[devInstance].egr_congst_thrsh4  = MAX_IP_PACKET_SIZE * 48;
    l2sw_netdev_qos_cluster_db[devInstance].egr_congst_thrsh_packets  = 48;

    rc = ti_ppm_qos_cluster_disable(PAL_CPPI41_SR_L2SW_QOS_CLUSTER_BASE + devInstance);
    rc = ti_ppm_qos_cluster_setup  (PAL_CPPI41_SR_L2SW_QOS_CLUSTER_BASE + devInstance, &l2sw_netdev_qos_cluster_db[devInstance]);
    rc = ti_ppm_qos_cluster_enable (PAL_CPPI41_SR_L2SW_QOS_CLUSTER_BASE + devInstance);

    dev->vpid_block.qos_cluster[0]  = &l2sw_netdev_qos_cluster_db[devInstance];
    dev->vpid_block.qos_clusters_count = 1;

    return rc;
}


/**************************************************************************/
/*! \fn         l2sw_netdev_pp_shutdown_qos
 **************************************************************************
 *
 *  \brief      Packet Processor QoS shutdown
 *
 *  \param[in]  Network Device
 *  \return     OK or Error
 **************************************************************************/
static int l2sw_netdev_pp_shutdown_qos(struct net_device *dev)
{
    int             rc;

    rc = ti_ppm_qos_cluster_disable(PAL_CPPI41_SR_L2SW_QOS_CLUSTER_BASE + dev->devInstance);

    dev->vpid_block.qos_clusters_count = 0;

    return rc;
}

/**************************************************************************/
/*! \fn         l2sw_netdev_pp_prepare_qos
 **************************************************************************
 *
 *  \brief      Packet Processor QoS preperation
 *
 *  \param[in]  Network Device
 *  \return     None
 **************************************************************************/
static void l2sw_netdev_pp_prepare_qos(struct net_device *dev)
{
    dev->qos_setup_hook                = l2sw_netdev_pp_setup_qos;
    dev->qos_shutdown_hook             = l2sw_netdev_pp_shutdown_qos;
    dev->qos_select_hook               = l2sw_netdev_pp_select_qos;
}

#endif // CONFIG_TI_PACKET_PROCESSOR


/**************************************************************************/
/*! \fn         l2sw_netdev_init_acc_chan
 **************************************************************************
 *
 *  \brief      Interrupt Accumulator Channels INIT routine
 *
 *  \param[in]  PAL Handle
 *  \param[in]  Accumulator channel number
 *  \param[in]  Queue for the accumulator to monitor
 *  \param[out] Accumulator channel handle
 *  \return     OK or error
 **************************************************************************/
static int l2sw_netdev_init_acc_chan(PAL_Handle palHnd, int chan_num, Cppi4Queue queue, PAL_Cppi4AccChHnd* acc_ch_hnd)
{
    Cppi4AccumulatorCfg cfg;
    unsigned int accListSize;

    *acc_ch_hnd = NULL;


    cfg.accChanNum             = chan_num;
    cfg.list.maxPageEntry      = PAL_CPPI41_L2SW_ACC_MAX_PAGE_ENTRIES;      /* This is entries per page (and we have 2 pages) */
    cfg.list.listEntrySize     = PAL_CPPI41_ACC_ENTRY_TYPE_D;               /* Only interested in register 'D' which has the desc pointer */
    cfg.list.listCountMode     = PAL_CPPI41_L2SW_ACC_LIST_NULL_TERM;        /* Zero indicates null terminated list. */
    cfg.list.pacingMode        = PAL_CPPI41_L2SW_ACC_PACE_MODE_LASTINTR;    /* Wait for time since last interrupt */
    cfg.pacingTickCnt          = PAL_CPPI41_L2SW_ACC_PACE_TICK_CNT;         /* Wait for 1000uS == 1ms (40*25usec timer) */
    cfg.list.maxPageCnt        = PAL_CPPI41_L2SW_ACC_MAX_PAGE_COUNT;        /* Use two pages */
    cfg.list.stallAvoidance    = 1;                                         /* Use the stall avoidance feature */
    cfg.queue                  = queue;
    cfg.mode                   = 0;

    accListSize = (cfg.list.maxPageEntry * (cfg.list.listEntrySize + 1)) * cfg.list.maxPageCnt * sizeof(Uint32);
    if(!(cfg.list.listBase = kzalloc(accListSize, GFP_KERNEL)))
    {
        ERR_PRINTK("Unable to allocate list page of size %d\n", accListSize);
        return -1;
    }

    PAL_CPPI4_CACHE_WRITEBACK((unsigned long)cfg.list.listBase, accListSize);

    cfg.list.listBase = (Ptr)PAL_CPPI4_VIRT_2_PHYS((Ptr)cfg.list.listBase);

    if(!(*acc_ch_hnd = PAL_cppi4AccChOpen(palHnd, &cfg)))
    {
        ERR_PRINTK("Unable to open accumulator channel #%d\n", chan_num);
        kfree(cfg.list.listBase);
        return -1;
    }

    return 0;
}

/**************************************************************************/
/*! \fn         l2sw_netdev_tx_start_xmit
 **************************************************************************
 *
 *  \brief      Transmit Function
 *
 *  \param[in]  SK buff
 *  \param[in]  Net Device
 *  \return     OK or error
 **************************************************************************/
static int l2sw_netdev_tx_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
    l2sw_netdev_private_t* priv = netdev_priv(dev);
    Cppi4HostDescLinux*    hostDesc;
    unsigned int len;
    unsigned int prio = PAL_CPPI4x_PRTY_LOW; // TBD: how to decide on priority

    /* get a free Tx descriptor */
    if(!(hostDesc = (Cppi4HostDescLinux *)PAL_cppi4QueuePop(hostToPpFDHostQHnd[prio]) ))
    {
        /* This should not occur in this driver (because of what is done later in this function) */
        priv->stats.tx_dropped++;
        dev_kfree_skb_any(skb);
        return NETDEV_TX_OK;
    }

    hostDesc = PAL_CPPI4_PHYS_2_VIRT(hostDesc);

    /* Put TX Complete as the return Queue */
    hostDesc->hw.pktInfo &= ~(PAL_CPPI4_HOSTDESC_PKT_RETQMGR_MASK | PAL_CPPI4_HOSTDESC_PKT_RETQNUM_MASK);
    hostDesc->hw.pktInfo |= (PAL_CPPI41_QUEUE_MGR_PARTITION_SR    << PAL_CPPI4_HOSTDESC_PKT_RETQMGR_SHIFT)
                         |  (PAL_CPPI41_SR_HOST_TX_COMPLETE_Q_NUM(prio) << PAL_CPPI4_HOSTDESC_PKT_RETQNUM_SHIFT);

    len = l2sw_netdev_tx_link_skb_to_desc(dev, hostDesc, skb);

    dev->trans_start = jiffies;

#if 0
    printk("TX %s packet=%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X.%02X\n",
            dev->name,
            skb->data[ 0],skb->data[ 1],skb->data[ 2],skb->data[ 3],skb->data[ 4],skb->data[ 5],skb->data[ 6],skb->data[ 7],skb->data[ 8],skb->data[ 9],
            skb->data[10],skb->data[11],skb->data[12],skb->data[13],skb->data[14],skb->data[15],skb->data[16],skb->data[17],skb->data[18],skb->data[19],
            skb->data[20],skb->data[21],skb->data[22],skb->data[23],skb->data[24],skb->data[25],skb->data[26],skb->data[27],skb->data[28],skb->data[29],
            skb->data[30],skb->data[31],skb->data[32],skb->data[33],skb->data[34],skb->data[35],skb->data[36],skb->data[37],skb->data[38],skb->data[39]
          );
#endif

    /* Push to PP(QPDSP) - then after handle PTID it will be sent to the PrxPDSP Tx queues */
    PAL_cppi4QueuePush(hostToPpInfraInputQHnd[prio], (Uint32 *)PAL_CPPI4_VIRT_2_PHYS(hostDesc), PAL_CPPI4_DESCSIZE_2_QMGRSIZE(PAL_CPPI41_SR_HOST_TO_PP_FD_HOST_DESC_SIZE), len);

    priv->stats.tx_packets++;
    priv->stats.tx_bytes += hostDesc->skb->len;

    return NETDEV_TX_OK;
}


/**************************************************************************/
/*! \fn         l2sw_netdev_tx_link_skb_to_desc
 **************************************************************************
 *
 *  \brief      Links an skb to a Tx Descriptor
 *
 *  \param[in]  Net Device
 *  \param[in]  Descriptor
 *  \param[in]  SK buff
 *  \return     length of final packet
 **************************************************************************/
static unsigned int l2sw_netdev_tx_link_skb_to_desc(struct net_device* dev, Cppi4HostDescLinux* hostDesc, struct sk_buff *skb)
{
    unsigned int len;
    unsigned int queueNum;

#ifdef CONFIG_TI_PACKET_PROCESSOR_EXT_SWITCH
    if ((L2SW_NETDEV_INSTANCE_DATA0_e == dev->devInstance) &&
        !((skb->data[12] == 0x88) && (skb->data[13] == 0x70))) /* Do NOT TAG MoCA Management packets */
    {
        u16 vlan_tci = ( VLAN_PRIO_MASK & (((u16)(skb->ti_meta_info)) << VLAN_PRIO_SHIFT) ) | 0x01 ;
        skb = __vlan_put_tag(skb,vlan_tci);
    }
#endif

    /* ======================================================================== */
    /*  Any accesses to the skb MUST appear below this line (!)                 */
    /*  This is beacuase the skb might be chamged by the above function (VLAN)  */
    /* ======================================================================== */
    len = ((skb->len < ETH_ZLEN) ? ETH_ZLEN : skb->len);

    hostDesc->hw.descInfo = (PAL_CPPI4_HOSTDESC_DESC_TYPE_HOST << PAL_CPPI4_HOSTDESC_DESC_TYPE_SHIFT)
                          | (1 << PAL_CPPI4_HOSTDESC_PROT_WORD_CNT_SHIFT)
                          | (len + 4);
    hostDesc->hw.bufPtr = PAL_CPPI4_VIRT_2_PHYS((unsigned int)skb->data);
    hostDesc->hw.buffLen = len + 4; // This is for CRC placeholder for UDMA
    hostDesc->skb = skb;
    hostDesc->psi[0] = skb->ti_meta_info;
    hostDesc->psi[1] = 0;

    if (0 != skb->dev->vpid_block.qos_clusters_count)
    {
        queueNum = PAL_CPPI41_SR_L2SW_QPDSP_QOS_Q_NUM(dev->devInstance, PAL_CPPI41_SR_L2SW_QPDSP_QOS_Q_PRIORITY_LOW); // TBD: how to decide on priority
    }
    else
    {
        queueNum = PAL_CPPI41_SR_L2SW_PrxPDSP_Q_NUM(dev->devInstance); // No QoS - send staright to PrxPDSP
    }

    /* Set SYNC Q PTID info in egress descriptor */
    if(skb->pp_packet_info.ti_pp_flags == TI_PPM_SESSION_INGRESS_RECORDED)
    {
        memcpy(&(hostDesc->hw.netInfoWord0), skb->pp_packet_info.ti_epi_header, 8);
        hostDesc->hw.netInfoWord1 &= ~(0xFFFF);
        hostDesc->hw.netInfoWord1 |= queueNum; /* after QPDSP, push to QoS device Queue */
    }
    else
    {
        hostDesc->hw.netInfoWord1 = queueNum;
    }

    PAL_CPPI4_CACHE_WRITEBACK((unsigned long)skb->data, skb->len);
    PAL_CPPI4_CACHE_WRITEBACK((unsigned long)hostDesc, PAL_CPPI41_SR_HOST_TO_PP_FD_HOST_DESC_SIZE);

    return len;
}

/**************************************************************************/
/*! \fn         l2sw_netdev_rx_open
 **************************************************************************
 *
 *  \brief      Open Rx routine
 *
 *  \param[in]  Net Device
 *  \return     Ok or error
 **************************************************************************/
static int l2sw_netdev_rx_open(struct net_device* dev)
{
    l2sw_netdev_private_t* priv = netdev_priv(dev);
    Cppi4Queue queue;


    /****************************************************************/
    /* Prepare Accumulator channel                                  */
    /****************************************************************/
    queue.qMgr = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
    queue.qNum = PAL_CPPI41_SR_L2SW_HOST_RX_Q_NUM(dev->devInstance);
    priv->rxCompleteAccChHnd = NULL;
    priv->rxCompleteAccCh = PAL_CPPI41_L2SW_ACC_CH_NUM(dev->devInstance);
    if(l2sw_netdev_init_acc_chan(priv->palHnd, priv->rxCompleteAccCh, queue, &priv->rxCompleteAccChHnd))
    {
        ERR_PRINTK("Unable to open accumulator channel #%d for device named %s\n", priv->rxCompleteAccCh, dev->name);
        return -ENOMEM;
    }

    priv->rxCompleteAccChListBase = priv->rxCompleteAccChList = PAL_cppi4AccChGetNextList(priv->rxCompleteAccChHnd);

#if defined(L2SW_NETDEV_USE_NAPI)
    napi_enable(&priv->napi);
#elif defined(L2SW_NETDEV_USE_TASKLET)
    tasklet_init(&priv->rx_tasklet, l2sw_netdev_rx_complete, (unsigned long)dev);
#elif defined(L2SW_NETDEV_USE_WORKQ)
    /* Create WQ */
    priv->l2sw_rx_wq = create_workqueue("l2sw_rx_wq");
    if (priv->l2sw_rx_wq == NULL)
    {
        ERR_PRINTK("Failed to create l2sw_rx_wq\n");
        return -ENOMEM;
    }
    /* Init the work */
    INIT_WORK(&(priv->l2sw_rx_work.work), l2sw_netdev_rx_complete);
#endif

    /* request the Rx IRQs */
    if(request_irq(MAP_INTD_TO_INTC(PAL_CPPI41_L2SW_ACC_INTV_NUM(dev->devInstance)), l2sw_netdev_rx_interrupt, IRQF_DISABLED, dev->name, dev))
    {
        ERR_PRINTK("unable to get IRQ #%d for device named %s\n", MAP_INTD_TO_INTC(PAL_CPPI41_L2SW_ACC_INTV_NUM(dev->devInstance)), dev->name);
        return -ENOMEM;
    }

    return 0;
}

/**************************************************************************/
/*! \fn         l2sw_netdev_rx_interrupt
 **************************************************************************
 *
 *  \brief      Receive ISR
 *
 *  \param[in]  IRQ
 *  \param[in]  Device
 *  \return     OK or error
 **************************************************************************/
static irqreturn_t l2sw_netdev_rx_interrupt(int irq, void *dev)
{
    l2sw_netdev_private_t *priv = netdev_priv((struct net_device *)dev);

#if defined(L2SW_NETDEV_USE_NAPI)
    /* if poll routine is not running, start it now. */
    if (likely(napi_schedule_prep(&priv->napi)))
    {
        /* Since the INTD interrupts are level, need to disable the IRQ in order to run the tasklet */
        disable_irq_nosync(MAP_INTD_TO_INTC(PAL_CPPI41_L2SW_ACC_INTV_NUM(((struct net_device *)dev)->devInstance)));
        __napi_schedule(&priv->napi);
    }
#elif defined(L2SW_NETDEV_USE_TASKLET)
    /* Since the INTD interrupts are level, need to mask the IRQ in order for the tasklet to run */
    disable_irq_nosync(MAP_INTD_TO_INTC(PAL_CPPI41_L2SW_ACC_INTV_NUM(((struct net_device *)dev)->devInstance)));
    tasklet_schedule(&priv->rx_tasklet);
#elif defined(L2SW_NETDEV_USE_WORKQ)
    /* Since the INTD interrupts are level, need to disable the IRQ in order to run the tasklet */
    disable_irq_nosync(MAP_INTD_TO_INTC(PAL_CPPI41_L2SW_ACC_INTV_NUM(((struct net_device *)dev)->devInstance)));

    /* Put dev in work */
    priv->l2sw_rx_work.dev = dev;
    /* Que work */
    queue_work(priv->l2sw_rx_wq, &(priv->l2sw_rx_work.work));
#endif

    return IRQ_RETVAL(1);
}

/**************************************************************************/
/*! \fn         l2sw_netdev_rx_complete
 **************************************************************************
 *
 *  \brief      Rx Complete handler
 *
 *  \param[in]  Net Device
 *  \param[in]  Processed packets budget
 *  \return     Number of processed packets
 **************************************************************************/
#if defined(L2SW_NETDEV_USE_NAPI)
static int l2sw_netdev_rx_complete(struct net_device* dev, int budget)
#elif defined(L2SW_NETDEV_USE_TASKLET)
static void l2sw_netdev_rx_complete(unsigned long data)
#elif defined(L2SW_NETDEV_USE_WORKQ)
static void l2sw_netdev_rx_complete(struct work_struct *work)
#endif
{
#if defined(L2SW_NETDEV_USE_TASKLET)
    struct net_device*      dev = (struct net_device*) data;
#elif defined(L2SW_NETDEV_USE_WORKQ)
    l2sw_rx_work_t *curr_work = (l2sw_rx_work_t *)work;
    struct net_device* dev = curr_work->dev;
#endif
    l2sw_netdev_private_t* priv = netdev_priv(dev);
    Cppi4HostDescLinux* hostDesc;
    int packets_processed = 0;

#if defined(L2SW_NETDEV_USE_WORKQ)
    static int first = 1;
    struct task_struct *ctask = current;

    if (first)
    {
        /* Set priority */
        set_user_nice(current, 1);
        first = 0;

        printk("**** ---- >>>> l2sw_netdev_rx_complete - name \"%s\", pid %d\n", ctask->comm, ctask->pid);
    }
#endif

    while(avalanche_intd_get_interrupt_count(0, priv->rxCompleteAccCh) && (packets_processed < L2SW_NETDEV_RX_SERVICE_MAX))
    {
        while((hostDesc = (Cppi4HostDescLinux*)((unsigned long)*priv->rxCompleteAccChList & QMGR_QUEUE_N_REG_D_DESC_ADDR_MASK)))
        {
            struct sk_buff *newskb;

            hostDesc = PAL_CPPI4_PHYS_2_VIRT(hostDesc);

            /* if not cleaning up .. */
            if(!test_bit(0, &priv->state))
            {
                /* get a new skb for this hostDesc */
                if((newskb = dev_alloc_skb(PAL_CPPI41_SR_L2SW_INFRA_FD_HOST_BUFFER_SIZE)))
                {
                    struct sk_buff* rxskb;

                    PAL_CPPI4_CACHE_INVALIDATE(hostDesc, PAL_CPPI41_SR_L2SW_INFRA_FD_HOST_DESC_SIZE);
                    rxskb = hostDesc->skb;

                    PAL_CPPI4_CACHE_INVALIDATE(rxskb->data, hostDesc->hw.buffLen - 4);
                    skb_put(rxskb, hostDesc->hw.buffLen - 4); /* remove CRC from length */
                    dev->last_rx = jiffies;
                    priv->stats.rx_packets++;
                    priv->stats.rx_bytes += hostDesc->hw.buffLen;

                    /* Keep SYNC Q PTID info in skb for egress */
                    if(hostDesc->hw.netInfoWord1)
                    {
                        memcpy(rxskb->pp_packet_info.ti_epi_header, &(hostDesc->hw.netInfoWord0), 8);
                        rxskb->pp_packet_info.ti_pp_flags = TI_PPM_SESSION_INGRESS_RECORDED;
                    }

                    /* ... then send the packet up */
                    rxskb->ti_meta_info = hostDesc->psi[0];
                    rxskb->ti_meta_info2 = hostDesc->psi[2];
                    rxskb->protocol = eth_type_trans(rxskb, dev);
                    netif_receive_skb(rxskb);
                    DPRINTK("packet received for network device %s\n", dev->name);

                    /* Prepare to return to free queue */
                    l2sw_netdev_rx_link_skb_to_desc(dev, hostDesc, newskb);
                }
            }

            packets_processed++;
            priv->rxCompleteAccChList++;
            /* Return to free queue */
            PAL_cppi4QueuePush(l2swInfraFDHostQHnd, (Uint32 *)PAL_CPPI4_VIRT_2_PHYS(hostDesc), PAL_CPPI4_DESCSIZE_2_QMGRSIZE(PAL_CPPI41_SR_L2SW_INFRA_FD_HOST_DESC_SIZE), 0);

#if defined(L2SW_NETDEV_USE_NAPI)
            if(!test_bit(0, &priv->state))
            {
                /* thats it, we did enough. Jump out now! */
                if(packets_processed >= budget)
                {
                    return packets_processed;
                }
            }
#endif
        }

        /* Update the list entry for next time */
        priv->rxCompleteAccChList = PAL_cppi4AccChGetNextList(priv->rxCompleteAccChHnd);
        avalanche_intd_set_interrupt_count(0, priv->rxCompleteAccCh, 1);
    }

#if defined(L2SW_NETDEV_USE_TASKLET) || defined(L2SW_NETDEV_USE_WORKQ)
    /* First clear the IRQ in order not to get a false interrupt since INTD is level */
    ack_irq(MAP_INTD_TO_INTC(PAL_CPPI41_L2SW_ACC_INTV_NUM(dev->devInstance)));

    /* Send INTD EOI */
    avalanche_intd_write_eoi(PAL_CPPI41_L2SW_ACC_INTV_NUM(dev->devInstance));

    /* It could be that between INTD count decrement and EOI the accumulator will issue another interrupt.
       The logic of INTD is such that level will remain active high even after EOI is set, so INTC will
       lose the interrupt after ack_irq is done (it now expects INTD polarity change).
       Therefore we must check INTD count and if it is not 0 - reschedule the tasklet */
    if (avalanche_intd_get_interrupt_count(0, priv->rxCompleteAccCh))
    {
#if defined(L2SW_NETDEV_USE_TASKLET)
        tasklet_schedule(&priv->rx_tasklet);
#elif defined(L2SW_NETDEV_USE_WORKQ)
            /* Put dev in work */
            priv->l2sw_rx_work.dev = dev;
            /* Que work */
            queue_work(priv->l2sw_rx_wq, &(priv->l2sw_rx_work.work));
#endif
        return;
    }

    /* Now enable the IRQ */
    enable_irq(MAP_INTD_TO_INTC(PAL_CPPI41_L2SW_ACC_INTV_NUM(dev->devInstance)));

#elif defined(L2SW_NETDEV_USE_NAPI)
    return packets_processed;
#endif
}

/**************************************************************************/
/*! \fn         l2sw_netdev_rx_link_skb_to_desc
 **************************************************************************
 *
 *  \brief      Links an skb to an Rx Descriptor
 *
 *  \param[in]  Net Device
 *  \param[in]  Descriptor
 *  \param[in]  SK buff
 *  \return     none
 **************************************************************************/
static void l2sw_netdev_rx_link_skb_to_desc(struct net_device* dev, Cppi4HostDescLinux* hostDesc, struct sk_buff *skb)
{
    skb_reserve (skb, NET_IP_ALIGN);    /* 16 byte align the IP fields. */
    hostDesc->hw.orgBuffLen  = PAL_CPPI41_SR_L2SW_INFRA_FD_HOST_BUFFER_SIZE - NET_IP_ALIGN;
    hostDesc->hw.orgBufPtr   = PAL_CPPI4_VIRT_2_PHYS(skb->data);
    hostDesc->skb            = skb;
    hostDesc->psi[0] = 0;
    hostDesc->psi[1] = 0;
    hostDesc->psi[2] = 0;

    /* Write the hostDesc to the RAM */
    PAL_CPPI4_CACHE_WRITEBACK(hostDesc, PAL_CPPI41_SR_L2SW_INFRA_FD_HOST_DESC_SIZE);
}

module_param(inpmac, charp, 0);
module_init(l2sw_driver_init);
module_exit(l2sw_driver_exit);
MODULE_LICENSE ("GPL");
