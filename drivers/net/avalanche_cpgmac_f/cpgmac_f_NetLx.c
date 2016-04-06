/*
 *
 * cpgmac_f_NetLx.c
 * Description:
 * see below
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


/** \file   cpgmac_f_NetLx.c
    \brief  CPMAC Linux DDA Source file

    This file contains the device driver adaptation for Linux for CPMAC/CPGMAC
    device based upon PSP Framework architecture.

    Acknowledgements: This DDA implementation for CP(G)MAC device is based upon
    Linux device driver for CP(G)MAC written using HAL 2.0.

    Notes:
    DDA/DDC Common features:

    The following flags should be defined by the make file for support of the features:

    (1) CPMAC_CACHE_WRITEBACK_MODE to support write back cache mode.

    (2) NOTE: THIS DRIVER DOES NOT SUPPORT MULTIFRAGMENTS AS OF NOW
        For future support - define CPMAC_MULTIFRAGMENT to support multifragments.


    DDA specific features:

    (3) CPMAC_USE_CONFIG_SERVICE - to use configuration service to obtain configuration
        information for the instance. The configuration information is string based and
        the parsing functions from the config service are used. The config string is as follows:

        "instId=,BaseAddr=,TxIntrLine=,RxIntrLine=,BusFreq=,speed=,duplex=,promEn=,BroadEn=,MultiEn=,maxRxPktLen=,
        txNumBD=,txServiceMax=,rxNumBD=,rxServiceMax=,rxBufExtra=,
        mdioBaseAddr=,mdioResetLine=,mdioBusFreq=,mdioClkFreq=,mdioPhyMask=,mdioTickMsec=,
        MaxRxFrags=,CPPI4TxChNum=,CPPI4RxChNum=,CPPI4CqIndex=,CPPI4FbqIndex[0]=,
        CPPI4FbqIndex[1]=,CPPI4FbqIndex[2]=,CPPI4FbqIndex[3]=,CPPI4RqIndex=,NWSS_QMGR_BASE=,
        NWSS_DMA_BASE=,NWSS_RESET_BIT="

        Example: "instId=0,BaseAddr=a304e000,TxIntrLine=15,RxIntrLine=32,
            BusFreq=12000000,speed=0,duplex=0,promEn=0,BroadEn=1,MultiEn=1,
            maxRxPktLen=1522,txNumBD=64,txServiceMax=5,rxNumBD=64,rxServiceMax=5,
            rxBufExtra=0,mdioBaseAddr=a8611e00,mdioResetLine=22,mdioBusFreq=12000000,
            mdioClkFreq=0,mdioPhyMask=2,mdioTickMsec=10,MaxRxFrags=1,
            CPPI4TxChNum=16,CPPI4RxChNum=16,CPPI4CqIndex=0,CPPI4FbqIndex[0]=0,
          CPPI4FbqIndex[1]=0,CPPI4FbqIndex[2]=0,CPPI4FbqIndex[3]=0,
          CPPI4RqIndex=0,NWSS_QMGR_BASE=a3068000,
          NWSS_DMA_BASE=a300a000,NWSS_RESET_BIT=9"

        Note: If speed = 9999 NO PHY mode selected

    (4) CPMAC_USE_ENV - to use ENV variables to get configuration information.
        When config service is not being used, either hardcoded (static) values can be used for
        unit testing or ENV variables can be used. The configuration information should be in
        the following string format (without the string names):

        "instId=,BaseAddr=,TxIntrLine=,RxIntrLine=,BusFreq=,speed=,duplex=,promEn=,BroadEn=,MultiEn=,maxRxPktLen=,
        txNumBD=,txServiceMax=,rxNumBD=,rxServiceMax=,rxBufExtra=,
        mdioBaseAddr=,mdioResetLine=,mdioBusFreq=,mdioClkFreq=,mdioPhyMask=,mdioTickMsec=,
        MaxRxFrags=,CPPI4TxChNum=,CPPI4RxChNum,CPPI4CqIndex,CPPI4FbqIndex[0],
        CPPI4FbqIndex[1],CPPI4FbqIndex[2],CPPI4FbqIndex[3],CPPI4RqIndex,NWSS_QMGR_BASE,
        NWSS_DMA_BASE,NWSS_RESET_BIT"

        Example: "0:a304e000:15:32:12000000:0:0:0:1:1:1522:64:5:64:5:0:a8611e00:22:12000000:0:2:10:1:16:16:0:0:0:0:0:0:a3068000:a300a000:9"
                 "1:a304e800:16:33:12000000:0:0:0:1:1:1522:64:5:64:5:0:a8611e00:22:12000000:0:4:10:1:17:17:1:1:1:1:1:1:a3068000:a300a000:9"

        Note: If speed = 9999 NO PHY mode selected

 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/errno.h>
#include <linux/in.h>
#include <linux/ioport.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/highmem.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <linux/spinlock.h>
#include <asm/irq.h>            /* For NR_IRQS only. */
#include <asm/bitops.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/semaphore.h>
#include <linux/ethtool.h>

#include <linux/in.h>
#include <linux/skbuff.h>

#include <linux/ip.h>
#include <linux/tcp.h>
#include <net/checksum.h>
#include <linux/inet_lro.h>

#ifdef CONFIG_ARM_AVALANCHE_PPD
#include <asm-arm/arch-avalanche/generic/ti_ppd.h>
#include <linux/ti_ppm.h>
#endif

#include <asm-arm/arch-avalanche/generic/ti_linux_porting.h>

#include <asm-arm/arch-avalanche/generic/pal.h>


#if defined (CONFIG_ARM_AVALANCHE_COLORED_LED)
#include <asm-arm/arch-avalanche/generic/led_manager.h>
#endif

#ifdef CONFIG_PSP_TRACE
#include <linux/psp_trace.h>
#endif

#ifdef  CONFIG_ARM_EXTERNAL_SWITCH
#include "dda_switch.h"
#include "ddc_switch.h"
#endif

#include "cpgmac_f_NetLx.h"     /* This will include required DDC headers */
#include "mib_ioctl.h"

#include <asm-arm/arch-avalanche/generic/pal_cppi41.h>


#define CONFIG_CPMAC_NOPHY                                    9999

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Maintainer: Suraj S Iyer <ssiyer@ti.com>");
MODULE_DESCRIPTION("Driver for TI CPMAC/CPGMAC");

static int cfg_link_speed = 0;
//module_param(cfg_link_speed, "i");
module_param(cfg_link_speed, int, 0);
MODULE_PARM_DESC(cfg_link_speed, "Fixed speed of the Link: <100/10>");

static char *cfg_link_mode = NULL;
//module_param(cfg_link_mode, "1-3s");
module_param(cfg_link_mode, charp, 0);
MODULE_PARM_DESC(cfg_link_mode, "Fixed mode of the Link: <fd/hd>");

int debug_mode = 0;
//MODULE_PARM(debug_mode, "i");
module_param(debug_mode, int, 1);
MODULE_PARM_DESC(debug_mode,
                 "Turn on the debug info: <0/1>. Default is 0 (off)");

/* Version Info */
#define CPMAC_DDA_MAJOR_VERSION         0
#define CPMAC_DDA_MINOR_VERSION         1


#define CPMAC_MODULE_VERSION "0.1"
MODULE_VERSION(CPMAC_MODULE_VERSION);


const char cpmac_DDA_version_string[] = "CPGMAC_F Linux DDA version 0.1";

/* Debug tracing */
int cpmac_link_status = 0;
int cpmac_debug_mode = 0;

/* Global variables required during initialization */
static int g_cfg_start_link_params = CFG_START_LINK_SPEED;
static int g_init_enable_flag = 0;

#ifdef CONFIG_ARM_AVALANCHE_PPD
/* Delay for SR */
#define DDA_CPMAC_SR_DELAY 200
#include <asm-arm/arch-avalanche/puma5/puma5_pp.h> /* For PID base config */
#ifndef CONFIG_TI_PACKET_PROCESSOR
static TI_PP_PID        pid_eth[PP_ETH_PID_COUNT];
#endif
#endif /* CONFIG_ARM_AVALANCHE_PPD */

/* Global device array */
static struct net_device *cpmac_net_dev[CPMAC_MAX_INSTANCES] =
    { NULL, NULL };
#ifndef MODULE
static char* cpmac_mac_string[CPMAC_MAX_INSTANCES];
static char* eth_mdio_num;
#endif
/* Last cpmac device accessed */
static struct net_device *last_cpmac_device = NULL;
/* Number of CPMAC instances */
static int cpmac_devices_installed = 0;

/* forward declarations */
//extern int cpmac_poll(struct net_device *p_dev, int *budget);
extern int cpmac_poll(struct napi_struct *napi, int budget);


/* Net device related private function prototypes */
static int cpmac_dev_init(struct net_device *p_dev);
extern int cpmac_dev_open(struct net_device *dev);
static int cpmac_ioctl(struct net_device *p_dev, struct ifreq *rq,
                       int cmd);
static int cpmac_dev_close(struct net_device *p_dev);
static void cpmac_dev_mcast_set(struct net_device *p_dev);
static void cpmac_tx_timeout(struct net_device *p_dev);
static struct net_device_stats *cpmac_dev_get_net_stats(struct net_device
                                                        *dev);
static void cpmac_set_ethtool_ops(struct net_device *dev);

/* Internal Function prototypes */
static int __init cpmac_p_detect_manual_cfg(int, char *, int);
static int cpmac_p_read_stats(char *buf, char **start, off_t offset, int count, int *eof, void *data);
static int cpmac_p_write_stats(struct file *fp, const char *buf, unsigned long count, void *data);
static int cpmac_p_read_link(char *buf, char **start, off_t offset, int count, int *eof, void *data);
static int cpmac_dump_config(char *buf, char **start, off_t offset, int count, int *eof, void *data);
static int cpmac_p_get_version(char *buf, char **start, off_t offset, int count, int *eof, void *data);
static int cpmac_p_update_statistics(struct net_device *p_dev, char *buf, int limit, int *p_len);
static int cpmac_p_reset_statistics(struct net_device *p_dev);
static int cpmac_p_read_rfc2665_stats(char *buf, char **start, off_t offset, int count, int *eof, void *data);
static int cpmac_p_dev_enable(CpmacNetDevice * hDDA);
static int cpmac_p_dev_disable(CpmacNetDevice * hDDA);
static void cpmac_p_tick_timer_expiry(CpmacNetDevice * hDDA);
static int cpmac_dev_set_mac_addr(struct net_device *p_dev, void *addr);
static int cpgmac_shutdown_qos (struct net_device *dev);
#ifdef CONFIG_ARM_AVALANCHE_PPD
static int cpgmac_add_pid (int indx);
static int cpgmac_setup_qos(struct net_device *dev);
static int cpgmac_select_qos(struct sk_buff *skb);
#endif
#ifdef CPMAC_POLL_MODE
static void cpmac_p_txpoll (unsigned long data);
static void cpmac_p_rxpoll (unsigned long data);
#endif

/* Proc entries */
static struct proc_dir_entry *gp_stats_file = NULL;
static struct proc_dir_entry *cpmacctl      = NULL;

/* The real device and driver matching will be done by the
 * match routine of the platform bus. It is necessary
 * for the probe function to be non null though.
 * We have a function that just returns zero. "All matched."
 */

static int __devinit cpmac_probe(struct device *dev)
{
    return 0;
}

static int eth_rst_gpio;

static int cpmac_suspend (struct device * dev, pm_message_t state)
{
    /* Put the External phy in reset
     */
    /* Set the pin as GPIO pin */
    PAL_sysGpioCtrl(eth_rst_gpio, GPIO_PIN, GPIO_OUTPUT_PIN);
    /* Assert the reset pin by bringing it low */
    PAL_sysGpioOutBit(eth_rst_gpio, 0);
    /* According to datasheet minimum delay required is 20us put 1000us to be on safer side */
    udelay(1000);

    PAL_sysPowerCtrl((INT32)PSC_SR_CLK6, PSC_SW_RST_DISABLE);
    return 0;
}

static int cpmac_resume(struct device * dev)
{

    PAL_sysPowerCtrl((INT32)PSC_SR_CLK6, PSC_ENABLE);

    PAL_sysGpioCtrl(eth_rst_gpio, GPIO_PIN, GPIO_OUTPUT_PIN);
    /* Assert the reset pin by bringing it low */
    PAL_sysGpioOutBit(eth_rst_gpio, 0);
    /* According to datasheet minimum delay required is 20us put 1000us to be on safer side */
    udelay(1000);
    /* De-assert the reset */
    PAL_sysGpioOutBit(eth_rst_gpio, 1);

    return 0;
}

/* structure describing the CPMAC driver */
static struct device_driver cpmac_driver = {
    .name = "cpmac",
    .bus = &platform_bus_type,
    .probe = cpmac_probe,
    .remove = NULL,
    .suspend = cpmac_suspend,
    .resume = cpmac_resume,
};


static char cpmac_cfg[CPMAC_MAX_INSTANCES][400];


#define LOW_CPMAC   0x00001
#define HIGH_CPMAC  0x00002
#define EXT_SWITCH  0x10000

int eth_mdio_phy_addr = -1;

static int __init set_mdio_phy_addr(char *str)
{
    if (!str)
    {
        return 0;
    }

    printk(" CPGMAC PHY MDIO ADDRESS [%s]\n", str);

    if (*str != '\0')
    {
        eth_mdio_phy_addr = simple_strtoul(str,NULL,0);
    }
    return 1;
}

__setup("eth0_mdio_phy_addr=", set_mdio_phy_addr);



static int cpmac_cfg_build(int connect, int external_switch)
{
    unsigned int BaseAddr = 0;
    unsigned int BusFreq = 0;
    unsigned int speed = 0;
    unsigned int mdioPhyMask = 0;
    unsigned int TxNumBD = 0;
    unsigned int TxServiceMax = 0;
    unsigned int RxNumBD = 0;
    unsigned int RxServiceMax = 0;
    unsigned int TxIntrLine = 0;
    unsigned int RxIntrLine = 0;
    static int cfg_instance = 0;
    int duplex = 0;
    int mdioClockFreq = 0;
    unsigned int extra = 0;
    unsigned int mdiotick = 0;


    /* CPPI4 Configuration */
    unsigned int CPPI4TxChNum = 0;
    unsigned int CPPI4RxChNum = 0;
    unsigned int CPPI4DefRxDesc;
    unsigned int CPPI4DefTxDesc;
    unsigned int CPPI4NumTxQs;
    unsigned int CPPI4TxQMgr0;
    unsigned int CPPI4TxQNum0;
    unsigned int CPPI4TxQMgr1;
    unsigned int CPPI4TxQNum1;
    unsigned int CPPI4TxCmplQMgr;
    unsigned int CPPI4TxCmplQNum;
    unsigned int CPPI4CqIndex = 0;
    unsigned int CPPI4FbqmgrIndex[4] = { 0, 0, 0, 0 };
    unsigned int CPPI4FbqIndex[4] = { 0, 0, 0, 0 };
    unsigned int CPPI4RqIndex = 0;
    unsigned int CPPI4RqmgrIndex;
    unsigned int accTxChNum;
    unsigned int accRxChNum;
    unsigned int accTxVecNum;
    unsigned int accRxVecNum;

    if (-1 != eth_mdio_phy_addr)
    {
        mdioPhyMask = 1 << eth_mdio_phy_addr;
    }
    else
    {
        switch (connect)
        {
            case LOW_CPMAC:     mdioPhyMask = AVALANCHE_LOW_CPMAC_PHY_MASK;     break;
#if defined (CONFIG_AVALANCHE_HIGH_CPMAC)
            case HIGH_CPMAC:    mdioPhyMask = AVALANCHE_HIGH_CPMAC_PHY_MASK;    break;
#endif
            default:    return (-1);
        }
    }

    printk ("   Rudek %08X \n", mdioPhyMask);

    switch (connect) {

    case LOW_CPMAC:

        BaseAddr = AVALANCHE_LOW_CPMAC_BASE;
        TxIntrLine = AVALANCHE_NWSS_TX0;
        RxIntrLine = AVALANCHE_NWSS_RX0;
        BusFreq = PAL_sysClkcGetFreq(AVALANCHE_CPGMAC_REF_CLOCK);
        #if defined (CONFIG_MACH_PUMA5EVM)
        speed = external_switch ? CONFIG_CPMAC_NOPHY : 0;
        #else
        speed = external_switch ? CONFIG_CPMAC_NOPHY : 10;
       #endif
        /*speed = CONFIG_CPMAC_NOPHY;*/
        duplex = 0;
        TxNumBD = LOW_CPMAC_DDA_DEFAULT_TX_NUM_BD;
        TxServiceMax = LOW_CPMAC_DDA_DEFAULT_TX_MAX_SERVICE;
        RxNumBD = LOW_CPMAC_DDA_DEFAULT_RX_NUM_BD;
        RxServiceMax = LOW_CPMAC_DDA_DEFAULT_RX_MAX_SERVICE;
        CPPI4TxChNum = LOW_CPMAC_DEFAULT_CPPI4_TX_CH_NUM;
        CPPI4RxChNum = LOW_CPMAC_DEFAULT_CPPI4_RX_CH_NUM;
        CPPI4CqIndex = LOW_CPMAC_DEFAULT_CPPI4_CQ_INDEX;
        CPPI4FbqIndex[0] = LOW_CPMAC_DEFAULT_CPPI4_FBQ_INDEX0;
        CPPI4FbqIndex[1] = LOW_CPMAC_DEFAULT_CPPI4_FBQ_INDEX1;
        CPPI4FbqIndex[2] = LOW_CPMAC_DEFAULT_CPPI4_FBQ_INDEX2;
        CPPI4FbqIndex[3] = LOW_CPMAC_DEFAULT_CPPI4_FBQ_INDEX3;
        CPPI4RqIndex = LOW_CPMAC_DEFAULT_CPPI4_RQ_INDEX;
        mdioClockFreq = CPMAC_MDIO_CLOCK_FREQUENCY;
        extra = 0;
        mdiotick = 10;

        /* !@1 NEW configs ***/
        CPPI4DefRxDesc  = CPMAC_CPPI41_DEF_RXDESC;
        CPPI4DefTxDesc  = CPMAC_CPPI41_DEF_TXDESC;
        CPPI4NumTxQs    = CPMAC_CPPI41_NUM_TXQS;
#ifdef CONFIG_ARM_AVALANCHE_PPD
        CPPI4TxQMgr0    = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
        CPPI4TxQNum0    = PPFW_CPPI4x_TX_EGRESS_HOST_QNUM(0);
        CPPI4TxQMgr1    = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
        CPPI4TxQNum1    = PPFW_CPPI4x_TX_EGRESS_HOST_QNUM(1);
#else
        CPPI4TxQMgr0    = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
        CPPI4TxQNum0    = CPMAC_CPPI4x_TX_QNUM(0);
        CPPI4TxQMgr1    = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
        CPPI4TxQNum1    = CPMAC_CPPI4x_TX_QNUM(1);
#endif

        CPPI4TxCmplQMgr = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
        CPPI4TxCmplQNum = CPMAC_CPPI4x_TX_COMP_QNUM(0);
        CPPI4FbqmgrIndex[0] = CPMAC_CPPI4x_FBD_QMGR;
        CPPI4FbqmgrIndex[1] = CPMAC_CPPI4x_FBD_QMGR;
        CPPI4FbqmgrIndex[2] = CPMAC_CPPI4x_FBD_QMGR;
        CPPI4FbqmgrIndex[3] = CPMAC_CPPI4x_FBD_QMGR;
        CPPI4FbqIndex[0]    = CPMAC_CPPI4x_FBD_QNUM(0);
        CPPI4FbqIndex[1]    = CPMAC_CPPI4x_FBD_QNUM(0);
        CPPI4FbqIndex[2]    = CPMAC_CPPI4x_FBD_QNUM(0);
        CPPI4FbqIndex[3]    = CPMAC_CPPI4x_FBD_QNUM(0);
        CPPI4RqmgrIndex     = CPMAC_CPPI4x_RX_QMGR;
        CPPI4RqIndex        = CPMAC_CPPI4x_RX_QNUM(0);
        accTxChNum          = CPMAC_ACC_TXCMPL_CHNUM;
        accRxChNum          = CPMAC_ACC_RX_CHNUM;
        accTxVecNum         = CPMAC_ACC_TXCMPL_INTV;
        accRxVecNum         = CPMAC_ACC_RX_INTV;

        break;

#if defined (CONFIG_AVALANCHE_HIGH_CPMAC)

    case HIGH_CPMAC:

        BaseAddr = AVALANCHE_HIGH_CPMAC_BASE;
        TxIntrLine = AVALANCHE_NWSS_TX1;
        RxIntrLine = AVALANCHE_NWSS_RX1;
        BusFreq = PAL_sysClkcGetFreq(AVALANCHE_CPGMAC_REF_CLOCK);
        speed = external_switch ? CONFIG_CPMAC_NOPHY : 0;
        duplex = 0;
        TxNumBD = HIGH_CPMAC_DDA_DEFAULT_TX_NUM_BD;
        RxNumBD = HIGH_CPMAC_DDA_DEFAULT_RX_NUM_BD;
        TxServiceMax = HIGH_CPMAC_DDA_DEFAULT_TX_MAX_SERVICE;
        RxServiceMax = HIGH_CPMAC_DDA_DEFAULT_RX_MAX_SERVICE;
        CPPI4TxChNum = CPMAC_CPPI4x_TX_DMA_CHNUM(0);
        CPPI4RxChNum = CPMAC_CPPI4x_RX_DMA_CHNUM(0);
        CPPI4CqIndex = HIGH_CPMAC_DEFAULT_CPPI4_CQ_INDEX;
        CPPI4FbqIndex[0] = HIGH_CPMAC_DEFAULT_CPPI4_FBQ_INDEX0;
        CPPI4FbqIndex[1] = HIGH_CPMAC_DEFAULT_CPPI4_FBQ_INDEX1;
        CPPI4FbqIndex[2] = HIGH_CPMAC_DEFAULT_CPPI4_FBQ_INDEX2;
        CPPI4FbqIndex[3] = HIGH_CPMAC_DEFAULT_CPPI4_FBQ_INDEX3;
        CPPI4RqIndex = HIGH_CPMAC_DEFAULT_CPPI4_RQ_INDEX;
        mdioClockFreq = CPMAC_MDIO_CLOCK_FREQUENCY;
        extra = 0;
        mdiotick = 10;

        break;
#endif
    default:
        return (-1);
    }

    sprintf(cpmac_cfg[cfg_instance],
            "%d:%x:%d:%d:%u:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%x:%d:%u:%u:%x:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d",
            cfg_instance, BaseAddr, TxIntrLine, RxIntrLine, BusFreq, speed,
            duplex, CPMAC_DEFAULT_PROMISCOUS_ENABLE,
            CPMAC_DEFAULT_BROADCAST_ENABLE, CPMAC_DEFAULT_MULTICAST_ENABLE,
            CPMAC_DDA_DEFAULT_MAX_FRAME_SIZE, TxNumBD, TxServiceMax,
            RxNumBD, RxServiceMax, extra, AVALANCHE_MDIO_BASE,
            AVALANCHE_MDIO_RESET_BIT, PAL_sysClkcGetFreq(AVALANCHE_MDIO_REF_CLOCK),
            mdioClockFreq, mdioPhyMask, mdiotick,
            CPMAC_DDA_DEFAULT_MAX_RX_FRAGS, CPPI4TxChNum, CPPI4RxChNum,
            CPPI4DefRxDesc, CPPI4DefTxDesc,CPPI4NumTxQs, CPPI4TxQMgr0, CPPI4TxQNum0,
            CPPI4TxQMgr1, CPPI4TxQNum1, CPPI4TxCmplQMgr, CPPI4TxCmplQNum,
            CPPI4FbqmgrIndex[0], CPPI4FbqIndex[0],
            CPPI4FbqmgrIndex[1], CPPI4FbqIndex[1],
            CPPI4FbqmgrIndex[2], CPPI4FbqIndex[2],
            CPPI4FbqmgrIndex[3], CPPI4FbqIndex[3],
            CPPI4RqmgrIndex, CPPI4RqIndex,
            accTxChNum, accRxChNum,
            accTxVecNum, accRxVecNum,
            AVALANCHE_NWSS_RESET_BIT);   /*AVALANCHE_NWSS_RESET_BIT name should be changed */


    dbgPrint("%s.\n", cpmac_cfg[cfg_instance]);
    cfg_instance++;

    return (0);
}


static int cpmac_cfg_probe(void)
{
    int external_switch = 0;


#if defined (CONFIG_AVALANCHE_LOW_CPMAC)
    external_switch = AVALANCHE_LOW_CPMAC_HAS_EXT_SWITCH;
    if (cpmac_cfg_build(LOW_CPMAC, external_switch))
        return (-1);
#endif

#if defined (CONFIG_AVALANCHE_HIGH_CPMAC)
    sdfsdf
    external_switch = AVALANCHE_HIGH_CPMAC_HAS_EXT_SWITCH;
    if (cpmac_cfg_build(HIGH_CPMAC, external_switch))
        return (-1);
#endif

    return (0);
}

/*
 * L3 Alignment mechanism:
 * The below given macro returns the number of bytes required to align the given size
 * to a L3 frame 4 byte boundry. This is typically required to add 2 bytes to the ethernet
 * frame start to make sure the IP header (L3) is aligned on a 4 byte boundry
 */

static char cpmac_L3_align[] = { 0x02, 0x01, 0x00, 0x03 };

#define CPMAC_L3_ALIGN(size)    cpmac_L3_align[(size) & 0x3]

/* 4 Byte alignment for skb's:
 * Currently Skb's dont need to be on a 4 byte boundry, but other OS's have such requirements
 * Just to make sure we comply if there is any such requirement on SKB's in future, we align
 * it on a 4 byte boundry.
 */
char cpmac_4byte_align[] = { 0x0, 0x03, 0x02, 0x1 };

#define CPMAC_4BYTE_ALIGN(size) cpmac_4byte_align[(size) & 0x3]


/* Function prototype for cpmac_p_tick_timer_expiry() function as per Linux Timer API */
typedef void (*timer_tick_func) (unsigned long);

/* DDA function table */
static PAL_Result DDA_cpmac_control_cb(CpmacNetDevice * hDDA, Int cmd,
                                       Ptr cmdArg, Ptr param);

/* Static CPMAC DDA function table */
static CpmacDDACbIf cpmac_DDA_cb_interface = {
    {                           /* DDC Net Class functions */
     {                          /*   DDC Class functions */
      (DDA_ControlCb) DDA_cpmac_control_cb      /*     Control Callback function */
      },
     NULL,                      /*   DDA_NetRxNotifyCb function - Not supported/required */
     (DDA_NetAllocRxBufCb) DDA_cpmac_net_alloc_rx_buf,  /*   DDA_NetAllocRxBufCb function */
     (DDA_NetFreeRxBufCb) DDA_cpmac_net_free_rx_buf,    /*   DDA_NetFreeRxBufCb function */

     (DDA_NetRxCb) DDA_cpmac_net_rx,    /*   DDA_NetRxCb function */
     NULL,                      /*   DDA_NetRxMultipleCb */
     (DDA_NetTxCompleteCb) DDA_cpmac_net_tx_complete    /*   DDA_NetTxCompleteCb function */
     },
    (DDA_Printf) printk,        /* DDA printf function */
    (DDA_ErrLog) printk,        /* DDA error log function */
};

/******************************************************************************
 *  Internal Utility functions
 *****************************************************************************/

/* String to Hex conversion */
static unsigned char cpmac_str_to_hexnum(unsigned char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return 0;
}

/* String to ethernet address conversion */
static void cpmac_str_to_ethaddr(unsigned char *ea, unsigned char *str)
{
    int i;
    unsigned char num;
    for (i = 0; i < 6; i++) {
        if ((*str == '.') || (*str == ':'))
            str++;
        num = cpmac_str_to_hexnum(*str++) << 4;
        num |= (cpmac_str_to_hexnum(*str++));
        ea[i] = num;
    }
}


/******************************************************************************
 *  DDA Callback functions
 *****************************************************************************/

/* DDA_cpmac_control_cb - ioctl function to be called by the DDC */
static PAL_Result
DDA_cpmac_control_cb(CpmacNetDevice * hDDA, Int cmd, Ptr cmdArg, Ptr param)
{
    switch (cmd) {

    case CPMAC_DDA_IOCTL_TIMER_START:
        {
            /* cmdArg will directly have the timer period of the periodic
             * timer, param not used */
            /* DDC asks for milliSecs. So calculate ticks from ticks per
             * 1000 mSec
             */
            struct timer_list *p_timer = &hDDA->periodicTimer;
            hDDA->periodicTicks = (CPMAC_DDA_TICKS_PER_SEC * (Int) cmdArg) / 1000;
            p_timer->expires = jiffies + hDDA->periodicTicks;
            add_timer(&hDDA->periodicTimer);
            hDDA->timerActive = TRUE;
        }
        break;

    case CPMAC_DDA_IOCTL_TIMER_STOP:
        {
            /* cmdArg and param not used */
            if (hDDA->timerActive == TRUE) {
                del_timer_sync(&hDDA->periodicTimer);
                hDDA->timerActive = FALSE;
            }
        }
        break;

    case CPMAC_DDA_IOCTL_STATUS_UPDATE:
        {                       /* cmdArg will point to CpmacDDCStatus structure, param not
                                 * used
                                 */
            struct net_device *p_dev = hDDA->owner;
            CpmacDDCStatus *status = &hDDA->ddcStatus;
            hDDA->ddcStatus = *((CpmacDDCStatus *) cmdArg);
            if ((status->hwStatus & CPMAC_DDC_TX_HOST_ERROR) ==
                CPMAC_DDC_TX_HOST_ERROR) {
                errPrint("DDA_cpmac_control_cb: TX Host Error. Transmit Stopped %s\n", p_dev->name);
            }
            if ((status->hwStatus & CPMAC_DDC_RX_HOST_ERROR) ==
                CPMAC_DDC_RX_HOST_ERROR) {
                errPrint("DDA_cpmac_control_cb: RX Host Error. Receive Stopped %s\n", p_dev->name);
            }
            if (status->PhyLinked) {
                /* Link ON */
                if (!netif_carrier_ok(p_dev))   {
                    netif_carrier_on(p_dev);
#if defined (CONFIG_ARM_AVALANCHE_COLORED_LED)
                    led_manager_led_action(hDDA->ledHandle, CPMAC_LINK_ON);
#endif


                }
#if defined(CONFIG_MACH_PUMA5EVM)
                hDDA->linkSpeed =
                    (status->PhySpeed == 2) ? 1000000000 : ((status->PhySpeed) ? 100000000 : 10000000 );
#else
                hDDA->linkSpeed =
                    ((status->PhySpeed) ? 100000000 : 10000000);

#endif
                hDDA->linkMode = ((status->PhyDuplex) ? 3 : 2);
#if defined(CONFIG_MACH_PUMA5EVM)

                //hDDA->initCfg.macInitCfg.macCfg.fullduplex = status->PhyDuplex;
                cpmacSetMacHwCfg(hDDA->hDDC);
#endif
                /* Reactivate the transmit queue if it is stopped */
                if (netif_running(p_dev) && netif_queue_stopped(p_dev)) {
                    netif_wake_queue(p_dev);
                }
#if defined (CONFIG_ARM_AVALANCHE_COLORED_LED)
                led_manager_led_action(hDDA->ledHandle, (status->PhySpeed) ? CPMAC_SPEED_100 : CPMAC_SPEED_10);
                led_manager_led_action(hDDA->ledHandle, (status->PhyDuplex) ? CPMAC_FULL_DPLX : CPMAC_HALF_DPLX);
#endif

            } else {
                /* Link OFF */
                if (netif_carrier_ok(p_dev)) {
                    /* do we need to register synchronization issues with stats here. */
                    hDDA->linkSpeed = 100000000;
                    hDDA->linkMode = 1;
                    netif_carrier_off(p_dev);
#if defined (CONFIG_ARM_AVALANCHE_COLORED_LED)
                    led_manager_led_action(hDDA->ledHandle, CPMAC_SPEED_100);
                    led_manager_led_action(hDDA->ledHandle, CPMAC_FULL_DPLX);
                    led_manager_led_action(hDDA->ledHandle, CPMAC_LINK_OFF);
#endif
                }

                if (!netif_queue_stopped(p_dev)) {
                    netif_stop_queue(p_dev);    /*So that kernel does not keep on xmiting pkts. */
                }


            }

            if (cpmac_link_status)
                dbgPrint("\n%s, Inst %d, PhyNum %d,  %s, %s, %s",
                         ((struct net_device *) hDDA->owner)->name,
                         hDDA->initCfg.instId, status->PhyNum,
                         ((status->
                           PhyDuplex) ? "Full Duplex" : "Half Duplex"),
                         ((status->PhySpeed) ? "100 Mbps" : "10 Mbps"),
                         ((status->PhyLinked) ? "Linked" : "NO LINK"));
        }
        break;

    case CPMAC_DDA_IOCTL_MIB64_CNT_TIMER_START:
        {
            /* cmdArg will directly have the timer period of the
             * periodic timer, param not used
             */
            /* DDC asks for milliSecs. So calculate ticks from ticks per 1000 mSec */
            struct timer_list *p_timer = &hDDA->mibTimer;
            hDDA->mibTicks = (CPMAC_DDA_TICKS_PER_SEC * (Int) cmdArg) / 1000;
            p_timer->expires = jiffies + hDDA->mibTicks;
            add_timer(p_timer);
            hDDA->mibTimerActive = TRUE;

        }
        break;

    case CPMAC_DDA_IOCTL_MIB64_CNT_TIMER_STOP:
        {
            /* cmdArg and param not used */
            if (hDDA->mibTimerActive == TRUE) {
                del_timer_sync(&hDDA->mibTimer);
                hDDA->mibTimerActive = FALSE;
            }
        }
        break;

    default:
        dbgPrint("Cpmac: Unhandled ioctl code %d in DDA_cpmac_control_cb\n", cmd);
        break;
    }

    return (CPMAC_SUCCESS);
}


/******************************************************************************
 *  Internal functions
 *****************************************************************************/

/*******************************************************************************
 *
 * cpmacEndGetConfig - Extract configuration for given unit number/instance
 *
 * This function gets the configuration information from the configuration service
 * or by some means for the given unit number/cpmac instance
 *
 * Note: For debug/default, static information is obtained from the header file
 *
 * RETURNS: OK or ERROR.
 */
static int cpmac_net_get_config(CpmacNetDevice * hDDA)
{
#if defined (CPMAC_USE_CONFIG_SERVICE)

#error " CPMAC_USE_CONFIG_SERVICE not supported as of now"

#else

#define CPMAC_TOKEN_PARSE(str)       { if ((tok = (char *)strsep ((str), ":")) == NULL) return -1; }
#define CPMAC_TOKEN_GET_INTEGER simple_strtoul (tok, NULL, 10)
#define CPMAC_TOKEN_GET_HEX     simple_strtoul (tok, NULL, 16)

    {
        CpmacInitConfig *iCfg = &hDDA->initCfg;
        CpmacChInfo *txChCfg = &hDDA->txChInfo;
        CpmacChInfo *rxChCfg = &hDDA->rxChInfo;
        int indx;
        int speed, duplex, extra;
        char localStringVal[400];
        char *localString = NULL;
        char *tok;
        char *pHolder = NULL;

#ifdef CPMAC_USE_ENV            /* Env string configuration */
        char *instanceName = NULL;
        switch (hDDA->instanceNum) {
        case 0:
            instanceName = CPMAC_DDA_CONFIG_A;
            break;
        case 1:
            instanceName = CPMAC_DDA_CONFIG_B;
            break;
        case 2:
            instanceName = CPMAC_DDA_CONFIG_C;
            break;
        case 3:
            instanceName = CPMAC_DDA_CONFIG_D;
            break;
        case 4:
            instanceName = CPMAC_DDA_CONFIG_E;
            break;
        case 5:
            instanceName = CPMAC_DDA_CONFIG_F;
            break;
        default:
            instanceName = "";
            break;
        }
        localString = (char *) prom_getenv(instanceName);

        if (localString == NULL) {
            printk("Error getting CPMAC Configuration params for instance:%d\n", hDDA->instanceNum);
            printk("Environment Variable:%s not set in bootloader\n",
                   instanceName);
            printk("Setting Default configuration params for CPMAC instance:%d\n", hDDA->instanceNum);

            switch (hDDA->instanceNum) {
            case 0:
                localString = cpmac_cfg[0];
                break;
            case 1:
                localString = cpmac_cfg[1];
                break;
            default:
                localString = cpmac_cfg[0];
                break;
            }
        }


#else /* Use static config string */
        switch (hDDA->instanceNum) {
        case 0:
            localString = cpmac_cfg[0];
            break;
        case 1:
            localString = cpmac_cfg[1];
            break;
        default:
            localString = cpmac_cfg[0];
            break;
        }
#endif
        strcpy(&localStringVal[0], localString);
        localString = &localStringVal[0];
        pHolder = NULL;
        tok = (char *) strsep(&localString, ":");

        if (tok == NULL) {

            return (-1);
        }

        /* New DDA layer configuration starts from here */

        iCfg->instId = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("iCfg->instId=%d", iCfg->instId);

        iCfg->baseAddress = CPMAC_TOKEN_GET_HEX;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\niCfg->baseAddress=%08X", iCfg->baseAddress);

        iCfg->TxIntrLine = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\niCfg->TxIntrLine=%d", iCfg->TxIntrLine);

        iCfg->RxIntrLine = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\niCfg->RxIntrLine=%d", iCfg->RxIntrLine);

        iCfg->cpmacBusFrequency = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\niCfg->CPMACBusFrequency=%d", iCfg->cpmacBusFrequency);

        speed = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\nspeed=%d", speed);

        duplex = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\nduplex=%d", duplex);

        iCfg->rxCfg.promiscousEnable = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\niCfg->rxCfg.promiscousEnable=%d", iCfg->rxCfg.promiscousEnable);

        iCfg->rxCfg.broadcastEnable = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\niCfg->rxCfg.broadcastEnable=%d", iCfg->rxCfg.broadcastEnable);

        iCfg->rxCfg.multicastEnable = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\niCfg->rxCfg.multicastEnable=%d", iCfg->rxCfg.multicastEnable);

        iCfg->rxCfg.maxRxPktLength = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\niCfg->rxCfg.maxRxPktLength=%d", iCfg->rxCfg.maxRxPktLength);


        txChCfg->numBD = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\ntxChCfg->numBD=%d", txChCfg->numBD);

        txChCfg->serviceMax = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\ntxChCfg->serviceMax=%d", txChCfg->serviceMax);

        rxChCfg->numBD = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\nrxChCfg->numBD=%d", rxChCfg->numBD);

        rxChCfg->serviceMax = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\nrxChCfg->serviceMax=%d", rxChCfg->serviceMax);

        extra = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\nextra=%d", extra);


        iCfg->mdioBaseAddress = CPMAC_TOKEN_GET_HEX;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\niCfg->mdioBaseAddress=%08X", iCfg->mdioBaseAddress);


        iCfg->mdioResetLine = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\niCfg->mdioResetLine=%d", iCfg->mdioResetLine);

        iCfg->MdioBusFrequency = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\niCfg->MdioBusFrequency=%d", iCfg->MdioBusFrequency);

        iCfg->MdioClockFrequency = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\niCfg->MdioClockFrequency=%d", iCfg->MdioClockFrequency);

        iCfg->PhyMask = CPMAC_TOKEN_GET_HEX;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\niCfg->PhyMask=%08X", iCfg->PhyMask);

        iCfg->MdioTickMSec = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\niCfg->MdioTickMSec=%d", iCfg->MdioTickMSec);
        dbgPrint("\n");


        iCfg->rxMaxFrags = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\niCfg->rxMaxFrags=%d", iCfg->rxMaxFrags);
        dbgPrint("\n");


        txChCfg->cppi4TxChInfo.chNum = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\ntx_ch_cfg->cppi4TxChInfo.chnum=%d", txChCfg->cppi4TxChInfo.chNum);
        dbgPrint("\n");

        rxChCfg->cppi4RxChInfo.chNum = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\nrx_ch_cfg->cppi4RxChInfo.chnum=%d", rxChCfg->cppi4RxChInfo.chNum);
        dbgPrint("\n");

        rxChCfg->cppi4RxChInfo.defDescType = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\nrx_ch_cfg->cppi4RxChInfo.defDescType=%d", rxChCfg->cppi4RxChInfo.defDescType);
        dbgPrint("\n");

        txChCfg->cppi4TxChInfo.defDescType = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\nrx_ch_cfg->cppi4RxChInfo.defDescType=%d", rxChCfg->cppi4RxChInfo.defDescType);
        dbgPrint("\n");


        txChCfg->numTxQs = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\ntx_ch_cfg->numTxQs=%d", txChCfg->numTxQs);
        dbgPrint("\n");

        for (indx = 0; indx < txChCfg->numTxQs; indx++) {
            txChCfg->cppi4TxChInfo.txInQueue [indx].qMgr
                = CPMAC_TOKEN_GET_INTEGER;
            CPMAC_TOKEN_PARSE(&localString);
            dbgPrint("\ntx_ch_cfg->cppi4TxChInfo.txInQueue [%d].qMgr=%d",
                     indx, txChCfg->cppi4TxChInfo.txInQueue [indx].qMgr);
            dbgPrint("\n");

            txChCfg->cppi4TxChInfo.txInQueue [indx].qNum
                = CPMAC_TOKEN_GET_INTEGER;
            CPMAC_TOKEN_PARSE(&localString);
            dbgPrint("\ntx_ch_cfg->cppi4TxChInfo.txInQueue [%d].qNum=%d",
                     indx, txChCfg->cppi4TxChInfo.txInQueue [indx].qNum);
            dbgPrint("\n");
        }

        txChCfg->cppi4TxChInfo.txCompQueue.qMgr = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\ntx_ch_cfg->cppi4TxChInfo.txCompQueue.qMgr=%d",
                 txChCfg->cppi4TxChInfo.txCompQueue.qMgr);
        dbgPrint("\n");

        txChCfg->cppi4TxChInfo.txCompQueue.qNum = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\ntx_ch_cfg->cppi4TxChInfo.txCompQueue.qNum=%d",
                 txChCfg->cppi4TxChInfo.txCompQueue.qNum);
        dbgPrint("\n");

        for (indx = 0; indx < 4; indx++) {
            rxChCfg->cppi4RxChInfo.u.hostPktCfg.fdbQueue [indx].qMgr
               = CPMAC_TOKEN_GET_INTEGER;
            CPMAC_TOKEN_PARSE(&localString);
            dbgPrint("\nrx_ch_cfg->cppi4RxChInfo.u.hostPktCfg.fdbQueue [%d]"
                    ".qMgr=%d", indx,
                    rxChCfg->cppi4RxChInfo.u.hostPktCfg.fdbQueue [indx].qMgr);

            rxChCfg->cppi4RxChInfo.u.hostPktCfg.fdbQueue [indx].qNum
               = CPMAC_TOKEN_GET_INTEGER;
            CPMAC_TOKEN_PARSE(&localString);
            dbgPrint("\nrx_ch_cfg->cppi4RxChInfo.u.hostPktCfg.fdbQueue [%d]"
                    ".qNum=%d", indx,
                    rxChCfg->cppi4RxChInfo.u.hostPktCfg.fdbQueue [indx].qNum);
        }

        rxChCfg->cppi4RxChInfo.rxCompQueue.qMgr = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\nrx_ch_cfg->cppi4RxChInfo.rxCompQueue.qMgr=%d",
                 rxChCfg->cppi4RxChInfo.rxCompQueue.qMgr);
        dbgPrint("\n");

        rxChCfg->cppi4RxChInfo.rxCompQueue.qNum = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\nrx_ch_cfg->cppi4RxChInfo.rxCompQueue.qNum=%d",
                 rxChCfg->cppi4RxChInfo.rxCompQueue.qNum);
        dbgPrint("\n");

        txChCfg->txAccChInfo.accChanNum = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\ntxChCfg->cppi4TxChInfo.accuCfg.accChanNum=%d",
                txChCfg->txAccChInfo.accChanNum);
        dbgPrint("\n");

        rxChCfg->rxAccChInfo.accChanNum = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\nrxChCfg->cppi4RxChInfo.accuCfg.accChanNum=%d",
                rxChCfg->rxAccChInfo.accChanNum);
        dbgPrint("\n");

        txChCfg->accVecNum = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\ntxChCfg->accVecNum=%d",
                txChCfg->accVecNum);
        dbgPrint("\n");

        rxChCfg->accVecNum = CPMAC_TOKEN_GET_INTEGER;
        CPMAC_TOKEN_PARSE(&localString);
        dbgPrint("\nrxChCfg->accVecNum=%d",
                rxChCfg->accVecNum);
        dbgPrint("\n");

        iCfg->cppi4InitCfg.resetLine = CPMAC_TOKEN_GET_INTEGER;
        dbgPrint("\niCfg.cppi4InitCfg.resetLine=%d", iCfg->cppi4InitCfg.resetLine);


        rxChCfg->cppi4RxChInfo.dmaNum = CPMAC_RX_DMA_BLOCK_NUM;
        dbgPrint("\nrx_ch_cfg->cppi4RxChInfo.dmaNum=%d",
                 rxChCfg->cppi4RxChInfo.dmaNum);
        dbgPrint("\n");

        txChCfg->cppi4TxChInfo.dmaNum = CPMAC_TX_DMA_BLOCK_NUM;
        dbgPrint("\ntx_ch_cfg->cppi4TxChInfo.dmaNum=%d",
                 txChCfg->cppi4TxChInfo.dmaNum);
        dbgPrint("\n");

        iCfg->Mib64CntMsec = CONFIG_CPMAC_MIB_TIMER_TIMEOUT;

        rxChCfg->bufSize = iCfg->rxCfg.maxRxPktLength;
        hDDA->rxBufOffset = CPMAC_L3_ALIGN(extra);
        hDDA->rxBufSize = (rxChCfg->bufSize + hDDA->rxBufOffset);
        /* align skb's on 4 byte boundry - no hard requirement currently - done for future use */
        hDDA->rxBufSize += CPMAC_4BYTE_ALIGN(hDDA->rxBufSize);


        /* Save NAPI related constants */
        hDDA->napiRxTx.rxMaxService = rxChCfg->serviceMax;
        hDDA->napiRxTx.txMaxService = txChCfg->serviceMax;

        /* determine phy speed/duplex mode - to be built as per MDIO
         * module requirements */
        if (speed == CONFIG_CPMAC_NOPHY) {
            iCfg->phyMode = SNWAY_NOPHY;
        } else {
            if ((speed == 0) && (duplex == 0)) {
                iCfg->phyMode = SNWAY_AUTOALL;  /* Auto detection */
            } else if (speed == 10) {
                if (duplex == 1) {
                    iCfg->phyMode = SNWAY_HD10;
                } else if (duplex == 2) {
                    iCfg->phyMode = SNWAY_FD10;
                } else {
                    iCfg->phyMode = SNWAY_AUTO| SNWAY_HD10 | SNWAY_FD10;
                }
            } else if (speed == 100) {
                if (duplex == 1) {
                    iCfg->phyMode = SNWAY_HD100;
                } else if (duplex == 2) {
                    iCfg->phyMode = SNWAY_FD100;
                } else {
                    iCfg->phyMode = SNWAY_AUTO | SNWAY_HD100 | SNWAY_FD100;
                }
            } else {
                if (duplex == 1) {
                    iCfg->phyMode = SNWAY_AUTO | SNWAY_HD10 | SNWAY_HD100;
                } else {
                    iCfg->phyMode = SNWAY_AUTO | SNWAY_FD10 | SNWAY_FD100;
                }
            }
        }


        hDDA->vlanEnable = CPMAC_DDA_DEFAULT_VLAN_ENABLE;
        iCfg->numTxChannels = CPMAC_DDA_DEFAULT_NUM_TX_CHANNELS;
        iCfg->numRxChannels = CPMAC_DDA_DEFAULT_NUM_RX_CHANNELS;
        iCfg->MLinkMask = CPMAC_DEFAULT_MLINK_MASK;


        iCfg->rxCfg.copyMACControlFramesEnable = CPMAC_DEFAULT_COPY_MAC_CONTROL_FRAMES_ENABLE;
        iCfg->rxCfg.copyShortFramesEnable = CPMAC_DEFAULT_COPY_SHORT_FRAMES_ENABLE;
        iCfg->rxCfg.copyErrorFramesEnable = CPMAC_DEFAULT_COPY_ERROR_FRAMES_ENABLE;
        iCfg->rxCfg.promiscousChannel = CPMAC_DEFAULT_PROMISCOUS_CHANNEL;
        iCfg->rxCfg.broadcastChannel = CPMAC_DEFAULT_BROADCAST_CHANNEL;
        iCfg->rxCfg.multicastChannel = CPMAC_DEFAULT_MULTICAST_CHANNEL;


        /* Newly added according to configuration structrue presently set to 0 */
        iCfg->rxCfg.rxFlowThreshold = CPMAC_DEFAULT_RX_FLOW_THRESHOLD;
        iCfg->rxCfg.hiPriThresh = CPMAC_DEFAULT_HI_PRI_THRESHOLD;


        /* Newly added according to new configuration structure vlan is disable */
        iCfg->macCfg.rxVlanEn = CPMAC_DEFAULT_VLAN_ENABLE;


        iCfg->macCfg.extEn = CPMAC_DEFAULT_EXT_ENABLE;

        iCfg->macCfg.gigForce = CPMAC_DEFAULT_GIG_FORCE;

        iCfg->macCfg.rxFifoFlowEnable = CPMAC_DEFAULT_RX_FIFO_FLOW_ENABLE;


        iCfg->macCfg.txShortGapEnable = CPMAC_DEFAULT_TX_SHORT_GAP_ENABLE;

        iCfg->macCfg.txPacingEnable =
            CPMAC_DEFAULT_TX_PACING_ENABLE(hDDA->instanceNum);


        iCfg->macCfg.txFlowEnable = CPMAC_DEFAULT_TX_FLOW_ENABLE;
        iCfg->macCfg.rxFlowEnable = CPMAC_DEFAULT_RX_FLOW_ENABLE;
        iCfg->macCfg.loopbackEnable = CPMAC_DEFAULT_LOOPBACK_ENABLE;

        iCfg->macCfg.cmdIdle = CPMAC_DEFAULT_CMD_IDLE;
        iCfg->macCfg.ifCtlB = CPMAC_DEFAULT_IFCTL_B;
        iCfg->macCfg.ifCtlA = CPMAC_DEFAULT_IFCTL_A;


        iCfg->macCfg.portVlan = CPMAC_DEFAULT_VLAN_PORT;


        txChCfg->chNum = CPMAC_DDA_DEFAULT_TX_CHANNEL;
        txChCfg->chDir = DDC_NET_CH_DIR_TX;
        txChCfg->chState = DDC_NET_CH_UNINITIALIZED;
        rxChCfg->chNum = CPMAC_DDA_DEFAULT_RX_CHANNEL;
        rxChCfg->chDir = DDC_NET_CH_DIR_RX;
        rxChCfg->chState = DDC_NET_CH_UNINITIALIZED;

    /* Set the DMA mode and Decriptor type for the channel */
        txChCfg->dmaMode = CPPI41_DMA_MODE_ENDPOINT;
        rxChCfg->dmaMode = CPPI41_DMA_MODE_ENDPOINT;




        /* Set to 0 */
        rxChCfg->cppi4RxChInfo.sopOffset = CPMAC_DEFAULT_CPPI4_SOP_OFFSET;
        //rxChCfg->cppi4RxChInfo.retryOnStarvation = 0x1;

    }

#endif /* Config service or other mechanism */

    return (0);
}


/* Detect Manual config */
static int __init
cpmac_p_detect_manual_cfg(int linkSpeed, char *linkMode, int debug)
{
    char *pSpeed = NULL;
    if (debug == 1) {
        cpmac_debug_mode = 1;
        dbgPrint("Cpmac: Enabled the debug print.\n");
    }

    if (!linkSpeed && !linkMode) {
        dbgPrint
            ("Cpmac: No manual link params, defaulting to auto negotiation.\n");
        return (0);
    }

    if (!linkSpeed || (linkSpeed != 10 && linkSpeed != 100)) {
        dbgPrint
            ("Cpmac: Invalid or No value of link speed specified, defaulting to auto speed.\n");
        pSpeed = "auto";
    }

    if (!linkMode || (!strcmp(linkMode, "fd") && !strcmp(linkMode, "hd")))
        dbgPrint
            ("Cpmac: Invalid or No value of link mode specified, defaulting to auto mode.\n");

    if ((linkSpeed == 10) && (strcmp(linkMode, "fd"))) {
        g_cfg_start_link_params = SNWAY_FD10;
        pSpeed = "10";
    } else if ((linkSpeed == 10) && (strcmp(linkMode, "hd"))) {
        g_cfg_start_link_params = SNWAY_HD10;
        pSpeed = "10";
    } else if ((linkSpeed == 100) && (strcmp(linkMode, "hd"))) {
        g_cfg_start_link_params = SNWAY_HD100;
        pSpeed = "100";
    } else if ((linkSpeed == 100) && (strcmp(linkMode, "fd"))) {
        g_cfg_start_link_params = SNWAY_FD100;
        pSpeed = "100";
    } else if ((linkSpeed == 1000) && (strcmp(linkMode, "fd"))) {
        g_cfg_start_link_params = SNWAY_FD1000;
        pSpeed = "1000";
    } else if ((linkSpeed == 1000) && (!strcmp(linkMode, "hd"))) {
        g_cfg_start_link_params = SNWAY_HD1000;
        pSpeed = "1000";
    }

    dbgPrint("Link is manually set to the speed of %s speed and %s mode.\n", pSpeed, linkMode ? linkMode : "auto");

    return (0);
}

/* Link read support */
static int
cpmac_p_read_link(char *buf, char **start, off_t offset, int count,
                  int *eof, void *data)
{
    int len = 0;
    struct net_device *p_dev;
    CpmacNetDevice *hDDA;
    struct net_device *cpmac_dev_list[cpmac_devices_installed];
    int i;

    len +=
        sprintf(buf + len, "CPMAC devices = %d\n",
                cpmac_devices_installed);
    p_dev = last_cpmac_device;

    /* Reverse the the device link list to list eth0,eth1...in correct order */
    for (i = 0; i < cpmac_devices_installed; i++) {
        cpmac_dev_list[cpmac_devices_installed - (i + 1)] = p_dev;
        hDDA = NETDEV_PRIV(p_dev);
        p_dev = hDDA->nextDevice;
    }

    for (i = 0; i < cpmac_devices_installed; i++) {
        p_dev = cpmac_dev_list[i];
        hDDA = NETDEV_PRIV(p_dev);


        /*  This prints them out from high to low because of how the devices are linked */
        if (netif_carrier_ok(p_dev)) {
            char linkspeed[10];
            sprintf(linkspeed, "%d", hDDA->linkSpeed / 1000000);
            len +=sprintf(buf + len,
                        "eth%d: Link State: %s    Phy %d, Speed = %s, Duplex = %s\n",
                        hDDA->instanceNum, "UP", hDDA->ddcStatus.PhyNum, linkspeed,
                        (hDDA->linkMode == 2) ? "Half" : "Full");

        } else {
            len += sprintf(buf + len, "eth%d: Link State: DOWN\n", hDDA->instanceNum);
        }
        p_dev = hDDA->nextDevice;
    }

    return len;

}


/* Dump Configuration information for debug purposes */
static int
cpmac_dump_config(char *buf, char **start, off_t offset, int count,
                  int *eof, void *data)
{
    int len = 0;
    struct net_device *p_dev;
    CpmacNetDevice *hDDA;
    struct net_device *cpmac_dev_list[cpmac_devices_installed];
    int i;

    len += sprintf(buf + len, "CPMAC devices = %d\n", cpmac_devices_installed);
    p_dev = last_cpmac_device;

    /* Reverse the the device link list to list eth0,eth1...in correct order */
    for (i = 0; i < cpmac_devices_installed; i++) {
        cpmac_dev_list[cpmac_devices_installed - (i + 1)] = p_dev;
        hDDA = NETDEV_PRIV(p_dev);
        p_dev = hDDA->nextDevice;
    }

    for (i = 0; i < cpmac_devices_installed; i++) {
        p_dev = cpmac_dev_list[i];
        hDDA = NETDEV_PRIV(p_dev);

        len += sprintf(buf + len, "\nEMAC Driver Internal Config Info for Unit %d\n", hDDA->instanceNum);
        len += sprintf(buf + len, "vlanEnable         = %d\n", hDDA->vlanEnable);

        len += sprintf(buf + len, "rxBufSize          = %d\n", hDDA->rxBufSize);
        len += sprintf(buf + len, "rxBufOffset        = %d\n", hDDA->rxBufOffset);
        len += sprintf(buf + len, "instId             = %d\n", hDDA->initCfg.instId);
        len += sprintf(buf + len, "numTxChannels      = %d\n", hDDA->initCfg.numTxChannels);
        len += sprintf(buf + len, "numRxChannels      = %d\n", hDDA->initCfg.numRxChannels);
        len += sprintf(buf + len, "cpmacBusFrequency  = %d\n", hDDA->initCfg.cpmacBusFrequency);
        len += sprintf(buf + len, "baseAddress        = %08X\n", hDDA->initCfg.baseAddress);
        len += sprintf(buf + len, "RxMaxFrags         = %d\n", hDDA->initCfg.rxMaxFrags);
        len += sprintf(buf + len, "mdioBaseAddress    = %08X\n", hDDA->initCfg.mdioBaseAddress);
        len += sprintf(buf + len, "mdioResetLine      = %d\n", hDDA->initCfg.mdioResetLine);
        len += sprintf(buf + len, "PhyMask            = %08X\n", hDDA->initCfg.PhyMask);
        len += sprintf(buf + len, "MLinkMask          = %08X\n", hDDA->initCfg.MLinkMask);
        len += sprintf(buf + len, "MdioBusFrequency   = %d\n", hDDA->initCfg.MdioBusFrequency);
        len += sprintf(buf + len, "MdioClockFrequency = %d\n", hDDA->initCfg.MdioClockFrequency);
        len += sprintf(buf + len, "intrLine           = %d\n", hDDA->initCfg.TxIntrLine);
        len += sprintf(buf + len, "intrLine           = %d\n", hDDA->initCfg.RxIntrLine);
        len += sprintf(buf + len, "MdioTickMSec       = %d\n", hDDA->initCfg.MdioTickMSec);
        len += sprintf(buf + len, "phyMode            = %d\n", hDDA->initCfg.phyMode);
        len += sprintf(buf + len, "RxVlan             = %d\n", hDDA->initCfg.macCfg.rxVlanEn);
        len += sprintf(buf + len, "ExtEn              = %d\n", hDDA->initCfg.macCfg.extEn);
        len += sprintf(buf + len, "GigForce           = %d\n", hDDA->initCfg.macCfg.gigForce);
        len += sprintf(buf + len, "IfCtlb             = %d\n", hDDA->initCfg.macCfg.ifCtlB);
        len += sprintf(buf + len, "IfCtla             = %d\n", hDDA->initCfg.macCfg.ifCtlA);
        len += sprintf(buf + len, "RxFifoFlowEn       = %d\n", hDDA->initCfg.macCfg.rxFifoFlowEnable);
        len += sprintf(buf + len, "CmdIdle            = %d\n", hDDA->initCfg.macCfg.cmdIdle);
        len += sprintf(buf + len, "txShortGapEnable   = %d\n", hDDA->initCfg.macCfg.txShortGapEnable);
        len += sprintf(buf + len, "txPacingEnable     = %d\n", hDDA->initCfg.macCfg.txPacingEnable);
        len += sprintf(buf + len, "txFlowEnable       = %d\n", hDDA->initCfg.macCfg.txFlowEnable);
        len += sprintf(buf + len, "rxFlowEnable       = %d\n", hDDA->initCfg.macCfg.rxFlowEnable);
        len += sprintf(buf + len, "loopbackEnable     = %d\n", hDDA->initCfg.macCfg.loopbackEnable);
        len += sprintf(buf + len, "PortVlan           = %d\n", hDDA->initCfg.macCfg.portVlan);
        len += sprintf(buf + len, "HiPriThres         = %d\n", hDDA->initCfg.rxCfg.hiPriThresh);
        len += sprintf(buf + len, "copyMACCntrlFrsEne = %d\n", hDDA->initCfg.rxCfg.copyMACControlFramesEnable);
        len += sprintf(buf + len, "copyShortFramesEn  = %d\n", hDDA->initCfg.rxCfg.copyShortFramesEnable);
        len += sprintf(buf + len, "copyErrorFramesEn  = %d\n", hDDA->initCfg.rxCfg.copyErrorFramesEnable);
        len += sprintf(buf + len, "promiscousEnable   = %d\n", hDDA->initCfg.rxCfg.promiscousEnable);
        len += sprintf(buf + len, "promiscousChannel  = %d\n", hDDA->initCfg.rxCfg.promiscousChannel);
        len += sprintf(buf + len, "broadcastEnable    = %d\n", hDDA->initCfg.rxCfg.broadcastEnable);
        len += sprintf(buf + len, "broadcastChannel   = %d\n", hDDA->initCfg.rxCfg.broadcastChannel);
        len += sprintf(buf + len, "multicastEnable    = %d\n", hDDA->initCfg.rxCfg.multicastEnable);
        len += sprintf(buf + len, "multicastChannel   = %d\n", hDDA->initCfg.rxCfg.multicastChannel);
        len += sprintf(buf + len, "maxRxPktLength     = %d\n", hDDA->initCfg.rxCfg.maxRxPktLength);
        len += sprintf(buf + len, "RxFlowThres        = %d\n", hDDA->initCfg.rxCfg.rxFlowThreshold);


        /* CPPI4.0 based configuration */
        len += sprintf(buf + len, "cppi4InitCfg.resetLine        = %0d\n",  hDDA->initCfg.cppi4InitCfg.resetLine);

        p_dev = hDDA->nextDevice;
    }
    return len;
}

/* Read stats */
static int
cpmac_p_read_stats(char *buf, char **start, off_t offset, int count,
                   int *eof, void *data)
{
    struct net_device *p_dev = last_cpmac_device;
    int len = 0;
    int limit = count - 80;
    int i;
    struct net_device *cpmac_dev_list[cpmac_devices_installed];
    CpmacNetDevice *hDDA;
    CpmacHwStatistics *p_deviceMib;

    /* Reverse the the device link list to list eth0,eth1...in correct order */
    for (i = 0; i < cpmac_devices_installed; i++) {
        cpmac_dev_list[cpmac_devices_installed - (i + 1)] = p_dev;
        hDDA = NETDEV_PRIV(p_dev);
        p_dev = hDDA->nextDevice;
    }

    for (i = 0; i < cpmac_devices_installed; i++) {
        p_dev = cpmac_dev_list[i];

        if (!p_dev)
            goto proc_error;

        /* Get Stats */
        cpmac_p_update_statistics(p_dev, NULL, 0, NULL);
        hDDA = NETDEV_PRIV(p_dev);
        p_deviceMib = &hDDA->deviceMib;

        len +=
            sprintf(buf + len, "\nCpmac %d, Address %lx\n", i + 1,
                    p_dev->base_addr);
        /* Transmit stats */
        if (len <= limit)
            len += sprintf(buf + len, " Transmit Stats\n");
        if (len <= limit)
            len += sprintf(buf + len, "   Tx Valid Bytes Sent        :%u\n", p_deviceMib->ifOutOctets);
        if (len <= limit)
            len += sprintf(buf + len, "   Good Tx Frames (Hardware)  :%u\n", p_deviceMib->ifOutGoodFrames);
        if (len <= limit)
            len += sprintf(buf + len, "   Good Tx Frames (Software)  :%lu\n", hDDA->netDevStats.tx_packets);
        if (len <= limit)
            len += sprintf(buf + len, "   Good Tx Broadcast Frames   :%u\n", p_deviceMib->ifOutBroadcasts);
        if (len <= limit)
            len += sprintf(buf + len, "   Good Tx Multicast Frames   :%u\n", p_deviceMib->ifOutMulticasts);
        if (len <= limit)
            len += sprintf(buf + len, "   Pause Frames Sent          :%u\n", p_deviceMib->ifOutPauseFrames);
        if (len <= limit)
            len += sprintf(buf + len, "   Collisions                 :%u\n", p_deviceMib->ifCollisionFrames);
        if (len <= limit)
            len += sprintf(buf + len, "   Tx Error Frames            :%lu\n", hDDA->netDevStats.tx_errors);
        if (len <= limit)
            len += sprintf(buf + len, "   Carrier Sense Errors       :%u\n", p_deviceMib->ifCarrierSenseErrors);
        if (len <= limit)
            len += sprintf(buf + len, "\n");

        /* Receive Stats */
        if (len <= limit)
            len += sprintf(buf + len, "\nCpmac %d, Address %lx\n", i + 1, p_dev->base_addr);
        if (len <= limit)
            len += sprintf(buf + len, " Receive Stats\n");
        if (len <= limit)
            len += sprintf(buf + len, "   Rx Valid Bytes Received    :%u\n", p_deviceMib->ifInOctets);
        if (len <= limit)
            len += sprintf(buf + len, "   Good Rx Frames (Hardware)  :%u\n", p_deviceMib->ifInGoodFrames);
        if (len <= limit)
            len += sprintf(buf + len, "   Good Rx Frames (Software)  :%lu\n", hDDA->netDevStats.rx_packets);
        if (len <= limit)
            len += sprintf(buf + len, "   Good Rx Broadcast Frames   :%u\n", p_deviceMib->ifInBroadcasts);
        if (len <= limit)
            len += sprintf(buf + len, "   Good Rx Multicast Frames   :%u\n", p_deviceMib->ifInMulticasts);
        if (len <= limit)
            len += sprintf(buf + len, "   Pause Frames Received      :%u\n", p_deviceMib->ifInPauseFrames);
        if (len <= limit)
            len += sprintf(buf + len, "   Rx CRC Errors              :%u\n", p_deviceMib->ifInCRCErrors);
        if (len <= limit)
            len += sprintf(buf + len, "   Rx Align/Code Errors       :%u\n", p_deviceMib->ifInAlignCodeErrors);
        if (len <= limit)
            len += sprintf(buf + len, "   Rx Jabbers                 :%u\n", p_deviceMib->ifInJabberFrames);
        if (len <= limit)
            len += sprintf(buf + len, "   Rx Filtered Frames         :%u\n", p_deviceMib->ifInFilteredFrames);
        if (len <= limit)
            len += sprintf(buf + len, "   Rx Fragments               :%u\n", p_deviceMib->ifInFragments);
        if (len <= limit)
            len += sprintf(buf + len, "   Rx Undersized Frames       :%u\n", p_deviceMib->ifInUndersizedFrames);
        if (len <= limit)
            len += sprintf(buf + len, "   Rx Overruns                :%u\n", p_deviceMib->ifRxDMAOverruns);
        if (len <= limit)
            len += sprintf(buf + len, "   Rx Unknown Protocols       :%u\n", hDDA->unknownProtPkts);

    }
    return len;

  proc_error:
    *eof = 1;
    return len;
}

/* Write stats */
static int
cpmac_p_write_stats(struct file *fp, const char *buf, unsigned long count,
                    void *data)
{
    char local_buf[31];
    int ret_val = 0;

    if (count > 30) {
        printk("Error : Buffer Overflow\n");
        printk("Use \"echo 0 > cpmac_stat\" to reset the statistics\n");
        return -EFAULT;
    }

    copy_from_user(local_buf, buf, count);
    local_buf[count - 1] = '\0';        /* Ignoring last \n char */
    ret_val = count;

    if (strcmp("0", local_buf) == 0) {
        struct net_device *p_dev = last_cpmac_device;
        int i;
        struct net_device *cpmac_dev_list[cpmac_devices_installed];
        CpmacNetDevice *hDDA;

        /* Valid command */
        printk("Resetting statistics for CPMAC interface.\n");

        /* Reverse the the device link list to list eth0,eth1...in correct order */
        for (i = 0; i < cpmac_devices_installed; i++) {
            cpmac_dev_list[cpmac_devices_installed - (i + 1)] = p_dev;
            hDDA = NETDEV_PRIV(p_dev);
            p_dev = hDDA->nextDevice;
        }

        for (i = 0; i < cpmac_devices_installed; i++) {
            p_dev = cpmac_dev_list[i];
            if (!p_dev) {
                ret_val = -EFAULT;
                break;
            }
            cpmac_p_reset_statistics(p_dev);
        }
    }else{
        printk("Error: Unknown operation on cpmac statistics\n");
        printk("Use \"echo 0 > cpmac_stats\" to reset the statistics\n");
        return -EFAULT;
    }
    return ret_val;
}


/* Update RFC2665 Statistics */
static int
cpmac_p_read_rfc2665_stats(char *buf, char **start, off_t offset,
                           int count, int *eof, void *data)
{
    int limit = count - 80;
    int len = 0;
    struct net_device *p_dev = (struct net_device *) data;

    cpmac_p_update_statistics(p_dev, buf, limit, &len);
    *eof = 1;

    return len;
}

/* Reset Statistics */
static int cpmac_p_reset_statistics(struct net_device *p_dev)
{
    CpmacNetDevice *hDDA = NETDEV_PRIV(p_dev);


    if(!netif_running(p_dev))
    {
        printk("Error clearing statistics for %s as Interface is down:\n",p_dev->name);
        return -1;
    }
    memset(&hDDA->deviceMib, 0, sizeof(CpmacHwStatistics));
    memset(&hDDA->deviceStats, 0, sizeof(CpmacDrvStats));
    memset(&hDDA->netDevStats, 0, sizeof(struct net_device_stats));
    hDDA->unknownProtPkts = 0;

    /* Clear statistics */
    if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_CLR_STATISTICS, NULL,
                   NULL) != CPMAC_SUCCESS) {
        errPrint("cpmac_p_reset_statistics: Error clearing statistics in DDC for %s\n", p_dev->name);
        return (-1);
    }

    return (0);
}

/* Update Statistics */
static int
cpmac_p_update_statistics(struct net_device *p_dev, char *buf, int limit,
                          int *p_len)
{
    unsigned long rx_hal_errors = 0;
    unsigned long rx_hal_discards = 0;
    unsigned long tx_hal_errors = 0;
    unsigned long ifOutDiscards = 0;
    unsigned long ifInDiscards = 0;
    unsigned long ifOutErrors = 0;
    unsigned long ifInErrors = 0;
    CpmacNetDevice *hDDA = NETDEV_PRIV(p_dev);

    CpmacHwStatistics *p_deviceMib = &hDDA->deviceMib;
    CpmacDrvStats *p_stats = &hDDA->deviceStats;
    CpmacHwStatistics local_mib;
    CpmacHwStatistics *p_local_mib = &local_mib;

    struct net_device_stats *p_netDevStats = &hDDA->netDevStats;

    int len = 0;
    int dev_mib_elem_count = 0;

    /* do not access the hardware if it is in the reset state. */
    if (!test_bit(0, &hDDA->setToClose)) {

        /* Get hardware statistics from DDC */
        if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.
            ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_GET_STATISTICS,
                       (Ptr) p_local_mib, NULL) != CPMAC_SUCCESS) {
            errPrint("Error getting statistics from DDC for %s\n",
                     p_dev->name);
            return (-1);
        }

        dev_mib_elem_count = sizeof(CpmacHwStatistics) / sizeof(unsigned long);

        /* Update the history of the stats. This takes care of any reset of the
         * device and stats that might have taken place during the life time of
         * the driver.
         */
        while (dev_mib_elem_count--) {
            *((unsigned long *) p_deviceMib + dev_mib_elem_count) =
                *((unsigned long *) p_local_mib + dev_mib_elem_count);
        }
    }

    /* RFC2665, section 3.2.7, page 9 */
    rx_hal_errors = p_deviceMib->ifInFragments +
        p_deviceMib->ifInCRCErrors +
        p_deviceMib->ifInAlignCodeErrors + p_deviceMib->ifInJabberFrames;

    /* RFC2233 */
    rx_hal_discards = p_deviceMib->ifRxDMAOverruns;

    /* RFC2665, section 3.2.7, page 9 */
    tx_hal_errors = p_deviceMib->ifExcessiveCollisionFrames +
        p_deviceMib->ifLateCollisions +
        p_deviceMib->ifCarrierSenseErrors + p_deviceMib->ifOutUnderrun;

    /* if not set, the short frames (< 64 bytes) are considered as errors */
    if (hDDA->initCfg.rxCfg.copyShortFramesEnable == FALSE)
        rx_hal_errors += p_deviceMib->ifInUndersizedFrames;

    /* All frames greater than max rx frame length set in hardware should be considered error frames
     * RFC2665, section 3.2.7, page 9. */
    rx_hal_errors += p_deviceMib->ifInOversizedFrames;

    /* if not in promiscous, then non addr matching frames are discarded */
    /* CPMAC 2.0 Manual Section 2.8.1.14 */
    if (hDDA->initCfg.rxCfg.promiscousEnable == FALSE) {
        ifInDiscards += p_deviceMib->ifInFilteredFrames;
    }

    /* total rx discards = hal discards + driver discards. */
    ifInDiscards = rx_hal_discards + p_netDevStats->rx_dropped;
    ifInErrors = rx_hal_errors;

    ifOutErrors = tx_hal_errors;
    ifOutDiscards = p_netDevStats->tx_dropped;

    /* Let us update the net device stats struct. To be updated in the later releases. */
    hDDA->netDevStats.rx_errors = ifInErrors;
    hDDA->netDevStats.collisions = p_deviceMib->ifCollisionFrames;

    if (buf == NULL || limit == 0) {
        return (0);
    }

    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifSpeed", hDDA->linkSpeed);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "dot3StatsDuplexStatus", hDDA->linkMode);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifAdminStatus", (p_dev->flags & IFF_UP ? 1 : 2));
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifOperStatus", (((p_dev->flags & IFF_UP)
                      && netif_carrier_ok(p_dev)) ? 1 : 2));
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %lu\n", "ifLastChange", p_stats->start_tick);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %lu\n", "ifInDiscards", ifInDiscards);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %lu\n", "ifInErrors", ifInErrors);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %lu\n", "ifOutDiscards", ifOutDiscards);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %lu\n", "ifOutErrors", ifOutErrors);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifInGoodFrames", p_deviceMib->ifInGoodFrames);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifInBroadcasts", p_deviceMib->ifInBroadcasts);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifInMulticasts", p_deviceMib->ifInMulticasts);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifInPauseFrames", p_deviceMib->ifInPauseFrames);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifInCRCErrors", p_deviceMib->ifInCRCErrors);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifInAlignCodeErrors", p_deviceMib->ifInAlignCodeErrors);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifInOversizedFrames", p_deviceMib->ifInOversizedFrames);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifInJabberFrames", p_deviceMib->ifInJabberFrames);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifInUndersizedFrames", p_deviceMib->ifInUndersizedFrames);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifInFragments", p_deviceMib->ifInFragments);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifInFilteredFrames", p_deviceMib->ifInFilteredFrames);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifInQosFilteredFrames", p_deviceMib->ifInQosFilteredFrames);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifInOctets", p_deviceMib->ifInOctets);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifOutGoodFrames", p_deviceMib->ifOutGoodFrames);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifOutBroadcasts", p_deviceMib->ifOutBroadcasts);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifOutMulticasts", p_deviceMib->ifOutMulticasts);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifOutPauseFrames", p_deviceMib->ifOutPauseFrames);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifDeferredTransmissions", p_deviceMib->ifDeferredTransmissions);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifCollisionFrames", p_deviceMib->ifCollisionFrames);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifSingleCollisionFrames", p_deviceMib->ifSingleCollisionFrames);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifMultipleCollisionFrames", p_deviceMib->ifMultipleCollisionFrames);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifExcessiveCollisionFrames", p_deviceMib->ifExcessiveCollisionFrames);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifLateCollisions", p_deviceMib->ifLateCollisions);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifOutUnderrun", p_deviceMib->ifOutUnderrun);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifCarrierSenseErrors", p_deviceMib->ifCarrierSenseErrors);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifOutOctets", p_deviceMib->ifOutOctets);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "if64OctetFrames", p_deviceMib->if64OctetFrames);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "if65To127POctetFrames", p_deviceMib->if65To127OctetFrames);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "if128To255OctetFrames", p_deviceMib->if128To255OctetFrames);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "if256To511OctetFrames", p_deviceMib->if256To511OctetFrames);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "if512To1023OctetFrames", p_deviceMib->if512To1023OctetFrames);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "if1024ToUpOctetFrames", p_deviceMib->if1024ToUPOctetFrames);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifNetOctets", p_deviceMib->ifNetOctets);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifRxSofOverruns", p_deviceMib->ifRxSofOverruns);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifRxMofOverruns", p_deviceMib->ifRxMofOverruns);
    if (len <= limit)
        len += sprintf(buf + len, "%-35s: %u\n", "ifRxDMAOverruns", p_deviceMib->ifRxDMAOverruns);

    *p_len = len;

    return (0);
}

/* Version info */
static int
cpmac_p_get_version(char *buf, char **start, off_t offset, int count,
                    int *eof, void *data)
{
    int len = 0;
    int limit = count - 80;
    unsigned int ddc_version = 0;
    char *ddc_version_string = NULL;

    ddc_version_string = DDC_cpmacGetVersionInfo(&ddc_version);

    len += sprintf(buf + len, "Texas Instruments : %s\n", cpmac_DDA_version_string);

    if (len <= limit && ddc_version_string)
        len += sprintf(buf + len, "Texas Instruments : %s\n", ddc_version_string);

    return len;
}

/* Tick Timer */
static void cpmac_p_tick_timer_expiry(CpmacNetDevice * hDDA)
{
    struct timer_list *p_timer = &hDDA->periodicTimer;

#ifdef CONFIG_PSP_TRACE
    psp_trace(ETH_DRV_TMR_ENTER);
#endif


    /* do not recharge the timer if we are closing down and know that DDCOpen will be called again */
    if (test_bit(0, &hDDA->setToClose) && !g_init_enable_flag) {
        return;
    }

    if (hDDA->timerActive == TRUE) {
        hDDA->ddcIf->ddctick(hDDA->hDDC, NULL);

        /* Restart the timer */
        p_timer->expires = jiffies + hDDA->periodicTicks;
        add_timer(p_timer);
    }
#ifdef CONFIG_PSP_TRACE
    psp_trace(ETH_DRV_TMR_EXIT);
#endif

}

/* Mib Timer */
static void cpmac_p_mib_timer_expiry(CpmacNetDevice * hDDA)
{
    struct timer_list *p_timer = &hDDA->mibTimer;

    /* do not recharge the timer if we are closing down and know that DDCOpen will be called again */
    if (test_bit(0, &hDDA->setToClose) && !g_init_enable_flag) {
        return;
    }

    if (hDDA->mibTimerActive == TRUE) {
        hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_IF_PARAMS_UPDT, NULL, NULL);

        /* Restart the timer */
        p_timer->expires = jiffies + hDDA->mibTicks;
        add_timer(p_timer);
    }
}

#ifdef CPMAC_POLL_MODE
unsigned char p_buf[4096];
static void cpmac_p_txpoll (unsigned long data)
{
    CpmacNetDevice * hDDA = (CpmacNetDevice*) data;
    int pkts_pending = 0;
    int len;
#if defined CPMAC_POLL_MODE && defined CPGMAC_USE_ACC_LIST
    RxTxParams txParams;
#endif

#ifdef CONFIG_PSP_TRACE
    psp_trace(ETH_DRV_TMR_ENTER);
#endif

    if (hDDA->setToClose)
        goto txpoll_exit;

    /* printk ("x polling routine called...\n"); */
    /* Process packets - Call the DDC packet processing function */

/*
    cpmac_p_update_statistics(hDDA->owner, p_buf, 4096, &len);
    p_buf[len] = 0;
    printk(p_buf);
*/

#if defined CPMAC_POLL_MODE && defined CPGMAC_USE_ACC_LIST
    txParams.txPkts = hDDA->txChInfo.serviceMax;
    hDDA->ddcIf->txPktProcess(hDDA->hDDC, &pkts_pending, (Ptr)&txParams);
    hDDA->ddcIf->txPktProcessEnd(hDDA->hDDC, (Ptr)&txParams);
#else
    hDDA->ddcIf->txPktProcess(hDDA->hDDC, &pkts_pending, NULL);
    hDDA->ddcIf->txPktProcessEnd(hDDA->hDDC, NULL);
#endif
    hDDA->cpmac_txpoll_timer.expires    = jiffies + HZ*10;
    add_timer (&hDDA->cpmac_txpoll_timer);

txpoll_exit:
#ifdef CONFIG_PSP_TRACE
    psp_trace(ETH_DRV_TMR_EXIT);
#endif
    return;
}

static void cpmac_p_rxpoll (unsigned long data)
{
    CpmacNetDevice * hDDA = (CpmacNetDevice*) data;
    int pkts_pending = 0;
#if defined CPMAC_POLL_MODE && defined CPGMAC_USE_ACC_LIST
    RxTxParams rxParams;
#endif

#ifdef CONFIG_PSP_TRACE
    psp_trace(ETH_DRV_TMR_ENTER);
#endif

    if (hDDA->setToClose)
        goto rxpoll_exit;

    /* Process packets - Call the DDC packet processing function */
#if defined CPMAC_POLL_MODE && defined CPGMAC_USE_ACC_LIST
    rxParams.rxPkts = hDDA->rxChInfo.serviceMax;
    hDDA->ddcIf->rxPktProcess (hDDA->hDDC, &pkts_pending, (Ptr)&rxParams);
    hDDA->ddcIf->pktProcessEnd (hDDA->hDDC, (Ptr)&rxParams);
#else
    hDDA->ddcIf->rxPktProcess (hDDA->hDDC, &pkts_pending, NULL);
    hDDA->ddcIf->pktProcessEnd (hDDA->hDDC, NULL);
#endif

    hDDA->cpmac_rxpoll_timer.expires    = jiffies + HZ;
    add_timer (&hDDA->cpmac_rxpoll_timer);

rxpoll_exit:
#ifdef CONFIG_PSP_TRACE
    psp_trace(ETH_DRV_TMR_EXIT);
#endif
    return;
}
#endif /* CPMAC_POLL_MODE */

/******************************************************************************
 *  Device enable/disable functions
 *****************************************************************************/
/* Enable the device - Init TX/RX Channels and Open DDC */
static int cpmac_p_dev_enable(CpmacNetDevice * hDDA)
{
    PAL_Result retCode;
    struct net_device *p_dev = hDDA->owner;

#ifdef CONFIG_ARM_AVALANCHE_PPD
#ifdef CONFIG_TI_PACKET_PROCESSOR
    ti_ppm_set_pid_flags (p_dev->pid_handle, 0);
#else
    ti_ppd_set_pid_flags (&pid_eth[0], 0);
#endif
#endif

#ifdef CPMAC_POLL_MODE
    init_timer (&hDDA->cpmac_txpoll_timer);
    hDDA->cpmac_txpoll_timer.function   = cpmac_p_txpoll;
    hDDA->cpmac_txpoll_timer.data       = (unsigned long) hDDA;
    hDDA->cpmac_txpoll_timer.expires    = 0;
    init_timer (&hDDA->cpmac_rxpoll_timer);
    hDDA->cpmac_rxpoll_timer.function   = cpmac_p_rxpoll;
    hDDA->cpmac_rxpoll_timer.data       = (unsigned long) hDDA;
    hDDA->cpmac_rxpoll_timer.expires    = 0;
    /* The timers are started in cpmac_dev_open.
     * This ensures DDC_cpmacOpen has successfully returned.
     * TODO Find better way
     */
#endif

    /* Create a TX channel */
    retCode =
        hDDA->ddcIf->ddcNetIf.ddcNetChOpen((DDC_Handle *) hDDA->hDDC,
                                           (DDC_NetChInfo *) & hDDA->
                                           txChInfo, NULL);
    if (retCode != CPMAC_SUCCESS) {
        errPrint("%s error: Error %08X from CPMAC DDC TX Channel Open(())\n", p_dev->name, retCode);
        return (-1);
    }
    /* If packet processor is defined open Tx and Rx eth2host proxy
     * infra mode DMA channels.
     */
#ifdef CONFIG_ARM_AVALANCHE_PPD
    hDDA->infraTxChInfo.cppi4TxChInfo.chNum = CPGMAC_TX_INFRA_CHNUM;
    hDDA->infraTxChInfo.cppi4TxChInfo.dmaNum = CPMAC_INFRA_CH_DMA_BLOCK_NUM;
    hDDA->infraTxChInfo.cppi4TxChInfo.tdQueue.qMgr = DMA0_CPPI4x_FTD_QMGR;
    hDDA->infraTxChInfo.cppi4TxChInfo.tdQueue.qNum = DMA0_CPPI4x_FTD_QNUM;
    hDDA->infraTxChInfo.cppi4TxChInfo.defDescType = CPPI41_DESC_TYPE_EMBEDDED;
    hDDA->infraTxChInfo.chDir = DDC_NET_CH_DIR_TX;
    hDDA->infraTxChInfo.dmaMode = CPPI41_DMA_MODE_INFRA;
    retCode =
        hDDA->ddcIf->ddcNetIf.ddcNetChOpen((DDC_Handle *) hDDA->hDDC,
                                           (DDC_NetChInfo *) & hDDA->
                                           infraTxChInfo, NULL);
    if (retCode != CPMAC_SUCCESS) {
        errPrint("%s error: Error %08X from CPMAC DDC TX Inframode Channel Open(())\n", p_dev->name, retCode);
        return (-1);
    }
    retCode =
    hDDA->ddcIf->ddcNetIf.ddcNetChOpen((DDC_Handle *) hDDA->hDDC,
                                           (DDC_NetChInfo *) & hDDA->
                                           rxChInfo,
                                           (Ptr) & hDDA->macAddr[0]);
    if (retCode != CPMAC_SUCCESS) {
        errPrint("%s error: Error %08X from CPMAC DDC RX Inframode Channel Open(())\n", p_dev->name, retCode);
        return (-1);
    }
#else
    /* Else open normal Rx endpoint channel */
    retCode = hDDA->ddcIf->ddcNetIf.ddcNetChOpen((DDC_Handle *) hDDA->hDDC,
                                           (DDC_NetChInfo *) & hDDA->
                                           rxChInfo,
                                           (Ptr) & hDDA->macAddr[0]);
    if (retCode != CPMAC_SUCCESS) {
        errPrint("%s error: Error %08X from CPMAC DDC RX Channel Open()\n", p_dev->name, retCode);
        return (-1);
    }
#endif
#ifdef CONFIG_ARM_AVALANCHE_PPD
        /* Open Rx endpoint channel in embedded mode if Packet processor
         * is defined
         */
    hDDA->epRxChInfo.cppi4RxChInfo.chNum = CDMA_CH_EMAC0;
    hDDA->epRxChInfo.cppi4RxChInfo.dmaNum = PP_DMA_BLOCK_NUM;
    hDDA->epRxChInfo.cppi4RxChInfo.defDescType = CPPI41_DESC_TYPE_EMBEDDED;          /* Embedded */
    hDDA->epRxChInfo.cppi4RxChInfo.sopOffset = 0;
    hDDA->epRxChInfo.cppi4RxChInfo.rxCompQueue.qMgr = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
    hDDA->epRxChInfo.cppi4RxChInfo.rxCompQueue.qNum = PAL_CPPI41_SR_PPDSP_LOW_Q_NUM;
    hDDA->epRxChInfo.cppi4RxChInfo.u.embeddedPktCfg.fdQueue.qMgr = CPMAC_CPPI4x_FD_QMGR;
    hDDA->epRxChInfo.cppi4RxChInfo.u.embeddedPktCfg.fdQueue.qNum = CPMAC_CPPI4x_FD_QNUM(0);
    hDDA->epRxChInfo.cppi4RxChInfo.u.embeddedPktCfg.numBufSlot = (EMSLOTCNT-1);
    hDDA->epRxChInfo.cppi4RxChInfo.u.embeddedPktCfg.sopSlotNum = 1;

    hDDA->epRxChInfo.cppi4RxChInfo.u.embeddedPktCfg.fBufPool[0].bMgr = BUF_POOL_MGR0;
    hDDA->epRxChInfo.cppi4RxChInfo.u.embeddedPktCfg.fBufPool[0].bPool = CPMAC_CPPI4x_POOL_NUM(0);

    hDDA->epRxChInfo.cppi4RxChInfo.u.embeddedPktCfg.fBufPool[1].bMgr = BUF_POOL_MGR0;
    hDDA->epRxChInfo.cppi4RxChInfo.u.embeddedPktCfg.fBufPool[1].bPool = CPMAC_CPPI4x_POOL_NUM(0);
    hDDA->epRxChInfo.cppi4RxChInfo.u.embeddedPktCfg.fBufPool[2].bMgr = BUF_POOL_MGR0;
    hDDA->epRxChInfo.cppi4RxChInfo.u.embeddedPktCfg.fBufPool[2].bPool = CPMAC_CPPI4x_POOL_NUM(1);
    hDDA->epRxChInfo.cppi4RxChInfo.u.embeddedPktCfg.fBufPool[3].bMgr = BUF_POOL_MGR0;
    hDDA->epRxChInfo.cppi4RxChInfo.u.embeddedPktCfg.fBufPool[3].bPool = CPMAC_CPPI4x_POOL_NUM(0);
    hDDA->epRxChInfo.numBD = CPMAC_RX_EMBEDDED_BD_NUM;
    hDDA->epRxChInfo.descAlignment = 64;
    hDDA->epRxChInfo.chDir = DDC_NET_CH_DIR_RX;
    hDDA->epRxChInfo.dmaMode = CPPI41_DMA_MODE_ENDPOINT;
    retCode =
        hDDA->ddcIf->ddcNetIf.ddcNetChOpen((DDC_Handle *) hDDA->hDDC,
                                           (DDC_NetChInfo *) & hDDA->
                                           epRxChInfo, NULL);
    if (retCode != CPMAC_SUCCESS) {
        errPrint("%s error: Error %08X from CPMAC DDC RX Channel Open(())\n", p_dev->name, retCode);
        return (-1);
    }
#endif
    /* Open DDC instance */
    retCode = hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcOpen(hDDA->hDDC, NULL);
    if (retCode != CPMAC_SUCCESS) {
        errPrint("%s error: Error %08X from CPMAC DDC Open()\n", p_dev->name, retCode);
        return (-1);
    }

    /* the driver always works in the tasklet mode */
#ifdef TX_TASKLET_MODE
    tasklet_init(&hDDA->tx_tasklet, cpmac_handle_tx_tasklet,
                 (unsigned long) hDDA);
#endif
#ifdef RX_TASKLET_MODE
    tasklet_init (&hDDA->rx_tasklet, cpmac_handle_rx_tasklet, (unsigned long) hDDA);
#endif

    return (0);
}

/* Disable the device - teardown chanels and close DDC */
static int cpmac_p_dev_disable(CpmacNetDevice * hDDA)
{
    PAL_Result retCode;
    struct net_device *p_dev = hDDA->owner;

    /* inform the upper layers. */
    netif_stop_queue(hDDA->owner);

#ifdef CONFIG_ARM_AVALANCHE_PPD
#ifdef CONFIG_TI_PACKET_PROCESSOR
    ti_ppm_set_pid_flags (p_dev->pid_handle, TI_PP_PID_DISCARD_ALL_RX);
#else
    ti_ppd_set_pid_flags (&pid_eth[0], TI_PP_PID_DISCARD_ALL_RX);
#endif
    /* this delay is to make sure all the packets with the PID successfully egress throgh the respective ports.*/
    mdelay(DDA_CPMAC_SR_DELAY);
#endif

    /* Prepare to close */
    set_bit(0, &hDDA->setToClose);

    /* Closing the DDC instance will close all channels also */
    retCode = hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcClose(hDDA->hDDC, NULL);
    if (retCode != CPMAC_SUCCESS) {
        errPrint("%s error: Error %08X from CPMAC DDC Close()\n", p_dev->name, retCode);
        return (-1);
    } else {

#ifdef TX_TASKLET_MODE
        tasklet_kill(&hDDA->tx_tasklet);
#endif
#ifdef RX_TASKLET_MODE
        tasklet_kill (&hDDA->rx_tasklet);
#endif

        /* DDC should turn off the timer, but just in case */
        if (hDDA->timerActive != FALSE) {
            del_timer_sync(&hDDA->periodicTimer);
            hDDA->timerActive = FALSE;
        }

        dbgPrint("Device %s Closed.\n", p_dev->name);
        hDDA->deviceStats.start_tick = jiffies;
        hDDA->linkSpeed = 100000000;
        hDDA->linkMode = 1;
        netif_carrier_off(p_dev);
#if defined (CONFIG_ARM_AVALANCHE_COLORED_LED)
        led_manager_led_action(hDDA->ledHandle, CPMAC_SPEED_100);
        led_manager_led_action(hDDA->ledHandle, CPMAC_FULL_DPLX);
        led_manager_led_action(hDDA->ledHandle, CPMAC_LINK_OFF);
#endif


    }
    return (0);
}

/******************************************************************************
 *  Net Device functions
 *****************************************************************************/

/* Get statistics */
static struct net_device_stats *cpmac_dev_get_net_stats(struct net_device
                                                        *p_dev)
{
    CpmacNetDevice *hDDA = NETDEV_PRIV(p_dev);

    cpmac_p_update_statistics(p_dev, NULL, 0, NULL);

    return &hDDA->netDevStats;
}

/* Set multicast address in the driver */
static void cpmac_dev_mcast_set(struct net_device *p_dev)
{
    CpmacNetDevice *hDDA = NETDEV_PRIV(p_dev);

    if (p_dev->flags & IFF_PROMISC) {
        /* Enable promiscous mode */
        hDDA->initCfg.rxCfg.promiscousEnable = TRUE;
        hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC,
                                                      CPMAC_DDC_IOCTL_SET_RXCFG,
                                                      (Ptr) & hDDA->
                                                      initCfg.rxCfg, NULL);
    } else if ((p_dev->flags & IFF_ALLMULTI)
               || (p_dev->mc.count >
                   CPMAC_DDA_DEFAULT_MAX_MULTICAST_ADDRESSES)
    ) {
        /* Enable multicast - disable promiscous */
        hDDA->initCfg.rxCfg.promiscousEnable = FALSE;
        hDDA->initCfg.rxCfg.multicastEnable = TRUE;
        hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC,
                                                      CPMAC_DDC_IOCTL_SET_RXCFG,
                                                      (Ptr) & hDDA->
                                                      initCfg.rxCfg, NULL);

        /* Enable all multicast addresses */
        hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC,
                                                      CPMAC_DDC_IOCTL_ALL_MULTI,
                                                      (Ptr)
                                                      CPMAC_ALL_MULTI_SET,
                                                      NULL);
    } else if (p_dev->mc.count == 0) {
        /* Only unicast mode to be set - clear promiscous and clear multicast modes */
        hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC,
                                                      CPMAC_DDC_IOCTL_ALL_MULTI,
                                                      (Ptr)
                                                      CPMAC_ALL_MULTI_CLR,
                                                      NULL);
        /* Disable promiscous and multicast modes */
        hDDA->initCfg.rxCfg.promiscousEnable = FALSE;
        hDDA->initCfg.rxCfg.multicastEnable = FALSE;
        hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC,
                                                      CPMAC_DDC_IOCTL_SET_RXCFG,
                                                      (Ptr) & hDDA->
                                                      initCfg.rxCfg, NULL);
    } else if (p_dev->mc.count) {
        struct netdev_hw_addr  *mc_ptr;

        /* Clear multicast list first */
        hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC,
                                                      CPMAC_DDC_IOCTL_ALL_MULTI,
                                                      (Ptr)
                                                      CPMAC_ALL_MULTI_CLR,
                                                      NULL);

        /* Enable multicast - disable promiscous */
        hDDA->initCfg.rxCfg.promiscousEnable = FALSE;
        hDDA->initCfg.rxCfg.multicastEnable = TRUE;
        hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC,
                                                      CPMAC_DDC_IOCTL_SET_RXCFG,
                                                      (Ptr) & hDDA->
                                                      initCfg.rxCfg, NULL);

        /* Program multicast address list into CPMAC hardware using DDC */

        netdev_for_each_mc_addr(mc_ptr,(p_dev)){
            /* Add ethernet address to hardware list via DDC ioctl */
            hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC,CPMAC_DDC_IOCTL_MULTICAST_ADDR,
                                                          (Ptr)CPMAC_MULTICAST_ADD,
                                                          (Ptr)mc_ptr->addr);
        }
    } else {
        dbgPrint("%s:No Multicast address to set.\n", p_dev->name);
    }
}

static int cpmac_dev_set_mac_addr(struct net_device *p_dev, void *addr)
{
    int retCode;
    CpmacAddressParams AddressParams;
    struct sockaddr *sa = addr;
    CpmacNetDevice *hDDA = NETDEV_PRIV(p_dev);

    AddressParams.channel = CPMAC_DDA_DEFAULT_RX_CHANNEL;
    AddressParams.macAddress = sa->sa_data;

    {
        CpmacDDCStatus status;
        /* Use DDC Status IOCTL to see is DDC is open or not :) */
        retCode  = hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_GET_STATUS,
                                                            &status, NULL);
        if (retCode == CPMAC_ERR_DEV_NOT_OPEN) {
            /* just remember the change and exit. When the DDC opens next time, the
             * values are updated to the hardware. */
            goto  cpmac_set_mac_success;
        }
    }


    retCode = hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC,
                                                             CPMAC_DDC_IOCTL_SET_MAC_ADDRESS,
                                                            (CpmacAddressParams *) & AddressParams,
                                                            NULL);

    if (retCode != CPMAC_SUCCESS) {
        errPrint("%s error: Error %08X from CPMAC DDC TX Channel Open()\n", p_dev->name, retCode);
        goto cpmac_set_mac_err;
    }
    retCode =
        hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC,
                                                      CPMAC_DDC_IOCTL_SET_SRC_MAC_ADDRESS,
                                                      (CpmacAddressParams
                                                       *) & AddressParams,
                                                      NULL);

    if (retCode != CPMAC_SUCCESS) {
        errPrint("%s error: Error %08X from CPMAC DDC TX Channel Open()\n", p_dev->name, retCode);
        goto cpmac_set_mac_err;
    }

cpmac_set_mac_success:

    memcpy(hDDA->macAddr, sa->sa_data, p_dev->addr_len);
    memcpy(p_dev->dev_addr, sa->sa_data, p_dev->addr_len);
    return 0;

cpmac_set_mac_err:
    return -EIO;

}


static void cpmac_tx_timeout(struct net_device *p_dev)
{
    int retCode = CPMAC_SUCCESS;
    CpmacNetDevice *hDDA = NETDEV_PRIV(p_dev);


    hDDA->ddcIf->ddcNetIf.ddcNetChClose(hDDA->hDDC, hDDA->txChInfo.chNum,
                                        hDDA->txChInfo.chDir, 0);
    retCode =
        hDDA->ddcIf->ddcNetIf.ddcNetChOpen((DDC_Handle *) hDDA->hDDC,
                                           (DDC_NetChInfo *) & hDDA->
                                           txChInfo, NULL);
    if (retCode != CPMAC_SUCCESS) {
        errPrint("%s error: Error %08X from CPMAC DDC TX Channel Open()\n", p_dev->name, retCode);
    }

}

#ifndef MODULE





static int __init cpmac_set_macstring(char *str)
{
    cpmac_mac_string[str[0] - '0'] = &str[2];
    return 1;
}

static char* cpmac_get_macstring(int instance)
{
    return cpmac_mac_string[instance];
}

__setup("ethaddr", cpmac_set_macstring);

#endif

static int get_skb_hdr(struct sk_buff *skb, void **iphdr,
               void **tcph, u64 *hdr_flags, void *data)
{
/* Check that this is an ethernet packet */
    if(skb->protocol != ntohs(ETH_P_IP) )
    {
        return -1;
    }

    if( !(struct iphdr*)skb_network_header(skb) )
    {
        if( !skb_mac_header_was_set(skb))
        {
            return -1;
        }
        /* In case the pointers are not initialized */
        skb_set_network_header(skb,ETH_HLEN);
        skb_set_transport_header(skb,((struct iphdr*)skb_network_header(skb))->ihl*4);
    }

    /* Continue only if its TCP */
    if( ((struct iphdr*)skb_network_header(skb))->protocol != IPPROTO_TCP )
    {
        return -1;
    }

    if(((struct iphdr*)skb_network_header(skb))->version == 4)
        *hdr_flags = LRO_IPV4;

    *tcph = (void *)skb_transport_header(skb);
    *iphdr = (struct iphdr*)skb_network_header(skb);
    *hdr_flags |= LRO_TCP;

    return 0;
}


/***************************************************************
 *  cpmac_dev_init
 *
 *  Returns:
 *      0 on success, error code otherwise.
 *  Parms:
 *      dev The structure of the device to be
 *          init'ed.
 *
 *  This function completes the initialization of the
 *  device structure and driver.  It reserves the IO
 *  addresses and assignes the device's methods.
 *
 **************************************************************/
static int cpmac_dev_init(struct net_device *p_dev)
{
    int cnt, ddcInitStatus = 0;
    PAL_Result retCode;
    char *mac_string = NULL;
    char default_mac_string[20];
    CpmacNetDevice *hDDA = NETDEV_PRIV(p_dev);
    int instanceNum = hDDA->instanceNum;
    extern int eth_reset_gpio_num;

    sprintf(default_mac_string, "08.00.28.32.06.0%d", instanceNum + 2);

#ifndef MODULE
    mac_string = cpmac_get_macstring(instanceNum);
#endif

    /* Obtain mac address and set in device structure */
    if (!mac_string) {
        mac_string = default_mac_string;
#ifndef MODULE
        printk("Cpmac: Error getting mac from Boot enviroment for %s\n", p_dev->name);
        printk("Cpmac: Using default mac address: %s\n", mac_string);
        printk("Pass kernel parameter ethaddr%d=xx.xx.xx.xx.xx.xx\n", instanceNum);
        printk("to set mac address\n");
#endif
    }

    /* Initialize the reset GPIO pin number */
    eth_rst_gpio = eth_reset_gpio_num;

    cpmac_str_to_ethaddr(hDDA->macAddr, mac_string);

    /* Clear device address */
    memset( p_dev->dev_addr, 0, MAX_ADDR_LEN );

    for (cnt = 0; cnt < ETH_ALEN; cnt++) {
        p_dev->dev_addr[cnt] = hDDA->macAddr[cnt];      /* This sets the hardware address */
    }

    hDDA->setToClose = 1;

#if defined (CONFIG_ARM_AVALANCHE_COLORED_LED)
    if ((hDDA->ledHandle = led_manager_register_module("cpmac", instanceNum)) == NULL) {
        errPrint("Cpmac: Could not allocate handle for CPMAC[%d] LED.\n", instanceNum);
        goto cpmac_dev_init_exit;
    }
#endif


    /* Get configuration information for this instance */
    /* When Config service is available, use it */
    if (cpmac_net_get_config(hDDA) != 0) {
        errPrint("Cpmac: Could not fetch configuration information for instance %d\n", instanceNum);
        goto cpmac_dev_init_exit;
    }

    /* Create DDC Instance */
    retCode = DDC_cpmacCreateInstance(instanceNum,      /* Instance Id */
                                      hDDA,     /* DDA Handle */
                                      &cpmac_DDA_cb_interface,  /* Pointer to DDA Callback function table */
                                      (DDC_Handle **) & hDDA->hDDC,     /* Address of pointer to DDC Handle */
                                      &hDDA->ddcIf,     /* Address of pointer to DDC function table */
                                      NULL);    /* Param not used */
    if (retCode != CPMAC_SUCCESS) {
        errPrint("Cpmac: cpmac_dev_init:%d: Error %08X from DDC_cpmacCreateInstance()\n", instanceNum, retCode);
        goto cpmac_dev_init_exit;
    }

    ddcInitStatus = 1;          /* Instance created */

    /* Initialize DDC Instance by passing initial configuration structure */
    retCode =
        hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcInit(hDDA->hDDC,
                                                   &hDDA->initCfg);
    if (retCode != CPMAC_SUCCESS) {
        errPrint("Cpmac: cpmac_dev_init:%d: Error %08X from DDC Init()\n",
                 instanceNum, retCode);
        goto cpmac_dev_init_exit;
    }

    ddcInitStatus = 2;          /* Instance initialized */

    /* Set as per RFC 2665 */
    hDDA->linkSpeed = 100000000;
    hDDA->linkMode = 1;

#if defined (CONFIG_ARM_AVALANCHE_COLORED_LED)
    led_manager_led_action(hDDA->ledHandle, CPMAC_SPEED_100);
    led_manager_led_action(hDDA->ledHandle, CPMAC_FULL_DPLX);
#endif


    /* initialize the timers for the net device - the timer will be started by DDC calling the Ioctl on DDA */
    init_timer(&hDDA->periodicTimer);
    hDDA->periodicTicks = 0;
    hDDA->periodicTimer.expires = 0;
    hDDA->timerActive = FALSE;
    hDDA->periodicTimer.data = (unsigned long) hDDA;
    hDDA->periodicTimer.function =
        (timer_tick_func) cpmac_p_tick_timer_expiry;

    /* This is not required as the mib is not supported in current DDC code needs to add that */
    /* Added in the DDC layer also */
    init_timer(&hDDA->mibTimer);
    hDDA->mibTimerActive = FALSE;
    hDDA->mibTimer.data = (unsigned long) hDDA;
    hDDA->mibTimer.function = (timer_tick_func) cpmac_p_mib_timer_expiry;

    /*Not required in new DDC design */
    hDDA->Clear_EOI = 0;

    /* Populate the device structure */
    p_dev->addr_len = 6;


#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
    netif_napi_add(p_dev, &hDDA->napi, cpmac_poll, hDDA->napiRxTx.rxMaxService);
#endif

    /* Reset the broadcast and multicast flags and enable them based upon configuration of driver */
    p_dev->flags &= ~(IFF_PROMISC | IFF_BROADCAST | IFF_MULTICAST);

    if (hDDA->initCfg.rxCfg.broadcastEnable == TRUE)
        p_dev->flags |= IFF_BROADCAST;
    if (hDDA->initCfg.rxCfg.multicastEnable == TRUE)
        p_dev->flags |= IFF_MULTICAST;

    netif_carrier_off(p_dev);

#if defined (CONFIG_ARM_AVALANCHE_COLORED_LED)
    led_manager_led_action(hDDA->ledHandle, CPMAC_LINK_OFF);
#endif


    /* Request memory region from the kernel */
    p_dev->base_addr = hDDA->initCfg.baseAddress;
    request_mem_region(p_dev->base_addr, CPMAC_DDA_DEFAULT_CPMAC_SIZE,
                       p_dev->name);

    /* If following flag ON then Open DDC */
    if (g_init_enable_flag) {
        if (cpmac_p_dev_enable(hDDA)) {
            errPrint("%s error: cpmac_dev_init: device could not OPEN DDC\n", p_dev->name);
            goto cpmac_dev_init_exit;
        }

    }

    /* LRO Setup */
    hDDA->lro_mgr.dev = p_dev;
    memset(&hDDA->lro_mgr.stats, 0, sizeof(hDDA->lro_mgr.stats));
    hDDA->lro_mgr.features = LRO_F_NAPI;
    hDDA->lro_mgr.ip_summed = CHECKSUM_UNNECESSARY;
    hDDA->lro_mgr.ip_summed_aggr = CHECKSUM_UNNECESSARY; //CHECKSUM_NONE;
    hDDA->lro_mgr.max_desc = ARRAY_SIZE(hDDA->lro_arr);
    hDDA->lro_mgr.max_aggr = 32;
    hDDA->lro_mgr.frag_align_pad = 0;
    hDDA->lro_mgr.lro_arr = hDDA->lro_arr;
    hDDA->lro_mgr.get_skb_header = get_skb_hdr;
    memset(&hDDA->lro_arr, 0, sizeof(hDDA->lro_arr));

    /* Disable LRO by default */
    p_dev->features &= ~NETIF_F_LRO;

    return (0);

  cpmac_dev_init_exit:
    /* All resources allocated are freed - call the un-init sequence on DDC */
    switch (ddcInitStatus) {
    case 2:                    /* Deinit DDC */
        retCode = hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcDeinit(hDDA->hDDC, NULL);
        if (retCode != CPMAC_SUCCESS)
            errPrint("Cpmac: cpmac_dev_init_exit:%s: Error %08X from DDC Deinit()\n", p_dev->name, retCode);
        /* Follow through to delete the instance also */
    case 1:                    /* Delete DDC Instance */
        retCode = hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcDelInst(hDDA->hDDC, NULL);
        if (retCode != CPMAC_SUCCESS)
            errPrint("Cpmac: cpmac_dev_init_exit:%s: Error %08X from DDC Delete Instance()\n", p_dev->name, retCode);
        break;
    default:
        break;
    }

    return (-1);
}                               /* cpmac_dev_init */


/******************************************************************************
 *  Device Open/Close functions
 *****************************************************************************/

/* Open the Adapter */

int cpmac_dev_open(struct net_device *p_dev)
{
    CpmacNetDevice *hDDA = NETDEV_PRIV(p_dev);

#ifdef CONFIG_ARM_AVALANCHE_PPD
    TI_PP_PID_RANGE  pid_range_eth;
    /* Install Ethernet PID */
    pid_range_eth.type        = TI_PP_PID_TYPE_ETHERNET;
    pid_range_eth.port_num    = CPPI41_SRCPORT_CPMAC0;
    pid_range_eth.count       = PP_ETH_PID_COUNT;
    pid_range_eth.base_index  = PP_ETH_PID_BASE;

#ifdef CONFIG_TI_PACKET_PROCESSOR
    if (ti_ppm_config_pid_range (&pid_range_eth))
#else
    if (ti_ppd_config_pid_range (&pid_range_eth))
#endif
    {
        printk ("%s: config_pid_range failed with error code %d.\n",
                    __FUNCTION__);
        return -1;
    }

#ifdef CONFIG_TI_PACKET_PROCESSOR
        p_dev->pid_handle   = cpgmac_add_pid (0);
        p_dev->vpid_block.type               = TI_PP_ETHERNET;
        p_dev->vpid_block.parent_pid_handle  = p_dev->pid_handle;
        p_dev->vpid_block.egress_mtu         = 0;
        p_dev->vpid_block.priv_tx_data_len   = 0;
        p_dev->qos_setup_hook                = cpgmac_setup_qos;
        p_dev->qos_shutdown_hook             = cpgmac_shutdown_qos;
        p_dev->qos_select_hook               = cpgmac_select_qos;
#else /* !CONFIG_TI_PACKET_PROCESSOR */
        cpgmac_add_pid (0);
#endif
#endif
    /* clear the setToClose bit. We are getting opened */
    clear_bit(0, &hDDA->setToClose);

    if (!g_init_enable_flag) {
        if (cpmac_p_dev_enable(hDDA)) {
            errPrint("%s error: cpmac_dev_open: device could not OPEN DDC\n", p_dev->name);
            return (-1);
        }
    }
#ifdef CONFIG_ARM_EXTERNAL_SWITCH
    if (ext_switch_reset_gpio != -1)
    {
        /* Note: Initialization of the switch requires MDIO to be initialized.
         * MDIO is initialized with in the cpmac_p_dev_enable( ), but cpmac_p_dev_enable()
         * is being called ahead of normal and standard sequence at the behest of NSP.
         * In cases where g_init_enable_flag is false, the following initialization shall
         * fail. So, we are warning developers.....
         */

        PHY_DEVICE *phy_info =
        (PHY_DEVICE *) DDC_cpmacGetPhyDev(hDDA->hDDC);

        switch_init(phy_info);

        /* create proc entries for the matvell ports */
        switch_create_proc_entry();

#ifdef CONFIG_ARM_AVALANCHE_MARVELL_6063
        /* NSP need the ability to add the port Id to the packet itself */
        switch_trailerSet(TRUE);
#endif
    }
#endif


#ifndef CPMAC_POLL_MODE
    napi_enable(&hDDA->napi);
    if (request_irq(LNXINTNUM(hDDA->initCfg.TxIntrLine), cpmac_hal_tx_isr, SA_INTERRUPT, "Cpmac Driver", hDDA)) {
        errPrint("Failed to register the irq %d for Cpmac %s.\n", hDDA->initCfg.TxIntrLine, p_dev->name);
        return (-1);
    }

    if (request_irq(LNXINTNUM(hDDA->initCfg.RxIntrLine), cpmac_hal_rx_isr, SA_INTERRUPT, "Cpmac Driver", hDDA)) {
        errPrint("Failed to register the irq %d for Cpmac %s.\n", hDDA->initCfg.RxIntrLine, p_dev->name);
        return (-1);
    }
#endif

    if (netif_carrier_ok(p_dev))
        netif_start_queue(p_dev);
    else
        netif_stop_queue(p_dev);

#ifdef CPMAC_POLL_MODE
    dbgPrint("Cpmac: cpmac_dev_open: Starting txpoll and rxpoll timers...\n");
    hDDA->cpmac_txpoll_timer.expires    = jiffies + HZ*10;
    add_timer (&hDDA->cpmac_txpoll_timer);
    hDDA->cpmac_rxpoll_timer.expires    = jiffies + HZ;
    add_timer (&hDDA->cpmac_rxpoll_timer);
#endif

    hDDA->deviceStats.start_tick = jiffies;
    dbgPrint("Started the network queue for %s.\n", p_dev->name);
    return (0);
}

static int cpmac_dev_close(struct net_device *p_dev)
{
    CpmacNetDevice *hDDA = NETDEV_PRIV(p_dev);

#ifdef CPMAC_POLL_MODE
    dbgPrint("Cpmac: cpmac_dev_close: Stopping txpoll and rxpoll timers...\n");
    del_timer_sync (&hDDA->cpmac_txpoll_timer);
    del_timer_sync (&hDDA->cpmac_rxpoll_timer);
#endif

    /* inform the upper layers. */
    netif_stop_queue(hDDA->owner);

    if (!g_init_enable_flag)
        cpmac_p_dev_disable(netdev_priv(p_dev));

#ifndef CPMAC_POLL_MODE
    napi_disable(&hDDA->napi);
    free_irq(LNXINTNUM(hDDA->initCfg.TxIntrLine), hDDA);
    free_irq(LNXINTNUM(hDDA->initCfg.RxIntrLine), hDDA);
#endif

#if 0
    if (hDDA->Clear_EOI) {
        hDDA->ddcIf->pktProcessEnd(hDDA->hDDC, NULL);
        hDDA->Clear_EOI = 0;
    }
#endif

    /* Prepare to close */
    set_bit(0, &hDDA->setToClose);
    netif_carrier_off(p_dev);


#ifdef CONFIG_ARM_AVALANCHE_PPD
    {
        int rc;

    #ifdef CONFIG_TI_PACKET_PROCESSOR
        if (rc = ti_ppm_delete_pid(p_dev->pid_handle))
    #else
        if (rc = ti_ppd_delete_pid(p_dev->pid_handle))
    #endif
        {
            printk ("%s: remove_pid failed with error code %d.\n",__FUNCTION__, rc);
            return -1;
        }

        p_dev->pid_handle = -1;
        p_dev->vpid_handle = -1;
        memset(&p_dev->vpid_block, 0, sizeof(p_dev->vpid_block));
        p_dev->vpid_block.parent_pid_handle = -1;

    #ifdef CONFIG_TI_PACKET_PROCESSOR
        if (rc = ti_ppm_remove_pid_range (CPPI41_SRCPORT_CPMAC0))
    #else
        if (rc = ti_ppd_remove_pid_range (CPPI41_SRCPORT_CPMAC0))
    #endif
        {
            printk ("%s: remove_pid_range failed with error code %d.\n",__FUNCTION__, rc);
            return -1;
        }
    }
#endif

    return (0);
}

/* Update MIB2 IF counters with DDA maintained stats */
static void cpmac_ifcnt_update(CpmacNetDevice *hDDA, struct mib2_ifCounters* mib2ifCounters)
{
    mib2ifCounters->inUnknownProtPkts += hDDA->unknownProtPkts;
}


/* Ioctl function */
static int cpmac_ioctl(struct net_device *p_dev, struct ifreq *rq, int cmd)
{
    CpmacDrvPrivIoctl privIoctl;
    CpmacNetDevice *hDDA = NETDEV_PRIV(p_dev);

    if (cmd == SIOCDEVPRIVATE) {
        /* Copy user data */
        if (copy_from_user
            ((char *) &privIoctl, (char *) rq->ifr_data,
             sizeof(CpmacDrvPrivIoctl)))
            return -EFAULT;

        switch (privIoctl.cmd) {
            /* Program Type 2/3 Address Filter */
        case CPMAC_DDA_PRIV_FILTERING:
            {
                CpmacType2_3_AddrFilterParams filterParams;
                if (copy_from_user
                    ((char *) &filterParams, (char *) privIoctl.data,
                     sizeof(CpmacType2_3_AddrFilterParams)))
                    return -EFAULT;

                if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.
                    ddcControl(hDDA->hDDC,
                               CPMAC_DDC_IOCTL_TYPE2_3_FILTERING,
                               (CpmacType2_3_AddrFilterParams *) &
                               filterParams, NULL) != CPMAC_SUCCESS) {
                    errPrint
                        ("Failed to read params (CPMAC_DDA_PRIV_FILTERING) from DDC for  %s.\n",
                         p_dev->name);
                    return -EFAULT;
                }
                break;
            }

            /* Read PHY register via MII interface */
        case CPMAC_DDA_PRIV_MII_READ:
            {
                CpmacPhyParams phyParams;
                spinlock_t mii_lock;
                unsigned long flag;

                spin_lock_init(&mii_lock);


                /* Copy user data into local variable */
                if (copy_from_user ((char *) &phyParams, (char *) privIoctl.data, sizeof(CpmacPhyParams)))
                    return -EFAULT;

                /* Make sure this function does not clash with mii access during tick function */
                spin_lock_irqsave(&mii_lock, flag);

                if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.  ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_READ_PHY_REG,
                               (Ptr) & phyParams, NULL) != CPMAC_SUCCESS) {
                    errPrint ("Failed to read params (CPMAC_DDA_PRIV_MII_READ) from DDC for  %s.\n", p_dev->name);
                    spin_unlock_irqrestore(&mii_lock, flag);
                    return -EFAULT;
                }

                /* Copy the local data to user space */
                if (copy_to_user ((char *) privIoctl.data, (char *) &phyParams, sizeof(CpmacPhyParams))) {
                    spin_unlock_irqrestore(&mii_lock, flag);
                    return -EFAULT;
                }

                /* Enable tick timer to access phy now if required */
                spin_unlock_irqrestore(&mii_lock, flag);
            }
            break;
            /* Write PHY register via MII interface */
        case CPMAC_DDA_PRIV_MII_WRITE:
            {
                CpmacPhyParams phyParams;
                spinlock_t mii_lock;
                unsigned long flag;

                spin_lock_init(&mii_lock);

                /* Copy user data into local variable */
                if (copy_from_user
                    ((char *) &phyParams, (char *) privIoctl.data,
                     sizeof(CpmacPhyParams)))
                    return -EFAULT;

                /* Make sure this function does not clash with mii access during tick function */
                spin_lock_irqsave(&mii_lock, flag);

                if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_WRITE_PHY_REG,
                               (Ptr) & phyParams, NULL) != CPMAC_SUCCESS) {
                    errPrint ("Failed to read params (CPMAC_DDA_PRIV_MII_READ) from DDC for  %s.\n", p_dev->name);
                    spin_unlock_irqrestore(&mii_lock, flag);
                    return -EFAULT;
                }

                /* Enable tick timer to access phy now if required */
                spin_unlock_irqrestore(&mii_lock, flag);
            }
            break;

            /* Get Statistics */
        case CPMAC_DDA_PRIV_GET_STATS:
            {
                CpmacHwStatistics stats;

                /* Caller provides memory for CpmacHwStatistics structure via "data" pointer */
                if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.
                    ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_GET_STATISTICS,
                               (Ptr) & stats, NULL) != CPMAC_SUCCESS) {
                    errPrint("Failed to get statistics (CPMAC_DDA_PRIV_GET_STATS) from DDC for  %s.\n", p_dev->name);
                    return (CPMAC_DDA_INTERNAL_FAILURE);
                }

                /* Copy the local data to user space */
                if (copy_to_user
                    ((char *) privIoctl.data, (char *) &stats,
                     sizeof(CpmacHwStatistics)))
                    return -EFAULT;

                break;
            }

            /* Clear Statistics */
        case CPMAC_DDA_PRIV_CLR_STATS:
            {
                if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.
                    ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_CLR_STATISTICS,
                               NULL, NULL) != CPMAC_SUCCESS) {
                    errPrint("Failed to clear statistics (CPMAC_DDA_PRIV_CLR_STATS) in DDC for  %s.\n", p_dev->name);
                    return (CPMAC_DDA_INTERNAL_FAILURE);
                }
                break;
            }

#ifdef  CONFIG_ARM_EXTERNAL_SWITCH
        case CPMAC_DDA_EXTERNAL_SWITCH:
            {
                SwitchIoctlType switchParams;

                /* Copy user data into local variable */
                if (copy_from_user
                    ((char *) &switchParams, (char *) privIoctl.data,
                     sizeof(SwitchIoctlType)))
                    return -EFAULT;

                if (external_switch_ioctl(&switchParams) == CPMAC_SUCCESS) {
                    copy_to_user((char *) privIoctl.data,
                                 (char *) &switchParams,
                                 sizeof(SwitchIoctlType));


                } else {
                    errPrint("Failed to  read params (CPMAC_DDA_EXTERNAL_SWITCH) in DDC for  %s.\n", p_dev->name);
                    return (CPMAC_DDA_INTERNAL_FAILURE);
                }

                break;
            }
#endif

        case CPMAC_DDA_ADD_RX_BD:
            {
                unsigned int numOfRxBd = (unsigned int) privIoctl.data;

                if (numOfRxBd <= hDDA->rxChInfo.numBD) {
                    errPrint("%s CPMAC_DDA_ADD_RX_BD Capable only to add RX buffer\n", __FUNCTION__);
                    return -EFAULT;
                }

                /* Calculate the number of buffer to add */
                numOfRxBd = numOfRxBd - hDDA->rxChInfo.numBD;

                if (hDDA->ddcIf->
                    AddRxBd((CpmacDDCObj *) hDDA->hDDC,
                            (CpmacChInfo *) & hDDA->rxChInfo,
                            numOfRxBd) != CPMAC_SUCCESS) {
                    errPrint("%s CPMAC_DDA_ADD_RX_BD Failed to add RX buffer\n", __FUNCTION__);
                    return -EFAULT;
                }

                break;
            }
        default:
            return -EFAULT;
            break;
        }                       /* End of switch for CpmacDrvPrivIoctl command */
    } else if (cmd == SIOTIMIB2) {
        TI_SNMP_CMD_T ti_snmp_cmd;

        /* Now copy the user data */
        if (copy_from_user
            ((char *) &ti_snmp_cmd, (char *) rq->ifr_data,
             sizeof(TI_SNMP_CMD_T)))
            return -EFAULT;

        switch (ti_snmp_cmd.cmd) {
        case TI_SIOCGINTFCOUNTERS:
            {
                struct mib2_ifCounters mib_counter;

                /* Caller provides memory for CpmacHwStatistics structure via "data" pointer */
                if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.
                    ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_IF_COUNTERS,
                               (Ptr) & mib_counter, NULL) != CPMAC_SUCCESS)
                {
                    errPrint("Failed to get statistics (TI_SIOCGINTFCOUNTERS) from DDC for  %s.\n", p_dev->name);
                    return (CPMAC_DDA_INTERNAL_FAILURE);
                }

                cpmac_ifcnt_update(hDDA, &mib_counter);

                /* Copy the local data to user space */
                if (copy_to_user
                    ((char *) ti_snmp_cmd.data, (char *) &mib_counter,
                     sizeof(struct mib2_ifCounters)))
                    return -EFAULT;

                break;
            }

        case TI_SIOCGINTFPARAMS:
            {
                struct mib2_ifParams localParams;

                localParams.ifSpeed = hDDA->linkSpeed;
                localParams.ifHighSpeed = (localParams.ifSpeed) / 1000000;
                localParams.ifOperStatus =
                    ((p_dev->
                      flags & IFF_UP) ? MIB2_STATUS_UP : MIB2_STATUS_DOWN);
                localParams.ifPromiscuousMode =
                    ((p_dev->flags & IFF_PROMISC) ? TRUE : FALSE);

                /* Now copy the counters to the user data */
                if (copy_to_user
                    ((char *) ti_snmp_cmd.data, (char *) &localParams,
                     sizeof(struct mib2_ifParams)))
                    return -EFAULT;
            }
            break;

        case TI_SIOCGETHERCOUNTERS:
            {
                struct mib2_phyCounters phy_counter;

                /* Caller provides memory for CpmacHwStatistics structure via "data" pointer */
                if (hDDA->ddcIf->ddcNetIf.ddcFuncTable.
                    ddcControl(hDDA->hDDC, CPMAC_DDC_IOCTL_ETHER_COUNTERS,
                               (Ptr) & phy_counter, NULL) != CPMAC_SUCCESS)
                {
                    errPrint("Failed to get statistics (TI_SIOCGETHERCOUNTERS) from DDC for  %s.\n", p_dev->name);
                    return (CPMAC_DDA_INTERNAL_FAILURE);
                }

                /* Copy the local data to user space */
                if (copy_to_user
                    ((char *) ti_snmp_cmd.data, (char *) &phy_counter,
                     sizeof(struct mib2_phyCounters)))
                    return -EFAULT;

                break;
            }

        case TI_SIOCGETHERPARAMS:
            {
                struct mib2_ethParams localParams;

                localParams.ethDuplexStatus =
                    ((hDDA->linkMode ==
                      2) ? MIB2_FULL_DUPLEX : MIB2_HALF_DUPLEX);

                /* Now copy the counters to the user data */
                if (copy_to_user
                    ((char *) ti_snmp_cmd.data, (char *) &localParams,
                     sizeof(struct mib2_ethParams)))
                    return -EFAULT;

                break;
            }

        default:
            return -EFAULT;

        }


    } else {
        return -EFAULT;
    }

    return (0);
}

/******************************************************************************
 *  Linux Module Init/Exit
 *****************************************************************************/


static struct platform_device *cpmac_dev;
static ssize_t cpmac_show_version(struct device_driver *drv, char *buf)
{
    /* Hack the proc function */
    return cpmac_p_get_version(buf, NULL, 0, 4096, NULL, NULL);
}

static DRIVER_ATTR(version, S_IRUGO, cpmac_show_version, NULL);


#ifdef CONFIG_ARM_AVALANCHE_PPD
static int cpgmac_select_qos(struct sk_buff *skb)
{
    skb->pp_packet_info.ti_session.cluster = 0;
    skb->pp_packet_info.ti_session.priority = 3 - (skb->pp_packet_info.ti_session.priority >> 1);
    return 0;
}

static TI_PP_QOS_CLST_CFG  cpgmac_qos_cluster_db;


static int cpgmac_setup_qos (struct net_device *dev)
{
    int rc;
    TI_PP_QOS_QUEUE     *qcfg;
    CpmacNetDevice      *hDDA = NETDEV_PRIV(dev);

    //
    // Cluster 0
    //
    // Setup 4 QOS queues, one that gets line speed and then trickles down to the other.
    //
    cpgmac_qos_cluster_db.qos_q_cnt  = 4;

    // Queue 0
    qcfg = &cpgmac_qos_cluster_db.qos_q_cfg[0];
    qcfg->q_num         = CPMAC_CPPI4x_QoS_HIGH_TX_QNUM - PAL_CPPI41_QPDSP_FW_Q_BASE;
    qcfg->flags         = 0;
    qcfg->egr_q         = CPMAC_CPPI4x_TX_QNUM(1);
    qcfg->it_credit     = hDDA->linkSpeed/40000/8; /* <link speed> / <PP ticks per sec> / <8 bits in byte> */
    qcfg->max_credit    = MAX_IP_PACKET_SIZE * 2;
    qcfg->congst_thrsh  = MAX_IP_PACKET_SIZE * 32;
    qcfg->congst_thrsh_packets = 0;

    // Queue 1
    qcfg = &cpgmac_qos_cluster_db.qos_q_cfg[1];
    qcfg->q_num         = CPMAC_CPPI4x_QoS_HIGH_MED_TX_QNUM - PAL_CPPI41_QPDSP_FW_Q_BASE;
    qcfg->flags         = 0;
    qcfg->egr_q         = CPMAC_CPPI4x_TX_QNUM(1);;
    qcfg->it_credit     = 0;
    qcfg->max_credit    = MAX_IP_PACKET_SIZE * 2;
    qcfg->congst_thrsh  = MAX_IP_PACKET_SIZE * 32;
    qcfg->congst_thrsh_packets = 0;

    // Queue 2
    qcfg = &cpgmac_qos_cluster_db.qos_q_cfg[2];
    qcfg->q_num         = CPMAC_CPPI4x_QoS_LOW_MED_TX_QNUM - PAL_CPPI41_QPDSP_FW_Q_BASE;
    qcfg->flags         = 0;
    qcfg->egr_q         = CPMAC_CPPI4x_TX_QNUM(1);;
    qcfg->it_credit     = 0;
    qcfg->max_credit    = MAX_IP_PACKET_SIZE * 2;
    qcfg->congst_thrsh  = MAX_IP_PACKET_SIZE * 64;
    qcfg->congst_thrsh_packets = 0;

    // Queue 3
    qcfg = &cpgmac_qos_cluster_db.qos_q_cfg[3];
    qcfg->q_num         = CPMAC_CPPI4x_QoS_LOW_TX_QNUM - PAL_CPPI41_QPDSP_FW_Q_BASE;
    qcfg->flags         = 0;
    qcfg->egr_q         = CPMAC_CPPI4x_TX_QNUM(1);;
    qcfg->it_credit     = 0;
    qcfg->max_credit    = MAX_IP_PACKET_SIZE * 2;
    qcfg->congst_thrsh  = MAX_IP_PACKET_SIZE * 300;
    qcfg->congst_thrsh_packets = 0;


    // Cluster 0
    cpgmac_qos_cluster_db.global_credit      = 0;
    cpgmac_qos_cluster_db.max_global_credit  = MAX_IP_PACKET_SIZE * 2;
    cpgmac_qos_cluster_db.egr_congst_thrsh1  = MAX_IP_PACKET_SIZE * 2;
    cpgmac_qos_cluster_db.egr_congst_thrsh2  = MAX_IP_PACKET_SIZE * 28;
    cpgmac_qos_cluster_db.egr_congst_thrsh3  = MAX_IP_PACKET_SIZE * 32;
    cpgmac_qos_cluster_db.egr_congst_thrsh4  = MAX_IP_PACKET_SIZE * 48;
    cpgmac_qos_cluster_db.egr_congst_thrsh_packets = 0;

    rc = ti_ppm_qos_cluster_disable (CPMAC_CPPI4x_QoS_CLUSTER_IDX);
    rc = ti_ppm_qos_cluster_setup   (CPMAC_CPPI4x_QoS_CLUSTER_IDX, &cpgmac_qos_cluster_db);
    rc = ti_ppm_qos_cluster_enable  (CPMAC_CPPI4x_QoS_CLUSTER_IDX);

    dev->vpid_block.qos_cluster[0]  = &cpgmac_qos_cluster_db;
    dev->vpid_block.qos_clusters_count = 1;

    return rc;
}


static int cpgmac_shutdown_qos (struct net_device *dev)
{
    int rc;

    rc = ti_ppm_qos_cluster_disable (CPMAC_CPPI4x_QoS_CLUSTER_IDX);

    dev->vpid_block.qos_clusters_count = 0;

    return rc;
}


static int cpgmac_add_pid (int indx)
{
#ifdef CONFIG_TI_PACKET_PROCESSOR
    TI_PP_PID        pid_eth[PP_ETH_PID_COUNT];
#endif
    int ret_val;

    /* Add Ethernet PIDs */

    pid_eth[indx].type            = TI_PP_PID_TYPE_ETHERNET;
    pid_eth[indx].ingress_framing   = TI_PP_PID_INGRESS_ETHERNET
                                    | TI_PP_PID_INGRESS_PPPOE
                                    | TI_PP_PID_INGRESS_IPV6
                                    | TI_PP_PID_INGRESS_IPV4
                                    | TI_PP_PID_INGRESS_IPOE;
    pid_eth[indx].pri_mapping     = 1;    /* Num prio Qs for fwd */
    pid_eth[indx].dflt_pri_drp    = 0;
    pid_eth[indx].dflt_dst_tag    = 0x3FFF;
    pid_eth[indx].dflt_fwd_q      = CPMAC_CPPI4x_ETH_TO_HOST_PRXY_QNUM(0);/* Fwd to inf0 by default */
    pid_eth[indx].tx_pri_q_map[0] = CPMAC_CPPI4x_TX_QNUM(1);  /* Low  priority Q used for egress rec */
    pid_eth[indx].tx_hw_data_len  = 0;

    pid_eth[indx].pid_handle      = PP_ETH_PID_BASE+indx;

#ifdef CONFIG_TI_PACKET_PROCESSOR
    if ((ret_val = ti_ppm_create_pid (&pid_eth[indx])) < 0)
#else
    if ((ret_val = ti_ppd_create_pid (&pid_eth[indx])))
#endif
    {
        errPrint ("%s: create_pid failed with error code %d.\n",
                    __FUNCTION__, ret_val);
        pid_eth[indx].pid_handle = -1;
    }

    return pid_eth[indx].pid_handle;
}
#endif

static const struct net_device_ops cpmac_netdev_ops = {
        .ndo_init               = cpmac_dev_init,
        .ndo_open               = cpmac_dev_open,
        .ndo_do_ioctl           = cpmac_ioctl,
        .ndo_start_xmit         = cpmac_dev_tx,
        .ndo_stop               = cpmac_dev_close,
        .ndo_get_stats          = cpmac_dev_get_net_stats,
        .ndo_set_multicast_list = cpmac_dev_mcast_set,
        .ndo_tx_timeout         = cpmac_tx_timeout,
       . ndo_set_mac_address    = cpmac_dev_set_mac_addr,
};

/* Probe number of CPMAC instances and register net_device structure */
static int __init cpmac_dev_probe(void)
{
    int retVal = 0;
    int unit;
    int instance_count = CONFIG_ARM_CPMAC_PORTS;


    printk("Loading cpgmac driver for puma5 \n");
    if (cpmac_cfg_probe()) {
        printk("Failed to probe for CPMAC configuration, Exiting.\n");
        return (-1);
    }

#if 0
    /*FIXME This is not there in base port.
     */
         PAL_sysProbeAndPrep(AVALANCHE_CPMAC_HW_MODULE_REV,
                             AVALANCHE_HIGH_CPMAC_BASE, 0)) != 0) {
        printk("CPMAC1 support not available\n");
        instance_count = 1;
    }
#endif

    cpmac_dev = platform_device_register_simple("cpmac", -1, NULL, 0);

    if (IS_ERR(cpmac_dev)) {
        return -1;
    }

    if (driver_register(&cpmac_driver)) {
        platform_device_unregister(cpmac_dev);
        return -1;
    }

    retVal = driver_create_file(&cpmac_driver, &driver_attr_version);
    if (retVal) {
        printk("Cpmac: Could not create sysfs file\n");
    }
/* CONFIG_ARM_AVALANCHE_PPD */

    for (unit = 0; unit < instance_count; unit++) {
        struct net_device *p_dev;
        CpmacNetDevice *hDDA;
        int failed;

        if (!
            (p_dev =
             alloc_netdev(sizeof(CpmacNetDevice), "eth%d", ether_setup))) {
            printk("CPMAC: Etherdev alloc failed for device inst %d.\n", unit);
            retVal = -ENOMEM;
            break;
        }

        hDDA = NETDEV_PRIV(p_dev);
        hDDA->owner = p_dev;

        hDDA->instanceNum = unit;

        p_dev->netdev_ops = &cpmac_netdev_ops; /* Set initialization functions */

 /* CONFIG_ARM_AVALANCHE_PPD */


        SET_NETDEV_DEV(p_dev, &(cpmac_dev->dev));
        platform_set_drvdata(cpmac_dev, p_dev);

        cpmac_net_dev[hDDA->instanceNum] = p_dev;

#if defined CONFIG_ARM_CPMAC_INIT_BUF_MALLOC
        g_init_enable_flag = 1;
#endif


        cpmac_p_detect_manual_cfg(cfg_link_speed, cfg_link_mode,
                                  debug_mode);

        /* Enable ethtool support */
        cpmac_set_ethtool_ops( p_dev );

        /* Register the network device with Linux */
        failed = register_netdev(p_dev);

        if (failed) {
            printk("Cpmac: Could not register device for inst %d because of reason code %d.\n", unit, failed);
            retVal = -1;
            FREE_NETDEV(p_dev);


            platform_device_unregister(cpmac_dev);
            printk("platform device unregistered.\n");

            driver_remove_file(&cpmac_driver, &driver_attr_version);
            printk("driver file removed.\n");

            driver_unregister(&cpmac_driver);
            printk("driver unregistered.\n");

            break;
        } else {
            char proc_name[100];
            int proc_category_name_len = 0;
            /* create base directory for cpmacctl */
            cpmacctl = proc_mkdir("avalanche", init_net.proc_net);
            if (!cpmacctl) {
                errPrint("Unable to proc dir entry\n");
                remove_proc_entry("avalanche", init_net.proc_net);
                retVal = -ENOMEM;
            }

            hDDA->nextDevice = last_cpmac_device;
            last_cpmac_device = p_dev;
            dbgPrint("Cpmac: %s io=%04x\n", p_dev->name,
                     (int) p_dev->base_addr);
            strcpy(proc_name, p_dev->name);
            proc_category_name_len = strlen(proc_name);
            strcpy(proc_name + proc_category_name_len, "_rfc2665_stats");
            create_proc_read_entry(proc_name, 0, cpmacctl, cpmac_p_read_rfc2665_stats, p_dev);
        }
    }

    if (retVal == 0) {
        /* To maintain backward compatibility with NSP. */
        gp_stats_file =
            create_proc_entry("cpmac_stats", 0644, cpmacctl);
        if (gp_stats_file) {
            gp_stats_file->read_proc = cpmac_p_read_stats;
            gp_stats_file->write_proc = cpmac_p_write_stats;
        }
        create_proc_read_entry("cpmac_link", 0, cpmacctl, cpmac_p_read_link, NULL);
        create_proc_read_entry("cpmac_ver", 0, cpmacctl, cpmac_p_get_version, NULL);
        create_proc_read_entry("cpmac_config", 0, cpmacctl, cpmac_dump_config, NULL);
    }

    cpmac_devices_installed = unit;
    printk("TI %s - %s\n", cpmac_DDA_version_string, DDC_cpmacGetVersionInfo(NULL));
    printk("Cpmac: Installed %d instances.\n", unit);
#if defined CONFIG_ARM_CPMAC_INIT_BUF_MALLOC
    printk("Cpmac driver is allocating buffer memory at init time.\n");
#endif

    return ((unit >= 0) ? 0 : -ENODEV);

}

/* Frees the CPMAC device structures */
void cpmac_exit(void)
{
    struct net_device *p_dev;
    CpmacNetDevice *hDDA;
    PAL_Result retCode;


    while (cpmac_devices_installed) {
        char proc_name[100];
        int proc_category_name_len = 0;

        p_dev = last_cpmac_device;
        hDDA = NETDEV_PRIV(p_dev);
        if( p_dev ) {
            dbgPrint("Cpmac: Unloading %s io=%04x\n", p_dev->name, (int) p_dev->base_addr);
        }
        if (g_init_enable_flag) {
            cpmac_p_dev_disable(hDDA);
        }

#ifdef CONFIG_ARM_AVALANCHE_PPD
#ifdef CONFIG_TI_PACKET_PROCESSOR
    ti_ppm_delete_pid (p_dev->pid_handle);
#else
    ti_ppd_delete_pid(PP_ETH_PID_BASE+0);
#endif
    /* this delay is to make sure all the packets with the PID successfully egress throgh the respective ports.*/
    mdelay(DDA_CPMAC_SR_DELAY);
#endif


        /* Deinit DDC */
        retCode =
            hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcDeinit(hDDA->hDDC, NULL);
        if (retCode != CPMAC_SUCCESS) {
            errPrint("Cpmac: cpmac_exit:%s: Error %08X from DDC Deinit()\n", p_dev->name, retCode);
            /* We dont want to quit from here, lets delete the instance also */
        }

        /* Delete DDC Instance */
        retCode = hDDA->ddcIf->ddcNetIf.ddcFuncTable.ddcDelInst(hDDA->hDDC, NULL);
        if (retCode != CPMAC_SUCCESS) {
            errPrint("Cpmac: cpmac_exit:%s: Error %08X from DDC Delete Instance()\n", p_dev->name, retCode);
            /* We dont want to quit from here, lets continue the exit procedure for the driver */
        }
#if defined (CONFIG_ARM_AVALANCHE_COLORED_LED)
        led_manager_unregister_module(hDDA->ledHandle);
#endif

        if (cpmacctl != NULL) {

            /* Delete the proc entry */
            strcpy(proc_name, p_dev->name);
            proc_category_name_len = strlen(proc_name);
            strcpy(proc_name + proc_category_name_len, "_rfc2665_stats");
            remove_proc_entry(proc_name, NULL);
        }

        /* Release memory region and unregister the device */
        release_mem_region(p_dev->base_addr, CPMAC_DDA_DEFAULT_CPMAC_SIZE);
        unregister_netdev(p_dev);
        last_cpmac_device = hDDA->nextDevice;

        if (p_dev)
            FREE_NETDEV(p_dev);

        cpmac_devices_installed--;
    }

    if (cpmacctl != NULL) {
        if (gp_stats_file)
            remove_proc_entry("cpmac_stats", cpmacctl);


        remove_proc_entry("cpmac_link", cpmacctl);
        remove_proc_entry("cpmac_ver", cpmacctl);
        remove_proc_entry("cpmac_config", cpmacctl);
    }

    remove_proc_entry("avalanche", init_net.proc_net);

    platform_device_unregister(cpmac_dev);
    printk("platform device unregistered.\n");

    driver_remove_file(&cpmac_driver, &driver_attr_version);
    printk("driver file removed.\n");

    driver_unregister(&cpmac_driver);
    printk("driver unregistered.\n");

#ifdef CONFIG_ARM_EXTERNAL_SWITCH
    switch_remove_proc_entry();
    switch_exit();
#endif

}

/* Very minimal ethtool support for LRO */
static const struct ethtool_ops cpmac_ethtool_ops = {
        .get_flags              = ethtool_op_get_flags,
        .set_flags              = ethtool_op_set_flags,
};

static void cpmac_set_ethtool_ops(struct net_device *dev)
{
        SET_ETHTOOL_OPS(dev, (struct ethtool_ops *)&cpmac_ethtool_ops);
}


module_init(cpmac_dev_probe);
module_exit(cpmac_exit);

