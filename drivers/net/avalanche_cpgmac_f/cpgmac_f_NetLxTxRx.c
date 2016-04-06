/*
 *
 * cpmacNetLxTxRx.c
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


/** \file   cpmacNetLxTxRx.c
    \brief  CPMAC Linux DDA Send/Receive Source file

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

        "instId:BaseAddr:IntrLine:ResetLine:BusFreq:speed:duplex:promEn:BroadEn:MultiEn:maxRxPktLen:
         txNumBD:txServiceMax:rxNumBD:rxServiceMax:rxBufExtra:
         mdioBaseAddr:mdioIntrLine:mdioResetLine:mdioBusFreq:mdioClkFreq:mdioPhyMask:mdioTickMsec:"

        Example: "0:a304e000:15:32:12000000:0:0:0:1:1:1522:64:5:64:5:0:a8611e00:22:12000000:0:2:10:1:16:16:0:0:0:0:0:0:a3068000:a300a000:9"
                 "1:a304e800:16:33:12000000:0:0:0:1:1:1522:64:5:64:5:0:a8611e00:22:12000000:0:4:10:1:17:17:1:1:1:1:1:1:a3068000:a300a000:9"
                 
        Note: If speed = 9999 NO PHY mode selected

    
    (5) CPMAC_DDA_CACHE_INVALIDATE_FIX - to use the fix of invalidating the receive buffer before
        providing it to the DMA (to avoid possible data cache corruption if a dirty cache line is
        evicted and gets written after the RX DMA has written to the memory). If not defined, the receive
        buffer is invalidated after the DMA has been done (with possible data corruption). Since there
        is less probability of this condition occuring, the user of the driver can choose to use this
        fix or not. Note that by using this fix, the whole buffer is being invalidated rather than just
        the size of received packet, there is a performance hit if this fix is used.
    
    
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
#include <linux/skbuff.h>
#include <linux/ethtool.h>
#include <linux/highmem.h>
#include <linux/proc_fs.h>
#include <linux/ctype.h>
#include <asm/irq.h>            /* For NR_IRQS only. */
#include <asm/bitops.h>
#include <asm/io.h>
#include <asm/uaccess.h>
#include <mach/semaphore.h>
#if defined (CONFIG_ARM_AVALANCHE_COLORED_LED)
#include <asm-arm/arch-avalanche/generic/led_manager.h>
#endif
#include <linux/inet_lro.h>
#include <asm-arm/arch-avalanche/generic/ti_linux_porting.h>
#include <asm-arm/arch-avalanche/generic/if_port.h>

#ifdef CONFIG_PSP_TRACE
#include <linux/psp_trace.h>
#endif

#include <asm-arm/arch-avalanche/generic/pal.h>
#include "cpgmac_f_NetLx.h"     /* This will include required DDC headers */
#include "cpgmac_f_NetLxCfg.h"


#if defined (CONFIG_ARM_AVALANCHE_STATIC_SKB)
struct sk_buff *ti_alloc_skb(unsigned int size, int gfp_mask);
#endif
void cpmac_stats(void);

int cpmac_poll(struct napi_struct *napi, int budget)
{
    CpmacNetDevice *hDDA = container_of(napi, CpmacNetDevice, napi);
    struct net_device *p_dev = (struct net_device*)hDDA->owner;
    unsigned int rx_pkts_pending = 0; 
    unsigned int work_done = 0;

    /* This is used to pass the rx packets to be processed and return
     * the number of rx packets processed */
    RxTxParams *napi_params = &hDDA->napiRxTx;


    if (netif_running(p_dev)) {
        napi_params->rxPkts = budget;
        napi_params->txPkts = napi_params->txMaxService;


        /* Process packets - Call the DDC packet processing function */
        /* napi_params->rxPkts returns the number of rx packets processed */

#ifdef CPGMAC_DEBUG
        printk ("%s: start rx proc\n", __FUNCTION__);
#endif
        hDDA->ddcIf->rxPktProcess(hDDA->hDDC, &rx_pkts_pending,
                                  (Ptr) napi_params);

        work_done = napi_params->retRxPkts;

        if (rx_pkts_pending) {
#ifdef CPGMAC_DEBUG
            printk ("%s: packets pending....\n", __FUNCTION__);    
#endif    
            return  work_done; //or 0 ????
        }


        /*
         * order is important here. If we do EOI before calling napi_complete,
         * an interrupt can occur just before we take ourselves out of the
         * poll list; we will not schedule NAPI thread on that interrupt,
         * no further Rx interrupts and Rx will stall forever. Scary...
         */
        if (work_done < budget)
        {
#ifdef CONFIG_INET_LRO 
            if( p_dev->features & NETIF_F_LRO ) {
                lro_flush_all(&hDDA->lro_mgr);
            }
#endif
        	napi_complete(napi);
            hDDA->ddcIf->pktProcessEnd(hDDA->hDDC, NULL);    
        }
    }

    return work_done;
}



/* Allocate RX buffer */
Ptr DDA_cpmac_net_alloc_rx_buf(CpmacNetDevice * hDDA, Int bufSize,
                           DDC_NetDataToken * dataToken, Uint32 channel,
                           Ptr allocArgs)
{
    struct net_device *p_dev = hDDA->owner;
    struct sk_buff *p_skb;


#if defined (CONFIG_ARM_AVALANCHE_STATIC_SKB)
    p_skb = ti_alloc_skb(hDDA->rxBufSize, GFP_ATOMIC);
#else
    p_skb = dev_alloc_skb(hDDA->rxBufSize);
#endif

    if (p_skb != NULL) {
        /* Set device pointer in skb and reserve space for extra bytes */
        p_skb->dev = p_dev;
        skb_reserve(p_skb, hDDA->rxBufOffset);

        /* Set the data token */
        *dataToken = (DDC_NetDataToken) p_skb;

#ifdef CPMAC_DDA_CACHE_INVALIDATE_FIX
        /* Invalidate buffer */
        CPMAC_DDA_CACHE_INVALIDATE((unsigned long) ret_ptr, bufSize);
#endif
    } else {
#ifdef CPMAC_DDA_DEBUG          /* We dont want the error printf to appear on screen as it clogs the serial port */
        /* errPrint("Failed to allocate skb for %s.\n", p_dev->name); */
#endif
        return (NULL);
    }

    return p_skb->data;
}


PAL_Result DDA_cpmac_net_free_rx_buf(CpmacNetDevice * hDDA, Ptr buffer,
                          DDC_NetDataToken dataToken, Uint32 channel,
                          Ptr freeArgs)
{
    dev_kfree_skb_any((struct sk_buff *) dataToken);
    return (CPMAC_SUCCESS);
}



#if 0
#define isprint(a) ((a >=' ')&&(a<= '~'))
void xdump(unsigned char *cp, int length, char *prefix)
{
    int col, count;
    u_char prntBuf[120];
    u_char *pBuf = prntBuf;
    count = 0;
    while (count < length) {
        pBuf += sprintf(pBuf, "%s", prefix);
        for (col = 0; count + col < length && col < 16; col++) {
            if (col != 0 && (col % 4) == 0)
                pBuf += sprintf(pBuf, " ");
            pBuf += sprintf(pBuf, "%02X ", cp[count + col]);
        }
        while (col++ < 16) {    /* pad end of buffer with blanks */
            if ((col % 4) == 0)
                sprintf(pBuf, " ");
            pBuf += sprintf(pBuf, "   ");
        }
        pBuf += sprintf(pBuf, "  ");
        for (col = 0; count + col < length && col < 16; col++) {
            if (isprint((int) cp[count + col]))
                pBuf += sprintf(pBuf, "%c", cp[count + col]);
            else
                pBuf += sprintf(pBuf, ".");
        }
        sprintf(pBuf, "\n");
        // SPrint(prntBuf);
        printk(prntBuf);
        count += col;
        pBuf = prntBuf;
    }

}                               /* close xdump(... */
#endif

/* Receive Packet */

PAL_Result DDA_cpmac_net_rx(CpmacNetDevice * hDDA, DDC_NetPktObj * pkt, Ptr rxArgs,
                 Ptr arg)
{
    struct sk_buff *p_skb = (struct sk_buff *) pkt->pktToken;
#ifdef CONFIG_TI_PACKET_PROCESSOR
    struct EPI * epi;
#endif
		
#if defined(CONFIG_ARM_AVALANCHE_MARVELL_6063) || defined(CONFIG_ARM_AVALANCHE_MARVELL_6060)
    struct net_device *p_cpmac_net_device =
        (struct net_device *) hDDA->owner;
#endif

    skb_put(p_skb, pkt->pktLength);

#ifndef CPMAC_DDA_CACHE_INVALIDATE_FIX
    /* Invalidate cache */
    CPMAC_DDA_CACHE_INVALIDATE(p_skb->data, pkt->pktLength);
#endif
#ifdef CONFIG_TI_PACKET_PROCESSOR
    epi = (struct EPI *)pkt->bufList[0].bufToken;
    if(epi->word1)
    {
#ifdef CPGMAC_DEBUG
        printk("\n\n\n\n\nPTID RECEIVED\n");
#endif
        memcpy(p_skb->pp_packet_info.ti_epi_header,epi,sizeof(struct EPI));
        p_skb->pp_packet_info.ti_pp_flags = TI_PPM_SESSION_INGRESS_RECORDED;
    }
#endif

    /* Remove the header or trailer from the frame if coming from a Ethernet Switch */
#if defined(CONFIG_ARM_AVALANCHE_MARVELL_6063)
    /* Fetch the receiving port information */
    p_cpmac_net_device->if_port = (unsigned char) p_skb->data[pkt->pktLength - (EGRESS_TRAILOR_LEN - 1)] 
	    + AVALANCHE_MARVELL_BASE_PORT_ID;
    skb_trim(p_skb, pkt->pktLength - EGRESS_TRAILOR_LEN);
#endif

#if defined(CONFIG_ARM_AVALANCHE_FAST_BRIDGE)
    if (p_skb->dev->br_port == NULL)
#endif
    /* Remove the header or trailer from the frame if coming from a Ethernet Switch */
    p_skb->protocol = eth_type_trans(p_skb, hDDA->owner);

    /* Drop unknown protocol frames */
    {
    	struct ethhdr *eth;
    	eth = eth_hdr(p_skb);

        if(eth->h_proto == 0) {
            hDDA->unknownProtPkts++;
            kfree_skb(p_skb);
            return (0);
        }
        
    }

    p_skb->dev->last_rx = jiffies;

 #ifdef CONFIG_INET_LRO 
  if( p_skb->dev->features & NETIF_F_LRO ) {
        lro_receive_skb(&hDDA->lro_mgr, p_skb, (void *)hDDA);
    } else {
        netif_receive_skb(p_skb);
    }
#else
        netif_receive_skb(p_skb);
#endif

#if defined (CONFIG_ARM_AVALANCHE_COLORED_LED)
    led_manager_led_action(hDDA->ledHandle, CPMAC_RX_ACTIVITY);
#endif


    hDDA->netDevStats.rx_packets++;
    hDDA->netDevStats.rx_bytes += pkt->pktLength;

#ifdef CONFIG_PSP_TRACE
    psp_trace_par(ETH_DRV_PKT_RX_EXIT, p_skb);
#endif

    return (0);
}


/* Transmit Complete Callback */
PAL_Result DDA_cpmac_net_tx_complete(CpmacNetDevice * hDDA,
                          DDC_NetDataToken * netDataTokens, Int numTokens,
                          Uint32 channel)
{
    Uint32 cnt;

    if (numTokens && netif_queue_stopped(hDDA->owner))
        netif_start_queue(hDDA->owner);

    for (cnt = 0; cnt < numTokens; cnt++) {

        struct sk_buff *skb = (struct sk_buff *) netDataTokens[cnt];

        struct net_device_stats *stat = &hDDA->netDevStats;

        if (skb == NULL)
            continue;
        stat->tx_packets++;
        stat->tx_bytes += skb->len;
        dev_kfree_skb_any(skb);
    }

    return (0);
}


/******************************************************************************
 *  Interrupt / Tasklet related functions
 *****************************************************************************/

/* DDA ISR */
irqreturn_t cpmac_hal_tx_isr(int irq, void *dev_id)
{
    CpmacNetDevice *hDDA = (CpmacNetDevice *) dev_id;

#ifdef CONFIG_PSP_TRACE
    psp_trace(ETH_DRV_TX_ISR_ENTER);
#endif

    ++hDDA->txisrCount;


    if (!hDDA->setToClose) {
#ifdef TX_TASKLET_MODE
        tasklet_schedule(&hDDA->tx_tasklet);
#else
        int tx_pkts_pending = 0;
#ifdef CPGMAC_DEBUG
        printk ("%s: start tx proc\n", __FUNCTION__);
#endif
        hDDA->ddcIf->txPktProcess(hDDA->hDDC, &tx_pkts_pending, NULL);
        hDDA->ddcIf->txPktProcessEnd(hDDA->hDDC, NULL);

#ifdef CPGMAC_USE_ACC_LIST
        if (tx_pkts_pending) {
            /* !@@ TODO : Log error since the execution should never reach
             * here if the configuration was proper. This condition means the
             * EOI was done and the list page was returned before all the the
             * packets/descriptors were processed.
             */
        printk ("%s: Pkts pending...\n", __FUNCTION__); 
        }
#endif /* CPGMAC_USE_ACC_LIST */

#endif
    } else {
        /* We are closing down, so dont process anything */
    }
#ifdef CONFIG_PSP_TRACE
    psp_trace(ETH_DRV_TX_ISR_EXIT);
#endif

    return IRQ_HANDLED;
}

irqreturn_t cpmac_hal_rx_isr(int irq, void *dev_id)
{
    CpmacNetDevice *hDDA = (CpmacNetDevice *) dev_id;

#ifdef CONFIG_PSP_TRACE
    psp_trace(ETH_DRV_RX_ISR_ENTER);
#endif

    ++hDDA->rxisrCount;

    if (!hDDA->setToClose) 
    {
   
#ifdef RX_TASKLET_MODE
      tasklet_schedule (&hDDA->rx_tasklet);
#else
        if (napi_schedule_prep(&hDDA->napi))
		{
            __napi_schedule(&hDDA->napi); 
        }
		else {           
                /* The interface has been pulled down.
                 * Indicate that the EOI is set to trigger interrupts next time. */
                hDDA->Clear_EOI++;
    	}
#endif
    } else {
        /* We are closing down, so dont process anything */
    }
#ifdef CONFIG_PSP_TRACE
    psp_trace(ETH_DRV_RX_ISR_EXIT);
#endif

    return IRQ_HANDLED;
}


/* Transmit Function - Only single fragment supported */
/**
 *  cpmac_dev_tx

 *    - sendArgs Bit Mapping as follows as required by the DDC_cpmacSend function:
 *
 *        (Note: Bits 15-0 map directly to CPPI4 BD Word 2 bits 31-16.)
 *        17-16 Priority Queue.  0 to 3.
 *        15-12 Packet Type
 *        11    Descriptor Type (0 = Host, 1 = Embedded Mode)
 *        10-8  Additional Buffer Count (0-7).  Set to 0 for Host Mode.
 *        7     Reserved
 *        6-4   Protocol Specific Region Offset.  Set to 0 for Host Mode.
 *        3     Pass CRC Bit.  When 0, the MAC calculates and inserts CRC.  When
 *              set, the MAC will expect a CRC to be in the data buffer with the
 *              packet data.
 *        2     No VLAN Bit.  Remove VLAN tag if set.
 *        1-0   Reserved.
 */
int cpmac_dev_tx(struct sk_buff *skb, struct net_device *p_dev)
{
    Uint32 sendArgs = 0;
    PAL_Result retCode = CPMAC_SUCCESS;
    DDC_NetBufObj txBuf;        /* Buffer object - Only single frame support */
    DDC_NetPktObj txPacket;     /* Packet object */
    CpmacNetDevice *hDDA = NETDEV_PRIV(p_dev);    
#ifdef CONFIG_PSP_TRACE
    psp_trace_par(ETH_DRV_PKT_TX_ENTER, skb);
#endif

    /* Build the sendArgs to pass it to ddc layer DDC_cpmacSend function 
     * Descriptor type set to 0 host mode. Additional buffer count 0 for host 
     * mode Pass CRC bit 0 Mac claculated and inserts CRC. VLAN bit set to 0
     */
    sendArgs |= (CPMAC_DEFAULT_CPPI4_ETHERNET_PKT_TYPE_BD << 12);

     if (skb->len < CPMAC_MIN_ETHERNET_PKT_SIZE)
     { 
		int pad = CPMAC_MIN_ETHERNET_PKT_SIZE - skb->len;
       	if (skb_tailroom(skb) >= pad) {
        	memset(skb->data+skb->len, 0, pad);
        	skb_put(skb,pad);     	   
       	} 
     }

    /* Build the buffer and packet objects - Since only single fragment is supported,
      need not set length and token in both packet & object. Doing so for completeness sake & to
     * show that this needs to be done in multifragment case
     */
    txPacket.bufList = &txBuf;
    txPacket.numBufs = 1;       /* Only single fragment supported */
    txPacket.pktLength = skb->len;
    txPacket.pktToken = (DDC_NetDataToken) skb;
    txBuf.length = skb->len;
#ifdef CONFIG_TI_PACKET_PROCESSOR 
    if(skb->pp_packet_info.ti_pp_flags == TI_PPM_SESSION_INGRESS_RECORDED)
        txBuf.bufToken = (DDC_NetDataToken)skb->pp_packet_info.ti_epi_header;
    else
#endif		
    txBuf.bufToken = (DDC_NetDataToken) skb;
    txBuf.dataPtr = skb->data;

    /****************************************************************/
    /* Chose to place packet on QoS queues or directly to the ETH   */
    /* ... of cause according to the desired priority.              */
    /****************************************************************/
    if (skb->dev->vpid_block.qos_clusters_count)
    {
        sendArgs |= CPMAC_QOS_ACTIVE;
        /*==================================*/
        /*  The following is a mapping of   */
        /*  priorities according to DOCSIS  */
        /*  specified priority:             */
        /*   DOCSIS     ETH                 */
        /*     0,1       3 (Low)            */
        /*     2,3       2 (Low-Med)        */
        /*     4,5       1 (Low-High)       */
        /*     6,7       0 (High)           */
        /*==================================*/   
        sendArgs |= ((3 - ((skb->ti_meta_info & 0x7) >> 1)) << 16);
    }
    else
    {
        /*==================================*/
        /*  The following is a mapping of   */
        /*  priorities according to DOCSIS  */
        /*  specified priority:             */
        /*   DOCSIS     ETH                 */
        /*     0-3       1 (Low)            */
        /*     4-7       0 (High)           */
        /*==================================*/   
        sendArgs |= ((1 - ((skb->ti_meta_info & 0x7) >> 2)) << 16);
    }
    /****************************************************************/

    /* Flush data buffer if write back mode */
    CPMAC_DDA_CACHE_WRITEBACK((unsigned long) skb->data, skb->len);

    p_dev->trans_start = jiffies;

    retCode =
        hDDA->ddcIf->ddcNetIf.ddcNetSend(hDDA->hDDC, &txPacket,
                                         CPMAC_DDA_DEFAULT_TX_CHANNEL,
                                         (Ptr) sendArgs);

    if (retCode == CPMAC_SUCCESS) {
#if defined (CONFIG_ARM_AVALANCHE_COLORED_LED)
        led_manager_led_action(hDDA->ledHandle, CPMAC_TX_ACTIVITY);
#endif
#ifdef CONFIG_PSP_TRACE
        psp_trace_par(ETH_DRV_PKT_TX_EXIT, skb);
#endif

        return NETDEV_TX_OK;
    } else {
        if (retCode == CPMAC_ERR_TX_OUT_OF_BD)
            netif_stop_queue(hDDA->owner);

        hDDA->netDevStats.tx_errors++;
        hDDA->netDevStats.tx_dropped++;
#ifdef CONFIG_PSP_TRACE
        psp_trace_2_par(ETH_DRV_PKT_TX_EXIT, skb, retCode);
#endif
        return NETDEV_TX_BUSY;
    }
}

#ifdef TX_TASKLET_MODE

void cpmac_handle_tx_tasklet(unsigned long data)
{
    CpmacNetDevice *hDDA = (CpmacNetDevice *) data;

#ifdef CONFIG_PSP_TRACE
    psp_trace(ETH_DRV_TX_TASKLET_ENTER);
#endif

    int pkts_pending = 0;

    if (!hDDA->setToClose) {
        /* Process packets - Call the DDC packet processing function */
        hDDA->ddcIf->txPktProcess(hDDA->hDDC, &pkts_pending, NULL);

        /* If more packets reschedule the tasklet or call pktProcessEnd */
        if (!pkts_pending) {
            hDDA->ddcIf->txPktProcessEnd(hDDA->hDDC, NULL);
        } else {
#ifdef CPGMAC_DEBUG
            printk ("%s: packets pending....\n", __FUNCTION__);    
#endif
            tasklet_schedule(&hDDA->tx_tasklet);
        }

    } else {
        /* if we are closing down, make sure interrupts can be generated in future 
         * as and when packets are available and we are willing to process them. */
        hDDA->ddcIf->pktProcessEnd(hDDA->hDDC, NULL);
    }
#ifdef CONFIG_PSP_TRACE
    psp_trace(ETH_DRV_TX_TASKLET_EXIT);
#endif

}

#endif

#ifdef RX_TASKLET_MODE
void cpmac_handle_rx_tasklet (unsigned long data)
{
    CpmacNetDevice *hDDA = (CpmacNetDevice *) data;
    int pkts_pending = 0;
    if (!hDDA->setToClose)
    {
#ifdef CPGMAC_DEBUG
        printk ("%s: start rx proc\n", __FUNCTION__);
#endif
        /* Process packets - Call the DDC packet processing function */
        hDDA->ddcIf->rxPktProcess (hDDA->hDDC, &pkts_pending, NULL);

        /* If more packets reschedule the tasklet or call pktProcessEnd */
        if (!pkts_pending)
        {
            hDDA->ddcIf->pktProcessEnd(hDDA->hDDC, NULL);
        }
        else
        {
#ifdef CPGMAC_DEBUG
            printk ("%s: pkts pending\n", __FUNCTION__);    
#endif
            tasklet_schedule (&hDDA->rx_tasklet);
        }
    }   
    else
    {
        /* if we are closing down, make sure interrupts can be generated in future
        * as and when packets are available and we are willing to process them. */
        hDDA->ddcIf->pktProcessEnd (hDDA->hDDC, NULL);
    }
}
#endif
