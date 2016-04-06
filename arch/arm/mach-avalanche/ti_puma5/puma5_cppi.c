/*
 *
 * puma5_cppi.c
 * Description:
 * Puma-5 CPPI initialization.
 * Contains Puma-5 specific initialization. The numbers (addresses etc) are
 * Puma-5 specific. The static structures are filled in with Puma-5 specific
 * data and the generic CPPI4.1 init function gets called in the end
 * with this data.
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


#include <asm-arm/arch-avalanche/generic/pal.h>
#include <asm-arm/arch-avalanche/puma5/puma5_cppi.h>


/* FIXME : uncomment line below to enable debug prints */
//#define DEBUG_PAL(fmt,arg...)  printk(KERN_EMERG fmt , ##arg);

#ifndef DEBUG_PAL
#define DEBUG_PAL(fmt,arg...)
#endif


/* This CPPI 4.1 global initialization structure. Contains static initialization
 * information for the whole CPPI subsystem. Details regarding structure fields
 * can be obtained from PAL documentation. Some comments on how the structure
 * can be populated/extended are thrown here as well.
 *
 * *** IMPORTANT *** *** ALWAYS REMEMBER *** *** FOLLOW THESE RULES ***
 *
 * - Keep all base addresses of descriptor regions in ascending order.
 *   Where descriptor is allocated by CPPI from SDRAM, the base here is
 *   specified as zero. Treat the zero as SDRAM base address while ensuring
 *   the ascending order. Keep all desc regiosns meant to be on SDRAM
 *   together.
 *
 * - Sort the desc regions defined on SDRAM (base == 0) in descending order of
 *   desc size. This is required to ensure easy alignment (bit 14 aligned to
 *   bit 15 and so on.)
 *
 * - Apply these rules to each queue manager in the system *separately*.
 *
 * - Read the PAL CPPI documentation (especially the Cppi4InitCfg structure doc)
 *   before starting to modify this structure.
 *
 */

Cppi4InitCfg cppi4InitCfg_g =
{
    .resetLine              = 0,

    .dmaBlock[PAL_CPPI41_DMA_BLOCK0].globalCtrlBase     = (Ptr) AVALANCHE_NWSS_DMA0_GBLCFG_BASE,//changed from DMA1 to DMA0
    .dmaBlock[PAL_CPPI41_DMA_BLOCK0].chCtrlStatusBase   = (Ptr) AVALANCHE_NWSS_DMA0_CHNCFG_BASE,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK0].schedCtrlBase      = (Ptr) AVALANCHE_NWSS_DMA0_SCHEDCFG_BASE,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK0].schedTableBase     = (Ptr) AVALANCHE_NWSS_DMA0_SCHEDTBL_BASE,

    .dmaBlock[PAL_CPPI41_DMA_BLOCK0].tdFQueue.qMgr      = DMA0_CPPI4x_FTD_QMGR,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK0].tdFQueue.qNum      = DMA0_CPPI4x_FTD_QNUM,

    .dmaBlock[PAL_CPPI41_DMA_BLOCK1].globalCtrlBase     = (Ptr) AVALANCHE_NWSS_DMA1_GBLCFG_BASE,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK1].chCtrlStatusBase   = (Ptr) AVALANCHE_NWSS_DMA1_CHNCFG_BASE,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK1].schedCtrlBase      = (Ptr) AVALANCHE_NWSS_DMA1_SCHEDCFG_BASE,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK1].schedTableBase     = (Ptr) AVALANCHE_NWSS_DMA1_SCHEDTBL_BASE,

    .dmaBlock[PAL_CPPI41_DMA_BLOCK1].tdFQueue.qMgr      = DMA1_CPPI4x_FTD_QMGR,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK1].tdFQueue.qNum      = DMA1_CPPI4x_FTD_QNUM,


    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].queueMgrRgnBase        = (Ptr) AVALANCHE_NWSS_QMGR_RGN_BASE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descMemRgnBase         = (Ptr) AVALANCHE_NWSS_DESCMEM_RGN_BASE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].queueMgmtRgnBase       = (Ptr) AVALANCHE_NWSS_QMGMT_RGN_BASE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].queueProxyRgnBase      = (Ptr) AVALANCHE_NWSS_QPROXY_0_RGN_BASE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].queueStatusRgnBase     = (Ptr) AVALANCHE_NWSS_QSTATUS_RGN_BASE,

    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].LinkingRAM0Base        = IO_VIRT2PHY(AVALANCHE_NWSS_PACKET_RAM_BASE),
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].LinkingRAM0Size        = 7168,
            /* Max pointers in this region could be 7K (assuming 32-bit pointers). */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].LinkingRAM1Base        = 0,

    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[0].base     = (Ptr) 0x01119000,   /* Docsis MAC internal memory */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[0].szDesc   = 128,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[0].numDesc  = 32,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[0].isOnChip = 1,

    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[1].base     = (Ptr) IO_VIRT2PHY( AVALANCHE_NWSS_ONCHIPDESC_BASE ),    /* Onchip */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[1].szDesc   = PPFW_CPPI4x_BD_SIZE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[1].numDesc  = PPFW_PREFETCH_DESC_COUNT + PPFW_REPLICA_DESC_COUNT,     /* Total BD Count: Prefetcher + Replication */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[1].isOnChip = 1,

    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[2].base     = (Ptr) IO_VIRT2PHY( AVALANCHE_NWSS_APDSP_PREFBLK_BASE ), /* Prefetch */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[2].szDesc   = PPFW_CPPI4x_BUF_BD_SIZE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[2].numDesc  = PPFW_PREFETCH_BUFF_COUNT,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[2].isOnChip = 1,

    /* Reserve region with following specifics for Embedded */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[3].base     = 0,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[3].szDesc   = 64,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[3].numDesc  = 2496 + PPFW_EVENT_DESC_NUM /* !@@ Use as per PP system req */,

    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[4].base     = 0,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[4].szDesc   = USB_CPPI4x_USB_BD_SIZE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[4].numDesc  = USB_CPPI4x_MAX_USB_DESC,

    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[5].base     = 0,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[5].szDesc   = CPMAC_CPPI4x_TX_HOST_BD_SIZE, /* sizeof (Cppi4HostDesc) + alignment */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[5].numDesc  = CPMAC_TX_HOST_BD_NUM,

    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[6].base     = 0,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[6].szDesc   = CPMAC_CPPI4x_RX_HOST_BD_SIZE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[6].numDesc  = CPMAC_RX_HOST_BD_NUM,

    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[7].base     = 0,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[7].szDesc   = CPPI4x_SR_DOCSIS_MGMT_RX_FD_HOST_DESC_SIZE, /* 64 byte sized */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[7].numDesc  = CPPI4x_SR_DOCSIS_MGMT_RX_FD_HOST_DESC_NUM + CPPI4x_SR_DOCSIS_MGMT_TX_FD_HOST_DESC_NUM,

    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[8].base     = 0,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[8].szDesc   = 64, /* 64 byte sized */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[8].numDesc  = 256, //512,

    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[9].base     = 0,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[9].szDesc   = sizeof(Cppi4TeardownDesc), /* 32 byte sized */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[9].numDesc  = PAL_CPPI41_NUM_TD_DESC * 2,

    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[10].base    = 0,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[10].szDesc  = CPPI4x_CNI_RX_INFRA_FD_HOST_DESC_SIZE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[10].numDesc = CPPI4x_CNI_RX_INFRA_FD_HOST_DESC_NUM,

    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[11].base    = 0,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[11].szDesc  = PPFW_HOST_BD_SIZE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[11].numDesc = PPFW_HOST_BD_NUM,

    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[12].base     = 0,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[12].szDesc   = MPEG_BD_SIZE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[12].numDesc  = DMAC_MPEG_RX_EMBEDDED_BD_NUM,

    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[13].base     = 0,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[13].szDesc   = MPEG_ENCAP_BD_SIZE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[13].numDesc  = DMAC_MPEG_ENCAP_RX_EMBEDDED_BD_NUM,

    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].basefdQNum              = PAL_CPPI41_FD_Q_BASE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].basefdbQNum             = PAL_CPPI41_FBD_Q_BASE,


    .bufMgrBase  [PAL_CPPI41_BUF_MGR_PARTITION_SR]                          = (CSL_BufMgr_RegsOvly) AVALANCHE_NWSS_BMGR_BASE,

    .dmaBlock[PAL_CPPI41_DMA_BLOCK0].schedTable.numEntries  = 2,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK0].schedTable.entry =
    {
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, CPMAC_CPPI4x_RX_DMA_CHNUM(0) ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, CPMAC_CPPI4x_TX_DMA_CHNUM(0) ),
    },

    .dmaBlock[PAL_CPPI41_DMA_BLOCK1].schedTable.numEntries  = 37,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK1].schedTable.entry =
    {
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, CPMAC_CPPI4x_ETH2HOST_PROXY_CHNUM(0) ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, CPMAC_CPPI4x_ETH2HOST_PROXY_CHNUM(0) ),

        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, RECYCLE_INFRA_CHN(0) ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, RECYCLE_INFRA_CHN(0) ),

        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, USB_CPPI4x_RX_DMA_CHNUM(0)  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, USB_CPPI4x_TX_DMA_CHNUM(0)  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, USB_CPPI4x_RX_DMA_CHNUM(1)  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, USB_CPPI4x_TX_DMA_CHNUM(1)  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, USB_CPPI4x_RX_DMA_CHNUM(2)  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, USB_CPPI4x_TX_DMA_CHNUM(2)  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, USB_CPPI4x_RX_DMA_CHNUM(3)  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, USB_CPPI4x_TX_DMA_CHNUM(3)  ),

        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, CPPI4x_SR_DOCSIS_RX_DMA_CHNUM(0)  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, CPPI4x_SR_DOCSIS_RX_DMA_CHNUM(1)  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, CPPI4x_SR_DOCSIS_RX_DMA_CHNUM(2)  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, MPEG_CPPI4x_RX_DMA_CHNUM    ),

        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, MPEGOUT_CPPI4x_CHNUM        ),

        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, CPPI4x_SR_DOCSIS_TX_DMA_CHNUM(0)  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, CPPI4x_SR_DOCSIS_TX_DMA_CHNUM(1)  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, CPPI4x_SR_DOCSIS_TX_DMA_CHNUM(2)  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, CPPI4x_SR_DOCSIS_TX_DMA_CHNUM(3)  ),

        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, CPPI4x_SR_DOCSIS_MGMT_RXCMPL_CHNUM ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, CPPI4x_SR_DOCSIS_MGMT_RXCMPL_CHNUM ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, CPPI4x_SR_DOCSIS_TX_CoP_DMA_RX_CHNUM(0)  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, CPPI4x_SR_DOCSIS_TX_CoP_DMA_TX_CHNUM(0)  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, CPPI4x_CNI_RX_INFRA_CH_NUM(0)  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, CPPI4x_CNI_RX_INFRA_CH_NUM(0)  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, CPPI4x_CNI_RX_INFRA_CH_NUM(1)  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, CPPI4x_CNI_RX_INFRA_CH_NUM(1)  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, CPPI4x_CNI_RX_INFRA_CH_NUM(2)  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, CPPI4x_CNI_RX_INFRA_CH_NUM(2)  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, USB_CPPI4x_USB2HOST_PROXY_CHNUM(0)      ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, USB_CPPI4x_USB2HOST_PROXY_CHNUM(0)      ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, CPPI4x_PPFW_TX_INFRA_CHNUM(0)   ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, CPPI4x_PPFW_TX_INFRA_CHNUM(0)   ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, CPPI4x_PPFW_TX_INFRA_CHNUM(1)   ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, CPPI4x_PPFW_TX_INFRA_CHNUM(1)   ),
    },
    .apdspInfo.pdspCmdBase      = (Ptr)     AVALANCHE_NWSS_APDSP_CMD_BASE,
};

int avalanche_cppi_init(void)
{
    if(PAL_cppi4Init(&cppi4InitCfg_g, NULL) != NULL)
    {
        DEBUG_PAL("PAL_cppi4Init: CPPI 4.1 API initialized successfully.\n");
    }
    else
    {
        DEBUG_PAL("PAL_cppi4Init: ERROR: CPPI 4.1 API initialization failed!\n");
    }

    return 0;
}

