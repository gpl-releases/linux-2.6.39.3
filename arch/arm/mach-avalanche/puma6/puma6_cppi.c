/*
 *
 * puma6_cppi.c
 * Description:
 * Puma-6 CPPI initialization.
 * Contains Puma-6 specific initialization. The numbers (addresses etc) are
 * Puma-6 specific. The static structures are filled in with Puma-6 specific
 * data and the generic CPPI4.1 init function gets called in the end
 * with this data.
 *
 */

#include <pal.h>
#include <puma6_cppi.h>
#include <linux/proc_fs.h>

/* FIXME : uncomment line below to enable debug prints */
//#define DEBUG_PAL(fmt,arg...)  printk(KERN_EMERG fmt , ##arg);

#ifndef DEBUG_PAL
#define DEBUG_PAL(fmt,arg...)
#endif

static PAL_Result puma6_cppi_proc_init (Ptr hnd, Ptr param);

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

    .bufMgrBase  [PAL_CPPI41_BUF_MGR_PARTITION_SR]                          = (CSL_BufMgr_RegsOvly) AVALANCHE_NWSS_BMGR_BASE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].queueMgrRgnBase        = (Ptr) AVALANCHE_NWSS_QMGR_RGN_BASE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descMemRgnBase         = (Ptr) AVALANCHE_NWSS_DESCMEM_RGN_BASE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].queueMgmtRgnBase       = (Ptr) AVALANCHE_NWSS_QMGMT_RGN_BASE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].queueProxyRgnBase      = (Ptr) AVALANCHE_NWSS_QPROXY_0_RGN_BASE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].queueStatusRgnBase     = (Ptr) AVALANCHE_NWSS_QSTATUS_RGN_BASE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].totalQNum              = PAL_CPPI41_SR_QMGR_TOTAL_Q_COUNT,

    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].LinkingRAM0Base        = IO_VIRT2PHY( AVALANCHE_NWSS_PACKET_RAM_BASE ),
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].LinkingRAM0Size        = 23040, /* Maximum number of descriptors */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].LinkingRAM1Base        = 0,

    /* DOCSIS TX Internal Memory, used for DOCSIS TX CoP Monolithic */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[0].base     = (Ptr) IO_VIRT2PHY( AVALANCHE_DOCSIS_SS_US_PACKET_RAM_BASE ),
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[0].szDesc   = PAL_CPPI41_SR_DOCSIS_TX_MONOLITHIC_DESC_SIZE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[0].numDesc  = PAL_CPPI41_SR_DOCSIS_TX_MONOLITHIC_DESC_COUNT,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[0].isOnChip = 1,

    /* PP Internal Memory, used for prefetcher buffers */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[1].base     = (Ptr) IO_VIRT2PHY( AVALANCHE_NWSS_APDSP_PREFBLK_BASE ),
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[1].szDesc   = PAL_CPPI41_SR_PPDSP_PREFETCH_BUFF_FD_DESC_SIZE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[1].numDesc  = PAL_CPPI41_SR_PPDSP_PREFETCH_BUFF_FD_DESC_COUNT,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[1].isOnChip = 1,

    /* PP Internal Memory, used for prefetcher descriptors */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[2].base     = (Ptr) IO_VIRT2PHY( AVALANCHE_NWSS_ONCHIPDESC_BASE ),
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[2].szDesc   = PAL_CPPI41_SR_PPDSP_PREFETCH_DESC_FD_DESC_SIZE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[2].numDesc  = PAL_CPPI41_SR_PPDSP_PREFETCH_DS_DESC_FD_DESC_COUNT + PAL_CPPI41_SR_PPDSP_PREFETCH_US_DESC_FD_DESC_COUNT,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[2].isOnChip = 1,

    /* MPEG ENCAP */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[3].base     = 0,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[3].szDesc   = MPEG_ENCAP_BD_SIZE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[3].numDesc  = DMAC_MPEG_ENCAP_RX_EMBEDDED_BD_NUM,

    /* Host RX & TX DOCSIS Management Free Host Descriptors */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[4].base     = 0,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[4].szDesc   = PAL_CPPI41_SR_DOCSIS_MGMT_RX_FD_HOST_DESC_SIZE, /* 64 byte sized */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[4].numDesc  = PAL_CPPI41_SR_DOCSIS_MGMT_RX_FD_HOST_DESC_COUNT + PAL_CPPI41_SR_DOCSIS_MGMT_TX_FD_HOST_DESC_COUNT,

    /* Group of embedded descriptors: DOCSIS RX CoP, Host2PP Infrastructure & PP Events */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[5].base     = 0,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[5].szDesc   = 64,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[5].numDesc  = PAL_CPPI41_SR_PP_EVENTS_FD_DESC_COUNT +
                                                                              PAL_CPPI41_SR_HOST_TO_PP_INFRA_LOW_FD_EMB_DESC_COUNT +
                                                                              PAL_CPPI41_SR_HOST_TO_PP_INFRA_HIGH_FD_EMB_DESC_COUNT +
                                                                              PAL_CPPI41_SR_DOCSIS_RX_CoP_LOW_FD_EMB_DESC_COUNT +
                                                                              PAL_CPPI41_SR_DOCSIS_RX_CoP_HIGH_FD_EMB_DESC_COUNT +
                                                                              PAL_CPPI41_SR_PROXY_PDSP_LOW_FD_EMB_DESC_COUNT +
                                                                              PAL_CPPI41_SR_PROXY_PDSP_MED_LOW_FD_EMB_DESC_COUNT +
                                                                              PAL_CPPI41_SR_PROXY_PDSP_MED_HIGH_FD_EMB_DESC_COUNT +
                                                                              PAL_CPPI41_SR_PROXY_PDSP_HIGH_FD_EMB_DESC_COUNT,

    /* VOICE DSP RX Free Embedded Descriptors */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[6].base     = 0,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[6].szDesc   = 64, /* 64 byte sized */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[6].numDesc  = 256, //512,

    /* CNI Infrastructure Free Host Descriptors */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[7].base    = 0,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[7].szDesc  = PAL_CPPI41_SR_CNI_INFRA_FD_HOST_DESC_SIZE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[7].numDesc = PAL_CPPI41_SR_CNI_INFRA_FD_HOST_DESC_COUNT,

    /* Host2PP Free Host Descriptors */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[8].base    = 0,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[8].szDesc  = PAL_CPPI41_SR_HOST_TO_PP_FD_HOST_DESC_SIZE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[8].numDesc = PAL_CPPI41_SR_HOST_TO_PP_FD_HOST_DESC_COUNT,

    /* MPEG */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[9].base     = 0,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[9].szDesc   = MPEG_BD_SIZE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[9].numDesc  = DMAC_MPEG_RX_EMBEDDED_BD_NUM,

    /* L2SW Infrastructure Free Host Descriptors */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[10].base     = 0,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[10].szDesc   = PAL_CPPI41_SR_L2SW_INFRA_FD_HOST_DESC_SIZE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[10].numDesc  = PAL_CPPI41_SR_L2SW_INFRA_FD_HOST_DESC_COUNT,

    /* Teardown descriptors */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[11].base    = 0,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[11].szDesc  = sizeof(Cppi4TeardownDesc),    /* 32 byte sized        */
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].descRegion[11].numDesc = PAL_CPPI41_NUM_TD_DESC * 4,   /* Have to serve 4 DMAs */


    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].basefdQNum             = PAL_CPPI41_SR_FD_EMB_Q_BASE,
    .queueMgrInfo[PAL_CPPI41_QUEUE_MGR_PARTITION_SR].basefdbQNum            = PAL_CPPI41_SR_FD_HOST_Q_BASE,

    .dmaBlock[PAL_CPPI41_DMA_BLOCK0].globalCtrlBase     = (Ptr) AVALANCHE_NWSS_DMA0_GBLCFG_BASE,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK0].chCtrlStatusBase   = (Ptr) AVALANCHE_NWSS_DMA0_CHNCFG_BASE,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK0].schedCtrlBase      = (Ptr) AVALANCHE_NWSS_DMA0_SCHEDCFG_BASE,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK0].schedTableBase     = (Ptr) AVALANCHE_NWSS_DMA0_SCHEDTBL_BASE,

    .dmaBlock[PAL_CPPI41_DMA_BLOCK0].tdFQueue.qMgr      = PAL_CPPI41_QUEUE_MGR_PARTITION_SR,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK0].tdFQueue.qNum      = PAL_CPPI41_SR_DMA_FD_TEARDOWN_Q_NUM,

    .dmaBlock[PAL_CPPI41_DMA_BLOCK1].globalCtrlBase     = (Ptr) AVALANCHE_NWSS_DMA1_GBLCFG_BASE,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK1].chCtrlStatusBase   = (Ptr) AVALANCHE_NWSS_DMA1_CHNCFG_BASE,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK1].schedCtrlBase      = (Ptr) AVALANCHE_NWSS_DMA1_SCHEDCFG_BASE,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK1].schedTableBase     = (Ptr) AVALANCHE_NWSS_DMA1_SCHEDTBL_BASE,

    .dmaBlock[PAL_CPPI41_DMA_BLOCK1].tdFQueue.qMgr      = PAL_CPPI41_QUEUE_MGR_PARTITION_SR,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK1].tdFQueue.qNum      = PAL_CPPI41_SR_DMA_FD_TEARDOWN_Q_NUM,

    .dmaBlock[PAL_CPPI41_DMA_BLOCK2].globalCtrlBase     = (Ptr) AVALANCHE_NWSS_DMA2_GBLCFG_BASE,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK2].chCtrlStatusBase   = (Ptr) AVALANCHE_NWSS_DMA2_CHNCFG_BASE,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK2].schedCtrlBase      = (Ptr) AVALANCHE_NWSS_DMA2_SCHEDCFG_BASE,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK2].schedTableBase     = (Ptr) AVALANCHE_NWSS_DMA2_SCHEDTBL_BASE,

    .dmaBlock[PAL_CPPI41_DMA_BLOCK2].tdFQueue.qMgr      = PAL_CPPI41_QUEUE_MGR_PARTITION_SR,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK2].tdFQueue.qNum      = PAL_CPPI41_SR_DMA_FD_TEARDOWN_Q_NUM,

    .dmaBlock[PAL_CPPI41_DMA_BLOCK3].globalCtrlBase     = (Ptr) AVALANCHE_NWSS_DMA3_GBLCFG_BASE,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK3].chCtrlStatusBase   = (Ptr) AVALANCHE_NWSS_DMA3_CHNCFG_BASE,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK3].schedCtrlBase      = (Ptr) AVALANCHE_NWSS_DMA3_SCHEDCFG_BASE,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK3].schedTableBase     = (Ptr) AVALANCHE_NWSS_DMA3_SCHEDTBL_BASE,

    .dmaBlock[PAL_CPPI41_DMA_BLOCK3].tdFQueue.qMgr      = PAL_CPPI41_QUEUE_MGR_PARTITION_SR,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK3].tdFQueue.qNum      = PAL_CPPI41_SR_DMA_FD_TEARDOWN_Q_NUM,


    .dmaBlock[PAL_CPPI41_DMA_BLOCK0].schedTable.numEntries  = 4,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK0].schedTable.entry =
    {
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, PAL_CPPI41_SR_DOCSIS_DS_CoP_LOW0_DMA01_RX_CH_NUM  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, PAL_CPPI41_SR_DOCSIS_DS_CoP_LOW1_DMA01_RX_CH_NUM  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, PAL_CPPI41_SR_DOCSIS_DS_CoP_HIGH_DMA01_RX_CH_NUM  ),

        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, PAL_CPPI41_SR_DOCSIS_DS_CoP_MGMT_DMA01_RX_CH_NUM  ),
    },

    .dmaBlock[PAL_CPPI41_DMA_BLOCK1].schedTable.numEntries  = 4,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK1].schedTable.entry =
    {
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, PAL_CPPI41_SR_DOCSIS_DS_CoP_LOW0_DMA01_RX_CH_NUM  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, PAL_CPPI41_SR_DOCSIS_DS_CoP_LOW1_DMA01_RX_CH_NUM  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, PAL_CPPI41_SR_DOCSIS_DS_CoP_HIGH_DMA01_RX_CH_NUM  ),

        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, PAL_CPPI41_SR_DOCSIS_DS_CoP_MGMT_DMA01_RX_CH_NUM  ),
    },

    .dmaBlock[PAL_CPPI41_DMA_BLOCK2].schedTable.numEntries  = 10,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK2].schedTable.entry =
    {
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, PAL_CPPI41_SR_DOCSIS_TX_US0_DMA2_TX_CH_NUM  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, PAL_CPPI41_SR_DOCSIS_TX_US1_DMA2_TX_CH_NUM  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, PAL_CPPI41_SR_DOCSIS_TX_US2_DMA2_TX_CH_NUM  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, PAL_CPPI41_SR_DOCSIS_TX_US3_DMA2_TX_CH_NUM  ),

        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_RX_CH, PAL_CPPI41_SR_DOCSIS_TX_COP_DMA_RX_CH_NUM  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, PAL_CPPI41_SR_DOCSIS_TX_COP_DMA_TX_CH_NUM  ),

        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, PAL_CPPI41_SR_CNI_LOW_INFRA_DMA2_TX_CH_NUM  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, PAL_CPPI41_SR_CNI_HIGH_INFRA_DMA2_TX_CH_NUM  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, PAL_CPPI41_SR_L2SW_DATA0_INFRA_DMA2_TX_CH_NUM  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, PAL_CPPI41_SR_L2SW_MGMT0_INFRA_DMA2_TX_CH_NUM  ),
    },

    .dmaBlock[PAL_CPPI41_DMA_BLOCK3].schedTable.numEntries  = 7,
    .dmaBlock[PAL_CPPI41_DMA_BLOCK3].schedTable.entry =
    {
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, PAL_CPPI41_SR_DOCSIS_TX_US4_DMA3_TX_CH_NUM  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, PAL_CPPI41_SR_DOCSIS_TX_US5_DMA3_TX_CH_NUM  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, PAL_CPPI41_SR_DOCSIS_TX_US6_DMA3_TX_CH_NUM  ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, PAL_CPPI41_SR_DOCSIS_TX_US7_DMA3_TX_CH_NUM  ),

        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, PAL_CPPI41_SR_HOST_TO_PP_LOW_INFRA_DMA3_TX_CH_NUM   ),
        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, PAL_CPPI41_SR_HOST_TO_PP_HIGH_INFRA_DMA3_TX_CH_NUM   ),

        PAL_CPPI41_DMA_CH_CONFIG( PAL_CPPI41_DMA_TX_CH, PAL_CPPI41_SR_RECYCLING_INFRA_DMA3_TX_CH_NUM ),
    },
    .apdspInfo.pdspCmdBase      = (Ptr)     AVALANCHE_NWSS_APDSP_CMD_BASE,

    .debugToolBind = puma6_cppi_proc_init,
};



/* proc interface  */
static PAL_Handle puma6palHnd;

typedef struct FDqueue
{
    PAL_CPPI41_SR_QMGR_QUEUES_e     id;
    unsigned int                    amount;
}
FDqueue_t;

FDqueue_t gFDqueues[] =
{
    {   .id = PAL_CPPI41_SR_PPDSP_PREFETCH_DS_DESC_FD_Q_NUM,    .amount = PAL_CPPI41_SR_PPDSP_PREFETCH_DS_DESC_FD_DESC_COUNT    },
    {   .id = PAL_CPPI41_SR_PPDSP_PREFETCH_US_DESC_FD_Q_NUM,    .amount = PAL_CPPI41_SR_PPDSP_PREFETCH_US_DESC_FD_DESC_COUNT    },
    {   .id = PAL_CPPI41_SR_PPDSP_PREFETCH_BUFF_FD_Q_NUM,       .amount = PAL_CPPI41_SR_PPDSP_PREFETCH_BUFF_FD_DESC_COUNT       },
    {   .id = PAL_CPPI41_SR_PP_EVENTS_FD_Q_NUM,                 .amount = PAL_CPPI41_SR_PP_EVENTS_FD_DESC_COUNT                 },
    {   .id = PAL_CPPI41_SR_HOST_TO_PP_LOW_FD_HOST_Q_NUM,       .amount = PAL_CPPI41_SR_HOST_TO_PP_LOW_FD_HOST_DESC_COUNT       },
    {   .id = PAL_CPPI41_SR_HOST_TO_PP_HIGH_FD_HOST_Q_NUM,      .amount = PAL_CPPI41_SR_HOST_TO_PP_HIGH_FD_HOST_DESC_COUNT      },
    {   .id = PAL_CPPI41_SR_L2SW_INFRA_FD_HOST_Q_NUM,           .amount = PAL_CPPI41_SR_L2SW_INFRA_FD_HOST_DESC_COUNT           },
    {   .id = PAL_CPPI41_SR_DOCSIS_RX_CoP_LOW_FD_EMB_Q_NUM,     .amount = PAL_CPPI41_SR_DOCSIS_RX_CoP_LOW_FD_EMB_DESC_COUNT     },
    {   .id = PAL_CPPI41_SR_DOCSIS_RX_CoP_HIGH_FD_EMB_Q_NUM,    .amount = PAL_CPPI41_SR_DOCSIS_RX_CoP_HIGH_FD_EMB_DESC_COUNT    },
    {   .id = PAL_CPPI41_SR_HOST_TO_PP_INFRA_LOW_FD_EMB_Q_NUM,  .amount = PAL_CPPI41_SR_HOST_TO_PP_INFRA_LOW_FD_EMB_DESC_COUNT  },
    {   .id = PAL_CPPI41_SR_HOST_TO_PP_INFRA_HIGH_FD_EMB_Q_NUM, .amount = PAL_CPPI41_SR_HOST_TO_PP_INFRA_HIGH_FD_EMB_DESC_COUNT },
    {   .id = PAL_CPPI41_SR_PROXY_PDSP_LOW_FD_EMB_Q_NUM,        .amount = PAL_CPPI41_SR_PROXY_PDSP_LOW_FD_EMB_DESC_COUNT        },
    {   .id = PAL_CPPI41_SR_PROXY_PDSP_MED_LOW_FD_EMB_Q_NUM,    .amount = PAL_CPPI41_SR_PROXY_PDSP_MED_LOW_FD_EMB_DESC_COUNT    },
    {   .id = PAL_CPPI41_SR_PROXY_PDSP_MED_HIGH_FD_EMB_Q_NUM,   .amount = PAL_CPPI41_SR_PROXY_PDSP_MED_HIGH_FD_EMB_DESC_COUNT   },
    {   .id = PAL_CPPI41_SR_PROXY_PDSP_HIGH_FD_EMB_Q_NUM,       .amount = PAL_CPPI41_SR_PROXY_PDSP_HIGH_FD_EMB_DESC_COUNT       },
    {   .id = PAL_CPPI41_SR_DMA_FD_TEARDOWN_Q_NUM,              .amount = PAL_CPPI41_NUM_TD_DESC * 4                            },
    {   .id = PAL_CPPI41_SR_DOCSIS_TX_MONOLITHIC_Q_NUM,         .amount = 16    },
    {   .id = PAL_CPPI41_SR_CNI_INFRA_LOW_FD_HOST_Q_NUM,        .amount = PAL_CPPI41_SR_CNI_INFRA_LOW_FD_HOST_DESC_COUNT        },
    {   .id = PAL_CPPI41_SR_CNI_INFRA_HIGH_FD_HOST_Q_NUM,       .amount = PAL_CPPI41_SR_CNI_INFRA_HIGH_FD_HOST_DESC_COUNT       },
    {   .id = PAL_CPPI41_SR_DOCSIS_MGMT_RX_FD_HOST_Q_NUM,       .amount = PAL_CPPI41_SR_DOCSIS_MGMT_RX_FD_HOST_DESC_COUNT       },
    {   .id = PAL_CPPI41_SR_DOCSIS_MGMT_TX_FD_HOST_Q_NUM,       .amount = PAL_CPPI41_SR_DOCSIS_MGMT_TX_FD_HOST_DESC_COUNT       },
    {   .id = PAL_CPPI41_SR_MPEG_FD_EMB_Q_NUM,                  .amount = DMAC_MPEG_RX_EMBEDDED_BD_NUM                          },
    {   .id = PAL_CPPI41_SR_MPEG_ENCAP_FD_EMB_Q_NUM,            .amount = DMAC_MPEG_ENCAP_RX_EMBEDDED_BD_NUM                    },

};



#define PUMA6_PROC_FS_BUFF_SZ   (512*128)
static char puma6_proc_fs_buffer[ PUMA6_PROC_FS_BUFF_SZ ];

PAL_CPPI41_SR_QMGR_QUEUES_STR(qname);

int puma6_cppi_sr_dump_all_stats(char* buf, char **start, off_t offset, int count, int *eof, void *data)
{
    int queue = 0;
    int len = 0;
    int i;
    unsigned int pktCount;
    unsigned int expectedCount;
    Cppi4Queue  cppiQueue;
    static   int buff_size;

    if (0 == offset)
    {
        for (queue = 0; queue < PAL_CPPI41_SR_QMGR_TOTAL_Q_COUNT; queue++)
        {
            cppiQueue.qMgr = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
            cppiQueue.qNum = queue ;

            PAL_cppi4Control(puma6palHnd, PAL_CPPI41_IOCTL_GET_QUEUE_ENTRY_COUNT,   &cppiQueue, &pktCount);

            expectedCount = 0;

            for (i=0; i<ARRAY_SIZE(gFDqueues); i++)
            {
                if (gFDqueues[i].id == queue)
                {
                    expectedCount = gFDqueues[i].amount;
                    break;
                }
            }

            if (pktCount != expectedCount)
            {
                len += sprintf (&puma6_proc_fs_buffer[len], "%4d %-65s : %4d [%d]\n",
                                queue, qname[queue], pktCount, expectedCount);
            }

            if (len + 128 > PUMA6_PROC_FS_BUFF_SZ)
            {
                sprintf(&puma6_proc_fs_buffer[len - 6],"\n...\n");
                break ;
            }
        }

        buff_size = len;
    }
    else
    {
        len = buff_size - offset;
    }

    if (len > count)
    {
        len = count;
    }

    memcpy(buf, &puma6_proc_fs_buffer[offset], len);

    return len;
}


static PAL_Result puma6_cppi_proc_init (Ptr hnd, Ptr param)
{
    struct proc_dir_entry * dir_1 = (struct proc_dir_entry *)param;

    puma6palHnd = (PAL_Handle)hnd;

    if (NULL == (dir_1 = proc_mkdir("sr",       dir_1))) {   printk("%s:%d ERROR ....\n",__FUNCTION__,__LINE__); return -1;  }
    if (NULL == (dir_1 = proc_mkdir("stats",    dir_1))) {   printk("%s:%d ERROR ....\n",__FUNCTION__,__LINE__); return -1;  }
    if (NULL == (create_proc_read_entry( "all" , 0, dir_1, puma6_cppi_sr_dump_all_stats, NULL ))) {   printk("%s:%d ERROR ....\n",__FUNCTION__,__LINE__); return -1;  }

    return (0);
}



int avalanche_cppi_init(void)
{
    if(PAL_cppi4Init(&cppi4InitCfg_g, NULL) != NULL)
    {
        printk("PAL_cppi4Init: CPPI 4.1 API initialized successfully.\n");
    }
    else
    {
        printk("PAL_cppi4Init: ERROR: CPPI 4.1 API initialization failed!\n");
    }

    return 0;
}

