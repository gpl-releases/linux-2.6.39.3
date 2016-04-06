/*
 * puma6_pp.c
 *  Description:
 *  See below.
 *
 *
 */

/* Puma-6 Packet Processor initialization.
 * Contains Puma-6 specific initialization. The numbers (addresses etc) are
 * Puma-6 specific. The static structures are filled in with Puma-6 specific
 * data and the generic PPD init function gets called in the end
 * with this data.
 */

#include <pal.h>
#include <pal_cppi41.h>
#include <pal_cppi41pvt.h>
#include <linux/netdevice.h>
#include <linux/proc_fs.h>
#include <unistd.h>
#include <ti_ppd.h>
#include <puma6_pp.h>
#include <puma6_cppi.h>

/* On-Chip descriptor management
 * IMPORTANT NOTES:
 * ===============
 *  -   Following macros control how the on-chip descriptor memory is devided
 *  into different descriptors. All the base addresses must be varefully
 *  determined not to avoid overlap.
 *  -   Also note that all the On-cip descriptors *must* be of size equal to the
 *  specified in puma6_cppi.c cppi config structure for respective region.
 */
#define PREFETCH_DESC_BASEADDR      AVALANCHE_NWSS_ONCHIPDESC_BASE

PrefchCfg prefCfg_g =
{
    /* Prefetcher Free Descriptors */
    .pfDescBase     = (Ptr) IO_VIRT2PHY( PREFETCH_DESC_BASEADDR ),
    /* DS */
    .pfDsDescCnt    = PAL_CPPI41_SR_PPDSP_PREFETCH_DS_DESC_FD_DESC_COUNT,
    .pfDsFQ.qMgr    = PAL_CPPI41_QUEUE_MGR_PARTITION_SR,
    .pfDsFQ.qNum    = PAL_CPPI41_SR_PPDSP_PREFETCH_DS_DESC_FD_Q_NUM,
    /* US */
    .pfUsDescCnt    = PAL_CPPI41_SR_PPDSP_PREFETCH_US_DESC_FD_DESC_COUNT,
    .pfUsFQ.qMgr    = PAL_CPPI41_QUEUE_MGR_PARTITION_SR,
    .pfUsFQ.qNum    = PAL_CPPI41_SR_PPDSP_PREFETCH_US_DESC_FD_Q_NUM,

    /* Prefetcher Free Buffers */
    .pfBlkBase      = (Ptr) IO_VIRT2PHY( AVALANCHE_NWSS_APDSP_PREFBLK_BASE ),
    .pfBlkCnt       = PAL_CPPI41_SR_PPDSP_PREFETCH_BUFF_FD_DESC_COUNT,
    .pfFBQ.qMgr     = PAL_CPPI41_QUEUE_MGR_PARTITION_SR,
    .pfFBQ.qNum     = PAL_CPPI41_SR_PPDSP_PREFETCH_BUFF_FD_Q_NUM,

    .pfDataSize     = PREFETCH_DATA_SIZE,
    .pfDataOffset   = PREFETCH_DATA_OFFSET,
    .pfCmdBase      = (Ptr) AVALANCHE_NWSS_APDSP_CMD_BASE
};

typedef struct
{
    Uint32  numDesc;
    Uint32  qNum;

    Ptr     firstDescPtr;
    Uint32  isAllocated;

    /* Desc config for the  group */
    Uint32 pktType;
} BDBlkInfo;


/* Embedded descriptor table:
 *  - This table aggregates the embedded descriptors in the system. It contains
 *  information about the first descriptor and allocation status of each
 *  division of the region. Generally this division is based on per driver or
 *  per Q basis.
 */
typedef struct
{
    Ptr         buffDescRegionPtr;
    Uint32      qMgr;
    Uint32      numDesc;
    Uint32      szDesc;
    Uint32      numBlks;
    BDBlkInfo   BDBlk [PAL_CPPI41_SR_FD_EMB_Q_LAST - PAL_CPPI41_SR_FD_EMB_Q_BASE];
} EmbBDCfg;

EmbBDCfg ppEmbBDCfg_g =
{
    .qMgr       = PAL_CPPI41_QUEUE_MGR_PARTITION_SR,
    .numDesc    = PAL_CPPI41_SR_PP_EVENTS_FD_DESC_COUNT +
                  PAL_CPPI41_SR_HOST_TO_PP_INFRA_LOW_FD_EMB_DESC_COUNT +
                  PAL_CPPI41_SR_HOST_TO_PP_INFRA_HIGH_FD_EMB_DESC_COUNT +
                  PAL_CPPI41_SR_DOCSIS_RX_CoP_LOW_FD_EMB_DESC_COUNT +
                  PAL_CPPI41_SR_DOCSIS_RX_CoP_HIGH_FD_EMB_DESC_COUNT + 
                  PAL_CPPI41_SR_PROXY_PDSP_LOW_FD_EMB_DESC_COUNT + 
                  PAL_CPPI41_SR_PROXY_PDSP_MED_LOW_FD_EMB_DESC_COUNT +
                  PAL_CPPI41_SR_PROXY_PDSP_MED_HIGH_FD_EMB_DESC_COUNT + 
                  PAL_CPPI41_SR_PROXY_PDSP_HIGH_FD_EMB_DESC_COUNT,

                      /* Must be same as in Cppi Cgf structure    */
    .szDesc     = 64,   /* Must be same as for specific region      */
    .numBlks    = 9,    /* Change for each block added/removed      */

    /* PP2Host Events */
    .BDBlk[0].numDesc   = PAL_CPPI41_SR_PP_EVENTS_FD_DESC_COUNT,
    .BDBlk[0].qNum      = PAL_CPPI41_SR_PP_EVENTS_FD_Q_NUM,
    .BDBlk[0].pktType   = PAL_CPPI4_HOSTDESC_PKT_TYPE_ETH,

    /* Host2PP Infra 0 */
    .BDBlk[1].numDesc   = PAL_CPPI41_SR_HOST_TO_PP_INFRA_LOW_FD_EMB_DESC_COUNT,
    .BDBlk[1].qNum      = PAL_CPPI41_SR_HOST_TO_PP_INFRA_LOW_FD_EMB_Q_NUM,
    .BDBlk[1].pktType   = PAL_CPPI4_HOSTDESC_PKT_TYPE_ETH,

    /* Host2PP Infra 1 */
    .BDBlk[2].numDesc   = PAL_CPPI41_SR_HOST_TO_PP_INFRA_HIGH_FD_EMB_DESC_COUNT,
    .BDBlk[2].qNum      = PAL_CPPI41_SR_HOST_TO_PP_INFRA_HIGH_FD_EMB_Q_NUM,
    .BDBlk[2].pktType   = PAL_CPPI4_HOSTDESC_PKT_TYPE_ETH,

    /* DOCSIS RX CoP */
    .BDBlk[3].numDesc   = PAL_CPPI41_SR_DOCSIS_RX_CoP_LOW_FD_EMB_DESC_COUNT,
    .BDBlk[3].qNum      = PAL_CPPI41_SR_DOCSIS_RX_CoP_LOW_FD_EMB_Q_NUM,
    .BDBlk[3].pktType   = PAL_CPPI4_HOSTDESC_PKT_TYPE_ETH,
    .BDBlk[4].numDesc   = PAL_CPPI41_SR_DOCSIS_RX_CoP_HIGH_FD_EMB_DESC_COUNT,
    .BDBlk[4].qNum      = PAL_CPPI41_SR_DOCSIS_RX_CoP_HIGH_FD_EMB_Q_NUM,
    .BDBlk[4].pktType   = PAL_CPPI4_HOSTDESC_PKT_TYPE_ETH,

    /* Proxy PDSP RX */
    .BDBlk[5].numDesc   = PAL_CPPI41_SR_PROXY_PDSP_LOW_FD_EMB_DESC_COUNT,
    .BDBlk[5].qNum      = PAL_CPPI41_SR_PROXY_PDSP_LOW_FD_EMB_Q_NUM,
    .BDBlk[5].pktType   = PAL_CPPI4_HOSTDESC_PKT_TYPE_ETH,
    .BDBlk[6].numDesc   = PAL_CPPI41_SR_PROXY_PDSP_MED_LOW_FD_EMB_DESC_COUNT,
    .BDBlk[6].qNum      = PAL_CPPI41_SR_PROXY_PDSP_MED_LOW_FD_EMB_Q_NUM,
    .BDBlk[6].pktType   = PAL_CPPI4_HOSTDESC_PKT_TYPE_ETH,
    .BDBlk[7].numDesc   = PAL_CPPI41_SR_PROXY_PDSP_MED_HIGH_FD_EMB_DESC_COUNT,
    .BDBlk[7].qNum      = PAL_CPPI41_SR_PROXY_PDSP_MED_HIGH_FD_EMB_Q_NUM,
    .BDBlk[7].pktType   = PAL_CPPI4_HOSTDESC_PKT_TYPE_ETH,
    .BDBlk[8].numDesc   = PAL_CPPI41_SR_PROXY_PDSP_HIGH_FD_EMB_DESC_COUNT,
    .BDBlk[8].qNum      = PAL_CPPI41_SR_PROXY_PDSP_HIGH_FD_EMB_Q_NUM,
    .BDBlk[8].pktType   = PAL_CPPI4_HOSTDESC_PKT_TYPE_ETH,
};

typedef struct 
{
    Uint32  buffSize;
    Uint32  buffCount;
    Uint32  refCount;
}PAL_CPPI41_SR_BUFF_INFO_t;

PAL_CPPI41_SR_BUFF_INFO_t ppBuffPoolInfo[PAL_CPPI41_BMGR_MAX_POOLS] = 
{
    {.buffSize = BMGR0_POOL00_BUF_SIZE, .buffCount = BMGR0_POOL00_BUF_COUNT, .refCount = BMGR0_POOL00_REF_CNT},
    {.buffSize = BMGR0_POOL01_BUF_SIZE, .buffCount = BMGR0_POOL01_BUF_COUNT, .refCount = BMGR0_POOL01_REF_CNT},
    {.buffSize = BMGR0_POOL02_BUF_SIZE, .buffCount = BMGR0_POOL02_BUF_COUNT, .refCount = BMGR0_POOL02_REF_CNT},
    {.buffSize = BMGR0_POOL03_BUF_SIZE, .buffCount = BMGR0_POOL03_BUF_COUNT, .refCount = BMGR0_POOL03_REF_CNT},
    {.buffSize = BMGR0_POOL04_BUF_SIZE, .buffCount = BMGR0_POOL04_BUF_COUNT, .refCount = BMGR0_POOL04_REF_CNT},
    {.buffSize = BMGR0_POOL05_BUF_SIZE, .buffCount = BMGR0_POOL05_BUF_COUNT, .refCount = BMGR0_POOL05_REF_CNT},
    {.buffSize = BMGR0_POOL06_BUF_SIZE, .buffCount = BMGR0_POOL06_BUF_COUNT, .refCount = BMGR0_POOL06_REF_CNT},
    {.buffSize = BMGR0_POOL07_BUF_SIZE, .buffCount = BMGR0_POOL07_BUF_COUNT, .refCount = BMGR0_POOL07_REF_CNT},
    {.buffSize = BMGR0_POOL08_BUF_SIZE, .buffCount = BMGR0_POOL08_BUF_COUNT, .refCount = BMGR0_POOL08_REF_CNT},
    {.buffSize = BMGR0_POOL09_BUF_SIZE, .buffCount = BMGR0_POOL09_BUF_COUNT, .refCount = BMGR0_POOL09_REF_CNT},
    {.buffSize = BMGR0_POOL10_BUF_SIZE, .buffCount = BMGR0_POOL10_BUF_COUNT, .refCount = BMGR0_POOL10_REF_CNT},
    {.buffSize = BMGR0_POOL11_BUF_SIZE, .buffCount = BMGR0_POOL11_BUF_COUNT, .refCount = BMGR0_POOL11_REF_CNT},
    {.buffSize = BMGR0_POOL12_BUF_SIZE, .buffCount = BMGR0_POOL12_BUF_COUNT, .refCount = BMGR0_POOL12_REF_CNT},
    {.buffSize = BMGR0_POOL13_BUF_SIZE, .buffCount = BMGR0_POOL13_BUF_COUNT, .refCount = BMGR0_POOL13_REF_CNT},
    {.buffSize = 0,                     .buffCount = 0,                      .refCount = 0},
    {.buffSize = 0,                     .buffCount = 0,                      .refCount = 0},
    {.buffSize = 0,                     .buffCount = 0,                      .refCount = 0},
    {.buffSize = 0,                     .buffCount = 0,                      .refCount = 0}
};

#ifdef P6_PP_DEBUG
/* note: prints function name for you */
#  define DPRINTK(fmt, args...) printk("%s: " fmt, __FUNCTION__ , ## args)
#else
#  define DPRINTK(fmt, args...)
#endif

#define PAL_CPPI41_ACC_MAX_PAGE_ENTRIES                32
#define PAL_CPPI41_ACC_LIST_NULL_TERM                  0
#define PAL_CPPI41_ACC_PACE_MODE_LASTINTR              1
#define PAL_CPPI41_ACC_PACE_TICK_CNT                   40
#define PAL_CPPI41_ACC_MAX_PAGE_COUNT                  2

struct tasklet_struct   gTxCompleteTasklet;     /* Tx completion processing tasklet */
PAL_Cppi4AccChHnd       gTxCompleteAccChHnd[PAL_CPPI41_TX_COMPLETE_ACC_CH_COUNT];
Ptr                     gTxCompleteAccListBase[PAL_CPPI41_TX_COMPLETE_ACC_CH_COUNT];
Cppi4HostDescLinux**    gTxCompleteAccList[PAL_CPPI41_TX_COMPLETE_ACC_CH_COUNT];
PAL_Cppi4QueueHnd       gHost2ppFreeHostDescQueueHnd[PAL_CPPI41_SR_HOST_TO_PP_Q_COUNT];


/**************************************************************************/
/*! \fn static void setupRecycleInfra (PAL_Handle hnd)
 **************************************************************************
 *  \brief Setup infrastructure DMA for recycling use resources.
 *  \return none.
 **************************************************************************/
static void setupRecycleInfra (PAL_Handle hnd)
{
    Cppi4TxChInitCfg txCh;
    Cppi4RxChInitCfg rxCh;
    PAL_Cppi4TxChHnd cppi4TxChHnd;
    PAL_Cppi4RxChHnd cppi4RxChHnd;
    Cppi4Queue tmpQ;

    tmpQ.qMgr = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
    tmpQ.qNum = PAL_CPPI41_SR_RECYCLE_FD_HOST_Q_NUM;
    PAL_cppi4QueueOpen (hnd, tmpQ);

    /* Set up Rx channel */
    rxCh.chNum              = PAL_CPPI41_SR_RECYCLING_INFRA_DMA3_RX_CH_NUM;
    rxCh.dmaNum             = PAL_CPPI41_DMA_BLOCK3;
    rxCh.sopOffset          = 0;
    rxCh.retryOnStarvation  = 0;
    rxCh.rxCompQueue.qMgr   = 0;
    rxCh.rxCompQueue.qNum   = 0;

    rxCh.defDescType        = CPPI41_DESC_TYPE_EMBEDDED;
    rxCh.u.embeddedPktCfg.fdQueue.qMgr = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
    rxCh.u.embeddedPktCfg.fdQueue.qNum = PAL_CPPI41_SR_RECYCLE_FD_HOST_Q_NUM;
    rxCh.u.embeddedPktCfg.numBufSlot = (EMSLOTCNT-1);
    rxCh.u.embeddedPktCfg.sopSlotNum = 0;
    rxCh.u.embeddedPktCfg.fBufPool[0].bMgr  = BUF_POOL_MGR0;
    rxCh.u.embeddedPktCfg.fBufPool[0].bPool = 0;
    rxCh.u.embeddedPktCfg.fBufPool[1].bMgr  = BUF_POOL_MGR0;
    rxCh.u.embeddedPktCfg.fBufPool[1].bPool = 0;
    rxCh.u.embeddedPktCfg.fBufPool[2].bMgr  = BUF_POOL_MGR0;
    rxCh.u.embeddedPktCfg.fBufPool[2].bPool = 0;
    rxCh.u.embeddedPktCfg.fBufPool[3].bMgr  = BUF_POOL_MGR0;
    rxCh.u.embeddedPktCfg.fBufPool[3].bPool = 0;

    cppi4RxChHnd        = PAL_cppi4RxChOpen(hnd, &rxCh, NULL);

    /* Set up Tx channel */
    txCh.chNum          = PAL_CPPI41_SR_RECYCLING_INFRA_DMA3_TX_CH_NUM;
    txCh.dmaNum         = PAL_CPPI41_DMA_BLOCK3;
    txCh.tdQueue.qMgr   = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
    txCh.tdQueue.qNum   = PAL_CPPI41_SR_DMA_FD_TEARDOWN_Q_NUM;

    cppi4TxChHnd        = PAL_cppi4TxChOpen(hnd, &txCh, NULL);

    if (!cppi4TxChHnd || !cppi4RxChHnd)
    {
        printk ("%s: infra channel setup failed for channel %d\n", __FUNCTION__, PAL_CPPI41_SR_RECYCLING_INFRA_DMA3_TX_CH_NUM);
        return;
    }

    /* Enable Tx-Rx channels */
    PAL_cppi4EnableRxChannel (cppi4RxChHnd, NULL);
    PAL_cppi4EnableTxChannel (cppi4TxChHnd, NULL);

    return;
}


/* Embedded descriptor table:
 *  - This table aggregates the embedded descriptors in the system. It contains
 *  information about the first descriptor and allocation status of each
 *  division of the region. Generally this division is based on per driver or
 *  per Q basis.
 */
typedef struct
{
    Ptr         buffDescRegionPtr;
    Uint32      qMgr;
    Uint32      numDesc;
    Uint32      szDesc;
    Uint32      numBlks;
    BDBlkInfo   BDBlk [PAL_CPPI41_SR_FD_HOST_Q_LAST - PAL_CPPI41_SR_FD_HOST_Q_BASE];
} HostBDCfg;


HostBDCfg ppHostBDCfg_g =
{
    .qMgr       = PAL_CPPI41_QUEUE_MGR_PARTITION_SR,
    .numDesc    = PAL_CPPI41_SR_HOST_TO_PP_FD_HOST_DESC_COUNT,    /* Must be same as in Cppi Cgf structure */
    .szDesc     = PAL_CPPI41_SR_HOST_TO_PP_FD_HOST_DESC_SIZE,   /* Must be same as for specific region   */
    .numBlks    = 2,                                            /* Change for each block added/removed   */

    /* Host2PP Low */
    .BDBlk[0].numDesc   = PAL_CPPI41_SR_HOST_TO_PP_LOW_FD_HOST_DESC_COUNT,
    .BDBlk[0].qNum      = PAL_CPPI41_SR_HOST_TO_PP_LOW_FD_HOST_Q_NUM,
    .BDBlk[0].pktType   = PAL_CPPI4_HOSTDESC_PKT_TYPE_ETH,

    /* Host2PP High */
    .BDBlk[1].numDesc   = PAL_CPPI41_SR_HOST_TO_PP_HIGH_FD_HOST_DESC_COUNT,
    .BDBlk[1].qNum      = PAL_CPPI41_SR_HOST_TO_PP_HIGH_FD_HOST_Q_NUM, //PPFW_CPPI4x_FD_QMGR
    .BDBlk[1].pktType   = PAL_CPPI4_HOSTDESC_PKT_TYPE_ETH,
};


void setup_host2pp_infra(PAL_Handle hnd, int infra_chan)
{
    Cppi4TxChInitCfg txCh;
    Cppi4RxChInitCfg rxCh;
    PAL_Cppi4TxChHnd cppi4TxChHnd;
    PAL_Cppi4RxChHnd cppi4RxChHnd;

    /* Set up Rx channel */
    rxCh.chNum              = PAL_CPPI41_SR_HOST_TO_PP_INFRA_DMA_CH_NUM(infra_chan);
    rxCh.dmaNum             = PAL_CPPI41_DMA_BLOCK3;
    rxCh.defDescType        = CPPI41_DESC_TYPE_EMBEDDED;
    rxCh.sopOffset          = 0;
    rxCh.rxCompQueue.qMgr   = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
    rxCh.rxCompQueue.qNum   = PAL_CPPI41_SR_HOST_TO_QPDSP_EMB_TYPE_Q_NUM(infra_chan);
    rxCh.retryOnStarvation  = 0;

    rxCh.u.embeddedPktCfg.fdQueue.qMgr = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
    rxCh.u.embeddedPktCfg.fdQueue.qNum = PAL_CPPI41_SR_HOST_TO_PP_INFRA_FD_EMB_Q_NUM(infra_chan);
    rxCh.u.embeddedPktCfg.numBufSlot = (EMSLOTCNT-1);
    rxCh.u.embeddedPktCfg.sopSlotNum = 0;
    rxCh.u.embeddedPktCfg.fBufPool[0].bMgr  = BUF_POOL_MGR0;
    rxCh.u.embeddedPktCfg.fBufPool[0].bPool = PAL_CPPI41_BMGR_POOL6;
    rxCh.u.embeddedPktCfg.fBufPool[1].bMgr  = BUF_POOL_MGR0;
    rxCh.u.embeddedPktCfg.fBufPool[1].bPool = PAL_CPPI41_BMGR_POOL7;
    rxCh.u.embeddedPktCfg.fBufPool[2].bMgr  = BUF_POOL_MGR0;
    rxCh.u.embeddedPktCfg.fBufPool[2].bPool = PAL_CPPI41_BMGR_POOL7;
    rxCh.u.embeddedPktCfg.fBufPool[3].bMgr  = BUF_POOL_MGR0;
    rxCh.u.embeddedPktCfg.fBufPool[3].bPool = PAL_CPPI41_BMGR_POOL7;
    cppi4RxChHnd        = PAL_cppi4RxChOpen(hnd, &rxCh, NULL);

    /* Set up Tx channel */
    txCh.chNum          = PAL_CPPI41_SR_HOST_TO_PP_INFRA_DMA_CH_NUM(infra_chan);
    txCh.dmaNum         = PAL_CPPI41_DMA_BLOCK3;
    txCh.tdQueue.qMgr   = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
    txCh.tdQueue.qNum   = PAL_CPPI41_SR_DMA_FD_TEARDOWN_Q_NUM;

    cppi4TxChHnd        = PAL_cppi4TxChOpen(hnd, &txCh, NULL);

    if (!cppi4TxChHnd || !cppi4RxChHnd)
    {
        printk ("%s: infra channel setup failed for channel %d\n", __FUNCTION__, infra_chan);
        return;
    }

    /* Enable Tx-Rx channels */
    PAL_cppi4EnableRxChannel (cppi4RxChHnd, NULL);
    PAL_cppi4EnableTxChannel (cppi4TxChHnd, NULL);

    return;
}

static void do_tx_complete(unsigned long data)
{
    Cppi4HostDescLinux* hostDesc;
    Uint32      packets_processed = 0;
    Int32       priority;

    /* Start with high priority channel */
    for (priority = PAL_CPPI41_TX_COMPLETE_ACC_CH_COUNT - 1; priority >= 0; priority--)
    {
        /* While there are ready pages... */
        while(avalanche_intd_get_interrupt_count(0, PAL_CPPI41_TX_COMPLETE_ACC_CH_NUM(priority)))
        {
            /* While there are descriptors in the page... */
            while((hostDesc = (Cppi4HostDescLinux*)((unsigned long)*gTxCompleteAccList[priority] & QMGR_QUEUE_N_REG_D_DESC_ADDR_MASK)))
            {
                hostDesc = PAL_CPPI4_PHYS_2_VIRT(hostDesc);
                PAL_CPPI4_CACHE_INVALIDATE(hostDesc, PAL_CPPI41_SR_HOST_TO_PP_FD_HOST_DESC_SIZE);
    
                dev_kfree_skb_any(hostDesc->skb);
                hostDesc->skb = NULL;
    
                /* Queue back the hostDesc to free pool */
                PAL_cppi4QueuePush(gHost2ppFreeHostDescQueueHnd[priority] , (Ptr)PAL_CPPI4_VIRT_2_PHYS(hostDesc), PAL_CPPI4_DESCSIZE_2_QMGRSIZE(PAL_CPPI41_SR_HOST_TO_PP_FD_HOST_DESC_SIZE), 0);
    
                packets_processed++;
                gTxCompleteAccList[priority]++;
            }
    
            /* Update the list entry for next time */
            gTxCompleteAccList[priority] = PAL_cppi4AccChGetNextList(gTxCompleteAccChHnd[priority]);
    
            /* Decrement number of pages by 1 */
            avalanche_intd_set_interrupt_count(0, PAL_CPPI41_TX_COMPLETE_ACC_CH_NUM(priority), 1);
        }
    }

    /* First clear the IRQ in order not to get a false interrupt since INTD is level */
    ack_irq(MAP_INTD_TO_INTC(PAL_CPPI41_TX_COMPLETE_ACC_INTV_NUM));
    
    /* Send INTD EOI */
    avalanche_intd_write_eoi(PAL_CPPI41_TX_COMPLETE_ACC_INTV_NUM);

    /* It could be that between INTD count decrement and EOI the accumulator will issue another interrupt. 
       The logic of INTD is such that level will remain active high even after EOI is set, so INTC will 
       lose the interrupt after ack_irq is done (it now expects INTD polarity change). 
       Therefore we must check INTD count and if it is not 0 - reschedule the tasklet */
    for (priority = PAL_CPPI41_TX_COMPLETE_ACC_CH_COUNT - 1; priority >= 0; priority--)
    {
        if (avalanche_intd_get_interrupt_count(0, PAL_CPPI41_TX_COMPLETE_ACC_CH_NUM(priority)))
        {
            tasklet_schedule(&gTxCompleteTasklet);
			return;
        }
    }

    /* Now enable the IRQ */
    enable_irq(MAP_INTD_TO_INTC(PAL_CPPI41_TX_COMPLETE_ACC_INTV_NUM));
}

static int init_acc_chan(PAL_Handle pal_hnd, int chan_num, Cppi4Queue queue, PAL_Cppi4AccChHnd* acc_hnd)
{
    Cppi4AccumulatorCfg cfg;
    unsigned int accListSize;

    *acc_hnd = NULL;

    cfg.accChanNum             = chan_num;
    cfg.list.maxPageEntry      = PAL_CPPI41_ACC_MAX_PAGE_ENTRIES;   /* This is entries per page (and we have 2 pages) */
    cfg.list.listEntrySize     = PAL_CPPI41_ACC_ENTRY_TYPE_D;   /* Only interested in register 'D' which has the desc pointer */
    cfg.list.listCountMode     = PAL_CPPI41_ACC_LIST_NULL_TERM;     /* Zero indicates null terminated list. */
    cfg.list.pacingMode        = PAL_CPPI41_ACC_PACE_MODE_LASTINTR; /* Wait for time since last interrupt */
    cfg.pacingTickCnt          = PAL_CPPI41_ACC_PACE_TICK_CNT;      /* Wait for 1000uS == 1ms */
    cfg.list.maxPageCnt        = PAL_CPPI41_ACC_MAX_PAGE_COUNT;     /* Use two pages */
    cfg.list.stallAvoidance    = 1;                             /* Use the stall avoidance feature */
    cfg.queue                  = queue;
    cfg.mode                   = 0;

    accListSize = (cfg.list.maxPageEntry * (cfg.list.listEntrySize + 1)) * cfg.list.maxPageCnt * sizeof(Uint32);
    if(!(cfg.list.listBase = kzalloc(accListSize, GFP_KERNEL)))
    {
        DPRINTK("Unable to allocate list page of size %d\n", accListSize);
        return -1;
    }

    PAL_CPPI4_CACHE_WRITEBACK((unsigned long)cfg.list.listBase, accListSize);

    cfg.list.listBase = (Ptr)PAL_CPPI4_VIRT_2_PHYS((Ptr)cfg.list.listBase);

    if(!(*acc_hnd = PAL_cppi4AccChOpen(pal_hnd, &cfg)))
    {
        DPRINTK("Unable to open accumulator channel #%d\n", chan_num);
        kfree(cfg.list.listBase);
        return -1;
    }

    return 0;
}

irqreturn_t tx_complete_interrupt(int irq, void *dev)
{
    /* Since the INTD interrupts are level, need to disable the IRQ in order to run the tasklet */
    disable_irq_nosync(MAP_INTD_TO_INTC(PAL_CPPI41_TX_COMPLETE_ACC_INTV_NUM));

    tasklet_schedule(&gTxCompleteTasklet);

    return IRQ_RETVAL(1);
}



static Int32 setup_txcomplete(PAL_Handle palHnd)
{
    Cppi4Queue  txCmplQ;
    Cppi4Queue  fdHostQ;
    Uint8       priority;

    for (priority = 0; priority < PAL_CPPI41_SR_HOST_TO_PP_Q_COUNT; priority++)
    {
        /************************************************/
        /* reset Tx complete queue                      */
        /************************************************/
        txCmplQ.qMgr = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
        txCmplQ.qNum = PAL_CPPI41_SR_HOST_TX_COMPLETE_Q_NUM(priority);
        PAL_cppi4QueueClose(palHnd, PAL_cppi4QueueOpen(palHnd, txCmplQ));

        fdHostQ.qMgr = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
        fdHostQ.qNum = PAL_CPPI41_SR_HOST_TO_PP_FD_HOST_Q_NUM(priority);

        if(!(gHost2ppFreeHostDescQueueHnd[priority] = PAL_cppi4QueueOpen(palHnd, fdHostQ)))
        {
            printk("unable to open FD Host Queue #%d for TX Complete task\n", fdHostQ.qNum);
            return -1;
        }

        /************************************************/
        /* Init the Tx complete accumulator channel     */
        /************************************************/
        if(init_acc_chan(palHnd, PAL_CPPI41_TX_COMPLETE_ACC_CH_NUM(priority), txCmplQ, &gTxCompleteAccChHnd[priority]))
        {
            printk("unable to open accumulator channel #%d for TX Complete task\n", PAL_CPPI41_TX_COMPLETE_ACC_CH_NUM(priority));
            return -1;
        }

        gTxCompleteAccListBase[priority] = gTxCompleteAccList[priority] = PAL_cppi4AccChGetNextList(gTxCompleteAccChHnd[priority]);

        /* request the Tx Complete IRQs - one IRQ per all TX complete priorities */
        if (priority == PAL_CPPI4x_PRTY_LOW)
        {
            tasklet_init(&gTxCompleteTasklet, do_tx_complete, 0);

            if(request_irq(MAP_INTD_TO_INTC(PAL_CPPI41_TX_COMPLETE_ACC_INTV_NUM), tx_complete_interrupt, IRQF_DISABLED, "TX Complete", NULL))
            {
                printk("unable to get IRQ #%d for TX Complete task\n", MAP_INTD_TO_INTC(PAL_CPPI41_TX_COMPLETE_ACC_INTV_NUM));
                return -1;
            }
        }
    }

    return 0;
}

/**************************************************************************
 * FUNCTION NAME : pp_test_usage
 **************************************************************************
 * DESCRIPTION   :
 *  Prints the pp_test proc usage
 *
 * RETURNS       :
 *  Nonne
 ***************************************************************************/
void pp_test_usage(void)
{
    printk("\nCommand Usage:\n");
    printk("send pktString[hex byte string with : as delimiter] pktSize[64-1518,pktSize>=length(pktString)] pktNum srcPort[0-%d] dstQueue[0-%d] recyclePrxQ[0,1]\n", PAL_CPPI41_MAX_SOURCE_PORTS-1, PAL_CPPI41_SR_QMGR_TOTAL_Q_COUNT);
}

static int pp_test(struct file *file, const char *buffer, unsigned long count, void *data)
{
    char    proc_cmd[2000];
    char*   argv[10];
    int     argc = 0;
    char*   ptr_cmd;
    char*   delimitters = " \n\t";
    char*   ptr_next_tok;

    Uint32 pktSize;
    Uint32 pktNum, pktNumIndex;
    Uint32 srcPort;
    Uint32 dstQueue;
    Uint32 doRecycle;
    Uint32 pktStringLength;
    Uint32 tmp;
    PAL_Handle hnd = PAL_cppi4Init(NULL, NULL);
    Cppi4Queue queue;
    PAL_Cppi4QueueHnd popQueueHnd, dstQueueHnd;
    PAL_Cppi4QueueHnd prxQHnd;
    PAL_Cppi4QueueHnd recycleQHnd;
    Cppi4EmbdDesc* desc = NULL;
    Ptr dataBuffer;
    char *c;
    const char * sep = ":";


    Uint8 pktData[1518] = // Set with default packet. If pktSize > length(pktString) then it will be padded by this packet content
    {
        0x00, 0x1B, 0x21, 0x3C, 0xD9, 0x5E, // ETH DA
        0x00, 0x50, 0xF1, 0x80, 0x00, 0x00, // ETH SA
        0x08, 0x00,                         // ETH Type
        0x45, 0x00,                         // IP Version/Header Length, IP TOS
        0x00, 0x2E,                         // IP Total Length - updated per packet
        0x00, 0x00, 0x00, 0x00,             // IP Identification, IP Fragment
        0x40,                               // IP TTL
        0x11,                               // IP Protocol - UDP
        0x9A, 0x2F,                         // IP Checksum
        0x0A, 0x64, 0x66, 0x64,             // IP SA - 10.100.102.100
        0x0A, 0x64, 0x65, 0x64,             // IP DA - 10.100.101.100
        0x03, 0xE8, 0x03, 0xE9,             // UDP SRC Port (1000), UDP DST Port (1001)
        0x00, 0x1A,                         // UDP Length
        0x00, 0x00,                         // UDP Checksum
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, // Payload  
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, // Payload  
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, // Payload  
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, // Payload  
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, // Payload  
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, // Payload  
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, // Payload  
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, // Payload  
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, // Payload  
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, // Payload  
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, // Payload  
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, // Payload  
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, // Payload  
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05, // Payload  
        0x00, 0x01, 0x02, 0x03, 0x04, 0x05  // Payload  
    };
    
    /* Validate the length of data passed. */
    if (count > 2000)
    {
        printk("\nERROR: Command exceeded maximum length\n");
        return -EFAULT;
    }

    /* Initialize the buffer before using it. */
    memset ((void *)&proc_cmd[0], 0, sizeof(proc_cmd));
    memset ((void *)&argv[0], 0, sizeof(argv));

    /* Copy from user space. */
    if (copy_from_user(&proc_cmd, buffer, count))
    {
        pp_test_usage();
        return -EFAULT;
    }

    ptr_next_tok = &proc_cmd[0];

    /* Tokenize the command. Check if there was a NULL entry. If so be the case the user did not know how to use the entry. Print the help screen */
    ptr_cmd = strsep(&ptr_next_tok, delimitters);
    if (ptr_cmd == NULL)
    {
        pp_test_usage();
        return -EFAULT;
    }

    /* Parse all the commands typed. */
    do
    {
        /* Extract the first command. */
        argv[argc++] = ptr_cmd;

        /* Validate if the user entered more commands.*/
        if (argc >= 10)
        {
            pp_test_usage();
            return -EFAULT;
        }

        /* Get the next valid command. */
        ptr_cmd = strsep(&ptr_next_tok, delimitters);
    } while (ptr_cmd != NULL);

    /* We have an extra argument when strsep is used instead of strtok */
    argc--;

    /******************************* Command Handlers *******************************/

    /* ds <pktNum> : Test DS packets */
    if (strncmp(argv[0], "send", strlen("send")) == 0)
    {
        if (!hnd)
        {
            printk("PAL_cppi4Init failed\n");
            return -EFAULT;
        }

        if (7 != argc)
        {
            pp_test_usage();
            return -EFAULT;
        }

        pktSize   = (int) simple_strtol(argv[2], NULL, 0);
        pktNum    = (int) simple_strtol(argv[3], NULL, 0);
        srcPort   = (int) simple_strtol(argv[4], NULL, 0);
        dstQueue  = (int) simple_strtol(argv[5], NULL, 0);
        doRecycle = (int) simple_strtol(argv[6], NULL, 0);

        pktStringLength = 0;
        while ((c = strsep(&argv[1], sep))) 
        {
            pktData[pktStringLength++] = simple_strtol(c, NULL, 16);

            if (pktStringLength >= 1518)
            {
                pp_test_usage();
                return -EFAULT;
            }
        }

        if (pktStringLength > pktSize || pktSize < 64 || pktSize > 1518 || srcPort >= PAL_CPPI41_MAX_SOURCE_PORTS || dstQueue >= PAL_CPPI41_SR_QMGR_TOTAL_Q_COUNT || doRecycle > 1)
        {
            pp_test_usage();
            return -EFAULT;
        }

        queue.qMgr = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
        queue.qNum = PAL_CPPI41_SR_DOCSIS_RX_CoP_LOW_FD_EMB_Q_NUM;
        popQueueHnd = PAL_cppi4QueueOpen(hnd, queue);

        queue.qNum = dstQueue;
        dstQueueHnd = PAL_cppi4QueueOpen(hnd, queue);

        for (pktNumIndex = 0; pktNumIndex < pktNum; pktNumIndex++)
        {
            /* Get descriptor from free queue */
            

            desc = (Cppi4EmbdDesc*)PAL_cppi4QueuePop(popQueueHnd);
            if (!desc)
            {
                printk("Pop descriptor from queue PAL_CPPI41_SR_DOCSIS_RX_CoP_LOW_FD_EMB_Q_NUM[#%d] failed\n", PAL_CPPI41_SR_DOCSIS_RX_CoP_LOW_FD_EMB_Q_NUM);
                return -EFAULT;
            }
            desc = (Cppi4EmbdDesc*)PAL_CPPI4_PHYS_2_VIRT(desc);

            /* Update Descriptor Info */
            desc->descInfo = 0x18400000 + pktSize; 
            desc->tagInfo = (srcPort << 27) | 0x3FFF;
            
            /* Update Descriptor's Buffer Info */
            desc->Buf[0].BufInfo = 0;                                                       // Slot 0 is empty
            desc->Buf[0].BufPtr = 0;                                                        // Slot 0 is empty

            /* Allocate buffer, set it and put in descriptor */
            {
                /* Get Buffer from pool 0 */
                dataBuffer = PAL_cppi4BufPopBuf(hnd, (Cppi4BufPool){PAL_CPPI41_QUEUE_MGR_PARTITION_SR, PAL_CPPI41_BMGR_POOL0});
                while (!dataBuffer)
                {
                    dataBuffer = PAL_cppi4BufPopBuf(hnd, (Cppi4BufPool){PAL_CPPI41_QUEUE_MGR_PARTITION_SR, PAL_CPPI41_BMGR_POOL0});
                }
                dataBuffer = PAL_CPPI4_PHYS_2_VIRT(dataBuffer);

                /* Update Buffer Data */
                for (tmp = 0; (tmp < pktSize) && (tmp < BMGR0_POOL00_BUF_SIZE); tmp+=4)
                {
                    *(volatile Uint32*)((Uint8*)dataBuffer + tmp) = (pktData[tmp] << 24) | (pktData[tmp+1] << 16) | (pktData[tmp+2] << 8) | (pktData[tmp+3] << 0);
                }
                
                /* Update buffer in descritptor */            
                desc->Buf[1].BufInfo = 0x80000000 + (PAL_CPPI41_BMGR_POOL0 << 24) + tmp;    // Slot 1 buffer info
                desc->Buf[1].BufPtr = PAL_CPPI4_VIRT_2_PHYS(dataBuffer);                           // Slot 1 buffer pointer
                desc->pktInfo = 0x1C108000 + PAL_CPPI41_SR_DOCSIS_RX_CoP_LOW_FD_EMB_Q_NUM;

                PAL_CPPI4_CACHE_WRITEBACK((unsigned long)dataBuffer, tmp);
            }

            /* Check if another buffer is needed */
            if (tmp < pktSize)
            {
                /* Get Buffer from pool 0 */
                dataBuffer = PAL_cppi4BufPopBuf(hnd, (Cppi4BufPool){PAL_CPPI41_QUEUE_MGR_PARTITION_SR, PAL_CPPI41_BMGR_POOL0});
                while (!dataBuffer)
                {
                    dataBuffer = PAL_cppi4BufPopBuf(hnd, (Cppi4BufPool){PAL_CPPI41_QUEUE_MGR_PARTITION_SR, PAL_CPPI41_BMGR_POOL0});
                }
                dataBuffer = PAL_CPPI4_PHYS_2_VIRT(dataBuffer);

                /* Update Buffer Data */
                for (; (tmp < pktSize) && (tmp < 2 * BMGR0_POOL00_BUF_SIZE); tmp+=4)
                {
                    *(volatile Uint32*)((Uint8*)dataBuffer + (tmp - BMGR0_POOL00_BUF_SIZE)) = (pktData[tmp] << 24) | (pktData[tmp+1] << 16) | (pktData[tmp+2] << 8) | (pktData[tmp+3] << 0);
                }
                
                /* Update buffer in descritptor */            
                desc->Buf[2].BufInfo = 0x80000000 + (PAL_CPPI41_BMGR_POOL0 << 24) + tmp - BMGR0_POOL00_BUF_SIZE;     // Slot 2 buffer info
                desc->Buf[2].BufPtr = PAL_CPPI4_VIRT_2_PHYS(dataBuffer);                                                    // Slot 2 buffer pointer
                desc->pktInfo = 0x1C208000 + PAL_CPPI41_SR_DOCSIS_RX_CoP_LOW_FD_EMB_Q_NUM;

                PAL_CPPI4_CACHE_WRITEBACK((unsigned long)dataBuffer, tmp - BMGR0_POOL00_BUF_SIZE);
            }
            else
            {
                desc->Buf[2].BufInfo = 0;                                                       // Slot 2 is empty
                desc->Buf[2].BufPtr = 0;                                                        // Slot 2 is empty
            }

            /* Check if another buffer is needed */
            if (tmp < pktSize)
            {
                /* Get Buffer from pool 4 */
                dataBuffer = PAL_cppi4BufPopBuf(hnd, (Cppi4BufPool){PAL_CPPI41_QUEUE_MGR_PARTITION_SR, PAL_CPPI41_BMGR_POOL4});
                while (!dataBuffer)
                {
                    dataBuffer = PAL_cppi4BufPopBuf(hnd, (Cppi4BufPool){PAL_CPPI41_QUEUE_MGR_PARTITION_SR, PAL_CPPI41_BMGR_POOL4});
                }
                dataBuffer = PAL_CPPI4_PHYS_2_VIRT(dataBuffer);

                /* Update Buffer Data */
                for (; (tmp < pktSize); tmp+=4)
                {
                    *(volatile Uint32*)((Uint8*)dataBuffer + (tmp - (2*BMGR0_POOL00_BUF_SIZE))) = (pktData[tmp] << 24) | (pktData[tmp+1] << 16) | (pktData[tmp+2] << 8) | (pktData[tmp+3] << 0);
                }
                
                /* Update buffer in descritptor */            
                desc->Buf[3].BufInfo = 0x80000000 + (PAL_CPPI41_BMGR_POOL4 << 24) + tmp - (2*BMGR0_POOL00_BUF_SIZE); // Slot 3 buffer info
                desc->Buf[3].BufPtr = PAL_CPPI4_VIRT_2_PHYS(dataBuffer);                                                    // Slot 3 buffer pointer
                desc->pktInfo = 0x1C308000 + PAL_CPPI41_SR_DOCSIS_RX_CoP_LOW_FD_EMB_Q_NUM;

                PAL_CPPI4_CACHE_WRITEBACK((unsigned long)dataBuffer, tmp - (2*BMGR0_POOL00_BUF_SIZE));
            }
            else
            {
                desc->Buf[3].BufInfo = 0;                                                       // Slot 3 is empty
                desc->Buf[3].BufPtr = 0;                                                        // Slot 3 is empty
            }

            
            PAL_CPPI4_CACHE_WRITEBACK((unsigned long)desc, 64);

            /* Push the packet to the dstQueue */
            PAL_cppi4QueuePush(dstQueueHnd, (Ptr)PAL_CPPI4_VIRT_2_PHYS((Uint32)desc), PAL_CPPI4_DESCSIZE_2_QMGRSIZE(64), pktSize);
        }

        if (doRecycle)
        {
            /* Clear PrxPDSP queues */
            queue.qNum = PAL_CPPI41_SR_DMA3_RECYCLE_INFRA_INPUT_LOW_Q_NUM;
            recycleQHnd = PAL_cppi4QueueOpen(hnd, queue);
    
            queue.qNum = PAL_CPPI41_SR_PrxPDSP_Q_BASE;
            prxQHnd = PAL_cppi4QueueOpen(hnd, queue);
            do
            {
                desc = (Cppi4EmbdDesc*)PAL_cppi4QueuePop(prxQHnd);
                if (desc != NULL)
                {
                    PAL_cppi4QueuePush(recycleQHnd, (Ptr)desc, PAL_CPPI4_DESCSIZE_2_QMGRSIZE(64), 0);
                }
            }
            while (desc != NULL);
        }
    }
    else
    {
        pp_test_usage();
        return -EFAULT;
    }

    return count;
}


extern int ti_pp_sys_initialize (void);

int avalanche_ppd_init(void)
{
    int i;
    Cppi4Queue tmpQ;    /* Just used for filling Q info for opening */
    Cppi4BufPool tmpBufPool; /* Used for Init calls */
    Uint32 pRcbMem;
    PAL_Handle hnd, preFQHnd, preFBQHnd;
    int reg;

    /* Set clock to PP peripherals (Only those that are used) */
    reg = AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_PPDSP             |
          AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_CPDSP1            |
          // AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_CPDSP2         |
          AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_MPDSP             |
          AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_QPDSP             |
          AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_PrxPDSP           |
          // AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_UsPrefPDSP     |
          // AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_CoePDSP        |
          AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_PrefSharedRAM     |
          AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_SessSharedRAM     |
          AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_MiscSharedRam     |
          AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_Counters          |
          AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_CDMA0             |
          AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_CDMA1             |
          AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_CDMA2             |
          AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_CDMA3             |
          AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_McDMA0            |
          AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_McDMA1            |
          // AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_McDMA2         |
          AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_McDMA3            |
          AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_McDMA4            |
          AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_LUT1;           //|
          // AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_LUT2           |
          // AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_LUT3);
    *(Uint32*)AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_REG = reg;

    /* Initialize the PP Subsystem. */
    if (ti_pp_sys_initialize () < 0)
    {
        printk ("Error: Failed to initialize the PP Subsystem\n");
        return -1;
    }
    else
    {
        printk("avalanche_ppd_init: PP system  initialized successfully.\n");
    }
    hnd = PAL_cppi4Init (NULL, NULL);

    if (!hnd)
    {
        printk("avalanche_ppd_init: CPPI41 Library NOT initialized.\n");
        return -1;
    }



    /************************************************************************/
    /*********** Init buffer pools used in the system ***********************/
    /************************************************************************/

    tmpBufPool.bMgr     = BUF_POOL_MGR0;

    for (i = PAL_CPPI41_BMGR_POOL0; i < PAL_CPPI41_BMGR_MAX_POOLS; i++)
    {
        if (ppBuffPoolInfo[i].buffCount != 0)
        {
            tmpBufPool.bPool = i;
            if ((PAL_cppi4BufPoolInit(hnd, tmpBufPool, ppBuffPoolInfo[i].refCount, ppBuffPoolInfo[i].buffSize, ppBuffPoolInfo[i].buffCount)) == NULL)
            {
                printk ("PAL_cppi4BufPoolInit for pool %d FAILED.\n", tmpBufPool.bPool);
                return -1;
            }
        }
    }
    /******************** Buffer pool Init done *****************************/



    /************************************************************************/
    /*********** Setup Free Embedded descriptors    *************************/
    /************************************************************************/

    /************************************************/
    /*      Allocate region                         */
    /*                                              */
    ppEmbBDCfg_g.buffDescRegionPtr =
        PAL_cppi4AllocDesc( hnd,ppEmbBDCfg_g.qMgr,
                                ppEmbBDCfg_g.numDesc,
                                ppEmbBDCfg_g.szDesc );

    if (!ppEmbBDCfg_g.buffDescRegionPtr)
    {
        printk ("Embedded descriptor region allocation FAILED.\n");
        return -1;
    }
    /************************************************/

    {
        Cppi4EmbdDesc* currBD;
        currBD = (Cppi4EmbdDesc*)ppEmbBDCfg_g.buffDescRegionPtr;

        for (i = 0; i < ppEmbBDCfg_g.numBlks; i++)
        {
            BDBlkInfo* BDBlk = &ppEmbBDCfg_g.BDBlk[i];
            int bd_cnt;
            PAL_Cppi4QueueHnd tmpQHnd;

            tmpQ.qMgr = ppEmbBDCfg_g.qMgr;
            tmpQ.qNum = BDBlk->qNum;
            tmpQHnd = PAL_cppi4QueueOpen (hnd, tmpQ);

            for (bd_cnt = 0; bd_cnt < BDBlk->numDesc; bd_cnt++)
            {
                PAL_osMemSet(currBD, 0, ppEmbBDCfg_g.szDesc);

                currBD->descInfo    = CPPI41_EM_DESCINFO_DTYPE_EMBEDDED |
                                      CPPI41_EM_DESCINFO_SLOTCNT_MYCNT;
                currBD->tagInfo     = 0x3FFF;
                currBD->pktInfo     =
                                     (BDBlk->pktType    << CPPI41_EM_PKTINFO_PKTTYPE_SHIFT)
                                    |(1                 << CPPI41_EM_PKTINFO_RETPOLICY_SHIFT)
                                    |(1                 << CPPI41_EM_PKTINFO_PROTSPEC_SHIFT)
                                    |(ppEmbBDCfg_g.qMgr << PAL_CPPI4_HOSTDESC_PKT_RETQMGR_SHIFT)
                                    |(BDBlk->qNum       << PAL_CPPI4_HOSTDESC_PKT_RETQNUM_SHIFT);

                PAL_CPPI4_CACHE_WRITEBACK(currBD, ppEmbBDCfg_g.szDesc);

                PAL_cppi4QueuePush (tmpQHnd,
                                    (Ptr)PAL_CPPI4_VIRT_2_PHYS((Uint32)currBD),
                                    (ppEmbBDCfg_g.szDesc-24)/4,
                                    0/*!@@*/);

                currBD = (Cppi4EmbdDesc*)((Uint32)currBD + ppEmbBDCfg_g.szDesc);
            }
        }
    }
    /********************** Free Embedded desc setup Done  ******************/


    /************************************************************************/
    /*********** Setup Free Host descriptors        *************************/
    /************************************************************************/

    /************************************************/
    /*      Allocate region                         */
    /*                                              */
    ppHostBDCfg_g.buffDescRegionPtr =
        PAL_cppi4AllocDesc( hnd,ppHostBDCfg_g.qMgr,
        ppHostBDCfg_g.numDesc,
        ppHostBDCfg_g.szDesc );

    if (!ppHostBDCfg_g.buffDescRegionPtr)
    {
        printk ("Host descriptor region allocation FAILED.\n");
        return -1;
    }
    /************************************************/

    {
        Cppi4HostDescLinux* currBD;

        currBD = (Cppi4HostDescLinux*)ppHostBDCfg_g.buffDescRegionPtr;

        for (i = 0; i < ppHostBDCfg_g.numBlks; i++)
        {
            BDBlkInfo* BDBlk = &ppHostBDCfg_g.BDBlk[i];
            int bd_cnt;
            PAL_Cppi4QueueHnd tmpQHnd;

            tmpQ.qMgr = ppHostBDCfg_g.qMgr;
            tmpQ.qNum = BDBlk->qNum;
            tmpQHnd = PAL_cppi4QueueOpen (hnd, tmpQ);

            for (bd_cnt = 0; bd_cnt < BDBlk->numDesc; bd_cnt++)
            {
                PAL_osMemSet(currBD, 0, ppHostBDCfg_g.szDesc);

                currBD->hw.descInfo    = (PAL_CPPI4_HOSTDESC_DESC_TYPE_HOST << PAL_CPPI4_HOSTDESC_DESC_TYPE_SHIFT);
                currBD->hw.tagInfo     = 0x3FFF;
                currBD->hw.pktInfo     =
                         (BDBlk->pktType                        << PAL_CPPI4_HOSTDESC_PKT_TYPE_SHIFT)
                        |(PAL_CPPI4_HOSTDESC_PKT_RETPLCY_LINKED << PAL_CPPI4_HOSTDESC_PKT_RETPLCY_SHIFT)
                        |(PAL_CPPI4_HOSTDESC_DESC_LOC_OFFCHIP   << PAL_CPPI4_HOSTDESC_DESC_LOC_SHIFT)
                        |(ppHostBDCfg_g.qMgr                    << PAL_CPPI4_HOSTDESC_PKT_RETQMGR_SHIFT)
                        |(BDBlk->qNum                           << PAL_CPPI4_HOSTDESC_PKT_RETQNUM_SHIFT);

                PAL_CPPI4_CACHE_WRITEBACK(currBD, ppHostBDCfg_g.szDesc);

                PAL_cppi4QueuePush (tmpQHnd,
                                    (Ptr)PAL_CPPI4_VIRT_2_PHYS((Uint32)currBD),
                                    (ppHostBDCfg_g.szDesc-24)/4,
                                    0/*!@@*/);

                currBD = (Cppi4HostDescLinux*)((Uint32)currBD + ppHostBDCfg_g.szDesc);
            }
        }
    }

    /********************** Free Host desc setup Done  ******************/



    /*********** Open the Queues common for devices in PP system
     * Actually the queues only need to be opened here if the handle is required
     * (for push/pop generally) and any driver depending on these Qs would do
     * so. Still opening here to provide idea of the system.
     * Also, opening these queue here would mean that any subsequent calls (from
     * drivers) to open these queues will just return the same handle to the
     * queue without resetting them
     */
    /************************************************************************/
    /* Open Tx Qs. These will be used by drivers to push for TX Infra Qs */
    tmpQ.qMgr   = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
    for (i = 0; PAL_CPPI41_SR_HOST_TO_PP_INFRA_DMA_CH_COUNT > i; i++)
    {
        /* Input Queue */
        tmpQ.qNum   = PAL_CPPI41_SR_HOST_TO_PP_INFRA_INPUT_Q_NUM(i);
        PAL_cppi4QueueOpen (hnd, tmpQ);

        /* Output Queue */
        tmpQ.qNum   = PAL_CPPI41_SR_HOST_TO_QPDSP_EMB_TYPE_Q_NUM(i);
        PAL_cppi4QueueOpen (hnd, tmpQ);

        /* Infrastructure Channels */
        setup_host2pp_infra(hnd, i);
    }

    tmpQ.qMgr   = PAL_CPPI41_QUEUE_MGR_PARTITION_SR;
    tmpQ.qNum   = PAL_CPPI41_RECYCLE_INFRA_INPUT_LOW_Q_NUM;
    PAL_cppi4QueueOpen(hnd, tmpQ);
    /* This is done in order to keep the reference count of these queues non zero so they will not be closed */
    setupRecycleInfra(hnd);
    /*****************************************************************
     ******************* Setup Prefetcher ***************************
     ****************************************************************/

    /* Push Prefetcher free desc */
    pRcbMem = (Uint32)prefCfg_g.pfDescBase;
    preFQHnd = PAL_cppi4QueueOpen (hnd, prefCfg_g.pfDsFQ);
    for(i = 0; i < prefCfg_g.pfDsDescCnt; i++) 
    {
        PAL_cppi4QueuePush (preFQHnd, (Ptr) pRcbMem, 0, 0);
        pRcbMem += PAL_CPPI41_SR_PPDSP_PREFETCH_DESC_FD_DESC_SIZE;    // Next buffer descriptor
    }
    preFQHnd = PAL_cppi4QueueOpen (hnd, prefCfg_g.pfUsFQ);
    for(i = 0; i < prefCfg_g.pfUsDescCnt; i++) 
    {
        PAL_cppi4QueuePush (preFQHnd, (Ptr) pRcbMem, 0, 0);
        pRcbMem += PAL_CPPI41_SR_PPDSP_PREFETCH_DESC_FD_DESC_SIZE;    // Next buffer descriptor
    }

    /* Setup prefetch buffers */
    pRcbMem = (Uint32)prefCfg_g.pfBlkBase;
    preFBQHnd = PAL_cppi4QueueOpen (hnd, prefCfg_g.pfFBQ);
    for( i=0; i<prefCfg_g.pfBlkCnt; i++ )
    {
        PAL_cppi4QueuePush (preFBQHnd, (Ptr) pRcbMem, 0, 0);
        pRcbMem += 128;                  // Next buffer descriptor
    }
    return 0;
}
int avalanche_prefetcher_init (void)
{
    int i;
    PAL_Handle hnd;
    APDSP_Command_Buffer_RegsOvly apdsp;
    hnd = PAL_cppi4Init (NULL, NULL);
    apdsp = prefCfg_g.pfCmdBase;

    //--------------------------------------------------------
    // Config Prefetcher
    //
    // Setup prefetch to fetch 96 bytes with a 24 byte offset
    // NOTE: Use 128 byte fetch with no offset on USB
    //
    //apdsp->Parameter0 = (24<<8) | 96;
    apdsp->Parameter0 = (0<<8) | 128;
    apdsp->Command = ACMD_COMMAND(ACMD_CONFIG_PREFETCH);

    for( i=0; i<1000000 && ACMD_GET_COMMAND(apdsp->Command); i++);
    if( i==1000000 ) 
    {
        printk("%s(%d): Error - APDSP firmware not responding!\n", __FUNCTION__, __LINE__);
        return -1;
    }

    if (ACMD_GET_RETCODE(apdsp->Command) != 1)
    {
        printk("%s(%d):return code 0x%02x\n", __FUNCTION__, __LINE__, ACMD_GET_RETCODE(apdsp->Command));
    }

    //--------------------------------------------------------
    //
    // Setup Teardown descriptor queue
    // Note:  The prefetcher is configured to route the teardown
    // descriptors back to free teardown desc queue. This avoids prefetcher
    // going into invalid state when TD is received.
    // CHI: __IMPORTANT__ This configuration constraints ALL PP related Endpoint
    // Rx channels to be configured to use SAME Teardown Queue otherwise TDs
    // will be lost (since the queue number specified in DMA configuration to
    // pick the TD and Queue number for Prefetcher to queue the TD will be
    // different).
    //
    apdsp->Parameter0 = (PAL_CPPI41_QUEUE_MGR_PARTITION_SR<<16) | PAL_CPPI41_SR_DMA_FD_TEARDOWN_Q_NUM;
    apdsp->Command = ACMD_COMMAND(ACMD_CONFIG_TDQ);

    for( i=0; i<1000000 && ACMD_GET_COMMAND(apdsp->Command); i++);
    if( i==1000000 ) 
    {
        printk("%s(%d): Error - APDSP firmware not responding!\n", __FUNCTION__, __LINE__);
        return -1;
    }

    if (ACMD_GET_RETCODE(apdsp->Command) != 1)
    {
        printk("%s(%d):return code 0x%02x\n", __FUNCTION__, __LINE__, ACMD_GET_RETCODE(apdsp->Command));
    }


    //--------------------------------------------------------
    // Enable Prefetcher
    //
    // Enable prefetcher
    //
    apdsp->Command = ACMD_INDEX(1) | ACMD_COMMAND(ACMD_ENABLE_PREFETCH);

    for( i=0; i<1000000 && ACMD_GET_COMMAND(apdsp->Command); i++);
    if( i==1000000 ) 
    {
        printk("%s(%d): Error - APDSP firmware not responding!\n", __FUNCTION__, __LINE__);
        return -1;
    }

    if (ACMD_GET_RETCODE(apdsp->Command) != 1)
    {
            printk("%s(%d):return code 0x%02x\n", __FUNCTION__, __LINE__, ACMD_GET_RETCODE(apdsp->Command));
    }
    /*********** Prefetcher setup done **************************/
    if (setup_txcomplete(hnd))
    {
        printk("%s(%d): Error - setup_txcomplete failed!\n", __FUNCTION__, __LINE__);
        return -1;
    }

    {
        struct proc_dir_entry *pp_test_proc = create_proc_entry("pp_test" ,0644, init_net.proc_net);
    
        if(pp_test_proc)
        {
            pp_test_proc->data = NULL;
            pp_test_proc->write_proc = pp_test;
            pp_test_proc->read_proc  = NULL;
        }
        else
        {
            printk("%s(%d): Error - pp_test proc creation failed\n", __FUNCTION__, __LINE__);
            return -1;
        }
    }

    return 0;
}

int avalanche_ppd_deinit (void)
{
    PAL_Handle  hnd;
    Int32       priority;

    hnd = PAL_cppi4Init (NULL, NULL);
    if (!hnd) 
    {
        printk("avalanche_ppd_deinit: CPPI41 Library NOT initialized.\n");
        return -1;
    }

    if (ppEmbBDCfg_g.buffDescRegionPtr)
    {
        PAL_cppi4DeallocDesc (  hnd,
            ppEmbBDCfg_g.qMgr,
            ppEmbBDCfg_g.buffDescRegionPtr );
    }

    if (ppHostBDCfg_g.buffDescRegionPtr)
    {
        PAL_cppi4DeallocDesc (  hnd,
            ppHostBDCfg_g.qMgr,
            ppHostBDCfg_g.buffDescRegionPtr );
    }

    /* de-init the Tx complete accumulator channel */
    for (priority = 0; priority < PAL_CPPI41_TX_COMPLETE_ACC_CH_COUNT; priority++)
    {
        PAL_cppi4AccChClose(gTxCompleteAccChHnd[priority], NULL);
    }
    
    /* disable the Tx IRQs */
    disable_irq(MAP_INTD_TO_INTC(PAL_CPPI41_TX_COMPLETE_ACC_INTV_NUM));

    do_tx_complete(0);

    tasklet_kill(&gTxCompleteTasklet);

    kfree(gTxCompleteAccListBase);

    free_irq(MAP_INTD_TO_INTC(PAL_CPPI41_TX_COMPLETE_ACC_INTV_NUM), NULL);

#ifdef CONFIG_TI_PACKET_PROCESSOR
    ti_ppm_deinitialize ();
#else
    ti_ppd_exit ();
#endif

    return 0;
}

static Int32 pp_pref_exec_stats_cmd (int stats_index)
{
    int i, ret_val;
    APDSP_Command_Buffer_RegsOvly apdsp;

    apdsp = prefCfg_g.pfCmdBase;

    printk ("Writing %#x\n", ACMD_INDEX(stats_index) | ACMD_COMMAND(ACMD_READ_STATISTICS));

    apdsp->Command = ACMD_INDEX(stats_index)
                        | ACMD_COMMAND(ACMD_READ_STATISTICS);

    for( i=0; i<1000000 && ACMD_GET_COMMAND(apdsp->Command); i++);
    if( i==1000000 ) 
    {
        printk("%s(%d): Error - APDSP firmware not responding!\n", __FUNCTION__, __LINE__);
        return -1;
    }

    if ((ret_val = ACMD_GET_RETCODE(apdsp->Command)) != 1)
    {
        printk("%s(%d):return code 0x%02x\n", __FUNCTION__, __LINE__, ACMD_GET_RETCODE(apdsp->Command));
        return -(ret_val);
    }

    return 0;
}

Int32 ti_pp_get_n_clear_pref_stats (TI_PP_PREF_STATS *stats)
{
    int i, ret_val;
    APDSP_Command_Buffer_RegsOvly apdsp;
    apdsp = prefCfg_g.pfCmdBase;

    /* Get and clear Group A stats */
    if (!(ret_val = pp_pref_exec_stats_cmd (0)))
    {
        stats->grp_a_preproc_pkts       = apdsp->Parameter0;
        stats->grp_a_pref_buf_pkts      = apdsp->Parameter1;
        stats->grp_a_pref_descbuff_pkts = apdsp->Parameter2;
        stats->grp_a_desc_starv_cnt     = apdsp->Parameter3;
    }
    else
        return ret_val;

    /* Get and clear Group B stats */
    if (!(ret_val = pp_pref_exec_stats_cmd (1)))
    {
        stats->grp_b_preproc_pkts       = apdsp->Parameter0;
        stats->grp_b_pref_buf_pkts      = apdsp->Parameter1;
        stats->grp_b_pref_descbuff_pkts = apdsp->Parameter2;
        stats->grp_b_desc_starv_cnt     = apdsp->Parameter3;
    }
    else
        return ret_val;

    /* Get and clear input Queue stats */
    for (i = 0; i < 6; i++)
    {
        if (!(ret_val = pp_pref_exec_stats_cmd (2+i)))
        {
            stats->in_q_congst_discards[i]  = apdsp->Parameter0;
            stats->in_q_congst_thrsh[i]     = apdsp->Parameter1;
        }
        else
            return ret_val;
    }

    return 0;
}

Int32 ti_pp_get_pref_stats (TI_PP_PREF_STATS *stats)
{
    int i;
    volatile Uint32* pref_statsblk_base
        = (volatile Uint32*)((Uint32)(AVALANCHE_NWSS_APDSP_PREFBLK_BASE)
            + 0xC20);

    /* Get Group A stats */
    stats->grp_a_preproc_pkts       = *(pref_statsblk_base + 0);
    stats->grp_a_pref_buf_pkts      = *(pref_statsblk_base + 1);
    stats->grp_a_pref_descbuff_pkts = *(pref_statsblk_base + 2);
    stats->grp_a_desc_starv_cnt     = *(pref_statsblk_base + 3);

    /* Get Group B stats */
    stats->grp_b_preproc_pkts       = *(pref_statsblk_base + 4);
    stats->grp_b_pref_buf_pkts      = *(pref_statsblk_base + 5);
    stats->grp_b_pref_descbuff_pkts = *(pref_statsblk_base + 6);
    stats->grp_b_desc_starv_cnt     = *(pref_statsblk_base + 7);


    /* Get input Queue stats */
    for (i = 0; i < 6; i++)
    {
        stats->in_q_congst_discards[i]  = *(pref_statsblk_base + 8 + i);
        stats->in_q_congst_thrsh[i]     = *(pref_statsblk_base + 8 + i);
    }

    return 0;
}

Int32 ti_pp_enable_psm (void)
{
    int i, ret_val;
    int reg;
    int* regAddr;
    APDSP_Command_Buffer_RegsOvly apdsp;
    apdsp = prefCfg_g.pfCmdBase;

    printk("%s: Enable prefetcher PSM mode\n", __FUNCTION__);
    /* Enable prefetcher PSM */
    apdsp->Command = ACMD_INDEX(2) | ACMD_COMMAND(ACMD_ENABLE_PREFETCH);

    /* Check APDSP responsiveness */
    for(i = 0; i < 1000000 && ACMD_GET_COMMAND(apdsp->Command); i++);
    if(i==1000000)
    {
        printk("%s(%d): Error - APDSP firmware not responding!\n", __FUNCTION__, __LINE__);
        return -1;
    }

    ret_val = ACMD_GET_RETCODE(apdsp->Command);

    /* For now, treat IDLE, NON IDLE as success */
    if ((ret_val) && (ret_val != 1) && (ret_val != 8))
    {
        printk("%s(%d):return code 0x%02x\n", __FUNCTION__, __LINE__, ret_val);
        return -(ret_val);
    }

    /*
    * CHK: Either use SETPSM command of SR or use PDSP control API to halt
    * the SR PDSPs
    */

    printk("%s: Enable PPD PSM mode\n", __FUNCTION__);
    i = 1;
    ti_ppd_pdsp_control (0, TI_PP_PDSPCTRL_PSM, (Ptr)&i);

    /* Power down PP PDSPs using clock gating */
    printk("%s: Halt PP PDSPs\n", __FUNCTION__);

    regAddr = AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_REG;
    reg = *regAddr;
    reg &= ~(AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_CPDSP1 |
             AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_LUT1   |
             AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_MPDSP  |
             AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_McDMA3);
    *regAddr = reg;

    return 0;
}

Int32 ti_pp_disable_psm (void)
{
    int i, ret_val;
    int reg;
    int* regAddr;
    APDSP_Command_Buffer_RegsOvly apdsp;
    apdsp = prefCfg_g.pfCmdBase;

    /* Power up PP PDSPs using clock gating */
    printk("%s: Run PP PDSPs\n", __FUNCTION__);
    regAddr = AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_REG;
    reg = *regAddr;
    reg |= (AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_CPDSP1 |
            AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_LUT1   |
            AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_MPDSP  |
            AVALANCHE_NWSS_GENERAL_MAILBOX_CLK_CTRL_McDMA3);
    *regAddr = reg;

    /* Disable prefetcher PSM */
    printk("%s: Disable prefetcher PSM mode\n", __FUNCTION__);
    apdsp->Command = ACMD_INDEX(1) | ACMD_COMMAND(ACMD_ENABLE_PREFETCH);

    for(i = 0; i < 1000000 && ACMD_GET_COMMAND(apdsp->Command); i++);
    if(i==1000000)
    {
        printk("%s(%d): Error - APDSP firmware not responding!\n", __FUNCTION__, __LINE__);
        return -1;
    }

    ret_val = ACMD_GET_RETCODE(apdsp->Command);
    /* For now, treat IDLE, NON IDLE as success */
    if ((ret_val) && (ret_val != 1) && (ret_val != 8))
    {
        printk("%s(%d):return code 0x%02x\n", __FUNCTION__, __LINE__, ret_val);
        return -(ret_val);
    }

    /*
     * CHK: Either use SETPSM command of SR or use PDSP control API to resume
     * the SR PDSPs
     */
    printk("%s: Disable PPD PSM mode\n", __FUNCTION__);
    i = 0;
    ti_ppd_pdsp_control (0, TI_PP_PDSPCTRL_PSM, (Ptr)&i);

    return 0;
}

subsys_initcall(avalanche_ppd_init);
EXPORT_SYMBOL(avalanche_prefetcher_init);
EXPORT_SYMBOL(ti_pp_get_pref_stats);
EXPORT_SYMBOL(ti_pp_get_n_clear_pref_stats);
EXPORT_SYMBOL(ti_pp_enable_psm);
EXPORT_SYMBOL(ti_pp_disable_psm);

/*
 * Exported PPD APIs to facilitate use from modules. Coule have put these in
 * avalanche_misc.c instead of here, but since ti_ppd.h depends on ti_ppm.h and
 * ti_ppm.h is not in standard path, we get build errors wherever pformCfg.h is
 * included.
 */
EXPORT_SYMBOL(ti_ppd_init);
EXPORT_SYMBOL(ti_ppd_exit);
EXPORT_SYMBOL(ti_ppd_create_session);
EXPORT_SYMBOL(ti_ppd_modify_session);
EXPORT_SYMBOL(ti_ppd_get_session_dump);
EXPORT_SYMBOL(ti_ppd_delete_session);
EXPORT_SYMBOL(ti_ppd_config_pid_range);
EXPORT_SYMBOL(ti_ppd_remove_pid_range);
EXPORT_SYMBOL(ti_ppd_create_pid);
EXPORT_SYMBOL(ti_ppd_set_pid_flags);
EXPORT_SYMBOL(ti_ppd_delete_pid);
EXPORT_SYMBOL(ti_ppd_create_vpid);
EXPORT_SYMBOL(ti_ppd_set_vpid_flags);
EXPORT_SYMBOL(ti_ppd_delete_vpid);
EXPORT_SYMBOL(ti_ppd_get_vpid_stats);
EXPORT_SYMBOL(ti_ppd_clear_vpid_stats);
EXPORT_SYMBOL(ti_ppd_get_n_clear_vpid_stats);
EXPORT_SYMBOL(ti_ppd_get_srl_pkt_stats);
EXPORT_SYMBOL(ti_ppd_clear_srl_pkt_stats);
EXPORT_SYMBOL(ti_ppd_get_n_clear_srl_pkt_stats);
EXPORT_SYMBOL(ti_ppd_get_session_pkt_stats);
EXPORT_SYMBOL(ti_ppd_clear_session_pkt_stats);
EXPORT_SYMBOL(ti_ppd_get_n_clear_session_pkt_stats);
EXPORT_SYMBOL(ti_ppd_register_event_handler);
EXPORT_SYMBOL(ti_ppd_deregister_event_handler);
EXPORT_SYMBOL(ti_ppd_health_check);
EXPORT_SYMBOL(ti_ppd_qos_cluster_setup);
EXPORT_SYMBOL(ti_ppd_qos_cluster_enable);
EXPORT_SYMBOL(ti_ppd_qos_cluster_disable);
EXPORT_SYMBOL(ti_ppd_get_qos_q_stats);
EXPORT_SYMBOL(ti_ppd_get_n_clear_qos_q_stats);
EXPORT_SYMBOL(ti_ppd_get_pdsp_status);
EXPORT_SYMBOL(ti_ppd_pdsp_control);
EXPORT_SYMBOL(ti_ppd_get_ses_age);

