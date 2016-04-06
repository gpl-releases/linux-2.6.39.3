/*
 * puma5_cppi_prv.h
 * Description:
 * See below.
 *
 *
 * Copyright (C) 2009, Texas Instruments, Incorporated
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

/*
 * File containing hadr wire CPPI configurations for each driver.
 * Put into a single file to (hopefully) avoid configuration
 * clashes.
 *
 */

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/*************************** Global parameters **********************/
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/** Not configuration, hardware defined. **/

#define PAL_CPPI41_SR_PP_EVENTS_FD_Q_NUM                117

/* Strictly according to hardware
 * Total 16 Queues.
 */
#define PAL_CPPI41_FBD_Q_BASE                           128
#define PAL_CPPI41_FBD_Q_LAST                           143

/* Strictly according to hardware
 * Total 16 Queues.
 */
#define PAL_CPPI41_FD_Q_BASE                            144
#define PAL_CPPI41_FD_Q_LAST                            159

/* Hardware has only 4 queues.
 * 6 queues required to using unassigned queues
 * from 122-127
 * Document change required.
 */
#define PAL_CPPI41_TX_CMPL_Q_BASE                       122
#define PAL_CPPI41_TX_CMPL_Q_LAST                       127

/* Strictly according to hardware
 * Total 16 Queues.
 */
#define PAL_CPPI41_RX_Q_BASE                            100
#define PAL_CPPI41_RX_Q_LAST                            115

/* Strictly according to hardware
 * Total 8 Queues.
 */
#define PAL_CPPI41_HOST2SR_TX_Q_BASE                    196
#define PAL_CPPI41_HOST2SR_TX_Q_LAST                    203


#define PAL_ACC_CHN_BASE                                 0
#define PAL_ACC_CHN_LAST                                31


/* External descriptor common size */
#define PAL_CPPI_COMMON1_BD_SIZE                        64
#define PAL_CPPI_COMMON2_BD_SIZE                        64
#define PAL_CPPI_COMMON3_BD_SIZE                        64
#define PAL_CPPI_COMMON4_BD_SIZE                        64

/**
 * Queue manager configurations
 */
#define PAL_CPPI41_NUM_QUEUE_MGR                2    /**< Number of queue managers in CPPI system */
#define PAL_CPPI41_MAX_DESC_REGIONS            16    /**< Maximum descriptor regions per queue manager */
#define PAL_CPPI41_SR_QMGR_TOTAL_Q_COUNT             368    /**< Maximum number of queues per queue manager */
#define PAL_CPPI41_QUEUE_MGR0                   0    /* To be removed (Mark) */
#define PAL_CPPI41_QUEUE_MGR_PARTITION_SR       0
#define PAL_CPPI41_QUEUE_MGR_PARTITION_DOCSIS   1

/**
 * Buffer manager configurations
 */
#define PAL_CPPI41_NUM_BUF_MGR                  2    /**< Number of buffer managers in the system */
#define PAL_CPPI41_BMGR_MAX_POOLS               14    /**< Maximum number of buffer pools per manager */
#define PAL_CPPI41_BUF_MGR_PARTITION_SR         0
#define PAL_CPPI41_BUF_MGR_PARTITION_DOCSIS     1

/**
 * DMA channel configuration
 */
#define PAL_CPPI41_SR_DMA_MAX_TX_CHANNELS             22    /**< Max CPPI4.1 Tx channles that could be opened */
#define PAL_CPPI41_SR_DMA_MAX_RX_CHANNELS             22    /**< Max CPPI4.1 Rx channles that could be opened */
#define PAL_CPPI41_NUM_TOTAL_CHAN              22    /**< Number of total channels */

/**
 * DMA scheduler entries.
 */
#define PAL_CPPI41_HOST2SR_DMA_CHN_NUM_BASE             0
#define PAL_CPPI41_HOST2SR_DMA_CHN_NUM_LAST             1

#define PAL_CPPI41_RECYCLE_DMA_CHN_NUM_BASE             2
#define PAL_CPPI41_RECYCLE_DMA_CHN_NUM_LAST             2

#define PAL_CPPI41_SR2C55_DMA_CHN_NUM_BASE              3
#define PAL_CPPI41_SR2C55_DMA_CHN_NUM_LAST              3

#define PAL_CPPI41_USB_DMA_CHN_NUM_BASE                 4
#define PAL_CPPI41_USB_DMA_CHN_NUM_LAST                 7

#define PAL_CPPI41_ETH_DMA_CHN_NUM_BASE                 8
#define PAL_CPPI41_ETH_DMA_CHN_NUM_LAST                 8

#define PAL_CPPI41_DOCSIS_DMA_CHN_NUM_BASE              9
#define PAL_CPPI41_DOCSIS_DMA_CHN_NUM_LAST              13

#define PAL_CPPI41_MPEG_DMA_CHN_NUM_BASE                13
#define PAL_CPPI41_MPEG_DMA_CHN_NUM_LAST                13

#define PAL_CPPI41_DOCSIS_MGMT_DMA_CHN_NUM_BASE         14
#define PAL_CPPI41_DOCSIS_MGMT_DMA_CHN_NUM_LAST         14

#define PAL_CPPI41_DOCSIS_US_DMA_CHN_NUM_BASE           15
#define PAL_CPPI41_DOCSIS_US_DMA_CHN_NUM_LAST           15

#define PAL_CPPI41_DOC2HOST_DMA_CHN_NUM_BASE            16
#define PAL_CPPI41_DOC2HOST_DMA_CHN_NUM_LAST            18

#define PAL_CPPI41_USB2HOST_DMA_CHN_NUM_BASE            20
#define PAL_CPPI41_USB2HOST_DMA_CHN_NUM_LAST            20

#define PAL_CPPI41_ETH2HOST_DMA_CHN_NUM_BASE            21
#define PAL_CPPI41_ETH2HOST_DMA_CHN_NUM_LAST            21

#define PAL_CPPI41_DMA_BLOCK0                           0
#define PAL_CPPI41_DMA_BLOCK1                           1

#define PAL_CPPI41_NUM_DMA_BLOCK                2    /**< Total number of DMA blocks in the system */



/************************************************************************/
/*                                                                      */
/*      Free Buffer (HOST) Descriptors queues allocation                */
/*                                                                      */
/************************************************************************/
#define CPPI4x_CNI_RX_INFRA_FD_HOST_Q_COUNT                  3
#define CPPI4x_CNI_RX_INFRA_FD_HOST_QNUM(pri)                (PAL_CPPI41_FBD_Q_BASE + 0 + (pri))
#define PAL_CPPI41_SR_CNI_INFRA_FD_HOST_Q_NUM(pri)           CPPI4x_CNI_RX_INFRA_FD_HOST_QNUM(pri)

#define CPPI4x_SR_DOCSIS_MGMT_FD_HOST_Q_COUNT   2
#define CPPI4x_SR_DOCSIS_MGMT_RX_FD_HOST_QNUM        (CPPI4x_CNI_RX_INFRA_FD_HOST_QNUM(CPPI4x_CNI_RX_INFRA_FD_HOST_Q_COUNT))
#define PAL_CPPI41_SR_DOCSIS_MGMT_RX_FD_HOST_Q_NUM   CPPI4x_SR_DOCSIS_MGMT_RX_FD_HOST_QNUM
#define CPPI4x_SR_DOCSIS_MGMT_TX_FD_HOST_QNUM        (CPPI4x_CNI_RX_INFRA_FD_HOST_QNUM(CPPI4x_CNI_RX_INFRA_FD_HOST_Q_COUNT) + 1)
#define PAL_CPPI41_SR_DOCSIS_MGMT_TX_FD_HOST_Q_NUM   CPPI4x_SR_DOCSIS_MGMT_TX_FD_HOST_QNUM

#define CPMAC_CPPI4x_FBD_QMGR                   PAL_CPPI41_QUEUE_MGR_PARTITION_SR
#define CPMAC_CPPI4x_FBD_Q_COUNT                1
#define CPMAC_CPPI4x_FBD_QNUM(pri)              (PAL_CPPI41_FBD_Q_BASE + 5 + (pri))

#define USB_CPPI4x_FBD_QMGR                     PAL_CPPI41_QUEUE_MGR_PARTITION_SR
#define USB_CPPI4x_FBD_Q_COUNT                  4
#define USB_CPPI4x_EP_FBD_QNUM(pri)             (PAL_CPPI41_FBD_Q_BASE + 6 + (pri))

#define USB_CPPI4x_EP0_FBD_QNUM(pri)            (PAL_CPPI41_FBD_Q_BASE + 6)
#define USB_CPPI4x_EP1_FBD_QNUM(pri)            (PAL_CPPI41_FBD_Q_BASE + 7)
#define USB_CPPI4x_EP2_FBD_QNUM(pri)            (PAL_CPPI41_FBD_Q_BASE + 8)
#define USB_CPPI4x_EP3_FBD_QNUM(pri)            (PAL_CPPI41_FBD_Q_BASE + 9)

#define PPFW_CPPI4x_FDB_Q_COUNT                 3
#define PPFW_CPPI4x_FDB_QNUM(pri)               (PAL_CPPI41_FBD_Q_BASE + 10 + (pri))
#define PAL_CPPI41_SR_HOST_TO_PP_FD_HOST_Q_NUM(pri)  PPFW_CPPI4x_FDB_QNUM(pri)

#define RECYCLE_CPPI4x_FBD_Q_COUNT              1
#define RECYCLE_CPPI4x_FBD_QNUM                 (PAL_CPPI41_FBD_Q_BASE + 13)

#if    (CPPI4x_SR_DOCSIS_MGMT_FD_HOST_Q_COUNT + \
        CPPI4x_CNI_RX_INFRA_FD_HOST_Q_COUNT      + \
        CPMAC_CPPI4x_FBD_Q_COUNT    + \
        USB_CPPI4x_FBD_Q_COUNT      + \
        PPFW_CPPI4x_FDB_Q_COUNT     + \
        RECYCLE_CPPI4x_FBD_Q_COUNT  + \
        PAL_CPPI41_FBD_Q_BASE - 1 > PAL_CPPI41_FBD_Q_LAST)
#error  "Allocated Free buffer descriptor queues are more than Hardware defined"
#endif
/************************************************************************/


/************************************************************************/
/*                                                                      */
/*      Free (Embedded) Descriptors queues allocation                   */
/*                                                                      */
/************************************************************************/
#define CPMAC_CPPI4x_FD_QMGR                    PAL_CPPI41_QUEUE_MGR_PARTITION_SR
#define CPMAC_CPPI4x_FD_Q_COUNT                 1
#define CPMAC_CPPI4x_FD_QNUM(pri)               (PAL_CPPI41_FD_Q_BASE + 0 + (pri))

#define USB_CPPI4x_FD_QMGR                      PAL_CPPI41_QUEUE_MGR_PARTITION_SR
#define USB_CPPI4x_FD_Q_COUNT                   4
#define USB_CPPI4x_EP0_FD_QNUM(pri)             (PAL_CPPI41_FD_Q_BASE + 1 + (pri))
#define USB_CPPI4x_EP1_FD_QNUM(pri)             (PAL_CPPI41_FD_Q_BASE + 2 + (pri))
#define USB_CPPI4x_EP2_FD_QNUM(pri)             (PAL_CPPI41_FD_Q_BASE + 3 + (pri))
#define USB_CPPI4x_EP3_FD_QNUM(pri)             (PAL_CPPI41_FD_Q_BASE + 4 + (pri))

#define DMA_CPPI4x_TD_Q_COUNT                   2
#define DMA0_CPPI4x_FTD_QMGR                    PAL_CPPI41_QUEUE_MGR_PARTITION_SR
#define DMA0_CPPI4x_FTD_QNUM                    (PAL_CPPI41_FD_Q_BASE + 5)
#define DMA1_CPPI4x_FTD_QMGR                    PAL_CPPI41_QUEUE_MGR_PARTITION_SR
#define DMA1_CPPI4x_FTD_QNUM                    (PAL_CPPI41_FD_Q_BASE + 6)

#define CPPI4x_SR_DOCSIS_RX_FD_EMB_Q_COUNT                   3
#define CPPI4x_SR_DOCSIS_RX_FD_EMB_QNUM(pri)                 (PAL_CPPI41_FD_Q_BASE + 7 + (pri))

#define PPFW_CPPI4x_FD_Q_COUNT                  2
#define PPFW_CPPI4x_FD_QNUM(pri)                (PAL_CPPI41_FD_Q_BASE + 10 + (pri))

#define C55_CPPI4x_FD_QMGR                      PAL_CPPI41_QUEUE_MGR_PARTITION_SR
#define C55_CPPI4x_FD_Q_COUNT                   1
#define C55_CPPI4x_FD_QNUM(pri)                 (PAL_CPPI41_FD_Q_BASE + 12)
#define PAL_CPPI41_SR_VOICE_DSP_C55_FD_EMB_Q_NUM   C55_CPPI4x_FD_QNUM

#define MPEG_CPPI4x_FD_QMGR                     PAL_CPPI41_QUEUE_MGR_PARTITION_SR
#define MPEG_CPPI4x_FD_Q_COUNT                  1
#define PAL_CPPI41_SR_MPEG_FD_EMB_Q_NUM                (PAL_CPPI41_FD_Q_BASE + 13)

#define MPEG_ENCAP_CPPI4x_FD_QMGR               PAL_CPPI41_QUEUE_MGR_PARTITION_SR
#define MPEG_ENCAP_CPPI4x_FD_Q_COUNT            1
#define PAL_CPPI41_SR_MPEG_ENCAP_FD_EMB_Q_NUM          (PAL_CPPI41_FD_Q_BASE + 14)


#if ((  CPPI4x_SR_DOCSIS_RX_FD_EMB_Q_COUNT             + \
        PPFW_CPPI4x_FD_Q_COUNT            + \
        CPMAC_CPPI4x_FD_Q_COUNT           + \
        USB_CPPI4x_FD_Q_COUNT             + \
        C55_CPPI4x_FD_Q_COUNT             + \
        DMA_CPPI4x_TD_Q_COUNT             + \
        MPEG_CPPI4x_FD_Q_COUNT            + \
        MPEG_ENCAP_CPPI4x_FD_Q_COUNT      + \
        PAL_CPPI41_FD_Q_BASE - 1) > PAL_CPPI41_FD_Q_LAST)
#error  "Allocated Free descriptor queues are more than Hardware defined"
#endif
/************************************************************************/


/************************************************************************/
/*                                                                      */
/*      TX Complete Descriptors (HOST) queues allocation                */
/*                                                                      */
/************************************************************************/
#define CPMAC_CPPI4x_TX_COMP_QMGR               PAL_CPPI41_QUEUE_MGR_PARTITION_SR
#define CPMAC_CPPI4x_TX_COMP_Q_COUNT            1
#define CPMAC_CPPI4x_TX_COMP_QNUM(pri)          (PAL_CPPI41_TX_CMPL_Q_BASE + 0 + pri)

#define USB_CPPI4x_TX_COMP_QMGR                 PAL_CPPI41_QUEUE_MGR_PARTITION_SR
#define USB_CPPI4x_TX_COMP_Q_COUNT              1
#define USB_CPPI4x_TX_COMP_QNUM(pri)            (PAL_CPPI41_TX_CMPL_Q_BASE + 1 + pri)

#define CPPI4x_SR_DOCSIS_DATA_TX_COMP_QMGR                 PAL_CPPI41_QUEUE_MGR_PARTITION_SR
#define CPPI4x_SR_DOCSIS_DATA_TX_COMP_Q_COUNT              3
#define CPPI4x_SR_DOCSIS_DATA_TX_COMP_QNUM(pri)            (PAL_CPPI41_TX_CMPL_Q_BASE + 2 + pri)
#define PAL_CPPI41_SR_HOST_TX_COMPLETE_LOW_Q_NUM           (CPPI4x_SR_DOCSIS_DATA_TX_COMP_QNUM(0))

#if    (CPPI4x_SR_DOCSIS_DATA_TX_COMP_Q_COUNT + \
        USB_CPPI4x_TX_COMP_Q_COUNT + \
        CPMAC_CPPI4x_TX_COMP_Q_COUNT + \
        PAL_CPPI41_TX_CMPL_Q_BASE - 1 > PAL_CPPI41_TX_CMPL_Q_LAST)
#error  "Allocated Tx completion queues are more than Hardware defined"
#endif
/************************************************************************/


/************************************************************************/
/*                                                                      */
/*      RX Complete Descriptors (HOST) queues allocation                */
/*                                                                      */
/************************************************************************/
#define CPMAC_CPPI4x_RX_QMGR                    PAL_CPPI41_QUEUE_MGR_PARTITION_SR
#define CPMAC_CPPI4x_RX_Q_COUNT                 1
#define CPMAC_CPPI4x_RX_QNUM(pri)               (PAL_CPPI41_RX_Q_BASE + 0)

#define USB_CPPI4x_RX_QMGR                      PAL_CPPI41_QUEUE_MGR_PARTITION_SR
#define USB_CPPI4x_RX_Q_COUNT                   4
#define USB_CPPI4x_EP0_RX_QNUM(pri)             (PAL_CPPI41_RX_Q_BASE + 1 + pri)
#define USB_CPPI4x_EP1_RX_QNUM(pri)             (PAL_CPPI41_RX_Q_BASE + 2 + pri)
#define USB_CPPI4x_EP2_RX_QNUM(pri)             (PAL_CPPI41_RX_Q_BASE + 3 + pri)
#define USB_CPPI4x_EP3_RX_QNUM(pri)             (PAL_CPPI41_RX_Q_BASE + 4 + pri)

#define CPPI4x_CNI_RXCMPL_Q_COUNT               3
#define CPPI4x_CNI_RXCMPL_QNUM(pri)             (PAL_CPPI41_RX_Q_BASE + 5 + (pri))
#define PAL_CPPI41_SR_CNI_HOST_RX_Q_NUM(pri)	CPPI4x_CNI_RXCMPL_QNUM(pri)

#define CPPI4x_SR_DOCSIS_MGMT_RXCMPL_Q_COUNT    1
#define CPPI4x_SR_DOCSIS_MGMT_RXCMPL_QNUM(pri)  (PAL_CPPI41_RX_Q_BASE + 8)
#define PAL_CPPI41_SR_DOCSIS_MGMT_HOST_RX_Q_NUM  CPPI4x_SR_DOCSIS_MGMT_RXCMPL_QNUM(0)

#define C55_CPPI4x_RX_QMGR                      PAL_CPPI41_QUEUE_MGR_PARTITION_SR
#define C55_CPPI4x_RX_Q_COUNT                   1
#define C55_CPPI4x_RX_QNUM(pri)                 (PAL_CPPI41_RX_Q_BASE + 9)
#define PAL_CPPI41_SR_VOICE_DSP_C55_HOST_RX_Q_NUM    PAL_CPPI41_SR_VOICE_DSP_C55_HOST_RX_Q_NUM

#define MPEG_CPPI4x_RX_QMGR                     PAL_CPPI41_QUEUE_MGR_PARTITION_SR
#define MPEG_CPPI4x_RX_Q_COUNT                  1
#define PAL_CPPI41_SR_MPEG_HOST_RX_Q_NUM                (PAL_CPPI41_RX_Q_BASE + 10)

#if    (USB_CPPI4x_RX_Q_COUNT       + \
        CPMAC_CPPI4x_RX_Q_COUNT     + \
        CPPI4x_CNI_RXCMPL_Q_COUNT       + \
        CPPI4x_SR_DOCSIS_MGMT_RXCMPL_Q_COUNT  + \
        C55_CPPI4x_RX_Q_COUNT       + \
        MPEG_CPPI4x_RX_Q_COUNT      + \
        PAL_CPPI41_RX_Q_BASE - 1 > PAL_CPPI41_RX_Q_LAST)
#error  "Allocated Free Buffer descriptor queueus queues are more than Hardware defined"
#endif
/************************************************************************/

/* Accumulator Configuration */
#define CPMAC_ACC_RX_CHNUM                  6
#define CPMAC_ACC_RX_INTV               2
#define CPMAC_ACC_TXCMPL_CHNUM              7
#define CPMAC_ACC_TXCMPL_INTV           3

#define USB_ACC_TXCMPL_CHNUM                8
#define USB_ACC_TXCMPL_INTV             4
#define USB_EP0_ACC_RX_CHNUM                2
#define USB_EP1_ACC_RX_CHNUM                3
#define USB_EP2_ACC_RX_CHNUM                4
#define USB_EP3_ACC_RX_CHNUM                5
#define USB_ALL_ACC_RX_INTV             1

#define CNI_ACC_RX_CHNUM_0                  28
#define CNI_ACC_RX_CHNUM_1                  29
#define CNI_ACC_RX_CHNUM_2                  30
#define PAL_CPPI41_CNI_ACC_CH_INTV_NUM  15
#define CNI_ACC_TXCMPL_CHNUM_0               24
#define PAL_CPPI41_TX_COMPLETE_ACC_INTV_NUM                  14

#define CNI_ACC_RX_CH_COUNT                     3
#define PAL_CPPI41_CNI_ACC_CH_NUM(idx)                   (CNI_ACC_RX_CHNUM_0 + (idx))

#define CNI_ACC_TX_CH_COUNT                     1
#define PAL_CPPI41_TX_COMPLETE_LOW_ACC_CH_NUM                    CNI_ACC_TXCMPL_CHNUM_0

#define PAL_CPPI41_DOCSIS_RX_MGMT_ACC_CH_NUM                   14
#define PAL_CPPI41_DOCSIS_RX_MGMT_ACC_INTV_NUM                10

#define C55_ACC_RX_CHNUM                    15
#define C55_ACC_RX_INTV                 11

#define MPEG_ACC_RX_CHNUM_0                 16
#define MPEG_ACC_RX_CHNUM_1                 17
#define MPEG_ACC_RX_CHNUM_2                 18
#define MPEG_ACC_RX_CHNUM_3                 19
#define PAL_CPPI41_MPEG_ACC_RX_INTV_NUM                12

#define MPEG_ACC_RX_CH_COUNT                    1
#define PAL_CPPI41_MPEG_ACC_RX_CH_NUM                  (MPEG_ACC_RX_CHNUM_0)


/* ---------------------------------------------------------------------- */
/*************Buffer pool allocation******************/
/* ---------------------------------------------------------------------- */
#define BUF_POOL_MGR0                    0

#define BMGR0_POOL00_BUF_COUNT          2048
#define BMGR0_POOL01_BUF_COUNT          2048
#define BMGR0_POOL02_BUF_COUNT          2048
#define BMGR0_POOL03_BUF_COUNT          0
#define BMGR0_POOL04_BUF_COUNT          512
#define BMGR0_POOL05_BUF_COUNT          128
#define BMGR0_POOL06_BUF_COUNT          512
#define BMGR0_POOL07_BUF_COUNT          1024
#define BMGR0_POOL08_BUF_COUNT          512
#define BMGR0_POOL09_BUF_COUNT          0
#define BMGR0_POOL10_BUF_COUNT          1024
#define BMGR0_POOL11_BUF_COUNT          512
#define BMGR0_POOL12_BUF_COUNT          256
#define BMGR0_POOL13_BUF_COUNT          1024

#define BMGR0_POOL00                    0
#define BMGR0_POOL01                    1
#define BMGR0_POOL02                    2
#define BMGR0_POOL03                    3
#define BMGR0_POOL04                    4
#define BMGR0_POOL05                    5
#define BMGR0_POOL06                    6
#define BMGR0_POOL07                    7
#define BMGR0_POOL08                    8
#define BMGR0_POOL09                    9
#define BMGR0_POOL10                    10
#define BMGR0_POOL11                    11
#define BMGR0_POOL12                    12
#define BMGR0_POOL13                    13

#define PAL_CPPI41_BMGR_POOL0                    0
#define PAL_CPPI41_BMGR_POOL1                    1
#define PAL_CPPI41_BMGR_POOL2                    2
#define PAL_CPPI41_BMGR_POOL3                    3
#define PAL_CPPI41_BMGR_POOL4                    4
#define PAL_CPPI41_BMGR_POOL5                    5
#define PAL_CPPI41_BMGR_POOL6                    6
#define PAL_CPPI41_BMGR_POOL7                    7
#define PAL_CPPI41_BMGR_POOL8                    8
#define PAL_CPPI41_BMGR_POOL9                    9
#define PAL_CPPI41_BMGR_POOL10                   10
#define PAL_CPPI41_BMGR_POOL11                    11
#define PAL_CPPI41_BMGR_POOL12                    12
#define PAL_CPPI41_BMGR_POOL13                    13

#ifdef CONFIG_INTEL_PP_TUNNEL_SUPPORT   // Support 1800 MTU
#define BMGR0_POOL00_BUF_SIZE           512
#define BMGR0_POOL01_BUF_SIZE           512
#else
#define BMGR0_POOL00_BUF_SIZE           256
#define BMGR0_POOL01_BUF_SIZE           256
#endif
#define BMGR0_POOL02_BUF_SIZE           256
#define BMGR0_POOL03_BUF_SIZE           0
#define BMGR0_POOL04_BUF_SIZE           1024
#define BMGR0_POOL05_BUF_SIZE           512
#define BMGR0_POOL06_BUF_SIZE           256
#define BMGR0_POOL07_BUF_SIZE           512
#define BMGR0_POOL08_BUF_SIZE           128
#define BMGR0_POOL09_BUF_SIZE           0
#ifdef CONFIG_INTEL_PP_TUNNEL_SUPPORT   // Support 1800 MTU
#define BMGR0_POOL10_BUF_SIZE           512
#else
#define BMGR0_POOL10_BUF_SIZE           256
#endif
#define BMGR0_POOL11_BUF_SIZE           1024
#define BMGR0_POOL12_BUF_SIZE           2048
#define BMGR0_POOL13_BUF_SIZE           128

#define BMGR0_POOL00_REF_CNT            1
#define BMGR0_POOL01_REF_CNT            1
#define BMGR0_POOL02_REF_CNT            1
#define BMGR0_POOL03_REF_CNT            0
#define BMGR0_POOL04_REF_CNT            1
#define BMGR0_POOL05_REF_CNT            1
#define BMGR0_POOL06_REF_CNT            1
#define BMGR0_POOL07_REF_CNT            0
#define BMGR0_POOL08_REF_CNT            0
#define BMGR0_POOL09_REF_CNT            0
#define BMGR0_POOL10_REF_CNT            1
#define BMGR0_POOL11_REF_CNT            1
#define BMGR0_POOL12_REF_CNT            1
#define BMGR0_POOL13_REF_CNT            0

/* ---------------------------------------------------------------------- */
#define BUF_POOL_MGR1                    1

#define BMGR1_POOL00_BUF_COUNT          64
#define BMGR1_POOL01_BUF_COUNT          0
#define BMGR1_POOL02_BUF_COUNT          0
#define BMGR1_POOL03_BUF_COUNT          0
#define BMGR1_POOL04_BUF_COUNT          0
#define BMGR1_POOL05_BUF_COUNT          0
#define BMGR1_POOL06_BUF_COUNT          0
#define BMGR1_POOL07_BUF_COUNT          0
#define BMGR1_POOL08_BUF_COUNT          0
#define BMGR1_POOL09_BUF_COUNT          0
#define BMGR1_POOL10_BUF_COUNT          0
#define BMGR1_POOL11_BUF_COUNT          0
#define BMGR1_POOL12_BUF_COUNT          0
#define BMGR1_POOL13_BUF_COUNT          0

#define BMGR1_POOL00                    0
#define BMGR1_POOL01                    1
#define BMGR1_POOL02                    2
#define BMGR1_POOL03                    3
#define BMGR1_POOL04                    4
#define BMGR1_POOL05                    5
#define BMGR1_POOL06                    6
#define BMGR1_POOL07                    7
#define BMGR1_POOL08                    8
#define BMGR1_POOL09                    9
#define BMGR1_POOL10                    10
#define BMGR1_POOL11                    11
#define BMGR1_POOL12                    12
#define BMGR1_POOL13                    13

#define BMGR1_POOL00_BUF_SIZE           512
#define BMGR1_POOL01_BUF_SIZE           0
#define BMGR1_POOL02_BUF_SIZE           0
#define BMGR1_POOL03_BUF_SIZE           0
#define BMGR1_POOL04_BUF_SIZE           0
#define BMGR1_POOL05_BUF_SIZE           0
#define BMGR1_POOL06_BUF_SIZE           0
#define BMGR1_POOL07_BUF_SIZE           0
#define BMGR1_POOL08_BUF_SIZE           0
#define BMGR1_POOL09_BUF_SIZE           0
#define BMGR1_POOL10_BUF_SIZE           0
#define BMGR1_POOL11_BUF_SIZE           0
#define BMGR1_POOL12_BUF_SIZE           0
#define BMGR1_POOL13_BUF_SIZE           0

#define BMGR1_POOL00_REF_CNT            0
#define BMGR1_POOL01_REF_CNT            0
#define BMGR1_POOL02_REF_CNT            0
#define BMGR1_POOL03_REF_CNT            0
#define BMGR1_POOL04_REF_CNT            0
#define BMGR1_POOL05_REF_CNT            0
#define BMGR1_POOL06_REF_CNT            0
#define BMGR1_POOL07_REF_CNT            0
#define BMGR1_POOL08_REF_CNT            0
#define BMGR1_POOL09_REF_CNT            0
#define BMGR1_POOL10_REF_CNT            0
#define BMGR1_POOL11_REF_CNT            0
#define BMGR1_POOL12_REF_CNT            0
#define BMGR1_POOL13_REF_CNT            0

/* CPPI source port numbers - not configuration, hardware defined */
#define CPPI41_SRCPORT_USBEP0                   1
#define CPPI41_SRCPORT_USBEP1                   2
#define CPPI41_SRCPORT_USBEP2                   3
#define CPPI41_SRCPORT_USBEP3                   4
#define CPPI41_SRCPORT_CPMAC0                   5
#define PAL_CPPI41_SOURCE_PORT_DOCSIS           6 /* only one CNI port 6 */
#define CPPI41_SRCPORT_DOCSISMACPHY1            6
#define CPPI41_SRCPORT_DOCSISMACPHY2            6
#define CPPI41_SRCPORT_DOCSISMACPHY3            6
#define CPPI41_SRCPORT_DOCSISMACPHY_HIGHPRIO    6
#define CPPI41_SRCPORT_DOCSISMACPHY_MGMT        6
#define CPPI41_SRCPORT_DOCSISUSCOP              0



/* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/************ Recycling Resource Allocation **************/
/* +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */


#define RECYCLE_INFRA_CHN(pri)                  (PAL_CPPI41_RECYCLE_DMA_CHN_NUM_BASE)
#define RECYCLE_INFRA_RX_QMGR                   PAL_CPPI41_QUEUE_MGR_PARTITION_SR
#define RECYCLE_INFRA_RX_Q(pri)                 (232)
#define PAL_CPPI41_RECYCLE_INFRA_INPUT_LOW_Q_NUM                 (232)


/************************************************************************/
/*                                                                      */
/*           ____                   _             _                     */
/*          |  _ \    __ _    ___  | | __   ___  | |_                   */
/*          | |_) |  / _` |  / __| | |/ /  / _ \ | __|                  */
/*          |  __/  | (_| | | (__  |   <  |  __/ | |_                   */
/*          |_|      \__,_|  \___| |_|\_\  \___|  \__|                  */
/*                                                                      */
/*                                                                      */
/*     ____                                                             */
/*    |  _ \   _ __    ___     ___    ___   ___   ___    ___    _ __    */
/*    | |_) | | '__|  / _ \   / __|  / _ \ / __| / __|  / _ \  | '__|   */
/*    |  __/  | |    | (_) | | (__  |  __/ \__ \ \__ \ | (_) | | |      */
/*    |_|     |_|     \___/   \___|  \___| |___/ |___/  \___/  |_|      */
/*                                                                      */
/*                                                                      */
/*                                                                      */
/************************************************************************/

#define PAL_CPPI41_CPDSP_FW_Q_BASE                  32
#define PAL_CPPI41_CPDSP_FW_Q_LAST                  39

#define PAL_CPPI41_MPDSP_FW_Q_BASE                  40
#define PAL_CPPI41_MPDSP_FW_Q_LAST                  47

#define PAL_CPPI41_QPDSP_IP_Q_BASE                  48
#define PAL_CPPI41_QPDSP_IP_Q_LAST                  55

#define PAL_CPPI41_APDSP_IP_Q_BASE                  56
#define PAL_CPPI41_APDSP_IP_Q_LAST                  63

#define PAL_CPPI41_QPDSP_FW_Q_BASE                  64
#define PAL_CPPI41_SR_QPDSP_QOS_Q_BASE              PAL_CPPI41_QPDSP_FW_Q_BASE
#define PAL_CPPI41_QPDSP_FW_Q_LAST                  95
#define PAL_CPPI41_SR_QPDSP_QOS_Q_LAST              PAL_CPPI41_QPDSP_FW_Q_LAST

#define PAL_CPPI4x_PRTY_LOW                         0
#define PAL_CPPI4x_PRTY_MED                         1
#define PAL_CPPI4x_PRTY_HIGH                        2
#define PAL_CPPI4x_PRTY_URGENT                      3  // Actually not used


#define CPPI4x_PPFW_TX_INFRA_CH_COUNT       2
#define CPPI4x_PPFW_TX_INFRA_CHNUM(idx)     (PAL_CPPI41_HOST2SR_DMA_CHN_NUM_BASE + (idx))

#define CPPI4x_PPFW_TX_INFRA_INPUT_QNUM(pri)      (PAL_CPPI41_HOST2SR_TX_Q_BASE + (4 * (pri)))

#define PPFW_CPPI4x_RX_INGRESS_Q_COUNT              4
#define PPFW_CPPI4x_RX_INGRESS_QNUM(pri)            (PAL_CPPI41_APDSP_IP_Q_BASE + (pri))
#define PAL_CPPI41_SR_PPDSP_LOW_Q_NUM               PPFW_CPPI4x_RX_INGRESS_QNUM(0)
#define PAL_CPPI41_SR_PPDSP_HIGH_Q_NUM              PPFW_CPPI4x_RX_INGRESS_QNUM(2)

#define PPFW_CPPI4x_TX_EGRESS_Q_COUNT               4
#define PPFW_CPPI4x_TX_EGRESS_EMB_QNUM(pri)         (PAL_CPPI41_QPDSP_IP_Q_BASE + 2*(pri) + 1)
#define PPFW_CPPI4x_TX_EGRESS_HOST_QNUM(pri)        (PAL_CPPI41_QPDSP_IP_Q_BASE + 2*(pri) + 0)

#define PPFW_CPPI4x_QoS_Q_COUNT                     ((PAL_CPPI41_QPDSP_FW_Q_LAST - PAL_CPPI41_QPDSP_FW_Q_BASE) + 1)
#define PPFW_CPPI4x_QoS_QNUM(pri)                   (PAL_CPPI41_QPDSP_FW_Q_BASE + (pri))

#define PAL_CPPI41_SR_HOST_TO_PP_Q_COUNT               3
#define PAL_CPPI41_SR_HOST_TO_PP_INFRA_INPUT_Q_NUM(pri)            (((pri)>(CPPI4x_PPFW_TX_INFRA_CH_COUNT - 1)) ? PPFW_CPPI4x_TX_EGRESS_HOST_QNUM(pri) : CPPI4x_PPFW_TX_INFRA_INPUT_QNUM(pri))



#define PPFW_RX_EMBEDDED_BD_NUM_LOW                 128
#define PPFW_RX_EMBEDDED_BD_NUM_MED                 64

#define PPFW_TX_HOST_BD_NUM_LOW                     8
#define PPFW_TX_HOST_BD_NUM_MED                     8
#define PPFW_TX_HOST_BD_NUM_HIGH                    16

#define PPFW_HOST_BD_SIZE                           64
#define PPFW_HOST_BD_NUM       (PPFW_TX_HOST_BD_NUM_LOW + \
                                PPFW_TX_HOST_BD_NUM_MED + \
                                PPFW_TX_HOST_BD_NUM_HIGH)

/* PP-FW parameters */
#define PPFW_CPPI4x_BD_SIZE                                 64
#define PPFW_CPPI4x_BUF_BD_SIZE                            128

#define PPFW_CPPI4x_APDSP_FD_QNUM                   120
#define PPFW_CPPI4x_MPDSP_FD_QNUM                   121
#define PPFW_CPPI4x_APDSP_PREBUF_QNUM               63

#define PPFW_EVENT_DESC_NUM                 256
#define PPFW_REPLICA_DESC_COUNT             (0)
#define PPFW_PREFETCH_DESC_COUNT            (510)
#define PPFW_PREFETCH_BUFF_COUNT            (24)

/* Queue configuration for PP --> Host Envets */
#define PP_HOST_EVENTQMGR       0
#define PP_HOST_EVENTQ          116
#define PP_HOST_EVENT_FDQMGR    0
#define PP_HOST_EVENT_FDQ       117


/************************************************************************/
/*            ____   ____    __  __      _       ____                   */
/*           / ___| |  _ \  |  \/  |    / \     / ___|                  */
/*          | |     | |_) | | |\/| |   / _ \   | |                      */
/*          | |___  |  __/  | |  | |  / ___ \  | |___                   */
/*           \____| |_|     |_|  |_| /_/   \_\  \____|                  */
/*                                                                      */
/************************************************************************/
/* Total 2 queues 2 used */
#define PAL_CPPI41_MAC_TX_Q_BASE                    212
#define PAL_CPPI41_MAC_TX_Q_LAST                    213

#define PAL_CPPI41_ETH_TO_HOST_PRXY_BASE            230
#define PAL_CPPI41_ETH_TO_HOST_PRXY_LAST            231

#define CPMAC_CPPI4x_TX_QMGR                      0
#define CPMAC_CPPI4x_TX_Q_BASE                    (PAL_CPPI41_MAC_TX_Q_BASE)
#define CPMAC_CPPI4x_TX_Q_COUNT                   2
#define CPMAC_CPPI4x_TX_QNUM(pri)                 (CPMAC_CPPI4x_TX_Q_BASE + pri)

#define CPMAC_CPPI4x_QoS_HIGH_TX_QNUM             PPFW_CPPI4x_QoS_QNUM(16)
#define CPMAC_CPPI4x_QoS_HIGH_MED_TX_QNUM         PPFW_CPPI4x_QoS_QNUM(17)
#define CPMAC_CPPI4x_QoS_LOW_MED_TX_QNUM          PPFW_CPPI4x_QoS_QNUM(18)
#define CPMAC_CPPI4x_QoS_LOW_TX_QNUM              PPFW_CPPI4x_QoS_QNUM(19)
#define CPMAC_CPPI4x_QoS_CLUSTER_COUNT            1
#define CPMAC_CPPI4x_QoS_CLUSTER_IDX              8

#define CPMAC_CPPI4x_ETH_TO_HOST_PRXY_QMGR        0
#define CPMAC_CPPI4x_ETH_TO_HOST_PRXY_Q_BASE      PAL_CPPI41_ETH_TO_HOST_PRXY_BASE
#define CPMAC_CPPI4x_ETH_TO_HOST_PRXY_Q_COUNT     2
#define CPMAC_CPPI4x_ETH_TO_HOST_PRXY_QNUM(pri)   (CPMAC_CPPI4x_ETH_TO_HOST_PRXY_Q_BASE + pri)

#define CPMAC_CPPI4x_RX_DMA_CH_COUNT              1
#define CPMAC_CPPI4x_RX_DMA_CHNUM(idx)            (PAL_CPPI41_ETH_DMA_CHN_NUM_BASE + idx)
#define CPMAC_CPPI4x_RX_HOST_BD_SIZE             64 /* sizeof Cppi4HostDesc + align */

#define CPMAC_CPPI4x_TX_DMA_CH_COUNT              1
#define CPMAC_CPPI4x_TX_DMA_CHNUM(idx)            (PAL_CPPI41_ETH_DMA_CHN_NUM_BASE + idx)
#define CPMAC_CPPI4x_TX_HOST_BD_SIZE             64 /* sizeof Cppi4HostDesc + align */

#define CPMAC_CPPI4x_ETH2HOST_PROXY_CH_COUNT      1
#define CPMAC_CPPI4x_ETH2HOST_PROXY_CHNUM(idx)    (PAL_CPPI41_ETH2HOST_DMA_CHN_NUM_BASE + idx)
#define CPMAC_CPPI4x_RX_EMB_BD_SIZE              64 /* sizeof Cppi4HostDesc + align */

#define CPMAC_CPPI4x_POOL_MGR                     0
#define CPMAC_CPPI4x_POOL_COUNT                   2
#define CPMAC_CPPI4x_POOL_NUM(sltcnt)             (BMGR0_POOL10 + sltcnt)

/************************************************************************/
/*                       _   _   ____    ____                           */
/*                      | | | | / ___|  | __ )                          */
/*                      | | | | \___ \  |  _ \                          */
/*                      | |_| |  ___) | | |_) |                         */
/*                       \___/  |____/  |____/                          */
/*                                                                      */
/************************************************************************/
/* Total 8 queues 4 used */
#define PAL_CPPI41_USB_TX_Q_BASE                    204
#define PAL_CPPI41_USB_TX_Q_LAST                    211

#define PAL_CPPI41_USB_TO_HOST_PRXY_BASE            226
#define PAL_CPPI41_USB_TO_HOST_PRXY_LAST            229

#define USB_CPPI4x_TX_QMGR                      0
#define USB_CPPI4x_TX_Q_BASE                    PAL_CPPI41_USB_TX_Q_BASE
#define USB_CPPI4x_TX_Q_COUNT                   8
#define USB_CPPI4x_EP0_TX_QNUM(pri)             (USB_CPPI4x_TX_Q_BASE + pri + 0)
#define USB_CPPI4x_EP1_TX_QNUM(pri)             (USB_CPPI4x_TX_Q_BASE + pri + 2)
#define USB_CPPI4x_EP2_TX_QNUM(pri)             (USB_CPPI4x_TX_Q_BASE + pri + 4)
#define USB_CPPI4x_EP3_TX_QNUM(pri)             (USB_CPPI4x_TX_Q_BASE + pri + 6)

#define USB_CPPI4x_USB2HOST_PROXY_CH_COUNT      1
#define USB_CPPI4x_USB2HOST_PROXY_CHNUM(idx)    (PAL_CPPI41_USB2HOST_DMA_CHN_NUM_BASE)
#define USB_CPPI4x_RX_EMB_BD_SIZE               64 /* sizeof Cppi4HostDesc + align */

#define USB_CPPI4x_USB_TO_HOST_PRXY_Q_COUNT     4
#define USB_CPPI4x_USB_TO_HOST_PRXY_QMGR        0
#define USB_CPPI4x_USB_TO_HOST_PRXY_Q_BASE      PAL_CPPI41_USB_TO_HOST_PRXY_BASE
#define USB_CPPI4x_USB_TO_HOST_PRXY_QNUM(pri)   (USB_CPPI4x_USB_TO_HOST_PRXY_Q_BASE + (pri))

#define USB_CPPI4x_RX_DMA_CH_COUNT              4
#define USB_CPPI4x_RX_DMA_CHNUM(idx)            (PAL_CPPI41_USB_DMA_CHN_NUM_BASE + (idx))

#define USB_CPPI4x_TX_DMA_CH_COUNT              4
#define USB_CPPI4x_TX_DMA_CHNUM(idx)            (PAL_CPPI41_USB_DMA_CHN_NUM_BASE + (idx))

#define USB_CPPI4x_USB_BD_SIZE                  64
#define USB_MAX_EPS                             8
#define USB_CPPI4x_MAX_USB_DESC                 (64 * USB_MAX_EPS) /* number of Descriptors for all tx/rx endpoints */


/************************************************************************/
/*                    ____   _   _   ___                                */
/*                   / ___| | \ | | |_ _|                               */
/*                  | |     |  \| |  | |                                */
/*                  | |___  | |\  |  | |                                */
/*                   \____| |_| \_| |___|                               */
/*                                                                      */
/************************************************************************/
#define PAL_CPPI41_DOCSIS_TX_Q_BASE                     180
#define PAL_CPPI41_DOCSIS_TX_Q_LAST                     189

#define PAL_CPPI4x_DOCSIS_TO_HOST_PROXY_Q_BASE          222
#define PAL_CPPI4x_DOCSIS_TO_HOST_PROXY_Q_LAST          224

#define PAL_CPPI41_SR_CNI_INFRA_INPUT_Q_NUM(idx)             (PAL_CPPI4x_DOCSIS_TO_HOST_PROXY_Q_BASE + (idx))

#define CPPI4x_CNI_RX_INFRA_CH_COUNT                    3
#define PAL_CPPI41_SR_CNI_INFRA_DMA_CH_COUNT            CPPI4x_CNI_RX_INFRA_CH_COUNT
#define CPPI4x_CNI_RX_INFRA_CH_NUM(idx)                 (PAL_CPPI41_DOC2HOST_DMA_CHN_NUM_BASE + (idx))

#define CPPI4x_SR_DOCSIS_RX_DMA_CH_COUNT                3
#define CPPI4x_SR_DOCSIS_RX_DMA_CHNUM(idx)              (PAL_CPPI41_DOCSIS_DMA_CHN_NUM_BASE + (idx))

#define CPPI4x_SR_DOCSIS_RX_FD_EMB_DESC_NUM_HIGH        128
#define CPPI4x_SR_DOCSIS_RX_FD_EMB_DESC_NUM_LOW         2048

#define CPPI4x_SR_DOCSIS_TX_DMA_CH_COUNT                4
#define CPPI4x_SR_DOCSIS_TX_DMA_CHNUM(idx)              (PAL_CPPI41_DOCSIS_DMA_CHN_NUM_BASE + (idx))

#define CPPI4x_SR_DOCSIS_TX_CoP_DMA_TX_CHNUM(idx)       (PAL_CPPI41_DOCSIS_US_DMA_CHN_NUM_BASE)
#define CPPI4x_SR_DOCSIS_TX_CoP_DMA_RX_CHNUM(idx)       (PAL_CPPI41_DOCSIS_US_DMA_CHN_NUM_BASE)

#define CPPI4x_SR_DOCSIS_MGMT_RXCMPL_CHNUM              (PAL_CPPI41_DOCSIS_MGMT_DMA_CHN_NUM_BASE)

#define CPPI4x_SR_DOCSIS_MGMT_TX_Q_COUNT                1
#define CPPI4x_SR_DOCSIS_MGMT_TX_QNUM(pri)              (PAL_CPPI41_DOCSIS_TX_Q_BASE)
#define PAL_CPPI41_SR_DOCSIS_TX_MGMT_Q_NUM              CPPI4x_SR_DOCSIS_MGMT_TX_QNUM(0)

#define PAL_CPPI41_SR_DOCSIS_TX_DATA_Q_COUNT            9
#define PAL_CPPI41_SR_DOCSIS_TX_VOICE_Q_NUM             (PAL_CPPI41_DOCSIS_TX_Q_BASE + 1)
#define PAL_CPPI41_SR_DOCSIS_TX_DATA_Q_NUM(pri)         (PAL_CPPI41_DOCSIS_TX_Q_BASE + 1 + (pri))
#define PAL_CPPI41_SR_DOCSIS_TX_DATA_Q_OFFSET(pri)      PAL_CPPI41_SR_DOCSIS_TX_DATA_Q_NUM(pri)


#define PAL_CPPI41_SR_DOCSIS_TX_HIGH_QPDSP_QOS_Q_NUM(clst)               PPFW_CPPI4x_QoS_QNUM(0 + (clst)*2)
#define PAL_CPPI41_SR_DOCSIS_TX_LOW_QPDSP_QOS_Q_NUM(clst)                PPFW_CPPI4x_QoS_QNUM(1 + (clst)*2)
#define PAL_CPPI41_SR_DOCSIS_TX_QOS_CLUSTER_COUNT       8
#define PAL_CPPI41_SR_DOCSIS_TX_QOS_CLUSTER_BASE        0


#define CPPI4x_CNI_RX_INFRA_FD_HOST_DESC_NUM_HIGH       8
#define CPPI4x_CNI_RX_INFRA_FD_HOST_DESC_NUM_MED        16
#define CPPI4x_CNI_RX_INFRA_FD_HOST_DESC_NUM_LOW        64

#define CPPI4x_CNI_RX_INFRA_FD_HOST_BUFF_SIZE           1600
#define PAL_CPPI41_SR_CNI_INFRA_FD_HOST_BUFFER_SIZE     CPPI4x_CNI_RX_INFRA_FD_HOST_BUFF_SIZE

#define CPPI4x_CNI_RX_INFRA_FD_HOST_DESC_SIZE           64
#define PAL_CPPI41_SR_CNI_INFRA_FD_HOST_DESC_SIZE       CPPI4x_CNI_RX_INFRA_FD_HOST_DESC_SIZE
#define CPPI4x_CNI_RX_INFRA_FD_HOST_DESC_NUM            ( CPPI4x_CNI_RX_INFRA_FD_HOST_DESC_NUM_HIGH + \
                                                          CPPI4x_CNI_RX_INFRA_FD_HOST_DESC_NUM_MED  + \
                                                          CPPI4x_CNI_RX_INFRA_FD_HOST_DESC_NUM_LOW  )
#define PAL_CPPI41_SR_CNI_INFRA_FD_HOST_DESC_COUNT      CPPI4x_CNI_RX_INFRA_FD_HOST_DESC_NUM
#define CPPI4x_SR_DOCSIS_MGMT_RX_FD_HOST_DESC_SIZE      64
#define PAL_CPPI41_SR_DOCSIS_MGMT_RX_FD_HOST_DESC_SIZE  CPPI4x_SR_DOCSIS_MGMT_RX_FD_HOST_DESC_SIZE
#define CPPI4x_SR_DOCSIS_MGMT_RX_FD_HOST_DESC_NUM       64
#define PAL_CPPI41_SR_DOCSIS_MGMT_RX_FD_HOST_DESC_COUNT CPPI4x_SR_DOCSIS_MGMT_RX_FD_HOST_DESC_NUM
#define CPPI4x_SR_DOCSIS_MGMT_RX_FD_HOST_BUFF_SIZE      2048

#define CPPI4x_SR_DOCSIS_MGMT_TX_FD_HOST_DESC_SIZE      64
#define PAL_CPPI41_SR_DOCSIS_MGMT_TX_FD_HOST_DESC_SIZE  CPPI4x_SR_DOCSIS_MGMT_TX_FD_HOST_DESC_SIZE
#define CPPI4x_SR_DOCSIS_MGMT_TX_FD_HOST_DESC_NUM       64
#define CPPI4x_SR_DOCSIS_MGMT_TX_FD_HOST_BUFF_SIZE      2048
#define PAL_CPPI41_SR_DOCSIS_MGMT_TX_FD_HOST_BUFF_SIZE  CPPI4x_SR_DOCSIS_MGMT_TX_FD_HOST_BUFF_SIZE

#define PAL_CPPI41_DOCSIS_INTERNAL_Q_BASE               0
#define PAL_CPPI41_DOCSIS_INTERNAL_Q_LAST               63

#define CNI_CPPI4x_DOCSIS_DS_CoP_Q                      (PAL_CPPI41_DOCSIS_INTERNAL_Q_BASE + 8)

#define CNI_CPPI4x_DOCSIS_DS_FDQ                        (PAL_CPPI41_DOCSIS_INTERNAL_Q_BASE + 58)
#define CNI_CPPI4x_DOCSIS_DS_DESC_SIZE                  128
#define CNI_CPPI4x_DOCSIS_DS_DESC_NUM                   32

#define CNI_CPPI4x_DOCSIS_US_FDQ                        (PAL_CPPI41_DOCSIS_INTERNAL_Q_BASE + 60)
#define CNI_CPPI4x_DOCSIS_US_DESC_SIZE                  64
#define CNI_CPPI4x_DOCSIS_US_DESC_NUM                   512

/************************************************************************/
/*                   __  __   ____    _____    ____                     */
/*                  |  \/  | |  _ \  | ____|  / ___|                    */
/*                  | |\/| | | |_) | |  _|   | |  _                     */
/*                  | |  | | |  __/  | |___  | |_| |                    */
/*                  |_|  |_| |_|     |_____|  \____|                    */
/*                                                                      */
/************************************************************************/
#define PAL_CPPI4x_MPEG_TX_Q_BASE               216
#define PAL_CPPI4x_MPEG_TX_Q_LAST               216

#define PAL_CPPI4x_MPEG_OUT_Q_BASE              218
#define PAL_CPPI4x_MPEG_OUT_Q_LAST              221


#define MPEG_CPPI4x_RX_DMA_CH_COUNT             1
#define MPEG_CPPI4x_RX_DMA_CHNUM                (PAL_CPPI41_DOCSIS_DMA_CHN_NUM_BASE + 3)

#define MPEG_CPPI4x_TX_SESSION_QMGR             PAL_CPPI41_QUEUE_MGR_PARTITION_SR
#define MPEG_CPPI4x_TX_SESSION_Q_COUNT          1
#define MPEG_CPPI4x_TX_SESSION_QNUM(idx)        (PAL_CPPI4x_MPEG_TX_Q_BASE + (idx))

#define MPEG_CPPI4x_POOL_MGR                    PAL_CPPI41_QUEUE_MGR_PARTITION_SR
#define MPEG_CPPI4x_POOL_COUNT                  1
#define MPEG_CPPI4x_POOL_NUM(sltcnt)            (BMGR0_POOL02)

#define MPEG_ENCAP_CPPI4x_POOL_MGR              PAL_CPPI41_QUEUE_MGR_PARTITION_SR
#define MPEG_ENCAP_CPPI4x_POOL_COUNT            1
#define MPEG_ENCAP_CPPI4x_POOL_NUM(sltcnt)      (BMGR0_POOL08)

#define MPEG_BD_SIZE                            64
#define MPEG_ENCAP_BD_SIZE                      128

#define MPEGOUT_CPPI4x_CHNUM                    PAL_CPPI41_MPEG_DMA_CHN_NUM_BASE
#define MPEGOUT_CPPI4x_TX_QMGR                  PAL_CPPI41_QUEUE_MGR_PARTITION_SR
#define MPEGOUT_CPPI4x_TX_Q_BASE                PAL_CPPI4x_MPEG_OUT_Q_BASE
#define MPEGOUT_CPPI4x_TX_Q_COUNT               4
#define MPEGOUT_CPPI4x_TX_QNUM(idx)             (MPEGOUT_CPPI4x_TX_Q_BASE + (idx))


#define DMAC_MPEG_RX_EMBEDDED_BD_NUM            512
#define DMAC_MPEG_ENCAP_RX_EMBEDDED_BD_NUM      256


/* ---------------------------------------------------------------- */
/********************** Shared resources parameters *****************/
/* ---------------------------------------------------------------- */

/* DOCSIS MAC parameters */
#define DMAC_US_BD_BASE                     0x01119000
#define DMAC_US_BD_SIZE                            128

/* ---------------------------------------------------------------- */
/*********************** Ports private parameters *******************/
/* ---------------------------------------------------------------- */

/* USB legacy parameters
 *  This will be removed once PSP team integrates USB driver
 *   with this header file.
 */

#define USB_CPPI41_TX1_QMGR                 0
#define USB_CPPI41_TX1_QNUM                 USB_CPPI4x_EP0_TX_QNUM(0)
#define USB_CPPI41_TX2_QMGR                 0
#define USB_CPPI41_TX2_QNUM                 USB_CPPI4x_EP1_TX_QNUM(0)
#define USB_CPPI41_TX3_QMGR                 0
#define USB_CPPI41_TX3_QNUM                 USB_CPPI4x_EP2_TX_QNUM(0)
#define USB_CPPI41_TX4_QMGR                 0
#define USB_CPPI41_TX4_QNUM                 USB_CPPI4x_EP3_TX_QNUM(0)

#define USB_CPPI41_TXCMPL_QMGR              USB_CPPI4x_TX_COMP_QMGR
#define USB_CPPI41_TXCMPL_QNUM              USB_CPPI4x_TX_COMP_QNUM(0)

#define USB_CPPI41_RX1_QMGR                 0
#define USB_CPPI41_RX1_QNUM                 USB_CPPI4x_EP0_RX_QNUM(0)
#define USB_CPPI41_RX2_QMGR                 0
#define USB_CPPI41_RX2_QNUM                 USB_CPPI4x_EP1_RX_QNUM(0)
#define USB_CPPI41_RX3_QMGR                 0
#define USB_CPPI41_RX3_QNUM                 USB_CPPI4x_EP2_RX_QNUM(0)
#define USB_CPPI41_RX4_QMGR                 0
#define USB_CPPI41_RX4_QNUM                 USB_CPPI4x_EP3_RX_QNUM(0)

#define USB_CPPI41_RX1_FDB_QMGR             0
#define USB_CPPI41_RX1_FDB_QNUM             USB_CPPI4x_EP0_FBD_QNUM(0)
#define USB_CPPI41_RX2_FDB_QMGR             0
#define USB_CPPI41_RX2_FDB_QNUM             USB_CPPI4x_EP1_FBD_QNUM(0)
#define USB_CPPI41_RX3_FDB_QMGR             0
#define USB_CPPI41_RX3_FDB_QNUM             USB_CPPI4x_EP2_FBD_QNUM(0)
#define USB_CPPI41_RX4_FDB_QMGR             0
#define USB_CPPI41_RX4_FDB_QNUM             USB_CPPI4x_EP3_FBD_QNUM(0)

#define CPPI41_USB_BD_SIZE                  USB_CPPI4x_USB_BD_SIZE
#define CPPI41_MAX_USB_EPS                  8
#define CPPI41_MAX_USB_DESC                 (64 * CPPI41_MAX_USB_EPS) /* number of Descriptors for all tx/rx endpoints */
#define USB_ACC_RX1_CHNUM                   USB_EP0_ACC_RX_CHNUM
#define USB_ACC_RX2_CHNUM                   USB_EP1_ACC_RX_CHNUM
#define USB_ACC_RX3_CHNUM                   USB_EP2_ACC_RX_CHNUM
#define USB_ACC_RX4_CHNUM                   USB_EP3_ACC_RX_CHNUM
#define USB_ACCTX_INTD_VEC                  USB_ACC_TXCMPL_INTV
#define USB_ACCRX_INTD_VEC                  USB_ALL_ACC_RX_INTV
#define USB_ACC_TX_CHNUM                    USB_ACC_TXCMPL_CHNUM
