/*
 * puma6_pp.h
 * Description:
 * See below.
 *
 */

/*
 * This file contains Packet Processor configuration. All the configurations
 * here are system level and hence a complete knowledge of configurations of all
 * devices/drivers related to the Packet Processor is required before modifying
 * any value.
 *
 * The private data structures used by avalanche_ppd_init() are also present in
 * this file and should not be modified unless the function avalanche_ppd_init
 * in file puma6_pp.c is also required to be modified for PP configuration.
 *
 */

#ifndef _INCLUDE_PUMA6_PP_H
#define _INCLUDE_PUMA6_PP_H

#define MAX_IP_PACKET_SIZE      1514
#define MIN_IP_PACKET_SIZE      64


/* Prefetcher configuration
 *  - Setup prefetch to fetch 128 bytes with a 0 byte offset
 */
#define PREFETCH_DATA_SIZE          128
#define PREFETCH_DATA_OFFSET        0

/*
 * PID Range configurations
 */
typedef enum PP_PID_NUM
{
    PP_L2SW_PID_BASE = 0,
        PP_L2SW_APP0_PID_NUM = PP_L2SW_PID_BASE,                    // 0
        PP_L2SW_MoCA0_PID_NUM,                                      // 1
        PP_L2SW_ETH0_PID_NUM,                                       // 2
        PP_L2SW_ETH1_PID_NUM,                                       // 3
        PP_L2SW_WLAN1_PID_NUM,                                      // 4
        PP_L2SW_WLAN0_PID_NUM,                                      // 5
    PP_L2SW_PID_LAST = PP_L2SW_WLAN0_PID_NUM,
    
    PP_CNI_PID_BASE,                                                // 6
    PP_L2SW_L2SW0_PID_NUM,                                          // 7
}PP_PID_NUM_e;

#define PP_L2SW_PID_COUNT   PP_L2SW_PID_LAST - PP_L2SW_PID_BASE + 1
#define PP_CNI_PID_COUNT    1

#define PP_ETH_PID_COUNT    1   // for backward compatible
#define PP_ETH_PID_BASE     31  // for backward compatible
#define PP_USB_PID_COUNT    4   // for backward compatible
#define PP_USB_PID_BASE     26  // for backward compatible

/* QoS Clusters definitions */
typedef enum PAL_CPPI41_SR_QOS_CLUSTERS
{
    PAL_CPPI41_SR_DOCSIS_TX_QOS_CLUSTER_BASE,
        PAL_CPPI41_SR_DOCSIS_TX_BE0_QOS_CLUSTER_NUM = PAL_CPPI41_SR_DOCSIS_TX_QOS_CLUSTER_BASE,     // 0
        PAL_CPPI41_SR_DOCSIS_TX_BE1_QOS_CLUSTER_NUM,                                                // 1
        PAL_CPPI41_SR_DOCSIS_TX_BE2_QOS_CLUSTER_NUM,                                                // 2
        PAL_CPPI41_SR_DOCSIS_TX_BE3_QOS_CLUSTER_NUM,                                                // 3
        PAL_CPPI41_SR_DOCSIS_TX_BE4_QOS_CLUSTER_NUM,                                                // 4
        PAL_CPPI41_SR_DOCSIS_TX_BE5_QOS_CLUSTER_NUM,                                                // 5
        PAL_CPPI41_SR_DOCSIS_TX_BE6_QOS_CLUSTER_NUM,                                                // 6
        PAL_CPPI41_SR_DOCSIS_TX_BE7_QOS_CLUSTER_NUM,                                                // 7
        PAL_CPPI41_SR_DOCSIS_TX_BE8_QOS_CLUSTER_NUM,                                                // 8
        PAL_CPPI41_SR_DOCSIS_TX_BE9_QOS_CLUSTER_NUM,                                                // 9
        PAL_CPPI41_SR_DOCSIS_TX_BE10_QOS_CLUSTER_NUM,                                               // 10
        PAL_CPPI41_SR_DOCSIS_TX_BE11_QOS_CLUSTER_NUM,                                               // 11
        PAL_CPPI41_SR_DOCSIS_TX_BE12_QOS_CLUSTER_NUM,                                               // 12
        PAL_CPPI41_SR_DOCSIS_TX_BE13_QOS_CLUSTER_NUM,                                               // 13
        PAL_CPPI41_SR_DOCSIS_TX_BE14_QOS_CLUSTER_NUM,                                               // 14
        PAL_CPPI41_SR_DOCSIS_TX_BE15_QOS_CLUSTER_NUM,                                               // 15
    PAL_CPPI41_SR_DOCSIS_TX_QOS_CLUSTER_LAST = PAL_CPPI41_SR_DOCSIS_TX_BE15_QOS_CLUSTER_NUM,

    PAL_CPPI41_SR_L2SW_QOS_CLUSTER_BASE = 16,
        PAL_CPPI41_SR_L2SW_APP0_QOS_CLUSTER_NUM = PAL_CPPI41_SR_L2SW_QOS_CLUSTER_BASE,              // 16
        PAL_CPPI41_SR_L2SW_MoCA0_QOS_CLUSTER_NUM,                                                   // 17
        PAL_CPPI41_SR_L2SW_ETH0_QOS_CLUSTER_NUM,                                                    // 18
        PAL_CPPI41_SR_L2SW_ETH1_QOS_CLUSTER_NUM,                                                    // 19
        PAL_CPPI41_SR_L2SW_WLAN1_QOS_CLUSTER_NUM,                                                   // 20
        PAL_CPPI41_SR_L2SW_WLAN0_QOS_CLUSTER_NUM,                                                   // 21
    PAL_CPPI41_SR_L2SW_QOS_CLUSTER_LAST = PAL_CPPI41_SR_L2SW_WLAN0_QOS_CLUSTER_NUM,

    PAL_CPPI41_SR_QOS_CLUSTERS_COUNT,

    PAL_CPPI41_SR_QOS_CLUSTERS_MAX = 32

}PAL_CPPI41_SR_QOS_CLUSTERS_e;

#define PAL_CPPI41_SR_DOCSIS_TX_QOS_CLUSTER_COUNT       (PAL_CPPI41_SR_DOCSIS_TX_QOS_CLUSTER_LAST - PAL_CPPI41_SR_DOCSIS_TX_QOS_CLUSTER_BASE + 1)
#define PAL_CPPI41_SR_L2SW_QOS_CLUSTER_COUNT            (PAL_CPPI41_SR_L2SW_QOS_CLUSTER_LAST - PAL_CPPI41_SR_L2SW_QOS_CLUSTER_BASE + 1)


#define ACMD_CONFIG_PREFETCH    0x84
#define ACMD_ENABLE_PREFETCH    0x83
#define ACMD_READ_STATISTICS    0x86
#define ACMD_CONFIG_TDQ         0x87
#define ACMD_COMMAND_SHIFT      8
#define ACMD_COMMAND_MASK       (0xFFu<<ACMD_COMMAND_SHIFT)
#define ACMD_COMMAND(x)         (((x)<<ACMD_COMMAND_SHIFT)&ACMD_COMMAND_MASK)
#define ACMD_GET_COMMAND(x)     (((x)&ACMD_COMMAND_MASK)>>ACMD_COMMAND_SHIFT)
#define ACMD_INDEX_SHIFT        0
#define ACMD_INDEX_MASK         (0xFFu<<ACMD_INDEX_SHIFT)
#define ACMD_INDEX(x)           (((x)<<ACMD_INDEX_SHIFT)&ACMD_INDEX_MASK)
#define ACMD_RETCODE_SHIFT      24
#define ACMD_RETCODE_MASK       (0xFFu<<ACMD_RETCODE_SHIFT)
#define ACMD_GET_RETCODE(x)     (((x)&ACMD_RETCODE_MASK)>>ACMD_RETCODE_SHIFT)

/*
 * Data structures used to hold and perform various PP configurations
 */
typedef volatile struct
{
    CSL_Reg32 Command;
    CSL_Reg32 Parameter0;
    CSL_Reg32 Parameter1;
    CSL_Reg32 Parameter2;
    CSL_Reg32 Parameter3;
} APDSP_Command_Buffer;

typedef APDSP_Command_Buffer *APDSP_Command_Buffer_RegsOvly;

/*
 * PrefchCfg -
 * Description:
 *  Prefetcher Configuration
 */
typedef struct
{
    volatile Uint32* pfDescBase;
    Uint32 pfDsDescCnt;
    Uint32 pfUsDescCnt;
    Cppi4Queue pfDsFQ;
    Cppi4Queue pfUsFQ;

    Uint32 repDescCnt;
    volatile Uint32* repDescBase;
    Cppi4Queue repFQ;

    Uint32 pfBlkCnt;
    volatile Uint32* pfBlkBase;
    Cppi4Queue pfFBQ;

    Uint8 pfDataSize;
    Uint8 pfDataOffset;
    APDSP_Command_Buffer_RegsOvly  pfCmdBase;
} PrefchCfg;

/* TI_PP_PREF_STATS -
 *  Prefetcher statistics structure
 */
typedef struct
{
    Uint32  grp_a_preproc_pkts;         /* Group A: Preprocessed Packets */
    Uint32  grp_a_pref_buf_pkts;        /* Group A: Packets with pre-fetched
                                           buffers */
    Uint32  grp_a_pref_descbuff_pkts;   /* Group A: Packets with pre-fetched
                                           descriptors and buffers */
    Uint32  grp_a_desc_starv_cnt;       /* Group A: Packet Pushback */

    Uint32  grp_b_preproc_pkts;         /* Group B: Preprocessed Packets */
    Uint32  grp_b_pref_buf_pkts;        /* Group B: Packets with pre-fetched
                                           buffers */
    Uint32  grp_b_pref_descbuff_pkts;   /* Group B: Packets with pre-fetched
                                           descriptors and buffers */
    Uint32  grp_b_desc_starv_cnt;       /* Group B: Packet Pushback */

    Uint32  in_q_congst_discards[6];    /* Input Queue (0-5): Packet Congestion
                                           Discards */
    Uint32  in_q_congst_thrsh[6];       /* Input Queue (0-5): Packet Congestion
                                           Threshold */
} TI_PP_PREF_STATS;

/*
 * ti_pp_get_pref_stats -
 *
 * Description:
 *  This API provides the prefetcher statistics. Note that the statictics in
 *  firmware are NOT cleared by this API.
 *
 * Precondition:
 *  APDSP must be up and prefetcher must have been initialized.
 *
 * Parameters:
 *  stats (OUT)         - Prefetcher statistics.
 *
 * Return:
 *  0 on Success, <0 on error.
 */
Int32 ti_pp_get_pref_stats (TI_PP_PREF_STATS *stats);

/*
 * ti_pp_get_n_clear_pref_stats -
 *
 * Description:
 *  This API provides the prefetcher statistics. The statictics are cleared.
 *
 * Precondition:
 *  APDSP must be up and prefetcher must have been initialized.
 *
 * Parameters:
 *  stats (OUT)         - Prefetcher statistics.
 *
 * Return:
 *  0 on Success, <0 on error.
 */
Int32 ti_pp_get_n_clear_pref_stats (TI_PP_PREF_STATS *stats);

/*
 * ti_pp_enable_psm -
 *
 * Description:
 *  This API enables the PP PSM mode. First the packet processor is set in PSM
 *  by using PPD API which also halts the PDSPs and then prefetcher PSM is
 *  enabled so that packets are forwarded as per pid configuration as well as tx
 *  is performed as per desc info bypassing PP.
 *
 * Precondition:
 *  PPD must have been initialized with ti_ppd_init, APDSP must be up and
 *  prefetcher must have been initialized.
 *
 * Parameters:
 *  None.
 *
 * Return:
 *  0 on Success, <0 on error.
 */
Int32 ti_pp_enable_psm (void);

/*
 * ti_pp_disable_psm -
 *
 * Description:
 *  This API disables the PP PSM mode.First the prefetcher PSM is disabled and
 *  then packet processor PSM is disabled.
 *
 * Precondition:
 *  PSM must have been enabled using ti_pp_enable_psm.
 *
 * Parameters:
 *  None.
 *
 * Return:
 *  0 on Success, <0 on error.
 */
Int32 ti_pp_disable_psm (void);

#endif /* !_INCLUDE_PUMA6_PP_H */
