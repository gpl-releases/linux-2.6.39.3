
/*
  BSD LICENSE 

  Copyright(c) 2011 Intel Corporation. All rights reserved.

  Redistribution and use in source and binary forms, with or without 
  modification, are permitted provided that the following conditions 
  are met:

    * Redistributions of source code must retain the above copyright 
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright 
      notice, this list of conditions and the following disclaimer in 
      the documentation and/or other materials provided with the 
      distribution.
    * Neither the name of Intel Corporation nor the names of its 
      contributors may be used to endorse or promote products derived 
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

/** \file   puma6_cru_ctrl.c
 *  \brief  PAL reset and power control APIs
 *          The Clock & Reset unit (CRU) enables power control
 *          of all the modules and peripherals. Power savings
 *          can be achieved by disabling modules (clock gating).
 *          
 *  \author     Intel
 *
 *  \version    0.1     Amihay Tabul   		Created
 */


#include <pal.h>
#include <puma6.h>
#include <puma6_cru_ctrl.h>


/* CRU - Clock and Reset Unit               */
/* Docsis IP has 33 CRUs                    */
/* Clock Control registers set memory map:  */

/* Start address 0x000D_0000                */
/* End address   0x000D_FFFF                */
#define CRU_MOD_STATE_BASE    (AVALANCHE_CRU_BASE)
#define CRU_MOD_STATUS_BASE   (CRU_MOD_STATE_BASE + 0x4)
#define CRU_RSTN_CLK_EN_BASE  (CRU_MOD_STATE_BASE + 0x8) /* should not be used, for debug only */

/* CRU_MOD_STATE register fields */
/* 31:2 Reserved (R) */
/* 1:0  MOD_STATE_REG (R/W) */
#define CRU_MOD_STATE_DISABLED      (0)
#define CRU_MOD_STATE_SYNC_RST      (1)
#define CRU_MOD_STATE_CLK_DISABLE   (2)
#define CRU_MOD_STATE_ENABLE        (3)

/* CRU_MOD_STATUS register fields */
/* 31:10 Reserved (R) */
/* 9:6	CRU_CG_EN_1-4 (R) - Module CG status (4 lines) */
/* 5:2	CRU_RST_N_1-4 (R) - Module resets status (4 lines) */
/* 1:0	CRU_SM_STATE (R)  - Module State Machine State (same as in CRU_MOD_STATE values) */

/* CRU_RSTN_CLK_EN register fields */
/* 31:1 Reserved (R) */
/* 0    CRU_RSTN_CLK_EN_FORCE (R/W) Force module reset and opens the clock gater */

#define CRU_MAX_STATUS_LOOP         (1000)


/* Macros to configure the CRU REGs */
#define CRU_MOD_STATE(cru_num)     *((volatile unsigned int *)(CRU_MOD_STATE_BASE   | ((cru_num)<<4)))
#define CRU_MOD_STATUS(cru_num)    *((volatile unsigned int *)(CRU_MOD_STATUS_BASE  | ((cru_num)<<4)))
#define CRU_RSTN_CLK_EN(cru_num)   *((volatile unsigned int *)(CRU_RSTN_CLK_EN_BASE | ((cru_num)<<4)))
#define CRU_GET_SM_STATE(cru_num)  (CRU_MOD_STATUS(cru_num) & (0x3)) /* bits 0-1 are the SM_STATE */


extern void avalanche_system_reset(PAL_SYS_SYSTEM_RST_MODE_T mode);



/*****************************************************************************
 * Reset Control Module.
 *****************************************************************************/
/*! \fn void PAL_sysResetCtrl(unsigned int cru_module_id, PAL_SYS_RESET_CTRL_T reset_ctrl)
    \brief This API is used to assert or de-assert reset for a module. It will block the caller.
    \param cru_module_id Unique module id to assert/de-assert reset
    \param reset_ctrl assert/de-assert reset (IN_RESET, OUT_OF_RESET)
*/
void PAL_sysResetCtrl(unsigned int cru_module_id, PAL_SYS_RESET_CTRL_T reset_ctrl)
{
    PAL_SYS_CRU_MODULE_T module_id = (PAL_SYS_CRU_MODULE_T)(cru_module_id);
    Uint32 loop_cnt = 0;
    Uint32 cru_status;

    cru_status = (reset_ctrl == OUT_OF_RESET) ? (CRU_MOD_STATE_ENABLE):(CRU_MOD_STATE_DISABLED);
    if (CRU_GET_SM_STATE(module_id) == cru_status)
    {
        printk (KERN_NOTICE "CRU %d is allready in CRU state %d [ignore operation]\n",cru_module_id,cru_status);
        return; /* If the current cru status is the same as the user ask for, we ignore the operation.*/
    }

    if ( reset_ctrl == OUT_OF_RESET )
    {
        CRU_MOD_STATE(module_id) = CRU_MOD_STATE_ENABLE;
    }
    else
    {
        CRU_MOD_STATE(module_id) = CRU_MOD_STATE_DISABLED;
    }

    /* Make sure that the CRU module is indeed in the correct ask state (OUT_OF_RESET or IN_RESET) */
    /* This loop will block the caller for some time! */
    do
    {
        if (++loop_cnt >= CRU_MAX_STATUS_LOOP)
        {
            printk (KERN_CRIT "CRU %d is not functional, current cru status %d !! [loop_cnt=%d ; ask_status=%d]\n",cru_module_id,CRU_MOD_STATUS(module_id),loop_cnt,reset_ctrl);
            return;
        }
    } while (CRU_GET_SM_STATE(module_id) != cru_status);
}


/*! \fn PAL_SYS_RESET_CTRL_T PAL_sysGetResetStatus(unsigned int cru_module_id)
    \brief This API returns the status reset status of a module
    \param cru_module_id Unique module id whose reset status has to be read
    \return Reset assert/de-assert (IN_RESET, OUT_OF_RESET)
*/
PAL_SYS_RESET_CTRL_T PAL_sysGetResetStatus(unsigned int cru_module_id)
{
    PAL_SYS_CRU_MODULE_T module_id = (PAL_SYS_CRU_MODULE_T)(cru_module_id);


    if ( CRU_GET_SM_STATE(module_id) == CRU_MOD_STATE_ENABLE )
    {
        return OUT_OF_RESET;
    }
    else
    {
        return IN_RESET;
    }
}

/*! \fn void PAL_sysSystemReset(PAL_SYS_SYSTEM_RST_MODE_T mode)
    \brief This API is used the reset the system
    \param mode system reset mode
*/
void PAL_sysSystemReset(PAL_SYS_SYSTEM_RST_MODE_T mode)
{
    /* This is processor specific so should be implemented in avalanche_misc.c */
    avalanche_system_reset(mode);
}


void PAL_sysPowerCtrl(unsigned int power_module,  PAL_SYS_POWER_CTRL_T power_ctrl)
{
    return;
}

