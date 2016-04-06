/*
 *
 * pal_cppi41.c
 * Description:
 * see below
 *
 *
 * Copyright (C) 2009-2011, Texas Instruments, Incorporated
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

/** \file   pal_cppi41.c
    \brief  PAL CPPI 4 Source file

    This file contains the main PAL implementation for CPPI4 common peripherals,
    including the CPPI 4 DMA Controller, the Queue Manager, and the Buffer
    Manager.  Based upon PSP Framework architecture.

    @author     Greg Guyotte
    @author     Sekhar Nori
 */
#include <asm-arm/arch-avalanche/generic/pal.h> /* PAL SYS/OS services required */
#include <asm-arm/arch-avalanche/generic/pal_cppi41.h>
#include <asm-arm/arch-avalanche/generic/pal_cppi41pvt.h>
#include <linux/proc_fs.h>

/* Version macro */
#define CPPI4_PAL_MAJOR_VERSION           0
#define CPPI4_PAL_MINOR_VERSION           1

//#define  CPPI4_DEBUG 1

#ifdef CPPI4_DEBUG
#  define dbgPrint(fmt, args...) printk("%s:%d " fmt, __FUNCTION__ , __LINE__ , ## args)
#else
#  define dbgPrint(fmt, args...)
#endif

const static Char Cppi4PALVersionString[] = "CPPI4 PAL version 0.1";

/* Static Global Instance Variable */
static Cppi4PALObj      Cppi4PALObject  [CPPI41_DOMAIN_NUM];
static Uint32           Cppi4PALNumInst [CPPI41_DOMAIN_NUM] = { 0 };
static Cppi4InitCfg     Cppi4PvtInitCfg [CPPI41_DOMAIN_NUM];
static struct proc_dir_entry * cppiDebugDir = NULL;

/* !@@
 * TODO: move in .h
 * TODO: Currenly using dummy routine, need to call proper PAL
 */

#define PAL_VirtToPhysMem(ptr) (virt_to_phys (ptr))
#define CPPI4_VIRT_TO_PHYS(ptr) (PAL_VirtToPhysMem (ptr))

/************************ CPPI4.1 PAL FUNCTIONS (External Interface) *************/

/*
 * PAL_cppi4Init
 *  - initializes the PAL CPPI4.1 instance for first caller only
 *  - returns handle to PAL
 *
 */
PAL_Handle PAL_cppi4Init (Cppi4InitCfg * initCfg, Ptr param)
{
    Cppi4TeardownDesc * tempBD = NULL;
    Cppi4PALObj *palCppi4Obj;
    Uint32 i = 0, j = 0;

    if (CPPI41_DOMAIN_NUM <= (CPPI41_DOMAIN)param)
    {
        return NULL;
    }
    printk("\n%s : domain is %d, cfg ptr is 0x%08X \n",__FUNCTION__, (Uint32)param, (Uint32)initCfg );

    palCppi4Obj = &Cppi4PALObject[(CPPI41_DOMAIN)param];

    dbgPrint("Object Address is 0x%08X \n", (Uint32)palCppi4Obj );

    /* Check if CPPI4 is already initialized.
     * TODO: Ideally this should be protected by locks, but can't think
     * of a case where multiple guys will be in CPPI4.1 init simaltaneously.
     * */
    if (Cppi4PALNumInst[(CPPI41_DOMAIN)param]++ > 0)
      return ((PAL_Handle) palCppi4Obj);
    

    if(initCfg == NULL) goto init_failed;

    PAL_osMemSet (palCppi4Obj, 0, sizeof (Cppi4PALObj));
    palCppi4Obj->myDomain = (CPPI41_DOMAIN)param;

    /* Store a local copy of the initial configuration (so that caller is free to free up
     * his configuration structure)
     */
    PAL_osMemCopy(&Cppi4PvtInitCfg[(CPPI41_DOMAIN)param], initCfg, sizeof(Cppi4InitCfg));
    /* Set CPPI4 object variables.
     *
     * Note: I assume here that accessing config data from init configuration structure is OK.
     * It will be one more level of indirection this way, but will avoid repeated information.
     * If there are performance concerns, it may be possible to replicate some (oft used)
     * initial configuration parameters in the PAL CPPI 4.1 object as well.
     */
    palCppi4Obj->initCfg = &Cppi4PvtInitCfg[(CPPI41_DOMAIN)param];

    /* use the new (private) structure now on... */
    initCfg = palCppi4Obj->initCfg;
    /* Initialize the queue manager(s) */

    dbgPrint("Before QMGR Init \n");
    for(i = 0; i < PAL_CPPI41_NUM_QUEUE_MGR; i++)
    {
        Cppi4QueueMgrCfg *curMgr = &(initCfg->queueMgrInfo[i]);
        CSL_Queue_Manager_Region_RegsOvly qMgrRegs = curMgr->queueMgrRgnBase;
        Uint32 size = 0, index = 0;
        Ptr buf = NULL;
        Uint32 offChipStart = PAL_CPPI41_MAX_DESC_REGIONS, offChipEnd = PAL_CPPI41_MAX_DESC_REGIONS; /* Set offChipStart and offChipEnd indices to invalid values */

        dbgPrint("QMGR [%d] Init ... \n",i );

        if (NULL == qMgrRegs)   continue;

        /* Check if the Linking ram is capable to hold total number of
         * descriptors, if LinkingRAM1Base is NULL
         */
        if (!curMgr->LinkingRAM1Base)
        {
            Uint32 totalDesc = 0;
            for (j = 0; j < PAL_CPPI41_MAX_DESC_REGIONS; j++)
                totalDesc += curMgr->descRegion[j].numDesc;

            if (totalDesc > curMgr->LinkingRAM0Size) {
                CPPI4_PAL_LOGERR ("\nERROR:PAL: Total number of allocated descriptors more than Linking Ram0 size");
                CPPI4_PAL_LOGERR ("\nAllocate space for linking Ram1 to accomodate more descriptors");

                goto init_failed;
            }

            dbgPrint("There is a room for %d more descriptors\n", curMgr->LinkingRAM0Size - totalDesc);
        }


        /* dbgPrint("Queue manager %d overlay address: %x, object %x\n", i, regs, curMgr); */

        qMgrRegs->Linking_RAM_Reg0_Base = curMgr->LinkingRAM0Base;
        qMgrRegs->Linking_RAM_Reg0_Size = curMgr->LinkingRAM0Size;
        qMgrRegs->Linking_RAM_Reg1_Base = curMgr->LinkingRAM1Base;
        for (j = 0; j < PAL_CPPI41_MAX_DESC_REGIONS; j++)
        {
            if(curMgr->descRegion[j].isOnChip == False) break;
        }

        offChipStart = j;

        for (j = offChipStart; j < PAL_CPPI41_MAX_DESC_REGIONS; j++)
        {
            if(curMgr->descRegion[j].isOnChip == True) break;
        }

        offChipEnd = j;

        /* Find the total size of descriptor memory required */
        for (j = offChipStart; j < offChipEnd; j++)
        {
            size += (curMgr->descRegion[j].szDesc * curMgr->descRegion[j].numDesc);
        }

        dbgPrint("offChipStart: %d, offChipEnd: %d\n", offChipStart, offChipEnd);
        dbgPrint("Aligning to size: %d\n", curMgr->descRegion[offChipStart].szDesc);

        dbgPrint("Off chip memory allocated: %d\n", size);

        /* allocate the number of bytes required - aligned to the highest descriptor size */
        // size += curMgr->descRegion[offChipStart].szDesc;
        // retVal = PAL_osMemAlloc(0, size, 0, (Ptr *) &buf);
        if(!(buf = PAL_osMemAllocSizeAligned(palCppi4Obj->myDomain, size)))
        {
            printk("\nERROR: %s : Failed to allocate descriptor memory",__FUNCTION__);
            goto init_failed;
        }

        // if (retVal != PAL_SOK) {
        //    CPPI4_PAL_LOGERR ("\nERROR:PAL: AllocDesc: Failed to allocate descriptor memory");
        //    return NULL;
        // }
        dbgPrint("QMgr Init: buffer before alignment: %p\n", buf);

        palCppi4Obj->qMgrDescRegPtr[i] = buf;
        palCppi4Obj->qMgrDescRegSize[i] = size;

        // buf += curMgr->descRegion[offChipStart].szDesc - ((Uint32) buf % curMgr->descRegion[offChipStart].szDesc);
        dbgPrint("QMgr Init: buffer after alignment: %p\n", buf);

        /* Populate the bases for all the reqions */
        for (j = offChipStart; j < offChipEnd; j++)
        {
            if(curMgr->descRegion[j].numDesc > 0)
            {
                curMgr->descRegion[j].base = buf;
                buf += curMgr->descRegion[j].szDesc * curMgr->descRegion[j].numDesc;
            }
            else
            {
                curMgr->descRegion[j].base = 0;
                curMgr->descRegion[j].szDesc = 0;
                curMgr->descRegion[j].numDesc = 0;
            }

            dbgPrint("base of BD region %d is %p and stored at %p\n", j, curMgr->descRegion[j].base, &(curMgr->descRegion[j].base));
            dbgPrint("desc region[%d].numDesc = %d, desc region[%d].szDesc = %d\n", j, curMgr->descRegion[j].numDesc, j, curMgr->descRegion[j].szDesc);

        }
        /* Update the hardware on region bases et. al. */
        for (j = 0; j < PAL_CPPI41_MAX_DESC_REGIONS; j++)
        {
            Uint32 val, powSzDesc, powRegSize;
            CSL_Desc_Mem_Setup_Region_RegsOvly descMemRegs = curMgr->descMemRgnBase;

            if (curMgr->descRegion[j].isOnChip == True)
            {
                descMemRegs->Desc_Mem_Setup[j].Mem_Reg_Base = (Uint32) curMgr->descRegion[j].base;
            }
            else
            {
                descMemRegs->Desc_Mem_Setup[j].Mem_Reg_Base = CPPI4_VIRT_TO_PHYS((Ptr)curMgr->descRegion[j].base);
            }

            for (powSzDesc = 0; (32UL << powSzDesc) < curMgr->descRegion[j].szDesc; powSzDesc++);
            for (powRegSize = 0; (32UL << powRegSize) < curMgr->descRegion[j].numDesc; powRegSize++);

            /* Write the control word */

            if(curMgr->descRegion[j].numDesc > 0)
            {
                val = (index << QMGR_MEMREG_CTRL_INDEX_SHIFT) & QMGR_MEMREG_CTRL_INDEX_MASK;
                val |= (powSzDesc << QMGR_MEMREG_CTRL_DESCSZ_SHIFT) & QMGR_MEMREG_CTRL_DESCSZ_MASK;
                val |= (powRegSize << QMGR_MEMREG_CTRL_REGSZ_SHIFT) & QMGR_MEMREG_CTRL_REGSZ_MASK;

                descMemRegs->Desc_Mem_Setup[j].Mem_Reg_Control = val;
                dbgPrint("writing to desc region base @ %p, value %08x\n", &descMemRegs->Desc_Mem_Setup[j].Mem_Reg_Base, descMemRegs->Desc_Mem_Setup[j].Mem_Reg_Base);
                dbgPrint("writing to desc region ctrl @ %p, value %08x\n", &descMemRegs->Desc_Mem_Setup[j].Mem_Reg_Control, descMemRegs->Desc_Mem_Setup[j].Mem_Reg_Control);
            }

            index += curMgr->descRegion[j].numDesc;

        }

    }

    dbgPrint("Queue manager initialized\n");

    /* Initialize the buffer manager functionality - Nothing to do right now */

    dbgPrint("Buffer manager initialized\n");

    /*
     * Initialze teardown and DMA scheduler
     * Note: The code below breaks the orthogonality of the CPPI 4.1 API.. but nevermind as long as things work..
     */
    if (CPPI41_DOMAIN_PRIMARY_SR == palCppi4Obj->myDomain)
    {
        {
            Uint32          totalNumOfTearDownDesc = 0;
            Cppi4Queue*     tdFQueue;

            for (j = 0; j < PAL_CPPI41_NUM_DMA_BLOCK; j++)
            {
                Cppi4DMABlock* dmaBlock = &initCfg->dmaBlock[j];

                if (dmaBlock->globalCtrlBase)
                {
                    tdFQueue = &dmaBlock->tdFQueue;
                    totalNumOfTearDownDesc += PAL_CPPI41_NUM_TD_DESC;
                }
            }

            if (totalNumOfTearDownDesc)
            {
                /* Pre-allocate teardown descriptors - Two per channel (is it a ballpark? I just maintained
                * what was present in CPPI 4.1 implementation. Anyway, it seems to be an OK number.).
                */
                if ((tempBD = (Cppi4TeardownDesc*) PAL_cppi4AllocDesc(palCppi4Obj, tdFQueue->qMgr , totalNumOfTearDownDesc, sizeof(Cppi4TeardownDesc))) == NULL)
                {
                    dbgPrint("WARN : Unable to allocate teardown desc.");
                    goto init_failed;
                }
            }
        }
    }
    for (j = 0; j < PAL_CPPI41_NUM_DMA_BLOCK; j++)
    {
        PAL_Cppi4QueueHnd hnd;
        Cppi4DMABlock* dmaBlock = &initCfg->dmaBlock[j];
        Cppi4Queue* tdFQueue = &dmaBlock->tdFQueue;

        if (NULL == dmaBlock->globalCtrlBase) continue;

        dbgPrint("Queue manager: %d, queue number: %d\n", tdFQueue->qMgr, tdFQueue->qNum);

        /* Tell the hardware about the Teardown descriptors free queue manager and queue number */
        dmaBlock->globalCtrlBase->Teardown_FD_Queue_Control = (tdFQueue->qMgr << DMA_GLOBCTRL_TDFQ_QMGR_SHIFT) | (tdFQueue->qNum << DMA_GLOBCTRL_TDFQ_QNUM_SHIFT);
        dbgPrint("dmaBlock->globalCtrlBase->Teardown_FD_Queue_Control address: %p, value: %x\n", &dmaBlock->globalCtrlBase->Teardown_FD_Queue_Control,
                        dmaBlock->globalCtrlBase->Teardown_FD_Queue_Control);

        if (CPPI41_DOMAIN_PRIMARY_SR == palCppi4Obj->myDomain)
        {
            /* Push all the teardown descriptors to the free teardown queue for the CPPI 4.1 system. */
            if((hnd = PAL_cppi4QueueOpen(palCppi4Obj, *tdFQueue)) == NULL)
            {
                dbgPrint("\nWARN: PAL_cppi4Init: Unable to open teardown free queue.");
                goto init_failed;
            }

            /* Store the open queue handle for later use */
            palCppi4Obj->dmaBlock[j].tdFQueue = hnd;

            /* Push the just allocated BDs */
            for (i = 0; i < PAL_CPPI41_NUM_TD_DESC; i++, tempBD++)
            {
                tempBD->swDmaNumber = j;
                PAL_cppi4QueuePush(hnd, (Ptr) CPPI4_VIRT_TO_PHYS(tempBD), (sizeof (Cppi4TeardownDesc) - 24)/4, 0);
            }
        }
        /* initialize the DMA scheduler */
        dmaBlock->schedCtrlBase->Control_Reg = ((dmaBlock->schedTable.numEntries - 1) << DMA_SCHED_CTRL_LAST_ENTRY_SHIFT);
        for (i = 0; i < 16; i++)
        {
            dmaBlock->schedTableBase->Sched_Table_Word[i] =dmaBlock->schedTable.entry[(4 * i) ] |
                            (dmaBlock->schedTable.entry[(4 * i) +1] << 8)  |
                            (dmaBlock->schedTable.entry[(4 * i) +2] << 16) |
                            (dmaBlock->schedTable.entry[(4 * i) +3] << 24);
        }
        dmaBlock->schedCtrlBase->Control_Reg |= (1 << DMA_SCHED_CTRL_ENABLE_SHIFT);

        dbgPrint("DMA sched @address %x, value %x\n", dmaBlock->schedCtrlBase->Control_Reg, dmaBlock->schedCtrlBase->Control_Reg);
        dbgPrint("DMA sched @address %p, value %x\n", &initCfg->dmaBlock[j].schedTableBase->Sched_Table_Word[0], initCfg->dmaBlock[j].schedTable.entry[0]);

        dbgPrint("DMA sched initialized\n");
    }
    dbgPrint("Teardown desc initialized\n");

    /* TODO: Do other init as necessary. Can't think of what else to do right now. */

    if (initCfg->debugToolBind)
    {
        if (NULL == cppiDebugDir)
        {
            if (NULL == (cppiDebugDir = proc_mkdir("cppi",     NULL )))
            {
                printk("%s:%d ERROR ....\n",__FUNCTION__,__LINE__);
            }
        }
        initCfg->debugToolBind((PAL_Handle) palCppi4Obj, (Ptr)cppiDebugDir);
    }

    return ((PAL_Handle) palCppi4Obj);

init_failed:
    --Cppi4PALNumInst[(CPPI41_DOMAIN)param];
    return NULL;
}

/*
 * PAL_cppi4Exit
 *  - delete the instance created via PAL_cppi4Init, if no other callers are
 *    holding a handler to the PAL instance (implemented via Cppi4PALNumInst
 *    counter)
 *  - if deleting the last instance, it also puts the NWSS in reset.
 */
PAL_Result PAL_cppi4Exit (PAL_Handle hnd, Ptr param)
{
    Cppi4PALObj *palCppi4Obj = (Cppi4PALObj *) hnd;
    Cppi4InitCfg* initCfg = palCppi4Obj->initCfg;
    Uint32 i;

    /* Check to see if an instance exists in order to exit */
    if (--Cppi4PALNumInst[palCppi4Obj->myDomain] > 0)
    {
        return (PAL_SOK);
    } else if (Cppi4PALNumInst[palCppi4Obj->myDomain] == 0)
    {
        PAL_sysResetCtrl (initCfg->resetLine, IN_RESET);

        for(i = 0; i < PAL_CPPI41_NUM_QUEUE_MGR; i++)
        {
            /* Need to free up the descriptor region only once. Descriptor region 0 for each
             * queue manager contains the base of the memory allocated for all the descriptor
             * regions.
             */
            PAL_osMemFreeSizeAligned(palCppi4Obj->myDomain, palCppi4Obj->qMgrDescRegPtr[i], palCppi4Obj->qMgrDescRegSize[i]);
        }

        /* close the teardown queues */
        for (i = 0; i < PAL_CPPI41_NUM_DMA_BLOCK; i++)
        {
            PAL_cppi4QueueClose(hnd,palCppi4Obj->dmaBlock[i].tdFQueue);
        }

        return (PAL_SOK);
    }

    return (CPPI4_ERR_DEV_NOT_INSTANTIATED);
}

/*
 * PAL_bufPoolInit
 *  - Create a buffer pool for the given index
 */
//Ptr PAL_cppi4BufPoolInit (PAL_Handle hnd, Cppi4BufPool pool, Bool refCntEnable, Uint32 bufSize, Uint32 numBuf)
Ptr PAL_cppi4BufPoolDirectInit (PAL_Handle hnd, Cppi4BufPool pool, Bool refCntEnable, Uint32 bufSize, Uint32 numBuf, Ptr poolBase)
{
    Cppi4PALObj *palCppi4Obj = (Cppi4PALObj *) hnd;
    Cppi4InitCfg* initCfg = palCppi4Obj->initCfg;
    CSL_BufMgr_RegsOvly regs = initCfg->bufMgrBase[pool.bMgr];
    Uint32 size, bufSzOrder, numBufOrder;
    Ptr  buf = NULL;
    Uint32 isVirtAddr = False;

    dbgPrint("%s : regs = 0x%08X\n",__FUNCTION__,regs);

    if (NULL == regs) return NULL;

    for(bufSzOrder = 0; (1 << bufSzOrder) < bufSize; bufSzOrder++);
    for(numBufOrder = 0; (1 << numBufOrder) < numBuf; numBufOrder++);

    /* Basic size contrains from Buf manager doc */
    if(bufSzOrder < 6) return NULL;
    if(numBufOrder < 5) return NULL;

    bufSize = (1 << bufSzOrder);
    numBuf = (1 << numBufOrder);

    size = bufSize * numBuf;

    if (NULL != poolBase)
    {
        buf = poolBase;
    }
    else
    {
        buf = PAL_osMemAllocSizeAligned(palCppi4Obj->myDomain, size);
        isVirtAddr = True;
    }

    /* Hardware requires the base to be naturally aligned to size. */
    if(buf == NULL) return buf;

    palCppi4Obj->bufPoolPtr[pool.bMgr][pool.bPool] = buf;
    palCppi4Obj->bufPoolSize[pool.bMgr][pool.bPool] = size;
    palCppi4Obj->bufSize[pool.bMgr][pool.bPool] = bufSize;
    palCppi4Obj->numBuf[pool.bMgr][pool.bPool] = numBuf;

    /* Allocate the buffer pool */
    regs->Base_Config[pool.bPool].Buffer_Pool_Base = (Uint32)((isVirtAddr) ? CPPI4_VIRT_TO_PHYS(buf) : (CSL_Reg32) (buf));

    /* Enable the pool */
    regs->Base_Config[pool.bPool].Buffer_Pool_Config |= BUFMGR_POOL_ENABLE_MASK;

    /* Program the buffer size */
    regs->Base_Config[pool.bPool].Buffer_Pool_Config &= ~BUFMGR_POOL_BUFFER_SIZE_MASK;
    regs->Base_Config[pool.bPool].Buffer_Pool_Config |= (bufSzOrder - 6) << BUFMGR_POOL_BUFFER_SIZE_SHIFT;

    /* Program the pool size */
    regs->Base_Config[pool.bPool].Buffer_Pool_Config &= ~BUFMGR_POOL_POOL_SIZE_MASK;
    regs->Base_Config[pool.bPool].Buffer_Pool_Config |= (numBufOrder - 5) << BUFMGR_POOL_POOL_SIZE_SHIFT;

    /* enable reference counting on the pool */
    if(refCntEnable)
    {
        regs->Base_Config[pool.bPool].Buffer_Pool_Config |= BUFMGR_POOL_REFCNT_ENABLE_MASK;
    }
    else
    {
        regs->Base_Config[pool.bPool].Buffer_Pool_Config &= ~BUFMGR_POOL_REFCNT_ENABLE_MASK;
    }

    dbgPrint("value at address %p = %x\n", &regs->Base_Config[pool.bPool].Buffer_Pool_Config, regs->Base_Config[pool.bPool].Buffer_Pool_Config);

#if 0
    int i;
    for(i = 0; i < numBuf + 1; i++) {
        dbgPrint("Buffer number: %d, address: %x, size: %d\n", i, regs->Pointer_Size[pool.bPool].Buffer_Pool_Pointer, regs->Pointer_Size[pool.bPool].Buffer_Pool_Size);
    }
#endif

    dbgPrint("Pool[%2d] is at 0x%p of 0x%08X size\n", pool.bPool, buf, size);
    return buf;
}

//Ptr PAL_cppi4BufPoolDirectInit (PAL_Handle hnd, Cppi4BufPool pool, Bool refCntEnable, Uint32 bufSize, Uint32 numBuf, Ptr poolBase)
Ptr PAL_cppi4BufPoolInit (PAL_Handle hnd, Cppi4BufPool pool, Bool refCntEnable, Uint32 bufSize, Uint32 numBuf)
{
    return PAL_cppi4BufPoolDirectInit(hnd,pool,refCntEnable,bufSize,numBuf,NULL);
}


/*
 * PAL_cppi4BufPoolSetBuffers
 *  - Init buffer memory for each buffer
 */
PAL_Result PAL_cppi4BufPoolSetBuffers (PAL_Handle hnd, Cppi4BufPool dstPool, Ptr srcBuf, Uint32 srcBufLen)
{
    Cppi4PALObj *palCppi4Obj = (Cppi4PALObj *) hnd;
    Ptr dstBuf;
    Uint32 i;

    for(i = 0; i < palCppi4Obj->numBuf[dstPool.bMgr][dstPool.bPool]; ++i)
    {
        dstBuf = (Ptr)((Uint32)(palCppi4Obj->bufPoolPtr[dstPool.bMgr][dstPool.bPool]) + (palCppi4Obj->bufSize[dstPool.bMgr][dstPool.bPool])*i);
        memcpy(dstBuf, srcBuf, srcBufLen);
        PAL_CPPI4_CACHE_WRITEBACK(dstBuf, (32+srcBufLen)/32*32);
    }

    return PAL_SOK;
}

/*
 * PAL_cppi4BufIncRefCnt
 *  - Increment the reference count of the valid buffer
 */
PAL_Result PAL_cppi4BufIncRefCnt (PAL_Handle hnd, Cppi4BufPool pool, Ptr bufPtr)
{
    Cppi4PALObj *palCppi4Obj = (Cppi4PALObj *) hnd;
    Cppi4InitCfg* initCfg = palCppi4Obj->initCfg;
    CSL_BufMgr_RegsOvly regs = initCfg->bufMgrBase[pool.bMgr];

    regs->Ref_Cnt_Inc_Val = (Uint32)(((pool.bPool) << 16) | 1);
    regs->Ref_Cnt_Inc_Ptr = (Uint32)bufPtr;

    return PAL_SOK;
}

/*
 * PAL_cppi4BufDecRefCnt
 *  - Decrement the reference count of the valid buffer
 */
PAL_Result PAL_cppi4BufDecRefCnt (PAL_Handle hnd, Cppi4BufPool pool, Ptr bufPtr)
{
    Cppi4PALObj *palCppi4Obj = (Cppi4PALObj *) hnd;
    Cppi4InitCfg* initCfg = palCppi4Obj->initCfg;
    CSL_BufMgr_RegsOvly regs = initCfg->bufMgrBase[pool.bMgr];

    regs->Pointer_Size[pool.bPool].Buffer_Pool_Pointer = (Uint32)bufPtr;

    return PAL_SOK;
}

/*
 * PAL_cppi4BufPopBuf
 *  - Decrement the reference count of the valid buffer
 */
Ptr PAL_cppi4BufPopBuf (PAL_Handle hnd, Cppi4BufPool pool)
{
    Cppi4PALObj *palCppi4Obj = (Cppi4PALObj *) hnd;
    Cppi4InitCfg* initCfg = palCppi4Obj->initCfg;
    CSL_BufMgr_RegsOvly regs = initCfg->bufMgrBase[pool.bMgr];

    return (Ptr)(regs->Pointer_Size[pool.bPool].Buffer_Pool_Pointer);
}

/*
 * PAL_bufPoolInit
 *  - Create a buffer pool for the given index
 */
PAL_Result PAL_cppi4BufPoolDestroy (PAL_Handle hnd, Cppi4BufPool pool)
{
    Cppi4PALObj *palCppi4Obj = (Cppi4PALObj *) hnd;
    Cppi4InitCfg* initCfg = palCppi4Obj->initCfg;
    CSL_BufMgr_RegsOvly regs = initCfg->bufMgrBase[pool.bMgr];

    if(palCppi4Obj->bufPoolPtr[pool.bMgr][pool.bPool] == NULL) {
        return PAL_ERROR_FLAG;
    }

    dbgPrint("freeing memory: %p\n", palCppi4Obj->bufPoolPtr[pool.bMgr][pool.bPool] );

    PAL_osMemFreeSizeAligned(palCppi4Obj->myDomain, palCppi4Obj->bufPoolPtr[pool.bMgr][pool.bPool], palCppi4Obj->bufPoolSize[pool.bMgr][pool.bPool]);

    palCppi4Obj->bufPoolPtr[pool.bMgr][pool.bPool] = NULL;

    /* Disable the pool */
    regs->Base_Config[pool.bPool].Buffer_Pool_Config &= ~BUFMGR_POOL_ENABLE_MASK;
    regs->Base_Config[pool.bPool].Buffer_Pool_Base = 0;

    return PAL_SOK;
}

/*
 * PAL_cppi4AllocDesc
 *  - Alloctes descripor array.
 */
Ptr PAL_cppi4AllocDesc (PAL_Handle hnd, Uint32 qMgr, Uint32 numDesc, Uint32 szDesc)
{
    Ptr buf = 0;
    Cppi4PALObj* palCppi4Obj = (Cppi4PALObj *) hnd;
    Cppi4InitCfg* initCfg = palCppi4Obj->initCfg;
    Cppi4QueueMgrCfg* curMgr = &(initCfg->queueMgrInfo[qMgr]);
    Cppi4DescReg *descRegion = &curMgr->descRegion[0];
    Uint32 i = 0, cookie;

    dbgPrint("requested: numDesc = %d, szDesc = %d\n", numDesc, szDesc);

    PAL_osProtectEntry(PAL_OSPROTECT_INTERRUPT, &cookie);

    /* Traverse the desc region array in reverse order to find the best fit */
    for (i = 0; i < PAL_CPPI41_MAX_DESC_REGIONS; i++)
    {
        if((descRegion[i].isAllocated == False) && (descRegion[i].isOnChip == False)) {
            dbgPrint("Desc region[%d], sz: %d, num: %d\n", i, descRegion[i].szDesc, descRegion[i].numDesc);
            if((descRegion[i].szDesc == szDesc) && (descRegion[i].numDesc == numDesc)) {
                dbgPrint("Got a buffer desc region of base: %p at address %p\n", curMgr->descRegion[i].base, &(curMgr->descRegion[i].base));
                buf = descRegion[i].base;
                descRegion[i].isAllocated = True;
                break;
            }
        }
    }

    PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, cookie);

    return buf;
}

/*
 * PAL_cppi4DeallocDesc
 *  - DeAlloctes descripor array.
 */
PAL_Result PAL_cppi4DeallocDesc (PAL_Handle hnd, Uint32 qMgr, Ptr base)
{
    Cppi4PALObj* palCppi4Obj = (Cppi4PALObj *) hnd;
    Cppi4InitCfg* initCfg = palCppi4Obj->initCfg;
    Cppi4QueueMgrCfg* curMgr = &(initCfg->queueMgrInfo[qMgr]);
    Cppi4DescReg *descRegion = &curMgr->descRegion[0];
    Uint32 i = 0, cookie;
    PAL_Result retVal = PAL_ERROR_FLAG;

    dbgPrint("De-alloc desc called.\n");

    PAL_osProtectEntry(PAL_OSPROTECT_INTERRUPT, &cookie);

    /* Traverse the desc region array in reverse order to find the best fit */
    for (i = 0; i < PAL_CPPI41_MAX_DESC_REGIONS; i++)
    {
        dbgPrint("Desc region[%d], sz: %d, num: %d\n", i, descRegion[i].szDesc, descRegion[i].numDesc);
        if((descRegion[i].isAllocated == True) && (descRegion[i].isOnChip == False)) {
            if(descRegion[i].base == base) {
                dbgPrint("desc region %d, base %p deallocated.\n", i, base);
                descRegion[i].isAllocated = False;
                retVal = PAL_SOK;
                break;
            }
        }
    }

    PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, cookie);

    return retVal;
}

/*
 * PAL_cppi4Control
 * Defines various CPPI 4.1 IOCTLs. Useful for debugging, monitoring etc.
 */
PAL_Result PAL_cppi4Control (PAL_Handle hnd, Uint32 cmd, Ptr cmdArg, Ptr param)
{
    Cppi4PALObj *palCppi4Obj = (Cppi4PALObj *) hnd;
    Cppi4InitCfg * initCfg = palCppi4Obj->initCfg;

    switch (cmd)
    {
        /*
         * TODO: add version query support !@@
         */
#ifdef PAL_CPPI_VER_SUPPORT
        case PAL_CPPI41_IOCTL_GET_SWVER:
        /* cmdArg is an ptr to an integer that will contain the integer version id
         and param is a double ptr to a string which will point to the static
         version string */
        *((Uint32 *) cmdArg) = (Uint32) palCppi4Obj->versionId;
        *((Char **) param) = (Char *) & Cppi4PALVersionString[0];
        break;

        case PAL_CPPI41_IOCTL_GET_HWVER:
        /* cmdArg is a ptr to an integer that will be written with the rev */
        *((Uint32 *) cmdArg) = palCppi4Obj->dmaRegs->Revision;
        /* param is a ptr to an integer that will be written with the rev */
        *((Uint32 *) param) = palCppi4Obj->qmRegs->Revision;
        break;
#endif /* PAL_CPPI_VER_SUPPORT */
        case PAL_CPPI41_IOCTL_GET_FDQ_STARVE_CNT:
        {
            Cppi4Queue* queue = (Cppi4Queue*) cmdArg;
            Uint32 qNum = queue->qNum;
            CSL_Queue_Manager_Region_RegsOvly regs = initCfg->queueMgrInfo[queue->qMgr].queueMgrRgnBase;
            qNum -= initCfg->queueMgrInfo[queue->qMgr].basefdQNum;
            *(Uint32*)param = regs->Free_Desc_Starvation[qNum/4];
        }
        break;
        case PAL_CPPI41_IOCTL_GET_FDBQ_STARVE_CNT:
        {
            Cppi4Queue* queue = (Cppi4Queue*) cmdArg;
            Uint32 qNum = queue->qNum;
            CSL_Queue_Manager_Region_RegsOvly regs = initCfg->queueMgrInfo[queue->qMgr].queueMgrRgnBase;
            qNum -= initCfg->queueMgrInfo[queue->qMgr].basefdbQNum;
            *(Uint32*)param = regs->Free_Desc_Buf_Starvation[qNum/4];
        }
        break;
        case PAL_CPPI41_IOCTL_GET_QUEUE_PEND_STATUS:
        {
            Cppi4Queue* queue = (Cppi4Queue*) cmdArg;
            CSL_Queue_Manager_Region_RegsOvly regs = initCfg->queueMgrInfo[queue->qMgr].queueMgrRgnBase;
            *(Uint32*)param = (regs->Queue_Pending[queue->qNum/32] >> (queue->qNum % 32)) & 0x1;
        }
        break;
        case PAL_CPPI41_IOCTL_GET_QUEUE_ENTRY_COUNT:
        {
            Cppi4Queue* queue = (Cppi4Queue*) cmdArg;
            CSL_Queue_Status_Regs* regs = &initCfg->queueMgrInfo[queue->qMgr].queueStatusRgnBase->Queue_Status[queue->qNum];
            *(Uint32*)param = regs->Queue_Status_Reg_A;
        }
        break;
        case PAL_CPPI41_IOCTL_GET_QUEUE_BYTE_COUNT:
        {
            Cppi4Queue* queue = (Cppi4Queue*) cmdArg;
            CSL_Queue_Status_Regs* regs = &initCfg->queueMgrInfo[queue->qMgr].queueStatusRgnBase->Queue_Status[queue->qNum];
            *(Uint32*)param = regs->Queue_Status_Reg_B;
        }
        break;
        case PAL_CPPI41_IOCTL_GET_QUEUE_HEAD_PKT_SIZE:
        {
            Cppi4Queue* queue = (Cppi4Queue*) cmdArg;
            CSL_Queue_Status_Regs* regs = &initCfg->queueMgrInfo[queue->qMgr].queueStatusRgnBase->Queue_Status[queue->qNum];
            *(Uint32*)param = regs->Queue_Status_Reg_C;
        }
        break;
        case PAL_CPPI41_IOCTL_QUEUE_DIVERT:
        {
            CSL_Queue_Manager_Region_RegsOvly regs = initCfg->queueMgrInfo[*(Uint32 *)param].queueMgrRgnBase;
            regs->Queue_Diversion = (Uint32)cmdArg;
        }
        break;
        case PAL_CPPI41_IOCTL_BUFMGR_SOFT_RESET:
        {
            CSL_BufMgr_RegsOvly regs = initCfg->bufMgrBase[(Uint32)cmdArg];
            regs->Soft_Reset = 1;
        }
        break;
        case PAL_CPPI41_IOCTL_BUF_REFCNT_INCR:
        {
            Uint32 bMgr = ((Uint32) cmdArg >> 8) & 0xF;
            CSL_BufMgr_RegsOvly regs = initCfg->bufMgrBase[bMgr];
            regs->Ref_Cnt_Inc_Val = (Uint32)cmdArg;
            regs->Ref_Cnt_Inc_Ptr = (Uint32)param;
        }
        break;
        default:
        CPPI4_PAL_LOGERR ("\nWARN: PAL_cppi4Control: Unhandled ioctl code %d", cmd);
        break;
    }
    return (PAL_SOK);
}

PAL_Cppi4AccChHnd PAL_cppi4AccChOpen(PAL_Handle hnd, Cppi4AccumulatorCfg* accCfg)
{
    Cppi4PALObj *palCppi4Obj = (Cppi4PALObj *) hnd;
    Cppi4InitCfg * initCfg = palCppi4Obj->initCfg;
    APDSP_Command_Status_RegsOvly cmdRegs = initCfg->apdspInfo.pdspCmdBase;
    Uint32 cookie, i;
    PAL_Cppi4AccChObj *accChObj;

    if (PAL_osMemAlloc(0, sizeof(PAL_Cppi4AccChObj), 0, (Ptr *) &accChObj) != PAL_SOK) {
        CPPI4_PAL_LOGERR ("\nERROR:PAL: PAL_cppi4AccChOpen: Failed to allocate Acc channel object structure.");
        return NULL;
    }

    PAL_osMemSet (accChObj, 0, sizeof (PAL_Cppi4AccChObj));

    PAL_osMemCopy(&accChObj->initCfg, accCfg, sizeof(Cppi4AccumulatorCfg));

    accChObj->palCppi4Obj = hnd;

    /* Need to protect the accumulator register writes. They are shared with pre-fetcher */
    PAL_osProtectEntry(PAL_OSPROTECT_INTERRUPT, &cookie);

    if(accCfg->mode) {
        /* monitor mode */
        cmdRegs->Config_A = (accCfg->queue.qNum) | (accCfg->queue.qMgr << 8) | (accCfg->monitor.pktCountThresh << 16);
        cmdRegs->Config_B = (accCfg->pacingTickCnt) | (accCfg->monitor.pacingMode << 22) | (0x1 << 31);
    } else {
        /* list mode */
        cmdRegs->List_Buffer_Address = CPPI4_VIRT_TO_PHYS((Ptr)accCfg->list.listBase);
        cmdRegs->Config_A = (accCfg->queue.qNum) | (accCfg->queue.qMgr << 8) | (accCfg->list.maxPageEntry << 16);
        cmdRegs->Config_B = (accCfg->pacingTickCnt) | (accCfg->list.maxPageCnt << 16)
                | (accCfg->list.listEntrySize << 18) | (accCfg->list.listCountMode << 20)
                | (accCfg->list.stallAvoidance << 21)| (accCfg->list.pacingMode << 22);
    }
    cmdRegs->Command = (accCfg->accChanNum) | (APDSP_CMD_ENABLE << 8);

    dbgPrint("APDSP config @%p, value %x\n", &cmdRegs->List_Buffer_Address, cmdRegs->List_Buffer_Address);
    dbgPrint("APDSP config @%p, value %x\n", &cmdRegs->Config_A, cmdRegs->Config_A);
    dbgPrint("APDSP config @%p, value %x\n", &cmdRegs->Config_B, cmdRegs->Config_B);
    dbgPrint("APDSP config @%p, value %x\n", &cmdRegs->Command, cmdRegs->Command);

    /* TODO: 1000000 is a magic word picked up from mike's code. Need to understand
     * timeout values and fix the code
     */
    for(i=0; (i < 1000000) && (cmdRegs->Command & (0xFF << 8)); i++);

    if( i==1000000 ) 
    {
        printk("%s[%d] Error: APDSP firmware not responding!\nAPDSP return code: 0x%02X\n", __FUNCTION__ , __LINE__, (cmdRegs->Command & (0xFF << 24)));
        PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, cookie);

        PAL_osMemFree( 0, accChObj, sizeof(PAL_Cppi4AccChObj) );

        return NULL;
    }

    accChObj->curPage = 0;

    PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, cookie);

    return (PAL_Cppi4AccChHnd) accChObj;
}

PAL_Result PAL_cppi4AccChClose(PAL_Cppi4AccChHnd hnd, Ptr closeArgs)
{
    PAL_Cppi4AccChObj *accChObj = (PAL_Cppi4AccChObj *) hnd;
    Cppi4PALObj *palCppi4Obj = accChObj->palCppi4Obj;
    Cppi4InitCfg * initCfg = palCppi4Obj->initCfg;
    APDSP_Command_Status_RegsOvly cmdRegs = initCfg->apdspInfo.pdspCmdBase;
    Uint32 i;

    cmdRegs->List_Buffer_Address = 0;
    cmdRegs->Config_A = 0;
    cmdRegs->Config_B = 0;
    cmdRegs->Command = (accChObj->initCfg.accChanNum) | (APDSP_CMD_DISABLE << 8);

    /* TODO: 1000000 is a magic word picked up from mike's code. Need to understand
     * timeout values and fix the code
     */
    for(i=0; (i < 1000000) && (cmdRegs->Command & (0xFF << 8)); i++);
    if( i==1000000 ) {
        dbgPrint ("\nError: APDSP firmware not responding!");
        dbgPrint("APDSP return code: %x\n", (cmdRegs->Command & (0xFF << 24)));
        return PAL_ERROR_FLAG;
    }

    PAL_osMemFree(0, hnd, sizeof(PAL_Cppi4AccChObj));

    return PAL_SOK;
}

/*
 * Get the next accumulator page
 */
Ptr PAL_cppi4AccChGetNextList(PAL_Cppi4AccChHnd hnd)
{
    PAL_Cppi4AccChObj *accChObj = (PAL_Cppi4AccChObj *) hnd;
    Cppi4AccumulatorCfg* initCfg = &accChObj->initCfg;
    Ptr ret = 0;

    if(initCfg->mode) return NULL; /* no lists in monitor mode */

    /* data available at base + (current page * number of entries per page * size of each entry) */
    ret = initCfg->list.listBase + (accChObj->curPage * initCfg->list.maxPageEntry * (initCfg->list.listEntrySize + 1) * sizeof(Uint32));

    ret = PAL_CPPI4_PHYS_2_VIRT(ret);

    /* invalidate the list page */
    PAL_CPPI4_CACHE_INVALIDATE(ret, initCfg->list.maxPageEntry * (initCfg->list.listEntrySize + 1) * sizeof(Uint32));

    accChObj->curPage++;
    if(accChObj->curPage >= initCfg->list.maxPageCnt) accChObj->curPage = 0;

    return ret;
}


/*
 *  PAL_cppi4TxChOpen
 *    - Verify channel info (range checking etc)
 *    - Allocate memory for the channel
 *    - Channel information stored within PAL structure
 *  \note "chOpenArgs" is not used
 */
PAL_Cppi4TxChHnd PAL_cppi4TxChOpen (PAL_Handle hnd, Cppi4TxChInitCfg* info, Ptr chOpenArgs)
{
    Uint32 chNum = info->chNum, val;
    Cppi4PALObj *palCppi4Obj = (Cppi4PALObj *) hnd;
    Cppi4InitCfg * initCfg = palCppi4Obj->initCfg;
    PAL_Cppi4TxChObj *txChObj;

    if (chNum >= PAL_CPPI41_SR_DMA_MAX_TX_CHANNELS)
        return (NULL);

    if (PAL_osMemAlloc(0, sizeof(PAL_Cppi4TxChObj), 0, (Ptr *) &txChObj) != PAL_SOK) {
        CPPI4_PAL_LOGERR ("\nERROR:PAL: PAL_cppi4TxChOpen: Failed to allocate Tx channel object structure.");
        return NULL;
    }

    PAL_osMemSet (txChObj, 0, sizeof (PAL_Cppi4TxChObj));

    PAL_osMemCopy(&txChObj->initCfg, info, sizeof(Cppi4TxChInitCfg));

    /* Populate channel Obj structure to return to calling function */
    txChObj->palCppi4Obj = hnd;

    /* initialize the hardware */

    /* setup the teardown complete queue */
    val = (info->tdQueue.qMgr << DMA_CHAN_CTRL_TX_GLOBAL_DEF_QMGR_SHIFT) & DMA_CHAN_CTRL_TX_GLOBAL_DEF_QMGR_MASK;
    val |= (info->tdQueue.qNum << DMA_CHAN_CTRL_TX_GLOBAL_DEF_QNUM_SHIFT) & DMA_CHAN_CTRL_TX_GLOBAL_DEF_QNUM_MASK;

    initCfg->dmaBlock[info->dmaNum].chCtrlStatusBase->Channel_Config[chNum].Tx_Global_Config = txChObj->txGlobalConfig = val;

    dbgPrint("initCfg->dmaBlock[info->dmaNum].chCtrlStatusBase->Channel_Config[chNum].Tx_Global_Config address: %p, value: %x\n", &initCfg->dmaBlock[info->dmaNum].chCtrlStatusBase->Channel_Config[chNum].Tx_Global_Config, val);

    /* if host channel, configure the accumulator too. */

    dbgPrint("info structure: %p\n", info);
    dbgPrint("Channel number: %x\n", info->chNum);

    txChObj->isEnabled = False;

    return (PAL_Cppi4TxChHnd) txChObj;
}

/*
 *  PAL_cppi4RxChOpen
 *    - Verify channel info (range checking etc)
 *    - Allocate memory for the channel
*    - Channel information stored within PAL structure
 *  \note "chOpenArgs" is not used
 */
PAL_Cppi4RxChHnd PAL_cppi4RxChOpen (PAL_Handle hnd, Cppi4RxChInitCfg* info, Ptr chOpenArgs)
{
    Uint32 chNum = info->chNum, val, dmaNum = info->dmaNum;
    Cppi4PALObj *palCppi4Obj = (Cppi4PALObj *) hnd;
    Cppi4InitCfg * initCfg = palCppi4Obj->initCfg;
    PAL_Cppi4RxChObj *rxChObj;

    if (chNum >= PAL_CPPI41_SR_DMA_MAX_RX_CHANNELS)
        return (NULL);

    if (PAL_osMemAlloc(0, sizeof(PAL_Cppi4RxChObj), 0, (Ptr *) &rxChObj) != PAL_SOK) {
        CPPI4_PAL_LOGERR ("\nERROR:PAL: PAL_cppi4RxChOpen: Failed to allocate Rx channel object structure.");
        return NULL;
    }

    PAL_osMemSet (rxChObj, 0, sizeof (PAL_Cppi4RxChObj));

    PAL_osMemCopy(&rxChObj->initCfg, info, sizeof(Cppi4RxChInitCfg));

    /* Populate channel Obj structure to return to calling function */
    rxChObj->palCppi4Obj = hnd;

    /* initialize the hardware */

    /* set the sop offset */
    val = (info->sopOffset << DMA_CHAN_CTRL_RX_GLOBAL_SOP_OFFSET_SHIFT) & DMA_CHAN_CTRL_RX_GLOBAL_SOP_OFFSET_MASK;
    /* set the desc type */
    val |= (info->defDescType << DMA_CHAN_CTRL_RX_GLOBAL_DEF_DESC_SHIFT) & DMA_CHAN_CTRL_RX_GLOBAL_DEF_DESC_MASK;
    /* set starvation retry policy */
    val |= (info->retryOnStarvation << DMA_CHAN_CTRL_RX_GLOBAL_ERROR_HANDLING_SHIFT) & DMA_CHAN_CTRL_RX_GLOBAL_ERROR_HANDLING_MASK;
    /* program completion queues */
    val |= (info->rxCompQueue.qMgr << DMA_CHAN_CTRL_RX_GLOBAL_RXCOMP_QMGR_SHIFT) & DMA_CHAN_CTRL_RX_GLOBAL_RXCOMP_QMGR_MASK;
    val |= (info->rxCompQueue.qNum << DMA_CHAN_CTRL_RX_GLOBAL_RXCOMP_QNUM_SHIFT) & DMA_CHAN_CTRL_RX_GLOBAL_RXCOMP_QNUM_MASK;


    initCfg->dmaBlock[dmaNum].chCtrlStatusBase->Channel_Config[chNum].Rx_Global_Config = rxChObj->rxGlobalConfig = val;

    dbgPrint("Rx channel global configuration @ %p value written: %x value read: %x\n", &initCfg->dmaBlock[dmaNum].chCtrlStatusBase->Channel_Config[chNum].Rx_Global_Config, val, initCfg->dmaBlock[dmaNum].chCtrlStatusBase->Channel_Config[chNum].Rx_Global_Config);

    /* set up the packet configuration register based on the descriptor type */

    switch(info->defDescType) {
    case 0: /* embedded desc type */
        val = (info->u.embeddedPktCfg.fdQueue.qMgr << DMA_CHAN_CTRL_RX_EMBEDPKT_FDQ_QMGR_SHIFT) & DMA_CHAN_CTRL_RX_EMBEDPKT_FDQ_QMGR_MASK;
        val |= (info->u.embeddedPktCfg.fdQueue.qNum << DMA_CHAN_CTRL_RX_EMBEDPKT_FDQ_QNUM_SHIFT) & DMA_CHAN_CTRL_RX_EMBEDPKT_FDQ_QNUM_MASK;
        val |= (info->u.embeddedPktCfg.numBufSlot << DMA_CHAN_CTRL_RX_EMBEDPKT_NUM_SLOT_SHIFT) & DMA_CHAN_CTRL_RX_EMBEDPKT_NUM_SLOT_MASK;
        val |= (info->u.embeddedPktCfg.sopSlotNum << DMA_CHAN_CTRL_RX_EMBEDPKT_SOP_SLOT_SHIFT) & DMA_CHAN_CTRL_RX_EMBEDPKT_SOP_SLOT_MASK;
        initCfg->dmaBlock[dmaNum].chCtrlStatusBase->Channel_Config[chNum].Embedded_Pkt_Config_Reg_B = val;

        val = (info->u.embeddedPktCfg.fBufPool[0].bPool << DMA_CHAN_CTRL_RX_EMBEDPKT_FBP0_PNUM_SHIFT) & DMA_CHAN_CTRL_RX_EMBEDPKT_FBP0_PNUM_MASK;
        val |= (info->u.embeddedPktCfg.fBufPool[0].bMgr << DMA_CHAN_CTRL_RX_EMBEDPKT_FBP0_BMGR_SHIFT) & DMA_CHAN_CTRL_RX_EMBEDPKT_FBP0_BMGR_MASK;

        val |= (info->u.embeddedPktCfg.fBufPool[1].bPool << DMA_CHAN_CTRL_RX_EMBEDPKT_FBP1_PNUM_SHIFT) & DMA_CHAN_CTRL_RX_EMBEDPKT_FBP1_PNUM_MASK;
        val |= (info->u.embeddedPktCfg.fBufPool[1].bMgr << DMA_CHAN_CTRL_RX_EMBEDPKT_FBP1_BMGR_SHIFT) & DMA_CHAN_CTRL_RX_EMBEDPKT_FBP1_BMGR_MASK;
        val |= (info->u.embeddedPktCfg.fBufPool[2].bPool << DMA_CHAN_CTRL_RX_EMBEDPKT_FBP2_PNUM_SHIFT) & DMA_CHAN_CTRL_RX_EMBEDPKT_FBP2_PNUM_MASK;
        val |= (info->u.embeddedPktCfg.fBufPool[2].bMgr << DMA_CHAN_CTRL_RX_EMBEDPKT_FBP2_BMGR_SHIFT) & DMA_CHAN_CTRL_RX_EMBEDPKT_FBP2_BMGR_MASK;
        val |= (info->u.embeddedPktCfg.fBufPool[3].bPool << DMA_CHAN_CTRL_RX_EMBEDPKT_FBP3_PNUM_SHIFT) & DMA_CHAN_CTRL_RX_EMBEDPKT_FBP3_PNUM_MASK;
        val |= (info->u.embeddedPktCfg.fBufPool[3].bMgr << DMA_CHAN_CTRL_RX_EMBEDPKT_FBP3_BMGR_SHIFT) & DMA_CHAN_CTRL_RX_EMBEDPKT_FBP3_BMGR_MASK;
        initCfg->dmaBlock[dmaNum].chCtrlStatusBase->Channel_Config[chNum].Embedded_Pkt_Config_Reg_A = val;
        break;

    case 1: /* host desc type */
        val = (info->u.hostPktCfg.fdbQueue[0].qNum << DMA_CHAN_CTRL_RX_HOSTPKT_FDB0_QNUM_SHIFT) & DMA_CHAN_CTRL_RX_HOSTPKT_FDB0_QNUM_MASK;
        val |= (info->u.hostPktCfg.fdbQueue[0].qMgr << DMA_CHAN_CTRL_RX_HOSTPKT_FDB0_QMGR_SHIFT) & DMA_CHAN_CTRL_RX_HOSTPKT_FDB0_QMGR_MASK;
        val |= (info->u.hostPktCfg.fdbQueue[1].qNum << DMA_CHAN_CTRL_RX_HOSTPKT_FDB1_QNUM_SHIFT) & DMA_CHAN_CTRL_RX_HOSTPKT_FDB1_QNUM_MASK;
        val |= (info->u.hostPktCfg.fdbQueue[1].qMgr << DMA_CHAN_CTRL_RX_HOSTPKT_FDB1_QMGR_SHIFT) & DMA_CHAN_CTRL_RX_HOSTPKT_FDB1_QMGR_MASK;
        initCfg->dmaBlock[dmaNum].chCtrlStatusBase->Channel_Config[chNum].Host_Pkt_Config_Reg_A = val;

        dbgPrint("Rx channel ctrl status A @ %p value written: %x value read: %x\n", &initCfg->dmaBlock[dmaNum].chCtrlStatusBase->Channel_Config[chNum].Host_Pkt_Config_Reg_A, val, initCfg->dmaBlock[dmaNum].chCtrlStatusBase->Channel_Config[chNum].Host_Pkt_Config_Reg_A);


        val = (info->u.hostPktCfg.fdbQueue[2].qNum << DMA_CHAN_CTRL_RX_HOSTPKT_FDB2_QNUM_SHIFT) & DMA_CHAN_CTRL_RX_HOSTPKT_FDB2_QNUM_MASK;
        val |= (info->u.hostPktCfg.fdbQueue[2].qMgr << DMA_CHAN_CTRL_RX_HOSTPKT_FDB2_QMGR_SHIFT) & DMA_CHAN_CTRL_RX_HOSTPKT_FDB2_QMGR_MASK;
        val |= (info->u.hostPktCfg.fdbQueue[3].qNum << DMA_CHAN_CTRL_RX_HOSTPKT_FDB3_QNUM_SHIFT) & DMA_CHAN_CTRL_RX_HOSTPKT_FDB3_QNUM_MASK;
        val |= (info->u.hostPktCfg.fdbQueue[3].qMgr << DMA_CHAN_CTRL_RX_HOSTPKT_FDB3_QMGR_SHIFT) & DMA_CHAN_CTRL_RX_HOSTPKT_FDB3_QMGR_MASK;
        initCfg->dmaBlock[dmaNum].chCtrlStatusBase->Channel_Config[chNum].Host_Pkt_Config_Reg_B = val;

        dbgPrint("Rx channel ctrl status B @ %p value written: %x value read: %x\n", &initCfg->dmaBlock[dmaNum].chCtrlStatusBase->Channel_Config[chNum].Host_Pkt_Config_Reg_B, val, initCfg->dmaBlock[dmaNum].chCtrlStatusBase->Channel_Config[chNum].Host_Pkt_Config_Reg_B);

        break;

    case 2: /* monolithic desc type */
        val = (info->u.monolithicPktCfg.fdQueue.qNum << DMA_CHAN_CTRL_RX_MONOLITHICPKT_FDQ_QNUM_SHIFT) & DMA_CHAN_CTRL_RX_MONOLITHICPKT_FDQ_QNUM_MASK;
        val |= (info->u.monolithicPktCfg.fdQueue.qMgr << DMA_CHAN_CTRL_RX_MONOLITHICPKT_FDQ_QMGR_SHIFT) & DMA_CHAN_CTRL_RX_MONOLITHICPKT_FDQ_QMGR_MASK;
        val |= (info->u.monolithicPktCfg.sopOffset << DMA_CHAN_CTRL_RX_MONOLITHICPKT_SOP_OFFSET_SHIFT) & DMA_CHAN_CTRL_RX_MONOLITHICPKT_SOP_OFFSET_MASK;
        initCfg->dmaBlock[dmaNum].chCtrlStatusBase->Channel_Config[chNum].Monolithic_Pkt_Config_Reg_A = val;
        break;
    }

    rxChObj->isEnabled = False;

    return (PAL_Cppi4RxChHnd) rxChObj;
}
/*
 * PAL_cppi4TxChStatus
 *    - Returns when the channel teardown is complete.
 *    - Used when Teardown descriptor is forwarded back to Free teardown queue.
 *   - Mostly used in case of Teardown of Embedded endpoint Channels and Infra mode channels
 */
PAL_Result PAL_cppi4TxChStatus (PAL_Cppi4TxChHnd hnd, Ptr chCloseArgs)
{

    PAL_Cppi4TxChObj *txChObj = (PAL_Cppi4TxChObj *) hnd;
    Cppi4PALObj *palCppi4Obj = txChObj->palCppi4Obj;
    Cppi4InitCfg * initCfg = palCppi4Obj->initCfg;
    Uint32 chNum = txChObj->initCfg.chNum, dmaNum = txChObj->initCfg.dmaNum;
    Uint32 status;
    Uint32 count = 0;
    do
    {
        status = initCfg->dmaBlock[dmaNum].chCtrlStatusBase->Channel_Config[chNum].Tx_Global_Config;
        status &= CPPI4_CH_ENABLE_MASK;
        count++;
    }while(status && count < 10000);
    if(count >= 10000 )
    {
        return PAL_ERROR_FLAG;
        CPPI4_PAL_LOGERR ("\nERROR:PAL: PAL_cppi4TxChStatus: Failed to Teardown Channel");
    }

    return PAL_SOK;
}

/*
 * PAL_cppi4TxChClose
 *    - Teardown a given Tx channel
 */
PAL_Result PAL_cppi4TxChClose (PAL_Cppi4TxChHnd hnd, Ptr chCloseArgs) {
    PAL_Cppi4TxChObj *txChObj = (PAL_Cppi4TxChObj *) hnd;
    Cppi4PALObj *palCppi4Obj = txChObj->palCppi4Obj;
    Cppi4InitCfg * initCfg = palCppi4Obj->initCfg;
    Uint32 chNum = txChObj->initCfg.chNum, dmaNum = txChObj->initCfg.dmaNum;

    /* initiate channel teardown */
    txChObj->txGlobalConfig |= (1 << DMA_CHAN_CTRL_TX_GLOBAL_CHAN_TD_SHIFT);
    initCfg->dmaBlock[dmaNum].chCtrlStatusBase->Channel_Config[chNum].Tx_Global_Config = txChObj->txGlobalConfig;

    dbgPrint("txChObj->txGlobalConfig: %x\n", txChObj->txGlobalConfig);

    return (PAL_SOK);
}

/*
 * PAL_cppi4TxChClose
 *    - Free all stored channel information for the given Transmit channel
 */
PAL_Result PAL_cppi4TxChDestroy (PAL_Cppi4TxChHnd hnd, Ptr chCloseArgs)
{
    PAL_Cppi4TxChObj *txChObj = (PAL_Cppi4TxChObj *) hnd;

    /* free up the handle itself */
    PAL_osMemFree(0, txChObj, sizeof(PAL_Cppi4TxChObj));

    return PAL_SOK;
}

/*
 * PAL_cppi4RxChStatus
 *    - Returns when the channel teardown is complete.
 *    - Used when Teardown descriptor is forwarded back to Free teardown queue.
 *   - Mostly used in case of Teardown of Embedded endpoint Channels and Infra mode channels
 */
PAL_Result PAL_cppi4RxChStatus (PAL_Cppi4RxChHnd hnd, Ptr chCloseArgs)
{
    PAL_Cppi4RxChObj *rxChObj = (PAL_Cppi4RxChObj *) hnd;
    Cppi4PALObj *palCppi4Obj = rxChObj->palCppi4Obj;
    Cppi4InitCfg * initCfg = palCppi4Obj->initCfg;
    Uint32 chNum = rxChObj->initCfg.chNum, dmaNum = rxChObj->initCfg.dmaNum;
    Uint32 status;
    Uint32 count = 0;

    do
    {
        status = initCfg->dmaBlock[dmaNum].chCtrlStatusBase->Channel_Config[chNum].Rx_Global_Config;
        status &= CPPI4_CH_ENABLE_MASK;
        count++;
    }while(status && count < 10000);
    if(count >= 10000 )
    {
        return PAL_ERROR_FLAG;
        CPPI4_PAL_LOGERR ("\nERROR:PAL: PAL_cppi4RxChStatus: Failed to Teardown Channel");
    }
    return PAL_SOK;
}

/*
 * PAL_cppi4RxChClose
 *    - Teardown the given Receive channel
 */
PAL_Result PAL_cppi4RxChClose (PAL_Cppi4RxChHnd  hnd, Ptr chCloseArgs) {
    PAL_Cppi4RxChObj *rxChObj = (PAL_Cppi4RxChObj *) hnd;
    Cppi4PALObj *palCppi4Obj = rxChObj->palCppi4Obj;
    Cppi4InitCfg * initCfg = palCppi4Obj->initCfg;
    Uint32 chNum = rxChObj->initCfg.chNum, dmaNum = rxChObj->initCfg.dmaNum;

    /* initiate channel teardown */
    rxChObj->rxGlobalConfig |= (1 << DMA_CHAN_CTRL_RX_GLOBAL_CHAN_TD_SHIFT);
    initCfg->dmaBlock[dmaNum].chCtrlStatusBase->Channel_Config[chNum].Rx_Global_Config = rxChObj->rxGlobalConfig;

    return (PAL_SOK);
}

/*
 * PAL_cppi4RxChDestroy
 *    - Free all stored channel information for the given Receive channel
 */
PAL_Result PAL_cppi4RxChDestroy (PAL_Cppi4RxChHnd  hnd, Ptr chCloseArgs)
{
    PAL_Cppi4RxChObj *rxChObj = (PAL_Cppi4RxChObj *) hnd;

    /* free up the handle itself */
    PAL_osMemFree(0, rxChObj, sizeof(PAL_Cppi4RxChObj));

    return (PAL_SOK);
}

/*
 *  PAL_cppi4EnableRxChannel
 *    - Channel DMA is enabled in hardware. Ready for data reception.
 */
PAL_Result PAL_cppi4EnableRxChannel (PAL_Cppi4RxChHnd hnd, Ptr chCloseArgs)
{
    PAL_Cppi4RxChObj *rxChObj = (PAL_Cppi4RxChObj *) hnd;
    Cppi4PALObj *palCppi4Obj = rxChObj->palCppi4Obj;
    Cppi4InitCfg * initCfg = palCppi4Obj->initCfg;
    Uint32 chNum = rxChObj->initCfg.chNum, dmaNum = rxChObj->initCfg.dmaNum;

    rxChObj->rxGlobalConfig |= (1 << DMA_CHAN_CTRL_RX_GLOBAL_CHAN_ENABLE_SHIFT);

    dbgPrint("PAL_cppi4EnableRxChannel: Rx global config: %x\n", rxChObj->rxGlobalConfig);

    initCfg->dmaBlock[dmaNum].chCtrlStatusBase->Channel_Config[chNum].Rx_Global_Config = rxChObj->rxGlobalConfig;

    /* Mark channel open */
    rxChObj->isEnabled = True;

    return (PAL_SOK);
}

/*
 *  PAL_cppi4EnableTxChannel
 *    - Channel DMA is enabled in hardware. Ready for data transmission.
 */
PAL_Result PAL_cppi4EnableTxChannel (PAL_Cppi4TxChHnd hnd, Ptr chCloseArgs)
{
    PAL_Cppi4TxChObj *txChObj = (PAL_Cppi4TxChObj *) hnd;
    Cppi4PALObj *palCppi4Obj = txChObj->palCppi4Obj;
    Cppi4InitCfg * initCfg = palCppi4Obj->initCfg;
    Uint32 chNum = txChObj->initCfg.chNum, dmaNum = txChObj->initCfg.dmaNum;

    txChObj->txGlobalConfig |= (1 << DMA_CHAN_CTRL_TX_GLOBAL_CHAN_ENABLE_SHIFT);
    initCfg->dmaBlock[dmaNum].chCtrlStatusBase->Channel_Config[chNum].Tx_Global_Config = txChObj->txGlobalConfig;

    dbgPrint("Enabling channel at address %p\n", &initCfg->dmaBlock[dmaNum].chCtrlStatusBase->Channel_Config[chNum].Tx_Global_Config);

    /* Mark channel open */
    txChObj->isEnabled = True;

    return (PAL_SOK);
}

/*
 *  PAL_cppi4DisableTxChannel
 *    - Channel DMA is disabled in hardware.
 *
 */
PAL_Result PAL_cppi4DisableTxChannel (PAL_Cppi4TxChHnd hnd, Ptr chCloseArgs)
{
    PAL_Cppi4TxChObj *txChObj = (PAL_Cppi4TxChObj *) hnd;
    Cppi4PALObj* palCppi4Obj = txChObj->palCppi4Obj;
    Cppi4InitCfg* initCfg = palCppi4Obj->initCfg;
    Uint32 chNum = txChObj->initCfg.chNum, dmaNum = txChObj->initCfg.dmaNum;


    txChObj->txGlobalConfig &=  ~(1 << DMA_CHAN_CTRL_TX_GLOBAL_CHAN_ENABLE_SHIFT);

    initCfg->dmaBlock[dmaNum].chCtrlStatusBase->Channel_Config[chNum].Tx_Global_Config = txChObj->txGlobalConfig;

    /* Mark channel open */
    txChObj->isEnabled = False;

    return (PAL_SOK);
}

/*
 *  PAL_cppi4DisableRxChannel
 *    - Channel DMA is disabled in hardware.
 */
PAL_Result PAL_cppi4DisableRxChannel (PAL_Cppi4RxChHnd hnd, Ptr chCloseArgs)
{
    PAL_Cppi4RxChObj *rxChObj = (PAL_Cppi4RxChObj *) hnd;
    Cppi4PALObj *palCppi4Obj = rxChObj->palCppi4Obj;
    Cppi4InitCfg * initCfg = palCppi4Obj->initCfg;
    Uint32 chNum = rxChObj->initCfg.chNum, dmaNum = rxChObj->initCfg.dmaNum;

    rxChObj->rxGlobalConfig &= ~(1 << DMA_CHAN_CTRL_RX_GLOBAL_CHAN_ENABLE_SHIFT);

    initCfg->dmaBlock[dmaNum].chCtrlStatusBase->Channel_Config[chNum].Rx_Global_Config = rxChObj->rxGlobalConfig;
            ;

    /* Mark channel open */
    rxChObj->isEnabled = False;

    return (PAL_SOK);
}

/*
 *  PAL_cppi4QueueOpen
 *  - Opens a CPPI4 queue for use.
 *  - The handle returned should be used for all push and pop
 *  operations on the queue.
 */
PAL_Cppi4QueueHnd PAL_cppi4QueueOpen (PAL_Handle hnd, Cppi4Queue  queue)
{
    Cppi4PALObj *palCppi4Obj = (Cppi4PALObj *) hnd;
    Cppi4InitCfg * initCfg = palCppi4Obj->initCfg;
    PAL_Cppi4QueueObj* queueObj;
    Uint32 retVal;
    Uint32 lockKey;

    /* Putting the whole process under lock since we are accessing the CPPI shared resources
     */
    PAL_osProtectEntry(PAL_OSPROTECT_INTERRUPT, &lockKey);
    if(palCppi4Obj->isQueueOpen[queue.qMgr][queue.qNum])
    {
        queueObj = (PAL_Cppi4QueueHnd) palCppi4Obj->qHnd[queue.qMgr][queue.qNum];
        queueObj->queueOpenCount++;
        PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);
        return (PAL_Cppi4QueueHnd) palCppi4Obj->qHnd[queue.qMgr][queue.qNum];
    }
    else
    {
        /* allocate the Queue object structure */
        retVal = PAL_osMemAlloc(0, sizeof(PAL_Cppi4QueueObj), 0, (Ptr *) &queueObj);
        if (retVal != PAL_SOK)
        {
            CPPI4_PAL_LOGERR ("\nERROR:PAL: PAL_cppi4QueueOpen: Failed to de-allocate descriptor memory");
            PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);
            return NULL;
        }

        PAL_osMemSet (queueObj, 0, sizeof (PAL_Cppi4QueueObj));

        queueObj->palCppi4Obj = hnd;
        queueObj->queue = queue;
        queueObj->baseAddress       = &initCfg->queueMgrInfo[queue.qMgr].queueMgmtRgnBase->Queue_Mgmt[queue.qNum];
        if (initCfg->queueMgrInfo[queue.qMgr].queueProxyRgnBase)
        {
            queueObj->baseAddressProxy  = &initCfg->queueMgrInfo[queue.qMgr].queueProxyRgnBase->Queue_Mgmt[queue.qNum];
        }

        PAL_cppi4QueuePush(queueObj, 0, 0, 0);

        palCppi4Obj->isQueueOpen[queue.qMgr][queue.qNum] = TRUE;
        queueObj->queueOpenCount++;
        palCppi4Obj->qHnd[queue.qMgr][queue.qNum] = (Uint32)queueObj;

        PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);
        return (PAL_Cppi4QueueHnd) (queueObj);
    }
}

/*
 *  PAL_cppi4QueueClose
 *  - Closes a CPPI4 queue.
 */
PAL_Result PAL_cppi4QueueClose (PAL_Handle hnd, PAL_Cppi4QueueHnd qHnd)
{
    Int retVal;
    Cppi4PALObj *palCppi4Obj = (Cppi4PALObj *) hnd;
    PAL_Cppi4QueueObj* queueObj;
    Uint32 lockKey;

    queueObj = (PAL_Cppi4QueueHnd) qHnd;

    /* Disable irqs since we are accessing the CPPI shared resources
     */
    PAL_osProtectEntry(PAL_OSPROTECT_INTERRUPT, &lockKey);
    /* allocate the Queue object structure */
    if(!palCppi4Obj->isQueueOpen[queueObj->queue.qMgr][queueObj->queue.qNum])
    {
        CPPI4_PAL_LOGERR("\nERROR: PAL_cppi4QueueClose: Queue is already closed\n");
        PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);
        return CPPI4_ERR_DEV_ALREADY_CLOSED;
    }
    else
    {
        queueObj->queueOpenCount--;
    }

    if(queueObj->queueOpenCount == 0 )
    {
        palCppi4Obj->isQueueOpen[queueObj->queue.qMgr][queueObj->queue.qNum] = 0;
        palCppi4Obj->qHnd[queueObj->queue.qMgr][queueObj->queue.qNum] = (Uint32)NULL;
        retVal = PAL_osMemFree(0, qHnd, sizeof(PAL_Cppi4QueueObj));
        if (retVal != PAL_SOK)
        {
            CPPI4_PAL_LOGERR ("\nERROR:PAL: PAL_cppi4QueueClose: Failed to de-allocate descriptor memory");
            PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);
            return PAL_OS_ERROR_NO_RESOURCES;
        }
    }
    PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);
    return PAL_SOK;
}

/*
 * PAL_cppi4QueuePush
 *  - Pushes a descriptor to the given queue
 */
PAL_Result PAL_cppi4QueuePush (PAL_Cppi4QueueHnd hnd, Ptr dAddr, Uint32 dSize, Uint32 pSize)
{
    PAL_Cppi4QueueObj* qObj = (PAL_Cppi4QueueObj*) hnd;
    CSL_Queue_Mgmt_Regs* regs = qObj->baseAddressProxy;
    Uint32 tmp;
    Uint32 lockKey;

    /* Write to tail of the queue. TODO: Cant think of a reason why a queue to head
     * may be required. If it is, the API may have to be extended.
     * Also, cant understand why packet size is required to queue up a descriptor. The spec
     * says packet size *must* be written prior to the packet write operaration
     */

    tmp = (dSize << QMGR_QUEUE_N_REG_D_DESCSZ_SHIFT) & QMGR_QUEUE_N_REG_D_DESCSZ_MASK;
    tmp|= ((Uint32)dAddr & QMGR_QUEUE_N_REG_D_DESC_ADDR_MASK);


    dbgPrint("Pushing value: %x to queue %p\n", tmp, regs);

    if (qObj->baseAddressProxy)
    {
        regs = qObj->baseAddressProxy;
        PAL_osProtectEntry(PAL_OSPROTECT_INTERRUPT, &lockKey);
        regs->Queue_Reg_C = pSize;
        regs->Queue_Reg_D = tmp;
        PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);
    }
    else
    {
        regs = qObj->baseAddress;
        regs->Queue_Reg_D = tmp;
    }

    return (PAL_SOK);
}

/*
 * PAL_cppi4QueuePop
 *  - Pops a descriptor from a given queue
 */
PAL_Cppi4BD *PAL_cppi4QueuePop (PAL_Cppi4QueueHnd hnd)
{
    CSL_Queue_Mgmt_Regs* regs = ((PAL_Cppi4QueueObj *)hnd)->baseAddress;

    dbgPrint ("Poping from Q address %p\n", regs);

    return (PAL_Cppi4BD*) (regs->Queue_Reg_D & QMGR_QUEUE_N_REG_D_DESC_ADDR_MASK);
}

/*
 * PAL_cppi4GetTdInfo
 *  - Takes a Teardown descriptor and gives back the teardown info
 */
PAL_Result PAL_cppi4GetTdInfo(PAL_Handle hnd, PAL_Cppi4BD * cppi4TdBD, Bool* txRx, Uint32*  dmaNum, Uint32* chanNum)
{
    Cppi4TeardownDesc* tdBD = (Cppi4TeardownDesc*) cppi4TdBD;
    Cppi4PALObj *palCppi4Obj = (Cppi4PALObj *)hnd;
    Uint32 dmaNumPvt;

    Uint32 type;

    type = (tdBD->tdInfo & PAL_CPPI4_TDDESC_DESC_TYPE_MASK) >> PAL_CPPI4_TDDESC_DESC_TYPE_SHIFT;

    if(type != PAL_CPPI4_TDDESC_DESC_TYPE_TD)
        return (CPPI4_ERR_INVALID_DESC_TYPE);

    if(txRx) *txRx = (tdBD->tdInfo & CPPI4_TD_DESC_TX_RX_MASK) >> CPPI4_TD_DESC_TX_RX_SHIFT;
    /* dma number is taken from the software populated field and it was set during initial
     * push of the teardown descriptors onto the td descriptor free queue.  Hardware is  not
     * giving the current dma number as of now
     */
    dmaNumPvt = tdBD->swDmaNumber;
    //dmaNumPvt = (tdBD->tdInfo & CPPI4_TD_DESC_DMA_NUM_MASK) >> CPPI4_TD_DESC_DMA_NUM_SHIFT;
    if(dmaNum) *dmaNum = dmaNumPvt;
    if(chanNum) *chanNum = (tdBD->tdInfo & CPPI4_TD_DESC_CHAN_NUM_MASK) >> CPPI4_TD_DESC_CHAN_NUM_SHIFT;

    PAL_cppi4QueuePush(palCppi4Obj->dmaBlock[dmaNumPvt].tdFQueue, cppi4TdBD, (sizeof(Cppi4TeardownDesc) - 24)/4, 0);

    return (PAL_SOK);

}

