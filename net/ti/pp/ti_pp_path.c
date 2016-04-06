    /*
 * Copyright (C) <2008>, Texas Instruments, Incorporated
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

#define _HIL_PP_PATH_C_

/*! \file ti_pp_path.c
    \brief
        This file contains the implementation of the PP Path counters (PPP).
        The PPP provides a service targeted at virtual network drives.
        It provides counters (In/Out Packets/Octets) for a virtual network
        drivers situated between accelerated physical network drivers.
*/

/**************************************************************************/
/*      INCLUDES:                                                         */
/**************************************************************************/

#include <linux/kernel.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/list.h>
#include <linux/string.h>
#include <linux/if.h>
#include <linux/proc_fs.h>


#include <asm-arm/arch-avalanche/generic/_tistdtypes.h>



#include <asm-arm/arch-avalanche/generic/pal.h>
#include "linux/ti_pp_path.h"
#include "linux/ti_ppm.h"

/**************************************************************************/
/*      EXTERNS Declaration:                                              */
/**************************************************************************/

/**************************************************************************/
/*      DEFINES:                                                          */
/**************************************************************************/

#define DLOG(fmt, ...) if (PPP_enableDebug) printk(KERN_NOTICE "PPP: " fmt "\n", ## __VA_ARGS__)
//#define DLOG(fmt, ...) printk("PPP_DBG: " fmt "\n", ## __VA_ARGS__)
#define WLOG(fmt, ...) printk(KERN_WARNING "PPP: " fmt "\n", ## __VA_ARGS__)
#define ELOG(fmt, ...) printk(KERN_ERR "PPP: "  fmt "\n", ## __VA_ARGS__)

#define PPP_IS_LEGAL_SESSION_HANDLE(s) ((s) < TI_PP_MAX_ACCLERABLE_SESSIONS)
#define PPP_IS_LEGAL_PATHDIR(p) (((p) == PPP_PATHDIR_IN) || ((p) == PPP_PATHDIR_OUT))
#define PPP_SESSION_HANDLE_FROM_PTR(p) ((p) - PPP_sessions)

#define TOKENIZE(varN, sepS, startP, endP, errS) \
    /* Parse line */                             \
    varN = startP;                               \
    /* Skip sep */                               \
    varN += strspn(varN, sepS);                  \
    /* Find end of token */                      \
    endP = varN + strcspn(varN, sepS);           \
    if (varN == endP)                            \
    {                                            \
        ELOG(errS);                              \
        return -EFAULT;                          \
    }                                            \
    /* Terminate */                              \
    *endP = '\0';                                \
    DLOG("param : \"%s\"", varN);

/**************************************************************************/
/*      LOCAL DECLARATIONS:                                               */
/**************************************************************************/

typedef unsigned int sessionHandle_t;

/*! \var typedef struct PPP_path_t
    \brief DB of paths
*/
typedef struct
{
    char rxFromDev[IFNAMSIZ];
    char txToDev[IFNAMSIZ];
    PPP_pathDir_e pathDir;
    PPP_uniCounetrs_t deadSessionCtr;
    struct list_head headSessionsOnPath;        /* Head */
    struct list_head headPathPathsOnVirtDev;    /* Head */
    struct list_head listPaths;                 /* Element */
} PPP_path_t;

/*! \var typedef struct PPP_virtualDev_t
    \brief DB of (via) virtual devices
*/
typedef struct
{
    char virtualDev[IFNAMSIZ];
    struct list_head headVirtDevPathsOnVirtDev;  /* Head */
    struct list_head listVirtualDevs;           /* Element */
} PPP_virtualDev_t;

/*! \var typedef struct PPP_pathOnVirtualDev_t
    \brief Path on virtual device
*/
typedef struct
{
    PPP_path_t *path;
    struct list_head listVirtDevPathsOnVirtDev;  /* Element */
    struct list_head listPathPathsOnVirtDev;    /* Element */
} PPP_pathOnVirtualDev_t;


/*! \var typedef struct PPP_session_t
    \brief Session
*/
typedef struct
{
    PPP_path_t *path;
    struct list_head listSessionsOnPath;        /* Element */
} PPP_session_t;

/**************************************************************************/
/*      LOCAL VARIABLES:                                                  */
/**************************************************************************/

/* Root of list of paths */
static LIST_HEAD(PPP_headPaths);

/* Root of list of virtual devices */
static LIST_HEAD(PPP_headVirtualDevs);

/* Sessions */
static PPP_session_t PPP_sessions[TI_PP_MAX_ACCLERABLE_SESSIONS];

/* PP event CB */
static unsigned int PPP_event_handler_handle;

/* procs */
static struct proc_dir_entry *pppProc;
static struct proc_dir_entry *pppProcDbg;

/* Debug */
static Bool PPP_enableDebug = False;

/* Local functions */
static PPP_path_t * PPP_GetPath(char *rxFrom, char *txTo);
static PPP_virtualDev_t * PPP_GetVirtDev(char *viaVirtualDev);
static int PPP_DelSessionEntry(Uint32 sessionHandle);
static int PPP_AddSession(sessionHandle_t sessionHandle, struct sk_buff *skb);
static int PPP_DelSession(sessionHandle_t sessionHandle, TI_PP_SESSION_STATS *sessionCounters);
static PPP_pathOnVirtualDev_t *PPP_GetVirtPathConnector(PPP_virtualDev_t *virtDev, PPP_path_t *path);
static void PPP_event_handler(unsigned int event_id, unsigned int param1, unsigned int param2);
static int PPP_Show(char* buf, char **start, off_t offset, int count, int *eof, void *data);
static int PPP_WrProc (struct file *file, const char *buffer, unsigned long count, void *data);
static int PPP_TraverseVirtDev(char *viaVirtualDev, PPP_counters_t *outPppCtrs, int limitLen, char *outBuf);
static int PPP_WrProcDbg (struct file *file, const char *buffer, unsigned long count, void *data);
static int PPP_RdProcDbg(char* page, char **start, off_t offset, int count, int *eof, void *data);

/**************************************************************************/
/*      INTERFACE FUNCTIONS Implementation:                               */
/**************************************************************************/

/**************************************************************************/
/*! \fn int PPP_Init(void)
 **************************************************************************
 *  \brief Init the PPP module
 *  \return 0 (OK) / 1 (NOK)
 **************************************************************************/
int PPP_Init(void)
{
    Uint32 i;

    /* Disable debug */
    PPP_enableDebug = False;

    /* Init DB */
    memset(PPP_sessions, 0, sizeof(PPP_sessions));
    for (i = 0; i < TI_PP_MAX_ACCLERABLE_SESSIONS; i++)
    {
        /* Initialize list */
        INIT_LIST_HEAD(&PPP_sessions[i].listSessionsOnPath);
    }

    /* Create procs */
    pppProc = create_proc_entry("ti_pp_path" ,0644, init_net.proc_net);
    if (pppProc == NULL)
    {
        ELOG("Unable to create the PPP proc entries");
        return -1;
    }
    pppProc->data = NULL;
    pppProc->read_proc = PPP_Show;
    pppProc->write_proc = PPP_WrProc;


    pppProcDbg = create_proc_entry("net/ti_pp_path_dbg" ,0644, NULL);
    if (pppProcDbg == NULL)
    {
        ELOG("Unable to create the PPP DBG proc entries");
        return -1;
    }
    pppProcDbg->data = NULL;
    pppProcDbg->read_proc = PPP_RdProcDbg;
    pppProcDbg->write_proc = PPP_WrProcDbg;

    /* Register event */
    PPP_event_handler_handle = ti_ppm_register_event_handler (PPP_event_handler);
    if (PPP_event_handler_handle == 0)
    {
        ELOG("Could not register PP event handler");
        return -1;
    }

    return 0;
}

/**************************************************************************/
/*! \fn int PPP_AddPath(char *rxFrom, char *txTo, char *viaVirtualDev, PPP_pathDir_e pathDir)
 **************************************************************************
 *  \brief Add a path
 *  \param[in] rxFrom - input device
 *  \param[in] txTo - output device
 *  \param[in] viaVirtualDev - virtual device through which the traffic flows
 *  \param[in] pathDir - does the count belong to the IN (to the box) or OUT (of the box)
 *  \return 0 (OK) / err (NOK)
 **************************************************************************/
int PPP_AddPath(char *rxFrom, char *txTo, char *viaVirtualDev, PPP_pathDir_e pathDir)
{
    Uint32 lockKey;
    PPP_path_t *currPath;
    PPP_virtualDev_t *currVirtDev;
    PPP_pathOnVirtualDev_t *currPathOnVirtdev;
    Bool newPath = False;
    Bool newVirtDev = False;

    if ((rxFrom == NULL) || (txTo == NULL) || (viaVirtualDev == NULL) || !PPP_IS_LEGAL_PATHDIR(pathDir))
    {
        ELOG("Illegal param: rxFrom %p, txTo %p, viaVirtualDev %p, pathDir %d",
             rxFrom, txTo, viaVirtualDev, pathDir);
        return -1;
    }

    DLOG("Add path %s -(%s)-> %s (%s)",
         rxFrom, viaVirtualDev, txTo,
         pathDir == PPP_PATHDIR_OUT ? "OUT" : "IN");

    PAL_osProtectEntry(PAL_OSPROTECT_INTERRUPT, &lockKey);

    /* Does the path exist? */
    currPath = PPP_GetPath(rxFrom, txTo);
    if (currPath == NULL)
    {
        /* Path does not exist, add a new path */

        DLOG("Path does not exist, creating...");

        currPath = kmalloc(sizeof(*currPath), GFP_KERNEL);
        if (currPath == NULL)
        {
            PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);
            /* Cannot alloc mem, reject */
            WLOG("Cannot kmalloc(%d, GPF_KERNEL) for path : rxFrom %s, txTo %s, virtDev %s",
                 sizeof(*currPath), rxFrom, txTo, viaVirtualDev);
            return ENOMEM;
        }

        /* Init path */
        strncpy(currPath->rxFromDev, rxFrom, sizeof(currPath->rxFromDev));
        strncpy(currPath->txToDev, txTo, sizeof(currPath->txToDev));
        currPath->pathDir = pathDir;
        currPath->deadSessionCtr.octets = 0;
        currPath->deadSessionCtr.packets = 0;
        INIT_LIST_HEAD(&currPath->headSessionsOnPath);
        INIT_LIST_HEAD(&currPath->headPathPathsOnVirtDev);

        /* Connect path to path list */
        list_add(&currPath->listPaths, &PPP_headPaths);

        newPath = True;
    }

    /* Does the virtual device exist? */
    currVirtDev = PPP_GetVirtDev(viaVirtualDev);
    if (currVirtDev == NULL)
    {
        /* Virtual device does not exist, add */

        DLOG("virtDev does not exist, creating...");

        currVirtDev = kmalloc(sizeof(*currVirtDev), GFP_KERNEL);
        if (currVirtDev == NULL)
        {
            /* If created the path here, remove it */
            if (newPath)
            {
                /* Could not create complete path, cleanup */
                PPP_DelPath(rxFrom, txTo, viaVirtualDev);
            }
            PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);
            /* Cannot alloc mem, reject */
            WLOG("Cannot kmalloc(%d, GPF_KERNEL) for virtual device : rxFrom %s, txTo %s, virtDev %s",
                 sizeof(*currVirtDev), rxFrom, txTo, viaVirtualDev);
            return ENOMEM;
        }

        /* Created virtual device, init and connect */
        strncpy(currVirtDev->virtualDev, viaVirtualDev, sizeof(currVirtDev->virtualDev));
        INIT_LIST_HEAD(&currVirtDev->headVirtDevPathsOnVirtDev);

        /* Add to list of virtual devices */
        list_add(&currVirtDev->listVirtualDevs, &PPP_headVirtualDevs);

        newVirtDev = True;
    }

    currPathOnVirtdev = PPP_GetVirtPathConnector(currVirtDev, currPath);
    if (currPathOnVirtdev == NULL)
    {
        /* Create connector */

        DLOG("Connector does not exist, creating...");

        currPathOnVirtdev = kmalloc(sizeof(*currPathOnVirtdev), GFP_KERNEL);
        if (currPathOnVirtdev == NULL)
        {
            /* If created the path here, remove it */
            if (newPath)
            {
                /* Could not create complete path, cleanup */
                PPP_DelPath(rxFrom, txTo, viaVirtualDev);
            }
            if (newVirtDev)
            {
                list_del(&currVirtDev->listVirtualDevs);
                kfree(currVirtDev);
            }
            PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);
            /* Cannot alloc mem, reject */
            WLOG("Cannot kmalloc(%d, GPF_KERNEL) for virtual device : rxFrom %s, txTo %s, virtDev %s",
                 sizeof(*currPathOnVirtdev), rxFrom, txTo, viaVirtualDev);
            return ENOMEM;
        }
        /* Connect connector to path */
        list_add(&currPathOnVirtdev->listPathPathsOnVirtDev, &currPath->headPathPathsOnVirtDev);
        /* Connect connector to virtual device */
        list_add(&currPathOnVirtdev->listVirtDevPathsOnVirtDev, &currVirtDev->headVirtDevPathsOnVirtDev);
        /* Point from connector to path */
        currPathOnVirtdev->path = currPath;
    }

    PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);

    /* Add path to virtual device */
    DLOG("Added path %s --> %s to list of paths on virtDev %s",
         currPath->rxFromDev, currPath->txToDev, viaVirtualDev);

    return 0;
}

/**************************************************************************/
/*! \fn int PPP_DelPath(char *rxFrom, char *txTo, char *viaVirtualDev)
 **************************************************************************
 *  \brief Delete a path
 *  \param[in] rxFrom - device used as IN
 *  \param[in] txTo - device used as OUT
 *  \param[in] viaVirtualDev - virtual device through which the traffic flows
 *  \return 0 (OK) / 1 (NOK)
 **************************************************************************/
int PPP_DelPath(char *rxFrom, char *txTo, char *viaVirtualDev)
{
    Uint32 lockKey;
    PPP_path_t *currPath;
    PPP_session_t *currSession;
    PPP_session_t *safeNextSession;
    Uint32 currSessionHandle;
    PPP_virtualDev_t *currVirtDev;
    PPP_pathOnVirtualDev_t *currPathOnVirtdev;

    if ((rxFrom == NULL) || (txTo == NULL) || (viaVirtualDev == NULL))
    {
        ELOG("Illegal param: rxFrom %p, txTo %p, viaVirtualDev %p",
             rxFrom, txTo, viaVirtualDev);
        return -1;
    }

    DLOG("Del path %s -> (%s) --> %s", rxFrom, viaVirtualDev, txTo);

    PAL_osProtectEntry(PAL_OSPROTECT_INTERRUPT, &lockKey);

    /* Does the path exist? */
    currPath = PPP_GetPath(rxFrom, txTo);
    if (currPath != NULL)
    {
        /* Get virtual dev */
        currVirtDev = PPP_GetVirtDev(viaVirtualDev);
        if (currVirtDev != NULL)
        {
            /* Find connector */
            currPathOnVirtdev = PPP_GetVirtPathConnector(currVirtDev, currPath);
            if (currPathOnVirtdev != NULL)
            {
                /* Found */
                DLOG("Disconnecting path from virtDev");
                /* Remove from path */
                list_del(&currPathOnVirtdev->listPathPathsOnVirtDev);
                /* Remove from virtual dev */
                list_del(&currPathOnVirtdev->listVirtDevPathsOnVirtDev);
                /* Release memory */
                kfree(currPathOnVirtdev);
            }
            else
            {
                ELOG("%s(%s, %s, %s) --> No connector found: Virtual device %s, path %s - %s",
                     __func__, rxFrom, txTo, viaVirtualDev, viaVirtualDev, rxFrom, txTo);
                /* Continue anyway, since the rest is only cleanup */
            }
        }
        else
        {
            ELOG("%s(%s, %s, %s) --> Virtual device %s not connected to path %s - %s",
                 __func__, rxFrom, txTo, viaVirtualDev, viaVirtualDev, rxFrom, txTo);
            /* Continue anyway, since the rest is only cleanup */
        }

        /* If there are no more virtDevs that pass through this path, remove the      */
        /* path and all sessions from the path.                                       */
        /* Note: If sessions knew the virtDev they go through, we would remove these  */
        /* here.                                                                      */
        if (list_empty(&currPath->headPathPathsOnVirtDev))
        {
            /* No other virtDev going through currPath, remove all sessions */
            DLOG("No other virtDevs on this path, removing all sessions from the path");
            list_for_each_entry_safe(currSession, safeNextSession, &currPath->headSessionsOnPath, listSessionsOnPath)
            {
                /* Del session */
                currSessionHandle = PPP_SESSION_HANDLE_FROM_PTR(currSession);
                DLOG("Del session %d on path", currSessionHandle);
                PPP_DelSessionEntry(currSessionHandle);
            }

            /* Remove path */
            list_del(&currPath->listPaths);
            /* Free mem */
            kfree(currPath);
        }

        /* If no more paths on dev, remove dev */
        if ((currVirtDev != NULL) && list_empty(&currVirtDev->headVirtDevPathsOnVirtDev))
        {
            DLOG("No more paths on virtDev, remove virtDev %s", viaVirtualDev);
            /* Remove from list of devs */
            list_del(&currVirtDev->listVirtualDevs);
            /* Free mem */
            kfree(currVirtDev);
        }

    }
    else
    {
        DLOG("Path does not exist: %s --> %s", rxFrom, txTo);
    }

    PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);

    return 0;
}

/**************************************************************************/
/*! \fn int PPP_ReadCounters(char *viaVirtualDev, PPP_counters_t *pppCtrs)
 **************************************************************************
 *  \brief Read a path's counters
 *  \param[in] viaVirtualDev - virtual device through which the traffic flows
 *  \param[out] pppCtrs - Counters of PP traffic going through the virtual device
 *              in/out are provided according to the paths requested when
 *              adding the paths.
 *  \return 0 (OK) / 1 (NOK)
 **************************************************************************/
int PPP_ReadCounters(char *viaVirtualDev, PPP_counters_t *pppCtrs)
{
    Uint32 lockKey;
    Bool savEnableDebug;

    if ((viaVirtualDev == NULL) || (pppCtrs == NULL))
    {
        ELOG("Illegal param: viaVirtualDev %p, pppCtrs %p", viaVirtualDev, pppCtrs);
        return -1;
    }

    /* Temporarily stop standard logs */
    savEnableDebug = PPP_enableDebug;
    PPP_enableDebug = False;

    PAL_osProtectEntry(PAL_OSPROTECT_INTERRUPT, &lockKey);

    PPP_TraverseVirtDev(viaVirtualDev, pppCtrs, 0, NULL);

    PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);

    /* Restore logs */
    PPP_enableDebug = savEnableDebug;

    return 0;
}

/**************************************************************************/
/*      LOCAL FUNCTIONS:                                                  */
/**************************************************************************/

/**************************************************************************/
/*! \fn static Bool PPP_IsExistPath(char *rxFrom, char *txTo)
 **************************************************************************
 *  \brief Get the requested path
 *  \param[in] rxFrom - input device
 *  \param[in] txTo - output device
 *  \return PPP_path_t*, NULL when not exist
 **************************************************************************/
static PPP_path_t * PPP_GetPath(char *rxFrom, char *txTo)
{
    PPP_path_t *candPath; /* Candidate */

    if ((rxFrom == NULL) || (txTo == NULL))
    {
        ELOG("Illegal param: rxFrom %p, txTo %p", rxFrom, txTo);
        return NULL;
    }

    DLOG("Get path %s --> %s", rxFrom, txTo);

    /* Does the path exist? */
    list_for_each_entry(candPath, &PPP_headPaths, listPaths)
    {
        if ((candPath->rxFromDev[0] == '\0') || (candPath->txToDev[0] == '\0'))
        {
            /* NULL device, error */
            ELOG("NULL device name on path %p : rxFrom \"%s\", txTo \"%s\"",
                 candPath, candPath->rxFromDev, candPath->txToDev);
            /* Next */
            continue;
        }

        DLOG("Trying %s --> %s", candPath->rxFromDev, candPath->txToDev);

        /* Names seem OK */
        if ((strncmp(rxFrom, candPath->rxFromDev, sizeof(candPath->rxFromDev)) == 0) &&
            (strncmp(txTo, candPath->txToDev, sizeof(candPath->txToDev)) == 0))
        {
            /* Found existing path */
            DLOG("Found path %s --> %s : %p", candPath->rxFromDev, candPath->txToDev, candPath);
            return candPath;
        }
    }

    /* Path does not exist */
    DLOG("No such path %s --> %s", rxFrom, txTo);
    return NULL;
}

/**************************************************************************/
/*! \fn static PPP_virtualDev_t * PPP_GetVirtDev(char *viaVirtualDev)
 **************************************************************************
 *  \brief Get the requested virtual dev
 *  \param[in] viaVirtualDev - virtual device name
 *  \return PPP_virtualDev_t*, NULL when not exist
 **************************************************************************/
static PPP_virtualDev_t * PPP_GetVirtDev(char *viaVirtualDev)
{
    PPP_virtualDev_t *candVirtDev; /* Candidate */

    if (viaVirtualDev == NULL)
    {
        ELOG("Illegal param: viaVirtualDev %p", viaVirtualDev);
        return NULL;
    }

    DLOG("Get virtDev %s", viaVirtualDev);

    list_for_each_entry(candVirtDev, &PPP_headVirtualDevs, listVirtualDevs)
    {
        if (candVirtDev->virtualDev[0] == '\0')
        {
            /* NULL device, error */
            ELOG("NULL device name on virtual device list element %p", candVirtDev);

            /* Next */
            continue;
        }

        DLOG("Trying %s", candVirtDev->virtualDev);

        /* Names seem OK */
        if (strncmp(viaVirtualDev, candVirtDev->virtualDev, sizeof(candVirtDev->virtualDev)) == 0)
        {
            /* Found existing path */
            DLOG("Found virtDev %s : %p", viaVirtualDev, candVirtDev);
            return candVirtDev;
        }
    }

    /* Device does not exist */
    DLOG("No such virtDev %s", viaVirtualDev);
    return NULL;
}

/**************************************************************************/
/*! \fn static PPP_pathOnVirtualDev_t *PPP_GetVirtPathConnector(PPP_path_t *path, PPP_virtualDev_t *virtDev)
 **************************************************************************
 *  \brief Get connector object of Virtual device to Path
 *  \param[in] virtDev - Virtual device
 *  \param[in] path - path
 *  \return PPP_pathOnVirtualDev_t*, NULL when not exist
 **************************************************************************/
static PPP_pathOnVirtualDev_t *PPP_GetVirtPathConnector(PPP_virtualDev_t *virtDev, PPP_path_t *path)
{
    PPP_pathOnVirtualDev_t *currPathOnVirtdev;

    if ((virtDev == NULL) || (path == NULL))
    {
        ELOG("Illegal param: virtDev %p, path %p", virtDev, path);
        return NULL;
    }

    /* Find connector */
    DLOG("Searching for connector: Virtual device %s to Path %s->%s", virtDev->virtualDev, path->rxFromDev, path->txToDev);

    /* list_for_each_entry_safe is not necessary here, since once we find a           */
    /* match, we return.                                                              */
    list_for_each_entry(currPathOnVirtdev, &virtDev->headVirtDevPathsOnVirtDev, listVirtDevPathsOnVirtDev)
    {
        if (path == currPathOnVirtdev->path)
        {
            /* Found */
            DLOG("Found connector (%p) Virtual device %s to Path %s->%s",
                 currPathOnVirtdev, virtDev->virtualDev, path->rxFromDev, path->txToDev);
            return currPathOnVirtdev;
        }
    }

    DLOG("Connector not found: Virtual device %s to Path %s->%s", virtDev->virtualDev, path->rxFromDev, path->txToDev);

    return NULL;
}

/**************************************************************************/
/*! \fn static int PPP_DelSessionEntry(Uint32 sessionHandle)
 **************************************************************************
 *  \brief Delete session entry
 *  \param[in] sessionHandle - Handle to the created session
 *  \return 0 - OK, -1, NOK
 **************************************************************************/
static int PPP_DelSessionEntry(Uint32 sessionHandle)
{
    if (!PPP_IS_LEGAL_SESSION_HANDLE(sessionHandle))
    {
        ELOG("Illegal param: sessionHandle %d", sessionHandle);
        return -1;
    }

    DLOG("Deleting session %d", sessionHandle);

    /* Remove session from  list of sessions on path */
    list_del(&PPP_sessions[sessionHandle].listSessionsOnPath);

    /* Mark session as unused */
    PPP_sessions[sessionHandle].path = NULL;

    return 0;
}

/**************************************************************************/
/*! \fn static int PPP_AddSession(sessionHandle, skb)
 **************************************************************************
 *  \brief Add a session
 *  \param[in] sessionHandle - Handle to the created session
 *  \param[in] skb - skb that started the session
 *  \return 0 (OK) / 1 (NOK)
 **************************************************************************/
static int PPP_AddSession(sessionHandle_t sessionHandle, struct sk_buff *skb)
{
    PPP_path_t *currPath = NULL;
    struct net_device*    input_dev = NULL;

    if (!PPP_IS_LEGAL_SESSION_HANDLE(sessionHandle) || (skb == NULL))
    {
        ELOG("Illegal param: sessionHandler %d, skb %p", sessionHandle, skb);
        return -1;
    }

    if (!skb->skb_iif)
    {
        input_dev = dev_get_by_index(&init_net,skb->skb_iif);
    }
    /* Is there a path that matches this session? */
    if (input_dev)
    {
        currPath = PPP_GetPath(input_dev->name, skb->dev->name);
    }

    if (currPath != NULL)
    {
        DLOG("Add session %d on path %s --> %s", sessionHandle, input_dev->name, skb->dev->name);
        /* Path exists */
        /* Point the session to the path */
        PPP_sessions[sessionHandle].path = currPath;
        /* Add the session to the list of sessions on the path */
        list_add(&PPP_sessions[sessionHandle].listSessionsOnPath, &currPath->headSessionsOnPath);
    }
    else
    {
        DLOG("Session %d not added, no such path", sessionHandle);
    }

    if (input_dev)
    {
        dev_put(input_dev);
    }
    return 0;
}

/**************************************************************************/
/*! \fn static int PPP_DelSession(sessionHandle, sessionCounters)
 **************************************************************************
 *  \brief Add a session
 *  \param[in] sessionHandle - Handle to the created session
 *  \param[in] sessionCounters - counetrs of deleted session
 *  \return 0 (OK) / 1 (NOK)
 **************************************************************************/
static int PPP_DelSession(sessionHandle_t sessionHandle, TI_PP_SESSION_STATS *sessionCounters)
{
    PPP_path_t *currPath;
    uint64_t uint64var;

    if (!PPP_IS_LEGAL_SESSION_HANDLE(sessionHandle) || (sessionCounters == NULL))
    {
        ELOG("Illegal param: sessionHandler %d, sessionCounters %p", sessionHandle, sessionCounters);
        return -1;
    }

    DLOG("Del session %d", sessionHandle);

    /* Does session exist? */
    currPath = PPP_sessions[sessionHandle].path;
    if (currPath != NULL)
    {
        /* Session exists */

        /* Update counters */
        uint64var = sessionCounters->bytes_forwarded_hi;
        uint64var <<= 32;
        uint64var += sessionCounters->bytes_forwarded_lo;
        currPath->deadSessionCtr.octets += uint64var;
        currPath->deadSessionCtr.packets += sessionCounters->packets_forwarded;

        PPP_DelSessionEntry(sessionHandle);
    }
    else
    {
        DLOG("Session %d not deleted, no path on session", sessionHandle);
    }

    return 0;
}

/**************************************************************************/
/*! \fn static void PPP_event_handler(unsigned int event_id, unsigned int param1, unsigned int param2)
 **************************************************************************
 *  \brief Handle PP events
 *  \param[in] event id
 *  \param[in] param1, param2 - change according to the event type.
 *  \return 0 (OK) / 1 (NOK)
 **************************************************************************/
static void PPP_event_handler(unsigned int event_id, unsigned int param1, unsigned int param2)
{
    unsigned int sessionHandle;
    struct sk_buff *skb;
    TI_PP_SESSION_STATS *ppStats;
    Uint32 lockKey;

    PAL_osProtectEntry(PAL_OSPROTECT_INTERRUPT, &lockKey);

    switch (event_id)
    {
        case TI_PPM_SESSION_CREATED:
        {
            /* param1 - sessionHandle, param2 - skb */
            sessionHandle = param1;
            skb = (struct sk_buff*)param2;
            PPP_AddSession(sessionHandle, skb);
            break;
        }
        case TI_PPM_SESSION_DELETED:
        {
            /* param1 - sessionHandle, param2 - ppStats */
            sessionHandle = param1;
            ppStats = (TI_PP_SESSION_STATS*)param2;
            PPP_DelSession(sessionHandle, ppStats);
            break;
        }
        default:
        {
            /* Silently ignore everything else */
            break;
        }
    }

    PAL_osProtectExit(PAL_OSPROTECT_INTERRUPT, lockKey);

    /* Our work is done. */
    return;
}

/**************************************************************************/
/*! \fn static int PPP_TraverseVirtDev(char *viaVirtualDev, PPP_counters_t *outPppCtrs, int limitLen, char *outBuf)
 **************************************************************************
 *  \brief Accumulate counters for a virtual dev
 *          Output can be as counters and/or printout
 *  \param[in] viaVirtualDev - virtual device through which the traffic flows
 *  \param[out] outPppCtrs - Counters of PP traffic going through the virtual device
 *              in/out are provided according to the paths requested when
 *              adding the paths.
 *  \param[in] limitLen - max len allowed in outBuf
 *  \param[out] outBuf - output buffer to write output
 *  \return 0 (OK) / <1 NOK / >1 len (when using outBuf)
 **************************************************************************/
static int PPP_TraverseVirtDev(char *viaVirtualDev, PPP_counters_t *outPppCtrs, int limitLen, char *outBuf)
{
    PPP_virtualDev_t *currVirtDev;
    PPP_uniCounetrs_t inCtrs = {0};
    PPP_uniCounetrs_t outCtrs = {0};
    PPP_uniCounetrs_t *pCtrs;
    PPP_counters_t localPppCtrs = {{0}};
    PPP_counters_t *pppCtrs;
    PPP_path_t *currPath;
    PPP_session_t *currSession;
    Uint32 currSessionHandle;
    TI_PP_SESSION_STATS sessionStats;
    int len;
    char wrBuf[200];
    char *buf;
    char *partialBuf;
    uint64_t var64;
    PPP_pathOnVirtualDev_t *currPathOnVirtDev;

    if ((viaVirtualDev == NULL) ||
        ((outPppCtrs == NULL) && ((outBuf == NULL) || (limitLen == 0))) )
    {
        ELOG("Illegal param: viaVirtualDev %p, pppCtrs %p, buf %p, limit len %d", viaVirtualDev, outPppCtrs, outBuf, limitLen);
        return -1;
    }

    DLOG("Read counters for virtDev %s", viaVirtualDev);

    if (outPppCtrs == NULL)
    {
        pppCtrs = &localPppCtrs;
    }
    else
    {
        pppCtrs = outPppCtrs;
        /* Init val to 0 */
        *pppCtrs = localPppCtrs;
    }

    len = 0;

    /* Get the device */
    currVirtDev = PPP_GetVirtDev(viaVirtualDev);
    if (currVirtDev != NULL)
    {
        /* Dev exists */
        DLOG("Found virtDev %s : %p", viaVirtualDev, currVirtDev);

        if (outBuf == NULL)
        {
            buf = wrBuf;
            limitLen = sizeof(wrBuf) - 1;
        }
        else
        {
            buf = outBuf;
        }

        if (outBuf != NULL)
        {
            len += snprintf(buf+len, limitLen - len, "virtDev : %s\n", currVirtDev->virtualDev);
        }

        /* Sum all paths on this device */
        list_for_each_entry(currPathOnVirtDev, &currVirtDev->headVirtDevPathsOnVirtDev, listVirtDevPathsOnVirtDev)
        {
            currPath = currPathOnVirtDev->path;
            /* Determine ctr direction */
            if (currPath->pathDir == PPP_PATHDIR_IN)
            {
                pCtrs = &inCtrs;
            }
            else if (currPath->pathDir == PPP_PATHDIR_OUT)
            {
                pCtrs = &outCtrs;
            }
            else
            {
                /* Corrupted DB */
                ELOG("Corrupted DB - pathDir = %d : path - rxFrom %s, txTo %s",
                     currPath->pathDir, currPath->rxFromDev, currPath->txToDev);
                /* Try next */
                continue;
            }

            partialBuf = buf+len;
            len += snprintf(buf+len, limitLen-len, "\tPath %s --> %s --> %s (%s)\n",
                            currPath->rxFromDev, viaVirtualDev, currPath->txToDev,
                            currPath->pathDir == PPP_PATHDIR_OUT ? "OUT" : "IN");

            DLOG("Read path %s", partialBuf);
            if (outBuf == NULL)
            {
                len = 0;
            }

            /* Dead sessions */
            pCtrs->octets += currPath->deadSessionCtr.octets;
            pCtrs->packets += currPath->deadSessionCtr.packets;

            if (outBuf != NULL)
            {
                len += snprintf(buf+len, limitLen-len, "\t\tDead sessions: Bytes %llu, Packets %llu\n",
                                currPath->deadSessionCtr.octets, currPath->deadSessionCtr.packets);
            }

            /* Live sessions */
            list_for_each_entry(currSession, &currPath->headSessionsOnPath, listSessionsOnPath)
            {
                /* Proper pointer arithmetic should give us the correct index */
                currSessionHandle = PPP_SESSION_HANDLE_FROM_PTR(currSession);
                if (PPP_IS_LEGAL_SESSION_HANDLE(currSessionHandle))
                {
                    /* Get running session counters */
                    if (ti_ppm_get_session_stats(currSessionHandle, &sessionStats) < 0)
                    {
                        /* Something is wrong */
                        ELOG("Could not get session stats - %d : path - rxFrom %s, txTo %s",
                             currSessionHandle, currPath->rxFromDev, currPath->txToDev);
                        /* Try next */
                        continue;
                    }

                    DLOG("Add live session %d", currSessionHandle);

                    var64 = sessionStats.bytes_forwarded_hi;
                    var64 <<= 32;
                    var64 += sessionStats.bytes_forwarded_lo;
                    pCtrs->octets += var64;
                    pCtrs->packets += sessionStats.packets_forwarded;

                    if (outBuf != NULL)
                    {
                        len += snprintf(buf+len, limitLen-len, "\t\tSession %d: Bytes %llu, Packets %u\n",
                                        currSessionHandle, var64, sessionStats.packets_forwarded);
                    }
                }
                else
                {
                    /* Something is wrong */
                    ELOG("Got illegal session handle 0x%08X : path - rxFrom %s, txTo %s",
                         currSessionHandle, currPath->rxFromDev, currPath->txToDev);
                    /* Try next */
                    continue;
                }
            }
        }
        /* Update counters */
        pppCtrs->inCtrs = inCtrs;
        pppCtrs->outCtrs = outCtrs;
        if (outBuf != NULL)
        {
            len += snprintf(buf+len, limitLen-len, "\t\tTotal: InBytes  %llu, InPackets %llu\n"
                                                   "\t\t       OutBytes %llu, outPackets %llu\n",
                            pppCtrs->inCtrs.octets, pppCtrs->inCtrs.packets,
                            pppCtrs->outCtrs.octets, pppCtrs->outCtrs.packets);
        }
    }

    DLOG("Read virDev %s - done", viaVirtualDev);

    return len;
}

/**************************************************************************/
/*! \fn static int PPP_Show(char* buf, char **start, off_t offset, int count, int *eof, void *data)
 **************************************************************************
 *  \brief Show PPP DB
 *  \param[in] read proc params
 *  \return len of data / -1 (NOK)
 **************************************************************************/
static int PPP_Show(char* page, char **start, off_t offset, int count, int *eof, void *data)
{
    int len=0;
    PPP_path_t *currPath;
    PPP_virtualDev_t *currVirtDev;
    int limit = count;
    int i;
    char* buf = page + offset;

    /* Print all paths */
    len += snprintf(buf+len, limit-len, "%-15s %s\n", "rxFrom", "txTo");
    len += snprintf(buf+len, limit-len, "%-15s %s\n", "---------------", "---------------");
    list_for_each_entry(currPath, &PPP_headPaths, listPaths)
    {
        len += snprintf(buf+len, limit-len, "%-15s %s\n", currPath->rxFromDev, currPath->txToDev);
    }
    len += snprintf(buf+len, limit-len, "\n");

    /* Print all virt devs */
    list_for_each_entry(currVirtDev, &PPP_headVirtualDevs, listVirtualDevs)
    {
        len += PPP_TraverseVirtDev(currVirtDev->virtualDev, NULL, limit-len, buf+len);
    }

    /* Print all sessions */
    len += snprintf(buf+len, limit-len, "Sessions:\n");
    len += snprintf(buf+len, limit-len, "---------\n");
    for(i = 0; i < TI_PP_MAX_ACCLERABLE_SESSIONS; i++)
    {
        if (PPP_sessions[i].path != NULL)
        {
            len += snprintf(buf+len, limit-len, "Session %d on path %s --> %s\n",
                            i, PPP_sessions[i].path->rxFromDev, PPP_sessions[i].path->txToDev);
        }
    }

    *eof = 1;
    return len;
}

/**************************************************************************/
/*! \fn static int PPP_WrProc (struct file *file, const char *buffer, unsigned long count, void *data)
 **************************************************************************
 *  \brief Handle WR proc
 *  \param[in] write proc parsm
 *              buffer: add/del rxFromDev txToDev viaVirtualDev
 *  \return count / -(1) (NOK)
 **************************************************************************/
static int PPP_WrProc (struct file *file, const char *buffer, unsigned long count, void *data)
{
    char pppCmd[100]; /* add/del==3 + \b + 3 * (IFNAMSIZ + \b) + in/out==3 ==> 55 */
    static const char *sep = " \t,\n";
    char *cmd;
    char *rxFrom;
    char *txTo;
    char *viaVirtualDev;
    char *dirStr;
    int  ret;
    char *endtok;
    PPP_pathDir_e pathDir;

    ret = count;

    /* Validate the length of data passed. */
    if (count > (sizeof(pppCmd) - 1))
        count = sizeof(pppCmd) - 1;

    /* Initialize the buffer before using it. */
    memset(pppCmd, 0, sizeof(pppCmd));

    /* Copy from user space. */
    if (copy_from_user(pppCmd, buffer, count))
        return -EFAULT;
    pppCmd[count] = '\0';

    /* Parse line */
    /* cmd */
    TOKENIZE(cmd, sep, pppCmd, endtok, "No command [add/del]");
    /* rxFrom */
    TOKENIZE(rxFrom, sep, endtok + 1, endtok, "No rxFrom device");
    /* txTo */
    TOKENIZE(txTo, sep, endtok + 1, endtok, "No txTo device");
    /* virtDev */
    TOKENIZE(viaVirtualDev, sep, endtok + 1, endtok, "No virtual device");

    /* Command Handlers */
    if (strcmp(cmd, "add") == 0)
    {
        /* direction */
        TOKENIZE(dirStr, sep, endtok + 1, endtok, "No direction");
        if (strcmp(dirStr, "in") == 0)
        {
            pathDir = PPP_PATHDIR_IN;
        }
        else if (strcmp(dirStr, "out") == 0)
        {
            pathDir = PPP_PATHDIR_OUT;
        }
        else
        {
            ELOG("Illegal pathDir \"%s\" [in/out]", dirStr);
            return -EFAULT;
        }

        PPP_AddPath(rxFrom, txTo, viaVirtualDev, pathDir);
        return ret;
    }
    else if (strcmp(cmd, "del") == 0)
    {
        PPP_DelPath(rxFrom, txTo, viaVirtualDev);
        return ret;
    }
    else
    {
        ELOG("Illegal command %s [add/del]", cmd);
        return -EFAULT;
    }
}

/**************************************************************************/
/*! \fn static int PPP_WrProcDbg (struct file *file, const char *buffer, unsigned long count, void *data)
 **************************************************************************
 *  \brief Enable/Disable debug
 *  \param[in] write proc parsm
 *  \return count / -(1) (NOK)
 **************************************************************************/
static int PPP_WrProcDbg (struct file *file, const char *buffer, unsigned long count, void *data)
{
    char pppDbg[2]; /* 1/0 */
    int  ret;

    ret = count;

    /* Validate the length of data passed. */
    if (count > (sizeof(pppDbg) - 1))
        count = sizeof(pppDbg) - 1;

    /* Initialize the buffer before using it. */
    memset(pppDbg, 0, sizeof(pppDbg));

    /* Copy from user space. */
    if (copy_from_user(pppDbg, buffer, count))
        return -EFAULT;
    pppDbg[count] = '\0';

    if (pppDbg[0] == '1')
    {
        /* Enable debug */
        PPP_enableDebug = True;
    }
    else
    {
        /* Enable debug */
        PPP_enableDebug = False;
    }

    return ret;
}

/**************************************************************************/
/*! \fn static int PPP_RdProcDbg(char* buf, char **start, off_t offset, int count, int *eof, void *data)
 **************************************************************************
 *  \brief Show PPP Debug
 *  \param[in] read proc params
 *  \return len of data / -1 (NOK)
 **************************************************************************/
static int PPP_RdProcDbg(char* page, char **start, off_t offset, int count, int *eof, void *data)
{
    int len=0;
    int limit = count;
    char* buf = page + offset;

    len += snprintf(buf+len, limit-len, "PPP Debug : %s (%d)\n", PPP_enableDebug ? "Enabled" : "Disabled", PPP_enableDebug);

    *eof = 1;
    return len;
}

EXPORT_SYMBOL(PPP_AddPath);
EXPORT_SYMBOL(PPP_DelPath);
EXPORT_SYMBOL(PPP_ReadCounters);

