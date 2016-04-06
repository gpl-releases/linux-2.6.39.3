/*
 * ti_ppm.c - Packet Processor Manager
 *
 * Description:
 *  The file contains the implementation of the Packet Processor Manager.
 *  The Packet Processor Manager interfaces the Packet Processor Driver
 *  with the Host Operating system. The code is OS agnostic.
 *
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#include <linux/ctype.h>
/*#include <linux/printk.h>*/
extern int printk(const char *fmt, ...);
#include "linux/ti_ppm.h"
#include "listlib.h"


#include <asm-arm/arch-avalanche/generic/ti_ppd.h>

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/

/* Definition for NULL */
#define NULL (void *)0

/* Size of the LUT Entry. */
#define LUT_SIZE        sizeof(TI_PP_PACKET_DESC)*2

/**************************************************************************
 *************************** Local Structures *****************************
 **************************************************************************/

/**************************************************************************
 * STRUCTURE NAME : PPM_HANDLE
 **************************************************************************
 * DESCRIPTION   :
 *  Generic structure which keeps track of the handles.
 **************************************************************************/
typedef struct PPM_HANDLE
{
    LIST_NODE       links;      /* List of Handles */
    unsigned char   handle;     /* Handle */
}PPM_HANDLE;

/**************************************************************************
 * STRUCTURE NAME : PPM_EVENT_HANDLER
 **************************************************************************
 * DESCRIPTION   :
 *  The structure defines the event handler structure used internally by
 *  the PPM.
 **************************************************************************/
typedef struct PPM_EVENT_HANDLER
{
    /* List of Event Handlers. */
    LIST_NODE   links;
    void        (*event_handler)(unsigned int event_id, unsigned int param1, unsigned int param2);
}PPM_EVENT_HANDLER;

/**************************************************************************
 * STRUCTURE NAME : PPM_SESSION_VPID_INFO
 **************************************************************************
 * DESCRIPTION   :
 *  Internal structure which stores the association between VPID and Session
 **************************************************************************/
typedef struct PPM_SESSION_VPID_INFO
{
    /* List of sessions */
    LIST_NODE           links;

    /* VPID information. */
    struct PPM_VPID*    vpid;

    /* Session Information */
    struct PPM_SESSION* session;
}PPM_SESSION_VPID_INFO;

/**************************************************************************
 * STRUCTURE NAME : PPM_SESSION
 **************************************************************************
 * DESCRIPTION   :
 *  Internal structure that for storing the session.
 **************************************************************************/
typedef struct PPM_SESSION
{
    /* Session Information as passed by the callee */
    TI_PP_SESSION           session;

    /* Session Hash Code used to identify in which bucket the session resides. */
    unsigned int            hash_code;

    /* The Session Handle associated with the session. */
    PPM_HANDLE*             session_handle;

    /* Session --> VPID Mapping */
    PPM_SESSION_VPID_INFO*  session_vpid_mapping[TI_PP_MAX_VPID];
}PPM_SESSION;

/**************************************************************************
 * STRUCTURE NAME : PPM_VPID
 **************************************************************************
 * DESCRIPTION   :
 *  Internal structure that is used for keeping track of the VPID.
 **************************************************************************/
typedef struct PPM_VPID
{
    /* List of VPID attached to a PID */
    LIST_NODE               links;

    /* VPID Information as passed by the callee. */
    TI_PP_VPID              vpid;

    /* Status of the VPID i,e. ACTIVE or INACTIVE. */
    PPM_STATUS              vpid_status;

    /* The handle of the VPID  */
    PPM_HANDLE*             vpid_handle;

    /* Pointer to the parent PID structure. */
    struct PPM_PID*         ptr_pid;

    /* VPID --> Session Mapping.  */
    PPM_SESSION_VPID_INFO*  vpid_session_mapping;
}PPM_VPID;

/**************************************************************************
 * STRUCTURE NAME : PPM_PID
 **************************************************************************
 * DESCRIPTION   :
 *  Internal structure that is used for keeping track of the PID.
 **************************************************************************/
typedef struct PPM_PID
{
    /* PID Information as passed by the callee. */
    TI_PP_PID             pid;

    /* Status of the PID i,e. ACTIVE or INACTIVE. */
    PPM_STATUS            pid_status;

    /* List of all VPID handles that are created on the PID. */
    PPM_VPID*             vpid_list;
}PPM_PID;

/**************************************************************************
 * STRUCTURE NAME : PPM_SESSION_HASH
 **************************************************************************
 * DESCRIPTION   :
 *  Internal structure that is used for keeping track of all sessions that
 *  exist in the PP Subsystem. The Hash bucket is required so that sessions
 *  can easily be identified by their "ingress" properties and are useful
 *  for checking for duplicate sessions.
 **************************************************************************/
typedef struct PPM_SESSION_HASH
{
    /* To keep track of collisions. */
    LIST_NODE               links;

    /* Store the session handle. Session Properites can be accessed from
     * the global session database using this handle. */
    int                     session_handle;
}PPM_SESSION_HASH;

/**************************************************************************
 * STRUCTURE NAME : TI_PPM_MCB
 **************************************************************************
 * DESCRIPTION   :
 *  The structure is the MCB block that contains all the information
 *  regarding the PPM entities.
 **************************************************************************/
typedef struct TI_PPM_MCB
{
    /* PPM Configuration information */
    TI_PPM_OS_FUNC_TABLE    os_table;

    /* Status of the PDSP; */
    PPM_STATUS              pdsp_status;

    /* Status of the Session Databse; */
    PPM_STATUS              sessiondb_status;

    /* PID, VPID and Session Database. These contain a list of all PID, VPID
     * and Sessions that are active in the PDSP.*/
    PPM_PID*                pid_database[TI_PP_MAX_PID];
    PPM_VPID*               vpid_database[TI_PP_MAX_VPID];
    PPM_SESSION*            session_database[TI_PP_MAX_ACCLERABLE_SESSIONS];

    /* Hash bucket which keeps track of all sessions. */
    PPM_SESSION_HASH*       session_hash[TI_PP_MAX_ACCLERABLE_SESSIONS];

    /* Lists which keep track of free and available handles.
     * These lists have been created so that the time taken
     * to get a free handle while creating a PID, VPID or
     * Session is an O(1) operation instead of an O(n) because
     * the same could be acheived by searching the above
     * sequential arrays. */

    /* Session Handle Maintainance */
    PPM_HANDLE*             session_free_list;
    PPM_HANDLE*             session_occupied_list;

    /* PID Handle Maintainance */
    PPM_HANDLE*             pid_free_list;
    PPM_HANDLE*             pid_occupied_list;

    /* VPID Handle Maintainance */
    PPM_HANDLE*             vpid_free_list;
    PPM_HANDLE*             vpid_occupied_list;

    /* Number of sessions that are active in the PDSP as per the PPM. */
    int                     ppm_num_session;

    /* This keeps track of all the event handlers that are registered with the
     * Packet Processor Manager. */
    PPM_EVENT_HANDLER       event_info;
}TI_PPM_MCB;

/**************************************************************************
 ************************************ Globals *****************************
 **************************************************************************/

/* Global Packet Processor Manager master control block. */
TI_PPM_MCB   ppm_mcb;

/**************************************************************************
 ************************************ Externs *****************************
 **************************************************************************/

/* HASH Functions. */
extern unsigned int hash(unsigned char* k, unsigned int len, unsigned int init_val);

/* PSM Functions. */
extern int ti_pp_enable_psm(void);
extern int ti_pp_disable_psm(void);
extern PPM_STATUS ti_pp_get_status(void);

/* Internal Functions */
static void ti_ppm_event_handler (unsigned int event_id, unsigned int param1, unsigned int param2);
static int ti_ppm_lookup_hash_table (TI_PP_SESSION* ptr_session, unsigned int* hash_code);
static int ti_ppm_add_hash_table    (int session_handle,         unsigned int  hash_code);
static int ti_ppm_delete_hash_table (int session_handle,         unsigned int  hash_code);
static int ti_ppm_session_vpid_match (PPM_SESSION* ptr_session, int vpid_handle);

/**************************************************************************
 *************************** Functions ************************************
 **************************************************************************/

/**************************************************************************
 * FUNCTION NAME : ti_ppm_get_handle
 **************************************************************************
 * DESCRIPTION   :
 *  The function is called to generate PID, VPID and Session handles which
 *  are unique in the system. The
 *
 * RETURNS       :
 *  Pointer to a handle - Success
 *  NULL                - No space available.
 **************************************************************************/
static PPM_HANDLE* ti_ppm_get_handle (PPM_HANDLE** free_list, PPM_HANDLE** occupied_list)
{
    PPM_HANDLE*         ptr_handle;
    PPM_OS_CONTEXT      os_context;

    /* Critical Section Start. */
    ppm_mcb.os_table.critical_section_start(&os_context);

    /* Get a handle from the free list and add it to the occupied list. */
    ptr_handle = (PPM_HANDLE *)list_remove((LIST_NODE**)free_list);
    if (ptr_handle != NULL)
        list_add ((LIST_NODE**)occupied_list, (LIST_NODE*)ptr_handle);

    /* Critical Section End. */
    ppm_mcb.os_table.critical_section_end(&os_context);

    /* Return the handle retreived, */
    return ptr_handle;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_release_handle
 **************************************************************************
 * DESCRIPTION   :
 *  The function is called to release a "previously" generated PID, VPID
 *  and Session handles.
 *
 * RETURNS       :
 *  >=0     - Success
 *  <0      - Error
 **************************************************************************/
static void ti_ppm_release_handle
(
    PPM_HANDLE*  ptr_handle,
    PPM_HANDLE** free_list,
    PPM_HANDLE** occupied_list
)
{
    PPM_OS_CONTEXT   os_context;

    /* Critical Section Start. */
    ppm_mcb.os_table.critical_section_start(&os_context);

    /* Delete the handle from the occupied list */
    list_remove_node ((LIST_NODE**)occupied_list, (LIST_NODE*)ptr_handle);

    /* Add the element to the free list */
    list_add ((LIST_NODE**)free_list, (LIST_NODE*)ptr_handle);

    /* Critical Section End. */
    ppm_mcb.os_table.critical_section_end(&os_context);

    /* Work is done. */
    return;
}

void ti_ppm_dump_hash( void )
{
    int i;

    printk(" =========== HASH TABLE =========\n");
    for (i=0; i<256; i++)
    {
        if (ppm_mcb.session_hash[i])
        {
            PPM_SESSION_HASH*       tmp = ppm_mcb.session_hash[i];
            while(tmp)
            {
                printk(" [%3d]:%p: session [%10d], next:%p prev:%p \n",
                        i,
                        tmp,
                        tmp->session_handle,
                        tmp->links.p_next,
                        tmp->links.p_prev );

                tmp = tmp->links.p_next;
            }
        }
        else
        {
            printk(" [%3d]: NULL \n", i );
        }
    }
    printk(" ================================\n");
}

void ti_ppm_dump_sessions( void )
{
    int i;
    PPM_HANDLE*  tmp;
    printk(" =========== SESSIONS FREE LIST =============\n");
    tmp = ppm_mcb.session_free_list;

    while(tmp)
    {
        printk(" %p: session [%10d], next:%p prev:%p \n",
                tmp,
                tmp->handle,
                tmp->links.p_next,
                tmp->links.p_prev );

        tmp = tmp->links.p_next;
    }

    printk(" =========== SESSIONS OCCUPIED LIST =========\n");
    tmp = ppm_mcb.session_occupied_list;

    while(tmp)
    {
        printk(" %p: session [%10d], next:%p prev:%p \n",
                tmp,
                tmp->handle,
                tmp->links.p_next,
                tmp->links.p_prev );

        tmp = tmp->links.p_next;
    }

    printk(" =========== SESSIONS DATABASE ==============\n");

    for (i=0; i<256; i++)
    {
        if (ppm_mcb.session_database[i])
        {
            printk(" [%3d]:%p: session [%10d], session_handle_p:%p \n",
                    i,
                    ppm_mcb.session_database[i],
                    ppm_mcb.session_database[i]->session.session_handle,
                    ppm_mcb.session_database[i]->session_handle );
        }
        else
        {
            printk(" [%3d]: NULL \n", i );
        }
    }

    printk(" ============================================\n");
}
/**************************************************************************
 * FUNCTION NAME : ti_ppm_lookup_hash_table
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to lookup the session in the session hash table.
 *
 * RETURNS       :
 *      Session Handle  -   Success
 *      <0              -   No Matching session exists.
 **************************************************************************/
static int ti_ppm_lookup_hash_table (TI_PP_SESSION* ptr_session, unsigned int* hash_code)
{
    PPM_SESSION_HASH*   ptr_session_hash;
    PPM_OS_CONTEXT      os_context;
    PPM_SESSION*        ptr_internal_session;
    int                 handle_found = -1;

    /* Get the hash code for the session information. */
    *hash_code = hash ((unsigned char *)&ptr_session->ingress.l2_packet, LUT_SIZE, 0);

    /* CRITICAL SECTION STARTS:-
     *  This falls into the critical section because the add/delete calls to the Session HASH bucket can be
     *  executed from different contexts which can cause the hash bucket iteration to get corrupted.
     *  Calculation of the HASH Function is done outside the critical section. */
    ppm_mcb.os_table.critical_section_start(&os_context);

    /* Once the hash code is known lookup the corresponding entry in the HASH Table. */
    ptr_session_hash = ppm_mcb.session_hash[*hash_code];

    /* Hash bucket was not empty. In this case iterate through all the entries in the hash bucket
     * for a perfect match. */
    while (ptr_session_hash != NULL)
    {
        /* Get the pointer to the internal session. */
        ptr_internal_session = ppm_mcb.session_database[ptr_session_hash->session_handle];

        if (ptr_internal_session == NULL)
            break;

        /* Compare the contents */
        if (ppm_mcb.os_table.memcmp ((void *)&ptr_internal_session->session.ingress.l2_packet,
                                     (void *)&ptr_session->ingress.l2_packet, LUT_SIZE) == 0)
        {
            handle_found = ptr_session_hash->session_handle;
			
            /* Critical Section Stop */
            ppm_mcb.os_table.critical_section_end(&os_context);

            /* Perfect Match! Return the session handle. */
            return handle_found;
        }

        /* Move to the next element. */
        ptr_session_hash = (PPM_SESSION_HASH*)list_get_next ((LIST_NODE*)ptr_session_hash);
    }

    /* Critical Section Stop */
    ppm_mcb.os_table.critical_section_end(&os_context);

    /* Control comes here implies that no match has been found! */
    return -1;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_add_hash_table
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to add a session to the Session HASH Table at the
 *  location specified by "hash_code"
 *
 * RETURNS       :
 *      0  -   Success
 *      <0 -   Error
 *
 *  NOTES:
 *      The function should always be protected by a critical section.
 **************************************************************************/
static int ti_ppm_add_hash_table (int session_handle, unsigned int hash_code)
{
    PPM_SESSION_HASH*   ptr_session_hash;
    PPM_OS_CONTEXT      os_context;

    /* Allocate memory for the Session HASH */
    ptr_session_hash = ppm_mcb.os_table.malloc (sizeof(PPM_SESSION_HASH));
    if (ptr_session_hash == NULL)
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to lack of memory\n", __FUNCTION__);
        return -1;
    }

    /* CRITICAL SECTION STARTS:-
     *  This falls into the critical section because the add/delete calls to the Session HASH bucket can be
     *  executed from different contexts which can cause the hash bucket iteration to get corrupted. */
    ppm_mcb.os_table.critical_section_start(&os_context);

    /* Initialize the structure. */
    ptr_session_hash->session_handle = session_handle;

    /* Add to the HASH Bucket */
    list_add((LIST_NODE**)&ppm_mcb.session_hash[hash_code], (LIST_NODE*)ptr_session_hash);

    /* Critical Section Stop */
    ppm_mcb.os_table.critical_section_end(&os_context);

    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_delete_hash_table
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to delete a session from the Session HASH Table at
 *  the location specified by "hash_code"
 *
 * RETURNS       :
 *      0  -   Success
 *      <0 -   Error
 *
 *  NOTES:
 *      The function should always be protected by a critical section.
 **************************************************************************/
static int ti_ppm_delete_hash_table (int session_handle, unsigned int hash_code)
{
    PPM_SESSION_HASH*   ptr_session_hash;
    PPM_OS_CONTEXT      os_context;


    /* CRITICAL SECTION STARTS:-
     *  This falls into the critical section because the add/delete calls to the Session HASH bucket can be
     *  executed from different contexts which can cause the hash bucket iteration to get corrupted. */
    ppm_mcb.os_table.critical_section_start(&os_context);

    /* Get the pointer to the HASH bucket. */
    ptr_session_hash = ppm_mcb.session_hash[hash_code];
    while (ptr_session_hash != NULL)
    {
        /* Delete the session only if there is a match? */
        if (ptr_session_hash->session_handle == session_handle)
        {
            /* Remove the element from the HASH Bucket. */
            list_remove_node((LIST_NODE**)&ppm_mcb.session_hash[hash_code], (LIST_NODE*)ptr_session_hash);

            /* Cleanup the allocated block of memory. */
            ppm_mcb.os_table.free (ptr_session_hash);

            /* Critical Section Stop */
            ppm_mcb.os_table.critical_section_end(&os_context);
            return 0;
        }

        /* Get the next element. */
        ptr_session_hash = (PPM_SESSION_HASH *)list_get_next((LIST_NODE*)ptr_session_hash);
    }

    /* Critical Section Stop */
    ppm_mcb.os_table.critical_section_end(&os_context);

    /* Control comes here; implies that there was no session handle in the HASH bucket. */
    return -1;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_session_vpid_association
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to create the association between the VPID and
 *  Session.
 *
 * RETURNS       :
 *      0   -   Success
 *      0x1 -   Out of memory error;
 *      0x2 -   Invalid VPID or VPID is going DOWN
 *
 * NOTES         :
 *  This function is called from within the Critical Section.
 **************************************************************************/
static int ti_ppm_session_vpid_association
(
    int          vpid_index,
    PPM_SESSION* ptr_session,
    int          session_vpid_mapping_index
)
{
    PPM_SESSION_VPID_INFO*  ptr_session_vpid;
    PPM_VPID*               ptr_vpid;

    /* Validate the VPID range before proceeding. */
    if ((vpid_index < 0) || (vpid_index >= TI_PP_MAX_VPID))
        return 0x2;

    /* Get the pointer to the VPID. */
    ptr_vpid = ppm_mcb.vpid_database[vpid_index];
    if (ptr_vpid != NULL)
    {
        /* Create the session only if the VPID is up and running? */
        if (ptr_vpid->vpid_status == ACTIVE)
        {
            /* Allocate memory for the session VPID association. */
            ptr_session_vpid = (PPM_SESSION_VPID_INFO*)ppm_mcb.os_table.malloc (sizeof(PPM_SESSION_VPID_INFO));
            if (ptr_session_vpid == NULL)
                return 0x1;

            /* Initialize the association. */
            ptr_session_vpid->vpid    = ptr_vpid;
            ptr_session_vpid->session = ptr_session;

            /* Add the association to the VPID */
            list_add ((LIST_NODE**)&ptr_vpid->vpid_session_mapping, (LIST_NODE*)ptr_session_vpid);

            /* Add the association to the Session. */
            ptr_session->session_vpid_mapping[session_vpid_mapping_index] = ptr_session_vpid;
            return 0;
        }
        else
        {
            /* VPID is going DOWN. There is no point in creating the session. */
            return 0x2;
        }
    }

    /* VPID does not exist in the System. */
    return 0x2;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_create_pid
 **************************************************************************
 * DESCRIPTION   :
 *  The function uses the information passed to create a PID in the PDSP.
 *
 * RETURNS       :
 *  Handle to the created VPID - Success
 *  <0                         - Error
 **************************************************************************/
int ti_ppm_create_pid (TI_PP_PID* ptr_pid)
{
    PPM_PID*        ptr_internal_pid;

    /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    /* Validate the arguments. */
    if (ptr_pid == NULL)
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to lack of input parameter\n", __FUNCTION__);
        return -1;
    }

    if (ptr_pid->pid_handle >= TI_PP_MAX_PID)
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to bad input parameter (pid_handle=%d, range:%d-%d)\n", __FUNCTION__, ptr_pid->pid_handle, 0, TI_PP_MAX_PID-1);
        return -1;
    }

    /*********************** End of Validation ***********************/
    if(!ppm_mcb.pid_database[ptr_pid->pid_handle])
    {
        /* Create the PID in the PDSP through the PPD */
        if (ti_ppd_create_pid(ptr_pid) == 0)
        {
            /* PID Created successfully: Allocate memory for the new PID */
            ptr_internal_pid = (PPM_PID *)ppm_mcb.os_table.malloc (sizeof(PPM_PID));
            if (ptr_internal_pid != NULL)
            {
                /* Copy the PID Information. */
                ppm_mcb.os_table.memcpy ((void *)&ptr_internal_pid->pid, (void *)ptr_pid, sizeof(TI_PP_PID));

                /* Initialize the VPID List. There are no active VPID at this time. */
                ptr_internal_pid->vpid_list = NULL;

                /* The PID is up and running. */
                ptr_internal_pid->pid_status  = ACTIVE;

                /* Register the PID in the global database. */
                ppm_mcb.pid_database[ptr_pid->pid_handle] = ptr_internal_pid;

                /* Generate an Event indicating that the PID has been created. */
                ti_ppm_event_handler (TI_PPM_PID_CREATED, (unsigned int)ptr_pid->pid_handle, 0);

                /* Return the handle of the PID */
                return ptr_pid->pid_handle;
            }
            else
            {
                /* No memory available: PID is in PDSP but we cannot keep track of it in the Host... */
                printk("ERROR: PP Operation %s cannot be accomplished due to lack of memory\n", __FUNCTION__);
                ti_ppm_event_handler (TI_PPM_OUT_OF_MEMORY, 0, 0);
            }
        }
        else
        {
            /* PDSP was unable to create a PID; generate the event and also release the handle. */
            printk("ERROR: PP Operation %s cannot be accomplished since ti_ppd_create_pid operation has failed\n", __FUNCTION__);
            ti_ppm_event_handler (TI_PPM_CREATE_PID_FAILED, (unsigned int)ptr_pid->pid_handle, 0);
        }
    }
    
    /* PID was not created. */
    printk("ERROR: PP Operation %s cannot be accomplished since this pid_handle=%d was already created\n", __FUNCTION__, ptr_pid->pid_handle);
    return -1;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_config_pid_range
 **************************************************************************
 * DESCRIPTION   :
 *  The function uses the information passed to config PID range in the PDSP.
 *
 * RETURNS       :
 *  0   - Success
 *  <0  - Error
 **************************************************************************/
int ti_ppm_config_pid_range(TI_PP_PID_RANGE *pid_range)
{
    /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    /* Validate the arguments. */
    if (pid_range == NULL)
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to lack of input parameter\n", __FUNCTION__);
        return -1;
    }

    /*********************** End of Validation ***********************/

    if(ti_ppd_config_pid_range(pid_range) != 0)
    {
        /* Error: PDSP was unable to configure the PID range. */
        printk("ERROR: PP Operation %s failed\n", __FUNCTION__);
        return -1;
    }

    /* PID range successfully configured in the system */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_remove_pid_range
 **************************************************************************
 * DESCRIPTION   :
 *  The function uses the information passed to remove PID range in the PDSP.
 *
 * RETURNS       :
 *  0   - Success
 *  <0  - Error
 **************************************************************************/
int ti_ppm_remove_pid_range(unsigned int port_num)
{
    /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    if(ti_ppd_remove_pid_range(port_num) != 0)
    {
        /* Error: PDSP was unable to delete the PID range. */
        printk("ERROR: PP Operation %s failed\n", __FUNCTION__);
        return -1;
    }

    /* PID range successfully removed from the system */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_delete_pid
 **************************************************************************
 * DESCRIPTION   :
 *  The function deletes the PID in the PDSP.
 *
 * RETURNS       :
 *  0   - Success
 *  <0  - Error
 **************************************************************************/
int ti_ppm_delete_pid (unsigned char pid_handle)
{
    PPM_PID*    ptr_internal_pid;
    PPM_VPID*   ptr_vpid;

    /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    /* Validate the arguments. */
    if (pid_handle >= TI_PP_MAX_PID)
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to bad input parameter (pid_handle=%d, range:%d-%d)\n", __FUNCTION__, pid_handle, 0, TI_PP_MAX_PID-1);
        return -1;
    }

    /*********************** End of Validation ***********************/

    /* Get the pointer to the internal PID Database entry. */
    ptr_internal_pid = ppm_mcb.pid_database[pid_handle];
    if (ptr_internal_pid != NULL)
    {
        /* The PID is going DOWN. */
        ptr_internal_pid->pid_status = INACTIVE;

        /* Get the VPID */
        ptr_vpid = (PPM_VPID*)list_get_head ((LIST_NODE**)&ptr_internal_pid->vpid_list);

        /* Cycle through all the VPID attached to the PID and delete. */
        while (ptr_vpid != NULL)
        {
            /* Delete the VPID */
            ti_ppm_delete_vpid (ptr_vpid->vpid_handle->handle);

            /* Get the next VPID. */
            ptr_vpid = (PPM_VPID*)list_get_head ((LIST_NODE**)&ptr_internal_pid->vpid_list);
        }

        /* Delete the PID from the PDSP also. */
        if (ti_ppd_delete_pid(pid_handle) < 0)
        {
            /* Error: PDSP was unable to delete the PID. */
            printk("ERROR: PP Operation %s cannot be accomplished since ti_ppd_delete_pid operation has failed\n", __FUNCTION__);
            ti_ppm_event_handler (TI_PPM_DELETE_PID_FAILED, (unsigned int)pid_handle, 0);
            return -1;
        }

        /* Remove the entry from the PID Global Database */
        ppm_mcb.pid_database[pid_handle] = NULL;

        /* Cleanup the allocated block of memory */
        ppm_mcb.os_table.free (ptr_internal_pid);

        /* Generate an event indicating that the PID has been deleted. */
        ti_ppm_event_handler (TI_PPM_PID_DELETED, (unsigned int)pid_handle, 0);

        /* PID has been deleted successfully. */
        return 0;
    }

    /* PID was not registered in the System */
    printk("ERROR: PP Operation %s cannot be accomplished since this pid_handle=%d is already deleted\n", __FUNCTION__, pid_handle);
    return -1;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_set_pid_flags
 **************************************************************************
 * DESCRIPTION   :
 *  The function uses the information passed to modify the PID flags in the
 *  PDSP.
 *
 * RETURNS       :
 *  0   - Success
 *  <0  - Error
 **************************************************************************/
int ti_ppm_set_pid_flags (unsigned char pid_handle, unsigned int new_flags)
{
    PPM_PID*        ptr_internal_pid = NULL;

    /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    /* Validate the arguments. */
    if (pid_handle >= TI_PP_MAX_PID)
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to bad input parameter (pid_handle=%d, range:%d-%d)\n", __FUNCTION__, pid_handle, 0, TI_PP_MAX_PID-1);
        return -1;
    }

    /* Make sure that the PID has been created and exists in the System. */
    ptr_internal_pid = ppm_mcb.pid_database[pid_handle];
    if(ptr_internal_pid == NULL)
    {
        printk("ERROR: PP Operation %s cannot be accomplished since pid_handle=%d does not exist\n", __FUNCTION__, pid_handle);
        return -1;
    }

    /*********************** End of Validation ***********************/

    /* Modify the PID flags through PPD */
    if (ti_ppd_set_pid_flags(&ptr_internal_pid->pid, new_flags) < 0)
    {
        /* PPD set PID flags failed */
        printk("ERROR: PP Operation %s for pid_handle=%d failed\n", __FUNCTION__, pid_handle);
        return -1;
    }

    /* PID flags has been changed succesfully. */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_create_vpid
 **************************************************************************
 * DESCRIPTION   :
 *  The function uses the information passed to create a VPID in the PDSP.
 *
 * RETURNS       :
 *  Handle to the created VPID - Success
 *  <0                         - Error
 **************************************************************************/
int ti_ppm_create_vpid (TI_PP_VPID* ptr_vpid)
{
    PPM_PID*            ptr_internal_pid;
    PPM_VPID*           ptr_internal_vpid;
    PPM_OS_CONTEXT      os_context;
    PPM_HANDLE*         ptr_handle = NULL;
    int                 error = 0;
    unsigned char       vpid_handle;

    /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status == INACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is INACTIVE\n", __FUNCTION__);
        return -1;
    }

    if (ppm_mcb.pdsp_status == PPM_PSM)
    {
        return -1;
    }

    /* Validate the parameters that have been passed. */
    if (ptr_vpid == NULL)
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to lack of input parameter\n", __FUNCTION__);
        return -1;
    }

    if (ptr_vpid->parent_pid_handle >= TI_PP_MAX_PID)
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to bad input parameter (parent_pid_handle=%d, range:%d-%d)\n", __FUNCTION__, ptr_vpid->parent_pid_handle, 0, TI_PP_MAX_PID-1);
        return -1;
    }

    /*********************** End of Validation ***********************/

    /* Critical Section Start: This falls into the critical section since we are accessing
     * the "vpid_list" in the PID structure. This list is also manipulated through the PPM
     * call delete_vpid. An example of a race condition would be a "create_vpid" on PID X
     * being done while a "delete_vpid" on PID X is being done at the same time. */
    ppm_mcb.os_table.critical_section_start(&os_context);

    /* Generate a handle for the VPID. */
    ptr_handle = ti_ppm_get_handle (&ppm_mcb.vpid_free_list, &ppm_mcb.vpid_occupied_list);
    if (ptr_handle == NULL)
    {
        /* Critical Section Stop. */
        ppm_mcb.os_table.critical_section_end (&os_context);
        printk("ERROR: PP Operation %s cannot be accomplished since ti_ppm_get_handle operation has failed\n", __FUNCTION__);
        return -1;
    }

    vpid_handle = ptr_handle->handle;

    /* Allocate memory for the new VPID */
    ptr_internal_vpid = (PPM_VPID *)ppm_mcb.os_table.malloc (sizeof(PPM_VPID));
    if (ptr_internal_vpid == NULL)
    {
        /* Error: No memory available; generate the OOM event. */
        ti_ppm_event_handler (TI_PPM_OUT_OF_MEMORY, 0, 0);
        ti_ppm_release_handle (ptr_handle, &ppm_mcb.vpid_free_list, &ppm_mcb.vpid_occupied_list);

        /* Critical Section Stop. */
        ppm_mcb.os_table.critical_section_end (&os_context);
        printk("ERROR: PP Operation %s cannot be accomplished due to lack of memory\n", __FUNCTION__);
        return -1;
    }

    /* Initialize the VPID structure. */
    ptr_internal_vpid->vpid_status          = ACTIVE;
    ptr_internal_vpid->vpid_handle          = ptr_handle;
    ptr_internal_vpid->vpid_session_mapping = NULL;
    ppm_mcb.os_table.memcpy ((void *)&ptr_internal_vpid->vpid, (void *)ptr_vpid, sizeof(TI_PP_VPID));

    /* This falls within the critical section because it is possible that the PID gets deleted before
     * the linkage between the PID and VPID gets created. */
    ptr_internal_pid = ppm_mcb.pid_database[ptr_vpid->parent_pid_handle];
    if (ptr_internal_pid != NULL)
    {
        /* There was a matching PID existing in the PPM. Is this PID active or is it going down? */
        if (ptr_internal_pid->pid_status == ACTIVE)
        {
            /* PID is active; so we can go ahead and create the VPID. */
            ptr_internal_vpid->ptr_pid = ptr_internal_pid;

            /* Initialize the allocated VPID Handle. */
            ptr_vpid->vpid_handle = vpid_handle;

            /* Create the VPID in the PDSP through the PPD */
            if (ti_ppd_create_vpid(ptr_vpid) < 0)
            {
                /* PDSP was unable to create a VPID; Error was detected */
                printk("ERROR: PP Operation %s cannot be accomplished since ti_ppd_create_vpid operation has failed\n", __FUNCTION__);
                error = 0x1;
            }
            else
            {
                /* The VPID belongs to the PID; so create the association. */
                list_add ((LIST_NODE**)&ptr_internal_pid->vpid_list, (LIST_NODE*)ptr_internal_vpid);

                /* Store the pointer into the global VPID database. */
                ppm_mcb.vpid_database[vpid_handle] = ptr_internal_vpid;

                /* Add the allocated VPID Handle to the structure also. */
                ptr_internal_vpid->vpid.vpid_handle = vpid_handle;
            }
        }
        else
        {
            /* Error: PID was going DOWN. This is an error condition but not enough to warrant an event being generated */
            error = 0x2;
            printk("WARNING: PP Operation ti_ppm_create_vpid cannot be accomplished since parent parent_pid_handle=%d status is not ACTIVE\n", ptr_vpid->parent_pid_handle);
        }
    }
    else
    {
        /* Error: Creating a VPID on a non-existent PID. This is an error condition but not enough to warrant an event being generated. */
        error = 0x2;
        printk("WARNING: PP Operation ti_ppm_create_vpid cannot be accomplished since parent parent_pid_handle=%d status is not set\n", ptr_vpid->parent_pid_handle);
    }

    if (error)
    {
        /* Memory cleanup */
        ti_ppm_release_handle (ptr_handle, &ppm_mcb.vpid_free_list, &ppm_mcb.vpid_occupied_list);
        ppm_mcb.os_table.free (ptr_internal_vpid);
    }

    /* Critical Section Stop. */
    ppm_mcb.os_table.critical_section_end (&os_context);



    if (error == 0)
    {
        /* Generate an event indicating that a VPID has been created. */
        ti_ppm_event_handler (TI_PPM_VPID_CREATED, (unsigned int)vpid_handle, 0);

        /* Return the handle to the created VPID. */
        return vpid_handle;
    }

    /* Error was detected! Send an event only if the PDSP returned an error. */
    if (error & 1)
    {
        ti_ppm_event_handler (TI_PPM_CREATE_VPID_FAILED, (unsigned int)vpid_handle, 0);
    }

    /* Return an error; VPID creation failed */
    return -1;

}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_delete_vpid
 **************************************************************************
 * DESCRIPTION   :
 *  The function deletes the VPID in the PDSP.
 *
 * RETURNS       :
 *  0   - Success
 *  <0  - Error
 **************************************************************************/
int ti_ppm_delete_vpid (unsigned char vpid_handle)
{
    PPM_VPID*               ptr_vpid;
    PPM_SESSION_VPID_INFO*  ptr_session_vpid_info;
    PPM_OS_CONTEXT          os_context;

    /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    /* Validate the parameters that have been passed. */
    if (vpid_handle >= TI_PP_MAX_VPID)
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to bad input parameter (vpid_handle=%d, range:%d-%d)\n", __FUNCTION__, vpid_handle, 0, TI_PP_MAX_VPID-1);
        return -1;
    }

    /*********************** End of Validation ***********************/

    /* Critical Section Start: This falls into the critical section since we are accessing
     * the "vpid_list" in the PID structure. This list is also manipulated through the PPM
     * call create_vpid. An example of a race condition would be a "create_vpid" on PID X being
     * done while a "delete_vpid" on PID X is being done at the same time. */
    ppm_mcb.os_table.critical_section_start(&os_context);

    /* Get the pointer to the internal VPID Database entry. */
    ptr_vpid = ppm_mcb.vpid_database[vpid_handle];
    if (ptr_vpid != NULL)
    {
        /* The VPID is going DOWN. */
        ptr_vpid->vpid_status = INACTIVE;

        /* Get a pointer to the list of all sessions that exist on the VPID. Cycle through and remove them! */
        ptr_session_vpid_info = (PPM_SESSION_VPID_INFO *)list_get_head((LIST_NODE**)&ptr_vpid->vpid_session_mapping);
        while (ptr_session_vpid_info != NULL)
        {
            /* Delete the session. */
            ti_ppm_delete_session (ptr_session_vpid_info->session->session_handle->handle, NULL);

            /* Get the next session */
            ptr_session_vpid_info = (PPM_SESSION_VPID_INFO *)list_get_head((LIST_NODE**)&ptr_vpid->vpid_session_mapping);
        }

        /* Delete the VPID from the PDSP. */
        if (ti_ppd_delete_vpid (vpid_handle) < 0)
        {
            /* Critical Section Stop */
            ppm_mcb.os_table.critical_section_end (&os_context);

            /* Error: PDSP was unable to delete the VPID. */
            printk("ERROR: PP Operation %s cannot be accomplished since ti_ppd_delete_vpid operation failed for vpid_handle=%d\n", __FUNCTION__, vpid_handle);
            ti_ppm_event_handler (TI_PPM_DELETE_VPID_FAILED, (unsigned int)vpid_handle, 0);
            return -1;
        }

        /* The VPID is no longer bound to the PID. */
        list_remove_node ((LIST_NODE **)&ptr_vpid->ptr_pid->vpid_list, (LIST_NODE*)ptr_vpid);

        /* Remove the entry from the global VPID database. */
        ppm_mcb.vpid_database[vpid_handle] = 0;

        /* Release the handle held by the VPID */
        ti_ppm_release_handle (ptr_vpid->vpid_handle, &ppm_mcb.vpid_free_list, &ppm_mcb.vpid_occupied_list);

        /* Cleanup the memory */
        ppm_mcb.os_table.free(ptr_vpid);

        /* Critical Section Stop */
        ppm_mcb.os_table.critical_section_end (&os_context);

        /* Generate an event indicating that the VPID has been deleted. */
        ti_ppm_event_handler (TI_PPM_VPID_DELETED, (unsigned int)vpid_handle, 0);

        /* Succesfully deleted the VPID. */
        return 0;
    }

    /* Error: Deleting a non-existent VPID.*/
    printk("ERROR: PP Operation %s cannot be accomplished since vpid_handle=%d does not exist\n", __FUNCTION__, vpid_handle);
    return -1;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_set_vpid_flags
 **************************************************************************
 * DESCRIPTION   :
 *  The function is called to modify the VPID flags. The VPID should have been
 *  created before this call is made.
 *
 * RETURNS       :
 *      0   -   Success
 *      <0  -   Error
 **************************************************************************/
int ti_ppm_set_vpid_flags (unsigned char vpid_handle, unsigned int new_flags)
{
    PPM_VPID* ptr_internal_vpid = NULL;

    /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    /* Validate the arguments. */
    if (vpid_handle >= TI_PP_MAX_VPID)
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to bad input parameter (vpid_handle=%d, range:%d-%d)\n", __FUNCTION__, vpid_handle, 0, TI_PP_MAX_VPID-1);
        return -1;
    }

    /* Make sure that the VPID has been created and exists in the System. */
    ptr_internal_vpid = ppm_mcb.vpid_database[vpid_handle];
    if (ptr_internal_vpid == NULL)
    {
        printk("ERROR: PP Operation %s cannot be accomplished since vpid_handle=%d does not exist\n", __FUNCTION__, vpid_handle);
        return -1;
    }

    /*********************** End of Validation ***********************/

    /* Enable the VPID. */
    if (ti_ppd_set_vpid_flags (&ptr_internal_vpid->vpid, new_flags) < 0)
    {
        printk("ERROR: PP Operation %s cannot be accomplished since ti_ppd_set_vpid_flags operation has failed for vpid_handle=%d\n", __FUNCTION__, vpid_handle);
        return -1;
    }

    /* VPID has been enabled succesfully. */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_check_session
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to check if the session matches an existing session
 *
 * RETURNS       :
 *      Session Handle   -   Success
 *      <0               -   No matching session exists
 **************************************************************************/
int ti_ppm_check_session(TI_PP_SESSION* ptr_session)
{
    unsigned int hash_code;

    /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        return -1;
    }

    /* Validate the arguments. */
    if (ptr_session == NULL)
    {
        return -1;
    }

    /*********************** End of Validation ***********************/

    /* Return the result of the session lookup. */
    return ti_ppm_lookup_hash_table (ptr_session, &hash_code);
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_create_session
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to create a session.
 *
 * RETURNS       :
 *      Session Handle  -   Success
 *      <0              -   Error
 **************************************************************************/
int ti_ppm_create_session (TI_PP_SESSION* ptr_session, void* ptr, int isTunnel)
{
    PPM_OS_CONTEXT      os_context;
    PPM_HANDLE*         ptr_handle;
    PPM_SESSION*        ptr_internal_session;
    int                 index;
    int                 sess_handle = -1;
    int                 error = 0;
    unsigned int        hash_code;
    TI_PPD_IF           ingress;
    TI_PPD_IF           egress[TI_PP_MAX_EGRESS_PROPERTY];

    /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    /* Ensure that the Session database is open to access. */
    if (ppm_mcb.sessiondb_status == INACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished since session database cannot be accessed\n", __FUNCTION__);
        return -1;
    }

    /* Make sure a valid session was passed. */
    if (ptr_session == NULL)
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to lacking input parameter\n", __FUNCTION__);
        return -1;
    }

    /* The number of egress records should be within the correct range. */
    if ((ptr_session->num_egress == 0) || (ptr_session->num_egress > TI_PP_MAX_EGRESS_PROPERTY))
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to bad input parameter (num_egress=%d, range:%d-%d)\n", __FUNCTION__, ptr_session->num_egress, 1, TI_PP_MAX_EGRESS_PROPERTY);
        return -1;
    }

    /*********************** End of Validation ***********************/

    /* Check for duplicate sessions; If this is a duplicate session then return the existing
     * session handle immediately. */
    sess_handle = ti_ppm_lookup_hash_table (ptr_session, &hash_code);

    if (sess_handle >= 0)
    {
        return sess_handle;
    }

    /* CRITICAL SECTION STARTS:-
     * This code falls into the critical section because we need to associate the VPID and Session together
     * We cannot afford to have the VPID be deleted while this association is being created. */
    ppm_mcb.os_table.critical_section_start(&os_context);

    /* Generate a handle for the Session. */
    ptr_handle = ti_ppm_get_handle (&ppm_mcb.session_free_list, &ppm_mcb.session_occupied_list);
    if (ptr_handle == NULL)
    {
        /* Critical Section Stop */
        ppm_mcb.os_table.critical_section_end (&os_context);
        return -1;
    }

    sess_handle = ptr_handle->handle;

    /* Allocate memory for the new session */
    ptr_internal_session = (PPM_SESSION *)ppm_mcb.os_table.malloc (sizeof(PPM_SESSION));
    if (ptr_internal_session == NULL)
    {
        /* Error: No memory available; generate the OOM event. */
        ti_ppm_event_handler (TI_PPM_OUT_OF_MEMORY, 0, 0);
        ti_ppm_release_handle (ptr_handle, &ppm_mcb.session_free_list, &ppm_mcb.session_occupied_list);
        /* Critical Section Stop */
        ppm_mcb.os_table.critical_section_end (&os_context);
        printk("ERROR: PP Operation %s cannot be accomplished due to lack of memory\n", __FUNCTION__);
        return -1;
    }

    /* Initialize the Session structure. */
    ptr_internal_session->session_handle = ptr_handle;
    ptr_internal_session->hash_code      = hash_code;
    ppm_mcb.os_table.memset ((void *)&ptr_internal_session->session_vpid_mapping, 0,
                             sizeof(ptr_internal_session->session_vpid_mapping));
    ppm_mcb.os_table.memcpy ((void *)&ptr_internal_session->session, (void *)ptr_session, sizeof(TI_PP_SESSION));


    /* Link the sessions with all the Egress VPID. Cycle through all the Egress Framing Records! */
    for (index = 0; index < ptr_session->num_egress; index++)
    {
        PPM_VPID*   ptr_vpid;
        PPM_PID*    ptr_pid;

        /* Create the Session-Egress VPID association */
        error = ti_ppm_session_vpid_association (ptr_session->egress[index].vpid_handle, ptr_internal_session, index);
        if (error != 0)
            break;

        /* Remember the Egress PID and VPID Information. This information needs to be passed down to the PPD.
         * We know for sure the VPID is correct because the association function has not returned an error. */
        ptr_vpid = ppm_mcb.vpid_database[ptr_session->egress[index].vpid_handle];
        ptr_pid  = ppm_mcb.pid_database[ptr_vpid->vpid.parent_pid_handle];
        egress[index].vpid = &ptr_vpid->vpid;
        egress[index].pid  = &ptr_pid->pid;
    }

    /* Check if we aborted because of an error or not? */
    if (error == 0)
    {
        /* All the Egress VPID associations have been done. Was an Ingress VPID Specified? */
        if (ptr_session->ingress.vpid_handle != TI_PP_IGNORE_INGRESS_VPID)
        {
            PPM_VPID*   ptr_vpid;
            PPM_PID*    ptr_pid;

            /* YES. Create the Session-Ingress VPID association. */
            error = ti_ppm_session_vpid_association (ptr_session->ingress.vpid_handle, ptr_internal_session, index);
            if (error == 0)
            {
                /* Remember the Ingress PID and VPID Information. */
                ptr_vpid = ppm_mcb.vpid_database[ptr_session->ingress.vpid_handle];
                ptr_pid  = ppm_mcb.pid_database[ptr_vpid->vpid.parent_pid_handle];
                ingress.vpid = &ptr_vpid->vpid;
                ingress.pid  = &ptr_pid->pid;
            }
        }
        else
        {
            /* The Ingress Interface was unspecified. Initialize the information correctly so that the PPD knows about it. */
            ingress.pid  = NULL;
            ingress.vpid = NULL;
        }
    }
    
    /* Check if all the Session VPID associations have been made. */
    if (error == 0)
    {
        /* Hook the session into the Session HASH Bucket. */
        if (ti_ppm_add_hash_table (sess_handle, hash_code) < 0)
        {
            printk("ERROR: PP Operation %s cannot be accomplished since ti_ppm_add_hash_table operation failed\n", __FUNCTION__);
            error = 0x1;
        }
    }

    /* If no error has been detected; create the session in the PDSP. */
    if (error == 0)
    {
        /* Initialize the session with the allocated handle */
        ptr_session->session_handle = sess_handle;

        /* Create the session in the PDSP. */
         if (ti_ppd_create_session (ptr_session, &ingress, &egress[0], isTunnel) == 0)
        {
            /* Hook the session into the global database. */
            ppm_mcb.session_database[sess_handle] = ptr_internal_session;

            /* Increment the number of sessions that have been created. */
            ppm_mcb.ppm_num_session = ppm_mcb.ppm_num_session + 1;

            /* Add the allocated Session Handle to the structure also. */
            ptr_internal_session->session.session_handle = sess_handle;
        }
        else
        {
            /* PDSP Create Session Failed. */
            printk("ERROR: PP Operation %s cannot be accomplished since ti_ppd_create_session operation failed\n", __FUNCTION__);
            error = 0x3;
        }
    }

    /* Was there an Error detected at any stage? Error handling should be done outside the scope
     * of the critical sections. No point in locking the system while we try and recover from the
     * error. */
    if (error != 0x0)
    {
        /* Session was not created. Release and cleanup! */
        for (index = 0; index < TI_PP_MAX_VPID; index++)
        {
            /* Get the session-vpid association. */
            PPM_SESSION_VPID_INFO* ptr_session_vpid = ptr_internal_session->session_vpid_mapping[index];
            if (ptr_session_vpid == NULL)
                break;

            /* Remove the session vpid association from the VPID */
            list_remove_node ((LIST_NODE**)&ptr_session_vpid->vpid->vpid_session_mapping, (LIST_NODE*)ptr_session_vpid);

            /* Clean up the memory. */
            ppm_mcb.os_table.free(ptr_session_vpid);
        }

        /* Clean up the memory allocated for the session.  */
        ppm_mcb.os_table.free(ptr_internal_session);

        /* Remove the entry from the Session HASH Bucket; if it exists. */
        ti_ppm_delete_hash_table (sess_handle, hash_code);

        /* Release the session handle. */
        ti_ppm_release_handle (ptr_handle, &ppm_mcb.session_free_list, &ppm_mcb.session_occupied_list);
    }

    /* Critical Section Stop */
    ppm_mcb.os_table.critical_section_end (&os_context);

    /* Was there an Error detected at any stage? Error handling should be done outside the scope
     * of the critical sections. No point in locking the system while we try and recover from the
     * error. */
    if (error != 0x0)
    {
        /* Do we need to generate an error event? */
        if (error == 0x1)
            ti_ppm_event_handler (TI_PPM_OUT_OF_MEMORY, 0, 0);
        else if (error == 0x3)
            ti_ppm_event_handler (TI_PPM_CREATE_SESSION_FAILED, sess_handle, 0);

        /* Return an error! */
        return -1;
    }

    /* Session was succesfully created; announce to the world! */
    ti_ppm_event_handler (TI_PPM_SESSION_CREATED, (unsigned int)sess_handle, (unsigned int)ptr);

    /* Return the session handle. */
    return sess_handle;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_modify_session
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to modify the session properties of an existing
 *  session
 *
 * RETURNS       :
 *      0   -   Success
 *      <0  -   Error
 **************************************************************************/
int ti_ppm_modify_session (TI_PP_SESSION* ptr_session, unsigned char session_handle)
{
    PPM_SESSION*        ptr_internal_session;
    PPM_SESSION*        ptr_original_session;
    int                 error = 0;
    int                 index;
    PPM_OS_CONTEXT      os_context;
    TI_PPD_IF           ingress;
    TI_PPD_IF           egress[TI_PP_MAX_EGRESS_PROPERTY];

    /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    /* Ensure that the Session database is open to access. */
    if (ppm_mcb.sessiondb_status == INACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished since session database cannot be accessed\n", __FUNCTION__);
        return -1;
    }

    /* Make sure a valid session was passed. */
    if (ptr_session == NULL)
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to lacking input parameter\n", __FUNCTION__);
        return -1;
    }

    /* The number of egress records should be within the correct range. */
    if ((ptr_session->num_egress == 0) || (ptr_session->num_egress > TI_PP_MAX_EGRESS_PROPERTY))
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to bad input parameter (num_egress=%d, range:%d-%d)\n", __FUNCTION__, ptr_session->num_egress, 1, TI_PP_MAX_EGRESS_PROPERTY);
        return -1;
    }

    /*********************** End of Validation ***********************/

    /* Allocate memory for the new Session */
    ptr_internal_session = (PPM_SESSION *)ppm_mcb.os_table.malloc (sizeof(PPM_SESSION));
    if (ptr_internal_session == NULL)
    {
        /* Error: No memory available; generate the OOM event. */
        printk("ERROR: PP Operation %s cannot be accomplished due to lack of memory\n", __FUNCTION__);
        ti_ppm_event_handler (TI_PPM_OUT_OF_MEMORY, 0, 0);
        return -1;
    }

    /* Initialize the Session structure. */
    ppm_mcb.os_table.memset ((void *)&ptr_internal_session->session_vpid_mapping, 0,
                             sizeof(ptr_internal_session->session_vpid_mapping));
    ppm_mcb.os_table.memcpy ((void *)&ptr_internal_session->session, (void *)ptr_session, sizeof(TI_PP_SESSION));

    /* Calculate the hash code for the new session being created. */
    ptr_internal_session->hash_code = hash ((unsigned char *)&ptr_session->ingress.l2_packet, LUT_SIZE, 0);

    /* CRITICAL SECTION STARTS:-
     * This code falls into the critical section because we need to associate the VPID and Sessions together
     * We cannot afford to have the VPID be deleted while this association is being created. */
    ppm_mcb.os_table.critical_section_start(&os_context);

    /* Get the pointer to the original session. This check is being done in the critical section because there
     * is a possibility that the session gets deleted while it is being modified. */
    ptr_original_session = ppm_mcb.session_database[session_handle];
    if (ptr_original_session != NULL)
    {
        /* Copy the session handle. */
        ptr_internal_session->session_handle = ptr_original_session->session_handle;

        /* Link the sessions with all the Egress VPID. Cycle through all the Egress Framing Records! */
        for (index = 0; index < ptr_session->num_egress; index++)
        {
            PPM_VPID*   ptr_vpid;
            PPM_PID*    ptr_pid;

            /* Create the Session-Egress VPID association */
            error = ti_ppm_session_vpid_association (ptr_session->egress[index].vpid_handle, ptr_internal_session, index);
            if (error != 0)
                break;

            /* Remember the Egress PID and VPID Information. This information needs to be passed down to the PPD.
             * We know for sure the VPID is correct because the association function has not returned an error. */
            ptr_vpid = ppm_mcb.vpid_database[ptr_session->egress[index].vpid_handle];
            ptr_pid  = ppm_mcb.pid_database[ptr_vpid->vpid.parent_pid_handle];
            egress[index].vpid = &ptr_vpid->vpid;
            egress[index].pid  = &ptr_pid->pid;
        }

        /* Check if we aborted because of an error or not? */
        if (error == 0)
        {
            /* All the Egress VPID associations have been done. Was an Ingress VPID Specified? */
            if (ptr_session->ingress.vpid_handle != TI_PP_IGNORE_INGRESS_VPID)
            {
                PPM_VPID*   ptr_vpid;
                PPM_PID*    ptr_pid;

                /* YES. Create the Session-Ingress VPID association. */
                error = ti_ppm_session_vpid_association (ptr_session->ingress.vpid_handle, ptr_internal_session, index);
                if (error == 0)
                {
                    /* Remember the Ingress PID and VPID Information. */
                    ptr_vpid = ppm_mcb.vpid_database[ptr_session->ingress.vpid_handle];
                    ptr_pid  = ppm_mcb.pid_database[ptr_vpid->vpid.parent_pid_handle];
                    ingress.vpid = &ptr_vpid->vpid;
                    ingress.pid  = &ptr_pid->pid;
                }
                else
                {
                    printk("ERROR: PP Operation %s cannot be accomplished since ingress ti_ppm_session_vpid_association operation failed\n", __FUNCTION__);
                }
            }
            else
            {
                /* The Ingress Interface was unspecified. Initialize the information correctly so that the PPD knows about it. */
                ingress.pid  = NULL;
                ingress.vpid = NULL;
            }
        }
        else
        {
            printk("ERROR: PP Operation %s cannot be accomplished since egress ti_ppm_session_vpid_association operation failed\n", __FUNCTION__);
        }

        /* If no error has been detected; create the session in the PDSP. */
        if (error == 0)
        {
            ptr_session->session_handle = session_handle;

            /* Modify the session in the PDSP. */
            if (ti_ppd_modify_session (ptr_session, &ingress, &egress[0]) < 0)
            {
                /* PDSP Modify Session Failed. */
                printk("ERROR: PP Operation %s cannot be accomplished since ti_ppd_modify_session operation failed\n", __FUNCTION__);
                error = 0x3;
            }
            else
            {
                /* PDSP Modify Session was succesful. Link the new session into the database. */
                ppm_mcb.session_database[session_handle] = ptr_internal_session;

                /* Add the allocated Session Handle to the structure also. */
                ptr_session->session_handle                  = (unsigned char)session_handle;
                ptr_internal_session->session.session_handle = (unsigned char)session_handle;

                /* Delete the old entry and add the new entry to the Session Hash Table. */
                ti_ppm_delete_hash_table (session_handle, ptr_original_session->hash_code);
                ti_ppm_add_hash_table (session_handle, ptr_internal_session->hash_code);
            }
        }
    }
    else
    {
        /* Session does not exist and so it cannot be modified. */
        printk("ERROR: PP Operation %s cannot be accomplished since session_handle=%d does not exist\n", __FUNCTION__, session_handle);
        error = 0x4;
    }

    /* Check if an error was detected on the path. */
    if (error == 0)
    {
        /* Succesful: No Error detected and the session has been modified. Clean memory for the old session. */
        for (index = 0; index < TI_PP_MAX_VPID; index++)
        {
            /* Get the session-vpid association. */
            PPM_SESSION_VPID_INFO* ptr_session_vpid = ptr_original_session->session_vpid_mapping[index];
            if (ptr_session_vpid == NULL)
                break;

            /* Remove the session vpid association from the VPID */
            list_remove_node ((LIST_NODE**)&ptr_session_vpid->vpid->vpid_session_mapping, (LIST_NODE*)ptr_session_vpid);

            /* Clean up the memory. */
            ppm_mcb.os_table.free(ptr_session_vpid);
        }

        /* Clean up the memory allocated for the session.  */
        ppm_mcb.os_table.free(ptr_original_session);
    }
    /* Critical Section Stop */
    ppm_mcb.os_table.critical_section_end (&os_context);

    if (error == 0)
    {
        /* PDSP Modify Session was Successful. */
        ti_ppm_event_handler (TI_PPM_SESSION_MODIFIED, (unsigned int)session_handle, 0);
        return 0;
    }

    /* Error was detected; use the error code to determine if the severity is enough to warrant an
     * Event being generated. */
    if (error == 0x3)
        ti_ppm_event_handler (TI_PPM_MODIFY_SESSION_FAILED, (unsigned int)session_handle, 0);
    if (error == 0x1)
        ti_ppm_event_handler (TI_PPM_OUT_OF_MEMORY, 0, 0);

    /* Return error; since the modification was not succesful. */
    return -1;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_lookup_session
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to lookup the session handle in the PDSP and return
 *  its status.
 *
 * RETURNS       :
 *  1    -   Session is Alive in the PDSP
 *  0    -   Session does not exist in the PDSP.
 **************************************************************************/
int ti_ppm_lookup_session (unsigned char session_handle)
{
   /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status == INACTIVE)
        return 0;

    /* Ensure that the Session database is open to access. */
    if (ppm_mcb.sessiondb_status == INACTIVE)
        return 0;

    /*********************** End of Validation ***********************/

    /* Check the Session Database and see if the session handle is ALIVE or not? */
    if (ppm_mcb.session_database[session_handle] != NULL)
        return 1;

    /* Session is NOT active in the PDSP. */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_delete_session
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to delete the session.
 *
 * RETURNS       :
 *  0    -   Success
 *  <0   -   Error
 **************************************************************************/
int ti_ppm_delete_session (unsigned char session_handle, TI_PP_SESSION_STATS* ptr_stats)
{
    PPM_OS_CONTEXT          os_context;
    int                     index;
    PPM_SESSION*            ptr_internal_session;
    int                     error = 0;
    TI_PP_SESSION_STATS     stats;

    ppm_mcb.os_table.memset(&stats, 0, sizeof(TI_PP_SESSION_STATS));

    /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    /*********************** End of Validation ***********************/

    /* CRITICAL SECTION STARTS:-
     *  This code falls into the critical section because the VPID-Session Association
     *  mapping in the VPID can be altered in the context of other PPM calls as
     *  delete_vpid etc. */
    ppm_mcb.os_table.critical_section_start(&os_context);

    /* Get the pointer to the session. */
    ptr_internal_session = ppm_mcb.session_database[session_handle];
    if (ptr_internal_session != NULL)
    {
        /* Delete the session in the PDSP. */
        if (ti_ppd_delete_session (session_handle) == 0)
        {
            /* Session has been succesfully deleted in the PDSP so now cycle through all the
             * session-vpid mapping and remove it also. */
            for (index = 0; index < TI_PP_MAX_VPID; index++)
            {
                /* Get the session-vpid association. */
                PPM_SESSION_VPID_INFO* ptr_session_vpid = ptr_internal_session->session_vpid_mapping[index];
                if (ptr_session_vpid == NULL)
                    break;

                /* Remove the session vpid association from the VPID */
                list_remove_node ((LIST_NODE**)&ptr_session_vpid->vpid->vpid_session_mapping, (LIST_NODE*)ptr_session_vpid);

                /* Clean up the memory. */
                ppm_mcb.os_table.free(ptr_session_vpid);
            }

            /* Unhook the session from the global database. */
            ppm_mcb.session_database[session_handle] = NULL;

            /* Unhook the session from the HASH Table also. */
            if (ti_ppm_delete_hash_table (session_handle, ptr_internal_session->hash_code) < 0)
            {
                printk("ERROR: PP Operation %s cannot be accomplished since ti_ppm_delete_hash_table operation has failed\n", __FUNCTION__);
                error = 0x3;
            }
            else
            {
                /* Get the session stats from the PDSP if requested to be retreived. The stats are valid in the
                 * PDSP till the session handle is allocated to another session. This is not possible till we
                 * release the handle. */
                /* Get the Session Stats from the PP and copy them into the user supplied buffer. */
                ti_ppd_get_session_pkt_stats (session_handle, &stats);

                if (ptr_stats)
                {
                    ppm_mcb.os_table.memcpy(ptr_stats, &stats, sizeof(TI_PP_SESSION_STATS));
                }

                /* Release the session handle. */
                ti_ppm_release_handle (ptr_internal_session->session_handle, &ppm_mcb.session_free_list, &ppm_mcb.session_occupied_list);

                /* Clean up the memory allocated for the session.  */
                ppm_mcb.os_table.free(ptr_internal_session);
            }

            /* Decrement the number of sessions that exist */
            ppm_mcb.ppm_num_session = ppm_mcb.ppm_num_session - 1;
        }
        else
        {
            /* PDSP Failed to delete the session. */
            printk("ERROR: PP Operation %s cannot be accomplished since ti_ppd_delete_session operation has failed\n", __FUNCTION__);
            error = 0x1;
        }
    }
    else
    {
        /* No existing session. */
        printk("ERROR: PP Operation %s cannot be accomplished since session_handle=%d does not exist\n", __FUNCTION__, session_handle);
        error = 0x2;
    }

    /* Critical Section Stop */
    ppm_mcb.os_table.critical_section_end (&os_context);

    /* Generate an appropriate event. */
    if (error == 0)
    {
        /* Send an event to the world indicating that the session was succesfully deleted */
        ti_ppm_event_handler (TI_PPM_SESSION_DELETED, (unsigned int)session_handle, (unsigned int)&stats);

        /* Session has been succesfully deleted. */
        return 0;
    }

    /* Delete session failed; since the PDSP was unable to delete the session */
    if (error == 0x1)
        ti_ppm_event_handler (TI_PPM_DELETE_SESSION_FAILED, (unsigned int)session_handle, 0);
    else if (error == 0x3)
        ti_ppm_event_handler (TI_PPM_INTERNAL_ERROR, 1, (unsigned int)session_handle);

    /* Return error since session deletion failed. */
    return -1;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_flush_sessions
 **************************************************************************
 * DESCRIPTION   :
 *  The function flushes sessions from the session database.The function is
 *  overloaded and does either of the following:-
 *  a) Delete all sessions (vpid=-1)
 *  b) Delete all sessions for a VPID (0 <= vpid < max_number_vpid)
 *
 * RETURNS       :
 *  0                   -   Success
 *  <0                  -   Error
 **************************************************************************/
int ti_ppm_flush_sessions (int vpid_handle)
{
    int index;
    PPM_OS_CONTEXT          os_context;

    /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    /* Basic validation: Make sure the function is called with a valid VPID/-1 */
    if ((vpid_handle != -1) && ((vpid_handle < 0) || (vpid_handle > TI_PP_MAX_VPID)))
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to bad input parameter (vpid_handle=%d, range:%d-%d)\n", __FUNCTION__, vpid_handle, -1, TI_PP_MAX_VPID-1);
        return -1;
    }

    /*********************** End of Validation ***********************/

    /* Lock the session db for the duration of session deletion, so that other
       PPM APIs do not access the ppm_mcb.session_database meanwhile.
       The actual locks are held only in the ti_ppm_delete_session, this extra
       variable lock helps us ensure that no other API gets an inconsistent
       snapshot of the session db meanwhile.
    */
    ppm_mcb.os_table.critical_section_start(&os_context);
    ppm_mcb.sessiondb_status = INACTIVE;

    /* Cycle through all the Sessions that can exist in the system. */
    for (index = 0; index < TI_PP_MAX_ACCLERABLE_SESSIONS; index++)
    {
        /* Check if the session exists or not? */
        if (ppm_mcb.session_database[index] != NULL)
        {
            /* Copy the Session information only if the following conditions are met:-
             *  i)  VPID handle is -1 i.e. dont care; flush all active sessions
             *  ii) The VPID handle of the session matches the one specified.*/
            if ((vpid_handle == -1) || (ti_ppm_session_vpid_match(ppm_mcb.session_database[index], vpid_handle) == 1))
            {
                /* Delete the Session. */
                ti_ppm_delete_session(ppm_mcb.session_database[index]->session.session_handle, NULL);
            }
        }
    }

    /* Session deletion done. Unlock database. */
    ppm_mcb.sessiondb_status = ACTIVE;
    ppm_mcb.os_table.critical_section_end (&os_context);

    /* Return success  */
    return 0;
}
/**************************************************************************
 * FUNCTION NAME : ti_ppm_set_ack_suppression
 **************************************************************************
 * DESCRIPTION   :
 *  The function sets the packet processor to do Ack Suppression or not to
 *  do in case Tdox is Enabled.
 *
 * RETURNS       :
 *  0                   -   Success
 *  <0                  -   Error
 **************************************************************************/
int ti_ppm_set_ack_suppression(int enDis)
{
    PPM_OS_CONTEXT          os_context;
    int ret;

    ppm_mcb.os_table.critical_section_start(&os_context);

    ret = ti_ppd_set_ack_suppression(enDis);

    /* Critical Section Stop */
    ppm_mcb.os_table.critical_section_end (&os_context);

    return ret;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_set_mta_mac_address
 **************************************************************************
 * DESCRIPTION   :
 *  The function sets MTA MAC address for the packet processor
 *
 * RETURNS       :
 *  0                   -   Success
 *  <0                  -   Error
 **************************************************************************/
int ti_ppm_set_mta_mac_address (unsigned char* mtaAddress)
{
    if (ppm_mcb.pdsp_status == INACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is INACTIVE\n", __FUNCTION__);
        return -1;
    }

    ti_ppd_set_mta_mac_address(mtaAddress);
    
    return 0;
}

#ifdef CONFIG_INTEL_PP_TUNNEL_SUPPORT
/**************************************************************************
 * FUNCTION NAME : ti_ppm_set_tunnel_mode
 **************************************************************************
 * DESCRIPTION   :
 *  The function sets the packet processor to tunnel mode or regular session mode
 *
 * RETURNS       :
 *  0                   -   Success
 *  <0                  -   Error
 **************************************************************************/
int ti_ppm_set_tunnel_mode(int tunnelMode)
{
    PPM_OS_CONTEXT          os_context;

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    /* Ensure that the Session database is open to access. */
    if (ppm_mcb.sessiondb_status == INACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished since session database cannot be accessed\n", __FUNCTION__);
        return -1;
    }

    /*********************** End of Validation ***********************/

    ppm_mcb.os_table.critical_section_start(&os_context);

    ti_ppd_set_tunnel_mode(tunnelMode);

    /* Critical Section Stop */
    ppm_mcb.os_table.critical_section_end (&os_context);

    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_set_cm_mac_address
 **************************************************************************
 * DESCRIPTION   :
 *  The function sets CM MAC address for the packet processor
 *
 * RETURNS       :
 *  0                   -   Success
 *  <0                  -   Error
 **************************************************************************/
int ti_ppm_set_cm_mac_address (Uint8* cmAddress)
{
    PPM_OS_CONTEXT          os_context;

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }
	
	/* Ensure that the Session database is open to access. */
    if (ppm_mcb.sessiondb_status == INACTIVE)
	{
	    printk("ERROR: PP Operation %s cannot be accomplished since sessiondb_status is INACTIVE\n", __FUNCTION__);
        return -1;
	}

    /*********************** End of Validation ***********************/
    
    ppm_mcb.os_table.critical_section_start(&os_context);

    ti_ppd_set_cm_mac_address(cmAddress);
    
    /* Critical Section Stop */
    ppm_mcb.os_table.critical_section_end (&os_context);
    
    return 0;
}
#endif

/**************************************************************************
 * FUNCTION NAME : ti_ppm_get_vpid
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to get all the VPID associated with a PID. The
 *  function is overloaded and can be used to retreive a list of all VPID
 *  that are active in the PP Sub-System by passing the pid_handle as -1; if
 *  a valid "pid_handle" is passed the function returns a list of VPID
 *  created on that "pid_handle" ONLY
 *
 * RETURNS       :
 *  Number of VPID's populated  -   Success
 *  <0                          -   Error
 **************************************************************************/
int ti_ppm_get_vpid (int pid_handle, int num_vpid, TI_PP_VPID* ptr_vpid)
{
    int                 index;
    int                 counter = 0;
    PPM_OS_CONTEXT      os_context;

    /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    /* Is there space allocated for the VPID? */
    if (num_vpid == 0)
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to bad input parameter (num_vpid=0)\n", __FUNCTION__);
        return counter;
    }

    /* Basic validation: Make sure the function is called with a valid pointer */
    if (ptr_vpid == NULL)
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to bad input parameter (ptr_vpid=NULL)\n", __FUNCTION__);
        return -1;
    }

    /*********************** End of Validation ***********************/

    /* CRITICAL SECTION STARTS:-
     *  This code falls into the critical section because while retreiving the VPID
     *  information we cannot have have the delete VPID call execute on the global database;
     *  since this can cause inconsistencies. */
    ppm_mcb.os_table.critical_section_start(&os_context);

    /* Cycle through all the VPID that can exist in the system. */
    for (index = 0; index < TI_PP_MAX_VPID; index++)
    {
        /* Check if the VPID exists in the System? */
        if (ppm_mcb.vpid_database[index] != NULL)
        {
            /* Copy the VPID information only if the following conditions are met
             *  a) PID handle is -1 i.e. dont care; return a list of all active VPID
             *  b) PID Handle for the VPID matches the "pid_handle" argument passed. */
            if ((pid_handle == -1) || (ppm_mcb.vpid_database[index]->ptr_pid->pid.pid_handle == pid_handle))
            {
                /* Copy the contents. */
                ppm_mcb.os_table.memcpy ((void *)&ptr_vpid[counter], (void *)&ppm_mcb.vpid_database[index]->vpid, sizeof(TI_PP_VPID));

                /* Increment the number of VPID that have been copied into the user supplied buffer. */
                counter = counter + 1;

                /* Have we copied more than what has been specified by the user? */
                if (counter == num_vpid)
                    break;
            }
        }
    }

    /* Critical Section Stop */
    ppm_mcb.os_table.critical_section_end (&os_context);

    /* Return the number of VPID that has been copied. */
    return counter;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_get_pid
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to get all the PID that exist in the PDSP. The
 *  function populates only "num_pid" entries at the location pointed by
 *  "ptr_pid"
 *
 * RETURNS       :
 *  Number of PID  -   Success
 *  <0             -   Error
 **************************************************************************/
int ti_ppm_get_pid (int num_pid, TI_PP_PID* ptr_pid)
{
    int                 index;
    int                 counter = 0;
    PPM_OS_CONTEXT      os_context;

    /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    /* Is there space allocated for the PID? */
    if (num_pid == 0)
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to bad input parameter (num_pid=0)\n", __FUNCTION__);
        return counter;
    }

    /* Basic validation: Make sure the function is called with a valid pointer */
    if (ptr_pid == NULL)
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to bad input parameter (ptr_pid=NULL)\n", __FUNCTION__);
        return -1;
    }

    /*********************** End of Validation ***********************/

    /* CRITICAL SECTION STARTS:-
     *  This code falls into the critical section because while retreiving the PID
     *  information we cannot have have the delete PID call execute on the global database;
     *  since this can cause inconsistencies. */
    ppm_mcb.os_table.critical_section_start(&os_context);

    /* Cycle through all the PID that can exist in the system. */
    for (index = 0; index < TI_PP_MAX_PID; index++)
    {
        /* Check if the PID exists in the System or not? */
        if (ppm_mcb.pid_database[index] != NULL)
        {
            /* Copy the contents. */
            ppm_mcb.os_table.memcpy ((void *)&ptr_pid[counter], (void *)&ppm_mcb.pid_database[index]->pid, sizeof(TI_PP_PID));

            /* Increment the number of active PID */
            counter = counter + 1;

            /* Check if we reached the condition where we have exceeded the space allocated? */
            if (counter == num_pid)
                break;
        }
    }

    /* Critical Section Stop */
    ppm_mcb.os_table.critical_section_end (&os_context);

    /* Return the number of PID that has been copied. */
    return counter;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_session_vpid_match
 **************************************************************************
 * DESCRIPTION   :
 *  The function matches the "vpid_handle" with the session_vpid_mapping that
 *  exists in "ptr_session".
 *
 * RETURNS       :
 *  1   -   Session exists on the VPID handle
 *  0   -   Session does not exist on the VPID handle.
 **************************************************************************/
static int ti_ppm_session_vpid_match (PPM_SESSION* ptr_session, int vpid_handle)
{
    int index;

    /* Cycle through all the Session Mapping. */
    for (index =0; index < TI_PP_MAX_VPID; index++)
    {
        PPM_SESSION_VPID_INFO* ptr_session_vpid_info;

        /* Get the session VPID mapping information */
        ptr_session_vpid_info = ptr_session->session_vpid_mapping[index];

        /* If no mapping exists there is no match. */
        if (ptr_session_vpid_info == NULL)
            return 0;

        /* Check for match? */
        if (ptr_session_vpid_info->vpid->vpid_handle->handle == vpid_handle)
            return 1;
    }

    /* No match found. */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_get_session
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to get information for all the sessions active
 *  in the PDSP. The function is overloaded and can be used to retrieve
 *  either of the following:-
 *  a) Number of sessions in the PP (vpid=-1; session = NULL; num_sessions=0)
 *  b) Information of all sessions in the PP (vpid = -1)
 *  c) Information of all sessions for a VPID
 *
 * RETURNS       :
 *  Number of Sessions  -   Success
 *  <0                  -   Error
 **************************************************************************/
int ti_ppm_get_session (int vpid_handle, int num_sessions, unsigned char* session_handle)
{
    int                 index;
    int                 counter = 0;
    PPM_OS_CONTEXT      os_context;

    /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    /* Ensure that the Session database is open to access. */
    if (ppm_mcb.sessiondb_status == INACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished since session database cannot be accessed\n", __FUNCTION__);
        return -1;
    }

    /* Basic validation: Make sure the function is called with a valid pointer */
    if ((num_sessions != 0) && (session_handle == NULL))
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to bad input parameter (session_handle=NULL)\n", __FUNCTION__);
        return -1;
    }

    /*********************** End of Validation ***********************/

    /* Check if we need to return the number of all the sessions that exist in the PP System */
    if ((vpid_handle == -1) && (session_handle == NULL) && (num_sessions == 0))
        return ppm_mcb.ppm_num_session;

    /* CRITICAL SECTION STARTS:-
     *  This code falls into the critical section because while retreiving the Session
     *  information we cannot have have the delete Session call execute on the global database;
     *  since this can cause inconsistencies. */
    ppm_mcb.os_table.critical_section_start(&os_context);

    /* Cycle through all the Sessions that can exist in the system. */
    for (index = 0; index < TI_PP_MAX_ACCLERABLE_SESSIONS; index++)
    {
        /* Check if the session exists or not? */
        if (ppm_mcb.session_database[index] != NULL)
        {
            /* Copy the Session information only if the following conditions are met:-
             *  i)  VPID handle is -1 i.e. dont care; return a list of all active session
             *  ii) The VPID handle of the session matches the one specified.*/
            if ((vpid_handle == -1) || (ti_ppm_session_vpid_match(ppm_mcb.session_database[index], vpid_handle) == 1))
            {
                /* Copy the Session Handle. */
                if (session_handle != NULL)
                    session_handle[counter] = (unsigned char)ppm_mcb.session_database[index]->session.session_handle;

                /* Increment the number of active sessions */
                counter = counter + 1;

                /* Check if we reached the condition where we have exceeded the space allocated? */
                if (counter == num_sessions)
                    break;
            }
        }
    }

    /* Critical Section Stop */
    ppm_mcb.os_table.critical_section_end (&os_context);

    /* Return the number of sessions  */
    return counter;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_get_session_info
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to get the session information from a session
 *  handle.
 *
 * RETURNS       :
 *  0   -   Success
 *  <0  -   Error
 **************************************************************************/
int ti_ppm_get_session_info (int session_handle, TI_PP_SESSION* ptr_session)
{
    PPM_OS_CONTEXT          os_context;
    int                     rc = -1;

    /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    /* Ensure that the Session database is open to access. */
    if (ppm_mcb.sessiondb_status == INACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished since session database cannot be accessed\n", __FUNCTION__);
        return -1;
    }

    /* Basic validation: Make sure the function is called with a valid pointer */
    if (ptr_session == NULL)
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to bad input parameter (ptr_session=NULL)\n", __FUNCTION__);
        return -1;
    }

    /*********************** End of Validation ***********************/

    ppm_mcb.os_table.critical_section_start(&os_context);

    /* Make sure that the session exists. */
    if (ppm_mcb.session_database[session_handle] != NULL)
    {
        /* Copy the session information. */
        ppm_mcb.os_table.memcpy((void*)ptr_session, (void*)&ppm_mcb.session_database[session_handle]->session, sizeof(TI_PP_SESSION));
        rc = 0;
    }

    ppm_mcb.os_table.critical_section_end (&os_context);

    /* Session did not exist. */
    return rc;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_get_vpid_info
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to get the VPID Information block given a handle
 *
 * RETURNS       :
 *  0   -   Success
 *  <0  -   Error
 **************************************************************************/
int ti_ppm_get_vpid_info (int vpid_handle, TI_PP_VPID* ptr_vpid)
{
    PPM_OS_CONTEXT          os_context;
    int                     rc = -1;

    /*********************** Parameter Validation ***********************/

    /* Basic validation: Make sure the function is called with a valid pointer */
    if (ptr_vpid == NULL)
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to bad input parameter (ptr_vpid=NULL)\n", __FUNCTION__);
        return -1;
    }

    /*********************** End of Validation ***********************/
    ppm_mcb.os_table.critical_section_start(&os_context);

    /* Make sure that the vpid exists. */
    if (ppm_mcb.vpid_database[vpid_handle] != NULL)
    {
        /* Copy the session information. */
        ppm_mcb.os_table.memcpy((void*)ptr_vpid, (void*)&ppm_mcb.vpid_database[vpid_handle]->vpid, sizeof(TI_PP_VPID));
        rc = 0;
    }
    else
    {
        printk("ERROR: PP Operation %s cannot be accomplished since vpid_handle=%d does not exist\n", __FUNCTION__);
    }
    ppm_mcb.os_table.critical_section_end (&os_context);

    /* VPID did not exist. */
    return rc;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_get_pid_info
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to get the PID Information block given a handle
 *
 * RETURNS       :
 *  0   -   Success
 *  <0  -   Error
 **************************************************************************/
int ti_ppm_get_pid_info (int pid_handle, TI_PP_PID* ptr_pid)
{
    PPM_OS_CONTEXT          os_context;
    int                     rc = -1;

    /*********************** Parameter Validation ***********************/

    /* Basic validation: Make sure the function is called with a valid pointer */
    if (ptr_pid == NULL)
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to bad input parameter (ptr_pid=NULL)\n", __FUNCTION__);
        return -1;
    }

    /*********************** End of Validation ***********************/
    ppm_mcb.os_table.critical_section_start(&os_context);

    /* Make sure that the session exists. */
    if (ppm_mcb.pid_database[pid_handle] != NULL)
    {
        /* Copy the session information. */
        ppm_mcb.os_table.memcpy((void*)ptr_pid, (void*)&ppm_mcb.pid_database[pid_handle]->pid, sizeof(TI_PP_PID));
        rc = 0;
    }
    else
    {
        printk("ERROR: PP Operation %s cannot be accomplished since pid_handle=%d does not exist\n", __FUNCTION__);
    }

    ppm_mcb.os_table.critical_section_end (&os_context);

    /* PID did not exist. */
    return rc;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_register_event_handler
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to register an event handler.
 *
 * RETURNS       :
 *      Handle to the event handler   -   Success
 *      0                             -   Error
 **************************************************************************/
unsigned int ti_ppm_register_event_handler (void (*event_handler)(unsigned int event_id, unsigned int param1, unsigned int param2))
{
    PPM_EVENT_HANDLER*  ptr_internal_event_handler;

    /* Validate the arguments passed. */
    if (event_handler == NULL)
        return 0;

    /* Allocate memory for an internal event handler information */
    ptr_internal_event_handler = (PPM_EVENT_HANDLER *)ppm_mcb.os_table.malloc (sizeof(PPM_EVENT_HANDLER));
    if (ptr_internal_event_handler == NULL)
        return 0;

    /* Initialize the allocated block of memory. */
    ppm_mcb.os_table.memset ((void *)ptr_internal_event_handler, 0,
                       sizeof(PPM_EVENT_HANDLER));

    /* Copy the contents of the event handler into the internal structure. */
    ptr_internal_event_handler->event_handler = event_handler;

    /* Add the event handler to the list of event handlers in the System. */
    list_add ((LIST_NODE **)&ppm_mcb.event_info, (LIST_NODE *)ptr_internal_event_handler);

    /* Return the handle of the event information */
    return (unsigned int)ptr_internal_event_handler;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_unregister_event_handler
 **************************************************************************
 * DESCRIPTION   :
 *  The function is used to unregister an event handler.
 *
 * RETURNS       :
 *      0   -   Success
 *      <0  -   Error
 **************************************************************************/
int ti_ppm_unregister_event_handler (unsigned int handle_event_handler)
{
    PPM_EVENT_HANDLER*  ptr_internal_event_handler;

    /* Traverse the list of all event handlers and find the correct one to remove. */
    ptr_internal_event_handler = (PPM_EVENT_HANDLER*)list_get_head ((LIST_NODE**)&ppm_mcb.event_info);
    while (ptr_internal_event_handler != NULL)
    {
        /* Check if we got a match? */
        if (ptr_internal_event_handler == (PPM_EVENT_HANDLER *)handle_event_handler)
        {
            /* YES. Delete the event handler. */
            list_remove_node ((LIST_NODE **)&ppm_mcb.event_info, (LIST_NODE *)ptr_internal_event_handler);

            /* Cleanup the allocated block of memory. */
            ppm_mcb.os_table.free(ptr_internal_event_handler);

            /* Event Handler has been successfully deleted. */
            return 0;
        }

        /* Get the next event handler. */
        ptr_internal_event_handler = (PPM_EVENT_HANDLER*)list_get_next((LIST_NODE*)ptr_internal_event_handler);
    }

    /* Control comes here implies that the event handler passed was incorrect. */
    return -1;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_event_handler
 **************************************************************************
 * DESCRIPTION   :
 *  The function is the dispatcher code that passes events from PPM
 *  entities to the registered event handler.
 **************************************************************************/
void ti_ppm_event_handler (unsigned int event_id, unsigned int param1, unsigned int param2)
{
    PPM_EVENT_HANDLER*  ptr_internal_event_handler;

    /* Traverse the list of all event handlers and pass the events to all of them. */
    ptr_internal_event_handler = (PPM_EVENT_HANDLER*)list_get_head ((LIST_NODE**)&ppm_mcb.event_info);
    while (ptr_internal_event_handler != NULL)
    {
        /* Post the message to the event handler. */
        ptr_internal_event_handler->event_handler (event_id, param1, param2);

        /* Get the next event handler. */
        ptr_internal_event_handler = (PPM_EVENT_HANDLER*)list_get_next((LIST_NODE*)ptr_internal_event_handler);
    }

    /* Our work is done. */
    return;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_get_session_stats
 **************************************************************************
 * DESCRIPTION   :
 *  The function is called to get the statistics of a particular session.
 *
 * RETURNS       :
 *      0   -   Success
 *      <0  -   Error
 **************************************************************************/
int ti_ppm_get_session_stats (unsigned char session_handle, TI_PP_SESSION_STATS* ptr_stats)
{
    PPM_SESSION*            ptr_internal_session;
    TI_PP_SESSION_STATS  ppd_session_stats;

    /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status == INACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is INACTIVE\n", __FUNCTION__);
        return -1;
    }

    /* In case of PSM, do not fail the opration - just return all 0 */
    if (ppm_mcb.pdsp_status == PPM_PSM)
    {
        ppm_mcb.os_table.memset((void *)ptr_stats, 0, sizeof(TI_PP_SESSION_STATS));
        return 0;
    }

    /* Ensure that the Session database is open to access. */
    if (ppm_mcb.sessiondb_status == INACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished since session database cannot be accessed\n", __FUNCTION__);
        return -1;
    }

    /* Validate the arguments */
    if (ptr_stats == NULL)
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to bad input parameter (ptr_stats=NULL)\n", __FUNCTION__);
        return -1;
    }

    /* Make sure that the session is alive */
    ptr_internal_session = ppm_mcb.session_database[session_handle];
    if (ptr_internal_session == NULL)
    {
        printk("ERROR: PP Operation %s cannot be accomplished since session does not exist\n", __FUNCTION__);
        return -1;
    }

    /*********************** End of Validation ***********************/

    /* Get the session statistics from the Packet Processor */
    if (ti_ppd_get_session_pkt_stats (session_handle, &ppd_session_stats) < 0)
        return -1;

    /* Copy the session stats into the user supplied buffer. */
    ppm_mcb.os_table.memcpy ((void *)ptr_stats, (void *)&ppd_session_stats, sizeof(TI_PP_SESSION_STATS));
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_clear_session_stats
 **************************************************************************
 * DESCRIPTION   :
 *  The function is called to clear the statistics of a particular session.
 *
 * RETURNS       :
 *      0   -   Success
 *      <0  -   Error
 **************************************************************************/
int ti_ppm_clear_session_stats (unsigned char session_handle)
{
    PPM_SESSION*            ptr_internal_session;

    /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    /* Ensure that the Session database is open to access. */
    if (ppm_mcb.sessiondb_status == INACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished since session database cannot be accessed\n", __FUNCTION__);
        return -1;
    }

    /* Make sure that the session is alive */
    ptr_internal_session = ppm_mcb.session_database[session_handle];
    if (ptr_internal_session == NULL)
    {
        printk("ERROR: PP Operation %s cannot be accomplished since session does not exist\n", __FUNCTION__);
        return -1;
    }

    /*********************** End of Validation ***********************/

    /* Clear the session statistics in the packet processor. */
    if (ti_ppd_clear_session_pkt_stats (session_handle) < 0)
        return -1;

    /* Session statistics have been succesfully cleared */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_get_vpid_stats
 **************************************************************************
 * DESCRIPTION   :
 *  The function is called to get the statistics of a VPID.
 *
 * RETURNS       :
 *      0   -   Success
 *      <0  -   Error
 **************************************************************************/
int ti_ppm_get_vpid_stats (unsigned char vpid_handle, TI_PP_VPID_STATS* ptr_stats)
{
    PPM_VPID*  ptr_internal_vpid;

    /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status == INACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is INACTIVE\n", __FUNCTION__);
        return -1;
    }

    /* In PSM mode we do not fail the operation but do not inquire the PP since it is down */
    if (ppm_mcb.pdsp_status == PPM_PSM)
    {
        ppm_mcb.os_table.memset((void *)ptr_stats, 0, sizeof(TI_PP_VPID_STATS));
        return 0;
    }

    /* Validate the arguments. */
    if (vpid_handle >= TI_PP_MAX_VPID)
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to bad input parameter (vpid_handle=%d, range:%d-%d)\n", __FUNCTION__, vpid_handle, 0, TI_PP_MAX_VPID-1);
        return -1;
    }

    /* Make sure that the VPID has been created and exists in the System. */
    ptr_internal_vpid = ppm_mcb.vpid_database[vpid_handle];
    if (ptr_internal_vpid == NULL)
    {
        printk("ERROR: PP Operation %s cannot be accomplished since vpid_handle=%d does not exist\n", __FUNCTION__);
        return -1;
    }

    /*********************** End of Validation ***********************/

    /* Get the VPID statistics from the Packet Processor */
    if (ti_ppd_get_vpid_stats (vpid_handle, ptr_stats) < 0)
    {
        printk("ERROR: PP Operation %s cannot be accomplished since ti_ppd_get_vpid_stats operation has failed\n", __FUNCTION__);
        return -1;
    }

    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_clear_vpid_stats
 **************************************************************************
 * DESCRIPTION   :
 *  The function is called to clear the statistics of a particular vpid.
 *
 * RETURNS       :
 *      0   -   Success
 *      <0  -   Error
 **************************************************************************/
int ti_ppm_clear_vpid_stats (unsigned char vpid_handle)
{
    PPM_VPID*           ptr_internal_vpid;
    TI_PP_VPID_STATS    stats;

    /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status == INACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is INACTIVE\n", __FUNCTION__);
        return -1;
    }

    /* Validate the arguments. */
    if (vpid_handle >= TI_PP_MAX_VPID)
    {
        printk("ERROR: PP Operation %s cannot be accomplished due to bad input parameter (vpid_handle=%d, range:%d-%d)\n", __FUNCTION__, vpid_handle, 0, TI_PP_MAX_VPID-1);
        return -1;
    }

    /* Make sure that the VPID has been created and exists in the System. */
    ptr_internal_vpid = ppm_mcb.vpid_database[vpid_handle];
    if (ptr_internal_vpid == NULL)
    {
        printk("ERROR: PP Operation %s cannot be accomplished since vpid_handle=%d does not exist\n", __FUNCTION__, vpid_handle);
        return -1;
    }

    /*********************** End of Validation ***********************/

    /* Clear the VPID statistics from the Packet Processor */
    if (ti_ppd_get_n_clear_vpid_stats (vpid_handle, &stats) < 0)
    {
        printk("ERROR: PP Operation %s cannot be accomplished since ti_ppd_get_n_clear_vpid_stats operation has failed\n", __FUNCTION__);
        return -1;
    }

    /* Successfully cleared the VPID statistics */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_get_global_stats
 **************************************************************************
 * DESCRIPTION   :
 *  The function is called to get the statistics of the packet processor
 *
 * RETURNS       :
 *      0   -   Success
 *      <0  -   Error
 **************************************************************************/
int ti_ppm_get_global_stats (TI_PP_GLOBAL_STATS* ptr_stats)
{
    /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status == INACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is INACTIVE\n", __FUNCTION__);
        return -1;
    }

    /* In PSM mode we do not fail the operation but do not inquire the PP since it is down */
    if (ppm_mcb.pdsp_status == PPM_PSM)
    {
        ppm_mcb.os_table.memset((void *)ptr_stats, 0, sizeof(TI_PP_GLOBAL_STATS));
        return 0;
    }

    /* Get the Global stats from the Packet Processor */
    if (ti_ppd_get_srl_pkt_stats (ptr_stats) < 0)
    {
        printk("ERROR: PP Operation %s cannot be accomplished since ti_ppd_get_srl_pkt_stats operation has failed\n", __FUNCTION__);
        return -1;
    }

    /* Successfully retreived the global statistics. */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_clear_global_stats
 **************************************************************************
 * DESCRIPTION   :
 *  The function is called to clear the statistics of the packet processor
 *
 * RETURNS       :
 *      0   -   Success
 *      <0  -   Error
 **************************************************************************/
int ti_ppm_clear_global_stats (void)
{
    TI_PP_GLOBAL_STATS  stats;

    /*********************** Parameter Validation ***********************/

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is INACTIVE\n", __FUNCTION__);
        return -1;
    }

    /*********************** End of Validation ***********************/

    /* Clear the Global stats from the Packet Processor */
    if (ti_ppd_get_n_clear_srl_pkt_stats (&stats) < 0)
    {
        printk("ERROR: PP Operation %s cannot be accomplished since ti_ppd_get_n_clear_srl_pkt_stats operation has failed\n", __FUNCTION__);
        return -1;
    }

    /* Successfully cleared the global statistics */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_get_version
 **************************************************************************
 * DESCRIPTION   :
 *  The function is called to get the version information from the packet
 *  processor.
 *
 * RETURNS       :
 *      0   -   Success
 *      <0  -   Error
 **************************************************************************/
int ti_ppm_get_version ( TI_PP_VERSION *version )
{
    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    if(ti_ppd_get_version( version ) != 0)
    {
        /* Error: couldn't retrieve version */
        printk("ERROR: PP Operation %s cannot be accomplished since ti_ppd_get_version operation has failed\n", __FUNCTION__);
        return -1;
    }

    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_health_check
 **************************************************************************
 * DESCRIPTION   :
 *  The function is called to determine the status of the Packet Processor
 *  PDSP.
 *
 * RETURNS       :
 *      0   -   Packet Processor PDSP is running perfectly.
 *      -1  -   Packet Processor PDSP has run into problems
 **************************************************************************/
int ti_ppm_health_check (void)
{
#ifdef SUPPORT_HEALTH_CHECK
    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    /* Ask the PPD if the PDSP are operational or not */
    if (ti_ppd_health_check () == 0)
    {
        return 0;
    }

    /* PDSP is not healthy */
    return -1;
#else
    /* PPD support not available always report healthy... */
    return 0;
#endif /* SUPPORT_HEALTH_CHECK */
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_qos_cluster_setup
 **************************************************************************
 * DESCRIPTION   :
 *  This function is called to setup a QoS cluster in PDSP.
 *
 * RETURNS       :
 *      0   -   Success
 *      -1  -   Error
 **************************************************************************/
int ti_ppm_qos_cluster_setup (unsigned char clst_indx, TI_PP_QOS_CLST_CFG* clst_cfg)
{
    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    if(ti_ppd_qos_cluster_setup(clst_indx, clst_cfg) != 0)
    {
        /* Error: PDSP was unable to configure the QoS cluster. */
        printk("ERROR: PP Operation %s cannot be accomplished since ti_ppd_qos_cluster_setup operation has failed\n", __FUNCTION__);
        return -1;
    }

    /* QoS cluster configured successfully in the PDSP */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_qos_cluster_enable
 **************************************************************************
 * DESCRIPTION   :
 *  This function enables specified QoS cluster.
 *
 * RETURNS       :
 *      0   -   Success
 *      -1  -   Error
 **************************************************************************/
int ti_ppm_qos_cluster_enable (unsigned char clst_indx)
{
    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    if(ti_ppd_qos_cluster_enable(clst_indx) != 0)
    {
        /* Error: PDSP was unable to enable the QoS cluster. */
        printk("ERROR: PP Operation %s cannot be accomplished since ti_ppd_qos_cluster_enable operation has failed\n", __FUNCTION__);
        return -1;
    }

    /* QoS cluster enabled successfully in the PDSP */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_qos_cluster_disable
 **************************************************************************
 * DESCRIPTION   :
 *  This function disables specified QoS cluster.
 *
 * RETURNS       :
 *      0   -   Success
 *      -1  -   Error
 **************************************************************************/
int ti_ppm_qos_cluster_disable (unsigned char clst_indx)
{
    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    if(ti_ppd_qos_cluster_disable(clst_indx) != 0)
    {
        /* Error: PDSP was unable to disable the QoS cluster. */
        printk("ERROR: PP Operation %s cannot be accomplished since ti_ppd_qos_cluster_disable operation has failed\n", __FUNCTION__);
        return -1;
    }

    /* QoS cluster disabled successfully in the PDSP */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_get_qos_q_stats
 **************************************************************************
 * DESCRIPTION   :
 *  This function retrieves the QoS statistics for the queue specified from
 *  the PDSP.
 *
 * RETURNS       :
 *      0   -   Success
 *      -1  -   Error
 **************************************************************************/
int ti_ppm_get_qos_q_stats (unsigned char qos_qnum, TI_PP_QOS_QUEUE_STATS *stats)
{
    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status == INACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is INACTIVE\n", __FUNCTION__);
        return -1;
    }

    /* In PSM mode we do not fail the operation but do not inquire the PP since it is down */
    if (ppm_mcb.pdsp_status == PPM_PSM)
    {
        ppm_mcb.os_table.memset ((void *)stats, 0, sizeof(TI_PP_QOS_QUEUE_STATS));
        return 0;
    }

    if(ti_ppd_get_qos_q_stats(qos_qnum, stats) != 0)
    {
        /* Error: PDSP was unable to get the QoS statistics for the specified queue */
        printk("ERROR: PP Operation %s cannot be accomplished since ti_ppd_get_qos_q_stats operation has failed\n", __FUNCTION__);
        return -1;
    }

    /* QoS statistics retrieved successfully from the PDSP */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_get_n_clear_qos_q_stats
 **************************************************************************
 * DESCRIPTION   :
 *  This function retrieves the QoS statistics for the queue specified from
 *  the PDSP. It also resets the statistics block in the PDSP.
 *
 * RETURNS       :
 *      0   -   Success
 *      -1  -   Error
 **************************************************************************/
int ti_ppm_get_n_clear_qos_q_stats (unsigned char qos_qnum, TI_PP_QOS_QUEUE_STATS *stats)
{
    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    if(ti_ppd_get_n_clear_qos_q_stats(qos_qnum, stats) != 0)
    {
        /* Error: PDSP was unable to perform the specified operation for this queue */
        printk("ERROR: PP Operation %s cannot be accomplished since ti_ppd_get_n_clear_qos_q_stats operation has failed\n", __FUNCTION__);
        return -1;
    }

    /* QoS statistics retrieved and reset successfully */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_enable_psm
 **************************************************************************
 * DESCRIPTION   :
 *  This function is called to enable Power Saving mode (PSM) of the
 *  PP prefetcher.
 *
 * RETURNS       :
 *      0   -   Success
 *      -1  -   Error
 **************************************************************************/
int ti_ppm_enable_psm (void)
{
    PPM_OS_CONTEXT      os_context;

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    /* Critical Section Start. */
    ppm_mcb.os_table.critical_section_start(&os_context);

    ppm_mcb.pdsp_status = PPM_PSM;

    if(ti_pp_enable_psm() != 0)
    {
        /* Critical Section End. */
        ppm_mcb.os_table.critical_section_end(&os_context);

        /* Error: PSM couldnt be enabled by PDSP */
        printk("ERROR: PP Operation %s cannot be accomplished since ti_pp_enable_psm operation has failed\n", __FUNCTION__);
        return -1;
    }

    /* Critical Section End. */
    ppm_mcb.os_table.critical_section_end(&os_context);

    /* PSM enabled successfully */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_disable_psm
 **************************************************************************
 * DESCRIPTION   :
 *  This function is called to disable the Power Saving mode (PSM) of the
 *  PP prefetcher.
 *
 * RETURNS       :
 *      0   -   Success
 *      -1  -   Error
 **************************************************************************/
int ti_ppm_disable_psm (void)
{
    PPM_OS_CONTEXT      os_context;

    /* Ensure that the PDSP is up and running. */
    if (ppm_mcb.pdsp_status != PPM_PSM)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "ACTIVE");
        return -1;
    }

    /* Critical Section Start. */
    ppm_mcb.os_table.critical_section_start(&os_context);

    if(ti_pp_disable_psm() != 0)
    {
        /* Critical Section End. */
        ppm_mcb.os_table.critical_section_end(&os_context);

        /* Error: PSM couldnt be disabled by PDSP */
        printk("ERROR: PP Operation %s cannot be accomplished since ti_pp_disable_psm operation has failed\n", __FUNCTION__);
        return -1;
    }

    ppm_mcb.pdsp_status = ACTIVE;

    /* Critical Section End. */
    ppm_mcb.os_table.critical_section_end(&os_context);

    /* PSM disabled successfully */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_pp_get_status
 **************************************************************************
 * DESCRIPTION   :
 *  This function is called to get the status of the PP
 *
 * RETURNS       :
 *      pdsp_status (ACTIVE, INACTIVE, PPM_PSM)
 **************************************************************************/
PPM_STATUS ti_pp_get_status(void)
{
    return ppm_mcb.pdsp_status;
}


/**************************************************************************
 * FUNCTION NAME : ti_ppm_init_handles
 **************************************************************************
 * DESCRIPTION   :
 *  The function initializes the handle lists. The PPM is responsible for
 *  the handle generation for PID, VPID and Sessions.
 *
 * RETURNS       :
 *      0   -   Success
 *      <0  -   Error
 **************************************************************************/
static int ti_ppm_init_handles (LIST_NODE** ptr_list, int num_elements)
{
    int           index;
    PPM_HANDLE*   ptr_handle;

    /* Create and Initialize the list with "num_elements" */
    for (index = 0; index < num_elements; index++)
    {
        /* Allocate memory for the handle. */
        ptr_handle = (PPM_HANDLE*)ppm_mcb.os_table.malloc(sizeof(PPM_HANDLE));
        if (ptr_handle == NULL)
            return -1;

        /* Initialize the handle */
        ptr_handle->handle = (unsigned char) index;

        /* Add the handle to the free list. */
        list_add (ptr_list, (LIST_NODE*)ptr_handle);
    }

    /* Successfully initialized the handles. */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_deinit_handles
 **************************************************************************
 * DESCRIPTION   :
 *  The function de-initializes the handle lists.
 **************************************************************************/
static void ti_ppm_deinit_handles (LIST_NODE** ptr_list)
{
    PPM_HANDLE*   ptr_handle;

    /* Get the head of the list. */
    ptr_handle = (PPM_HANDLE *)list_remove (ptr_list);
    while (ptr_handle != NULL)
    {
        /* Clean the memory allocated. */
        ppm_mcb.os_table.free (ptr_handle);

        /* Get the next element in the list.*/
        ptr_handle = (PPM_HANDLE *)list_remove (ptr_list);
    }

    /* Work is completed. */
    return;
}


/**************************************************************************
 * FUNCTION NAME : ti_ppm_pp_event_handler
 **************************************************************************
 * DESCRIPTION   :
 *  The function handles the PP Firmware events.
 *  It introduces the corresponding PPM events accordingly.
 *  Currently only session timeout is handled.
 *
 * RETURNS       :None
 **************************************************************************/
void ti_ppm_pp_event_handler (unsigned short event_id, unsigned int param1, unsigned int param2)
{
    switch( event_id )
    {
        case TI_PPD_PP_EVENT_SESSION_IDLE:
        {
            ti_ppm_event_handler( TI_PP_SESSION_EXPIRATION,param1,param2 );
            break;
        }
        default:
            break;
    }

    return;
};


/**************************************************************************
 * FUNCTION NAME : ti_ppm_initialize
 **************************************************************************
 * DESCRIPTION   :
 *  The function is the initialization function that needs to be called to
 *  start and initialize the PDSP, PPM and SRD interfaces.
 *
 * RETURNS       :
 *      0   -   Success
 *      <0  -   Error
 **************************************************************************/
int ti_ppm_initialize (TI_PPM_OS_FUNC_TABLE* ptr_os_table)
{
    /* Basic Validations: Ensure that the OS abstraction function table is correctly initialized.
     * All the functions are correctly initialized. */
    if((ptr_os_table->memcpy == NULL) || (ptr_os_table->memset == NULL) || (ptr_os_table->memcmp == NULL) ||
       (ptr_os_table->malloc == NULL) || (ptr_os_table->free   == NULL) ||
       (ptr_os_table->critical_section_start == NULL) || (ptr_os_table->critical_section_end == NULL))
    {
        printk("FATAL ERROR: PP Operation %s cannot be accomplished since ptr_os_table operations are not set correctly\n", __FUNCTION__);
        return -1;
    }

    /* Initialize the PPM Master Control Block. */
    ptr_os_table->memset ((void *)&ppm_mcb, 0, sizeof(TI_PPM_MCB));

    /* Copy the OS Table into the PPM Master control block. */
    ptr_os_table->memcpy ((void *)&ppm_mcb.os_table, (void *)ptr_os_table, sizeof(TI_PPM_OS_FUNC_TABLE));

    /* Initialize the Packet Processor PDSP and PPD. */
    if (ti_ppd_sram_test() < 0)
    {
        printk("FATAL ERROR: PP Operation %s cannot be accomplished since ti_ppd_sram_test operation has failed\n", __FUNCTION__);
        return -1;
    }

    if (0 != ti_ppd_register_event_handler (ti_ppm_pp_event_handler))
    {
        printk("FATAL ERROR: PP Operation %s cannot be accomplished since ti_ppd_register_event_handler operation has failed\n", __FUNCTION__);
        return -1;
    }
    /* Initialize the PID Handles */
    if (ti_ppm_init_handles ((LIST_NODE**)&ppm_mcb.pid_free_list, TI_PP_MAX_PID) < 0)
    {
        /* Memory Allocation Failed: Cleanup the allocated PID handles if any */
        ti_ppm_deinit_handles ((LIST_NODE**)&ppm_mcb.pid_free_list);
        printk("FATAL ERROR: PP Operation %s cannot be accomplished since ti_ppm_init_handles for PIDs operation has failed\n", __FUNCTION__);
        return -1;
    }

    /* Initialize the VPID Handles. */
    if (ti_ppm_init_handles ((LIST_NODE**)&ppm_mcb.vpid_free_list, TI_PP_MAX_VPID) < 0)
    {
        /* Memory Allocation Failed: Cleanup the PID handles and any VPID handles */
        ti_ppm_deinit_handles ((LIST_NODE**)&ppm_mcb.pid_free_list);
        ti_ppm_deinit_handles ((LIST_NODE**)&ppm_mcb.vpid_free_list);
        printk("FATAL ERROR: PP Operation %s cannot be accomplished since ti_ppm_init_handles for VPIDs operation has failed\n", __FUNCTION__);
        return -1;
    }

    /* Initialize the Session Handles. */
    if (ti_ppm_init_handles ((LIST_NODE**)&ppm_mcb.session_free_list, TI_PP_MAX_ACCLERABLE_SESSIONS) < 0)
    {
        /* Memory Allocation Failed: Cleanup the PID/VPID handles and any Session handles */
        ti_ppm_deinit_handles ((LIST_NODE**)&ppm_mcb.pid_free_list);
        ti_ppm_deinit_handles ((LIST_NODE**)&ppm_mcb.vpid_free_list);
        ti_ppm_deinit_handles ((LIST_NODE**)&ppm_mcb.session_free_list);
        printk("FATAL ERROR: PP Operation %s cannot be accomplished since ti_ppm_init_handles for SESSIONs operation has failed\n", __FUNCTION__);
        return -1;
    }


    /* The PPM and SR PDSP are now up and running. */
    ppm_mcb.pdsp_status = ACTIVE;

    /* Initialize the session_db status to active. */
    ppm_mcb.sessiondb_status = ACTIVE;

    /* Return success. */
    return 0;
}

/**************************************************************************
 * FUNCTION NAME : ti_ppm_deinitialize
 **************************************************************************
 * DESCRIPTION   :
 *  The function is the close & shutdown the PPM, PPD and the PDSP.
 *
 * RETURNS       :
 *      0   -   Success
 *      <0  -   Error
 **************************************************************************/
int ti_ppm_deinitialize (void)
{
    int                 index;
    PPM_PID*            ptr_internal_pid;
    PPM_EVENT_HANDLER*  ptr_internal_event_handler;

    /* Cycle through all the PID and delete them. Deleting a PID internally implies
     * that the VPID and all sessions for the VPID are also deleted. */
    for (index = 0; index < TI_PP_MAX_PID; index++)
    {
        /* Get the PID Information. */
        ptr_internal_pid = ppm_mcb.pid_database[index];

        /* Is this a valid PID and if so delete it... */
        if (ptr_internal_pid != NULL)
            ti_ppm_delete_pid (ptr_internal_pid->pid.pid_handle);
    }

    /* Cycle through all the event handler and clear them */
    ptr_internal_event_handler = (PPM_EVENT_HANDLER*)list_get_head ((LIST_NODE**)&ppm_mcb.event_info);
    while (ptr_internal_event_handler != NULL)
    {
        /* Unregister the event handler. */
        ti_ppm_unregister_event_handler ((unsigned int)ptr_internal_event_handler);

        /* Get the next event handler. */
        ptr_internal_event_handler = (PPM_EVENT_HANDLER*)list_get_head((LIST_NODE**)&ppm_mcb.event_info);
    }

    /* Lets make sure none of the PPM calls get executed from this point onwards... */
    ppm_mcb.pdsp_status = INACTIVE;

    /* Concatenate the Free and Occupied Lists for handles. */
    list_cat ((LIST_NODE**)&ppm_mcb.session_free_list, (LIST_NODE**)&ppm_mcb.session_occupied_list);
    list_cat ((LIST_NODE**)&ppm_mcb.vpid_free_list, (LIST_NODE**)&ppm_mcb.vpid_occupied_list);
    list_cat ((LIST_NODE**)&ppm_mcb.pid_free_list, (LIST_NODE**)&ppm_mcb.pid_occupied_list);

    /* Once the lists are concatenated free all the elements in the list. */
    ti_ppm_deinit_handles ((LIST_NODE**)&ppm_mcb.session_free_list);
    ti_ppm_deinit_handles ((LIST_NODE**)&ppm_mcb.vpid_free_list);
    ti_ppm_deinit_handles ((LIST_NODE**)&ppm_mcb.pid_free_list);

    /* Close the PPD and PDSP */
    if (ti_ppd_exit() < 0)
        return -1;

    /* Deinitialization was successful. */
    return 0;
}

int ti_ppm_dispaly_session_info(int session_id)
{
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    if (ppm_mcb.session_database[session_id] == NULL)
    {
        printk("Session %d inactive\n", session_id);
        return -1;
    }

    ti_ppd_dispaly_session_info(session_id);
    return 0;
}

int ti_ppm_dispaly_qos_queue_info(int queue_id)
{
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    ti_ppd_dispaly_qos_queue_info(queue_id);
    return 0;
}

int ti_ppm_dispaly_qos_cluster_info(int cluster_id)
{
    if (ppm_mcb.pdsp_status != ACTIVE)
    {
        printk("ERROR: PP Operation %s cannot be accomplished while PP status is %s\n", __FUNCTION__, ppm_mcb.pdsp_status == INACTIVE ? "INACTIVE" : "PPM_PSM");
        return -1;
    }

    ti_ppd_dispaly_qos_cluster_info(cluster_id);
    return 0;
}

