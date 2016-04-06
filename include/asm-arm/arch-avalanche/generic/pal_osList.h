/*
 *
 * pal_osList.h
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


/** \file   pal_osList.h
    \brief  OsLIST Services Header File

    This file declares APIs for handling simple bi-directional linked
    list that works with arbitrary data objects in a thread-safe manner.
    Movement of elements within the OsLIST is FIFO mode. 

    Only requirement on Elements strung onto the OsLIST is that they
    must begin with an OsLIST header comprising of a forward and reverse
    pointers. No Memory allocation or freeing is performed by these APIs.
    Memory allocation/freeing must be handled outside by the caller
    

    @author     PSP Architecture Team
    @version    1.0
 */

#ifndef __PAL_OSLIST_H__
#define __PAL_OSLIST_H__

#include "pal_defs.h"
#include "pal_os.h"

/**
 * \defgroup PalOSList PAL OS List Interface
 * 
 * PAL OS List Interface
 * \{
 */

/** \name PAL OS List Interface
 *  PAL OS List Interface
 * \{
 */

/**
 * \brief List Node Header
 * 
 * List node header used by all elements as the first member
 */
typedef struct 
{
    Ptr next;       /**< pointer to next node on the list */
    Ptr prev;       /**< pointer to previous node on the list */
} PAL_OsListNodeHeader;

/**
 * \def   PAL_OSLIST_MKNODE(hNode)
 * 
 *      This macro stitches link pointers of given node to
 *      reset conditions whereby the node can serve as
 *      seed for constructing a fresh list
 * \param   hNode is pointer to a arbitrary data object whose first
 *      member is of type PAL_OsListNodeHeader.
 * \warning This macro does NOT allocate memory for the new node.
 *      Caller must define the object elsewhere and only pass its
 *      pointer to the macro.
 */
#define PAL_OSLIST_MKNODE(hNode) do { \
    ((PAL_OsListNodeHeader*)hNode)->next = hNode; \
    ((PAL_OsListNodeHeader*)hNode)->prev = hNode; \
    } while (0)

/**
 * \brief   PAL OS List Append
 * 
 * This function "appends" the new node at tail end of list
 *
 * \param   hListHead is pointer to head of the list
 * \param   hNewNode is pointer to new element to be placed on the list
 * \return  nil return value
 */
PAL_INLINE void PAL_osListAppend(Ptr hListHead, Ptr hNewNode);

/**
 * \brief   PAL OS List Insert
 * 
 *      This function inserts the new node "after" the specified
 *      node on the list
 * \param   hListNode is pointer to node already on list "after" which
 *      new node is to be inserted
 * \param   hNewNode is pointer to new node to be placed on list
 * \return  nil return value
 */
PAL_INLINE void PAL_osListInsert(Ptr hListNode, Ptr hNewNode);

/**
 * \brief   PAL OS List Remove
 * 
 *      This function pulls out the specified node from the list.
 *      Rest of nodes will continue to remain normally on list
 * \param   hListNode is pointer of specific node to be removed
 * \warning No memory is freed up by this call
 * \return  Returns the pointer to the next node in the list.
 *          hListNode is returned if it is the only element in the list
 */
PAL_INLINE Ptr PAL_osListRemove(Ptr hListNode);

/**
 * \brief   PAL OS List Prepend
 * 
 *      This function prepends the specified node "ahead" of the
 *      current header node on the given list
 * \param   hListHead is pointer to head of the list
 * \param   hNewNode is pointer to new node to be placed at front
 *      of the current list
 * \return  Pointer to new head of list
 */
PAL_INLINE Ptr PAL_osListPrepend(Ptr hListHead, Ptr hNewNode);

/*\}*/
/*\}*/

#endif /* _PAL_OSLIST_H_ */
