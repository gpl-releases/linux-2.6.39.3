/*
 * listlib.c
 *
 * 	Implementation of a doubly linked list.
 *
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

#include "listlib.h"

/**************************************************************************
 *************************** Local Definitions ****************************
 **************************************************************************/

/* Definition for NULL */
#define NULL (void *)0

/**************************************************************************
 * FUNCTION NAME : list_add
 **************************************************************************
 * DESCRIPTION   :
 * 	The function is called to add a node 'ptr_node' to the 'ptr_list'.
 ***************************************************************************/
void list_add (LIST_NODE **ptr_list, LIST_NODE *ptr_node)
{
	LIST_NODE*	ptr_head;

	/* Check if the list is empty ? */
	if (*ptr_list == NULL)
	{
		/* YES the list is empty. Initialize the links */
		ptr_node->p_next = NULL;
        ptr_node->p_prev = NULL;

		/* Initialize the LIST */
		*ptr_list = ptr_node;
		return;
	}

	/* No the list was NOT empty. Add the node to the beginning of list.
     * Get the current head of the list. */
	ptr_head = *ptr_list;

	/* Initialize the new head of the list. */
	ptr_node->p_next  = ptr_head;
    ptr_node->p_prev = NULL;

    /* Update the old head to point to the new head */
    ptr_head->p_prev = ptr_node;

    /* Update the pointer to the head of the list. */
	*ptr_list = ptr_node;
	return;
}

/**************************************************************************
 * FUNCTION NAME : list_cat
 **************************************************************************
 * DESCRIPTION   :
 * 	The function is called to concatenate the src list to the end of the
 *  destination list.
 ***************************************************************************/
void list_cat (LIST_NODE **ptr_dst, LIST_NODE **ptr_src)
{
	LIST_NODE*	ptr_node;
	LIST_NODE*	ptr_prev;

	/* Is the source list empty ? */
	if (*ptr_src == NULL)
		return;

	/* Is the destination list empty ? */
	if (*ptr_dst == NULL)
	{
		/* Make the source now as the destination. */
		*ptr_dst = *ptr_src;
		return;
	}

	/* Both the lists are not empty. */
	ptr_node = *ptr_dst;
	ptr_prev = NULL;

	/* Reach the end of the list. */
	while (ptr_node != NULL)
	{
		ptr_prev = ptr_node;
		ptr_node = ptr_node->p_next;
	}

	/* Link the last element to the source list. */
	ptr_prev->p_next = *ptr_src;
    (*ptr_src)->p_prev = ptr_prev;
	return;
}

/**************************************************************************
 * FUNCTION NAME : list_remove
 **************************************************************************
 * DESCRIPTION   :
 * 	The function is called to remove the head node from the list.
 *
 * RETURNS		 :
 * 		NULL  - If there are no elements in the list.
 * 		Pointer to the head of the list
 ***************************************************************************/
LIST_NODE* list_remove (LIST_NODE **ptr_list)
{
	LIST_NODE*	ptr_head;
	LIST_NODE*	ptr_node;

	/* Check if the list is empty ? */
	if (*ptr_list == NULL)
		return NULL;

	/* Get the head of the list. */
	ptr_node = *ptr_list;

	/* Move the head to the next element in the list. */
	ptr_head = ptr_node->p_next;
	*ptr_list = ptr_head;

    /* Did we remove the last element?*/
    if (ptr_head != NULL)
    {
        /* No; in that case update the pointers for the new head. */
        ptr_head->p_prev = NULL;
    }

	/* Kill the links before returning the OLD head. */
	ptr_node->p_next = NULL;
    ptr_node->p_prev = NULL;
	return ptr_node;
}

/**************************************************************************
 * FUNCTION NAME : list_remove_node
 **************************************************************************
 * DESCRIPTION   :
 * 	The function is called to the specified 'node' from the list.
 ***************************************************************************/
int list_remove_node (LIST_NODE **ptr_list, LIST_NODE *ptr_remove)
{
	LIST_NODE*	ptr_next;
	LIST_NODE*	ptr_prev;

    /* Are there any nodes in the list? */
    if (*ptr_list == NULL)
		return -1;

    /* Are we removing the head? */
    if (ptr_remove == *ptr_list)
    {
        /* Use the other API to acheive the needful. */
        list_remove (ptr_list);
        return 0;
    }

    /* OK; we are trying to remove a non head element; so lets get the
     * previous and next pointer of the elements that needs to be removed. */
    ptr_prev = ptr_remove->p_prev;
    ptr_next = ptr_remove->p_next;

    /* Kill the Links for element that is being removed. */
    ptr_remove->p_prev = NULL;
    ptr_remove->p_next = NULL;

    /* Are we removing the last element */
    if (ptr_next == NULL)
    {
        /* The last element points to nothing. */
        ptr_prev->p_next = NULL;
        return 0;
    }

    /* We are trying to remove an element in the middle of the list. */
	ptr_prev->p_next = ptr_next;
    ptr_next->p_prev = ptr_prev;

	/* Successful. */
	return 0;
}

/**************************************************************************
 * FUNCTION NAME : list_get_head
 **************************************************************************
 * DESCRIPTION   :
 *	The function is used to get the head of the specific list
 *
 * RETURNS		 :
 *	NULL		- If the list is empty
 *  Not NULL	- Pointer to the head of the list.
 ***************************************************************************/
LIST_NODE* list_get_head (LIST_NODE **ptr_list)
{
	return *ptr_list;
}

/**************************************************************************
 * FUNCTION NAME : list_get_next
 **************************************************************************
 * DESCRIPTION   :
 *	The function is used to traverse the specific list.
 *
 * RETURNS		 :
 *	NULL		- If there is no next element.
 *  Not NULL	- Pointer to the next element.
 ***************************************************************************/
LIST_NODE* list_get_next (LIST_NODE *ptr_list)
{
	return ptr_list->p_next;
}


