/*
 * listlib.h
 *
 * 	Contains structures and exported function that are used by the linked
 * 	list library.
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

#ifndef __LISTLIB_H__
#define __LISTLIB_H__

/**************************************************************************
 * STRUCTURE -  LIST_NODE
 **************************************************************************
 *	The structure defines a LIST NODE structure that contains links to the
 *	previous and next element in the list.
 **************************************************************************/
typedef struct LIST_NODE
{
	void*	p_next;		/* Pointer to the next element in the list. 	*/
    void*   p_prev;     /* Pointer to the prev element in the list. */
}LIST_NODE;

/************************ EXTERN Functions *********************************/

extern void list_add (LIST_NODE **ptr_list, LIST_NODE *ptr_node);
extern LIST_NODE* list_remove (LIST_NODE **ptr_list);
extern LIST_NODE* list_get_head (LIST_NODE **ptr_list);
extern LIST_NODE* list_get_next (LIST_NODE *ptr_list);
extern int list_remove_node (LIST_NODE **ptr_list, LIST_NODE *ptr_remove);
extern void list_cat (LIST_NODE **ptr_dst, LIST_NODE **ptr_src);

#endif	/* __LISTLIB_H__ */



