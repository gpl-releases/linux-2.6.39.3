/*
 * keypad.h
 * Description:
 * See below.
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
 */


/*******************************************************************************   
 * FILE PURPOSE:    Keypad Module Header file                                      
 *******************************************************************************   
 * FILE NAME:       keypad.h                                                   
 *                                                                                 
 * DESCRIPTION:     Header file for Linux Keypad Driver                             
 *                                                                                 
 * REVISION HISTORY:  
 *   
 * Date           Description                               Author
 *-----------------------------------------------------------------------------
 * 27 Aug 2003    Initial Creation                          Sharath Kumar  
 * 
 * 16 Dec 2003    Updates for 5.7                           Sharath Kumar                                                          
 *                                                                                 
 ******************************************************************************/   
#ifndef _TI_KEYPAD_H_
#define _TI_KEYPAD_H_


typedef struct key_event {
    int row;  /* Stores the row  number of the key   */
    int column;  /* Stores the column number of the key */ 
    int key_press_time;  /* This is a flag to indicate whether the. 
                          * event was key-press or key-release event.
                          * For key-press event this will contain 0.
                          * For key-release event this will contain the
                          * duration(in milisec) for which the last key was pressed.
                          */
}KEY_EVENT;


/* Defines for IOCTLs */
#define TI_KEY_DEBOUNCE_VALUE    1


/* KEYPAD config parameters */
#ifdef CONFIG_MIPS_TNETV1050SDB
#define COLUMN_MAP             0x0F00                   
#define ROW_MAP                0x003F                   
#endif
#ifdef CONFIG_MIPS_TNETV1050RDB
#define COLUMN_MAP             0x00FF                   
#define ROW_MAP                0x0F00                  
#endif
#define DEBOUNCE_TIME             (20) /* in milsecs */    
#define BUFFER_SIZE               (256)                    


#endif /* _TI_KEYPAD_H_ */
