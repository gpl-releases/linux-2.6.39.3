/*
 * lcd.h
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
 * FILE PURPOSE:    LCD Module Driver Header file                                     
 *******************************************************************************   
 * FILE NAME:       lcd.h                                                   
 *                                                                                 
 * DESCRIPTION:     Header file for Linux LCD Driver                             
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
#ifndef _TI_LCD_H_
#define _TI_LCD_H_

typedef struct lcd_pos {
	int row;
	int column;
} LCD_POS;

typedef struct lcd_pulse_arg {
	unsigned char is_up;
	unsigned char cnt;
} LCD_PULSE_ARG;

/* LCD configuration params  */
#define    MAX_ROWS                          (4)  
#define    MAX_COLS                         (40)  
#define    DEFAULT_ROWS                      (2)  
#define    DEFAULT_COLS                     (24)  
#define    NO_LCD_DEVICES                    (1)  

/* Defines for IOCTLs */
#define TI_LIDD_CLEAR_SCREEN   1               
#define TI_LIDD_CURSOR_HOME    2               
#define TI_LIDD_GOTO_XY        3               
#define TI_LIDD_DISPLAY        4               
#define TI_LIDD_BLINK          5               
#define TI_LIDD_CURSOR_STATE   6               
#define TI_LIDD_DISPLAY_SHIFT  7               
#define TI_LIDD_CURSOR_SHIFT   8               
#define TI_LIDD_CURSOR_MOVE    9               
#define TI_LIDD_DISPLAY_MOVE   10              
#define TI_LIDD_WR_CHAR        11              
#define TI_LIDD_RD_CHAR        12              
#define TI_LIDD_LINE_WRAP      13              
#define TI_LIDD_RD_CMD         14              
#define TI_LIDD_PULSE_CMD      15
                                               
#define RIGHT                  1               
#define LEFT                   0               
#define ON                     1               
#define OFF                    0               

#endif /* _TI_LCD_H_ */
