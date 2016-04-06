/*
 *
 * puma6_boards.h
 * Description:
 * board specific macros
 */


#ifndef _PUMA6_BOARDS_H
#define _PUMA6_BOARDS_H

/* Puma6 boards name */
#define PUMA6_FM_BOARD_NAME       "falconmine"
#define PUMA6_CI_BOARD_NAME       "catisland"
#define PUMA6_HP_BOARD_NAME       "harborpark"
#define PUMA6_HP_MG_BOARD_NAME    "harborpark-mg"
#define PUMA6_GS_BOARD_NAME       "goledn-springs"

/* Data definitions for board types */
#define PUMA6_UNKNOWN_BOARD_ID    (0x0) /* " ERROR "       */
#define PUMA6_HP_BOARD_ID         (0x1) /* "harborpark"    */
#define PUMA6_HP_MG_BOARD_ID      (0x2) /* "harborpark-mg" */
#define PUMA6_FM_BOARD_ID         (0x3) /* "falconmine"    */ 
#define PUMA6_CI_BOARD_ID         (0x4) /* "catisland"     */
#define PUMA6_GS_BOARD_ID         (0x5) /* "goledn-springs"     */


#endif /* _PUMA6_BOARDS_H */
