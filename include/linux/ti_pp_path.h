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

/***************************************************************************/

/*! \file ti_pp_path.h
    \brief Exports of PP Path counters

****************************************************************************/

#ifndef _TI_PP_PATH_H_
#define _TI_PP_PATH_H_

#ifndef _TI_PP_PATH_C
    #define EXTERN
#else
    #define EXTERN extern
#endif


/**************************************************************************/
/*      INCLUDES                                                          */
/**************************************************************************/

#include <asm-arm/arch-avalanche/generic/_tistdtypes.h>

/**************************************************************************/
/*      INTERFACE  Defines and Structs                                    */
/**************************************************************************/

/*! \var typedef enum PPP_pathDir_e
    \brief Defines the direction of a path
*/
typedef enum
{
    PPP_PATHDIR_IN,
    PPP_PATHDIR_OUT,

    PPP_PATHDIR_NUM
} PPP_pathDir_e;


/*! \var typedef struct PPP_uniCounetrs_t
    \brief Uni-directional counters
*/
typedef struct
{
    Uint64 octets;
    Uint64 packets;
} PPP_uniCounetrs_t;


/*! \var typedef struct PPP_counters_t
    \brief Path counters
*/
typedef struct
{
    PPP_uniCounetrs_t inCtrs;
    PPP_uniCounetrs_t outCtrs;
} PPP_counters_t;



/**************************************************************************/
/*      EXTERN definition block                                           */
/**************************************************************************/

/**************************************************************************/
/*      INTERFACE VARIABLES (prefix with EXTERN)                          */
/**************************************************************************/

/**************************************************************************/
/*      INTERFACE FUNCTIONS Prototypes:                                   */
/**************************************************************************/

#if defined(__KERNEL__)
/**************************************************************************/
/*! \fn int PPP_Init(void)
 **************************************************************************
 *  \brief Init the PPP module
 *  \return 0 (OK) / 1 (NOK)
 **************************************************************************/
int PPP_Init(void);

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
int PPP_AddPath(char *rxFrom, char *txTo, char *viaVirtualDev, PPP_pathDir_e pathDir);

/**************************************************************************/
/*! \fn int PPP_DelPath(char *rxFrom, char *txTo, char *viaVirtualDev)
 **************************************************************************
 *  \brief Delete a path
 *  \param[in] rxFrom - device used as IN
 *  \param[in] txTo - device used as OUT
 *  \param[in] viaVirtualDev - virtual device through which the traffic flows
 *  \return 0 (OK) / 1 (NOK)
 **************************************************************************/
int PPP_DelPath(char *rxFrom, char *txTo, char *viaVirtualDev);

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
int PPP_ReadCounters(char *viaVirtualDev, PPP_counters_t *pppCtrs);

#endif /*KERNEL*/
#endif

