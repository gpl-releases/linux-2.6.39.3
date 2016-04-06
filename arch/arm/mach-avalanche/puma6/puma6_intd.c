/*
 *
 * puma6_intd.c 
 * Description:
 * see below
 *
 *
 */


/** \file   puma6_intd.c
    \brief  puma6 SoC related info required by alalanche
            interrupt distributor.

    \author     PSP TII
    \version    0.1
 */
#include <pal.h>

#define AVALANCHE_SOC_NAME         "PUMA6"   
#define AVALANCHE_SOC_MAX_HOSTS    (2)
#define ARM11_MAX_IP_INTS_MAPPED   (32)
#define C55x_MAX_IP_INTS_MAPPED    (00)
#define AVALANCHE_SOC_TOTAL_IP_INTS_MAPPED  ( ARM11_MAX_IP_INTS_MAPPED +\
                                              C55x_MAX_IP_INTS_MAPPED )

/* NOTE: In the below enum populate  the hosts 
 * alphabetically for  proper interrupt  mappings 
 */
enum puma6_hosts {ARM11, C55X };

/*
 *Please name the structure as soc_info so that generic distributor layer works. 
 */

AVALANCHE_SOC_INFO soc_info = {
	AVALANCHE_SOC_NAME,
	AVALANCHE_SOC_MAX_HOSTS,
	AVALANCHE_SOC_TOTAL_IP_INTS_MAPPED,
        {
        {ARM11,ARM11_MAX_IP_INTS_MAPPED },
        {C55X, C55x_MAX_IP_INTS_MAPPED }
        }
       };

