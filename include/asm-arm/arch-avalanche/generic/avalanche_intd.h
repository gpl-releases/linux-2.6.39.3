/*
 *
 * avalanche_intd.h 
 * Description:
 * interrupt distributor header file
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

#ifndef _AVALANCHE_INTD_H
#define _AVALANCHE_INTD_H

#define AVALANCHE_MAX_HOSTS_PER_SOC          ( 10 )
#define ALALANCHE_INTD_MAX_INTS_SUPPORTED    ( 1024 )

#define AVALANCHE_IDIST_REGS_BASE    AVALANCHE_INTD_BASE

#ifndef NUM_INTS_PER_REG
#define NUM_INTS_PER_REG   32
#endif



/* Interrupt Distributor relies on the below structure for SoC related 
 *information 
 */

/* NOTE:
 *      Do name the structure instances for the following structures as 
 * as SOC_HOST_INFO as "host_info" and AVALANCHE_SOC_INFO as "soc_info"
 * as these are been reffered by interrupt distributor code.
 */
struct avalanche_intd_host_info {
    unsigned int host_num;
    unsigned int max_ip_ints_mapped;
};
typedef struct avalanche_intd_host_info SOC_HOST_INFO;

struct avalanche_intd_soc_info {
    char *soc_name;
    unsigned int max_num_hosts;
    unsigned int total_ip_ints_mapped; 
    SOC_HOST_INFO host_info[AVALANCHE_MAX_HOSTS_PER_SOC];
};
typedef struct avalanche_intd_soc_info AVALANCHE_SOC_INFO;


/* NOTE: Read INTD as Interrupt Distributor */
struct avalanche_idist_pcer_regs        
{
  volatile unsigned int idpparamr; /* INTD Pacer Parameter Register */
  volatile unsigned int  idpdecr;       /* INTD Pacer decerement Register */
};  
typedef struct avalanche_idist_pcer_regs IDIST_PACER_REGS;
 
struct avalanche_idist_regs    /* Avalanche Interrupt Distributor registers */
{
    volatile unsigned int idrevr;        /* INTD Revision Reg */
    volatile unsigned int idcntrlr;      /* INTD Control Reg */
    volatile unsigned int unused1[2];    /* 0x08 */
    volatile unsigned int ideoir;        /* INTD End Of Interrupt (EOI) Reg */
    volatile unsigned int ideoiintvr;    /* INTD EOI Interrupt Vector Reg */
    volatile unsigned int unused2[2];    /* 0x18  to 0x1C */
    volatile unsigned int idpprer;       /* INTD Pacer Prescale  Reg */     
    volatile unsigned int unused4[3];    /* 0x24  to 0x2C */
    IDIST_PACER_REGS  idpacer[26];        /* INTD pacer Reg */
    volatile unsigned int idenbsr[32];   /* INTD Enabler  set Reg */
    volatile unsigned int idenbcr[32];   /* INTD Enabler Clear Reg */
    volatile unsigned int idstasr[32];   /* INTD Status  Set  Reg */
    volatile unsigned int idstacr[32];   /* INTD Status Clear  Reg */
    volatile unsigned int idcounterr[32];/* INTD Counter  Reg */
    volatile unsigned int idpolr[32];    /* INTD polarity Reg */
    volatile unsigned int idtypr[32];    /* INTD type Reg */
    volatile unsigned int idintvr[32];   /* INTD Interrupt Vector Reg */
};
 
int avalanche_intd_enable_interrupt( unsigned int host_num,
                                     unsigned int ip_int_num );

int avalanche_intd_disable_interrupt( unsigned int host_num,
                                      unsigned int ip_int_num);

int avalanche_intd_set_interrupt_status( unsigned int host_num,
                                         unsigned int ip_int_num ); 

int avalanche_intd_clear_interrupt_status( unsigned int host_num,
                                           unsigned int ip_int_num ); 

int avalanche_intd_is_status_set( unsigned int host_num,
                                  unsigned int ip_int_num );

int avalanche_intd_is_status_cleared( unsigned int host_num, 
                                      unsigned int ip_int_num );

int avalanche_intd_read_eoi( void );

int avalanche_intd_write_eoi( unsigned int vect_val );

int avalanche_intd_get_interrupt_count( unsigned int host_num,
                                        unsigned int ip_int_num );

int avalanche_intd_set_interrupt_count( unsigned int host_num,
                                        unsigned int ip_int_num,
                                        unsigned int count_val );
#endif /* _AVALANCHE_INTD_H */
