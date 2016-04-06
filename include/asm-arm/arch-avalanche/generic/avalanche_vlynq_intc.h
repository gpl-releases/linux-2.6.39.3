/*
 *
 * avalanche_vlync_intc.h
 * Description:
 * vlynq intc hooks header
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

#ifndef _AVALANCHE_VLYNQ_INTC_H_
#define _AVALANCHE_VLYNQ_INTC_H_

#include <asm/irq.h>
#include <asm/arch/generic/pal_vlynq.h>
#include <asm/arch/generic/avalanche_intc.h>

int avalanche_vlynq_get_sys_irq_num(PAL_VLYNQ_HND *vlynq, int irq);

#define VLYNQINTNUM(vlynq, irq) avalanche_vlynq_get_sys_irq_num(vlynq, irq)

void avalanche_vlynq_disable_irq(unsigned int irq);
void avalanche_vlynq_enable_irq(unsigned int irq);

int avalanche_vlynq_request_irq(unsigned int irq, irqreturn_t (*handler)(int, void *, struct pt_regs *),
		                unsigned long irqflags, const char * devname, void *dev_id);
void avalanche_vlynq_free_irq(unsigned int irq, void *dev_id);

int avalanche_vlynq_set_irq_polarity(unsigned int irq, unsigned int pol);
int avalanche_vlynq_get_irq_polarity(unsigned int irq);

int avalanche_vlynq_set_irq_type(unsigned int irq, unsigned val);
int avalanche_vlynq_get_irq_type(unsigned int irq);

int avalanche_vlynq_irq_list(char *buf);

unsigned int avalanche_vlynq_get_irq_count(unsigned int);

#endif
