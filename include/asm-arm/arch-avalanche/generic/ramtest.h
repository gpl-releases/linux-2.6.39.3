/*
 *
 * ramtest.h
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


/*
 * Generic RAM testing code 
 *
 */

#define AVALANCHE_RAMTEST_BASIC_DATA_FAIL   -1
#define AVALANCHE_RAMTEST_ADDRESS_FAIL      -2
#define AVALANCHE_RAMTEST_DATA_FAIL         -3

int avalanche_ram_basic_data_test(volatile unsigned int* base);
int avalanche_ram_address_test(volatile unsigned int* base, unsigned int size);
int avalanche_ram_data_test(volatile unsigned int* base, unsigned int size);
int avalanche_do_ram_test(volatile unsigned int* base, unsigned int size);

