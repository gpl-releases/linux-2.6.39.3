/*
 *
 * _tistdtypres.h
 * Description:
 * TI Standard defines for primitive "C" types only
 *
 * Copyright (C) 2009 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
*/

/** \file   _tistdtypes.h
    \brief  TI Standard defines for primitive "C" types only

    This file provides TI Standard defines for primitive "C" types only


    \author     PSP Architecture Team
    \version    1.1
 */

#ifndef _TI_STD_TYPES
#define _TI_STD_TYPES


/**
 * \defgroup TIBasicTypes TI Basic Types
 * 
 * All components - PAL, SRV, DDC, CSL shall use TI basic types 
 * to maintain compatibility of code with all systems. 
 * \n File _tistdtypes.h will adapt the types to the native compiler.
 */
/*@{*/

/**
 * \enum Bool - Boolean Type
 */
typedef enum 
{
    False = 0,
    True = 1
} Bool;

typedef int             Int;    /**< Signed base integer quanity */
typedef unsigned int    Uns;    /**< This is depricated support only */
typedef unsigned int    Uint;   /**< Unsigned base integer quantity */
typedef unsigned long   Ulong;  /**< Unsigned base long quantity */
typedef char            Char;   /**< Character quantity */
typedef char*           String; /**< Character pointer quantity */
typedef void*           Ptr;    /**< Arbitrary (void) pointer (works for pointing to any datum) */

/* Signed integer definitions (64bit, 32bit, 16bit, 8bit) follow... */ 
typedef long long       Int64; 
typedef int             Int32;
typedef short           Int16;
typedef signed char     Int8; 

/* Unsigned integer definitions (64bit, 32bit, 16bit, 8bit) follow... */ 
typedef unsigned long long  Uint64; 
typedef unsigned int        Uint32;
typedef unsigned short      Uint16;
typedef unsigned char       Uint8;

/*@}*/
#endif /* _TI_STD_TYPES */

