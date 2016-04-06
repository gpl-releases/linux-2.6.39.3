/***************************************************************************
7-Zip Copyright (C) 1999-2002 Igor Pavlov.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

***************************************************************************/

/* Copyright 2008, Texas Instruments Incorporated
 *
 * This program has been modified from its original operation by 
 * Texas Instruments to do the following:
 * 
 * Explanation of modification:
 * Decoder.h : Standalone lzma decompressor header, a derivative work of
 * 7-Zip 2.30 Beta 24. 
 *  
 *
 * THIS MODIFIED SOFTWARE AND DOCUMENTATION ARE PROVIDED
 * "AS IS," AND TEXAS INSTRUMENTS MAKES NO REPRESENTATIONS
 * OR WARRENTIES, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO, WARRANTIES OF MERCHANTABILITY OR FITNESS FOR ANY
 * PARTICULAR PURPOSE OR THAT THE USE OF THE SOFTWARE OR
 * DOCUMENTATION WILL NOT INFRINGE ANY THIRD PARTY PATENTS,
 * COPYRIGHTS, TRADEMARKS OR OTHER RIGHTS.
 * See The GNU General Public License for more details.
 *
 * These changes are covered under version 2 of the GNU General Public License,
 * dated June 1991.
 */

#ifndef __LZARITHMETIC_DECODER_H
#define __LZARITHMETIC_DECODER_H

/*************************	Types 	******************************/

typedef unsigned char   UINT8;
typedef unsigned short  UINT16;
typedef short 		INT16;
typedef unsigned int 	UINT32;
typedef UINT8 		BYTE;

#define NUM_TOP_BITS    24
UINT32 kTopValue;

/*************************	RangeCoder 	******************************/

typedef struct {
    UINT32 Range;
    UINT32 Code;
    UINT8  *local_ptr;	  
} CRangeDecoder;


void CRangeDecoder_Normalize(CRangeDecoder *);
void CRangeDecoder_Init(CRangeDecoder *, unsigned char *stream);
void CRangeDecoder_ReleaseStream(CRangeDecoder *);
UINT32 CRangeDecoder_GetThreshold(CRangeDecoder *, UINT32 total);
void CRangeDecoder_Decode(CRangeDecoder *, UINT32 start, UINT32 size, UINT32 total);
UINT32 CRangeDecoder_DecodeDirectBits(CRangeDecoder *, UINT32 numTotalBits);
UINT32 CRangeDecoder_DecodeBit(CRangeDecoder *,UINT32 size0, UINT32 numTotalBits);
UINT32 CRangeDecoder_GetProcessedSize(CRangeDecoder *);


/*************************	AriBitCoder 	******************************/

#define NUM_BIT_MODEL_TOTAL_BITS    11
UINT32 kBitModelTotal;
#define NUM_MOVE_REDUCING_BITS      2
#define NUM_BIT_PRICE_SHIFT_BITS    6
UINT32 kBitPrice;

typedef struct {
  UINT32 Probability;
  int aNumMoveBits;
} CMyBitDecoder;


inline void init_AriBitCoder_Variables(void)
{
  kBitModelTotal = (1 << NUM_BIT_MODEL_TOTAL_BITS);
  kBitPrice = 1 << NUM_BIT_PRICE_SHIFT_BITS;
}

void CMyBitDecoder_Init(CMyBitDecoder *);
UINT32 CMyBitDecoder_Decode(CMyBitDecoder *, CRangeDecoder *rangeDecoder);

typedef CRangeDecoder CMyRangeDecoder;


/*************************	RCDefs 	******************************/

#define RC_INIT_VAR                            \
  UINT32 range = rangeDecoder->Range;      \
  UINT32 code = rangeDecoder->Code;        

#define RC_FLUSH_VAR                          \
  rangeDecoder->Range = range;            \
  rangeDecoder->Code = code;

#define RC_NORMALIZE                                    \
    if (range < kTopValue)               \
    {                                                              \
      code = (code << 8) | *(rangeDecoder->local_ptr++);   \
      range <<= 8; }

#define RC_GETBIT2(aNumMoveBits, aProb, aModelIndex, Action0, Action1)                        \
    {UINT32 aNewBound = (range >> NUM_BIT_MODEL_TOTAL_BITS) * aProb; \
    if (code < aNewBound)                               \
    {                                                             \
      Action0;                                                    \
      range = aNewBound;                                         \
      aProb += (kBitModelTotal - aProb) >> aNumMoveBits;          \
      aModelIndex <<= 1;                                          \
    }                                                             \
    else                                                          \
    {                                                             \
      Action1;                                                    \
      range -= aNewBound;                                        \
      code -= aNewBound;                                          \
      aProb -= (aProb) >> aNumMoveBits;                           \
      aModelIndex = (aModelIndex << 1) + 1;                       \
    }}                                                             \
    RC_NORMALIZE

#define RC_GETBIT(aNumMoveBits, aProb, aModelIndex) RC_GETBIT2(aNumMoveBits, aProb, aModelIndex, ; , ;)               


/*************************	BitTreeCoder 	******************************/


typedef struct {
  int 		numMoveBits;
  UINT32 	NumBitLevels;
  CMyBitDecoder *Models;    
} CBitTreeDecoder;


void CBitTreeDecoder_Init(CBitTreeDecoder *);
UINT32 CBitTreeDecoder_Decode(CBitTreeDecoder *, CMyRangeDecoder *);

typedef struct {
  int numMoveBits;
  UINT32 NumBitLevels;
  CMyBitDecoder *Models;
} CReverseBitTreeDecoder2;

UINT8 CReverseBitTreeDecoder2_Create(CReverseBitTreeDecoder2 *, UINT32);
void CReverseBitTreeDecoder2_Init(CReverseBitTreeDecoder2 *);
UINT32 CReverseBitTreeDecoder2_Decode(CReverseBitTreeDecoder2 *, CMyRangeDecoder *);

typedef struct {
  int 		numMoveBits;
  UINT32 	NumBitLevels;
  CMyBitDecoder *Models;
} CReverseBitTreeDecoder;


void CReverseBitTreeDecoder_Init(CReverseBitTreeDecoder *);
UINT32 CReverseBitTreeDecoder_Decode(CReverseBitTreeDecoder *, CMyRangeDecoder *);


/*************************	LenCoder 	******************************/


#define NUM_MOVE_BITS                       5
#define NUM_POS_STATES_BITS_MAX             4
UINT32 kNumPosStatesMax;
#define NUM_POS_STATES_BITS_ENCODING_MAX    4
UINT32 kNumPosStatesEncodingMax;
#define NUM_LEN_BITS                        3
UINT32 kNumLowSymbols;
#define NUM_MID_BITS                        3
UINT32 kNumMidSymbols;
#define NUM_HIGH_BITS                       8
UINT32 kNumSymbolsTotal;


typedef struct {
  CMyBitDecoder 	_choice;
  CBitTreeDecoder 	_lowCoder[1 << 4];
  CMyBitDecoder		_choice2;
  CBitTreeDecoder	_midCoder[1 << 4];
  CBitTreeDecoder	_highCoder;
  UINT32		_numPosStates;
} CDecoderLength;


inline void init_LenCoder_Variables(void)
{
  kNumPosStatesMax = (1 << NUM_POS_STATES_BITS_MAX);
  kNumPosStatesEncodingMax = (1 << NUM_POS_STATES_BITS_ENCODING_MAX);
  kNumLowSymbols = 1 << NUM_LEN_BITS;
  kNumMidSymbols = 1 << NUM_MID_BITS;
  kNumSymbolsTotal = kNumLowSymbols + kNumMidSymbols + (1 << NUM_HIGH_BITS);
}


void CDecoderLength_Initialize(CDecoderLength *);
void CDecoderLength_Create(CDecoderLength *, UINT32 );
void CDecoderLength_Init(CDecoderLength *);
UINT32 CDecoderLength_Decode(CDecoderLength *, CMyRangeDecoder *rangeDecoder, UINT32 posState);



/*************************	LiterCoder 	******************************/


typedef struct {
  CMyBitDecoder _decoders[3][1 << 8];
} CDecoder2;


void CDecoder2_Init(CDecoder2 *);
BYTE CDecoder2_DecodeNormal(CDecoder2 *,CMyRangeDecoder *);
BYTE CDecoder2_DecodeWithMatchByte(CDecoder2 *,CMyRangeDecoder *, BYTE );


typedef struct {
  CDecoder2 	*_coders;
  UINT32	_numPrevBits;
  UINT32	_numPosBits;
  UINT32	_posMask;	
} CDecoderLiteral;


void CDecoderLiteral_Create(CDecoderLiteral *,UINT32 , UINT32 );
void CDecoderLiteral_Init(CDecoderLiteral *);
UINT32 CDecoderLiteral_GetState(CDecoderLiteral *,UINT32 , BYTE );
BYTE CDecoderLiteral_DecodeNormal(CDecoderLiteral *,CMyRangeDecoder *, UINT32 , BYTE );
BYTE CDecoderLiteral_DecodeWithMatchByte(CDecoderLiteral *,CMyRangeDecoder *, UINT32 , BYTE , BYTE );



/*************************	LZMA 	******************************/


const UINT32 kNumRepDistances = 4;

const BYTE kNumStates = 12;

const BYTE kLiteralNextStates[12] = {0, 0, 0, 0, 1, 2, 3, 4,  5,  6,   4, 5};
const BYTE kMatchNextStates[12]   = {7, 7, 7, 7, 7, 7, 7, 10, 10, 10, 10, 10};
const BYTE kRepNextStates[12]     = {8, 8, 8, 8, 8, 8, 8, 11, 11, 11, 11, 11};
const BYTE kShortRepNextStates[12]= {9, 9, 9, 9, 9, 9, 9, 11, 11, 11, 11, 11};
const int kNumPosSlotBits = 6; 
const int kDicLogSizeMin = 0; 
const int kDicLogSizeMax = 32; 
int kDistTableSizeMax; 

const UINT32 kNumLenToPosStates = 4;
inline UINT32 GetLenToPosState(UINT32 len)
{
  len -= 2;
  if (len < kNumLenToPosStates)
    return len;
  return kNumLenToPosStates - 1;
}

const int kMatchMinLen = 2;

int intkMatchMaxLen;

const int kNumAlignBits = 4;
int kAlignTableSize;
UINT32 kAlignMask;

const int kStartPosModelIndex = 4;
const int kEndPosModelIndex = 14;
int kNumPosModels;

int kNumFullDistances;

const int kMainChoiceLiteralIndex = 0;
const int kMainChoiceMatchIndex = 1;

const int kMatchChoiceDistanceIndex= 0;
const int kMatchChoiceRepetitionIndex = 1;

const int kNumMoveBitsForMainChoice = 5;
const int kNumMoveBitsForPosCoders = 5;

const int kNumMoveBitsForAlignCoders = 5;

const int kNumMoveBitsForPosSlotCoder = 5;

const int kNumLitPosStatesBitsEncodingMax = 4;
const int kNumLitContextBitsMax = 8;


typedef struct {
  BYTE Index;
} CState;

void CState_Init(CState *);
void UpdateChar(CState *);
void UpdateMatch(CState *);
void UpdateRep(CState *);
void UpdateShortRep(CState *);


typedef struct {
  CState _state;
  BYTE _previousByte;
  UINT32 _repDistances[4];
} CBaseCoder;


inline void init_LZMA_Variables(void)
{
  kDistTableSizeMax = kDicLogSizeMax * 2; 
  intkMatchMaxLen = kMatchMinLen + kNumSymbolsTotal - 1;
  kAlignTableSize = 1 << kNumAlignBits;
  kAlignMask = (kAlignTableSize - 1);
  kNumPosModels = kEndPosModelIndex - kStartPosModelIndex;
  kNumFullDistances = 1 << (kEndPosModelIndex / 2);
}


void CBaseCoder_Init(CBaseCoder *);


/*************************	Decoder 	******************************/


typedef struct {
   unsigned int _pos;
   unsigned char *_buffer;
} RareOutputStream;


void RareOutputStream_OutStreamInit(RareOutputStream *routStream, unsigned char *ptr);
void RareOutputStream_PutOneByte(RareOutputStream *routStream, BYTE b);
unsigned char RareOutputStream_GetOneByte(RareOutputStream *routStream, unsigned int index);
void RareOutputStream_CopyBackBlock(RareOutputStream *routStream, unsigned int distance, unsigned int len);


typedef struct {
  RareOutputStream 	_outWindowStream;
  CMyRangeDecoder  	_rangeDecoder;
  
  CMyBitDecoder 	_mainChoiceDecoders[12][1 << 4];
  CMyBitDecoder 	_matchChoiceDecoders[12];
  CMyBitDecoder 	_matchRepChoiceDecoders[12];
  CMyBitDecoder 	_matchRep1ChoiceDecoders[12];
  CMyBitDecoder 	_matchRep2ChoiceDecoders[12];
  CMyBitDecoder 	_matchRepShortChoiceDecoders[12][1 << 4];
  CBitTreeDecoder	_posSlotDecoder[4];
  CReverseBitTreeDecoder2	_posDecoders[10];

  CReverseBitTreeDecoder	_posAlignDecoder;
  
  CDecoderLength 	_lenDecoder;
  CDecoderLength 	_repMatchLenDecoder;

  CDecoderLiteral 	_literalDecoder;

  UINT32 		_dictionarySize;
  UINT32 		_dictionarySizeCheck;
  UINT32 		_posStateMask;
} CDecoder;


void CDecoder_CDecoder(CDecoder *decoder);
int Create(CDecoder *decoder);
int Init(CDecoder *decoder, unsigned char *InPtr, unsigned char *OutPtr);
int CodeReal(CDecoder *decoder, unsigned char *InPtr,unsigned char *OutPtr, const UINT32 *inSize, const UINT32 *outSize,unsigned int *progress);
int SetDictionarySize(CDecoder *decoder, UINT32 dictionarySize);
int SetLiteralProperties(CDecoder *decoder, UINT32 numLiteralPosStateBits, UINT32 numLiteralContextBits);
int SetPosBitsProperties(CDecoder *decoder, UINT32 numPosStateBits);


#endif

