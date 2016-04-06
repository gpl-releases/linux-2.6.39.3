/**************************************************************************/
/*                                                                        */
/*   MODULE:                                                              */
/*   PURPOSE:                                                             */
/*                                                                        */
/**************************************************************************/
#ifndef _LZMA_DECODER_H_
#define _LZMA_DECODER_H_

int cm_hwDecodeLZMA(unsigned char *pbDest,unsigned int uiDecompressedLength, unsigned char *pbSource, unsigned int uiCompressedLength, void *);

#endif

