/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2017, ITU/ISO/IEC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  * Neither the name of the ITU/ISO/IEC nor the names of its contributors may
 *    be used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

/** \file     AdaptiveLoopFilterX86.h
    \brief    adaptive loop filter class
*/
#include "CommonDefX86.h"
#include "../AdaptiveLoopFilter.h"

//! \ingroup CommonLib
//! \{

#ifdef TARGET_SIMD_X86
#if defined _MSC_VER
#include <tmmintrin.h>
#else
#include <immintrin.h>
#endif

template<X86_VEXT vext>
static void simdDeriveClassificationBlk(AlfClassifier** classifier, int** laplacian[NUM_DIRECTIONS], const CPelBuf& srcLuma, const Area& blkDst, const Area& blk, const int shift, int vbCTUHeight, int vbPos)
{
  const int img_stride = srcLuma.stride;
  const Pel* srcExt = srcLuma.buf;

  const int fl = 2;
  const int flplusOne = fl + 1;
  const int fl2plusTwo = 2 * fl + 2;
  const int var_max = 15;

  const int imgHExtended = blk.height + fl2plusTwo;
  const int imgWExtended = blk.width + fl2plusTwo;

  const int posX = blk.pos().x;
  const int posY = blk.pos().y;
  const int start_height1 = posY - flplusOne;

  static uint16_t _temp[( AdaptiveLoopFilter::m_CLASSIFICATION_BLK_SIZE + 4 ) >> 1][AdaptiveLoopFilter::m_CLASSIFICATION_BLK_SIZE + 4];

  for( int i = 0; i < imgHExtended - 2; i += 2 )
  {
    int yoffset = ( i + 1 + start_height1 ) * img_stride - flplusOne;

    const Pel *p_imgY_pad_down = &srcExt[yoffset - img_stride];
    const Pel *p_imgY_pad = &srcExt[yoffset];
    const Pel *p_imgY_pad_up = &srcExt[yoffset + img_stride];
    const Pel *p_imgY_pad_up2 = &srcExt[yoffset + img_stride * 2];

    // pixel padding for gradient calculation
    if (((blkDst.pos().y - 2 + i) > 0) && ((blkDst.pos().y - 2 + i) % vbCTUHeight) == (vbPos - 2))
    {
      p_imgY_pad_up2 = &srcExt[yoffset + img_stride];
    }
    else if (((blkDst.pos().y - 2 + i) > 0) && ((blkDst.pos().y - 2 + i) % vbCTUHeight) == vbPos)
    {
      p_imgY_pad_down = &srcExt[yoffset];
    }

    __m128i mmStore = _mm_setzero_si128();

    for( int j = 2; j < imgWExtended; j += 8 )
    {
      const int pixY = j - 1 + posX;

      const __m128i* pY = ( __m128i* )( p_imgY_pad + pixY - 1 );
      const __m128i* pYdown = ( __m128i* )( p_imgY_pad_down + pixY - 1 );
      const __m128i* pYup = ( __m128i* )( p_imgY_pad_up + pixY - 1 );
      const __m128i* pYup2 = ( __m128i* )( p_imgY_pad_up2 + pixY - 1 );

      const __m128i* pY_next = ( __m128i* )( p_imgY_pad + pixY + 7 );
      const __m128i* pYdown_next = ( __m128i* )( p_imgY_pad_down + pixY + 7 );
      const __m128i* pYup_next = ( __m128i* )( p_imgY_pad_up + pixY + 7 );
      const __m128i* pYup2_next = ( __m128i* )( p_imgY_pad_up2 + pixY + 7 );

      __m128i xmm0 = _mm_loadu_si128( pYdown );
      __m128i xmm1 = _mm_loadu_si128( pY );
      __m128i xmm2 = _mm_loadu_si128( pYup );
      __m128i xmm3 = _mm_loadu_si128( pYup2 );

      const __m128i xmm0_next = _mm_loadu_si128( pYdown_next );
      const __m128i xmm1_next = _mm_loadu_si128( pY_next );
      const __m128i xmm2_next = _mm_loadu_si128( pYup_next );
      const __m128i xmm3_next = _mm_loadu_si128( pYup2_next );

      __m128i xmm4 = _mm_slli_epi16( _mm_alignr_epi8( xmm1_next, xmm1, 2 ), 1 );
      __m128i xmm5 = _mm_slli_epi16( _mm_alignr_epi8( xmm2_next, xmm2, 2 ), 1 );

      __m128i xmm15 = _mm_setzero_si128();

      //dig0
      __m128i xmm6 = _mm_add_epi16( _mm_alignr_epi8( xmm2_next, xmm2, 4 ), xmm0 );
      xmm6 = _mm_sub_epi16( _mm_blend_epi16 ( xmm4, xmm15, 0xAA ), _mm_blend_epi16 ( xmm6, xmm15, 0xAA ) );
      __m128i xmm8 = _mm_add_epi16( _mm_alignr_epi8( xmm3_next, xmm3, 4 ), xmm1 );
      xmm8 = _mm_sub_epi16( _mm_blend_epi16 ( xmm5, xmm15, 0x55 ), _mm_blend_epi16 ( xmm8, xmm15, 0x55 ) );

      //dig1
      __m128i xmm9 = _mm_add_epi16( _mm_alignr_epi8( xmm0_next, xmm0, 4 ), xmm2 );
      xmm9 = _mm_sub_epi16( _mm_blend_epi16 ( xmm4, xmm15, 0xAA ), _mm_blend_epi16 ( xmm9, xmm15, 0xAA ) );
      __m128i xmm10 = _mm_add_epi16( _mm_alignr_epi8( xmm1_next, xmm1, 4 ), xmm3 );
      xmm10 = _mm_sub_epi16( _mm_blend_epi16 ( xmm5, xmm15, 0x55 ), _mm_blend_epi16 ( xmm10, xmm15, 0x55 ) );

      //hor
      __m128i xmm13 = _mm_add_epi16( _mm_alignr_epi8( xmm1_next, xmm1, 4 ), xmm1 );
      xmm13 = _mm_sub_epi16( _mm_blend_epi16 ( xmm4, xmm15, 0xAA ), _mm_blend_epi16 ( xmm13, xmm15, 0xAA ) );
      __m128i xmm14 = _mm_add_epi16( _mm_alignr_epi8( xmm2_next, xmm2, 4 ), xmm2 );
      xmm14 = _mm_sub_epi16( _mm_blend_epi16 ( xmm5, xmm15, 0x55 ), _mm_blend_epi16 ( xmm14, xmm15, 0x55 ) );

      //ver
      __m128i xmm11 = _mm_add_epi16( _mm_alignr_epi8( xmm0_next, xmm0, 2 ), _mm_alignr_epi8( xmm2_next, xmm2, 2 ) );
      xmm11 = _mm_sub_epi16( _mm_blend_epi16 ( xmm4, xmm15, 0xAA ), _mm_blend_epi16 ( xmm11, xmm15, 0xAA ) );
      __m128i xmm12 = _mm_add_epi16( _mm_alignr_epi8( xmm1_next, xmm1, 2 ), _mm_alignr_epi8( xmm3_next, xmm3, 2 ) );
      xmm12 = _mm_sub_epi16( _mm_blend_epi16 ( xmm5, xmm15, 0x55 ), _mm_blend_epi16 ( xmm12, xmm15, 0x55 ) );

      xmm6 = _mm_abs_epi16( xmm6 );
      xmm8 = _mm_abs_epi16( xmm8 );
      xmm9 = _mm_abs_epi16( xmm9 );
      xmm10 = _mm_abs_epi16( xmm10 );
      xmm11 = _mm_abs_epi16( xmm11 );
      xmm12 = _mm_abs_epi16( xmm12 );
      xmm13 = _mm_abs_epi16( xmm13 );
      xmm14 = _mm_abs_epi16( xmm14 );

      xmm6 = _mm_add_epi16( xmm6, xmm8 );
      xmm9 = _mm_add_epi16( xmm9, xmm10 );
      xmm11 = _mm_add_epi16( xmm11, xmm12 );
      xmm13 = _mm_add_epi16( xmm13, xmm14 );

      xmm6 = _mm_add_epi16( xmm6, _mm_srli_si128( xmm6, 2 ) );
      xmm9 = _mm_add_epi16( xmm9, _mm_slli_si128( xmm9, 2 ) );
      xmm11 = _mm_add_epi16( xmm11, _mm_srli_si128( xmm11, 2 ) );
      xmm13 = _mm_add_epi16( xmm13, _mm_slli_si128( xmm13, 2 ) );

      xmm6 = _mm_blend_epi16( xmm6, xmm9, 0xAA );
      xmm11 = _mm_blend_epi16( xmm11, xmm13, 0xAA );

      xmm6 = _mm_add_epi16( xmm6, _mm_slli_si128( xmm6, 4 ) );
      xmm11 = _mm_add_epi16( xmm11, _mm_srli_si128( xmm11, 4 ) );

      xmm6 = _mm_blend_epi16( xmm11, xmm6, 0xCC );

      xmm9 = _mm_srli_si128( xmm6, 8 );

      if( j > 2 )
      {
        _mm_storel_epi64( ( __m128i* )( &( _temp[i >> 1][j - 2 - 4] ) ), _mm_add_epi16( xmm6, mmStore ) );
      }

      xmm6 = _mm_add_epi16( xmm6, xmm9 );  //V H D0 D1
      _mm_storel_epi64( ( __m128i* )( &( _temp[i >> 1][j - 2] ) ), xmm6 );

      mmStore = xmm9;
    }
  }

  //const int offset = 8 << NO_VALS_LAGR_SHIFT;

  const __m128i mm_0 = _mm_setzero_si128();
  const __m128i mm_15 = _mm_set1_epi64x( 0x000000000000000F );
  const __m128i mm_th = _mm_set1_epi64x( 0x4333333332222210 );

  const __m128i xmm14 = _mm_set1_epi32( 1 ); //offset
  const __m128i xmm13 = _mm_set1_epi32( var_max );

  for( int i = 0; i < ( blk.height >> 1 ); i += 2 )
  {
    for( int j = 0; j < blk.width; j += 8 )
    {
      __m128i  xmm0, xmm1, xmm2, xmm3;
      if ((((i << 1) + blkDst.pos().y) % vbCTUHeight) == (vbPos - 4))
      {
        xmm0 = _mm_loadu_si128((__m128i*)(&(_temp[i + 0][j])));
        xmm1 = _mm_loadu_si128((__m128i*)(&(_temp[i + 1][j])));
        xmm2 = _mm_loadu_si128((__m128i*)(&(_temp[i + 2][j])));
        xmm3 = mm_0;
      }
      else if ((((i << 1) + blkDst.pos().y) % vbCTUHeight) == vbPos)
      {
        xmm0 = mm_0;
        xmm1 = _mm_loadu_si128((__m128i*)(&(_temp[i + 1][j])));
        xmm2 = _mm_loadu_si128((__m128i*)(&(_temp[i + 2][j])));
        xmm3 = _mm_loadu_si128((__m128i*)(&(_temp[i + 3][j])));
      }
      else
      {
        xmm0 = _mm_loadu_si128((__m128i*)(&(_temp[i + 0][j])));
        xmm1 = _mm_loadu_si128((__m128i*)(&(_temp[i + 1][j])));
        xmm2 = _mm_loadu_si128((__m128i*)(&(_temp[i + 2][j])));
        xmm3 = _mm_loadu_si128((__m128i*)(&(_temp[i + 3][j])));
      }


      __m128i xmm4 = _mm_add_epi16( xmm0, xmm1 );
      __m128i xmm6 = _mm_add_epi16( xmm2, xmm3 );

      xmm0 = _mm_unpackhi_epi16( xmm4, mm_0 );
      xmm2 = _mm_unpackhi_epi16( xmm6, mm_0 );
      xmm0 = _mm_add_epi32( xmm0, xmm2 );

      xmm4 = _mm_unpacklo_epi16( xmm4, mm_0 );
      xmm6 = _mm_unpacklo_epi16( xmm6, mm_0 );
      xmm4 = _mm_add_epi32( xmm4, xmm6 );

      __m128i xmm12 = _mm_blend_epi16( xmm4, _mm_shuffle_epi32( xmm0, 0x40 ), 0xF0 );
      __m128i xmm10 = _mm_shuffle_epi32( xmm12, 0xB1 );
      xmm12 = _mm_add_epi32( xmm10, xmm12 );
      if (((((i << 1) + blkDst.pos().y) % vbCTUHeight) == (vbPos - 4)) || ((((i << 1) + blkDst.pos().y) % vbCTUHeight) == vbPos))
      {
        xmm12 = _mm_srai_epi32(_mm_add_epi32(_mm_slli_epi32(xmm12, 5), _mm_slli_epi32(xmm12, 6)), shift);
      }
      else
      {
        xmm12 = _mm_srai_epi32(xmm12, shift - 6);
      }
      xmm12 = _mm_min_epi32( xmm12, xmm13 );

      xmm12 = _mm_and_si128( xmm12, mm_15 );
      xmm12 = _mm_slli_epi32( xmm12, 2 );
      __m128i xmm11 = _mm_shuffle_epi32( xmm12, 0x0E ); //extracted from second half coz no different shifts are available
      xmm12 = _mm_srl_epi64( mm_th, xmm12 );
      xmm11 = _mm_srl_epi64( mm_th, xmm11 );
      xmm12 = _mm_blend_epi16( xmm12, xmm11, 0xF0 );
      xmm12 = _mm_and_si128( xmm12, mm_15 ); // avg_var in lower 4 bits of both halves

      xmm6 = _mm_shuffle_epi32( xmm4, 0xB1 );
      xmm2 = _mm_shuffle_epi32( xmm0, 0xB1 );

      __m128i xmm7 = _mm_set_epi32( 0, 2, 1, 3 );
      __m128i xmm9 = _mm_shuffle_epi32( xmm7, 0xB1 );

      __m128i xmm5 = _mm_cmplt_epi32( xmm6, xmm4 );
      __m128i xmm8 = _mm_cmplt_epi32( xmm2, xmm0 ); //2 masks coz 4 integers for every parts are compared

      xmm5 = _mm_shuffle_epi32( xmm5, 0xA0 );
      xmm8 = _mm_shuffle_epi32( xmm8, 0xA0 );

      xmm4 = _mm_or_si128( _mm_andnot_si128( xmm5, xmm4 ), _mm_and_si128( xmm5, xmm6 ) ); //HV + D
      xmm0 = _mm_or_si128( _mm_andnot_si128( xmm8, xmm0 ), _mm_and_si128( xmm8, xmm2 ) ); //HV + D <--second part

      xmm10 = _mm_or_si128( _mm_andnot_si128( xmm8, xmm7 ), _mm_and_si128( xmm8, xmm9 ) ); //dirTemp <-- second part
      xmm7 = _mm_or_si128( _mm_andnot_si128( xmm5, xmm7 ), _mm_and_si128( xmm5, xmm9 ) ); //dirTemp

      xmm3 = _mm_shuffle_epi32( xmm0, 0x1B );  // need higher part from this
      xmm6 = _mm_shuffle_epi32( xmm4, 0x1B );
      xmm8 = _mm_blend_epi16( xmm4, xmm3, 0xF0 ); // 0 or 3
      xmm6 = _mm_blend_epi16( xmm6, xmm0, 0xF0 );

      xmm6 = _mm_mullo_epi32( xmm8, xmm6 );
      xmm9 = _mm_shuffle_epi32( xmm6, 0xB1 );
      xmm5 = _mm_cmpgt_epi32(_mm_add_epi32(xmm6, _mm_set1_epi32(0x80000000)), _mm_add_epi32(xmm9, _mm_set1_epi32(0x80000000)));
      xmm5 = _mm_shuffle_epi32( xmm5, 0xF0 ); //second mask is for all upper part

      xmm8 = _mm_shuffle_epi32( xmm4, 0x0E );
      xmm8 = _mm_blend_epi16( xmm8, xmm0, 0xF0 ); // (DL, DH in upepr part)
      xmm4 = _mm_blend_epi16( xmm4, _mm_shuffle_epi32( xmm0, 0x40 ), 0xF0 ); //(HVL, HVH) in upper part

      xmm7 = _mm_shuffle_epi32( xmm7, 0x08 ); // 2 -> 1
      xmm7 = _mm_blend_epi16( xmm7, _mm_shuffle_epi32( xmm10, 0x80 ), 0xF0 );
      xmm1 = _mm_shuffle_epi32( xmm7, 0xB1 ); // 1 -> 0, 0 -> 1

      xmm4 = _mm_or_si128( _mm_andnot_si128( xmm5, xmm4 ), _mm_and_si128( xmm5, xmm8 ) ); //HV_D
      xmm7 = _mm_or_si128( _mm_andnot_si128( xmm5, xmm7 ), _mm_and_si128( xmm5, xmm1 ) ); //main - secondary (upper halves are for second value)

                                                                                          //xmm7 not to mix

      xmm0 = _mm_shuffle_epi32( xmm4, 0xFA );
      xmm4 = _mm_shuffle_epi32( xmm4, 0x50 ); //low, low, high, high
      xmm6 = _mm_set_epi32( 2, 1, 9, 2 );

      xmm2 = _mm_mullo_epi32( xmm0, xmm6 );
      xmm6 = _mm_mullo_epi32( xmm4, xmm6 );
      xmm4 = _mm_shuffle_epi32( xmm6, 0x4E );
      xmm0 = _mm_shuffle_epi32( xmm2, 0x4E ); //p to xmm6
      xmm6 = _mm_blend_epi16( xmm6, xmm0, 0xF0 );
      xmm4 = _mm_blend_epi16( xmm4, xmm2, 0xF0 );

      xmm5 = _mm_cmpgt_epi32( xmm4, xmm6 );
      xmm4 = _mm_and_si128( xmm5, xmm14 ); // 1 + 1

      xmm8 = _mm_and_si128( xmm7, xmm14 );
      xmm8 = _mm_slli_epi32( xmm8, 1 );

      xmm5 = _mm_add_epi32( xmm4, _mm_shuffle_epi32( xmm4, 0xB1 ) ); //directionStrength
      xmm4 = _mm_cmpgt_epi32( xmm5, mm_0 ); //is a mask now
      xmm4 = _mm_and_si128( _mm_add_epi32( xmm8, xmm5 ), xmm4 );

      xmm4 = _mm_add_epi32( xmm4, _mm_slli_epi32( xmm4, 2 ) ); //x5
      xmm4 = _mm_add_epi32( xmm4, xmm12 ); //+=

      xmm9 = _mm_shuffle_epi32( xmm7, 0xB1 );// <--
      xmm7 = _mm_slli_epi32( xmm7, 1 );
      xmm9 = _mm_srai_epi32( xmm9, 1 );
      xmm7 = _mm_add_epi32( xmm7, xmm9 );

      //to write to struct
      const int t0 = _mm_extract_epi32( xmm7, 0 );
      const int t1 = _mm_extract_epi32( xmm7, 2 );
      const int c0 = _mm_extract_epi32( xmm4, 0 );
      const int c1 = _mm_extract_epi32( xmm4, 2 );

      const int transposeTable[8] = { 0, 1, 0, 2, 2, 3, 1, 3 };
      int transposeIdx0 = transposeTable[t0];
      int transposeIdx1 = transposeTable[t1];
      int classIdx0 = c0;
      int classIdx1 = c1;

      const int yOffset = (i << 1) + blkDst.pos().y;
      const int xOffset = j + blkDst.pos().x;

      AlfClassifier *cl0 = classifier[yOffset] + xOffset;
      AlfClassifier *cl1 = classifier[yOffset + 1] + xOffset;
      AlfClassifier *cl2 = classifier[yOffset + 2] + xOffset;
      AlfClassifier *cl3 = classifier[yOffset + 3] + xOffset;

      AlfClassifier *_cl0 = cl0 + 4;
      AlfClassifier *_cl1 = cl1 + 4;
      AlfClassifier *_cl2 = cl2 + 4;
      AlfClassifier *_cl3 = cl3 + 4;

      cl0[0] = cl0[1] = cl0[2] = cl0[3] = cl1[0] = cl1[1] = cl1[2] = cl1[3] = cl2[0] = cl2[1] = cl2[2] = cl2[3] = cl3[0] = cl3[1] = cl3[2] = cl3[3] = AlfClassifier( classIdx0, transposeIdx0 );
      _cl0[0] = _cl0[1] = _cl0[2] = _cl0[3] = _cl1[0] = _cl1[1] = _cl1[2] = _cl1[3] = _cl2[0] = _cl2[1] = _cl2[2] = _cl2[3] = _cl3[0] = _cl3[1] = _cl3[2] = _cl3[3] = AlfClassifier( classIdx1, transposeIdx1 );
    }
  }
}

template<X86_VEXT vext>
static void simdFilter5x5Blk(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc,
                             const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet,
                             const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, const int vbCTUHeight,
                             int vbPos)
{
  CHECK((vbCTUHeight & (vbCTUHeight - 1)) != 0, "vbCTUHeight must be a power of 2");

  const bool bChroma = isChroma( compId );

  const SPS*     sps = cs.slice->getSPS();
  bool isDualTree = CS::isDualITree(cs);
  bool isPCMFilterDisabled = sps->getPCMFilterDisableFlag();
  ChromaFormat nChromaFormat = sps->getChromaFormatIdc();

  const CPelBuf srcLuma = recSrc.get( compId );
  PelBuf dstLuma = recDst.get( compId );

  const int srcStride = srcLuma.stride;
  const int dstStride = dstLuma.stride;

  const int numBitsMinus1 = AdaptiveLoopFilter::m_NUM_BITS - 1;
  const int offset = ( 1 << ( AdaptiveLoopFilter::m_NUM_BITS - 2 ) );

  const int startHeight = blk.y;
  const int endHeight = blk.y + blk.height;
  const int startWidth = blk.x;
  const int endWidth = blk.x + blk.width;

  const Pel* src = srcLuma.buf;
  Pel* dst = dstLuma.buf + blkDst.y * dstStride;

  const Pel *pImgYPad0, *pImgYPad1, *pImgYPad2, *pImgYPad3, *pImgYPad4;
  const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4;

  const short *coef[2] = { filterSet, filterSet };
  const short *clip[2] = { fClipSet, fClipSet };

  int transposeIdx[2] = {0, 0};
  const int clsSizeY = 4;
  const int clsSizeX = 4;

  bool pcmFlags2x2[8] = {0,0,0,0,0,0,0,0};
  Pel  pcmRec2x2[32];

  CHECK( startHeight % clsSizeY, "Wrong startHeight in filtering" );
  CHECK( startWidth % clsSizeX, "Wrong startWidth in filtering" );
  CHECK( ( endHeight - startHeight ) % clsSizeY, "Wrong endHeight in filtering" );
  CHECK( ( endWidth - startWidth ) % clsSizeX, "Wrong endWidth in filtering" );

  AlfClassifier *pClass = nullptr;

  int dstStride2 = dstStride * clsSizeY;
  int srcStride2 = srcStride * clsSizeY;

  const __m128i mmOffset = _mm_set1_epi32( offset );
  const __m128i mmMin = _mm_set1_epi16( clpRng.min );
  const __m128i mmMax = _mm_set1_epi16( clpRng.max );

  const unsigned char *filterCoeffIdx[2];
  Pel filterCoeff[MAX_NUM_ALF_LUMA_COEFF][2];
  Pel filterClipp[MAX_NUM_ALF_LUMA_COEFF][2];

  pImgYPad0 = src + startHeight * srcStride + startWidth;
  pImgYPad1 = pImgYPad0 + srcStride;
  pImgYPad2 = pImgYPad0 - srcStride;
  pImgYPad3 = pImgYPad1 + srcStride;
  pImgYPad4 = pImgYPad2 - srcStride;

  Pel* pRec0 = dst + blkDst.x;
  Pel* pRec1;

  for( int i = 0; i < endHeight - startHeight; i += clsSizeY )
  {
    if( !bChroma )
    {
      pClass = classifier[blkDst.y + i] + blkDst.x;
    }

    for( int j = 0; j < endWidth - startWidth; j += 8 )
    {
      for( int k = 0; k < 2; ++k )
      {
        if( !bChroma )
        {
          const AlfClassifier& cl = pClass[j+4*k];
          transposeIdx[k] = cl.transposeIdx;
          coef[k] = filterSet + cl.classIdx * MAX_NUM_ALF_LUMA_COEFF;
          clip[k] = fClipSet + cl.classIdx * MAX_NUM_ALF_LUMA_COEFF;
          if ( isPCMFilterDisabled && cl.classIdx == AdaptiveLoopFilter::m_ALF_UNUSED_CLASSIDX && transposeIdx[k] == AdaptiveLoopFilter::m_ALF_UNUSED_TRANSPOSIDX )
          {
            // Note that last one (i.e. filterCoeff[6][k]) is not unused with JVET_N0242_NON_LINEAR_ALF; could be simplified
            static const unsigned char _filterCoeffIdx[7] = { 0, 0, 0, 0, 0, 0, 0 };
            static short _identityFilterCoeff[] = { 0 };
            static short _identityFilterClipp[] = { 0 };
            filterCoeffIdx[k] = _filterCoeffIdx;
            coef[k] = _identityFilterCoeff;
            clip[k] = _identityFilterClipp;
          }
          else if( transposeIdx[k] == 1 )
          {
            static const unsigned char _filterCoeffIdx[7] = { 4, 1, 5, 3, 0, 2, 6 };
            filterCoeffIdx[k] = _filterCoeffIdx;
          }
          else if( transposeIdx[k] == 2 )
          {
            static const unsigned char _filterCoeffIdx[7] = { 0, 3, 2, 1, 4, 5, 6 };
            filterCoeffIdx[k] = _filterCoeffIdx;
          }
          else if( transposeIdx[k] == 3 )
          {
            static const unsigned char _filterCoeffIdx[7] = { 4, 3, 5, 1, 0, 2, 6 };
            filterCoeffIdx[k] = _filterCoeffIdx;
          }
          else
          {
            static const unsigned char _filterCoeffIdx[7] = { 0, 1, 2, 3, 4, 5, 6 };
            filterCoeffIdx[k] = _filterCoeffIdx;
          }
        }
        else
        {
          static const unsigned char _filterCoeffIdx[7] = { 0, 1, 2, 3, 4, 5, 6 };
          filterCoeffIdx[k] = _filterCoeffIdx;
        }

        for ( int i=0; i < 7; ++i )
        {
          filterCoeff[i][k] = coef[k][filterCoeffIdx[k][i]];
          filterClipp[i][k] = clip[k][filterCoeffIdx[k][i]];
        }
      }


      pRec1 = pRec0 + j;

      if ( bChroma && isPCMFilterDisabled )
      {
        int  blkX, blkY;
        bool *flags  = pcmFlags2x2;
        Pel  *pcmRec = pcmRec2x2;

        // check which chroma 2x2 blocks use PCM
        // chroma PCM may not be aligned with 4x4 ALF processing grid
        for( blkY=0; blkY<4; blkY+=2 )
        {
          for( blkX=0; blkX<8; blkX+=2 )
          {
            Position pos(j + blkDst.x + blkX, i + blkDst.y + blkY);
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB && !JVET_O0050_LOCAL_DUAL_TREE
            const CodingUnit* cu = isDualTree ? cs.getCU(pos, CH_C) : cs.getCU(recalcPosition(nChromaFormat, CH_C, CH_L, pos), CH_L);
#else
            CodingUnit* cu = isDualTree ? cs.getCU(pos, CH_C) : cs.getCU(recalcPosition(nChromaFormat, CH_C, CH_L, pos), CH_L);
#endif
#if JVET_O0050_LOCAL_DUAL_TREE
            cu = cu->isSepTree() ? cs.getCU( pos, CH_C ) : cu;
#endif
            if(cu != NULL) 
            {
              *flags++ = cu->ipcm ? 1 : 0;
            }
            else 
            {
              *flags++ = 0;
            }

            // save original samples from 2x2 PCM blocks
            if( cu != NULL && cu->ipcm )
            {
              *pcmRec++ = pRec1[(blkY+0)*dstStride + (blkX+0)];
              *pcmRec++ = pRec1[(blkY+0)*dstStride + (blkX+1)];
              *pcmRec++ = pRec1[(blkY+1)*dstStride + (blkX+0)];
              *pcmRec++ = pRec1[(blkY+1)*dstStride + (blkX+1)];
            }
          }
        }
      }

      __m128i xmmNull = _mm_setzero_si128();

      for( int ii = 0; ii < clsSizeY; ii++ )
      {
        pImg0 = pImgYPad0 + j + ii * srcStride;
        pImg1 = pImgYPad1 + j + ii * srcStride;
        pImg2 = pImgYPad2 + j + ii * srcStride;
        pImg3 = pImgYPad3 + j + ii * srcStride;
        pImg4 = pImgYPad4 + j + ii * srcStride;

        const int yVb = (blkDst.y + i + ii) & (vbCTUHeight - 1);
        if (yVb < vbPos && (yVb >= vbPos - (bChroma ? 2 : 4)))   // above
        {
          pImg1 = (yVb == vbPos - 1) ? pImg0 : pImg1;
          pImg3 = (yVb >= vbPos - 2) ? pImg1 : pImg3;

          pImg2 = (yVb == vbPos - 1) ? pImg0 : pImg2;
          pImg4 = (yVb >= vbPos - 2) ? pImg2 : pImg4;
        }
        else if (yVb >= vbPos && (yVb <= vbPos + (bChroma ? 1 : 3)))   // bottom
        {
          pImg2 = (yVb == vbPos) ? pImg0 : pImg2;
          pImg4 = (yVb <= vbPos + 1) ? pImg2 : pImg4;

          pImg1 = (yVb == vbPos) ? pImg0 : pImg1;
          pImg3 = (yVb <= vbPos + 1) ? pImg1 : pImg3;
        }
        __m128i clipp, clipm;
        __m128i coeffa, coeffb;
        __m128i xmmCur = _mm_lddqu_si128( ( __m128i* ) ( pImg0 + 0 ) );

        // coeff 0 and 1
        __m128i xmm00 = _mm_lddqu_si128( ( __m128i* ) ( pImg3 + 0 ) );
        xmm00 = _mm_sub_epi16( xmm00, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[0][0], filterClipp[0][0], filterClipp[0][0], filterClipp[0][0],
                                filterClipp[0][1], filterClipp[0][1], filterClipp[0][1], filterClipp[0][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm00 = _mm_min_epi16( xmm00, clipp );
        xmm00 = _mm_max_epi16( xmm00, clipm );

        __m128i xmm01 = _mm_lddqu_si128( ( __m128i* ) ( pImg4 + 0 ) );
        xmm01 = _mm_sub_epi16( xmm01, xmmCur );
        xmm01 = _mm_min_epi16( xmm01, clipp );
        xmm01 = _mm_max_epi16( xmm01, clipm );

        xmm00 = _mm_add_epi16( xmm00, xmm01 );

        __m128i xmm10 = _mm_lddqu_si128( ( __m128i* ) ( pImg1 + 1 ) );
        xmm10 = _mm_sub_epi16( xmm10, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[1][0], filterClipp[1][0], filterClipp[1][0], filterClipp[1][0],
                                filterClipp[1][1], filterClipp[1][1], filterClipp[1][1], filterClipp[1][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm10 = _mm_min_epi16( xmm10, clipp );
        xmm10 = _mm_max_epi16( xmm10, clipm );

        __m128i xmm11 = _mm_lddqu_si128( ( __m128i* ) ( pImg2 - 1 ) );
        xmm11 = _mm_sub_epi16( xmm11, xmmCur );
        xmm11 = _mm_min_epi16( xmm11, clipp );
        xmm11 = _mm_max_epi16( xmm11, clipm );

        xmm10 = _mm_add_epi16( xmm10, xmm11 );

        // 4 first samples
        coeffa = _mm_set1_epi16( filterCoeff[0][0] );
        coeffb = _mm_set1_epi16( filterCoeff[1][0] );
        __m128i xmm0 = _mm_unpacklo_epi16( xmm00, xmm10 );
        __m128i xmmS0 = _mm_madd_epi16( xmm0, _mm_unpackhi_epi16( coeffa, coeffb ) );

        // 4 next samples
        coeffa = _mm_set1_epi16( filterCoeff[0][1] );
        coeffb = _mm_set1_epi16( filterCoeff[1][1] );
        __m128i xmm1 = _mm_unpackhi_epi16( xmm00, xmm10 );
        __m128i xmmS1 = _mm_madd_epi16( xmm1, _mm_unpackhi_epi16( coeffa, coeffb ) );

        // coeff 2 and 3
        __m128i xmm20 = _mm_lddqu_si128( ( __m128i* ) ( pImg1 + 0 ) );
        xmm20 = _mm_sub_epi16( xmm20, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[2][0], filterClipp[2][0], filterClipp[2][0], filterClipp[2][0],
                                filterClipp[2][1], filterClipp[2][1], filterClipp[2][1], filterClipp[2][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm20 = _mm_min_epi16( xmm20, clipp );
        xmm20 = _mm_max_epi16( xmm20, clipm );

        __m128i xmm21 = _mm_lddqu_si128( ( __m128i* ) ( pImg2 + 0 ) );
        xmm21 = _mm_sub_epi16( xmm21, xmmCur );
        xmm21 = _mm_min_epi16( xmm21, clipp );
        xmm21 = _mm_max_epi16( xmm21, clipm );

        xmm20 = _mm_add_epi16( xmm20, xmm21 );

        __m128i xmm30 = _mm_lddqu_si128( ( __m128i* ) ( pImg1 - 1 ) );
        xmm30 = _mm_sub_epi16( xmm30, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[3][0], filterClipp[3][0], filterClipp[3][0], filterClipp[3][0],
                                filterClipp[3][1], filterClipp[3][1], filterClipp[3][1], filterClipp[3][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm30 = _mm_min_epi16( xmm30, clipp );
        xmm30 = _mm_max_epi16( xmm30, clipm );

        __m128i xmm31 = _mm_lddqu_si128( ( __m128i* ) ( pImg2 + 1 ) );
        xmm31 = _mm_sub_epi16( xmm31, xmmCur );
        xmm31 = _mm_min_epi16( xmm31, clipp );
        xmm31 = _mm_max_epi16( xmm31, clipm );

        xmm30 = _mm_add_epi16( xmm30, xmm31 );

        // 4 first samples
        coeffa = _mm_set1_epi16( filterCoeff[2][0] );
        coeffb = _mm_set1_epi16( filterCoeff[3][0] );
        xmm0 = _mm_unpacklo_epi16( xmm20, xmm30 );
        __m128i xmmSt = _mm_madd_epi16( xmm0, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS0 = _mm_add_epi32(xmmS0, xmmSt);

        // 4 next samples
        coeffa = _mm_set1_epi16( filterCoeff[2][1] );
        coeffb = _mm_set1_epi16( filterCoeff[3][1] );
        xmm1 = _mm_unpackhi_epi16( xmm20, xmm30 );
        xmmSt = _mm_madd_epi16( xmm1, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS1 = _mm_add_epi32(xmmS1, xmmSt);

        // coeff 4 and 5
        __m128i xmm40 = _mm_lddqu_si128( ( __m128i* ) ( pImg0 + 2 ) );
        xmm40 = _mm_sub_epi16( xmm40, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[4][0], filterClipp[4][0], filterClipp[4][0], filterClipp[4][0],
                                filterClipp[4][1], filterClipp[4][1], filterClipp[4][1], filterClipp[4][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm40 = _mm_min_epi16( xmm40, clipp );
        xmm40 = _mm_max_epi16( xmm40, clipm );

        __m128i xmm41 = _mm_lddqu_si128( ( __m128i* ) ( pImg0 - 2 ) );
        xmm41 = _mm_sub_epi16( xmm41, xmmCur );
        xmm41 = _mm_min_epi16( xmm41, clipp );
        xmm41 = _mm_max_epi16( xmm41, clipm );

        xmm40 = _mm_add_epi16( xmm40, xmm41 );

        __m128i xmm50 = _mm_lddqu_si128( ( __m128i* ) ( pImg0 + 1 ) );
        xmm50 = _mm_sub_epi16( xmm50, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[5][0], filterClipp[5][0], filterClipp[5][0], filterClipp[5][0],
                                filterClipp[5][1], filterClipp[5][1], filterClipp[5][1], filterClipp[5][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm50 = _mm_min_epi16( xmm50, clipp );
        xmm50 = _mm_max_epi16( xmm50, clipm );

        __m128i xmm51 = _mm_lddqu_si128( ( __m128i* ) ( pImg0 - 1 ) );
        xmm51 = _mm_sub_epi16( xmm51, xmmCur );
        xmm51 = _mm_min_epi16( xmm51, clipp );
        xmm51 = _mm_max_epi16( xmm51, clipm );

        xmm50 = _mm_add_epi16( xmm50, xmm51 );

        // 4 first samples
        coeffa = _mm_set1_epi16( filterCoeff[4][0] );
        coeffb = _mm_set1_epi16( filterCoeff[5][0] );
        xmm0 = _mm_unpacklo_epi16( xmm40, xmm50 );
        xmmSt = _mm_madd_epi16( xmm0, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS0 = _mm_add_epi32(xmmS0, xmmSt);

        // 4 next samples
        coeffa = _mm_set1_epi16( filterCoeff[4][1] );
        coeffb = _mm_set1_epi16( filterCoeff[5][1] );
        xmm1 = _mm_unpackhi_epi16( xmm40, xmm50 );
        xmmSt = _mm_madd_epi16( xmm1, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS1 = _mm_add_epi32(xmmS1, xmmSt);

        // finish
        xmmS0 = _mm_add_epi32( xmmS0, mmOffset );
        xmmS0 = _mm_srai_epi32( xmmS0, numBitsMinus1 );
        xmmS1 = _mm_add_epi32( xmmS1, mmOffset );
        xmmS1 = _mm_srai_epi32( xmmS1, numBitsMinus1 );

        xmmS0 = _mm_packs_epi32( xmmS0, xmmS1 );
        // coeff 6
        xmmS0 = _mm_add_epi16(xmmS0, xmmCur);
        xmmS0 = _mm_min_epi16( mmMax, _mm_max_epi16( xmmS0, mmMin ) );

        if( j + 8 <= endWidth - startWidth )
        {
          _mm_storeu_si128( ( __m128i* )( pRec1 ), xmmS0 );
        }
        else if( j + 6 == endWidth - startWidth )
        {
          xmmS0 = _mm_blend_epi16( xmmS0, xmmCur, 0xC0 );
          _mm_storeu_si128( ( __m128i* )( pRec1 ), xmmS0 );
        }
        else if( j + 4 == endWidth - startWidth )
        {
          xmmS0 = _mm_blend_epi16( xmmS0, xmmCur, 0xF0 );
          _mm_storeu_si128( ( __m128i* )( pRec1 ), xmmS0 );
        }
        else
        {
          xmmS0 = _mm_blend_epi16( xmmS0, xmmCur, 0xFC );
          _mm_storeu_si128( ( __m128i* )( pRec1 ), xmmS0 );
        }

        pRec1 += dstStride;
      }
      pRec1 -= dstStride2;
      // restore 2x2 PCM chroma blocks
      if( bChroma && isPCMFilterDisabled )
      {
        int  blkX, blkY;
        bool *flags  = pcmFlags2x2;
        Pel  *pcmRec = pcmRec2x2;
        for( blkY=0; blkY<4; blkY+=2 )
        {
          for( blkX=0; blkX<8; blkX+=2 )
          {
            if( *flags++ )
            {
              pRec1[(blkY+0)*dstStride + (blkX+0)] = *pcmRec++;
              pRec1[(blkY+0)*dstStride + (blkX+1)] = *pcmRec++;
              pRec1[(blkY+1)*dstStride + (blkX+0)] = *pcmRec++;
              pRec1[(blkY+1)*dstStride + (blkX+1)] = *pcmRec++;
            }
          }
        }
      }

    }

    pRec0 += dstStride2;

    pImgYPad0 += srcStride2;
    pImgYPad1 += srcStride2;
    pImgYPad2 += srcStride2;
    pImgYPad3 += srcStride2;
    pImgYPad4 += srcStride2;
  }
}

template<X86_VEXT vext>
static void simdFilter7x7Blk(AlfClassifier **classifier, const PelUnitBuf &recDst, const CPelUnitBuf &recSrc,
                             const Area &blkDst, const Area &blk, const ComponentID compId, const short *filterSet,
                             const short *fClipSet, const ClpRng &clpRng, CodingStructure &cs, const int vbCTUHeight,
                             int vbPos)
{
  CHECK((vbCTUHeight & (vbCTUHeight - 1)) != 0, "vbCTUHeight must be a power of 2");

  const bool bChroma = isChroma( compId );

  const SPS*     sps = cs.slice->getSPS();
  bool isDualTree = CS::isDualITree(cs);
  bool isPCMFilterDisabled = sps->getPCMFilterDisableFlag();
  ChromaFormat nChromaFormat = sps->getChromaFormatIdc();
  const CPelBuf srcLuma = recSrc.get( compId );
  PelBuf dstLuma = recDst.get( compId );

  const int srcStride = srcLuma.stride;
  const int dstStride = dstLuma.stride;

  const int numBitsMinus1 = AdaptiveLoopFilter::m_NUM_BITS - 1;
  const int offset = ( 1 << ( AdaptiveLoopFilter::m_NUM_BITS - 2 ) );

  const int startHeight = blk.y;
  const int endHeight = blk.y + blk.height;
  const int startWidth = blk.x;
  const int endWidth = blk.x + blk.width;

  const Pel* src = srcLuma.buf;
  Pel* dst = dstLuma.buf + blkDst.y * dstStride;

  const Pel *pImgYPad0, *pImgYPad1, *pImgYPad2, *pImgYPad3, *pImgYPad4, *pImgYPad5, *pImgYPad6;
  const Pel *pImg0, *pImg1, *pImg2, *pImg3, *pImg4, *pImg5, *pImg6;

  const short *coef[2] = { filterSet, filterSet };
  const short *clip[2] = { fClipSet, fClipSet };

  int transposeIdx[2] = {0, 0};
  const int clsSizeY = 4;
  const int clsSizeX = 4;

  bool pcmFlags2x2[8] = {0,0,0,0,0,0,0,0};
  Pel  pcmRec2x2[32];

  CHECK( startHeight % clsSizeY, "Wrong startHeight in filtering" );
  CHECK( startWidth % clsSizeX, "Wrong startWidth in filtering" );
  CHECK( ( endHeight - startHeight ) % clsSizeY, "Wrong endHeight in filtering" );
  CHECK( ( endWidth - startWidth ) % clsSizeX, "Wrong endWidth in filtering" );

  AlfClassifier *pClass = nullptr;

  int dstStride2 = dstStride * clsSizeY;
  int srcStride2 = srcStride * clsSizeY;

  const __m128i mmOffset = _mm_set1_epi32( offset );
  const __m128i mmMin = _mm_set1_epi16( clpRng.min );
  const __m128i mmMax = _mm_set1_epi16( clpRng.max );

  const unsigned char *filterCoeffIdx[2];
  Pel filterCoeff[MAX_NUM_ALF_LUMA_COEFF][2];
  Pel filterClipp[MAX_NUM_ALF_LUMA_COEFF][2];

  pImgYPad0 = src + startHeight * srcStride + startWidth;
  pImgYPad1 = pImgYPad0 + srcStride;
  pImgYPad2 = pImgYPad0 - srcStride;
  pImgYPad3 = pImgYPad1 + srcStride;
  pImgYPad4 = pImgYPad2 - srcStride;
  pImgYPad5 = pImgYPad3 + srcStride;
  pImgYPad6 = pImgYPad4 - srcStride;

  Pel* pRec0 = dst + blkDst.x;
  Pel* pRec1;

  for( int i = 0; i < endHeight - startHeight; i += clsSizeY )
  {
    if( !bChroma )
    {
      pClass = classifier[blkDst.y + i] + blkDst.x;
    }

    for( int j = 0; j < endWidth - startWidth; j += 8 )
    {
      for (int k = 0; k < 2; ++k)
      {
        if( !bChroma )
        {
          const AlfClassifier& cl = pClass[j+4*k];
          transposeIdx[k] = cl.transposeIdx;
          coef[k] = filterSet + cl.classIdx * MAX_NUM_ALF_LUMA_COEFF;
          clip[k] = fClipSet + cl.classIdx * MAX_NUM_ALF_LUMA_COEFF;
          if ( isPCMFilterDisabled && cl.classIdx == AdaptiveLoopFilter::m_ALF_UNUSED_CLASSIDX && transposeIdx[k] == AdaptiveLoopFilter::m_ALF_UNUSED_TRANSPOSIDX )
          {
            // Note that last one (i.e. filterCoeff[12][k]) is not unused with JVET_N0242_NON_LINEAR_ALF; could be simplified
            static const unsigned char _filterCoeffIdx[13] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            static short _identityFilterCoeff[] = { 0 };
            static short _identityFilterClipp[] = { 0 };
            filterCoeffIdx[k] = _filterCoeffIdx;
            coef[k] = _identityFilterCoeff;
            clip[k] = _identityFilterClipp;
          }
          else if( transposeIdx[k] == 1 )
          {
            static const unsigned char _filterCoeffIdx[13] = { 9, 4, 10, 8, 1, 5, 11, 7, 3, 0, 2, 6, 12 };
            filterCoeffIdx[k] = _filterCoeffIdx;
          }
          else if( transposeIdx[k] == 2 )
          {
            static const unsigned char _filterCoeffIdx[13] = { 0, 3, 2, 1, 8, 7, 6, 5, 4, 9, 10, 11, 12 };
            filterCoeffIdx[k] = _filterCoeffIdx;
          }
          else if( transposeIdx[k] == 3 )
          {
            static const unsigned char _filterCoeffIdx[13] = { 9, 8, 10, 4, 3, 7, 11, 5, 1, 0, 2, 6, 12 };
            filterCoeffIdx[k] = _filterCoeffIdx;
          }
          else
          {
            static const unsigned char _filterCoeffIdx[13] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
            filterCoeffIdx[k] = _filterCoeffIdx;
          }
        }
        else
        {
          static const unsigned char _filterCoeffIdx[13] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
          filterCoeffIdx[k] = _filterCoeffIdx;
        }

        for ( int i=0; i < 13; ++i )
        {
          filterCoeff[i][k] = coef[k][filterCoeffIdx[k][i]];
          filterClipp[i][k] = clip[k][filterCoeffIdx[k][i]];
        }
      }

      pRec1 = pRec0 + j;

      if ( bChroma && isPCMFilterDisabled )
      {
        int  blkX, blkY;
        bool *flags  = pcmFlags2x2;
        Pel  *pcmRec = pcmRec2x2;

        // check which chroma 2x2 blocks use PCM
        // chroma PCM may not be aligned with 4x4 ALF processing grid
        for( blkY=0; blkY<4; blkY+=2 )
        {
          for( blkX=0; blkX<8; blkX+=2 )
          {
            Position pos(j + blkDst.x + blkX, i + blkDst.y + blkY);
#if JVET_O0090_ALF_CHROMA_FILTER_ALTERNATIVES_CTB && !JVET_O0050_LOCAL_DUAL_TREE
            const CodingUnit* cu = isDualTree ? cs.getCU(pos, CH_C) : cs.getCU(recalcPosition(nChromaFormat, CH_C, CH_L, pos), CH_L);
#else
            CodingUnit* cu = isDualTree ? cs.getCU(pos, CH_C) : cs.getCU(recalcPosition(nChromaFormat, CH_C, CH_L, pos), CH_L);
#endif
#if JVET_O0050_LOCAL_DUAL_TREE
            cu = cu->isSepTree() ? cs.getCU( pos, CH_C ) : cu;
#endif
            if( cu != NULL) 
            {
              *flags++ = cu->ipcm ? 1 : 0;
            }
            else
            {
              *flags++ = 0;
            }

            // save original samples from 2x2 PCM blocks
            if( cu != NULL && cu->ipcm )
            {
              *pcmRec++ = pRec1[(blkY+0)*dstStride + (blkX+0)];
              *pcmRec++ = pRec1[(blkY+0)*dstStride + (blkX+1)];
              *pcmRec++ = pRec1[(blkY+1)*dstStride + (blkX+0)];
              *pcmRec++ = pRec1[(blkY+1)*dstStride + (blkX+1)];
            }
          }
        }
      }

      __m128i xmmNull = _mm_setzero_si128();

      for( int ii = 0; ii < clsSizeY; ii++ )
      {
        pImg0 = pImgYPad0 + j + ii * srcStride;
        pImg1 = pImgYPad1 + j + ii * srcStride;
        pImg2 = pImgYPad2 + j + ii * srcStride;
        pImg3 = pImgYPad3 + j + ii * srcStride;
        pImg4 = pImgYPad4 + j + ii * srcStride;
        pImg5 = pImgYPad5 + j + ii * srcStride;
        pImg6 = pImgYPad6 + j + ii * srcStride;

        const int yVb = (blkDst.y + i + ii) & (vbCTUHeight - 1);
        if (yVb < vbPos && (yVb >= vbPos - (bChroma ? 2 : 4)))   // above
        {
          pImg1 = (yVb == vbPos - 1) ? pImg0 : pImg1;
          pImg3 = (yVb >= vbPos - 2) ? pImg1 : pImg3;
          pImg5 = (yVb >= vbPos - 3) ? pImg3 : pImg5;

          pImg2 = (yVb == vbPos - 1) ? pImg0 : pImg2;
          pImg4 = (yVb >= vbPos - 2) ? pImg2 : pImg4;
          pImg6 = (yVb >= vbPos - 3) ? pImg4 : pImg6;
        }
        else if (yVb >= vbPos && (yVb <= vbPos + (bChroma ? 1 : 3)))   // bottom
        {
          pImg2 = (yVb == vbPos) ? pImg0 : pImg2;
          pImg4 = (yVb <= vbPos + 1) ? pImg2 : pImg4;
          pImg6 = (yVb <= vbPos + 2) ? pImg4 : pImg6;

          pImg1 = (yVb == vbPos) ? pImg0 : pImg1;
          pImg3 = (yVb <= vbPos + 1) ? pImg1 : pImg3;
          pImg5 = (yVb <= vbPos + 2) ? pImg3 : pImg5;
        }
        __m128i clipp, clipm;
        __m128i coeffa, coeffb;
        __m128i xmmCur = _mm_lddqu_si128( ( __m128i* ) ( pImg0 + 0 ) );

        // coeff 0 and 1
        __m128i xmm00 = _mm_lddqu_si128( ( __m128i* ) ( pImg5 + 0 ) );
        xmm00 = _mm_sub_epi16( xmm00, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[0][0], filterClipp[0][0], filterClipp[0][0], filterClipp[0][0],
                                filterClipp[0][1], filterClipp[0][1], filterClipp[0][1], filterClipp[0][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm00 = _mm_min_epi16( xmm00, clipp );
        xmm00 = _mm_max_epi16( xmm00, clipm );

        __m128i xmm01 = _mm_lddqu_si128( ( __m128i* ) ( pImg6 + 0 ) );
        xmm01 = _mm_sub_epi16( xmm01, xmmCur );
        xmm01 = _mm_min_epi16( xmm01, clipp );
        xmm01 = _mm_max_epi16( xmm01, clipm );

        xmm00 = _mm_add_epi16( xmm00, xmm01 );

        __m128i xmm10 = _mm_lddqu_si128( ( __m128i* ) ( pImg3 + 1 ) );
        xmm10 = _mm_sub_epi16( xmm10, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[1][0], filterClipp[1][0], filterClipp[1][0], filterClipp[1][0],
                                filterClipp[1][1], filterClipp[1][1], filterClipp[1][1], filterClipp[1][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm10 = _mm_min_epi16( xmm10, clipp );
        xmm10 = _mm_max_epi16( xmm10, clipm );

        __m128i xmm11 = _mm_lddqu_si128( ( __m128i* ) ( pImg4 - 1 ) );
        xmm11 = _mm_sub_epi16( xmm11, xmmCur );
        xmm11 = _mm_min_epi16( xmm11, clipp );
        xmm11 = _mm_max_epi16( xmm11, clipm );

        xmm10 = _mm_add_epi16( xmm10, xmm11 );

        // 4 first samples
        coeffa = _mm_set1_epi16( filterCoeff[0][0] );
        coeffb = _mm_set1_epi16( filterCoeff[1][0] );
        __m128i xmm0 = _mm_unpacklo_epi16( xmm00, xmm10 );
        __m128i xmmS0 = _mm_madd_epi16( xmm0, _mm_unpackhi_epi16( coeffa, coeffb ) );

        // 4 next samples
        coeffa = _mm_set1_epi16( filterCoeff[0][1] );
        coeffb = _mm_set1_epi16( filterCoeff[1][1] );
        __m128i xmm1 = _mm_unpackhi_epi16( xmm00, xmm10 );
        __m128i xmmS1 = _mm_madd_epi16( xmm1, _mm_unpackhi_epi16( coeffa, coeffb ) );

        // coeff 2 and 3
        __m128i xmm20 = _mm_lddqu_si128( ( __m128i* ) ( pImg3 + 0 ) );
        xmm20 = _mm_sub_epi16( xmm20, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[2][0], filterClipp[2][0], filterClipp[2][0], filterClipp[2][0],
                                filterClipp[2][1], filterClipp[2][1], filterClipp[2][1], filterClipp[2][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm20 = _mm_min_epi16( xmm20, clipp );
        xmm20 = _mm_max_epi16( xmm20, clipm );

        __m128i xmm21 = _mm_lddqu_si128( ( __m128i* ) ( pImg4 + 0 ) );
        xmm21 = _mm_sub_epi16( xmm21, xmmCur );
        xmm21 = _mm_min_epi16( xmm21, clipp );
        xmm21 = _mm_max_epi16( xmm21, clipm );

        xmm20 = _mm_add_epi16( xmm20, xmm21 );

        __m128i xmm30 = _mm_lddqu_si128( ( __m128i* ) ( pImg3 - 1 ) );
        xmm30 = _mm_sub_epi16( xmm30, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[3][0], filterClipp[3][0], filterClipp[3][0], filterClipp[3][0],
                                filterClipp[3][1], filterClipp[3][1], filterClipp[3][1], filterClipp[3][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm30 = _mm_min_epi16( xmm30, clipp );
        xmm30 = _mm_max_epi16( xmm30, clipm );

        __m128i xmm31 = _mm_lddqu_si128( ( __m128i* ) ( pImg4 + 1 ) );
        xmm31 = _mm_sub_epi16( xmm31, xmmCur );
        xmm31 = _mm_min_epi16( xmm31, clipp );
        xmm31 = _mm_max_epi16( xmm31, clipm );

        xmm30 = _mm_add_epi16( xmm30, xmm31 );

        // 4 first samples
        coeffa = _mm_set1_epi16( filterCoeff[2][0] );
        coeffb = _mm_set1_epi16( filterCoeff[3][0] );
        xmm0 = _mm_unpacklo_epi16( xmm20, xmm30 );
        __m128i xmmSt = _mm_madd_epi16( xmm0, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS0 = _mm_add_epi32(xmmS0, xmmSt);

        // 4 next samples
        coeffa = _mm_set1_epi16( filterCoeff[2][1] );
        coeffb = _mm_set1_epi16( filterCoeff[3][1] );
        xmm1 = _mm_unpackhi_epi16( xmm20, xmm30 );
        xmmSt = _mm_madd_epi16( xmm1, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS1 = _mm_add_epi32(xmmS1, xmmSt);

        // coeff 4 and 5
        __m128i xmm40 = _mm_lddqu_si128( ( __m128i* ) ( pImg1 + 2 ) );
        xmm40 = _mm_sub_epi16( xmm40, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[4][0], filterClipp[4][0], filterClipp[4][0], filterClipp[4][0],
                                filterClipp[4][1], filterClipp[4][1], filterClipp[4][1], filterClipp[4][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm40 = _mm_min_epi16( xmm40, clipp );
        xmm40 = _mm_max_epi16( xmm40, clipm );

        __m128i xmm41 = _mm_lddqu_si128( ( __m128i* ) ( pImg2 - 2 ) );
        xmm41 = _mm_sub_epi16( xmm41, xmmCur );
        xmm41 = _mm_min_epi16( xmm41, clipp );
        xmm41 = _mm_max_epi16( xmm41, clipm );

        xmm40 = _mm_add_epi16( xmm40, xmm41 );

        __m128i xmm50 = _mm_lddqu_si128( ( __m128i* ) ( pImg1 + 1 ) );
        xmm50 = _mm_sub_epi16( xmm50, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[5][0], filterClipp[5][0], filterClipp[5][0], filterClipp[5][0],
                                filterClipp[5][1], filterClipp[5][1], filterClipp[5][1], filterClipp[5][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm50 = _mm_min_epi16( xmm50, clipp );
        xmm50 = _mm_max_epi16( xmm50, clipm );

        __m128i xmm51 = _mm_lddqu_si128( ( __m128i* ) ( pImg2 - 1 ) );
        xmm51 = _mm_sub_epi16( xmm51, xmmCur );
        xmm51 = _mm_min_epi16( xmm51, clipp );
        xmm51 = _mm_max_epi16( xmm51, clipm );

        xmm50 = _mm_add_epi16( xmm50, xmm51 );

        // 4 first samples
        coeffa = _mm_set1_epi16( filterCoeff[4][0] );
        coeffb = _mm_set1_epi16( filterCoeff[5][0] );
        xmm0 = _mm_unpacklo_epi16( xmm40, xmm50 );
        xmmSt = _mm_madd_epi16( xmm0, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS0 = _mm_add_epi32(xmmS0, xmmSt);

        // 4 next samples
        coeffa = _mm_set1_epi16( filterCoeff[4][1] );
        coeffb = _mm_set1_epi16( filterCoeff[5][1] );
        xmm1 = _mm_unpackhi_epi16( xmm40, xmm50 );
        xmmSt = _mm_madd_epi16( xmm1, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS1 = _mm_add_epi32(xmmS1, xmmSt);


        // coeff 6 and 7
        __m128i xmm60 = _mm_lddqu_si128( ( __m128i* ) ( pImg1 + 0 ) );
        xmm60 = _mm_sub_epi16( xmm60, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[6][0], filterClipp[6][0], filterClipp[6][0], filterClipp[6][0],
                                filterClipp[6][1], filterClipp[6][1], filterClipp[6][1], filterClipp[6][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm60 = _mm_min_epi16( xmm60, clipp );
        xmm60 = _mm_max_epi16( xmm60, clipm );

        __m128i xmm61 = _mm_lddqu_si128( ( __m128i* ) ( pImg2 + 0 ) );
        xmm61 = _mm_sub_epi16( xmm61, xmmCur );
        xmm61 = _mm_min_epi16( xmm61, clipp );
        xmm61 = _mm_max_epi16( xmm61, clipm );

        xmm60 = _mm_add_epi16( xmm60, xmm61 );

        __m128i xmm70 = _mm_lddqu_si128( ( __m128i* ) ( pImg1 - 1 ) );
        xmm70 = _mm_sub_epi16( xmm70, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[7][0], filterClipp[7][0], filterClipp[7][0], filterClipp[7][0],
                                filterClipp[7][1], filterClipp[7][1], filterClipp[7][1], filterClipp[7][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm70 = _mm_min_epi16( xmm70, clipp );
        xmm70 = _mm_max_epi16( xmm70, clipm );

        __m128i xmm71 = _mm_lddqu_si128( ( __m128i* ) ( pImg2 + 1 ) );
        xmm71 = _mm_sub_epi16( xmm71, xmmCur );
        xmm71 = _mm_min_epi16( xmm71, clipp );
        xmm71 = _mm_max_epi16( xmm71, clipm );

        xmm70 = _mm_add_epi16( xmm70, xmm71 );

        // 4 first samples
        coeffa = _mm_set1_epi16( filterCoeff[6][0] );
        coeffb = _mm_set1_epi16( filterCoeff[7][0] );
        xmm0 = _mm_unpacklo_epi16( xmm60, xmm70 );
        xmmSt = _mm_madd_epi16( xmm0, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS0 = _mm_add_epi32(xmmS0, xmmSt);

        // 4 next samples
        coeffa = _mm_set1_epi16( filterCoeff[6][1] );
        coeffb = _mm_set1_epi16( filterCoeff[7][1] );
        xmm1 = _mm_unpackhi_epi16( xmm60, xmm70 );
        xmmSt = _mm_madd_epi16( xmm1, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS1 = _mm_add_epi32(xmmS1, xmmSt);


        // coeff 8 and 9
        __m128i xmm80 = _mm_lddqu_si128( ( __m128i* ) ( pImg1 - 2 ) );
        xmm80 = _mm_sub_epi16( xmm80, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[8][0], filterClipp[8][0], filterClipp[8][0], filterClipp[8][0],
                                filterClipp[8][1], filterClipp[8][1], filterClipp[8][1], filterClipp[8][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm80 = _mm_min_epi16( xmm80, clipp );
        xmm80 = _mm_max_epi16( xmm80, clipm );

        __m128i xmm81 = _mm_lddqu_si128( ( __m128i* ) ( pImg2 + 2 ) );
        xmm81 = _mm_sub_epi16( xmm81, xmmCur );
        xmm81 = _mm_min_epi16( xmm81, clipp );
        xmm81 = _mm_max_epi16( xmm81, clipm );

        xmm80 = _mm_add_epi16( xmm80, xmm81 );

        __m128i xmm90 = _mm_lddqu_si128( ( __m128i* ) ( pImg0 + 3 ) );
        xmm90 = _mm_sub_epi16( xmm90, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[9][0], filterClipp[9][0], filterClipp[9][0], filterClipp[9][0],
                                filterClipp[9][1], filterClipp[9][1], filterClipp[9][1], filterClipp[9][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm90 = _mm_min_epi16( xmm90, clipp );
        xmm90 = _mm_max_epi16( xmm90, clipm );

        __m128i xmm91 = _mm_lddqu_si128( ( __m128i* ) ( pImg0 - 3 ) );
        xmm91 = _mm_sub_epi16( xmm91, xmmCur );
        xmm91 = _mm_min_epi16( xmm91, clipp );
        xmm91 = _mm_max_epi16( xmm91, clipm );

        xmm90 = _mm_add_epi16( xmm90, xmm91 );

        // 4 first samples
        coeffa = _mm_set1_epi16( filterCoeff[8][0] );
        coeffb = _mm_set1_epi16( filterCoeff[9][0] );
        xmm0 = _mm_unpacklo_epi16( xmm80, xmm90 );
        xmmSt = _mm_madd_epi16( xmm0, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS0 = _mm_add_epi32(xmmS0, xmmSt);

        // 4 next samples
        coeffa = _mm_set1_epi16( filterCoeff[8][1] );
        coeffb = _mm_set1_epi16( filterCoeff[9][1] );
        xmm1 = _mm_unpackhi_epi16( xmm80, xmm90 );
        xmmSt = _mm_madd_epi16( xmm1, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS1 = _mm_add_epi32(xmmS1, xmmSt);


        // coeff 10 and 11
        __m128i xmm100 = _mm_lddqu_si128( ( __m128i* ) ( pImg0 + 2 ) );
        xmm100 = _mm_sub_epi16( xmm100, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[10][0], filterClipp[10][0], filterClipp[10][0], filterClipp[10][0],
                                filterClipp[10][1], filterClipp[10][1], filterClipp[10][1], filterClipp[10][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm100 = _mm_min_epi16( xmm100, clipp );
        xmm100 = _mm_max_epi16( xmm100, clipm );

        __m128i xmm101 = _mm_lddqu_si128( ( __m128i* ) ( pImg0 - 2 ) );
        xmm101 = _mm_sub_epi16( xmm101, xmmCur );
        xmm101 = _mm_min_epi16( xmm101, clipp );
        xmm101 = _mm_max_epi16( xmm101, clipm );

        xmm100 = _mm_add_epi16( xmm100, xmm101 );

        __m128i xmm110 = _mm_lddqu_si128( ( __m128i* ) ( pImg0 + 1 ) );
        xmm110 = _mm_sub_epi16( xmm110, xmmCur );
        clipp = _mm_setr_epi16( filterClipp[11][0], filterClipp[11][0], filterClipp[11][0], filterClipp[11][0],
                                filterClipp[11][1], filterClipp[11][1], filterClipp[11][1], filterClipp[11][1] );
        clipm = _mm_sub_epi16( xmmNull, clipp );
        xmm110 = _mm_min_epi16( xmm110, clipp );
        xmm110 = _mm_max_epi16( xmm110, clipm );

        __m128i xmm111 = _mm_lddqu_si128( ( __m128i* ) ( pImg0 - 1 ) );
        xmm111 = _mm_sub_epi16( xmm111, xmmCur );
        xmm111 = _mm_min_epi16( xmm111, clipp );
        xmm111 = _mm_max_epi16( xmm111, clipm );

        xmm110 = _mm_add_epi16( xmm110, xmm111 );

        // 4 first samples
        coeffa = _mm_set1_epi16( filterCoeff[10][0] );
        coeffb = _mm_set1_epi16( filterCoeff[11][0] );
        xmm0 = _mm_unpacklo_epi16( xmm100, xmm110 );
        xmmSt = _mm_madd_epi16( xmm0, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS0 = _mm_add_epi32(xmmS0, xmmSt);

        // 4 next samples
        coeffa = _mm_set1_epi16( filterCoeff[10][1] );
        coeffb = _mm_set1_epi16( filterCoeff[11][1] );
        xmm1 = _mm_unpackhi_epi16( xmm100, xmm110 );
        xmmSt = _mm_madd_epi16( xmm1, _mm_unpackhi_epi16( coeffa, coeffb ) );

        xmmS1 = _mm_add_epi32(xmmS1, xmmSt);

        // finish
        xmmS0 = _mm_add_epi32( xmmS0, mmOffset );
        xmmS0 = _mm_srai_epi32( xmmS0, numBitsMinus1 );
        xmmS1 = _mm_add_epi32( xmmS1, mmOffset );
        xmmS1 = _mm_srai_epi32( xmmS1, numBitsMinus1 );

        xmmS0 = _mm_packs_epi32( xmmS0, xmmS1 );
        // coeff 12
        xmmS0 = _mm_add_epi16(xmmS0, xmmCur);
        xmmS0 = _mm_min_epi16( mmMax, _mm_max_epi16( xmmS0, mmMin ) );

        if( j + 8 <= endWidth - startWidth )
        {
          _mm_storeu_si128( ( __m128i* )( pRec1 ), xmmS0 );
        }
        else if( j + 6 == endWidth - startWidth )
        {
          xmmS0 = _mm_blend_epi16( xmmS0, xmmCur, 0xC0 );
          _mm_storeu_si128( ( __m128i* )( pRec1 ), xmmS0 );
        }
        else if( j + 4 == endWidth - startWidth )
        {
          xmmS0 = _mm_blend_epi16( xmmS0, xmmCur, 0xF0 );
          _mm_storeu_si128( ( __m128i* )( pRec1 ), xmmS0 );
        }
        else
        {
          xmmS0 = _mm_blend_epi16( xmmS0, xmmCur, 0xFC );
          _mm_storeu_si128( ( __m128i* )( pRec1 ), xmmS0 );
        }


        pRec1 += dstStride;
      }
      pRec1 -= dstStride2;
      // restore 2x2 PCM chroma blocks
      if( bChroma && isPCMFilterDisabled )
      {
        int  blkX, blkY;
        bool *flags  = pcmFlags2x2;
        Pel  *pcmRec = pcmRec2x2;
        for( blkY=0; blkY<4; blkY+=2 )
        {
          for( blkX=0; blkX<8; blkX+=2 )
          {
            if( *flags++ )
            {
              pRec1[(blkY+0)*dstStride + (blkX+0)] = *pcmRec++;
              pRec1[(blkY+0)*dstStride + (blkX+1)] = *pcmRec++;
              pRec1[(blkY+1)*dstStride + (blkX+0)] = *pcmRec++;
              pRec1[(blkY+1)*dstStride + (blkX+1)] = *pcmRec++;
            }
          }
        }
      }

    }

    pRec0 += dstStride2;

    pImgYPad0 += srcStride2;
    pImgYPad1 += srcStride2;
    pImgYPad2 += srcStride2;
    pImgYPad3 += srcStride2;
    pImgYPad4 += srcStride2;
    pImgYPad5 += srcStride2;
    pImgYPad6 += srcStride2;
  }
}

template <X86_VEXT vext>
void AdaptiveLoopFilter::_initAdaptiveLoopFilterX86()
{
  m_deriveClassificationBlk = simdDeriveClassificationBlk<vext>;
  m_filter5x5Blk = simdFilter5x5Blk<vext>;
  m_filter7x7Blk = simdFilter7x7Blk<vext>;
}

template void AdaptiveLoopFilter::_initAdaptiveLoopFilterX86<SIMDX86>();
#endif //#ifdef TARGET_SIMD_X86
//! \}
