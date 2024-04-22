/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2023, ITU/ISO/IEC
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

/**
 * \file
 * \brief Implementation of InterpolationFilter class
 */
//#define USE_AVX2
// ====================================================================================================================
// Includes
// ====================================================================================================================

#include "CommonDefX86.h"
#include "../Rom.h"
#include "../InterpolationFilter.h"

#if GPM_CURVE
#include "../UnitTools.h"
#endif

//#include "../ChromaFormat.h"

//! \ingroup CommonLib
//! \{

#ifdef TARGET_SIMD_X86

#if defined _MSC_VER
#include <tmmintrin.h>
#else
#include <immintrin.h>
#endif

// because of sub pu atmvp and tripple split
#define JEM_UNALIGNED_DST 1

// ===========================
// Full-pel copy 8-bit/16-bit
// ===========================
template<typename Tsrc, int N, bool isFirst, bool isLast>
#if MCIF_SIMD_NEW
static void fullPelCopySSE(const ClpRng& clpRng, const void*_src, int srcStride, int16_t *dst, int dstStride, int width, int height, bool biMCForDMVR)
#else
static void fullPelCopySSE( const ClpRng& clpRng, const void*_src, int srcStride, int16_t *dst, int dstStride, int width, int height )
#endif
{
  Tsrc* src = (Tsrc*)_src;

#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  int headroom = IF_INTERNAL_FRAC_BITS(clpRng.bd);
#else
  int headroom = IF_INTERNAL_PREC - clpRng.bd;
#endif
  int headroom_offset = 1 << ( headroom - 1 );
  int offset   = IF_INTERNAL_OFFS;
  __m128i voffset  = _mm_set1_epi16( offset );
  __m128i voffset_headroom  = _mm_set1_epi16( headroom_offset );

  __m128i vibdimin = _mm_set1_epi16( clpRng.min );
  __m128i vibdimax = _mm_set1_epi16( clpRng.max );
  __m128i vsrc, vsum;

  for( int row = 0; row < height; row++ )
  {
    for( int col = 0; col < width; col+=N )
    {
      _mm_prefetch( (const char*)src+2*srcStride, _MM_HINT_T0 );
      _mm_prefetch( (const char*)src+( width>>1 ) + 2*srcStride, _MM_HINT_T0 );
      _mm_prefetch( (const char*)src+width-1 + 2*srcStride, _MM_HINT_T0 );
      for( int i=0; i<N; i+=8 )
      {
        if( sizeof( Tsrc )==1 )
        {
          vsrc = _mm_cvtepu8_epi16( _mm_lddqu_si128( ( __m128i const * )&src[col+i] ) );
        }
        else
        {
          vsrc = _mm_lddqu_si128( ( __m128i const * )&src[col+i] );
        }
#if MCIF_SIMD_NEW
        if (biMCForDMVR)
        {
          vsum = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsrc));
        }
        else if (isFirst == isLast)
        {
          vsum = vsrc;
        }
#else
        if (isFirst == isLast)
        {
          vsum = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsrc));
        }
#endif
        else if( isFirst )
        {
          vsrc = _mm_slli_epi16( vsrc, headroom );
          vsum = _mm_sub_epi16( vsrc, voffset );
        }
        else
        {
          vsrc = _mm_add_epi16( vsrc, voffset );
          vsrc = _mm_add_epi16( vsrc, voffset_headroom );
          vsrc = _mm_srai_epi16( vsrc, headroom );
          vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsrc ) );
        }
#if JEM_UNALIGNED_DST
        _mm_storeu_si128( ( __m128i * )&dst[col+i], vsum );
#else
        _mm_store_si128( ( __m128i * )&dst[col+i], vsum );
#endif
      }
    }
    src += srcStride;
    dst += dstStride;
  }
}

#if MCIF_SIMD_NEW
template<typename Tsrc, int N, bool isFirst, bool isLast>
static void fullPelCopyVerSSE(const ClpRng& clpRng, const void*_src, int srcStride, int16_t *dst, int dstStride, int width, int height, bool biMCForDMVR)
{
  Tsrc* src = (Tsrc*)_src;
  short* dstinit = dst;

  int headroom = IF_INTERNAL_PREC - clpRng.bd;
  int headroom_offset = 1 << (headroom - 1);
  int offset = IF_INTERNAL_OFFS;
  __m128i voffset = _mm_set1_epi16(offset);
  __m128i voffset_headroom = _mm_set1_epi16(headroom_offset);

  __m128i vibdimin = _mm_set1_epi16(clpRng.min);
  __m128i vibdimax = _mm_set1_epi16(clpRng.max);
  __m128i vsrc, vsum;

  for (int col = 0; col < width; col++)
  {
    src = (Tsrc*)_src;
    dst = dstinit;
    for (int row = 0; row < height; row += N)
    {
      _mm_prefetch((const char*)src + 2 * srcStride, _MM_HINT_T0);
      _mm_prefetch((const char*)src + (width >> 1) + 2 * srcStride, _MM_HINT_T0);
      _mm_prefetch((const char*)src + width - 1 + 2 * srcStride, _MM_HINT_T0);
      for (int i = 0; i < N; i += 8)
      {
        {
          vsrc = _mm_set_epi16(src[col + (7 + i) * srcStride], src[col + (6 + i) * srcStride], src[col + (5 + i) * srcStride], src[col + (4 + i) * srcStride]
            , src[col + (3 + i) * srcStride], src[col + (2 + i) * srcStride], src[col + (1 + i) * srcStride], src[col + i * srcStride]);
        }
        if (biMCForDMVR)
        {
          vsum = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsrc));
        }
        else if (isFirst == isLast)
        {
          vsum = vsrc;
        }
        else if (isFirst)
        {
          vsrc = _mm_slli_epi16(vsrc, headroom);
          vsum = _mm_sub_epi16(vsrc, voffset);
        }
        else
        {
          vsrc = _mm_add_epi16(vsrc, voffset);
          vsrc = _mm_add_epi16(vsrc, voffset_headroom);
          vsrc = _mm_srai_epi16(vsrc, headroom);
          vsum = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsrc));
        }
        short result[8];
        _mm_storeu_si128((__m128i *)&result[0], vsum);
        for (int j = 0; j < 8; j++)
        {
          dst[col + (j + i) * dstStride] = result[j];
        }
      }
      src += srcStride * N;
      dst += dstStride * N;
    }
  }
}
#endif


#if MCIF_SIMD_NEW
template<typename Tsrc, bool isFirst, bool isLast>
static void fullPelCopySSE_M4(const ClpRng& clpRng, const void*_src, ptrdiff_t srcStride, int16_t *dst, ptrdiff_t dstStride, int width, int height, bool biMCForDMVR)
{
  Tsrc* src = (Tsrc*)_src;

  int headroom = IF_INTERNAL_PREC - clpRng.bd;
  int headroom_offset = 1 << (headroom - 1);
  int offset = IF_INTERNAL_OFFS;
  __m128i voffset = _mm_set1_epi16(offset);
  __m128i voffset_headroom = _mm_set1_epi16(headroom_offset);
  __m128i vibdimin = _mm_set1_epi16(clpRng.min);
  __m128i vibdimax = _mm_set1_epi16(clpRng.max);

  __m128i vsrc, vsum;

  for (int row = 0; row < height; row++)
  {
    for (int col = 0; col < width; col += 4)
    {
      _mm_prefetch((const char*)src + 2 * srcStride, _MM_HINT_T0);
      _mm_prefetch((const char*)src + (width >> 1) + 2 * srcStride, _MM_HINT_T0);
      _mm_prefetch((const char*)src + width - 1 + 2 * srcStride, _MM_HINT_T0);

      if (sizeof(Tsrc) == 1)
      {
        vsrc = _mm_cvtepu8_epi16(_mm_loadl_epi64((__m128i const *)&src[col]));
      }
      else
      {
        vsrc = _mm_loadl_epi64((__m128i const *)&src[col]);
      }
      if (biMCForDMVR)
      {
        vsum = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsrc));
      }
      else if (isFirst == isLast)
      {
        vsum = vsrc;
      }
      else if (isFirst)
      {
        vsrc = _mm_slli_epi16(vsrc, headroom);
        vsum = _mm_sub_epi16(vsrc, voffset);
      }
      else
      {
        vsrc = _mm_add_epi16(vsrc, voffset);
        vsrc = _mm_add_epi16(vsrc, voffset_headroom);
        vsrc = _mm_srai_epi16(vsrc, headroom);
        vsum = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsrc));
      }

      _mm_storel_epi64((__m128i *)&dst[col], vsum);
    }
    src += srcStride;
    dst += dstStride;
  }
}
#endif

#if MCIF_SIMD_NEW
template<typename Tsrc, bool isFirst, bool isLast>
static void fullPelCopyVerSSE_M4(const ClpRng& clpRng, const void*_src, ptrdiff_t srcStride, int16_t *dst, ptrdiff_t dstStride, int width, int height, bool biMCForDMVR)
{
  Tsrc* src = (Tsrc*)_src;
  //  short* dstinit = dst;

  int headroom = IF_INTERNAL_PREC - clpRng.bd;
  int headroom_offset = 1 << (headroom - 1);
  int offset = IF_INTERNAL_OFFS;
  __m128i voffset = _mm_set1_epi16(offset);
  __m128i voffset_headroom = _mm_set1_epi16(headroom_offset);
  __m128i vibdimin = _mm_set1_epi16(clpRng.min);
  __m128i vibdimax = _mm_set1_epi16(clpRng.max);

  __m128i vsrc, vsum;

  for (int col = 0; col < width; col++)
  {
    for (int row = 0; row < height; row += 4)
    {
      _mm_prefetch((const char*)src + 2 * srcStride, _MM_HINT_T0);
      _mm_prefetch((const char*)src + (width >> 1) + 2 * srcStride, _MM_HINT_T0);
      _mm_prefetch((const char*)src + width - 1 + 2 * srcStride, _MM_HINT_T0);

      {
        vsrc = _mm_set_epi16(0, 0, 0, 0
          , src[col + (row + 3) * srcStride], src[col + (row + 2) * srcStride], src[col + (row + 1) * srcStride], src[col + row * srcStride]);
      }
      if (biMCForDMVR)
      {
        vsum = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsrc));
      }
      else if (isFirst == isLast)
      {
        vsum = vsrc;
      }
      else if (isFirst)
      {
        vsrc = _mm_slli_epi16(vsrc, headroom);
        vsum = _mm_sub_epi16(vsrc, voffset);
      }
      else
      {
        vsrc = _mm_add_epi16(vsrc, voffset);
        vsrc = _mm_add_epi16(vsrc, voffset_headroom);
        vsrc = _mm_srai_epi16(vsrc, headroom);
        vsum = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsrc));
      }
      short result[4];
      _mm_storel_epi64((__m128i *)&result[0], vsum);
      for (int j = 0; j < 4; j++)
      {
        dst[col + (j + row) * dstStride] = result[j];
      }
    }
  }
}
#endif

template<typename Tsrc, int N, bool isFirst, bool isLast>
#if MCIF_SIMD_NEW
static void fullPelCopyAVX2(const ClpRng& clpRng, const void*_src, int srcStride, short *dst, int dstStride, int width, int height, bool biMCForDMVR)
#else
static void fullPelCopyAVX2( const ClpRng& clpRng, const void*_src, int srcStride, short *dst, int dstStride, int width, int height )
#endif
{
#ifdef USE_AVX2
  Tsrc* src = (Tsrc*)_src;

#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  int headroom = IF_INTERNAL_FRAC_BITS(clpRng.bd);
#else
  int headroom = IF_INTERNAL_PREC - clpRng.bd;
#endif
  int offset   = 1 << ( headroom - 1 );
  int internal_offset = IF_INTERNAL_OFFS;

  __m256i vinternal_offset = _mm256_set1_epi16( internal_offset );
  __m256i vheadroom_offset = _mm256_set1_epi16( offset );

  __m256i vibdimin = _mm256_set1_epi16( clpRng.min );
  __m256i vibdimax = _mm256_set1_epi16( clpRng.max );
  __m256i vsrc, vsum;


  for( int row = 0; row < height; row++ )
  {
    for( int col = 0; col < width; col+=N )
    {
      _mm_prefetch( (const char*)( src+3*srcStride ), _MM_HINT_T0 );
      _mm_prefetch( (const char*)( src+( width>>1 ) + 3*srcStride ), _MM_HINT_T0 );
      _mm_prefetch( (const char*)( src+width-1 + 3*srcStride ), _MM_HINT_T0 );
      for( int i=0; i<N; i+=16 )
      {
        if( sizeof( Tsrc )==1 )
        {
          vsrc = _mm256_cvtepu8_epi16( _mm_loadu_si128( ( const __m128i * )&src[col+i] ) );
        }
        else
        {
          vsrc = _mm256_lddqu_si256( ( const __m256i * )&src[col+i] );
        }
#if MCIF_SIMD_NEW
        if (biMCForDMVR)
        {
          vsum = _mm256_min_epi16(vibdimax, _mm256_max_epi16(vibdimin, vsrc));
        }
        else if (isFirst == isLast)
        {
          vsum = vsrc;
        }
#else
        if (isFirst == isLast)
        {
          vsum = _mm256_min_epi16(vibdimax, _mm256_max_epi16(vibdimin, vsrc));
        }
#endif
        else if( isFirst )
        {
          vsrc = _mm256_slli_epi16( vsrc, headroom );
          vsum = _mm256_sub_epi16( vsrc, vinternal_offset );
        }
        else
        {
          vsrc = _mm256_add_epi16( vsrc, vinternal_offset );
          vsrc = _mm256_add_epi16( vsrc, vheadroom_offset );
          vsrc = _mm256_srai_epi16( vsrc, headroom );
          vsum = _mm256_min_epi16( vibdimax, _mm256_max_epi16( vibdimin, vsrc ) );
        }
#if JEM_UNALIGNED_DST
        _mm256_storeu_si256( ( __m256i * )&dst[col+i], vsum );
#else
        _mm256_store_si256( ( __m256i * )&dst[col+i], vsum );
#endif

      }
    }
    src += srcStride;
    dst += dstStride;
  }
#endif
}

#if MCIF_SIMD_NEW
template<typename Tsrc, int N, bool isFirst, bool isLast>
static void fullPelCopyVerAVX2(const ClpRng& clpRng, const void*_src, int srcStride, short *dst, int dstStride, int width, int height, bool biMCForDMVR)
{
#ifdef USE_AVX2
  Tsrc* src = (Tsrc*)_src;
  short* dstinit = dst;

  int headroom = IF_INTERNAL_PREC - clpRng.bd;
  int offset = 1 << (headroom - 1);
  int internal_offset = IF_INTERNAL_OFFS;

  __m256i vinternal_offset = _mm256_set1_epi16(internal_offset);
  __m256i vheadroom_offset = _mm256_set1_epi16(offset);

  __m256i vibdimin = _mm256_set1_epi16(clpRng.min);
  __m256i vibdimax = _mm256_set1_epi16(clpRng.max);
  __m256i vsrc, vsum;

  for (int col = 0; col < width; col++)
  {
    src = (Tsrc*)_src;
    dst = dstinit;
    for (int row = 0; row < height; row += N)
    {
      _mm_prefetch((const char*)(src + 3 * srcStride), _MM_HINT_T0);
      _mm_prefetch((const char*)(src + (width >> 1) + 3 * srcStride), _MM_HINT_T0);
      _mm_prefetch((const char*)(src + width - 1 + 3 * srcStride), _MM_HINT_T0);
      for (int i = 0; i < N; i += 16)
      {
        {
          vsrc = _mm256_set_epi16(src[col + 15 * srcStride], src[col + 14 * srcStride], src[col + 13 * srcStride], src[col + 12 * srcStride]
            , src[col + 11 * srcStride], src[col + 10 * srcStride], src[col + 9 * srcStride], src[col + 8 * srcStride]
            , src[col + 7 * srcStride], src[col + 6 * srcStride], src[col + 5 * srcStride], src[col + 4 * srcStride]
            , src[col + 3 * srcStride], src[col + 2 * srcStride], src[col + 1 * srcStride], src[col]);
        }
        if (biMCForDMVR)
        {
          vsum = _mm256_min_epi16(vibdimax, _mm256_max_epi16(vibdimin, vsrc));
        }
        else if (isFirst == isLast)
        {
          vsum = vsrc;
        }
        else if (isFirst)
        {
          vsrc = _mm256_slli_epi16(vsrc, headroom);
          vsum = _mm256_sub_epi16(vsrc, vinternal_offset);
        }
        else
        {
          vsrc = _mm256_add_epi16(vsrc, vinternal_offset);
          vsrc = _mm256_add_epi16(vsrc, vheadroom_offset);
          vsrc = _mm256_srai_epi16(vsrc, headroom);
          vsum = _mm256_min_epi16(vibdimax, _mm256_max_epi16(vibdimin, vsrc));
        }
        short result[16];
        _mm256_storeu_si256((__m256i *)&result[0], vsum);
        for (int j = 0; j < N; j++)
        {
          dst[col + j * dstStride] = result[j];
        }

      }
      src += srcStride * 16;
      dst += dstStride * 16;
    }
  }
#endif
}
#endif

template<X86_VEXT vext, bool isFirst, bool isLast>
static void simdFilterCopy( const ClpRng& clpRng, const Pel* src, int srcStride, int16_t* dst, int dstStride, int width, int height, bool biMCForDMVR)
{
#if !MCIF_SIMD_NEW
  { //Scalar
    InterpolationFilter::filterCopy<isFirst, isLast>( clpRng, src, srcStride, dst, dstStride, width, height, biMCForDMVR);
  }
#endif
#if MCIF_SIMD_NEW
  if (vext >= AVX2 && (width % 16) == 0)
  {
    fullPelCopyAVX2<Pel, 16, isFirst, isLast >(clpRng, src, srcStride, dst, dstStride, width, height, biMCForDMVR); // if IF_INTERNAL_PREC_BILINEAR is not equal to 10, need to modify filter copy SIMD or revert to filterCopy<>
  }
  else if ((width % 16) == 0)
  {
    fullPelCopySSE<Pel, 16, isFirst, isLast >(clpRng, src, srcStride, dst, dstStride, width, height, biMCForDMVR);
  }
  else if ((width % 8) == 0)
  {
    fullPelCopySSE<Pel, 8, isFirst, isLast>(clpRng, src, srcStride, dst, dstStride, width, height, biMCForDMVR);
  }
  else if ((width % 4) == 0)
  {
    fullPelCopySSE_M4<Pel, isFirst, isLast>(clpRng, src, srcStride, dst, dstStride, width, height, biMCForDMVR);
  }
  else if (vext >= AVX2 && (height % 16) == 0)
  {
    fullPelCopyVerAVX2<Pel, 16, isFirst, isLast >(clpRng, src, srcStride, dst, dstStride, width, height, biMCForDMVR);
  }
  else if ((height % 16) == 0)
  {
    fullPelCopyVerSSE<Pel, 16, isFirst, isLast >(clpRng, src, srcStride, dst, dstStride, width, height, biMCForDMVR);
  }
  else if ((height % 8) == 0)
  {
    fullPelCopyVerSSE<Pel, 8, isFirst, isLast >(clpRng, src, srcStride, dst, dstStride, width, height, biMCForDMVR);
  }
  else if ((height % 4) == 0)
  {
    fullPelCopyVerSSE_M4<Pel, isFirst, isLast>(clpRng, src, srcStride, dst, dstStride, width, height, biMCForDMVR);
  }
  else
  { //Scalar
    InterpolationFilter::filterCopy<isFirst, isLast>(clpRng, src, srcStride, dst, dstStride, width, height, biMCForDMVR);
  }
#endif
}

#if JVET_AC0104_IBC_BVD_PREDICTION
// ============================================
// Full-pel copy 8-bit/16-bit with no clipping
// ============================================

template<typename Tsrc, int N>
static void fullPelCopyWithNoClippingAVX2(const ClpRng& clpRng, const Pel* _src, int srcStride, int16_t* dst, int dstStride, int width, int height)
{
#ifdef USE_AVX2
  Tsrc* src = (Tsrc*)_src;
  __m256i vsum;


  for (int row = 0; row < height; row++)
  {
    for (int col = 0; col < width; col += N)
    {
      _mm_prefetch((const char*)(src +                3 * srcStride), _MM_HINT_T0);
      _mm_prefetch((const char*)(src + (width >> 1) + 3 * srcStride), _MM_HINT_T0);
      _mm_prefetch((const char*)(src +  width - 1   + 3 * srcStride), _MM_HINT_T0);
      for (int i = 0; i < N; i += 16)
      {
        vsum = (sizeof(Tsrc) == 1) ? _mm256_cvtepu8_epi16(_mm_loadu_si128((const __m128i*) & src[col + i]))
                                   : _mm256_lddqu_si256((const __m256i*) & src[col + i]);
#if JEM_UNALIGNED_DST
        _mm256_storeu_si256((__m256i*) & dst[col + i], vsum);
#else
        _mm256_store_si256((__m256i*) & dst[col + i], vsum);
#endif
      }
    }
    src += srcStride;
    dst += dstStride;
  }
#endif
}

template<typename Tsrc, int N>
static void fullPelCopyWithNoClippingVerAVX2(const ClpRng& clpRng, const Pel* _src, int srcStride, int16_t* dst, int dstStride, int width, int height)
{
#ifdef USE_AVX2
  Tsrc* src = (Tsrc*)_src;
  short* dstinit = dst;
  __m256i vsum;

  for (int col = 0; col < width; col++)
  {
    src = (Tsrc*)_src;
    dst = dstinit;
    for (int row = 0; row < height; row += N)
    {
      _mm_prefetch((const char*)(src +                3 * srcStride), _MM_HINT_T0);
      _mm_prefetch((const char*)(src + (width >> 1) + 3 * srcStride), _MM_HINT_T0);
      _mm_prefetch((const char*)(src +  width - 1   + 3 * srcStride), _MM_HINT_T0);
      {
        vsum = _mm256_set_epi16(src[col + 15 * srcStride], src[col + 14 * srcStride], src[col + 13 * srcStride], src[col + 12 * srcStride]
                              , src[col + 11 * srcStride], src[col + 10 * srcStride], src[col +  9 * srcStride], src[col +  8 * srcStride]
                              , src[col +  7 * srcStride], src[col +  6 * srcStride], src[col +  5 * srcStride], src[col +  4 * srcStride]
                              , src[col +  3 * srcStride], src[col +  2 * srcStride], src[col +  1 * srcStride], src[col]);

        _mm256_storeu_si256((__m256i*) &(dst[col]), vsum);

      }
      src += srcStride * 16;
      dst += dstStride * 16;
    }
  }
#endif
}

template<typename Tsrc, int N>
static void fullPelCopyWithNoClippingSSE(const ClpRng& clpRng, const Pel* _src, int srcStride, int16_t* dst, int dstStride, int width, int height)
{
  Tsrc* src = (Tsrc*)_src;
  __m128i vsum;

  for (int row = 0; row < height; row++)
  {
    for (int col = 0; col < width; col += N)
    {
      _mm_prefetch((const char*)src +                2 * srcStride, _MM_HINT_T0);
      _mm_prefetch((const char*)src + (width >> 1) + 2 * srcStride, _MM_HINT_T0);
      _mm_prefetch((const char*)src +  width - 1   + 2 * srcStride, _MM_HINT_T0);
      for (int i = 0; i < N; i += 8)
      {
        vsum = (sizeof(Tsrc) == 1) ? _mm_cvtepu8_epi16(_mm_lddqu_si128((__m128i const*) & src[col + i]))
                                   : _mm_lddqu_si128((__m128i const*) & src[col + i]);

#if JEM_UNALIGNED_DST
        _mm_storeu_si128((__m128i*) & dst[col + i], vsum);
#else
        _mm_store_si128((__m128i*) & dst[col + i], vsum);
#endif
      }
    }
    src += srcStride;
    dst += dstStride;
  }
}

template<typename Tsrc, int N>
static void fullPelCopyWithNoClippingVerSSE(const ClpRng& clpRng, const Pel* _src, int srcStride, int16_t* dst, int dstStride, int width, int height)
{
  Tsrc* src = (Tsrc*)_src;
  short* dstinit = dst;
  __m128i vsum;
  
  for (int col = 0; col < width; col++)
  {
    src = (Tsrc*)_src;
    dst = dstinit;
    for (int row = 0; row < height; row += N)
    {
      _mm_prefetch((const char*)src +                2 * srcStride, _MM_HINT_T0);
      _mm_prefetch((const char*)src + (width >> 1) + 2 * srcStride, _MM_HINT_T0);
      _mm_prefetch((const char*)src +  width - 1   + 2 * srcStride, _MM_HINT_T0);
      for (int i = 0; i < N; i += 8)
      {
        vsum = _mm_set_epi16(src[col + (7 + i) * srcStride], src[col + (6 + i) * srcStride], src[col + (5 + i) * srcStride], src[col + (4 + i) * srcStride]
                           , src[col + (3 + i) * srcStride], src[col + (2 + i) * srcStride], src[col + (1 + i) * srcStride], src[col +      i  * srcStride]);

        _mm_storeu_si128((__m128i*) & dst[col + (i) * dstStride], vsum);
      }
      src += srcStride * N;
      dst += dstStride * N;
    }
  }
}

template<typename Tsrc>
static void fullPelCopyWithNoClippingSSE_M4(const ClpRng& clpRng, const Pel* _src, int srcStride, int16_t* dst, int dstStride, int width, int height)
{
  Tsrc* src = (Tsrc*)_src;
  __m128i vsum;

  for (int row = 0; row < height; row++)
  {
    for (int col = 0; col < width; col += 4)
    {
      _mm_prefetch((const char*)src +                2 * srcStride, _MM_HINT_T0);
      _mm_prefetch((const char*)src + (width >> 1) + 2 * srcStride, _MM_HINT_T0);
      _mm_prefetch((const char*)src +  width - 1   + 2 * srcStride, _MM_HINT_T0);

      vsum = (sizeof(Tsrc) == 1) ? _mm_cvtepu8_epi16(_mm_loadl_epi64((__m128i const*) & src[col]))
                                 : _mm_loadl_epi64((__m128i const*) & src[col]);
      _mm_storel_epi64((__m128i*) & dst[col], vsum);
    }
    src += srcStride;
    dst += dstStride;
  }
}

template<typename Tsrc>
static void fullPelCopyWithNoClippingVerSSE_M4(const ClpRng& clpRng, const Pel* _src, int srcStride, int16_t* dst, int dstStride, int width, int height)
{
  Tsrc* src = (Tsrc*)_src;
  __m128i vsum;

  for (int col = 0; col < width; col++)
  {
    for (int row = 0; row < height; row += 4)
    {
      _mm_prefetch((const char*)src +                2 * srcStride, _MM_HINT_T0);
      _mm_prefetch((const char*)src + (width >> 1) + 2 * srcStride, _MM_HINT_T0);
      _mm_prefetch((const char*)src +  width - 1   + 2 * srcStride, _MM_HINT_T0);

      vsum = _mm_set_epi16(0, 0, 0, 0, src[col + (row + 3) * srcStride], src[col + (row + 2) * srcStride], src[col + (row + 1) * srcStride], src[col + row * srcStride]);
      _mm_storel_epi64((__m128i*) & dst[col + (row) * dstStride], vsum);
    }
  }
}

template<X86_VEXT vext>
static void simdFilterCopyWithNoClipping(const ClpRng& clpRng, const Pel* src, int srcStride, int16_t* dst, int dstStride, int width, int height)
{
  if (vext >= AVX2 && (width % 16) == 0)
  {
    fullPelCopyWithNoClippingAVX2<Pel, 16>(clpRng, src, srcStride, dst, dstStride, width, height);
  }
  else if ((width % 16) == 0)
  {
    fullPelCopyWithNoClippingSSE<Pel, 16>(clpRng, src, srcStride, dst, dstStride, width, height);
  }
  else if ((width % 8) == 0)
  {
    fullPelCopyWithNoClippingSSE<Pel, 8>(clpRng, src, srcStride, dst, dstStride, width, height);
  }
  else if ((width % 4) == 0)
  {
    fullPelCopyWithNoClippingSSE_M4<Pel>(clpRng, src, srcStride, dst, dstStride, width, height);
  }
  else if (vext >= AVX2 && (height % 16) == 0)
  {
    fullPelCopyWithNoClippingVerAVX2<Pel, 16>(clpRng, src, srcStride, dst, dstStride, width, height);
  }
  else if ((height % 16) == 0)
  {
    fullPelCopyWithNoClippingVerSSE<Pel, 16>(clpRng, src, srcStride, dst, dstStride, width, height);
  }
  else if ((height % 8) == 0)
  {
    fullPelCopyWithNoClippingVerSSE<Pel, 8>(clpRng, src, srcStride, dst, dstStride, width, height);
  }
  else if ((height % 4) == 0)
  {
    fullPelCopyWithNoClippingVerSSE_M4<Pel>(clpRng, src, srcStride, dst, dstStride, width, height);
  }
  else
  { //Scalar
    InterpolationFilter::filterCopyWithNoClipping(clpRng, src, srcStride, dst, dstStride, width, height);
  }
}
#endif

// SIMD interpolation horizontal, block width modulo 4
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateHorM4( const int16_t* src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
  _mm_prefetch( (const char*)src + srcStride, _MM_HINT_T0 );
  __m128i voffset = _mm_set1_epi32( offset );
  __m128i vibdimin = _mm_set1_epi16( clpRng.min );
  __m128i vibdimax = _mm_set1_epi16( clpRng.max );
  __m128i vcoeffh = _mm_lddqu_si128( ( __m128i const * )coeff );

#if JVET_Z0117_CHROMA_IF
  __m128i vcoeffh1;
#endif
  __m128i vzero, vshufc0, vshufc1;
  __m128i vsum;

  if( N != 8 ){
    vcoeffh = _mm_shuffle_epi32( vcoeffh, 0x44 );
    vzero = _mm_setzero_si128();
    vshufc0 = _mm_set_epi8( 0x9, 0x8, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0 );
    vshufc1 = _mm_set_epi8( 0xd, 0xc, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0x5, 0x4 );
  }
#if JVET_Z0117_CHROMA_IF
  int16_t c0[8];
  int16_t c1[8];
  __m128i vshufc2;
  if(N == 6)
  {
    vshufc2 = _mm_set_epi8(  0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0 );
    for(int i = 0; i < 8; i += 4)
    {
      c0[i] = coeff[0];
      c0[i+1] = coeff[1];
      c0[i+2] = coeff[2];
      c0[i+3] = coeff[3];
      c1[i] = coeff[4];
      c1[i+1] = coeff[5];
      c1[i+2] = coeff[4];
      c1[i+3] = coeff[5];
    }
    vcoeffh = _mm_lddqu_si128( ( __m128i const * )c0 );
    vcoeffh1 = _mm_lddqu_si128( ( __m128i const * )c1 );
  }
#endif

  vzero = _mm_setzero_si128();

  for( int row = 0; row < height; row++ )
  {
    _mm_prefetch( (const char*)src + 2 * srcStride, _MM_HINT_T0 );
    for( int col = 0; col < width; col += 4 )
    {
      if( N == 8 )
      {
        __m128i vtmp[2];
        for( int i = 0; i < 4; i += 2 )
        {
          __m128i vsrc0 = _mm_lddqu_si128( ( __m128i const * )&src[col + i] );
          __m128i vsrc1 = _mm_lddqu_si128( ( __m128i const * )&src[col + i + 1] );
          vsrc0 = _mm_madd_epi16( vsrc0, vcoeffh );
          vsrc1 = _mm_madd_epi16( vsrc1, vcoeffh );
          vtmp[i / 2] = _mm_hadd_epi32( vsrc0, vsrc1 );
        }
        vsum = _mm_hadd_epi32( vtmp[0], vtmp[1] );
      }
#if JVET_Z0117_CHROMA_IF
      else if(N == 6)
      {
          __m128i vsrc0 = _mm_lddqu_si128( ( __m128i const * )&src[col] );
          __m128i vsrc1 = _mm_lddqu_si128( ( __m128i const * )&src[col+4] );
          __m128i temp1 = _mm_shuffle_epi8( vsrc0, vshufc0 );
          __m128i temp2 = _mm_shuffle_epi8( vsrc0, vshufc1 );
          __m128i temp3 = _mm_shuffle_epi8( vsrc1, vshufc2 );
          temp1 = _mm_madd_epi16( temp1, vcoeffh );
          temp2 = _mm_madd_epi16( temp2, vcoeffh );
          temp3 = _mm_madd_epi16( temp3, vcoeffh1 );
          temp1 = _mm_hadd_epi32(temp1, temp2);
          vsum = _mm_add_epi32(temp1, temp3);
      }
#endif
      else
      {
        __m128i vtmp0, vtmp1;
        __m128i vsrc = _mm_lddqu_si128( ( __m128i const * )&src[col] );
        vtmp0 = _mm_shuffle_epi8( vsrc, vshufc0 );
        vtmp1 = _mm_shuffle_epi8( vsrc, vshufc1 );

        vtmp0 = _mm_madd_epi16( vtmp0, vcoeffh );
        vtmp1 = _mm_madd_epi16( vtmp1, vcoeffh );
        vsum = _mm_hadd_epi32( vtmp0, vtmp1 );
      }

      {
        vsum = _mm_add_epi32( vsum, voffset );
        vsum = _mm_srai_epi32( vsum, shift );
        vsum = _mm_packs_epi32( vsum, vzero );
      }

      if( shiftBack )
      { //clip
        vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
      }
      _mm_storel_epi64( ( __m128i * )&dst[col], vsum );
    }
    src += srcStride;
    dst += dstStride;
  }
}

#if JVET_Z0117_CHROMA_IF
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateHorM4_sym( const int16_t* src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
  _mm_prefetch( (const char*)src + srcStride, _MM_HINT_T0 );
  __m128i voffset = _mm_set1_epi32( offset );
  __m128i vibdimin = _mm_set1_epi16( clpRng.min );
  __m128i vibdimax = _mm_set1_epi16( clpRng.max );
  __m128i vcoeffh = _mm_lddqu_si128( ( __m128i const * )coeff );
  __m128i vcoeffh1;
  __m128i vzero, vshufc0, vshufc1, vshufc2, vshufc3, vshufc4;
  __m128i vsum;
  vcoeffh = _mm_shuffle_epi32( vcoeffh, 0x44 );
  vzero = _mm_setzero_si128();
  vshufc0 = _mm_set_epi8( 0x9, 0x8, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0 );
  vshufc1 = _mm_set_epi8( 0xd, 0xc, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0x5, 0x4);
  vshufc2 = _mm_set_epi8( 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0, 0x9, 0x8, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2);
  vshufc3 = _mm_set_epi8( 0xf, 0xe, 0x7, 0x6, 0xd, 0xc, 0x5, 0x4, 0xb, 0xa, 0x3, 0x2, 0x9, 0x8, 0x1, 0x0 );
  vshufc4 = _mm_set_epi8( 0xf, 0xe, 0xd, 0xc, 0xb, 0xa, 0x9, 0x8, 0xd, 0xc, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6);
  int16_t c0[8];
  for(int i = 0; i < 8; i += 4)
  {
    c0[i] = coeff[0];
    c0[i+1] = coeff[1];
    c0[i+2] = coeff[0];
    c0[i+3] = coeff[1];
  }
  vcoeffh = _mm_lddqu_si128( ( __m128i const * )c0 );
  vcoeffh1 = _mm_set1_epi32( coeff[2]);
  vzero = _mm_setzero_si128();
  for( int row = 0; row < height; row++ )
  {
    _mm_prefetch( (const char*)src + 2 * srcStride, _MM_HINT_T0 );
    for( int col = 0; col < width; col += 4 )
    {
      __m128i vsrc0 = _mm_lddqu_si128( ( __m128i const * )&src[col] );
      __m128i vsrc1 = _mm_lddqu_si128( ( __m128i const * )&src[col+4] );
      __m128i temp1 = _mm_shuffle_epi8( vsrc0, vshufc0 );
      __m128i temp3 = _mm_shuffle_epi8( vsrc1, vshufc2 );
      __m128i temp2 = _mm_shuffle_epi8( vsrc0, vshufc1 );
      __m128i temp4 = _mm_shuffle_epi8( vsrc0, vshufc4) ;
      temp1 = _mm_add_epi16(temp1, temp3);
      temp2 = _mm_add_epi16(temp2, temp4);
      temp1 = _mm_shuffle_epi8(temp1, vshufc3);
      temp1 = _mm_madd_epi16( temp1, vcoeffh );
      temp2 = _mm_cvtepi16_epi32(temp2);
      temp2 = _mm_mullo_epi32(temp2, vcoeffh1);
      vsum = _mm_add_epi32(temp1, temp2);
      vsum = _mm_add_epi32( vsum, voffset );
      vsum = _mm_srai_epi32( vsum, shift );
      vsum = _mm_packs_epi32( vsum, vzero );
      if( shiftBack )
      {
        vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
      }
      _mm_storel_epi64( ( __m128i * )&dst[col], vsum );
    }
    src += srcStride;
    dst += dstStride;
  }
}
#endif

// SIMD interpolation horizontal, block width modulo 8
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateHorM8( const int16_t* src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
  const int filterSpan = ( N - 1 );
  _mm_prefetch( (const char*)src + srcStride, _MM_HINT_T0 );
  _mm_prefetch( (const char*)src + ( width >> 1 ) + srcStride, _MM_HINT_T0 );
  _mm_prefetch( (const char*)src + width + filterSpan + srcStride, _MM_HINT_T0 );

  __m128i voffset  = _mm_set1_epi32( offset );
  __m128i vibdimin = _mm_set1_epi16( clpRng.min );
  __m128i vibdimax = _mm_set1_epi16( clpRng.max );
  __m128i vcoeffh  = _mm_lddqu_si128( ( __m128i const * )coeff );

  __m128i vshufc0, vshufc1;
  __m128i vsum, vsuma, vsumb;
#if JVET_Z0117_CHROMA_IF
  __m128i vcoeffh1;
#endif
  if( N != 8 ){
    vcoeffh = _mm_shuffle_epi32( vcoeffh, 0x44 );
    vshufc0 = _mm_set_epi8( 0x9, 0x8, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0 );
    vshufc1 = _mm_set_epi8( 0xd, 0xc, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0x5, 0x4 );
  }

#if JVET_Z0117_CHROMA_IF
  int16_t c0[8];
  int16_t c1[8];
  __m128i vshufc2;
  if(N == 6)
  {
    vshufc2 = _mm_set_epi8(  0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0 );
    for(int i = 0; i < 8; i += 4)
    {
      c0[i] = coeff[0];
      c0[i+1] = coeff[1];
      c0[i+2] = coeff[2];
      c0[i+3] = coeff[3];
      c1[i] = coeff[4];
      c1[i+1] = coeff[5];
      c1[i+2] = coeff[4];
      c1[i+3] = coeff[5];
    }
    vcoeffh = _mm_lddqu_si128( ( __m128i const * )c0 );
    vcoeffh1 = _mm_lddqu_si128( ( __m128i const * )c1 );
  }
#endif
  for( int row = 0; row < height; row++ )
  {
    _mm_prefetch( (const char*)src + 2 * srcStride, _MM_HINT_T0 );
    _mm_prefetch( (const char*)src + ( width >> 1 ) + 2 * srcStride, _MM_HINT_T0 );
    _mm_prefetch( (const char*)src + width + filterSpan + 2 * srcStride, _MM_HINT_T0 );
    for( int col = 0; col < width; col += 8 )
    {
      if( N == 8 )
      {
        __m128i vtmp[4];
        for( int i = 0; i < 8; i += 2 ){
          __m128i vsrc0 = _mm_lddqu_si128( ( __m128i const * )&src[col + i] );
          __m128i vsrc1 = _mm_lddqu_si128( ( __m128i const * )&src[col + i + 1] );
          vsrc0 = _mm_madd_epi16( vsrc0, vcoeffh );
          vsrc1 = _mm_madd_epi16( vsrc1, vcoeffh );
          vtmp[i / 2] = _mm_hadd_epi32( vsrc0, vsrc1 );
        }
        vsuma = _mm_hadd_epi32( vtmp[0], vtmp[1] );
        vsumb = _mm_hadd_epi32( vtmp[2], vtmp[3] );
      }
#if JVET_Z0117_CHROMA_IF
      else if(N == 6)
      {
        __m128i vtmp[2];
        for( int i = 0; i < 8; i += 4 )
        {
          __m128i vsrc0 = _mm_lddqu_si128( ( __m128i const * )&src[col + i] );
          __m128i vsrc1 = _mm_lddqu_si128( ( __m128i const * )&src[col + i + 4] );
          __m128i temp1 = _mm_shuffle_epi8( vsrc0, vshufc0 );
          __m128i temp2 = _mm_shuffle_epi8( vsrc0, vshufc1 );
          __m128i temp3 = _mm_shuffle_epi8( vsrc1, vshufc2 );
          temp1 = _mm_madd_epi16( temp1, vcoeffh );
          temp2 = _mm_madd_epi16( temp2, vcoeffh );
          temp3 = _mm_madd_epi16( temp3, vcoeffh1 );
          temp1 = _mm_hadd_epi32(temp1, temp2);
          vtmp[i / 4] = _mm_add_epi32(temp1, temp3);
        }
        vsuma = vtmp[0];
        vsumb = vtmp[1];
      }
#endif
      else
      {
        __m128i vtmp00, vtmp01, vtmp10, vtmp11;
        __m128i vsrc0 = _mm_lddqu_si128( ( __m128i const * )&src[col] );
        __m128i vsrc1 = _mm_lddqu_si128( ( __m128i const * )&src[col + 4] );
        vtmp00 = _mm_shuffle_epi8( vsrc0, vshufc0 );
        vtmp01 = _mm_shuffle_epi8( vsrc0, vshufc1 );
        vtmp10 = _mm_shuffle_epi8( vsrc1, vshufc0 );
        vtmp11 = _mm_shuffle_epi8( vsrc1, vshufc1 );

        vtmp00 = _mm_madd_epi16( vtmp00, vcoeffh );
        vtmp01 = _mm_madd_epi16( vtmp01, vcoeffh );
        vtmp10 = _mm_madd_epi16( vtmp10, vcoeffh );
        vtmp11 = _mm_madd_epi16( vtmp11, vcoeffh );

        vsuma = _mm_hadd_epi32( vtmp00, vtmp01 );
        vsumb = _mm_hadd_epi32( vtmp10, vtmp11 );
      }

      {
        vsuma = _mm_add_epi32( vsuma, voffset );
        vsumb = _mm_add_epi32( vsumb, voffset );

        vsuma = _mm_srai_epi32( vsuma, shift );
        vsumb = _mm_srai_epi32( vsumb, shift );

        vsum = _mm_packs_epi32( vsuma, vsumb );
      }

      if( shiftBack ){ //clip
        vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
      }
#if JEM_UNALIGNED_DST
      _mm_storeu_si128( ( __m128i * )&dst[col], vsum );
#else
      _mm_store_si128( ( __m128i * )&dst[col], vsum );
#endif
    }
    src += srcStride;
    dst += dstStride;
  }
}

#if IF_12TAP_SIMD
#if SIMD_4x4_12
template<bool isLast>
static void simdInterpolate4x4_12tap(const int16_t* src, int srcStride, int16_t *dst, int dstStride, int shiftH, int offsetH, int shiftV, int offsetV, int16_t const *coeffH, int16_t const *coeffV, int ibdimin, int ibdimax)
{
  __m128i xmm0 = _mm_lddqu_si128((__m128i const *)(coeffH)); //vcoeffh
  __m128i xmm1 = _mm_lddqu_si128((__m128i const *)(coeffH + 8)); //vcoeffl
   xmm1 = _mm_unpacklo_epi64(xmm1, xmm1);

  __m128i xmm13 = _mm_set1_epi32(offsetH);
  __m128i xmm14 = _mm_set_epi8( 9, 8, 7, 6, 5, 4, 3, 2, 7, 6, 5, 4, 3, 2, 1, 0 );
  __m128i xmm15 = _mm_set_epi8( 13, 12, 11, 10, 9, 8, 7, 6, 11, 10, 9, 8, 7, 6, 5, 4 );

  __m128i b[8]; //intermediate storage, 2x4 in line

  for (int i = 0; i < 15; i++)
  {
    __m128i xmm2 = _mm_lddqu_si128((__m128i const *)src);
    __m128i xmm3 = _mm_lddqu_si128((__m128i const *)(src + 8));

    __m128i xmm4 = _mm_shuffle_epi8(xmm3, xmm14);
    __m128i xmm5 = _mm_shuffle_epi8(xmm3, xmm15);

    __m128i xmm6 = _mm_alignr_epi8(xmm3, xmm2, 2);
    __m128i xmm7 = _mm_alignr_epi8(xmm3, xmm2, 4);
    __m128i xmm8 = _mm_alignr_epi8(xmm3, xmm2, 6);

    xmm2 = _mm_madd_epi16(xmm2, xmm0);
    xmm6 = _mm_madd_epi16(xmm6, xmm0);
    xmm7 = _mm_madd_epi16(xmm7, xmm0);
    xmm8 = _mm_madd_epi16(xmm8, xmm0);

    xmm4 = _mm_madd_epi16(xmm4, xmm1);
    xmm5 = _mm_madd_epi16(xmm5, xmm1);

    __m128i xmm9 = _mm_unpacklo_epi64(xmm2, xmm6);
    xmm2 = _mm_unpackhi_epi64(xmm2, xmm6);
    __m128i xmm10 = _mm_unpacklo_epi64(xmm7, xmm8);
    xmm7 = _mm_unpackhi_epi64(xmm7, xmm8);

    xmm2 = _mm_add_epi32(xmm2, xmm9);
    xmm7 = _mm_add_epi32(xmm7, xmm10);
    xmm2 = _mm_add_epi32(xmm2, xmm4);
    xmm7 = _mm_add_epi32(xmm7, xmm5);

    xmm2 = _mm_hadd_epi32(xmm2, xmm7);

    xmm2 = _mm_add_epi32(xmm2, xmm13);
    xmm2 = _mm_srai_epi32(xmm2, shiftH); // 4 16-bit packed to 32-bit

    if( i & 1 ) {
#if DIST_SSE_ENABLE
      xmm2 = _mm_slli_si128(xmm2, 2);
#else
      xmm2 = _mm_bslli_si128(xmm2, 2);
#endif
      b[(i-1) >> 1] = _mm_blend_epi16(b[(i-1) >> 1], xmm2, 0xAA);
    }
    else
      b[i >> 1] = xmm2;

    src += srcStride;
  }

  //vertical filter
          xmm0 = _mm_unpacklo_epi16(_mm_set1_epi16(coeffV[0]),  _mm_set1_epi16(coeffV[1]));
          xmm1 = _mm_unpacklo_epi16(_mm_set1_epi16(coeffV[2]),  _mm_set1_epi16(coeffV[3]));
  __m128i xmm2 = _mm_unpacklo_epi16(_mm_set1_epi16(coeffV[4]),  _mm_set1_epi16(coeffV[5]));
  __m128i xmm3 = _mm_unpacklo_epi16(_mm_set1_epi16(coeffV[6]),  _mm_set1_epi16(coeffV[7]));
  __m128i xmm4 = _mm_unpacklo_epi16(_mm_set1_epi16(coeffV[8]),  _mm_set1_epi16(coeffV[9]));
  __m128i xmm5 = _mm_unpacklo_epi16(_mm_set1_epi16(coeffV[10]), _mm_set1_epi16(coeffV[11]));

  __m128i d[4];

  //even
  for(int i = 0; i < 2; i++) {

    __m128i xmm6 =  _mm_madd_epi16(xmm0, b[i + 0]);
    __m128i xmm7 =  _mm_madd_epi16(xmm1, b[i + 1]);
    __m128i xmm8 =  _mm_madd_epi16(xmm2, b[i + 2]);
    __m128i xmm9 =  _mm_madd_epi16(xmm3, b[i + 3]);
    __m128i xmm10 = _mm_madd_epi16(xmm4, b[i + 4]);
    __m128i xmm11 = _mm_madd_epi16(xmm5, b[i + 5]);

    xmm6  = _mm_add_epi32(xmm6,  xmm7);
    xmm8  = _mm_add_epi32(xmm8,  xmm9);
    xmm10 = _mm_add_epi32(xmm10, xmm11);

    xmm6 = _mm_add_epi32(xmm6, xmm8);
    d[i << 1] = _mm_add_epi32(xmm6, xmm10);
  }

  //odd
  __m128i xmm12 = _mm_set_epi8( 1, 0, 3, 2, 1, 0, 3, 2, 1, 0, 3, 2, 1, 0, 3, 2 );

  xmm0 = _mm_shuffle_epi8(xmm0, xmm12);
  xmm1 = _mm_shuffle_epi8(xmm1, xmm12);
  xmm2 = _mm_shuffle_epi8(xmm2, xmm12);
  xmm3 = _mm_shuffle_epi8(xmm3, xmm12);
  xmm4 = _mm_shuffle_epi8(xmm4, xmm12);
  xmm5 = _mm_shuffle_epi8(xmm5, xmm12);

  for(int j = 0; j < 7; j++) {
    b[j] = _mm_blend_epi16(b[j], b[j+1], 0x55);
  }

  for(int i = 0; i < 2; i++) {

    __m128i xmm6 =  _mm_madd_epi16(xmm0, b[i + 0]);
    __m128i xmm7 =  _mm_madd_epi16(xmm1, b[i + 1]);
    __m128i xmm8 =  _mm_madd_epi16(xmm2, b[i + 2]);
    __m128i xmm9 =  _mm_madd_epi16(xmm3, b[i + 3]);
    __m128i xmm10 = _mm_madd_epi16(xmm4, b[i + 4]);
    __m128i xmm11 = _mm_madd_epi16(xmm5, b[i + 5]);

    xmm6  = _mm_add_epi32(xmm6,  xmm7);
    xmm8  = _mm_add_epi32(xmm8,  xmm9);
    xmm10 = _mm_add_epi32(xmm10, xmm11);

    xmm6 = _mm_add_epi32(xmm6, xmm8);
    d[(i << 1) + 1] = _mm_add_epi32(xmm6, xmm10);
  }

  //pack and save to destination
  xmm13  = _mm_set1_epi32(offsetV);
  xmm12 = _mm_setzero_si128();

  __m128i vibdimin, vibdimax;
  if( isLast ) {
    vibdimin = _mm_set1_epi16(ibdimin);
    vibdimax = _mm_set1_epi16(ibdimax);
  }

  for(int i=0; i < 4; i++) {

    d[i] = _mm_add_epi32(d[i], xmm13);
    d[i] = _mm_srai_epi32(d[i], shiftV);
    d[i] = _mm_packs_epi32(d[i], xmm12);

    if( isLast )
        d[i] = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, d[i]));

    _mm_storel_epi64((__m128i*)dst, d[i]);

    dst += dstStride;
  }

}
#endif
// SIMD interpolation horizontal, block width modulo 4
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateHorM4_12tap(const int16_t* src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff)
{
  _mm_prefetch((const char*)src + srcStride, _MM_HINT_T0);

  const __m128i voffset = _mm_set1_epi32(offset);
  const __m128i vibdimin = _mm_set1_epi16(clpRng.min);
  const __m128i vibdimax = _mm_set1_epi16(clpRng.max);
  const __m128i vcoeffh = _mm_lddqu_si128((__m128i const *)coeff);
  const __m128i vcoeffl = _mm_lddqu_si128((__m128i const *)(coeff + 8));

  const __m128i vshuf0 = _mm_set_epi8( 9, 8, 7, 6, 5, 4, 3, 2, 7, 6, 5, 4, 3, 2, 1, 0 );
  const __m128i vshuf1 = _mm_set_epi8( 13, 12, 11, 10, 9, 8, 7, 6, 11, 10, 9, 8, 7, 6, 5, 4 );

  for (int row = 0; row < height; row++)
  {
    _mm_prefetch((const char*)src + 2 * srcStride, _MM_HINT_T0);

    for (int col = 0; col < width; col += 4)
    {
       __m128i vsum  = _mm_setzero_si128();

      __m128i vsrc0 = _mm_lddqu_si128((__m128i const *)&src[col]);
       const __m128i vsrc4 = _mm_lddqu_si128((__m128i const *)&src[col + 8]);

      __m128i vtmp0 = _mm_shuffle_epi8(vsrc4, vshuf0);
      __m128i vtmp1 = _mm_shuffle_epi8(vsrc4, vshuf1);

      __m128i vsrc1 = _mm_alignr_epi8(vsrc4, vsrc0, 2);
      __m128i vsrc2 = _mm_alignr_epi8(vsrc4, vsrc0, 4); 
      __m128i vsrc3 = _mm_alignr_epi8(vsrc4, vsrc0, 6);

        vsrc0 = _mm_madd_epi16(vsrc0, vcoeffh);
        vsrc1 = _mm_madd_epi16(vsrc1, vcoeffh);
        vsrc2 = _mm_madd_epi16(vsrc2, vcoeffh);
        vsrc3 = _mm_madd_epi16(vsrc3, vcoeffh);

        vtmp0 = _mm_madd_epi16(vtmp0, vcoeffl);
        vtmp1 = _mm_madd_epi16(vtmp1, vcoeffl);

         __m128i vtmp2 = _mm_unpacklo_epi64(vsrc0, vsrc1);
         vsrc0 = _mm_unpackhi_epi64(vsrc0, vsrc1);
         __m128i vtmp3 = _mm_unpacklo_epi64(vsrc2, vsrc3);
         vsrc2 = _mm_unpackhi_epi64(vsrc2, vsrc3);

         vsrc0 = _mm_add_epi32(vsrc0, vtmp2);
         vsrc2 = _mm_add_epi32(vsrc2, vtmp3);
         vsrc0 = _mm_add_epi32(vsrc0, vtmp0);
         vsrc2 = _mm_add_epi32(vsrc2, vtmp1);

         vsum = _mm_hadd_epi32(vsrc0, vsrc2);
      {
        vsum = _mm_add_epi32(vsum, voffset);
        vsum = _mm_srai_epi32(vsum, shift);

        const __m128i vzero = _mm_setzero_si128();
        vsum = _mm_packs_epi32(vsum, vzero);
      }

      if (shiftBack)
      { //clip
        vsum = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsum));
      }
      _mm_storel_epi64((__m128i *)&dst[col], vsum);
    }
    src += srcStride;
    dst += dstStride;
  }
}
#if MCIF_SIMD_NEW
// SIMD interpolation horizontal, block not modulo of 4
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateHorNonM4_12tap(const int16_t* src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff)
{
  _mm_prefetch((const char*)src + srcStride, _MM_HINT_T0);

  const __m128i voffset = _mm_set1_epi32(offset);
  const __m128i vibdimin = _mm_set1_epi16(clpRng.min);
  const __m128i vibdimax = _mm_set1_epi16(clpRng.max);
  const __m128i vcoeffh = _mm_lddqu_si128((__m128i const *)coeff);
  const __m128i vcoeffl = _mm_lddqu_si128((__m128i const *)(coeff + 8));

  const __m128i vshuf0 = _mm_set_epi8(9, 8, 7, 6, 5, 4, 3, 2, 7, 6, 5, 4, 3, 2, 1, 0);
  const __m128i vshuf1 = _mm_set_epi8(13, 12, 11, 10, 9, 8, 7, 6, 11, 10, 9, 8, 7, 6, 5, 4);

  int multiple = (width >> 2) << 2;
  const int16_t* initialSrc = src;
  int16_t* initialDst = dst;

  for (int row = 0; row < height; row++)
  {
    _mm_prefetch((const char*)src + 2 * srcStride, _MM_HINT_T0);

    for (int col = 0; col < multiple; col += 4)
    {
      __m128i vsum = _mm_setzero_si128();

      __m128i vsrc0 = _mm_lddqu_si128((__m128i const *)&src[col]);
      const __m128i vsrc4 = _mm_lddqu_si128((__m128i const *)&src[col + 8]);

      __m128i vtmp0 = _mm_shuffle_epi8(vsrc4, vshuf0);
      __m128i vtmp1 = _mm_shuffle_epi8(vsrc4, vshuf1);

      __m128i vsrc1 = _mm_alignr_epi8(vsrc4, vsrc0, 2);
      __m128i vsrc2 = _mm_alignr_epi8(vsrc4, vsrc0, 4);
      __m128i vsrc3 = _mm_alignr_epi8(vsrc4, vsrc0, 6);

      vsrc0 = _mm_madd_epi16(vsrc0, vcoeffh);
      vsrc1 = _mm_madd_epi16(vsrc1, vcoeffh);
      vsrc2 = _mm_madd_epi16(vsrc2, vcoeffh);
      vsrc3 = _mm_madd_epi16(vsrc3, vcoeffh);

      vtmp0 = _mm_madd_epi16(vtmp0, vcoeffl);
      vtmp1 = _mm_madd_epi16(vtmp1, vcoeffl);

      __m128i vtmp2 = _mm_unpacklo_epi64(vsrc0, vsrc1);
      vsrc0 = _mm_unpackhi_epi64(vsrc0, vsrc1);
      __m128i vtmp3 = _mm_unpacklo_epi64(vsrc2, vsrc3);
      vsrc2 = _mm_unpackhi_epi64(vsrc2, vsrc3);

      vsrc0 = _mm_add_epi32(vsrc0, vtmp2);
      vsrc2 = _mm_add_epi32(vsrc2, vtmp3);
      vsrc0 = _mm_add_epi32(vsrc0, vtmp0);
      vsrc2 = _mm_add_epi32(vsrc2, vtmp1);

      vsum = _mm_hadd_epi32(vsrc0, vsrc2);
      {
        vsum = _mm_add_epi32(vsum, voffset);
        vsum = _mm_srai_epi32(vsum, shift);

        const __m128i vzero = _mm_setzero_si128();
        vsum = _mm_packs_epi32(vsum, vzero);
      }

      if (shiftBack)
      { //clip
        vsum = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsum));
      }
      _mm_storel_epi64((__m128i *)&dst[col], vsum);
    }
    src += srcStride;
    dst += dstStride;
  }

  for (int row = 0; row < height; row++)
  {
    for (int col = multiple; col < width; col++)
    {
      int sum = 0;

      sum = initialSrc[col] * coeff[0];
      sum += initialSrc[col + 1] * coeff[1];
      sum += initialSrc[col + 2] * coeff[2];
      sum += initialSrc[col + 3] * coeff[3];
      sum += initialSrc[col + 4] * coeff[4];
      sum += initialSrc[col + 5] * coeff[5];
      sum += initialSrc[col + 6] * coeff[6];
      sum += initialSrc[col + 7] * coeff[7];
      sum += initialSrc[col + 8] * coeff[8];
      sum += initialSrc[col + 9] * coeff[9];
      sum += initialSrc[col + 10] * coeff[10];
      sum += initialSrc[col + 11] * coeff[11];

      Pel val = (sum + offset) >> shift;
      if (shiftBack)
      {
        val = ClipPel(val, clpRng);
      }
      initialDst[col] = val;
    }

    initialSrc += srcStride;
    initialDst += dstStride;
  }
}
#endif
#endif
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateHorM8_AVX2( const int16_t* src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
#ifdef USE_AVX2
  const int filterSpan =/* bChromaIntl ? 2* ( N-1 ) : */( N-1 );
  _mm_prefetch( (const char*)( src+srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+( width>>1 )+srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+width+filterSpan+srcStride ), _MM_HINT_T0 );

  __m256i voffset    = _mm256_set1_epi32( offset );
  __m128i vibdimin   = _mm_set1_epi16( clpRng.min );
  __m128i vibdimax   = _mm_set1_epi16( clpRng.max );

  __m256i vshuf0 = _mm256_set_epi8( 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0,
                                    0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0 );
  __m256i vshuf1 = _mm256_set_epi8( 0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4,
                                    0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4 );

#if JVET_Z0117_CHROMA_IF
  __m256i vcoeff[6];
  int16_t c0[8];
  int16_t c1[8];
  if(N == 6)
  {
    for(int i = 0; i < 8; i += 4)
    {
      c0[i] = coeff[0];
      c0[i+1] = coeff[1];
      c0[i+2] = coeff[2];
      c0[i+3] = coeff[3];
      c1[i] = coeff[4];
      c1[i+1] = coeff[5];
      c1[i+2] = coeff[4];
      c1[i+3] = coeff[5];
    }
    __m128i vcoeff0 = _mm_lddqu_si128( ( __m128i const * )c0 );
    __m128i vcoeff1 = _mm_lddqu_si128( ( __m128i const * )c1 );
    vcoeff[0] = _mm256_insertf128_si256(_mm256_castsi128_si256(vcoeff0), (vcoeff0), 1);
    vcoeff[1] = _mm256_insertf128_si256(_mm256_castsi128_si256(vcoeff1), (vcoeff1), 1);
  }
  else
  {
#else
  __m256i vcoeff[N/2];
#endif
  for( int i=0; i<N; i+=2 )
  {
    vcoeff[i/2] = _mm256_unpacklo_epi16( _mm256_set1_epi16( coeff[i] ), _mm256_set1_epi16( coeff[i+1] ) );
  }
#if JVET_Z0117_CHROMA_IF
  }
#endif

//   __m256i vw0, vw1, voffsetW, vwp;
  __m256i vsum;

  for( int row = 0; row < height; row++ )
  {
    _mm_prefetch( (const char*)( src+2*srcStride ), _MM_HINT_T0 );
    _mm_prefetch( (const char*)( src+( width>>1 )+2*srcStride ), _MM_HINT_T0 );
    _mm_prefetch( (const char*)( src+width+filterSpan + 2*srcStride ), _MM_HINT_T0 );
    for( int col = 0; col < width; col+=8 )
    {
      if( N==8 )
      {
        __m128i vsrc[3];
        for( int i=0; i<3; i++ )
        {
          vsrc[i] = _mm_loadu_si128( ( const __m128i * )&src[col+i*4] );
        }
        vsum = _mm256_setzero_si256();
        for( int i=0; i<2; i++ )
        {
          __m256i vsrc0 = _mm256_castsi128_si256( vsrc[i] );
          vsrc0 = _mm256_inserti128_si256( vsrc0, vsrc[i+1], 1 );
          __m256i vsrca0 = _mm256_shuffle_epi8( vsrc0, vshuf0 );
          __m256i vsrca1 = _mm256_shuffle_epi8( vsrc0, vshuf1 );
          vsum  = _mm256_add_epi32( vsum, _mm256_add_epi32( _mm256_madd_epi16( vsrca0, vcoeff[2*i] ), _mm256_madd_epi16( vsrca1, vcoeff[2*i+1] ) ) );
        }
      }
#if JVET_Z0117_CHROMA_IF
      else if( N==6 )
      {
        __m128i vshuf2 = _mm_set_epi8(0x9, 0x8, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2,0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0 );
        __m128i vshuf4 = _mm_set_epi8(   0x9, 0x8, 0x7, 0x6,
                                         0x7, 0x6, 0x5, 0x4,
                                         0x5, 0x4, 0x3, 0x2,
                                         0x3, 0x2, 0x1, 0x0 );
        __m128i vshuf3 = _mm_set_epi8(0xd, 0xc, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6,0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0x5, 0x4);
        __m128i t0 = _mm_lddqu_si128( ( const __m128i* )&src[col] );
        __m128i t2 = _mm_lddqu_si128( ( const __m128i* )&src[col+8]);
        __m128i t1 = _mm_alignr_epi8(t2, t0, 8);
        __m128i b0 = _mm_shuffle_epi8(t1, vshuf4);
        __m128i b1 = _mm_shuffle_epi8(t2, vshuf4);
        __m128i c0 = _mm_shuffle_epi8(t0, vshuf2);
        __m128i c1 = _mm_shuffle_epi8(t1, vshuf2);
        __m128i d0 = _mm_shuffle_epi8(t0, vshuf3);
        __m128i d1 = _mm_shuffle_epi8(t1, vshuf3);
        __m256i vsrca0 = _mm256_insertf128_si256(_mm256_castsi128_si256(c0), (c1), 1);
        __m256i vsrca1 = _mm256_insertf128_si256(_mm256_castsi128_si256(d0), (d1), 1);
        __m256i vsrca2 = _mm256_insertf128_si256(_mm256_castsi128_si256(b0), (b1), 1);
        vsrca0 = _mm256_madd_epi16(vsrca0, vcoeff[0]);
        vsrca1 = _mm256_madd_epi16(vsrca1, vcoeff[0]);
        vsrca2 = _mm256_madd_epi16(vsrca2, vcoeff[1]);
        vsrca0 = _mm256_hadd_epi32(vsrca0, vsrca1);
        vsum = _mm256_add_epi32(vsrca0, vsrca2);
      }
#endif
      else
      {
        __m256i vtmp02, vtmp13;
        {
          __m256i vsrc = _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i* )&src[col] ) );
          vsrc = _mm256_inserti128_si256( vsrc, _mm_loadu_si128( ( const __m128i* )&src[col+4] ), 1 );

          vtmp02 = _mm256_shuffle_epi8( vsrc, vshuf0 );
          vtmp13 = _mm256_shuffle_epi8( vsrc, vshuf1 );
        }
        vtmp02 = _mm256_madd_epi16( vtmp02, vcoeff[0] );
        vtmp13 = _mm256_madd_epi16( vtmp13, vcoeff[1] );
        vsum = _mm256_add_epi32( vtmp02, vtmp13 );
      }
      {
        vsum = _mm256_add_epi32( vsum, voffset );
        vsum = _mm256_srai_epi32( vsum, shift );
      }

      __m128i vsump = _mm256_castsi256_si128( _mm256_permute4x64_epi64( _mm256_packs_epi32( vsum, vsum ), 0x88 ) );
      if( shiftBack )
      { //clip
        vsump = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsump ) );
      }
#if JEM_UNALIGNED_DST
      _mm_storeu_si128( ( __m128i * )&dst[col], vsump );
#else
      _mm_store_si128( ( __m128i * )&dst[col], vsump );
#endif
    }
    src += srcStride;
    dst += dstStride;
  }
#endif
}
#if JVET_Z0117_CHROMA_IF
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateHorM8_sym_AVX2( const int16_t* src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
#ifdef USE_AVX2
  const int filterSpan =/* bChromaIntl ? 2* ( N-1 ) : */( N-1 );
  _mm_prefetch( (const char*)( src+srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+( width>>1 )+srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+width+filterSpan+srcStride ), _MM_HINT_T0 );
  __m256i voffset    = _mm256_set1_epi32( offset );
  __m128i vibdimin   = _mm_set1_epi16( clpRng.min );
  __m128i vibdimax   = _mm_set1_epi16( clpRng.max );
  __m256i vcoeff[2];
  int16_t c0[8];
  if(N == 6)
  {
    for(int i = 0; i < 8; i += 4)
    {
      c0[i] = coeff[0];
      c0[i+1] = coeff[1];
      c0[i+2] = coeff[0];
      c0[i+3] = coeff[1];
      
    }
    __m128i vcoeff0 = _mm_lddqu_si128( ( __m128i const * )c0 );
    vcoeff[0] = _mm256_insertf128_si256(_mm256_castsi128_si256(vcoeff0), (vcoeff0), 1);
    vcoeff[1] = _mm256_set1_epi32(coeff[2]);
    
  }

  __m256i vsum;
  for( int row = 0; row < height; row++ )
  {
    _mm_prefetch( (const char*)( src+2*srcStride ), _MM_HINT_T0 );
    _mm_prefetch( (const char*)( src+( width>>1 )+2*srcStride ), _MM_HINT_T0 );
    _mm_prefetch( (const char*)( src+width+filterSpan + 2*srcStride ), _MM_HINT_T0 );
    for( int col = 0; col < width; col+=8 )
    {
      if( N==6 )
      {
        __m128i t0 = _mm_lddqu_si128( ( const __m128i* )&src[col] );
        __m128i t6 = _mm_lddqu_si128( ( const __m128i* )&src[col+8]);
        __m128i t1 = _mm_alignr_epi8(t6, t0, 2);
        __m128i t2 = _mm_alignr_epi8(t6, t0, 4);
        __m128i t3 = _mm_alignr_epi8(t6, t0, 6);
        __m128i t4 = _mm_alignr_epi8(t6, t0, 8);
        __m128i t5 = _mm_alignr_epi8(t6, t0, 10);
        t0 = _mm_add_epi16(t5, t0);
        t1 = _mm_add_epi16(t4, t1);
        t2 = _mm_add_epi16(t3, t2);
        t3 = _mm_unpacklo_epi16(t0, t1);
        t4 = _mm_unpackhi_epi16(t0, t1);
        __m256i vsrca0 = _mm256_insertf128_si256(_mm256_castsi128_si256(t3), (t4), 1);
        vsrca0 = _mm256_madd_epi16(vsrca0, vcoeff[0]);
        __m256i vsrca1 = _mm256_cvtepi16_epi32(t2);
        vsrca1 = _mm256_mullo_epi32(vsrca1, vcoeff[1]);
        vsum = _mm256_add_epi32(vsrca0, vsrca1 );
      }

      vsum = _mm256_add_epi32( vsum, voffset );
      vsum = _mm256_srai_epi32( vsum, shift );

      __m128i vsump = _mm256_castsi256_si128( _mm256_permute4x64_epi64( _mm256_packs_epi32( vsum, vsum ), 0x88 ) );
      if( shiftBack )
      { //clip
        vsump = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsump ) );
      }
#if JEM_UNALIGNED_DST
      _mm_storeu_si128( ( __m128i * )&dst[col], vsump );
#else
      _mm_store_si128( ( __m128i * )&dst[col], vsump );
#endif
    }
    src += srcStride;
    dst += dstStride;
  }
#endif
}
#endif
#if MCIF_SIMD_NEW
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateHorM16_AVX2(const int16_t* src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff)
{
#ifdef USE_AVX2
  const int filterSpan = (N - 1);
  _mm_prefetch((const char*)(src + srcStride), _MM_HINT_T0);
  _mm_prefetch((const char*)(src + (width >> 1) + srcStride), _MM_HINT_T0);
  _mm_prefetch((const char*)(src + width + filterSpan + srcStride), _MM_HINT_T0);
  _mm_prefetch((const char*)(src + 2 * srcStride), _MM_HINT_T0);
  _mm_prefetch((const char*)(src + (width >> 1) + 2 * srcStride), _MM_HINT_T0);
  _mm_prefetch((const char*)(src + width + filterSpan + 2 * srcStride), _MM_HINT_T0);
  _mm_prefetch((const char*)(src + 3 * srcStride), _MM_HINT_T0);
  _mm_prefetch((const char*)(src + (width >> 1) + 3 * srcStride), _MM_HINT_T0);
  _mm_prefetch((const char*)(src + width + filterSpan + 3 * srcStride), _MM_HINT_T0);

  __m256i voffset = _mm256_set1_epi32(offset);
  __m256i vibdimin = _mm256_set1_epi16(clpRng.min);
  __m256i vibdimax = _mm256_set1_epi16(clpRng.max);
  __m256i vzero = _mm256_setzero_si256();
  __m256i vsum, vsuma, vsumb;

  __m256i vshuf0 = _mm256_set_epi8(0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0,
    0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0);
  __m256i vshuf1 = _mm256_set_epi8(0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4,
    0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4);
#if __INTEL_COMPILER
  __m256i vcoeff[4];
#else
#if JVET_Z0117_CHROMA_IF
  __m256i vcoeff[6];
#else
  __m256i vcoeff[N / 2];
#endif
#endif
#if JVET_Z0117_CHROMA_IF
  int16_t c0[8];
  int16_t c1[8];
  if(N == 6)
  {
    for(int i = 0; i < 8; i += 4)
    {
      c0[i] = coeff[0];
      c0[i+1] = coeff[1];
      c0[i+2] = coeff[2];
      c0[i+3] = coeff[3];
      c1[i] = coeff[4];
      c1[i+1] = coeff[5];
      c1[i+2] = coeff[4];
      c1[i+3] = coeff[5];
    }
    __m128i vcoeff0 = _mm_lddqu_si128( ( __m128i const * )c0 );
    __m128i vcoeff1 = _mm_lddqu_si128( ( __m128i const * )c1 );
    vcoeff[0] = _mm256_insertf128_si256(_mm256_castsi128_si256(vcoeff0), (vcoeff0), 1);
    vcoeff[1] = _mm256_insertf128_si256(_mm256_castsi128_si256(vcoeff1), (vcoeff1), 1);
  }
  else
  {
#endif
  for (int i = 0; i < N; i += 2)
  {
    vcoeff[i / 2] = _mm256_unpacklo_epi16(_mm256_set1_epi16(coeff[i]), _mm256_set1_epi16(coeff[i + 1]));
  }
#if JVET_Z0117_CHROMA_IF
  }
#endif

  for (int row = 0; row < height; row++)
  {
    _mm_prefetch((const char*)(src + 2 * srcStride), _MM_HINT_T0);
    _mm_prefetch((const char*)(src + (width >> 1) + 2 * srcStride), _MM_HINT_T0);
    _mm_prefetch((const char*)(src + width + filterSpan + 2 * srcStride), _MM_HINT_T0);

    for (int col = 0; col < width; col += 16)
    {
      if (N == 8)
      {
        __m256i vsrca0, vsrca1, vsrcb0, vsrcb1;
        __m256i vsrc0 = _mm256_loadu_si256((const __m256i *)&src[col]);
        __m256i vsrc1 = _mm256_loadu_si256((const __m256i *)&src[col + 4]);

        vsuma = vsumb = vzero;

        vsrca0 = _mm256_shuffle_epi8(vsrc0, vshuf0);
        vsrca1 = _mm256_shuffle_epi8(vsrc0, vshuf1);
        vsrc0 = _mm256_loadu_si256((const __m256i *)&src[col + 8]);
        vsuma = _mm256_add_epi32(_mm256_madd_epi16(vsrca0, vcoeff[0]), _mm256_madd_epi16(vsrca1, vcoeff[1]));
        vsrcb0 = _mm256_shuffle_epi8(vsrc1, vshuf0);
        vsrcb1 = _mm256_shuffle_epi8(vsrc1, vshuf1);
        vsumb = _mm256_add_epi32(_mm256_madd_epi16(vsrcb0, vcoeff[0]), _mm256_madd_epi16(vsrcb1, vcoeff[1]));
        vsrc1 = _mm256_add_epi32(_mm256_madd_epi16(vsrcb0, vcoeff[2]), _mm256_madd_epi16(vsrcb1, vcoeff[3]));
        vsrca0 = _mm256_shuffle_epi8(vsrc0, vshuf0);
        vsrca1 = _mm256_shuffle_epi8(vsrc0, vshuf1);
        vsrc0 = _mm256_add_epi32(_mm256_madd_epi16(vsrca0, vcoeff[2]), _mm256_madd_epi16(vsrca1, vcoeff[3]));
        vsuma = _mm256_add_epi32(vsuma, vsrc1);
        vsumb = _mm256_add_epi32(vsumb, vsrc0);
      }
#if JVET_Z0117_CHROMA_IF
      else if( N == 6 )
      {
        __m128i vshuf2 = _mm_set_epi8(0x9, 0x8, 0x7, 0x6, 0x5, 0x4, 0x3, 0x2,0x7, 0x6, 0x5, 0x4, 0x3, 0x2, 0x1, 0x0 );
        __m128i vshuf4 = _mm_set_epi8(   0x9, 0x8, 0x7, 0x6,
                                         0x7, 0x6, 0x5, 0x4,
                                         0x5, 0x4, 0x3, 0x2,
                                         0x3, 0x2, 0x1, 0x0 );
        __m128i vshuf3 = _mm_set_epi8(0xd, 0xc, 0xb, 0xa, 0x9, 0x8, 0x7, 0x6,0xb, 0xa, 0x9, 0x8, 0x7, 0x6, 0x5, 0x4);
        vsuma = vsumb = vzero;
        __m128i t0 = _mm_lddqu_si128( ( const __m128i* )&src[col] );
        __m128i t2 = _mm_lddqu_si128( ( const __m128i* )&src[col+8]);
        __m128i t4 = _mm_lddqu_si128( ( const __m128i* )&src[col+16]);
        __m128i t1, t3, b0, b1, b2, b3, c0, c1, c2, c3, d0, d1, d2, d3;
        __m256i vsrca0, vsrca1, vsrca2;
        t1 = _mm_alignr_epi8(t2, t0, 8);
        t3 = _mm_alignr_epi8(t4, t2, 8);
        b0 = _mm_shuffle_epi8(t1, vshuf4);
        b1 = _mm_shuffle_epi8(t2, vshuf4);
        c0 = _mm_shuffle_epi8(t0, vshuf2);
        c1 = _mm_shuffle_epi8(t1, vshuf2);
        d0 = _mm_shuffle_epi8(t0, vshuf3);
        d1 = _mm_shuffle_epi8(t1, vshuf3);
        b2 = _mm_shuffle_epi8(t3, vshuf4);
        b3 = _mm_shuffle_epi8(t4, vshuf4);
        c2 = _mm_shuffle_epi8(t2, vshuf2);
        c3 = _mm_shuffle_epi8(t3, vshuf2);
        d2 = _mm_shuffle_epi8(t2, vshuf3);
        d3 = _mm_shuffle_epi8(t3, vshuf3);
        vsrca0 = _mm256_insertf128_si256(_mm256_castsi128_si256(c0), (c2), 1);
        vsrca1 = _mm256_insertf128_si256(_mm256_castsi128_si256(d0), (d2), 1);
        vsrca2 = _mm256_insertf128_si256(_mm256_castsi128_si256(b0), (b2), 1);
        vsrca0 = _mm256_madd_epi16(vsrca0, vcoeff[0]);
        vsrca1 = _mm256_madd_epi16(vsrca1, vcoeff[0]);
        vsrca2 = _mm256_madd_epi16(vsrca2, vcoeff[1]);
        vsrca0 = _mm256_hadd_epi32(vsrca0, vsrca1);
        vsuma = _mm256_add_epi32(vsrca0, vsrca2);
        vsrca0 = _mm256_insertf128_si256(_mm256_castsi128_si256(c1), (c3), 1);
        vsrca1 = _mm256_insertf128_si256(_mm256_castsi128_si256(d1), (d3), 1);
        vsrca2 = _mm256_insertf128_si256(_mm256_castsi128_si256(b1), (b3), 1);
        vsrca0 = _mm256_madd_epi16(vsrca0, vcoeff[0]);
        vsrca1 = _mm256_madd_epi16(vsrca1, vcoeff[0]);
        vsrca2 = _mm256_madd_epi16(vsrca2, vcoeff[1]);
        vsrca0 = _mm256_hadd_epi32(vsrca0, vsrca1);
        vsumb = _mm256_add_epi32(vsrca0, vsrca2);
      }
#endif
      else
      {
        __m256i vsrc0 = _mm256_loadu_si256((const __m256i *)&src[col]);
        __m256i vsrc1 = _mm256_loadu_si256((const __m256i *)&src[col + 4]);

        __m256i vtmp00, vtmp01, vtmp10, vtmp11;

        vtmp00 = _mm256_shuffle_epi8(vsrc0, vshuf0);
        vtmp01 = _mm256_shuffle_epi8(vsrc0, vshuf1);
        vtmp10 = _mm256_shuffle_epi8(vsrc1, vshuf0);
        vtmp11 = _mm256_shuffle_epi8(vsrc1, vshuf1);

        vtmp00 = _mm256_madd_epi16(vtmp00, vcoeff[0]);
        vtmp01 = _mm256_madd_epi16(vtmp01, vcoeff[1]);
        vtmp10 = _mm256_madd_epi16(vtmp10, vcoeff[0]);
        vtmp11 = _mm256_madd_epi16(vtmp11, vcoeff[1]);

        vsuma = _mm256_add_epi32(vtmp00, vtmp01);
        vsumb = _mm256_add_epi32(vtmp10, vtmp11);
      }

      vsuma = _mm256_add_epi32(vsuma, voffset);
      vsumb = _mm256_add_epi32(vsumb, voffset);
      vsuma = _mm256_srai_epi32(vsuma, shift);
      vsumb = _mm256_srai_epi32(vsumb, shift);
      vsum = _mm256_packs_epi32(vsuma, vsumb);

      if (shiftBack)
      { //clip
        vsum = _mm256_min_epi16(vibdimax, _mm256_max_epi16(vibdimin, vsum));
      }
#if JEM_UNALIGNED_DST
      _mm256_storeu_si256((__m256i *)&dst[col], vsum);
#else
      _mm256_store_si256((__m256i *)&dst[col], vsum);
#endif
}
    src += srcStride;
    dst += dstStride;
  }
#endif
#if USE_AVX2

  _mm256_zeroupper();
#endif
}
#if JVET_Z0117_CHROMA_IF
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateHorM16_sym_AVX2(const int16_t* src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff)
{
#ifdef USE_AVX2
  const int filterSpan = (N - 1);
  _mm_prefetch((const char*)(src + srcStride), _MM_HINT_T0);
  _mm_prefetch((const char*)(src + (width >> 1) + srcStride), _MM_HINT_T0);
  _mm_prefetch((const char*)(src + width + filterSpan + srcStride), _MM_HINT_T0);
  _mm_prefetch((const char*)(src + 2 * srcStride), _MM_HINT_T0);
  _mm_prefetch((const char*)(src + (width >> 1) + 2 * srcStride), _MM_HINT_T0);
  _mm_prefetch((const char*)(src + width + filterSpan + 2 * srcStride), _MM_HINT_T0);
  _mm_prefetch((const char*)(src + 3 * srcStride), _MM_HINT_T0);
  _mm_prefetch((const char*)(src + (width >> 1) + 3 * srcStride), _MM_HINT_T0);
  _mm_prefetch((const char*)(src + width + filterSpan + 3 * srcStride), _MM_HINT_T0);
  __m256i voffset = _mm256_set1_epi32(offset);
  __m256i vibdimin = _mm256_set1_epi16(clpRng.min);
  __m256i vibdimax = _mm256_set1_epi16(clpRng.max);
  __m256i vzero = _mm256_setzero_si256();
  __m256i vsum, vsuma, vsumb;
#if __INTEL_COMPILER
  __m256i vcoeff[4];
#else
  __m256i vcoeff[6];
#endif
  int16_t c0[8];
  for(int i = 0; i < 8; i += 4)
  {
    c0[i] = coeff[0];
    c0[i+1] = coeff[1];
    c0[i+2] = coeff[0];
    c0[i+3] = coeff[1];
    
  }
  __m128i vcoeff0 = _mm_lddqu_si128( ( __m128i const * )c0 );
  vcoeff[0] = _mm256_insertf128_si256(_mm256_castsi128_si256(vcoeff0), (vcoeff0), 1);
  vcoeff[1] = _mm256_set1_epi32(coeff[2]);
  for (int row = 0; row < height; row++)
  {
    _mm_prefetch((const char*)(src + 2 * srcStride), _MM_HINT_T0);
    _mm_prefetch((const char*)(src + (width >> 1) + 2 * srcStride), _MM_HINT_T0);
    _mm_prefetch((const char*)(src + width + filterSpan + 2 * srcStride), _MM_HINT_T0);
    for (int col = 0; col < width; col += 16)
    {
      vsuma = vsumb = vzero;
      __m128i t0 = _mm_lddqu_si128( ( const __m128i* )&src[col] );
      __m128i t6 = _mm_lddqu_si128( ( const __m128i* )&src[col+8]);
      __m128i t7 = _mm_lddqu_si128( ( const __m128i* )&src[col+16]);
      __m128i t1 = _mm_alignr_epi8(t6, t0, 2);
      __m128i t2 = _mm_alignr_epi8(t6, t0, 4);
      __m128i t3 = _mm_alignr_epi8(t6, t0, 6);
      __m128i t4 = _mm_alignr_epi8(t6, t0, 8);
      __m128i t5 = _mm_alignr_epi8(t6, t0, 10);
      t0 = _mm_add_epi16(t5, t0);
      t1 = _mm_add_epi16(t4, t1);
      t2 = _mm_add_epi16(t3, t2);
      t3 = _mm_unpacklo_epi16(t0, t1);
      t4 = _mm_unpackhi_epi16(t0, t1);
      __m256i vsrca0 = _mm256_insertf128_si256(_mm256_castsi128_si256(t3), (t4), 1);
      vsrca0 = _mm256_madd_epi16(vsrca0, vcoeff[0]);
      __m256i vsrca1 = _mm256_cvtepi16_epi32(t2);
      vsrca1 = _mm256_mullo_epi32(vsrca1, vcoeff[1]);
      __m256i vsumatmp = _mm256_add_epi32(vsrca0, vsrca1 );

      t1 = _mm_alignr_epi8(t7, t6, 2);
      t2 = _mm_alignr_epi8(t7, t6, 4);
      t3 = _mm_alignr_epi8(t7, t6, 6);
      t4 = _mm_alignr_epi8(t7, t6, 8);
      t5 = _mm_alignr_epi8(t7, t6, 10);
      t0 = _mm_add_epi16(t5, t6);
      t1 = _mm_add_epi16(t4, t1);
      t2 = _mm_add_epi16(t3, t2);
      t3 = _mm_unpacklo_epi16(t0, t1);
      t4 = _mm_unpackhi_epi16(t0, t1);
      vsrca0 = _mm256_insertf128_si256(_mm256_castsi128_si256(t3), (t4), 1);
      vsrca0 = _mm256_madd_epi16(vsrca0, vcoeff[0]);

      vsrca1 = _mm256_cvtepi16_epi32(t2);
      vsrca1 = _mm256_mullo_epi32(vsrca1, vcoeff[1]);
      __m256i vsumbtmp = _mm256_add_epi32(vsrca0, vsrca1 );
      vsuma = _mm256_permute2f128_si256(vsumatmp, vsumbtmp, 0x20);
      vsumb = _mm256_permute2f128_si256(vsumatmp, vsumbtmp, 0x31);
        
      vsuma = _mm256_add_epi32(vsuma, voffset);
      vsumb = _mm256_add_epi32(vsumb, voffset);
      vsuma = _mm256_srai_epi32(vsuma, shift);
      vsumb = _mm256_srai_epi32(vsumb, shift);
      vsum = _mm256_packs_epi32(vsuma, vsumb);

      if (shiftBack)
      { //clip
        vsum = _mm256_min_epi16(vibdimax, _mm256_max_epi16(vibdimin, vsum));
      }
#if JEM_UNALIGNED_DST
      _mm256_storeu_si256((__m256i *)&dst[col], vsum);
#else
      _mm256_store_si256((__m256i *)&dst[col], vsum);
#endif
    }
    src += srcStride;
    dst += dstStride;
  }
#endif
#if USE_AVX2
  _mm256_zeroupper();
#endif
}
#endif
#endif
#if !MCIF_SIMD_NEW
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateHorM16_AVX2( const int16_t* src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
#ifdef USE_AVX2
  const int filterSpan = ( N-1 );
  _mm_prefetch( (const char*)( src+srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+( width>>1 )+srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+width+filterSpan+srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+2*srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+( width>>1 )+2*srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+width+filterSpan+2*srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+3*srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+( width>>1 )+3*srcStride ), _MM_HINT_T0 );
  _mm_prefetch( (const char*)( src+width+filterSpan+3*srcStride ), _MM_HINT_T0 );

  __m256i voffset    = _mm256_set1_epi32( offset );
  __m256i vibdimin   = _mm256_set1_epi16( clpRng.min );
  __m256i vibdimax   = _mm256_set1_epi16( clpRng.max );
  __m256i vzero      = _mm256_setzero_si256();
  __m256i vsum, vsuma, vsumb;

  __m256i vshuf0 = _mm256_set_epi8( 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0,
                                    0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4, 0x5, 0x4, 0x3, 0x2, 0x3, 0x2, 0x1, 0x0 );
  __m256i vshuf1 = _mm256_set_epi8( 0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4,
                                    0xd, 0xc, 0xb, 0xa, 0xb, 0xa, 0x9, 0x8, 0x9, 0x8, 0x7, 0x6, 0x7, 0x6, 0x5, 0x4 );
  __m256i vcoeff[N/2];
  for( int i=0; i<N; i+=2 )
  {
    vcoeff[i/2] = _mm256_unpacklo_epi16( _mm256_set1_epi16( coeff[i] ), _mm256_set1_epi16( coeff[i+1] ) );
  }

  for( int row = 0; row < height; row++ )
  {
    _mm_prefetch( (const char*)( src+2*srcStride ), _MM_HINT_T0 );
    _mm_prefetch( (const char*)( src+( width>>1 )+2*srcStride ), _MM_HINT_T0 );
    _mm_prefetch( (const char*)( src+width+filterSpan + 2*srcStride ), _MM_HINT_T0 );

//     _mm_prefetch( (const char*)( src+4*srcStride ), _MM_HINT_T0 );
//     _mm_prefetch( (const char*)( src+( width>>1 )+4*srcStride ), _MM_HINT_T0 );
//     _mm_prefetch( (const char*)( src+width+filterSpan + 4*srcStride ), _MM_HINT_T0 );
    for( int col = 0; col < width; col+=16 )
    {
      __m256i vsrc[3];
      for( int i=0; i<3; i++ )
      {
        vsrc[i] = _mm256_lddqu_si256( ( const __m256i * )&src[col+i*4] );
      }
      if( N==8 )
      {
        vsuma = vsumb = vzero;
        for( int i=0; i<2; i++ )
        {
          __m256i vsrca0 = _mm256_shuffle_epi8( vsrc[i], vshuf0 );
          __m256i vsrca1 = _mm256_shuffle_epi8( vsrc[i], vshuf1 );
          __m256i vsrcb0 = _mm256_shuffle_epi8( vsrc[i+1], vshuf0 );
          __m256i vsrcb1 = _mm256_shuffle_epi8( vsrc[i+1], vshuf1 );
          vsuma  = _mm256_add_epi32( vsuma, _mm256_add_epi32( _mm256_madd_epi16( vsrca0, vcoeff[2*i] ), _mm256_madd_epi16( vsrca1, vcoeff[2*i+1] ) ) );
          vsumb  = _mm256_add_epi32( vsumb, _mm256_add_epi32( _mm256_madd_epi16( vsrcb0, vcoeff[2*i] ), _mm256_madd_epi16( vsrcb1, vcoeff[2*i+1] ) ) );
        }
      }
      else
      {
        __m256i vtmp00, vtmp01, vtmp10, vtmp11;
        {
          vtmp00 = _mm256_shuffle_epi8( vsrc[0], vshuf0 );
          vtmp01 = _mm256_shuffle_epi8( vsrc[0], vshuf1 );
          vtmp10 = _mm256_shuffle_epi8( vsrc[1], vshuf0 );
          vtmp11 = _mm256_shuffle_epi8( vsrc[1], vshuf1 );
        }
        vtmp00 = _mm256_madd_epi16( vtmp00, vcoeff[0] );
        vtmp01 = _mm256_madd_epi16( vtmp01, vcoeff[1] );
        vtmp10 = _mm256_madd_epi16( vtmp10, vcoeff[0] );
        vtmp11 = _mm256_madd_epi16( vtmp11, vcoeff[1] );

        vsuma = _mm256_add_epi32( vtmp00, vtmp01 );
        vsumb = _mm256_add_epi32( vtmp10, vtmp11 );
      }

      {
        vsuma = _mm256_add_epi32( vsuma, voffset );
        vsumb = _mm256_add_epi32( vsumb, voffset );
        vsuma = _mm256_srai_epi32( vsuma, shift );
        vsumb = _mm256_srai_epi32( vsumb, shift );
        vsum  = _mm256_packs_epi32( vsuma, vsumb );
      }

      if( shiftBack )
      { //clip
        vsum = _mm256_min_epi16( vibdimax, _mm256_max_epi16( vibdimin, vsum ) );
      }
#if JEM_UNALIGNED_DST
      _mm256_storeu_si256( ( __m256i * )&dst[col], vsum );
#else
      _mm256_store_si256( ( __m256i * )&dst[col], vsum );
#endif
    }
    src += srcStride;
    dst += dstStride;
  }
#endif
}
#endif
#if IF_12TAP_SIMD
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateVerM4_12tap(const int16_t *src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff)
{
  const int16_t *srcOrig = src;
  int16_t *dstOrig = dst;
  __m128i vcoeff[N / 2], vsrc[N];
  __m128i vzero = _mm_setzero_si128();
  __m128i voffset = _mm_set1_epi32(offset);
  __m128i vibdimin = _mm_set1_epi16(clpRng.min);
  __m128i vibdimax = _mm_set1_epi16(clpRng.max);

  __m128i vsum;

  for (int i = 0; i < N; i += 2)
  {
    vcoeff[i / 2] = _mm_unpacklo_epi16(_mm_set1_epi16(coeff[i]), _mm_set1_epi16(coeff[i + 1]));
  }

  for (int col = 0; col < width; col += 4)
  {
    for (int i = 0; i < N - 1; i++)
    {
      vsrc[i] = _mm_loadl_epi64((__m128i const *)&src[col + i * srcStride]);
    }
    for (int row = 0; row < height; row++)
    {
      vsrc[N - 1] = _mm_loadl_epi64((__m128i const *)&src[col + (N - 1) * srcStride]);

      vsum = vzero;
      for (int i = 0; i < N; i += 2)
      {
        __m128i vsrc0 = _mm_unpacklo_epi16(vsrc[i], vsrc[i + 1]);
        vsum = _mm_add_epi32(vsum, _mm_madd_epi16(vsrc0, vcoeff[i / 2]));
      }

      for (int i = 0; i < N - 1; i++)
      {
        vsrc[i] = vsrc[i + 1];
      }

      //       if( shiftBack )
      {
        vsum = _mm_add_epi32(vsum, voffset);
        vsum = _mm_srai_epi32(vsum, shift);
        vsum = _mm_packs_epi32(vsum, vzero);
      }

      if (shiftBack) //clip
      {
        vsum = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsum));
      }

      _mm_storel_epi64((__m128i*)&dst[col], vsum);

      src += srcStride;
      dst += dstStride;
    }
    src = srcOrig;
    dst = dstOrig;
  }
}
#if MCIF_SIMD_NEW
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateVerNonM4_12tap(const int16_t *src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff)
{
  const int16_t *srcOrig = src;
  int16_t *dstOrig = dst;
  __m128i vcoeff[N / 2], vsrc[N];
  __m128i vzero = _mm_setzero_si128();
  __m128i voffset = _mm_set1_epi32(offset);
  __m128i vibdimin = _mm_set1_epi16(clpRng.min);
  __m128i vibdimax = _mm_set1_epi16(clpRng.max);

  __m128i vsum;

  int multiple = (width >> 2) << 2;
  const int16_t* initialSrc = src;
  int16_t* initialDst = dst;

  for (int i = 0; i < N; i += 2)
  {
    vcoeff[i / 2] = _mm_unpacklo_epi16(_mm_set1_epi16(coeff[i]), _mm_set1_epi16(coeff[i + 1]));
  }

  for (int col = 0; col < multiple; col += 4)
  {
    for (int i = 0; i < N - 1; i++)
    {
      vsrc[i] = _mm_loadl_epi64((__m128i const *)&src[col + i * srcStride]);
    }
    for (int row = 0; row < height; row++)
    {
      vsrc[N - 1] = _mm_loadl_epi64((__m128i const *)&src[col + (N - 1) * srcStride]);

      vsum = vzero;
      for (int i = 0; i < N; i += 2)
      {
        __m128i vsrc0 = _mm_unpacklo_epi16(vsrc[i], vsrc[i + 1]);
        vsum = _mm_add_epi32(vsum, _mm_madd_epi16(vsrc0, vcoeff[i / 2]));
      }

      for (int i = 0; i < N - 1; i++)
      {
        vsrc[i] = vsrc[i + 1];
      }

      //       if( shiftBack )
      {
        vsum = _mm_add_epi32(vsum, voffset);
        vsum = _mm_srai_epi32(vsum, shift);
        vsum = _mm_packs_epi32(vsum, vzero);
      }

      if (shiftBack) //clip
      {
        vsum = _mm_min_epi16(vibdimax, _mm_max_epi16(vibdimin, vsum));
      }

      _mm_storel_epi64((__m128i*)&dst[col], vsum);

      src += srcStride;
      dst += dstStride;
    }
    src = srcOrig;
    dst = dstOrig;
  }
  for (int row = 0; row < height; row++)
  {
    for (int col = multiple; col < width; col++)
    {
      int sum = 0;

      sum = initialSrc[col] * coeff[0];
      sum += initialSrc[col + 1 * srcStride] * coeff[1];
      sum += initialSrc[col + 2 * srcStride] * coeff[2];
      sum += initialSrc[col + 3 * srcStride] * coeff[3];
      sum += initialSrc[col + 4 * srcStride] * coeff[4];
      sum += initialSrc[col + 5 * srcStride] * coeff[5];
      sum += initialSrc[col + 6 * srcStride] * coeff[6];
      sum += initialSrc[col + 7 * srcStride] * coeff[7];
      sum += initialSrc[col + 8 * srcStride] * coeff[8];
      sum += initialSrc[col + 9 * srcStride] * coeff[9];
      sum += initialSrc[col + 10 * srcStride] * coeff[10];
      sum += initialSrc[col + 11 * srcStride] * coeff[11];

      Pel val = (sum + offset) >> shift;
      if (shiftBack)
      {
        val = ClipPel(val, clpRng);
      }
      initialDst[col] = val;
    }

    initialSrc += srcStride;
    initialDst += dstStride;
  }
}
#endif
#endif

#if JVET_Z0117_CHROMA_IF
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateVerM4_sym( const int16_t *src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
#if MCIF_SIMD_NEW
  _mm_prefetch((const char *)&src[0 * srcStride], _MM_HINT_T0);
  _mm_prefetch((const char *)&src[1 * srcStride], _MM_HINT_T0);
  if (N >= 2)
  {
    _mm_prefetch((const char *)&src[2 * srcStride], _MM_HINT_T0);
    _mm_prefetch((const char *)&src[3 * srcStride], _MM_HINT_T0);
  }
  if (N >= 6)
  {
    _mm_prefetch((const char *)&src[4 * srcStride], _MM_HINT_T0);
    _mm_prefetch((const char *)&src[5 * srcStride], _MM_HINT_T0);
  }
  if (N >= 8)
  {
    _mm_prefetch((const char *)&src[6 * srcStride], _MM_HINT_T0);
    _mm_prefetch((const char *)&src[7 * srcStride], _MM_HINT_T0);
  }
#endif
  const int16_t *srcOrig = src;
  int16_t *dstOrig = dst;
  __m128i vcoeff[2], vsrc[6];
  __m128i vzero = _mm_setzero_si128();
  __m128i voffset = _mm_set1_epi32( offset );
  __m128i vibdimin = _mm_set1_epi16( clpRng.min );
  __m128i vibdimax = _mm_set1_epi16( clpRng.max );
  __m128i vsum;
  vcoeff[0] = _mm_unpacklo_epi16( _mm_set1_epi16( coeff[0] ), _mm_set1_epi16( coeff[1] ) );
  vcoeff[1] = _mm_set1_epi32(coeff[2]);

  for( int col = 0; col < width; col += 4 )
  {
    vsrc[0] = _mm_loadl_epi64( ( __m128i const * )&src[col + 0 * srcStride] );
    vsrc[1] = _mm_loadl_epi64( ( __m128i const * )&src[col + 1 * srcStride] );
    vsrc[2] = _mm_loadl_epi64( ( __m128i const * )&src[col + 2 * srcStride] );
    vsrc[3] = _mm_loadl_epi64( ( __m128i const * )&src[col + 3 * srcStride] );
    vsrc[4] = _mm_loadl_epi64( ( __m128i const * )&src[col + 4 * srcStride] );
    for( int row = 0; row < height; row++ )
    {
#if MCIF_SIMD_NEW
      _mm_prefetch((const char *)&src[col + (N + 0) * srcStride], _MM_HINT_T0);
      _mm_prefetch((const char *)&src[col + (N + 1) * srcStride], _MM_HINT_T0);
#endif
      vsrc[5] = _mm_loadl_epi64( ( __m128i const * )&src[col + ( N - 1 ) * srcStride] );
      vsum = vzero;
      __m128i vsrc0 = _mm_unpacklo_epi16( vsrc[0], vsrc[1] );
      __m128i vsrc1 = _mm_unpacklo_epi16( vsrc[5], vsrc[4] );
      vsrc0 = _mm_add_epi16(vsrc0 , vsrc1);
      vsum = _mm_add_epi32( vsum, _mm_madd_epi16( vsrc0, vcoeff[0] ) );
      __m128i vsrc2 = _mm_add_epi16( vsrc[2], vsrc[3] );
      __m128i vsrc3 = _mm_cvtepi16_epi32(vsrc2);
      vsum = _mm_add_epi32( vsum, _mm_mullo_epi32(vsrc3, vcoeff[1]));
      vsrc[0] = vsrc[1];
      vsrc[1] = vsrc[2];
      vsrc[2] = vsrc[3];
      vsrc[3] = vsrc[4];
      vsum = _mm_add_epi32( vsum, voffset );
      vsum = _mm_srai_epi32( vsum, shift );
      vsum = _mm_packs_epi32( vsum, vzero );
      if( shiftBack )
      {
        vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
      }
      _mm_storel_epi64( ( __m128i* )&dst[col], vsum );
      src += srcStride;
      dst += dstStride;
    }
    src = srcOrig;
    dst = dstOrig;
  }
}
#endif
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateVerM4( const int16_t *src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
#if MCIF_SIMD_NEW
  _mm_prefetch((const char *)&src[0 * srcStride], _MM_HINT_T0);
  _mm_prefetch((const char *)&src[1 * srcStride], _MM_HINT_T0);
  if (N >= 2)
  {
    _mm_prefetch((const char *)&src[2 * srcStride], _MM_HINT_T0);
    _mm_prefetch((const char *)&src[3 * srcStride], _MM_HINT_T0);
  }
  if (N >= 6)
  {
    _mm_prefetch((const char *)&src[4 * srcStride], _MM_HINT_T0);
    _mm_prefetch((const char *)&src[5 * srcStride], _MM_HINT_T0);
  }
  if (N >= 8)
  {
    _mm_prefetch((const char *)&src[6 * srcStride], _MM_HINT_T0);
    _mm_prefetch((const char *)&src[7 * srcStride], _MM_HINT_T0);
  }
#endif
  const int16_t *srcOrig = src;
  int16_t *dstOrig = dst;

  __m128i vcoeff[N / 2], vsrc[N];
  __m128i vzero = _mm_setzero_si128();
  __m128i voffset = _mm_set1_epi32( offset );
  __m128i vibdimin = _mm_set1_epi16( clpRng.min );
  __m128i vibdimax = _mm_set1_epi16( clpRng.max );

  __m128i vsum;

  for( int i = 0; i < N; i += 2 )
  {
    vcoeff[i / 2] = _mm_unpacklo_epi16( _mm_set1_epi16( coeff[i] ), _mm_set1_epi16( coeff[i + 1] ) );
  }

  for( int col = 0; col < width; col += 4 )
  {
    for( int i = 0; i < N - 1; i++ )
    {
      vsrc[i] = _mm_loadl_epi64( ( __m128i const * )&src[col + i * srcStride] );
    }
    for( int row = 0; row < height; row++ )
    {
#if MCIF_SIMD_NEW
      _mm_prefetch((const char *)&src[col + (N + 0) * srcStride], _MM_HINT_T0);
      _mm_prefetch((const char *)&src[col + (N + 1) * srcStride], _MM_HINT_T0);
#endif
      vsrc[N - 1] = _mm_loadl_epi64( ( __m128i const * )&src[col + ( N - 1 ) * srcStride] );

      vsum = vzero;
      for( int i = 0; i < N; i += 2 )
      {
        __m128i vsrc0 = _mm_unpacklo_epi16( vsrc[i], vsrc[i + 1] );
        vsum = _mm_add_epi32( vsum, _mm_madd_epi16( vsrc0, vcoeff[i / 2] ) );
      }

      for( int i = 0; i < N - 1; i++ )
      {
        vsrc[i] = vsrc[i + 1];
      }

//       if( shiftBack )
      {
        vsum = _mm_add_epi32( vsum, voffset );
        vsum = _mm_srai_epi32( vsum, shift );
        vsum = _mm_packs_epi32( vsum, vzero );
      }
//       else
//       {
//         vsum = _mm_srai_epi32( vsum, shift );       // val = sum >> shift;
//         vsum = _mm_packs_epi32( vsum, vzero );
//       }
      if( shiftBack ) //clip
      {
        vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
      }

      _mm_storel_epi64( ( __m128i* )&dst[col], vsum );

      src += srcStride;
      dst += dstStride;
    }
    src = srcOrig;
    dst = dstOrig;
  }
}


template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateVerM8( const int16_t *src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
#if MCIF_SIMD_NEW
  _mm_prefetch((const char *)&src[0 * srcStride], _MM_HINT_T0);
  _mm_prefetch((const char *)&src[1 * srcStride], _MM_HINT_T0);
  if (N >= 2)
  {
    _mm_prefetch((const char *)&src[2 * srcStride], _MM_HINT_T0);
    _mm_prefetch((const char *)&src[3 * srcStride], _MM_HINT_T0);
  }
  if (N >= 6)
  {
    _mm_prefetch((const char *)&src[4 * srcStride], _MM_HINT_T0);
    _mm_prefetch((const char *)&src[5 * srcStride], _MM_HINT_T0);
  }
  if (N >= 8)
  {
    _mm_prefetch((const char *)&src[6 * srcStride], _MM_HINT_T0);
    _mm_prefetch((const char *)&src[7 * srcStride], _MM_HINT_T0);
  }
#endif
//   src -= ( N / 2 - 1 ) * srcStride;
  const Pel *srcOrig = src;
  int16_t *dstOrig = dst;

  __m128i vcoeff[N / 2], vsrc[N];
  __m128i vzero = _mm_setzero_si128();
  __m128i voffset = _mm_set1_epi32( offset );
  __m128i vibdimin = _mm_set1_epi16( clpRng.min );
  __m128i vibdimax = _mm_set1_epi16( clpRng.max );

  __m128i vsum, vsuma, vsumb;

  for( int i = 0; i < N; i += 2 )
  {
    vcoeff[i / 2] = _mm_unpacklo_epi16( _mm_set1_epi16( coeff[i] ), _mm_set1_epi16( coeff[i + 1] ) );
  }

  for( int col = 0; col < width; col += 8 )
  {
    for( int i = 0; i < N - 1; i++ )
    {
      vsrc[i] = _mm_lddqu_si128( ( __m128i const * )&src[col + i * srcStride] );
    }

    for( int row = 0; row < height; row++ )
    {
#if MCIF_SIMD_NEW
      _mm_prefetch((const char *)&src[col + (N + 0) * srcStride], _MM_HINT_T0);
      _mm_prefetch((const char *)&src[col + (N + 1) * srcStride], _MM_HINT_T0);
#endif
      vsrc[N - 1] = _mm_lddqu_si128( ( __m128i const * )&src[col + ( N - 1 ) * srcStride] );
      vsuma = vsumb = vzero;
      for( int i = 0; i < N; i += 2 )
      {
        __m128i vsrca = _mm_unpacklo_epi16( vsrc[i], vsrc[i + 1] );
        __m128i vsrcb = _mm_unpackhi_epi16( vsrc[i], vsrc[i + 1] );
        vsuma = _mm_add_epi32( vsuma, _mm_madd_epi16( vsrca, vcoeff[i / 2] ) );
        vsumb = _mm_add_epi32( vsumb, _mm_madd_epi16( vsrcb, vcoeff[i / 2] ) );
      }
      for( int i = 0; i < N - 1; i++ )
      {
        vsrc[i] = vsrc[i + 1];
      }

//       if( shiftBack )
      {
        vsuma = _mm_add_epi32( vsuma, voffset );
        vsumb = _mm_add_epi32( vsumb, voffset );

        vsuma = _mm_srai_epi32( vsuma, shift );
        vsumb = _mm_srai_epi32( vsumb, shift );

        vsum = _mm_packs_epi32( vsuma, vsumb );
      }
//       else
//       {
//         vsuma = _mm_srai_epi32( vsuma, shift );       // val = sum >> shift;
//         vsumb = _mm_srai_epi32( vsumb, shift );
//
//         vsum = _mm_packs_epi32( vsuma, vsumb );
//         }
//       }
      if( shiftBack ) //clip
      {
        vsum = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsum ) );
      }

#if JEM_UNALIGNED_DST
      _mm_storeu_si128( ( __m128i * )&dst[col], vsum );
#else
      _mm_store_si128( ( __m128i * )&dst[col], vsum );
#endif

      src += srcStride;
      dst += dstStride;
    }
    src = srcOrig;
    dst = dstOrig;
  }
}

#if JVET_Z0117_CHROMA_IF
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateVerM8_sym_AVX2( const int16_t *src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
#ifdef USE_AVX2
#if MCIF_SIMD_NEW
  _mm_prefetch((const char *)&src[0 * srcStride], _MM_HINT_T0);
  _mm_prefetch((const char *)&src[1 * srcStride], _MM_HINT_T0);
  if (N >= 2)
  {
    _mm_prefetch((const char *)&src[2 * srcStride], _MM_HINT_T0);
    _mm_prefetch((const char *)&src[3 * srcStride], _MM_HINT_T0);
  }
  if (N >= 6)
  {
    _mm_prefetch((const char *)&src[4 * srcStride], _MM_HINT_T0);
    _mm_prefetch((const char *)&src[5 * srcStride], _MM_HINT_T0);
  }
  if (N >= 8)
  {
    _mm_prefetch((const char *)&src[6 * srcStride], _MM_HINT_T0);
    _mm_prefetch((const char *)&src[7 * srcStride], _MM_HINT_T0);
  }
#endif
  __m256i voffset    = _mm256_set1_epi32( offset );
  __m128i vibdimin   = _mm_set1_epi16( clpRng.min );
  __m128i vibdimax   = _mm_set1_epi16( clpRng.max );
  __m256i vsum;
  __m256i vsrc[6];
  __m256i vcoeff[2];
  vcoeff[0] = _mm256_unpacklo_epi16( _mm256_set1_epi16( coeff[0] ), _mm256_set1_epi16( coeff[1] ) );
  vcoeff[1] = _mm256_set1_epi32(coeff[2]);
  const short *srcOrig = src;
  int16_t *dstOrig = dst;
  for( int col = 0; col < width; col+=8 )
  {
    for( int i=0; i<N-1; i++ )
    {
      vsrc[i]= _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i * )&src[col + i * srcStride] ) );
      vsrc[i]= _mm256_permute4x64_epi64( vsrc[i], 0x50 );
    }
    for( int row = 0; row < height; row++ )
    {
#if MCIF_SIMD_NEW
      _mm_prefetch((const char *)&src[col + (N + 0) * srcStride], _MM_HINT_T0);
      _mm_prefetch((const char *)&src[col + (N + 1) * srcStride], _MM_HINT_T0);
#endif
      vsrc[5]= _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i * )&src[col + ( N-1 ) * srcStride] ) );
      vsrc[5]= _mm256_permute4x64_epi64( vsrc[5], 0x50 );
      vsum = _mm256_setzero_si256();
      __m256i vsrc0 = _mm256_unpacklo_epi16( vsrc[0], vsrc[1] );
      __m256i vsrc1 = _mm256_unpacklo_epi16( vsrc[5], vsrc[4] );
      vsrc0 = _mm256_add_epi16(vsrc0, vsrc1);
      vsum  = _mm256_add_epi32( vsum, _mm256_madd_epi16( vsrc0, vcoeff[0] ) );
      __m256i tmpa = _mm256_permute4x64_epi64(vsrc[2], 0x08);
      __m256i tmpb = _mm256_permute4x64_epi64(vsrc[3], 0x08);
      __m128i tmpc = _mm256_extracti128_si256(tmpa, 0);
      __m128i tmpd = _mm256_extracti128_si256(tmpb, 0);
      tmpc = _mm_add_epi16(tmpc, tmpd);
      tmpa = _mm256_cvtepi16_epi32(tmpc);
      tmpb = _mm256_mullo_epi32(tmpa, vcoeff[1]);
      vsum = _mm256_add_epi32(vsum, tmpb);
      vsrc[0] = vsrc[1];
      vsrc[1] = vsrc[2];
      vsrc[2] = vsrc[3];
      vsrc[3] = vsrc[4];
      vsrc[4] = vsrc[5];
      vsum = _mm256_add_epi32( vsum, voffset );
      vsum = _mm256_srai_epi32( vsum, shift );
      __m128i vsump = _mm256_castsi256_si128( _mm256_permute4x64_epi64( _mm256_packs_epi32( vsum, vsum ), 0x88 ) );
      if( shiftBack )
      { //clip
        vsump = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsump ) );
      }
      _mm_storeu_si128( ( __m128i * )&dst[col], vsump );
      src += srcStride;
      dst += dstStride;
    }
    src= srcOrig;
    dst= dstOrig;
  }
#endif
}
#endif
// template<typename Tdst, int N, bool shiftBack, bool biPred, bool bWeight, bool bChromaIntl>
// static void qpelV16AVX2M8( const short* src, int srcStride, Tdst *dst, int dstStride, int width, int height, int shift, int bitdepth, short const *coeff, wpPredParam *pwp1=NULL, wpPredParam *pwp2=NULL )
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateVerM8_AVX2( const int16_t *src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
#ifdef USE_AVX2
#if MCIF_SIMD_NEW
  _mm_prefetch((const char *)&src[0 * srcStride], _MM_HINT_T0);
  _mm_prefetch((const char *)&src[1 * srcStride], _MM_HINT_T0);
  if (N >= 2)
  {
    _mm_prefetch((const char *)&src[2 * srcStride], _MM_HINT_T0);
    _mm_prefetch((const char *)&src[3 * srcStride], _MM_HINT_T0);
  }
  if (N >= 6)
  {
    _mm_prefetch((const char *)&src[4 * srcStride], _MM_HINT_T0);
    _mm_prefetch((const char *)&src[5 * srcStride], _MM_HINT_T0);
  }
  if (N >= 8)
  {
    _mm_prefetch((const char *)&src[6 * srcStride], _MM_HINT_T0);
    _mm_prefetch((const char *)&src[7 * srcStride], _MM_HINT_T0);
  }
#endif
  __m256i voffset    = _mm256_set1_epi32( offset );
  __m128i vibdimin   = _mm_set1_epi16( clpRng.min );
  __m128i vibdimax   = _mm_set1_epi16( clpRng.max );

  __m256i vsum;
  __m256i vsrc[N];
  __m256i vcoeff[N/2];
  for( int i=0; i<N; i+=2 )
  {
    vcoeff[i/2] = _mm256_unpacklo_epi16( _mm256_set1_epi16( coeff[i] ), _mm256_set1_epi16( coeff[i+1] ) );
  }

  const short *srcOrig = src;
  int16_t *dstOrig = dst;

  for( int col = 0; col < width; col+=8 )
  {
    for( int i=0; i<N-1; i++ )
    {
      vsrc[i]= _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i * )&src[col + i * srcStride] ) );
      vsrc[i]= _mm256_permute4x64_epi64( vsrc[i], 0x50 );
    }
    for( int row = 0; row < height; row++ )
    {
#if MCIF_SIMD_NEW
      _mm_prefetch((const char *)&src[col + (N + 0) * srcStride], _MM_HINT_T0);
      _mm_prefetch((const char *)&src[col + (N + 1) * srcStride], _MM_HINT_T0);
#endif
      vsrc[N-1]= _mm256_castsi128_si256( _mm_loadu_si128( ( const __m128i * )&src[col + ( N-1 ) * srcStride] ) );
      vsrc[N-1]= _mm256_permute4x64_epi64( vsrc[N-1], 0x50 );
      vsum = _mm256_setzero_si256();
      for( int i=0; i<N; i+=2 )
      {
        __m256i vsrc0 = _mm256_unpacklo_epi16( vsrc[i], vsrc[i+1] );
        vsum  = _mm256_add_epi32( vsum, _mm256_madd_epi16( vsrc0, vcoeff[i/2] ) );
      }
      for( int i=0; i<N-1; i++ )
      {
        vsrc[i] = vsrc[i+1];
      }

      {
        vsum = _mm256_add_epi32( vsum, voffset );
        vsum = _mm256_srai_epi32( vsum, shift );
      }

      __m128i vsump = _mm256_castsi256_si128( _mm256_permute4x64_epi64( _mm256_packs_epi32( vsum, vsum ), 0x88 ) );
      if( shiftBack )
      { //clip
        vsump = _mm_min_epi16( vibdimax, _mm_max_epi16( vibdimin, vsump ) );
      }
      _mm_storeu_si128( ( __m128i * )&dst[col], vsump );

      src += srcStride;
      dst += dstStride;
    }
    src= srcOrig;
    dst= dstOrig;
  }
#endif
}

#if !MCIF_SIMD_NEW
// template<typename Tdst, int N, bool shiftBack, bool biPred, bool bWeight, bool bChromaIntl>
// static void qpelV16AVX2M16( const short *src, int srcStride, Tdst *dst, int dstStride, int width, int height, int shift, int bitdepth, short const *coeff, wpPredParam *pwp1=NULL, wpPredParam *pwp2=NULL )
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateVerM16_AVX2( const int16_t *src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff )
{
#ifdef USE_AVX2
  __m256i voffset    = _mm256_set1_epi32( offset );
  __m256i vibdimin   = _mm256_set1_epi16( clpRng.min );
  __m256i vibdimax   = _mm256_set1_epi16( clpRng.max );
  __m256i vzero      = _mm256_setzero_si256();
  __m256i vsum, vsuma, vsumb;

  __m256i vsrc[N];
  __m256i vcoeff[N/2];
  for( int i=0; i<N; i+=2 )
  {
    vcoeff[i/2] = _mm256_unpacklo_epi16( _mm256_set1_epi16( coeff[i] ), _mm256_set1_epi16( coeff[i+1] ) );
  }

  const short *srcOrig = src;
  int16_t *dstOrig = dst;

  for( int col = 0; col < width; col+=16 )
  {
    for( int i=0; i<N-1; i++ )
    {
      vsrc[i] = _mm256_lddqu_si256( ( const __m256i * )&src[col + i * srcStride] );
    }
    for( int row = 0; row < height; row++ )
    {
      vsrc[N-1]= _mm256_lddqu_si256( ( const __m256i * )&src[col + ( N-1 ) * srcStride] );
      vsuma = vsumb = vzero;
      for( int i=0; i<N; i+=2 )
      {
        __m256i vsrca = _mm256_unpacklo_epi16( vsrc[i], vsrc[i+1] );
        __m256i vsrcb = _mm256_unpackhi_epi16( vsrc[i], vsrc[i+1] );
        vsuma  = _mm256_add_epi32( vsuma, _mm256_madd_epi16( vsrca, vcoeff[i/2] ) );
        vsumb  = _mm256_add_epi32( vsumb, _mm256_madd_epi16( vsrcb, vcoeff[i/2] ) );
      }
      for( int i=0; i<N-1; i++ )
      {
        vsrc[i] = vsrc[i+1];
      }

      {
        vsuma = _mm256_add_epi32( vsuma, voffset );
        vsumb = _mm256_add_epi32( vsumb, voffset );
        vsuma = _mm256_srai_epi32( vsuma, shift );
        vsumb = _mm256_srai_epi32( vsumb, shift );
        vsum  = _mm256_packs_epi32( vsuma, vsumb );
      }

      if( shiftBack )
      { //clip
        vsum = _mm256_min_epi16( vibdimax, _mm256_max_epi16( vibdimin, vsum ) );
      }

      _mm256_storeu_si256( ( __m256i * )&dst[col], vsum );

      src += srcStride;
      dst += dstStride;
    }
    src= srcOrig;
    dst= dstOrig;
  }
#endif
}
#endif

#if MCIF_SIMD_NEW
#if JVET_Z0117_CHROMA_IF
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateVerM16_sym_AVX2(const int16_t *src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff)
{
#ifdef USE_AVX2
  _mm_prefetch((const char *)&src[0 * srcStride], _MM_HINT_T0);
  _mm_prefetch((const char *)&src[1 * srcStride], _MM_HINT_T0);
  if (N >= 2)
  {
    _mm_prefetch((const char *)&src[2 * srcStride], _MM_HINT_T0);
    _mm_prefetch((const char *)&src[3 * srcStride], _MM_HINT_T0);
  }
  if (N >= 6)
  {
    _mm_prefetch((const char *)&src[4 * srcStride], _MM_HINT_T0);
    _mm_prefetch((const char *)&src[5 * srcStride], _MM_HINT_T0);
  }
  if (N >= 8)
  {
    _mm_prefetch((const char *)&src[6 * srcStride], _MM_HINT_T0);
    _mm_prefetch((const char *)&src[7 * srcStride], _MM_HINT_T0);
  }

  __m256i voffset = _mm256_set1_epi32(offset);
  __m256i vibdimin = _mm256_set1_epi16(clpRng.min);
  __m256i vibdimax = _mm256_set1_epi16(clpRng.max);
  __m256i vzero = _mm256_setzero_si256();
  __m256i vsum, vsuma, vsumb;
  __m256i vsrc[6];
  __m256i vcoeff[2];
  vcoeff[0] = _mm256_unpacklo_epi16(_mm256_set1_epi16(coeff[0]), _mm256_set1_epi16(coeff[1]));
  vcoeff[1] = _mm256_set1_epi32(coeff[2]);
  const short *srcOrig = src;
  int16_t *dstOrig = dst;
  
  for (int col = 0; col < width; col += 16)
  {
    vsrc[0] = _mm256_loadu_si256((const __m256i *)&src[col + 0 * srcStride]);
    vsrc[1] = _mm256_loadu_si256((const __m256i *)&src[col + 1 * srcStride]);
    vsrc[2] = _mm256_loadu_si256((const __m256i *)&src[col + 2 * srcStride]);
    vsrc[3] = _mm256_loadu_si256((const __m256i *)&src[col + 3 * srcStride]);
    vsrc[4] = _mm256_loadu_si256((const __m256i *)&src[col + 4 * srcStride]);
    for (int row = 0; row < height; row++)
    {
      _mm_prefetch((const char *)&src[col + (N + 0) * srcStride], _MM_HINT_T0);
      _mm_prefetch((const char *)&src[col + (N + 1) * srcStride], _MM_HINT_T0);
      vsrc[5] = _mm256_loadu_si256((const __m256i *)&src[col + (N - 1) * srcStride]);
      vsuma = vsumb = vzero;
      __m256i tmp05 = _mm256_add_epi16(vsrc[0], vsrc[5]);
      __m256i tmp14 = _mm256_add_epi16(vsrc[1], vsrc[4]);
      __m256i tmp23 = _mm256_add_epi16(vsrc[2], vsrc[3]);
      __m256i vsrca = _mm256_unpacklo_epi16(tmp05, tmp14);
      __m256i vsrcb = _mm256_unpackhi_epi16(tmp05, tmp14);
      vsuma = _mm256_add_epi32(vsuma, _mm256_madd_epi16(vsrca, vcoeff[0]));
      vsumb = _mm256_add_epi32(vsumb, _mm256_madd_epi16(vsrcb, vcoeff[0]));
      tmp23 = _mm256_permute4x64_epi64(tmp23, 0xd8);
      vsrca = _mm256_cvtepi16_epi32(_mm256_extracti128_si256(tmp23, 0));
      vsrcb = _mm256_cvtepi16_epi32(_mm256_extracti128_si256(tmp23, 1));
      vsuma = _mm256_add_epi32(vsuma, _mm256_mullo_epi32(vsrca, vcoeff[1]));
      vsumb = _mm256_add_epi32(vsumb, _mm256_mullo_epi32(vsrcb, vcoeff[1]));
      vsrc[0] = vsrc[1];
      vsrc[1] = vsrc[2];
      vsrc[2] = vsrc[3];
      vsrc[3] = vsrc[4];
      vsrc[4] = vsrc[5];
      vsuma = _mm256_add_epi32(vsuma, voffset);
      vsumb = _mm256_add_epi32(vsumb, voffset);
      vsuma = _mm256_srai_epi32(vsuma, shift);
      vsumb = _mm256_srai_epi32(vsumb, shift);
      vsum = _mm256_packs_epi32(vsuma, vsumb);
      if (shiftBack)
      { //clip
        vsum = _mm256_min_epi16(vibdimax, _mm256_max_epi16(vibdimin, vsum));
      }
      _mm256_storeu_si256((__m256i *)&dst[col], vsum);
      src += srcStride;
      dst += dstStride;
    }
    src = srcOrig;
    dst = dstOrig;
  }
#endif
#if USE_AVX2
  _mm256_zeroupper();
#endif
}
#endif
template<X86_VEXT vext, int N, bool shiftBack>
static void simdInterpolateVerM16_AVX2(const int16_t *src, int srcStride, int16_t *dst, int dstStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *coeff)
{
#ifdef USE_AVX2
  _mm_prefetch((const char *)&src[0 * srcStride], _MM_HINT_T0);
  _mm_prefetch((const char *)&src[1 * srcStride], _MM_HINT_T0);
  if (N >= 2)
  {
    _mm_prefetch((const char *)&src[2 * srcStride], _MM_HINT_T0);
    _mm_prefetch((const char *)&src[3 * srcStride], _MM_HINT_T0);
  }
  if (N >= 6)
  {
    _mm_prefetch((const char *)&src[4 * srcStride], _MM_HINT_T0);
    _mm_prefetch((const char *)&src[5 * srcStride], _MM_HINT_T0);
  }
  if (N >= 8)
  {
    _mm_prefetch((const char *)&src[6 * srcStride], _MM_HINT_T0);
    _mm_prefetch((const char *)&src[7 * srcStride], _MM_HINT_T0);
  }

  __m256i voffset = _mm256_set1_epi32(offset);
  __m256i vibdimin = _mm256_set1_epi16(clpRng.min);
  __m256i vibdimax = _mm256_set1_epi16(clpRng.max);
  __m256i vzero = _mm256_setzero_si256();
  __m256i vsum, vsuma, vsumb;

  __m256i vsrc[N];
  __m256i vcoeff[N / 2];
  for (int i = 0; i < N; i += 2)
  {
    vcoeff[i / 2] = _mm256_unpacklo_epi16(_mm256_set1_epi16(coeff[i]), _mm256_set1_epi16(coeff[i + 1]));
  }

  const short *srcOrig = src;
  int16_t *dstOrig = dst;

  for (int col = 0; col < width; col += 16)
  {
    for (int i = 0; i < N - 1; i++)
    {
      vsrc[i] = _mm256_loadu_si256((const __m256i *)&src[col + i * srcStride]);
    }
    for (int row = 0; row < height; row++)
    {
      _mm_prefetch((const char *)&src[col + (N + 0) * srcStride], _MM_HINT_T0);
      _mm_prefetch((const char *)&src[col + (N + 1) * srcStride], _MM_HINT_T0);

      vsrc[N - 1] = _mm256_loadu_si256((const __m256i *)&src[col + (N - 1) * srcStride]);
      vsuma = vsumb = vzero;
      for (int i = 0; i < N; i += 2)
      {
        __m256i vsrca = _mm256_unpacklo_epi16(vsrc[i], vsrc[i + 1]);
        __m256i vsrcb = _mm256_unpackhi_epi16(vsrc[i], vsrc[i + 1]);
        vsuma = _mm256_add_epi32(vsuma, _mm256_madd_epi16(vsrca, vcoeff[i / 2]));
        vsumb = _mm256_add_epi32(vsumb, _mm256_madd_epi16(vsrcb, vcoeff[i / 2]));
      }
      for (int i = 0; i < N - 1; i++)
      {
        vsrc[i] = vsrc[i + 1];
      }

      vsuma = _mm256_add_epi32(vsuma, voffset);
      vsumb = _mm256_add_epi32(vsumb, voffset);
      vsuma = _mm256_srai_epi32(vsuma, shift);
      vsumb = _mm256_srai_epi32(vsumb, shift);

      vsum = _mm256_packs_epi32(vsuma, vsumb);

      if (shiftBack)
      { //clip
        vsum = _mm256_min_epi16(vibdimax, _mm256_max_epi16(vibdimin, vsum));
      }

      _mm256_storeu_si256((__m256i *)&dst[col], vsum);

      src += srcStride;
      dst += dstStride;
    }
    src = srcOrig;
    dst = dstOrig;
  }
#endif
#if USE_AVX2

  _mm256_zeroupper();
#endif
}
#endif

template<int N, bool isLast>
inline void interpolate( const int16_t* src, int cStride, int16_t *dst, int width, int shift, int offset, int bitdepth, int maxVal, int16_t const *c )
{
  for( int col = 0; col < width; col++ )
  {
    int sum;

    sum = src[col + 0 * cStride] * c[0];
    sum += src[col + 1 * cStride] * c[1];
    if( N >= 4 )
    {
      sum += src[col + 2 * cStride] * c[2];
      sum += src[col + 3 * cStride] * c[3];
    }
    if( N >= 6 )
    {
      sum += src[col + 4 * cStride] * c[4];
      sum += src[col + 5 * cStride] * c[5];
    }
    if( N == 8 )
    {
      sum += src[col + 6 * cStride] * c[6];
      sum += src[col + 7 * cStride] * c[7];
    }

    Pel val = ( sum + offset ) >> shift;
    if( isLast )
    {
      val = ( val < 0 ) ? 0 : val;
      val = ( val > maxVal ) ? maxVal : val;
    }
    dst[col] = val;
  }
}


static inline __m128i simdInterpolateLuma2P8( int16_t const *src, int srcStride, __m128i *mmCoeff, const __m128i & mmOffset, int shift )
{
  __m128i sumHi = _mm_setzero_si128();
  __m128i sumLo = _mm_setzero_si128();
  for( int n = 0; n < 2; n++ )
  {
    __m128i mmPix = _mm_loadu_si128( ( __m128i* )src );
    __m128i hi = _mm_mulhi_epi16( mmPix, mmCoeff[n] );
    __m128i lo = _mm_mullo_epi16( mmPix, mmCoeff[n] );
    sumHi = _mm_add_epi32( sumHi, _mm_unpackhi_epi16( lo, hi ) );
    sumLo = _mm_add_epi32( sumLo, _mm_unpacklo_epi16( lo, hi ) );
    src += srcStride;
  }
  sumHi = _mm_srai_epi32( _mm_add_epi32( sumHi, mmOffset ), shift );
  sumLo = _mm_srai_epi32( _mm_add_epi32( sumLo, mmOffset ), shift );
  return( _mm_packs_epi32( sumLo, sumHi ) );
}

static inline __m128i simdInterpolateLuma2P4( int16_t const *src, int srcStride, __m128i *mmCoeff, const __m128i & mmOffset, int shift )
{
  __m128i sumHi = _mm_setzero_si128();
  __m128i sumLo = _mm_setzero_si128();
  for( int n = 0; n < 2; n++ )
  {
    __m128i mmPix = _mm_loadl_epi64( ( __m128i* )src );
    __m128i hi = _mm_mulhi_epi16( mmPix, mmCoeff[n] );
    __m128i lo = _mm_mullo_epi16( mmPix, mmCoeff[n] );
    sumHi = _mm_add_epi32( sumHi, _mm_unpackhi_epi16( lo, hi ) );
    sumLo = _mm_add_epi32( sumLo, _mm_unpacklo_epi16( lo, hi ) );
    src += srcStride;
  }
  sumHi = _mm_srai_epi32( _mm_add_epi32( sumHi, mmOffset ), shift );
  sumLo = _mm_srai_epi32( _mm_add_epi32( sumLo, mmOffset ), shift );
  return( _mm_packs_epi32( sumLo, sumHi ) );
}

static inline __m128i simdClip3( __m128i mmMin, __m128i mmMax, __m128i mmPix )
{
  __m128i mmMask = _mm_cmpgt_epi16( mmPix, mmMin );
  mmPix = _mm_or_si128( _mm_and_si128( mmMask, mmPix ), _mm_andnot_si128( mmMask, mmMin ) );
  mmMask = _mm_cmplt_epi16( mmPix, mmMax );
  mmPix = _mm_or_si128( _mm_and_si128( mmMask, mmPix ), _mm_andnot_si128( mmMask, mmMax ) );
  return( mmPix );
}

template<X86_VEXT vext, bool isLast>
static void simdInterpolateN2_M8( const int16_t* src, int srcStride, int16_t *dst, int dstStride, int cStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *c )
{
  int row, col;
  __m128i mmOffset = _mm_set1_epi32( offset );
  __m128i mmCoeff[2];
  __m128i mmMin = _mm_set1_epi16( clpRng.min );
  __m128i mmMax = _mm_set1_epi16( clpRng.max );
  for( int n = 0; n < 2; n++ )
    mmCoeff[n] = _mm_set1_epi16( c[n] );
  for( row = 0; row < height; row++ )
  {
    for( col = 0; col < width; col += 8 )
    {
      __m128i mmFiltered = simdInterpolateLuma2P8( src + col, cStride, mmCoeff, mmOffset, shift );
      if( isLast )
      {
        mmFiltered = simdClip3( mmMin, mmMax, mmFiltered );
      }
      _mm_storeu_si128( ( __m128i * )( dst + col ), mmFiltered );
    }
    src += srcStride;
    dst += dstStride;
  }
}

template<X86_VEXT vext, bool isLast>
static void simdInterpolateN2_M4( const int16_t* src, int srcStride, int16_t *dst, int dstStride, int cStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *c )
{
  int row, col;
  __m128i mmOffset = _mm_set1_epi32( offset );
  __m128i mmCoeff[8];
  __m128i mmMin = _mm_set1_epi16( clpRng.min );
  __m128i mmMax = _mm_set1_epi16( clpRng.max );
  for( int n = 0; n < 2; n++ )
    mmCoeff[n] = _mm_set1_epi16( c[n] );
  for( row = 0; row < height; row++ )
  {
    for( col = 0; col < width; col += 4 )
    {
      __m128i mmFiltered = simdInterpolateLuma2P4( src + col, cStride, mmCoeff, mmOffset, shift );
      if( isLast )
      {
        mmFiltered = simdClip3( mmMin, mmMax, mmFiltered );
      }
      _mm_storel_epi64( ( __m128i * )( dst + col ), mmFiltered );
    }
    src += srcStride;
    dst += dstStride;
  }
}
#ifdef USE_AVX2
static inline __m256i simdInterpolateLuma10Bit2P16(int16_t const *src1, int srcStride, __m256i *mmCoeff, const __m256i & mmOffset, __m128i &mmShift)
{
  __m256i sumLo;
  {
    __m256i mmPix = _mm256_loadu_si256((__m256i*)src1);
    __m256i mmPix1 = _mm256_loadu_si256((__m256i*)(src1 + srcStride));
    __m256i lo0 = _mm256_mullo_epi16(mmPix, mmCoeff[0]);
    __m256i lo1 = _mm256_mullo_epi16(mmPix1, mmCoeff[1]);
    sumLo = _mm256_add_epi16(lo0, lo1);
  }
  sumLo = _mm256_sra_epi16(_mm256_add_epi16(sumLo, mmOffset), mmShift);
  return(sumLo);
}
#endif

static inline __m128i simdInterpolateLuma10Bit2P8(int16_t const *src1, int srcStride, __m128i *mmCoeff, const __m128i & mmOffset, __m128i &mmShift)
{
  __m128i sumLo;
  {
    __m128i mmPix = _mm_loadu_si128((__m128i*)src1);
    __m128i mmPix1 = _mm_loadu_si128((__m128i*)(src1 + srcStride));
    __m128i lo0 = _mm_mullo_epi16(mmPix, mmCoeff[0]);
    __m128i lo1 = _mm_mullo_epi16(mmPix1, mmCoeff[1]);
    sumLo = _mm_add_epi16(lo0, lo1);
  }
  sumLo = _mm_sra_epi16(_mm_add_epi16(sumLo, mmOffset), mmShift);
  return(sumLo);
}

static inline __m128i simdInterpolateLuma10Bit2P4(int16_t const *src, int srcStride, __m128i *mmCoeff, const __m128i & mmOffset, __m128i &mmShift)
{
  __m128i sumLo;
  {
    __m128i mmPix = _mm_loadl_epi64((__m128i*)src);
    __m128i mmPix1 = _mm_loadl_epi64((__m128i*)(src + srcStride));
    __m128i lo0 = _mm_mullo_epi16(mmPix, mmCoeff[0]);
    __m128i lo1 = _mm_mullo_epi16(mmPix1, mmCoeff[1]);
    sumLo = _mm_add_epi16(lo0, lo1);
  }
  sumLo = _mm_sra_epi16(_mm_add_epi16(sumLo, mmOffset), mmShift);
  return sumLo;
}

#ifdef USE_AVX2
static inline __m256i simdInterpolateLumaHighBit2P16(int16_t const *src1, int srcStride, __m256i *mmCoeff, const __m256i & mmOffset, __m128i &mmShift)
{
  __m256i mm_mul_lo = _mm256_setzero_si256();
  __m256i mm_mul_hi = _mm256_setzero_si256();

  for (int coefIdx = 0; coefIdx < 2; coefIdx++)
  {
    __m256i mmPix = _mm256_lddqu_si256((__m256i*)(src1 + coefIdx * srcStride));
    __m256i mm_hi = _mm256_mulhi_epi16(mmPix, mmCoeff[coefIdx]);
    __m256i mm_lo = _mm256_mullo_epi16(mmPix, mmCoeff[coefIdx]);
    mm_mul_lo = _mm256_add_epi32(mm_mul_lo, _mm256_unpacklo_epi16(mm_lo, mm_hi));
    mm_mul_hi = _mm256_add_epi32(mm_mul_hi, _mm256_unpackhi_epi16(mm_lo, mm_hi));
  }
  mm_mul_lo = _mm256_sra_epi32(_mm256_add_epi32(mm_mul_lo, mmOffset), mmShift);
  mm_mul_hi = _mm256_sra_epi32(_mm256_add_epi32(mm_mul_hi, mmOffset), mmShift);
  __m256i mm_sum = _mm256_packs_epi32(mm_mul_lo, mm_mul_hi);
  return (mm_sum);
}
#endif

static inline __m128i simdInterpolateLumaHighBit2P8(int16_t const *src1, int srcStride, __m128i *mmCoeff, const __m128i & mmOffset, __m128i &mmShift)
{
  __m128i mm_mul_lo = _mm_setzero_si128();
  __m128i mm_mul_hi = _mm_setzero_si128();

  for (int coefIdx = 0; coefIdx < 2; coefIdx++)
  {
    __m128i mmPix = _mm_loadu_si128((__m128i*)(src1 + coefIdx * srcStride));
    __m128i mm_hi = _mm_mulhi_epi16(mmPix, mmCoeff[coefIdx]);
    __m128i mm_lo = _mm_mullo_epi16(mmPix, mmCoeff[coefIdx]);
    mm_mul_lo = _mm_add_epi32(mm_mul_lo, _mm_unpacklo_epi16(mm_lo, mm_hi));
    mm_mul_hi = _mm_add_epi32(mm_mul_hi, _mm_unpackhi_epi16(mm_lo, mm_hi));
  }
  mm_mul_lo = _mm_sra_epi32(_mm_add_epi32(mm_mul_lo, mmOffset), mmShift);
  mm_mul_hi = _mm_sra_epi32(_mm_add_epi32(mm_mul_hi, mmOffset), mmShift);
  __m128i mm_sum = _mm_packs_epi32(mm_mul_lo, mm_mul_hi);
  return(mm_sum);
}

static inline __m128i simdInterpolateLumaHighBit2P4(int16_t const *src1, int srcStride, __m128i *mmCoeff, const __m128i & mmOffset, __m128i &mmShift)
{
  __m128i mm_sum = _mm_setzero_si128();
  __m128i mm_zero = _mm_setzero_si128();
  for (int coefIdx = 0; coefIdx < 2; coefIdx++)
  {
    __m128i mmPix = _mm_loadl_epi64((__m128i*)(src1 + coefIdx * srcStride));
    __m128i mm_hi = _mm_mulhi_epi16(mmPix, mmCoeff[coefIdx]);
    __m128i mm_lo = _mm_mullo_epi16(mmPix, mmCoeff[coefIdx]);
    __m128i mm_mul = _mm_unpacklo_epi16(mm_lo, mm_hi);
    mm_sum = _mm_add_epi32(mm_sum, mm_mul);
  }
  mm_sum = _mm_sra_epi32(_mm_add_epi32(mm_sum, mmOffset), mmShift);
  mm_sum = _mm_packs_epi32(mm_sum, mm_zero);
  return(mm_sum);
}

template<X86_VEXT vext, bool isLast>
static void simdInterpolateN2_HIGHBIT_M4(const int16_t* src, int srcStride, int16_t *dst, int dstStride, int cStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *c)
{
#if USE_AVX2
  __m256i mm256Offset = _mm256_set1_epi32(offset);
  __m256i mm256Coeff[2];
  for (int n = 0; n < 2; n++)
  {
    mm256Coeff[n] = _mm256_set1_epi16(c[n]);
  }
#endif
  __m128i mmOffset = _mm_set1_epi32(offset);
  __m128i mmCoeff[2];
  for (int n = 0; n < 2; n++)
    mmCoeff[n] = _mm_set1_epi16(c[n]);

  __m128i mmShift = _mm_cvtsi64_si128(shift);

  CHECK(isLast, "Not Supported");
  CHECK(width % 4 != 0, "Not Supported");

  for (int row = 0; row < height; row++)
  {
    int col = 0;
#if USE_AVX2
    for (; col < ((width >> 4) << 4); col += 16)
    {
      __m256i mmFiltered = simdInterpolateLumaHighBit2P16(src + col, cStride, mm256Coeff, mm256Offset, mmShift);
      _mm256_storeu_si256((__m256i *)(dst + col), mmFiltered);
    }
#endif
    for (; col < ((width >> 3) << 3); col += 8)
    {
      __m128i mmFiltered = simdInterpolateLumaHighBit2P8(src + col, cStride, mmCoeff, mmOffset, mmShift);
      _mm_storeu_si128((__m128i *)(dst + col), mmFiltered);
    }

    for (; col < ((width >> 2) << 2); col += 4)
    {
      __m128i mmFiltered = simdInterpolateLumaHighBit2P4(src + col, cStride, mmCoeff, mmOffset, mmShift);
      _mm_storel_epi64((__m128i *)(dst + col), mmFiltered);
    }
    src += srcStride;
    dst += dstStride;
  }
}

template<X86_VEXT vext, bool isLast>
static void simdInterpolateN2_10BIT_M4(const int16_t* src, int srcStride, int16_t *dst, int dstStride, int cStride, int width, int height, int shift, int offset, const ClpRng& clpRng, int16_t const *c)
{
  int row, col;
  __m128i mmOffset = _mm_set1_epi16(offset);
  __m128i mmShift = _mm_set_epi64x(0, shift);
  __m128i mmCoeff[2];
  for (int n = 0; n < 2; n++)
    mmCoeff[n] = _mm_set1_epi16(c[n]);

  CHECK(isLast, "Not Supported");

#if USE_AVX2
  __m256i mm256Offset = _mm256_set1_epi16(offset);
  __m256i mm256Coeff[2];
  for (int n = 0; n < 2; n++)
    mm256Coeff[n] = _mm256_set1_epi16(c[n]);
#endif
  for (row = 0; row < height; row++)
  {
    col = 0;
#if USE_AVX2
    // multiple of 16
    for (; col < ((width >> 4) << 4); col += 16)
    {
      __m256i mmFiltered = simdInterpolateLuma10Bit2P16(src + col, cStride, mm256Coeff, mm256Offset, mmShift);
      _mm256_storeu_si256((__m256i *)(dst + col), mmFiltered);
    }
#endif
    // multiple of 8
    for (; col < ((width >> 3) << 3); col += 8)
    {
      __m128i mmFiltered = simdInterpolateLuma10Bit2P8(src + col, cStride, mmCoeff, mmOffset, mmShift);
      _mm_storeu_si128((__m128i *)(dst + col), mmFiltered);
    }

    // last 4 samples
    for (; col < ((width >> 2) << 2); col += 4)
    {
      __m128i mmFiltered = simdInterpolateLuma10Bit2P4(src + col, cStride, mmCoeff, mmOffset, mmShift);
      _mm_storel_epi64((__m128i *)(dst + col), mmFiltered);
    }
    src += srcStride;
    dst += dstStride;
  }
}

template<X86_VEXT vext, int N, bool isVertical, bool isFirst, bool isLast>
static void simdFilter( const ClpRng& clpRng, Pel const *src, int srcStride, Pel *dst, int dstStride, int width, int height, TFilterCoeff const *coeff, bool biMCForDMVR)
{
  int row, col;
#if JVET_Z0117_CHROMA_IF
  bool issym = false;
#endif
#if IF_12TAP_SIMD
  Pel c[16] = { 0, };
#else
  Pel c[8];
#endif
  c[0] = coeff[0];
  c[1] = coeff[1];
  if( N >= 4 )
  {
    c[2] = coeff[2];
    c[3] = coeff[3];
  }
  if( N >= 6 )
  {
    c[4] = coeff[4];
    c[5] = coeff[5];
  }
#if IF_12TAP_SIMD
  if (N >= 8)
  {
    c[6] = coeff[6];
    c[7] = coeff[7];
  }
  if (N >= 10)
  {
    c[8] = coeff[8];
    c[9] = coeff[9];
  }
  if (N == 12)
  {
    c[10] = coeff[10];
    c[11] = coeff[11];
    c[12] = c[8];
    c[13] = c[9];
    c[14] = c[10];
    c[15] = c[11];
  }
#else
  if( N == 8 )
  {
    c[6] = coeff[6];
    c[7] = coeff[7];
  }
#endif
  int cStride = ( isVertical ) ? srcStride : 1;
  src -= ( N/2 - 1 ) * cStride;

  int offset;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  int headRoom = IF_INTERNAL_FRAC_BITS(clpRng.bd);
#else
  int headRoom = std::max<int>( 2, ( IF_INTERNAL_PREC - clpRng.bd ) );
#endif
  int shift    = IF_FILTER_PREC;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20
  CHECK( shift < 0, "Negative shift" );


  if( isLast )
  {
    shift += ( isFirst ) ? 0 : headRoom;
    offset = 1 << ( shift - 1 );
    offset += ( isFirst ) ? 0 : IF_INTERNAL_OFFS << IF_FILTER_PREC;
  }
  else
  {
    shift -= ( isFirst ) ? headRoom : 0;
    offset = ( isFirst ) ? -IF_INTERNAL_OFFS << shift : 0;
  }

  if (biMCForDMVR)
  {
    if( isFirst )
    {
      shift = IF_FILTER_PREC_BILINEAR - (IF_INTERNAL_PREC_BILINEAR - clpRng.bd);
      offset = 1 << (shift - 1);
    }
    else
    {
      shift = 4;
      offset = 1 << (shift - 1);
    }
  }
  {
#if IF_12TAP_SIMD
    if (N == 12 && !(width & 0x03))
    {
      if (!isVertical)
      {
        simdInterpolateHorM4_12tap<vext, 12, isLast>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
      }
      else
        simdInterpolateVerM4_12tap<vext, 12, isLast>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
      return;
    }
#if MCIF_SIMD_NEW
    else if (N == 12 && width > 4)
    {
      if (!isVertical)
      {
        simdInterpolateHorNonM4_12tap<vext, 12, isLast>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
      }
      else
      {
        simdInterpolateVerNonM4_12tap<vext, 12, isLast>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
      }
      return;
  }
#endif
#if JVET_Z0117_CHROMA_IF
    else if ((N == 8 || (N == 6)) && !(width & 0x07))
#else
    else if (N == 8 && !(width & 0x07))
#endif
#else
    if (N == 8 && !(width & 0x07))
#endif
    {
      if( !isVertical )
      {
        if( vext>= AVX2 )
#if MCIF_SIMD_NEW
          if (!(width & 15))
#if JVET_Z0117_CHROMA_IF
          {
            if(issym)
            {
              simdInterpolateHorM16_sym_AVX2<vext, 6, isLast>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
            }
            else if(N == 6)
            {
              simdInterpolateHorM16_AVX2<vext, 6, isLast>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
            }
            else
            {
              simdInterpolateHorM16_AVX2<vext, 8, isLast>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
            }
          }
#else
            simdInterpolateHorM16_AVX2<vext, 8, isLast>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
#endif
          else
#endif
#if JVET_Z0117_CHROMA_IF
          {
            if(issym)
            {
              simdInterpolateHorM8_sym_AVX2<vext, 6, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
            }
            else if(N == 6)
            {
              simdInterpolateHorM8_AVX2<vext, 6, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
            }
            else
            {
              simdInterpolateHorM8_AVX2<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
            }
          }
#else
          simdInterpolateHorM8_AVX2<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
#endif
        else
#if JVET_Z0117_CHROMA_IF
          if(N == 6)
          {
            simdInterpolateHorM8<vext, 6, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
          }
          else
          {
            simdInterpolateHorM8<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
          }
#else
          simdInterpolateHorM8<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
#endif
      }
      else
      {
        if( vext>= AVX2 )
#if MCIF_SIMD_NEW
          if (!(width & 15))
#if JVET_Z0117_CHROMA_IF
          {
            if(issym)
            {
              simdInterpolateVerM16_AVX2<vext, 6, isLast>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
            }
            else if(N == 6)
            {
              simdInterpolateVerM16_AVX2<vext, 6, isLast>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
            }
            else
            {
              simdInterpolateVerM16_AVX2<vext, 8, isLast>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
            }
          }
#else
            simdInterpolateVerM16_AVX2<vext, 8, isLast>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
#endif
          else
#endif
#if JVET_Z0117_CHROMA_IF
          {
            if(issym)
            {
              simdInterpolateVerM8_AVX2<vext, 6, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
            }
            else if(N == 6)
            {
              simdInterpolateVerM8_AVX2<vext, 6, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
            }
            else
            {
              simdInterpolateVerM8_AVX2<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
            }
          }
#else
          simdInterpolateVerM8_AVX2<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
#endif
        else
#if JVET_Z0117_CHROMA_IF
        {
          if(N == 6)
          {
            simdInterpolateVerM8<vext, 6, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
          }
          else
          {
            simdInterpolateVerM8<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
          }
        }
#else
          simdInterpolateVerM8<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
#endif
      }
      return;
    }
#if JVET_Z0117_CHROMA_IF
    else if( (N == 8 || N == 6) && !( width & 0x03 ) )
#else
    else if( N == 8 && !( width & 0x03 ) )
#endif
    {
      if( !isVertical )
      {
#if JVET_Z0117_CHROMA_IF
      {
        if(issym)
        {
          simdInterpolateHorM4_sym<vext, 6, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
        }
        else if(N == 6)
        {
          simdInterpolateHorM4<vext, 6, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
        }
        else
        {
          simdInterpolateHorM4<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
        }
      }
#else
        simdInterpolateHorM4<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
#endif
      }
      else
#if JVET_Z0117_CHROMA_IF
      {
        if(issym)
        {
          simdInterpolateVerM4<vext, 6, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
        }
        else if(N == 6)
        {
          simdInterpolateVerM4<vext, 6, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
        }
        else
        {
          simdInterpolateVerM4<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
        }
      }
#else
        simdInterpolateVerM4<vext, 8, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
#endif
      return;
    }
#if IF_FILTER_WIDTH_NON_M4
#if JVET_Z0117_CHROMA_IF
    else if ((N == 8 || (N == 6)) && width > 16)
#else
    else if (N == 8 && width > 16)
#endif
    {
      const int16_t* initialSrc = src;
      int16_t* initialDst = dst;
      if (!isVertical)
      {
#if JVET_Z0117_CHROMA_IF
        if(issym)
        {
          simdInterpolateHorM16_sym_AVX2<vext, 6, isLast>(src, srcStride, dst, dstStride, width - 1, height, shift, offset, clpRng, c);
        }
        else if(N == 6)
        {
          simdInterpolateHorM16_AVX2<vext, 6, isLast>(src, srcStride, dst, dstStride, width - 1, height, shift, offset, clpRng, c);
        }
        else
        {
          simdInterpolateHorM16_AVX2<vext, 8, isLast>(src, srcStride, dst, dstStride, width - 1, height, shift, offset, clpRng, c);
        }
#else
        simdInterpolateHorM16_AVX2<vext, 8, isLast>(src, srcStride, dst, dstStride, width - 1, height, shift, offset, clpRng, c);
#endif
      }
      else
      {
#if JVET_Z0117_CHROMA_IF
        if(issym)
        {
          simdInterpolateVerM16_AVX2<vext, 6, isLast>(src, srcStride, dst, dstStride, width - 1, height, shift, offset, clpRng, c);
        }
        else if(N == 6)
        {
          simdInterpolateVerM16_AVX2<vext, 6, isLast>(src, srcStride, dst, dstStride, width - 1, height, shift, offset, clpRng, c);
        }
        else
        {
          simdInterpolateVerM16_AVX2<vext, 8, isLast>(src, srcStride, dst, dstStride, width - 1, height, shift, offset, clpRng, c);
        }
#else
        simdInterpolateVerM16_AVX2<vext, 8, isLast>(src, srcStride, dst, dstStride, width - 1, height, shift, offset, clpRng, c);
#endif
      }
      int sum = 0;
      int stride = isVertical ? srcStride : 1;
#if JVET_Z0117_CHROMA_IF
      if(N >= 8)
      {
#endif
        for (int row = 0; row < height; row++)
        {
          for (int col = width - 1; col < width; col++)
          {

            sum = initialSrc[col] * coeff[0];
            sum += initialSrc[col + 1 * stride] * coeff[1];
            sum += initialSrc[col + 2 * stride] * coeff[2];
            sum += initialSrc[col + 3 * stride] * coeff[3];
            sum += initialSrc[col + 4 * stride] * coeff[4];
            sum += initialSrc[col + 5 * stride] * coeff[5];
            sum += initialSrc[col + 6 * stride] * coeff[6];
            sum += initialSrc[col + 7 * stride] * coeff[7];

            Pel val = (sum + offset) >> shift;
            if (isLast)
            {
              val = ClipPel(val, clpRng);
            }
            initialDst[col] = val;
          }

          initialSrc += srcStride;
          initialDst += dstStride;
        }
#if JVET_Z0117_CHROMA_IF
      }
      else if(N == 6){
        col = width - 1
        for (int row = 0; row < height; row++)
        {
          {
            sum = initialSrc[col] * coeff[0];
            sum += initialSrc[col + 1 * stride] * coeff[1];
            sum += initialSrc[col + 2 * stride] * coeff[2];
            sum += initialSrc[col + 3 * stride] * coeff[3];
            sum += initialSrc[col + 4 * stride] * coeff[4];
            sum += initialSrc[col + 5 * stride] * coeff[5];

            Pel val = (sum + offset) >> shift;
            if (isLast)
            {
              val = ClipPel(val, clpRng);
            }
            initialDst[col] = val;
          }

          initialSrc += srcStride;
          initialDst += dstStride;
        }
    }
      }
#endif
    }
#endif
    else if( N == 4 && !( width & 0x03 ) )
    {
      if( !isVertical )
      {
        if( ( width % 8 ) == 0 )
        {
          if( vext>= AVX2 )
#if MCIF_SIMD_NEW
            if (!(width & 15))
              simdInterpolateHorM16_AVX2<vext, 4, isLast>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
            else
#endif
            simdInterpolateHorM8_AVX2<vext, 4, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
          else
            simdInterpolateHorM8<vext, 4, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
        }
        else
          simdInterpolateHorM4<vext, 4, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
      }
      else
#if MCIF_SIMD_NEW
      {
        if ((width % 8) == 0)
        {
          if (vext >= AVX2)
            if (!(width & 15))
              simdInterpolateVerM16_AVX2<vext, 4, isLast>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
            else
              simdInterpolateVerM8_AVX2 <vext, 4, isLast>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
          else
            simdInterpolateVerM8<vext, 4, isLast>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
        }
        else
          simdInterpolateVerM4<vext, 4, isLast>(src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c);
      }
#else
        simdInterpolateVerM4<vext, 4, isLast>( src, srcStride, dst, dstStride, width, height, shift, offset, clpRng, c );
#endif
      return;
    }
    else if (biMCForDMVR)
    {
      if (N == 2 && !(width & 0x03))
      {
        if (clpRng.bd <= 10)
        {
        simdInterpolateN2_10BIT_M4<vext, isLast>(src, srcStride, dst, dstStride, cStride, width, height, shift, offset, clpRng, c);
        }
        else
        {
          simdInterpolateN2_HIGHBIT_M4<vext, isLast>(src, srcStride, dst, dstStride, cStride, width, height, shift, offset, clpRng, c);
        }
        return;
      }
    }
    else if( N == 2 && !( width & 0x07 ) )
    {
      simdInterpolateN2_M8<vext, isLast>( src, srcStride, dst, dstStride, cStride, width, height, shift, offset, clpRng, c );
      return;
    }
    else if( N == 2 && !( width & 0x03 ) )
    {
      simdInterpolateN2_M4<vext, isLast>( src, srcStride, dst, dstStride, cStride, width, height, shift, offset, clpRng, c );
      return;
    }
  }

  for( row = 0; row < height; row++ )
  {
    for( col = 0; col < width; col++ )
    {
      int sum;

      sum  = src[col + 0 * cStride] * c[0];
      sum += src[col + 1 * cStride] * c[1];
      if( N >= 4 )
      {
        sum += src[col + 2 * cStride] * c[2];
        sum += src[col + 3 * cStride] * c[3];
      }
      if( N >= 6 )
      {
        sum += src[col + 4 * cStride] * c[4];
        sum += src[col + 5 * cStride] * c[5];
      }
#if IF_12TAP_SIMD
      if (N >= 8)
      {
        sum += src[col + 6 * cStride] * c[6];
        sum += src[col + 7 * cStride] * c[7];
      }
      if (N >= 10)
      {
        sum += src[col + 8 * cStride] * c[8];
        sum += src[col + 9 * cStride] * c[9];
      }
      if (N == 12)
      {
        sum += src[col + 10 * cStride] * c[10];
        sum += src[col + 11 * cStride] * c[11];
      }
#else
      if (N == 8)
      {
        sum += src[col + 6 * cStride] * c[6];
        sum += src[col + 7 * cStride] * c[7];
      }
#endif
      Pel val = ( sum + offset ) >> shift;
      if( isLast )
      {
        val = ClipPel( val, clpRng );
      }
      dst[col] = val;
    }

    src += srcStride;
    dst += dstStride;
  }
}

#if JVET_AB0155_SGPM

template<X86_VEXT vext> 
int xSadTM_SSE(const PredictionUnit &pu, const int width, const int height, const int templateWidth,
                                const int templateHeight, const ComponentID compIdx, PelBuf &predBuf, PelBuf &recBuf,
                                PelBuf &adBuf)
{
  int     sad        = 0;
  int32_t iPredStride = predBuf.stride;
  int32_t iRecStride  = recBuf.stride;
  int32_t iAdStride   = adBuf.stride;

  // top template
  Pel *piPred = predBuf.buf + templateWidth;
  // start point of predBuf is (-templateWidth, -templateHeight) of current block
  Pel *piAd  = adBuf.buf + templateWidth;
  Pel *piRec = recBuf.buf - templateHeight * iRecStride;   // start point of recBuf is (0,0) of current block

  if (width == 4)
  {
    __m128i vzero = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    // for luma, to be confirmed
    for (int y = 0; y < templateHeight; y++)
    {
      __m128i vPred = _mm_loadl_epi64((__m128i *) (piPred));
      __m128i vRec  = _mm_loadl_epi64((__m128i *) (piRec));
      __m128i vAd   = _mm_abs_epi16(_mm_sub_epi16(vRec, vPred));
      _mm_storel_epi64((__m128i *) (piAd), vAd);
      __m128i vsumtemp = _mm_unpacklo_epi16(vAd, vzero);
      vsum32           = _mm_add_epi32(vsum32, vsumtemp);
      piPred += iPredStride;
      piAd += iAdStride;
      piRec += iRecStride;
    }
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    sad  = _mm_cvtsi128_si32(vsum32);
  }
#if USE_AVX2
  else if (0 == (width % 16))
  {
    __m256i vzero  = _mm256_setzero_si256();
    __m256i vsum32 = vzero;
    for (int y = 0; y < templateHeight; y++)
    {
      __m256i vsum16 = vzero;
      for (int x = 0; x < width; x += 16)
      {
        __m256i vPred = _mm256_lddqu_si256((__m256i *) (piPred + x));   // why not aligned with 128/256 bit boundaries
        __m256i vRec  = _mm256_lddqu_si256((__m256i *) (piRec + x));
        __m256i vAd   = _mm256_abs_epi16(_mm256_sub_epi16(vRec, vPred));
        _mm256_storeu_si256((__m256i *) (piAd + x), vAd);

        vsum16 = _mm256_add_epi16(vsum16, vAd);
      }
      __m256i vsumtemp = _mm256_add_epi32(_mm256_unpacklo_epi16(vsum16, vzero), _mm256_unpackhi_epi16(vsum16, vzero));
      vsum32           = _mm256_add_epi32(vsum32, vsumtemp);
      piPred += iPredStride;
      piAd += iAdStride;
      piRec += iRecStride;
    }
    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    sad  = _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum32))
            + _mm_cvtsi128_si32(_mm256_castsi256_si128(_mm256_permute2x128_si256(vsum32, vsum32, 0x11)));
  }
#endif
  else
  {
    __m128i vzero  = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    for (int y = 0; y < templateHeight; y++)
    {
      __m128i vsum16 = vzero;
      for (int x = 0; x < width; x += 8)
      {
        __m128i vPred = _mm_lddqu_si128((__m128i *) (piPred + x));
        __m128i vRec  = _mm_lddqu_si128((__m128i *) (piRec + x));
        __m128i vAd   = _mm_abs_epi16(_mm_sub_epi16(vRec, vPred));
        _mm_storeu_si128((__m128i *) (piAd + x), vAd);
        vsum16 = _mm_add_epi16(vsum16, vAd);
      }
      __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
      vsum32           = _mm_add_epi32(vsum32, vsumtemp);
      piPred += iPredStride;
      piAd += iAdStride;
      piRec += iRecStride;
    }
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    sad  = _mm_cvtsi128_si32(vsum32);
  }

  // left template
  piPred = predBuf.buf + templateHeight * iPredStride;
  // start point of predBuf is (-templateWidth, -templateHeight) of current block
  piAd  = adBuf.buf + templateHeight * iAdStride;
  piRec = recBuf.buf - templateWidth;   // start point of recBuf is (0,0) of current block

  if (templateWidth == 4)
  {
    __m128i vzero  = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    // for luma, to be confirmed
    for (int y = 0; y < height; y++)
    {
      __m128i vPred = _mm_loadl_epi64((__m128i *) (piPred));
      __m128i vRec  = _mm_loadl_epi64((__m128i *) (piRec));
      __m128i vAd   = _mm_abs_epi16(_mm_sub_epi16(vRec, vPred));
      _mm_storel_epi64((__m128i *) (piAd), vAd);
      __m128i vsumtemp = _mm_unpacklo_epi16(vAd, vzero);
      vsum32           = _mm_add_epi32(vsum32, vsumtemp);
      piPred += iPredStride;
      piAd += iAdStride;
      piRec += iRecStride;
    }
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    sad    += _mm_cvtsi128_si32(vsum32);
  }
#if USE_AVX2
  else if (0 == (templateWidth % 16))
  {
    __m256i vzero  = _mm256_setzero_si256();
    __m256i vsum32 = vzero;
    for (int y = 0; y < height; y++)
    {
      __m256i vsum16 = vzero;
      for (int x = 0; x < templateWidth; x += 16)
      {
        __m256i vPred = _mm256_lddqu_si256((__m256i *) (piPred + x));   // why not aligned with 128/256 bit boundaries
        __m256i vRec  = _mm256_lddqu_si256((__m256i *) (piRec + x));
        __m256i vAd   = _mm256_abs_epi16(_mm256_sub_epi16(vRec, vPred));
        _mm256_storeu_si256((__m256i *) (piAd + x), vAd);

        vsum16 = _mm256_add_epi16(vsum16, vAd);
      }
      __m256i vsumtemp = _mm256_add_epi32(_mm256_unpacklo_epi16(vsum16, vzero), _mm256_unpackhi_epi16(vsum16, vzero));
      vsum32           = _mm256_add_epi32(vsum32, vsumtemp);
      piPred += iPredStride;
      piAd += iAdStride;
      piRec += iRecStride;
    }
    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    sad    += _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum32))
          + _mm_cvtsi128_si32(_mm256_castsi256_si128(_mm256_permute2x128_si256(vsum32, vsum32, 0x11)));
  }
#endif
  else if (0 == (templateWidth % 8))
  {
    __m128i vzero  = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    for (int y = 0; y < height; y++)
    {
      __m128i vsum16 = vzero;
      for (int x = 0; x < templateWidth; x += 8)
      {
        __m128i vPred = _mm_lddqu_si128((__m128i *) (piPred + x));
        __m128i vRec  = _mm_lddqu_si128((__m128i *) (piRec + x));
        __m128i vAd   = _mm_abs_epi16(_mm_sub_epi16(vRec, vPred));
        _mm_storeu_si128((__m128i *) (piAd + x), vAd);
        vsum16 = _mm_add_epi16(vsum16, vAd);
      }
      __m128i vsumtemp = _mm_add_epi32(_mm_unpacklo_epi16(vsum16, vzero), _mm_unpackhi_epi16(vsum16, vzero));
      vsum32           = _mm_add_epi32(vsum32, vsumtemp);
      piPred += iPredStride;
      piAd += iAdStride;
      piRec += iRecStride;
    }
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    sad  += _mm_cvtsi128_si32(vsum32);
  }
  else
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < templateWidth; x++)
      {
        *piAd = abs(*piRec - *piPred);
        sad += *piAd;
        piRec++;
        piPred++;
        piAd++;
      }

      piPred += (iPredStride - templateWidth);
      piAd += (iAdStride - templateWidth);
      piRec += (iRecStride - templateWidth);
    }
  }

  return sad;
}

template<X86_VEXT vext>
int xSgpmSadTM_SSE(const PredictionUnit &pu, const int width, const int height, const int templateWidth,
                    const int templateHeight, const ComponentID compIdx, const uint8_t splitDir, PelBuf &adBuf)
{
  int      sum        = 0;
  int16_t  wIdx   = floorLog2(pu.lwidth()) - GEO_MIN_CU_LOG2_EX;
  int16_t  hIdx   = floorLog2(pu.lheight()) - GEO_MIN_CU_LOG2_EX;
  int16_t  angle  = g_geoParams[splitDir][0];
  int16_t  stepY  = 0;
  int16_t  stepX  = 1;
  int16_t *weightMask = nullptr;

  if (g_angle2mirror[angle] == 2)
  {
    stepY = -GEO_WEIGHT_MASK_SIZE_EXT;
    weightMask = &g_geoWeightsTpl[g_angle2mask[angle]]
                                       [(GEO_WEIGHT_MASK_SIZE_EXT - 1
                                                       - g_weightOffsetEx[splitDir][hIdx][wIdx][1] - GEO_TM_ADDED_WEIGHT_MASK_SIZE)
                                                        * GEO_WEIGHT_MASK_SIZE_EXT
                                                      + g_weightOffsetEx[splitDir][hIdx][wIdx][0] + GEO_TM_ADDED_WEIGHT_MASK_SIZE];
  }
  else if (g_angle2mirror[angle] == 1)
  {
    stepX  = -1;
    stepY  = GEO_WEIGHT_MASK_SIZE_EXT;
    weightMask = &g_geoWeightsTpl[g_angle2mask[angle]]
                                     [(g_weightOffsetEx[splitDir][hIdx][wIdx][1] + GEO_TM_ADDED_WEIGHT_MASK_SIZE)
                                        * GEO_WEIGHT_MASK_SIZE_EXT
                                      + (GEO_WEIGHT_MASK_SIZE_EXT - 1 - g_weightOffsetEx[splitDir][hIdx][wIdx][0]
                                         - GEO_TM_ADDED_WEIGHT_MASK_SIZE)];
  }
  else
  {
    stepY = GEO_WEIGHT_MASK_SIZE_EXT;
    weightMask = &g_geoWeightsTpl[g_angle2mask[angle]]
                                       [(g_weightOffsetEx[splitDir][hIdx][wIdx][1] + GEO_TM_ADDED_WEIGHT_MASK_SIZE)
                                                        * GEO_WEIGHT_MASK_SIZE_EXT
                                                      + g_weightOffsetEx[splitDir][hIdx][wIdx][0] + GEO_TM_ADDED_WEIGHT_MASK_SIZE];
  }

  int32_t iAdStride = adBuf.stride;

  if (compIdx != COMPONENT_Y && pu.chromaFormat == CHROMA_420)
  {
    stepY <<= 1;
  }

  // top template
  Pel *piAd = adBuf.buf + templateWidth;   // start point of adBuf is (-templateWidth, -templateHeight) of current block
  int16_t *weightBackup = weightMask;
  weightMask            = weightMask - templateHeight * stepY;
  if (width == 4)
  {
    __m128i vzero  = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    for (int y = 0; y < templateHeight; y++)
    {
      __m128i vAd = _mm_loadl_epi64((__m128i *) (piAd));
      __m128i vMask;

      if (g_angle2mirror[angle] == 1)
      {
        vMask                      = _mm_loadl_epi64((__m128i *) (weightMask - (4 - 1)));
        const __m128i shuffle_mask = _mm_set_epi8(15, 14, 13, 12, 11, 10, 9, 8, 1, 0, 3, 2, 5, 4, 7, 6);
        vMask                      = _mm_shuffle_epi8(vMask, shuffle_mask);
      }
      else
      {
        vMask = _mm_loadl_epi64((__m128i *) weightMask);
      }
      vsum32 = _mm_add_epi32(vsum32, _mm_madd_epi16(vMask, vAd));

      piAd += iAdStride;
      weightMask += stepY;
    }
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    sum    = _mm_cvtsi128_si32(vsum32);
  }
#if USE_AVX2
  else if (0 == (width % 16))
  {
    __m256i vzero  = _mm256_setzero_si256();
    __m256i vsum32 = vzero;
    for (int y = 0; y < templateHeight; y++)
    {
      for (int x = 0; x < width; x += 16)
      {
        __m256i vAd = _mm256_lddqu_si256((__m256i *) (piAd + x));   

        __m256i vMask;

        if (g_angle2mirror[angle] == 1)
        {
          vMask                         = _mm256_lddqu_si256((__m256i *) (weightMask - x - (16 - 1)));
          const __m256i shuffle_mask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3, 2,
                                                       5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
          vMask                         = _mm256_shuffle_epi8(vMask, shuffle_mask);
          vMask                         = _mm256_permute4x64_epi64(vMask, _MM_SHUFFLE(1, 0, 3, 2));
        }
        else
        {
          vMask = _mm256_lddqu_si256((__m256i *) (weightMask + x));
        }
        vsum32 = _mm256_add_epi32(vsum32, _mm256_madd_epi16(vMask, vAd));
      }
      piAd += iAdStride;
      weightMask += stepY;
    }
    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    sum    = _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum32))
          + _mm_cvtsi128_si32(_mm256_castsi256_si128(_mm256_permute2x128_si256(vsum32, vsum32, 0x11)));
  }
#endif
  else
  {
    __m128i vzero  = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    for (int y = 0; y < templateHeight; y++)
    {
      for (int x = 0; x < width; x += 8)
      {
        __m128i vAd = _mm_lddqu_si128((__m128i *) (piAd + x));
        __m128i vMask;

        if (g_angle2mirror[angle] == 1)
        {
          vMask                         = _mm_lddqu_si128((__m128i *) (weightMask - x - (8 - 1)));
          const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
          vMask                         = _mm_shuffle_epi8(vMask, shuffle_mask);
        }
        else
        {
          vMask = _mm_lddqu_si128((__m128i *) (weightMask + x));
        }
        vsum32     = _mm_add_epi32(vsum32, _mm_madd_epi16(vMask, vAd));
      }
      piAd += iAdStride;
      weightMask += stepY;
    }
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    sum    = _mm_cvtsi128_si32(vsum32);
  }

  // left template
  piAd   = adBuf.buf + templateHeight * iAdStride;
  weightMask = weightBackup - templateWidth * stepX;
  if (templateWidth == 4)
  {
    __m128i vzero  = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    for (int y = 0; y < height; y++)
    {
      __m128i vAd = _mm_loadl_epi64((__m128i *) (piAd));
      __m128i vMask;

      if (g_angle2mirror[angle] == 1)
      {
        vMask                      = _mm_loadl_epi64((__m128i *) (weightMask - (4 - 1)));
        const __m128i shuffle_mask = _mm_set_epi8(15, 14, 13, 12, 11, 10, 9, 8, 1, 0, 3, 2, 5, 4, 7, 6);
        vMask                      = _mm_shuffle_epi8(vMask, shuffle_mask);
      }
      else
      {
        vMask = _mm_loadl_epi64((__m128i *) weightMask);
      }
      vsum32 = _mm_add_epi32(vsum32, _mm_madd_epi16(vMask, vAd));

      piAd += iAdStride;
      weightMask += stepY;
    }
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    sum    += _mm_cvtsi128_si32(vsum32);
  }
#if USE_AVX2
  else if (0 == (templateWidth % 16))
  {
    __m256i vzero  = _mm256_setzero_si256();
    __m256i vsum32 = vzero;
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < templateWidth; x += 16)
      {
        __m256i vAd = _mm256_lddqu_si256((__m256i *) (piAd + x));

        __m256i vMask;

        if (g_angle2mirror[angle] == 1)
        {
          vMask                      = _mm256_lddqu_si256((__m256i *) (weightMask - x - (16 - 1)));
          const __m256i shuffle_mask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3, 2,
                                                       5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
          vMask                      = _mm256_shuffle_epi8(vMask, shuffle_mask);
          vMask                      = _mm256_permute4x64_epi64(vMask, _MM_SHUFFLE(1, 0, 3, 2));
        }
        else
        {
          vMask = _mm256_lddqu_si256((__m256i *) (weightMask + x));
        }
        vsum32 = _mm256_add_epi32(vsum32, _mm256_madd_epi16(vMask, vAd));
      }
      piAd += iAdStride;
      weightMask += stepY;
    }
    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    vsum32 = _mm256_hadd_epi32(vsum32, vzero);
    sum    += _mm_cvtsi128_si32(_mm256_castsi256_si128(vsum32))
          + _mm_cvtsi128_si32(_mm256_castsi256_si128(_mm256_permute2x128_si256(vsum32, vsum32, 0x11)));
  }
#endif
  else if (0 == (templateWidth % 8))
  {
    __m128i vzero  = _mm_setzero_si128();
    __m128i vsum32 = vzero;
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < templateWidth; x += 8)
      {
        __m128i vAd = _mm_lddqu_si128((__m128i *) (piAd + x));
        __m128i vMask;

        if (g_angle2mirror[angle] == 1)
        {
          vMask                      = _mm_lddqu_si128((__m128i *) (weightMask - x - (8 - 1)));
          const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
          vMask                      = _mm_shuffle_epi8(vMask, shuffle_mask);
        }
        else
        {
          vMask = _mm_lddqu_si128((__m128i *) (weightMask + x));
        }
        vsum32 = _mm_add_epi32(vsum32, _mm_madd_epi16(vMask, vAd));
      }
      piAd += iAdStride;
      weightMask += stepY;
    }
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0x4e));   // 01001110
    vsum32 = _mm_add_epi32(vsum32, _mm_shuffle_epi32(vsum32, 0xb1));   // 10110001
    sum    += _mm_cvtsi128_si32(vsum32);
  }
  else
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < templateWidth; x++)
      {
        sum += *piAd * (*weightMask);
        piAd++;
        weightMask += stepX;
      }

      piAd += (iAdStride - templateWidth);
      weightMask += (stepY - templateWidth * stepX);
    }
  }

  return sum;
}

template<X86_VEXT vext>
void xWeightedSgpm_SSE(const PredictionUnit &pu, const uint32_t width, const uint32_t height,
                         const ComponentID compIdx, const uint8_t splitDir, PelBuf &predDst, PelBuf &predSrc0,
                         PelBuf &predSrc1)
{
  Pel *   dst        = predDst.buf;
  Pel *   src0       = predSrc0.buf;
  Pel *   src1       = predSrc1.buf;
  int32_t strideDst  = predDst.stride;
  int32_t strideSrc0 = predSrc0.stride;
  int32_t strideSrc1 = predSrc1.stride;

  //const char   log2WeightBase = 3;
  const ClpRng clpRng         = pu.cu->slice->clpRngs().comp[compIdx];

  const int32_t shiftWeighted  = 5;
  const int32_t offsetWeighted = 16;
  int16_t  wIdx   = floorLog2(pu.lwidth()) - GEO_MIN_CU_LOG2_EX;
  int16_t  hIdx   = floorLog2(pu.lheight()) - GEO_MIN_CU_LOG2_EX;
  int16_t  angle  = g_geoParams[splitDir][0];
  int16_t  stepY  = 0;
  int16_t *weight = nullptr;

#if JVET_AC0189_SGPM_NO_BLENDING
  int blendWIdx = 0;
  if (pu.cs->pps->getUseSgpmNoBlend())
  {
    blendWIdx = 0;
  }
  else
  {
    blendWIdx= GET_SGPM_BLD_IDX(pu.lwidth(), pu.lheight());
  }
#endif
  if (g_angle2mirror[angle] == 2)
  {
    stepY  = -GEO_WEIGHT_MASK_SIZE;
    weight = &g_geoWeights
#if JVET_AC0189_SGPM_NO_BLENDING
               [blendWIdx]
#else
               [GET_SGPM_BLD_IDX(pu.lwidth(), pu.lheight())]
#endif
               [g_angle2mask[angle]]
               [(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffsetEx[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE
                + g_weightOffsetEx[splitDir][hIdx][wIdx][0]];
  }
  else if (g_angle2mirror[angle] == 1)
  {
    stepY  = GEO_WEIGHT_MASK_SIZE;
    weight = &g_geoWeights
#if JVET_AC0189_SGPM_NO_BLENDING
               [blendWIdx]
#else
               [GET_SGPM_BLD_IDX(pu.lwidth(), pu.lheight())]
#endif
               [g_angle2mask[angle]][g_weightOffsetEx[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE
                                     + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffsetEx[splitDir][hIdx][wIdx][0])];
  }
  else
  {
    stepY  = GEO_WEIGHT_MASK_SIZE;
    weight = &g_geoWeights
#if JVET_AC0189_SGPM_NO_BLENDING
               [blendWIdx]
#else
               [GET_SGPM_BLD_IDX(pu.lwidth(), pu.lheight())]
#endif
               [g_angle2mask[angle]][g_weightOffsetEx[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE
                                     + g_weightOffsetEx[splitDir][hIdx][wIdx][0]];
  }
  const __m128i mmEight = _mm_set1_epi16(32);
  const __m128i mmOffset = _mm_set1_epi32(offsetWeighted);
  const __m128i mmShift  = _mm_cvtsi32_si128(shiftWeighted);
  const __m128i mmMin    = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax    = _mm_set1_epi16(clpRng.max);

  if (compIdx != COMPONENT_Y && pu.chromaFormat == CHROMA_420)
    stepY <<= 1;
  if (width == 4)
  {
    // for luma, to be confirmed
    for (int y = 0; y < height; y++)
    {
      __m128i s0 = _mm_loadl_epi64((__m128i *) (src0));
      __m128i s1 = _mm_loadl_epi64((__m128i *) (src1));
      __m128i w0;
      if (compIdx != COMPONENT_Y && pu.chromaFormat != CHROMA_444)
      {
        if (g_angle2mirror[angle] == 1)
        {
          w0                         = _mm_loadu_si128((__m128i *) (weight - (8 - 1)));
          const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
          w0                         = _mm_shuffle_epi8(w0, shuffle_mask);
        }
        else
        {
          w0 = _mm_loadu_si128((__m128i *) (weight));
        }
        w0         = _mm_shuffle_epi8(w0, _mm_setr_epi8(0, 1, 4, 5, 8, 9, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0));
      }
      else
      {
        if (g_angle2mirror[angle] == 1)
        {
          w0                         = _mm_loadl_epi64((__m128i *) (weight - (4 - 1)));
          const __m128i shuffle_mask = _mm_set_epi8(15,14,13,12,11,10,9,8,1,0,3,2,5,4,7,6);
          w0                         = _mm_shuffle_epi8(w0, shuffle_mask);
        }
        else
        {
          w0 = _mm_loadl_epi64((__m128i *) weight);
        }
      }
      
      __m128i w1 = _mm_sub_epi16(mmEight, w0);
      s0         = _mm_unpacklo_epi16(s0, s1);
      w0         = _mm_unpacklo_epi16(w0, w1);
      s0         = _mm_add_epi32(_mm_madd_epi16(s0, w0), mmOffset);
      s0         = _mm_sra_epi32(s0, mmShift);
      s0         = _mm_packs_epi32(s0, s0);
      s0         = _mm_min_epi16(mmMax, _mm_max_epi16(s0, mmMin));
      _mm_storel_epi64((__m128i *) (dst), s0);
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
#if USE_AVX2
  else if (0 == (width % 16))
  {
    const __m256i mmEightAVX2 = _mm256_set1_epi16(32);
    const __m256i mmOffsetAVX2 = _mm256_set1_epi32(offsetWeighted);
    const __m256i mmMinAVX2    = _mm256_set1_epi16(clpRng.min);
    const __m256i mmMaxAVX2    = _mm256_set1_epi16(clpRng.max);
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 16)
      {
        __m256i s0 = _mm256_lddqu_si256((__m256i *) (src0 + x));   // why not aligned with 128/256 bit boundaries
        __m256i s1 = _mm256_lddqu_si256((__m256i *) (src1 + x));

        __m256i w0 = _mm256_lddqu_si256((__m256i *) (weight + x));
        if (compIdx != COMPONENT_Y && pu.chromaFormat != CHROMA_444)
        {
          const __m256i mask = _mm256_set_epi16(0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1);
          __m256i       w0p0, w0p1;
          if (g_angle2mirror[angle] == 1)
          {
            w0p0 = _mm256_lddqu_si256(
              (__m256i *) (weight - (x << 1) - (16 - 1)));   // first sub-sample the required weights.
            w0p1                       = _mm256_lddqu_si256((__m256i *) (weight - (x << 1) - 16 - (16 - 1)));
            const __m256i shuffle_mask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3,
                                                         2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0p0                       = _mm256_shuffle_epi8(w0p0, shuffle_mask);
            w0p0                       = _mm256_permute4x64_epi64(w0p0, _MM_SHUFFLE(1, 0, 3, 2));
            w0p1                       = _mm256_shuffle_epi8(w0p1, shuffle_mask);
            w0p1                       = _mm256_permute4x64_epi64(w0p1, _MM_SHUFFLE(1, 0, 3, 2));
          }
          else
          {
            w0p0 = _mm256_lddqu_si256((__m256i *) (weight + (x << 1)));   // first sub-sample the required weights.
            w0p1 = _mm256_lddqu_si256((__m256i *) (weight + (x << 1) + 16));
          }
          w0p0 = _mm256_mullo_epi16(w0p0, mask);
          w0p1 = _mm256_mullo_epi16(w0p1, mask);
          w0   = _mm256_packs_epi16(w0p0, w0p1);
          w0   = _mm256_permute4x64_epi64(w0, _MM_SHUFFLE(3, 1, 2, 0));
        }
        else
        {
          if (g_angle2mirror[angle] == 1)
          {
            w0                         = _mm256_lddqu_si256((__m256i *) (weight - x - (16 - 1)));
            const __m256i shuffle_mask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3,
                                                         2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0                         = _mm256_shuffle_epi8(w0, shuffle_mask);
            w0                         = _mm256_permute4x64_epi64(w0, _MM_SHUFFLE(1, 0, 3, 2));
          }
          else
          {
            w0 = _mm256_lddqu_si256((__m256i *) (weight + x));
          }
        }
        __m256i w1 = _mm256_sub_epi16(mmEightAVX2, w0);

        __m256i s0tmp = _mm256_unpacklo_epi16(s0, s1);
        __m256i w0tmp = _mm256_unpacklo_epi16(w0, w1);
        s0tmp         = _mm256_add_epi32(_mm256_madd_epi16(s0tmp, w0tmp), mmOffsetAVX2);
        s0tmp         = _mm256_sra_epi32(s0tmp, mmShift);

        s0 = _mm256_unpackhi_epi16(s0, s1);
        w0 = _mm256_unpackhi_epi16(w0, w1);
        s0 = _mm256_add_epi32(_mm256_madd_epi16(s0, w0), mmOffsetAVX2);
        s0 = _mm256_sra_epi32(s0, mmShift);

        s0 = _mm256_packs_epi32(s0tmp, s0);
        s0 = _mm256_min_epi16(mmMaxAVX2, _mm256_max_epi16(s0, mmMinAVX2));
        _mm256_storeu_si256((__m256i *) (dst + x), s0);
      }
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
#endif
  else
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 8)
      {
        __m128i s0 = _mm_lddqu_si128((__m128i *) (src0 + x));
        __m128i s1 = _mm_lddqu_si128((__m128i *) (src1 + x));
        __m128i w0;
        if (compIdx != COMPONENT_Y && pu.chromaFormat != CHROMA_444)
        {
          const __m128i mask = _mm_set_epi16(0, 1, 0, 1, 0, 1, 0, 1);
          __m128i       w0p0, w0p1;
          if (g_angle2mirror[angle] == 1)
          {
            w0p0 =
              _mm_lddqu_si128((__m128i *) (weight - (x << 1) - (8 - 1)));   // first sub-sample the required weights.
            w0p1                       = _mm_lddqu_si128((__m128i *) (weight - (x << 1) - 8 - (8 - 1)));
            const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0p0                       = _mm_shuffle_epi8(w0p0, shuffle_mask);
            w0p1                       = _mm_shuffle_epi8(w0p1, shuffle_mask);
          }
          else
          {
            w0p0 = _mm_lddqu_si128((__m128i *) (weight + (x << 1)));   // first sub-sample the required weights.
            w0p1 = _mm_lddqu_si128((__m128i *) (weight + (x << 1) + 8));
          }
          w0p0 = _mm_mullo_epi16(w0p0, mask);
          w0p1 = _mm_mullo_epi16(w0p1, mask);
          w0   = _mm_packs_epi32(w0p0, w0p1);
        }
        else
        {
          if (g_angle2mirror[angle] == 1)
          {
            w0                         = _mm_lddqu_si128((__m128i *) (weight - x - (8 - 1)));
            const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0                         = _mm_shuffle_epi8(w0, shuffle_mask);
          }
          else
          {
            w0 = _mm_lddqu_si128((__m128i *) (weight + x));
          }
        }
        __m128i w1 = _mm_sub_epi16(mmEight, w0);

        __m128i s0tmp = _mm_unpacklo_epi16(s0, s1);
        __m128i w0tmp = _mm_unpacklo_epi16(w0, w1);
        s0tmp         = _mm_add_epi32(_mm_madd_epi16(s0tmp, w0tmp), mmOffset);
        s0tmp         = _mm_sra_epi32(s0tmp, mmShift);

        s0 = _mm_unpackhi_epi16(s0, s1);
        w0 = _mm_unpackhi_epi16(w0, w1);
        s0 = _mm_add_epi32(_mm_madd_epi16(s0, w0), mmOffset);
        s0 = _mm_sra_epi32(s0, mmShift);

        s0 = _mm_packs_epi32(s0tmp, s0);
        s0 = _mm_min_epi16(mmMax, _mm_max_epi16(s0, mmMin));
        _mm_storeu_si128((__m128i *) (dst + x), s0);
      }
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
}
#endif

template< X86_VEXT vext >
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
void xWeightedGeoBlk_SSE(const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, const uint8_t bldIdx, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1)
#else
void xWeightedGeoBlk_SSE(const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1)
#endif
{
  Pel* dst = predDst.get(compIdx).buf;
  Pel* src0 = predSrc0.get(compIdx).buf;
  Pel* src1 = predSrc1.get(compIdx).buf;
  int32_t strideDst = predDst.get(compIdx).stride;
  int32_t strideSrc0 = predSrc0.get(compIdx).stride;
  int32_t strideSrc1 = predSrc1.get(compIdx).stride;

#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
  const char    log2WeightBase = 5;
#else
  const char    log2WeightBase = 3;
#endif
  const ClpRng  clpRng = pu.cu->slice->clpRngs().comp[compIdx];
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const int32_t shiftWeighted = IF_INTERNAL_FRAC_BITS(clpRng.bd) + log2WeightBase;
#else
  const int32_t shiftWeighted = std::max<int>(2, (IF_INTERNAL_PREC - clpRng.bd)) + log2WeightBase;
#endif
  const int32_t offsetWeighted = (1 << (shiftWeighted - 1)) + (IF_INTERNAL_OFFS << log2WeightBase);

  int16_t wIdx = floorLog2(pu.lwidth()) - GEO_MIN_CU_LOG2;
  int16_t hIdx = floorLog2(pu.lheight()) - GEO_MIN_CU_LOG2;
  int16_t angle = g_geoParams[splitDir][0];
  int16_t stepY = 0;
  int16_t* weight = nullptr;
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
  if (g_angle2mirror[angle] == 2)
  {
    stepY = -GEO_WEIGHT_MASK_SIZE;
    weight = &g_geoWeights[bldIdx][g_angle2mask[angle]][(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
#if GPM_CURVE
    if(g_geoParams[splitDir][1] == 0 && wIdx == hIdx && wIdx >= GEO_CURVE_MIN_CU_IDX && CU::isGeoCURVEAvailable(splitDir,pu.lwidth()))
    {
      weight = &g_geoCurveWeights[bldIdx][g_angle2mask[angle]][(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
    }
#endif
  }
  else if (g_angle2mirror[angle] == 1)
  {
    stepY = GEO_WEIGHT_MASK_SIZE;
    weight = &g_geoWeights[bldIdx][g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
#if GPM_CURVE
    if(g_geoParams[splitDir][1] == 0 && wIdx == hIdx && wIdx >= GEO_CURVE_MIN_CU_IDX && CU::isGeoCURVEAvailable(splitDir,pu.lwidth()))
    {
      weight = &g_geoCurveWeights[bldIdx][g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
    }
#endif
  }
  else
  {
    stepY = GEO_WEIGHT_MASK_SIZE;
    weight = &g_geoWeights[bldIdx][g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
#if GPM_CURVE
    if(g_geoParams[splitDir][1] == 0 && wIdx == hIdx && wIdx >= GEO_CURVE_MIN_CU_IDX && CU::isGeoCURVEAvailable(splitDir,pu.lwidth()))
    {
      weight = &g_geoCurveWeights[bldIdx][g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
    }
#endif
  }
#else
  if (g_angle2mirror[angle] == 2)
  {
    stepY = -GEO_WEIGHT_MASK_SIZE;
    weight = &g_geoWeights[g_angle2mask[angle]][(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
  }
  else if (g_angle2mirror[angle] == 1)
  {
    stepY = GEO_WEIGHT_MASK_SIZE;
    weight = &g_geoWeights[g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
  }
  else
  {
    stepY = GEO_WEIGHT_MASK_SIZE;
    weight = &g_geoWeights[g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
  }
#endif

#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
  const __m128i mmEight = _mm_set1_epi16(32);
#else
  const __m128i mmEight = _mm_set1_epi16(8);
#endif
  const __m128i mmOffset = _mm_set1_epi32(offsetWeighted);
  const __m128i mmShift = _mm_cvtsi32_si128(shiftWeighted);
  const __m128i mmMin = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax = _mm_set1_epi16(clpRng.max);

  if (compIdx != COMPONENT_Y && pu.chromaFormat == CHROMA_420)
  {
    stepY <<= 1;
  }
  if (width == 4)
  {
    // it will occur to chroma only
    for (int y = 0; y < height; y++)
    {
      __m128i s0 = _mm_loadl_epi64((__m128i *) (src0));
      __m128i s1 = _mm_loadl_epi64((__m128i *) (src1));
      __m128i w0;
      if (g_angle2mirror[angle] == 1)
      {
        w0 = _mm_loadu_si128((__m128i *) (weight - (8 - 1)));
        const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
        w0 = _mm_shuffle_epi8(w0, shuffle_mask);
      }
      else
      {
        w0 = _mm_loadu_si128((__m128i *) (weight));
      }
      w0 = _mm_shuffle_epi8(w0, _mm_setr_epi8(0, 1, 4, 5, 8, 9, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0));
      __m128i w1 = _mm_sub_epi16(mmEight, w0);
      s0 = _mm_unpacklo_epi16(s0, s1);
      w0 = _mm_unpacklo_epi16(w0, w1);
      s0 = _mm_add_epi32(_mm_madd_epi16(s0, w0), mmOffset);
      s0 = _mm_sra_epi32(s0, mmShift);
      s0 = _mm_packs_epi32(s0, s0);
      s0 = _mm_min_epi16(mmMax, _mm_max_epi16(s0, mmMin));
      _mm_storel_epi64((__m128i *) (dst), s0);
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
#if USE_AVX2
  else if (width >= 16)
  {
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
    const __m256i mmEightAVX2 = _mm256_set1_epi16(32);
#else
    const __m256i mmEightAVX2 = _mm256_set1_epi16(8);
#endif
    const __m256i mmOffsetAVX2 = _mm256_set1_epi32(offsetWeighted);
    const __m256i mmMinAVX2 = _mm256_set1_epi16(clpRng.min);
    const __m256i mmMaxAVX2 = _mm256_set1_epi16(clpRng.max);
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 16)
      {
        __m256i s0 = _mm256_lddqu_si256((__m256i *) (src0 + x)); // why not aligned with 128/256 bit boundaries
        __m256i s1 = _mm256_lddqu_si256((__m256i *) (src1 + x));
        __m256i w0;
        if (compIdx != COMPONENT_Y &&  pu.chromaFormat != CHROMA_444)
        {
          const __m256i mask = _mm256_set_epi16(0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1);
          __m256i w0p0, w0p1;
          if (g_angle2mirror[angle] == 1)
          {
            w0p0 = _mm256_lddqu_si256((__m256i *) (weight - (x << 1) - (16 - 1))); // first sub-sample the required weights.
            w0p1 = _mm256_lddqu_si256((__m256i *) (weight - (x << 1) - 16 - (16 - 1)));
            const __m256i shuffle_mask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0p0 = _mm256_shuffle_epi8(w0p0, shuffle_mask);
            w0p0 = _mm256_permute4x64_epi64(w0p0, _MM_SHUFFLE(1, 0, 3, 2));
            w0p1 = _mm256_shuffle_epi8(w0p1, shuffle_mask);
            w0p1 = _mm256_permute4x64_epi64(w0p1, _MM_SHUFFLE(1, 0, 3, 2));
          }
          else
          {
            w0p0 = _mm256_lddqu_si256((__m256i *) (weight + (x << 1))); // first sub-sample the required weights.
            w0p1 = _mm256_lddqu_si256((__m256i *) (weight + (x << 1) + 16));
          }
          w0p0 = _mm256_mullo_epi16(w0p0, mask);
          w0p1 = _mm256_mullo_epi16(w0p1, mask);
          w0 = _mm256_packs_epi16(w0p0, w0p1);
          w0 = _mm256_permute4x64_epi64(w0, _MM_SHUFFLE(3, 1, 2, 0));
        }
        else
        {
          if (g_angle2mirror[angle] == 1)
          {
            w0 = _mm256_lddqu_si256((__m256i *) (weight - x - (16 - 1)));
            const __m256i shuffle_mask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0 = _mm256_shuffle_epi8(w0, shuffle_mask);
            w0 = _mm256_permute4x64_epi64(w0, _MM_SHUFFLE(1, 0, 3, 2));
          }
          else
          {
            w0 = _mm256_lddqu_si256((__m256i *) (weight + x));
          }
        }
        __m256i w1 = _mm256_sub_epi16(mmEightAVX2, w0);

        __m256i s0tmp = _mm256_unpacklo_epi16(s0, s1);
        __m256i w0tmp = _mm256_unpacklo_epi16(w0, w1);
        s0tmp = _mm256_add_epi32(_mm256_madd_epi16(s0tmp, w0tmp), mmOffsetAVX2);
        s0tmp = _mm256_sra_epi32(s0tmp, mmShift);

        s0 = _mm256_unpackhi_epi16(s0, s1);
        w0 = _mm256_unpackhi_epi16(w0, w1);
        s0 = _mm256_add_epi32(_mm256_madd_epi16(s0, w0), mmOffsetAVX2);
        s0 = _mm256_sra_epi32(s0, mmShift);

        s0 = _mm256_packs_epi32(s0tmp, s0);
        s0 = _mm256_min_epi16(mmMaxAVX2, _mm256_max_epi16(s0, mmMinAVX2));
        _mm256_storeu_si256((__m256i *) (dst + x), s0);
      }
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
#endif
  else
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 8)
      {
        __m128i s0 = _mm_lddqu_si128((__m128i *) (src0 + x));
        __m128i s1 = _mm_lddqu_si128((__m128i *) (src1 + x));
        __m128i w0;
        if (compIdx != COMPONENT_Y && pu.chromaFormat != CHROMA_444)
        {
          const __m128i mask = _mm_set_epi16(0, 1, 0, 1, 0, 1, 0, 1);
          __m128i w0p0, w0p1;
          if (g_angle2mirror[angle] == 1)
          {
            w0p0 = _mm_lddqu_si128((__m128i *) (weight - (x << 1) - (8 - 1))); // first sub-sample the required weights.
            w0p1 = _mm_lddqu_si128((__m128i *) (weight - (x << 1) - 8 - (8 - 1)));
            const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0p0 = _mm_shuffle_epi8(w0p0, shuffle_mask);
            w0p1 = _mm_shuffle_epi8(w0p1, shuffle_mask);
          }
          else
          {
            w0p0 = _mm_lddqu_si128((__m128i *) (weight + (x << 1))); // first sub-sample the required weights.
            w0p1 = _mm_lddqu_si128((__m128i *) (weight + (x << 1) + 8));
          }
          w0p0 = _mm_mullo_epi16(w0p0, mask);
          w0p1 = _mm_mullo_epi16(w0p1, mask);
          w0 = _mm_packs_epi32(w0p0, w0p1);
        }
        else
        {
          if (g_angle2mirror[angle] == 1)
          {
            w0 = _mm_lddqu_si128((__m128i *) (weight - x - (8 - 1)));
            const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0 = _mm_shuffle_epi8(w0, shuffle_mask);
          }
          else
          {
            w0 = _mm_lddqu_si128((__m128i *) (weight + x));
          }
        }
        __m128i w1 = _mm_sub_epi16(mmEight, w0);

        __m128i s0tmp = _mm_unpacklo_epi16(s0, s1);
        __m128i w0tmp = _mm_unpacklo_epi16(w0, w1);
        s0tmp = _mm_add_epi32(_mm_madd_epi16(s0tmp, w0tmp), mmOffset);
        s0tmp = _mm_sra_epi32(s0tmp, mmShift);

        s0 = _mm_unpackhi_epi16(s0, s1);
        w0 = _mm_unpackhi_epi16(w0, w1);
        s0 = _mm_add_epi32(_mm_madd_epi16(s0, w0), mmOffset);
        s0 = _mm_sra_epi32(s0, mmShift);

        s0 = _mm_packs_epi32(s0tmp, s0);
        s0 = _mm_min_epi16(mmMax, _mm_max_epi16(s0, mmMin));
        _mm_storeu_si128((__m128i *) (dst + x), s0);
      }
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
}

#if JVET_Y0065_GPM_INTRA
template< X86_VEXT vext >
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
void xWeightedGeoBlkRounded_SSE(const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, const uint8_t bldIdx, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1)
#else
void xWeightedGeoBlkRounded_SSE(const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1)
#endif
{
  Pel* dst = predDst.get(compIdx).buf;
  Pel* src0 = predSrc0.get(compIdx).buf;
  Pel* src1 = predSrc1.get(compIdx).buf;
  int32_t strideDst = predDst.get(compIdx).stride;
  int32_t strideSrc0 = predSrc0.get(compIdx).stride;
  int32_t strideSrc1 = predSrc1.get(compIdx).stride;

  int16_t wIdx = floorLog2(pu.lwidth()) - GEO_MIN_CU_LOG2;
  int16_t hIdx = floorLog2(pu.lheight()) - GEO_MIN_CU_LOG2;
  int16_t angle = g_geoParams[splitDir][0];
  int16_t stepY = 0;
  int16_t* weight = nullptr;
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
  if (g_angle2mirror[angle] == 2)
  {
    stepY = -GEO_WEIGHT_MASK_SIZE;
    weight = &g_geoWeights[bldIdx][g_angle2mask[angle]][(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
#if GPM_CURVE
    if(g_geoParams[splitDir][1] == 0 && wIdx == hIdx && wIdx >= GEO_CURVE_MIN_CU_IDX && CU::isGeoCURVEAvailable(splitDir,pu.lwidth()))
    {
      weight = &g_geoCurveWeights[bldIdx][g_angle2mask[angle]][(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
    }
#endif
  }
  else if (g_angle2mirror[angle] == 1)
  {
    stepY = GEO_WEIGHT_MASK_SIZE;
    weight = &g_geoWeights[bldIdx][g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
#if GPM_CURVE
    if(g_geoParams[splitDir][1] == 0 && wIdx == hIdx && wIdx >= GEO_CURVE_MIN_CU_IDX && CU::isGeoCURVEAvailable(splitDir,pu.lwidth())) 
    {
      weight = &g_geoCurveWeights[bldIdx][g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
    }
#endif
  }
  else
  {
    stepY = GEO_WEIGHT_MASK_SIZE;
    weight = &g_geoWeights[bldIdx][g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
#if GPM_CURVE
    if(g_geoParams[splitDir][1] == 0 && wIdx == hIdx && wIdx >= GEO_CURVE_MIN_CU_IDX && CU::isGeoCURVEAvailable(splitDir,pu.lwidth()))
    {
      weight = &g_geoCurveWeights[bldIdx][g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
    }
#endif
  }
#else
  if (g_angle2mirror[angle] == 2)
  {
    stepY = -GEO_WEIGHT_MASK_SIZE;
    weight = &g_geoWeights[g_angle2mask[angle]][(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
  }
  else if (g_angle2mirror[angle] == 1)
  {
    stepY = GEO_WEIGHT_MASK_SIZE;
    weight = &g_geoWeights[g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
  }
  else
  {
    stepY = GEO_WEIGHT_MASK_SIZE;
    weight = &g_geoWeights[g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
  }
#endif

#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
  const __m128i mmEight = _mm_set1_epi16(32);
  const __m128i mmOffset = _mm_set1_epi32(16);
  const __m128i mmShift = _mm_cvtsi32_si128(5);
#else
  const __m128i mmEight = _mm_set1_epi16(8);
  const __m128i mmOffset = _mm_set1_epi32(4);
  const __m128i mmShift = _mm_cvtsi32_si128(3);
#endif

  if (compIdx != COMPONENT_Y && pu.chromaFormat == CHROMA_420)
  {
    stepY <<= 1;
  }
  if (width == 4)
  {
    // it will occur to chroma only
    for (int y = 0; y < height; y++)
    {
      __m128i s0 = _mm_loadl_epi64((__m128i *) (src0));
      __m128i s1 = _mm_loadl_epi64((__m128i *) (src1));
      __m128i w0;
      if (g_angle2mirror[angle] == 1)
      {
        w0 = _mm_loadu_si128((__m128i *) (weight - (8 - 1)));
        const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
        w0 = _mm_shuffle_epi8(w0, shuffle_mask);
      }
      else
      {
        w0 = _mm_loadu_si128((__m128i *) (weight));
      }
      w0 = _mm_shuffle_epi8(w0, _mm_setr_epi8(0, 1, 4, 5, 8, 9, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0));
      __m128i w1 = _mm_sub_epi16(mmEight, w0);
      s0 = _mm_unpacklo_epi16(s0, s1);
      w0 = _mm_unpacklo_epi16(w0, w1);
      s0 = _mm_add_epi32(_mm_madd_epi16(s0, w0), mmOffset);
      s0 = _mm_sra_epi32(s0, mmShift);
      s0 = _mm_packs_epi32(s0, s0);
      _mm_storel_epi64((__m128i *) (dst), s0);
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
#if USE_AVX2
  else if (width >= 16)
  {
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
    const __m256i mmEightAVX2 = _mm256_set1_epi16(32);
    const __m256i mmOffsetAVX2 = _mm256_set1_epi32(16);
#else
    const __m256i mmEightAVX2 = _mm256_set1_epi16(8);
    const __m256i mmOffsetAVX2 = _mm256_set1_epi32(4);
#endif
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 16)
      {
        __m256i s0 = _mm256_lddqu_si256((__m256i *) (src0 + x)); // why not aligned with 128/256 bit boundaries
        __m256i s1 = _mm256_lddqu_si256((__m256i *) (src1 + x));
        __m256i w0;
        if (compIdx != COMPONENT_Y &&  pu.chromaFormat != CHROMA_444)
        {
          const __m256i mask = _mm256_set_epi16(0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1);
          __m256i w0p0, w0p1;
          if (g_angle2mirror[angle] == 1)
          {
            w0p0 = _mm256_lddqu_si256((__m256i *) (weight - (x << 1) - (16 - 1))); // first sub-sample the required weights.
            w0p1 = _mm256_lddqu_si256((__m256i *) (weight - (x << 1) - 16 - (16 - 1)));
            const __m256i shuffle_mask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0p0 = _mm256_shuffle_epi8(w0p0, shuffle_mask);
            w0p0 = _mm256_permute4x64_epi64(w0p0, _MM_SHUFFLE(1, 0, 3, 2));
            w0p1 = _mm256_shuffle_epi8(w0p1, shuffle_mask);
            w0p1 = _mm256_permute4x64_epi64(w0p1, _MM_SHUFFLE(1, 0, 3, 2));
          }
          else
          {
            w0p0 = _mm256_lddqu_si256((__m256i *) (weight + (x << 1))); // first sub-sample the required weights.
            w0p1 = _mm256_lddqu_si256((__m256i *) (weight + (x << 1) + 16));
          }
          w0p0 = _mm256_mullo_epi16(w0p0, mask);
          w0p1 = _mm256_mullo_epi16(w0p1, mask);
          w0 = _mm256_packs_epi16(w0p0, w0p1);
          w0 = _mm256_permute4x64_epi64(w0, _MM_SHUFFLE(3, 1, 2, 0));
        }
        else
        {
          if (g_angle2mirror[angle] == 1)
          {
            w0 = _mm256_lddqu_si256((__m256i *) (weight - x - (16 - 1)));
            const __m256i shuffle_mask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0 = _mm256_shuffle_epi8(w0, shuffle_mask);
            w0 = _mm256_permute4x64_epi64(w0, _MM_SHUFFLE(1, 0, 3, 2));
          }
          else
          {
            w0 = _mm256_lddqu_si256((__m256i *) (weight + x));
          }
        }
        __m256i w1 = _mm256_sub_epi16(mmEightAVX2, w0);

        __m256i s0tmp = _mm256_unpacklo_epi16(s0, s1);
        __m256i w0tmp = _mm256_unpacklo_epi16(w0, w1);
        s0tmp = _mm256_add_epi32(_mm256_madd_epi16(s0tmp, w0tmp), mmOffsetAVX2);
        s0tmp = _mm256_sra_epi32(s0tmp, mmShift);

        s0 = _mm256_unpackhi_epi16(s0, s1);
        w0 = _mm256_unpackhi_epi16(w0, w1);
        s0 = _mm256_add_epi32(_mm256_madd_epi16(s0, w0), mmOffsetAVX2);
        s0 = _mm256_sra_epi32(s0, mmShift);

        s0 = _mm256_packs_epi32(s0tmp, s0);
        _mm256_storeu_si256((__m256i *) (dst + x), s0);
      }
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
#endif
  else
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 8)
      {
        __m128i s0 = _mm_lddqu_si128((__m128i *) (src0 + x));
        __m128i s1 = _mm_lddqu_si128((__m128i *) (src1 + x));
        __m128i w0;
        if (compIdx != COMPONENT_Y && pu.chromaFormat != CHROMA_444)
        {
          const __m128i mask = _mm_set_epi16(0, 1, 0, 1, 0, 1, 0, 1);
          __m128i w0p0, w0p1;
          if (g_angle2mirror[angle] == 1)
          {
            w0p0 = _mm_lddqu_si128((__m128i *) (weight - (x << 1) - (8 - 1))); // first sub-sample the required weights.
            w0p1 = _mm_lddqu_si128((__m128i *) (weight - (x << 1) - 8 - (8 - 1)));
            const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0p0 = _mm_shuffle_epi8(w0p0, shuffle_mask);
            w0p1 = _mm_shuffle_epi8(w0p1, shuffle_mask);
          }
          else
          {
            w0p0 = _mm_lddqu_si128((__m128i *) (weight + (x << 1))); // first sub-sample the required weights.
            w0p1 = _mm_lddqu_si128((__m128i *) (weight + (x << 1) + 8));
          }
          w0p0 = _mm_mullo_epi16(w0p0, mask);
          w0p1 = _mm_mullo_epi16(w0p1, mask);
          w0 = _mm_packs_epi32(w0p0, w0p1);
        }
        else
        {
          if (g_angle2mirror[angle] == 1)
          {
            w0 = _mm_lddqu_si128((__m128i *) (weight - x - (8 - 1)));
            const __m128i shuffle_mask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
            w0 = _mm_shuffle_epi8(w0, shuffle_mask);
          }
          else
          {
            w0 = _mm_lddqu_si128((__m128i *) (weight + x));
          }
        }
        __m128i w1 = _mm_sub_epi16(mmEight, w0);

        __m128i s0tmp = _mm_unpacklo_epi16(s0, s1);
        __m128i w0tmp = _mm_unpacklo_epi16(w0, w1);
        s0tmp = _mm_add_epi32(_mm_madd_epi16(s0tmp, w0tmp), mmOffset);
        s0tmp = _mm_sra_epi32(s0tmp, mmShift);

        s0 = _mm_unpackhi_epi16(s0, s1);
        w0 = _mm_unpackhi_epi16(w0, w1);
        s0 = _mm_add_epi32(_mm_madd_epi16(s0, w0), mmOffset);
        s0 = _mm_sra_epi32(s0, mmShift);

        s0 = _mm_packs_epi32(s0tmp, s0);
        _mm_storeu_si128((__m128i *) (dst + x), s0);
      }
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
}
#endif

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
template< X86_VEXT vext, bool trueTFalseL >
void xWeightedGeoTpl_SSE(const PredictionUnit &pu, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1)
{
  const ComponentID compIdx = COMPONENT_Y;
  if (trueTFalseL == false || (predDst.bufs[compIdx].width & 7) != 0)
  {
    InterpolationFilter::xWeightedGeoTpl<trueTFalseL>(pu, splitDir, predDst, predSrc0, predSrc1);
    return;
  }

  Pel*    dst  = predDst.get(compIdx).buf;
  Pel*    src0 = predSrc0.get(compIdx).buf;
  Pel*    src1 = predSrc1.get(compIdx).buf;
  int32_t strideDst  = predDst .get(compIdx).stride;
  int32_t strideSrc0 = predSrc0.get(compIdx).stride;
  int32_t strideSrc1 = predSrc1.get(compIdx).stride;

  const uint32_t scaleX = getComponentScaleX(compIdx, pu.chromaFormat);
  const uint32_t scaleY = getComponentScaleY(compIdx, pu.chromaFormat);

  int16_t angle = g_geoParams[splitDir][0];
  int16_t wIdx  = floorLog2(pu.lwidth()) - GEO_MIN_CU_LOG2;
  int16_t hIdx  = floorLog2(pu.lheight()) - GEO_MIN_CU_LOG2;
  int16_t stepX = 1 << scaleX;
  int16_t stepY = 0;
  Pel*   weight = &g_geoWeightsTpl[g_angle2mask[angle]][GEO_TM_ADDED_WEIGHT_MASK_SIZE * GEO_WEIGHT_MASK_SIZE_EXT + GEO_TM_ADDED_WEIGHT_MASK_SIZE];
  if (g_angle2mirror[angle] == 2)
  {
    stepY = -(int)(GEO_WEIGHT_MASK_SIZE_EXT << scaleY);
    weight += ((GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE_EXT + g_weightOffset[splitDir][hIdx][wIdx][0]);
    weight += (trueTFalseL ? GEO_WEIGHT_MASK_SIZE_EXT * GEO_MODE_SEL_TM_SIZE : -GEO_MODE_SEL_TM_SIZE ); // Shift to template pos
  }
  else if (g_angle2mirror[angle] == 1)
  {
    stepX = -1 << scaleX;
    stepY = (GEO_WEIGHT_MASK_SIZE_EXT << scaleY);
    weight += (g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE_EXT + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0]));
    weight -= (trueTFalseL ? GEO_WEIGHT_MASK_SIZE_EXT * GEO_MODE_SEL_TM_SIZE : -GEO_MODE_SEL_TM_SIZE ); // Shift to template pos
  }
  else
  {
    stepY = (GEO_WEIGHT_MASK_SIZE_EXT << scaleY);
    weight += (g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE_EXT + g_weightOffset[splitDir][hIdx][wIdx][0]);
    weight -= (trueTFalseL ? GEO_WEIGHT_MASK_SIZE_EXT * GEO_MODE_SEL_TM_SIZE : GEO_MODE_SEL_TM_SIZE ); // Shift to template pos
  }

  int  rows = predDst.bufs[compIdx].height;
  int  cols = predDst.bufs[compIdx].width;

#ifdef USE_AVX2
  if (vext >= AVX2 && (cols & 15) == 0)
  {
    // Do for width that multiple of 16
    __m256i vzero = _mm256_setzero_si256();
    __m256i vone  = _mm256_set1_epi16(1);
    for (int y = 0; y < rows; y++)
    {
      for (int x = 0; x < cols; x += 16)
      {
        __m256i vsrc0 = _mm256_lddqu_si256( ( __m256i* )( &src0[x] ) );
        __m256i vsrc1 = _mm256_lddqu_si256( ( __m256i* )( &src1[x] ) );
        __m256i vmask;
        if (stepX < 0)
        {
          vmask = _mm256_lddqu_si256((__m256i*)((&weight[x]) - (x << 1) - (16 - 1)));
          const __m256i shuffleMask = _mm256_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14, 1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
          vmask = _mm256_shuffle_epi8(vmask, shuffleMask);
          vmask = _mm256_permute4x64_epi64(vmask, _MM_SHUFFLE(1, 0, 3, 2));
        }
        else
        {
          vmask = _mm256_lddqu_si256((__m256i*)(&weight[x]));
        }

        __m256i vtemp16Part0 = _mm256_and_si256( _mm256_sub_epi16( vzero, vmask ), vsrc0 );
        __m256i vtemp16Part1 = _mm256_and_si256( _mm256_sub_epi16( vmask, vone  ), vsrc1 );
        _mm256_storeu_si256( ( __m256i* )( &dst[x] ), _mm256_or_si256( vtemp16Part0, vtemp16Part1 ) );
      }

      src0 += strideSrc0;
      src1 += strideSrc1;
      dst  += strideDst;
      weight += stepY;
    }
  }
  else
#endif
  {
    // Do for width that multiple of 8
    __m128i vzero = _mm_setzero_si128();
    __m128i vone  = _mm_set1_epi16(1);
    for (int y = 0; y < rows; y++)
    {
      for (int x = 0; x < cols; x += 8)
      {
        __m128i vsrc0 = _mm_lddqu_si128( ( const __m128i* )( &src0[x] ) );
        __m128i vsrc1 = _mm_lddqu_si128( ( const __m128i* )( &src1[x] ) );
        __m128i vmask;
        if (stepX < 0)
        {
          vmask = _mm_lddqu_si128((__m128i*)((&weight[x]) - (x << 1) - (8 - 1)));
          const __m128i shuffleMask = _mm_set_epi8(1, 0, 3, 2, 5, 4, 7, 6, 9, 8, 11, 10, 13, 12, 15, 14);
          vmask = _mm_shuffle_epi8(vmask, shuffleMask);
        }
        else
        {
          vmask = _mm_lddqu_si128((const __m128i*)(&weight[x]));
        }

        __m128i vtemp16Part0 = _mm_and_si128( _mm_sub_epi16( vzero, vmask ), vsrc0 );
        __m128i vtemp16Part1 = _mm_and_si128( _mm_sub_epi16( vmask, vone  ), vsrc1 );
        _mm_storeu_si128( ( __m128i* )( &dst[x] ), _mm_or_si128( vtemp16Part0, vtemp16Part1 ) );
      }

      src0 += strideSrc0;
      src1 += strideSrc1;
      dst  += strideDst;
      weight += stepY;
    }
  }
}
#endif

#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
template< X86_VEXT vext >
void  xWeightedBlendBlk_SSE(const PredictionUnit& pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1, WeightBuf& weightBuf, const int log2WeightBase, const bool roundOutputBD)
{
  Pel* dst  = predDst.get(compIdx).buf;
  Pel* src0 = predSrc0.get(compIdx).buf;
  Pel* src1 = predSrc1.get(compIdx).buf;
  int32_t strideDst  = predDst.get(compIdx).stride;
  int32_t strideSrc0 = predSrc0.get(compIdx).stride;
  int32_t strideSrc1 = predSrc1.get(compIdx).stride;

  const ClpRng  clpRng = pu.cu->slice->clpRngs().comp[compIdx];
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const int32_t shiftWeighted = (roundOutputBD ? IF_INTERNAL_FRAC_BITS(clpRng.bd) : 0) + log2WeightBase;
#else
  const int32_t shiftWeighted = std::max<int>(2, (IF_INTERNAL_PREC - clpRng.bd)) + log2WeightBase;
#endif
  const int32_t offsetWeighted = (1 << (shiftWeighted - 1)) + (roundOutputBD ? (IF_INTERNAL_OFFS << log2WeightBase) : 0);

  int16_t* weight = weightBuf.buf;
  int16_t stepY   = weightBuf.stride;

#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
  const __m128i mmEight = _mm_set1_epi16(32);
#else
  const __m128i mmEight = _mm_set1_epi16(8);
#endif
  const __m128i mmOffset = _mm_set1_epi32(offsetWeighted);
  const __m128i mmShift = _mm_cvtsi32_si128(shiftWeighted);
  const __m128i mmMin = _mm_set1_epi16(clpRng.min);
  const __m128i mmMax = _mm_set1_epi16(clpRng.max);

  if (compIdx != COMPONENT_Y && pu.chromaFormat == CHROMA_420)
  {
    stepY <<= 1;
  }
  if (width == 4)
  {
    // it will occur to chroma only
    for (int y = 0; y < height; y++)
    {
      __m128i s0 = _mm_loadl_epi64((__m128i *) (src0));
      __m128i s1 = _mm_loadl_epi64((__m128i *) (src1));
      __m128i w0 = _mm_loadu_si128((__m128i *) (weight));
      w0 = _mm_shuffle_epi8(w0, _mm_setr_epi8(0, 1, 4, 5, 8, 9, 12, 13, 0, 0, 0, 0, 0, 0, 0, 0));
      __m128i w1 = _mm_sub_epi16(mmEight, w0);
      s0 = _mm_unpacklo_epi16(s0, s1);
      w0 = _mm_unpacklo_epi16(w1, w0);
      s0 = _mm_add_epi32(_mm_madd_epi16(s0, w0), mmOffset);
      s0 = _mm_sra_epi32(s0, mmShift);
      s0 = _mm_packs_epi32(s0, s0);
      s0 = _mm_min_epi16(mmMax, _mm_max_epi16(s0, mmMin));
      _mm_storel_epi64((__m128i *) (dst), s0);
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
#if USE_AVX2
  else if (width >= 16)
  {
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
    const __m256i mmEightAVX2 = _mm256_set1_epi16(32);
#else
    const __m256i mmEightAVX2 = _mm256_set1_epi16(8);
#endif
    const __m256i mmOffsetAVX2 = _mm256_set1_epi32(offsetWeighted);
    const __m256i mmMinAVX2 = _mm256_set1_epi16(clpRng.min);
    const __m256i mmMaxAVX2 = _mm256_set1_epi16(clpRng.max);
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 16)
      {
        __m256i s0 = _mm256_lddqu_si256((__m256i *) (src0 + x)); // why not aligned with 128/256 bit boundaries
        __m256i s1 = _mm256_lddqu_si256((__m256i *) (src1 + x));
        __m256i w0;
        if (compIdx != COMPONENT_Y &&  pu.chromaFormat != CHROMA_444)
        {
          const __m256i mask = _mm256_set_epi16(0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1);
          __m256i w0p0, w0p1;
          w0p0 = _mm256_lddqu_si256((__m256i *) (weight + (x << 1))); // first sub-sample the required weights.
          w0p1 = _mm256_lddqu_si256((__m256i *) (weight + (x << 1) + 16));
          w0p0 = _mm256_mullo_epi16(w0p0, mask);
          w0p1 = _mm256_mullo_epi16(w0p1, mask);
          w0 = _mm256_packs_epi16(w0p0, w0p1);
          w0 = _mm256_permute4x64_epi64(w0, _MM_SHUFFLE(3, 1, 2, 0));
        }
        else
        {
          w0 = _mm256_lddqu_si256((__m256i *) (weight + x));
        }
        __m256i w1 = _mm256_sub_epi16(mmEightAVX2, w0);

        __m256i s0tmp = _mm256_unpacklo_epi16(s0, s1);
        __m256i w0tmp = _mm256_unpacklo_epi16(w1, w0);
        s0tmp = _mm256_add_epi32(_mm256_madd_epi16(s0tmp, w0tmp), mmOffsetAVX2);
        s0tmp = _mm256_sra_epi32(s0tmp, mmShift);

        s0 = _mm256_unpackhi_epi16(s0, s1);
        w0 = _mm256_unpackhi_epi16(w1, w0);
        s0 = _mm256_add_epi32(_mm256_madd_epi16(s0, w0), mmOffsetAVX2);
        s0 = _mm256_sra_epi32(s0, mmShift);

        s0 = _mm256_packs_epi32(s0tmp, s0);
        s0 = _mm256_min_epi16(mmMaxAVX2, _mm256_max_epi16(s0, mmMinAVX2));
        _mm256_storeu_si256((__m256i *) (dst + x), s0);
      }
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
#endif
  else
  {
    for (int y = 0; y < height; y++)
    {
      for (int x = 0; x < width; x += 8)
      {
        __m128i s0 = _mm_lddqu_si128((__m128i *) (src0 + x));
        __m128i s1 = _mm_lddqu_si128((__m128i *) (src1 + x));
        __m128i w0;
        if (compIdx != COMPONENT_Y && pu.chromaFormat != CHROMA_444)
        {
          const __m128i mask = _mm_set_epi16(0, 1, 0, 1, 0, 1, 0, 1);
          __m128i w0p0, w0p1;
          w0p0 = _mm_lddqu_si128((__m128i *) (weight + (x << 1))); // first sub-sample the required weights.
          w0p1 = _mm_lddqu_si128((__m128i *) (weight + (x << 1) + 8));
          w0p0 = _mm_mullo_epi16(w0p0, mask);
          w0p1 = _mm_mullo_epi16(w0p1, mask);
          w0 = _mm_packs_epi32(w0p0, w0p1);
        }
        else
        {
          w0 = _mm_lddqu_si128((__m128i *) (weight + x));
        }
        __m128i w1 = _mm_sub_epi16(mmEight, w0);

        __m128i s0tmp = _mm_unpacklo_epi16(s0, s1);
        __m128i w0tmp = _mm_unpacklo_epi16(w1, w0);
        s0tmp = _mm_add_epi32(_mm_madd_epi16(s0tmp, w0tmp), mmOffset);
        s0tmp = _mm_sra_epi32(s0tmp, mmShift);

        s0 = _mm_unpackhi_epi16(s0, s1);
        w0 = _mm_unpackhi_epi16(w1, w0);
        s0 = _mm_add_epi32(_mm_madd_epi16(s0, w0), mmOffset);
        s0 = _mm_sra_epi32(s0, mmShift);

        s0 = _mm_packs_epi32(s0tmp, s0);
        s0 = _mm_min_epi16(mmMax, _mm_max_epi16(s0, mmMin));
        _mm_storeu_si128((__m128i *) (dst + x), s0);
      }
      dst += strideDst;
      src0 += strideSrc0;
      src1 += strideSrc1;
      weight += stepY;
    }
  }
}

template<X86_VEXT vext>
void  xWeightAffineBlk_SSE( const PredictionUnit& pu, WeightBuf& bufWeight, const int bcwBlendingLog2WeightBase, AffineBlendingModel& blendModel )
{
  int* param  = blendModel.params;
  int offset  = blendModel.offset;
  int shift   = blendModel.shift;
  int min     = blendModel.min;
  int max     = blendModel.max;

  const int biasPlusOffset = param[2] + offset;
  const int x0 = 0;
  const int y0 = 0;

  int width   = pu.lwidth();
  int height  = pu.lheight();

  int16_t*  bcwBlendBuf = bufWeight.buf;
  int       stride      = bufWeight.stride;

  if ( width % 8 )
  {
    printf("computeWeightMap_X86( width = %d ) not implemented.", width );
    exit(0);
  }
#if USE_AVX2
  else if ( (width % 16) == 0 )
  {
    const __m256i mmMin = _mm256_set1_epi16( min );
    const __m256i mmMax = _mm256_set1_epi16( max );

    __m256i vOne    = _mm256_set1_epi32( 1 );
    __m256i vEight  = _mm256_set1_epi32( 8 );
    __m256i vSixt   = _mm256_set1_epi32( 16 );

    __m256i vpar0 = _mm256_set1_epi32( param[0] ); // 8 32-bits values
    __m256i vpar1 = _mm256_set1_epi32( param[1] );
    __m256i bias0 = _mm256_set1_epi32( biasPlusOffset ); // 8 32-bits values

    __m256i vparXA = _mm256_set_epi32( x0 + 7, x0 + 6, x0 + 5, x0 + 4, x0 + 3, x0 + 2, x0 + 1, x0 + 0 );
    __m256i vparXB = _mm256_add_epi32( vparXA, vEight ); // X = X + 8
    __m256i vparY  = _mm256_set1_epi32( y0 );

    const int imm8_control = 0b11011000;

    for (int row = 0; row < height; row++)
    {
      __m256i vparX0 = vparXA;
      __m256i vparX1 = vparXB;
      __m256i vsum03 = _mm256_mullo_epi32( vparY, vpar1 );  // b.Y
      for (int col = 0; col < width; col += 16)
      {
        __m256i vsum01 = _mm256_add_epi32( _mm256_mullo_epi32( vparX0, vpar0 ), vsum03 ); // a.X + b.Y
        __m256i vsum02 = _mm256_add_epi32( _mm256_mullo_epi32( vparX1, vpar0 ), vsum03 ); // a.X + b.Y

        vsum01 = _mm256_add_epi32( vsum01, bias0 );  // a.X + b.Y + c
        vsum02 = _mm256_add_epi32( vsum02, bias0 );  // a.X + b.Y + c

        vsum01 = _mm256_srai_epi32( vsum01, shift );
        vsum02 = _mm256_srai_epi32( vsum02, shift );

        vsum01 = _mm256_packs_epi32( vsum01, vsum02 );

        vsum01 = _mm256_permute4x64_epi64( vsum01, imm8_control );

        vsum01 = _mm256_min_epi16( mmMax, _mm256_max_epi16( vsum01, mmMin ) );  // clipping

        _mm256_storeu_si256( (__m256i*)(bcwBlendBuf + col), vsum01 );

        vparX0 = _mm256_add_epi32( vparX0, vSixt ); // X = X + 16
        vparX1 = _mm256_add_epi32( vparX1, vSixt ); // X = X + 16
      }

      vparY = _mm256_add_epi32( vparY, vOne ); // increment by 1

      bcwBlendBuf += stride;
    }

  }
#endif
  else if ( (width % 8) == 0 )
  {

    const __m128i mmMin = _mm_set1_epi16( min );
    const __m128i mmMax = _mm_set1_epi16( max );

    __m128i vOne    = _mm_set1_epi32( 1 );
    __m128i vEight  = _mm_set1_epi32( 8 );

    __m128i vpar0 = _mm_set1_epi32( param[0] ); // 4 32-bits values
    __m128i vpar1 = _mm_set1_epi32( param[1] );
    __m128i bias0 = _mm_set1_epi32( biasPlusOffset ); // 4 32-bits values

    __m128i vparXA = _mm_set_epi32( x0 + 3, x0 + 2, x0 + 1, x0 + 0 );
    __m128i vparXB = _mm_set_epi32( x0 + 7, x0 + 6, x0 + 5, x0 + 4 );
    __m128i vparY  = _mm_set1_epi32( y0 );

    for (int row = 0; row < height; row++)
    {
      __m128i vparX0 = vparXA;
      __m128i vparX1 = vparXB;
      for (int col = 0; col < width; col += 8)
      {
        __m128i vsum03 = _mm_mullo_epi32( vparY, vpar1 );
        __m128i vsum01 = _mm_add_epi32( _mm_mullo_epi32( vparX0, vpar0 ), vsum03 ); // a.X + b.Y
        __m128i vsum02 = _mm_add_epi32( _mm_mullo_epi32( vparX1, vpar0 ), vsum03 ); // a.X + b.Y

        vsum01 = _mm_add_epi32( vsum01, bias0 );  // a.X + b.Y + c
        vsum02 = _mm_add_epi32( vsum02, bias0 );  // a.X + b.Y + c

        vsum01 = _mm_srai_epi32( vsum01, shift );
        vsum02 = _mm_srai_epi32( vsum02, shift );

        vsum01 = _mm_packs_epi32( vsum01, vsum02 );

        vsum01 = _mm_min_epi16( mmMax, _mm_max_epi16( vsum01, mmMin ) );  // clipping

        _mm_storeu_si128( (__m128i*)(bcwBlendBuf + col), vsum01 );

        vparX0 = _mm_add_epi32( vparX0, vEight ); // X = X + 8
        vparX1 = _mm_add_epi32( vparX1, vEight ); // X = X + 8
      }

      vparY = _mm_add_epi32( vparY, vOne ); // increment by 1

      bcwBlendBuf += stride;
    }

  }
}

#endif

template <X86_VEXT vext>
void InterpolationFilter::_initInterpolationFilterX86()
{
#if IF_12TAP_SIMD
#if SIMD_4x4_12
  m_filter4x4[0] = simdInterpolate4x4_12tap<false>;
  m_filter4x4[1] = simdInterpolate4x4_12tap<true>;
#endif
  m_filterHor[0][0][0] = simdFilter<vext, 12, false, false, false>;
  m_filterHor[0][0][1] = simdFilter<vext, 12, false, false, true>;
  m_filterHor[0][1][0] = simdFilter<vext, 12, false, true, false>;
  m_filterHor[0][1][1] = simdFilter<vext, 12, false, true, true>;

  m_filterHor[1][0][0] = simdFilter<vext, 8, false, false, false>;
  m_filterHor[1][0][1] = simdFilter<vext, 8, false, false, true>;
  m_filterHor[1][1][0] = simdFilter<vext, 8, false, true, false>;
  m_filterHor[1][1][1] = simdFilter<vext, 8, false, true, true>;
#if JVET_Z0117_CHROMA_IF
  m_filterHor[2][0][0] = simdFilter<vext, 6, false, false, false>;
  m_filterHor[2][0][1] = simdFilter<vext, 6, false, false, true>;
  m_filterHor[2][1][0] = simdFilter<vext, 6, false, true, false>;
  m_filterHor[2][1][1] = simdFilter<vext, 6, false, true, true>;
  
  m_filterHor[3][0][0] = simdFilter<vext, 4, false, false, false>;
  m_filterHor[3][0][1] = simdFilter<vext, 4, false, false, true>;
  m_filterHor[3][1][0] = simdFilter<vext, 4, false, true, false>;
  m_filterHor[3][1][1] = simdFilter<vext, 4, false, true, true>;

  m_filterHor[4][0][0] = simdFilter<vext, 2, false, false, false>;
  m_filterHor[4][0][1] = simdFilter<vext, 2, false, false, true>;
  m_filterHor[4][1][0] = simdFilter<vext, 2, false, true, false>;
  m_filterHor[4][1][1] = simdFilter<vext, 2, false, true, true>;
#else
  m_filterHor[2][0][0] = simdFilter<vext, 4, false, false, false>;
  m_filterHor[2][0][1] = simdFilter<vext, 4, false, false, true>;
  m_filterHor[2][1][0] = simdFilter<vext, 4, false, true, false>;
  m_filterHor[2][1][1] = simdFilter<vext, 4, false, true, true>;

  m_filterHor[3][0][0] = simdFilter<vext, 2, false, false, false>;
  m_filterHor[3][0][1] = simdFilter<vext, 2, false, false, true>;
  m_filterHor[3][1][0] = simdFilter<vext, 2, false, true, false>;
  m_filterHor[3][1][1] = simdFilter<vext, 2, false, true, true>;
#endif
  m_filterVer[0][0][0] = simdFilter<vext, 12, true, false, false>;
  m_filterVer[0][0][1] = simdFilter<vext, 12, true, false, true>;
  m_filterVer[0][1][0] = simdFilter<vext, 12, true, true, false>;
  m_filterVer[0][1][1] = simdFilter<vext, 12, true, true, true>;

  m_filterVer[1][0][0] = simdFilter<vext, 8, true, false, false>;
  m_filterVer[1][0][1] = simdFilter<vext, 8, true, false, true>;
  m_filterVer[1][1][0] = simdFilter<vext, 8, true, true, false>;
  m_filterVer[1][1][1] = simdFilter<vext, 8, true, true, true>;
#if JVET_Z0117_CHROMA_IF
  m_filterVer[2][0][0] = simdFilter<vext, 6, true, false, false>;
  m_filterVer[2][0][1] = simdFilter<vext, 6, true, false, true>;
  m_filterVer[2][1][0] = simdFilter<vext, 6, true, true, false>;
  m_filterVer[2][1][1] = simdFilter<vext, 6, true, true, true>;
  
  m_filterVer[3][0][0] = simdFilter<vext, 4, true, false, false>;
  m_filterVer[3][0][1] = simdFilter<vext, 4, true, false, true>;
  m_filterVer[3][1][0] = simdFilter<vext, 4, true, true, false>;
  m_filterVer[3][1][1] = simdFilter<vext, 4, true, true, true>;

  m_filterVer[4][0][0] = simdFilter<vext, 2, true, false, false>;
  m_filterVer[4][0][1] = simdFilter<vext, 2, true, false, true>;
  m_filterVer[4][1][0] = simdFilter<vext, 2, true, true, false>;
  m_filterVer[4][1][1] = simdFilter<vext, 2, true, true, true>;
#else
  m_filterVer[2][0][0] = simdFilter<vext, 4, true, false, false>;
  m_filterVer[2][0][1] = simdFilter<vext, 4, true, false, true>;
  m_filterVer[2][1][0] = simdFilter<vext, 4, true, true, false>;
  m_filterVer[2][1][1] = simdFilter<vext, 4, true, true, true>;

  m_filterVer[3][0][0] = simdFilter<vext, 2, true, false, false>;
  m_filterVer[3][0][1] = simdFilter<vext, 2, true, false, true>;
  m_filterVer[3][1][0] = simdFilter<vext, 2, true, true, false>;
  m_filterVer[3][1][1] = simdFilter<vext, 2, true, true, true>;
#endif
  m_filterCopy[0][0] = simdFilterCopy<vext, false, false>;
  m_filterCopy[0][1] = simdFilterCopy<vext, false, true>;
  m_filterCopy[1][0] = simdFilterCopy<vext, true, false>;
  m_filterCopy[1][1] = simdFilterCopy<vext, true, true>;
#if JVET_AC0104_IBC_BVD_PREDICTION
  m_filterCopyWithNoClipping = simdFilterCopyWithNoClipping<vext>;
#endif
#else
  // [taps][bFirst][bLast]
  m_filterHor[0][0][0] = simdFilter<vext, 8, false, false, false>;
  m_filterHor[0][0][1] = simdFilter<vext, 8, false, false, true>;
  m_filterHor[0][1][0] = simdFilter<vext, 8, false, true, false>;
  m_filterHor[0][1][1] = simdFilter<vext, 8, false, true, true>;
#if JVET_Z0117_CHROMA_IF
  m_filterHor[1][0][0] = simdFilter<vext, 6, false, false, false>;
  m_filterHor[1][0][1] = simdFilter<vext, 6, false, false, true>;
  m_filterHor[1][1][0] = simdFilter<vext, 6, false, true, false>;
  m_filterHor[1][1][1] = simdFilter<vext, 6, false, true, true>;
  
  m_filterHor[2][0][0] = simdFilter<vext, 4, false, false, false>;
  m_filterHor[2][0][1] = simdFilter<vext, 4, false, false, true>;
  m_filterHor[2][1][0] = simdFilter<vext, 4, false, true, false>;
  m_filterHor[2][1][1] = simdFilter<vext, 4, false, true, true>;

  m_filterHor[3][0][0] = simdFilter<vext, 2, false, false, false>;
  m_filterHor[3][0][1] = simdFilter<vext, 2, false, false, true>;
  m_filterHor[3][1][0] = simdFilter<vext, 2, false, true, false>;
  m_filterHor[3][1][1] = simdFilter<vext, 2, false, true, true>;
#else
  m_filterHor[1][0][0] = simdFilter<vext, 4, false, false, false>;
  m_filterHor[1][0][1] = simdFilter<vext, 4, false, false, true>;
  m_filterHor[1][1][0] = simdFilter<vext, 4, false, true, false>;
  m_filterHor[1][1][1] = simdFilter<vext, 4, false, true, true>;

  m_filterHor[2][0][0] = simdFilter<vext, 2, false, false, false>;
  m_filterHor[2][0][1] = simdFilter<vext, 2, false, false, true>;
  m_filterHor[2][1][0] = simdFilter<vext, 2, false, true, false>;
  m_filterHor[2][1][1] = simdFilter<vext, 2, false, true, true>;
#endif
  m_filterVer[0][0][0] = simdFilter<vext, 8, true, false, false>;
  m_filterVer[0][0][1] = simdFilter<vext, 8, true, false, true>;
  m_filterVer[0][1][0] = simdFilter<vext, 8, true, true, false>;
  m_filterVer[0][1][1] = simdFilter<vext, 8, true, true, true>;
  m_filterVer[0][1][1] = simdFilter<vext, 8, true, true, true>;
#if JVET_Z0117_CHROMA_IF
  m_filterVer[1][0][0] = simdFilter<vext, 6, true, false, false>;
  m_filterVer[1][0][1] = simdFilter<vext, 6, true, false, true>;
  m_filterVer[1][1][0] = simdFilter<vext, 6, true, true, false>;
  m_filterVer[1][1][1] = simdFilter<vext, 6, true, true, true>;
  
  m_filterVer[2][0][0] = simdFilter<vext, 4, true, false, false>;
  m_filterVer[2][0][1] = simdFilter<vext, 4, true, false, true>;
  m_filterVer[2][1][0] = simdFilter<vext, 4, true, true, false>;
  m_filterVer[2][1][1] = simdFilter<vext, 4, true, true, true>;

  m_filterVer[3][0][0] = simdFilter<vext, 2, true, false, false>;
  m_filterVer[3][0][1] = simdFilter<vext, 2, true, false, true>;
  m_filterVer[3][1][0] = simdFilter<vext, 2, true, true, false>;
  m_filterVer[3][1][1] = simdFilter<vext, 2, true, true, true>;
#else
  m_filterVer[1][0][0] = simdFilter<vext, 4, true, false, false>;
  m_filterVer[1][0][1] = simdFilter<vext, 4, true, false, true>;
  m_filterVer[1][1][0] = simdFilter<vext, 4, true, true, false>;
  m_filterVer[1][1][1] = simdFilter<vext, 4, true, true, true>;

  m_filterVer[2][0][0] = simdFilter<vext, 2, true, false, false>;
  m_filterVer[2][0][1] = simdFilter<vext, 2, true, false, true>;
  m_filterVer[2][1][0] = simdFilter<vext, 2, true, true, false>;
  m_filterVer[2][1][1] = simdFilter<vext, 2, true, true, true>;
#endif
  m_filterCopy[0][0]   = simdFilterCopy<vext, false, false>;
  m_filterCopy[0][1]   = simdFilterCopy<vext, false, true>;
  m_filterCopy[1][0]   = simdFilterCopy<vext, true, false>;
  m_filterCopy[1][1]   = simdFilterCopy<vext, true, true>;
#if JVET_AC0104_IBC_BVD_PREDICTION
  m_filterCopyWithNoClipping = simdFilterCopyWithNoClipping<vext>;
#endif
#endif
  m_weightedGeoBlk = xWeightedGeoBlk_SSE<vext>;
#if JVET_Y0065_GPM_INTRA
  m_weightedGeoBlkRounded = xWeightedGeoBlkRounded_SSE<vext>;
#endif
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  m_weightedBlendBlk = xWeightedBlendBlk_SSE<vext>;
  m_weightAffineBlk  = xWeightAffineBlk_SSE<vext>;
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  m_weightedGeoTplA = xWeightedGeoTpl_SSE<vext, true>;
#endif
#if JVET_AB0155_SGPM
  m_weightedSgpm = xWeightedSgpm_SSE<vext>;
  m_sadTM = xSadTM_SSE<vext>;
  m_sgpmSadTM = xSgpmSadTM_SSE<vext>;
#endif
}

template void InterpolationFilter::_initInterpolationFilterX86<SIMDX86>();

#endif //#ifdef TARGET_SIMD_X86
//! \}
