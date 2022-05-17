/* The copyright in this software is being made available under the BSD
 * License, included below. This software may be subject to other third party
 * and contributor rights, including patent rights, and no such rights are
 * granted under this license.
 *
 * Copyright (c) 2010-2022, ITU/ISO/IEC
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

// ====================================================================================================================
// Includes
// ====================================================================================================================

#include "Rom.h"
#include "InterpolationFilter.h"

#include "ChromaFormat.h"

#if JVET_J0090_MEMORY_BANDWITH_MEASURE
CacheModel* InterpolationFilter::m_cacheModel;
#endif
//! \ingroup CommonLib
//! \{

// ====================================================================================================================
// Tables
// ====================================================================================================================
#if IF_12TAP
const TFilterCoeff InterpolationFilter::m_lumaFilter4x4[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][8] =
#else
const TFilterCoeff InterpolationFilter::m_lumaFilter4x4[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA] =
#endif
{
#if IF_12TAP
  {  0, 0,   0, 64 * 4,  0,   0,  0,  0 },
  {  0, 1 * 4,  -3 * 4, 63 * 4,  4 * 4,  -2 * 4,  1 * 4,  0 },
  {  0, 1 * 4,  -5 * 4, 62 * 4,  8 * 4,  -3 * 4,  1 * 4,  0 },
  {  0, 2 * 4,  -8 * 4, 60 * 4, 13 * 4,  -4 * 4,  1 * 4,  0 },
  {  0, 3 * 4, -10 * 4, 58 * 4, 17 * 4,  -5 * 4,  1 * 4,  0 }, //1/4
  {  0, 3 * 4, -11 * 4, 52 * 4, 26 * 4,  -8 * 4,  2 * 4,  0 },
  {  0, 2 * 4,  -9 * 4, 47 * 4, 31 * 4, -10 * 4,  3 * 4,  0 },
  {  0, 3 * 4, -11 * 4, 45 * 4, 34 * 4, -10 * 4,  3 * 4,  0 },
  {  0, 3 * 4, -11 * 4, 40 * 4, 40 * 4, -11 * 4,  3 * 4,  0 }, //1/2
  {  0, 3 * 4, -10 * 4, 34 * 4, 45 * 4, -11 * 4,  3 * 4,  0 },
  {  0, 3 * 4, -10 * 4, 31 * 4, 47 * 4,  -9 * 4,  2 * 4,  0 },
  {  0, 2 * 4,  -8 * 4, 26 * 4, 52 * 4, -11 * 4,  3 * 4,  0 },
  {  0, 1 * 4,  -5 * 4, 17 * 4, 58 * 4, -10 * 4,  3 * 4,  0 }, //3/4
  {  0, 1 * 4,  -4 * 4, 13 * 4, 60 * 4,  -8 * 4,  2 * 4,  0 },
  {  0, 1 * 4,  -3 * 4,  8 * 4, 62 * 4,  -5 * 4,  1 * 4,  0 },
  {  0, 1 * 4,  -2 * 4,  4 * 4, 63 * 4,  -3 * 4,  1 * 4,  0 }
#else
  {  0, 0,   0, 64,  0,   0,  0,  0 },
  {  0, 1,  -3, 63,  4,  -2,  1,  0 },
  {  0, 1,  -5, 62,  8,  -3,  1,  0 },
  {  0, 2,  -8, 60, 13,  -4,  1,  0 },
  {  0, 3, -10, 58, 17,  -5,  1,  0 }, //1/4
  {  0, 3, -11, 52, 26,  -8,  2,  0 },
  {  0, 2,  -9, 47, 31, -10,  3,  0 },
  {  0, 3, -11, 45, 34, -10,  3,  0 },
  {  0, 3, -11, 40, 40, -11,  3,  0 }, //1/2
  {  0, 3, -10, 34, 45, -11,  3,  0 },
  {  0, 3, -10, 31, 47,  -9,  2,  0 },
  {  0, 2,  -8, 26, 52, -11,  3,  0 },
  {  0, 1,  -5, 17, 58, -10,  3,  0 }, //3/4
  {  0, 1,  -4, 13, 60,  -8,  2,  0 },
  {  0, 1,  -3,  8, 62,  -5,  1,  0 },
  {  0, 1,  -2,  4, 63,  -3,  1,  0 }
#endif
};
#if IF_12TAP
// from higher cut-off freq to lower cut-off freq.
const TFilterCoeff InterpolationFilter::m_lumaFilter12[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS + 1][12] =
{
    { 0,     0,     0,     0,     0,   256,     0,     0,     0,     0,     0,     0, },
    {-1,     2,    -3,     6,   -14,   254,    16,    -7,     4,    -2,     1,     0, },
    {-1,     3,    -7,    12,   -26,   249,    35,   -15,     8,    -4,     2,     0, },
    {-2,     5,    -9,    17,   -36,   241,    54,   -22,    12,    -6,     3,    -1, },
    {-2,     5,   -11,    21,   -43,   230,    75,   -29,    15,    -8,     4,    -1, },
    {-2,     6,   -13,    24,   -48,   216,    97,   -36,    19,   -10,     4,    -1, },
    {-2,     7,   -14,    25,   -51,   200,   119,   -42,    22,   -12,     5,    -1, },
    {-2,     7,   -14,    26,   -51,   181,   140,   -46,    24,   -13,     6,    -2, },
    {-2,     6,   -13,    25,   -50,   162,   162,   -50,    25,   -13,     6,    -2, },
    {-2,     6,   -13,    24,   -46,   140,   181,   -51,    26,   -14,     7,    -2, },
    {-1,     5,   -12,    22,   -42,   119,   200,   -51,    25,   -14,     7,    -2, },
    {-1,     4,   -10,    19,   -36,    97,   216,   -48,    24,   -13,     6,    -2, },
    {-1,     4,    -8,    15,   -29,    75,   230,   -43,    21,   -11,     5,    -2, },
    {-1,     3,    -6,    12,   -22,    54,   241,   -36,    17,    -9,     5,    -2, },
    { 0,     2,    -4,     8,   -15,    35,   249,   -26,    12,    -7,     3,    -1, },
    { 0,     1,    -2,     4,    -7,    16,   254,   -14,     6,    -3,     2,    -1, },
#if SIMD_4x4_12
    {  0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0  }  // dummy line for SIMD reading of 2 chunks for 12-tap filter not to address wrong memory //kolya
#endif
};
// This is used for affine
const TFilterCoeff InterpolationFilter::m_lumaFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][8] =
{
  {      0,     0,       0, 64 * 4,      0,       0,      0,      0 },
  {      0, 1 * 4,  -3 * 4, 63 * 4,  4 * 4,  -2 * 4,  1 * 4,      0 },
  { -1 * 4, 2 * 4,  -5 * 4, 62 * 4,  8 * 4,  -3 * 4,  1 * 4,      0 },
  { -1 * 4, 3 * 4,  -8 * 4, 60 * 4, 13 * 4,  -4 * 4,  1 * 4,      0 },
  { -1 * 4, 4 * 4, -10 * 4, 58 * 4, 17 * 4,  -5 * 4,  1 * 4,      0 },
  { -1 * 4, 4 * 4, -11 * 4, 52 * 4, 26 * 4,  -8 * 4,  3 * 4, -1 * 4 },
  { -1 * 4, 3 * 4,  -9 * 4, 47 * 4, 31 * 4, -10 * 4,  4 * 4, -1 * 4 },
  { -1 * 4, 4 * 4, -11 * 4, 45 * 4, 34 * 4, -10 * 4,  4 * 4, -1 * 4 },
  { -1 * 4, 4 * 4, -11 * 4, 40 * 4, 40 * 4, -11 * 4,  4 * 4, -1 * 4 },
  { -1 * 4, 4 * 4, -10 * 4, 34 * 4, 45 * 4, -11 * 4,  4 * 4, -1 * 4 },
  { -1 * 4, 4 * 4, -10 * 4, 31 * 4, 47 * 4,  -9 * 4,  3 * 4, -1 * 4 },
  { -1 * 4, 3 * 4,  -8 * 4, 26 * 4, 52 * 4, -11 * 4,  4 * 4, -1 * 4 },
  {      0, 1 * 4,  -5 * 4, 17 * 4, 58 * 4, -10 * 4,  4 * 4, -1 * 4 },
  {      0, 1 * 4,  -4 * 4, 13 * 4, 60 * 4,  -8 * 4,  3 * 4, -1 * 4 },
  {      0, 1 * 4,  -3 * 4,  8 * 4, 62 * 4,  -5 * 4,  2 * 4, -1 * 4 },
  {      0, 1 * 4,  -2 * 4,  4 * 4, 63 * 4,  -3 * 4,  1 * 4,      0 }
};
#else
const TFilterCoeff InterpolationFilter::m_lumaFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA] =
{
  {  0, 0,   0, 64,  0,   0,  0,  0 },
  {  0, 1,  -3, 63,  4,  -2,  1,  0 },
  { -1, 2,  -5, 62,  8,  -3,  1,  0 },
  { -1, 3,  -8, 60, 13,  -4,  1,  0 },
  { -1, 4, -10, 58, 17,  -5,  1,  0 },
  { -1, 4, -11, 52, 26,  -8,  3, -1 },
  { -1, 3,  -9, 47, 31, -10,  4, -1 },
  { -1, 4, -11, 45, 34, -10,  4, -1 },
  { -1, 4, -11, 40, 40, -11,  4, -1 },
  { -1, 4, -10, 34, 45, -11,  4, -1 },
  { -1, 4, -10, 31, 47,  -9,  3, -1 },
  { -1, 3,  -8, 26, 52, -11,  4, -1 },
  {  0, 1,  -5, 17, 58, -10,  4, -1 },
  {  0, 1,  -4, 13, 60,  -8,  3, -1 },
  {  0, 1,  -3,  8, 62,  -5,  2, -1 },
  {  0, 1,  -2,  4, 63,  -3,  1,  0 }
};
#endif
// 1.5x
#if IF_12TAP
const TFilterCoeff InterpolationFilter::m_lumaFilterRPR1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][8] =
#else
const TFilterCoeff InterpolationFilter::m_lumaFilterRPR1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA] =
#endif
{
#if IF_12TAP
  { -1 * 4, -5 * 4, 17 * 4, 42 * 4, 17 * 4, -5 * 4, -1 * 4,      0 },
  {      0, -5 * 4, 15 * 4, 41 * 4, 19 * 4, -5 * 4, -1 * 4,      0 },
  {      0, -5 * 4, 13 * 4, 40 * 4, 21 * 4, -4 * 4, -1 * 4,      0 },
  {      0, -5 * 4, 11 * 4, 39 * 4, 24 * 4, -4 * 4, -2 * 4,  1 * 4 },
  {      0, -5 * 4,  9 * 4, 38 * 4, 26 * 4, -3 * 4, -2 * 4,  1 * 4 },
  {      0, -5 * 4,  7 * 4, 38 * 4, 28 * 4, -2 * 4, -3 * 4,  1 * 4 },
  {  1 * 4, -5 * 4,  5 * 4, 36 * 4, 30 * 4, -1 * 4, -3 * 4,  1 * 4 },
  {  1 * 4, -4 * 4,  3 * 4, 35 * 4, 32 * 4,      0, -4 * 4,  1 * 4 },
  {  1 * 4, -4 * 4,  2 * 4, 33 * 4, 33 * 4,  2 * 4, -4 * 4,  1 * 4 },
  {  1 * 4, -4 * 4,      0, 32 * 4, 35 * 4,  3 * 4, -4 * 4,  1 * 4 },
  {  1 * 4, -3 * 4, -1 * 4, 30 * 4, 36 * 4,  5 * 4, -5 * 4,  1 * 4 },
  {  1 * 4, -3 * 4, -2 * 4, 28 * 4, 38 * 4,  7 * 4, -5 * 4,      0 },
  {  1 * 4, -2 * 4, -3 * 4, 26 * 4, 38 * 4,  9 * 4, -5 * 4,      0 },
  {  1 * 4, -2 * 4, -4 * 4, 24 * 4, 39 * 4, 11 * 4, -5 * 4,      0 },
  {      0, -1 * 4, -4 * 4, 21 * 4, 40 * 4, 13 * 4, -5 * 4,      0 },
  {      0, -1 * 4, -5 * 4, 19 * 4, 41 * 4, 15 * 4, -5 * 4,      0 }
#else
  { -1, -5, 17, 42, 17, -5, -1,  0 },
  {  0, -5, 15, 41, 19, -5, -1,  0 },
  {  0, -5, 13, 40, 21, -4, -1,  0 },
  {  0, -5, 11, 39, 24, -4, -2,  1 },
  {  0, -5,  9, 38, 26, -3, -2,  1 },
  {  0, -5,  7, 38, 28, -2, -3,  1 },
  {  1, -5,  5, 36, 30, -1, -3,  1 },
  {  1, -4,  3, 35, 32,  0, -4,  1 },
  {  1, -4,  2, 33, 33,  2, -4,  1 },
  {  1, -4,  0, 32, 35,  3, -4,  1 },
  {  1, -3, -1, 30, 36,  5, -5,  1 },
  {  1, -3, -2, 28, 38,  7, -5,  0 },
  {  1, -2, -3, 26, 38,  9, -5,  0 },
  {  1, -2, -4, 24, 39, 11, -5,  0 },
  {  0, -1, -4, 21, 40, 13, -5,  0 },
  {  0, -1, -5, 19, 41, 15, -5,  0 }
#endif
};

// 2x
#if IF_12TAP
const TFilterCoeff InterpolationFilter::m_lumaFilterRPR2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][8] =
#else
const TFilterCoeff InterpolationFilter::m_lumaFilterRPR2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA] =
#endif
{
#if IF_12TAP
  { -4 * 4,  2 * 4, 20 * 4, 28 * 4, 20 * 4,  2 * 4, -4 * 4,      0 },
  { -4 * 4,      0, 19 * 4, 29 * 4, 21 * 4,  5 * 4, -4 * 4, -2 * 4 },
  { -4 * 4, -1 * 4, 18 * 4, 29 * 4, 22 * 4,  6 * 4, -4 * 4, -2 * 4 },
  { -4 * 4, -1 * 4, 16 * 4, 29 * 4, 23 * 4,  7 * 4, -4 * 4, -2 * 4 },
  { -4 * 4, -1 * 4, 16 * 4, 28 * 4, 24 * 4,  7 * 4, -4 * 4, -2 * 4 },
  { -4 * 4, -1 * 4, 14 * 4, 28 * 4, 25 * 4,  8 * 4, -4 * 4, -2 * 4 },
  { -3 * 4, -3 * 4, 14 * 4, 27 * 4, 26 * 4,  9 * 4, -3 * 4, -3 * 4 },
  { -3 * 4, -1 * 4, 12 * 4, 28 * 4, 25 * 4, 10 * 4, -4 * 4, -3 * 4 },
  { -3 * 4, -3 * 4, 11 * 4, 27 * 4, 27 * 4, 11 * 4, -3 * 4, -3 * 4 },
  { -3 * 4, -4 * 4, 10 * 4, 25 * 4, 28 * 4, 12 * 4, -1 * 4, -3 * 4 },
  { -3 * 4, -3 * 4,  9 * 4, 26 * 4, 27 * 4, 14 * 4, -3 * 4, -3 * 4 },
  { -2 * 4, -4 * 4,  8 * 4, 25 * 4, 28 * 4, 14 * 4, -1 * 4, -4 * 4 },
  { -2 * 4, -4 * 4,  7 * 4, 24 * 4, 28 * 4, 16 * 4, -1 * 4, -4 * 4 },
  { -2 * 4, -4 * 4,  7 * 4, 23 * 4, 29 * 4, 16 * 4, -1 * 4, -4 * 4 },
  { -2 * 4, -4 * 4,  6 * 4, 22 * 4, 29 * 4, 18 * 4, -1 * 4, -4 * 4 },
  { -2 * 4, -4 * 4,  5 * 4, 21 * 4, 29 * 4, 19 * 4,      0, -4 * 4 }
#else
  { -4,  2, 20, 28, 20,  2, -4,  0 },
  { -4,  0, 19, 29, 21,  5, -4, -2 },
  { -4, -1, 18, 29, 22,  6, -4, -2 },
  { -4, -1, 16, 29, 23,  7, -4, -2 },
  { -4, -1, 16, 28, 24,  7, -4, -2 },
  { -4, -1, 14, 28, 25,  8, -4, -2 },
  { -3, -3, 14, 27, 26,  9, -3, -3 },
  { -3, -1, 12, 28, 25, 10, -4, -3 },
  { -3, -3, 11, 27, 27, 11, -3, -3 },
  { -3, -4, 10, 25, 28, 12, -1, -3 },
  { -3, -3,  9, 26, 27, 14, -3, -3 },
  { -2, -4,  8, 25, 28, 14, -1, -4 },
  { -2, -4,  7, 24, 28, 16, -1, -4 },
  { -2, -4,  7, 23, 29, 16, -1, -4 },
  { -2, -4,  6, 22, 29, 18, -1, -4 },
  { -2, -4,  5, 21, 29, 19,  0, -4 }
#endif
};

// 1.5x
#if IF_12TAP
const TFilterCoeff InterpolationFilter::m_affineLumaFilterRPR1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][8] =
#else
const TFilterCoeff InterpolationFilter::m_affineLumaFilterRPR1[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA] =
#endif
{
#if IF_12TAP
  {  0, -6 * 4, 17 * 4, 42 * 4, 17 * 4, -5 * 4, -1 * 4,  0 },
  {  0, -5 * 4, 15 * 4, 41 * 4, 19 * 4, -5 * 4, -1 * 4,  0 },
  {  0, -5 * 4, 13 * 4, 40 * 4, 21 * 4, -4 * 4, -1 * 4,  0 },
  {  0, -5 * 4, 11 * 4, 39 * 4, 24 * 4, -4 * 4, -1 * 4,  0 },
  {  0, -5 * 4,  9 * 4, 38 * 4, 26 * 4, -3 * 4, -1 * 4,  0 },
  {  0, -5 * 4,  7 * 4, 38 * 4, 28 * 4, -2 * 4, -2 * 4,  0 },
  {  0, -4 * 4,  5 * 4, 36 * 4, 30 * 4, -1 * 4, -2 * 4,  0 },
  {  0, -3 * 4,  3 * 4, 35 * 4, 32 * 4,      0, -3 * 4,  0 },
  {  0, -3 * 4,  2 * 4, 33 * 4, 33 * 4,  2 * 4, -3 * 4,  0 },
  {  0, -3 * 4,      0, 32 * 4, 35 * 4,  3 * 4, -3 * 4,  0 },
  {  0, -2 * 4, -1 * 4, 30 * 4, 36 * 4,  5 * 4, -4 * 4,  0 },
  {  0, -2 * 4, -2 * 4, 28 * 4, 38 * 4,  7 * 4, -5 * 4,  0 },
  {  0, -1 * 4, -3 * 4, 26 * 4, 38 * 4,  9 * 4, -5 * 4,  0 },
  {  0, -1 * 4, -4 * 4, 24 * 4, 39 * 4, 11 * 4, -5 * 4,  0 },
  {  0, -1 * 4, -4 * 4, 21 * 4, 40 * 4, 13 * 4, -5 * 4,  0 },
  {  0, -1 * 4, -5 * 4, 19 * 4, 41 * 4, 15 * 4, -5 * 4,  0 }
#else
  {  0, -6, 17, 42, 17, -5, -1,  0 },
  {  0, -5, 15, 41, 19, -5, -1,  0 },
  {  0, -5, 13, 40, 21, -4, -1,  0 },
  {  0, -5, 11, 39, 24, -4, -1,  0 },
  {  0, -5,  9, 38, 26, -3, -1,  0 },
  {  0, -5,  7, 38, 28, -2, -2,  0 },
  {  0, -4,  5, 36, 30, -1, -2,  0 },
  {  0, -3,  3, 35, 32,  0, -3,  0 },
  {  0, -3,  2, 33, 33,  2, -3,  0 },
  {  0, -3,  0, 32, 35,  3, -3,  0 },
  {  0, -2, -1, 30, 36,  5, -4,  0 },
  {  0, -2, -2, 28, 38,  7, -5,  0 },
  {  0, -1, -3, 26, 38,  9, -5,  0 },
  {  0, -1, -4, 24, 39, 11, -5,  0 },
  {  0, -1, -4, 21, 40, 13, -5,  0 },
  {  0, -1, -5, 19, 41, 15, -5,  0 }
#endif
};

// 2x
#if IF_12TAP
const TFilterCoeff InterpolationFilter::m_affineLumaFilterRPR2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][8] =
#else
const TFilterCoeff InterpolationFilter::m_affineLumaFilterRPR2[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_LUMA] =
#endif
{
#if IF_12TAP
  {  0, -2 * 4, 20 * 4, 28 * 4, 20 * 4,  2 * 4, -4 * 4,  0 },
  {  0, -4 * 4, 19 * 4, 29 * 4, 21 * 4,  5 * 4, -6 * 4,  0 },
  {  0, -5 * 4, 18 * 4, 29 * 4, 22 * 4,  6 * 4, -6 * 4,  0 },
  {  0, -5 * 4, 16 * 4, 29 * 4, 23 * 4,  7 * 4, -6 * 4,  0 },
  {  0, -5 * 4, 16 * 4, 28 * 4, 24 * 4,  7 * 4, -6 * 4,  0 },
  {  0, -5 * 4, 14 * 4, 28 * 4, 25 * 4,  8 * 4, -6 * 4,  0 },
  {  0, -6 * 4, 14 * 4, 27 * 4, 26 * 4,  9 * 4, -6 * 4,  0 },
  {  0, -4 * 4, 12 * 4, 28 * 4, 25 * 4, 10 * 4, -7 * 4,  0 },
  {  0, -6 * 4, 11 * 4, 27 * 4, 27 * 4, 11 * 4, -6 * 4,  0 },
  {  0, -7 * 4, 10 * 4, 25 * 4, 28 * 4, 12 * 4, -4 * 4,  0 },
  {  0, -6 * 4,  9 * 4, 26 * 4, 27 * 4, 14 * 4, -6 * 4,  0 },
  {  0, -6 * 4,  8 * 4, 25 * 4, 28 * 4, 14 * 4, -5 * 4,  0 },
  {  0, -6 * 4,  7 * 4, 24 * 4, 28 * 4, 16 * 4, -5 * 4,  0 },
  {  0, -6 * 4,  7 * 4, 23 * 4, 29 * 4, 16 * 4, -5 * 4,  0 },
  {  0, -6 * 4,  6 * 4, 22 * 4, 29 * 4, 18 * 4, -5 * 4,  0 },
  {  0, -6 * 4,  5 * 4, 21 * 4, 29 * 4, 19 * 4, -4 * 4,  0 }
#else
  {  0, -2, 20, 28, 20,  2, -4,  0 },
  {  0, -4, 19, 29, 21,  5, -6,  0 },
  {  0, -5, 18, 29, 22,  6, -6,  0 },
  {  0, -5, 16, 29, 23,  7, -6,  0 },
  {  0, -5, 16, 28, 24,  7, -6,  0 },
  {  0, -5, 14, 28, 25,  8, -6,  0 },
  {  0, -6, 14, 27, 26,  9, -6,  0 },
  {  0, -4, 12, 28, 25, 10, -7,  0 },
  {  0, -6, 11, 27, 27, 11, -6,  0 },
  {  0, -7, 10, 25, 28, 12, -4,  0 },
  {  0, -6,  9, 26, 27, 14, -6,  0 },
  {  0, -6,  8, 25, 28, 14, -5,  0 },
  {  0, -6,  7, 24, 28, 16, -5,  0 },
  {  0, -6,  7, 23, 29, 16, -5,  0 },
  {  0, -6,  6, 22, 29, 18, -5,  0 },
  {  0, -6,  5, 21, 29, 19, -4,  0 }
#endif
};

#if IF_12TAP
const TFilterCoeff InterpolationFilter::m_lumaAltHpelIFilter[8] = { 0 * 4, 3 * 4, 9 * 4, 20 * 4, 20 * 4, 9 * 4, 3 * 4, 0 * 4 };
#else
const TFilterCoeff InterpolationFilter::m_lumaAltHpelIFilter[NTAPS_LUMA] = { 0, 3, 9, 20, 20, 9, 3, 0 };
#endif

#if IF_12TAP
#if JVET_Z0117_CHROMA_IF
const TFilterCoeff InterpolationFilter::m_chromaFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA] =
{
    {0, 0, 256, 0, 0, 0},
    {1, -6, 256, 7, -2, 0},
    {2, -11, 253, 15, -4, 1},
    {3, -16, 251, 23, -6, 1},
    {4, -21, 248, 33, -10, 2},
    {5, -25, 244, 42, -12, 2},
    {7, -30, 239, 53, -17, 4},
    {7, -32, 234, 62, -19, 4},
    {8, -35, 227, 73, -22, 5},
    {9, -38, 220, 84, -26, 7},
    {10, -40, 213, 95, -29, 7},
    {10, -41, 204, 106, -31, 8},
    {10, -42, 196, 117, -34, 9},
    {10, -41, 187, 127, -35, 8},
    {11, -42, 177, 138, -38, 10},
    {10, -41, 168, 148, -39, 10},
    {10, -40, 158, 158, -40, 10},
    {10, -39, 148, 168, -41, 10},
    {10, -38, 138, 177, -42, 11},
    {8, -35, 127, 187, -41, 10},
    {9, -34, 117, 196, -42, 10},
    {8, -31, 106, 204, -41, 10},
    {7, -29, 95, 213, -40, 10},
    {7, -26, 84, 220, -38, 9},
    {5, -22, 73, 227, -35, 8},
    {4, -19, 62, 234, -32, 7},
    {4, -17, 53, 239, -30, 7},
    {2, -12, 42, 244, -25, 5},
    {2, -10, 33, 248, -21, 4},
    {1, -6, 23, 251, -16, 3},
    {1, -4, 15, 253, -11, 2},
    {0, -2, 7, 256, -6, 1},
};

const TFilterCoeff InterpolationFilter::m_chromaFilter4[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][4] =
#else
const TFilterCoeff InterpolationFilter::m_chromaFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA] =
#endif
{
{  0 * 4, 64 * 4,  0 * 4,  0 * 4 },
{ -1 * 4, 63 * 4,  2 * 4,  0 * 4 },
{ -2 * 4, 62 * 4,  4 * 4,  0 * 4 },
{ -2 * 4, 60 * 4,  7 * 4, -1 * 4 },
{ -2 * 4, 58 * 4, 10 * 4, -2 * 4 },
{ -3 * 4, 57 * 4, 12 * 4, -2 * 4 },
{ -4 * 4, 56 * 4, 14 * 4, -2 * 4 },
{ -4 * 4, 55 * 4, 15 * 4, -2 * 4 },
{ -4 * 4, 54 * 4, 16 * 4, -2 * 4 },
{ -5 * 4, 53 * 4, 18 * 4, -2 * 4 },
{ -6 * 4, 52 * 4, 20 * 4, -2 * 4 },
{ -6 * 4, 49 * 4, 24 * 4, -3 * 4 },
{ -6 * 4, 46 * 4, 28 * 4, -4 * 4 },
{ -5 * 4, 44 * 4, 29 * 4, -4 * 4 },
{ -4 * 4, 42 * 4, 30 * 4, -4 * 4 },
{ -4 * 4, 39 * 4, 33 * 4, -4 * 4 },
{ -4 * 4, 36 * 4, 36 * 4, -4 * 4 },
{ -4 * 4, 33 * 4, 39 * 4, -4 * 4 },
{ -4 * 4, 30 * 4, 42 * 4, -4 * 4 },
{ -4 * 4, 29 * 4, 44 * 4, -5 * 4 },
{ -4 * 4, 28 * 4, 46 * 4, -6 * 4 },
{ -3 * 4, 24 * 4, 49 * 4, -6 * 4 },
{ -2 * 4, 20 * 4, 52 * 4, -6 * 4 },
{ -2 * 4, 18 * 4, 53 * 4, -5 * 4 },
{ -2 * 4, 16 * 4, 54 * 4, -4 * 4 },
{ -2 * 4, 15 * 4, 55 * 4, -4 * 4 },
{ -2 * 4, 14 * 4, 56 * 4, -4 * 4 },
{ -2 * 4, 12 * 4, 57 * 4, -3 * 4 },
{ -2 * 4, 10 * 4, 58 * 4, -2 * 4 },
{ -1 * 4,  7 * 4, 60 * 4, -2 * 4 },
{  0 * 4,  4 * 4, 62 * 4, -2 * 4 },
{  0 * 4,  2 * 4, 63 * 4, -1 * 4 },
};
#else
const TFilterCoeff InterpolationFilter::m_chromaFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA] =
{
  {  0, 64,  0,  0 },
  { -1, 63,  2,  0 },
  { -2, 62,  4,  0 },
  { -2, 60,  7, -1 },
  { -2, 58, 10, -2 },
  { -3, 57, 12, -2 },
  { -4, 56, 14, -2 },
  { -4, 55, 15, -2 },
  { -4, 54, 16, -2 },
  { -5, 53, 18, -2 },
  { -6, 52, 20, -2 },
  { -6, 49, 24, -3 },
  { -6, 46, 28, -4 },
  { -5, 44, 29, -4 },
  { -4, 42, 30, -4 },
  { -4, 39, 33, -4 },
  { -4, 36, 36, -4 },
  { -4, 33, 39, -4 },
  { -4, 30, 42, -4 },
  { -4, 29, 44, -5 },
  { -4, 28, 46, -6 },
  { -3, 24, 49, -6 },
  { -2, 20, 52, -6 },
  { -2, 18, 53, -5 },
  { -2, 16, 54, -4 },
  { -2, 15, 55, -4 },
  { -2, 14, 56, -4 },
  { -2, 12, 57, -3 },
  { -2, 10, 58, -2 },
  { -1,  7, 60, -2 },
  {  0,  4, 62, -2 },
  {  0,  2, 63, -1 },
};
#endif

#if INTRA_6TAP
#if JVET_Z0117_CHROMA_IF
const TFilterCoeff InterpolationFilter::m_weak4TapFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][4] =
#else
const TFilterCoeff InterpolationFilter::m_weak4TapFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA] =
#endif
{
  {  0, 64,  0,  0},
  { -1, 64,  1,  0},
  { -3, 65,  3, -1},
  { -3, 63,  5, -1},
  { -4, 63,  6, -1},
  { -5, 62,  9, -2},
  { -5, 60, 11, -2},
  { -5, 58, 13, -2},
  { -6, 57, 16, -3},
  { -6, 55, 18, -3},
  { -7, 54, 21, -4},
  { -7, 52, 23, -4},
  { -6, 48, 26, -4},
  { -7, 47, 29, -5},
  { -6, 43, 32, -5},
  { -6, 41, 34, -5},
  { -5, 37, 37, -5},
  { -5, 34, 41, -6},
  { -5, 32, 43, -6},
  { -5, 29, 47, -7},
  { -4, 26, 48, -6},
  { -4, 23, 52, -7},
  { -4, 21, 54, -7},
  { -3, 18, 55, -6},
  { -3, 16, 57, -6},
  { -2, 13, 58, -5},
  { -2, 11, 60, -5},
  { -2,  9, 62, -5},
  { -1,  6, 63, -4},
  { -1,  5, 63, -3},
  { -1,  3, 65, -3},
  {  0,  1, 64, -1},
};


const TFilterCoeff InterpolationFilter::m_lumaIntraFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][6] =
{
  {   0,   0, 256,   0,   0,   0 },  //  0/32 position
  {   0,  -4, 253,   9,  -2,   0 },  //  1/32 position
  {   1,  -7, 249,  17,  -4,   0 },  //  2/32 position
  {   1, -10, 245,  25,  -6,   1 },  //  3/32 position
  {   1, -13, 241,  34,  -8,   1 },  //  4/32 position
  {   2, -16, 235,  44, -10,   1 },  //  5/32 position
  {   2, -18, 229,  53, -12,   2 },  //  6/32 position
  {   2, -20, 223,  63, -14,   2 },  //  7/32 position
  {   2, -22, 217,  72, -15,   2 },  //  8/32 position
  {   3, -23, 209,  82, -17,   2 },  //  9/32 position
  {   3, -24, 202,  92, -19,   2 },  // 10/32 position
  {   3, -25, 194, 101, -20,   3 },  // 11/32 position
  {   3, -25, 185, 111, -21,   3 },  // 12/32 position
  {   3, -26, 178, 121, -23,   3 },  // 13/32 position
  {   3, -25, 168, 131, -24,   3 },  // 14/32 position
  {   3, -25, 159, 141, -25,   3 },  // 15/32 position
  {   3, -25, 150, 150, -25,   3 },  // half-pel position
  {   3, -25, 141, 159, -25,   3 },  //  17/32 position
  {   3, -24, 131, 168, -25,   3 },  //  18/32 position
  {   3, -23, 121, 178, -26,   3 },  //  19/32 position
  {   3, -21, 111, 185, -25,   3 },  //  20/32 position
  {   3, -20, 101, 194, -25,   3 },  //  21/32 position
  {   2, -19,  92, 202, -24,   3 },  //  22/32 position
  {   2, -17,  82, 209, -23,   3 },  //  23/32 position
  {   2, -15,  72, 217, -22,   2 },  //  24/32 position
  {   2, -14,  63, 223, -20,   2 },  //  25/32 position
  {   2, -12,  53, 229, -18,   2 },  //  26/32 position
  {   1, -10,  44, 235, -16,   2 },  // 27/32 position
  {   1,  -8,  34, 241, -13,   1 },  // 28/32 position
  {   1,  -6,  25, 245, -10,   1 },  // 29/32 position
  {   0,  -4,  17, 249,  -7,   1 },  // 30/32 position
  {   0, - 2,   9, 253,  -4,   0 },  // 31/32 position
};

#if JVET_W0123_TIMD_FUSION
const TFilterCoeff InterpolationFilter::m_lumaIntraFilterExt[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS<<1][6] =
{
  {   0,   0, 256,   0,   0,   0 },
  {   0,  -2, 255,   4,  -1,   0 },
  {   0,  -4, 253,   9,  -2,   0 },
  {   0,  -5, 251,  13,  -3,   0 },
  {   1,  -7, 249,  17,  -4,   0 },
  {   1,  -9, 247,  21,  -5,   1 },
  {   1, -10, 245,  25,  -6,   1 },
  {   1, -12, 243,  30,  -7,   1 },
  {   1, -13, 241,  34,  -8,   1 },
  {   2, -15, 238,  39,  -9,   1 },
  {   2, -16, 235,  44, -10,   1 },
  {   2, -17, 232,  49, -11,   1 },
  {   2, -18, 229,  53, -12,   2 },
  {   2, -19, 226,  58, -13,   2 },
  {   2, -20, 223,  63, -14,   2 },
  {   2, -21, 220,  68, -15,   2 },
  {   2, -22, 217,  72, -15,   2 },
  {   2, -23, 213,  78, -16,   2 },
  {   3, -23, 209,  82, -17,   2 },
  {   3, -24, 205,  88, -18,   2 },
  {   3, -24, 202,  92, -19,   2 },
  {   3, -24, 198,  97, -20,   2 },
  {   3, -25, 194, 101, -20,   3 },
  {   3, -25, 189, 106, -20,   3 },
  {   3, -25, 185, 111, -21,   3 },
  {   3, -25, 181, 116, -22,   3 },
  {   3, -26, 178, 121, -23,   3 },
  {   3, -26, 173, 126, -23,   3 },
  {   3, -25, 168, 131, -24,   3 },
  {   3, -25, 163, 137, -25,   3 },
  {   3, -25, 159, 141, -25,   3 },
  {   3, -25, 155, 145, -25,   3 },
  {   3, -25, 150, 150, -25,   3 },
  {   3, -25, 145, 155, -25,   3 },
  {   3, -25, 141, 159, -25,   3 },
  {   3, -25, 137, 163, -25,   3 },
  {   3, -24, 131, 168, -25,   3 },
  {   3, -24, 126, 173, -25,   3 },
  {   3, -23, 121, 178, -26,   3 },
  {   3, -22, 116, 181, -25,   3 },
  {   3, -21, 111, 185, -25,   3 },
  {   3, -21, 106, 180, -25,   3 },
  {   3, -20, 101, 194, -25,   3 },
  {   2, -20,  97, 198, -24,   3 },
  {   2, -19,  92, 202, -24,   3 },
  {   2, -18,  86, 206, -23,   3 },
  {   2, -17,  82, 209, -23,   3 },
  {   2, -16,  77, 213, -23,   3 },
  {   2, -15,  72, 217, -22,   2 },
  {   2, -15,  68, 220, -21,   2 },
  {   2, -14,  63, 223, -20,   2 },
  {   2, -13,  58, 226, -19,   2 },
  {   2, -12,  53, 229, -18,   2 },
  {   2, -11,  48, 232, -17,   2 },
  {   1, -10,  44, 235, -16,   2 },
  {   1,  -9,  39, 238, -15,   2 },
  {   1,  -8,  34, 241, -13,   1 },
  {   1,  -7,  29, 243, -11,   1 },
  {   1,  -6,  25, 245, -10,   1 },
  {   0,  -5,  21, 247,  -8,   1 },
  {   0,  -4,  17, 249,  -7,   1 },
  {   0,  -3,  13, 251,  -5,   0 },
  {   0,  -2,   9, 253,  -4,   0 },
  {   0,  -1,   5, 255,  -3,   0 },
};
#endif
#endif

#if JVET_W0123_TIMD_FUSION
#if JVET_Z0117_CHROMA_IF
const TFilterCoeff InterpolationFilter::g_aiExtIntraCubicFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << 1][4] = {
#else
const TFilterCoeff InterpolationFilter::g_aiExtIntraCubicFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS<<1][NTAPS_CHROMA] = {
#endif
  {   0, 256,   0,   0 },
  {  -1, 254,   4,  -1 },
  {  -3, 252,   8,  -1 },
  {  -4, 250,  12,  -2 },
  {  -5, 247,  17,  -3 },
  {  -6, 244,  21,  -3 },
  {  -7, 242,  25,  -4 },
  {  -8, 239,  29,  -4 },
  {  -9, 236,  34,  -5 },
  {  -9, 233,  38,  -6 },
  { -10, 230,  43,  -7 },
  { -11, 227,  47,  -7 },
  { -12, 224,  52,  -8 },
  { -12, 220,  56,  -8 },
  { -13, 217,  61,  -9 },
  { -14, 214,  65,  -9 },
  { -14, 210,  70, -10 },
  { -14, 206,  75, -11 },
  { -15, 203,  79, -11 },
  { -15, 199,  84, -12 },
  { -16, 195,  89, -12 },
  { -16, 191,  93, -12 },
  { -16, 187,  98, -13 },
  { -16, 183, 102, -13 },
  { -16, 179, 107, -14 },
  { -16, 174, 112, -14 },
  { -16, 170, 116, -14 },
  { -16, 166, 121, -15 },
  { -17, 162, 126, -15 },
  { -16, 157, 130, -15 },
  { -16, 153, 135, -16 },
  { -16, 148, 140, -16 },
  { -16, 144, 144, -16 },
  { -16, 140, 148, -16},
  { -16, 135, 153, -16},
  { -15, 130, 157, -16},
  { -15, 126, 162, -17},
  { -15, 121, 166, -16},
  { -14, 116, 170, -16},
  { -14, 112, 174, -16},
  { -14, 107, 179, -16},
  { -13, 102, 183, -16},
  { -13,  98, 187, -16},
  { -12,  93, 191, -16},
  { -12,  89, 195, -16},
  { -12,  84, 199, -15},
  { -11,  79, 203, -15},
  { -11,  75, 206, -14},
  { -10,  70, 210, -14},
  {  -9,  65, 214, -14},
  {  -9,  61, 217, -13},
  {  -8,  56, 220, -12},
  {  -8,  52, 224, -12},
  {  -7,  47, 227, -11},
  {  -7,  43, 230, -10},
  {  -6,  38, 233,  -9},
  {  -5,  34, 236,  -9},
  {  -4,  29, 239,  -8},
  {  -4,  25, 242,  -7},
  {  -3,  21, 244,  -6},
  {  -3,  17, 247,  -5},
  {  -2,  12, 250,  -4},
  {  -1,   8, 252,  -3},
  {  -1,   4, 254,  -1},
};
#if JVET_Z0117_CHROMA_IF
const TFilterCoeff InterpolationFilter::g_aiExtIntraGaussFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS << 1][4] = {
#else
const TFilterCoeff InterpolationFilter::g_aiExtIntraGaussFilter[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS<<1][NTAPS_CHROMA] = {
#endif
  {  47, 161,  47,   1 },
  {  45, 161,  49,   1 },
  {  43, 161,  51,   1 },
  {  42, 160,  52,   2 },
  {  40, 160,  54,   2 },
  {  38, 160,  56,   2 },
  {  37, 159,  58,   2 },
  {  35, 158,  61,   2 },
  {  34, 158,  62,   2 },
  {  32, 157,  65,   2 },
  {  31, 156,  67,   2 },
  {  29, 155,  69,   3 },
  {  28, 154,  71,   3 },
  {  27, 153,  73,   3 },
  {  26, 151,  76,   3 },
  {  25, 150,  78,   3 },
  {  23, 149,  80,   4 },
  {  22, 147,  83,   4 },
  {  21, 146,  85,   4 },
  {  20, 144,  87,   5 },
  {  19, 142,  90,   5 },
  {  18, 141,  92,   5 },
  {  17, 139,  94,   6 },
  {  16, 137,  97,   6 },
  {  16, 135,  99,   6 },
  {  15, 133, 101,   7 },
  {  14, 131, 104,   7 },
  {  13, 129, 106,   8 },
  {  13, 127, 108,   8 },
  {  12, 125, 111,   8 },
  {  11, 123, 113,   9 },
  {  11, 120, 116,   9 },
  {  10, 118, 118,  10 },
  {   9, 116, 120,  11},
  {   9, 113, 123,  11},
  {   8, 111, 125,  12},
  {   8, 108, 127,  13},
  {   8, 106, 129,  13},
  {   7, 104, 131,  14},
  {   7, 101, 133,  15},
  {   6,  99, 135,  16},
  {   6,  97, 137,  16},
  {   6,  94, 139,  17},
  {   5,  92, 141,  18},
  {   5,  90, 142,  19},
  {   5,  87, 144,  20},
  {   4,  85, 146,  21},
  {   4,  83, 147,  22},
  {   4,  80, 149,  23},
  {   3,  78, 150,  25},
  {   3,  76, 151,  26},
  {   3,  73, 153,  27},
  {   3,  71, 154,  28},
  {   3,  69, 155,  29},
  {   2,  67, 156,  31},
  {   2,  65, 157,  32},
  {   2,  62, 158,  34},
  {   2,  61, 158,  35},
  {   2,  58, 159,  37},
  {   2,  56, 160,  38},
  {   2,  54, 160,  40},
  {   2,  52, 160,  42},
  {   1,  51, 161,  43},
  {   1,  49, 161,  45},
};
#endif

//1.5x
#if JVET_Z0117_CHROMA_IF
const TFilterCoeff InterpolationFilter::m_chromaFilterRPR1[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA_RPR] =
#else
const TFilterCoeff InterpolationFilter::m_chromaFilterRPR1[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA] =
#endif
{
#if IF_12TAP
  { 12 * 4, 40 * 4, 12 * 4,      0 },
  { 11 * 4, 40 * 4, 13 * 4,      0 },
  { 10 * 4, 40 * 4, 15 * 4, -1 * 4 },
  {  9 * 4, 40 * 4, 16 * 4, -1 * 4 },
  {  8 * 4, 40 * 4, 17 * 4, -1 * 4 },
  {  8 * 4, 39 * 4, 18 * 4, -1 * 4 },
  {  7 * 4, 39 * 4, 19 * 4, -1 * 4 },
  {  6 * 4, 38 * 4, 21 * 4, -1 * 4 },
  {  5 * 4, 38 * 4, 22 * 4, -1 * 4 },
  {  4 * 4, 38 * 4, 23 * 4, -1 * 4 },
  {  4 * 4, 37 * 4, 24 * 4, -1 * 4 },
  {  3 * 4, 36 * 4, 25 * 4,      0 },
  {  3 * 4, 35 * 4, 26 * 4,      0 },
  {  2 * 4, 34 * 4, 28 * 4,      0 },
  {  2 * 4, 33 * 4, 29 * 4,      0 },
  {  1 * 4, 33 * 4, 30 * 4,      0 },
  {  1 * 4, 31 * 4, 31 * 4,  1 * 4 },
  {      0, 30 * 4, 33 * 4,  1 * 4 },
  {      0, 29 * 4, 33 * 4,  2 * 4 },
  {      0, 28 * 4, 34 * 4,  2 * 4 },
  {      0, 26 * 4, 35 * 4,  3 * 4 },
  {      0, 25 * 4, 36 * 4,  3 * 4 },
  { -1 * 4, 24 * 4, 37 * 4,  4 * 4 },
  { -1 * 4, 23 * 4, 38 * 4,  4 * 4 },
  { -1 * 4, 22 * 4, 38 * 4,  5 * 4 },
  { -1 * 4, 21 * 4, 38 * 4,  6 * 4 },
  { -1 * 4, 19 * 4, 39 * 4,  7 * 4 },
  { -1 * 4, 18 * 4, 39 * 4,  8 * 4 },
  { -1 * 4, 17 * 4, 40 * 4,  8 * 4 },
  { -1 * 4, 16 * 4, 40 * 4,  9 * 4 },
  { -1 * 4, 15 * 4, 40 * 4, 10 * 4 },
  {      0, 13 * 4, 40 * 4, 11 * 4 },
#else
  { 12, 40, 12,  0 },
  { 11, 40, 13,  0 },
  { 10, 40, 15, -1 },
  {  9, 40, 16, -1 },
  {  8, 40, 17, -1 },
  {  8, 39, 18, -1 },
  {  7, 39, 19, -1 },
  {  6, 38, 21, -1 },
  {  5, 38, 22, -1 },
  {  4, 38, 23, -1 },
  {  4, 37, 24, -1 },
  {  3, 36, 25,  0 },
  {  3, 35, 26,  0 },
  {  2, 34, 28,  0 },
  {  2, 33, 29,  0 },
  {  1, 33, 30,  0 },
  {  1, 31, 31,  1 },
  {  0, 30, 33,  1 },
  {  0, 29, 33,  2 },
  {  0, 28, 34,  2 },
  {  0, 26, 35,  3 },
  {  0, 25, 36,  3 },
  { -1, 24, 37,  4 },
  { -1, 23, 38,  4 },
  { -1, 22, 38,  5 },
  { -1, 21, 38,  6 },
  { -1, 19, 39,  7 },
  { -1, 18, 39,  8 },
  { -1, 17, 40,  8 },
  { -1, 16, 40,  9 },
  { -1, 15, 40, 10 },
  {  0, 13, 40, 11 },
#endif
};

//2x
#if JVET_Z0117_CHROMA_IF
const TFilterCoeff InterpolationFilter::m_chromaFilterRPR2[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA_RPR] =
#else
const TFilterCoeff InterpolationFilter::m_chromaFilterRPR2[CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_CHROMA] =
#endif
{
#if IF_12TAP
  { 17 * 4, 30 * 4, 17 * 4,      0 },
  { 17 * 4, 30 * 4, 18 * 4, -1 * 4 },
  { 16 * 4, 30 * 4, 18 * 4,      0 },
  { 16 * 4, 30 * 4, 18 * 4,      0 },
  { 15 * 4, 30 * 4, 18 * 4,  1 * 4 },
  { 14 * 4, 30 * 4, 18 * 4,  2 * 4 },
  { 13 * 4, 29 * 4, 19 * 4,  3 * 4 },
  { 13 * 4, 29 * 4, 19 * 4,  3 * 4 },
  { 12 * 4, 29 * 4, 20 * 4,  3 * 4 },
  { 11 * 4, 28 * 4, 21 * 4,  4 * 4 },
  { 10 * 4, 28 * 4, 22 * 4,  4 * 4 },
  { 10 * 4, 27 * 4, 22 * 4,  5 * 4 },
  {  9 * 4, 27 * 4, 23 * 4,  5 * 4 },
  {  9 * 4, 26 * 4, 24 * 4,  5 * 4 },
  {  8 * 4, 26 * 4, 24 * 4,  6 * 4 },
  {  7 * 4, 26 * 4, 25 * 4,  6 * 4 },
  {  7 * 4, 25 * 4, 25 * 4,  7 * 4 },
  {  6 * 4, 25 * 4, 26 * 4,  7 * 4 },
  {  6 * 4, 24 * 4, 26 * 4,  8 * 4 },
  {  5 * 4, 24 * 4, 26 * 4,  9 * 4 },
  {  5 * 4, 23 * 4, 27 * 4,  9 * 4 },
  {  5 * 4, 22 * 4, 27 * 4, 10 * 4 },
  {  4 * 4, 22 * 4, 28 * 4, 10 * 4 },
  {  4 * 4, 21 * 4, 28 * 4, 11 * 4 },
  {  3 * 4, 20 * 4, 29 * 4, 12 * 4 },
  {  3 * 4, 19 * 4, 29 * 4, 13 * 4 },
  {  3 * 4, 19 * 4, 29 * 4, 13 * 4 },
  {  2 * 4, 18 * 4, 30 * 4, 14 * 4 },
  {  1 * 4, 18 * 4, 30 * 4, 15 * 4 },
  {      0, 18 * 4, 30 * 4, 16 * 4 },
  {      0, 18 * 4, 30 * 4, 16 * 4 },
  { -1 * 4, 18 * 4, 30 * 4, 17 * 4 }
#else
  { 17, 30, 17,  0 },
  { 17, 30, 18, -1 },
  { 16, 30, 18,  0 },
  { 16, 30, 18,  0 },
  { 15, 30, 18,  1 },
  { 14, 30, 18,  2 },
  { 13, 29, 19,  3 },
  { 13, 29, 19,  3 },
  { 12, 29, 20,  3 },
  { 11, 28, 21,  4 },
  { 10, 28, 22,  4 },
  { 10, 27, 22,  5 },
  {  9, 27, 23,  5 },
  {  9, 26, 24,  5 },
  {  8, 26, 24,  6 },
  {  7, 26, 25,  6 },
  {  7, 25, 25,  7 },
  {  6, 25, 26,  7 },
  {  6, 24, 26,  8 },
  {  5, 24, 26,  9 },
  {  5, 23, 27,  9 },
  {  5, 22, 27, 10 },
  {  4, 22, 28, 10 },
  {  4, 21, 28, 11 },
  {  3, 20, 29, 12 },
  {  3, 19, 29, 13 },
  {  3, 19, 29, 13 },
  {  2, 18, 30, 14 },
  {  1, 18, 30, 15 },
  {  0, 18, 30, 16 },
  {  0, 18, 30, 16 },
  { -1, 18, 30, 17 }
#endif
};

const TFilterCoeff InterpolationFilter::m_bilinearFilter[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_BILINEAR] =
{
#if IF_12TAP
  { 64 * 4,  0 * 4, },
  { 60 * 4,  4 * 4, },
  { 56 * 4,  8 * 4, },
  { 52 * 4, 12 * 4, },
  { 48 * 4, 16 * 4, },
  { 44 * 4, 20 * 4, },
  { 40 * 4, 24 * 4, },
  { 36 * 4, 28 * 4, },
  { 32 * 4, 32 * 4, },
  { 28 * 4, 36 * 4, },
  { 24 * 4, 40 * 4, },
  { 20 * 4, 44 * 4, },
  { 16 * 4, 48 * 4, },
  { 12 * 4, 52 * 4, },
  {  8 * 4, 56 * 4, },
  {  4 * 4, 60 * 4, },
#else
  { 64,  0, },
  { 60,  4, },
  { 56,  8, },
  { 52, 12, },
  { 48, 16, },
  { 44, 20, },
  { 40, 24, },
  { 36, 28, },
  { 32, 32, },
  { 28, 36, },
  { 24, 40, },
  { 20, 44, },
  { 16, 48, },
  { 12, 52, },
  { 8, 56, },
  { 4, 60, },
#endif
};

const TFilterCoeff InterpolationFilter::m_bilinearFilterPrec4[LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS][NTAPS_BILINEAR] =
{
  { 16,  0, },
  { 15,  1, },
  { 14,  2, },
  { 13, 3, },
  { 12, 4, },
  { 11, 5, },
  { 10, 6, },
  { 9, 7, },
  { 8, 8, },
  { 7, 9, },
  { 6, 10, },
  { 5, 11, },
  { 4, 12, },
  { 3, 13, },
  { 2, 14, },
  { 1, 15, }
};
// ====================================================================================================================
// Private member functions
// ====================================================================================================================

InterpolationFilter::InterpolationFilter()
{
#if IF_12TAP
#if !QC_SIF_SIMD
  m_filterHor[0][0][0] = filter<12, false, false, false>;
  m_filterHor[0][0][1] = filter<12, false, false, true>;
  m_filterHor[0][1][0] = filter<12, false, true, false>;
  m_filterHor[0][1][1] = filter<12, false, true, true>;

  m_filterHor[1][0][0] = filter<8, false, false, false>;
  m_filterHor[1][0][1] = filter<8, false, false, true>;
  m_filterHor[1][1][0] = filter<8, false, true, false>;
  m_filterHor[1][1][1] = filter<8, false, true, true>;

#if JVET_Z0117_CHROMA_IF
  m_filterHor[2][0][0] = filter<6, false, false, false>;
  m_filterHor[2][0][1] = filter<6, false, false, true>;
  m_filterHor[2][1][0] = filter<6, false, true, false>;
  m_filterHor[2][1][1] = filter<6, false, true, true>;
  
  m_filterHor[3][0][0] = filter<4, false, false, false>;
  m_filterHor[3][0][1] = filter<4, false, false, true>;
  m_filterHor[3][1][0] = filter<4, false, true, false>;
  m_filterHor[3][1][1] = filter<4, false, true, true>;

  m_filterHor[4][0][0] = filter<2, false, false, false>;
  m_filterHor[4][0][1] = filter<2, false, false, true>;
  m_filterHor[4][1][0] = filter<2, false, true, false>;
  m_filterHor[4][1][1] = filter<2, false, true, true>;
#else
  m_filterHor[2][0][0] = filter<4, false, false, false>;
  m_filterHor[2][0][1] = filter<4, false, false, true>;
  m_filterHor[2][1][0] = filter<4, false, true, false>;
  m_filterHor[2][1][1] = filter<4, false, true, true>;

  m_filterHor[3][0][0] = filter<2, false, false, false>;
  m_filterHor[3][0][1] = filter<2, false, false, true>;
  m_filterHor[3][1][0] = filter<2, false, true, false>;
  m_filterHor[3][1][1] = filter<2, false, true, true>;
#endif

  m_filterVer[0][0][0] = filter<12, true, false, false>;
  m_filterVer[0][0][1] = filter<12, true, false, true>;
  m_filterVer[0][1][0] = filter<12, true, true, false>;
  m_filterVer[0][1][1] = filter<12, true, true, true>;

  m_filterVer[1][0][0] = filter<8, true, false, false>;
  m_filterVer[1][0][1] = filter<8, true, false, true>;
  m_filterVer[1][1][0] = filter<8, true, true, false>;
  m_filterVer[1][1][1] = filter<8, true, true, true>;

#if JVET_Z0117_CHROMA_IF
  m_filterVer[2][0][0] = filter<6, true, false, false>;
  m_filterVer[2][0][1] = filter<6, true, false, true>;
  m_filterVer[2][1][0] = filter<6, true, true, false>;
  m_filterVer[2][1][1] = filter<6, true, true, true>;
  
  m_filterVer[3][0][0] = filter<4, true, false, false>;
  m_filterVer[3][0][1] = filter<4, true, false, true>;
  m_filterVer[3][1][0] = filter<4, true, true, false>;
  m_filterVer[3][1][1] = filter<4, true, true, true>;

  m_filterVer[4][0][0] = filter<2, true, false, false>;
  m_filterVer[4][0][1] = filter<2, true, false, true>;
  m_filterVer[4][1][0] = filter<2, true, true, false>;
  m_filterVer[4][1][1] = filter<2, true, true, true>;
#else
  m_filterVer[2][0][0] = filter<4, true, false, false>;
  m_filterVer[2][0][1] = filter<4, true, false, true>;
  m_filterVer[2][1][0] = filter<4, true, true, false>;
  m_filterVer[2][1][1] = filter<4, true, true, true>;

  m_filterVer[3][0][0] = filter<2, true, false, false>;
  m_filterVer[3][0][1] = filter<2, true, false, true>;
  m_filterVer[3][1][0] = filter<2, true, true, false>;
  m_filterVer[3][1][1] = filter<2, true, true, true>;
#endif
#endif
#else
#if !QC_SIF_SIMD
  m_filterHor[0][0][0] = filter<8, false, false, false>;
  m_filterHor[0][0][1] = filter<8, false, false, true>;
  m_filterHor[0][1][0] = filter<8, false, true, false>;
  m_filterHor[0][1][1] = filter<8, false, true, true>;

  m_filterHor[1][0][0] = filter<4, false, false, false>;
  m_filterHor[1][0][1] = filter<4, false, false, true>;
  m_filterHor[1][1][0] = filter<4, false, true, false>;
  m_filterHor[1][1][1] = filter<4, false, true, true>;

  m_filterHor[2][0][0] = filter<2, false, false, false>;
  m_filterHor[2][0][1] = filter<2, false, false, true>;
  m_filterHor[2][1][0] = filter<2, false, true, false>;
  m_filterHor[2][1][1] = filter<2, false, true, true>;

  m_filterVer[0][0][0] = filter<8, true, false, false>;
  m_filterVer[0][0][1] = filter<8, true, false, true>;
  m_filterVer[0][1][0] = filter<8, true, true, false>;
  m_filterVer[0][1][1] = filter<8, true, true, true>;

  m_filterVer[1][0][0] = filter<4, true, false, false>;
  m_filterVer[1][0][1] = filter<4, true, false, true>;
  m_filterVer[1][1][0] = filter<4, true, true, false>;
  m_filterVer[1][1][1] = filter<4, true, true, true>;

  m_filterVer[2][0][0] = filter<2, true, false, false>;
  m_filterVer[2][0][1] = filter<2, true, false, true>;
  m_filterVer[2][1][0] = filter<2, true, true, false>;
  m_filterVer[2][1][1] = filter<2, true, true, true>;
#endif
#endif
  m_filterCopy[0][0]   = filterCopy<false, false>;
  m_filterCopy[0][1]   = filterCopy<false, true>;
  m_filterCopy[1][0]   = filterCopy<true, false>;
  m_filterCopy[1][1]   = filterCopy<true, true>;
#if !QC_SIF_SIMD
  m_weightedGeoBlk = xWeightedGeoBlk;
#if JVET_Y0065_GPM_INTRA
  m_weightedGeoBlkRounded = xWeightedGeoBlkRounded;
#endif
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  m_weightedGeoTplA = xWeightedGeoTpl<true>;
  m_weightedGeoTplL = xWeightedGeoTpl<false>;
#endif
}


/**
 * \brief Apply unit FIR filter to a block of samples
 *
 * \param bitDepth   bitDepth of samples
 * \param src        Pointer to source samples
 * \param srcStride  Stride of source samples
 * \param dst        Pointer to destination samples
 * \param dstStride  Stride of destination samples
 * \param width      Width of block
 * \param height     Height of block
 * \param isFirst    Flag indicating whether it is the first filtering operation
 * \param isLast     Flag indicating whether it is the last filtering operation
 */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// !!! NOTE !!!
//
//  This is the scalar version of the function.
//  If you change the functionality here, consider to switch off the SIMD implementation of this function.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<bool isFirst, bool isLast>
void InterpolationFilter::filterCopy( const ClpRng& clpRng, const Pel *src, int srcStride, Pel *dst, int dstStride, int width, int height, bool biMCForDMVR)
{
  int row, col;

  if ( isFirst == isLast )
  {
    int lineSize = sizeof(Pel) * width;

    for (row = 0; row < height; row++)
    {
      ::memcpy(dst, src, lineSize);

      src += srcStride;
      dst += dstStride;
    }
  }
  else if ( isFirst )
  {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    const int shift = IF_INTERNAL_FRAC_BITS(clpRng.bd);
#else
    const int shift = std::max<int>(2, (IF_INTERNAL_PREC - clpRng.bd));
#endif

    if (biMCForDMVR)
    {
      int shift10BitOut, offset;
      if ((clpRng.bd - IF_INTERNAL_PREC_BILINEAR) > 0)
      {
        shift10BitOut = (clpRng.bd - IF_INTERNAL_PREC_BILINEAR);
        offset = (1 << (shift10BitOut - 1));
        for (row = 0; row < height; row++)
        {
          for (col = 0; col < width; col++)
          {
            dst[col] = (src[col] + offset) >> shift10BitOut;
          }
          src += srcStride;
          dst += dstStride;
        }
      }
      else
      {
        shift10BitOut = (IF_INTERNAL_PREC_BILINEAR - clpRng.bd);
        for (row = 0; row < height; row++)
        {
          for (col = 0; col < width; col++)
          {
            dst[col] = src[col] << shift10BitOut;
          }
          src += srcStride;
          dst += dstStride;
        }
      }
    }
    else
    for (row = 0; row < height; row++)
    {
      for (col = 0; col < width; col++)
      {
        Pel val = leftShift_round(src[col], shift);
        dst[col] = val - (Pel)IF_INTERNAL_OFFS;
        JVET_J0090_CACHE_ACCESS( &src[col], __FILE__, __LINE__ );
      }

      src += srcStride;
      dst += dstStride;
    }
  }
  else
  {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    const int shift = IF_INTERNAL_FRAC_BITS(clpRng.bd);
#else
    const int shift = std::max<int>(2, (IF_INTERNAL_PREC - clpRng.bd));
#endif

    if (biMCForDMVR)
    {
      int shift10BitOut, offset;
      if ((clpRng.bd - IF_INTERNAL_PREC_BILINEAR) > 0)
      {
        shift10BitOut = (clpRng.bd - IF_INTERNAL_PREC_BILINEAR);
        offset = (1 << (shift10BitOut - 1));
        for (row = 0; row < height; row++)
        {
          for (col = 0; col < width; col++)
          {
            dst[col] = (src[col] + offset) >> shift10BitOut;
          }
          src += srcStride;
          dst += dstStride;
        }
      }
      else
      {
        shift10BitOut = (IF_INTERNAL_PREC_BILINEAR - clpRng.bd);
        for (row = 0; row < height; row++)
        {
          for (col = 0; col < width; col++)
          {
            dst[col] = src[col] << shift10BitOut;
          }
          src += srcStride;
          dst += dstStride;
        }
      }
    }
    else
    {
      for (row = 0; row < height; row++)
      {
        for (col = 0; col < width; col++)
        {
          Pel val = src[col];
          val     = rightShift_round((val + IF_INTERNAL_OFFS), shift);

          dst[col] = ClipPel(val, clpRng);
          JVET_J0090_CACHE_ACCESS(&src[col], __FILE__, __LINE__);
        }

        src += srcStride;
        dst += dstStride;
      }
    }
  }
}

#if SIMD_4x4_12 && defined(TARGET_SIMD_X86)
void InterpolationFilter::filter4x4( const ClpRng& clpRng, Pel const *src, int srcStride, Pel *dst, int dstStride, int xFrac, int yFrac, bool isLast)
{
  const TFilterCoeff* coeffH =  m_lumaFilter12[xFrac];
  const TFilterCoeff* coeffV =  m_lumaFilter12[yFrac];

  src = src - 5 - 5*srcStride; //for 12-tap filter

  const int headRoom   = std::max<int>( 2, ( IF_INTERNAL_PREC - clpRng.bd ) );
  int shiftH   = IF_FILTER_PREC - headRoom;
  int offsetH  = -IF_INTERNAL_OFFS << shiftH;
  int shiftV = IF_FILTER_PREC;
  int offsetV = 0;

  int ibdimin = INT16_MIN; //for 16-bit
  int ibdimax = INT16_MAX;

  if( isLast ) //todo: check for template
  {
    shiftV += headRoom;
    offsetV  = (1 << ( shiftV - 1 )) + (IF_INTERNAL_OFFS << IF_FILTER_PREC);
    ibdimin = clpRng.min;
    ibdimax = clpRng.max;
  }
  if( !isLast )
    m_filter4x4[0]( src, srcStride, dst, dstStride, shiftH, offsetH, shiftV, offsetV, coeffH, coeffV, ibdimin, ibdimax );
  else
    m_filter4x4[1]( src, srcStride, dst, dstStride, shiftH, offsetH, shiftV, offsetV, coeffH, coeffV, ibdimin, ibdimax );
}
#endif

/**
 * \brief Apply FIR filter to a block of samples
 *
 * \tparam N          Number of taps
 * \tparam isVertical Flag indicating filtering along vertical direction
 * \tparam isFirst    Flag indicating whether it is the first filtering operation
 * \tparam isLast     Flag indicating whether it is the last filtering operation
 * \param  bitDepth   Bit depth of samples
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  coeff      Pointer to filter taps
 */
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// !!! NOTE !!!
//
//  This is the scalar version of the function.
//  If you change the functionality here, consider to switch off the SIMD implementation of this function.
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<int N, bool isVertical, bool isFirst, bool isLast>
void InterpolationFilter::filter(const ClpRng& clpRng, Pel const *src, int srcStride, Pel *dst, int dstStride, int width, int height, TFilterCoeff const *coeff, bool biMCForDMVR)
{
  int row, col;
#if IF_12TAP
  Pel c[N];
  for( int i = 0; i < N; i++ )
  {
    c[i] = coeff[i];
  }
#else
  Pel c[8];
  c[0] = coeff[0];
  c[1] = coeff[1];
  if ( N >= 4 )
  {
    c[2] = coeff[2];
    c[3] = coeff[3];
  }
  if ( N >= 6 )
  {
    c[4] = coeff[4];
    c[5] = coeff[5];
  }
  if ( N == 8 )
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
  int headRoom = std::max<int>(2, (IF_INTERNAL_PREC - clpRng.bd));
#endif
  int shift    = IF_FILTER_PREC;
  // with the current settings (IF_INTERNAL_PREC = 14 and IF_FILTER_PREC = 6), though headroom can be
  // negative for bit depths greater than 14, shift will remain non-negative for bit depths of 8->20
  CHECK(shift < 0, "Negative shift");

  if ( isLast )
  {
    shift += (isFirst) ? 0 : headRoom;
    offset = 1 << (shift - 1);
    offset += (isFirst) ? 0 : IF_INTERNAL_OFFS << IF_FILTER_PREC;
  }
  else
  {
    shift -= (isFirst) ? headRoom : 0;
    offset = (isFirst) ? -IF_INTERNAL_OFFS << shift : 0;
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
  for (row = 0; row < height; row++)
  {
    for (col = 0; col < width; col++)
    {
      int sum;

      sum  = src[ col + 0 * cStride] * c[0];
      sum += src[ col + 1 * cStride] * c[1];
      JVET_J0090_CACHE_ACCESS( &src[ col + 0 * cStride], __FILE__, __LINE__ );
      JVET_J0090_CACHE_ACCESS( &src[ col + 1 * cStride], __FILE__, __LINE__ );
      if ( N >= 4 )
      {
        sum += src[ col + 2 * cStride] * c[2];
        sum += src[ col + 3 * cStride] * c[3];
        JVET_J0090_CACHE_ACCESS( &src[ col + 2 * cStride], __FILE__, __LINE__ );
        JVET_J0090_CACHE_ACCESS( &src[ col + 3 * cStride], __FILE__, __LINE__ );
      }
      if ( N >= 6 )
      {
        sum += src[ col + 4 * cStride] * c[4];
        sum += src[ col + 5 * cStride] * c[5];
        JVET_J0090_CACHE_ACCESS( &src[ col + 4 * cStride], __FILE__, __LINE__ );
        JVET_J0090_CACHE_ACCESS( &src[ col + 5 * cStride], __FILE__, __LINE__ );
      }
#if IF_12TAP
      if (N >= 8)
#else
      if (N == 8)
#endif
      {
        sum += src[ col + 6 * cStride] * c[6];
        sum += src[ col + 7 * cStride] * c[7];
        JVET_J0090_CACHE_ACCESS( &src[ col + 6 * cStride], __FILE__, __LINE__ );
        JVET_J0090_CACHE_ACCESS( &src[ col + 7 * cStride], __FILE__, __LINE__ );
      }
#if IF_12TAP
      if (N >= 10)
      {
        sum += src[col + 8 * cStride] * c[8];
        sum += src[col + 9 * cStride] * c[9];
        JVET_J0090_CACHE_ACCESS(&src[col + 8 * cStride], __FILE__, __LINE__);
        JVET_J0090_CACHE_ACCESS(&src[col + 9 * cStride], __FILE__, __LINE__);
      }
      if (N >= 12)
      {
        sum += src[col + 10 * cStride] * c[10];
        sum += src[col + 11 * cStride] * c[11];
        JVET_J0090_CACHE_ACCESS(&src[col + 10 * cStride], __FILE__, __LINE__);
        JVET_J0090_CACHE_ACCESS(&src[col + 11 * cStride], __FILE__, __LINE__);
      }

      if (N == 16)
      {
        sum += src[col + 12 * cStride] * c[12];
        sum += src[col + 13 * cStride] * c[13];
        sum += src[col + 14 * cStride] * c[14];
        sum += src[col + 15 * cStride] * c[15];
        JVET_J0090_CACHE_ACCESS(&src[col + 12 * cStride], __FILE__, __LINE__);
        JVET_J0090_CACHE_ACCESS(&src[col + 13 * cStride], __FILE__, __LINE__);
        JVET_J0090_CACHE_ACCESS(&src[col + 14 * cStride], __FILE__, __LINE__);
        JVET_J0090_CACHE_ACCESS(&src[col + 15 * cStride], __FILE__, __LINE__);
      }
#endif
      Pel val = ( sum + offset ) >> shift;
      if ( isLast )
      {
        val = ClipPel( val, clpRng );
      }
      dst[col] = val;
    }

    src += srcStride;
    dst += dstStride;
  }
}

/**
 * \brief Filter a block of samples (horizontal)
 *
 * \tparam N          Number of taps
 * \param  bitDepth   Bit depth of samples
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  coeff      Pointer to filter taps
 */
template<int N>
void InterpolationFilter::filterHor(const ClpRng& clpRng, Pel const *src, int srcStride, Pel *dst, int dstStride, int width, int height, bool isLast, TFilterCoeff const *coeff, bool biMCForDMVR)
{
//#if ENABLE_SIMD_OPT_MCIF
#if IF_12TAP
  if (N == 12)
  {
    m_filterHor[0][1][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
  else if (N == 8)
  {
    m_filterHor[1][1][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
#if JVET_Z0117_CHROMA_IF
  else if( N == 6 )
  {
    m_filterHor[2][1][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
  else if (N == 4)
  {
    m_filterHor[3][1][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
  else if (N == 2)
  {
    m_filterHor[4][1][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
#else
  else if (N == 4)
  {
    m_filterHor[2][1][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
  else if (N == 2)
  {
    m_filterHor[3][1][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
#endif
#else
  if( N == 8 )
  {
    m_filterHor[0][1][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
#if JVET_Z0117_CHROMA_IF
  else if( N == 6 )
  {
    m_filterHor[1][1][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
  else if (N == 4)
  {
    m_filterHor[2][1][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
  else if (N == 2)
  {
    m_filterHor[3][1][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
#else
  else if( N == 4 )
  {
    m_filterHor[1][1][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
  else if( N == 2 )
  {
    m_filterHor[2][1][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
#endif
#endif
  else
  {
    THROW( "Invalid tap number" );
  }
}

/**
 * \brief Filter a block of samples (vertical)
 *
 * \tparam N          Number of taps
 * \param  bitDepth   Bit depth
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  isFirst    Flag indicating whether it is the first filtering operation
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  coeff      Pointer to filter taps
 */
template<int N>
void InterpolationFilter::filterVer(const ClpRng& clpRng, Pel const *src, int srcStride, Pel *dst, int dstStride, int width, int height, bool isFirst, bool isLast, TFilterCoeff const *coeff, bool biMCForDMVR)
{
//#if ENABLE_SIMD_OPT_MCIF
#if IF_12TAP
  if (N == 12)
  {
    m_filterVer[0][isFirst][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
  else if (N == 8)
  {
    m_filterVer[1][isFirst][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
#if JVET_Z0117_CHROMA_IF
  else if (N == 6)
  {
    m_filterVer[2][isFirst][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
  else if (N == 4)
  {
    m_filterVer[3][isFirst][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
  else if (N == 2)
  {
    m_filterVer[4][isFirst][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
#else
  else if (N == 4)
  {
    m_filterVer[2][isFirst][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
  else if (N == 2)
  {
    m_filterVer[3][isFirst][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
#endif
#else
  if( N == 8 )
  {
    m_filterVer[0][isFirst][isLast]( clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
#if JVET_Z0117_CHROMA_IF
  else if (N == 6)
  {
    m_filterVer[1][isFirst][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
  else if (N == 4)
  {
    m_filterVer[2][isFirst][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
  else if (N == 2)
  {
    m_filterVer[3][isFirst][isLast](clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
#else
  else if( N == 4 )
  {
    m_filterVer[1][isFirst][isLast]( clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
  else if( N == 2 )
  {
    m_filterVer[2][isFirst][isLast]( clpRng, src, srcStride, dst, dstStride, width, height, coeff, biMCForDMVR);
  }
#endif
#endif
  else
  {
    THROW( "Invalid tap number" );
  }
}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

/**
 * \brief Filter a block of Luma/Chroma samples (horizontal)
 *
 * \param  compID     Chroma component ID
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  fmt        Chroma format
 * \param  bitDepth   Bit depth
 */
void InterpolationFilter::filterHor(const ComponentID compID, Pel const *src, int srcStride, Pel *dst, int dstStride, int width, int height, int frac, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, int nFilterIdx, bool biMCForDMVR, bool useAltHpelIf)
{
  if( frac == 0 && nFilterIdx < 2 )
  {
    m_filterCopy[true][isLast]( clpRng, src, srcStride, dst, dstStride, width, height, biMCForDMVR );
  }
  else if( isLuma( compID ) )
  {
#if IF_12TAP
    CHECK(frac < 0 || frac >= LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS, "Invalid fraction");
    if (nFilterIdx == 1)
    {
#if TM_AMVP || TM_MRG || JVET_W0090_ARMC_TM || JVET_Z0056_GPM_SPLIT_MODE_REORDERING
      filterHor<NTAPS_BILINEAR>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, (biMCForDMVR ? m_bilinearFilterPrec4 : m_bilinearFilter)[frac], biMCForDMVR );
#else
      filterHor<NTAPS_BILINEAR>(clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_bilinearFilterPrec4[frac], biMCForDMVR);
#endif
    }
    else if (nFilterIdx == 2)
    {
      filterHor<8>(clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilter4x4[frac], biMCForDMVR);
    }
    else if (nFilterIdx == 3)
    {
      filterHor<8>(clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilterRPR1[frac], biMCForDMVR);
    }
    else if (nFilterIdx == 4)
    {
      filterHor<8>(clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilterRPR2[frac], biMCForDMVR);
    }

    else if (nFilterIdx == 5)
    {
      filterHor<8>(clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_affineLumaFilterRPR1[frac], biMCForDMVR);
    }
    else if (nFilterIdx == 6)
    {
      filterHor<8>(clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_affineLumaFilterRPR2[frac], biMCForDMVR);
    }

    else if (frac == 8 && useAltHpelIf)
    {
      filterHor<8>(clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaAltHpelIFilter, biMCForDMVR);
    }
#if !AFFINE_RM_CONSTRAINTS_AND_OPT
    else if ((width == 4 && height == 4) || (width == 4 && height == (4 + NTAPS_LUMA(0) - 1)))
    {
      filterHor<NTAPS_LUMA(0)>(clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilter12[frac], biMCForDMVR);
    }
#endif
    else
    {
      filterHor<NTAPS_LUMA(0)>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilter12[frac], biMCForDMVR );
    }
#else
    CHECK( frac < 0 || frac >= LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS, "Invalid fraction" );
    if( nFilterIdx == 1 )
    {
#if TM_AMVP || TM_MRG || JVET_W0090_ARMC_TM || JVET_Z0056_GPM_SPLIT_MODE_REORDERING
      filterHor<NTAPS_BILINEAR>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, (biMCForDMVR ? m_bilinearFilterPrec4 : m_bilinearFilter)[frac], biMCForDMVR );
#else
      filterHor<NTAPS_BILINEAR>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_bilinearFilterPrec4[frac], biMCForDMVR );
#endif
    }
    else if( nFilterIdx == 2 )
    {
      filterHor<NTAPS_LUMA>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilter4x4[frac], biMCForDMVR );
    }
    else if( nFilterIdx == 3 )
    {
      filterHor<NTAPS_LUMA>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilterRPR1[frac], biMCForDMVR );
    }
    else if( nFilterIdx == 4 )
    {
      filterHor<NTAPS_LUMA>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilterRPR2[frac], biMCForDMVR );
    }
    else if (nFilterIdx == 5)
    {
      filterHor<NTAPS_LUMA>(clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_affineLumaFilterRPR1[frac], biMCForDMVR);
    }
    else if (nFilterIdx == 6)
    {
      filterHor<NTAPS_LUMA>(clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_affineLumaFilterRPR2[frac], biMCForDMVR);
    }
    else if( frac == 8 && useAltHpelIf )
    {
      filterHor<NTAPS_LUMA>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaAltHpelIFilter, biMCForDMVR );
    }
#if !AFFINE_RM_CONSTRAINTS_AND_OPT
    else if( ( width == 4 && height == 4 ) || ( width == 4 && height == ( 4 + NTAPS_LUMA - 1 ) ) )
    {
      filterHor<NTAPS_LUMA>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilter4x4[frac], biMCForDMVR );
    }
#endif
    else
    {
      filterHor<NTAPS_LUMA>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_lumaFilter[frac], biMCForDMVR );
    }
#endif
  }
  else
  {
    const uint32_t csx = getComponentScaleX( compID, fmt );
    CHECK( frac < 0 || csx >= 2 || ( frac << ( 1 - csx ) ) >= CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS, "Invalid fraction" );
    if( nFilterIdx == 3 )
    {
#if JVET_Z0117_CHROMA_IF
      filterHor<NTAPS_CHROMA_RPR>(clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_chromaFilterRPR1[frac << (1 - csx)], biMCForDMVR);
#else
      filterHor<NTAPS_CHROMA>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_chromaFilterRPR1[frac << ( 1 - csx )], biMCForDMVR );
#endif
    }
    else if( nFilterIdx == 4 )
    {
#if JVET_Z0117_CHROMA_IF
      filterHor<NTAPS_CHROMA_RPR>(clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_chromaFilterRPR2[frac << (1 - csx)], biMCForDMVR);
#else
      filterHor<NTAPS_CHROMA>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_chromaFilterRPR2[frac << ( 1 - csx )], biMCForDMVR );
#endif
    }
    else
    {
      filterHor<NTAPS_CHROMA>( clpRng, src, srcStride, dst, dstStride, width, height, isLast, m_chromaFilter[frac << ( 1 - csx )], biMCForDMVR );
    }
  }
}


/**
 * \brief Filter a block of Luma/Chroma samples (vertical)
 *
 * \param  compID     Colour component ID
 * \param  src        Pointer to source samples
 * \param  srcStride  Stride of source samples
 * \param  dst        Pointer to destination samples
 * \param  dstStride  Stride of destination samples
 * \param  width      Width of block
 * \param  height     Height of block
 * \param  frac       Fractional sample offset
 * \param  isFirst    Flag indicating whether it is the first filtering operation
 * \param  isLast     Flag indicating whether it is the last filtering operation
 * \param  fmt        Chroma format
 * \param  bitDepth   Bit depth
 */
void InterpolationFilter::filterVer(const ComponentID compID, Pel const *src, int srcStride, Pel *dst, int dstStride, int width, int height, int frac, bool isFirst, bool isLast, const ChromaFormat fmt, const ClpRng& clpRng, int nFilterIdx, bool biMCForDMVR, bool useAltHpelIf)
{
  if( frac == 0 && nFilterIdx < 2 )
  {
    m_filterCopy[isFirst][isLast]( clpRng, src, srcStride, dst, dstStride, width, height, biMCForDMVR );
  }
  else if( isLuma( compID ) )
  {
#if IF_12TAP
    CHECK(frac < 0 || frac >= LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS, "Invalid fraction");
    if (nFilterIdx == 1)
    {
#if TM_AMVP || TM_MRG || JVET_W0090_ARMC_TM || JVET_Z0056_GPM_SPLIT_MODE_REORDERING
      filterVer<NTAPS_BILINEAR>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, (biMCForDMVR ? m_bilinearFilterPrec4 : m_bilinearFilter)[frac], biMCForDMVR );
#else
      filterVer<NTAPS_BILINEAR>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_bilinearFilterPrec4[frac], biMCForDMVR);
#endif
    }
    else if (nFilterIdx == 2)
    {
      filterVer<8>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaFilter4x4[frac], biMCForDMVR);
    }
    else if (nFilterIdx == 3)
    {
      filterVer<8>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaFilterRPR1[frac], biMCForDMVR);
    }
    else if (nFilterIdx == 4)
    {
      filterVer<8>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaFilterRPR2[frac], biMCForDMVR);
    }
    else if (nFilterIdx == 5)
    {
      filterVer<8>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_affineLumaFilterRPR1[frac], biMCForDMVR);
    }
    else if (nFilterIdx == 6)
    {
      filterVer<8>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_affineLumaFilterRPR2[frac], biMCForDMVR);
    }
    else if (frac == 8 && useAltHpelIf)
    {
      filterVer<8>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaAltHpelIFilter, biMCForDMVR);
    }
#if !AFFINE_RM_CONSTRAINTS_AND_OPT
    else if (width == 4 && height == 4)
    {
      filterVer<NTAPS_LUMA(0)>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaFilter12[frac], biMCForDMVR);
    }
#endif
    else
    {
      filterVer<NTAPS_LUMA(0)>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaFilter12[frac], biMCForDMVR );
    }
#else
    CHECK( frac < 0 || frac >= LUMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS, "Invalid fraction" );
    if( nFilterIdx == 1 )
    {
#if TM_AMVP || TM_MRG || JVET_W0090_ARMC_TM || JVET_Z0056_GPM_SPLIT_MODE_REORDERING
      filterVer<NTAPS_BILINEAR>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, (biMCForDMVR ? m_bilinearFilterPrec4 : m_bilinearFilter)[frac], biMCForDMVR );
#else
      filterVer<NTAPS_BILINEAR>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_bilinearFilterPrec4[frac], biMCForDMVR );
#endif
    }
    else if( nFilterIdx == 2 )
    {
      filterVer<NTAPS_LUMA>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaFilter4x4[frac], biMCForDMVR );
    }
    else if( nFilterIdx == 3 )
    {
      filterVer<NTAPS_LUMA>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaFilterRPR1[frac], biMCForDMVR );
    }
    else if( nFilterIdx == 4 )
    {
      filterVer<NTAPS_LUMA>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaFilterRPR2[frac], biMCForDMVR );
    }
    else if (nFilterIdx == 5)
    {
      filterVer<NTAPS_LUMA>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_affineLumaFilterRPR1[frac], biMCForDMVR);
    }
    else if (nFilterIdx == 6)
    {
      filterVer<NTAPS_LUMA>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_affineLumaFilterRPR2[frac], biMCForDMVR);
    }
    else if( frac == 8 && useAltHpelIf )
    {
      filterVer<NTAPS_LUMA>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaAltHpelIFilter, biMCForDMVR );
    }
#if !AFFINE_RM_CONSTRAINTS_AND_OPT
    else if( width == 4 && height == 4 )
    {
      filterVer<NTAPS_LUMA>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaFilter4x4[frac], biMCForDMVR );
    }
#endif
    else
    {
      filterVer<NTAPS_LUMA>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_lumaFilter[frac], biMCForDMVR );
    }
#endif
  }
  else
  {
    const uint32_t csy = getComponentScaleY( compID, fmt );
    CHECK( frac < 0 || csy >= 2 || ( frac << ( 1 - csy ) ) >= CHROMA_INTERPOLATION_FILTER_SUB_SAMPLE_POSITIONS, "Invalid fraction" );
    if( nFilterIdx == 3 )
    {
#if JVET_Z0117_CHROMA_IF
      filterVer<NTAPS_CHROMA_RPR>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_chromaFilterRPR1[frac << (1 - csy)], biMCForDMVR);
#else
      filterVer<NTAPS_CHROMA>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_chromaFilterRPR1[frac << ( 1 - csy )], biMCForDMVR );
#endif
    }
    else if( nFilterIdx == 4 )
    {
#if JVET_Z0117_CHROMA_IF
      filterVer<NTAPS_CHROMA_RPR>(clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_chromaFilterRPR2[frac << (1 - csy)], biMCForDMVR);
#else
      filterVer<NTAPS_CHROMA>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_chromaFilterRPR2[frac << ( 1 - csy )], biMCForDMVR );
#endif
    }
    else
    {
      filterVer<NTAPS_CHROMA>( clpRng, src, srcStride, dst, dstStride, width, height, isFirst, isLast, m_chromaFilter[frac << ( 1 - csy )], biMCForDMVR );
    }
  }
}

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
template <bool trueTFalseL>
void InterpolationFilter::xWeightedGeoTpl(const PredictionUnit &pu, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1)
{
  const ComponentID compIdx = COMPONENT_Y;

  Pel*    dst = predDst.get(compIdx).buf;
  Pel*    src0 = predSrc0.get(compIdx).buf;
  Pel*    src1 = predSrc1.get(compIdx).buf;
  int32_t strideDst  = predDst .get(compIdx).stride;
  int32_t strideSrc0 = predSrc0.get(compIdx).stride;
  int32_t strideSrc1 = predSrc1.get(compIdx).stride;

  const uint32_t scaleX = getComponentScaleX(compIdx, pu.chromaFormat);
  const uint32_t scaleY = getComponentScaleY(compIdx, pu.chromaFormat);

  int16_t angle = g_GeoParams[splitDir][0];
  int16_t wIdx  = floorLog2(pu.lwidth()) - GEO_MIN_CU_LOG2;
  int16_t hIdx  = floorLog2(pu.lheight()) - GEO_MIN_CU_LOG2;
  int16_t stepX = 1 << scaleX;
  int16_t stepY = 0;
  Pel*   weight = &g_globalGeoWeightsTpl[g_angle2mask[angle]][GEO_TM_ADDED_WEIGHT_MASK_SIZE * GEO_WEIGHT_MASK_SIZE_EXT + GEO_TM_ADDED_WEIGHT_MASK_SIZE];
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

  if (trueTFalseL)
  {
    for (int x = 0; x < predDst.bufs[compIdx].width; x++)
    {
      const Pel w = -(*weight);
      dst[x]  = ((w & src0[x]) | ((~w) & src1[x])); // Same as dst[x] = *weight != 0 ? src0[x] : src0[x]
      weight += stepX;
    }
  }
  else
  {
    for (int y = 0; y < predDst.bufs[compIdx].height; y++)
    {
      const Pel w = -(*weight);
      dst[0] = ((w & src0[0]) | ((~w) & src1[0])); // Same as dst[0] = *weight != 0 ? src0[0] : src0[1]

      dst    += strideDst;
      src0   += strideSrc0;
      src1   += strideSrc1;
      weight += stepY;
    }
  }
}
#endif

void InterpolationFilter::weightedGeoBlk(const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1)
{
  m_weightedGeoBlk(pu, width, height, compIdx, splitDir, predDst, predSrc0, predSrc1);
}

void InterpolationFilter::xWeightedGeoBlk(const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1)
{
  Pel*    dst = predDst.get(compIdx).buf;
  Pel*    src0 = predSrc0.get(compIdx).buf;
  Pel*    src1 = predSrc1.get(compIdx).buf;
  int32_t strideDst = predDst.get(compIdx).stride - width;
  int32_t strideSrc0 = predSrc0.get(compIdx).stride - width;
  int32_t strideSrc1 = predSrc1.get(compIdx).stride - width;

  const char    log2WeightBase = 3;
  const ClpRng  clipRng = pu.cu->slice->clpRngs().comp[compIdx];
  const int32_t clipbd = clipRng.bd;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  const int32_t shiftWeighted = IF_INTERNAL_FRAC_BITS(clipbd) + log2WeightBase;
#else
  const int32_t shiftWeighted = std::max<int>(2, (IF_INTERNAL_PREC - clipbd)) + log2WeightBase;
#endif
  const int32_t offsetWeighted = (1 << (shiftWeighted - 1)) + (IF_INTERNAL_OFFS << log2WeightBase);
  const uint32_t scaleX = getComponentScaleX(compIdx, pu.chromaFormat);
  const uint32_t scaleY = getComponentScaleY(compIdx, pu.chromaFormat);

  int16_t angle = g_GeoParams[splitDir][0];
  int16_t wIdx = floorLog2(pu.lwidth()) - GEO_MIN_CU_LOG2;
  int16_t hIdx = floorLog2(pu.lheight()) - GEO_MIN_CU_LOG2;
  int16_t stepX = 1 << scaleX;
  int16_t stepY = 0;
  int16_t* weight = nullptr;
  if (g_angle2mirror[angle] == 2)
  {
    stepY = -(int)((GEO_WEIGHT_MASK_SIZE << scaleY) + pu.lwidth());
    weight = &g_globalGeoWeights[g_angle2mask[angle]][(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
  }
  else if (g_angle2mirror[angle] == 1)
  {
    stepX = -1 << scaleX;
    stepY = (GEO_WEIGHT_MASK_SIZE << scaleY) + pu.lwidth();
    weight = &g_globalGeoWeights[g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
  }
  else
  {
    stepY = (GEO_WEIGHT_MASK_SIZE << scaleY) - pu.lwidth();
    weight = &g_globalGeoWeights[g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
  }
  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      *dst++  = ClipPel(rightShift((*weight*(*src0++) + ((8 - *weight) * (*src1++)) + offsetWeighted), shiftWeighted), clipRng);
      weight += stepX;
    }
    dst    += strideDst;
    src0   += strideSrc0;
    src1   += strideSrc1;
    weight += stepY;
  }
}

#if JVET_Y0065_GPM_INTRA
void InterpolationFilter::weightedGeoBlkRounded(const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1)
{
  m_weightedGeoBlkRounded(pu, width, height, compIdx, splitDir, predDst, predSrc0, predSrc1);
}

void InterpolationFilter::xWeightedGeoBlkRounded(const PredictionUnit &pu, const uint32_t width, const uint32_t height, const ComponentID compIdx, const uint8_t splitDir, PelUnitBuf& predDst, PelUnitBuf& predSrc0, PelUnitBuf& predSrc1)
{
  Pel*    dst = predDst.get(compIdx).buf;
  Pel*    src0 = predSrc0.get(compIdx).buf;
  Pel*    src1 = predSrc1.get(compIdx).buf;
  int32_t strideDst = predDst.get(compIdx).stride - width;
  int32_t strideSrc0 = predSrc0.get(compIdx).stride - width;
  int32_t strideSrc1 = predSrc1.get(compIdx).stride - width;

  const uint32_t scaleX = getComponentScaleX(compIdx, pu.chromaFormat);
  const uint32_t scaleY = getComponentScaleY(compIdx, pu.chromaFormat);

  int16_t angle = g_GeoParams[splitDir][0];
  int16_t wIdx = floorLog2(pu.lwidth()) - GEO_MIN_CU_LOG2;
  int16_t hIdx = floorLog2(pu.lheight()) - GEO_MIN_CU_LOG2;
  int16_t stepX = 1 << scaleX;
  int16_t stepY = 0;
  int16_t* weight = nullptr;
  if (g_angle2mirror[angle] == 2)
  {
    stepY = -(int)((GEO_WEIGHT_MASK_SIZE << scaleY) + pu.lwidth());
    weight = &g_globalGeoWeights[g_angle2mask[angle]][(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
  }
  else if (g_angle2mirror[angle] == 1)
  {
    stepX = -1 << scaleX;
    stepY = (GEO_WEIGHT_MASK_SIZE << scaleY) + pu.lwidth();
    weight = &g_globalGeoWeights[g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
  }
  else
  {
    stepY = (GEO_WEIGHT_MASK_SIZE << scaleY) - pu.lwidth();
    weight = &g_globalGeoWeights[g_angle2mask[angle]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
  }
  for( int y = 0; y < height; y++ )
  {
    for( int x = 0; x < width; x++ )
    {
      *dst++ = (*weight*(*src0++) + ((8 - *weight) * (*src1++)) + 4)>>3;
      weight += stepX;
    }
    dst    += strideDst;
    src0   += strideSrc0;
    src1   += strideSrc1;
    weight += stepY;
  }
}
#endif

/**
 * \brief turn on SIMD fuc
 *
 * \param bEn   enabled of SIMD function for interpolation
 */
void InterpolationFilter::initInterpolationFilter( bool enable )
{
#if ENABLE_SIMD_OPT_MCIF
#ifdef TARGET_SIMD_X86
  if ( enable )
  {
    initInterpolationFilterX86();
  }
#endif
#endif
}

//! \}
