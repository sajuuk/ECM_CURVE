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

/** \file     EncSearch.cpp
 *  \brief    encoder intra search class
 */

#include "IntraSearch.h"

#include "EncModeCtrl.h"

#include "CommonLib/CommonDef.h"
#include "CommonLib/Rom.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
#include "CommonLib/BilateralFilter.h"
#endif

#include "CommonLib/dtrace_next.h"
#include "CommonLib/dtrace_buffer.h"

#include <math.h>
#include <limits>
 //! \ingroup EncoderLib
 //! \{
#define PLTCtx(c) SubCtx( Ctx::Palette, c )
IntraSearch::IntraSearch()
  : m_pSplitCS      (nullptr)
  , m_pFullCS       (nullptr)
  , m_pBestCS       (nullptr)
  , m_pcEncCfg      (nullptr)
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
  , m_bilateralFilter(nullptr)
#endif
  , m_pcTrQuant     (nullptr)
  , m_pcRdCost      (nullptr)
  , m_pcReshape     (nullptr)
  , m_CABACEstimator(nullptr)
  , m_CtxCache      (nullptr)
  , m_isInitialized (false)
{
  for( uint32_t ch = 0; ch < MAX_NUM_TBLOCKS; ch++ )
  {
    m_pSharedPredTransformSkip[ch] = nullptr;
  }
  m_truncBinBits = nullptr;
  m_escapeNumBins = nullptr;
  m_minErrorIndexMap = nullptr;
  for (unsigned i = 0; i < (MAXPLTSIZE + 1); i++)
  {
    m_indexError[i] = nullptr;
  }
  for (unsigned i = 0; i < NUM_TRELLIS_STATE; i++)
  {
    m_statePtRDOQ[i] = nullptr;
  }
}


void IntraSearch::destroy()
{
  CHECK( !m_isInitialized, "Not initialized" );

  if( m_pcEncCfg )
  {
    const uint32_t uiNumLayersToAllocateSplit = 1;
    const uint32_t uiNumLayersToAllocateFull  = 1;
    const int uiNumSaveLayersToAllocate = 2;

    for( uint32_t layer = 0; layer < uiNumSaveLayersToAllocate; layer++ )
    {
      m_pSaveCS[layer]->destroy();
      delete m_pSaveCS[layer];
    }

    uint32_t numWidths  = gp_sizeIdxInfo->numWidths();
    uint32_t numHeights = gp_sizeIdxInfo->numHeights();

    for( uint32_t width = 0; width < numWidths; width++ )
    {
      for( uint32_t height = 0; height < numHeights; height++ )
      {
        if( gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( width ) ) && gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( height ) ) )
        {
          for( uint32_t layer = 0; layer < uiNumLayersToAllocateSplit; layer++ )
          {
            m_pSplitCS[width][height][layer]->destroy();

            delete m_pSplitCS[width][height][layer];
          }

          for( uint32_t layer = 0; layer < uiNumLayersToAllocateFull; layer++ )
          {
            m_pFullCS[width][height][layer]->destroy();

            delete m_pFullCS[width][height][layer];
          }

          delete[] m_pSplitCS[width][height];
          delete[] m_pFullCS [width][height];

          m_pBestCS[width][height]->destroy();
          m_pTempCS[width][height]->destroy();

          delete m_pTempCS[width][height];
          delete m_pBestCS[width][height];
        }
      }

      delete[] m_pSplitCS[width];
      delete[] m_pFullCS [width];

      delete[] m_pTempCS[width];
      delete[] m_pBestCS[width];
    }

    delete[] m_pSplitCS;
    delete[] m_pFullCS;

    delete[] m_pBestCS;
    delete[] m_pTempCS;

    delete[] m_pSaveCS;
  }

  m_pSplitCS = m_pFullCS = nullptr;

  m_pBestCS = m_pTempCS = nullptr;

  m_pSaveCS = nullptr;

  for( uint32_t ch = 0; ch < MAX_NUM_TBLOCKS; ch++ )
  {
    delete[] m_pSharedPredTransformSkip[ch];
    m_pSharedPredTransformSkip[ch] = nullptr;
  }

#if JVET_AB0143_CCCM_TS
  for (uint32_t cccmIdx = 0; cccmIdx < 6; cccmIdx++)
  {
    m_cccmStorage[cccmIdx].destroy();
  }
#endif

  m_tmpStorageLCU.destroy();
  m_colorTransResiBuf.destroy();
  m_isInitialized = false;
  if (m_truncBinBits != nullptr)
  {
    for (unsigned i = 0; i < m_symbolSize; i++)
    {
      delete[] m_truncBinBits[i];
      m_truncBinBits[i] = nullptr;
    }
    delete[] m_truncBinBits;
    m_truncBinBits = nullptr;
  }
  if (m_escapeNumBins != nullptr)
  {
    delete[] m_escapeNumBins;
    m_escapeNumBins = nullptr;
  }
  if (m_indexError[0] != nullptr)
  {
    for (unsigned i = 0; i < (MAXPLTSIZE + 1); i++)
    {
      delete[] m_indexError[i];
      m_indexError[i] = nullptr;
    }
  }
  if (m_minErrorIndexMap != nullptr)
  {
    delete[] m_minErrorIndexMap;
    m_minErrorIndexMap = nullptr;
  }
  if (m_statePtRDOQ[0] != nullptr)
  {
    for (unsigned i = 0; i < NUM_TRELLIS_STATE; i++)
    {
      delete[] m_statePtRDOQ[i];
      m_statePtRDOQ[i] = nullptr;
    }
  }
}

IntraSearch::~IntraSearch()
{
  if( m_isInitialized )
  {
    destroy();
  }
}

void IntraSearch::init( EncCfg*        pcEncCfg,
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
                       BilateralFilter* bilateralFilter,
#endif
                        TrQuant*       pcTrQuant,
                        RdCost*        pcRdCost,
                        CABACWriter*   CABACEstimator,
                        CtxCache*      ctxCache,
                        const uint32_t     maxCUWidth,
                        const uint32_t     maxCUHeight,
                        const uint32_t     maxTotalCUDepth
                       , EncReshape*   pcReshape
                       , const unsigned bitDepthY
)
{
  CHECK(m_isInitialized, "Already initialized");
  m_pcEncCfg                     = pcEncCfg;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
  m_bilateralFilter              = bilateralFilter;
#endif
  m_pcTrQuant                    = pcTrQuant;
  m_pcRdCost                     = pcRdCost;
  m_CABACEstimator               = CABACEstimator;
  m_CtxCache                     = ctxCache;
  m_pcReshape                    = pcReshape;

  const ChromaFormat cform = pcEncCfg->getChromaFormatIdc();

  IntraPrediction::init( cform, pcEncCfg->getBitDepth( CHANNEL_TYPE_LUMA ) );
  m_tmpStorageLCU.create(UnitArea(cform, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
  m_colorTransResiBuf.create(UnitArea(cform, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));

#if JVET_AB0143_CCCM_TS
  for (uint32_t cccmIdx = 0; cccmIdx < 6; cccmIdx++)
  {
    m_cccmStorage[cccmIdx].create(UnitArea(cform, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
  }
#endif

  for( uint32_t ch = 0; ch < MAX_NUM_TBLOCKS; ch++ )
  {
    m_pSharedPredTransformSkip[ch] = new Pel[MAX_CU_SIZE * MAX_CU_SIZE];
  }

  uint32_t numWidths  = gp_sizeIdxInfo->numWidths();
  uint32_t numHeights = gp_sizeIdxInfo->numHeights();

  const uint32_t uiNumLayersToAllocateSplit = 1;
  const uint32_t uiNumLayersToAllocateFull  = 1;

  m_pBestCS = new CodingStructure**[numWidths];
  m_pTempCS = new CodingStructure**[numWidths];

  m_pFullCS  = new CodingStructure***[numWidths];
  m_pSplitCS = new CodingStructure***[numWidths];

  for( uint32_t width = 0; width < numWidths; width++ )
  {
    m_pBestCS[width] = new CodingStructure*[numHeights];
    m_pTempCS[width] = new CodingStructure*[numHeights];

    m_pFullCS [width] = new CodingStructure**[numHeights];
    m_pSplitCS[width] = new CodingStructure**[numHeights];

    for( uint32_t height = 0; height < numHeights; height++ )
    {
      if(  gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( width ) ) && gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( height ) ) )
      {
        m_pBestCS[width][height] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
        m_pTempCS[width][height] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );

#if JVET_Z0118_GDR
        m_pBestCS[width][height]->create(m_pcEncCfg->getChromaFormatIdc(), Area(0, 0, gp_sizeIdxInfo->sizeFrom(width), gp_sizeIdxInfo->sizeFrom(height)), false, (bool)pcEncCfg->getPLTMode(), pcEncCfg->getGdrEnabled());
        m_pTempCS[width][height]->create(m_pcEncCfg->getChromaFormatIdc(), Area(0, 0, gp_sizeIdxInfo->sizeFrom(width), gp_sizeIdxInfo->sizeFrom(height)), false, (bool)pcEncCfg->getPLTMode(), pcEncCfg->getGdrEnabled());
#else
        m_pBestCS[width][height]->create(m_pcEncCfg->getChromaFormatIdc(), Area(0, 0, gp_sizeIdxInfo->sizeFrom(width), gp_sizeIdxInfo->sizeFrom(height)), false, (bool)pcEncCfg->getPLTMode());
        m_pTempCS[width][height]->create(m_pcEncCfg->getChromaFormatIdc(), Area(0, 0, gp_sizeIdxInfo->sizeFrom(width), gp_sizeIdxInfo->sizeFrom(height)), false, (bool)pcEncCfg->getPLTMode());
#endif

        m_pFullCS [width][height] = new CodingStructure*[uiNumLayersToAllocateFull];
        m_pSplitCS[width][height] = new CodingStructure*[uiNumLayersToAllocateSplit];

        for( uint32_t layer = 0; layer < uiNumLayersToAllocateFull; layer++ )
        {
          m_pFullCS [width][height][layer] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );

#if JVET_Z0118_GDR
          m_pFullCS[width][height][layer]->create(m_pcEncCfg->getChromaFormatIdc(), Area(0, 0, gp_sizeIdxInfo->sizeFrom(width), gp_sizeIdxInfo->sizeFrom(height)), false, (bool)pcEncCfg->getPLTMode(), pcEncCfg->getGdrEnabled());
#else
          m_pFullCS[width][height][layer]->create(m_pcEncCfg->getChromaFormatIdc(), Area(0, 0, gp_sizeIdxInfo->sizeFrom(width), gp_sizeIdxInfo->sizeFrom(height)), false, (bool)pcEncCfg->getPLTMode());
#endif
        }

        for( uint32_t layer = 0; layer < uiNumLayersToAllocateSplit; layer++ )
        {
          m_pSplitCS[width][height][layer] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
#if JVET_Z0118_GDR
          m_pSplitCS[width][height][layer]->create(m_pcEncCfg->getChromaFormatIdc(), Area(0, 0, gp_sizeIdxInfo->sizeFrom(width), gp_sizeIdxInfo->sizeFrom(height)), false, (bool)pcEncCfg->getPLTMode(), pcEncCfg->getGdrEnabled());
#else
          m_pSplitCS[width][height][layer]->create(m_pcEncCfg->getChromaFormatIdc(), Area(0, 0, gp_sizeIdxInfo->sizeFrom(width), gp_sizeIdxInfo->sizeFrom(height)), false, (bool)pcEncCfg->getPLTMode());
#endif
        }
      }
      else
      {
        m_pBestCS[width][height] = nullptr;
        m_pTempCS[width][height] = nullptr;

        m_pFullCS [width][height] = nullptr;
        m_pSplitCS[width][height] = nullptr;
      }
    }
  }

  const int uiNumSaveLayersToAllocate = 2;

  m_pSaveCS = new CodingStructure*[uiNumSaveLayersToAllocate];

  for( uint32_t depth = 0; depth < uiNumSaveLayersToAllocate; depth++ )
  {
    m_pSaveCS[depth] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
#if JVET_Z0118_GDR
    m_pSaveCS[depth]->create(UnitArea(cform, Area(0, 0, maxCUWidth, maxCUHeight)), false, (bool)pcEncCfg->getPLTMode(), pcEncCfg->getGdrEnabled());
#else
    m_pSaveCS[depth]->create(UnitArea(cform, Area(0, 0, maxCUWidth, maxCUHeight)), false, (bool)pcEncCfg->getPLTMode());
#endif
  }

  m_isInitialized = true;
  if (pcEncCfg->getPLTMode())
  {
    m_symbolSize = (1 << bitDepthY); // pixel values are within [0, SymbolSize-1] with size SymbolSize
    if (m_truncBinBits == nullptr)
    {
      m_truncBinBits = new uint16_t*[m_symbolSize];
      for (unsigned i = 0; i < m_symbolSize; i++)
      {
        m_truncBinBits[i] = new uint16_t[m_symbolSize + 1];
      }
    }
    if (m_escapeNumBins == nullptr)
    {
      m_escapeNumBins = new uint16_t[m_symbolSize];
    }
    initTBCTable(bitDepthY);
    if (m_indexError[0] == nullptr)
    {
      for (unsigned i = 0; i < (MAXPLTSIZE + 1); i++)
      {
        m_indexError[i] = new double[MAX_CU_BLKSIZE_PLT*MAX_CU_BLKSIZE_PLT];
      }
    }
    if (m_minErrorIndexMap == nullptr)
    {
      m_minErrorIndexMap = new uint8_t[MAX_CU_BLKSIZE_PLT*MAX_CU_BLKSIZE_PLT];
    }
    if (m_statePtRDOQ[0] == nullptr)
    {
      for (unsigned i = 0; i < NUM_TRELLIS_STATE; i++)
      {
        m_statePtRDOQ[i] = new uint8_t[MAX_CU_BLKSIZE_PLT*MAX_CU_BLKSIZE_PLT];
      }
    }
  }
#if INTRA_TRANS_ENC_OPT
  m_skipTimdLfnstMtsPass = false;
#endif
}


//////////////////////////////////////////////////////////////////////////
// INTRA PREDICTION
//////////////////////////////////////////////////////////////////////////
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
static constexpr double COST_UNKNOWN = -65536.0;

double IntraSearch::findInterCUCost( CodingUnit &cu )
{
  if( cu.isConsIntra() && !cu.slice->isIntra() )
  {
    //search corresponding inter CU cost
    for( int i = 0; i < m_numCuInSCIPU; i++ )
    {
      if( cu.lumaPos() == m_cuAreaInSCIPU[i].pos() && cu.lumaSize() == m_cuAreaInSCIPU[i].size() )
      {
        return m_cuCostInSCIPU[i];
      }
    }
  }
  return COST_UNKNOWN;
}
#endif
#if JVET_W0103_INTRA_MTS
bool IntraSearch::testISPforCurrCU(const CodingUnit &cu)
{
  CodingStructure       &cs = *cu.cs;
  auto &pu = *cu.firstPU;
  const CompArea &area = pu.Y();
  PelBuf piOrg = cs.getOrgBuf(area);

  Pel* pOrg = piOrg.buf;
  int uiWidth = area.width;
  int uiHeight = area.height;
  int iStride = piOrg.stride;
  int Gsum = 0;
  int nPix = (uiWidth - 2) * (uiHeight - 2);
  for (int y = 1; y < (uiHeight - 1); y++)
  {
    for (int x = 1; x < (uiWidth - 1); x++)
    {
      const Pel *p = pOrg + y * iStride + x;

      int iDy = p[-iStride - 1] + 2 * p[-1] + p[iStride - 1] - p[-iStride + 1] - 2 * p[+1] - p[iStride + 1];
      int iDx = p[iStride - 1] + 2 * p[iStride] + p[iStride + 1] - p[-iStride - 1] - 2 * p[-iStride] - p[-iStride + 1];

      if (iDy == 0 && iDx == 0)
        continue;

      int iAmp = (int)(abs(iDx) + abs(iDy));
      Gsum += iAmp;
    }
  }
  Gsum = (Gsum + (nPix >> 1)) / nPix;

  bool testISP = true;
  CHECK(m_numModesISPRDO != -1, "m_numModesISPRDO!=-1");

  m_numModesISPRDO = (Gsum < 50 && uiWidth >= 16 && uiHeight >= 16) ? 1 : 2;
  return testISP;
}
#endif
bool IntraSearch::estIntraPredLumaQT(CodingUnit &cu, Partitioner &partitioner, const double bestCostSoFar, bool mtsCheckRangeFlag, int mtsFirstCheckId, int mtsLastCheckId, bool moreProbMTSIdxFirst, CodingStructure* bestCS)
{
  CodingStructure       &cs            = *cu.cs;
  const SPS             &sps           = *cs.sps;
  const uint32_t             uiWidthBit    = floorLog2(partitioner.currArea().lwidth() );
  const uint32_t             uiHeightBit   =                   floorLog2(partitioner.currArea().lheight());

  // Lambda calculation at equivalent Qp of 4 is recommended because at that Qp, the quantization divisor is 1.
  const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda( ) * FRAC_BITS_SCALE;

  //===== loop over partitions =====

  const TempCtx ctxStart          ( m_CtxCache, m_CABACEstimator->getCtx() );
  const TempCtx ctxStartMipFlag    ( m_CtxCache, SubCtx( Ctx::MipFlag,          m_CABACEstimator->getCtx() ) );
#if JVET_V0130_INTRA_TMP
  const TempCtx ctxStartTpmFlag(m_CtxCache, SubCtx(Ctx::TmpFlag, m_CABACEstimator->getCtx()));
#endif
#if JVET_W0123_TIMD_FUSION
  const TempCtx ctxStartTimdFlag   ( m_CtxCache, SubCtx( Ctx::TimdFlag,      m_CABACEstimator->getCtx() ) );
#endif
  const TempCtx ctxStartIspMode    ( m_CtxCache, SubCtx( Ctx::ISPMode,          m_CABACEstimator->getCtx() ) );
#if SECONDARY_MPM
  const TempCtx ctxStartMPMIdxFlag(m_CtxCache, SubCtx(Ctx::IntraLumaMPMIdx, m_CABACEstimator->getCtx()));
#endif
  const TempCtx ctxStartPlanarFlag ( m_CtxCache, SubCtx( Ctx::IntraLumaPlanarFlag, m_CABACEstimator->getCtx() ) );
  const TempCtx ctxStartIntraMode(m_CtxCache, SubCtx(Ctx::IntraLumaMpmFlag, m_CABACEstimator->getCtx()));
#if SECONDARY_MPM
  const TempCtx ctxStartIntraMode2(m_CtxCache, SubCtx(Ctx::IntraLumaSecondMpmFlag, m_CABACEstimator->getCtx()));
#endif
  const TempCtx ctxStartMrlIdx      ( m_CtxCache, SubCtx( Ctx::MultiRefLineIdx,        m_CABACEstimator->getCtx() ) );

  CHECK( !cu.firstPU, "CU has no PUs" );
  // variables for saving fast intra modes scan results across multiple LFNST passes
  bool LFNSTLoadFlag = sps.getUseLFNST() && cu.lfnstIdx != 0;
  bool LFNSTSaveFlag = sps.getUseLFNST() && cu.lfnstIdx == 0;

  LFNSTSaveFlag &= sps.getUseIntraMTS() ? cu.mtsFlag == 0 : true;

  const uint32_t lfnstIdx = cu.lfnstIdx;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  double costInterCU = findInterCUCost( cu );
#endif
  const int width  = partitioner.currArea().lwidth();
  const int height = partitioner.currArea().lheight();

  // Marking MTS usage for faster MTS
  // 0: MTS is either not applicable for current CU (cuWidth > MTS_INTRA_MAX_CU_SIZE or cuHeight > MTS_INTRA_MAX_CU_SIZE), not active in the config file or the fast decision algorithm is not used in this case
  // 1: MTS fast algorithm can be applied for the current CU, and the DCT2 is being checked
  // 2: MTS is being checked for current CU. Stored results of DCT2 can be utilized for speedup
  uint8_t mtsUsageFlag = 0;
  const int maxSizeEMT = MTS_INTRA_MAX_CU_SIZE;
  if( width <= maxSizeEMT && height <= maxSizeEMT && sps.getUseIntraMTS() )
  {
    mtsUsageFlag = ( sps.getUseLFNST() && cu.mtsFlag == 1 ) ? 2 : 1;
  }

  if( width * height < 64 && !m_pcEncCfg->getUseFastLFNST() )
  {
    mtsUsageFlag = 0;
  }

#if JVET_W0103_INTRA_MTS
  if (!cu.mtsFlag && !cu.lfnstIdx)
  {
    m_globalBestCostStore = MAX_DOUBLE;
    m_globalBestCostValid = false;
    if (bestCS->getCU(partitioner.chType) != NULL && bestCS->getCU(partitioner.chType)->predMode != MODE_INTRA && bestCostSoFar != MAX_DOUBLE)
    {
      m_globalBestCostStore = bestCostSoFar;
      m_globalBestCostValid = true;
    }
#if JVET_Y0142_ADAPT_INTRA_MTS
    m_modesForMTS.clear();
    m_modesCoeffAbsSumDCT2.clear();
#endif
  }
#endif
  const bool colorTransformIsEnabled = sps.getUseColorTrans() && !CS::isDualITree(cs);
  const bool isFirstColorSpace       = colorTransformIsEnabled && ((m_pcEncCfg->getRGBFormatFlag() && cu.colorTransform) || (!m_pcEncCfg->getRGBFormatFlag() && !cu.colorTransform));
  const bool isSecondColorSpace      = colorTransformIsEnabled && ((m_pcEncCfg->getRGBFormatFlag() && !cu.colorTransform) || (!m_pcEncCfg->getRGBFormatFlag() && cu.colorTransform));

  double bestCurrentCost = bestCostSoFar;
  bool ispCanBeUsed   = sps.getUseISP() && cu.mtsFlag == 0 && cu.lfnstIdx == 0 && CU::canUseISP(width, height, cu.cs->sps->getMaxTbSize());
  bool saveDataForISP = ispCanBeUsed && (!colorTransformIsEnabled || isFirstColorSpace);
  bool testISP        = ispCanBeUsed && (!colorTransformIsEnabled || !cu.colorTransform);
#if JVET_W0103_INTRA_MTS 
  if (testISP && m_pcEncCfg->getUseFastISP())
  {
    m_numModesISPRDO = -1;
    testISP &= testISPforCurrCU(cu);
  }
#endif
  if ( saveDataForISP )
  {
    //reset the intra modes lists variables
    m_ispCandListHor.clear();
    m_ispCandListVer.clear();
  }
  if( testISP )
  {
    //reset the variables used for the tests
    m_regIntraRDListWithCosts.clear();
    int numTotalPartsHor = (int)width  >> floorLog2(CU::getISPSplitDim(width, height, TU_1D_VERT_SPLIT));
    int numTotalPartsVer = (int)height >> floorLog2(CU::getISPSplitDim(width, height, TU_1D_HORZ_SPLIT));
    m_ispTestedModes[0].init( numTotalPartsHor, numTotalPartsVer );
    //the total number of subpartitions is modified to take into account the cases where LFNST cannot be combined with ISP due to size restrictions
    numTotalPartsHor = sps.getUseLFNST() && CU::canUseLfnstWithISP(cu.Y(), HOR_INTRA_SUBPARTITIONS) ? numTotalPartsHor : 0;
    numTotalPartsVer = sps.getUseLFNST() && CU::canUseLfnstWithISP(cu.Y(), VER_INTRA_SUBPARTITIONS) ? numTotalPartsVer : 0;
    for (int j = 1; j < NUM_LFNST_NUM_PER_SET; j++)
    {
      m_ispTestedModes[j].init(numTotalPartsHor, numTotalPartsVer);
    }
  }

#if INTRA_TRANS_ENC_OPT
  double regAngCost = MAX_DOUBLE;
  bool setSkipTimdControl = (m_pcEncCfg->getIntraPeriod() == 1) && !cu.lfnstIdx && !cu.mtsFlag;
  double timdAngCost = MAX_DOUBLE;
#endif
  const bool testBDPCM = sps.getBDPCMEnabledFlag() && CU::bdpcmAllowed(cu, ComponentID(partitioner.chType)) && cu.mtsFlag == 0 && cu.lfnstIdx == 0;
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> uiHadModeList;
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM> CandCostList;
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM> CandHadList;

  auto &pu = *cu.firstPU;
  bool validReturn = false;
  {
    CandHadList.clear();
    CandCostList.clear();
    uiHadModeList.clear();

    CHECK(pu.cu != &cu, "PU is not contained in the CU");

#if SECONDARY_MPM
    std::memcpy( pu.intraMPM, m_mpmList, sizeof( pu.intraMPM ) );
    std::memcpy( pu.intraNonMPM, m_nonMPMList, sizeof( pu.intraNonMPM ) );
#endif

    //===== determine set of modes to be tested (using prediction signal only) =====
    int numModesAvailable = NUM_LUMA_MODE; // total number of Intra modes
    const bool fastMip    = sps.getUseMIP() && m_pcEncCfg->getUseFastMIP();
    const bool mipAllowed = sps.getUseMIP() && isLuma(partitioner.chType) && ((cu.lfnstIdx == 0) || allowLfnstWithMip(cu.firstPU->lumaSize()));
    const bool testMip = mipAllowed && !(cu.lwidth() > (8 * cu.lheight()) || cu.lheight() > (8 * cu.lwidth()));
    const bool supportedMipBlkSize = pu.lwidth() <= MIP_MAX_WIDTH && pu.lheight() <= MIP_MAX_HEIGHT;
#if JVET_V0130_INTRA_TMP
    const bool tpmAllowed = sps.getUseIntraTMP() && isLuma(partitioner.chType) && ((cu.lfnstIdx == 0) || allowLfnstWithTmp());
    const bool testTpm = tpmAllowed && (cu.lwidth() <= sps.getIntraTMPMaxSize() && cu.lheight() <= sps.getIntraTMPMaxSize());
#endif

    static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> uiRdModeList;

    int numModesForFullRD = 3;
    numModesForFullRD = g_aucIntraModeNumFast_UseMPM_2D[uiWidthBit - MIN_CU_LOG2][uiHeightBit - MIN_CU_LOG2];

#if INTRA_FULL_SEARCH
    numModesForFullRD = numModesAvailable;
#endif
#if ENABLE_DIMD
    bool bestDimdMode = false;
#endif
#if JVET_W0123_TIMD_FUSION
    bool bestTimdMode = false;
#endif
    if (isSecondColorSpace)
    {
      uiRdModeList.clear();
      if (m_numSavedRdModeFirstColorSpace[m_savedRdModeIdx] > 0)
      {
        for (int i = 0; i < m_numSavedRdModeFirstColorSpace[m_savedRdModeIdx]; i++)
        {
          uiRdModeList.push_back(m_savedRdModeFirstColorSpace[m_savedRdModeIdx][i]);
        }
      }
      else
      {
        return false;
      }
    }
    else
    {
      if (mtsUsageFlag != 2)
      {
        // this should always be true
        CHECK(!pu.Y().valid(), "PU is not valid");
        bool isFirstLineOfCtu     = (((pu.block(COMPONENT_Y).y) & ((pu.cs->sps)->getMaxCUWidth() - 1)) == 0);
#if JVET_Y0116_EXTENDED_MRL_LIST
        int  numOfPassesExtendRef = MRL_NUM_REF_LINES;
        if (!sps.getUseMRL() || isFirstLineOfCtu) 
        {
          numOfPassesExtendRef = 1;
        }
        else
        {
          bool checkLineOutsideCtu[MRL_NUM_REF_LINES - 1];
          for (int mrlIdx = 1; mrlIdx < MRL_NUM_REF_LINES; mrlIdx++)
          {
            bool isLineOutsideCtu =
              ((cu.block(COMPONENT_Y).y) % ((cu.cs->sps)->getMaxCUWidth()) <= MULTI_REF_LINE_IDX[mrlIdx]) ? true
                                                                                                          : false;
            checkLineOutsideCtu[mrlIdx-1] = isLineOutsideCtu;
          }
          if (checkLineOutsideCtu[0]) 
          {
            numOfPassesExtendRef = 1;
          }
          else
          {
            for (int mrlIdx = MRL_NUM_REF_LINES - 2; mrlIdx > 0; mrlIdx--)
            {
              if (checkLineOutsideCtu[mrlIdx] && !checkLineOutsideCtu[mrlIdx - 1])
              {
                numOfPassesExtendRef = mrlIdx + 1;
                break;
              }
            }
          }
        }
#else
        int  numOfPassesExtendRef = ((!sps.getUseMRL() || isFirstLineOfCtu) ? 1 : MRL_NUM_REF_LINES);
#endif
        pu.multiRefIdx            = 0;

        if (numModesForFullRD != numModesAvailable)
        {
          CHECK(numModesForFullRD >= numModesAvailable, "Too many modes for full RD search");

          const CompArea &area = pu.Y();

          PelBuf piOrg  = cs.getOrgBuf(area);
          PelBuf piPred = cs.getPredBuf(area);

          DistParam distParamSad;
          DistParam distParamHad;
          if (cu.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
          {
            CompArea tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
            PelBuf   tmpOrg = m_tmpStorageLCU.getBuf(tmpArea);
            tmpOrg.rspSignal( piOrg, m_pcReshape->getFwdLUT() );
            m_pcRdCost->setDistParam(distParamSad, tmpOrg, piPred, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y,
                                     false);   // Use SAD cost
            m_pcRdCost->setDistParam(distParamHad, tmpOrg, piPred, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y,
                                     true);   // Use HAD (SATD) cost
          }
          else
          {
            m_pcRdCost->setDistParam(distParamSad, piOrg, piPred, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y,
                                     false);   // Use SAD cost
            m_pcRdCost->setDistParam(distParamHad, piOrg, piPred, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y,
                                     true);   // Use HAD (SATD) cost
          }

          distParamSad.applyWeight = false;
          distParamHad.applyWeight = false;

          if (testMip && supportedMipBlkSize)
          {
            numModesForFullRD += fastMip
                                   ? std::max(numModesForFullRD, floorLog2(std::min(pu.lwidth(), pu.lheight())) - 1)
                                   : numModesForFullRD;
          }
#if JVET_V0130_INTRA_TMP
          if( testTpm )
          {
            numModesForFullRD += 1; // testing tpm
          }
          const int numHadCand = (testMip ? 2 : 1) * 3 + testTpm;
          
          cu.tmpFlag = false;
#else
          const int numHadCand = (testMip ? 2 : 1) * 3;
#endif

          //*** Derive (regular) candidates using Hadamard
          cu.mipFlag = false;

          //===== init pattern for luma prediction =====
          initIntraPatternChType(cu, pu.Y(), true);
          bool bSatdChecked[NUM_INTRA_MODE];
          memset(bSatdChecked, 0, sizeof(bSatdChecked));

          if (!LFNSTLoadFlag)
          {
            for (int modeIdx = 0; modeIdx < numModesAvailable; modeIdx++)
            {
              uint32_t   uiMode    = modeIdx;
              Distortion minSadHad = 0;

              // Skip checking extended Angular modes in the first round of SATD
              if (uiMode > DC_IDX && (uiMode & 1))
              {
                continue;
              }

              bSatdChecked[uiMode] = true;

              pu.intraDir[0] = modeIdx;

              initPredIntraParams(pu, pu.Y(), sps);
              predIntraAng(COMPONENT_Y, piPred, pu);
              // Use the min between SAD and HAD as the cost criterion
              // SAD is scaled by 2 to align with the scaling of HAD
              minSadHad += std::min(distParamSad.distFunc(distParamSad) * 2, distParamHad.distFunc(distParamHad));

              // NB xFracModeBitsIntra will not affect the mode for chroma that may have already been pre-estimated.
#if JVET_V0130_INTRA_TMP
              m_CABACEstimator->getCtx() = SubCtx( Ctx::TmpFlag, ctxStartTpmFlag );
#endif
              m_CABACEstimator->getCtx() = SubCtx( Ctx::MipFlag, ctxStartMipFlag );
#if JVET_W0123_TIMD_FUSION
              m_CABACEstimator->getCtx() = SubCtx( Ctx::TimdFlag, ctxStartTimdFlag );
#endif
              m_CABACEstimator->getCtx() = SubCtx( Ctx::ISPMode, ctxStartIspMode );
#if SECONDARY_MPM
              m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaMPMIdx, ctxStartMPMIdxFlag);
#endif
              m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaPlanarFlag, ctxStartPlanarFlag);
              m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaMpmFlag, ctxStartIntraMode);
#if SECONDARY_MPM
              m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaSecondMpmFlag, ctxStartIntraMode2);
#endif
              m_CABACEstimator->getCtx() = SubCtx( Ctx::MultiRefLineIdx, ctxStartMrlIdx );

              uint64_t fracModeBits = xFracModeBitsIntra(pu, uiMode, CHANNEL_TYPE_LUMA);

              double cost = (double) minSadHad + (double) fracModeBits * sqrtLambdaForFirstPass;

              DTRACE(g_trace_ctx, D_INTRA_COST, "IntraHAD: %u, %llu, %f (%d)\n", minSadHad, fracModeBits, cost, uiMode);

              updateCandList(ModeInfo(false, false, 0, NOT_INTRA_SUBPARTITIONS, uiMode), cost, uiRdModeList,
                             CandCostList, numModesForFullRD);
              updateCandList(ModeInfo(false, false, 0, NOT_INTRA_SUBPARTITIONS, uiMode), double(minSadHad),
                             uiHadModeList, CandHadList, numHadCand);
            }
            if (!sps.getUseMIP() && LFNSTSaveFlag)
            {
              // save found best modes
              m_uiSavedNumRdModesLFNST = numModesForFullRD;
              m_uiSavedRdModeListLFNST = uiRdModeList;
              m_dSavedModeCostLFNST    = CandCostList;
              // PBINTRA fast
              m_uiSavedHadModeListLFNST = uiHadModeList;
              m_dSavedHadListLFNST      = CandHadList;
              LFNSTSaveFlag             = false;
            }
          }   // NSSTFlag
          if (!sps.getUseMIP() && LFNSTLoadFlag)
          {
            // restore saved modes
            numModesForFullRD = m_uiSavedNumRdModesLFNST;
            uiRdModeList      = m_uiSavedRdModeListLFNST;
            CandCostList      = m_dSavedModeCostLFNST;
            // PBINTRA fast
            uiHadModeList = m_uiSavedHadModeListLFNST;
            CandHadList   = m_dSavedHadListLFNST;
          }   // !LFNSTFlag

          if (!(sps.getUseMIP() && LFNSTLoadFlag))
          {
            static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> parentCandList = uiRdModeList;

            // Second round of SATD for extended Angular modes
            for (int modeIdx = 0; modeIdx < numModesForFullRD; modeIdx++)
            {
              unsigned parentMode = parentCandList[modeIdx].modeId;
              if (parentMode > (DC_IDX + 1) && parentMode < (NUM_LUMA_MODE - 1))
              {
                for (int subModeIdx = -1; subModeIdx <= 1; subModeIdx += 2)
                {
                  unsigned mode = parentMode + subModeIdx;

                  if (!bSatdChecked[mode])
                  {
                    pu.intraDir[0] = mode;

                    initPredIntraParams(pu, pu.Y(), sps);
                    predIntraAng(COMPONENT_Y, piPred, pu);

                    // Use the min between SAD and SATD as the cost criterion
                    // SAD is scaled by 2 to align with the scaling of HAD
                    Distortion minSadHad =
                      std::min(distParamSad.distFunc(distParamSad) * 2, distParamHad.distFunc(distParamHad));

                    // NB xFracModeBitsIntra will not affect the mode for chroma that may have already been
                    // pre-estimated.
#if JVET_V0130_INTRA_TMP
                    m_CABACEstimator->getCtx() = SubCtx( Ctx::TmpFlag, ctxStartTpmFlag );
#endif
                    m_CABACEstimator->getCtx() = SubCtx(Ctx::MipFlag, ctxStartMipFlag);
#if JVET_W0123_TIMD_FUSION
                    m_CABACEstimator->getCtx() = SubCtx( Ctx::TimdFlag, ctxStartTimdFlag );
#endif
                    m_CABACEstimator->getCtx() = SubCtx(Ctx::ISPMode, ctxStartIspMode);
#if SECONDARY_MPM
                    m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaMPMIdx, ctxStartMPMIdxFlag);
#endif
                    m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaPlanarFlag, ctxStartPlanarFlag);
                    m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaMpmFlag, ctxStartIntraMode);
#if SECONDARY_MPM
                    m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaSecondMpmFlag, ctxStartIntraMode2);
#endif
                    m_CABACEstimator->getCtx() = SubCtx(Ctx::MultiRefLineIdx, ctxStartMrlIdx);

                    uint64_t fracModeBits = xFracModeBitsIntra(pu, mode, CHANNEL_TYPE_LUMA);

                    double cost = (double) minSadHad + (double) fracModeBits * sqrtLambdaForFirstPass;

                    updateCandList(ModeInfo(false, false, 0, NOT_INTRA_SUBPARTITIONS, mode), cost, uiRdModeList,
                                   CandCostList, numModesForFullRD);
                    updateCandList(ModeInfo(false, false, 0, NOT_INTRA_SUBPARTITIONS, mode), double(minSadHad),
                                   uiHadModeList, CandHadList, numHadCand);

                    bSatdChecked[mode] = true;
                  }
                }
              }
            }
            if (saveDataForISP)
            {
              // we save the regular intra modes list
              m_ispCandListHor = uiRdModeList;
            }
            pu.multiRefIdx    = 1;
#if SECONDARY_MPM
            const int numMPMs = NUM_PRIMARY_MOST_PROBABLE_MODES;
            uint8_t* multiRefMPM = m_mpmList;
#else
            const int numMPMs = NUM_MOST_PROBABLE_MODES;
            unsigned  multiRefMPM[numMPMs];
#endif
#if !SECONDARY_MPM
            PU::getIntraMPMs(pu, multiRefMPM);
#endif
            for (int mRefNum = 1; mRefNum < numOfPassesExtendRef; mRefNum++)
            {
              int multiRefIdx = MULTI_REF_LINE_IDX[mRefNum];

              pu.multiRefIdx = multiRefIdx;
              {
                initIntraPatternChType(cu, pu.Y(), true);
              }
              for (int x = 1; x < numMPMs; x++)
              {
                uint32_t mode = multiRefMPM[x];
                {
                  pu.intraDir[0] = mode;
                  initPredIntraParams(pu, pu.Y(), sps);

                  predIntraAng(COMPONENT_Y, piPred, pu);

                  // Use the min between SAD and SATD as the cost criterion
                  // SAD is scaled by 2 to align with the scaling of HAD
                  Distortion minSadHad =
                    std::min(distParamSad.distFunc(distParamSad) * 2, distParamHad.distFunc(distParamHad));

                  // NB xFracModeBitsIntra will not affect the mode for chroma that may have already been pre-estimated.
#if JVET_V0130_INTRA_TMP
                  m_CABACEstimator->getCtx() = SubCtx( Ctx::TmpFlag, ctxStartTpmFlag );
#endif
                  m_CABACEstimator->getCtx() = SubCtx(Ctx::MipFlag, ctxStartMipFlag);
#if JVET_W0123_TIMD_FUSION
                  m_CABACEstimator->getCtx() = SubCtx( Ctx::TimdFlag, ctxStartTimdFlag );
#endif
                  m_CABACEstimator->getCtx() = SubCtx(Ctx::ISPMode, ctxStartIspMode);
#if SECONDARY_MPM
                  m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaMPMIdx, ctxStartMPMIdxFlag);
#endif
                  m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaPlanarFlag, ctxStartPlanarFlag);
                  m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaMpmFlag, ctxStartIntraMode);
#if SECONDARY_MPM
                  m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaSecondMpmFlag, ctxStartIntraMode2);
#endif
                  m_CABACEstimator->getCtx() = SubCtx(Ctx::MultiRefLineIdx, ctxStartMrlIdx);

                  uint64_t fracModeBits = xFracModeBitsIntra(pu, mode, CHANNEL_TYPE_LUMA);

                  double cost = (double) minSadHad + (double) fracModeBits * sqrtLambdaForFirstPass;
                  updateCandList(ModeInfo(false, false, multiRefIdx, NOT_INTRA_SUBPARTITIONS, mode), cost, uiRdModeList,
                                 CandCostList, numModesForFullRD);
                  updateCandList(ModeInfo(false, false, multiRefIdx, NOT_INTRA_SUBPARTITIONS, mode), double(minSadHad),
                                 uiHadModeList, CandHadList, numHadCand);
                }
              }
            }
            CHECKD(uiRdModeList.size() != numModesForFullRD, "Error: RD mode list size");

            if (LFNSTSaveFlag && testMip
                && !allowLfnstWithMip(cu.firstPU->lumaSize()))   // save a different set for the next run
            {
              // save found best modes
              m_uiSavedRdModeListLFNST = uiRdModeList;
              m_dSavedModeCostLFNST    = CandCostList;
              // PBINTRA fast
              m_uiSavedHadModeListLFNST = uiHadModeList;
              m_dSavedHadListLFNST      = CandHadList;
              m_uiSavedNumRdModesLFNST =
                g_aucIntraModeNumFast_UseMPM_2D[uiWidthBit - MIN_CU_LOG2][uiHeightBit - MIN_CU_LOG2];
              m_uiSavedRdModeListLFNST.resize(m_uiSavedNumRdModesLFNST);
              m_dSavedModeCostLFNST.resize(m_uiSavedNumRdModesLFNST);
              // PBINTRA fast
              m_uiSavedHadModeListLFNST.resize(3);
              m_dSavedHadListLFNST.resize(3);
              LFNSTSaveFlag = false;
            }
#if JVET_V0130_INTRA_TMP
            // derive TPM candidate using hadamard
            if( testTpm )
            {
              cu.tmpFlag = true;
              cu.mipFlag = false;
              pu.multiRefIdx = 0;

              int foundCandiNum = 0;
              bool bsuccessfull = 0;
              CodingUnit cu_cpy = cu;

#if JVET_W0069_TMP_BOUNDARY
              RefTemplateType templateType = getRefTemplateType( cu_cpy, cu_cpy.blocks[COMPONENT_Y] );
              if( templateType != NO_TEMPLATE )
#else
              if( isRefTemplateAvailable( cu_cpy, cu_cpy.blocks[COMPONENT_Y] ) )
#endif
              {
#if JVET_W0069_TMP_BOUNDARY
                getTargetTemplate( &cu_cpy, pu.lwidth(), pu.lheight(), templateType );
                candidateSearchIntra( &cu_cpy, pu.lwidth(), pu.lheight(), templateType );
                bsuccessfull = generateTMPrediction( piPred.buf, piPred.stride, pu.lwidth(), pu.lheight(), foundCandiNum );
#else
                getTargetTemplate( &cu_cpy, pu.lwidth(), pu.lheight() );
                candidateSearchIntra( &cu_cpy, pu.lwidth(), pu.lheight() );
                bsuccessfull = generateTMPrediction( piPred.buf, piPred.stride, pu.lwidth(), pu.lheight(), foundCandiNum );
#endif
              }
#if JVET_W0069_TMP_BOUNDARY
              else
              {
                foundCandiNum = 1;
                bsuccessfull = generateTmDcPrediction( piPred.buf, piPred.stride, pu.lwidth(), pu.lheight(), 1 << (cu_cpy.cs->sps->getBitDepth( CHANNEL_TYPE_LUMA ) - 1) );
              }
#endif
              if( bsuccessfull && foundCandiNum >= 1 )
              {

                Distortion minSadHad =
                  std::min( distParamSad.distFunc( distParamSad ) * 2, distParamHad.distFunc( distParamHad ) );

                m_CABACEstimator->getCtx() = SubCtx( Ctx::TmpFlag, ctxStartTpmFlag );

                uint64_t fracModeBits = xFracModeBitsIntra( pu, 0, CHANNEL_TYPE_LUMA );

                double cost = double( minSadHad ) + double( fracModeBits ) * sqrtLambdaForFirstPass;
                DTRACE( g_trace_ctx, D_INTRA_COST, "IntraTPM: %u, %llu, %f (%d)\n", minSadHad, fracModeBits, cost, 0 );

                updateCandList( ModeInfo( 0, 0, 0, NOT_INTRA_SUBPARTITIONS, 0, 1 ), cost, uiRdModeList, CandCostList, numModesForFullRD );
                updateCandList( ModeInfo( 0, 0, 0, NOT_INTRA_SUBPARTITIONS, 0, 1 ), 0.8 * double( minSadHad ), uiHadModeList, CandHadList, numHadCand );
              }
            }
#endif
            //*** Derive MIP candidates using Hadamard
            if (testMip && !supportedMipBlkSize)
            {
              // avoid estimation for unsupported blk sizes
              const int transpOff    = getNumModesMip(pu.Y());
              const int numModesFull = (transpOff << 1);
              for (uint32_t uiModeFull = 0; uiModeFull < numModesFull; uiModeFull++)
              {
                const bool     isTransposed = (uiModeFull >= transpOff ? true : false);
                const uint32_t uiMode       = (isTransposed ? uiModeFull - transpOff : uiModeFull);

                numModesForFullRD++;
                uiRdModeList.push_back(ModeInfo(true, isTransposed, 0, NOT_INTRA_SUBPARTITIONS, uiMode));
                CandCostList.push_back(0);
              }
            }
            else if (testMip)
            {
#if JVET_V0130_INTRA_TMP
              cu.tmpFlag = 0;
#endif
              cu.mipFlag     = true;
              pu.multiRefIdx = 0;

              double mipHadCost[MAX_NUM_MIP_MODE] = { MAX_DOUBLE };

              initIntraPatternChType(cu, pu.Y());
              initIntraMip(pu, pu.Y());

              const int transpOff    = getNumModesMip(pu.Y());
              const int numModesFull = (transpOff << 1);
              for (uint32_t uiModeFull = 0; uiModeFull < numModesFull; uiModeFull++)
              {
                const bool     isTransposed = (uiModeFull >= transpOff ? true : false);
                const uint32_t uiMode       = (isTransposed ? uiModeFull - transpOff : uiModeFull);

                pu.mipTransposedFlag           = isTransposed;
                pu.intraDir[CHANNEL_TYPE_LUMA] = uiMode;
                predIntraMip(COMPONENT_Y, piPred, pu);

                // Use the min between SAD and HAD as the cost criterion
                // SAD is scaled by 2 to align with the scaling of HAD
                Distortion minSadHad =
                  std::min(distParamSad.distFunc(distParamSad) * 2, distParamHad.distFunc(distParamHad));

                m_CABACEstimator->getCtx() = SubCtx(Ctx::MipFlag, ctxStartMipFlag);

                uint64_t fracModeBits = xFracModeBitsIntra(pu, uiMode, CHANNEL_TYPE_LUMA);

                double cost            = double(minSadHad) + double(fracModeBits) * sqrtLambdaForFirstPass;
                mipHadCost[uiModeFull] = cost;
                DTRACE(g_trace_ctx, D_INTRA_COST, "IntraMIP: %u, %llu, %f (%d)\n", minSadHad, fracModeBits, cost,
                       uiModeFull);

                updateCandList(ModeInfo(true, isTransposed, 0, NOT_INTRA_SUBPARTITIONS, uiMode), cost, uiRdModeList,
                               CandCostList, numModesForFullRD + 1);
                updateCandList(ModeInfo(true, isTransposed, 0, NOT_INTRA_SUBPARTITIONS, uiMode),
                               0.8 * double(minSadHad), uiHadModeList, CandHadList, numHadCand);
              }

              const double thresholdHadCost = 1.0 + 1.4 / sqrt((double) (pu.lwidth() * pu.lheight()));
              reduceHadCandList(uiRdModeList, CandCostList, numModesForFullRD, thresholdHadCost, mipHadCost, pu,
                                fastMip);
            }
            if (sps.getUseMIP() && LFNSTSaveFlag)
            {
              // save found best modes
              m_uiSavedNumRdModesLFNST = numModesForFullRD;
              m_uiSavedRdModeListLFNST = uiRdModeList;
              m_dSavedModeCostLFNST    = CandCostList;
              // PBINTRA fast
              m_uiSavedHadModeListLFNST = uiHadModeList;
              m_dSavedHadListLFNST      = CandHadList;
              LFNSTSaveFlag             = false;
            }
          }
          else   // if( sps.getUseMIP() && LFNSTLoadFlag)
          {
            // restore saved modes
            numModesForFullRD = m_uiSavedNumRdModesLFNST;
            uiRdModeList      = m_uiSavedRdModeListLFNST;
            CandCostList      = m_dSavedModeCostLFNST;
            // PBINTRA fast
            uiHadModeList = m_uiSavedHadModeListLFNST;
            CandHadList   = m_dSavedHadListLFNST;
          }

          if (m_pcEncCfg->getFastUDIUseMPMEnabled())
          {

#if SECONDARY_MPM
            auto uiPreds = m_mpmList;
#else
            const int numMPMs = NUM_MOST_PROBABLE_MODES;
            unsigned  uiPreds[numMPMs];
#endif

            pu.multiRefIdx = 0;

#if SECONDARY_MPM
            int numCand = m_mpmListSize;
            numCand = (numCand > 2) ? 2 : numCand;
#else
            const int numCand = PU::getIntraMPMs(pu, uiPreds);
#endif

            for (int j = 0; j < numCand; j++)
            {
              bool     mostProbableModeIncluded = false;
              ModeInfo mostProbableMode( false, false, 0, NOT_INTRA_SUBPARTITIONS, uiPreds[j] );

              for (int i = 0; i < numModesForFullRD; i++)
              {
                mostProbableModeIncluded |= (mostProbableMode == uiRdModeList[i]);
              }
              if (!mostProbableModeIncluded)
              {
                numModesForFullRD++;
                uiRdModeList.push_back(mostProbableMode);
                CandCostList.push_back(0);
              }
            }
            if (saveDataForISP)
            {
              // we add the MPMs to the list that contains only regular intra modes
              for (int j = 0; j < numCand; j++)
              {
                bool     mostProbableModeIncluded = false;
                ModeInfo mostProbableMode(false, false, 0, NOT_INTRA_SUBPARTITIONS, uiPreds[j]);

                for (int i = 0; i < m_ispCandListHor.size(); i++)
                {
                  mostProbableModeIncluded |= (mostProbableMode == m_ispCandListHor[i]);
                }
                if (!mostProbableModeIncluded)
                {
                  m_ispCandListHor.push_back(mostProbableMode);
                }
              }
            }
          }
        }
        else
        {
          THROW("Full search not supported for MIP");
        }
        if (sps.getUseLFNST() && mtsUsageFlag == 1)
        {
          // Store the modes to be checked with RD
          m_savedNumRdModes[lfnstIdx] = numModesForFullRD;
          std::copy_n(uiRdModeList.begin(), numModesForFullRD, m_savedRdModeList[lfnstIdx]);
        }
      }
      else   // mtsUsage = 2 (here we potentially reduce the number of modes that will be full-RD checked)
      {
        if ((m_pcEncCfg->getUseFastLFNST() || !cu.slice->isIntra()) && m_bestModeCostValid[lfnstIdx])
        {
          numModesForFullRD = 0;
#if JVET_W0103_INTRA_MTS
          double thresholdSkipMode = 1.0 + ((cu.lfnstIdx > 0) ? 0.1 : 0.8) * (1.4 / sqrt((double)(width * height)));
          std::vector<std::pair<ModeInfo, double>> ModeInfoWithDCT2Cost(m_savedNumRdModes[0]);
          for (int i = 0; i < m_savedNumRdModes[0]; i++)
          {
            ModeInfoWithDCT2Cost[i] = { m_savedRdModeList[0][i], m_modeCostStore[0][i] };
          }
          std::stable_sort(ModeInfoWithDCT2Cost.begin(), ModeInfoWithDCT2Cost.end(), [](const std::pair<ModeInfo, double> & l, const std::pair<ModeInfo, double> & r) {return l.second < r.second; });

          // **Reorder the modes** and Skip checking the modes with much larger R-D cost than the best mode
          for (int i = 0; i < m_savedNumRdModes[0]; i++)
          {
            if (ModeInfoWithDCT2Cost[i].second <= thresholdSkipMode * ModeInfoWithDCT2Cost[0].second)
            {
              uiRdModeList.push_back(ModeInfoWithDCT2Cost[i].first);
              numModesForFullRD++;
            }
          }
#else
          double thresholdSkipMode = 1.0 + ((cu.lfnstIdx > 0) ? 0.1 : 1.0) * (1.4 / sqrt((double) (width * height)));

          // Skip checking the modes with much larger R-D cost than the best mode
          for (int i = 0; i < m_savedNumRdModes[lfnstIdx]; i++)
          {
            if (m_modeCostStore[lfnstIdx][i] <= thresholdSkipMode * m_bestModeCostStore[lfnstIdx])
            {
              uiRdModeList.push_back(m_savedRdModeList[lfnstIdx][i]);
              numModesForFullRD++;
            }
          }
#endif
        }
        else   // this is necessary because we skip the candidates list calculation, since it was already obtained for
               // the DCT-II. Now we load it
        {
          // Restore the modes to be checked with RD
          numModesForFullRD = m_savedNumRdModes[lfnstIdx];
          uiRdModeList.resize(numModesForFullRD);
          std::copy_n(m_savedRdModeList[lfnstIdx], m_savedNumRdModes[lfnstIdx], uiRdModeList.begin());
          CandCostList.resize(numModesForFullRD);
        }
      }
#if ENABLE_DIMD
      bool isDimdValid = cu.slice->getSPS()->getUseDimd();
      if (isDimdValid)
      {
        cu.dimd = false;
        ModeInfo m = ModeInfo( false, false, 0, NOT_INTRA_SUBPARTITIONS, DIMD_IDX );
        uiRdModeList.push_back(m);
#if !JVET_V0087_DIMD_NO_ISP
        if (testISP)
        {
          m.ispMod = HOR_INTRA_SUBPARTITIONS;
          m_ispCandListHor.push_back(m);
          m.ispMod = VER_INTRA_SUBPARTITIONS;
          m_ispCandListVer.push_back(m);
        }
#endif
      }
#else
      CHECK(numModesForFullRD != uiRdModeList.size(), "Inconsistent state!");
#endif
      // after this point, don't use numModesForFullRD
      // PBINTRA fast
      if (m_pcEncCfg->getUsePbIntraFast() && !cs.slice->isIntra() && uiRdModeList.size() < numModesAvailable
          && !cs.slice->getDisableSATDForRD() && (mtsUsageFlag != 2 || lfnstIdx > 0))
      {
        double   pbintraRatio = (lfnstIdx > 0) ? 1.25 : PBINTRA_RATIO;
        int      maxSize      = -1;
        ModeInfo bestMipMode;
        int      bestMipIdx = -1;
        for (int idx = 0; idx < uiRdModeList.size(); idx++)
        {
          if (uiRdModeList[idx].mipFlg)
          {
            bestMipMode = uiRdModeList[idx];
            bestMipIdx  = idx;
            break;
          }
        }
        const int numHadCand = 3;
        for (int k = numHadCand - 1; k >= 0; k--)
        {
          if (CandHadList.size() < (k + 1) || CandHadList[k] > cs.interHad * pbintraRatio)
          {
            maxSize = k;
          }
        }
        if (maxSize > 0)
        {
          uiRdModeList.resize(std::min<size_t>(uiRdModeList.size(), maxSize));

          if (sps.getUseLFNST() && mtsUsageFlag == 1)
          {
            // Update also the number of stored modes to avoid partial fill of mode storage
            m_savedNumRdModes[lfnstIdx] = std::min<int32_t>(int32_t(uiRdModeList.size()), m_savedNumRdModes[lfnstIdx]);
          }

          if (bestMipIdx >= 0)
          {
            if (uiRdModeList.size() <= bestMipIdx)
            {
              uiRdModeList.push_back(bestMipMode);
            }
          }
          if (saveDataForISP)
          {
            m_ispCandListHor.resize(std::min<size_t>(m_ispCandListHor.size(), maxSize));
          }
        }
        if (maxSize == 0)
        {
          cs.dist     = std::numeric_limits<Distortion>::max();
          cs.interHad = 0;

          //===== reset context models =====
#if JVET_V0130_INTRA_TMP
          m_CABACEstimator->getCtx() = SubCtx( Ctx::TmpFlag, ctxStartTpmFlag );
#endif
          m_CABACEstimator->getCtx() = SubCtx(Ctx::MipFlag, ctxStartMipFlag);
#if JVET_W0123_TIMD_FUSION
          m_CABACEstimator->getCtx() = SubCtx( Ctx::TimdFlag, ctxStartTimdFlag );
#endif
          m_CABACEstimator->getCtx() = SubCtx(Ctx::ISPMode, ctxStartIspMode);
#if SECONDARY_MPM
          m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaMPMIdx, ctxStartMPMIdxFlag);
#endif
          m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaPlanarFlag, ctxStartPlanarFlag);
          m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaMpmFlag, ctxStartIntraMode);
#if SECONDARY_MPM
          m_CABACEstimator->getCtx() = SubCtx(Ctx::IntraLumaSecondMpmFlag, ctxStartIntraMode2);
#endif
          m_CABACEstimator->getCtx() = SubCtx(Ctx::MultiRefLineIdx, ctxStartMrlIdx);

          return false;
        }
      }
    }
#if JVET_Y0142_ADAPT_INTRA_MTS
    if (sps.getUseLFNST() && m_modesForMTS.size() == 0 && cu.mtsFlag)
    {
      return false;
    }
#endif
    int numNonISPModes = (int)uiRdModeList.size();
#if JVET_W0123_TIMD_FUSION
    bool isTimdValid = cu.slice->getSPS()->getUseTimd();
    if (cu.lwidth() * cu.lheight() > 1024 && cu.slice->getSliceType() == I_SLICE)
    {
      isTimdValid = false;
    }
    if (isTimdValid)
    {
      cu.timd = false;
      uiRdModeList.push_back( ModeInfo( false, false, 0, NOT_INTRA_SUBPARTITIONS, TIMD_IDX ) );
      numNonISPModes++;
      if (lfnstIdx == 0 && !cu.mtsFlag)
      {
        bool isFirstLineOfCtu     = (((pu.block(COMPONENT_Y).y) & ((pu.cs->sps)->getMaxCUWidth() - 1)) == 0);
#if JVET_Y0116_EXTENDED_MRL_LIST
        int  numOfPassesExtendRef = 3;
        if (!sps.getUseMRL() || isFirstLineOfCtu) 
        {
          numOfPassesExtendRef = 1;
        }
        else
        {
          bool checkLineOutsideCtu[2];
          for (int mrlIdx = 1; mrlIdx < 3; mrlIdx++)
          {
            bool isLineOutsideCtu =
              ((cu.block(COMPONENT_Y).y) % ((cu.cs->sps)->getMaxCUWidth()) <= MULTI_REF_LINE_IDX[mrlIdx]) ? true
                                                                                                          : false;
            checkLineOutsideCtu[mrlIdx-1] = isLineOutsideCtu;
          }
          if (checkLineOutsideCtu[0]) 
          {
            numOfPassesExtendRef = 1;
          }
          else
          {
            if (checkLineOutsideCtu[1] && !checkLineOutsideCtu[0])
            {
              numOfPassesExtendRef = 2;
            }
          }
        }
#else
        int  numOfPassesExtendRef = ((!sps.getUseMRL() || isFirstLineOfCtu) ? 1 : MRL_NUM_REF_LINES);
#endif
        for (int mRefNum = 1; mRefNum < numOfPassesExtendRef; mRefNum++)
        {
          int multiRefIdx = MULTI_REF_LINE_IDX[mRefNum];
          uiRdModeList.push_back( ModeInfo( false, false, multiRefIdx, NOT_INTRA_SUBPARTITIONS, TIMD_IDX ) );
          numNonISPModes++;
        }
      }
    }
#endif

    if ( testISP )
    {
      // we reserve positions for ISP in the common full RD list
      const int maxNumRDModesISP = sps.getUseLFNST() ? 16 * NUM_LFNST_NUM_PER_SET : 16;
      m_curIspLfnstIdx = 0;
      for (int i = 0; i < maxNumRDModesISP; i++)
      {
        uiRdModeList.push_back( ModeInfo( false, false, 0, INTRA_SUBPARTITIONS_RESERVED, 0 ) );
      }
    }
#if JVET_W0123_TIMD_FUSION
    if (isTimdValid && sps.getUseISP() && CU::canUseISP(width, height, cu.cs->sps->getMaxTbSize()) && lfnstIdx == 0 && !cu.mtsFlag)
    {
      uiRdModeList.push_back( ModeInfo( false, false, 0, HOR_INTRA_SUBPARTITIONS, TIMD_IDX ) );
      uiRdModeList.push_back( ModeInfo( false, false, 0, VER_INTRA_SUBPARTITIONS, TIMD_IDX ) );
    }
#endif
    //===== check modes (using r-d costs) =====
    ModeInfo       uiBestPUMode;
    int            bestBDPCMMode = 0;
    double         bestCostNonBDPCM = MAX_DOUBLE;
#if INTRA_TRANS_ENC_OPT
    double         bestISPCostTested = MAX_DOUBLE;
    ISPType        bestISPModeTested = NOT_INTRA_SUBPARTITIONS;
#endif
    CodingStructure *csTemp = m_pTempCS[gp_sizeIdxInfo->idxFrom( cu.lwidth() )][gp_sizeIdxInfo->idxFrom( cu.lheight() )];
    CodingStructure *csBest = m_pBestCS[gp_sizeIdxInfo->idxFrom( cu.lwidth() )][gp_sizeIdxInfo->idxFrom( cu.lheight() )];

    csTemp->slice = cs.slice;
    csBest->slice = cs.slice;
    csTemp->initStructData();
    csBest->initStructData();
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    csTemp->picture = cs.picture;
    csBest->picture = cs.picture;
#endif
    // just to be sure
    numModesForFullRD = ( int ) uiRdModeList.size();
    TUIntraSubPartitioner subTuPartitioner( partitioner );
    if ( testISP )
    {
      m_modeCtrl->setIspCost( MAX_DOUBLE );
      m_modeCtrl->setMtsFirstPassNoIspCost( MAX_DOUBLE );
    }
    int bestLfnstIdx = cu.lfnstIdx;

    for (int mode = isSecondColorSpace ? 0 : -2 * int(testBDPCM); mode < (int)uiRdModeList.size(); mode++)
    {
      // set CU/PU to luma prediction mode
      ModeInfo uiOrgMode;
      if (sps.getUseColorTrans() && !m_pcEncCfg->getRGBFormatFlag() && isSecondColorSpace && mode)
      {
        continue;
      }

      if (mode < 0 || (isSecondColorSpace && m_savedBDPCMModeFirstColorSpace[m_savedRdModeIdx][mode]))
      {
        cu.bdpcmMode = mode < 0 ? -mode : m_savedBDPCMModeFirstColorSpace[m_savedRdModeIdx][mode];
        uiOrgMode = ModeInfo( false, false, 0, NOT_INTRA_SUBPARTITIONS, cu.bdpcmMode == 2 ? VER_IDX : HOR_IDX );
      }
      else
      {
        cu.bdpcmMode = 0;
        uiOrgMode = uiRdModeList[mode];
      }

      if (!cu.bdpcmMode && uiRdModeList[mode].ispMod == INTRA_SUBPARTITIONS_RESERVED)
      {
        if (mode == numNonISPModes)   // the list needs to be sorted only once
        {
          if (m_pcEncCfg->getUseFastISP())
          {
#if JVET_W0123_TIMD_FUSION
            if (bestTimdMode)
            {
              m_modeCtrl->setBestPredModeDCT2(MAP131TO67(uiBestPUMode.modeId));
            }
            else
            {
              m_modeCtrl->setBestPredModeDCT2(uiBestPUMode.modeId);
            }
#else
            m_modeCtrl->setBestPredModeDCT2(uiBestPUMode.modeId);
#endif
          }
#if JVET_W0123_TIMD_FUSION
          ModeInfo tempBestPUMode = uiBestPUMode;
          if (bestTimdMode)
          {
            tempBestPUMode.modeId = MAP131TO67(tempBestPUMode.modeId);
          }
          if (!xSortISPCandList(bestCurrentCost, csBest->cost, tempBestPUMode))
#else
          if (!xSortISPCandList(bestCurrentCost, csBest->cost, uiBestPUMode))
#endif
          {
            break;
          }
        }
        xGetNextISPMode(uiRdModeList[mode], (mode > 0 ? &uiRdModeList[mode - 1] : nullptr), Size(width, height));
        if (uiRdModeList[mode].ispMod == INTRA_SUBPARTITIONS_RESERVED)
        {
          continue;
        }
        cu.lfnstIdx = m_curIspLfnstIdx;
        uiOrgMode   = uiRdModeList[mode];
      }
#if ENABLE_DIMD && INTRA_TRANS_ENC_OPT
      if ((m_pcEncCfg->getIntraPeriod() == 1) && cu.slice->getSPS()->getUseDimd() && mode >= 0 && !cu.dimdBlending && uiOrgMode.ispMod == 0 && uiOrgMode.mRefId == 0 && uiOrgMode.modeId != TIMD_IDX && uiOrgMode.modeId != DIMD_IDX)
      {
        bool modeDuplicated = (uiOrgMode.modeId == cu.dimdMode);
        if (modeDuplicated)
        {
          m_modeCostStore[lfnstIdx][mode] = MAX_DOUBLE / 2.0;
          continue;
        }
      }
#endif	  
#if ENABLE_DIMD
      cu.dimd = false;
      if( mode >= 0 && uiOrgMode.modeId == DIMD_IDX ) /*to check*/
      {
        uiOrgMode.modeId = cu.dimdMode;
        cu.dimd = true;
      }
#endif
#if JVET_V0130_INTRA_TMP
      cu.tmpFlag = uiOrgMode.tmpFlag;
#if JVET_W0103_INTRA_MTS
      if (cu.tmpFlag && cu.mtsFlag) continue;
#endif
#endif
      cu.mipFlag                     = uiOrgMode.mipFlg;
      pu.mipTransposedFlag           = uiOrgMode.mipTrFlg;
      cu.ispMode                     = uiOrgMode.ispMod;
      pu.multiRefIdx                 = uiOrgMode.mRefId;
      pu.intraDir[CHANNEL_TYPE_LUMA] = uiOrgMode.modeId;
#if JVET_W0123_TIMD_FUSION
      cu.timd = false;
      if (mode >= 0 && uiOrgMode.modeId == TIMD_IDX)
      {
        if (cu.ispMode)
        {
          cu.lfnstIdx = lfnstIdx;
#if INTRA_TRANS_ENC_OPT
          if ((m_pcEncCfg->getIntraPeriod() == 1) && ((bestISPModeTested == HOR_INTRA_SUBPARTITIONS) || (bestISPModeTested == VER_INTRA_SUBPARTITIONS)))
          {
            if (cu.ispMode != bestISPModeTested)
            {
              continue;
            }
          }
#endif
          if (cu.ispMode == VER_INTRA_SUBPARTITIONS && uiBestPUMode.ispMod == 0 && !bestTimdMode)
          {
            continue;
          }
        }
#if INTRA_TRANS_ENC_OPT
        else if (m_skipTimdLfnstMtsPass)
        {
          CHECK(!cu.lfnstIdx && !cu.mtsFlag, "invalid logic");
          continue;
        }
#endif
        uiOrgMode.modeId = cu.timdMode;
        pu.intraDir[CHANNEL_TYPE_LUMA] = uiOrgMode.modeId;
        cu.timd = true;
      }
#endif

      CHECK(cu.mipFlag && pu.multiRefIdx, "Error: combination of MIP and MRL not supported");
#if JVET_W0123_TIMD_FUSION
      if (!cu.timd)
      {
#endif
        CHECK(pu.multiRefIdx && (pu.intraDir[0] == PLANAR_IDX),
              "Error: combination of MRL and Planar mode not supported");
#if JVET_W0123_TIMD_FUSION
      }
#endif
      CHECK(cu.ispMode && cu.mipFlag, "Error: combination of ISP and MIP not supported");
      CHECK(cu.ispMode && pu.multiRefIdx, "Error: combination of ISP and MRL not supported");
      CHECK(cu.ispMode&& cu.colorTransform, "Error: combination of ISP and ACT not supported");
#if JVET_V0130_INTRA_TMP
      CHECK( cu.mipFlag && cu.tmpFlag, "Error: combination of MIP and TPM not supported" );
      CHECK( cu.tmpFlag && cu.ispMode, "Error: combination of TPM and ISP not supported" );
      CHECK( cu.tmpFlag && pu.multiRefIdx, "Error: combination of TPM and MRL not supported" );
#endif
#if ENABLE_DIMD && JVET_V0087_DIMD_NO_ISP
      CHECK(cu.ispMode && cu.dimd, "Error: combination of ISP and DIMD not supported");
#endif
      pu.intraDir[CHANNEL_TYPE_CHROMA] = cu.colorTransform ? DM_CHROMA_IDX : pu.intraDir[CHANNEL_TYPE_CHROMA];
#if JVET_Y0142_ADAPT_INTRA_MTS
      if (cu.mtsFlag)
      {
        int mtsModeIdx = -1;
        for (int i = 0; i < m_modesForMTS.size(); i++)
        {
          if (uiOrgMode == m_modesForMTS[i])
          {
            mtsModeIdx = i;
            break;
          }
        }
        if (mtsModeIdx == -1)
        {
          mtsModeIdx = 0;
        }
        CHECK(mtsModeIdx == -1, "mtsModeIdx==-1");
        m_coeffAbsSumDCT2 = (m_modesForMTS.size() == 0) ? 10 : m_modesCoeffAbsSumDCT2[mtsModeIdx];
      }
#endif
      // set context models
      m_CABACEstimator->getCtx() = ctxStart;

      // determine residual for partition
      cs.initSubStructure( *csTemp, partitioner.chType, cs.area, true );

      bool tmpValidReturn = false;
      if( cu.ispMode )
      {
        if ( m_pcEncCfg->getUseFastISP() )
        {
          m_modeCtrl->setISPWasTested(true);
        }
        tmpValidReturn = xIntraCodingLumaISP(*csTemp, subTuPartitioner, bestCurrentCost);
        if (csTemp->tus.size() == 0)
        {
          // no TUs were coded
          csTemp->cost = MAX_DOUBLE;
          continue;
        }
        // we save the data for future tests
#if JVET_W0123_TIMD_FUSION
        if (!cu.timd)
        {
#endif
        m_ispTestedModes[m_curIspLfnstIdx].setModeResults((ISPType)cu.ispMode, (int)uiOrgMode.modeId, (int)csTemp->tus.size(), csTemp->cus[0]->firstTU->cbf[COMPONENT_Y] ? csTemp->cost : MAX_DOUBLE, csBest->cost);
#if JVET_W0123_TIMD_FUSION
        }
#endif
        csTemp->cost = !tmpValidReturn ? MAX_DOUBLE : csTemp->cost;
#if INTRA_TRANS_ENC_OPT
        if (csTemp->cost < bestISPCostTested)
        {
          bestISPCostTested = csTemp->cost;
          bestISPModeTested = (ISPType)cu.ispMode;
        }
#endif
      }
      else
      {
        if (cu.colorTransform)
        {
          tmpValidReturn = xRecurIntraCodingACTQT(*csTemp, partitioner, mtsCheckRangeFlag, mtsFirstCheckId, mtsLastCheckId, moreProbMTSIdxFirst);
        }
        else
        {
          tmpValidReturn = xRecurIntraCodingLumaQT(
            *csTemp, partitioner, uiBestPUMode.ispMod ? bestCurrentCost : MAX_DOUBLE, -1, TU_NO_ISP,
            uiBestPUMode.ispMod, mtsCheckRangeFlag, mtsFirstCheckId, mtsLastCheckId, moreProbMTSIdxFirst);
        }
      }
#if JVET_Y0142_ADAPT_INTRA_MTS
#if JVET_W0123_TIMD_FUSION
      if (!cu.mtsFlag && !lfnstIdx && mode < numNonISPModes && !(cu.timd && pu.multiRefIdx))
#else
      if( !cu.mtsFlag && !lfnstIdx && mode < numNonISPModes && !pu.multiRefIdx )
#endif
      {
        m_modesForMTS.push_back(uiOrgMode);
        m_modesCoeffAbsSumDCT2.push_back(m_coeffAbsSumDCT2);
      }
#endif
#if JVET_V0130_INTRA_TMP
#if JVET_W0123_TIMD_FUSION
      if (!cu.ispMode && !cu.mtsFlag && !cu.lfnstIdx && !cu.bdpcmMode && !pu.multiRefIdx && !cu.mipFlag && !cu.tmpFlag && testISP && !cu.timd)
#else
      if( !cu.ispMode && !cu.mtsFlag && !cu.lfnstIdx && !cu.bdpcmMode && !pu.multiRefIdx && !cu.mipFlag && !cu.tmpFlag && testISP )
#endif
#else
#if JVET_W0123_TIMD_FUSION
      if (!cu.ispMode && !cu.mtsFlag && !cu.lfnstIdx && !cu.bdpcmMode && !pu.multiRefIdx && !cu.mipFlag && testISP && !cu.timd)
#else
      if (!cu.ispMode && !cu.mtsFlag && !cu.lfnstIdx && !cu.bdpcmMode && !pu.multiRefIdx && !cu.mipFlag && testISP)
#endif
#endif
      {
#if JVET_V0130_INTRA_TMP
        m_regIntraRDListWithCosts.push_back( ModeInfoWithCost( cu.mipFlag, pu.mipTransposedFlag, pu.multiRefIdx, cu.ispMode, uiOrgMode.modeId, cu.tmpFlag, csTemp->cost ) );
#else
        m_regIntraRDListWithCosts.push_back( ModeInfoWithCost( cu.mipFlag, pu.mipTransposedFlag, pu.multiRefIdx, cu.ispMode, uiOrgMode.modeId, csTemp->cost ) );
#endif
      }

      if( cu.ispMode && !csTemp->cus[0]->firstTU->cbf[COMPONENT_Y] )
      {
        csTemp->cost = MAX_DOUBLE;
        csTemp->costDbOffset = 0;
        tmpValidReturn = false;
      }
      validReturn |= tmpValidReturn;

#if JVET_W0123_TIMD_FUSION
      if( sps.getUseLFNST() && mtsUsageFlag == 1 && !cu.ispMode && mode >= 0 && !cu.timd )
#else
      if( sps.getUseLFNST() && mtsUsageFlag == 1 && !cu.ispMode && mode >= 0 )
#endif
      {
        m_modeCostStore[lfnstIdx][mode] = tmpValidReturn ? csTemp->cost : (MAX_DOUBLE / 2.0); //(MAX_DOUBLE / 2.0) ??
      }
#if JVET_V0130_INTRA_TMP
      DTRACE( g_trace_ctx, D_INTRA_COST, "IntraCost T [x=%d,y=%d,w=%d,h=%d] %f (%d,%d,%d,%d,%d,%d,%d) \n", cu.blocks[0].x,
              cu.blocks[0].y, ( int ) width, ( int ) height, csTemp->cost, uiOrgMode.modeId, uiOrgMode.ispMod,
              pu.multiRefIdx, cu.tmpFlag, cu.mipFlag, cu.lfnstIdx, cu.mtsFlag );
#else
      DTRACE(g_trace_ctx, D_INTRA_COST, "IntraCost T [x=%d,y=%d,w=%d,h=%d] %f (%d,%d,%d,%d,%d,%d) \n", cu.blocks[0].x,
             cu.blocks[0].y, (int) width, (int) height, csTemp->cost, uiOrgMode.modeId, uiOrgMode.ispMod,
             pu.multiRefIdx, cu.mipFlag, cu.lfnstIdx, cu.mtsFlag);
#endif

      if( tmpValidReturn )
      {
        if (isFirstColorSpace)
        {
          if (m_pcEncCfg->getRGBFormatFlag() || !cu.ispMode)
          {
            sortRdModeListFirstColorSpace(uiOrgMode, csTemp->cost, cu.bdpcmMode, m_savedRdModeFirstColorSpace[m_savedRdModeIdx], m_savedRdCostFirstColorSpace[m_savedRdModeIdx], m_savedBDPCMModeFirstColorSpace[m_savedRdModeIdx], m_numSavedRdModeFirstColorSpace[m_savedRdModeIdx]);
          }
        }
#if INTRA_TRANS_ENC_OPT
        if (setSkipTimdControl && !cu.ispMode)
        {
#if JVET_W0123_TIMD_FUSION || ENABLE_DIMD
#if JVET_W0123_TIMD_FUSION && ENABLE_DIMD
          if (!cu.dimd && !cu.timd)
#elif ENABLE_DIMD
          if( !cu.dimd )
#else
          if( !cu.timd )
#endif
          {
            if (csTemp->cost < regAngCost)
            {
              regAngCost = csTemp->cost;
            }
          }
#endif
#if JVET_W0123_TIMD_FUSION
          if (cu.timd)
          {
            if (csTemp->cost < timdAngCost)
            {
              timdAngCost = csTemp->cost;
            }
          }
#endif
        }
#endif
        // check r-d cost
        if( csTemp->cost < csBest->cost )
        {
          std::swap( csTemp, csBest );

          uiBestPUMode  = uiOrgMode;
          bestBDPCMMode = cu.bdpcmMode;
#if ENABLE_DIMD
          bestDimdMode = cu.dimd;
#endif
#if JVET_W0123_TIMD_FUSION
          bestTimdMode = cu.timd;
#endif
          if( sps.getUseLFNST() && mtsUsageFlag == 1 && !cu.ispMode )
          {
            m_bestModeCostStore[ lfnstIdx ] = csBest->cost; //cs.cost;
            m_bestModeCostValid[ lfnstIdx ] = true;
          }
#if JVET_W0103_INTRA_MTS
          if (sps.getUseLFNST() && m_globalBestCostStore > csBest->cost)
          {
            m_globalBestCostStore = csBest->cost;
            m_globalBestCostValid = true;
          }
#endif
          if( csBest->cost < bestCurrentCost )
          {
            bestCurrentCost = csBest->cost;
          }
          if ( cu.ispMode )
          {
            m_modeCtrl->setIspCost(csBest->cost);
            bestLfnstIdx = cu.lfnstIdx;
          }
          else if ( testISP )
          {
            m_modeCtrl->setMtsFirstPassNoIspCost(csBest->cost);
          }
        }
        if( !cu.ispMode && !cu.bdpcmMode && csBest->cost < bestCostNonBDPCM )
        {
          bestCostNonBDPCM = csBest->cost;
        }
      }

      csTemp->releaseIntermediateData();
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if( m_pcEncCfg->getFastLocalDualTreeMode() )
      {
        if( cu.isConsIntra() && !cu.slice->isIntra() && csBest->cost != MAX_DOUBLE && costInterCU != COST_UNKNOWN && mode >= 0 )
        {
          if( m_pcEncCfg->getFastLocalDualTreeMode() == 2 )
          {
            //Note: only try one intra mode, which is especially useful to reduce EncT for LDB case (around 4%)
            break;
          }
          else
          {
            if( csBest->cost > costInterCU * 1.5 )
            {
              break;
            }
          }
        }
      }
      if (sps.getUseColorTrans() && !CS::isDualITree(cs))
      {
        if ((m_pcEncCfg->getRGBFormatFlag() && !cu.colorTransform) && csBest->cost != MAX_DOUBLE && bestCS->cost != MAX_DOUBLE && mode >= 0)
        {
          if (csBest->cost > bestCS->cost)
          {
            break;
          }
        }
      }
#endif
    } // Mode loop
#if INTRA_TRANS_ENC_OPT
    if (setSkipTimdControl && regAngCost != MAX_DOUBLE && timdAngCost != MAX_DOUBLE)
    {
      if (regAngCost * 1.3 < timdAngCost)
      {
        m_skipTimdLfnstMtsPass = true;
      }
    }
#endif
    cu.ispMode = uiBestPUMode.ispMod;
    cu.lfnstIdx = bestLfnstIdx;

    if( validReturn )
    {
      if (cu.colorTransform)
      {
        cs.useSubStructure(*csBest, partitioner.chType, pu, true, true, KEEP_PRED_AND_RESI_SIGNALS, KEEP_PRED_AND_RESI_SIGNALS, true);
      }
      else
      {
        cs.useSubStructure(*csBest, partitioner.chType, pu.singleChan(CHANNEL_TYPE_LUMA), true, true, KEEP_PRED_AND_RESI_SIGNALS,
                           KEEP_PRED_AND_RESI_SIGNALS, true);
      }
    }
#if JVET_AB0061_ITMP_BV_FOR_IBC
    if (uiBestPUMode.tmpFlag)
    {
      pu.interDir               = 1;             // use list 0 for IBC mode
      pu.refIdx[REF_PIC_LIST_0] = MAX_NUM_REF;   // last idx in the list
      pu.mv[0]                  = csBest->pus[0]->mv[0];
      pu.bv                     = csBest->pus[0]->bv;
    }
#endif
    csBest->releaseIntermediateData();
    if( validReturn )
    {
      //=== update PU data ====
#if JVET_V0130_INTRA_TMP
      cu.tmpFlag = uiBestPUMode.tmpFlag;
#endif
      cu.mipFlag = uiBestPUMode.mipFlg;
      pu.mipTransposedFlag             = uiBestPUMode.mipTrFlg;
      pu.multiRefIdx = uiBestPUMode.mRefId;
      pu.intraDir[ CHANNEL_TYPE_LUMA ] = uiBestPUMode.modeId;
#if ENABLE_DIMD
      cu.dimd = bestDimdMode;
      if (cu.dimd)
      {
        CHECK(pu.multiRefIdx > 0, "use of DIMD");
      }
#endif
      cu.bdpcmMode = bestBDPCMMode;
#if JVET_W0123_TIMD_FUSION
      cu.timd = bestTimdMode;
      if (cu.timd)
      {
        pu.intraDir[ CHANNEL_TYPE_LUMA ] = cu.timdMode;
      }
#endif
      if (cu.colorTransform)
      {
        CHECK(pu.intraDir[CHANNEL_TYPE_CHROMA] != DM_CHROMA_IDX, "chroma should use DM mode for adaptive color transform");
      }
    }
  }

  //===== reset context models =====
  m_CABACEstimator->getCtx() = ctxStart;

  return validReturn;
}

void IntraSearch::estIntraPredChromaQT( CodingUnit &cu, Partitioner &partitioner, const double maxCostAllowed )
{
  const ChromaFormat format   = cu.chromaFormat;
  const uint32_t    numberValidComponents = getNumberValidComponents(format);
  CodingStructure &cs = *cu.cs;
  const TempCtx ctxStart  ( m_CtxCache, m_CABACEstimator->getCtx() );

  cs.setDecomp( cs.area.Cb(), false );

  double    bestCostSoFar = maxCostAllowed;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  bool      lumaUsesISP   = !cu.isSepTree() && cu.ispMode;
#else
  bool      lumaUsesISP = !CS::isDualITree(*cu.cs) && cu.ispMode;
#endif
  PartSplit ispType       = lumaUsesISP ? CU::getISPType( cu, COMPONENT_Y ) : TU_NO_ISP;
  CHECK( cu.ispMode && bestCostSoFar < 0, "bestCostSoFar must be positive!" );

  auto &pu = *cu.firstPU;

  {
    uint32_t       uiBestMode = 0;
    Distortion uiBestDist = 0;
    double     dBestCost = MAX_DOUBLE;
    int32_t bestBDPCMMode = 0;
#if JVET_AA0057_CCCM
    int      cccmModeBest = 0;
#endif
#if JVET_Z0050_CCLM_SLOPE
    CclmOffsets bestCclmOffsets = {};
    CclmOffsets satdCclmOffsetsBest[NUM_CHROMA_MODE];
    int64_t     satdCclmCosts      [NUM_CHROMA_MODE] = { 0 };
#endif
#if JVET_AA0126_GLM
    GlmIdc      bestGlmIdc = {};
    GlmIdc      satdGlmIdcBest     [NUM_CHROMA_MODE];
    int64_t     satdGlmCosts       [NUM_CHROMA_MODE] = { 0 };
#endif
#if JVET_Z0050_DIMD_CHROMA_FUSION
    bool isChromaFusion = false;
#endif

    //----- init mode list ----
    {
      int32_t  uiMinMode = 0;
      int32_t  uiMaxMode = NUM_CHROMA_MODE;
      //----- check chroma modes -----
      uint32_t chromaCandModes[ NUM_CHROMA_MODE ];
      PU::getIntraChromaCandModes( pu, chromaCandModes );
#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
      // derive DIMD chroma mode
      CompArea areaCb = pu.Cb();
      CompArea areaCr = pu.Cr();
      CompArea lumaArea = CompArea(COMPONENT_Y, pu.chromaFormat, areaCb.lumaPos(), recalcSize(pu.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, areaCb.size()));//needed for correct pos/size (4x4 Tus)
      IntraPrediction::deriveDimdChromaMode(cs.picture->getRecoBuf(lumaArea), cs.picture->getRecoBuf(areaCb), cs.picture->getRecoBuf(areaCr), lumaArea, areaCb, areaCr, *pu.cu);
#endif

      // create a temporary CS
      CodingStructure &saveCS = *m_pSaveCS[0];
      saveCS.pcv      = cs.pcv;
      saveCS.picture  = cs.picture;
#if JVET_Z0118_GDR
      saveCS.m_pt = cs.m_pt;
#endif
      saveCS.area.repositionTo( cs.area );
      saveCS.clearTUs();
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if( !cu.isSepTree() && cu.ispMode )
#else
      if (!CS::isDualITree(cs) && cu.ispMode)
#endif
      {
        saveCS.clearCUs();
        saveCS.clearPUs();
      }
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if( cu.isSepTree() )
#else
      if (CS::isDualITree(cs))
#endif
      {
        if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
        {
          partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );

          do
          {
            cs.addTU( CS::getArea( cs, partitioner.currArea(), partitioner.chType ), partitioner.chType ).depth = partitioner.currTrDepth;
          } while( partitioner.nextPart( cs ) );

          partitioner.exitCurrSplit();
        }
        else
        cs.addTU( CS::getArea( cs, partitioner.currArea(), partitioner.chType ), partitioner.chType );
      }

      std::vector<TransformUnit*> orgTUs;

      if( lumaUsesISP )
      {
        CodingUnit& auxCU = saveCS.addCU( cu, partitioner.chType );
        auxCU.ispMode = cu.ispMode;
        saveCS.sps = cu.cs->sps;
        saveCS.addPU( *cu.firstPU, partitioner.chType );
      }


      // create a store for the TUs
      for( const auto &ptu : cs.tus )
      {
        // for split TUs in HEVC, add the TUs without Chroma parts for correct setting of Cbfs
        if( lumaUsesISP || pu.contains( *ptu, CHANNEL_TYPE_CHROMA ) )
        {
          saveCS.addTU( *ptu, partitioner.chType );
          orgTUs.push_back( ptu );
        }
      }
      if( lumaUsesISP )
      {
        saveCS.clearCUs();
      }
      // SATD pre-selecting.
      int satdModeList[NUM_CHROMA_MODE];
      int64_t satdSortedCost[NUM_CHROMA_MODE];
      for (int i = 0; i < NUM_CHROMA_MODE; i++)
      {
        satdSortedCost[i] = 0; // for the mode not pre-select by SATD, do RDO by default, so set the initial value 0.
        satdModeList[i] = 0;
      }
#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
      bool modeIsEnable[NUM_INTRA_MODE + 2]; // use intra mode idx to check whether enable
      for (int i = 0; i < NUM_INTRA_MODE + 2; i++)
      {
        modeIsEnable[i] = 1;
      }
#else
      bool modeIsEnable[NUM_INTRA_MODE + 1]; // use intra mode idx to check whether enable
      for (int i = 0; i < NUM_INTRA_MODE + 1; i++)
      {
        modeIsEnable[i] = 1;
      }
#endif
      DistParam distParamSad;
      DistParam distParamSatd;
      pu.intraDir[1] = MDLM_L_IDX; // temporary assigned, just to indicate this is a MDLM mode. for luma down-sampling operation.

      initIntraPatternChType(cu, pu.Cb());
      initIntraPatternChType(cu, pu.Cr());
      xGetLumaRecPixels(pu, pu.Cb());

#if JVET_AA0126_GLM
      if ( PU::isLMCModeEnabled( pu, LM_CHROMA_IDX ) && PU::hasGlmFlag( pu, LM_CHROMA_IDX ) )
      {
        // Generate all GLM templates at encoder
        xGetLumaRecPixelsGlmAll(pu, pu.Cb());
        pu.intraDir[1] = LM_CHROMA_IDX;
        xGetLumaRecPixels(pu, pu.Cb());

        for ( int mode = LM_CHROMA_IDX; mode <= MMLM_T_IDX; mode++ )
        {
          satdGlmIdcBest[mode - LM_CHROMA_IDX].setAllZero();
          
#if JVET_AB0092_GLM_WITH_LUMA
          CodedCUInfo& relatedCU = ((EncModeCtrlMTnoRQT *)m_modeCtrl)->getBlkInfo(partitioner.currArea());
          if (PU::hasGlmFlag(pu, mode) && !relatedCU.skipGLM)
#else
          if ( PU::hasGlmFlag( pu, mode ) )
#endif
          {
#if !JVET_AB0092_GLM_WITH_LUMA
            for ( int comp = COMPONENT_Cb; comp <= COMPONENT_Cr; comp++ )
            {
              ComponentID       compID = ComponentID( comp );
#else
            ComponentID       compID = COMPONENT_Cb;
#endif
              int              idcBest = 0;
              int64_t         satdBest = 0;
              GlmIdc&         idcsBest = satdGlmIdcBest[mode - LM_CHROMA_IDX];
              
              pu.intraDir[1] = mode;
              pu.glmIdc.setAllZero();

              xFindBestGlmIdcSATD(pu, compID, idcBest, satdBest );

              idcsBest.setIdc(compID, 0, idcBest);
              idcsBest.setIdc(compID, 1, idcBest);

#if JVET_AB0092_GLM_WITH_LUMA
              idcsBest.setIdc(COMPONENT_Cr, 0, idcBest);
              idcsBest.setIdc(COMPONENT_Cr, 1, idcBest);
#endif
              
              satdGlmCosts[mode - LM_CHROMA_IDX] += satdBest; // Summing up Cb and Cr cost
#if !JVET_AB0092_GLM_WITH_LUMA
            }
            
            if ( !satdGlmIdcBest[0].isActive() )
            {
              break;
            }
#endif
          }
        }
      }

      pu.glmIdc.setAllZero();
#endif
      
#if JVET_Z0050_CCLM_SLOPE
      if ( PU::isLMCModeEnabled( pu, LM_CHROMA_IDX ) && PU::hasCclmDeltaFlag( pu, LM_CHROMA_IDX ) )
      {
        // Fill luma reference buffer for the two-sided CCLM
        pu.intraDir[1] = LM_CHROMA_IDX;
        xGetLumaRecPixels(pu, pu.Cb());

        for ( int mode = LM_CHROMA_IDX; mode <= MDLM_T_IDX; mode++ )
        {
          satdCclmOffsetsBest[mode - LM_CHROMA_IDX].setAllZero();
          
          if ( PU::hasCclmDeltaFlag( pu, mode ) )
          {
            for ( int comp = COMPONENT_Cb; comp <= COMPONENT_Cr; comp++ )
            {
              ComponentID       compID = ComponentID( comp );
              int            deltaBest = 0;
              int64_t         satdBest = 0;
              CclmOffsets& offsetsBest = satdCclmOffsetsBest[mode - LM_CHROMA_IDX];
              
              pu.intraDir[1] = mode;
              pu.cclmOffsets.setAllZero();

              xFindBestCclmDeltaSlopeSATD(pu, compID, 0, deltaBest, satdBest );

              offsetsBest.setOffset(compID, 0, deltaBest);

#if MMLM
              if ( PU::isMultiModeLM( mode ) )
              {
                // Set best found values for the first model to get a matching second model
                pu.cclmOffsets.setOffsets(offsetsBest.cb0, offsetsBest.cr0, 0, 0);

                xFindBestCclmDeltaSlopeSATD(pu, compID, 1, deltaBest, satdBest );

                offsetsBest.setOffset(compID, 1, deltaBest);
              }
#endif

              satdCclmCosts[mode - LM_CHROMA_IDX] += satdBest; // Summing up Cb and Cr cost
            }
          }
        }
      }

      pu.cclmOffsets.setAllZero();
#endif

#if MMLM
      m_encPreRDRun = true;
#endif
      for (int idx = uiMinMode; idx <= uiMaxMode - 1; idx++)
      {
        int mode = chromaCandModes[idx];
        satdModeList[idx] = mode;
        if (PU::isLMCMode(mode) && !PU::isLMCModeEnabled(pu, mode))
        {
          continue;
        }
        if ((mode == LM_CHROMA_IDX) || (mode == PLANAR_IDX) || (mode == DM_CHROMA_IDX)
#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
          || (mode == DIMD_CHROMA_IDX)
#endif
          ) // only pre-check regular modes and MDLM modes, not including DM, DIMD, Planar, and LM
        {
          continue;
        }
        pu.intraDir[1] = mode; // temporary assigned, for SATD checking.

        int64_t sad = 0;
        int64_t sadCb = 0;
        int64_t satdCb = 0;
        int64_t sadCr = 0;
        int64_t satdCr = 0;
        CodingStructure& cs = *(pu.cs);

        CompArea areaCb = pu.Cb();
        PelBuf orgCb = cs.getOrgBuf(areaCb);
        PelBuf predCb = cs.getPredBuf(areaCb);
        m_pcRdCost->setDistParam(distParamSad, orgCb, predCb, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Cb, false);
        m_pcRdCost->setDistParam(distParamSatd, orgCb, predCb, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Cb, true);
        distParamSad.applyWeight = false;
        distParamSatd.applyWeight = false;
        if (PU::isLMCMode(mode))
        {
          predIntraChromaLM(COMPONENT_Cb, predCb, pu, areaCb, mode);
        }
        else
        {
          initPredIntraParams(pu, pu.Cb(), *pu.cs->sps);
          predIntraAng(COMPONENT_Cb, predCb, pu);
        }
        sadCb = distParamSad.distFunc(distParamSad) * 2;
        satdCb = distParamSatd.distFunc(distParamSatd);
        sad += std::min(sadCb, satdCb);
        CompArea areaCr = pu.Cr();
        PelBuf orgCr = cs.getOrgBuf(areaCr);
        PelBuf predCr = cs.getPredBuf(areaCr);
        m_pcRdCost->setDistParam(distParamSad, orgCr, predCr, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Cr, false);
        m_pcRdCost->setDistParam(distParamSatd, orgCr, predCr, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Cr, true);
        distParamSad.applyWeight = false;
        distParamSatd.applyWeight = false;
        if (PU::isLMCMode(mode))
        {
          predIntraChromaLM(COMPONENT_Cr, predCr, pu, areaCr, mode);
        }
        else
        {
          initPredIntraParams(pu, pu.Cr(), *pu.cs->sps);
          predIntraAng(COMPONENT_Cr, predCr, pu);
        }
        sadCr = distParamSad.distFunc(distParamSad) * 2;
        satdCr = distParamSatd.distFunc(distParamSatd);
        sad += std::min(sadCr, satdCr);
        satdSortedCost[idx] = sad;
      }

#if JVET_AB0143_CCCM_TS
#if MMLM
      uint32_t chromaCandCccmModes[6] = { LM_CHROMA_IDX, MDLM_L_IDX, MDLM_T_IDX, MMLM_CHROMA_IDX, MMLM_L_IDX, MMLM_T_IDX };
#else
      uint32_t chromaCandCccmModes[6] = { LM_CHROMA_IDX, MDLM_L_IDX, MDLM_T_IDX };
#endif
      int64_t satdCccmSortedCost[6];
      int satdCccmModeList[6];
      for (int i = 0; i < 6; i++)
      {
        satdCccmSortedCost[i] = LLONG_MAX; // for the mode not pre-select by SATD, do RDO by default, so set the initial value 0.
        satdCccmModeList[i] = chromaCandCccmModes[i];
      }
      int64_t bestCccmCost = LLONG_MAX;

      bool isCccmFullEnabled = PU::cccmSingleModeAvail(pu, LM_CHROMA_IDX);
      bool isCccmLeftEnabled = PU::isLeftCccmMode(pu, MDLM_L_IDX);
      bool isCccmTopEnabled = PU::isTopCccmMode(pu, MDLM_T_IDX);
#if MMLM
      bool isMultiCccmFullEnabled = PU::cccmMultiModeAvail(pu, MMLM_CHROMA_IDX);
      bool isMultiCccmLeftEnabled = PU::cccmMultiModeAvail(pu, MMLM_L_IDX);
      bool isMultiCccmTopEnabled = PU::cccmMultiModeAvail(pu, MMLM_T_IDX);
#endif

      const UnitArea localUnitArea(cs.area.chromaFormat, Area(0, 0, (pu.Cb().width) << 1, (pu.Cb().height) << 1));
      PelUnitBuf cccmStorage[6];

      pu.cccmFlag = 1;
      xGetLumaRecPixels(pu, pu.Cb());

      bool isCCCMEnabled;
#if MMLM
      for (int idx = 0; idx < 6; idx++)
#else
      for (int idx = 0; idx < 3; idx++)
#endif
      {
        int mode = chromaCandCccmModes[idx];
        if (idx == 0)
        {
          isCCCMEnabled = isCccmFullEnabled;
          pu.cccmFlag = 1;
        }
        else if (idx == 1)
        {
          isCCCMEnabled = isCccmLeftEnabled;
          pu.cccmFlag = 2;
        }
        else if (idx == 2)
        {
          isCCCMEnabled = isCccmTopEnabled;
          pu.cccmFlag = 3;
        }
#if MMLM
        else if (idx == 3)
        {
          isCCCMEnabled = isMultiCccmFullEnabled;
          pu.cccmFlag = 1;
        }
        else if (idx == 4)
        {
          isCCCMEnabled = isMultiCccmLeftEnabled;
          pu.cccmFlag = 2;
        }
        else if (idx == 5)
        {
          isCCCMEnabled = isMultiCccmTopEnabled;
          pu.cccmFlag = 3;
        }
#endif

        if (isCCCMEnabled)
        {
          pu.intraDir[1] = mode; // temporary assigned, for SATD checking.

          int64_t sad = 0;
          int64_t sadCb = 0;
          int64_t satdCb = 0;
          int64_t sadCr = 0;
          int64_t satdCr = 0;
          CodingStructure& cs = *(pu.cs);

          DistParam distParamSadCb;
          DistParam distParamSatdCb;
          DistParam distParamSadCr;
          DistParam distParamSatdCr;

          cccmStorage[idx] = m_cccmStorage[idx].getBuf(localUnitArea);

          CompArea areaCb = pu.Cb();
          PelBuf orgCb = cs.getOrgBuf(areaCb);
          CompArea areaCr = pu.Cr();
          PelBuf orgCr = cs.getOrgBuf(areaCr);

          m_pcRdCost->setDistParam(distParamSadCb, orgCb, cccmStorage[idx].Cb(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Cb, false);
          m_pcRdCost->setDistParam(distParamSatdCb, orgCb, cccmStorage[idx].Cb(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Cb, true);
          distParamSadCb.applyWeight = false;
          distParamSatdCb.applyWeight = false;
          m_pcRdCost->setDistParam(distParamSadCr, orgCr, cccmStorage[idx].Cr(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Cr, false);
          m_pcRdCost->setDistParam(distParamSatdCr, orgCr, cccmStorage[idx].Cr(), pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Cr, true);

          distParamSadCr.applyWeight = false;
          distParamSatdCr.applyWeight = false;

          predIntraCCCM(pu, cccmStorage[idx].Cb(), cccmStorage[idx].Cr(), mode);

          sadCb = distParamSadCb.distFunc(distParamSadCb) * 2;
          satdCb = distParamSatdCb.distFunc(distParamSatdCb);
          sad += std::min(sadCb, satdCb);
          sadCr = distParamSadCr.distFunc(distParamSadCr) * 2;
          satdCr = distParamSatdCr.distFunc(distParamSatdCr);
          sad += std::min(sadCr, satdCr);

          satdCccmSortedCost[idx] = sad;

          if (sad < bestCccmCost)
          {
            bestCccmCost = sad;
          }
        }
      }

      int tempCccmIdx = 0;
      int64_t tempCccmCost = 0;
#if MMLM
      for (int i = 1; i < 4; i++)
#else
      for (int i = 1; i < 3; i++)
#endif
      {
        for (int j = i + 1; j < 6; j++)
        {
          if (satdCccmSortedCost[j] < satdCccmSortedCost[i])
          {
            tempCccmIdx = satdCccmModeList[i];
            satdCccmModeList[i] = satdCccmModeList[j];
            satdCccmModeList[j] = tempCccmIdx;

            tempCccmCost = satdCccmSortedCost[i];
            satdCccmSortedCost[i] = satdCccmSortedCost[j];
            satdCccmSortedCost[j] = tempCccmCost;
          }
        }
      }

#if MMLM
      bool isCccmModeEnabledInRdo[MMLM_T_IDX + 1] = { false };
      isCccmModeEnabledInRdo[satdCccmModeList[0]] = true;
      for (int i = 1; i < 4; i++)
#else
      bool isCccmModeEnabledInRdo[MDLM_T_IDX + 1] = { false };
      isCccmModeEnabledInRdo[satdCccmModeList[0]] = true;
      for (int i = 1; i < 3; i++)
#endif
      {
        if (satdCccmSortedCost[i] >= 1.15 * bestCccmCost)
        {
          break;
        }
        isCccmModeEnabledInRdo[satdCccmModeList[i]] = true;
      }

      pu.cccmFlag = 0;
#endif

#if MMLM
      m_encPreRDRun = false;
#endif
      // sort the mode based on the cost from small to large.
      int tempIdx = 0;
      int64_t tempCost = 0;
      for (int i = uiMinMode; i <= uiMaxMode - 1; i++)
      {
        for (int j = i + 1; j <= uiMaxMode - 1; j++)
        {
          if (satdSortedCost[j] < satdSortedCost[i])
          {
            tempIdx = satdModeList[i];
            satdModeList[i] = satdModeList[j];
            satdModeList[j] = tempIdx;

            tempCost = satdSortedCost[i];
            satdSortedCost[i] = satdSortedCost[j];
            satdSortedCost[j] = tempCost;

          }
        }
      }
      int reducedModeNumber = 2; // reduce the number of chroma modes
#if MMLM
      reducedModeNumber += 3;    // Match number of RDs with the anchor
#endif
      for (int i = 0; i < reducedModeNumber; i++)
      {
        modeIsEnable[satdModeList[uiMaxMode - 1 - i]] = 0; // disable the last reducedModeNumber modes
      }

      // save the dist
      Distortion baseDist = cs.dist;
      bool testBDPCM = true;
      testBDPCM = testBDPCM && CU::bdpcmAllowed(cu, COMPONENT_Cb) && cu.ispMode == 0 && cu.mtsFlag == 0 && cu.lfnstIdx == 0;
#if JVET_Z0050_DIMD_CHROMA_FUSION
      double dBestNonLmCost = MAX_DOUBLE;
#if ENABLE_DIMD
      int bestNonLmMode = (cu.slice->getSPS()->getUseDimd()) ? DIMD_CHROMA_IDX : DM_CHROMA_IDX;
#else
      int bestNonLmMode = DM_CHROMA_IDX;
#endif
#endif
      for (int32_t uiMode = uiMinMode - (2 * int(testBDPCM)); uiMode < uiMaxMode; uiMode++)
      {
        int chromaIntraMode;

        if (uiMode < 0)
        {
            cu.bdpcmModeChroma = -uiMode;
            chromaIntraMode = cu.bdpcmModeChroma == 2 ? chromaCandModes[1] : chromaCandModes[2];
        }
        else
        {
          chromaIntraMode = chromaCandModes[uiMode];

          cu.bdpcmModeChroma = 0;
          if( PU::isLMCMode( chromaIntraMode ) && ! PU::isLMCModeEnabled( pu, chromaIntraMode ) )
          {
            continue;
          }
          if (!modeIsEnable[chromaIntraMode] && PU::isLMCModeEnabled(pu, chromaIntraMode)) // when CCLM is disable, then MDLM is disable. not use satd checking
          {
            continue;
          }
#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
          if (chromaIntraMode == DIMD_CHROMA_IDX && !cu.slice->getSPS()->getUseDimd()) // when DIMD is disable, then DIMD_CHROMA is disable.
          {
            continue;
          }
#endif
        }
        cs.setDecomp( pu.Cb(), false );
        cs.dist = baseDist;
        //----- restore context models -----
        m_CABACEstimator->getCtx() = ctxStart;

        //----- chroma coding -----
        pu.intraDir[1] = chromaIntraMode;

        xRecurIntraChromaCodingQT( cs, partitioner, bestCostSoFar, ispType );
        if( lumaUsesISP && cs.dist == MAX_UINT )
        {
          continue;
        }

        if (cs.sps->getTransformSkipEnabledFlag())
        {
          m_CABACEstimator->getCtx() = ctxStart;
        }

        uint64_t fracBits   = xGetIntraFracBitsQT( cs, partitioner, false, true, -1, ispType );
        Distortion uiDist = cs.dist;
        double    dCost   = m_pcRdCost->calcRdCost( fracBits, uiDist - baseDist );

        //----- compare -----
        if( dCost < dBestCost )
        {
          if( lumaUsesISP && dCost < bestCostSoFar )
          {
            bestCostSoFar = dCost;
          }
          for( uint32_t i = getFirstComponentOfChannel( CHANNEL_TYPE_CHROMA ); i < numberValidComponents; i++ )
          {
            const CompArea &area = pu.blocks[i];

            saveCS.getRecoBuf     ( area ).copyFrom( cs.getRecoBuf   ( area ) );
#if KEEP_PRED_AND_RESI_SIGNALS
            saveCS.getPredBuf     ( area ).copyFrom( cs.getPredBuf   ( area ) );
            saveCS.getResiBuf     ( area ).copyFrom( cs.getResiBuf   ( area ) );
#endif
            saveCS.getPredBuf     ( area ).copyFrom( cs.getPredBuf   (area ) );
            cs.picture->getPredBuf( area ).copyFrom( cs.getPredBuf   (area ) );
#if JVET_Z0118_GDR
            cs.updateReconMotIPM(area);
#else
            cs.picture->getRecoBuf( area ).copyFrom( cs.getRecoBuf( area ) );
#endif

            for( uint32_t j = 0; j < saveCS.tus.size(); j++ )
            {
              saveCS.tus[j]->copyComponentFrom( *orgTUs[j], area.compID );
            }
          }

          dBestCost  = dCost;
          uiBestDist = uiDist;
          uiBestMode = chromaIntraMode;
          bestBDPCMMode = cu.bdpcmModeChroma;
        }
#if JVET_Z0050_DIMD_CHROMA_FUSION
        bool findBestNonLm = !PU::isLMCMode(chromaIntraMode) && !cu.bdpcmModeChroma && pu.cs->slice->isIntra();
        if (findBestNonLm && dCost < dBestNonLmCost)
        {
          bestNonLmMode = chromaIntraMode;
          dBestNonLmCost = dCost;
        }
#endif
      }

#if JVET_AA0126_GLM
      for (int32_t uiMode = 0; uiMode < NUM_LMC_MODE; uiMode++)
      {
        int chromaIntraMode = LM_CHROMA_IDX + uiMode;
        if ( PU::isLMCModeEnabled( pu, chromaIntraMode ) && PU::hasGlmFlag( pu, chromaIntraMode ) )
        {
          if ( satdGlmIdcBest[chromaIntraMode - LM_CHROMA_IDX].isActive() )
          {
            pu.intraDir[1] = chromaIntraMode;
            pu.glmIdc      = satdGlmIdcBest[chromaIntraMode - LM_CHROMA_IDX];

            // RD search replicated from above
            cs.setDecomp( pu.Cb(), false );
            cs.dist = baseDist;
            //----- restore context models -----
            m_CABACEstimator->getCtx() = ctxStart;

            xRecurIntraChromaCodingQT( cs, partitioner, bestCostSoFar, ispType );
            if( lumaUsesISP && cs.dist == MAX_UINT )
            {
              continue;
            }

            if (cs.sps->getTransformSkipEnabledFlag())
            {
              m_CABACEstimator->getCtx() = ctxStart;
            }

            uint64_t fracBits = xGetIntraFracBitsQT( cs, partitioner, false, true, -1, ispType );
            Distortion uiDist = cs.dist;
            double    dCost   = m_pcRdCost->calcRdCost( fracBits, uiDist - baseDist );

            //----- compare -----
            if( dCost < dBestCost )
            {
              if( lumaUsesISP && dCost < bestCostSoFar )
              {
                bestCostSoFar = dCost;
              }
              for( uint32_t i = getFirstComponentOfChannel( CHANNEL_TYPE_CHROMA ); i < numberValidComponents; i++ )
              {
                const CompArea &area = pu.blocks[i];

                saveCS.getRecoBuf     ( area ).copyFrom( cs.getRecoBuf   ( area ) );
#if KEEP_PRED_AND_RESI_SIGNALS
                saveCS.getPredBuf     ( area ).copyFrom( cs.getPredBuf   ( area ) );
                saveCS.getResiBuf     ( area ).copyFrom( cs.getResiBuf   ( area ) );
#endif
                saveCS.getPredBuf     ( area ).copyFrom( cs.getPredBuf   (area ) );
                cs.picture->getPredBuf( area ).copyFrom( cs.getPredBuf   (area ) );
                cs.picture->getRecoBuf( area ).copyFrom( cs.getRecoBuf( area ) );

                for( uint32_t j = 0; j < saveCS.tus.size(); j++ )
                {
                  saveCS.tus[j]->copyComponentFrom( *orgTUs[j], area.compID );
                }
              }

              dBestCost       = dCost;
              uiBestDist      = uiDist;
              uiBestMode      = chromaIntraMode;
              bestBDPCMMode   = cu.bdpcmModeChroma;
              bestGlmIdc      = pu.glmIdc;
            }
#if !JVET_AB0092_GLM_WITH_LUMA
            if ( chromaIntraMode == LM_CHROMA_IDX && !bestGlmIdc.isActive() )
            {
              break;
            }
#endif
          }
        }
      }
      
      pu.glmIdc.setAllZero();
#endif

#if JVET_Z0050_DIMD_CHROMA_FUSION
      // RDO for chroma fusion mode
      for (int32_t uiMode = 0; uiMode < 1; uiMode++)
      {
        int chromaIntraMode = bestNonLmMode;
#if ENABLE_DIMD
        if (!pu.cs->slice->isIntra() && cu.slice->getSPS()->getUseDimd())
        {
          chromaIntraMode = DIMD_CHROMA_IDX;
        }
#endif
        if (PU::hasChromaFusionFlag(pu, chromaIntraMode))
        {
          pu.isChromaFusion = true;
          cs.setDecomp(pu.Cb(), false);
          cs.dist = baseDist;
          //----- restore context models -----
          m_CABACEstimator->getCtx() = ctxStart;

          //----- chroma coding -----
          pu.intraDir[1] = chromaIntraMode;
          xRecurIntraChromaCodingQT(cs, partitioner, bestCostSoFar, ispType);
          if (lumaUsesISP && cs.dist == MAX_UINT)
          {
            continue;
          }
          if (cs.sps->getTransformSkipEnabledFlag())
          {
            m_CABACEstimator->getCtx() = ctxStart;
          }
          uint64_t fracBits = xGetIntraFracBitsQT(cs, partitioner, false, true, -1, ispType);
          Distortion uiDist = cs.dist;
          double    dCost = m_pcRdCost->calcRdCost(fracBits, uiDist - baseDist);
          if (dCost < dBestCost)
          {
            if (lumaUsesISP && dCost < bestCostSoFar)
            {
              bestCostSoFar = dCost;
            }
            for (uint32_t i = getFirstComponentOfChannel(CHANNEL_TYPE_CHROMA); i < numberValidComponents; i++)
            {
              const CompArea &area = pu.blocks[i];
              saveCS.getRecoBuf(area).copyFrom(cs.getRecoBuf(area));
              saveCS.getPredBuf(area).copyFrom(cs.getPredBuf(area));
              saveCS.getResiBuf(area).copyFrom(cs.getResiBuf(area));
              saveCS.getPredBuf(area).copyFrom(cs.getPredBuf(area));
              cs.picture->getPredBuf(area).copyFrom(cs.getPredBuf(area));
              cs.picture->getRecoBuf(area).copyFrom(cs.getRecoBuf(area));
              for (uint32_t j = 0; j < saveCS.tus.size(); j++)
              {
                saveCS.tus[j]->copyComponentFrom(*orgTUs[j], area.compID);
              }
            }
            dBestCost = dCost;
            uiBestDist = uiDist;
            uiBestMode = chromaIntraMode;
            bestBDPCMMode = cu.bdpcmModeChroma;
            isChromaFusion = pu.isChromaFusion;
#if JVET_AA0126_GLM
            bestGlmIdc = pu.glmIdc;
#endif
          }
        }
      }
      pu.isChromaFusion = false;
#endif

#if JVET_Z0050_CCLM_SLOPE
#if MMLM
      for (int32_t uiMode = 0; uiMode < 2; uiMode++)
      {
        int chromaIntraMode = uiMode ? MMLM_CHROMA_IDX : LM_CHROMA_IDX;
#else
      for (int32_t uiMode = 0; uiMode < 1; uiMode++)
      {
        int chromaIntraMode = LM_CHROMA_IDX;
#endif

        if ( PU::isLMCModeEnabled( pu, chromaIntraMode ) && PU::hasCclmDeltaFlag( pu, chromaIntraMode ) )
        {
          if ( satdCclmOffsetsBest[chromaIntraMode - LM_CHROMA_IDX].isActive() )
          {
            pu.intraDir[1] = chromaIntraMode;
            pu.cclmOffsets = satdCclmOffsetsBest[chromaIntraMode - LM_CHROMA_IDX];
#if JVET_Z0050_DIMD_CHROMA_FUSION
            pu.isChromaFusion = false;
#endif

            // RD search replicated from above
            cs.setDecomp( pu.Cb(), false );
            cs.dist = baseDist;
            //----- restore context models -----
            m_CABACEstimator->getCtx() = ctxStart;

            xRecurIntraChromaCodingQT( cs, partitioner, bestCostSoFar, ispType );
            if( lumaUsesISP && cs.dist == MAX_UINT )
            {
              continue;
            }

            if (cs.sps->getTransformSkipEnabledFlag())
            {
              m_CABACEstimator->getCtx() = ctxStart;
            }

            uint64_t fracBits = xGetIntraFracBitsQT( cs, partitioner, false, true, -1, ispType );
            Distortion uiDist = cs.dist;
            double    dCost   = m_pcRdCost->calcRdCost( fracBits, uiDist - baseDist );

            //----- compare -----
            if( dCost < dBestCost )
            {
              if( lumaUsesISP && dCost < bestCostSoFar )
              {
                bestCostSoFar = dCost;
              }
              for( uint32_t i = getFirstComponentOfChannel( CHANNEL_TYPE_CHROMA ); i < numberValidComponents; i++ )
              {
                const CompArea &area = pu.blocks[i];

                saveCS.getRecoBuf     ( area ).copyFrom( cs.getRecoBuf   ( area ) );
#if KEEP_PRED_AND_RESI_SIGNALS
                saveCS.getPredBuf     ( area ).copyFrom( cs.getPredBuf   ( area ) );
                saveCS.getResiBuf     ( area ).copyFrom( cs.getResiBuf   ( area ) );
#endif
                saveCS.getPredBuf     ( area ).copyFrom( cs.getPredBuf   (area ) );
                cs.picture->getPredBuf( area ).copyFrom( cs.getPredBuf   (area ) );
                cs.picture->getRecoBuf( area ).copyFrom( cs.getRecoBuf( area ) );

                for( uint32_t j = 0; j < saveCS.tus.size(); j++ )
                {
                  saveCS.tus[j]->copyComponentFrom( *orgTUs[j], area.compID );
                }
              }

              dBestCost       = dCost;
              uiBestDist      = uiDist;
              uiBestMode      = chromaIntraMode;
              bestBDPCMMode   = cu.bdpcmModeChroma;
              bestCclmOffsets = pu.cclmOffsets;
#if JVET_Z0050_DIMD_CHROMA_FUSION
              isChromaFusion  = pu.isChromaFusion;
#endif
#if JVET_AA0126_GLM
              bestGlmIdc      = pu.glmIdc;
#endif
            }
          }
        }
      }
      
      pu.cclmOffsets.setAllZero();
#endif

#if JVET_AA0057_CCCM
#if JVET_AB0143_CCCM_TS
      int chromaIntraModeInCCCM = LM_CHROMA_IDX;
      isCCCMEnabled = isCccmFullEnabled;

      pu.cccmFlag = 1;
#if MMLM
      for (int32_t uiMode = 0; uiMode < 6; uiMode++)
#else
      for (int32_t uiMode = 0; uiMode < 3; uiMode++)
#endif
      {
        if (uiMode == 1)
        {
          chromaIntraModeInCCCM = MDLM_L_IDX;
          isCCCMEnabled = isCccmLeftEnabled;
          pu.cccmFlag = 2;
        }
        else if (uiMode == 2)
        {
          chromaIntraModeInCCCM = MDLM_T_IDX;
          isCCCMEnabled = isCccmTopEnabled;
          pu.cccmFlag = 3;
        }
#if MMLM
        else if (uiMode == 3)
        {
          chromaIntraModeInCCCM = MMLM_CHROMA_IDX;
          isCCCMEnabled = isMultiCccmFullEnabled;
          pu.cccmFlag = 1;
        }
        else if (uiMode == 4)
        {
          chromaIntraModeInCCCM = MMLM_L_IDX;
          isCCCMEnabled = isMultiCccmLeftEnabled;
          pu.cccmFlag = 2;
        }
        else if (uiMode == 5)
        {
          chromaIntraModeInCCCM = MMLM_T_IDX;
          isCCCMEnabled = isMultiCccmTopEnabled;
          pu.cccmFlag = 3;
        }
#endif

        if (!isCccmModeEnabledInRdo[chromaIntraModeInCCCM])
        {
          continue;
        }

        if (isCCCMEnabled)
        {
#else
#if MMLM
      for (int32_t uiMode = 0; uiMode < 2; uiMode++)
      {
        int chromaIntraMode = uiMode ? MMLM_CHROMA_IDX : LM_CHROMA_IDX;
#else
      for (int32_t uiMode = 0; uiMode < 1; uiMode++)
      {
        int chromaIntraMode = LM_CHROMA_IDX;
#endif

        if ( PU::cccmSingleModeAvail(pu, chromaIntraMode) || PU::cccmMultiModeAvail(pu, chromaIntraMode) )
        {
          pu.cccmFlag = 1;
#endif

          // Original RD check code replicated from above
          cs.setDecomp( pu.Cb(), false );
          cs.dist = baseDist;
          //----- restore context models -----
          m_CABACEstimator->getCtx() = ctxStart;

          //----- chroma coding -----
#if JVET_AB0143_CCCM_TS
          pu.intraDir[1] = chromaIntraModeInCCCM;

          xRecurIntraChromaCodingQT(cs, partitioner, bestCostSoFar, ispType, cccmStorage[uiMode]);
#else
          pu.intraDir[1] = chromaIntraMode;

          xRecurIntraChromaCodingQT( cs, partitioner, bestCostSoFar, ispType );
#endif
          if( lumaUsesISP && cs.dist == MAX_UINT )
          {
            continue;
          }

          if (cs.sps->getTransformSkipEnabledFlag())
          {
            m_CABACEstimator->getCtx() = ctxStart;
          }

          uint64_t fracBits   = xGetIntraFracBitsQT( cs, partitioner, false, true, -1, ispType );
          Distortion uiDist = cs.dist;
          double    dCost   = m_pcRdCost->calcRdCost( fracBits, uiDist - baseDist );

          //----- compare -----
          if( dCost < dBestCost )
          {
            if( lumaUsesISP && dCost < bestCostSoFar )
            {
              bestCostSoFar = dCost;
            }
            for( uint32_t i = getFirstComponentOfChannel( CHANNEL_TYPE_CHROMA ); i < numberValidComponents; i++ )
            {
              const CompArea &area = pu.blocks[i];

              saveCS.getRecoBuf     ( area ).copyFrom( cs.getRecoBuf   ( area ) );
#if KEEP_PRED_AND_RESI_SIGNALS
              saveCS.getPredBuf     ( area ).copyFrom( cs.getPredBuf   ( area ) );
              saveCS.getResiBuf     ( area ).copyFrom( cs.getResiBuf   ( area ) );
#endif
              saveCS.getPredBuf     ( area ).copyFrom( cs.getPredBuf   (area ) );
              cs.picture->getPredBuf( area ).copyFrom( cs.getPredBuf   (area ) );
#if !JVET_AB0143_CCCM_TS
              cs.picture->getRecoBuf( area ).copyFrom( cs.getRecoBuf( area ) );
#endif

              for( uint32_t j = 0; j < saveCS.tus.size(); j++ )
              {
                saveCS.tus[j]->copyComponentFrom( *orgTUs[j], area.compID );
              }
            }

            dBestCost  = dCost;
            uiBestDist = uiDist;
#if JVET_AB0143_CCCM_TS
            uiBestMode = chromaIntraModeInCCCM;
#else
            uiBestMode = chromaIntraMode;
#endif
            bestBDPCMMode = cu.bdpcmModeChroma;
#if JVET_Z0050_DIMD_CHROMA_FUSION
            isChromaFusion  = pu.isChromaFusion;
#endif
#if JVET_Z0050_CCLM_SLOPE
            bestCclmOffsets = pu.cclmOffsets;
#endif
            cccmModeBest    = pu.cccmFlag;
#if JVET_AA0126_GLM
            bestGlmIdc      = pu.glmIdc;
#endif 
          }
        }
      }
        
      pu.cccmFlag = 0;
#endif
      for( uint32_t i = getFirstComponentOfChannel( CHANNEL_TYPE_CHROMA ); i < numberValidComponents; i++ )
      {
        const CompArea &area = pu.blocks[i];

        cs.getRecoBuf         ( area ).copyFrom( saveCS.getRecoBuf( area ) );
#if KEEP_PRED_AND_RESI_SIGNALS
        cs.getPredBuf         ( area ).copyFrom( saveCS.getPredBuf( area ) );
        cs.getResiBuf         ( area ).copyFrom( saveCS.getResiBuf( area ) );
#endif
        cs.getPredBuf         ( area ).copyFrom( saveCS.getPredBuf( area ) );
        cs.picture->getPredBuf( area ).copyFrom( cs.getPredBuf    ( area ) );

#if JVET_Z0118_GDR
        cs.updateReconMotIPM(area);
#else
        cs.picture->getRecoBuf( area ).copyFrom( cs.    getRecoBuf( area ) );
#endif

        for( uint32_t j = 0; j < saveCS.tus.size(); j++ )
        {
          orgTUs[ j ]->copyComponentFrom( *saveCS.tus[ j ], area.compID );
        }
      }
    }

    pu.intraDir[1] = uiBestMode;
    cs.dist        = uiBestDist;
    cu.bdpcmModeChroma = bestBDPCMMode;
#if JVET_Z0050_CCLM_SLOPE
    pu.cclmOffsets     = bestCclmOffsets;
#endif
#if JVET_AA0057_CCCM
    pu.cccmFlag        = cccmModeBest;
#endif
#if JVET_Z0050_DIMD_CHROMA_FUSION
    pu.isChromaFusion = isChromaFusion;
#endif
#if JVET_AA0126_GLM
    pu.glmIdc          = bestGlmIdc;
#endif
  }

  //----- restore context models -----
  m_CABACEstimator->getCtx() = ctxStart;
  if( lumaUsesISP && bestCostSoFar >= maxCostAllowed )
  {
    cu.ispMode = 0;
  }
}

#if JVET_Z0050_CCLM_SLOPE
void IntraSearch::xFindBestCclmDeltaSlopeSATD(PredictionUnit &pu, ComponentID compID, int cclmModel, int &deltaBest, int64_t &sadBest )
{
  CclmModel cclmModelStored;
  CodingStructure& cs = *(pu.cs);
  CompArea       area = compID == COMPONENT_Cb ? pu.Cb() : pu.Cr();
  PelBuf       orgBuf = cs.getOrgBuf(area);
  PelBuf      predBuf = cs.getPredBuf(area);
  int       maxOffset = 4;
  int            mode = pu.intraDir[1];
  bool createNewModel = true;

  DistParam distParamSad;
  DistParam distParamSatd;

  m_pcRdCost->setDistParam(distParamSad,  orgBuf, predBuf, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), compID, false);
  m_pcRdCost->setDistParam(distParamSatd, orgBuf, predBuf, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), compID, true);
  
  distParamSad.applyWeight  = false;
  distParamSatd.applyWeight = false;
  
  sadBest = -1;

  // Search positive offsets
  for ( int offset = 0; offset <= maxOffset; offset++)
  {
    pu.cclmOffsets.setOffset(compID, cclmModel, offset);
    
    predIntraChromaLM( compID, predBuf, pu, area, mode, createNewModel, &cclmModelStored );
    
    createNewModel  = false; // Need to calculate the base model just once
    int64_t sad     = distParamSad.distFunc(distParamSad) * 2;
    int64_t satd    = distParamSatd.distFunc(distParamSatd);
    int64_t sadThis = std::min(sad, satd);
    
    if ( sadBest == -1 || sadThis < sadBest )
    {
      sadBest   = sadThis;
      deltaBest = offset;
    }
    else
    {
      break;
    }
  }
  
  // Search negative offsets only if positives didn't help
  if ( deltaBest == 0 )
  {
    for ( int offset = -1; offset >= -maxOffset; offset--)
    {
      pu.cclmOffsets.setOffset(compID, cclmModel, offset);

      predIntraChromaLM( compID, predBuf, pu, area, mode, createNewModel, &cclmModelStored );
      
      int64_t sad     = distParamSad.distFunc(distParamSad) * 2;
      int64_t satd    = distParamSatd.distFunc(distParamSatd);
      int64_t sadThis = std::min(sad, satd);
      
      if ( sadThis < sadBest )
      {
        sadBest   = sadThis;
        deltaBest = offset;
      }
      else
      {
        break;
      }
    }
  }
}
#endif

#if JVET_AA0126_GLM
void IntraSearch::xFindBestGlmIdcSATD(PredictionUnit &pu, ComponentID compID, int &idcBest, int64_t &sadBest )
{
  CodingStructure& cs = *(pu.cs);
  CompArea       area = compID == COMPONENT_Cb ? pu.Cb() : pu.Cr();
  PelBuf       orgBuf = cs.getOrgBuf(area);
  PelBuf      predBuf = cs.getPredBuf(area);
#if JVET_AB0092_GLM_WITH_LUMA
  int          maxIdc = NUM_GLM_PATTERN * NUM_GLM_WEIGHT;
#else
  int          maxIdc = NUM_GLM_IDC - 1;
#endif
  int            mode = pu.intraDir[1];

  DistParam distParamSad;
  DistParam distParamSatd;

  m_pcRdCost->setDistParam(distParamSad,  orgBuf, predBuf, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), compID, false);
  m_pcRdCost->setDistParam(distParamSatd, orgBuf, predBuf, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), compID, true);
  
  distParamSad.applyWeight  = false;
  distParamSatd.applyWeight = false;
  
  sadBest = -1;

#if JVET_AB0092_GLM_WITH_LUMA
  CompArea       areacr = pu.Cr();
  PelBuf       orgBufcr = cs.getOrgBuf(areacr);
  PelBuf      predBufcr = cs.getPredBuf(areacr);

  DistParam distParamSadcr;
  DistParam distParamSatdcr;

  m_pcRdCost->setDistParam(distParamSadcr, orgBufcr, predBufcr, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Cr, false);
  m_pcRdCost->setDistParam(distParamSatdcr, orgBufcr, predBufcr, pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA), COMPONENT_Cr, true);

  distParamSadcr.applyWeight = false;
  distParamSatdcr.applyWeight = false;
#endif

  // Search positive idcs
  for ( int idc = 0; idc <= maxIdc; idc++ )
  {
    pu.glmIdc.setIdc(compID, 0, idc);
    pu.glmIdc.setIdc(compID, 1, idc);

    predIntraChromaLM( compID, predBuf, pu, area, mode );
    
    int64_t sad     = distParamSad.distFunc(distParamSad) * 2;
    int64_t satd    = distParamSatd.distFunc(distParamSatd);
    int64_t sadThis = std::min(sad, satd);

#if JVET_AB0092_GLM_WITH_LUMA
    pu.glmIdc.setIdc(COMPONENT_Cr, 0, idc);
    pu.glmIdc.setIdc(COMPONENT_Cr, 1, idc);

    predIntraChromaLM(COMPONENT_Cr, predBufcr, pu, areacr, mode);

    int64_t sadcr = distParamSadcr.distFunc(distParamSadcr) * 2;
    int64_t satdcr = distParamSatdcr.distFunc(distParamSatdcr);
    int64_t sadThiscr = std::min(sadcr, satdcr);
    sadThis += sadThiscr;
#endif
    
    if ( sadBest == -1 || sadThis < sadBest )
    {
      sadBest   = sadThis;
      idcBest   = idc;
    }
  }
}
#endif

#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
void IntraSearch::saveCuAreaCostInSCIPU( Area area, double cost )
{
  if( m_numCuInSCIPU < NUM_INTER_CU_INFO_SAVE )
  {
    m_cuAreaInSCIPU[m_numCuInSCIPU] = area;
    m_cuCostInSCIPU[m_numCuInSCIPU] = cost;
    m_numCuInSCIPU++;
  }
}

void IntraSearch::initCuAreaCostInSCIPU()
{
  for( int i = 0; i < NUM_INTER_CU_INFO_SAVE; i++ )
  {
    m_cuAreaInSCIPU[i] = Area();
    m_cuCostInSCIPU[i] = 0;
  }
  m_numCuInSCIPU = 0;
}
#endif
void IntraSearch::PLTSearch(CodingStructure &cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp)
{
  CodingUnit    &cu = *cs.getCU(partitioner.chType);
  TransformUnit &tu = *cs.getTU(partitioner.chType);
  uint32_t height = cu.block(compBegin).height;
  uint32_t width = cu.block(compBegin).width;
  if (m_pcEncCfg->getLmcs() && (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()))
  {
    cs.getPredBuf().copyFrom(cs.getOrgBuf());
    cs.getPredBuf().Y().rspSignal(m_pcReshape->getFwdLUT());
  }
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( cu.isLocalSepTree() )
  {
    cs.prevPLT.curPLTSize[compBegin] = cs.prevPLT.curPLTSize[COMPONENT_Y];
  }
#endif
  cu.lastPLTSize[compBegin] = cs.prevPLT.curPLTSize[compBegin];
  //derive palette
  derivePLTLossy(cs, partitioner, compBegin, numComp);
  reorderPLT(cs, partitioner, compBegin, numComp);

  bool idxExist[MAXPLTSIZE + 1] = { false };
  preCalcPLTIndexRD(cs, partitioner, compBegin, numComp); // Pre-calculate distortions for each pixel
  double rdCost = MAX_DOUBLE;
  deriveIndexMap(cs, partitioner, compBegin, numComp, PLT_SCAN_HORTRAV, rdCost, idxExist); // Optimize palette index map (horizontal scan)
  if ((cu.curPLTSize[compBegin] + cu.useEscape[compBegin]) > 1)
  {
    deriveIndexMap(cs, partitioner, compBegin, numComp, PLT_SCAN_VERTRAV, rdCost, idxExist); // Optimize palette index map (vertical scan)
  }
  // Remove unused palette entries
  uint8_t newPLTSize = 0;
  int idxMapping[MAXPLTSIZE + 1];
  memset(idxMapping, -1, sizeof(int) * (MAXPLTSIZE + 1));
  for (int i = 0; i < cu.curPLTSize[compBegin]; i++)
  {
    if (idxExist[i])
    {
      idxMapping[i] = newPLTSize;
      newPLTSize++;
    }
  }
  idxMapping[cu.curPLTSize[compBegin]] = cu.useEscape[compBegin]? newPLTSize: -1;
  if (newPLTSize != cu.curPLTSize[compBegin]) // there exist unused palette entries
  { // update palette table and reuseflag
    Pel curPLTtmp[MAX_NUM_COMPONENT][MAXPLTSIZE];
    int reuseFlagIdx = 0, curPLTtmpIdx = 0, reuseEntrySize = 0;
    memset(cu.reuseflag[compBegin], false, sizeof(bool) * MAXPLTPREDSIZE);
    int compBeginTmp = compBegin;
    int numCompTmp   = numComp;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if( cu.isLocalSepTree() )
    {
      memset(cu.reuseflag[COMPONENT_Y], false, sizeof(bool) * MAXPLTPREDSIZE);
      compBeginTmp = COMPONENT_Y;
      numCompTmp   = (cu.chromaFormat != CHROMA_400) ? 3 : 1;
    }
#endif
    for (int curIdx = 0; curIdx < cu.curPLTSize[compBegin]; curIdx++)
    {
      if (idxExist[curIdx])
      {
        for (int comp = compBeginTmp; comp < (compBeginTmp + numCompTmp); comp++)
          curPLTtmp[comp][curPLTtmpIdx] = cu.curPLT[comp][curIdx];

        // Update reuse flags
        if (curIdx < cu.reusePLTSize[compBegin])
        {
          bool match = false;
          for (; reuseFlagIdx < cs.prevPLT.curPLTSize[compBegin]; reuseFlagIdx++)
          {
            bool matchTmp = true;
            for (int comp = compBegin; comp < (compBegin + numComp); comp++)
            {
              matchTmp = matchTmp && (curPLTtmp[comp][curPLTtmpIdx] == cs.prevPLT.curPLT[comp][reuseFlagIdx]);
            }
            if (matchTmp)
            {
              match = true;
              break;
            }
          }
          if (match)
          {
            cu.reuseflag[compBegin][reuseFlagIdx] = true;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
            if( cu.isLocalSepTree() )
            {
              cu.reuseflag[COMPONENT_Y][reuseFlagIdx] = true;
            }
#endif
            reuseEntrySize++;
          }
        }
        curPLTtmpIdx++;
      }
    }
    cu.reusePLTSize[compBegin] = reuseEntrySize;
    // update palette table
    cu.curPLTSize[compBegin] = newPLTSize;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if( cu.isLocalSepTree() )
    {
      cu.curPLTSize[COMPONENT_Y] = newPLTSize;
    }
#endif
    for (int comp = compBeginTmp; comp < (compBeginTmp + numCompTmp); comp++)
    {
      memcpy( cu.curPLT[comp], curPLTtmp[comp], sizeof(Pel)*cu.curPLTSize[compBegin]);
    }
  }
  cu.useRotation[compBegin] = m_bestScanRotationMode;
  int indexMaxSize = cu.useEscape[compBegin] ? (cu.curPLTSize[compBegin] + 1) : cu.curPLTSize[compBegin];
  if (indexMaxSize <= 1)
  {
    cu.useRotation[compBegin] = false;
  }
  //reconstruct pixel
  PelBuf    curPLTIdx = tu.getcurPLTIdx(compBegin);
  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      curPLTIdx.at(x, y) = idxMapping[curPLTIdx.at(x, y)];
      if (curPLTIdx.at(x, y) == cu.curPLTSize[compBegin])
      {
        calcPixelPred(cs, partitioner, y, x, compBegin, numComp);
      }
      else
      {
        for (uint32_t compID = compBegin; compID < (compBegin + numComp); compID++)
        {
          CompArea area = cu.blocks[compID];
          PelBuf   recBuf = cs.getRecoBuf(area);
          uint32_t scaleX = getComponentScaleX((ComponentID)COMPONENT_Cb, cs.sps->getChromaFormatIdc());
          uint32_t scaleY = getComponentScaleY((ComponentID)COMPONENT_Cb, cs.sps->getChromaFormatIdc());
          if (compBegin != COMPONENT_Y || compID == COMPONENT_Y)
          {
            recBuf.at(x, y) = cu.curPLT[compID][curPLTIdx.at(x, y)];
          }
          else if (compBegin == COMPONENT_Y && compID != COMPONENT_Y && y % (1 << scaleY) == 0 && x % (1 << scaleX) == 0)
          {
            recBuf.at(x >> scaleX, y >> scaleY) = cu.curPLT[compID][curPLTIdx.at(x, y)];
          }
        }
      }
    }
  }

  cs.getPredBuf().fill(0);
  cs.getResiBuf().fill(0);
  cs.getOrgResiBuf().fill(0);

  cs.fracBits = MAX_UINT;
  cs.cost = MAX_DOUBLE;
  Distortion distortion = 0;
  for (uint32_t comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    const ComponentID compID = ComponentID(comp);
    CPelBuf reco = cs.getRecoBuf(compID);
    CPelBuf org = cs.getOrgBuf(compID);
#if WCG_EXT
    if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || (
      m_pcEncCfg->getLmcs() && (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())))
    {
      const CPelBuf orgLuma = cs.getOrgBuf(cs.area.blocks[COMPONENT_Y]);

      if (compID == COMPONENT_Y && !(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()))
      {
        const CompArea &areaY = cu.Y();
        CompArea tmpArea1(COMPONENT_Y, areaY.chromaFormat, Position(0, 0), areaY.size());
        PelBuf   tmpRecLuma = m_tmpStorageLCU.getBuf(tmpArea1);
        tmpRecLuma.rspSignal( reco, m_pcReshape->getInvLUT() );
        distortion += m_pcRdCost->getDistPart(org, tmpRecLuma, cs.sps->getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
      }
      else
      {
        distortion += m_pcRdCost->getDistPart(org, reco, cs.sps->getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
      }
    }
    else
#endif
    {
      distortion += m_pcRdCost->getDistPart(org, reco, cs.sps->getBitDepth(toChannelType(compID)), compID, DF_SSE);
    }
  }

  cs.dist += distortion;
  const CompArea &area = cu.blocks[compBegin];
  cs.setDecomp(area);
#if JVET_Z0118_GDR
  cs.updateReconMotIPM(area);
#else
  cs.picture->getRecoBuf(area).copyFrom(cs.getRecoBuf(area));
#endif
}
void IntraSearch::calcPixelPredRD(CodingStructure& cs, Partitioner& partitioner, Pel* orgBuf, Pel* paPixelValue, Pel* paRecoValue, ComponentID compBegin, uint32_t numComp)
{
  CodingUnit &cu = *cs.getCU(partitioner.chType);
  TransformUnit &tu = *cs.getTU(partitioner.chType);

  int qp[3];
  int qpRem[3];
  int qpPer[3];
  int quantiserScale[3];
  int quantiserRightShift[3];
  int rightShiftOffset[3];
  int invquantiserRightShift[3];
  int add[3];
  for (uint32_t ch = compBegin; ch < (compBegin + numComp); ch++)
  {
    QpParam cQP(tu, ComponentID(ch));
    qp[ch] = cQP.Qp(true);
    qpRem[ch] = qp[ch] % 6;
    qpPer[ch] = qp[ch] / 6;
    quantiserScale[ch] = g_quantScales[0][qpRem[ch]];
    quantiserRightShift[ch] = QUANT_SHIFT + qpPer[ch];
    rightShiftOffset[ch] = 1 << (quantiserRightShift[ch] - 1);
    invquantiserRightShift[ch] = IQUANT_SHIFT;
    add[ch] = 1 << (invquantiserRightShift[ch] - 1);
  }

  for (uint32_t ch = compBegin; ch < (compBegin + numComp); ch++)
  {
    const int  channelBitDepth = cu.cs->sps->getBitDepth(toChannelType((ComponentID)ch));
    paPixelValue[ch] = Pel(std::max<int>(0, ((orgBuf[ch] * quantiserScale[ch] + rightShiftOffset[ch]) >> quantiserRightShift[ch])));
    assert(paPixelValue[ch] < (1 << (channelBitDepth + 1)));
    paRecoValue[ch] = (((paPixelValue[ch] * g_invQuantScales[0][qpRem[ch]]) << qpPer[ch]) + add[ch]) >> invquantiserRightShift[ch];
    paRecoValue[ch] = Pel(ClipBD<int>(paRecoValue[ch], channelBitDepth));//to be checked
  }
}

void IntraSearch::preCalcPLTIndexRD(CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp)
{
  CodingUnit &cu = *cs.getCU(partitioner.chType);
  uint32_t height = cu.block(compBegin).height;
  uint32_t width = cu.block(compBegin).width;
  bool lossless = (m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && cs.slice->isLossless());

  CPelBuf   orgBuf[3];
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    CompArea  area = cu.blocks[comp];
    if (m_pcEncCfg->getLmcs() && (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()))
    {
      orgBuf[comp] = cs.getPredBuf(area);
    }
    else
    {
      orgBuf[comp] = cs.getOrgBuf(area);
    }
  }

  int rasPos;
  uint32_t scaleX = getComponentScaleX(COMPONENT_Cb, cs.sps->getChromaFormatIdc());
  uint32_t scaleY = getComponentScaleY(COMPONENT_Cb, cs.sps->getChromaFormatIdc());
  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      rasPos = y * width + x;;
      // chroma discard
      bool discardChroma = (compBegin == COMPONENT_Y) && (y&scaleY || x&scaleX);
      Pel curPel[3];
      for (int comp = compBegin; comp < (compBegin + numComp); comp++)
      {
        uint32_t pX1 = (comp > 0 && compBegin == COMPONENT_Y) ? (x >> scaleX) : x;
        uint32_t pY1 = (comp > 0 && compBegin == COMPONENT_Y) ? (y >> scaleY) : y;
        curPel[comp] = orgBuf[comp].at(pX1, pY1);
      }

      uint8_t  pltIdx = 0;
      double minError = MAX_DOUBLE;
      uint8_t  bestIdx = 0;
      for (uint8_t z = 0; z < cu.curPLTSize[compBegin]; z++)
      {
        m_indexError[z][rasPos] = minError;
      }
      while (pltIdx < cu.curPLTSize[compBegin])
      {
        uint64_t sqrtError = 0;
        if (lossless)
        {
          for (int comp = compBegin; comp < (discardChroma ? 1 : (compBegin + numComp)); comp++)
          {
            sqrtError += int64_t(abs(curPel[comp] - cu.curPLT[comp][pltIdx]));
          }
          if (sqrtError == 0)
          {
            m_indexError[pltIdx][rasPos] = (double) sqrtError;
            minError                     = (double) sqrtError;
            bestIdx                      = pltIdx;
            break;
          }
        }
        else
        {
          for (int comp = compBegin; comp < (discardChroma ? 1 : (compBegin + numComp)); comp++)
          {
            int64_t tmpErr = int64_t(curPel[comp] - cu.curPLT[comp][pltIdx]);
            if (isChroma((ComponentID) comp))
            {
              sqrtError += uint64_t(tmpErr * tmpErr * ENC_CHROMA_WEIGHTING);
            }
            else
            {
              sqrtError += tmpErr * tmpErr;
            }
          }
          m_indexError[pltIdx][rasPos] = (double) sqrtError;
          if (sqrtError < minError)
          {
            minError = (double) sqrtError;
            bestIdx  = pltIdx;
          }
        }
        pltIdx++;
      }

      Pel paPixelValue[3], paRecoValue[3];
      if (!lossless)
      {
        calcPixelPredRD(cs, partitioner, curPel, paPixelValue, paRecoValue, compBegin, numComp);
      }
      uint64_t error = 0, rate = 0;
      for (int comp = compBegin; comp < (discardChroma ? 1 : (compBegin + numComp)); comp++)
      {
        if (lossless)
        {
          rate += m_escapeNumBins[curPel[comp]];
        }
        else
        {
          int64_t tmpErr = int64_t(curPel[comp] - paRecoValue[comp]);
          if (isChroma((ComponentID) comp))
          {
            error += uint64_t(tmpErr * tmpErr * ENC_CHROMA_WEIGHTING);
          }
          else
          {
            error += tmpErr * tmpErr;
          }
          rate += m_escapeNumBins[paPixelValue[comp]];   // encode quantized escape color
        }
      }
      double rdCost = (double)error + m_pcRdCost->getLambda()*(double)rate;
      m_indexError[cu.curPLTSize[compBegin]][rasPos] = rdCost;
      if (rdCost < minError)
      {
        minError = rdCost;
        bestIdx = (uint8_t)cu.curPLTSize[compBegin];
      }
      m_minErrorIndexMap[rasPos] = bestIdx; // save the optimal index of the current pixel
    }
  }
}

void IntraSearch::deriveIndexMap(CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp, PLTScanMode pltScanMode, double& dMinCost, bool* idxExist)
{
  CodingUnit    &cu = *cs.getCU(partitioner.chType);
  TransformUnit &tu = *cs.getTU(partitioner.chType);
  uint32_t      height = cu.block(compBegin).height;
  uint32_t      width = cu.block(compBegin).width;

  int   total     = height*width;
  Pel  *runIndex = tu.getPLTIndex(compBegin);
  bool *runType  = tu.getRunTypes(compBegin);
  m_scanOrder = g_scanOrder[SCAN_UNGROUPED][pltScanMode ? SCAN_TRAV_VER : SCAN_TRAV_HOR][gp_sizeIdxInfo->idxFrom(width)][gp_sizeIdxInfo->idxFrom(height)];
// Trellis initialization
  for (int i = 0; i < 2; i++)
  {
    memset(m_prevRunTypeRDOQ[i], 0, sizeof(Pel)*NUM_TRELLIS_STATE);
    memset(m_prevRunPosRDOQ[i],  0, sizeof(int)*NUM_TRELLIS_STATE);
    memset(m_stateCostRDOQ[i],  0, sizeof (double)*NUM_TRELLIS_STATE);
  }
  for (int state = 0; state < NUM_TRELLIS_STATE; state++)
  {
    m_statePtRDOQ[state][0] = 0;
  }
// Context modeling
  const FracBitsAccess& fracBits = m_CABACEstimator->getCtx().getFracBitsAcess();
  BinFracBits fracBitsPltCopyFlagIndex[RUN_IDX_THRE + 1];
  for (int dist = 0; dist <= RUN_IDX_THRE; dist++)
  {
    const unsigned  ctxId = DeriveCtx::CtxPltCopyFlag(PLT_RUN_INDEX, dist);
    fracBitsPltCopyFlagIndex[dist] = fracBits.getFracBitsArray(Ctx::IdxRunModel( ctxId ) );
  }
  BinFracBits fracBitsPltCopyFlagAbove[RUN_IDX_THRE + 1];
  for (int dist = 0; dist <= RUN_IDX_THRE; dist++)
  {
    const unsigned  ctxId = DeriveCtx::CtxPltCopyFlag(PLT_RUN_COPY, dist);
    fracBitsPltCopyFlagAbove[dist] = fracBits.getFracBitsArray(Ctx::CopyRunModel( ctxId ) );
  }
  const BinFracBits fracBitsPltRunType = fracBits.getFracBitsArray( Ctx::RunTypeFlag() );

// Trellis RDO per CG
  bool contTrellisRD = true;
  for (int subSetId = 0; ( subSetId <= (total - 1) >> LOG2_PALETTE_CG_SIZE ) && contTrellisRD; subSetId++)
  {
    int minSubPos = subSetId << LOG2_PALETTE_CG_SIZE;
    int maxSubPos = minSubPos + (1 << LOG2_PALETTE_CG_SIZE);
    maxSubPos = (maxSubPos > total) ? total : maxSubPos; // if last position is out of the current CU size
    contTrellisRD = deriveSubblockIndexMap(cs, partitioner, compBegin, pltScanMode, minSubPos, maxSubPos, fracBitsPltRunType, fracBitsPltCopyFlagIndex, fracBitsPltCopyFlagAbove, dMinCost, (bool)pltScanMode);
  }
  if (!contTrellisRD)
  {
    return;
  }


// best state at the last scan position
  double  sumRdCost = MAX_DOUBLE;
  uint8_t bestState = 0;
  for (uint8_t state = 0; state < NUM_TRELLIS_STATE; state++)
  {
    if (m_stateCostRDOQ[0][state] < sumRdCost)
    {
      sumRdCost = m_stateCostRDOQ[0][state];
      bestState = state;
    }
  }

     bool checkRunTable  [MAX_CU_BLKSIZE_PLT*MAX_CU_BLKSIZE_PLT];
  uint8_t checkIndexTable[MAX_CU_BLKSIZE_PLT*MAX_CU_BLKSIZE_PLT];
  uint8_t bestStateTable [MAX_CU_BLKSIZE_PLT*MAX_CU_BLKSIZE_PLT];
  uint8_t nextState = bestState;
// best trellis path
  for (int i = (width*height - 1); i >= 0; i--)
  {
    bestStateTable[i] = nextState;
    int rasterPos = m_scanOrder[i].idx;
    nextState = m_statePtRDOQ[nextState][rasterPos];
  }
// reconstruct index and runs based on the state pointers
  for (int i = 0; i < (width*height); i++)
  {
    int rasterPos = m_scanOrder[i].idx;
    int  abovePos = (pltScanMode == PLT_SCAN_HORTRAV) ? m_scanOrder[i].idx - width : m_scanOrder[i].idx - 1;
        nextState = bestStateTable[i];
    if ( nextState == 0 ) // same as the previous
    {
      checkRunTable[rasterPos] = checkRunTable[ m_scanOrder[i - 1].idx ];
      if ( checkRunTable[rasterPos] == PLT_RUN_INDEX )
      {
        checkIndexTable[rasterPos] = checkIndexTable[m_scanOrder[i - 1].idx];
      }
      else
      {
        checkIndexTable[rasterPos] = checkIndexTable[ abovePos ];
      }
    }
    else if (nextState == 1) // CopyAbove mode
    {
      checkRunTable[rasterPos] = PLT_RUN_COPY;
      checkIndexTable[rasterPos] = checkIndexTable[abovePos];
    }
    else if (nextState == 2) // Index mode
    {
      checkRunTable[rasterPos] = PLT_RUN_INDEX;
      checkIndexTable[rasterPos] = m_minErrorIndexMap[rasterPos];
    }
  }

// Escape flag
  m_bestEscape = false;
  for (int pos = 0; pos < (width*height); pos++)
  {
    uint8_t index = checkIndexTable[pos];
    if (index == cu.curPLTSize[compBegin])
    {
      m_bestEscape = true;
      break;
    }
  }

// Horizontal scan v.s vertical scan
  if (sumRdCost < dMinCost)
  {
    cu.useEscape[compBegin] = m_bestEscape;
    m_bestScanRotationMode = pltScanMode;
    memset(idxExist, false, sizeof(bool) * (MAXPLTSIZE + 1));
    for (int pos = 0; pos < (width*height); pos++)
    {
      runIndex[pos] = checkIndexTable[pos];
      runType[pos] = checkRunTable[pos];
      idxExist[checkIndexTable[pos]] = true;
    }
    dMinCost = sumRdCost;
  }
}

bool IntraSearch::deriveSubblockIndexMap(
  CodingStructure& cs,
  Partitioner&  partitioner,
  ComponentID   compBegin,
  PLTScanMode   pltScanMode,
  int           minSubPos,
  int           maxSubPos,
  const BinFracBits& fracBitsPltRunType,
  const BinFracBits* fracBitsPltIndexINDEX,
  const BinFracBits* fracBitsPltIndexCOPY,
  const double minCost,
  bool         useRotate
)
{
  CodingUnit &cu    = *cs.getCU(partitioner.chType);
  uint32_t   height = cu.block(compBegin).height;
  uint32_t   width  = cu.block(compBegin).width;
  int indexMaxValue = cu.curPLTSize[compBegin];

  int refId = 0;
  int currRasterPos, currScanPos, prevScanPos, aboveScanPos, roffset;
  int log2Width = (pltScanMode == PLT_SCAN_HORTRAV) ? floorLog2(width): floorLog2(height);
  int buffersize = (pltScanMode == PLT_SCAN_HORTRAV) ? 2*width: 2*height;
  for (int curPos = minSubPos; curPos < maxSubPos; curPos++)
  {
    currRasterPos = m_scanOrder[curPos].idx;
    prevScanPos = (curPos == 0) ? 0 : (curPos - 1) % buffersize;
    roffset = (curPos >> log2Width) << log2Width;
    aboveScanPos = roffset - (curPos - roffset + 1);
    aboveScanPos %= buffersize;
    currScanPos = curPos % buffersize;
    if ((pltScanMode == PLT_SCAN_HORTRAV && curPos < width) || (pltScanMode == PLT_SCAN_VERTRAV && curPos < height))
    {
      aboveScanPos = -1; // first column/row: above row is not valid
    }

// Trellis stats:
// 1st state: same as previous scanned sample
// 2nd state: Copy_Above mode
// 3rd state: Index mode
// Loop of current state
    for ( int curState = 0; curState < NUM_TRELLIS_STATE; curState++ )
    {
      double    minRdCost          = MAX_DOUBLE;
      int       minState           = 0; // best prevState
      uint8_t   bestRunIndex       = 0;
      bool      bestRunType        = 0;
      bool      bestPrevCodedType  = 0;
      int       bestPrevCodedPos   = 0;
      if ( ( curState == 0 && curPos == 0 ) || ( curState == 1 && aboveScanPos < 0 ) ) // state not available
      {
        m_stateCostRDOQ[1 - refId][curState] = MAX_DOUBLE;
        continue;
      }

      bool    runType  = 0;
      uint8_t runIndex = 0;
      if ( curState == 1 ) // 2nd state: Copy_Above mode
      {
        runType = PLT_RUN_COPY;
      }
      else if ( curState == 2 ) // 3rd state: Index mode
      {
        runType = PLT_RUN_INDEX;
        runIndex = m_minErrorIndexMap[currRasterPos];
      }

// Loop of previous state
      for ( int stateID = 0; stateID < NUM_TRELLIS_STATE; stateID++ )
      {
        if ( m_stateCostRDOQ[refId][stateID] == MAX_DOUBLE )
        {
          continue;
        }
        if ( curState == 0 ) // 1st state: same as previous scanned sample
        {
          runType = m_runMapRDOQ[refId][stateID][prevScanPos];
          runIndex = ( runType == PLT_RUN_INDEX ) ? m_indexMapRDOQ[refId][stateID][ prevScanPos ] : m_indexMapRDOQ[refId][stateID][ aboveScanPos ];
        }
        else if ( curState == 1 ) // 2nd state: Copy_Above mode
        {
          runIndex = m_indexMapRDOQ[refId][stateID][aboveScanPos];
        }
        bool    prevRunType   = m_runMapRDOQ[refId][stateID][prevScanPos];
        uint8_t prevRunIndex  = m_indexMapRDOQ[refId][stateID][prevScanPos];
        uint8_t aboveRunIndex = (aboveScanPos >= 0) ? m_indexMapRDOQ[refId][stateID][aboveScanPos] : 0;
        int      dist = curPos - m_prevRunPosRDOQ[refId][stateID] - 1;
        double rdCost = m_stateCostRDOQ[refId][stateID];
        if ( rdCost >= minRdCost ) continue;

// Calculate Rd cost
        bool prevCodedRunType = m_prevRunTypeRDOQ[refId][stateID];
        int  prevCodedPos     = m_prevRunPosRDOQ [refId][stateID];
        const BinFracBits* fracBitsPt = (m_prevRunTypeRDOQ[refId][stateID] == PLT_RUN_INDEX) ? fracBitsPltIndexINDEX : fracBitsPltIndexCOPY;
        rdCost += rateDistOptPLT(runType, runIndex, prevRunType, prevRunIndex, aboveRunIndex, prevCodedRunType, prevCodedPos, curPos, (pltScanMode == PLT_SCAN_HORTRAV) ? width : height, dist, indexMaxValue, fracBitsPt, fracBitsPltRunType);
        if (rdCost < minRdCost) // update minState ( minRdCost )
        {
          minRdCost    = rdCost;
          minState     = stateID;
          bestRunType  = runType;
          bestRunIndex = runIndex;
          bestPrevCodedType = prevCodedRunType;
          bestPrevCodedPos  = prevCodedPos;
        }
      }
// Update trellis info of current state
      m_stateCostRDOQ  [1 - refId][curState]  = minRdCost;
      m_prevRunTypeRDOQ[1 - refId][curState]  = bestPrevCodedType;
      m_prevRunPosRDOQ [1 - refId][curState]  = bestPrevCodedPos;
      m_statePtRDOQ[curState][currRasterPos] = minState;
      int buffer2update = std::min(buffersize, curPos);
      memcpy(m_indexMapRDOQ[1 - refId][curState], m_indexMapRDOQ[refId][minState], sizeof(uint8_t)*buffer2update);
      memcpy(m_runMapRDOQ[1 - refId][curState], m_runMapRDOQ[refId][minState], sizeof(bool)*buffer2update);
      m_indexMapRDOQ[1 - refId][curState][currScanPos] = bestRunIndex;
      m_runMapRDOQ  [1 - refId][curState][currScanPos] = bestRunType;
    }

    if (useRotate) // early terminate: Rd cost >= min cost in horizontal scan
    {
      if ((m_stateCostRDOQ[1 - refId][0] >= minCost) &&
         (m_stateCostRDOQ[1 - refId][1] >= minCost) &&
         (m_stateCostRDOQ[1 - refId][2] >= minCost) )
      {
        return 0;
      }
    }
    refId = 1 - refId;
  }
  return 1;
}

double IntraSearch::rateDistOptPLT(
  bool      runType,
  uint8_t   runIndex,
  bool      prevRunType,
  uint8_t   prevRunIndex,
  uint8_t   aboveRunIndex,
  bool&     prevCodedRunType,
  int&      prevCodedPos,
  int       scanPos,
  uint32_t  width,
  int       dist,
  int       indexMaxValue,
  const BinFracBits* IndexfracBits,
  const BinFracBits& TypefracBits)
{
  double rdCost = 0.0;
  bool identityFlag = !( (runType != prevRunType) || ( (runType == PLT_RUN_INDEX) && (runIndex != prevRunIndex) ) );

  if ( ( !identityFlag && runType == PLT_RUN_INDEX ) || scanPos == 0 ) // encode index value
  {
    uint8_t refIndex = (prevRunType == PLT_RUN_INDEX) ? prevRunIndex : aboveRunIndex;
    refIndex = (scanPos == 0) ? ( indexMaxValue + 1) : refIndex;
    if ( runIndex == refIndex )
    {
      rdCost = MAX_DOUBLE;
      return rdCost;
    }
    rdCost += m_pcRdCost->getLambda()*(m_truncBinBits[(runIndex > refIndex) ? runIndex - 1 : runIndex][(scanPos == 0) ? (indexMaxValue + 1) : indexMaxValue] << SCALE_BITS);
  }
  rdCost += m_indexError[runIndex][m_scanOrder[scanPos].idx] * (1 << SCALE_BITS);
  if (scanPos > 0)
  {
    rdCost += m_pcRdCost->getLambda()*( identityFlag ? (IndexfracBits[(dist < RUN_IDX_THRE) ? dist : RUN_IDX_THRE].intBits[1]) : (IndexfracBits[(dist < RUN_IDX_THRE) ? dist : RUN_IDX_THRE].intBits[0] ) );
  }
  if ( !identityFlag && scanPos >= width && prevRunType != PLT_RUN_COPY )
  {
    rdCost += m_pcRdCost->getLambda()*TypefracBits.intBits[runType];
  }
  if (!identityFlag || scanPos == 0)
  {
    prevCodedRunType = runType;
    prevCodedPos = scanPos;
  }
  return rdCost;
}
uint32_t IntraSearch::getEpExGolombNumBins(uint32_t symbol, uint32_t count)
{
  uint32_t numBins = 0;
  while (symbol >= (uint32_t)(1 << count))
  {
    numBins++;
    symbol -= 1 << count;
    count++;
  }
  numBins++;
  numBins += count;
  assert(numBins <= 32);
  return numBins;
}

uint32_t IntraSearch::getTruncBinBits(uint32_t symbol, uint32_t maxSymbol)
{
  uint32_t idxCodeBit = 0;
  uint32_t thresh;
  if (maxSymbol > 256)
  {
    uint32_t threshVal = 1 << 8;
    thresh = 8;
    while (threshVal <= maxSymbol)
    {
      thresh++;
      threshVal <<= 1;
    }
    thresh--;
  }
  else
  {
    thresh = g_tbMax[maxSymbol];
  }
  uint32_t uiVal = 1 << thresh;
  assert(uiVal <= maxSymbol);
  assert((uiVal << 1) > maxSymbol);
  assert(symbol < maxSymbol);
  uint32_t b = maxSymbol - uiVal;
  assert(b < uiVal);
  if (symbol < uiVal - b)
  {
    idxCodeBit = thresh;
  }
  else
  {
    idxCodeBit = thresh + 1;
  }
  return idxCodeBit;
}

void IntraSearch::initTBCTable(int bitDepth)
{
  for (uint32_t i = 0; i < m_symbolSize; i++)
  {
    memset(m_truncBinBits[i], 0, sizeof(uint16_t)*(m_symbolSize + 1));
  }
  for (uint32_t i = 0; i < (m_symbolSize + 1); i++)
  {
    for (uint32_t j = 0; j < i; j++)
    {
      m_truncBinBits[j][i] = getTruncBinBits(j, i);
    }
  }
  memset(m_escapeNumBins, 0, sizeof(uint16_t)*m_symbolSize);
  for (uint32_t i = 0; i < m_symbolSize; i++)
  {
    m_escapeNumBins[i] = getEpExGolombNumBins(i, 5);
  }
}

void IntraSearch::calcPixelPred(CodingStructure& cs, Partitioner& partitioner, uint32_t yPos, uint32_t xPos, ComponentID compBegin, uint32_t numComp)
{
  CodingUnit    &cu = *cs.getCU(partitioner.chType);
  TransformUnit &tu = *cs.getTU(partitioner.chType);
  bool lossless = (m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && cs.slice->isLossless());

  CPelBuf   orgBuf[3];
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    CompArea  area = cu.blocks[comp];
    if (m_pcEncCfg->getLmcs() && (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()))
    {
      orgBuf[comp] = cs.getPredBuf(area);
    }
    else
    {
      orgBuf[comp] = cs.getOrgBuf(area);
    }
  }

  int qp[3];
  int qpRem[3];
  int qpPer[3];
  int quantiserScale[3];
  int quantiserRightShift[3];
  int rightShiftOffset[3];
  int invquantiserRightShift[3];
  int add[3];
  if (!lossless)
  {
    for (uint32_t ch = compBegin; ch < (compBegin + numComp); ch++)
    {
      QpParam cQP(tu, ComponentID(ch));
      qp[ch]                     = cQP.Qp(true);
      qpRem[ch]                  = qp[ch] % 6;
      qpPer[ch]                  = qp[ch] / 6;
      quantiserScale[ch]         = g_quantScales[0][qpRem[ch]];
      quantiserRightShift[ch]    = QUANT_SHIFT + qpPer[ch];
      rightShiftOffset[ch]       = 1 << (quantiserRightShift[ch] - 1);
      invquantiserRightShift[ch] = IQUANT_SHIFT;
      add[ch]                    = 1 << (invquantiserRightShift[ch] - 1);
    }
  }

  uint32_t scaleX = getComponentScaleX(COMPONENT_Cb, cs.sps->getChromaFormatIdc());
  uint32_t scaleY = getComponentScaleY(COMPONENT_Cb, cs.sps->getChromaFormatIdc());
  for (uint32_t ch = compBegin; ch < (compBegin + numComp); ch++)
  {
    const int channelBitDepth = cu.cs->sps->getBitDepth(toChannelType((ComponentID)ch));
    CompArea  area = cu.blocks[ch];
    PelBuf    recBuf = cs.getRecoBuf(area);
    PLTescapeBuf escapeValue = tu.getescapeValue((ComponentID)ch);
    if (compBegin != COMPONENT_Y || ch == 0)
    {
      if (lossless)
      {
        escapeValue.at(xPos, yPos) = orgBuf[ch].at(xPos, yPos);
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
        recBuf.at(xPos, yPos)      = orgBuf[ch].at(xPos, yPos);
#else
        recBuf.at(xPos, yPos)      = escapeValue.at(xPos, yPos);
#endif
      }
      else
      {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
      escapeValue.at(xPos, yPos) = std::max<TCoeff>(0, ((orgBuf[ch].at(xPos, yPos) * quantiserScale[ch] + rightShiftOffset[ch]) >> quantiserRightShift[ch]));
      assert(escapeValue.at(xPos, yPos) < (TCoeff(1) << (channelBitDepth + 1)));
      TCoeff value = (((escapeValue.at(xPos, yPos)*g_invQuantScales[0][qpRem[ch]]) << qpPer[ch]) + add[ch]) >> invquantiserRightShift[ch];
      recBuf.at(xPos, yPos) = Pel(ClipBD<TCoeff>(value, channelBitDepth));//to be checked
#else
      escapeValue.at(xPos, yPos) = TCoeff(std::max<int>(0, ((orgBuf[ch].at(xPos, yPos) * quantiserScale[ch] + rightShiftOffset[ch]) >> quantiserRightShift[ch])));
      assert(escapeValue.at(xPos, yPos) < (1 << (channelBitDepth + 1)));
      recBuf.at(xPos, yPos) = (((escapeValue.at(xPos, yPos)*g_invQuantScales[0][qpRem[ch]]) << qpPer[ch]) + add[ch]) >> invquantiserRightShift[ch];
      recBuf.at(xPos, yPos) = Pel(ClipBD<int>(recBuf.at(xPos, yPos), channelBitDepth));//to be checked
#endif
      }
    }
    else if (compBegin == COMPONENT_Y && ch > 0 && yPos % (1 << scaleY) == 0 && xPos % (1 << scaleX) == 0)
    {
      uint32_t yPosC = yPos >> scaleY;
      uint32_t xPosC = xPos >> scaleX;
      if (lossless)
      {
        escapeValue.at(xPosC, yPosC) = orgBuf[ch].at(xPosC, yPosC);
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
        recBuf.at(xPosC, yPosC)      = orgBuf[ch].at(xPosC, yPosC);
#else
        recBuf.at(xPosC, yPosC)      = escapeValue.at(xPosC, yPosC);
#endif
      }
      else
      {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
        escapeValue.at(xPosC, yPosC) = std::max<TCoeff>(
          0, ((orgBuf[ch].at(xPosC, yPosC) * quantiserScale[ch] + rightShiftOffset[ch]) >> quantiserRightShift[ch]));
        assert(escapeValue.at(xPosC, yPosC) < (TCoeff(1) << (channelBitDepth + 1)));
        TCoeff value = (((escapeValue.at(xPosC, yPosC) * g_invQuantScales[0][qpRem[ch]]) << qpPer[ch]) + add[ch])
                       >> invquantiserRightShift[ch];
        recBuf.at(xPosC, yPosC) = Pel(ClipBD<TCoeff>(value, channelBitDepth));   // to be checked
#else
        escapeValue.at(xPosC, yPosC) = TCoeff(std::max<int>(
          0, ((orgBuf[ch].at(xPosC, yPosC) * quantiserScale[ch] + rightShiftOffset[ch]) >> quantiserRightShift[ch])));
        assert(escapeValue.at(xPosC, yPosC) < (1 << (channelBitDepth + 1)));
        recBuf.at(xPosC, yPosC) =
          (((escapeValue.at(xPosC, yPosC) * g_invQuantScales[0][qpRem[ch]]) << qpPer[ch]) + add[ch])
          >> invquantiserRightShift[ch];
        recBuf.at(xPosC, yPosC) = Pel(ClipBD<int>(recBuf.at(xPosC, yPosC), channelBitDepth));   // to be checked
#endif
      }
    }
  }
}

void IntraSearch::derivePLTLossy(CodingStructure& cs, Partitioner& partitioner, ComponentID compBegin, uint32_t numComp)
{
  CodingUnit &cu = *cs.getCU(partitioner.chType);
  const int channelBitDepth_L = cs.sps->getBitDepth(CHANNEL_TYPE_LUMA);
  const int channelBitDepth_C = cs.sps->getBitDepth(CHANNEL_TYPE_CHROMA);

  bool lossless        = (m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && cs.slice->isLossless());
  int  pcmShiftRight_L = (channelBitDepth_L - PLT_ENCBITDEPTH);
  int  pcmShiftRight_C = (channelBitDepth_C - PLT_ENCBITDEPTH);
  if (lossless)
  {
    pcmShiftRight_L = 0;
    pcmShiftRight_C = 0;
  }
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  int maxPltSize = cu.isSepTree() ? MAXPLTSIZE_DUALTREE : MAXPLTSIZE;
#else
  int maxPltSize = CS::isDualITree(cs) ? MAXPLTSIZE_DUALTREE : MAXPLTSIZE;
#endif
  uint32_t height = cu.block(compBegin).height;
  uint32_t width = cu.block(compBegin).width;

  CPelBuf   orgBuf[3];
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    CompArea  area = cu.blocks[comp];
    if (m_pcEncCfg->getLmcs() && (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()))
    {
      orgBuf[comp] = cs.getPredBuf(area);
    }
    else
    {
      orgBuf[comp] = cs.getOrgBuf(area);
    }
  }

  TransformUnit &tu = *cs.getTU(partitioner.chType);
  QpParam cQP(tu, compBegin);
  int qp = cQP.Qp(true) - 12;
  qp = (qp < 0) ? 0 : ((qp > 56) ? 56 : qp);
  int errorLimit = g_paletteQuant[qp];
  if (lossless)
  {
    errorLimit = 0;
  }
  uint32_t totalSize = height*width;
  SortingElement *pelList = new SortingElement[totalSize];
  SortingElement  element;
  SortingElement *pelListSort = new SortingElement[MAXPLTSIZE + 1];
  uint32_t dictMaxSize = maxPltSize;
  uint32_t idx = 0;
  int last = -1;

  uint32_t scaleX = getComponentScaleX(COMPONENT_Cb, cs.sps->getChromaFormatIdc());
  uint32_t scaleY = getComponentScaleY(COMPONENT_Cb, cs.sps->getChromaFormatIdc());
  for (uint32_t y = 0; y < height; y++)
  {
    for (uint32_t x = 0; x < width; x++)
    {
      uint32_t org[3], pX, pY;
      for (int comp = compBegin; comp < (compBegin + numComp); comp++)
      {
        pX = (comp > 0 && compBegin == COMPONENT_Y) ? (x >> scaleX) : x;
        pY = (comp > 0 && compBegin == COMPONENT_Y) ? (y >> scaleY) : y;
        org[comp] = orgBuf[comp].at(pX, pY);
      }
      element.setAll(org, compBegin, numComp);

      ComponentID tmpCompBegin = compBegin;
      int tmpNumComp = numComp;
      if( cs.sps->getChromaFormatIdc() != CHROMA_444 &&
          numComp == 3 &&
         (x != ((x >> scaleX) << scaleX) || (y != ((y >> scaleY) << scaleY))) )
      {
        tmpCompBegin = COMPONENT_Y;
        tmpNumComp   = 1;
      }
      int besti = last, bestSAD = (last == -1) ? MAX_UINT : pelList[last].getSAD(element, cs.sps->getBitDepths(), tmpCompBegin, tmpNumComp, lossless);
      if (lossless)
      {
        if (bestSAD)
        {
          for (int i = idx - 1; i >= 0; i--)
          {
            uint32_t sad = pelList[i].getSAD(element, cs.sps->getBitDepths(), tmpCompBegin, tmpNumComp, lossless);
            if (sad == 0)
            {
              bestSAD = sad;
              besti   = i;
              break;
            }
          }
        }
      }
      else
      {
        if (bestSAD)
        {
          for (int i = idx - 1; i >= 0; i--)
          {
            uint32_t sad = pelList[i].getSAD(element, cs.sps->getBitDepths(), tmpCompBegin, tmpNumComp, lossless);
            if (sad < bestSAD)
            {
              bestSAD = sad;
              besti   = i;
              if (!sad)
              {
                break;
              }
            }
          }
        }
      }
      if (besti >= 0 && pelList[besti].almostEqualData(element, errorLimit, cs.sps->getBitDepths(), tmpCompBegin, tmpNumComp, lossless))
      {
        pelList[besti].addElement(element, tmpCompBegin, tmpNumComp);
        last = besti;
      }
      else
      {
        pelList[idx].copyDataFrom(element, tmpCompBegin, tmpNumComp);
        for (int comp = tmpCompBegin; comp < (tmpCompBegin + tmpNumComp); comp++)
        {
          pelList[idx].setCnt(1, comp);
        }
        last = idx;
        idx++;
      }
    }
  }

  if( cs.sps->getChromaFormatIdc() != CHROMA_444 && numComp == 3 )
  {
    for( int i = 0; i < idx; i++ )
    {
      pelList[i].setCnt( pelList[i].getCnt(COMPONENT_Y) + (pelList[i].getCnt(COMPONENT_Cb) >> 2), MAX_NUM_COMPONENT);
    }
  }
  else
  {
    if( compBegin == 0 )
    {
      for( int i = 0; i < idx; i++ )
      {
        pelList[i].setCnt(pelList[i].getCnt(COMPONENT_Y), COMPONENT_Cb);
        pelList[i].setCnt(pelList[i].getCnt(COMPONENT_Y), COMPONENT_Cr);
        pelList[i].setCnt(pelList[i].getCnt(COMPONENT_Y), MAX_NUM_COMPONENT);
      }
    }
    else
    {
      for( int i = 0; i < idx; i++ )
      {
        pelList[i].setCnt(pelList[i].getCnt(COMPONENT_Cb), COMPONENT_Y);
        pelList[i].setCnt(pelList[i].getCnt(COMPONENT_Cb), MAX_NUM_COMPONENT);
      }
    }
  }

  for (int i = 0; i < dictMaxSize; i++)
  {
    pelListSort[i].setCnt(0, COMPONENT_Y);
    pelListSort[i].setCnt(0, COMPONENT_Cb);
    pelListSort[i].setCnt(0, COMPONENT_Cr);
    pelListSort[i].setCnt(0, MAX_NUM_COMPONENT);
    pelListSort[i].resetAll(compBegin, numComp);
  }

  //bubble sorting
  dictMaxSize = 1;
  for (int i = 0; i < idx; i++)
  {
    if( pelList[i].getCnt(MAX_NUM_COMPONENT) > pelListSort[dictMaxSize - 1].getCnt(MAX_NUM_COMPONENT) )
    {
      int j;
      for (j = dictMaxSize; j > 0; j--)
      {
        if (pelList[i].getCnt(MAX_NUM_COMPONENT) > pelListSort[j - 1].getCnt(MAX_NUM_COMPONENT))
        {
          pelListSort[j].copyAllFrom(pelListSort[j - 1], compBegin, numComp);
          dictMaxSize = std::min(dictMaxSize + 1, (uint32_t)maxPltSize);
        }
        else
        {
          break;
        }
      }
      pelListSort[j].copyAllFrom(pelList[i], compBegin, numComp);
    }
  }

  uint32_t paletteSize = 0;
  uint64_t numColorBits = 0;
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    numColorBits += (comp > 0) ? channelBitDepth_C : channelBitDepth_L;
  }
  const int plt_lambda_shift = (compBegin > 0) ? pcmShiftRight_C : pcmShiftRight_L;
  double    bitCost          = m_pcRdCost->getLambda() / (double) (1 << (2 * plt_lambda_shift)) * numColorBits;
  bool   reuseflag[MAXPLTPREDSIZE] = { false };
  int    run;
  double reuseflagCost;
  for (int i = 0; i < maxPltSize; i++)
  {
    if( pelListSort[i].getCnt(MAX_NUM_COMPONENT) )
    {
      ComponentID tmpCompBegin = compBegin;
      int tmpNumComp = numComp;
      if( cs.sps->getChromaFormatIdc() != CHROMA_444 && numComp == 3 && pelListSort[i].getCnt(COMPONENT_Cb) == 0 )
      {
        tmpCompBegin = COMPONENT_Y;
        tmpNumComp   = 1;
      }

      for( int comp = tmpCompBegin; comp < (tmpCompBegin + tmpNumComp); comp++ )
      {
        int half = pelListSort[i].getCnt(comp) >> 1;
        cu.curPLT[comp][paletteSize] = (pelListSort[i].getSumData(comp) + half) / pelListSort[i].getCnt(comp);
      }

      int best = -1;
      if( errorLimit )
      {
        double pal[MAX_NUM_COMPONENT], err = 0.0, bestCost = 0.0;
        for( int comp = tmpCompBegin; comp < (tmpCompBegin + tmpNumComp); comp++ )
        {
          pal[comp] = pelListSort[i].getSumData(comp) / (double)pelListSort[i].getCnt(comp);
          err = pal[comp] - cu.curPLT[comp][paletteSize];
          if( isChroma((ComponentID) comp) )
          {
            bestCost += (err * err * PLT_CHROMA_WEIGHTING) / (1 << (2 * pcmShiftRight_C)) * pelListSort[i].getCnt(comp);
          }
          else
          {
            bestCost += (err * err) / (1 << (2 * pcmShiftRight_L)) * pelListSort[i].getCnt(comp);
          }
        }
        bestCost += bitCost;

        for( int t = 0; t < cs.prevPLT.curPLTSize[compBegin]; t++ )
        {
          double cost = 0.0;
          for( int comp = tmpCompBegin; comp < (tmpCompBegin + tmpNumComp); comp++ )
          {
            err = pal[comp] - cs.prevPLT.curPLT[comp][t];
            if( isChroma((ComponentID) comp) )
            {
              cost += (err * err * PLT_CHROMA_WEIGHTING) / (1 << (2 * pcmShiftRight_C)) * pelListSort[i].getCnt(comp);
            }
            else
            {
              cost += (err * err) / (1 << (2 * pcmShiftRight_L)) * pelListSort[i].getCnt(comp);
            }
          }
          run = 0;
          for (int t2 = t; t2 >= 0; t2--)
          {
            if (!reuseflag[t2])
            {
              run++;
            }
            else
            {
              break;
            }
          }
          reuseflagCost = m_pcRdCost->getLambda() / (double)(1 << (2 * plt_lambda_shift)) * getEpExGolombNumBins(run ? run + 1 : run, 0);
          cost += reuseflagCost;

          if( cost < bestCost )
          {
            best = t;
            bestCost = cost;
          }
        }
        if( best != -1 )
        {
          for( int comp = tmpCompBegin; comp < (tmpCompBegin + tmpNumComp); comp++ )
          {
            cu.curPLT[comp][paletteSize] = cs.prevPLT.curPLT[comp][best];
          }
          reuseflag[best] = true;
        }
      }

      bool duplicate = false;
      if( pelListSort[i].getCnt(MAX_NUM_COMPONENT) == 1 && best == -1 )
      {
        duplicate = true;
      }
      else
      {
        for( int t = 0; t < paletteSize; t++ )
        {
          bool duplicateTmp = true;
          for( int comp = tmpCompBegin; comp < (tmpCompBegin + tmpNumComp); comp++ )
          {
            duplicateTmp = duplicateTmp && (cu.curPLT[comp][paletteSize] == cu.curPLT[comp][t]);
          }
          if( duplicateTmp )
          {
            duplicate = true;
            break;
          }
        }
      }
      if( !duplicate )
      {
        if( cs.sps->getChromaFormatIdc() != CHROMA_444 && numComp == 3 && pelListSort[i].getCnt(COMPONENT_Cb) == 0 )
        {
          if( best != -1 )
          {
            cu.curPLT[COMPONENT_Cb][paletteSize] = cs.prevPLT.curPLT[COMPONENT_Cb][best];
            cu.curPLT[COMPONENT_Cr][paletteSize] = cs.prevPLT.curPLT[COMPONENT_Cr][best];
          }
          else
          {
            cu.curPLT[COMPONENT_Cb][paletteSize] = 1 << (channelBitDepth_C - 1);
            cu.curPLT[COMPONENT_Cr][paletteSize] = 1 << (channelBitDepth_C - 1);
          }
        }
        paletteSize++;
      }
    }
    else
    {
      break;
    }
  }
  cu.curPLTSize[compBegin] = paletteSize;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( cu.isLocalSepTree() )
  {
    cu.curPLTSize[COMPONENT_Y] = paletteSize;
  }
#endif
  delete[] pelList;
  delete[] pelListSort;
}
// -------------------------------------------------------------------------------------------------------------------
// Intra search
// -------------------------------------------------------------------------------------------------------------------

void IntraSearch::xEncIntraHeader( CodingStructure &cs, Partitioner &partitioner, const bool &bLuma, const bool &bChroma, const int subTuIdx )
{
  CodingUnit &cu = *cs.getCU( partitioner.chType );

  if (bLuma)
  {
    bool isFirst = cu.ispMode ? subTuIdx == 0 : partitioner.currArea().lumaPos() == cs.area.lumaPos();

    // CU header
    if( isFirst )
    {
      if ((!cs.slice->isIntra() || cs.slice->getSPS()->getIBCFlag() || cs.slice->getSPS()->getPLTMode())
          && cu.Y().valid())
      {
        m_CABACEstimator->cu_skip_flag( cu );
        m_CABACEstimator->pred_mode   ( cu );
      }
#if ENABLE_DIMD
      m_CABACEstimator->cu_dimd_flag(cu);
#endif
      if (CU::isPLT(cu))
      {
        return;
      }
    }

    PredictionUnit &pu = *cs.getPU(partitioner.currArea().lumaPos(), partitioner.chType);

    // luma prediction mode
    if (isFirst)
    {
      if ( !cu.Y().valid())
      {
        m_CABACEstimator->pred_mode( cu );
      }
      m_CABACEstimator->bdpcm_mode( cu, COMPONENT_Y );
      m_CABACEstimator->intra_luma_pred_mode( pu );
    }
  }

  if (bChroma)
  {
    bool isFirst = partitioner.currArea().Cb().valid() && partitioner.currArea().chromaPos() == cs.area.chromaPos();

    PredictionUnit &pu = *cs.getPU( partitioner.currArea().chromaPos(), CHANNEL_TYPE_CHROMA );

    if( isFirst )
    {
      m_CABACEstimator->bdpcm_mode( cu, ComponentID(CHANNEL_TYPE_CHROMA) );
      m_CABACEstimator->intra_chroma_pred_mode( pu );
    }
  }
}

void IntraSearch::xEncSubdivCbfQT( CodingStructure &cs, Partitioner &partitioner, const bool &bLuma, const bool &bChroma, const int subTuIdx, const PartSplit ispType )
{
  const UnitArea &currArea = partitioner.currArea();
          int subTuCounter = subTuIdx;
  TransformUnit &currTU = *cs.getTU( currArea.blocks[partitioner.chType], partitioner.chType, subTuCounter );
  CodingUnit    &currCU = *currTU.cu;
  uint32_t currDepth           = partitioner.currTrDepth;

  const bool subdiv        = currTU.depth > currDepth;
  ComponentID compID = partitioner.chType == CHANNEL_TYPE_LUMA ? COMPONENT_Y : COMPONENT_Cb;

  if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
  {
    CHECK( !subdiv, "TU split implied" );
  }
  else
  {
    CHECK( subdiv && !currCU.ispMode && isLuma( compID ), "No TU subdivision is allowed with QTBT" );
  }

  if (bChroma)
  {
    const bool chromaCbfISP = currArea.blocks[COMPONENT_Cb].valid() && currCU.ispMode && !subdiv;
    if ( !currCU.ispMode || chromaCbfISP )
    {
      const uint32_t numberValidComponents = getNumberValidComponents(currArea.chromaFormat);
      const uint32_t cbfDepth              = (chromaCbfISP ? currDepth - 1 : currDepth);

      for (uint32_t ch = COMPONENT_Cb; ch < numberValidComponents; ch++)
      {
        const ComponentID compID = ComponentID(ch);

        if (currDepth == 0 || TU::getCbfAtDepth(currTU, compID, currDepth - 1) || chromaCbfISP)
        {
          const bool prevCbf = (compID == COMPONENT_Cr ? TU::getCbfAtDepth(currTU, COMPONENT_Cb, currDepth) : false);
          m_CABACEstimator->cbf_comp(cs, TU::getCbfAtDepth(currTU, compID, currDepth), currArea.blocks[compID],
                                     cbfDepth, prevCbf);
        }
      }
    }
  }

  if (subdiv)
  {
    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
    {
      partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
    }
    else if( currCU.ispMode && isLuma( compID ) )
    {
      partitioner.splitCurrArea( ispType, cs );
    }
    else
    {
      THROW("Cannot perform an implicit split!");
    }

    do
    {
      xEncSubdivCbfQT( cs, partitioner, bLuma, bChroma, subTuCounter, ispType );
      subTuCounter += subTuCounter != -1 ? 1 : 0;
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();
  }
  else
  {
    //===== Cbfs =====
    if (bLuma)
    {
      bool previousCbf       = false;
      bool lastCbfIsInferred = false;
      if( ispType != TU_NO_ISP )
      {
        bool rootCbfSoFar = false;
        uint32_t nTus = currCU.ispMode == HOR_INTRA_SUBPARTITIONS ? currCU.lheight() >> floorLog2(currTU.lheight()) : currCU.lwidth() >> floorLog2(currTU.lwidth());
        if( subTuCounter == nTus - 1 )
        {
          TransformUnit* tuPointer = currCU.firstTU;
          for( int tuIdx = 0; tuIdx < nTus - 1; tuIdx++ )
          {
            rootCbfSoFar |= TU::getCbfAtDepth( *tuPointer, COMPONENT_Y, currDepth );
            tuPointer = tuPointer->next;
          }
          if( !rootCbfSoFar )
          {
            lastCbfIsInferred = true;
          }
        }
        if( !lastCbfIsInferred )
        {
          previousCbf = TU::getPrevTuCbfAtDepth( currTU, COMPONENT_Y, partitioner.currTrDepth );
        }
      }
      if( !lastCbfIsInferred )
      {
        m_CABACEstimator->cbf_comp( cs, TU::getCbfAtDepth( currTU, COMPONENT_Y, currDepth ), currTU.Y(), currTU.depth, previousCbf, currCU.ispMode );
      }
    }
  }
}

void IntraSearch::xEncCoeffQT( CodingStructure &cs, Partitioner &partitioner, const ComponentID compID, const int subTuIdx, const PartSplit ispType, CUCtx* cuCtx )
{
  const UnitArea &currArea  = partitioner.currArea();

       int subTuCounter     = subTuIdx;
  TransformUnit &currTU     = *cs.getTU( currArea.blocks[partitioner.chType], partitioner.chType, subTuIdx );
  uint32_t      currDepth       = partitioner.currTrDepth;
  const bool subdiv         = currTU.depth > currDepth;

  if (subdiv)
  {
    if (partitioner.canSplit(TU_MAX_TR_SPLIT, cs))
    {
      partitioner.splitCurrArea(TU_MAX_TR_SPLIT, cs);
    }
    else if( currTU.cu->ispMode )
    {
      partitioner.splitCurrArea( ispType, cs );
    }
    else
    {
      THROW("Implicit TU split not available!");
    }

    do
    {
      xEncCoeffQT( cs, partitioner, compID, subTuCounter, ispType, cuCtx );
      subTuCounter += subTuCounter != -1 ? 1 : 0;
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();
  }
  else
  {
    if (currArea.blocks[compID].valid())
    {
      if (compID == COMPONENT_Cr)
      {
        const int cbfMask = (TU::getCbf(currTU, COMPONENT_Cb) ? 2 : 0) + (TU::getCbf(currTU, COMPONENT_Cr) ? 1 : 0);
        m_CABACEstimator->joint_cb_cr(currTU, cbfMask);
      }
      if (TU::getCbf(currTU, compID))
      {
        if (isLuma(compID))
        {
          m_CABACEstimator->residual_coding(currTU, compID, cuCtx);
          m_CABACEstimator->mts_idx(*currTU.cu, cuCtx);
        }
        else
        {
          m_CABACEstimator->residual_coding(currTU, compID);
        }
      }
    }
  }
}

uint64_t IntraSearch::xGetIntraFracBitsQT( CodingStructure &cs, Partitioner &partitioner, const bool &bLuma, const bool &bChroma, const int subTuIdx, const PartSplit ispType, CUCtx* cuCtx )
{
  m_CABACEstimator->resetBits();

  xEncIntraHeader( cs, partitioner, bLuma, bChroma, subTuIdx );
  xEncSubdivCbfQT( cs, partitioner, bLuma, bChroma, subTuIdx, ispType );

  if( bLuma )
  {
    xEncCoeffQT( cs, partitioner, COMPONENT_Y, subTuIdx, ispType, cuCtx );
  }
  if( bChroma )
  {
    xEncCoeffQT( cs, partitioner, COMPONENT_Cb, subTuIdx, ispType );
    xEncCoeffQT( cs, partitioner, COMPONENT_Cr, subTuIdx, ispType );
  }

  CodingUnit& cu = *cs.getCU(partitioner.chType);
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if ( cuCtx && bLuma && cu.isSepTree() && ( !cu.ispMode || ( cu.lfnstIdx && subTuIdx == 0 ) || ( !cu.lfnstIdx && subTuIdx == m_ispTestedModes[cu.lfnstIdx].numTotalParts[cu.ispMode - 1] - 1 ) ) )
#else
  if (cuCtx && bLuma && CS::isDualITree(cs) && (!cu.ispMode || (cu.lfnstIdx && subTuIdx == 0) || (!cu.lfnstIdx && subTuIdx == m_ispTestedModes[cu.lfnstIdx].numTotalParts[cu.ispMode - 1] - 1)))
#endif
  {
    m_CABACEstimator->residual_lfnst_mode(cu, *cuCtx);
  }

  uint64_t fracBits = m_CABACEstimator->getEstFracBits();
  return fracBits;
}

uint64_t IntraSearch::xGetIntraFracBitsQTSingleChromaComponent( CodingStructure &cs, Partitioner &partitioner, const ComponentID compID )
{
  m_CABACEstimator->resetBits();

  if( compID == COMPONENT_Cb )
  {
    //intra mode coding
    PredictionUnit &pu = *cs.getPU( partitioner.currArea().lumaPos(), partitioner.chType );
    m_CABACEstimator->intra_chroma_pred_mode( pu );
    //xEncIntraHeader(cs, partitioner, false, true);
  }
  CHECK( partitioner.currTrDepth != 1, "error in the depth!" );
  const UnitArea &currArea = partitioner.currArea();

  TransformUnit &currTU = *cs.getTU( currArea.blocks[partitioner.chType], partitioner.chType );

  //cbf coding
  const bool prevCbf = ( compID == COMPONENT_Cr ? TU::getCbfAtDepth( currTU, COMPONENT_Cb, partitioner.currTrDepth ) : false );
  m_CABACEstimator->cbf_comp( cs, TU::getCbfAtDepth( currTU, compID, partitioner.currTrDepth ), currArea.blocks[compID], partitioner.currTrDepth - 1, prevCbf );
  //coeffs coding and cross comp coding
  if( TU::getCbf( currTU, compID ) )
  {
    m_CABACEstimator->residual_coding( currTU, compID );
  }

  uint64_t fracBits = m_CABACEstimator->getEstFracBits();
  return fracBits;
}

uint64_t IntraSearch::xGetIntraFracBitsQTChroma(TransformUnit& currTU, const ComponentID &compID)
{
  m_CABACEstimator->resetBits();
  // Include Cbf and jointCbCr flags here as we make decisions across components
  CodingStructure &cs = *currTU.cs;

  if ( currTU.jointCbCr )
  {
    const int cbfMask = ( TU::getCbf( currTU, COMPONENT_Cb ) ? 2 : 0 ) + ( TU::getCbf( currTU, COMPONENT_Cr ) ? 1 : 0 );
    m_CABACEstimator->cbf_comp( cs, cbfMask>>1, currTU.blocks[ COMPONENT_Cb ], currTU.depth, false );
    m_CABACEstimator->cbf_comp( cs, cbfMask &1, currTU.blocks[ COMPONENT_Cr ], currTU.depth, cbfMask>>1 );
    if( cbfMask )
    {
      m_CABACEstimator->joint_cb_cr( currTU, cbfMask );
    }
    if( cbfMask >> 1 )
    {
      m_CABACEstimator->residual_coding( currTU, COMPONENT_Cb );
    }
    if( cbfMask & 1 )
    {
      m_CABACEstimator->residual_coding( currTU, COMPONENT_Cr );
    }
  }
  else
  {
    if ( compID == COMPONENT_Cb )
    {
      m_CABACEstimator->cbf_comp( cs, TU::getCbf( currTU, compID ), currTU.blocks[ compID ], currTU.depth, false );
    }
    else
    {
      const bool cbCbf    = TU::getCbf( currTU, COMPONENT_Cb );
      const bool crCbf    = TU::getCbf( currTU, compID );
      const int  cbfMask  = ( cbCbf ? 2 : 0 ) + ( crCbf ? 1 : 0 );
      m_CABACEstimator->cbf_comp( cs, crCbf, currTU.blocks[ compID ], currTU.depth, cbCbf );
      m_CABACEstimator->joint_cb_cr( currTU, cbfMask );
    }
  }

  if( !currTU.jointCbCr && TU::getCbf( currTU, compID ) )
  {
    m_CABACEstimator->residual_coding( currTU, compID );
  }

  uint64_t fracBits = m_CABACEstimator->getEstFracBits();
  return fracBits;
}
#if JVET_W0103_INTRA_MTS
void IntraSearch::xSelectAMTForFullRD(TransformUnit &tu)
{
  if (!tu.blocks[COMPONENT_Y].valid())
  {
    return;
  }

  if (!tu.cu->mtsFlag)
  {
    return;
  }

  CodingStructure &cs = *tu.cs;
  m_pcRdCost->setChromaFormat(cs.sps->getChromaFormatIdc());

  const CompArea      &area = tu.blocks[COMPONENT_Y];

  const ChannelType    chType = toChannelType(COMPONENT_Y);


  PelBuf         piOrg = cs.getOrgBuf(area);
  PelBuf         piPred = cs.getPredBuf(area);
  PelBuf         piResi = cs.getResiBuf(area);


  const PredictionUnit &pu = *cs.getPU(area.pos(), chType);

  //===== init availability pattern =====

  PelBuf sharedPredTS(m_pSharedPredTransformSkip[COMPONENT_Y], area);
  initIntraPatternChType(*tu.cu, area);

  //===== get prediction signal =====
  if (PU::isMIP(pu, chType))
  {
    initIntraMip(pu, area);
    predIntraMip(COMPONENT_Y, piPred, pu);
  }
  else
  {
    predIntraAng(COMPONENT_Y, piPred, pu);
  }


  // save prediction
  sharedPredTS.copyFrom(piPred);

  const Slice           &slice = *cs.slice;
  //===== get residual signal =====
  piResi.copyFrom(piOrg);
  if (slice.getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
  {
    piResi.rspSignal(m_pcReshape->getFwdLUT());
    piResi.subtract(piPred);
  }
  else
  {
    piResi.subtract(piPred);
  }
  // do transform and calculate Coeff AbsSum for all MTS candidates
#if JVET_Y0142_ADAPT_INTRA_MTS
  int nCands = MTS_NCANDS[2];
  if (m_coeffAbsSumDCT2 >= 0 && m_coeffAbsSumDCT2 <= MTS_TH_COEFF[0])
  {
    nCands = MTS_NCANDS[0];
  }
  else if(m_coeffAbsSumDCT2 > MTS_TH_COEFF[0] && m_coeffAbsSumDCT2 <= MTS_TH_COEFF[1])
  {
    nCands = MTS_NCANDS[1];
  }
  std::vector<std::pair<int, uint64_t>> coeffAbsSum(nCands);
  for (int i = 0; i < nCands; i++)
#else
  std::vector<std::pair<int, uint64_t>> coeffAbsSum(4);
  for (int i = 0; i < 4; i++)
#endif
  {
    tu.mtsIdx[0] = i + MTS_DST7_DST7;
    uint64_t AbsSum = m_pcTrQuant->transformNxN(tu);
    coeffAbsSum[i] = { i, AbsSum };
  }
  std::stable_sort(coeffAbsSum.begin(), coeffAbsSum.end(), [](const std::pair<int, uint64_t> & l, const std::pair<int, uint64_t> & r) {return l.second < r.second; });
#if JVET_Y0142_ADAPT_INTRA_MTS
  for (int i = 0; i < nCands; i++)
#else
  for (int i = 0; i < 4; i++)
#endif
  {
    m_testAMTForFullRD[i] = coeffAbsSum[i].first;
  }
#if JVET_Y0142_ADAPT_INTRA_MTS
  m_numCandAMTForFullRD = nCands;
#else
  m_numCandAMTForFullRD = 4;
#endif
#if !JVET_Y0142_ADAPT_INTRA_MTS
  if (m_pcEncCfg->getUseFastLFNST())
  {
    double skipThreshold = 1.0 + 1.0 / sqrt((double)(area.width*area.height));
    skipThreshold = std::max(skipThreshold, 1.03);
    for (int i = 1; i < m_numCandAMTForFullRD; i++)
    {
      if (coeffAbsSum[i].second > skipThreshold * coeffAbsSum[0].second)
      {
        m_numCandAMTForFullRD = i;
        break;
      }
    }
  }
#endif
}
#endif
void IntraSearch::xIntraCodingTUBlock(TransformUnit &tu, const ComponentID &compID, Distortion& ruiDist, const int &default0Save1Load2, uint32_t* numSig, std::vector<TrMode>* trModes, const bool loadTr)
{
  if (!tu.blocks[compID].valid())
  {
    return;
  }

  CodingStructure &cs                       = *tu.cs;
  m_pcRdCost->setChromaFormat(cs.sps->getChromaFormatIdc());

  const CompArea      &area                 = tu.blocks[compID];
  const SPS           &sps                  = *cs.sps;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
  const PPS           &pps                  = *cs.pps;
#endif

  const ChannelType    chType               = toChannelType(compID);
  const int            bitDepth             = sps.getBitDepth(chType);

  PelBuf         piOrg                      = cs.getOrgBuf    (area);
  PelBuf         piPred                     = cs.getPredBuf   (area);
  PelBuf         piResi                     = cs.getResiBuf   (area);
  PelBuf         piReco                     = cs.getRecoBuf   (area);

#if JVET_AB0061_ITMP_BV_FOR_IBC
  PredictionUnit &pu = *cs.getPU(area.pos(), chType);
#else
  const PredictionUnit &pu                  = *cs.getPU(area.pos(), chType);
#endif
  const uint32_t           uiChFinalMode        = PU::getFinalIntraMode(pu, chType);

  //===== init availability pattern =====
  CHECK( tu.jointCbCr && compID == COMPONENT_Cr, "wrong combination of compID and jointCbCr" );
  bool jointCbCr = tu.jointCbCr && compID == COMPONENT_Cb;

  if (compID == COMPONENT_Y)
  {
    PelBuf sharedPredTS( m_pSharedPredTransformSkip[compID], area );
    if( default0Save1Load2 != 2 )
    {
      bool predRegDiffFromTB = CU::isPredRegDiffFromTB(*tu.cu, compID);
      bool firstTBInPredReg = CU::isFirstTBInPredReg(*tu.cu, compID, area);
      CompArea areaPredReg(COMPONENT_Y, tu.chromaFormat, area);
      if (tu.cu->ispMode && isLuma(compID))
      {
        if (predRegDiffFromTB)
        {
          if (firstTBInPredReg)
          {
            CU::adjustPredArea(areaPredReg);
            initIntraPatternChTypeISP(*tu.cu, areaPredReg, piReco);
          }
        }
        else
        {
          initIntraPatternChTypeISP(*tu.cu, area, piReco);
        }
      }
      else
      {
        initIntraPatternChType(*tu.cu, area);
      }

      //===== get prediction signal =====
      if(compID != COMPONENT_Y && !tu.cu->bdpcmModeChroma && PU::isLMCMode(uiChFinalMode))
      {
        xGetLumaRecPixels( pu, area );
        predIntraChromaLM( compID, piPred, pu, area, uiChFinalMode );
      }
      else
      {
#if JVET_V0130_INTRA_TMP
        if( PU::isTmp( pu, chType ) )
        {
          int foundCandiNum;
#if JVET_W0069_TMP_BOUNDARY
          RefTemplateType tempType = getRefTemplateType( *(tu.cu), tu.cu->blocks[COMPONENT_Y] );
          if( tempType != NO_TEMPLATE )
          {
            getTargetTemplate( tu.cu, pu.lwidth(), pu.lheight(), tempType );
            candidateSearchIntra( tu.cu, pu.lwidth(), pu.lheight(), tempType );
            generateTMPrediction( piPred.buf, piPred.stride, pu.lwidth(), pu.lheight(), foundCandiNum );
#if JVET_AB0061_ITMP_BV_FOR_IBC
            pu.interDir               = 1;             // use list 0 for IBC mode
            pu.refIdx[REF_PIC_LIST_0] = MAX_NUM_REF;   // last idx in the list
            pu.mv->set(m_tempLibFast.getX() << MV_FRACTIONAL_BITS_INTERNAL, m_tempLibFast.getY() << MV_FRACTIONAL_BITS_INTERNAL);
            pu.bv.set(m_tempLibFast.getX(), m_tempLibFast.getY());
#endif
          }
          else
          {
            foundCandiNum = 1;
            generateTmDcPrediction( piPred.buf, piPred.stride, pu.lwidth(), pu.lheight(), 1 << (tu.cu->cs->sps->getBitDepth( CHANNEL_TYPE_LUMA ) - 1) );
#if JVET_AB0061_ITMP_BV_FOR_IBC
            pu.interDir               = 1;             // use list 0 for IBC mode
            pu.refIdx[REF_PIC_LIST_0] = MAX_NUM_REF;   // last idx in the list
            pu.mv->set(0, 0);
            pu.bv.set(0, 0);
#endif
          }
#else
          getTargetTemplate( tu.cu, pu.lwidth(), pu.lheight() );
          candidateSearchIntra( tu.cu, pu.lwidth(), pu.lheight() );
          generateTMPrediction( piPred.buf, piPred.stride, pu.lwidth(), pu.lheight(), foundCandiNum );
#endif
          CHECK( foundCandiNum < 1, "" );
        }
        else if( PU::isMIP( pu, chType ) )
#else
        if( PU::isMIP( pu, chType ) )
#endif
        {
          initIntraMip( pu, area );
          predIntraMip( compID, piPred, pu );
        }
        else
        {
          if (predRegDiffFromTB)
          {
            if (firstTBInPredReg)
            {
              PelBuf piPredReg = cs.getPredBuf(areaPredReg);
              predIntraAng(compID, piPredReg, pu);
            }
          }
          else
          {
            predIntraAng(compID, piPred, pu);
          }
#if JVET_Z0050_DIMD_CHROMA_FUSION
          if (compID != COMPONENT_Y && pu.isChromaFusion)
          {
            geneChromaFusionPred(compID, piPred, pu);
          }
#endif
        }
      }

      // save prediction
      if( default0Save1Load2 == 1 )
      {
        sharedPredTS.copyFrom( piPred );
      }
    }
    else
    {
      // load prediction
      piPred.copyFrom( sharedPredTS );
    }
  }


  DTRACE( g_trace_ctx, D_PRED, "@(%4d,%4d) [%2dx%2d] IMode=%d\n", tu.lx(), tu.ly(), tu.lwidth(), tu.lheight(), uiChFinalMode );
  //DTRACE_PEL_BUF( D_PRED, piPred, tu, tu.cu->predMode, COMPONENT_Y );

  const Slice           &slice = *cs.slice;
  bool flag = slice.getLmcsEnabledFlag() && (slice.isIntra() || (!slice.isIntra() && m_pcReshape->getCTUFlag()));
#if JVET_W0103_INTRA_MTS
  if (!tu.cu->mtsFlag && isLuma(compID))
#else
  if (isLuma(compID))
#endif
  {
    //===== get residual signal =====
    piResi.copyFrom( piOrg  );
    if (slice.getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && compID == COMPONENT_Y)
    {
      piResi.rspSignal( m_pcReshape->getFwdLUT() );
      piResi.subtract( piPred );
    }
    else
    {
      piResi.subtract( piPred );
    }
  }

  //===== transform and quantization =====
  //--- init rate estimation arrays for RDOQ ---
  //--- transform and quantization           ---
  TCoeff uiAbsSum = 0;

  const QpParam cQP(tu, compID);

#if RDOQ_CHROMA_LAMBDA
  m_pcTrQuant->selectLambda(compID);
#endif

  flag =flag && (tu.blocks[compID].width*tu.blocks[compID].height > 4);
  if (flag && isChroma(compID) && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() )
  {
    int cResScaleInv = tu.getChromaAdj();
    double cResScale = (double)(1 << CSCALE_FP_PREC) / (double)cResScaleInv;
    m_pcTrQuant->setLambda(m_pcTrQuant->getLambda() / (cResScale*cResScale));
  }

  PelBuf          crOrg;
  PelBuf          crPred;
  PelBuf          crResi;
  PelBuf          crReco;

  if (isChroma(compID))
  {
    const CompArea &crArea = tu.blocks[ COMPONENT_Cr ];
    crOrg  = cs.getOrgBuf  ( crArea );
    crPred = cs.getPredBuf ( crArea );
    crResi = cs.getResiBuf ( crArea );
    crReco = cs.getRecoBuf ( crArea );
  }

  if ( jointCbCr )
  {
    // Lambda is loosened for the joint mode with respect to single modes as the same residual is used for both chroma blocks
    const int    absIct = abs( TU::getICTMode(tu) );
    const double lfact  = ( absIct == 1 || absIct == 3 ? 0.8 : 0.5 );
    m_pcTrQuant->setLambda( lfact * m_pcTrQuant->getLambda() );
  }
  if ( sps.getJointCbCrEnabledFlag() && isChroma(compID) && (tu.cu->cs->slice->getSliceQp() > 18) )
  {
    m_pcTrQuant->setLambda( 1.3 * m_pcTrQuant->getLambda() );
  }

  if( isLuma(compID) )
  {
    if (trModes)
    {
#if JVET_Y0142_ADAPT_INTRA_MTS
      m_pcTrQuant->transformNxN(tu, compID, cQP, trModes, 8);
#else
      m_pcTrQuant->transformNxN(tu, compID, cQP, trModes, m_pcEncCfg->getMTSIntraMaxCand());
#endif
      tu.mtsIdx[compID] = trModes->at(0).first;
    }

    if (!(m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless() && tu.mtsIdx[compID] == 0) || tu.cu->bdpcmMode != 0)
    {
      m_pcTrQuant->transformNxN(tu, compID, cQP, uiAbsSum, m_CABACEstimator->getCtx(), loadTr);
    }

    DTRACE(g_trace_ctx, D_TU_ABS_SUM, "%d: comp=%d, abssum=%d\n", DTRACE_GET_COUNTER(g_trace_ctx, D_TU_ABS_SUM), compID,
           uiAbsSum);

    if (tu.cu->ispMode && isLuma(compID) && CU::isISPLast(*tu.cu, area, area.compID) && CU::allLumaCBFsAreZero(*tu.cu))
    {
      // ISP has to have at least one non-zero CBF
      ruiDist = MAX_INT;
      return;
    }
#if JVET_Y0142_ADAPT_INTRA_MTS
    if (isLuma(compID) && tu.mtsIdx[compID] >= MTS_DST7_DST7)
    {
      bool signHiding = cs.slice->getSignDataHidingEnabledFlag();
      CoeffCodingContext  cctx(tu, compID, signHiding);
      const TCoeff*       coeff = tu.getCoeffs(compID).buf;
      int          scanPosLast = -1;
      uint64_t     coeffAbsSum = 0;

      for (int scanPos = 0; scanPos < cctx.maxNumCoeff(); scanPos++)
      {
        unsigned blkPos = cctx.blockPos(scanPos);
        if (coeff[blkPos])
        {
          scanPosLast = scanPos;
          coeffAbsSum += abs(coeff[blkPos]);
        }
      }
      int nCands = (coeffAbsSum > MTS_TH_COEFF[1]) ? MTS_NCANDS[2] : (coeffAbsSum > MTS_TH_COEFF[0]) ? MTS_NCANDS[1] : MTS_NCANDS[0];
      bool isInvalid = (scanPosLast <= 0) || ((tu.mtsIdx[COMPONENT_Y] - MTS_DST7_DST7) >= nCands);
      if (isInvalid)
      {
        m_validMTSReturn = false;
        ruiDist = MAX_INT;
        return;
      }
    }
#endif
    if ((m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless() && tu.mtsIdx[compID] == 0)
        && 0 == tu.cu->bdpcmMode)
    {
      uiAbsSum = 0;
      tu.getCoeffs(compID).fill(0);
      TU::setCbfAtDepth(tu, compID, tu.depth, 0);
    }

    //--- inverse transform ---
    if (uiAbsSum > 0)
    {
      m_pcTrQuant->invTransformNxN(tu, compID, piResi, cQP);
    }
    else
    {
      piResi.fill(0);
    }
  }
  else // chroma
  {
    int         codedCbfMask  = 0;
    ComponentID codeCompId    = (tu.jointCbCr ? (tu.jointCbCr >> 1 ? COMPONENT_Cb : COMPONENT_Cr) : compID);
    const QpParam qpCbCr(tu, codeCompId);

    if( tu.jointCbCr )
    {
      ComponentID otherCompId = ( codeCompId==COMPONENT_Cr ? COMPONENT_Cb : COMPONENT_Cr );
      tu.getCoeffs( otherCompId ).fill(0); // do we need that?
      TU::setCbfAtDepth (tu, otherCompId, tu.depth, false );
    }
    PelBuf& codeResi = ( codeCompId == COMPONENT_Cr ? crResi : piResi );
    uiAbsSum = 0;

    if (trModes)
    {
      m_pcTrQuant->transformNxN(tu, codeCompId, qpCbCr, trModes, m_pcEncCfg->getMTSIntraMaxCand());
      tu.mtsIdx[codeCompId] = trModes->at(0).first;
      if (tu.jointCbCr)
      {
        tu.mtsIdx[(codeCompId == COMPONENT_Cr) ? COMPONENT_Cb : COMPONENT_Cr] = MTS_DCT2_DCT2;
      }
    }
    // encoder bugfix: Set loadTr to aovid redundant transform process
    if (!(m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless() && tu.mtsIdx[compID] == 0) || tu.cu->bdpcmModeChroma != 0)
    {
        m_pcTrQuant->transformNxN(tu, codeCompId, qpCbCr, uiAbsSum, m_CABACEstimator->getCtx(), loadTr);
    }
    if ((m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless() && tu.mtsIdx[compID] == 0) && 0 == tu.cu->bdpcmModeChroma)
    {
        uiAbsSum = 0;
        tu.getCoeffs(compID).fill(0);
        TU::setCbfAtDepth(tu, compID, tu.depth, 0);
    }

    DTRACE( g_trace_ctx, D_TU_ABS_SUM, "%d: comp=%d, abssum=%d\n", DTRACE_GET_COUNTER( g_trace_ctx, D_TU_ABS_SUM ), codeCompId, uiAbsSum );
    if( uiAbsSum > 0 )
    {
      m_pcTrQuant->invTransformNxN(tu, codeCompId, codeResi, qpCbCr);
      codedCbfMask += ( codeCompId == COMPONENT_Cb ? 2 : 1 );
    }
    else
    {
      codeResi.fill(0);
    }

    if( tu.jointCbCr )
    {
      if( tu.jointCbCr == 3 && codedCbfMask == 2 )
      {
        codedCbfMask = 3;
        TU::setCbfAtDepth (tu, COMPONENT_Cr, tu.depth, true );
      }
      if( tu.jointCbCr != codedCbfMask )
      {
        ruiDist = std::numeric_limits<Distortion>::max();
        return;
      }
      m_pcTrQuant->invTransformICT( tu, piResi, crResi );
      uiAbsSum = codedCbfMask;
    }
  }

  //===== reconstruction =====
  if ( flag && uiAbsSum > 0 && isChroma(compID) && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() )
  {
    piResi.scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(compID));
    if( jointCbCr )
    {
      crResi.scaleSignal(tu.getChromaAdj(), 0, tu.cu->cs->slice->clpRng(COMPONENT_Cr));
    }
  }

  if (slice.getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && compID == COMPONENT_Y)
  {
    CompArea      tmpArea(COMPONENT_Y, area.chromaFormat, Position(0,0), area.size());
    PelBuf tmpPred = m_tmpStorageLCU.getBuf(tmpArea);
    tmpPred.copyFrom(piPred);
    piReco.reconstruct(tmpPred, piResi, cs.slice->clpRng(compID));
  }
  else
  {
    piReco.reconstruct(piPred, piResi, cs.slice->clpRng( compID ));
    if( jointCbCr )
    {
      crReco.reconstruct(crPred, crResi, cs.slice->clpRng( COMPONENT_Cr ));
    }
  }
#if SIGN_PREDICTION
#if INTRA_TRANS_ENC_OPT 
  bool doSignPrediction = true;
  if (isLuma(compID) && ((tu.mtsIdx[COMPONENT_Y] > MTS_SKIP) || (CS::isDualITree(cs) && tu.cu->lfnstIdx && !tu.cu->ispMode)))
  {
    bool signHiding = cs.slice->getSignDataHidingEnabledFlag();
    CoeffCodingContext  cctx(tu, COMPONENT_Y, signHiding);
    int scanPosLast = -1;
    TCoeff* coeff = tu.getCoeffs(compID).buf;
    for (int scanPos = cctx.maxNumCoeff() - 1; scanPos >= 0; scanPos--)
    {
      unsigned blkPos = cctx.blockPos(scanPos);
      if (coeff[blkPos])
      {
        scanPosLast = scanPos;
        break;
      }
    }
    if (scanPosLast < 1)
    {
      doSignPrediction = false;
    }
  }
#endif

  if ( sps.getNumPredSigns() > 0)
  {
#if INTRA_TRANS_ENC_OPT
    if (doSignPrediction)
    {
#endif
      bool bJccrWithCr = tu.jointCbCr && !(tu.jointCbCr >> 1);
      bool bIsJccr = tu.jointCbCr && isChroma(compID);
      ComponentID signPredCompID = bIsJccr ? (bJccrWithCr ? COMPONENT_Cr : COMPONENT_Cb) : compID;
      bool reshapeChroma = flag && (TU::getCbf(tu, signPredCompID) || tu.jointCbCr) && isChroma(signPredCompID) && slice.getPicHeader()->getLmcsChromaResidualScaleFlag();
      m_pcTrQuant->predCoeffSigns(tu, compID, reshapeChroma);
#if INTRA_TRANS_ENC_OPT
    }
#endif
  }

#if INTRA_TRANS_ENC_OPT 
  if (doSignPrediction)
  {
#endif
#endif
#if JVET_V0094_BILATERAL_FILTER
    CompArea      tmpArea1(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
    PelBuf tmpRecLuma;
    if(isLuma(compID))
    {
      tmpRecLuma = m_tmpStorageLCU.getBuf(tmpArea1);
      tmpRecLuma.copyFrom(piReco);
    }
#if JVET_X0071_CHROMA_BILATERAL_FILTER
    CompArea tmpArea2(compID, area.chromaFormat, Position(0, 0), area.size());
    PelBuf tmpRecChroma;
    if(isChroma(compID))
    {
      tmpRecChroma = m_tmpStorageLCU.getBuf(tmpArea2);
      tmpRecChroma.copyFrom(piReco);
    }
#endif
  //===== update distortion =====
#if WCG_EXT
  
    if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || (m_pcEncCfg->getLmcs()
      && slice.getLmcsEnabledFlag() && (m_pcReshape->getCTUFlag() || (isChroma(compID) && m_pcEncCfg->getReshapeIntraCMD()))))
    {
    
      const CPelBuf orgLuma = cs.getOrgBuf( cs.area.blocks[COMPONENT_Y] );
      if (compID == COMPONENT_Y)
      {
        if(!(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()))
        {
          tmpRecLuma.rspSignal(m_pcReshape->getInvLUT());
        }

        if (pps.getUseBIF() /*&& (uiAbsSum > 0)*/ && isLuma(compID) && (tu.cu->qp > 17) && (128 > std::max(tu.lumaSize().width, tu.lumaSize().height)))
        {
          CompArea compArea = tu.blocks[compID];
          PelBuf recIPredBuf = cs.slice->getPic()->getRecoBuf(compArea);
          if(!(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()))
          {
            m_bilateralFilter->bilateralFilterRDOdiamond5x5(tmpRecLuma, tmpRecLuma, tmpRecLuma, tu.cu->qp, recIPredBuf, cs.slice->clpRng(compID), tu, true, true, m_pcReshape->getInvLUT());
          }
          else
          {
            std::vector<Pel> invLUT;
            m_bilateralFilter->bilateralFilterRDOdiamond5x5(tmpRecLuma, tmpRecLuma, tmpRecLuma, tu.cu->qp, recIPredBuf, cs.slice->clpRng(compID), tu, true, false, invLUT);
          }
        }
      
        ruiDist += m_pcRdCost->getDistPart(piOrg, tmpRecLuma, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
      }
      else
      {
#if JVET_X0071_CHROMA_BILATERAL_FILTER
        if(pps.getUseChromaBIF() && isChroma(compID) && (tu.cu->qp > 17))
        {
          CompArea compArea = tu.blocks[compID];
          PelBuf recIPredBuf = cs.slice->getPic()->getRecoBuf(compArea);
          bool isCb = compID == COMPONENT_Cb ? true : false;
          m_bilateralFilter->bilateralFilterRDOdiamond5x5Chroma(tmpRecChroma, tmpRecChroma, tmpRecChroma, tu.cu->qp, recIPredBuf, cs.slice->clpRng(compID), tu, true, isCb);
        }
        ruiDist += m_pcRdCost->getDistPart(piOrg, tmpRecChroma, bitDepth, compID, DF_SSE_WTD, &orgLuma);
#else
        ruiDist += m_pcRdCost->getDistPart(piOrg, piReco, bitDepth, compID, DF_SSE_WTD, &orgLuma);
#endif
        if( jointCbCr )
        {
#if JVET_X0071_CHROMA_BILATERAL_FILTER
          if(compID == COMPONENT_Cr)
          {
            ruiDist += m_pcRdCost->getDistPart(crOrg, tmpRecChroma, bitDepth, COMPONENT_Cr, DF_SSE_WTD, &orgLuma);
          }
          else
          {
            ruiDist += m_pcRdCost->getDistPart(crOrg, crReco, bitDepth, COMPONENT_Cr, DF_SSE_WTD, &orgLuma);
          }
#else
          ruiDist += m_pcRdCost->getDistPart(crOrg, crReco, bitDepth, COMPONENT_Cr, DF_SSE_WTD, &orgLuma);
#endif
        }
      }
    }
    else
#endif
    {
      if(isLuma(compID))
      {
        if (pps.getUseBIF() /*&& (uiAbsSum > 0)*/ && isLuma(compID) && (tu.cu->qp > 17) && (128 > std::max(tu.lumaSize().width, tu.lumaSize().height)))
        {
          CompArea compArea = tu.blocks[compID];
          PelBuf recIPredBuf = cs.slice->getPic()->getRecoBuf(compArea);
          std::vector<Pel> invLUT;
          m_bilateralFilter->bilateralFilterRDOdiamond5x5(tmpRecLuma, tmpRecLuma, tmpRecLuma, tu.cu->qp, recIPredBuf, cs.slice->clpRng(compID), tu, true, false, invLUT);
        }
      
        ruiDist += m_pcRdCost->getDistPart( piOrg, tmpRecLuma, bitDepth, compID, DF_SSE );
      }
      else
      {
#if JVET_X0071_CHROMA_BILATERAL_FILTER
        if (pps.getUseChromaBIF() && isChroma(compID) && (tu.cu->qp > 17))
        {
          CompArea compArea = tu.blocks[compID];
          PelBuf recIPredBuf = cs.slice->getPic()->getRecoBuf(compArea);
          bool isCb = compID == COMPONENT_Cb ? true : false;
          m_bilateralFilter->bilateralFilterRDOdiamond5x5Chroma(tmpRecChroma, tmpRecChroma, tmpRecChroma, tu.cu->qp, recIPredBuf, cs.slice->clpRng(compID), tu, true, isCb);
        }
        ruiDist += m_pcRdCost->getDistPart( piOrg, tmpRecChroma, bitDepth, compID, DF_SSE );
#else
        ruiDist += m_pcRdCost->getDistPart( piOrg, piReco, bitDepth, compID, DF_SSE );
#endif
        if( jointCbCr )
        {
#if JVET_X0071_CHROMA_BILATERAL_FILTER
          if(compID == COMPONENT_Cr)
          {
            ruiDist += m_pcRdCost->getDistPart( crOrg, tmpRecChroma, bitDepth, COMPONENT_Cr, DF_SSE );
          }
          else
          {
            ruiDist += m_pcRdCost->getDistPart( crOrg, crReco, bitDepth, COMPONENT_Cr, DF_SSE );
          }
#else
          ruiDist += m_pcRdCost->getDistPart( crOrg, crReco, bitDepth, COMPONENT_Cr, DF_SSE );
#endif
        }
      }
    }

#else
  //===== update distortion =====
#if JVET_X0071_CHROMA_BILATERAL_FILTER
    CompArea tmpArea2(compID, area.chromaFormat, Position(0, 0), area.size());
    PelBuf tmpRecChroma;
    if(isChroma(compID))
    {
      tmpRecChroma = m_tmpStorageLCU.getBuf(tmpArea2);
      tmpRecChroma.copyFrom(piReco);
    }
#if WCG_EXT
    if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || (m_pcEncCfg->getLmcs() && slice.getLmcsEnabledFlag() && (m_pcReshape->getCTUFlag() || (isChroma(compID) && m_pcEncCfg->getReshapeIntraCMD()))))
    {
      const CPelBuf orgLuma = cs.getOrgBuf( cs.area.blocks[COMPONENT_Y] );
      if(isLuma(compID))
      {
        if (compID == COMPONENT_Y  && !(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()))
        {
          CompArea      tmpArea1(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
          PelBuf tmpRecLuma = m_tmpStorageLCU.getBuf(tmpArea1);
          tmpRecLuma.rspSignal( piReco, m_pcReshape->getInvLUT() );
          ruiDist += m_pcRdCost->getDistPart(piOrg, tmpRecLuma, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
        }
        else
        {
          ruiDist += m_pcRdCost->getDistPart(piOrg, piReco, bitDepth, compID, DF_SSE_WTD, &orgLuma);
          if( jointCbCr )
          {
            ruiDist += m_pcRdCost->getDistPart(crOrg, crReco, bitDepth, COMPONENT_Cr, DF_SSE_WTD, &orgLuma);
          }
        }
      }
      else
      {
        if(pps.getUseChromaBIF() && isChroma(compID) && (tu.cu->qp > 17))
        {
          CompArea compArea = tu.blocks[compID];
          PelBuf recIPredBuf = cs.slice->getPic()->getRecoBuf(compArea);
          bool isCb = compID == COMPONENT_Cb ? true : false;
          m_bilateralFilter->bilateralFilterRDOdiamond5x5Chroma(tmpRecChroma, tmpRecChroma, tmpRecChroma, tu.cu->qp, recIPredBuf, cs.slice->clpRng(compID), tu, true, isCb);
        }
        ruiDist += m_pcRdCost->getDistPart(piOrg, tmpRecChroma, bitDepth, compID, DF_SSE_WTD, &orgLuma);
        if( jointCbCr )
        {
          if(compID == COMPONENT_Cr)
          {
            ruiDist += m_pcRdCost->getDistPart(crOrg, tmpRecChroma, bitDepth, COMPONENT_Cr, DF_SSE_WTD, &orgLuma);
          }
          else
          {
            ruiDist += m_pcRdCost->getDistPart(crOrg, crReco, bitDepth, COMPONENT_Cr, DF_SSE_WTD, &orgLuma);
          }
        }
      }
    }
    else
#endif
    {
      if(isLuma(compID))
      {
        ruiDist += m_pcRdCost->getDistPart( piOrg, piReco, bitDepth, compID, DF_SSE );
      }
      else
      {
        if (pps.getUseChromaBIF() && isChroma(compID) && (tu.cu->qp > 17))
        {
          CompArea compArea = tu.blocks[compID];
          PelBuf recIPredBuf = cs.slice->getPic()->getRecoBuf(compArea);
          bool isCb = compID == COMPONENT_Cb ? true : false;
          m_bilateralFilter->bilateralFilterRDOdiamond5x5Chroma(tmpRecChroma, tmpRecChroma, tmpRecChroma, tu.cu->qp, recIPredBuf, cs.slice->clpRng(compID), tu, true, isCb);
        }
        ruiDist += m_pcRdCost->getDistPart( piOrg, tmpRecChroma, bitDepth, compID, DF_SSE );
      }
      if( jointCbCr )
      {
        if(compID == COMPONENT_Cr)
        {
          ruiDist += m_pcRdCost->getDistPart( crOrg, tmpRecChroma, bitDepth, COMPONENT_Cr, DF_SSE );
        }
        else
        {
          ruiDist += m_pcRdCost->getDistPart( crOrg, crReco, bitDepth, COMPONENT_Cr, DF_SSE );
        }
      }
    }
#else
#if WCG_EXT
    if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || (m_pcEncCfg->getLmcs()
      && slice.getLmcsEnabledFlag() && (m_pcReshape->getCTUFlag() || (isChroma(compID) && m_pcEncCfg->getReshapeIntraCMD()))))
    {
      const CPelBuf orgLuma = cs.getOrgBuf( cs.area.blocks[COMPONENT_Y] );
      if (compID == COMPONENT_Y  && !(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()))
      {
        CompArea      tmpArea1(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
        PelBuf tmpRecLuma = m_tmpStorageLCU.getBuf(tmpArea1);
        tmpRecLuma.rspSignal( piReco, m_pcReshape->getInvLUT() );
        ruiDist += m_pcRdCost->getDistPart(piOrg, tmpRecLuma, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
      }
      else
      {
        ruiDist += m_pcRdCost->getDistPart(piOrg, piReco, bitDepth, compID, DF_SSE_WTD, &orgLuma);
        if( jointCbCr )
        {
          ruiDist += m_pcRdCost->getDistPart(crOrg, crReco, bitDepth, COMPONENT_Cr, DF_SSE_WTD, &orgLuma);
        }
      }
    }
    else
#endif
    {
      ruiDist += m_pcRdCost->getDistPart( piOrg, piReco, bitDepth, compID, DF_SSE );
      if( jointCbCr )
      {
        ruiDist += m_pcRdCost->getDistPart( crOrg, crReco, bitDepth, COMPONENT_Cr, DF_SSE );
      }
    }
#endif
#endif
#if INTRA_TRANS_ENC_OPT && SIGN_PREDICTION
  }
#endif
}

void IntraSearch::xIntraCodingACTTUBlock(TransformUnit &tu, const ComponentID &compID, Distortion& ruiDist, std::vector<TrMode>* trModes, const bool loadTr)
{
  if (!tu.blocks[compID].valid())
  {
    CHECK(1, "tu does not exist");
  }

  CodingStructure     &cs = *tu.cs;
  const SPS           &sps = *cs.sps;
  const Slice         &slice = *cs.slice;
  const CompArea      &area = tu.blocks[compID];
  const CompArea &crArea = tu.blocks[COMPONENT_Cr];

  PelBuf              piOrgResi = cs.getOrgResiBuf(area);
  PelBuf              piResi = cs.getResiBuf(area);
  PelBuf              crOrgResi = cs.getOrgResiBuf(crArea);
  PelBuf              crResi = cs.getResiBuf(crArea);
  TCoeff              uiAbsSum = 0;

  CHECK(tu.jointCbCr && compID == COMPONENT_Cr, "wrong combination of compID and jointCbCr");
  bool jointCbCr = tu.jointCbCr && compID == COMPONENT_Cb;

  m_pcRdCost->setChromaFormat(cs.sps->getChromaFormatIdc());
  if (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless())
  m_pcTrQuant->lambdaAdjustColorTrans(true);

  if (jointCbCr)
  {
    ComponentID compIdCode = (tu.jointCbCr >> 1 ? COMPONENT_Cb : COMPONENT_Cr);
    m_pcTrQuant->selectLambda(compIdCode);
  }
  else
  {
    m_pcTrQuant->selectLambda(compID);
  }

  bool flag = slice.getLmcsEnabledFlag() && (slice.isIntra() || (!slice.isIntra() && m_pcReshape->getCTUFlag())) && (tu.blocks[compID].width*tu.blocks[compID].height > 4);
  if (flag && isChroma(compID) && slice.getPicHeader()->getLmcsChromaResidualScaleFlag())
  {
    int    cResScaleInv = tu.getChromaAdj();
    double cResScale = (double)(1 << CSCALE_FP_PREC) / (double)cResScaleInv;
    m_pcTrQuant->setLambda(m_pcTrQuant->getLambda() / (cResScale*cResScale));
  }

  if (jointCbCr)
  {
    // Lambda is loosened for the joint mode with respect to single modes as the same residual is used for both chroma blocks
    const int    absIct = abs(TU::getICTMode(tu));
    const double lfact = (absIct == 1 || absIct == 3 ? 0.8 : 0.5);
    m_pcTrQuant->setLambda(lfact * m_pcTrQuant->getLambda());
  }
  if (sps.getJointCbCrEnabledFlag() && isChroma(compID) && (slice.getSliceQp() > 18))
  {
    m_pcTrQuant->setLambda(1.3 * m_pcTrQuant->getLambda());
  }

  if (isLuma(compID))
  {
    QpParam cQP(tu, compID);

    if (trModes)
    {
      m_pcTrQuant->transformNxN(tu, compID, cQP, trModes, m_pcEncCfg->getMTSIntraMaxCand());
      tu.mtsIdx[compID] = trModes->at(0).first;
    }
    if (!(m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless() && tu.mtsIdx[compID] == 0) || tu.cu->bdpcmMode != 0)
    {
      m_pcTrQuant->transformNxN(tu, compID, cQP, uiAbsSum, m_CABACEstimator->getCtx(), loadTr);
    }
    if ((m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless() && tu.mtsIdx[compID] == 0) && tu.cu->bdpcmMode == 0)
    {
      uiAbsSum = 0;
      tu.getCoeffs(compID).fill(0);
      TU::setCbfAtDepth(tu, compID, tu.depth, 0);
    }

    if (uiAbsSum > 0)
    {
      m_pcTrQuant->invTransformNxN(tu, compID, piResi, cQP);
    }
    else
    {
      piResi.fill(0);
    }
  }
  else
  {
    int         codedCbfMask = 0;
    ComponentID codeCompId = (tu.jointCbCr ? (tu.jointCbCr >> 1 ? COMPONENT_Cb : COMPONENT_Cr) : compID);
    QpParam qpCbCr(tu, codeCompId);

    if (tu.jointCbCr)
    {
      ComponentID otherCompId = (codeCompId == COMPONENT_Cr ? COMPONENT_Cb : COMPONENT_Cr);
      tu.getCoeffs(otherCompId).fill(0);
      TU::setCbfAtDepth(tu, otherCompId, tu.depth, false);
    }

    PelBuf& codeResi = (codeCompId == COMPONENT_Cr ? crResi : piResi);
    uiAbsSum = 0;
    if (trModes)
    {
      m_pcTrQuant->transformNxN(tu, codeCompId, qpCbCr, trModes, m_pcEncCfg->getMTSIntraMaxCand());
      tu.mtsIdx[codeCompId] = trModes->at(0).first;
      if (tu.jointCbCr)
      {
        tu.mtsIdx[(codeCompId == COMPONENT_Cr) ? COMPONENT_Cb : COMPONENT_Cr] = MTS_DCT2_DCT2;
      }
    }
    if (!(m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless() && tu.mtsIdx[codeCompId] == 0) || tu.cu->bdpcmModeChroma != 0)
    {
      m_pcTrQuant->transformNxN(tu, codeCompId, qpCbCr, uiAbsSum, m_CABACEstimator->getCtx(), loadTr);
    }
    if (uiAbsSum > 0)
    {
      m_pcTrQuant->invTransformNxN(tu, codeCompId, codeResi, qpCbCr);
      codedCbfMask += (codeCompId == COMPONENT_Cb ? 2 : 1);
    }
    else
    {
      codeResi.fill(0);
    }

    if (tu.jointCbCr)
    {
      if (tu.jointCbCr == 3 && codedCbfMask == 2)
      {
        codedCbfMask = 3;
        TU::setCbfAtDepth(tu, COMPONENT_Cr, tu.depth, true);
      }
      if (tu.jointCbCr != codedCbfMask)
      {
        ruiDist = std::numeric_limits<Distortion>::max();
        if (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless())
        m_pcTrQuant->lambdaAdjustColorTrans(false);
        return;
      }
      m_pcTrQuant->invTransformICT(tu, piResi, crResi);
      uiAbsSum = codedCbfMask;
    }
  }

#if !JVET_S0234_ACT_CRS_FIX
  if (flag && uiAbsSum > 0 && isChroma(compID) && slice.getPicHeader()->getLmcsChromaResidualScaleFlag())
  {
    piResi.scaleSignal(tu.getChromaAdj(), 0, slice.clpRng(compID));
    if (jointCbCr)
    {
      crResi.scaleSignal(tu.getChromaAdj(), 0, slice.clpRng(COMPONENT_Cr));
    }
  }
#endif
  if (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless())
  m_pcTrQuant->lambdaAdjustColorTrans(false);

  ruiDist += m_pcRdCost->getDistPart(piOrgResi, piResi, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE);
  if (jointCbCr)
  {
    ruiDist += m_pcRdCost->getDistPart(crOrgResi, crResi, sps.getBitDepth(toChannelType(COMPONENT_Cr)), COMPONENT_Cr, DF_SSE);
  }
}

bool IntraSearch::xIntraCodingLumaISP(CodingStructure& cs, Partitioner& partitioner, const double bestCostSoFar)
{
  int               subTuCounter = 0;
  const CodingUnit& cu = *cs.getCU(partitioner.currArea().lumaPos(), partitioner.chType);
  bool              earlySkipISP = false;
  bool              splitCbfLuma = false;
  const PartSplit   ispType = CU::getISPType(cu, COMPONENT_Y);

  cs.cost = 0;

  partitioner.splitCurrArea(ispType, cs);

  CUCtx cuCtx;
  cuCtx.isDQPCoded = true;
  cuCtx.isChromaQpAdjCoded = true;

  do   // subpartitions loop
  {
    uint32_t   numSig = 0;
    Distortion singleDistTmpLuma = 0;
    uint64_t   singleTmpFracBits = 0;
    double     singleCostTmp = 0;

    TransformUnit& tu = cs.addTU(CS::getArea(cs, partitioner.currArea(), partitioner.chType), partitioner.chType);
    tu.depth = partitioner.currTrDepth;

    // Encode TU
    xIntraCodingTUBlock(tu, COMPONENT_Y, singleDistTmpLuma, 0, &numSig);

#if SIGN_PREDICTION
#if JVET_Z0118_GDR
    cs.updateReconMotIPM(partitioner.currArea());
#else
    cs.picture->getRecoBuf( partitioner.currArea() ).copyFrom( cs.getRecoBuf( partitioner.currArea() ) );
#endif
#endif

    if (singleDistTmpLuma == MAX_INT)   // all zero CBF skip
    {
      earlySkipISP = true;
      partitioner.exitCurrSplit();
      cs.cost = MAX_DOUBLE;
      return false;
    }

    if (m_pcRdCost->calcRdCost(cs.fracBits, cs.dist + singleDistTmpLuma) > bestCostSoFar)
    {
      // The accumulated cost + distortion is already larger than the best cost so far, so it is not necessary to
      // calculate the rate
      earlySkipISP = true;
    }
    else
    {
      singleTmpFracBits = xGetIntraFracBitsQT(cs, partitioner, true, false, subTuCounter, ispType, &cuCtx);
    }
    singleCostTmp = m_pcRdCost->calcRdCost(singleTmpFracBits, singleDistTmpLuma);

    cs.cost += singleCostTmp;
    cs.dist += singleDistTmpLuma;
    cs.fracBits += singleTmpFracBits;

    subTuCounter++;

    splitCbfLuma |= TU::getCbfAtDepth(*cs.getTU(partitioner.currArea().lumaPos(), partitioner.chType, subTuCounter - 1), COMPONENT_Y, partitioner.currTrDepth);
    int nSubPartitions = m_ispTestedModes[cu.lfnstIdx].numTotalParts[cu.ispMode - 1];
    if (subTuCounter < nSubPartitions)
    {
      // exit condition if the accumulated cost is already larger than the best cost so far (no impact in RD performance)
      if (cs.cost > bestCostSoFar)
      {
        earlySkipISP = true;
        break;
      }
      else if (subTuCounter < nSubPartitions)
      {
        // more restrictive exit condition
        double threshold = nSubPartitions == 2 ? 0.95 : subTuCounter == 1 ? 0.83 : 0.91;
        if (subTuCounter < nSubPartitions && cs.cost > bestCostSoFar * threshold)
        {
          earlySkipISP = true;
          break;
        }
      }
    }
  } while (partitioner.nextPart(cs));   // subpartitions loop

  partitioner.exitCurrSplit();
  const UnitArea& currArea = partitioner.currArea();
  const uint32_t  currDepth = partitioner.currTrDepth;

  if (earlySkipISP)
  {
    cs.cost = MAX_DOUBLE;
  }
  else
  {
    cs.cost = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);
    // The cost check is necessary here again to avoid superfluous operations if the maximum number of coded subpartitions was reached and yet ISP did not win
    if (cs.cost < bestCostSoFar)
    {
      cs.setDecomp(cu.Y());
#if JVET_Z0118_GDR
      cs.updateReconMotIPM(currArea.Y());
#else
      cs.picture->getRecoBuf(currArea.Y()).copyFrom(cs.getRecoBuf(currArea.Y()));
#endif

      for (auto& ptu : cs.tus)
      {
        if (currArea.Y().contains(ptu->Y()))
        {
          TU::setCbfAtDepth(*ptu, COMPONENT_Y, currDepth, splitCbfLuma ? 1 : 0);
        }
      }
    }
    else
    {
      earlySkipISP = true;
    }
  }
  return !earlySkipISP;
}


bool IntraSearch::xRecurIntraCodingLumaQT( CodingStructure &cs, Partitioner &partitioner, const double bestCostSoFar, const int subTuIdx, const PartSplit ispType, const bool ispIsCurrentWinner, bool mtsCheckRangeFlag, int mtsFirstCheckId, int mtsLastCheckId, bool moreProbMTSIdxFirst )
{
        int   subTuCounter = subTuIdx;
  const UnitArea &currArea = partitioner.currArea();
  const CodingUnit     &cu = *cs.getCU( currArea.lumaPos(), partitioner.chType );
        bool  earlySkipISP = false;
  uint32_t currDepth       = partitioner.currTrDepth;
  const SPS &sps           = *cs.sps;
  bool bCheckFull          = true;
  bool bCheckSplit         = false;
  bCheckFull               = !partitioner.canSplit( TU_MAX_TR_SPLIT, cs );
  bCheckSplit              = partitioner.canSplit( TU_MAX_TR_SPLIT, cs );
  const Slice           &slice = *cs.slice;

  if( cu.ispMode )
  {
    bCheckSplit = partitioner.canSplit( ispType, cs );
    bCheckFull = !bCheckSplit;
  }
  uint32_t    numSig           = 0;

  double     dSingleCost                        = MAX_DOUBLE;
  Distortion uiSingleDistLuma                   = 0;
  uint64_t   singleFracBits                     = 0;
  bool       checkTransformSkip                 = sps.getTransformSkipEnabledFlag();
  int        bestModeId[ MAX_NUM_COMPONENT ]    = { 0, 0, 0 };
  uint8_t    nNumTransformCands                 = cu.mtsFlag ? 4 : 1;
  uint8_t    numTransformIndexCands             = nNumTransformCands;

  const TempCtx ctxStart  ( m_CtxCache, m_CABACEstimator->getCtx() );
  TempCtx       ctxBest   ( m_CtxCache );

  CodingStructure *csSplit = nullptr;
  CodingStructure *csFull  = nullptr;

  CUCtx cuCtx;
  cuCtx.isDQPCoded = true;
  cuCtx.isChromaQpAdjCoded = true;

  if( bCheckSplit )
  {
    csSplit = &cs;
  }
  else if( bCheckFull )
  {
    csFull = &cs;
  }

  bool validReturnFull = false;

  if( bCheckFull )
  {
    csFull->cost = 0.0;

    TransformUnit &tu = csFull->addTU( CS::getArea( *csFull, currArea, partitioner.chType ), partitioner.chType );
    tu.depth = currDepth;

    const bool tsAllowed  = TU::isTSAllowed( tu, COMPONENT_Y );
    const bool mtsAllowed = CU::isMTSAllowed( cu, COMPONENT_Y );
    std::vector<TrMode> trModes;

    if( sps.getUseLFNST() )
    {
      checkTransformSkip &= tsAllowed;
      checkTransformSkip &= !cu.mtsFlag;
      checkTransformSkip &= !cu.lfnstIdx;

      if( !cu.mtsFlag && checkTransformSkip )
      {
        trModes.push_back( TrMode( 0, true ) ); //DCT2
        trModes.push_back( TrMode( 1, true ) ); //TS
      }
    }
    else
    {
#if JVET_Y0142_ADAPT_INTRA_MTS
      nNumTransformCands = 1 + (tsAllowed ? 1 : 0) + (mtsAllowed ? 6 : 0); // DCT + TS + 6 MTS = 8 tests
#else
      nNumTransformCands = 1 + ( tsAllowed ? 1 : 0 ) + ( mtsAllowed ? 4 : 0 ); // DCT + TS + 4 MTS = 6 tests
#endif
      if (m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless())
      {
        nNumTransformCands = 1;
        CHECK(!tsAllowed && !cu.bdpcmMode, "transform skip should be enabled for LS");
        if (cu.bdpcmMode)
        {
          trModes.push_back(TrMode(0, true));
        }
        else
        {
          trModes.push_back(TrMode(1, true));
        }
      }
      else
      {
        trModes.push_back(TrMode(0, true));   // DCT2
        if (tsAllowed)
        {
          trModes.push_back(TrMode(1, true));
        }
        if (mtsAllowed)
        {
#if JVET_Y0142_ADAPT_INTRA_MTS
          for (int i = 2; i < 8; i++)
#else
          for (int i = 2; i < 6; i++)
#endif
          {
            trModes.push_back(TrMode(i, true));
          }
        }
      }
    }

    CHECK( !tu.Y().valid(), "Invalid TU" );

    CodingStructure &saveCS = *m_pSaveCS[0];

    TransformUnit *tmpTU = nullptr;

    Distortion singleDistTmpLuma = 0;
    uint64_t     singleTmpFracBits = 0;
    double     singleCostTmp     = 0;
    int        firstCheckId      = ( sps.getUseLFNST() && mtsCheckRangeFlag && cu.mtsFlag ) ? mtsFirstCheckId : 0;

    //we add the MTS candidates to the loop. TransformSkip will still be the last one to be checked (when modeId == lastCheckId) as long as checkTransformSkip is true
    int        lastCheckId       = sps.getUseLFNST() ? ( ( mtsCheckRangeFlag && cu.mtsFlag ) ? ( mtsLastCheckId + ( int ) checkTransformSkip ) : ( numTransformIndexCands - ( firstCheckId + 1 ) + ( int ) checkTransformSkip ) ) :
                                   trModes[ nNumTransformCands - 1 ].first;
    bool isNotOnlyOneMode        = sps.getUseLFNST() ? lastCheckId != firstCheckId : nNumTransformCands != 1;

    if( isNotOnlyOneMode )
    {
      saveCS.pcv     = cs.pcv;
      saveCS.picture = cs.picture;
#if JVET_Z0118_GDR
      saveCS.m_pt = cs.m_pt;
#endif
      saveCS.area.repositionTo(cs.area);
      saveCS.clearTUs();
      tmpTU = &saveCS.addTU(currArea, partitioner.chType);
    }

    bool    cbfBestMode      = false;
    bool    cbfBestModeValid = false;
    bool    cbfDCT2  = true;
#if JVET_W0103_INTRA_MTS
    if (sps.getUseLFNST() && cu.mtsFlag)
    {
      xSelectAMTForFullRD(tu);
    }
#endif
    double bestDCT2cost = MAX_DOUBLE;
    double threshold = m_pcEncCfg->getUseFastISP() && !cu.ispMode && ispIsCurrentWinner && nNumTransformCands > 1 ? 1 + 1.4 / sqrt( cu.lwidth() * cu.lheight() ) : 1;
    for( int modeId = firstCheckId; modeId <= ( sps.getUseLFNST() ? lastCheckId : ( nNumTransformCands - 1 ) ); modeId++ )
    {
      uint8_t transformIndex = modeId;
#if JVET_W0103_INTRA_MTS
      if (sps.getUseLFNST() && cu.mtsFlag)
      {
        if (modeId >= m_numCandAMTForFullRD)
        {
          continue;
        }
        transformIndex = m_testAMTForFullRD[modeId];
      }
#if JVET_Y0142_ADAPT_INTRA_MTS
      m_validMTSReturn = true;
#endif
#endif
      if( sps.getUseLFNST() )
      {
        if( ( transformIndex < lastCheckId ) || ( ( transformIndex == lastCheckId ) && !checkTransformSkip ) ) //we avoid this if the mode is transformSkip
        {
          // Skip checking other transform candidates if zero CBF is encountered and it is the best transform so far
          if( m_pcEncCfg->getUseFastLFNST() && transformIndex && !cbfBestMode && cbfBestModeValid )
          {
            continue;
          }
        }
      }
      else
      {
        if (!(m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless()))
        {
          if (!cbfDCT2 || (m_pcEncCfg->getUseTransformSkipFast() && bestModeId[COMPONENT_Y] == MTS_SKIP))
          {
            break;
          }
          if (!trModes[modeId].second)
          {
            continue;
          }
          // we compare the DCT-II cost against the best ISP cost so far (except for TS)
          if (m_pcEncCfg->getUseFastISP() && !cu.ispMode && ispIsCurrentWinner && trModes[modeId].first != MTS_DCT2_DCT2
              && (trModes[modeId].first != MTS_SKIP || !tsAllowed) && bestDCT2cost > bestCostSoFar * threshold)
          {
            continue;
          }
        }
        tu.mtsIdx[COMPONENT_Y] = trModes[modeId].first;
      }


      if ((modeId != firstCheckId) && isNotOnlyOneMode)
      {
        m_CABACEstimator->getCtx() = ctxStart;
      }

      int default0Save1Load2 = 0;
      singleDistTmpLuma = 0;

      if( modeId == firstCheckId && ( sps.getUseLFNST() ? ( modeId != lastCheckId ) : ( nNumTransformCands > 1 ) ) )
      {
        default0Save1Load2 = 1;
      }
      else if (modeId != firstCheckId)
      {
        if( sps.getUseLFNST() && !cbfBestModeValid )
        {
          default0Save1Load2 = 1;
        }
        else
        {
          default0Save1Load2 = 2;
        }
      }
      if( cu.ispMode )
      {
        default0Save1Load2 = 0;
      }
      if( sps.getUseLFNST() )
      {
        if( cu.mtsFlag )
        {
          if( moreProbMTSIdxFirst )
          {
            const ChannelType     chType      = toChannelType( COMPONENT_Y );
            const CompArea&       area        = tu.blocks[ COMPONENT_Y ];
            const PredictionUnit& pu          = *cs.getPU( area.pos(), chType );
            uint32_t              uiIntraMode = pu.intraDir[ chType ];

            if( transformIndex == 1 )
            {
              tu.mtsIdx[COMPONENT_Y] = (uiIntraMode < 34) ? MTS_DST7_DCT8 : MTS_DCT8_DST7;
            }
            else if( transformIndex == 2 )
            {
              tu.mtsIdx[COMPONENT_Y] = (uiIntraMode < 34) ? MTS_DCT8_DST7 : MTS_DST7_DCT8;
            }
            else
            {
              tu.mtsIdx[COMPONENT_Y] = MTS_DST7_DST7 + transformIndex;
            }
          }
          else
          {
            tu.mtsIdx[COMPONENT_Y] = MTS_DST7_DST7 + transformIndex;
          }
        }
        else
        {
          tu.mtsIdx[COMPONENT_Y] = transformIndex;
        }

        if( !cu.mtsFlag && checkTransformSkip )
        {
          xIntraCodingTUBlock( tu, COMPONENT_Y, singleDistTmpLuma, default0Save1Load2, &numSig, modeId == 0 ? &trModes : nullptr, true );
          if( modeId == 0 )
          {
            for( int i = 0; i < 2; i++ )
            {
              if( trModes[ i ].second )
              {
                lastCheckId = trModes[ i ].first;
              }
            }
          }
        }
#if JVET_W0103_INTRA_MTS
        else if (cu.mtsFlag)
        {
          xIntraCodingTUBlock(tu, COMPONENT_Y, singleDistTmpLuma, 2, &numSig, nullptr, true);
        }
#endif
        else
        {
          xIntraCodingTUBlock( tu, COMPONENT_Y, singleDistTmpLuma, default0Save1Load2, &numSig );
        }
      }
      else
      {
        if( nNumTransformCands > 1 )
        {
          xIntraCodingTUBlock( tu, COMPONENT_Y, singleDistTmpLuma, default0Save1Load2, &numSig, modeId == 0 ? &trModes : nullptr, true );
          if( modeId == 0 )
          {
            for( int i = 0; i < nNumTransformCands; i++ )
            {
              if( trModes[ i ].second )
              {
                lastCheckId = trModes[ i ].first;
              }
            }
          }
        }
        else
        {
          xIntraCodingTUBlock( tu, COMPONENT_Y, singleDistTmpLuma, default0Save1Load2, &numSig );
        }
      }
#if JVET_Y0142_ADAPT_INTRA_MTS
      cuCtx.mtsCoeffAbsSum = 0;
#endif
      cuCtx.mtsLastScanPos = false;
#if INTRA_TRANS_ENC_OPT
      cuCtx.lfnstLastScanPos = false;
#endif

      //----- determine rate and r-d cost -----
      if( ( sps.getUseLFNST() ? ( modeId == lastCheckId && modeId != 0 && checkTransformSkip ) : ( trModes[ modeId ].first != 0 ) ) && !TU::getCbfAtDepth( tu, COMPONENT_Y, currDepth ) )
      {
        //In order not to code TS flag when cbf is zero, the case for TS with cbf being zero is forbidden.
        if (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless())
        {
          singleCostTmp = MAX_DOUBLE;
        }
        else
        {
          singleTmpFracBits = xGetIntraFracBitsQT(*csFull, partitioner, true, false, subTuCounter, ispType, &cuCtx);
          singleCostTmp = m_pcRdCost->calcRdCost(singleTmpFracBits, singleDistTmpLuma);
        }
      }
      else
      {
        if( cu.ispMode && m_pcRdCost->calcRdCost( csFull->fracBits, csFull->dist + singleDistTmpLuma ) > bestCostSoFar )
        {
          earlySkipISP = true;
        }
        else
        {
#if JVET_Y0142_ADAPT_INTRA_MTS
          if (tu.mtsIdx[COMPONENT_Y] > MTS_SKIP && !m_validMTSReturn)
          {
            singleTmpFracBits = 0;
          }
          else
          {
            singleTmpFracBits = xGetIntraFracBitsQT(*csFull, partitioner, true, false, subTuCounter, ispType, &cuCtx);
          }
#else
          singleTmpFracBits = xGetIntraFracBitsQT( *csFull, partitioner, true, false, subTuCounter, ispType, &cuCtx );
#endif
        }
        if (tu.mtsIdx[COMPONENT_Y] > MTS_SKIP)
        {
#if JVET_Y0142_ADAPT_INTRA_MTS
          if(!m_validMTSReturn)
#else
          if (!cuCtx.mtsLastScanPos)
#endif
          {
            singleCostTmp = MAX_DOUBLE;
          }
          else
          {
            singleCostTmp = m_pcRdCost->calcRdCost(singleTmpFracBits, singleDistTmpLuma);
          }
        }
#if INTRA_TRANS_ENC_OPT
        else if (CS::isDualITree(cs) && cu.lfnstIdx && !cu.ispMode)
        {
          if (!cuCtx.lfnstLastScanPos)
          {
            singleCostTmp = MAX_DOUBLE;
          }
          else
          {
            singleCostTmp = m_pcRdCost->calcRdCost(singleTmpFracBits, singleDistTmpLuma);
          }
        }
#endif
        else
        {
          singleCostTmp = m_pcRdCost->calcRdCost(singleTmpFracBits, singleDistTmpLuma);
        }
      }

      if ( !cu.ispMode && nNumTransformCands > 1 && modeId == firstCheckId )
      {
        bestDCT2cost = singleCostTmp;
      }
#if JVET_W0103_INTRA_MTS
      if (sps.getUseLFNST() && cu.mtsFlag)
      {
        if (singleCostTmp != MAX_DOUBLE)
        {
          const CompArea&       area = tu.blocks[COMPONENT_Y];
          double skipThreshold = 1.0 + 1.0 / sqrt((double)(area.width*area.height));
          skipThreshold = std::max(skipThreshold, !m_pcEncCfg->getUseFastLFNST()? 1.06: 1.03);
#if JVET_Y0142_ADAPT_INTRA_MTS
          skipThreshold = (m_coeffAbsSumDCT2 >= MTS_TH_COEFF[1])? std::max(skipThreshold, 1.06) : skipThreshold;
#endif
          if (singleCostTmp > skipThreshold * m_globalBestCostStore)
          {
            m_numCandAMTForFullRD = modeId + 1;
          }
        }
      }
#if JVET_Y0142_ADAPT_INTRA_MTS
      if (tu.mtsIdx[0] == 0 && !cu.ispMode && !cu.lfnstIdx)
      {
        m_coeffAbsSumDCT2 = cuCtx.mtsCoeffAbsSum;
      }
#endif
#endif
      if (singleCostTmp < dSingleCost)
      {
        dSingleCost       = singleCostTmp;
        uiSingleDistLuma  = singleDistTmpLuma;
        singleFracBits    = singleTmpFracBits;

        if( sps.getUseLFNST() )
        {
          bestModeId[ COMPONENT_Y ] = modeId;
          cbfBestMode = TU::getCbfAtDepth( tu, COMPONENT_Y, currDepth );
          cbfBestModeValid = true;
          validReturnFull = true;
        }
        else
        {
          bestModeId[ COMPONENT_Y ] = trModes[ modeId ].first;
          if( trModes[ modeId ].first == 0 )
          {
            cbfDCT2 = TU::getCbfAtDepth( tu, COMPONENT_Y, currDepth );
          }
        }

        if( bestModeId[COMPONENT_Y] != lastCheckId )
        {
          saveCS.getPredBuf( tu.Y() ).copyFrom( csFull->getPredBuf( tu.Y() ) );
          saveCS.getRecoBuf( tu.Y() ).copyFrom( csFull->getRecoBuf( tu.Y() ) );

          if( KEEP_PRED_AND_RESI_SIGNALS )
          {
            saveCS.getResiBuf   ( tu.Y() ).copyFrom( csFull->getResiBuf   ( tu.Y() ) );
            saveCS.getOrgResiBuf( tu.Y() ).copyFrom( csFull->getOrgResiBuf( tu.Y() ) );
          }

          tmpTU->copyComponentFrom( tu, COMPONENT_Y );

          ctxBest = m_CABACEstimator->getCtx();
        }
      }
    }

    if( sps.getUseLFNST() && !validReturnFull )
    {
      csFull->cost = MAX_DOUBLE;

      if( bCheckSplit )
      {
        ctxBest = m_CABACEstimator->getCtx();
      }
    }
    else
    {
      if( bestModeId[COMPONENT_Y] != lastCheckId )
      {
        csFull->getPredBuf( tu.Y() ).copyFrom( saveCS.getPredBuf( tu.Y() ) );
        csFull->getRecoBuf( tu.Y() ).copyFrom( saveCS.getRecoBuf( tu.Y() ) );

        if( KEEP_PRED_AND_RESI_SIGNALS )
        {
          csFull->getResiBuf   ( tu.Y() ).copyFrom( saveCS.getResiBuf   ( tu.Y() ) );
          csFull->getOrgResiBuf( tu.Y() ).copyFrom( saveCS.getOrgResiBuf( tu.Y() ) );
        }

        tu.copyComponentFrom( *tmpTU, COMPONENT_Y );

        if( !bCheckSplit )
        {
          m_CABACEstimator->getCtx() = ctxBest;
        }
      }
      else if( bCheckSplit )
      {
        ctxBest = m_CABACEstimator->getCtx();
      }

      csFull->cost     += dSingleCost;
      csFull->dist     += uiSingleDistLuma;
      csFull->fracBits += singleFracBits;
    }
  }

  bool validReturnSplit = false;
  if( bCheckSplit )
  {
    //----- store full entropy coding status, load original entropy coding status -----
    if( bCheckFull )
    {
      m_CABACEstimator->getCtx() = ctxStart;
    }
    //----- code splitted block -----
    csSplit->cost = 0;

    bool uiSplitCbfLuma  = false;
    bool splitIsSelected = true;
    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
    {
      partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
    }

    if( cu.ispMode )
    {
      partitioner.splitCurrArea( ispType, *csSplit );
    }
    do
    {
      bool tmpValidReturnSplit = xRecurIntraCodingLumaQT( *csSplit, partitioner, bestCostSoFar, subTuCounter, ispType, false, mtsCheckRangeFlag, mtsFirstCheckId, mtsLastCheckId );
#if SIGN_PREDICTION
#if JVET_Z0118_GDR
      cs.updateReconMotIPM(partitioner.currArea());
#else
      cs.picture->getRecoBuf(  partitioner.currArea()  ).copyFrom( cs.getRecoBuf( partitioner.currArea() ) );
#endif
#endif
      subTuCounter += subTuCounter != -1 ? 1 : 0;
      if( sps.getUseLFNST() && !tmpValidReturnSplit )
      {
        splitIsSelected = false;
        break;
      }

      if( !cu.ispMode )
      {
        csSplit->setDecomp( partitioner.currArea().Y() );
      }
      else if( CU::isISPFirst( cu, partitioner.currArea().Y(), COMPONENT_Y ) )
      {
        csSplit->setDecomp( cu.Y() );
      }

      uiSplitCbfLuma |= TU::getCbfAtDepth( *csSplit->getTU( partitioner.currArea().lumaPos(), partitioner.chType, subTuCounter - 1 ), COMPONENT_Y, partitioner.currTrDepth );
      if( cu.ispMode )
      {
        //exit condition if the accumulated cost is already larger than the best cost so far (no impact in RD performance)
        if( csSplit->cost > bestCostSoFar )
        {
          earlySkipISP    = true;
          splitIsSelected = false;
          break;
        }
        else
        {
          //more restrictive exit condition
          bool tuIsDividedInRows = CU::divideTuInRows( cu );
          int nSubPartitions = tuIsDividedInRows ? cu.lheight() >> floorLog2(cu.firstTU->lheight()) : cu.lwidth() >> floorLog2(cu.firstTU->lwidth());
          double threshold = nSubPartitions == 2 ? 0.95 : subTuCounter == 1 ? 0.83 : 0.91;
          if( subTuCounter < nSubPartitions && csSplit->cost > bestCostSoFar*threshold )
          {
            earlySkipISP    = true;
            splitIsSelected = false;
            break;
          }
        }
      }
    } while( partitioner.nextPart( *csSplit ) );

    partitioner.exitCurrSplit();

    if( splitIsSelected )
    {
      for( auto &ptu : csSplit->tus )
      {
        if( currArea.Y().contains( ptu->Y() ) )
        {
          TU::setCbfAtDepth( *ptu, COMPONENT_Y, currDepth, uiSplitCbfLuma ? 1 : 0 );
        }
      }

      //----- restore context states -----
      m_CABACEstimator->getCtx() = ctxStart;

      cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_LUMA] = false;
      cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_CHROMA] = false;
      cuCtx.lfnstLastScanPos = false;
      cuCtx.violatesMtsCoeffConstraint = false;
      cuCtx.mtsLastScanPos = false;
#if JVET_Y0142_ADAPT_INTRA_MTS
      cuCtx.mtsCoeffAbsSum = 0;
#endif

      //----- determine rate and r-d cost -----
      csSplit->fracBits = xGetIntraFracBitsQT( *csSplit, partitioner, true, false, cu.ispMode ? 0 : -1, ispType, &cuCtx );

      //--- update cost ---
      csSplit->cost     = m_pcRdCost->calcRdCost(csSplit->fracBits, csSplit->dist);

      validReturnSplit = true;
    }
  }

  bool retVal = false;
  if( csFull || csSplit )
  {
    if( !sps.getUseLFNST() || validReturnFull || validReturnSplit )
    {
      // otherwise this would've happened in useSubStructure
#if JVET_Z0118_GDR
      cs.updateReconMotIPM(currArea.Y());
#else
      cs.picture->getRecoBuf(currArea.Y()).copyFrom(cs.getRecoBuf(currArea.Y()));
#endif
      cs.picture->getPredBuf(currArea.Y()).copyFrom(cs.getPredBuf(currArea.Y()));

      if( cu.ispMode && earlySkipISP )
      {
        cs.cost = MAX_DOUBLE;
      }
      else
      {
        cs.cost = m_pcRdCost->calcRdCost( cs.fracBits, cs.dist );
        retVal = true;
      }
    }
  }
  return retVal;
}

bool IntraSearch::xRecurIntraCodingACTQT(CodingStructure &cs, Partitioner &partitioner, bool mtsCheckRangeFlag, int mtsFirstCheckId, int mtsLastCheckId, bool moreProbMTSIdxFirst)
{
  const UnitArea &currArea = partitioner.currArea();
  uint32_t       currDepth = partitioner.currTrDepth;
  const Slice    &slice = *cs.slice;
  const SPS      &sps = *cs.sps;

  bool bCheckFull = !partitioner.canSplit(TU_MAX_TR_SPLIT, cs);
  bool bCheckSplit = !bCheckFull;

  TempCtx ctxStart(m_CtxCache, m_CABACEstimator->getCtx());
  TempCtx ctxBest(m_CtxCache);

  CodingStructure *csSplit = nullptr;
  CodingStructure *csFull = nullptr;
  if (bCheckSplit)
  {
    csSplit = &cs;
  }
  else if (bCheckFull)
  {
    csFull = &cs;
  }

  bool validReturnFull = false;

  if (bCheckFull)
  {
    TransformUnit        &tu = csFull->addTU(CS::getArea(*csFull, currArea, partitioner.chType), partitioner.chType);
    tu.depth = currDepth;
    const CodingUnit     &cu = *csFull->getCU(tu.Y().pos(), CHANNEL_TYPE_LUMA);
    const PredictionUnit &pu = *csFull->getPU(tu.Y().pos(), CHANNEL_TYPE_LUMA);
    CHECK(!tu.Y().valid() || !tu.Cb().valid() || !tu.Cr().valid(), "Invalid TU");
    CHECK(tu.cu != &cu, "wrong CU fetch");
    CHECK(cu.ispMode, "adaptive color transform cannot be applied to ISP");
    CHECK(pu.intraDir[CHANNEL_TYPE_CHROMA] != DM_CHROMA_IDX, "chroma should use DM mode for adaptive color transform");

    // 1. intra prediction and forward color transform

    PelUnitBuf orgBuf = csFull->getOrgBuf(tu);
    PelUnitBuf predBuf = csFull->getPredBuf(tu);
    PelUnitBuf resiBuf = csFull->getResiBuf(tu);
    PelUnitBuf orgResiBuf = csFull->getOrgResiBuf(tu);
#if JVET_S0234_ACT_CRS_FIX
    bool doReshaping = (slice.getLmcsEnabledFlag() && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() && (slice.isIntra() || m_pcReshape->getCTUFlag()) && (tu.blocks[COMPONENT_Cb].width * tu.blocks[COMPONENT_Cb].height > 4));
    if (doReshaping)
    {
      const Area      area = tu.Y().valid() ? tu.Y() : Area(recalcPosition(tu.chromaFormat, tu.chType, CHANNEL_TYPE_LUMA, tu.blocks[tu.chType].pos()), recalcSize(tu.chromaFormat, tu.chType, CHANNEL_TYPE_LUMA, tu.blocks[tu.chType].size()));
      const CompArea &areaY = CompArea(COMPONENT_Y, tu.chromaFormat, area);
      int             adj = m_pcReshape->calculateChromaAdjVpduNei(tu, areaY);
      tu.setChromaAdj(adj);
    }
#endif

    for (int i = 0; i < getNumberValidComponents(tu.chromaFormat); i++)
    {
      ComponentID          compID = (ComponentID)i;
      const CompArea       &area = tu.blocks[compID];
      const ChannelType    chType = toChannelType(compID);

      PelBuf         piOrg = orgBuf.bufs[compID];
      PelBuf         piPred = predBuf.bufs[compID];
      PelBuf         piResi = resiBuf.bufs[compID];

      initIntraPatternChType(*tu.cu, area);
#if JVET_V0130_INTRA_TMP && !JVET_W0069_TMP_BOUNDARY
      if( PU::isTmp( pu, chType ) )
      {
        int foundCandiNum;
        getTargetTemplate( pu.cu, pu.lwidth(), pu.lheight() );
        candidateSearchIntra( pu.cu, pu.lwidth(), pu.lheight() );
        generateTMPrediction( piPred.buf, piPred.stride, pu.lwidth(), pu.lheight(), foundCandiNum );
        CHECK( foundCandiNum < 1, "" );

      }
      else if( PU::isMIP( pu, chType ) )
#else
      if (PU::isMIP(pu, chType))
#endif
      {
        initIntraMip(pu, area);
        predIntraMip(compID, piPred, pu);
      }
      else
      {
        predIntraAng(compID, piPred, pu);
      }

      piResi.copyFrom(piOrg);
      if (slice.getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && compID == COMPONENT_Y)
      {
        CompArea tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
        PelBuf   tmpPred = m_tmpStorageLCU.getBuf(tmpArea);
        piResi.rspSignal( piPred, m_pcReshape->getFwdLUT() );
        piResi.subtract(tmpPred);
      }
#if JVET_S0234_ACT_CRS_FIX
      else if (doReshaping && (compID != COMPONENT_Y))
      {
        piResi.subtract(piPred);
        int cResScaleInv = tu.getChromaAdj();
        piResi.scaleSignal(cResScaleInv, 1, slice.clpRng(compID));
      }
#endif
      else
      {
        piResi.subtract(piPred);
      }
    }

    resiBuf.colorSpaceConvert(orgResiBuf, true, cs.slice->clpRng(COMPONENT_Y));

    // 2. luma residual optimization
    double     dSingleCostLuma = MAX_DOUBLE;
    bool       checkTransformSkip = sps.getTransformSkipEnabledFlag();
    int        bestLumaModeId = 0;
    uint8_t    nNumTransformCands = cu.mtsFlag ? 4 : 1;
    uint8_t    numTransformIndexCands = nNumTransformCands;

    const bool tsAllowed = TU::isTSAllowed(tu, COMPONENT_Y);
    const bool mtsAllowed = CU::isMTSAllowed(cu, COMPONENT_Y);
    std::vector<TrMode> trModes;

    if (sps.getUseLFNST())
    {
      checkTransformSkip &= tsAllowed;
      checkTransformSkip &= !cu.mtsFlag;
      checkTransformSkip &= !cu.lfnstIdx;

      if (!cu.mtsFlag && checkTransformSkip)
      {
        trModes.push_back(TrMode(0, true)); //DCT2
        trModes.push_back(TrMode(1, true)); //TS
      }
    }
    else
    {
      if (m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless())
      {
        nNumTransformCands = 1;
        CHECK(!tsAllowed && !cu.bdpcmMode, "transform skip should be enabled for LS");
        if (cu.bdpcmMode)
        {
          trModes.push_back(TrMode(0, true));
        }
        else
        {
          trModes.push_back(TrMode(1, true));
        }
      }
      else
      {
        nNumTransformCands = 1 + (tsAllowed ? 1 : 0) + (mtsAllowed ? 4 : 0);   // DCT + TS + 4 MTS = 6 tests

        trModes.push_back(TrMode(0, true));   // DCT2
        if (tsAllowed)
        {
          trModes.push_back(TrMode(1, true));
        }
        if (mtsAllowed)
        {
          for (int i = 2; i < 6; i++)
          {
            trModes.push_back(TrMode(i, true));
          }
        }
      }
    }

    CodingStructure &saveLumaCS = *m_pSaveCS[0];
    TransformUnit   *tmpTU = nullptr;
    Distortion      singleDistTmpLuma = 0;
    uint64_t        singleTmpFracBits = 0;
    double          singleCostTmp = 0;
    int             firstCheckId = (sps.getUseLFNST() && mtsCheckRangeFlag && cu.mtsFlag) ? mtsFirstCheckId : 0;
    int             lastCheckId = sps.getUseLFNST() ? ((mtsCheckRangeFlag && cu.mtsFlag) ? (mtsLastCheckId + (int)checkTransformSkip) : (numTransformIndexCands - (firstCheckId + 1) + (int)checkTransformSkip)) : trModes[nNumTransformCands - 1].first;
    bool            isNotOnlyOneMode = sps.getUseLFNST() ? lastCheckId != firstCheckId : nNumTransformCands != 1;

    if (isNotOnlyOneMode)
    {
      saveLumaCS.pcv = csFull->pcv;
      saveLumaCS.picture = csFull->picture;
      saveLumaCS.area.repositionTo(csFull->area);
      saveLumaCS.clearTUs();
      tmpTU = &saveLumaCS.addTU(currArea, partitioner.chType);
    }

    bool    cbfBestMode = false;
    bool    cbfBestModeValid = false;
    bool    cbfDCT2 = true;
    if (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless())
    m_pcRdCost->lambdaAdjustColorTrans(true, COMPONENT_Y);
    for (int modeId = firstCheckId; modeId <= ((m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless()) ? (nNumTransformCands - 1) : lastCheckId); modeId++)
    {
      uint8_t transformIndex = modeId;
      csFull->getResiBuf(tu.Y()).copyFrom(csFull->getOrgResiBuf(tu.Y()));

      m_CABACEstimator->getCtx() = ctxStart;
      m_CABACEstimator->resetBits();

      if (sps.getUseLFNST())
      {
        if ((transformIndex < lastCheckId) || ((transformIndex == lastCheckId) && !checkTransformSkip)) //we avoid this if the mode is transformSkip
        {
          // Skip checking other transform candidates if zero CBF is encountered and it is the best transform so far
          if (m_pcEncCfg->getUseFastLFNST() && transformIndex && !cbfBestMode && cbfBestModeValid)
          {
            continue;
          }
        }
      }
      else
      {
        if (!(m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless()))
        {
          if (!cbfDCT2 || (m_pcEncCfg->getUseTransformSkipFast() && bestLumaModeId == 1))
          {
            break;
          }
          if (!trModes[modeId].second)
          {
            continue;
          }
        }
        tu.mtsIdx[COMPONENT_Y] = trModes[modeId].first;
      }

      singleDistTmpLuma = 0;
      if (sps.getUseLFNST())
      {
        if (cu.mtsFlag)
        {
          if (moreProbMTSIdxFirst)
          {
            uint32_t uiIntraMode = pu.intraDir[CHANNEL_TYPE_LUMA];

            if (transformIndex == 1)
            {
              tu.mtsIdx[COMPONENT_Y] = (uiIntraMode < 34) ? MTS_DST7_DCT8 : MTS_DCT8_DST7;
            }
            else if (transformIndex == 2)
            {
              tu.mtsIdx[COMPONENT_Y] = (uiIntraMode < 34) ? MTS_DCT8_DST7 : MTS_DST7_DCT8;
            }
            else
            {
              tu.mtsIdx[COMPONENT_Y] = MTS_DST7_DST7 + transformIndex;
            }
          }
          else
          {
            tu.mtsIdx[COMPONENT_Y] = MTS_DST7_DST7 + transformIndex;
          }
        }
        else
        {
          tu.mtsIdx[COMPONENT_Y] = transformIndex;
        }

        if (!cu.mtsFlag && checkTransformSkip)
        {
          xIntraCodingACTTUBlock(tu, COMPONENT_Y, singleDistTmpLuma, modeId == 0 ? &trModes : nullptr, true);
          if (modeId == 0)
          {
            for (int i = 0; i < 2; i++)
            {
              if (trModes[i].second)
              {
                lastCheckId = trModes[i].first;
              }
            }
          }
        }
        else
        {
          xIntraCodingACTTUBlock(tu, COMPONENT_Y, singleDistTmpLuma);
        }
      }
      else
      {
        if (nNumTransformCands > 1)
        {
          xIntraCodingACTTUBlock(tu, COMPONENT_Y, singleDistTmpLuma, modeId == 0 ? &trModes : nullptr, true);
          if (modeId == 0)
          {
            for (int i = 0; i < nNumTransformCands; i++)
            {
              if (trModes[i].second)
              {
                lastCheckId = trModes[i].first;
              }
            }
          }
        }
        else
        {
          xIntraCodingACTTUBlock(tu, COMPONENT_Y, singleDistTmpLuma);
        }
      }

      CUCtx cuCtx;
      cuCtx.isDQPCoded = true;
      cuCtx.isChromaQpAdjCoded = true;
      //----- determine rate and r-d cost -----
      if ((sps.getUseLFNST() ? (modeId == lastCheckId && modeId != 0 && checkTransformSkip) : (trModes[modeId].first != 0)) && !TU::getCbfAtDepth(tu, COMPONENT_Y, currDepth))
      {
        //In order not to code TS flag when cbf is zero, the case for TS with cbf being zero is forbidden.
        if (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless())
        singleCostTmp = MAX_DOUBLE;
        else
        {
          singleTmpFracBits = xGetIntraFracBitsQT(*csFull, partitioner, true, false, -1, TU_NO_ISP);
          singleCostTmp = m_pcRdCost->calcRdCost(singleTmpFracBits, singleDistTmpLuma, false);
        }
      }
      else
      {
        singleTmpFracBits = xGetIntraFracBitsQT(*csFull, partitioner, true, false, -1, TU_NO_ISP, &cuCtx);

        if (tu.mtsIdx[COMPONENT_Y] > MTS_SKIP)
        {
#if JVET_Y0142_ADAPT_INTRA_MTS
          int nCands = (cuCtx.mtsCoeffAbsSum > MTS_TH_COEFF[1]) ? MTS_NCANDS[2] : (cuCtx.mtsCoeffAbsSum > MTS_TH_COEFF[0]) ? MTS_NCANDS[1] : MTS_NCANDS[0];
          bool isInvalid = !cuCtx.mtsLastScanPos || ((tu.mtsIdx[COMPONENT_Y] - MTS_DST7_DST7) >= nCands);
          if (isInvalid)
#else
          if (!cuCtx.mtsLastScanPos)
#endif
          {
            singleCostTmp = MAX_DOUBLE;
          }
          else
          {
            singleCostTmp = m_pcRdCost->calcRdCost(singleTmpFracBits, singleDistTmpLuma, false);
          }
        }
        else
        {
          singleCostTmp = m_pcRdCost->calcRdCost(singleTmpFracBits, singleDistTmpLuma, false);
        }
      }

      if (singleCostTmp < dSingleCostLuma)
      {
        dSingleCostLuma = singleCostTmp;
        validReturnFull = true;

        if (sps.getUseLFNST())
        {
          bestLumaModeId = modeId;
          cbfBestMode = TU::getCbfAtDepth(tu, COMPONENT_Y, currDepth);
          cbfBestModeValid = true;
        }
        else
        {
          bestLumaModeId = trModes[modeId].first;
          if (trModes[modeId].first == 0)
          {
            cbfDCT2 = TU::getCbfAtDepth(tu, COMPONENT_Y, currDepth);
          }
        }

        if (bestLumaModeId != lastCheckId)
        {
          saveLumaCS.getResiBuf(tu.Y()).copyFrom(csFull->getResiBuf(tu.Y()));
          tmpTU->copyComponentFrom(tu, COMPONENT_Y);
          ctxBest = m_CABACEstimator->getCtx();
        }
      }
    }
    if (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless())
    m_pcRdCost->lambdaAdjustColorTrans(false, COMPONENT_Y);

    if (sps.getUseLFNST())
    {
      if (!validReturnFull)
      {
        csFull->cost = MAX_DOUBLE;
        return false;
      }
    }
    else
    {
      CHECK(!validReturnFull, "no transform mode was tested for luma");
    }

    csFull->setDecomp(currArea.Y(), true);
    csFull->setDecomp(currArea.Cb(), true);

    if (bestLumaModeId != lastCheckId)
    {
      csFull->getResiBuf(tu.Y()).copyFrom(saveLumaCS.getResiBuf(tu.Y()));
      tu.copyComponentFrom(*tmpTU, COMPONENT_Y);
      m_CABACEstimator->getCtx() = ctxBest;
    }

    // 3 chroma residual optimization
    CodingStructure &saveChromaCS = *m_pSaveCS[1];
    saveChromaCS.pcv = csFull->pcv;
    saveChromaCS.picture = csFull->picture;
    saveChromaCS.area.repositionTo(csFull->area);
    saveChromaCS.initStructData(MAX_INT, true);
    tmpTU = &saveChromaCS.addTU(currArea, partitioner.chType);

    CompArea&  cbArea = tu.blocks[COMPONENT_Cb];
    CompArea&  crArea = tu.blocks[COMPONENT_Cr];

    tu.jointCbCr = 0;

#if !JVET_S0234_ACT_CRS_FIX
    bool doReshaping = (slice.getLmcsEnabledFlag() && slice.getPicHeader()->getLmcsChromaResidualScaleFlag() && (slice.isIntra() || m_pcReshape->getCTUFlag()) && (cbArea.width * cbArea.height > 4));
    if (doReshaping)
    {
#if LMCS_CHROMA_CALC_CU
      const Area      area = tu.cu->Y().valid() ? tu.cu->Y() : Area(recalcPosition(tu.chromaFormat, tu.chType, CHANNEL_TYPE_LUMA, tu.cu->blocks[tu.chType].pos()), recalcSize(tu.chromaFormat, tu.chType, CHANNEL_TYPE_LUMA, tu.cu->blocks[tu.chType].size()));
#else
      const Area      area = tu.Y().valid() ? tu.Y() : Area(recalcPosition(tu.chromaFormat, tu.chType, CHANNEL_TYPE_LUMA, tu.blocks[tu.chType].pos()), recalcSize(tu.chromaFormat, tu.chType, CHANNEL_TYPE_LUMA, tu.blocks[tu.chType].size()));
#endif
      const CompArea &areaY = CompArea(COMPONENT_Y, tu.chromaFormat, area);
      int             adj = m_pcReshape->calculateChromaAdjVpduNei(tu, areaY);
      tu.setChromaAdj(adj);
    }
#endif

    CompStorage  orgResiCb[5], orgResiCr[5]; // 0:std, 1-3:jointCbCr (placeholder at this stage), 4:crossComp
    orgResiCb[0].create(cbArea);
    orgResiCr[0].create(crArea);
    orgResiCb[0].copyFrom(csFull->getOrgResiBuf(cbArea));
    orgResiCr[0].copyFrom(csFull->getOrgResiBuf(crArea));
#if !JVET_S0234_ACT_CRS_FIX
    if (doReshaping)
    {
      int cResScaleInv = tu.getChromaAdj();
      orgResiCb[0].scaleSignal(cResScaleInv, 1, slice.clpRng(COMPONENT_Cb));
      orgResiCr[0].scaleSignal(cResScaleInv, 1, slice.clpRng(COMPONENT_Cr));
    }
#endif

    // 3.1 regular chroma residual coding
    csFull->getResiBuf(cbArea).copyFrom(orgResiCb[0]);
    csFull->getResiBuf(crArea).copyFrom(orgResiCr[0]);

    for (uint32_t c = COMPONENT_Cb; c < ::getNumberValidTBlocks(*csFull->pcv); c++)
    {
      const ComponentID compID = ComponentID(c);
      double  dSingleBestCostChroma = MAX_DOUBLE;
      int     bestModeId = -1;
      bool    tsAllowed = TU::isTSAllowed(tu, compID) && (m_pcEncCfg->getUseChromaTS()) && !cu.lfnstIdx;
      uint8_t numTransformCands = 1 + (tsAllowed ? 1 : 0);  // DCT + TS = 2 tests
      bool        cbfDCT2 = true;

      trModes.clear();
      if (m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless())
      {
        numTransformCands = 1;
        CHECK(!tsAllowed && !cu.bdpcmModeChroma, "transform skip should be enabled for LS");
        if (cu.bdpcmModeChroma)
        {
          trModes.push_back(TrMode(0, true));
        }
        else
        {
          trModes.push_back(TrMode(1, true));
        }
      }
      else
      {
        trModes.push_back(TrMode(0, true));                    // DCT
        if (tsAllowed)
        {
          trModes.push_back(TrMode(1, true));                  // TS
        }
      }
      if (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless())
#if JVET_S0234_ACT_CRS_FIX
      {
        if (doReshaping)
        {
          int cResScaleInv = tu.getChromaAdj();
          m_pcRdCost->lambdaAdjustColorTrans(true, compID, true, &cResScaleInv);
        }
        else
        {
          m_pcRdCost->lambdaAdjustColorTrans(true, compID);
        }
      }
#else
      {
        m_pcRdCost->lambdaAdjustColorTrans(true, compID);
      }
#endif

      TempCtx ctxBegin(m_CtxCache);
      ctxBegin = m_CABACEstimator->getCtx();

      for (int modeId = 0; modeId < numTransformCands; modeId++)
      {
        if (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless())
        {
          if (modeId && !cbfDCT2)
          {
            continue;
          }
          if (!trModes[modeId].second)
          {
            continue;
          }
        }

        if (modeId > 0)
        {
          m_CABACEstimator->getCtx() = ctxBegin;
        }

        tu.mtsIdx[compID] = trModes[modeId].first;
        Distortion singleDistChroma = 0;
        if (numTransformCands > 1)
        {
          xIntraCodingACTTUBlock(tu, compID, singleDistChroma, modeId == 0 ? &trModes : nullptr, true);
        }
        else
        {
          xIntraCodingACTTUBlock(tu, compID, singleDistChroma);
        }
        if (!tu.mtsIdx[compID])
        {
          cbfDCT2 = TU::getCbfAtDepth(tu, compID, currDepth);
        }
        uint64_t fracBitChroma     = xGetIntraFracBitsQTChroma(tu, compID);
        double   dSingleCostChroma = m_pcRdCost->calcRdCost(fracBitChroma, singleDistChroma, false);
        if (dSingleCostChroma < dSingleBestCostChroma)
        {
          dSingleBestCostChroma = dSingleCostChroma;
          bestModeId            = modeId;
          if (bestModeId != (numTransformCands - 1))
          {
            saveChromaCS.getResiBuf(tu.blocks[compID]).copyFrom(csFull->getResiBuf(tu.blocks[compID]));
            tmpTU->copyComponentFrom(tu, compID);
            ctxBest = m_CABACEstimator->getCtx();
          }
        }
      }

      if (bestModeId != (numTransformCands - 1))
      {
        csFull->getResiBuf(tu.blocks[compID]).copyFrom(saveChromaCS.getResiBuf(tu.blocks[compID]));
        tu.copyComponentFrom(*tmpTU, compID);
        m_CABACEstimator->getCtx() = ctxBest;
      }
      if (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless())
      {
        m_pcRdCost->lambdaAdjustColorTrans(false, compID);
      }
    }

    Position tuPos = tu.Y();
    tuPos.relativeTo(cu.Y());
    const UnitArea relativeUnitArea(tu.chromaFormat, Area(tuPos, tu.Y().size()));
    PelUnitBuf     invColorTransResidual = m_colorTransResiBuf.getBuf(relativeUnitArea);
    csFull->getResiBuf(tu).colorSpaceConvert(invColorTransResidual, false, cs.slice->clpRng(COMPONENT_Y));

    Distortion totalDist = 0;
    for (uint32_t c = COMPONENT_Y; c < ::getNumberValidTBlocks(*csFull->pcv); c++)
    {
      const ComponentID compID = ComponentID(c);
      const CompArea&   area = tu.blocks[compID];
      PelBuf            piOrg = csFull->getOrgBuf(area);
      PelBuf            piReco = csFull->getRecoBuf(area);
      PelBuf            piPred = csFull->getPredBuf(area);
      PelBuf            piResi = invColorTransResidual.bufs[compID];

#if JVET_S0234_ACT_CRS_FIX
      if (doReshaping && (compID != COMPONENT_Y))
      {
        piResi.scaleSignal(tu.getChromaAdj(), 0, slice.clpRng(compID));
      }
#endif
      piReco.reconstruct(piPred, piResi, cs.slice->clpRng(compID));

      if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || (m_pcEncCfg->getLmcs()
        & slice.getLmcsEnabledFlag() && (m_pcReshape->getCTUFlag() || (isChroma(compID) && m_pcEncCfg->getReshapeIntraCMD()))))
      {
        const CPelBuf orgLuma = csFull->getOrgBuf(csFull->area.blocks[COMPONENT_Y]);
        if (compID == COMPONENT_Y && !(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()))
        {
          CompArea      tmpArea1(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
          PelBuf tmpRecLuma = m_tmpStorageLCU.getBuf(tmpArea1);
          tmpRecLuma.rspSignal( piReco, m_pcReshape->getInvLUT() );
          totalDist += m_pcRdCost->getDistPart(piOrg, tmpRecLuma, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
        }
        else
        {
          totalDist += m_pcRdCost->getDistPart(piOrg, piReco, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
        }
      }
      else
      {
        totalDist += m_pcRdCost->getDistPart(piOrg, piReco, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE);
      }
    }

    m_CABACEstimator->getCtx() = ctxStart;
    uint64_t totalBits = xGetIntraFracBitsQT(*csFull, partitioner, true, true, -1, TU_NO_ISP);
    double   totalCost = m_pcRdCost->calcRdCost(totalBits, totalDist);

    saveChromaCS.getResiBuf(cbArea).copyFrom(csFull->getResiBuf(cbArea));
    saveChromaCS.getResiBuf(crArea).copyFrom(csFull->getResiBuf(crArea));
    saveChromaCS.getRecoBuf(tu).copyFrom(csFull->getRecoBuf(tu));
    tmpTU->copyComponentFrom(tu, COMPONENT_Cb);
    tmpTU->copyComponentFrom(tu, COMPONENT_Cr);
    ctxBest = m_CABACEstimator->getCtx();

    // 3.2 jointCbCr
    double     bestCostJointCbCr = totalCost;
    Distortion bestDistJointCbCr = totalDist;
    uint64_t   bestBitsJointCbCr = totalBits;
    int        bestJointCbCr = tu.jointCbCr; assert(!bestJointCbCr);

    bool       lastIsBest = false;
    std::vector<int>  jointCbfMasksToTest;
    if (sps.getJointCbCrEnabledFlag() && (TU::getCbf(tu, COMPONENT_Cb) || TU::getCbf(tu, COMPONENT_Cr)))
    {
      jointCbfMasksToTest = m_pcTrQuant->selectICTCandidates(tu, orgResiCb, orgResiCr);
    }

    for (int cbfMask : jointCbfMasksToTest)
    {
      tu.jointCbCr = (uint8_t)cbfMask;

      ComponentID codeCompId = ((cbfMask >> 1) ? COMPONENT_Cb : COMPONENT_Cr);
      ComponentID otherCompId = ((codeCompId == COMPONENT_Cb) ? COMPONENT_Cr : COMPONENT_Cb);
      bool        tsAllowed = TU::isTSAllowed(tu, codeCompId) && (m_pcEncCfg->getUseChromaTS()) && !cu.lfnstIdx;
      uint8_t     numTransformCands = 1 + (tsAllowed ? 1 : 0); // DCT + TS = 2 tests
      bool        cbfDCT2 = true;

      trModes.clear();
      trModes.push_back(TrMode(0, true)); // DCT2
      if (tsAllowed)
      {
        trModes.push_back(TrMode(1, true));//TS
      }

      for (int modeId = 0; modeId < numTransformCands; modeId++)
      {
        if (modeId && !cbfDCT2)
        {
          continue;
        }
        if (!trModes[modeId].second)
        {
          continue;
        }
        Distortion distTmp = 0;
        tu.mtsIdx[codeCompId] = trModes[modeId].first;
        tu.mtsIdx[otherCompId] = MTS_DCT2_DCT2;
        m_CABACEstimator->getCtx() = ctxStart;
        csFull->getResiBuf(cbArea).copyFrom(orgResiCb[cbfMask]);
        csFull->getResiBuf(crArea).copyFrom(orgResiCr[cbfMask]);
        if (nNumTransformCands > 1)
        {
          xIntraCodingACTTUBlock(tu, COMPONENT_Cb, distTmp, modeId == 0 ? &trModes : nullptr, true);
        }
        else
        {
          xIntraCodingACTTUBlock(tu, COMPONENT_Cb, distTmp);
        }

        double   costTmp = std::numeric_limits<double>::max();
        uint64_t bitsTmp = 0;
        if (distTmp < std::numeric_limits<Distortion>::max())
        {
          if (!tu.mtsIdx[codeCompId])
          {
            cbfDCT2 = true;
          }
          csFull->getResiBuf(tu).colorSpaceConvert(invColorTransResidual, false, csFull->slice->clpRng(COMPONENT_Y));
          distTmp = 0;
          for (uint32_t c = COMPONENT_Y; c < ::getNumberValidTBlocks(*csFull->pcv); c++)
          {
            const ComponentID compID = ComponentID(c);
            const CompArea &  area   = tu.blocks[compID];
            PelBuf            piOrg  = csFull->getOrgBuf(area);
            PelBuf            piReco = csFull->getRecoBuf(area);
            PelBuf            piPred = csFull->getPredBuf(area);
            PelBuf            piResi = invColorTransResidual.bufs[compID];

#if JVET_S0234_ACT_CRS_FIX
            if (doReshaping && (compID != COMPONENT_Y))
            {
              piResi.scaleSignal(tu.getChromaAdj(), 0, slice.clpRng(compID));
            }
#endif
            piReco.reconstruct(piPred, piResi, cs.slice->clpRng(compID));
            if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()
                || (m_pcEncCfg->getLmcs() & slice.getLmcsEnabledFlag()
                    && (m_pcReshape->getCTUFlag() || (isChroma(compID) && m_pcEncCfg->getReshapeIntraCMD()))))
            {
              const CPelBuf orgLuma = csFull->getOrgBuf(csFull->area.blocks[COMPONENT_Y]);
              if (compID == COMPONENT_Y && !(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()))
              {
                CompArea tmpArea1(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
                PelBuf   tmpRecLuma = m_tmpStorageLCU.getBuf(tmpArea1);
                tmpRecLuma.rspSignal(piReco, m_pcReshape->getInvLUT());
                distTmp += m_pcRdCost->getDistPart(piOrg, tmpRecLuma, sps.getBitDepth(toChannelType(compID)), compID,
                                                   DF_SSE_WTD, &orgLuma);
              }
              else
              {
                distTmp += m_pcRdCost->getDistPart(piOrg, piReco, sps.getBitDepth(toChannelType(compID)), compID,
                                                   DF_SSE_WTD, &orgLuma);
              }
            }
            else
            {
              distTmp += m_pcRdCost->getDistPart(piOrg, piReco, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE);
            }
          }

          bitsTmp = xGetIntraFracBitsQT(*csFull, partitioner, true, true, -1, TU_NO_ISP);
          costTmp = m_pcRdCost->calcRdCost(bitsTmp, distTmp);
        }
        else if (!tu.mtsIdx[codeCompId])
        {
          cbfDCT2 = false;
        }

        if (costTmp < bestCostJointCbCr)
        {
          bestCostJointCbCr = costTmp;
          bestDistJointCbCr = distTmp;
          bestBitsJointCbCr = bitsTmp;
          bestJointCbCr     = tu.jointCbCr;
          lastIsBest        = (cbfMask == jointCbfMasksToTest.back() && modeId == (numTransformCands - 1));

          // store data
          if (!lastIsBest)
          {
            saveChromaCS.getResiBuf(cbArea).copyFrom(csFull->getResiBuf(cbArea));
            saveChromaCS.getResiBuf(crArea).copyFrom(csFull->getResiBuf(crArea));
            saveChromaCS.getRecoBuf(tu).copyFrom(csFull->getRecoBuf(tu));
            tmpTU->copyComponentFrom(tu, COMPONENT_Cb);
            tmpTU->copyComponentFrom(tu, COMPONENT_Cr);

            ctxBest = m_CABACEstimator->getCtx();
          }
        }
      }
    }

    if (!lastIsBest)
    {
      csFull->getResiBuf(cbArea).copyFrom(saveChromaCS.getResiBuf(cbArea));
      csFull->getResiBuf(crArea).copyFrom(saveChromaCS.getResiBuf(crArea));
      csFull->getRecoBuf(tu).copyFrom(saveChromaCS.getRecoBuf(tu));
      tu.copyComponentFrom(*tmpTU, COMPONENT_Cb);
      tu.copyComponentFrom(*tmpTU, COMPONENT_Cr);

      m_CABACEstimator->getCtx() = ctxBest;
    }
    tu.jointCbCr = bestJointCbCr;
#if JVET_Z0118_GDR
    csFull->updateReconMotIPM(tu);
#else
    csFull->picture->getRecoBuf(tu).copyFrom(csFull->getRecoBuf(tu));
#endif

    csFull->dist += bestDistJointCbCr;
    csFull->fracBits += bestBitsJointCbCr;
    csFull->cost = m_pcRdCost->calcRdCost(csFull->fracBits, csFull->dist);
  }

  bool validReturnSplit = false;
  if (bCheckSplit)
  {
    if (partitioner.canSplit(TU_MAX_TR_SPLIT, *csSplit))
    {
      partitioner.splitCurrArea(TU_MAX_TR_SPLIT, *csSplit);
    }

    bool splitIsSelected = true;
    do
    {
      bool tmpValidReturnSplit = xRecurIntraCodingACTQT(*csSplit, partitioner, mtsCheckRangeFlag, mtsFirstCheckId, mtsLastCheckId, moreProbMTSIdxFirst);
      if (sps.getUseLFNST())
      {
        if (!tmpValidReturnSplit)
        {
          splitIsSelected = false;
          break;
        }
      }
      else
      {
        CHECK(!tmpValidReturnSplit, "invalid RD of sub-TU partitions for ACT");
      }
    } while (partitioner.nextPart(*csSplit));

    partitioner.exitCurrSplit();

    if (splitIsSelected)
    {
      unsigned compCbf[3] = { 0, 0, 0 };
      for (auto &currTU : csSplit->traverseTUs(currArea, partitioner.chType))
      {
        for (unsigned ch = 0; ch < getNumberValidTBlocks(*csSplit->pcv); ch++)
        {
          compCbf[ch] |= (TU::getCbfAtDepth(currTU, ComponentID(ch), currDepth + 1) ? 1 : 0);
        }
      }

      for (auto &currTU : csSplit->traverseTUs(currArea, partitioner.chType))
      {
        TU::setCbfAtDepth(currTU, COMPONENT_Y, currDepth, compCbf[COMPONENT_Y]);
        TU::setCbfAtDepth(currTU, COMPONENT_Cb, currDepth, compCbf[COMPONENT_Cb]);
        TU::setCbfAtDepth(currTU, COMPONENT_Cr, currDepth, compCbf[COMPONENT_Cr]);
      }

      m_CABACEstimator->getCtx() = ctxStart;
      csSplit->fracBits = xGetIntraFracBitsQT(*csSplit, partitioner, true, true, -1, TU_NO_ISP);
      csSplit->cost = m_pcRdCost->calcRdCost(csSplit->fracBits, csSplit->dist);

      validReturnSplit = true;
    }
  }

  bool retVal = false;
  if (csFull || csSplit)
  {
    if (sps.getUseLFNST())
    {
      if (validReturnFull || validReturnSplit)
      {
        retVal = true;
      }
    }
    else
    {
      CHECK(!validReturnFull && !validReturnSplit, "illegal TU optimization");
      retVal = true;
    }
  }
  return retVal;
}

ChromaCbfs IntraSearch::xRecurIntraChromaCodingQT( CodingStructure &cs, Partitioner& partitioner, const double bestCostSoFar, const PartSplit ispType 
#if JVET_AB0143_CCCM_TS
  , PelUnitBuf cccmStorage
#endif
)
{
  UnitArea currArea                   = partitioner.currArea();
  const bool keepResi                 = cs.sps->getUseLMChroma() || KEEP_PRED_AND_RESI_SIGNALS;
  if( !currArea.Cb().valid() ) return ChromaCbfs( false );
  const Slice           &slice = *cs.slice;

  TransformUnit &currTU               = *cs.getTU( currArea.chromaPos(), CHANNEL_TYPE_CHROMA );
  const PredictionUnit &pu            = *cs.getPU( currArea.chromaPos(), CHANNEL_TYPE_CHROMA );

  bool lumaUsesISP                    = false;
  uint32_t     currDepth                  = partitioner.currTrDepth;
  ChromaCbfs cbfs                     ( false );

  if (currDepth == currTU.depth)
  {
    if (!currArea.Cb().valid() || !currArea.Cr().valid())
    {
      return cbfs;
    }

    CodingStructure &saveCS = *m_pSaveCS[1];
    saveCS.pcv      = cs.pcv;
    saveCS.picture  = cs.picture;
#if JVET_Z0118_GDR
    saveCS.m_pt = cs.m_pt;
#endif
    saveCS.area.repositionTo( cs.area );
    saveCS.initStructData( MAX_INT, true );
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if( !currTU.cu->isSepTree() && currTU.cu->ispMode )
#else
    if (!CS::isDualITree(cs) && currTU.cu->ispMode)
#endif
    {
      saveCS.clearCUs();
      CodingUnit& auxCU = saveCS.addCU( *currTU.cu, partitioner.chType );
      auxCU.ispMode = currTU.cu->ispMode;
      saveCS.sps = currTU.cs->sps;
      saveCS.clearPUs();
      saveCS.addPU( *currTU.cu->firstPU, partitioner.chType );
    }

    TransformUnit &tmpTU = saveCS.addTU(currArea, partitioner.chType);

    cs.setDecomp(currArea.Cb(), true); // set in advance (required for Cb2/Cr2 in 4:2:2 video)

    const unsigned      numTBlocks  = ::getNumberValidTBlocks( *cs.pcv );

    CompArea&  cbArea         = currTU.blocks[COMPONENT_Cb];
    CompArea&  crArea         = currTU.blocks[COMPONENT_Cr];
    double     bestCostCb     = MAX_DOUBLE;
    double     bestCostCr     = MAX_DOUBLE;
    Distortion bestDistCb     = 0;
    Distortion bestDistCr     = 0;
    int        maxModesTested = 0;
    bool       earlyExitISP   = false;

    TempCtx ctxStartTU( m_CtxCache );
    TempCtx ctxStart  ( m_CtxCache );
    TempCtx ctxBest   ( m_CtxCache );

    ctxStartTU       = m_CABACEstimator->getCtx();
    currTU.jointCbCr = 0;

    // Do predictions here to avoid repeating the "default0Save1Load2" stuff
    int  predMode   = pu.cu->bdpcmModeChroma ? BDPCM_IDX : PU::getFinalIntraMode(pu, CHANNEL_TYPE_CHROMA);

    PelBuf piPredCb = cs.getPredBuf(cbArea);
    PelBuf piPredCr = cs.getPredBuf(crArea);

    initIntraPatternChType( *currTU.cu, cbArea);
    initIntraPatternChType( *currTU.cu, crArea);

#if JVET_AA0057_CCCM
    if( pu.cccmFlag )
    {
#if JVET_AB0143_CCCM_TS
      if (pu.cs->slice->isIntra())
      {
        piPredCb.copyFrom(cccmStorage.Cb());
        piPredCr.copyFrom(cccmStorage.Cr());
      }
      else
      {
        predIntraCCCM(pu, piPredCb, piPredCr, predMode);
      }
#else
      xGetLumaRecPixels( pu, cbArea );
      predIntraCCCM( pu, piPredCb, piPredCr, predMode );
#endif
    }
    else
#endif
    if( PU::isLMCMode( predMode ) )
    {
      xGetLumaRecPixels( pu, cbArea );
      predIntraChromaLM( COMPONENT_Cb, piPredCb, pu, cbArea, predMode );
#if JVET_AA0126_GLM && !JVET_AB0092_GLM_WITH_LUMA
      xGetLumaRecPixels( pu, crArea ); // generate GLM luma samples for Cr prediction
#endif
      predIntraChromaLM( COMPONENT_Cr, piPredCr, pu, crArea, predMode );
    }
    else if (PU::isMIP(pu, CHANNEL_TYPE_CHROMA))
    {
      initIntraMip(pu, cbArea);
      predIntraMip(COMPONENT_Cb, piPredCb, pu);

      initIntraMip(pu, crArea);
      predIntraMip(COMPONENT_Cr, piPredCr, pu);
    }
    else
    {
      predIntraAng( COMPONENT_Cb, piPredCb, pu);
      predIntraAng( COMPONENT_Cr, piPredCr, pu);
#if JVET_Z0050_DIMD_CHROMA_FUSION
      if (pu.isChromaFusion)
      {
        geneChromaFusionPred(COMPONENT_Cb, piPredCb, pu);
        geneChromaFusionPred(COMPONENT_Cr, piPredCr, pu);
      }
#endif
    }

    // determination of chroma residuals including reshaping and cross-component prediction
    //----- get chroma residuals -----
    PelBuf resiCb  = cs.getResiBuf(cbArea);
    PelBuf resiCr  = cs.getResiBuf(crArea);
    resiCb.copyFrom( cs.getOrgBuf (cbArea) );
    resiCr.copyFrom( cs.getOrgBuf (crArea) );
    resiCb.subtract( piPredCb );
    resiCr.subtract( piPredCr );

    //----- get reshape parameter ----
    bool doReshaping = ( cs.slice->getLmcsEnabledFlag() && cs.picHeader->getLmcsChromaResidualScaleFlag()
                         && (cs.slice->isIntra() || m_pcReshape->getCTUFlag()) && (cbArea.width * cbArea.height > 4) );
    if( doReshaping )
    {
#if LMCS_CHROMA_CALC_CU
      const Area area = currTU.cu->Y().valid() ? currTU.cu->Y() : Area(recalcPosition(currTU.chromaFormat, currTU.chType, CHANNEL_TYPE_LUMA, currTU.cu->blocks[currTU.chType].pos()), recalcSize(currTU.chromaFormat, currTU.chType, CHANNEL_TYPE_LUMA, currTU.cu->blocks[currTU.chType].size()));
#else
      const Area area = currTU.Y().valid() ? currTU.Y() : Area(recalcPosition(currTU.chromaFormat, currTU.chType, CHANNEL_TYPE_LUMA, currTU.blocks[currTU.chType].pos()), recalcSize(currTU.chromaFormat, currTU.chType, CHANNEL_TYPE_LUMA, currTU.blocks[currTU.chType].size()));
#endif
      const CompArea &areaY = CompArea(COMPONENT_Y, currTU.chromaFormat, area);
      int adj = m_pcReshape->calculateChromaAdjVpduNei(currTU, areaY);
      currTU.setChromaAdj(adj);
    }

    //----- get cross component prediction parameters -----
    //===== store original residual signals =====
    CompStorage  orgResiCb[4], orgResiCr[4]; // 0:std, 1-3:jointCbCr (placeholder at this stage)
    orgResiCb[0].create( cbArea );
    orgResiCr[0].create( crArea );
    orgResiCb[0].copyFrom( resiCb );
    orgResiCr[0].copyFrom( resiCr );
    if( doReshaping )
    {
      int cResScaleInv = currTU.getChromaAdj();
      orgResiCb[0].scaleSignal( cResScaleInv, 1, currTU.cu->cs->slice->clpRng(COMPONENT_Cb) );
      orgResiCr[0].scaleSignal( cResScaleInv, 1, currTU.cu->cs->slice->clpRng(COMPONENT_Cr) );
    }

    for( uint32_t c = COMPONENT_Cb; c < numTBlocks; c++)
    {
      const ComponentID compID  = ComponentID(c);
      const CompArea&   area    = currTU.blocks[compID];

      double     dSingleCost    = MAX_DOUBLE;
      int        bestModeId     = 0;
      Distortion singleDistCTmp = 0;
      double     singleCostTmp  = 0;
      const bool tsAllowed = TU::isTSAllowed(currTU, compID) && m_pcEncCfg->getUseChromaTS() && !currTU.cu->lfnstIdx;
      uint8_t nNumTransformCands = 1 + (tsAllowed ? 1 : 0); // DCT + TS = 2 tests
      std::vector<TrMode> trModes;
      if (m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless())
      {
        nNumTransformCands = 1;
        CHECK(!tsAllowed && !currTU.cu->bdpcmModeChroma, "transform skip should be enabled for LS");
        if (currTU.cu->bdpcmModeChroma)
        {
          trModes.push_back(TrMode(0, true));
        }
        else
        {
          trModes.push_back(TrMode(1, true));
        }
      }
      else
      {
        trModes.push_back(TrMode(0, true));   // DCT2

        if (tsAllowed)
        {
          trModes.push_back(TrMode(1, true));   // TS
        }
      }
      CHECK(!currTU.Cb().valid(), "Invalid TU");

      const int  totalModesToTest            = nNumTransformCands;
      bool cbfDCT2 = true;
      const bool isOneMode                   = false;
      maxModesTested                         = totalModesToTest > maxModesTested ? totalModesToTest : maxModesTested;

      int currModeId = 0;
      int default0Save1Load2 = 0;

      if (!isOneMode)
      {
        ctxStart = m_CABACEstimator->getCtx();
      }

      for (int modeId = 0; modeId < nNumTransformCands; modeId++)
      {
        resiCb.copyFrom(orgResiCb[0]);
        resiCr.copyFrom(orgResiCr[0]);
        currTU.mtsIdx[compID] = currTU.cu->bdpcmModeChroma ? MTS_SKIP : trModes[modeId].first;

        currModeId++;

        const bool isFirstMode = (currModeId == 1);
        const bool isLastMode  = false;   // Always store output to saveCS and tmpTU
        if (!(m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless()))
        {
          // if DCT2's cbf==0, skip ts search
          if (!cbfDCT2 && trModes[modeId].first == MTS_SKIP)
          {
              break;
          }
          if (!trModes[modeId].second)
          {
              continue;
          }
        }

        if (!isFirstMode)   // if not first mode to be tested
        {
          m_CABACEstimator->getCtx() = ctxStart;
        }

        singleDistCTmp = 0;

        if (nNumTransformCands > 1)
        {
          xIntraCodingTUBlock(currTU, compID, singleDistCTmp, default0Save1Load2, nullptr,
                              modeId == 0 ? &trModes : nullptr, true);
        }
        else
        {
          xIntraCodingTUBlock(currTU, compID, singleDistCTmp, default0Save1Load2);
        }

        if (((currTU.mtsIdx[compID] == MTS_SKIP && !currTU.cu->bdpcmModeChroma)
             && !TU::getCbf(currTU, compID)))   // In order not to code TS flag when cbf is zero, the case for TS with
                                                // cbf being zero is forbidden.
        {
          if (m_pcEncCfg->getCostMode() != COST_LOSSLESS_CODING || !slice.isLossless())
          {
            singleCostTmp = MAX_DOUBLE;
          }
          else
          {
            uint64_t fracBitsTmp = xGetIntraFracBitsQTChroma(currTU, compID);
            singleCostTmp        = m_pcRdCost->calcRdCost(fracBitsTmp, singleDistCTmp);
          }
        }
        else if (lumaUsesISP && bestCostSoFar != MAX_DOUBLE && c == COMPONENT_Cb)
        {
          uint64_t fracBitsTmp = xGetIntraFracBitsQTSingleChromaComponent(cs, partitioner, ComponentID(c));
          singleCostTmp        = m_pcRdCost->calcRdCost(fracBitsTmp, singleDistCTmp);
          if (isOneMode || (!isOneMode && !isLastMode))
          {
            m_CABACEstimator->getCtx() = ctxStart;
          }
        }
        else if (!isOneMode)
        {
          uint64_t fracBitsTmp = xGetIntraFracBitsQTChroma(currTU, compID);
          singleCostTmp        = m_pcRdCost->calcRdCost(fracBitsTmp, singleDistCTmp);
        }

        if (singleCostTmp < dSingleCost)
        {
          dSingleCost = singleCostTmp;
          bestModeId  = currModeId;

          if (c == COMPONENT_Cb)
          {
            bestCostCb = singleCostTmp;
            bestDistCb = singleDistCTmp;
          }
          else
          {
            bestCostCr = singleCostTmp;
            bestDistCr = singleDistCTmp;
          }

          if (currTU.mtsIdx[compID] == MTS_DCT2_DCT2)
          {
            cbfDCT2 = TU::getCbfAtDepth(currTU, compID, currDepth);
          }

          if (!isLastMode)
          {
#if KEEP_PRED_AND_RESI_SIGNALS
            saveCS.getPredBuf(area).copyFrom(cs.getPredBuf(area));
            saveCS.getOrgResiBuf(area).copyFrom(cs.getOrgResiBuf(area));
#endif
            saveCS.getPredBuf(area).copyFrom(cs.getPredBuf(area));
            if (keepResi)
            {
              saveCS.getResiBuf(area).copyFrom(cs.getResiBuf(area));
            }
            saveCS.getRecoBuf(area).copyFrom(cs.getRecoBuf(area));

            tmpTU.copyComponentFrom(currTU, compID);

            ctxBest = m_CABACEstimator->getCtx();
          }
        }
      }

      if( lumaUsesISP && dSingleCost > bestCostSoFar && c == COMPONENT_Cb )
      {
        //Luma + Cb cost is already larger than the best cost, so we don't need to test Cr
        cs.dist = MAX_UINT;
        m_CABACEstimator->getCtx() = ctxStart;
        earlyExitISP               = true;
        break;
        //return cbfs;
      }

      // Done with one component of separate coding of Cr and Cb, just switch to the best Cb contexts if Cr coding is still to be done
      if ((c == COMPONENT_Cb && bestModeId < totalModesToTest) || (c == COMPONENT_Cb && m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless()))
      {
        m_CABACEstimator->getCtx() = ctxBest;

        currTU.copyComponentFrom(tmpTU, COMPONENT_Cb); // Cbf of Cb is needed to estimate cost for Cr Cbf
      }
    }

    if ( !earlyExitISP )
    {
      // Test using joint chroma residual coding
      double     bestCostCbCr   = bestCostCb + bestCostCr;
      Distortion bestDistCbCr   = bestDistCb + bestDistCr;
      int        bestJointCbCr  = 0;
      std::vector<int>  jointCbfMasksToTest;
      if ( cs.sps->getJointCbCrEnabledFlag() && (TU::getCbf(tmpTU, COMPONENT_Cb) || TU::getCbf(tmpTU, COMPONENT_Cr)))
      {
        jointCbfMasksToTest = m_pcTrQuant->selectICTCandidates(currTU, orgResiCb, orgResiCr);
      }
      bool checkDCTOnly = (TU::getCbf(tmpTU, COMPONENT_Cb) && tmpTU.mtsIdx[COMPONENT_Cb] == MTS_DCT2_DCT2 && !TU::getCbf(tmpTU, COMPONENT_Cr)) ||
                          (TU::getCbf(tmpTU, COMPONENT_Cr) && tmpTU.mtsIdx[COMPONENT_Cr] == MTS_DCT2_DCT2 && !TU::getCbf(tmpTU, COMPONENT_Cb)) ||
                          (TU::getCbf(tmpTU, COMPONENT_Cb) && tmpTU.mtsIdx[COMPONENT_Cb] == MTS_DCT2_DCT2 && TU::getCbf(tmpTU, COMPONENT_Cr) && tmpTU.mtsIdx[COMPONENT_Cr] == MTS_DCT2_DCT2);

      bool checkTSOnly = (TU::getCbf(tmpTU, COMPONENT_Cb) && tmpTU.mtsIdx[COMPONENT_Cb] == MTS_SKIP && !TU::getCbf(tmpTU, COMPONENT_Cr)) ||
                         (TU::getCbf(tmpTU, COMPONENT_Cr) && tmpTU.mtsIdx[COMPONENT_Cr] == MTS_SKIP && !TU::getCbf(tmpTU, COMPONENT_Cb)) ||
                         (TU::getCbf(tmpTU, COMPONENT_Cb) && tmpTU.mtsIdx[COMPONENT_Cb] == MTS_SKIP && TU::getCbf(tmpTU, COMPONENT_Cr) && tmpTU.mtsIdx[COMPONENT_Cr] == MTS_SKIP);

      if (jointCbfMasksToTest.size() && currTU.cu->bdpcmModeChroma)
      {
        CHECK(!checkTSOnly || checkDCTOnly, "bdpcm only allows transform skip");
      }
      for( int cbfMask : jointCbfMasksToTest )
      {

        currTU.jointCbCr               = (uint8_t)cbfMask;
        ComponentID codeCompId = ((currTU.jointCbCr >> 1) ? COMPONENT_Cb : COMPONENT_Cr);
        ComponentID otherCompId = ((codeCompId == COMPONENT_Cb) ? COMPONENT_Cr : COMPONENT_Cb);
        bool        tsAllowed = TU::isTSAllowed(currTU, codeCompId) && (m_pcEncCfg->getUseChromaTS()) && !currTU.cu->lfnstIdx;
        uint8_t     numTransformCands = 1 + (tsAllowed ? 1 : 0); // DCT + TS = 2 tests
        bool        cbfDCT2 = true;

        std::vector<TrMode> trModes;
        if (checkDCTOnly || checkTSOnly)
        {
          numTransformCands = 1;
        }

        if (!checkTSOnly || currTU.cu->bdpcmModeChroma)
        {
          trModes.push_back(TrMode(0, true)); // DCT2
        }
        if (tsAllowed && !checkDCTOnly)
        {
          trModes.push_back(TrMode(1, true));//TS
        }
        for (int modeId = 0; modeId < numTransformCands; modeId++)
        {
          if (modeId && !cbfDCT2)
          {
            continue;
          }
          if (!trModes[modeId].second)
          {
            continue;
          }
          Distortion distTmp = 0;
          currTU.mtsIdx[codeCompId] = currTU.cu->bdpcmModeChroma ? MTS_SKIP : trModes[modeId].first;
          currTU.mtsIdx[otherCompId] = MTS_DCT2_DCT2;
          m_CABACEstimator->getCtx() = ctxStartTU;

          resiCb.copyFrom(orgResiCb[cbfMask]);
          resiCr.copyFrom(orgResiCr[cbfMask]);
          if (numTransformCands > 1)
          {
            xIntraCodingTUBlock(currTU, COMPONENT_Cb, distTmp, 0, nullptr, modeId == 0 ? &trModes : nullptr, true);
          }
          else
          {
            xIntraCodingTUBlock(currTU, COMPONENT_Cb, distTmp, 0);
          }
          double costTmp = std::numeric_limits<double>::max();
          if (distTmp < std::numeric_limits<Distortion>::max())
          {
            uint64_t bits = xGetIntraFracBitsQTChroma(currTU, COMPONENT_Cb);
            costTmp       = m_pcRdCost->calcRdCost(bits, distTmp);
            if (!currTU.mtsIdx[codeCompId])
            {
              cbfDCT2 = true;
            }
          }
          else if (!currTU.mtsIdx[codeCompId])
          {
            cbfDCT2 = false;
          }

          if (costTmp < bestCostCbCr)
          {
            bestCostCbCr  = costTmp;
            bestDistCbCr  = distTmp;
            bestJointCbCr = currTU.jointCbCr;

            // store data
            {
#if KEEP_PRED_AND_RESI_SIGNALS
              saveCS.getOrgResiBuf(cbArea).copyFrom(cs.getOrgResiBuf(cbArea));
              saveCS.getOrgResiBuf(crArea).copyFrom(cs.getOrgResiBuf(crArea));
#endif
              saveCS.getPredBuf(cbArea).copyFrom(cs.getPredBuf(cbArea));
              saveCS.getPredBuf(crArea).copyFrom(cs.getPredBuf(crArea));
              if (keepResi)
              {
                saveCS.getResiBuf(cbArea).copyFrom(cs.getResiBuf(cbArea));
                saveCS.getResiBuf(crArea).copyFrom(cs.getResiBuf(crArea));
              }
              saveCS.getRecoBuf(cbArea).copyFrom(cs.getRecoBuf(cbArea));
              saveCS.getRecoBuf(crArea).copyFrom(cs.getRecoBuf(crArea));

              tmpTU.copyComponentFrom(currTU, COMPONENT_Cb);
              tmpTU.copyComponentFrom(currTU, COMPONENT_Cr);

              ctxBest = m_CABACEstimator->getCtx();
            }
          }
        }
      }

      // Retrieve the best CU data (unless it was the very last one tested)
      {
#if KEEP_PRED_AND_RESI_SIGNALS
        cs.getPredBuf   (cbArea).copyFrom(saveCS.getPredBuf   (cbArea));
        cs.getOrgResiBuf(cbArea).copyFrom(saveCS.getOrgResiBuf(cbArea));
        cs.getPredBuf   (crArea).copyFrom(saveCS.getPredBuf   (crArea));
        cs.getOrgResiBuf(crArea).copyFrom(saveCS.getOrgResiBuf(crArea));
#endif
        cs.getPredBuf   (cbArea).copyFrom(saveCS.getPredBuf   (cbArea));
        cs.getPredBuf   (crArea).copyFrom(saveCS.getPredBuf   (crArea));

        if( keepResi )
        {
          cs.getResiBuf (cbArea).copyFrom(saveCS.getResiBuf   (cbArea));
          cs.getResiBuf (crArea).copyFrom(saveCS.getResiBuf   (crArea));
        }
        cs.getRecoBuf   (cbArea).copyFrom(saveCS.getRecoBuf   (cbArea));
        cs.getRecoBuf   (crArea).copyFrom(saveCS.getRecoBuf   (crArea));

        currTU.copyComponentFrom(tmpTU, COMPONENT_Cb);
        currTU.copyComponentFrom(tmpTU, COMPONENT_Cr);

        m_CABACEstimator->getCtx() = ctxBest;
      }

      // Copy results to the picture structures
#if JVET_Z0118_GDR
      cs.updateReconMotIPM(cbArea);
#else
      cs.picture->getRecoBuf(cbArea).copyFrom(cs.getRecoBuf(cbArea));
#endif

#if JVET_Z0118_GDR
      cs.updateReconMotIPM(crArea);
#else
      cs.picture->getRecoBuf(crArea).copyFrom(cs.getRecoBuf(crArea));
#endif
      cs.picture->getPredBuf(cbArea).copyFrom(cs.getPredBuf(cbArea));
      cs.picture->getPredBuf(crArea).copyFrom(cs.getPredBuf(crArea));

      cbfs.cbf(COMPONENT_Cb) = TU::getCbf(currTU, COMPONENT_Cb);
      cbfs.cbf(COMPONENT_Cr) = TU::getCbf(currTU, COMPONENT_Cr);

      currTU.jointCbCr = ( (cbfs.cbf(COMPONENT_Cb) + cbfs.cbf(COMPONENT_Cr)) ? bestJointCbCr : 0 );
      cs.dist         += bestDistCbCr;
    }
  }
  else
  {
    unsigned    numValidTBlocks   = ::getNumberValidTBlocks( *cs.pcv );
    ChromaCbfs  SplitCbfs         ( false );

    if( partitioner.canSplit( TU_MAX_TR_SPLIT, cs ) )
    {
      partitioner.splitCurrArea( TU_MAX_TR_SPLIT, cs );
    }
    else if( currTU.cu->ispMode )
    {
      partitioner.splitCurrArea( ispType, cs );
    }
    else
    {
      THROW( "Implicit TU split not available" );
    }

    do
    {
      ChromaCbfs subCbfs = xRecurIntraChromaCodingQT( cs, partitioner, bestCostSoFar, ispType );

      for( uint32_t ch = COMPONENT_Cb; ch < numValidTBlocks; ch++ )
      {
        const ComponentID compID = ComponentID( ch );
        SplitCbfs.cbf( compID ) |= subCbfs.cbf( compID );
      }
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();

    if( lumaUsesISP && cs.dist == MAX_UINT )
    {
      return cbfs;
    }
    cbfs.Cb |= SplitCbfs.Cb;
    cbfs.Cr |= SplitCbfs.Cr;

    if (!lumaUsesISP)
    {
      for (auto &ptu: cs.tus)
      {
        if (currArea.Cb().contains(ptu->Cb()) || (!ptu->Cb().valid() && currArea.Y().contains(ptu->Y())))
        {
          TU::setCbfAtDepth(*ptu, COMPONENT_Cb, currDepth, SplitCbfs.Cb);
          TU::setCbfAtDepth(*ptu, COMPONENT_Cr, currDepth, SplitCbfs.Cr);
        }
      }
    }
  }

  return cbfs;
}

uint64_t IntraSearch::xFracModeBitsIntra(PredictionUnit &pu, const uint32_t &uiMode, const ChannelType &chType)
{
  uint8_t orgMode = uiMode;

#if JVET_Y0065_GPM_INTRA
  if (!pu.ciipFlag && !pu.gpmIntraFlag)
#else
  if (!pu.ciipFlag)
#endif
  std::swap(orgMode, pu.intraDir[chType]);

  m_CABACEstimator->resetBits();

  if( isLuma( chType ) )
  {
#if JVET_Y0065_GPM_INTRA
    if (!pu.ciipFlag && !pu.gpmIntraFlag)
#else
    if (!pu.ciipFlag)
#endif
    {
      m_CABACEstimator->intra_luma_pred_mode(pu);
    }
  }
  else
  {
    m_CABACEstimator->intra_chroma_pred_mode( pu );
  }

#if JVET_Y0065_GPM_INTRA
  if ( !pu.ciipFlag && !pu.gpmIntraFlag )
#else
  if ( !pu.ciipFlag )
#endif
  std::swap(orgMode, pu.intraDir[chType]);

  return m_CABACEstimator->getEstFracBits();
}

void IntraSearch::sortRdModeListFirstColorSpace(ModeInfo mode, double cost, char bdpcmMode, ModeInfo* rdModeList, double* rdCostList, char* bdpcmModeList, int& candNum)
{
  if (candNum == 0)
  {
    rdModeList[0] = mode;
    rdCostList[0] = cost;
    bdpcmModeList[0] = bdpcmMode;
    candNum++;
    return;
  }

  int insertPos = -1;
  for (int pos = candNum - 1; pos >= 0; pos--)
  {
    if (cost < rdCostList[pos])
    {
      insertPos = pos;
    }
  }

  if (insertPos >= 0)
  {
    for (int i = candNum - 1; i >= insertPos; i--)
    {
      rdModeList[i + 1] = rdModeList[i];
      rdCostList[i + 1] = rdCostList[i];
      bdpcmModeList[i + 1] = bdpcmModeList[i];
    }
    rdModeList[insertPos] = mode;
    rdCostList[insertPos] = cost;
    bdpcmModeList[insertPos] = bdpcmMode;
    candNum++;
  }
  else
  {
    rdModeList[candNum] = mode;
    rdCostList[candNum] = cost;
    bdpcmModeList[candNum] = bdpcmMode;
    candNum++;
  }

  CHECK(candNum > FAST_UDI_MAX_RDMODE_NUM, "exceed intra mode candidate list capacity");

  return;
}

void IntraSearch::invalidateBestRdModeFirstColorSpace()
{
  int numSaveRdClass = 4 * NUM_LFNST_NUM_PER_SET * 2;
  int savedRdModeListSize = FAST_UDI_MAX_RDMODE_NUM;

  for (int i = 0; i < numSaveRdClass; i++)
  {
    m_numSavedRdModeFirstColorSpace[i] = 0;
    for (int j = 0; j < savedRdModeListSize; j++)
    {
      m_savedRdModeFirstColorSpace[i][j] = ModeInfo(false, false, 0, NOT_INTRA_SUBPARTITIONS, 0);
      m_savedBDPCMModeFirstColorSpace[i][j] = 0;
      m_savedRdCostFirstColorSpace[i][j] = MAX_DOUBLE;
    }
  }
}

template<typename T, size_t N>
void IntraSearch::reduceHadCandList(static_vector<T, N>& candModeList, static_vector<double, N>& candCostList, int& numModesForFullRD, const double thresholdHadCost, const double* mipHadCost, const PredictionUnit &pu, const bool fastMip)
{
  const int maxCandPerType = numModesForFullRD >> 1;
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM> tempRdModeList;
  static_vector<double, FAST_UDI_MAX_RDMODE_NUM> tempCandCostList;
  const double minCost = candCostList[0];
  bool keepOneMip = candModeList.size() > numModesForFullRD;

  int numConv = 0;
  int numMip = 0;
  for (int idx = 0; idx < candModeList.size() - (keepOneMip?0:1); idx++)
  {
    bool addMode = false;
    const ModeInfo& orgMode = candModeList[idx];

    if (!orgMode.mipFlg)
    {
      addMode = (numConv < 3);
      numConv += addMode ? 1:0;
    }
    else
    {
      addMode = ( numMip < maxCandPerType || (candCostList[idx] < thresholdHadCost * minCost) || keepOneMip );
      keepOneMip = false;
      numMip += addMode ? 1:0;
    }
    if( addMode )
    {
      tempRdModeList.push_back(orgMode);
      tempCandCostList.push_back(candCostList[idx]);
    }
  }

  if ((pu.lwidth() > 8 && pu.lheight() > 8))
  {
    // Sort MIP candidates by Hadamard cost
    const int transpOff = getNumModesMip( pu.Y() );
    static_vector<uint8_t, FAST_UDI_MAX_RDMODE_NUM> sortedMipModes(0);
    static_vector<double, FAST_UDI_MAX_RDMODE_NUM> sortedMipCost(0);
    for( uint8_t mode : { 0, 1, 2 } )
    {
      uint8_t candMode = mode + uint8_t((mipHadCost[mode + transpOff] < mipHadCost[mode]) ? transpOff : 0);
      updateCandList(candMode, mipHadCost[candMode], sortedMipModes, sortedMipCost, 3);
    }

    // Append MIP mode to RD mode list
    const int modeListSize = int(tempRdModeList.size());
    for (int idx = 0; idx < 3; idx++)
    {
      const bool     isTransposed = (sortedMipModes[idx] >= transpOff ? true : false);
      const uint32_t mipIdx       = (isTransposed ? sortedMipModes[idx] - transpOff : sortedMipModes[idx]);
      const ModeInfo mipMode( true, isTransposed, 0, NOT_INTRA_SUBPARTITIONS, mipIdx );
      bool alreadyIncluded = false;
      for (int modeListIdx = 0; modeListIdx < modeListSize; modeListIdx++)
      {
        if (tempRdModeList[modeListIdx] == mipMode)
        {
          alreadyIncluded = true;
          break;
        }
      }

      if (!alreadyIncluded)
      {
        tempRdModeList.push_back(mipMode);
        tempCandCostList.push_back(0);
        if( fastMip ) break;
      }
    }
  }

  candModeList = tempRdModeList;
  candCostList = tempCandCostList;
  numModesForFullRD = int(candModeList.size());
}

// It decides which modes from the ISP lists can be full RD tested
void IntraSearch::xGetNextISPMode(ModeInfo& modeInfo, const ModeInfo* lastMode, const Size cuSize)
{
  static_vector<ModeInfo, FAST_UDI_MAX_RDMODE_NUM>* rdModeLists[2] = { &m_ispCandListHor, &m_ispCandListVer };

  const int curIspLfnstIdx = m_curIspLfnstIdx;
  if (curIspLfnstIdx >= NUM_LFNST_NUM_PER_SET)
  {
    //All lfnst indices have been checked
    return;
  }

  ISPType nextISPcandSplitType;
  auto& ispTestedModes = m_ispTestedModes[curIspLfnstIdx];
  const bool horSplitIsTerminated = ispTestedModes.splitIsFinished[HOR_INTRA_SUBPARTITIONS - 1];
  const bool verSplitIsTerminated = ispTestedModes.splitIsFinished[VER_INTRA_SUBPARTITIONS - 1];
  if (!horSplitIsTerminated && !verSplitIsTerminated)
  {
    nextISPcandSplitType = !lastMode ? HOR_INTRA_SUBPARTITIONS : lastMode->ispMod == HOR_INTRA_SUBPARTITIONS ? VER_INTRA_SUBPARTITIONS : HOR_INTRA_SUBPARTITIONS;
  }
  else if (!horSplitIsTerminated && verSplitIsTerminated)
  {
    nextISPcandSplitType = HOR_INTRA_SUBPARTITIONS;
  }
  else if (horSplitIsTerminated && !verSplitIsTerminated)
  {
    nextISPcandSplitType = VER_INTRA_SUBPARTITIONS;
  }
  else
  {
    xFinishISPModes();
    return;   // no more modes will be tested
  }

  int maxNumSubPartitions = ispTestedModes.numTotalParts[nextISPcandSplitType - 1];

  // We try to break the split here for lfnst > 0 according to the first mode
  if (curIspLfnstIdx > 0 && ispTestedModes.numTestedModes[nextISPcandSplitType - 1] == 1)
  {
    int firstModeThisSplit = ispTestedModes.getTestedIntraMode(nextISPcandSplitType, 0);
    int numSubPartsFirstModeThisSplit = ispTestedModes.getNumCompletedSubParts(nextISPcandSplitType, firstModeThisSplit);
    CHECK(numSubPartsFirstModeThisSplit < 0, "wrong number of subpartitions!");
    bool stopThisSplit = false;
    bool stopThisSplitAllLfnsts = false;
    if (numSubPartsFirstModeThisSplit < maxNumSubPartitions)
    {
      stopThisSplit = true;
      if (m_pcEncCfg->getUseFastISP() && curIspLfnstIdx == 1 && numSubPartsFirstModeThisSplit < maxNumSubPartitions - 1)
      {
        stopThisSplitAllLfnsts = true;
      }
    }

    if (stopThisSplit)
    {
      ispTestedModes.splitIsFinished[nextISPcandSplitType - 1] = true;
      if (curIspLfnstIdx == 1 && stopThisSplitAllLfnsts)
      {
        m_ispTestedModes[2].splitIsFinished[nextISPcandSplitType - 1] = true;
      }
      return;
    }
  }

  // We try to break the split here for lfnst = 0 or all lfnst indices according to the first two modes
  if (curIspLfnstIdx == 0 && ispTestedModes.numTestedModes[nextISPcandSplitType - 1] == 2)
  {
    // Split stop criteria after checking the performance of previously tested intra modes
    const int thresholdSplit1 = maxNumSubPartitions;
    bool stopThisSplit = false;
    bool stopThisSplitForAllLFNSTs = false;
    const int thresholdSplit1ForAllLFNSTs = maxNumSubPartitions - 1;

    int mode1 = ispTestedModes.getTestedIntraMode((ISPType)nextISPcandSplitType, 0);
#if ENABLE_DIMD && !JVET_V0087_DIMD_NO_ISP
    mode1 = ( mode1 == DC_IDX || mode1 == DIMD_IDX ) ? -1 : mode1;
#else
    mode1 = mode1 == DC_IDX ? -1 : mode1;
#endif
    int numSubPartsBestMode1 = mode1 != -1 ? ispTestedModes.getNumCompletedSubParts((ISPType)nextISPcandSplitType, mode1) : -1;
    int mode2 = ispTestedModes.getTestedIntraMode((ISPType)nextISPcandSplitType, 1);
#if ENABLE_DIMD && !JVET_V0087_DIMD_NO_ISP
    mode2 = ( mode2 == DC_IDX || mode2 == DIMD_IDX ) ? -1 : mode2;
#else
    mode2 = mode2 == DC_IDX ? -1 : mode2;
#endif
    int numSubPartsBestMode2 = mode2 != -1 ? ispTestedModes.getNumCompletedSubParts((ISPType)nextISPcandSplitType, mode2) : -1;

    // 1) The 2 most promising modes do not reach a certain number of sub-partitions
    if (numSubPartsBestMode1 != -1 && numSubPartsBestMode2 != -1)
    {
      if (numSubPartsBestMode1 < thresholdSplit1 && numSubPartsBestMode2 < thresholdSplit1)
      {
        stopThisSplit = true;
        if (curIspLfnstIdx == 0 && numSubPartsBestMode1 < thresholdSplit1ForAllLFNSTs && numSubPartsBestMode2 < thresholdSplit1ForAllLFNSTs)
        {
          stopThisSplitForAllLFNSTs = true;
        }
      }
      else
      {
        //we stop also if the cost is MAX_DOUBLE for both modes
        double mode1Cost = ispTestedModes.getRDCost(nextISPcandSplitType, mode1);
        double mode2Cost = ispTestedModes.getRDCost(nextISPcandSplitType, mode2);
        if (!(mode1Cost < MAX_DOUBLE || mode2Cost < MAX_DOUBLE))
        {
          stopThisSplit = true;
        }
      }
    }

    if (!stopThisSplit)
    {
      // 2) One split type may be discarded by comparing the number of sub-partitions of the best angle modes of both splits
      ISPType otherSplit = nextISPcandSplitType == HOR_INTRA_SUBPARTITIONS ? VER_INTRA_SUBPARTITIONS : HOR_INTRA_SUBPARTITIONS;
      int  numSubPartsBestMode2OtherSplit = mode2 != -1 ? ispTestedModes.getNumCompletedSubParts(otherSplit, mode2) : -1;
      if (numSubPartsBestMode2OtherSplit != -1 && numSubPartsBestMode2 != -1 && ispTestedModes.bestSplitSoFar != nextISPcandSplitType)
      {
        if (numSubPartsBestMode2OtherSplit > numSubPartsBestMode2)
        {
          stopThisSplit = true;
        }
        // both have the same number of subpartitions
        else if (numSubPartsBestMode2OtherSplit == numSubPartsBestMode2)
        {
          // both have the maximum number of subpartitions, so it compares RD costs to decide
          if (numSubPartsBestMode2OtherSplit == maxNumSubPartitions)
          {
            double rdCostBestMode2ThisSplit = ispTestedModes.getRDCost(nextISPcandSplitType, mode2);
            double rdCostBestMode2OtherSplit = ispTestedModes.getRDCost(otherSplit, mode2);
            double threshold = 1.3;
            if (rdCostBestMode2ThisSplit == MAX_DOUBLE || rdCostBestMode2OtherSplit < rdCostBestMode2ThisSplit * threshold)
            {
              stopThisSplit = true;
            }
          }
          else // none of them reached the maximum number of subpartitions with the best angle modes, so it compares the results with the the planar mode
          {
            int  numSubPartsBestMode1OtherSplit = mode1 != -1 ? ispTestedModes.getNumCompletedSubParts(otherSplit, mode1) : -1;
            if (numSubPartsBestMode1OtherSplit != -1 && numSubPartsBestMode1 != -1 && numSubPartsBestMode1OtherSplit > numSubPartsBestMode1)
            {
              stopThisSplit = true;
            }
          }
        }
      }
    }
    if (stopThisSplit)
    {
      ispTestedModes.splitIsFinished[nextISPcandSplitType - 1] = true;
      if (stopThisSplitForAllLFNSTs)
      {
        for (int lfnstIdx = 1; lfnstIdx < NUM_LFNST_NUM_PER_SET; lfnstIdx++)
        {
          m_ispTestedModes[lfnstIdx].splitIsFinished[nextISPcandSplitType - 1] = true;
        }
      }
      return;
    }
  }

  // Now a new mode is retrieved from the list and it has to be decided whether it should be tested or not
  if (ispTestedModes.candIndexInList[nextISPcandSplitType - 1] < rdModeLists[nextISPcandSplitType - 1]->size())
  {
    ModeInfo candidate = rdModeLists[nextISPcandSplitType - 1]->at(ispTestedModes.candIndexInList[nextISPcandSplitType - 1]);
    ispTestedModes.candIndexInList[nextISPcandSplitType - 1]++;

    // extra modes are only tested if ISP has won so far
    if (ispTestedModes.candIndexInList[nextISPcandSplitType - 1] > ispTestedModes.numOrigModesToTest)
    {
      if (ispTestedModes.bestSplitSoFar != candidate.ispMod || ispTestedModes.bestModeSoFar == PLANAR_IDX)
      {
        ispTestedModes.splitIsFinished[nextISPcandSplitType - 1] = true;
        return;
      }
    }

    bool testCandidate = true;

    // we look for a reference mode that has already been tested within the window and decide to test the new one according to the reference mode costs
    if (
#if ENABLE_DIMD && !JVET_V0087_DIMD_NO_ISP
      candidate.modeId != DIMD_IDX &&
#endif
#if JVET_W0123_TIMD_FUSION
      candidate.modeId != TIMD_IDX &&
#endif
      maxNumSubPartitions > 2 && (curIspLfnstIdx > 0 || (candidate.modeId >= DC_IDX && ispTestedModes.numTestedModes[nextISPcandSplitType - 1] >= 2)))
    {
      int       refLfnstIdx = -1;
      const int angWindowSize = 5;
      int       numSubPartsLeftMode, numSubPartsRightMode, numSubPartsRefMode, leftIntraMode = -1, rightIntraMode = -1;
      int       windowSize = candidate.modeId > DC_IDX ? angWindowSize : 1;
      int       numSamples = cuSize.width << floorLog2(cuSize.height);
      int       numSubPartsLimit = numSamples >= 256 ? maxNumSubPartitions - 1 : 2;

      xFindAlreadyTestedNearbyIntraModes(curIspLfnstIdx, (int)candidate.modeId, &refLfnstIdx, &leftIntraMode, &rightIntraMode, (ISPType)candidate.ispMod, windowSize);

      if (refLfnstIdx != -1 && refLfnstIdx != curIspLfnstIdx)
      {
        CHECK(leftIntraMode != candidate.modeId || rightIntraMode != candidate.modeId, "wrong intra mode and lfnstIdx values!");
        numSubPartsRefMode = m_ispTestedModes[refLfnstIdx].getNumCompletedSubParts((ISPType)candidate.ispMod, candidate.modeId);
        CHECK(numSubPartsRefMode <= 0, "Wrong value of the number of subpartitions completed!");
      }
      else
      {
        numSubPartsLeftMode = leftIntraMode != -1 ? ispTestedModes.getNumCompletedSubParts((ISPType)candidate.ispMod, leftIntraMode) : -1;
        numSubPartsRightMode = rightIntraMode != -1 ? ispTestedModes.getNumCompletedSubParts((ISPType)candidate.ispMod, rightIntraMode) : -1;

        numSubPartsRefMode = std::max(numSubPartsLeftMode, numSubPartsRightMode);
      }

      if (numSubPartsRefMode > 0)
      {
        // The mode was found. Now we check the condition
        testCandidate = numSubPartsRefMode > numSubPartsLimit;
      }
    }

    if (testCandidate)
    {
      modeInfo = candidate;
    }
  }
  else
  {
    //the end of the list was reached, so the split is invalidated
    ispTestedModes.splitIsFinished[nextISPcandSplitType - 1] = true;
  }
}

void IntraSearch::xFindAlreadyTestedNearbyIntraModes(int lfnstIdx, int currentIntraMode, int* refLfnstIdx, int* leftIntraMode, int* rightIntraMode, ISPType ispOption, int windowSize)
{
  bool leftModeFound = false, rightModeFound = false;
  *leftIntraMode = -1;
  *rightIntraMode = -1;
  *refLfnstIdx = -1;
  const unsigned st = ispOption - 1;

  //first we check if the exact intra mode was already tested for another lfnstIdx value
  if (lfnstIdx > 0)
  {
    bool sameIntraModeFound = false;
    if (lfnstIdx == 2 && m_ispTestedModes[1].modeHasBeenTested[currentIntraMode][st])
    {
      sameIntraModeFound = true;
      *refLfnstIdx = 1;
    }
    else if (m_ispTestedModes[0].modeHasBeenTested[currentIntraMode][st])
    {
      sameIntraModeFound = true;
      *refLfnstIdx = 0;
    }

    if (sameIntraModeFound)
    {
      *leftIntraMode = currentIntraMode;
      *rightIntraMode = currentIntraMode;
      return;
    }
  }

  //The mode has not been checked for another lfnstIdx value, so now we look for a similar mode within a window using the same lfnstIdx
  for (int k = 1; k <= windowSize; k++)
  {
    int off = currentIntraMode - 2 - k;
    int leftMode = (off < 0) ? NUM_LUMA_MODE + off : currentIntraMode - k;
    int rightMode = currentIntraMode > DC_IDX ? (((int)currentIntraMode - 2 + k) % 65) + 2 : PLANAR_IDX;

    leftModeFound  = leftMode  != (int)currentIntraMode ? m_ispTestedModes[lfnstIdx].modeHasBeenTested[leftMode][st]  : false;
    rightModeFound = rightMode != (int)currentIntraMode ? m_ispTestedModes[lfnstIdx].modeHasBeenTested[rightMode][st] : false;
    if (leftModeFound || rightModeFound)
    {
      *leftIntraMode = leftModeFound ? leftMode : -1;
      *rightIntraMode = rightModeFound ? rightMode : -1;
      *refLfnstIdx = lfnstIdx;
      break;
    }
  }
}

//It prepares the list of potential intra modes candidates that will be tested using RD costs
bool IntraSearch::xSortISPCandList(double bestCostSoFar, double bestNonISPCost, ModeInfo bestNonISPMode)
{
  int bestISPModeInRelCU = -1;
  m_modeCtrl->setStopNonDCT2Transforms(false);

  if (m_pcEncCfg->getUseFastISP())
  {
    //we check if the ISP tests can be cancelled
    double thSkipISP = 1.4;
    if (bestNonISPCost > bestCostSoFar * thSkipISP)
    {
      for (int splitIdx = 0; splitIdx < NUM_INTRA_SUBPARTITIONS_MODES - 1; splitIdx++)
      {
        for (int j = 0; j < NUM_LFNST_NUM_PER_SET; j++)
        {
          m_ispTestedModes[j].splitIsFinished[splitIdx] = true;
        }
      }
      return false;
    }
    if (!updateISPStatusFromRelCU(bestNonISPCost, bestNonISPMode, bestISPModeInRelCU))
    {
      return false;
    }
  }

  for (int k = 0; k < m_ispCandListHor.size(); k++)
  {
    m_ispCandListHor.at(k).ispMod = HOR_INTRA_SUBPARTITIONS; //we set the correct ISP split type value
  }

  auto origHadList = m_ispCandListHor;   // save the original hadamard list of regular intra
  bool modeIsInList[NUM_LUMA_MODE] = { false };

  m_ispCandListHor.clear();
  m_ispCandListVer.clear();

  // we sort the normal intra modes according to their full RD costs
  std::stable_sort(m_regIntraRDListWithCosts.begin(), m_regIntraRDListWithCosts.end(), ModeInfoWithCost::compareModeInfoWithCost);

  // we get the best angle from the regular intra list
  int bestNormalIntraAngle = -1;
  for (int modeIdx = 0; modeIdx < m_regIntraRDListWithCosts.size(); modeIdx++)
  {
    if (bestNormalIntraAngle == -1 && m_regIntraRDListWithCosts.at(modeIdx).modeId > DC_IDX)
    {
      bestNormalIntraAngle = m_regIntraRDListWithCosts.at(modeIdx).modeId;
      break;
    }
  }

  int mode1 = PLANAR_IDX;
  int mode2 = bestNormalIntraAngle;

  ModeInfo refMode = origHadList.at(0);
  auto* destListPtr = &m_ispCandListHor;
  //List creation

  if (m_pcEncCfg->getUseFastISP() && bestISPModeInRelCU != -1) //RelCU intra mode
  {
    destListPtr->push_back(
      ModeInfo(refMode.mipFlg, refMode.mipTrFlg, refMode.mRefId, refMode.ispMod, bestISPModeInRelCU));
    modeIsInList[bestISPModeInRelCU] = true;
  }
  // Planar
#if JVET_W0103_INTRA_MTS
  // push planar later when FastISP is on.
  if (!m_pcEncCfg->getUseFastISP() && !modeIsInList[mode1])
#else
  if (!modeIsInList[mode1])
#endif
  {
    destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mipTrFlg, refMode.mRefId, refMode.ispMod, mode1));
    modeIsInList[mode1] = true;
  }

  // Best angle in regular intra
  if (mode2 != -1 && !modeIsInList[mode2])
  {
    destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mipTrFlg, refMode.mRefId, refMode.ispMod, mode2));
    modeIsInList[mode2] = true;
  }
  // Remaining regular intra modes that were full RD tested (except DC, which is added after the angles from regular intra)
  int dcModeIndex = -1;
  for (int remModeIdx = 0; remModeIdx < m_regIntraRDListWithCosts.size(); remModeIdx++)
  {
    int currentMode = m_regIntraRDListWithCosts.at(remModeIdx).modeId;
    if (currentMode != mode1 && currentMode != mode2 && !modeIsInList[currentMode])
    {
      if (currentMode > DC_IDX)
      {
        destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mipTrFlg, refMode.mRefId, refMode.ispMod, currentMode));
        modeIsInList[currentMode] = true;
      }
      else if (currentMode == DC_IDX)
      {
        dcModeIndex = remModeIdx;
      }
    }
  }
#if JVET_W0103_INTRA_MTS
  // Planar (after angular modes when FastISP is on)
  if (!modeIsInList[mode1])
  {
    destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mipTrFlg, refMode.mRefId, refMode.ispMod, mode1));
    modeIsInList[mode1] = true;
  }
#endif
  // DC is added after the angles from regular intra
  if (dcModeIndex != -1 && !modeIsInList[DC_IDX])
  {
    destListPtr->push_back(ModeInfo(refMode.mipFlg, refMode.mipTrFlg, refMode.mRefId, refMode.ispMod, DC_IDX));
    modeIsInList[DC_IDX] = true;
  }

  // We add extra candidates to the list that will only be tested if ISP is likely to win
  for (int j = 0; j < NUM_LFNST_NUM_PER_SET; j++)
  {
    m_ispTestedModes[j].numOrigModesToTest = (int)destListPtr->size();
#if JVET_W0103_INTRA_MTS
    if (m_pcEncCfg->getUseFastISP() && m_numModesISPRDO != -1 && destListPtr->size() > m_numModesISPRDO)
    {
      m_ispTestedModes[j].numOrigModesToTest = m_numModesISPRDO;
    }
#endif
  }
  const int addedModesFromHadList = 3;
  int       newModesAdded = 0;

  for (int k = 0; k < origHadList.size(); k++)
  {
    if (newModesAdded == addedModesFromHadList)
    {
      break;
    }
    if (
#if ENABLE_DIMD && !JVET_V0087_DIMD_NO_ISP
      origHadList.at(k).modeId == DIMD_IDX ||
#endif
#if JVET_W0123_TIMD_FUSION
      origHadList.at(k).modeId == TIMD_IDX ||
#endif
	!modeIsInList[origHadList.at(k).modeId])
    {
      destListPtr->push_back( ModeInfo( refMode.mipFlg, refMode.mipTrFlg, refMode.mRefId, refMode.ispMod, origHadList.at(k).modeId ) );
      newModesAdded++;
    }
  }

  if (m_pcEncCfg->getUseFastISP() && bestISPModeInRelCU != -1)
  {
    destListPtr->resize(1);
  }

  // Copy modes to other split-type list
  m_ispCandListVer = m_ispCandListHor;
  for (int i = 0; i < m_ispCandListVer.size(); i++)
  {
    m_ispCandListVer[i].ispMod = VER_INTRA_SUBPARTITIONS;
  }

  // Reset the tested modes information to 0
  for (int j = 0; j < NUM_LFNST_NUM_PER_SET; j++)
  {
    for (int i = 0; i < m_ispCandListHor.size(); i++)
    {
      m_ispTestedModes[j].clearISPModeInfo(m_ispCandListHor[i].modeId);
    }
  }
  return true;
}

void IntraSearch::xSortISPCandListLFNST()
{
  //It resorts the list of intra mode candidates for lfnstIdx > 0 by checking the RD costs for lfnstIdx = 0
  ISPTestedModesInfo& ispTestedModesRef = m_ispTestedModes[0];
  for (int splitIdx = 0; splitIdx < NUM_INTRA_SUBPARTITIONS_MODES - 1; splitIdx++)
  {
    ISPType ispMode = splitIdx ? VER_INTRA_SUBPARTITIONS : HOR_INTRA_SUBPARTITIONS;
    if (!m_ispTestedModes[m_curIspLfnstIdx].splitIsFinished[splitIdx] && ispTestedModesRef.testedModes[splitIdx].size() > 1)
    {
      auto& candList   = ispMode == HOR_INTRA_SUBPARTITIONS ? m_ispCandListHor : m_ispCandListVer;
      int bestModeId   = candList[1].modeId > DC_IDX ? candList[1].modeId : -1;
      int bestSubParts = candList[1].modeId > DC_IDX ? ispTestedModesRef.getNumCompletedSubParts(ispMode, bestModeId) : -1;
      double bestCost  = candList[1].modeId > DC_IDX ? ispTestedModesRef.getRDCost(ispMode, bestModeId) : MAX_DOUBLE;
      for (int i = 0; i < candList.size(); i++)
      {
#if ENABLE_DIMD && !JVET_V0087_DIMD_NO_ISP
        if( candList[i].modeId == DIMD_IDX )
        {
          continue;
        }
#endif
#if JVET_W0123_TIMD_FUSION
        if( candList[i].modeId == TIMD_IDX )
        {
          continue;
        }
#endif
        const int candSubParts = ispTestedModesRef.getNumCompletedSubParts(ispMode, candList[i].modeId);
        const double candCost = ispTestedModesRef.getRDCost(ispMode, candList[i].modeId);
        if (candSubParts > bestSubParts || candCost < bestCost)
        {
          bestModeId = candList[i].modeId;
          bestCost = candCost;
          bestSubParts = candSubParts;
        }
      }

      if (bestModeId != -1)
      {
        if (bestModeId != candList[0].modeId)
        {
          auto prevMode = candList[0];
          candList[0].modeId = bestModeId;
          for (int i = 1; i < candList.size(); i++)
          {
            auto nextMode = candList[i];
            candList[i] = prevMode;
            if (nextMode.modeId == bestModeId)
            {
              break;
            }
            prevMode = nextMode;
          }
        }
      }
    }
  }
}

bool IntraSearch::updateISPStatusFromRelCU( double bestNonISPCostCurrCu, ModeInfo bestNonISPModeCurrCu, int& bestISPModeInRelCU )
{
  //It compares the data of a related CU with the current CU to cancel or reduce the ISP tests
  bestISPModeInRelCU = -1;
  if (m_modeCtrl->getRelatedCuIsValid())
  {
    double bestNonISPCostRelCU = m_modeCtrl->getBestDCT2NonISPCostRelCU();
    double costRatio           = bestNonISPCostCurrCu / bestNonISPCostRelCU;
    bool   bestModeRelCuIsMip  = (m_modeCtrl->getIspPredModeValRelCU() >> 5) & 0x1;
    bool   bestModeCurrCuIsMip = bestNonISPModeCurrCu.mipFlg;
    int    relatedCuIntraMode  = m_modeCtrl->getIspPredModeValRelCU() >> 9;
    bool   isSameTypeOfMode    = (bestModeRelCuIsMip && bestModeCurrCuIsMip) || (!bestModeRelCuIsMip && !bestModeCurrCuIsMip);
    bool   bothModesAreAngular = bestNonISPModeCurrCu.modeId > DC_IDX && relatedCuIntraMode > DC_IDX;
    bool   modesAreComparable  = isSameTypeOfMode && (bestModeCurrCuIsMip || bestNonISPModeCurrCu.modeId == relatedCuIntraMode || (bothModesAreAngular && abs(relatedCuIntraMode - (int)bestNonISPModeCurrCu.modeId) <= 5));
    int    status              = m_modeCtrl->getIspPredModeValRelCU();

    if ((status & 0x3) == 0x3) //ISP was not selected in the relCU
    {
      double bestNonDCT2Cost = m_modeCtrl->getBestNonDCT2Cost();
      double ratioWithNonDCT2 = bestNonDCT2Cost / bestNonISPCostRelCU;
      double margin = ratioWithNonDCT2 < 0.95 ? 0.2 : 0.1;

      if (costRatio > 1 - margin && costRatio < 1 + margin && modesAreComparable)
      {
        for (int lfnstVal = 0; lfnstVal < NUM_LFNST_NUM_PER_SET; lfnstVal++)
        {
          m_ispTestedModes[lfnstVal].splitIsFinished[HOR_INTRA_SUBPARTITIONS - 1] = true;
          m_ispTestedModes[lfnstVal].splitIsFinished[VER_INTRA_SUBPARTITIONS - 1] = true;
        }
        return false;
      }
    }
    else if ((status & 0x3) == 0x1) //ISP was selected in the relCU
    {
      double margin = 0.05;

      if (costRatio > 1 - margin && costRatio < 1 + margin && modesAreComparable)
      {
        int  ispSplitIdx = (m_modeCtrl->getIspPredModeValRelCU() >> 2) & 0x1;
        bool lfnstIdxIsNot0 = (bool)((m_modeCtrl->getIspPredModeValRelCU() >> 3) & 0x1);
        bool lfnstIdxIs2 = (bool)((m_modeCtrl->getIspPredModeValRelCU() >> 4) & 0x1);
        int  lfnstIdx = !lfnstIdxIsNot0 ? 0 : lfnstIdxIs2 ? 2 : 1;
        bestISPModeInRelCU = (int)m_modeCtrl->getBestISPIntraModeRelCU();

        for (int splitIdx = 0; splitIdx < NUM_INTRA_SUBPARTITIONS_MODES - 1; splitIdx++)
        {
          for (int lfnstVal = 0; lfnstVal < NUM_LFNST_NUM_PER_SET; lfnstVal++)
          {
            if (lfnstVal == lfnstIdx && splitIdx == ispSplitIdx)
            {
              continue;
            }
            m_ispTestedModes[lfnstVal].splitIsFinished[splitIdx] = true;
          }
        }

        bool stopNonDCT2Transforms = (bool)((m_modeCtrl->getIspPredModeValRelCU() >> 6) & 0x1);
        m_modeCtrl->setStopNonDCT2Transforms(stopNonDCT2Transforms);
      }
    }
    else
    {
      THROW("Wrong ISP relCU status");
    }
  }

  return true;
}

void IntraSearch::xFinishISPModes()
{
  //Continue to the next lfnst index
  m_curIspLfnstIdx++;

  if (m_curIspLfnstIdx < NUM_LFNST_NUM_PER_SET)
  {
    //Check if LFNST is applicable
    if (m_curIspLfnstIdx == 1)
    {
      bool canTestLFNST = false;
      for (int lfnstIdx = 1; lfnstIdx < NUM_LFNST_NUM_PER_SET; lfnstIdx++)
      {
        canTestLFNST |= !m_ispTestedModes[lfnstIdx].splitIsFinished[HOR_INTRA_SUBPARTITIONS - 1] || !m_ispTestedModes[lfnstIdx].splitIsFinished[VER_INTRA_SUBPARTITIONS - 1];
      }
      if (canTestLFNST)
      {
        //Construct the intra modes candidates list for the lfnst > 0 cases
        xSortISPCandListLFNST();
      }
    }
  }
}

