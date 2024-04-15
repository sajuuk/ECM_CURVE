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

/** \file     EncCu.cpp
    \brief    Coding Unit (CU) encoder class
*/

#include "EncCu.h"

#include "EncLib.h"
#include "Analyze.h"
#include "AQp.h"

#include "CommonLib/dtrace_codingstruct.h"
#include "CommonLib/Picture.h"
#include "CommonLib/UnitTools.h"
#include "MCTS.h"


#include "CommonLib/dtrace_buffer.h"

#include <stdio.h>
#include <cmath>
#include <algorithm>


//! \ingroup EncoderLib
//! \{

// ====================================================================================================================
#if JVET_Y0065_GPM_INTRA
EncCu::EncCu()
#else
EncCu::EncCu() : m_GeoModeTest
{
  GeoMotionInfo(0, 1), GeoMotionInfo(1, 0),GeoMotionInfo(0, 2), GeoMotionInfo(1, 2), GeoMotionInfo(2, 0),
  GeoMotionInfo(2, 1), GeoMotionInfo(0, 3),GeoMotionInfo(1, 3), GeoMotionInfo(2, 3), GeoMotionInfo(3, 0),
  GeoMotionInfo(3, 1), GeoMotionInfo(3, 2),GeoMotionInfo(0, 4), GeoMotionInfo(1, 4), GeoMotionInfo(2, 4),
  GeoMotionInfo(3, 4), GeoMotionInfo(4, 0),GeoMotionInfo(4, 1), GeoMotionInfo(4, 2), GeoMotionInfo(4, 3),
  GeoMotionInfo(0, 5), GeoMotionInfo(1, 5),GeoMotionInfo(2, 5), GeoMotionInfo(3, 5), GeoMotionInfo(4, 5),
  GeoMotionInfo(5, 0), GeoMotionInfo(5, 1),GeoMotionInfo(5, 2), GeoMotionInfo(5, 3), GeoMotionInfo(5, 4)
}
#endif
{
#if NON_ADJACENT_MRG_CAND
  int numGeoModeTest = 0;
#if JVET_Y0065_GPM_INTRA
  for (int i = 1; i < GEO_MAX_NUM_UNI_CANDS+GEO_MAX_NUM_INTRA_CANDS; i++)
#else
  for (int i = 1; i < GEO_MAX_NUM_UNI_CANDS; i++)
#endif
  {
    for (int j = 0; j < i; j++)
    {
      m_GeoModeTest[numGeoModeTest] = GeoMotionInfo(j, i);
      numGeoModeTest++;
    }
    for (int j = 0; j < i; j++)
    {
      m_GeoModeTest[numGeoModeTest] = GeoMotionInfo(i, j);
      numGeoModeTest++;
    }
  }
#endif
#if JVET_W0097_GPM_MMVD_TM
  m_fastGpmMmvdSearch = false;
  m_fastGpmMmvdRelatedCU = false;
  m_includeMoreMMVDCandFirstPass = false;
  m_maxNumGPMDirFirstPass = 64;
  m_numCandPerPar = 5;
#endif
}

void EncCu::create( EncCfg* encCfg )
{
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
  m_bilateralFilter = new BilateralFilter();;
  m_bilateralFilter->create();
#endif

  unsigned      uiMaxWidth    = encCfg->getMaxCUWidth();
  unsigned      uiMaxHeight   = encCfg->getMaxCUHeight();
  ChromaFormat  chromaFormat  = encCfg->getChromaFormatIdc();

  unsigned      numWidths     = gp_sizeIdxInfo->numWidths();
  unsigned      numHeights    = gp_sizeIdxInfo->numHeights();
  m_pTempCS = new CodingStructure**  [numWidths];
  m_pBestCS = new CodingStructure**  [numWidths];
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  m_pTempCS2 = new CodingStructure** [numWidths];
  m_pBestCS2 = new CodingStructure** [numWidths];
#endif
#if ENABLE_OBMC
  m_tempWoOBMCBuffer.create(UnitArea(chromaFormat, Area(0, 0, uiMaxWidth, uiMaxHeight)));
#endif
  for( unsigned w = 0; w < numWidths; w++ )
  {
    m_pTempCS[w] = new CodingStructure*  [numHeights];
    m_pBestCS[w] = new CodingStructure*  [numHeights];
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    m_pTempCS2[w] = new CodingStructure* [numHeights];
    m_pBestCS2[w] = new CodingStructure* [numHeights];
#endif
    for( unsigned h = 0; h < numHeights; h++ )
    {
      unsigned width  = gp_sizeIdxInfo->sizeFrom( w );
      unsigned height = gp_sizeIdxInfo->sizeFrom( h );

      if (gp_sizeIdxInfo->isCuSize( width ) && gp_sizeIdxInfo->isCuSize( height )
        && width <= uiMaxWidth && height <= uiMaxHeight)
      {
        m_pTempCS[w][h] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
        m_pBestCS[w][h] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );

#if JVET_Z0118_GDR
        m_pTempCS[w][h]->create(chromaFormat, Area(0, 0, width, height), false, (bool)encCfg->getPLTMode(), encCfg->getGdrEnabled());
        m_pBestCS[w][h]->create(chromaFormat, Area(0, 0, width, height), false, (bool)encCfg->getPLTMode(), encCfg->getGdrEnabled());
#else        
        m_pTempCS[w][h]->create(chromaFormat, Area(0, 0, width, height), false, (bool)encCfg->getPLTMode());
        m_pBestCS[w][h]->create(chromaFormat, Area(0, 0, width, height), false, (bool)encCfg->getPLTMode());
#endif
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
        m_pTempCS2[w][h] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );
        m_pBestCS2[w][h] = new CodingStructure( m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache );

        m_pTempCS2[w][h]->create(chromaFormat, Area(0, 0, width, height), false, (bool)encCfg->getPLTMode());
        m_pBestCS2[w][h]->create(chromaFormat, Area(0, 0, width, height), false, (bool)encCfg->getPLTMode());
#endif
      }
      else
      {
        m_pTempCS[w][h] = nullptr;
        m_pBestCS[w][h] = nullptr;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
        m_pTempCS2[w][h] = nullptr;
        m_pBestCS2[w][h] = nullptr;
#endif
      }
    }
  }
#if ENABLE_OBMC
  m_pTempCUWoOBMC = nullptr;
  m_pPredBufWoOBMC = nullptr;

  if (encCfg->getUseOBMC())
  {
    m_pTempCUWoOBMC = new CodingStructure**[numWidths];
    m_pPredBufWoOBMC = new PelStorage*[numWidths];

    for (unsigned w = 0; w < numWidths; w++)
    {
      m_pTempCUWoOBMC[w] = new CodingStructure*[numHeights];
      m_pPredBufWoOBMC[w] = new PelStorage[numHeights];

      for (unsigned h = 0; h < numHeights; h++)
      {
        uint32_t width = gp_sizeIdxInfo->sizeFrom(w);
        uint32_t height = gp_sizeIdxInfo->sizeFrom(h);

        if (gp_sizeIdxInfo->isCuSize(width) && gp_sizeIdxInfo->isCuSize(height)
          && width <= uiMaxWidth && height <= uiMaxHeight)
        {
          m_pTempCUWoOBMC[w][h] = new CodingStructure(m_unitCache.cuCache, m_unitCache.puCache, m_unitCache.tuCache);
#if JVET_Z0118_GDR
          m_pTempCUWoOBMC[w][h]->create(chromaFormat, Area(0, 0, width, height), false, false, encCfg->getGdrEnabled());
#else
          m_pTempCUWoOBMC[w][h]->create(chromaFormat, Area(0, 0, width, height), false, false);
#endif
          m_pPredBufWoOBMC[w][h].create(UnitArea(chromaFormat, Area(0, 0, width, height)));
        }
        else
        {
          m_pTempCUWoOBMC[w][h] = nullptr;
        }
      }
    }
  }
#endif
  m_cuChromaQpOffsetIdxPlus1 = 0;

  unsigned maxDepth = numWidths + numHeights;

  m_modeCtrl = new EncModeCtrlMTnoRQT();

  m_modeCtrl->create( *encCfg );

#if JVET_Y0065_GPM_INTRA
  for (unsigned ui = 0; ui < GEO_NUM_RDO_BUFFER; ui++)
#else
  for (unsigned ui = 0; ui < MMVD_MRG_MAX_RD_BUF_NUM; ui++)
#endif
  {
    m_acMergeBuffer[ui].create( chromaFormat, Area( 0, 0, uiMaxWidth, uiMaxHeight ) );
  }
  for (unsigned ui = 0; ui < MRG_MAX_NUM_CANDS; ui++)
  {
    m_acRealMergeBuffer[ui].create(chromaFormat, Area(0, 0, uiMaxWidth, uiMaxHeight));
#if INTER_LIC || MULTI_HYP_PRED
    m_acRealMergeBuffer[ui+MRG_MAX_NUM_CANDS].create(chromaFormat, Area(0, 0, uiMaxWidth, uiMaxHeight));
#if JVET_AD0213_LIC_IMP
    m_acRealMergeBuffer[ui + (MRG_MAX_NUM_CANDS << 1)].create(chromaFormat, Area(0, 0, uiMaxWidth, uiMaxHeight));
#endif
#endif
    m_acMergeTmpBuffer[ui].create(chromaFormat, Area(0, 0, uiMaxWidth, uiMaxHeight));
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
    m_acTmMergeTmpBuffer[ui].create(chromaFormat, Area(0, 0, uiMaxWidth, uiMaxHeight));
#endif
  }
#if JVET_AG0135_AFFINE_CIIP
  for (unsigned ui = 0; ui < AFFINE_MRG_MAX_NUM_CANDS; ui++)
  {
    m_acMergeAffineBuffer[ui].create(chromaFormat, Area(0, 0, uiMaxWidth, uiMaxHeight));
  }
#endif 
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && JVET_AC0112_IBC_LIC
  m_acMergeTmpBuffer[MRG_MAX_NUM_CANDS].create(chromaFormat, Area(0, 0, uiMaxWidth, uiMaxHeight));
#endif

#if JVET_Y0065_GPM_INTRA
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
  for (unsigned ui = 0; ui < GEO_MAX_TRY_WEIGHTED_SAD*GEO_BLENDING_NUM+1; ui++)
#else
  for( unsigned ui = 0; ui < GEO_MAX_TRY_WEIGHTED_SAD+1; ui++ )
#endif
#else
  for( unsigned ui = 0; ui < GEO_MAX_TRY_WEIGHTED_SAD; ui++ )
#endif
  {
    m_acGeoWeightedBuffer[ui].create( chromaFormat, Area( 0, 0, uiMaxWidth, uiMaxHeight ) );
  }
#if JVET_W0097_GPM_MMVD_TM
#if JVET_AG0164_AFFINE_GPM
  for (unsigned ui = 0; ui < GEO_MAX_ALL_INTER_UNI_CANDS; ui++)
  {
    for (unsigned vi = 0; vi < GPM_EXT_MMVD_MAX_REFINE_NUM; vi++)
    {
      if (ui >= GEO_MAX_NUM_UNI_CANDS && vi > 0)
      {
        continue;
      }
      m_acGeoMMVDBuffer[ui][vi].create(chromaFormat, Area(0, 0, std::min((int)uiMaxWidth, GEO_MAX_CU_SIZE), std::min((int)uiMaxHeight, GEO_MAX_CU_SIZE)));
      m_acGeoMMVDTmpBuffer[ui][vi].create(chromaFormat,Area(0, 0, std::min((int)uiMaxWidth, GEO_MAX_CU_SIZE), std::min((int)uiMaxHeight, GEO_MAX_CU_SIZE)));
    }
  }
#else
  for (unsigned ui = 0; ui < MRG_MAX_NUM_CANDS; ui++)
  {
    for (unsigned vi = 0; vi < GPM_EXT_MMVD_MAX_REFINE_NUM; vi++)
    {
      m_acGeoMMVDBuffer[ui][vi].create(chromaFormat, Area(0, 0, uiMaxWidth, uiMaxHeight));
      m_acGeoMMVDTmpBuffer[ui][vi].create(chromaFormat, Area(0, 0, uiMaxWidth, uiMaxHeight));
    }
  }
#endif
  int sourceWidth = encCfg->getSourceWidth();
  int sourceHeight = encCfg->getSourceHeight();
  m_fastGpmMmvdSearch = (((encCfg->getIntraPeriod() > 0) && ((sourceWidth * sourceHeight) <= (1920 * 1080))) || ((encCfg->getIntraPeriod() < 0) && ((sourceWidth * sourceHeight) >= (1280 * 720)))) && !encCfg->getIBCMode();
#if TM_MRG
  m_fastGpmMmvdRelatedCU = ((encCfg->getIntraPeriod() < 0) && ((sourceWidth * sourceHeight) >= (1920 * 1080))) && !encCfg->getIBCMode();
#else
  m_fastGpmMmvdRelatedCU = ((encCfg->getIntraPeriod() < 0) && ((sourceWidth * sourceHeight) >= (1280 * 720))) && !encCfg->getIBCMode();
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && TM_MRG
  if (!encCfg->getUseTMMrgMode())
  {
    m_fastGpmMmvdRelatedCU = ((encCfg->getIntraPeriod() < 0) && ((sourceWidth * sourceHeight) >= (1280 * 720))) && !encCfg->getIBCMode();
  }
#endif

  m_includeMoreMMVDCandFirstPass = ((encCfg->getIntraPeriod() > 0) || ((encCfg->getIntraPeriod() < 0) && m_fastGpmMmvdSearch));
  m_maxNumGPMDirFirstPass = ((encCfg->getIntraPeriod() < 0) ? 50 : (m_fastGpmMmvdSearch ? 36 : 64));
  m_numCandPerPar = (m_fastGpmMmvdSearch ? 4 : 5);
#if TM_MRG
  for (uint16_t ui = 0; ui < GEO_TM_MAX_NUM_CANDS; ui++)
  {
    m_acGeoMergeTmpBuffer[ui].create(chromaFormat, Area(0, 0, uiMaxWidth, uiMaxHeight));
    m_acGeoSADTmpBuffer[ui].create(chromaFormat, Area(0, 0, uiMaxWidth, uiMaxHeight));
  }
#endif
#endif
  m_ciipBuffer[0].create(chromaFormat, Area(0, 0, uiMaxWidth, uiMaxHeight));
  m_ciipBuffer[1].create(chromaFormat, Area(0, 0, uiMaxWidth, uiMaxHeight));

  m_ctxBuffer.resize( maxDepth );
  m_CurrCtx = 0;
}


void EncCu::destroy()
{
  unsigned numWidths  = gp_sizeIdxInfo->numWidths();
  unsigned numHeights = gp_sizeIdxInfo->numHeights();

#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
  m_bilateralFilter->destroy();
  delete m_bilateralFilter;
#endif

#if ENABLE_OBMC
  m_tempWoOBMCBuffer.destroy();
#endif
  for( unsigned w = 0; w < numWidths; w++ )
  {
    for( unsigned h = 0; h < numHeights; h++ )
    {
      if( m_pBestCS[w][h] ) m_pBestCS[w][h]->destroy();
      if( m_pTempCS[w][h] ) m_pTempCS[w][h]->destroy();

      delete m_pBestCS[w][h];
      delete m_pTempCS[w][h];
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if( m_pBestCS2[w][h] ) m_pBestCS2[w][h]->destroy();
      if( m_pTempCS2[w][h] ) m_pTempCS2[w][h]->destroy();

      delete m_pBestCS2[w][h];
      delete m_pTempCS2[w][h];
#endif
    }

    delete[] m_pTempCS[w];
    delete[] m_pBestCS[w];
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    delete[] m_pTempCS2[w];
    delete[] m_pBestCS2[w];
#endif
  }

  delete[] m_pBestCS; m_pBestCS = nullptr;
  delete[] m_pTempCS; m_pTempCS = nullptr;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  delete[] m_pBestCS2; m_pBestCS2 = nullptr;
  delete[] m_pTempCS2; m_pTempCS2 = nullptr;
#endif
#if REUSE_CU_RESULTS
  if (m_tmpStorageLCU)
  {
    m_tmpStorageLCU->destroy();
    delete m_tmpStorageLCU;  m_tmpStorageLCU = nullptr;
  }
#endif

#if REUSE_CU_RESULTS
  m_modeCtrl->destroy();

#endif
  delete m_modeCtrl;
  m_modeCtrl = nullptr;
#if ENABLE_OBMC
  if (m_pTempCUWoOBMC)
  {
    for (unsigned w = 0; w < numWidths; w++)
    {
      for (unsigned h = 0; h < numHeights; h++)
      {
        if( gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( w ) ) && gp_sizeIdxInfo->isCuSize( gp_sizeIdxInfo->sizeFrom( h ) )
          && gp_sizeIdxInfo->sizeFrom(w) <= m_pcEncCfg->getMaxCUWidth() && gp_sizeIdxInfo->sizeFrom(h) <= m_pcEncCfg->getMaxCUHeight())
        {
          m_pTempCUWoOBMC[w][h]->destroy();
          delete m_pTempCUWoOBMC[w][h];

          m_pPredBufWoOBMC[w][h].destroy();
        }
      }
      delete[] m_pTempCUWoOBMC[w];
      delete[] m_pPredBufWoOBMC[w];
    }

    delete[] m_pTempCUWoOBMC;
    m_pTempCUWoOBMC = nullptr;

    delete[] m_pPredBufWoOBMC;
    m_pPredBufWoOBMC = nullptr;
  }

#endif
#if JVET_Y0065_GPM_INTRA
  for (unsigned ui = 0; ui < GEO_NUM_RDO_BUFFER; ui++)
#else
  for (unsigned ui = 0; ui < MMVD_MRG_MAX_RD_BUF_NUM; ui++)
#endif
  {
    m_acMergeBuffer[ui].destroy();
  }
  for (unsigned ui = 0; ui < MRG_MAX_NUM_CANDS; ui++)
  {
    m_acRealMergeBuffer[ui].destroy();
#if INTER_LIC || MULTI_HYP_PRED
    m_acRealMergeBuffer[ui+MRG_MAX_NUM_CANDS].destroy();
#if JVET_AD0213_LIC_IMP
    m_acRealMergeBuffer[ui + (MRG_MAX_NUM_CANDS << 1)].destroy();
#endif
#endif
    m_acMergeTmpBuffer[ui].destroy();
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
    m_acTmMergeTmpBuffer[ui].destroy();
#endif
  }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && JVET_AC0112_IBC_LIC
  m_acMergeTmpBuffer[MRG_MAX_NUM_CANDS].destroy();
#endif
#if JVET_AG0135_AFFINE_CIIP
  for (unsigned ui = 0; ui < AFFINE_MRG_MAX_NUM_CANDS; ui++)
  {
    m_acMergeAffineBuffer[ui].destroy();
  }
#endif 
#if JVET_Y0065_GPM_INTRA
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
  for (unsigned ui = 0; ui < GEO_MAX_TRY_WEIGHTED_SAD*GEO_BLENDING_NUM+1; ui++)
#else
  for (unsigned ui = 0; ui < GEO_MAX_TRY_WEIGHTED_SAD+1; ui++)
#endif
#else
  for (unsigned ui = 0; ui < GEO_MAX_TRY_WEIGHTED_SAD; ui++)
#endif
  {
    m_acGeoWeightedBuffer[ui].destroy();
  }
#if JVET_W0097_GPM_MMVD_TM
#if JVET_AG0164_AFFINE_GPM
  for (unsigned ui = 0; ui < GEO_MAX_ALL_INTER_UNI_CANDS; ui++)
  {
    for (unsigned vi = 0; vi < GPM_EXT_MMVD_MAX_REFINE_NUM; vi++)
    {
      if (ui >= GEO_MAX_NUM_UNI_CANDS && vi > 0)
      {
        continue;
      }
      m_acGeoMMVDBuffer[ui][vi].destroy();
      m_acGeoMMVDTmpBuffer[ui][vi].destroy();
    }
  }
#else
  for (unsigned ui = 0; ui < MRG_MAX_NUM_CANDS; ui++)
  {
    for (unsigned vi = 0; vi < GPM_EXT_MMVD_MAX_REFINE_NUM; vi++)
    {
      m_acGeoMMVDBuffer[ui][vi].destroy();
      m_acGeoMMVDTmpBuffer[ui][vi].destroy();
    }
  }
#endif

#if TM_MRG
  for (uint16_t ui = 0; ui < GEO_TM_MAX_NUM_CANDS; ui++)
  {
    m_acGeoMergeTmpBuffer[ui].destroy();
    m_acGeoSADTmpBuffer[ui].destroy();
  }
#endif
#endif
  m_ciipBuffer[0].destroy();
  m_ciipBuffer[1].destroy();
}



EncCu::~EncCu()
{
}



/** \param    pcEncLib      pointer of encoder class
 */
void EncCu::init( EncLib* pcEncLib, const SPS& sps PARL_PARAM( const int tId ) )
{
  m_pcEncCfg           = pcEncLib;
  m_pcIntraSearch      = pcEncLib->getIntraSearch( PARL_PARAM0( tId ) );
  m_pcInterSearch      = pcEncLib->getInterSearch( PARL_PARAM0( tId ) );
  m_pcTrQuant          = pcEncLib->getTrQuant( PARL_PARAM0( tId ) );
  m_pcRdCost           = pcEncLib->getRdCost ( PARL_PARAM0( tId ) );
  m_CABACEstimator     = pcEncLib->getCABACEncoder( PARL_PARAM0( tId ) )->getCABACEstimator( &sps );
  m_CABACEstimator->setEncCu(this);
  m_ctxCache           = pcEncLib->getCtxCache( PARL_PARAM0( tId ) );
  m_pcRateCtrl         = pcEncLib->getRateCtrl();
  m_pcSliceEncoder     = pcEncLib->getSliceEncoder();
#if ENABLE_SPLIT_PARALLELISM
  m_pcEncLib           = pcEncLib;
  m_dataId             = tId;
#endif
  m_pcLoopFilter       = pcEncLib->getLoopFilter();

#if !JVET_AG0164_AFFINE_GPM
  m_GeoCostList.init(GEO_NUM_PARTITION_MODE, m_pcEncCfg->getMaxNumGeoCand());
#endif
  m_AFFBestSATDCost = MAX_DOUBLE;

  DecCu::init( m_pcTrQuant, m_pcIntraSearch, m_pcInterSearch );

  m_modeCtrl->init( m_pcEncCfg, m_pcRateCtrl, m_pcRdCost );
#if JVET_Y0240_BIM
  m_modeCtrl->setBIMQPMap( m_pcEncCfg->getAdaptQPmap() );
#endif

  m_pcInterSearch->setModeCtrl( m_modeCtrl );
  m_modeCtrl->setInterSearch(m_pcInterSearch);
  m_pcIntraSearch->setModeCtrl( m_modeCtrl );

#if JVET_AE0046_BI_GPM
  for (int i = 0; i < MRG_MAX_NUM_CANDS; i++)
  {
    m_mvBufEncBDOF4GPM[0][i] = &(m_mvBufEncBDOF[i][0]);
    m_mvBufEncBDOF4GPM[1][i] = &(m_mvBufEncBDOF4TM[i][0]);
    m_mvBufEncBDOF4GPM[2][i] = &(m_mvBufBDOF4GPM[i][0]);
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
    m_mvBufEncBDOF4GPM[3][i] = &(m_mvBufEncBDOF4BM[i][0]);
#else
    m_mvBufEncBDOF4GPM[3][i] = &(m_mvExtraBufEncBDOF4GPM[i][0]);
#endif
  }

  m_mvBufBDMVR4GPM[0] = &m_mvBufEncAmBDMVR[0][0];
  m_mvBufBDMVR4GPM[1] = &m_mvBufEncAmBDMVR[1][0];
#endif

}

// ====================================================================================================================
// Public member functions
// ====================================================================================================================

void EncCu::compressCtu( CodingStructure& cs, const UnitArea& area, const unsigned ctuRsAddr, const int prevQP[], const int currQP[] )
{
  m_modeCtrl->initCTUEncoding( *cs.slice );
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  cs.treeType = TREE_D;
#endif
  cs.slice->m_mapPltCost[0].clear();
  cs.slice->m_mapPltCost[1].clear();
#if ENABLE_SPLIT_PARALLELISM
  if( m_pcEncCfg->getNumSplitThreads() > 1 )
  {
    for( int jId = 1; jId < NUM_RESERVERD_SPLIT_JOBS; jId++ )
    {
      EncCu*            jobEncCu  = m_pcEncLib->getCuEncoder( cs.picture->scheduler.getSplitDataId( jId ) );
      CacheBlkInfoCtrl* cacheCtrl = dynamic_cast< CacheBlkInfoCtrl* >( jobEncCu->m_modeCtrl );
#if REUSE_CU_RESULTS
      BestEncInfoCache* bestCache = dynamic_cast< BestEncInfoCache* >( jobEncCu->m_modeCtrl );
#endif
      SaveLoadEncInfoSbt *sbtCache = dynamic_cast< SaveLoadEncInfoSbt* >( jobEncCu->m_modeCtrl );
      if( cacheCtrl )
      {
        cacheCtrl->init( *cs.slice );
      }
#if REUSE_CU_RESULTS
      if (bestCache)
      {
        bestCache->init(*cs.slice);
      }
#endif
      if (sbtCache)
      {
        sbtCache->init(*cs.slice);
      }
    }
  }

#if REUSE_CU_RESULTS
  if( auto* cacheCtrl = dynamic_cast<BestEncInfoCache*>( m_modeCtrl ) ) { cacheCtrl->tick(); }
#endif
  if( auto* cacheCtrl = dynamic_cast<CacheBlkInfoCtrl*>( m_modeCtrl ) ) { cacheCtrl->tick(); }
#endif
  // init the partitioning manager
  QTBTPartitioner partitioner;
  partitioner.initCtu(area, CH_L, *cs.slice);
  if (m_pcEncCfg->getIBCMode())
  {
    if (area.lx() == 0 && area.ly() == 0)
    {
      m_pcInterSearch->resetIbcSearch();
    }
    m_pcInterSearch->resetCtuRecord();
    m_ctuIbcSearchRangeX = m_pcEncCfg->getIBCLocalSearchRangeX();
    m_ctuIbcSearchRangeY = m_pcEncCfg->getIBCLocalSearchRangeY();
  }
  if (m_pcEncCfg->getIBCMode() && m_pcEncCfg->getIBCHashSearch() && (m_pcEncCfg->getIBCFastMethod() & IBC_FAST_METHOD_ADAPTIVE_SEARCHRANGE))
  {
    const int hashHitRatio = m_ibcHashMap.getHashHitRatio(area.Y()); // in percent
    if (hashHitRatio < 5) // 5%
    {
      m_ctuIbcSearchRangeX >>= 1;
      m_ctuIbcSearchRangeY >>= 1;
    }
    if (cs.slice->getNumRefIdx(REF_PIC_LIST_0) > 0)
    {
      m_ctuIbcSearchRangeX >>= 1;
      m_ctuIbcSearchRangeY >>= 1;
    }
  }
  // init current context pointer
  m_CurrCtx = m_ctxBuffer.data();

  CodingStructure *tempCS = m_pTempCS[gp_sizeIdxInfo->idxFrom( area.lumaSize().width )][gp_sizeIdxInfo->idxFrom( area.lumaSize().height )];
  CodingStructure *bestCS = m_pBestCS[gp_sizeIdxInfo->idxFrom( area.lumaSize().width )][gp_sizeIdxInfo->idxFrom( area.lumaSize().height )];

  cs.initSubStructure(*tempCS, partitioner.chType, partitioner.currArea(), false);
  cs.initSubStructure(*bestCS, partitioner.chType, partitioner.currArea(), false);
  tempCS->currQP[CH_L] = bestCS->currQP[CH_L] =
  tempCS->baseQP       = bestCS->baseQP       = currQP[CH_L];
  tempCS->prevQP[CH_L] = bestCS->prevQP[CH_L] = prevQP[CH_L];

  xCompressCU(tempCS, bestCS, partitioner);
  cs.slice->m_mapPltCost[0].clear();
  cs.slice->m_mapPltCost[1].clear();
  // all signals were already copied during compression if the CTU was split - at this point only the structures are copied to the top level CS
  const bool copyUnsplitCTUSignals = bestCS->cus.size() == 1;
  cs.useSubStructure(*bestCS, partitioner.chType, CS::getArea(*bestCS, area, partitioner.chType), copyUnsplitCTUSignals,
                     false, false, copyUnsplitCTUSignals, true);

  if (CS::isDualITree (cs) && isChromaEnabled (cs.pcv->chrFormat))
  {
    m_CABACEstimator->getCtx() = m_CurrCtx->start;

    partitioner.initCtu(area, CH_C, *cs.slice);

    cs.initSubStructure(*tempCS, partitioner.chType, partitioner.currArea(), false);
    cs.initSubStructure(*bestCS, partitioner.chType, partitioner.currArea(), false);
    tempCS->currQP[CH_C] = bestCS->currQP[CH_C] =
    tempCS->baseQP       = bestCS->baseQP       = currQP[CH_C];
    tempCS->prevQP[CH_C] = bestCS->prevQP[CH_C] = prevQP[CH_C];

    xCompressCU(tempCS, bestCS, partitioner);

    const bool copyUnsplitCTUSignals = bestCS->cus.size() == 1;
    cs.useSubStructure(*bestCS, partitioner.chType, CS::getArea(*bestCS, area, partitioner.chType),
                       copyUnsplitCTUSignals, false, false, copyUnsplitCTUSignals, true);
  }

  if (m_pcEncCfg->getUseRateCtrl())
  {
    (m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr)).m_actualMSE = (double)bestCS->dist / (double)m_pcRateCtrl->getRCPic()->getLCU(ctuRsAddr).m_numberOfPixel;
  }
  // reset context states and uninit context pointer
  m_CABACEstimator->getCtx() = m_CurrCtx->start;
  m_CurrCtx                  = 0;


  // Ensure that a coding was found
  // Selected mode's RD-cost must be not MAX_DOUBLE.
  CHECK( bestCS->cus.empty()                                   , "No possible encoding found" );
  CHECK( bestCS->cus[0]->predMode == NUMBER_OF_PREDICTION_MODES, "No possible encoding found" );
  CHECK( bestCS->cost             == MAX_DOUBLE                , "No possible encoding found" );
}

// ====================================================================================================================
// Protected member functions
// ====================================================================================================================

static int xCalcHADs8x8_ISlice(const Pel *piOrg, const int iStrideOrg)
{
  int k, i, j, jj;
  int diff[64], m1[8][8], m2[8][8], m3[8][8], iSumHad = 0;

  for (k = 0; k < 64; k += 8)
  {
    diff[k + 0] = piOrg[0];
    diff[k + 1] = piOrg[1];
    diff[k + 2] = piOrg[2];
    diff[k + 3] = piOrg[3];
    diff[k + 4] = piOrg[4];
    diff[k + 5] = piOrg[5];
    diff[k + 6] = piOrg[6];
    diff[k + 7] = piOrg[7];

    piOrg += iStrideOrg;
  }

  //horizontal
  for (j = 0; j < 8; j++)
  {
    jj = j << 3;
    m2[j][0] = diff[jj    ] + diff[jj + 4];
    m2[j][1] = diff[jj + 1] + diff[jj + 5];
    m2[j][2] = diff[jj + 2] + diff[jj + 6];
    m2[j][3] = diff[jj + 3] + diff[jj + 7];
    m2[j][4] = diff[jj    ] - diff[jj + 4];
    m2[j][5] = diff[jj + 1] - diff[jj + 5];
    m2[j][6] = diff[jj + 2] - diff[jj + 6];
    m2[j][7] = diff[jj + 3] - diff[jj + 7];

    m1[j][0] = m2[j][0] + m2[j][2];
    m1[j][1] = m2[j][1] + m2[j][3];
    m1[j][2] = m2[j][0] - m2[j][2];
    m1[j][3] = m2[j][1] - m2[j][3];
    m1[j][4] = m2[j][4] + m2[j][6];
    m1[j][5] = m2[j][5] + m2[j][7];
    m1[j][6] = m2[j][4] - m2[j][6];
    m1[j][7] = m2[j][5] - m2[j][7];

    m2[j][0] = m1[j][0] + m1[j][1];
    m2[j][1] = m1[j][0] - m1[j][1];
    m2[j][2] = m1[j][2] + m1[j][3];
    m2[j][3] = m1[j][2] - m1[j][3];
    m2[j][4] = m1[j][4] + m1[j][5];
    m2[j][5] = m1[j][4] - m1[j][5];
    m2[j][6] = m1[j][6] + m1[j][7];
    m2[j][7] = m1[j][6] - m1[j][7];
  }

  //vertical
  for (i = 0; i < 8; i++)
  {
    m3[0][i] = m2[0][i] + m2[4][i];
    m3[1][i] = m2[1][i] + m2[5][i];
    m3[2][i] = m2[2][i] + m2[6][i];
    m3[3][i] = m2[3][i] + m2[7][i];
    m3[4][i] = m2[0][i] - m2[4][i];
    m3[5][i] = m2[1][i] - m2[5][i];
    m3[6][i] = m2[2][i] - m2[6][i];
    m3[7][i] = m2[3][i] - m2[7][i];

    m1[0][i] = m3[0][i] + m3[2][i];
    m1[1][i] = m3[1][i] + m3[3][i];
    m1[2][i] = m3[0][i] - m3[2][i];
    m1[3][i] = m3[1][i] - m3[3][i];
    m1[4][i] = m3[4][i] + m3[6][i];
    m1[5][i] = m3[5][i] + m3[7][i];
    m1[6][i] = m3[4][i] - m3[6][i];
    m1[7][i] = m3[5][i] - m3[7][i];

    m2[0][i] = m1[0][i] + m1[1][i];
    m2[1][i] = m1[0][i] - m1[1][i];
    m2[2][i] = m1[2][i] + m1[3][i];
    m2[3][i] = m1[2][i] - m1[3][i];
    m2[4][i] = m1[4][i] + m1[5][i];
    m2[5][i] = m1[4][i] - m1[5][i];
    m2[6][i] = m1[6][i] + m1[7][i];
    m2[7][i] = m1[6][i] - m1[7][i];
  }

  for (i = 0; i < 8; i++)
  {
    for (j = 0; j < 8; j++)
    {
      iSumHad += abs(m2[i][j]);
    }
  }
  iSumHad -= abs(m2[0][0]);
  iSumHad = (iSumHad + 2) >> 2;
  return(iSumHad);
}

int  EncCu::updateCtuDataISlice(const CPelBuf buf)
{
  int  xBl, yBl;
  const int iBlkSize = 8;
  const Pel* pOrgInit = buf.buf;
  int  iStrideOrig = buf.stride;

  int iSumHad = 0;
  for( yBl = 0; ( yBl + iBlkSize ) <= buf.height; yBl += iBlkSize )
  {
    for( xBl = 0; ( xBl + iBlkSize ) <= buf.width; xBl += iBlkSize )
    {
      const Pel* pOrg = pOrgInit + iStrideOrig*yBl + xBl;
      iSumHad += xCalcHADs8x8_ISlice( pOrg, iStrideOrig );
    }
  }
  return( iSumHad );
}

bool EncCu::xCheckBestMode( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  bool bestCSUpdated = false;

  if( !tempCS->cus.empty() )
  {
    if( tempCS->cus.size() == 1 )
    {
      const CodingUnit& cu = *tempCS->cus.front();
      CHECK( cu.skip && !cu.firstPU->mergeFlag, "Skip flag without a merge flag is not allowed!" );
    }

#if WCG_EXT
    DTRACE_BEST_MODE( tempCS, bestCS, m_pcRdCost->getLambda( true ) );
#else
    DTRACE_BEST_MODE( tempCS, bestCS, m_pcRdCost->getLambda() );
#endif

#if MULTI_HYP_PRED
    if (tempCS->sps->getUseInterMultiHyp() && tempCS->slice->isInterB())
    {
      m_baseResultsForMH.insert(m_baseResultsForMH.end(), tempCS->m_meResults.begin(), tempCS->m_meResults.end());
#if MULTI_HYP_PRED
      tempCS->m_meResults.clear(); // avoid duplicate insert
#endif
    }
#endif
    if( m_modeCtrl->useModeResult( encTestMode, tempCS, partitioner ) )
    {

      std::swap( tempCS, bestCS );
      // store temp best CI for next CU coding
      m_CurrCtx->best = m_CABACEstimator->getCtx();
      m_bestModeUpdated = true;
      bestCSUpdated = true;
    }
  }

  // reset context states
  m_CABACEstimator->getCtx() = m_CurrCtx->start;
  return bestCSUpdated;

}

void EncCu::xCompressCU( CodingStructure*& tempCS, CodingStructure*& bestCS, Partitioner& partitioner, double maxCostAllowed )
{
  CHECK(maxCostAllowed < 0, "Wrong value of maxCostAllowed!");
#if JVET_AA0133_INTER_MTS_OPT
  m_pcInterSearch->setBestCost(maxCostAllowed);
#endif
#if ENABLE_SPLIT_PARALLELISM
  CHECK( m_dataId != tempCS->picture->scheduler.getDataId(), "Working in the wrong dataId!" );

  if( m_pcEncCfg->getNumSplitThreads() != 1 && tempCS->picture->scheduler.getSplitJobId() == 0 )
  {
    if( m_modeCtrl->isParallelSplit( *tempCS, partitioner ) )
    {
      m_modeCtrl->setParallelSplit( true );
      xCompressCUParallel( tempCS, bestCS, partitioner );
      return;
    }
  }

#endif
  uint32_t compBegin;
  uint32_t numComp;
  bool jointPLT = false;
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (CS::isDualITree(*bestCS))
#else
  if (partitioner.isSepTree( *tempCS ))
#endif
  {
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if( !CS::isDualITree(*tempCS) && partitioner.treeType != TREE_D )
    {
      compBegin = COMPONENT_Y;
      numComp = (tempCS->area.chromaFormat != CHROMA_400)?3: 1;
      jointPLT = true;
    }
    else
#endif
    {
    if (isLuma(partitioner.chType))
    {
      compBegin = COMPONENT_Y;
      numComp = 1;
    }
    else
    {
      compBegin = COMPONENT_Cb;
      numComp = 2;
    }
    }
  }
  else
  {
    compBegin = COMPONENT_Y;
    numComp = (tempCS->area.chromaFormat != CHROMA_400) ? 3 : 1;
    jointPLT = true;
  }
  SplitSeries splitmode = -1;
  uint8_t   bestLastPLTSize[MAX_NUM_CHANNEL_TYPE];
  Pel       bestLastPLT[MAX_NUM_COMPONENT][MAXPLTPREDSIZE]; // store LastPLT for
  uint8_t   curLastPLTSize[MAX_NUM_CHANNEL_TYPE];
  Pel       curLastPLT[MAX_NUM_COMPONENT][MAXPLTPREDSIZE]; // store LastPLT if no partition
  for (int i = compBegin; i < (compBegin + numComp); i++)
  {
    ComponentID comID = jointPLT ? (ComponentID)compBegin : ((i > 0) ? COMPONENT_Cb : COMPONENT_Y);
    bestLastPLTSize[comID] = 0;
    curLastPLTSize[comID] = tempCS->prevPLT.curPLTSize[comID];
    memcpy(curLastPLT[i], tempCS->prevPLT.curPLT[i], tempCS->prevPLT.curPLTSize[comID] * sizeof(Pel));
  }

  Slice&   slice      = *tempCS->slice;
  const PPS &pps      = *tempCS->pps;
  const SPS &sps      = *tempCS->sps;
  const uint32_t uiLPelX  = tempCS->area.Y().lumaPos().x;
  const uint32_t uiTPelY  = tempCS->area.Y().lumaPos().y;

#if ENABLE_OBMC
  const unsigned wIdx = gp_sizeIdxInfo->idxFrom(partitioner.currArea().lwidth());
  const unsigned hIdx = gp_sizeIdxInfo->idxFrom(partitioner.currArea().lheight());
#endif
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  const ModeType modeTypeParent  = partitioner.modeType;
  const TreeType treeTypeParent  = partitioner.treeType;
  const ChannelType chTypeParent = partitioner.chType;
#endif
  const UnitArea currCsArea = clipArea( CS::getArea( *bestCS, bestCS->area, partitioner.chType ), *tempCS->picture );
#if MULTI_HYP_PRED
  m_baseResultsForMH.clear();
#endif
#if ENABLE_OBMC
  if (m_pTempCUWoOBMC && !slice.isIntra())
  {
    tempCS->initSubStructure(*m_pTempCUWoOBMC[wIdx][hIdx], partitioner.chType, partitioner.currArea(), false);
  }
#endif
#if JVET_Y0152_TT_ENC_SPEEDUP
  tempCS->splitRdCostBest = NULL;
#endif
  m_modeCtrl->initCULevel( partitioner, *tempCS );
#if JVET_Z0054_BLK_REF_PIC_REORDER
  m_pcInterSearch->setFillCurTplAboveARMC(false);
  m_pcInterSearch->setFillCurTplLeftARMC(false);
#endif
#if JVET_AD0213_LIC_IMP
  m_pcInterSearch->resetFillLicTpl();
#endif

#if JVET_Z0118_GDR
  if (m_pcEncCfg->getGdrEnabled())
  {
    bool isCuInCleanArea = false;
    bool isCuInRefreshArea = false;

    bool isInGdrInterval = slice.getPicHeader()->getInGdrInterval();
    bool isRecoveryPocPic = slice.getPicHeader()->getIsGdrRecoveryPocPic();
    
    // 1.0 pic in GDR interval
    if (isInGdrInterval || isRecoveryPocPic)
    {
      // 1.1 set intra/inter area     
      int gdrBegX = tempCS->picHeader->getGdrBegX();
      int gdrEndX = tempCS->picHeader->getGdrEndX();

      isCuInCleanArea   = tempCS->isClean(tempCS->area.Y(), CHANNEL_TYPE_LUMA);     
      isCuInRefreshArea = tempCS->withinRefresh(gdrBegX, gdrEndX);

      // 1.2 switch recon based on clean/dirty current area
      tempCS->setReconBuf((isCuInCleanArea) ? PIC_RECONSTRUCTION_1 : PIC_RECONSTRUCTION_0);
      tempCS->picture->setCleanDirty(isCuInCleanArea);

      for (int rlist = REF_PIC_LIST_0; rlist < NUM_REF_PIC_LIST_01; rlist++)
      {
        int n = slice.getNumRefIdx((RefPicList)rlist);
        for (int idx = 0; idx < n; idx++)
        {
          Picture *refPic = slice.getReferencePicture((RefPicList)rlist, idx);
          if (refPic)
          {
            refPic->setCleanDirty(isCuInCleanArea);
          }
        }
      }

      // Need to keep the begining of intra refresh area when HashME is enabled
      bool splitCondition = tempCS->isCuCrossVB(gdrEndX);
      bool forceRefreshIRA = false;
      if (m_pcEncCfg->getUseHashME())
      {
        splitCondition |= tempCS->isCuCrossIRA(gdrBegX);
        forceRefreshIRA = true;
      }


      if (isCuInRefreshArea)
      {
        if (forceRefreshIRA)
        {
          m_modeCtrl->forceIntraMode();
        }
      }

      if (splitCondition)
      {
        // remove every prediction mode (remain split only)
        m_modeCtrl->forceRemovePredMode();

        const unsigned minQtSize = tempCS->pcv->getMinQtSize(*tempCS->slice, CHANNEL_TYPE_LUMA); // 8

        if (tempCS->area.lheight() <= minQtSize)
        {
          m_modeCtrl->forceRemoveTTH();
        }

        if (tempCS->area.lheight() < minQtSize)
        {
          m_modeCtrl->forceRemoveBTH();
        }

        if (tempCS->area.lwidth() > minQtSize * 2)
        {
          m_modeCtrl->forceRemoveBTV();
          m_modeCtrl->forceRemoveTTV();
        }

        if (tempCS->area.lwidth() < minQtSize || tempCS->area.lheight() < minQtSize)
        {
          m_modeCtrl->forceRemoveQT();
        }

        if (tempCS->area.lwidth() != tempCS->area.lheight())
        {
          m_modeCtrl->forceRemoveQT();
        }

        if (!m_modeCtrl->anyPredModeLeft())
        {
          m_modeCtrl->forceRemoveDontSplit();
        }
      }
    }
    // 2. pic in non-GDR interval
    else
    {
      tempCS->setReconBuf(PIC_RECONSTRUCTION_0);
      tempCS->picture->setCleanDirty(false);
            
      {
        // 2.1 setup reference picture for non-GDR
        for (int rlist = REF_PIC_LIST_0; rlist < NUM_REF_PIC_LIST_01; rlist++)
        {
          int n = slice.getNumRefIdx((RefPicList)rlist);
          for (int idx = 0; idx < n; idx++)
          {
            Picture *refPic = slice.getReferencePicture((RefPicList)rlist, idx);

            if (refPic)
            {
              bool isRefInGdrInterval  = refPic->cs->picHeader->getInGdrInterval();
              bool isRefRecoveryPocPic = refPic->cs->picHeader->getIsGdrRecoveryPocPic();

              if (isRefInGdrInterval || isRefRecoveryPocPic)
              {
                refPic->setCleanDirty(true);
              }
              else
              {
                refPic->setCleanDirty(false);
              }
            }
          }
        }
      }
    }
  }
#endif


  if( partitioner.currQtDepth == 0 && partitioner.currMtDepth == 0 && !tempCS->slice->isIntra() && ( sps.getUseSBT() || sps.getUseInterMTS() ) )
  {
    auto slsSbt = dynamic_cast<SaveLoadEncInfoSbt*>( m_modeCtrl );
    int maxSLSize = sps.getUseSBT() ? tempCS->slice->getSPS()->getMaxTbSize() : MTS_INTER_MAX_CU_SIZE;
    slsSbt->resetSaveloadSbt( maxSLSize );
#if ENABLE_SPLIT_PARALLELISM
    CHECK( tempCS->picture->scheduler.getSplitJobId() != 0, "The SBT search reset need to happen in sequential region." );
    if (m_pcEncCfg->getNumSplitThreads() > 1)
    {
      for (int jId = 1; jId < NUM_RESERVERD_SPLIT_JOBS; jId++)
      {
        auto slsSbt = dynamic_cast<SaveLoadEncInfoSbt *>(m_pcEncLib->getCuEncoder(jId)->m_modeCtrl);
        slsSbt->resetSaveloadSbt(maxSLSize);
      }
    }
#endif
  }
  m_sbtCostSave[0] = m_sbtCostSave[1] = MAX_DOUBLE;
#if JVET_AA0133_INTER_MTS_OPT
  m_mtsCostSave = MAX_DOUBLE;
#endif
#if JVET_AG0061_INTER_LFNST_NSPT
  m_LNCostSave = MAX_DOUBLE;
#endif
  m_CurrCtx->start = m_CABACEstimator->getCtx();

  m_cuChromaQpOffsetIdxPlus1 = 0;

  if( slice.getUseChromaQpAdj() )
  {
    // TODO M0133 : double check encoder decisions with respect to chroma QG detection and actual encode
    int lgMinCuSize = sps.getLog2MinCodingBlockSize() +
      std::max<int>(0, floorLog2(sps.getCTUSize()) - sps.getLog2MinCodingBlockSize() - int(slice.getCuChromaQpOffsetSubdiv() / 2));
    if( partitioner.currQgChromaEnable() )
    {
      m_cuChromaQpOffsetIdxPlus1 = ( ( uiLPelX >> lgMinCuSize ) + ( uiTPelY >> lgMinCuSize ) ) % ( pps.getChromaQpOffsetListLen() + 1 );
    }
  }

  if( !m_modeCtrl->anyMode() )
  {
    m_modeCtrl->finishCULevel( partitioner );
    return;
  }

  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cux", uiLPelX ) );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cuy", uiTPelY ) );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cuw", tempCS->area.lwidth() ) );
  DTRACE_UPDATE( g_trace_ctx, std::make_pair( "cuh", tempCS->area.lheight() ) );
  DTRACE( g_trace_ctx, D_COMMON, "@(%4d,%4d) [%2dx%2d]\n", tempCS->area.lx(), tempCS->area.ly(), tempCS->area.lwidth(), tempCS->area.lheight() );


  m_pcInterSearch->resetSavedAffineMotion();
#if TM_AMVP
  if (!slice.isIntra())
  {
    m_pcInterSearch->clearTplAmvpBuffer();
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    m_pcInterSearch->clearAmvpTmvpBuffer();
#endif
  }
#endif
#if INTER_LIC
  m_pcInterSearch->m_fastLicCtrl.init();
#endif

#if JVET_AF0073_INTER_CCP_MERGE
  m_pcInterSearch->m_isInterCcpModelReady = false;
  m_pcInterSearch->m_validNum = 0;
  for (int i = 0; i < MAX_CCP_CAND_LIST_SIZE; i++)
  {
    m_pcInterSearch->m_interCcpMergeList[i] = {};
  }
#endif

#if MULTI_HYP_PRED
  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));
#if JVET_AD0213_LIC_IMP
  m_pcInterSearch->initMHPTmpBuffer(m_acRealMergeBuffer + MRG_MAX_NUM_CANDS, m_acRealMergeBuffer + (MRG_MAX_NUM_CANDS << 1), GEO_MAX_NUM_UNI_CANDS,
#else
  m_pcInterSearch->initMHPTmpBuffer(m_acRealMergeBuffer + MRG_MAX_NUM_CANDS, GEO_MAX_NUM_UNI_CANDS,
#endif
    m_acGeoWeightedBuffer, GEO_MAX_TRY_WEIGHTED_SAD,
    localUnitArea);
#endif
#if JVET_W0097_GPM_MMVD_TM
  m_mergeCandAvail = false;
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  m_skipIbcMerge = (m_pcEncCfg->getIBCFastMethod() & IBC_FAST_METHOD_NONSCC) ? true : false;
#endif
  double bestIntPelCost = MAX_DOUBLE;

  if (tempCS->slice->getSPS()->getUseColorTrans())
  {
    tempCS->tmpColorSpaceCost = MAX_DOUBLE;
    bestCS->tmpColorSpaceCost = MAX_DOUBLE;
    tempCS->firstColorSpaceSelected = true;
    bestCS->firstColorSpaceSelected = true;
  }

  if (tempCS->slice->getSPS()->getUseColorTrans() && !CS::isDualITree(*tempCS))
  {
    tempCS->firstColorSpaceTestOnly = false;
    bestCS->firstColorSpaceTestOnly = false;
    tempCS->tmpColorSpaceIntraCost[0] = MAX_DOUBLE;
    tempCS->tmpColorSpaceIntraCost[1] = MAX_DOUBLE;
    bestCS->tmpColorSpaceIntraCost[0] = MAX_DOUBLE;
    bestCS->tmpColorSpaceIntraCost[1] = MAX_DOUBLE;

    if (tempCS->bestParent && tempCS->bestParent->firstColorSpaceTestOnly)
    {
      tempCS->firstColorSpaceTestOnly = bestCS->firstColorSpaceTestOnly = true;
    }
  }

#if JVET_Y0152_TT_ENC_SPEEDUP
  double splitRdCostBest[NUM_PART_SPLIT];
  std::fill(std::begin(splitRdCostBest), std::end(splitRdCostBest), MAX_DOUBLE);
#endif
  if( tempCS->slice->getCheckLDC() )
  {
    m_bestBcwCost[0] = m_bestBcwCost[1] = std::numeric_limits<double>::max();
    m_bestBcwIdx[0] = m_bestBcwIdx[1] = -1;
  }
  do
  {
    for (int i = compBegin; i < (compBegin + numComp); i++)
    {
      ComponentID comID = jointPLT ? (ComponentID)compBegin : ((i > 0) ? COMPONENT_Cb : COMPONENT_Y);
      tempCS->prevPLT.curPLTSize[comID] = curLastPLTSize[comID];
      memcpy(tempCS->prevPLT.curPLT[i], curLastPLT[i], curLastPLTSize[comID] * sizeof(Pel));
    }
    EncTestMode currTestMode = m_modeCtrl->currTestMode();
    currTestMode.maxCostAllowed = maxCostAllowed;
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (pps.getUseDQP() && CS::isDualITree(*tempCS) && isChroma(partitioner.chType))
#else
    if (pps.getUseDQP() && partitioner.isSepTree(*tempCS) && isChroma( partitioner.chType ))
#endif
    {
      const Position chromaCentral(tempCS->area.Cb().chromaPos().offset(tempCS->area.Cb().chromaSize().width >> 1, tempCS->area.Cb().chromaSize().height >> 1));
      const Position lumaRefPos(chromaCentral.x << getComponentScaleX(COMPONENT_Cb, tempCS->area.chromaFormat), chromaCentral.y << getComponentScaleY(COMPONENT_Cb, tempCS->area.chromaFormat));
      const CodingStructure* baseCS = bestCS->picture->cs;
      const CodingUnit* colLumaCu = baseCS->getCU(lumaRefPos, CHANNEL_TYPE_LUMA);

      if (colLumaCu)
      {
        currTestMode.qp = colLumaCu->qp;
      }
    }

#if SHARP_LUMA_DELTA_QP || ENABLE_QPA_SUB_CTU
    if (partitioner.currQgEnable() && (
#if JVET_Y0240_BIM
        (m_pcEncCfg->getBIM()) ||
#endif
#if SHARP_LUMA_DELTA_QP
        (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()) ||
#endif
#if ENABLE_QPA_SUB_CTU
        (m_pcEncCfg->getUsePerceptQPA() && !m_pcEncCfg->getUseRateCtrl() && pps.getUseDQP())
#else
        false
#endif
      ))
    {
#if ENABLE_SPLIT_PARALLELISM
      CHECK( tempCS->picture->scheduler.getSplitJobId() > 0, "Changing lambda is only allowed in the master thread!" );
#endif
      if (currTestMode.qp >= 0)
      {
        updateLambda (&slice, currTestMode.qp,
 #if WCG_EXT && ER_CHROMA_QP_WCG_PPS
                      m_pcEncCfg->getWCGChromaQPControl().isEnabled(),
 #endif
                      CS::isDualITree (*tempCS) || (partitioner.currDepth == 0));
      }
    }
#endif


    if( currTestMode.type == ETM_INTER_ME )
    {
#if ENABLE_OBMC
      bool tryObmc = true;
#if JVET_AA0129_INTERHASH_OBMCOFF_RD
      if (m_pcEncCfg->getUseHashME())
      {
        tryObmc = false;
      }
#endif
#endif

      if( ( currTestMode.opts & ETO_IMV ) != 0 )
      {
        const bool skipAltHpelIF = ( int( ( currTestMode.opts & ETO_IMV ) >> ETO_IMV_SHIFT ) == 4 ) && ( bestIntPelCost > 1.25 * bestCS->cost );
        if (!skipAltHpelIF)
        {
          tempCS->bestCS = bestCS;
#if ENABLE_OBMC
          tryObmc = xCheckRDCostInterIMV(tempCS, bestCS, partitioner, currTestMode, bestIntPelCost);
#else
          xCheckRDCostInterIMV(tempCS, bestCS, partitioner, currTestMode, bestIntPelCost);
#endif
          tempCS->bestCS = nullptr;
#if JVET_Y0152_TT_ENC_SPEEDUP
          splitRdCostBest[CTU_LEVEL] = bestCS->cost;
          tempCS->splitRdCostBest = splitRdCostBest;
#endif
        }
      }
      else
      {
        tempCS->bestCS = bestCS;
#if ENABLE_OBMC && JVET_AA0129_INTERHASH_OBMCOFF_RD
        tryObmc = xCheckRDCostInter(tempCS, bestCS, partitioner, currTestMode);
#else
        xCheckRDCostInter( tempCS, bestCS, partitioner, currTestMode );
#endif
        tempCS->bestCS = nullptr;
#if JVET_Y0152_TT_ENC_SPEEDUP
        splitRdCostBest[CTU_LEVEL] = bestCS->cost;
        tempCS->splitRdCostBest = splitRdCostBest;
#endif
      }
#if ENABLE_OBMC
#if JVET_AA0129_INTERHASH_OBMCOFF_RD
      if ((!m_pcEncCfg->getUseHashME() && tryObmc && tempCS->cus.size() != 0) || (m_pcEncCfg->getUseHashME() && tryObmc))//todo 
#else
      if (tryObmc && tempCS->cus.size() != 0)//todo
#endif
      {
        xCheckRDCostInterWoOBMC(tempCS, bestCS, partitioner, currTestMode);
      }
#endif
    }
    else if (currTestMode.type == ETM_HASH_INTER)
    {
#if ENABLE_OBMC && JVET_AA0129_INTERHASH_OBMCOFF_RD
      bool tryObmc = xCheckRDCostHashInter( tempCS, bestCS, partitioner, currTestMode );
#else
      xCheckRDCostHashInter( tempCS, bestCS, partitioner, currTestMode );
#endif
#if JVET_Y0152_TT_ENC_SPEEDUP
      splitRdCostBest[CTU_LEVEL] = bestCS->cost;
      tempCS->splitRdCostBest = splitRdCostBest;
#endif
#if ENABLE_OBMC && JVET_AA0129_INTERHASH_OBMCOFF_RD
      if (tryObmc)
      {
        xCheckRDCostInterWoOBMC(  tempCS, bestCS, partitioner, currTestMode );
      }
#endif
    }
#if !MERGE_ENC_OPT
    else if( currTestMode.type == ETM_AFFINE )
    {
      xCheckRDCostAffineMerge2Nx2N( tempCS, bestCS, partitioner, currTestMode );
#if JVET_Y0152_TT_ENC_SPEEDUP
      splitRdCostBest[CTU_LEVEL] = bestCS->cost;
      tempCS->splitRdCostBest = splitRdCostBest;
#endif
    }
#endif
#if AFFINE_MMVD && !MERGE_ENC_OPT
    else if (currTestMode.type == ETM_AF_MMVD)
    {
      xCheckRDCostAffineMmvd2Nx2N(tempCS, bestCS, partitioner, currTestMode);
#if JVET_Y0152_TT_ENC_SPEEDUP
      splitRdCostBest[CTU_LEVEL] = bestCS->cost;
      tempCS->splitRdCostBest = splitRdCostBest;
#endif
    }
#endif
#if TM_MRG && !MERGE_ENC_OPT
    else if (currTestMode.type == ETM_MERGE_TM)
    {
      xCheckRDCostTMMerge2Nx2N(tempCS, bestCS, partitioner, currTestMode);
#if JVET_Y0152_TT_ENC_SPEEDUP
      splitRdCostBest[CTU_LEVEL] = bestCS->cost;
      tempCS->splitRdCostBest = splitRdCostBest;
#endif
    }
#endif
#if REUSE_CU_RESULTS
    else if( currTestMode.type == ETM_RECO_CACHED )
    {
      xReuseCachedResult( tempCS, bestCS, partitioner );
#if JVET_Y0152_TT_ENC_SPEEDUP
      splitRdCostBest[CTU_LEVEL] = bestCS->cost;
      tempCS->splitRdCostBest = splitRdCostBest;
#endif
    }
#endif
    else if( currTestMode.type == ETM_MERGE_SKIP )
    {
      xCheckRDCostMerge2Nx2N( tempCS, bestCS, partitioner, currTestMode );

      CodingUnit* cu = bestCS->getCU(partitioner.chType);

      if (cu)
      cu->mmvdSkip = cu->skip == false ? false : cu->mmvdSkip;
#if JVET_Y0152_TT_ENC_SPEEDUP
      splitRdCostBest[CTU_LEVEL] = bestCS->cost;
      tempCS->splitRdCostBest = splitRdCostBest;
#endif
    }
    else if( currTestMode.type == ETM_MERGE_GEO )
    {
#if JVET_W0097_GPM_MMVD_TM
      CodedCUInfo    &relatedCU = ((EncModeCtrlMTnoRQT *)m_modeCtrl)->getBlkInfo(partitioner.currArea());
      if (!relatedCU.isGPMTested)
      {
        xCheckRDCostMergeGeoComb2Nx2N(tempCS, bestCS, partitioner, currTestMode);
        relatedCU.isGPMTested = 1;
      }
      else
      {
        xCheckRDCostMergeGeoComb2Nx2N(tempCS, bestCS, partitioner, currTestMode, true);
      }
#else
      xCheckRDCostMergeGeo2Nx2N( tempCS, bestCS, partitioner, currTestMode );
#endif
#if JVET_Y0152_TT_ENC_SPEEDUP
      splitRdCostBest[CTU_LEVEL] = bestCS->cost;
      tempCS->splitRdCostBest = splitRdCostBest;
#endif
    }
#if MULTI_HYP_PRED
    else if (currTestMode.type == ETM_INTER_MULTIHYP)
    {
      xCheckRDCostInterMultiHyp2Nx2N(tempCS, bestCS, partitioner, currTestMode);
#if JVET_Y0152_TT_ENC_SPEEDUP
      splitRdCostBest[CTU_LEVEL] = bestCS->cost;
      tempCS->splitRdCostBest = splitRdCostBest;
#endif
    }
#endif
    else if( currTestMode.type == ETM_INTRA )
    {
      if (slice.getSPS()->getUseColorTrans() && !CS::isDualITree(*tempCS))
      {
        bool skipSecColorSpace = false;
        skipSecColorSpace = xCheckRDCostIntra(tempCS, bestCS, partitioner, currTestMode, (m_pcEncCfg->getRGBFormatFlag() ? true : false));
        if ((m_pcEncCfg->getCostMode() == COST_LOSSLESS_CODING && slice.isLossless()) && !m_pcEncCfg->getRGBFormatFlag())
        {
          skipSecColorSpace = true;
        }
        if (!skipSecColorSpace && !tempCS->firstColorSpaceTestOnly)
        {
          xCheckRDCostIntra(tempCS, bestCS, partitioner, currTestMode, (m_pcEncCfg->getRGBFormatFlag() ? false : true));
        }

        if (!tempCS->firstColorSpaceTestOnly)
        {
          if (tempCS->tmpColorSpaceIntraCost[0] != MAX_DOUBLE && tempCS->tmpColorSpaceIntraCost[1] != MAX_DOUBLE)
          {
            double skipCostRatio = m_pcEncCfg->getRGBFormatFlag() ? 1.1 : 1.0;
            if (tempCS->tmpColorSpaceIntraCost[1] > (skipCostRatio*tempCS->tmpColorSpaceIntraCost[0]))
            {
              tempCS->firstColorSpaceTestOnly = bestCS->firstColorSpaceTestOnly = true;
            }
          }
        }
        else
        {
          CHECK(tempCS->tmpColorSpaceIntraCost[1] != MAX_DOUBLE, "the RD test of the second color space should be skipped");
        }
      }
      else
      {
        xCheckRDCostIntra(tempCS, bestCS, partitioner, currTestMode, false);
      }
#if JVET_AE0057_MTT_ET
      if (partitioner.currQtDepth == (tempCS->sps->getCTUSize() == 256 ? 2 : 1) && partitioner.currBtDepth == 0
          && partitioner.currArea().lwidth() == 64 && partitioner.currArea().lheight() == 64)
      {
        if ((partitioner.chType == CHANNEL_TYPE_LUMA)
            && ((partitioner.currArea().Y().x + 63 < bestCS->picture->lwidth())
                && (partitioner.currArea().Y().y + 63 < bestCS->picture->lheight())))

        {
          m_modeCtrl->setNoSplitIntraCost(bestCS->cost);
        }
      }
#endif

#if JVET_Y0152_TT_ENC_SPEEDUP
      splitRdCostBest[CTU_LEVEL] = bestCS->cost;
      tempCS->splitRdCostBest = splitRdCostBest;
#endif
    }
    else if (currTestMode.type == ETM_PALETTE)
    {
      xCheckPLT( tempCS, bestCS, partitioner, currTestMode );
#if JVET_Y0152_TT_ENC_SPEEDUP
      splitRdCostBest[CTU_LEVEL] = bestCS->cost;
      tempCS->splitRdCostBest = splitRdCostBest;
#endif
    }
    else if (currTestMode.type == ETM_IBC)
    {
#if JVET_AA0070_RRIBC
      if (m_pcEncCfg->getIntraPeriod() <= 1)
      {
        CodedCUInfo &relatedCU = ((EncModeCtrlMTnoRQT *) m_modeCtrl)->getBlkInfo(partitioner.currArea());
        if (!relatedCU.isRribcTested)
        {
          xCheckRDCostIBCMode(tempCS, bestCS, partitioner, currTestMode);
          relatedCU.isRribcTested = 1;
        }
        else
        {
          xCheckRDCostIBCMode(tempCS, bestCS, partitioner, currTestMode, true);
        }
      }
      else
#endif
      xCheckRDCostIBCMode(tempCS, bestCS, partitioner, currTestMode);
#if JVET_Y0152_TT_ENC_SPEEDUP
      splitRdCostBest[CTU_LEVEL] = bestCS->cost;
      tempCS->splitRdCostBest = splitRdCostBest;
#endif
    }
    else if (currTestMode.type == ETM_IBC_MERGE)
    {
#if JVET_AE0169_BIPREDICTIVE_IBC
      if (!m_skipIbcMerge)
#endif
      xCheckRDCostIBCModeMerge2Nx2N(tempCS, bestCS, partitioner, currTestMode);
#if JVET_Y0152_TT_ENC_SPEEDUP
      splitRdCostBest[CTU_LEVEL] = bestCS->cost;
      tempCS->splitRdCostBest = splitRdCostBest;
#endif
    }
    else if( isModeSplit( currTestMode ) )
    {
      if (bestCS->cus.size() != 0)
      {
        splitmode = bestCS->cus[0]->splitSeries;
      }
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      assert( partitioner.modeType == tempCS->modeType );
      int signalModeConsVal = tempCS->signalModeCons( getPartSplit( currTestMode ), partitioner, modeTypeParent );
      int numRoundRdo = signalModeConsVal == LDT_MODE_TYPE_SIGNAL ? 2 : 1;
      bool skipInterPass = false;
      for( int i = 0; i < numRoundRdo; i++ )
      {
        //change cons modes
        if( signalModeConsVal == LDT_MODE_TYPE_SIGNAL )
        {
          CHECK( numRoundRdo != 2, "numRoundRdo shall be 2 - [LDT_MODE_TYPE_SIGNAL]" );
          tempCS->modeType = partitioner.modeType = (i == 0) ? MODE_TYPE_INTER : MODE_TYPE_INTRA;
        }
        else if( signalModeConsVal == LDT_MODE_TYPE_INFER )
        {
          CHECK( numRoundRdo != 1, "numRoundRdo shall be 1 - [LDT_MODE_TYPE_INFER]" );
          tempCS->modeType = partitioner.modeType = MODE_TYPE_INTRA;
        }
        else if( signalModeConsVal == LDT_MODE_TYPE_INHERIT )
        {
          CHECK( numRoundRdo != 1, "numRoundRdo shall be 1 - [LDT_MODE_TYPE_INHERIT]" );
          tempCS->modeType = partitioner.modeType = modeTypeParent;
        }

        //for lite intra encoding fast algorithm, set the status to save inter coding info
        if( modeTypeParent == MODE_TYPE_ALL && tempCS->modeType == MODE_TYPE_INTER )
        {
          m_pcIntraSearch->setSaveCuCostInSCIPU( true );
          m_pcIntraSearch->setNumCuInSCIPU( 0 );
        }
        else if( modeTypeParent == MODE_TYPE_ALL && tempCS->modeType != MODE_TYPE_INTER )
        {
          m_pcIntraSearch->setSaveCuCostInSCIPU( false );
          if( tempCS->modeType == MODE_TYPE_ALL )
          {
            m_pcIntraSearch->setNumCuInSCIPU( 0 );
          }
        }

#if JVET_Y0152_TT_ENC_SPEEDUP
        xCheckModeSplit(tempCS, bestCS, partitioner, currTestMode, modeTypeParent, skipInterPass, splitRdCostBest);
        tempCS->splitRdCostBest = splitRdCostBest;
#else
        xCheckModeSplit( tempCS, bestCS, partitioner, currTestMode, modeTypeParent, skipInterPass );
#endif
#else
#if JVET_Y0152_TT_ENC_SPEEDUP
      xCheckModeSplit(tempCS, bestCS, partitioner, currTestMode, splitRdCostBest);
      tempCS->splitRdCostBest = splitRdCostBest;
#else
      xCheckModeSplit(tempCS, bestCS, partitioner, currTestMode);
#endif
#endif
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
        //recover cons modes
        tempCS->modeType = partitioner.modeType = modeTypeParent;
        tempCS->treeType = partitioner.treeType = treeTypeParent;
        partitioner.chType = chTypeParent;
        if( modeTypeParent == MODE_TYPE_ALL )
        {
          m_pcIntraSearch->setSaveCuCostInSCIPU( false );
          if( numRoundRdo == 2 && tempCS->modeType == MODE_TYPE_INTRA )
          {
            m_pcIntraSearch->initCuAreaCostInSCIPU();
          }
        }
        if( skipInterPass )
        {
          break;
        }
      }
#endif
#if JVET_Z0118_GDR
    if (bestCS->cus.size() > 0 && splitmode != bestCS->cus[0]->splitSeries)
#else
      if (splitmode != bestCS->cus[0]->splitSeries)
#endif
      {
        splitmode = bestCS->cus[0]->splitSeries;
        const CodingUnit&     cu = *bestCS->cus.front();
        cu.cs->prevPLT = bestCS->prevPLT;
        for (int i = compBegin; i < (compBegin + numComp); i++)
        {
          ComponentID comID = jointPLT ? (ComponentID)compBegin : ((i > 0) ? COMPONENT_Cb : COMPONENT_Y);
          bestLastPLTSize[comID] = bestCS->cus[0]->cs->prevPLT.curPLTSize[comID];
          memcpy(bestLastPLT[i], bestCS->cus[0]->cs->prevPLT.curPLT[i], bestCS->cus[0]->cs->prevPLT.curPLTSize[comID] * sizeof(Pel));
        }
      }
    }
    else
    {
      THROW( "Don't know how to handle mode: type = " << currTestMode.type << ", options = " << currTestMode.opts );
    }
  } while( m_modeCtrl->nextMode( *tempCS, partitioner ) );


  //////////////////////////////////////////////////////////////////////////
  // Finishing CU
#if ENABLE_SPLIT_PARALLELISM
  if( bestCS->cus.empty() )
  {
    CHECK( bestCS->cost != MAX_DOUBLE, "Cost should be maximal if no encoding found" );
    CHECK( bestCS->picture->scheduler.getSplitJobId() == 0, "Should always get a result in serial case" );

    m_modeCtrl->finishCULevel( partitioner );
    return;
  }

#endif
  if( tempCS->cost == MAX_DOUBLE && bestCS->cost == MAX_DOUBLE )
  {
    //although some coding modes were planned to be tried in RDO, no coding mode actually finished encoding due to early termination
    //thus tempCS->cost and bestCS->cost are both MAX_DOUBLE; in this case, skip the following process for normal case
    m_modeCtrl->finishCULevel( partitioner );
    return;
  }

  // set context states
  m_CABACEstimator->getCtx() = m_CurrCtx->best;

  // QP from last processed CU for further processing
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  //copy the qp of the last non-chroma CU
  int numCUInThisNode = (int)bestCS->cus.size();
  if( numCUInThisNode > 1 && bestCS->cus.back()->chType == CHANNEL_TYPE_CHROMA && !CS::isDualITree( *bestCS ) )
  {
    CHECK( bestCS->cus[numCUInThisNode-2]->chType != CHANNEL_TYPE_LUMA, "wrong chType" );
    bestCS->prevQP[partitioner.chType] = bestCS->cus[numCUInThisNode-2]->qp;
  }
  else
  {
#endif
  bestCS->prevQP[partitioner.chType] = bestCS->cus.back()->qp;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  }
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AF0079_STORING_INTRATMP
  if ((!slice.isIntra() || slice.getUseIBC() || slice.getSPS()->getUseIntraTMP())
#else
  if ((!slice.isIntra() || slice.getUseIBC())
#endif
#else
  if ((!slice.isIntra() || slice.getSPS()->getIBCFlag())
#endif
    && partitioner.chType == CHANNEL_TYPE_LUMA
    && bestCS->cus.size() == 1 && (bestCS->cus.back()->predMode == MODE_INTER || bestCS->cus.back()->predMode == MODE_IBC
#if JVET_AF0079_STORING_INTRATMP
          || bestCS->cus.back()->tmpFlag
#endif
      )
    && bestCS->area.Y() == (*bestCS->cus.back()).Y()
    )
  {
    const CodingUnit&     cu = *bestCS->cus.front();

#if JVET_AF0079_STORING_INTRATMP
    bool isIbcSmallBlk = false;
#else
    bool isIbcSmallBlk = CU::isIBC(cu) && (cu.lwidth() * cu.lheight() <= 16);

#if JVET_AE0094_IBC_NONADJACENT_SPATIAL_CANDIDATES
    if(cu.cs->sps->getUseIbcNonAdjCand())
    {
      isIbcSmallBlk = false;
    }
#endif
#endif
    CU::saveMotionInHMVP( cu, isIbcSmallBlk );
  }

#if JVET_AD0188_CCP_MERGE
  {
    const CodingUnit &cu          = *bestCS->cus.front();
    bool              lumaUsesISP = !CS::isDualITree(*bestCS) && cu.ispMode;
    if (!(cu.chromaFormat == CHROMA_400 || (CS::isDualITree(*bestCS) && cu.chType == CHANNEL_TYPE_LUMA))
        && CU::isIntra(cu) && !lumaUsesISP && bestCS->cus.size() == 1
        && bestCS->area.Cb() == (*bestCS->cus.back()).Cb())
    {
      CU::saveModelsInHCCP(cu);
    }
  }
#endif
#if JVET_AG0058_EIP
  {
    const CodingUnit &cu          = *bestCS->cus.front();
    CHECK(cu.chType != partitioner.chType, "different partition type." );
    if(slice.getSPS()->getUseEip() && partitioner.chType == CHANNEL_TYPE_LUMA && CU::isIntra(cu)
       && bestCS->cus.size() == 1 && bestCS->area.Y() == (*bestCS->cus.back()).Y())
    {
      CU::saveModelsInHEIP(cu);
    }
  }
#endif
#if JVET_AG0059_CCP_MERGE_ENHANCEMENT
  {
    CodingUnit& cu = *bestCS->cus.front();
    bool              lumaUsesISP = !CS::isDualITree(*bestCS) && cu.ispMode;
    if (!(cu.chromaFormat == CHROMA_400 || (CS::isDualITree(*bestCS) && cu.chType == CHANNEL_TYPE_LUMA))
      && CU::isIntra(cu) && !lumaUsesISP && bestCS->cus.size() == 1
      && bestCS->area.Cb() == (*bestCS->cus.back()).Cb())
    {
      CU::saveCcInsideFilterFlagInCCP(cu);
    }
  }
#endif
#if JVET_AF0073_INTER_CCP_MERGE
  {
    CodingUnit& cu = *bestCS->cus.front();
    if (!(cu.chromaFormat == CHROMA_400 || CS::isDualITree(*bestCS))
      && partitioner.chType == CHANNEL_TYPE_LUMA
      && (!CU::isIntra(cu)) && bestCS->cus.size() == 1
      && bestCS->area.Cb() == (*bestCS->cus.back()).Cb()
      )
    {
      CU::saveModelsInHCCP(cu);
      CU::saveProCcpInfo(cu);
    }
  }
#endif

  bestCS->picture->getPredBuf(currCsArea).copyFrom(bestCS->getPredBuf(currCsArea));
#if JVET_Z0118_GDR
  bestCS->updateReconMotIPM(currCsArea); // xcomrpessCU - need 
#else
  bestCS->picture->getRecoBuf(currCsArea).copyFrom(bestCS->getRecoBuf(currCsArea));
#endif  
  
  m_modeCtrl->finishCULevel( partitioner );
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( m_pcIntraSearch->getSaveCuCostInSCIPU() && bestCS->cus.size() == 1 )
  {
    m_pcIntraSearch->saveCuAreaCostInSCIPU( Area( partitioner.currArea().lumaPos(), partitioner.currArea().lumaSize() ), bestCS->cost );
  }
#endif
#if ENABLE_SPLIT_PARALLELISM
  if( tempCS->picture->scheduler.getSplitJobId() == 0 && m_pcEncCfg->getNumSplitThreads() != 1 )
  {
    tempCS->picture->finishParallelPart( currCsArea );
  }

#endif
  if (bestCS->cus.size() == 1) // no partition
  {
    CHECK(bestCS->cus[0]->tileIdx != bestCS->pps->getTileIdx(bestCS->area.lumaPos()), "Wrong tile index!");
    if (bestCS->cus[0]->predMode == MODE_PLT)
    {
      for (int i = compBegin; i < (compBegin + numComp); i++)
      {
        ComponentID comID = jointPLT ? (ComponentID)compBegin : ((i > 0) ? COMPONENT_Cb : COMPONENT_Y);
        bestCS->prevPLT.curPLTSize[comID] = curLastPLTSize[comID];
        memcpy(bestCS->prevPLT.curPLT[i], curLastPLT[i], curLastPLTSize[comID] * sizeof(Pel));
      }
      bestCS->reorderPrevPLT(bestCS->prevPLT, bestCS->cus[0]->curPLTSize, bestCS->cus[0]->curPLT, bestCS->cus[0]->reuseflag, compBegin, numComp, jointPLT);
    }
    else
    {
      for (int i = compBegin; i<(compBegin + numComp); i++)
      {
        ComponentID comID = jointPLT ? (ComponentID)compBegin : ((i > 0) ? COMPONENT_Cb : COMPONENT_Y);
        bestCS->prevPLT.curPLTSize[comID] = curLastPLTSize[comID];
        memcpy(bestCS->prevPLT.curPLT[i], curLastPLT[i], bestCS->prevPLT.curPLTSize[comID] * sizeof(Pel));
      }
    }
  }
  else
  {
    for (int i = compBegin; i<(compBegin + numComp); i++)
    {
      ComponentID comID = jointPLT ? (ComponentID)compBegin : ((i > 0) ? COMPONENT_Cb : COMPONENT_Y);
      bestCS->prevPLT.curPLTSize[comID] = bestLastPLTSize[comID];
      memcpy(bestCS->prevPLT.curPLT[i], bestLastPLT[i], bestCS->prevPLT.curPLTSize[comID] * sizeof(Pel));
    }
  }
  const CodingUnit&     cu = *bestCS->cus.front();
  cu.cs->prevPLT = bestCS->prevPLT;
  // Assert if Best prediction mode is NONE
  // Selected mode's RD-cost must be not MAX_DOUBLE.
  CHECK( bestCS->cus.empty()                                   , "No possible encoding found" );
  CHECK( bestCS->cus[0]->predMode == NUMBER_OF_PREDICTION_MODES, "No possible encoding found" );
  CHECK( bestCS->cost             == MAX_DOUBLE                , "No possible encoding found" );
}

#if SHARP_LUMA_DELTA_QP || ENABLE_QPA_SUB_CTU
void EncCu::updateLambda (Slice* slice, const int dQP,
 #if WCG_EXT && ER_CHROMA_QP_WCG_PPS
                          const bool useWCGChromaControl,
 #endif
                          const bool updateRdCostLambda)
{
#if WCG_EXT && ER_CHROMA_QP_WCG_PPS
  if (useWCGChromaControl)
  {
    const double lambda = m_pcSliceEncoder->initializeLambda (slice, m_pcSliceEncoder->getGopId(), slice->getSliceQp(), (double)dQP);
    const int clippedQP = Clip3 (-slice->getSPS()->getQpBDOffset (CHANNEL_TYPE_LUMA), MAX_QP, dQP);

    m_pcSliceEncoder->setUpLambda (slice, lambda, clippedQP);
    return;
  }
#endif
  int iQP = dQP;
  const double oldQP     = (double)slice->getSliceQpBase();
#if ENABLE_QPA_SUB_CTU
  const double oldLambda = (m_pcEncCfg->getUsePerceptQPA() && !m_pcEncCfg->getUseRateCtrl() && slice->getPPS()->getUseDQP()) ? slice->getLambdas()[0] :
                           m_pcSliceEncoder->calculateLambda (slice, m_pcSliceEncoder->getGopId(), oldQP, oldQP, iQP);
#else
  const double oldLambda = m_pcSliceEncoder->calculateLambda (slice, m_pcSliceEncoder->getGopId(), oldQP, oldQP, iQP);
#endif
  const double newLambda = oldLambda * pow (2.0, ((double)dQP - oldQP) / 3.0);
#if RDOQ_CHROMA_LAMBDA
  const double lambdaArray[MAX_NUM_COMPONENT] = {newLambda / m_pcRdCost->getDistortionWeight (COMPONENT_Y),
                                                 newLambda / m_pcRdCost->getDistortionWeight (COMPONENT_Cb),
                                                 newLambda / m_pcRdCost->getDistortionWeight (COMPONENT_Cr)};
  m_pcTrQuant->setLambdas (lambdaArray);
#else
  m_pcTrQuant->setLambda (newLambda);
#endif
  if (updateRdCostLambda)
  {
    m_pcRdCost->setLambda (newLambda, slice->getSPS()->getBitDepths());
#if WCG_EXT
    if (!m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled())
    {
      m_pcRdCost->saveUnadjustedLambda();
    }
#endif
  }
}
#endif // SHARP_LUMA_DELTA_QP || ENABLE_QPA_SUB_CTU

#if ENABLE_SPLIT_PARALLELISM
//#undef DEBUG_PARALLEL_TIMINGS
//#define DEBUG_PARALLEL_TIMINGS 1
void EncCu::xCompressCUParallel( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner )
{
  const unsigned wIdx = gp_sizeIdxInfo->idxFrom( partitioner.currArea().lwidth() );
  const unsigned hIdx = gp_sizeIdxInfo->idxFrom( partitioner.currArea().lheight() );

  Picture* picture = tempCS->picture;

  int numJobs = m_modeCtrl->getNumParallelJobs( *bestCS, partitioner );

  bool    jobUsed                            [NUM_RESERVERD_SPLIT_JOBS];
  std::fill( jobUsed, jobUsed + NUM_RESERVERD_SPLIT_JOBS, false );

  const UnitArea currArea = CS::getArea( *tempCS, partitioner.currArea(), partitioner.chType );
  const bool doParallel   = !m_pcEncCfg->getForceSingleSplitThread();
  omp_set_num_threads( m_pcEncCfg->getNumSplitThreads() );

#pragma omp parallel for schedule(dynamic,1) if(doParallel)
  for( int jId = 1; jId <= numJobs; jId++ )
  {
    // thread start
    picture->scheduler.setSplitThreadId();
    picture->scheduler.setSplitJobId( jId );

    QTBTPartitioner jobPartitioner;
    EncCu*       jobCuEnc       = m_pcEncLib->getCuEncoder( picture->scheduler.getSplitDataId( jId ) );
    auto*        jobBlkCache    = dynamic_cast<CacheBlkInfoCtrl*>( jobCuEnc->m_modeCtrl );
#if REUSE_CU_RESULTS
    auto*        jobBestCache   = dynamic_cast<BestEncInfoCache*>( jobCuEnc->m_modeCtrl );
#endif

    jobPartitioner.copyState( partitioner );
    jobCuEnc      ->copyState( this, jobPartitioner, currArea, true );

    if( jobBlkCache  ) { jobBlkCache ->tick(); }
#if REUSE_CU_RESULTS
    if( jobBestCache ) { jobBestCache->tick(); }

#endif
    CodingStructure *&jobBest = jobCuEnc->m_pBestCS[wIdx][hIdx];
    CodingStructure *&jobTemp = jobCuEnc->m_pTempCS[wIdx][hIdx];

    jobUsed[jId] = true;

    jobCuEnc->xCompressCU( jobTemp, jobBest, jobPartitioner );

    picture->scheduler.setSplitJobId( 0 );
    // thread stop
  }
  picture->scheduler.setSplitThreadId( 0 );

  int    bestJId  = 0;
  double bestCost = bestCS->cost;
  for( int jId = 1; jId <= numJobs; jId++ )
  {
    EncCu* jobCuEnc = m_pcEncLib->getCuEncoder( picture->scheduler.getSplitDataId( jId ) );

    if( jobUsed[jId] && jobCuEnc->m_pBestCS[wIdx][hIdx]->cost < bestCost )
    {
      bestCost = jobCuEnc->m_pBestCS[wIdx][hIdx]->cost;
      bestJId  = jId;
    }
  }

  if( bestJId > 0 )
  {
    copyState( m_pcEncLib->getCuEncoder( picture->scheduler.getSplitDataId( bestJId ) ), partitioner, currArea, false );
    m_CurrCtx->best = m_CABACEstimator->getCtx();

    tempCS = m_pTempCS[wIdx][hIdx];
    bestCS = m_pBestCS[wIdx][hIdx];
  }

  const int      bitDepthY = tempCS->sps->getBitDepth( CH_L );
  const UnitArea clipdArea = clipArea( currArea, *picture );

  CHECK( calcCheckSum( picture->getRecoBuf( clipdArea.Y() ), bitDepthY ) != calcCheckSum( bestCS->getRecoBuf( clipdArea.Y() ), bitDepthY ), "Data copied incorrectly!" );

  picture->finishParallelPart( currArea );

  if( auto *blkCache = dynamic_cast<CacheBlkInfoCtrl*>( m_modeCtrl ) )
  {
    for( int jId = 1; jId <= numJobs; jId++ )
    {
      if( !jobUsed[jId] || jId == bestJId ) continue;

      auto *jobBlkCache = dynamic_cast<CacheBlkInfoCtrl*>( m_pcEncLib->getCuEncoder( picture->scheduler.getSplitDataId( jId ) )->m_modeCtrl );
      CHECK( !jobBlkCache, "If own mode controller has blk info cache capability so should all other mode controllers!" );
      blkCache->CacheBlkInfoCtrl::copyState( *jobBlkCache, partitioner.currArea() );
    }

    blkCache->tick();
  }
#if REUSE_CU_RESULTS

  if( auto *blkCache = dynamic_cast<BestEncInfoCache*>( m_modeCtrl ) )
  {
    for( int jId = 1; jId <= numJobs; jId++ )
    {
      if( !jobUsed[jId] || jId == bestJId ) continue;

      auto *jobBlkCache = dynamic_cast<BestEncInfoCache*>( m_pcEncLib->getCuEncoder( picture->scheduler.getSplitDataId( jId ) )->m_modeCtrl );
      CHECK( !jobBlkCache, "If own mode controller has blk info cache capability so should all other mode controllers!" );
      blkCache->BestEncInfoCache::copyState( *jobBlkCache, partitioner.currArea() );
    }

    blkCache->tick();
  }
#endif
}

void EncCu::copyState( EncCu* other, Partitioner& partitioner, const UnitArea& currArea, const bool isDist )
{
  const unsigned wIdx = gp_sizeIdxInfo->idxFrom( partitioner.currArea().lwidth () );
  const unsigned hIdx = gp_sizeIdxInfo->idxFrom( partitioner.currArea().lheight() );

  if( isDist )
  {
    other->m_pBestCS[wIdx][hIdx]->initSubStructure( *m_pBestCS[wIdx][hIdx], partitioner.chType, partitioner.currArea(), false );
    other->m_pTempCS[wIdx][hIdx]->initSubStructure( *m_pTempCS[wIdx][hIdx], partitioner.chType, partitioner.currArea(), false );
  }
  else
  {
          CodingStructure* dst =        m_pBestCS[wIdx][hIdx];
    const CodingStructure* src = other->m_pBestCS[wIdx][hIdx];
    bool keepResi = KEEP_PRED_AND_RESI_SIGNALS;
    bool keepPred = true;

    dst->useSubStructure( *src, partitioner.chType, currArea, keepPred, true, keepResi, keepResi, true );

    dst->cost           =  src->cost;
    dst->dist           =  src->dist;
    dst->fracBits       =  src->fracBits;
    dst->features       =  src->features;
  }

  if( isDist )
  {
    m_CurrCtx = m_ctxBuffer.data();
  }

  m_pcInterSearch->copyState( *other->m_pcInterSearch );
  m_modeCtrl     ->copyState( *other->m_modeCtrl, partitioner.currArea() );
  m_pcRdCost     ->copyState( *other->m_pcRdCost );
  m_pcTrQuant    ->copyState( *other->m_pcTrQuant );
  if( m_pcEncCfg->getLmcs() )
  {
    EncReshape *encReshapeThis  = dynamic_cast<EncReshape*>(       m_pcReshape);
    EncReshape *encReshapeOther = dynamic_cast<EncReshape*>(other->m_pcReshape);
    encReshapeThis->copyState( *encReshapeOther );
  }

  m_CABACEstimator->getCtx() = other->m_CABACEstimator->getCtx();
}
#endif
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
#if JVET_Y0152_TT_ENC_SPEEDUP
void EncCu::xCheckModeSplit(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode, double *splitRdCostBest)
#else
void EncCu::xCheckModeSplit(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
#endif
#else
#if JVET_Y0152_TT_ENC_SPEEDUP
void EncCu::xCheckModeSplit(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode, const ModeType modeTypeParent, bool &skipInterPass, double *splitRdCostBest)
#else
void EncCu::xCheckModeSplit(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode, const ModeType modeTypeParent, bool &skipInterPass )
#endif
#endif
{
  const int qp                = encTestMode.qp;
  const Slice &slice          = *tempCS->slice;
  const int oldPrevQp         = tempCS->prevQP[partitioner.chType];
  const auto oldMotionLut     = tempCS->motionLut;
#if JVET_AD0188_CCP_MERGE
  const auto oldCCPLut        = tempCS->ccpLut;
#endif
#if JVET_AG0058_EIP
  const auto oldEipLut        = tempCS->eipLut;
#endif
#if ENABLE_QPA_SUB_CTU
  const PPS &pps              = *tempCS->pps;
  const uint32_t currDepth    = partitioner.currDepth;
#endif
  const auto oldPLT           = tempCS->prevPLT;

  const PartSplit split = getPartSplit( encTestMode );
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  const ModeType modeTypeChild = partitioner.modeType;
#endif
  CHECK( split == CU_DONT_SPLIT, "No proper split provided!" );

  tempCS->initStructData( qp );

  m_CABACEstimator->getCtx() = m_CurrCtx->start;

  m_CABACEstimator->resetBits();

  m_CABACEstimator->split_cu_mode( split, *tempCS, partitioner );
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  m_CABACEstimator->mode_constraint( split, *tempCS, partitioner, modeTypeChild );
#endif
  const double factor = ( tempCS->currQP[partitioner.chType] > 30 ? 1.1 : 1.075 );
  tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();
  if (!tempCS->useDbCost)
    CHECK(bestCS->costDbOffset != 0, "error");
  const double cost   = m_pcRdCost->calcRdCost( uint64_t( m_CABACEstimator->getEstFracBits() + ( ( bestCS->fracBits ) / factor ) ), Distortion( bestCS->dist / factor ) ) + bestCS->costDbOffset / factor;

  m_CABACEstimator->getCtx() = SubCtx( Ctx::Split, m_CurrCtx->start);
  if (cost > bestCS->cost + bestCS->costDbOffset
#if ENABLE_QPA_SUB_CTU
    || (m_pcEncCfg->getUsePerceptQPA() && !m_pcEncCfg->getUseRateCtrl() && pps.getUseDQP() && (slice.getCuQpDeltaSubdiv() > 0) && (split == CU_HORZ_SPLIT || split == CU_VERT_SPLIT) &&
        (currDepth == 0)) // force quad-split or no split at CTU level
#endif
    )
  {
    xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );
    return;
  }
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  const bool chromaNotSplit = modeTypeParent == MODE_TYPE_ALL && modeTypeChild == MODE_TYPE_INTRA ? true : false;
  if( partitioner.treeType != TREE_D )
  {
    tempCS->treeType = TREE_L;
  }
  else
  {
    if( chromaNotSplit )
    {
      CHECK( partitioner.chType != CHANNEL_TYPE_LUMA, "chType must be luma" );
      tempCS->treeType = partitioner.treeType = TREE_L;
    }
    else
    {
      tempCS->treeType = partitioner.treeType = TREE_D;
    }
  }
#endif

  partitioner.splitCurrArea( split, *tempCS );
  bool qgEnableChildren = partitioner.currQgEnable(); // QG possible at children level

  m_CurrCtx++;

  tempCS->getRecoBuf().fill( 0 );

  tempCS->getPredBuf().fill(0);
  AffineMVInfo tmpMVInfo;
  bool isAffMVInfoSaved;
  m_pcInterSearch->savePrevAffMVInfo(0, tmpMVInfo, isAffMVInfoSaved);
  BlkUniMvInfo tmpUniMvInfo;
  bool         isUniMvInfoSaved = false;
  if (!tempCS->slice->isIntra())
  {
    m_pcInterSearch->savePrevUniMvInfo(tempCS->area.Y(), tmpUniMvInfo, isUniMvInfoSaved);
  }
#if INTER_LIC
  BlkUniMvInfo tmpUniMvInfoLIC;
  bool         isUniMvInfoSavedLIC = false;
  if (tempCS->slice->getUseLIC() && !tempCS->slice->isIntra())
  {
    m_pcInterSearch->swapUniMvBuffer();
    m_pcInterSearch->savePrevUniMvInfo(tempCS->area.Y(), tmpUniMvInfoLIC, isUniMvInfoSavedLIC);
    m_pcInterSearch->swapUniMvBuffer();
  }
#endif

  do
  {
    const auto &subCUArea  = partitioner.currArea();

    if( tempCS->picture->Y().contains( subCUArea.lumaPos() ) )
    {
      const unsigned wIdx    = gp_sizeIdxInfo->idxFrom( subCUArea.lwidth () );
      const unsigned hIdx    = gp_sizeIdxInfo->idxFrom( subCUArea.lheight() );

      CodingStructure *tempSubCS = m_pTempCS[wIdx][hIdx];
      CodingStructure *bestSubCS = m_pBestCS[wIdx][hIdx];

#if JVET_AE0057_MTT_ET
      if (partitioner.currQtDepth == (tempCS->sps->getCTUSize() == 256 ? 2 : 1) && partitioner.currBtDepth == 0
          && partitioner.currArea().lwidth() == 64 && partitioner.currArea().lheight() == 64)
      {
        if ((partitioner.chType == CHANNEL_TYPE_LUMA)
            && ((partitioner.currArea().Y().x + 63 < bestCS->picture->lwidth())
                && (partitioner.currArea().Y().y + 63 < bestCS->picture->lheight())))
        {
          m_modeCtrl->setNoSplitIntraCost(0.0);
        }
      }
#endif 
      tempCS->initSubStructure( *tempSubCS, partitioner.chType, subCUArea, false );
      tempCS->initSubStructure( *bestSubCS, partitioner.chType, subCUArea, false );
      tempSubCS->bestParent = bestSubCS->bestParent = bestCS;
      double newMaxCostAllowed = isLuma(partitioner.chType) ? std::min(encTestMode.maxCostAllowed, bestCS->cost - m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist)) : MAX_DOUBLE;
      newMaxCostAllowed = std::max(0.0, newMaxCostAllowed);
      xCompressCU(tempSubCS, bestSubCS, partitioner, newMaxCostAllowed);
      tempSubCS->bestParent = bestSubCS->bestParent = nullptr;

      if( bestSubCS->cost == MAX_DOUBLE )
      {
        CHECK( split == CU_QUAD_SPLIT, "Split decision reusing cannot skip quad split" );
        tempCS->cost = MAX_DOUBLE;
        tempCS->costDbOffset = 0;
        tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();
        m_CurrCtx--;
        partitioner.exitCurrSplit();
        xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );

#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
        if( partitioner.chType == CHANNEL_TYPE_LUMA )
        {
          tempCS->motionLut = oldMotionLut;
        }
#endif

#if JVET_Z0118_GDR
        tempCS->motionLut = oldMotionLut;
#if JVET_AD0188_CCP_MERGE
        tempCS->ccpLut  = oldCCPLut;
#endif
#if JVET_AG0058_EIP
        tempCS->eipLut  = oldEipLut;
#endif
        tempCS->prevPLT = oldPLT;
        tempCS->releaseIntermediateData();
        tempCS->prevQP[partitioner.chType] = oldPrevQp;
#endif        

        return;
      }

      bool keepResi = KEEP_PRED_AND_RESI_SIGNALS;
      tempCS->useSubStructure( *bestSubCS, partitioner.chType, CS::getArea( *tempCS, subCUArea, partitioner.chType ), KEEP_PRED_AND_RESI_SIGNALS, true, keepResi, keepResi, true );

      if( partitioner.currQgEnable() )
      {
        tempCS->prevQP[partitioner.chType] = bestSubCS->prevQP[partitioner.chType];
      }
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if( partitioner.isConsInter() )
      {
        for( int i = 0; i < bestSubCS->cus.size(); i++ )
        {
          CHECK( bestSubCS->cus[i]->predMode != MODE_INTER, "all CUs must be inter mode in an Inter coding region (SCIPU)" );
        }
      }
      else if( partitioner.isConsIntra() )
      {
        for( int i = 0; i < bestSubCS->cus.size(); i++ )
        {
          CHECK( bestSubCS->cus[i]->predMode == MODE_INTER, "all CUs must not be inter mode in an Intra coding region (SCIPU)" );
        }
      }
#endif
      tempSubCS->releaseIntermediateData();
      bestSubCS->releaseIntermediateData();
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if( !tempCS->slice->isIntra() && partitioner.isConsIntra() )
      {
        tempCS->cost = m_pcRdCost->calcRdCost( tempCS->fracBits, tempCS->dist );
        if( tempCS->cost > bestCS->cost )
        {
          tempCS->cost = MAX_DOUBLE;
          tempCS->costDbOffset = 0;
          tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();
          m_CurrCtx--;
          partitioner.exitCurrSplit();
          if( partitioner.chType == CHANNEL_TYPE_LUMA )
          {
            tempCS->motionLut = oldMotionLut;
          }
          return;
        }
      }
#endif
    }
  } while( partitioner.nextPart( *tempCS ) );

  partitioner.exitCurrSplit();


  m_CurrCtx--;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( chromaNotSplit )
  {
    //Note: In local dual tree region, the chroma CU refers to the central luma CU's QP.
    //If the luma CU QP shall be predQP (no residual in it and before it in the QG), it must be revised to predQP before encoding the chroma CU
    //Otherwise, the chroma CU uses predQP+deltaQP in encoding but is decoded as using predQP, thus causing encoder-decoded mismatch on chroma qp.
    if( tempCS->pps->getUseDQP() )
    {
      //find parent CS that including all coded CUs in the QG before this node
      CodingStructure* qgCS = tempCS;
      bool deltaQpCodedBeforeThisNode = false;
      if( partitioner.currArea().lumaPos() != partitioner.currQgPos )
      {
        int numParentNodeToQgCS = 0;
        while( qgCS->area.lumaPos() != partitioner.currQgPos )
        {
          CHECK( qgCS->parent == nullptr, "parent of qgCS shall exsit" );
          qgCS = qgCS->parent;
          numParentNodeToQgCS++;
        }

        //check whether deltaQP has been coded (in luma CU or luma&chroma CU) before this node
        CodingStructure* parentCS = tempCS->parent;
        for( int i = 0; i < numParentNodeToQgCS; i++ )
        {
          //checking each parent
          CHECK( parentCS == nullptr, "parentCS shall exsit" );
          for( const auto &cu : parentCS->cus )
          {
            if( cu->rootCbf && !isChroma( cu->chType ) )
            {
              deltaQpCodedBeforeThisNode = true;
              break;
            }
          }
          parentCS = parentCS->parent;
        }
      }

      //revise luma CU qp before the first luma CU with residual in the SCIPU to predQP
      if( !deltaQpCodedBeforeThisNode )
      {
        //get pred QP of the QG
        const CodingUnit* cuFirst = qgCS->getCU( CHANNEL_TYPE_LUMA );
        CHECK( cuFirst->lumaPos() != partitioner.currQgPos, "First cu of the Qg is wrong" );
        int predQp = CU::predictQP( *cuFirst, qgCS->prevQP[CHANNEL_TYPE_LUMA] );

        //revise to predQP
        int firstCuHasResidual = (int)tempCS->cus.size();
        for( int i = 0; i < tempCS->cus.size(); i++ )
        {
          if( tempCS->cus[i]->rootCbf )
          {
            firstCuHasResidual = i;
            break;
          }
        }

        for( int i = 0; i < firstCuHasResidual; i++ )
        {
          tempCS->cus[i]->qp = predQp;
        }
      }
    }
    assert( tempCS->treeType == TREE_L );
    uint32_t numCuPuTu[6];
    tempCS->picture->cs->getNumCuPuTuOffset( numCuPuTu );
    tempCS->picture->cs->useSubStructure( *tempCS, partitioner.chType, CS::getArea( *tempCS, partitioner.currArea(), partitioner.chType ), false, true, false, false, false );

    if (isChromaEnabled(tempCS->pcv->chrFormat))
    {
    partitioner.chType = CHANNEL_TYPE_CHROMA;
    tempCS->treeType = partitioner.treeType = TREE_C;

    m_CurrCtx++;

    const unsigned wIdx = gp_sizeIdxInfo->idxFrom( partitioner.currArea().lwidth() );
    const unsigned hIdx = gp_sizeIdxInfo->idxFrom( partitioner.currArea().lheight() );
    CodingStructure *tempCSChroma = m_pTempCS2[wIdx][hIdx];
    CodingStructure *bestCSChroma = m_pBestCS2[wIdx][hIdx];
    tempCS->initSubStructure( *tempCSChroma, partitioner.chType, partitioner.currArea(), false );
    tempCS->initSubStructure( *bestCSChroma, partitioner.chType, partitioner.currArea(), false );
    tempCS->treeType = TREE_D;
    xCompressCU( tempCSChroma, bestCSChroma, partitioner );

    //attach chromaCS to luma CS and update cost
    bool keepResi = KEEP_PRED_AND_RESI_SIGNALS;
    //bestCSChroma->treeType = tempCSChroma->treeType = TREE_C;
    CHECK( bestCSChroma->treeType != TREE_C || tempCSChroma->treeType != TREE_C, "wrong treeType for chroma CS" );
    tempCS->useSubStructure( *bestCSChroma, partitioner.chType, CS::getArea( *bestCSChroma, partitioner.currArea(), partitioner.chType ), KEEP_PRED_AND_RESI_SIGNALS, true, keepResi, true, true );

    //release tmp resource
    tempCSChroma->releaseIntermediateData();
    bestCSChroma->releaseIntermediateData();
    //tempCS->picture->cs->releaseIntermediateData();
      m_CurrCtx--;
    }
    tempCS->picture->cs->clearCuPuTuIdxMap( partitioner.currArea(), numCuPuTu[0], numCuPuTu[1], numCuPuTu[2], numCuPuTu + 3 );


    //recover luma tree status
    partitioner.chType = CHANNEL_TYPE_LUMA;
    partitioner.treeType = TREE_D;
    partitioner.modeType = MODE_TYPE_ALL;
  }
#endif
  // Finally, generate split-signaling bits for RD-cost check
  const PartSplit implicitSplit = partitioner.getImplicitSplit( *tempCS );

  {
    bool enforceQT = implicitSplit == CU_QUAD_SPLIT;

    // LARGE CTU bug
    if( m_pcEncCfg->getUseFastLCTU() )
    {
      unsigned minDepth = 0;
      unsigned maxDepth = floorLog2(tempCS->sps->getCTUSize()) - floorLog2(tempCS->sps->getMinQTSize(slice.getSliceType(), partitioner.chType));

      if( auto ad = dynamic_cast<AdaptiveDepthPartitioner*>( &partitioner ) )
      {
        ad->setMaxMinDepth( minDepth, maxDepth, *tempCS );
      }

      if( minDepth > partitioner.currQtDepth )
      {
        // enforce QT
        enforceQT = true;
      }
    }

    if( !enforceQT )
    {
      m_CABACEstimator->resetBits();

      m_CABACEstimator->split_cu_mode( split, *tempCS, partitioner );
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      partitioner.modeType = modeTypeParent;
      m_CABACEstimator->mode_constraint( split, *tempCS, partitioner, modeTypeChild );
#endif
      tempCS->fracBits += m_CABACEstimator->getEstFracBits(); // split bits
    }
  }

  tempCS->cost = m_pcRdCost->calcRdCost( tempCS->fracBits, tempCS->dist );

  // Check Delta QP bits for splitted structure
  if( !qgEnableChildren ) // check at deepest QG level only
  xCheckDQP( *tempCS, partitioner, true );

  // If the configuration being tested exceeds the maximum number of bytes for a slice / slice-segment, then
  // a proper RD evaluation cannot be performed. Therefore, termination of the
  // slice/slice-segment must be made prior to this CTU.
  // This can be achieved by forcing the decision to be that of the rpcTempCU.
  // The exception is each slice / slice-segment must have at least one CTU.
  if (bestCS->cost != MAX_DOUBLE)
  {
  }
  else
  {
    bestCS->costDbOffset = 0;
  }
  tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( tempCS->cus.size() > 0 && modeTypeParent == MODE_TYPE_ALL && modeTypeChild == MODE_TYPE_INTER )
  {
    int areaSizeNoResiCu = 0;
    for( int k = 0; k < tempCS->cus.size(); k++ )
    {
      areaSizeNoResiCu += (tempCS->cus[k]->rootCbf == false) ? tempCS->cus[k]->lumaSize().area() : 0;
    }
    if( areaSizeNoResiCu >= (tempCS->area.lumaSize().area() >> 1) )
    {
      skipInterPass = true;
    }
  }
#endif
#if JVET_Y0152_TT_ENC_SPEEDUP
  splitRdCostBest[getPartSplit(encTestMode)] = tempCS->cost;
#endif
  // RD check for sub partitioned coding structure.
  xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );

  if (isAffMVInfoSaved)
    m_pcInterSearch->addAffMVInfo(tmpMVInfo);
  if (!tempCS->slice->isIntra() && isUniMvInfoSaved)
  {
    m_pcInterSearch->addUniMvInfo(tmpUniMvInfo);
  }
#if INTER_LIC
  if (!tempCS->slice->isIntra() && isUniMvInfoSavedLIC)
  {
    m_pcInterSearch->swapUniMvBuffer();
    m_pcInterSearch->addUniMvInfo(tmpUniMvInfoLIC);
    m_pcInterSearch->swapUniMvBuffer();
  }
#endif

  tempCS->motionLut = oldMotionLut;

  tempCS->prevPLT   = oldPLT;
#if JVET_AD0188_CCP_MERGE
  tempCS->ccpLut    = oldCCPLut;
#endif
#if JVET_AG0058_EIP
  tempCS->eipLut    = oldEipLut;
#endif

  tempCS->releaseIntermediateData();

  tempCS->prevQP[partitioner.chType] = oldPrevQp;
}

bool EncCu::xCheckRDCostIntra(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode, bool adaptiveColorTrans)
{
  double          bestInterCost             = m_modeCtrl->getBestInterCost();
  double          costSize2Nx2NmtsFirstPass = m_modeCtrl->getMtsSize2Nx2NFirstPassCost();
  bool            skipSecondMtsPass         = m_modeCtrl->getSkipSecondMTSPass();
  const SPS&      sps                       = *tempCS->sps;
  const int       maxSizeMTS                = MTS_INTRA_MAX_CU_SIZE;
  uint8_t         considerMtsSecondPass     = ( sps.getUseIntraMTS() && isLuma( partitioner.chType ) && partitioner.currArea().lwidth() <= maxSizeMTS && partitioner.currArea().lheight() <= maxSizeMTS ) ? 1 : 0;

  bool   useIntraSubPartitions   = false;
  double maxCostAllowedForChroma = MAX_DOUBLE;
  const  CodingUnit *bestCU      = bestCS->getCU( partitioner.chType );
  Distortion interHad = m_modeCtrl->getInterHad();
#if JVET_AG0146_DIMD_ITMP_IBC
  bool isBvDimd = 0;
  Mv   bvDimd   = Mv(0, 0);
  bool bvDimdDerived = 0;
#endif
#if (JVET_AG0146_DIMD_ITMP_IBC || JVET_AG0152_SGPM_ITMP_IBC || JVET_AG0151_INTRA_TMP_MERGE_MODE)
  bool bvListDerived = 0;
#endif
#if JVET_W0123_TIMD_FUSION
  int timdMode = 0;
  int timdModeSecondary = 0;
  bool timdIsBlended = false;
#if JVET_AG0092_ENHANCED_TIMD_FUSION
  int timdModeNonAng = 0;
  int timdFusionWeight[TIMD_FUSION_NUM] = { 0 };
  int8_t timdLocDep[TIMD_FUSION_NUM] = { 0 };
#else
  int  timdFusionWeight[2] = { 0 };
#endif
#if JVET_AC0094_REF_SAMPLES_OPT
  bool timdModeCheckWA          = true;
  bool timdModeSecondaryCheckWA = true;
#endif
#endif
#if JVET_AB0155_SGPM
  int timdHorMode = 0;
  int timdVerMode = 0;
#endif
#if JVET_AD0086_ENHANCED_INTRA_TMP
#if (JVET_AG0146_DIMD_ITMP_IBC || JVET_AG0152_SGPM_ITMP_IBC || JVET_AG0151_INTRA_TMP_MERGE_MODE)
  int tmpXdisp = 0;
  int tmpYdisp = 0;
#endif
  m_pcIntraSearch->initTmpDisp();
  m_pcIntraSearch->initTmpFlmParams();
  m_pcIntraSearch->initTmpFusionInfo();
#elif TMP_FAST_ENC
  m_pcIntraSearch->initTmpDisp();
#endif


  double dct2Cost                =   MAX_DOUBLE;
  double bestNonDCT2Cost         = MAX_DOUBLE;
  double trGrpBestCost     [ 4 ] = { MAX_DOUBLE, MAX_DOUBLE, MAX_DOUBLE, MAX_DOUBLE };
  double globalBestCost          =   MAX_DOUBLE;
  bool   bestSelFlag       [ 4 ] = { false, false, false, false };
  bool   trGrpCheck        [ 4 ] = { true, true, true, true };
  int    startMTSIdx       [ 4 ] = { 0, 1, 2, 3 };
  int    endMTSIdx         [ 4 ] = { 0, 1, 2, 3 };
#if JVET_W0103_INTRA_MTS
#if JVET_Y0142_ADAPT_INTRA_MTS
  endMTSIdx[0] = 5; //put all MTS candidates in "Grp 0"
#else
  endMTSIdx[0] = 3; //put all MTS candidates in "Grp 0"
#endif
#endif
  double trGrpStopThreshold[ 3 ] = { 1.001, 1.001, 1.001 };
  int    bestMtsFlag             =   0;
  int    bestLfnstIdx            =   0;

#if EXTENDED_LFNST || JVET_W0119_LFNST_EXTENSION
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  const int  maxLfnstIdx         = (CS::isDualITree(*tempCS) && partitioner.chType == CHANNEL_TYPE_CHROMA && (partitioner.currArea().lwidth() < 8 || partitioner.currArea().lheight() < 8))
                                   || (partitioner.currArea().lwidth() > sps.getMaxTbSize() || partitioner.currArea().lheight() > sps.getMaxTbSize()) ? 0 : 3;
#else
  const int  maxLfnstIdx         = ( partitioner.isSepTree( *tempCS ) && partitioner.chType == CHANNEL_TYPE_CHROMA && ( partitioner.currArea().lwidth() < 8 || partitioner.currArea().lheight() < 8 ) )
                                   || ( partitioner.currArea().lwidth() > sps.getMaxTbSize() || partitioner.currArea().lheight() > sps.getMaxTbSize() ) ? 0 : 3;
#endif
#else
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  const int  maxLfnstIdx         = (CS::isDualITree(*tempCS) && partitioner.chType == CHANNEL_TYPE_CHROMA && (partitioner.currArea().lwidth() < 8 || partitioner.currArea().lheight() < 8))
                                   || (partitioner.currArea().lwidth() > sps.getMaxTbSize() || partitioner.currArea().lheight() > sps.getMaxTbSize()) ? 0 : 2;
#else
  const int  maxLfnstIdx         = ( partitioner.isSepTree( *tempCS ) && partitioner.chType == CHANNEL_TYPE_CHROMA && ( partitioner.currArea().lwidth() < 8 || partitioner.currArea().lheight() < 8 ) )
                                   || ( partitioner.currArea().lwidth() > sps.getMaxTbSize() || partitioner.currArea().lheight() > sps.getMaxTbSize() ) ? 0 : 2;
#endif
#endif

  bool       skipOtherLfnst      = false;
  int        startLfnstIdx       = 0;
  int        endLfnstIdx         = sps.getUseLFNST() ? maxLfnstIdx : 0;
#if INTRA_TRANS_ENC_OPT
  if (m_pcEncCfg->getIntraPeriod() == 1)
  {
    CodedCUInfo    &relatedCU = ((EncModeCtrlMTnoRQT *)m_modeCtrl)->getBlkInfo(partitioner.currArea());
    if (isLuma(partitioner.chType) && relatedCU.skipLfnstTest)
    {
      endLfnstIdx = startLfnstIdx;
    }
  }
#endif
#if JVET_W0103_INTRA_MTS
  int grpNumMax = 1;
#else
  int grpNumMax = sps.getUseLFNST() ? m_pcEncCfg->getMTSIntraMaxCand() : 1;
#endif
  m_modeCtrl->setISPWasTested(false);
  m_pcIntraSearch->invalidateBestModeCost();
  if (sps.getUseColorTrans() && !CS::isDualITree(*tempCS))
  {
    if ((m_pcEncCfg->getRGBFormatFlag() && adaptiveColorTrans) || (!m_pcEncCfg->getRGBFormatFlag() && !adaptiveColorTrans))
    {
      m_pcIntraSearch->invalidateBestRdModeFirstColorSpace();
    }
  }

  bool foundZeroRootCbf = false;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (sps.getUseColorTrans())
  {
    CHECK(tempCS->treeType != TREE_D || partitioner.treeType != TREE_D, "localtree should not be applied when adaptive color transform is enabled");
    CHECK(tempCS->modeType != MODE_TYPE_ALL || partitioner.modeType != MODE_TYPE_ALL, "localtree should not be applied when adaptive color transform is enabled");
    CHECK(adaptiveColorTrans && (CS::isDualITree(*tempCS) || partitioner.chType != CHANNEL_TYPE_LUMA), "adaptive color transform cannot be applied to dual-tree");
  }
#endif
#if JVET_AC0094_REF_SAMPLES_OPT
  bool areAboveRightUnavail{ false };
  bool areBelowLeftUnavail{ false };
#endif
#if ENABLE_DIMD
  bool dimdBlending = false;
#if JVET_AC0098_LOC_DEP_DIMD
#if JVET_AB0157_INTRA_FUSION
  int dimdLocDep[DIMD_FUSION_NUM-1] = { 0 };
#else
  int dimdLocDep[2] = { 0 };
#endif
#endif
  int  dimdMode = 0;
#if JVET_AB0157_INTRA_FUSION
  int  dimdBlendMode[DIMD_FUSION_NUM-1] = { 0 };
  int  dimdRelWeight[DIMD_FUSION_NUM] = { 0 };
#else
  int  dimdBlendMode[2] = { 0 };
  int  dimdRelWeight[3] = { 0 };
#endif
  bool dimdDerived = false;

#if JVET_Z0050_DIMD_CHROMA_FUSION && (JVET_AC0094_REF_SAMPLES_OPT)
  int8_t dimdChromaMode = -1;
  int8_t dimdChromaModeSecond = -1;
#endif

  if (isLuma(partitioner.chType))
  {
    CodingUnit cu(tempCS->area);
    cu.cs = tempCS;
    cu.slice = tempCS->slice;
    cu.tileIdx = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
    PredictionUnit pu(tempCS->area);
    pu.cu = &cu;
    pu.cs = tempCS;

#if JVET_AC0094_REF_SAMPLES_OPT
    areAboveRightUnavail = tempCS->getCURestricted(cu.Y().topRight().offset(1, -1), cu, partitioner.chType) == nullptr;
    areBelowLeftUnavail = tempCS->getCURestricted(cu.Y().bottomLeft().offset(-1, 1), cu, partitioner.chType) == nullptr;
    cu.areAboveRightUnavail = areAboveRightUnavail;
    cu.areBelowLeftUnavail = areBelowLeftUnavail;
#endif

    if (cu.slice->getSPS()->getUseDimd())
    {
      const CompArea &area = cu.Y();
      IntraPrediction::deriveDimdMode(bestCS->picture->getRecoBuf(area), area, cu);

      dimdDerived = true;
      dimdBlending = cu.dimdBlending;
#if JVET_AC0098_LOC_DEP_DIMD
#if JVET_AB0157_INTRA_FUSION
      for( int i = 0; i < DIMD_FUSION_NUM-1; i++ )
      {
        dimdLocDep[i] = cu.dimdLocDep[i];
      }
#else
      dimdLocDep[0] = cu.dimdLocDep[0];
      dimdLocDep[1] = cu.dimdLocDep[1];
#endif
#endif
      dimdMode = cu.dimdMode;
#if JVET_AB0157_INTRA_FUSION
      for( int i = 0; i < DIMD_FUSION_NUM-1; i++ )
      {
        dimdBlendMode[i] = cu.dimdBlendMode[i];
      }

      for( int i = 0; i < DIMD_FUSION_NUM; i++ )
      {
        dimdRelWeight[i] = cu.dimdRelWeight[i];
      }
#else
      dimdBlendMode[0] = cu.dimdBlendMode[0];
      dimdBlendMode[1] = cu.dimdBlendMode[1];
      dimdRelWeight[0] = cu.dimdRelWeight[0];
      dimdRelWeight[1] = cu.dimdRelWeight[1];
      dimdRelWeight[2] = cu.dimdRelWeight[2];
#endif
    }

#if SECONDARY_MPM
#if JVET_AD0085_MPM_SORTING
    if (PU::allowMPMSorted(pu))
    {
      m_pcIntraSearch->getMpmListSize() = PU::getIntraMPMs(pu, m_pcIntraSearch->m_intraMPM, m_pcIntraSearch->m_intraNonMPM
#if JVET_AC0094_REF_SAMPLES_OPT
                                                         , true
#endif
                                                         , m_pcIntraSearch
      );
    }
    else
    {
#endif
      m_pcIntraSearch->getMpmListSize() = PU::getIntraMPMs(pu, m_pcIntraSearch->m_intraMPM, m_pcIntraSearch->m_intraNonMPM
#if JVET_AC0094_REF_SAMPLES_OPT
                                                         , false
#endif
      );
#if JVET_AD0085_MPM_SORTING
    }
#endif
#endif
  }
#if JVET_Z0050_DIMD_CHROMA_FUSION && (JVET_AC0094_REF_SAMPLES_OPT)
  if (tempCS->slice->getSPS()->getUseDimd() && (CS::isDualITree(*tempCS) ? isChroma(partitioner.chType) : false))
  {
    CodingUnit cu(tempCS->area);
    cu.cs = tempCS;
    cu.slice = tempCS->slice;
    cu.tileIdx = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
    const CompArea areaCb = tempCS->area.Cb();
    const CompArea areaCr = tempCS->area.Cr();
    const CompArea lumaArea = CompArea(COMPONENT_Y, (tempCS->area).chromaFormat, areaCb.lumaPos(), recalcSize((tempCS->area).chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, areaCb.size()));
    IntraPrediction::deriveDimdChromaMode(bestCS->picture->getRecoBuf(lumaArea), bestCS->picture->getRecoBuf(areaCb), bestCS->picture->getRecoBuf(areaCr), lumaArea, areaCb, areaCr, cu);
    dimdChromaMode = cu.dimdChromaMode;
    dimdChromaModeSecond = cu.dimdChromaModeSecond;
  }
#endif
#elif SECONDARY_MPM
  if( isLuma( partitioner.chType ) )
  {
    CodingUnit cu( tempCS->area );
    cu.cs = tempCS;
    cu.slice = tempCS->slice;
    cu.tileIdx = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
    PredictionUnit pu( tempCS->area );
    pu.cu = &cu;
    pu.cs = tempCS;

    m_pcIntraSearch->getMpmListSize() = PU::getIntraMPMs( pu, m_pcIntraSearch->getMPMList(), m_pcIntraSearch->getNonMPMList() );
  }
#endif

#if JVET_W0123_TIMD_FUSION
  bool timdDerived = false;
#endif
#if TMP_FAST_ENC
  bool tmpDerived = 0;
#endif
#if JVET_AB0157_TMRL
  bool tmrlDerived = false;
#endif
#if INTRA_TRANS_ENC_OPT
  m_pcIntraSearch->m_skipTimdLfnstMtsPass = false;
  m_modeCtrl->resetLfnstCost();
#endif
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  m_pcIntraSearch->m_skipCCCMSATD = false;
#endif
#if JVET_AD0202_CCCM_MDF
  m_pcIntraSearch->m_skipCCCMwithMdfSATD = false;
#endif
#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
  if (tempCS->slice->getSPS()->getUseDdCcpFusion())
  {
    m_pcIntraSearch->m_skipDdCcpListConstruction = false;
    m_pcIntraSearch->firstTransformDdccp = true;
    CodingUnit cu(tempCS->area);
    cu.cs = tempCS;
    cu.slice = tempCS->slice;
    cu.tileIdx = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
    PredictionUnit pu(tempCS->area);
    pu.cu = &cu;
    pu.cs = tempCS;
    CodingStructure &cs = *cu.cs;
    const UnitArea localUnitArea(cs.area.chromaFormat, Area(0, 0, (pu.Cb().width) << 1, (pu.Cb().height) << 1));
    m_pcIntraSearch->m_ddCcpStorageTemp = m_pcIntraSearch->m_ddCcpStorage.getBuf(localUnitArea);
    for (int i = 0; i < 2; i++)
    {
      m_pcIntraSearch->m_ddCcpFusionStorageTemp[i] = m_pcIntraSearch->m_ddCcpFusionStorage[i].getBuf(localUnitArea);
    }
    m_pcIntraSearch->m_skipDdCcpMergeFusionList = false;
    for (int i = 0; i < 2; i++)
    {
      m_pcIntraSearch->m_ddccpMergeFusionCost[i] = MAX_DOUBLE;
    }
  }
#endif
  for( int trGrpIdx = 0; trGrpIdx < grpNumMax; trGrpIdx++ )
  {
    const uint8_t startMtsFlag = trGrpIdx > 0;
    const uint8_t endMtsFlag   = sps.getUseLFNST() ? considerMtsSecondPass : 0;

    if( ( trGrpIdx == 0 || ( !skipSecondMtsPass && considerMtsSecondPass ) ) && trGrpCheck[ trGrpIdx ] )
    {
      for( int lfnstIdx = startLfnstIdx; lfnstIdx <= endLfnstIdx; lfnstIdx++ )
      {
        for( uint8_t mtsFlag = startMtsFlag; mtsFlag <= endMtsFlag; mtsFlag++ )
        {
          if (sps.getUseColorTrans() && !CS::isDualITree(*tempCS))
          {
            m_pcIntraSearch->setSavedRdModeIdx(trGrpIdx*(NUM_LFNST_NUM_PER_SET * 2) + lfnstIdx * 2 + mtsFlag);
          }
          if (mtsFlag > 0 && lfnstIdx > 0)
          {
            continue;
          }
          //3) if interHad is 0, only try further modes if some intra mode was already better than inter
          if( sps.getUseLFNST() && m_pcEncCfg->getUsePbIntraFast() && !tempCS->slice->isIntra() && bestCU && CU::isInter( *bestCS->getCU( partitioner.chType ) ) && interHad == 0 )
          {
            continue;
          }

          tempCS->initStructData( encTestMode.qp );

          CodingUnit &cu      = tempCS->addCU( CS::getArea( *tempCS, tempCS->area, partitioner.chType ), partitioner.chType );

          partitioner.setCUData( cu );
          cu.slice            = tempCS->slice;
          cu.tileIdx          = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
          cu.skip             = false;
          cu.mmvdSkip = false;
          cu.predMode         = MODE_INTRA;
          cu.chromaQpAdj      = m_cuChromaQpOffsetIdxPlus1;
          cu.qp               = encTestMode.qp;
#if ENABLE_DIMD
          cu.dimd = false;
          if( dimdDerived )
          {
            cu.dimdBlending = dimdBlending;
#if JVET_AC0098_LOC_DEP_DIMD
#if JVET_AB0157_INTRA_FUSION
            for( int i = 0; i < DIMD_FUSION_NUM-1; i++ )
            {
              cu.dimdLocDep[i] = dimdLocDep[i];
            }
#else
            cu.dimdLocDep[0] = dimdLocDep[0];
            cu.dimdLocDep[1] = dimdLocDep[1];
#endif
#endif
            cu.dimdMode = dimdMode;
#if JVET_AB0157_INTRA_FUSION
            for( int i = 0; i < DIMD_FUSION_NUM-1; i++ )
            {
              cu.dimdBlendMode[i] = dimdBlendMode[i];
            }

            for( int i = 0; i < DIMD_FUSION_NUM; i++ )
            {
              cu.dimdRelWeight[i] = dimdRelWeight[i];
            }
#else
            cu.dimdBlendMode[0] = dimdBlendMode[0];
            cu.dimdBlendMode[1] = dimdBlendMode[1];
            cu.dimdRelWeight[0] = dimdRelWeight[0];
            cu.dimdRelWeight[1] = dimdRelWeight[1];
            cu.dimdRelWeight[2] = dimdRelWeight[2];
#endif
          }
#if JVET_Z0050_DIMD_CHROMA_FUSION && (JVET_AC0094_REF_SAMPLES_OPT)
          if (cu.slice->getSPS()->getUseDimd() && (CS::isDualITree(*tempCS) ? isChroma(partitioner.chType) : false))
          {
            cu.dimdChromaMode = dimdChromaMode;
            cu.dimdChromaModeSecond = dimdChromaModeSecond;
          }
#endif
#endif
          cu.lfnstIdx         = lfnstIdx;
          cu.mtsFlag          = mtsFlag;
          cu.ispMode          = NOT_INTRA_SUBPARTITIONS;
          cu.colorTransform = adaptiveColorTrans;

          CU::addPUs( cu );
#if JVET_AC0094_REF_SAMPLES_OPT
#if !ENABLE_DIMD && !SECONDARY_MPM
          areAboveRightUnavail =
            tempCS->getCURestricted(cu.Y().topRight().offset(cu.lwidth(), -1), cu, partitioner.chType) == nullptr;
          areBelowLeftUnavail =
            tempCS->getCURestricted(cu.Y().bottomLeft().offset(-1, cu.lheight()), cu, partitioner.chType) == nullptr;
#endif
          cu.areAboveRightUnavail = areAboveRightUnavail;
          cu.areBelowLeftUnavail  = areBelowLeftUnavail;
#endif
#if (JVET_AG0146_DIMD_ITMP_IBC || JVET_AG0152_SGPM_ITMP_IBC || JVET_AG0151_INTRA_TMP_MERGE_MODE)
          if (bvListDerived == 0)
          {
            PU::getItmpMergeCandidate(*cu.firstPU, m_pcIntraSearch->m_bvBasedMergeCandidates);
            bvListDerived = 1;
          }
#endif
#if JVET_AG0146_DIMD_ITMP_IBC
          if (bvDimdDerived == 0)
          {
            const CompArea& area = cu.Y();
            m_pcIntraSearch->getBestNonAnglularMode(bestCS->picture->getRecoBuf(area), area, cu, m_pcIntraSearch->m_bvBasedMergeCandidates);
            isBvDimd = cu.isBvDimd;
            bvDimd = cu.bvDimd;
            bvDimdDerived = 1;
          }
          else
          {
            cu.isBvDimd = isBvDimd;
            cu.bvDimd = bvDimd;
          }
#endif
#if JVET_W0123_TIMD_FUSION
          cu.timd = false;
          if (isLuma(partitioner.chType) && cu.slice->getSPS()->getUseTimd())
          {
            if (cu.lwidth() * cu.lheight() > 1024 && cu.slice->getSliceType() == I_SLICE)
            {
              timdDerived = true;
            }
            if (!timdDerived)
            {
              const CompArea &area = cu.Y();

#if JVET_AB0155_SGPM
              cu.timdMode = m_pcIntraSearch->deriveTimdMode(bestCS->picture->getRecoBuf(area), area, cu, true, true);
#else
              cu.timdMode = m_pcIntraSearch->deriveTimdMode(bestCS->picture->getRecoBuf(area), area, cu);
#endif
              timdMode = cu.timdMode;
              timdDerived = true;
              timdModeSecondary = cu.timdModeSecondary;
              timdIsBlended     = cu.timdIsBlended;
#if JVET_AC0094_REF_SAMPLES_OPT
              timdModeCheckWA          = cu.timdModeCheckWA;
              timdModeSecondaryCheckWA = cu.timdModeSecondaryCheckWA;
#endif
#if JVET_AG0092_ENHANCED_TIMD_FUSION
              timdModeNonAng = cu.timdModeNonAng;
              for( int i = 0; i < TIMD_FUSION_NUM; i++ )
              {
                timdFusionWeight[i] = cu.timdFusionWeight[i];
                timdLocDep[i] = cu.timdLocDep[i];
              }
#else
              timdFusionWeight[0] = cu.timdFusionWeight[0];
              timdFusionWeight[1] = cu.timdFusionWeight[1];
#endif
#if JVET_AB0155_SGPM
              timdHorMode = cu.timdHor;
              timdVerMode = cu.timdVer;
#endif
            }
            else
            {
              cu.timdMode = timdMode;
#if JVET_AC0094_REF_SAMPLES_OPT
              cu.timdModeCheckWA          = timdModeCheckWA;
              cu.timdModeSecondaryCheckWA = timdModeSecondaryCheckWA;
#endif
              cu.timdModeSecondary = timdModeSecondary;
              cu.timdIsBlended     = timdIsBlended;
#if JVET_AG0092_ENHANCED_TIMD_FUSION
              cu.timdModeNonAng = timdModeNonAng;
              for( int i = 0; i < TIMD_FUSION_NUM; i++ )
              {
                cu.timdFusionWeight[i] = timdFusionWeight[i];
                cu.timdLocDep[i] = timdLocDep[i];
              }
#else
              cu.timdFusionWeight[0] = timdFusionWeight[0];
              cu.timdFusionWeight[1] = timdFusionWeight[1];
#endif

#if JVET_AB0155_SGPM
              cu.timdHor = timdHorMode;
              cu.timdVer = timdVerMode;
#endif
            }
          }
#endif
#if JVET_AB0157_TMRL
          cu.tmrlFlag = false;
          if (CU::allowTmrl(cu))
          {
            if (!tmrlDerived)
            {
              m_pcIntraSearch->getTmrlList(cu);
              tmrlDerived = true;
            }
          }
#endif

#if TMP_FAST_ENC
          if (isLuma(partitioner.chType) && cu.slice->getSPS()->getUseIntraTMP())
          {
            const bool tpmAllowed = sps.getUseIntraTMP() && isLuma(partitioner.chType) && ((cu.lfnstIdx == 0) || allowLfnstWithTmp());
            const bool testTpm = tpmAllowed && (cu.lwidth() <= sps.getIntraTMPMaxSize() && cu.lheight() <= sps.getIntraTMPMaxSize());
            if (testTpm == 1 && tmpDerived == 0)
            {
#if JVET_W0069_TMP_BOUNDARY
              RefTemplateType templateType = m_pcIntraSearch->getRefTemplateType(cu, cu.blocks[COMPONENT_Y]);
              if (templateType != NO_TEMPLATE)
              {
                m_pcIntraSearch->getTargetTemplate(&cu, cu.lwidth(), cu.lheight(), templateType);
                m_pcIntraSearch->candidateSearchIntra(&cu, cu.lwidth(), cu.lheight(), templateType
#if JVET_AG0136_INTRA_TMP_LIC || (JVET_AG0146_DIMD_ITMP_IBC || JVET_AG0152_SGPM_ITMP_IBC || JVET_AG0151_INTRA_TMP_MERGE_MODE)
                                                      , true
#endif
                                                      );
#if JVET_AG0136_INTRA_TMP_LIC
                PelBuf bufDumb;
                cu.ibcLicFlag = true;
                cu.tmpLicFlag = true;
                for (int licIdc = 0; licIdc < (cu.slice->getSPS()->getItmpLicExtension() ? 4 : 1); licIdc++)
                {
                  cu.ibcLicIdx = licIdc;
                  for (int idx = 0; idx < m_pcIntraSearch->getTmpNumCandUseMR(); idx++)
                  {
                    m_pcIntraSearch->setBvMvFromMemory(cu, idx, true);
                    m_pcInterSearch->LicItmp(*(cu.firstPU), bufDumb, false);
                    m_pcIntraSearch->getMemLicParams(licIdc, idx) = m_pcInterSearch->getArrayLicParams();
                  }
                  for (int idx = m_pcIntraSearch->getTmpNumCandUseMR(); idx < MTMP_NUM; idx++)
                  {
                    m_pcIntraSearch->getMemLicParams(licIdc, idx) = m_pcIntraSearch->getMemLicParams(licIdc, idx - m_pcIntraSearch->getTmpNumCandUseMR());
                  }
                }
                cu.ibcLicFlag = false;
                cu.tmpLicFlag = false;
#endif
#if JVET_AD0086_ENHANCED_INTRA_TMP
                for (int idx = 0; idx < m_pcIntraSearch->getTmpNumCand() && idx < MTMP_NUM; idx++)
                {
                  cu.tmpIdx = idx;
                  m_pcIntraSearch->xCalTmpFlmParam(&cu, cu.lwidth(), cu.lheight(), templateType);
                }
                cu.tmpIdx = 0;
                m_pcIntraSearch->xTMPBuildFusionCandidate(cu, templateType
#if JVET_AG0136_INTRA_TMP_LIC
                                                          , false
#endif
                                                          );
#if !JVET_AG0136_INTRA_TMP_LIC
                cu.tmpIdx = 0;
                cu.tmpFusionFlag = 0;
#endif
#endif
#if JVET_AG0136_INTRA_TMP_LIC
                m_pcIntraSearch->xTMPBuildFusionCandidate(cu, templateType, true);
#endif
              }
#else
              m_pcIntraSearch->getTargetTemplate(&cu, cu.lwidth(), cu.lheight());
              m_pcIntraSearch->candidateSearchIntra(&cu, cu.lwidth(), cu.lheight());
#endif
              tmpDerived = 1;
#if (JVET_AG0146_DIMD_ITMP_IBC || JVET_AG0152_SGPM_ITMP_IBC || JVET_AG0151_INTRA_TMP_MERGE_MODE)
              tmpXdisp = cu.tmpXdisp;
              tmpYdisp = cu.tmpYdisp;
#endif
            }
#if (JVET_AG0146_DIMD_ITMP_IBC || JVET_AG0152_SGPM_ITMP_IBC || JVET_AG0151_INTRA_TMP_MERGE_MODE)
            else
            {
              cu.tmpXdisp = tmpXdisp;
              cu.tmpYdisp = tmpYdisp;
            }
#endif
          }
#endif
          tempCS->interHad    = interHad;

          m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;

          bool validCandRet = false;
          if( isLuma( partitioner.chType ) )
          {
            //ISP uses the value of the best cost so far (luma if it is the fast version) to avoid test non-necessary subpartitions
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
            double bestCostSoFar = CS::isDualITree(*tempCS) ? m_modeCtrl->getBestCostWithoutSplitFlags() : bestCU && bestCU->predMode == MODE_INTRA ? bestCS->lumaCost : bestCS->cost;
            if (CS::isDualITree(*tempCS) && encTestMode.maxCostAllowed < bestCostSoFar)
#else
            double bestCostSoFar = partitioner.isSepTree(*tempCS) ? m_modeCtrl->getBestCostWithoutSplitFlags() : bestCU && bestCU->predMode == MODE_INTRA ? bestCS->lumaCost : bestCS->cost;
            if (partitioner.isSepTree(*tempCS) && encTestMode.maxCostAllowed < bestCostSoFar)
#endif
            {
              bestCostSoFar = encTestMode.maxCostAllowed;
            }
            validCandRet = m_pcIntraSearch->estIntraPredLumaQT(cu, partitioner, bestCostSoFar, mtsFlag, startMTSIdx[trGrpIdx], endMTSIdx[trGrpIdx], (trGrpIdx > 0), !cu.colorTransform ? bestCS : nullptr
#if JVET_AG0136_INTRA_TMP_LIC
                                                               , m_pcInterSearch
#endif
            );
            if ((!validCandRet || (cu.ispMode && cu.firstTU->cbf[COMPONENT_Y] == 0)))
            {
              continue;
            }
#if JVET_W0123_TIMD_FUSION
            PU::spanIpmInfoIntra(*cu.firstPU);
#endif
#if JVET_AB0061_ITMP_BV_FOR_IBC
            if (cu.tmpFlag)
            {
              PU::spanMotionInfo(*cu.firstPU);
            }
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
            if (!cu.tmpFlag)
            {
               PU::spanSCCInfo(*cu.firstPU);
            }
#endif
#if JVET_W0123_TIMD_FUSION
            if (m_pcEncCfg->getUseFastISP() && validCandRet && !mtsFlag && !lfnstIdx && !cu.colorTransform && !cu.timd)
#else
            if (m_pcEncCfg->getUseFastISP() && validCandRet && !mtsFlag && !lfnstIdx && !cu.colorTransform)
#endif
            {
              m_modeCtrl->setISPMode(cu.ispMode);
              m_modeCtrl->setISPLfnstIdx(cu.lfnstIdx);
              m_modeCtrl->setMIPFlagISPPass(cu.mipFlag);
#if JVET_V0130_INTRA_TMP
			        m_modeCtrl->setTPMFlagISPPass(cu.tmpFlag);
#endif
              m_modeCtrl->setBestISPIntraModeRelCU(cu.ispMode ? PU::getFinalIntraMode(*cu.firstPU, CHANNEL_TYPE_LUMA) : UINT8_MAX);
              m_modeCtrl->setBestDCT2NonISPCostRelCU(m_modeCtrl->getMtsFirstPassNoIspCost());
#if JVET_AG0058_EIP
              m_modeCtrl->setAeipFlagISPPass(cu.eipFlag);
#endif
            }

            if (sps.getUseColorTrans() && m_pcEncCfg->getRGBFormatFlag() && !CS::isDualITree(*tempCS) && !cu.colorTransform)
            {
              double curLumaCost = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);
              if (curLumaCost > bestCS->cost)
              {
                continue;
              }
            }

            useIntraSubPartitions = cu.ispMode != NOT_INTRA_SUBPARTITIONS;
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
            if (!CS::isDualITree(*tempCS))
#else
            if( !partitioner.isSepTree( *tempCS ) )
#endif
            {
              tempCS->lumaCost = m_pcRdCost->calcRdCost( tempCS->fracBits, tempCS->dist );
              if( useIntraSubPartitions )
              {
                //the difference between the best cost so far and the current luma cost is stored to avoid testing the Cr component if the cost of luma + Cb is larger than the best cost
                maxCostAllowedForChroma = bestCS->cost < MAX_DOUBLE ? bestCS->cost - tempCS->lumaCost : MAX_DOUBLE;
              }
            }

            if (m_pcEncCfg->getUsePbIntraFast() && tempCS->dist == std::numeric_limits<Distortion>::max()
                && tempCS->interHad == 0)
            {
              interHad = 0;
              // JEM assumes only perfect reconstructions can from now on beat the inter mode
              m_modeCtrl->enforceInterHad( 0 );
              continue;
            }
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
            if (!CS::isDualITree(*tempCS))
#else
            if( !partitioner.isSepTree( *tempCS ) )
#endif
            {
              if (!cu.colorTransform)
              {
#if JVET_Z0118_GDR
                cu.cs->updateReconMotIPM(cu.Y()); // xcomrpessCU - need 
#else
                cu.cs->picture->getRecoBuf(cu.Y()).copyFrom(cu.cs->getRecoBuf(COMPONENT_Y));
#endif
                cu.cs->picture->getPredBuf(cu.Y()).copyFrom(cu.cs->getPredBuf(COMPONENT_Y));
              }
              else
              {
#if JVET_Z0118_GDR
                cu.cs->updateReconMotIPM(cu); // xcomrpessCU - need 
#else
                cu.cs->picture->getRecoBuf(cu).copyFrom(cu.cs->getRecoBuf(cu));
#endif
                cu.cs->picture->getPredBuf(cu).copyFrom(cu.cs->getPredBuf(cu));
              }
            }
          }
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
          if (tempCS->area.chromaFormat != CHROMA_400 && (partitioner.chType == CHANNEL_TYPE_CHROMA || !CS::isDualITree(*tempCS)) && !cu.colorTransform)
#else
          if( tempCS->area.chromaFormat != CHROMA_400 && ( partitioner.chType == CHANNEL_TYPE_CHROMA || !cu.isSepTree() ) && !cu.colorTransform )
#endif
          {
            TUIntraSubPartitioner subTuPartitioner( partitioner );
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
            m_pcIntraSearch->estIntraPredChromaQT(cu, (!useIntraSubPartitions || (CS::isDualITree(*tempCS) && !isLuma(CHANNEL_TYPE_CHROMA))) ? partitioner : subTuPartitioner, maxCostAllowedForChroma
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                                , m_pcInterSearch
#endif
            );
#else
            m_pcIntraSearch->estIntraPredChromaQT( cu, ( !useIntraSubPartitions || ( cu.isSepTree() && !isLuma( CHANNEL_TYPE_CHROMA ) ) ) ? partitioner : subTuPartitioner, maxCostAllowedForChroma 
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                                 , m_pcInterSearch
#endif
            );
#endif
            if( useIntraSubPartitions && !cu.ispMode )
            {
              //At this point the temp cost is larger than the best cost. Therefore, we can already skip the remaining calculations
              continue;
            }
          }

          cu.rootCbf = false;

          for( uint32_t t = 0; t < getNumberValidTBlocks( *cu.cs->pcv ); t++ )
          {
            cu.rootCbf |= cu.firstTU->cbf[t] != 0;
          }

          if (!cu.rootCbf)
          {
            cu.colorTransform = false;
            foundZeroRootCbf = true;
          }
          for (auto& pu : CU::traversePUs(cu))
          {
            m_pcIntraSearch->setLumaIntraPredIdx(pu);
          }

          // Get total bits for current mode: encode CU
          m_CABACEstimator->resetBits();

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
          if ((!cu.cs->slice->isIntra() || cu.cs->slice->getUseIBC())
#else
          if ((!cu.cs->slice->isIntra() || cu.cs->slice->getSPS()->getIBCFlag())
#endif
            && cu.Y().valid()
            )
          {
            m_CABACEstimator->cu_skip_flag ( cu );
          }
          m_CABACEstimator->pred_mode      ( cu );
#if ENABLE_DIMD
          m_CABACEstimator->cu_dimd_flag   ( cu );
#endif
          m_CABACEstimator->adaptive_color_transform(cu);
          m_CABACEstimator->cu_pred_data   ( cu );

          // Encode Coefficients
          CUCtx cuCtx;
          cuCtx.isDQPCoded = true;
          cuCtx.isChromaQpAdjCoded = true;
          m_CABACEstimator->cu_residual( cu, partitioner, cuCtx );

          tempCS->fracBits = m_CABACEstimator->getEstFracBits();
          tempCS->cost     = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);


          double tmpCostWithoutSplitFlags = tempCS->cost;
          xEncodeDontSplit( *tempCS, partitioner );

          xCheckDQP( *tempCS, partitioner );
          xCheckChromaQPOffset( *tempCS, partitioner );

          // Check if low frequency non-separable transform (LFNST) is too expensive
          if( lfnstIdx && !cuCtx.lfnstLastScanPos && !cu.ispMode )
          {
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
            bool cbfAtZeroDepth = CS::isDualITree(*tempCS) ?
#else
            bool cbfAtZeroDepth = cu.isSepTree() ?
#endif
                                       cu.rootCbf
                                     : (tempCS->area.chromaFormat != CHROMA_400 && std::min( cu.firstTU->blocks[ 1 ].width, cu.firstTU->blocks[ 1 ].height ) < 4) ?
                                            TU::getCbfAtDepth( *cu.firstTU, COMPONENT_Y, 0 )
                                          : cu.rootCbf;
#if INTRA_TRANS_ENC_OPT
            if (CS::isDualITree(*tempCS) && (partitioner.chType == CHANNEL_TYPE_LUMA))
            {
              CHECK(cbfAtZeroDepth, "such case should be wrapped out during the RD!");
            }
#endif
            if( cbfAtZeroDepth )
            {
              tempCS->cost = MAX_DOUBLE;
              tmpCostWithoutSplitFlags = MAX_DOUBLE;
            }
          }

          if (isLuma(partitioner.chType) && cu.firstTU->mtsIdx[COMPONENT_Y] > MTS_SKIP)
          {
            CHECK(!cuCtx.mtsLastScanPos, "MTS is disallowed to only contain DC coefficient");
          }

          if( mtsFlag == 0 && lfnstIdx == 0 )
          {
            dct2Cost = tempCS->cost;
          }
          else if (tmpCostWithoutSplitFlags < bestNonDCT2Cost)
          {
            bestNonDCT2Cost = tmpCostWithoutSplitFlags;
          }

          if( tempCS->cost < bestCS->cost )
          {
            m_modeCtrl->setBestCostWithoutSplitFlags( tmpCostWithoutSplitFlags );
          }

          if( !mtsFlag ) static_cast< double& >( costSize2Nx2NmtsFirstPass ) = tempCS->cost;

          if( sps.getUseLFNST() && !tempCS->cus.empty() )
          {
            skipOtherLfnst = m_modeCtrl->checkSkipOtherLfnst( encTestMode, tempCS, partitioner );
          }

          xCalDebCost( *tempCS, partitioner );
          tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();

#if WCG_EXT
          DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda( true ) );
#else
          DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );
#endif
          if (sps.getUseColorTrans() && !CS::isDualITree(*tempCS))
          {
            int colorSpaceIdx = ((m_pcEncCfg->getRGBFormatFlag() && adaptiveColorTrans) || (!m_pcEncCfg->getRGBFormatFlag() && !adaptiveColorTrans)) ? 0 : 1;
            if (tempCS->cost < tempCS->tmpColorSpaceIntraCost[colorSpaceIdx])
            {
              tempCS->tmpColorSpaceIntraCost[colorSpaceIdx] = tempCS->cost;
              bestCS->tmpColorSpaceIntraCost[colorSpaceIdx] = tempCS->cost;
            }
          }
         
          if( !sps.getUseLFNST() )
          {
            xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );
          }
          else
          {
            if( xCheckBestMode( tempCS, bestCS, partitioner, encTestMode ) )
            {
              trGrpBestCost[ trGrpIdx ] = globalBestCost = bestCS->cost;
              bestSelFlag  [ trGrpIdx ] = true;
              bestMtsFlag               = mtsFlag;
              bestLfnstIdx              = lfnstIdx;

              if( bestCS->cus.size() == 1 )
              {
                CodingUnit &cu = *bestCS->cus.front();
                if (cu.firstTU->mtsIdx[COMPONENT_Y] == MTS_SKIP)
                {
                  if( ( floorLog2( cu.firstTU->blocks[ COMPONENT_Y ].width ) + floorLog2( cu.firstTU->blocks[ COMPONENT_Y ].height ) ) >= 6 )
                  {
                    endLfnstIdx = 0;
                  }
                }
              }
            }
            
            //we decide to skip the non-DCT-II transforms and LFNST according to the ISP results
            if ((endMtsFlag > 0 || endLfnstIdx > 0) && (cu.ispMode || (bestCS && bestCS->cus[0]->ispMode)) && tempCS->slice->isIntra() && m_pcEncCfg->getUseFastISP())
            {
              double bestCostDct2NoIsp = m_modeCtrl->getMtsFirstPassNoIspCost();
              double bestIspCost       = m_modeCtrl->getIspCost();
              CHECKD( cu.ispMode && bestCostDct2NoIsp <= bestIspCost, "wrong cost!" );
              double threshold = 1.4;
              
              double lfnstThreshold = 1.01 * threshold;
              if( m_modeCtrl->getStopNonDCT2Transforms() || bestCostDct2NoIsp > bestIspCost*lfnstThreshold )
              {
                endLfnstIdx = lfnstIdx;
              }

              if ( m_modeCtrl->getStopNonDCT2Transforms() || bestCostDct2NoIsp > bestIspCost*threshold )
              {
                skipSecondMtsPass = true;
                m_modeCtrl->setSkipSecondMTSPass( true );
                break;
              }
            }
            //now we check whether the second pass of SIZE_2Nx2N and the whole Intra SIZE_NxN should be skipped or not
            if( !mtsFlag && !tempCS->slice->isIntra() && bestCU && bestCU->predMode != MODE_INTRA )
            {
              const double thEmtInterFastSkipIntra = 1.4; // Skip checking Intra if "2Nx2N using DCT2" is worse than best Inter mode
              if( costSize2Nx2NmtsFirstPass > thEmtInterFastSkipIntra * bestInterCost )
              {
                skipSecondMtsPass = true;
                m_modeCtrl->setSkipSecondMTSPass( true );
                break;
              }
            }
#if JVET_W0103_INTRA_MTS
            if (lfnstIdx && m_modeCtrl->getMtsFirstPassNoIspCost() != MAX_DOUBLE && isLuma(partitioner.chType))
            {
              double threshold = 1.5;
              if (m_modeCtrl->getMtsFirstPassNoIspCost() > threshold * bestCS->cost)
              {
                endLfnstIdx = lfnstIdx;
              }
            }
#endif
          }

        } //for emtCuFlag
        if( skipOtherLfnst )
        {
          startLfnstIdx = lfnstIdx;
          endLfnstIdx   = lfnstIdx;
          break;
        }
      } //for lfnstIdx
    } //if (!skipSecondMtsPass && considerMtsSecondPass && trGrpCheck[iGrpIdx])

    if( sps.getUseLFNST() && trGrpIdx < 3 )
    {
      trGrpCheck[ trGrpIdx + 1 ] = false;

      if( bestSelFlag[ trGrpIdx ] && considerMtsSecondPass )
      {
        double dCostRatio = dct2Cost / trGrpBestCost[ trGrpIdx ];
        trGrpCheck[ trGrpIdx + 1 ] = ( bestMtsFlag != 0 || bestLfnstIdx != 0 ) && dCostRatio < trGrpStopThreshold[ trGrpIdx ];
      }
    }
  } //trGrpIdx

  if(!adaptiveColorTrans)
  m_modeCtrl->setBestNonDCT2Cost(bestNonDCT2Cost);
  return foundZeroRootCbf;
}


void EncCu::xCheckPLT(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  if (((partitioner.currArea().lumaSize().width * partitioner.currArea().lumaSize().height <= 16) && (isLuma(partitioner.chType)) )
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
        || ((partitioner.currArea().chromaSize().width * partitioner.currArea().chromaSize().height <= 16) && (!isLuma(partitioner.chType)) && CS::isDualITree(*tempCS)) )
#else
        || ((partitioner.currArea().chromaSize().width * partitioner.currArea().chromaSize().height <= 16) && (!isLuma(partitioner.chType)) && partitioner.isSepTree(*tempCS) )
        || (partitioner.isLocalSepTree(*tempCS)  && (!isLuma(partitioner.chType))  )  )
#endif
  {
    return;
  }
  tempCS->initStructData(encTestMode.qp);
  CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, partitioner.chType), partitioner.chType);
  partitioner.setCUData(cu);
  cu.slice = tempCS->slice;
  cu.tileIdx = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
  cu.skip = false;
  cu.mmvdSkip = false;
  cu.predMode = MODE_PLT;

  cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
  cu.qp = encTestMode.qp;
  cu.bdpcmMode = 0;

  tempCS->addPU(CS::getArea(*tempCS, tempCS->area, partitioner.chType), partitioner.chType);
  tempCS->addTU(CS::getArea(*tempCS, tempCS->area, partitioner.chType), partitioner.chType);
  // Search
  tempCS->dist = 0;
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (CS::isDualITree(*tempCS))
#else
  if (cu.isSepTree())
#endif
  {
    if (isLuma(partitioner.chType))
    {
      m_pcIntraSearch->PLTSearch(*tempCS, partitioner, COMPONENT_Y, 1);
    }
    if (tempCS->area.chromaFormat != CHROMA_400 && (partitioner.chType == CHANNEL_TYPE_CHROMA))
    {
      m_pcIntraSearch->PLTSearch(*tempCS, partitioner, COMPONENT_Cb, 2);
    }
  }
  else
  {
    if( cu.chromaFormat != CHROMA_400 )
    {
      m_pcIntraSearch->PLTSearch(*tempCS, partitioner, COMPONENT_Y, 3);
    }
    else
    {
      m_pcIntraSearch->PLTSearch(*tempCS, partitioner, COMPONENT_Y, 1);
    }
  }


  m_CABACEstimator->getCtx() = m_CurrCtx->start;
  m_CABACEstimator->resetBits();
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if ((!cu.cs->slice->isIntra() || cu.cs->slice->getUseIBC())
#else
  if ((!cu.cs->slice->isIntra() || cu.cs->slice->getSPS()->getIBCFlag())
#endif
    && cu.Y().valid())
  {
    m_CABACEstimator->cu_skip_flag(cu);
  }
  m_CABACEstimator->pred_mode(cu);

#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
  if (isLuma(partitioner.chType))
  {
    PU::spanSCCInfo(*cu.firstPU);
  }
#endif

  // signaling
  CUCtx cuCtx;
  cuCtx.isDQPCoded = true;
  cuCtx.isChromaQpAdjCoded = true;
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (CS::isDualITree(*tempCS))
#else
  if (cu.isSepTree())
#endif
  {
    if (isLuma(partitioner.chType))
    {
      m_CABACEstimator->cu_palette_info(cu, COMPONENT_Y, 1, cuCtx);
    }
    if (tempCS->area.chromaFormat != CHROMA_400 && (partitioner.chType == CHANNEL_TYPE_CHROMA))
    {
      m_CABACEstimator->cu_palette_info(cu, COMPONENT_Cb, 2, cuCtx);
    }
  }
  else
  {
    if( cu.chromaFormat != CHROMA_400 )
    {
      m_CABACEstimator->cu_palette_info(cu, COMPONENT_Y, 3, cuCtx);
    }
    else
    {
      m_CABACEstimator->cu_palette_info(cu, COMPONENT_Y, 1, cuCtx);
    }
  }
  tempCS->fracBits = m_CABACEstimator->getEstFracBits();
  tempCS->cost = m_pcRdCost->calcRdCost(tempCS->fracBits, tempCS->dist);

  xEncodeDontSplit(*tempCS, partitioner);
  xCheckDQP(*tempCS, partitioner);
  xCheckChromaQPOffset( *tempCS, partitioner );
  xCalDebCost(*tempCS, partitioner);
  tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();

  const Area currCuArea = cu.block(getFirstComponentOfChannel(partitioner.chType));
  cu.slice->m_mapPltCost[isChroma(partitioner.chType)][currCuArea.pos()][currCuArea.size()] = tempCS->cost;
#if WCG_EXT
  DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda(true));
#else
  DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda());
#endif
  xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);
}

void EncCu::xCheckDQP( CodingStructure& cs, Partitioner& partitioner, bool bKeepCtx )
{
  CHECK( bKeepCtx && cs.cus.size() <= 1 && partitioner.getImplicitSplit( cs ) == CU_DONT_SPLIT, "bKeepCtx should only be set in split case" );
  CHECK( !bKeepCtx && cs.cus.size() > 1, "bKeepCtx should never be set for non-split case" );

  if( !cs.pps->getUseDQP() )
  {
    return;
  }

#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (CS::isDualITree(cs) && isChroma(partitioner.chType))
#else
  if (partitioner.isSepTree(cs) && isChroma(partitioner.chType))
#endif
  {
    return;
  }

  if( !partitioner.currQgEnable() ) // do not consider split or leaf/not leaf QG condition (checked by caller)
  {
    return;
  }


  CodingUnit* cuFirst = cs.getCU( partitioner.chType );

  CHECK( !cuFirst, "No CU available" );

  bool hasResidual = false;
  for( const auto &cu : cs.cus )
  {
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if ( cu->rootCbf )
#else
    //not include the chroma CU because chroma CU is decided based on corresponding luma QP and deltaQP is not signaled at chroma CU
    if( cu->rootCbf && !isChroma( cu->chType ))
#endif
    {
      hasResidual = true;
      break;
    }
  }

  int predQP = CU::predictQP( *cuFirst, cs.prevQP[partitioner.chType] );

  if( hasResidual )
  {
    TempCtx ctxTemp( m_ctxCache );
    if( !bKeepCtx ) ctxTemp = SubCtx( Ctx::DeltaQP, m_CABACEstimator->getCtx() );

    m_CABACEstimator->resetBits();
    m_CABACEstimator->cu_qp_delta( *cuFirst, predQP, cuFirst->qp );

    cs.fracBits += m_CABACEstimator->getEstFracBits(); // dQP bits
    cs.cost      = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);


    if( !bKeepCtx ) m_CABACEstimator->getCtx() = SubCtx( Ctx::DeltaQP, ctxTemp );

    // NOTE: reset QPs for CUs without residuals up to first coded CU
    for( const auto &cu : cs.cus )
    {
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if ( cu->rootCbf )
#else
      //not include the chroma CU because chroma CU is decided based on corresponding luma QP and deltaQP is not signaled at chroma CU
      if( cu->rootCbf && !isChroma( cu->chType ))
#endif
      {
        break;
      }
      cu->qp = predQP;
    }
  }
  else
  {
    // No residuals: reset CU QP to predicted value
    for( const auto &cu : cs.cus )
    {
      cu->qp = predQP;
    }
  }
}

void EncCu::xCheckChromaQPOffset( CodingStructure& cs, Partitioner& partitioner )
{
  // doesn't apply if CU chroma QP offset is disabled
  if( !cs.slice->getUseChromaQpAdj() )
  {
    return;
  }

  // doesn't apply to luma CUs
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (CS::isDualITree(cs) && isLuma(partitioner.chType))
#else
  if( partitioner.isSepTree(cs) && isLuma(partitioner.chType) )
#endif
  {
    return;
  }

  // not needed after the first coded TU in the chroma QG
  if( !partitioner.currQgChromaEnable() )
  {
    return;
  }

  CodingUnit& cu = *cs.getCU( partitioner.chType );

  // check if chroma is coded or not
  bool hasResidual = false;
  for( const TransformUnit &tu : CU::traverseTUs(cu) )
  {
    if( tu.cbf[COMPONENT_Cb] || tu.cbf[COMPONENT_Cr] )
    {
      hasResidual = true;
      break;
    }
  }

  if( hasResidual )
  {
    // estimate cost for coding cu_chroma_qp_offset
    TempCtx ctxTempAdjFlag( m_ctxCache );
    TempCtx ctxTempAdjIdc( m_ctxCache );
    ctxTempAdjFlag = SubCtx( Ctx::ChromaQpAdjFlag, m_CABACEstimator->getCtx() );
    ctxTempAdjIdc = SubCtx( Ctx::ChromaQpAdjIdc,   m_CABACEstimator->getCtx() );
    m_CABACEstimator->resetBits();
    m_CABACEstimator->cu_chroma_qp_offset( cu );
    cs.fracBits += m_CABACEstimator->getEstFracBits();
    cs.cost      = m_pcRdCost->calcRdCost(cs.fracBits, cs.dist);
    m_CABACEstimator->getCtx() = SubCtx( Ctx::ChromaQpAdjFlag, ctxTempAdjFlag );
    m_CABACEstimator->getCtx() = SubCtx( Ctx::ChromaQpAdjIdc,  ctxTempAdjIdc  );
  }
  else
  {
    // reset chroma QP offset to 0 if it will not be coded
    cu.chromaQpAdj = 0;
  }
}

#if!REMOVE_PCM
void EncCu::xFillPCMBuffer( CodingUnit &cu )
{
  const ChromaFormat format        = cu.chromaFormat;
  const uint32_t numberValidComponents = getNumberValidComponents(format);

  for( auto &tu : CU::traverseTUs( cu ) )
  {
    for( uint32_t ch = 0; ch < numberValidComponents; ch++ )
    {
      const ComponentID compID = ComponentID( ch );
      const CompArea &compArea = tu.blocks[ compID ];

      if( tu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag() && compID == COMPONENT_Y )
      {
        tu.getPcmbuf( compID ).rspSignal( tu.cs->getOrgBuf( compArea ), m_pcReshape->getFwdLUT() );
      }
      else
      {
        tu.getPcmbuf( compID ).copyFrom( tu.cs->getOrgBuf( compArea ) );
      }
    }
  }
}
#endif

#if ENABLE_OBMC && JVET_AA0129_INTERHASH_OBMCOFF_RD
bool EncCu::xCheckRDCostHashInter( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
#else
void EncCu::xCheckRDCostHashInter( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
#endif
{
#if ENABLE_OBMC
  double bestOBMCCost = MAX_DOUBLE;
#if ENABLE_OBMC && JVET_AA0129_INTERHASH_OBMCOFF_RD
  bool   validMode = false;
#endif
#endif
  bool isPerfectMatch = false;

  tempCS->initStructData(encTestMode.qp);
  m_pcInterSearch->resetBufferedUniMotions();
  m_pcInterSearch->setAffineModeSelected(false);
#if JVET_AD0213_LIC_IMP
  m_pcInterSearch->setDoAffineLic(true);
#endif
  CodingUnit &cu = tempCS->addCU(tempCS->area, partitioner.chType);

  partitioner.setCUData(cu);
  cu.slice = tempCS->slice;
  cu.tileIdx = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
  cu.skip = false;
  cu.predMode = MODE_INTER;
  cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
  cu.qp = encTestMode.qp;
#if INTER_LIC
  cu.licFlag = false;
#endif
  CU::addPUs(cu);
  cu.mmvdSkip = false;
  cu.firstPU->mmvdMergeFlag = false;

  if (m_pcInterSearch->predInterHashSearch(cu, partitioner, isPerfectMatch))
  {
    double equBcwCost = MAX_DOUBLE;

    m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;
#if ENABLE_OBMC //normal inter
  const unsigned wIdx = gp_sizeIdxInfo->idxFrom(partitioner.currArea().lwidth());
  CodingStructure *prevCS = tempCS;
  PelUnitBuf tempWoOBMCBuf = m_tempWoOBMCBuffer.subBuf(UnitAreaRelative(cu, cu));
  tempWoOBMCBuf.copyFrom(tempCS->getPredBuf(cu));
  cu.isobmcMC = true;
#if JVET_AC0158_PIXEL_AFFINE_MC
#if JVET_AD0213_LIC_IMP
  cu.obmcFlag = (cu.lwidth() * cu.lheight() < 32) || (tempCS->sps->getUseOBMC() == false) ? false : true;
#else
  cu.obmcFlag = (cu.lwidth() * cu.lheight() < 32) || (cu.licFlag == true || tempCS->sps->getUseOBMC() == false) ? false : true;
#endif
#else
  cu.obmcFlag = true;
#endif
  m_pcInterSearch->subBlockOBMC(*cu.firstPU);
  cu.isobmcMC = false;
#endif
    xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, 0
      , 0
      , &equBcwCost
    );
#if ENABLE_OBMC // xCheckRDCostInter
  double tempCost = (prevCS == tempCS) ? tempCS->cost : bestCS->cost;
#if JVET_AC0158_PIXEL_AFFINE_MC
  if (m_pTempCUWoOBMC && tempCost < bestOBMCCost && tempCS->sps->getUseOBMC() == true)
#else
  if (m_pTempCUWoOBMC && tempCost < bestOBMCCost)
#endif
  {
    const unsigned hIdx = gp_sizeIdxInfo->idxFrom(prevCS->area.lheight());
    m_pTempCUWoOBMC[wIdx][hIdx]->clearCUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->clearPUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->clearTUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->copyStructure(*prevCS, partitioner.chType);

    m_pPredBufWoOBMC[wIdx][hIdx].copyFrom(tempWoOBMCBuf);
    m_pTempCUWoOBMC[wIdx][hIdx]->getPredBuf(cu).copyFrom(prevCS->getPredBuf(cu));

    bestOBMCCost = tempCost;
#if ENABLE_OBMC && JVET_AA0129_INTERHASH_OBMCOFF_RD
    validMode = true;
#endif
  }
#endif
    if ( m_bestModeUpdated && bestCS->cost != MAX_DOUBLE )
    {
      xCalDebCost( *bestCS, partitioner );
    }
  }
  tempCS->initStructData(encTestMode.qp);
  int minSize = min(cu.lwidth(), cu.lheight());
  if (minSize < 64)
  {
    isPerfectMatch = false;
  }
  m_modeCtrl->setIsHashPerfectMatch(isPerfectMatch);
#if ENABLE_OBMC && JVET_AA0129_INTERHASH_OBMCOFF_RD
  return validMode;
#endif
}

void EncCu::xCheckRDCostMerge2Nx2N( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  const Slice &slice = *tempCS->slice;

  CHECK( slice.getSliceType() == I_SLICE, "Merge modes not available for I-slices" );

  tempCS->initStructData( encTestMode.qp );

  MergeCtx mergeCtx;
#if JVET_W0090_ARMC_TM
  MergeCtx mergeCtxtmp;
#if JVET_AG0135_AFFINE_CIIP
  MergeCtx mergeCtxCIIPtmp;
#endif
#endif
#if JVET_AG0276_NLIC
  MergeCtx mergeOrgCtx;
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
  MergeCtx mergeCtxOppositeLic;
  mergeCtxOppositeLic.numValidMergeCand = 0;
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  uint32_t               mmvdLUT[MMVD_ADD_NUM];
#endif
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  uint8_t numBaseAffine = AF_MMVD_BASE_NUM;
  unsigned ctxId = 0;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  if (tempCS->sps->getUseTMMMVD())
#endif
  {
    CodingUnit cu( tempCS->area );
    cu.cs       = tempCS;
    cu.predMode = MODE_INTER;
    cu.slice    = tempCS->slice;
    cu.tileIdx  = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
    const CodingUnit*  cuLeft  = tempCS->getCURestricted(cu.lumaPos().offset(-1, 0), cu, partitioner.chType);
    ctxId = (cuLeft && cuLeft->affine) ? 1 : 0;
    const CodingUnit*  cuAbove = tempCS->getCURestricted(cu.lumaPos().offset( 0,-1), cu, partitioner.chType);
    ctxId += (cuAbove && cuAbove->affine) ? 1 : 0;
  }
  numBaseAffine = (ctxId == 0) ? 1 : ((ctxId == 1) ? 2 : AF_MMVD_BASE_NUM);
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  uint32_t               affMmvdLUT[AF_MMVD_NUM];
#endif
  const SPS &sps = *tempCS->sps;

#if MERGE_ENC_OPT
  const bool affineMrgAvail = (sps.getUseAffine() || sps.getSbTMVPEnabledFlag()) && slice.getPicHeader()->getMaxNumAffineMergeCand()
    && !(bestCS->area.lumaSize().width < 8 || bestCS->area.lumaSize().height < 8);

  AffineMergeCtx affineMergeCtx;
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
  AffineMergeCtx affineBMMergeCtx;
  AffineMergeCtx affineBMMergeL0;
  AffineMergeCtx affineBMMergeL1;
  AffineMergeCtx affineRMVFCtx;
  AffineMergeCtx affineRMVFOriCtx;
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
  AffineMergeCtx affineMergeCtxOppositeLic;
  affineMergeCtxOppositeLic.numValidMergeCand = 0;
#endif
#if JVET_W0090_ARMC_TM
  AffineMergeCtx affineMergeCtxTmp;
#endif
  MergeCtx mrgCtx;
#if TM_MRG
  MergeCtx tmMrgCtx;
#if JVET_AG0276_LIC_FLAG_SIGNALING
  MergeCtx tmMrgCtxOppositeLic;
  tmMrgCtxOppositeLic.numValidMergeCand = 0;
#endif
#if JVET_X0141_CIIP_TIMD_TM
  MergeCtx ciipTmMrgCtx;
#endif
#endif
#if JVET_AB0079_TM_BCW_MRG
  MergeCtx mrgCtxCiip;
#endif

#if JVET_X0049_ADAPT_DMVR
  MergeCtx bmMrgCtx;
  bool checkBmMrg = false;
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  MergeCtx bmMrgCtxDir2;
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
  bool checkaffBmMrg = false;
#endif
#endif

  if (sps.getSbTMVPEnabledFlag())
  {
    Size bufSize = g_miScaling.scale(tempCS->area.lumaSize());
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
    for (int i = 0; i < SUB_TMVP_NUM; i++)
    {
      mergeCtx.subPuMvpMiBuf[i] = MotionBuf(m_subPuMiBuf[i], bufSize);
#if JVET_AG0276_NLIC
      mergeOrgCtx.subPuMvpMiBuf[i] = MotionBuf(m_subPuMiBuf[i], bufSize);
#endif
      mrgCtx.subPuMvpMiBuf[i] = MotionBuf(m_subPuMiBuf[i], bufSize);
#if JVET_AG0276_LIC_FLAG_SIGNALING
      mergeCtxOppositeLic.subPuMvpMiBuf[i] = MotionBuf(m_subPuMiBuf[i], bufSize);
#endif
    }
#else
    mergeCtx.subPuMvpMiBuf = MotionBuf(m_subPuMiBuf, bufSize);
    mrgCtx.subPuMvpMiBuf = MotionBuf(m_subPuMiBuf, bufSize);
#endif
    affineMergeCtx.mrgCtx = &mrgCtx;
#if TM_MRG
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
    for (int i = 0; i < SUB_TMVP_NUM; i++)
    {
      tmMrgCtx.subPuMvpMiBuf[i] = MotionBuf(m_subPuMiBuf[i], bufSize);
#if JVET_AG0276_LIC_FLAG_SIGNALING
      tmMrgCtxOppositeLic.subPuMvpMiBuf[i] = MotionBuf(m_subPuMiBuf[i], bufSize);
#endif
#if JVET_X0141_CIIP_TIMD_TM
      ciipTmMrgCtx.subPuMvpMiBuf[i] = MotionBuf(m_subPuMiBuf[i], bufSize);
#endif
#if JVET_AB0079_TM_BCW_MRG
      mrgCtxCiip.subPuMvpMiBuf[i] = MotionBuf(m_subPuMiBuf[i], bufSize);
#endif
    }
#else
    tmMrgCtx.subPuMvpMiBuf = MotionBuf(m_subPuMiBuf, bufSize);
#if JVET_X0141_CIIP_TIMD_TM
    ciipTmMrgCtx.subPuMvpMiBuf = MotionBuf(m_subPuMiBuf, bufSize);
#endif
#if JVET_AB0079_TM_BCW_MRG
    mrgCtxCiip.subPuMvpMiBuf = MotionBuf(m_subPuMiBuf, bufSize);
#endif
#endif
#endif
  }
#endif

#if MULTI_PASS_DMVR
  bool applyBDMVR[MRG_MAX_NUM_CANDS] = { false };
#if JVET_AG0276_LIC_FLAG_SIGNALING
  bool applyBDMVROppositeLic[MRG_MAX_NUM_CANDS] = { false };
#endif
#if JVET_AF0057
  bool dmvrImpreciseMv[MRG_MAX_NUM_CANDS] = { false };
#endif
#if TM_MRG && MERGE_ENC_OPT
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  bool applyBDMVR4TM[TM_MRG_MAX_NUM_INIT_CANDS] = { false };
#if JVET_AG0276_LIC_FLAG_SIGNALING
  bool applyBDMVR4TMOppositeLic[TM_MRG_MAX_NUM_INIT_CANDS] = { false };
#endif
#else
  bool applyBDMVR4TM[TM_MRG_MAX_NUM_CANDS] = { false };
#endif
#if JVET_AF0057
  bool dmvr4TMImpreciseMv = false;
#endif
#endif
#if JVET_X0049_ADAPT_DMVR
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  bool admvrRefinedMotion = false;
  bool applyBDMVR4BM[(BM_MRG_MAX_NUM_INIT_CANDS << 1)<<1] = { false };
#else
  bool applyBDMVR4BM[(BM_MRG_MAX_NUM_CANDS << 1)<<1] = { false };
#endif
#endif
#endif
#if !MULTI_PASS_DMVR
  Mv   refinedMvdL0[MAX_NUM_PARTS_IN_CTU][MRG_MAX_NUM_CANDS];
#endif
  setMergeBestSATDCost( MAX_DOUBLE );

#if JVET_AG0276_LIC_FLAG_SIGNALING
  bool hasOppositelicMrg = false;
  bool hasOppositelicAff = false;
#endif
  {
    // first get merge candidates
    CodingUnit cu( tempCS->area );
    cu.cs       = tempCS;
    cu.predMode = MODE_INTER;
    cu.slice    = tempCS->slice;
    cu.tileIdx  = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
#if INTER_LIC
    cu.licFlag  = false;
#endif

    PredictionUnit pu( tempCS->area );
    pu.cu = &cu;
    pu.cs = tempCS;
#if JVET_AG0276_LIC_FLAG_SIGNALING
    hasOppositelicMrg = PU::hasOppositeLICFlag(pu);
    pu.cu->affine = true;
    hasOppositelicAff = PU::hasOppositeLICFlag(pu);
    pu.cu->affine = false;
#endif
#if JVET_AF0057
    bool enableVisualCheck = false;
    if (((m_pcEncCfg->getFrameRate() <= DMVR_ENC_SELECT_FRAME_RATE_THR) || !(m_pcEncCfg->getDMVREncMvSelectDisableHighestTemporalLayer() && (pu.cu->slice->getTLayer() == (pu.cu->slice->getSPS()->getMaxTLayers() - 1))))
      && m_pcEncCfg->getDMVREncMvSelection()
      && (pu.lwidth() >= DMVR_ENC_SELECT_SIZE_THR && pu.lheight() >= DMVR_ENC_SELECT_SIZE_THR))
    {
      enableVisualCheck = true; // only set this to true when cfg, size, tid, framerate all fulfilled
    }
    m_pcInterSearch->xDmvrSetEncoderCheckFlag(enableVisualCheck);
#endif
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
    pu.tmMergeFlag = false;
#endif
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC && !JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
    int nWidth = pu.lumaSize().width;
    int nHeight = pu.lumaSize().height;
    bool tplAvail = m_pcInterSearch->xAMLGetCurBlkTemplate(pu, nWidth, nHeight);
#endif
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
    int nWidth = pu.lumaSize().width;
    int nHeight = pu.lumaSize().height;
    bool tplAvail = m_pcInterSearch->xAMLGetCurBlkTemplate(pu, nWidth, nHeight);

    MergeCtx tmvpMergeCandCtx;
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
    if (sps.getUseAML() && tplAvail
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
      )
    {
      PU::getTmvpMergeCand(pu, tmvpMergeCandCtx);
    }
#else
    if (sps.getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
      )
    {
      PU::getTmvpMergeCand(pu, tmvpMergeCandCtx);
      if (tplAvail)
      {
        m_pcInterSearch->adjustMergeCandidatesInOneCandidateGroup(pu, tmvpMergeCandCtx, 1);
      }
      else
      {
        tmvpMergeCandCtx.numValidMergeCand = std::min(1, tmvpMergeCandCtx.numValidMergeCand);
      }
    }
#endif
    MergeCtx namvpMergeCandCtx;
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
    if (sps.getUseAML() && tplAvail
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
      )
    {
      PU::getNonAdjacentMergeCand(pu, namvpMergeCandCtx);
    }
#else    
    if (sps.getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
      )
    {
      PU::getNonAdjacentMergeCand(pu, namvpMergeCandCtx);
      if (tplAvail)
      {
        m_pcInterSearch->adjustMergeCandidatesInOneCandidateGroup(pu, namvpMergeCandCtx, 9);
      }
      else
      {
        namvpMergeCandCtx.numValidMergeCand = std::min(9, namvpMergeCandCtx.numValidMergeCand);
      }
    }
#endif
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
    if (!tplAvail)
    {
      PU::getInterMergeCandidates(pu, mergeCtx, 0, -1);
      mergeCtx.numValidMergeCand = pu.cs->sps->getMaxNumMergeCand();
    }
    else
#endif
#endif

    PU::getInterMergeCandidates(pu, mergeCtx
      , 0
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
      , -1
      , (sps.getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
        ) ? &tmvpMergeCandCtx : NULL
      , (sps.getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
        ) ? &namvpMergeCandCtx : NULL
#endif
    );
#if JVET_AF0128_LIC_MERGE_TM
    if (pu.cs->sps->getUseAML()
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
      && pu.cs->sps->getTMToolsEnableFlag()
#endif
      )
    {
      m_pcInterSearch->adjustMergeCandidatesLicFlag(pu, mergeCtx);
    }
#endif
#if JVET_AG0276_NLIC
    mergeOrgCtx = mergeCtx;
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
    if (hasOppositelicMrg && pu.cs->sps->getUseMergeOppositeLic())
    {
      mergeCtxOppositeLic = mergeCtx;
      for (int i = 0; i < mergeCtx.numValidMergeCand; i++)
      {
        mergeCtxOppositeLic.licFlags[i] = !mergeCtx.licFlags[i];
      }
    }
#endif
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
    if (sps.getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
      )
    {
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
      if (!sps.getUseTmvpNmvpReordering())
      {
        m_pcInterSearch->adjustInterMergeCandidates(pu, mergeCtx);
#if JVET_AG0276_NLIC
        mergeOrgCtx = mergeCtx;
#endif
      }
      else
#endif
      if (tplAvail)
      {
#if JVET_Z0102_NO_ARMC_FOR_ZERO_CAND 
        m_pcInterSearch->adjustMergeCandidates(pu, mergeCtx, pu.cs->sps->getMaxNumMergeCand());
#else
        m_pcInterSearch->adjustMergeCandidatesInOneCandidateGroup(pu, mergeCtx, pu.cs->sps->getMaxNumMergeCand());
#endif
#if JVET_AG0276_NLIC
        if (pu.cs->sps->getUseAltLM() && !CU::isTLCond(*pu.cu))
        {
          AltLMMergeCtx altLMMrgCtx;
          PU::getAltMergeCandidates(pu, altLMMrgCtx);
          m_pcInterSearch->adjustMergeCandidates(pu, mergeOrgCtx, altLMMrgCtx, pu.cs->sps->getMaxNumMergeCand());
#if JVET_AG0276_LIC_FLAG_SIGNALING
          if (hasOppositelicMrg && pu.cs->sps->getUseMergeOppositeLic())
          {
            AltLMMergeCtx altLMBRMrgCtx;
            PU::getAltBRMergeCandidates(pu, altLMBRMrgCtx);
            m_pcInterSearch->adjustMergeCandidates(pu, mergeCtxOppositeLic, altLMBRMrgCtx, pu.cs->sps->getMaxNumMergeCand());
          }
#endif
        }
        else
        {
          mergeOrgCtx = mergeCtx;
#if JVET_AG0276_LIC_FLAG_SIGNALING
          if (hasOppositelicMrg && pu.cs->sps->getUseMergeOppositeLic())
          {
            m_pcInterSearch->adjustMergeCandidates(pu, mergeCtxOppositeLic, pu.cs->sps->getMaxNumMergeCand());
          }
#endif
        }
#elif JVET_AG0276_LIC_FLAG_SIGNALING
        if (hasOppositelicMrg && pu.cs->sps->getUseMergeOppositeLic())
        {
          m_pcInterSearch->adjustMergeCandidates(pu, mergeCtxOppositeLic, pu.cs->sps->getMaxNumMergeCand());
        }
#endif
      }
    }
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
    if (mergeCtx.numValidMergeCand != pu.cs->sps->getMaxNumMergeCand())
    {
      mergeCtx.numValidMergeCand = pu.cs->sps->getMaxNumMergeCand();
    }
#if JVET_AG0276_LIC_FLAG_SIGNALING
    if (hasOppositelicMrg && pu.cs->sps->getUseMergeOppositeLic())
    {
      if (mergeCtxOppositeLic.numValidMergeCand > pu.cs->sps->getMaxNumOppositeLicMergeCand())
      {
        mergeCtxOppositeLic.numValidMergeCand = pu.cs->sps->getMaxNumOppositeLicMergeCand();
      }
    }
#endif
    for (uint32_t ui = mergeCtx.numValidMergeCand; ui < NUM_MERGE_CANDS; ++ui)
    {
      mergeCtx.bcwIdx[ui] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
      mergeCtx.altLMFlag[ui] = false;
      mergeCtx.altLMParaNeighbours[ui].resetAltLinearModel();
#endif
#if INTER_LIC
      mergeCtx.licFlags[ui] = false;
#endif
      mergeCtx.interDirNeighbours[ui]                  = 0;
      mergeCtx.mvFieldNeighbours[(ui << 1)].refIdx     = NOT_VALID;
      mergeCtx.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
      mergeCtx.useAltHpelIf[ui]                        = false;
#if MULTI_HYP_PRED
      mergeCtx.addHypNeighbours[ui].clear();
#endif
      mergeCtx.candCost[ui] = MAX_UINT64;
#if JVET_AG0276_LIC_FLAG_SIGNALING
      mergeCtxOppositeLic.bcwIdx[ui] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
      mergeCtxOppositeLic.altLMFlag[ui] = false;
      mergeCtxOppositeLic.altLMParaNeighbours[ui].resetAltLinearModel();
#endif
#if INTER_LIC
      mergeCtxOppositeLic.licFlags[ui] = false;
#endif
      mergeCtxOppositeLic.interDirNeighbours[ui] = 0;
      mergeCtxOppositeLic.mvFieldNeighbours[(ui << 1)].refIdx = NOT_VALID;
      mergeCtxOppositeLic.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
      mergeCtxOppositeLic.useAltHpelIf[ui] = false;
#if MULTI_HYP_PRED
      mergeCtxOppositeLic.addHypNeighbours[ui].clear();
#endif
      mergeCtxOppositeLic.candCost[ui] = MAX_UINT64;
#endif
    }
#endif

#if JVET_AG0276_NLIC
    if (mergeOrgCtx.numValidMergeCand != pu.cs->sps->getMaxNumMergeCand())
    {
      mergeOrgCtx.numValidMergeCand = pu.cs->sps->getMaxNumMergeCand();
    }
    for (uint32_t ui = mergeOrgCtx.numValidMergeCand; ui < NUM_MERGE_CANDS; ++ui)
    {
      mergeOrgCtx.bcwIdx[ui] = BCW_DEFAULT;
      mergeOrgCtx.altLMFlag[ui] = false;
      mergeOrgCtx.altLMParaNeighbours[ui].resetAltLinearModel();
#if INTER_LIC
      mergeOrgCtx.licFlags[ui] = false;
#endif
      mergeOrgCtx.interDirNeighbours[ui] = 0;
      mergeOrgCtx.mvFieldNeighbours[(ui << 1)].refIdx = NOT_VALID;
      mergeOrgCtx.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
      mergeOrgCtx.useAltHpelIf[ui] = false;
#if MULTI_HYP_PRED
      mergeOrgCtx.addHypNeighbours[ui].clear();
#endif
      mergeOrgCtx.candCost[ui] = MAX_UINT64;
    }
#endif

#if JVET_AB0079_TM_BCW_MRG
    mrgCtxCiip.numValidMergeCand = mergeCtx.numValidMergeCand;
#if JVET_Z0102_NO_ARMC_FOR_ZERO_CAND
    mrgCtxCiip.numCandToTestEnc = mergeCtx.numCandToTestEnc;
#endif
    for (uint32_t uiMergeCand = 0; uiMergeCand < mergeCtx.numValidMergeCand; uiMergeCand++)
    {
      mrgCtxCiip.bcwIdx[uiMergeCand] = mergeCtx.bcwIdx[uiMergeCand];
      mrgCtxCiip.interDirNeighbours[uiMergeCand] = mergeCtx.interDirNeighbours[uiMergeCand];
      mrgCtxCiip.mvFieldNeighbours[(uiMergeCand << 1)] = mergeCtx.mvFieldNeighbours[(uiMergeCand << 1)];
      mrgCtxCiip.mvFieldNeighbours[(uiMergeCand << 1) + 1] = mergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1];
      mrgCtxCiip.useAltHpelIf[uiMergeCand] = mergeCtx.useAltHpelIf[uiMergeCand];
#if JVET_AG0276_NLIC
      mrgCtxCiip.altLMFlag[uiMergeCand] = mergeCtx.altLMFlag[uiMergeCand];
      mrgCtxCiip.altLMParaNeighbours[uiMergeCand] = mergeCtx.altLMParaNeighbours[uiMergeCand];
#endif
#if INTER_LIC 
      mrgCtxCiip.licFlags[uiMergeCand] = mergeCtx.licFlags[uiMergeCand];
#endif
#if MULTI_HYP_PRED
      mrgCtxCiip.addHypNeighbours[uiMergeCand] = mergeCtx.addHypNeighbours[uiMergeCand];
#endif
      mrgCtxCiip.candCost[uiMergeCand] = mergeCtx.candCost[uiMergeCand];
    }
    
    if (sps.getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
      )
    {
      m_pcInterSearch->adjustMergeCandidatesBcwIdx(pu, mergeCtx);
#if JVET_AG0276_NLIC
      m_pcInterSearch->adjustMergeCandidatesBcwIdx(pu, mergeOrgCtx);
      MergeCtx mergeCtxTemp;
      mergeCtxTemp = mergeCtx;
      mergeCtx = mergeOrgCtx;
      mergeOrgCtx = mergeCtxTemp;
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
      if (hasOppositelicMrg && pu.cs->sps->getUseMergeOppositeLic())
      {
        m_pcInterSearch->adjustMergeCandidatesBcwIdx(pu, mergeCtxOppositeLic);
      }
#endif
    }
#endif
    PU::getInterMergeCandidates(pu, mergeCtxtmp, 0);
#if JVET_AG0135_AFFINE_CIIP
    pu.tmMergeFlag = true;
    PU::getInterMergeCandidates(pu, mergeCtxCIIPtmp, 0);
    pu.tmMergeFlag = false;
#endif
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
    MergeCtx mrgCtxAll[2];
    for (int i = 0; i < 2; i++)
    {
      mrgCtxAll[i].numValidMergeCand = 0;
    }
    if (pu.cs->picHeader->getEnableTMVPFlag())
    {
      if (pu.cs->sps->getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
          && pu.cs->sps->getTMToolsEnableFlag()
#endif
        )
      {
        MergeCtx namvpMergeCandCtxSubTMVP[2];
        int nWidth = pu.lumaSize().width;
        int nHeight = pu.lumaSize().height;
        bool tplAvail = m_pcInterSearch->xAMLGetCurBlkTemplate(pu, nWidth, nHeight);
        int poc0 = slice.getRefPic(RefPicList(1 - slice.getColFromL0Flag()), slice.getColRefIdx())->getPOC();
        int poc1 = slice.getRefPic(RefPicList(1 - slice.getColFromL0Flag2nd()), slice.getColRefIdx2nd())->getPOC();
        for (int col = 0; col < ((pu.cu->slice->getCheckLDC() || (poc0 == poc1)) ? 1 : 2); col++)
        {
          PU::getNonAdjacentMergeCandSubTMVP(pu, namvpMergeCandCtxSubTMVP[col], col);
          if (col == 0 && tplAvail)
          {
            m_pcInterSearch->adjustMergeCandidatesInOneCandidateGroupSubTMVP(pu, namvpMergeCandCtxSubTMVP[col], 9);
          }
          else
          {
            namvpMergeCandCtxSubTMVP[col].numValidMergeCand = std::min(9, namvpMergeCandCtxSubTMVP[col].numValidMergeCand);
          }
          PU::getInterMergeCandidatesSubTMVP(pu, mrgCtxAll[col], col, -1, &namvpMergeCandCtxSubTMVP[col]);

          if (col == 0 && tplAvail)
          {
            m_pcInterSearch->adjustMergeCandidatesInOneCandidateGroupSubTMVP(pu, mrgCtxAll[col], pu.cs->sps->getMaxNumMergeCand());
          }
        }
      }
    }
#endif
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
    if (cu.cs->sps->getUseCiipTmMrg())
    {
      pu.tmMergeFlag = true;
      pu.ciipFlag = true;
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
#if JVET_AG0135_AFFINE_CIIP
      ciipTmMrgCtx = mergeCtxCIIPtmp;
      ciipTmMrgCtx.numValidMergeCand = int(pu.cs->sps->getMaxNumCiipTMMergeCand());
#if !(JVET_AG0276_LIC_FLAG_SIGNALING || JVET_AG0276_LIC_BDOF_BDMVR || JVET_AG0276_NLIC)
      memcpy(ciipTmMrgCtx.bcwIdx, mergeCtxCIIPtmp.bcwIdx, CIIP_TM_MRG_MAX_NUM_CANDS * sizeof(uint8_t));
      memcpy(ciipTmMrgCtx.interDirNeighbours, mergeCtxCIIPtmp.interDirNeighbours, CIIP_TM_MRG_MAX_NUM_CANDS * sizeof(unsigned char));
      memcpy(ciipTmMrgCtx.mvFieldNeighbours, mergeCtxCIIPtmp.mvFieldNeighbours, (CIIP_TM_MRG_MAX_NUM_CANDS << 1) * sizeof(MvField));
      memcpy(ciipTmMrgCtx.useAltHpelIf, mergeCtxCIIPtmp.useAltHpelIf, CIIP_TM_MRG_MAX_NUM_CANDS * sizeof(bool));
#if INTER_LIC
      memcpy(ciipTmMrgCtx.licFlags, mergeCtxCIIPtmp.licFlags, CIIP_TM_MRG_MAX_NUM_CANDS * sizeof(bool));
#endif
#if MULTI_HYP_PRED
      memcpy(ciipTmMrgCtx.addHypNeighbours, mergeCtxCIIPtmp.addHypNeighbours, CIIP_TM_MRG_MAX_NUM_CANDS * sizeof(MultiHypVec));
#endif
#endif
#else
      ciipTmMrgCtx = mergeCtxtmp;
      ciipTmMrgCtx.numValidMergeCand = int(pu.cs->sps->getMaxNumCiipTMMergeCand());
#if !(JVET_AG0276_LIC_FLAG_SIGNALING || JVET_AG0276_LIC_BDOF_BDMVR || JVET_AG0276_NLIC)
      memcpy(ciipTmMrgCtx.bcwIdx, mergeCtxtmp.bcwIdx, CIIP_TM_MRG_MAX_NUM_CANDS * sizeof(uint8_t));
      memcpy(ciipTmMrgCtx.interDirNeighbours, mergeCtxtmp.interDirNeighbours, CIIP_TM_MRG_MAX_NUM_CANDS * sizeof(unsigned char));
      memcpy(ciipTmMrgCtx.mvFieldNeighbours, mergeCtxtmp.mvFieldNeighbours, (CIIP_TM_MRG_MAX_NUM_CANDS << 1) * sizeof(MvField));
      memcpy(ciipTmMrgCtx.useAltHpelIf, mergeCtxtmp.useAltHpelIf, CIIP_TM_MRG_MAX_NUM_CANDS * sizeof(bool));
#if JVET_AG0276_NLIC
      memcpy(ciipTmMrgCtx.altLMFlag, mergeCtxtmp.altLMFlag, CIIP_TM_MRG_MAX_NUM_CANDS * sizeof(bool));
      memcpy(ciipTmMrgCtx.altLMParaNeighbours, mergeCtxtmp.altLMParaNeighbours, CIIP_TM_MRG_MAX_NUM_CANDS * sizeof(AltLMInterUnit));
#endif
#if INTER_LIC
      memcpy(ciipTmMrgCtx.licFlags, mergeCtxtmp.licFlags, CIIP_TM_MRG_MAX_NUM_CANDS * sizeof(bool));
#endif
#if MULTI_HYP_PRED
      memcpy(ciipTmMrgCtx.addHypNeighbours, mergeCtxtmp.addHypNeighbours, CIIP_TM_MRG_MAX_NUM_CANDS * sizeof(MultiHypVec));
#endif
#endif
#endif
#else
      ciipTmMrgCtx = mergeCtx;
      ciipTmMrgCtx.numValidMergeCand = int(pu.cs->sps->getMaxNumCiipTMMergeCand());
#if !(JVET_AG0276_LIC_FLAG_SIGNALING || JVET_AG0276_LIC_BDOF_BDMVR || JVET_AG0276_NLIC)
      memcpy(ciipTmMrgCtx.bcwIdx, mergeCtx.bcwIdx, CIIP_TM_MRG_MAX_NUM_CANDS * sizeof(uint8_t));
      memcpy(ciipTmMrgCtx.interDirNeighbours, mergeCtx.interDirNeighbours, CIIP_TM_MRG_MAX_NUM_CANDS * sizeof(unsigned char));
      memcpy(ciipTmMrgCtx.mvFieldNeighbours, mergeCtx.mvFieldNeighbours, (CIIP_TM_MRG_MAX_NUM_CANDS << 1) * sizeof(MvField));
      memcpy(ciipTmMrgCtx.useAltHpelIf, mergeCtx.useAltHpelIf, CIIP_TM_MRG_MAX_NUM_CANDS * sizeof(bool));
#if INTER_LIC
      memcpy(ciipTmMrgCtx.licFlags, mergeCtx.licFlags, CIIP_TM_MRG_MAX_NUM_CANDS * sizeof(bool));
#endif
#if MULTI_HYP_PRED
      memcpy(ciipTmMrgCtx.addHypNeighbours, mergeCtx.addHypNeighbours, CIIP_TM_MRG_MAX_NUM_CANDS * sizeof(MultiHypVec));
#endif
#endif
#endif

      for (uint32_t uiMergeCand = 0; uiMergeCand < ciipTmMrgCtx.numValidMergeCand; uiMergeCand++)
      {
        ciipTmMrgCtx.setMergeInfo(pu, uiMergeCand);
        m_pcInterSearch->deriveTMMv(pu);

        // Store refined motion back to ciipTmMrgCtx
        ciipTmMrgCtx.interDirNeighbours[uiMergeCand] = pu.interDir;
        ciipTmMrgCtx.bcwIdx[uiMergeCand] = pu.cu->bcwIdx;  // BCW may change, because bi may be reduced to uni by deriveTMMv(pu)
        ciipTmMrgCtx.mvFieldNeighbours[2 * uiMergeCand].setMvField(pu.mv[0], pu.refIdx[0]);
        ciipTmMrgCtx.mvFieldNeighbours[2 * uiMergeCand + 1].setMvField(pu.mv[1], pu.refIdx[1]);
        if (pu.interDir == 1)
        {
          ciipTmMrgCtx.mvFieldNeighbours[2 * uiMergeCand + 1].setMvField(Mv(), NOT_VALID);
        }
        if (pu.interDir == 2)
        {
          ciipTmMrgCtx.mvFieldNeighbours[2 * uiMergeCand].setMvField(Mv(), NOT_VALID);
        }
      }
#if JVET_W0090_ARMC_TM
      if (sps.getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
        )
      {
        m_pcInterSearch->adjustInterMergeCandidates(pu, ciipTmMrgCtx);
      }
#endif
      pu.tmMergeFlag = false;
      pu.ciipFlag = false;
    }
#endif
#if JVET_W0097_GPM_MMVD_TM
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
    m_mergeCand.copyMergeCtx(mergeCtxtmp);
#else
    m_mergeCand.copyMergeCtx(mergeCtx);
#endif
    m_mergeCandAvail = true;
#endif
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
    PU::getInterMMVDMergeCandidates(pu, mergeCtxtmp);
#else
    PU::getInterMMVDMergeCandidates(pu, mergeCtx);
#if JVET_W0090_ARMC_TM
    mergeCtxtmp = mergeCtx;
    if (sps.getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
      )
    {
      m_pcInterSearch->adjustInterMergeCandidates(pu, mergeCtx);
    }
#endif
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    bool flag = pu.mmvdMergeFlag;
    pu.mmvdMergeFlag = true;
#if JVET_AF0128_LIC_MERGE_TM
    if (pu.cs->sps->getUseAML()
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
      && pu.cs->sps->getTMToolsEnableFlag()
#endif
      )
    {
      m_pcInterSearch->adjustMergeCandidatesLicFlag(pu, mergeCtxtmp);
    }
#endif
#if JVET_AB0079_TM_BCW_MRG
    if (sps.getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
      )
    {
      m_pcInterSearch->adjustMergeCandidatesBcwIdx(pu, mergeCtxtmp);
    }
#endif
    m_pcInterSearch->sortInterMergeMMVDCandidates(pu, mergeCtxtmp, mmvdLUT);
    pu.mmvdMergeFlag = flag;
#endif
#if TM_MRG && MERGE_ENC_OPT
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    if (cu.cs->sps->getUseTMMrgMode())
#else
    if (cu.cs->sps->getUseDMVDMode())
#endif
    {
      cu.firstPU = &pu;
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC || JVET_AA0093_REFINED_MOTION_FOR_ARMC
      pu.tmMergeFlag = true;
#endif
      MergeCtx tmvpTmMergeCandCtx;
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
      if (sps.getUseAML() && tplAvail
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
        )
      {
        PU::getTmvpMergeCand(pu, tmvpTmMergeCandCtx);
      }
#else
      if (sps.getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
        )
      {
        PU::getTmvpMergeCand(pu, tmvpTmMergeCandCtx);
        if (tplAvail)
        {
          m_pcInterSearch->adjustMergeCandidatesInOneCandidateGroup(pu, tmvpTmMergeCandCtx, 1);
        }
        else
        {
          tmvpTmMergeCandCtx.numValidMergeCand = std::min(1, tmvpTmMergeCandCtx.numValidMergeCand);
        }
      }
#endif
      MergeCtx namvpTmMergeCandCtx;
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
      if (sps.getUseAML() && tplAvail
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
        )
      {
        PU::getNonAdjacentMergeCand(pu, namvpTmMergeCandCtx);
      }
#else
      if (sps.getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
        )
      {
        PU::getNonAdjacentMergeCand(pu, namvpTmMergeCandCtx);
        if (tplAvail)
        {
          m_pcInterSearch->adjustMergeCandidatesInOneCandidateGroup(pu, namvpTmMergeCandCtx, 9);
        }
        else
        {
          namvpTmMergeCandCtx.numValidMergeCand = std::min(9, namvpTmMergeCandCtx.numValidMergeCand);
        }
      }
#endif
#endif
      pu.tmMergeFlag = true;
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC && JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
      if (!tplAvail)
      {
        PU::getInterMergeCandidates(pu, tmMrgCtx, 0, -1);
        tmMrgCtx.numValidMergeCand = pu.cs->sps->getMaxNumTMMergeCand();
      }
      else
#endif
      PU::getInterMergeCandidates(pu, tmMrgCtx, 0
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
        , -1
        , (sps.getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
          ) ? &tmvpTmMergeCandCtx : NULL
        , (sps.getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
          ) ? &namvpTmMergeCandCtx : NULL
#endif
      );
#if JVET_AF0128_LIC_MERGE_TM
      if (pu.cs->sps->getUseAML()
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
        && pu.cs->sps->getTMToolsEnableFlag()
#endif
        )
      {
        m_pcInterSearch->adjustMergeCandidatesLicFlag(pu, tmMrgCtx);
      }
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
      if (hasOppositelicMrg && pu.cs->sps->getUseTMMergeOppositeLic())
      {
        tmMrgCtxOppositeLic = tmMrgCtx;
        for (int i = 0; i < tmMrgCtxOppositeLic.numValidMergeCand; i++)
        {
          tmMrgCtxOppositeLic.licFlags[i] = !tmMrgCtx.licFlags[i];
        }
      }
#endif
#if JVET_W0090_ARMC_TM
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
      bool tmMergeRefinedMotion = PU::isArmcRefinedMotionEnabled(pu, 2);
      tmMergeRefinedMotion &= tplAvail;
#endif
      if (sps.getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
        )
      {
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
        if (!sps.getUseTmvpNmvpReordering())
        {
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
          if (!tmMergeRefinedMotion)
#endif
          m_pcInterSearch->adjustInterMergeCandidates(pu, tmMrgCtx);
#if JVET_AB0079_TM_BCW_MRG
          m_pcInterSearch->adjustMergeCandidatesBcwIdx(pu, tmMrgCtx);
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
          if (hasOppositelicMrg && pu.cs->sps->getUseTMMergeOppositeLic())
          {
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
            if (!tmMergeRefinedMotion)
#endif
            {
              m_pcInterSearch->adjustInterMergeCandidates(pu, tmMrgCtxOppositeLic);
            }
#if JVET_AB0079_TM_BCW_MRG
            m_pcInterSearch->adjustMergeCandidatesBcwIdx(pu, tmMrgCtxOppositeLic);
#endif
          }
#endif
        }
        else
#endif
        if (tplAvail)
        {
#if JVET_Z0102_NO_ARMC_FOR_ZERO_CAND 
          m_pcInterSearch->adjustMergeCandidates(pu, tmMrgCtx, pu.cs->sps->getMaxNumTMMergeCand());
#else
          m_pcInterSearch->adjustMergeCandidatesInOneCandidateGroup(pu, tmMrgCtx, pu.cs->sps->getMaxNumTMMergeCand());
#endif
#if JVET_AB0079_TM_BCW_MRG
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
          if (!tmMergeRefinedMotion)
#endif
          {
            if (tmMrgCtx.numValidMergeCand > pu.cs->sps->getMaxNumTMMergeCand())
            {
              tmMrgCtx.numValidMergeCand = pu.cs->sps->getMaxNumTMMergeCand();
            }
            m_pcInterSearch->adjustMergeCandidatesBcwIdx(pu, tmMrgCtx);
          }
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
          if (hasOppositelicMrg && pu.cs->sps->getUseTMMergeOppositeLic())
          {
#if JVET_Z0102_NO_ARMC_FOR_ZERO_CAND 
            m_pcInterSearch->adjustMergeCandidates(pu, tmMrgCtxOppositeLic, pu.cs->sps->getMaxNumTMMergeCand());
#else
            m_pcInterSearch->adjustMergeCandidatesInOneCandidateGroup(pu, tmMrgCtx, pu.cs->sps->getMaxNumTMMergeCand());
#endif
#if JVET_AB0079_TM_BCW_MRG
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
            if (!tmMergeRefinedMotion)
#endif
            {
              if (tmMrgCtxOppositeLic.numValidMergeCand > pu.cs->sps->getMaxNumTMOppositeLicMergeCand())
              {
                tmMrgCtxOppositeLic.numValidMergeCand = pu.cs->sps->getMaxNumTMOppositeLicMergeCand();
              }
              m_pcInterSearch->adjustMergeCandidatesBcwIdx(pu, tmMrgCtxOppositeLic);
            }
#endif
          }
#endif
        }
        if (tmMrgCtx.numValidMergeCand > pu.cs->sps->getMaxNumTMMergeCand())
        {
          tmMrgCtx.numValidMergeCand = pu.cs->sps->getMaxNumTMMergeCand();
        }
#if JVET_AG0276_LIC_FLAG_SIGNALING
        if (hasOppositelicMrg && pu.cs->sps->getUseTMMergeOppositeLic())
        {
          if (tmMrgCtxOppositeLic.numValidMergeCand > pu.cs->sps->getMaxNumTMOppositeLicMergeCand())
          {
            tmMrgCtxOppositeLic.numValidMergeCand = pu.cs->sps->getMaxNumTMOppositeLicMergeCand();
          }
        }
#endif
#else
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
        if (!tmMergeRefinedMotion)
#endif
        m_pcInterSearch->adjustInterMergeCandidates(pu, tmMrgCtx);
#endif
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
        if (tmMrgCtx.numCandToTestEnc > tmMrgCtx.numValidMergeCand)
        {
          tmMrgCtx.numCandToTestEnc = tmMrgCtx.numValidMergeCand;
        }
        for (uint32_t ui = tmMrgCtx.numValidMergeCand; ui < NUM_MERGE_CANDS; ++ui)
        {
          tmMrgCtx.bcwIdx[ui] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
          tmMrgCtx.altLMFlag[ui] = false;
          tmMrgCtx.altLMParaNeighbours[ui].resetAltLinearModel();
#endif
#if INTER_LIC
          tmMrgCtx.licFlags[ui] = false;
#endif
          tmMrgCtx.interDirNeighbours[ui] = 0;
          tmMrgCtx.mvFieldNeighbours[(ui << 1)].refIdx = NOT_VALID;
          tmMrgCtx.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
          tmMrgCtx.useAltHpelIf[ui] = false;
#if MULTI_HYP_PRED
          tmMrgCtx.addHypNeighbours[ui].clear();
#endif
          tmMrgCtx.candCost[ui] = MAX_UINT64;
        }
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
        if (tmMrgCtxOppositeLic.numCandToTestEnc > tmMrgCtxOppositeLic.numValidMergeCand)
        {
          tmMrgCtxOppositeLic.numCandToTestEnc = tmMrgCtxOppositeLic.numValidMergeCand;
        }
        for (uint32_t ui = tmMrgCtxOppositeLic.numValidMergeCand; ui < NUM_MERGE_CANDS; ++ui)
        {
          tmMrgCtxOppositeLic.bcwIdx[ui] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
          tmMrgCtxOppositeLic.altLMFlag[ui] = false;
          tmMrgCtxOppositeLic.altLMParaNeighbours[ui].resetAltLinearModel();
#endif
#if INTER_LIC
          tmMrgCtxOppositeLic.licFlags[ui] = false;
#endif
          tmMrgCtxOppositeLic.interDirNeighbours[ui] = 0;
          tmMrgCtxOppositeLic.mvFieldNeighbours[(ui << 1)].refIdx = NOT_VALID;
          tmMrgCtxOppositeLic.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
          tmMrgCtxOppositeLic.useAltHpelIf[ui] = false;
#if MULTI_HYP_PRED
          tmMrgCtxOppositeLic.addHypNeighbours[ui].clear();
#endif
          tmMrgCtxOppositeLic.candCost[ui] = MAX_UINT64;
        }
#endif
#endif
      }
#endif

#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
      if (tmMergeRefinedMotion)
      {
        pu.reduceTplSize = true;
      }
      Distortion tempCost[1];
#endif

      for( uint32_t uiMergeCand = 0; uiMergeCand < tmMrgCtx.numValidMergeCand; uiMergeCand++ )
      {
        tmMrgCtx.setMergeInfo( pu, uiMergeCand );
#if MULTI_PASS_DMVR
#if JVET_AF0057
        pu.dmvrImpreciseMv = false;
#endif
        applyBDMVR4TM[uiMergeCand] = PU::checkBDMVRCondition(pu);
        if (applyBDMVR4TM[uiMergeCand])
        {
          pu.bdmvrRefine = true;
          m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4TM[uiMergeCand << 1], m_mvBufBDMVR4TM[(uiMergeCand << 1) + 1]);
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
          if (tmMergeRefinedMotion)
          {
            applyBDMVR4TM[uiMergeCand] =  m_pcInterSearch->processBDMVR(pu, 1, tempCost);
          }
          else
#endif
          applyBDMVR4TM[uiMergeCand] =  m_pcInterSearch->processBDMVR(pu);
        }
        else
        {
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
          m_pcInterSearch->deriveTMMv(pu, tempCost);
#else
          m_pcInterSearch->deriveTMMv(pu);
#endif
        }
#else
        m_pcInterSearch->deriveTMMv( pu );
#endif
#if JVET_AF0057
        if (m_pcInterSearch->dmvrEnableEncoderCheck && pu.dmvrImpreciseMv)
        {
          dmvr4TMImpreciseMv = true;
        }
#endif

#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
        tmMrgCtx.candCost[uiMergeCand] = tempCost[0];
#endif

        // Store refined motion back to tmMrgCtx
        tmMrgCtx.interDirNeighbours[uiMergeCand] = pu.interDir;
        tmMrgCtx.bcwIdx[uiMergeCand] = pu.cu->bcwIdx;  // BCW may change, because bi may be reduced to uni by deriveTMMv(pu)
        tmMrgCtx.mvFieldNeighbours[2 * uiMergeCand    ].setMvField( pu.mv[0], pu.refIdx[0] );
        tmMrgCtx.mvFieldNeighbours[2 * uiMergeCand + 1].setMvField( pu.mv[1], pu.refIdx[1] );
        if( pu.interDir == 1 )
        {
          tmMrgCtx.mvFieldNeighbours[2 * uiMergeCand + 1].setMvField( Mv(), NOT_VALID );
        }
        if( pu.interDir == 2 )
        {
          tmMrgCtx.mvFieldNeighbours[2 * uiMergeCand    ].setMvField( Mv(), NOT_VALID );
        }
      }
#if JVET_AG0276_LIC_FLAG_SIGNALING
      pu.tmMergeFlagOppositeLic = true;
      for (uint32_t uiMergeCand = 0; uiMergeCand < tmMrgCtxOppositeLic.numValidMergeCand; uiMergeCand++)
      {
        tmMrgCtxOppositeLic.setMergeInfo(pu, uiMergeCand);
#if MULTI_PASS_DMVR
        applyBDMVR4TMOppositeLic[uiMergeCand] = PU::checkBDMVRCondition(pu);
        if (applyBDMVR4TMOppositeLic[uiMergeCand])
        {
          pu.bdmvrRefine = true;
          m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4TMOPPOSITELIC[uiMergeCand << 1], m_mvBufBDMVR4TMOPPOSITELIC[(uiMergeCand << 1) + 1]);
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
          if (tmMergeRefinedMotion)
          {
            applyBDMVR4TMOppositeLic[uiMergeCand] = m_pcInterSearch->processBDMVR(pu, 1, tempCost);
          }
          else
#endif
            applyBDMVR4TMOppositeLic[uiMergeCand] = m_pcInterSearch->processBDMVR(pu);
        }
        else
        {
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
          m_pcInterSearch->deriveTMMv(pu, tempCost);
#else
          m_pcInterSearch->deriveTMMv(pu);
#endif
        }
#else
        m_pcInterSearch->deriveTMMv(pu);
#endif

#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
        tmMrgCtxOppositeLic.candCost[uiMergeCand] = tempCost[0];
#endif
        tmMrgCtxOppositeLic.interDirNeighbours[uiMergeCand] = pu.interDir;
        tmMrgCtxOppositeLic.bcwIdx[uiMergeCand] = pu.cu->bcwIdx;
        tmMrgCtxOppositeLic.mvFieldNeighbours[2 * uiMergeCand].setMvField(pu.mv[0], pu.refIdx[0]);
        tmMrgCtxOppositeLic.mvFieldNeighbours[2 * uiMergeCand + 1].setMvField(pu.mv[1], pu.refIdx[1]);
        if (pu.interDir == 1)
        {
          tmMrgCtxOppositeLic.mvFieldNeighbours[2 * uiMergeCand + 1].setMvField(Mv(), NOT_VALID);
        }
        if (pu.interDir == 2)
        {
          tmMrgCtxOppositeLic.mvFieldNeighbours[2 * uiMergeCand].setMvField(Mv(), NOT_VALID);
        }
      }
      pu.tmMergeFlagOppositeLic = false;
#endif
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
      pu.reduceTplSize = false;
#endif

#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
      if (tmMergeRefinedMotion)
      {
        m_pcInterSearch->adjustMergeCandidatesInOneCandidateGroup(pu, tmMrgCtx, applyBDMVR4TM, NULL, NULL, pu.cs->sps->getMaxNumTMMergeCand());
#if JVET_AB0079_TM_BCW_MRG
        m_pcInterSearch->adjustMergeCandidatesBcwIdx(pu, tmMrgCtx);
#endif
        pu.tmMergeFlag = true;
        for( uint32_t uiMergeCand = 0; uiMergeCand < tmMrgCtx.numValidMergeCand; uiMergeCand++ )
        {
          tmMrgCtx.setMergeInfo( pu, uiMergeCand );
#if MULTI_PASS_DMVR
#if JVET_AF0057
          pu.dmvrImpreciseMv = false;
#endif
          applyBDMVR4TM[uiMergeCand] = PU::checkBDMVRCondition(pu);
          if (applyBDMVR4TM[uiMergeCand])
          {
            pu.bdmvrRefine = true;
            m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4TM[uiMergeCand << 1], m_mvBufBDMVR4TM[(uiMergeCand << 1) + 1]);

            applyBDMVR4TM[uiMergeCand] =  m_pcInterSearch->processBDMVR(pu);
          }
          else
          {
            m_pcInterSearch->deriveTMMv(pu);
          }
#if JVET_AF0057
          if (m_pcInterSearch->dmvrEnableEncoderCheck && pu.dmvrImpreciseMv && PU::checkBDMVRCondition(pu))
          {
            dmvr4TMImpreciseMv = true;
          }
#endif

          tmMrgCtx.interDirNeighbours[uiMergeCand] = pu.interDir;
          tmMrgCtx.bcwIdx[uiMergeCand] = pu.cu->bcwIdx;  // BCW may change, because bi may be reduced to uni by deriveTMMv(pu)
          tmMrgCtx.mvFieldNeighbours[2 * uiMergeCand    ].setMvField( pu.mv[0], pu.refIdx[0] );
          tmMrgCtx.mvFieldNeighbours[2 * uiMergeCand + 1].setMvField( pu.mv[1], pu.refIdx[1] );
          if( pu.interDir == 1 )
          {
            tmMrgCtx.mvFieldNeighbours[2 * uiMergeCand + 1].setMvField( Mv(), NOT_VALID );
          }
          if( pu.interDir == 2 )
          {
            tmMrgCtx.mvFieldNeighbours[2 * uiMergeCand    ].setMvField( Mv(), NOT_VALID );
          }
#endif
        }
      }
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
      if (tmMergeRefinedMotion)
      {
        pu.tmMergeFlagOppositeLic = true;
        m_pcInterSearch->adjustMergeCandidatesInOneCandidateGroup(pu, tmMrgCtxOppositeLic, applyBDMVR4TMOppositeLic, NULL, NULL, pu.cs->sps->getMaxNumTMMergeCand());
#if JVET_AB0079_TM_BCW_MRG
        m_pcInterSearch->adjustMergeCandidatesBcwIdx(pu, tmMrgCtxOppositeLic);
#endif
        pu.tmMergeFlag = true;
        for (uint32_t uiMergeCand = 0; uiMergeCand < tmMrgCtxOppositeLic.numValidMergeCand; uiMergeCand++)
        {
          tmMrgCtxOppositeLic.setMergeInfo(pu, uiMergeCand);
#if MULTI_PASS_DMVR
          applyBDMVR4TMOppositeLic[uiMergeCand] = PU::checkBDMVRCondition(pu);
          if (applyBDMVR4TMOppositeLic[uiMergeCand])
          {
            pu.bdmvrRefine = true;
            m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4TMOPPOSITELIC[uiMergeCand << 1], m_mvBufBDMVR4TMOPPOSITELIC[(uiMergeCand << 1) + 1]);
            applyBDMVR4TMOppositeLic[uiMergeCand] = m_pcInterSearch->processBDMVR(pu);
          }
          else
          {
            m_pcInterSearch->deriveTMMv(pu);
          }

          tmMrgCtxOppositeLic.interDirNeighbours[uiMergeCand] = pu.interDir;
          tmMrgCtxOppositeLic.bcwIdx[uiMergeCand] = pu.cu->bcwIdx;  // BCW may change, because bi may be reduced to uni by deriveTMMv(pu)
          tmMrgCtxOppositeLic.mvFieldNeighbours[2 * uiMergeCand].setMvField(pu.mv[0], pu.refIdx[0]);
          tmMrgCtxOppositeLic.mvFieldNeighbours[2 * uiMergeCand + 1].setMvField(pu.mv[1], pu.refIdx[1]);
          if (pu.interDir == 1)
          {
            tmMrgCtxOppositeLic.mvFieldNeighbours[2 * uiMergeCand + 1].setMvField(Mv(), NOT_VALID);
          }
          if (pu.interDir == 2)
          {
            tmMrgCtxOppositeLic.mvFieldNeighbours[2 * uiMergeCand].setMvField(Mv(), NOT_VALID);
          }
#endif
        }
      }
#endif
      pu.tmMergeFlagOppositeLic = false;
#endif
      pu.tmMergeFlag = false;
#if MULTI_PASS_DMVR
      pu.bdmvrRefine = false;
#endif
    }
#endif
#if MERGE_ENC_OPT
    if (affineMrgAvail)
    {
      pu.regularMergeFlag = false;
      cu.affine = true;
#if JVET_AG0276_NLIC
      AltLMAffineMergeCtx altAffineRMVFCtx;
      altAffineRMVFCtx.init();
#if JVET_AG0276_LIC_FLAG_SIGNALING
      AltLMAffineMergeCtx altBRAffineRMVFCtx;
      altBRAffineRMVFCtx.init();
#endif
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
      checkaffBmMrg = PU::isAffBMMergeFlagCoded(pu);
      uint16_t addNumRMVF = 0;
      PU::getRMVFAffineCand(pu, affineRMVFCtx, affineRMVFOriCtx, m_pcInterSearch, addNumRMVF
#if JVET_AG0276_NLIC
        , altAffineRMVFCtx
#if JVET_AG0276_LIC_FLAG_SIGNALING
        , altBRAffineRMVFCtx
#endif
#endif
      );
#endif
      PU::getAffineMergeCand(pu, affineMergeCtx
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION  
        , mrgCtxAll
#endif
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION && JVET_W0090_ARMC_TM
        , m_pcInterSearch
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
        , affineRMVFCtx
        , affineRMVFOriCtx
        , addNumRMVF
#endif
      );
#if JVET_AG0276_LIC_FLAG_SIGNALING
      int cntAffOppositeLic = 0;
      affineMergeCtxOppositeLic.numAffCandToTestEnc = 0;
      affineMergeCtxOppositeLic.numValidMergeCand = 0;
      if (hasOppositelicAff && pu.cs->sps->getUseAffMergeOppositeLic())
      {
        for (int i = 0; i < affineMergeCtx.numValidMergeCand; i++)
        {
          if (affineMergeCtx.mergeType[i] == MRG_TYPE_DEFAULT_N)
          {
            for (int mvNum = 0; mvNum < 3; mvNum++)
            {
              affineMergeCtxOppositeLic.mvFieldNeighbours[(cntAffOppositeLic << 1) + 0][mvNum] = affineMergeCtx.mvFieldNeighbours[(i << 1) + 0][mvNum];
              affineMergeCtxOppositeLic.mvFieldNeighbours[(cntAffOppositeLic << 1) + 1][mvNum] = affineMergeCtx.mvFieldNeighbours[(i << 1) + 1][mvNum];
            }
            affineMergeCtxOppositeLic.interDirNeighbours[cntAffOppositeLic] = affineMergeCtx.interDirNeighbours[i];
            affineMergeCtxOppositeLic.affineType[cntAffOppositeLic] = affineMergeCtx.affineType[i];
            affineMergeCtxOppositeLic.mergeType[cntAffOppositeLic] = affineMergeCtx.mergeType[i];
            affineMergeCtxOppositeLic.bcwIdx[cntAffOppositeLic] = affineMergeCtx.bcwIdx[i];
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
            affineMergeCtxOppositeLic.colIdx[cntAffOppositeLic] = affineMergeCtx.colIdx[i];
#endif
#if JVET_AG0276_NLIC
            affineMergeCtxOppositeLic.altLMFlag[cntAffOppositeLic] = affineMergeCtx.altLMFlag[i];
            affineMergeCtxOppositeLic.altLMParaNeighbours[cntAffOppositeLic] = affineMergeCtx.altLMParaNeighbours[i];
#endif
#if INTER_LIC                                                   
            affineMergeCtxOppositeLic.licFlags[cntAffOppositeLic] = !affineMergeCtx.licFlags[i];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
            affineMergeCtxOppositeLic.obmcFlags[cntAffOppositeLic] = affineMergeCtx.obmcFlags[i];
#endif
            affineMergeCtxOppositeLic.candCost[cntAffOppositeLic] = affineMergeCtx.candCost[i];
            cntAffOppositeLic++;
            affineMergeCtxOppositeLic.numValidMergeCand++;
          }
          else
          {
            continue;
          }
        }
        affineMergeCtxOppositeLic.numAffCandToTestEnc = std::min(affineMergeCtxOppositeLic.numValidMergeCand, affineMergeCtx.numAffCandToTestEnc);
        affineMergeCtxOppositeLic.maxNumMergeCand = affineMergeCtxOppositeLic.numValidMergeCand;
      }
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
      if (checkaffBmMrg)
      {
        PU::getBMAffineMergeCand(pu, affineBMMergeCtx, affineRMVFOriCtx, -1);
      }
#endif
#if JVET_W0090_ARMC_TM
      affineMergeCtxTmp = affineMergeCtx;
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
      affineMergeCtxTmp.numValidMergeCand = slice.getPicHeader()->getMaxNumAffineMergeCand();
      affineMergeCtxTmp.maxNumMergeCand = slice.getPicHeader()->getMaxNumAffineMergeCand();
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
#if JVET_AC0144_AFFINE_DMVR_REGRESSION
      EAffineModel affType[AFFINE_MRG_MAX_NUM_CANDS];
      Mv refinedAffineMv[AFFINE_MRG_MAX_NUM_CANDS << 1][3];
      bool applyBDMVR4Affine[AFFINE_MRG_MAX_NUM_CANDS] = { false };
      int counter = 0;
#endif
      for (uint32_t uiAffMergeCand = 0; uiAffMergeCand < affineMergeCtxTmp.numValidMergeCand; uiAffMergeCand++)
      {
#if !JVET_AC0144_AFFINE_DMVR_REGRESSION
        m_mvBufBDMVR4AFFINE[uiAffMergeCand << 1][0].setZero();
        m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1][0].setZero();
#endif
        if (affineMergeCtxTmp.mergeType[uiAffMergeCand] != MRG_TYPE_SUBPU_ATMVP)
        {
          counter++;
          if (counter > numBaseAffine)
          {
            break;
          }
        }
        pu.bdmvrRefine = false;
        if (affineMergeCtxTmp.interDirNeighbours[uiAffMergeCand] == 3 && affineMergeCtxTmp.mergeType[uiAffMergeCand] != MRG_TYPE_SUBPU_ATMVP)
        {
          pu.regularMergeFlag = false;
          pu.mergeFlag = true;
          pu.mmvdMergeFlag = false;
          pu.cu->affine = true;
          pu.interDir = affineMergeCtxTmp.interDirNeighbours[uiAffMergeCand];
          pu.cu->imv = 0;
          pu.mergeType = affineMergeCtxTmp.mergeType[uiAffMergeCand];
          pu.mv[0].setZero();
          pu.mv[1].setZero();
          pu.mergeIdx = uiAffMergeCand;
          cu.affineType = affineMergeCtxTmp.affineType[uiAffMergeCand];
          cu.bcwIdx = affineMergeCtxTmp.bcwIdx[uiAffMergeCand];
          pu.mmvdEncOptMode = 0;
#if JVET_AG0276_NLIC
          cu.altLMFlag = affineMergeCtxTmp.altLMFlag[uiAffMergeCand];
          cu.altLMParaUnit = affineMergeCtxTmp.altLMParaNeighbours[uiAffMergeCand];
#endif
#if INTER_LIC
          cu.licFlag = affineMergeCtxTmp.licFlags[uiAffMergeCand];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
          cu.obmcFlag = affineMergeCtxTmp.obmcFlags[uiAffMergeCand];
#endif
          pu.refIdx[0] = affineMergeCtxTmp.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].refIdx;
          pu.refIdx[1] = affineMergeCtxTmp.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].refIdx;
          pu.mvAffi[REF_PIC_LIST_0][0] = affineMergeCtxTmp.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv;
          pu.mvAffi[REF_PIC_LIST_0][1] = affineMergeCtxTmp.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv;
          pu.mvAffi[REF_PIC_LIST_0][2] = affineMergeCtxTmp.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv;
          pu.mvAffi[REF_PIC_LIST_1][0] = affineMergeCtxTmp.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv;
          pu.mvAffi[REF_PIC_LIST_1][1] = affineMergeCtxTmp.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv;
          pu.mvAffi[REF_PIC_LIST_1][2] = affineMergeCtxTmp.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv;
#if JVET_AG0276_LIC_BDOF_BDMVR
          if (PU::checkBDMVRCondition4Aff(pu))
#else
          if (PU::checkBDMVRCondition(pu))
#endif
          {
            if (PU::checkBDMVR4Affine(pu))
            {
              m_pcInterSearch->processBDMVR4Affine(pu);
            }
#if JVET_AC0144_AFFINE_DMVR_REGRESSION
            refinedAffineMv[(uiAffMergeCand << 1) + 0][0] = pu.mvAffi[REF_PIC_LIST_0][0];
            refinedAffineMv[(uiAffMergeCand << 1) + 0][1] = pu.mvAffi[REF_PIC_LIST_0][1];
            refinedAffineMv[(uiAffMergeCand << 1) + 0][2] = pu.mvAffi[REF_PIC_LIST_0][2];
            refinedAffineMv[(uiAffMergeCand << 1) + 1][0] = pu.mvAffi[REF_PIC_LIST_1][0];
            refinedAffineMv[(uiAffMergeCand << 1) + 1][1] = pu.mvAffi[REF_PIC_LIST_1][1];
            refinedAffineMv[(uiAffMergeCand << 1) + 1][2] = pu.mvAffi[REF_PIC_LIST_1][2];
            affType[uiAffMergeCand] = (EAffineModel)pu.cu->affineType;
            applyBDMVR4Affine[uiAffMergeCand] = true;
#endif
          }
        }
      }
      counter = 0;
      for (uint32_t uiAffMergeCand = 0; uiAffMergeCand < affineMergeCtxTmp.numValidMergeCand; uiAffMergeCand++)
      {
        if (affineMergeCtxTmp.mergeType[uiAffMergeCand] != MRG_TYPE_SUBPU_ATMVP)
        {
          counter++;
          if (counter > numBaseAffine)
          {
            break;
          }
        }
#if JVET_AC0144_AFFINE_DMVR_REGRESSION
        if (!applyBDMVR4Affine[uiAffMergeCand])
        {
          continue;
        }
        affineMergeCtxTmp.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv = refinedAffineMv[(uiAffMergeCand << 1) + 0][0];
        affineMergeCtxTmp.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv = refinedAffineMv[(uiAffMergeCand << 1) + 0][1];
        affineMergeCtxTmp.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv = refinedAffineMv[(uiAffMergeCand << 1) + 0][2];
        affineMergeCtxTmp.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv = refinedAffineMv[(uiAffMergeCand << 1) + 1][0];
        affineMergeCtxTmp.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv = refinedAffineMv[(uiAffMergeCand << 1) + 1][1];
        affineMergeCtxTmp.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv = refinedAffineMv[(uiAffMergeCand << 1) + 1][2];
        affineMergeCtxTmp.affineType[uiAffMergeCand] = affType[uiAffMergeCand];
#else
        affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 0][0];
        affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 0][0];
        affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 0][0];
        affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1][0];
        affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1][0];
        affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1][0];
#endif
      }
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
      m_pcInterSearch->sortAffineMergeCandidates(pu, affineMergeCtxTmp, affMmvdLUT, (numBaseAffine - 1 ) * AF_MMVD_MAX_REFINE_NUM, true);
#else
      m_pcInterSearch->sortAffineMergeCandidates(pu, affineMergeCtxTmp, affMmvdLUT);
#endif
#endif
      if (sps.getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
        )
      {
#if JVET_AG0276_NLIC
        if (sps.getUseAffAltLM() && !CU::isTLCond(*pu.cu))
        {
          AltLMAffineMergeCtx altLMAffMrgCtx;
          PU::getAltLMAffineMergeCand(pu, altLMAffMrgCtx);
          m_pcInterSearch->adjustAffineMergeCandidates(pu, affineMergeCtx, altLMAffMrgCtx, altAffineRMVFCtx);
#if JVET_AG0276_LIC_FLAG_SIGNALING
          if (hasOppositelicAff && pu.cs->sps->getUseAffMergeOppositeLic())
          {
            AltLMAffineMergeCtx altLMBRAffMrgCtx;
            PU::getAltLMBRAffineMergeCand(pu, altLMBRAffMrgCtx);
            m_pcInterSearch->adjustAffineMergeCandidates(pu, affineMergeCtxOppositeLic, altLMBRAffMrgCtx, altBRAffineRMVFCtx);
          }
#endif
        }
        else
        {
          m_pcInterSearch->adjustAffineMergeCandidates(pu, affineMergeCtx);
#if JVET_AG0276_LIC_FLAG_SIGNALING
          if (hasOppositelicAff && pu.cs->sps->getUseAffMergeOppositeLic())
          {
            m_pcInterSearch->adjustAffineMergeCandidates(pu, affineMergeCtxOppositeLic);
          }
#endif
        }
#else
        m_pcInterSearch->adjustAffineMergeCandidates(pu, affineMergeCtx);
#if JVET_AG0276_LIC_FLAG_SIGNALING
        if (hasOppolicAff && pu.cs->sps->getUseAffMergeOppositeLic())
        {
          m_pcInterSearch->adjustAffineMergeCandidates(pu, affineMergeCtxOppositeLic);
        }
#endif
#endif
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
        affineMergeCtx.numValidMergeCand = slice.getPicHeader()->getMaxNumAffineMergeCand();
        affineMergeCtx.maxNumMergeCand = slice.getPicHeader()->getMaxNumAffineMergeCand();
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
        if (checkaffBmMrg)
        {
          m_pcInterSearch->adjustAffineMergeCandidates(pu, affineBMMergeCtx);
        }
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
        if (hasOppositelicAff && pu.cs->sps->getUseAffMergeOppositeLic())
        {
          affineMergeCtxOppositeLic.numValidMergeCand = slice.getPicHeader()->getMaxNumAffineOppositeLicMergeCand();
          affineMergeCtxOppositeLic.maxNumMergeCand = slice.getPicHeader()->getMaxNumAffineOppositeLicMergeCand();
        }
#endif
      }
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
      if (checkaffBmMrg)
      {
        affineBMMergeCtx.numValidMergeCand = AFFINE_ADAPTIVE_DMVR_MAX_CAND;
        affineBMMergeCtx.maxNumMergeCand = AFFINE_ADAPTIVE_DMVR_MAX_CAND;
        affineBMMergeCtx.numAffCandToTestEnc = affineBMMergeCtx.numAffCandToTestEnc >= AFFINE_ADAPTIVE_DMVR_MAX_CAND - 1 ? AFFINE_ADAPTIVE_DMVR_MAX_CAND : affineBMMergeCtx.numAffCandToTestEnc + 1;
      }
#endif
      cu.affine = false;
    }
#endif
    pu.regularMergeFlag = true;

#if MULTI_PASS_DMVR
    if (cu.cs->sps->getUseDMVDMode())
    {
      cu.firstPU = &pu;
      for (uint32_t uiMergeCand = 0; uiMergeCand < mergeCtx.numValidMergeCand; uiMergeCand++)
      {
        if( mergeCtx.interDirNeighbours[uiMergeCand] == 3 )
        {
          mergeCtx.setMergeInfo( pu, uiMergeCand );
          applyBDMVR[uiMergeCand] = PU::checkBDMVRCondition(pu);
#if JVET_AF0057
          pu.dmvrImpreciseMv = false;
#endif
          if (applyBDMVR[uiMergeCand])
          {
            pu.bdmvrRefine = true;
            m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR[uiMergeCand << 1], m_mvBufBDMVR[(uiMergeCand << 1) + 1]);

            if (mergeCtx.xCheckSimilarMotion(pu.mergeIdx, PU::getBDMVRMvdThreshold(pu)))
            {
              // span motion to subPU
              for (int subPuIdx = 0; subPuIdx < MAX_NUM_SUBCU_DMVR; subPuIdx++)
              {
                m_mvBufBDMVR[uiMergeCand << 1][subPuIdx] = pu.mv[0];
                m_mvBufBDMVR[(uiMergeCand << 1) + 1][subPuIdx] = pu.mv[1];
              }
            }
            else
            {
              m_pcInterSearch->processBDMVR(pu);
#if JVET_AF0057
              if (m_pcInterSearch->dmvrEnableEncoderCheck && pu.dmvrImpreciseMv)
              {
                dmvrImpreciseMv[uiMergeCand] = true;
              }
#endif
            }
          }
        }
      }
#if JVET_AG0276_LIC_FLAG_SIGNALING
      if (hasOppositelicMrg && pu.cs->sps->getUseMergeOppositeLic())
      {
        pu.mergeOppositeLic = true;
        cu.firstPU = &pu;
        for (uint32_t uiMergeCand = 0; uiMergeCand < mergeCtxOppositeLic.numValidMergeCand; uiMergeCand++)
        {
          if (mergeCtxOppositeLic.interDirNeighbours[uiMergeCand] == 3)
          {
            mergeCtxOppositeLic.setMergeInfo(pu, uiMergeCand);
            applyBDMVROppositeLic[uiMergeCand] = PU::checkBDMVRCondition(pu);

            if (applyBDMVROppositeLic[uiMergeCand])
            {
              pu.bdmvrRefine = true;
              m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4OPPOSITELIC[uiMergeCand << 1], m_mvBufBDMVR4OPPOSITELIC[(uiMergeCand << 1) + 1]);

              if (mergeCtxOppositeLic.xCheckSimilarMotion(pu.mergeIdx, PU::getBDMVRMvdThreshold(pu)))
              {
                for (int subPuIdx = 0; subPuIdx < MAX_NUM_SUBCU_DMVR; subPuIdx++)
                {
                  m_mvBufBDMVR4OPPOSITELIC[uiMergeCand << 1][subPuIdx] = pu.mv[0];
                  m_mvBufBDMVR4OPPOSITELIC[(uiMergeCand << 1) + 1][subPuIdx] = pu.mv[1];
                }
              }
              else
              {
                m_pcInterSearch->processBDMVR(pu);
              }
            }
          }
        }
        pu.mergeOppositeLic = false;
      }
#endif
#if JVET_AF0163_TM_SUBBLOCK_REFINEMENT
      int cnt = 0;
      int validNum = 0;
      if (cu.cs->sps->getUseAffineTM()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
        && sps.getTMToolsEnableFlag()
#endif
        && PU::checkAffineTMCondition(pu)
        )
      {
        for (uint32_t uiAffMergeCand = 0; uiAffMergeCand < affineMergeCtx.numValidMergeCand; uiAffMergeCand++)
        {
          if (affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv == Mv(0, 0) && affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv == Mv(0, 0) &&
            affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv == Mv(0, 0) && affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv == Mv(0, 0) &&
            affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv == Mv(0, 0) && affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv == Mv(0, 0)
            && affineMergeCtx.mergeType[uiAffMergeCand] != MRG_TYPE_SUBPU_ATMVP)
          {
            validNum = uiAffMergeCand;
            break;
          }
          if (uiAffMergeCand == pu.cu->slice->getPicHeader()->getMaxNumAffineMergeCand() - 1)
          {
            validNum = pu.cu->slice->getPicHeader()->getMaxNumAffineMergeCand();
          }
        }
      }
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
#if JVET_AF0163_TM_SUBBLOCK_REFINEMENT
      int cntOppositeLic = 0;
      int validNumOppositeLic = 0;
      if (cu.cs->sps->getUseAffineTM()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
        && sps.getTMToolsEnableFlag()
#endif
        && PU::checkAffineTMCondition(pu)
        )
      {
        for (uint32_t uiAffMergeCand = 0; uiAffMergeCand < affineMergeCtxOppositeLic.numValidMergeCand; uiAffMergeCand++)
        {
          if (affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv == Mv(0, 0) && affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv == Mv(0, 0) &&
            affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv == Mv(0, 0) && affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv == Mv(0, 0) &&
            affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv == Mv(0, 0) && affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv == Mv(0, 0)
            && affineMergeCtxOppositeLic.mergeType[uiAffMergeCand] != MRG_TYPE_SUBPU_ATMVP)
          {
            validNumOppositeLic = uiAffMergeCand;
            break;
          }
          if (uiAffMergeCand == int(pu.cs->picHeader->getMaxNumAffineOppositeLicMergeCand()) - 1)
          {
            validNumOppositeLic = int(pu.cs->picHeader->getMaxNumAffineOppositeLicMergeCand());
          }
        }
      }
#endif
#endif
#if JVET_AB0112_AFFINE_DMVR
      if (affineMrgAvail
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
        && PU::checkBDMVR4Affine(pu)
#endif
      )
      {
#if JVET_AC0144_AFFINE_DMVR_REGRESSION
        EAffineModel affType[AFFINE_MRG_MAX_NUM_CANDS];
        Mv refinedAffineMv[AFFINE_MRG_MAX_NUM_CANDS << 1][3];
        bool applyBDMVR4Affine[AFFINE_MRG_MAX_NUM_CANDS] = { false };
#endif
        for (uint32_t uiAffMergeCand = 0; uiAffMergeCand < affineMergeCtx.numValidMergeCand; uiAffMergeCand++)
        {
#if !JVET_AC0144_AFFINE_DMVR_REGRESSION
          m_mvBufBDMVR4AFFINE[uiAffMergeCand << 1][0].setZero();
          m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1][0].setZero();
#endif
          pu.bdmvrRefine = false;
          if (affineMergeCtx.interDirNeighbours[uiAffMergeCand] == 3 && affineMergeCtx.mergeType[uiAffMergeCand] != MRG_TYPE_SUBPU_ATMVP)
          {
            pu.regularMergeFlag = false;
            pu.mergeFlag = true;
            pu.mmvdMergeFlag = false;
            pu.cu->affine = true;
            pu.interDir = affineMergeCtx.interDirNeighbours[uiAffMergeCand];
            pu.cu->imv = 0;
            pu.mergeType = affineMergeCtx.mergeType[uiAffMergeCand];
            pu.mv[0].setZero();
            pu.mv[1].setZero();
            pu.mergeIdx = uiAffMergeCand;
            cu.affineType = affineMergeCtx.affineType[uiAffMergeCand];
            cu.bcwIdx = affineMergeCtx.bcwIdx[uiAffMergeCand];
            pu.mmvdEncOptMode = 0;
#if JVET_AG0276_NLIC
            cu.altLMFlag = affineMergeCtx.altLMFlag[uiAffMergeCand];
            cu.altLMParaUnit = affineMergeCtx.altLMParaNeighbours[uiAffMergeCand];
#endif
#if INTER_LIC
            cu.licFlag = affineMergeCtx.licFlags[uiAffMergeCand];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
            cu.obmcFlag = affineMergeCtx.obmcFlags[uiAffMergeCand];
#endif
            pu.refIdx[0] = affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].refIdx;
            pu.refIdx[1] = affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].refIdx;
            pu.mvAffi[REF_PIC_LIST_0][0] = affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv;
            pu.mvAffi[REF_PIC_LIST_0][1] = affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv;
            pu.mvAffi[REF_PIC_LIST_0][2] = affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv;
            pu.mvAffi[REF_PIC_LIST_1][0] = affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv;
            pu.mvAffi[REF_PIC_LIST_1][1] = affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv;
            pu.mvAffi[REF_PIC_LIST_1][2] = affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv;
#if JVET_AG0276_LIC_BDOF_BDMVR
            if (PU::checkBDMVRCondition4Aff(pu))
#else
            if(PU::checkBDMVRCondition(pu))
#endif
            {
#if !JVET_AC0144_AFFINE_DMVR_REGRESSION
                // set merge information   
              m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4AFFINE[uiAffMergeCand << 1], m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1]);
#endif
              if (!affineMergeCtx.xCheckSimilarMotion(pu.mergeIdx, PU::getBDMVRMvdThreshold(pu)))
              {
                m_pcInterSearch->processBDMVR4Affine(pu);
#if JVET_AC0144_AFFINE_DMVR_REGRESSION
                refinedAffineMv[(uiAffMergeCand << 1) + 0][0] = pu.mvAffi[REF_PIC_LIST_0][0];
                refinedAffineMv[(uiAffMergeCand << 1) + 0][1] = pu.mvAffi[REF_PIC_LIST_0][1];
                refinedAffineMv[(uiAffMergeCand << 1) + 0][2] = pu.mvAffi[REF_PIC_LIST_0][2];
                refinedAffineMv[(uiAffMergeCand << 1) + 1][0] = pu.mvAffi[REF_PIC_LIST_1][0];
                refinedAffineMv[(uiAffMergeCand << 1) + 1][1] = pu.mvAffi[REF_PIC_LIST_1][1];
                refinedAffineMv[(uiAffMergeCand << 1) + 1][2] = pu.mvAffi[REF_PIC_LIST_1][2];
                affType[uiAffMergeCand] = (EAffineModel)pu.cu->affineType;
                applyBDMVR4Affine[uiAffMergeCand] = true;
#endif
              }
            }
          }
        }
        for (uint32_t uiAffMergeCand = 0; uiAffMergeCand < affineMergeCtx.numValidMergeCand; uiAffMergeCand++)
        {
#if JVET_AC0144_AFFINE_DMVR_REGRESSION
          if (!applyBDMVR4Affine[uiAffMergeCand])
          {
            continue;
          }
          affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv = refinedAffineMv[(uiAffMergeCand << 1) + 0][0];
          affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv = refinedAffineMv[(uiAffMergeCand << 1) + 0][1];
          affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv = refinedAffineMv[(uiAffMergeCand << 1) + 0][2];
          affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv = refinedAffineMv[(uiAffMergeCand << 1) + 1][0];
          affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv = refinedAffineMv[(uiAffMergeCand << 1) + 1][1];
          affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv = refinedAffineMv[(uiAffMergeCand << 1) + 1][2];
          affineMergeCtx.affineType[uiAffMergeCand] = affType[uiAffMergeCand];
#else
          affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 0][0];
          affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 0][0];
          affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 0][0];
          affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1][0];
          affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1][0];
          affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1][0];
#endif
        }
      }
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
      if (checkaffBmMrg && PU::checkBDMVR4Affine(pu))
      {
        affineBMMergeL0.maxNumMergeCand = AFFINE_ADAPTIVE_DMVR_MAX_CAND;
        affineBMMergeL1.maxNumMergeCand = AFFINE_ADAPTIVE_DMVR_MAX_CAND;
        Mv refinedMvL0[2][3];
        Mv refinedMvL1[2][3];
        EAffineModel affTypeL0;
        EAffineModel affTypeL1;
        pu.bdmvrRefine = false;
        pu.regularMergeFlag = false;
        pu.mergeFlag = true;
        pu.mmvdMergeFlag = false;
        pu.cu->affine = true;
        pu.mv[0].setZero();
        pu.mv[1].setZero();
        pu.cu->imv = 0;
        pu.mmvdEncOptMode = 0;
        pu.affBMDir = 0;
        for (uint32_t uiAffMergeCand = 0; uiAffMergeCand < affineBMMergeCtx.numValidMergeCand; uiAffMergeCand++)
        {
          if (uiAffMergeCand >= affineBMMergeCtx.numAffCandToTestEnc)
          {
            break;
          }
          pu.interDir = affineBMMergeCtx.interDirNeighbours[uiAffMergeCand];
          pu.mergeType = affineBMMergeCtx.mergeType[uiAffMergeCand];
          pu.mergeIdx = uiAffMergeCand;
          cu.affineType = affineBMMergeCtx.affineType[uiAffMergeCand];
          cu.bcwIdx = affineBMMergeCtx.bcwIdx[uiAffMergeCand];
#if JVET_AG0276_NLIC
          cu.altLMFlag = affineBMMergeCtx.altLMFlag[uiAffMergeCand];
          cu.altLMParaUnit = affineBMMergeCtx.altLMParaNeighbours[uiAffMergeCand];
#endif
#if INTER_LIC
          cu.licFlag = affineBMMergeCtx.licFlags[uiAffMergeCand];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
          cu.obmcFlag = affineBMMergeCtx.obmcFlags[uiAffMergeCand];
#endif
          pu.refIdx[0] = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].refIdx;
          pu.refIdx[1] = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].refIdx;
          pu.mvAffi[REF_PIC_LIST_0][0] = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv;
          pu.mvAffi[REF_PIC_LIST_0][1] = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv;
          pu.mvAffi[REF_PIC_LIST_0][2] = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv;
          pu.mvAffi[REF_PIC_LIST_1][0] = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv;
          pu.mvAffi[REF_PIC_LIST_1][1] = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv;
          pu.mvAffi[REF_PIC_LIST_1][2] = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv;

#if JVET_AG0276_LIC_BDOF_BDMVR
          if (PU::checkBDMVRCondition4Aff(pu))
#else
          if (PU::checkBDMVRCondition(pu))
#endif
          {
            m_pcInterSearch->processBDMVR4AdaptiveAffine(pu, refinedMvL0, refinedMvL1, affTypeL0, affTypeL1);
          
            affineBMMergeL0.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv = refinedMvL0[REF_PIC_LIST_0][0];
            affineBMMergeL0.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv = refinedMvL0[REF_PIC_LIST_0][1];
            affineBMMergeL0.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv = refinedMvL0[REF_PIC_LIST_0][2];
            affineBMMergeL0.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv = refinedMvL0[REF_PIC_LIST_1][0];
            affineBMMergeL0.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv = refinedMvL0[REF_PIC_LIST_1][1];
            affineBMMergeL0.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv = refinedMvL0[REF_PIC_LIST_1][2];

            affineBMMergeL0.affineType[uiAffMergeCand] = affTypeL0;

            affineBMMergeL1.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv = refinedMvL1[REF_PIC_LIST_0][0];
            affineBMMergeL1.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv = refinedMvL1[REF_PIC_LIST_0][1];
            affineBMMergeL1.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv = refinedMvL1[REF_PIC_LIST_0][2];
            affineBMMergeL1.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv = refinedMvL1[REF_PIC_LIST_1][0];
            affineBMMergeL1.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv = refinedMvL1[REF_PIC_LIST_1][1];
            affineBMMergeL1.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv = refinedMvL1[REF_PIC_LIST_1][2];

            affineBMMergeL1.affineType[uiAffMergeCand] = affTypeL1;
          }
          else
          {
            affineBMMergeL0.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv;
            affineBMMergeL0.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv;
            affineBMMergeL0.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv;
            affineBMMergeL0.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv;
            affineBMMergeL0.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv;
            affineBMMergeL0.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv;
            affineBMMergeL0.affineType[uiAffMergeCand] = affineBMMergeCtx.affineType[uiAffMergeCand];

            affineBMMergeL1.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv;
            affineBMMergeL1.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv;
            affineBMMergeL1.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv;
            affineBMMergeL1.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv;
            affineBMMergeL1.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv;
            affineBMMergeL1.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv;
            affineBMMergeL1.affineType[uiAffMergeCand] = affineBMMergeCtx.affineType[uiAffMergeCand];;
          }

          affineBMMergeL0.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].refIdx = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].refIdx;
          affineBMMergeL0.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].refIdx = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].refIdx;
          affineBMMergeL0.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].refIdx = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].refIdx;
          affineBMMergeL0.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].refIdx = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].refIdx;
          affineBMMergeL0.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].refIdx = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].refIdx;
          affineBMMergeL0.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].refIdx = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].refIdx;

          affineBMMergeL0.interDirNeighbours[uiAffMergeCand] = affineBMMergeCtx.interDirNeighbours[uiAffMergeCand];

          affineBMMergeL0.mergeType[uiAffMergeCand] = affineBMMergeCtx.mergeType[uiAffMergeCand];
          affineBMMergeL0.bcwIdx[uiAffMergeCand] = affineBMMergeCtx.bcwIdx[uiAffMergeCand];
#if JVET_AG0276_NLIC
          affineBMMergeL0.altLMFlag[uiAffMergeCand] = affineBMMergeCtx.altLMFlag[uiAffMergeCand];
          affineBMMergeL0.altLMParaNeighbours[uiAffMergeCand] = affineBMMergeCtx.altLMParaNeighbours[uiAffMergeCand];
#endif
#if INTER_LIC
          affineBMMergeL0.licFlags[uiAffMergeCand] = affineBMMergeCtx.licFlags[uiAffMergeCand];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
          affineBMMergeL0.obmcFlags[uiAffMergeCand] = affineBMMergeCtx.obmcFlags[uiAffMergeCand];
#endif
          affineBMMergeL0.numValidMergeCand++;

          affineBMMergeL1.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].refIdx = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].refIdx;
          affineBMMergeL1.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].refIdx = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].refIdx;
          affineBMMergeL1.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].refIdx = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].refIdx;
          affineBMMergeL1.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].refIdx = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].refIdx;
          affineBMMergeL1.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].refIdx = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].refIdx;
          affineBMMergeL1.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].refIdx = affineBMMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].refIdx;

          affineBMMergeL1.interDirNeighbours[uiAffMergeCand] = affineBMMergeCtx.interDirNeighbours[uiAffMergeCand];

          affineBMMergeL1.mergeType[uiAffMergeCand] = affineBMMergeCtx.mergeType[uiAffMergeCand];
          affineBMMergeL1.bcwIdx[uiAffMergeCand] = affineBMMergeCtx.bcwIdx[uiAffMergeCand];
#if JVET_AG0276_NLIC
          affineBMMergeL1.altLMFlag[uiAffMergeCand] = affineBMMergeCtx.altLMFlag[uiAffMergeCand];
          affineBMMergeL1.altLMParaNeighbours[uiAffMergeCand] = affineBMMergeCtx.altLMParaNeighbours[uiAffMergeCand];
#endif
#if INTER_LIC
          affineBMMergeL1.licFlags[uiAffMergeCand] = affineBMMergeCtx.licFlags[uiAffMergeCand];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
          affineBMMergeL1.obmcFlags[uiAffMergeCand] = affineBMMergeCtx.obmcFlags[uiAffMergeCand];
#endif
          affineBMMergeL1.numValidMergeCand++;
        }
      }
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
#if JVET_AB0112_AFFINE_DMVR
      pu.affineOppositeLic = true;
      if (affineMrgAvail
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
        && PU::checkBDMVR4Affine(pu)
#endif
        )
      {
#if JVET_AC0144_AFFINE_DMVR_REGRESSION
        EAffineModel affType[AFFINE_MRG_MAX_NUM_CANDS];
        Mv refinedAffineMv[AFFINE_MRG_MAX_NUM_CANDS << 1][3];
        bool applyBDMVR4Affine[AFFINE_MRG_MAX_NUM_CANDS] = { false };
#endif
        for (uint32_t uiAffMergeCand = 0; uiAffMergeCand < affineMergeCtxOppositeLic.numValidMergeCand; uiAffMergeCand++)
        {
#if !JVET_AC0144_AFFINE_DMVR_REGRESSION
          m_mvBufBDMVR4AFFINE[uiAffMergeCand << 1][0].setZero();
          m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1][0].setZero();
#endif
          pu.bdmvrRefine = false;
          if (affineMergeCtxOppositeLic.interDirNeighbours[uiAffMergeCand] == 3 && affineMergeCtxOppositeLic.mergeType[uiAffMergeCand] != MRG_TYPE_SUBPU_ATMVP)
          {
            pu.regularMergeFlag = false;
            pu.mergeFlag = true;
            pu.mmvdMergeFlag = false;
            pu.cu->affine = true;
            pu.interDir = affineMergeCtxOppositeLic.interDirNeighbours[uiAffMergeCand];
            pu.cu->imv = 0;
            pu.mergeType = affineMergeCtxOppositeLic.mergeType[uiAffMergeCand];
            pu.mv[0].setZero();
            pu.mv[1].setZero();
            pu.mergeIdx = uiAffMergeCand;
            cu.affineType = affineMergeCtxOppositeLic.affineType[uiAffMergeCand];
            cu.bcwIdx = affineMergeCtxOppositeLic.bcwIdx[uiAffMergeCand];
            pu.mmvdEncOptMode = 0;
#if JVET_AG0276_NLIC
            cu.altLMFlag = affineMergeCtxOppositeLic.altLMFlag[uiAffMergeCand];
            cu.altLMParaUnit = affineMergeCtxOppositeLic.altLMParaNeighbours[uiAffMergeCand];
#endif
#if INTER_LIC
            cu.licFlag = affineMergeCtxOppositeLic.licFlags[uiAffMergeCand];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
            cu.obmcFlag = affineMergeCtxOppositeLic.obmcFlags[uiAffMergeCand];
#endif
            pu.refIdx[0] = affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].refIdx;
            pu.refIdx[1] = affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].refIdx;
            pu.mvAffi[REF_PIC_LIST_0][0] = affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv;
            pu.mvAffi[REF_PIC_LIST_0][1] = affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv;
            pu.mvAffi[REF_PIC_LIST_0][2] = affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv;
            pu.mvAffi[REF_PIC_LIST_1][0] = affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv;
            pu.mvAffi[REF_PIC_LIST_1][1] = affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv;
            pu.mvAffi[REF_PIC_LIST_1][2] = affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv;
#if JVET_AG0276_LIC_BDOF_BDMVR
            if (PU::checkBDMVRCondition4Aff(pu))
#else
            if (PU::checkBDMVRCondition(pu))
#endif
            {
#if !JVET_AC0144_AFFINE_DMVR_REGRESSION
              // set merge information   
              m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4AFFINE[uiAffMergeCand << 1], m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1]);
#endif
              if (!affineMergeCtxOppositeLic.xCheckSimilarMotion(pu.mergeIdx, PU::getBDMVRMvdThreshold(pu)))
              {
                m_pcInterSearch->processBDMVR4Affine(pu);
#if JVET_AC0144_AFFINE_DMVR_REGRESSION
                refinedAffineMv[(uiAffMergeCand << 1) + 0][0] = pu.mvAffi[REF_PIC_LIST_0][0];
                refinedAffineMv[(uiAffMergeCand << 1) + 0][1] = pu.mvAffi[REF_PIC_LIST_0][1];
                refinedAffineMv[(uiAffMergeCand << 1) + 0][2] = pu.mvAffi[REF_PIC_LIST_0][2];
                refinedAffineMv[(uiAffMergeCand << 1) + 1][0] = pu.mvAffi[REF_PIC_LIST_1][0];
                refinedAffineMv[(uiAffMergeCand << 1) + 1][1] = pu.mvAffi[REF_PIC_LIST_1][1];
                refinedAffineMv[(uiAffMergeCand << 1) + 1][2] = pu.mvAffi[REF_PIC_LIST_1][2];
                affType[uiAffMergeCand] = (EAffineModel)pu.cu->affineType;
                applyBDMVR4Affine[uiAffMergeCand] = true;
#endif
              }
            }
          }
        }
        for (uint32_t uiAffMergeCand = 0; uiAffMergeCand < affineMergeCtxOppositeLic.numValidMergeCand; uiAffMergeCand++)
        {
#if JVET_AC0144_AFFINE_DMVR_REGRESSION
          if (!applyBDMVR4Affine[uiAffMergeCand])
          {
            continue;
          }
          affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv = refinedAffineMv[(uiAffMergeCand << 1) + 0][0];
          affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv = refinedAffineMv[(uiAffMergeCand << 1) + 0][1];
          affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv = refinedAffineMv[(uiAffMergeCand << 1) + 0][2];
          affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv = refinedAffineMv[(uiAffMergeCand << 1) + 1][0];
          affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv = refinedAffineMv[(uiAffMergeCand << 1) + 1][1];
          affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv = refinedAffineMv[(uiAffMergeCand << 1) + 1][2];
          affineMergeCtxOppositeLic.affineType[uiAffMergeCand] = affType[uiAffMergeCand];
#else
          affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 0][0];
          affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 0][0];
          affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 0][0];
          affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1][0];
          affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1][0];
          affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1][0];
#endif
        }
      }
      pu.affineOppositeLic = false;
#endif
#endif
#endif
#if JVET_AF0163_TM_SUBBLOCK_REFINEMENT 
      if (cu.cs->sps->getUseAffineTM() && affineMrgAvail
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
        && sps.getTMToolsEnableFlag()
#endif
        && PU::checkAffineTMCondition(pu)
        )
      {
        for (uint32_t uiAffMergeCand = 0; uiAffMergeCand < validNum; uiAffMergeCand++)
        {
          for (int i = 0; i < 3; i++)
          {
            m_mvBufBDMVR4AFFINE[uiAffMergeCand << 1][i].setZero();
            m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1][i].setZero();
          }
          pu.bdmvrRefine = false;
#if JVET_AG0276_NLIC
          if ((sps.getUseAffAltLMTM() || !affineMergeCtx.altLMFlag[uiAffMergeCand]) && affineMergeCtx.interDirNeighbours[uiAffMergeCand] != 3 && affineMergeCtx.mergeType[uiAffMergeCand] != MRG_TYPE_SUBPU_ATMVP)
#else
          if (affineMergeCtx.interDirNeighbours[uiAffMergeCand] != 3 && affineMergeCtx.mergeType[uiAffMergeCand] != MRG_TYPE_SUBPU_ATMVP)
#endif
          {
            int index = -1;
#if JVET_AG0276_NLIC
            if ((validNum + cnt) < affineMergeCtx.maxNumMergeCand && !affineMergeCtx.altLMFlag[uiAffMergeCand])
#else
            if ((validNum + cnt) < affineMergeCtx.maxNumMergeCand)
#endif
            {
              index = validNum + cnt;
              affineMergeCtx.interDirNeighbours[index] = affineMergeCtx.interDirNeighbours[uiAffMergeCand];
              affineMergeCtx.mergeType[index] = affineMergeCtx.mergeType[uiAffMergeCand];
              affineMergeCtx.affineType[index] = affineMergeCtx.affineType[uiAffMergeCand];
              affineMergeCtx.bcwIdx[index] = affineMergeCtx.bcwIdx[uiAffMergeCand];
#if JVET_AG0276_NLIC
              affineMergeCtx.altLMFlag[index] = affineMergeCtx.altLMFlag[uiAffMergeCand];
              affineMergeCtx.altLMParaNeighbours[index] = affineMergeCtx.altLMParaNeighbours[uiAffMergeCand];
#endif
#if INTER_LIC
              affineMergeCtx.licFlags[index] = affineMergeCtx.licFlags[uiAffMergeCand];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
              affineMergeCtx.obmcFlags[index] = affineMergeCtx.obmcFlags[uiAffMergeCand];
#endif
              for (int i = 0; i < 2; i++)
              {
                for (int mvNum = 0; mvNum < 3; mvNum++)
                {
                  affineMergeCtx.mvFieldNeighbours[(index << 1) + i][mvNum].refIdx = affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + i][mvNum].refIdx;
                  affineMergeCtx.mvFieldNeighbours[(index << 1) + i][mvNum].mv = affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + i][mvNum].mv;
                }
              }
              cnt++;
            }
            pu.regularMergeFlag = false;
            pu.mergeFlag = true;
            pu.mmvdMergeFlag = false;
            pu.cu->affine = true;
            pu.interDir = affineMergeCtx.interDirNeighbours[uiAffMergeCand];
            pu.cu->imv = 0;
            pu.mergeType = affineMergeCtx.mergeType[uiAffMergeCand];
            pu.mv[0].setZero();
            pu.mv[1].setZero();
            pu.mergeIdx = uiAffMergeCand;
            cu.affineType = affineMergeCtx.affineType[uiAffMergeCand];
            cu.bcwIdx = affineMergeCtx.bcwIdx[uiAffMergeCand];
            pu.mmvdEncOptMode = 0;
#if JVET_AG0276_NLIC
            pu.cu->altLMFlag = affineMergeCtx.altLMFlag[uiAffMergeCand];
            pu.cu->altLMParaUnit = affineMergeCtx.altLMParaNeighbours[uiAffMergeCand];
#endif
#if INTER_LIC
            cu.licFlag = affineMergeCtx.licFlags[uiAffMergeCand];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
            cu.obmcFlag = affineMergeCtx.obmcFlags[uiAffMergeCand];
#endif
            pu.refIdx[0] = affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].refIdx;
            pu.refIdx[1] = affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].refIdx;
            pu.mvAffi[REF_PIC_LIST_0][0] = affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv;
            pu.mvAffi[REF_PIC_LIST_0][1] = affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv;
            pu.mvAffi[REF_PIC_LIST_0][2] = affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv;
            pu.mvAffi[REF_PIC_LIST_1][0] = affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv;
            pu.mvAffi[REF_PIC_LIST_1][1] = affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv;
            pu.mvAffi[REF_PIC_LIST_1][2] = affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv;

            m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4AFFINE[uiAffMergeCand << 1], m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1]);
#if JVET_AG0276_NLIC
            if ((!affineMergeCtx.altLMFlag[uiAffMergeCand] && !affineMergeCtx.xCheckSimilarMotion1(pu.mergeIdx, PU::getBDMVRMvdThreshold(pu), false)) ||
                ( affineMergeCtx.altLMFlag[uiAffMergeCand] && !affineMergeCtx.xCheckSimilarMotion1(pu.mergeIdx, PU::getBDMVRMvdThreshold(pu), true) )
               )
#else
            if (!affineMergeCtx.xCheckSimilarMotion(pu.mergeIdx, PU::getBDMVRMvdThreshold(pu)))
#endif
            {
              m_pcInterSearch->processTM4Affine(pu, affineMergeCtx, index, true);
            }
          }
        }
        for (uint32_t uiAffMergeCand = 0; uiAffMergeCand < validNum; uiAffMergeCand++)
        {
          affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 0][0];
          affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 0][0];
          affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 0][0];
          affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1][0];
          affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1][0];
          affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1][0];
        }
      }
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
#if JVET_AF0163_TM_SUBBLOCK_REFINEMENT 
      if (cu.cs->sps->getUseAffineTM() && affineMrgAvail
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
        && sps.getTMToolsEnableFlag()
#endif
        && PU::checkAffineTMCondition(pu)
        )
      {
        for (uint32_t uiAffMergeCand = 0; uiAffMergeCand < validNumOppositeLic; uiAffMergeCand++)
        {
          for (int i = 0; i < 3; i++)
          {
            m_mvBufBDMVR4AFFINE[uiAffMergeCand << 1][i].setZero();
            m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1][i].setZero();
          }
          pu.bdmvrRefine = false;
#if JVET_AG0276_NLIC
          if ((sps.getUseAffAltLMTM() || !affineMergeCtxOppositeLic.altLMFlag[uiAffMergeCand]) && affineMergeCtxOppositeLic.interDirNeighbours[uiAffMergeCand] != 3 && affineMergeCtxOppositeLic.mergeType[uiAffMergeCand] != MRG_TYPE_SUBPU_ATMVP)
#else
          if (affineMergeCtxOppositeLic.interDirNeighbours[uiAffMergeCand] != 3 && affineMergeCtxOppositeLic.mergeType[uiAffMergeCand] != MRG_TYPE_SUBPU_ATMVP)
#endif
          {
            int index = -1;
#if JVET_AG0276_NLIC
            if ((validNumOppositeLic + cntOppositeLic) < affineMergeCtxOppositeLic.maxNumMergeCand && !affineMergeCtxOppositeLic.altLMFlag[uiAffMergeCand])
#else
            if ((validNumOppoLic + cntOppoLic) < affineMergeCtxOppoLic.maxNumMergeCand)
#endif
            {
              index = validNumOppositeLic + cntOppositeLic;
              affineMergeCtxOppositeLic.interDirNeighbours[index] = affineMergeCtxOppositeLic.interDirNeighbours[uiAffMergeCand];
              affineMergeCtxOppositeLic.mergeType[index] = affineMergeCtxOppositeLic.mergeType[uiAffMergeCand];
              affineMergeCtxOppositeLic.affineType[index] = affineMergeCtxOppositeLic.affineType[uiAffMergeCand];
              affineMergeCtxOppositeLic.bcwIdx[index] = affineMergeCtxOppositeLic.bcwIdx[uiAffMergeCand];
#if JVET_AG0276_NLIC
              affineMergeCtxOppositeLic.altLMFlag[index] = affineMergeCtxOppositeLic.altLMFlag[uiAffMergeCand];
              affineMergeCtxOppositeLic.altLMParaNeighbours[index] = affineMergeCtxOppositeLic.altLMParaNeighbours[uiAffMergeCand];
#endif
#if INTER_LIC
              affineMergeCtxOppositeLic.licFlags[index] = affineMergeCtxOppositeLic.licFlags[uiAffMergeCand];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
              affineMergeCtxOppositeLic.obmcFlags[index] = affineMergeCtxOppositeLic.obmcFlags[uiAffMergeCand];
#endif
              for (int i = 0; i < 2; i++)
              {
                for (int mvNum = 0; mvNum < 3; mvNum++)
                {
                  affineMergeCtxOppositeLic.mvFieldNeighbours[(index << 1) + i][mvNum].refIdx = affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + i][mvNum].refIdx;
                  affineMergeCtxOppositeLic.mvFieldNeighbours[(index << 1) + i][mvNum].mv = affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + i][mvNum].mv;
                }
              }
              cntOppositeLic++;
            }
            pu.regularMergeFlag = false;
            pu.mergeFlag = true;
            pu.mmvdMergeFlag = false;
            pu.cu->affine = true;
            pu.interDir = affineMergeCtxOppositeLic.interDirNeighbours[uiAffMergeCand];
            pu.cu->imv = 0;
            pu.mergeType = affineMergeCtxOppositeLic.mergeType[uiAffMergeCand];
            pu.mv[0].setZero();
            pu.mv[1].setZero();
            pu.mergeIdx = uiAffMergeCand;
            cu.affineType = affineMergeCtxOppositeLic.affineType[uiAffMergeCand];
            cu.bcwIdx = affineMergeCtxOppositeLic.bcwIdx[uiAffMergeCand];
            pu.mmvdEncOptMode = 0;
#if JVET_AG0276_NLIC
            pu.cu->altLMFlag = affineMergeCtxOppositeLic.altLMFlag[uiAffMergeCand];
            pu.cu->altLMParaUnit = affineMergeCtxOppositeLic.altLMParaNeighbours[uiAffMergeCand];
#endif
#if INTER_LIC
            cu.licFlag = affineMergeCtxOppositeLic.licFlags[uiAffMergeCand];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
            cu.obmcFlag = affineMergeCtxOppositeLic.obmcFlags[uiAffMergeCand];
#endif
            pu.refIdx[0] = affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].refIdx;
            pu.refIdx[1] = affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].refIdx;
            pu.mvAffi[REF_PIC_LIST_0][0] = affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv;
            pu.mvAffi[REF_PIC_LIST_0][1] = affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv;
            pu.mvAffi[REF_PIC_LIST_0][2] = affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv;
            pu.mvAffi[REF_PIC_LIST_1][0] = affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv;
            pu.mvAffi[REF_PIC_LIST_1][1] = affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv;
            pu.mvAffi[REF_PIC_LIST_1][2] = affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv;

            m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4AFFINE[uiAffMergeCand << 1], m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1]);
#if JVET_AG0276_NLIC
            if ((!affineMergeCtxOppositeLic.altLMFlag[uiAffMergeCand] && !affineMergeCtxOppositeLic.xCheckSimilarMotion1(pu.mergeIdx, PU::getBDMVRMvdThreshold(pu), false)) ||
                ( affineMergeCtxOppositeLic.altLMFlag[uiAffMergeCand] && !affineMergeCtxOppositeLic.xCheckSimilarMotion1(pu.mergeIdx, PU::getBDMVRMvdThreshold(pu), true) )
               )
#else
            if (!affineMergeCtxOppositeLic.xCheckSimilarMotion(pu.mergeIdx, PU::getBDMVRMvdThreshold(pu)))
#endif
            {
              m_pcInterSearch->processTM4Affine(pu, affineMergeCtxOppositeLic, index, true);
            }
          }
        }
        for (uint32_t uiAffMergeCand = 0; uiAffMergeCand < validNumOppositeLic; uiAffMergeCand++)
        {
          affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 0][0];
          affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][1].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 0][0];
          affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][2].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 0][0];
          affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1][0];
          affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][1].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1][0];
          affineMergeCtxOppositeLic.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][2].mv += m_mvBufBDMVR4AFFINE[(uiAffMergeCand << 1) + 1][0];
        }
      }
#endif
#endif
#if JVET_X0049_ADAPT_DMVR
      checkBmMrg = PU::isBMMergeFlagCoded(pu);
      if (checkBmMrg)
      {
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
        admvrRefinedMotion = PU::isArmcRefinedMotionEnabled(pu, 1);
        admvrRefinedMotion &= tplAvail;
#endif
        pu.bmMergeFlag = true;
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
        MergeCtx tmvpMergeCandCtx2;
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
        if (sps.getUseAML() && tplAvail
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
          )
        {
          PU::getTmvpBMCand(pu, tmvpMergeCandCtx2);
        }
#else
        if (sps.getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
          )
        {
          PU::getTmvpBMCand(pu, tmvpMergeCandCtx2);
          pu.bmDir = 0;
          if (tplAvail)
          {
            m_pcInterSearch->adjustMergeCandidatesInOneCandidateGroup(pu, tmvpMergeCandCtx2, 1);
          }
          else
          {
            tmvpMergeCandCtx2.numValidMergeCand = std::min(1, tmvpMergeCandCtx2.numValidMergeCand);
          }
        }
#endif
        MergeCtx namvpMergeCandCtx2;
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
        if (sps.getUseAML() && tplAvail
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
          )
        {
          PU::getNonAdjacentBMCand(pu, namvpMergeCandCtx2);
        }
#else        
        if (sps.getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
          )
        {
          PU::getNonAdjacentBMCand(pu, namvpMergeCandCtx2);
          pu.bmDir = 0;
          if (tplAvail)
          {
            m_pcInterSearch->adjustMergeCandidatesInOneCandidateGroup(pu, namvpMergeCandCtx2, 3);
          }
          else
          {
            namvpMergeCandCtx2.numValidMergeCand = std::min(3, namvpMergeCandCtx2.numValidMergeCand);
          }
        }
#endif
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
        if (!tplAvail)
        {
          PU::getInterBMCandidates(pu, bmMrgCtx
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
            , -1
            , NULL
            , NULL
#endif
          );
        }
        else
#endif
#endif

        PU::getInterBMCandidates(pu, bmMrgCtx
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
          , -1
          , (sps.getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
            ) ? &tmvpMergeCandCtx2 : NULL
          , (sps.getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
            ) ? &namvpMergeCandCtx2 : NULL
#endif
        );
#if JVET_W0090_ARMC_TM
        if (pu.cs->sps->getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
            && pu.cs->sps->getTMToolsEnableFlag()
#endif
          )
        {
          pu.bmDir = 0;
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
          if (!sps.getUseTmvpNmvpReordering())
          {
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
            if(!admvrRefinedMotion)
#endif
            m_pcInterSearch->adjustInterMergeCandidates(pu, bmMrgCtx);
#if JVET_AB0079_TM_BCW_MRG
            m_pcInterSearch->adjustMergeCandidatesBcwIdx(pu, bmMrgCtx);
#endif
          }
          else
#endif
          if (tplAvail)
          {
#if JVET_Z0102_NO_ARMC_FOR_ZERO_CAND 
            m_pcInterSearch->adjustMergeCandidates(pu, bmMrgCtx, pu.cs->sps->getMaxNumBMMergeCand());
#if !JVET_AA0093_REFINED_MOTION_FOR_ARMC && JVET_AB0079_TM_BCW_MRG
            if (bmMrgCtx.numValidMergeCand > pu.cs->sps->getMaxNumBMMergeCand())
            {
              bmMrgCtx.numValidMergeCand = pu.cs->sps->getMaxNumBMMergeCand();
            }
            m_pcInterSearch->adjustMergeCandidatesBcwIdx(pu, bmMrgCtx);
#endif
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
            if (bmMrgCtx.numCandToTestEnc > bmMrgCtx.numValidMergeCand)
            {
              bmMrgCtx.numCandToTestEnc = bmMrgCtx.numValidMergeCand;
            }
#endif
#else
            m_pcInterSearch->adjustMergeCandidatesInOneCandidateGroup(pu, bmMrgCtx, pu.cs->sps->getMaxNumBMMergeCand());
#endif
          }
          if (bmMrgCtx.numValidMergeCand > pu.cs->sps->getMaxNumBMMergeCand())
          {
            bmMrgCtx.numValidMergeCand = pu.cs->sps->getMaxNumBMMergeCand();
          }
#else
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
          if (!admvrRefinedMotion)
#endif
          m_pcInterSearch->adjustInterMergeCandidates(pu, bmMrgCtx);
#endif
        }
#endif
        if (bmMrgCtx.numValidMergeCand == 0)
        {
          checkBmMrg = false;
        }
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
        bmMrgCtxDir2 = bmMrgCtx;
#endif

        pu.bmMergeFlag = false;
        pu.bdmvrRefine = false;
      }
#endif
    }
#endif
  }
#if AFFINE_MMVD && MERGE_ENC_OPT
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  int  afMmvdBaseIdxToMergeIdxOffset = 
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
                                       !tempCS->sps->getUseTMMMVD() ?
                                       (int)PU::getMergeIdxFromAfMmvdBaseIdx(affineMergeCtx,    0) :
#endif
                                       (int)PU::getMergeIdxFromAfMmvdBaseIdx(affineMergeCtxTmp, 0);
#else
  int  afMmvdBaseIdxToMergeIdxOffset = (int)PU::getMergeIdxFromAfMmvdBaseIdx(affineMergeCtx, 0);
#endif
  int  afMmvdBaseCount = std::min<int>((int)AF_MMVD_BASE_NUM, affineMergeCtx.numValidMergeCand - afMmvdBaseIdxToMergeIdxOffset);
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_AA0093_ENHANCED_MMVD_EXTENSION
  if (!tempCS->sps->getUseTMMMVD())
  {
    afMmvdBaseCount = std::min<int>((int)ECM3_AF_MMVD_BASE_NUM, affineMergeCtx.numValidMergeCand - afMmvdBaseIdxToMergeIdxOffset);
  }
#endif
  bool affineMmvdAvail = affineMrgAvail && afMmvdBaseCount >= 1 && sps.getUseAffineMmvdMode();
#endif
  bool candHasNoResidual[MRG_MAX_NUM_CANDS + MMVD_ADD_NUM] = { false };
  bool                                        bestIsSkip = false;
  bool                                        bestIsMMVDSkip = true;
#if !MERGE_ENC_OPT
  PelUnitBuf                                  acMergeBuffer[MRG_MAX_NUM_CANDS];
#endif
  PelUnitBuf                                  acMergeTmpBuffer[MRG_MAX_NUM_CANDS];
#if JVET_AG0135_AFFINE_CIIP
  PelUnitBuf                                  acMergeAffineBuffer[AFFINE_MRG_MAX_NUM_CANDS];
#endif 
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
  PelUnitBuf                                  acTmMergeTmpBuffer[MRG_MAX_NUM_CANDS];
#endif
  PelUnitBuf                                  acMergeRealBuffer[MMVD_MRG_MAX_RD_BUF_NUM];
  PelUnitBuf *                                acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM];
  PelUnitBuf *                                singleMergeTempBuffer;
#if !MERGE_ENC_OPT
  int                                         insertPos;
#endif
  unsigned                                    uiNumMrgSATDCand = 
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                                                                 !tempCS->sps->getUseTMMMVD() ?
                                                                 mergeCtx.numValidMergeCand + VVC_MMVD_ADD_NUM :
#endif
                                                                 mergeCtx.numValidMergeCand + MMVD_ADD_NUM;

#if !MERGE_ENC_OPT
  struct ModeInfo
  {
    uint32_t mergeCand;
    bool     isRegularMerge;
    bool     isMMVD;
    bool     isCIIP;
#if CIIP_PDPC
    bool     isCiipPDPC;
    ModeInfo() : mergeCand(0), isRegularMerge(false), isMMVD(false), isCIIP(false), isCiipPDPC(false) {}
    ModeInfo(const uint32_t mergeCand, const bool isRegularMerge, const bool isMMVD, const bool isCIIP, const bool isCiipPDPC) :
      mergeCand(mergeCand), isRegularMerge(isRegularMerge), isMMVD(isMMVD), isCIIP(isCIIP), isCiipPDPC( isCiipPDPC ) {}
#else
    ModeInfo() : mergeCand(0), isRegularMerge(false), isMMVD(false), isCIIP(false) {}
    ModeInfo(const uint32_t mergeCand, const bool isRegularMerge, const bool isMMVD, const bool isCIIP) :
      mergeCand(mergeCand), isRegularMerge(isRegularMerge), isMMVD(isMMVD), isCIIP(isCIIP) {}
#endif
  };
#endif
  static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  rdModeList;
  bool                                        mrgTempBufSet = false;
  const int candNum = 
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                      !tempCS->sps->getUseTMMMVD() ?
                      mergeCtx.numValidMergeCand + (tempCS->sps->getUseMMVD() ? std::min<int>(VVC_MMVD_BASE_MV_NUM, mergeCtx.numValidMergeCand) * VVC_MMVD_MAX_REFINE_NUM : 0) :
#endif
                      mergeCtx.numValidMergeCand + (tempCS->sps->getUseMMVD() ? std::min<int>(MMVD_BASE_MV_NUM, mergeCtx.numValidMergeCand) * MMVD_MAX_REFINE_NUM : 0);

  for (int i = 0; i < candNum; i++)
  {
    if (i < mergeCtx.numValidMergeCand)
    {
#if CIIP_PDPC
#if MERGE_ENC_OPT
#if JVET_X0141_CIIP_TIMD_TM && JVET_W0123_TIMD_FUSION
#if JVET_AG0135_AFFINE_CIIP
      rdModeList.push_back(ModeInfo(i, true, false, false, false, false, 0, false));
#else
      rdModeList.push_back(ModeInfo(i, true, false, false, false, 0, false));
#endif
#else
      rdModeList.push_back(ModeInfo(i, true, false, false, false, false));
#endif
#else
      rdModeList.push_back(ModeInfo(i, true, false, false, false));
#endif
#else
#if MERGE_ENC_OPT
#if JVET_X0141_CIIP_TIMD_TM && JVET_W0123_TIMD_FUSION
      rdModeList.push_back(ModeInfo(i, true, false, false, 0, false));
#else
      rdModeList.push_back(ModeInfo(i, true, false, false, false));
#endif
#else
      rdModeList.push_back(ModeInfo(i, true, false, false));
#endif
#endif
    }
    else
    {
#if MERGE_ENC_OPT
#if CIIP_PDPC
#if JVET_X0141_CIIP_TIMD_TM && JVET_W0123_TIMD_FUSION
#if JVET_AG0135_AFFINE_CIIP
      rdModeList.push_back(ModeInfo(std::min(MMVD_ADD_NUM, i - mergeCtx.numValidMergeCand), false, true, false, false, false, 0, false));
#else
      rdModeList.push_back(ModeInfo(std::min(MMVD_ADD_NUM, i - mergeCtx.numValidMergeCand), false, true, false, false, 0, false));
#endif
#else
      rdModeList.push_back(ModeInfo(std::min(MMVD_ADD_NUM, i - mergeCtx.numValidMergeCand), false, true, false, false, false));
#endif
#else
#if JVET_X0141_CIIP_TIMD_TM && JVET_W0123_TIMD_FUSION
      rdModeList.push_back(ModeInfo(std::min(MMVD_ADD_NUM, i - mergeCtx.numValidMergeCand), false, true, false, 0, false));
#else
      rdModeList.push_back(ModeInfo(std::min(MMVD_ADD_NUM, i - mergeCtx.numValidMergeCand), false, true, false, false));
#endif
#endif
#else
#if CIIP_PDPC
      rdModeList.push_back(ModeInfo(std::min(MMVD_ADD_NUM, i - mergeCtx.numValidMergeCand), false, true, false, false));
#else
      rdModeList.push_back(ModeInfo(std::min(MMVD_ADD_NUM, i - mergeCtx.numValidMergeCand), false, true, false));
#endif
#endif
    }
  }

  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));
  for (unsigned i = 0; i < MMVD_MRG_MAX_RD_BUF_NUM; i++)
  {
    acMergeRealBuffer[i] = m_acMergeBuffer[i].getBuf(localUnitArea);
    if (i < MMVD_MRG_MAX_RD_NUM)
    {
      acMergeTempBuffer[i] = acMergeRealBuffer + i;
    }
    else
    {
      singleMergeTempBuffer = acMergeRealBuffer + i;
    }
  }

  bool isIntrainterEnabled = sps.getUseCiip();
#if CIIP_RM_BLOCK_SIZE_CONSTRAINTS
#if CTU_256
  const int maxSize = std::min<int>( MAX_TB_SIZEY, MAX_INTRA_SIZE );

  if( bestCS->area.lwidth() * bestCS->area.lheight() < 32 || bestCS->area.lwidth() > maxSize || bestCS->area.lheight() > maxSize )
#else
  if (bestCS->area.lwidth() * bestCS->area.lheight() < 32)
#endif
#else
  if (bestCS->area.lwidth() * bestCS->area.lheight() < 64 || bestCS->area.lwidth() >= MAX_CU_SIZE || bestCS->area.lheight() >= MAX_CU_SIZE)
#endif
  {
    isIntrainterEnabled = false;
  }
  bool isTestSkipMerge[MRG_MAX_NUM_CANDS] = { false };
#if JVET_AG0135_AFFINE_CIIP 
  bool isAffineTestSkipMerge[AFFINE_MRG_MAX_NUM_CANDS] = { false };
#endif
#if MERGE_ENC_OPT
  if (m_pcEncCfg->getUseFastMerge() || isIntrainterEnabled || affineMrgAvail
    )
#else
  if( m_pcEncCfg->getUseFastMerge() || isIntrainterEnabled)
#endif
  {
#if MERGE_ENC_OPT
    uiNumMrgSATDCand = m_pcEncCfg->getNumFullRDMerge();
#else
    uiNumMrgSATDCand = NUM_MRG_SATD_CAND;
#endif
    if (isIntrainterEnabled)
    {
      uiNumMrgSATDCand += 1;
    }
    bestIsSkip       = false;

    if( auto blkCache = dynamic_cast< CacheBlkInfoCtrl* >( m_modeCtrl ) )
    {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      if (slice.getUseIBC())
#else
      if (slice.getSPS()->getIBCFlag())
#endif
      {
        ComprCUCtx cuECtx = m_modeCtrl->getComprCUCtx();
        bestIsSkip = blkCache->isSkip(tempCS->area) && cuECtx.bestCU;
      }
      else
      {
        bestIsSkip = blkCache->isSkip( tempCS->area );
      }
      bestIsMMVDSkip = blkCache->isMMVDSkip(tempCS->area);
    }

    if (isIntrainterEnabled) // always perform low complexity check
    {
      bestIsSkip = false;
    }
#if MERGE_ENC_OPT
    if (affineMrgAvail)
    {
      bestIsSkip = false;
      uiNumMrgSATDCand += NUM_AFF_MRG_SATD_CAND;
    }

#if TM_MRG
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    if (tempCS->sps->getUseTMMrgMode())
#else
    if (tempCS->sps->getUseDMVDMode())
#endif
    {
      bestIsSkip = false;
      uiNumMrgSATDCand += TM_MAX_NUM_SATD_CAND;
    }
#endif
#endif

    static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> candCostList;

    // 1. Pass: get SATD-cost for selected candidates and reduce their count
    if( !bestIsSkip )
    {
      rdModeList.clear();
      mrgTempBufSet       = true;
      const TempCtx ctxStart(m_ctxCache, m_CABACEstimator->getCtx());

      CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );
#if !MERGE_ENC_OPT
      const double sqrtLambdaForFirstPassIntra = m_pcRdCost->getMotionLambda( ) * FRAC_BITS_SCALE;
#endif
      partitioner.setCUData( cu );
      cu.slice            = tempCS->slice;
      cu.tileIdx          = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
#if JVET_AG0276_NLIC
      cu.altLMFlag = false;
      cu.altLMParaUnit.resetAltLinearModel();
#endif
#if INTER_LIC
      cu.licFlag          = false;
#endif
      cu.skip             = false;
      cu.mmvdSkip = false;
      cu.geoFlag          = false;
    //cu.affine
      cu.predMode         = MODE_INTER;
    //cu.licFlag
      cu.chromaQpAdj      = m_cuChromaQpOffsetIdxPlus1;
      cu.qp               = encTestMode.qp;
    //cu.emtFlag  is set below

      PredictionUnit &pu  = tempCS->addPU( cu, partitioner.chType );
#if MERGE_ENC_OPT
      cu.affine = false;
      pu.ciipFlag = false;
#if CIIP_PDPC
      pu.ciipPDPC = false;
#endif
#if JVET_AG0135_AFFINE_CIIP
      pu.ciipAffine = false;
#endif
      pu.mmvdMergeFlag = false;
#if AFFINE_MMVD
      pu.afMmvdFlag = false;
#endif
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
      pu.tmMergeFlag = false;
#endif
#if JVET_X0049_ADAPT_DMVR
      pu.bmMergeFlag = false;
#endif
#if MULTI_PASS_DMVR
      pu.bdmvrRefine = false;
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
      pu.mergeOppositeLic = false;
      pu.affineOppositeLic = false;
      pu.tmMergeFlagOppositeLic = false;
#endif
#endif
      DistParam distParam;
      const bool bUseHadamard = !tempCS->slice->getDisableSATDForRD();
      m_pcRdCost->setDistParam (distParam, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y(), sps.getBitDepth (CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);

#if MERGE_ENC_OPT
      xCheckSATDCostRegularMerge(tempCS, cu, pu, mergeCtx, acMergeTempBuffer, singleMergeTempBuffer, acMergeTmpBuffer
#if !MULTI_PASS_DMVR
                               , refinedMvdL0
#endif
                               , uiNumMrgSATDCand, rdModeList, candCostList, distParam, ctxStart
#if MULTI_PASS_DMVR
                               , applyBDMVR
#endif
#if JVET_AF0057
                               , dmvrImpreciseMv
#endif

      );
#else
      const UnitArea localUnitArea( tempCS->area.chromaFormat, Area( 0, 0, tempCS->area.Y().width, tempCS->area.Y().height) );
#if MULTI_HYP_PRED
      const bool testMHP = tempCS->sps->getUseInterMultiHyp()
        && (tempCS->area.lumaSize().area() > MULTI_HYP_PRED_RESTRICT_BLOCK_SIZE 
        && std::min(tempCS->area.lwidth(), tempCS->area.lheight()) >= MULTI_HYP_PRED_RESTRICT_MIN_WH);
#endif
      for( uint32_t uiMergeCand = 0; uiMergeCand < mergeCtx.numValidMergeCand; uiMergeCand++ )
      {
        mergeCtx.setMergeInfo( pu, uiMergeCand );

        PU::spanMotionInfo( pu, mergeCtx );
#if !MULTI_PASS_DMVR
        pu.mvRefine = true;
#endif
        distParam.cur = singleMergeTempBuffer->Y();
        acMergeTmpBuffer[uiMergeCand] = m_acMergeTmpBuffer[uiMergeCand].getBuf(localUnitArea);
#if INTER_LIC
        m_pcInterSearch->m_storeBeforeLIC = mergeCtx.interDirNeighbours[uiMergeCand] == 3 ? false : true;
        if (m_pcInterSearch->m_storeBeforeLIC)
        {
          m_pcInterSearch->m_predictionBeforeLIC = acMergeTmpBuffer[uiMergeCand];
          m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X, true, true);
        }
        else
#endif
#if MULTI_PASS_DMVR
        if (applyBDMVR[uiMergeCand])
        {
          if (isIntrainterEnabled)
          {
#if MULTI_HYP_PRED
            pu.addHypData.clear();
            pu.numMergedAddHyps = 0;
#endif
            pu.mvRefine = false;
            pu.ciipFlag = true;
            m_pcInterSearch->motionCompensation(pu, acMergeTmpBuffer[uiMergeCand]);
            pu.ciipFlag = false;
#if MULTI_HYP_PRED
            mergeCtx.setMergeInfo(pu, uiMergeCand);
#endif
          }
          pu.bdmvrRefine = true;
          m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR[uiMergeCand << 1], m_mvBufBDMVR[(uiMergeCand << 1) + 1]);
          pu.mvRefine = true;

          m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer);

          if( pu.bdmvrRefine )
          {
            ::memcpy( m_mvBufEncBDOF[uiMergeCand], m_pcInterSearch->getBdofSubPuMvOffset(), sizeof( Mv ) * BDOF_SUBPU_MAX_NUM );
          }

          pu.mvRefine = false;
        }
        else
        {
#endif
        m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X, true, true, &(acMergeTmpBuffer[uiMergeCand]));
#if MULTI_PASS_DMVR
        }
#endif
#if INTER_LIC
        m_pcInterSearch->m_storeBeforeLIC = false;
#endif
        acMergeBuffer[uiMergeCand] = m_acRealMergeBuffer[uiMergeCand].getBuf(localUnitArea);
        acMergeBuffer[uiMergeCand].copyFrom(*singleMergeTempBuffer);
#if !MULTI_PASS_DMVR
        pu.mvRefine = false;
        if( mergeCtx.interDirNeighbours[uiMergeCand] == 3 )
        {
          mergeCtx.mvFieldNeighbours[2*uiMergeCand].mv   = pu.mv[0];
          mergeCtx.mvFieldNeighbours[2*uiMergeCand+1].mv = pu.mv[1];
          {
            int dx, dy, i, j, num = 0;
            dy = std::min<int>(pu.lumaSize().height, DMVR_SUBCU_HEIGHT);
            dx = std::min<int>(pu.lumaSize().width, DMVR_SUBCU_WIDTH);
            if (PU::checkDMVRCondition(pu))
            {
              for (i = 0; i < (pu.lumaSize().height); i += dy)
              {
                for (j = 0; j < (pu.lumaSize().width); j += dx)
                {
                  refinedMvdL0[num][uiMergeCand] = pu.mvdL0SubPu[num];
                  num++;
                }
              }
            }
          }
        }
#endif

        Distortion uiSad = distParam.distFunc(distParam);
        m_CABACEstimator->getCtx() = ctxStart;
        uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
        double cost = (double)uiSad + (double)fracBits * sqrtLambdaForFirstPassIntra;
#if MULTI_HYP_PRED
        if (testMHP && pu.addHypData.size() < tempCS->sps->getMaxNumAddHyps())
        {
          uint32_t uiBitsCand = uiMergeCand + 1 + 1 + 1; // one bit for merge flag,  one bit for subblock_merge_flag, and one bit for regualr_merge_flag
          MEResult mergeResult;
          mergeResult.cu = cu;
          mergeResult.pu = pu;
          mergeResult.bits = uiBitsCand;
          mergeResult.cost = uiSad + m_pcRdCost->getCost(uiBitsCand);
          m_baseResultsForMH.push_back(mergeResult);
        }
#endif
        insertPos = -1;
#if CIIP_PDPC
        updateCandList(ModeInfo(uiMergeCand, true, false, false, false), cost, rdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
#else
        updateCandList(ModeInfo(uiMergeCand, true, false, false), cost, rdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
#endif
        if (insertPos != -1)
        {
          if (insertPos == rdModeList.size() - 1)
          {
            swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
          }
          else
          {
            for (uint32_t i = uint32_t(rdModeList.size()) - 1; i > insertPos; i--)
            {
              swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
            }
            swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
          }
        }
        CHECK(std::min(uiMergeCand + 1, uiNumMrgSATDCand) != rdModeList.size(), "");
#if MULTI_PASS_DMVR
        pu.bdmvrRefine = false;
#endif
      }
#endif
#if JVET_AG0135_AFFINE_CIIP
      if (affineMrgAvail)
      {
        xCheckSATDCostAffineMerge(tempCS, cu, pu, affineMergeCtx, mrgCtx, acMergeTempBuffer, singleMergeTempBuffer, acMergeAffineBuffer, uiNumMrgSATDCand, rdModeList, candCostList, distParam, ctxStart);
      }
#endif
      if (isIntrainterEnabled)
      {
#if MERGE_ENC_OPT
#if JVET_AB0079_TM_BCW_MRG
#if JVET_AG0276_NLIC
        xCheckSATDCostCiipMerge(tempCS, cu, pu, mrgCtxCiip, acMergeTempBuffer, singleMergeTempBuffer, acMergeTmpBuffer, uiNumMrgSATDCand, rdModeList, candCostList, distParam, ctxStart, mergeOrgCtx);
#else
        xCheckSATDCostCiipMerge(tempCS, cu, pu, mrgCtxCiip, acMergeTempBuffer, singleMergeTempBuffer, acMergeTmpBuffer, uiNumMrgSATDCand, rdModeList, candCostList, distParam, ctxStart);
#endif
#else
        xCheckSATDCostCiipMerge(tempCS, cu, pu, mergeCtx, acMergeTempBuffer, singleMergeTempBuffer, acMergeTmpBuffer, uiNumMrgSATDCand, rdModeList, candCostList, distParam, ctxStart);
#endif
#if JVET_AG0135_AFFINE_CIIP
        if (sps.getUseCiipAffine())
        {
          xCheckSATDCostCiipAffineMerge(tempCS, cu, pu, affineMergeCtx, mrgCtx, acMergeTempBuffer, singleMergeTempBuffer, acMergeAffineBuffer, uiNumMrgSATDCand, rdModeList, candCostList, distParam, ctxStart);
        }
#endif
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
        if (sps.getUseCiipTmMrg())
        {
            xCheckSATDCostCiipTmMerge(tempCS, cu, pu, ciipTmMrgCtx, acMergeTempBuffer, singleMergeTempBuffer, acTmMergeTmpBuffer, uiNumMrgSATDCand, rdModeList, candCostList, distParam, ctxStart);
        }
#endif
#else
        // prepare for Intra bits calculation
        pu.ciipFlag = true;

        // save the to-be-tested merge candidates
        uint32_t CiipMergeCand[NUM_MRG_SATD_CAND];
        for (uint32_t mergeCnt = 0; mergeCnt < std::min(NUM_MRG_SATD_CAND, (const int)mergeCtx.numValidMergeCand); mergeCnt++)
        {
          CiipMergeCand[mergeCnt] = rdModeList[mergeCnt].mergeCand;
        }
#if JVET_X0141_CIIP_TIMD_TM && JVET_W0123_TIMD_FUSION
        int intraMode = PLANAR_IDX;
        if (mergeCtx.numValidMergeCand)
        {
          const CompArea &area = cu.Y();
          if (cu.slice->getSPS()->getUseTimd() && (cu.lwidth() * cu.lheight() <= CIIP_MAX_SIZE) && cu.cs->slice->getSPS()->getUseCiipTimd())
          {
#if SECONDARY_MPM && ENABLE_DIMD
            IntraPrediction::deriveDimdMode(cu.cs->picture->getRecoBuf(area), area, cu);
#endif
            cu.timdMode = m_pcIntraSearch->deriveTimdMode(cu.cs->picture->getRecoBuf(area), area, cu);
            intraMode = MAP131TO67(cu.timdMode);
          }
        }
#endif
        for (uint32_t mergeCnt = 0; mergeCnt < std::min(std::min(NUM_MRG_SATD_CAND, (const int)mergeCtx.numValidMergeCand), 4); mergeCnt++)
        {
          uint32_t mergeCand = CiipMergeCand[mergeCnt];
          acMergeTmpBuffer[mergeCand] = m_acMergeTmpBuffer[mergeCand].getBuf(localUnitArea);

          // estimate merge bits
          mergeCtx.setMergeInfo(pu, mergeCand);

          // first round
          pu.intraDir[0] = PLANAR_IDX;
#if CIIP_PDPC
          for (int intraCnt = 0; intraCnt < 2; intraCnt++)
          {
            pu.ciipPDPC = intraCnt == 1;
#else
          uint32_t intraCnt = 0;
#endif
#if JVET_X0141_CIIP_TIMD_TM && JVET_W0123_TIMD_FUSION
          pu.intraDir[0] = pu.ciipPDPC ? PLANAR_IDX : intraMode;
#endif
          PelBuf ciipBuff = m_ciipBuffer[intraCnt].getBuf(localUnitArea.Y());
          // generate intrainter Y prediction
          if (mergeCnt == 0)
          {
            m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Y());
            m_pcIntraSearch->predIntraAng(COMPONENT_Y, ciipBuff, pu);
          }
//#if INTER_LIC
//          if( mergeCtx.interDirNeighbours[mergeCand] != 3 )
//          {
//            pu.cs->getPredBuf( pu ).copyFrom( m_acRealMergeBuffer[MRG_MAX_NUM_CANDS + mergeCand].getBuf( localUnitArea ) );
//          }
//          else
//#endif
            if (pu.cs->picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
            {
              m_pcIntraSearch->geneWeightedPred<true>(COMPONENT_Y, singleMergeTempBuffer->Y(), pu, acMergeTmpBuffer[mergeCand].Y(), ciipBuff, m_pcReshape->getFwdLUT().data());
            }
            else
            {
              m_pcIntraSearch->geneWeightedPred<false>(COMPONENT_Y, singleMergeTempBuffer->Y(), pu, acMergeTmpBuffer[mergeCand].Y(), ciipBuff);
            }

          // calculate cost
            if (pu.cs->picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
            {
              PelBuf tmp = m_acGeoWeightedBuffer->getBuf(localUnitArea.Y());
              tmp.rspSignal(singleMergeTempBuffer->Y(), m_pcReshape->getInvLUT());
              distParam.cur = tmp;
            }
            else
            {
              distParam.cur = singleMergeTempBuffer->Y();
            }

          //distParam.cur = pu.cs->getPredBuf(pu).Y();
          Distortion sadValue = distParam.distFunc(distParam);
          if (pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
          {
            pu.cs->getPredBuf(pu).Y().rspSignal(m_pcReshape->getFwdLUT());
          }
          m_CABACEstimator->getCtx() = ctxStart;
          pu.regularMergeFlag = false;
          uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
          double cost = (double)sadValue + (double)fracBits * sqrtLambdaForFirstPassIntra;
          insertPos = -1;
#if CIIP_PDPC
          updateCandList(ModeInfo(mergeCand, false, false, true, pu.ciipPDPC), cost, rdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
#else
          updateCandList(ModeInfo(mergeCand, false, false, true), cost, rdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
#endif
          if (insertPos != -1)
          {
            for (int i = int(rdModeList.size()) - 1; i > insertPos; i--)
            {
              swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
            }
            swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
          }
#if CIIP_PDPC
          }
#endif
        }
        pu.ciipFlag = false;
#if CIIP_PDPC
        pu.ciipPDPC = false;
#endif
#if JVET_AG0135_AFFINE_CIIP
        pu.ciipAffine = false;
#endif
#endif
      }
      if ( pu.cs->sps->getUseMMVD() )
      {
#if MERGE_ENC_OPT
#if JVET_W0090_ARMC_TM
        xCheckSATDCostMmvdMerge(tempCS, cu, pu, mergeCtxtmp, acMergeTempBuffer, singleMergeTempBuffer, uiNumMrgSATDCand, rdModeList, candCostList, distParam, ctxStart
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                           ,     mmvdLUT
#endif
                                );
#else
        xCheckSATDCostMmvdMerge(tempCS, cu, pu, mergeCtx, acMergeTempBuffer, singleMergeTempBuffer, uiNumMrgSATDCand, rdModeList, candCostList, distParam, ctxStart
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                           ,     mmvdLUT
#endif
                                );
#endif
#else
        cu.mmvdSkip = true;
        pu.regularMergeFlag = true;
        const int tempNum = (mergeCtx.numValidMergeCand > 1) ? MMVD_ADD_NUM : MMVD_ADD_NUM >> 1;
        for (int mmvdMergeCand = 0; mmvdMergeCand < tempNum; mmvdMergeCand++)
        {
          int baseIdx = mmvdMergeCand / MMVD_MAX_REFINE_NUM;
          int refineStep = (mmvdMergeCand - (baseIdx * MMVD_MAX_REFINE_NUM)) / 4;
          if (refineStep >= m_pcEncCfg->getMmvdDisNum())
            continue;
#if JVET_W0090_ARMC_TM
          mergeCtxtmp.setMmvdMergeCandiInfo(pu, mmvdMergeCand);
#else
          mergeCtx.setMmvdMergeCandiInfo(pu, mmvdMergeCand);
#endif

          PU::spanMotionInfo(pu, mergeCtx);
          pu.mvRefine = true;
          distParam.cur = singleMergeTempBuffer->Y();
          pu.mmvdEncOptMode = (refineStep > 2 ? 2 : 1);
          CHECK(!pu.mmvdMergeFlag, "MMVD merge should be set");
          // Don't do chroma MC here
          m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X, true, false);
          pu.mmvdEncOptMode = 0;
          pu.mvRefine = false;
          Distortion uiSad = distParam.distFunc(distParam);

          m_CABACEstimator->getCtx() = ctxStart;
          uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
          double cost = (double)uiSad + (double)fracBits * sqrtLambdaForFirstPassIntra;
#if MULTI_HYP_PRED
          if (testMHP && pu.addHypData.size() < tempCS->sps->getMaxNumAddHyps())
          {
            uint32_t uiBitsCand = baseIdx + refineStep + 2 + 1 + 1 + 1; // one bit for merge flag,  one bit for subblock_merge_flag, and one bit for regualr_merge_flag
            MEResult mergeResult;
            mergeResult.cu = cu;
            mergeResult.pu = pu;
            mergeResult.bits = uiBitsCand;
            mergeResult.cost = uiSad + m_pcRdCost->getCost(uiBitsCand);
            m_baseResultsForMH.push_back(mergeResult);
          }
#endif
          insertPos = -1;
#if CIIP_PDPC
          updateCandList(ModeInfo(mmvdMergeCand, false, true, false, false), cost, rdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
#else
          updateCandList(ModeInfo(mmvdMergeCand, false, true, false), cost, rdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
#endif
          if (insertPos != -1)
          {
            for (int i = int(rdModeList.size()) - 1; i > insertPos; i--)
            {
              swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
            }
            swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
          }
        }
#endif
      }
#if MERGE_ENC_OPT
#if TM_MRG
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
      if (sps.getUseTMMrgMode())
#else
      if (sps.getUseDMVDMode())
#endif
      {
        xCheckSATDCostTMMerge(tempCS, cu, pu, tmMrgCtx, acMergeTempBuffer, singleMergeTempBuffer, uiNumMrgSATDCand, rdModeList, candCostList, distParam, ctxStart
#if MULTI_PASS_DMVR
          , applyBDMVR4TM
#endif
#if JVET_AF0057
          , dmvr4TMImpreciseMv
#endif
        );
      }
#endif
#if !JVET_AG0135_AFFINE_CIIP
      if (affineMrgAvail)
      {
        xCheckSATDCostAffineMerge(tempCS, cu, pu, affineMergeCtx, mrgCtx, acMergeTempBuffer, singleMergeTempBuffer, uiNumMrgSATDCand, rdModeList, candCostList, distParam, ctxStart);
      }
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
      if (checkaffBmMrg)
      {
        xCheckSATDCostBMAffineMerge(tempCS, cu, pu, affineBMMergeL0, REF_PIC_LIST_0, mrgCtx, acMergeTempBuffer, singleMergeTempBuffer, uiNumMrgSATDCand, rdModeList, candCostList, distParam, ctxStart);
        xCheckSATDCostBMAffineMerge(tempCS, cu, pu, affineBMMergeL1, REF_PIC_LIST_1, mrgCtx, acMergeTempBuffer, singleMergeTempBuffer, uiNumMrgSATDCand, rdModeList, candCostList, distParam, ctxStart);
      }
#endif
#if AFFINE_MMVD
      if (affineMmvdAvail)
      {
#if JVET_W0090_ARMC_TM
        xCheckSATDCostAffineMmvdMerge(tempCS, cu, pu, affineMergeCtxTmp, mrgCtx, acMergeTempBuffer, singleMergeTempBuffer, uiNumMrgSATDCand, rdModeList, candCostList, distParam, ctxStart
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                                          , affMmvdLUT
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  , numBaseAffine
#endif
#endif
                                      );
#else
        xCheckSATDCostAffineMmvdMerge(tempCS, cu, pu, affineMergeCtx, mrgCtx, acMergeTempBuffer, singleMergeTempBuffer, uiNumMrgSATDCand, rdModeList, candCostList, distParam, ctxStart
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                                          , affMmvdLUT
#endif
                                      );
#endif
      }
#endif
#endif
#if JVET_X0049_ADAPT_DMVR
      if (sps.getUseDMVDMode() && checkBmMrg)
      {
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
        xCheckSATDCostBMMerge(tempCS, cu, pu, bmMrgCtx, bmMrgCtxDir2, admvrRefinedMotion, acMergeTempBuffer, singleMergeTempBuffer, uiNumMrgSATDCand, rdModeList, candCostList, distParam, ctxStart
#if MULTI_PASS_DMVR
          , applyBDMVR4BM
#endif
        );
#else
        xCheckSATDCostBMMerge(tempCS, cu, pu, bmMrgCtx, acMergeTempBuffer, singleMergeTempBuffer, uiNumMrgSATDCand, rdModeList, candCostList, distParam, ctxStart
#if MULTI_PASS_DMVR
          , applyBDMVR4BM
#endif
        );
#endif
      }
#endif
      // Try to limit number of candidates using SATD-costs
      for( uint32_t i = 1; i < uiNumMrgSATDCand; i++ )
      {
        if( candCostList[i] > MRG_FAST_RATIO * candCostList[0] )
        {
          uiNumMrgSATDCand = i;
          break;
        }
      }
#if JVET_AG0276_LIC_FLAG_SIGNALING
      if (hasOppositelicMrg && pu.cs->sps->getUseMergeOppositeLic())
      {
        xCheckSATDCostRegularMergeOppositeLic(tempCS, cu, pu, mergeCtxOppositeLic, acMergeTempBuffer, singleMergeTempBuffer, acMergeTmpBuffer
#if !MULTI_PASS_DMVR
          , refinedMvdL0
#endif
          , uiNumMrgSATDCand, rdModeList, candCostList, distParam, ctxStart
#if MULTI_PASS_DMVR
          , applyBDMVROppositeLic
#endif
        );
      }
#if TM_MRG
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
      if (sps.getUseTMMrgMode())
#else
      if (sps.getUseDMVDMode())
#endif
      {
        if (hasOppositelicMrg && pu.cs->sps->getUseTMMergeOppositeLic())
        {
          xCheckSATDCostTMMergeOppositeLic(tempCS, cu, pu, tmMrgCtxOppositeLic, acMergeTempBuffer, singleMergeTempBuffer, uiNumMrgSATDCand, rdModeList, candCostList, distParam, ctxStart
#if MULTI_PASS_DMVR
            , applyBDMVR4TMOppositeLic
#endif
          );
        }
#endif
      }
      if (affineMrgAvail)
      {
        if (hasOppositelicAff && pu.cs->sps->getUseAffMergeOppositeLic())
        {
          xCheckSATDCostAffineMergeOppositeLic(tempCS, cu, pu, affineMergeCtxOppositeLic, mrgCtx, acMergeTempBuffer, singleMergeTempBuffer, uiNumMrgSATDCand, rdModeList, candCostList, distParam, ctxStart
          );
        }
      }
#endif
      setMergeBestSATDCost( candCostList[0] );

      if (isIntrainterEnabled && isChromaEnabled(pu.cs->pcv->chrFormat))
      {
        pu.ciipFlag = true;
        bool tag[2] = { false, false };

        for (uint32_t mergeCnt = 0; mergeCnt < uiNumMrgSATDCand; mergeCnt++)
        {
          if (rdModeList[mergeCnt].isCIIP)
          {
            pu.intraDir[0] = PLANAR_IDX;
            pu.intraDir[1] = DM_CHROMA_IDX;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
            if (pu.chromaSize().width == 2)
              continue;
#endif
#if CIIP_PDPC
            pu.ciipPDPC = rdModeList[mergeCnt].isCiipPDPC;
            uint32_t bufIdx = pu.ciipPDPC ? 1 : 0;
#else
            uint32_t bufIdx = 0;
#endif
#if JVET_AG0135_AFFINE_CIIP
            pu.ciipAffine = rdModeList[mergeCnt].isCiipAffine;
#endif
#if JVET_X0141_CIIP_TIMD_TM && JVET_W0123_TIMD_FUSION
              pu.intraDir[0] = rdModeList[mergeCnt].intraMode;
#endif
            if (!tag[bufIdx])
            {
              m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cb());
              PelBuf ciipBuffCb = m_ciipBuffer[bufIdx].getBuf(localUnitArea.Cb());
              m_pcIntraSearch->predIntraAng(COMPONENT_Cb, ciipBuffCb, pu);

              m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cr());
              PelBuf ciipBuffCr = m_ciipBuffer[bufIdx].getBuf(localUnitArea.Cr());
              m_pcIntraSearch->predIntraAng(COMPONENT_Cr, ciipBuffCr, pu);

              tag[bufIdx] = true;
            }
          }
        }
        pu.ciipFlag = false;
#if CIIP_PDPC
        pu.ciipPDPC = false;
#endif
#if JVET_AG0135_AFFINE_CIIP
        pu.ciipAffine = false;
#endif
      }

      tempCS->initStructData( encTestMode.qp );
      m_CABACEstimator->getCtx() = ctxStart;
    }
    else
    {
      if (bestIsMMVDSkip)
      {
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
        uiNumMrgSATDCand = mergeCtx.numValidMergeCand + std::min<int>(MMVD_BASE_MV_NUM, mergeCtx.numValidMergeCand) * MMVD_MAX_REFINE_NUM;
#else
        uiNumMrgSATDCand = mergeCtx.numValidMergeCand + ((mergeCtx.numValidMergeCand > 1) ? MMVD_ADD_NUM : MMVD_ADD_NUM >> 1);
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
        if(!tempCS->sps->getUseTMMMVD())
        {
          uiNumMrgSATDCand = mergeCtx.numValidMergeCand + ((mergeCtx.numValidMergeCand > 1) ? VVC_MMVD_ADD_NUM : VVC_MMVD_ADD_NUM >> 1);
        }
#endif
      }
      else
      {
        uiNumMrgSATDCand = mergeCtx.numValidMergeCand;
      }
    }
  }
  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;
  uint32_t iteration;
  uint32_t iterationBegin = 0;
  iteration = 2;

  for (uint32_t uiNoResidualPass = iterationBegin; uiNoResidualPass < iteration; ++uiNoResidualPass) // 0: no skip 1 : skip
  {
    for( uint32_t uiMrgHADIdx = 0; uiMrgHADIdx < uiNumMrgSATDCand; uiMrgHADIdx++ )
    {
      uint32_t uiMergeCand = rdModeList[uiMrgHADIdx].mergeCand;

#if JVET_AG0135_AFFINE_CIIP
      if (uiNoResidualPass != 0 && rdModeList[uiMrgHADIdx].isCIIP && rdModeList[uiMrgHADIdx].isCiipAffine)
      {
        if (isAffineTestSkipMerge[uiMergeCand])
        {
          continue;
        }
      }
      if (uiNoResidualPass != 0 && rdModeList[uiMrgHADIdx].isCIIP && !rdModeList[uiMrgHADIdx].isCiipAffine && !rdModeList[uiMrgHADIdx].isTMMrg) // intrainter does not support skip mode
#else
      if (uiNoResidualPass != 0 && rdModeList[uiMrgHADIdx].isCIIP) // intrainter does not support skip mode
#endif
      {
        if (isTestSkipMerge[uiMergeCand])
        {
          continue;
        }
      }
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
     if(tempCS->sps->getUseTMMMVD())
#endif
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
       if (rdModeList[uiMrgHADIdx].isMMVD && (uiMergeCand - (uiMergeCand / MMVD_MAX_REFINE_NUM)* MMVD_MAX_REFINE_NUM >= (MMVD_MAX_REFINE_NUM >> MMVD_SIZE_SHIFT)/MMVD_BI_DIR))
#else
     if (rdModeList[uiMrgHADIdx].isMMVD && (uiMergeCand - (uiMergeCand / MMVD_MAX_REFINE_NUM)* MMVD_MAX_REFINE_NUM >= (MMVD_MAX_REFINE_NUM >> MMVD_SIZE_SHIFT)))
#endif
     {
	     continue;
     }
#endif
#if JVET_AG0135_AFFINE_CIIP
     if (uiNoResidualPass != 0 && rdModeList[uiMrgHADIdx].isCiipAffine)
     {
       rdModeList[uiMrgHADIdx].isCIIP = false;
       rdModeList[uiMrgHADIdx].isCiipAffine = false;
       rdModeList[uiMrgHADIdx].isAffine = true;
     }
#endif
      if (((uiNoResidualPass != 0) && candHasNoResidual[uiMrgHADIdx])
       || ( (uiNoResidualPass == 0) && bestIsSkip ) )
      {
        continue;
      }
      // first get merge candidates
      CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );

      partitioner.setCUData( cu );
      cu.slice            = tempCS->slice;
      cu.tileIdx          = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
      cu.skip             = false;
      cu.mmvdSkip = false;
#if JVET_AG0276_NLIC
      cu.altLMFlag = false;
      cu.altLMParaUnit.resetAltLinearModel();
#endif
#if INTER_LIC
      cu.licFlag          = false;
#if JVET_AD0213_LIC_IMP
      for (int list = 0; list < 2; list++)
      {
        for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
        {
          cu.licScale[list][comp] = MAX_INT;
          cu.licOffset[list][comp] = MAX_INT;
        }
      }
#endif
#endif
#if MERGE_ENC_OPT
      cu.affine = false;
#endif

      cu.geoFlag          = false;
    //cu.affine
      cu.predMode         = MODE_INTER;
      cu.chromaQpAdj      = m_cuChromaQpOffsetIdxPlus1;
      cu.qp               = encTestMode.qp;
      PredictionUnit &pu  = tempCS->addPU( cu, partitioner.chType );
#if AFFINE_MMVD && MERGE_ENC_OPT
      pu.afMmvdFlag       = false;
#endif
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
      pu.tmMergeFlag      = false;
#endif
#if JVET_X0049_ADAPT_DMVR
      pu.bmMergeFlag      = false;
      pu.bmDir            = 0;
#endif
#if MULTI_PASS_DMVR
      bool isDMVR         = false;
#endif
#if ENABLE_OBMC
      pu.ciipFlag = false;
#endif
#if JVET_X0141_CIIP_TIMD_TM
      pu.intraDir[0] = PLANAR_IDX;
#endif
      if (uiNoResidualPass == 0 && rdModeList[uiMrgHADIdx].isCIIP)
      {
        cu.mmvdSkip = false;
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
        pu.tmMergeFlag = rdModeList[uiMrgHADIdx].isTMMrg;
#endif
#if MULTI_HYP_PRED
        pu.ciipFlag = true;
#if JVET_AG0135_AFFINE_CIIP
        pu.ciipAffine = rdModeList[uiMrgHADIdx].isCiipAffine;
#endif
#if  JVET_X0141_CIIP_TIMD_TM && TM_MRG
        if (pu.tmMergeFlag)
        {
          ciipTmMrgCtx.setMergeInfo(pu, uiMergeCand);
        }
        else
#endif
#if JVET_AB0079_TM_BCW_MRG
#if JVET_AG0135_AFFINE_CIIP
        {
          if (pu.ciipAffine)
          {
            CHECK(uiMergeCand >= affineMergeCtx.numValidMergeCand, "");
            cu.affine = true;
            cu.imv = 0;
            pu.regularMergeFlag = false;
            pu.mergeFlag = true;
            pu.mergeIdx = uiMergeCand;
            pu.mmvdMergeFlag = false;
            pu.interDir = affineMergeCtx.interDirNeighbours[uiMergeCand];
            cu.affineType = affineMergeCtx.affineType[uiMergeCand];
            cu.bcwIdx = affineMergeCtx.bcwIdx[uiMergeCand];
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
            pu.colIdx = affineMergeCtx.colIdx[uiMergeCand];
#endif
#if INTER_LIC
            cu.licFlag = affineMergeCtx.licFlags[uiMergeCand];
#endif
#if JVET_AG0276_NLIC
            cu.altLMFlag = affineMergeCtx.altLMFlag[uiMergeCand];
            cu.altLMParaUnit = affineMergeCtx.altLMParaNeighbours[uiMergeCand];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
            cu.obmcFlag = affineMergeCtx.obmcFlags[uiMergeCand];
#endif
            pu.mv[0].setZero();
            pu.mv[1].setZero();
            pu.mvd[REF_PIC_LIST_0] = Mv();
            pu.mvd[REF_PIC_LIST_1] = Mv();
            pu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
            pu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
            pu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
            pu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;
            pu.mergeType = affineMergeCtx.mergeType[uiMergeCand];
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
            pu.affBMMergeFlag = false;
#endif
            if (pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
            {
              pu.refIdx[0] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0][0].refIdx;
              pu.refIdx[1] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1][0].refIdx;
              PU::spanMotionInfo(pu, mrgCtx
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                , pu.colIdx
#endif
              );
            }
            else
            {
              for (int i = 0; i < 2; i++)
              {
                pu.refIdx[i] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + i][0].refIdx;
                pu.mvAffi[i][0] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + i][0].mv;
                pu.mvAffi[i][1] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + i][1].mv;
                pu.mvAffi[i][2] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + i][2].mv;
              }

              PU::spanMotionInfo(pu);
            }
          }
          else
          {
            mrgCtxCiip.setMergeInfo(pu, uiMergeCand);
          }
      }
#else
          mrgCtxCiip.setMergeInfo(pu, uiMergeCand);
#endif
#else
        mergeCtx.setMergeInfo(pu, uiMergeCand);
#endif
#else
        mergeCtx.setMergeInfo(pu, uiMergeCand);
        pu.ciipFlag = true;
#endif
#if CIIP_PDPC
        pu.ciipPDPC = rdModeList[uiMrgHADIdx].isCiipPDPC;
#endif
        pu.regularMergeFlag = false;
#if JVET_X0141_CIIP_TIMD_TM && JVET_W0123_TIMD_FUSION
        pu.intraDir[0] = rdModeList[uiMrgHADIdx].intraMode;
#else
        pu.intraDir[0] = PLANAR_IDX;
#endif
        CHECK(pu.intraDir[0]<0 || pu.intraDir[0]>(NUM_LUMA_MODE - 1), "out of intra mode");
        pu.intraDir[1] = DM_CHROMA_IDX;
      }
      else if (rdModeList[uiMrgHADIdx].isMMVD)
      {
        cu.mmvdSkip = true;
        pu.regularMergeFlag = true;
#if JVET_W0090_ARMC_TM
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
        mergeCtxtmp.setMmvdMergeCandiInfo(pu, uiMergeCand, mmvdLUT[uiMergeCand]);
#else
        mergeCtxtmp.setMmvdMergeCandiInfo(pu, uiMergeCand);
#endif
#else
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
        mergeCtx.setMmvdMergeCandiInfo(pu, uiMergeCand, mmvdLUT[uiMergeCand]);
#else
        mergeCtx.setMmvdMergeCandiInfo(pu, uiMergeCand);
#endif
#endif
      }
#if MERGE_ENC_OPT
#if AFFINE_MMVD
      else if (rdModeList[uiMrgHADIdx].isAffineMmvd)
      {
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
        int uiMergeCandTemp = affMmvdLUT[uiMergeCand];
        int baseIdx = 
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
                      !pu.cs->sps->getUseTMMMVD() ?
                      (int)uiMergeCandTemp / ECM3_AF_MMVD_MAX_REFINE_NUM :
#endif
                      (int)uiMergeCandTemp / AF_MMVD_MAX_REFINE_NUM;
        int stepIdx = 
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
                      !pu.cs->sps->getUseTMMMVD() ?
                      (int)uiMergeCandTemp - baseIdx * ECM3_AF_MMVD_MAX_REFINE_NUM :
#endif
                      (int)uiMergeCandTemp - baseIdx * AF_MMVD_MAX_REFINE_NUM;
#else
        int baseIdx = (int)uiMergeCand / AF_MMVD_MAX_REFINE_NUM;
        int stepIdx = (int)uiMergeCand - baseIdx * AF_MMVD_MAX_REFINE_NUM;
#endif
        int dirIdx  = 
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                      !pu.cs->sps->getUseTMMMVD() ?
                      stepIdx % ECM3_AF_MMVD_OFFSET_DIR :
#endif
                      stepIdx % AF_MMVD_OFFSET_DIR;
            stepIdx = 
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                      !pu.cs->sps->getUseTMMMVD() ?
                      stepIdx / ECM3_AF_MMVD_OFFSET_DIR :
#endif
                      stepIdx / AF_MMVD_OFFSET_DIR;

        cu.affine           = true;
        cu.imv              = IMV_OFF;
        cu.mmvdSkip         = false;
        pu.regularMergeFlag = false;
        pu.mmvdMergeFlag    = false;
#if JVET_AG0276_LIC_FLAG_SIGNALING
        pu.affineOppositeLic = false;
#endif

        pu.mergeFlag      = true;
        pu.afMmvdFlag     = true;
        pu.afMmvdBaseIdx  = (uint8_t)baseIdx;
        pu.afMmvdDir      = (uint8_t)dirIdx;
        pu.afMmvdStep     = (uint8_t)stepIdx;
        pu.mergeIdx       = (uint8_t)(baseIdx + afMmvdBaseIdxToMergeIdxOffset);
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
        pu.afMmvdMergeIdx = uiMergeCand;
#endif
#if JVET_W0090_ARMC_TM
        pu.mergeType = affineMergeCtxTmp.mergeType[pu.mergeIdx];
#if JVET_AG0276_NLIC
        pu.cu->altLMFlag = affineMergeCtxTmp.altLMFlag[pu.mergeIdx];
        pu.cu->altLMParaUnit = affineMergeCtxTmp.altLMParaNeighbours[pu.mergeIdx];
#endif
#if INTER_LIC
        pu.cu->licFlag = affineMergeCtxTmp.licFlags[pu.mergeIdx];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
        pu.cu->obmcFlag = affineMergeCtxTmp.obmcFlags[pu.mergeIdx];
#endif
        pu.interDir = affineMergeCtxTmp.interDirNeighbours[pu.mergeIdx];
        pu.cu->affineType = affineMergeCtxTmp.affineType[pu.mergeIdx];
        pu.cu->bcwIdx = affineMergeCtxTmp.bcwIdx[pu.mergeIdx];
        pu.mmvdMergeFlag = false;
        pu.ciipFlag = false;
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
        pu.affBMMergeFlag = false;
#endif
        CHECK(pu.mergeIdx >= affineMergeCtxTmp.numValidMergeCand, "Invalid merge index for AffineMMVD");

        MvField mvfMmvd[2][3];
        PU::getAfMmvdMvf(pu, affineMergeCtxTmp, mvfMmvd, pu.mergeIdx, pu.afMmvdStep, pu.afMmvdDir);
#else
        pu.mergeType      = affineMergeCtx.mergeType         [pu.mergeIdx];
#if INTER_LIC
        pu.cu->licFlag    = affineMergeCtx.licFlags          [pu.mergeIdx];
#endif
        pu.interDir       = affineMergeCtx.interDirNeighbours[pu.mergeIdx];
        pu.cu->affineType = affineMergeCtx.affineType        [pu.mergeIdx];
        pu.cu->bcwIdx     = affineMergeCtx.bcwIdx            [pu.mergeIdx];
        pu.mmvdMergeFlag  = false;
        pu.ciipFlag       = false;

        CHECK(pu.mergeIdx >= affineMergeCtx.numValidMergeCand, "Invalid merge index for AffineMMVD");

        MvField mvfMmvd[2][3];
        PU::getAfMmvdMvf(pu, affineMergeCtx, mvfMmvd, pu.mergeIdx, pu.afMmvdStep, pu.afMmvdDir);
#endif

        for (int i = 0; i < 2; i++)
        {
          pu.refIdx[i] = mvfMmvd[i][0].refIdx;
          pu.mvAffi[i][0] = mvfMmvd[i][0].mv;
          pu.mvAffi[i][1] = mvfMmvd[i][1].mv;
          pu.mvAffi[i][2] = mvfMmvd[i][2].mv;
        }

        PU::spanMotionInfo(pu);
      }
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
      else if (rdModeList[uiMrgHADIdx].isAffBMMrg)
      {
        if (rdModeList[uiMrgHADIdx].affBMDir == 1)
        {
          CHECK(uiMergeCand >= affineBMMergeL0.numValidMergeCand, "");
          cu.mmvdSkip = false;
          cu.affine = true;
#if JVET_AG0276_LIC_FLAG_SIGNALING
          pu.affineOppositeLic = false;
#endif
          cu.imv = 0;
          pu.afMmvdFlag = false;
          pu.regularMergeFlag = false;
          pu.mergeFlag = true;
          pu.mergeIdx = uiMergeCand;
          pu.mmvdMergeFlag = false;
          pu.interDir = affineBMMergeL0.interDirNeighbours[uiMergeCand];
          cu.affineType = affineBMMergeL0.affineType[uiMergeCand];
          cu.bcwIdx = affineBMMergeL0.bcwIdx[uiMergeCand];
#if JVET_AG0276_NLIC
          cu.altLMFlag = affineBMMergeL0.altLMFlag[uiMergeCand];
          cu.altLMParaUnit = affineBMMergeL0.altLMParaNeighbours[uiMergeCand];
#endif
#if INTER_LIC
          cu.licFlag = affineBMMergeL0.licFlags[uiMergeCand];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
          cu.obmcFlag = affineBMMergeL0.obmcFlags[uiMergeCand];
#endif
          pu.affBMMergeFlag = true;
          pu.affBMDir = rdModeList[uiMrgHADIdx].affBMDir;
          pu.mv[0].setZero();
          pu.mv[1].setZero();
          pu.mvd[REF_PIC_LIST_0] = Mv();
          pu.mvd[REF_PIC_LIST_1] = Mv();
          pu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
          pu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
          pu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
          pu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;
          pu.mergeType = affineBMMergeL0.mergeType[uiMergeCand];
          for (int i = 0; i < 2; i++)
          {
            pu.refIdx[i] = affineBMMergeL0.mvFieldNeighbours[(uiMergeCand << 1) + i][0].refIdx;
            pu.mvAffi[i][0] = affineBMMergeL0.mvFieldNeighbours[(uiMergeCand << 1) + i][0].mv;
            pu.mvAffi[i][1] = affineBMMergeL0.mvFieldNeighbours[(uiMergeCand << 1) + i][1].mv;
            pu.mvAffi[i][2] = affineBMMergeL0.mvFieldNeighbours[(uiMergeCand << 1) + i][2].mv;
          }

          PU::spanMotionInfo(pu);
        }
        else if (rdModeList[uiMrgHADIdx].affBMDir == 2)
        {
          CHECK(uiMergeCand >= affineBMMergeL1.numValidMergeCand, "");
          cu.mmvdSkip = false;
          cu.affine = true;
#if JVET_AG0276_LIC_FLAG_SIGNALING
          pu.affineOppositeLic = false;
#endif
          cu.imv = 0;
          pu.afMmvdFlag = false;
          pu.regularMergeFlag = false;
          pu.mergeFlag = true;
          pu.mergeIdx = uiMergeCand;
          pu.mmvdMergeFlag = false;
          pu.interDir = affineBMMergeL1.interDirNeighbours[uiMergeCand];
          cu.affineType = affineBMMergeL1.affineType[uiMergeCand];
          cu.bcwIdx = affineBMMergeL1.bcwIdx[uiMergeCand];
#if JVET_AG0276_NLIC
          cu.altLMFlag = affineBMMergeL1.altLMFlag[uiMergeCand];
          cu.altLMParaUnit = affineBMMergeL1.altLMParaNeighbours[uiMergeCand];
#endif
#if INTER_LIC
          cu.licFlag = affineBMMergeL1.licFlags[uiMergeCand];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
          cu.obmcFlag = affineBMMergeL1.obmcFlags[uiMergeCand];
#endif
          pu.affBMMergeFlag = true;
          pu.affBMDir = rdModeList[uiMrgHADIdx].affBMDir;
          pu.mv[0].setZero();
          pu.mv[1].setZero();
          pu.mvd[REF_PIC_LIST_0] = Mv();
          pu.mvd[REF_PIC_LIST_1] = Mv();
          pu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
          pu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
          pu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
          pu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;
          pu.mergeType = affineBMMergeL1.mergeType[uiMergeCand];
          for (int i = 0; i < 2; i++)
          {
            pu.refIdx[i] = affineBMMergeL1.mvFieldNeighbours[(uiMergeCand << 1) + i][0].refIdx;
            pu.mvAffi[i][0] = affineBMMergeL1.mvFieldNeighbours[(uiMergeCand << 1) + i][0].mv;
            pu.mvAffi[i][1] = affineBMMergeL1.mvFieldNeighbours[(uiMergeCand << 1) + i][1].mv;
            pu.mvAffi[i][2] = affineBMMergeL1.mvFieldNeighbours[(uiMergeCand << 1) + i][2].mv;
          }

          PU::spanMotionInfo(pu);
        }
        else
        {
          CHECK(true, "wrong affine BM dir!");
        }
      }
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
      else if (rdModeList[uiMrgHADIdx].isAffOppositeLic)
      {
        cu.mmvdSkip = false;
        cu.affine = true;
        pu.affineOppositeLic = true;
        cu.imv = 0;
        pu.regularMergeFlag = false;
        pu.mergeFlag = true;
        pu.mergeIdx = uiMergeCand;
        pu.mmvdMergeFlag = false;
        pu.interDir = affineMergeCtxOppositeLic.interDirNeighbours[uiMergeCand];
        cu.affineType = affineMergeCtxOppositeLic.affineType[uiMergeCand];
        cu.bcwIdx = affineMergeCtxOppositeLic.bcwIdx[uiMergeCand];
#if JVET_AG0276_NLIC
        cu.altLMFlag = affineMergeCtxOppositeLic.altLMFlag[uiMergeCand];
        cu.altLMParaUnit = affineMergeCtxOppositeLic.altLMParaNeighbours[uiMergeCand];
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
        pu.colIdx = affineMergeCtxOppositeLic.colIdx[uiMergeCand];
#endif
#if INTER_LIC
        cu.licFlag = affineMergeCtxOppositeLic.licFlags[uiMergeCand];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
        cu.obmcFlag = affineMergeCtxOppositeLic.obmcFlags[uiMergeCand];
#endif
        pu.mv[0].setZero();
        pu.mv[1].setZero();
        pu.mvd[REF_PIC_LIST_0] = Mv();
        pu.mvd[REF_PIC_LIST_1] = Mv();
        pu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
        pu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
        pu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
        pu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;
        pu.mergeType = affineMergeCtxOppositeLic.mergeType[uiMergeCand];
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
        pu.affBMMergeFlag = false;
#endif

        for (int i = 0; i < 2; i++)
        {
          pu.refIdx[i] = affineMergeCtxOppositeLic.mvFieldNeighbours[(uiMergeCand << 1) + i][0].refIdx;
          pu.mvAffi[i][0] = affineMergeCtxOppositeLic.mvFieldNeighbours[(uiMergeCand << 1) + i][0].mv;
          pu.mvAffi[i][1] = affineMergeCtxOppositeLic.mvFieldNeighbours[(uiMergeCand << 1) + i][1].mv;
          pu.mvAffi[i][2] = affineMergeCtxOppositeLic.mvFieldNeighbours[(uiMergeCand << 1) + i][2].mv;
        }
        PU::spanMotionInfo(pu);
      }
#endif
      else if (rdModeList[uiMrgHADIdx].isAffine)
      {
        CHECK(uiMergeCand >= affineMergeCtx.numValidMergeCand, "");
        cu.mmvdSkip = false;
        cu.affine = true;
#if JVET_AG0276_LIC_FLAG_SIGNALING
        pu.affineOppositeLic = false;
#endif
        cu.imv = 0;
        pu.regularMergeFlag = false;
        pu.mergeFlag = true;
        pu.mergeIdx = uiMergeCand;
        pu.mmvdMergeFlag = false;
        pu.interDir = affineMergeCtx.interDirNeighbours[uiMergeCand];
        cu.affineType = affineMergeCtx.affineType[uiMergeCand];
        cu.bcwIdx = affineMergeCtx.bcwIdx[uiMergeCand];
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
        pu.colIdx = affineMergeCtx.colIdx[uiMergeCand];
#endif
#if JVET_AG0276_NLIC
        cu.altLMFlag = affineMergeCtx.altLMFlag[uiMergeCand];
        cu.altLMParaUnit = affineMergeCtx.altLMParaNeighbours[uiMergeCand];
#endif
#if INTER_LIC
        cu.licFlag = affineMergeCtx.licFlags[uiMergeCand];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
        cu.obmcFlag = affineMergeCtx.obmcFlags[uiMergeCand];
#endif
        pu.mv[0].setZero();
        pu.mv[1].setZero();
        pu.mvd[REF_PIC_LIST_0] = Mv();
        pu.mvd[REF_PIC_LIST_1] = Mv();
        pu.mvpIdx[REF_PIC_LIST_0] = NOT_VALID;
        pu.mvpIdx[REF_PIC_LIST_1] = NOT_VALID;
        pu.mvpNum[REF_PIC_LIST_0] = NOT_VALID;
        pu.mvpNum[REF_PIC_LIST_1] = NOT_VALID;
        pu.mergeType = affineMergeCtx.mergeType[uiMergeCand];
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
        pu.affBMMergeFlag = false;
#endif
        if (pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
        {
          pu.refIdx[0] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0][0].refIdx;
          pu.refIdx[1] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1][0].refIdx;
          PU::spanMotionInfo(pu, mrgCtx
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
            , pu.colIdx
#endif
          );
        }
        else
        {
          for (int i = 0; i < 2; i++)
          {
            pu.refIdx[i] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + i][0].refIdx;
            pu.mvAffi[i][0] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + i][0].mv;
            pu.mvAffi[i][1] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + i][1].mv;
            pu.mvAffi[i][2] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + i][2].mv;
          }

          PU::spanMotionInfo(pu);
        }
      }
#if JVET_AG0276_LIC_FLAG_SIGNALING
#if TM_MRG && MERGE_ENC_OPT
#if JVET_X0141_CIIP_TIMD_TM
      else if (rdModeList[uiMrgHADIdx].isTMMrg && rdModeList[uiMrgHADIdx].isTMMrgOppositeLic && !rdModeList[uiMrgHADIdx].isCIIP)
#else
      else if (rdModeList[uiMrgHADIdx].isTMMrg)
#endif
      {
        cu.mmvdSkip = false;
        pu.regularMergeFlag = true;
        pu.tmMergeFlag = true;
        pu.tmMergeFlagOppositeLic = true;
#if JVET_X0141_CIIP_TIMD_TM
        pu.ciipFlag = false;
#endif
        tmMrgCtxOppositeLic.setMergeInfo(pu, uiMergeCand);
#if MULTI_PASS_DMVR
        if (applyBDMVR4TMOppositeLic[uiMergeCand])
        {
          isDMVR = true;
          pu.bdmvrRefine = true;
          m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4TMOPPOSITELIC[uiMergeCand << 1], m_mvBufBDMVR4TMOPPOSITELIC[(uiMergeCand << 1) + 1]);
        }
#endif
      }
#endif
#endif
#if TM_MRG && MERGE_ENC_OPT
#if JVET_X0141_CIIP_TIMD_TM
      else if (rdModeList[uiMrgHADIdx].isTMMrg && !rdModeList[uiMrgHADIdx].isCIIP)
#else
      else if (rdModeList[uiMrgHADIdx].isTMMrg)
#endif
      {
        cu.mmvdSkip         = false;
        pu.regularMergeFlag = true;
        pu.tmMergeFlag      = true;
#if JVET_AG0276_LIC_FLAG_SIGNALING
        pu.tmMergeFlagOppositeLic = false;
#endif
#if JVET_X0141_CIIP_TIMD_TM
        pu.ciipFlag = false;
#endif
        tmMrgCtx.setMergeInfo(pu, uiMergeCand);
#if MULTI_PASS_DMVR
        if (applyBDMVR4TM[uiMergeCand])
        {
          isDMVR = true;
          pu.bdmvrRefine = true;
          m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4TM[uiMergeCand << 1], m_mvBufBDMVR4TM[(uiMergeCand << 1) + 1]);
        }
#endif
      }
#endif
#if JVET_X0049_ADAPT_DMVR
      else if (rdModeList[uiMrgHADIdx].isBMMrg)
      {
        cu.mmvdSkip = false;
        pu.regularMergeFlag = true;
        pu.bmMergeFlag = true;
        pu.bmDir = rdModeList[uiMrgHADIdx].bmDir;
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
        if (pu.bmDir == 1)
        {
          bmMrgCtx.setMergeInfo(pu, uiMergeCand);
        }
        else
        {
          bmMrgCtxDir2.setMergeInfo(pu, uiMergeCand);
        }
#else
        bmMrgCtx.setMergeInfo(pu, uiMergeCand);
#endif
        if (applyBDMVR4BM[uiMergeCand])
        {
          isDMVR = true;
          pu.bdmvrRefine = true;
          m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4BM[uiMergeCand << 1], m_mvBufBDMVR4BM[(uiMergeCand << 1) + 1]);
        }
      }
#endif
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
      else if (rdModeList[uiMrgHADIdx].isOppositeLic)
      {
        pu.mergeOppositeLic = true;
        cu.mmvdSkip = false;
        pu.regularMergeFlag = true;
        pu.bmMergeFlag = false;
        pu.affBMMergeFlag = false;
        pu.cu->geoFlag = false;
        mergeCtxOppositeLic.setMergeInfo(pu, uiMergeCand);
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
        pu.ciipFlag = false;
        pu.tmMergeFlag = false;
#endif
#if MULTI_PASS_DMVR
        if (applyBDMVROppositeLic[uiMergeCand])
        {
          isDMVR = true;
          pu.bdmvrRefine = true;
          m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4OPPOSITELIC[uiMergeCand << 1], m_mvBufBDMVR4OPPOSITELIC[(uiMergeCand << 1) + 1]);
        }
#endif
      }
#endif
      else
      {
#if JVET_AG0276_LIC_FLAG_SIGNALING
        pu.mergeOppositeLic = false;
#endif
        cu.mmvdSkip = false;
        pu.regularMergeFlag = true;
        mergeCtx.setMergeInfo(pu, uiMergeCand);
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
        pu.ciipFlag = false;
        pu.tmMergeFlag = false;
#endif
#if MULTI_PASS_DMVR
        if (applyBDMVR[uiMergeCand])
        {
          isDMVR = true;
          pu.bdmvrRefine = true;
          m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR[uiMergeCand << 1], m_mvBufBDMVR[(uiMergeCand << 1) + 1]);
        }
#endif
      }
#if MERGE_ENC_OPT
      if (!rdModeList[uiMrgHADIdx].isAffine && !rdModeList[uiMrgHADIdx].isGeo)
#endif
#if MULTI_PASS_DMVR
      if( !pu.bdmvrRefine )
      {
        PU::spanMotionInfo(pu, mergeCtx
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
          , pu.colIdx
#endif
        );
      }
#else
        PU::spanMotionInfo(pu, mergeCtx
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
          , pu.colIdx
#endif
        );
#endif

      if( m_pcEncCfg->getMCTSEncConstraint() )
      {
#if !MULTI_PASS_DMVR
        bool isDMVR = PU::checkDMVRCondition( pu );
#endif
        if( ( isDMVR && MCTSHelper::isRefBlockAtRestrictedTileBoundary( pu ) ) || ( !isDMVR && !( MCTSHelper::checkMvBufferForMCTSConstraint( pu ) ) ) )
        {
          // Do not use this mode
          tempCS->initStructData( encTestMode.qp );
          continue;
        }
      }
#if MERGE_ENC_OPT
#if JVET_AG0276_LIC_BDOF_BDMVR
      if (mrgTempBufSet && uiMrgHADIdx < MMVD_MRG_MAX_RD_NUM && !(pu.bdmvrRefine == true && pu.cu->licFlag == true))
#else
      if (mrgTempBufSet && uiMrgHADIdx < MMVD_MRG_MAX_RD_NUM)
#endif
#else
      if( mrgTempBufSet )
#endif
      {
#if !MULTI_PASS_DMVR
        {
          int dx, dy, i, j, num = 0;
          dy = std::min<int>(pu.lumaSize().height, DMVR_SUBCU_HEIGHT);
          dx = std::min<int>(pu.lumaSize().width, DMVR_SUBCU_WIDTH);
          if (PU::checkDMVRCondition(pu))
          {
            for (i = 0; i < (pu.lumaSize().height); i += dy)
            {
              for (j = 0; j < (pu.lumaSize().width); j += dx)
              {
                pu.mvdL0SubPu[num] = refinedMvdL0[num][uiMergeCand];
                num++;
              }
            }
          }
        }
#endif
        if (pu.ciipFlag)
        {
#if CIIP_PDPC
          uint32_t bufIdx = pu.ciipPDPC ? 1 : 0;
#else
          uint32_t bufIdx = 0;
#endif
#if JVET_X0090_CIIP_FIX
#if JVET_AG0135_AFFINE_CIIP
          if (pu.ciipAffine && rdModeList[uiMrgHADIdx].isAffBMMrg == false && m_doEncAffineBDOF[uiMergeCand] == true)
          {
            tempCS->getPredBuf().copyFrom(acMergeAffineBuffer[uiMergeCand]);
            PU::setAffineBdofRefinedMotion(pu, m_mvBufEncAffineBDOF[uiMergeCand]);
          }
          else
          {
#endif
            m_pcInterSearch->motionCompensation(pu);
#if JVET_AG0135_AFFINE_CIIP
          }
#endif
#if JVET_AD0213_LIC_IMP
          if (cu.licFlag)
          {
            bool alwCond = ((slice.getPOC() - slice.getRefPOC(REF_PIC_LIST_0, 0)) == 1) && slice.getCheckLDC();
            for (int list = 0; list < 2; list++)
            {
              if (pu.refIdx[list] >= 0)
              {
                for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
                {
                  CHECK(cu.licScale[list][comp] != MAX_INT || cu.licScale[list][comp] != MAX_INT, "invalid lic parameters for ciip mode");
                  if (alwCond)
                  {
                    m_pcInterSearch->setLicParam(list, comp, pu.cu->licScale[list][comp], pu.cu->licOffset[list][comp]);
                  }
                  else
                  {
                    cu.licScale[list][comp] = 32;
                    cu.licOffset[list][comp] = 0;
                  }
                }
              }
            }
          }
#endif
#if ENABLE_OBMC
          cu.isobmcMC = true;
#if JVET_AG0135_AFFINE_CIIP
          if (!pu.ciipAffine)
          {
#endif
            cu.obmcFlag = true;
#if JVET_AG0135_AFFINE_CIIP
          }
#endif
          m_pcInterSearch->subBlockOBMC(pu);
          cu.isobmcMC = false;
#endif
#if JVET_AG0135_AFFINE_CIIP
          if (pu.ciipAffine)
          {
            if (cu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
            {
              m_pcIntraSearch->geneWeightedCIIPAffinePred<true>(COMPONENT_Y, tempCS->getPredBuf(pu).Y(), pu, tempCS->getPredBuf(pu).Y(), m_ciipBuffer[bufIdx].getBuf(localUnitArea.Y()), m_pcReshape->getFwdLUT().data());
            }
            else
            {
              m_pcIntraSearch->geneWeightedCIIPAffinePred<false>(COMPONENT_Y, tempCS->getPredBuf(pu).Y(), pu, tempCS->getPredBuf(pu).Y(), m_ciipBuffer[bufIdx].getBuf(localUnitArea.Y()));
            }
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
            if (isChromaEnabled(pu.chromaFormat))
#else
            if (isChromaEnabled(pu.chromaFormat) && pu.chromaSize().width > 2)
#endif
            {
              m_pcIntraSearch->geneWeightedCIIPAffinePred<false>(COMPONENT_Cb, tempCS->getPredBuf(pu).Cb(), pu, tempCS->getPredBuf(pu).Cb(), m_ciipBuffer[bufIdx].getBuf(localUnitArea.Cb()));
              m_pcIntraSearch->geneWeightedCIIPAffinePred<false>(COMPONENT_Cr, tempCS->getPredBuf(pu).Cr(), pu, tempCS->getPredBuf(pu).Cr(), m_ciipBuffer[bufIdx].getBuf(localUnitArea.Cr()));
            }
          }
          else
          {
#endif
            if (cu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
            {
              m_pcIntraSearch->geneWeightedPred<true>(COMPONENT_Y, tempCS->getPredBuf(pu).Y(), pu, tempCS->getPredBuf(pu).Y(), m_ciipBuffer[bufIdx].getBuf(localUnitArea.Y()), m_pcReshape->getFwdLUT().data());
            }
            else
            {
              m_pcIntraSearch->geneWeightedPred<false>(COMPONENT_Y, tempCS->getPredBuf(pu).Y(), pu, tempCS->getPredBuf(pu).Y(), m_ciipBuffer[bufIdx].getBuf(localUnitArea.Y()));
            }
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
            if (isChromaEnabled(pu.chromaFormat))
#else
            if (isChromaEnabled(pu.chromaFormat) && pu.chromaSize().width > 2)
#endif
            {
              m_pcIntraSearch->geneWeightedPred<false>(COMPONENT_Cb, tempCS->getPredBuf(pu).Cb(), pu, tempCS->getPredBuf(pu).Cb(), m_ciipBuffer[bufIdx].getBuf(localUnitArea.Cb()));
              m_pcIntraSearch->geneWeightedPred<false>(COMPONENT_Cr, tempCS->getPredBuf(pu).Cr(), pu, tempCS->getPredBuf(pu).Cr(), m_ciipBuffer[bufIdx].getBuf(localUnitArea.Cr()));
            }
#if JVET_AG0135_AFFINE_CIIP
          }
#endif
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
          else if (isChromaEnabled(pu.chromaFormat))
          {
            tempCS->getPredBuf().Cb().copyFrom(tempCS->getPredBuf(pu).Cb());
            tempCS->getPredBuf().Cr().copyFrom(tempCS->getPredBuf(pu).Cr());
          }
#endif
#else
          // Luma CIIP was already done in SATD check stage and stored
          tempCS->getPredBuf().Y().copyFrom( acMergeTempBuffer[uiMrgHADIdx]->Y() );

#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
          if (isChromaEnabled(pu.chromaFormat))
#else
          if( isChromaEnabled( pu.chromaFormat ) && pu.chromaSize().width > 2 )
#endif
          {
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
            if (pu.tmMergeFlag)
            {
              m_pcIntraSearch->geneWeightedPred<false>(COMPONENT_Cb, tempCS->getPredBuf(pu).Cb(), pu, acTmMergeTmpBuffer[uiMergeCand].Cb(), m_ciipBuffer[bufIdx].getBuf(localUnitArea.Cb()));
              m_pcIntraSearch->geneWeightedPred<false>(COMPONENT_Cr, tempCS->getPredBuf(pu).Cr(), pu, acTmMergeTmpBuffer[uiMergeCand].Cr(), m_ciipBuffer[bufIdx].getBuf(localUnitArea.Cr()));
            }
            else
            {
              m_pcIntraSearch->geneWeightedPred<false>(COMPONENT_Cb, tempCS->getPredBuf(pu).Cb(), pu, acMergeTmpBuffer[uiMergeCand].Cb(), m_ciipBuffer[bufIdx].getBuf(localUnitArea.Cb()));
              m_pcIntraSearch->geneWeightedPred<false>(COMPONENT_Cr, tempCS->getPredBuf(pu).Cr(), pu, acMergeTmpBuffer[uiMergeCand].Cr(), m_ciipBuffer[bufIdx].getBuf(localUnitArea.Cr()));
            }
#else
            m_pcIntraSearch->geneWeightedPred<false>( COMPONENT_Cb, tempCS->getPredBuf( pu ).Cb(), pu, acMergeTmpBuffer[uiMergeCand].Cb(), m_ciipBuffer[bufIdx].getBuf( localUnitArea.Cb() ) );
            m_pcIntraSearch->geneWeightedPred<false>( COMPONENT_Cr, tempCS->getPredBuf( pu ).Cr(), pu, acMergeTmpBuffer[uiMergeCand].Cr(), m_ciipBuffer[bufIdx].getBuf( localUnitArea.Cr() ) );
#endif
          }
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
          else if (isChromaEnabled(pu.chromaFormat))
          {
            tempCS->getPredBuf().Cb().copyFrom(acMergeTmpBuffer[uiMergeCand].Cb());
            tempCS->getPredBuf().Cr().copyFrom(acMergeTmpBuffer[uiMergeCand].Cr());
          }
#endif
#endif
        }
        else
        {
          if (rdModeList[uiMrgHADIdx].isMMVD
#if AFFINE_MMVD && MERGE_ENC_OPT
            || rdModeList[uiMrgHADIdx].isAffineMmvd
#endif
            )
          {
            pu.mmvdEncOptMode = 0;
            m_pcInterSearch->motionCompensation(pu);
#if JVET_AD0213_LIC_IMP
            if (pu.cu->licFlag)
            {
              for (int list = 0; list < 2; list++)
              {
                if (pu.refIdx[list] >= 0)
                {
                  for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
                  {
                    m_pcInterSearch->setLicParam(list, comp, pu.cu->licScale[list][comp], pu.cu->licOffset[list][comp]);
                  }
                }
              }
            }
#endif
          }
#if MERGE_ENC_OPT
          else if (uiNoResidualPass != 0 && rdModeList[uiMrgHADIdx].isCIIP)
          {
            // perform regular MC instead, i.e. test skip mode
            pu.mvRefine = true;
            m_pcInterSearch->motionCompensation(pu);
#if JVET_AD0213_LIC_IMP
            if (pu.cu->licFlag)
            {
              for (int list = 0; list < 2; list++)
              {
                if (pu.refIdx[list] >= 0)
                {
                  for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
                  {
                    m_pcInterSearch->setLicParam(list, comp, pu.cu->licScale[list][comp], pu.cu->licOffset[list][comp]);
                  }
                }
              }
            }
#endif
            pu.mvRefine = false;
#if MULTI_PASS_DMVR
            if (!rdModeList[uiMrgHADIdx].isAffine && !rdModeList[uiMrgHADIdx].isGeo && pu.bdmvrRefine)
            {
#if JVET_AG0276_LIC_FLAG_SIGNALING
#if TM_MRG
#if JVET_X0141_CIIP_TIMD_TM
              if (pu.tmMergeFlag && pu.tmMergeFlagOppositeLic && !rdModeList[uiMrgHADIdx].isCIIP)
#else
              if (pu.tmMergeFlag)
#endif
              {
                PU::spanMotionInfo(pu, mergeCtx,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                  pu.colIdx,
#endif
                  m_mvBufBDMVR4TMOPPOSITELIC[uiMergeCand << 1], m_mvBufBDMVR4TMOPPOSITELIC[(uiMergeCand << 1) + 1], m_pcInterSearch->getBdofSubPuMvOffset());
              }
              else
#endif
#endif
#if TM_MRG
#if JVET_X0141_CIIP_TIMD_TM
              if (pu.tmMergeFlag && !rdModeList[uiMrgHADIdx].isCIIP)
#else
              if ( pu.tmMergeFlag )
#endif
              {
                PU::spanMotionInfo(pu, mergeCtx,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                  pu.colIdx,
#endif
                  m_mvBufBDMVR4TM[uiMergeCand << 1], m_mvBufBDMVR4TM[(uiMergeCand << 1) + 1], m_pcInterSearch->getBdofSubPuMvOffset());
              }
              else
#endif
#if JVET_X0049_ADAPT_DMVR
              if( pu.bmMergeFlag ) 
              {
                PU::spanMotionInfo(pu, bmMrgCtx,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                  pu.colIdx,
#endif
                  m_mvBufBDMVR4BM[uiMergeCand << 1], m_mvBufBDMVR4BM[(uiMergeCand << 1) + 1], m_pcInterSearch->getBdofSubPuMvOffset());
              }
              else
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
                if (pu.mergeOppositeLic)
                {
#if JVET_AG0276_LIC_BDOF_BDMVR
                  if (pu.cu->licFlag == true)
                  {
                    memset((void*)m_pcInterSearch->getBdofSubPuMvOffset(), 0, BDOF_SUBPU_MAX_NUM * sizeof(Mv));
                  }
#endif

                  PU::spanMotionInfo(pu, mergeCtxOppositeLic,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                    pu.colIdx,
#endif
                    m_mvBufBDMVR4OPPOSITELIC[uiMergeCand << 1], m_mvBufBDMVR4OPPOSITELIC[(uiMergeCand << 1) + 1], m_pcInterSearch->getBdofSubPuMvOffset());

                }
                else
#endif
                PU::spanMotionInfo(pu, mergeCtx,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                  pu.colIdx,
#endif
                  m_mvBufBDMVR[uiMergeCand << 1], m_mvBufBDMVR[(uiMergeCand << 1) + 1], m_pcInterSearch->getBdofSubPuMvOffset());
            }
#endif
          }
#else
          else if (uiNoResidualPass != 0 && rdModeList[uiMrgHADIdx].isCIIP)
          {
            tempCS->getPredBuf().copyFrom(acMergeBuffer[uiMergeCand]);
#if MULTI_PASS_DMVR
            if (pu.bdmvrRefine)
            {
#if TM_MRG
              if( pu.tmMergeFlag )
              {
                PU::spanMotionInfo( pu, mergeCtx, m_mvBufBDMVR4TM[uiMergeCand << 1], m_mvBufBDMVR4TM[( uiMergeCand << 1 ) + 1], m_mvBufEncBDOF4TM[uiMergeCand] );
              }
              else
#endif
                PU::spanMotionInfo(pu, mergeCtx, m_mvBufBDMVR[uiMergeCand << 1], m_mvBufBDMVR[(uiMergeCand << 1) + 1], m_mvBufEncBDOF[uiMergeCand]);
            }
#endif
          }
#endif
#if MERGE_ENC_OPT
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
          else if (rdModeList[uiMrgHADIdx].isAffBMMrg == true && m_doEncAffineBmBDOF[(uiMergeCand << 1) + rdModeList[uiMrgHADIdx].affBMDir - 1] == true)
          {
            tempCS->getPredBuf().copyFrom(*acMergeTempBuffer[uiMrgHADIdx]);
            PU::setAffineBdofRefinedMotion(pu, m_mvBufEncAffineBmBDOF[(uiMergeCand << 1) + rdModeList[uiMrgHADIdx].affBMDir - 1]);
          }
#if JVET_AG0276_LIC_FLAG_SIGNALING
          else if (rdModeList[uiMrgHADIdx].isAffine == true && rdModeList[uiMrgHADIdx].isAffBMMrg == false && ((!pu.affineOppositeLic && m_doEncAffineBDOF[uiMergeCand] == true) || (pu.affineOppositeLic && m_doEncAffineBDOFOppositeLic[uiMergeCand] == true)))
#else
          else if (rdModeList[uiMrgHADIdx].isAffine == true && rdModeList[uiMrgHADIdx].isAffBMMrg == false && m_doEncAffineBDOF[uiMergeCand] == true)
#endif
          {
#if JVET_AG0135_AFFINE_CIIP
            tempCS->getPredBuf().copyFrom(acMergeAffineBuffer[uiMergeCand]);
#else
            tempCS->getPredBuf().copyFrom(*acMergeTempBuffer[uiMrgHADIdx]);
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
            if (pu.affineOppositeLic)
            {
              PU::setAffineBdofRefinedMotion(pu, m_mvBufEncAffineBDOFOppositeLic[uiMergeCand]);
            }
            else
            {
              PU::setAffineBdofRefinedMotion(pu, m_mvBufEncAffineBDOF[uiMergeCand]);
            }
#else
            PU::setAffineBdofRefinedMotion(pu, m_mvBufEncAffineBDOF[uiMergeCand]);
#endif
          }
#endif
          else if (rdModeList[uiMrgHADIdx].isAffine)
          {
#if !JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
            tempCS->getPredBuf().copyFrom(*acMergeTempBuffer[uiMrgHADIdx], true);
#endif
#if JVET_Z0136_OOB
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
            if (pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
            {
#if JVET_AG0135_AFFINE_CIIP
              tempCS->getPredBuf().copyFrom(acMergeAffineBuffer[uiMergeCand]);
#else
              tempCS->getPredBuf().copyFrom(*acMergeTempBuffer[uiMrgHADIdx]);
#endif
            }
            else
#endif
            m_pcInterSearch->motionCompensation(pu, REF_PIC_LIST_X, true, true);
#else
#if JVET_AD0213_LIC_IMP
            if (pu.cu->LICFlag)
            {
              m_pcInterSearch->motionCompensation(pu, REF_PIC_LIST_X, true, true);
            }
            else
#endif
            m_pcInterSearch->motionCompensation(pu, REF_PIC_LIST_X, false, true);
#endif
#if JVET_AD0213_LIC_IMP
            if (pu.cu->licFlag)
            {
              for (int list = 0; list < 2; list++)
              {
                if (pu.refIdx[list] >= 0)
                {
                  for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
                  {
                    m_pcInterSearch->setLicParam(list, comp, pu.cu->licScale[list][comp], pu.cu->licOffset[list][comp]);
                  }
                }
              }
            }
#endif
          }
#endif
          else
          {
#if JVET_AD0213_LIC_IMP
          if (pu.cu->licFlag)
          {
            m_pcInterSearch->motionCompensation(pu);
            for (int list = 0; list < 2; list++)
            {
              if (pu.refIdx[list] >= 0)
              {
                for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
                {
                  m_pcInterSearch->setLicParam(list, comp, pu.cu->licScale[list][comp], pu.cu->licOffset[list][comp]);
                }
              }
            }
          }
          else
#endif
            tempCS->getPredBuf().copyFrom(*acMergeTempBuffer[uiMrgHADIdx]);
#if MULTI_PASS_DMVR
#if MERGE_ENC_OPT
            if (!rdModeList[uiMrgHADIdx].isAffine && !rdModeList[uiMrgHADIdx].isGeo && pu.bdmvrRefine)
#else
            if(pu.bdmvrRefine)
#endif
            {
#if JVET_AG0276_LIC_FLAG_SIGNALING
#if TM_MRG
              if (pu.tmMergeFlag && pu.tmMergeFlagOppositeLic)
              {
                PU::spanMotionInfo(pu, mergeCtx,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                  pu.colIdx,
#endif
                  m_mvBufBDMVR4TMOPPOSITELIC[uiMergeCand << 1], m_mvBufBDMVR4TMOPPOSITELIC[(uiMergeCand << 1) + 1], m_mvBufEncBDOF4TMOPPOSITELIC[uiMergeCand]);
              }
              else
#endif
#endif
#if TM_MRG
              if( pu.tmMergeFlag )
              {
                PU::spanMotionInfo(pu, mergeCtx,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                  pu.colIdx,
#endif
                  m_mvBufBDMVR4TM[uiMergeCand << 1], m_mvBufBDMVR4TM[(uiMergeCand << 1) + 1], m_mvBufEncBDOF4TM[uiMergeCand]);
              }
              else
#endif
#if JVET_X0049_ADAPT_DMVR
              if( pu.bmMergeFlag )
              {
                PU::spanMotionInfo(pu, bmMrgCtx,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                  pu.colIdx,
#endif
                  m_mvBufBDMVR4BM[uiMergeCand << 1], m_mvBufBDMVR4BM[(uiMergeCand << 1) + 1], m_mvBufEncBDOF4BM[uiMergeCand]);
              }
              else
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
                if (pu.mergeOppositeLic)
                {
                  PU::spanMotionInfo(pu, mergeCtxOppositeLic,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                    pu.colIdx,
#endif
                    m_mvBufBDMVR4OPPOSITELIC[uiMergeCand << 1], m_mvBufBDMVR4OPPOSITELIC[(uiMergeCand << 1) + 1], m_mvBufEncBDOF4OPPOSITELIC[uiMergeCand]);

                }
                else
#endif
                PU::spanMotionInfo(pu, mergeCtx,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                  pu.colIdx,
#endif
                  m_mvBufBDMVR[uiMergeCand << 1], m_mvBufBDMVR[(uiMergeCand << 1) + 1], m_mvBufEncBDOF[uiMergeCand]);
            }
#endif
          }
        }
      }
      else
      {
        pu.mvRefine = true;
        m_pcInterSearch->motionCompensation( pu );
#if JVET_AD0213_LIC_IMP
        if (pu.cu->licFlag)
        {
          bool disCond = pu.ciipFlag && !(((slice.getPOC() - slice.getRefPOC(REF_PIC_LIST_0, 0)) == 1) && slice.getCheckLDC());
          for (int list = 0; list < 2; list++)
          {
            if (pu.refIdx[list] >= 0)
            {
              for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
              {
                if (disCond)
                {
                  pu.cu->licScale[list][comp] = 32;
                  pu.cu->licOffset[list][comp] = 0;
                }
                else
                {
                  m_pcInterSearch->setLicParam(list, comp, pu.cu->licScale[list][comp], pu.cu->licOffset[list][comp]);
                }
              }
            }
          }
        }
#endif
        pu.mvRefine = false;
#if MULTI_PASS_DMVR
        if (pu.bdmvrRefine)
        {
#if JVET_AG0276_LIC_FLAG_SIGNALING
#if TM_MRG
          if (pu.tmMergeFlag && pu.tmMergeFlagOppositeLic)
          {
            PU::spanMotionInfo(pu, mergeCtx,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
              pu.colIdx,
#endif
              m_mvBufBDMVR4TMOPPOSITELIC[uiMergeCand << 1], m_mvBufBDMVR4TMOPPOSITELIC[(uiMergeCand << 1) + 1], m_pcInterSearch->getBdofSubPuMvOffset());
          }
          else
#endif
#endif
#if TM_MRG
          if( pu.tmMergeFlag )
          {
            PU::spanMotionInfo(pu, mergeCtx,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
              pu.colIdx,
#endif
              m_mvBufBDMVR4TM[uiMergeCand << 1], m_mvBufBDMVR4TM[(uiMergeCand << 1) + 1], m_pcInterSearch->getBdofSubPuMvOffset());
          }
          else
#endif
#if JVET_X0049_ADAPT_DMVR 
            if (pu.bmMergeFlag)
            {
              PU::spanMotionInfo(pu, bmMrgCtx,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                pu.colIdx,
#endif
                m_mvBufBDMVR4BM[uiMergeCand << 1], m_mvBufBDMVR4BM[(uiMergeCand << 1) + 1], m_mvBufEncBDOF4BM[uiMergeCand]);
            }
            else
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
              if (pu.mergeOppositeLic)
              {
                PU::spanMotionInfo(pu, mergeCtxOppositeLic,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                  pu.colIdx,
#endif
                  m_mvBufBDMVR4OPPOSITELIC[uiMergeCand << 1], m_mvBufBDMVR4OPPOSITELIC[(uiMergeCand << 1) + 1], m_mvBufEncBDOF4OPPOSITELIC[uiMergeCand]);

              }
              else
#endif
#if JVET_AG0276_LIC_BDOF_BDMVR
              {
                if (pu.cu->licFlag == true)
                {
                  memset((void*)m_pcInterSearch->getBdofSubPuMvOffset(), 0, BDOF_SUBPU_MAX_NUM * sizeof(Mv));
                }
#endif
              PU::spanMotionInfo(pu, mergeCtx,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                pu.colIdx,
#endif
                m_mvBufBDMVR[uiMergeCand << 1], m_mvBufBDMVR[(uiMergeCand << 1) + 1], m_pcInterSearch->getBdofSubPuMvOffset());
#if JVET_AG0276_LIC_BDOF_BDMVR
              }
#endif
        }
#endif
      }
      if (!cu.mmvdSkip && !pu.ciipFlag && uiNoResidualPass != 0 && !cu.affine 
#if TM_MRG
          && !pu.tmMergeFlag
#endif
#if JVET_X0049_ADAPT_DMVR
        && !pu.bmMergeFlag
#endif
#if AFFINE_MMVD && MERGE_ENC_OPT
          && !pu.afMmvdFlag
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
        && !pu.mergeOppositeLic
        && !pu.tmMergeFlagOppositeLic
#endif
        )
      {
        CHECK(uiMergeCand >= mergeCtx.numValidMergeCand, "out of normal merge");
        isTestSkipMerge[uiMergeCand] = true;
      }
#if JVET_AG0135_AFFINE_CIIP
      if (!pu.affBMMergeFlag && uiNoResidualPass != 0 && cu.affine
#if AFFINE_MMVD && MERGE_ENC_OPT
        && !pu.afMmvdFlag
#endif
        )
      {
        isAffineTestSkipMerge[uiMergeCand] = true;
      }
#endif
#if ENABLE_OBMC
      cu.isobmcMC = true;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
      if (cu.affine == false)
      {
#endif
      cu.obmcFlag = true;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
      }
#endif
#if JVET_X0090_CIIP_FIX
      if (!pu.ciipFlag)
      {
        m_pcInterSearch->subBlockOBMC(pu);
      }
#else
      m_pcInterSearch->subBlockOBMC( pu );
#endif
      cu.isobmcMC = false;
#endif

      xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, uiNoResidualPass, uiNoResidualPass == 0 ? &candHasNoResidual[uiMrgHADIdx] : NULL );

      if( m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip && !pu.ciipFlag)
      {
        bestIsSkip = !bestCS->cus.empty() && bestCS->getCU( partitioner.chType )->rootCbf == 0;
      }
      tempCS->initStructData( encTestMode.qp );
    }// end loop uiMrgHADIdx

    if( uiNoResidualPass == 0 && m_pcEncCfg->getUseEarlySkipDetection() )
    {
      const CodingUnit     &bestCU = *bestCS->getCU( partitioner.chType );
      const PredictionUnit &bestPU = *bestCS->getPU( partitioner.chType );

      if( bestCU.rootCbf == 0 )
      {
        if( bestPU.mergeFlag )
        {
          m_modeCtrl->setEarlySkipDetected();
        }
        else if( m_pcEncCfg->getMotionEstimationSearchMethod() != MESEARCH_SELECTIVE )
        {
          int absolute_MV = 0;

          for( uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
          {
            if( slice.getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
            {
              absolute_MV += bestPU.mvd[uiRefListIdx].getAbsHor() + bestPU.mvd[uiRefListIdx].getAbsVer();
            }
          }

          if( absolute_MV == 0 )
          {
            m_modeCtrl->setEarlySkipDetected();
          }
        }
      }
    }
  }
  if ( m_bestModeUpdated && bestCS->cost != MAX_DOUBLE )
  {
    xCalDebCost( *bestCS, partitioner );
  }
#if JVET_AF0057
  m_pcInterSearch->xDmvrSetEncoderCheckFlag(false);
#endif
}

#if JVET_W0097_GPM_MMVD_TM
void EncCu::xCheckRDCostMergeGeoComb2Nx2N(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode, bool isSecondPass)
{
  int numSATDCands = (m_fastGpmMmvdSearch && isSecondPass) ? 60 : 70;

  tempCS->initStructData(encTestMode.qp);
#if TM_MRG
  MergeCtx mergeCtx[GEO_NUM_TM_MV_CAND];
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  MergeCtx& mergeCtxRegular = mergeCtx[GEO_TM_OFF];
#endif
#else
  MergeCtx mergeCtx;
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  MergeCtx& mergeCtxRegular = mergeCtx;
#endif
#endif
  const SPS &sps = *tempCS->sps;
  CodedCUInfo& relatedCU = ((EncModeCtrlMTnoRQT *)m_modeCtrl)->getBlkInfo(pm.currArea());
  bool extMMVD = tempCS->picHeader->getGPMMMVDTableFlag();

  if (sps.getSbTMVPEnabledFlag())
  {
    Size bufSize = g_miScaling.scale(tempCS->area.lumaSize());
#if TM_MRG
    for (int i = 0; i < GEO_NUM_TM_MV_CAND; i++)
    {
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
      for (int j = 0; j < SUB_TMVP_NUM; j++)
      {
        mergeCtx[i].subPuMvpMiBuf[j] = MotionBuf(m_subPuMiBuf[j], bufSize);
    }
#else
      mergeCtx[i].subPuMvpMiBuf = MotionBuf(m_subPuMiBuf, bufSize);
#endif
    }
#else
    mergeCtx.subPuMvpMiBuf = MotionBuf(m_subPuMiBuf, bufSize);
#endif
  }

#if JVET_AE0046_BI_GPM
  std::array<bool, GEO_MAX_NUM_UNI_CANDS> refinePossible;
  refinePossible.fill(false);
#endif

  // 1. bit estimation
  const double sqrtLambdaFracBits = m_pcRdCost->getMotionLambda() * FRAC_BITS_SCALE;
  uint8_t maxNumMergeCandidates = tempCS->sps->getMaxNumGeoCand();
  const TempCtx ctxStart(m_ctxCache, m_CABACEstimator->getCtx());

  double geoModeCost[GEO_NUM_PARTITION_MODE], geoMergeIdxCost[MRG_MAX_NUM_CANDS], geoMMVDFlagCost[2], geoMMVDIdxCost[GPM_EXT_MMVD_MAX_REFINE_NUM];
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  double geoSigModeCost[GEO_NUM_SIG_PARTMODE];
#endif
#if TM_MRG
  double geoTMFlagCost[2];
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
  double geoBldFlagCost[GEO_BLENDING_NUM];
#endif
#if JVET_AG0164_AFFINE_GPM
  double geoAffMergeIdxCost[GEO_MAX_NUM_UNI_AFF_CANDS];
  uint8_t maxNumGpmAffMergeCandidates = tempCS->sps->getMaxNumGpmAffCand();
  CHECK(maxNumGpmAffMergeCandidates > GEO_MAX_NUM_UNI_AFF_CANDS, "Maximum GPM Affine Num is too large");
#endif

  for (int idx = 0; idx < GEO_NUM_PARTITION_MODE; idx++)
  {
    uint64_t fracBits = m_CABACEstimator->geo_mode_est(ctxStart, idx);
    geoModeCost[idx] = (double)fracBits * sqrtLambdaFracBits;
  }
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  if (sps.getUseAltGPMSplitModeCode())
  {
    for (int idx = 0; idx < GEO_NUM_SIG_PARTMODE; idx++)
    {
      uint64_t fracBits = m_CABACEstimator->geo_mode_est(ctxStart, idx, 1);
      geoSigModeCost[idx] = (double)fracBits * sqrtLambdaFracBits;
    }
  }
#endif
  for (int idx = 0; idx < maxNumMergeCandidates; idx++)
  {
    uint64_t fracBits = m_CABACEstimator->geo_mergeIdx_est(ctxStart, idx, maxNumMergeCandidates);
    geoMergeIdxCost[idx] = (double)fracBits * sqrtLambdaFracBits;
  } 
#if JVET_AG0164_AFFINE_GPM
  for (int idx = 0; idx < maxNumGpmAffMergeCandidates; idx++)
  {
    uint64_t fracBits = m_CABACEstimator->geo_mergeIdx_est(ctxStart, idx, maxNumGpmAffMergeCandidates, 1);
    geoAffMergeIdxCost[idx] = (double)fracBits * sqrtLambdaFracBits;
  }
#endif
  for (int idx = 0; idx < 2; idx++)
  {
    uint64_t fracBits = m_CABACEstimator->geo_mmvdFlag_est(ctxStart, idx);
    geoMMVDFlagCost[idx] = (double)fracBits * sqrtLambdaFracBits;
  }
  for (int idx = 0; idx < (extMMVD ? GPM_EXT_MMVD_MAX_REFINE_NUM : GPM_MMVD_MAX_REFINE_NUM); idx++)
  {
    uint64_t fractBits = m_CABACEstimator->geo_mmvdIdx_est(ctxStart, idx, extMMVD);
    geoMMVDIdxCost[idx] = (double)fractBits * sqrtLambdaFracBits;
  }
#if TM_MRG
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  for (int idx = 0; idx < 2 && sps.getUseGPMTMMode(); idx++)
#else
  for (int idx = 0; idx < 2 && sps.getUseDMVDMode(); idx++)
#endif
  {
    uint64_t fracBits = m_CABACEstimator->geo_tmFlag_est(ctxStart, idx);
    geoTMFlagCost[idx] = (double)fracBits * sqrtLambdaFracBits;
  }
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
  for (int idx = 0; idx < GEO_BLENDING_NUM; idx++)
  {
    uint64_t fracBits = m_CABACEstimator->geoBldFlagEst(ctxStart, idx);
    geoBldFlagCost[idx] = (double)fracBits * sqrtLambdaFracBits;
  }
#endif
#if JVET_Y0065_GPM_INTRA
  bool bUseOnlyOneVector = (tempCS->slice->isInterP() || tempCS->sps->getMaxNumGeoCand() == 1);
  double geoIntraFlag0Cost[2], geoIntraFlag1Cost[2][2], geoIntraIdxCost[GEO_MAX_NUM_INTRA_CANDS];
  for (int idx = 0; idx < 2; idx++)
  {
    uint64_t fracBits = m_CABACEstimator->geo_intraFlag_est(ctxStart, idx);
    geoIntraFlag0Cost[idx] = (double)fracBits * sqrtLambdaFracBits;
    geoIntraFlag1Cost[0][idx] = !bUseOnlyOneVector ? geoIntraFlag0Cost[idx] : 0;
    geoIntraFlag1Cost[1][idx] = 0;
  }
  for (int idx = 0; idx < GEO_MAX_NUM_INTRA_CANDS; idx++)
  {
    uint64_t fracBits = m_CABACEstimator->geo_intraIdx_est(idx);
    geoIntraIdxCost[idx] = (double)fracBits * sqrtLambdaFracBits;
  }
#endif
  m_CABACEstimator->getCtx() = ctxStart;

  // 2. get SAD for all candidates
  CodingUnit &cu = tempCS->addCU(tempCS->area, pm.chType);
  pm.setCUData(cu);
  cu.predMode = MODE_INTER;
  cu.slice = tempCS->slice;
  cu.tileIdx = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
  cu.qp = encTestMode.qp;
  cu.affine = false;
  cu.mtsFlag = false;
#if JVET_AG0061_INTER_LFNST_NSPT
  cu.lfnstFlag = false;
  cu.lfnstIdx = 0;
#endif
#if INTER_LIC
  cu.licFlag = false;
#endif
  cu.bcwIdx = BCW_DEFAULT;
  cu.geoFlag = true;
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  cu.geoBlendFlag = false;
#endif
  cu.imv = 0;
  cu.mmvdSkip = false;
  cu.skip = false;
  cu.mipFlag = false;
#if JVET_V0130_INTRA_TMP
  cu.tmpFlag = false;
#endif
  cu.bdpcmMode = 0;
#if JVET_AG0058_EIP
  cu.eipFlag = false;
#endif

  PredictionUnit &pu = tempCS->addPU(cu, pm.chType);
  pu.mergeFlag = true;
  pu.regularMergeFlag = false;
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
  pu.tmMergeFlag = false;
#endif
#if JVET_X0049_ADAPT_DMVR
  pu.bmMergeFlag = false;
#endif

#if JVET_AG0164_AFFINE_GPM
  double geoAffineFlagCost[2] = { 0.0, 0.0 };
  int  affGPMFlagCtxOffset = 0;

  affGPMFlagCtxOffset = PU::getAffGPMCtxOffset(pu);

  for (int idx = 0; idx < 2; idx++)
  {
    uint64_t fracBits = m_CABACEstimator->geo_affFlag_est(ctxStart, idx, affGPMFlagCtxOffset);
    geoAffineFlagCost[idx] = (double)fracBits * sqrtLambdaFracBits;
  }

  bool affGPMValid = PU::isAffineGPMValid(pu);

  pu.affineGPM[0]= pu.affineGPM[1] = 0;
  if (!affGPMValid)
  {
    geoAffineFlagCost[0] = geoAffineFlagCost[1] = 0.0;
  }
#endif

  CHECK(!m_mergeCandAvail, "merge candidates are not available");

#if JVET_AE0046_BI_GPM
  PU::setGpmDirMode(pu);
  m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4GPM[0], m_mvBufBDMVR4GPM[1]);
#endif

#if TM_MRG
#if JVET_AE0046_BI_GPM
  PU::getGeoMergeCandidates(pu, mergeCtx[GEO_TM_OFF], &m_mergeCand, true);
#else
  PU::getGeoMergeCandidates(pu, mergeCtx[GEO_TM_OFF], &m_mergeCand);
#endif
  maxNumMergeCandidates = min((int)maxNumMergeCandidates, mergeCtx[GEO_TM_OFF].numValidMergeCand);
#else
  PU::getGeoMergeCandidates(pu, mergeCtx, &m_mergeCand);
  maxNumMergeCandidates = min((int)maxNumMergeCandidates, mergeCtx.numValidMergeCand);
#endif
#if JVET_AG0164_AFFINE_GPM
  AffineMergeCtx affMergeCtx;
  int numRegularGpmMergeCand = maxNumMergeCandidates;

  if (affGPMValid)
  {
    PU::getGeoAffMergeCandidates(pu, affMergeCtx, m_pcInterSearch);
    if (pu.cs->sps->getUseAML() && (affMergeCtx.numValidMergeCand > 1)
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && pu.cs->sps->getTMToolsEnableFlag()
#endif
      )
    {
      affMergeCtx.maxNumMergeCand = affMergeCtx.numValidMergeCand;
      PredictionUnit puSaved = pu;
      CodingUnit     cuSaved = *pu.cu;
      puSaved.cu = &cuSaved;
      m_pcInterSearch->adjustAffineMergeCandidates(puSaved, affMergeCtx);

      affMergeCtx.numValidMergeCand = std::min(affMergeCtx.numValidMergeCand, (int)pu.cs->sps->getMaxNumGpmAffCand());
      affMergeCtx.maxNumMergeCand = affMergeCtx.numValidMergeCand;

      if (!isSecondPass)
      {
        numSATDCands += affMergeCtx.numValidMergeCand + 1;
      }
    }
    CHECK(affMergeCtx.numValidMergeCand > maxNumGpmAffMergeCandidates, "Invalid GPM Affine candidates");

    maxNumMergeCandidates += affMergeCtx.numValidMergeCand;

    affMergeCtx.m_indexOffset = numRegularGpmMergeCand;
  }

  PelUnitBuf geoBuffer[GEO_MAX_ALL_INTER_UNI_CANDS];
  PelUnitBuf geoTempBuf[GEO_MAX_ALL_INTER_UNI_CANDS];
  PelUnitBuf geoMMVDBuf[GEO_MAX_ALL_INTER_UNI_CANDS][GPM_EXT_MMVD_MAX_REFINE_NUM];
  PelUnitBuf geoMMVDTempBuf[GEO_MAX_ALL_INTER_UNI_CANDS][GPM_EXT_MMVD_MAX_REFINE_NUM];
#else
  PelUnitBuf geoBuffer[GEO_MAX_NUM_UNI_CANDS];
  PelUnitBuf geoTempBuf[GEO_MAX_NUM_UNI_CANDS];
  PelUnitBuf geoMMVDBuf[GEO_MAX_NUM_UNI_CANDS][GPM_EXT_MMVD_MAX_REFINE_NUM];
  PelUnitBuf geoMMVDTempBuf[GEO_MAX_NUM_UNI_CANDS][GPM_EXT_MMVD_MAX_REFINE_NUM];
#endif

#if JVET_Y0065_GPM_INTRA
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
  PelUnitBuf geoCombinations[GEO_MAX_TRY_WEIGHTED_SAD*GEO_BLENDING_NUM+1];
#else
  PelUnitBuf geoCombinations[GEO_MAX_TRY_WEIGHTED_SAD+1];
#endif
  PelUnitBuf geoIntraBuffer[GEO_NUM_INTRA_RDO_BUFFER];
  PelUnitBuf geoIntraTempBuf[GEO_NUM_INTRA_RDO_BUFFER];
#else
  PelUnitBuf geoCombinations[GEO_MAX_TRY_WEIGHTED_SAD];
#endif
  DistParam  distParam;

  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));
  DistParam distParamWholeBlk;
  m_pcRdCost->setDistParam(distParamWholeBlk, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y().buf, m_acMergeBuffer[0].Y().stride, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y);
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  const bool bUseHadamard = !tempCS->slice->getDisableSATDForRD();
  DistParam distParamWholeBlkGeoBlend;
  m_pcRdCost->setDistParam(distParamWholeBlkGeoBlend, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y().buf, m_acMergeBuffer[0].Y().stride, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, 0, 1, bUseHadamard);
#endif
#if JVET_AG0164_AFFINE_GPM
  Distortion sadWholeBlk[GEO_MAX_ALL_INTER_UNI_CANDS], sadMMVDWholeBlk[GEO_MAX_ALL_INTER_UNI_CANDS][GPM_EXT_MMVD_MAX_REFINE_NUM];

  int  pocMrg[GEO_MAX_ALL_INTER_UNI_CANDS];
  Mv   mrgMv[GEO_MAX_ALL_INTER_UNI_CANDS];

  bool mrgDuplicated[GEO_MAX_ALL_INTER_UNI_CANDS];

  double bestMrgCost = MAX_DOUBLE;
  double bestNormalMrgCost = MAX_DOUBLE;

#if JVET_Y0065_GPM_INTRA
  Distortion sadIntraWholeBlk[GEO_MAX_ALL_INTER_UNI_CANDS];
  uint8_t isGeoChromaAvail[GEO_MAX_ALL_INTER_UNI_CANDS];
  uint8_t isGeoMMVDChromaAvail[GEO_MAX_ALL_INTER_UNI_CANDS][GPM_EXT_MMVD_MAX_REFINE_NUM];
  uint8_t isGeoIntraChromaAvail[GEO_NUM_INTRA_RDO_BUFFER];
  memset(isGeoChromaAvail, 0, sizeof(uint8_t) * GEO_MAX_ALL_INTER_UNI_CANDS);
  memset(isGeoMMVDChromaAvail, 0, sizeof(uint8_t) * GEO_MAX_ALL_INTER_UNI_CANDS* GPM_EXT_MMVD_MAX_REFINE_NUM);
  memset(isGeoIntraChromaAvail, 0, sizeof(uint8_t) * GEO_NUM_INTRA_RDO_BUFFER);
#else
  bool isGeoChromaAvail[GEO_MAX_ALL_INTER_UNI_CANDS];
  bool isGeoMMVDChromaAvail[GEO_MAX_ALL_INTER_UNI_CANDS][GPM_EXT_MMVD_MAX_REFINE_NUM];
  memset(isGeoChromaAvail, false, sizeof(bool) * GEO_MAX_ALL_INTER_UNI_CANDS);
  memset(isGeoMMVDChromaAvail, false, sizeof(bool) * GEO_MAX_ALL_INTER_UNI_CANDS* GPM_EXT_MMVD_MAX_REFINE_NUM);
#endif
#else
  Distortion sadWholeBlk[GEO_MAX_NUM_UNI_CANDS], sadMMVDWholeBlk[GEO_MAX_NUM_UNI_CANDS][GPM_EXT_MMVD_MAX_REFINE_NUM];

  int  pocMrg[GEO_MAX_NUM_UNI_CANDS];
  Mv   mrgMv[GEO_MAX_NUM_UNI_CANDS];
  bool mrgDuplicated[GEO_MAX_NUM_UNI_CANDS];

  double bestMrgCost = MAX_DOUBLE;
  double bestNormalMrgCost = MAX_DOUBLE;

#if JVET_Y0065_GPM_INTRA
  Distortion sadIntraWholeBlk[GEO_NUM_INTRA_RDO_BUFFER];
  uint8_t isGeoChromaAvail[GEO_MAX_NUM_UNI_CANDS];
  uint8_t isGeoMMVDChromaAvail[GEO_MAX_NUM_UNI_CANDS][GPM_EXT_MMVD_MAX_REFINE_NUM];
  uint8_t isGeoIntraChromaAvail[GEO_NUM_INTRA_RDO_BUFFER];
  memset(isGeoChromaAvail, 0, sizeof(uint8_t) * GEO_MAX_NUM_UNI_CANDS);
  memset(isGeoMMVDChromaAvail, 0, sizeof(uint8_t) * GEO_MAX_NUM_UNI_CANDS * GPM_EXT_MMVD_MAX_REFINE_NUM);
  memset(isGeoIntraChromaAvail, 0, sizeof(uint8_t) * GEO_NUM_INTRA_RDO_BUFFER);
#else
  bool isGeoChromaAvail[GEO_MAX_NUM_UNI_CANDS];
  bool isGeoMMVDChromaAvail[GEO_MAX_NUM_UNI_CANDS][GPM_EXT_MMVD_MAX_REFINE_NUM];
  memset(isGeoChromaAvail, false, sizeof(bool) * GEO_MAX_NUM_UNI_CANDS);
  memset(isGeoMMVDChromaAvail, false, sizeof(bool) * GEO_MAX_NUM_UNI_CANDS * GPM_EXT_MMVD_MAX_REFINE_NUM);
#endif
#endif
#if TM_MRG
  bool isGeoTmChromaAvail[GEO_TM_MAX_NUM_CANDS];
  memset(isGeoTmChromaAvail, false, sizeof(bool) * GEO_TM_MAX_NUM_CANDS);
#endif

#if JVET_AG0164_AFFINE_GPM
  auto performOBMCOnOnePartition = [&]
  ( PredictionUnit& pu, int mergeCand, int geoCombIdx, PelUnitBuf& predSrc, PelUnitBuf& obmcBuf, int mmvdCand = -1)
  {
    if (mergeCand < numRegularGpmMergeCand)//non Affine
    {
#if TM_MRG
      if (mmvdCand != -1)
      {
        mergeCtx[GEO_TM_OFF].setGeoMmvdMergeInfo(pu, mergeCand, mmvdCand);
      }
      else
      {
        mergeCtx[GEO_TM_OFF].setMergeInfo(pu, mergeCand);
      }
#else
      if (mmvdCand != -1)
      {
        mergeCtx.setGeoMmvdMergeInfo(pu, mergeCand, mmvdCand);
      }
      else
      {
        mergeCtx.setMergeInfo(pu, mergeCand);
      }
#endif
#if JVET_W0123_TIMD_FUSION
#if JVET_AE0046_BI_GPM
      Mv* bdofSubPuMvOffset = nullptr;
      if (refinePossible[mergeCand])
      {
        bdofSubPuMvOffset = m_mvBufEncBDOF4GPM[GEO_TM_OFF][mergeCand];
      }
      PU::spanMotionInfo2(pu, MergeCtx(),
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
        0,
#endif
        nullptr, nullptr, bdofSubPuMvOffset);
#else
      PU::spanMotionInfo2(pu);
#endif
#else
      PU::spanMotionInfo(pu);
#endif
    }
    else//Affine
    {
      affMergeCtx.setAffMergeInfo(pu, mergeCand, mmvdCand);
      PU::spanMotionInfo2(pu);
    }
    obmcBuf.copyFrom(predSrc);
    pu.cu->isobmcMC = true;
    pu.cu->obmcFlag = true;
    m_pcInterSearch->subBlockOBMC(pu, &obmcBuf);
    pu.cu->affine = false;
    pu.cu->isobmcMC = false;
  };
#endif

  for (uint8_t mergeCand = 0; mergeCand < maxNumMergeCandidates; mergeCand++)
  {
#if JVET_AG0164_AFFINE_GPM
    int isAffine = mergeCand < numRegularGpmMergeCand ? 0: 1;
    if (!isAffine)//regular merge cand
    {
#endif
#if TM_MRG
    int mrgList = mergeCtx[GEO_TM_OFF].mvFieldNeighbours[(mergeCand << 1) + 0].refIdx == -1 ? 1 : 0;
    int mrgRefIdx = mergeCtx[GEO_TM_OFF].mvFieldNeighbours[(mergeCand << 1) + mrgList].refIdx;
#else
    int mrgList = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 0].refIdx == -1 ? 1 : 0;
    int mrgRefIdx = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + mrgList].refIdx;
#endif
    pocMrg[mergeCand] = tempCS->slice->getRefPic((RefPicList)mrgList, mrgRefIdx)->getPOC();
#if TM_MRG
    mrgMv[mergeCand] = mergeCtx[GEO_TM_OFF].mvFieldNeighbours[(mergeCand << 1) + mrgList].mv;
#else
    mrgMv[mergeCand] = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + mrgList].mv;
#endif
    mrgDuplicated[mergeCand] = false;
    if (mergeCand)
    {
      for (int i = 0; i < mergeCand; i++)
      {
        if (pocMrg[mergeCand] == pocMrg[i] && mrgMv[mergeCand] == mrgMv[i])
        {
          mrgDuplicated[mergeCand] = true;
          break;
        }
      }
    }
#if JVET_AE0046_BI_GPM
    if (mrgDuplicated[mergeCand])
    {
      continue;
    }
#else
#if !MULTI_HYP_PRED
    if (mrgDuplicated[mergeCand])
    {
      continue;
    }
#endif
#endif
    geoBuffer[mergeCand] = m_acMergeBuffer[mergeCand].getBuf(localUnitArea);
#if TM_MRG
    mergeCtx[GEO_TM_OFF].setMergeInfo(pu, mergeCand);
#else
    mergeCtx.setMergeInfo(pu, mergeCand);
#endif
    if (m_pcEncCfg->getMCTSEncConstraint() && (!(MCTSHelper::checkMvBufferForMCTSConstraint(pu))))
    {
      tempCS->initStructData(encTestMode.qp);
      return;
    }

#if JVET_AE0046_BI_GPM
      if (
#if JVET_AG0164_AFFINE_GPM
        !isAffine &&
#endif  
        PU::checkBDMVRCondition(pu, true))
    {
      refinePossible[mergeCand] = true;
      pu.bdmvrRefine = true;
      PU::spanPuMv2DmvrBuffer(pu, m_mvBufBDMVR4GPM[0], m_mvBufBDMVR4GPM[1]);
    }
#endif

    m_pcInterSearch->motionCompensation(pu, geoBuffer[mergeCand], REF_PIC_LIST_X, true, false);

#if JVET_AE0046_BI_GPM
    if (pu.bdmvrRefine)
    {
      pu.bdmvrRefine = false;
      ::memcpy(m_mvBufEncBDOF4GPM[GEO_TM_OFF][mergeCand], m_pcInterSearch->getBdofSubPuMvOffset(), sizeof(Mv) * BDOF_SUBPU_MAX_NUM);
    }
#endif
#if JVET_AG0164_AFFINE_GPM
    }
    else//affine cand
    {
      mrgDuplicated[mergeCand] = false;

      geoBuffer[mergeCand] = m_acMergeBuffer[mergeCand].getBuf(localUnitArea);

      affMergeCtx.setAffMergeInfo(pu, mergeCand);
      if (m_pcEncCfg->getMCTSEncConstraint() && (!(MCTSHelper::checkMvBufferForMCTSConstraint(pu))))
      {
        tempCS->initStructData(encTestMode.qp);
        return;
      }
      m_pcInterSearch->motionCompensation(pu, geoBuffer[mergeCand], REF_PIC_LIST_X, true, false);
      pu.cu->affine = false;
    }
#endif
#if MULTI_HYP_PRED
    geoTempBuf[mergeCand] = m_acRealMergeBuffer[MRG_MAX_NUM_CANDS + mergeCand].getBuf(localUnitArea);
#else
    geoTempBuf[mergeCand] = m_acMergeTmpBuffer[mergeCand].getBuf(localUnitArea);
#endif
    geoTempBuf[mergeCand].Y().copyFrom(geoBuffer[mergeCand].Y());
    geoTempBuf[mergeCand].Y().roundToOutputBitdepth(geoTempBuf[mergeCand].Y(), cu.slice->clpRng(COMPONENT_Y));
    distParamWholeBlk.cur.buf = geoTempBuf[mergeCand].Y().buf;
    distParamWholeBlk.cur.stride = geoTempBuf[mergeCand].Y().stride;
    sadWholeBlk[mergeCand] = distParamWholeBlk.distFunc(distParamWholeBlk);

#if JVET_AG0164_AFFINE_GPM
    double curCost = sadWholeBlk[mergeCand] + (isAffine? geoAffMergeIdxCost[mergeCand - numRegularGpmMergeCand]: geoMergeIdxCost[mergeCand]);
#else
    double curCost = sadWholeBlk[mergeCand] + geoMergeIdxCost[mergeCand];
#endif
#if JVET_AG0164_AFFINE_GPM
    curCost += geoAffineFlagCost[isAffine];
    if(!isAffine)
#endif
#if JVET_Y0065_GPM_INTRA
    curCost += geoIntraFlag0Cost[0];
#endif
    if (curCost < bestNormalMrgCost)
    {
      bestNormalMrgCost = curCost;
    }
    curCost += geoMMVDFlagCost[0];
    if (curCost < bestMrgCost)
    {
      bestMrgCost = curCost;
    }
  }
#if MULTI_HYP_PRED
#if JVET_AE0046_BI_GPM
  // avoid setting GEO merge list for MHP
#else
#if TM_MRG
#if JVET_AD0213_LIC_IMP
  m_pcInterSearch->setGeoTmpBuffer(mergeCtx[GEO_TM_OFF], 1);
#else
  m_pcInterSearch->setGeoTmpBuffer(mergeCtx[GEO_TM_OFF]);
#endif
#else
#if JVET_AD0213_LIC_IMP
  m_pcInterSearch->setGeoTmpBuffer(mergeCtx, 1);
#else
  m_pcInterSearch->setGeoTmpBuffer(mergeCtx);
#endif
#endif
#endif
#endif

#if JVET_Y0065_GPM_INTRA
  uint8_t geoIntraMPMList[GEO_NUM_PARTITION_MODE][2][GEO_MAX_NUM_INTRA_CANDS];
  uint8_t intraRDOBufIdx[NUM_LUMA_MODE];
  memset(intraRDOBufIdx, -1, sizeof(uint8_t)*NUM_LUMA_MODE);
  int intraRDOBufCnt = 0;
#if ENABLE_DIMD && JVET_W0123_TIMD_FUSION
  if (sps.getUseDimd() || sps.getUseTimd())
  {
    IntraPrediction::deriveDimdMode(tempCS->picture->getRecoBuf(tempCS->area.Y()), tempCS->area.Y(), cu);
    if (sps.getUseTimd())
    {
      cu.timdMode = m_pcIntraSearch->deriveTimdMode(tempCS->picture->getRecoBuf(tempCS->area.Y()), tempCS->area.Y(), cu);
    }
  }
#elif ENABLE_DIMD
  if (sps.getUseDimd())
  {
    IntraPrediction::deriveDimdMode(tempCS->picture->getRecoBuf(tempCS->area.Y()), tempCS->area.Y(), cu);
  }
#elif JVET_W0123_TIMD_FUSION
  if (sps.getUseTimd())
  {
    cu.timdMode = m_pcIntraSearch->deriveTimdMode(tempCS->picture->getRecoBuf(tempCS->area.Y()), tempCS->area.Y(), cu);
  }
#endif
#if ENABLE_DIMD
  int8_t dimdMode = cu.dimdMode;
#endif
#if JVET_W0123_TIMD_FUSION
  int timdMode = cu.timdMode;
#if JVET_AC0094_REF_SAMPLES_OPT
  bool timdModeCheckWA = cu.timdModeCheckWA;
#endif
#endif
  for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
  {
    for (int partIdx = 0; partIdx < 2; partIdx++)
    {
      PU::getGeoIntraMPMs(pu, geoIntraMPMList[splitDir][partIdx], splitDir, g_geoTmShape[partIdx][g_geoParams[splitDir][0]]
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
                        , (splitDir == 0 && partIdx == 0)
#endif
      );
      for (int intraIdx = 0; intraIdx < GEO_MAX_NUM_INTRA_CANDS; intraIdx++)
      {
        uint8_t intraPred = geoIntraMPMList[splitDir][partIdx][intraIdx];
        if (intraRDOBufIdx[intraPred] >= GEO_NUM_INTRA_RDO_BUFFER)
        {
          uint8_t intraCand = intraRDOBufCnt++;
          CHECK(intraCand >= GEO_NUM_INTRA_RDO_BUFFER, "Geo Intra buffer overflow");
          intraRDOBufIdx[intraPred] = intraCand;
          pu.intraDir[0] = intraPred;

#if JVET_AG0164_AFFINE_GPM
          geoIntraBuffer[intraCand] = m_acMergeBuffer[intraCand + GEO_MAX_ALL_INTER_UNI_CANDS].getBuf(localUnitArea);
#else
          geoIntraBuffer[intraCand] = m_acMergeBuffer[intraCand + GEO_MAX_NUM_UNI_CANDS].getBuf(localUnitArea);
#endif
          pu.gpmIntraFlag = true;
          m_pcIntraSearch->initIntraPatternChType(cu, pu.Y());
          m_pcIntraSearch->predIntraAng(COMPONENT_Y, geoIntraBuffer[intraCand].Y(), pu);
          if (pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
          {
            geoIntraTempBuf[intraCand] = m_acGeoMMVDTmpBuffer[0][intraCand].getBuf(localUnitArea);
            geoIntraTempBuf[intraCand].Y().rspSignal(geoIntraBuffer[intraCand].Y(), m_pcReshape->getInvLUT());
          }
          else
          {
            geoIntraTempBuf[intraCand] = geoIntraBuffer[intraCand];
          }
          pu.gpmIntraFlag = false;
          distParamWholeBlk.cur.buf = geoIntraTempBuf[intraCand].Y().buf;
          distParamWholeBlk.cur.stride = geoIntraTempBuf[intraCand].Y().stride;
          sadIntraWholeBlk[intraCand] = distParamWholeBlk.distFunc(distParamWholeBlk);
        }
      }
    }
  }

#endif // JVET_Y0065_GPM_INTRA

#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
//----------------------------------------------------------
//  Create List and estim cost
//----------------------------------------------------------
  const int idxBlendGeoFirst = GEO_MAX_NUM_UNI_CANDS + 67;
  int idxBufGeo = idxBlendGeoFirst;
  GeoBlendInfo geoBlendInfo[GEO_BLEND_MAX_NUM_CANDS];
  int numGeoBlendInfoCand = 0;

  GeoBlendInfo  geoBIdst;
  m_pcInterSearch->getGeoBlendCand( cu, mergeCtx[GEO_TM_OFF], -1, geoBIdst, geoBlendInfo, &numGeoBlendInfoCand );

  double  bestEstimGeoBlendPredCost = MAX_DOUBLE;

  idxBufGeo = idxBlendGeoFirst;
  for (int i = 0; i < numGeoBlendInfoCand; i++)
  {
    GeoBlendInfo& geoBI = geoBlendInfo[i];

    pu.cu->blendModel.copy( geoBI.blendModel );

    int  mergeCand0 = geoBI.mergeCand[0];
    int  mergeCand1 = geoBI.mergeCand[1];

    PelUnitBuf  blendBuffer = m_acMergeBuffer[idxBufGeo].getBuf(localUnitArea);

    m_pcInterSearch->weightedBlend( pu, blendBuffer, geoBuffer[mergeCand0], geoBuffer[mergeCand1], false, true, true );

    DistParam& distParamGeo = distParamWholeBlkGeoBlend;   // SAD

    distParamGeo.cur.buf     = blendBuffer.Y().buf;
    distParamGeo.cur.stride  = blendBuffer.Y().stride;
    Distortion sad = distParamGeo.distFunc(distParamGeo);

    geoBI.sad           = sad;        // sad bi-prediction geo-blended distortion
    geoBI.idxBufGeo     = idxBufGeo;
  }

  double geoBlendFlagCost[2] = { 0.0, 0.0 };
  double geoBlendCandCost[GEO_BLEND_MAX_NUM_CANDS];
  for ( int idx = 0; idx < numGeoBlendInfoCand; idx++ )
  {
#if JVET_AG0164_AFFINE_GPM
    geoBlendCandCost[idx] = geoMergeIdxCost[ std::min(idx, numRegularGpmMergeCand - 1) ];
#else
    geoBlendCandCost[idx] = geoMergeIdxCost[ std::min(idx, maxNumMergeCandidates - 1) ];
#endif

    GeoBlendInfo& geoBI = geoBlendInfo[idx];
    geoBI.uiCost = (double)geoBI.sad + geoBlendCandCost[idx] + geoBlendFlagCost[1];   // estimated RD-cost 
    bestEstimGeoBlendPredCost = std::min( geoBI.uiCost, bestEstimGeoBlendPredCost );

    geoBI.iMergeIdx = idx;
  }

  pu.geoBldIdx = 0;
#endif

  int wIdx = floorLog2(cu.lwidth()) - GEO_MIN_CU_LOG2;
  int hIdx = floorLog2(cu.lheight()) - GEO_MIN_CU_LOG2;
  Distortion sadSmall = 0, sadLarge = 0;
  int maskStride = 0, maskStride2 = 0, stepX = 1;
  Pel* sadMask;
  static_vector<int, GEO_NUM_PARTITION_MODE> selGeoModeList;
  static_vector<double, GEO_NUM_PARTITION_MODE> selGeoModeRDList;
  static_vector<int, 5> mergeCandList0[GEO_NUM_PARTITION_MODE];
  static_vector<int, 5> mergeCandList1[GEO_NUM_PARTITION_MODE];
  static_vector<int, 5> mmvdCandList0[GEO_NUM_PARTITION_MODE];
  static_vector<int, 5> mmvdCandList1[GEO_NUM_PARTITION_MODE];
  static_vector<double, 5> sadCostList0[GEO_NUM_PARTITION_MODE];
  static_vector<double, 5> sadCostList1[GEO_NUM_PARTITION_MODE];
#if JVET_Y0065_GPM_INTRA
  static_vector<int, GEO_MAX_NUM_INTRA_CANDS> intraCandList0[GEO_NUM_PARTITION_MODE];
  static_vector<int, GEO_MAX_NUM_INTRA_CANDS> intraCandList1[GEO_NUM_PARTITION_MODE];
  static_vector<double, GEO_MAX_NUM_INTRA_CANDS> intraSadCostList0[GEO_NUM_PARTITION_MODE];
  static_vector<double, GEO_MAX_NUM_INTRA_CANDS> intraSadCostList1[GEO_NUM_PARTITION_MODE];
#endif

  for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
  {
    int16_t angle = g_geoParams[splitDir][0];
    if (g_angle2mirror[angle] == 2)
    {
      stepX = 1;
      maskStride = -GEO_WEIGHT_MASK_SIZE;
      maskStride2 = -(int)cu.lwidth();
      sadMask = &g_geoEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
#if GPM_CURVE
      if(g_geoParams[splitDir][1] == 0 && wIdx == hIdx && wIdx >= GEO_CURVE_MIN_CU_IDX && CU::isGeoCURVEAvailable(splitDir,cu.lwidth()))
      {
        sadMask = &g_geoCurveEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
      }
#endif
    }
    else if (g_angle2mirror[angle] == 1)
    {
      stepX = -1;
      maskStride2 = cu.lwidth();
      maskStride = GEO_WEIGHT_MASK_SIZE;
      sadMask = &g_geoEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
#if GPM_CURVE
      if(g_geoParams[splitDir][1] == 0 && wIdx == hIdx && wIdx >= GEO_CURVE_MIN_CU_IDX && CU::isGeoCURVEAvailable(splitDir,cu.lwidth()))
      {
        sadMask = &g_geoCurveEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
      }
#endif
    }
    else
    {
      stepX = 1;
      maskStride = GEO_WEIGHT_MASK_SIZE;
      maskStride2 = -(int)cu.lwidth();
      sadMask = &g_geoEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
#if GPM_CURVE
      if(g_geoParams[splitDir][1] == 0 && wIdx == hIdx && wIdx >= GEO_CURVE_MIN_CU_IDX && CU::isGeoCURVEAvailable(splitDir,cu.lwidth()))
      {
        sadMask = &g_geoCurveEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
      }
#endif
    }

#if JVET_AG0164_AFFINE_GPM
    for (uint8_t mergeCand = 0; mergeCand < GEO_MAX_ALL_INTER_UNI_CANDS + GEO_MAX_NUM_INTRA_CANDS; mergeCand++)
    {
      if ((mergeCand < maxNumMergeCandidates && mrgDuplicated[mergeCand]) || (mergeCand >= maxNumMergeCandidates && mergeCand < GEO_MAX_ALL_INTER_UNI_CANDS))
      {
        continue;
      }
      double tempCost;

      int isAffine = (mergeCand < numRegularGpmMergeCand || mergeCand >= GEO_MAX_ALL_INTER_UNI_CANDS) ? 0 : 1;
      int mergeIdx = mergeCand < numRegularGpmMergeCand? mergeCand :(mergeCand - numRegularGpmMergeCand );
      if (mergeCand < GEO_MAX_ALL_INTER_UNI_CANDS)
      { 
#else
#if JVET_Y0065_GPM_INTRA
    for (uint8_t mergeCand = 0; mergeCand < GEO_MAX_NUM_UNI_CANDS + GEO_MAX_NUM_INTRA_CANDS; mergeCand++)
#else
    for (uint8_t mergeCand = 0; mergeCand < maxNumMergeCandidates; mergeCand++)
#endif
    {
#if JVET_Y0065_GPM_INTRA
      if ((mergeCand < maxNumMergeCandidates && mrgDuplicated[mergeCand]) || (mergeCand >= maxNumMergeCandidates && mergeCand < GEO_MAX_NUM_UNI_CANDS))
#else
      if (mrgDuplicated[mergeCand])
#endif
      {
        continue;
      }
#if JVET_Y0065_GPM_INTRA
      double tempCost;
      if (mergeCand < GEO_MAX_NUM_UNI_CANDS)
      {
#endif
#endif
      m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), geoTempBuf[mergeCand].Y().buf, geoTempBuf[mergeCand].Y().stride, sadMask, maskStride, stepX, maskStride2, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y);
      sadLarge = distParam.distFunc(distParam);

#if JVET_AG0164_AFFINE_GPM
      tempCost = (double)sadLarge + (isAffine? geoAffMergeIdxCost[mergeIdx]: geoMergeIdxCost[mergeIdx]) + geoMMVDFlagCost[0];
      tempCost += geoAffineFlagCost[isAffine];
      if (!isAffine)
      {
        tempCost+= geoIntraFlag0Cost[0];
      }
#else
#if JVET_Y0065_GPM_INTRA
      tempCost = (double)sadLarge + geoMergeIdxCost[mergeCand] + geoIntraFlag0Cost[0] + geoMMVDFlagCost[0];
#else
      double tempCost = (double)sadLarge + geoMergeIdxCost[mergeCand] + geoMMVDFlagCost[0];
#endif
#endif

      m_geoMMVDCostList.insert(splitDir, 0, mergeCand, 0, tempCost);
      sortCandList(tempCost, mergeCand, 0, sadCostList0[splitDir], mergeCandList0[splitDir], mmvdCandList0[splitDir], m_numCandPerPar);
      sadSmall = sadWholeBlk[mergeCand] - sadLarge;

#if JVET_AG0164_AFFINE_GPM
      tempCost = (double)sadSmall + (isAffine ? geoAffMergeIdxCost[mergeIdx] : geoMergeIdxCost[mergeIdx]) + geoMMVDFlagCost[0];
      tempCost += geoAffineFlagCost[isAffine];
      if (!isAffine)
      {
        tempCost += geoIntraFlag0Cost[0];
      }
#else
#if JVET_Y0065_GPM_INTRA
      tempCost = (double)sadSmall + geoMergeIdxCost[mergeCand] + geoIntraFlag0Cost[0] + geoMMVDFlagCost[0];
#else
      tempCost = (double)sadSmall + geoMergeIdxCost[mergeCand] + geoMMVDFlagCost[0];
#endif
#endif

      m_geoMMVDCostList.insert(splitDir, 1, mergeCand, 0, tempCost);
      sortCandList(tempCost, mergeCand, 0, sadCostList1[splitDir], mergeCandList1[splitDir], mmvdCandList1[splitDir], m_numCandPerPar);
#if JVET_Y0065_GPM_INTRA
      }
      else
      {
#if JVET_AG0164_AFFINE_GPM
        int intraIdx = mergeCand - GEO_MAX_ALL_INTER_UNI_CANDS;
#else
        int intraIdx = mergeCand - GEO_MAX_NUM_UNI_CANDS;
#endif
        int rdobuffer = intraRDOBufIdx[geoIntraMPMList[splitDir][0][intraIdx]];
        m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), geoIntraTempBuf[rdobuffer].Y().buf, geoIntraTempBuf[rdobuffer].Y().stride, sadMask, maskStride, stepX, maskStride2, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y);
        sadLarge = distParam.distFunc(distParam);
        tempCost = (double)sadLarge + geoIntraIdxCost[intraIdx] + geoIntraFlag0Cost[1] + geoMMVDFlagCost[0];
#if JVET_AG0164_AFFINE_GPM
        tempCost += geoAffineFlagCost[0];
#endif
        m_geoMMVDCostList.insert(splitDir, 0, mergeCand, 0, tempCost);
        sortIntraCandList(tempCost, mergeCand, intraSadCostList0[splitDir], intraCandList0[splitDir]);

        if (geoIntraMPMList[splitDir][0][intraIdx] != geoIntraMPMList[splitDir][1][intraIdx])
        {
          rdobuffer = intraRDOBufIdx[geoIntraMPMList[splitDir][1][intraIdx]];
          m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), geoIntraTempBuf[rdobuffer].Y().buf, geoIntraTempBuf[rdobuffer].Y().stride, sadMask, maskStride, stepX, maskStride2, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y);
          sadLarge = distParam.distFunc(distParam);
        }
        sadSmall = sadIntraWholeBlk[rdobuffer] - sadLarge;
        tempCost = (double)sadSmall + geoIntraIdxCost[intraIdx] + geoIntraFlag0Cost[1] + geoMMVDFlagCost[0];
#if JVET_AG0164_AFFINE_GPM
        tempCost += geoAffineFlagCost[0];
#endif
        m_geoMMVDCostList.insert(splitDir, 1, mergeCand, 0, tempCost);
        sortIntraCandList(tempCost, mergeCand, intraSadCostList1[splitDir], intraCandList1[splitDir]);
      }
#endif
    }
    updateCandList(splitDir, (sadCostList0[splitDir][0] + sadCostList1[splitDir][0]), selGeoModeList, selGeoModeRDList, GEO_NUM_PARTITION_MODE);
  }

  static_vector<int, GEO_MAX_TRY_WEIGHTED_SAD> geoSplitDirList;
  static_vector<int, GEO_MAX_TRY_WEIGHTED_SAD> geoMergeCand0;
  static_vector<int, GEO_MAX_TRY_WEIGHTED_SAD> geoMergeCand1;
  static_vector<int, GEO_MAX_TRY_WEIGHTED_SAD> geoMmvdCand0;
  static_vector<int, GEO_MAX_TRY_WEIGHTED_SAD> geoMmvdCand1;
  static_vector<double, GEO_MAX_TRY_WEIGHTED_SAD> geoSADCostList;
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  if (sps.getUseAltGPMSplitModeCode())
  {
    m_pcInterSearch->initGeoAngleSelection(pu
#if JVET_Y0065_GPM_INTRA
                                         , m_pcIntraSearch, geoIntraMPMList
#endif
    );
  }
#if TM_MRG
  const int tmMmvdBufIdx0 = GPM_EXT_MMVD_MAX_REFINE_NUM + 1;
  const int tmMmvdBufIdx1 = GPM_EXT_MMVD_MAX_REFINE_NUM + 1;
#endif
#endif

  for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
  {
#if JVET_Y0065_GPM_INTRA
    int numCandMerge0 = min(m_numCandPerPar, (int)mergeCandList0[splitDir].size());
    int numCandIntra0 = (int)intraCandList0[splitDir].size();
    int numCandPart0 = numCandMerge0 + numCandIntra0;
    for (int candIdx0 = 0; candIdx0 < numCandPart0; candIdx0++)
    {
      int mergeCand0 = candIdx0 < numCandMerge0 ? mergeCandList0[splitDir][candIdx0] : intraCandList0[splitDir][candIdx0-numCandMerge0];
      int numCandMerge1 = min(m_numCandPerPar, (int)mergeCandList1[splitDir].size());
      int numCandIntra1 = candIdx0 < numCandMerge0 ? (int)intraCandList1[splitDir].size() : 0;
      int numCandPart1 = numCandMerge1 + numCandIntra1;
      int candStart1 = (bUseOnlyOneVector && candIdx0 < numCandMerge0) ? numCandMerge1 : 0;
      for (int candIdx1 = candStart1; candIdx1 < numCandPart1; candIdx1++)
      {
        int mergeCand1 = candIdx1 < numCandMerge1 ? mergeCandList1[splitDir][candIdx1] : intraCandList1[splitDir][candIdx1-numCandMerge1];
#if JVET_AG0164_AFFINE_GPM
        int isAffine0 = (mergeCand0 >= numRegularGpmMergeCand && mergeCand0 < GEO_MAX_ALL_INTER_UNI_CANDS) ? 1 : 0;
        int isAffine1 = (mergeCand1 >= numRegularGpmMergeCand && mergeCand1 < GEO_MAX_ALL_INTER_UNI_CANDS) ? 1 : 0;
        pu.affineGPM[0] = isAffine0;
        pu.affineGPM[1] = isAffine1;
#endif
#else
    int numCandPart0 = min(m_numCandPerPar, (int)mergeCandList0[splitDir].size());
    int numCandPart1 = min(m_numCandPerPar, (int)mergeCandList1[splitDir].size());
    for (int candIdx0 = 0; candIdx0 < numCandPart0; candIdx0++)
    {
      for (int candIdx1 = 0; candIdx1 < numCandPart1; candIdx1++)
      {
        int mergeCand0 = mergeCandList0[splitDir][candIdx0];
        int mergeCand1 = mergeCandList1[splitDir][candIdx1];
#endif

        if (mergeCand0 == mergeCand1)
        {
          continue;
        }

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
        int geoSyntaxMode = std::numeric_limits<uint8_t>::max();
        if(sps.getUseAltGPMSplitModeCode())
        {
          m_pcInterSearch->setGeoSplitModeToSyntaxTable(pu, mergeCtxRegular, mergeCand0, mergeCtxRegular, mergeCand1
#if JVET_AG0164_AFFINE_GPM
                                                      , affMergeCtx
#endif
#if JVET_Y0065_GPM_INTRA
                                                      , m_pcIntraSearch
#endif
          );
          geoSyntaxMode = m_pcInterSearch->convertGeoSplitModeToSyntax(splitDir, mergeCand0, mergeCand1);
          if (geoSyntaxMode == std::numeric_limits<uint8_t>::max())
          {
            continue;
          }
        }
#endif

        double tempCost = m_geoMMVDCostList.singleDistList[0][splitDir][mergeCand0][0].cost + m_geoMMVDCostList.singleDistList[1][splitDir][mergeCand1][0].cost;
        tempCost = tempCost +
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
                  (geoSyntaxMode == std::numeric_limits<uint8_t>::max() ? geoModeCost[splitDir] : geoSigModeCost[geoSyntaxMode]);
#else
                   geoModeCost[splitDir];
#endif
#if TM_MRG
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
        if (sps.getUseGPMTMMode() 

#if JVET_AG0164_AFFINE_GPM
          && (!isAffine0 && !isAffine1)
          && mergeCand0 < GEO_MAX_ALL_INTER_UNI_CANDS&& mergeCand1 < GEO_MAX_ALL_INTER_UNI_CANDS
#else
#if JVET_Y0065_GPM_INTRA
          && mergeCand0 < GEO_MAX_NUM_UNI_CANDS && mergeCand1 < GEO_MAX_NUM_UNI_CANDS
#endif
#endif
          )
#else
#if JVET_Y0065_GPM_INTRA
        if (sps.getUseDMVDMode() && mergeCand0 < GEO_MAX_NUM_UNI_CANDS && mergeCand1 < GEO_MAX_NUM_UNI_CANDS)
#else
        if (sps.getUseDMVDMode())
#endif
#endif
        {
          tempCost += geoTMFlagCost[0];
        }
#endif
        updateGeoMMVDCandList(tempCost, splitDir, mergeCand0, mergeCand1, 0, 0, geoSADCostList, geoSplitDirList, geoMergeCand0, geoMergeCand1, geoMmvdCand0, geoMmvdCand1, numSATDCands);
      }
    }
  }
#if JVET_AG0164_AFFINE_GPM
  pu.affineGPM[0] = 0;
  pu.affineGPM[1] = 0;
#endif
  static_vector<uint8_t, GEO_MAX_TRY_WEIGHTED_SAD>  geoRdModeList;
  static_vector<bool, GEO_MAX_TRY_WEIGHTED_SAD>  isNonMMVDListIdx;
  static_vector<int, GEO_MAX_TRY_WEIGHTED_SAD>  geoPartitionModeList;
  static_vector<double, GEO_MAX_TRY_WEIGHTED_SAD>  geocandCostList;
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
  static_vector<uint8_t, GEO_MAX_TRY_WEIGHTED_SAD> geoBldList;
#endif

  DistParam distParamSAD2;
  const bool useHadamard = !tempCS->slice->getDisableSATDForRD();
  m_pcRdCost->setDistParam(distParamSAD2, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y(), sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, useHadamard);

  int numberGeoCandChecked = (int)geoSADCostList.size();
  int geoNumMrgSATDCand = min(GEO_MAX_TRY_WEIGHTED_SATD, numberGeoCandChecked);
  int numStoredCands = geoNumMrgSATDCand;
  for (uint8_t candidateIdx = 0; candidateIdx < numberGeoCandChecked; candidateIdx++)
  {
    int splitDir = geoSplitDirList[candidateIdx];
    int mergeCand0 = geoMergeCand0[candidateIdx];
    int mergeCand1 = geoMergeCand1[candidateIdx];
    bool mmvdFlag0 = false;
    bool mmvdFlag1 = false;

#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
    for (uint8_t bldIdx = 0; bldIdx < GEO_BLENDING_NUM; bldIdx++)
    {
      geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx] = m_acGeoWeightedBuffer[candidateIdx * GEO_BLENDING_NUM + bldIdx].getBuf(localUnitArea);
#if JVET_Y0065_GPM_INTRA
#if JVET_AG0164_AFFINE_GPM
      int isIntra0 = (mergeCand0 >= GEO_MAX_ALL_INTER_UNI_CANDS) ? 1 : 0;
      int isIntra1 = (mergeCand1 >= GEO_MAX_ALL_INTER_UNI_CANDS) ? 1 : 0;
      int isAffine0 = (mergeCand0 >= numRegularGpmMergeCand && mergeCand0 < GEO_MAX_ALL_INTER_UNI_CANDS) ? 1 : 0;
      int isAffine1 = (mergeCand1 >= numRegularGpmMergeCand && mergeCand1 < GEO_MAX_ALL_INTER_UNI_CANDS) ? 1 : 0;
      pu.affineGPM[0] = isAffine0;
      pu.affineGPM[1] = isAffine1;
      if (isAffine0 || isAffine1)
      {
        CHECK(!affGPMValid, "AffGPMFlag should be false");
      }
#else
      int isIntra0 = (mergeCand0 >= GEO_MAX_NUM_UNI_CANDS) ? 1 : 0;
      int isIntra1 = (mergeCand1 >= GEO_MAX_NUM_UNI_CANDS) ? 1 : 0;
#endif
      int candidateSAD = candidateIdx * GEO_BLENDING_NUM + bldIdx;
      if (isIntra0 || isIntra1)
      {
        PelUnitBuf predSrc0, predSrc1;
        if (isIntra0)
        {
#if JVET_AG0164_AFFINE_GPM
          int intraIdx0 = mergeCand0 - GEO_MAX_ALL_INTER_UNI_CANDS;
#else
          int intraIdx0 = mergeCand0 - GEO_MAX_NUM_UNI_CANDS;
#endif
          int rdoBuffer = intraRDOBufIdx[geoIntraMPMList[splitDir][0][intraIdx0]];
          predSrc0 = geoIntraBuffer[rdoBuffer];
        }
        else
        {
          predSrc0 = geoTempBuf[mergeCand0];
        }
        if (isIntra1)
        {
#if JVET_AG0164_AFFINE_GPM
          int intraIdx1 = mergeCand1 - GEO_MAX_ALL_INTER_UNI_CANDS;
#else
          int intraIdx1 = mergeCand1 - GEO_MAX_NUM_UNI_CANDS;
#endif
          int rdoBuffer = intraRDOBufIdx[geoIntraMPMList[splitDir][1][intraIdx1]];
          predSrc1 = geoIntraBuffer[rdoBuffer];
        }
        else
        {
          predSrc1 = geoTempBuf[mergeCand1];
        }
        if (pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
        {
          if (!isIntra0) // Inter+Intra
          {
            geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx].Y().rspSignal(predSrc0.Y(), m_pcReshape->getFwdLUT());
            predSrc0 = geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx];
          }
          else if (!isIntra1) // Intra+Inter
          {
            geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx].Y().rspSignal(predSrc1.Y(), m_pcReshape->getFwdLUT());
            predSrc1 = geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx];
          }
          m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, bldIdx, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx], predSrc0, predSrc1);
          candidateSAD = GEO_MAX_TRY_WEIGHTED_SAD * GEO_BLENDING_NUM;
          geoCombinations[candidateSAD] = m_acGeoWeightedBuffer[candidateSAD].getBuf(localUnitArea);
          geoCombinations[candidateSAD].Y().rspSignal(geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx].Y(), m_pcReshape->getInvLUT());
        }
        else
        {
          m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, bldIdx, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx], predSrc0, predSrc1);
        }
      }
      else
#endif
        m_pcInterSearch->weightedGeoBlk(pu, splitDir, bldIdx, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx], geoBuffer[mergeCand0], geoBuffer[mergeCand1]);
#if JVET_Y0065_GPM_INTRA
      distParamSAD2.cur = geoCombinations[candidateSAD].Y();
#else
      distParamSAD2.cur = geoCombinations[candidateIdx].Y();
#endif
      Distortion sad = distParamSAD2.distFunc(distParamSAD2);

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
      int geoSyntaxMode = std::numeric_limits<uint8_t>::max();
      if (sps.getUseAltGPMSplitModeCode())
      {
        geoSyntaxMode = m_pcInterSearch->convertGeoSplitModeToSyntax(splitDir, mergeCand0, mergeCand1);
        CHECK(geoSyntaxMode < 0 || geoSyntaxMode >= GEO_NUM_SIG_PARTMODE, "Invalid GEO split direction!");
      }
#endif

#if JVET_AG0164_AFFINE_GPM
      double updateCost =(geoSyntaxMode == std::numeric_limits<uint8_t>::max() ? geoModeCost[splitDir] : geoSigModeCost[geoSyntaxMode])
        + geoMMVDFlagCost[mmvdFlag0];
      updateCost += geoAffineFlagCost[isAffine0];
      int intraIdx0 = mergeCand0 - GEO_MAX_ALL_INTER_UNI_CANDS;
      int intraIdx1 = mergeCand1 - GEO_MAX_ALL_INTER_UNI_CANDS;
      if (!isAffine0)
      {
        updateCost+=  geoIntraFlag0Cost[isIntra0];        
      }
      int mergeIdx0 = mergeCand0 - isAffine0 * numRegularGpmMergeCand;
      int mergeIdx1 = mergeCand1 - isAffine1 * numRegularGpmMergeCand;

      updateCost += (isIntra0 ? geoIntraIdxCost[intraIdx0] : (isAffine0? geoAffMergeIdxCost[mergeIdx0]: geoMergeIdxCost[mergeIdx0]));

      if (!bUseOnlyOneVector||isIntra0)
      { 
        updateCost +=  geoMMVDFlagCost[mmvdFlag1];
        updateCost += geoAffineFlagCost[isAffine1];
        if (!isAffine1)
        {
          updateCost += geoIntraFlag1Cost[isIntra0][isIntra1];
        }
        if (isIntra1)
        {
          updateCost +=  geoIntraIdxCost[intraIdx1];
        }
        else if (m_fastGpmMmvdSearch && !isIntra0 && (isAffine0 == isAffine1))
        {
          updateCost += (isAffine1 ? geoAffMergeIdxCost[mergeIdx1 > mergeIdx0 ? (mergeIdx1 - 1) : mergeIdx1] : geoMergeIdxCost[mergeIdx1 > mergeIdx0 ? (mergeIdx1 - 1) : mergeIdx1]);
        }
        else
        {
          updateCost += (isAffine1 ? geoAffMergeIdxCost[mergeIdx1] : geoMergeIdxCost[mergeIdx1]);
        }
      }
#else
#if JVET_Y0065_GPM_INTRA
      double updateCost =
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
                          (geoSyntaxMode == std::numeric_limits<uint8_t>::max() ? geoModeCost[splitDir] : geoSigModeCost[geoSyntaxMode])
#else
                          geoModeCost[splitDir]
#endif
                        + geoMMVDFlagCost[mmvdFlag0] + geoIntraFlag0Cost[isIntra0] + geoMMVDFlagCost[mmvdFlag1] + geoIntraFlag1Cost[isIntra0][isIntra1];
      int intraIdx0 = mergeCand0 - GEO_MAX_NUM_UNI_CANDS;
      int intraIdx1 = mergeCand1 - GEO_MAX_NUM_UNI_CANDS;
      updateCost += (isIntra0 ? geoIntraIdxCost[intraIdx0] : geoMergeIdxCost[mergeCand0]);
      updateCost += (isIntra1 ? geoIntraIdxCost[intraIdx1] : ((m_fastGpmMmvdSearch && !isIntra0) ? geoMergeIdxCost[mergeCand1 > mergeCand0 ? (mergeCand1 - 1) : mergeCand1] : geoMergeIdxCost[mergeCand1]));
#else
      double updateCost =
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
                          (geoSyntaxMode == std::numeric_limits<uint8_t>::max() ? geoModeCost[splitDir] : geoSigModeCost[geoSyntaxMode])
#else
                          geoModeCost[splitDir]
#endif
                        + geoMergeIdxCost[mergeCand0] + (m_fastGpmMmvdSearch ? geoMergeIdxCost[mergeCand1 > mergeCand0 ? (mergeCand1 - 1) : mergeCand1] : geoMergeIdxCost[mergeCand1]) + geoMMVDFlagCost[mmvdFlag0] + geoMMVDFlagCost[mmvdFlag1];
#endif
#endif
#if TM_MRG
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
      if (sps.getUseGPMTMMode()
#if JVET_Y0065_GPM_INTRA
        && !isIntra0 && !isIntra1
#endif
#if JVET_AG0164_AFFINE_GPM
        && ( !isAffine0 && !isAffine1)
#endif
        )
#else
#if JVET_Y0065_GPM_INTRA
      if (sps.getUseDMVDMode() && !isIntra0 && !isIntra1)
#else
      if (sps.getUseDMVDMode())
#endif
#endif
      {
        updateCost += geoTMFlagCost[0];
      }
#endif
      updateCost += geoBldFlagCost[bldIdx];
      updateCost += (double)sad;
      orderCandList(candidateIdx, true, splitDir, updateCost, bldIdx, geoRdModeList, isNonMMVDListIdx, geoPartitionModeList, geocandCostList, geoBldList, numStoredCands);
    }
#else
    geoCombinations[candidateIdx] = m_acGeoWeightedBuffer[candidateIdx].getBuf(localUnitArea);
#if JVET_Y0065_GPM_INTRA
    int isIntra0 = (mergeCand0 >= GEO_MAX_NUM_UNI_CANDS) ? 1 : 0;
    int isIntra1 = (mergeCand1 >= GEO_MAX_NUM_UNI_CANDS) ? 1 : 0;
    uint8_t candidateSAD = candidateIdx;
    if (isIntra0 || isIntra1)
    {
      PelUnitBuf predSrc0, predSrc1;
      if (isIntra0)
      {
        int intraIdx0 = mergeCand0 - GEO_MAX_NUM_UNI_CANDS;
        int rdoBuffer = intraRDOBufIdx[geoIntraMPMList[splitDir][0][intraIdx0]];
        predSrc0 = geoIntraBuffer[rdoBuffer];
      }
      else
      {
        predSrc0 = geoTempBuf[mergeCand0];
      }
      if (isIntra1)
      {
        int intraIdx1 = mergeCand1 - GEO_MAX_NUM_UNI_CANDS;
        int rdoBuffer = intraRDOBufIdx[geoIntraMPMList[splitDir][1][intraIdx1]];
        predSrc1 = geoIntraBuffer[rdoBuffer];
      }
      else
      {
        predSrc1 = geoTempBuf[mergeCand1];
      }
      if (pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
      {
        if (!isIntra0) // Inter+Intra
        {
          geoCombinations[candidateIdx].Y().rspSignal(predSrc0.Y(), m_pcReshape->getFwdLUT());
          predSrc0 = geoCombinations[candidateIdx];
        }
        else if (!isIntra1) // Intra+Inter
        {
          geoCombinations[candidateIdx].Y().rspSignal(predSrc1.Y(), m_pcReshape->getFwdLUT());
          predSrc1 = geoCombinations[candidateIdx];
        }
        m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx], predSrc0, predSrc1);
        candidateSAD = GEO_MAX_TRY_WEIGHTED_SAD;
        geoCombinations[GEO_MAX_TRY_WEIGHTED_SAD] = m_acGeoWeightedBuffer[GEO_MAX_TRY_WEIGHTED_SAD].getBuf(localUnitArea);
        geoCombinations[GEO_MAX_TRY_WEIGHTED_SAD].Y().rspSignal(geoCombinations[candidateIdx].Y(), m_pcReshape->getInvLUT());
      }
      else
      {
        m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx], predSrc0, predSrc1);
      }
    }
    else
#endif
    m_pcInterSearch->weightedGeoBlk(pu, splitDir, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx], geoBuffer[mergeCand0], geoBuffer[mergeCand1]);
#if JVET_Y0065_GPM_INTRA
    distParamSAD2.cur = geoCombinations[candidateSAD].Y();
#else
    distParamSAD2.cur = geoCombinations[candidateIdx].Y();
#endif
    Distortion sad = distParamSAD2.distFunc(distParamSAD2);

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
    int geoSyntaxMode = std::numeric_limits<uint8_t>::max();
    if(sps.getUseAltGPMSplitModeCode())
    {
      geoSyntaxMode = m_pcInterSearch->convertGeoSplitModeToSyntax(splitDir, mergeCand0, mergeCand1);
      CHECK(geoSyntaxMode < 0 || geoSyntaxMode >= GEO_NUM_SIG_PARTMODE, "Invalid GEO split direction!");
    }
#endif
#if JVET_Y0065_GPM_INTRA
    double updateCost =
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
                        (geoSyntaxMode == std::numeric_limits<uint8_t>::max() ? geoModeCost[splitDir] : geoSigModeCost[geoSyntaxMode])
#else
                        geoModeCost[splitDir]
#endif
                      + geoMMVDFlagCost[mmvdFlag0] + geoIntraFlag0Cost[isIntra0] + geoMMVDFlagCost[mmvdFlag1] + geoIntraFlag1Cost[isIntra0][isIntra1];
    int intraIdx0 = mergeCand0 - GEO_MAX_NUM_UNI_CANDS;
    int intraIdx1 = mergeCand1 - GEO_MAX_NUM_UNI_CANDS;
    updateCost += (isIntra0 ? geoIntraIdxCost[intraIdx0] : geoMergeIdxCost[mergeCand0]);
    updateCost += (isIntra1 ? geoIntraIdxCost[intraIdx1] : ((m_fastGpmMmvdSearch && !isIntra0) ? geoMergeIdxCost[mergeCand1 > mergeCand0 ? (mergeCand1 - 1) : mergeCand1] : geoMergeIdxCost[mergeCand1]));
#else
    double updateCost =
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
                        (geoSyntaxMode == std::numeric_limits<uint8_t>::max() ? geoModeCost[splitDir] : geoSigModeCost[geoSyntaxMode])
#else
                        geoModeCost[splitDir]
#endif
                      + geoMergeIdxCost[mergeCand0] + (m_fastGpmMmvdSearch ? geoMergeIdxCost[mergeCand1 > mergeCand0 ? (mergeCand1 - 1) : mergeCand1] : geoMergeIdxCost[mergeCand1]) + geoMMVDFlagCost[mmvdFlag0] + geoMMVDFlagCost[mmvdFlag1];
#endif
#if TM_MRG
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    if (sps.getUseGPMTMMode()
#if JVET_Y0065_GPM_INTRA
      && !isIntra0 && !isIntra1
#endif
      )
#else
#if JVET_Y0065_GPM_INTRA
    if (sps.getUseDMVDMode() && !isIntra0 && !isIntra1)
#else
    if (sps.getUseDMVDMode())
#endif
#endif
    {
      updateCost += geoTMFlagCost[0];
    }
#endif
    updateCost += (double)sad;
    orderCandList(candidateIdx, true, splitDir, updateCost, geoRdModeList, isNonMMVDListIdx, geoPartitionModeList, geocandCostList, numStoredCands);
#endif
  }
#if JVET_AG0164_AFFINE_GPM
  pu.affineGPM[0] = 0;
  pu.affineGPM[1] = 0;
#endif

  for (uint8_t i = 1; i < geoNumMrgSATDCand; i++)
  {
#if MERGE_ENC_OPT
    if (geocandCostList[i] > MRG_FAST_RATIO * geocandCostList[0] || geocandCostList[i] > getMergeBestSATDCost())
#else
    if (geocandCostList[i] > MRG_FAST_RATIO * geocandCostList[0] || geocandCostList[i] > getMergeBestSATDCost() || geocandCostList[i] > getAFFBestSATDCost())
#endif
    {
      geoNumMrgSATDCand = i;
      break;
    }
  }

#if JVET_Y0065_GPM_INTRA
  for (uint8_t i = 0; i < geoNumMrgSATDCand; i++)
#else
  for (uint8_t i = 0; i < geoNumMrgSATDCand && isChromaEnabled(pu.chromaFormat); i++)
#endif
  {
    uint8_t candidateIdx = geoRdModeList[i];
    int splitDir = geoSplitDirList[candidateIdx];
    int mergeCand0 = geoMergeCand0[candidateIdx];
    int mergeCand1 = geoMergeCand1[candidateIdx];
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
    uint8_t bldIdx = geoBldList[i];
#endif

#if JVET_Y0065_GPM_INTRA
    PelUnitBuf predSrc0, predSrc1;
#if JVET_AG0164_AFFINE_GPM
    int isIntra0 = (mergeCand0 >= GEO_MAX_ALL_INTER_UNI_CANDS) ? 1 : 0;
    int isIntra1 = (mergeCand1 >= GEO_MAX_ALL_INTER_UNI_CANDS) ? 1 : 0;
    int isAffine0 = (mergeCand0 >= numRegularGpmMergeCand && mergeCand0 < GEO_MAX_ALL_INTER_UNI_CANDS) ? 1 : 0;
    int isAffine1 = (mergeCand1 >= numRegularGpmMergeCand && mergeCand1 < GEO_MAX_ALL_INTER_UNI_CANDS) ? 1 : 0;

    pu.affineGPM[0] = isAffine0;
    pu.affineGPM[1] = isAffine1;
#else
    int isIntra0 = (mergeCand0 >= GEO_MAX_NUM_UNI_CANDS) ? 1 : 0;
    int isIntra1 = (mergeCand1 >= GEO_MAX_NUM_UNI_CANDS) ? 1 : 0;
#endif

    if (!isChromaEnabled(pu.chromaFormat) && !isIntra0 && !isIntra1)
    {
      continue;
    }
    if (isIntra0)
    {
#if JVET_AG0164_AFFINE_GPM
      int intraIdx0 = mergeCand0 - GEO_MAX_ALL_INTER_UNI_CANDS;
#else
      int intraIdx0 = mergeCand0 - GEO_MAX_NUM_UNI_CANDS;
#endif
      uint8_t intraPred = geoIntraMPMList[splitDir][0][intraIdx0];
      int rdoBuffer = intraRDOBufIdx[intraPred];
      if (isChromaEnabled(pu.chromaFormat) && !isGeoIntraChromaAvail[rdoBuffer])
      {
        pu.intraDir[1] = intraPred;
        pu.gpmIntraFlag = true;
        m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cb());
        m_pcIntraSearch->predIntraAng(COMPONENT_Cb, geoIntraBuffer[rdoBuffer].Cb(), pu);
        m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cr());
        m_pcIntraSearch->predIntraAng(COMPONENT_Cr, geoIntraBuffer[rdoBuffer].Cr(), pu);
        pu.gpmIntraFlag = false;
        isGeoIntraChromaAvail[rdoBuffer] = 2;
      }
      predSrc0 = geoIntraBuffer[rdoBuffer];
    }
#if JVET_AG0164_AFFINE_GPM
    else if (isAffine0)
    {
      if (isChromaEnabled(pu.chromaFormat) && !isGeoChromaAvail[mergeCand0])
      {
        affMergeCtx.setAffMergeInfo(pu, mergeCand0);
        if (pu.refIdx[0] >= 0)
        {
          PU::setAllAffineMv(pu, pu.mvAffi[0][0], pu.mvAffi[0][1], pu.mvAffi[0][2], REF_PIC_LIST_0);
        }
        if (pu.refIdx[1] >= 0)
        {
          PU::setAllAffineMv(pu, pu.mvAffi[1][0], pu.mvAffi[1][1], pu.mvAffi[1][2], REF_PIC_LIST_1);
        }
        m_pcInterSearch->motionCompensation(pu, geoBuffer[mergeCand0], REF_PIC_LIST_X, false, true);
        isGeoChromaAvail[mergeCand0] = 1;
        pu.cu->affine = false;
      }
      predSrc0 = geoTempBuf[mergeCand0];
    }
#endif
    else
    {
    if (isChromaEnabled(pu.chromaFormat) && !isGeoChromaAvail[mergeCand0])
#else
    if (!isGeoChromaAvail[mergeCand0])
#endif
    {
#if TM_MRG
      mergeCtx[GEO_TM_OFF].setMergeInfo(pu, mergeCand0);
#else
      mergeCtx.setMergeInfo(pu, mergeCand0);
#endif
#if JVET_AE0046_BI_GPM
      if (refinePossible[mergeCand0])
      {
        CHECK(refinePossible[mergeCand0] != PU::checkBDMVRCondition(pu, true), "The BDMVR condition mismatches with the ones in previous luma loop");
        pu.bdmvrRefine = true;
        PU::spanPuMv2DmvrBuffer(pu, m_mvBufBDMVR4GPM[0], m_mvBufBDMVR4GPM[1]);
        ::memcpy(m_pcInterSearch->getBdofSubPuMvOffset(), m_mvBufEncBDOF4GPM[GEO_TM_OFF][mergeCand0], sizeof(Mv)* BDOF_SUBPU_MAX_NUM);
        m_pcInterSearch->setLumaBdofReady( true );
      }
#endif
      m_pcInterSearch->motionCompensation(pu, geoBuffer[mergeCand0], REF_PIC_LIST_X, false, true);
#if JVET_AE0046_BI_GPM
      pu.bdmvrRefine = false;
      m_pcInterSearch->setLumaBdofReady(false);
#endif
#if JVET_Y0065_GPM_INTRA
      isGeoChromaAvail[mergeCand0] = 1;
#else
      isGeoChromaAvail[mergeCand0] = true;
#endif
    }

#if JVET_Y0065_GPM_INTRA
      predSrc0 = geoTempBuf[mergeCand0];
    }

    if (isIntra1)
    {
#if JVET_AG0164_AFFINE_GPM
      int intraIdx1 = mergeCand1 - GEO_MAX_ALL_INTER_UNI_CANDS;
#else
      int intraIdx1 = mergeCand1 - GEO_MAX_NUM_UNI_CANDS;
#endif
      uint8_t intraPred = geoIntraMPMList[splitDir][1][intraIdx1];
      int rdoBuffer = intraRDOBufIdx[intraPred];
      if (isChromaEnabled(pu.chromaFormat) && !isGeoIntraChromaAvail[rdoBuffer])
      {
        pu.intraDir[1] = intraPred;
        pu.gpmIntraFlag = true;
        m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cb());
        m_pcIntraSearch->predIntraAng(COMPONENT_Cb, geoIntraBuffer[rdoBuffer].Cb(), pu);
        m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cr());
        m_pcIntraSearch->predIntraAng(COMPONENT_Cr, geoIntraBuffer[rdoBuffer].Cr(), pu);
        pu.gpmIntraFlag = false;
        isGeoIntraChromaAvail[rdoBuffer] = 2;
      }
      predSrc1 = geoIntraBuffer[rdoBuffer];
    }
#if JVET_AG0164_AFFINE_GPM
    else if (isAffine1)
    {
      if (isChromaEnabled(pu.chromaFormat) && !isGeoChromaAvail[mergeCand1])
      {
        affMergeCtx.setAffMergeInfo(pu, mergeCand1);
        if (pu.refIdx[0] >= 0)
        {
          PU::setAllAffineMv(pu, pu.mvAffi[0][0], pu.mvAffi[0][1], pu.mvAffi[0][2], REF_PIC_LIST_0);
        }
        if (pu.refIdx[1] >= 0)
        {
          PU::setAllAffineMv(pu, pu.mvAffi[1][0], pu.mvAffi[1][1], pu.mvAffi[1][2], REF_PIC_LIST_1);
        }
        m_pcInterSearch->motionCompensation(pu, geoBuffer[mergeCand1], REF_PIC_LIST_X, false, true);
        isGeoChromaAvail[mergeCand1] = 1;
        pu.cu->affine = false;
      }
      predSrc1 = geoTempBuf[mergeCand1];
    }
#endif
    else
    {
    if (isChromaEnabled(pu.chromaFormat) && !isGeoChromaAvail[mergeCand1])
#else
    if (!isGeoChromaAvail[mergeCand1])
#endif
    {
#if TM_MRG
      mergeCtx[GEO_TM_OFF].setMergeInfo(pu, mergeCand1);
#else
      mergeCtx.setMergeInfo(pu, mergeCand1);
#endif
#if JVET_AE0046_BI_GPM
      if (refinePossible[mergeCand1])
      {
        pu.bdmvrRefine = true;
        PU::spanPuMv2DmvrBuffer(pu, m_mvBufBDMVR4GPM[0], m_mvBufBDMVR4GPM[1]);
        ::memcpy(m_pcInterSearch->getBdofSubPuMvOffset(), m_mvBufEncBDOF4GPM[GEO_TM_OFF][mergeCand1], sizeof(Mv) * BDOF_SUBPU_MAX_NUM);
        m_pcInterSearch->setLumaBdofReady(true);
      }
#endif
      m_pcInterSearch->motionCompensation(pu, geoBuffer[mergeCand1], REF_PIC_LIST_X, false, true);
#if JVET_AE0046_BI_GPM
      pu.bdmvrRefine = false;
      m_pcInterSearch->setLumaBdofReady(false);
#endif
#if JVET_Y0065_GPM_INTRA
      isGeoChromaAvail[mergeCand1] = 1;
#else
      isGeoChromaAvail[mergeCand1] = true;
#endif
    }
#if JVET_Y0065_GPM_INTRA
      predSrc1 = geoTempBuf[mergeCand1];
    }
#endif

#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
    geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx] = m_acGeoWeightedBuffer[candidateIdx * GEO_BLENDING_NUM + bldIdx].getBuf(localUnitArea);
#else
    geoCombinations[candidateIdx] = m_acGeoWeightedBuffer[candidateIdx].getBuf(localUnitArea);
#endif
#if JVET_Y0065_GPM_INTRA
    if (isIntra0 || isIntra1)
    {
      if (isChromaEnabled(pu.chromaFormat))
      {
        if (!isIntra0)
        {
          if (isGeoChromaAvail[mergeCand0] < 2)
          {
            geoTempBuf[mergeCand0].Cb().roundToOutputBitdepth(geoBuffer[mergeCand0].Cb(), cu.slice->clpRng(COMPONENT_Cb));
            geoTempBuf[mergeCand0].Cr().roundToOutputBitdepth(geoBuffer[mergeCand0].Cr(), cu.slice->clpRng(COMPONENT_Cr));
            isGeoChromaAvail[mergeCand0] = 2;
          }
        }
        else if (!isIntra1)
        {
          if (isGeoChromaAvail[mergeCand1] < 2)
          {
            geoTempBuf[mergeCand1].Cb().roundToOutputBitdepth(geoBuffer[mergeCand1].Cb(), cu.slice->clpRng(COMPONENT_Cb));
            geoTempBuf[mergeCand1].Cr().roundToOutputBitdepth(geoBuffer[mergeCand1].Cr(), cu.slice->clpRng(COMPONENT_Cr));
            isGeoChromaAvail[mergeCand1] = 2;
          }
        }
      }

      int interMergeCand = isIntra0 ? mergeCand1 : mergeCand0;
#if JVET_AG0164_AFFINE_GPM
      if (interMergeCand < numRegularGpmMergeCand)
      {
#endif
#if TM_MRG
      mergeCtx[GEO_TM_OFF].setMergeInfo(pu, interMergeCand);
#else
      mergeCtx.setMergeInfo(pu, interMergeCand );
#endif
#if ENABLE_OBMC
#if JVET_W0123_TIMD_FUSION
#if JVET_AE0046_BI_GPM
      pu.gpmDmvrRefinePart0 = isIntra0 ? false : (refinePossible[mergeCand0]);
      pu.gpmDmvrRefinePart1 = isIntra1 ? false : (refinePossible[mergeCand1]);
      CHECK(pu.gpmDmvrRefinePart0 && pu.gpmDmvrRefinePart1, "This is not possible to have two parts with refine possible since one is intra");
      Mv* bdofSubPuMvOffset = nullptr;
      if (pu.gpmDmvrRefinePart0 || pu.gpmDmvrRefinePart1)
      {
        bdofSubPuMvOffset = m_mvBufEncBDOF4GPM[GEO_TM_OFF][interMergeCand];
      }
      PU::spanMotionInfo2(pu, MergeCtx(),
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                    0,
#endif
              nullptr, nullptr, bdofSubPuMvOffset);

      pu.gpmDmvrRefinePart0 = false;
      pu.gpmDmvrRefinePart1 = false;
#else
      PU::spanMotionInfo2(pu);
#endif
#else
      PU::spanMotionInfo(pu);
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
      if (!isIntra0)
      {
        geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx].copyFrom(predSrc0);
        predSrc0 = geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx];
      }
      else
      {
        geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx].copyFrom(predSrc1);
        predSrc1 = geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx];
      }
#else
      if (!isIntra0)
      {
        geoCombinations[candidateIdx].copyFrom(predSrc0);
        predSrc0 = geoCombinations[candidateIdx];
      }
      else
      {
        geoCombinations[candidateIdx].copyFrom(predSrc1);
        predSrc1 = geoCombinations[candidateIdx];
      }
#endif
      cu.isobmcMC = true;
      cu.obmcFlag = true;
      m_pcInterSearch->subBlockOBMC(pu, !isIntra0 ? &predSrc0 : &predSrc1);
      cu.isobmcMC = false;
#endif
#if JVET_AG0164_AFFINE_GPM
      }
      else
      {
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
        if (!isIntra0)
        {
          geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx].copyFrom(predSrc0);
          predSrc0 = geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx];

          affMergeCtx.setAffMergeInfo(pu, mergeCand0);
          PU::spanMotionInfo2(pu, MergeCtx());
          cu.isobmcMC = true;
          cu.obmcFlag = true;
          m_pcInterSearch->subBlockOBMC(pu, &predSrc0);
          pu.cu->affine = false;
          pu.cu->isobmcMC = false;

        }
        else
        {
          geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx].copyFrom(predSrc1);
          predSrc1 = geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx];

          affMergeCtx.setAffMergeInfo(pu, mergeCand1);
          PU::spanMotionInfo2(pu, MergeCtx());
          cu.isobmcMC = true;
          cu.obmcFlag = true;
          m_pcInterSearch->subBlockOBMC(pu, &predSrc1);
          pu.cu->affine = false;
          pu.cu->isobmcMC = false;

        }
#else
        if (!isIntra0)
        {
          geoCombinations[candidateIdx].copyFrom(predSrc0);
          predSrc0 = geoCombinations[candidateIdx];
        }
        else
        {
          geoCombinations[candidateIdx].copyFrom(predSrc1);
          predSrc1 = geoCombinations[candidateIdx];
        }
#endif
      }
#endif
      if (pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
      {
        if (!isIntra0) // Inter+Intra
        {
          predSrc0.Y().rspSignal(predSrc0.Y(), m_pcReshape->getFwdLUT());
        }
        else if (!isIntra1) // Intra+Inter
        {
          predSrc1.Y().rspSignal(predSrc1.Y(), m_pcReshape->getFwdLUT());
        }
      }
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
      m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, bldIdx, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx], predSrc0, predSrc1);
      m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, bldIdx, CHANNEL_TYPE_CHROMA, geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx], predSrc0, predSrc1);
#else
      m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx], predSrc0, predSrc1);
      m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, CHANNEL_TYPE_CHROMA, geoCombinations[candidateIdx], predSrc0, predSrc1);
#endif
    }
    else
#endif
#if JVET_AG0164_AFFINE_GPM
    if (isAffine0 || isAffine1)
    {
      if (isChromaEnabled(pu.chromaFormat))
      {
        if (isGeoChromaAvail[mergeCand0] < 2)
        {
          geoTempBuf[mergeCand0].Cb().roundToOutputBitdepth(geoBuffer[mergeCand0].Cb(), cu.slice->clpRng(COMPONENT_Cb));
          geoTempBuf[mergeCand0].Cr().roundToOutputBitdepth(geoBuffer[mergeCand0].Cr(), cu.slice->clpRng(COMPONENT_Cr));
          isGeoChromaAvail[mergeCand0] = 2;
        }

        if (isGeoChromaAvail[mergeCand1] < 2)
        {
          geoTempBuf[mergeCand1].Cb().roundToOutputBitdepth(geoBuffer[mergeCand1].Cb(), cu.slice->clpRng(COMPONENT_Cb));
          geoTempBuf[mergeCand1].Cr().roundToOutputBitdepth(geoBuffer[mergeCand1].Cr(), cu.slice->clpRng(COMPONENT_Cr));
          isGeoChromaAvail[mergeCand1] = 2;
        }
      }

      int geoCombIdx = candidateIdx * GEO_BLENDING_NUM + bldIdx;
      PelUnitBuf obmcBuf0 = m_ciipBuffer[0].getBuf(localUnitArea);   // Borrow the CIIP buffer
      PelUnitBuf obmcBuf1 = m_ciipBuffer[1].getBuf(localUnitArea);   // Borrow the CIIP buffer
      if (mergeCand0 < numRegularGpmMergeCand)
      {
        pu.gpmDmvrRefinePart0 = refinePossible[mergeCand0];
      }
      performOBMCOnOnePartition(pu, mergeCand0, geoCombIdx, predSrc0, obmcBuf0); // Borrow the CIIP buffer
      pu.gpmDmvrRefinePart0 = pu.gpmDmvrRefinePart1 = false;

      if (mergeCand1 < numRegularGpmMergeCand)
      {
        pu.gpmDmvrRefinePart1 = refinePossible[mergeCand1];
      }
      performOBMCOnOnePartition(pu, mergeCand1, geoCombIdx, predSrc1, obmcBuf1);
      pu.gpmDmvrRefinePart0 = pu.gpmDmvrRefinePart1 = false;
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
      m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, bldIdx, CHANNEL_TYPE_LUMA, geoCombinations[geoCombIdx], obmcBuf0, obmcBuf1);
      m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, bldIdx, CHANNEL_TYPE_CHROMA, geoCombinations[geoCombIdx], obmcBuf0, obmcBuf1);
#else
      m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx], predSrc0, predSrc1);
      m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, CHANNEL_TYPE_CHROMA, geoCombinations[candidateIdx], predSrc0, predSrc1);
#endif
    }
    else
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
    m_pcInterSearch->weightedGeoBlk(pu, splitDir, bldIdx, CHANNEL_TYPE_CHROMA, geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx], geoBuffer[mergeCand0], geoBuffer[mergeCand1]);
#else
    m_pcInterSearch->weightedGeoBlk(pu, splitDir, CHANNEL_TYPE_CHROMA, geoCombinations[candidateIdx], geoBuffer[mergeCand0], geoBuffer[mergeCand1]);
#endif
  }
#if JVET_AG0164_AFFINE_GPM
  pu.affineGPM[0] = 0;
  pu.affineGPM[1] = 0;
#endif

  bool geocandHasNoResidual[GEO_MAX_TRY_WEIGHTED_SAD];
  bool bestIsSkip = false;
  std::memset(geocandHasNoResidual, false, GEO_MAX_TRY_WEIGHTED_SAD * sizeof(bool));

  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;
  tempCS->initStructData(encTestMode.qp);
  uint8_t iteration = 2, iterationBegin = 0;
  for (uint8_t noResidualPass = iterationBegin; noResidualPass < iteration; ++noResidualPass)
  {
    for (uint8_t mrgHADIdx = 0; mrgHADIdx < geoNumMrgSATDCand; mrgHADIdx++)
    {
      uint8_t candidateIdx = geoRdModeList[mrgHADIdx];
      if (((noResidualPass != 0) && geocandHasNoResidual[candidateIdx])
        || ((noResidualPass == 0) && bestIsSkip))
      {
        continue;
      }
      CodingUnit &cu = tempCS->addCU(tempCS->area, pm.chType);
      pm.setCUData(cu);
      cu.predMode = MODE_INTER;
      cu.slice = tempCS->slice;
      cu.tileIdx = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
      cu.qp = encTestMode.qp;
      cu.affine = false;
      cu.mtsFlag = false;
#if INTER_LIC
      cu.licFlag = false;
#endif
      cu.bcwIdx = BCW_DEFAULT;
      cu.geoFlag = true;
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
      cu.geoBlendFlag = false;
#endif
#if JVET_AG0061_INTER_LFNST_NSPT
      cu.lfnstFlag = false;
      cu.lfnstIdx = 0;
#endif
      cu.imv = 0;
      cu.mmvdSkip = false;
      cu.skip = false;
      cu.mipFlag = false;
      cu.bdpcmMode = 0;
      PredictionUnit &pu = tempCS->addPU(cu, pm.chType);
      pu.mergeFlag = true;
      pu.regularMergeFlag = false;
      pu.geoSplitDir = geoSplitDirList[candidateIdx];
      pu.geoMergeIdx0 = geoMergeCand0[candidateIdx];
      pu.geoMergeIdx1 = geoMergeCand1[candidateIdx];
#if JVET_AE0046_BI_GPM
      PU::setGpmDirMode(pu);
#endif
#if JVET_AG0164_AFFINE_GPM
      pu.affineGPM[0] = pu.affineGPM[1] = 0;
      if (pu.geoMergeIdx0 < GEO_MAX_ALL_INTER_UNI_CANDS && pu.geoMergeIdx0 >= numRegularGpmMergeCand)
      {
        pu.geoMergeIdx0 -= numRegularGpmMergeCand;
        pu.affineGPM[0] = 1;
      }
      if (pu.geoMergeIdx1 < GEO_MAX_ALL_INTER_UNI_CANDS && pu.geoMergeIdx1 >= numRegularGpmMergeCand)
      {
        pu.geoMergeIdx1 -= numRegularGpmMergeCand;
        pu.affineGPM[1] = 1;
      }
#endif
#if JVET_Y0065_GPM_INTRA
#if JVET_AG0164_AFFINE_GPM
      pu.gpmIntraFlag = pu.geoMergeIdx0 >= GEO_MAX_ALL_INTER_UNI_CANDS || pu.geoMergeIdx1 >= GEO_MAX_ALL_INTER_UNI_CANDS;
      if (pu.geoMergeIdx0 >= GEO_MAX_ALL_INTER_UNI_CANDS)
      {
        memcpy(m_pcIntraSearch->m_intraMPM, geoIntraMPMList[pu.geoSplitDir][0], sizeof(uint8_t)*GEO_MAX_NUM_INTRA_CANDS);
      }
      if (pu.geoMergeIdx1 >= GEO_MAX_ALL_INTER_UNI_CANDS)
      {
        memcpy(m_pcIntraSearch->m_intraMPM +GEO_MAX_NUM_INTRA_CANDS, geoIntraMPMList[pu.geoSplitDir][1], sizeof(uint8_t)*GEO_MAX_NUM_INTRA_CANDS);
      }
#else
      pu.gpmIntraFlag = pu.geoMergeIdx0 >= GEO_MAX_NUM_UNI_CANDS || pu.geoMergeIdx1 >= GEO_MAX_NUM_UNI_CANDS;
      if (pu.geoMergeIdx0 >= GEO_MAX_NUM_UNI_CANDS)
      {
        memcpy(m_pcIntraSearch->m_intraMPM, geoIntraMPMList[pu.geoSplitDir][0], sizeof(uint8_t)*GEO_MAX_NUM_INTRA_CANDS);
      }
      if (pu.geoMergeIdx1 >= GEO_MAX_NUM_UNI_CANDS)
      {
        memcpy(m_pcIntraSearch->m_intraMPM +GEO_MAX_NUM_INTRA_CANDS, geoIntraMPMList[pu.geoSplitDir][1], sizeof(uint8_t)*GEO_MAX_NUM_INTRA_CANDS);
      }
#endif
#if ENABLE_DIMD
      cu.dimdMode = dimdMode;
#endif
#if JVET_W0123_TIMD_FUSION
      cu.timdMode = timdMode;
#if JVET_AC0094_REF_SAMPLES_OPT
      cu.timdModeCheckWA = timdModeCheckWA;
#endif
#endif
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
      pu.geoBldIdx = geoBldList[mrgHADIdx];
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
      if(sps.getUseAltGPMSplitModeCode())
      {
#if JVET_AG0164_AFFINE_GPM
          int cand0 = pu.geoMergeIdx0;
          int cand1 = pu.geoMergeIdx1;
          if (pu.affineGPM[0])
          {
            cand0 += numRegularGpmMergeCand;
          }
          if (pu.affineGPM[1])
          {
            cand1 += numRegularGpmMergeCand;
          }
          int geoSyntaxMode = m_pcInterSearch->convertGeoSplitModeToSyntax(pu.geoSplitDir, cand0, cand1);
#else
        int geoSyntaxMode = m_pcInterSearch->convertGeoSplitModeToSyntax(pu.geoSplitDir, pu.geoMergeIdx0, pu.geoMergeIdx1);
#endif
        CHECK(geoSyntaxMode < 0 || geoSyntaxMode >= GEO_NUM_SIG_PARTMODE, "Invalid GEO split direction!");
        pu.geoSyntaxMode = (uint8_t)geoSyntaxMode;
      }
#endif
#if TM_MRG
      pu.tmMergeFlag = false;
      pu.geoTmFlag0 = false;
      pu.geoTmFlag1 = false;
#endif
      pu.geoMMVDFlag0 = false;
      pu.geoMMVDFlag1 = false;

      pu.mmvdMergeFlag = false;
      pu.mmvdMergeIdx = MAX_UCHAR;

#if JVET_AE0046_BI_GPM
      pu.gpmDmvrRefinePart0 = pu.geoMergeIdx0 < refinePossible.size() ? refinePossible[pu.geoMergeIdx0] : false;
      pu.gpmDmvrRefinePart1 = pu.geoMergeIdx1 < refinePossible.size() ? refinePossible[pu.geoMergeIdx1] : false;
#endif

#if TM_MRG
      MergeCtx *mergeTmCtx0 = nullptr;
      MergeCtx *mergeTmCtx1 = nullptr;
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
#if JVET_AE0046_BI_GPM
      PU::spanGeoMMVDMotionInfo(pu, mergeCtx[GEO_TM_OFF]
#if JVET_AG0164_AFFINE_GPM
        , affMergeCtx
#endif
        , *mergeTmCtx0, *mergeTmCtx1, pu.geoSplitDir, pu.geoMergeIdx0, pu.geoMergeIdx1, pu.geoTmFlag0, pu.geoMMVDFlag0, pu.geoMMVDIdx0, pu.geoTmFlag1, pu.geoMMVDFlag1, pu.geoMMVDIdx1, pu.geoBldIdx, m_pcIntraSearch->m_intraMPM,
        pu.gpmDmvrRefinePart0, pu.gpmDmvrRefinePart1, pu.gpmDmvrRefinePart0 ? m_mvBufEncBDOF4GPM[GEO_TM_OFF][pu.geoMergeIdx0] : nullptr, pu.gpmDmvrRefinePart1 ? m_mvBufEncBDOF4GPM[GEO_TM_OFF][pu.geoMergeIdx1] : nullptr);
#else
      PU::spanGeoMMVDMotionInfo(pu, mergeCtx[GEO_TM_OFF], *mergeTmCtx0, *mergeTmCtx1, pu.geoSplitDir, pu.geoMergeIdx0, pu.geoMergeIdx1, pu.geoTmFlag0, pu.geoMMVDFlag0, pu.geoMMVDIdx0, pu.geoTmFlag1, pu.geoMMVDFlag1, pu.geoMMVDIdx1, pu.geoBldIdx, m_pcIntraSearch->m_intraMPM);
#endif
#else
      PU::spanGeoMMVDMotionInfo(pu, mergeCtx[GEO_TM_OFF], *mergeTmCtx0, *mergeTmCtx1, pu.geoSplitDir, pu.geoMergeIdx0, pu.geoMergeIdx1, pu.geoTmFlag0, pu.geoMMVDFlag0, pu.geoMMVDIdx0, pu.geoTmFlag1, pu.geoMMVDFlag1, pu.geoMMVDIdx1);
#endif
#else
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
      PU::spanGeoMMVDMotionInfo(pu, mergeCtx, pu.geoSplitDir, pu.geoMergeIdx0, pu.geoMergeIdx1, pu.geoMMVDFlag0, pu.geoMMVDIdx0, pu.geoMMVDFlag1, pu.geoMMVDIdx1, pu.geoBldIdx);
#else
      PU::spanGeoMMVDMotionInfo(pu, mergeCtx, pu.geoSplitDir, pu.geoMergeIdx0, pu.geoMergeIdx1, pu.geoMMVDFlag0, pu.geoMMVDIdx0, pu.geoMMVDFlag1, pu.geoMMVDIdx1);
#endif
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
      tempCS->getPredBuf().copyFrom(geoCombinations[candidateIdx * GEO_BLENDING_NUM + pu.geoBldIdx]);
#else
      tempCS->getPredBuf().copyFrom(geoCombinations[candidateIdx]);
#endif
#if ENABLE_OBMC
#if JVET_Y0065_GPM_INTRA
      if (!pu.gpmIntraFlag
#if JVET_AG0164_AFFINE_GPM
        &&!pu.affineGPM[0] && !pu.affineGPM[1]
#endif
        )
      {
#endif
      cu.isobmcMC = true;
      cu.obmcFlag = true;
      m_pcInterSearch->subBlockOBMC(pu);
      cu.isobmcMC = false;
#if JVET_Y0065_GPM_INTRA
      }
#endif
#endif

      xEncodeInterResidual(tempCS, bestCS, pm, encTestMode, noResidualPass, (noResidualPass == 0 ? &geocandHasNoResidual[candidateIdx] : NULL));

      if (m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip)
      {
        bestIsSkip = bestCS->getCU(pm.chType)->rootCbf == 0;
      }
      tempCS->initStructData(encTestMode.qp);
    }
  }

  CodingUnit *bestCU = bestCS->getCU(CHANNEL_TYPE_LUMA);
  bool skipGPMMMVD = false;
  if (geoNumMrgSATDCand > 0)
  {
    if (bestCU->skip && !bestCU->geoFlag && !bestCU->affine && !bestCU->mmvdSkip && !bestCU->firstPU->mmvdMergeFlag)
    {
      skipGPMMMVD = true;
    }
    else if (bestCU->affine && bestCU->skip && (bestCU->lwidth() >= 16 || bestCU->lheight() >= 16))
    {
      skipGPMMMVD = true;
    }
  }
#if JVET_AG0164_AFFINE_GPM
  bool isBaseMergeCandIncluded[GEO_MAX_ALL_INTER_UNI_CANDS];
  std::memset(isBaseMergeCandIncluded, false, GEO_MAX_ALL_INTER_UNI_CANDS * sizeof(bool));
#else
  bool isBaseMergeCandIncluded[GEO_MAX_NUM_UNI_CANDS];
  std::memset(isBaseMergeCandIncluded, false, GEO_MAX_NUM_UNI_CANDS * sizeof(bool));
#endif
  bool isGPMModeIncludedForMMVD[GEO_NUM_PARTITION_MODE];
  std::memset(isGPMModeIncludedForMMVD, false, GEO_NUM_PARTITION_MODE * sizeof(bool));

  if (!skipGPMMMVD)
  {
    skipGPMMMVD = (selGeoModeRDList[0] > (bestNormalMrgCost * 1.1));
  }

  if (!skipGPMMMVD)
  {
    if (isSecondPass)
    {
      for (int i = 0; i < relatedCU.numGeoDirCand; i++)
      {
        isGPMModeIncludedForMMVD[relatedCU.geoDirCandList[i]] = true;
        if (m_fastGpmMmvdRelatedCU)
        {
#if JVET_Y0065_GPM_INTRA
#if JVET_AG0164_AFFINE_GPM
          if (relatedCU.geoMrgIdx0List[i] < GEO_MAX_ALL_INTER_UNI_CANDS)
#else
          if (relatedCU.geoMrgIdx0List[i] < GEO_MAX_NUM_UNI_CANDS)
#endif
#endif
          isBaseMergeCandIncluded[relatedCU.geoMrgIdx0List[i]] = true;
#if JVET_Y0065_GPM_INTRA
#if JVET_AG0164_AFFINE_GPM
          if (relatedCU.geoMrgIdx1List[i] < GEO_MAX_ALL_INTER_UNI_CANDS)
#else
          if (relatedCU.geoMrgIdx1List[i] < GEO_MAX_NUM_UNI_CANDS)
#endif
#endif
          isBaseMergeCandIncluded[relatedCU.geoMrgIdx1List[i]] = true;
        }
      }
      if (!m_fastGpmMmvdRelatedCU)
      {
#if JVET_AG0164_AFFINE_GPM
        std::memset(isBaseMergeCandIncluded, true, GEO_MAX_ALL_INTER_UNI_CANDS * sizeof(bool));
#else
        std::memset(isBaseMergeCandIncluded, true, GEO_MAX_NUM_UNI_CANDS * sizeof(bool));
#endif
      }
    }
    else
    {
      double dirCostThresh = (selGeoModeRDList[0] * 1.2);
      isGPMModeIncludedForMMVD[selGeoModeList[0]] = true;
      isBaseMergeCandIncluded[mergeCandList0[selGeoModeList[0]][0]] = true;
      isBaseMergeCandIncluded[mergeCandList1[selGeoModeList[0]][0]] = true;

      for (int i = 1; i < m_maxNumGPMDirFirstPass; i++)
      {
        if (selGeoModeRDList[i] > dirCostThresh)
        {
          break;
        }
        else
        {
          isGPMModeIncludedForMMVD[selGeoModeList[i]] = true;
          isBaseMergeCandIncluded[mergeCandList0[selGeoModeList[i]][0]] = true;
          isBaseMergeCandIncluded[mergeCandList1[selGeoModeList[i]][0]] = true;
        }
      }
      if (m_includeMoreMMVDCandFirstPass)
      {
        int num = 0;
        // add more cands from best combo results obtained in weighted blended nonmmvd combo
        num = min((int)geocandCostList.size(), GEO_MAX_TRY_WEIGHTED_SATD);
        for (int i = 0; i < num; i++)
        {
          if (m_fastGpmMmvdSearch && (geocandCostList[i] > dirCostThresh))
          {
            break;
          }
          else
          {
            isGPMModeIncludedForMMVD[geoPartitionModeList[i]] = true;
#if JVET_Y0065_GPM_INTRA
#if JVET_AG0164_AFFINE_GPM
            if (geoMergeCand0[geoRdModeList[i]] < GEO_MAX_ALL_INTER_UNI_CANDS)
#else
            if (geoMergeCand0[geoRdModeList[i]] < GEO_MAX_NUM_UNI_CANDS)
#endif
#endif
            isBaseMergeCandIncluded[geoMergeCand0[geoRdModeList[i]]] = true;
#if JVET_Y0065_GPM_INTRA
#if JVET_AG0164_AFFINE_GPM
            if (geoMergeCand1[geoRdModeList[i]] < GEO_MAX_ALL_INTER_UNI_CANDS)
#else
            if (geoMergeCand1[geoRdModeList[i]] < GEO_MAX_NUM_UNI_CANDS)
#endif
#endif
            isBaseMergeCandIncluded[geoMergeCand1[geoRdModeList[i]]] = true;
          }
        }
      }
    }
  }

  if (!skipGPMMMVD)
  {
    CodingUnit &cu = tempCS->addCU(tempCS->area, pm.chType);
    pm.setCUData(cu);
    cu.predMode = MODE_INTER;
    cu.slice = tempCS->slice;
    cu.tileIdx = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
    cu.qp = encTestMode.qp;
    cu.affine = false;
    cu.mtsFlag = false;
#if INTER_LIC
    cu.licFlag = false;
#endif
    cu.bcwIdx = BCW_DEFAULT;
    cu.geoFlag = true;
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
    cu.geoBlendFlag = false;
#endif
#if JVET_AG0061_INTER_LFNST_NSPT
    cu.lfnstFlag = false;
    cu.lfnstIdx  = 0;
#endif
    cu.imv = 0;
    cu.mmvdSkip = false;
    cu.skip = false;
    cu.mipFlag = false;
    cu.bdpcmMode = 0;

    PredictionUnit &pu = tempCS->addPU(cu, pm.chType);
    pu.mergeFlag = true;
    pu.regularMergeFlag = false;
#if JVET_AE0046_BI_GPM
    PU::setGpmDirMode(pu);
#endif
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
    pu.tmMergeFlag = false;
#endif
#if JVET_AG0164_AFFINE_GPM
    pu.affineGPM[0] = pu.affineGPM[1] = 0;
#endif

    bool simpleGPMMMVDStep = (m_pcEncCfg->getIntraPeriod() == -1);
#if JVET_AG0164_AFFINE_GPM
    double mmvdMrgCost[GEO_MAX_ALL_INTER_UNI_CANDS][GPM_EXT_MMVD_MAX_REFINE_NUM];
#else
    double mmvdMrgCost[GEO_MAX_NUM_UNI_CANDS][GPM_EXT_MMVD_MAX_REFINE_NUM];
#endif
#if JVET_AG0164_AFFINE_GPM
    for (uint8_t mergeCand = 0; mergeCand < numRegularGpmMergeCand; mergeCand++)
#else
    for (uint8_t mergeCand = 0; mergeCand < maxNumMergeCandidates; mergeCand++)
#endif
    {
      if (mrgDuplicated[mergeCand])
      {
        continue;
      }
      if (!isBaseMergeCandIncluded[mergeCand])
      {
        continue;
      }
#if JVET_AG0164_AFFINE_GPM
      int isAffine = mergeCand < numRegularGpmMergeCand ? 0 : 1;
#endif
      for (uint8_t mmvdCand = 0; mmvdCand < (extMMVD ? GPM_EXT_MMVD_MAX_REFINE_NUM : GPM_MMVD_MAX_REFINE_NUM); mmvdCand++)
      {
        if (simpleGPMMMVDStep)
        {
          int mmvdStep = (extMMVD ? (mmvdCand >> 3) : (mmvdCand >> 2));
          if (mmvdStep >= 5 && (!m_fastGpmMmvdSearch || (m_fastGpmMmvdSearch && !isSecondPass)))
          {
            continue;
          }
        }
        geoMMVDBuf[mergeCand][mmvdCand] = m_acGeoMMVDBuffer[mergeCand][mmvdCand].getBuf(localUnitArea);

#if JVET_AG0164_AFFINE_GPM
        if (isAffine)
        {
          affMergeCtx.setAffMergeInfo(pu, mergeCand, mmvdCand);
        }
        else
#endif
#if TM_MRG
        mergeCtx[GEO_TM_OFF].setGeoMmvdMergeInfo(pu, mergeCand, mmvdCand);
#else
        mergeCtx.setGeoMmvdMergeInfo(pu, mergeCand, mmvdCand);
#endif
        if (m_pcEncCfg->getMCTSEncConstraint() && (!(MCTSHelper::checkMvBufferForMCTSConstraint(pu))))
        {
          tempCS->initStructData(encTestMode.qp);
          return;
        }
        m_pcInterSearch->motionCompensation(pu, geoMMVDBuf[mergeCand][mmvdCand], REF_PIC_LIST_X, true, false);
        geoMMVDTempBuf[mergeCand][mmvdCand] = m_acGeoMMVDTmpBuffer[mergeCand][mmvdCand].getBuf(localUnitArea);
        geoMMVDTempBuf[mergeCand][mmvdCand].Y().copyFrom(geoMMVDBuf[mergeCand][mmvdCand].Y());
        geoMMVDTempBuf[mergeCand][mmvdCand].Y().roundToOutputBitdepth(geoMMVDTempBuf[mergeCand][mmvdCand].Y(), cu.slice->clpRng(COMPONENT_Y));
        distParamWholeBlk.cur.buf = geoMMVDTempBuf[mergeCand][mmvdCand].Y().buf;
        distParamWholeBlk.cur.stride = geoMMVDTempBuf[mergeCand][mmvdCand].Y().stride;
        sadMMVDWholeBlk[mergeCand][mmvdCand] = distParamWholeBlk.distFunc(distParamWholeBlk);

#if JVET_AG0164_AFFINE_GPM
        mmvdMrgCost[mergeCand][mmvdCand] = sadMMVDWholeBlk[mergeCand][mmvdCand] + 
          (isAffine? geoAffMergeIdxCost[mergeCand - numRegularGpmMergeCand]: geoMergeIdxCost[mergeCand]) + geoAffineFlagCost[isAffine] + geoMMVDFlagCost[1] + geoMMVDIdxCost[mmvdCand];
#else
        mmvdMrgCost[mergeCand][mmvdCand] = sadMMVDWholeBlk[mergeCand][mmvdCand] + geoMergeIdxCost[mergeCand] + geoMMVDFlagCost[1] + geoMMVDIdxCost[mmvdCand];
#endif
        if (mmvdMrgCost[mergeCand][mmvdCand] < bestMrgCost)
        {
          bestMrgCost = mmvdMrgCost[mergeCand][mmvdCand];
        }
#if JVET_AG0164_AFFINE_GPM
        pu.cu->affine = false;
#endif
      }
    }

    double mrgCostThres = (bestMrgCost * 3.0);
    for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
    {
      if (!isGPMModeIncludedForMMVD[splitDir])
      {
        continue;
      }
      int16_t angle = g_geoParams[splitDir][0];
      if (g_angle2mirror[angle] == 2)
      {
        stepX = 1;
        maskStride = -GEO_WEIGHT_MASK_SIZE;
        maskStride2 = -(int)cu.lwidth();
        sadMask = &g_geoEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
#if GPM_CURVE
        if(g_geoParams[splitDir][1] == 0 && wIdx == hIdx && wIdx >= GEO_CURVE_MIN_CU_IDX && CU::isGeoCURVEAvailable(splitDir,cu.lwidth()))
        {
          sadMask = &g_geoCurveEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
        }
#endif
      }
      else if (g_angle2mirror[angle] == 1)
      {
        stepX = -1;
        maskStride2 = cu.lwidth();
        maskStride = GEO_WEIGHT_MASK_SIZE;
        sadMask = &g_geoEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
#if GPM_CURVE
        if(g_geoParams[splitDir][1] == 0 && wIdx == hIdx && wIdx >= GEO_CURVE_MIN_CU_IDX && CU::isGeoCURVEAvailable(splitDir,cu.lwidth()))
        {
          sadMask = &g_geoCurveEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
        }
#endif
      }
      else
      {
        stepX = 1;
        maskStride = GEO_WEIGHT_MASK_SIZE;
        maskStride2 = -(int)cu.lwidth();
        sadMask = &g_geoEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
#if GPM_CURVE
        if(g_geoParams[splitDir][1] == 0 && wIdx == hIdx && wIdx >= GEO_CURVE_MIN_CU_IDX && CU::isGeoCURVEAvailable(splitDir,cu.lwidth()))
        {
          sadMask = &g_geoCurveEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
        }
#endif
      }
#if JVET_AG0164_AFFINE_GPM
      for (uint8_t mergeCand = 0; mergeCand < numRegularGpmMergeCand; mergeCand++)
#else
      for (uint8_t mergeCand = 0; mergeCand < maxNumMergeCandidates; mergeCand++)	  
#endif
      {
        if (mrgDuplicated[mergeCand])
        {
          continue;
        }
        if (!isBaseMergeCandIncluded[mergeCand])
        {
          continue;
        }
        for (uint8_t mmvdCand = 0; mmvdCand < (extMMVD ? GPM_EXT_MMVD_MAX_REFINE_NUM : GPM_MMVD_MAX_REFINE_NUM); mmvdCand++)
        {
          if (simpleGPMMMVDStep)
          {
            int mmvdStep = (extMMVD ? (mmvdCand >> 3) : (mmvdCand >> 2));
            if (mmvdStep >= 5 && (!m_fastGpmMmvdSearch || (m_fastGpmMmvdSearch && !isSecondPass)))
            {
              continue;
            }
          }
          if (mmvdMrgCost[mergeCand][mmvdCand] > mrgCostThres)
          {
            continue;
          }
          m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), geoMMVDTempBuf[mergeCand][mmvdCand].Y().buf, geoMMVDTempBuf[mergeCand][mmvdCand].Y().stride, sadMask, maskStride, stepX, maskStride2, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y);
          sadLarge = distParam.distFunc(distParam);
#if JVET_AG0164_AFFINE_GPM
          int isAffine = mergeCand < numRegularGpmMergeCand ? 0 : 1;
          double tempCost = (double)sadLarge + 
            (isAffine ? geoAffMergeIdxCost[mergeCand - numRegularGpmMergeCand] : geoMergeIdxCost[mergeCand]) + geoAffineFlagCost[isAffine] + geoMMVDFlagCost[1] + geoMMVDIdxCost[mmvdCand];
#else
          double tempCost = (double)sadLarge + geoMergeIdxCost[mergeCand] + geoMMVDFlagCost[1] + geoMMVDIdxCost[mmvdCand];
#endif
          m_geoMMVDCostList.insert(splitDir, 0, mergeCand, (mmvdCand + 1), tempCost);
          sortCandList(tempCost, mergeCand, (mmvdCand + 1), sadCostList0[splitDir], mergeCandList0[splitDir], mmvdCandList0[splitDir], m_numCandPerPar);

          sadSmall = sadMMVDWholeBlk[mergeCand][mmvdCand] - sadLarge;
#if JVET_AG0164_AFFINE_GPM
          tempCost = (double)sadSmall + 
            (isAffine ? geoAffMergeIdxCost[mergeCand - numRegularGpmMergeCand] : geoMergeIdxCost[mergeCand]) + geoAffineFlagCost[isAffine] + geoMMVDFlagCost[1] + geoMMVDIdxCost[mmvdCand];
#else
          tempCost = (double)sadSmall + geoMergeIdxCost[mergeCand] + geoMMVDFlagCost[1] + geoMMVDIdxCost[mmvdCand];
#endif
          m_geoMMVDCostList.insert(splitDir, 1, mergeCand, (mmvdCand + 1), tempCost);
          sortCandList(tempCost, mergeCand, (mmvdCand + 1), sadCostList1[splitDir], mergeCandList1[splitDir], mmvdCandList1[splitDir], m_numCandPerPar);
        }
      }
    }

    for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
    {
#if JVET_Y0065_GPM_INTRA
      int numCandMerge0 = min(m_numCandPerPar, (int)mergeCandList0[splitDir].size());
      int numCandIntra0 = (int)intraCandList0[splitDir].size();
      int numCandPart0 = numCandMerge0 + numCandIntra0;
      for (int candIdx0 = 0; candIdx0 < numCandPart0; candIdx0++)
      {
        int mergeCand0 = candIdx0 < numCandMerge0 ? mergeCandList0[splitDir][candIdx0] : intraCandList0[splitDir][candIdx0 - numCandMerge0];
        int mmvdCand0 = candIdx0 < numCandMerge0 ? mmvdCandList0[splitDir][candIdx0] : 0;
        int numCandMerge1 = min(m_numCandPerPar, (int)mergeCandList1[splitDir].size());
        int numCandIntra1 = candIdx0 < numCandMerge0 ? (int)intraCandList1[splitDir].size() : 0;
        int numCandPart1 = numCandMerge0 + numCandIntra1;
        int candStart1 = (bUseOnlyOneVector && candIdx0 < numCandMerge0) ? numCandMerge0 : 0;
        for (int candIdx1 = candStart1; candIdx1 < numCandPart1; candIdx1++)
        {
          int mergeCand1 = candIdx1 < numCandMerge1 ? mergeCandList1[splitDir][candIdx1] : intraCandList1[splitDir][candIdx1 - numCandMerge1];
          int mmvdCand1 = candIdx1 < numCandMerge1 ? mmvdCandList1[splitDir][candIdx1] : 0;
#if JVET_AG0164_AFFINE_GPM
          int isAffine0 = (mergeCand0 >= numRegularGpmMergeCand && mergeCand0 < GEO_MAX_ALL_INTER_UNI_CANDS) ? 1 : 0;
          int isAffine1 = (mergeCand1 >= numRegularGpmMergeCand && mergeCand1 < GEO_MAX_ALL_INTER_UNI_CANDS) ? 1 : 0;
          CHECK(mmvdCand0 && isAffine0, "Aff GPM MMVD is not allowed");
          CHECK(mmvdCand1 && isAffine1, "Aff GPM MMVD is not allowed");
          pu.affineGPM[0] = isAffine0;
          pu.affineGPM[1] = isAffine1;
#endif
#else
      int numCandPart0 = min(m_numCandPerPar, (int)mergeCandList0[splitDir].size());
      int numCandPart1 = min(m_numCandPerPar, (int)mergeCandList1[splitDir].size());
      for (int candIdx0 = 0; candIdx0 < numCandPart0; candIdx0++)
      {
        for (int candIdx1 = 0; candIdx1 < numCandPart1; candIdx1++)
        {
          int mergeCand0 = mergeCandList0[splitDir][candIdx0];
          int mergeCand1 = mergeCandList1[splitDir][candIdx1];
          int mmvdCand0 = mmvdCandList0[splitDir][candIdx0];
          int mmvdCand1 = mmvdCandList1[splitDir][candIdx1];
#endif
#if TM_MRG
          bool geoTmFlag0 = (mmvdCand0 == (GPM_EXT_MMVD_MAX_REFINE_NUM + 1));
          bool geoTmFlag1 = (mmvdCand1 == (GPM_EXT_MMVD_MAX_REFINE_NUM + 1));
          CHECK(geoTmFlag0 || geoTmFlag1, "GPM TM has not been tested by far");
#endif
          if ((mmvdCand0 == 0) && (mmvdCand1 == 0))
          {
            continue;
          }
          if ((mmvdCand0 == mmvdCand1) && (mmvdCand0 > 0))
          {
            if (mergeCand0 == mergeCand1)
            {
              continue;
            }
          }

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
          int geoSyntaxMode = std::numeric_limits<uint8_t>::max();
          if (sps.getUseAltGPMSplitModeCode())
          {
            m_pcInterSearch->setGeoSplitModeToSyntaxTable(pu, mergeCtxRegular, mergeCand0, mergeCtxRegular, mergeCand1
#if JVET_AG0164_AFFINE_GPM
              , affMergeCtx
#endif
#if JVET_Y0065_GPM_INTRA
                                                        , m_pcIntraSearch
#endif
                                                        , mmvdCand0 - 1, mmvdCand1 - 1);
            geoSyntaxMode = m_pcInterSearch->convertGeoSplitModeToSyntax(splitDir, mergeCand0, mergeCand1, mmvdCand0 - 1, mmvdCand1 - 1);
            if (geoSyntaxMode == std::numeric_limits<uint8_t>::max())
            {
              continue;
            }
          }
#endif

          double tempCost = m_geoMMVDCostList.singleDistList[0][splitDir][mergeCand0][mmvdCand0].cost + m_geoMMVDCostList.singleDistList[1][splitDir][mergeCand1][mmvdCand1].cost;
          tempCost = tempCost 
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
                   + (geoSyntaxMode == std::numeric_limits<uint8_t>::max() ? geoModeCost[splitDir] : geoSigModeCost[geoSyntaxMode]);
#else
                   + geoModeCost[splitDir];
#endif
#if TM_MRG
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
          if (sps.getUseGPMTMMode() 
#if JVET_AG0164_AFFINE_GPM
            && mergeCand0 < GEO_MAX_ALL_INTER_UNI_CANDS && mergeCand1 < GEO_MAX_ALL_INTER_UNI_CANDS
            && !isAffine0 && !isAffine1
#else
#if JVET_Y0065_GPM_INTRA
            && mergeCand0 < GEO_MAX_NUM_UNI_CANDS && mergeCand1 < GEO_MAX_NUM_UNI_CANDS
#endif
#endif
            )
#else
#if JVET_Y0065_GPM_INTRA
          if (sps.getUseDMVDMode() && mergeCand0 < GEO_MAX_NUM_UNI_CANDS && mergeCand1 < GEO_MAX_NUM_UNI_CANDS)
#else
          if (sps.getUseDMVDMode())
#endif
#endif
          {
            tempCost += geoTMFlagCost[0];
          }
#endif
          updateGeoMMVDCandList(tempCost, splitDir, mergeCand0, mergeCand1, mmvdCand0, mmvdCand1,
            geoSADCostList, geoSplitDirList, geoMergeCand0, geoMergeCand1, geoMmvdCand0, geoMmvdCand1, numSATDCands);
        }
      }
    }
#if JVET_AG0164_AFFINE_GPM
    pu.affineGPM[0] = 0;
    pu.affineGPM[1] = 0;
#endif
#if TM_MRG
#if JVET_AG0164_AFFINE_GPM
    uint8_t maxNumTmMrgCand = numRegularGpmMergeCand;
#else
    uint8_t maxNumTmMrgCand = maxNumMergeCandidates;
#endif
    PelUnitBuf geoTmBuffer[GEO_TM_MAX_NUM_CANDS];
    PelUnitBuf geoTmTempBuf[GEO_TM_MAX_NUM_CANDS];
#if JVET_AE0046_BI_GPM
    bool tmRefinePossible[GEO_TM_MAX_NUM_CANDS];
    for (int idx = 0; idx < GEO_TM_MAX_NUM_CANDS; idx++)
    {
      tmRefinePossible[idx] = false;
    }
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    if (sps.getUseGPMTMMode()
#if JVET_Y0065_GPM_INTRA
      && !bUseOnlyOneVector
#endif
      )
#else
#if JVET_Y0065_GPM_INTRA
    if (sps.getUseDMVDMode() && !bUseOnlyOneVector)
#else
    if (sps.getUseDMVDMode())
#endif
#endif
    {
      for (int i = GEO_TM_SHAPE_AL; i < GEO_NUM_TM_MV_CAND; i++)
      {
        mergeCtx[i].numValidMergeCand = maxNumTmMrgCand;
        for (int idx = 0; idx < maxNumTmMrgCand; idx++)
        {
          if (mrgDuplicated[idx])
          {
            continue;
          }
          mergeCtx[i].bcwIdx[idx] = BCW_DEFAULT;
          mergeCtx[i].useAltHpelIf[idx] = false;
#if JVET_AG0276_NLIC
          mergeCtx[i].altLMFlag[idx] = false;
          mergeCtx[i].altLMParaNeighbours[idx].resetAltLinearModel();
#endif
#if INTER_LIC
          mergeCtx[i].licFlags[idx] = false;
#endif
          mergeCtx[i].interDirNeighbours[idx] = mergeCtx[GEO_TM_OFF].interDirNeighbours[idx];
          mergeCtx[i].mvFieldNeighbours[(idx << 1)].mv = mergeCtx[GEO_TM_OFF].mvFieldNeighbours[(idx << 1)].mv;
          mergeCtx[i].mvFieldNeighbours[(idx << 1) + 1].mv = mergeCtx[GEO_TM_OFF].mvFieldNeighbours[(idx << 1) + 1].mv;
          mergeCtx[i].mvFieldNeighbours[(idx << 1)].refIdx = mergeCtx[GEO_TM_OFF].mvFieldNeighbours[(idx << 1)].refIdx;
          mergeCtx[i].mvFieldNeighbours[(idx << 1) + 1].refIdx = mergeCtx[GEO_TM_OFF].mvFieldNeighbours[(idx << 1) + 1].refIdx;
        }
      }

      pu.tmMergeFlag = true;
      Distortion sadTmWholeBlk[GEO_TM_MAX_NUM_CANDS];
      for (uint8_t tmType = GEO_TM_SHAPE_AL; tmType < GEO_NUM_TM_MV_CAND; tmType++)
      {
        pu.geoTmType = tmType;
        for (uint8_t mrgIdx = 0; mrgIdx < maxNumTmMrgCand; mrgIdx++)
        {
          if (mrgDuplicated[mrgIdx])
          {
            continue;
          }
          uint8_t mergeCand = mrgIdx + (tmType - 1) * GEO_MAX_NUM_UNI_CANDS;
          mergeCtx[tmType].setMergeInfo(pu, mrgIdx);
          m_pcInterSearch->deriveTMMv(pu);
#if JVET_AE0046_BI_GPM
          mergeCtx[tmType].interDirNeighbours[mrgIdx] = pu.interDir;
          mergeCtx[tmType].bcwIdx[mrgIdx] = (pu.interDir != 3) ? BCW_DEFAULT : mergeCtx[tmType].bcwIdx[mrgIdx];
          mergeCtx[tmType].mvFieldNeighbours[(mrgIdx << 1)].refIdx = pu.refIdx[0];
          mergeCtx[tmType].mvFieldNeighbours[(mrgIdx << 1) + 1].refIdx = pu.refIdx[1];
#endif
          mergeCtx[tmType].mvFieldNeighbours[(mrgIdx << 1)].mv = pu.mv[0];
          mergeCtx[tmType].mvFieldNeighbours[(mrgIdx << 1) + 1].mv = pu.mv[1];

          geoTmBuffer[mergeCand] = m_acGeoMergeTmpBuffer[mergeCand].getBuf(localUnitArea);
#if JVET_AE0046_BI_GPM
          if (PU::checkBDMVRCondition(pu, true))
          {
            tmRefinePossible[mergeCand] = true;
            pu.bdmvrRefine = true;
            PU::spanPuMv2DmvrBuffer(pu, m_mvBufBDMVR4GPM[0], m_mvBufBDMVR4GPM[1]);
          }
#endif
          m_pcInterSearch->motionCompensation(pu, geoTmBuffer[mergeCand], REF_PIC_LIST_X, true, false);

#if JVET_AE0046_BI_GPM
          if (pu.bdmvrRefine)
          {
            pu.bdmvrRefine = false;
            ::memcpy(m_mvBufEncBDOF4GPM[tmType][mrgIdx], m_pcInterSearch->getBdofSubPuMvOffset(), sizeof(Mv) * BDOF_SUBPU_MAX_NUM);
          }
#endif

          // calculate SAD for each candidate
          geoTmTempBuf[mergeCand] = m_acGeoSADTmpBuffer[mergeCand].getBuf(localUnitArea);
          geoTmTempBuf[mergeCand].Y().copyFrom(geoTmBuffer[mergeCand].Y());
          geoTmTempBuf[mergeCand].Y().roundToOutputBitdepth(geoTmTempBuf[mergeCand].Y(), cu.slice->clpRng(COMPONENT_Y));
          distParamWholeBlk.cur.buf = geoTmTempBuf[mergeCand].Y().buf;
          distParamWholeBlk.cur.stride = geoTmTempBuf[mergeCand].Y().stride;
          sadTmWholeBlk[mergeCand] = distParamWholeBlk.distFunc(distParamWholeBlk);
        }
      }
      pu.tmMergeFlag = false;

      for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
      {
        int16_t angle = g_geoParams[splitDir][0];
        if (g_angle2mirror[angle] == 2)
        {
          stepX = 1;
          maskStride = -GEO_WEIGHT_MASK_SIZE;
          maskStride2 = -(int)cu.lwidth();
          sadMask = &g_geoEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
#if GPM_CURVE
          if(g_geoParams[splitDir][1] == 0 && wIdx == hIdx && wIdx >= GEO_CURVE_MIN_CU_IDX && CU::isGeoCURVEAvailable(splitDir,cu.lwidth()))
          {
            sadMask = &g_geoCurveEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
          }
#endif
        }
        else if (g_angle2mirror[angle] == 1)
        {
          stepX = -1;
          maskStride2 = cu.lwidth();
          maskStride = GEO_WEIGHT_MASK_SIZE;
          sadMask = &g_geoEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
#if GPM_CURVE
          if(g_geoParams[splitDir][1] == 0 && wIdx == hIdx && wIdx >= GEO_CURVE_MIN_CU_IDX && CU::isGeoCURVEAvailable(splitDir,cu.lwidth()))
          {
            sadMask = &g_geoCurveEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
          }
#endif
        }
        else
        {
          stepX = 1;
          maskStride = GEO_WEIGHT_MASK_SIZE;
          maskStride2 = -(int)cu.lwidth();
          sadMask = &g_geoEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
#if GPM_CURVE
          if(g_geoParams[splitDir][1] == 0 && wIdx == hIdx && wIdx >= GEO_CURVE_MIN_CU_IDX && CU::isGeoCURVEAvailable(splitDir,cu.lwidth()))
          {
            sadMask = &g_geoCurveEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
          }
#endif
        }
        for (uint8_t mergeCand = 0; mergeCand < maxNumTmMrgCand; mergeCand++)
        {
          if (mrgDuplicated[mergeCand])
          {
            continue;
          }

          uint8_t mergeCand0 = mergeCand + (g_geoTmShape[0][g_geoParams[splitDir][0]] - 1) * GEO_MAX_NUM_UNI_CANDS;
          m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), geoTmTempBuf[mergeCand0].Y().buf, geoTmTempBuf[mergeCand0].Y().stride, sadMask, maskStride, stepX, maskStride2, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y);
          sadLarge = distParam.distFunc(distParam);
          double tempCost = (double)sadLarge + geoMergeIdxCost[mergeCand] + geoMMVDFlagCost[0];
          m_geoMMVDCostList.insert(splitDir, 0, mergeCand, (GPM_EXT_MMVD_MAX_REFINE_NUM + 1), tempCost);

          uint8_t mergeCand1 = mergeCand + (g_geoTmShape[1][g_geoParams[splitDir][0]] - 1) * GEO_MAX_NUM_UNI_CANDS;
          m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), geoTmTempBuf[mergeCand1].Y().buf, geoTmTempBuf[mergeCand1].Y().stride, sadMask, maskStride, stepX, maskStride2, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y);
          sadSmall = sadTmWholeBlk[mergeCand1] - distParam.distFunc(distParam);
          tempCost = (double)sadSmall + geoMergeIdxCost[mergeCand] + geoMMVDFlagCost[0];
          m_geoMMVDCostList.insert(splitDir, 1, mergeCand, (GPM_EXT_MMVD_MAX_REFINE_NUM + 1), tempCost);
        }
      }

      for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
      {
        for (int mergeCand0 = 0; mergeCand0 < maxNumTmMrgCand; mergeCand0++)
        {
          if (mrgDuplicated[mergeCand0])
          {
            continue;
          }
          for (int mergeCand1 = 0; mergeCand1 < maxNumTmMrgCand; mergeCand1++)
          {
            if (mrgDuplicated[mergeCand1])
            {
              continue;
            }
            if (mergeCand0 == mergeCand1)
            {
              continue;
            }

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
            int geoSyntaxMode = std::numeric_limits<uint8_t>::max();
            if(sps.getUseAltGPMSplitModeCode())
            {
              m_pcInterSearch->setGeoTMSplitModeToSyntaxTable(pu, mergeCtx, mergeCand0, mergeCand1, tmMmvdBufIdx0 - 1, tmMmvdBufIdx1 - 1);
              geoSyntaxMode = m_pcInterSearch->convertGeoSplitModeToSyntax(splitDir, mergeCand0, mergeCand1, tmMmvdBufIdx0 - 1, tmMmvdBufIdx1 - 1);
              if (geoSyntaxMode == std::numeric_limits<uint8_t>::max())
              {
                continue;
              }
            }
#endif

            double tempCost = m_geoMMVDCostList.singleDistList[0][splitDir][mergeCand0][GPM_EXT_MMVD_MAX_REFINE_NUM + 1].cost + m_geoMMVDCostList.singleDistList[1][splitDir][mergeCand1][GPM_EXT_MMVD_MAX_REFINE_NUM + 1].cost;
            tempCost = tempCost 
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
                     + (geoSyntaxMode == std::numeric_limits<uint8_t>::max() ? geoModeCost[splitDir] : geoSigModeCost[geoSyntaxMode])
#else
                     + geoModeCost[splitDir]
#endif
                     + geoTMFlagCost[1];
            updateGeoMMVDCandList(tempCost, splitDir, mergeCand0, mergeCand1, (GPM_EXT_MMVD_MAX_REFINE_NUM + 1), (GPM_EXT_MMVD_MAX_REFINE_NUM + 1),
              geoSADCostList, geoSplitDirList, geoMergeCand0, geoMergeCand1, geoMmvdCand0, geoMmvdCand1, numSATDCands);
          }
        }
      }
    }
#endif

    int numberGeoCandChecked = (int)geoSADCostList.size();
    if (numberGeoCandChecked == 0)
    {
      return;
    }

    geoNumMrgSATDCand = min(GEO_MAX_TRY_WEIGHTED_SATD, numberGeoCandChecked);
    if (geoRdModeList.size() > geoNumMrgSATDCand)
    {
      geoRdModeList.resize(geoNumMrgSATDCand);
      isNonMMVDListIdx.resize(geoNumMrgSATDCand);
      geoPartitionModeList.resize(geoNumMrgSATDCand);
      geocandCostList.resize(geoNumMrgSATDCand);
    }
    for (uint8_t candidateIdx = 0; candidateIdx < numberGeoCandChecked; candidateIdx++)
    {
      int splitDir = geoSplitDirList[candidateIdx];
      int mergeCand0 = geoMergeCand0[candidateIdx];
      int mergeCand1 = geoMergeCand1[candidateIdx];
#if TM_MRG
      bool tmFlag0 = (geoMmvdCand0[candidateIdx] == (GPM_EXT_MMVD_MAX_REFINE_NUM + 1));
      bool tmFlag1 = (geoMmvdCand1[candidateIdx] == (GPM_EXT_MMVD_MAX_REFINE_NUM + 1));
      bool mmvdFlag0 = (geoMmvdCand0[candidateIdx] >= 1 && geoMmvdCand0[candidateIdx] <= GPM_EXT_MMVD_MAX_REFINE_NUM);
      bool mmvdFlag1 = (geoMmvdCand1[candidateIdx] >= 1 && geoMmvdCand1[candidateIdx] <= GPM_EXT_MMVD_MAX_REFINE_NUM);
      int  mmvdCand0 = (mmvdFlag0 ? (geoMmvdCand0[candidateIdx] - 1) : MAX_INT);
      int  mmvdCand1 = (mmvdFlag1 ? (geoMmvdCand1[candidateIdx] - 1) : MAX_INT);
      CHECK(tmFlag0 != tmFlag1, "TM flag cannot be enabled/disabled for two partitions separately");

      if (!tmFlag0 && !tmFlag1 && !mmvdFlag0 && !mmvdFlag1)
      {
        continue;
      }
      if (tmFlag0 && mergeCand0 == mergeCand1)
      {
        continue;
      }
#else
      int mmvdCand0 = geoMmvdCand0[candidateIdx] - 1;
      int mmvdCand1 = geoMmvdCand1[candidateIdx] - 1;
      bool mmvdFlag0 = (mmvdCand0 >= 0);
      bool mmvdFlag1 = (mmvdCand1 >= 0);

      if (!mmvdFlag0 && !mmvdFlag1)
      {
        continue;
      }
#endif
#if JVET_AG0164_AFFINE_GPM                          
      int isAffine0 = (mergeCand0 >= numRegularGpmMergeCand && mergeCand0 < GEO_MAX_ALL_INTER_UNI_CANDS && !tmFlag0) ? 1 : 0;
      int isAffine1 = (mergeCand1 >= numRegularGpmMergeCand && mergeCand1 < GEO_MAX_ALL_INTER_UNI_CANDS && !tmFlag1) ? 1 : 0;
      pu.affineGPM[0] = isAffine0;
      pu.affineGPM[1] = isAffine1;
      CHECK(mmvdFlag0&& isAffine0, "Aff GPM MMVD is not allowed");
      CHECK(mmvdFlag1&& isAffine1, "Aff GPM MMVD is not allowed");
#endif

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
      int geoSyntaxMode = std::numeric_limits<uint8_t>::max();
      if (sps.getUseAltGPMSplitModeCode()
#if JVET_Y0065_GPM_INTRA
#if JVET_AG0164_AFFINE_GPM
        && mergeCand0 < GEO_MAX_ALL_INTER_UNI_CANDS&& mergeCand1 < GEO_MAX_ALL_INTER_UNI_CANDS
#else
        && mergeCand0 < GEO_MAX_NUM_UNI_CANDS && mergeCand1 < GEO_MAX_NUM_UNI_CANDS
#endif
#endif
        )
      {
#if JVET_W0097_GPM_MMVD_TM && TM_MRG
        if (tmFlag0 && tmFlag1)
        {
          geoSyntaxMode = m_pcInterSearch->convertGeoSplitModeToSyntax(splitDir, mergeCand0, mergeCand1, tmMmvdBufIdx0 - 1, tmMmvdBufIdx1 - 1);
          CHECK(geoSyntaxMode < 0 || geoSyntaxMode >= GEO_NUM_SIG_PARTMODE, "Invalid GEO split direction!");
          CHECK(tmFlag0 != tmFlag1, "tmFlag0 and tmFlag1 have to be identical to each other");
        }
        else
#endif
        {
          int  mmvdCandTmp0 = geoMmvdCand0[candidateIdx];
          int  mmvdCandTmp1 = geoMmvdCand1[candidateIdx];
          geoSyntaxMode = m_pcInterSearch->convertGeoSplitModeToSyntax(splitDir, mergeCand0, mergeCand1, mmvdCandTmp0 - 1, mmvdCandTmp1 - 1);
          CHECK(geoSyntaxMode < 0 || geoSyntaxMode >= GEO_NUM_SIG_PARTMODE, "Invalid GEO split direction!");
          CHECK(!mmvdFlag0 && !mmvdFlag1, "GPM MMVD has to be used at least for one partition");
        }
      }
#endif

#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
      for (uint8_t bldIdx = 0; bldIdx < GEO_BLENDING_NUM; bldIdx++)
      {
        geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx] = m_acGeoWeightedBuffer[candidateIdx * GEO_BLENDING_NUM + bldIdx].getBuf(localUnitArea);

#if JVET_Y0065_GPM_INTRA
#if JVET_AG0164_AFFINE_GPM
        int isIntra0 = (mergeCand0 >= GEO_MAX_ALL_INTER_UNI_CANDS) ? 1 : 0;
        int isIntra1 = (mergeCand1 >= GEO_MAX_ALL_INTER_UNI_CANDS) ? 1 : 0;
#else
        int isIntra0 = (mergeCand0 >= GEO_MAX_NUM_UNI_CANDS) ? 1 : 0;
        int isIntra1 = (mergeCand1 >= GEO_MAX_NUM_UNI_CANDS) ? 1 : 0;
#endif
        int candidateSAD = candidateIdx * GEO_BLENDING_NUM + bldIdx;
#endif
#if TM_MRG || JVET_Y0065_GPM_INTRA
        PelUnitBuf predSrc0, predSrc1;
#if JVET_Y0065_GPM_INTRA
#if JVET_AG0164_AFFINE_GPM
        int intraIdx0 = mergeCand0 - GEO_MAX_ALL_INTER_UNI_CANDS;
        int intraIdx1 = mergeCand1 - GEO_MAX_ALL_INTER_UNI_CANDS;
#else
        int intraIdx0 = mergeCand0 - GEO_MAX_NUM_UNI_CANDS;
        int intraIdx1 = mergeCand1 - GEO_MAX_NUM_UNI_CANDS;
#endif
        if (isIntra0 || isIntra1)
        {
          if (isIntra0)
          {
            int rdoBuffer = intraRDOBufIdx[geoIntraMPMList[splitDir][0][intraIdx0]];
            predSrc0 = geoIntraBuffer[rdoBuffer];
          }
#if TM_MRG
          else if (tmFlag0)
          {
            int mrgTmCand0 = mergeCand0 + (g_geoTmShape[0][g_geoParams[splitDir][0]] - 1) * GEO_MAX_NUM_UNI_CANDS;
            predSrc0 = geoTmTempBuf[mrgTmCand0];
          }
#endif
          else // mmvdFlag0
          {
            predSrc0 = geoMMVDTempBuf[mergeCand0][mmvdCand0];
          }

          if (isIntra1)
          {
            int rdoBuffer = intraRDOBufIdx[geoIntraMPMList[splitDir][1][intraIdx1]];
            predSrc1 = geoIntraBuffer[rdoBuffer];
          }
#if TM_MRG
          else if (tmFlag1)
          {
            int mrgTmCand1 = mergeCand1 + (g_geoTmShape[1][g_geoParams[splitDir][0]] - 1) * GEO_MAX_NUM_UNI_CANDS;
            predSrc1 = geoTmTempBuf[mrgTmCand1];
          }
#endif
          else // mmvdFlag1
          {
            predSrc1 = geoMMVDTempBuf[mergeCand1][mmvdCand1];
          }
          if (pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
          {
            if (!isIntra0) // Inter+Intra
            {
              geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx].Y().rspSignal(predSrc0.Y(), m_pcReshape->getFwdLUT());
              m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, bldIdx, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx], geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx], predSrc1);
            }
            else if (!isIntra1) // Intra+Inter
            {
              geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx].Y().rspSignal(predSrc1.Y(), m_pcReshape->getFwdLUT());
              m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, bldIdx, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx], predSrc0, geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx]);
            }
            candidateSAD = GEO_MAX_TRY_WEIGHTED_SAD * GEO_BLENDING_NUM;
            geoCombinations[candidateSAD] = m_acGeoWeightedBuffer[candidateSAD].getBuf(localUnitArea);
            geoCombinations[candidateSAD].Y().rspSignal(geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx].Y(), m_pcReshape->getInvLUT());
          }
          else
          {
            m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, bldIdx, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx], predSrc0, predSrc1);
          }
        }
        else
        {
#endif
#if TM_MRG 
          if (tmFlag0)
          {
            int mrgTmCand0 = mergeCand0 + (g_geoTmShape[0][g_geoParams[splitDir][0]] - 1) * GEO_MAX_NUM_UNI_CANDS;
            predSrc0 = geoTmBuffer[mrgTmCand0];
          }
          else if (mmvdFlag0)
          {
            predSrc0 = geoMMVDBuf[mergeCand0][mmvdCand0];
          }
          else
          {
            predSrc0 = geoBuffer[mergeCand0];
          }

          if (tmFlag1)
          {
            int mrgTmCand1 = mergeCand1 + (g_geoTmShape[1][g_geoParams[splitDir][0]] - 1) * GEO_MAX_NUM_UNI_CANDS;
            predSrc1 = geoTmBuffer[mrgTmCand1];
          }
          else if (mmvdFlag1)
          {
            predSrc1 = geoMMVDBuf[mergeCand1][mmvdCand1];
          }
          else
          {
            predSrc1 = geoBuffer[mergeCand1];
          }

          m_pcInterSearch->weightedGeoBlk(pu, splitDir, bldIdx, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx], predSrc0, predSrc1);
#else
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
          m_pcInterSearch->weightedGeoBlk(pu, splitDir, bldIdx, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx], (mmvdFlag0 ? geoMMVDBuf[mergeCand0][mmvdCand0] : geoBuffer[mergeCand0]), (mmvdFlag1 ? geoMMVDBuf[mergeCand1][mmvdCand1] : geoBuffer[mergeCand1]));
#else
          m_pcInterSearch->weightedGeoBlk(pu, splitDir, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx], (mmvdFlag0 ? geoMMVDBuf[mergeCand0][mmvdCand0] : geoBuffer[mergeCand0]), (mmvdFlag1 ? geoMMVDBuf[mergeCand1][mmvdCand1] : geoBuffer[mergeCand1]));
#endif
#endif
#if JVET_Y0065_GPM_INTRA
        }
#endif
#else
        m_pcInterSearch->weightedGeoBlk(pu, splitDir, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx], (mmvdFlag0 ? geoMMVDBuf[mergeCand0][mmvdCand0] : geoBuffer[mergeCand0]), (mmvdFlag1 ? geoMMVDBuf[mergeCand1][mmvdCand1] : geoBuffer[mergeCand1]));
#endif
#if JVET_Y0065_GPM_INTRA
        distParamSAD2.cur = geoCombinations[candidateSAD].Y();
#else
        distParamSAD2.cur = geoCombinations[candidateIdx].Y();
#endif
        Distortion sad = distParamSAD2.distFunc(distParamSAD2);

#if JVET_Y0065_GPM_INTRA
#if JVET_AG0164_AFFINE_GPM
        double updateCost =
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
        ((geoSyntaxMode == std::numeric_limits<uint8_t>::max())? geoModeCost[splitDir] : geoSigModeCost[geoSyntaxMode])
#else
          geoModeCost[splitDir]
#endif
          + geoMMVDFlagCost[mmvdFlag0] + geoMMVDFlagCost[mmvdFlag1] + geoAffineFlagCost[isAffine0] + geoAffineFlagCost[isAffine1];
        if (mmvdFlag0)
        {
          updateCost += geoMMVDIdxCost[mmvdCand0];
        }
        if (!mmvdFlag0 && !isAffine0)
        {
          updateCost += geoIntraFlag0Cost[isIntra0];
        }
        updateCost += (isIntra0 ? geoIntraIdxCost[intraIdx0] : (isAffine0 ? geoAffMergeIdxCost[mergeCand0 - numRegularGpmMergeCand] : geoMergeIdxCost[mergeCand0]));
        if (!bUseOnlyOneVector || isIntra0)
        {
          if (mmvdFlag1)
          {
            updateCost += geoMMVDIdxCost[mmvdCand1];
          }
          if (!mmvdFlag1 && !isAffine1)
          {
            updateCost += geoIntraFlag1Cost[isIntra0][isIntra1];
          }
          updateCost += (isIntra1 ? geoIntraIdxCost[intraIdx1] : (isAffine1 ? geoAffMergeIdxCost[mergeCand1 - numRegularGpmMergeCand] : geoMergeIdxCost[mergeCand1]));
        }
#else
        double updateCost =
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
        (geoSyntaxMode == std::numeric_limits<uint8_t>::max() ? geoModeCost[splitDir] : geoSigModeCost[geoSyntaxMode])
#else
          geoModeCost[splitDir]
#endif
          + geoMMVDFlagCost[mmvdFlag0] + geoMMVDFlagCost[mmvdFlag1];
        updateCost += (mmvdFlag0 ? geoMMVDIdxCost[mmvdCand0] : geoIntraFlag0Cost[isIntra0]);
        updateCost += (isIntra0 ? geoIntraIdxCost[intraIdx0] : geoMergeIdxCost[mergeCand0]);
        updateCost += (mmvdFlag1 ? geoMMVDIdxCost[mmvdCand1] : geoIntraFlag1Cost[isIntra0][isIntra1]);
        updateCost += (isIntra1 ? geoIntraIdxCost[intraIdx1] : geoMergeIdxCost[mergeCand1]);
#endif
#else
        double updateCost =
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
        (geoSyntaxMode == std::numeric_limits<uint8_t>::max(); ? geoModeCost[splitDir] : geoSigModeCost[geoSyntaxMode])
#else
          geoModeCost[splitDir]
#endif
          + geoMergeIdxCost[mergeCand0] + geoMergeIdxCost[mergeCand1] + geoMMVDFlagCost[mmvdFlag0] + geoMMVDFlagCost[mmvdFlag1];
        if (mmvdFlag0)
        {
          updateCost += geoMMVDIdxCost[mmvdCand0];
        }
        if (mmvdFlag1)
        {
          updateCost += geoMMVDIdxCost[mmvdCand1];
        }
#endif
#if TM_MRG
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
        if (sps.getUseGPMTMMode())
#else
        if (sps.getUseDMVDMode())
#endif
        {
#if JVET_Y0065_GPM_INTRA
          if (!mmvdFlag0 && !mmvdFlag1 && !isIntra0 && !isIntra1
#if JVET_AG0164_AFFINE_GPM
            && !isAffine0 && !isAffine1
#endif
            )
#else
          if (!mmvdFlag0 && !mmvdFlag1)
#endif
          {
            updateCost += geoTMFlagCost[tmFlag0];
          }
        }
#endif

        updateCost += geoBldFlagCost[bldIdx];
        updateCost += (double)sad;
        orderCandList(candidateIdx, false, splitDir, updateCost, bldIdx, geoRdModeList, isNonMMVDListIdx, geoPartitionModeList, geocandCostList, geoBldList, geoNumMrgSATDCand);
      }
#else
      geoCombinations[candidateIdx] = m_acGeoWeightedBuffer[candidateIdx].getBuf(localUnitArea);

#if JVET_Y0065_GPM_INTRA
      int isIntra0 = (mergeCand0 >= GEO_MAX_NUM_UNI_CANDS) ? 1 : 0;
      int isIntra1 = (mergeCand1 >= GEO_MAX_NUM_UNI_CANDS) ? 1 : 0;
      uint8_t candidateSAD = candidateIdx;
#endif
#if TM_MRG || JVET_Y0065_GPM_INTRA
      PelUnitBuf predSrc0, predSrc1;
#if JVET_Y0065_GPM_INTRA
      int intraIdx0 = mergeCand0 - GEO_MAX_NUM_UNI_CANDS;
      int intraIdx1 = mergeCand1 - GEO_MAX_NUM_UNI_CANDS;
      if (isIntra0 || isIntra1)
      {
        if (isIntra0)
        {
          int rdoBuffer = intraRDOBufIdx[geoIntraMPMList[splitDir][0][intraIdx0]];
          predSrc0 = geoIntraBuffer[rdoBuffer];
        }
#if TM_MRG
        else if (tmFlag0)
        {
          int mrgTmCand0 = mergeCand0 + (g_geoTmShape[0][g_geoParams[splitDir][0]] - 1) * GEO_MAX_NUM_UNI_CANDS;
          predSrc0 = geoTmTempBuf[mrgTmCand0];
        }
#endif
        else // mmvdFlag0
        {
          predSrc0 = geoMMVDTempBuf[mergeCand0][mmvdCand0];
        }

        if (isIntra1)
        {
          int rdoBuffer = intraRDOBufIdx[geoIntraMPMList[splitDir][1][intraIdx1]];
          predSrc1 = geoIntraBuffer[rdoBuffer];
        }
#if TM_MRG
        else if (tmFlag1)
        {
          int mrgTmCand1 = mergeCand1 + (g_geoTmShape[1][g_geoParams[splitDir][0]] - 1) * GEO_MAX_NUM_UNI_CANDS;
          predSrc1 = geoTmTempBuf[mrgTmCand1];
        }
#endif
        else // mmvdFlag1
        {
          predSrc1 = geoMMVDTempBuf[mergeCand1][mmvdCand1];
        }
        if (pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
        {
          if (!isIntra0) // Inter+Intra
          {
            geoCombinations[candidateIdx].Y().rspSignal(predSrc0.Y(), m_pcReshape->getFwdLUT());
            m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx], geoCombinations[candidateIdx], predSrc1);
          }
          else if (!isIntra1) // Intra+Inter
          {
            geoCombinations[candidateIdx].Y().rspSignal(predSrc1.Y(), m_pcReshape->getFwdLUT());
            m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx], predSrc0, geoCombinations[candidateIdx]);
          }
          candidateSAD = GEO_MAX_TRY_WEIGHTED_SAD;
          geoCombinations[GEO_MAX_TRY_WEIGHTED_SAD] = m_acGeoWeightedBuffer[GEO_MAX_TRY_WEIGHTED_SAD].getBuf(localUnitArea);
          geoCombinations[GEO_MAX_TRY_WEIGHTED_SAD].Y().rspSignal(geoCombinations[candidateIdx].Y(), m_pcReshape->getInvLUT());
        }
        else
        {
          m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx], predSrc0, predSrc1);
        }
      }
      else
      {
#endif
#if TM_MRG 
      if (tmFlag0)
      {
        int mrgTmCand0 = mergeCand0 + (g_geoTmShape[0][g_geoParams[splitDir][0]] - 1) * GEO_MAX_NUM_UNI_CANDS;
        predSrc0 = geoTmBuffer[mrgTmCand0];
      }
      else if (mmvdFlag0)
      {
        predSrc0 = geoMMVDBuf[mergeCand0][mmvdCand0];
      }
      else
      {
        predSrc0 = geoBuffer[mergeCand0];
      }

      if (tmFlag1)
      {
        int mrgTmCand1 = mergeCand1 + (g_geoTmShape[1][g_geoParams[splitDir][0]] - 1) * GEO_MAX_NUM_UNI_CANDS;
        predSrc1 = geoTmBuffer[mrgTmCand1];
      }
      else if (mmvdFlag1)
      {
        predSrc1 = geoMMVDBuf[mergeCand1][mmvdCand1];
      }
      else
      {
        predSrc1 = geoBuffer[mergeCand1];
      }

      m_pcInterSearch->weightedGeoBlk(pu, splitDir, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx], predSrc0, predSrc1);
#else
      m_pcInterSearch->weightedGeoBlk( pu, splitDir, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx], (mmvdFlag0 ? geoMMVDBuf[mergeCand0][mmvdCand0] : geoBuffer[mergeCand0]), (mmvdFlag1 ? geoMMVDBuf[mergeCand1][mmvdCand1] : geoBuffer[mergeCand1]) );
#endif
#if JVET_Y0065_GPM_INTRA
      }
#endif
#else
      m_pcInterSearch->weightedGeoBlk(pu, splitDir, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx], (mmvdFlag0 ? geoMMVDBuf[mergeCand0][mmvdCand0] : geoBuffer[mergeCand0]), (mmvdFlag1 ? geoMMVDBuf[mergeCand1][mmvdCand1] : geoBuffer[mergeCand1]));
#endif
#if JVET_Y0065_GPM_INTRA
      distParamSAD2.cur = geoCombinations[candidateSAD].Y();
#else
      distParamSAD2.cur = geoCombinations[candidateIdx].Y();
#endif
      Distortion sad = distParamSAD2.distFunc(distParamSAD2);

#if JVET_Y0065_GPM_INTRA
      double updateCost = 
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
                          ( geoSyntaxMode == std::numeric_limits<uint8_t>::max() ? geoModeCost[splitDir] : geoSigModeCost[geoSyntaxMode])
#else
                          geoModeCost[splitDir] 
#endif
                        + geoMMVDFlagCost[mmvdFlag0] + geoMMVDFlagCost[mmvdFlag1];
      updateCost += (mmvdFlag0 ? geoMMVDIdxCost[mmvdCand0] : geoIntraFlag0Cost[isIntra0]);
      updateCost += (isIntra0 ? geoIntraIdxCost[intraIdx0] : geoMergeIdxCost[mergeCand0]);
      updateCost += (mmvdFlag1 ? geoMMVDIdxCost[mmvdCand1] : geoIntraFlag1Cost[isIntra0][isIntra1]);
      updateCost += (isIntra1 ? geoIntraIdxCost[intraIdx1] : geoMergeIdxCost[mergeCand1]);
#else
      double updateCost = 
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
                          ( geoSyntaxMode == std::numeric_limits<uint8_t>::max(); ? geoModeCost[splitDir] : geoSigModeCost[geoSyntaxMode])
#else
                          geoModeCost[splitDir] 
#endif
                        + geoMergeIdxCost[mergeCand0] + geoMergeIdxCost[mergeCand1] + geoMMVDFlagCost[mmvdFlag0] + geoMMVDFlagCost[mmvdFlag1];
      if (mmvdFlag0)
      {
        updateCost += geoMMVDIdxCost[mmvdCand0];
      }
      if (mmvdFlag1)
      {
        updateCost += geoMMVDIdxCost[mmvdCand1];
      }
#endif
#if TM_MRG
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
      if (sps.getUseGPMTMMode())
#else
      if (sps.getUseDMVDMode())
#endif
      {
#if JVET_Y0065_GPM_INTRA
        if (!mmvdFlag0 && !mmvdFlag1 && !isIntra0 && !isIntra1)
#else
        if (!mmvdFlag0 && !mmvdFlag1)
#endif
        {
          updateCost += geoTMFlagCost[tmFlag0];
        }
      }
#endif
      updateCost += (double)sad;
      orderCandList(candidateIdx, false, splitDir, updateCost, geoRdModeList, isNonMMVDListIdx, geoPartitionModeList, geocandCostList, geoNumMrgSATDCand);
#endif
    }
#if JVET_AG0164_AFFINE_GPM
    pu.affineGPM[0] = 0;
    pu.affineGPM[1] = 0;
#endif
    if (m_fastGpmMmvdRelatedCU)
    {
      int cnt = 0;
      for (uint8_t i = 0; i < geoNumMrgSATDCand; i++)
      {
        if (isNonMMVDListIdx[i] == false)
        {
          relatedCU.geoDirCandList[cnt] = geoPartitionModeList[i];
          relatedCU.geoMrgIdx0List[cnt] = geoMergeCand0[geoRdModeList[i]];
          relatedCU.geoMrgIdx1List[cnt] = geoMergeCand1[geoRdModeList[i]];
          cnt++;
        }
      }
      relatedCU.numGeoDirCand = cnt;
    }
    else
    {
      relatedCU.numGeoDirCand = geoNumMrgSATDCand;
      for (uint8_t i = 0; i < geoNumMrgSATDCand; i++)
      {
        relatedCU.geoDirCandList[i] = geoPartitionModeList[i];
      }
    }

    for (uint8_t i = 1; i < geoNumMrgSATDCand; i++)
    {
#if MERGE_ENC_OPT
      if (geocandCostList[i] > MRG_FAST_RATIO * geocandCostList[0] || geocandCostList[i] > getMergeBestSATDCost())
#else
      if (geocandCostList[i] > MRG_FAST_RATIO * geocandCostList[0] || geocandCostList[i] > getMergeBestSATDCost() || geocandCostList[i] > getAFFBestSATDCost())
#endif
      {
        geoNumMrgSATDCand = i;
        break;
      }
    }


#if JVET_AG0164_AFFINE_GPM
    CHECK(pu.cu->affine, "Should not be affine");
#endif

#if JVET_Y0065_GPM_INTRA
    for (uint8_t i = 0; i < geoNumMrgSATDCand; i++)
    {
      if (isNonMMVDListIdx[i])
      {
        continue;
      }
      uint8_t candidateIdx = geoRdModeList[i];
      int splitDir = geoSplitDirList[candidateIdx];
      int mergeCand0 = geoMergeCand0[candidateIdx];
      int mergeCand1 = geoMergeCand1[candidateIdx];
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
      uint8_t bldIdx = geoBldList[i];
#endif
#if TM_MRG
      bool tmFlag0 = (geoMmvdCand0[candidateIdx] == (GPM_EXT_MMVD_MAX_REFINE_NUM + 1));
      bool tmFlag1 = (geoMmvdCand1[candidateIdx] == (GPM_EXT_MMVD_MAX_REFINE_NUM + 1));
      bool mmvdFlag0 = (geoMmvdCand0[candidateIdx] >= 1 && geoMmvdCand0[candidateIdx] <= GPM_EXT_MMVD_MAX_REFINE_NUM);
      bool mmvdFlag1 = (geoMmvdCand1[candidateIdx] >= 1 && geoMmvdCand1[candidateIdx] <= GPM_EXT_MMVD_MAX_REFINE_NUM);
      int  mmvdCand0 = (mmvdFlag0 ? (geoMmvdCand0[candidateIdx] - 1) : MAX_INT);
      int  mmvdCand1 = (mmvdFlag1 ? (geoMmvdCand1[candidateIdx] - 1) : MAX_INT);
      int mrgTmCand0 = MAX_INT, mrgTmCand1 = MAX_INT;
#else
      int mmvdCand0 = geoMmvdCand0[candidateIdx] - 1;
      int mmvdCand1 = geoMmvdCand1[candidateIdx] - 1;

      bool mmvdFlag0 = (mmvdCand0 >= 0);
      bool mmvdFlag1 = (mmvdCand1 >= 0);
#endif

#if JVET_AG0164_AFFINE_GPM
      int isIntra0 = (mergeCand0 >= GEO_MAX_ALL_INTER_UNI_CANDS) ? 1 : 0;
      int isIntra1 = (mergeCand1 >= GEO_MAX_ALL_INTER_UNI_CANDS) ? 1 : 0;
      int isAffine0 = (mergeCand0 >= numRegularGpmMergeCand && mergeCand0 < GEO_MAX_ALL_INTER_UNI_CANDS && !tmFlag0) ? 1 : 0;
      int isAffine1 = (mergeCand1 >= numRegularGpmMergeCand && mergeCand1 < GEO_MAX_ALL_INTER_UNI_CANDS && !tmFlag1) ? 1 : 0;

      pu.affineGPM[0] = isAffine0;
      pu.affineGPM[1] = isAffine1;
      CHECK(isAffine0 && tmFlag0, "GPM affine cannot be used together with GPM TM");
      CHECK(isAffine1 && tmFlag1, "GPM affine cannot be used together with GPM TM");
#else
      int isIntra0 = (mergeCand0 >= GEO_MAX_NUM_UNI_CANDS) ? 1 : 0;
      int isIntra1 = (mergeCand1 >= GEO_MAX_NUM_UNI_CANDS) ? 1 : 0;
#endif

      if (!isChromaEnabled(pu.chromaFormat) && !isIntra0 && !isIntra1)
      {
        continue;
      }
      int mrgIntraCand0 = MAX_INT, mrgIntraCand1 = MAX_INT;
      if (isIntra0)
      {
#if JVET_AG0164_AFFINE_GPM
        int intraIdx0 = mergeCand0 - GEO_MAX_ALL_INTER_UNI_CANDS;
#else
        int intraIdx0 = mergeCand0 - GEO_MAX_NUM_UNI_CANDS;
#endif
        uint8_t intraPred = geoIntraMPMList[splitDir][0][intraIdx0];
        mrgIntraCand0 = intraRDOBufIdx[intraPred];
        if (isChromaEnabled(pu.chromaFormat) && !isGeoIntraChromaAvail[mrgIntraCand0])
        {
          pu.intraDir[1] = intraPred;
          pu.gpmIntraFlag = true;
          m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cb());
          m_pcIntraSearch->predIntraAng(COMPONENT_Cb, geoIntraBuffer[mrgIntraCand0].Cb(), pu);
          m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cr());
          m_pcIntraSearch->predIntraAng(COMPONENT_Cr, geoIntraBuffer[mrgIntraCand0].Cr(), pu);
          pu.gpmIntraFlag = false;
          isGeoIntraChromaAvail[mrgIntraCand0] = 2;
        }
      }
#if TM_MRG
      else if (tmFlag0)
      {
        int geoTmType = g_geoTmShape[0][g_geoParams[splitDir][0]];
        mrgTmCand0 = mergeCand0 + (geoTmType - 1) * GEO_MAX_NUM_UNI_CANDS;
        if (isChromaEnabled(pu.chromaFormat) && !isGeoTmChromaAvail[mrgTmCand0])
        {
          mergeCtx[geoTmType].setMergeInfo(pu, mergeCand0);
#if JVET_AE0046_BI_GPM
          if (tmRefinePossible[mrgTmCand0])
          {
            CHECK(tmRefinePossible[mrgTmCand0] != PU::checkBDMVRCondition(pu, true), "The BDMVR condition mismatches with the ones in previous luma loop for GPM-TM");
            pu.bdmvrRefine = true;
            PU::spanPuMv2DmvrBuffer(pu, m_mvBufBDMVR4GPM[0], m_mvBufBDMVR4GPM[1]);
            ::memcpy(m_pcInterSearch->getBdofSubPuMvOffset(), m_mvBufEncBDOF4GPM[geoTmType][mergeCand0], sizeof(Mv)* BDOF_SUBPU_MAX_NUM);
            m_pcInterSearch->setLumaBdofReady(true);
          }
#endif
#if JVET_AG0164_AFFINE_GPM
          CHECK( pu.cu->affine, "Should not be affine");
#endif
          m_pcInterSearch->motionCompensation(pu, geoTmBuffer[mrgTmCand0], REF_PIC_LIST_X, false, true);
          isGeoTmChromaAvail[mrgTmCand0] = true;
#if JVET_AE0046_BI_GPM
          pu.bdmvrRefine = false;
          m_pcInterSearch->setLumaBdofReady(false);
#endif
        }
      }
#endif
#if JVET_AG0164_AFFINE_GPM
      else if (isAffine0)
      {
        if (isChromaEnabled(pu.chromaFormat))
        {
          if (mmvdFlag0 && !isGeoMMVDChromaAvail[mergeCand0][mmvdCand0])
          {
            affMergeCtx.setAffMergeInfo(pu, mergeCand0, mmvdCand0);
            if (pu.refIdx[0] >= 0)
            {
              PU::setAllAffineMv(pu, pu.mvAffi[0][0], pu.mvAffi[0][1], pu.mvAffi[0][2], REF_PIC_LIST_0);
            }
            if (pu.refIdx[1] >= 0)
            {
              PU::setAllAffineMv(pu, pu.mvAffi[1][0], pu.mvAffi[1][1], pu.mvAffi[1][2], REF_PIC_LIST_1);
            }
            m_pcInterSearch->motionCompensation(pu, geoMMVDBuf[mergeCand0][mmvdCand0], REF_PIC_LIST_X, false, true);
            pu.cu->affine = false;
            isGeoMMVDChromaAvail[mergeCand0][mmvdCand0] = 1;
          }
          else if (!mmvdFlag0 && !isGeoChromaAvail[mergeCand0])
          {
            affMergeCtx.setAffMergeInfo(pu, mergeCand0);
            if (pu.refIdx[0] >= 0)
            {
              PU::setAllAffineMv(pu, pu.mvAffi[0][0], pu.mvAffi[0][1], pu.mvAffi[0][2], REF_PIC_LIST_0);
            }
            if (pu.refIdx[1] >= 0)
            {
              PU::setAllAffineMv(pu, pu.mvAffi[1][0], pu.mvAffi[1][1], pu.mvAffi[1][2], REF_PIC_LIST_1);
            }
            m_pcInterSearch->motionCompensation(pu, geoBuffer[mergeCand0], REF_PIC_LIST_X, false, true);
            pu.cu->affine = false;
            isGeoChromaAvail[mergeCand0] = 1;
          }
        }
      }
#endif
      else if( mmvdFlag0 )
      {
        if( isChromaEnabled( pu.chromaFormat ) && !isGeoMMVDChromaAvail[mergeCand0][mmvdCand0] )
        {
#if TM_MRG
          mergeCtx[GEO_TM_OFF].setGeoMmvdMergeInfo( pu, mergeCand0, mmvdCand0 );
#else
          mergeCtx.setGeoMmvdMergeInfo( pu, mergeCand0, mmvdCand0 );
#endif
          m_pcInterSearch->motionCompensation( pu, geoMMVDBuf[mergeCand0][mmvdCand0], REF_PIC_LIST_X, false, true );
          isGeoMMVDChromaAvail[mergeCand0][mmvdCand0] = 1;
        }
      }
      else
      {
        if( isChromaEnabled( pu.chromaFormat ) && !isGeoChromaAvail[mergeCand0] )
        {
#if TM_MRG
          mergeCtx[GEO_TM_OFF].setMergeInfo( pu, mergeCand0 );
#else
          mergeCtx.setMergeInfo( pu, mergeCand0 );
#endif

#if JVET_AE0046_BI_GPM
          if (refinePossible[mergeCand0])
          {
            pu.bdmvrRefine = true;
            PU::spanPuMv2DmvrBuffer(pu, m_mvBufBDMVR4GPM[0], m_mvBufBDMVR4GPM[1]);
            ::memcpy(m_pcInterSearch->getBdofSubPuMvOffset(), m_mvBufEncBDOF4GPM[GEO_TM_OFF][mergeCand0], sizeof(Mv) * BDOF_SUBPU_MAX_NUM);
            m_pcInterSearch->setLumaBdofReady(true);
          }
#endif

          m_pcInterSearch->motionCompensation( pu, geoBuffer[mergeCand0], REF_PIC_LIST_X, false, true );
          isGeoChromaAvail[mergeCand0] = 1;

#if JVET_AE0046_BI_GPM
          pu.bdmvrRefine = false;
          m_pcInterSearch->setLumaBdofReady(false);
#endif
        }
      }

      if (isIntra1)
      {
#if JVET_AG0164_AFFINE_GPM
        int intraIdx1 = mergeCand1 - GEO_MAX_ALL_INTER_UNI_CANDS;
#else
        int intraIdx1 = mergeCand1 - GEO_MAX_NUM_UNI_CANDS;
#endif
        uint8_t intraPred = geoIntraMPMList[splitDir][1][intraIdx1];
        mrgIntraCand1 = intraRDOBufIdx[intraPred];
        if (isChromaEnabled(pu.chromaFormat) && !isGeoIntraChromaAvail[mrgIntraCand1])
        {
          pu.intraDir[1] = intraPred;
          pu.gpmIntraFlag = true;
          m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cb());
          m_pcIntraSearch->predIntraAng(COMPONENT_Cb, geoIntraBuffer[mrgIntraCand1].Cb(), pu);
          m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cr());
          m_pcIntraSearch->predIntraAng(COMPONENT_Cr, geoIntraBuffer[mrgIntraCand1].Cr(), pu);
          pu.gpmIntraFlag = false;
          isGeoIntraChromaAvail[mrgIntraCand1] = 2;
        }
      }
#if TM_MRG
      else if (tmFlag1)
      {
        int geoTmType = g_geoTmShape[1][g_geoParams[splitDir][0]];
        mrgTmCand1 = mergeCand1 + (geoTmType - 1) * GEO_MAX_NUM_UNI_CANDS;
        if (isChromaEnabled(pu.chromaFormat) && !isGeoTmChromaAvail[mrgTmCand1])
        {
          mergeCtx[geoTmType].setMergeInfo(pu, mergeCand1);
#if JVET_AE0046_BI_GPM
          if (tmRefinePossible[mrgTmCand1])
          {
            pu.bdmvrRefine = true;
            PU::spanPuMv2DmvrBuffer(pu, m_mvBufBDMVR4GPM[0], m_mvBufBDMVR4GPM[1]);
            ::memcpy(m_pcInterSearch->getBdofSubPuMvOffset(), m_mvBufEncBDOF4GPM[geoTmType][mergeCand1], sizeof(Mv) * BDOF_SUBPU_MAX_NUM);
            m_pcInterSearch->setLumaBdofReady(true);
          }
#endif

#if JVET_AG0164_AFFINE_GPM
          CHECK(pu.cu->affine, "Should not be affine");
#endif
          m_pcInterSearch->motionCompensation(pu, geoTmBuffer[mrgTmCand1], REF_PIC_LIST_X, false, true);
#if JVET_AE0046_BI_GPM
          pu.bdmvrRefine = false;
          m_pcInterSearch->setLumaBdofReady(false);
#endif
          isGeoTmChromaAvail[mrgTmCand1] = true;
        }
      }
#endif
#if JVET_AG0164_AFFINE_GPM
      else if (isAffine1 )
      {
        if (isChromaEnabled(pu.chromaFormat))
        {

          if (mmvdFlag1 && !isGeoMMVDChromaAvail[mergeCand1][mmvdCand1])
          {

            affMergeCtx.setAffMergeInfo(pu, mergeCand1, mmvdCand1);
            if (pu.refIdx[0] >= 0)
            {
              PU::setAllAffineMv(pu, pu.mvAffi[0][0], pu.mvAffi[0][1], pu.mvAffi[0][2], REF_PIC_LIST_0);
            }
            if (pu.refIdx[1] >= 0)
            {
              PU::setAllAffineMv(pu, pu.mvAffi[1][0], pu.mvAffi[1][1], pu.mvAffi[1][2], REF_PIC_LIST_1);
            }
            m_pcInterSearch->motionCompensation(pu, geoMMVDBuf[mergeCand1][mmvdCand1], REF_PIC_LIST_X, false, true);
            pu.cu->affine = false;
            isGeoMMVDChromaAvail[mergeCand1][mmvdCand1] = 1;

          }
          else if (!mmvdFlag1 && !isGeoChromaAvail[mergeCand1])
          {
            affMergeCtx.setAffMergeInfo(pu, mergeCand1);

            if (pu.refIdx[0] >= 0)
            {
              PU::setAllAffineMv(pu, pu.mvAffi[0][0], pu.mvAffi[0][1], pu.mvAffi[0][2], REF_PIC_LIST_0);
            }
            if (pu.refIdx[1] >= 0)
            {
              PU::setAllAffineMv(pu, pu.mvAffi[1][0], pu.mvAffi[1][1], pu.mvAffi[1][2], REF_PIC_LIST_1);
            }
            m_pcInterSearch->motionCompensation(pu, geoBuffer[mergeCand1], REF_PIC_LIST_X, false, true);
            pu.cu->affine = false;
            isGeoChromaAvail[mergeCand1] = 1;
          }
        }
      }
#endif
      else if( mmvdFlag1 )
      {
        if( isChromaEnabled( pu.chromaFormat ) && !isGeoMMVDChromaAvail[mergeCand1][mmvdCand1] )
        {
#if TM_MRG
          mergeCtx[GEO_TM_OFF].setGeoMmvdMergeInfo( pu, mergeCand1, mmvdCand1 );
#else
          mergeCtx.setGeoMmvdMergeInfo( pu, mergeCand1, mmvdCand1 );
#endif
          m_pcInterSearch->motionCompensation( pu, geoMMVDBuf[mergeCand1][mmvdCand1], REF_PIC_LIST_X, false, true );
          isGeoMMVDChromaAvail[mergeCand1][mmvdCand1] = 1;
        }
      }
      else
      {
        if( isChromaEnabled( pu.chromaFormat ) && !isGeoChromaAvail[mergeCand1] )
        {
#if TM_MRG
          mergeCtx[GEO_TM_OFF].setMergeInfo( pu, mergeCand1 );
#else
          mergeCtx.setMergeInfo( pu, mergeCand1 );
#endif
#if JVET_AE0046_BI_GPM
          if (refinePossible[mergeCand1])
          {
            pu.bdmvrRefine = true;
            PU::spanPuMv2DmvrBuffer(pu, m_mvBufBDMVR4GPM[0], m_mvBufBDMVR4GPM[1]);
            ::memcpy(m_pcInterSearch->getBdofSubPuMvOffset(), m_mvBufEncBDOF4GPM[GEO_TM_OFF][mergeCand1], sizeof(Mv) * BDOF_SUBPU_MAX_NUM);
            m_pcInterSearch->setLumaBdofReady(true);
          }
#endif
          m_pcInterSearch->motionCompensation( pu, geoBuffer[mergeCand1], REF_PIC_LIST_X, false, true );
#if JVET_AE0046_BI_GPM
          pu.bdmvrRefine = false;
          m_pcInterSearch->setLumaBdofReady(false);
#endif
          isGeoChromaAvail[mergeCand1] = 1;
        }
      }

#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
      geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx] = m_acGeoWeightedBuffer[candidateIdx * GEO_BLENDING_NUM + bldIdx].getBuf(localUnitArea);
#else
      geoCombinations[candidateIdx] = m_acGeoWeightedBuffer[candidateIdx].getBuf(localUnitArea);
#endif

      PelUnitBuf predSrc0, predSrc1;
      PelUnitBuf predSrcTemp0, predSrcTemp1;
      uint8_t* chromaAvailPtr0 = nullptr;
      uint8_t* chromaAvailPtr1 = nullptr;
      if (isIntra0)
      {
        predSrcTemp0 = geoIntraBuffer[mrgIntraCand0];
      }
#if TM_MRG
      else if (tmFlag0)
      {
        predSrc0 = geoTmBuffer[mrgTmCand0];
      }
#endif
#if JVET_AG0164_AFFINE_GPM
      else if (isAffine0 )
      {
        if (mmvdFlag0)
        {
          predSrc0 = geoMMVDBuf[mergeCand0][mmvdCand0];
          chromaAvailPtr0 = &isGeoMMVDChromaAvail[mergeCand0][mmvdCand0];
          predSrcTemp0 = geoMMVDTempBuf[mergeCand0][mmvdCand0];
        }
        else
        {
          predSrc0 = geoBuffer[mergeCand0];
          chromaAvailPtr0 = &isGeoChromaAvail[mergeCand0];
          predSrcTemp0 = geoTempBuf[mergeCand0];
        }
      }
#endif
      else if (mmvdFlag0)
      {
        predSrc0 = geoMMVDBuf[mergeCand0][mmvdCand0];
        chromaAvailPtr0 = &isGeoMMVDChromaAvail[mergeCand0][mmvdCand0];
        predSrcTemp0 = geoMMVDTempBuf[mergeCand0][mmvdCand0];
        if (isIntra1)
        {
#if TM_MRG
          mergeCtx[GEO_TM_OFF].setGeoMmvdMergeInfo(pu, mergeCand0, mmvdCand0);
#else
          mergeCtx.setGeoMmvdMergeInfo( pu, mergeCand0, mmvdCand0 );
#endif
        }
      }
      else
      {
        predSrc0 = geoBuffer[mergeCand0];
#if JVET_AG0164_AFFINE_GPM
        chromaAvailPtr0 = &isGeoChromaAvail[mergeCand0];
        predSrcTemp0 = geoTempBuf[mergeCand0];
#endif
      }

      if (isIntra1)
      {
        predSrcTemp1 = geoIntraBuffer[mrgIntraCand1];
      }
#if TM_MRG
      else if (tmFlag1)
      {
        predSrc1 = geoTmBuffer[mrgTmCand1];
      }
#endif
#if JVET_AG0164_AFFINE_GPM
      else if (isAffine1)  
      {
        if (mmvdFlag1)
        {
          predSrc1 = geoMMVDBuf[mergeCand1][mmvdCand1];
          chromaAvailPtr1 = &isGeoMMVDChromaAvail[mergeCand1][mmvdCand1];
          predSrcTemp1 = geoMMVDTempBuf[mergeCand1][mmvdCand1];
        }
        else
        {
          predSrc1 = geoBuffer[mergeCand1];
          chromaAvailPtr1 = &isGeoChromaAvail[mergeCand1];
          predSrcTemp1 = geoTempBuf[mergeCand1];
        }
      }
#endif
      else if (mmvdFlag1)
      {
        predSrc1 = geoMMVDBuf[mergeCand1][mmvdCand1];
        chromaAvailPtr1 = &isGeoMMVDChromaAvail[mergeCand1][mmvdCand1];
        predSrcTemp1 = geoMMVDTempBuf[mergeCand1][mmvdCand1];
        if (isIntra0)
        {
#if TM_MRG
          mergeCtx[GEO_TM_OFF].setGeoMmvdMergeInfo(pu, mergeCand1, mmvdCand1);
#else
          mergeCtx.setGeoMmvdMergeInfo( pu, mergeCand1, mmvdCand1 );
#endif
        }
      }
      else
      {
        predSrc1 = geoBuffer[mergeCand1];
#if JVET_AG0164_AFFINE_GPM
        chromaAvailPtr1 = &isGeoChromaAvail[mergeCand1];
        predSrcTemp1 = geoTempBuf[mergeCand1];
#endif
      }

      if (isIntra0 || isIntra1)
      {
        if (isChromaEnabled(pu.chromaFormat))
        {
          if (!isIntra0)
          {
            CHECK(!mmvdFlag0, "mmvdFlag0 must be true");
            if (*chromaAvailPtr0 < 2)
            {
              predSrcTemp0.Cb().roundToOutputBitdepth(predSrc0.Cb(), cu.slice->clpRng(COMPONENT_Cb));
              predSrcTemp0.Cr().roundToOutputBitdepth(predSrc0.Cr(), cu.slice->clpRng(COMPONENT_Cr));
              *chromaAvailPtr0 = 2;
            }
          }
          else if (!isIntra1)
          {
            CHECK(!mmvdFlag1, "mmvdFlag1 must be true");
            if (*chromaAvailPtr1 < 2)
            {
              predSrcTemp1.Cb().roundToOutputBitdepth(predSrc1.Cb(), cu.slice->clpRng(COMPONENT_Cb));
              predSrcTemp1.Cr().roundToOutputBitdepth(predSrc1.Cr(), cu.slice->clpRng(COMPONENT_Cr));
              *chromaAvailPtr1 = 2;
            }
          }
        }

#if ENABLE_OBMC
#if JVET_AG0164_AFFINE_GPM
        if (!isAffine0 && !isAffine1)
        {
#endif
#if JVET_W0123_TIMD_FUSION
#if JVET_AE0046_BI_GPM
#if TM_MRG
        pu.gpmDmvrRefinePart0 = isIntra0 ? false : ( mmvdFlag0 ? false : (tmFlag0 ? tmRefinePossible[mrgTmCand0] : refinePossible[mergeCand0]) );
        pu.gpmDmvrRefinePart1 = isIntra1 ? false : ( mmvdFlag1 ? false : (tmFlag1 ? tmRefinePossible[mrgTmCand1] : refinePossible[mergeCand1]) );
#else
        pu.gpmDmvrRefinePart0 = isIntra0 ? false : (mmvdFlag0 ? false : refinePossible[mergeCand0]);
        pu.gpmDmvrRefinePart1 = isIntra1 ? false : (mmvdFlag1 ? false : refinePossible[mergeCand1]);
#endif

        CHECK(pu.gpmDmvrRefinePart0 && pu.gpmDmvrRefinePart1, "This is not possible to have two parts with refine possible since one is intra");
        Mv* bdofSubPuMvOffset = nullptr;

        if (pu.gpmDmvrRefinePart0)
        {
          CHECK(mmvdFlag0, "MMVD0 = 1 should not have refine possibility, it has to be normal or TM");
#if TM_MRG
          if (tmFlag0)
          {
            int geoTmType = g_geoTmShape[0][g_geoParams[splitDir][0]];
            bdofSubPuMvOffset = m_mvBufEncBDOF4GPM[geoTmType][mergeCand0];
          }
          else
#endif
          {
            bdofSubPuMvOffset = m_mvBufEncBDOF4GPM[GEO_TM_OFF][mergeCand0];
          }
        }
        else if (pu.gpmDmvrRefinePart1)
        {
          CHECK(mmvdFlag1, "MMVD1 = 1 should not have refine possibility, it has to be normal or TM");
#if TM_MRG
          if (tmFlag1)
          {
            int geoTmType = g_geoTmShape[1][g_geoParams[splitDir][0]];
            bdofSubPuMvOffset = m_mvBufEncBDOF4GPM[geoTmType][mergeCand1];
          }
          else
#endif
          {
            bdofSubPuMvOffset = m_mvBufEncBDOF4GPM[GEO_TM_OFF][mergeCand1];
          }
        }

        PU::spanMotionInfo2(pu, MergeCtx(),
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
          0,
#endif
          nullptr, nullptr, bdofSubPuMvOffset);

        pu.gpmDmvrRefinePart0 = false;
        pu.gpmDmvrRefinePart1 = false;
#else
        PU::spanMotionInfo2(pu);
#endif
#else
        PU::spanMotionInfo(pu);
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
        if (!isIntra0)
        {
          geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx].copyFrom(predSrcTemp0);
          predSrcTemp0 = geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx];
        }
        else
        {
          geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx].copyFrom(predSrcTemp1);
          predSrcTemp1 = geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx];
        }
#else
        if (!isIntra0)
        {
          geoCombinations[candidateIdx].copyFrom(predSrcTemp0);
          predSrcTemp0 = geoCombinations[candidateIdx];
        }
        else
        {
          geoCombinations[candidateIdx].copyFrom(predSrcTemp1);
          predSrcTemp1 = geoCombinations[candidateIdx];
        }
#endif
        cu.isobmcMC = true;
        cu.obmcFlag = true;
        m_pcInterSearch->subBlockOBMC(pu, !isIntra0 ? &predSrcTemp0 : &predSrcTemp1);
        cu.isobmcMC = false;
#endif
#if JVET_AG0164_AFFINE_GPM
        }
        else
        {
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
          if (!isIntra0)
          {
            geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx].copyFrom(predSrcTemp0);
            predSrcTemp0 = geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx];

            affMergeCtx.setAffMergeInfo(pu, mergeCand0, mmvdCand0);
            PU::spanMotionInfo2(pu, MergeCtx());
            cu.isobmcMC = true;
            cu.obmcFlag = true;
            m_pcInterSearch->subBlockOBMC(pu, &predSrcTemp0);
            pu.cu->affine = false;
            cu.isobmcMC = false;

          }
          else
          {
            geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx].copyFrom(predSrcTemp1);
            predSrcTemp1 = geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx];

            affMergeCtx.setAffMergeInfo(pu, mergeCand1, mmvdCand1);
            PU::spanMotionInfo2(pu, MergeCtx());
            cu.isobmcMC = true;
            cu.obmcFlag = true;
            m_pcInterSearch->subBlockOBMC(pu, &predSrcTemp1);
            pu.cu->affine = false;
            cu.isobmcMC = false;
          }
#else
          if (!isIntra0)
          {
            geoCombinations[candidateIdx].copyFrom(predSrcTemp0);
            predSrcTemp0 = geoCombinations[candidateIdx];
          }
          else
          {
            geoCombinations[candidateIdx].copyFrom(predSrcTemp1);
            predSrcTemp1 = geoCombinations[candidateIdx];
          }
#endif
        }
#endif
        if (pu.cs->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
        {
          if (!isIntra0) // Inter+Intra
          {
            predSrcTemp0.Y().rspSignal(predSrcTemp0.Y(), m_pcReshape->getFwdLUT());
          }
          else if (!isIntra1) // Intra+Inter
          {
            predSrcTemp1.Y().rspSignal(predSrcTemp1.Y(), m_pcReshape->getFwdLUT());
          }
        }
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
        m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, bldIdx, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx], predSrcTemp0, predSrcTemp1);
        m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, bldIdx, CHANNEL_TYPE_CHROMA, geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx], predSrcTemp0, predSrcTemp1);
      }
#if JVET_AG0164_AFFINE_GPM
      else if (isAffine0 || isAffine1)
      {
        if (isChromaEnabled(pu.chromaFormat))
        {
          if (*chromaAvailPtr0 < 2)
          {
            predSrcTemp0.Cb().roundToOutputBitdepth(predSrc0.Cb(), cu.slice->clpRng(COMPONENT_Cb));
            predSrcTemp0.Cr().roundToOutputBitdepth(predSrc0.Cr(), cu.slice->clpRng(COMPONENT_Cr));
            *chromaAvailPtr0 = 2;
          }

          if (*chromaAvailPtr1 < 2)
          {
            predSrcTemp1.Cb().roundToOutputBitdepth(predSrc1.Cb(), cu.slice->clpRng(COMPONENT_Cb));
            predSrcTemp1.Cr().roundToOutputBitdepth(predSrc1.Cr(), cu.slice->clpRng(COMPONENT_Cr));
            *chromaAvailPtr1 = 2;
          }
        }

        int geoCombIdx = candidateIdx * GEO_BLENDING_NUM + bldIdx;
        predSrcTemp0 = mmvdFlag0? geoMMVDTempBuf[mergeCand0][mmvdCand0]: geoTempBuf[mergeCand0];
        predSrcTemp1 = mmvdFlag1? geoMMVDTempBuf[mergeCand1][mmvdCand1]: geoTempBuf[mergeCand1];

        PelUnitBuf  obmcBuf0 = m_ciipBuffer[0].getBuf(localUnitArea);   // Borrow the CIIP buffer
        PelUnitBuf  obmcBuf1 = m_ciipBuffer[1].getBuf(localUnitArea);   // Borrow the CIIP buffer

        pu.gpmDmvrRefinePart0 = pu.gpmDmvrRefinePart1 = false;
        performOBMCOnOnePartition(pu, mergeCand0, geoCombIdx, predSrcTemp0, obmcBuf0, mmvdCand0);
        performOBMCOnOnePartition(pu, mergeCand1, geoCombIdx, predSrcTemp1, obmcBuf1, mmvdCand1);
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
        m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, bldIdx, CHANNEL_TYPE_LUMA, geoCombinations[geoCombIdx], obmcBuf0, obmcBuf1);
        m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, bldIdx, CHANNEL_TYPE_CHROMA, geoCombinations[geoCombIdx], obmcBuf0, obmcBuf1);
#else
        m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx], predSrc0, predSrc1);
        m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, CHANNEL_TYPE_CHROMA, geoCombinations[candidateIdx], predSrc0, predSrc1);
#endif 
      }
#endif
      else
      {
        m_pcInterSearch->weightedGeoBlk(pu, splitDir, bldIdx, CHANNEL_TYPE_CHROMA, geoCombinations[candidateIdx * GEO_BLENDING_NUM + bldIdx], predSrc0, predSrc1);
      }
#else
        m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx], predSrcTemp0, predSrcTemp1);
        m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, CHANNEL_TYPE_CHROMA, geoCombinations[candidateIdx], predSrcTemp0, predSrcTemp1);
      }
      else
      {
        m_pcInterSearch->weightedGeoBlk(pu, splitDir, CHANNEL_TYPE_CHROMA, geoCombinations[candidateIdx], predSrc0, predSrc1);
      }
#endif
    }
#if JVET_AG0164_AFFINE_GPM
    pu.affineGPM[0] = 0;
    pu.affineGPM[1] = 0;
#endif
#else
    for (uint8_t i = 0; i < geoNumMrgSATDCand && isChromaEnabled(pu.chromaFormat); i++)
    {
      if (isNonMMVDListIdx[i])
      {
        continue;
      }
      uint8_t candidateIdx = geoRdModeList[i];
      int splitDir = geoSplitDirList[candidateIdx];
      int mergeCand0 = geoMergeCand0[candidateIdx];
      int mergeCand1 = geoMergeCand1[candidateIdx];
#if TM_MRG
      bool tmFlag0 = (geoMmvdCand0[candidateIdx] == (GPM_EXT_MMVD_MAX_REFINE_NUM + 1));
      bool tmFlag1 = (geoMmvdCand1[candidateIdx] == (GPM_EXT_MMVD_MAX_REFINE_NUM + 1));
      bool mmvdFlag0 = (geoMmvdCand0[candidateIdx] >= 1 && geoMmvdCand0[candidateIdx] <= GPM_EXT_MMVD_MAX_REFINE_NUM);
      bool mmvdFlag1 = (geoMmvdCand1[candidateIdx] >= 1 && geoMmvdCand1[candidateIdx] <= GPM_EXT_MMVD_MAX_REFINE_NUM);
      int  mmvdCand0 = (mmvdFlag0 ? (geoMmvdCand0[candidateIdx] - 1) : MAX_INT);
      int  mmvdCand1 = (mmvdFlag1 ? (geoMmvdCand1[candidateIdx] - 1) : MAX_INT);
#else
      int mmvdCand0 = geoMmvdCand0[candidateIdx] - 1;
      int mmvdCand1 = geoMmvdCand1[candidateIdx] - 1;

      bool mmvdFlag0 = (mmvdCand0 >= 0);
      bool mmvdFlag1 = (mmvdCand1 >= 0);
#endif

#if TM_MRG
      int mrgTmCand0 = MAX_INT, mrgTmCand1 = MAX_INT;
      if (tmFlag0)
      {
        int geoTmType = g_geoTmShape[0][g_geoParams[splitDir][0]];
        mrgTmCand0 = mergeCand0 + (geoTmType - 1) * GEO_MAX_NUM_UNI_CANDS;
        if (!isGeoTmChromaAvail[mrgTmCand0])
        {
          mergeCtx[geoTmType].setMergeInfo(pu, mergeCand0);
          m_pcInterSearch->motionCompensation(pu, geoTmBuffer[mrgTmCand0], REF_PIC_LIST_X, false, true);
          isGeoTmChromaAvail[mrgTmCand0] = true;
        }
      }
      else
#endif
        if (mmvdFlag0)
        {
          if (!isGeoMMVDChromaAvail[mergeCand0][mmvdCand0])
          {
#if TM_MRG
            mergeCtx[GEO_TM_OFF].setGeoMmvdMergeInfo(pu, mergeCand0, mmvdCand0);
#else
            mergeCtx.setGeoMmvdMergeInfo(pu, mergeCand0, mmvdCand0);
#endif
            m_pcInterSearch->motionCompensation(pu, geoMMVDBuf[mergeCand0][mmvdCand0], REF_PIC_LIST_X, false, true);
            isGeoMMVDChromaAvail[mergeCand0][mmvdCand0] = true;
          }
        }
        else
        {
          if (!isGeoChromaAvail[mergeCand0])
          {
#if TM_MRG
            mergeCtx[GEO_TM_OFF].setMergeInfo(pu, mergeCand0);
#else
            mergeCtx.setMergeInfo(pu, mergeCand0);
#endif
            m_pcInterSearch->motionCompensation(pu, geoBuffer[mergeCand0], REF_PIC_LIST_X, false, true);
            isGeoChromaAvail[mergeCand0] = true;
          }
        }

#if TM_MRG
      if (tmFlag1)
      {
        int geoTmType = g_geoTmShape[1][g_geoParams[splitDir][0]];
        mrgTmCand1 = mergeCand1 + (geoTmType - 1) * GEO_MAX_NUM_UNI_CANDS;
        if (!isGeoTmChromaAvail[mrgTmCand1])
        {
          mergeCtx[geoTmType].setMergeInfo(pu, mergeCand1);
          m_pcInterSearch->motionCompensation(pu, geoTmBuffer[mrgTmCand1], REF_PIC_LIST_X, false, true);
          isGeoTmChromaAvail[mrgTmCand1] = true;
        }
      }
      else
#endif
        if (mmvdFlag1)
        {
          if (!isGeoMMVDChromaAvail[mergeCand1][mmvdCand1])
          {
#if TM_MRG
            mergeCtx[GEO_TM_OFF].setGeoMmvdMergeInfo(pu, mergeCand1, mmvdCand1);
#else
            mergeCtx.setGeoMmvdMergeInfo(pu, mergeCand1, mmvdCand1);
#endif
            m_pcInterSearch->motionCompensation(pu, geoMMVDBuf[mergeCand1][mmvdCand1], REF_PIC_LIST_X, false, true);
            isGeoMMVDChromaAvail[mergeCand1][mmvdCand1] = true;
          }
        }
        else
        {
          if (!isGeoChromaAvail[mergeCand1])
          {
#if TM_MRG
            mergeCtx[GEO_TM_OFF].setMergeInfo(pu, mergeCand1);
#else
            mergeCtx.setMergeInfo(pu, mergeCand1);
#endif
            m_pcInterSearch->motionCompensation(pu, geoBuffer[mergeCand1], REF_PIC_LIST_X, false, true);
            isGeoChromaAvail[mergeCand1] = true;
          }
        }

      geoCombinations[candidateIdx] = m_acGeoWeightedBuffer[candidateIdx].getBuf(localUnitArea);
#if TM_MRG
      PelUnitBuf predSrc0, predSrc1;
      if (tmFlag0)
      {
        predSrc0 = geoTmBuffer[mrgTmCand0];
      }
      else if (mmvdFlag0)
      {
        predSrc0 = geoMMVDBuf[mergeCand0][mmvdCand0];
      }
      else
      {
        predSrc0 = geoBuffer[mergeCand0];
      }

      if (tmFlag1)
      {
        predSrc1 = geoTmBuffer[mrgTmCand1];
      }
      else if (mmvdFlag1)
      {
        predSrc1 = geoMMVDBuf[mergeCand1][mmvdCand1];
      }
      else
      {
        predSrc1 = geoBuffer[mergeCand1];
      }

      m_pcInterSearch->weightedGeoBlk(pu, splitDir, CHANNEL_TYPE_CHROMA, geoCombinations[candidateIdx], predSrc0, predSrc1);
#else
      m_pcInterSearch->weightedGeoBlk(pu, splitDir, CHANNEL_TYPE_CHROMA, geoCombinations[candidateIdx], (mmvdFlag0 ? geoMMVDBuf[mergeCand0][mmvdCand0] : geoBuffer[mergeCand0]), (mmvdFlag1 ? geoMMVDBuf[mergeCand1][mmvdCand1] : geoBuffer[mergeCand1]));
#endif
    }
#endif

    std::memset(geocandHasNoResidual, false, GEO_MAX_TRY_WEIGHTED_SAD * sizeof(bool));
    tempCS->initStructData(encTestMode.qp);
    uint8_t iteration = 2, iterationBegin = 0;
    for (uint8_t noResidualPass = iterationBegin; noResidualPass < iteration; ++noResidualPass)
    {
      for (uint8_t mrgHADIdx = 0; mrgHADIdx < geoNumMrgSATDCand; mrgHADIdx++)
      {
        if (isNonMMVDListIdx[mrgHADIdx])
        {
          continue;
        }
        uint8_t candidateIdx = geoRdModeList[mrgHADIdx];
        if (((noResidualPass != 0) && geocandHasNoResidual[candidateIdx])
          || ((noResidualPass == 0) && bestIsSkip))
        {
          continue;
        }
        CodingUnit &cu = tempCS->addCU(tempCS->area, pm.chType);
        pm.setCUData(cu);
        cu.predMode = MODE_INTER;
        cu.slice = tempCS->slice;
        cu.tileIdx = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
        cu.qp = encTestMode.qp;
        cu.affine = false;
        cu.mtsFlag = false;
#if INTER_LIC
        cu.licFlag = false;
#endif
        cu.bcwIdx = BCW_DEFAULT;
        cu.geoFlag = true;
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
        cu.geoBlendFlag = false;
#endif
#if JVET_AG0061_INTER_LFNST_NSPT
        cu.lfnstFlag = false;
        cu.lfnstIdx  = 0;
#endif
        cu.imv = 0;
        cu.mmvdSkip = false;
        cu.skip = false;
        cu.mipFlag = false;
        cu.bdpcmMode = 0;
        PredictionUnit &pu = tempCS->addPU(cu, pm.chType);
        pu.mergeFlag = true;
        pu.regularMergeFlag = false;
        pu.geoSplitDir = geoSplitDirList[candidateIdx];
        pu.geoMergeIdx0 = geoMergeCand0[candidateIdx];
        pu.geoMergeIdx1 = geoMergeCand1[candidateIdx];
#if JVET_AE0046_BI_GPM
        PU::setGpmDirMode(pu);
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
        pu.geoBldIdx = geoBldList[mrgHADIdx];
#endif
#if JVET_Y0065_GPM_INTRA
#if JVET_AG0164_AFFINE_GPM
        pu.gpmIntraFlag = pu.geoMergeIdx0 >= GEO_MAX_ALL_INTER_UNI_CANDS || pu.geoMergeIdx1 >= GEO_MAX_ALL_INTER_UNI_CANDS;
        if (pu.geoMergeIdx0 >= GEO_MAX_ALL_INTER_UNI_CANDS)
        {
          memcpy(m_pcIntraSearch->m_intraMPM, geoIntraMPMList[pu.geoSplitDir][0], sizeof(uint8_t) * GEO_MAX_NUM_INTRA_CANDS);
        }
        if (pu.geoMergeIdx1 >= GEO_MAX_ALL_INTER_UNI_CANDS)
        {
          memcpy(m_pcIntraSearch->m_intraMPM + GEO_MAX_NUM_INTRA_CANDS, geoIntraMPMList[pu.geoSplitDir][1], sizeof(uint8_t) * GEO_MAX_NUM_INTRA_CANDS);
        }
#else
        pu.gpmIntraFlag = pu.geoMergeIdx0 >= GEO_MAX_NUM_UNI_CANDS || pu.geoMergeIdx1 >= GEO_MAX_NUM_UNI_CANDS;
        if (pu.geoMergeIdx0 >= GEO_MAX_NUM_UNI_CANDS)
        {
          memcpy(m_pcIntraSearch->m_intraMPM, geoIntraMPMList[pu.geoSplitDir][0], sizeof(uint8_t)*GEO_MAX_NUM_INTRA_CANDS);
        }
        if (pu.geoMergeIdx1 >= GEO_MAX_NUM_UNI_CANDS)
        {
          memcpy(m_pcIntraSearch->m_intraMPM +GEO_MAX_NUM_INTRA_CANDS, geoIntraMPMList[pu.geoSplitDir][1], sizeof(uint8_t)*GEO_MAX_NUM_INTRA_CANDS);
        }
#endif
#if ENABLE_DIMD
        cu.dimdMode = dimdMode;
#endif
#if JVET_W0123_TIMD_FUSION
        cu.timdMode = timdMode;
#if JVET_AC0094_REF_SAMPLES_OPT
        cu.timdModeCheckWA = timdModeCheckWA;
#endif
#endif
#endif
#if TM_MRG
        pu.geoTmFlag0 = (geoMmvdCand0[candidateIdx] == (GPM_EXT_MMVD_MAX_REFINE_NUM + 1));
        pu.geoTmFlag1 = (geoMmvdCand1[candidateIdx] == (GPM_EXT_MMVD_MAX_REFINE_NUM + 1));
        pu.geoMMVDFlag0 = (geoMmvdCand0[candidateIdx] >= 1 && geoMmvdCand0[candidateIdx] <= GPM_EXT_MMVD_MAX_REFINE_NUM);
        pu.geoMMVDFlag1 = (geoMmvdCand1[candidateIdx] >= 1 && geoMmvdCand1[candidateIdx] <= GPM_EXT_MMVD_MAX_REFINE_NUM);
        CHECK(pu.geoTmFlag0 != pu.geoTmFlag1, "TM flag cannot be enabled/disabled for two partitions separately");
        pu.tmMergeFlag = pu.geoTmFlag0;
#else
        pu.geoMMVDFlag0 = (geoMmvdCand0[candidateIdx] > 0);
        pu.geoMMVDFlag1 = (geoMmvdCand1[candidateIdx] > 0);
#endif

#if JVET_AG0164_AFFINE_GPM
        pu.affineGPM[0] = pu.affineGPM[1] = 0;
        if (!pu.geoTmFlag0 && pu.geoMergeIdx0 < GEO_MAX_ALL_INTER_UNI_CANDS && pu.geoMergeIdx0 >= numRegularGpmMergeCand)
        {
          pu.geoMergeIdx0 -= numRegularGpmMergeCand;
          pu.affineGPM[0] = 1;
        }
        if (!pu.geoTmFlag1 && pu.geoMergeIdx1 < GEO_MAX_ALL_INTER_UNI_CANDS && pu.geoMergeIdx1 >= numRegularGpmMergeCand)
        {
          pu.geoMergeIdx1 -= numRegularGpmMergeCand;
          pu.affineGPM[1] = 1;
        }
#endif

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
        if(sps.getUseAltGPMSplitModeCode())
        {
#if JVET_W0097_GPM_MMVD_TM && TM_MRG
          if(pu.tmMergeFlag)
          {
            int geoSyntaxMode = m_pcInterSearch->convertGeoSplitModeToSyntax(pu.geoSplitDir, pu.geoMergeIdx0, pu.geoMergeIdx1, tmMmvdBufIdx0 - 1, tmMmvdBufIdx1 - 1);
            CHECK(geoSyntaxMode < 0 || geoSyntaxMode >= GEO_NUM_SIG_PARTMODE, "Invalid GEO split direction!");
            CHECK(pu.geoMMVDFlag0 || pu.geoMMVDFlag1, "GPM MMVD should not be used in GPM-TM mode");
            pu.geoSyntaxMode = (uint8_t)geoSyntaxMode;
          }
          else
#endif
          {
            int  mmvdCandTmp0 = geoMmvdCand0[candidateIdx];
            int  mmvdCandTmp1 = geoMmvdCand1[candidateIdx];
#if JVET_AG0164_AFFINE_GPM
            int cand0 = pu.geoMergeIdx0;
            int cand1 = pu.geoMergeIdx1;
            if (pu.affineGPM[0])
            {
              cand0 += numRegularGpmMergeCand;
            }
            if (pu.affineGPM[1])
            {
              cand1 += numRegularGpmMergeCand;
            }
            int geoSyntaxMode = m_pcInterSearch->convertGeoSplitModeToSyntax(pu.geoSplitDir, cand0, cand1, mmvdCandTmp0 - 1, mmvdCandTmp1 - 1);
#else
            int geoSyntaxMode = m_pcInterSearch->convertGeoSplitModeToSyntax(pu.geoSplitDir, pu.geoMergeIdx0, pu.geoMergeIdx1, mmvdCandTmp0 - 1, mmvdCandTmp1 - 1);
#endif
            CHECK(geoSyntaxMode < 0 || geoSyntaxMode >= GEO_NUM_SIG_PARTMODE, "Invalid GEO split direction!");
            CHECK(!pu.geoMMVDFlag0 && !pu.geoMMVDFlag1, "GPM MMVD has to be used at least for one partition");
            pu.geoSyntaxMode = (uint8_t)geoSyntaxMode;
          }
        }
#endif

        if (pu.geoMMVDFlag0)
        {
          pu.geoMMVDIdx0 = (geoMmvdCand0[candidateIdx] - 1);
        }
        if (pu.geoMMVDFlag1)
        {
          pu.geoMMVDIdx1 = (geoMmvdCand1[candidateIdx] - 1);
        }
        pu.mmvdMergeFlag = false;
        pu.mmvdMergeIdx = MAX_UCHAR;
#if JVET_AE0046_BI_GPM
#if JVET_AG0164_AFFINE_GPM
        if (pu.geoMergeIdx0 >= GEO_MAX_ALL_INTER_UNI_CANDS)
#else
        if (pu.geoMergeIdx0 >= GEO_MAX_NUM_UNI_CANDS)
#endif
        {
          pu.gpmDmvrRefinePart0 = false;
        }
        else if (pu.geoMMVDFlag0)
        {
          pu.gpmDmvrRefinePart0 = false;
        }
#if TM_MRG
        else if (pu.geoTmFlag0)
        {
          int geoTmType = g_geoTmShape[0][g_geoParams[pu.geoSplitDir][0]];
          int mrgTmCand0 = pu.geoMergeIdx0 + (geoTmType - 1) * GEO_MAX_NUM_UNI_CANDS;
          pu.gpmDmvrRefinePart0 = tmRefinePossible[mrgTmCand0];
        }
#endif
        else
        {
          pu.gpmDmvrRefinePart0 = refinePossible[pu.geoMergeIdx0];
        }
#if JVET_AG0164_AFFINE_GPM
        if (pu.geoMergeIdx1 >= GEO_MAX_ALL_INTER_UNI_CANDS)
#else
        if (pu.geoMergeIdx1 >= GEO_MAX_NUM_UNI_CANDS)
#endif
        {
          pu.gpmDmvrRefinePart1 = false;
        }
        else if (pu.geoMMVDFlag1)
        {
          pu.gpmDmvrRefinePart1 = false;
        }
#if TM_MRG
        else if (pu.geoTmFlag1)
        {
          int geoTmType = g_geoTmShape[1][g_geoParams[pu.geoSplitDir][0]];
          int mrgTmCand1 = pu.geoMergeIdx1 + (geoTmType - 1) * GEO_MAX_NUM_UNI_CANDS;
          pu.gpmDmvrRefinePart1 = tmRefinePossible[mrgTmCand1];
        }
#endif
        else
        {
          pu.gpmDmvrRefinePart1 = refinePossible[pu.geoMergeIdx1];
        }
#endif
#if TM_MRG
        MergeCtx* mrgTmCtx0 = (pu.geoTmFlag0 == 0 ? nullptr : &mergeCtx[g_geoTmShape[0][g_geoParams[pu.geoSplitDir][0]]]);
        MergeCtx* mrgTmCtx1 = (pu.geoTmFlag1 == 0 ? nullptr : &mergeCtx[g_geoTmShape[1][g_geoParams[pu.geoSplitDir][0]]]);
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
#if JVET_AE0046_BI_GPM
        Mv* bdofSubPuMvOffsetPart0 = pu.gpmDmvrRefinePart0 ? (pu.geoTmFlag0 ? m_mvBufEncBDOF4GPM[g_geoTmShape[0][g_geoParams[pu.geoSplitDir][0]]][pu.geoMergeIdx0] : m_mvBufEncBDOF4GPM[GEO_TM_OFF][pu.geoMergeIdx0]) : nullptr;
        Mv* bdofSubPuMvOffsetPart1 = pu.gpmDmvrRefinePart1 ? (pu.geoTmFlag1 ? m_mvBufEncBDOF4GPM[g_geoTmShape[1][g_geoParams[pu.geoSplitDir][0]]][pu.geoMergeIdx1] : m_mvBufEncBDOF4GPM[GEO_TM_OFF][pu.geoMergeIdx1]) : nullptr;

        PU::spanGeoMMVDMotionInfo(pu, mergeCtx[GEO_TM_OFF]
#if JVET_AG0164_AFFINE_GPM
          , affMergeCtx
#endif
          , *mrgTmCtx0, *mrgTmCtx1, pu.geoSplitDir, pu.geoMergeIdx0, pu.geoMergeIdx1, pu.geoTmFlag0, pu.geoMMVDFlag0, pu.geoMMVDIdx0, pu.geoTmFlag1, pu.geoMMVDFlag1, pu.geoMMVDIdx1, pu.geoBldIdx, m_pcIntraSearch->m_intraMPM,
          pu.gpmDmvrRefinePart0, pu.gpmDmvrRefinePart1, bdofSubPuMvOffsetPart0, bdofSubPuMvOffsetPart1);
#else
        PU::spanGeoMMVDMotionInfo(pu, mergeCtx[GEO_TM_OFF], *mrgTmCtx0, *mrgTmCtx1, pu.geoSplitDir, pu.geoMergeIdx0, pu.geoMergeIdx1, pu.geoTmFlag0, pu.geoMMVDFlag0, pu.geoMMVDIdx0, pu.geoTmFlag1, pu.geoMMVDFlag1, pu.geoMMVDIdx1, pu.geoBldIdx, m_pcIntraSearch->m_intraMPM);
#endif
#else
        PU::spanGeoMMVDMotionInfo(pu, mergeCtx[GEO_TM_OFF], *mrgTmCtx0, *mrgTmCtx1, pu.geoSplitDir, pu.geoMergeIdx0, pu.geoMergeIdx1, pu.geoTmFlag0, pu.geoMMVDFlag0, pu.geoMMVDIdx0, pu.geoTmFlag1, pu.geoMMVDFlag1, pu.geoMMVDIdx1);
#endif
#else
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
        PU::spanGeoMMVDMotionInfo(pu, mergeCtx, pu.geoSplitDir, pu.geoMergeIdx0, pu.geoMergeIdx1, pu.geoMMVDFlag0, pu.geoMMVDIdx0, pu.geoMMVDFlag1, pu.geoMMVDIdx1, pu.geoBldIdx);
#else
        PU::spanGeoMMVDMotionInfo(pu, mergeCtx, pu.geoSplitDir, pu.geoMergeIdx0, pu.geoMergeIdx1, pu.geoMMVDFlag0, pu.geoMMVDIdx0, pu.geoMMVDFlag1, pu.geoMMVDIdx1);
#endif
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
        tempCS->getPredBuf().copyFrom(geoCombinations[candidateIdx * GEO_BLENDING_NUM + pu.geoBldIdx]);
#else
        tempCS->getPredBuf().copyFrom(geoCombinations[candidateIdx]);
#endif
#if ENABLE_OBMC
#if JVET_Y0065_GPM_INTRA
        if (!pu.gpmIntraFlag
#if JVET_AG0164_AFFINE_GPM 
          && !pu.affineGPM[0] && !pu.affineGPM[1]
#endif
          )
        {
#endif
        cu.isobmcMC = true;
        cu.obmcFlag = true;
        m_pcInterSearch->subBlockOBMC(pu);
        cu.isobmcMC = false;
#if JVET_Y0065_GPM_INTRA
        }
#endif
#endif

        xEncodeInterResidual(tempCS, bestCS, pm, encTestMode, noResidualPass, (noResidualPass == 0 ? &geocandHasNoResidual[candidateIdx] : NULL));

        if (m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip)
        {
          bestIsSkip = bestCS->getCU(pm.chType)->rootCbf == 0;
        }
        tempCS->initStructData(encTestMode.qp);
      }
    }
  }

#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  // encoder re-order with uiCost
  for (int i = 0; i < (numGeoBlendInfoCand - 1); i++)
  {
    for (int j = (i + 1); j < numGeoBlendInfoCand; j++)
    {
      if (geoBlendInfo[j].uiCost < geoBlendInfo[i].uiCost)
      {
        std::swap(geoBlendInfo[i], geoBlendInfo[j]);
      }
    }
  }

  bool  bStopRdoLoop = false;
  bool  bIsBestCost = false;  // current CU-geoBlend is best
  tempCS->initStructData(encTestMode.qp);

  for (uint8_t i = 0; i < numGeoBlendInfoCand && !bStopRdoLoop; i++)
  {
    double geoBlendFastRatio      = MRG_FAST_RATIO;
    double geoBlendFastRatioBest  = 1.0;
    geoBlendFastRatioBest         = 1.25;
    if (geoBlendInfo[i].uiCost > geoBlendFastRatio * bestEstimGeoBlendPredCost || geoBlendInfo[i].uiCost > geoBlendFastRatioBest * getMergeBestSATDCost())
    {
      continue;
    }

    bool  geoBlendCandHasNoResidual = false;
    for (uint8_t noResidualPass = iterationBegin; noResidualPass < iteration; ++noResidualPass)
    {
      if (((noResidualPass != 0) && geoBlendCandHasNoResidual)
        //  || ((noResidualPass == 0) && bestIsSkip)
        )
      {
        continue;
      }

      GeoBlendInfo& geoBI = geoBlendInfo[i];

      PelUnitBuf blendBuffer = m_acMergeBuffer[geoBI.idxBufGeo].getBuf(localUnitArea);

      CodingUnit &cu = tempCS->addCU(tempCS->area, pm.chType);
      pm.setCUData(cu);
      cu.predMode = MODE_INTER;
      cu.slice = tempCS->slice;
      cu.tileIdx = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
      cu.qp = encTestMode.qp;
      cu.affine = false;
      cu.mtsFlag = false;
#if INTER_LIC
      cu.licFlag = false;
#endif
      cu.bcwIdx = BCW_DEFAULT;
      cu.geoBlendFlag = true;
      cu.blendModel.copy(geoBI.blendModel);
      cu.geoFlag = true;
      cu.imv = 0;
      cu.mmvdSkip = false;
      cu.skip = false;
      cu.mipFlag = false;
      cu.bdpcmMode = 0;
      PredictionUnit &pu = tempCS->addPU(cu, pm.chType);
      pu.mergeFlag = true;
      pu.regularMergeFlag = false;
      pu.geoSplitDir = 0;
      pu.geoMergeIdx0 = geoBI.iMergeIdx;
      pu.geoMergeIdx1 = geoBI.iMergeIdx;
#if JVET_AG0164_AFFINE_GPM
      pu.affineGPM[0] = 0;
      pu.affineGPM[1] = 0;
#endif
#if JVET_AE0046_BI_GPM
      PU::setGpmDirMode(pu);
#endif
#if JVET_Y0065_GPM_INTRA
      pu.gpmIntraFlag = false;
#if ENABLE_DIMD
      cu.dimdMode = dimdMode;
#endif
#if JVET_W0123_TIMD_FUSION
      cu.timdMode = timdMode;
#if JVET_AC0094_REF_SAMPLES_OPT
      cu.timdModeCheckWA = timdModeCheckWA;
#endif
#endif
#endif
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
      pu.geoBldIdx = -1;
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
      if (sps.getUseAltGPMSplitModeCode())
      {
        pu.geoSyntaxMode = 0;
      }
#endif
#if TM_MRG
      pu.tmMergeFlag = false;
      pu.geoTmFlag0 = false;
      pu.geoTmFlag1 = false;
#endif
      pu.geoMMVDFlag0 = false;
      pu.geoMMVDFlag1 = false;

      pu.mmvdMergeFlag = false;
      pu.mmvdMergeIdx = MAX_UCHAR;

      if (noResidualPass == iterationBegin)
      {
        // do chroma (luma has been done yet)
        for (int k = 0; k < 2; k++)
        {
          const int mergeCand = geoBI.mergeCand[k];
          mergeCtx[GEO_TM_OFF].setMergeInfo(pu, mergeCand);
          if (isGeoChromaAvail[mergeCand] == 2)
          {
            continue;
          }
#if JVET_AE0046_BI_GPM
          if (refinePossible[mergeCand])
          {
            pu.bdmvrRefine = true;
            PU::spanPuMv2DmvrBuffer(pu, m_mvBufBDMVR4GPM[0], m_mvBufBDMVR4GPM[1]);
            ::memcpy(m_pcInterSearch->getBdofSubPuMvOffset(), m_mvBufEncBDOF4GPM[GEO_TM_OFF][mergeCand], sizeof(Mv) * BDOF_SUBPU_MAX_NUM);
            m_pcInterSearch->setLumaBdofReady(true);
          }
#endif
          m_pcInterSearch->motionCompensation(pu, geoBuffer[mergeCand], REF_PIC_LIST_X, false, true);

#if JVET_AE0046_BI_GPM
          pu.bdmvrRefine = false;
          m_pcInterSearch->setLumaBdofReady(false);
#endif
        }
      }

      mergeCtx[GEO_TM_OFF].setMergeInfo(pu, 0);

      pu.interDir = 3;
      pu.mergeIdx = MAX_UCHAR;
      pu.geoMergeIdx0 = geoBI.mergeCand[0];
      pu.geoMergeIdx1 = geoBI.mergeCand[1];

      pu.cu->blendModel.copy(geoBI.blendModel);

      if (noResidualPass == iterationBegin)
      {
        m_pcInterSearch->weightedBlend(pu, blendBuffer, geoBuffer[geoBI.mergeCand[0]], geoBuffer[geoBI.mergeCand[1]], false, false, true); // do again luma since OBMC has been applied
      }

      int geoMergeIdx0 = geoBI.mergeCand[0];
      int geoMergeIdx1 = geoBI.mergeCand[1];
      pu.gpmDmvrRefinePart0 = refinePossible[geoMergeIdx0];
      pu.gpmDmvrRefinePart1 = refinePossible[geoMergeIdx1];
      if (pu.geoTmFlag0 || pu.geoMMVDFlag0 || pu.geoTmFlag1 || pu.geoMMVDFlag1)
      {
        printf("spanGeoMMVDMotionInfo() failed");
        exit(0);
      }

      uint8_t intraMPM[2] = { PLANAR_IDX, PLANAR_IDX };
#if JVET_AG0164_AFFINE_GPM
      PU::spanGeoMMVDMotionInfo(pu, mergeCtx[GEO_TM_OFF], affMergeCtx, mergeCtx[GEO_TM_OFF], mergeCtx[GEO_TM_OFF], 0, geoMergeIdx0, geoMergeIdx1, pu.geoTmFlag0, pu.geoMMVDFlag0, pu.geoMMVDIdx0, pu.geoTmFlag1, pu.geoMMVDFlag1, pu.geoMMVDIdx1, 0, intraMPM,
        pu.gpmDmvrRefinePart0, pu.gpmDmvrRefinePart1, pu.gpmDmvrRefinePart0 ? m_mvBufEncBDOF4GPM[GEO_TM_OFF][geoMergeIdx0] : nullptr, pu.gpmDmvrRefinePart1 ? m_mvBufEncBDOF4GPM[GEO_TM_OFF][geoMergeIdx1] : nullptr);
#else
      PU::spanGeoMMVDMotionInfo(pu, mergeCtx[GEO_TM_OFF], mergeCtx[GEO_TM_OFF], mergeCtx[GEO_TM_OFF], 0, geoMergeIdx0, geoMergeIdx1, pu.geoTmFlag0, pu.geoMMVDFlag0, pu.geoMMVDIdx0, pu.geoTmFlag1, pu.geoMMVDFlag1, pu.geoMMVDIdx1, 0, intraMPM,
        pu.gpmDmvrRefinePart0, pu.gpmDmvrRefinePart1, pu.gpmDmvrRefinePart0 ? m_mvBufEncBDOF4GPM[GEO_TM_OFF][geoMergeIdx0] : nullptr, pu.gpmDmvrRefinePart1 ? m_mvBufEncBDOF4GPM[GEO_TM_OFF][geoMergeIdx1] : nullptr);
#endif

      tempCS->getPredBuf().copyFrom(blendBuffer);

#if ENABLE_OBMC
      if (!pu.gpmIntraFlag)
      {
        cu.isobmcMC = true;
        cu.obmcFlag = true;
        m_pcInterSearch->subBlockOBMC(pu);
        cu.isobmcMC = false;
      }
#endif

      pu.mergeIdx = MAX_UCHAR;
      pu.geoMergeIdx0 = geoBI.iMergeIdx;
      pu.geoMergeIdx1 = MAX_UCHAR;
      CHECK(i >= pu.cs->sps->getMaxNumGeoCand(), "geoBlend idx should be < sps->getMaxNumGeoCand()");

      double lastBestCost = bestCS->cost;

      xEncodeInterResidual(tempCS, bestCS, pm, encTestMode, noResidualPass, (noResidualPass == 0 ? &geoBlendCandHasNoResidual : NULL));

      if (m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip)
      {
        bestIsSkip = bestCS->getCU(pm.chType)->rootCbf == 0;
      }
      tempCS->initStructData(encTestMode.qp);

      bIsBestCost |= lastBestCost > bestCS->cost;
      if (!bIsBestCost && (i + 1) >= 2)
      {
        bStopRdoLoop = true;
      }
    } // noResidualPass
  } // i (numGeoBlendInfoCand)
#endif

  if (m_bestModeUpdated && bestCS->cost != MAX_DOUBLE)
  {
    xCalDebCost(*bestCS, pm);
  }
}
#else
void EncCu::xCheckRDCostMergeGeo2Nx2N(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &pm, const EncTestMode& encTestMode)
{
  const Slice &slice = *tempCS->slice;
  CHECK(slice.getSliceType() == I_SLICE, "Merge modes not available for I-slices");

  tempCS->initStructData(encTestMode.qp);

  MergeCtx mergeCtx;
  const SPS &sps = *tempCS->sps;

  if (sps.getSbTMVPEnabledFlag())
  {
    Size bufSize = g_miScaling.scale(tempCS->area.lumaSize());
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
    for (int i = 0; i < SUB_TMVP_NUM; i++)
    {
      mergeCtx.subPuMvpMiBuf[i] = MotionBuf(m_subPuMiBuf[i], bufSize);
    }
#else
    mergeCtx.subPuMvpMiBuf = MotionBuf(m_subPuMiBuf, bufSize);
#endif
  }

  CodingUnit &cu = tempCS->addCU(tempCS->area, pm.chType);
  pm.setCUData(cu);
  cu.predMode = MODE_INTER;
  cu.slice = tempCS->slice;
  cu.tileIdx = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
  cu.qp = encTestMode.qp;
  cu.affine = false;
  cu.mtsFlag = false;
#if INTER_LIC
  cu.licFlag = false;
#endif
  cu.bcwIdx = BCW_DEFAULT;
  cu.geoFlag = true;
  cu.imv = 0;
  cu.mmvdSkip = false;
  cu.skip = false;
  cu.mipFlag = false;
#if JVET_V0130_INTRA_TMP
  cu.tmpFlag = false;
#endif
  cu.bdpcmMode = 0;
#if JVET_AG0058_EIP
  cu.eipFlag = false;
#endif

  PredictionUnit &pu = tempCS->addPU(cu, pm.chType);
  pu.mergeFlag = true;
  pu.regularMergeFlag = false;
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
  pu.tmMergeFlag = false;
#endif
  PU::getGeoMergeCandidates(pu, mergeCtx);

  GeoComboCostList comboList;
  int bitsCandTB = floorLog2(GEO_NUM_PARTITION_MODE);
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  double geoSigModeBits[GEO_NUM_SIG_PARTMODE];
  if(sps.getUseAltGPMSplitModeCode())
  {
    const int maxNumBins = (GEO_NUM_SIG_PARTMODE / GEO_SPLIT_MODE_RICE_CODE_DIVISOR) - 1;
    for (int idx = 0; idx < GEO_NUM_SIG_PARTMODE; idx++)
    {
      int geoModePrefix = idx / GEO_SPLIT_MODE_RICE_CODE_DIVISOR;
      geoSigModeBits[idx] = geoModePrefix + (geoModePrefix == maxNumBins ? 0 : 1)
                          + (GEO_SPLIT_MODE_RICE_CODE_DIVISOR > 1 ? floorLog2(GEO_SPLIT_MODE_RICE_CODE_DIVISOR): 0);
    }
  }
#endif
  PelUnitBuf geoBuffer[GEO_MAX_NUM_UNI_CANDS];
  PelUnitBuf geoTempBuf[GEO_MAX_NUM_UNI_CANDS];
  PelUnitBuf geoCombinations[GEO_MAX_TRY_WEIGHTED_SAD];
  DistParam distParam;

  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));
  const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda();
  uint8_t maxNumMergeCandidates = cu.cs->sps->getMaxNumGeoCand();
  DistParam distParamWholeBlk;
  m_pcRdCost->setDistParam(distParamWholeBlk, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y().buf, m_acMergeBuffer[0].Y().stride, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y);
  Distortion bestWholeBlkSad = MAX_UINT64;
  double bestWholeBlkCost = MAX_DOUBLE;

  Distortion sadWholeBlk[GEO_MAX_NUM_UNI_CANDS];
  int pocMrg[GEO_MAX_NUM_UNI_CANDS];
  Mv MrgMv[GEO_MAX_NUM_UNI_CANDS];
  bool isSkipThisCand[GEO_MAX_NUM_UNI_CANDS] = { false };

  for (uint8_t mergeCand = 0; mergeCand < maxNumMergeCandidates; mergeCand++)
  {
    geoBuffer[mergeCand] = m_acMergeBuffer[mergeCand].getBuf(localUnitArea);
    mergeCtx.setMergeInfo(pu, mergeCand);
    int MrgList = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + 0].refIdx == -1 ? 1 : 0;
    RefPicList MrgeRefPicList = (MrgList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
    int MrgrefIdx = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + MrgList].refIdx;
    pocMrg[mergeCand] = tempCS->slice->getRefPic(MrgeRefPicList, MrgrefIdx)->getPOC();
    MrgMv[mergeCand] = mergeCtx.mvFieldNeighbours[(mergeCand << 1) + MrgList].mv;

    for( int i = 0; i < mergeCand; i++ )
    {
      if( pocMrg[mergeCand] == pocMrg[i] && MrgMv[mergeCand] == MrgMv[i] )
      {
        isSkipThisCand[mergeCand] = true;
        break;
      }
    }

    if (m_pcEncCfg->getMCTSEncConstraint() && (!(MCTSHelper::checkMvBufferForMCTSConstraint(pu))))
    {
      tempCS->initStructData(encTestMode.qp);
      return;
    }
    m_pcInterSearch->motionCompensation(pu, geoBuffer[mergeCand]);
#if MULTI_HYP_PRED
    geoTempBuf[mergeCand] = m_acRealMergeBuffer[MRG_MAX_NUM_CANDS + mergeCand].getBuf(localUnitArea);
#else
    geoTempBuf[mergeCand] = m_acMergeTmpBuffer[mergeCand].getBuf(localUnitArea);
#endif
    geoTempBuf[mergeCand].Y().copyFrom(geoBuffer[mergeCand].Y());
    geoTempBuf[mergeCand].Y().roundToOutputBitdepth(geoTempBuf[mergeCand].Y(), cu.slice->clpRng(COMPONENT_Y));
    distParamWholeBlk.cur.buf = geoTempBuf[mergeCand].Y().buf;
    distParamWholeBlk.cur.stride = geoTempBuf[mergeCand].Y().stride;
    sadWholeBlk[mergeCand] = distParamWholeBlk.distFunc(distParamWholeBlk);
    if (sadWholeBlk[mergeCand] < bestWholeBlkSad)
    {
      bestWholeBlkSad = sadWholeBlk[mergeCand];
      bestWholeBlkCost = ( double ) bestWholeBlkSad + ( mergeCand + 1 ) * sqrtLambdaForFirstPass;
    }
  }
#if MULTI_HYP_PRED
#if JVET_AD0213_LIC_IMP
  m_pcInterSearch->setGeoTmpBuffer(mergeCtx, 1);
#else
  m_pcInterSearch->setGeoTmpBuffer(mergeCtx);
#endif
#endif
  bool isGeo = true;
  for (uint8_t mergeCand = 1; mergeCand < maxNumMergeCandidates; mergeCand++)
  {
    isGeo &= isSkipThisCand[mergeCand];
  }
  if (isGeo)
  {
    return;
  }

  int wIdx = floorLog2(cu.lwidth()) - GEO_MIN_CU_LOG2;
  int hIdx = floorLog2(cu.lheight()) - GEO_MIN_CU_LOG2;
  for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
  {
    int maskStride = 0, maskStride2 = 0;
    int stepX = 1;
    Pel* sadMask;
    int16_t angle = g_geoParams[splitDir][0];
    if (g_angle2mirror[angle] == 2)
    {
      maskStride = -GEO_WEIGHT_MASK_SIZE;
      maskStride2 = -(int)cu.lwidth();
      sadMask = &g_geoEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
    }
    else if (g_angle2mirror[angle] == 1)
    {
      stepX = -1;
      maskStride2 = cu.lwidth();
      maskStride = GEO_WEIGHT_MASK_SIZE;
      sadMask = &g_geoEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
    }
    else
    {
      maskStride = GEO_WEIGHT_MASK_SIZE;
      maskStride2 = -(int)cu.lwidth();
      sadMask = &g_geoEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
    }
    Distortion sadSmall = 0, sadLarge = 0;
    for (uint8_t mergeCand = 0; mergeCand < maxNumMergeCandidates; mergeCand++)
    {
      int bitsCand = mergeCand + 1;

      m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), geoTempBuf[mergeCand].Y().buf, geoTempBuf[mergeCand].Y().stride, sadMask, maskStride, stepX, maskStride2, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y);
      sadLarge = distParam.distFunc(distParam);
      m_GeoCostList.insert(splitDir, 0, mergeCand, (double)sadLarge + (double)bitsCand * sqrtLambdaForFirstPass);
      sadSmall = sadWholeBlk[mergeCand] - sadLarge;
      m_GeoCostList.insert(splitDir, 1, mergeCand, (double)sadSmall + (double)bitsCand * sqrtLambdaForFirstPass);
    }
  }

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  if (sps.getUseAltGPMSplitModeCode())
  {
    m_pcInterSearch->initGeoAngleSelection(pu
#if JVET_Y0065_GPM_INTRA
                                         , m_pcIntraSearch, geoIntraMPMList
#endif
    );
  }
#endif

  for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
  {
    for (int GeoMotionIdx = 0; GeoMotionIdx < maxNumMergeCandidates * (maxNumMergeCandidates - 1); GeoMotionIdx++)
    {
      unsigned int mergeCand0 = m_GeoModeTest[GeoMotionIdx].m_candIdx0;
      unsigned int mergeCand1 = m_GeoModeTest[GeoMotionIdx].m_candIdx1;

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
      int geoSyntaxMode = std::numeric_limits<uint8_t>::max();
      if (sps.getUseAltGPMSplitModeCode())
      {
        m_pcInterSearch->setGeoSplitModeToSyntaxTable(pu, mergeCtx, mergeCand0, mergeCtx, mergeCand1
#if JVET_Y0065_GPM_INTRA
                                                    , m_pcIntraSearch
#endif
        );
        geoSyntaxMode = m_pcInterSearch->convertGeoSplitModeToSyntax(splitDir, mergeCand0, mergeCand1);
        if (geoSyntaxMode == std::numeric_limits<uint8_t>::max())
        {
          continue;
        }
      }
#endif

      double tempCost = m_GeoCostList.singleDistList[0][splitDir][mergeCand0].cost + m_GeoCostList.singleDistList[1][splitDir][mergeCand1].cost;
      if( tempCost > bestWholeBlkCost )
      {
        continue;
      }
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
      tempCost = tempCost + (double)(sps.getUseAltGPMSplitModeCode() ? geoSigModeBits[geoSyntaxMode] : bitsCandTB) * sqrtLambdaForFirstPass;
#else
      tempCost = tempCost + (double)bitsCandTB * sqrtLambdaForFirstPass;
#endif
      comboList.list.push_back(GeoMergeCombo(splitDir, mergeCand0, mergeCand1, tempCost));
    }
  }
  if( comboList.list.empty() )
  {
    return;
  }

  comboList.sortByCost();

  bool geocandHasNoResidual[GEO_MAX_TRY_WEIGHTED_SAD] = { false };
  bool bestIsSkip = false;
  int geoNumCobo = (int)comboList.list.size();
  static_vector<uint8_t, GEO_MAX_TRY_WEIGHTED_SAD> geoRdModeList;
  static_vector<double, GEO_MAX_TRY_WEIGHTED_SAD> geocandCostList;

  DistParam distParamSAD2;
  const bool useHadamard = !tempCS->slice->getDisableSATDForRD();
  m_pcRdCost->setDistParam(distParamSAD2, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y(), sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, useHadamard);
  int geoNumMrgSATDCand = min(GEO_MAX_TRY_WEIGHTED_SATD, geoNumCobo);

  for (uint8_t candidateIdx = 0; candidateIdx < min(geoNumCobo, GEO_MAX_TRY_WEIGHTED_SAD); candidateIdx++)
  {
    int splitDir = comboList.list[candidateIdx].splitDir;
    int mergeCand0 = comboList.list[candidateIdx].mergeIdx0;
    int mergeCand1 = comboList.list[candidateIdx].mergeIdx1;

    geoCombinations[candidateIdx] = m_acGeoWeightedBuffer[candidateIdx].getBuf(localUnitArea);
    m_pcInterSearch->weightedGeoBlk(pu, splitDir, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx], geoBuffer[mergeCand0], geoBuffer[mergeCand1]);
    distParamSAD2.cur = geoCombinations[candidateIdx].Y();
    Distortion sad = distParamSAD2.distFunc(distParamSAD2);
    int mvBits = 2;
    mergeCand1 -= mergeCand1 < mergeCand0 ? 0 : 1;
    mvBits += mergeCand0;
    mvBits += mergeCand1;
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
    int geoSyntaxMode = std::numeric_limits<uint8_t>::max();
    if (sps.getUseAltGPMSplitModeCode())
    {
      geoSyntaxMode = m_pcInterSearch->convertGeoSplitModeToSyntax(splitDir, mergeCand0, mergeCand1);
      CHECK(geoSyntaxMode < 0 || geoSyntaxMode >= GEO_NUM_SIG_PARTMODE, "Invalid GEO split direction!");
    }
#endif
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
    double updateCost = (double)sad + (double)((sps.getUseAltGPMSplitModeCode() ? geoSigModeBits[geoSyntaxMode] : bitsCandTB) + mvBits) * sqrtLambdaForFirstPass;
#else
    double updateCost = (double)sad + (double)(bitsCandTB + mvBits) * sqrtLambdaForFirstPass;
#endif
    comboList.list[candidateIdx].cost = updateCost;
    updateCandList(candidateIdx, updateCost, geoRdModeList, geocandCostList, geoNumMrgSATDCand);
  }
  for (uint8_t i = 0; i < geoNumMrgSATDCand; i++)
  {
    if (geocandCostList[i] > MRG_FAST_RATIO * geocandCostList[0] || geocandCostList[i] > getMergeBestSATDCost() || geocandCostList[i] > getAFFBestSATDCost())
    {
      geoNumMrgSATDCand = i;
      break;
    }
  }
  for (uint8_t i = 0; i < geoNumMrgSATDCand && isChromaEnabled(pu.chromaFormat); i++)
  {
    uint8_t candidateIdx = geoRdModeList[i];
    int splitDir = comboList.list[candidateIdx].splitDir;
    int mergeCand0 = comboList.list[candidateIdx].mergeIdx0;
    int mergeCand1 = comboList.list[candidateIdx].mergeIdx1;
    geoCombinations[candidateIdx] = m_acGeoWeightedBuffer[candidateIdx].getBuf(localUnitArea);
    m_pcInterSearch->weightedGeoBlk(pu, splitDir, CHANNEL_TYPE_CHROMA, geoCombinations[candidateIdx], geoBuffer[mergeCand0], geoBuffer[mergeCand1]);
  }

  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;
  tempCS->initStructData(encTestMode.qp);
  uint8_t iteration;
  uint8_t iterationBegin = 0;
  iteration = 2;
  for (uint8_t noResidualPass = iterationBegin; noResidualPass < iteration; ++noResidualPass)
  {
    for (uint8_t mrgHADIdx = 0; mrgHADIdx < geoNumMrgSATDCand; mrgHADIdx++)
    {
      uint8_t candidateIdx = geoRdModeList[mrgHADIdx];
      if (((noResidualPass != 0) && geocandHasNoResidual[candidateIdx])
        || ((noResidualPass == 0) && bestIsSkip))
      {
        continue;
      }
      CodingUnit &cu = tempCS->addCU(tempCS->area, pm.chType);
      pm.setCUData(cu);
      cu.predMode = MODE_INTER;
      cu.slice = tempCS->slice;
      cu.tileIdx = tempCS->pps->getTileIdx(tempCS->area.lumaPos());
      cu.qp = encTestMode.qp;
      cu.affine = false;
      cu.mtsFlag = false;
#if INTER_LIC
      cu.licFlag = false;
#endif
      cu.bcwIdx = BCW_DEFAULT;
      cu.geoFlag = true;
      cu.imv = 0;
      cu.mmvdSkip = false;
      cu.skip = false;
      cu.mipFlag = false;
#if JVET_V0130_INTRA_TMP
	    cu.tmpFlag = false;
#endif
      cu.bdpcmMode = 0;
#if JVET_AG0058_EIP
      cu.eipFlag = false;
#endif
      PredictionUnit &pu = tempCS->addPU(cu, pm.chType);
      pu.mergeFlag = true;
      pu.regularMergeFlag = false;
      pu.geoSplitDir = comboList.list[candidateIdx].splitDir;
      pu.geoMergeIdx0 = comboList.list[candidateIdx].mergeIdx0;
      pu.geoMergeIdx1 = comboList.list[candidateIdx].mergeIdx1;
      pu.mmvdMergeFlag = false;
      pu.mmvdMergeIdx = MAX_UCHAR;
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
      if (sps.getUseAltGPMSplitModeCode())
      {
        int geoSyntaxMode = m_pcInterSearch->convertGeoSplitModeToSyntax(pu.geoSplitDir, pu.geoMergeIdx0, pu.geoMergeIdx1);
        CHECK(geoSyntaxMode < 0 || geoSyntaxMode >= GEO_NUM_SIG_PARTMODE, "Invalid GEO split direction!");
        pu.geoSyntaxMode = (uint8_t)geoSyntaxMode;
      }
#endif

      PU::spanGeoMotionInfo(pu, mergeCtx, pu.geoSplitDir, pu.geoMergeIdx0, pu.geoMergeIdx1);
      tempCS->getPredBuf().copyFrom(geoCombinations[candidateIdx]);
#if ENABLE_OBMC
      cu.isobmcMC = true;
      cu.obmcFlag = true;
      m_pcInterSearch->subBlockOBMC(pu);
      cu.isobmcMC = false;
#endif
      xEncodeInterResidual(tempCS, bestCS, pm, encTestMode, noResidualPass, (noResidualPass == 0 ? &geocandHasNoResidual[candidateIdx] : NULL));

      if (m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip)
      {
        bestIsSkip = bestCS->getCU(pm.chType)->rootCbf == 0;
      }
      tempCS->initStructData(encTestMode.qp);
    }
  }
  if (m_bestModeUpdated && bestCS->cost != MAX_DOUBLE)
  {
    xCalDebCost(*bestCS, pm);
  }
}
#endif
#if MERGE_ENC_OPT
void EncCu::xCheckSATDCostRegularMerge(CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, MergeCtx mergeCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer, PelUnitBuf  acMergeTmpBuffer[MRG_MAX_NUM_CANDS]
#if !MULTI_PASS_DMVR
  , Mv   refinedMvdL0[MAX_NUM_PARTS_IN_CTU][MRG_MAX_NUM_CANDS]
#endif
  , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart
#if MULTI_PASS_DMVR
  , bool* applyBDMVR
#endif
#if JVET_AF0057
  , bool* dmvrImpreciseMv
#endif

)
{
#if JVET_AG0276_NLIC
  cu.altLMFlag = false;
  cu.altLMParaUnit.resetAltLinearModel();
#endif
#if INTER_LIC
  cu.licFlag = false;
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
  pu.mergeOppositeLic = false;
#endif
  cu.mmvdSkip = false;
  cu.geoFlag = false;
  cu.affine = false;
  cu.imv = 0;

  pu.ciipFlag = false;
#if CIIP_PDPC
  pu.ciipPDPC = false;
#endif
#if JVET_AG0135_AFFINE_CIIP
  pu.ciipAffine = false;
#endif
  pu.mmvdMergeFlag = false;
  pu.regularMergeFlag = true;

  const double sqrtLambdaForFirstPassIntra = m_pcRdCost->getMotionLambda() * FRAC_BITS_SCALE;
  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));
#if MULTI_HYP_PRED
  const bool testMHP = tempCS->sps->getUseInterMultiHyp()
    && (tempCS->area.lumaSize().area() > MULTI_HYP_PRED_RESTRICT_BLOCK_SIZE 
    && std::min(tempCS->area.lwidth(), tempCS->area.lheight()) >= MULTI_HYP_PRED_RESTRICT_MIN_WH);
#endif

  int insertPos = -1;
  for (uint32_t uiMergeCand = 0; uiMergeCand < mergeCtx.numValidMergeCand; uiMergeCand++)
  {
    mergeCtx.setMergeInfo(pu, uiMergeCand);
#if MULTI_PASS_DMVR
    pu.bdmvrRefine = false; // init as false
#endif
    pu.mvRefine = true;
    distParam.cur = singleMergeTempBuffer->Y();
    acMergeTmpBuffer[uiMergeCand] = m_acMergeTmpBuffer[uiMergeCand].getBuf(localUnitArea);
#if JVET_AG0276_NLIC
    m_pcInterSearch->m_storeBeforeLIC = false;
#else
#if INTER_LIC
#if JVET_AD0213_LIC_IMP
    m_pcInterSearch->m_storeBeforeLIC = mergeCtx.licFlags[uiMergeCand] ? true : false;
#else
    m_pcInterSearch->m_storeBeforeLIC = mergeCtx.interDirNeighbours[uiMergeCand] == 3 ? false : true;
#endif
#if JVET_AG0276_LIC_BDOF_BDMVR
    m_pcInterSearch->m_storeBeforeLIC &= (applyBDMVR[uiMergeCand] == false);
#endif
    if (m_pcInterSearch->m_storeBeforeLIC)
    {
      m_pcInterSearch->m_predictionBeforeLIC = acMergeTmpBuffer[uiMergeCand];
      m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X, true, true);
    }
    else
#endif
#endif
#if MULTI_PASS_DMVR
    if (applyBDMVR[uiMergeCand])
    {
#if !JVET_AG0276_NLIC
      if (pu.cu->cs->sps->getUseCiip())
      {
#if MULTI_HYP_PRED
        pu.addHypData.clear();
        pu.numMergedAddHyps = 0;
#endif
        pu.mvRefine = false;
        pu.ciipFlag = true;
        m_pcInterSearch->motionCompensation(pu, acMergeTmpBuffer[uiMergeCand]);
        pu.ciipFlag = false;
#if MULTI_HYP_PRED
        mergeCtx.setMergeInfo(pu, uiMergeCand);
#endif
      }
#endif
      pu.bdmvrRefine = true;
      m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR[uiMergeCand << 1], m_mvBufBDMVR[(uiMergeCand << 1) + 1]);

      pu.mvRefine = true;
      m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer);

      if( pu.bdmvrRefine )
      {
#if JVET_AG0276_LIC_BDOF_BDMVR
        if (pu.cu->licFlag == true)
        {
          Mv* bdofMvBuf = m_pcInterSearch->getBdofSubPuMvOffset();
          for (int z = 0; z < BDOF_SUBPU_MAX_NUM; z++)
          {
            bdofMvBuf[z].setZero();
          }
        }
#endif
        ::memcpy( m_mvBufEncBDOF[uiMergeCand], m_pcInterSearch->getBdofSubPuMvOffset(), sizeof( Mv ) * BDOF_SUBPU_MAX_NUM );
      }

      pu.mvRefine = false;
    }
    else
#endif
    {
#if JVET_AG0276_NLIC
      m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer);
#else
      m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X, true, true, &(acMergeTmpBuffer[uiMergeCand]));
#endif
    }
#if INTER_LIC
    m_pcInterSearch->m_storeBeforeLIC = false;
#endif
    pu.mvRefine = false;
#if !MULTI_PASS_DMVR
    if (mergeCtx.interDirNeighbours[uiMergeCand] == 3 )
    {
      mergeCtx.mvFieldNeighbours[2 * uiMergeCand].mv = pu.mv[0];
      mergeCtx.mvFieldNeighbours[2 * uiMergeCand + 1].mv = pu.mv[1];
      {
        int dx, dy, i, j, num = 0;
        dy = std::min<int>(pu.lumaSize().height, DMVR_SUBCU_HEIGHT);
        dx = std::min<int>(pu.lumaSize().width, DMVR_SUBCU_WIDTH);
        if (PU::checkDMVRCondition(pu))
        {
          for (i = 0; i < (pu.lumaSize().height); i += dy)
          {
            for (j = 0; j < (pu.lumaSize().width); j += dx)
            {
              refinedMvdL0[num][uiMergeCand] = pu.mvdL0SubPu[num];
              num++;
            }
          }
        }
      }
    }
#endif

    Distortion uiSad = distParam.distFunc(distParam);
    m_CABACEstimator->getCtx() = ctxStart;
    uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
    double cost = (double)uiSad + (double)fracBits * sqrtLambdaForFirstPassIntra;

#if JVET_AF0057
      if (m_pcInterSearch->dmvrEnableEncoderCheck && dmvrImpreciseMv[uiMergeCand])
      {
        cost = MAX_DOUBLE;
      }
#endif

#if MULTI_HYP_PRED
    if (testMHP && pu.addHypData.size() < tempCS->sps->getMaxNumAddHyps())
    {
#if MULTI_PASS_DMVR
      CHECK(pu.bdmvrRefine && !applyBDMVR[uiMergeCand], "");
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
      uint32_t uiBitsCand = uiMergeCand + 1 + 1 + 1 + 1; // one bit for merge flag,  one bit for subblock_merge_flag, and one bit for regualr_merge_flag, one bit for mergeOppoLic flag
#else
      uint32_t uiBitsCand = uiMergeCand + 1 + 1 + 1; // one bit for merge flag,  one bit for subblock_merge_flag, and one bit for regualr_merge_flag
#endif
      MEResult mergeResult;
      mergeResult.cu = cu;
      mergeResult.pu = pu;
      mergeResult.bits = uiBitsCand;
      mergeResult.cost = uiSad + m_pcRdCost->getCost(uiBitsCand);
#if JVET_AF0057
        if (m_pcInterSearch->dmvrEnableEncoderCheck && dmvrImpreciseMv[uiMergeCand])
        {
          mergeResult.cost = (Distortion)MAX_UINT64;
        }
#endif
      m_baseResultsForMH.push_back(mergeResult);
    }
#endif
    insertPos = -1;
    updateCandList(ModeInfo(cu, pu), cost, rdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
    if (insertPos != -1 && insertPos < MMVD_MRG_MAX_RD_NUM)
    {
      if (insertPos == rdModeList.size() - 1)
      {
        swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
      }
      else
      {
        for (uint32_t i = uint32_t(rdModeList.size()) - 1; i > insertPos; i--)
        {
          swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
        }
        swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
      }
    }
    CHECK(std::min(uiMergeCand + 1, uiNumMrgSATDCand) != rdModeList.size(), "");
  }
#if MULTI_PASS_DMVR
  pu.bdmvrRefine = false;
#endif
}
#if JVET_AG0135_AFFINE_CIIP
void EncCu::xCheckSATDCostCiipAffineMerge(CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, AffineMergeCtx affineMergeCtx, MergeCtx mergeCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer, PelUnitBuf  acMergeCIIPAffineBuffer[AFFINE_MRG_MAX_NUM_CANDS]
  , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart)
{
#if INTER_LIC
  cu.licFlag = false;
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
  pu.affineOppositeLic = false;
#endif
  cu.mmvdSkip = false;
  cu.geoFlag = false;
  cu.affine = true;
  //cu.imv = 0;

  pu.ciipFlag = true;
#if CIIP_PDPC
  pu.ciipPDPC = false;
#endif
  pu.ciipAffine = true;
  pu.mmvdMergeFlag = false;
  pu.regularMergeFlag = false;
  const double sqrtLambdaForFirstPassIntra = m_pcRdCost->getMotionLambda() * FRAC_BITS_SCALE;
  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));

#if JVET_X0141_CIIP_TIMD_TM && JVET_W0123_TIMD_FUSION
  int intraMode = PLANAR_IDX;
  if (affineMergeCtx.numValidMergeCand)
  {
    const CompArea &area = cu.Y();
    if (cu.slice->getSPS()->getUseTimd() && (cu.lwidth() * cu.lheight() <= CIIP_MAX_SIZE) && cu.cs->slice->getSPS()->getUseCiipTimd())
    {
#if SECONDARY_MPM && ENABLE_DIMD
      IntraPrediction::deriveDimdMode(cu.cs->picture->getRecoBuf(area), area, cu);
#endif
      cu.timdMode = m_pcIntraSearch->deriveTimdMode(cu.cs->picture->getRecoBuf(area), area, cu);
      intraMode = MAP131TO67(cu.timdMode);
    }
  }
#endif

  int insertPos = -1;
  for (uint32_t uiAffMergeCand = 0; uiAffMergeCand < affineMergeCtx.numValidMergeCand; uiAffMergeCand++)
  {
    pu.mergeIdx = uiAffMergeCand;
    pu.interDir = affineMergeCtx.interDirNeighbours[uiAffMergeCand];
    cu.affineType = affineMergeCtx.affineType[uiAffMergeCand];
    cu.bcwIdx = affineMergeCtx.bcwIdx[uiAffMergeCand];
#if INTER_LIC
    cu.licFlag = affineMergeCtx.licFlags[uiAffMergeCand];
#endif
#if JVET_AG0276_NLIC
    cu.altLMFlag = affineMergeCtx.altLMFlag[uiAffMergeCand];
    cu.altLMParaUnit = affineMergeCtx.altLMParaNeighbours[uiAffMergeCand];
#endif
    pu.mv[0].setZero();
    pu.mv[1].setZero();
    cu.imv = 0;
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    pu.colIdx = affineMergeCtx.colIdx[uiAffMergeCand];
#endif
    pu.mergeType = affineMergeCtx.mergeType[uiAffMergeCand];
    if (pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
    {
      pu.refIdx[0] = affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].refIdx;
      pu.refIdx[1] = affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].refIdx;
      // the SbTmvp use xSubPuMC which will need to access the motion buffer for subblock MV
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
      cu.obmcFlag = true;
#endif
      PU::spanMotionInfo(pu, mergeCtx
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
        , pu.colIdx
#endif 
      );
    }
    else
    {
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
      cu.obmcFlag = affineMergeCtx.obmcFlags[uiAffMergeCand];
#endif
      PU::setAllAffineMvField(pu, affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0], REF_PIC_LIST_0);
      PU::setAllAffineMvField(pu, affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1], REF_PIC_LIST_1);
    }

    // first round
    pu.intraDir[0] = PLANAR_IDX;
#if CIIP_PDPC
    for (int intraCnt = 0; intraCnt < 1; intraCnt++)
    {
#else
    uint32_t intraCnt = 0;
#endif
#if CIIP_PDPC
    pu.ciipPDPC = intraCnt == 1;
#endif
    PelBuf ciipBuff = m_ciipBuffer[intraCnt].getBuf(localUnitArea.Y());

#if JVET_X0141_CIIP_TIMD_TM && JVET_W0123_TIMD_FUSION
    pu.intraDir[0] = (intraCnt == 1) ? PLANAR_IDX : intraMode;
#endif
    // generate intrainter Y prediction
    if (uiAffMergeCand == 0)
    {
      m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Y());
      m_pcIntraSearch->predIntraAng(COMPONENT_Y, ciipBuff, pu);
    }
    if (pu.cs->picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
    {
      m_pcIntraSearch->geneWeightedCIIPAffinePred<true>(COMPONENT_Y, singleMergeTempBuffer->Y(), pu, acMergeCIIPAffineBuffer[uiAffMergeCand].Y(), ciipBuff, m_pcReshape->getFwdLUT().data());
    }
    else
    {
      m_pcIntraSearch->geneWeightedCIIPAffinePred<false>(COMPONENT_Y, singleMergeTempBuffer->Y(), pu, acMergeCIIPAffineBuffer[uiAffMergeCand].Y(), ciipBuff);
    }
    // calculate cost
    if (pu.cs->picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
    {
      PelBuf tmp = m_acGeoWeightedBuffer->getBuf(localUnitArea.Y());
      tmp.rspSignal(singleMergeTempBuffer->Y(), m_pcReshape->getInvLUT());
      distParam.cur = tmp;
    }
    else
    {
      distParam.cur = singleMergeTempBuffer->Y();
    }

    Distortion sadValue = distParam.distFunc(distParam);
    m_CABACEstimator->getCtx() = ctxStart;
    pu.regularMergeFlag = false;
    uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
    double cost = (double)sadValue + (double)fracBits * sqrtLambdaForFirstPassIntra; // need to check the cost calculation again???
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
    m_pcInterSearch->setDoAffineSubPuBdof(false);
#endif
    insertPos = -1;
    updateCandList(ModeInfo(cu, pu), cost, rdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
    if (insertPos != -1 && insertPos < MMVD_MRG_MAX_RD_NUM)
    {
      for (int i = int(rdModeList.size()) - 1; i > insertPos; i--)
      {
        swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
      }
      swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
    }
#if CIIP_PDPC
    }
#endif
  }
  cu.affine = false;
  pu.ciipFlag = false;
#if CIIP_PDPC
  pu.ciipPDPC = false;
#endif
  pu.ciipAffine = false;
}
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
void EncCu::xCheckSATDCostRegularMergeOppositeLic(CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, MergeCtx mergeCtxOppositeLic, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer, PelUnitBuf  acMergeTmpBuffer[MRG_MAX_NUM_CANDS]
#if !MULTI_PASS_DMVR
  , Mv   refinedMvdL0[MAX_NUM_PARTS_IN_CTU][MRG_MAX_NUM_CANDS]
#endif
  , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart
#if MULTI_PASS_DMVR
  , bool* applyBDMVR
#endif
)
{
#if JVET_AG0276_NLIC
  cu.altLMFlag = false;
  cu.altLMParaUnit.resetAltLinearModel();
#endif
#if INTER_LIC
  cu.licFlag = false;
#endif
  pu.mergeOppositeLic = true;
  cu.mmvdSkip = false;
  cu.geoFlag = false;
  cu.affine = false;
  cu.imv = 0;

  pu.ciipFlag = false;
#if CIIP_PDPC
  pu.ciipPDPC = false;
#endif
  pu.mmvdMergeFlag = false;
  pu.regularMergeFlag = true;

  const double sqrtLambdaForFirstPassIntra = m_pcRdCost->getMotionLambda() * FRAC_BITS_SCALE;
  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));
#if MULTI_HYP_PRED
  const bool testMHP = tempCS->sps->getUseInterMultiHyp()
    && (tempCS->area.lumaSize().area() > MULTI_HYP_PRED_RESTRICT_BLOCK_SIZE
      && std::min(tempCS->area.lwidth(), tempCS->area.lheight()) >= MULTI_HYP_PRED_RESTRICT_MIN_WH);
#endif

  int insertPos = -1;
  for (uint32_t uiMergeCand = 0; uiMergeCand < mergeCtxOppositeLic.numValidMergeCand; uiMergeCand++)
  {
    mergeCtxOppositeLic.setMergeInfo(pu, uiMergeCand);
#if MULTI_PASS_DMVR
    pu.bdmvrRefine = false; // init as false
#endif
    pu.mvRefine = true;
    distParam.cur = singleMergeTempBuffer->Y();
#if MULTI_PASS_DMVR
    if (applyBDMVR[uiMergeCand])
    {
      pu.bdmvrRefine = true;
      m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4OPPOSITELIC[uiMergeCand << 1], m_mvBufBDMVR4OPPOSITELIC[(uiMergeCand << 1) + 1]);
      
      pu.mvRefine = true;
      m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer);
      
      if (pu.bdmvrRefine)
      {
        ::memcpy(m_mvBufEncBDOF4OPPOSITELIC[uiMergeCand], m_pcInterSearch->getBdofSubPuMvOffset(), sizeof(Mv) * BDOF_SUBPU_MAX_NUM);
      }
      
      pu.mvRefine = false;
    }
    else
#endif
    {
      m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer);
    }
    pu.mvRefine = false;
#if !MULTI_PASS_DMVR
    if (mergeCtxOppositeLic.interDirNeighbours[uiMergeCand] == 3)
    {
      mergeCtxOppositeLic.mvFieldNeighbours[2 * uiMergeCand].mv = pu.mv[0];
      mergeCtxOppositeLic.mvFieldNeighbours[2 * uiMergeCand + 1].mv = pu.mv[1];
      {
        int dx, dy, i, j, num = 0;
        dy = std::min<int>(pu.lumaSize().height, DMVR_SUBCU_HEIGHT);
        dx = std::min<int>(pu.lumaSize().width, DMVR_SUBCU_WIDTH);
        if (PU::checkDMVRCondition(pu))
        {
          for (i = 0; i < (pu.lumaSize().height); i += dy)
          {
            for (j = 0; j < (pu.lumaSize().width); j += dx)
            {
              refinedMvdL0[num][uiMergeCand] = pu.mvdL0SubPu[num];
              num++;
            }
          }
        }
      }
    }
#endif
    
    Distortion uiSad = distParam.distFunc(distParam);
    m_CABACEstimator->getCtx() = ctxStart;
    uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
    double cost = (double)uiSad + (double)fracBits * sqrtLambdaForFirstPassIntra;
#if MULTI_HYP_PRED
    if (testMHP && pu.addHypData.size() < tempCS->sps->getMaxNumAddHyps())
    {
      uint32_t uiBitsCand = uiMergeCand + 1 + 1 + 1 + 1; // one bit for merge flag,  one bit for subblock_merge_flag, and one bit for regualr_merge_flag, one bit for mergeOppoLic flag
      MEResult mergeResult;
      mergeResult.cu = cu;
      mergeResult.pu = pu;
      mergeResult.bits = uiBitsCand;
      mergeResult.cost = uiSad + m_pcRdCost->getCost(uiBitsCand);
      m_baseResultsForMH.push_back(mergeResult);
    }
#endif
    insertPos = -1;
    updateCandList(ModeInfo(cu, pu), cost, rdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
    if (insertPos != -1 && insertPos < MMVD_MRG_MAX_RD_NUM)
    {
      if (insertPos == rdModeList.size() - 1)
      {
        swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
      }
      else
      {
        for (uint32_t i = uint32_t(rdModeList.size()) - 1; i > insertPos; i--)
        {
          swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
        }
        swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
      }
    }
  }
#if MULTI_PASS_DMVR
  pu.bdmvrRefine = false;
#endif
  pu.mergeOppositeLic = false;
}
#endif
void EncCu::xCheckSATDCostCiipMerge(CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, MergeCtx mergeCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer, PelUnitBuf  acMergeTmpBuffer[MRG_MAX_NUM_CANDS]
#if JVET_AG0276_NLIC
  , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart, MergeCtx mergeCtx1)
#else
  , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart)
#endif
{
#if JVET_AG0276_NLIC
  cu.altLMFlag = false;
  cu.altLMParaUnit.resetAltLinearModel();
#endif
#if INTER_LIC
  cu.licFlag = false;
#endif
  cu.mmvdSkip = false;
  cu.geoFlag = false;
  cu.affine = false;
  //cu.imv = 0;

  pu.ciipFlag = true;
#if CIIP_PDPC
  pu.ciipPDPC = false;
#endif
#if JVET_AG0135_AFFINE_CIIP
  pu.ciipAffine = false;
#endif
  pu.mmvdMergeFlag = false;
  pu.regularMergeFlag = false;

  const double sqrtLambdaForFirstPassIntra = m_pcRdCost->getMotionLambda() * FRAC_BITS_SCALE;
  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));

#if JVET_X0141_CIIP_TIMD_TM && JVET_W0123_TIMD_FUSION
  int intraMode = PLANAR_IDX;
  if (mergeCtx.numValidMergeCand)
  {
    const CompArea &area = cu.Y();
    if (cu.slice->getSPS()->getUseTimd() && (cu.lwidth() * cu.lheight() <= CIIP_MAX_SIZE) && cu.cs->slice->getSPS()->getUseCiipTimd())
    {
#if SECONDARY_MPM && ENABLE_DIMD
      IntraPrediction::deriveDimdMode(cu.cs->picture->getRecoBuf(area), area, cu);
#endif
      cu.timdMode = m_pcIntraSearch->deriveTimdMode(cu.cs->picture->getRecoBuf(area), area, cu);
      intraMode = MAP131TO67(cu.timdMode);
    }
  }
#endif

  int insertPos = -1;
  for (uint32_t mergeCand = 0; mergeCand < mergeCtx.numValidMergeCand; mergeCand++)
  {
    //acMergeTmpBuffer[mergeCand] = m_acMergeTmpBuffer[mergeCand].getBuf(localUnitArea);

    // estimate merge bits
#if JVET_AG0276_NLIC
    // lic flag is used to generate luma prediction samples for SATD RD to ensure the identical results as anchor; however, the correct solution is to remove such condition on lic flag
    if (mergeCtx1.licFlags[mergeCand])
    {
      pu.ciipFlag = false;
      mergeCtx1.setMergeInfo(pu, mergeCand);
      m_pcInterSearch->m_storeBeforeLIC = true;
      m_pcInterSearch->m_predictionBeforeLIC = acMergeTmpBuffer[mergeCand];
      m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X, true, false);
      m_pcInterSearch->m_storeBeforeLIC = false;
      pu.ciipFlag = true;
      mergeCtx.setMergeInfo(pu, mergeCand);
    }
    else
    {
#endif
    mergeCtx.setMergeInfo(pu, mergeCand);
#if JVET_AG0276_NLIC
    m_pcInterSearch->motionCompensation(pu, acMergeTmpBuffer[mergeCand], REF_PIC_LIST_X, true, false);
    }
#endif

    // first round
    pu.intraDir[0] = PLANAR_IDX;
#if CIIP_PDPC
    for (int intraCnt = 0; intraCnt < 2; intraCnt++)
    {
#else
    uint32_t intraCnt = 0;
#endif
#if CIIP_PDPC
    pu.ciipPDPC = intraCnt == 1;
#endif
    PelBuf ciipBuff = m_ciipBuffer[intraCnt].getBuf(localUnitArea.Y());

#if JVET_X0141_CIIP_TIMD_TM && JVET_W0123_TIMD_FUSION
    pu.intraDir[0] = (intraCnt == 1) ? PLANAR_IDX : intraMode;
#endif
    // generate intrainter Y prediction
    if (mergeCand == 0)
    {
      m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Y());
      m_pcIntraSearch->predIntraAng(COMPONENT_Y, ciipBuff, pu);
    }
    if (pu.cs->picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
    {
      m_pcIntraSearch->geneWeightedPred<true>(COMPONENT_Y, singleMergeTempBuffer->Y(), pu, acMergeTmpBuffer[mergeCand].Y(), ciipBuff, m_pcReshape->getFwdLUT().data());
    }
    else
    {
      m_pcIntraSearch->geneWeightedPred<false>( COMPONENT_Y, singleMergeTempBuffer->Y(), pu, acMergeTmpBuffer[mergeCand].Y(), ciipBuff );
    }

    // calculate cost
    if (pu.cs->picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
    {
      PelBuf tmp = m_acGeoWeightedBuffer->getBuf(localUnitArea.Y());
      tmp.rspSignal(singleMergeTempBuffer->Y(), m_pcReshape->getInvLUT());
      distParam.cur = tmp;
    }
    else
    {
      distParam.cur = singleMergeTempBuffer->Y();
    }

    Distortion sadValue = distParam.distFunc(distParam);
    m_CABACEstimator->getCtx() = ctxStart;
    pu.regularMergeFlag = false;
    uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
    double cost = (double)sadValue + (double)fracBits * sqrtLambdaForFirstPassIntra; // need to check the cost calculation again???
    insertPos = -1;
    updateCandList(ModeInfo(cu, pu), cost, rdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
    if (insertPos != -1 && insertPos < MMVD_MRG_MAX_RD_NUM)
    {
      for (int i = int(rdModeList.size()) - 1; i > insertPos; i--)
      {
        swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
      }
      swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
    }
#if CIIP_PDPC
  }
#endif
  }
pu.ciipFlag = false;
#if CIIP_PDPC
pu.ciipPDPC = false;
#endif
#if JVET_AG0135_AFFINE_CIIP
pu.ciipAffine = false;
#endif
}

#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
void EncCu::xCheckSATDCostCiipTmMerge(CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, MergeCtx mergeCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer, PelUnitBuf  acTmMergeTmpBuffer[MRG_MAX_NUM_CANDS]
  , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart)
{
#if JVET_AG0276_NLIC
  cu.altLMFlag = false;
  cu.altLMParaUnit.resetAltLinearModel();
#endif
#if INTER_LIC
  cu.licFlag = false;
#endif
  cu.mmvdSkip = false;
  cu.geoFlag = false;
  cu.affine = false;
  //cu.imv = 0;

  pu.ciipFlag = true;
#if CIIP_PDPC
  pu.ciipPDPC = false;
#endif
#if JVET_AG0135_AFFINE_CIIP
  pu.ciipAffine = false;
#endif
  pu.mmvdMergeFlag = false;
  pu.regularMergeFlag = false;
  pu.tmMergeFlag = true;

  const double sqrtLambdaForFirstPassIntra = m_pcRdCost->getMotionLambda() * FRAC_BITS_SCALE;
  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));

  int intraMode = PLANAR_IDX;
#if JVET_W0123_TIMD_FUSION
  if (mergeCtx.numValidMergeCand)
  {
    const CompArea &area = cu.Y();
    if (cu.slice->getSPS()->getUseTimd() && (cu.lwidth() * cu.lheight() <= CIIP_MAX_SIZE))
    {
#if SECONDARY_MPM && ENABLE_DIMD
      IntraPrediction::deriveDimdMode(cu.cs->picture->getRecoBuf(area), area, cu);
#endif
      cu.timdMode = m_pcIntraSearch->deriveTimdMode(cu.cs->picture->getRecoBuf(area), area, cu);
      intraMode = MAP131TO67(cu.timdMode);
    }
  }
#endif

  int insertPos = -1;
  for (uint32_t mergeCand = 0; mergeCand < mergeCtx.numValidMergeCand; mergeCand++)
  {
    //acTmMergeTmpBuffer[mergeCand] = m_acTmMergeTmpBuffer[mergeCand].getBuf(localUnitArea);
    // estimate merge bits
    mergeCtx.setMergeInfo(pu, mergeCand);

#if MULTI_HYP_PRED
    pu.addHypData.clear();
    pu.numMergedAddHyps = 0;
#endif
    acTmMergeTmpBuffer[mergeCand] = m_acTmMergeTmpBuffer[mergeCand].getBuf(localUnitArea);
#if JVET_AG0276_NLIC
    m_pcInterSearch->motionCompensation(pu, acTmMergeTmpBuffer[mergeCand], REF_PIC_LIST_X, true, false);
#else
    m_pcInterSearch->motionCompensation(pu, acTmMergeTmpBuffer[mergeCand]);
#endif

    // first round
    pu.intraDir[0] = PLANAR_IDX;
#if CIIP_PDPC
    for (int intraCnt = 0; intraCnt < 2; intraCnt++)
    {
#else
    uint32_t intraCnt = 0;
#endif
#if CIIP_PDPC
    pu.ciipPDPC = intraCnt == 1;
#endif
    PelBuf ciipBuff = m_ciipBuffer[intraCnt].getBuf(localUnitArea.Y());
    pu.intraDir[0] = (intraCnt == 1) ? PLANAR_IDX : intraMode;

    // generate intrainter Y prediction
    if (mergeCand == 0)
    {
      m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Y());
      m_pcIntraSearch->predIntraAng(COMPONENT_Y, ciipBuff, pu);
    }

    if (pu.cs->picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
    {
      m_pcIntraSearch->geneWeightedPred<true>(COMPONENT_Y, singleMergeTempBuffer->Y(), pu, acTmMergeTmpBuffer[mergeCand].Y(), ciipBuff, m_pcReshape->getFwdLUT().data());
    }
    else
    {
      m_pcIntraSearch->geneWeightedPred<false>(COMPONENT_Y, singleMergeTempBuffer->Y(), pu, acTmMergeTmpBuffer[mergeCand].Y(), ciipBuff);
    }

    // calculate cost
    if (pu.cs->picHeader->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
    {
      PelBuf tmp = m_acGeoWeightedBuffer->getBuf(localUnitArea.Y());
      tmp.rspSignal(singleMergeTempBuffer->Y(), m_pcReshape->getInvLUT());
      distParam.cur = tmp;
    }
    else
    {
      distParam.cur = singleMergeTempBuffer->Y();
    }

    Distortion sadValue = distParam.distFunc(distParam);
    m_CABACEstimator->getCtx() = ctxStart;
    pu.regularMergeFlag = false;
    uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
    double cost = (double)sadValue + (double)fracBits * sqrtLambdaForFirstPassIntra; // need to check the cost calculation again???
    insertPos = -1;
    updateCandList(ModeInfo(cu, pu), cost, rdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
    if (insertPos != -1 && insertPos < MMVD_MRG_MAX_RD_NUM)
    {
      for (int i = int(rdModeList.size()) - 1; i > insertPos; i--)
      {
        swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
      }
      swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
    }
#if CIIP_PDPC
    }
#endif
  }
  pu.ciipFlag = false;
#if CIIP_PDPC
  pu.ciipPDPC = false;
#endif
#if JVET_AG0135_AFFINE_CIIP
  pu.ciipAffine = false;
#endif
  pu.tmMergeFlag = false;
}
#endif

void EncCu::xCheckSATDCostMmvdMerge(CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, MergeCtx mergeCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer
  , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    , uint32_t * mmvdLUT
#endif
                                    )
{
#if JVET_AG0276_NLIC
  cu.altLMFlag = false;
  cu.altLMParaUnit.resetAltLinearModel();
#endif
#if INTER_LIC
  cu.licFlag = false;
#endif
  cu.mmvdSkip = true;
  cu.geoFlag = false;
  cu.affine = false;
  cu.imv = 0;

  pu.ciipFlag = false;
#if CIIP_PDPC
  pu.ciipPDPC = false;
#endif
#if JVET_AG0135_AFFINE_CIIP
  pu.ciipAffine = false;
#endif
  pu.mmvdMergeFlag = true;
  pu.regularMergeFlag = true;

  const double sqrtLambdaForFirstPassIntra = m_pcRdCost->getMotionLambda() * FRAC_BITS_SCALE;
  int insertPos = -1;
#if MULTI_HYP_PRED
  const bool testMHP = tempCS->sps->getUseInterMultiHyp()
    && (tempCS->area.lumaSize().area() > MULTI_HYP_PRED_RESTRICT_BLOCK_SIZE
      && std::min(tempCS->area.lwidth(), tempCS->area.lheight()) >= MULTI_HYP_PRED_RESTRICT_MIN_WH);
#endif

  const int tempNum =
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                      !pu.cs->sps->getUseTMMMVD() ?
                      ((mergeCtx.numValidMergeCand > 1) ? VVC_MMVD_ADD_NUM : (VVC_MMVD_ADD_NUM >> 1)) :
#endif
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
                      (std::min<int>(MMVD_BASE_MV_NUM, mergeCtx.numValidMergeCand) * MMVD_MAX_REFINE_NUM);
#else
                      (mergeCtx.numValidMergeCand > 1) ? MMVD_ADD_NUM : MMVD_ADD_NUM >> 1;
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  for (int mmvdMergeCandtemp = 0; mmvdMergeCandtemp < tempNum; mmvdMergeCandtemp++)
  {
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    if(pu.cs->sps->getUseTMMMVD())
#endif
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
    if(mmvdMergeCandtemp - (mmvdMergeCandtemp/MMVD_MAX_REFINE_NUM )* MMVD_MAX_REFINE_NUM  >= ((MMVD_MAX_REFINE_NUM >> MMVD_SIZE_SHIFT )/MMVD_BI_DIR))
#else
    if(mmvdMergeCandtemp - (mmvdMergeCandtemp/MMVD_MAX_REFINE_NUM )* MMVD_MAX_REFINE_NUM  >= (MMVD_MAX_REFINE_NUM >> MMVD_SIZE_SHIFT ))
#endif
    {
      continue;
    }
    int mmvdMergeCand = (mmvdLUT == NULL) ? mmvdMergeCandtemp : mmvdLUT[mmvdMergeCandtemp];
#else
  for (int mmvdMergeCand = 0; mmvdMergeCand < tempNum; mmvdMergeCand++)
  {
#endif
    int baseIdx = 
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                 !pu.cs->sps->getUseTMMMVD() ?
                 mmvdMergeCand / VVC_MMVD_MAX_REFINE_NUM :
#endif
                  mmvdMergeCand / MMVD_MAX_REFINE_NUM;
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    int refineStep = 
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
                     !pu.cs->sps->getUseTMMMVD() ?
                     (mmvdMergeCand - (baseIdx * VVC_MMVD_MAX_REFINE_NUM )) / VVC_MMVD_MAX_DIR :
#endif
                     (mmvdMergeCand - (baseIdx * MMVD_MAX_REFINE_NUM )) / MMVD_MAX_DIR ;
#else
    int refineStep = (mmvdMergeCand - (baseIdx * MMVD_MAX_REFINE_NUM)) / 4;
#endif
#if  !JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED)
    if( refineStep >= m_pcEncCfg->getMmvdDisNum() 
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
     && !pu.cs->sps->getUseTMMMVD()
#endif
      )
    {
      continue;
    }
#endif

#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    mergeCtx.setMmvdMergeCandiInfo(pu, mmvdMergeCandtemp, mmvdMergeCand);
#else
    mergeCtx.setMmvdMergeCandiInfo(pu, mmvdMergeCand);
#endif
    pu.mvRefine = true;
    distParam.cur = singleMergeTempBuffer->Y();
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    pu.mmvdEncOptMode = (refineStep > 1
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
                                    + (pu.cs->sps->getUseTMMMVD() ? 0 : 1)
#endif
                                    ? 2 : 1);
#else
    pu.mmvdEncOptMode = (refineStep > 2 ? 2 : 1);
#endif
    CHECK(!pu.mmvdMergeFlag, "MMVD merge should be set");
    // Don't do chroma MC here
    m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X, true, false);
    pu.mmvdEncOptMode = 0;
    pu.mvRefine = false;
    Distortion uiSad = distParam.distFunc(distParam);

    m_CABACEstimator->getCtx() = ctxStart;
    uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
    double cost = (double)uiSad + (double)fracBits * sqrtLambdaForFirstPassIntra;
#if MULTI_HYP_PRED
    if (testMHP && pu.addHypData.size() < tempCS->sps->getMaxNumAddHyps())
    {
      uint32_t uiBitsCand = baseIdx + refineStep + 2 + 1 + 1 + 1; // one bit for merge flag,  one bit for subblock_merge_flag, and one bit for regualr_merge_flag
      MEResult mergeResult;
      mergeResult.cu = cu;
      mergeResult.pu = pu;
      mergeResult.bits = uiBitsCand;
      mergeResult.cost = uiSad + m_pcRdCost->getCost(uiBitsCand);

      m_baseResultsForMH.push_back(mergeResult);
    }
#endif
    insertPos = -1;
    updateCandList(ModeInfo(cu, pu), cost, rdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
    if (insertPos != -1 && insertPos < MMVD_MRG_MAX_RD_NUM)
    {
      for (int i = int(rdModeList.size()) - 1; i > insertPos; i--)
      {
        swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
      }
      swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
    }
  }
}
#if JVET_AG0276_LIC_FLAG_SIGNALING
void EncCu::xCheckSATDCostAffineMergeOppositeLic(CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, AffineMergeCtx affineMergeCtx, MergeCtx& mrgCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer
  , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart
)
{
  cu.mmvdSkip = false;
  cu.geoFlag = false;
  cu.affine = true;
#if INTER_LIC
  cu.licFlag = false;
#endif
  pu.affineOppositeLic = true;
  pu.afMmvdFlag = false;
  pu.affBMMergeFlag = false;
  pu.mergeFlag = true;
  pu.ciipFlag = false;
#if CIIP_PDPC
  pu.ciipPDPC = false;
#endif
  pu.mmvdMergeFlag = false;
  pu.regularMergeFlag = false;
#if MULTI_HYP_PRED
  pu.addHypData.clear();
  pu.numMergedAddHyps = 0;
  const bool testMHP = tempCS->sps->getUseInterMultiHyp()
    && (tempCS->area.lumaSize().area() > MULTI_HYP_PRED_RESTRICT_BLOCK_SIZE
      && std::min(tempCS->area.lwidth(), tempCS->area.lheight()) >= MULTI_HYP_PRED_RESTRICT_MIN_WH);
#endif

  const double sqrtLambdaForFirstPassIntra = m_pcRdCost->getMotionLambda() * FRAC_BITS_SCALE;
  int insertPos = -1;
  for (uint32_t uiAffMergeCand = 0; uiAffMergeCand < affineMergeCtx.numValidMergeCand; uiAffMergeCand++)
  {
    // set merge information
    pu.interDir = affineMergeCtx.interDirNeighbours[uiAffMergeCand];
    pu.mergeIdx = uiAffMergeCand;
    cu.affineType = affineMergeCtx.affineType[uiAffMergeCand];
    cu.bcwIdx = affineMergeCtx.bcwIdx[uiAffMergeCand];
#if JVET_AG0276_NLIC
    cu.altLMFlag = affineMergeCtx.altLMFlag[uiAffMergeCand];
    cu.altLMParaUnit = affineMergeCtx.altLMParaNeighbours[uiAffMergeCand];
#endif
#if INTER_LIC
    cu.licFlag = affineMergeCtx.licFlags[uiAffMergeCand];
#endif
    pu.mv[0].setZero();
    pu.mv[1].setZero();
    cu.imv = 0;
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    pu.colIdx = affineMergeCtx.colIdx[uiAffMergeCand];
#endif
    pu.mergeType = affineMergeCtx.mergeType[uiAffMergeCand];
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
    cu.obmcFlag = affineMergeCtx.obmcFlags[uiAffMergeCand];
#endif
    PU::setAllAffineMvField(pu, affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0], REF_PIC_LIST_0);
    PU::setAllAffineMvField(pu, affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1], REF_PIC_LIST_1);

    distParam.cur = singleMergeTempBuffer->Y();
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
    if (PU::checkDoAffineBdofRefine(pu, m_pcInterSearch))
    {
      pu.availableBdofRefinedMv = AFFINE_SUBPU_BDOF_APPLY_AND_STORE_MV;
      m_pcInterSearch->setDoAffineSubPuBdof(false);
      m_pcInterSearch->setBdofSubPuMvBuf(m_mvBufEncAffineBDOFOppositeLic[uiAffMergeCand]);
      if (pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
      {
        int bioSubPuIdx = 0;
        const int bioSubPuStrideIncr = BDOF_SUBPU_STRIDE - (int)(pu.lumaSize().width >> BDOF_SUBPU_DIM_LOG2);
        for (int y = 0; y < pu.lumaSize().height; y += 4)
        {
          for (int x = 0; x < pu.lumaSize().width; x += 4)
          {
            m_mvBufEncAffineBDOF[uiAffMergeCand][bioSubPuIdx].setZero();
            bioSubPuIdx++;
          }
          bioSubPuIdx += bioSubPuStrideIncr;
        }
      }
      else
      {
        PU::spanMotionInfo(pu);
      }
      m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X);
    }
    else
    {
      m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X, true, false);
    }
    pu.availableBdofRefinedMv = AFFINE_SUBPU_BDOF_NOT_APPLY;
    m_doEncAffineBDOFOppositeLic[uiAffMergeCand] = false;
#else
    m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X, true, false);
#endif
    Distortion uiSad = distParam.distFunc(distParam);

    m_CABACEstimator->getCtx() = ctxStart;
    uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
    double cost = (double)uiSad + (double)fracBits * sqrtLambdaForFirstPassIntra;
#if MULTI_HYP_PRED
    if (testMHP && pu.addHypData.size() < tempCS->sps->getMaxNumAddHyps())
    {
      uint32_t   uiBitsCand = uiAffMergeCand + 1;
      if (uiAffMergeCand == tempCS->picHeader->getMaxNumAffineMergeCand() - 1)
      {
        uiBitsCand--;
      }
      uiBitsCand = uiBitsCand + 1 + 1 + 1; // one bit for merge flag, and one bit for subblock_merge_flag, one for affineOppoLic
      MEResult mergeResult;
      mergeResult.cu = cu;
      mergeResult.pu = pu;
      mergeResult.bits = uiBitsCand;
      mergeResult.cost = uiSad + m_pcRdCost->getCost(uiBitsCand);
      m_baseResultsForMH.push_back(mergeResult);
    }
#endif
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
    m_pcInterSearch->setDoAffineSubPuBdof(false);
#endif
    insertPos = -1;
    updateCandList(ModeInfo(cu, pu), cost, rdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
#if MERGE_ENC_OPT
    if (insertPos != -1 && insertPos < MMVD_MRG_MAX_RD_NUM)
#else
    if (insertPos != -1)
#endif
    {
      for (int i = int(rdModeList.size()) - 1; i > insertPos; i--)
      {
        swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
      }
      swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
    }
  }
  pu.regularMergeFlag = true;
  cu.affine = false;
  pu.affineOppositeLic = false;
}
#endif
#if JVET_AG0135_AFFINE_CIIP
void EncCu::xCheckSATDCostAffineMerge(CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, AffineMergeCtx affineMergeCtx, MergeCtx& mrgCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer, PelUnitBuf  acMergeAffineBuffer[AFFINE_MRG_MAX_NUM_CANDS]
  , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart)
#else
void EncCu::xCheckSATDCostAffineMerge(CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, AffineMergeCtx affineMergeCtx, MergeCtx& mrgCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer
  , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart)
#endif
{
  cu.mmvdSkip = false;
  cu.geoFlag = false;
  cu.affine = true;
#if INTER_LIC
  cu.licFlag = false;
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
  pu.affineOppositeLic = false;
#endif

  pu.mergeFlag = true;
  pu.ciipFlag = false;
#if CIIP_PDPC
  pu.ciipPDPC = false;
#endif
#if JVET_AG0135_AFFINE_CIIP
  pu.ciipAffine = false;
#endif
  pu.mmvdMergeFlag = false;
  pu.regularMergeFlag = false;
#if MULTI_HYP_PRED
  pu.addHypData.clear();
  pu.numMergedAddHyps = 0;
  const bool testMHP = tempCS->sps->getUseInterMultiHyp()
    && (tempCS->area.lumaSize().area() > MULTI_HYP_PRED_RESTRICT_BLOCK_SIZE
      && std::min(tempCS->area.lwidth(), tempCS->area.lheight()) >= MULTI_HYP_PRED_RESTRICT_MIN_WH);
#endif
#if JVET_AG0135_AFFINE_CIIP
  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));
#endif
  const double sqrtLambdaForFirstPassIntra = m_pcRdCost->getMotionLambda() * FRAC_BITS_SCALE;
  int insertPos = -1;
  for (uint32_t uiAffMergeCand = 0; uiAffMergeCand < affineMergeCtx.numValidMergeCand; uiAffMergeCand++)
  {
    // set merge information
    pu.interDir = affineMergeCtx.interDirNeighbours[uiAffMergeCand];
    pu.mergeIdx = uiAffMergeCand;
    cu.affineType = affineMergeCtx.affineType[uiAffMergeCand];
    cu.bcwIdx = affineMergeCtx.bcwIdx[uiAffMergeCand];
#if JVET_AG0276_NLIC
    cu.altLMFlag = affineMergeCtx.altLMFlag[uiAffMergeCand];
    cu.altLMParaUnit = affineMergeCtx.altLMParaNeighbours[uiAffMergeCand];
#endif
#if INTER_LIC
    cu.licFlag = affineMergeCtx.licFlags[uiAffMergeCand];
#endif
    pu.mv[0].setZero();
    pu.mv[1].setZero();
    cu.imv = 0;
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    pu.colIdx = affineMergeCtx.colIdx[uiAffMergeCand];
#endif
    pu.mergeType = affineMergeCtx.mergeType[uiAffMergeCand];
    if (pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
    {
      pu.refIdx[0] = affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0][0].refIdx;
      pu.refIdx[1] = affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1][0].refIdx;
      // the SbTmvp use xSubPuMC which will need to access the motion buffer for subblock MV
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
      cu.obmcFlag = true;
#endif
      PU::spanMotionInfo(pu, mrgCtx
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
        , pu.colIdx
#endif 
      );
    }
    else
    {
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
      cu.obmcFlag = affineMergeCtx.obmcFlags[uiAffMergeCand];
#endif
      PU::setAllAffineMvField(pu, affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0], REF_PIC_LIST_0);
      PU::setAllAffineMvField(pu, affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1], REF_PIC_LIST_1);
    }

    distParam.cur = singleMergeTempBuffer->Y();

#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
    if (PU::checkDoAffineBdofRefine(pu, m_pcInterSearch))
    {
      pu.availableBdofRefinedMv = AFFINE_SUBPU_BDOF_APPLY_AND_STORE_MV;
      m_pcInterSearch->setDoAffineSubPuBdof(false);
      m_pcInterSearch->setBdofSubPuMvBuf(m_mvBufEncAffineBDOF[uiAffMergeCand]);
      if (pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
      {
        int bioSubPuIdx = 0;
        const int bioSubPuStrideIncr = BDOF_SUBPU_STRIDE - (int)(pu.lumaSize().width >> BDOF_SUBPU_DIM_LOG2);
        for (int yy = 0; yy < pu.lumaSize().height; yy += 4)
        {
          for (int xx = 0; xx < pu.lumaSize().width; xx += 4)
          {
            m_mvBufEncAffineBDOF[uiAffMergeCand][bioSubPuIdx].setZero();
            bioSubPuIdx++;
          }
          bioSubPuIdx += bioSubPuStrideIncr;
        }
      }
      else
      {
        PU::spanMotionInfo(pu);
      }
      m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X);
    }
    else
    {
      m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X, true, false);
    }
    pu.availableBdofRefinedMv = AFFINE_SUBPU_BDOF_NOT_APPLY;
    m_doEncAffineBDOF[uiAffMergeCand] = m_pcInterSearch->getDoAffineSubPuBdof();
#else
    m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X, true, false);
#endif
#if JVET_AG0135_AFFINE_CIIP
    const UnitArea localUnitArea(cu.cs->area.chromaFormat, Area(0, 0, cu.Y().width, cu.Y().height));
    acMergeAffineBuffer[uiAffMergeCand] = m_acMergeAffineBuffer[uiAffMergeCand].getBuf(localUnitArea);
    acMergeAffineBuffer[uiAffMergeCand].copyFrom(*singleMergeTempBuffer);
#endif
    Distortion uiSad = distParam.distFunc(distParam);

    m_CABACEstimator->getCtx() = ctxStart;
    uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
    double cost = (double)uiSad + (double)fracBits * sqrtLambdaForFirstPassIntra;
#if MULTI_HYP_PRED
    if (testMHP && pu.addHypData.size() < tempCS->sps->getMaxNumAddHyps())
    {
      uint32_t   uiBitsCand = uiAffMergeCand + 1;
      if (uiAffMergeCand == tempCS->picHeader->getMaxNumAffineMergeCand() - 1)
      {
        uiBitsCand--;
      }
#if JVET_AG0276_LIC_FLAG_SIGNALING
      uiBitsCand = uiBitsCand + 1 + 1 + 1; // one bit for merge flag, and one bit for subblock_merge_flag
#else
      uiBitsCand = uiBitsCand + 1 + 1; // one bit for merge flag, and one bit for subblock_merge_flag
#endif
      MEResult mergeResult;
      mergeResult.cu = cu;
      mergeResult.pu = pu;
      mergeResult.bits = uiBitsCand;
      mergeResult.cost = uiSad + m_pcRdCost->getCost(uiBitsCand);
      m_baseResultsForMH.push_back(mergeResult);
    }
#endif
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
    m_pcInterSearch->setDoAffineSubPuBdof(false);
#endif
    insertPos = -1;
    updateCandList(ModeInfo(cu, pu), cost, rdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
#if MERGE_ENC_OPT
    if (insertPos != -1 && insertPos < MMVD_MRG_MAX_RD_NUM)
#else
    if (insertPos != -1)
#endif
    {
      for (int i = int(rdModeList.size()) - 1; i > insertPos; i--)
      {
        swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
      }
      swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
    }
  }
  pu.regularMergeFlag = true;
  cu.affine = false;
}
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
void EncCu::xCheckSATDCostBMAffineMerge(CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, AffineMergeCtx affineMergeCtx, RefPicList reflist, MergeCtx& mrgCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer
  , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart
)
{
  cu.mmvdSkip = false;
  cu.geoFlag = false;
  cu.affine = true;
#if INTER_LIC
  cu.licFlag = false;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
   cu.obmcFlag= true;
#endif
  pu.affBMMergeFlag = true;
  pu.affBMDir = reflist == REF_PIC_LIST_0 ? 1 : 2;
  pu.afMmvdFlag = false;
  pu.mergeFlag = true;
  pu.ciipFlag = false;
#if CIIP_PDPC
  pu.ciipPDPC = false;
#endif
#if JVET_AG0135_AFFINE_CIIP
  pu.ciipAffine = false;
#endif
  pu.mmvdMergeFlag = false;
  pu.regularMergeFlag = false;
#if MULTI_HYP_PRED
  pu.addHypData.clear();
  pu.numMergedAddHyps = 0;
  const bool testMHP = tempCS->sps->getUseInterMultiHyp()
    && (tempCS->area.lumaSize().area() > MULTI_HYP_PRED_RESTRICT_BLOCK_SIZE
      && std::min(tempCS->area.lwidth(), tempCS->area.lheight()) >= MULTI_HYP_PRED_RESTRICT_MIN_WH);
#endif

  const double sqrtLambdaForFirstPassIntra = m_pcRdCost->getMotionLambda() * FRAC_BITS_SCALE;
  int insertPos = -1;
  for (uint32_t uiAffMergeCand = 0; uiAffMergeCand < affineMergeCtx.numValidMergeCand; uiAffMergeCand++)
  {
    // set merge information
    pu.interDir = affineMergeCtx.interDirNeighbours[uiAffMergeCand];
    pu.mergeIdx = uiAffMergeCand;
    cu.affineType = affineMergeCtx.affineType[uiAffMergeCand];
    cu.bcwIdx = affineMergeCtx.bcwIdx[uiAffMergeCand];
#if JVET_AG0276_NLIC
    cu.altLMFlag = affineMergeCtx.altLMFlag[uiAffMergeCand];
    cu.altLMParaUnit = affineMergeCtx.altLMParaNeighbours[uiAffMergeCand];
#endif
#if INTER_LIC
    cu.licFlag = affineMergeCtx.licFlags[uiAffMergeCand];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
   cu.obmcFlag= affineMergeCtx.obmcFlags[uiAffMergeCand];
#endif
    pu.mv[0].setZero();
    pu.mv[1].setZero();
    cu.imv = 0;

    pu.mergeType = affineMergeCtx.mergeType[uiAffMergeCand];
    {
      PU::setAllAffineMvField(pu, affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 0], REF_PIC_LIST_0);
      PU::setAllAffineMvField(pu, affineMergeCtx.mvFieldNeighbours[(uiAffMergeCand << 1) + 1], REF_PIC_LIST_1);
    }

    distParam.cur = singleMergeTempBuffer->Y();

#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
    if (PU::checkDoAffineBdofRefine(pu, m_pcInterSearch))
    {
      pu.availableBdofRefinedMv = AFFINE_SUBPU_BDOF_APPLY_AND_STORE_MV;
      m_pcInterSearch->setDoAffineSubPuBdof(false);
      m_pcInterSearch->setBdofSubPuMvBuf(m_mvBufEncAffineBmBDOF[(uiAffMergeCand << 1) + pu.affBMDir - 1]);
      PU::spanMotionInfo(pu);
      m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X);
    }
    else
    {
      m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X, true, false);
    }
    pu.availableBdofRefinedMv = AFFINE_SUBPU_BDOF_NOT_APPLY;
    m_doEncAffineBmBDOF[(uiAffMergeCand << 1) + pu.affBMDir - 1] = m_pcInterSearch->getDoAffineSubPuBdof();
#else
    m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X, true, false);
#endif
    Distortion uiSad = distParam.distFunc(distParam);

    m_CABACEstimator->getCtx() = ctxStart;
    uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
    double cost = (double)uiSad + (double)fracBits * sqrtLambdaForFirstPassIntra;
#if MULTI_HYP_PRED
    if (testMHP && pu.addHypData.size() < tempCS->sps->getMaxNumAddHyps())
    {
      uint32_t   uiBitsCand = uiAffMergeCand + 1;
      if (uiAffMergeCand == tempCS->picHeader->getMaxNumAffineMergeCand() - 1)
      {
        uiBitsCand--;
      }
      uiBitsCand = uiBitsCand + 1 + 1; // one bit for merge flag, and one bit for subblock_merge_flag
      MEResult mergeResult;
      mergeResult.cu = cu;
      mergeResult.pu = pu;
      mergeResult.bits = uiBitsCand;
      mergeResult.cost = uiSad + m_pcRdCost->getCost(uiBitsCand);
      m_baseResultsForMH.push_back(mergeResult);
    }
#endif
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
    m_pcInterSearch->setDoAffineSubPuBdof(false);
#endif
    insertPos = -1;
    updateCandList(ModeInfo(cu, pu), cost, rdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
#if MERGE_ENC_OPT
    if (insertPos != -1 && insertPos < MMVD_MRG_MAX_RD_NUM)
#else
    if (insertPos != -1)
#endif
    {
      for (int i = int(rdModeList.size()) - 1; i > insertPos; i--)
      {
        swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
      }
      swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
    }
  }
  pu.regularMergeFlag = true;
  cu.affine = false;
  pu.affBMMergeFlag = false;
  pu.affBMDir = 0;
}
#endif
#if TM_MRG && MERGE_ENC_OPT
void EncCu::xCheckSATDCostTMMerge(       CodingStructure*& tempCS,
                                         CodingUnit&       cu,
                                         PredictionUnit&   pu,
                                         MergeCtx&         mrgCtx,
                                         PelUnitBuf*       acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM],
                                         PelUnitBuf*&      singleMergeTempBuffer,
                                         unsigned&         uiNumMrgSATDCand,
                                         static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList,
                                         static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>    &candCostList,
                                         DistParam         distParam,
                                   const TempCtx&          ctxStart
#if MULTI_PASS_DMVR
                                       , bool*             applyBDMVR
#endif
#if JVET_AF0057
                                       , bool dmvr4TMImpreciseMv
#endif
)
{
#if MULTI_PASS_DMVR
  CHECK(applyBDMVR == nullptr, "Unexpected error");
#endif
  pu.mergeFlag        = true;
  cu.mmvdSkip         = false;
  cu.geoFlag          = false;
  cu.affine           = false;
  cu.imv              = IMV_OFF;
  pu.ciipFlag         = false;
#if CIIP_PDPC
  pu.ciipPDPC         = false;
#endif
#if JVET_AG0135_AFFINE_CIIP
  pu.ciipAffine       = false;
#endif
  pu.mmvdMergeFlag    = false;
  pu.regularMergeFlag = false;
  pu.tmMergeFlag      = true;
#if JVET_AG0276_LIC_FLAG_SIGNALING
  pu.tmMergeFlagOppositeLic = false;
#endif

  const double sqrtLambdaForFirstPassIntra = m_pcRdCost->getMotionLambda() * FRAC_BITS_SCALE;
  int insertPos = -1;
  for (uint32_t uiMergeCand = 0; uiMergeCand < mrgCtx.numValidMergeCand; uiMergeCand++)
  {
    mrgCtx.setMergeInfo( pu, uiMergeCand );
#if MULTI_PASS_DMVR
    if (applyBDMVR[uiMergeCand])
    {
      pu.bdmvrRefine = true;
      m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4TM[uiMergeCand << 1], m_mvBufBDMVR4TM[(uiMergeCand << 1) + 1]);
    }
#if !BDOF_RM_CONSTRAINTS
    else
#endif
#endif
#if !BDOF_RM_CONSTRAINTS
    {
      PU::spanMotionInfo(pu, mrgCtx);
    }
#endif

    pu.mvRefine = false;
#if INTER_LIC
    m_pcInterSearch->m_storeBeforeLIC = false;
#endif
    m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer);
#if MULTI_PASS_DMVR
    if( pu.bdmvrRefine )
    {
      ::memcpy( m_mvBufEncBDOF4TM[uiMergeCand], m_pcInterSearch->getBdofSubPuMvOffset(), sizeof( Mv ) * BDOF_SUBPU_MAX_NUM );
    }
#endif
    distParam.cur = singleMergeTempBuffer->Y();
    Distortion uiSad = distParam.distFunc(distParam);

    m_CABACEstimator->getCtx() = ctxStart;
    uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
    double cost = (double)uiSad + (double)fracBits * sqrtLambdaForFirstPassIntra;
#if JVET_AF0057
    if (m_pcInterSearch->dmvrEnableEncoderCheck && dmvr4TMImpreciseMv)
    {
      cost = MAX_DOUBLE;
    }
#endif
    insertPos = -1;
    updateCandList(ModeInfo(cu, pu), cost, rdModeList, candCostList, uiNumMrgSATDCand, &insertPos);

    if (insertPos != -1 && insertPos < MMVD_MRG_MAX_RD_NUM)
    {
      for (int i = int(rdModeList.size()) - 1; i > insertPos; i--)
      {
        swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
      }
      swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
    }
  }
  pu.regularMergeFlag = true;
  cu.affine           = false;
#if AFFINE_MMVD
  pu.afMmvdFlag       = false;
#endif
  pu.tmMergeFlag      = false;
#if MULTI_PASS_DMVR
  pu.bdmvrRefine      = false;
#endif
}
#if JVET_AG0276_LIC_FLAG_SIGNALING
#if TM_MRG && MERGE_ENC_OPT
void EncCu::xCheckSATDCostTMMergeOppositeLic( CodingStructure*& tempCS,
                                          CodingUnit&       cu,
                                          PredictionUnit&   pu,
                                          MergeCtx&         mrgCtx,
                                          PelUnitBuf*       acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM],
                                          PelUnitBuf*&      singleMergeTempBuffer,
                                          unsigned&         uiNumMrgSATDCand,
                                          static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList,
                                          static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>    &candCostList,
                                          DistParam         distParam,
                                          const TempCtx&    ctxStart,
#if MULTI_PASS_DMVR
                                          bool*             applyBDMVR
#endif
)
{
  pu.mergeFlag = true;
  cu.mmvdSkip = false;
  cu.geoFlag = false;
  cu.affine = false;
  cu.imv = IMV_OFF;
  pu.ciipFlag = false;
#if CIIP_PDPC
  pu.ciipPDPC = false;
#endif
  pu.mmvdMergeFlag = false;
  pu.regularMergeFlag = false;
  pu.tmMergeFlag = true;
  pu.tmMergeFlagOppositeLic = true;
  const double sqrtLambdaForFirstPassIntra = m_pcRdCost->getMotionLambda() * FRAC_BITS_SCALE;
  int insertPos = -1;
  for (uint32_t uiMergeCand = 0; uiMergeCand < mrgCtx.numValidMergeCand; uiMergeCand++)
  {
    mrgCtx.setMergeInfo(pu, uiMergeCand);
#if MULTI_PASS_DMVR
    if (applyBDMVR[uiMergeCand])
    {
      pu.bdmvrRefine = true;
      m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4TMOPPOSITELIC[uiMergeCand << 1], m_mvBufBDMVR4TMOPPOSITELIC[(uiMergeCand << 1) + 1]);
    }
#if !BDOF_RM_CONSTRAINTS
    else
#endif
#endif
#if !BDOF_RM_CONSTRAINTS
    {
      PU::spanMotionInfo(pu, mrgCtx);
    }
#endif

    pu.mvRefine = false;
#if INTER_LIC
    m_pcInterSearch->m_storeBeforeLIC = false;
#endif
    m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer);
#if MULTI_PASS_DMVR
    if (pu.bdmvrRefine)
    {
      ::memcpy(m_mvBufEncBDOF4TMOPPOSITELIC[uiMergeCand], m_pcInterSearch->getBdofSubPuMvOffset(), sizeof(Mv) * BDOF_SUBPU_MAX_NUM);
    }
#endif
    distParam.cur = singleMergeTempBuffer->Y();
    Distortion uiSad = distParam.distFunc(distParam);

    m_CABACEstimator->getCtx() = ctxStart;
    uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
    double cost = (double)uiSad + (double)fracBits * sqrtLambdaForFirstPassIntra;
    insertPos = -1;
    updateCandList(ModeInfo(cu, pu), cost, rdModeList, candCostList, uiNumMrgSATDCand, &insertPos);

    if (insertPos != -1 && insertPos < MMVD_MRG_MAX_RD_NUM)
    {
      for (int i = int(rdModeList.size()) - 1; i > insertPos; i--)
      {
        swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
      }
      swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
    }
  }
  pu.regularMergeFlag = true;
  cu.affine = false;
#if AFFINE_MMVD
  pu.afMmvdFlag = false;
#endif
  pu.tmMergeFlag = false;
  pu.tmMergeFlagOppositeLic = false;
#if MULTI_PASS_DMVR
  pu.bdmvrRefine = false;
#endif
}
#endif
#endif
#endif

#if AFFINE_MMVD && MERGE_ENC_OPT
void EncCu::xCheckSATDCostAffineMmvdMerge(       CodingStructure*& tempCS,
                                                 CodingUnit&       cu,
                                                 PredictionUnit&   pu,
                                                 AffineMergeCtx    affineMergeCtx,
                                                 MergeCtx&         mrgCtx,
                                                 PelUnitBuf*       acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM],
                                                 PelUnitBuf*&      singleMergeTempBuffer,
                                                 unsigned&         uiNumMrgSATDCand,
                                                 static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList,
                                                 static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>    &candCostList,
                                                 DistParam         distParam,
                                           const TempCtx&          ctxStart
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                                          , uint32_t * affMmvdLUT
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  , uint8_t numBaseAffine
#endif
#endif
)
{
  cu.mmvdSkip         = false;
  cu.geoFlag          = false;
  cu.affine           = true;
  cu.imv              = IMV_OFF;
#if INTER_LIC
  cu.licFlag = false;
#endif

  pu.mergeFlag        = true;
  pu.ciipFlag         = false;
#if CIIP_PDPC
  pu.ciipPDPC         = false;
#endif
#if JVET_AG0135_AFFINE_CIIP
  pu.ciipAffine       = false;
#endif
  pu.mmvdMergeFlag    = false;
  pu.regularMergeFlag = false;
#if MULTI_HYP_PRED
  pu.addHypData.clear();
  pu.numMergedAddHyps = 0;
#endif

  int baseIdxToMergeIdxOffset = (int)PU::getMergeIdxFromAfMmvdBaseIdx(affineMergeCtx, 0);
  int baseCount               = std::min<int>((int)AF_MMVD_BASE_NUM, affineMergeCtx.numValidMergeCand - baseIdxToMergeIdxOffset);
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_AA0093_ENHANCED_MMVD_EXTENSION
  if (!tempCS->sps->getUseTMMMVD())
  {
    baseCount = std::min<int>((int)ECM3_AF_MMVD_BASE_NUM, affineMergeCtx.numValidMergeCand - baseIdxToMergeIdxOffset);
  }
#endif
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  baseCount = std::min<int>(baseCount, numBaseAffine);
#endif
  int afMmvdCandCount         = 
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                                !pu.cs->sps->getUseTMMMVD() ?
                                baseCount * ECM3_AF_MMVD_MAX_REFINE_NUM :
#endif
                                baseCount * AF_MMVD_MAX_REFINE_NUM;
  if (baseCount < 1)
  {
    return;
  }

#if MULTI_HYP_PRED
  const bool testMHP = tempCS->sps->getUseInterMultiHyp()
    && (tempCS->area.lumaSize().area() > MULTI_HYP_PRED_RESTRICT_BLOCK_SIZE
      && std::min(tempCS->area.lwidth(), tempCS->area.lheight()) >= MULTI_HYP_PRED_RESTRICT_MIN_WH);
#endif
  const double sqrtLambdaForFirstPassIntra = m_pcRdCost->getMotionLambda() * FRAC_BITS_SCALE;
  int insertPos = -1;
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  for (uint32_t uiMergeCandTemp = 0; uiMergeCandTemp < afMmvdCandCount; uiMergeCandTemp++)
  {
    uint32_t uiMergeCand = affMmvdLUT[uiMergeCandTemp];
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    if(pu.cs->sps->getUseTMMMVD())
#endif
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
    if(uiMergeCandTemp - (uiMergeCandTemp/AF_MMVD_MAX_REFINE_NUM )* AF_MMVD_MAX_REFINE_NUM  >= ((AF_MMVD_MAX_REFINE_NUM >> AFFINE_MMVD_SIZE_SHIFT ) / AFFINE_BI_DIR))
#else
    if(uiMergeCandTemp - (uiMergeCandTemp/AF_MMVD_MAX_REFINE_NUM )* AF_MMVD_MAX_REFINE_NUM  >= (AF_MMVD_MAX_REFINE_NUM >> AFFINE_MMVD_SIZE_SHIFT))
#endif
    {
      continue;
    }
#else
  for (uint32_t uiMergeCand = 0; uiMergeCand < afMmvdCandCount; uiMergeCand++)
  {
#endif
    int baseIdx = 
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                  !pu.cs->sps->getUseTMMMVD() ?
                  (int)uiMergeCand / ECM3_AF_MMVD_MAX_REFINE_NUM :
#endif
                  (int)uiMergeCand / AF_MMVD_MAX_REFINE_NUM;
    int stepIdx = 
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                  !pu.cs->sps->getUseTMMMVD() ?
                  (int)uiMergeCand - baseIdx * ECM3_AF_MMVD_MAX_REFINE_NUM :
#endif
                  (int)uiMergeCand - baseIdx * AF_MMVD_MAX_REFINE_NUM;
    int dirIdx  = 
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                  !pu.cs->sps->getUseTMMMVD() ?
                  stepIdx % ECM3_AF_MMVD_OFFSET_DIR :
#endif
                  stepIdx % AF_MMVD_OFFSET_DIR;
        stepIdx = 
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                  !pu.cs->sps->getUseTMMMVD() ?
                  stepIdx / ECM3_AF_MMVD_OFFSET_DIR :
#endif
                  stepIdx / AF_MMVD_OFFSET_DIR;

    // Pass Affine MMVD parameters from candidate to PU
    {
      pu.afMmvdFlag     = true;
      pu.afMmvdBaseIdx  = (uint8_t)baseIdx;
      pu.afMmvdDir      = (uint8_t)dirIdx;
      pu.afMmvdStep     = (uint8_t)stepIdx;
      pu.mergeIdx       = (uint8_t)(baseIdx + baseIdxToMergeIdxOffset);
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
      pu.afMmvdMergeIdx = (uint16_t)uiMergeCandTemp;
#else
      pu.afMmvdMergeIdx = (uint8_t)uiMergeCandTemp;
#endif
#endif
      pu.mergeType      = affineMergeCtx.mergeType         [pu.mergeIdx];
      pu.interDir       = affineMergeCtx.interDirNeighbours[pu.mergeIdx];
      pu.cu->affineType = affineMergeCtx.affineType        [pu.mergeIdx];
#if JVET_AG0276_NLIC
      pu.cu->altLMFlag = affineMergeCtx.altLMFlag[pu.mergeIdx];
      pu.cu->altLMParaUnit = affineMergeCtx.altLMParaNeighbours[pu.mergeIdx];
#endif
#if INTER_LIC
      pu.cu->licFlag    = affineMergeCtx.licFlags          [pu.mergeIdx];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
      pu.cu->obmcFlag   = affineMergeCtx.obmcFlags         [pu.mergeIdx];
#endif
      pu.cu->bcwIdx     = affineMergeCtx.bcwIdx            [pu.mergeIdx];
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
      CHECK(pu.afMmvdDir >= (pu.cs->sps->getUseTMMMVD() ? AF_MMVD_OFFSET_DIR : ECM3_AF_MMVD_OFFSET_DIR) || pu.afMmvdStep >= (pu.cs->sps->getUseTMMMVD() ? AF_MMVD_STEP_NUM : ECM3_AF_MMVD_STEP_NUM), "Affine MMVD dir or Affine MMVD step is out of range ");
#else
      CHECK(pu.afMmvdDir >= AF_MMVD_OFFSET_DIR || pu.afMmvdStep >= AF_MMVD_STEP_NUM, "Affine MMVD dir or Affine MMVD step is out of range ");
#endif
      CHECK(pu.mergeType != MRG_TYPE_DEFAULT_N, "Affine MMVD must have non-SbTMVP base!");
    }

    MvField mvfMmvd[2][3];
    PU::getAfMmvdMvf(pu, affineMergeCtx, mvfMmvd, pu.mergeIdx, pu.afMmvdStep, pu.afMmvdDir);
    PU::setAllAffineMvField(pu, mvfMmvd[0], REF_PIC_LIST_0);
    PU::setAllAffineMvField(pu, mvfMmvd[1], REF_PIC_LIST_1);
    distParam.cur = singleMergeTempBuffer->Y();
    pu.mmvdEncOptMode = (stepIdx > 2 ? 3 : 0);
    m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer, REF_PIC_LIST_X, true, false);
    pu.mmvdEncOptMode = 0;
    Distortion uiSad = distParam.distFunc(distParam);

    m_CABACEstimator->getCtx() = ctxStart;
    uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
    double cost = (double)uiSad + (double)fracBits * sqrtLambdaForFirstPassIntra;
#if MULTI_HYP_PRED
    if (testMHP && pu.addHypData.size() < tempCS->sps->getMaxNumAddHyps())
    {
      uint32_t uiBitsCand = baseIdx + stepIdx + 2 + 1 + 1 + 1; // one bit for merge flag,  one bit for subblock_merge_flag, and one bit for afMmvdFlag
      MEResult mergeResult;
      mergeResult.cu = cu;
      mergeResult.pu = pu;
      mergeResult.bits = uiBitsCand;
      mergeResult.cost = uiSad + m_pcRdCost->getCost(uiBitsCand);

      m_baseResultsForMH.push_back(mergeResult);
    }
#endif
    insertPos = -1;
    updateCandList(ModeInfo(cu, pu), cost, rdModeList, candCostList, uiNumMrgSATDCand, &insertPos);

    if (insertPos != -1 && insertPos < MMVD_MRG_MAX_RD_NUM)
    {
      for (int i = int(rdModeList.size()) - 1; i > insertPos; i--)
      {
        swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
      }
      swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
    }
  }
  pu.regularMergeFlag = true;
  cu.affine = false;
  pu.afMmvdFlag = false;
}
#endif
#if !JVET_W0097_GPM_MMVD_TM && !JVET_Z0056_GPM_SPLIT_MODE_REORDERING
void EncCu::xCheckSATDCostGeoMerge(CodingStructure *&tempCS, CodingUnit &cu, PredictionUnit &pu, MergeCtx geoMergeCtx, PelUnitBuf *acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM], PelUnitBuf *&singleMergeTempBuffer
  , unsigned& uiNumMrgSATDCand, static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList, static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM> &candCostList, DistParam distParam, const TempCtx &ctxStart)
{
  const SPS &sps = *tempCS->sps;
  int numGeoChecked = 0;
  GeoComboCostList comboList;
  int bitsCandTB = floorLog2(GEO_NUM_PARTITION_MODE);
  PelUnitBuf geoBuffer[GEO_MAX_NUM_UNI_CANDS];
  PelUnitBuf geoTempBuf[GEO_MAX_NUM_UNI_CANDS];

  uint8_t maxNumMergeCandidates = cu.cs->sps->getMaxNumGeoCand();
  DistParam distParamWholeBlk;
  m_pcRdCost->setDistParam(distParamWholeBlk, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y().buf, m_acMergeBuffer[0].Y().stride, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y);
  Distortion bestWholeBlkSad = MAX_UINT64;
  double bestWholeBlkCost = MAX_DOUBLE;

  Distortion sadWholeBlk[GEO_MAX_NUM_UNI_CANDS];
  int pocMrg[GEO_MAX_NUM_UNI_CANDS];
  Mv MrgMv[GEO_MAX_NUM_UNI_CANDS];
  bool isSkipThisCand[GEO_MAX_NUM_UNI_CANDS] = { false };

  cu.affine = false;
  cu.mtsFlag = false;
#if INTER_LIC
  cu.licFlag = false;
#endif
  cu.bcwIdx = BCW_DEFAULT;
  cu.geoFlag = true;
  cu.imv = 0;
  cu.mmvdSkip = false;
  cu.skip = false;
  cu.mipFlag = false;
#if JVET_V0130_INTRA_TMP
  cu.tmpFlag = false;
#endif
  cu.bdpcmMode = 0;
  pu.mergeFlag = true;
  pu.regularMergeFlag = false;
  pu.mmvdMergeFlag = false;
  pu.mmvdMergeIdx = MAX_UCHAR;

  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));
  const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda();
  const double sqrtLambdaForFirstPassIntra = m_pcRdCost->getMotionLambda() * FRAC_BITS_SCALE;
  int insertPos = -1;

  for (uint8_t mergeCand = 0; mergeCand < maxNumMergeCandidates; mergeCand++)
  {
    geoBuffer[mergeCand] = m_acRealMergeBuffer[mergeCand].getBuf(localUnitArea);
    geoMergeCtx.setMergeInfo(pu, mergeCand);
    int MrgList = geoMergeCtx.mvFieldNeighbours[(mergeCand << 1) + 0].refIdx == -1 ? 1 : 0;
    RefPicList MrgeRefPicList = (MrgList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
    int MrgrefIdx = geoMergeCtx.mvFieldNeighbours[(mergeCand << 1) + MrgList].refIdx;
    pocMrg[mergeCand] = tempCS->slice->getRefPic(MrgeRefPicList, MrgrefIdx)->getPOC();
    MrgMv[mergeCand] = geoMergeCtx.mvFieldNeighbours[(mergeCand << 1) + MrgList].mv;

    for( int i = 0; i < mergeCand; i++ )
    {
      if( pocMrg[mergeCand] == pocMrg[i] && MrgMv[mergeCand] == MrgMv[i] )
      {
        isSkipThisCand[mergeCand] = true;
        break;
      }
    }

    m_pcInterSearch->motionCompensation(pu, geoBuffer[mergeCand]);
    geoTempBuf[mergeCand] = m_acRealMergeBuffer[MRG_MAX_NUM_CANDS + mergeCand].getBuf(localUnitArea);
    geoTempBuf[mergeCand].Y().copyFrom(geoBuffer[mergeCand].Y());
    geoTempBuf[mergeCand].Y().roundToOutputBitdepth(geoTempBuf[mergeCand].Y(), cu.slice->clpRng(COMPONENT_Y));
    distParamWholeBlk.cur.buf = geoTempBuf[mergeCand].Y().buf;
    distParamWholeBlk.cur.stride = geoTempBuf[mergeCand].Y().stride;
    sadWholeBlk[mergeCand] = distParamWholeBlk.distFunc(distParamWholeBlk);
    if (sadWholeBlk[mergeCand] < bestWholeBlkSad)
    {
      bestWholeBlkSad = sadWholeBlk[mergeCand];
      bestWholeBlkCost = (double)bestWholeBlkSad + (double)( mergeCand + 1 ) * sqrtLambdaForFirstPass;
    }
  }

  bool skipGeo = true;
  for (uint8_t mergeCand = 1; mergeCand < maxNumMergeCandidates; mergeCand++)
  {
    if( !isSkipThisCand[mergeCand] )
    {
      skipGeo = false;
      break;
    }
  }

  if( !skipGeo )
  {
    DistParam distParamGeo;
    int wIdx = floorLog2(cu.lwidth()) - GEO_MIN_CU_LOG2;
    int hIdx = floorLog2(cu.lheight()) - GEO_MIN_CU_LOG2;
    for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
    {
      int maskStride = 0, maskStride2 = 0;
      int stepX = 1;
      Pel* sadMask;
      int16_t angle = g_geoParams[splitDir][0];
      if (g_angle2mirror[angle] == 2)
      {
        maskStride = -GEO_WEIGHT_MASK_SIZE;
        maskStride2 = -(int)cu.lwidth();
        sadMask = &g_geoEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
      }
      else if (g_angle2mirror[angle] == 1)
      {
        stepX = -1;
        maskStride2 = cu.lwidth();
        maskStride = GEO_WEIGHT_MASK_SIZE;
        sadMask = &g_geoEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
      }
      else
      {
        maskStride = GEO_WEIGHT_MASK_SIZE;
        maskStride2 = -(int)cu.lwidth();
        sadMask = &g_geoEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
      }
      Distortion sadSmall = 0, sadLarge = 0;
      for (uint8_t mergeCand = 0; mergeCand < maxNumMergeCandidates; mergeCand++)
      {
        int bitsCand = mergeCand + 1;

        m_pcRdCost->setDistParam(distParamGeo, tempCS->getOrgBuf().Y(), geoTempBuf[mergeCand].Y().buf, geoTempBuf[mergeCand].Y().stride, sadMask, maskStride, stepX, maskStride2, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y);
        sadLarge = distParamGeo.distFunc(distParamGeo);
        m_GeoCostList.insert(splitDir, 0, mergeCand, (double)sadLarge + (double)bitsCand * sqrtLambdaForFirstPass);
        sadSmall = sadWholeBlk[mergeCand] - sadLarge;
        m_GeoCostList.insert(splitDir, 1, mergeCand, (double)sadSmall + (double)bitsCand * sqrtLambdaForFirstPass);
      }
    }

    for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
    {
      for (int GeoMotionIdx = 0; GeoMotionIdx < maxNumMergeCandidates * (maxNumMergeCandidates - 1); GeoMotionIdx++)
      {
        unsigned int mergeCand0 = m_GeoModeTest[GeoMotionIdx].m_candIdx0;
        unsigned int mergeCand1 = m_GeoModeTest[GeoMotionIdx].m_candIdx1;
        double tempCost = m_GeoCostList.singleDistList[0][splitDir][mergeCand0].cost + m_GeoCostList.singleDistList[1][splitDir][mergeCand1].cost;
        if( tempCost > bestWholeBlkCost )
        {
          continue;
        }
        tempCost = tempCost + (double)bitsCandTB * sqrtLambdaForFirstPass;
        comboList.list.push_back(GeoMergeCombo(splitDir, mergeCand0, mergeCand1, tempCost));
      }
    }
    if (!comboList.list.empty())
    {
      comboList.sortByCost();
      int geoNumCobo = (int)comboList.list.size();
      const int numGeoSATD = min(geoNumCobo, GEO_MAX_TRY_WEIGHTED_SAD);
      double bestPrevCost = candCostList.size() > 0 ? candCostList[0] : MAX_DOUBLE;
      for (uint8_t candidateIdx = 0; candidateIdx < numGeoSATD; candidateIdx++)
      {
        int splitDir = comboList.list[candidateIdx].splitDir;
        int mergeCand0 = comboList.list[candidateIdx].mergeIdx0;
        int mergeCand1 = comboList.list[candidateIdx].mergeIdx1;

        pu.geoSplitDir = comboList.list[candidateIdx].splitDir;
        pu.geoMergeIdx0 = comboList.list[candidateIdx].mergeIdx0;
        pu.geoMergeIdx1 = comboList.list[candidateIdx].mergeIdx1;

        m_pcInterSearch->weightedGeoBlk(pu, splitDir, CHANNEL_TYPE_LUMA, *singleMergeTempBuffer, geoBuffer[mergeCand0], geoBuffer[mergeCand1]);

        distParam.cur = singleMergeTempBuffer->Y();
        Distortion uiSad = distParam.distFunc(distParam);

        m_CABACEstimator->getCtx() = ctxStart;
        uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
        double cost = (double)uiSad + (double)fracBits * sqrtLambdaForFirstPassIntra;

        if (cost > bestPrevCost) // skip GEO candidate if cost larger than the best from regular and affine as in original design
        {
          continue;
        }
        numGeoChecked++;
        insertPos = -1;
        updateCandList(ModeInfo(cu, pu), cost, rdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
#if MERGE_ENC_OPT
        if (insertPos != -1 && insertPos < MMVD_MRG_MAX_RD_NUM)
#else
        if (insertPos != -1)
#endif
        {
          m_pcInterSearch->weightedGeoBlk(pu, splitDir, CHANNEL_TYPE_CHROMA, *singleMergeTempBuffer, geoBuffer[pu.geoMergeIdx0], geoBuffer[pu.geoMergeIdx1]); //have to use pu.geoMergeIdx1 since  mergeCand1 maybe changed
          for (int i = int(rdModeList.size()) - 1; i > insertPos; i--)
          {
            swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
          }
          swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
        }
      }
    }
  }

  cu.geoFlag = false;
  if( numGeoChecked < GEO_MAX_TRY_WEIGHTED_SATD ) // try to match the original number of full RD
  {
    uiNumMrgSATDCand = uiNumMrgSATDCand - GEO_MAX_TRY_WEIGHTED_SATD + numGeoChecked;
  }

  if (uiNumMrgSATDCand > rdModeList.size()) //to make sure we have engough candidates in the list
  {
    uiNumMrgSATDCand = (unsigned int)rdModeList.size();
  }
}
#endif
#else
void EncCu::xCheckRDCostAffineMerge2Nx2N( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
{
  if( m_modeCtrl->getFastDeltaQp() )
  {
    return;
  }

  if ( bestCS->area.lumaSize().width < 8 || bestCS->area.lumaSize().height < 8 )
  {
    return;
  }
  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;
  const Slice &slice = *tempCS->slice;

  CHECK( slice.getSliceType() == I_SLICE, "Affine Merge modes not available for I-slices" );

  tempCS->initStructData( encTestMode.qp );

  AffineMergeCtx affineMergeCtx;
  const SPS &sps = *tempCS->sps;
  if (sps.getMaxNumAffineMergeCand() == 0)
  {
    return;
  }

  setAFFBestSATDCost(MAX_DOUBLE);

  MergeCtx mrgCtx;
  if (sps.getSbTMVPEnabledFlag())
  {
    Size bufSize = g_miScaling.scale( tempCS->area.lumaSize() );
    mrgCtx.subPuMvpMiBuf = MotionBuf( m_subPuMiBuf, bufSize );
    affineMergeCtx.mrgCtx = &mrgCtx;
  }

  {
    // first get merge candidates
    CodingUnit cu( tempCS->area );
    cu.cs = tempCS;
    cu.predMode = MODE_INTER;
    cu.slice = tempCS->slice;
    cu.tileIdx          = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
    cu.mmvdSkip = false;
#if INTER_LIC
    cu.licFlag = false;
#endif

    PredictionUnit pu( tempCS->area );
    pu.cu = &cu;
    pu.cs = tempCS;
    pu.regularMergeFlag = false;
    PU::getAffineMergeCand( pu, affineMergeCtx 
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION && JVET_W0090_ARMC_TM
      ,m_pcInterPred
#endif
    );
#if JVET_W0090_ARMC_TM
    if (sps.getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
      )
    {
      m_pcInterSearch->adjustAffineMergeCandidates(pu, affineMergeCtx);
    }
#endif

    if ( affineMergeCtx.numValidMergeCand <= 0 )
    {
      return;
    }
  }

  bool candHasNoResidual[AFFINE_MRG_MAX_NUM_CANDS];
  for ( uint32_t ui = 0; ui < affineMergeCtx.numValidMergeCand; ui++ )
  {
    candHasNoResidual[ui] = false;
  }

  bool                                        bestIsSkip = false;
  uint32_t                                    uiNumMrgSATDCand = affineMergeCtx.numValidMergeCand;
  PelUnitBuf                                  acMergeBuffer[AFFINE_MRG_MAX_NUM_CANDS];
  static_vector<uint32_t, AFFINE_MRG_MAX_NUM_CANDS>  rdModeList;
  bool                                        mrgTempBufSet = false;

  for ( uint32_t i = 0; i < AFFINE_MRG_MAX_NUM_CANDS; i++ )
  {
    rdModeList.push_back( i );
  }

  if ( m_pcEncCfg->getUseFastMerge() )
  {
    uiNumMrgSATDCand = std::min( NUM_AFF_MRG_SATD_CAND, affineMergeCtx.numValidMergeCand );
    bestIsSkip = false;

    if ( auto blkCache = dynamic_cast<CacheBlkInfoCtrl*>(m_modeCtrl) )
    {
      bestIsSkip = blkCache->isSkip( tempCS->area );
    }

    static_vector<double, AFFINE_MRG_MAX_NUM_CANDS> candCostList;

    // 1. Pass: get SATD-cost for selected candidates and reduce their count
    if ( !bestIsSkip )
    {
      rdModeList.clear();
      mrgTempBufSet = true;
#if JVET_W0097_GPM_MMVD_TM
      const double sqrtLambdaForFirstPassIntra = m_pcRdCost->getMotionLambda() * FRAC_BITS_SCALE;
      const TempCtx ctxStart(m_ctxCache, m_CABACEstimator->getCtx());
#else
      const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda( );
#endif

      CodingUnit &cu = tempCS->addCU( tempCS->area, partitioner.chType );

      partitioner.setCUData( cu );
      cu.slice = tempCS->slice;
      cu.tileIdx          = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
      cu.skip = false;
      cu.affine = true;
#if INTER_LIC
      cu.licFlag = false;
#endif
      cu.predMode = MODE_INTER;
      cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
      cu.qp = encTestMode.qp;

      PredictionUnit &pu = tempCS->addPU( cu, partitioner.chType );

      DistParam distParam;
      const bool bUseHadamard = !tempCS->slice->getDisableSATDForRD();
      m_pcRdCost->setDistParam( distParam, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y(), sps.getBitDepth( CHANNEL_TYPE_LUMA ), COMPONENT_Y, bUseHadamard );

      const UnitArea localUnitArea( tempCS->area.chromaFormat, Area( 0, 0, tempCS->area.Y().width, tempCS->area.Y().height ) );
#if MULTI_HYP_PRED
      const bool testMHP = tempCS->sps->getUseInterMultiHyp()
        && (tempCS->area.lumaSize().area() > MULTI_HYP_PRED_RESTRICT_BLOCK_SIZE 
        && std::min(tempCS->area.lwidth(), tempCS->area.lheight()) >= MULTI_HYP_PRED_RESTRICT_MIN_WH);
#endif

      for ( uint32_t uiMergeCand = 0; uiMergeCand < affineMergeCtx.numValidMergeCand; uiMergeCand++ )
      {
        acMergeBuffer[uiMergeCand] = m_acMergeBuffer[uiMergeCand].getBuf( localUnitArea );

        // set merge information
        pu.interDir = affineMergeCtx.interDirNeighbours[uiMergeCand];
        pu.mergeFlag = true;
        pu.regularMergeFlag = false;
        pu.mergeIdx = uiMergeCand;
        cu.affineType = affineMergeCtx.affineType[uiMergeCand];
#if AFFINE_MMVD
        pu.afMmvdFlag = false;
#endif
        cu.bcwIdx = affineMergeCtx.bcwIdx[uiMergeCand];
#if INTER_LIC
        cu.licFlag = affineMergeCtx.licFlags[uiMergeCand];
#endif

        pu.mergeType = affineMergeCtx.mergeType[uiMergeCand];
        if ( pu.mergeType == MRG_TYPE_SUBPU_ATMVP )
        {
          pu.refIdx[0] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0][0].refIdx;
          pu.refIdx[1] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1][0].refIdx;
          PU::spanMotionInfo(pu, mrgCtx
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
            , pu.colIdx
#endif
          );
        }
        else
        {
          PU::setAllAffineMvField( pu, affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0], REF_PIC_LIST_0 );
          PU::setAllAffineMvField( pu, affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1], REF_PIC_LIST_1 );

          PU::spanMotionInfo( pu );
        }

        distParam.cur = acMergeBuffer[uiMergeCand].Y();

        m_pcInterSearch->motionCompensation( pu, acMergeBuffer[uiMergeCand], REF_PIC_LIST_X, true, false );

        Distortion uiSad = distParam.distFunc( distParam );
#if JVET_W0097_GPM_MMVD_TM
        m_CABACEstimator->getCtx() = ctxStart;
        uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
        double cost = (double)uiSad + (double)fracBits * sqrtLambdaForFirstPassIntra;
#else
        uint32_t   uiBitsCand = uiMergeCand + 1;
        if ( uiMergeCand == tempCS->picHeader->getMaxNumAffineMergeCand() - 1 )
        {
          uiBitsCand--;
        }
        double cost = (double)uiSad + (double)uiBitsCand * sqrtLambdaForFirstPass;
#endif
#if MULTI_HYP_PRED
        if (testMHP && pu.addHypData.size() < tempCS->sps->getMaxNumAddHyps())
        {
          uiBitsCand = uiBitsCand + 1 + 1; // one bit for merge flag, and one bit for subblock_merge_flag
          MEResult mergeResult;
          mergeResult.cu = cu;
          mergeResult.pu = pu;
          mergeResult.bits = uiBitsCand;
          mergeResult.cost = uiSad + m_pcRdCost->getCost(uiBitsCand);
          m_baseResultsForMH.push_back(mergeResult);
        }
#endif
        updateCandList( uiMergeCand, cost, rdModeList, candCostList
          , uiNumMrgSATDCand );

        CHECK( std::min( uiMergeCand + 1, uiNumMrgSATDCand ) != rdModeList.size(), "" );
      }

      // Try to limit number of candidates using SATD-costs
      for ( uint32_t i = 1; i < uiNumMrgSATDCand; i++ )
      {
        if ( candCostList[i] > MRG_FAST_RATIO * candCostList[0] )
        {
          uiNumMrgSATDCand = i;
          break;
        }
      }

      tempCS->initStructData( encTestMode.qp );
      setAFFBestSATDCost(candCostList[0]);
#if JVET_W0097_GPM_MMVD_TM
      m_CABACEstimator->getCtx() = ctxStart;
#endif
    }
    else
    {
      uiNumMrgSATDCand = affineMergeCtx.numValidMergeCand;
    }
  }

  uint32_t iteration;
  uint32_t iterationBegin = 0;
  iteration = 2;
  for (uint32_t uiNoResidualPass = iterationBegin; uiNoResidualPass < iteration; ++uiNoResidualPass)
  {
    for ( uint32_t uiMrgHADIdx = 0; uiMrgHADIdx < uiNumMrgSATDCand; uiMrgHADIdx++ )
    {
      uint32_t uiMergeCand = rdModeList[uiMrgHADIdx];

      if ( ((uiNoResidualPass != 0) && candHasNoResidual[uiMergeCand])
        || ((uiNoResidualPass == 0) && bestIsSkip) )
      {
        continue;
      }

      // first get merge candidates
      CodingUnit &cu = tempCS->addCU( tempCS->area, partitioner.chType );

      partitioner.setCUData( cu );
      cu.slice = tempCS->slice;
      cu.tileIdx          = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
#if INTER_LIC
      cu.licFlag = false;
#endif
      cu.skip = false;
      cu.affine = true;
      cu.predMode = MODE_INTER;
      cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
      cu.qp = encTestMode.qp;
      PredictionUnit &pu = tempCS->addPU( cu, partitioner.chType );

      // set merge information
      pu.mergeFlag = true;
      pu.mergeIdx = uiMergeCand;
      pu.interDir = affineMergeCtx.interDirNeighbours[uiMergeCand];
      cu.affineType = affineMergeCtx.affineType[uiMergeCand];
#if AFFINE_MMVD
      pu.afMmvdFlag = false;
#endif
      cu.bcwIdx = affineMergeCtx.bcwIdx[uiMergeCand];
#if INTER_LIC
      cu.licFlag = affineMergeCtx.licFlags[uiMergeCand];
#endif

      pu.mergeType = affineMergeCtx.mergeType[uiMergeCand];
      if ( pu.mergeType == MRG_TYPE_SUBPU_ATMVP )
      {
        pu.refIdx[0] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0][0].refIdx;
        pu.refIdx[1] = affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1][0].refIdx;
        PU::spanMotionInfo( pu, mrgCtx );
      }
      else
      {
        PU::setAllAffineMvField( pu, affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 0], REF_PIC_LIST_0 );
        PU::setAllAffineMvField( pu, affineMergeCtx.mvFieldNeighbours[(uiMergeCand << 1) + 1], REF_PIC_LIST_1 );

        PU::spanMotionInfo( pu );
      }

      if( m_pcEncCfg->getMCTSEncConstraint() && ( !( MCTSHelper::checkMvBufferForMCTSConstraint( *cu.firstPU ) ) ) )
      {
        // Do not use this mode
        tempCS->initStructData( encTestMode.qp );
        return;
      }
      if ( mrgTempBufSet )
      {
        tempCS->getPredBuf().copyFrom(acMergeBuffer[uiMergeCand], true, false);   // Copy Luma Only
        m_pcInterSearch->motionCompensation(pu, REF_PIC_LIST_X, false, true);
      }
      else
      {
        m_pcInterSearch->motionCompensation( pu );
      }
#if ENABLE_OBMC
      cu.isobmcMC = true;
      m_pcInterSearch->subBlockOBMC(*cu.firstPU);
      cu.isobmcMC = false;
#endif
      xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, uiNoResidualPass, ( uiNoResidualPass == 0 ? &candHasNoResidual[uiMergeCand] : NULL ) );

      if ( m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip )
      {
        bestIsSkip = bestCS->getCU( partitioner.chType )->rootCbf == 0;
      }
      tempCS->initStructData( encTestMode.qp );
    }// end loop uiMrgHADIdx

    if ( uiNoResidualPass == 0 && m_pcEncCfg->getUseEarlySkipDetection() )
    {
      const CodingUnit     &bestCU = *bestCS->getCU( partitioner.chType );
      const PredictionUnit &bestPU = *bestCS->getPU( partitioner.chType );

      if ( bestCU.rootCbf == 0 )
      {
        if ( bestPU.mergeFlag )
        {
          m_modeCtrl->setEarlySkipDetected();
        }
        else if ( m_pcEncCfg->getMotionEstimationSearchMethod() != MESEARCH_SELECTIVE )
        {
          int absolute_MV = 0;

          for ( uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
          {
            if ( slice.getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
            {
              absolute_MV += bestPU.mvd[uiRefListIdx].getAbsHor() + bestPU.mvd[uiRefListIdx].getAbsVer();
            }
          }

          if ( absolute_MV == 0 )
          {
            m_modeCtrl->setEarlySkipDetected();
          }
        }
      }
    }
  }
  if ( m_bestModeUpdated && bestCS->cost != MAX_DOUBLE )
  {
    xCalDebCost( *bestCS, partitioner );
  }
}
#endif

#if AFFINE_MMVD && !MERGE_ENC_OPT
void EncCu::xCheckRDCostAffineMmvd2Nx2N(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  if (m_modeCtrl->getFastDeltaQp())
  {
    return;
  }
  if (bestCS->area.lumaSize().width < 8 || bestCS->area.lumaSize().height < 8)
  {
    return;
  }
  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;
  const Slice &slice = *tempCS->slice;

  CHECK(slice.getSliceType() == I_SLICE, "Affine MMVD modes not available for I-slices");

  tempCS->initStructData(encTestMode.qp);

  AffineMergeCtx affineMergeCtx;
  const SPS &sps = *tempCS->sps;
  MergeCtx mrgCtx;
  if (sps.getSbTMVPEnabledFlag())
  {
    Size bufSize = g_miScaling.scale(tempCS->area.lumaSize());
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
    for (int i = 0; i < SUB_TMVP_NUM; i++)
    {
      mergeCtx.subPuMvpMiBuf[i] = MotionBuf(m_subPuMiBuf[i], bufSize);
    }
#else
    mergeCtx.subPuMvpMiBuf = MotionBuf(m_subPuMiBuf, bufSize);
#endif
    affineMergeCtx.mrgCtx = &mrgCtx;
  }

  {
    // first get merge candidates
    CodingUnit cu(tempCS->area);
    cu.cs       = tempCS;
    cu.predMode = MODE_INTER;
    cu.slice    = tempCS->slice;
    cu.tileIdx  = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
    cu.mmvdSkip = false;
    cu.geoFlag  = false;
#if INTER_LIC
    cu.licFlag  = false;
#endif

    PredictionUnit pu(tempCS->area);
    pu.cu = &cu;
    pu.cs = tempCS;

    PU::getAffineMergeCand(pu, affineMergeCtx);
  }
  int baseIdxToMergeIdxOffset = (int)PU::getMergeIdxFromAfMmvdBaseIdx(affineMergeCtx, 0);
  int baseCount = std::min<int>((int)AF_MMVD_BASE_NUM, affineMergeCtx.numValidMergeCand - baseIdxToMergeIdxOffset);
  if (baseCount < 1)
  {
    return;
  }

  bool candHasNoResidual[AF_MMVD_NUM];
  for (uint32_t ui = 0; ui < AF_MMVD_NUM; ui++)
  {
    candHasNoResidual[ui] = false;
  }

  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));

  bool                                    bestIsSkip       = false;
  int                                     afMmvdCandCount  = baseCount * AF_MMVD_MAX_REFINE_NUM;
  uint32_t                                uiNumMrgSATDCand = std::min( AF_MMVD_NUM, afMmvdCandCount );
  static_vector<uint32_t, AF_MMVD_NUM>    rdModeList;

  for (uint32_t i = 0; i < AF_MMVD_NUM; i++)
  {
    rdModeList.push_back(i);
  }

  if (m_pcEncCfg->getUseFastMerge())
  {
    uiNumMrgSATDCand = std::min((uint32_t)NUM_AF_MMVD_SATD_CAND, uiNumMrgSATDCand);
    bestIsSkip = false;
    if (auto blkCache = dynamic_cast<CacheBlkInfoCtrl*>(m_modeCtrl))
    {
      bestIsSkip = blkCache->isSkip(tempCS->area);
    }

    static_vector<double, AF_MMVD_NUM> candCostList;

    // 1. Pass: get SATD-cost for selected candidates and reduce their count
    if (!bestIsSkip)
    {
      rdModeList.clear();
      const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda();

      CodingUnit &cu = tempCS->addCU(tempCS->area, partitioner.chType);

      partitioner.setCUData(cu);
      cu.slice       = tempCS->slice;
      cu.tileIdx     = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
      cu.skip        = false;
      cu.affine      = true;
#if INTER_LIC
      cu.licFlag     = false;
#endif
      cu.predMode    = MODE_INTER;
      cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
      cu.qp          = encTestMode.qp;

      PredictionUnit &pu = tempCS->addPU(cu, partitioner.chType);

#if MULTI_HYP_PRED
      const bool testMHP = tempCS->sps->getUseInterMultiHyp()
        && (tempCS->area.lumaSize().area() > MULTI_HYP_PRED_RESTRICT_BLOCK_SIZE
          && std::min(tempCS->area.lwidth(), tempCS->area.lheight()) >= MULTI_HYP_PRED_RESTRICT_MIN_WH);
#endif
      DistParam distParam;
      const bool bUseHadamard = !tempCS->slice->getDisableSATDForRD();
      m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y(), sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);

      {
        for (uint32_t uiMergeCand = 0; uiMergeCand < afMmvdCandCount; uiMergeCand++)
        {
          int baseIdx = (int)uiMergeCand / AF_MMVD_MAX_REFINE_NUM;
          int stepIdx = (int)uiMergeCand - baseIdx * AF_MMVD_MAX_REFINE_NUM;
          int dirIdx  = stepIdx % AF_MMVD_OFFSET_DIR;
              stepIdx = stepIdx / AF_MMVD_OFFSET_DIR;

          PelUnitBuf predTempBuf = m_acMergeBuffer[0].getBuf(localUnitArea);

          // Pass Affine MMVD parameters from candidate to PU
          {
            pu.mergeFlag      = true;
            pu.afMmvdFlag     = true;
            pu.afMmvdBaseIdx  = (uint8_t)baseIdx;
            pu.afMmvdDir      = (uint8_t)dirIdx;
            pu.afMmvdStep     = (uint8_t)stepIdx;
            pu.mergeIdx       = (uint8_t)(baseIdx + baseIdxToMergeIdxOffset);
            pu.mergeType      = affineMergeCtx.mergeType         [pu.mergeIdx];
            pu.interDir       = affineMergeCtx.interDirNeighbours[pu.mergeIdx];
            pu.cu->affineType = affineMergeCtx.affineType        [pu.mergeIdx];
#if INTER_LIC
            pu.cu->licFlag    = affineMergeCtx.licFlags          [pu.mergeIdx];
#endif
            pu.cu->bcwIdx     = affineMergeCtx.bcwIdx            [pu.mergeIdx];
            pu.mmvdMergeFlag  = false;
            pu.ciipFlag       = false;

            CHECK(pu.afMmvdDir >= AF_MMVD_OFFSET_DIR || pu.afMmvdStep >= AF_MMVD_STEP_NUM, "Affine MMVD dir or Affine MMVD step is out of range ");
            CHECK(pu.mergeType != MRG_TYPE_DEFAULT_N, "Affine MMVD must have non-SbTMVP base!");
          }

          MvField mvfMmvd[2][3];
          PU::getAfMmvdMvf(pu, affineMergeCtx, mvfMmvd, pu.mergeIdx, pu.afMmvdStep, pu.afMmvdDir);
          PU::setAllAffineMvField(pu, mvfMmvd[0], REF_PIC_LIST_0);
          PU::setAllAffineMvField(pu, mvfMmvd[1], REF_PIC_LIST_1);
          PU::spanMotionInfo(pu);
          pu.mmvdEncOptMode = (stepIdx > 2 ? 3 : 0);
          m_pcInterSearch->motionCompensation(pu, predTempBuf, REF_PIC_LIST_X, true, false);
          pu.mmvdEncOptMode = 0;

          distParam.cur = predTempBuf.Y();
          Distortion uiSad = distParam.distFunc(distParam);
          uint32_t   uiBitsCand = PU::getAfMmvdEstBits(pu);
          double cost = (double)uiSad + (double)uiBitsCand * sqrtLambdaForFirstPass;
#if MULTI_HYP_PRED
          if (testMHP && pu.addHypData.size() < tempCS->sps->getMaxNumAddHyps())
          {
            uint32_t uiBitsCand = baseIdx + stepIdx + 2 + 1 + 1 + 1; // one bit for merge flag,  one bit for subblock_merge_flag, and one bit for afMmvdFlag
            uiBitsCand++; // for mmvd_flag
            MEResult mergeResult;
            mergeResult.cu = cu;
            mergeResult.pu = pu;
            mergeResult.bits = uiBitsCand;
            mergeResult.cost = uiSad + m_pcRdCost->getCost(uiBitsCand);

            m_baseResultsForMH.push_back(mergeResult);
          }
#endif
          updateCandList(uiMergeCand, cost, rdModeList, candCostList, uiNumMrgSATDCand);

          CHECK(std::min(uiMergeCand + 1, uiNumMrgSATDCand) != rdModeList.size(), "");
        }
      }

      // Try to limit number of candidates using SATD-costs
      for (uint32_t i = 1; i < uiNumMrgSATDCand; i++)
      {
        if (candCostList[i] > MRG_FAST_RATIO * candCostList[0])
        {
          uiNumMrgSATDCand = i;
          break;
        }
      }

      tempCS->initStructData(encTestMode.qp);
#if !MERGE_ENC_OPT
      setAFFBestSATDCost(std::min<double>(getAFFBestSATDCost(), candCostList[0]));
#endif
    }
    else
    {
      uiNumMrgSATDCand = afMmvdCandCount;
    }
  }

  bool bBufferMcFromNoResidualPass = (uiNumMrgSATDCand <= NUM_AF_MMVD_SATD_CAND);

  // 2. Pass: check candidates using full RD test
  for (uint32_t uiNoResidualPass = 0; uiNoResidualPass < 2; ++uiNoResidualPass)
  {
    for (uint32_t uiMrgHADIdx = 0; uiMrgHADIdx < uiNumMrgSATDCand; uiMrgHADIdx++)
    {
      uint32_t uiMergeCand = rdModeList[uiMrgHADIdx];

      if (((uiNoResidualPass != 0) && candHasNoResidual[uiMergeCand])
        || ((uiNoResidualPass == 0) && bestIsSkip))
      {
        continue;
      }
      // first get merge candidates
      CodingUnit &cu = tempCS->addCU(tempCS->area, partitioner.chType);

      partitioner.setCUData(cu);
      cu.slice       = tempCS->slice;
      cu.tileIdx     = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
      cu.skip        = false;
      cu.affine      = true;
#if INTER_LIC
      cu.licFlag     = false;
#endif
      cu.predMode    = MODE_INTER;
      cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
      cu.qp          = encTestMode.qp;
      PredictionUnit &pu = tempCS->addPU(cu, partitioner.chType);

      // Pass Affine MMVD parameters from candidate to PU
      int baseIdx = (int)uiMergeCand / AF_MMVD_MAX_REFINE_NUM;
      int stepIdx = (int)uiMergeCand - baseIdx * AF_MMVD_MAX_REFINE_NUM;
      int dirIdx  = stepIdx % AF_MMVD_OFFSET_DIR;
          stepIdx = stepIdx / AF_MMVD_OFFSET_DIR;
      {
        // set merge information
        pu.mergeFlag      = true;
        pu.afMmvdFlag     = true;
        pu.afMmvdBaseIdx  = (uint8_t)baseIdx;
        pu.afMmvdDir      = (uint8_t)dirIdx;
        pu.afMmvdStep     = (uint8_t)stepIdx;
        pu.mergeIdx       = (uint8_t)(baseIdx + baseIdxToMergeIdxOffset);
        pu.mergeType      = affineMergeCtx.mergeType         [pu.mergeIdx];
#if INTER_LIC
        pu.cu->licFlag    = affineMergeCtx.licFlags          [pu.mergeIdx];
#endif
        pu.interDir       = affineMergeCtx.interDirNeighbours[pu.mergeIdx];
        pu.cu->affineType = affineMergeCtx.affineType        [pu.mergeIdx];
        pu.cu->bcwIdx     = affineMergeCtx.bcwIdx            [pu.mergeIdx];
        pu.mmvdMergeFlag  = false;
        pu.ciipFlag       = false;
      }

      MvField mvfMmvd[2][3];
      PU::getAfMmvdMvf(pu, affineMergeCtx, mvfMmvd, pu.mergeIdx, pu.afMmvdStep, pu.afMmvdDir);
      PU::setAllAffineMvField(pu, mvfMmvd[0], REF_PIC_LIST_0);
      PU::setAllAffineMvField(pu, mvfMmvd[1], REF_PIC_LIST_1);
      PU::spanMotionInfo(pu);
      if (m_pcEncCfg->getMCTSEncConstraint() && (!(MCTSHelper::checkMvBufferForMCTSConstraint(*cu.firstPU))))
      {
        // Do not use this mode
        tempCS->initStructData(encTestMode.qp);
        return;
      }

      pu.mmvdEncOptMode = 0;
      if (bBufferMcFromNoResidualPass)
      {
        PelUnitBuf predTempBuf = m_acMergeBuffer[uiMrgHADIdx].getBuf(localUnitArea);
        if (uiNoResidualPass == 0)
        {
          m_pcInterSearch->motionCompensation(pu, predTempBuf, REF_PIC_LIST_X);
        }
        pu.cs->getPredBuf(pu).copyFrom(predTempBuf);
      }
      else
      {
        m_pcInterSearch->motionCompensation(pu);
      }

      xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, uiNoResidualPass, ( uiNoResidualPass == 0 ? &candHasNoResidual[uiMergeCand] : NULL ) );

      if (m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip)
      {
        bestIsSkip = bestCS->getCU(partitioner.chType)->rootCbf == 0;
      }
      tempCS->initStructData(encTestMode.qp);
    }// end loop uiMrgHADIdx

    if (uiNoResidualPass == 0 && m_pcEncCfg->getUseEarlySkipDetection())
    {
      const CodingUnit     &bestCU = *bestCS->getCU(partitioner.chType);
      const PredictionUnit &bestPU = *bestCS->getPU(partitioner.chType);

      if (bestCU.rootCbf == 0)
      {
        if (bestPU.mergeFlag)
        {
          m_modeCtrl->setEarlySkipDetected();
        }
        else if (m_pcEncCfg->getMotionEstimationSearchMethod() != MESEARCH_SELECTIVE)
        {
          int absolute_MV = 0;

          for (uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++)
          {
            if (slice.getNumRefIdx(RefPicList(uiRefListIdx)) > 0)
            {
              absolute_MV += bestPU.mvd[uiRefListIdx].getAbsHor() + bestPU.mvd[uiRefListIdx].getAbsVer();
            }
          }

          if (absolute_MV == 0)
          {
            m_modeCtrl->setEarlySkipDetected();
          }
        }
      }
    }
  }
}
#endif

#if TM_MRG && !MERGE_ENC_OPT
void EncCu::xCheckRDCostTMMerge2Nx2N(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  const Slice &slice = *tempCS->slice;
  CHECK(slice.getSliceType() == I_SLICE, "Merge modes not available for I-slices");
  tempCS->initStructData(encTestMode.qp);

  MergeCtx mergeCtx;
  const SPS &sps = *tempCS->sps;
  if (sps.getSbTMVPEnabledFlag())
  {
    Size bufSize = g_miScaling.scale(tempCS->area.lumaSize());
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
    for (int i = 0; i < SUB_TMVP_NUM; i++)
    {
      mergeCtx.subPuMvpMiBuf[i] = MotionBuf(m_subPuMiBuf[i], bufSize);
    }
#else
    mergeCtx.subPuMvpMiBuf = MotionBuf(m_subPuMiBuf, bufSize);
#endif
  }
#if MULTI_PASS_DMVR
  bool applyBDMVR4TM[TM_MRG_MAX_NUM_CANDS] = { false };
#endif

  {
    // first get merge candidates
    CodingUnit cu( tempCS->area );
    cu.cs       = tempCS;
    cu.predMode = MODE_INTER;
    cu.slice    = tempCS->slice;
    cu.tileIdx  = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
#if INTER_LIC
    cu.licFlag  = false;
#endif

    PredictionUnit pu( tempCS->area );
    pu.cu = &cu;
    pu.cs = tempCS;
    cu.firstPU = &pu;

    pu.tmMergeFlag = true;
    PU::getInterMergeCandidates(pu, mergeCtx, 0);
#if JVET_AF0128_LIC_MERGE_TM
    if (pu.cs->sps->getUseAML()
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
      && pu.cs->sps->getTMToolsEnableFlag()
#endif
      )
    {
      m_pcInterSearch->adjustMergeCandidatesLicFlag(pu, mergeCtx);
    }
#endif
#if JVET_W0090_ARMC_TM
    if (sps.getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && sps.getTMToolsEnableFlag()
#endif
      )
    {
      m_pcInterSearch->adjustInterMergeCandidates(pu, mergeCtx);
    }
#endif

    for( uint32_t uiMergeCand = 0; uiMergeCand < mergeCtx.numValidMergeCand; uiMergeCand++ )
    {
      mergeCtx.setMergeInfo( pu, uiMergeCand );

#if MULTI_PASS_DMVR
      applyBDMVR4TM[uiMergeCand] = PU::checkBDMVRCondition(pu);
      if (applyBDMVR4TM[uiMergeCand])
      {
        pu.bdmvrRefine = true;
        m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4TM[uiMergeCand << 1], m_mvBufBDMVR4TM[(uiMergeCand << 1) + 1]);
        applyBDMVR4TM[uiMergeCand] = m_pcInterSearch->processBDMVR(pu);
      }
      else
      {
        m_pcInterSearch->deriveTMMv(pu);
      }
#else
      m_pcInterSearch->deriveTMMv( pu );
#endif

      // Store refined motion back to mergeCtx
      mergeCtx.interDirNeighbours[uiMergeCand] = pu.interDir;
      mergeCtx.bcwIdx[uiMergeCand] = pu.cu->bcwIdx;  // Bcw may change, because bi may be reduced to uni by deriveTMMv(pu)
      mergeCtx.mvFieldNeighbours[2 * uiMergeCand].setMvField( pu.mv[0], pu.refIdx[0] );
      mergeCtx.mvFieldNeighbours[2 * uiMergeCand + 1].setMvField( pu.mv[1], pu.refIdx[1] );
      if( pu.interDir == 1 )
      {
        mergeCtx.mvFieldNeighbours[2 * uiMergeCand + 1].setMvField( Mv(), NOT_VALID );
      }
      if( pu.interDir == 2 )
      {
        mergeCtx.mvFieldNeighbours[2 * uiMergeCand].setMvField( Mv(), NOT_VALID );
      }
    }
    pu.regularMergeFlag = true;
  }

  bool candHasNoResidual[TM_MRG_MAX_NUM_CANDS];
  for (uint32_t ui = 0; ui < TM_MRG_MAX_NUM_CANDS; ui++)
  {
    candHasNoResidual[ui] = false;
  }

  bool    bestIsSkip     = false;
  int32_t candNum        = std::min(TM_MRG_MAX_NUM_CANDS, mergeCtx.numValidMergeCand);
  int32_t numMrgSATDCand = candNum;
  bool    mrgTempBufSet = false;

  static_vector<uint32_t, TM_MRG_MAX_NUM_CANDS> rdModeList;
  for (uint32_t i = 0; i < TM_MRG_MAX_NUM_CANDS; i++)
  {
    rdModeList.push_back(i);
  }

  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));
  PelUnitBuf  acMergeRealBuffer[TM_MRG_MAX_NUM_CANDS];  CHECK(TM_MRG_MAX_NUM_CANDS > MMVD_MRG_MAX_RD_BUF_NUM, "TM cannot buffer a larger number of merge candidates than that of regular merge");
  for (unsigned i = 0; i < TM_MRG_MAX_NUM_CANDS; i++)
  {
    acMergeRealBuffer[i] = m_acMergeBuffer[i].getBuf(localUnitArea);
  }

  if (m_pcEncCfg->getUseFastMerge())
  {
    numMrgSATDCand = std::min(candNum, TM_MAX_NUM_SATD_CAND);
    bestIsSkip     = false;
    if (auto blkCache = dynamic_cast<CacheBlkInfoCtrl*>(m_modeCtrl))
    {
      bestIsSkip = blkCache->isSkip(tempCS->area);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      if (slice.getUseIBC())
#else
      if (slice.getSPS()->getIBCFlag())
#endif
      {
        ComprCUCtx cuECtx = m_modeCtrl->getComprCUCtx();
        bestIsSkip = bestIsSkip && cuECtx.bestCU;
      }
    }

    static_vector<double, TM_MRG_MAX_NUM_CANDS> candCostList;

    // 1. Pass: get SATD-cost for selected candidates and reduce their count
    if( !bestIsSkip )
    {
      rdModeList.clear();
      mrgTempBufSet = true;
      const TempCtx ctxStart(m_ctxCache, m_CABACEstimator->getCtx());

      CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );
      const double sqrtLambdaForFirstPassIntra = m_pcRdCost->getMotionLambda( ) * FRAC_BITS_SCALE;
      partitioner.setCUData( cu );
      cu.slice            = tempCS->slice;
      cu.tileIdx          = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
#if INTER_LIC
      cu.licFlag          = false;
#endif
      cu.skip             = false;
      cu.mmvdSkip         = false;
      cu.geoFlag          = false;
      cu.affine           = false;
      cu.predMode         = MODE_INTER;
      cu.chromaQpAdj      = m_cuChromaQpOffsetIdxPlus1;
      cu.qp               = encTestMode.qp;
    //cu.emtFlag  is set below

      PredictionUnit &pu  = tempCS->addPU( cu, partitioner.chType );
#if AFFINE_MMVD
      pu.afMmvdFlag       = false;
#endif
      pu.tmMergeFlag      = true;

      DistParam distParam;
      const bool bUseHadamard = !tempCS->slice->getDisableSATDForRD();
      m_pcRdCost->setDistParam (distParam, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y(), sps.getBitDepth (CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);

      for( uint32_t uiMergeCand = 0; uiMergeCand < candNum; uiMergeCand++ )
      {
        mergeCtx.setMergeInfo( pu, uiMergeCand );
#if MULTI_PASS_DMVR
        if (applyBDMVR4TM[uiMergeCand])
        {
          pu.bdmvrRefine = true;
          m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4TM[uiMergeCand << 1], m_mvBufBDMVR4TM[(uiMergeCand << 1) + 1]);
        }
        else
        {
          PU::spanMotionInfo(pu, mergeCtx);
        }
#else
        PU::spanMotionInfo(pu, mergeCtx);
#endif

        pu.mvRefine = false;
#if INTER_LIC
        m_pcInterSearch->m_storeBeforeLIC = false;
#endif

        m_pcInterSearch->motionCompensation(pu, acMergeRealBuffer[uiMergeCand], REF_PIC_LIST_X, true, true);

#if MULTI_PASS_DMVR
        if( pu.bdmvrRefine )
        {
          ::memcpy( m_mvBufEncBDOF4TM[uiMergeCand], m_pcInterSearch->getBdofSubPuMvOffset(), sizeof( Mv ) * BDOF_SUBPU_MAX_NUM );
          PU::spanMotionInfo( pu, mergeCtx, m_mvBufBDMVR4TM[uiMergeCand << 1], m_mvBufBDMVR4TM[( uiMergeCand << 1 ) + 1], m_mvBufEncBDOF4TM[uiMergeCand] );
        }
#endif
        distParam.cur = acMergeRealBuffer[uiMergeCand].Y();
        Distortion uiSad = distParam.distFunc(distParam);
        m_CABACEstimator->getCtx() = ctxStart;
        uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
        double cost = (double)uiSad + (double)fracBits * sqrtLambdaForFirstPassIntra;

        updateCandList(uiMergeCand, cost, rdModeList, candCostList, numMrgSATDCand);

        CHECK(std::min(uiMergeCand + 1, (uint32_t)numMrgSATDCand) != rdModeList.size(), "");
      }

      // Try to limit number of candidates using SATD-costs
      for( uint32_t i = 1; i < numMrgSATDCand; i++ )
      {
        if( candCostList[i] > MRG_FAST_RATIO * candCostList[0] )
        {
          numMrgSATDCand = i;
          break;
        }
      }

      tempCS->initStructData( encTestMode.qp );
      m_CABACEstimator->getCtx() = ctxStart;
    }
    else
    {
      numMrgSATDCand = candNum;
    }
  }

  // 2. Pass: check candidates using full RD test
  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;

  for (uint32_t uiNoResidualPass = 0; uiNoResidualPass < 2; ++uiNoResidualPass)
  {
    for( uint32_t uiMrgHADIdx = 0; uiMrgHADIdx < numMrgSATDCand; uiMrgHADIdx++ )
    {
      uint32_t uiMergeCand = rdModeList[uiMrgHADIdx];

      if (((uiNoResidualPass != 0) && candHasNoResidual[uiMrgHADIdx])
       || ( (uiNoResidualPass == 0) && bestIsSkip ) )
      {
        continue;
      }

      // first get merge candidates
      CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );

      partitioner.setCUData( cu );
      cu.slice            = tempCS->slice;
      cu.tileIdx          = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
      cu.skip             = false;
      cu.mmvdSkip         = false;
#if INTER_LIC
      cu.licFlag          = false;
#endif
      cu.affine           = false;
      cu.geoFlag          = false;
      cu.predMode         = MODE_INTER;
      cu.chromaQpAdj      = m_cuChromaQpOffsetIdxPlus1;
      cu.qp               = encTestMode.qp;

      PredictionUnit &pu  = tempCS->addPU( cu, partitioner.chType );
#if AFFINE_MMVD
      pu.afMmvdFlag       = false;
#endif
      pu.tmMergeFlag      = true;

      {
        mergeCtx.setMergeInfo(pu, uiMergeCand);
#if MULTI_PASS_DMVR
        if (applyBDMVR4TM[uiMergeCand])
        {
          pu.bdmvrRefine = true;
          m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4TM[uiMergeCand << 1], m_mvBufBDMVR4TM[(uiMergeCand << 1) + 1]);
        }
#endif
      }
#if MULTI_PASS_DMVR
      if (!pu.bdmvrRefine)
      {
        PU::spanMotionInfo(pu, mergeCtx);
      }
#else
      PU::spanMotionInfo(pu, mergeCtx);
#endif

      if( mrgTempBufSet )
      {
        tempCS->getPredBuf().copyFrom(acMergeRealBuffer[uiMergeCand]);
#if MULTI_PASS_DMVR
        if( pu.bdmvrRefine )
        {
          PU::spanMotionInfo( pu, mergeCtx, m_mvBufBDMVR4TM[uiMergeCand << 1], m_mvBufBDMVR4TM[( uiMergeCand << 1 ) + 1], m_mvBufEncBDOF4TM[uiMergeCand] );
        }
#endif
      }
      else
      {
        pu.mvRefine = false;
#if INTER_LIC
        m_pcInterSearch->m_storeBeforeLIC = false;
#endif
        m_pcInterSearch->motionCompensation( pu );
#if MULTI_PASS_DMVR
        if( pu.bdmvrRefine )
        {
          ::memcpy( m_mvBufEncBDOF4TM[uiMergeCand], m_pcInterSearch->getBdofSubPuMvOffset(), sizeof( Mv ) * BDOF_SUBPU_MAX_NUM );
          PU::spanMotionInfo( pu, mergeCtx, m_mvBufBDMVR4TM[uiMergeCand << 1], m_mvBufBDMVR4TM[( uiMergeCand << 1 ) + 1], m_mvBufEncBDOF4TM[uiMergeCand] );
        }
#endif
      }

      xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, uiNoResidualPass, uiNoResidualPass == 0 ? &candHasNoResidual[uiMrgHADIdx] : NULL );

      if( m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip )
      {
        bestIsSkip = !bestCS->cus.empty() && bestCS->getCU( partitioner.chType )->rootCbf == 0;
      }
      tempCS->initStructData( encTestMode.qp );
    }// end loop uiMrgHADIdx

    if( uiNoResidualPass == 0 && m_pcEncCfg->getUseEarlySkipDetection() )
    {
      const CodingUnit     &bestCU = *bestCS->getCU( partitioner.chType );
      const PredictionUnit &bestPU = *bestCS->getPU( partitioner.chType );

      if( bestCU.rootCbf == 0 )
      {
        if( bestPU.mergeFlag )
        {
          m_modeCtrl->setEarlySkipDetected();
        }
        else if( m_pcEncCfg->getMotionEstimationSearchMethod() != MESEARCH_SELECTIVE )
        {
          int absolute_MV = 0;

          for( uint32_t uiRefListIdx = 0; uiRefListIdx < 2; uiRefListIdx++ )
          {
            if( slice.getNumRefIdx( RefPicList( uiRefListIdx ) ) > 0 )
            {
              absolute_MV += bestPU.mvd[uiRefListIdx].getAbsHor() + bestPU.mvd[uiRefListIdx].getAbsVer();
            }
          }

          if( absolute_MV == 0 )
          {
            m_modeCtrl->setEarlySkipDetected();
          }
        }
      }
    }
  }
  if ( m_bestModeUpdated && bestCS->cost != MAX_DOUBLE )
  {
    xCalDebCost( *bestCS, partitioner );
  }
}
#endif

//////////////////////////////////////////////////////////////////////////////////////////////
// ibc merge/skip mode check
void EncCu::xCheckRDCostIBCModeMerge2Nx2N(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  assert(partitioner.chType != CHANNEL_TYPE_CHROMA); // chroma IBC is derived
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  CHECK( !tempCS->sps->getUseIbcMerge(), "IBC merge should not be chekced at encoder with IBC merge disabled" );
#endif
#if CTU_256
  if( tempCS->area.lwidth() >= 128 || tempCS->area.lheight() >= 128 ) // disable IBC mode larger than 64x64
#else
  if (tempCS->area.lwidth() == 128 || tempCS->area.lheight() == 128) // disable IBC mode larger than 64x64
#endif
  {
    return;
  }
  const SPS &sps = *tempCS->sps;

  tempCS->initStructData(encTestMode.qp);
  MergeCtx mergeCtx;
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
  MergeCtx mergeCtxTm;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  mergeCtxTm.numValidMergeCand = 0;
#endif
#endif
#if JVET_AA0061_IBC_MBVD
  MergeCtx mergeCtxTmp;
  uint32_t ibcMbvdLUT[IBC_MBVD_NUM];
  uint32_t ibcMbvdValidNum[IBC_MBVD_BASE_NUM] = { 0 };
#endif

#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_GPM
  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));
  PelUnitBuf intraPredBuf[NUM_LUMA_MODE];
  bool intraPredBufSet[NUM_LUMA_MODE] = {false, };
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && !(JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_GPM)
  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));
#endif
#if JVET_AC0112_IBC_CIIP
  bool testIbcCiip = tempCS->sps->getUseIbcCiip() && (tempCS->area.lwidth() * tempCS->area.lheight() >= 32) && tempCS->area.lwidth() <= 32 && tempCS->area.lheight() <= 32 && (tempCS->area.lx() > 0 || tempCS->area.ly() > 0);
  int wIbc = 13, wIntra = 3, shift = 4;
  PelBuf ibcCiipBuf = m_ciipBuffer[1].getBuf(localUnitArea.Y());
  int ibcCiipIntraList[IBC_CIIP_MAX_NUM_INTRA_CANDS];
  double ibcCiipBestSatdCost = MAX_DOUBLE;
  int ibcCiipBestMergeCand = 0;
  int ibcCiipBestIntraCand = 0;
#if JVET_AE0169_BIPREDICTIVE_IBC
  int ibcCiipBestIntraDir = PLANAR_IDX;
#endif
#endif
#if JVET_AC0112_IBC_GPM
  MergeCtx mergeCtxIbcGeo;
  int skipCandNum[IBC_MRG_MAX_NUM_CANDS] = {0, };
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
  MergeCtx mergeCtxTmIbcGeo;
  int skipCandNumTm[IBC_MRG_MAX_NUM_CANDS<<1] = {0, };
#endif
  DistParam distParamSad;
  Distortion sadIntraWholeBlk[NUM_LUMA_MODE];
  uint8_t ibcGpmIntraCandList[GEO_NUM_PARTITION_MODE][2][IBC_GPM_MAX_NUM_INTRA_CANDS];
#endif

  if (sps.getSbTMVPEnabledFlag())
  {
    Size bufSize = g_miScaling.scale(tempCS->area.lumaSize());
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
    for (int i = 0; i < SUB_TMVP_NUM; i++)
    {
      mergeCtx.subPuMvpMiBuf[i] = MotionBuf(m_subPuMiBuf[i], bufSize);
    }
#else
    mergeCtx.subPuMvpMiBuf = MotionBuf(m_subPuMiBuf, bufSize);
#endif
  }

  {
    // first get merge candidates
    CodingUnit cu(tempCS->area);
    cu.cs = tempCS;
    cu.predMode = MODE_IBC;
    cu.slice = tempCS->slice;
    cu.tileIdx          = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
    PredictionUnit pu(tempCS->area);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    cu.firstPU = (cu.firstPU == nullptr ? &pu : cu.firstPU);
#endif
    pu.cu = &cu;
    pu.cs = tempCS;
    cu.mmvdSkip = false;
    pu.mmvdMergeFlag = false;
    pu.regularMergeFlag = false;
#if INTER_LIC
    cu.licFlag = false;
#endif
    cu.geoFlag = false;
#if JVET_AC0112_IBC_LIC
    cu.ibcLicFlag = false;
#if JVET_AE0078_IBC_LIC_EXTENSION
    cu.ibcLicIdx = 0;
#endif
#endif
#if JVET_AE0159_FIBC
    cu.ibcFilterFlag = false;
#endif
#if JVET_AA0070_RRIBC
    cu.rribcFlipType = 0;
    pu.mergeFlag = true;
    PU::getIBCMergeCandidates(pu, mergeCtx);
    pu.mergeFlag = false;
#else
    PU::getIBCMergeCandidates(pu, mergeCtx);
#endif
#if JVET_Y0058_IBC_LIST_MODIFY
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
    if (pu.cs->sps->getUseAML() && pu.cs->sps->getTMnoninterToolsEnableFlag())
#else
    if (pu.cs->sps->getUseAML())
#endif
    {
#if JVET_Z0075_IBC_HMVP_ENLARGE
      m_pcInterSearch->adjustIBCMergeCandidates(pu, mergeCtx, 0, IBC_MRG_MAX_NUM_CANDS_MEM);
#else
      m_pcInterSearch->adjustIBCMergeCandidates(pu, mergeCtx);
#endif
    }
#endif
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    mergeCtxTm.numValidMergeCand = 0;
    if (pu.cs->sps->getUseTMIbc())
    {
#endif
    if (pu.cs->sps->getUseDMVDMode() == true)
    {
      pu.tmMergeFlag = true;
#if JVET_AA0070_RRIBC
      pu.mergeFlag = true;
      PU::getIBCMergeCandidates(pu, mergeCtxTm);
      pu.mergeFlag = false;
#else
      PU::getIBCMergeCandidates(pu, mergeCtxTm);
#endif
#if JVET_Y0058_IBC_LIST_MODIFY
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
              if (pu.cs->sps->getUseAML() && pu.cs->sps->getTMnoninterToolsEnableFlag())
#else
              if (pu.cs->sps->getUseAML())
#endif
      {
#if JVET_Z0075_IBC_HMVP_ENLARGE
        m_pcInterSearch->adjustIBCMergeCandidates(pu, mergeCtxTm, 0, IBC_MRG_MAX_NUM_CANDS_MEM);
#else
        m_pcInterSearch->adjustIBCMergeCandidates(pu, mergeCtxTm);
#endif
      }
#endif
      pu.tmMergeFlag = false;
    }
    else
    {
      mergeCtxTm.numValidMergeCand = 0;
    }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    }
#endif
#endif
#if JVET_AA0070_RRIBC && JVET_AA0061_IBC_MBVD
    pu.ibcMbvdMergeFlag = true;
    pu.mergeFlag = true;
    PU::getIBCMergeCandidates(pu, mergeCtxTmp);
    pu.mergeFlag = false;

#if JVET_Y0058_IBC_LIST_MODIFY
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
              if (pu.cs->sps->getUseAML() && pu.cs->sps->getTMnoninterToolsEnableFlag())
#else
              if (pu.cs->sps->getUseAML())
#endif
    {
#if JVET_Z0075_IBC_HMVP_ENLARGE
      m_pcInterSearch->adjustIBCMergeCandidates(pu, mergeCtxTmp, 0, IBC_MRG_MAX_NUM_CANDS_MEM);
#else
      m_pcInterSearch->adjustIBCMergeCandidates(pu, mergeCtxTmp);
#endif
    }
#endif
    pu.ibcMbvdMergeFlag = false;
#endif
  }


#if JVET_Z0084_IBC_TM && IBC_TM_MRG
#if JVET_AA0061_IBC_MBVD
  const int numCandidates = ( IBC_MRG_MAX_NUM_CANDS << 1 ) + IBC_MBVD_NUM;
#else
  const int numCandidates = IBC_MRG_MAX_NUM_CANDS << 1;
#endif
#elif JVET_AA0061_IBC_MBVD
  const int numCandidates = MRG_MAX_NUM_CANDS + IBC_MBVD_NUM;
#else
  const int numCandidates = MRG_MAX_NUM_CANDS;
#endif

#if JVET_AA0061_IBC_MBVD
  int candHasNoResidual[numCandidates] = { 0, };
  bool bestIsSkip = false;
  int32_t numMrgSATDCand = NUM_IBC_MRG_SATD_CAND;

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (tempCS->sps->getUseTMIbc())
  {
#endif
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
  if( m_pcEncCfg->getIntraPeriod() != 1 )
  {
    numMrgSATDCand += 2;
  }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  }
#endif  
  numMrgSATDCand = std::min( numMrgSATDCand, (const int)( mergeCtx.numValidMergeCand + mergeCtxTm.numValidMergeCand ) );
#else
  numMrgSATDCand = std::min( numMrgSATDCand, (const int)( mergeCtx.numValidMergeCand ) );
#endif

#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_GPM
  static_vector<ModeIbcInfo, numCandidates>  rdModeList( numCandidates );
#else
  static_vector<unsigned, numCandidates>  rdModeList( numCandidates );
#endif
  for( unsigned i = 0; i < numCandidates; i++ )
  {
#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_GPM
    rdModeList[i].mergeCand = i;
#if JVET_AC0112_IBC_CIIP
    rdModeList[i].isCIIP = false;
#endif
#if JVET_AC0112_IBC_GPM
    rdModeList[i].isIbcGpm = false;
#endif
#else
    rdModeList[i] = i;
#endif
  }

  static_vector<double, numCandidates>  candCostList( numCandidates, MAX_DOUBLE );
#else
  int candHasNoResidual[numCandidates] = { 0, };
  bool bestIsSkip = false;
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
  unsigned numMrgSATDCand = mergeCtx.numValidMergeCand + mergeCtxTm.numValidMergeCand;
#else
  unsigned numMrgSATDCand = mergeCtx.numValidMergeCand;
#endif

#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_GPM
  static_vector<ModeIbcInfo, numCandidates + 2>  rdModeList( numCandidates + 2 );
  for( unsigned i = 0; i < numCandidates + 2; i++ )
  {
    rdModeList[i].mergeCand = i;
#if JVET_AC0112_IBC_CIIP
    rdModeList[i].isCIIP = false;
#endif
#if JVET_AC0112_IBC_GPM
    rdModeList[i].isIbcGpm = false;
#endif
  }

  static_vector<double, numCandidates + 2>  candCostList( numCandidates + 2, MAX_DOUBLE );
#else
  static_vector<unsigned, numCandidates>  rdModeList( numCandidates );
  for( unsigned i = 0; i < numCandidates; i++ )
  {
    rdModeList[i] = i;
  }

  static_vector<double, numCandidates>  candCostList( numCandidates, MAX_DOUBLE );
#endif
#endif

#if JVET_AC0112_IBC_GPM
  Distortion sadWholeBlk[IBC_MRG_MAX_NUM_CANDS<<1];
  bool isSkipThisCand[IBC_GPM_MAX_NUM_UNI_CANDS<<1] = {false, };
  PelUnitBuf ibcPredBuf[IBC_MRG_MAX_NUM_CANDS<<1];
  static_vector<int, IBC_MRG_MAX_NUM_CANDS<<1> mergeCandList0[GEO_NUM_PARTITION_MODE];
  static_vector<int, IBC_MRG_MAX_NUM_CANDS<<1> mergeCandList1[GEO_NUM_PARTITION_MODE];
  static_vector<double, IBC_MRG_MAX_NUM_CANDS<<1> sadCostList0[GEO_NUM_PARTITION_MODE];
  static_vector<double, IBC_MRG_MAX_NUM_CANDS<<1> sadCostList1[GEO_NUM_PARTITION_MODE];
  int ibcGpmCandNumValid = 0;
  PelUnitBuf geoCombinations[IBC_GPM_MAX_TRY_WEIGHTED_SAD*IBC_GPM_NUM_BLENDING+1];
  static_vector<int, IBC_GPM_MAX_NUM_INTRA_CANDS> intraCandList0[GEO_NUM_PARTITION_MODE];
  static_vector<int, IBC_GPM_MAX_NUM_INTRA_CANDS> intraCandList1[GEO_NUM_PARTITION_MODE];
  static_vector<double, IBC_GPM_MAX_NUM_INTRA_CANDS> intraSadCostList0[GEO_NUM_PARTITION_MODE];
  static_vector<double, IBC_GPM_MAX_NUM_INTRA_CANDS> intraSadCostList1[GEO_NUM_PARTITION_MODE];
  bool testIbcGpm = tempCS->sps->getUseIbcGpm() && (tempCS->area.lx() > 0 || tempCS->area.ly() > 0);
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  bool testIbcBi = (tempCS->slice->getBiPredictionIBCFlag() && tempCS->sps->getMaxNumIBCMergeCand() > 1);
#endif

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  bool encOptMC = tempCS->sps->getIBCFracFlag();
#if JVET_AC0112_IBC_LIC
  m_pcInterSearch->m_storeBeforeLIC = false;
#endif
#if JVET_AC0112_IBC_LIC && (JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_GPM)
  m_pcInterSearch->m_predictionBeforeLIC = m_acMergeTmpBuffer[MRG_MAX_NUM_CANDS].getBuf(localUnitArea);
#endif
  auto isFracBv = [&tempCS](Mv mv)
  {
    return tempCS->sps->getIBCFracFlag()
       && ((mv.hor & ((1 << MV_FRACTIONAL_BITS_INTERNAL) - 1)) != 0 || (mv.ver & ((1 << MV_FRACTIONAL_BITS_INTERNAL) - 1)) != 0);
  };
#endif

    // 1. Pass: get SATD-cost for selected candidates and reduce their count
    {
      const double sqrtLambdaForFirstPass = m_pcRdCost->getMotionLambda( );

      CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, (const ChannelType)partitioner.chType), (const ChannelType)partitioner.chType);

      partitioner.setCUData(cu);
      cu.slice = tempCS->slice;
      cu.tileIdx = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
      cu.skip = false;
      cu.predMode = MODE_IBC;
      cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
      cu.qp = encTestMode.qp;
      cu.mmvdSkip = false;
#if INTER_LIC
      cu.licFlag = false;
#endif
#if JVET_AC0112_IBC_LIC
      cu.ibcLicFlag = false;
#if JVET_AE0078_IBC_LIC_EXTENSION
      cu.ibcLicIdx = 0;
#endif
#endif
#if JVET_AE0159_FIBC
      cu.ibcFilterFlag = false;
#endif
      cu.geoFlag = false;

      PredictionUnit &pu = tempCS->addPU(cu, partitioner.chType); //tempCS->addPU(cu);
      pu.mmvdMergeFlag = false;
      pu.regularMergeFlag = false;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AC0112_IBC_CIIP
      pu.ibcCiipFlag = false;
#endif
#if JVET_AC0112_IBC_GPM
      pu.ibcGpmFlag = false;
#endif
#endif

      DistParam distParam;
      const bool bUseHadamard = !cu.slice->getDisableSATDForRD();
      Picture* refPic = pu.cu->slice->getPic();
      const CPelBuf refBuf = refPic->getRecoBuf(pu.blocks[COMPONENT_Y]);
      const Pel*        piRefSrch = refBuf.buf;
#if JVET_AC0112_IBC_LIC || JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      PelBuf predBuf = tempCS->getPredBuf(pu.Y());
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      PelUnitBuf predUnitBuf = pu.cs->getPredBuf( pu );
#endif
#endif
#if JVET_AA0070_RRIBC
      pu.cu->rribcFlipType = 0;
      const CompArea &area = cu.blocks[COMPONENT_Y];
      CompArea        tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
      PelBuf          tmpOrgLuma = m_tmpStorageLCU->getBuf(tmpArea);
      tmpOrgLuma.copyFrom(tempCS->getOrgBuf().Y());
      if (tempCS->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
      {
        tmpOrgLuma.rspSignal(tempCS->getOrgBuf().Y(), m_pcReshape->getFwdLUT());
      }
      PelStorage m_tmpStorageCUflipH;
      m_tmpStorageCUflipH.create(UnitArea(pu.chromaFormat, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
      PelBuf tmpOrgLumaFlipH = m_tmpStorageCUflipH.getBuf(tmpArea);
      tmpOrgLumaFlipH.copyFrom(tmpOrgLuma);
      tmpOrgLumaFlipH.flipSignal(true);

      PelStorage m_tmpStorageCUflipV;
      m_tmpStorageCUflipV.create(UnitArea(pu.chromaFormat, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
      PelBuf tmpOrgLumaFlipV = m_tmpStorageCUflipV.getBuf(tmpArea);
      tmpOrgLumaFlipV.copyFrom(tmpOrgLuma);
      tmpOrgLumaFlipV.flipSignal(false);
#else
      if (tempCS->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
      {
        const CompArea &area = cu.blocks[COMPONENT_Y];
        CompArea    tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
        PelBuf tmpLuma = m_tmpStorageLCU->getBuf(tmpArea);
        tmpLuma.rspSignal( tempCS->getOrgBuf().Y(), m_pcReshape->getFwdLUT() );
        m_pcRdCost->setDistParam(distParam, tmpLuma, refBuf, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);
      }
      else
      {
        m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), refBuf, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);
      }
#endif

#if JVET_AC0112_IBC_GPM
      int dimdMode = -1;
      int timdMode = -1;
#endif
#if JVET_AC0112_IBC_CIIP
      if (testIbcCiip)
      {
        IntraPrediction::deriveDimdMode(tempCS->picture->getRecoBuf(tempCS->area.Y()), tempCS->area.Y(), cu);
        cu.timdMode = m_pcIntraSearch->deriveTimdMode(tempCS->picture->getRecoBuf(cu.Y()), cu.Y(), cu);
        ibcCiipIntraList[0] = MAP131TO67(cu.timdMode);
#if JVET_AC0112_IBC_GPM
        dimdMode = cu.dimdMode;
        timdMode = cu.timdMode;
#endif
        pu.ibcCiipFlag = true;
        ibcCiipIntraList[1] = HOR_IDX;
        pu.intraDir[0] = ibcCiipIntraList[0];
        m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Y(), true);
        m_pcIntraSearch->initPredIntraParams(pu, pu.Y(), *pu.cs->sps, 0);
        intraPredBuf[pu.intraDir[0]] = m_acMergeBuffer[pu.intraDir[0] + IBC_GPM_MAX_NUM_UNI_CANDS].getBuf(localUnitArea);
        m_pcIntraSearch->predIntraAng(COMPONENT_Y, intraPredBuf[pu.intraDir[0]].Y(), pu);
        intraPredBufSet[pu.intraDir[0]] = true;
        pu.ibcCiipFlag = false;
      }
#endif
#if JVET_AC0112_IBC_GPM
      if (pu.lwidth() < 8 || pu.lheight() < 8 || pu.lwidth() > 32 || pu.lheight() > 32)
      {
        testIbcGpm &= false;
      }
      if (testIbcGpm)
      {
#if JVET_AA0070_RRIBC
        m_pcRdCost->setDistParam(distParamSad, tmpOrgLuma, m_acMergeBuffer[0].Y().buf, m_acMergeBuffer[0].Y().stride, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y);
#else
        if (tempCS->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
        {
          const CompArea &area = cu.blocks[COMPONENT_Y];
          CompArea    tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
          PelBuf tmpLuma = m_tmpStorageLCU->getBuf(tmpArea);
          tmpLuma.rspSignal( tempCS->getOrgBuf().Y(), m_pcReshape->getFwdLUT() );
          m_pcRdCost->setDistParam(distParamSad, tmpLuma, m_acMergeBuffer[0].Y().buf, m_acMergeBuffer[0].Y().stride, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y);
        }
        else
        {
          m_pcRdCost->setDistParam(distParamSad, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y().buf, m_acMergeBuffer[0].Y().stride, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y);
        }
#endif
        if (dimdMode == -1 || timdMode == -1)
        {
          IntraPrediction::deriveDimdMode(tempCS->picture->getRecoBuf(cu.Y()), cu.Y(), cu);
          dimdMode = cu.dimdMode;
          cu.timdMode = m_pcIntraSearch->deriveTimdMode(tempCS->picture->getRecoBuf(cu.Y()), cu.Y(), cu);
          timdMode = cu.timdMode;
        }
        pu.ibcGpmFlag = true;
        m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Y(), true);
        for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
        {
          if (!g_ibcGpmSecondSetSplitDir[splitDir] && g_geoParams[splitDir][0] % 8 != 0)
          {
            continue;
          }
          for (int partIdx = 0; partIdx < 2; partIdx++)
          {
            PU::getGeoIntraMPMs(pu, ibcGpmIntraCandList[splitDir][partIdx], splitDir, g_geoTmShape[partIdx][g_geoParams[splitDir][0]]
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
                                , (splitDir == 0 && partIdx == 0)
#endif
                                );
            for (int intraIdx = 0; intraIdx < IBC_GPM_MAX_NUM_INTRA_CANDS; intraIdx++)
            {
              uint8_t intraCand = ibcGpmIntraCandList[splitDir][partIdx][intraIdx];
              if (!intraPredBufSet[intraCand])
              {
                intraPredBufSet[intraCand] = true;
                pu.intraDir[0] = intraCand;
                intraPredBuf[intraCand] = m_acMergeBuffer[intraCand + IBC_GPM_MAX_NUM_UNI_CANDS].getBuf(localUnitArea);
                m_pcIntraSearch->initPredIntraParams(pu, pu.Y(), *pu.cs->sps, 0);
                m_pcIntraSearch->predIntraAng(COMPONENT_Y, intraPredBuf[intraCand].Y(), pu);
              }
              distParamSad.cur.buf = intraPredBuf[intraCand].Y().buf;
              distParamSad.cur.stride = intraPredBuf[intraCand].Y().stride;
              sadIntraWholeBlk[intraCand] = distParamSad.distFunc(distParamSad);
            }
          }
        }
        pu.ibcGpmFlag = false;
      }
#endif

      int refStride = refBuf.stride;
#if !JVET_Y0058_IBC_LIST_MODIFY
      const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));
      const int cuPelX = pu.Y().x;
      const int cuPelY = pu.Y().y;
      const int roiWidth  = pu.lwidth();
      const int roiHeight = pu.lheight();
      const int picWidth  = pu.cs->slice->getPPS()->getPicWidthInLumaSamples();
      const int picHeight = pu.cs->slice->getPPS()->getPicHeightInLumaSamples();
      const unsigned int lcuWidth = pu.cs->slice->getSPS()->getMaxCUWidth();
#endif

#if JVET_Z0084_IBC_TM && IBC_TM_MRG
      int numValidBv = mergeCtx.numValidMergeCand + mergeCtxTm.numValidMergeCand;
#else
      int numValidBv = mergeCtx.numValidMergeCand;
#endif
#if JVET_AA0061_IBC_MBVD && !JVET_AA0070_RRIBC
      int numValidBvIBC = mergeCtx.numValidMergeCand;
#endif
      for (unsigned int mergeCand = 0; mergeCand < mergeCtx.numValidMergeCand; mergeCand++)
      {
        mergeCtx.setMergeInfo(pu, mergeCand); // set bv info in merge mode
#if JVET_AC0112_IBC_GPM
        int mergeCandIbcGpm = mergeCand - skipCandNum[mergeCand > 0 ? mergeCand - 1 : 0];
#endif

        int xPred = pu.bv.getHor();
        int yPred = pu.bv.getVer();
#if !JVET_Y0058_IBC_LIST_MODIFY  //should have already been checked at merge list construction
#if JVET_Z0084_IBC_TM
        if (!PU::searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, xPred, yPred, lcuWidth)) // not valid bv derived
#else
        if (!m_pcInterSearch->searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, xPred, yPred, lcuWidth)) // not valid bv derived
#endif
#else
        if (pu.bv == Mv(0, 0))
#endif
        {
          numValidBv--;
#if JVET_AA0061_IBC_MBVD && !JVET_AA0070_RRIBC
          numValidBvIBC--;
#endif
#if JVET_AC0112_IBC_GPM
          isSkipThisCand[mergeCandIbcGpm] = true;
          skipCandNum[mergeCand] = mergeCand > 0 ? skipCandNum[mergeCand - 1] + 1 : 1;
#endif
          continue;
        }
#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_GPM
#if JVET_AC0112_IBC_GPM
#if JVET_AA0070_RRIBC
        if (pu.cu->rribcFlipType > 0)
        {
#if !JVET_AE0169_BIPREDICTIVE_IBC||!(JVET_AE0169_GPM_IBC_IBC||JVET_AE0169_BIPREDICTIVE_IBC)
          isSkipThisCand[mergeCandIbcGpm] = true;
#endif
          skipCandNum[mergeCand] = mergeCand > 0 ? skipCandNum[mergeCand - 1] + 1 : 1;
        }
        else
#endif
        {
          skipCandNum[mergeCand] = mergeCand > 0 ? skipCandNum[mergeCand - 1] : 0;
        }
#endif
#else
        PU::spanMotionInfo(pu, mergeCtx);
#endif

#if JVET_AA0070_RRIBC
        if (pu.cu->rribcFlipType == 0)
        {
          m_pcRdCost->setDistParam(distParam, tmpOrgLuma, refBuf, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);
        }
        else if (pu.cu->rribcFlipType == 1)
        {
          m_pcRdCost->setDistParam(distParam, tmpOrgLumaFlipH, refBuf, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);
        }
        else
        {
          m_pcRdCost->setDistParam(distParam, tmpOrgLumaFlipV, refBuf, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);
        }
#endif

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
        bool foundFracBV = isFracBv(pu.mv[0]);
        if (foundFracBV)
        {
#if JVET_AC0112_IBC_LIC && (JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_GPM)
          m_pcInterSearch->m_storeBeforeLIC = pu.cu->ibcLicFlag;
#endif
          m_pcInterSearch->getPredIBCBlk(pu, COMPONENT_Y, pu.cu->slice->getPic(), pu.mv[0], predUnitBuf, encOptMC);
#if JVET_AC0112_IBC_LIC && (JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_GPM)
          m_pcInterSearch->m_storeBeforeLIC = false;
#endif
          distParam.cur = predBuf;
        }
        else
#endif
#if JVET_AC0112_IBC_LIC
        if (pu.cu->ibcLicFlag)
        {
          Pel* piPred = predBuf.buf;
          int predStride = predBuf.stride;
          int height = predBuf.height;
          int width = predBuf.width;
          for (int h = 0; h < height; h++)
          {
            memcpy(piPred, piRefSrch + h * refStride + refStride * yPred + xPred, width * sizeof(Pel));
            piPred += predStride;
          }
          m_pcInterSearch->xLocalIlluComp(pu, COMPONENT_Y
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                        , pu.mv[0]
#else
                                        , pu.bv
#endif
                                        , predBuf
#if JVET_AG0136_INTRA_TMP_LIC
                                        , true
#endif
                                        );
          distParam.cur = predBuf;
        }
        else
#endif
        distParam.cur.buf = piRefSrch + refStride * yPred + xPred;

        Distortion sad = distParam.distFunc(distParam);
        unsigned int bitsCand = mergeCand + 1;
#if JVET_Z0084_IBC_TM
        if (mergeCand == tempCS->sps->getMaxNumIBCMergeCand() - 1)
#else
        if (mergeCand == tempCS->sps->getMaxNumMergeCand() - 1)
#endif
        {
          bitsCand--;
        }
#if JVET_AE0169_BIPREDICTIVE_IBC
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
        if (pu.cs->sps->getUseTMIbc() && pu.cs->sps->getUseDMVDMode())
#else
        if (pu.cs->sps->getUseDMVDMode())
#endif
        {
          bitsCand++;   // for tm_merge_flag
        }
#endif
#if JVET_AA0061_IBC_MBVD
        if (pu.cs->sps->getUseIbcMbvd())
        {
          bitsCand++;   // for ibc_mbvd_flag
        }
#endif
#if JVET_AC0112_IBC_CIIP
        if (testIbcCiip)
        {
          bitsCand++;   // for ibc_ciip_flag
        }
#endif
#if JVET_AC0112_IBC_GPM
        if (testIbcGpm)
        {
          bitsCand++;   // for ibc_gpm_flag
        }
#endif
#else
#if JVET_AA0061_IBC_MBVD
        bitsCand++; // for ibc_mbvd_flag
#endif
#endif
        double cost = (double)sad + (double)bitsCand * sqrtLambdaForFirstPass;

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && (JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_GPM)
        CPelBuf cPredIBC = distParam.cur;
#if JVET_AC0112_IBC_LIC
        if (pu.cu->ibcLicFlag)
        {
          cPredIBC.buf    = foundFracBV ? m_pcInterSearch->m_predictionBeforeLIC.Y().buf    : piRefSrch + refStride * yPred + xPred;
          cPredIBC.stride = foundFracBV ? m_pcInterSearch->m_predictionBeforeLIC.Y().stride : refStride;
        }
#endif
#endif
#if JVET_AC0112_IBC_CIIP
        if (testIbcCiip)
        {
          const PredictionUnit *puBv = pu.cs->getPURestricted(pu.lumaPos().offset(xPred, yPred), pu, pu.chType);
          int intraDir = puBv ? puBv->getIpmInfo(pu.lumaPos().offset(xPred, yPred)) : PLANAR_IDX;
          if (intraDir != ibcCiipIntraList[0])
          {
            ibcCiipIntraList[1] = intraDir;
          }
          else
          {
            ibcCiipIntraList[1] = ibcCiipIntraList[0] == PLANAR_IDX ? HOR_IDX : PLANAR_IDX;
          }
          intraDir = ibcCiipIntraList[1];
          if (!intraPredBufSet[intraDir])
          {
            pu.ibcCiipFlag = true;
            pu.intraDir[0] = intraDir;
            intraPredBufSet[intraDir] = true;
            intraPredBuf[intraDir] = m_acMergeBuffer[intraDir + IBC_GPM_MAX_NUM_UNI_CANDS].getBuf(localUnitArea);
            m_pcIntraSearch->initPredIntraParams(pu, pu.Y(), *pu.cs->sps, 0);
            m_pcIntraSearch->predIntraAng(COMPONENT_Y, intraPredBuf[intraDir].Y(), pu);
            pu.ibcCiipFlag = false;
          }
          for (int dirIdx = 0; dirIdx < IBC_CIIP_MAX_NUM_INTRA_CANDS; dirIdx++)
          {
            int height = pu.lheight();
            int width = pu.lwidth();
#if !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
            const Pel *pPredIbc = piRefSrch + refStride * yPred + xPred;
#endif
            Pel *pPredDst = ibcCiipBuf.buf;
            int dstStride = ibcCiipBuf.stride;
            Pel *pPredIntra = intraPredBuf[ibcCiipIntraList[dirIdx]].Y().buf;
            int intraStride = intraPredBuf[ibcCiipIntraList[dirIdx]].Y().stride;
            m_pcIntraSearch->m_ibcCiipBlending(pPredDst, dstStride
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                             , cPredIBC.buf, cPredIBC.stride
#else
                                             , pPredIbc, refStride
#endif
                                             , pPredIntra, intraStride, wIbc, wIntra, shift, width, height);

            distParam.cur.buf = ibcCiipBuf.buf;
            distParam.cur.stride = ibcCiipBuf.stride;

            Distortion sad2 = distParam.distFunc(distParam);
            unsigned int bitsCand2 = mergeCand + 1;
#if JVET_Z0084_IBC_TM
            if (mergeCand == tempCS->sps->getMaxNumIBCMergeCand() - 1)
#else
            if (mergeCand == tempCS->sps->getMaxNumMergeCand() - 1)
#endif
            {
              bitsCand2--;
            }
            bitsCand2 += dirIdx + 1;
            if (dirIdx == IBC_CIIP_MAX_NUM_INTRA_CANDS - 1)
            {
              bitsCand2--;
            }
#if JVET_AE0169_BIPREDICTIVE_IBC
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
            if (pu.cs->sps->getUseTMIbc() && pu.cs->sps->getUseDMVDMode())
#else
            if (pu.cs->sps->getUseDMVDMode())
#endif
            {
              bitsCand2++;   // for tm_merge_flag
            }
#endif
#if JVET_AA0061_IBC_MBVD
            if (pu.cs->sps->getUseIbcMbvd())
            {
              bitsCand2++;   // for ibc_mbvd_flag
            }
#endif
            bitsCand2++;   // for ibc_ciip_flag
#else
            bitsCand2++; // for ibc_mbvd_flag
#endif
            double cost2 = (double)sad2 + (double)bitsCand2 * sqrtLambdaForFirstPass;
            if (cost2 < ibcCiipBestSatdCost)
            {
              ibcCiipBestMergeCand = mergeCand;
#if JVET_AE0169_BIPREDICTIVE_IBC
              ibcCiipBestIntraDir = intraDir;
#endif
              ibcCiipBestIntraCand = dirIdx;
              ibcCiipBestSatdCost = cost2;
            }
          }
        }
#endif

#if JVET_AC0112_IBC_GPM
#if JVET_AA0070_RRIBC
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && JVET_AE0169_BIPREDICTIVE_IBC
        if (pu.cu->rribcFlipType == 0 && (testIbcGpm || testIbcBi))
#else
        if (pu.cu->rribcFlipType == 0 && testIbcGpm)
#endif
#else
        if (testIbcGpm)
#endif
        {
          ibcGpmCandNumValid++;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
          distParamSad.cur = cPredIBC;
#else
          distParamSad.cur.buf = piRefSrch + refStride * yPred + xPred;
          distParamSad.cur.stride = refBuf.stride;
#endif
          ibcPredBuf[mergeCandIbcGpm] = m_acMergeBuffer[mergeCandIbcGpm].getBuf(localUnitArea);
          ibcPredBuf[mergeCandIbcGpm].bufs[COMPONENT_Y].copyFrom(distParamSad.cur);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && JVET_AE0169_BIPREDICTIVE_IBC
          if (testIbcGpm)
#endif
          sadWholeBlk[mergeCandIbcGpm] = distParamSad.distFunc(distParamSad);
        }
#endif

#if JVET_AC0112_IBC_CIIP
#if JVET_AC0112_IBC_GPM
#if JVET_AE0169_BIPREDICTIVE_IBC
        updateCandList(ModeIbcInfo(mergeCand, false, 0, false, MAX_INT, MAX_INT, 0, 0, 0), cost, rdModeList, candCostList
#else
        updateCandList(ModeIbcInfo(mergeCand, false, 0, false, 0, 0, 0, 0, 0), cost, rdModeList, candCostList
#endif
         , numMrgSATDCand);
#else
        updateCandList(ModeIbcInfo(mergeCand, false, 0), cost, rdModeList, candCostList
         , numMrgSATDCand);
#endif
#else
#if JVET_AC0112_IBC_GPM
        updateCandList(ModeIbcInfo(mergeCand, false, 0, 0, 0, 0, 0), cost, rdModeList, candCostList
         , numMrgSATDCand);
#else
        updateCandList(mergeCand, cost, rdModeList, candCostList
         , numMrgSATDCand);
#endif
#endif
      }
#if JVET_AC0112_IBC_GPM
#if JVET_AE0169_BIPREDICTIVE_IBC
      if ((testIbcBi || testIbcGpm) && mergeCtx.numValidMergeCand > 0)
#else
      if (testIbcGpm && mergeCtx.numValidMergeCand > 0)
#endif
      {
        mergeCtxIbcGeo = mergeCtx;
#if JVET_AA0070_RRIBC
        m_pcInterSearch->adjustIbcMergeRribcCand(pu, mergeCtxIbcGeo, 0, IBC_MRG_MAX_NUM_CANDS_MEM, isSkipThisCand);
#endif
        for (int i = 0; i < skipCandNum[mergeCtx.numValidMergeCand - 1]; i++)
        {
          isSkipThisCand[mergeCtx.numValidMergeCand - 1 - i] = true;
        }
      }
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
      if (testIbcBi && mergeCtx.numValidMergeCand > 1)
      {
        for (int mergeCand0 = 0; mergeCand0 < mergeCtx.numValidMergeCand-1; mergeCand0++)
        {
          if (isSkipThisCand[mergeCand0])
          {
            continue;
          }
          for (int mergeCand1 = mergeCand0+1; mergeCand1 < mergeCtx.numValidMergeCand; mergeCand1++)
          {
            if (isSkipThisCand[mergeCand1])
            {
              continue;
            }

            mergeCtxIbcGeo.setMergeInfo(pu, mergeCand0); // set bv info in merge mode
            mergeCtxIbcGeo.setIbcL1Info(pu, mergeCand1);

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
            PelBuf refBufBi = predBuf;
            refBufBi.avg(ibcPredBuf[mergeCand0].Y(), ibcPredBuf[mergeCand1].Y());
#else
            Position posL0(pu.bv.getHor(), pu.bv.getVer());
            CPelBuf refBufL0 = refBuf.subBuf(posL0, refBuf);
            Mv bvL1 = pu.mv[1];
            bvL1.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
            Position posL1(bvL1.getHor(), bvL1.getVer());
            CPelBuf refBufL1 = refBuf.subBuf(posL1, refBuf);
            PelBuf refBufBi = predBuf;
            refBufBi.avg(refBufL0, refBufL1);
#endif

            m_pcRdCost->setDistParam(distParam, tmpOrgLuma, refBufBi, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);
            Distortion sad = distParam.distFunc(distParam);
            unsigned int bitsCand = mergeCand1 + 1;
#if JVET_Z0084_IBC_TM
            if (mergeCand1 == tempCS->sps->getMaxNumIBCMergeCand() - 1)
#else
            if (mergeCand1 == tempCS->sps->getMaxNumMergeCand() - 1)
#endif
            {
              bitsCand--;
            }
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
            if (pu.cs->sps->getUseTMIbc() && pu.cs->sps->getUseDMVDMode())
#else
            if (pu.cs->sps->getUseDMVDMode())
#endif
            {
              bitsCand++; // for tm_merge_flag
            }
#endif
#if JVET_AA0061_IBC_MBVD
            if (pu.cs->sps->getUseIbcMbvd())
            {
              bitsCand++; // for ibc_mbvd_flag
            }
#endif
            double cost = (double)sad + (double)bitsCand * sqrtLambdaForFirstPass;
            updateCandList(ModeIbcInfo(mergeCand0, false, 0, false, mergeCand0, mergeCand1, 0, 0, 0), cost, rdModeList, candCostList);
          }
        }
      }
#endif


#if JVET_Z0084_IBC_TM && IBC_TM_MRG
    // Add TM refined candidates
    for (unsigned int mergeCand = 0; mergeCand < mergeCtxTm.numValidMergeCand; mergeCand++)
    {
      mergeCtxTm.setMergeInfo(pu, mergeCand); // set bv info in merge mode
#if JVET_AC0112_IBC_GPM
      int mergeCandIbcGpm = mergeCand - skipCandNumTm[mergeCtx.numValidMergeCand + (mergeCand  > 0 ? mergeCand - 1 : 0)];
#endif

      Mv tempBv = 
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                  pu.cs->sps->getIBCFracFlag() ? pu.mv[0] : 
#endif
                  pu.bv;
      pu.tmMergeFlag = true;
      m_pcInterSearch->deriveTMMv(pu);
      pu.tmMergeFlag = false;

      pu.bv = pu.mv[0];
      pu.bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
      // Check if mv has been refined
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      if ((pu.cs->sps->getIBCFracFlag() && pu.mv[0] == tempBv) || (!pu.cs->sps->getIBCFracFlag() && pu.bv == tempBv))
#else
      if (pu.bv == tempBv)
#endif
      {
        numValidBv--;
        isSkipThisCand[mergeCtx.numValidMergeCand + mergeCandIbcGpm] = true;
        continue;
      }

      //Store refined result for RDO loop
      mergeCtxTm.mvFieldNeighbours[mergeCand << 1].mv = pu.mv[0];

      int xPred = pu.bv.getHor();
      int yPred = pu.bv.getVer();
#if !JVET_Y0058_IBC_LIST_MODIFY  //should have already been checked at merge list construction and during refinement
      if (!PU::searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, xPred, yPred, lcuWidth)) // not valid bv derived
#else
      if (pu.bv == Mv(0, 0))
#endif
      {
        numValidBv--;
#if JVET_AC0112_IBC_GPM
        isSkipThisCand[mergeCtx.numValidMergeCand + mergeCandIbcGpm] = true;
        skipCandNumTm[mergeCtx.numValidMergeCand + mergeCand] = mergeCand > 0 ? skipCandNumTm[mergeCtx.numValidMergeCand + mergeCand - 1] + 1 : 1;
#endif
        continue;
      }
#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_GPM
#if JVET_AC0112_IBC_GPM
#if JVET_AA0070_RRIBC
      if (pu.cu->rribcFlipType > 0)
      {
        isSkipThisCand[mergeCtx.numValidMergeCand + mergeCandIbcGpm] = true;
        skipCandNumTm[mergeCtx.numValidMergeCand + mergeCand] = mergeCand > 0 ? skipCandNumTm[mergeCtx.numValidMergeCand + mergeCand - 1] + 1 : 1;
      }
      else
#endif
      {
        skipCandNumTm[mergeCtx.numValidMergeCand + mergeCand] = mergeCand > 0 ? skipCandNumTm[mergeCtx.numValidMergeCand + mergeCand - 1] : 0;
      }
#endif
#else
      PU::spanMotionInfo(pu, mergeCtxTm
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
        , pu.colIdx
#endif
      );
#endif

#if JVET_AA0070_RRIBC
      if (pu.cu->rribcFlipType == 0)
      {
        m_pcRdCost->setDistParam(distParam, tmpOrgLuma, refBuf, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);
      }
      else if (pu.cu->rribcFlipType == 1)
      {
        m_pcRdCost->setDistParam(distParam, tmpOrgLumaFlipH, refBuf, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);
      }
      else
      {
        m_pcRdCost->setDistParam(distParam, tmpOrgLumaFlipV, refBuf, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);
      }
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      bool foundFracBV = isFracBv(pu.mv[0]);
      if (foundFracBV)
      {
#if JVET_AC0112_IBC_LIC && (JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_GPM)
        m_pcInterSearch->m_storeBeforeLIC = pu.cu->ibcLicFlag;
#endif
        m_pcInterSearch->getPredIBCBlk(pu, COMPONENT_Y, pu.cu->slice->getPic(), pu.mv[0], predUnitBuf, encOptMC);
#if JVET_AC0112_IBC_LIC && (JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_GPM)
        m_pcInterSearch->m_storeBeforeLIC = false;
#endif
        distParam.cur = predBuf;
      }
      else
#endif
#if JVET_AC0112_IBC_LIC
      if (pu.cu->ibcLicFlag)
      {
        Pel* piPred = predBuf.buf;
        int predStride = predBuf.stride;
        int height = predBuf.height;
        int width = predBuf.width;
        for (int h = 0; h < height; h++)
        {
          memcpy(piPred, piRefSrch + h * refStride + refStride * yPred + xPred, width * sizeof(Pel));
          piPred += predStride;
        }
        m_pcInterSearch->xLocalIlluComp(pu, COMPONENT_Y
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                      , pu.mv[0]
#else
                                      , pu.bv
#endif
                                      , predBuf
#if JVET_AG0136_INTRA_TMP_LIC
                                      , true
#endif
                                      );
        distParam.cur = predBuf;
      }
      else
#endif
      distParam.cur.buf = piRefSrch + refStride * yPred + xPred;

      Distortion sad = distParam.distFunc(distParam);
      unsigned int bitsCand = mergeCand + 1;
      if (mergeCand == tempCS->sps->getMaxNumIBCMergeCand() - 1)
      {
        bitsCand--;
      }
#if JVET_AE0169_BIPREDICTIVE_IBC
      bitsCand++;   // for tm_merge_flag
#if JVET_AA0061_IBC_MBVD
      if (pu.cs->sps->getUseIbcMbvd())
      {
        bitsCand++;   // for ibc_mbvd_flag
      }
#endif
#if JVET_AC0112_IBC_CIIP
      if (testIbcCiip)
      {
        bitsCand++;   // for ibc_ciip_flag
      }
#endif
#if JVET_AC0112_IBC_GPM
      if (testIbcGpm)
      {
        bitsCand++;   // for ibc_gpm_flag
      }
#endif
#else
#if JVET_AA0061_IBC_MBVD
      bitsCand++; // for ibc_mbvd_flag
#endif
#endif
      double cost = (double)sad + (double)bitsCand * sqrtLambdaForFirstPass;

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && (JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_GPM)
      CPelBuf cPredIBC = distParam.cur;
#if JVET_AC0112_IBC_LIC
      if (pu.cu->ibcLicFlag)
      {
        cPredIBC.buf    = foundFracBV ? m_pcInterSearch->m_predictionBeforeLIC.Y().buf    : piRefSrch + refStride * yPred + xPred;
        cPredIBC.stride = foundFracBV ? m_pcInterSearch->m_predictionBeforeLIC.Y().stride : refStride;
      }
#endif
#endif
#if JVET_AC0112_IBC_CIIP
      if (testIbcCiip)
      {
        const PredictionUnit *puBv = pu.cs->getPURestricted(pu.lumaPos().offset(xPred, yPred), pu, pu.chType);
        int intraDir = puBv ? puBv->getIpmInfo(pu.lumaPos().offset(xPred, yPred)) : PLANAR_IDX;
        if (intraDir != ibcCiipIntraList[0])
        {
          ibcCiipIntraList[1] = intraDir;
        }
        else
        {
          ibcCiipIntraList[1] = ibcCiipIntraList[0] == PLANAR_IDX ? HOR_IDX : PLANAR_IDX;
        }
        intraDir = ibcCiipIntraList[1];
        if (!intraPredBufSet[intraDir])
        {
          pu.ibcCiipFlag = true;
          pu.intraDir[0] = intraDir;
          intraPredBufSet[intraDir] = true;
          intraPredBuf[intraDir] = m_acMergeBuffer[intraDir + IBC_GPM_MAX_NUM_UNI_CANDS].getBuf(localUnitArea);
          m_pcIntraSearch->initPredIntraParams(pu, pu.Y(), *pu.cs->sps, 0);
          m_pcIntraSearch->predIntraAng(COMPONENT_Y, intraPredBuf[intraDir].Y(), pu);
          pu.ibcCiipFlag = false;
        }
        for (int dirIdx = 0; dirIdx < IBC_CIIP_MAX_NUM_INTRA_CANDS; dirIdx++)
        {
          int height = pu.lheight();
          int width = pu.lwidth();
#if !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
          const Pel *pPredIbc = piRefSrch + refStride * yPred + xPred;
#endif
          Pel *pPredDst = ibcCiipBuf.buf;
          int dstStride = ibcCiipBuf.stride;
          Pel *pPredIntra = intraPredBuf[ibcCiipIntraList[dirIdx]].Y().buf;
          int intraStride = intraPredBuf[ibcCiipIntraList[dirIdx]].Y().stride;
          m_pcIntraSearch->m_ibcCiipBlending(pPredDst, dstStride
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                           , cPredIBC.buf, cPredIBC.stride
#else
                                           , pPredIbc, refStride
#endif
                                           , pPredIntra, intraStride, wIbc, wIntra, shift, width, height);

          distParam.cur.buf = ibcCiipBuf.buf;
          distParam.cur.stride = ibcCiipBuf.stride;

          Distortion sad2 = distParam.distFunc(distParam);
          unsigned int bitsCand2 = mergeCand + 1;
#if JVET_Z0084_IBC_TM
          if (mergeCand == tempCS->sps->getMaxNumIBCMergeCand() - 1)
#else
          if (mergeCand == tempCS->sps->getMaxNumMergeCand() - 1)
#endif
          {
            bitsCand2--;
          }
          bitsCand2 += dirIdx + 1;
          if (dirIdx == IBC_CIIP_MAX_NUM_INTRA_CANDS - 1)
          {
            bitsCand2--;
          }
#if JVET_AE0169_BIPREDICTIVE_IBC
          bitsCand2++; // for tm_merge_flag
#if JVET_AA0061_IBC_MBVD
          if (pu.cs->sps->getUseIbcMbvd())
          {
            bitsCand2++; // for ibc_mbvd_flag
          }
#endif
          bitsCand2++; // for ibc_ciip_flag
#else
          bitsCand2++; // for ibc_mbvd_flag
#endif
          double cost2 = (double)sad2 + (double)bitsCand2 * sqrtLambdaForFirstPass;
          if (cost2 < ibcCiipBestSatdCost)
          {
            ibcCiipBestMergeCand = mergeCand+mergeCtx.numValidMergeCand;
#if JVET_AE0169_BIPREDICTIVE_IBC
            ibcCiipBestIntraDir = intraDir;
#endif
            ibcCiipBestIntraCand = dirIdx;
            ibcCiipBestSatdCost = cost2;
          }
        }
      }
#endif

#if JVET_AC0112_IBC_GPM
#if JVET_AA0070_RRIBC
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && JVET_AE0169_BIPREDICTIVE_IBC
      if (pu.cu->rribcFlipType == 0 && (testIbcGpm || testIbcBi))
#else
      if (pu.cu->rribcFlipType == 0 && testIbcGpm)
#endif
#else
      if (testIbcGpm)
#endif
      {
        ibcGpmCandNumValid++;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
        distParamSad.cur = cPredIBC;
#else
        distParamSad.cur.buf = piRefSrch + refStride * yPred + xPred;
        distParamSad.cur.stride = refBuf.stride;
#endif
        ibcPredBuf[mergeCtx.numValidMergeCand + mergeCandIbcGpm] = m_acMergeBuffer[mergeCtx.numValidMergeCand + mergeCandIbcGpm].getBuf(localUnitArea);
        ibcPredBuf[mergeCtx.numValidMergeCand + mergeCandIbcGpm].bufs[COMPONENT_Y].copyFrom(distParamSad.cur);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && JVET_AE0169_BIPREDICTIVE_IBC
        if (testIbcGpm)
#endif
        sadWholeBlk[mergeCtx.numValidMergeCand + mergeCandIbcGpm] = distParamSad.distFunc(distParamSad);
      }
#endif

#if JVET_AC0112_IBC_CIIP
#if JVET_AC0112_IBC_GPM
#if JVET_AE0169_BIPREDICTIVE_IBC
      updateCandList(ModeIbcInfo(mergeCand+mergeCtx.numValidMergeCand, false, 0, false, MAX_INT, MAX_INT, 0, 0, 0), cost, rdModeList, candCostList, numMrgSATDCand);
#else
      updateCandList(ModeIbcInfo(mergeCand+mergeCtx.numValidMergeCand, false, 0, false, 0, 0, 0, 0, 0), cost, rdModeList, candCostList, numMrgSATDCand);
#endif
#else
      updateCandList(ModeIbcInfo(mergeCand+mergeCtx.numValidMergeCand, false, 0), cost, rdModeList, candCostList, numMrgSATDCand);
#endif
#else
#if JVET_AC0112_IBC_GPM
      updateCandList(ModeIbcInfo(mergeCand+mergeCtx.numValidMergeCand, false, 0, 0, 0, 0, 0), cost, rdModeList, candCostList, numMrgSATDCand);
#else
      updateCandList(mergeCand+mergeCtx.numValidMergeCand, cost, rdModeList, candCostList, numMrgSATDCand);
#endif
#endif
    }
#if JVET_AC0112_IBC_GPM
#if JVET_AE0169_BIPREDICTIVE_IBC
    if ((testIbcBi || testIbcGpm) && mergeCtxTm.numValidMergeCand > 0)
#else
    if (testIbcGpm && mergeCtxTm.numValidMergeCand > 0)
#endif
    {
      mergeCtxTmIbcGeo = mergeCtxTm;
#if JVET_AA0070_RRIBC
      m_pcInterSearch->adjustIbcMergeRribcCand(pu, mergeCtxTmIbcGeo, 0, IBC_MRG_MAX_NUM_CANDS_MEM, isSkipThisCand + mergeCtx.numValidMergeCand);
#endif
      for (int i = 0; i < skipCandNumTm[mergeCtx.numValidMergeCand + mergeCtxTm.numValidMergeCand - 1]; i++)
      {
        isSkipThisCand[mergeCtx.numValidMergeCand + mergeCtxTm.numValidMergeCand - 1 - i] = true;
      }
    }
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (testIbcBi && mergeCtxTm.numValidMergeCand > 1)
    {
      for (int mergeCand0 = 0; mergeCand0 < mergeCtxTm.numValidMergeCand-1; mergeCand0++)
      {
        if (isSkipThisCand[mergeCtx.numValidMergeCand + mergeCand0])
        {
          continue;
        }
        for (int mergeCand1 = mergeCand0+1; mergeCand1 < mergeCtxTm.numValidMergeCand; mergeCand1++)
        {
          if (isSkipThisCand[mergeCtx.numValidMergeCand + mergeCand1])
          {
            continue;
          }

          mergeCtxTmIbcGeo.setMergeInfo(pu, mergeCand0); // set bv info in merge mode
          mergeCtxTmIbcGeo.setIbcL1Info(pu, mergeCand1);

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
          PelBuf refBufBi = predBuf;
          refBufBi.avg(ibcPredBuf[mergeCtx.numValidMergeCand + mergeCand0].Y(), ibcPredBuf[mergeCtx.numValidMergeCand + mergeCand1].Y());
#else
          Position posL0(pu.bv.getHor(), pu.bv.getVer());
          CPelBuf refBufL0 = refBuf.subBuf(posL0, refBuf);
          Mv bvL1 = pu.mv[1];
          bvL1.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
          Position posL1(bvL1.getHor(), bvL1.getVer());
          CPelBuf refBufL1 = refBuf.subBuf(posL1, refBuf);
          PelBuf refBufBi = predBuf;
          refBufBi.avg(refBufL0, refBufL1);
#endif

          m_pcRdCost->setDistParam(distParam, tmpOrgLuma, refBufBi, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);
          Distortion sad = distParam.distFunc(distParam);
          unsigned int bitsCand = mergeCand1 + 1;
#if JVET_Z0084_IBC_TM
          if (mergeCand1 == tempCS->sps->getMaxNumIBCMergeCand() - 1)
#else
          if (mergeCand1 == tempCS->sps->getMaxNumMergeCand() - 1)
#endif
          {
            bitsCand--;
          }
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
          bitsCand++; // for tm_merge_flag
#endif
#if JVET_AA0061_IBC_MBVD
          if (pu.cs->sps->getUseIbcMbvd())
          {
            bitsCand++; // for ibc_mbvd_flag
          }
#endif
          double cost = (double)sad + (double)bitsCand * sqrtLambdaForFirstPass;
          updateCandList(ModeIbcInfo(mergeCand0+mergeCtx.numValidMergeCand, false, 0, false, mergeCand0, mergeCand1, 0, 0, 0), cost, rdModeList, candCostList);
        }
      }
    }
#endif
#endif

#if JVET_AA0061_IBC_MBVD
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
    const bool mbvdAdaptiveSearch = sps.getUseIbcMbvdAdSearch();
#endif
#if JVET_AA0070_RRIBC
    int numValidBvIBC = mergeCtxTmp.numValidMergeCand;
#endif
    if (numValidBvIBC)
    {
      if (pu.cs->sps->getUseIbcMbvd())
      {
#if JVET_AA0070_RRIBC
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
        if (!mbvdAdaptiveSearch)
        {
#endif
        for (unsigned int mergeCand = 0; mergeCand < mergeCtxTmp.numValidMergeCand; mergeCand++)
        {
          mergeCtxTmp.setMergeInfo(pu, mergeCand); // set bv info in merge mode

#if !JVET_Y0058_IBC_LIST_MODIFY  //should have already been checked at merge list construction
#if JVET_Z0084_IBC_TM
          if (!PU::searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, xPred, yPred, lcuWidth)) // not valid bv derived
#else
          if (!m_pcInterSearch->searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, xPred, yPred, lcuWidth)) // not valid bv derived
#endif
#else
          if (pu.bv == Mv(0, 0))
#endif
          {
            numValidBvIBC--;
            continue;
          }
        }
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
        }
#endif
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
        const int baseNum = std::min(std::min(numValidBvIBC, IBC_MBVD_BASE_NUM), (int)tempCS->sps->getMaxNumIBCMergeCand());
        mergeCtxTmp.numValidMergeCand = baseNum;
#else
        const int baseNum = std::min(numValidBvIBC, IBC_MBVD_BASE_NUM);
#endif
#if !JVET_AA0070_RRIBC
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
        if (!mbvdAdaptiveSearch)
        {
#endif
        mergeCtxTmp = mergeCtx;
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
        }
#endif
#endif
        PU::getIbcMbvdMergeCandidates(pu, mergeCtxTmp, baseNum);

#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
        if (pu.cs->sps->getUseAML() && pu.cs->sps->getTMnoninterToolsEnableFlag())
        {
#endif
          bool flag = pu.ibcMbvdMergeFlag;
          pu.ibcMbvdMergeFlag = true;
          m_pcInterSearch->sortIbcMergeMbvdCandidates(pu, mergeCtxTmp, ibcMbvdLUT, ibcMbvdValidNum);
          pu.ibcMbvdMergeFlag = flag;
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
        }
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
        struct CheckBiSadList
        {
          int mmvdMergeCandtemp;
          Distortion sad;
        };
        CheckBiSadList mbvdSadList[IBC_MBVD_NUM_BI_CANDIDATES];
        int numMbvdSadList = 0;
        bool testIbcBiMbvd = tempCS->slice->getBiPredictionIBCFlag();
        if (testIbcBiMbvd)
        {
          for (int i = 0; i < IBC_MBVD_NUM_BI_CANDIDATES; i++)
          {
            mbvdSadList[i].mmvdMergeCandtemp = -1;
            mbvdSadList[i].sad = std::numeric_limits<Distortion>::max();
          }
        }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
        CPelBuf predBufMbvd[IBC_MBVD_SIZE_ENC*IBC_MBVD_BASE_NUM];
#endif
#endif

#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
        const int tempNum = mbvdAdaptiveSearch ? IBC_MBVD_ENC_NUM : baseNum * IBC_MBVD_MAX_REFINE_NUM;
#else
        const int tempNum = baseNum * IBC_MBVD_MAX_REFINE_NUM;
#endif
        for (unsigned int mmvdMergeCandtemp = 0; mmvdMergeCandtemp < tempNum; mmvdMergeCandtemp++)
        {
          int baseIdx = 0;
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
          if (mbvdAdaptiveSearch)
          {
            baseIdx = mmvdMergeCandtemp / IBC_MBVD_SIZE_ENC;
            if (mmvdMergeCandtemp - baseIdx * IBC_MBVD_SIZE_ENC >= ibcMbvdValidNum[baseIdx])
            {
              continue;
            }
            numValidBv++;
          }
          else
          {
#endif
          baseIdx = mmvdMergeCandtemp / IBC_MBVD_MAX_REFINE_NUM;
          if (mmvdMergeCandtemp - (mmvdMergeCandtemp / IBC_MBVD_MAX_REFINE_NUM) * IBC_MBVD_MAX_REFINE_NUM >= IBC_MBVD_SIZE_ENC)
          {
            continue;
          }
          if (mmvdMergeCandtemp - (mmvdMergeCandtemp / IBC_MBVD_MAX_REFINE_NUM) * IBC_MBVD_MAX_REFINE_NUM >= ibcMbvdValidNum[baseIdx])
          {
            continue;
          }
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
          }
#endif
          unsigned int mmvdMergeCand = ibcMbvdLUT[mmvdMergeCandtemp];
          bool mbvdCandMisAlign = mergeCtxTmp.setIbcMbvdMergeCandiInfo(pu, mmvdMergeCandtemp, mmvdMergeCand);
          CHECK(mbvdCandMisAlign, "MBVD candidate is invalid");

          int xPred = pu.bv.getHor();
          int yPred = pu.bv.getVer();

          PU::spanMotionInfo(pu, mergeCtxTmp);

#if JVET_AA0070_RRIBC
          if (pu.cu->rribcFlipType == 0)
          {
            m_pcRdCost->setDistParam(distParam, tmpOrgLuma, refBuf, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);
          }
          else if (pu.cu->rribcFlipType == 1)
          {
            m_pcRdCost->setDistParam(distParam, tmpOrgLumaFlipH, refBuf, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);
          }
          else
          {
            m_pcRdCost->setDistParam(distParam, tmpOrgLumaFlipV, refBuf, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);
          }
#endif

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AE0169_BIPREDICTIVE_IBC
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
          int bufNum = mbvdAdaptiveSearch ? mmvdMergeCandtemp : mmvdMergeCandtemp - (baseIdx * IBC_MBVD_MAX_REFINE_NUM) + (baseIdx * IBC_MBVD_SIZE_ENC);
#else
          int bufNum = mmvdMergeCandtemp - (baseIdx*IBC_MBVD_MAX_REFINE_NUM) + (baseIdx*IBC_MBVD_SIZE_ENC);
#endif
#endif
          bool foundFracBV = isFracBv(pu.mv[0]);
          if (foundFracBV)
          {
#if JVET_AE0169_BIPREDICTIVE_IBC
            if (testIbcCiip || testIbcBiMbvd)
            {
              if (pu.cu->ibcLicFlag)
              {
                predBuf = tempCS->getPredBuf(pu.Y());
                m_pcInterSearch->m_predictionBeforeLIC = m_acGeoWeightedBuffer[bufNum].getBuf(localUnitArea);
                predBufMbvd[bufNum] = m_pcInterSearch->m_predictionBeforeLIC.Y();
              }
              else
              {
                predBuf = m_acGeoWeightedBuffer[bufNum].getBuf(localUnitArea).Y();
                predBufMbvd[bufNum] = predBuf;
              }
              predUnitBuf = PelUnitBuf(pu.chromaFormat, predBuf);
            }
#endif
#if JVET_AC0112_IBC_LIC && JVET_AC0112_IBC_CIIP
            m_pcInterSearch->m_storeBeforeLIC = pu.cu->ibcLicFlag;
#endif
            m_pcInterSearch->getPredIBCBlk(pu, COMPONENT_Y, pu.cu->slice->getPic(), pu.mv[0], predUnitBuf, encOptMC);
#if JVET_AC0112_IBC_LIC && JVET_AC0112_IBC_CIIP
            m_pcInterSearch->m_storeBeforeLIC = false;
#endif
            distParam.cur = predBuf;
          }
          else
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && JVET_AE0169_BIPREDICTIVE_IBC
          {
            if (testIbcCiip || testIbcBiMbvd)
            {
              predBuf = tempCS->getPredBuf(pu.Y());
              predBufMbvd[bufNum] = refBuf;
              predBufMbvd[bufNum].buf = piRefSrch + refStride * yPred + xPred;
            }
#endif
#endif
#if JVET_AC0112_IBC_LIC
          if (pu.cu->ibcLicFlag)
          {
            Pel* piPred = predBuf.buf;
            int predStride = predBuf.stride;
            int height = predBuf.height;
            int width = predBuf.width;
            for (int h = 0; h < height; h++)
            {
              memcpy(piPred, piRefSrch + h * refStride + refStride * yPred + xPred, width * sizeof(Pel));
              piPred += predStride;
            }
            m_pcInterSearch->xLocalIlluComp(pu, COMPONENT_Y
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                          , pu.mv[0]
#else
                                          , pu.bv
#endif
                                          , predBuf
#if JVET_AG0136_INTRA_TMP_LIC
                                          , true
#endif
                                          );
            distParam.cur = predBuf;
          }
          else
#endif
          distParam.cur.buf = piRefSrch + refStride * yPred + xPred;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && JVET_AE0169_BIPREDICTIVE_IBC
          }
#endif

          Distortion sad = distParam.distFunc(distParam);
#if JVET_AE0169_BIPREDICTIVE_IBC
          if (testIbcBiMbvd)
          {
            int pos;
            for (pos = 0; pos < IBC_MBVD_NUM_BI_CANDIDATES; pos++)
            {
              if (sad < mbvdSadList[pos].sad)
              {
                break;
              }
            }
            if (pos < IBC_MBVD_NUM_BI_CANDIDATES)
            {
              for (int i = std::min(numMbvdSadList, IBC_MBVD_NUM_BI_CANDIDATES-1); i > pos; i--)
              {
                mbvdSadList[i] = mbvdSadList[i-1];
              }
              mbvdSadList[pos].mmvdMergeCandtemp = mmvdMergeCandtemp;
              mbvdSadList[pos].sad = sad;
              numMbvdSadList = std::min(numMbvdSadList+1, IBC_MBVD_NUM_BI_CANDIDATES);
            }
          }
#endif
          uint32_t bitsCand = PU::getIbcMbvdEstBits(pu, mmvdMergeCandtemp);
#if JVET_AC0112_IBC_CIIP
          uint32_t ibcMbvdBits = bitsCand;
#endif
          bitsCand++; // for ibc_mbvd_flag
#if JVET_AC0112_IBC_CIIP&&JVET_AE0169_BIPREDICTIVE_IBC
          if (testIbcCiip)
          {
            bitsCand++; // for ibc_ciip_flag
          }
#endif
          double cost = (double)sad + (double)bitsCand * sqrtLambdaForFirstPass;

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && JVET_AC0112_IBC_CIIP
#if !JVET_AE0169_BIPREDICTIVE_IBC
          CPelBuf cPredIBC = distParam.cur;
#if JVET_AC0112_IBC_LIC
          if (pu.cu->ibcLicFlag)
          {
            cPredIBC.buf = foundFracBV ? m_pcInterSearch->m_predictionBeforeLIC.Y().buf : piRefSrch + refStride * yPred + xPred;
            cPredIBC.stride = foundFracBV ? m_pcInterSearch->m_predictionBeforeLIC.Y().stride : refStride;
          }
#endif
#endif
#endif
#if JVET_AC0112_IBC_CIIP
          if (testIbcCiip)
          {
            const PredictionUnit *puBv = pu.cs->getPURestricted(pu.lumaPos().offset(xPred, yPred), pu, pu.chType);
            int intraDir = puBv ? puBv->getIpmInfo(pu.lumaPos().offset(xPred, yPred)) : PLANAR_IDX;
            if (intraDir != ibcCiipIntraList[0])
            {
              ibcCiipIntraList[1] = intraDir;
            }
            else
            {
              ibcCiipIntraList[1] = ibcCiipIntraList[0] == PLANAR_IDX ? HOR_IDX : PLANAR_IDX;
            }
            intraDir = ibcCiipIntraList[1];
            if (!intraPredBufSet[intraDir])
            {
              pu.ibcCiipFlag = true;
              pu.intraDir[0] = intraDir;
              intraPredBufSet[intraDir] = true;
              intraPredBuf[intraDir] = m_acMergeBuffer[intraDir + IBC_GPM_MAX_NUM_UNI_CANDS].getBuf(localUnitArea);
              m_pcIntraSearch->initPredIntraParams(pu, pu.Y(), *pu.cs->sps, 0);
              m_pcIntraSearch->predIntraAng(COMPONENT_Y, intraPredBuf[intraDir].Y(), pu);
              pu.ibcCiipFlag = false;
            }
            for (int dirIdx = 0; dirIdx < IBC_CIIP_MAX_NUM_INTRA_CANDS; dirIdx++)
            {
              int height = pu.lheight();
              int width = pu.lwidth();
#if !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
              const Pel *pPredIbc = piRefSrch + refStride * yPred + xPred;
#endif
              Pel *pPredDst = ibcCiipBuf.buf;
              int dstStride = ibcCiipBuf.stride;
              Pel *pPredIntra = intraPredBuf[ibcCiipIntraList[dirIdx]].Y().buf;
              int intraStride = intraPredBuf[ibcCiipIntraList[dirIdx]].Y().stride;
              m_pcIntraSearch->m_ibcCiipBlending(pPredDst, dstStride
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AE0169_BIPREDICTIVE_IBC
                                               , predBufMbvd[bufNum].buf, predBufMbvd[bufNum].stride
#else
                                               , cPredIBC.buf, cPredIBC.stride
#endif
#else
                                               , pPredIbc, refStride
#endif
                                               , pPredIntra, intraStride, wIbc, wIntra, shift, width, height);

              distParam.cur.buf = ibcCiipBuf.buf;
              distParam.cur.stride = ibcCiipBuf.stride;

              Distortion sad2 = distParam.distFunc(distParam);
              uint32_t bitsCand2 = ibcMbvdBits;
              bitsCand2++; // for ibc_mbvd_flag
              bitsCand2 += dirIdx + 1;
#if JVET_AE0169_BIPREDICTIVE_IBC
              bitsCand2++; // for ibc_ciip_flag
#endif
              if (dirIdx == IBC_CIIP_MAX_NUM_INTRA_CANDS - 1)
              {
                bitsCand2--;
              }
              double cost2 = (double)sad2 + (double)bitsCand2 * sqrtLambdaForFirstPass;
              if (cost2 < ibcCiipBestSatdCost)
              {
#if JVET_Z0084_IBC_TM
                ibcCiipBestMergeCand = mmvdMergeCandtemp + mergeCtx.numValidMergeCand + mergeCtxTm.numValidMergeCand;
#else
                ibcCiipBestMergeCand = mmvdMergeCandtemp + mergeCtx.numValidMergeCand;
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
                ibcCiipBestIntraDir = intraDir;
#endif
                ibcCiipBestIntraCand = dirIdx;
                ibcCiipBestSatdCost = cost2;
              }
            }
          }
#endif

          updateCandList( 
#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_GPM
            ModeIbcInfo(
#endif
              mmvdMergeCandtemp + mergeCtx.numValidMergeCand
#if JVET_Z0084_IBC_TM
              + mergeCtxTm.numValidMergeCand
#endif
#if JVET_AC0112_IBC_CIIP
              , false
              , 0
#endif
#if JVET_AC0112_IBC_GPM
              , false
#if JVET_AE0169_BIPREDICTIVE_IBC
              , MAX_INT, MAX_INT, 0, 0, 0
#else
              , 0, 0, 0, 0, 0
#endif
#endif
#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_GPM
            )
#endif
            , cost, rdModeList, candCostList, numMrgSATDCand );
        }
#if JVET_AE0169_BIPREDICTIVE_IBC
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
        m_pcInterSearch->m_predictionBeforeLIC = m_acMergeTmpBuffer[MRG_MAX_NUM_CANDS].getBuf(localUnitArea);
        predBuf = tempCS->getPredBuf(pu.Y());
#endif
        if (testIbcBiMbvd && numMbvdSadList > 0)
        {
          bool checkBiMbvd[IBC_MBVD_BASE_NUM] = {false,};
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
          PelBuf mbvdCenterBuf[IBC_MBVD_BASE_NUM];
#endif

          for (int candIdx = 0; candIdx < numMbvdSadList; candIdx++)
          {
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
            int baseIdx = mbvdSadList[candIdx].mmvdMergeCandtemp / (mbvdAdaptiveSearch ? IBC_MBVD_SIZE_ENC : IBC_MBVD_MAX_REFINE_NUM);
#else
            int baseIdx = mbvdSadList[candIdx].mmvdMergeCandtemp / IBC_MBVD_MAX_REFINE_NUM;
#endif
            if (!mergeCtxTmp.rribcFlipTypes[baseIdx])
            {
              checkBiMbvd[baseIdx] = true;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
              const Mv& baseBv = mbvdAdaptiveSearch ? mergeCtxTmp.ibcMbvdBaseBvFrac[baseIdx][0].mv : mergeCtxTmp.ibcMbvdBaseBv[baseIdx][0].mv;
#else
              const Mv& baseBv = mergeCtxTmp.ibcMbvdBaseBv[baseIdx][0].mv;
#endif
              bool foundFracBV = isFracBv(baseBv);
              if (foundFracBV)
              {
                mbvdCenterBuf[baseIdx] = m_acMergeTmpBuffer[baseIdx].getBuf(localUnitArea).Y();
#if JVET_AC0112_IBC_LIC && JVET_AC0112_IBC_CIIP
                m_pcInterSearch->m_storeBeforeLIC = false;
#endif
                PelUnitBuf dstPic = PelUnitBuf(pu.chromaFormat, mbvdCenterBuf[baseIdx]);
                m_pcInterSearch->getPredIBCBlk(pu, COMPONENT_Y, pu.cu->slice->getPic(), baseBv, dstPic, encOptMC);
              }
#endif
            }
          }
          for (unsigned int mmvdMergeCandidx0 = 0; mmvdMergeCandidx0 < numMbvdSadList; mmvdMergeCandidx0++)
          {
            for (unsigned int mmvdMergeCandidx1 = mmvdMergeCandidx0+1; mmvdMergeCandidx1 < numMbvdSadList+baseNum; mmvdMergeCandidx1++)
            {
              int mmvdMergeCandtemp0 = mbvdSadList[mmvdMergeCandidx0].mmvdMergeCandtemp;
              int mmvdMergeCandtemp1;
              int mmvdMergeCand1;
              if (mmvdMergeCandidx1 >= numMbvdSadList)
              {
                mmvdMergeCandtemp1 = mmvdMergeCandidx1 - numMbvdSadList;
                if (!checkBiMbvd[mmvdMergeCandtemp1])
                {
                  continue;
                }
                mmvdMergeCand1 = -1;
              }
              else
              {
                mmvdMergeCandtemp1 = mbvdSadList[mmvdMergeCandidx1].mmvdMergeCandtemp;
                if (mmvdMergeCandtemp0 > mmvdMergeCandtemp1)
                {
                  std::swap(mmvdMergeCandtemp0, mmvdMergeCandtemp1);
                }
                mmvdMergeCand1 = ibcMbvdLUT[mmvdMergeCandtemp1];
                mmvdMergeCandtemp1 += IBC_MRG_MAX_NUM_CANDS;
              }
              int mmvdMergeCand0 = ibcMbvdLUT[mmvdMergeCandtemp0];
              bool mbvdCandMisAlign = mergeCtxTmp.setIbcMbvdMergeCandiInfo(pu, mmvdMergeCandtemp0, mmvdMergeCand0, mmvdMergeCandtemp1, mmvdMergeCand1);
              CHECK(mbvdCandMisAlign, "MBVD candidate is invalid");

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
              int bufNum0;
              if (mbvdAdaptiveSearch)
              {
                bufNum0 = mmvdMergeCandtemp0;
              }
              else
              {
                int baseIdx0 = mmvdMergeCandtemp0 / IBC_MBVD_MAX_REFINE_NUM;
                bufNum0 = mmvdMergeCandtemp0 - (baseIdx0 * IBC_MBVD_MAX_REFINE_NUM) + (baseIdx0 * IBC_MBVD_SIZE_ENC);
              }
#else
              int baseIdx0 = mmvdMergeCandtemp0 / IBC_MBVD_MAX_REFINE_NUM;
              int bufNum0 = mmvdMergeCandtemp0 - (baseIdx0*IBC_MBVD_MAX_REFINE_NUM) + (baseIdx0*IBC_MBVD_SIZE_ENC);
#endif
              CPelBuf refBufL1;
              if (mmvdMergeCandidx1 >= numMbvdSadList)
              {
                bool foundFracBV = isFracBv(pu.mv[1]);
                if (foundFracBV)
                {
                  refBufL1 = mbvdCenterBuf[mmvdMergeCandtemp1];
                }
                else
                {
                  Mv bvL1 = pu.mv[1];
                  bvL1.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
                  Position posL1(bvL1.getHor(), bvL1.getVer());
                  refBufL1 = refBuf.subBuf(posL1, refBuf);
                }
              }
              else
              {
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
                int bufNum1;
                if (mbvdAdaptiveSearch)
                {
                  bufNum1 = (mmvdMergeCandtemp1 - IBC_MRG_MAX_NUM_CANDS);
                }
                else
                {
                  int baseIdx1 = (mmvdMergeCandtemp1 - IBC_MRG_MAX_NUM_CANDS) / IBC_MBVD_MAX_REFINE_NUM;
                  bufNum1 = (mmvdMergeCandtemp1 - IBC_MRG_MAX_NUM_CANDS) - (baseIdx1 * IBC_MBVD_MAX_REFINE_NUM) + (baseIdx1 * IBC_MBVD_SIZE_ENC);
                }
#else
                int baseIdx1 = (mmvdMergeCandtemp1-IBC_MRG_MAX_NUM_CANDS) / IBC_MBVD_MAX_REFINE_NUM;
                int bufNum1 = (mmvdMergeCandtemp1-IBC_MRG_MAX_NUM_CANDS) - (baseIdx1*IBC_MBVD_MAX_REFINE_NUM) + (baseIdx1*IBC_MBVD_SIZE_ENC);
#endif
                refBufL1 = predBufMbvd[bufNum1];
              }
              PelBuf refBufBi = predBuf;
              refBufBi.avg(predBufMbvd[bufNum0], refBufL1);
#else
              Position posL0(pu.bv.getHor(), pu.bv.getVer());
              CPelBuf refBufL0 = refBuf.subBuf(posL0, refBuf);
              Mv bvL1 = pu.mv[1];
              bvL1.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
              Position posL1(bvL1.getHor(), bvL1.getVer());
              CPelBuf refBufL1 = refBuf.subBuf(posL1, refBuf);
              PelBuf refBufBi = predBuf;
              refBufBi.avg(refBufL0, refBufL1);
#endif

              m_pcRdCost->setDistParam(distParam, tmpOrgLuma, refBufBi, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);
              Distortion sad = distParam.distFunc(distParam);
              uint32_t bitsCand = PU::getIbcMbvdEstBits(pu, mmvdMergeCandtemp0, mmvdMergeCandtemp1);
              bitsCand+=2; // for ibc_mbvd_flag+ibc_mbvd_mode
              double cost = (double)sad + (double)bitsCand * sqrtLambdaForFirstPass;

              updateCandList( 
#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_GPM
                ModeIbcInfo(
#endif
                  mmvdMergeCandtemp0 + mergeCtx.numValidMergeCand
#if JVET_Z0084_IBC_TM
                  + mergeCtxTm.numValidMergeCand
#endif
#if JVET_AC0112_IBC_CIIP
                  , false
                  , 0
#endif
#if JVET_AC0112_IBC_GPM
                  , false
                  , mmvdMergeCandtemp0, mmvdMergeCandtemp1, 0, 0, 0
#endif
#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_GPM
                )
#endif
                , cost, rdModeList, candCostList, numMrgSATDCand );
            }
          }
        }
#endif
      }
    }
#endif
#if JVET_AA0070_RRIBC
    m_tmpStorageCUflipH.destroy();
    m_tmpStorageCUflipV.destroy();
#endif

#if JVET_AC0112_IBC_GPM
      static_vector<int, IBC_GPM_MAX_TRY_WEIGHTED_SATD> geoSplitDirList;
      static_vector<int, IBC_GPM_MAX_TRY_WEIGHTED_SATD> geoMergeCand0;
      static_vector<int, IBC_GPM_MAX_TRY_WEIGHTED_SATD> geoMergeCand1;
      static_vector<uint8_t, IBC_GPM_MAX_TRY_WEIGHTED_SATD>  geoBldIdxList;
      static_vector<double, IBC_GPM_MAX_TRY_WEIGHTED_SATD> geoSADCostList;
      int numSATDCands = IBC_GPM_MAX_TRY_WEIGHTED_SATD;
      double ibcGpmBestSatdCost = MAX_DOUBLE;
      int bestCandidateIdx = 0;
      int m_numIbcCandPerPar = 6;
      if (testIbcGpm && ibcGpmCandNumValid > 0)
      {
        int bitsCandSplit = floorLog2(IBC_GPM_MAX_SPLIT_DIR_SECOND_SET_NUM) + 1;

        int wIdx = floorLog2(cu.lwidth()) - GEO_MIN_CU_LOG2;
        int hIdx = floorLog2(cu.lheight()) - GEO_MIN_CU_LOG2;
        for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
        {
          if (!g_ibcGpmSecondSetSplitDir[splitDir] && g_geoParams[splitDir][0] % 8 != 0)
          {
            continue;
          }
          int maskStride = 0, maskStride2 = 0;
          int stepX = 1;
          Pel* sadMask;
          int16_t angle = g_geoParams[splitDir][0];
          if (g_angle2mirror[angle] == 2)
          {
            maskStride = -GEO_WEIGHT_MASK_SIZE;
            maskStride2 = -(int)cu.lwidth();
            sadMask = &g_geoEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
#if GPM_CURVE
            if(g_geoParams[splitDir][1] == 0 && wIdx == hIdx && wIdx >= GEO_CURVE_MIN_CU_IDX && CU::isGeoCURVEAvailable(splitDir,cu.lwidth()))
            {
              sadMask = &g_geoCurveEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][(GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][1]) * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
            }
#endif
          }
          else if (g_angle2mirror[angle] == 1)
          {
            stepX = -1;
            maskStride2 = cu.lwidth();
            maskStride = GEO_WEIGHT_MASK_SIZE;
            sadMask = &g_geoEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
#if GPM_CURVE
            if(g_geoParams[splitDir][1] == 0 && wIdx == hIdx && wIdx >= GEO_CURVE_MIN_CU_IDX && CU::isGeoCURVEAvailable(splitDir,cu.lwidth()))
            {
              sadMask = &g_geoCurveEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + (GEO_WEIGHT_MASK_SIZE - 1 - g_weightOffset[splitDir][hIdx][wIdx][0])];
            }
#endif
          }
          else
          {
            maskStride = GEO_WEIGHT_MASK_SIZE;
            maskStride2 = -(int)cu.lwidth();
            sadMask = &g_geoEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
#if GPM_CURVE
            if(g_geoParams[splitDir][1] == 0 && wIdx == hIdx && wIdx >= GEO_CURVE_MIN_CU_IDX && CU::isGeoCURVEAvailable(splitDir,cu.lwidth()))
            {
              sadMask = &g_geoCurveEncSadMask[g_angle2mask[g_geoParams[splitDir][0]]][g_weightOffset[splitDir][hIdx][wIdx][1] * GEO_WEIGHT_MASK_SIZE + g_weightOffset[splitDir][hIdx][wIdx][0]];
            }
#endif
          }
          Distortion sadSmall = 0, sadLarge = 0;
          double tempCost = 0;
#if JVET_AA0070_RRIBC
          m_pcRdCost->setDistParam(distParam, tmpOrgLuma, refBuf.buf, refBuf.stride, sadMask, maskStride, stepX, maskStride2, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y);
#else
        if (tempCS->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
        {
          const CompArea &area = cu.blocks[COMPONENT_Y];
          CompArea    tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
          PelBuf tmpLuma = m_tmpStorageLCU->getBuf(tmpArea);
          tmpLuma.rspSignal( tempCS->getOrgBuf().Y(), m_pcReshape->getFwdLUT() );
          m_pcRdCost->setDistParam(distParam, tmpLuma, refBuf.buf, refBuf.stride, sadMask, maskStride, stepX, maskStride2, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y);
        }
        else
        {
          m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), refBuf.buf, refBuf.stride, sadMask, maskStride, stepX, maskStride2, sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y);
        }
#endif
          for (unsigned int mergeCand = 0; mergeCand < mergeCtx.numValidMergeCand; mergeCand++)
          {
            if (isSkipThisCand[mergeCand])
            {
              continue;
            }
            distParam.cur = ibcPredBuf[mergeCand].Y();
            sadLarge = distParam.distFunc(distParam);
            tempCost = (double)sadLarge;
            m_geoMMVDCostList.insert(splitDir, 0, mergeCand, 0, tempCost);
            sortCandList(tempCost, mergeCand, sadCostList0[splitDir], mergeCandList0[splitDir], m_numIbcCandPerPar);
            sadSmall = sadWholeBlk[mergeCand] - sadLarge;
            tempCost = (double)sadSmall;
            m_geoMMVDCostList.insert(splitDir, 1, mergeCand, 0, tempCost);
            sortCandList(tempCost, mergeCand, sadCostList1[splitDir], mergeCandList1[splitDir], m_numIbcCandPerPar);
          }
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
          for (unsigned int mergeCand = 0; mergeCand < mergeCtxTm.numValidMergeCand; mergeCand++)
          {
            if (isSkipThisCand[mergeCtx.numValidMergeCand + mergeCand])
            {
              continue;
            }
            distParam.cur = ibcPredBuf[mergeCtx.numValidMergeCand + mergeCand].Y();
            sadLarge = distParam.distFunc(distParam);
            tempCost = (double)sadLarge;
            m_geoMMVDCostList.insert(splitDir, 0, mergeCtx.numValidMergeCand + mergeCand, 0, tempCost);
            sortCandList(tempCost, mergeCtx.numValidMergeCand + mergeCand, sadCostList0[splitDir], mergeCandList0[splitDir], m_numIbcCandPerPar);
            sadSmall = sadWholeBlk[mergeCtx.numValidMergeCand + mergeCand] - sadLarge;
            tempCost = (double)sadSmall;
            m_geoMMVDCostList.insert(splitDir, 1, mergeCtx.numValidMergeCand + mergeCand, 0, tempCost);
            sortCandList(tempCost, mergeCtx.numValidMergeCand + mergeCand, sadCostList1[splitDir], mergeCandList1[splitDir], m_numIbcCandPerPar);
          }
#endif
          for (unsigned int mergeCand = IBC_GPM_MAX_NUM_UNI_CANDS; mergeCand < IBC_GPM_MAX_NUM_UNI_CANDS + IBC_GPM_MAX_NUM_INTRA_CANDS; mergeCand++)
          {
            int intraIdx = mergeCand - IBC_GPM_MAX_NUM_UNI_CANDS;
            int rdobuffer = ibcGpmIntraCandList[splitDir][0][intraIdx];
            distParam.cur =  intraPredBuf[rdobuffer].Y();
            sadLarge = distParam.distFunc(distParam);
            tempCost = (double)sadLarge;
            m_geoMMVDCostList.insert(splitDir, 0, mergeCand, 0, tempCost);
            sortIntraCandList(tempCost, mergeCand, intraSadCostList0[splitDir], intraCandList0[splitDir]);

            if (ibcGpmIntraCandList[splitDir][0][intraIdx] != ibcGpmIntraCandList[splitDir][1][intraIdx])
            {
              rdobuffer = ibcGpmIntraCandList[splitDir][1][intraIdx];
              distParam.cur = intraPredBuf[rdobuffer].Y();
              sadLarge = distParam.distFunc(distParam);
            }
            sadSmall = sadIntraWholeBlk[rdobuffer] - sadLarge;
            tempCost = (double)sadSmall;
            m_geoMMVDCostList.insert(splitDir, 1, mergeCand, 0, tempCost);
            sortIntraCandList(tempCost, mergeCand, intraSadCostList1[splitDir], intraCandList1[splitDir]);
          }
        }
        for (int splitDir = 0; splitDir < GEO_NUM_PARTITION_MODE; splitDir++)
        {
          if (!g_ibcGpmSecondSetSplitDir[splitDir] && g_geoParams[splitDir][0] % 8 != 0)
          {
            continue;
          }
          int numCandMerge0 = min(m_numIbcCandPerPar, (int)mergeCandList0[splitDir].size());
          int numCandIntra0 = (int)intraCandList0[splitDir].size();
          int numCandPart0 = numCandMerge0 + numCandIntra0;
          for (int candIdx0 = 0; candIdx0 < numCandPart0; candIdx0++)
          {
            int mergeCand0 = candIdx0 < numCandMerge0 ? mergeCandList0[splitDir][candIdx0] : intraCandList0[splitDir][candIdx0-numCandMerge0];
            int numCandMerge1 = min(m_numIbcCandPerPar, (int)mergeCandList1[splitDir].size());
            int numCandIntra1 = candIdx0 < numCandMerge0 ? (int)intraCandList1[splitDir].size() : 0;
            int numCandPart1 = numCandMerge1 + numCandIntra1;
            int candStart1 = 0;
            for (int candIdx1 = candStart1; candIdx1 < numCandPart1; candIdx1++)
            {
              int mergeCand1 = candIdx1 < numCandMerge1 ? mergeCandList1[splitDir][candIdx1] : intraCandList1[splitDir][candIdx1-numCandMerge1];
              if (mergeCand0 == mergeCand1)
              {
                continue;
              }
#if JVET_AE0169_GPM_IBC_IBC
              if (candIdx0 >= numCandMerge0 && candIdx1 >= numCandMerge1)
#else
              if ((candIdx0 < numCandMerge0 && candIdx1 < numCandMerge1) || (candIdx0 >= numCandMerge0 && candIdx1 >= numCandMerge1))
#endif
              {
                continue;
              }
#if JVET_AE0169_GPM_IBC_IBC
              if (candIdx0 < numCandMerge0 && candIdx1 < numCandMerge1)
              {
                if ((mergeCand0 >= mergeCtx.numValidMergeCand && mergeCand1 < mergeCtx.numValidMergeCand)
                    || (mergeCand0 < mergeCtx.numValidMergeCand && mergeCand1 >= mergeCtx.numValidMergeCand))
                {
                  continue;
                }
              }
#endif
              if (isSkipThisCand[mergeCand0] || isSkipThisCand[mergeCand1])
              {
                continue;
              }
              double tempCost = m_geoMMVDCostList.singleDistList[0][splitDir][mergeCand0][0].cost + m_geoMMVDCostList.singleDistList[1][splitDir][mergeCand1][0].cost;
              updateGeoIbcCandList(tempCost, splitDir, mergeCand0, mergeCand1, 0, 0, geoSADCostList, geoSplitDirList, geoMergeCand0, geoMergeCand1, numSATDCands);
            }
          }
        }

#if JVET_AA0070_RRIBC
        m_pcRdCost->setDistParam(distParam, tmpOrgLuma, m_acMergeBuffer[0].Y(), sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);
#else
        if (tempCS->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
        {
          const CompArea &area = cu.blocks[COMPONENT_Y];
          CompArea    tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
          PelBuf tmpLuma = m_tmpStorageLCU->getBuf(tmpArea);
          tmpLuma.rspSignal( tempCS->getOrgBuf().Y(), m_pcReshape->getFwdLUT() );
          m_pcRdCost->setDistParam(distParam, tmpLuma, m_acMergeBuffer[0].Y(), sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);
        }
        else
        {
          m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), m_acMergeBuffer[0].Y(), sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, bUseHadamard);
        }
#endif
        int numberGeoCandChecked = (int)geoSADCostList.size();
        for (uint8_t bldIdx = 0; bldIdx < IBC_GPM_NUM_BLENDING; bldIdx++)
        {
#if JVET_AE0169_GPM_IBC_IBC
          if (pu.cs->slice->getSliceType() != I_SLICE)
          {
            if (bldIdx > 0)
            {
              continue;
            }
          }
#endif
          for (uint8_t candidateIdx = 0; candidateIdx < numberGeoCandChecked; candidateIdx++)
          {
            int splitDir = geoSplitDirList[candidateIdx];
            int mergeCand0 = geoMergeCand0[candidateIdx];
            int mergeCand1 = geoMergeCand1[candidateIdx];
            geoCombinations[candidateIdx * IBC_GPM_NUM_BLENDING + bldIdx] = m_acGeoWeightedBuffer[candidateIdx * IBC_GPM_NUM_BLENDING + bldIdx].getBuf(localUnitArea);
            int isIntra0 = (mergeCand0 >= IBC_GPM_MAX_NUM_UNI_CANDS) ? 1 : 0;
            int isIntra1 = (mergeCand1 >= IBC_GPM_MAX_NUM_UNI_CANDS) ? 1 : 0;
            int bitsCand = 1 + (g_geoParams[splitDir][0] % 8 == 0 ? 4 : bitsCandSplit);
            if (IBC_GPM_NUM_BLENDING > 1)
            {
              bitsCand++;
              if (bldIdx > 0)
              {
                bitsCand += 2;
              }
            }
#if JVET_AE0169_BIPREDICTIVE_IBC
            if (!isIntra0 && tempCS->sps->getMaxNumIBCMergeCand() > 1)
            {
              bitsCand++;
            }
#endif
            {
              PelUnitBuf predSrc0, predSrc1;
              if (isIntra0)
              {
                int intraIdx0 = mergeCand0 - IBC_GPM_MAX_NUM_UNI_CANDS;
                int rdoBuffer = ibcGpmIntraCandList[splitDir][0][intraIdx0];
                predSrc0 = intraPredBuf[rdoBuffer];
                bitsCand += intraIdx0 + 1;
                if (intraIdx0 == IBC_GPM_MAX_NUM_INTRA_CANDS - 1)
                {
                  bitsCand--;
                }
              }
              else
              {
                predSrc0 = ibcPredBuf[mergeCand0];
                int tempMergeCand0 = mergeCand0 > mergeCtx.numValidMergeCand ? mergeCand0 - mergeCtx.numValidMergeCand : mergeCand0;
                bitsCand += tempMergeCand0 + 1;
                if (tempMergeCand0 == tempCS->sps->getMaxNumIBCMergeCand() - 1)
                {
                  bitsCand--;
                }
              }
              if (isIntra1)
              {
                int intraIdx1 = mergeCand1 - IBC_GPM_MAX_NUM_UNI_CANDS;
                int rdoBuffer = ibcGpmIntraCandList[splitDir][1][intraIdx1];
                predSrc1 = intraPredBuf[rdoBuffer];
                bitsCand += intraIdx1 + 1;
                if (intraIdx1 == IBC_GPM_MAX_NUM_INTRA_CANDS - 1)
                {
                  bitsCand--;
                }
              }
              else
              {
                predSrc1 = ibcPredBuf[mergeCand1];
                int tempMergeCand1 = mergeCand1 > mergeCtx.numValidMergeCand ? mergeCand1 - mergeCtx.numValidMergeCand : mergeCand1;
                bitsCand += tempMergeCand1 + 1;
                if (tempMergeCand1 == tempCS->sps->getMaxNumIBCMergeCand() - 1)
                {
                  bitsCand--;
                }
#if JVET_AE0169_BIPREDICTIVE_IBC
                if (!isIntra0 && mergeCand1 > mergeCand0)
                {
                  bitsCand--;
                }
#endif
              }
              m_pcInterSearch->weightedGeoBlkRounded(pu, splitDir, bldIdx, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx*IBC_GPM_NUM_BLENDING+bldIdx], predSrc0, predSrc1);
            }
            distParam.cur = geoCombinations[candidateIdx * IBC_GPM_NUM_BLENDING + bldIdx].Y();
            Distortion sad = distParam.distFunc(distParam);
            double updateCost = (double)sad;
#if JVET_AE0169_BIPREDICTIVE_IBC
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
            if (pu.cs->sps->getUseTMIbc() && pu.cs->sps->getUseDMVDMode())
#else
            if (pu.cs->sps->getUseDMVDMode())
#endif
            {
              bitsCand++; // for tm_merge_flag
            }
#endif
#if JVET_AA0061_IBC_MBVD
            if (pu.cs->sps->getUseIbcMbvd())
            {
              bitsCand++; // for ibc_mbvd_flag
            }
#endif
#if JVET_AC0112_IBC_CIIP
            if (testIbcCiip)
            {
              bitsCand++; // for ibc_ciip_flag
            }
#endif
            bitsCand++; // for ibc_gpm_flag
#endif
            updateCost += bitsCand * sqrtLambdaForFirstPass;
            if (updateCost < ibcGpmBestSatdCost)
            {
              bestCandidateIdx = candidateIdx * IBC_GPM_NUM_BLENDING + bldIdx;
              ibcGpmBestSatdCost = updateCost;
            }
          }
        }
      }
#endif

      // Try to limit number of candidates using SATD-costs
      if (numValidBv)
      {
#if JVET_AA0061_IBC_MBVD
        numMrgSATDCand = std::min(numMrgSATDCand,numValidBv);
        for (int i = 1; i < numMrgSATDCand; i++)
#else
        numMrgSATDCand = numValidBv;
        for (unsigned int i = 1; i < numValidBv; i++)
#endif
        {
          if (candCostList[i] > MRG_FAST_RATIO*candCostList[0])
          {
            numMrgSATDCand = i;
            break;
          }
#if JVET_AE0169_BIPREDICTIVE_IBC
          if (m_pcEncCfg->getIBCFastMethod() & IBC_FAST_METHOD_NONSCC)
          {
            double intraCost = bUseHadamard ? m_pcIntraSearch->getBestIntraSADHADCost() : m_pcIntraSearch->getBestIntraSADCost();
            if (candCostList[i] > 1.1 * intraCost)
            {
              numMrgSATDCand = i;
              break;
            }
            Distortion bvpCost = m_pcInterSearch->getBestBvpSADHADCost();
            if (candCostList[i] > 1.1 * (double)bvpCost)
            {
              numMrgSATDCand = i;
              break;
            }
          }
#endif
        }

#if JVET_AC0112_IBC_CIIP
        if (testIbcCiip && ibcCiipBestSatdCost != MAX_DOUBLE)
        {
          if (ibcCiipBestSatdCost < candCostList[candCostList.size() - 1])
          {
            rdModeList[numMrgSATDCand].mergeCand = ibcCiipBestMergeCand;
            rdModeList[numMrgSATDCand].isCIIP = true;
            rdModeList[numMrgSATDCand].dirIdx = ibcCiipBestIntraCand;
#if JVET_AE0169_BIPREDICTIVE_IBC
            rdModeList[numMrgSATDCand].mergeIdx0 = MAX_INT;
            rdModeList[numMrgSATDCand].mergeIdx1 = MAX_INT;
#endif
            numMrgSATDCand++;
          }
        }
#endif
#if JVET_AC0112_IBC_GPM
        if (testIbcGpm && ibcGpmBestSatdCost != MAX_DOUBLE && ibcGpmBestSatdCost < candCostList[candCostList.size() - 1])
        {
          uint8_t candidateIdx = bestCandidateIdx;
          int splitDir = geoSplitDirList[candidateIdx/IBC_GPM_NUM_BLENDING];
          int mergeCand0 = geoMergeCand0[candidateIdx/IBC_GPM_NUM_BLENDING];
          int mergeCand1 = geoMergeCand1[candidateIdx/IBC_GPM_NUM_BLENDING];
          int bldIdx = candidateIdx%IBC_GPM_NUM_BLENDING;
#if JVET_AC0112_IBC_CIIP
          rdModeList.at(numMrgSATDCand++) = ModeIbcInfo(0, false, 0, true, mergeCand0, mergeCand1, splitDir, bldIdx, candidateIdx);
#else
          rdModeList.at(numMrgSATDCand++) = ModeIbcInfo(0, true, mergeCand0, mergeCand1, splitDir, bldIdx, candidateIdx);
#endif
        }
#endif
      }
      else
      {
        tempCS->dist = 0;
        tempCS->fracBits = 0;
        tempCS->cost = MAX_DOUBLE;
        tempCS->costDbOffset = 0;
        tempCS->initStructData(encTestMode.qp);
        return;
      }

      tempCS->initStructData(encTestMode.qp);
    }
  //}

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  bool  satdCandPredFilled[MRG_MAX_NUM_CANDS] = { false, };
#endif
  const unsigned int iteration = 2;
  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;
  // 2. Pass: check candidates using full RD test
  for (unsigned int numResidualPass = 0; numResidualPass < iteration; numResidualPass++)
  {
    for (unsigned int mrgHADIdx = 0; mrgHADIdx < numMrgSATDCand; mrgHADIdx++)
    {
#if JVET_AC0112_IBC_CIIP
      unsigned int mergeCand = rdModeList[mrgHADIdx].mergeCand;
      if (numResidualPass == 1 && rdModeList[mrgHADIdx].isCIIP)
      {
        continue;
      }
#if JVET_AC0112_IBC_GPM
#if JVET_AE0169_GPM_IBC_IBC
      unsigned int mergeCand1Gpm = mergeCand;
#endif
      if (rdModeList[mrgHADIdx].isIbcGpm)
      {
        mergeCand = rdModeList[mrgHADIdx].mergeIdx0 < IBC_GPM_MAX_NUM_UNI_CANDS ? rdModeList[mrgHADIdx].mergeIdx0 : rdModeList[mrgHADIdx].mergeIdx1;
#if JVET_AE0169_GPM_IBC_IBC
        if (rdModeList[mrgHADIdx].mergeIdx0 < IBC_GPM_MAX_NUM_UNI_CANDS
            && rdModeList[mrgHADIdx].mergeIdx1 < IBC_GPM_MAX_NUM_UNI_CANDS)
        {
          mergeCand1Gpm = rdModeList[mrgHADIdx].mergeIdx1;
        }
#endif
      }
#endif
#else
#if JVET_AC0112_IBC_GPM
      unsigned int mergeCand = rdModeList[mrgHADIdx].mergeCand;
      if (rdModeList[mrgHADIdx].isIbcGpm)
      {
        mergeCand = rdModeList[mrgHADIdx].mergeIdx0 < IBC_GPM_MAX_NUM_UNI_CANDS ? rdModeList[mrgHADIdx].mergeIdx0 : rdModeList[mrgHADIdx].mergeIdx1;
      }
#else
      unsigned int mergeCand = rdModeList[mrgHADIdx];
#endif
#endif
      if (!(numResidualPass == 1 && candHasNoResidual[mergeCand] == 1))
      {
        if (!(bestIsSkip && (numResidualPass == 0)))
        {
          {

            // first get merge candidates
            CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, (const ChannelType)partitioner.chType), (const ChannelType)partitioner.chType);

            partitioner.setCUData(cu);
            cu.slice = tempCS->slice;
            cu.tileIdx = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
            cu.skip = false;
            cu.predMode = MODE_IBC;
            cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
            cu.qp = encTestMode.qp;
            cu.sbtInfo = 0;
#if INTER_LIC
            cu.licFlag = false;
#endif
#if JVET_AC0112_IBC_LIC
            cu.ibcLicFlag = false;
#if JVET_AE0078_IBC_LIC_EXTENSION
            cu.ibcLicIdx = 0;
#endif
#endif
#if JVET_AE0159_FIBC
            cu.ibcFilterFlag = false;
#endif

#if JVET_AA0070_RRIBC
            cu.rribcFlipType = 0;
#endif
            PredictionUnit &pu = tempCS->addPU(cu, partitioner.chType);// tempCS->addPU(cu);
            pu.intraDir[0] = DC_IDX; // set intra pred for ibc block
            pu.intraDir[1] = PLANAR_IDX; // set intra pred for ibc block
#if JVET_AC0112_IBC_CIIP
            pu.ibcCiipFlag = rdModeList[mrgHADIdx].isCIIP;
            pu.ibcCiipIntraIdx = rdModeList[mrgHADIdx].dirIdx;
#endif
            cu.mmvdSkip = false;
            pu.mmvdMergeFlag = false;
            pu.regularMergeFlag = false;
            cu.geoFlag = false;
#if JVET_AC0112_IBC_GPM
            pu.ibcGpmFlag = rdModeList[mrgHADIdx].isIbcGpm;
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
            int mergeCand1 = pu.ibcGpmFlag ? MAX_INT : rdModeList[mrgHADIdx].mergeIdx1;
#endif
#if JVET_AA0061_IBC_MBVD
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
            int numPreviousBv = mergeCtx.numValidMergeCand + mergeCtxTm.numValidMergeCand;
#else
            int numPreviousBv = mergeCtx.numValidMergeCand;
#endif          
            if (mergeCand >= numPreviousBv)
            {
#if JVET_AE0169_BIPREDICTIVE_IBC
              int ibcMbvdIdx1 = mergeCand1 == MAX_INT ? -1 : mergeCand1;
              int ibcMbvdIdx1LUT = (mergeCand1 == MAX_INT || ibcMbvdIdx1 < IBC_MRG_MAX_NUM_CANDS) ? -1 : ibcMbvdLUT[ibcMbvdIdx1-IBC_MRG_MAX_NUM_CANDS];
              bool mbvdCandMisAlign = mergeCtxTmp.setIbcMbvdMergeCandiInfo(pu, mergeCand - numPreviousBv, ibcMbvdLUT[mergeCand - numPreviousBv], ibcMbvdIdx1, ibcMbvdIdx1LUT);
#else
              bool mbvdCandMisAlign = mergeCtxTmp.setIbcMbvdMergeCandiInfo(pu, mergeCand - numPreviousBv, ibcMbvdLUT[mergeCand - numPreviousBv]);
#endif
              CHECK(mbvdCandMisAlign, "MBVD candidate is invalid");
              PU::spanMotionInfo(pu, mergeCtxTmp
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                , pu.colIdx
#endif
              );
            }
            else
            {
#endif
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
              pu.tmMergeFlag = false;
              if (mergeCand >= mergeCtx.numValidMergeCand)
              {
                pu.tmMergeFlag = true;
                mergeCand -= mergeCtx.numValidMergeCand;
#if JVET_AC0112_IBC_GPM
                if (pu.ibcGpmFlag)
                {
                  mergeCtxTmIbcGeo.setMergeInfo(pu, mergeCand);
                  PU::spanMotionInfo(pu, mergeCtxTmIbcGeo);
                }
#if JVET_AE0169_BIPREDICTIVE_IBC
                else if (mergeCand1 != MAX_INT)
                {
                  mergeCtxTmIbcGeo.setMergeInfo(pu, mergeCand);
                  mergeCtxTmIbcGeo.setIbcL1Info(pu, mergeCand1);
                  PU::spanMotionInfo(pu, mergeCtxTmIbcGeo);
                }
#endif
                else
                {
#endif
                mergeCtxTm.setMergeInfo(pu, mergeCand);
                PU::spanMotionInfo(pu, mergeCtxTm
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                  , pu.colIdx
#endif
                );
#if JVET_AC0112_IBC_GPM
                }
#endif
              }
              else
#endif
              {
#if JVET_AC0112_IBC_GPM
                if (pu.ibcGpmFlag)
                {
                  mergeCtxIbcGeo.setMergeInfo(pu, mergeCand);
                  PU::spanMotionInfo(pu, mergeCtxIbcGeo);
                }
#if JVET_AE0169_BIPREDICTIVE_IBC
                else if (mergeCand1 != MAX_INT)
                {
                  mergeCtxIbcGeo.setMergeInfo(pu, mergeCand);
                  mergeCtxIbcGeo.setIbcL1Info(pu, mergeCand1);
                  PU::spanMotionInfo(pu, mergeCtxIbcGeo);
                }
#endif
                else
                {
#endif
                mergeCtx.setMergeInfo(pu, mergeCand);
                PU::spanMotionInfo(pu, mergeCtx
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                  , pu.colIdx
#endif
                );
#if JVET_AC0112_IBC_GPM
                }
#endif
              }
#if JVET_AA0061_IBC_MBVD
            }
#endif
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
            const bool chroma = !(CS::isDualITree(*tempCS));
#else
            const bool chroma = !pu.cu->isSepTree();
#endif
            //  MC
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
            bool hasBufferedCanPred = mrgHADIdx < MRG_MAX_NUM_CANDS && satdCandPredFilled[mrgHADIdx];
#endif
#if JVET_AC0112_IBC_GPM
            if (pu.ibcGpmFlag)
            {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
              int candidateIdx = rdModeList[mrgHADIdx].combIdx;
#endif
              pu.ibcGpmMergeIdx0 = rdModeList[mrgHADIdx].mergeIdx0;
              pu.ibcGpmMergeIdx1 = rdModeList[mrgHADIdx].mergeIdx1;
              if (rdModeList[mrgHADIdx].mergeIdx0 < IBC_GPM_MAX_NUM_UNI_CANDS && rdModeList[mrgHADIdx].mergeIdx0 >= mergeCtx.numValidMergeCand)
              {
                pu.ibcGpmMergeIdx0 =  pu.ibcGpmMergeIdx0 - mergeCtx.numValidMergeCand;
              }
              if (rdModeList[mrgHADIdx].mergeIdx1 < IBC_GPM_MAX_NUM_UNI_CANDS && rdModeList[mrgHADIdx].mergeIdx1 >= mergeCtx.numValidMergeCand)
              {
                pu.ibcGpmMergeIdx1 =  pu.ibcGpmMergeIdx1 - mergeCtx.numValidMergeCand;
              }
              pu.ibcGpmSplitDir = rdModeList[mrgHADIdx].splitDir;
              pu.ibcGpmBldIdx = rdModeList[mrgHADIdx].bldIdx;
#if JVET_AE0169_GPM_IBC_IBC
              uint8_t intraDir = PLANAR_IDX;
              if (pu.ibcGpmMergeIdx0 >= IBC_GPM_MAX_NUM_UNI_CANDS)
              {
                intraDir = ibcGpmIntraCandList[pu.ibcGpmSplitDir][0][pu.ibcGpmMergeIdx0 - IBC_GPM_MAX_NUM_UNI_CANDS];
              }
              else if (pu.ibcGpmMergeIdx1 >= IBC_GPM_MAX_NUM_UNI_CANDS)
              {
                intraDir = ibcGpmIntraCandList[pu.ibcGpmSplitDir][1][pu.ibcGpmMergeIdx1 - IBC_GPM_MAX_NUM_UNI_CANDS];
              }
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
              pu.intraDir[1] = PLANAR_IDX;
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
              if (!hasBufferedCanPred)
              {
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
              if (encOptMC)
              {
                m_pcInterSearch->motionCompensation(pu, REF_PIC_LIST_0, true, false);
#if JVET_AE0169_GPM_IBC_IBC
                if (rdModeList[mrgHADIdx].mergeIdx0 < IBC_GPM_MAX_NUM_UNI_CANDS
                    && rdModeList[mrgHADIdx].mergeIdx1 < IBC_GPM_MAX_NUM_UNI_CANDS)
                {
                  if (pu.tmMergeFlag)
                  {
                    mergeCand1Gpm -= mergeCtx.numValidMergeCand;
                    mergeCtxTmIbcGeo.setMergeInfo(pu, mergeCand1Gpm);
                    PU::spanMotionInfo(pu, mergeCtxTmIbcGeo);
                  }
                  else
                  {
                    mergeCtxIbcGeo.setMergeInfo(pu, mergeCand1Gpm);
                    PU::spanMotionInfo(pu, mergeCtxIbcGeo);
                  }
                  PelStorage m_tmpStorageCuIbcAnother;
                  m_tmpStorageCuIbcAnother.create(UnitArea(pu.chromaFormat, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
                  PelUnitBuf ibcAnotherBuf = m_tmpStorageCuIbcAnother.getBuf(localUnitArea);

                  m_pcInterSearch->motionCompensation(pu, ibcAnotherBuf, REF_PIC_LIST_0, true, false);
                  PelUnitBuf predBuf = cu.cs->getPredBuf(pu);
                  m_pcInterSearch->weightedGeoBlkRounded(pu, pu.ibcGpmSplitDir, pu.ibcGpmBldIdx, CHANNEL_TYPE_LUMA,
                                                          geoCombinations[candidateIdx], predBuf, ibcAnotherBuf);
                  m_tmpStorageCuIbcAnother.destroy();

                  if (pu.tmMergeFlag)
                  {
                    mergeCtxTmIbcGeo.setMergeInfo(pu, mergeCand);
                    PU::spanMotionInfo(pu, mergeCtxTmIbcGeo);
                  }
                  else
                  {
                    mergeCtxIbcGeo.setMergeInfo(pu, mergeCand);
                    PU::spanMotionInfo(pu, mergeCtxIbcGeo);
                  }
                }
                else
                {
#endif
                int rdoBuffer = pu.ibcGpmMergeIdx0 >= IBC_GPM_MAX_NUM_UNI_CANDS
                              ? ibcGpmIntraCandList[pu.ibcGpmSplitDir][0][pu.ibcGpmMergeIdx0 - IBC_GPM_MAX_NUM_UNI_CANDS]
                              : ibcGpmIntraCandList[pu.ibcGpmSplitDir][1][pu.ibcGpmMergeIdx1 - IBC_GPM_MAX_NUM_UNI_CANDS];
                PelUnitBuf predBuf = cu.cs->getPredBuf(pu);
                if (pu.ibcGpmMergeIdx0 < IBC_GPM_MAX_NUM_UNI_CANDS)
                {
                  m_pcInterSearch->weightedGeoBlkRounded(pu, pu.ibcGpmSplitDir, pu.ibcGpmBldIdx, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx], predBuf, intraPredBuf[rdoBuffer]);
                }
                else
                {
                  m_pcInterSearch->weightedGeoBlkRounded(pu, pu.ibcGpmSplitDir, pu.ibcGpmBldIdx, CHANNEL_TYPE_LUMA, geoCombinations[candidateIdx], intraPredBuf[rdoBuffer], predBuf);
                }
#if JVET_AE0169_GPM_IBC_IBC
                }
#endif
              }
#endif
              if (chroma)
              {
                m_pcInterSearch->motionCompensation(pu, REF_PIC_LIST_0, false, chroma);
#if JVET_AE0169_GPM_IBC_IBC
                if (rdModeList[mrgHADIdx].mergeIdx0 < IBC_GPM_MAX_NUM_UNI_CANDS
                    && rdModeList[mrgHADIdx].mergeIdx1 < IBC_GPM_MAX_NUM_UNI_CANDS)
                {
                  if (pu.tmMergeFlag)
                  {
                    mergeCtxTmIbcGeo.setMergeInfo(pu, mergeCand1Gpm);
                    PU::spanMotionInfo(pu, mergeCtxTmIbcGeo);
                  }
                  else
                  {
                    mergeCtxIbcGeo.setMergeInfo(pu, mergeCand1Gpm);
                    PU::spanMotionInfo(pu, mergeCtxIbcGeo);
                  }
                  PelStorage m_tmpStorageCuIbcAnother;
                  m_tmpStorageCuIbcAnother.create(UnitArea(pu.chromaFormat, Area(0, 0, MAX_CU_SIZE, MAX_CU_SIZE)));
                  PelUnitBuf ibcAnotherBuf = m_tmpStorageCuIbcAnother.getBuf(localUnitArea);

                  m_pcInterSearch->motionCompensation(pu, ibcAnotherBuf, REF_PIC_LIST_0, false, chroma);
                  PelUnitBuf predBuf = cu.cs->getPredBuf(pu);
                  m_pcInterSearch->weightedGeoBlkRounded(pu, pu.ibcGpmSplitDir, pu.ibcGpmBldIdx, CHANNEL_TYPE_CHROMA,
                                                          geoCombinations[candidateIdx], predBuf, ibcAnotherBuf);
                  m_tmpStorageCuIbcAnother.destroy();

                  if (pu.tmMergeFlag)
                  {
                    mergeCtxTmIbcGeo.setMergeInfo(pu, mergeCand);
                    PU::spanMotionInfo(pu, mergeCtxTmIbcGeo);
                  }
                  else
                  {
                    mergeCtxIbcGeo.setMergeInfo(pu, mergeCand);
                    PU::spanMotionInfo(pu, mergeCtxIbcGeo);
                  }
                }
                else
                {
#endif
                if (pu.ibcGpmMergeIdx0 >= IBC_GPM_MAX_NUM_UNI_CANDS)
                {
                  pu.intraDir[1] = ibcGpmIntraCandList[pu.ibcGpmSplitDir][0][pu.ibcGpmMergeIdx0 - IBC_GPM_MAX_NUM_UNI_CANDS];
                }
                else
                {
                  pu.intraDir[1] = ibcGpmIntraCandList[pu.ibcGpmSplitDir][1][pu.ibcGpmMergeIdx1 - IBC_GPM_MAX_NUM_UNI_CANDS];
                }
                if (!intraPredBufSet[0])
                {
                  intraPredBuf[0] = m_acMergeBuffer[0 + IBC_GPM_MAX_NUM_UNI_CANDS].getBuf(localUnitArea);
                  intraPredBufSet[0] = true;
                }

                m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cb());
                m_pcIntraSearch->predIntraAng(COMPONENT_Cb, intraPredBuf[0].Cb(), pu);
                m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cr());
                m_pcIntraSearch->predIntraAng(COMPONENT_Cr, intraPredBuf[0].Cr(), pu);
                pu.intraDir[1] = PLANAR_IDX;
                PelUnitBuf predBuf = cu.cs->getPredBuf(pu);
                if (pu.ibcGpmMergeIdx0 < IBC_GPM_MAX_NUM_UNI_CANDS)
                {
                  m_pcInterSearch->weightedGeoBlkRounded(pu, pu.ibcGpmSplitDir, pu.ibcGpmBldIdx, CHANNEL_TYPE_CHROMA, geoCombinations[candidateIdx], predBuf, intraPredBuf[0]);
                }
                else
                {
                  m_pcInterSearch->weightedGeoBlkRounded(pu, pu.ibcGpmSplitDir, pu.ibcGpmBldIdx, CHANNEL_TYPE_CHROMA, geoCombinations[candidateIdx], intraPredBuf[0], predBuf);
                }
#if JVET_AE0169_GPM_IBC_IBC
                }
#endif
              }
              tempCS->getPredBuf().copyFrom(geoCombinations[candidateIdx]
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                         , !chroma, false
#endif
              );
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
              }
#endif
#if JVET_AE0169_GPM_IBC_IBC
              pu.intraDir[0] = intraDir;
              if (pu.tmMergeFlag)
              {
                PU::spanGeoIBCMotionInfo(pu, mergeCtxTmIbcGeo);
              }
              else
              {
                PU::spanGeoIBCMotionInfo(pu, mergeCtxIbcGeo);
              }
              pu.intraDir[0] = DC_IDX;
#endif
            }
            else
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
            if (!hasBufferedCanPred)
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
            m_pcInterSearch->motionCompensation(pu,REF_PIC_LIST_X, true, chroma);
#else
            m_pcInterSearch->motionCompensation(pu,REF_PIC_LIST_0, true, chroma);
#endif
#if JVET_AC0112_IBC_CIIP
            if (pu.ibcCiipFlag)
            {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
              if (!hasBufferedCanPred)
              {
#endif
              if (pu.ibcCiipIntraIdx > 0)
              {
#if JVET_AE0169_BIPREDICTIVE_IBC
                int intraDir = ibcCiipIntraList[1] = ibcCiipBestIntraDir;
#else
                int xPred = pu.bv.getHor();
                int yPred = pu.bv.getVer();
                const PredictionUnit *puBv = pu.cs->getPURestricted(pu.lumaPos().offset(xPred, yPred), pu, pu.chType);
                int intraDir = puBv ? puBv->getIpmInfo(pu.lumaPos().offset(xPred, yPred)) : PLANAR_IDX;
                if (intraDir != ibcCiipIntraList[0])
                {
                  ibcCiipIntraList[1] = intraDir;
                }
                else
                {
                  ibcCiipIntraList[1] = ibcCiipIntraList[0] == PLANAR_IDX ? HOR_IDX : PLANAR_IDX;
                }
                intraDir = ibcCiipIntraList[1];
#endif
                if (!intraPredBufSet[intraDir])
                {
                  pu.intraDir[0] = intraDir;
                  intraPredBufSet[intraDir] = true;
                  intraPredBuf[intraDir] = m_acMergeBuffer[intraDir + IBC_GPM_MAX_NUM_UNI_CANDS].getBuf(localUnitArea);
                  m_pcIntraSearch->initPredIntraParams(pu, pu.Y(), *pu.cs->sps, 0);
                  m_pcIntraSearch->predIntraAng(COMPONENT_Y, intraPredBuf[intraDir].Y(), pu);
                }
              }
              m_pcIntraSearch->geneWeightedPred( COMPONENT_Y, tempCS->getPredBuf(pu).Y(), pu, tempCS->getPredBuf(pu).Y(), intraPredBuf[ibcCiipIntraList[pu.ibcCiipIntraIdx]].Y() );

              if (chroma)
              {
                pu.intraDir[1] = ibcCiipIntraList[pu.ibcCiipIntraIdx];
                m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cb());
                m_pcIntraSearch->predIntraAng(COMPONENT_Cb, intraPredBuf[pu.intraDir[1]].Cb(), pu);
                m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cr());
                m_pcIntraSearch->predIntraAng(COMPONENT_Cr, intraPredBuf[pu.intraDir[1]].Cr(), pu);
                m_pcIntraSearch->geneWeightedPred( COMPONENT_Cb, tempCS->getPredBuf(pu).Cb(), pu, tempCS->getPredBuf(pu).Cb(), intraPredBuf[pu.intraDir[1]].Cb() );
                m_pcIntraSearch->geneWeightedPred( COMPONENT_Cr, tempCS->getPredBuf(pu).Cr(), pu, tempCS->getPredBuf(pu).Cr(), intraPredBuf[pu.intraDir[1]].Cr() );
              }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
              }
#endif
              pu.intraDir[0] = DC_IDX;
              pu.intraDir[1] = PLANAR_IDX;
            }
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
            if (hasBufferedCanPred)
            {
              tempCS->getPredBuf().copyFrom(m_acMergeTmpBuffer[mrgHADIdx].getBuf(localUnitArea), !chroma, false);
            }
            else if (mrgHADIdx < MRG_MAX_NUM_CANDS)
            {
              satdCandPredFilled[mrgHADIdx] = true;
              m_acMergeTmpBuffer[mrgHADIdx].getBuf(localUnitArea).copyFrom(tempCS->getPredBuf(), !chroma, false);
            }
#endif
            m_CABACEstimator->getCtx() = m_CurrCtx->start;

            m_pcInterSearch->encodeResAndCalcRdInterCU(*tempCS, partitioner, (numResidualPass != 0), true, chroma);
            if (tempCS->slice->getSPS()->getUseColorTrans())
            {
              bestCS->tmpColorSpaceCost = tempCS->tmpColorSpaceCost;
              bestCS->firstColorSpaceSelected = tempCS->firstColorSpaceSelected;
            }
            xEncodeDontSplit(*tempCS, partitioner);

#if ENABLE_QPA_SUB_CTU
            xCheckDQP (*tempCS, partitioner);
#else
            // this if-check is redundant
            if (tempCS->pps->getUseDQP() && partitioner.currQgEnable())
            {
              xCheckDQP(*tempCS, partitioner);
            }
#endif
            xCheckChromaQPOffset( *tempCS, partitioner );


#if JVET_AC0112_IBC_CIIP
            if((tempCS->getCU(partitioner.chType))->skip && (tempCS->getCU(partitioner.chType))->firstPU->ibcCiipFlag)
            {
              tempCS->cost = MAX_DOUBLE;
              tempCS->costDbOffset = 0;
            }
#endif
            DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda());
            xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);

            tempCS->initStructData(encTestMode.qp);
          }

            if (m_pcEncCfg->getUseFastDecisionForMerge() && !bestIsSkip)
            {
              if (bestCS->getCU(partitioner.chType) == NULL)
                bestIsSkip = 0;
              else
              bestIsSkip = bestCS->getCU(partitioner.chType)->rootCbf == 0;
            }
        }
      }
    }
  }
  if ( m_bestModeUpdated && bestCS->cost != MAX_DOUBLE )
  {
    xCalDebCost( *bestCS, partitioner );
  }
}

#if JVET_AA0070_RRIBC
void EncCu::xCheckRDCostIBCMode( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode, bool isSecondPass)
#else
void EncCu::xCheckRDCostIBCMode(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
#endif
{
#if CTU_256
  if( tempCS->area.lwidth() >= 128 || tempCS->area.lheight() >= 128 ) // disable IBC mode larger than 64x64
#else
  if (tempCS->area.lwidth() == 128 || tempCS->area.lheight() == 128) // disable IBC mode larger than 64x64
#endif
  {
    return;
  }

  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;

#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_LIC
  double curBestCost = bestCS->cost;
  bool searchedByHash[1] = {false};
  Distortion tempCost[1] = {0};
#if JVET_AE0159_FIBC
  Distortion searchCost[3] = {0, 0, 0};
#else
  Distortion searchCost[2] = {0, 0};
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  m_pcInterSearch->m_bestSrchCostIntBv.init(true);
#if JVET_AC0112_IBC_LIC
  m_pcInterSearch->m_storeBeforeLIC = false;
#endif
#endif
#endif
#if JVET_AC0112_IBC_CIIP
  const UnitArea localUnitArea(tempCS->area.chromaFormat, Area(0, 0, tempCS->area.Y().width, tempCS->area.Y().height));
  PelBuf ibcCiipIntraBuf[3];
  bool skipSecondIbcCiipPass = false;
  Distortion intraCandCost[2] = {0, 0};
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  bool skipSecondBiPass = false;
  int biIdxMax = tempCS->slice->getBiPredictionIBCFlag() ? 2 : 1;
  bool predCiip = false;
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  m_pcInterSearch->resetBestBvpSADHADCost();
#endif

#if JVET_AC0112_IBC_LIC
  bool skipSecondLicPass = false;
#if JVET_AE0159_FIBC
  double bestNonLicCost = MAX_DOUBLE;
  int licIdxMax = (m_pcEncCfg->getIntraPeriod() == 1) && tempCS->slice->getSPS()->getUseIbcLic() && (tempCS->area.lx() > 0 || tempCS->area.ly() > 0) ? 2 : tempCS->slice->getSPS()->getUseIbcLic() && (tempCS->area.lx() > FIBC_TEMPLATE_SIZE || tempCS->area.ly() > FIBC_TEMPLATE_SIZE) ? 3 : (tempCS->slice->getSPS()->getUseIbcLic() && (tempCS->area.lx() > 0 || tempCS->area.ly() > 0) ? 2 : 1);
  if ( tempCS->area.lwidth() * tempCS->area.lheight() < 32 )
#else
  int licIdxMax = tempCS->slice->getSPS()->getUseIbcLic() && (tempCS->area.lx() > 0 || tempCS->area.ly() > 0) ? 2 : 1;
#if JVET_AE0078_IBC_LIC_EXTENSION
  if (tempCS->area.lwidth() * tempCS->area.lheight() < 32)
#else
  if (tempCS->area.lwidth() * tempCS->area.lheight() < 32 || tempCS->area.lwidth() * tempCS->area.lheight() > 256)
#endif
#endif
  {
    licIdxMax = 1;
  }
#endif
#if JVET_AC0112_IBC_CIIP
    int ibcCiipLoopNum = (tempCS->sps->getUseIbcCiip() && tempCS->slice->getSliceType() == I_SLICE && (tempCS->area.lwidth() * tempCS->area.lheight() >= 32) && tempCS->area.lwidth() <= 32 && tempCS->area.lheight() <= 32 && (tempCS->area.lx() > 0 || tempCS->area.ly() > 0)) ? 2 : 1;
#if JVET_AA0070_RRIBC
    if (isSecondPass && ibcCiipLoopNum > 1)
    {
      CodedCUInfo& relatedCU = ((EncModeCtrlMTnoRQT *)m_modeCtrl)->getBlkInfo(partitioner.currArea());
      if (relatedCU.isRribcCoded)
      {
        ibcCiipLoopNum = 1;
#if JVET_AE0169_BIPREDICTIVE_IBC
        biIdxMax = 1;
#endif
      }
    }
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
    for (int biIdx = 0; biIdx < biIdxMax; biIdx++)
    {
      if (biIdx == 1 && (skipSecondBiPass || !m_pcInterSearch->getAmvpMergeValidCand()))
      {
        continue;
      }
      if (biIdx == 1)
      {
        ibcCiipLoopNum = 1;
        licIdxMax = 1;
      }
#endif
    for (int ibcCiipIdx = 0; ibcCiipIdx < ibcCiipLoopNum; ibcCiipIdx++)
    {
#if JVET_AC0112_IBC_LIC
      if (ibcCiipIdx > 0)
      {
        CodingUnit* preBestCu = bestCS->getCU(partitioner.chType);
        if (preBestCu && preBestCu->ibcLicFlag)
        {
          continue;
        }
        licIdxMax = 1;
      }
      for (int licIdx = 0; licIdx < licIdxMax; licIdx++)
      {
#if JVET_AE0159_FIBC && !JVET_AE0078_IBC_LIC_EXTENSION
        if (licIdx == 1 && tempCS->area.lwidth() * tempCS->area.lheight() > 256)
        {
          continue;
        }
#endif
        if (licIdx == 1 && skipSecondLicPass)
        {
          continue;
        }
#endif
      if (ibcCiipIdx == 1 && skipSecondIbcCiipPass)
      {
        continue;
      }
#else
#if JVET_AC0112_IBC_LIC
      for (int licIdx = 0; licIdx < licIdxMax; licIdx++)
      {
        if (licIdx == 1 && skipSecondLicPass)
        {
          continue;
        }
#endif
#endif
    tempCS->initStructData(encTestMode.qp);

    CodingUnit &cu = tempCS->addCU(CS::getArea(*tempCS, tempCS->area, partitioner.chType), partitioner.chType);

    partitioner.setCUData(cu);
    cu.slice = tempCS->slice;
    cu.tileIdx = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
    cu.skip = false;
    cu.predMode = MODE_IBC;
    cu.chromaQpAdj = m_cuChromaQpOffsetIdxPlus1;
    cu.qp = encTestMode.qp;
    cu.imv = 0;
    cu.sbtInfo = 0;
#if JVET_AC0112_IBC_LIC
#if JVET_AE0159_FIBC
    cu.ibcLicFlag = licIdx > 0 ? true: false;
#if JVET_AE0078_IBC_LIC_EXTENSION
    cu.ibcLicIdx = 0;
#endif
    if (m_pcEncCfg->getIntraPeriod() == 1)
    {
      cu.ibcFilterFlag = false;
    }
    else
    {
      cu.ibcFilterFlag = licIdx > 1 ? true : false;
#if !JVET_AE0078_IBC_LIC_EXTENSION
      if ( cu.ibcLicFlag && !cu.ibcFilterFlag && tempCS->area.lwidth() * tempCS->area.lheight() > 256)
      {
        continue;
      }
#endif
    }
    if ((licIdx == 2) && ( !tempCS->slice->getSPS()->getUseIbcFilter() || cu.cs->slice->getSliceType() != I_SLICE))
    {
      continue;
    }
#else
    cu.ibcLicFlag = licIdx;
#if JVET_AE0078_IBC_LIC_EXTENSION
    cu.ibcLicIdx = 0;
#endif
#endif
#endif

    CU::addPUs(cu);

    m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;

    PredictionUnit& pu = *cu.firstPU;
    cu.mmvdSkip = false;
    pu.mmvdMergeFlag = false;
#if JVET_AC0112_IBC_CIIP
    pu.ibcCiipFlag = ibcCiipIdx;
#endif
    pu.regularMergeFlag = false;
#if INTER_LIC
    cu.licFlag = false;
#endif

#if JVET_AC0112_IBC_CIIP
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (ibcCiipIdx > 0 && !predCiip)
#else
    if (ibcCiipIdx > 0)
#endif
    {
      int ibcCiipIntraList[IBC_CIIP_MAX_NUM_INTRA_CANDS] = {PLANAR_IDX, HOR_IDX};
      IntraPrediction::deriveDimdMode(tempCS->picture->getRecoBuf(tempCS->area.Y()), tempCS->area.Y(), cu);
      cu.timdMode = m_pcIntraSearch->deriveTimdMode(tempCS->picture->getRecoBuf(cu.Y()), cu.Y(), cu);
      ibcCiipIntraList[0] = MAP131TO67(cu.timdMode);
      ibcCiipIntraList[1] = ibcCiipIntraList[0] == HOR_IDX ? PLANAR_IDX : HOR_IDX;
      m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Y(), true);
      for (int i = 0; i < IBC_CIIP_MAX_NUM_INTRA_CANDS; i++)
      {
        pu.ibcCiipIntraIdx = i;
        ibcCiipIntraBuf[0] = m_ciipBuffer[i].getBuf(localUnitArea.Y());
        pu.intraDir[0] = ibcCiipIntraList[i];
        m_pcIntraSearch->initPredIntraParams(pu, pu.Y(), *pu.cs->sps, 0);
        m_pcIntraSearch->predIntraAng(COMPONENT_Y, ibcCiipIntraBuf[0], pu);
        pu.interDir = 1;
        pu.refIdx[REF_PIC_LIST_0] = MAX_NUM_REF;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AA0070_RRIBC
        pu.cu->rribcFlipType = 0;
#endif
        m_pcInterSearch->predIBCSearch(cu, partitioner, m_ctuIbcSearchRangeX, m_ctuIbcSearchRangeY, m_ibcHashMap, tempCost, &ibcCiipIntraBuf[0]
#if JVET_AA0070_RRIBC
                                     , isSecondPass
#endif
#if JVET_AC0112_IBC_LIC || JVET_AC0112_IBC_CIIP
                                     , nullptr
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && JVET_AC0112_IBC_CIIP
                                     , true
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && (JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_LIC)
                                     , std::numeric_limits<double>::max()
#endif
        );
#else
#if JVET_AA0070_RRIBC
        pu.cu->rribcFlipType = 0;
        m_pcInterSearch->predIBCSearch(cu, partitioner, m_ctuIbcSearchRangeX, m_ctuIbcSearchRangeY, m_ibcHashMap, tempCost, &ibcCiipIntraBuf[0], isSecondPass);
#else
        m_pcInterSearch->predIBCSearch(cu, partitioner, m_ctuIbcSearchRangeX, m_ctuIbcSearchRangeY, m_ibcHashMap, tempCost, &ibcCiipIntraBuf[0]);
#endif
#endif
        intraCandCost[i] = tempCost[0];
      }
      pu.ibcCiipIntraIdx = intraCandCost[0] <= intraCandCost[1] ? 0 : 1;
      tempCost[0] = intraCandCost[0] <= intraCandCost[1] ? intraCandCost[0] : intraCandCost[1];
      ibcCiipIntraBuf[0] = m_ciipBuffer[pu.ibcCiipIntraIdx].getBuf(localUnitArea.Y());
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      const bool chroma = !(CS::isDualITree(*tempCS));
#else
      const bool chroma = !pu.cu->isSepTree();
#endif
      if (chroma)
      {
        ibcCiipIntraBuf[1] = m_ciipBuffer[0].getBuf(localUnitArea.Cb());
        ibcCiipIntraBuf[2] = m_ciipBuffer[0].getBuf(localUnitArea.Cr());
        pu.intraDir[1] = ibcCiipIntraList[pu.ibcCiipIntraIdx];
        m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cb());
        m_pcIntraSearch->predIntraAng(COMPONENT_Cb, ibcCiipIntraBuf[1], pu);
        m_pcIntraSearch->initIntraPatternChType(*pu.cu, pu.Cr());
        m_pcIntraSearch->predIntraAng(COMPONENT_Cr, ibcCiipIntraBuf[2], pu);
      }
#if JVET_AE0169_BIPREDICTIVE_IBC
      predCiip = true;
#endif
    }
#if JVET_AE0169_BIPREDICTIVE_IBC
    else if (ibcCiipIdx > 0)
    {
      pu.ibcCiipIntraIdx = intraCandCost[0] <= intraCandCost[1] ? 0 : 1;
      tempCost[0] = intraCandCost[0] <= intraCandCost[1] ? intraCandCost[0] : intraCandCost[1];
    }
#endif
#endif

    pu.intraDir[0] = DC_IDX; // set intra pred for ibc block
    pu.intraDir[1] = PLANAR_IDX; // set intra pred for ibc block

#if JVET_AE0169_BIPREDICTIVE_IBC
    pu.interDir = biIdx == 1 ? 3 : 1;
#else
    pu.interDir = 1; // use list 0 for IBC mode
#endif
    pu.refIdx[REF_PIC_LIST_0] = MAX_NUM_REF; // last idx in the list
#if JVET_AE0169_BIPREDICTIVE_IBC
    pu.refIdx[REF_PIC_LIST_1] = (pu.interDir == 3) ? MAX_NUM_REF : -1;
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AA0070_RRIBC
    pu.cu->rribcFlipType = 0;
#endif
    bool bValid = m_pcInterSearch->predIBCSearch(cu, partitioner, m_ctuIbcSearchRangeX, m_ctuIbcSearchRangeY, m_ibcHashMap
#if JVET_AC0112_IBC_LIC || JVET_AC0112_IBC_CIIP
                                               , tempCost
#endif
#if JVET_AC0112_IBC_CIIP
                                               , pu.ibcCiipFlag ? &ibcCiipIntraBuf[0] : nullptr
#endif
#if JVET_AA0070_RRIBC
                                               , isSecondPass
#endif
#if JVET_AC0112_IBC_LIC || JVET_AC0112_IBC_CIIP
#if JVET_AC0112_IBC_CIIP
                                               , pu.ibcCiipFlag ? nullptr : searchedByHash
#else
                                               , searchedByHash
#endif
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && JVET_AC0112_IBC_CIIP
                                               , false
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && (JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_LIC)
                                               , 1.1 * m_pcIntraSearch->getBestIntraSADCost()
#endif
    );
#else
#if JVET_AA0070_RRIBC
      pu.cu->rribcFlipType = 0;
#if JVET_AC0112_IBC_CIIP
      bool bValid = m_pcInterSearch->predIBCSearch(cu, partitioner, m_ctuIbcSearchRangeX, m_ctuIbcSearchRangeY, m_ibcHashMap, tempCost, pu.ibcCiipFlag ? &ibcCiipIntraBuf[0] : NULL, isSecondPass, pu.ibcCiipFlag ? NULL : searchedByHash);
#else
#if JVET_AC0112_IBC_LIC
      bool bValid = m_pcInterSearch->predIBCSearch(cu, partitioner, m_ctuIbcSearchRangeX, m_ctuIbcSearchRangeY, m_ibcHashMap, tempCost, isSecondPass, searchedByHash);
#else
      bool bValid = m_pcInterSearch->predIBCSearch(cu, partitioner, m_ctuIbcSearchRangeX, m_ctuIbcSearchRangeY, m_ibcHashMap, isSecondPass);
#endif
#endif
#else
#if JVET_AC0112_IBC_CIIP
      bool bValid = m_pcInterSearch->predIBCSearch(cu, partitioner, m_ctuIbcSearchRangeX, m_ctuIbcSearchRangeY, m_ibcHashMap, tempCost, pu.ibcCiipFlag ? &ibcCiipIntraBuf[0] : NULL, pu.ibcCiipFlag ? NULL : searchedByHash);
#else
#if JVET_AC0112_IBC_LIC
      bool bValid = m_pcInterSearch->predIBCSearch(cu, partitioner, m_ctuIbcSearchRangeX, m_ctuIbcSearchRangeY, m_ibcHashMap, tempCost, searchedByHash);
#else
      bool bValid = m_pcInterSearch->predIBCSearch(cu, partitioner, m_ctuIbcSearchRangeX, m_ctuIbcSearchRangeY, m_ibcHashMap);
#endif
#endif
#endif
#endif

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AC0112_IBC_CIIP || JVET_AC0112_IBC_LIC
      double dCost = (double)tempCost[0];
      if ((m_pcEncCfg->getIBCFastMethod() & IBC_FAST_METHOD_NONSCC) && (dCost > 1.1 * m_pcIntraSearch->getBestIntraSADCost()))
      {
        continue;
      }
#endif
#endif
#if JVET_AE0159_FIBC
      if (tempCS->slice->getSPS()->getUseIbcFilter())
      {
        if (!cu.ibcLicFlag)
        {
          bestNonLicCost = min(dCost, bestNonLicCost);
        }
        else if (dCost > 1.4 * bestNonLicCost)
        {
          continue;
        }
      }
#endif
#if JVET_AC0112_IBC_LIC
      if (licIdx == 0 && searchedByHash[0])
      {
        skipSecondLicPass = true;
      }
#endif
#if JVET_AC0112_IBC_CIIP
      if (ibcCiipIdx == 0 && searchedByHash[0])
      {
        skipSecondIbcCiipPass = true;
      }
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
      if (biIdx == 0 && searchedByHash[0])
      {
        skipSecondBiPass = true;
      }
#endif

      if (bValid)
      {
#if JVET_AC0112_IBC_LIC
#if JVET_AC0112_IBC_CIIP
#if JVET_AE0169_BIPREDICTIVE_IBC
        if (ibcCiipIdx == 0 && biIdx == 0)
#else
        if (ibcCiipIdx == 0)
#endif
        {
          searchCost[licIdx] = tempCost[0];
        }
#else
        searchCost[licIdx] = tempCost[0];
#endif
#if JVET_AE0159_FIBC
        if (licIdx > 0 && searchCost[0] > 0 && searchCost[licIdx] > 1.05 * searchCost[0])
#else
        if (licIdx > 0 && searchCost[0] > 0 && searchCost[1] > 1.05 * searchCost[0])
#endif
        {
          continue;
        }
#endif
#if JVET_AC0112_IBC_CIIP
#if JVET_AC0112_IBC_LIC
#if JVET_AE0169_BIPREDICTIVE_IBC
        if (licIdx == 0 && biIdx == 0)
#else
        if (licIdx == 0)
#endif
        {
          searchCost[ibcCiipIdx] = tempCost[0];
        }
#else
        searchCost[ibcCiipIdx] = tempCost[0];
#endif
        if (ibcCiipIdx > 0 && searchCost[0] > 0 && searchCost[1] > 1.2 * searchCost[0])
        {
          continue;
        }
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
        if (ibcCiipIdx == 0 && licIdx == 0)
        {
          searchCost[biIdx] = tempCost[0];
        }
        if (biIdx > 0 && searchCost[0] > 0 && searchCost[1] > 1.2 * searchCost[0])
        {
          continue;
        }
#endif
        PU::spanMotionInfo(pu);
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
        const bool chroma = !(CS::isDualITree(*tempCS));
#else
        const bool chroma = !pu.cu->isSepTree();
#endif
        //  MC
#if JVET_AE0169_BIPREDICTIVE_IBC
        m_pcInterSearch->motionCompensation(pu, REF_PIC_LIST_X, true, chroma);
#else
        m_pcInterSearch->motionCompensation(pu, REF_PIC_LIST_0, true, chroma);
#endif
#if JVET_AC0112_IBC_CIIP
        if (pu.ibcCiipFlag)
        {
          m_pcIntraSearch->geneWeightedPred( COMPONENT_Y, cu.cs->getPredBuf(pu).Y(), pu, cu.cs->getPredBuf(pu).Y(), ibcCiipIntraBuf[0] );
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
          const bool chroma = !(CS::isDualITree(*tempCS));
#else
          const bool chroma = !pu.cu->isSepTree();
#endif
          if (chroma)
          {
            m_pcIntraSearch->geneWeightedPred( COMPONENT_Cb, cu.cs->getPredBuf(pu).Cb(), pu, cu.cs->getPredBuf(pu).Cb(), ibcCiipIntraBuf[1] );
            m_pcIntraSearch->geneWeightedPred( COMPONENT_Cr, cu.cs->getPredBuf(pu).Cr(), pu, cu.cs->getPredBuf(pu).Cr(), ibcCiipIntraBuf[2] );
          }
        }
#endif

        {

          m_pcInterSearch->encodeResAndCalcRdInterCU(*tempCS, partitioner, false, true, chroma);
          if (tempCS->slice->getSPS()->getUseColorTrans())
          {
            bestCS->tmpColorSpaceCost = tempCS->tmpColorSpaceCost;
            bestCS->firstColorSpaceSelected = tempCS->firstColorSpaceSelected;
          }

          xEncodeDontSplit(*tempCS, partitioner);

#if ENABLE_QPA_SUB_CTU
          xCheckDQP (*tempCS, partitioner);
#else
          // this if-check is redundant
          if (tempCS->pps->getUseDQP() && partitioner.currQgEnable())
          {
            xCheckDQP(*tempCS, partitioner);
          }
#endif
          xCheckChromaQPOffset( *tempCS, partitioner );

          tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();
          if ( m_bestModeUpdated )
          {
            xCalDebCost( *tempCS, partitioner );
          }
#if JVET_AE0169_BIPREDICTIVE_IBC
          if (biIdx == 0 && ibcCiipIdx == 0 && licIdx == 0)
          {
#if JVET_AC0112_IBC_CIIP
            if (tempCS->cost > curBestCost * 1.1)
            {
              skipSecondIbcCiipPass = true;
            }
#endif
#if JVET_AC0112_IBC_LIC && !JVET_AE0078_IBC_LIC_EXTENSION
            if (tempCS->cost > curBestCost * 1.4)
            {
              skipSecondLicPass = true;
            }
#endif
            if (tempCS->cost > curBestCost * 1.2)
            {
              skipSecondBiPass = true;
            }
          }
#else
#if JVET_AC0112_IBC_CIIP
          if (ibcCiipIdx == 0 && tempCS->cost > curBestCost * 1.1)
          {
            skipSecondIbcCiipPass = true;
          }
#endif
#if JVET_AC0112_IBC_LIC
#if JVET_AE0159_FIBC
          if ( !tempCS->slice->getSPS()->getUseIbcFilter() && licIdx == 0 && tempCS->cost > curBestCost * 1.4)
#else
          if (licIdx == 0 && tempCS->cost > curBestCost * 1.4)
#endif
          {
            skipSecondLicPass = true;
          }
#endif
#endif

#if JVET_AE0169_BIPREDICTIVE_IBC
          if (tempCS->cost < curBestCost * 1.2)
          {
            m_skipIbcMerge = false;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
            if (m_pcEncCfg->getIBCFastMethod() & IBC_FAST_METHOD_NONSCC)
            {
              if (cu.slice->getDisableSATDForRD())
              {
                m_pcInterSearch->setBestBvpSADHADCost(tempCost[0]);
              }
              else
              {
                DistParam distParam;
                const CPelBuf predBuf = pu.cs->getPredBuf( pu ).Y();
                if (tempCS->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())
                {
                  const CompArea &area = cu.blocks[COMPONENT_Y];
                  CompArea    tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
                  PelBuf tmpLuma = m_tmpStorageLCU->getBuf(tmpArea);
                  tmpLuma.rspSignal( tempCS->getOrgBuf().Y(), m_pcReshape->getFwdLUT() );
                  m_pcRdCost->setDistParam(distParam, tmpLuma, predBuf, tempCS->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, true);
                }
                else
                {
                  m_pcRdCost->setDistParam(distParam, tempCS->getOrgBuf().Y(), predBuf, tempCS->sps->getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, true);
                }
                Distortion satd = distParam.distFunc(distParam);
                uint32_t   addExtraBits  = 1 + pu.cu->bvZeroCompDir;
                Distortion binCost = m_pcRdCost->getBvCost(pu.mvd[0], pu.cu->imv, cu.cs->sps->getIBCFracFlag() ? 0 : 1, pu.cu->bvZeroCompDir == 2, pu.cu->bvZeroCompDir == 1, addExtraBits, pu.mvpIdx[0]);
                if (pu.interDir == 3 && !pu.cu->ibcLicFlag && !pu.ibcCiipFlag && !pu.cu->rribcFlipType)
                {
                  binCost += m_pcRdCost->getBvpMergeCost(pu.mergeIdx);
                }
                m_pcInterSearch->setBestBvpSADHADCost(satd+binCost);
              }
            }
#endif
          }
#endif
          DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda());
          xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);

        }

      } // bValid
      else
      {
        tempCS->dist = 0;
        tempCS->fracBits = 0;
        tempCS->cost = MAX_DOUBLE;
        tempCS->costDbOffset = 0;
#if JVET_AC0112_IBC_CIIP
        if (ibcCiipIdx == 0)
        {
          break;
        }
#endif
      }
#if JVET_AE0169_BIPREDICTIVE_IBC
      }
#endif
#if JVET_AC0112_IBC_LIC
    }
#endif
#if JVET_AC0112_IBC_CIIP
    }
#endif
}
  // check ibc mode in encoder RD
  //////////////////////////////////////////////////////////////////////////////////////////////

#if ENABLE_OBMC && JVET_AA0129_INTERHASH_OBMCOFF_RD
bool EncCu::xCheckRDCostInter( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
#else
void EncCu::xCheckRDCostInter( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode )
#endif
{
#if ENABLE_OBMC && JVET_AA0129_INTERHASH_OBMCOFF_RD
  bool validMode = false;
#endif
#if INTER_LIC
  if (m_pcInterSearch->m_fastLicCtrl.skipRDCheckForLIC((encTestMode.opts & ETO_LIC) > 0, IMV_OFF, bestCS->cost, tempCS->area.Y().area()))
  {
#if ENABLE_OBMC && JVET_AA0129_INTERHASH_OBMCOFF_RD
    return (m_pcEncCfg->getUseHashME() ? validMode : true);
#else
    return;
#endif
  }
#endif
#if ENABLE_OBMC
  double bestOBMCCost = MAX_DOUBLE;
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
  int maxBdmvrAmSearchLoop = 3;
  m_pcInterSearch->m_amvpOnlyCost = std::numeric_limits<Distortion>::max();
#if JVET_Y0128_NON_CTC
  if (!tempCS->slice->isInterB() || (tempCS->slice->getUseAmvpMergeMode() == false)
#else
  if (!tempCS->slice->isInterB() || (tempCS->picHeader->getMvdL1ZeroFlag() == true)
#endif
#if INTER_LIC
      || (tempCS->slice->getUseLIC() && (encTestMode.opts & ETO_LIC))
#endif
      )
  {
    maxBdmvrAmSearchLoop = 1;
  }
  for (int bdmvrAmSearchLoop = 0; bdmvrAmSearchLoop < maxBdmvrAmSearchLoop; bdmvrAmSearchLoop++)
  {
  bool bdmvrAmMergeNotValid = false;
#endif
  tempCS->initStructData( encTestMode.qp );
  m_pcInterSearch->setAffineModeSelected(false);
#if JVET_AD0213_LIC_IMP
  m_pcInterSearch->setDoAffineLic(true);
#endif

  m_pcInterSearch->resetBufferedUniMotions();
  int bcwLoopNum = (tempCS->slice->isInterB() ? BCW_NUM : 1);
  bcwLoopNum = (tempCS->sps->getUseBcw() ? bcwLoopNum : 1);
#if INTER_LIC
  bool lic = encTestMode.opts & ETO_LIC;
#if !JVET_AD0213_LIC_IMP
  bcwLoopNum = lic ? 1 : bcwLoopNum;
#endif
#endif

  if( tempCS->area.lwidth() * tempCS->area.lheight() < BCW_SIZE_CONSTRAINT )
  {
    bcwLoopNum = 1;
  }
#if JVET_X0083_BM_AMVP_MERGE_MODE
  bcwLoopNum = (bdmvrAmSearchLoop > 0) ? 1 : bcwLoopNum;
#endif

  double curBestCost = bestCS->cost;
  double equBcwCost = MAX_DOUBLE;

  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;

#if JVET_AG0276_LIC_SLOPE_ADJUST
  bool testLicSlopeAdjust = tempCS->sps->getLicSlopeAdjustEnabledFlag() && lic && bdmvrAmSearchLoop == 0;
  testLicSlopeAdjust &= (tempCS->area.Y().x || tempCS->area.Y().y);
  testLicSlopeAdjust &= CU::licSlopeSizeTlCond(tempCS->area.Y().width, tempCS->area.Y().height, tempCS->slice->getTLayer());
  int licDeltaE = testLicSlopeAdjust ? LIC_SLOPE_MAX_NUM_DELTA : 0;
  for (int licDeltaIdx = 0; licDeltaIdx <= licDeltaE; licDeltaIdx++)
  {
    int licDelta = g_licSlopeDeltaSet[licDeltaIdx];
    bcwLoopNum = licDelta == 0 ? bcwLoopNum : 1;
    if (licDelta != 0)
    {
      ComprCUCtx cuECtx = m_modeCtrl->getComprCUCtx();
      double licCost = cuECtx.get<double>(BEST_LIC_COST);
      if (licCost != MAX_DOUBLE * .5 && m_pcEncCfg->getFastLicAffine() && ((licCost > bestCS->cost * 1.5 && m_pcEncCfg->getIntraPeriod() > 1) || (licCost > bestCS->cost * 1.4 && m_pcEncCfg->getIntraPeriod() < 0)))
      {
        continue;
      }
    }
#endif
  for( int bcwLoopIdx = 0; bcwLoopIdx < bcwLoopNum; bcwLoopIdx++ )
  {
    if( m_pcEncCfg->getUseBcwFast() )
    {
      auto blkCache = dynamic_cast< CacheBlkInfoCtrl* >(m_modeCtrl);

      if( blkCache )
      {
        bool isBestInter = blkCache->getInter(bestCS->area);
        uint8_t bestBcwIdx = blkCache->getBcwIdx(bestCS->area);

        if( isBestInter && g_bcwSearchOrder[bcwLoopIdx] != BCW_DEFAULT && g_bcwSearchOrder[bcwLoopIdx] != bestBcwIdx )
        {
          continue;
        }
      }
    }
    if( !tempCS->slice->getCheckLDC() )
    {
      if( bcwLoopIdx != 0 && bcwLoopIdx != 3 && bcwLoopIdx != 4 )
      {
        continue;
      }
    }
#if JVET_AD0213_LIC_IMP
    if (m_pcEncCfg->getFastLicBcw() && lic && g_bcwSearchOrder[bcwLoopIdx] != BCW_DEFAULT)
    {
      continue;
    }
#endif

  CodingUnit &cu      = tempCS->addCU( tempCS->area, partitioner.chType );

  partitioner.setCUData( cu );
  cu.slice            = tempCS->slice;
  cu.tileIdx          = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
  cu.skip             = false;
  cu.mmvdSkip = false;
//cu.affine
  cu.predMode         = MODE_INTER;
#if INTER_LIC
  cu.licFlag          = lic;
#if JVET_AG0276_LIC_SLOPE_ADJUST
  cu.licDelta        = licDelta;
#endif
#endif
  cu.chromaQpAdj      = m_cuChromaQpOffsetIdxPlus1;
  cu.qp               = encTestMode.qp;
  CU::addPUs( cu );

  cu.bcwIdx = g_bcwSearchOrder[bcwLoopIdx];
  uint8_t bcwIdx = cu.bcwIdx;
  bool  testBcw = (bcwIdx != BCW_DEFAULT);

#if JVET_AC0158_PIXEL_AFFINE_MC && !JVET_AD0213_LIC_IMP
  if ((tempCS->area.lwidth() * tempCS->area.lheight() < 32) || (lic == true) || tempCS->sps->getUseOBMC() == false)
  {
    cu.obmcFlag = false;
  }
#endif

#if INTER_LIC
  if (cu.slice->getUseLIC() && lic) { m_pcInterSearch->swapUniMvBuffer(); }
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
  if (bdmvrAmSearchLoop == 0)
  {
    cu.firstPU->amvpMergeModeFlag[REF_PIC_LIST_0] = false;
    cu.firstPU->amvpMergeModeFlag[REF_PIC_LIST_1] = false;
  }
  else if (bdmvrAmSearchLoop == 1)
  {
    cu.firstPU->amvpMergeModeFlag[REF_PIC_LIST_0] = true;
    cu.firstPU->amvpMergeModeFlag[REF_PIC_LIST_1] = false;
  }
  else
  {
    cu.firstPU->amvpMergeModeFlag[REF_PIC_LIST_0] = false;
    cu.firstPU->amvpMergeModeFlag[REF_PIC_LIST_1] = true;
  }
#if JVET_AD0213_LIC_IMP
  if (m_pcEncCfg->getFastLicAffine())
  {
    m_pcInterSearch->setDoAffineLic(bestCS->cus.front()->affine || bestCS->cus.front()->licFlag);
  }
#if JVET_AG0276_LIC_SLOPE_ADJUST
  else if (licDelta != 0)
  {
    m_pcInterSearch->setDoAffineLic(bestCS->cus.front()->affine || bestCS->cus.front()->licFlag);
  }
#endif
#endif
  if (cu.firstPU->amvpMergeModeFlag[0] || cu.firstPU->amvpMergeModeFlag[1])
  {
#if JVET_AB0078_AMVPMERGE_LDB
    if (bdmvrAmSearchLoop == 1 && tempCS->picHeader->getMvdL1ZeroFlag())
    {
      tempCS->initStructData(encTestMode.qp);
      continue;
    }
#endif
    m_pcInterSearch->predInterSearch( cu, partitioner, bdmvrAmMergeNotValid,
#if JVET_X0083_BM_AMVP_MERGE_MODE && JVET_AD0213_LIC_IMP
        m_mvFieldAmListEnc, m_licAmListEnc, m_mvBufEncAmBDMVR[0], m_mvBufEncAmBDMVR[1]);
#else
        m_mvFieldAmListEnc, m_mvBufEncAmBDMVR[0], m_mvBufEncAmBDMVR[1] );
#endif
  }
  else
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
  {
    m_pcInterSearch->setBdofSubPuMvBuf(m_mvBufEncMhpAffineBDOF);
    m_pcInterSearch->predInterSearch( cu, partitioner, bdmvrAmMergeNotValid );
  }
#else
  m_pcInterSearch->predInterSearch( cu, partitioner, bdmvrAmMergeNotValid );
#endif
  if ((cu.firstPU->refIdx[REF_PIC_LIST_0] < 0 && cu.firstPU->refIdx[REF_PIC_LIST_1] < 0) || bdmvrAmMergeNotValid)
  {
    tempCS->initStructData(encTestMode.qp);
    continue;
  }
#else
  m_pcInterSearch->predInterSearch( cu, partitioner );
#endif
#if INTER_LIC
  if (cu.slice->getUseLIC() && lic) { m_pcInterSearch->swapUniMvBuffer(); }
#endif

  bcwIdx = CU::getValidBcwIdx(cu);
  if( testBcw && bcwIdx == BCW_DEFAULT ) // Enabled Bcw but the search results is uni.
  {
    tempCS->initStructData(encTestMode.qp);
    continue;
  }
  CHECK(!(testBcw || (!testBcw && bcwIdx == BCW_DEFAULT)), " !( bTestBcw || (!bTestBcw && bcwIdx == BCW_DEFAULT ) )");

#if JVET_AG0276_LIC_SLOPE_ADJUST
  if (licDelta != 0 && cu.firstPU->interDir == 3)
  {
    tempCS->initStructData(encTestMode.qp);
    continue;
  }
#endif

  bool isEqualUni = false;
  if( m_pcEncCfg->getUseBcwFast() )
  {
    if( cu.firstPU->interDir != 3 && testBcw == 0 )
    {
      isEqualUni = true;
    }
  }
#if JVET_Y0128_NON_CTC && INTER_LIC && !JVET_AF0190_RPR_TMP_REORDER_LIC
  if (cu.licFlag)
  {
    if (!PU::checkRprLicCondition(*cu.firstPU)) // To check whether LIC actually performs in MC
    {
      cu.licFlag = false;
      PU::spanLicFlags(*cu.firstPU, false);
#if JVET_AD0213_LIC_IMP
      tempCS->initStructData(encTestMode.qp);
      continue;
#endif
    }
  }
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER
  PredictionUnit& pu = *cu.firstPU;

#if JVET_AD0140_MVD_PREDICTION
  if (pu.cu->smvdMode)
  {

    const Mv& cMvd = pu.mvd[REF_PIC_LIST_0];

    if (pu.isMvdPredApplicable() && cMvd.isMvdPredApplicable())
    {
      if (MotionModel::Undefined == pu.mvdSuffixInfo.m_motionModel)
      {
        THROW("Undefined Motion Model for SMVD");
      }
    }
    else
    {
      pu.mvdSuffixInfo.m_motionModel = MotionModel::BiTranslationalSmvd;
      pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_0][0].m_motionModel = MotionModel::BiTranslationalSmvd;
    }
  }
#endif


  if (PU::useRefCombList(pu))
  {
    m_pcInterSearch->setUniRefIdxLC(pu);
  }
  else if (PU::useRefPairList(pu))
  {
    m_pcInterSearch->setBiRefPairIdx(pu);
  }
#endif
#if JVET_AC0158_PIXEL_AFFINE_MC
  if (cu.affine == true)
  {
#if JVET_AD0213_LIC_IMP
    if (tempCS->sps->getUseOBMC() == false)
    {
#else
    if (lic == true || tempCS->sps->getUseOBMC() == false)
    {
      CHECK(cu.obmcFlag == true, "this is not possible");
#endif
      xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, 0
                            , 0
                            , &equBcwCost
      );
    }
    else
    {
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
      PelUnitBuf tempWoOBMCBuf;
      if (PU::checkDoAffineBdofRefine(*(cu.firstPU), m_pcInterSearch))
      {
        tempWoOBMCBuf = m_tempWoOBMCBuffer.subBuf(UnitAreaRelative(cu, cu));
        tempWoOBMCBuf.copyFrom(tempCS->getPredBuf(cu));
      }
#endif
      CodingStructure *prevCS = tempCS;
#if JVET_AD0213_LIC_IMP
      CHECK(!cu.obmcFlag, "obmcFlag is false for affine non-LIC");
#else
      CHECK(cu.obmcFlag == false, "this is not possible");
#endif
      cu.isobmcMC = true;
      m_pcInterSearch->subBlockOBMC(*cu.firstPU);
      cu.isobmcMC = false;
      xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, 0
                            , 0
                            , &equBcwCost
      );
      if (prevCS == tempCS)
      {
        cu.obmcFlag = false;
        PelUnitBuf predBuf = tempCS->getPredBuf(cu);
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
        if (PU::checkDoAffineBdofRefine(*(cu.firstPU), m_pcInterSearch))
        {
          predBuf.copyFrom(tempWoOBMCBuf);
        }
        else
        {
#endif
        m_pcInterSearch->motionCompensation( *(cu.firstPU), predBuf, REF_PIC_LIST_X );
#if JVET_AD0213_LIC_IMP
        if (cu.licFlag)
        {
          for (int list = 0; list < 2; list++)
          {
            if (cu.firstPU->refIdx[list] >= 0)
            {
              for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
              {
                m_pcInterSearch->setLicParam(list, comp, cu.licScale[list][comp], cu.licOffset[list][comp]);
              }
            }
          }
        }
#endif
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
        }
#endif
        xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, 0
                              , 0
                              , &equBcwCost
        );
      }
      else
      {
        CHECK(prevCS != bestCS, "this is not possible");
        tempCS->initStructData(encTestMode.qp);
        tempCS->copyStructure(*bestCS, partitioner.chType);
        CodingUnit * cuBest = tempCS->getCU(partitioner.chType);
        cuBest->obmcFlag = false;
        PelUnitBuf predBuf = tempCS->getPredBuf(*cuBest);
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
        if (PU::checkDoAffineBdofRefine(*(cuBest->firstPU), m_pcInterSearch))
        {
          predBuf.copyFrom(tempWoOBMCBuf);
        }
        else
        {
#endif
        m_pcInterSearch->motionCompensation( *(cuBest->firstPU), predBuf, REF_PIC_LIST_X );
#if JVET_AD0213_LIC_IMP
        if (cuBest->licFlag)
        {
          for (int list = 0; list < 2; list++)
          {
            if (cuBest->firstPU->refIdx[list] >= 0)
            {
              for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
              {
                m_pcInterSearch->setLicParam(list, comp, cuBest->licScale[list][comp], cuBest->licOffset[list][comp]);
              }
            }
          }
        }
#endif
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
        }
#endif
        xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, 0
                              , 0
                              , &equBcwCost
        );
      }
    }
  }
#if JVET_AD0213_LIC_IMP
  else if ((tempCS->area.lwidth() * tempCS->area.lheight() < 32) || tempCS->sps->getUseOBMC() == false)
  {
#else
  else if ((tempCS->area.lwidth() * tempCS->area.lheight() < 32) || (lic == true) || tempCS->sps->getUseOBMC() == false)
  {
    CHECK(cu.obmcFlag == true, "this is not possible");
#endif
    xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, 0
                          , 0
                          , &equBcwCost
    );
  }
  else
  {
#endif
#if ENABLE_OBMC //normal inter
  const unsigned wIdx = gp_sizeIdxInfo->idxFrom(partitioner.currArea().lwidth());
  CodingStructure *prevCS = tempCS;
  PelUnitBuf tempWoOBMCBuf = m_tempWoOBMCBuffer.subBuf(UnitAreaRelative(cu, cu));
  tempWoOBMCBuf.copyFrom(tempCS->getPredBuf(cu));
  cu.isobmcMC = true;
#if JVET_AC0158_PIXEL_AFFINE_MC && JVET_AD0213_LIC_IMP
  CHECK(!cu.obmcFlag, "obmcFlag is false for regualr inter");
#else
  cu.obmcFlag = true;
#endif
  m_pcInterSearch->subBlockOBMC(*cu.firstPU);
  cu.isobmcMC = false;
#endif
  xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, 0
                        , 0
                        , &equBcwCost
  );
#if ENABLE_OBMC // xCheckRDCostInter
  double tempCost = (prevCS == tempCS) ? tempCS->cost : bestCS->cost;
#if JVET_AC0158_PIXEL_AFFINE_MC
  if (m_pTempCUWoOBMC && tempCost < bestOBMCCost && tempCS->sps->getUseOBMC() == true)
#else
  if (m_pTempCUWoOBMC && tempCost < bestOBMCCost)
#endif
  {
    const unsigned hIdx = gp_sizeIdxInfo->idxFrom(prevCS->area.lheight());
    m_pTempCUWoOBMC[wIdx][hIdx]->clearCUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->clearPUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->clearTUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->copyStructure(*prevCS, partitioner.chType);

    m_pPredBufWoOBMC[wIdx][hIdx].copyFrom(tempWoOBMCBuf);
    m_pTempCUWoOBMC[wIdx][hIdx]->getPredBuf(cu).copyFrom(prevCS->getPredBuf(cu));

    bestOBMCCost = tempCost;
#if JVET_AA0129_INTERHASH_OBMCOFF_RD
    validMode = true;
#endif
  }
#endif
#if JVET_AC0158_PIXEL_AFFINE_MC
  }
#endif
  if( g_bcwSearchOrder[bcwLoopIdx] == BCW_DEFAULT )
    m_pcInterSearch->setAffineModeSelected((bestCS->cus.front()->affine && !(bestCS->cus.front()->firstPU->mergeFlag)));

  tempCS->initStructData(encTestMode.qp);

  double skipTH = MAX_DOUBLE;
  skipTH = (m_pcEncCfg->getUseBcwFast() ? 1.05 : MAX_DOUBLE);
  if( equBcwCost > curBestCost * skipTH )
  {
    break;
  }

  if( m_pcEncCfg->getUseBcwFast() )
  {
    if( isEqualUni == true && m_pcEncCfg->getIntraPeriod() == -1 )
    {
      break;
    }
  }
  if( g_bcwSearchOrder[bcwLoopIdx] == BCW_DEFAULT && xIsBcwSkip(cu) && m_pcEncCfg->getUseBcwFast() )
  {
    break;
  }
 }  // for( UChar bcwLoopIdx = 0; bcwLoopIdx < bcwLoopNum; bcwLoopIdx++ )
#if JVET_AG0276_LIC_SLOPE_ADJUST
  }
#endif
  if ( m_bestModeUpdated && bestCS->cost != MAX_DOUBLE )
  {
    xCalDebCost( *bestCS, partitioner );
  }
#if JVET_X0083_BM_AMVP_MERGE_MODE
  }
#endif
#if ENABLE_OBMC && JVET_AA0129_INTERHASH_OBMCOFF_RD
  return (m_pcEncCfg->getUseHashME() ? validMode : true);
#endif
}




bool EncCu::xCheckRDCostInterIMV(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode, double &bestIntPelCost)
{
#if JVET_X0083_BM_AMVP_MERGE_MODE
  bool bdmvrAmMergeNotValid = false;
#endif
#if ENABLE_OBMC
  double bestOBMCCost = MAX_DOUBLE;
#endif
  int iIMV = int( ( encTestMode.opts & ETO_IMV ) >> ETO_IMV_SHIFT );
  m_pcInterSearch->setAffineModeSelected(false);
#if JVET_AD0213_LIC_IMP
  m_pcInterSearch->setDoAffineLic(true);
#endif
  // Only Half-Pel, int-Pel, 4-Pel and fast 4-Pel allowed
  CHECK(iIMV < 1 || iIMV > 4, "Unsupported IMV Mode");
  const bool testAltHpelFilter = iIMV == 4;
  // Fast 4-Pel Mode

#if INTER_LIC
  if (m_pcInterSearch->m_fastLicCtrl.skipRDCheckForLIC((encTestMode.opts & ETO_LIC) > 0, (iIMV <= 2 ? iIMV : iIMV - 1), bestCS->cost, tempCS->area.Y().area()))
  {
    return false;
  }
#endif

  m_bestModeUpdated = tempCS->useDbCost = bestCS->useDbCost = false;

  EncTestMode encTestModeBase = encTestMode;                                        // copy for clearing non-IMV options
  encTestModeBase.opts        = EncTestModeOpts( encTestModeBase.opts & ETO_IMV );  // clear non-IMV options (is that intended?)

  tempCS->initStructData( encTestMode.qp );

#if INTER_LIC
  bool lic = encTestMode.opts & ETO_LIC;
#endif

  m_pcInterSearch->resetBufferedUniMotions();
  int bcwLoopNum = (tempCS->slice->isInterB() ? BCW_NUM : 1);
  bcwLoopNum = (tempCS->slice->getSPS()->getUseBcw() ? bcwLoopNum : 1);
#if INTER_LIC && !JVET_AD0213_LIC_IMP
  bcwLoopNum = lic ? 1 : bcwLoopNum;
#endif

  if( tempCS->area.lwidth() * tempCS->area.lheight() < BCW_SIZE_CONSTRAINT )
  {
    bcwLoopNum = 1;
  }

  bool validMode = false;
#if ENABLE_OBMC && JVET_AA0129_INTERHASH_OBMCOFF_RD
  bool availMode = false;
#endif
  double curBestCost = bestCS->cost;
  double equBcwCost = MAX_DOUBLE;
#if JVET_AG0276_LIC_SLOPE_ADJUST
  bool testLicSlopeAdjust = tempCS->sps->getLicSlopeAdjustEnabledFlag() && lic;
  testLicSlopeAdjust &= (tempCS->area.Y().x || tempCS->area.Y().y);
  testLicSlopeAdjust &= CU::licSlopeSizeTlCond(tempCS->area.Y().width, tempCS->area.Y().height, tempCS->slice->getTLayer());
  int licDeltaE = testLicSlopeAdjust ? LIC_SLOPE_MAX_NUM_DELTA : 0;
  for (int licDeltaIdx = 0; licDeltaIdx <= licDeltaE; licDeltaIdx++)
  {
    int licDelta = g_licSlopeDeltaSet[licDeltaIdx];
    bcwLoopNum = licDelta == 0 ? bcwLoopNum : 1;
    if (licDelta != 0 && iIMV > 1)
    {
      if (!bestCS->cus.front()->licFlag)
      {
        continue;
      }
    }
    if (licDelta != 0)
    {
      ComprCUCtx cuECtx = m_modeCtrl->getComprCUCtx();
      double licCost = cuECtx.get<double>(BEST_LIC_COST);
      if (licCost != MAX_DOUBLE * .5 && m_pcEncCfg->getFastLicAffine() && ((licCost > bestCS->cost * 1.5 && m_pcEncCfg->getIntraPeriod() > 1) || (licCost > bestCS->cost * 1.4 && m_pcEncCfg->getIntraPeriod() < 0)))
      {
        continue;
      }
    }
#endif
  for( int bcwLoopIdx = 0; bcwLoopIdx < bcwLoopNum; bcwLoopIdx++ )
  {
    if( m_pcEncCfg->getUseBcwFast() )
    {
      auto blkCache = dynamic_cast< CacheBlkInfoCtrl* >(m_modeCtrl);

      if( blkCache )
      {
        bool isBestInter = blkCache->getInter(bestCS->area);
        uint8_t bestBcwIdx = blkCache->getBcwIdx(bestCS->area);

        if( isBestInter && g_bcwSearchOrder[bcwLoopIdx] != BCW_DEFAULT && g_bcwSearchOrder[bcwLoopIdx] != bestBcwIdx )
        {
          continue;
        }
      }
    }

    if( !tempCS->slice->getCheckLDC() )
    {
      if( bcwLoopIdx != 0 && bcwLoopIdx != 3 && bcwLoopIdx != 4 )
      {
        continue;
      }
    }

#if JVET_AD0213_LIC_IMP
    if (!lic)
    {
#endif
    if( m_pcEncCfg->getUseBcwFast() && tempCS->slice->getCheckLDC() && g_bcwSearchOrder[bcwLoopIdx] != BCW_DEFAULT
      && (m_bestBcwIdx[0] >= 0 && g_bcwSearchOrder[bcwLoopIdx] != m_bestBcwIdx[0])
      && (m_bestBcwIdx[1] >= 0 && g_bcwSearchOrder[bcwLoopIdx] != m_bestBcwIdx[1]))
    {
      continue;
    }
#if JVET_AD0213_LIC_IMP
    }
    if (m_pcEncCfg->getFastLicBcw() && lic && g_bcwSearchOrder[bcwLoopIdx] != BCW_DEFAULT)
    {
      continue;
    }
#endif

  CodingUnit &cu = tempCS->addCU( tempCS->area, partitioner.chType );

  partitioner.setCUData( cu );
  cu.slice            = tempCS->slice;
  cu.tileIdx          = tempCS->pps->getTileIdx( tempCS->area.lumaPos() );
  cu.skip             = false;
  cu.mmvdSkip = false;
//cu.affine
  cu.predMode         = MODE_INTER;
#if INTER_LIC
  cu.licFlag          = lic;
#if JVET_AG0276_LIC_SLOPE_ADJUST
  cu.licDelta        = licDelta;
#endif
#endif
  cu.chromaQpAdj      = m_cuChromaQpOffsetIdxPlus1;
  cu.qp               = encTestMode.qp;

  CU::addPUs( cu );

  if (testAltHpelFilter)
  {
    cu.imv = IMV_HPEL;
  }
  else
  {
    cu.imv = iIMV == 1 ? IMV_FPEL : IMV_4PEL;
  }

  bool testBcw;
  uint8_t bcwIdx;
  bool affineAmvrEanbledFlag = !testAltHpelFilter && cu.slice->getSPS()->getAffineAmvrEnabledFlag();

  cu.bcwIdx = g_bcwSearchOrder[bcwLoopIdx];
  bcwIdx = cu.bcwIdx;
  testBcw = (bcwIdx != BCW_DEFAULT);

#if JVET_AC0158_PIXEL_AFFINE_MC && !JVET_AD0213_LIC_IMP
  if ((tempCS->area.lwidth() * tempCS->area.lheight() < 32) || (lic == true) || tempCS->sps->getUseOBMC() == false)
  {
    cu.obmcFlag = false;
  }
#endif

  cu.firstPU->interDir = MAX_UCHAR;


#if JVET_AD0140_MVD_PREDICTION
  cu.firstPU->mvdSuffixInfo.clear();
#endif

#if INTER_LIC
  if (cu.slice->getUseLIC() && lic) { m_pcInterSearch->swapUniMvBuffer(); }
#if JVET_AD0213_LIC_IMP
  if (m_pcEncCfg->getFastLicAffine())
  {
    m_pcInterSearch->setDoAffineLic(bestCS->cus.front()->affine || bestCS->cus.front()->licFlag);
  }
#if JVET_AG0276_LIC_SLOPE_ADJUST
  else if (licDelta != 0)
  {
    m_pcInterSearch->setDoAffineLic(bestCS->cus.front()->affine || bestCS->cus.front()->licFlag);
  }
#endif
#endif
#endif
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
  m_pcInterSearch->setBdofSubPuMvBuf(m_mvBufEncMhpAffineBDOF);
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
  m_pcInterSearch->predInterSearch( cu, partitioner, bdmvrAmMergeNotValid );
#else
  m_pcInterSearch->predInterSearch( cu, partitioner );
#endif
#if INTER_LIC
  if (cu.slice->getUseLIC() && lic) { m_pcInterSearch->swapUniMvBuffer(); }
#endif

  if ( cu.firstPU->interDir <= 3 )
  {
    bcwIdx = CU::getValidBcwIdx(cu);
  }
  else
  {
    return false;
  }

  if( m_pcEncCfg->getMCTSEncConstraint() && ( ( cu.firstPU->refIdx[L0] < 0 && cu.firstPU->refIdx[L1] < 0 ) || ( !( MCTSHelper::checkMvBufferForMCTSConstraint( *cu.firstPU ) ) ) ) )
  {
    // Do not use this mode
    tempCS->initStructData( encTestMode.qp );
    continue;
  }
  if( testBcw && bcwIdx == BCW_DEFAULT ) // Enabled Bcw but the search results is uni.
  {
    tempCS->initStructData(encTestMode.qp);
    continue;
  }
  CHECK(!(testBcw || (!testBcw && bcwIdx == BCW_DEFAULT)), " !( bTestBcw || (!bTestBcw && bcwIdx == BCW_DEFAULT ) )");

#if JVET_AG0276_LIC_SLOPE_ADJUST
  if (licDelta != 0 && cu.firstPU->interDir == 3)
  {
    tempCS->initStructData(encTestMode.qp);
    continue;
  }
#endif

  bool isEqualUni = false;
  if( m_pcEncCfg->getUseBcwFast() )
  {
    if( cu.firstPU->interDir != 3 && testBcw == 0 )
    {
      isEqualUni = true;
    }
  }
#if JVET_Y0128_NON_CTC && INTER_LIC && !JVET_AF0190_RPR_TMP_REORDER_LIC
  if (cu.licFlag)
  {
    if (!PU::checkRprLicCondition(*cu.firstPU)) // To check whether LIC actually performs in MC
    {
      cu.licFlag = false;
      PU::spanLicFlags(*cu.firstPU, false);
#if JVET_AD0213_LIC_IMP
      tempCS->initStructData(encTestMode.qp);
      continue;
#endif
    }
  }
#endif

  if ( !CU::hasSubCUNonZeroMVd( cu ) && !CU::hasSubCUNonZeroAffineMVd( cu ) )
  {
    if (m_modeCtrl->useModeResult(encTestModeBase, tempCS, partitioner))
    {
      std::swap(tempCS, bestCS);
      // store temp best CI for next CU coding
      m_CurrCtx->best = m_CABACEstimator->getCtx();
    }
    if ( affineAmvrEanbledFlag )
    {
      tempCS->initStructData( encTestMode.qp );
      continue;
    }
    else
    {
      return false;
    }
  }
#if JVET_Z0054_BLK_REF_PIC_REORDER
  PredictionUnit& pu = *cu.firstPU;
  if (PU::useRefCombList(pu))
  {
#if JVET_AD0140_MVD_PREDICTION
    pu.mvdSuffixInfo.clear();
#endif
    m_pcInterSearch->setUniRefIdxLC(pu);
  }
  else if (PU::useRefPairList(pu))
  {
#if JVET_AD0140_MVD_PREDICTION
    pu.mvdSuffixInfo.clear();
#endif
    m_pcInterSearch->setBiRefPairIdx(pu);
  }

#if JVET_AD0140_MVD_PREDICTION
  if (pu.cu->smvdMode)
  {

    const Mv& cMvd = pu.mvd[REF_PIC_LIST_0];

    if (pu.isMvdPredApplicable() && cMvd.isMvdPredApplicable())
    {
      if (MotionModel::Undefined == pu.mvdSuffixInfo.m_motionModel)
      {
        THROW("Undefined Motion Model for SMVD");
      }
    }
    else
    {
      pu.mvdSuffixInfo.m_motionModel = MotionModel::BiTranslationalSmvd;
      pu.mvdSuffixInfo.mvBins[REF_PIC_LIST_0][0].m_motionModel = MotionModel::BiTranslationalSmvd;
    }
  }
#endif
#endif
#if JVET_AC0158_PIXEL_AFFINE_MC
  if (cu.affine == true)
  {
#if JVET_AD0213_LIC_IMP
    if (tempCS->sps->getUseOBMC() == false)
    {
#else
    if (lic == true || tempCS->sps->getUseOBMC() == false)
    {
#endif
      xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, 0
                            , 0
                            , &equBcwCost
      );
    }
    else
    {
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
      PelUnitBuf tempWoOBMCBuf;
      if (PU::checkDoAffineBdofRefine(*cu.firstPU, m_pcInterSearch))
      {
        tempWoOBMCBuf = m_tempWoOBMCBuffer.subBuf(UnitAreaRelative(cu, cu));
        tempWoOBMCBuf.copyFrom(tempCS->getPredBuf(cu));
      }
#endif
      CodingStructure *prevCS = tempCS;
      cu.isobmcMC = true;
      m_pcInterSearch->subBlockOBMC(*cu.firstPU);
      cu.isobmcMC = false;
      xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, 0
                            , 0
                            , &equBcwCost
      );
      if (prevCS == tempCS)
      {
        cu.obmcFlag = false;
        PelUnitBuf predBuf = tempCS->getPredBuf(cu);
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
        if (PU::checkDoAffineBdofRefine(*cu.firstPU, m_pcInterSearch))
        {
          predBuf.copyFrom(tempWoOBMCBuf);
        }
        else
        {
#endif
        m_pcInterSearch->motionCompensation( *(cu.firstPU), predBuf, REF_PIC_LIST_X );
#if JVET_AD0213_LIC_IMP
        if (cu.licFlag)
        {
          for (int list = 0; list < 2; list++)
          {
            if (cu.firstPU->refIdx[list] >= 0)
            {
              for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
              {
                m_pcInterSearch->setLicParam(list, comp, cu.licScale[list][comp], cu.licOffset[list][comp]);
              }
            }
          }
        }
#endif
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
        }
#endif
        xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, 0
                              , 0
                              , &equBcwCost
        );
      }
      else
      {
        CHECK(prevCS != bestCS, "this is not possible");
        tempCS->initStructData(encTestMode.qp);
        tempCS->copyStructure(*bestCS, partitioner.chType);
        CodingUnit * cuBest = tempCS->getCU(partitioner.chType);
        cuBest->obmcFlag = false;
        PelUnitBuf predBuf = tempCS->getPredBuf(*cuBest);
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
        if (PU::checkDoAffineBdofRefine(*(cuBest->firstPU), m_pcInterSearch))
        {
          predBuf.copyFrom(tempWoOBMCBuf);
        }
        else
        {
#endif
        m_pcInterSearch->motionCompensation( *(cuBest->firstPU), predBuf, REF_PIC_LIST_X );
#if JVET_AD0213_LIC_IMP
        if (cuBest->licFlag)
        {
          for (int list = 0; list < 2; list++)
          {
            if (cuBest->firstPU->refIdx[list] >= 0)
            {
              for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
              {
                m_pcInterSearch->setLicParam(list, comp, cuBest->licScale[list][comp], cuBest->licOffset[list][comp]);
              }
            }
          }
        }
#endif
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
        }
#endif
        xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, 0
                              , 0
                              , &equBcwCost
        );
      }
    }
  }
#if JVET_AD0213_LIC_IMP
  else if ((tempCS->area.lwidth() * tempCS->area.lheight() < 32) || tempCS->sps->getUseOBMC() == false)
  {
#else
  else if ((tempCS->area.lwidth() * tempCS->area.lheight() < 32) || (lic == true) || tempCS->sps->getUseOBMC() == false)
  {
    CHECK(cu.obmcFlag == true, "this is not possible");
#endif
    xEncodeInterResidual( tempCS, bestCS, partitioner, encTestMode, 0
                          , 0
                          , &equBcwCost
    );
  }
  else
  {
#endif
#if ENABLE_OBMC //normal inter IMV
  CodingStructure *prevCS = tempCS;
  PelUnitBuf tempWoOBMCBuf = m_tempWoOBMCBuffer.subBuf(UnitAreaRelative(cu, cu));
  tempWoOBMCBuf.copyFrom(tempCS->getPredBuf(cu));
  cu.isobmcMC = true;
#if JVET_AC0158_PIXEL_AFFINE_MC && JVET_AD0213_LIC_IMP
  CHECK(!cu.obmcFlag, "obmcFlag is false for regular IMV");
#else
  cu.obmcFlag = true;
#endif
  m_pcInterSearch->subBlockOBMC(*cu.firstPU);
  cu.isobmcMC = false;
#endif
  xEncodeInterResidual( tempCS, bestCS, partitioner, encTestModeBase, 0
                        , 0
                        , &equBcwCost
  );
#if ENABLE_OBMC
  double tempCost = (prevCS == tempCS) ? tempCS->cost : bestCS->cost;
#if JVET_AC0158_PIXEL_AFFINE_MC
  if (m_pTempCUWoOBMC && tempCost < bestOBMCCost && tempCS->sps->getUseOBMC() == true)
#else
  if (m_pTempCUWoOBMC && tempCost < bestOBMCCost)
#endif
  {
    const unsigned wIdx = gp_sizeIdxInfo->idxFrom(tempCS->area.lwidth());
    const unsigned hIdx = gp_sizeIdxInfo->idxFrom(tempCS->area.lheight());

    m_pTempCUWoOBMC[wIdx][hIdx]->clearCUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->clearPUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->clearTUs();
    m_pTempCUWoOBMC[wIdx][hIdx]->copyStructure(*prevCS, partitioner.chType);

    m_pPredBufWoOBMC[wIdx][hIdx].copyFrom(tempWoOBMCBuf);
    m_pTempCUWoOBMC[wIdx][hIdx]->getPredBuf(cu).copyFrom(prevCS->getPredBuf(cu));

    bestOBMCCost = tempCost;
#if JVET_AA0129_INTERHASH_OBMCOFF_RD
    availMode = true;
#endif
  }
#endif
#if JVET_AC0158_PIXEL_AFFINE_MC
  }
#endif
  if( cu.imv == IMV_FPEL && tempCS->cost < bestIntPelCost )
  {
    bestIntPelCost = tempCS->cost;
  }
  tempCS->initStructData(encTestMode.qp);

  double skipTH = MAX_DOUBLE;
  skipTH = (m_pcEncCfg->getUseBcwFast() ? 1.05 : MAX_DOUBLE);
  if( equBcwCost > curBestCost * skipTH )
  {
    break;
  }

  if( m_pcEncCfg->getUseBcwFast() )
  {
    if( isEqualUni == true && m_pcEncCfg->getIntraPeriod() == -1 )
    {
      break;
    }
  }
  if( g_bcwSearchOrder[bcwLoopIdx] == BCW_DEFAULT && xIsBcwSkip(cu) && m_pcEncCfg->getUseBcwFast() )
  {
    break;
  }
  validMode = true;
 } // for( UChar bcwLoopIdx = 0; bcwLoopIdx < bcwLoopNum; bcwLoopIdx++ )
#if JVET_AG0276_LIC_SLOPE_ADJUST
  }
#endif

  if ( m_bestModeUpdated && bestCS->cost != MAX_DOUBLE )
  {
    xCalDebCost( *bestCS, partitioner );
  }

#if ENABLE_OBMC && JVET_AA0129_INTERHASH_OBMCOFF_RD
  if (m_pcEncCfg->getUseHashME())
  {
    return availMode;
  }
  else
#endif
  return tempCS->slice->getSPS()->getAffineAmvrEnabledFlag() ? validMode : true;
}

void EncCu::xCalDebCost( CodingStructure &cs, Partitioner &partitioner, bool calDist )
{
  if ( cs.cost == MAX_DOUBLE )
  {
    cs.costDbOffset = 0;
  }

  if ( cs.slice->getDeblockingFilterDisable() || ( !m_pcEncCfg->getUseEncDbOpt() && !calDist ) )
  {
    return;
  }

  m_pcLoopFilter->setEnc(true);
  const ChromaFormat format = cs.area.chromaFormat;
  CodingUnit*                cu = cs.getCU(partitioner.chType);
  const Position lumaPos = cu->Y().valid() ? cu->Y().pos() : recalcPosition( format, cu->chType, CHANNEL_TYPE_LUMA, cu->blocks[cu->chType].pos() );
  bool topEdgeAvai = lumaPos.y > 0 && ((lumaPos.y % 4) == 0);
  bool leftEdgeAvai = lumaPos.x > 0 && ((lumaPos.x % 4) == 0);
  bool anyEdgeAvai = topEdgeAvai || leftEdgeAvai;
  cs.costDbOffset = 0;

  if ( calDist )
  {
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    ComponentID compStr = (CS::isDualITree(cs) && !isLuma(partitioner.chType)) ? COMPONENT_Cb : COMPONENT_Y;
    ComponentID compEnd = ((CS::isDualITree(cs) && isLuma(partitioner.chType)) || cs.area.chromaFormat == CHROMA_400 ) ? COMPONENT_Y : COMPONENT_Cr;
#else
    ComponentID compStr = ( cu->isSepTree() && !isLuma( partitioner.chType ) ) ? COMPONENT_Cb : COMPONENT_Y;
    ComponentID compEnd = ( ( cu->isSepTree() && isLuma( partitioner.chType ) ) || cs.area.chromaFormat == CHROMA_400 ) ? COMPONENT_Y : COMPONENT_Cr;
#endif
    Distortion finalDistortion = 0;
    for ( int comp = compStr; comp <= compEnd; comp++ )
    {
      const ComponentID compID = ComponentID( comp );
      CPelBuf org = cs.getOrgBuf( compID );
      CPelBuf reco = cs.getRecoBuf( compID );
      finalDistortion += getDistortionDb( cs, org, reco, compID, cs.area.block( COMPONENT_Y ), false );
    }
    //updated distortion
    cs.dist = finalDistortion;
  }

  if ( anyEdgeAvai && m_pcEncCfg->getUseEncDbOpt() )
  {
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    ComponentID compStr = (CS::isDualITree(cs) && !isLuma(partitioner.chType)) ? COMPONENT_Cb : COMPONENT_Y;
    ComponentID compEnd = ((CS::isDualITree(cs) && isLuma(partitioner.chType)) || cs.area.chromaFormat == CHROMA_400) ? COMPONENT_Y : COMPONENT_Cr;
#else
    ComponentID compStr = ( cu->isSepTree() && !isLuma( partitioner.chType ) ) ? COMPONENT_Cb : COMPONENT_Y;
    ComponentID compEnd = ( ( cu->isSepTree() &&  isLuma( partitioner.chType ) ) || cs.area.chromaFormat == CHROMA_400 ) ? COMPONENT_Y : COMPONENT_Cr;
#endif
    const UnitArea currCsArea = clipArea( cs.area, *cs.picture );

    PelStorage&          picDbBuf = m_pcLoopFilter->getDbEncPicYuvBuffer();

    //deblock neighbour pixels
    const Size     lumaSize = cu->Y().valid() ? cu->Y().size() : recalcSize( format, cu->chType, CHANNEL_TYPE_LUMA, cu->blocks[cu->chType].size() );

    const int verOffset = lumaPos.y > 7 ? 8 : 4;
    const int horOffset = lumaPos.x > 7 ? 8 : 4;
    const UnitArea areaTop(  format, Area( lumaPos.x, lumaPos.y - verOffset, lumaSize.width, verOffset  ) );
    const UnitArea areaLeft( format, Area( lumaPos.x - horOffset, lumaPos.y, horOffset, lumaSize.height ) );
    for ( int compIdx = compStr; compIdx <= compEnd; compIdx++ )
    {
      ComponentID compId = (ComponentID)compIdx;

      //Copy current CU's reco to Deblock Pic Buffer
      const CompArea&  curCompArea = currCsArea.block( compId );

      if( cs.slice->getLmcsEnabledFlag() && m_pcReshape->getSliceReshaperInfo().getUseSliceReshaper() && isLuma( compId ) )
      {
        picDbBuf.getBuf( curCompArea ).rspSignal( cs.getRecoBuf( curCompArea ), m_pcReshape->getInvLUT() );
      }
      else
      {
        picDbBuf.getBuf( curCompArea ).copyFrom( cs.getRecoBuf( curCompArea ) );
      }

      //left neighbour
      if ( leftEdgeAvai )
      {
        const CompArea&  compArea = areaLeft.block(compId);

        if( cs.slice->getLmcsEnabledFlag() && m_pcReshape->getSliceReshaperInfo().getUseSliceReshaper() && isLuma( compId ) )
        {
          picDbBuf.getBuf( compArea ).rspSignal( cs.picture->getRecoBuf( compArea ), m_pcReshape->getInvLUT() );
        }
        else
        {
          picDbBuf.getBuf( compArea ).copyFrom( cs.picture->getRecoBuf( compArea ) );
        }
      }
      //top neighbour
      if ( topEdgeAvai )
      {
        const CompArea&  compArea = areaTop.block( compId );

        if( cs.slice->getLmcsEnabledFlag() && m_pcReshape->getSliceReshaperInfo().getUseSliceReshaper() && isLuma( compId ) )
        {
          picDbBuf.getBuf( compArea ).rspSignal( cs.picture->getRecoBuf( compArea ), m_pcReshape->getInvLUT() );
        }
        else
        {
          picDbBuf.getBuf( compArea ).copyFrom( cs.picture->getRecoBuf( compArea ) );
        }
      }
    }
    
#if JVET_V0094_BILATERAL_FILTER
    // Bilateral:
    // The CU itself, the above area and the area to the left have been copied into
    //     PelStorage&          picDbBuf = m_pcLoopFilter->getDbEncPicYuvBuffer();
    //  It is now possible to insert the code for bilateral filtering here.
    
    if( cs.pps->getUseBIF() && ( !CS::isDualITree( cs ) || isLuma( partitioner.chType ) ) )
    {
      if( leftEdgeAvai && topEdgeAvai )
      {
        Pel* pDst = picDbBuf.bufs[COMPONENT_Y].buf + ( currCsArea.block( COMPONENT_Y ).y - 1 ) * picDbBuf.bufs[COMPONENT_Y].stride + currCsArea.block( COMPONENT_Y ).x - 1;
        Pel* pSrc = cs.picture->getRecoBuf().bufs[COMPONENT_Y].buf + ( currCsArea.block( COMPONENT_Y ).y - 1) * cs.picture->getRecoBuf().bufs[COMPONENT_Y].stride + currCsArea.block( COMPONENT_Y ).x - 1;
        if ( cs.slice->getLmcsEnabledFlag() && m_pcReshape->getSliceReshaperInfo().getUseSliceReshaper() )
        {
          *pDst = m_pcReshape->getInvLUT()[*pSrc];
        }
        else
        {
          *pDst = *pSrc;
        }        
      }

      for (auto &currTU : CU::traverseTUs(*cu))
      {
#if JVET_AF0112_BIF_DYNAMIC_SCALING
        bool applyBIF = m_bilateralFilter->getApplyBIF(currTU, COMPONENT_Y);
#else
        bool isInter = (cu->predMode == MODE_INTER) ? true : false;
        bool applyBIF = ((TU::getCbf(currTU, COMPONENT_Y) || isInter == false) && (currTU.cu->qp > 17)) && (128 > std::max(currTU.lumaSize().width, currTU.lumaSize().height)) && ((isInter == false) || (32 > std::min(currTU.lumaSize().width, currTU.lumaSize().height)));
#endif
        if (applyBIF)
        {
          CompArea& compArea = currTU.block(COMPONENT_Y);
          PelBuf    recBuf = picDbBuf.getBuf(compArea);
          PelBuf recIPredBuf = recBuf;
          std::vector<Pel> invLUT;
          m_bilateralFilter->bilateralFilterRDOdiamond5x5(COMPONENT_Y, recBuf, recBuf, recBuf, currTU.cu->qp, recIPredBuf, cs.slice->clpRng(COMPONENT_Y), currTU, true, false, &invLUT);
        }
      }
    }
#endif
#if JVET_X0071_CHROMA_BILATERAL_FILTER
    if(cs.pps->getUseChromaBIF())
    {
      bool applyChromaBIF = false;

      if (leftEdgeAvai && topEdgeAvai)
      {
        Pel* pDst = picDbBuf.bufs[COMPONENT_Cb].buf + ( currCsArea.block( COMPONENT_Cb ).y - 1 ) * picDbBuf.bufs[COMPONENT_Cb].stride + currCsArea.block( COMPONENT_Cb ).x - 1;
        Pel* pSrc = cs.picture->getRecoBuf().bufs[COMPONENT_Cb].buf + ( currCsArea.block( COMPONENT_Cb ).y - 1) * cs.picture->getRecoBuf().bufs[COMPONENT_Cb].stride + currCsArea.block( COMPONENT_Cb ).x - 1;
        *pDst = *pSrc;

        pDst = picDbBuf.bufs[COMPONENT_Cr].buf + ( currCsArea.block( COMPONENT_Cr ).y - 1 ) * picDbBuf.bufs[COMPONENT_Cr].stride + currCsArea.block( COMPONENT_Cr ).x - 1;
        pSrc = cs.picture->getRecoBuf().bufs[COMPONENT_Cr].buf + ( currCsArea.block( COMPONENT_Cr ).y - 1) * cs.picture->getRecoBuf().bufs[COMPONENT_Cr].stride + currCsArea.block( COMPONENT_Cr ).x - 1;
        *pDst = *pSrc;
      }

      for (auto &currTU : CU::traverseTUs(*cu))
      {
        for(int compIdx = COMPONENT_Cb; compIdx < MAX_NUM_COMPONENT; compIdx++)
        {
          ComponentID compID = ComponentID( compIdx );
#if JVET_AF0112_BIF_DYNAMIC_SCALING
          applyChromaBIF = m_bilateralFilter->getApplyBIF(currTU, compID);
#else
          bool tuValid = false;
          bool tuCBF = false;
          bool isInter = (cu->predMode == MODE_INTER) ? true : false;
          bool isDualTree = CS::isDualITree(cs);
          bool chromaValid = cu->Cb().valid() && cu->Cr().valid();
          applyChromaBIF = false;
          if(!isDualTree && chromaValid)
          {
            tuValid = currTU.blocks[compIdx].valid();
            tuCBF = false;
            if(tuValid)
            {
              tuCBF = TU::getCbf(currTU, compID);
            }
            applyChromaBIF = ((tuCBF || isInter == false) && (currTU.cu->qp > 17) && (tuValid));
          }

          if(isDualTree && chromaValid)
          {
            tuCBF = TU::getCbf(currTU, compID);
            applyChromaBIF = ((tuCBF || isInter == false) && (currTU.cu->qp > 17));
          }
#endif
          if (applyChromaBIF)
          {
            CompArea &compArea = currTU.block(compID);
            PelBuf    recBuf = picDbBuf.getBuf(compArea);
            PelBuf recIPredBuf = recBuf;
            m_bilateralFilter->bilateralFilterRDOdiamond5x5( compID, recBuf, recBuf, recBuf, currTU.cu->qp, recIPredBuf, cs.slice->clpRng(compID), currTU, true );
          }
        }
      }
    }
#endif
    
    //deblock
    if ( leftEdgeAvai )
    {
      m_pcLoopFilter->resetFilterLengths();
      m_pcLoopFilter->xDeblockCU( *cu, EDGE_VER );
    }

    if (topEdgeAvai)
    {
      m_pcLoopFilter->resetFilterLengths();
      m_pcLoopFilter->xDeblockCU( *cu, EDGE_HOR );
    }

    //update current CU SSE
    Distortion distCur = 0;
    for ( int compIdx = compStr; compIdx <= compEnd; compIdx++ )
    {
      ComponentID compId = (ComponentID)compIdx;
      CPelBuf reco = picDbBuf.getBuf( currCsArea.block( compId ) );
      CPelBuf org = cs.getOrgBuf( compId );
      distCur += getDistortionDb( cs, org, reco, compId, currCsArea.block( COMPONENT_Y ), true );
    }

    //calculate difference between DB_before_SSE and DB_after_SSE for neighbouring CUs
    Distortion distBeforeDb = 0, distAfterDb = 0;
    for (int compIdx = compStr; compIdx <= compEnd; compIdx++)
    {
      ComponentID compId = (ComponentID)compIdx;
      if ( leftEdgeAvai )
      {
        const CompArea&  compArea = areaLeft.block( compId );
        CPelBuf org = cs.picture->getOrigBuf( compArea );
        CPelBuf reco = cs.picture->getRecoBuf( compArea );
        CPelBuf recoDb = picDbBuf.getBuf( compArea );
        distBeforeDb += getDistortionDb( cs, org, reco, compId, areaLeft.block( COMPONENT_Y ), false );
        distAfterDb += getDistortionDb( cs, org, recoDb, compId, areaLeft.block( COMPONENT_Y ), true );
      }
      if ( topEdgeAvai )
      {
        const CompArea&  compArea = areaTop.block( compId );
        CPelBuf org = cs.picture->getOrigBuf( compArea );
        CPelBuf reco = cs.picture->getRecoBuf( compArea );
        CPelBuf recoDb = picDbBuf.getBuf( compArea );
        distBeforeDb += getDistortionDb( cs, org, reco, compId, areaTop.block( COMPONENT_Y ), false );
        distAfterDb += getDistortionDb( cs, org, recoDb, compId, areaTop.block( COMPONENT_Y ), true );
      }
    }

    //updated cost
    int64_t distTmp = distCur - cs.dist + distAfterDb - distBeforeDb;
    int sign = distTmp < 0 ? -1 : 1;
    distTmp = distTmp < 0 ? -distTmp : distTmp;
    cs.costDbOffset = sign * m_pcRdCost->calcRdCost( 0, distTmp );
  }

  m_pcLoopFilter->setEnc( false );
}

Distortion EncCu::getDistortionDb( CodingStructure &cs, CPelBuf org, CPelBuf reco, ComponentID compID, const CompArea& compArea, bool afterDb )
{
  Distortion dist = 0;
#if WCG_EXT
  m_pcRdCost->setChromaFormat(cs.sps->getChromaFormatIdc());
  CPelBuf orgLuma = cs.picture->getOrigBuf( compArea );
  if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || (
    m_pcEncCfg->getLmcs() && (cs.slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())))
  {
    if ( compID == COMPONENT_Y && !afterDb && !m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled())
    {
      CompArea    tmpArea( COMPONENT_Y, cs.area.chromaFormat, Position( 0, 0 ), compArea.size() );
      PelBuf tmpRecLuma = m_tmpStorageLCU->getBuf( tmpArea );
      tmpRecLuma.rspSignal( reco, m_pcReshape->getInvLUT() );
      dist += m_pcRdCost->getDistPart( org, tmpRecLuma, cs.sps->getBitDepth( toChannelType( compID ) ), compID, DF_SSE_WTD, &orgLuma );
    }
    else
    {
      dist += m_pcRdCost->getDistPart( org, reco, cs.sps->getBitDepth( toChannelType( compID ) ), compID, DF_SSE_WTD, &orgLuma );
    }
  }
  else if (m_pcEncCfg->getLmcs() && cs.slice->getLmcsEnabledFlag() && cs.slice->isIntra()) //intra slice
  {
    if ( compID == COMPONENT_Y && afterDb )
    {
      CompArea    tmpArea( COMPONENT_Y, cs.area.chromaFormat, Position( 0, 0 ), compArea.size() );
      PelBuf tmpRecLuma = m_tmpStorageLCU->getBuf( tmpArea );
      tmpRecLuma.rspSignal( reco, m_pcReshape->getFwdLUT() );
      dist += m_pcRdCost->getDistPart( org, tmpRecLuma, cs.sps->getBitDepth( toChannelType( compID ) ), compID, DF_SSE );
    }
    else
    {
      if ((isChroma(compID) && m_pcEncCfg->getReshapeIntraCMD()))
      {
        dist += m_pcRdCost->getDistPart(org, reco, cs.sps->getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
      }
      else
      {
        dist += m_pcRdCost->getDistPart( org, reco, cs.sps->getBitDepth(toChannelType( compID ) ), compID, DF_SSE );
      }
    }
  }
  else
#endif
  {
    dist = m_pcRdCost->getDistPart( org, reco, cs.sps->getBitDepth( toChannelType( compID ) ), compID, DF_SSE );
  }
  return dist;
}

void EncCu::xEncodeInterResidual(   CodingStructure *&tempCS
                                  , CodingStructure *&bestCS
                                  , Partitioner &partitioner
                                  , const EncTestMode& encTestMode
                                  , int residualPass
                                  , bool* bestHasNonResi
                                  , double* equBcwCost
  )
{

  CodingUnit*            cu        = tempCS->getCU( partitioner.chType );
  double   bestCostInternal        = MAX_DOUBLE;
  double           bestCost        = bestCS->cost;
  double           bestCostBegin   = bestCS->cost;
  CodingUnit*      prevBestCU      = bestCS->getCU( partitioner.chType );
  uint8_t          prevBestSbt     = ( prevBestCU == nullptr ) ? 0 : prevBestCU->sbtInfo;
#if JVET_AA0133_INTER_MTS_OPT
  bool             prevBestMts     = (prevBestCU == nullptr) ? 0 : (prevBestCU->firstTU->mtsIdx[COMPONENT_Y] > MTS_SKIP)? true : false;
#endif
#if JVET_AG0061_INTER_LFNST_NSPT
  bool             prevBestLN      = (prevBestCU == nullptr) ? 0 : prevBestCU->lfnstIdx ? true : false;
#endif
  bool              swapped        = false; // avoid unwanted data copy
  bool             reloadCU        = false;

  const PredictionUnit& pu = *cu->firstPU;

  // clang-format off
  const int affineShiftTab[3] =
  {
    MV_PRECISION_INTERNAL - MV_PRECISION_QUARTER,
    MV_PRECISION_INTERNAL - MV_PRECISION_SIXTEENTH,
    MV_PRECISION_INTERNAL - MV_PRECISION_INT
  };

  const int normalShiftTab[NUM_IMV_MODES] =
  {
    MV_PRECISION_INTERNAL - MV_PRECISION_QUARTER,
    MV_PRECISION_INTERNAL - MV_PRECISION_INT,
    MV_PRECISION_INTERNAL - MV_PRECISION_4PEL,
    MV_PRECISION_INTERNAL - MV_PRECISION_HALF,
  };
  // clang-format on

  int mvShift;

  for (int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
  {
    if (pu.refIdx[refList] >= 0)
    {
      if (!cu->affine)
      {
        mvShift = normalShiftTab[cu->imv];
        Mv signaledmvd(pu.mvd[refList].getHor() >> mvShift, pu.mvd[refList].getVer() >> mvShift);
        if (!((signaledmvd.getHor() >= MVD_MIN) && (signaledmvd.getHor() <= MVD_MAX)) || !((signaledmvd.getVer() >= MVD_MIN) && (signaledmvd.getVer() <= MVD_MAX)))
          return;
      }
      else
      {
        for (int ctrlP = 1 + (cu->affineType == AFFINEMODEL_6PARAM); ctrlP >= 0; ctrlP--)
        {
          mvShift = affineShiftTab[cu->imv];
          Mv signaledmvd(pu.mvdAffi[refList][ctrlP].getHor() >> mvShift, pu.mvdAffi[refList][ctrlP].getVer() >> mvShift);
          if (!((signaledmvd.getHor() >= MVD_MIN) && (signaledmvd.getHor() <= MVD_MAX)) || !((signaledmvd.getVer() >= MVD_MIN) && (signaledmvd.getVer() <= MVD_MAX)))
          {
            return;
          }
        }
      }
    }
  }
  // avoid MV exceeding 18-bit dynamic range
  const int maxMv = 1 << 17;
  if (!cu->affine && !pu.mergeFlag)
  {
    if ( (pu.refIdx[0] >= 0 && (pu.mv[0].getAbsHor() >= maxMv || pu.mv[0].getAbsVer() >= maxMv))
      || (pu.refIdx[1] >= 0 && (pu.mv[1].getAbsHor() >= maxMv || pu.mv[1].getAbsVer() >= maxMv)))
    {
      return;
    }
  }
  if (cu->affine && !pu.mergeFlag)
  {
    for (int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
    {
      if (pu.refIdx[refList] >= 0)
      {
        for (int ctrlP = 1 + (cu->affineType == AFFINEMODEL_6PARAM); ctrlP >= 0; ctrlP--)
        {
          if (pu.mvAffi[refList][ctrlP].getAbsHor() >= maxMv || pu.mvAffi[refList][ctrlP].getAbsVer() >= maxMv)
          {
            return;
          }
        }
      }
    }
  }
#if JVET_AA0133_INTER_MTS_OPT
  m_pcInterSearch->setBestCost(bestCS->cost);
  cu->mtsFlag = false;
  const bool mtsAllowed = tempCS->sps->getUseInterMTS() && CU::isInter(*cu) && partitioner.currArea().lwidth() <= tempCS->sps->getInterMTSMaxSize() && partitioner.currArea().lheight() <= tempCS->sps->getInterMTSMaxSize();
#else
  const bool mtsAllowed = tempCS->sps->getUseInterMTS() && CU::isInter( *cu ) && partitioner.currArea().lwidth() <= MTS_INTER_MAX_CU_SIZE && partitioner.currArea().lheight() <= MTS_INTER_MAX_CU_SIZE;
#endif
#if JVET_AG0061_INTER_LFNST_NSPT
  cu->lfnstFlag           = false;
  cu->lfnstIdx            = 0;
  const bool lfnstAllowed = tempCS->sps->getUseLFNST() && CU::isInter(*cu)
                            && partitioner.currArea().lwidth() <= tempCS->sps->getMaxTbSize()
                            && partitioner.currArea().lheight() <= tempCS->sps->getMaxTbSize();
#endif

  uint8_t sbtAllowed = cu->checkAllowedSbt();
  //SBT resolution-dependent fast algorithm: not try size-64 SBT in RDO for low-resolution sequences (now resolution below HD)
  if( tempCS->pps->getPicWidthInLumaSamples() < (uint32_t)m_pcEncCfg->getSBTFast64WidthTh() )
  {
    sbtAllowed = ((cu->lwidth() > 32 || cu->lheight() > 32)) ? 0 : sbtAllowed;
  }
  uint8_t numRDOTried = 0;
  Distortion sbtOffDist = 0;
  bool    sbtOffRootCbf = 0;
  double  sbtOffCost      = MAX_DOUBLE;
  double  currBestCost = MAX_DOUBLE;
  bool    doPreAnalyzeResi = ( sbtAllowed || mtsAllowed ) && residualPass == 0;
#if JVET_AA0133_INTER_MTS_OPT
  double  mtsOffCost = MAX_DOUBLE;
#endif
#if JVET_AG0061_INTER_LFNST_NSPT
  double LNOffCost = MAX_DOUBLE;
#endif
  m_pcInterSearch->initTuAnalyzer();
  if( doPreAnalyzeResi )
  {
    m_pcInterSearch->calcMinDistSbt( *tempCS, *cu, sbtAllowed );
  }

  auto    slsSbt = dynamic_cast<SaveLoadEncInfoSbt*>( m_modeCtrl );
  int     slShift = 4 + std::min( (int)gp_sizeIdxInfo->idxFrom( cu->lwidth() ) + (int)gp_sizeIdxInfo->idxFrom( cu->lheight() ), 9 );
  Distortion curPuSse = m_pcInterSearch->getEstDistSbt( NUMBER_SBT_MODE );
  uint8_t currBestSbt = 0;
  uint8_t currBestTrs = MAX_UCHAR;
  uint8_t histBestSbt = MAX_UCHAR;
  uint8_t histBestTrs = MAX_UCHAR;
  m_pcInterSearch->setHistBestTrs( MAX_UCHAR, MAX_UCHAR );
  if( doPreAnalyzeResi )
  {
    if( m_pcInterSearch->getSkipSbtAll() && !mtsAllowed ) //emt is off
    {
      histBestSbt = 0; //try DCT2
      m_pcInterSearch->setHistBestTrs( histBestSbt, histBestTrs );
    }
    else
    {
      assert( curPuSse != std::numeric_limits<uint64_t>::max() );
      uint16_t compositeSbtTrs = slsSbt->findBestSbt( cu->cs->area, (uint32_t)( curPuSse >> slShift ) );
      histBestSbt = ( compositeSbtTrs >> 0 ) & 0xff;
      histBestTrs = ( compositeSbtTrs >> 8 ) & 0xff;
      if( m_pcInterSearch->getSkipSbtAll() && CU::isSbtMode( histBestSbt ) ) //special case, skip SBT when loading SBT
      {
        histBestSbt = 0; //try DCT2
      }
      m_pcInterSearch->setHistBestTrs( histBestSbt, histBestTrs );
    }
  }

  {
    if( reloadCU )
    {
      if( bestCost == bestCS->cost ) //The first EMT pass didn't become the bestCS, so we clear the TUs generated
      {
        tempCS->clearTUs();
      }
      else if( false == swapped )
      {
        tempCS->initStructData( encTestMode.qp );
        tempCS->copyStructure( *bestCS, partitioner.chType );
        tempCS->getPredBuf().copyFrom( bestCS->getPredBuf() );
        bestCost = bestCS->cost;
        cu       = tempCS->getCU( partitioner.chType );
        swapped = true;
      }
      else
      {
        tempCS->clearTUs();
        bestCost = bestCS->cost;
        cu       = tempCS->getCU( partitioner.chType );
      }

      //we need to restart the distortion for the new tempCS, the bit count and the cost
      tempCS->dist     = 0;
      tempCS->fracBits = 0;
      tempCS->cost     = MAX_DOUBLE;
      tempCS->costDbOffset = 0;
    }

    reloadCU    = true; // enable cu reloading
    cu->skip    = false;
    cu->sbtInfo = 0;
#if JVET_AG0061_INTER_LFNST_NSPT
    cu->lfnstFlag = false;
    cu->lfnstIdx  = 0;
#endif
    const bool skipResidual = residualPass == 1;
    if( skipResidual || histBestSbt == MAX_UCHAR || !CU::isSbtMode( histBestSbt ) )
    {
    m_pcInterSearch->encodeResAndCalcRdInterCU( *tempCS, partitioner, skipResidual );
    if (tempCS->slice->getSPS()->getUseColorTrans())
    {
      bestCS->tmpColorSpaceCost = tempCS->tmpColorSpaceCost;
      bestCS->firstColorSpaceSelected = tempCS->firstColorSpaceSelected;
    }
#if JVET_AA0133_INTER_MTS_OPT
    numRDOTried += 1;
#else
    numRDOTried += mtsAllowed ? 2 : 1;
#endif
    xEncodeDontSplit( *tempCS, partitioner );

    xCheckDQP( *tempCS, partitioner );
    xCheckChromaQPOffset( *tempCS, partitioner );


    if( NULL != bestHasNonResi && (bestCostInternal > tempCS->cost) )
    {
      bestCostInternal = tempCS->cost;
      if (!(tempCS->getPU(partitioner.chType)->ciipFlag))
      *bestHasNonResi  = !cu->rootCbf;
    }

    if (cu->rootCbf == false)
    {
      if (tempCS->getPU(partitioner.chType)->ciipFlag)
      {
        tempCS->cost = MAX_DOUBLE;
        tempCS->costDbOffset = 0;
        return;
      }
    }
    currBestCost = tempCS->cost;
    sbtOffCost = tempCS->cost;
    sbtOffDist = tempCS->dist;
    sbtOffRootCbf = cu->rootCbf;
    currBestSbt = CU::getSbtInfo(cu->firstTU->mtsIdx[COMPONENT_Y] > MTS_SKIP ? SBT_OFF_MTS : SBT_OFF_DCT, 0);
    currBestTrs = cu->firstTU->mtsIdx[COMPONENT_Y];

#if WCG_EXT
    DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda( true ) );
#else
    DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );
#endif
    xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );

    }

    uint8_t numSbtRdo = CU::numSbtModeRdo( sbtAllowed );
    //early termination if all SBT modes are not allowed
    //normative
    if( !sbtAllowed || skipResidual )
    {
      numSbtRdo = 0;
    }
    //fast algorithm
    if( ( histBestSbt != MAX_UCHAR && !CU::isSbtMode( histBestSbt ) ) || m_pcInterSearch->getSkipSbtAll() )
    {
      numSbtRdo = 0;
    }
    if( bestCost != MAX_DOUBLE && sbtOffCost != MAX_DOUBLE )
    {
      double th = 1.07;
      if( !( prevBestSbt == 0 || m_sbtCostSave[0] == MAX_DOUBLE ) )
      {
        assert( m_sbtCostSave[1] <= m_sbtCostSave[0] );
        th *= ( m_sbtCostSave[0] / m_sbtCostSave[1] );
      }
      if( sbtOffCost > bestCost * th )
      {
        numSbtRdo = 0;
      }
    }
    if( !sbtOffRootCbf && sbtOffCost != MAX_DOUBLE )
    {
      double th = Clip3( 0.05, 0.55, ( 27 - cu->qp ) * 0.02 + 0.35 );
      if( sbtOffCost < m_pcRdCost->calcRdCost( ( cu->lwidth() * cu->lheight() ) << SCALE_BITS, 0 ) * th )
      {
        numSbtRdo = 0;
      }
    }

    if( histBestSbt != MAX_UCHAR && numSbtRdo != 0 )
    {
      numSbtRdo = 1;
      m_pcInterSearch->initSbtRdoOrder( CU::getSbtMode( CU::getSbtIdx( histBestSbt ), CU::getSbtPos( histBestSbt ) ) );
    }

    for( int sbtModeIdx = 0; sbtModeIdx < numSbtRdo; sbtModeIdx++ )
    {
      uint8_t sbtMode = m_pcInterSearch->getSbtRdoOrder( sbtModeIdx );
      uint8_t sbtIdx = CU::getSbtIdxFromSbtMode( sbtMode );
      uint8_t sbtPos = CU::getSbtPosFromSbtMode( sbtMode );

      //fast algorithm (early skip, save & load)
      if( histBestSbt == MAX_UCHAR )
      {
        uint8_t skipCode = m_pcInterSearch->skipSbtByRDCost( cu->lwidth(), cu->lheight(), cu->mtDepth, sbtIdx, sbtPos, bestCS->cost, sbtOffDist, sbtOffCost, sbtOffRootCbf );
        if( skipCode != MAX_UCHAR )
        {
          continue;
        }

        if( sbtModeIdx > 0 )
        {
          uint8_t prevSbtMode = m_pcInterSearch->getSbtRdoOrder( sbtModeIdx - 1 );
          //make sure the prevSbtMode is the same size as the current SBT mode (otherwise the estimated dist may not be comparable)
          if( CU::isSameSbtSize( prevSbtMode, sbtMode ) )
          {
            Distortion currEstDist = m_pcInterSearch->getEstDistSbt( sbtMode );
            Distortion prevEstDist = m_pcInterSearch->getEstDistSbt( prevSbtMode );
            if( currEstDist > prevEstDist * 1.15 )
            {
              continue;
            }
          }
        }
      }

      //init tempCS and TU
      if( bestCost == bestCS->cost ) //The first EMT pass didn't become the bestCS, so we clear the TUs generated
      {
        tempCS->clearTUs();
      }
      else if( false == swapped )
      {
        tempCS->initStructData( encTestMode.qp );
        tempCS->copyStructure( *bestCS, partitioner.chType );
        tempCS->getPredBuf().copyFrom( bestCS->getPredBuf() );
        bestCost = bestCS->cost;
        cu = tempCS->getCU( partitioner.chType );
        swapped = true;
      }
      else
      {
        tempCS->clearTUs();
        bestCost = bestCS->cost;
        cu = tempCS->getCU( partitioner.chType );
      }

      //we need to restart the distortion for the new tempCS, the bit count and the cost
      tempCS->dist = 0;
      tempCS->fracBits = 0;
      tempCS->cost = MAX_DOUBLE;
      cu->skip = false;
#if JVET_AG0061_INTER_LFNST_NSPT
      cu->lfnstFlag = false;
      cu->lfnstIdx  = 0;
#endif
      //set SBT info
      cu->setSbtIdx( sbtIdx );
      cu->setSbtPos( sbtPos );

      //try residual coding
      m_pcInterSearch->encodeResAndCalcRdInterCU( *tempCS, partitioner, skipResidual );
      if (tempCS->slice->getSPS()->getUseColorTrans())
      {
        bestCS->tmpColorSpaceCost = tempCS->tmpColorSpaceCost;
        bestCS->firstColorSpaceSelected = tempCS->firstColorSpaceSelected;
      }
      numRDOTried++;

      xEncodeDontSplit( *tempCS, partitioner );

      xCheckDQP( *tempCS, partitioner );
      xCheckChromaQPOffset( *tempCS, partitioner );

      if( NULL != bestHasNonResi && ( bestCostInternal > tempCS->cost ) )
      {
        bestCostInternal = tempCS->cost;
        if( !( tempCS->getPU( partitioner.chType )->ciipFlag ) )
          *bestHasNonResi = !cu->rootCbf;
      }

      if( tempCS->cost < currBestCost )
      {
        currBestSbt = cu->sbtInfo;
        currBestTrs = tempCS->tus[cu->sbtInfo ? cu->getSbtPos() : 0]->mtsIdx[COMPONENT_Y];
        assert( currBestTrs == 0 || currBestTrs == 1 );
        currBestCost = tempCS->cost;
      }

#if WCG_EXT
      DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda( true ) );
#else
      DTRACE_MODE_COST( *tempCS, m_pcRdCost->getLambda() );
#endif
      xCheckBestMode( tempCS, bestCS, partitioner, encTestMode );
    }
#if JVET_AA0133_INTER_MTS_OPT
    if (!skipResidual && mtsAllowed)
    {
#if !JVET_AG0061_INTER_LFNST_NSPT
      if (bestCost == bestCS->cost) //The first EMT pass didn't become the bestCS, so we clear the TUs generated
      {
        tempCS->clearTUs();
      }
      else if (false == swapped)
      {
        tempCS->initStructData(encTestMode.qp);
        tempCS->copyStructure(*bestCS, partitioner.chType);
        tempCS->getPredBuf().copyFrom(bestCS->getPredBuf());
        bestCost = bestCS->cost;
        cu = tempCS->getCU(partitioner.chType);
        swapped = true;
      }
      else
      {
        tempCS->clearTUs();
        bestCost = bestCS->cost;
        cu = tempCS->getCU(partitioner.chType);
      }

      //we need to restart the distortion for the new tempCS, the bit count and the cost
      tempCS->dist = 0;
      tempCS->fracBits = 0;
      tempCS->cost = MAX_DOUBLE;
      tempCS->costDbOffset = 0;
      cu->skip = false;
      cu->sbtInfo = 0;
      cu->mtsFlag = true;
#if JVET_AG0061_INTER_LFNST_NSPT
      cu->lfnstFlag = false;
      cu->lfnstIdx  = 0;
#endif
#endif
      m_pcInterSearch->setBestCost(bestCS->cost);
      mtsOffCost = currBestCost;
      bool testMts = true;
      if (bestCost != MAX_DOUBLE && mtsOffCost != MAX_DOUBLE)
      {
        double th = 1.07;
        if (!(prevBestMts == 0 || m_mtsCostSave == MAX_DOUBLE))
        {
          assert(m_sbtCostSave[1] <= m_mtsCostSave);
          th *= (m_mtsCostSave / m_sbtCostSave[1]);
        }
        if (mtsOffCost > bestCost * th)
        {
          testMts = false;
        }
      }
      if(testMts)
      {
#if JVET_AG0061_INTER_LFNST_NSPT
        if (bestCost == bestCS->cost)   // The first EMT pass didn't become the bestCS, so we clear the TUs generated
        {
          tempCS->clearTUs();
        }
        else if (false == swapped)
        {
          tempCS->initStructData(encTestMode.qp);
          tempCS->copyStructure(*bestCS, partitioner.chType);
          tempCS->getPredBuf().copyFrom(bestCS->getPredBuf());
          bestCost = bestCS->cost;
          cu       = tempCS->getCU(partitioner.chType);
          swapped  = true;
        }
        else
        {
          tempCS->clearTUs();
          bestCost = bestCS->cost;
          cu       = tempCS->getCU(partitioner.chType);
        }

        // we need to restart the distortion for the new tempCS, the bit count and the cost
        tempCS->dist         = 0;
        tempCS->fracBits     = 0;
        tempCS->cost         = MAX_DOUBLE;
        tempCS->costDbOffset = 0;
        cu->skip             = false;
        cu->sbtInfo          = 0;
        cu->mtsFlag          = true;
        cu->lfnstFlag        = false;
        cu->lfnstIdx         = 0;
#endif
      //try residual coding
        bool isValid = m_pcInterSearch->encodeResAndCalcRdInterCU(*tempCS, partitioner, skipResidual);
        if (isValid)
        {
          if (tempCS->slice->getSPS()->getUseColorTrans())
          {
            bestCS->tmpColorSpaceCost = tempCS->tmpColorSpaceCost;
            bestCS->firstColorSpaceSelected = tempCS->firstColorSpaceSelected;
          }
          numRDOTried++;

          xEncodeDontSplit(*tempCS, partitioner);

          xCheckDQP(*tempCS, partitioner);
          xCheckChromaQPOffset(*tempCS, partitioner);

          if (NULL != bestHasNonResi && (bestCostInternal > tempCS->cost))
          {
            bestCostInternal = tempCS->cost;
            if (!(tempCS->getPU(partitioner.chType)->ciipFlag))
            {
              *bestHasNonResi = !cu->rootCbf;
            }
          }

          if (cu->rootCbf == false)
          {
            if (tempCS->getPU(partitioner.chType)->ciipFlag)
            {
              tempCS->cost = MAX_DOUBLE;
              tempCS->costDbOffset = 0;
              return;
            }
          }
          if (tempCS->cost < currBestCost)
          {
            currBestCost = tempCS->cost;
            sbtOffCost = tempCS->cost;
            sbtOffDist = tempCS->dist;
            sbtOffRootCbf = cu->rootCbf;
            currBestSbt = CU::getSbtInfo(cu->firstTU->mtsIdx[COMPONENT_Y] > MTS_SKIP ? SBT_OFF_MTS : SBT_OFF_DCT, 0);
            currBestTrs = cu->firstTU->mtsIdx[COMPONENT_Y];
          }

  #if WCG_EXT
          DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda(true));
  #else
          DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda());
  #endif
          xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);
        }
      }
    }
#endif
    if( bestCostBegin != bestCS->cost )
    {
      m_sbtCostSave[0] = sbtOffCost;
      m_sbtCostSave[1] = currBestCost;
#if JVET_AA0133_INTER_MTS_OPT
      m_mtsCostSave = mtsOffCost;
#endif
    }
#if JVET_AG0061_INTER_LFNST_NSPT
    if (!skipResidual && lfnstAllowed)
    {
      /*fast algorithm*/
      LNOffCost   = currBestCost;
      bool testLN = true;
      if (bestCost != MAX_DOUBLE && LNOffCost != MAX_DOUBLE)
      {
        double th = std::max((1.0 + 1.0 / sqrt(cu->lwidth() * cu->lheight())), 1.07);
        if (!(prevBestLN == 0 || m_LNCostSave == MAX_DOUBLE))
        {
          assert(m_sbtCostSave[1] <= m_LNCostSave);
          th *= (m_LNCostSave / m_sbtCostSave[1]);
        }
        if (LNOffCost > bestCost * th)
        {
          testLN = false;
        }
      }
      if (testLN)
      {
        if (bestCost == bestCS->cost)   // The first EMT pass didn't become the bestCS, so we clear the TUs generated
        {
          tempCS->clearTUs();
        }
        else if (false == swapped)
        {
          tempCS->initStructData(encTestMode.qp);
          tempCS->copyStructure(*bestCS, partitioner.chType);
          tempCS->getPredBuf().copyFrom(bestCS->getPredBuf());
          bestCost = bestCS->cost;
          cu       = tempCS->getCU(partitioner.chType);
          swapped  = true;
        }
        else
        {
          tempCS->clearTUs();
          bestCost = bestCS->cost;
          cu       = tempCS->getCU(partitioner.chType);
        }
        cu->dimdDerivedIntraDir = m_pcInterSearch->deriveInterDimdMode(*cu, cu->cs->getPredBuf(*cu->firstPU).Y());
        // we need to restart the distortion for the new tempCS, the bit count and the cost
        tempCS->dist         = 0;
        tempCS->fracBits     = 0;
        tempCS->cost         = MAX_DOUBLE;
        tempCS->costDbOffset = 0;
        cu->skip             = false;
        cu->sbtInfo          = 0;
        cu->mtsFlag          = false;
        cu->lfnstFlag        = true;
        cu->lfnstIdx         = 0;
        m_pcInterSearch->setBestCost(bestCS->cost);
        {
          // try residual coding
          bool isValid = m_pcInterSearch->encodeResAndCalcRdInterCU(*tempCS, partitioner, skipResidual);
          if (isValid)
          {
            if (tempCS->slice->getSPS()->getUseColorTrans())
            {
              bestCS->tmpColorSpaceCost       = tempCS->tmpColorSpaceCost;
              bestCS->firstColorSpaceSelected = tempCS->firstColorSpaceSelected;
            }
            numRDOTried++;

            xEncodeDontSplit(*tempCS, partitioner);

            xCheckDQP(*tempCS, partitioner);
            xCheckChromaQPOffset(*tempCS, partitioner);

            if (NULL != bestHasNonResi && (bestCostInternal > tempCS->cost))
            {
              bestCostInternal = tempCS->cost;
              if (!(tempCS->getPU(partitioner.chType)->ciipFlag))
              {
                *bestHasNonResi = !cu->rootCbf;
              }
            }

#if JVET_AC0112_IBC_CIIP
            if ((tempCS->getCU(partitioner.chType))->skip && (tempCS->getCU(partitioner.chType))->firstPU->ibcCiipFlag)
            {
              tempCS->cost         = MAX_DOUBLE;
              tempCS->costDbOffset = 0;
            }
#endif
            if (cu->rootCbf == false)
            {
              if (tempCS->getPU(partitioner.chType)->ciipFlag)
              {
                tempCS->cost         = MAX_DOUBLE;
                tempCS->costDbOffset = 0;
                return;
              }
            }
            if (tempCS->cost < currBestCost)
            {
              currBestCost = tempCS->cost;
              currBestSbt  = CU::getSbtInfo(SBT_OFF_MTS, 0);
            }

#if WCG_EXT
            DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda(true));
#else
            DTRACE_MODE_COST(*tempCS, m_pcRdCost->getLambda());
#endif
            xCheckBestMode(tempCS, bestCS, partitioner, encTestMode);
          }
        }
      }
      if (bestCostBegin != bestCS->cost)
      {
        m_sbtCostSave[1] = currBestCost;
        m_LNCostSave     = LNOffCost;
      }
    }
#endif
  } //end emt loop

  if( histBestSbt == MAX_UCHAR && doPreAnalyzeResi && numRDOTried > 1 )
  {
    slsSbt->saveBestSbt( cu->cs->area, (uint32_t)( curPuSse >> slShift ), currBestSbt, currBestTrs );
  }
  tempCS->cost = currBestCost;
  if( ETM_INTER_ME == encTestMode.type )
  {
    if( equBcwCost != NULL )
    {
      if( tempCS->cost < ( *equBcwCost ) && cu->bcwIdx == BCW_DEFAULT )
      {
        ( *equBcwCost ) = tempCS->cost;
      }
    }
    else
    {
#if ENABLE_OBMC
      if (cu->obmcFlag)
#endif
      CHECK( equBcwCost == NULL, "equBcwCost == NULL" );
    }
#if JVET_AD0213_LIC_IMP
    if (!cu->licFlag)
    {
#endif
    if( tempCS->slice->getCheckLDC() && !cu->imv && cu->bcwIdx != BCW_DEFAULT && tempCS->cost < m_bestBcwCost[1] )
    {
      if( tempCS->cost < m_bestBcwCost[0] )
      {
        m_bestBcwCost[1] = m_bestBcwCost[0];
        m_bestBcwCost[0] = tempCS->cost;
        m_bestBcwIdx[1] = m_bestBcwIdx[0];
        m_bestBcwIdx[0] = cu->bcwIdx;
      }
      else
      {
        m_bestBcwCost[1] = tempCS->cost;
        m_bestBcwIdx[1] = cu->bcwIdx;
      }
    }
#if JVET_AD0213_LIC_IMP
    }
#endif
#if INTER_LIC
    m_pcInterSearch->m_fastLicCtrl.setBestAmvpRDBeforeLIC(*cu, currBestCost);
#endif
  }
}


void EncCu::xEncodeDontSplit( CodingStructure &cs, Partitioner &partitioner )
{
  m_CABACEstimator->resetBits();

  m_CABACEstimator->split_cu_mode( CU_DONT_SPLIT, cs, partitioner );
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( partitioner.treeType == TREE_C )
    CHECK( m_CABACEstimator->getEstFracBits() != 0, "must be 0 bit" );
#endif
  cs.fracBits += m_CABACEstimator->getEstFracBits(); // split bits
  cs.cost      = m_pcRdCost->calcRdCost( cs.fracBits, cs.dist );

}

#if REUSE_CU_RESULTS
void EncCu::xReuseCachedResult( CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner )
{
  m_pcRdCost->setChromaFormat(tempCS->sps->getChromaFormatIdc());
  BestEncInfoCache* bestEncCache = dynamic_cast<BestEncInfoCache*>( m_modeCtrl );
  CHECK( !bestEncCache, "If this mode is chosen, mode controller has to implement the mode caching capabilities" );
  EncTestMode cachedMode;

  if( bestEncCache->setCsFrom( *tempCS, cachedMode, partitioner ) )
  {
    CodingUnit& cu = *tempCS->cus.front();
    partitioner.setCUData( cu );
#if MULTI_PASS_DMVR
    for( auto &pu : CU::traversePUs( cu ) )
    {
      pu.bdmvrRefine = false;
    }
#endif
    if( CU::isIntra( cu )
    || CU::isPLT(cu)
    )
    {
      xReconIntraQT( cu );
    }
    else
    {
      xDeriveCUMV( cu );
      xReconInter( cu );
    }

#if JVET_Z0118_GDR
    bestCS->updateReconMotIPM(cu); // cache    
#endif

    Distortion finalDistortion = 0;
    tempCS->useDbCost = m_pcEncCfg->getUseEncDbOpt();
    if (!tempCS->slice->getDeblockingFilterDisable() && m_pcEncCfg->getUseEncDbOpt())
    {
      xCalDebCost( *tempCS, partitioner, true );
      finalDistortion = tempCS->dist;
    }
    else
    {
    const SPS &sps = *tempCS->sps;
    const int  numValidComponents = getNumberValidComponents( tempCS->area.chromaFormat );

    for( int comp = 0; comp < numValidComponents; comp++ )
    {
      const ComponentID compID = ComponentID( comp );
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if( CS::isDualITree(*tempCS) && toChannelType(compID) != partitioner.chType )
#else
      if( partitioner.isSepTree( *tempCS ) && toChannelType( compID ) != partitioner.chType )
#endif
      {
        continue;
      }

      CPelBuf reco = tempCS->getRecoBuf( compID );
      CPelBuf org  = tempCS->getOrgBuf ( compID );

      
#if JVET_V0094_BILATERAL_FILTER
        const CompArea &area = cu.blocks[COMPONENT_Y];
        CompArea    tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
        PelBuf tmpRecLuma = m_tmpStorageLCU->getBuf(tmpArea);
        if(isLuma(compID))
        {
          tmpRecLuma.copyFrom(reco);

          if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || (
            m_pcEncCfg->getLmcs() && (tempCS->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())))
          {
            tmpRecLuma.rspSignal(m_pcReshape->getInvLUT());
          }
        }
        if(tempCS->pps->getUseBIF() && isLuma(compID))
        {
          for (auto &currTU : CU::traverseTUs(cu))
          {
#if JVET_AF0112_BIF_DYNAMIC_SCALING
            bool applyBIF = m_bilateralFilter->getApplyBIF(currTU, compID);
#else
            bool isInter = (cu.predMode == MODE_INTER) ? true : false;
            bool applyBIF = ((TU::getCbf(currTU, COMPONENT_Y) || isInter == false) && (currTU.cu->qp > 17)) && (128 > std::max(currTU.lumaSize().width, currTU.lumaSize().height)) && ((isInter == false) || (32 > std::min(currTU.lumaSize().width, currTU.lumaSize().height)));
#endif
            if (applyBIF)
            {
              Position tuPosInCu = currTU.lumaPos() - cu.lumaPos();
              PelBuf tmpSubBuf = tmpRecLuma.subBuf(tuPosInCu, currTU.lumaSize());
              CompArea compArea = currTU.blocks[compID];
              PelBuf recIPredBuf = tempCS->slice->getPic()->getRecoBuf(compArea);
              // Do we need to use clipArea?
              
              // Only reshape surrounding samples if reshaping is on
              if( m_pcEncCfg->getLmcs() && (tempCS->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag()) && !(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()) )
              {
                m_bilateralFilter->bilateralFilterRDOdiamond5x5( compID, tmpSubBuf, tmpSubBuf, tmpSubBuf, currTU.cu->qp, recIPredBuf, tempCS->slice->clpRng( compID ), currTU, true, true, &m_pcReshape->getInvLUT() );
              }
              else
              {
                std::vector<Pel> invLUT;
                m_bilateralFilter->bilateralFilterRDOdiamond5x5( compID, tmpSubBuf, tmpSubBuf, tmpSubBuf, currTU.cu->qp, recIPredBuf, tempCS->slice->clpRng( compID ), currTU, true, false, &invLUT );
              }
            }
          }
        }
#if JVET_X0071_CHROMA_BILATERAL_FILTER
        const CompArea &areaChroma = cu.blocks[compID];
        CompArea    tmpAreaChroma(compID, areaChroma.chromaFormat, Position(0, 0), areaChroma.size());
        PelBuf tmpRecChroma;
        if(isChroma(compID))
        {
          tmpRecChroma = m_tmpStorageLCU->getBuf(tmpAreaChroma);
          tmpRecChroma.copyFrom(reco);
        }

        if(tempCS->pps->getUseChromaBIF() && isChroma(compID))
        {
          bool applyChromaBIF = false;
          for (auto &currTU : CU::traverseTUs(cu))
          {
#if JVET_AF0112_BIF_DYNAMIC_SCALING
            applyChromaBIF = m_bilateralFilter->getApplyBIF(currTU, compID);
#else
            bool tuValid = false;
            bool tuCBF = false;
            bool isInter = (cu.predMode == MODE_INTER) ? true : false;
            bool isDualTree = CS::isDualITree(*tempCS);
            bool chromaValid = cu.Cb().valid() && cu.Cr().valid();
            applyChromaBIF = false;
            if(!isDualTree && chromaValid)
            {
              tuValid = currTU.blocks[compID].valid();
              tuCBF = false;//if CHROMA TU is not vaild, CBF must be zero
              if(tuValid)
              {
                tuCBF = TU::getCbf(currTU, compID);
              }
              applyChromaBIF = (( tuCBF || isInter == false) && (currTU.cu->qp > 17) && (tuValid));
            }

            if(isDualTree && chromaValid)
            {
              applyChromaBIF = ((TU::getCbf(currTU, compID) || isInter == false) && (currTU.cu->qp > 17));
            }
#endif

            if(applyChromaBIF)
            {
              Position tuPosInCu = currTU.chromaPos() - cu.chromaPos();
              PelBuf tmpSubBuf = tmpRecChroma.subBuf(tuPosInCu, currTU.chromaSize());
              CompArea compArea = currTU.blocks[compID];
              PelBuf recIPredBuf = tempCS->slice->getPic()->getRecoBuf(compArea);
              m_bilateralFilter->bilateralFilterRDOdiamond5x5( compID, tmpSubBuf, tmpSubBuf, tmpSubBuf, currTU.cu->qp, recIPredBuf, tempCS->slice->clpRng(compID), currTU, true );
            }
          }
        }
#endif

#if WCG_EXT
        if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || (
          m_pcEncCfg->getLmcs() && (tempCS->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())))
        {
          const CPelBuf orgLuma = tempCS->getOrgBuf(tempCS->area.blocks[COMPONENT_Y]);
          if (compID == COMPONENT_Y && !(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()))
          {
            finalDistortion += m_pcRdCost->getDistPart(org, tmpRecLuma, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
          }
          else
          {
#if JVET_X0071_CHROMA_BILATERAL_FILTER
            if(isChroma(compID) && tempCS->pps->getUseChromaBIF())
            {
              finalDistortion += m_pcRdCost->getDistPart(org, tmpRecChroma, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
            }
            else
            {
              finalDistortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE_WTD, &orgLuma );
            }
#else
            finalDistortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE_WTD, &orgLuma );
#endif
          }
        }
        else
#endif
        {
          finalDistortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE );
        }
#else
#if JVET_X0071_CHROMA_BILATERAL_FILTER
        const CompArea &areaChroma = cu.blocks[compID];
        CompArea    tmpAreaChroma(compID, areaChroma.chromaFormat, Position(0, 0), areaChroma.size());
        PelBuf tmpRecChroma;
        if(isChroma(compID))
        {
          tmpRecChroma = m_tmpStorageLCU->getBuf(tmpAreaChroma);
          tmpRecChroma.copyFrom(reco);
        }

        if(tempCS->pps->getUseChromaBIF() && isChroma(compID))
        {
          bool tuValid = false;
          bool tuCBF = false;
          bool isDualTree = CS::isDualITree(*tempCS);
          bool chromaValid = cu.Cb().valid() && cu.Cr().valid();
          bool applyChromaBIF = false;

          for (auto &currTU : CU::traverseTUs(cu))
          {
#if JVET_AF0112_BIF_DYNAMIC_SCALING
            applyChromaBIF = m_bilateralFilter->getApplyBIF(currTU, compID);
#else
            bool isInter = (cu.predMode == MODE_INTER) ? true : false;
            applyChromaBIF = false;
            if(!isDualTree && chromaValid)
            {
              tuValid = currTU.blocks[compID].valid();
              tuCBF = false;//if CHROMA TU is not vaild, CBF must be zero
              if(tuValid)
              {
                tuCBF = TU::getCbf(currTU, compID);
              }
              applyChromaBIF = ((tuCBF || isInter == false) && (currTU.cu->qp > 17) && (tuValid));
            }
            if(isDualTree && chromaValid)
            {
              applyChromaBIF = ((TU::getCbf(currTU, compID) || isInter == false) && (currTU.cu->qp > 17));
            }
#endif
            if(applyChromaBIF)
            {
              Position tuPosInCu = currTU.chromaPos() - cu.chromaPos();
              PelBuf tmpSubBuf = tmpRecChroma.subBuf(tuPosInCu, currTU.chromaSize());
              CompArea compArea = currTU.blocks[compID];
              PelBuf recIPredBuf = tempCS->slice->getPic()->getRecoBuf(compArea);
              m_bilateralFilter->bilateralFilterRDOdiamond5x5( compID, tmpSubBuf, tmpSubBuf, tmpSubBuf, currTU.cu->qp, recIPredBuf, tempCS->slice->clpRng(compID), currTU, true );
            }
          }
        }
#endif
#if WCG_EXT
      if (m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled() || (m_pcEncCfg->getLmcs() && (tempCS->slice->getLmcsEnabledFlag() && m_pcReshape->getCTUFlag())))
      {
        const CPelBuf orgLuma = tempCS->getOrgBuf(tempCS->area.blocks[COMPONENT_Y]);
        if (compID == COMPONENT_Y && !(m_pcEncCfg->getLumaLevelToDeltaQPMapping().isEnabled()))
        {
          const CompArea &area = cu.blocks[COMPONENT_Y];
          CompArea    tmpArea(COMPONENT_Y, area.chromaFormat, Position(0, 0), area.size());
          PelBuf tmpRecLuma = m_tmpStorageLCU->getBuf(tmpArea);
          tmpRecLuma.rspSignal( reco, m_pcReshape->getInvLUT() );
          finalDistortion += m_pcRdCost->getDistPart(org, tmpRecLuma, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
        }
        else
        {
#if JVET_X0071_CHROMA_BILATERAL_FILTER
          if(isChroma(compID) && tempCS->pps->getUseChromaBIF())
          {
            finalDistortion += m_pcRdCost->getDistPart(org, tmpRecChroma, sps.getBitDepth(toChannelType(compID)), compID, DF_SSE_WTD, &orgLuma);
          }
          else
          {
            finalDistortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE_WTD, &orgLuma );
          }
#else
          finalDistortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE_WTD, &orgLuma );
#endif
        }
      }
      else
#endif
      {
        finalDistortion += m_pcRdCost->getDistPart( org, reco, sps.getBitDepth( toChannelType( compID ) ), compID, DF_SSE );
      }
#endif
    }
    }

    m_CABACEstimator->getCtx() = m_CurrCtx->start;
    m_CABACEstimator->resetBits();

    CUCtx cuCtx;
    cuCtx.isDQPCoded = true;
    cuCtx.isChromaQpAdjCoded = true;
    m_CABACEstimator->coding_unit( cu, partitioner, cuCtx );


    tempCS->dist     = finalDistortion;
    tempCS->fracBits = m_CABACEstimator->getEstFracBits();
    tempCS->cost     = m_pcRdCost->calcRdCost( tempCS->fracBits, tempCS->dist );

    xEncodeDontSplit( *tempCS,         partitioner );
    xCheckDQP       ( *tempCS,         partitioner );
    xCheckChromaQPOffset( *tempCS,     partitioner );
    xCheckBestMode  (  tempCS, bestCS, partitioner, cachedMode );
  }
  else
  {
    THROW( "Should never happen!" );
  }
}
#endif

#if MULTI_HYP_PRED
void EncCu::predInterSearchAdditionalHypothesisMulti(const MEResultVec& in, MEResultVec& out, PredictionUnit& pu, const MergeCtx &mrgCtx)
{
  for (const auto &x : in)
  {
    *pu.cu = x.cu;
    pu = x.pu;
#if JVET_AC0158_PIXEL_AFFINE_MC
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
#if JVET_AD0213_LIC_IMP
    pu.cu->obmcFlag = (pu.lwidth() * pu.lheight() < 32) || (pu.cu->cs->sps->getUseOBMC() == false) ? false : !pu.cu->affine;
#else
    pu.cu->obmcFlag = (pu.lwidth() * pu.lheight() < 32) || (pu.cu->licFlag == true || pu.cu->cs->sps->getUseOBMC() == false) ? false : !pu.cu->affine;
#endif
#else
#if JVET_AD0213_LIC_IMP
    pu.cu->obmcFlag = (pu.lwidth() * pu.lheight() < 32) || (pu.cu->cs->sps->getUseOBMC() == false) ? false : true;
#else
    pu.cu->obmcFlag = (pu.lwidth() * pu.lheight() < 32) || (pu.cu->licFlag == true || pu.cu->cs->sps->getUseOBMC() == false) ? false : true;
#endif
#endif
#endif

    if (pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
    {
      // the SbTmvp use xSubPuMC which will need to access the motion buffer for subblock MV
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
#if JVET_AD0213_LIC_IMP
      pu.cu->obmcFlag = (pu.lwidth() * pu.lheight() < 32) || (pu.cu->cs->sps->getUseOBMC() == false) ? false : true;
#else
      pu.cu->obmcFlag = (pu.lwidth() * pu.lheight() < 32) || (pu.cu->licFlag == true || pu.cu->cs->sps->getUseOBMC() == false) ? false : true;
#endif
#endif
      PU::spanMotionInfo(pu, mrgCtx
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
        , pu.colIdx
#endif       
      );
    }
    else if (x.cu.affine)
    {
      PU::setAllAffineMv(pu, pu.mvAffi[0][0], pu.mvAffi[0][1], pu.mvAffi[0][2], REF_PIC_LIST_0);
      PU::setAllAffineMv(pu, pu.mvAffi[1][0], pu.mvAffi[1][1], pu.mvAffi[1][2], REF_PIC_LIST_1);
    }
#if MULTI_PASS_DMVR
    else if( pu.bdmvrRefine )
    {
#if JVET_AG0276_LIC_FLAG_SIGNALING
#if TM_MRG
      if (pu.tmMergeFlag && pu.tmMergeFlagOppositeLic)
      {
        m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4TMOPPOSITELIC[pu.mergeIdx << 1], m_mvBufBDMVR4TMOPPOSITELIC[(pu.mergeIdx << 1) + 1]);
      }
      else
#endif
#endif
#if TM_MRG
      if( pu.tmMergeFlag )
      {
        m_pcInterSearch->setBdmvrSubPuMvBuf( m_mvBufBDMVR4TM[pu.mergeIdx << 1], m_mvBufBDMVR4TM[( pu.mergeIdx << 1 ) + 1] );
      }
      else
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
      if (pu.mergeOppositeLic)
      {
        m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4OPPOSITELIC[pu.mergeIdx << 1], m_mvBufBDMVR4OPPOSITELIC[(pu.mergeIdx << 1) + 1]);
      }
      else
#endif
      {
        m_pcInterSearch->setBdmvrSubPuMvBuf( m_mvBufBDMVR[pu.mergeIdx << 1], m_mvBufBDMVR[( pu.mergeIdx << 1 ) + 1] );
      }
    }
#endif
    m_pcInterSearch->predInterSearchAdditionalHypothesis(pu, x, out);
  }
}

void EncCu::xCheckRDCostInterMultiHyp2Nx2N(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
#if !JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
#if ENABLE_OBMC
  double bestOBMCCost = MAX_DOUBLE;
#endif
#endif
  if (tempCS->area.Y().area() <= MULTI_HYP_PRED_RESTRICT_BLOCK_SIZE || std::min(tempCS->area.Y().width, tempCS->area.Y().height) < MULTI_HYP_PRED_RESTRICT_MIN_WH)
  {
    return;
  }
  const SPS &sps = *tempCS->sps;
  CHECK(!sps.getUseInterMultiHyp(), "Multi Hyp is not active");
  CHECK(!tempCS->slice->isInterB(), "Multi Hyp only allowed in B slices");
  CHECK(encTestMode.opts != ETO_STANDARD, "unknown encoding option to EncCu::xCheckRDCostInterMultiHyp2Nx2N()");

  if ((m_pcEncCfg->getBaseQP() > 32) || (m_pcEncCfg->getBaseQP() > 27 && tempCS->slice->getTLayer() >= 4)) // KRNOTE: explicit QP tests
    return;

  if (m_modeCtrl->getFastDeltaQp())
  {
    if (tempCS->area.lumaSize().width > tempCS->pcv->fastDeltaQPCuMaxSize)
    {
      return; // only check necessary 2Nx2N Inter in fast deltaqp mode
    }
  }

  MEResultVec mhResults;
  const auto RDCostComp = [](const MEResult &x, const MEResult &y) { return x.cost < y.cost; };

  MergeCtx mrgCtx;
  if (sps.getSbTMVPEnabledFlag())
  {
    Size bufSize = g_miScaling.scale(tempCS->area.lumaSize());
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION   
    for (int i = 0; i < SUB_TMVP_NUM; i++)
    {
      mrgCtx.subPuMvpMiBuf[i] = MotionBuf(m_subPuMiBuf[i], bufSize);
    }
#else
    mrgCtx.subPuMvpMiBuf = MotionBuf(m_subPuMiBuf, bufSize);
#endif
  }

  // Hadamard-based pre-search
  {
    MEResultVec base = m_baseResultsForMH;

    if (base.empty())
      return;

    tempCS->initStructData(encTestMode.qp);
    CodingUnit &cu = tempCS->addCU(tempCS->area, partitioner.chType);
    PredictionUnit &pu = tempCS->addPU(cu, partitioner.chType);

    int iter = 0;
    do
    {
      MEResultVec out;
      const auto survivors = iter > 0 ? 2 : m_pcEncCfg->getNumMHPCandsToTest();
      iter++;

      std::stable_sort(base.begin(), base.end(), RDCostComp);
      if (base.size() > survivors)
        base.resize(survivors);
      predInterSearchAdditionalHypothesisMulti(base, out, pu, mrgCtx);
      mhResults.insert(mhResults.end(), out.begin(), out.end());
      base = out;
    } while (!base.empty());
  }

  std::stable_sort(mhResults.begin(), mhResults.end(), RDCostComp);

  // actual testing with "true" RD costs
  for (int i = 0; i < std::min((int)mhResults.size(), m_pcEncCfg->getAddHypTries()); ++i)
  {
    tempCS->initStructData(encTestMode.qp);
    CodingUnit &cu = tempCS->addCU(tempCS->area, partitioner.chType);
    PredictionUnit &pu = tempCS->addPU(cu, partitioner.chType);

    pu = mhResults[i].pu;
    cu = mhResults[i].cu;
#if JVET_AD0213_LIC_IMP && !JVET_AF0190_RPR_TMP_REORDER_LIC
    if (cu.licFlag && !PU::checkRprLicCondition(pu))
    {
      tempCS->initStructData(encTestMode.qp);
      continue;
    }
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER
    if (!pu.mergeFlag && PU::useRefCombList(pu))
    {
      m_pcInterSearch->setUniRefIdxLC(pu);
    }
    else if (PU::useRefPairList(pu))
    {
      m_pcInterSearch->setBiRefPairIdx(pu);
    }
#endif

#if MULTI_PASS_DMVR
    if (pu.bdmvrRefine)
    {
#if JVET_AG0276_LIC_FLAG_SIGNALING
#if TM_MRG
      if (pu.tmMergeFlag && pu.tmMergeFlagOppositeLic)
      {
        m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4TMOPPOSITELIC[pu.mergeIdx << 1], m_mvBufBDMVR4TMOPPOSITELIC[(pu.mergeIdx << 1) + 1]);
      }
      else
#endif
#endif
#if TM_MRG
      if( pu.tmMergeFlag )
      {
        m_pcInterSearch->setBdmvrSubPuMvBuf( m_mvBufBDMVR4TM[pu.mergeIdx << 1], m_mvBufBDMVR4TM[( pu.mergeIdx << 1 ) + 1] );
      }
      else
#endif
#if JVET_X0049_ADAPT_DMVR 
      if( pu.bmMergeFlag )
      {
        m_pcInterSearch->setBdmvrSubPuMvBuf( m_mvBufBDMVR4BM[pu.mergeIdx << 1], m_mvBufBDMVR4BM[( pu.mergeIdx << 1 ) + 1] );
      }
      else
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
      if (pu.mergeOppositeLic)
      {
        m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4OPPOSITELIC[pu.mergeIdx << 1], m_mvBufBDMVR4OPPOSITELIC[(pu.mergeIdx << 1) + 1]);
      }
      else
      {
#endif
        m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR[pu.mergeIdx << 1], m_mvBufBDMVR[(pu.mergeIdx << 1) + 1]);
#if JVET_AG0276_LIC_FLAG_SIGNALING
      }
#endif
    }
    else
    {
      PU::spanMotionInfo(pu, mrgCtx
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
        , pu.colIdx
#endif
      );
    }
#else
    PU::spanMotionInfo(pu, mrgCtx
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
      , pu.colIdx
#endif
    );
#endif
    cu.skip = false;
    cu.mmvdSkip = false;

    CHECK(cu.qtDepth != partitioner.currQtDepth, "Mismatch");
    CHECK(cu.btDepth != partitioner.currBtDepth, "Mismatch");
    CHECK(cu.mtDepth != partitioner.currMtDepth, "Mismatch");
    pu.mvRefine = true;
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
    if (PU::checkDoAffineBdofRefine(pu, m_pcInterSearch))
    {
      pu.availableBdofRefinedMv = AFFINE_SUBPU_BDOF_APPLY_AND_STORE_MV;
      m_pcInterSearch->setDoAffineSubPuBdof(false);
      m_pcInterSearch->setBdofSubPuMvBuf(m_mvBufEncMhpAffineBDOF);
      if (pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
      {
        int bioSubPuIdx = 0;
        const int bioSubPuStrideIncr = BDOF_SUBPU_STRIDE - (int)(pu.lumaSize().width >> BDOF_SUBPU_DIM_LOG2);
        for (int yy = 0; yy < pu.lumaSize().height; yy += 4)
        {
          for (int xx = 0; xx < pu.lumaSize().width; xx += 4)
          {
            m_mvBufEncMhpAffineBDOF[bioSubPuIdx].setZero();
            bioSubPuIdx++;
          }
          bioSubPuIdx += bioSubPuStrideIncr;
        }
      }
    }
    m_pcInterSearch->motionCompensation(pu);
    pu.availableBdofRefinedMv = AFFINE_SUBPU_BDOF_NOT_APPLY;
    if (m_pcInterSearch->getDoAffineSubPuBdof() == true)
    {
      PU::setAffineBdofRefinedMotion(pu, m_mvBufEncMhpAffineBDOF);
      m_pcInterSearch->setDoAffineSubPuBdof(false);
    }
#else
    m_pcInterSearch->motionCompensation(pu);
#endif
#if JVET_AD0213_LIC_IMP
    if (pu.cu->licFlag)
    {
      for (int list = 0; list < 2; list++)
      {
        if (pu.refIdx[list] >= 0)
        {
          for (int comp = 0; comp < MAX_NUM_COMPONENT; comp++)
          {
            m_pcInterSearch->setLicParam(list, comp, pu.cu->licScale[list][comp], pu.cu->licOffset[list][comp]);
          }
        }
      }
    }
#endif
#if MULTI_PASS_DMVR
    if (pu.bdmvrRefine)
    {
#if JVET_AG0276_LIC_FLAG_SIGNALING
#if TM_MRG
      if (pu.tmMergeFlag && pu.tmMergeFlagOppositeLic)
      {
        PU::spanMotionInfo(pu, mrgCtx,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
          pu.colIdx,
#endif
          m_mvBufBDMVR4TMOPPOSITELIC[pu.mergeIdx << 1], m_mvBufBDMVR4TMOPPOSITELIC[(pu.mergeIdx << 1) + 1], m_pcInterSearch->getBdofSubPuMvOffset());
      }
      else
#endif
#endif
#if TM_MRG
      if( pu.tmMergeFlag )
      {
        PU::spanMotionInfo(pu, mrgCtx,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
          pu.colIdx,
#endif
          m_mvBufBDMVR4TM[pu.mergeIdx << 1], m_mvBufBDMVR4TM[(pu.mergeIdx << 1) + 1], m_pcInterSearch->getBdofSubPuMvOffset());
      }
      else
#endif
#if JVET_X0049_ADAPT_DMVR
      if( pu.bmMergeFlag )
      {
        PU::spanMotionInfo(pu, mrgCtx,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
          pu.colIdx,
#endif
          m_mvBufBDMVR4BM[pu.mergeIdx << 1], m_mvBufBDMVR4BM[(pu.mergeIdx << 1) + 1], m_pcInterSearch->getBdofSubPuMvOffset());
      }
      else
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
        if (pu.mergeOppositeLic)
        {
#if JVET_AG0276_LIC_BDOF_BDMVR
          if (pu.cu->licFlag == true)
          {
            memset((void*)m_pcInterSearch->getBdofSubPuMvOffset(), 0, BDOF_SUBPU_MAX_NUM * sizeof(Mv));
          }
#endif
          PU::spanMotionInfo(pu, mrgCtx,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
            pu.colIdx,
#endif
            m_mvBufBDMVR4OPPOSITELIC[pu.mergeIdx << 1], m_mvBufBDMVR4OPPOSITELIC[(pu.mergeIdx << 1) + 1], m_pcInterSearch->getBdofSubPuMvOffset());
        }
        else
#endif
#if JVET_AG0276_LIC_BDOF_BDMVR
        {
          if (pu.cu->licFlag == true)
          {
            memset((void*)m_pcInterSearch->getBdofSubPuMvOffset(), 0, BDOF_SUBPU_MAX_NUM * sizeof(Mv));
          }
#endif
          PU::spanMotionInfo(pu, mrgCtx,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
          pu.colIdx,
#endif
          m_mvBufBDMVR[pu.mergeIdx << 1], m_mvBufBDMVR[(pu.mergeIdx << 1) + 1], m_pcInterSearch->getBdofSubPuMvOffset());
#if JVET_AG0276_LIC_BDOF_BDMVR
        }
#endif
    }
#endif
    pu.mvRefine = false;

#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
#if ENABLE_OBMC //multi hyp inter IMV
    cu.isobmcMC = true;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
    if (cu.affine == false || pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
    {
#endif
#if JVET_AC0158_PIXEL_AFFINE_MC
#if JVET_AD0213_LIC_IMP
    cu.obmcFlag = (pu.lwidth() * pu.lheight() < 32) || (pu.cu->cs->sps->getUseOBMC() == false) ? false : true;
#else
    cu.obmcFlag = (pu.lwidth() * pu.lheight() < 32) || (pu.cu->licFlag == true || pu.cu->cs->sps->getUseOBMC() == false) ? false : true;
#endif
#else
    cu.obmcFlag = true;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
    }
#endif
    m_pcInterSearch->subBlockOBMC(*cu.firstPU);
    cu.isobmcMC = false;
#endif
    xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode);
#else
#if ENABLE_OBMC //multi hyp inter IMV
    CodingStructure *prevCS = tempCS;
    PelUnitBuf tempWoOBMCBuf = m_tempWoOBMCBuffer.subBuf(UnitAreaRelative(cu, cu));
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
    if (cu.affine == false || pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
    {
#endif
    tempWoOBMCBuf.copyFrom(tempCS->getPredBuf(cu));
    cu.isobmcMC = true;
#if JVET_AC0158_PIXEL_AFFINE_MC
#if JVET_AD0213_LIC_IMP
    cu.obmcFlag = (pu.lwidth() * pu.lheight() < 32) || (pu.cu->cs->sps->getUseOBMC() == false) ? false : true;
#else
    cu.obmcFlag = (pu.lwidth() * pu.lheight() < 32) || (pu.cu->licFlag == true || pu.cu->cs->sps->getUseOBMC() == false) ? false : true;
#endif
#else
    cu.obmcFlag = true;
#endif
    m_pcInterSearch->subBlockOBMC(*cu.firstPU);
    cu.isobmcMC = false;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
    }
#endif
#endif
    xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode);
#if ENABLE_OBMC
    double tempCost = (prevCS == tempCS) ? tempCS->cost : bestCS->cost;
    if (m_pTempCUWoOBMC && tempCost < bestOBMCCost)
    {
      const unsigned wIdx = gp_sizeIdxInfo->idxFrom(tempCS->area.lwidth());
      const unsigned hIdx = gp_sizeIdxInfo->idxFrom(tempCS->area.lheight());

      m_pTempCUWoOBMC[wIdx][hIdx]->clearCUs();
      m_pTempCUWoOBMC[wIdx][hIdx]->clearPUs();
      m_pTempCUWoOBMC[wIdx][hIdx]->clearTUs();
      m_pTempCUWoOBMC[wIdx][hIdx]->copyStructure(*prevCS, partitioner.chType);

      m_pPredBufWoOBMC[wIdx][hIdx].copyFrom(tempWoOBMCBuf);
      m_pTempCUWoOBMC[wIdx][hIdx]->getPredBuf(cu).copyFrom(prevCS->getPredBuf(cu));

      bestOBMCCost = tempCost;
    }
#endif
#endif
  }
}
#endif
#if ENABLE_OBMC
void EncCu::xCheckRDCostInterWoOBMC(CodingStructure *&tempCS, CodingStructure *&bestCS, Partitioner &partitioner, const EncTestMode& encTestMode)
{
  if (!tempCS->sps->getUseOBMC())
  {
    return;
  }

  if (m_modeCtrl->getFastDeltaQp())
  {
    if (tempCS->area.lumaSize().width > tempCS->pcv->fastDeltaQPCuMaxSize)
    {
      return; // only check necessary 2Nx2N Inter in fast deltaqp mode
    }
  }

  tempCS->initStructData(encTestMode.qp);

  const SPS &sps = *tempCS->sps;
  const unsigned wIdx = gp_sizeIdxInfo->idxFrom(tempCS->area.lwidth());
  const unsigned hIdx = gp_sizeIdxInfo->idxFrom(tempCS->area.lheight());

  CodingStructure* CSWoOBMC = m_pTempCUWoOBMC[wIdx][hIdx];

  if(CSWoOBMC->cus.size() == 0)
    return;

  CodingUnit *cu = CSWoOBMC->getCU(partitioner.chType);

  if (
    !cu->obmcFlag
    || cu->predMode == MODE_INTRA
    || cu->firstPU->mergeFlag
    || CU::isIBC(*cu)
    || cu->geoFlag
#if INTER_LIC && !JVET_AD0213_LIC_IMP
    || cu->licFlag
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
    || cu->firstPU->amvpSbTmvpFlag
#endif
    )
  {
    return;
  }

  const Distortion uiSADOBMCOff = m_pcRdCost->getDistPart(tempCS->getOrgBuf(cu->Y()), m_pPredBufWoOBMC[wIdx][hIdx].Y(),
    sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, DF_SAD_FULL_NBIT);
  const Distortion uiSADOBMCOn = m_pcRdCost->getDistPart(tempCS->getOrgBuf(cu->Y()), CSWoOBMC->getPredBuf(cu->Y()),
    sps.getBitDepth(CHANNEL_TYPE_LUMA), COMPONENT_Y, DF_SAD_FULL_NBIT);

#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
  const double    dOBMCThOff = 0.65;
#else
  const double    dOBMCThOff = 1.0;
#endif
  const bool   bCheckOBMCOff = uiSADOBMCOff * dOBMCThOff < uiSADOBMCOn;

  if (!bCheckOBMCOff)
  {
    return;
  }

  tempCS->copyStructure(*CSWoOBMC, partitioner.chType);
  tempCS->getPredBuf(*cu).copyFrom(m_pPredBufWoOBMC[wIdx][hIdx]);
  cu = tempCS->getCU(partitioner.chType);
#if JVET_AC0158_PIXEL_AFFINE_MC
  CHECK(cu->affine == true, "this is not possible");
  CHECK(cu->lwidth() * cu->lheight() < 32, "this is not possible");
#endif
  cu->obmcFlag = false;
  //
  xEncodeInterResidual(tempCS, bestCS, partitioner, encTestMode, 0);
}
#endif

#if JVET_X0049_ADAPT_DMVR
void EncCu::xCheckSATDCostBMMerge(CodingStructure*& tempCS,
  CodingUnit&       cu,
  PredictionUnit&   pu,
  MergeCtx&         mrgCtx,
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  MergeCtx&         mrgCtxDir2,
  bool              armcRefinedMotion,
#endif
  PelUnitBuf*       acMergeTempBuffer[MMVD_MRG_MAX_RD_NUM],
  PelUnitBuf*&      singleMergeTempBuffer,
  unsigned&         uiNumMrgSATDCand,
  static_vector<ModeInfo, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>  &rdModeList,
  static_vector<double, MRG_MAX_NUM_CANDS + MMVD_ADD_NUM>    &candCostList,
  DistParam         distParam,
  const TempCtx&          ctxStart
#if MULTI_PASS_DMVR && !ADAPT_DIRECTIONAL_DMVR_SKIP_SUBPU_BDOF_REFINE
  , bool*             applyBDMVR
#endif
)
{
  pu.mergeFlag = true;
  cu.mmvdSkip = false;
  cu.geoFlag = false;
  cu.affine = false;
  cu.imv = IMV_OFF;
  pu.ciipFlag = false;
#if CIIP_PDPC
  pu.ciipPDPC = false;
#endif
#if JVET_AG0135_AFFINE_CIIP
  pu.ciipAffine = false;
#endif
  pu.mmvdMergeFlag = false;
  pu.regularMergeFlag = false;
  pu.bmMergeFlag = true;
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
  pu.tmMergeFlag = false;
#endif
  pu.mvRefine = false;
#if INTER_LIC
  m_pcInterSearch->m_storeBeforeLIC = false;
#endif
  pu.bdmvrRefine = true;
  mrgCtx.setMergeInfo(pu, 0);

  const double sqrtLambdaForFirstPassIntra = m_pcRdCost->getMotionLambda() * FRAC_BITS_SCALE;
  int insertPos = -1;
  uint32_t maxNumCand = mrgCtx.numCandToTestEnc;
#if (JVET_Y0134_TMVP_NAMVP_CAND_REORDERING || JVET_AA0093_REFINED_MOTION_FOR_ARMC) && JVET_W0090_ARMC_TM
  if (pu.cs->sps->getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
     && pu.cs->sps->getTMToolsEnableFlag()
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
    && pu.cs->sps->getUseTmvpNmvpReordering()
#endif
    )
  {
    maxNumCand = min(mrgCtx.numValidMergeCand, mrgCtx.numCandToTestEnc);
  }
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  maxNumCand = armcRefinedMotion ? mrgCtx.numValidMergeCand : maxNumCand;
#endif
#endif

  bool subPuRefine[2] = { false, false };
  Mv   finalMvDir[2];
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
  bool hasAtLeastOne2nd = false;
  bool subRefineList[BM_MRG_MAX_NUM_INIT_CANDS << 2][2] = {{false, false}, };
  bool subRefineListTmp[BM_MRG_MAX_NUM_INIT_CANDS << 2][2] = {{false, false}, };
  if (armcRefinedMotion)
  {
    for (uint32_t candIdx = 0; candIdx < maxNumCand; candIdx++)
    {
      pu.cu->imv = mrgCtx.useAltHpelIf[candIdx] ? IMV_HPEL : 0;
      pu.cu->bcwIdx = mrgCtx.bcwIdx[candIdx];
#if JVET_AD0213_LIC_IMP
      pu.cu->licFlag = false;
#endif
      pu.mv[REF_PIC_LIST_0] = mrgCtx.mvFieldNeighbours[(candIdx << 1)].mv;
      pu.mv[REF_PIC_LIST_1] = mrgCtx.mvFieldNeighbours[(candIdx << 1) + 1].mv;
      pu.refIdx[REF_PIC_LIST_0] = mrgCtx.mvFieldNeighbours[(candIdx << 1)].refIdx;
      pu.refIdx[REF_PIC_LIST_1] = mrgCtx.mvFieldNeighbours[(candIdx << 1) + 1].refIdx;
      bool test2nd = m_pcInterSearch->processBDMVRPU2Dir(pu, subPuRefine, finalMvDir);
      hasAtLeastOne2nd |= test2nd;
      for (pu.bmDir = 1; pu.bmDir <= (test2nd ? 2 : 1); pu.bmDir++)
      {
        uint8_t curDir = pu.bmDir - 1;
        uint8_t refDir = 1 - curDir;
        uint32_t uiMergeCand = candIdx;
        if (pu.bmDir == 2)
        {
          uiMergeCand = candIdx + BM_MRG_MAX_NUM_INIT_CANDS;
        }
        applyBDMVR[uiMergeCand] = true;
        pu.mergeIdx = uiMergeCand;
        pu.mv[curDir] = finalMvDir[curDir];
        if (pu.bmDir == 1)
        {
          pu.mv[refDir] = mrgCtx.mvFieldNeighbours[(candIdx << 1) + refDir].mv;
        }
        else
        {
          pu.mv[refDir] = mrgCtxDir2.mvFieldNeighbours[(candIdx << 1) + refDir].mv;
        }
        if (pu.bmDir == 1)
        {
          mrgCtx.mvFieldNeighbours[2 * candIdx    ].setMvField( pu.mv[0], pu.refIdx[0] );
          mrgCtx.mvFieldNeighbours[2 * candIdx + 1].setMvField( pu.mv[1], pu.refIdx[1] );
        }
        else
        {
          mrgCtxDir2.mvFieldNeighbours[2 * candIdx    ].setMvField( pu.mv[0], pu.refIdx[0] );
          mrgCtxDir2.mvFieldNeighbours[2 * candIdx + 1].setMvField( pu.mv[1], pu.refIdx[1] );
        }
        subRefineList[uiMergeCand][curDir] = subPuRefine[curDir];
        subRefineListTmp[uiMergeCand][curDir] = subRefineList[uiMergeCand][curDir];
      }
      if (!test2nd)
      {
        uint32_t uiMergeCand = candIdx + BM_MRG_MAX_NUM_INIT_CANDS;
        applyBDMVR[uiMergeCand] = false;
        pu.mv[0] = mrgCtxDir2.mvFieldNeighbours[(candIdx << 1)].mv;
        pu.mv[1] = mrgCtxDir2.mvFieldNeighbours[(candIdx << 1) + 1].mv;
        subRefineList[uiMergeCand][1] = subPuRefine[1];
        subRefineListTmp[uiMergeCand][1] = subRefineList[uiMergeCand][1];
      }
    }
    pu.bmDir = 0;
    m_pcInterSearch->adjustMergeCandidatesInOneCandidateGroup(pu, mrgCtx, NULL, NULL, NULL, mrgCtx.numValidMergeCand, subRefineList, subRefineListTmp);
#if JVET_AB0079_TM_BCW_MRG
    m_pcInterSearch->adjustMergeCandidatesBcwIdx(pu, mrgCtx);
#endif
    if (hasAtLeastOne2nd)
    {
      m_pcInterSearch->adjustMergeCandidatesInOneCandidateGroup(pu, mrgCtxDir2, applyBDMVR + BM_MRG_MAX_NUM_INIT_CANDS, NULL, NULL, mrgCtxDir2.numValidMergeCand, &subRefineList[BM_MRG_MAX_NUM_INIT_CANDS], &subRefineListTmp[BM_MRG_MAX_NUM_INIT_CANDS]);
      for (uint32_t candIdx = BM_MRG_MAX_NUM_CANDS; candIdx < 2*BM_MRG_MAX_NUM_CANDS; candIdx++)
      {
        subRefineList[candIdx][1] = subRefineList[candIdx - BM_MRG_MAX_NUM_CANDS + BM_MRG_MAX_NUM_INIT_CANDS][1];
      }
    }
    for (uint32_t candIdx = BM_MRG_MAX_NUM_CANDS; candIdx < 2*BM_MRG_MAX_NUM_CANDS; candIdx++)
    {
      applyBDMVR[candIdx] = applyBDMVR[candIdx - BM_MRG_MAX_NUM_CANDS + BM_MRG_MAX_NUM_INIT_CANDS];
    }
    if (mrgCtx.numValidMergeCand > pu.cs->sps->getMaxNumBMMergeCand())
    {
      mrgCtx.numValidMergeCand = pu.cs->sps->getMaxNumBMMergeCand();
    }
    if (mrgCtxDir2.numValidMergeCand > pu.cs->sps->getMaxNumBMMergeCand())
    {
      mrgCtxDir2.numValidMergeCand = pu.cs->sps->getMaxNumBMMergeCand();
    }
#if JVET_AB0079_TM_BCW_MRG
      m_pcInterSearch->adjustMergeCandidatesBcwIdx(pu, mrgCtxDir2);
#endif
    maxNumCand = ::min(mrgCtx.numValidMergeCand, (int)pu.cs->sps->getMaxNumBMMergeCand());
  }
#if JVET_AB0079_TM_BCW_MRG
  if(pu.cs->sps->getUseAML() && !armcRefinedMotion
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
     && pu.cs->sps->getTMToolsEnableFlag()
#endif
    )
  {
    pu.bmDir = 0;
    m_pcInterSearch->adjustMergeCandidatesBcwIdx(pu, mrgCtx);
    mrgCtxDir2 = mrgCtx;
  }
#endif
#endif
  for (uint32_t candIdx = 0; candIdx < maxNumCand; candIdx++)
  {
    pu.cu->imv = mrgCtx.useAltHpelIf[candIdx] ? IMV_HPEL : 0;
    pu.cu->bcwIdx = mrgCtx.bcwIdx[candIdx];
    pu.mv[REF_PIC_LIST_0] = mrgCtx.mvFieldNeighbours[(candIdx << 1) + 0].mv;
    pu.mv[REF_PIC_LIST_1] = mrgCtx.mvFieldNeighbours[(candIdx << 1) + 1].mv;
    pu.refIdx[REF_PIC_LIST_0] = mrgCtx.mvFieldNeighbours[(candIdx << 1) + 0].refIdx;
    pu.refIdx[REF_PIC_LIST_1] = mrgCtx.mvFieldNeighbours[(candIdx << 1) + 1].refIdx;
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
    bool test2nd = armcRefinedMotion ? applyBDMVR[candIdx + BM_MRG_MAX_NUM_CANDS] : m_pcInterSearch->processBDMVRPU2Dir(pu, subPuRefine, finalMvDir);
#else
    bool test2nd = m_pcInterSearch->processBDMVRPU2Dir(pu, subPuRefine, finalMvDir);
#endif
    for (pu.bmDir = 1; pu.bmDir <= (test2nd ? 2 : 1); pu.bmDir++)
    {
      uint8_t curDir = pu.bmDir - 1;
      uint8_t refDir = 1 - curDir;
      uint32_t uiMergeCand = candIdx;
      if (pu.bmDir == 2)
      {
        uiMergeCand = candIdx + BM_MRG_MAX_NUM_CANDS;
      }

      pu.mergeIdx = uiMergeCand;
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
      if (armcRefinedMotion)
      {
        if (pu.bmDir == 1)
        {
          pu.cu->imv = mrgCtx.useAltHpelIf[candIdx] ? IMV_HPEL : 0;
          pu.cu->bcwIdx = mrgCtx.bcwIdx[candIdx];
          pu.mv[0] = mrgCtx.mvFieldNeighbours[(candIdx << 1)].mv;
          pu.mv[1] = mrgCtx.mvFieldNeighbours[(candIdx << 1) + 1].mv;
          pu.refIdx[REF_PIC_LIST_0] = mrgCtx.mvFieldNeighbours[(candIdx << 1)].refIdx;
          pu.refIdx[REF_PIC_LIST_1] = mrgCtx.mvFieldNeighbours[(candIdx << 1) + 1].refIdx;
        }
        else
        {
          pu.cu->imv = mrgCtxDir2.useAltHpelIf[candIdx] ? IMV_HPEL : 0;
          pu.cu->bcwIdx = mrgCtxDir2.bcwIdx[candIdx];
          pu.mv[0] = mrgCtxDir2.mvFieldNeighbours[(candIdx << 1)].mv;
          pu.mv[1] = mrgCtxDir2.mvFieldNeighbours[(candIdx << 1) + 1].mv;
          pu.refIdx[REF_PIC_LIST_0] = mrgCtxDir2.mvFieldNeighbours[(candIdx << 1)].refIdx;
          pu.refIdx[REF_PIC_LIST_1] = mrgCtxDir2.mvFieldNeighbours[(candIdx << 1) + 1].refIdx;
        }
      }
      else
      {
#endif
      pu.mv[curDir] = finalMvDir[curDir];
      pu.mv[refDir] = mrgCtx.mvFieldNeighbours[(candIdx << 1) + refDir].mv;
      applyBDMVR[uiMergeCand] = true;
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
      }
#endif
      pu.bdmvrRefine = true;
      m_pcInterSearch->setBdmvrSubPuMvBuf(m_mvBufBDMVR4BM[uiMergeCand << 1], m_mvBufBDMVR4BM[(uiMergeCand << 1) + 1]);
#if JVET_AF0057
      pu.dmvrImpreciseMv = false;
#endif
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
      if (armcRefinedMotion)
      {
        m_pcInterSearch->processBDMVRSubPU(pu, subRefineList[uiMergeCand][curDir]);
      }
      else
      {
#endif
      m_pcInterSearch->processBDMVRSubPU(pu, subPuRefine[curDir]);
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
      }
#endif

      m_pcInterSearch->motionCompensation(pu, *singleMergeTempBuffer);
#if MULTI_PASS_DMVR 
      ::memcpy(m_mvBufEncBDOF4BM[uiMergeCand], m_pcInterSearch->getBdofSubPuMvOffset(), sizeof(Mv) * BDOF_SUBPU_MAX_NUM);
#endif
      distParam.cur = singleMergeTempBuffer->Y();
      Distortion uiSad = distParam.distFunc(distParam);

      m_CABACEstimator->getCtx() = ctxStart;
      uint64_t fracBits = m_pcInterSearch->xCalcPuMeBits(pu);
      double cost = (double)uiSad + (double)fracBits * sqrtLambdaForFirstPassIntra;
#if JVET_AF0057
      if (m_pcInterSearch->dmvrEnableEncoderCheck && pu.dmvrImpreciseMv)
      {
        cost = MAX_DOUBLE;
      }
#endif
      insertPos = -1;
      updateCandList(ModeInfo(cu, pu), cost, rdModeList, candCostList, uiNumMrgSATDCand, &insertPos);
      if (insertPos != -1 && insertPos < MMVD_MRG_MAX_RD_NUM)
      {
        for (int i = int(rdModeList.size()) - 1; i > insertPos; i--)
        {
          swap(acMergeTempBuffer[i - 1], acMergeTempBuffer[i]);
        }
        swap(singleMergeTempBuffer, acMergeTempBuffer[insertPos]);
      }
    }
  }
  pu.bmDir = 0;
  pu.bmMergeFlag = false;
  pu.regularMergeFlag = true;
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
  pu.tmMergeFlag = false;
#endif
  cu.affine = false;
#if AFFINE_MMVD
  pu.afMmvdFlag = false;
#endif
#if MULTI_PASS_DMVR
  pu.bdmvrRefine = false;
#endif
}
#endif
//! \}
