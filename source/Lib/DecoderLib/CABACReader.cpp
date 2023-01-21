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

/** \file     CABACReader.cpp
 *  \brief    Reader for low level syntax
 */

#include "CABACReader.h"

#include "CommonLib/CodingStructure.h"
#include "CommonLib/TrQuant.h"
#include "CommonLib/UnitTools.h"
#include "CommonLib/SampleAdaptiveOffset.h"
#include "CommonLib/dtrace_next.h"
#include "CommonLib/Picture.h"

#if RExt__DECODER_DEBUG_BIT_STATISTICS
#include "CommonLib/CodingStatistics.h"
#endif

#if RExt__DECODER_DEBUG_BIT_STATISTICS
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(x)           const CodingStatisticsClassType CSCT(x);                       m_BinDecoder.set( CSCT )
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET2(x,y)        const CodingStatisticsClassType CSCT(x,y);                     m_BinDecoder.set( CSCT )
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(x,s)    const CodingStatisticsClassType CSCT(x, s.width, s.height);    m_BinDecoder.set( CSCT )
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(x,s,z) const CodingStatisticsClassType CSCT(x, s.width, s.height, z); m_BinDecoder.set( CSCT )
#define RExt__DECODER_DEBUG_BIT_STATISTICS_SET(x)                  m_BinDecoder.set( x );
#else
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(x)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET2(x,y)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(x,s)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(x,s,z)
#define RExt__DECODER_DEBUG_BIT_STATISTICS_SET(x)
#endif


void CABACReader::initCtxModels( Slice& slice )
{
  SliceType sliceType  = slice.getSliceType();
  int       qp         = slice.getSliceQp();
  if( slice.getPPS()->getCabacInitPresentFlag() && slice.getCabacInitFlag() )
  {
    switch( sliceType )
    {
    case P_SLICE:           // change initialization table to B_SLICE initialization
      sliceType = B_SLICE;
      break;
    case B_SLICE:           // change initialization table to P_SLICE initialization
      sliceType = P_SLICE;
      break;
    default     :           // should not occur
      THROW( "Invalid slice type" );
      break;
    }
  }
  m_BinDecoder.reset( qp, (int)sliceType );

#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
  if( slice.getSPS()->getTempCabacInitMode() )
  {
    m_CABACDataStore->loadCtxStates( &slice, getCtx() );
  }
#endif
}


//================================================================================
//  clause 7.3.8.1
//--------------------------------------------------------------------------------
//    bool  terminating_bit()
//    void  remaining_bytes( noTrailingBytesExpected )
//================================================================================

bool CABACReader::terminating_bit()
{
  if( m_BinDecoder.decodeBinTrm() )
  {
    m_BinDecoder.finish();
#if RExt__DECODER_DEBUG_BIT_STATISTICS
    CodingStatistics::IncrementStatisticEP( STATS__TRAILING_BITS, m_Bitstream->readOutTrailingBits(), 0 );
#else
    m_Bitstream->readOutTrailingBits();
#endif
    return true;
  }
  return false;
}

void CABACReader::remaining_bytes( bool noTrailingBytesExpected )
{
  if( noTrailingBytesExpected )
  {
    CHECK( 0 != m_Bitstream->getNumBitsLeft(), "Bits left when not supposed" );
  }
  else
  {
    while( m_Bitstream->getNumBitsLeft() )
    {
      unsigned trailingNullByte = m_Bitstream->readByte();
      if( trailingNullByte != 0 )
      {
        THROW( "Trailing byte should be '0', but has a value of " << std::hex << trailingNullByte << std::dec << "\n" );
      }
    }
  }
}

//================================================================================
//  clause 7.3.8.2
//--------------------------------------------------------------------------------
//    void  coding_tree_unit( cs, area, qpL, qpC, ctuRsAddr )
//================================================================================

void CABACReader::coding_tree_unit( CodingStructure& cs, const UnitArea& area, int (&qps)[2], unsigned ctuRsAddr )
{
  CUCtx cuCtx( qps[CH_L] );
  QTBTPartitioner partitioner;

  partitioner.initCtu(area, CH_L, *cs.slice);
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  cs.treeType = partitioner.treeType = TREE_D;
  cs.modeType = partitioner.modeType = MODE_TYPE_ALL;
#endif

  sao( cs, ctuRsAddr );
#if JVET_W0066_CCSAO
  if (cs.sps->getCCSAOEnabledFlag())
  {
    for ( int compIdx = 0; compIdx < getNumberValidComponents( cs.pcv->chrFormat ); compIdx++ )
    {
      if (cs.slice->m_ccSaoComParam.enabled[compIdx])
      {
        const int setNum = cs.slice->m_ccSaoComParam.setNum[compIdx];

        const int      ry = ctuRsAddr / cs.pcv->widthInCtus;
        const int      rx = ctuRsAddr % cs.pcv->widthInCtus;
        const Position lumaPos(rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight);

        ccSaoControlIdc(cs, ComponentID(compIdx), ctuRsAddr, cs.slice->m_ccSaoControl[compIdx], lumaPos, setNum);
      }
    }
  }
#endif
  if (cs.sps->getALFEnabledFlag() && (cs.slice->getTileGroupAlfEnabledFlag(COMPONENT_Y)))
  {
    const PreCalcValues& pcv = *cs.pcv;
    int                 frame_width_in_ctus = pcv.widthInCtus;
    int                 ry = ctuRsAddr / frame_width_in_ctus;
    int                 rx = ctuRsAddr - ry * frame_width_in_ctus;
    const Position      pos( rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight );
    const uint32_t          curSliceIdx = cs.slice->getIndependentSliceIdx();
    const uint32_t          curTileIdx = cs.pps->getTileIdx( pos );
    bool                leftAvail = cs.getCURestricted( pos.offset( -(int)pcv.maxCUWidth, 0 ), pos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
    bool                aboveAvail = cs.getCURestricted( pos.offset( 0, -(int)pcv.maxCUHeight ), pos, curSliceIdx, curTileIdx, CH_L ) ? true : false;

    int leftCTUAddr = leftAvail ? ctuRsAddr - 1 : -1;
    int aboveCTUAddr = aboveAvail ? ctuRsAddr - frame_width_in_ctus : -1;

    for( int compIdx = 0; compIdx < MAX_NUM_COMPONENT; compIdx++ )
    {
      if (cs.slice->getTileGroupAlfEnabledFlag((ComponentID)compIdx))
      {
        uint8_t* ctbAlfFlag = cs.slice->getPic()->getAlfCtuEnableFlag( compIdx );
        int ctx = 0;
        ctx += leftCTUAddr > -1 ? ( ctbAlfFlag[leftCTUAddr] ? 1 : 0 ) : 0;
        ctx += aboveCTUAddr > -1 ? ( ctbAlfFlag[aboveCTUAddr] ? 1 : 0 ) : 0;

        RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET(STATS__CABAC_BITS__ALF);
        ctbAlfFlag[ctuRsAddr] = m_BinDecoder.decodeBin( Ctx::ctbAlfFlag( compIdx * 3 + ctx ) );

        if (isLuma((ComponentID)compIdx) && ctbAlfFlag[ctuRsAddr])
        {
          readAlfCtuFilterIndex(cs, ctuRsAddr);
#if ALF_IMPROVEMENT
          int apsIdxCtu = cs.slice->getPic()->getAlfCtbFilterIndex()[ctuRsAddr] - NUM_FIXED_FILTER_SETS;
          if (apsIdxCtu >= 0)
          {
            int apsIdx = cs.slice->getTileGroupApsIdLuma()[apsIdxCtu];
            CHECK(cs.slice->getAlfAPSs()[apsIdx] == nullptr, "APS not initialized");
            const AlfParam& alfParam = cs.slice->getAlfAPSs()[apsIdx]->getAlfAPSParam();
            const int numAlts = alfParam.numAlternativesLuma;
            uint8_t* ctbAlfAlternative = cs.slice->getPic()->getAlfCtuAlternativeData(compIdx);
            ctbAlfAlternative[ctuRsAddr] = 0;
            uint8_t decoded = 0;
            while( decoded < numAlts - 1 && m_BinDecoder.decodeBin( Ctx::ctbAlfAlternative( COMPONENT_Y ) ) )
            {
              ++decoded;
            }
            ctbAlfAlternative[ctuRsAddr] = decoded;
          }
#endif
        }
        if( isChroma( (ComponentID)compIdx ) )
        {
          int apsIdx = cs.slice->getTileGroupApsIdChroma();
          CHECK(cs.slice->getAlfAPSs()[apsIdx] == nullptr, "APS not initialized");
          const AlfParam& alfParam = cs.slice->getAlfAPSs()[apsIdx]->getAlfAPSParam();
          const int numAlts = alfParam.numAlternativesChroma;
          uint8_t* ctbAlfAlternative = cs.slice->getPic()->getAlfCtuAlternativeData( compIdx );
          ctbAlfAlternative[ctuRsAddr] = 0;
          if( ctbAlfFlag[ctuRsAddr] )
          {
            uint8_t decoded = 0;
#if ALF_IMPROVEMENT
            while( decoded < numAlts-1 && m_BinDecoder.decodeBin( Ctx::ctbAlfAlternative( compIdx ) ) )
#else
            while( decoded < numAlts - 1 && m_BinDecoder.decodeBin( Ctx::ctbAlfAlternative( compIdx - 1 ) ) )
#endif
            {
              ++decoded;
            }

            ctbAlfAlternative[ctuRsAddr] = decoded;
          }
        }
      }
    }
  }
  if (cs.sps->getCCALFEnabledFlag())
  {
    for ( int compIdx = 1; compIdx < getNumberValidComponents( cs.pcv->chrFormat ); compIdx++ )
    {
      if (cs.slice->m_ccAlfFilterParam.ccAlfFilterEnabled[compIdx - 1])
      {
        const int filterCount   = cs.slice->m_ccAlfFilterParam.ccAlfFilterCount[compIdx - 1];

        const int      ry = ctuRsAddr / cs.pcv->widthInCtus;
        const int      rx = ctuRsAddr % cs.pcv->widthInCtus;
        const Position lumaPos(rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight);

        ccAlfFilterControlIdc(cs, ComponentID(compIdx), ctuRsAddr, cs.slice->m_ccAlfFilterControl[compIdx - 1], lumaPos,
                              filterCount);
      }
    }
  }

#if TU_256
  if( CS::isDualITree( cs ) && cs.pcv->chrFormat != CHROMA_400 && cs.pcv->maxCUWidth > std::min<int>( MAX_TB_SIZEY, MAX_INTRA_SIZE ) )
#else
  if ( CS::isDualITree(cs) && cs.pcv->chrFormat != CHROMA_400 && cs.pcv->maxCUWidth > 64 )
#endif
  {
    QTBTPartitioner chromaPartitioner;
    chromaPartitioner.initCtu(area, CH_C, *cs.slice);
    CUCtx cuCtxChroma(qps[CH_C]);
    coding_tree(cs, partitioner, cuCtx, &chromaPartitioner, &cuCtxChroma);
    qps[CH_L] = cuCtx.qp;
    qps[CH_C] = cuCtxChroma.qp;
  }
  else
  {
    coding_tree(cs, partitioner, cuCtx);
    qps[CH_L] = cuCtx.qp;
    if( CS::isDualITree( cs ) && cs.pcv->chrFormat != CHROMA_400 )
    {
      CUCtx cuCtxChroma( qps[CH_C] );
      partitioner.initCtu(area, CH_C, *cs.slice);
      coding_tree(cs, partitioner, cuCtxChroma);
      qps[CH_C] = cuCtxChroma.qp;
    }
  }

  DTRACE_COND( ctuRsAddr == 0, g_trace_ctx, D_QP_PER_CTU, "\n%4d %2d", cs.picture->poc, cs.slice->getSliceQpBase() );
  DTRACE     (                 g_trace_ctx, D_QP_PER_CTU, " %3d",           qps[CH_L] - cs.slice->getSliceQpBase() );

}

void CABACReader::readAlfCtuFilterIndex(CodingStructure& cs, unsigned ctuRsAddr)
{
  short* alfCtbFilterSetIndex = cs.slice->getPic()->getAlfCtbFilterIndex();
  unsigned numAps = cs.slice->getTileGroupNumAps();
  unsigned numAvailableFiltSets = numAps + NUM_FIXED_FILTER_SETS;
  uint32_t filtIndex = 0;
  if (numAvailableFiltSets > NUM_FIXED_FILTER_SETS)
  {
    unsigned usePrevFilt = m_BinDecoder.decodeBin(Ctx::AlfUseTemporalFilt());
    if (usePrevFilt)
    {
      if (numAps > 1)
      {
        xReadTruncBinCode(filtIndex, numAvailableFiltSets - NUM_FIXED_FILTER_SETS);
      }
      filtIndex += (unsigned)(NUM_FIXED_FILTER_SETS);
    }
    else
    {
      xReadTruncBinCode(filtIndex, NUM_FIXED_FILTER_SETS);
    }
  }
  else
  {
    xReadTruncBinCode(filtIndex, NUM_FIXED_FILTER_SETS);
  }
  alfCtbFilterSetIndex[ctuRsAddr] = filtIndex;
}
void CABACReader::ccAlfFilterControlIdc(CodingStructure &cs, const ComponentID compID, const int curIdx,
                                        uint8_t *filterControlIdc, Position lumaPos, int filterCount)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__CROSS_COMPONENT_ALF_BLOCK_LEVEL_IDC );

  Position       leftLumaPos    = lumaPos.offset(-(int)cs.pcv->maxCUWidth, 0);
  Position       aboveLumaPos   = lumaPos.offset(0, -(int)cs.pcv->maxCUWidth);
  const uint32_t curSliceIdx    = cs.slice->getIndependentSliceIdx();
  const uint32_t curTileIdx     = cs.pps->getTileIdx( lumaPos );
  bool           leftAvail      = cs.getCURestricted( leftLumaPos,  lumaPos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
  bool           aboveAvail     = cs.getCURestricted( aboveLumaPos, lumaPos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
  int            ctxt           = 0;

  if (leftAvail)
  {
#if JVET_X0071_LONGER_CCALF
    ctxt += (filterControlIdc[curIdx - 1] != 1) ? 1 : 0;
#else
    ctxt += ( filterControlIdc[curIdx - 1] ) ? 1 : 0;
#endif
  }
  if (aboveAvail)
  {
#if JVET_X0071_LONGER_CCALF
    ctxt += (filterControlIdc[curIdx - cs.pcv->widthInCtus] != 1) ? 1 : 0;
#else
    ctxt += ( filterControlIdc[curIdx - cs.pcv->widthInCtus] ) ? 1 : 0;
#endif
  }
  ctxt += ( compID == COMPONENT_Cr ) ? 3 : 0;

  int idcVal  = m_BinDecoder.decodeBin( Ctx::CcAlfFilterControlFlag( ctxt ) );
  if ( idcVal )
  {
    while ( ( idcVal != filterCount ) && m_BinDecoder.decodeBinEP() )
    {
      idcVal++;
    }
  }

#if JVET_X0071_LONGER_CCALF
  int pos0 = 1;
  idcVal = (idcVal == pos0 ? 0 : idcVal < pos0 ? idcVal + 1 : idcVal);
#endif

  filterControlIdc[curIdx] = idcVal;

  DTRACE(g_trace_ctx, D_SYNTAX, "ccAlfFilterControlIdc() compID=%d pos=(%d,%d) ctxt=%d, filterCount=%d, idcVal=%d\n",
         compID, lumaPos.x, lumaPos.y, ctxt, filterCount, idcVal);
}

//================================================================================
//  clause 7.3.8.3
//--------------------------------------------------------------------------------
//    void  sao( slice, ctuRsAddr )
//================================================================================

void CABACReader::sao( CodingStructure& cs, unsigned ctuRsAddr )
{
  const SPS&   sps   = *cs.sps;

#if JVET_V0094_BILATERAL_FILTER
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  if(!(cs.pps->getUseBIF() || cs.sps->getSAOEnabledFlag() || cs.pps->getUseChromaBIF()))
#else
  // If neither BIF nor SAO is enabled, we can return.
  if(!(cs.pps->getUseBIF() || cs.sps->getSAOEnabledFlag()))
#endif
  {
    return;
  }
  // At least one is enabled, it is safe to assume we can do getSAO().
  SAOBlkParam&      sao_ctu_pars            = cs.picture->getSAO()[ctuRsAddr];
#else
#if JVET_X0071_CHROMA_BILATERAL_FILTER
  if(!(cs.pps->getUseChromaBIF() || cs.sps->getSAOEnabledFlag()))
  {
        return;
  }
    SAOBlkParam&      sao_ctu_pars            = cs.picture->getSAO()[ctuRsAddr];
#endif
#endif
  
  if( !sps.getSAOEnabledFlag() )
  {
    return;
  }

  const Slice& slice                        = *cs.slice;
#if JVET_V0094_BILATERAL_FILTER || JVET_X0071_CHROMA_BILATERAL_FILTER
  // The getSAO() function call has been moved up.
#else
  SAOBlkParam&      sao_ctu_pars            = cs.picture->getSAO()[ctuRsAddr];
#endif
  bool              slice_sao_luma_flag     = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_LUMA ) );
  bool              slice_sao_chroma_flag   = ( slice.getSaoEnabledFlag( CHANNEL_TYPE_CHROMA ) && sps.getChromaFormatIdc() != CHROMA_400 );
  sao_ctu_pars[ COMPONENT_Y  ].modeIdc      = SAO_MODE_OFF;
  sao_ctu_pars[ COMPONENT_Cb ].modeIdc      = SAO_MODE_OFF;
  sao_ctu_pars[ COMPONENT_Cr ].modeIdc      = SAO_MODE_OFF;
  if( !slice_sao_luma_flag && !slice_sao_chroma_flag )
  {
    return;
  }

  // merge
  int             frame_width_in_ctus     = cs.pcv->widthInCtus;
  int             ry                      = ctuRsAddr      / frame_width_in_ctus;
  int             rx                      = ctuRsAddr - ry * frame_width_in_ctus;
  int             sao_merge_type          = -1;
  const Position  pos( rx * cs.pcv->maxCUWidth, ry * cs.pcv->maxCUHeight );
  const unsigned  curSliceIdx = cs.slice->getIndependentSliceIdx();

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__SAO );

  const unsigned  curTileIdx  = cs.pps->getTileIdx( pos );
  if( cs.getCURestricted( pos.offset(-(int)cs.pcv->maxCUWidth, 0), pos, curSliceIdx, curTileIdx, CH_L ) )
  {
    // sao_merge_left_flag
    sao_merge_type  += int( m_BinDecoder.decodeBin( Ctx::SaoMergeFlag() ) );
  }

  if( sao_merge_type < 0 && cs.getCURestricted( pos.offset(0, -(int)cs.pcv->maxCUHeight), pos, curSliceIdx, curTileIdx, CH_L ) )
  {
    // sao_merge_above_flag
    sao_merge_type  += int( m_BinDecoder.decodeBin( Ctx::SaoMergeFlag() ) ) << 1;
  }
  if( sao_merge_type >= 0 )
  {
    if( slice_sao_luma_flag || slice_sao_chroma_flag )
    {
      sao_ctu_pars[ COMPONENT_Y  ].modeIdc  = SAO_MODE_MERGE;
      sao_ctu_pars[ COMPONENT_Y  ].typeIdc  = sao_merge_type;
    }
    if( slice_sao_chroma_flag )
    {
      sao_ctu_pars[ COMPONENT_Cb ].modeIdc  = SAO_MODE_MERGE;
      sao_ctu_pars[ COMPONENT_Cr ].modeIdc  = SAO_MODE_MERGE;
      sao_ctu_pars[ COMPONENT_Cb ].typeIdc  = sao_merge_type;
      sao_ctu_pars[ COMPONENT_Cr ].typeIdc  = sao_merge_type;
    }
    return;
  }

  // explicit parameters
  ComponentID firstComp = ( slice_sao_luma_flag   ? COMPONENT_Y  : COMPONENT_Cb );
  ComponentID lastComp  = ( slice_sao_chroma_flag ? COMPONENT_Cr : COMPONENT_Y  );
  for( ComponentID compID = firstComp; compID <= lastComp; compID = ComponentID( compID + 1 ) )
  {
    SAOOffset& sao_pars = sao_ctu_pars[ compID ];

    // sao_type_idx_luma / sao_type_idx_chroma
    if( compID != COMPONENT_Cr )
    {
      if( m_BinDecoder.decodeBin( Ctx::SaoTypeIdx() ) )
      {
        if( m_BinDecoder.decodeBinEP( ) )
        {
          // edge offset
          sao_pars.modeIdc = SAO_MODE_NEW;
          sao_pars.typeIdc = SAO_TYPE_START_EO;
        }
        else
        {
          // band offset
          sao_pars.modeIdc = SAO_MODE_NEW;
          sao_pars.typeIdc = SAO_TYPE_START_BO;
        }
      }
    }
    else //Cr, follow Cb SAO type
    {
      sao_pars.modeIdc = sao_ctu_pars[ COMPONENT_Cb ].modeIdc;
      sao_pars.typeIdc = sao_ctu_pars[ COMPONENT_Cb ].typeIdc;
    }
    if( sao_pars.modeIdc == SAO_MODE_OFF )
    {
      continue;
    }

    // sao_offset_abs
    int       offset[4];
    const int maxOffsetQVal = SampleAdaptiveOffset::getMaxOffsetQVal( sps.getBitDepth( toChannelType(compID) ) );
    offset    [0]           = (int)unary_max_eqprob( maxOffsetQVal );
    offset    [1]           = (int)unary_max_eqprob( maxOffsetQVal );
    offset    [2]           = (int)unary_max_eqprob( maxOffsetQVal );
    offset    [3]           = (int)unary_max_eqprob( maxOffsetQVal );

    // band offset mode
    if( sao_pars.typeIdc == SAO_TYPE_START_BO )
    {
      // sao_offset_sign
      for( int k = 0; k < 4; k++ )
      {
        if( offset[k] && m_BinDecoder.decodeBinEP( ) )
        {
          offset[k] = -offset[k];
        }
      }
      // sao_band_position
      sao_pars.typeAuxInfo = m_BinDecoder.decodeBinsEP( NUM_SAO_BO_CLASSES_LOG2 );
      for( int k = 0; k < 4; k++ )
      {
        sao_pars.offset[ ( sao_pars.typeAuxInfo + k ) % MAX_NUM_SAO_CLASSES ] = offset[k];
      }
      continue;
    }

    // edge offset mode
    sao_pars.typeAuxInfo = 0;
    if( compID != COMPONENT_Cr )
    {
      // sao_eo_class_luma / sao_eo_class_chroma
      sao_pars.typeIdc += m_BinDecoder.decodeBinsEP( NUM_SAO_EO_TYPES_LOG2 );
    }
    else
    {
      sao_pars.typeIdc  = sao_ctu_pars[ COMPONENT_Cb ].typeIdc;
    }
    sao_pars.offset[ SAO_CLASS_EO_FULL_VALLEY ] =  offset[0];
    sao_pars.offset[ SAO_CLASS_EO_HALF_VALLEY ] =  offset[1];
    sao_pars.offset[ SAO_CLASS_EO_PLAIN       ] =  0;
    sao_pars.offset[ SAO_CLASS_EO_HALF_PEAK   ] = -offset[2];
    sao_pars.offset[ SAO_CLASS_EO_FULL_PEAK   ] = -offset[3];
  }
}

#if JVET_V0094_BILATERAL_FILTER
void CABACReader::bif(CodingStructure& cs)
{
  int width = cs.picture->lwidth();
  int height = cs.picture->lheight();

  int block_width = cs.pcv->maxCUWidth;
  int block_height = cs.pcv->maxCUHeight;

  int width_in_blocks = width / block_width + (width % block_width != 0);
  int height_in_blocks = height / block_height + (height % block_height != 0);

  for (int i = 0; i < width_in_blocks * height_in_blocks; i++)
  {
    bif(cs, i);
  }
}

void CABACReader::bif(CodingStructure& cs, unsigned ctuRsAddr)
{
//  const SPS&   sps = *cs.sps;
  const PPS&   pps = *cs.pps;

  if (!pps.getUseBIF())
  {
    return;
  }

  BifParams& bifParams = cs.picture->getBifParam();

  if (ctuRsAddr == 0)
  {
    int width = cs.picture->lwidth();
    int height = cs.picture->lheight();
    int block_width = cs.pcv->maxCUWidth;
    int block_height = cs.pcv->maxCUHeight;

    int width_in_blocks = width / block_width + (width % block_width != 0);
    int height_in_blocks = height / block_height + (height % block_height != 0);
    bifParams.numBlocks = width_in_blocks * height_in_blocks;
    bifParams.ctuOn.resize(bifParams.numBlocks);
    std::fill(bifParams.ctuOn.begin(), bifParams.ctuOn.end(), 0);
  }
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET2(STATS__CABAC_BITS__BIF, COMPONENT_Y);
  if (ctuRsAddr == 0)
  {
    bifParams.allCtuOn = m_BinDecoder.decodeBinEP();
    if (bifParams.allCtuOn == 0)
      bifParams.frmOn = m_BinDecoder.decodeBinEP();
  }
  int i = ctuRsAddr;
  if (bifParams.allCtuOn)
  {
    bifParams.ctuOn[i] = 1;
  }
  else
  {
    if (bifParams.frmOn)
    {
      bifParams.ctuOn[i] = m_BinDecoder.decodeBin(Ctx::BifCtrlFlags());
    }
    else
    {
      bifParams.ctuOn[i] = 0;
    }
  }
}
#endif
#if JVET_X0071_CHROMA_BILATERAL_FILTER
void CABACReader::chromaBifCb(CodingStructure& cs)
{
  int width = cs.picture->lwidth();
  int height = cs.picture->lheight();

  int blockWidth = cs.pcv->maxCUWidth;
  int blockHeight = cs.pcv->maxCUHeight;

  int widthInBlocks = width / blockWidth + (width % blockWidth != 0);
  int heightInBlocks = height / blockHeight + (height % blockHeight != 0);
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET2(STATS__CABAC_BITS__BIF, COMPONENT_Cb);

  for (int i = 0; i < widthInBlocks * heightInBlocks; i++)
  {
    chromaBifCb(cs, i);
  }
}

void CABACReader::chromaBifCb(CodingStructure& cs, unsigned ctuRsAddr)
{
  const PPS&   pps = *cs.pps;

  if (!pps.getUseChromaBIF())
  {
    return;
  }
  ChromaBifParams& chromaBifParams = cs.picture->getChromaBifParam();
  if (ctuRsAddr == 0)
  {
    int width = cs.picture->lwidth();
    int height = cs.picture->lheight();
    int blockWidth = cs.pcv->maxCUWidth;
    int blockHeight = cs.pcv->maxCUHeight;

    int widthInBlocks = width / blockWidth + (width % blockWidth != 0);
    int heightInBlocks = height / blockHeight + (height % blockHeight != 0);
    int numBlocks = widthInBlocks * heightInBlocks;
    chromaBifParams.numBlocks = numBlocks;
    chromaBifParams.ctuOnCb.resize(numBlocks);
    std::fill(chromaBifParams.ctuOnCb.begin(), chromaBifParams.ctuOnCb.end(), 0);
  }
  if (ctuRsAddr == 0)
  {
    chromaBifParams.allCtuOnCb = m_BinDecoder.decodeBinEP();
    if (chromaBifParams.allCtuOnCb == 0)
    {
      chromaBifParams.frmOnCb = m_BinDecoder.decodeBinEP();
    }
  }
  int i = ctuRsAddr;
  if (chromaBifParams.allCtuOnCb)
  {
    chromaBifParams.ctuOnCb[i] = 1;
  }
  else
  {
    if (chromaBifParams.frmOnCb)
    {
      chromaBifParams.ctuOnCb[i] = m_BinDecoder.decodeBin(Ctx::ChromaBifCtrlFlagsCb());
    }
    else
    {
      chromaBifParams.ctuOnCb[i] = 0;
    }
  }
}

void CABACReader::chromaBifCr(CodingStructure& cs)
{
  int width = cs.picture->lwidth();
  int height = cs.picture->lheight();

  int blockWidth = cs.pcv->maxCUWidth;
  int blockHeight = cs.pcv->maxCUHeight;

  int widthInBlocks = width / blockWidth + (width % blockWidth != 0);
  int heightInBlocks = height / blockHeight + (height % blockHeight != 0);
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET2(STATS__CABAC_BITS__BIF, COMPONENT_Cr);

  for (int i = 0; i < widthInBlocks * heightInBlocks; i++)
  {
    chromaBifCr(cs, i);
  }
}

void CABACReader::chromaBifCr(CodingStructure& cs, unsigned ctuRsAddr)
{
  const PPS&   pps = *cs.pps;

  if (!pps.getUseChromaBIF())
  {
    return;
  }

  ChromaBifParams& chromaBifParams = cs.picture->getChromaBifParam();

  if (ctuRsAddr == 0)
  {
    int width = cs.picture->lwidth();
    int height = cs.picture->lheight();
    int blockWidth = cs.pcv->maxCUWidth;
    int blockHeight = cs.pcv->maxCUHeight;

    int widthInBlocks = width / blockWidth + (width % blockWidth != 0);
    int heightInBlocks = height / blockHeight + (height % blockHeight != 0);
    int numBlocks = widthInBlocks * heightInBlocks;
    chromaBifParams.numBlocks = numBlocks;
    chromaBifParams.ctuOnCr.resize(numBlocks);
    std::fill(chromaBifParams.ctuOnCr.begin(), chromaBifParams.ctuOnCr.end(), 0);
  }
  if (ctuRsAddr == 0)
  {
    chromaBifParams.allCtuOnCr = m_BinDecoder.decodeBinEP();
    if (chromaBifParams.allCtuOnCr == 0)
    {
      chromaBifParams.frmOnCr = m_BinDecoder.decodeBinEP();
    }
  }
  int i = ctuRsAddr;
  if (chromaBifParams.allCtuOnCr)
  {
    chromaBifParams.ctuOnCr[i] = 1;
  }
  else
  {
    if (chromaBifParams.frmOnCr)
    {
      chromaBifParams.ctuOnCr[i] = m_BinDecoder.decodeBin(Ctx::ChromaBifCtrlFlagsCr());
    }
    else
    {
      chromaBifParams.ctuOnCr[i] = 0;
    }
  }
}
#endif

#if JVET_W0066_CCSAO
void CABACReader::ccSaoControlIdc(CodingStructure &cs, const ComponentID compID, const int curIdx,
                                  uint8_t *controlIdc, Position lumaPos, int setNum)
{
  //RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET( STATS__CABAC_BITS__CROSS_COMPONENT_SAO_BLOCK_LEVEL_IDC );

  Position       leftLumaPos    = lumaPos.offset(-(int)cs.pcv->maxCUWidth, 0);
  Position       aboveLumaPos   = lumaPos.offset(0, -(int)cs.pcv->maxCUWidth);
  const uint32_t curSliceIdx    = cs.slice->getIndependentSliceIdx();
  const uint32_t curTileIdx     = cs.pps->getTileIdx( lumaPos );
  bool           leftAvail      = cs.getCURestricted( leftLumaPos,  lumaPos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
  bool           aboveAvail     = cs.getCURestricted( aboveLumaPos, lumaPos, curSliceIdx, curTileIdx, CH_L ) ? true : false;
  int            ctxt           = 0;

  if (leftAvail)
  {
    ctxt += ( controlIdc[curIdx - 1] ) ? 1 : 0;
  }
  if (aboveAvail)
  {
    ctxt += ( controlIdc[curIdx - cs.pcv->widthInCtus] ) ? 1 : 0;
  }
  ctxt += ( compID == COMPONENT_Y  ) ? 0 
        : ( compID == COMPONENT_Cb ) ? 3 : 6;

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET2(STATS__CABAC_BITS__SAO, compID);
  int idcVal  = m_BinDecoder.decodeBin( Ctx::CcSaoControlIdc( ctxt ) );
  if ( idcVal )
  {
    while ( ( idcVal != setNum ) && m_BinDecoder.decodeBinEP() )
    {
      idcVal++;
    }
  }
  controlIdc[curIdx] = idcVal;

  DTRACE(g_trace_ctx, D_SYNTAX, "ccSaoControlIdc() compID=%d pos=(%d,%d) ctxt=%d, setNum=%d, idcVal=%d\n",
         compID, lumaPos.x, lumaPos.y, ctxt, setNum, idcVal);
}
#endif

//================================================================================
//  clause 7.3.8.4
//--------------------------------------------------------------------------------
//    void  coding_tree       ( cs, partitioner, cuCtx )
//    bool  split_cu_flag     ( cs, partitioner )
//    split split_cu_mode_mt  ( cs, partitioner )
//================================================================================

void CABACReader::coding_tree( CodingStructure& cs, Partitioner& partitioner, CUCtx& cuCtx, Partitioner* pPartitionerChroma, CUCtx* pCuCtxChroma)
{
  const PPS      &pps         = *cs.pps;
  const UnitArea &currArea    = partitioner.currArea();

  // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (pps.getUseDQP() && partitioner.currQgEnable())
#else
  //Note: do not reset qg at chroma CU
  if( pps.getUseDQP() && partitioner.currQgEnable() && !isChroma(partitioner.chType) )
#endif
  {
    cuCtx.qgStart    = true;
    cuCtx.isDQPCoded = false;
  }
  if( cs.slice->getUseChromaQpAdj() && partitioner.currQgChromaEnable() )
  {
    cuCtx.isChromaQpAdjCoded  = false;
    cs.chromaQpAdj = 0;
  }

  // Reset delta QP coding flag and ChromaQPAdjustemt coding flag
  if (CS::isDualITree(cs) && pPartitionerChroma != nullptr)
  {
    if (pps.getUseDQP() && pPartitionerChroma->currQgEnable())
    {
      pCuCtxChroma->qgStart    = true;
      pCuCtxChroma->isDQPCoded = false;
    }
    if (cs.slice->getUseChromaQpAdj() && pPartitionerChroma->currQgChromaEnable())
    {
      pCuCtxChroma->isChromaQpAdjCoded = false;
      cs.chromaQpAdj = 0;
    }
  }

  const PartSplit splitMode = split_cu_mode( cs, partitioner );

  CHECK( !partitioner.canSplit( splitMode, cs ), "Got an invalid split!" );

  if( splitMode != CU_DONT_SPLIT )
  {
#if TU_256
    const int maxSize = std::min<int>( MAX_TB_SIZEY, MAX_INTRA_SIZE );
    
    if (CS::isDualITree(cs) && pPartitionerChroma != nullptr
        && (partitioner.currArea().lwidth() >= maxSize || partitioner.currArea().lheight() >= maxSize))
#else
    if (CS::isDualITree(cs) && pPartitionerChroma != nullptr
        && (partitioner.currArea().lwidth() >= 64 || partitioner.currArea().lheight() >= 64))
#endif
    {
      partitioner.splitCurrArea(CU_QUAD_SPLIT, cs);
      pPartitionerChroma->splitCurrArea(CU_QUAD_SPLIT, cs);
      bool beContinue     = true;
      bool lumaContinue   = true;
      bool chromaContinue = true;

      while (beContinue)
      {
#if TU_256
        if( partitioner.currArea().lwidth() > maxSize || partitioner.currArea().lheight() > maxSize )
#else
        if (partitioner.currArea().lwidth() > 64 || partitioner.currArea().lheight() > 64)
#endif
        {
          if (cs.area.blocks[partitioner.chType].contains(partitioner.currArea().blocks[partitioner.chType].pos()))
          {
            coding_tree(cs, partitioner, cuCtx, pPartitionerChroma, pCuCtxChroma);
          }
          lumaContinue   = partitioner.nextPart(cs);
          chromaContinue = pPartitionerChroma->nextPart(cs);
          CHECK(lumaContinue != chromaContinue, "luma chroma partition should be matched");
          beContinue = lumaContinue;
        }
        else
        {
          // dual tree coding under 64x64 block
          if (cs.area.blocks[partitioner.chType].contains(partitioner.currArea().blocks[partitioner.chType].pos()))
          {
            coding_tree(cs, partitioner, cuCtx);
          }
          lumaContinue = partitioner.nextPart(cs);
          if (cs.area.blocks[pPartitionerChroma->chType].contains(
                pPartitionerChroma->currArea().blocks[pPartitionerChroma->chType].pos()))
          {
            coding_tree(cs, *pPartitionerChroma, *pCuCtxChroma);
          }
          chromaContinue = pPartitionerChroma->nextPart(cs);
          CHECK(lumaContinue != chromaContinue, "luma chroma partition should be matched");
          beContinue = lumaContinue;
        }
      }
      partitioner.exitCurrSplit();
      pPartitionerChroma->exitCurrSplit();

      // cat the chroma CUs together
      CodingUnit *currentCu        = cs.getCU(partitioner.currArea().lumaPos(), CHANNEL_TYPE_LUMA);
      CodingUnit *nextCu           = nullptr;
      CodingUnit *tempLastLumaCu   = nullptr;
      CodingUnit *tempLastChromaCu = nullptr;
      ChannelType currentChType    = currentCu->chType;
      while (currentCu->next != nullptr)
      {
        nextCu = currentCu->next;
        if (currentChType != nextCu->chType && currentChType == CHANNEL_TYPE_LUMA)
        {
          tempLastLumaCu = currentCu;
          if (tempLastChromaCu != nullptr)   // swap
          {
            tempLastChromaCu->next = nextCu;
          }
        }
        else if (currentChType != nextCu->chType && currentChType == CHANNEL_TYPE_CHROMA)
        {
          tempLastChromaCu = currentCu;
          if (tempLastLumaCu != nullptr)   // swap
          {
            tempLastLumaCu->next = nextCu;
          }
        }
        currentCu     = nextCu;
        currentChType = currentCu->chType;
      }

      CodingUnit *chromaFirstCu = cs.getCU(pPartitionerChroma->currArea().chromaPos(), CHANNEL_TYPE_CHROMA);
      tempLastLumaCu->next      = chromaFirstCu;
    }
    else
    {
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      const ModeType modeTypeParent = partitioner.modeType;
      cs.modeType = partitioner.modeType = mode_constraint(cs, partitioner, splitMode);   // change for child nodes
      // decide chroma split or not
      bool chromaNotSplit = modeTypeParent == MODE_TYPE_ALL && partitioner.modeType == MODE_TYPE_INTRA;
      CHECK(chromaNotSplit && partitioner.chType != CHANNEL_TYPE_LUMA, "chType must be luma");
      if (partitioner.treeType == TREE_D)
      {
        cs.treeType = partitioner.treeType = chromaNotSplit ? TREE_L : TREE_D;
      }
#endif
      partitioner.splitCurrArea( splitMode, cs );
      do
      {
        if( cs.area.blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
        {
          coding_tree( cs, partitioner, cuCtx );
        }
      } while( partitioner.nextPart( cs ) );

      partitioner.exitCurrSplit();
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if( chromaNotSplit )
      {
        CHECK( partitioner.chType != CHANNEL_TYPE_LUMA, "must be luma status" );
        partitioner.chType = CHANNEL_TYPE_CHROMA;
        cs.treeType = partitioner.treeType = TREE_C;

        if( cs.picture->blocks[partitioner.chType].contains( partitioner.currArea().blocks[partitioner.chType].pos() ) )
        {
          coding_tree( cs, partitioner, cuCtx );
        }

        //recover treeType
        partitioner.chType = CHANNEL_TYPE_LUMA;
        cs.treeType = partitioner.treeType = TREE_D;
      }

      //recover ModeType
      cs.modeType = partitioner.modeType = modeTypeParent;
#endif
    }
    return;
  }

  CodingUnit& cu = cs.addCU( CS::getArea( cs, currArea, partitioner.chType ), partitioner.chType );

  partitioner.setCUData( cu );
  cu.slice   = cs.slice;
  cu.tileIdx = cs.pps->getTileIdx( currArea.lumaPos() );
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  CHECK( cu.cs->treeType != partitioner.treeType, "treeType mismatch" );
  int lumaQPinLocalDualTree = -1;
#endif
  // Predict QP on start of quantization group
  if( cuCtx.qgStart )
  {
    cuCtx.qgStart = false;
    cuCtx.qp = CU::predictQP( cu, cuCtx.qp );
  }
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (pps.getUseDQP() && CS::isDualITree(cs) && isChroma(cu.chType))
#else
  if (pps.getUseDQP() && partitioner.isSepTree(cs) && isChroma(cu.chType))
#endif
  {
    const Position chromaCentral(cu.chromaPos().offset(cu.chromaSize().width >> 1, cu.chromaSize().height >> 1));
    const Position lumaRefPos(chromaCentral.x << getComponentScaleX(COMPONENT_Cb, cu.chromaFormat), chromaCentral.y << getComponentScaleY(COMPONENT_Cb, cu.chromaFormat));
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    const CodingUnit* colLumaCu = cs.getCU(lumaRefPos, CHANNEL_TYPE_LUMA);
#else
    //derive chroma qp, but the chroma qp is saved in cuCtx.qp which is used for luma qp
    //therefore, after decoding the chroma CU, the cuCtx.qp shall be recovered to luma qp in order to decode next luma cu qp
    const CodingUnit* colLumaCu = cs.getLumaCU( lumaRefPos );
    CHECK( colLumaCu == nullptr, "colLumaCU shall exist" );
    lumaQPinLocalDualTree = cuCtx.qp;
#endif
    if (colLumaCu)
    {
      cuCtx.qp = colLumaCu->qp;
    }
  }

  cu.qp = cuCtx.qp;                 //NOTE: CU QP can be changed by deltaQP signaling at TU level
  cu.chromaQpAdj = cs.chromaQpAdj;  //NOTE: CU chroma QP adjustment can be changed by adjustment signaling at TU level

  // coding unit

  coding_unit( cu, partitioner, cuCtx );
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  //recover cuCtx.qp to luma qp after decoding the chroma CU
  if( pps.getUseDQP() && partitioner.isSepTree( cs ) && isChroma( cu.chType ) )
  {
    cuCtx.qp = lumaQPinLocalDualTree;
  }
#endif
  uint32_t compBegin;
  uint32_t numComp;
  bool jointPLT = false;
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (CS::isDualITree(*cu.cs))
#else
  if (cu.isSepTree())
#endif
  {
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if( cu.isLocalSepTree() )
    {
      compBegin = COMPONENT_Y;
      numComp = (cu.chromaFormat != CHROMA_400)?3: 1;
      jointPLT = true;
    }
    else
    {
#endif
      if (isLuma(partitioner.chType))
      {
        compBegin = COMPONENT_Y;
        numComp   = 1;
      }
      else
      {
        compBegin = COMPONENT_Cb;
        numComp   = 2;
      }
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    }
#endif
  }
  else
  {
    compBegin = COMPONENT_Y;
    numComp = (cu.chromaFormat != CHROMA_400) ? 3 : 1;
    jointPLT = true;
  }
  if (CU::isPLT(cu))
  {
    cs.reorderPrevPLT(cs.prevPLT, cu.curPLTSize, cu.curPLT, cu.reuseflag, compBegin, numComp, jointPLT);
  }
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( cu.chType == CHANNEL_TYPE_CHROMA )
  {
    DTRACE( g_trace_ctx, D_QP, "[chroma CU]x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.Cb().x, cu.Cb().y, cu.Cb().width, cu.Cb().height, cu.qp );
  }
  else
  {
#endif
    DTRACE(g_trace_ctx, D_QP, "x=%d, y=%d, w=%d, h=%d, qp=%d\n", cu.Y().x, cu.Y().y, cu.Y().width, cu.Y().height,
           cu.qp);
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  }
#endif
}
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
ModeType CABACReader::mode_constraint( CodingStructure& cs, Partitioner &partitioner, PartSplit splitMode )
{
  int val = cs.signalModeCons( splitMode, partitioner, partitioner.modeType );
  if( val == LDT_MODE_TYPE_SIGNAL )
  {
    int ctxIdx = DeriveCtx::CtxModeConsFlag( cs, partitioner );
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__MODE_CONSTRAINT_FLAG, partitioner.currArea().blocks[partitioner.chType].size(), partitioner.chType );
    bool flag = m_BinDecoder.decodeBin( Ctx::ModeConsFlag( ctxIdx ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "mode_cons_flag() flag=%d\n", flag );
    return flag ? MODE_TYPE_INTRA : MODE_TYPE_INTER;
  }
  else if( val == LDT_MODE_TYPE_INFER )
  {
    return MODE_TYPE_INTRA;
  }
  else
  {
    return partitioner.modeType;
  }
}
#endif
PartSplit CABACReader::split_cu_mode( CodingStructure& cs, Partitioner &partitioner )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__SPLIT_FLAG, partitioner.currArea().blocks[partitioner.chType].size(), partitioner.chType );

  PartSplit mode = CU_DONT_SPLIT;

  bool canNo, canQt, canBh, canBv, canTh, canTv;
  partitioner.canSplit( cs, canNo, canQt, canBh, canBv, canTh, canTv );

  bool canSpl[6] = { canNo, canQt, canBh, canBv, canTh, canTv };

  unsigned ctxSplit = 0, ctxQtSplit = 0, ctxBttHV = 0, ctxBttH12 = 0, ctxBttV12;
  DeriveCtx::CtxSplit( cs, partitioner, ctxSplit, ctxQtSplit, ctxBttHV, ctxBttH12, ctxBttV12, canSpl );

  bool isSplit = canBh || canBv || canTh || canTv || canQt;

  if( canNo && isSplit )
  {
    isSplit = m_BinDecoder.decodeBin( Ctx::SplitFlag( ctxSplit ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=%d split=%d\n", ctxSplit, isSplit );

  if( !isSplit )
  {
    return CU_DONT_SPLIT;
  }

  const bool canBtt = canBh || canBv || canTh || canTv;
  bool       isQt   = canQt;

  if( isQt && canBtt )
  {
    isQt = m_BinDecoder.decodeBin( Ctx::SplitQtFlag( ctxQtSplit ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctx=%d qt=%d\n", ctxQtSplit, isQt );

  if( isQt )
  {
    return CU_QUAD_SPLIT;
  }

  const bool canHor = canBh || canTh;
  bool        isVer = canBv || canTv;

  if( isVer && canHor )
  {
    isVer = m_BinDecoder.decodeBin( Ctx::SplitHvFlag( ctxBttHV ) );
  }

  const bool can14 = isVer ? canTv : canTh;
  bool        is12 = isVer ? canBv : canBh;

  if( is12 && can14 )
  {
    is12 = m_BinDecoder.decodeBin( Ctx::Split12Flag( isVer ? ctxBttV12 : ctxBttH12 ) );
  }

  if (isVer && is12)
  {
    mode = CU_VERT_SPLIT;
  }
  else if (isVer && !is12)
  {
    mode = CU_TRIV_SPLIT;
  }
  else if (!isVer && is12)
  {
    mode = CU_HORZ_SPLIT;
  }
  else
  {
    mode = CU_TRIH_SPLIT;
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "split_cu_mode() ctxHv=%d ctx12=%d mode=%d\n", ctxBttHV, isVer ? ctxBttV12 : ctxBttH12, mode );

  return mode;
}

//================================================================================
//  clause 7.3.8.5
//--------------------------------------------------------------------------------
//    void  coding_unit               ( cu, partitioner, cuCtx )
//    void  cu_skip_flag              ( cu )
//    void  pred_mode                 ( cu )
//    void  part_mode                 ( cu )
//    void  cu_pred_data              ( pus )
//    void  cu_lic_flag               ( cu )
//    void  intra_luma_pred_modes     ( pus )
//    void  intra_chroma_pred_mode    ( pu )
//    void  cu_residual               ( cu, partitioner, cuCtx )
//    void  rqt_root_cbf              ( cu )
//    void  end_of_ctu                ( cu, cuCtx )
//================================================================================

void CABACReader::coding_unit( CodingUnit &cu, Partitioner &partitioner, CUCtx& cuCtx )
{
  CodingStructure& cs = *cu.cs;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  CHECK( cu.treeType != partitioner.treeType || cu.modeType != partitioner.modeType, "treeType or modeType mismatch" );
  DTRACE( g_trace_ctx, D_SYNTAX, "coding_unit() treeType=%d modeType=%d\n", cu.treeType, cu.modeType );
#endif
  PredictionUnit&    pu = cs.addPU(cu, partitioner.chType);
  // skip flag
  if ((!cs.slice->isIntra() || cs.slice->getSPS()->getIBCFlag()) && cu.Y().valid())
  {
    cu_skip_flag( cu );
  }

  // skip data
  if( cu.skip )
  {
    cu.colorTransform = false;
    cs.addEmptyTUs( partitioner );
    MergeCtx           mrgCtx;
    prediction_unit  ( pu, mrgCtx );
#if INTER_LIC
    cu_lic_flag      ( cu );
#endif
#if ENABLE_OBMC
    // it just sets the OBMC flag and does not parse any syntax.
    obmc_flag        ( cu );
#endif
    end_of_ctu( cu, cuCtx );
    return;
  }

  // prediction mode and partitioning data
  pred_mode ( cu );
#if ENABLE_DIMD
  cu_dimd_flag( cu );
#endif
  if (CU::isIntra(cu))
  {
    adaptive_color_transform(cu);
  }
  if (CU::isPLT(cu))
  {
    cu.colorTransform = false;
    cs.addTU(cu, partitioner.chType);
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (CS::isDualITree(*cu.cs))
#else
    if (cu.isSepTree())
#endif
    {
      if (isLuma(partitioner.chType))
      {
        cu_palette_info(cu, COMPONENT_Y, 1, cuCtx);
      }
      if (cu.chromaFormat != CHROMA_400 && (partitioner.chType == CHANNEL_TYPE_CHROMA))
      {
        cu_palette_info(cu, COMPONENT_Cb, 2, cuCtx);
      }
    }
    else
    {
      if( cu.chromaFormat != CHROMA_400 )
      {
        cu_palette_info(cu, COMPONENT_Y, 3, cuCtx);
      }
      else
      {
        cu_palette_info(cu, COMPONENT_Y, 1, cuCtx);
      }
    }
    end_of_ctu(cu, cuCtx);
    return;
  }

  // --> create PUs

  // prediction data ( intra prediction modes / reference indexes + motion vectors )
  cu_pred_data( cu );

  // residual data ( coded block flags + transform coefficient levels )
  cu_residual( cu, partitioner, cuCtx );

  // check end of cu
  end_of_ctu( cu, cuCtx );
}

void CABACReader::cu_skip_flag( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__SKIP_FLAG, cu.lumaSize());
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (cu.slice->isIntra() && cu.cs->slice->getSPS()->getIBCFlag())
#else
  if ((cu.slice->isIntra() || cu.isConsIntra()) && cu.cs->slice->getSPS()->getIBCFlag())
#endif
  {
    cu.skip = false;
    cu.rootCbf = false;
    cu.predMode = MODE_INTRA;
    cu.mmvdSkip = false;
    if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
    {
      unsigned ctxId = DeriveCtx::CtxSkipFlag(cu);
      unsigned skip  = m_BinDecoder.decodeBin(Ctx::SkipFlag(ctxId));
      DTRACE(g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, skip ? 1 : 0);
      if (skip)
      {
        cu.skip     = true;
        cu.rootCbf  = false;
        cu.predMode = MODE_IBC;
        cu.mmvdSkip = false;
      }
    }
    return;
  }
#if !INTER_RM_SIZE_CONSTRAINTS
  if ( !cu.cs->slice->getSPS()->getIBCFlag() && cu.lwidth() == 4 && cu.lheight() == 4 )
  {
    return;
  }
#endif
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( !cu.cs->slice->getSPS()->getIBCFlag() && cu.isConsIntra() )
  {
    return;
  }
#endif
  unsigned ctxId  = DeriveCtx::CtxSkipFlag(cu);
  unsigned skip   = m_BinDecoder.decodeBin( Ctx::SkipFlag(ctxId) );

  DTRACE( g_trace_ctx, D_SYNTAX, "cu_skip_flag() ctx=%d skip=%d\n", ctxId, skip ? 1 : 0 );

  if (skip && cu.cs->slice->getSPS()->getIBCFlag())
  {
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64 and disable IBC when only allowing inter mode
#else
    if (cu.lwidth() < 128 && cu.lheight() < 128 && !cu.isConsInter()) // disable IBC mode larger than 64x64 and disable IBC when only allowing inter mode
#endif
    {
#if !INTER_RM_SIZE_CONSTRAINTS
      if ( cu.lwidth() == 4 && cu.lheight() == 4 )
      {
        cu.skip     = true;
        cu.rootCbf  = false;
        cu.predMode = MODE_IBC;
        cu.mmvdSkip = false;
        return;
      }
#endif
      unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
      if (m_BinDecoder.decodeBin(Ctx::IBCFlag(ctxidx)))
      {
        cu.skip                      = true;
        cu.rootCbf                   = false;
        cu.predMode                  = MODE_IBC;
        cu.mmvdSkip                  = false;
        cu.firstPU->regularMergeFlag = false;
      }
      else
      {
        cu.predMode = MODE_INTER;
      }
      DTRACE(g_trace_ctx, D_SYNTAX, "ibc() ctx=%d cu.predMode=%d\n", ctxidx, cu.predMode);
    }
    else
    {
      cu.predMode = MODE_INTER;
    }
  }
  if ((skip && CU::isInter(cu) && cu.cs->slice->getSPS()->getIBCFlag()) ||
    (skip && !cu.cs->slice->getSPS()->getIBCFlag()))
  {
    cu.skip     = true;
    cu.rootCbf  = false;
    cu.predMode = MODE_INTER;
  }
}

void CABACReader::imv_mode( CodingUnit& cu, MergeCtx& mrgCtx )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__IMV_FLAG, cu.lumaSize());

  if( !cu.cs->sps->getAMVREnabledFlag() )
  {
    return;
  }

#if JVET_X0083_BM_AMVP_MERGE_MODE
  auto &pu = *cu.firstPU;
  if (pu.amvpMergeModeFlag[REF_PIC_LIST_0] || pu.amvpMergeModeFlag[REF_PIC_LIST_1])
  {
    return;
  }
#endif
  bool bNonZeroMvd = CU::hasSubCUNonZeroMVd( cu );
  if( !bNonZeroMvd )
  {
    return;
  }

  if ( cu.affine )
  {
    return;
  }

  const SPS *sps = cu.cs->sps;

  unsigned value = 0;
  if (CU::isIBC(cu))
  {
    value = 1;
  }
  else
  {
    value = m_BinDecoder.decodeBin(Ctx::ImvFlag(0));
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", value, 0 );

  cu.imv = value;
  if( sps->getAMVREnabledFlag() && value )
  {
    if (!CU::isIBC(cu))
    {
      value = m_BinDecoder.decodeBin(Ctx::ImvFlag(4));
      DTRACE(g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", value, 4);
      cu.imv = value ? 1 : IMV_HPEL;
    }
    if (value)
    {
      value = m_BinDecoder.decodeBin(Ctx::ImvFlag(1));
      DTRACE(g_trace_ctx, D_SYNTAX, "imv_mode() value=%d ctx=%d\n", value, 1);
      value++;
      cu.imv = value;
    }
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "imv_mode() IMVFlag=%d\n", cu.imv );
}
void CABACReader::affine_amvr_mode( CodingUnit& cu, MergeCtx& mrgCtx )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__AFFINE_AMVR, cu.lumaSize());

  const SPS* sps = cu.slice->getSPS();

  if( !sps->getAffineAmvrEnabledFlag() || !cu.affine )
  {
    return;
  }

  if ( !CU::hasSubCUNonZeroAffineMVd( cu ) )
  {
    return;
  }

  unsigned value = 0;
  value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 2 ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", value, 2 );

  if( value )
  {
    value = m_BinDecoder.decodeBin( Ctx::ImvFlag( 3 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() value=%d ctx=%d\n", value, 3 );
    value++;
  }

  cu.imv = value;
  DTRACE( g_trace_ctx, D_SYNTAX, "affine_amvr_mode() IMVFlag=%d\n", cu.imv );
}

void CABACReader::pred_mode( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__PRED_MODE, cu.block((!CS::isDualITree(*cu.cs) || isLuma(cu.chType))? COMPONENT_Y: COMPONENT_Cb).lumaSize(), (!CS::isDualITree(*cu.cs) || isLuma(cu.chType)) ? CHANNEL_TYPE_LUMA : CHANNEL_TYPE_CHROMA);
  if (cu.cs->slice->getSPS()->getIBCFlag() && cu.chType != CHANNEL_TYPE_CHROMA)
  {
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if( cu.isConsInter() )
    {
      cu.predMode = MODE_INTER;
      return;
    }
#endif
#if INTER_RM_SIZE_CONSTRAINTS
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (cu.cs->slice->isIntra())
#else
    if ( cu.cs->slice->isIntra() || cu.isConsIntra() )
#endif
#else
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (cu.cs->slice->isIntra() || (cu.lwidth() == 4 && cu.lheight() == 4))
#else
    if ( cu.cs->slice->isIntra() || ( cu.lwidth() == 4 && cu.lheight() == 4 ) || cu.isConsIntra() )
#endif
#endif
    {
      cu.predMode = MODE_INTRA;
      if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
      {
        unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
        if (m_BinDecoder.decodeBin(Ctx::IBCFlag(ctxidx)))
        {
          cu.predMode = MODE_IBC;
        }
      }
      if (!CU::isIBC(cu) && cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64 && (cu.lumaSize().width * cu.lumaSize().height > 16) )
      {
        if (m_BinDecoder.decodeBin(Ctx::PLTFlag(0)))
        {
          cu.predMode = MODE_PLT;
        }
      }
    }
    else
    {
      if (m_BinDecoder.decodeBin(Ctx::PredMode(DeriveCtx::CtxPredModeFlag(cu))))
      {
        cu.predMode = MODE_INTRA;
        if (cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64 && (cu.lumaSize().width * cu.lumaSize().height > 16) )
        {
          if (m_BinDecoder.decodeBin(Ctx::PLTFlag(0)))
          {
            cu.predMode = MODE_PLT;
          }
        }
      }
      else
      {
        cu.predMode = MODE_INTER;
        if (cu.lwidth() < 128 && cu.lheight() < 128) // disable IBC mode larger than 64x64
        {
          unsigned ctxidx = DeriveCtx::CtxIBCFlag(cu);
          if (m_BinDecoder.decodeBin(Ctx::IBCFlag(ctxidx)))
          {
            cu.predMode = MODE_IBC;
          }
        }
      }
    }
  }
  else
  {
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if( cu.isConsInter() )
    {
      cu.predMode = MODE_INTER;
      return;
    }
#endif
#if INTER_RM_SIZE_CONSTRAINTS
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (cu.cs->slice->isIntra())
#else
    if ( cu.cs->slice->isIntra() || cu.isConsIntra() )
#endif
#else
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (cu.cs->slice->isIntra() || (cu.lwidth() == 4 && cu.lheight() == 4))
#else
    if ( cu.cs->slice->isIntra() || (cu.lwidth() == 4 && cu.lheight() == 4) || cu.isConsIntra() )
#endif
#endif
    {
      cu.predMode = MODE_INTRA;
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if (cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64 && (((!isLuma(cu.chType)) && (cu.chromaSize().width * cu.chromaSize().height > 16)) || ((isLuma(cu.chType)) && ((cu.lumaSize().width * cu.lumaSize().height) > 16))) && (CS::isDualITree(*cu.cs) || isLuma(cu.chType)))
#else
      if (cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64 && ( ( (!isLuma(cu.chType)) && (cu.chromaSize().width * cu.chromaSize().height > 16) ) || ((isLuma(cu.chType)) && ((cu.lumaSize().width * cu.lumaSize().height) > 16 ) )  ) && (!cu.isLocalSepTree() || isLuma(cu.chType)  )  )
#endif
      {
        if (m_BinDecoder.decodeBin(Ctx::PLTFlag(0)))
        {
          cu.predMode = MODE_PLT;
        }
      }
    }
    else
    {
      cu.predMode = m_BinDecoder.decodeBin(Ctx::PredMode(DeriveCtx::CtxPredModeFlag(cu))) ? MODE_INTRA : MODE_INTER;
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if (CU::isIntra(cu) && cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64 && (((!isLuma(cu.chType)) && (cu.chromaSize().width * cu.chromaSize().height > 16)) || ((isLuma(cu.chType)) && ((cu.lumaSize().width * cu.lumaSize().height) > 16))) && (CS::isDualITree(*cu.cs) || isLuma(cu.chType)))
#else
      if (CU::isIntra(cu) && cu.cs->slice->getSPS()->getPLTMode() && cu.lwidth() <= 64 && cu.lheight() <= 64 && ( ( (!isLuma(cu.chType)) && (cu.chromaSize().width * cu.chromaSize().height > 16) ) || ((isLuma(cu.chType)) && ((cu.lumaSize().width * cu.lumaSize().height) > 16 ) )  ) && (!cu.isLocalSepTree() || isLuma(cu.chType)  )  )
#endif
      {
        if (m_BinDecoder.decodeBin(Ctx::PLTFlag(0)))
        {
          cu.predMode = MODE_PLT;
        }
      }
    }
  }
}

void CABACReader::bdpcm_mode( CodingUnit& cu, const ComponentID compID )
{
  if (!CU::bdpcmAllowed(cu, compID))
  {
    if (isLuma(compID))
    {
      cu.bdpcmMode = 0;
      if (!CS::isDualITree(*cu.cs))
      {
        cu.bdpcmModeChroma = 0;
      }
    }
    else
    {
      cu.bdpcmModeChroma = 0;
    }
    return;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__BDPCM_MODE, cu.block(compID).lumaSize(), compID );

  int bdpcmMode;
  unsigned ctxId = isLuma( compID ) ? 0 : 2;
  bdpcmMode = m_BinDecoder.decodeBin( Ctx::BDPCMMode(ctxId) );
  if (bdpcmMode)
  {
    bdpcmMode += m_BinDecoder.decodeBin( Ctx::BDPCMMode(ctxId+1) );
  }
  if (isLuma(compID))
  {
    cu.bdpcmMode = bdpcmMode;
  }
  else
  {
    cu.bdpcmModeChroma = bdpcmMode;
  }
  if (isLuma(compID))
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "bdpcm_mode(%d) x=%d, y=%d, w=%d, h=%d, bdpcm=%d\n", CHANNEL_TYPE_LUMA, cu.lumaPos().x, cu.lumaPos().y, cu.lwidth(), cu.lheight(), cu.bdpcmMode);
  }
  else
  {
    DTRACE(g_trace_ctx, D_SYNTAX, "bdpcm_mode(%d) x=%d, y=%d, w=%d, h=%d, bdpcm=%d\n", CHANNEL_TYPE_CHROMA, cu.chromaPos().x, cu.chromaPos().y, cu.chromaSize().width, cu.chromaSize().height, cu.bdpcmModeChroma);
  }
}

void CABACReader::cu_pred_data( CodingUnit &cu )
{
  if( CU::isIntra( cu ) )
  {
    if( cu.Y().valid() )
	{
      bdpcm_mode(cu, COMPONENT_Y );
    }
    intra_luma_pred_modes( cu );
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if ((!cu.Y().valid() || (!CS::isDualITree(*cu.cs) && cu.Y().valid())) && isChromaEnabled(cu.chromaFormat))
#else
    if( ( !cu.Y().valid() || (!cu.isSepTree() && cu.Y().valid() ) ) && isChromaEnabled(cu.chromaFormat) )
#endif
    {
      bdpcm_mode(cu, ComponentID(CHANNEL_TYPE_CHROMA));
    }
    intra_chroma_pred_modes( cu );
    return;
  }
  if (!cu.Y().valid()) // dual tree chroma CU
  {
    cu.predMode = MODE_IBC;
    return;
  }
  MergeCtx mrgCtx;

  for( auto &pu : CU::traversePUs( cu ) )
  {
    prediction_unit( pu, mrgCtx );
  }

  imv_mode   ( cu, mrgCtx );
  affine_amvr_mode( cu, mrgCtx );
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  for (auto &pu : CU::traversePUs(cu))
  {
    mvsd_data(pu);
  }
#endif
#if INTER_LIC
  cu_lic_flag( cu ); // local illumination compensation
#endif
#if !JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED)
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  if (!cu.cs->sps->getUseMVSD())
  {
#endif
  cu_bcw_flag(cu);
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  }
#endif
#endif
#if ENABLE_OBMC
  obmc_flag  ( cu );
#endif
}

void CABACReader::cu_bcw_flag(CodingUnit& cu)
{
  if(!CU::isBcwIdxCoded(cu))
  {
    return;
  }
#if JVET_X0083_BM_AMVP_MERGE_MODE
  auto &pu = *cu.firstPU;
  if (pu.amvpMergeModeFlag[REF_PIC_LIST_0] || pu.amvpMergeModeFlag[REF_PIC_LIST_1])
  {
    return;
  }
#endif

  CHECK(!(BCW_NUM > 1 && (BCW_NUM == 2 || (BCW_NUM & 0x01) == 1)), " !( BCW_NUM > 1 && ( BCW_NUM == 2 || ( BCW_NUM & 0x01 ) == 1 ) ) ");

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__BCW_IDX, cu.lumaSize());

  uint32_t idx = 0;

  uint32_t symbol = m_BinDecoder.decodeBin(Ctx::BcwIdx(0));

  int32_t numBcw = (cu.slice->getCheckLDC()) ? 5 : 3;
  if(symbol == 1)
  {
    uint32_t prefixNumBits = numBcw - 2;
    uint32_t step = 1;

    idx = 1;

    for(int ui = 0; ui < prefixNumBits; ++ui)
    {
      symbol = m_BinDecoder.decodeBinEP();
      if (symbol == 0)
      {
        break;
      }
      idx += step;
    }
  }

  uint8_t bcwIdx = (uint8_t)g_BcwParsingOrder[idx];
  CU::setBcwIdx(cu, bcwIdx);

  DTRACE(g_trace_ctx, D_SYNTAX, "cu_bcw_flag() bcw_idx=%d\n", cu.BcwIdx ? 1 : 0);
#if MULTI_HYP_PRED
  mh_pred_data(*cu.firstPU);
#endif
}

#if ENABLE_OBMC
void CABACReader::obmc_flag(CodingUnit& cu)
{
  if (!cu.cs->sps->getUseOBMC() || CU::isIBC(cu) || cu.predMode == MODE_INTRA
#if INTER_LIC
    || cu.LICFlag
#endif
    || cu.lwidth() * cu.lheight() < 32
    )
  {
    cu.obmcFlag = false;
    return;
  }
  if (cu.firstPU->mergeFlag)
  {
    cu.obmcFlag = true;
    return;
  }
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__OBMC, cu.lumaSize());

  cu.obmcFlag = (bool)m_BinDecoder.decodeBin(Ctx::ObmcFlag());
}
#endif

void CABACReader::xReadTruncBinCode(uint32_t& symbol, uint32_t maxSymbol)
{
  int thresh;
  if (maxSymbol > 256)
  {
    int threshVal = 1 << 8;
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

  int val = 1 << thresh;
  int b = maxSymbol - val;
  symbol = m_BinDecoder.decodeBinsEP(thresh);
  if (symbol >= val - b)
  {
    uint32_t altSymbol;
    altSymbol = m_BinDecoder.decodeBinEP();
    symbol <<= 1;
    symbol += altSymbol;
    symbol -= (val - b);
  }
}

void CABACReader::extend_ref_line(CodingUnit& cu)
{
  if ( !cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType) || cu.bdpcmMode
#if ENABLE_DIMD
    || cu.dimd
#endif
#if JVET_AB0155_SGPM
    || cu.sgpm
#endif
    )
  {
    cu.firstPU->multiRefIdx = 0;
    return;
  }
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MULTI_REF_LINE, cu.lumaSize());

  const int numBlocks = CU::getNumPUs(cu);
  PredictionUnit* pu = cu.firstPU;

  for (int k = 0; k < numBlocks; k++)
  {
    if( !cu.cs->sps->getUseMRL() )
    {
      pu->multiRefIdx = 0;
      pu = pu->next;
      continue;
    }
    bool isFirstLineOfCtu = (((cu.block(COMPONENT_Y).y)&((cu.cs->sps)->getMaxCUWidth() - 1)) == 0);
    if (isFirstLineOfCtu)
    {
      pu->multiRefIdx = 0;
      continue;
    }
    int multiRefIdx = 0;

#if JVET_Y0116_EXTENDED_MRL_LIST
    if (MRL_NUM_REF_LINES > 1)
    {
#if JVET_W0123_TIMD_FUSION
      multiRefIdx = m_BinDecoder.decodeBin(cu.timd ? Ctx::MultiRefLineIdx(5) : Ctx::MultiRefLineIdx(0)) == 1 ? MULTI_REF_LINE_IDX[1] : MULTI_REF_LINE_IDX[0];
#else
      multiRefIdx = m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(0)) == 1 ? MULTI_REF_LINE_IDX[1] : MULTI_REF_LINE_IDX[0];
#endif
      if (MRL_NUM_REF_LINES > 2 && multiRefIdx != MULTI_REF_LINE_IDX[0])
      {
#if JVET_W0123_TIMD_FUSION
        multiRefIdx = m_BinDecoder.decodeBin(cu.timd ? Ctx::MultiRefLineIdx(6) : Ctx::MultiRefLineIdx(1)) == 1 ? MULTI_REF_LINE_IDX[2] : MULTI_REF_LINE_IDX[1];
#else
        multiRefIdx = m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(1)) == 1 ? MULTI_REF_LINE_IDX[2] : MULTI_REF_LINE_IDX[1];
#endif
        if (MRL_NUM_REF_LINES > 3 && multiRefIdx != MULTI_REF_LINE_IDX[1]
#if JVET_W0123_TIMD_FUSION
        && !cu.timd
#endif
          )
        {
          multiRefIdx = m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(2)) == 1 ? MULTI_REF_LINE_IDX[3] : MULTI_REF_LINE_IDX[2];
          if (MRL_NUM_REF_LINES > 4 && multiRefIdx != MULTI_REF_LINE_IDX[2])
          {
            multiRefIdx = m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(3)) == 1 ? MULTI_REF_LINE_IDX[4] : MULTI_REF_LINE_IDX[3];
            if (MRL_NUM_REF_LINES > 5 && multiRefIdx != MULTI_REF_LINE_IDX[3])
            {
              multiRefIdx = m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(4)) == 1 ? MULTI_REF_LINE_IDX[5] : MULTI_REF_LINE_IDX[4];
            }
          }
        }
      }
    }
#else
    if (MRL_NUM_REF_LINES > 1)
    {
#if JVET_W0123_TIMD_FUSION
      multiRefIdx = m_BinDecoder.decodeBin(cu.timd ? Ctx::MultiRefLineIdx(2) : Ctx::MultiRefLineIdx(0)) == 1 ? MULTI_REF_LINE_IDX[1] : MULTI_REF_LINE_IDX[0];
#else
      multiRefIdx = m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(0)) == 1 ? MULTI_REF_LINE_IDX[1] : MULTI_REF_LINE_IDX[0];
#endif
      if (MRL_NUM_REF_LINES > 2 && multiRefIdx != MULTI_REF_LINE_IDX[0])
      {
#if JVET_W0123_TIMD_FUSION
        multiRefIdx = m_BinDecoder.decodeBin(cu.timd ? Ctx::MultiRefLineIdx(3) : Ctx::MultiRefLineIdx(1)) == 1 ? MULTI_REF_LINE_IDX[2] : MULTI_REF_LINE_IDX[1];
#else
        multiRefIdx = m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(1)) == 1 ? MULTI_REF_LINE_IDX[2] : MULTI_REF_LINE_IDX[1];
#endif
      }

    }
#endif
    pu->multiRefIdx = multiRefIdx;
    DTRACE(g_trace_ctx, D_SYNTAX, "extend_ref_line() idx=%d pos=(%d,%d) ref_idx=%d\n", k, pu->lumaPos().x, pu->lumaPos().y, pu->multiRefIdx);
    pu = pu->next;
  }
}

void CABACReader::intra_luma_pred_modes( CodingUnit &cu )
{
  if( !cu.Y().valid() )
  {
    return;
  }

  if( cu.bdpcmMode )
  {
    cu.firstPU->intraDir[0] = cu.bdpcmMode == 2? VER_IDX : HOR_IDX;
    return;
  }
#if JVET_V0130_INTRA_TMP
  int tmpMaxSize=cu.cs->sps->getIntraTMPMaxSize();
  if( cu.lwidth() <= tmpMaxSize && cu.lheight() <= tmpMaxSize )
  {
	  tmp_flag(cu);
    if( cu.tmpFlag )
    {
      return;
    }
  }
  else
  {
    cu.tmpFlag = 0;
  }
#endif
  mip_flag(cu);
  if (cu.mipFlag)
  {
    mip_pred_modes(cu);
    return;
  }
#if JVET_W0123_TIMD_FUSION
  cu_timd_flag(cu);
#endif
#if JVET_AB0155_SGPM
  sgpm_flag(cu);
  if (cu.sgpm)
  {
    return;
  }
#endif

#if JVET_AB0157_TMRL
  cuTmrlFlag(cu);
  if (cu.tmrlFlag)
  {
    return;
  }
#else
  extend_ref_line( cu );
#endif
  isp_mode( cu );
#if ENABLE_DIMD
  if (cu.dimd)
  {
    return;
  }
#endif
#if JVET_W0123_TIMD_FUSION
  if (cu.timd)
  {
    return;
  }
#endif
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__INTRA_DIR_ANG, cu.lumaSize(), CHANNEL_TYPE_LUMA );

  // prev_intra_luma_pred_flag
  int numBlocks = CU::getNumPUs( cu );
  int mpmFlag[4];
  for( int k = 0; k < numBlocks; k++ )
  {
    CHECK(numBlocks != 1, "not supported yet");
    if ( cu.firstPU->multiRefIdx )
    {
      mpmFlag[0] = true;
    }
    else
    {
      mpmFlag[k] = m_BinDecoder.decodeBin(Ctx::IntraLumaMpmFlag());
    }
  }

  PredictionUnit *pu = cu.firstPU;

#if SECONDARY_MPM
  uint8_t* mpm_pred = pu->intraMPM;  // mpm_idx / rem_intra_luma_pred_mode
  uint8_t* non_mpm_pred = pu->intraNonMPM;
#else
  unsigned int mpm_pred[NUM_MOST_PROBABLE_MODES];  // mpm_idx / rem_intra_luma_pred_mode
#endif

  for( int k = 0; k < numBlocks; k++ )
  {
#if SECONDARY_MPM
    PU::getIntraMPMs( *pu, mpm_pred, non_mpm_pred);
#else
    PU::getIntraMPMs(*pu, mpm_pred);
#endif
#if ENABLE_DIMD || JVET_W0123_TIMD_FUSION
    pu->parseLumaMode = true;
    pu->mpmFlag = mpmFlag[k];
#endif
    if( mpmFlag[k] )
    {
      uint32_t ipred_idx = 0;
      {
        unsigned ctx = (pu->cu->ispMode == NOT_INTRA_SUBPARTITIONS ? 1 : 0);
#if SECONDARY_MPM
        unsigned ctx2 = (ctx ? (cu.firstPU->multiRefIdx == 0 ? 2 : 1) : 0);
#endif
        if( pu->multiRefIdx == 0 )
        {
          ipred_idx = m_BinDecoder.decodeBin( Ctx::IntraLumaPlanarFlag( ctx ) );
        }
        else
        {
          ipred_idx = 1;
        }

        if( ipred_idx )
        {
#if SECONDARY_MPM
          ipred_idx += m_BinDecoder.decodeBin(Ctx::IntraLumaMPMIdx(0 + ctx2));
#else
          ipred_idx += m_BinDecoder.decodeBinEP();
#endif
        }
        if (ipred_idx > 1)
        {
          ipred_idx += m_BinDecoder.decodeBinEP();
        }
        if (ipred_idx > 2)
        {
          ipred_idx += m_BinDecoder.decodeBinEP();
        }
        if (ipred_idx > 3)
        {
          ipred_idx += m_BinDecoder.decodeBinEP();
        }
      }
#if ENABLE_DIMD || JVET_W0123_TIMD_FUSION
      pu->secondMpmFlag = false;
      pu->ipred_idx = ipred_idx;
#endif
      pu->intraDir[0] = mpm_pred[ipred_idx];
    }
    else
    {
      unsigned ipred_mode = 0;

      {
#if SECONDARY_MPM
        if (m_BinDecoder.decodeBin(Ctx::IntraLumaSecondMpmFlag()))
        {
#if ENABLE_DIMD || JVET_W0123_TIMD_FUSION
          int idx = m_BinDecoder.decodeBinsEP(4) + NUM_PRIMARY_MOST_PROBABLE_MODES;
          ipred_mode = mpm_pred[idx];
          pu->secondMpmFlag = true;
          pu->ipred_idx = idx;
#else
          ipred_mode = mpm_pred[m_BinDecoder.decodeBinsEP(4) + NUM_PRIMARY_MOST_PROBABLE_MODES];
#endif
        }
        else
        {
          xReadTruncBinCode(ipred_mode, NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES);
#if ENABLE_DIMD || JVET_W0123_TIMD_FUSION
          pu->secondMpmFlag = false;
          pu->ipred_idx = ipred_mode;
#endif
          ipred_mode = non_mpm_pred[ipred_mode];
        }
#else
        xReadTruncBinCode(ipred_mode, NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES);
#if ENABLE_DIMD || JVET_W0123_TIMD_FUSION
        pu->ipred_idx = ipred_mode;
#endif
#endif
      }
#if !SECONDARY_MPM
      //postponed sorting of MPMs (only in remaining branch)
      std::sort( mpm_pred, mpm_pred + NUM_MOST_PROBABLE_MODES );

      for( uint32_t i = 0; i < NUM_MOST_PROBABLE_MODES; i++ )
      {
        ipred_mode += (ipred_mode >= mpm_pred[i]);
      }
#endif

      pu->intraDir[0] = ipred_mode;
    }

#if JVET_AC0105_DIRECTIONAL_PLANAR
    if (CU::isDirectionalPlanarAvailable(cu) && pu->ipred_idx == 0 && mpmFlag[k])
    {
      uint8_t plIdx = 0;
      plIdx         = m_BinDecoder.decodeBin(Ctx::IntraLumaPlanarFlag(2));
      if (plIdx)
      {
        plIdx += m_BinDecoder.decodeBin(Ctx::IntraLumaPlanarFlag(3));
      }
      cu.plIdx = plIdx;
    }
    else
    {
      cu.plIdx = 0;
    }
#endif

#if ENABLE_DIMD || JVET_W0123_TIMD_FUSION
    DTRACE( g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) predIdx=%d mpm=%d secondmpm=%d \n", k, pu->lumaPos().x, pu->lumaPos().y, pu->ipred_idx, pu->mpmFlag, pu->secondMpmFlag);
#else
    DTRACE( g_trace_ctx, D_SYNTAX, "intra_luma_pred_modes() idx=%d pos=(%d,%d) mode=%d\n", k, pu->lumaPos().x, pu->lumaPos().y, pu->intraDir[0] );
#endif
    pu = pu->next;
  }
}
#if ENABLE_DIMD
void CABACReader::cu_dimd_flag(CodingUnit& cu)
{
  if (!cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType) || !cu.slice->getSPS()->getUseDimd())
  {
    cu.dimd = false;
    return;
  }
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__INTRA_DIMD, cu.lumaSize(), COMPONENT_Y);

  unsigned ctxId = DeriveCtx::CtxDIMDFlag(cu);
  cu.dimd = m_BinDecoder.decodeBin(Ctx::DimdFlag(ctxId));
  DTRACE(g_trace_ctx, D_SYNTAX, "cu_dimd_flag() ctx=%d pos=(%d,%d) dimd=%d\n", ctxId, cu.lumaPos().x, cu.lumaPos().y, cu.dimd);
}
#endif

#if JVET_W0123_TIMD_FUSION
void CABACReader::cu_timd_flag( CodingUnit& cu )
{
  if (!cu.cs->sps->getUseTimd())
  {
    cu.timd = false;
    return;
  }
  if (cu.lwidth() * cu.lheight() > 1024 && cu.slice->getSliceType() == I_SLICE)
  {
    cu.timd = false;
    return;
  }
#if ENABLE_DIMD
  if (cu.dimd)
  {
    cu.timd = false;
    return;
  }
#endif
  if (!cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType))
  {
    cu.timd = false;
    return;
  }
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__INTRA_TIMD, cu.lumaSize(), COMPONENT_Y);

  unsigned ctxId = DeriveCtx::CtxTimdFlag( cu );
  cu.timd = m_BinDecoder.decodeBin( Ctx::TimdFlag(ctxId) );
  DTRACE(g_trace_ctx, D_SYNTAX, "cu_timd_flag() ctx=%d pos=(%d,%d) timd=%d\n", ctxId, cu.lumaPos().x, cu.lumaPos().y, cu.timd);
}
#endif

#if JVET_AB0155_SGPM
void CABACReader::sgpm_flag(CodingUnit &cu)
{
  if (!cu.cs->sps->getUseSgpm())
  {
    cu.sgpm = false;
    return;
  }
  if (!(cu.lwidth() >= GEO_MIN_CU_SIZE_EX && cu.lheight() >= GEO_MIN_CU_SIZE_EX && cu.lwidth() <= GEO_MAX_CU_SIZE_EX
        && cu.lheight() <= GEO_MAX_CU_SIZE_EX && cu.lwidth() < 8 * cu.lheight() && cu.lheight() < 8 * cu.lwidth()
        && cu.lwidth() * cu.lheight() >= SGPM_MIN_PIX))
  {
    cu.sgpm = false;
    return;
  }

  if( cu.mipFlag
#if ENABLE_DIMD
    || cu.dimd
#endif
#if JVET_W0123_TIMD_FUSION
    || cu.timd
#endif
#if JVET_V0130_INTRA_TMP
    || cu.tmpFlag
#endif
    )
  {
    cu.sgpm = false;
    return;
  }
  if (!cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType))
  {
    cu.sgpm = false;
    return;
  }
  if (!(cu.lx() && cu.ly()))
  {
    cu.sgpm = false;
    return;
  }

  unsigned ctxId = DeriveCtx::CtxSgpmFlag(cu);
  cu.sgpm        = m_BinDecoder.decodeBin(Ctx::SgpmFlag(ctxId));

  if (cu.sgpm)
  {
    uint32_t sgpmIdx = 0;
    xReadTruncBinCode(sgpmIdx, SGPM_NUM);
    cu.sgpmIdx = sgpmIdx;
  }
}
#endif

void CABACReader::intra_chroma_pred_modes( CodingUnit& cu )
{
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (cu.chromaFormat == CHROMA_400 || (CS::isDualITree(*cu.cs) && cu.chType == CHANNEL_TYPE_LUMA))
#else
  if( cu.chromaFormat == CHROMA_400 || ( cu.isSepTree() && cu.chType == CHANNEL_TYPE_LUMA ) )
#endif
  {
    return;
  }

  if( cu.bdpcmModeChroma )
  {
    cu.firstPU->intraDir[1] = cu.bdpcmModeChroma == 2 ? VER_IDX : HOR_IDX;
    return;
  }
  PredictionUnit *pu = cu.firstPU;

  CHECK(pu->cu != &cu, "Inkonsistent PU-CU mapping");
  intra_chroma_pred_mode(*pu);
}

#if JVET_Z0050_CCLM_SLOPE
void CABACReader::cclmDelta(PredictionUnit& pu, int8_t &delta)
{
  if ( delta )
  {
    int flag = 1;
    
    while (flag && delta < 4)
    {
      flag   = m_BinDecoder.decodeBinEP();
      delta += flag;
    }

    delta *= m_BinDecoder.decodeBin(Ctx::CclmDeltaFlags(4)) ? -1 : +1;
  }
}

void CABACReader::cclmDeltaSlope(PredictionUnit& pu)
{
#if JVET_AA0057_CCCM
  if ( pu.cccmFlag )
  {
    return;
  }
#endif

  if ( PU::hasCclmDeltaFlag( pu ) )
  {
    bool deltaActive = m_BinDecoder.decodeBin(Ctx::CclmDeltaFlags(0));

    if ( deltaActive )
    {
      bool bothActive = m_BinDecoder.decodeBin(Ctx::CclmDeltaFlags(3));

      pu.cclmOffsets.cb0 = bothActive;
      pu.cclmOffsets.cr0 = bothActive;
      
      if ( !bothActive )
      {
        pu.cclmOffsets.cb0 = m_BinDecoder.decodeBin(Ctx::CclmDeltaFlags(1));

#if MMLM
        if ( PU::isMultiModeLM( pu.intraDir[1] ) && !pu.cclmOffsets.cb0 )
        {
          pu.cclmOffsets.cr0 = m_BinDecoder.decodeBin(Ctx::CclmDeltaFlags(2));
        }
        else
#endif
        {
          pu.cclmOffsets.cr0 = !pu.cclmOffsets.cb0;
        }
      }

      cclmDelta( pu, pu.cclmOffsets.cb0 );
      cclmDelta( pu, pu.cclmOffsets.cr0 );
    
#if MMLM
      // Now the same for the second model (if applicable)
      if ( PU::isMultiModeLM( pu.intraDir[1] ) )
      {
        bool bothActive = m_BinDecoder.decodeBin(Ctx::CclmDeltaFlags(3));

        pu.cclmOffsets.cb1 = bothActive;
        pu.cclmOffsets.cr1 = bothActive;
        
        if ( !bothActive )
        {
          pu.cclmOffsets.cb1 = m_BinDecoder.decodeBin(Ctx::CclmDeltaFlags(1));

          if ( pu.cclmOffsets.cb1 )
          {
            pu.cclmOffsets.cr1 = 0;
          }
          else if ( pu.cclmOffsets.cb0 == 0 && pu.cclmOffsets.cr0 == 0 && pu.cclmOffsets.cb1 == 0 )
          {
            pu.cclmOffsets.cr1 = 1;
          }
          else
          {
            pu.cclmOffsets.cr1 = m_BinDecoder.decodeBin(Ctx::CclmDeltaFlags(2));
          }
        }

        cclmDelta( pu, pu.cclmOffsets.cb1 );
        cclmDelta( pu, pu.cclmOffsets.cr1 );
      }
#endif
    }
  }
}
#endif

#if JVET_AA0126_GLM
void CABACReader::glmIdc(PredictionUnit& pu)
{
  if ( PU::hasGlmFlag( pu ) )
  {
    bool glmActive = m_BinDecoder.decodeBin(Ctx::GlmFlags(0));

    if ( glmActive )
    {
#if JVET_AB0092_GLM_WITH_LUMA
      pu.glmIdc.cb0 = 1;
#if NUM_GLM_WEIGHT
      pu.glmIdc.cb0 += m_BinDecoder.decodeBin(Ctx::GlmFlags(1)) ? NUM_GLM_PATTERN : 0;
#endif
#if JVET_AA0057_CCCM
      if (m_BinDecoder.decodeBin(Ctx::GlmFlags(2)))
      {
        pu.glmIdc.cb0 += 1;
        if (m_BinDecoder.decodeBin(Ctx::GlmFlags(3)))
        {
          pu.glmIdc.cb0 += 1;
          if (m_BinDecoder.decodeBin(Ctx::GlmFlags(4)))
          {
            pu.glmIdc.cb0 += 1;
          }
        }
      }
#else
      pu.glmIdc.cb0 += m_BinDecoder.decodeBinsEP(NUM_GLM_PATTERN_BITS);
#endif
      pu.glmIdc.cr0 = pu.glmIdc.cb0;
#else
      bool bothActive = m_BinDecoder.decodeBin(Ctx::GlmFlags(3));

      pu.glmIdc.cb0 = bothActive;
      pu.glmIdc.cr0 = bothActive;
      
      if ( !bothActive )
      {
        pu.glmIdc.cb0 = m_BinDecoder.decodeBin(Ctx::GlmFlags(1));
        pu.glmIdc.cr0 = !pu.glmIdc.cb0;
      }

      if ( pu.glmIdc.cb0 )
      {
#if NUM_GLM_WEIGHT
        pu.glmIdc.cb0 += m_BinDecoder.decodeBin(Ctx::GlmFlags(2)) ? NUM_GLM_PATTERN : 0;
#endif
        pu.glmIdc.cb0 += m_BinDecoder.decodeBinsEP(NUM_GLM_PATTERN_BITS);
      }

      if ( pu.glmIdc.cr0 )
      {
#if NUM_GLM_WEIGHT
        pu.glmIdc.cr0 += m_BinDecoder.decodeBin(Ctx::GlmFlags(4)) ? NUM_GLM_PATTERN : 0;
#endif
        pu.glmIdc.cr0 += m_BinDecoder.decodeBinsEP(NUM_GLM_PATTERN_BITS);
      }
#endif

#if MMLM
      if ( PU::isMultiModeLM( pu.intraDir[1] ) )
      {
        pu.glmIdc.cb1 = pu.glmIdc.cb0;
        pu.glmIdc.cr1 = pu.glmIdc.cr0;
      }
#endif
    }
  }
}
#endif
bool CABACReader::intra_chroma_lmc_mode(PredictionUnit& pu)
{
#if MMLM
  int lmModeList[NUM_CHROMA_MODE];
#else 
  int lmModeList[10];
#endif
  PU::getLMSymbolList(pu, lmModeList);

  int symbol = m_BinDecoder.decodeBin(Ctx::CclmModeIdx(0));

  if (symbol == 0)
  {
    pu.intraDir[1] = lmModeList[symbol];
    CHECK(pu.intraDir[1] != LM_CHROMA_IDX, "should be LM_CHROMA");
  }
  else
  {
#if MMLM // Modifying signalling may get minor gain
    int modeIdx = 1; // MMLM_Chroma
    modeIdx += m_BinDecoder.decodeBin(Ctx::MMLMFlag(0));

    if (modeIdx > 1) // = 2 means MDLM_L
    {
      modeIdx += m_BinDecoder.decodeBinEP();
    }

    if (modeIdx > 2) // == 3 means MDLM_T
    {
      modeIdx += m_BinDecoder.decodeBinEP();
    }
    if (modeIdx > 3) // == 4 MMLM_L
    {
      modeIdx += m_BinDecoder.decodeBinEP();
    } // == 5 mean MMLM_T

    pu.intraDir[1] = lmModeList[modeIdx];
#else
    symbol += m_BinDecoder.decodeBinEP();
    pu.intraDir[1] = lmModeList[symbol];
#endif
  }
  
#if JVET_AA0057_CCCM
  cccmFlag( pu );
#endif
#if JVET_AA0126_GLM
  glmIdc( pu );
#endif
#if JVET_Z0050_CCLM_SLOPE
  cclmDeltaSlope( pu );
#endif

  return true; //it will only enter this function for LMC modes, so always return true ;
}

#if JVET_AA0057_CCCM
void CABACReader::cccmFlag(PredictionUnit& pu)
{
  const unsigned intraDir = pu.intraDir[1];
  
#if JVET_AB0143_CCCM_TS
  bool isCCCMEnabled = false;
  if (intraDir == LM_CHROMA_IDX)
  {
    isCCCMEnabled = PU::cccmSingleModeAvail(pu, LM_CHROMA_IDX);
  }
  else if (intraDir == MDLM_L_IDX)
  {
    isCCCMEnabled = PU::isLeftCccmMode(pu, MDLM_L_IDX);
  }
  else if (intraDir == MDLM_T_IDX)
  {
    isCCCMEnabled = PU::isTopCccmMode(pu, MDLM_T_IDX);
  }
#if MMLM
  else if (intraDir == MMLM_CHROMA_IDX)
  {
    isCCCMEnabled = PU::cccmMultiModeAvail(pu, MMLM_CHROMA_IDX);
  }
  else if (intraDir == MMLM_L_IDX)
  {
    isCCCMEnabled = PU::cccmMultiModeAvail(pu, MMLM_L_IDX);
  }
  else if (intraDir == MMLM_T_IDX)
  {
    isCCCMEnabled = PU::cccmMultiModeAvail(pu, MMLM_T_IDX);
  }
#endif

  if (isCCCMEnabled)
#else
  if ( PU::cccmSingleModeAvail(pu, intraDir) || PU::cccmMultiModeAvail(pu, intraDir) )
#endif
  {
    pu.cccmFlag = m_BinDecoder.decodeBin( Ctx::CccmFlag( 0 ) );
#if JVET_AB0143_CCCM_TS
    if (pu.cccmFlag)
    {
#if MMLM
      pu.cccmFlag = (intraDir == MDLM_T_IDX || intraDir == MMLM_T_IDX) ? 3 : (intraDir == MDLM_L_IDX || intraDir == MMLM_L_IDX) ? 2 : 1;
#else
      pu.cccmFlag = (intraDir == MDLM_T_IDX) ? 3 : (intraDir == MDLM_L_IDX) ? 2 : 1;
#endif
    }
#endif
  }
}
#endif

void CABACReader::intra_chroma_pred_mode(PredictionUnit& pu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__INTRA_DIR_ANG, pu.cu->blocks[pu.chType].lumaSize(), CHANNEL_TYPE_CHROMA);
  if (pu.cu->colorTransform)
  {
    pu.intraDir[CHANNEL_TYPE_CHROMA] = DM_CHROMA_IDX;
    return;
  }

  // LM chroma mode
#if CCLM_LATENCY_RESTRICTION_RMV
  if (pu.cs->sps->getUseLMChroma() )
#else
  if (pu.cs->sps->getUseLMChroma() && pu.cu->checkCCLMAllowed())
#endif  
  {
    bool isLMCMode = m_BinDecoder.decodeBin(Ctx::CclmModeFlag(0)) ? true : false;
    if (isLMCMode)
    {
      intra_chroma_lmc_mode(pu);
      return;
    }
  }

  if (m_BinDecoder.decodeBin(Ctx::IntraChromaPredMode(0)) == 0)
  {
    pu.intraDir[1] = DM_CHROMA_IDX;
#if JVET_Z0050_DIMD_CHROMA_FUSION
    if (PU::hasChromaFusionFlag(pu, pu.intraDir[1]))
    {
      if (m_BinDecoder.decodeBin(Ctx::ChromaFusionMode()) == 1)
      {
        pu.isChromaFusion = true;
      }
    }
#endif
    return;
  }

#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
  if (pu.cu->slice->getSPS()->getUseDimd())
  {
    if (m_BinDecoder.decodeBin(Ctx::DimdChromaMode()) == 0)
    {
      pu.intraDir[1] = DIMD_CHROMA_IDX;
      if (PU::hasChromaFusionFlag(pu, pu.intraDir[1]))
      {
        if (m_BinDecoder.decodeBin(Ctx::ChromaFusionMode()) == 1)
        {
          pu.isChromaFusion = true;
        }
      }
      return;
    }
  }
#endif

#if ENABLE_DIMD || JVET_W0123_TIMD_FUSION
  pu.parseChromaMode = true;
#endif
  unsigned candId = m_BinDecoder.decodeBinsEP(2);
#if JVET_Z0050_DIMD_CHROMA_FUSION
  if (PU::hasChromaFusionFlag(pu, pu.intraDir[1]))
  {
    if (m_BinDecoder.decodeBin(Ctx::ChromaFusionMode()) == 1)
    {
      pu.isChromaFusion = true;
    }
  }
#endif

  unsigned chromaCandModes[NUM_CHROMA_MODE];
  PU::getIntraChromaCandModes(pu, chromaCandModes);

  CHECK(candId >= NUM_CHROMA_MODE, "Chroma prediction mode index out of bounds");
  CHECK(PU::isLMCMode(chromaCandModes[candId]), "The intra dir cannot be LM_CHROMA for this path");
  CHECK(chromaCandModes[candId] == DM_CHROMA_IDX, "The intra dir cannot be DM_CHROMA for this path");
#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
  CHECK(chromaCandModes[candId] == DIMD_CHROMA_IDX, "The intra dir cannot be DIMD_CHROMA for this path");
#endif

  pu.intraDir[1] = chromaCandModes[candId];
#if ENABLE_DIMD || JVET_W0123_TIMD_FUSION
  pu.candId = candId;
#endif
}

void CABACReader::cu_residual( CodingUnit& cu, Partitioner &partitioner, CUCtx& cuCtx )
{
  if (!CU::isIntra(cu))
  {
    PredictionUnit& pu = *cu.firstPU;
#if MULTI_HYP_PRED
    if (!(pu.mergeFlag && pu.numMergedAddHyps == pu.addHypData.size()))
#else
    if( !pu.mergeFlag )
#endif
    {
      rqt_root_cbf( cu );
    }
    else
    {
      cu.rootCbf = true;
    }
    if( cu.rootCbf )
    {
      sbt_mode( cu );
    }
    if( !cu.rootCbf )
    {
      cu.colorTransform = false;
      cu.cs->addEmptyTUs( partitioner );
      return;
    }
  }

  if (CU::isInter(cu) || CU::isIBC(cu))
  {
    adaptive_color_transform(cu);
  }

  cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_LUMA]   = false;
  cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_CHROMA] = false;
  cuCtx.lfnstLastScanPos                              = false;
  cuCtx.violatesMtsCoeffConstraint                    = false;
  cuCtx.mtsLastScanPos                                = false;
#if JVET_Y0142_ADAPT_INTRA_MTS
  cuCtx.mtsCoeffAbsSum                                = 0;
#endif
  ChromaCbfs chromaCbfs;
  if( cu.ispMode && isLuma( partitioner.chType ) )
  {
    TUIntraSubPartitioner subTuPartitioner( partitioner );
    transform_tree( *cu.cs, subTuPartitioner, cuCtx, CU::getISPType(cu, getFirstComponentOfChannel(partitioner.chType)), 0 );
  }
  else
  {
    transform_tree( *cu.cs, partitioner, cuCtx             );
  }

  residual_lfnst_mode( cu, cuCtx );
  mts_idx            ( cu, cuCtx );
#if SIGN_PREDICTION
  for( auto &currTU : CU::traverseTUs( cu ) )
  {
    for( int compIdx = COMPONENT_Y; compIdx < MAX_NUM_COMPONENT; ++compIdx)
    {
      ComponentID compID = (ComponentID)compIdx;
      if(compIdx >= currTU.blocks.size())
      {
        continue;
      }

      if(currTU.jointCbCr)
      {
        if( !( currTU.jointCbCr >> 1 ) && compID == COMPONENT_Cb )
        {
          continue;
        }
        if( ( currTU.jointCbCr >> 1 ) && compID == COMPONENT_Cr )
        {
          continue;
        }
      }

      if(currTU.blocks[compID].valid() && TU::getCbf( currTU, compID ) && TU::getDelayedSignCoding(currTU, compID))
      {
        parsePredictedSigns(const_cast<TransformUnit &>(currTU), compID);
      }
    }
  }
#endif
}

void CABACReader::rqt_root_cbf( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__QT_ROOT_CBF, cu.lumaSize());

  cu.rootCbf = ( m_BinDecoder.decodeBin( Ctx::QtRootCbf() ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "rqt_root_cbf() ctx=0 root_cbf=%d pos=(%d,%d)\n", cu.rootCbf ? 1 : 0, cu.lumaPos().x, cu.lumaPos().y );
}

void CABACReader::adaptive_color_transform(CodingUnit& cu)
{
  if (!cu.slice->getSPS()->getUseColorTrans())
  {
    return;
  }
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (CS::isDualITree(*cu.cs))
#else
  if (cu.isSepTree())
#endif
  {
    return;
  }

  if (CU::isInter(cu) || CU::isIBC(cu) || CU::isIntra(cu))
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__ACT, cu.lumaSize());
    cu.colorTransform = (m_BinDecoder.decodeBin(Ctx::ACTFlag()));
  }
}

void CABACReader::sbt_mode( CodingUnit& cu )
{
  const uint8_t sbtAllowed = cu.checkAllowedSbt();
  if( !sbtAllowed )
  {
    return;
  }

  SizeType cuWidth = cu.lwidth();
  SizeType cuHeight = cu.lheight();

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__SBT_MODE, cu.lumaSize());
  //bin - flag
  uint8_t ctxIdx = ( cuWidth * cuHeight <= 256 ) ? 1 : 0;
  bool sbtFlag = m_BinDecoder.decodeBin( Ctx::SbtFlag( ctxIdx ) );
  if( !sbtFlag )
  {
    return;
  }

  uint8_t sbtVerHalfAllow = CU::targetSbtAllowed( SBT_VER_HALF, sbtAllowed );
  uint8_t sbtHorHalfAllow = CU::targetSbtAllowed( SBT_HOR_HALF, sbtAllowed );
  uint8_t sbtVerQuadAllow = CU::targetSbtAllowed( SBT_VER_QUAD, sbtAllowed );
  uint8_t sbtHorQuadAllow = CU::targetSbtAllowed( SBT_HOR_QUAD, sbtAllowed );

  //bin - type
  bool sbtQuadFlag = false;
  if( ( sbtHorHalfAllow || sbtVerHalfAllow ) && ( sbtHorQuadAllow || sbtVerQuadAllow ) )
  {
    sbtQuadFlag = m_BinDecoder.decodeBin( Ctx::SbtQuadFlag( 0 ) );
  }
  else
  {
    sbtQuadFlag = 0;
  }

  //bin - dir
  bool sbtHorFlag = false;
  if( ( sbtQuadFlag && sbtVerQuadAllow && sbtHorQuadAllow ) || ( !sbtQuadFlag && sbtVerHalfAllow && sbtHorHalfAllow ) ) //both direction allowed
  {
    uint8_t ctxIdx = ( cuWidth == cuHeight ) ? 0 : ( cuWidth < cuHeight ? 1 : 2 );
    sbtHorFlag = m_BinDecoder.decodeBin( Ctx::SbtHorFlag( ctxIdx ) );
  }
  else
  {
    sbtHorFlag = ( sbtQuadFlag && sbtHorQuadAllow ) || ( !sbtQuadFlag && sbtHorHalfAllow );
  }
  cu.setSbtIdx( sbtHorFlag ? ( sbtQuadFlag ? SBT_HOR_QUAD : SBT_HOR_HALF ) : ( sbtQuadFlag ? SBT_VER_QUAD : SBT_VER_HALF ) );

  //bin - pos
  bool sbtPosFlag = m_BinDecoder.decodeBin( Ctx::SbtPosFlag( 0 ) );
  cu.setSbtPos( sbtPosFlag ? SBT_POS1 : SBT_POS0 );

  DTRACE( g_trace_ctx, D_SYNTAX, "sbt_mode() pos=(%d,%d) sbtInfo=%d\n", cu.lx(), cu.ly(), (int)cu.sbtInfo );
}


void CABACReader::end_of_ctu( CodingUnit& cu, CUCtx& cuCtx )
{
  const Position rbPos = recalcPosition( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].bottomRight().offset( 1, 1 ) );

  if (((rbPos.x & cu.cs->pcv->maxCUWidthMask) == 0 || rbPos.x == cu.cs->pps->getPicWidthInLumaSamples())
      && ((rbPos.y & cu.cs->pcv->maxCUHeightMask) == 0 || rbPos.y == cu.cs->pps->getPicHeightInLumaSamples())
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS      
      && (!CS::isDualITree(*cu.cs) || cu.chromaFormat == CHROMA_400 || isChroma(cu.chType)))
#else
      && (!cu.isSepTree() || cu.chromaFormat == CHROMA_400 || isChroma(cu.chType)))
#endif
  {
    cuCtx.isDQPCoded = ( cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded );
  }
}

void CABACReader::cu_palette_info(CodingUnit& cu, ComponentID compBegin, uint32_t numComp, CUCtx& cuCtx)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__PLT_MODE, cu.lumaSize());

  const SPS&      sps = *(cu.cs->sps);
  TransformUnit&   tu = *cu.firstTU;
  int curPLTidx = 0;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( cu.isLocalSepTree() )
    cu.cs->prevPLT.curPLTSize[compBegin] = cu.cs->prevPLT.curPLTSize[COMPONENT_Y];
#endif
  cu.lastPLTSize[compBegin] = cu.cs->prevPLT.curPLTSize[compBegin];
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  int maxPltSize = CS::isDualITree(*cu.cs) ? MAXPLTSIZE_DUALTREE : MAXPLTSIZE;
#else
  int maxPltSize = cu.isSepTree() ? MAXPLTSIZE_DUALTREE : MAXPLTSIZE;
#endif
  if (cu.lastPLTSize[compBegin])
  {
    xDecodePLTPredIndicator(cu, maxPltSize, compBegin);
  }

  for (int idx = 0; idx < cu.lastPLTSize[compBegin]; idx++)
  {
    if (cu.reuseflag[compBegin][idx])
    {
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if( cu.isLocalSepTree() )
      {
        for( int comp = COMPONENT_Y; comp < MAX_NUM_COMPONENT; comp++ )
        {
          cu.curPLT[comp][curPLTidx] = cu.cs->prevPLT.curPLT[comp][idx];
        }
      }
      else
      {
#endif
        for (int comp = compBegin; comp < (compBegin + numComp); comp++)
        {
          cu.curPLT[comp][curPLTidx] = cu.cs->prevPLT.curPLT[comp][idx];
        }
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      }
#endif
      curPLTidx++;
    }
  }

  uint32_t recievedPLTnum = 0;
  if (curPLTidx < maxPltSize)
  {
    recievedPLTnum = exp_golomb_eqprob(0);
  }

  cu.curPLTSize[compBegin] = curPLTidx + recievedPLTnum;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( cu.isLocalSepTree() )
    cu.curPLTSize[COMPONENT_Y] = cu.curPLTSize[compBegin];
#endif
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    for (int idx = curPLTidx; idx < cu.curPLTSize[compBegin]; idx++)
    {
      ComponentID compID = (ComponentID)comp;
      const int  channelBitDepth = sps.getBitDepth(toChannelType(compID));
      cu.curPLT[compID][idx] = m_BinDecoder.decodeBinsEP(channelBitDepth);
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if( cu.isLocalSepTree() )
      {
        if( isLuma( cu.chType ) )
        {
          cu.curPLT[COMPONENT_Cb][idx] = 1 << (cu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA) - 1);
          cu.curPLT[COMPONENT_Cr][idx] = 1 << (cu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA) - 1);
        }
        else
        {
          cu.curPLT[COMPONENT_Y][idx] = 1 << (cu.cs->sps->getBitDepth(CHANNEL_TYPE_LUMA) - 1);
        }
      }
#endif
    }
  }
  cu.useEscape[compBegin] = true;
  if (cu.curPLTSize[compBegin] > 0)
  {
    uint32_t escCode = 0;
    escCode = m_BinDecoder.decodeBinEP();
    cu.useEscape[compBegin] = (escCode != 0);
  }
  uint32_t    indexMaxSize = cu.useEscape[compBegin] ? (cu.curPLTSize[compBegin] + 1) : cu.curPLTSize[compBegin];
  //encode index map
  uint32_t    height = cu.block(compBegin).height;
  uint32_t    width = cu.block(compBegin).width;

  uint32_t total = height * width;
  if (indexMaxSize > 1)
  {
    parseScanRotationModeFlag(cu, compBegin);
  }
  else
  {
    cu.useRotation[compBegin] = false;
  }

  if (cu.useEscape[compBegin] && cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded)
  {
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (!CS::isDualITree(*tu.cs) || isLuma(tu.chType))
#else
    if (!cu.isSepTree() || isLuma(tu.chType))
#endif
    {
      cu_qp_delta(cu, cuCtx.qp, cu.qp);
      cuCtx.qp = cu.qp;
      cuCtx.isDQPCoded = true;
    }
  }
  if (cu.useEscape[compBegin] && cu.cs->slice->getUseChromaQpAdj() && !cuCtx.isChromaQpAdjCoded)
  {
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    if (!CS::isDualITree(*tu.cs) || isChroma(tu.chType))
#else
    if (!cu.isSepTree() || isChroma(tu.chType))
#endif
    {
      cu_chroma_qp_offset(cu);
      cuCtx.isChromaQpAdjCoded = true;
    }
  }

  m_scanOrder = g_scanOrder[SCAN_UNGROUPED][(cu.useRotation[compBegin]) ? SCAN_TRAV_VER : SCAN_TRAV_HOR][gp_sizeIdxInfo->idxFrom(width)][gp_sizeIdxInfo->idxFrom(height)];
  uint32_t prevRunPos = 0;
  unsigned prevRunType = 0;
  for (int subSetId = 0; subSetId <= (total - 1) >> LOG2_PALETTE_CG_SIZE; subSetId++)
  {
    cuPaletteSubblockInfo(cu, compBegin, numComp, subSetId, prevRunPos, prevRunType);
  }
  CHECK(cu.curPLTSize[compBegin] > maxPltSize, " Current palette size is larger than maximum palette size");
}

void CABACReader::cuPaletteSubblockInfo(CodingUnit& cu, ComponentID compBegin, uint32_t numComp, int subSetId, uint32_t& prevRunPos, unsigned& prevRunType)
{
  const SPS&      sps = *(cu.cs->sps);
  TransformUnit&  tu = *cu.firstTU;
  PLTtypeBuf      runType = tu.getrunType(compBegin);
  PelBuf          curPLTIdx = tu.getcurPLTIdx(compBegin);
  uint32_t        indexMaxSize = cu.useEscape[compBegin] ? (cu.curPLTSize[compBegin] + 1) : cu.curPLTSize[compBegin];
  uint32_t        totalPel = cu.block(compBegin).height*cu.block(compBegin).width;

  int minSubPos = subSetId << LOG2_PALETTE_CG_SIZE;
  int maxSubPos = minSubPos + (1 << LOG2_PALETTE_CG_SIZE);
  maxSubPos = (maxSubPos > totalPel) ? totalPel : maxSubPos; // if last position is out of the current CU size

  unsigned runCopyFlag[(1 << LOG2_PALETTE_CG_SIZE)];
  for (int i = 0; i < (1 << LOG2_PALETTE_CG_SIZE); i++)
  {
    runCopyFlag[i] = MAX_INT;
  }
  if (minSubPos == 0)
  {
    runCopyFlag[0] = 0;
  }

// PLT runCopy flag and runType - context coded
  int curPos = minSubPos;
  for (; curPos < maxSubPos && indexMaxSize > 1; curPos++)
  {
    uint32_t posy = m_scanOrder[curPos].y;
    uint32_t posx = m_scanOrder[curPos].x;
    uint32_t posyprev = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].y;
    uint32_t posxprev = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].x;
    unsigned identityFlag = 1;

    const CtxSet&   ctxSet = (prevRunType == PLT_RUN_INDEX) ? Ctx::IdxRunModel : Ctx::CopyRunModel;
    if (curPos > 0)
    {
      int dist = curPos - prevRunPos - 1;
      const unsigned  ctxId = DeriveCtx::CtxPltCopyFlag(prevRunType, dist);
      identityFlag = m_BinDecoder.decodeBin( ctxSet( ctxId ) );
      DTRACE(g_trace_ctx, D_SYNTAX, "plt_copy_flag() bin=%d ctx=%d\n", identityFlag, ctxId);
      runCopyFlag[curPos - minSubPos] = identityFlag;
    }

    if ( identityFlag == 0 || curPos == 0 )
    {
      if (((posy == 0) && !cu.useRotation[compBegin]) || ((posx == 0) && cu.useRotation[compBegin]))
      {
        runType.at(posx, posy) = PLT_RUN_INDEX;
      }
      else if (curPos != 0 && runType.at(posxprev, posyprev) == PLT_RUN_COPY)
      {
        runType.at(posx, posy) = PLT_RUN_INDEX;
      }
      else
      {
        runType.at(posx, posy) = (m_BinDecoder.decodeBin(Ctx::RunTypeFlag()));
      }
      DTRACE(g_trace_ctx, D_SYNTAX, "plt_type_flag() bin=%d sp=%d\n", runType.at(posx, posy), curPos);
      prevRunType = runType.at(posx, posy);
      prevRunPos  = curPos;
    }
    else //assign run information
    {
      runType.at(posx, posy) = runType.at(posxprev, posyprev);
    }
  }

// PLT index values - bypass coded
  uint32_t adjust;
  uint32_t symbol = 0;
  curPos = minSubPos;
  if (indexMaxSize > 1)
  {
    for (; curPos < maxSubPos; curPos++)
    {
      if (curPos > 0)
      {
        adjust = 1;
      }
      else
      {
        adjust = 0;
      }

      uint32_t posy = m_scanOrder[curPos].y;
      uint32_t posx = m_scanOrder[curPos].x;
      uint32_t posyprev = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].y;
      uint32_t posxprev = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].x;
      if ( runCopyFlag[curPos - minSubPos] == 0 && runType.at(posx, posy) == PLT_RUN_INDEX )
      {
        xReadTruncBinCode(symbol, indexMaxSize - adjust);
        xAdjustPLTIndex(cu, symbol, curPos, curPLTIdx, runType, indexMaxSize, compBegin);
        DTRACE(g_trace_ctx, D_SYNTAX, "plt_idx_idc() value=%d sp=%d\n", curPLTIdx.at(posx, posy), curPos);
      }
      else if (runType.at(posx, posy) == PLT_RUN_INDEX)
      {
        curPLTIdx.at(posx, posy) = curPLTIdx.at(posxprev, posyprev);
      }
      else
      {
        curPLTIdx.at(posx, posy) = (cu.useRotation[compBegin]) ? curPLTIdx.at(posx - 1, posy) : curPLTIdx.at(posx, posy - 1);
      }
    }
  }
  else
  {
    for (; curPos < maxSubPos; curPos++)
    {
      uint32_t posy = m_scanOrder[curPos].y;
      uint32_t posx = m_scanOrder[curPos].x;
      uint32_t posyprev = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].y;
      uint32_t posxprev = (curPos == 0) ? 0 : m_scanOrder[curPos - 1].x;
      runType.at(posx, posy) = PLT_RUN_INDEX;
      if (runCopyFlag[curPos - minSubPos] == 0 && runType.at(posx, posy) == PLT_RUN_INDEX)
      {
        curPLTIdx.at(posx, posy) = 0;
      }
      else
      {
        curPLTIdx.at(posx, posy) = curPLTIdx.at(posxprev, posyprev);
      }
    }
  }

// Quantized escape colors - bypass coded
  uint32_t scaleX = getComponentScaleX(COMPONENT_Cb, sps.getChromaFormatIdc());
  uint32_t scaleY = getComponentScaleY(COMPONENT_Cb, sps.getChromaFormatIdc());
  for (int comp = compBegin; comp < (compBegin + numComp); comp++)
  {
    ComponentID compID = (ComponentID)comp;
    for (curPos = minSubPos; curPos < maxSubPos; curPos++)
    {
      uint32_t posy = m_scanOrder[curPos].y;
      uint32_t posx = m_scanOrder[curPos].x;
      if (curPLTIdx.at(posx, posy) == cu.curPLTSize[compBegin])
      {
          PLTescapeBuf    escapeValue = tu.getescapeValue((ComponentID)comp);
          if (compID == COMPONENT_Y || compBegin != COMPONENT_Y)
          {
            escapeValue.at(posx, posy) = exp_golomb_eqprob(5);
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
            assert(escapeValue.at(posx, posy) < (TCoeff(1) << (cu.cs->sps->getBitDepth(toChannelType((ComponentID)comp)) + 1)));
#else
            assert(escapeValue.at(posx, posy) < (1 << (cu.cs->sps->getBitDepth(toChannelType((ComponentID)comp)) + 1)));
#endif
            DTRACE(g_trace_ctx, D_SYNTAX, "plt_escape_val() value=%d etype=%d sp=%d\n", escapeValue.at(posx, posy), comp, curPos);
          }
          if (compBegin == COMPONENT_Y && compID != COMPONENT_Y && posy % (1 << scaleY) == 0 && posx % (1 << scaleX) == 0)
          {
            uint32_t posxC = posx >> scaleX;
            uint32_t posyC = posy >> scaleY;
            escapeValue.at(posxC, posyC) = exp_golomb_eqprob(5);
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
            assert(escapeValue.at(posxC, posyC) < (TCoeff(1) << (cu.cs->sps->getBitDepth(toChannelType(compID)) + 1)));
#else
            assert(escapeValue.at(posxC, posyC) < (1 << (cu.cs->sps->getBitDepth(toChannelType(compID)) + 1)));
#endif
            DTRACE(g_trace_ctx, D_SYNTAX, "plt_escape_val() value=%d etype=%d sp=%d\n", escapeValue.at(posx, posy), comp, curPos);
          }
      }
    }
  }
}

void CABACReader::parseScanRotationModeFlag(CodingUnit& cu, ComponentID compBegin)
{
  cu.useRotation[compBegin] = m_BinDecoder.decodeBin(Ctx::RotationFlag());
}

void CABACReader::xDecodePLTPredIndicator(CodingUnit& cu, uint32_t maxPLTSize, ComponentID compBegin)
{
  uint32_t symbol, numPltPredicted = 0, idx = 0;

  symbol = exp_golomb_eqprob(0);

  if (symbol != 1)
  {
    while (idx < cu.lastPLTSize[compBegin] && numPltPredicted < maxPLTSize)
    {
      if (idx > 0)
      {
        symbol = exp_golomb_eqprob(0);
      }
      if (symbol == 1)
      {
        break;
      }

      if (symbol)
      {
        idx += symbol - 1;
      }
      cu.reuseflag[compBegin][idx] = 1;
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
      if( cu.isLocalSepTree() )
      {
        cu.reuseflag[COMPONENT_Y][idx] = 1;
      }
#endif
      numPltPredicted++;
      idx++;
    }
  }
}
void CABACReader::xAdjustPLTIndex(CodingUnit& cu, Pel curLevel, uint32_t idx, PelBuf& paletteIdx, PLTtypeBuf& paletteRunType, int maxSymbol, ComponentID compBegin)
{
  uint32_t symbol;
  int refLevel = MAX_INT;
  uint32_t posy = m_scanOrder[idx].y;
  uint32_t posx = m_scanOrder[idx].x;
  if (idx)
  {
    uint32_t prevposy = m_scanOrder[idx - 1].y;
    uint32_t prevposx = m_scanOrder[idx - 1].x;
    if (paletteRunType.at(prevposx, prevposy) == PLT_RUN_INDEX)
    {
      refLevel = paletteIdx.at(prevposx, prevposy);
      if (paletteIdx.at(prevposx, prevposy) == cu.curPLTSize[compBegin]) // escape
      {
        refLevel = maxSymbol - 1;
      }
    }
    else
    {
      if (cu.useRotation[compBegin])
      {
        assert(prevposx > 0);
        refLevel = paletteIdx.at(posx - 1, posy);
        if (paletteIdx.at(posx - 1, posy) == cu.curPLTSize[compBegin]) // escape mode
        {
          refLevel = maxSymbol - 1;
        }
      }
      else
      {
        assert(prevposy > 0);
        refLevel = paletteIdx.at(posx, posy - 1);
        if (paletteIdx.at(posx, posy - 1) == cu.curPLTSize[compBegin]) // escape mode
        {
          refLevel = maxSymbol - 1;
        }
      }
    }
    maxSymbol--;
  }
  symbol = curLevel;
  if (curLevel >= refLevel) // include escape mode
  {
    symbol++;
  }
  paletteIdx.at(posx, posy) = symbol;
}

//================================================================================
//  clause 7.3.8.6
//--------------------------------------------------------------------------------
//    void  prediction_unit ( pu, mrgCtx );
//    void  merge_flag      ( pu );
//    void  merge_data      ( pu, mrgCtx );
//    void  merge_idx       ( pu );
//    void  inter_pred_idc  ( pu );
//    void  ref_idx         ( pu, refList );
//    void  mvp_flag        ( pu, refList );
//================================================================================

void CABACReader::prediction_unit( PredictionUnit& pu, MergeCtx& mrgCtx )
{
  if( pu.cu->skip )
  {
    pu.mergeFlag = true;
  }
  else
  {
    merge_flag( pu );
  }

#if JVET_AA0070_RRIBC
  rribcData(*pu.cu);
#endif
#if JVET_AC0112_IBC_LIC
  cuIbcLicFlag(*pu.cu);
#endif

  if( pu.mergeFlag )
  {
    merge_data(pu);
#if MULTI_HYP_PRED
    if( !pu.cu->skip && !CU::isIBC( *pu.cu ) )
    {
      CHECK(pu.cu->imv != 0, "Multi Hyp: pu.cu->imv != 0");
      mh_pred_data(pu);
    }
#endif
  }
  else if (CU::isIBC(*pu.cu))
  {
#if JVET_AC0112_IBC_CIIP
    ibcCiipFlag(pu);
    if (pu.ibcCiipFlag)
    {
      ibcCiipIntraIdx(pu);
    }
#endif
    pu.interDir = 1;
    pu.cu->affine = false;
    pu.refIdx[REF_PIC_LIST_0] = MAX_NUM_REF;
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MVD, pu.lumaSize());
#if JVET_AA0070_RRIBC
#if JVET_Z0131_IBC_BVD_BINARIZATION
    bvdCoding(pu.mvd[REF_PIC_LIST_0], pu.cu->rribcFlipType);
#else
    mvd_coding(pu.mvd[REF_PIC_LIST_0], true, pu.cu->rribcFlipType);
#endif
#else
#if JVET_Z0131_IBC_BVD_BINARIZATION
    bvdCoding(pu.mvd[REF_PIC_LIST_0]);
#else
    mvd_coding(pu.mvd[REF_PIC_LIST_0]);
#endif
#endif
    if (pu.cs->sps->getMaxNumIBCMergeCand() == 1)
    {
      pu.mvpIdx[REF_PIC_LIST_0] = 0;
    }
    else
    {
      mvp_flag(pu, REF_PIC_LIST_0);
    }
  }
  else
  {
#if JVET_X0083_BM_AMVP_MERGE_MODE
    amvpMerge_mode( pu );
    if (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])
    {
      pu.cu->affine = 0;
      pu.cu->smvdMode = 0;
      CHECK(pu.interDir != 3, "this is not possible");
      CHECK(CU::isIBC(*pu.cu) != 0, "this is not possible");
    }
    else
    {
#endif
    inter_pred_idc( pu );
    affine_flag   ( *pu.cu );
    smvd_mode( pu );
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    if(pu.cs->sps->getUseMVSD())
    {
#endif
    cu_bcw_flag(*pu.cu);
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    }
#endif
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER
    refPairIdx(pu);
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
    }
#endif

#if JVET_Z0054_BLK_REF_PIC_REORDER
    refIdxLC(pu);
#endif
    if( pu.interDir != 2 /* PRED_L1 */ )
    {
#if JVET_X0083_BM_AMVP_MERGE_MODE
      if (!pu.amvpMergeModeFlag[REF_PIC_LIST_0])
      {
#endif
      ref_idx     ( pu, REF_PIC_LIST_0 );
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
      mvp_flag    ( pu, REF_PIC_LIST_0 );
#endif
      if( pu.cu->affine )
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MVD, pu.lumaSize());
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
        mvd_coding(pu.mvdAffi[REF_PIC_LIST_0][0],  !pu.isMvsdApplicable());
#else
        mvd_coding(pu.mvdAffi[REF_PIC_LIST_0][0]);
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
        mvd_coding(pu.mvdAffi[REF_PIC_LIST_0][1],  !pu.isMvsdApplicable());
#else
        mvd_coding(pu.mvdAffi[REF_PIC_LIST_0][1]);
#endif
        if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
        {
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
          mvd_coding(pu.mvdAffi[REF_PIC_LIST_0][2],  !pu.isMvsdApplicable());
#else
          mvd_coding(pu.mvdAffi[REF_PIC_LIST_0][2]);
#endif
        }
      }
      else
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MVD, pu.lumaSize());
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
        if (pu.amvpMergeModeFlag[REF_PIC_LIST_1] == true && pu.mvpIdx[REF_PIC_LIST_0] < 2)
        {
          pu.mvd[REF_PIC_LIST_0].setZero();
        }
        else
        {
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
        mvd_coding(pu.mvd[REF_PIC_LIST_0] ,  !pu.isMvsdApplicable());
#else
        mvd_coding(pu.mvd[REF_PIC_LIST_0] );
#endif
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
        }
#endif
      }
#if JVET_X0083_BM_AMVP_MERGE_MODE
      }
#endif
#if !JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
      mvp_flag    ( pu, REF_PIC_LIST_0 );
#endif
    }

    if( pu.interDir != 1 /* PRED_L0 */ )
    {
      if ( pu.cu->smvdMode != 1 )
      {
#if JVET_X0083_BM_AMVP_MERGE_MODE
        if (!pu.amvpMergeModeFlag[REF_PIC_LIST_1])
        {
#endif
        ref_idx(pu, REF_PIC_LIST_1);
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
        mvp_flag    ( pu, REF_PIC_LIST_1 );
#endif
        if (pu.cu->cs->picHeader->getMvdL1ZeroFlag() && pu.interDir == 3 /* PRED_BI */)
        {
          pu.mvd[REF_PIC_LIST_1]        = Mv();
          pu.mvdAffi[REF_PIC_LIST_1][0] = Mv();
          pu.mvdAffi[REF_PIC_LIST_1][1] = Mv();
          pu.mvdAffi[REF_PIC_LIST_1][2] = Mv();
        }
        else if (pu.cu->affine)
        {
          RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MVD, pu.lumaSize());
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
          mvd_coding(pu.mvdAffi[REF_PIC_LIST_1][0],  !pu.isMvsdApplicable());
#else
          mvd_coding(pu.mvdAffi[REF_PIC_LIST_1][0]);
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
          mvd_coding(pu.mvdAffi[REF_PIC_LIST_1][1],  !pu.isMvsdApplicable());
#else
          mvd_coding(pu.mvdAffi[REF_PIC_LIST_1][1]);
#endif
          if (pu.cu->affineType == AFFINEMODEL_6PARAM)
          {
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
            mvd_coding(pu.mvdAffi[REF_PIC_LIST_1][2],  !pu.isMvsdApplicable());
#else
            mvd_coding(pu.mvdAffi[REF_PIC_LIST_1][2]);
#endif
          }
        }
        else
        {
          RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MVD, pu.lumaSize());
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
          if (pu.amvpMergeModeFlag[REF_PIC_LIST_0] == true && pu.mvpIdx[REF_PIC_LIST_1] < 2)
          {
            pu.mvd[REF_PIC_LIST_1].setZero();
          }
          else
          {
#endif
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
            mvd_coding(pu.mvd[REF_PIC_LIST_1],  !pu.isMvsdApplicable());
#else
          mvd_coding(pu.mvd[REF_PIC_LIST_1]);
#endif
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
          }
#endif
        }
#if JVET_X0083_BM_AMVP_MERGE_MODE
        }
#endif
      }
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
      else
      {
        pu.refIdx[REF_PIC_LIST_1] = pu.cs->slice->getSymRefIdx(REF_PIC_LIST_1);
        mvp_flag    ( pu, REF_PIC_LIST_1 );
      }
#endif
#if !JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
      mvp_flag    ( pu, REF_PIC_LIST_1 );
#endif
    }
  }
#if MULTI_HYP_PRED
  CHECK(PU::isBipredRestriction(pu) && !pu.addHypData.empty(), "additional hypotheseis signalled in restricted bi-pred mode");
#endif
  if( pu.interDir == 3 /* PRED_BI */ && PU::isBipredRestriction(pu) )
  {
    pu.mv    [REF_PIC_LIST_1] = Mv(0, 0);
    pu.refIdx[REF_PIC_LIST_1] = -1;
    pu.interDir               =  1;
    pu.cu->BcwIdx = BCW_DEFAULT;
  }

  if ( pu.cu->smvdMode )
  {
    RefPicList eCurRefList = (RefPicList)(pu.cu->smvdMode - 1);
    pu.mvd[1 - eCurRefList].set( -pu.mvd[eCurRefList].hor, -pu.mvd[eCurRefList].ver );
    CHECK(!((pu.mvd[1 - eCurRefList].getHor() >= MVD_MIN) && (pu.mvd[1 - eCurRefList].getHor() <= MVD_MAX)) || !((pu.mvd[1 - eCurRefList].getVer() >= MVD_MIN) && (pu.mvd[1 - eCurRefList].getVer() <= MVD_MAX)), "Illegal MVD value");
    pu.refIdx[1 - eCurRefList] = pu.cs->slice->getSymRefIdx( 1 - eCurRefList );
  }
}
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
void CABACReader::mvsd_data(PredictionUnit&  pu)
{
  CHECK(pu.cu->slice->getSliceType() == I_SLICE && !CU::isIBC(*pu.cu), "cannot be I Slice");
  if (CU::isIBC(*pu.cu))
  {
    return;
  }
  if (pu.cu->skip || pu.mergeFlag || CU::isIBC(*pu.cu) || !pu.isMvsdApplicable())
  {
    return;
  }
  if (pu.interDir != 2 /* PRED_L1 */)
  {
    if (pu.cu->affine)
    {
      mvsdAffineIdxFunc(pu, REF_PIC_LIST_0);
    }
    else
    {
      mvsdIdxFunc(pu, REF_PIC_LIST_0);
    }
  }
  if (pu.interDir != 1 /* PRED_L0 */ && pu.cu->smvdMode != 1)
  {
    if (pu.cu->affine)
    {
      mvsdAffineIdxFunc(pu, REF_PIC_LIST_1);
    }
    else
    {
      mvsdIdxFunc(pu, REF_PIC_LIST_1);
    }
  }
}
#endif
void CABACReader::smvd_mode( PredictionUnit& pu )
{
  pu.cu->smvdMode = 0;
  if ( pu.interDir != 3 || pu.cu->affine )
  {
    return;
  }

  if ( pu.cs->slice->getBiDirPred() == false )
  {
    return;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__SYMMVD_FLAG, pu.lumaSize());

  pu.cu->smvdMode = m_BinDecoder.decodeBin( Ctx::SmvdFlag() ) ? 1 : 0;

  DTRACE( g_trace_ctx, D_SYNTAX, "symmvd_flag() symmvd=%d pos=(%d,%d) size=%dx%d\n", pu.cu->smvdMode ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height );
}

void CABACReader::subblock_merge_flag( CodingUnit& cu )
{
  cu.affine = false;

  if ( !cu.cs->slice->isIntra() && (cu.slice->getPicHeader()->getMaxNumAffineMergeCand() > 0) && cu.lumaSize().width >= 8 && cu.lumaSize().height >= 8 )
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__AFFINE_FLAG, cu.lumaSize());

    unsigned ctxId = DeriveCtx::CtxAffineFlag( cu );
    cu.affine = m_BinDecoder.decodeBin( Ctx::SubblockMergeFlag( ctxId ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "subblock_merge_flag() subblock_merge_flag=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
  }
}

void CABACReader::affine_flag( CodingUnit& cu )
{
#if INTER_RM_SIZE_CONSTRAINTS
  if (!cu.cs->slice->isIntra() && cu.cs->sps->getUseAffine() && cu.lumaSize().width >= 8 && cu.lumaSize().height >= 8)
#else
  if ( !cu.cs->slice->isIntra() && cu.cs->sps->getUseAffine() && cu.lumaSize().width > 8 && cu.lumaSize().height > 8 )
#endif
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__AFFINE_FLAG, cu.lumaSize());

    unsigned ctxId = DeriveCtx::CtxAffineFlag( cu );
    cu.affine = m_BinDecoder.decodeBin( Ctx::AffineFlag( ctxId ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "affine_flag() affine=%d ctx=%d pos=(%d,%d)\n", cu.affine ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );

    if ( cu.affine && cu.cs->sps->getUseAffineType() )
    {
      ctxId = 0;
      cu.affineType = m_BinDecoder.decodeBin( Ctx::AffineType( ctxId ) );
      DTRACE( g_trace_ctx, D_SYNTAX, "affine_type() affine_type=%d ctx=%d pos=(%d,%d)\n", cu.affineType ? 1 : 0, ctxId, cu.Y().x, cu.Y().y );
    }
    else
    {
      cu.affineType = AFFINEMODEL_4PARAM;
    }
  }
}

#if AFFINE_MMVD
void CABACReader::affine_mmvd_data(PredictionUnit& pu)
{
  if (!pu.cs->sps->getUseAffineMmvdMode() || !pu.mergeFlag || !pu.cu->affine)
  {
    return;
  }
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__AFFINE_MMVD, pu.lumaSize());
  
  pu.afMmvdFlag = (m_BinDecoder.decodeBin(Ctx::AfMmvdFlag()));
  DTRACE(g_trace_ctx, D_SYNTAX, "affine_mmvd_flag() af_mmvd_merge=%d pos=(%d,%d) size=%dx%d\n", pu.afMmvdFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
  
  if (!pu.afMmvdFlag)
  {
    return;
  }
  
  // Base affine merge candidate idx
  uint8_t afMmvdBaseIdx = 0;
  int numCandMinus1Base = AF_MMVD_BASE_NUM - 1;
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  unsigned ctxId = 0;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  static_assert(ECM3_AF_MMVD_BASE_NUM == 1, "The value of ECM3_AF_MMVD_BASE_NUM must be 1");
  if (pu.cs->sps->getUseTMMMVD())
#endif
  {
    const CodingStructure *cs = pu.cu->cs;
    const CodingUnit *cuLeft = cs->getCURestricted(pu.cu->lumaPos().offset(-1, 0), *pu.cu, CH_L);
    ctxId = (cuLeft && cuLeft->affine) ? 1 : 0;
    const CodingUnit *cuAbove = cs->getCURestricted(pu.cu->lumaPos().offset(0, -1), *pu.cu, CH_L);
    ctxId += (cuAbove && cuAbove->affine) ? 1 : 0;
  }
  numCandMinus1Base = (ctxId == 0) ? 0 : ((ctxId == 1) ? 1 : AF_MMVD_BASE_NUM-1);
#endif
  if (numCandMinus1Base > 0)
  {
    // to support more base candidates
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
    int ctx2 =  (numCandMinus1Base == 1) ? 1 : 0;
    if (m_BinDecoder.decodeBin(Ctx::AfMmvdIdx(ctx2)))
#else
      if (m_BinDecoder.decodeBin(Ctx::AfMmvdIdx()))
#endif
      {
        afMmvdBaseIdx++;
        for (; afMmvdBaseIdx < numCandMinus1Base; afMmvdBaseIdx++)
        {
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
          if (!m_BinDecoder.decodeBin(Ctx::AfMmvdIdx(afMmvdBaseIdx + 1)))
#else
            if (!m_BinDecoder.decodeBinEP())
#endif
            {
              break;
            }
        }
      }
  }
  pu.afMmvdBaseIdx = afMmvdBaseIdx;
  DTRACE(g_trace_ctx, D_SYNTAX, "afMmvd_base_idx() afMmvd_base_idx=%d\n", afMmvdBaseIdx);
  
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  if (pu.cs->sps->getUseTMMMVD())
  {
#endif
  unsigned int ricePar = 1;
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  int numStepCandMinus1 =  (AF_MMVD_MAX_REFINE_NUM >> (ricePar+AFFINE_MMVD_SIZE_SHIFT)) / AFFINE_BI_DIR  - 1;
#else
  int numStepCandMinus1 =  (AF_MMVD_MAX_REFINE_NUM >> (ricePar + AFFINE_MMVD_SIZE_SHIFT))  - 1;
#endif
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  int temp = m_BinDecoder.decodeBin(Ctx::AfMmvdOffsetStep(5));
#else
  int temp = (ricePar == 0) ? 0 : m_BinDecoder.decodeBinsEP(ricePar);
#endif
  int uiUnaryIdx = 0;
  for (; uiUnaryIdx < numStepCandMinus1; ++uiUnaryIdx)
  {
    if (!m_BinDecoder.decodeBin(Ctx::AfMmvdOffsetStep((uiUnaryIdx > LAST_MERGE_MMVD_IDX_CABAC - 1 ? LAST_MERGE_MMVD_IDX_CABAC - 1 : uiUnaryIdx))))
    {
      break;
    }
  }
  uiUnaryIdx <<= ricePar;
  uiUnaryIdx += temp;
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  uiUnaryIdx += afMmvdBaseIdx * AF_MMVD_MAX_REFINE_NUM;
#endif
  pu.afMmvdMergeIdx = uiUnaryIdx;
  pu.afMmvdStep = 0;
  pu.afMmvdDir = 0;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    return;
  }
#endif
#endif

#if !JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED)
  {
    // Decode Step Value
    uint8_t stepOffset = 0;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    int numStepCandMinus1 = ECM3_AF_MMVD_STEP_NUM - 1;
#else
    int numStepCandMinus1 = AF_MMVD_STEP_NUM - 1;
#endif
    if (numStepCandMinus1 > 0)
    {
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
      if (m_BinDecoder.decodeBin(Ctx::AfMmvdOffsetStepECM3()))
#else
      if (m_BinDecoder.decodeBin(Ctx::AfMmvdOffsetStep()))
#endif
      {
        stepOffset++;
        for (; stepOffset < numStepCandMinus1; stepOffset++)
        {
          if (!m_BinDecoder.decodeBinEP())
          {
            break;
          }
        }
      }
    }
    pu.afMmvdStep = stepOffset;
    DTRACE(g_trace_ctx, D_SYNTAX, "afMmvd_offsetStep() afMmvd_offSetstep=%d\n", stepOffset);
    
    // Decode offset direction
    uint8_t b0 = 0, b1 = 0;
    b0 = m_BinDecoder.decodeBinEP();
    b1 = m_BinDecoder.decodeBinEP();
    uint8_t offDir = (b1 << 1) | b0;
    pu.afMmvdDir = offDir;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    pu.afMmvdMergeIdx = (uint8_t)(pu.afMmvdBaseIdx * ECM3_AF_MMVD_MAX_REFINE_NUM + pu.afMmvdStep * ECM3_AF_MMVD_OFFSET_DIR + pu.afMmvdDir);
#endif
    DTRACE(g_trace_ctx, D_SYNTAX, "afMmvd_offsetDir() afMmvd_offsetDir=%d\n", offDir);
  }
#endif
}
#endif

#if JVET_AA0061_IBC_MBVD
void CABACReader::ibcMbvdData(PredictionUnit& pu)
{
  if (!pu.cs->sps->getUseIbcMbvd() || !pu.mergeFlag || !CU::isIBC(*pu.cu))
  {
    return;
  }

  pu.ibcMbvdMergeFlag = (m_BinDecoder.decodeBin(Ctx::IbcMbvdFlag()));
  DTRACE(g_trace_ctx, D_SYNTAX, "IBC_mbvd_flag() IBC_mbvd_merge=%d pos=(%d,%d) size=%dx%d\n", pu.ibcMbvdMergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);

  if (!pu.ibcMbvdMergeFlag)
  {
    return;
  }

  // Base IBC merge candidate idx
  uint8_t var0 = 0;
  int numBaseCandMinus1 = IBC_MBVD_BASE_NUM - 1;
  if (numBaseCandMinus1 > 0)
  {
    // to support more base candidates
    if (m_BinDecoder.decodeBin(Ctx::IbcMbvdMergeIdx()))
    {
      var0++;
      for (; var0 < numBaseCandMinus1; var0++)
      {
        if (!m_BinDecoder.decodeBinEP())
        {
          break;
        }
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "ibcMbvdBaseIdx() ibcMbvdBaseIdx=%d\n", var0);

  unsigned int uiUnaryIdx = 0;
  unsigned int ricePar = 1;
  int numCandStepMinus1 = (IBC_MBVD_SIZE_ENC >> ricePar) - 1;
  int temp = 0;
  temp = m_BinDecoder.decodeBinsEP(ricePar);

  for (; uiUnaryIdx < numCandStepMinus1; ++uiUnaryIdx)
  {
    if (!m_BinDecoder.decodeBin(Ctx::IbcMbvdStepMvpIdx((uiUnaryIdx > LAST_MERGE_MMVD_IDX_CABAC - 1 ? LAST_MERGE_MMVD_IDX_CABAC - 1 : uiUnaryIdx))))
    {
      break;
    }
  }
  uiUnaryIdx <<= ricePar;
  uiUnaryIdx += temp;
  pu.ibcMbvdMergeIdx = var0 * IBC_MBVD_MAX_REFINE_NUM  +uiUnaryIdx;
  DTRACE(g_trace_ctx, D_SYNTAX, "IBC_mbvd_merge_idx() IBC_mbvd_merge_idx=%d\n", pu.ibcMbvdMergeIdx);
}
#endif

#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
void CABACReader::tm_merge_flag(PredictionUnit& pu)
{
  pu.tmMergeFlag = false;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_X0141_CIIP_TIMD_TM && TM_MRG
  if (pu.ciipFlag)
  {
    if (pu.cs->slice->getSPS()->getUseCiipTmMrg())
    {
      pu.tmMergeFlag = m_BinDecoder.decodeBin(Ctx::CiipTMMergeFlag());
    }
    return;
  }
#endif

#if (JVET_AA0132_CONFIGURABLE_TM_TOOLS && TM_MRG) || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_W0097_GPM_MMVD_TM && TM_MRG)
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_W0097_GPM_MMVD_TM && TM_MRG
  if (pu.cu->geoFlag)
  {
    if (!pu.cs->slice->getSPS()->getUseGPMTMMode())
    {
      return;
    }
  }
  else
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && TM_MRG
  if (pu.regularMergeFlag
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
    && !CU::isIBC(*pu.cu)
#endif
    )
  {
    if (!pu.cs->slice->getSPS()->getUseTMMrgMode())
    {
      return;
    }
  }
  else
#endif
  if (!pu.cs->slice->getSPS()->getUseDMVDMode())
  {
#if (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_X0141_CIIP_TIMD_TM && TM_MRG) && (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_W0097_GPM_MMVD_TM && TM_MRG) && (JVET_AA0132_CONFIGURABLE_TM_TOOLS && TM_MRG)
#if !(JVET_Z0084_IBC_TM && IBC_TM_MRG)
    CHECK(true, "Unknown mode to read tm_merge_flag");
#endif
#endif
    return;
  }
#else
  if (pu.cs->slice->getSPS()->getUseDMVDMode())
#endif
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MERGE_FLAG, pu.lumaSize());
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
    pu.tmMergeFlag = (m_BinDecoder.decodeBin(Ctx::TMMergeFlag(CU::isIBC(*pu.cu) ? 1 : 0)));
#else
    pu.tmMergeFlag = (m_BinDecoder.decodeBin(Ctx::TMMergeFlag()));
#endif
#if JVET_X0049_ADAPT_DMVR
    DTRACE(g_trace_ctx, D_SYNTAX, "tm_merge_flag() tmMergeFlag || bmMergeFlag=%d\n", pu.tmMergeFlag);
#else
    DTRACE(g_trace_ctx, D_SYNTAX, "tm_merge_flag() tmMergeFlag=%d\n", pu.tmMergeFlag ? 1 : 0);
#endif
  }
}
#endif

#if JVET_AC0112_IBC_CIIP
void CABACReader::ibcCiipFlag(PredictionUnit& pu)
{
  if (!pu.cs->sps->getUseIbcCiip() || ( pu.lx() == 0 && pu.ly() == 0))
  {
    pu.ibcCiipFlag = false;
    return;
  }
  if (pu.lwidth() * pu.lheight() < 32 || pu.lwidth() > 32 || pu.lheight() > 32)
  {
    pu.ibcCiipFlag = false;
    return;
  }
  if (pu.mergeFlag)
  {
    if (pu.cu->skip)
    {
      pu.ibcCiipFlag = false;
      return;
    }
    pu.ibcCiipFlag = m_BinDecoder.decodeBin(Ctx::IbcCiipFlag(0));
  }
  else
  {
#if JVET_AA0070_RRIBC
    if (pu.cu->rribcFlipType)
    {
      pu.ibcCiipFlag = false;
      return;
    }
#endif
#if JVET_AC0112_IBC_LIC
    if (pu.cu->ibcLicFlag)
    {
      pu.ibcCiipFlag = false;
      return;
    }
#endif
    if (pu.cs->slice->getSliceType() != I_SLICE)
    {
      pu.ibcCiipFlag = false;
      return;
    }
    pu.ibcCiipFlag = m_BinDecoder.decodeBin(Ctx::IbcCiipFlag(1));
  }
}

void CABACReader::ibcCiipIntraIdx(PredictionUnit& pu)
{
  pu.ibcCiipIntraIdx = m_BinDecoder.decodeBin( Ctx::IbcCiipIntraIdx() );
}
#endif

#if JVET_AC0112_IBC_GPM
void CABACReader::ibcGpmFlag(PredictionUnit& pu)
{
  if (!pu.cs->sps->getUseIbcGpm() || (pu.lx() == 0 && pu.ly() == 0))
  {
    pu.ibcGpmFlag = false;
    return;
  }
  if (pu.lwidth() < 8 || pu.lheight() < 8 || pu.lwidth() > 32 || pu.lheight() > 32)
  {
    pu.ibcGpmFlag = false;
    return;
  }
  pu.ibcGpmFlag = (m_BinDecoder.decodeBin(Ctx::IbcGpmFlag()));
}

void CABACReader::ibcGpmMergeIdx(PredictionUnit& pu)
{
  uint32_t splitDir = 0;
  bool splitDirFirstSet = m_BinDecoder.decodeBin( Ctx::IbcGpmSplitDirSetFlag() );
  if (splitDirFirstSet)
  {
    int splitDirIdx = m_BinDecoder.decodeBinsEP(3);
    splitDir = g_ibcGpmFirstSetSplitDir[splitDirIdx];
  }
  else
  {
    xReadTruncBinCode(splitDir, IBC_GPM_MAX_SPLIT_DIR_SECOND_SET_NUM);
    uint8_t prefix = splitDir;
    splitDir++;
    for (uint8_t i = 0; i < GEO_NUM_PARTITION_MODE && splitDir != 0; i++)
    {
      if (!g_ibcGpmSecondSetSplitDir[i])
      {
        prefix++;
      }
      else
      {
        splitDir--;
      }
    }
    splitDir = prefix;
  }
  pu.ibcGpmSplitDir = splitDir;

  bool isIntra0 = m_BinDecoder.decodeBin( Ctx::IbcGpmIntraFlag() ) ? true : false;
  bool isIntra1 = isIntra0 ? false : true;

  const int maxNumIbcGpmCand = pu.cs->sps->getMaxNumIBCMergeCand();
  int numCandminus2 = maxNumIbcGpmCand - 2;
  int mergeCand0 = 0;
  int mergeCand1 = 0;
  if (isIntra0)
  {
    mergeCand0 = IBC_GPM_MAX_NUM_UNI_CANDS + unary_max_eqprob(IBC_GPM_MAX_NUM_INTRA_CANDS-1);
  }
  else if (numCandminus2 >= 0)
  {
    if (m_BinDecoder.decodeBin(Ctx::MergeIdx()))
    {
      mergeCand0 += unary_max_eqprob(numCandminus2) + 1;
    }
  }
  if (isIntra1)
  {
    mergeCand1 = IBC_GPM_MAX_NUM_UNI_CANDS + unary_max_eqprob(IBC_GPM_MAX_NUM_INTRA_CANDS-1);
  }
  else if (numCandminus2 >= 0)
  {
    if (m_BinDecoder.decodeBin(Ctx::MergeIdx()))
    {
      mergeCand1 += unary_max_eqprob(numCandminus2) + 1;
    }
  }
  pu.ibcGpmMergeIdx0 = mergeCand0;
  pu.ibcGpmMergeIdx1 = mergeCand1;
}

void CABACReader::ibcGpmAdaptBlendIdx(PredictionUnit& pu)
{
  if (IBC_GPM_NUM_BLENDING == 1)
  {
    pu.ibcGpmBldIdx = 0;
    return;
  }
  int bin0 = m_BinDecoder.decodeBin(Ctx::IbcGpmBldIdx(0));
  if (bin0 == 1)
  {
    pu.ibcGpmBldIdx = 0;
  }
  else
  {
    int bin1 = m_BinDecoder.decodeBin(Ctx::IbcGpmBldIdx(1));
    if (bin1 == 0)
    {
      int bin2 = m_BinDecoder.decodeBin(Ctx::IbcGpmBldIdx(3));
      if (bin2 == 0)
      {
        pu.ibcGpmBldIdx = 4;
      }
      else
      {
        pu.ibcGpmBldIdx = 3;
      }
    }
    else
    {
      int bin2 = m_BinDecoder.decodeBin(Ctx::IbcGpmBldIdx(2));
      if (bin2 == 0)
      {
        pu.ibcGpmBldIdx = 1;
      }
      else
      {
        pu.ibcGpmBldIdx = 2;
      }
    }
  }
}
#endif

#if JVET_AC0112_IBC_LIC
void CABACReader::cuIbcLicFlag( CodingUnit& cu )
{
  if (!cu.cs->sps->getUseIbcLic() || !CU::isIBC(cu) || cu.firstPU->mergeFlag)
  {
    cu.ibcLicFlag = false;
    return;
  }
#if JVET_AA0070_RRIBC
  if (cu.rribcFlipType > 0)
  {
    cu.ibcLicFlag = false;
    return;
  }
#endif
  if (cu.lwidth() * cu.lheight() < 32 || cu.lwidth() * cu.lheight() > 256)
  {
    cu.ibcLicFlag = false;
    return;
  }
  cu.ibcLicFlag = m_BinDecoder.decodeBin( Ctx::IbcLicFlag() );
}
#endif

#if JVET_X0049_ADAPT_DMVR
void CABACReader::bm_merge_flag(PredictionUnit& pu)
{
  pu.bmDir = 0;
  pu.bmMergeFlag = false;
  if (!PU::isBMMergeFlagCoded(pu))
  {
    return;
  }
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MERGE_FLAG, pu.lumaSize());

  unsigned ctxId = DeriveCtx::CtxBMMrgFlag(*pu.cu);
  pu.bmMergeFlag = (m_BinDecoder.decodeBin(Ctx::BMMergeFlag(ctxId)));
  if (pu.bmMergeFlag)
  {
    pu.bmDir = 1 << m_BinDecoder.decodeBin(Ctx::BMMergeFlag(3));
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "bm_merge_flag() bmMergeFlag=%d, bmDir = %d\n", pu.bmMergeFlag ? 1 : 0, pu.bmDir);
}
#endif

void CABACReader::merge_flag( PredictionUnit& pu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__MERGE_FLAG, pu.lumaSize());

  pu.mergeFlag = ( m_BinDecoder.decodeBin( Ctx::MergeFlag() ) );

  DTRACE( g_trace_ctx, D_SYNTAX, "merge_flag() merge=%d pos=(%d,%d) size=%dx%d\n", pu.mergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height );

  if (pu.mergeFlag && CU::isIBC(*pu.cu))
  {
    pu.mmvdMergeFlag = false;
    pu.regularMergeFlag = false;
    return;
  }
}


void CABACReader::merge_data( PredictionUnit& pu )
{
  if (CU::isIBC(*pu.cu))
  {
#if JVET_AA0061_IBC_MBVD
    ibcMbvdData(pu);
#endif
#if JVET_Z0084_IBC_TM && IBC_TM_MRG
#if JVET_AA0061_IBC_MBVD
    if (!pu.ibcMbvdMergeFlag)
    {
#endif
      tm_merge_flag(pu);
#if JVET_AA0061_IBC_MBVD
    }
#endif
#endif
#if JVET_AC0112_IBC_CIIP
    ibcCiipFlag(pu);
    if (pu.ibcCiipFlag)
    {
      ibcCiipIntraIdx(pu);
    }
#endif
#if JVET_AC0112_IBC_GPM
#if JVET_AC0112_IBC_CIIP && JVET_AA0061_IBC_MBVD
    if (!pu.ibcMbvdMergeFlag && !pu.ibcCiipFlag)
#else
#if JVET_AA0061_IBC_MBVD
    if (!pu.ibcMbvdMergeFlag)
#else
#if JVET_AC0112_IBC_CIIP
    if (!pu.ibcCiipFlag)
#endif
#endif
#endif
    {
      ibcGpmFlag(pu);
      if (pu.ibcGpmFlag)
      {
        ibcGpmMergeIdx(pu);
        ibcGpmAdaptBlendIdx(pu);
      }
    }
#endif
    merge_idx(pu);
    return;
  }
  else
  {
    CodingUnit cu = *pu.cu;
    subblock_merge_flag(*pu.cu);
    if (pu.cu->affine)
    {
#if AFFINE_MMVD
      affine_mmvd_data(pu);
#endif
      merge_idx(pu);
      cu.firstPU->regularMergeFlag = false;
      return;
    }

    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__MERGE_FLAG, pu.lumaSize());

#if CIIP_RM_BLOCK_SIZE_CONSTRAINTS
#if CTU_256
    const int maxSize = std::min<int>( MAX_TB_SIZEY, MAX_INTRA_SIZE );
    
    const bool ciipAvailable = pu.cs->sps->getUseCiip() && !pu.cu->skip && pu.cu->lwidth() * pu.cu->lheight() >= 32 && pu.cu->lwidth() <= maxSize && pu.cu->lheight() <= maxSize;
#else
    const bool ciipAvailable = pu.cs->sps->getUseCiip() && !pu.cu->skip && pu.cu->lwidth() * pu.cu->lheight() >= 32;
#endif
#else
    const bool ciipAvailable = pu.cs->sps->getUseCiip() && !pu.cu->skip && pu.cu->lwidth() < MAX_CU_SIZE && pu.cu->lheight() < MAX_CU_SIZE && pu.cu->lwidth() * pu.cu->lheight() >= 64;
#endif

#if JVET_Y0065_GPM_INTRA
    const bool geoAvailable  = pu.cu->cs->slice->getSPS()->getUseGeo() && !pu.cu->cs->slice->isIntra()
                              && pu.cs->sps->getMaxNumGeoCand() > 0 && pu.cu->lwidth() >= GEO_MIN_CU_SIZE
#else
    const bool geoAvailable  = pu.cu->cs->slice->getSPS()->getUseGeo() && pu.cu->cs->slice->isInterB()
                              && pu.cs->sps->getMaxNumGeoCand() > 1 && pu.cu->lwidth() >= GEO_MIN_CU_SIZE
#endif
                              && pu.cu->lheight() >= GEO_MIN_CU_SIZE && pu.cu->lwidth() <= GEO_MAX_CU_SIZE
                              && pu.cu->lheight() <= GEO_MAX_CU_SIZE && pu.cu->lwidth() < 8 * pu.cu->lheight()
                              && pu.cu->lheight() < 8 * pu.cu->lwidth();

    if (geoAvailable || ciipAvailable)
    {
      cu.firstPU->regularMergeFlag = m_BinDecoder.decodeBin(Ctx::RegularMergeFlag(cu.skip ? 0 : 1));
    }
    else
    {
      cu.firstPU->regularMergeFlag = true;
    }
    if (cu.firstPU->regularMergeFlag)
    {
#if TM_MRG
      tm_merge_flag(pu);
#endif
#if JVET_X0049_ADAPT_DMVR
#if TM_MRG
      if (pu.tmMergeFlag
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
        || !pu.cs->slice->getSPS()->getUseTMMrgMode()
#endif
        )
#endif
      {
        bm_merge_flag(pu);
#if TM_MRG
        if (pu.bmMergeFlag)
        {
          pu.tmMergeFlag = false;
        }
#endif
      }
#endif
      if (cu.cs->slice->getSPS()->getUseMMVD()
#if TM_MRG
        && !pu.tmMergeFlag
#endif
#if JVET_X0049_ADAPT_DMVR
        && !pu.bmMergeFlag
#endif
        )
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MERGE_FLAG, pu.lumaSize());
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
        unsigned  ctxId =
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
                          !pu.cs->sps->getUseTMMMVD() ||
#endif
                          pu.cu->skip ? 0 : 1;
        cu.firstPU->mmvdMergeFlag = m_BinDecoder.decodeBin(Ctx::MmvdFlag(ctxId));
#else
        cu.firstPU->mmvdMergeFlag = m_BinDecoder.decodeBin(Ctx::MmvdFlag(0));
#endif
        DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_flag() mmvd_merge=%d pos=(%d,%d) size=%dx%d\n", cu.firstPU->mmvdMergeFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
      }
      else
      {
        cu.firstPU->mmvdMergeFlag = false;
      }
      if (cu.skip)
      {
        cu.mmvdSkip = cu.firstPU->mmvdMergeFlag;
      }
    }
    else
    {
      pu.mmvdMergeFlag = false;
      pu.cu->mmvdSkip = false;
#if TM_MRG
      pu.tmMergeFlag = false;
#endif
#if JVET_X0049_ADAPT_DMVR
      pu.bmMergeFlag = false;
#endif
      if (geoAvailable && ciipAvailable)
      {
        Ciip_flag(pu);
      }
      else if (ciipAvailable)
      {
        pu.ciipFlag = true;
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
        tm_merge_flag(pu);
#else
        if (pu.cs->slice->getSPS()->getUseCiipTmMrg())
        {
          RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MERGE_FLAG, pu.lumaSize()); 
          pu.tmMergeFlag = (m_BinDecoder.decodeBin(Ctx::CiipTMMergeFlag()));
        }
#endif
#endif
#if CIIP_PDPC
        RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__INTRA_PDPC_FLAG, pu.lumaSize()); 
        pu.ciipPDPC = ( m_BinDecoder.decodeBin( Ctx::CiipFlag( 1 ) ) );
#endif
      }
      else
      {
        pu.ciipFlag = false;
      }
      if (pu.ciipFlag)
      {
        pu.intraDir[0] = PLANAR_IDX;
        pu.intraDir[1] = DM_CHROMA_IDX;
      }
      else
      {
        pu.cu->geoFlag = true;
      }
    }
  }
  if (pu.mmvdMergeFlag || pu.cu->mmvdSkip)
  {
    mmvd_merge_idx(pu);
  }
  else
  {
    merge_idx(pu);
  }
}


void CABACReader::merge_idx( PredictionUnit& pu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MERGE_INDEX, pu.lumaSize());

  if ( pu.cu->affine )
  {
#if AFFINE_MMVD
    if (pu.afMmvdFlag)
    {
      return;
    }
#endif
    int numCandminus1 = int( pu.cs->picHeader->getMaxNumAffineMergeCand() ) - 1;
    pu.mergeIdx = 0;
    if ( numCandminus1 > 0 )
    {
#if JVET_AA0128_AFFINE_MERGE_CTX_INC
      unsigned int unaryIdx = 0;
      for (; unaryIdx < numCandminus1; ++unaryIdx)
      {
        if (!m_BinDecoder.decodeBin(Ctx::AffMergeIdx((unaryIdx > 2 ? 2 : unaryIdx))))
        {
          break;
        }
      }
      pu.mergeIdx = unaryIdx;
#else
      if ( m_BinDecoder.decodeBin( Ctx::AffMergeIdx() ) )
      {
        pu.mergeIdx++;
        for ( ; pu.mergeIdx < numCandminus1; pu.mergeIdx++ )
        {
          if (!m_BinDecoder.decodeBinEP())
          {
            break;
          }
        }
      }
#endif
    }
    DTRACE( g_trace_ctx, D_SYNTAX, "aff_merge_idx() aff_merge_idx=%d\n", pu.mergeIdx );
  }
  else
  {
    int numCandminus1 = int(pu.cs->sps->getMaxNumMergeCand()) - 1;
    pu.mergeIdx       = 0;

    if (pu.cu->geoFlag)
    {
      RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__GEO_INDEX, pu.lumaSize());
#if JVET_AA0058_GPM_ADP_BLD
      geoAdaptiveBlendingIdx(pu);
#endif
#if JVET_W0097_GPM_MMVD_TM
#if JVET_Y0065_GPM_INTRA
      bool isIntra0 = false;
      bool isIntra1 = false;
      bool bUseOnlyOneVector = pu.cs->slice->isInterP() || pu.cs->sps->getMaxNumGeoCand() == 1;
#endif
      pu.geoMMVDFlag0 = m_BinDecoder.decodeBin(Ctx::GeoMmvdFlag());
      if (pu.geoMMVDFlag0)
      {
        geo_mmvd_idx(pu, REF_PIC_LIST_0);
      }

#if JVET_Y0065_GPM_INTRA
      else
      {
        isIntra0 = m_BinDecoder.decodeBin( Ctx::GPMIntraFlag() ) ? true : false;
      }

      if (!bUseOnlyOneVector || isIntra0)
      {
#endif
      pu.geoMMVDFlag1 = m_BinDecoder.decodeBin(Ctx::GeoMmvdFlag());
      if (pu.geoMMVDFlag1)
      {
        geo_mmvd_idx(pu, REF_PIC_LIST_1);
      }
#if JVET_Y0065_GPM_INTRA
      else if (!isIntra0)
      {
        isIntra1 = m_BinDecoder.decodeBin( Ctx::GPMIntraFlag() ) ? true : false;
      }
      }
      else
      {
        isIntra1 = true;
      }
      pu.gpmIntraFlag = (isIntra0 || isIntra1);
#endif

#if TM_MRG
      if (!pu.geoMMVDFlag0 && !pu.geoMMVDFlag1)
      {
#if JVET_Y0065_GPM_INTRA
        if (isIntra0 || isIntra1)
        {
          pu.tmMergeFlag = false;
        }
        else
#endif
        tm_merge_flag(pu);
        if (pu.tmMergeFlag)
        {
          pu.geoTmFlag0 = true;
          pu.geoTmFlag1 = true;
          geo_merge_idx(pu);
        }
        else
        {
          pu.geoTmFlag0 = false;
          pu.geoTmFlag1 = false;
#if JVET_Y0065_GPM_INTRA
          if (isIntra0 || isIntra1)
          {
            geo_merge_idx1(pu, isIntra0, isIntra1);
          }
          else
#endif
          geo_merge_idx(pu);
        }
      }
#else
      if (!pu.geoMMVDFlag0 && !pu.geoMMVDFlag1)
      {
#if JVET_Y0065_GPM_INTRA
        if( isIntra0 || isIntra1 )
        {
          geo_merge_idx1( pu, isIntra0, isIntra1 );
        }
        else
#endif
        geo_merge_idx(pu);
      }
#endif
      else if (pu.geoMMVDFlag0 && pu.geoMMVDFlag1)
      {
        if (pu.geoMMVDIdx0 == pu.geoMMVDIdx1)
        {
          geo_merge_idx(pu);
        }
        else
        {
#if JVET_Y0065_GPM_INTRA
          geo_merge_idx1(pu, isIntra0, isIntra1);
#else
          geo_merge_idx1(pu);
#endif
        }
      }
      else
      {
#if JVET_Y0065_GPM_INTRA
        geo_merge_idx1(pu, isIntra0, isIntra1);
#else
        geo_merge_idx1(pu);
#endif
      }
#else
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
      geoModeIdx(pu);
#else
      uint32_t splitDir = 0;
      xReadTruncBinCode(splitDir, GEO_NUM_PARTITION_MODE);
      pu.geoSplitDir          = splitDir;
#endif
      const int maxNumGeoCand = pu.cs->sps->getMaxNumGeoCand();
      CHECK(maxNumGeoCand < 2, "Incorrect max number of geo candidates");
      CHECK(pu.cu->lheight() > 64 || pu.cu->lwidth() > 64, "Incorrect block size of geo flag");
      int numCandminus2 = maxNumGeoCand - 2;
      pu.mergeIdx       = 0;
      int mergeCand0    = 0;
      int mergeCand1    = 0;
      if (m_BinDecoder.decodeBin(Ctx::MergeIdx()))
      {
        mergeCand0 += unary_max_eqprob(numCandminus2) + 1;
      }
      if (numCandminus2 > 0)
      {
        if (m_BinDecoder.decodeBin(Ctx::MergeIdx()))
        {
          mergeCand1 += unary_max_eqprob(numCandminus2 - 1) + 1;
        }
      }
      mergeCand1 += mergeCand1 >= mergeCand0 ? 1 : 0;
      pu.geoMergeIdx0 = mergeCand0;
      pu.geoMergeIdx1 = mergeCand1;
#if !JVET_Z0056_GPM_SPLIT_MODE_REORDERING
      DTRACE(g_trace_ctx, D_SYNTAX, "merge_idx() geo_split_dir=%d\n", splitDir);
#endif
      DTRACE(g_trace_ctx, D_SYNTAX, "merge_idx() geo_idx0=%d\n", mergeCand0);
      DTRACE(g_trace_ctx, D_SYNTAX, "merge_idx() geo_idx1=%d\n", mergeCand1);
#endif
      return;
    }

  if (pu.cu->predMode == MODE_IBC)
  {
#if JVET_AA0061_IBC_MBVD
    if (pu.ibcMbvdMergeFlag)
    {
      return;
    }
#endif
#if JVET_AC0112_IBC_GPM
    if (pu.ibcGpmFlag)
    {
      pu.mergeIdx = pu.ibcGpmMergeIdx0 < IBC_GPM_MAX_NUM_UNI_CANDS ? pu.ibcGpmMergeIdx0 : pu.ibcGpmMergeIdx1;
      return;
    }
#endif
    numCandminus1 = int(pu.cs->sps->getMaxNumIBCMergeCand()) - 1;
  }
#if TM_MRG
  else if (pu.tmMergeFlag)
  {
#if JVET_X0141_CIIP_TIMD_TM
    if (pu.ciipFlag)
    {
      numCandminus1 = int(pu.cs->sps->getMaxNumCiipTMMergeCand()) - 1;
    }
    else
    {
      numCandminus1 = int(pu.cs->sps->getMaxNumTMMergeCand()) - 1;
    }
#else
    numCandminus1 = int(pu.cs->sps->getMaxNumTMMergeCand()) - 1;
#endif
  }
#endif
#if JVET_X0049_ADAPT_DMVR
  else if (pu.bmMergeFlag)
  {
    numCandminus1 = int(pu.cs->sps->getMaxNumBMMergeCand()) - 1;
  }
#endif
  if( numCandminus1 > 0 )
  {
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
    const CtxSet mrgIdxCtxSet = pu.tmMergeFlag ? Ctx::TmMergeIdx : Ctx::MergeIdx;
#endif
#if NON_ADJACENT_MRG_CAND
    unsigned int uiUnaryIdx = 0;
    for (; uiUnaryIdx < numCandminus1; ++uiUnaryIdx)
    {
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
      if (!m_BinDecoder.decodeBin(mrgIdxCtxSet((uiUnaryIdx > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : uiUnaryIdx))))
#else
      if (!m_BinDecoder.decodeBin(Ctx::MergeIdx((uiUnaryIdx > LAST_MERGE_IDX_CABAC - 1 ? LAST_MERGE_IDX_CABAC - 1 : uiUnaryIdx))))
#endif
      {
        break;
      }
    }
    pu.mergeIdx = uiUnaryIdx;
#else
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
    if (m_BinDecoder.decodeBin(mrgIdxCtxSet()))
#else
    if (m_BinDecoder.decodeBin(Ctx::MergeIdx()))
#endif
    {
      pu.mergeIdx++;
      for (; pu.mergeIdx < numCandminus1; pu.mergeIdx++)
      {
        if( !m_BinDecoder.decodeBinEP() )
        {
          break;
        }
      }
    }
#endif
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "merge_idx() merge_idx=%d\n", pu.mergeIdx );
#if JVET_X0049_ADAPT_DMVR
  if (pu.bmMergeFlag && pu.bmDir == 2)
  {
    pu.mergeIdx += BM_MRG_MAX_NUM_CANDS;
  }
#endif
  }
}
#if JVET_W0097_GPM_MMVD_TM
void CABACReader::geo_mmvd_idx(PredictionUnit& pu, RefPicList eRefPicList)
{
  bool extMMVD = pu.cs->picHeader->getGPMMMVDTableFlag();
  int numStepCandMinus1 = (extMMVD ? GPM_EXT_MMVD_REFINE_STEP : GPM_MMVD_REFINE_STEP) - 1;
  int step = 0;
  if (m_BinDecoder.decodeBin(Ctx::GeoMmvdStepMvpIdx()))
  {
    step++;
    for (; step < numStepCandMinus1; step++)
    {
      if (!m_BinDecoder.decodeBinEP())
      {
        break;
      }
    }
  }

  int idxToMMVDStep[GPM_EXT_MMVD_REFINE_STEP] = { 1, 2, 3, 4, 5, 0, 6, 7, 8 };
  step = idxToMMVDStep[step];

  int direction = 0;
  int maxMMVDDir = (extMMVD ? GPM_EXT_MMVD_REFINE_DIRECTION : GPM_MMVD_REFINE_DIRECTION);
  direction = m_BinDecoder.decodeBinsEP(maxMMVDDir > 4 ? 3 : 2);
  int mvpIdx = (step * maxMMVDDir + direction);
  if (eRefPicList == REF_PIC_LIST_0)
  {
    pu.geoMMVDIdx0 = mvpIdx;
  }
  else
  {
    pu.geoMMVDIdx1 = mvpIdx;
  }
}


void CABACReader::geo_merge_idx(PredictionUnit& pu)
{
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  geoModeIdx(pu);
#else
  uint32_t splitDir = 0;
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__GEO_INDEX, pu.lumaSize());
  xReadTruncBinCode(splitDir, GEO_NUM_PARTITION_MODE);
  pu.geoSplitDir = splitDir;
#endif
  const int maxNumGeoCand = pu.cs->sps->getMaxNumGeoCand();
  CHECK(maxNumGeoCand < 2, "Incorrect max number of geo candidates");
  CHECK(pu.cu->lheight() > 64 || pu.cu->lwidth() > 64, "Incorrect block size of geo flag");
  int numCandminus2 = maxNumGeoCand - 2;
  pu.mergeIdx = 0;
  int mergeCand0 = 0;
  int mergeCand1 = 0;
  if (m_BinDecoder.decodeBin(Ctx::MergeIdx()))
  {
    mergeCand0 += unary_max_eqprob(numCandminus2) + 1;
  }
  if (numCandminus2 > 0)
  {
    if (m_BinDecoder.decodeBin(Ctx::MergeIdx()))
    {
      mergeCand1 += unary_max_eqprob(numCandminus2 - 1) + 1;
    }
  }
  mergeCand1 += mergeCand1 >= mergeCand0 ? 1 : 0;
  pu.geoMergeIdx0 = mergeCand0;
  pu.geoMergeIdx1 = mergeCand1;
}

#if JVET_Y0065_GPM_INTRA
void CABACReader::geo_merge_idx1(PredictionUnit& pu, bool isIntra0, bool isIntra1)
#else
void CABACReader::geo_merge_idx1(PredictionUnit& pu)
#endif
{
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  geoModeIdx(pu);
#else
  uint32_t splitDir = 0;
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__GEO_INDEX, pu.lumaSize());
  xReadTruncBinCode(splitDir, GEO_NUM_PARTITION_MODE);
  pu.geoSplitDir = splitDir;
#endif
  const int maxNumGeoCand = pu.cs->sps->getMaxNumGeoCand();
#if JVET_Y0065_GPM_INTRA
  CHECK(maxNumGeoCand < 1, "Incorrect max number of geo candidates");
#else
  CHECK(maxNumGeoCand < 2, "Incorrect max number of geo candidates");
#endif
  CHECK(pu.cu->lheight() > 64 || pu.cu->lwidth() > 64, "Incorrect block size of geo flag");
  int numCandminus2 = maxNumGeoCand - 2;
  pu.mergeIdx = 0;
  int mergeCand0 = 0;
  int mergeCand1 = 0;
#if JVET_Y0065_GPM_INTRA
  if (isIntra0)
  {
    mergeCand0 = GEO_MAX_NUM_UNI_CANDS + unary_max_eqprob(GEO_MAX_NUM_INTRA_CANDS-1);
  }
  else if (numCandminus2 >= 0)
  {
#endif
  if (m_BinDecoder.decodeBin(Ctx::MergeIdx()))
  {
    mergeCand0 += unary_max_eqprob(numCandminus2) + 1;
  }
#if JVET_Y0065_GPM_INTRA
  }
  if (isIntra1)
  {
    mergeCand1 = GEO_MAX_NUM_UNI_CANDS + unary_max_eqprob(GEO_MAX_NUM_INTRA_CANDS-1);
  }
  else if (numCandminus2 >= 0)
  {
#endif
  if (m_BinDecoder.decodeBin(Ctx::MergeIdx()))
  {
    mergeCand1 += unary_max_eqprob(numCandminus2) + 1;
  }
#if JVET_Y0065_GPM_INTRA
  }
#endif
  pu.geoMergeIdx0 = mergeCand0;
  pu.geoMergeIdx1 = mergeCand1;
}
#endif

#if JVET_AA0058_GPM_ADP_BLD
void CABACReader::geoAdaptiveBlendingIdx( PredictionUnit& pu )
{
  int bin0 = m_BinDecoder.decodeBin( Ctx::GeoBldFlag( 0 ) );
  if( bin0 == 1 )
  {
    pu.geoBldIdx = 2; //1
  }
  else
  {
    int bin1 = m_BinDecoder.decodeBin( Ctx::GeoBldFlag( 1 ) );
    if( bin1 == 0 )
    {
      int bin2 = m_BinDecoder.decodeBin( Ctx::GeoBldFlag( 3 ) );
      if( bin2 == 0 )
      {
        pu.geoBldIdx = 4; //000
      }
      else
      {
        pu.geoBldIdx = 3; //001
      }
    }
    else
    {
      int bin2 = m_BinDecoder.decodeBin( Ctx::GeoBldFlag( 2 ) );
      if( bin2 == 0 )
      {
        pu.geoBldIdx = 1; //010
      }
      else
      {
        pu.geoBldIdx = 0; //011
      }
    }
  }
}
#endif

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
void CABACReader::geoModeIdx(PredictionUnit& pu)
{
  if (!pu.cs->slice->getSPS()->getUseAltGPMSplitModeCode())
  {
    uint32_t geoMode = 0;
    xReadTruncBinCode(geoMode, GEO_NUM_PARTITION_MODE);
    pu.geoSplitDir = geoMode;
    DTRACE(g_trace_ctx, D_SYNTAX, "merge_idx() geo_split_dir=%u\n", geoMode);
    return;
  }

  const int maxNumBins    = (GEO_NUM_SIG_PARTMODE / GEO_SPLIT_MODE_RICE_CODE_DIVISOR) - 1;
  const int maxNumCtxBins = 5;
        int geoModePrefix = 0;
  for (int binIdx = 0; binIdx < maxNumBins; ++binIdx, ++geoModePrefix)
  {
    if (binIdx < maxNumCtxBins)
    {
      if (!m_BinDecoder.decodeBin(Ctx::GeoSubModeIdx(binIdx)))
      {
        break;
      }
    }
    else
    {
      if (!m_BinDecoder.decodeBinEP())
      {
        break;
      }
    }
  }

  if (GEO_SPLIT_MODE_RICE_CODE_DIVISOR > 1)
  {
    uint32_t geoModeSuffix = 0;
    xReadTruncBinCode(geoModeSuffix, GEO_SPLIT_MODE_RICE_CODE_DIVISOR);
    uint32_t geoMode = ((uint32_t)(geoModePrefix * GEO_SPLIT_MODE_RICE_CODE_DIVISOR)) + geoModeSuffix;
    pu.geoSyntaxMode = geoMode;
    pu.geoSplitDir   = geoMode;
    DTRACE(g_trace_ctx, D_SYNTAX, "merge_idx() geo_split_dir=%d(prefix=%d suffix=%u)\n", geoMode, geoModePrefix, geoModeSuffix);
  }
  else
  {
    uint32_t geoMode = (uint32_t)geoModePrefix;
    pu.geoSyntaxMode = geoMode;
    pu.geoSplitDir   = geoMode;
    DTRACE(g_trace_ctx, D_SYNTAX, "merge_idx() geo_split_dir=%u\n", geoMode);
  }
}
#endif

void CABACReader::mmvd_merge_idx(PredictionUnit& pu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MERGE_INDEX, pu.lumaSize());
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  if (pu.cs->sps->getUseTMMMVD())
  {
#endif
  unsigned int uiUnaryIdx = 0;
  int var0 = 0;
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  int numCandMinus1Base = std::min<int>(MMVD_BASE_MV_NUM, pu.cs->sps->getMaxNumMergeCand()) - 1;
  if (numCandMinus1Base > 0)
  {
    // to support more base candidates
    if (m_BinDecoder.decodeBin(Ctx::MmvdMergeIdx(0)))
    {
      var0++;
      for (; var0 < numCandMinus1Base; var0++)
      {
        if (!m_BinDecoder.decodeBin(Ctx::MmvdMergeIdx(var0)))
        {
          break;
        }
      }
    }
  }
#else
  if (pu.cs->sps->getMaxNumMergeCand() > 1)
  {
    static_assert(MMVD_BASE_MV_NUM == 2, "");
    var0 = m_BinDecoder.decodeBin(Ctx::MmvdMergeIdx());
  }
#endif
  DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_idx() base_mvp_idx=%d\n", var0);
  unsigned int ricePar = 1;
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  int numStepCandMinus1 =  ((MMVD_MAX_REFINE_NUM >> (ricePar+MMVD_SIZE_SHIFT)))/MMVD_BI_DIR  - 1;
#else
  int numStepCandMinus1 =  (MMVD_MAX_REFINE_NUM >> (ricePar+MMVD_SIZE_SHIFT))  - 1;
#endif
  int temp = 0;
  temp = m_BinDecoder.decodeBinsEP(ricePar);
  
  for (; uiUnaryIdx < numStepCandMinus1; ++uiUnaryIdx)
  {
    if (!m_BinDecoder.decodeBin(Ctx::MmvdStepMvpIdx((uiUnaryIdx > LAST_MERGE_MMVD_IDX_CABAC - 1 ? LAST_MERGE_MMVD_IDX_CABAC - 1 : uiUnaryIdx))))
    {
      break;
    }
  }
  uiUnaryIdx <<= ricePar;
  uiUnaryIdx += temp;
  pu.mmvdMergeIdx = var0 * MMVD_MAX_REFINE_NUM  +uiUnaryIdx;
  DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_idx() mmvd_merge_idx=%d\n", pu.mmvdMergeIdx);
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    return;
  }
#endif
#endif

#if !JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED)
  int var0 = 0;
  if (pu.cs->sps->getMaxNumMergeCand() > 1)
  {
#if !JVET_AA0093_ENHANCED_MMVD_EXTENSION
    static_assert(MMVD_BASE_MV_NUM == 2, "");
#endif
    var0 = m_BinDecoder.decodeBin(Ctx::MmvdMergeIdx());
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "base_mvp_idx() base_mvp_idx=%d\n", var0);
  int numStepCandMinus1 = 
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
                          VVC_MMVD_REFINE_STEP - 1;
#else
                          MMVD_REFINE_STEP - 1;
#endif
  int var1 = 0;

#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  if (m_BinDecoder.decodeBin(Ctx::MmvdStepMvpIdxECM3()))
#else
  if (m_BinDecoder.decodeBin(Ctx::MmvdStepMvpIdx()))
#endif
  {
    var1++;
    for (; var1 < numStepCandMinus1; var1++)
    {
      if (!m_BinDecoder.decodeBinEP())
      {
        break;
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "MmvdStepMvpIdx() MmvdStepMvpIdx=%d\n", var1);
  int var2 = 0;
  if (m_BinDecoder.decodeBinEP())
  {
    var2 += 2;
    if (m_BinDecoder.decodeBinEP())
    {
      var2 += 1;
    }
  }
  else
  {
    var2 += 0;
    if (m_BinDecoder.decodeBinEP())
    {
      var2 += 1;
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "pos() pos=%d\n", var2);
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  int mvpIdx = (var0 * VVC_MMVD_MAX_REFINE_NUM + var1 * VVC_MMVD_MAX_DIR + var2);
#else
  int mvpIdx = (var0 * MMVD_MAX_REFINE_NUM + var1 * 4 + var2);
#endif
  pu.mmvdMergeIdx = mvpIdx;
  DTRACE(g_trace_ctx, D_SYNTAX, "mmvd_merge_idx() mmvd_merge_idx=%d\n", pu.mmvdMergeIdx);
#endif
}

void CABACReader::inter_pred_idc( PredictionUnit& pu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__INTER_DIR, pu.lumaSize());

  if( pu.cs->slice->isInterP() )
  {
    pu.interDir = 1;
    return;
  }
  if( !(PU::isBipredRestriction(pu)) )
  {
    unsigned ctxId = DeriveCtx::CtxInterDir(pu);
    if( m_BinDecoder.decodeBin( Ctx::InterDir(ctxId) ) )
    {
      DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=%d value=%d pos=(%d,%d)\n", ctxId, 3, pu.lumaPos().x, pu.lumaPos().y );
      pu.interDir = 3;
      return;
    }
  }
#if JVET_Z0054_BLK_REF_PIC_REORDER
  if (pu.cs->sps->getUseARL())
  {
    pu.interDir = 1; // temporally set as 1 for enabling refIdxLC()
    return;
  }
#endif
#if CTU_256
  if( m_BinDecoder.decodeBin( Ctx::InterDir( 7 ) ) )
#else
  if( m_BinDecoder.decodeBin( Ctx::InterDir(6) ) )
#endif
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=5 value=%d pos=(%d,%d)\n", 2, pu.lumaPos().x, pu.lumaPos().y );
    pu.interDir = 2;
    return;
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "inter_pred_idc() ctx=5 value=%d pos=(%d,%d)\n", 1, pu.lumaPos().x, pu.lumaPos().y );
  pu.interDir = 1;
  return;
}

#if JVET_Z0054_BLK_REF_PIC_REORDER
void CABACReader::refIdxLC(PredictionUnit& pu)
{
  if (!PU::useRefCombList(pu))
  {
    return;
  }
  int numRefMinus1 = (int)pu.cs->slice->getRefPicCombinedList().size() - 1;
  int refIdxLC = 0;
#if JVET_X0083_BM_AMVP_MERGE_MODE
  if (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])
  {
    numRefMinus1 = (int)pu.cs->slice->getRefPicCombinedListAmvpMerge().size() - 1;
  }
#endif
  if (numRefMinus1 > 0)
  {
    if (m_BinDecoder.decodeBin(Ctx::RefPicLC(0)))
    {
      refIdxLC++;
      for (; refIdxLC < numRefMinus1; refIdxLC++)
      {
        if (!m_BinDecoder.decodeBin(Ctx::RefPicLC(std::min(refIdxLC, 2))))
        {
          break;
        }
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "refIdxLC() value=%d pos=(%d,%d)\n", refIdxLC, pu.lumaPos().x, pu.lumaPos().y);
  pu.refIdxLC = refIdxLC;
  // temporally set interDir and refIdx (has to be done)
  if (pu.interDir != 3)
  {
    pu.interDir = 1;
  }
  pu.refIdx[0] = 0;
}

void CABACReader::refPairIdx(PredictionUnit& pu)
{
  if (!PU::useRefPairList(pu))
  {
    return;
  }
  int numRefMinus1 = (int)pu.cs->slice->getRefPicPairList().size() - 1;
  int refPairIdx = 0;
  if (numRefMinus1 > 0)
  {
    if (m_BinDecoder.decodeBin(Ctx::RefPicLC(0)))
    {
      refPairIdx++;
      for (; refPairIdx < numRefMinus1; refPairIdx++)
      {
        if (!m_BinDecoder.decodeBin(Ctx::RefPicLC(std::min(refPairIdx, 2))))
        {
          break;
        }
      }
    }
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "refPairIdx() value=%d pos=(%d,%d)\n", refPairIdx, pu.lumaPos().x, pu.lumaPos().y);
  pu.refPairIdx = refPairIdx;
  // temporally set refIdx
  pu.refIdx[0] = 0;
  pu.refIdx[1] = 0;
}
#endif

void CABACReader::ref_idx( PredictionUnit &pu, RefPicList eRefList )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__REF_FRM_IDX, pu.lumaSize());

  if ( pu.cu->smvdMode )
  {
    pu.refIdx[eRefList] = pu.cs->slice->getSymRefIdx( eRefList );
    return;
  }
#if JVET_Z0054_BLK_REF_PIC_REORDER
  if (PU::useRefCombList(pu) || PU::useRefPairList(pu))
  {
    return;
  }
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
  if (pu.amvpMergeModeFlag[1 - eRefList])
  {
#if JVET_Y0128_NON_CTC
    if (pu.cs->slice->getAmvpMergeModeOnlyOneValidRefIdx(eRefList) >= 0)
    {
      pu.refIdx[eRefList] = pu.cs->slice->getAmvpMergeModeOnlyOneValidRefIdx(eRefList);
      return;
    }
#else
    const RefPicList refListAmvp = eRefList;
    const RefPicList refListMerge = RefPicList(1 - eRefList);
    const int curPoc = pu.cs->slice->getPOC();
    const int numRefAmvp = pu.cs->slice->getNumRefIdx(refListAmvp);
    const int numRefMerge = pu.cs->slice->getNumRefIdx(refListMerge);
    int candidateRefIdxCount = 0;
    int onlyOneValidRefIdxAmvp = -1;
    for (int refIdxAmvp = 0; refIdxAmvp < numRefAmvp; refIdxAmvp++)
    {
      const int amvpPoc = pu.cs->slice->getRefPOC(refListAmvp, refIdxAmvp);
      bool validCandidate = false;
      for (int refIdxMerge = 0; refIdxMerge < numRefMerge; refIdxMerge++)
      {
        const int mergePoc = pu.cs->slice->getRefPOC(refListMerge, refIdxMerge);
        if ((amvpPoc - curPoc) * (mergePoc - curPoc) < 0)
        {
          validCandidate = true;
        }
      }
      if (validCandidate)
      {
        onlyOneValidRefIdxAmvp = refIdxAmvp;
        candidateRefIdxCount++;
      }
    }
    CHECK(candidateRefIdxCount == 0, "this is not possible");
    if (candidateRefIdxCount == 1)
    {
      pu.refIdx[eRefList] = onlyOneValidRefIdxAmvp;
      return;
    }
#endif
  }
#endif

  int numRef  = pu.cs->slice->getNumRefIdx(eRefList);

  if( numRef <= 1 || !m_BinDecoder.decodeBin( Ctx::RefPic() ) )
  {
    if( numRef > 1 )
    {
      DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", 0, pu.lumaPos().x, pu.lumaPos().y );
    }
    pu.refIdx[eRefList] = 0;
    return;
  }
  if( numRef <= 2 || !m_BinDecoder.decodeBin( Ctx::RefPic(1) ) )
  {
    DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", 1, pu.lumaPos().x, pu.lumaPos().y );
    pu.refIdx[eRefList] = 1;
    return;
  }
  for( int idx = 3; ; idx++ )
  {
    if( numRef <= idx || !m_BinDecoder.decodeBinEP() )
    {
      pu.refIdx[eRefList] = (signed char)(idx - 1);
      DTRACE( g_trace_ctx, D_SYNTAX, "ref_idx() value=%d pos=(%d,%d)\n", idx-1, pu.lumaPos().x, pu.lumaPos().y );
      return;
    }
  }
}

#if MULTI_HYP_PRED
int CABACReader::ref_idx_mh(const int numRef)
{

  if (numRef <= 1 || !m_BinDecoder.decodeBin(Ctx::MHRefPic()))
  {
    return 0;
  }
  if (numRef <= 2 || !m_BinDecoder.decodeBin(Ctx::MHRefPic(1)))
  {
    return 1;
  }
  for (int idx = 3; ; idx++)
  {
    if (numRef <= idx || !m_BinDecoder.decodeBinEP())
    {
      return(idx - 1);
    }
  }
}
#endif


void CABACReader::mvp_flag( PredictionUnit& pu, RefPicList eRefList )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__MVP_IDX, pu.lumaSize());

#if JVET_X0083_BM_AMVP_MERGE_MODE
  if (pu.amvpMergeModeFlag[eRefList])
  {
    return;
  }
#endif
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
  if (pu.amvpMergeModeFlag[1 - eRefList] == true)
  {
    unsigned mvpIdx = 0;
#if TM_AMVP
#if JVET_Y0128_NON_CTC || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && TM_AMVP)
#if JVET_Z0054_BLK_REF_PIC_REORDER
    if (pu.cs->sps->getUseARL())
    {
      RefListAndRefIdx refListComb = pu.cs->slice->getRefPicCombinedListAmvpMerge()[pu.refIdxLC];
      if (PU::checkTmEnableCondition(pu.cs->sps, pu.cs->pps, pu.cu->slice->getRefPic(refListComb.refList, refListComb.refIdx)) == false)
      {
        mvpIdx = m_BinDecoder.decodeBin(Ctx::MVPIdx());
      }
    }
    else
#endif
      if (PU::checkTmEnableCondition(pu.cs->sps, pu.cs->pps, pu.cu->slice->getRefPic(eRefList, pu.refIdx[eRefList])) == false)
#else
    if (!pu.cu->cs->sps->getUseDMVDMode() || pu.cu->affine || CU::isIBC(*pu.cu))
#endif
#endif
    {
      mvpIdx = m_BinDecoder.decodeBin(Ctx::MVPIdx());
    }
    if (mvpIdx == 0)
    {
      pu.mvpIdx [eRefList] = mvpIdx + m_BinDecoder.decodeBinEP();
    }
    else
    {
      pu.mvpIdx [eRefList] = mvpIdx + 1;
    }
    return;
  }
#endif
#if TM_AMVP
#if JVET_Y0128_NON_CTC || JVET_AA0132_CONFIGURABLE_TM_TOOLS
  unsigned mvpIdx = 0;
  if (pu.cu->affine || CU::isIBC(*pu.cu))
  {
    mvpIdx = m_BinDecoder.decodeBin( Ctx::MVPIdx() );
  }
#if JVET_Z0054_BLK_REF_PIC_REORDER
  else if (PU::useRefCombList(pu))
  {
    mvpIdx = pu.refIdxLC >= pu.cs->slice->getNumNonScaledRefPic()
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
          || !pu.cs->sps->getUseTMAmvpMode()
#else
          || !pu.cs->sps->getUseDMVDMode()
#endif
           ? m_BinDecoder.decodeBin(Ctx::MVPIdx()) : 0;
  }
  else if (PU::useRefPairList(pu))
  {
    mvpIdx = pu.refPairIdx >= pu.cs->slice->getNumNonScaledRefPicPair()
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
          || !pu.cs->sps->getUseTMAmvpMode()
#else
          || !pu.cs->sps->getUseDMVDMode()
#endif
           ? m_BinDecoder.decodeBin(Ctx::MVPIdx()) : 0;
  }
#endif
  else if (PU::checkTmEnableCondition(pu.cs->sps, pu.cs->pps, pu.cu->slice->getRefPic(eRefList, pu.refIdx[eRefList])) == false)
  {
    mvpIdx = m_BinDecoder.decodeBin( Ctx::MVPIdx() );
  }
#else
  unsigned mvpIdx = !pu.cu->cs->sps->getUseDMVDMode() || pu.cu->affine || CU::isIBC(*pu.cu) ? m_BinDecoder.decodeBin( Ctx::MVPIdx() ) : 0;
#endif
#else
  unsigned mvpIdx = m_BinDecoder.decodeBin( Ctx::MVPIdx() );
#endif
  DTRACE( g_trace_ctx, D_SYNTAX, "mvp_flag() value=%d pos=(%d,%d)\n", mvpIdx, pu.lumaPos().x, pu.lumaPos().y );
  pu.mvpIdx [eRefList] = mvpIdx;
  DTRACE( g_trace_ctx, D_SYNTAX, "mvpIdx(refList:%d)=%d\n", eRefList, mvpIdx );
}

void CABACReader::Ciip_flag(PredictionUnit& pu)
{
  if (!pu.cs->sps->getUseCiip())
  {
    pu.ciipFlag = false;
#if CIIP_PDPC
    pu.ciipPDPC = false;
#endif
    return;
  }
  if (pu.cu->skip)
  {
    pu.ciipFlag = false;
#if CIIP_PDPC
    pu.ciipPDPC = false;
#endif
    return;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MH_INTRA_FLAG, pu.lumaSize());

  pu.ciipFlag = (m_BinDecoder.decodeBin(Ctx::CiipFlag()));
  DTRACE(g_trace_ctx, D_SYNTAX, "Ciip_flag() Ciip=%d pos=(%d,%d) size=%dx%d\n", pu.ciipFlag ? 1 : 0, pu.lumaPos().x, pu.lumaPos().y, pu.lumaSize().width, pu.lumaSize().height);
#if CIIP_PDPC
  if( pu.ciipFlag )
  {
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    tm_merge_flag(pu);
#else
    if (pu.cs->slice->getSPS()->getUseCiipTmMrg())
    {
      pu.tmMergeFlag = (m_BinDecoder.decodeBin(Ctx::CiipTMMergeFlag()));
    }
#endif
#endif
    pu.ciipPDPC = (m_BinDecoder.decodeBin(Ctx::CiipFlag(1)));
  }
#else
#if JVET_X0141_CIIP_TIMD_TM && TM_MRG
  if (pu.ciipFlag && pu.cs->slice->getSPS()->getUseCiipTmMrg())
  {
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    tm_merge_flag(pu);
#else
    pu.tmMergeFlag = (m_BinDecoder.decodeBin(Ctx::CiipTMMergeFlag()));
#endif
  }
#endif
#endif
}

#if MULTI_HYP_PRED
void CABACReader::mh_pred_data(PredictionUnit& pu)
{
  CHECK(!pu.addHypData.empty(), "Multi Hyp: !pu.addHypData.empty()");
  if (!pu.cs->sps->getUseInterMultiHyp() || !pu.cs->slice->isInterB())
  {
    return;
  }
  if (pu.ciipFlag)
  {
    return;
  }
  if (pu.cu->geoFlag)
  {
    return;
  }
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
  if (pu.tmMergeFlag)
  {
    return;
  }
#endif
#if JVET_X0049_ADAPT_DMVR
  if (pu.bmMergeFlag)
  {
    return;
  }
#endif
#if !JVET_Z0083_PARSINGERROR_FIX
  if (!pu.mergeFlag && pu.cu->affine && pu.cu->imv)
  {
    return;
  }
#endif
  if (!pu.mergeFlag && pu.cu->BcwIdx == BCW_DEFAULT)
  {
    return;
  }
  if( CU::isIBC( *pu.cu ) )
  {
    return;
  }

  if (pu.Y().area() <= MULTI_HYP_PRED_RESTRICT_BLOCK_SIZE || std::min(pu.Y().width, pu.Y().height) < MULTI_HYP_PRED_RESTRICT_MIN_WH)
  {
    return;
  }

#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatisticsClassType ctype_mh(STATS__CABAC_BITS__MH_INTER, pu.lumaSize().width, pu.lumaSize().height);
  CodingStatisticsClassType ctype_mvpIdx(STATS__CABAC_BITS__MVP_IDX, pu.lumaSize().width, pu.lumaSize().height);
  CodingStatisticsClassType ctype_mergeIdx(STATS__CABAC_BITS__MERGE_INDEX, pu.lumaSize().width, pu.lumaSize().height);
  CodingStatisticsClassType ctype_refIdx(STATS__CABAC_BITS__REF_FRM_IDX, pu.lumaSize().width, pu.lumaSize().height);
  CodingStatisticsClassType ctype_mvd(STATS__CABAC_BITS__MVD, pu.lumaSize().width, pu.lumaSize().height);
#endif
  RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_mh);

  const int numMHRef = pu.cs->slice->getNumMultiHypRefPics();
  CHECK(numMHRef <= 0, "Multi Hyp: numMHRef <= 0");
  int hypIdx = 0;
  const size_t maxNumAddHyps = pu.cs->sps->getMaxNumAddHyps();
  while (pu.addHypData.size() < maxNumAddHyps && m_BinDecoder.decodeBin(Ctx::MultiHypothesisFlag(hypIdx)) == 1)
  {
    if (hypIdx < 1)
    {
      hypIdx++;
    }
    MultiHypPredictionData mhData;
    RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_mh);
#if JVET_Z0127_SPS_MHP_MAX_MRG_CAND
    const int maxNumMHPCand = pu.cs->sps->getMaxNumMHPCand();
    if (maxNumMHPCand > 0 && m_BinDecoder.decodeBin(Ctx::MultiHypothesisFlag(2)))
    {
      mhData.isMrg = true;
      int numCandminus2 = maxNumMHPCand - 2;
#else
    if (m_BinDecoder.decodeBin(Ctx::MultiHypothesisFlag(2)))
    {
      mhData.isMrg = true;
      const int maxNumGeoCand = pu.cs->sps->getMaxNumGeoCand();
      CHECK(maxNumGeoCand < 2, "Incorrect max number of geo candidates");
      int numCandminus2 = maxNumGeoCand - 2;
#endif
      int mergeCand0 = 0;
      RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_mergeIdx);
      if (m_BinDecoder.decodeBin(Ctx::MergeIdx()))
      {
        mergeCand0 += unary_max_eqprob(numCandminus2) + 1;
      }
      mhData.mrgIdx = mergeCand0;
      mhData.weightIdx = unary_max_symbol(Ctx::MHWeight(), Ctx::MHWeight(1), pu.cs->sps->getNumAddHypWeights() - 1);
      pu.addHypData.push_back(mhData);
      continue;
    }
    mhData.isMrg = false;

    RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_refIdx);
    mhData.refIdx = ref_idx_mh(numMHRef);
    CHECK(mhData.refIdx < 0, "Multi Hyp: mhData.refIdx < 0");
    CHECK(mhData.refIdx >= numMHRef, "Multi Hyp: mhData.refIdx >= numMHRef");
    RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_mvd);
    mvd_coding(mhData.mvd);
    RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_mvpIdx);
    mhData.mvpIdx = m_BinDecoder.decodeBin(Ctx::MVPIdx());
    mhData.weightIdx = unary_max_symbol(Ctx::MHWeight(), Ctx::MHWeight(1), pu.cs->sps->getNumAddHypWeights() - 1);
    pu.addHypData.push_back(mhData);
  }
}
#endif



//================================================================================
//  clause 7.3.8.8
//--------------------------------------------------------------------------------
//    void  transform_tree      ( cs, area, cuCtx, chromaCbfs )
//    bool  split_transform_flag( depth )
//    bool  cbf_comp            ( area, depth )
//================================================================================

void CABACReader::transform_tree( CodingStructure &cs, Partitioner &partitioner, CUCtx& cuCtx,                         const PartSplit ispType, const int subTuIdx )
{
  const UnitArea&   area = partitioner.currArea();
  CodingUnit&         cu = *cs.getCU(area.blocks[partitioner.chType], partitioner.chType);
  int       subTuCounter = subTuIdx;

  // split_transform_flag
  bool split = partitioner.canSplit(TU_MAX_TR_SPLIT, cs);
  const unsigned  trDepth = partitioner.currTrDepth;

  if( cu.sbtInfo && partitioner.canSplit( PartSplit( cu.getSbtTuSplit() ), cs ) )
  {
    split = true;
  }

  if( !split && cu.ispMode )
  {
    split = partitioner.canSplit( ispType, cs );
  }

  if( split )
  {
    if (partitioner.canSplit(TU_MAX_TR_SPLIT, cs))
    {
#if ENABLE_TRACING
      const CompArea &tuArea = partitioner.currArea().blocks[partitioner.chType];
      DTRACE(g_trace_ctx, D_SYNTAX, "transform_tree() maxTrSplit chType=%d pos=(%d,%d) size=%dx%d\n",
             partitioner.chType, tuArea.x, tuArea.y, tuArea.width, tuArea.height);

#endif
      partitioner.splitCurrArea(TU_MAX_TR_SPLIT, cs);
    }
    else if (cu.ispMode)
    {
      partitioner.splitCurrArea(ispType, cs);
    }
    else if (cu.sbtInfo && partitioner.canSplit(PartSplit(cu.getSbtTuSplit()), cs))
    {
      partitioner.splitCurrArea(PartSplit(cu.getSbtTuSplit()), cs);
    }
    else
    {
      THROW("Implicit TU split not available!");
    }

    do
    {
      transform_tree( cs, partitioner, cuCtx,          ispType, subTuCounter );
      subTuCounter += subTuCounter != -1 ? 1 : 0;
    } while( partitioner.nextPart( cs ) );

    partitioner.exitCurrSplit();
  }
  else
  {
    TransformUnit &tu = cs.addTU( CS::getArea( cs, area, partitioner.chType ), partitioner.chType );
    unsigned numBlocks = ::getNumberValidTBlocks( *cs.pcv );
    tu.checkTuNoResidual( partitioner.currPartIdx() );

    for( unsigned compID = COMPONENT_Y; compID < numBlocks; compID++ )
    {
      if( tu.blocks[compID].valid() )
      {
        tu.getCoeffs( ComponentID( compID ) ).fill( 0 );
#if !REMOVE_PCM
        tu.getPcmbuf( ComponentID( compID ) ).fill( 0 );
#endif
      }
    }
    tu.depth = trDepth;
    DTRACE( g_trace_ctx, D_SYNTAX, "transform_unit() pos=(%d,%d) size=%dx%d depth=%d trDepth=%d\n", tu.blocks[tu.chType].x, tu.blocks[tu.chType].y, tu.blocks[tu.chType].width, tu.blocks[tu.chType].height, cu.depth, partitioner.currTrDepth );

    transform_unit(tu, cuCtx, partitioner, subTuCounter);
  }
}

bool CABACReader::cbf_comp( CodingStructure& cs, const CompArea& area, unsigned depth, const bool prevCbf, const bool useISP )
{
  unsigned  ctxId = DeriveCtx::CtxQtCbf(area.compID, prevCbf, useISP && isLuma(area.compID));
  const CtxSet&   ctxSet  = Ctx::QtCbf[ area.compID ];

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__QT_CBF, area.size(), area.compID);

  unsigned  cbf = 0;
  if( (area.compID == COMPONENT_Y && cs.getCU(area.pos(), toChannelType(area.compID))->bdpcmMode)
   || (area.compID != COMPONENT_Y && cs.getCU(area.pos(), toChannelType(area.compID))->bdpcmModeChroma))
  {
    if (area.compID == COMPONENT_Y)
    {
      ctxId = 1;
    }
    else if (area.compID == COMPONENT_Cb)
    {
      ctxId = 1;
    }
    else
    {
      ctxId = 2;
    }
    cbf = m_BinDecoder.decodeBin(ctxSet(ctxId));
  }
  else
  {
    cbf = m_BinDecoder.decodeBin( ctxSet( ctxId ) );
  }

  DTRACE( g_trace_ctx, D_SYNTAX, "cbf_comp() etype=%d pos=(%d,%d) ctx=%d cbf=%d\n", area.compID, area.x, area.y, ctxId, cbf );
  return cbf;
}

//================================================================================
//  clause 7.3.8.9
//--------------------------------------------------------------------------------
//    void  mvd_coding( pu, refList )
//================================================================================
#if JVET_AA0070_RRIBC
void CABACReader::mvd_coding( Mv &rMvd
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  , bool codeSign
#endif
  , const int &rribcFlipType
)
{
  // abs_mvd_greater0_flag[ 0 | 1 ]
  int horAbs = 0, verAbs = 0;
  if (rribcFlipType != 2)
  {
    horAbs = (int) m_BinDecoder.decodeBin(Ctx::Mvd());
  }
  if (rribcFlipType != 1)
  {
    verAbs = (int) m_BinDecoder.decodeBin(Ctx::Mvd());
  }
#else
void CABACReader::mvd_coding( Mv &rMvd
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  , bool codeSign
#endif
)
{
  // abs_mvd_greater0_flag[ 0 | 1 ]
  int horAbs = (int)m_BinDecoder.decodeBin(Ctx::Mvd());
  int verAbs = (int)m_BinDecoder.decodeBin(Ctx::Mvd());
#endif

  // abs_mvd_greater1_flag[ 0 | 1 ]
  if (horAbs)
  {
    horAbs += (int)m_BinDecoder.decodeBin(Ctx::Mvd(1));
  }
  if (verAbs)
  {
    verAbs += (int)m_BinDecoder.decodeBin(Ctx::Mvd(1));
  }

  // abs_mvd_minus2[ 0 | 1 ] and mvd_sign_flag[ 0 | 1 ]
  if (horAbs)
  {
    if (horAbs > 1)
    {
      horAbs += m_BinDecoder.decodeRemAbsEP(1, 0, MV_BITS - 1);
    }
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    if (codeSign)
#endif
    if (m_BinDecoder.decodeBinEP())
    {
      horAbs = -horAbs;
    }
  }
  if (verAbs)
  {
    if (verAbs > 1)
    {
      verAbs += m_BinDecoder.decodeRemAbsEP(1, 0, MV_BITS - 1);
    }
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    if (codeSign)
#endif
    if (m_BinDecoder.decodeBinEP())
    {
      verAbs = -verAbs;
    }
  }
  rMvd = Mv(horAbs, verAbs);
  CHECK(!((horAbs >= MVD_MIN) && (horAbs <= MVD_MAX)) || !((verAbs >= MVD_MIN) && (verAbs <= MVD_MAX)), "Illegal MVD value");
}

#if JVET_Z0131_IBC_BVD_BINARIZATION
unsigned CABACReader::xReadBvdContext(unsigned ctxT, int offset, int param)
{
  unsigned symbol = 0;
  unsigned bit    = 1;
  unsigned uiIdx = 0;
  while( bit )
  {
    if (uiIdx >= ctxT)
    {
      bit = m_BinDecoder.decodeBinEP();
    }
    else
    {
      bit = m_BinDecoder.decodeBin(Ctx::Bvd( offset + uiIdx + 1));
    }
    uiIdx++;
    symbol += bit << param++;
  }
  if( --param )
  {
    bit = m_BinDecoder.decodeBinsEP(param);
    symbol += bit;
  }
  return symbol;
}
#endif

#if JVET_Z0131_IBC_BVD_BINARIZATION
#if JVET_AA0070_RRIBC
void CABACReader::bvdCoding(Mv &rMvd, const int &rribcFlipType)
{
  int horAbs = 0, verAbs = 0;
  if (rribcFlipType != 2)
  {
    horAbs = (int) m_BinDecoder.decodeBin(Ctx::Bvd(HOR_BVD_CTX_OFFSET));
  }
  if (rribcFlipType != 1)
  {
    verAbs = (int) m_BinDecoder.decodeBin(Ctx::Bvd(VER_BVD_CTX_OFFSET));
  }
#else
void CABACReader::bvdCoding( Mv &rMvd )
{
  int horAbs = (int)m_BinDecoder.decodeBin(Ctx::Bvd(HOR_BVD_CTX_OFFSET));
  int verAbs = (int)m_BinDecoder.decodeBin(Ctx::Bvd(VER_BVD_CTX_OFFSET));
#endif

  if (horAbs)
  {
    horAbs += xReadBvdContext(NUM_HOR_BVD_CTX, HOR_BVD_CTX_OFFSET, BVD_CODING_GOLOMB_ORDER);

    if (m_BinDecoder.decodeBinEP())
    {
      horAbs = -horAbs;
    }
  }
  if (verAbs)
  {
    verAbs += xReadBvdContext(NUM_VER_BVD_CTX, VER_BVD_CTX_OFFSET, BVD_CODING_GOLOMB_ORDER);

    if (m_BinDecoder.decodeBinEP())
    {
      verAbs = -verAbs;
    }
  }
  rMvd = Mv(horAbs, verAbs);
  CHECK(!((horAbs >= MVD_MIN) && (horAbs <= MVD_MAX)) || !((verAbs >= MVD_MIN) && (verAbs <= MVD_MAX)), "Illegal BVD value");
}
#endif

#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
void CABACReader::mvsdIdxFunc(PredictionUnit &pu, RefPicList eRefList)
{
  if (!pu.isMvsdApplicable())
  {
    return;
  }
  
  if (pu.cu->cs->picHeader->getMvdL1ZeroFlag() && eRefList == REF_PIC_LIST_1 && pu.interDir == 3)
  {
    return;
  }
  if (pu.cu->smvdMode && eRefList == REF_PIC_LIST_1)
  {
    return;
  }
#if JVET_Z0054_BLK_REF_PIC_REORDER
  if (PU::useRefPairList(pu))
  {
    if (pu.interDir == 3 && eRefList == REF_PIC_LIST_1 && (pu.mvd[0].getHor() || pu.mvd[0].getVer()))
    {
      if (pu.mvd[eRefList].getHor())
      {
        if (m_BinDecoder.decodeBinEP())
        {
          pu.mvd[eRefList].setHor(-pu.mvd[eRefList].getHor());
        }
      }
      if (pu.mvd[eRefList].getVer())
      {
        if (m_BinDecoder.decodeBinEP())
        {
          pu.mvd[eRefList].setVer(-pu.mvd[eRefList].getVer());
        }
      }
      return;
    }
  }
#endif
  
  Mv TrMv = pu.mvd[eRefList].getAbsMv();
  TrMv.changeTransPrecAmvr2Internal(pu.cu->imv);
  int Thres = THRES_TRANS;
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MVS, pu.lumaSize());
  
  int mvsdIdx = 0;
  int shift = 0;
  if (pu.mvd[eRefList].getHor())
  {
    uint8_t ctxId = (TrMv.getHor() <= Thres) ? 0 : 1;
    uint8_t bin = m_BinDecoder.decodeBin(Ctx::MvsdIdx(ctxId));
    mvsdIdx += (bin << shift);
    shift++;
  }
  if (pu.mvd[eRefList].getVer())
  {
    uint8_t ctxId = (TrMv.getVer() <= Thres) ? 0 : 1;
    uint8_t bin = m_BinDecoder.decodeBin(Ctx::MvsdIdx(ctxId));
    mvsdIdx += (bin << shift);
    shift++;
  }
  pu.mvsdIdx[eRefList] = mvsdIdx;
}
void CABACReader::mvsdAffineIdxFunc(PredictionUnit &pu, RefPicList eRefList)
{
  if (!pu.cu->affine)
  {
    return;
  }
  if (!pu.isMvsdApplicable())
  {
    return;
  }
  
  if (pu.cu->cs->picHeader->getMvdL1ZeroFlag() && eRefList == REF_PIC_LIST_1 && pu.interDir == 3)
  {
    return;
  }

#if JVET_Z0054_BLK_REF_PIC_REORDER
  if (PU::useRefPairList(pu))
  {
    if (pu.interDir == 3 && eRefList == REF_PIC_LIST_1 &&
      (pu.mvdAffi[0][0].getHor() || pu.mvdAffi[0][0].getVer() ||
        pu.mvdAffi[0][1].getHor() || pu.mvdAffi[0][1].getVer() ||
        (pu.cu->affineType == AFFINEMODEL_6PARAM && (pu.mvdAffi[0][2].getHor() || pu.mvdAffi[0][2].getVer()))
        )
      )
    {
      if (pu.mvdAffi[eRefList][0].getHor())
      {
        if (m_BinDecoder.decodeBinEP())
        {
          pu.mvdAffi[eRefList][0].setHor(-pu.mvdAffi[eRefList][0].getHor());
        }
      }
      if (pu.mvdAffi[eRefList][0].getVer())
      {
        if (m_BinDecoder.decodeBinEP())
        {
          pu.mvdAffi[eRefList][0].setVer(-pu.mvdAffi[eRefList][0].getVer());
        }
      }
      if (pu.mvdAffi[eRefList][1].getHor())
      {
        if (m_BinDecoder.decodeBinEP())
        {
          pu.mvdAffi[eRefList][1].setHor(-pu.mvdAffi[eRefList][1].getHor());
        }
      }
      if (pu.mvdAffi[eRefList][1].getVer())
      {
        if (m_BinDecoder.decodeBinEP())
        {
          pu.mvdAffi[eRefList][1].setVer(-pu.mvdAffi[eRefList][1].getVer());
        }
      }

      if (pu.cu->affineType == AFFINEMODEL_6PARAM)
      {
        if (pu.mvdAffi[eRefList][2].getHor())
        {
          if (m_BinDecoder.decodeBinEP())
          {
            pu.mvdAffi[eRefList][2].setHor(-pu.mvdAffi[eRefList][2].getHor());
          }
        }
        if (pu.mvdAffi[eRefList][2].getVer())
        {
          if (m_BinDecoder.decodeBinEP())
          {
            pu.mvdAffi[eRefList][2].setVer(-pu.mvdAffi[eRefList][2].getVer());
          }
        }
      }
      return;
    }
  }
#endif
  
  Mv AffMv[3];
  AffMv[0] = pu.mvdAffi[eRefList][0].getAbsMv();
  AffMv[1] = pu.mvdAffi[eRefList][1].getAbsMv();
  AffMv[2] = pu.mvdAffi[eRefList][2].getAbsMv();
  AffMv[0].changeAffinePrecAmvr2Internal(pu.cu->imv);
  AffMv[1].changeAffinePrecAmvr2Internal(pu.cu->imv);
  AffMv[2].changeAffinePrecAmvr2Internal(pu.cu->imv);
  int Thres = THRES_AFFINE;
  
  int mvsdIdx = 0;
  int shift = 0;
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__MVS, pu.lumaSize());
  
  if (pu.mvdAffi[eRefList][0].getHor())
  {
    uint8_t ctxId = (AffMv[0].getHor() <= Thres) ? 2 : 3;
    uint8_t bin = m_BinDecoder.decodeBin(Ctx::MvsdIdx(ctxId));
    mvsdIdx += (bin << shift);
    shift++;
  }
  if (pu.mvdAffi[eRefList][0].getVer())
  {
    uint8_t ctxId = (AffMv[0].getVer() <= Thres) ? 2 : 3;
    uint8_t bin = m_BinDecoder.decodeBin(Ctx::MvsdIdx(ctxId));
    mvsdIdx += (bin << shift);
    shift++;
  }
  if (pu.mvdAffi[eRefList][1].getHor())
  {
    uint8_t ctxId = (AffMv[1].getHor() <= Thres) ? 2 : 3;
    uint8_t bin = m_BinDecoder.decodeBin(Ctx::MvsdIdx(ctxId));
    mvsdIdx += (bin << shift);
    shift++;
  }
  if (pu.mvdAffi[eRefList][1].getVer())
  {
    uint8_t ctxId = (AffMv[1].getVer() <= Thres) ? 2 : 3;
    uint8_t bin = m_BinDecoder.decodeBin(Ctx::MvsdIdx(ctxId));
    mvsdIdx += (bin << shift);
    shift++;
  }
  if (pu.cu->affineType == AFFINEMODEL_6PARAM)
  {
    if (pu.mvdAffi[eRefList][2].getHor())
    {
      uint8_t ctxId = (AffMv[2].getHor() <= Thres) ? 2 : 3;
      uint8_t bin = m_BinDecoder.decodeBin(Ctx::MvsdIdx(ctxId));
      mvsdIdx += (bin << shift);
      shift++;
    }
    if (pu.mvdAffi[eRefList][2].getVer())
    {
      uint8_t ctxId = (AffMv[2].getVer() <= Thres) ? 2 : 3;
      uint8_t bin = m_BinDecoder.decodeBin(Ctx::MvsdIdx(ctxId));
      mvsdIdx += (bin << shift);
      shift++;
    }
  }
  pu.mvsdIdx[eRefList] = mvsdIdx;
}
#endif
//================================================================================
//  clause 7.3.8.10
//--------------------------------------------------------------------------------
//    void  transform_unit      ( tu, cuCtx, chromaCbfs )
//    void  cu_qp_delta         ( cu )
//    void  cu_chroma_qp_offset ( cu )
//================================================================================
void CABACReader::transform_unit( TransformUnit& tu, CUCtx& cuCtx, Partitioner& partitioner, const int subTuCounter)
{
  const UnitArea&         area = partitioner.currArea();
  const unsigned          trDepth = partitioner.currTrDepth;

  CodingStructure&  cs = *tu.cs;
  CodingUnit&       cu = *tu.cu;
  ChromaCbfs        chromaCbfs;
  chromaCbfs.Cb = chromaCbfs.Cr = false;

  const bool chromaCbfISP = area.chromaFormat != CHROMA_400 && area.blocks[COMPONENT_Cb].valid() && cu.ispMode;

  // cbf_cb & cbf_cr
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (area.chromaFormat != CHROMA_400 && area.blocks[COMPONENT_Cb].valid() && (!CS::isDualITree(cs) || partitioner.chType == CHANNEL_TYPE_CHROMA) && (!cu.ispMode || chromaCbfISP))
#else
  if (area.chromaFormat != CHROMA_400 && area.blocks[COMPONENT_Cb].valid() && (!cu.isSepTree() || partitioner.chType == CHANNEL_TYPE_CHROMA) && (!cu.ispMode || chromaCbfISP))
#endif
  {
    const int cbfDepth = chromaCbfISP ? trDepth - 1 : trDepth;
    if (!(cu.sbtInfo && tu.noResidual))
    {
      chromaCbfs.Cb = cbf_comp(cs, area.blocks[COMPONENT_Cb], cbfDepth);
    }

    if (!(cu.sbtInfo && tu.noResidual))
    {
      chromaCbfs.Cr = cbf_comp(cs, area.blocks[COMPONENT_Cr], cbfDepth, chromaCbfs.Cb);
    }
  }
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  else if (CS::isDualITree(cs))
#else
  else if (cu.isSepTree())
#endif
  {
    chromaCbfs = ChromaCbfs(false);
  }

  if (!isChroma(partitioner.chType))
  {
    if (!CU::isIntra(cu) && trDepth == 0 && !chromaCbfs.sigChroma(area.chromaFormat))
    {
      TU::setCbfAtDepth(tu, COMPONENT_Y, trDepth, 1);
    }
    else if (cu.sbtInfo && tu.noResidual)
    {
      TU::setCbfAtDepth(tu, COMPONENT_Y, trDepth, 0);
    }
    else if (cu.sbtInfo && !chromaCbfs.sigChroma(area.chromaFormat))
    {
      assert(!tu.noResidual);
      TU::setCbfAtDepth(tu, COMPONENT_Y, trDepth, 1);
    }
    else
    {
      bool lumaCbfIsInferredACT = (cu.colorTransform && cu.predMode == MODE_INTRA && trDepth == 0 && !chromaCbfs.sigChroma(area.chromaFormat));
      bool lastCbfIsInferred    = lumaCbfIsInferredACT; // ISP and ACT are mutually exclusive
      bool previousCbf          = false;
      bool rootCbfSoFar         = false;
      if (cu.ispMode)
      {
        uint32_t nTus = cu.ispMode == HOR_INTRA_SUBPARTITIONS ? cu.lheight() >> floorLog2(tu.lheight()) : cu.lwidth() >> floorLog2(tu.lwidth());
        if (subTuCounter == nTus - 1)
        {
          TransformUnit* tuPointer = cu.firstTU;
          for (int tuIdx = 0; tuIdx < nTus - 1; tuIdx++)
          {
            rootCbfSoFar |= TU::getCbfAtDepth(*tuPointer, COMPONENT_Y, trDepth);
            tuPointer = tuPointer->next;
          }
          if (!rootCbfSoFar)
          {
            lastCbfIsInferred = true;
          }
        }
        if (!lastCbfIsInferred)
        {
          previousCbf = TU::getPrevTuCbfAtDepth(tu, COMPONENT_Y, trDepth);
        }
      }
      bool cbfY = lastCbfIsInferred ? true : cbf_comp(cs, tu.Y(), trDepth, previousCbf, cu.ispMode);
      TU::setCbfAtDepth(tu, COMPONENT_Y, trDepth, (cbfY ? 1 : 0));
    }
  }
  if (area.chromaFormat != CHROMA_400 && (!cu.ispMode || chromaCbfISP))
  {
    TU::setCbfAtDepth(tu, COMPONENT_Cb, trDepth, (chromaCbfs.Cb ? 1 : 0));
    TU::setCbfAtDepth(tu, COMPONENT_Cr, trDepth, (chromaCbfs.Cr ? 1 : 0));
  }
  bool        lumaOnly   = ( cu.chromaFormat == CHROMA_400 || !tu.blocks[COMPONENT_Cb].valid() );
  bool        cbfLuma    = ( tu.cbf[ COMPONENT_Y ] != 0 );
  bool        cbfChroma  = ( lumaOnly ? false : ( chromaCbfs.Cb || chromaCbfs.Cr ) );

#if TU_256
  if( ( cu.lwidth() > MAX_TB_SIZEY || cu.lheight() > MAX_TB_SIZEY || cbfLuma || cbfChroma ) &&
#else
  if( ( cu.lwidth() > 64 || cu.lheight() > 64 || cbfLuma || cbfChroma ) &&
#endif
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    (!CS::isDualITree(*tu.cs) || isLuma(tu.chType)))
#else
    (!tu.cu->isSepTree() || isLuma(tu.chType)) )
#endif
  {
    if( cu.cs->pps->getUseDQP() && !cuCtx.isDQPCoded )
    {
      cu_qp_delta(cu, cuCtx.qp, cu.qp);
      cuCtx.qp = cu.qp;
      cuCtx.isDQPCoded = true;
    }
  }
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (!CS::isDualITree(*tu.cs) || isChroma(tu.chType))   // !DUAL_TREE_LUMA
#else
  if (!cu.isSepTree() || isChroma(tu.chType))   // !DUAL_TREE_LUMA
#endif
  {
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    SizeType channelWidth = !CS::isDualITree(*tu.cs) ? cu.lwidth() : cu.chromaSize().width;
    SizeType channelHeight = !CS::isDualITree(*tu.cs) ? cu.lheight() : cu.chromaSize().height;
#else
    SizeType channelWidth = !cu.isSepTree() ? cu.lwidth() : cu.chromaSize().width;
    SizeType channelHeight = !cu.isSepTree() ? cu.lheight() : cu.chromaSize().height;
#endif
#if TU_256
    if( cu.cs->slice->getUseChromaQpAdj() && ( channelWidth > MAX_TB_SIZEY || channelHeight > MAX_TB_SIZEY || cbfChroma ) && !cuCtx.isChromaQpAdjCoded )
#else
    if (cu.cs->slice->getUseChromaQpAdj() && (channelWidth > 64 || channelHeight > 64 || cbfChroma) && !cuCtx.isChromaQpAdjCoded)
#endif
    {
      cu_chroma_qp_offset(cu);
      cuCtx.isChromaQpAdjCoded = true;
    }
  }

  if( !lumaOnly )
  {
    joint_cb_cr( tu, ( tu.cbf[COMPONENT_Cb] ? 2 : 0 ) + ( tu.cbf[COMPONENT_Cr] ? 1 : 0 ) );
  }

  if (cbfLuma)
  {
    residual_coding(tu, COMPONENT_Y, cuCtx);
  }
  if (!lumaOnly)
  {
    for (ComponentID compID = COMPONENT_Cb; compID <= COMPONENT_Cr; compID = ComponentID(compID + 1))
    {
      if (tu.cbf[compID])
      {
        residual_coding(tu, compID, cuCtx);
      }
    }
  }
}

void CABACReader::cu_qp_delta( CodingUnit& cu, int predQP, int8_t& qp )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__DELTA_QP_EP, cu.lumaSize());

  CHECK( predQP == std::numeric_limits<int>::max(), "Invalid predicted QP" );
  int qpY = predQP;
  int DQp = unary_max_symbol( Ctx::DeltaQP(), Ctx::DeltaQP(1), CU_DQP_TU_CMAX );
  if( DQp >= CU_DQP_TU_CMAX )
  {
    DQp += exp_golomb_eqprob( CU_DQP_EG_k  );
  }
  if( DQp > 0 )
  {
    if( m_BinDecoder.decodeBinEP( ) )
    {
      DQp = -DQp;
    }
    int     qpBdOffsetY = cu.cs->sps->getQpBDOffset( CHANNEL_TYPE_LUMA );
    qpY = ( (predQP + DQp + (MAX_QP + 1) + 2 * qpBdOffsetY) % ((MAX_QP + 1) + qpBdOffsetY)) - qpBdOffsetY;
  }
  qp = (int8_t)qpY;

  DTRACE( g_trace_ctx, D_DQP, "x=%d, y=%d, d=%d, pred_qp=%d, DQp=%d, qp=%d\n", cu.blocks[cu.chType].lumaPos().x, cu.blocks[cu.chType].lumaPos().y, cu.qtDepth, predQP, DQp, qp );
}


void CABACReader::cu_chroma_qp_offset( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__CHROMA_QP_ADJUSTMENT, cu.blocks[cu.chType].lumaSize(), CHANNEL_TYPE_CHROMA );

  // cu_chroma_qp_offset_flag
  int       length  = cu.cs->pps->getChromaQpOffsetListLen();
  unsigned  qpAdj   = m_BinDecoder.decodeBin( Ctx::ChromaQpAdjFlag() );
  if( qpAdj && length > 1 )
  {
    // cu_chroma_qp_offset_idx
    qpAdj += unary_max_symbol( Ctx::ChromaQpAdjIdc(), Ctx::ChromaQpAdjIdc(), length-1 );
  }
  /* NB, symbol = 0 if outer flag is not set,
   *              1 if outer flag is set and there is no inner flag
   *              1+ otherwise */
  cu.chromaQpAdj = cu.cs->chromaQpAdj = qpAdj;
}

//================================================================================
//  clause 7.3.8.11
//--------------------------------------------------------------------------------
//    void        residual_coding         ( tu, compID )
//    bool        transform_skip_flag     ( tu, compID )
//    int         last_sig_coeff          ( coeffCtx )
//    void        residual_coding_subblock( coeffCtx )
//================================================================================

void CABACReader::joint_cb_cr( TransformUnit& tu, const int cbfMask )
{
  if ( !tu.cu->slice->getSPS()->getJointCbCrEnabledFlag() )
  {
    return;
  }

  if( ( CU::isIntra( *tu.cu ) && cbfMask ) || ( cbfMask == 3 ) )
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__JOINT_CB_CR, tu.blocks[COMPONENT_Cr].lumaSize(), CHANNEL_TYPE_CHROMA );
    tu.jointCbCr = ( m_BinDecoder.decodeBin( Ctx::JointCbCrFlag( cbfMask-1 ) ) ? cbfMask : 0 );
  }
}

void CABACReader::residual_coding( TransformUnit& tu, ComponentID compID, CUCtx& cuCtx )
{
  const CodingUnit& cu = *tu.cu;
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_coding() etype=%d pos=(%d,%d) size=%dx%d predMode=%d\n", tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height, cu.predMode );

  if( compID == COMPONENT_Cr && tu.jointCbCr == 3 )
  {
    return;
  }

  ts_flag            ( tu, compID );

  if( tu.mtsIdx[compID] == MTS_SKIP && !tu.cs->slice->getTSResidualCodingDisabledFlag() )
  {
    residual_codingTS( tu, compID );
    return;
  }

  // determine sign hiding
  bool signHiding = cu.cs->slice->getSignDataHidingEnabledFlag();

  // init coeff coding context
  CoeffCodingContext  cctx    ( tu, compID, signHiding );
  TCoeff*             coeff   = tu.getCoeffs( compID ).buf;

  // parse last coeff position
  cctx.setScanPosLast( last_sig_coeff( cctx, tu, compID ) );

#if !EXTENDED_LFNST
  if (tu.mtsIdx[compID] != MTS_SKIP && tu.blocks[compID].height >= 4 && tu.blocks[compID].width >= 4 )
  {
#if JVET_W0119_LFNST_EXTENSION
    const int maxLfnstPos = PU::getLFNSTMatrixDim( tu.blocks[ compID ].width, tu.blocks[ compID ].height ) - 1;
#else
    const int maxLfnstPos = ((tu.blocks[compID].height == 4 && tu.blocks[compID].width == 4) || (tu.blocks[compID].height == 8 && tu.blocks[compID].width == 8)) ? 7 : 15;
#endif
    cuCtx.violatesLfnstConstrained[ toChannelType(compID) ] |= cctx.scanPosLast() > maxLfnstPos;
  }
#endif

  if( tu.mtsIdx[compID] != MTS_SKIP && tu.blocks[compID].height >= 4 && tu.blocks[compID].width >= 4 )
  {
    const int lfnstLastScanPosTh = isLuma( compID ) ? LFNST_LAST_SIG_LUMA : LFNST_LAST_SIG_CHROMA;
    cuCtx.lfnstLastScanPos |= cctx.scanPosLast() >= lfnstLastScanPosTh;
  }
  if (isLuma(compID) && tu.mtsIdx[compID] != MTS_SKIP)
  {
    cuCtx.mtsLastScanPos |= cctx.scanPosLast() >= 1;
  }

#if SIGN_PREDICTION
  CoeffBuf signBuff = tu.getCoeffSigns(compID);
  tu.initSignBuffers( compID );
#endif

  // parse subblocks
#if TCQ_8STATES
  const uint64_t stateTransTab = g_stateTransTab[ tu.cs->slice->getDepQuantEnabledIdc() ];
#else
  const int stateTransTab = ( tu.cs->slice->getDepQuantEnabledFlag() ? 32040 : 0 );
#endif
  int       state         = 0;

  int ctxBinSampleRatio = (compID == COMPONENT_Y) ? MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_LUMA : MAX_TU_LEVEL_CTX_CODED_BIN_CONSTRAINT_CHROMA;
  cctx.regBinLimit = (tu.getTbAreaAfterCoefZeroOut(compID) * ctxBinSampleRatio) >> 4;

    for( int subSetId = ( cctx.scanPosLast() >> cctx.log2CGSize() ); subSetId >= 0; subSetId--)
    {
      cctx.initSubblock       ( subSetId );

#if !TU_256
      if( tu.cs->sps->getUseMTS() && tu.cu->sbtInfo != 0 && tu.blocks[ compID ].height <= 32 && tu.blocks[ compID ].width <= 32 && compID == COMPONENT_Y )
      {
        if( ( tu.blocks[ compID ].height == 32 && cctx.cgPosY() >= ( 16 >> cctx.log2CGHeight() ) ) || ( tu.blocks[ compID ].width == 32 && cctx.cgPosX() >= ( 16 >> cctx.log2CGWidth() ) ) )
        {
          continue;
        }
      }
#endif

#if SIGN_PREDICTION
      residual_coding_subblock( cctx, coeff, signBuff.buf, stateTransTab, state );
#else
      residual_coding_subblock( cctx, coeff, stateTransTab, state );
#endif

#if EXTENDED_LFNST
      if ( tu.blocks[compID].width >= 4 && tu.blocks[compID].height >= 4 )
      {
        const bool whge3 = tu.blocks[compID].width >= 8 && tu.blocks[compID].height >= 8;
        const bool isLfnstViolated = whge3 ? (cctx.isSigGroup() && (cctx.cgPosY() > 1 || cctx.cgPosX() > 1)) : (cctx.isSigGroup() && (cctx.cgPosY() > 0 || cctx.cgPosX() > 0));
        cuCtx.violatesLfnstConstrained[toChannelType(compID)] |= isLfnstViolated;
      }
#endif
#if !TU_256
      if ( isLuma(compID) && cctx.isSigGroup() && ( cctx.cgPosY() > 3 || cctx.cgPosX() > 3 ) )
      {
        cuCtx.violatesMtsCoeffConstraint = true;
      }
#endif
    }
#if JVET_Y0142_ADAPT_INTRA_MTS
    if (isLuma(compID) && tu.mtsIdx[compID] != MTS_SKIP)
    {
      const int  coeffStride = tu.getCoeffs(compID).stride;
      const int  uiWidth = tu.getCoeffs(compID).width;
      const int  uiHeight = tu.getCoeffs(compID).height;
      uint64_t coeffAbsSum = 0;

      for (int y = 0; y < uiHeight; y++)
      {
        for (int x = 0; x < uiWidth; x++)
        {
          coeffAbsSum += abs(coeff[(y * coeffStride) + x]);
        }
      }
      cuCtx.mtsCoeffAbsSum = (int64_t)coeffAbsSum;
    }
#endif
}

void CABACReader::ts_flag( TransformUnit& tu, ComponentID compID )
{
  int tsFlag = ( (tu.cu->bdpcmMode && isLuma(compID)) || (tu.cu->bdpcmModeChroma && isChroma(compID)) ) ? 1 : tu.mtsIdx[compID] == MTS_SKIP ? 1 : 0;
  int ctxIdx = isLuma(compID) ? 0 : 1;

  if( TU::isTSAllowed ( tu, compID ) )
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__MTS_FLAGS, tu.blocks[compID], compID );
    tsFlag = m_BinDecoder.decodeBin( Ctx::TransformSkipFlag( ctxIdx ) );
  }

  tu.mtsIdx[compID] = tsFlag ? MTS_SKIP : MTS_DCT2_DCT2;

  DTRACE(g_trace_ctx, D_SYNTAX, "ts_flag() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMPONENT_Y, tu.cu->lx(), tu.cu->ly(), tsFlag);
}

void CABACReader::mts_idx( CodingUnit& cu, CUCtx& cuCtx )
{
  TransformUnit &tu = *cu.firstTU;
  int        mtsIdx = tu.mtsIdx[COMPONENT_Y]; // Transform skip flag has already been decoded

  if( CU::isMTSAllowed( cu, COMPONENT_Y ) && !cuCtx.violatesMtsCoeffConstraint &&
      cuCtx.mtsLastScanPos && cu.lfnstIdx == 0 && mtsIdx != MTS_SKIP)
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__MTS_FLAGS, tu.blocks[COMPONENT_Y], COMPONENT_Y );
#if JVET_W0103_INTRA_MTS
#if JVET_Y0142_ADAPT_INTRA_MTS
#if JVET_Y0159_INTER_MTS
    int ctxIdx = 0;
    if (CU::isIntra(cu))
    {
      ctxIdx = (cuCtx.mtsCoeffAbsSum > MTS_TH_COEFF[1]) ? 2 : (cuCtx.mtsCoeffAbsSum > MTS_TH_COEFF[0]) ? 1 : 0;
    }
#else
    int ctxIdx = (cuCtx.mtsCoeffAbsSum > MTS_TH_COEFF[1]) ? 2 : (cuCtx.mtsCoeffAbsSum > MTS_TH_COEFF[0]) ? 1 : 0;
#endif
#else
    int ctxIdx = (cu.mipFlag) ? 3 : 0;
#endif
#else
    int ctxIdx = 0;
#endif
    int symbol = m_BinDecoder.decodeBin( Ctx::MTSIdx(ctxIdx));

    if( symbol )
    {
#if JVET_W0103_INTRA_MTS
#if JVET_Y0142_ADAPT_INTRA_MTS
#if JVET_Y0159_INTER_MTS
      int nCands = CU::isIntra(cu)? MTS_NCANDS[ctxIdx] : 4;
#else
      int nCands = MTS_NCANDS[ctxIdx];
#endif
      uint32_t val;
      if (nCands > 1)
      {
        xReadTruncBinCode(val, nCands);
      }
      else
      {
        val = 0;
      }
      mtsIdx = MTS_DST7_DST7 + val;
#else
      int bins[2];
      bins[0] = m_BinDecoder.decodeBin(Ctx::MTSIdx(1));
      bins[1] = m_BinDecoder.decodeBin(Ctx::MTSIdx(2));
      mtsIdx = MTS_DST7_DST7 + (bins[0] << 1) + bins[1];
#endif
#else
      ctxIdx = 1;
      mtsIdx = MTS_DST7_DST7; // mtsIdx = 2 -- 4
      for( int i = 0; i < 3; i++, ctxIdx++ )
      {
        symbol  = m_BinDecoder.decodeBin( Ctx::MTSIdx(ctxIdx));
        mtsIdx += symbol;

        if( !symbol )
        {
          break;
        }
      }
#endif
    }
  }

#if TU_256
  for( auto &tmpTu : CU::traverseTUs( cu ) )
  {
    tmpTu.mtsIdx[COMPONENT_Y] = mtsIdx;
  }
#else
  tu.mtsIdx[COMPONENT_Y] = mtsIdx;
#endif

  DTRACE(g_trace_ctx, D_SYNTAX, "mts_idx() etype=%d pos=(%d,%d) mtsIdx=%d\n", COMPONENT_Y, tu.cu->lx(), tu.cu->ly(), mtsIdx);
}

void CABACReader::isp_mode( CodingUnit& cu )
{
  if( !CU::isIntra( cu ) || !isLuma( cu.chType ) || cu.firstPU->multiRefIdx || !cu.cs->sps->getUseISP() || cu.bdpcmMode || !CU::canUseISP( cu, getFirstComponentOfChannel( cu.chType ) ) || cu.colorTransform 
#if ENABLE_DIMD && JVET_V0087_DIMD_NO_ISP
      || cu.dimd
#endif
#if JVET_AB0155_SGPM
      || cu.sgpm
#endif
    )
  {
    cu.ispMode = NOT_INTRA_SUBPARTITIONS;
    return;
  }

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__ISP_MODE_FLAG, cu.lumaSize());

#if JVET_W0123_TIMD_FUSION
  int symbol = m_BinDecoder.decodeBin(cu.timd ? Ctx::ISPMode(2) : Ctx::ISPMode(0));
#else
  int symbol = m_BinDecoder.decodeBin(Ctx::ISPMode(0));
#endif

  if( symbol )
  {
    cu.ispMode = 1 + m_BinDecoder.decodeBin( Ctx::ISPMode( 1 ) );
  }
  DTRACE( g_trace_ctx, D_SYNTAX, "intra_subPartitions() etype=%d pos=(%d,%d) ispIdx=%d\n", cu.chType, cu.blocks[cu.chType].x, cu.blocks[cu.chType].y, (int)cu.ispMode );
}

void CABACReader::residual_lfnst_mode( CodingUnit& cu,  CUCtx& cuCtx  )
{
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  int chIdx = CS::isDualITree(*cu.cs) && cu.chType == CHANNEL_TYPE_CHROMA ? 1 : 0;
#else
  int chIdx = cu.isSepTree() && cu.chType == CHANNEL_TYPE_CHROMA ? 1 : 0;
#endif
  if ((cu.ispMode && !CU::canUseLfnstWithISP(cu, cu.chType))
#if JVET_V0130_INTRA_TMP
    || (cu.cs->sps->getUseLFNST() && CU::isIntra(cu) && ((cu.mipFlag && !allowLfnstWithMip(cu.firstPU->lumaSize())) || (cu.tmpFlag && !allowLfnstWithTmp())))
#else
    || (cu.cs->sps->getUseLFNST() && CU::isIntra(cu) && cu.mipFlag && !allowLfnstWithMip(cu.firstPU->lumaSize()))
#endif
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    || (CS::isDualITree(*cu.cs) && cu.chType == CHANNEL_TYPE_CHROMA && std::min(cu.blocks[1].width, cu.blocks[1].height) < 4)
#else
    || (cu.isSepTree() && cu.chType == CHANNEL_TYPE_CHROMA && std::min(cu.blocks[1].width, cu.blocks[1].height) < 4)
#endif
    || (cu.blocks[chIdx].lumaSize().width > cu.cs->sps->getMaxTbSize()
          || cu.blocks[chIdx].lumaSize().height > cu.cs->sps->getMaxTbSize()))
  {
    return;
  }
#if JVET_W0123_TIMD_FUSION
  if (cu.timd && (cu.ispMode || cu.firstPU->multiRefIdx))
  {
    cu.lfnstIdx = 0;
    return;
  }
#endif


  if( cu.cs->sps->getUseLFNST() && CU::isIntra( cu ) )
  {
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
    const bool lumaFlag              = CS::isDualITree(*cu.cs) ? (isLuma(cu.chType) ? true : false) : true;
    const bool chromaFlag            = CS::isDualITree(*cu.cs) ? (isChroma(cu.chType) ? true : false) : true;
#else
    const bool lumaFlag              = cu.isSepTree() ? (   isLuma( cu.chType ) ? true : false ) : true;
    const bool chromaFlag            = cu.isSepTree() ? ( isChroma( cu.chType ) ? true : false ) : true;
#endif
    bool nonZeroCoeffNonTsCorner8x8 = ( lumaFlag && cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_LUMA] ) || (chromaFlag && cuCtx.violatesLfnstConstrained[CHANNEL_TYPE_CHROMA] );
    bool isTrSkip = false;
    for (auto &currTU : CU::traverseTUs(cu))
    {
      const uint32_t numValidComp = getNumberValidComponents(cu.chromaFormat);
      for (uint32_t compID = COMPONENT_Y; compID < numValidComp; compID++)
      {
        if (currTU.blocks[compID].valid() && TU::getCbf(currTU, (ComponentID)compID) && currTU.mtsIdx[compID] == MTS_SKIP)
        {
          isTrSkip = true;
          break;
        }
      }
    }
    if ((!cuCtx.lfnstLastScanPos && !cu.ispMode) || nonZeroCoeffNonTsCorner8x8 || isTrSkip)
    {
      cu.lfnstIdx = 0;
      return;
    }
  }
  else
  {
    cu.lfnstIdx = 0;
    return;
  }

  unsigned cctx = 0;
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if (CS::isDualITree(*cu.cs)) cctx++;
#else
  if (cu.isSepTree()) cctx++;
#endif

  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__LFNST, cu.blocks[chIdx].lumaSize(), ChannelType(chIdx));
#if EXTENDED_LFNST || JVET_W0119_LFNST_EXTENSION
  uint32_t firstBit = m_BinDecoder.decodeBin(Ctx::LFNSTIdx(cctx));
  cctx = 2 + firstBit;
  uint32_t secondBit = m_BinDecoder.decodeBin(Ctx::LFNSTIdx(cctx));
  uint32_t idxLFNST = firstBit + (secondBit << 1);
#else
  uint32_t idxLFNST = m_BinDecoder.decodeBin( Ctx::LFNSTIdx( cctx ) );
  if( idxLFNST )
  {
    idxLFNST += m_BinDecoder.decodeBin(Ctx::LFNSTIdx(2));
  }
#endif
  cu.lfnstIdx = idxLFNST;

  DTRACE( g_trace_ctx, D_SYNTAX, "residual_lfnst_mode() etype=%d pos=(%d,%d) mode=%d\n", COMPONENT_Y, cu.lx(), cu.ly(), ( int ) cu.lfnstIdx );
}

int CABACReader::last_sig_coeff( CoeffCodingContext& cctx, TransformUnit& tu, ComponentID compID )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__LAST_SIG_X_Y, Size( cctx.width(), cctx.height() ), cctx.compID() );

  unsigned PosLastX = 0, PosLastY = 0;
  unsigned maxLastPosX = cctx.maxLastPosX();
  unsigned maxLastPosY = cctx.maxLastPosY();

#if !TU_256
  if( tu.cs->sps->getUseMTS() && tu.cu->sbtInfo != 0 && tu.blocks[ compID ].width <= 32 && tu.blocks[ compID ].height <= 32 && compID == COMPONENT_Y )
  {
    maxLastPosX = ( tu.blocks[ compID ].width  == 32 ) ? g_uiGroupIdx[ 15 ] : maxLastPosX;
    maxLastPosY = ( tu.blocks[ compID ].height == 32 ) ? g_uiGroupIdx[ 15 ] : maxLastPosY;
  }
#endif

  for( ; PosLastX < maxLastPosX; PosLastX++ )
  {
    if( !m_BinDecoder.decodeBin( cctx.lastXCtxId( PosLastX ) ) )
    {
      break;
    }
  }
  for( ; PosLastY < maxLastPosY; PosLastY++ )
  {
    if( !m_BinDecoder.decodeBin( cctx.lastYCtxId( PosLastY ) ) )
    {
      break;
    }
  }
  if( PosLastX > 3 )
  {
    uint32_t uiTemp  = 0;
    uint32_t uiCount = ( PosLastX - 2 ) >> 1;
    for ( int i = uiCount - 1; i >= 0; i-- )
    {
      uiTemp += m_BinDecoder.decodeBinEP( ) << i;
    }
    PosLastX = g_uiMinInGroup[ PosLastX ] + uiTemp;
  }
  if( PosLastY > 3 )
  {
    uint32_t uiTemp  = 0;
    uint32_t uiCount = ( PosLastY - 2 ) >> 1;
    for ( int i = uiCount - 1; i >= 0; i-- )
    {
      uiTemp += m_BinDecoder.decodeBinEP( ) << i;
    }
    PosLastY = g_uiMinInGroup[ PosLastY ] + uiTemp;
  }

  int blkPos;
  blkPos = PosLastX + (PosLastY * cctx.width());

  int scanPos = 0;
  for( ; scanPos < cctx.maxNumCoeff() - 1; scanPos++ )
  {
    if( blkPos == cctx.blockPos( scanPos ) )
    {
      break;
    }
  }
  return scanPos;
}

#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
static void check_coeff_conformance(const CoeffCodingContext& cctx, const TCoeff coeff)
#else
static void check_coeff_conformance(TCoeff coeff)
#endif
{
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
  CHECK( coeff < cctx.minCoeff() || coeff > cctx.maxCoeff(),
         "TransCoeffLevel outside allowable range" );
#else
  CHECK( coeff < COEFF_MIN || coeff > COEFF_MAX,
         "TransCoeffLevel should be in the range [-32768, 32767]" );
#endif
}

#if TCQ_8STATES
#if SIGN_PREDICTION
void CABACReader::residual_coding_subblock( CoeffCodingContext& cctx, TCoeff* coeff, TCoeff* sign, const uint64_t stateTransTable, int& state )
#else
void CABACReader::residual_coding_subblock( CoeffCodingContext& cctx, TCoeff* coeff, const uint64_t stateTransTable, int& state )
#endif
#else
#if SIGN_PREDICTION
void CABACReader::residual_coding_subblock( CoeffCodingContext& cctx, TCoeff* coeff, TCoeff* sign, const int stateTransTable, int& state )
#else
void CABACReader::residual_coding_subblock( CoeffCodingContext& cctx, TCoeff* coeff, const int stateTransTable, int& state )
#endif
#endif
{
  // NOTE: All coefficients of the subblock must be set to zero before calling this function
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatisticsClassType ctype_group ( STATS__CABAC_BITS__SIG_COEFF_GROUP_FLAG,  cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_map   ( STATS__CABAC_BITS__SIG_COEFF_MAP_FLAG,    cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_par   ( STATS__CABAC_BITS__PAR_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt1   ( STATS__CABAC_BITS__GT1_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt2   ( STATS__CABAC_BITS__GT2_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_escs  ( STATS__CABAC_BITS__ESCAPE_BITS,           cctx.width(), cctx.height(), cctx.compID() );
#endif

  //===== init =====
  const int   minSubPos   = cctx.minSubPos();
  const bool  isLast      = cctx.isLast();
#if SIGN_PREDICTION
#if JVET_Y0141_SIGN_PRED_IMPROVE
  const int   cgStartPosX = cctx.cgPosX() << cctx.log2CGWidth();
  const int   cgStartPosY = cctx.cgPosY() << cctx.log2CGHeight();
  const bool  isSPArea = (cgStartPosX < SIGN_PRED_FREQ_RANGE) && (cgStartPosY < SIGN_PRED_FREQ_RANGE);
  const bool  signPredQualified = cctx.getPredSignsQualified() > 0 && isSPArea && cctx.width() >= 4 && cctx.height() >= 4;
#else
  const bool  isFirst      = !cctx.isNotFirst();
  const bool  signPredQualified = cctx.getPredSignsQualified() && isFirst && cctx.width() >= 4 && cctx.height() >= 4 ;
#endif
#endif
  int         firstSigPos = ( isLast ? cctx.scanPosLast() : cctx.maxSubPos() );
  int         nextSigPos  = firstSigPos;

  //===== decode significant_coeffgroup_flag =====
  RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_group );
  bool sigGroup = ( isLast || !minSubPos );
  if( !sigGroup )
  {
    sigGroup = m_BinDecoder.decodeBin( cctx.sigGroupCtxId() );
  }
  if( sigGroup )
  {
    cctx.setSigGroup();
  }
  else
  {
    return;
  }

  uint8_t   ctxOffset[16];

  //===== decode absolute values =====
  const int inferSigPos   = nextSigPos != cctx.scanPosLast() ? ( cctx.isNotFirst() ? minSubPos : -1 ) : nextSigPos;
  int       firstNZPos    = nextSigPos;
  int       lastNZPos     = -1;
  int       numNonZero    =  0;
  int       remRegBins    = cctx.regBinLimit;
  int       firstPosMode2 = minSubPos - 1;
  int       sigBlkPos[ 1 << MLS_CG_SIZE ];

  for( ; nextSigPos >= minSubPos && remRegBins >= 4; nextSigPos-- )
  {
    int      blkPos     = cctx.blockPos( nextSigPos );
    unsigned sigFlag    = ( !numNonZero && nextSigPos == inferSigPos );
    if( !sigFlag )
    {
      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_map );
      const unsigned sigCtxId = cctx.sigCtxIdAbs( nextSigPos, coeff, state );
      sigFlag = m_BinDecoder.decodeBin( sigCtxId );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId );
      remRegBins--;
    }
    else if( nextSigPos != cctx.scanPosLast() )
    {
      cctx.sigCtxIdAbs( nextSigPos, coeff, state ); // required for setting variables that are needed for gtx/par context selection
    }

    if( sigFlag )
    {
      uint8_t&  ctxOff = ctxOffset[ nextSigPos - minSubPos ];
      ctxOff           = cctx.ctxOffsetAbs();
      sigBlkPos[ numNonZero++ ] = blkPos;
      firstNZPos = nextSigPos;
      lastNZPos  = std::max<int>( lastNZPos, nextSigPos );

      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_gt1 );
      unsigned gt1Flag = m_BinDecoder.decodeBin( cctx.greater1CtxIdAbs(ctxOff) );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt1_flag() bin=%d ctx=%d\n", gt1Flag, cctx.greater1CtxIdAbs(ctxOff) );
      remRegBins--;

      unsigned parFlag = 0;
      unsigned gt2Flag = 0;
      if( gt1Flag )
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_par );
        parFlag = m_BinDecoder.decodeBin( cctx.parityCtxIdAbs( ctxOff ) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "par_flag() bin=%d ctx=%d\n", parFlag, cctx.parityCtxIdAbs( ctxOff ) );

        remRegBins--;
        RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_gt2);
        gt2Flag = m_BinDecoder.decodeBin( cctx.greater2CtxIdAbs( ctxOff ) );
        DTRACE( g_trace_ctx, D_SYNTAX_RESI, "gt2_flag() bin=%d ctx=%d\n", gt2Flag, cctx.greater2CtxIdAbs( ctxOff ) );
        remRegBins--;
      }
      coeff[ blkPos ] += 1 + parFlag + gt1Flag + (gt2Flag << 1);
    }
#if TCQ_8STATES
    state = int( ( stateTransTable >> ((state<<3)+((coeff[blkPos]&1)<<2)) ) & 15 );
#else
    state = ( stateTransTable >> ((state<<2)+((coeff[blkPos]&1)<<1)) ) & 3;
#endif
  }
  firstPosMode2 = nextSigPos;
  cctx.regBinLimit = remRegBins;


  //===== 2nd PASS: Go-rice codes =====
  unsigned ricePar = 0;
  for( int scanPos = firstSigPos; scanPos > firstPosMode2; scanPos-- )
  {
    int       sumAll = cctx.templateAbsSum(scanPos, coeff, 4);
    ricePar = g_auiGoRiceParsCoeff[sumAll];
    TCoeff& tcoeff = coeff[ cctx.blockPos( scanPos ) ];
    if( tcoeff >= 4 )
    {
      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_escs );
      int       rem     = m_BinDecoder.decodeRemAbsEP( ricePar, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange() );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, ricePar );
      tcoeff += (rem<<1);
    }
  }

  //===== coeff bypass ====
  for( int scanPos = firstPosMode2; scanPos >= minSubPos; scanPos-- )
  {
    int       sumAll = cctx.templateAbsSum(scanPos, coeff, 0);
    int       rice      = g_auiGoRiceParsCoeff                        [sumAll];
    int       pos0      = g_auiGoRicePosCoeff0(state, rice);
    RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_escs);
    int       rem       = m_BinDecoder.decodeRemAbsEP( rice, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange() );
    DTRACE( g_trace_ctx, D_SYNTAX_RESI, "rem_val() bin=%d ctx=%d\n", rem, rice );
    TCoeff    tcoeff  = ( rem == pos0 ? 0 : rem < pos0 ? rem+1 : rem );
#if TCQ_8STATES
    state = int( ( stateTransTable >> ((state<<3)+((tcoeff&1)<<2)) ) & 15 );
#else
    state = ( stateTransTable >> ((state<<2)+((tcoeff&1)<<1)) ) & 3;
#endif
    if( tcoeff )
    {
      int        blkPos         = cctx.blockPos( scanPos );
      sigBlkPos[ numNonZero++ ] = blkPos;
      firstNZPos = scanPos;
      lastNZPos  = std::max<int>( lastNZPos, scanPos );
      coeff[blkPos] = tcoeff;
    }
  }

  //===== decode sign's =====
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__SIGN_BIT, Size( cctx.width(), cctx.height() ), cctx.compID() );
  const unsigned  numSigns    = ( cctx.hideSign( firstNZPos, lastNZPos ) ? numNonZero - 1 : numNonZero );
#if SIGN_PREDICTION
  unsigned        signPattern;
  if(!signPredQualified)
  {
    signPattern = m_BinDecoder.decodeBinsEP( numSigns ) << ( 32 - numSigns );
  }
#else
  unsigned        signPattern = m_BinDecoder.decodeBinsEP( numSigns ) << ( 32 - numSigns );
#endif

  //===== set final coefficents =====
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
  TCoeff sumAbs = 0;
#else
  int sumAbs = 0;
#endif

#if SIGN_PREDICTION
  if( !signPredQualified)
  {
#endif
  for( unsigned k = 0; k < numSigns; k++ )
  {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
    TCoeff AbsCoeff       = coeff[ sigBlkPos[ k ] ];
#else
    int AbsCoeff          = coeff[ sigBlkPos[ k ] ];
#endif
    sumAbs               += AbsCoeff;
    coeff[ sigBlkPos[k] ] = ( signPattern & ( 1u << 31 ) ? -AbsCoeff : AbsCoeff );
    signPattern         <<= 1;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    check_coeff_conformance( cctx, coeff[ sigBlkPos[k] ] );
#else
    check_coeff_conformance( coeff[ sigBlkPos[k] ] );
#endif
  }
#if SIGN_PREDICTION
  }
#endif
  if( numNonZero > numSigns )
  {
    int k                 = numSigns;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
    TCoeff AbsCoeff       = coeff[ sigBlkPos[ k ] ];
#else
    int AbsCoeff          = coeff[ sigBlkPos[ k ] ];
#endif
    sumAbs               += AbsCoeff;
    coeff[ sigBlkPos[k] ] = ( sumAbs & 1 ? -AbsCoeff : AbsCoeff );
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    check_coeff_conformance( cctx, coeff[ sigBlkPos[k] ] );
#else
    check_coeff_conformance( coeff[ sigBlkPos[k] ] );
#endif

#if SIGN_PREDICTION
    sign [ sigBlkPos[k] ] = TrQuant::SIGN_PRED_HIDDEN;
#endif
  }
}

void CABACReader::residual_codingTS( TransformUnit& tu, ComponentID compID )
{
  DTRACE( g_trace_ctx, D_SYNTAX, "residual_codingTS() etype=%d pos=(%d,%d) size=%dx%d\n", tu.blocks[compID].compID, tu.blocks[compID].x, tu.blocks[compID].y, tu.blocks[compID].width, tu.blocks[compID].height );

  // init coeff coding context
  CoeffCodingContext  cctx    ( tu, compID, false, isLuma(compID) ? tu.cu->bdpcmMode : tu.cu->bdpcmModeChroma);
  TCoeff*             coeff   = tu.getCoeffs( compID ).buf;
  int maxCtxBins = (cctx.maxNumCoeff() * 7) >> 2;
  cctx.setNumCtxBins(maxCtxBins);

  for( int subSetId = 0; subSetId <= ( cctx.maxNumCoeff() - 1 ) >> cctx.log2CGSize(); subSetId++ )
  {
    cctx.initSubblock         ( subSetId );
    residual_coding_subblockTS( cctx, coeff );
  }
}

void CABACReader::residual_coding_subblockTS( CoeffCodingContext& cctx, TCoeff* coeff )
{
  // NOTE: All coefficients of the subblock must be set to zero before calling this function
#if RExt__DECODER_DEBUG_BIT_STATISTICS
  CodingStatisticsClassType ctype_group ( STATS__CABAC_BITS__SIG_COEFF_GROUP_FLAG,  cctx.width(), cctx.height(), cctx.compID() );
#if TR_ONLY_COEFF_STATS
  CodingStatisticsClassType ctype_map   ( STATS__CABAC_BITS__SIG_COEFF_MAP_FLAG_TS, cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_par   ( STATS__CABAC_BITS__PAR_FLAG_TS,           cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt1   ( STATS__CABAC_BITS__GT1_FLAG_TS,           cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt2   ( STATS__CABAC_BITS__GT2_FLAG_TS,           cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_escs  ( STATS__CABAC_BITS__ESCAPE_BITS_TS,        cctx.width(), cctx.height(), cctx.compID() );
#else
  CodingStatisticsClassType ctype_map   ( STATS__CABAC_BITS__SIG_COEFF_MAP_FLAG,    cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_par   ( STATS__CABAC_BITS__PAR_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt1   ( STATS__CABAC_BITS__GT1_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_gt2   ( STATS__CABAC_BITS__GT2_FLAG,              cctx.width(), cctx.height(), cctx.compID() );
  CodingStatisticsClassType ctype_escs  ( STATS__CABAC_BITS__ESCAPE_BITS,           cctx.width(), cctx.height(), cctx.compID() );
#endif

#endif

  //===== init =====
  const int   minSubPos   = cctx.maxSubPos();
  int         firstSigPos = cctx.minSubPos();
  int         nextSigPos  = firstSigPos;
  unsigned    signPattern = 0;

  //===== decode significant_coeffgroup_flag =====
  RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_group );
  bool sigGroup = cctx.isLastSubSet() && cctx.noneSigGroup();
  if( !sigGroup )
  {
    sigGroup = m_BinDecoder.decodeBin(cctx.sigGroupCtxId(true));
    DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_sigGroup() bin=%d ctx=%d\n", sigGroup, cctx.sigGroupCtxId());
  }
  if( sigGroup )
  {
    cctx.setSigGroup();
  }
  else
  {
    return;
  }

  //===== decode absolute values =====
  const int inferSigPos   = minSubPos;
  int       numNonZero    =  0;
  int       sigBlkPos[ 1 << MLS_CG_SIZE ];

  int lastScanPosPass1 = -1;
  int lastScanPosPass2 = -1;
  for (; nextSigPos <= minSubPos && cctx.numCtxBins() >= 4; nextSigPos++)
  {
    int      blkPos     = cctx.blockPos( nextSigPos );
    unsigned sigFlag    = ( !numNonZero && nextSigPos == inferSigPos );
    if( !sigFlag )
    {
      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_map );
      const unsigned sigCtxId = cctx.sigCtxIdAbsTS(nextSigPos, coeff);
      sigFlag                 = m_BinDecoder.decodeBin(sigCtxId);
      DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_sig_bin() bin=%d ctx=%d\n", sigFlag, sigCtxId);
      cctx.decimateNumCtxBins(1);
    }

    if( sigFlag )
    {
      //===== decode sign's =====
#if TR_ONLY_COEFF_STATS
      RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__SIGN_BIT_TS, Size(cctx.width(), cctx.height()), cctx.compID());
#else
      RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2( STATS__CABAC_BITS__SIGN_BIT, Size( cctx.width(), cctx.height() ), cctx.compID() );
#endif
      int sign;
      const unsigned signCtxId = cctx.signCtxIdAbsTS(nextSigPos, coeff, cctx.bdpcm());
      sign                     = m_BinDecoder.decodeBin(signCtxId);
      cctx.decimateNumCtxBins(1);

      signPattern += ( sign << numNonZero );

      sigBlkPos[numNonZero++] = blkPos;

      RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_gt1 );
      unsigned gt1Flag;
      const unsigned gt1CtxId = cctx.lrg1CtxIdAbsTS(nextSigPos, coeff, cctx.bdpcm());
      gt1Flag                 = m_BinDecoder.decodeBin(gt1CtxId);
      DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_gt1_flag() bin=%d ctx=%d\n", gt1Flag, gt1CtxId);
      cctx.decimateNumCtxBins(1);

      unsigned parFlag = 0;
      if( gt1Flag )
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_par );
        parFlag = m_BinDecoder.decodeBin(cctx.parityCtxIdAbsTS());
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_par_flag() bin=%d ctx=%d\n", parFlag, cctx.parityCtxIdAbsTS());
        cctx.decimateNumCtxBins(1);
      }
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
      coeff[ blkPos ] = (sign ? -1 : 1 ) * (TCoeff)(1 + parFlag + gt1Flag);
#else
      coeff[ blkPos ] = (sign ? -1 : 1 ) * (1 + parFlag + gt1Flag);
#endif
    }
    lastScanPosPass1 = nextSigPos;
  }

  int cutoffVal = 2;
  const int numGtBins = 4;

  //===== 2nd PASS: gt2 =====
  for (int scanPos = firstSigPos; scanPos <= minSubPos && cctx.numCtxBins() >= 4; scanPos++)
  {
    TCoeff& tcoeff = coeff[cctx.blockPos(scanPos)];
    cutoffVal = 2;
    for (int i = 0; i < numGtBins; i++)
    {
      if( tcoeff < 0)
      {
        tcoeff = -tcoeff;
      }
      if (tcoeff >= cutoffVal)
      {
        RExt__DECODER_DEBUG_BIT_STATISTICS_SET(ctype_gt2);
        unsigned gt2Flag;
        gt2Flag = m_BinDecoder.decodeBin(cctx.greaterXCtxIdAbsTS(cutoffVal >> 1));
        tcoeff += (gt2Flag << 1);
        DTRACE(g_trace_ctx, D_SYNTAX_RESI, "ts_gt%d_flag() bin=%d ctx=%d sp=%d coeff=%d\n", i, gt2Flag,
               cctx.greaterXCtxIdAbsTS(cutoffVal >> 1), scanPos, tcoeff);
        cctx.decimateNumCtxBins(1);
      }
      cutoffVal += 2;
    }
    lastScanPosPass2 = scanPos;
  }
  //===== 3rd PASS: Go-rice codes =====
  for( int scanPos = firstSigPos; scanPos <= minSubPos; scanPos++ )
  {
    TCoeff& tcoeff = coeff[ cctx.blockPos( scanPos ) ];
    RExt__DECODER_DEBUG_BIT_STATISTICS_SET( ctype_escs );

    cutoffVal = (scanPos <= lastScanPosPass2 ? 10 : (scanPos <= lastScanPosPass1 ? 2 : 0));
    if (tcoeff < 0)
    {
      tcoeff = -tcoeff;
    }
    if( tcoeff >= cutoffVal )
    {
      int       rice = cctx.templateAbsSumTS( scanPos, coeff );
      int       rem  = m_BinDecoder.decodeRemAbsEP( rice, COEF_REMAIN_BIN_REDUCTION, cctx.maxLog2TrDRange() );
      DTRACE( g_trace_ctx, D_SYNTAX_RESI, "ts_rem_val() bin=%d ctx=%d sp=%d\n", rem, rice, scanPos );
      tcoeff += (scanPos <= lastScanPosPass1) ? (rem << 1) : rem;
      if (tcoeff && scanPos > lastScanPosPass1)
      {
        int      blkPos = cctx.blockPos(scanPos);
        int sign = m_BinDecoder.decodeBinEP();
        signPattern += (sign << numNonZero);
        sigBlkPos[numNonZero++] = blkPos;
      }
    }
    if (!cctx.bdpcm() && cutoffVal)
    {
      if (tcoeff > 0)
      {
        int rightPixel, belowPixel;
        cctx.neighTS(rightPixel, belowPixel, scanPos, coeff);
        tcoeff = cctx.decDeriveModCoeff(rightPixel, belowPixel, tcoeff);
      }
    }
  }

  //===== set final coefficents =====
  for( unsigned k = 0; k < numNonZero; k++ )
  {
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS
    TCoeff AbsCoeff       = coeff[ sigBlkPos[ k ] ];
#else
    int AbsCoeff          = coeff[ sigBlkPos[ k ] ];
#endif
    coeff[ sigBlkPos[k] ] = ( signPattern & 1 ? -AbsCoeff : AbsCoeff );
    signPattern         >>= 1;
#if JVET_R0351_HIGH_BIT_DEPTH_SUPPORT
    check_coeff_conformance( cctx, coeff[ sigBlkPos[k] ] );
#else
    check_coeff_conformance( coeff[ sigBlkPos[k] ] );
#endif
  }
}

//================================================================================
//  helper functions
//--------------------------------------------------------------------------------
//    unsigned  unary_max_symbol ( ctxId0, ctxId1, maxSymbol )
//    unsigned  unary_max_eqprob (                 maxSymbol )
//    unsigned  exp_golomb_eqprob( count )
//================================================================================

unsigned CABACReader::unary_max_symbol( unsigned ctxId0, unsigned ctxIdN, unsigned maxSymbol  )
{
  unsigned onesRead = 0;
  while( onesRead < maxSymbol && m_BinDecoder.decodeBin( onesRead == 0 ? ctxId0 : ctxIdN ) == 1 )
  {
    ++onesRead;
  }
  return onesRead;
}

unsigned CABACReader::unary_max_eqprob( unsigned maxSymbol )
{
  for( unsigned k = 0; k < maxSymbol; k++ )
  {
    if( !m_BinDecoder.decodeBinEP() )
    {
      return k;
    }
  }
  return maxSymbol;
}

unsigned CABACReader::exp_golomb_eqprob( unsigned count )
{
  unsigned symbol = 0;
  unsigned bit    = 1;
  while( bit )
  {
    bit     = m_BinDecoder.decodeBinEP( );
    symbol += bit << count++;
  }
  if( --count )
  {
    symbol += m_BinDecoder.decodeBinsEP( count );
  }
  return symbol;
}

unsigned CABACReader::code_unary_fixed( unsigned ctxId, unsigned unary_max, unsigned fixed )
{
  unsigned idx;
  bool unary = m_BinDecoder.decodeBin( ctxId );
  if( unary )
  {
    idx = unary_max_eqprob( unary_max );
  }
  else
  {
    idx = unary_max + 1 + m_BinDecoder.decodeBinsEP( fixed );
  }
  return idx;
}
#if JVET_V0130_INTRA_TMP
void CABACReader::tmp_flag(CodingUnit& cu)
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__INTRA_TMP_FLAG, cu.lumaSize(), COMPONENT_Y);

	if (!cu.Y().valid())
	{
		return;
	}

#if JVET_X0124_TMP_SIGNAL
  if (cu.dimd)
  {
    cu.tmpFlag = false;
    return;
  }
#endif

  if( !cu.cs->sps->getUseIntraTMP() )
  {
    cu.tmpFlag = false;
    return;
  }

	unsigned ctxId = DeriveCtx::CtxTmpFlag(cu);
	cu.tmpFlag = m_BinDecoder.decodeBin(Ctx::TmpFlag(ctxId));
	DTRACE(g_trace_ctx, D_SYNTAX, "tmp_flag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.tmpFlag ? 1 : 0);
}
#endif
void CABACReader::mip_flag( CodingUnit& cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__INTRA_MIP, cu.lumaSize(), COMPONENT_Y);
#if ENABLE_DIMD
  if (cu.dimd)
  {
    cu.mipFlag = false;
    return;
  }
#endif
  if( !cu.Y().valid() )
  {
    return;
  }
  if( !cu.cs->sps->getUseMIP() )
  {
    cu.mipFlag = false;
    return;
  }


  unsigned ctxId = DeriveCtx::CtxMipFlag( cu );
  cu.mipFlag = m_BinDecoder.decodeBin( Ctx::MipFlag( ctxId ) );
  DTRACE( g_trace_ctx, D_SYNTAX, "mip_flag() pos=(%d,%d) mode=%d\n", cu.lumaPos().x, cu.lumaPos().y, cu.mipFlag ? 1 : 0 );
}

void CABACReader::mip_pred_modes( CodingUnit &cu )
{
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__INTRA_MIP, cu.lumaSize(), COMPONENT_Y);

  if( !cu.Y().valid() )
  {
    return;
  }
  for( auto &pu : CU::traversePUs( cu ) )
  {
    mip_pred_mode( pu );
  }
}

void CABACReader::mip_pred_mode( PredictionUnit &pu )
{
  pu.mipTransposedFlag = bool(m_BinDecoder.decodeBinEP());

  uint32_t mipMode;
  const int numModes = getNumModesMip( pu.Y() );
  xReadTruncBinCode( mipMode, numModes );
  pu.intraDir[CHANNEL_TYPE_LUMA] = mipMode;
  CHECKD( pu.intraDir[CHANNEL_TYPE_LUMA] < 0 || pu.intraDir[CHANNEL_TYPE_LUMA] >= numModes, "Invalid MIP mode" );

  DTRACE( g_trace_ctx, D_SYNTAX, "mip_pred_mode() pos=(%d,%d) mode=%d transposed=%d\n", pu.lumaPos().x, pu.lumaPos().y, pu.intraDir[CHANNEL_TYPE_LUMA], pu.mipTransposedFlag ? 1 : 0 );
}

#if INTER_LIC
void CABACReader::cu_lic_flag( CodingUnit& cu )
{
  if( CU::isLICFlagPresent( cu ) )
  {
    RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE( STATS__CABAC_BITS__LIC_FLAG, cu.lumaSize() );
    cu.LICFlag = m_BinDecoder.decodeBin( Ctx::LICFlag( 0 ) );
    DTRACE( g_trace_ctx, D_SYNTAX, "cu_lic_flag() lic_flag=%d\n", cu.LICFlag ? 1 : 0 );
  }
}
#endif

#if JVET_AA0070_RRIBC
void CABACReader::rribcData(CodingUnit &cu)
{
  if (!CU::isIBC(cu) || cu.firstPU->mergeFlag)
  {
    return;
  }

  cu.rribcFlipType = 0;

  unsigned ctxId   = DeriveCtx::CtxRribcFlipType(cu);
  cu.rribcFlipType = (m_BinDecoder.decodeBin(Ctx::rribcFlipType(ctxId)));
  if (cu.rribcFlipType)
  {
    cu.rribcFlipType += m_BinDecoder.decodeBin(Ctx::rribcFlipType(3));
  }
  DTRACE(g_trace_ctx, D_SYNTAX, "rribcData() rribcFlipType = %d\n", cu.rribcFlipType);
}
#endif


#if SIGN_PREDICTION
void CABACReader::parsePredictedSigns( TransformUnit &tu, ComponentID compID )
{
  uint32_t bin;
  const CtxSet* ctx = &Ctx::signPred[toChannelType( compID )];
  int ctxOffset = CU::isIntra( *tu.cu ) ? 0 : 2;
#if JVET_Y0141_SIGN_PRED_IMPROVE
  bool lfnstEnabled = tu.checkLFNSTApplied(compID);
  const int32_t maxNumPredSigns = lfnstEnabled ? std::min<int>(4, tu.cs->sps->getNumPredSigns()) : tu.cs->sps->getNumPredSigns();
#else
  const int32_t maxNumPredSigns = tu.cs->sps->getNumPredSigns();
#endif
  const bool useSignPred = TU::getUseSignPred( tu, compID );
  int numSignPred = 0;
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE2(STATS__CABAC_BITS__SIGN_BIT, tu.cu->block(compID).lumaSize(), compID);

  CoeffBuf buff = tu.getCoeffs( compID );
  CoeffBuf signBuff = tu.getCoeffSigns( compID );
  TCoeff *coeff = buff.buf;
  TCoeff *signs = signBuff.buf;
#if JVET_Y0141_SIGN_PRED_IMPROVE
  uint32_t extAreaWidth = std::min(tu.blocks[compID].width, (uint32_t)SIGN_PRED_FREQ_RANGE);
  uint32_t extAreaHeight = std::min(tu.blocks[compID].height, (uint32_t)SIGN_PRED_FREQ_RANGE);
  uint32_t extAreaSize  = lfnstEnabled ? 4 : tu.cs->sps->getSignPredArea();
  uint32_t spAreaWidth  = useSignPred ? std::min(tu.blocks[compID].width, extAreaSize)  : extAreaWidth;
  uint32_t spAreaHeight = useSignPred ? std::min(tu.blocks[compID].height, extAreaSize) : extAreaHeight;

  for ( uint32_t y = 0; y < spAreaHeight; y++ )
#else
  for ( uint32_t y = 0; y < SIGN_PRED_FREQ_RANGE; y++ )
#endif
  {
#if JVET_Y0141_SIGN_PRED_IMPROVE
    for ( uint32_t x = 0; x < spAreaWidth; x++ )
#else
    for( uint32_t x = 0; x < SIGN_PRED_FREQ_RANGE; x++ )
#endif
    {
      TCoeff coef = coeff[x];

      if( coef )
      {
        if( signs[x] != TrQuant::SIGN_PRED_HIDDEN )
        {
          if( !useSignPred || numSignPred >= maxNumPredSigns )
          {
            bin = m_BinDecoder.decodeBinEP();
          }
          else
          {
#if JVET_Y0141_SIGN_PRED_IMPROVE
            int levOffset = (coef < 2) ? 0 : 1;
            bin = m_BinDecoder.decodeBin((*ctx)(ctxOffset + levOffset));
#else
            uint32_t ctxId = ( x || y ) ? 1 : 0;
            bin = m_BinDecoder.decodeBin( ( *ctx )( ctxId + ctxOffset ) );
#endif
            numSignPred++;
          }

          if( bin )
          {
            coeff[x] = -coef;
          }
        }
      }
    }
    coeff += buff.stride;
    signs += signBuff.stride;
  }
#if JVET_Y0141_SIGN_PRED_IMPROVE
  if ( spAreaWidth != extAreaWidth || spAreaHeight != extAreaHeight )
  {
    coeff = buff.buf;
    for ( uint32_t y = 0; y < extAreaHeight; y++ )
    {
      uint32_t startX = (y < spAreaHeight) ? spAreaWidth : 0;
      uint32_t endX = extAreaWidth - 1;
      for ( uint32_t x = startX; x <= endX; x++ )
      {
        TCoeff coef = coeff[x];
        if (coef)
        {
          bin = m_BinDecoder.decodeBinEP();
          if (bin)
          {
            coeff[x] = -coef;
          }
        }
      }
      coeff += buff.stride;
    }
  }
#endif
}
#endif

#if JVET_X0083_BM_AMVP_MERGE_MODE
void CABACReader::amvpMerge_mode( PredictionUnit& pu )
{
  if (PU::isBipredRestriction(pu))
  {
    CHECK(1, "this is not possible");
    return;
  }
  unsigned useAmMode = 0;
  RExt__DECODER_DEBUG_BIT_STATISTICS_CREATE_SET_SIZE(STATS__CABAC_BITS__AMVP_MERGE_FLAG, pu.lumaSize());
#if JVET_Y0128_NON_CTC
  if (pu.cu->slice->getUseAmvpMergeMode() == true)
#else
  if (!pu.cu->slice->getCheckLDC())
#endif
  {
    useAmMode = m_BinDecoder.decodeBin(Ctx::amFlagState());
  }
  if (useAmMode)
  {
    pu.interDir = 3; // implicit BI prediction
    unsigned mergeDir = 0;
#if JVET_Z0054_BLK_REF_PIC_REORDER
    if (pu.cs->sps->getUseARL())
    {
      mergeDir = 1;
    }
    else
#endif
    if (pu.cu->cs->picHeader->getMvdL1ZeroFlag() == false)
    {
      mergeDir = m_BinDecoder.decodeBinEP();
    }
    else
    {
      // when L1ZeroFlag is true, only L1 can be mergeDir
      mergeDir = 1;
    }
    pu.mvpIdx[mergeDir] = 2;
    pu.amvpMergeModeFlag[mergeDir] = true;
    pu.amvpMergeModeFlag[1 - mergeDir] = false;
  }
}
#endif

#if JVET_AB0157_TMRL
void CABACReader::cuTmrlFlag(CodingUnit& cu)
{
  if (!CU::allowTmrl(cu))
  {
    cu.firstPU->multiRefIdx = 0;
    cu.tmrlFlag = false;
    cu.tmrlListIdx = 0;
    return;
  }
  PredictionUnit* pu = cu.firstPU;
#if JVET_W0123_TIMD_FUSION
  if (cu.timd)
  {
    CHECK(cu.tmrlFlag, "TMRL cannot combine with TIMD.");
    uint8_t bin = 0;
    bin = m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(5)); // TIMD MRL
    if (bin)
    {
      bin += m_BinDecoder.decodeBin(Ctx::MultiRefLineIdx(6)); // which line
    }
    pu->multiRefIdx = MULTI_REF_LINE_IDX[bin];
    DTRACE(g_trace_ctx, D_SYNTAX, "extend_ref_line() idx=%d pos=(%d,%d) ref_idx=%d\n", bin, pu->lumaPos().x, pu->lumaPos().y, pu->multiRefIdx);
  }
  else
  {
#endif
    int ctxId = 0;
    cu.tmrlFlag = bool(m_BinDecoder.decodeBin(Ctx::TmrlDerive(ctxId++)));
    DTRACE(g_trace_ctx, D_SYNTAX, "cu_tmrl_flag() ctx=%d pos=(%d,%d) tmrl=%d\n", 0, cu.lumaPos().x, cu.lumaPos().y, cu.tmrlFlag);
    if (cu.tmrlFlag)
    {
      const int maxNumCtxBins = (MRL_LIST_SIZE / MRL_IDX_RICE_CODE_DIVISOR) - 1;
      int mrlIdxPrefix = 0;
      for (int val = 0; val < maxNumCtxBins; val++)
      {
        unsigned int bin = m_BinDecoder.decodeBin(Ctx::TmrlDerive(ctxId++));
        mrlIdxPrefix += bin;
        if (!bin)
        {
          break;
        }
      }
      uint32_t mrlSuffixBin0 = 0, mrlSuffixBin1 = 0;
      mrlSuffixBin0 = m_BinDecoder.decodeBin(Ctx::TmrlDerive(maxNumCtxBins + 1));
      mrlSuffixBin1 = m_BinDecoder.decodeBin(Ctx::TmrlDerive(maxNumCtxBins + 2));
      cu.tmrlListIdx = mrlIdxPrefix * MRL_IDX_RICE_CODE_DIVISOR + (mrlSuffixBin0 + (mrlSuffixBin1 << 1));
      DTRACE(g_trace_ctx, D_SYNTAX, "cu_tmrl_idx() ctx=%d pos=(%d,%d) tmrlidx=%d\n", 0, cu.lumaPos().x, cu.lumaPos().y, cu.tmrlListIdx);
    }
    else
    {
      cu.tmrlListIdx = 0;
    }
#if JVET_W0123_TIMD_FUSION
  }
#endif
}
#endif
