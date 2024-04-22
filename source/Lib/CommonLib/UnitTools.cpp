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

/** \file     UnitTool.cpp
 *  \brief    defines operations for basic units
 */

#include "UnitTools.h"

#include "dtrace_next.h"

#include "Unit.h"
#include "Slice.h"
#include "Picture.h"

#include <utility>
#include <algorithm>

// CS tools


uint64_t CS::getEstBits(const CodingStructure &cs)
{
  return cs.fracBits >> SCALE_BITS;
}

bool CS::isDualITree( const CodingStructure &cs )
{
  return cs.slice->isIntra() && !cs.pcv->ISingleTree;
}

UnitArea CS::getArea( const CodingStructure &cs, const UnitArea &area, const ChannelType chType )
{
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  return isDualITree(cs) ? area.singleChan(chType) : area;
#else
  return isDualITree( cs ) || cs.treeType != TREE_D ? area.singleChan( chType ) : area;
#endif
}
#if !MULTI_PASS_DMVR
void CS::setRefinedMotionField(CodingStructure &cs)
{
  for (CodingUnit *cu : cs.cus)
  {
    for (auto &pu : CU::traversePUs(*cu))
    {
      PredictionUnit subPu = pu;
      int dx, dy, x, y, num = 0;
      dy = std::min<int>(pu.lumaSize().height, DMVR_SUBCU_HEIGHT);
      dx = std::min<int>(pu.lumaSize().width, DMVR_SUBCU_WIDTH);
      Position puPos = pu.lumaPos();
      if (PU::checkDMVRCondition(pu))
      {
        for (y = puPos.y; y < (puPos.y + pu.lumaSize().height); y = y + dy)
        {
          for (x = puPos.x; x < (puPos.x + pu.lumaSize().width); x = x + dx)
          {
            subPu.UnitArea::operator=(UnitArea(pu.chromaFormat, Area(x, y, dx, dy)));
            subPu.mv[0] = pu.mv[0];
            subPu.mv[1] = pu.mv[1];
            subPu.mv[REF_PIC_LIST_0] += pu.mvdL0SubPu[num];
            subPu.mv[REF_PIC_LIST_1] -= pu.mvdL0SubPu[num];
            subPu.mv[REF_PIC_LIST_0].clipToStorageBitDepth();
            subPu.mv[REF_PIC_LIST_1].clipToStorageBitDepth();
            pu.mvdL0SubPu[num].setZero();
            num++;
#if JVET_W0123_TIMD_FUSION
            PU::spanMotionInfo2(subPu);
#else
            PU::spanMotionInfo(subPu);
#endif
          }
        }
      }
    }
  }
}
#endif
#if JVET_AE0043_CCP_MERGE_TEMPORAL
void CS::saveTemporalCcpModel(CodingStructure &cs)
{
  if (cs.area.Cb().area() > 0)
  {
    CCPModelIdxBuf ccpIdxBuf = cs.getCcpmIdxBuf(cs.area.Cb());
    ccpIdxBuf.fill(0);
  }
  cs.m_ccpModelLUT.resize(0);

  int idx = 0;
  for (CodingUnit *cu : cs.cus)
  {
#if JVET_AF0073_INTER_CCP_MERGE
    if (CU::isIntra(*cu))
    {
#endif
    for (auto &pu : CU::traversePUs(*cu))
    {
      if (pu.curCand.type > 0)
      {
        CCPModelCandidate ccpModel = pu.curCand;
        cs.m_ccpModelLUT.push_back(ccpModel);
        idx++;
        CCPModelIdxBuf ccpIdxBuf = pu.cs->getCcpmIdxBuf(pu.Cb());
        ccpIdxBuf.fill(idx);
      }
    }
#if JVET_AF0073_INTER_CCP_MERGE
    }
    else
    {
      for (auto& tu : CU::traverseTUs(*cu))
      {
        if (tu.curCand.type > 0)
        {
          CCPModelCandidate ccpModel = tu.curCand;
          cs.m_ccpModelLUT.push_back(ccpModel);
          idx++;
          CCPModelIdxBuf ccpIdxBuf = tu.cs->getCcpmIdxBuf(tu.Cb());
          ccpIdxBuf.fill(idx);
        }
      }
    }
#endif
  }
}
#endif
#if JVET_AG0058_EIP
void CS::saveTemporalEipModel(CodingStructure &cs)
{
  if (cs.area.Y().area() > 0)
  {
    EipModelIdxBuf eipIdxBuf = cs.getEipIdxBuf(cs.area.Y());
    eipIdxBuf.fill(0);
  }
  cs.m_eipModelLUT.resize(0);

  int idx = 0;
  for (CodingUnit *cu : cs.cus)
  {
    if (cu->Y().valid()&& cu->eipFlag)
    {
      EipModelCandidate eipModel = cu->eipModel;
      cs.m_eipModelLUT.push_back(eipModel);
      idx++;
      EipModelIdxBuf eipIdxBuf = cu->cs->getEipIdxBuf(cu->Y());
      eipIdxBuf.fill(idx);
    }
  }
}
#endif
// CU tools

#if JVET_AG0117_CABAC_SPATIAL_TUNING
bool CU::isOnCtuBottom( const CodingUnit& cu )
{
  const CodingStructure &cs = *cu.cs;
  const ComponentID compID  = cu.chType == CHANNEL_TYPE_CHROMA ? COMPONENT_Cb : COMPONENT_Y;
  const Area& cuArea        = cu.chType == CHANNEL_TYPE_CHROMA ? cu.Cb() : cu.Y();

  const int ctuHeight = cs.pcv->maxCUHeight >> getComponentScaleY(compID, cs.pcv->chrFormat);
  const int cuBottomY = cuArea.y + cuArea.height;

  return cuBottomY % ctuHeight == 0;
}
#endif

bool CU::getRprScaling( const SPS* sps, const PPS* curPPS, Picture* refPic, int& xScale, int& yScale )
{
  const Window& curScalingWindow = curPPS->getScalingWindow();
  int curPicWidth = curPPS->getPicWidthInLumaSamples()   - SPS::getWinUnitX( sps->getChromaFormatIdc() ) * (curScalingWindow.getWindowLeftOffset() + curScalingWindow.getWindowRightOffset());
  int curPicHeight = curPPS->getPicHeightInLumaSamples() - SPS::getWinUnitY( sps->getChromaFormatIdc() ) * (curScalingWindow.getWindowTopOffset()  + curScalingWindow.getWindowBottomOffset());

  const Window& refScalingWindow = refPic->getScalingWindow();
  int refPicWidth = refPic->getPicWidthInLumaSamples()   - SPS::getWinUnitX( sps->getChromaFormatIdc() ) * (refScalingWindow.getWindowLeftOffset() + refScalingWindow.getWindowRightOffset());
  int refPicHeight = refPic->getPicHeightInLumaSamples() - SPS::getWinUnitY( sps->getChromaFormatIdc() ) * (refScalingWindow.getWindowTopOffset()  + refScalingWindow.getWindowBottomOffset());

  xScale = ( ( refPicWidth << SCALE_RATIO_BITS ) + ( curPicWidth >> 1 ) ) / curPicWidth;
  yScale = ( ( refPicHeight << SCALE_RATIO_BITS ) + ( curPicHeight >> 1 ) ) / curPicHeight;

  int curSeqMaxPicWidthY = sps->getMaxPicWidthInLumaSamples();                  // pic_width_max_in_luma_samples
  int curSeqMaxPicHeightY = sps->getMaxPicHeightInLumaSamples();                // pic_height_max_in_luma_samples
  int curPicWidthY = curPPS->getPicWidthInLumaSamples();                        // pic_width_in_luma_samples
  int curPicHeightY = curPPS->getPicHeightInLumaSamples();                      // pic_height_in_luma_samples
  int max8MinCbSizeY = std::max((int)8, (1<<sps->getLog2MinCodingBlockSize())); // Max(8, MinCbSizeY)

  CHECK((curPicWidth * curSeqMaxPicWidthY) < refPicWidth * (curPicWidthY - max8MinCbSizeY), "(curPicWidth * curSeqMaxPicWidthY) should be greater than or equal to refPicWidth * (curPicWidthY - max8MinCbSizeY))");
  CHECK((curPicHeight * curSeqMaxPicHeightY) < refPicHeight * (curPicHeightY - max8MinCbSizeY), "(curPicHeight * curSeqMaxPicHeightY) should be greater than or equal to refPicHeight * (curPicHeightY - max8MinCbSizeY))");

  CHECK(curPicWidth * 2 < refPicWidth, "curPicWidth * 2 shall be greater than or equal to refPicWidth");
  CHECK(curPicHeight * 2 < refPicHeight, "curPicHeight * 2 shall be greater than or equal to refPicHeight");
  CHECK(curPicWidth > refPicWidth * 8, "curPicWidth shall be less than or equal to refPicWidth * 8");
  CHECK(curPicHeight > refPicHeight * 8, "curPicHeight shall be less than or equal to refPicHeight * 8");

#if JVET_S0048_SCALING_OFFSET
  int subWidthC = SPS::getWinUnitX(sps->getChromaFormatIdc());
  int subHeightC = SPS::getWinUnitY(sps->getChromaFormatIdc());

  CHECK(subWidthC * curScalingWindow.getWindowLeftOffset() < (-curPicWidthY) * 15, "The value of SubWidthC * pps_scaling_win_left_offset shall be greater than or equal to -pps_pic_width_in_luma_samples * 15");
  CHECK(subWidthC * curScalingWindow.getWindowLeftOffset() >= curPicWidthY, "The value of SubWidthC * pps_scaling_win_left_offset shall be less than pic_width_in_luma_samples");
  CHECK(subWidthC * curScalingWindow.getWindowRightOffset() < (-curPicWidthY) * 15, "The value of SubWidthC * pps_scaling_win_right_offset shall be greater than or equal to -pps_pic_width_in_luma_samples * 15");
  CHECK(subWidthC * curScalingWindow.getWindowRightOffset() >= curPicWidthY, "The value of SubWidthC * pps_scaling_win_right_offset shall be less than pic_width_in_luma_samples");

  CHECK(subHeightC * curScalingWindow.getWindowTopOffset() < (-curPicHeightY) * 15, "The value of SubHeightC * pps_scaling_win_top_offset shall be greater than or equal to -pps_pic_height_in_luma_samples * 15");
  CHECK(subHeightC * curScalingWindow.getWindowTopOffset() >= curPicHeightY, "The value of SubHeightC * pps_scaling_win_top_offset shall be less than pps_pic_height_in_luma_samples");
  CHECK(subHeightC * curScalingWindow.getWindowBottomOffset() < (-curPicHeightY) * 15, "The value of SubHeightC *pps_scaling_win_bottom_offset shall be greater than or equal to -pps_pic_height_in_luma_samples * 15");
  CHECK(subHeightC * curScalingWindow.getWindowBottomOffset() >= curPicHeightY, "The value of SubHeightC *pps_scaling_win_bottom_offset shall be less than pps_pic_height_in_luma_samples");

  CHECK(subWidthC * (curScalingWindow.getWindowLeftOffset() + curScalingWindow.getWindowRightOffset()) < (-curPicWidthY) * 15, "The value of SubWidthC * ( pps_scaling_win_left_offset + pps_scaling_win_right_offset ) shall be greater than or equal to -pps_pic_width_in_luma_samples * 15");
  CHECK(subWidthC * (curScalingWindow.getWindowLeftOffset() + curScalingWindow.getWindowRightOffset()) >= curPicWidthY, "The value of SubWidthC * ( pps_scaling_win_left_offset + pps_scaling_win_right_offset ) shall be less than pic_width_in_luma_samples");
  CHECK(subHeightC * (curScalingWindow.getWindowTopOffset() + curScalingWindow.getWindowBottomOffset()) < (-curPicHeightY) * 15, "The value of SubHeightC * ( pps_scaling_win_top_offset + pps_scaling_win_bottom_offset ) shall be greater than or equal to -pps_pic_height_in_luma_samples * 15");
  CHECK(subHeightC * (curScalingWindow.getWindowTopOffset() + curScalingWindow.getWindowBottomOffset()) >= curPicHeightY, "The value of SubHeightC * ( pps_scaling_win_top_offset + pps_scaling_win_bottom_offset ) shall be less than pic_height_in_luma_samples");
#else
  CHECK(SPS::getWinUnitX(sps->getChromaFormatIdc()) * (abs(curScalingWindow.getWindowLeftOffset()) + abs(curScalingWindow.getWindowRightOffset())) > curPPS->getPicWidthInLumaSamples(), "The value of SubWidthC * ( Abs(pps_scaling_win_left_offset) + Abs(pps_scaling_win_right_offset) ) shall be less than pic_width_in_luma_samples");
  CHECK(SPS::getWinUnitY(sps->getChromaFormatIdc()) * (abs(curScalingWindow.getWindowTopOffset()) + abs(curScalingWindow.getWindowBottomOffset())) > curPPS->getPicHeightInLumaSamples(), "The value of SubHeightC * ( Abs(pps_scaling_win_top_offset) + Abs(pps_scaling_win_bottom_offset) ) shall be less than pic_height_in_luma_samples");
#endif

  return refPic->isRefScaled( curPPS );
}

void CU::checkConformanceILRP(Slice *slice)
{
  const int numRefList = slice->isInterB() ? 2 : 1;

#if JVET_S0258_SUBPIC_CONSTRAINTS
  int currentSubPicIdx = NOT_VALID;

  // derive sub-picture index for the current slice
  for( int subPicIdx = 0; subPicIdx < slice->getPic()->cs->sps->getNumSubPics(); subPicIdx++ )
  {
    if( slice->getPic()->cs->pps->getSubPic( subPicIdx ).getSubPicID() == slice->getSliceSubPicId() )
    {
      currentSubPicIdx = subPicIdx;
      break;
    }
  }

  CHECK( currentSubPicIdx == NOT_VALID, "Sub-picture was not found" );

  if( !slice->getPic()->cs->sps->getSubPicTreatedAsPicFlag( currentSubPicIdx ) )
  {
    return;
  }
#endif

  //constraint 1: The picture referred to by each active entry in RefPicList[ 0 ] or RefPicList[ 1 ] has the same subpicture layout as the current picture
  bool isAllRefSameSubpicLayout = true;
  for (int refList = 0; refList < numRefList; refList++) // loop over l0 and l1
  {
    RefPicList  eRefPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);

    for (int refIdx = 0; refIdx < slice->getNumRefIdx(eRefPicList); refIdx++)
    {
#if JVET_S0258_SUBPIC_CONSTRAINTS
      const Picture* refPic = slice->getRefPic( eRefPicList, refIdx );

      if( refPic->subPictures.size() != slice->getPic()->cs->pps->getNumSubPics() )
#else
      const Picture* refPic = slice->getRefPic(eRefPicList, refIdx)->unscaledPic;

      if (refPic->numSubpics != slice->getPic()->cs->pps->getNumSubPics())
#endif
      {
        isAllRefSameSubpicLayout = false;
        refList = numRefList;
        break;
      }
      else
      {
#if JVET_S0258_SUBPIC_CONSTRAINTS
        for( int i = 0; i < refPic->subPictures.size(); i++ )
        {
          const SubPic& refSubPic = refPic->subPictures[i];
          const SubPic& curSubPic = slice->getPic()->cs->pps->getSubPic( i );

          if( refSubPic.getSubPicWidthInCTUs() != curSubPic.getSubPicWidthInCTUs()
            || refSubPic.getSubPicHeightInCTUs() != curSubPic.getSubPicHeightInCTUs()
            || refSubPic.getSubPicCtuTopLeftX() != curSubPic.getSubPicCtuTopLeftX()
            || refSubPic.getSubPicCtuTopLeftY() != curSubPic.getSubPicCtuTopLeftY()
            || ( refPic->layerId != slice->getPic()->layerId && refSubPic.getSubPicID() != curSubPic.getSubPicID() )
            || refSubPic.getTreatedAsPicFlag() != curSubPic.getTreatedAsPicFlag())
#else
        for (int i = 0; i < refPic->numSubpics; i++)
        {
          if (refPic->subpicWidthInCTUs[i] != slice->getPic()->cs->pps->getSubPic(i).getSubPicWidthInCTUs()
            || refPic->subpicHeightInCTUs[i] != slice->getPic()->cs->pps->getSubPic(i).getSubPicHeightInCTUs()
            || refPic->subpicCtuTopLeftX[i] != slice->getPic()->cs->pps->getSubPic(i).getSubPicCtuTopLeftX()
            || refPic->subpicCtuTopLeftY[i] != slice->getPic()->cs->pps->getSubPic(i).getSubPicCtuTopLeftY())
#endif
          {
            isAllRefSameSubpicLayout = false;
            refIdx = slice->getNumRefIdx(eRefPicList);
            refList = numRefList;
            break;
          }
        }

#if JVET_S0258_SUBPIC_CONSTRAINTS
        // A picture with different sub-picture ID of the collocated sub-picture cannot be used as an active reference picture in the same layer
        if( refPic->layerId == slice->getPic()->layerId )
        {
          isAllRefSameSubpicLayout = isAllRefSameSubpicLayout && refPic->subPictures[currentSubPicIdx].getSubPicID() == slice->getSliceSubPicId();
        }
#endif
      }
    }
  }

  //constraint 2: The picture referred to by each active entry in RefPicList[ 0 ] or RefPicList[ 1 ] is an ILRP for which the value of sps_num_subpics_minus1 is equal to 0
  if (!isAllRefSameSubpicLayout)
  {
    for (int refList = 0; refList < numRefList; refList++) // loop over l0 and l1
    {
      RefPicList  eRefPicList = (refList ? REF_PIC_LIST_1 : REF_PIC_LIST_0);
      for (int refIdx = 0; refIdx < slice->getNumRefIdx(eRefPicList); refIdx++)
      {
#if JVET_S0258_SUBPIC_CONSTRAINTS
        const Picture* refPic = slice->getRefPic( eRefPicList, refIdx );
        CHECK( refPic->layerId == slice->getPic()->layerId || refPic->subPictures.size() > 1, "The inter-layer reference shall contain a single subpicture or have same subpicture layout with the current picture" );
#else
        const Picture* refPic = slice->getRefPic(eRefPicList, refIdx)->unscaledPic;
        CHECK(!(refPic->layerId != slice->getPic()->layerId && refPic->numSubpics == 1), "The inter-layer reference shall contain a single subpicture or have same subpicture layout with the current picture");
#endif
      }
    }
  }

  return;
}

#if JVET_AG0276_NLIC
bool CU::isSecLicParaNeeded(const CodingUnit &cu)
{
  if (!(cu.slice->getSPS()->getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
    && cu.slice->getSPS()->getTMToolsEnableFlag()
#endif
    ))
  {
    return false;
  }
  if (!isInter(cu))
  {
    return false;
  }
  if (cu.geoFlag)
  {
    return false;
  }
  if (cu.firstPU->mergeFlag && cu.firstPU->mergeType == MRG_TYPE_SUBPU_ATMVP)
  {
    return false;
  }
#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (cu.firstPU->amvpSbTmvpFlag)
  {
    return false;
  }
#endif
  return true;
}

bool CU::isPredRefined(const CodingUnit &cu)
{
  PredictionUnit* pu = cu.firstPU;

#if INTER_LIC
  if (cu.licFlag)
  {
    return true;
  }
#endif
  if (cu.altLMFlag)
  {
    return true;
  }
#if MULTI_PASS_DMVR
  if (pu->bdmvrRefine)
  {
    return true;
  }
#endif
  if (pu->ciipFlag)
  {
    return true;
  }
#if MULTI_HYP_PRED
  if (!pu->addHypData.empty())
  {
    return true;
  }
#endif

  bool bioApplied = false;
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
  if (pu->cu->affine)
  {
    bioApplied = PU::checkDoAffineBdofRefine(*pu, NULL);
  }
  else
  {
#endif
    if (pu->cs->sps->getBDOFEnabledFlag() && !pu->cs->picHeader->getDisBdofFlag())
    {
#if INTER_LIC
      if (cu.affine || (pu->mergeFlag && (pu->mergeType == MRG_TYPE_SUBPU_ATMVP)) || cu.licFlag)
#else
      if (cu.affine || (pu->mergeFlag && (pu->mergeType == MRG_TYPE_SUBPU_ATMVP)))
#endif
      {
        bioApplied = false;
      }
      else
      {
        int                  refIdx0 = pu->refIdx[REF_PIC_LIST_0];
        int                  refIdx1 = pu->refIdx[REF_PIC_LIST_1];
        const WPScalingParam *wp0 = cu.cs->slice->getWpScaling(REF_PIC_LIST_0, refIdx0);
        const WPScalingParam *wp1 = cu.cs->slice->getWpScaling(REF_PIC_LIST_1, refIdx1);

        const bool biocheck0 = !((WPScalingParam::isWeighted(wp0) || WPScalingParam::isWeighted(wp1)) && cu.cs->slice->getSliceType() == B_SLICE);
        const bool biocheck1 = !(cu.cs->pps->getUseWP() && cu.cs->slice->getSliceType() == P_SLICE);
        if (biocheck0 && biocheck1
          && PU::isBiPredFromDifferentDirEqDistPoc(*pu)
#if !BDOF_RM_CONSTRAINTS
          && (cu.Y().height >= 8)
          && (cu.Y().width >= 8)
          && ((cu.Y().height * cu.Y().width) >= 128)
#endif
          )
        {
          bioApplied = true;
        }
        if (bioApplied && pu->ciipFlag)
        {
          bioApplied = false;
        }
        if (bioApplied && cu.smvdMode)
        {
          bioApplied = false;
        }
        if (cu.cs->sps->getUseBcw() && bioApplied && cu.bcwIdx != BCW_DEFAULT)
        {
          bioApplied = false;
        }
      }
    }
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
  }
#endif
  if (bioApplied)
  {
    return true;
  }

  return false;
}
bool CU::isAllowSecLicPara(const CodingUnit &cu)
{
  if (!cu.slice->getSPS()->getUseAltLM())
  {
    if (!cu.slice->getSPS()->getUseAffAltLM())
    {
      return false;
    }
    else
    {
      if (!cu.affine)
      {
        return false;
      }
    }
  }
  if (isTLCond(cu))
  {
    return false;
  }
  return true;
}
bool CU::isTLCond(const CodingUnit &cu)
{
  if (!cu.slice->getSPS()->getUseAltLM() && !cu.slice->getSPS()->getUseAffAltLM())
  {
    return false;
  }
  if (cu.slice->getTLayer() >= 5)
  {
    return true;
  }
  else
  {
    return false;
  }
}
#endif

bool CU::isIntra(const CodingUnit &cu)
{
  return cu.predMode == MODE_INTRA;
}

bool CU::isInter(const CodingUnit &cu)
{
  return cu.predMode == MODE_INTER;
}

bool CU::isIBC(const CodingUnit &cu)
{
  return cu.predMode == MODE_IBC;
}

bool CU::isPLT(const CodingUnit &cu)
{
  return cu.predMode == MODE_PLT;
}

bool CU::isSameSlice(const CodingUnit& cu, const CodingUnit& cu2)
{
  return cu.slice->getIndependentSliceIdx() == cu2.slice->getIndependentSliceIdx();
}

bool CU::isSameTile(const CodingUnit& cu, const CodingUnit& cu2)
{
  return cu.tileIdx == cu2.tileIdx;
}

bool CU::isSameSliceAndTile(const CodingUnit& cu, const CodingUnit& cu2)
{
  return ( cu.slice->getIndependentSliceIdx() == cu2.slice->getIndependentSliceIdx() ) && ( cu.tileIdx == cu2.tileIdx );
}

bool CU::isSameSubPic(const CodingUnit& cu, const CodingUnit& cu2)
{
  return (cu.slice->getPPS()->getSubPicFromCU(cu).getSubPicIdx() == cu2.slice->getPPS()->getSubPicFromCU(cu2).getSubPicIdx()) ;
}

bool CU::isSameCtu(const CodingUnit& cu, const CodingUnit& cu2)
{
  uint32_t ctuSizeBit = floorLog2(cu.cs->sps->getMaxCUWidth());

  Position pos1Ctu(cu.lumaPos().x  >> ctuSizeBit, cu.lumaPos().y  >> ctuSizeBit);
  Position pos2Ctu(cu2.lumaPos().x >> ctuSizeBit, cu2.lumaPos().y >> ctuSizeBit);

  return pos1Ctu.x == pos2Ctu.x && pos1Ctu.y == pos2Ctu.y;
}

bool CU::isLastSubCUOfCtu( const CodingUnit &cu )
{
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  const Area cuAreaY = CS::isDualITree(*cu.cs) ? Area(recalcPosition(cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].pos()), recalcSize(cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].size())) : (const Area&)cu.Y();
#else
  const Area cuAreaY = cu.isSepTree() ? Area( recalcPosition( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].pos() ), recalcSize( cu.chromaFormat, cu.chType, CHANNEL_TYPE_LUMA, cu.blocks[cu.chType].size() ) ) : (const Area&)cu.Y();
#endif

  return ( ( ( ( cuAreaY.x + cuAreaY.width  ) & cu.cs->pcv->maxCUWidthMask  ) == 0 || cuAreaY.x + cuAreaY.width  == cu.cs->pps->getPicWidthInLumaSamples()  ) &&
           ( ( ( cuAreaY.y + cuAreaY.height ) & cu.cs->pcv->maxCUHeightMask ) == 0 || cuAreaY.y + cuAreaY.height == cu.cs->pps->getPicHeightInLumaSamples() ) );
}

uint32_t CU::getCtuAddr( const CodingUnit &cu )
{
  return getCtuAddr( cu.blocks[cu.chType].lumaPos(), *cu.cs->pcv );
}
#if JVET_V0130_INTRA_TMP
Position CU::getCtuXYAddr(const CodingUnit& cu)
{
	return Position((cu.blocks[cu.chType].lumaPos().x >> cu.cs->pcv->maxCUWidthLog2) << cu.cs->pcv->maxCUWidthLog2, (cu.blocks[cu.chType].lumaPos().y >> cu.cs->pcv->maxCUHeightLog2) << cu.cs->pcv->maxCUHeightLog2);
}
#endif
int CU::predictQP( const CodingUnit& cu, const int prevQP )
{
  const CodingStructure &cs = *cu.cs;

  uint32_t  ctuRsAddr       = getCtuAddr( cu );
  uint32_t  ctuXPosInCtus   = ctuRsAddr % cs.pcv->widthInCtus;
  uint32_t  tileColIdx      = cu.slice->getPPS()->ctuToTileCol( ctuXPosInCtus );
  uint32_t  tileXPosInCtus  = cu.slice->getPPS()->getTileColumnBd( tileColIdx );
  if( ctuXPosInCtus == tileXPosInCtus &&
      !( cu.blocks[cu.chType].x & ( cs.pcv->maxCUWidthMask  >> getChannelTypeScaleX( cu.chType, cu.chromaFormat ) ) ) &&
      !( cu.blocks[cu.chType].y & ( cs.pcv->maxCUHeightMask >> getChannelTypeScaleY( cu.chType, cu.chromaFormat ) ) ) &&
      ( cs.getCU( cu.blocks[cu.chType].pos().offset( 0, -1 ), cu.chType) != NULL ) &&
      CU::isSameSliceAndTile( *cs.getCU( cu.blocks[cu.chType].pos().offset( 0, -1 ), cu.chType), cu ) )
  {
    return ( ( cs.getCU( cu.blocks[cu.chType].pos().offset( 0, -1 ), cu.chType ) )->qp );
  }
  else
  {
    const int a = ( cu.blocks[cu.chType].y & ( cs.pcv->maxCUHeightMask >> getChannelTypeScaleY( cu.chType, cu.chromaFormat ) ) ) ? ( cs.getCU(cu.blocks[cu.chType].pos().offset( 0, -1 ), cu.chType))->qp : prevQP;
    const int b = ( cu.blocks[cu.chType].x & ( cs.pcv->maxCUWidthMask  >> getChannelTypeScaleX( cu.chType, cu.chromaFormat ) ) ) ? ( cs.getCU(cu.blocks[cu.chType].pos().offset( -1, 0 ), cu.chType))->qp : prevQP;

    return ( a + b + 1 ) >> 1;
  }
}


uint32_t CU::getNumPUs( const CodingUnit& cu )
{
  uint32_t cnt = 0;
  PredictionUnit *pu = cu.firstPU;

  do
  {
    cnt++;
  } while( ( pu != cu.lastPU ) && ( pu = pu->next ) );

  return cnt;
}

void CU::addPUs( CodingUnit& cu )
{
  cu.cs->addPU( CS::getArea( *cu.cs, cu, cu.chType ), cu.chType );
}

void CU::saveMotionInHMVP( const CodingUnit& cu, const bool isToBeDone )
{
  const PredictionUnit& pu = *cu.firstPU;
#if JVET_Z0118_GDR
  bool isClean = pu.cs->isClean(pu.cu->Y().bottomRight(), CHANNEL_TYPE_LUMA);
#endif

#if JVET_Z0139_HIST_AFF
#if JVET_AG0164_AFFINE_GPM
  if ((cu.affine && pu.mergeType == MRG_TYPE_DEFAULT_N) ||
   (cu.geoFlag &&( pu.affineGPM[0] || pu.affineGPM[1]) )
    )
  {
#else
  if (cu.affine)
  {
    if (pu.mergeType != MRG_TYPE_DEFAULT_N)
    {
      return;
    }
#endif
    AffineMotionInfo addMi[2];
    int addRefIdx[2];

#if JVET_AG0164_AFFINE_GPM
    AffineInheritInfo addAffInherit;
    addAffInherit.basePos = cu.lumaPos();
    pu.getAffineMotionInfo(addMi, addRefIdx, addAffInherit.baseMV);
#else
    pu.getAffineMotionInfo(addMi, addRefIdx);
#endif
#if JVET_Z0118_GDR 
    cu.cs->addAffMiToLut((isClean) ? cu.cs->motionLut.lutAff1 : cu.cs->motionLut.lutAff0, addMi, addRefIdx);
#else
    cu.cs->addAffMiToLut(cu.cs->motionLut.lutAff, addMi, addRefIdx);
#endif

#if !JVET_AG0164_AFFINE_GPM
    AffineInheritInfo addAffInherit;
    addAffInherit.basePos = cu.lumaPos();
    addAffInherit.baseMV[0] = MvField(pu.mvAffi[0][0], pu.refIdx[0]);
    addAffInherit.baseMV[1] = MvField(pu.mvAffi[1][0], pu.refIdx[1]);
#endif
    addAffInherit.oneSetAffineParametersPattern0 = addMi[0].oneSetAffineParametersPattern;
    addAffInherit.oneSetAffineParametersPattern1 = addMi[1].oneSetAffineParametersPattern;
    if (addAffInherit.oneSetAffineParametersPattern0 == 0)
    {
      addAffInherit.baseMV[0].refIdx = -1;
    }
    if (addAffInherit.oneSetAffineParametersPattern1 == 0)
    {
      addAffInherit.baseMV[1].refIdx = -1;
    }
    if (addAffInherit.baseMV[0].refIdx != -1 || addAffInherit.baseMV[1].refIdx != -1)
    {
#if JVET_Z0118_GDR
      cu.cs->addAffInheritToLut((isClean) ? cu.cs->motionLut.lutAffInherit1 : cu.cs->motionLut.lutAffInherit0, addAffInherit);
#else
      cu.cs->addAffInheritToLut(cu.cs->motionLut.lutAffInherit, addAffInherit);
#endif
    }
    return;
  }
#endif

#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (cu.firstPU->amvpSbTmvpFlag)
  {
    return;
  }
#endif

  if (!cu.geoFlag && !cu.affine && !isToBeDone)
  {
    MotionInfo mi = pu.getMotionInfo();
#if MULTI_HYP_PRED
    mi.addHypData = pu.addHypData;
#endif
#if JVET_AA0070_RRIBC
#if JVET_AF0079_STORING_INTRATMP
    if (CU::isIBC(cu) || cu.tmpFlag)
#else
    if(CU::isIBC(cu))
#endif
    {
      mi.centerPos.x = cu.lx() + (cu.lwidth() >> 1);
      mi.centerPos.y = cu.ly() + (cu.lheight() >> 1);
    }
#endif

    mi.bcwIdx = (mi.interDir == 3) ? cu.bcwIdx : BCW_DEFAULT;

    const unsigned log2ParallelMergeLevel = (pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2);
    const unsigned xBr = pu.cu->Y().width + pu.cu->Y().x;
    const unsigned yBr = pu.cu->Y().height + pu.cu->Y().y;
    bool enableHmvp = ((xBr >> log2ParallelMergeLevel) > (pu.cu->Y().x >> log2ParallelMergeLevel)) && ((yBr >> log2ParallelMergeLevel) > (pu.cu->Y().y >> log2ParallelMergeLevel));
#if JVET_AF0079_STORING_INTRATMP
    bool enableInsertion = CU::isIBC(cu) || enableHmvp || cu.tmpFlag;
#else
    bool enableInsertion = CU::isIBC(cu) || enableHmvp;
#endif
    if (enableInsertion)
    {
#if JVET_Z0075_IBC_HMVP_ENLARGE
#if JVET_Z0118_GDR
      if (isClean)
      {
#if JVET_AF0079_STORING_INTRATMP
        if (CU::isIBC(cu) || cu.tmpFlag)
#else
        if (CU::isIBC(cu))
#endif
        {
          cu.cs->addMiToLutIBC(cu.cs->motionLut.lutIbc1, mi);
        }
        else
        {
          cu.cs->addMiToLut(cu.cs->motionLut.lut1, mi);
        }
      }

#if JVET_AF0079_STORING_INTRATMP
      if (CU::isIBC(cu) || cu.tmpFlag)
#else
      if (CU::isIBC(cu))
#endif
      {
        cu.cs->addMiToLutIBC(cu.cs->motionLut.lutIbc0, mi);
      }
      else
      {
        cu.cs->addMiToLut(cu.cs->motionLut.lut0, mi);
      }
#else 
      if (CU::isIBC(cu))
      {
        cu.cs->addMiToLutIBC(cu.cs->motionLut.lutIbc, mi);
      }
      else
      {
        cu.cs->addMiToLut(cu.cs->motionLut.lut, mi);
      }
#endif
#else
#if JVET_Z0118_GDR
      if (isClean)
      {
        cu.cs->addMiToLut(CU::isIBC(cu) ? cu.cs->motionLut.lutIbc1 : cu.cs->motionLut.lut1, mi);
      }
      cu.cs->addMiToLut(CU::isIBC(cu) ? cu.cs->motionLut.lutIbc0 : cu.cs->motionLut.lut0, mi);      
#else
      cu.cs->addMiToLut(CU::isIBC(cu) ? cu.cs->motionLut.lutIbc : cu.cs->motionLut.lut, mi);
#endif
#endif
    }
  }
}

PartSplit CU::getSplitAtDepth( const CodingUnit& cu, const unsigned depth )
{
  if (depth >= cu.depth)
  {
    return CU_DONT_SPLIT;
  }

  const PartSplit cuSplitType = PartSplit( ( cu.splitSeries >> ( depth * SPLIT_DMULT ) ) & SPLIT_MASK );

  if (cuSplitType == CU_QUAD_SPLIT)
  {
    return CU_QUAD_SPLIT;
  }

  else if (cuSplitType == CU_HORZ_SPLIT)
  {
    return CU_HORZ_SPLIT;
  }

  else if (cuSplitType == CU_VERT_SPLIT)
  {
    return CU_VERT_SPLIT;
  }

  else if (cuSplitType == CU_TRIH_SPLIT)
  {
    return CU_TRIH_SPLIT;
  }
  else if (cuSplitType == CU_TRIV_SPLIT)
  {
    return CU_TRIV_SPLIT;
  }
  else
  {
    THROW("Unknown split mode");
    return CU_QUAD_SPLIT;
  }
}
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
ModeType CU::getModeTypeAtDepth( const CodingUnit& cu, const unsigned depth )
{
  ModeType modeType = ModeType( (cu.modeTypeSeries >> (depth * 3)) & 0x07 );
  CHECK( depth > cu.depth, " depth is wrong" );
  return modeType;
}
#endif


bool CU::divideTuInRows( const CodingUnit &cu )
{
  CHECK( cu.ispMode != HOR_INTRA_SUBPARTITIONS && cu.ispMode != VER_INTRA_SUBPARTITIONS, "Intra Subpartitions type not recognized!" );
  return cu.ispMode == HOR_INTRA_SUBPARTITIONS ? true : false;
}


PartSplit CU::getISPType( const CodingUnit &cu, const ComponentID compID )
{
  if( cu.ispMode && isLuma( compID ) )
  {
    const bool tuIsDividedInRows = CU::divideTuInRows( cu );

    return tuIsDividedInRows ? TU_1D_HORZ_SPLIT : TU_1D_VERT_SPLIT;
  }
  return TU_NO_ISP;
}

bool CU::isISPLast( const CodingUnit &cu, const CompArea &tuArea, const ComponentID compID )
{
  PartSplit partitionType = CU::getISPType( cu, compID );

  Area originalArea = cu.blocks[compID];
  switch( partitionType )
  {
    case TU_1D_HORZ_SPLIT:
      return tuArea.y + tuArea.height == originalArea.y + originalArea.height;
    case TU_1D_VERT_SPLIT:
      return tuArea.x + tuArea.width == originalArea.x + originalArea.width;
    default:
      THROW( "Unknown ISP processing order type!" );
      return false;
  }
}

bool CU::isISPFirst( const CodingUnit &cu, const CompArea &tuArea, const ComponentID compID )
{
  return tuArea == cu.firstTU->blocks[compID];
}

bool CU::canUseISP( const CodingUnit &cu, const ComponentID compID )
{
  const int width     = cu.blocks[compID].width;
  const int height    = cu.blocks[compID].height;
  const int maxTrSize = cu.cs->sps->getMaxTbSize();
  return CU::canUseISP( width, height, maxTrSize );
}

bool CU::canUseISP( const int width, const int height, const int maxTrSize )
{
  bool  notEnoughSamplesToSplit = ( floorLog2(width) + floorLog2(height) <= ( floorLog2(MIN_TB_SIZEY) << 1 ) );
  bool  cuSizeLargerThanMaxTrSize = width > maxTrSize || height > maxTrSize;
  if ( notEnoughSamplesToSplit || cuSizeLargerThanMaxTrSize )
  {
    return false;
  }
  return true;
}

bool CU::canUseLfnstWithISP( const CompArea& cuArea, const ISPType ispSplitType )
{
  if( ispSplitType == NOT_INTRA_SUBPARTITIONS )
  {
    return false;
  }
  Size tuSize = ( ispSplitType == HOR_INTRA_SUBPARTITIONS ) ? Size( cuArea.width, CU::getISPSplitDim( cuArea.width, cuArea.height, TU_1D_HORZ_SPLIT ) ) :
    Size( CU::getISPSplitDim( cuArea.width, cuArea.height, TU_1D_VERT_SPLIT ), cuArea.height );

  if( !( tuSize.width >= MIN_TB_SIZEY && tuSize.height >= MIN_TB_SIZEY ) )
  {
    return false;
  }
  return true;
}

bool CU::canUseLfnstWithISP( const CodingUnit& cu, const ChannelType chType )
{
  CHECK( !isLuma( chType ), "Wrong ISP mode!" );
  return CU::canUseLfnstWithISP( cu.blocks[chType == CHANNEL_TYPE_LUMA ? 0 : 1], (ISPType)cu.ispMode );
}

#if JVET_W0119_LFNST_EXTENSION
Size CU::getLfnstSize( const CodingUnit& cu, const ChannelType chType )
{
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  int chIdx = CS::isDualITree( *cu.cs ) && cu.chType == CHANNEL_TYPE_CHROMA ? 1 : 0;
#else
  int chIdx = cu.isSepTree() && cu.chType == CHANNEL_TYPE_CHROMA ? 1 : 0;
#endif
  Size lfnstSize;
  if( cu.ispMode )
  {
    CHECK( !isLuma( chType ), "Wrong ISP mode!" );
    const CompArea& cuArea = cu.blocks[ 0 ];
    lfnstSize = ( cu.ispMode == HOR_INTRA_SUBPARTITIONS ) ? Size( cuArea.width, CU::getISPSplitDim( cuArea.width, cuArea.height, TU_1D_HORZ_SPLIT ) ) :
                Size( CU::getISPSplitDim( cuArea.width, cuArea.height, TU_1D_VERT_SPLIT ), cuArea.height );
  }
  else
  {
    lfnstSize.width  = cu.blocks[ chIdx ].lumaSize().width;
    lfnstSize.height = cu.blocks[ chIdx ].lumaSize().height;
  }
  return lfnstSize;
}
#endif

uint32_t CU::getISPSplitDim( const int width, const int height, const PartSplit ispType )
{
  bool divideTuInRows = ispType == TU_1D_HORZ_SPLIT;
  uint32_t splitDimensionSize, nonSplitDimensionSize, partitionSize, divShift = 2;

  if( divideTuInRows )
  {
    splitDimensionSize    = height;
    nonSplitDimensionSize = width;
  }
  else
  {
    splitDimensionSize    = width;
    nonSplitDimensionSize = height;
  }

  const int minNumberOfSamplesPerCu = 1 << ( ( floorLog2(MIN_TB_SIZEY) << 1 ) );
  const int factorToMinSamples = nonSplitDimensionSize < minNumberOfSamplesPerCu ? minNumberOfSamplesPerCu >> floorLog2(nonSplitDimensionSize) : 1;
  partitionSize = ( splitDimensionSize >> divShift ) < factorToMinSamples ? factorToMinSamples : ( splitDimensionSize >> divShift );

  CHECK( floorLog2(partitionSize) + floorLog2(nonSplitDimensionSize) < floorLog2(minNumberOfSamplesPerCu), "A partition has less than the minimum amount of samples!" );
  return partitionSize;
}

bool CU::allLumaCBFsAreZero(const CodingUnit& cu)
{
  if (!cu.ispMode)
  {
    return TU::getCbf(*cu.firstTU, COMPONENT_Y) == false;
  }
  else
  {
    int numTotalTUs = cu.ispMode == HOR_INTRA_SUBPARTITIONS ? cu.lheight() >> floorLog2(cu.firstTU->lheight()) : cu.lwidth() >> floorLog2(cu.firstTU->lwidth());
    TransformUnit* tuPtr = cu.firstTU;
    for (int tuIdx = 0; tuIdx < numTotalTUs; tuIdx++)
    {
      if (TU::getCbf(*tuPtr, COMPONENT_Y) == true)
      {
        return false;
      }
      tuPtr = tuPtr->next;
    }
    return true;
  }
}

#if JVET_W0123_TIMD_FUSION
TemplateType CU::deriveTimdRefType( int iCurX, int iCurY, uint32_t uiCurWidth, uint32_t uiCurHeight, int iTemplateWidth, int iTemplateHeight, int& iRefX, int& iRefY, uint32_t& uiRefWidth, uint32_t& uiRefHeight )
{
  if(iCurX == 0 && iCurY == 0)
  {
    return NO_NEIGHBOR;
  }

  TemplateType eTempType = NO_NEIGHBOR;
  iRefX = iRefY = -1;
  if(iCurX > 0 && iCurY > 0)
  {
    iRefX       = iCurX - iTemplateWidth;
    iRefY       = iCurY - iTemplateHeight;
    uiRefWidth  = uiCurWidth + iTemplateWidth;
    uiRefHeight = uiCurHeight + iTemplateHeight;
    eTempType   = LEFT_ABOVE_NEIGHBOR;
  }
  else if(iCurX == 0 && iCurY > 0)
  {
    iRefX       = iCurX;
    iRefY       = iCurY - iTemplateHeight;
    uiRefWidth  = uiCurWidth;
    uiRefHeight = uiCurHeight;
    eTempType   = ABOVE_NEIGHBOR;
  }
  else if(iCurX > 0 && iCurY == 0)
  {
    iRefX       = iCurX - iTemplateWidth;
    iRefY       = iCurY;
    uiRefWidth  = uiCurWidth;
    uiRefHeight = uiCurHeight;
    eTempType   = LEFT_NEIGHBOR;
  }
  else
  {
    assert(0);
  }
  return eTempType;
}
#endif


PUTraverser CU::traversePUs( CodingUnit& cu )
{
  return PUTraverser( cu.firstPU, cu.lastPU->next );
}

TUTraverser CU::traverseTUs( CodingUnit& cu )
{
  return TUTraverser( cu.firstTU, cu.lastTU->next );
}

cPUTraverser CU::traversePUs( const CodingUnit& cu )
{
  return cPUTraverser( cu.firstPU, cu.lastPU->next );
}

cTUTraverser CU::traverseTUs( const CodingUnit& cu )
{
  return cTUTraverser( cu.firstTU, cu.lastTU->next );
}

#if JVET_AE0059_INTER_CCCM
bool CU::interCccmSearchAllowed(const CodingUnit& cu)
{
  if (!cu.cs->slice->getSPS()->getUseInterCccm())
  {
    return false;
  }
  if (cu.predMode != MODE_INTER && cu.predMode != MODE_IBC)
  {
    return false;
  }
  if (cu.blocks[COMPONENT_Cb].area() < 16)
  {
    return false;
  }
  if (!cu.firstTU->blocks[COMPONENT_Cb].valid())
  {
    return false;
  }
  if (!cu.firstTU->blocks[COMPONENT_Cr].valid())
  {
    return false;
  }
  return true;
}
#endif

#if JVET_AF0073_INTER_CCP_MERGE
bool CU::interCcpMergeSearchAllowed(const CodingUnit& cu)
{
  if (!cu.cs->slice->getSPS()->getUseInterCcpMerge())
  {
    return false;
  }
  if (cu.predMode != MODE_INTER)
  {
    return false;
  }
  if (cu.blocks[COMPONENT_Cb].area() < 16)
  {
    return false;
  }
  if (cu.firstTU->blocks[COMPONENT_Cb].width != cu.blocks[COMPONENT_Cb].width || cu.firstTU->blocks[COMPONENT_Cb].height != cu.blocks[COMPONENT_Cb].height)
  {
    return false;
  }
  if (!cu.firstTU->blocks[COMPONENT_Cb].valid())
  {
    return false;
  }
  if (!cu.firstTU->blocks[COMPONENT_Cr].valid())
  {
    return false;
  }
  if (cu.blocks[COMPONENT_Cb].area() > 1024)
  {
    return false;
  }
  if (!cu.firstPU->mergeFlag)
  {
    return false;
  }
  return true;
}
#endif

#if JVET_AC0094_REF_SAMPLES_OPT
void CU::getNbModesRemovedFirstLast(const bool &areAboveRightUnavail, const bool &areBelowLeftUnavail, const SizeType &height, const SizeType &width, int &nbRemovedFirst, int &nbRemovedLast)
{
  nbRemovedFirst = 0;
  nbRemovedLast  = 0;
  if (areAboveRightUnavail)
  {
    if (width > height)
    {
      const int deltaSize{ abs(floorLog2(width) - floorLog2(height)) };
      if (deltaSize == 1)
      {
        /*
        The first 6 horizontal positive modes subtituted via
        the wide-angle process are removed.
        */
        nbRemovedFirst += 6;
      }
      else if (deltaSize == 2)
      {
        nbRemovedFirst += 10;
      }
      else
      {
        nbRemovedFirst += 12;
      }
    }
    else if (height == width)
    {
      /*
      The last `NUM_MODES_REMOVED_POSITIVE` vertical positive
      modes are removed.
      */
      nbRemovedLast += NUM_MODES_REMOVED_POSITIVE;
    }

    /*
    If none of the decoded reference samples on the above-right
    side of the current luminance CB is available and `height`
    is strictly larger than `width`, the wide-angle process
    results in an underuse of these padded decoded reference
    samples.
    */
  }
  if (areBelowLeftUnavail)
  {
    if (height > width)
    {
      const int deltaSize{ abs(floorLog2(height) - floorLog2(width)) };
      if (deltaSize == 1)
      {
        /*
        The last 6 vertical positive modes subtituted via
        the wide-angle process are removed.
        */
        nbRemovedLast += 6;
      }
      else if (deltaSize == 2)
      {
        nbRemovedLast += 10;
      }
      else
      {
        nbRemovedLast += 12;
      }
    }
    else if (height == width)
    {
      /*
      The first `NUM_MODES_REMOVED_POSITIVE` horizontal positive
      modes are removed.
      */
      nbRemovedFirst += NUM_MODES_REMOVED_POSITIVE;
    }

    /*
    If none of the decoded reference samples on the below-left
    side of the current luminance CB is available and `width`
    is strictly larger than `height`, the wide-angle process
    results in an underuse of these padded decoded reference
    samples.
    */
  }
}

bool CU::isIdxModeValid(const bool &areAboveRightUnavail, const bool &areBelowLeftUnavail,
                        const SizeType &height, const SizeType &width, const SizeType &idx_mode_tested,
                        const bool &isForcedValid)
{
  CHECK(idx_mode_tested < PLANAR_IDX || idx_mode_tested >= NUM_LUMA_MODE,
        "`idx_mode_tested` does not belong to [|" + std::to_string(PLANAR_IDX) + ", "
          + std::to_string(NUM_LUMA_MODE - 1) + "|].");
  if (isForcedValid || idx_mode_tested == PLANAR_IDX || idx_mode_tested == DC_IDX)
  {
    return true;
  }
  else
  {
    int nbRemovedFirst{ 0 };
    int nbRemovedLast{ 0 };
    getNbModesRemovedFirstLast(areAboveRightUnavail, areBelowLeftUnavail, height, width, nbRemovedFirst,
                               nbRemovedLast);
    return idx_mode_tested > DC_IDX + nbRemovedFirst && idx_mode_tested < NUM_LUMA_MODE - nbRemovedLast;
  }
}
#endif
// PU tools
#if (JVET_AG0146_DIMD_ITMP_IBC || JVET_AG0152_SGPM_ITMP_IBC || JVET_AG0151_INTRA_TMP_MERGE_MODE)
bool CheckBvAvailable(std::vector<Mv>& pBv, Mv curBv)
{
  for (int i = 0; i < pBv.size(); i++)
  {
    if (pBv[i] == curBv)
    {
      return 1;
    }
  }
  return 0;
}

void getNeighBv(const PredictionUnit& puOrg, const PredictionUnit* pu, std::vector<Mv>& pBv)
{
  if (!pu || ((pu->cu->predMode != MODE_IBC) && (!pu->cu->tmpFlag)))
  {
    return;
  }
  if (pu && pu->cu->predMode == MODE_IBC)
  {
    if (PU::validItmpBv(puOrg, pu->bv.hor, pu->bv.ver))
    {
      if (!CheckBvAvailable(pBv, pu->bv))
      {
        pBv.push_back(pu->bv);
      }
    }

    if (pu->interDir == 3)
    {
      Mv bv(pu->mv[REF_PIC_LIST_1]);
      bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
      if (PU::validItmpBv(puOrg, bv.hor, bv.ver))
      {
        if (!CheckBvAvailable(pBv, bv))
        {
          pBv.push_back(bv);
        }
      }
    }

    return;
  }

  if (pu && pu->cu->tmpFlag)
  {
    if (PU::validItmpBv(puOrg, pu->bv.hor, pu->bv.ver))
    {
      if (!CheckBvAvailable(pBv, pu->bv))
      {
        pBv.push_back(pu->bv);
      }
    }
    if (pu->cu->tmpIdx > 0
#if JVET_AG0136_INTRA_TMP_LIC
      && !pu->cu->tmpLicFlag
#endif
      )
    {
      Mv bv(pu->cu->tmpXdisp, pu->cu->tmpYdisp);

      if (PU::validItmpBv(puOrg, bv.hor, bv.ver))
      {
        if (!CheckBvAvailable(pBv, bv))
        {
          pBv.push_back(bv);
        }
      }
    }
    return;
  }
}

int PU::getItmpMergeCandidate(const PredictionUnit& pu, std::vector<Mv>& pBvs)
{
  const CompArea area = pu.Y();
  const Position topLeft = area.topLeft();
  const ChannelType ch = pu.chType;

  getNeighBv(pu, (pu.cs->getPURestricted(topLeft.offset(-1, pu.Y().height - 1), pu, ch)), pBvs);
  getNeighBv(pu, (pu.cs->getPURestricted(topLeft.offset(pu.Y().width - 1, -1), pu, ch)), pBvs);
  getNeighBv(pu, (pu.cs->getPURestricted(topLeft.offset(-1, -1), pu, ch)), pBvs);
  getNeighBv(pu, (pu.cs->getPURestricted(topLeft.offset(pu.Y().width, -1), pu, ch)), pBvs);
  getNeighBv(pu, (pu.cs->getPURestricted(topLeft.offset(-1, pu.Y().height), pu, ch)), pBvs);

  int offsetX = 0;
  int offsetY = 0;
  const int numNACandidate[4] = { 3, 5, 5, 5 };
  const int idxMap[4][5] = { { 0, 1, 4 },{ 0, 1, 2, 3, 4 },{ 0, 1, 2, 3, 4 },{ 0, 1, 2, 3, 4 } };
  for (int iDistanceIndex = 0; iDistanceIndex < NADISTANCE_LEVEL; iDistanceIndex++)
  {
    const int iNADistanceHor = pu.Y().width * (iDistanceIndex + 1);
    const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);

    for (int iNASPIdx = 0; iNASPIdx < numNACandidate[iDistanceIndex]; iNASPIdx++)
    {
      switch (idxMap[iDistanceIndex][iNASPIdx])
      {
      case 0:offsetX = -iNADistanceHor - 1; offsetY = pu.Y().height + iNADistanceVer - 1; break;
      case 1:offsetX = pu.Y().width + iNADistanceHor - 1; offsetY = -iNADistanceVer - 1; break;
      case 2:offsetX = pu.Y().width >> 1;   offsetY = -iNADistanceVer - 1;    break;
      case 3:offsetX = -iNADistanceHor - 1; offsetY = pu.Y().height >> 1; break;
      case 4:offsetX = -iNADistanceHor - 1; offsetY = -iNADistanceVer - 1;    break;
      default: printf("error!"); exit(0); break;
      }
      getNeighBv(pu, pu.cs->getPURestricted(topLeft.offset(offsetX, offsetY), pu, ch), pBvs);
    }
  }

  return (int)pBvs.size();
}

bool PU::checkValidIntraTmpMergeCand(const PredictionUnit& pu, Mv Bv)
{
  return validItmpBv(pu, Bv.hor, Bv.ver);
}

bool PU::validItmpBv(const PredictionUnit& pu, int tmpXdisp, int tmpYdisp)
{
  const int x = pu.lx();
  const int y = pu.ly();
  const int w = pu.lwidth();
  const int h = pu.lheight();
  const Position p1(x + tmpXdisp + w - 1, y + tmpYdisp + h - 1);
  if (!pu.cs->isDecomp(p1, CHANNEL_TYPE_LUMA))
  {
    return 0;
  }
  const Position p2(x + tmpXdisp - TMP_TEMPLATE_SIZE, y + tmpYdisp - TMP_TEMPLATE_SIZE);
  if (!pu.cs->isDecomp(p2, CHANNEL_TYPE_LUMA))
  {
    return 0;
  }
  return 1;
}
#endif

#if SECONDARY_MPM
int PU::getIntraMPMs( const PredictionUnit &pu, uint8_t* mpm, uint8_t* non_mpm
#if JVET_AC0094_REF_SAMPLES_OPT
                    , const bool& isForcedValid
#endif
#if JVET_AD0085_MPM_SORTING
                    , IntraPrediction* pIntraPred/* = nullptr*/
#endif
                    , const ChannelType &channelType /*= CHANNEL_TYPE_LUMA*/)
#else
int PU::getIntraMPMs(const PredictionUnit &pu, unsigned* mpm, const ChannelType &channelType /*= CHANNEL_TYPE_LUMA*/)
#endif
{
#if JVET_AD0085_TMRL_EXTENSION && SECONDARY_MPM
  int numCand = 0;
  mpm[numCand++] = PLANAR_IDX;
#if JVET_AD0085_MPM_SORTING
  int maxCand = pIntraPred ? NUM_PRIMARY_MOST_PROBABLE_MODES + 1 : NUM_MOST_PROBABLE_MODES - 1;
#else
  int maxCand = NUM_MOST_PROBABLE_MODES - 1;
#endif
  numCand += getSpatialIpm(pu, mpm + 1, maxCand
#if JVET_AC0094_REF_SAMPLES_OPT
                         , isForcedValid
#endif
                         , false
#if JVET_AD0085_MPM_SORTING
                         , pIntraPred
#endif
  );

  fillMPMList(pu, mpm, NUM_MOST_PROBABLE_MODES, numCand);
#if JVET_AD0085_MPM_SORTING
  if (!pu.cs->pcv->isEncoder && (pu.mpmFlag || pu.secondMpmFlag))
  {
    return numCand;
  }
#endif
  fillNonMPMList(mpm, non_mpm);

  return numCand;
#else
#if SECONDARY_MPM
  bool includedMode[NUM_INTRA_MODE];
  memset(includedMode, false, sizeof(includedMode));
#if JVET_AC0094_REF_SAMPLES_OPT
  uint8_t arrayReserved[5];
  int     nbReserved{ 0 };
#endif

  int numValidMPM = 0;
  mpm[numValidMPM++] = PLANAR_IDX;
  includedMode[PLANAR_IDX] = true;
#endif

  const int numMPMs = NUM_MOST_PROBABLE_MODES;
  {
    CHECK(channelType != CHANNEL_TYPE_LUMA, "Not harmonized yet");
    int numCand      = -1;
#if !SECONDARY_MPM
    int leftIntraDir = PLANAR_IDX, aboveIntraDir = PLANAR_IDX;
#endif

    const CompArea &area = pu.block(getFirstComponentOfChannel(channelType));
    const Position posRT = area.topRight();
    const Position posLB = area.bottomLeft();

    // Get intra direction of left PU
#if SECONDARY_MPM
    const PredictionUnit *puLeft = (pu.lheight() >= pu.lwidth())
      ? pu.cs->getPURestricted(posRT.offset(0, -1), pu, channelType)
      : pu.cs->getPURestricted(posLB.offset(-1, 0), pu, channelType);
#else
    const PredictionUnit *puLeft = pu.cs->getPURestricted(posLB.offset(-1, 0), pu, channelType);
#endif
    if (puLeft && CU::isIntra(*puLeft->cu))
    {
#if SECONDARY_MPM
#if JVET_W0123_TIMD_FUSION
      mpm[numValidMPM] = puLeft->cu->timd ? MAP131TO67(PU::getIntraDirLuma(*puLeft)) : PU::getIntraDirLuma(*puLeft);
#else
      mpm[numValidMPM] = PU::getIntraDirLuma(*puLeft);
#endif
      if( !includedMode[mpm[numValidMPM]] )
      {
#if JVET_AC0094_REF_SAMPLES_OPT
        if ( CU::isIdxModeValid((pu.cu)->areAboveRightUnavail, (pu.cu)->areBelowLeftUnavail, (pu.cu)->lheight(), (pu.cu)->lwidth(), mpm[numValidMPM], isForcedValid) )
        {
#endif
        includedMode[mpm[numValidMPM++]] = true;
#if JVET_AC0094_REF_SAMPLES_OPT
        }
        else
        {
          arrayReserved[nbReserved] = mpm[numValidMPM];
          nbReserved++;
        }
#endif
      }
#else
      leftIntraDir = PU::getIntraDirLuma( *puLeft );
#if JVET_W0123_TIMD_FUSION
      if (puLeft->cu->timd)
      {
        leftIntraDir = MAP131TO67(leftIntraDir);
      }
#endif
#endif
    }
#if JVET_W0123_TIMD_FUSION && !SECONDARY_MPM
    if (puLeft && CU::isInter(*puLeft->cu))
    {
      leftIntraDir = puLeft->getIpmInfo(posLB.offset(-1, 0));
    }
#endif

    // Get intra direction of above PU
#if SECONDARY_MPM
    const PredictionUnit *puAbove = (pu.lheight() >= pu.lwidth())
      ? pu.cs->getPURestricted(posLB.offset(-1, 0), pu, channelType)
      : pu.cs->getPURestricted(posRT.offset(0, -1), pu, channelType);
#else
    const PredictionUnit *puAbove = pu.cs->getPURestricted(posRT.offset(0, -1), pu, channelType);
#endif
    if (puAbove && CU::isIntra(*puAbove->cu) && CU::isSameCtu(*pu.cu, *puAbove->cu))
    {
#if SECONDARY_MPM
#if JVET_W0123_TIMD_FUSION
      mpm[numValidMPM] = puAbove->cu->timd ? MAP131TO67(PU::getIntraDirLuma(*puAbove)) : PU::getIntraDirLuma(*puAbove);
#else
      mpm[numValidMPM] = PU::getIntraDirLuma(*puAbove);
#endif
      if (!includedMode[mpm[numValidMPM]])
      {
#if JVET_AC0094_REF_SAMPLES_OPT
        if ( CU::isIdxModeValid((pu.cu)->areAboveRightUnavail, (pu.cu)->areBelowLeftUnavail, (pu.cu)->lheight(), (pu.cu)->lwidth(), mpm[numValidMPM], isForcedValid) )
        {
#endif
        includedMode[mpm[numValidMPM++]] = true;
#if JVET_AC0094_REF_SAMPLES_OPT
        }
        else
        {
          arrayReserved[nbReserved] = mpm[numValidMPM];
          nbReserved++;
        }
#endif
      }
#else
      aboveIntraDir = PU::getIntraDirLuma(*puAbove);
#if JVET_W0123_TIMD_FUSION
      if (puAbove->cu->timd)
      {
        aboveIntraDir = MAP131TO67(aboveIntraDir);
      }
#endif
#endif
    }
#if JVET_W0123_TIMD_FUSION && !SECONDARY_MPM
    if (puAbove && CU::isInter(*puAbove->cu))
    {
      aboveIntraDir = puAbove->getIpmInfo(posRT.offset(0, -1));
    }
#endif

#if SECONDARY_MPM
#if JVET_W0123_TIMD_FUSION
#if JVET_AC0112_IBC_CIIP
    if (puLeft && (CU::isInter(*puLeft->cu) || CU::isIBC(*puLeft->cu)))
#else
    if (puLeft && CU::isInter(*puLeft->cu))
#endif
    {
      mpm[numValidMPM] = puLeft->getIpmInfo(pu.lheight() >= pu.lwidth() ? posRT.offset(0, -1) : posLB.offset(-1, 0));
      if( !includedMode[mpm[numValidMPM]] )
      {
#if JVET_AC0094_REF_SAMPLES_OPT
        if ( CU::isIdxModeValid((pu.cu)->areAboveRightUnavail, (pu.cu)->areBelowLeftUnavail, (pu.cu)->lheight(), (pu.cu)->lwidth(), mpm[numValidMPM], isForcedValid) )
        {
#endif
        includedMode[mpm[numValidMPM++]] = true;
#if JVET_AC0094_REF_SAMPLES_OPT
        }
        else
        {
          arrayReserved[nbReserved] = mpm[numValidMPM];
          nbReserved++;
        }
#endif
      }
    }


#if JVET_AC0112_IBC_CIIP
    if (puAbove && (CU::isInter(*puAbove->cu) || CU::isIBC(*puAbove->cu)))
#else
    if (puAbove && CU::isInter(*puAbove->cu))
#endif
    {
      mpm[numValidMPM] = puAbove->getIpmInfo(pu.lheight() >= pu.lwidth() ? posLB.offset(-1, 0) : posRT.offset(0, -1));
      if( !includedMode[mpm[numValidMPM]] )
      {
#if JVET_AC0094_REF_SAMPLES_OPT
        if ( CU::isIdxModeValid((pu.cu)->areAboveRightUnavail, (pu.cu)->areBelowLeftUnavail, (pu.cu)->lheight(), (pu.cu)->lwidth(), mpm[numValidMPM], isForcedValid) )
        {
#endif
        includedMode[mpm[numValidMPM++]] = true;
#if JVET_AC0094_REF_SAMPLES_OPT
        }
        else
        {
          arrayReserved[nbReserved] = mpm[numValidMPM];
          nbReserved++;
        }
#endif
      }
    }
#endif
    // Get intra direction of below-left PU
    const PredictionUnit *puBelowLeft = pu.cs->getPURestricted(posLB.offset(-1, 1), pu, channelType);
    if (puBelowLeft && CU::isIntra(*puBelowLeft->cu))
    {
#if JVET_W0123_TIMD_FUSION
      mpm[numValidMPM] = puBelowLeft->cu->timd ? MAP131TO67(PU::getIntraDirLuma(*puBelowLeft)) : PU::getIntraDirLuma(*puBelowLeft);
#else
      mpm[numValidMPM] = PU::getIntraDirLuma(*puBelowLeft);
#endif
      if( !includedMode[mpm[numValidMPM]] )
      {
#if JVET_AC0094_REF_SAMPLES_OPT
        if ( CU::isIdxModeValid((pu.cu)->areAboveRightUnavail, (pu.cu)->areBelowLeftUnavail, (pu.cu)->lheight(), (pu.cu)->lwidth(), mpm[numValidMPM], isForcedValid) )
        {
#endif
        includedMode[mpm[numValidMPM++]] = true;
#if JVET_AC0094_REF_SAMPLES_OPT
        }
        else
        {
          arrayReserved[nbReserved] = mpm[numValidMPM];
          nbReserved++;
        }
#endif
      }
    }

    // Get intra direction of above-right PU
    const PredictionUnit *puAboveRight = pu.cs->getPURestricted(posRT.offset(1, -1), pu, channelType);
    if (puAboveRight && CU::isIntra(*puAboveRight->cu) && CU::isSameCtu(*pu.cu, *puAboveRight->cu))
    {
#if JVET_W0123_TIMD_FUSION
      mpm[numValidMPM] = puAboveRight->cu->timd ? MAP131TO67(PU::getIntraDirLuma(*puAboveRight)) : PU::getIntraDirLuma(*puAboveRight);
#else
      mpm[numValidMPM] = PU::getIntraDirLuma(*puAboveRight);
#endif
      if( !includedMode[mpm[numValidMPM]] )
      {
#if JVET_AC0094_REF_SAMPLES_OPT
        if ( CU::isIdxModeValid((pu.cu)->areAboveRightUnavail, (pu.cu)->areBelowLeftUnavail, (pu.cu)->lheight(), (pu.cu)->lwidth(), mpm[numValidMPM], isForcedValid) )
        {
#endif
        includedMode[mpm[numValidMPM++]] = true;
#if JVET_AC0094_REF_SAMPLES_OPT
        }
        else
        {
          arrayReserved[nbReserved] = mpm[numValidMPM];
          nbReserved++;
        }
#endif
      }
    }

    // Get intra direction of above-left PU
    const Position posTL = area.topLeft();
    const PredictionUnit *puAboveLeft = pu.cs->getPURestricted(posTL.offset(-1, -1), pu, channelType);
    if (puAboveLeft && CU::isIntra(*puAboveLeft->cu) && CU::isSameCtu(*pu.cu, *puAboveLeft->cu))
    {
#if JVET_W0123_TIMD_FUSION
      mpm[numValidMPM] = puAboveLeft->cu->timd ? MAP131TO67(PU::getIntraDirLuma(*puAboveLeft)) : PU::getIntraDirLuma(*puAboveLeft);
#else
      mpm[numValidMPM] = PU::getIntraDirLuma(*puAboveLeft);
#endif
      if( !includedMode[mpm[numValidMPM]] )
      {
#if JVET_AC0094_REF_SAMPLES_OPT
        if ( CU::isIdxModeValid((pu.cu)->areAboveRightUnavail, (pu.cu)->areBelowLeftUnavail, (pu.cu)->lheight(), (pu.cu)->lwidth(), mpm[numValidMPM], isForcedValid) )
        {
#endif
        includedMode[mpm[numValidMPM++]] = true;
#if JVET_AC0094_REF_SAMPLES_OPT
        }
        else
        {
          arrayReserved[nbReserved] = mpm[numValidMPM];
          nbReserved++;
        }
#endif
      }
    }
#if JVET_W0123_TIMD_FUSION
#if JVET_AC0112_IBC_CIIP
    if (puBelowLeft && (CU::isInter(*puBelowLeft->cu) || CU::isIBC(*puBelowLeft->cu)))
#else
    if (puBelowLeft && CU::isInter(*puBelowLeft->cu))
#endif
    {
      mpm[numValidMPM] = puBelowLeft->getIpmInfo(posLB.offset(-1, 1));
      if( !includedMode[mpm[numValidMPM]] )
      {
#if JVET_AC0094_REF_SAMPLES_OPT
        if ( CU::isIdxModeValid((pu.cu)->areAboveRightUnavail, (pu.cu)->areBelowLeftUnavail, (pu.cu)->lheight(), (pu.cu)->lwidth(), mpm[numValidMPM], isForcedValid) )
        {
#endif
        includedMode[mpm[numValidMPM++]] = true;
#if JVET_AC0094_REF_SAMPLES_OPT
        }
        else
        {
          arrayReserved[nbReserved] = mpm[numValidMPM];
          nbReserved++;
        }
#endif
      }
    }


#if JVET_AC0112_IBC_CIIP
    if (puAboveRight && (CU::isInter(*puAboveRight->cu) || CU::isIBC(*puAboveRight->cu)))
#else

    if (puAboveRight && CU::isInter(*puAboveRight->cu))
#endif
    {
      mpm[numValidMPM] = puAboveRight->getIpmInfo(posRT.offset(1, -1));
      if( !includedMode[mpm[numValidMPM]] )
      {
#if JVET_AC0094_REF_SAMPLES_OPT
        if ( CU::isIdxModeValid((pu.cu)->areAboveRightUnavail, (pu.cu)->areBelowLeftUnavail, (pu.cu)->lheight(), (pu.cu)->lwidth(), mpm[numValidMPM], isForcedValid) )
        {
#endif
        includedMode[mpm[numValidMPM++]] = true;
#if JVET_AC0094_REF_SAMPLES_OPT
        }
        else
        {
          arrayReserved[nbReserved] = mpm[numValidMPM];
          nbReserved++;
        }
#endif
      }
    }


#if JVET_AC0112_IBC_CIIP
    if (puAboveLeft && (CU::isInter(*puAboveLeft->cu) || CU::isIBC(*puAboveLeft->cu)))
#else
    if (puAboveLeft && CU::isInter(*puAboveLeft->cu))
#endif
    {
      mpm[numValidMPM] = puAboveLeft->getIpmInfo(posTL.offset(-1, -1));
      if( !includedMode[mpm[numValidMPM]] )
      {
#if JVET_AC0094_REF_SAMPLES_OPT
        if ( CU::isIdxModeValid((pu.cu)->areAboveRightUnavail, (pu.cu)->areBelowLeftUnavail, (pu.cu)->lheight(), (pu.cu)->lwidth(), mpm[numValidMPM], isForcedValid) )
        {
#endif
        includedMode[mpm[numValidMPM++]] = true;
#if JVET_AC0094_REF_SAMPLES_OPT
        }
        else
        {
          arrayReserved[nbReserved] = mpm[numValidMPM];
          nbReserved++;
        }
#endif
      }
    }
#endif
#endif
#if JVET_AC0094_REF_SAMPLES_OPT
    if (!isForcedValid)
    {
      for (int i{ 0 }; i < nbReserved; i++)
      {
        if (!includedMode[arrayReserved[i]])
        {
          mpm[numValidMPM]                 = arrayReserved[i];
          includedMode[mpm[numValidMPM++]] = true;
        }
      }
    }
#endif
#if JVET_AD0085_MPM_SORTING
    if (pu.cs->sps->getUseMpmSorting())
    {
#if ENABLE_DIMD
      //adding dimd modes
      if (pu.cu->slice->getSPS()->getUseDimd())
      {
        if (pu.cu->dimdMode != -1)
        {
          mpm[numValidMPM] = pu.cu->dimdMode;
          if (!includedMode[mpm[numValidMPM]])
          {
            includedMode[mpm[numValidMPM++]] = true;
          }

          for (int i = 0; i < DIMD_FUSION_NUM - 2; i++)
          {
            if (pu.cu->dimdBlendMode[i] != -1)
            {
              mpm[numValidMPM] = pu.cu->dimdBlendMode[i];
              if (!includedMode[mpm[numValidMPM]])
              {
                includedMode[mpm[numValidMPM++]] = true;
              }
            }
          }
        }
      }
#endif

      numCand = numValidMPM;

      if (pIntraPred && numValidMPM > 2)
      {
        pIntraPred->deriveMPMSorted(pu, mpm, numValidMPM, 1);

        if (!pu.cs->pcv->isEncoder && pu.mpmFlag && pu.ipredIdx < numValidMPM)
        {
          return numCand;
        }

        numCand = numValidMPM;
        memset(includedMode, false, sizeof(includedMode));
        for (int i = 0; i < numValidMPM; i++)
        {
          includedMode[mpm[i]] = true;
        }
      }
    }
#endif

    CHECK(2 >= numMPMs, "Invalid number of most probable modes");

    const int offset = ( int ) NUM_LUMA_MODE - 6;
    const int mod = offset + 3;

    {
#if JVET_AD0085_MPM_SORTING
      if (!pu.cs->sps->getUseMpmSorting())
      {
#endif
#if SECONDARY_MPM
      numCand = numValidMPM;
#else
      mpm[0] = PLANAR_IDX;
      mpm[1] = DC_IDX;
      mpm[2] = VER_IDX;
      mpm[3] = HOR_IDX;
      mpm[4] = VER_IDX - 4;
      mpm[5] = VER_IDX + 4;
#endif
#if ENABLE_DIMD && SECONDARY_MPM
      //adding dimd modes
      if (pu.cu->slice->getSPS()->getUseDimd())
      {
        if (pu.cu->dimdMode != -1)
        {
          mpm[numValidMPM] = pu.cu->dimdMode;
          if( !includedMode[mpm[numValidMPM]] )
          {
            includedMode[mpm[numValidMPM++]] = true;
          }
        }

        if (pu.cu->dimdBlendMode[0] != -1)
        {
          mpm[numValidMPM] = pu.cu->dimdBlendMode[0];
          if( !includedMode[mpm[numValidMPM]] )
          {
            includedMode[mpm[numValidMPM++]] = true;
          }
        }
      }
#endif
#if JVET_AD0085_MPM_SORTING
      }
#endif

#if SECONDARY_MPM
#if JVET_AD0085_MPM_SORTING
      if (pu.cs->sps->getUseMpmSorting())
      {
        for (int i = 0; i < numCand && numValidMPM < numMPMs; i++)
        {
          if (mpm[i] <= DC_IDX)
          {
            continue;
          }

          for (int deltaAngular = 0; deltaAngular < 4 && numValidMPM < numMPMs; deltaAngular++)
          {
            // try to fill mode - (delta + 1)
            mpm[numValidMPM] = ((mpm[i] + offset - deltaAngular) % mod) + 2;
            if (!includedMode[mpm[numValidMPM]])
            {
              includedMode[mpm[numValidMPM++]] = true;
            }

            if (numValidMPM >= numMPMs)
            {
              break;
            }

            // try to fill mode + delta + 1
            mpm[numValidMPM] = ((mpm[i] - 1 + deltaAngular) % mod) + 2;
            if (!includedMode[mpm[numValidMPM]])
            {
              includedMode[mpm[numValidMPM++]] = true;
            }
          }
        }
      }
      else
      {
#endif
      bool checkDCEnabled = false;

      // Derived modes of mpm[1]
      if (numCand >= 2)
      {
        if (mpm[1] > DC_IDX)
        {
          for (int i = 0; i < 4 && numValidMPM < numMPMs; i++)
          {
            mpm[numValidMPM] = ((mpm[1] + offset - i) % mod) + 2;
            if (!includedMode[mpm[numValidMPM]])
            {
              includedMode[mpm[numValidMPM++]] = true;
            }

            if (numValidMPM >= numMPMs)
            {
              break;
            }

            mpm[numValidMPM] = ((mpm[1] - 1 + i) % mod) + 2;
            if (!includedMode[mpm[numValidMPM]])
            {
              includedMode[mpm[numValidMPM++]] = true;
            }
          }
        }
        else if (mpm[1] == DC_IDX)
        {
          checkDCEnabled = true;
        }
      }


      // Derived modes of mpm[2]
      if (numCand >= 3)
      {
        if (mpm[2] > DC_IDX)
        {
          for (int i = 0; i < 4 && numValidMPM < numMPMs; i++)
          {
            mpm[numValidMPM] = ((mpm[2] + offset - i) % mod) + 2;
            if (!includedMode[mpm[numValidMPM]])
            {
              includedMode[mpm[numValidMPM++]] = true;
            }

            if (numValidMPM >= numMPMs)
              break;

            mpm[numValidMPM] = ((mpm[2] - 1 + i) % mod) + 2;
            if (!includedMode[mpm[numValidMPM]])
            {
              includedMode[mpm[numValidMPM++]] = true;
            }
          }
        }
        else if (mpm[2] == DC_IDX)
        {
          checkDCEnabled = true;
        }
      }


      // Derived modes of mpm[3]
      if (checkDCEnabled && numCand >= 4 && mpm[3] > DC_IDX)
      {
        for (int i = 0; i < 3 && numValidMPM < numMPMs; i++)
        {
          mpm[numValidMPM] = ((mpm[3] + offset - i) % mod) + 2;
          if (!includedMode[mpm[numValidMPM]])
          {
            includedMode[mpm[numValidMPM++]] = true;
          }

          if (numValidMPM >= numMPMs)
          {
            break;
          }

          mpm[numValidMPM] = ((mpm[3] - 1 + i) % mod) + 2;
          if (!includedMode[mpm[numValidMPM]])
          {
            includedMode[mpm[numValidMPM++]] = true;
          }
        }
      }
#if JVET_AD0085_MPM_SORTING
      }
#endif
#else
      if (leftIntraDir == aboveIntraDir)
      {
        numCand = 1;
        if (leftIntraDir > DC_IDX)
        {
          mpm[0] = PLANAR_IDX;
          mpm[1] = leftIntraDir;
          mpm[2] = ((leftIntraDir + offset) % mod) + 2;
          mpm[3] = ((leftIntraDir - 1) % mod) + 2;
          mpm[4] = ((leftIntraDir + offset - 1) % mod) + 2;
          mpm[5] = ( leftIntraDir               % mod) + 2;
        }
      }
      else //L!=A
      {
        numCand = 2;
        int  maxCandModeIdx = mpm[0] > mpm[1] ? 0 : 1;

        if ((leftIntraDir > DC_IDX) && (aboveIntraDir > DC_IDX))
        {
          mpm[0] = PLANAR_IDX;
          mpm[1] = leftIntraDir;
          mpm[2] = aboveIntraDir;
          maxCandModeIdx = mpm[1] > mpm[2] ? 1 : 2;
          int minCandModeIdx = mpm[1] > mpm[2] ? 2 : 1;
          if (mpm[maxCandModeIdx] - mpm[minCandModeIdx] == 1)
          {
            mpm[3] = ((mpm[minCandModeIdx] + offset)     % mod) + 2;
            mpm[4] = ((mpm[maxCandModeIdx] - 1)          % mod) + 2;
            mpm[5] = ((mpm[minCandModeIdx] + offset - 1) % mod) + 2;
          }
          else if (mpm[maxCandModeIdx] - mpm[minCandModeIdx] >= 62)
          {
            mpm[3] = ((mpm[minCandModeIdx] - 1)      % mod) + 2;
            mpm[4] = ((mpm[maxCandModeIdx] + offset) % mod) + 2;
            mpm[5] = ( mpm[minCandModeIdx]           % mod) + 2;
          }
          else if (mpm[maxCandModeIdx] - mpm[minCandModeIdx] == 2)
          {
            mpm[3] = ((mpm[minCandModeIdx] - 1)      % mod) + 2;
            mpm[4] = ((mpm[minCandModeIdx] + offset) % mod) + 2;
            mpm[5] = ((mpm[maxCandModeIdx] - 1)      % mod) + 2;
          }
          else
          {
            mpm[3] = ((mpm[minCandModeIdx] + offset) % mod) + 2;
            mpm[4] = ((mpm[minCandModeIdx] - 1)      % mod) + 2;
            mpm[5] = ((mpm[maxCandModeIdx] + offset) % mod) + 2;
          }
        }
        else if (leftIntraDir + aboveIntraDir >= 2)
        {
          mpm[0] = PLANAR_IDX;
          mpm[1] = (leftIntraDir < aboveIntraDir) ? aboveIntraDir : leftIntraDir;
          maxCandModeIdx = 1;
          mpm[2] = ((mpm[maxCandModeIdx] + offset)     % mod) + 2;
          mpm[3] = ((mpm[maxCandModeIdx] - 1)          % mod) + 2;
          mpm[4] = ((mpm[maxCandModeIdx] + offset - 1) % mod) + 2;
          mpm[5] = ( mpm[maxCandModeIdx]               % mod) + 2;
        }
      }
#endif

#if SECONDARY_MPM
      unsigned mpm_default[numMPMs - 1] = { DC_IDX, VER_IDX, HOR_IDX, VER_IDX - 4, VER_IDX + 4, 14, 22, 42, 58, 10, 26,
                                           38, 62, 6, 30, 34, 66, 2, 48, 52, 16 };
      for (int idx = 0; (idx < numMPMs - 1) && numValidMPM < numMPMs; idx++)
      {
        mpm[numValidMPM] = mpm_default[idx];
        if( !includedMode[mpm[numValidMPM]] )
        {
          includedMode[mpm[numValidMPM++]] = true;
        }
      }

      int numNonMPM = 0;
      for (int idx = 0; idx < NUM_LUMA_MODE; idx++)
      {
        if( !includedMode[idx] )
        {
          non_mpm[numNonMPM++] = idx;
        }
      }
#endif
    }
    for (int i = 0; i < numMPMs; i++)
    {
      CHECK(mpm[i] >= NUM_LUMA_MODE, "Invalid MPM");
    }
    CHECK(numCand == 0, "No candidates found");
    return numCand;
  }
#endif
}

#if JVET_Y0065_GPM_INTRA
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
void PU::getGeoIntraMPMs(const PredictionUnit &pu, uint8_t* mpm, uint8_t splitDir, uint8_t shape, bool doInit, bool doInitAL, bool doInitA, bool doInitL)
{
  static uint8_t  partialMPMsAll[GEO_NUM_TM_MV_CAND - 1][GEO_MAX_NUM_INTRA_CANDS]; // [0] for above-left, [1] for above [1] for left
  if (doInit)
  {
    if (doInitAL)
    {
      PU::getGeoIntraMPMs(pu, partialMPMsAll[0], GEO_NUM_PARTITION_MODE, GEO_TM_SHAPE_AL);
    }
    if (doInitA)
    {
      PU::getGeoIntraMPMs(pu, partialMPMsAll[1], GEO_NUM_PARTITION_MODE, GEO_TM_SHAPE_A);
    }
    if (doInitL)
    {
      PU::getGeoIntraMPMs(pu, partialMPMsAll[2], GEO_NUM_PARTITION_MODE, GEO_TM_SHAPE_L);
    }
  }

  const  uint8_t* partialMPMs = partialMPMsAll[shape - 1];
  uint8_t numValidMPM = 1;
  mpm[0] = g_geoAngle2IntraAng[g_geoParams[splitDir][0]];
  for (int i = 0; i < GEO_MAX_NUM_INTRA_CANDS; ++i)
  {
    if (partialMPMs[i] == NOMODE_IDX)
    {
      break;
    }

    if (partialMPMs[i] != mpm[0])
    {
      mpm[numValidMPM++] = partialMPMs[i];
      if (numValidMPM == GEO_MAX_NUM_INTRA_CANDS)
      {
        return;
      }
    }
  }

  mpm[numValidMPM] = (mpm[0] > DIA_IDX) ? (mpm[0] - 32) : (mpm[0] + 32);
  for (int i = 1; i < numValidMPM; ++i)
  {
    if (mpm[numValidMPM] == mpm[i])
    {
      --numValidMPM;
      break;
    }
  }
  ++numValidMPM;
  if (numValidMPM == GEO_MAX_NUM_INTRA_CANDS)
  {
    return;
  }

  mpm[numValidMPM] = PLANAR_IDX;
}
#endif

void PU::getGeoIntraMPMs( const PredictionUnit &pu, uint8_t* mpm, uint8_t splitDir, uint8_t shape )
{
  bool includedMode[NUM_INTRA_MODE];
  memset(includedMode, false, sizeof(includedMode));

  int numValidMPM = 0;
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  bool outputFullMPMs = splitDir < GEO_NUM_PARTITION_MODE;
  if(outputFullMPMs)
  {
#endif
  mpm[numValidMPM++] = g_geoAngle2IntraAng[g_geoParams[splitDir][0]];
  includedMode[mpm[0]] = true;
#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  }
#endif

  CodingUnit* cu = pu.cu;
#if ENABLE_DIMD
  if (cu->slice->getSPS()->getUseDimd())
  {
    if (cu->dimdMode != -1)
    {
      mpm[numValidMPM] = cu->dimdMode;
      if( !includedMode[mpm[numValidMPM]] )
      {
        includedMode[mpm[numValidMPM++]] = true;
      }
    }
  }
#endif
#if JVET_W0123_TIMD_FUSION
#if JVET_AC0094_REF_SAMPLES_OPT
  if (cu->slice->getSPS()->getUseTimd() && cu->timdMode != INVALID_TIMD_IDX)
#else
  if (cu->slice->getSPS()->getUseTimd() && cu->timdMode != -1)
#endif
  {
    mpm[numValidMPM] = MAP131TO67(cu->timdMode);
    if( !includedMode[mpm[numValidMPM]] )
    {
      includedMode[mpm[numValidMPM++]] = true;
      if (numValidMPM == GEO_MAX_NUM_INTRA_CANDS)
      {
        return;
      }
    }
  }
#endif
  const CompArea &area = pu.block(COMPONENT_Y);
  const Position posA = area.topRight().offset(0, -1);
  const Position posAR = area.topRight().offset(1, -1);
  const Position posL = area.bottomLeft().offset(-1, 0);
  const Position posBL = area.bottomLeft().offset(-1, 1);
  const Position posAL = area.topLeft().offset(-1, -1);

  if (shape == GEO_TM_SHAPE_L || shape == GEO_TM_SHAPE_AL)
  {
    const PredictionUnit *puLeft = pu.cs->getPURestricted(posL, pu, CHANNEL_TYPE_LUMA);
    if (puLeft && CU::isIntra(*puLeft->cu))
    {
#if JVET_W0123_TIMD_FUSION
      mpm[numValidMPM] = puLeft->cu->timd ? MAP131TO67(PU::getIntraDirLuma(*puLeft)) : PU::getIntraDirLuma(*puLeft);
#else
      mpm[numValidMPM] = PU::getIntraDirLuma(*puLeft);
#endif
#if JVET_AD0085_TMRL_EXTENSION
      if (puLeft->cu->tmrlFlag)
      {
        mpm[numValidMPM] = MAP131TO67(mpm[numValidMPM]);
      }
#endif
      if( !includedMode[mpm[numValidMPM]] )
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == GEO_MAX_NUM_INTRA_CANDS)
        {
          return;
        }
      }
    }
  }

  if (shape == GEO_TM_SHAPE_A || shape == GEO_TM_SHAPE_AL)
  {
    const PredictionUnit *puAbove = pu.cs->getPURestricted(posA, pu, CHANNEL_TYPE_LUMA);
    if (puAbove && CU::isIntra(*puAbove->cu))
    {
#if JVET_W0123_TIMD_FUSION
      mpm[numValidMPM] = puAbove->cu->timd ? MAP131TO67(PU::getIntraDirLuma(*puAbove)) : PU::getIntraDirLuma(*puAbove);
#else
      mpm[numValidMPM] = PU::getIntraDirLuma(*puAbove);
#endif
#if JVET_AD0085_TMRL_EXTENSION
      if (puAbove->cu->tmrlFlag)
      {
        mpm[numValidMPM] = MAP131TO67(mpm[numValidMPM]);
      }
#endif
      if( !includedMode[mpm[numValidMPM]] )
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == GEO_MAX_NUM_INTRA_CANDS)
        {
          return;
        }
      }
    }
  }

#if JVET_W0123_TIMD_FUSION
  if (shape == GEO_TM_SHAPE_L || shape == GEO_TM_SHAPE_AL)
  {
    const PredictionUnit *puLeft = pu.cs->getPURestricted(posL, pu, CHANNEL_TYPE_LUMA);
    if (puLeft && CU::isInter(*puLeft->cu))
    {
      mpm[numValidMPM] = puLeft->getIpmInfo(posL);
      if( !includedMode[mpm[numValidMPM]] )
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == GEO_MAX_NUM_INTRA_CANDS)
        {
          return;
        }
      }
    }
  }

  if (shape == GEO_TM_SHAPE_A || shape == GEO_TM_SHAPE_AL)
  {
    const PredictionUnit *puAbove = pu.cs->getPURestricted(posA, pu, CHANNEL_TYPE_LUMA);
    if (puAbove && CU::isInter(*puAbove->cu))
    {
      mpm[numValidMPM] = puAbove->getIpmInfo(posA);
      if( !includedMode[mpm[numValidMPM]] )
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == GEO_MAX_NUM_INTRA_CANDS)
        {
          return;
        }
      }
    }
  }
#endif

  if (shape == GEO_TM_SHAPE_L || shape == GEO_TM_SHAPE_AL)
  {
    const PredictionUnit *puBelowLeft = pu.cs->getPURestricted(posBL, pu, CHANNEL_TYPE_LUMA);
    if (puBelowLeft && CU::isIntra(*puBelowLeft->cu))
    {
#if JVET_W0123_TIMD_FUSION
      mpm[numValidMPM] = puBelowLeft->cu->timd ? MAP131TO67(PU::getIntraDirLuma(*puBelowLeft)) : PU::getIntraDirLuma(*puBelowLeft);
#else
      mpm[numValidMPM] = PU::getIntraDirLuma(*puBelowLeft);
#endif
#if JVET_AD0085_TMRL_EXTENSION
      if (puBelowLeft->cu->tmrlFlag)
      {
        mpm[numValidMPM] = MAP131TO67(mpm[numValidMPM]);
      }
#endif
      if( !includedMode[mpm[numValidMPM]] )
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == GEO_MAX_NUM_INTRA_CANDS)
        {
          return;
        }
      }
    }
  }

  if (shape == GEO_TM_SHAPE_A || shape == GEO_TM_SHAPE_AL)
  {
    const PredictionUnit *puAboveRight = pu.cs->getPURestricted(posAR, pu, CHANNEL_TYPE_LUMA);
    if (puAboveRight && CU::isIntra(*puAboveRight->cu))
    {
#if JVET_W0123_TIMD_FUSION
      mpm[numValidMPM] = puAboveRight->cu->timd ? MAP131TO67(PU::getIntraDirLuma(*puAboveRight)) : PU::getIntraDirLuma(*puAboveRight);
#else
      mpm[numValidMPM] = PU::getIntraDirLuma(*puAboveRight);
#endif
#if JVET_AD0085_TMRL_EXTENSION
      if (puAboveRight->cu->tmrlFlag)
      {
        mpm[numValidMPM] = MAP131TO67(mpm[numValidMPM]);
      }
#endif
      if( !includedMode[mpm[numValidMPM]] )
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == GEO_MAX_NUM_INTRA_CANDS)
        {
          return;
        }
      }
    }
  }

  {
    const PredictionUnit *puAboveLeft = pu.cs->getPURestricted(posAL, pu, CHANNEL_TYPE_LUMA);
    if (puAboveLeft && CU::isIntra(*puAboveLeft->cu))
    {
#if JVET_W0123_TIMD_FUSION
      mpm[numValidMPM] = puAboveLeft->cu->timd ? MAP131TO67(PU::getIntraDirLuma(*puAboveLeft)) : PU::getIntraDirLuma(*puAboveLeft);
#else
      mpm[numValidMPM] = PU::getIntraDirLuma(*puAboveLeft);
#endif
#if JVET_AD0085_TMRL_EXTENSION
      if (puAboveLeft->cu->tmrlFlag)
      {
        mpm[numValidMPM] = MAP131TO67(mpm[numValidMPM]);
      }
#endif
      if( !includedMode[mpm[numValidMPM]] )
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == GEO_MAX_NUM_INTRA_CANDS)
        {
          return;
        }
      }
    }
  }

#if JVET_W0123_TIMD_FUSION
  if (shape == GEO_TM_SHAPE_L || shape == GEO_TM_SHAPE_AL)
  {
    const PredictionUnit *puBelowLeft = pu.cs->getPURestricted(posBL, pu, CHANNEL_TYPE_LUMA);
    if (puBelowLeft && CU::isInter(*puBelowLeft->cu))
    {
      mpm[numValidMPM] = puBelowLeft->getIpmInfo(posBL);
      if( !includedMode[mpm[numValidMPM]] )
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == GEO_MAX_NUM_INTRA_CANDS)
        {
          return;
        }
      }
    }
  }

  if (shape == GEO_TM_SHAPE_A || shape == GEO_TM_SHAPE_AL)
  {
    const PredictionUnit *puAboveRight = pu.cs->getPURestricted(posAR, pu, CHANNEL_TYPE_LUMA);
    if (puAboveRight && CU::isInter(*puAboveRight->cu))
    {
      mpm[numValidMPM] = puAboveRight->getIpmInfo(posAR);
      if( !includedMode[mpm[numValidMPM]] )
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == GEO_MAX_NUM_INTRA_CANDS)
        {
          return;
        }
      }
    }
  }

  {
    const PredictionUnit *puAboveLeft = pu.cs->getPURestricted(posAL, pu, CHANNEL_TYPE_LUMA);
    if (puAboveLeft && CU::isInter(*puAboveLeft->cu))
    {
      mpm[numValidMPM] = puAboveLeft->getIpmInfo(posAL);
      if( !includedMode[mpm[numValidMPM]] )
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == GEO_MAX_NUM_INTRA_CANDS)
        {
          return;
        }
      }
    }
  }
#endif

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
  if (!outputFullMPMs)
  {
    for (int i = numValidMPM; i < GEO_MAX_NUM_INTRA_CANDS; ++i)
    {
      mpm[i] = NOMODE_IDX;
    }
    return;
  }
#endif

  mpm[numValidMPM] = (mpm[0] > DIA_IDX) ? (mpm[0]-32) : (mpm[0]+32);
  if( !includedMode[mpm[numValidMPM]] )
  {
    includedMode[mpm[numValidMPM++]] = true;
    if (numValidMPM == GEO_MAX_NUM_INTRA_CANDS)
    {
      return;
    }
  }
  mpm[numValidMPM] = PLANAR_IDX;
}
#endif
#if JVET_AB0155_SGPM
void PU::getSgpmIntraMPMs(const PredictionUnit &pu, uint8_t *mpm, uint8_t splitDir, uint8_t shape)
{
  bool includedMode[NUM_INTRA_MODE];
  memset(includedMode, false, sizeof(includedMode));

  int numValidMPM = 0;
  CodingUnit *cu          = pu.cu;
  bool timdDerived = !(cu->lwidth() * cu->lheight() > 1024);
  if (timdDerived)
  {
    if (includedMode[MAP131TO67(cu->timdHor)] == false && cu->timdHor > DC_IDX)
    {
      mpm[numValidMPM] = MAP131TO67(cu->timdHor);
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == SGPM_NUM_MPM)
        {
          return;
        }
      }
    }

    if (includedMode[MAP131TO67(cu->timdVer)] == false && cu->timdVer > DC_IDX)
    {
      mpm[numValidMPM] = MAP131TO67(cu->timdVer);
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == SGPM_NUM_MPM)
        {
          return;
        }
      }
    }
  }

  mpm[numValidMPM] = g_geoAngle2IntraAng[g_geoParams[splitDir][0]];
  if (!includedMode[mpm[numValidMPM]])
  {
    includedMode[mpm[numValidMPM++]] = true;
    if (numValidMPM == SGPM_NUM_MPM)
    {
      return;
    }
  }

#if ENABLE_DIMD
  if (cu->slice->getSPS()->getUseDimd())
  {
    if (cu->dimdMode != -1)
    {
      mpm[numValidMPM] = cu->dimdMode;
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == SGPM_NUM_MPM)
        {
          return;
        }
      }
    }
  }
#endif

  const CompArea &area  = pu.block(COMPONENT_Y);
  const Position  posA  = area.topRight().offset(0, -1);
  const Position  posAR = area.topRight().offset(1, -1);
  const Position  posL  = area.bottomLeft().offset(-1, 0);
  const Position  posBL = area.bottomLeft().offset(-1, 1);
  const Position  posAL = area.topLeft().offset(-1, -1);

  if (shape == GEO_TM_SHAPE_L || shape == GEO_TM_SHAPE_AL)
  {
    const PredictionUnit *puLeft = pu.cs->getPURestricted(posL, pu, CHANNEL_TYPE_LUMA);
    if (puLeft && CU::isIntra(*puLeft->cu))
    {
#if JVET_W0123_TIMD_FUSION
      mpm[numValidMPM] = puLeft->cu->timd ? MAP131TO67(PU::getIntraDirLuma(*puLeft)) : PU::getIntraDirLuma(*puLeft);
#else
      mpm[numValidMPM] = PU::getIntraDirLuma(*puLeft);
#endif
#if JVET_AD0085_TMRL_EXTENSION
      if (puLeft->cu->tmrlFlag)
      {
        mpm[numValidMPM] = MAP131TO67(mpm[numValidMPM]);
      }
#endif
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == SGPM_NUM_MPM)
        {
          return;
        }
      }
    }
  }

  if (shape == GEO_TM_SHAPE_A || shape == GEO_TM_SHAPE_AL)
  {
    const PredictionUnit *puAbove = pu.cs->getPURestricted(posA, pu, CHANNEL_TYPE_LUMA);
    if (puAbove && CU::isIntra(*puAbove->cu))
    {
#if JVET_W0123_TIMD_FUSION
      mpm[numValidMPM] = puAbove->cu->timd ? MAP131TO67(PU::getIntraDirLuma(*puAbove)) : PU::getIntraDirLuma(*puAbove);
#else
      mpm[numValidMPM] = PU::getIntraDirLuma(*puAbove);
#endif
#if JVET_AD0085_TMRL_EXTENSION
      if (puAbove->cu->tmrlFlag)
      {
        mpm[numValidMPM] = MAP131TO67(mpm[numValidMPM]);
      }
#endif
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == SGPM_NUM_MPM)
        {
          return;
        }
      }
    }
  }

#if JVET_W0123_TIMD_FUSION
  if (shape == GEO_TM_SHAPE_L || shape == GEO_TM_SHAPE_AL)
  {
    const PredictionUnit *puLeft = pu.cs->getPURestricted(posL, pu, CHANNEL_TYPE_LUMA);
#if JVET_AC0112_IBC_CIIP
    if (puLeft && (CU::isInter(*puLeft->cu) || CU::isIBC(*puLeft->cu)))
#else
    if (puLeft && CU::isInter(*puLeft->cu))
#endif
    {
      mpm[numValidMPM] = puLeft->getIpmInfo(posL);
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == SGPM_NUM_MPM)
        {
          return;
        }
      }
    }
  }

  if (shape == GEO_TM_SHAPE_A || shape == GEO_TM_SHAPE_AL)
  {
    const PredictionUnit *puAbove = pu.cs->getPURestricted(posA, pu, CHANNEL_TYPE_LUMA);
#if JVET_AC0112_IBC_CIIP
    if (puAbove && (CU::isInter(*puAbove->cu) || CU::isIBC(*puAbove->cu)))
#else
    if (puAbove && CU::isInter(*puAbove->cu))
#endif
    {
      mpm[numValidMPM] = puAbove->getIpmInfo(posA);
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == SGPM_NUM_MPM)
        {
          return;
        }
      }
    }
  }
#endif

  if (shape == GEO_TM_SHAPE_L || shape == GEO_TM_SHAPE_AL)
  {
    const PredictionUnit *puBelowLeft = pu.cs->getPURestricted(posBL, pu, CHANNEL_TYPE_LUMA);
    if (puBelowLeft && CU::isIntra(*puBelowLeft->cu))
    {
#if JVET_W0123_TIMD_FUSION
      mpm[numValidMPM] =
        puBelowLeft->cu->timd ? MAP131TO67(PU::getIntraDirLuma(*puBelowLeft)) : PU::getIntraDirLuma(*puBelowLeft);
#else
      mpm[numValidMPM] = PU::getIntraDirLuma(*puBelowLeft);
#endif
#if JVET_AD0085_TMRL_EXTENSION
      if (puBelowLeft->cu->tmrlFlag)
      {
        mpm[numValidMPM] = MAP131TO67(mpm[numValidMPM]);
      }
#endif
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == SGPM_NUM_MPM)
        {
          return;
        }
      }
    }
  }

  if (shape == GEO_TM_SHAPE_A || shape == GEO_TM_SHAPE_AL)
  {
    const PredictionUnit *puAboveRight = pu.cs->getPURestricted(posAR, pu, CHANNEL_TYPE_LUMA);
    if (puAboveRight && CU::isIntra(*puAboveRight->cu))
    {
#if JVET_W0123_TIMD_FUSION
      mpm[numValidMPM] =
        puAboveRight->cu->timd ? MAP131TO67(PU::getIntraDirLuma(*puAboveRight)) : PU::getIntraDirLuma(*puAboveRight);
#else
      mpm[numValidMPM] = PU::getIntraDirLuma(*puAboveRight);
#endif
#if JVET_AD0085_TMRL_EXTENSION
      if (puAboveRight->cu->tmrlFlag)
      {
        mpm[numValidMPM] = MAP131TO67(mpm[numValidMPM]);
      }
#endif
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == SGPM_NUM_MPM)
        {
          return;
        }
      }
    }
  }

  {
    const PredictionUnit *puAboveLeft = pu.cs->getPURestricted(posAL, pu, CHANNEL_TYPE_LUMA);
    if (puAboveLeft && CU::isIntra(*puAboveLeft->cu))
    {
#if JVET_W0123_TIMD_FUSION
      mpm[numValidMPM] =
        puAboveLeft->cu->timd ? MAP131TO67(PU::getIntraDirLuma(*puAboveLeft)) : PU::getIntraDirLuma(*puAboveLeft);
#else
      mpm[numValidMPM] = PU::getIntraDirLuma(*puAboveLeft);
#endif
#if JVET_AD0085_TMRL_EXTENSION
      if (puAboveLeft->cu->tmrlFlag)
      {
        mpm[numValidMPM] = MAP131TO67(mpm[numValidMPM]);
      }
#endif
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == SGPM_NUM_MPM)
        {
          return;
        }
      }
    }
  }

#if JVET_W0123_TIMD_FUSION
  if (shape == GEO_TM_SHAPE_L || shape == GEO_TM_SHAPE_AL)
  {
    const PredictionUnit *puBelowLeft = pu.cs->getPURestricted(posBL, pu, CHANNEL_TYPE_LUMA);
#if JVET_AC0112_IBC_CIIP
    if (puBelowLeft && (CU::isInter(*puBelowLeft->cu) || CU::isIBC(*puBelowLeft->cu)))
#else
    if (puBelowLeft && CU::isInter(*puBelowLeft->cu))
#endif
    {
      mpm[numValidMPM] = puBelowLeft->getIpmInfo(posBL);
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == SGPM_NUM_MPM)
        {
          return;
        }
      }
    }
  }

  if (shape == GEO_TM_SHAPE_A || shape == GEO_TM_SHAPE_AL)
  {
    const PredictionUnit *puAboveRight = pu.cs->getPURestricted(posAR, pu, CHANNEL_TYPE_LUMA);
#if JVET_AC0112_IBC_CIIP
    if (puAboveRight && (CU::isInter(*puAboveRight->cu) || CU::isIBC(*puAboveRight->cu)))
#else
    if (puAboveRight && CU::isInter(*puAboveRight->cu))
#endif
    {
      mpm[numValidMPM] = puAboveRight->getIpmInfo(posAR);
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == SGPM_NUM_MPM)
        {
          return;
        }
      }
    }
  }

  {
    const PredictionUnit *puAboveLeft = pu.cs->getPURestricted(posAL, pu, CHANNEL_TYPE_LUMA);
#if JVET_AC0112_IBC_CIIP
    if (puAboveLeft && (CU::isInter(*puAboveLeft->cu) || CU::isIBC(*puAboveLeft->cu)))
#else
    if (puAboveLeft && CU::isInter(*puAboveLeft->cu))
#endif
    {
      mpm[numValidMPM] = puAboveLeft->getIpmInfo(posAL);
      if (!includedMode[mpm[numValidMPM]])
      {
        includedMode[mpm[numValidMPM++]] = true;
        if (numValidMPM == SGPM_NUM_MPM)
        {
          return;
        }
      }
    }
  }
#endif

  mpm[numValidMPM] = (mpm[0] > DIA_IDX) ? (mpm[0] - 32) : (mpm[0] + 32);
  if (!includedMode[mpm[numValidMPM]])
  {
    includedMode[mpm[numValidMPM++]] = true;
    if (numValidMPM == SGPM_NUM_MPM)
    {
      return;
    }
  }
  mpm[numValidMPM] = PLANAR_IDX;
}
#endif

bool PU::isMIP(const PredictionUnit &pu, const ChannelType &chType)
{
  if (chType == CHANNEL_TYPE_LUMA)
  {
    // Default case if chType is omitted.
    return pu.cu->mipFlag;
  }
  else
  {
    return isDMChromaMIP(pu) && (pu.intraDir[CHANNEL_TYPE_CHROMA] == DM_CHROMA_IDX);
  }
}
#if JVET_V0130_INTRA_TMP
bool PU::isTmp(const PredictionUnit& pu, const ChannelType& chType)
{
	return (chType == CHANNEL_TYPE_LUMA && pu.cu->tmpFlag);
}
#endif
bool PU::isDMChromaMIP(const PredictionUnit &pu)
{
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  return !pu.cu->isSepTree() && (pu.chromaFormat == CHROMA_444) && getCoLocatedLumaPU(pu).cu->mipFlag;
#else
  return !(CS::isDualITree(*pu.cs)) && (pu.chromaFormat == CHROMA_444) && getCoLocatedLumaPU(pu).cu->mipFlag;
#endif
}

#if JVET_AA0057_CCCM || JVET_AB0092_GLM_WITH_LUMA || JVET_AC0119_LM_CHROMA_FUSION
void PU::getCccmRefLineNum(const PredictionUnit& pu, const Area area, int& th, int& tv)
{
  th = area.x < CCCM_WINDOW_SIZE ? area.x : CCCM_WINDOW_SIZE;
  tv = area.y < CCCM_WINDOW_SIZE ? area.y : CCCM_WINDOW_SIZE;

  if( CCCM_REF_LINES_ABOVE_CTU )
  {
    int ctuHeight  = pu.cs->sps->getMaxCUHeight() >> getComponentScaleY(COMPONENT_Cb, pu.chromaFormat);
    int borderDist = area.y % ctuHeight;
    int tvMax      = borderDist + CCCM_REF_LINES_ABOVE_CTU;

    tv = tv > tvMax ? tvMax : tv;
  }
}
#endif

#if JVET_AA0057_CCCM
bool PU::cccmSingleModeAvail(const PredictionUnit& pu, int intraMode)
{
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  if ( pu.cs->sps->getUseCccm() == 0 )
  {    
    return false;
  }
  else if( ( pu.cs->sps->getUseCccm() == 1 ) && ( pu.cccmNoSubFlag == 1 )  )
  {    
    return false;
  }
#endif
  const Area area = pu.blocks[COMPONENT_Cb];
  bool modeIsOk   = intraMode == LM_CHROMA_IDX;
  modeIsOk        = modeIsOk && ( area.width * area.height >= CCCM_MIN_PU_SIZE );
#if CCLM_LATENCY_RESTRICTION_RMV
  modeIsOk       &= pu.cs->sps->getUseLMChroma();
#else
  modeIsOk       &= pu.cs->sps->getUseLMChroma() && pu.cu->checkCCLMAllowed();
#endif

  return modeIsOk && (area.x > 0 || area.y > 0);
}
  
bool PU::cccmMultiModeAvail(const PredictionUnit& pu, int intraMode)
{
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  if ( pu.cs->sps->getUseCccm() == 0 )
  {    
    return false;
  }
  else if( ( pu.cs->sps->getUseCccm() == 1 ) && ( pu.cccmNoSubFlag == 1 )  )
  {    
    return false;
  }
#endif
#if MMLM
  const Area area = pu.blocks[COMPONENT_Cb];
#if JVET_AB0143_CCCM_TS
  bool modeIsOk   = (intraMode == MMLM_CHROMA_IDX) || (intraMode == MMLM_L_IDX) || (intraMode == MMLM_T_IDX);
#else
  bool modeIsOk   = intraMode == MMLM_CHROMA_IDX;
#endif
  modeIsOk        = modeIsOk && ( area.width * area.height >= CCCM_MIN_PU_SIZE );
#if CCLM_LATENCY_RESTRICTION_RMV
  modeIsOk       &= pu.cs->sps->getUseLMChroma();
#else
  modeIsOk       &= pu.cs->sps->getUseLMChroma() && pu.cu->checkCCLMAllowed();
#endif

  int th, tv;
  PU::getCccmRefLineNum(pu, area, th, tv);
#if JVET_AB0143_CCCM_TS
  int nsamples;
  bool nSampleCheck;
  if (intraMode == MMLM_CHROMA_IDX)
  {
    nsamples = ((area.width + th) * (area.height + tv) - (area.area()));
    nSampleCheck = (nsamples >= 64);
  }
  else if (intraMode == MMLM_L_IDX)
  {
    nsamples = th * area.height;
    nSampleCheck = (nsamples >= 16) && area.x && area.y;
  }
  else
  {
    nsamples = tv * area.width;
    nSampleCheck = (nsamples >= 16) && area.x && area.y;
  }
  return modeIsOk && nSampleCheck;
#else
  const int nsamples = ((area.width + th) * (area.height + tv) - (area.area()));
  return modeIsOk && nsamples >= 128;
#endif
#else
  return false;
#endif
}

#if JVET_AB0143_CCCM_TS
bool PU::isLeftCccmMode(const PredictionUnit& pu, int intraMode)
{
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  if ( pu.cs->sps->getUseCccm() == 0 )
  {    
    return false;
  }
  else if( ( pu.cs->sps->getUseCccm() == 1 ) && ( pu.cccmNoSubFlag == 1 )  )
  {    
    return false;
  }
#endif
  const Area area = pu.blocks[COMPONENT_Cb];
  bool modeIsOk = (intraMode == MDLM_L_IDX);
  modeIsOk = modeIsOk && (area.width * area.height >= CCCM_MIN_PU_SIZE);
#if CCLM_LATENCY_RESTRICTION_RMV
  modeIsOk &= pu.cs->sps->getUseLMChroma();
#else
  modeIsOk &= pu.cs->sps->getUseLMChroma() && pu.cu->checkCCLMAllowed();
#endif

  int th, tv;
  PU::getCccmRefLineNum(pu, area, th, tv);
  const int nsamples = th * area.height;
  modeIsOk = modeIsOk && (nsamples >= 16);

  return modeIsOk && area.x && area.y;
}

bool PU::isTopCccmMode(const PredictionUnit& pu, int intraMode)
{
#if JVET_AC0147_CCCM_NO_SUBSAMPLING
  if ( pu.cs->sps->getUseCccm() == 0 )
  {    
    return false;
  }
  else if( ( pu.cs->sps->getUseCccm() == 1 ) && ( pu.cccmNoSubFlag == 1 )  )
  {    
    return false;
  }
#endif
  const Area area = pu.blocks[COMPONENT_Cb];
  bool modeIsOk = (intraMode == MDLM_T_IDX);
  modeIsOk = modeIsOk && (area.width * area.height >= CCCM_MIN_PU_SIZE);
#if CCLM_LATENCY_RESTRICTION_RMV
  modeIsOk &= pu.cs->sps->getUseLMChroma();
#else
  modeIsOk &= pu.cs->sps->getUseLMChroma() && pu.cu->checkCCLMAllowed();
#endif

  int th, tv;
  PU::getCccmRefLineNum(pu, area, th, tv);
  const int nsamples = tv * area.width;
  modeIsOk = modeIsOk && (nsamples >= 16);

  return modeIsOk && area.x && area.y;
}

#if JVET_AD0202_CCCM_MDF
bool PU::isMultiCccmWithMdf(const PredictionUnit& pu, int intraMode)
{
  const Area area = pu.blocks[COMPONENT_Cb];
  int th, tv;
  PU::getCccmRefLineNum(pu, area, th, tv);
  int nsamples;
  bool nSampleCheck;
  if (intraMode == MMLM_CHROMA_IDX)
  {
    nsamples = ((area.width + th) * (area.height + tv) - (area.area()));
    nSampleCheck = (nsamples >= 96);
  }
  else if (intraMode == MMLM_L_IDX)
  {
    nsamples = th * area.height;
    nSampleCheck = (nsamples >= 256);
  }
  else
  {
    nsamples = tv * area.width;
    nSampleCheck = (nsamples >= 256);
  }
  return nSampleCheck;
}
#endif
#endif
#endif

#if JVET_Z0050_DIMD_CHROMA_FUSION
bool PU::hasChromaFusionFlag(const PredictionUnit &pu, int intraMode)
{
#if ENABLE_DIMD
  bool hasChromaFusionFlag = pu.cs->slice->getSliceType() == I_SLICE || (pu.cs->sps->getUseDimd() && intraMode == DIMD_CHROMA_IDX);
#else
  bool hasChromaFusionFlag = pu.cs->slice->getSliceType() == I_SLICE;
#endif
#if JVET_AC0071_DBV && JVET_AC0119_LM_CHROMA_FUSION
  hasChromaFusionFlag &= intraMode != DBV_CHROMA_IDX;
#endif
#if JVET_AC0119_LM_CHROMA_FUSION
  hasChromaFusionFlag &= PU::isLMCModeEnabled(pu, LM_CHROMA_IDX);
#endif
#if MMLM
  hasChromaFusionFlag &= PU::isLMCModeEnabled(pu, MMLM_CHROMA_IDX);
#else
  hasChromaFusionFlag &= PU::isLMCModeEnabled(pu, LM_CHROMA_IDX);
#endif
  return hasChromaFusionFlag;
}
#endif
#if JVET_AE0100_BVGCCCM
bool PU::hasBvgCccmFlag(const PredictionUnit &pu)
{
  if (CS::isDualITree(*pu.cs) && pu.cu->slice->getSPS()->getUseBvgCccm())
  {
    return bvgCccmModeAvail(pu);
  }
  return false;
}
bool PU::bvgCccmMultiModeAvail(const PredictionUnit& pu, int intraMode)
{
  if (intraMode == MMLM_CHROMA_IDX && pu.chromaSize().height * pu.chromaSize().width <= 16)
  {
    return false;
  }
  return true;
}
void PU::getBvgCccmCands(PredictionUnit &pu, bool &validBv)
{
  validBv = false;
  for (int posIdx = 0; posIdx < NUM_BVG_CCCM_CANDS; posIdx++)
  {
    Mv chromaBv = Mv(0, 0);
    int rrIbcType = 0;
    pu.bvList[posIdx] = Mv(0, 0);
    if (PU::isBvgCccmCand(pu, chromaBv, rrIbcType, posIdx))
    {
      if (chromaBv == Mv(0, 0))
      {
        continue;
      }
      bool bvExist = false;
      for (int i = 0; i < pu.numBvgCands; i++)
      {
        if (chromaBv == pu.bvList[i])
        {
          bvExist = true;
          break;
        }
      }
      if (!bvExist)
      {
        pu.bvList[pu.numBvgCands] = chromaBv;
        pu.rrIbcList[pu.numBvgCands] = rrIbcType;
        pu.numBvgCands++;
        validBv = true;
      }
    }
  }
}
bool PU::bvgCccmModeAvail(const PredictionUnit &pu)
{
  CompArea lumaArea = CompArea(COMPONENT_Y, pu.chromaFormat, pu.Cb().lumaPos(), recalcSize(pu.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, pu.Cb().size()));
  lumaArea = clipArea(lumaArea, pu.cs->picture->block(COMPONENT_Y));
  Position posList[NUM_BVG_CCCM_CANDS] = { lumaArea.center(), lumaArea.topLeft(), lumaArea.topRight(), lumaArea.bottomLeft(), lumaArea.bottomRight() };
  int checkOffset = BVG_CCCM_POS_OFFSET >> 1;
  
  for (int n = 0; n < NUM_BVG_CCCM_CANDS; n++)
  {
    const PredictionUnit *lumaPU = pu.cs->picture->cs->getPU(posList[n], CHANNEL_TYPE_LUMA);
    if (lumaPU && (CU::isIBC(*lumaPU->cu) || isTmp(*lumaPU)))
    {
      return true;
    }
    else
    {
      for (int a = 0; a < 3; a++)
      {
        int offsetX = (a == 0) ? 1 : (a >> 1);
        int offsetY = (a == 0) ? 1 : (a % 2);
        Position temp(posList[n].getX() - offsetX * checkOffset, posList[n].getY() - offsetY * checkOffset);
        const PredictionUnit* lumaPU = pu.cs->picture->cs->getPU(temp, CHANNEL_TYPE_LUMA);
        if (lumaPU && (CU::isIBC(*lumaPU->cu) || isTmp(*lumaPU)))
        {
          return true;
        }
      }
    }
  }
  return false;
}
bool PU::isBvgCccmCand(const PredictionUnit &pu, Mv &chromaBv, int& rrIbcType, int candIdx)
{
  rrIbcType = 0;
  chromaBv = Mv(0, 0);
  const int shiftHor = ::getComponentScaleX(COMPONENT_Cb, pu.chromaFormat);
  const int shiftVer = ::getComponentScaleY(COMPONENT_Cb, pu.chromaFormat);
  int checkOffset = BVG_CCCM_POS_OFFSET >> 1;
  bool isBvgFound = true;
  CompArea lumaArea = CompArea(COMPONENT_Y, pu.chromaFormat, pu.Cb().lumaPos(), recalcSize(pu.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, pu.Cb().size()));
  lumaArea = clipArea(lumaArea, pu.cs->picture->block(COMPONENT_Y));
  Position posList[NUM_BVG_CCCM_CANDS] = { lumaArea.center(), lumaArea.topLeft(), lumaArea.topRight(), lumaArea.bottomLeft(), lumaArea.bottomRight() };
  const PredictionUnit *lumaPU = pu.cs->picture->cs->getPU(posList[candIdx], CHANNEL_TYPE_LUMA);
  
  if (lumaPU && (CU::isIBC(*lumaPU->cu) || isTmp(*lumaPU)))
  {
    if (candIdx > 0)
    {
      for (int n = 0; n < candIdx; n++)
      {
        const PredictionUnit *prevLumaPU = pu.cs->picture->cs->getPU(posList[n], CHANNEL_TYPE_LUMA);
        if (prevLumaPU == lumaPU)
        {
          isBvgFound= false;
        }
      }
    }
    if (isBvgFound)
    {
      Mv lumaBv = lumaPU->bv;
#if JVET_AA0070_RRIBC
      lumaBv = adjustChromaBv(*lumaPU, lumaArea);
#endif
      const int bvShiftHor = MV_FRACTIONAL_BITS_INTERNAL + shiftHor;
      const int bvShiftVer = MV_FRACTIONAL_BITS_INTERNAL + shiftVer;
      lumaBv.hor = (lumaBv.hor >> bvShiftHor) << shiftHor;
      lumaBv.ver = (lumaBv.ver >> bvShiftVer) << shiftVer;
      chromaBv = Mv(lumaBv.hor >> shiftHor, lumaBv.ver >> shiftVer);
      int maxWidth = pu.chromaSize().width;
      int maxHeight = pu.chromaSize().height;
      if (PU::checkIsChromaBvCandidateValid(pu, chromaBv, maxWidth, maxHeight))
      {
        rrIbcType = (int)lumaPU->cu->rribcFlipType;
        return true;
      }
    }
  }
  {
    for (int a = 0; a < 3; a++)
    {
      int offsetX = (a == 0) ? 1 : (a >> 1);
      int offsetY = (a == 0) ? 1 : (a % 2);
      Position temp(posList[candIdx].getX() - offsetX * checkOffset, posList[candIdx].getY() - offsetY * checkOffset);
      const PredictionUnit* lumaPU = pu.cs->picture->cs->getPU(temp, CHANNEL_TYPE_LUMA);
      if (lumaPU && (CU::isIBC(*lumaPU->cu) || isTmp(*lumaPU)))
      {
        if (candIdx > 0)
        {
          for (int n = 0; n < candIdx; n++)
          {
            const PredictionUnit* prevLumaPU = pu.cs->picture->cs->getPU(posList[n], CHANNEL_TYPE_LUMA);
            if (prevLumaPU == lumaPU)
            {
              isBvgFound = false;
            }
          }
        }
        if (isBvgFound)
        {
          Mv lumaBv = lumaPU->bv;
#if JVET_AA0070_RRIBC
          lumaBv = adjustChromaBv(*lumaPU, lumaArea);
#endif
          const int bvShiftHor = MV_FRACTIONAL_BITS_INTERNAL + shiftHor;
          const int bvShiftVer = MV_FRACTIONAL_BITS_INTERNAL + shiftVer;
          lumaBv.hor = (lumaBv.hor >> bvShiftHor) << shiftHor;
          lumaBv.ver = (lumaBv.ver >> bvShiftVer) << shiftVer;
          chromaBv = Mv(lumaBv.hor >> shiftHor, lumaBv.ver >> shiftVer);
          int maxWidth = pu.chromaSize().width;
          int maxHeight = pu.chromaSize().height;
          if (PU::checkIsChromaBvCandidateValid(pu, chromaBv, maxWidth, maxHeight))
          {
            rrIbcType = (int)lumaPU->cu->rribcFlipType;
            return true;
          }
        }
      }
    }
  }
  return false;
}
bool PU::checkIsChromaBvCandidateValid(const PredictionUnit &pu, const Mv chromaBv, int &maxWidth, int &maxHeight, bool isRefTemplate, bool isRefAbove)
{
  const int cuPelX = pu.Cb().x;
  const int cuPelY = pu.Cb().y;
  int roiWidth = (isRefTemplate && !isRefAbove) ? DBV_TEMPLATE_SIZE : pu.Cb().width;
  int roiHeight = (isRefTemplate && isRefAbove) ? DBV_TEMPLATE_SIZE : pu.Cb().height;
  int xPred = chromaBv.getHor();
  int yPred = chromaBv.getVer();
  if ((xPred + roiWidth) > 0 && (yPred + roiHeight) > 0)
  {
    maxWidth = 0;
    maxHeight = 0;
    return false;
  }
  int refRightX = cuPelX + xPred + roiWidth - 1;
  int refLeftX = cuPelX + xPred;
  int refBottomY = cuPelY + yPred + roiHeight - 1;
  int refTopY = cuPelY + yPred;
  const Position refPosLT(refLeftX, refTopY);
  Position refPosBR(refRightX, refBottomY);
  if (!pu.cs->isDecomp(refPosLT, pu.chType))
  {
    return false;
  }
  if (!pu.cs->isDecomp(refPosBR, pu.chType))
  {
    Position tmpPosBR(refRightX, refTopY);
    int validX = refRightX;
    int validY = refBottomY;
    for (int x = tmpPosBR.x; x >= 0; x -= 1)
    {
      if (!pu.cs->isDecomp(tmpPosBR, pu.chType))
      {
        tmpPosBR.x -= 1;
      }
      else
      {
        validX = tmpPosBR.x;
        break;
      }
    }
    tmpPosBR.x = refLeftX;
    tmpPosBR.y = refBottomY;
    for (int y = tmpPosBR.y; y >= 0; y -= 1)
    {
      if (!pu.cs->isDecomp(tmpPosBR, pu.chType))
      {
        tmpPosBR.y -= 1;
      }
      else
      {
        validY = tmpPosBR.y;
        break;
      }
    }
    tmpPosBR.x = validX;
    tmpPosBR.y = validY;
    if (pu.cs->isDecomp(tmpPosBR, pu.chType))
    {
      maxHeight = maxHeight - (refBottomY - validY);
      maxWidth = maxWidth - (refRightX - validX);
      return true;
    }
    return false;
  }
  return true;
}
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
bool PU::isOppositeLIC(const PredictionUnit &pu)
{
  return pu.mergeOppositeLic
    || pu.tmMergeFlagOppositeLic
    || pu.affineOppositeLic
    ;
}

bool PU::hasOppositeLICFlag(const PredictionUnit &pu)
{
  if (pu.cu->slice->getUseLIC() && pu.cs->slice->getTLayer() < 5)
  {
    int blkArea = pu.lumaSize().area();
    if (pu.cu->affine)
    {
      return blkArea > 64 && blkArea < 8192;
    }
    else
    {
      return blkArea >= 32 && blkArea < 16384;
    }
  }
  return false;
}
#endif
#if JVET_AD0120_LBCCP
bool PU::isModetobeFiltered(int intraMode)
{
#if MMLM
  if (intraMode == MMLM_CHROMA_IDX )
  {
    return true;
  }
#endif
  return false;
}

bool PU::hasCcInsideFilterFlag(const PredictionUnit &pu, int intraMode)
{
#if JVET_AA0126_GLM
  if (pu.glmIdc.isActive())
  {
    return false;
  }
#endif
#if JVET_Z0050_CCLM_SLOPE
  if (pu.cclmOffsets.isActive())
  {
    return false;
  }
#endif
  return isModetobeFiltered(intraMode);
}
#endif

#if JVET_AG0059_CCP_MERGE_ENHANCEMENT
bool PU::hasCCPMergeFusionFlag(const PredictionUnit& pu)
{
  if (pu.cs->slice->getSliceType() != I_SLICE)
  {
    return false;
  }
#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
  if (!pu.cu->slice->getSPS()->getUseDimd())
  {
    return false;
  }
#endif
#if JVET_AA0057_CCCM
  if (pu.cu->slice->getSPS()->getUseCccm() == 0)
  {
    return false;
  }
#endif
  return true;
}
#endif

#if JVET_AC0071_DBV
bool PU::hasChromaBvFlag(const PredictionUnit &pu)
{
#if JVET_AF0066_ENABLE_DBV_4_SINGLE_TREE
  if (pu.cu->slice->getSPS()->getUseIntraDBV())
#else
  if (CS::isDualITree(*pu.cs) && pu.cu->slice->getSPS()->getUseIntraDBV())
#endif
  {
    return dbvModeAvail(pu);
  }
  return false;
}
#endif

#if JVET_AD0085_MPM_SORTING
bool PU::allowMPMSorted(const PredictionUnit& pu)
{
  if (!pu.cs->sps->getUseMpmSorting())
  {
    return false;
  }

  const Area area = pu.Y();

#if JVET_AD0085_TMRL_EXTENSION
  if ((pu.cs->slice->getSliceType() == I_SLICE && area.width * area.height > 128) || (area.width * area.height > 1024))
#else
  if ((pu.cs->slice->getSliceType() == I_SLICE && area.width * area.height > 32) || (area.width * area.height > 1024))
#endif
  {
    return false;
  }
  return true;
}
#endif

#if JVET_AG0154_DECODER_DERIVED_CCP_FUSION
bool PU::hasDecoderDerivedCCP(const PredictionUnit &pu)
{
  if (!pu.cs->sps->getUseLMChroma() || !pu.cu->slice->getSPS()->getUseDdCcpFusion())
  {
    return false;
  }

  if (pu.chromaSize().width * pu.chromaSize().height <= 16)
  {
    return false;
  }

  const Area area = pu.blocks[COMPONENT_Cb];

  return (area.x > 0 || area.y > 0);
}
#endif

#if JVET_AD0188_CCP_MERGE || JVET_AG0154_DECODER_DERIVED_CCP_FUSION
bool PU::hasNonLocalCCP(const PredictionUnit &pu)
{
  if (!pu.cs->sps->getUseLMChroma() || !pu.cu->slice->getSPS()->getUseCcpMerge())
  {
    return false;
  }
  if (pu.cu->ispMode && !CS::isDualITree(*pu.cs))
  {
    return false;
  }
  if (pu.chromaSize().width * pu.chromaSize().height <= 16
#if JVET_AE0043_CCP_MERGE_TEMPORAL
      && pu.cs->slice->getSliceType() == I_SLICE
#endif
  )
  {
    return false;
  }
  return true;
}

const PredictionUnit *PU::getPUFromPos(const PredictionUnit &pu, const ChannelType &chType, const Position &refPos)
{
  const CodingStructure &cs = *pu.cs;

  if (!cs.isDecomp(refPos, chType))
  {
    return nullptr;
  }

  return cs.getPURestricted(refPos, pu, chType);
}

#if JVET_AF0073_INTER_CCP_MERGE
int PU::getCCPModelCandidateList(const PredictionUnit &pu, CCPModelCandidate candList[], bool isInterCcp, int validNum, CCPModelCandidate interCcpMergeList[], int selIdx)
#else
int PU::getCCPModelCandidateList(const PredictionUnit &pu, CCPModelCandidate candList[], int selIdx)
#endif
{
  int maxCandIdx = 0;
  bool    found1stCCLM = false;
  int64_t scaleCclm[2] = { 0 };
  int     shiftCclm[2] = { 3 };

#if JVET_AF0073_INTER_CCP_MERGE
  maxCandIdx = isInterCcp ? validNum : 0;
  if (validNum)
  {
    for (int i = 0; i < validNum; i++)
    {
      candList[i] = interCcpMergeList[i];
    }

    for (int i = 0; i < validNum; i++)
    {
      if (!found1stCCLM && candList[i].type == CCP_TYPE_CCLM)
      {
        scaleCclm[0] = candList[i].params[0][0];
        shiftCclm[0] = candList[i].shift[0];
        scaleCclm[1] = candList[i].params[1][0];
        shiftCclm[1] = candList[i].shift[1];
        found1stCCLM = true;
      }
    }
  }
#endif

  int iW = pu.blocks[1].width;
  int iH = pu.blocks[1].height;

  auto tryToAddOnePU = [&](const PredictionUnit *puRef)
  {
    candList[maxCandIdx] = puRef->curCand;
    
    for (int j = 0; j < maxCandIdx; j++)
    {
      if (candList[maxCandIdx] == candList[j])
      {
        return false;
      }
    }
    return true;
  };

#if JVET_AF0073_INTER_CCP_MERGE
  auto tryToAddOneTU = [&](const TransformUnit* tuRef)
  {
    candList[maxCandIdx] = tuRef->curCand;
    
    for (int j = 0; j < maxCandIdx; j++)
    {
      if (candList[maxCandIdx] == candList[j])
      {
        return false;
      }
    }
    return true;
  };
#endif

#if JVET_AE0043_CCP_MERGE_TEMPORAL
  auto tryToAddOneModel = [&](const CCPModelCandidate& curCand)
  {
    candList[maxCandIdx] = curCand;
    
    for (int j = 0; j < maxCandIdx; j++)
    {
      if (candList[maxCandIdx] == candList[j])
      {
        return false;
      }
    }
    return true;
  };
#endif

  const Position posCand[5] =
  {
    pu.chromaPos().offset(-1, iH - 1),
    pu.chromaPos().offset(iW - 1, -1),
    pu.chromaPos().offset(-1, iH),
    pu.chromaPos().offset(iW, -1),
    pu.chromaPos().offset(-1, -1)
  };

  for (const Position &posLT : posCand)
  {
    const PredictionUnit* puRef = getPUFromPos(pu, CHANNEL_TYPE_CHROMA, posLT);
#if JVET_AF0073_INTER_CCP_MERGE
    if (puRef != nullptr && CU::isIntra(*puRef->cu) && puRef->curCand.type > 0)
#else
    if (puRef != nullptr && puRef->curCand.type > 0)
#endif
    {
      if (!tryToAddOnePU(puRef))
      {
        continue;
      }
      if (!found1stCCLM && candList[maxCandIdx].type == CCP_TYPE_CCLM)
      {
        scaleCclm[0] = candList[maxCandIdx].params[0][0];
        shiftCclm[0] = candList[maxCandIdx].shift[0];
        scaleCclm[1] = candList[maxCandIdx].params[1][0];
        shiftCclm[1] = candList[maxCandIdx].shift[1];
        found1stCCLM = true;
      }
      maxCandIdx++;
      if (maxCandIdx == MAX_CCP_CAND_LIST_SIZE)
      {
        return maxCandIdx;
      }
    }
#if JVET_AF0073_INTER_CCP_MERGE
    else if (puRef != nullptr && !CU::isIntra(*puRef->cu))
    {
      for (auto &tuRef : CU::traverseTUs(*puRef->cu))
      {
        if (tuRef.curCand.type > 0)
        {
          if (!tryToAddOneTU(&tuRef))
          {
            continue;
          }
          if (!found1stCCLM && candList[maxCandIdx].type == CCP_TYPE_CCLM)
          {
            scaleCclm[0] = candList[maxCandIdx].params[0][0];
            shiftCclm[0] = candList[maxCandIdx].shift[0];
            scaleCclm[1] = candList[maxCandIdx].params[1][0];
            shiftCclm[1] = candList[maxCandIdx].shift[1];
            found1stCCLM = true;
          }
          maxCandIdx++;
          if (maxCandIdx == MAX_CCP_CAND_LIST_SIZE)
          {
            return maxCandIdx;
          }
        }
      }
    }
#endif
  }

#if JVET_AE0043_CCP_MERGE_TEMPORAL
  // Temporal candidates
  const CodingStructure& cs = *pu.cs;
  const Slice &slice = *pu.cs->slice;

#if JVET_AF0073_INTER_CCP_MERGE
  // Second type of temporal motion vector shifted candidates
  if (CU::isInter(*pu.cu))
  {
    const Picture* const pColPic = slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0), slice.getColRefIdx());

    if (pColPic && !pColPic->isRefScaled( slice.getPPS()))
    {
      const PreCalcValues& pcv = *cs.pcv;

      bool c0Avail;
      bool c1Avail;
      bool boundaryCond;
      const SubPic& curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
      const int lumaScaleX = getChannelTypeScaleX(CHANNEL_TYPE_CHROMA, pu.chromaFormat);
      const int lumaScaleY = getChannelTypeScaleY(CHANNEL_TYPE_CHROMA, pu.chromaFormat);
      const Position posRB = pu.Cb().bottomRight().offset(-1, -1);
      const Position posCenter = pu.Cb().center();

      Position posC0;
      Position posC1;

      int offsetX0 = 0, offsetX1 = 0, offsetX2 = 0, offsetX3 = pu.Cb().width >> 1;
      int offsetY0 = 0, offsetY1 = 0, offsetY2 = 0, offsetY3 = pu.Cb().height >> 1;

      const int numNACandidate[5] = { 2, 2, 2, 2, 2 };
      const int idxMap[5][2] = { { 0, 1 },{ 0, 2 },{ 0, 2 },{ 0, 2 },{ 0, 2 } };

      MotionInfo miCurr;
      bool foundCurrMV = false;
      bool useL0;
      const int colPOC = pColPic->getPOC();

      bool isAvailableCurr = CU::isInter(*pu.cu)
#if JVET_Y0065_GPM_INTRA
                             && pu.getMotionInfo().isInter
#endif
        ;

      if (isAvailableCurr)
      {
        miCurr = pu.getMotionInfo();
        for (int i = 0; i < 2 && foundCurrMV == false; i++)
        {
          int refIdx = miCurr.refIdx[i];
          if (refIdx != -1)
          {
            const int currRefPOC = slice.getRefPic(RefPicList(i), refIdx)->getPOC();
            if (currRefPOC == colPOC)
            {
              foundCurrMV = true;
              useL0 = i == 0 ? 1 : 0;
            }
          }
        }
      }

      Mv   shiftChromaMv;
      if (foundCurrMV)
      {
        shiftChromaMv = useL0 ? miCurr.mv[0] : miCurr.mv[1];
        shiftChromaMv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
        shiftChromaMv.hor = shiftChromaMv.hor >> lumaScaleX;
        shiftChromaMv.ver = shiftChromaMv.ver >> lumaScaleY;
      }
      else
      {
        shiftChromaMv.set(0, 0);
      }

      for (int iDistanceIndex = 0; iDistanceIndex < 5 && maxCandIdx < MAX_CCP_CAND_LIST_SIZE; iDistanceIndex++)
      {
        const int iNADistanceHor = pu.Cb().width * iDistanceIndex;
        const int iNADistanceVer = pu.Cb().height * iDistanceIndex;

        for (int naspIdx = 0; naspIdx < numNACandidate[iDistanceIndex] && maxCandIdx < MAX_CCP_CAND_LIST_SIZE; naspIdx++)
        {
          switch (idxMap[iDistanceIndex][naspIdx])
          {
          case 0: offsetX0 = offsetX2 = 2 + iNADistanceHor; offsetY0 = offsetY2 = 2 + iNADistanceVer; offsetX1 = iNADistanceHor; offsetY1 = iNADistanceVer; break;
          case 1: offsetX0 = 2; offsetY0 = 0; offsetX1 = 0; offsetY1 = 2; break;
          case 2: offsetX0 = offsetX2; offsetY0 = 2 - offsetY3; offsetX1 = 2 - offsetX3; offsetY1 = offsetY2; break;
          default: printf("error!"); exit(0); break;
          }

          c0Avail = false;
          if (curSubPic.getTreatedAsPicFlag())
          {
            boundaryCond = ((posRB.x + shiftChromaMv.hor + offsetX0) <= (curSubPic.getSubPicRight() >> lumaScaleX)) && ((posRB.y + shiftChromaMv.ver + offsetY0) <= (curSubPic.getSubPicBottom() >> lumaScaleY));
          }
          else
          {
            boundaryCond = ((posRB.x + shiftChromaMv.hor + offsetX0) < (pcv.lumaWidth >> lumaScaleX)) && ((posRB.y + shiftChromaMv.ver + offsetY0) < (pcv.lumaHeight >> lumaScaleY));
          }
          if (boundaryCond)
          {
            posC0 = posRB.offset(shiftChromaMv.hor + offsetX0, shiftChromaMv.ver + offsetY0);
            c0Avail = true;
          }

          if (idxMap[iDistanceIndex][naspIdx] == 0)
          {
            c1Avail = false;
            if (curSubPic.getTreatedAsPicFlag())
            {
              boundaryCond = ((posCenter.x + shiftChromaMv.hor + offsetX1) <= (curSubPic.getSubPicRight() >> lumaScaleX) && (posCenter.y + shiftChromaMv.ver + offsetY1) <= (curSubPic.getSubPicBottom() >> lumaScaleY));
            }
            else
            {
              boundaryCond = ((posCenter.x + shiftChromaMv.hor + offsetX1) < (pcv.lumaWidth >> lumaScaleX)) && ((posCenter.y + shiftChromaMv.ver + offsetY1) < (pcv.lumaHeight >> lumaScaleY));
            }
            if (boundaryCond)
            {
              posC1 = posCenter.offset(shiftChromaMv.hor + offsetX1, shiftChromaMv.ver + offsetY1);
              c1Avail = true;
            }
          }
          else
          {
            c1Avail = false;
            if (curSubPic.getTreatedAsPicFlag())
            {
              boundaryCond = ((posRB.x + shiftChromaMv.hor + offsetX1) <= (curSubPic.getSubPicRight() >> lumaScaleX) && (posRB.y + shiftChromaMv.ver + offsetY1) <= (curSubPic.getSubPicBottom() >> lumaScaleY));
            }
            else
            {
              boundaryCond = ((posRB.x + shiftChromaMv.hor + offsetX1) < (pcv.lumaWidth >> lumaScaleX)) && ((posRB.y + shiftChromaMv.ver + offsetY1) < (pcv.lumaHeight >> lumaScaleY));
            }
            if (boundaryCond)
            {
              posC1 = posRB.offset(shiftChromaMv.hor + offsetX1, shiftChromaMv.ver + offsetY1);
              c1Avail = true;
            }
          }
          if (c0Avail || c1Avail)
          {
            int modelIdx = c0Avail ? pColPic->cs->getCcpmIdxInfo(posC0)
              : pColPic->cs->getCcpmIdxInfo(posC1);
            if (modelIdx > 0)
            {
              const CCPModelCandidate currCCPModel = pColPic->cs->m_ccpModelLUT[modelIdx - 1];
              CHECK(currCCPModel.type <= 0, "Invalid type");
              if (!tryToAddOneModel(currCCPModel))
              {
                continue;
              }
              if (!found1stCCLM && candList[maxCandIdx].type == CCP_TYPE_CCLM)
              {
                scaleCclm[0] = candList[maxCandIdx].params[0][0];
                shiftCclm[0] = candList[maxCandIdx].shift[0];
                scaleCclm[1] = candList[maxCandIdx].params[1][0];
                shiftCclm[1] = candList[maxCandIdx].shift[1];
                found1stCCLM = true;
              }
              maxCandIdx++;
              if (maxCandIdx == MAX_CCP_CAND_LIST_SIZE)
              {
                return maxCandIdx;
              }
            }
          }
        }
      }    
    }
  }
#endif

  if (!slice.isIntra())
  {
    const Picture* const pColPic = slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0), slice.getColRefIdx()); 

    if( pColPic && !pColPic->isRefScaled( slice.getPPS() ) )
    {
      const PreCalcValues& pcv = *cs.pcv;
      bool c0Avail;
      bool c1Avail;
      bool boundaryCond;
      const SubPic& curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
      int lumaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, pu.chromaFormat );
      int lumaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, pu.chromaFormat );
      Position posRB = pu.Cb().bottomRight().offset(-1, -1);
      Position posCenter = pu.Cb().center();
      Position posC0;
      Position posC1;

      int offsetX0 = 0, offsetX1 = 0, offsetX2 = 0, offsetX3 = pu.Cb().width >> 1;
      int offsetY0 = 0, offsetY1 = 0, offsetY2 = 0, offsetY3 = pu.Cb().height >> 1;

      const int numNACandidate[5] = { 2, 2, 2, 2, 2 };
      const int idxMap[5][2] = { { 0, 1 },{ 0, 2 },{ 0, 2 },{ 0, 2 },{ 0, 2 } };

      for (int iDistanceIndex = 0; iDistanceIndex < 5 && maxCandIdx < MAX_CCP_CAND_LIST_SIZE; iDistanceIndex++)
      {
        const int iNADistanceHor = pu.Cb().width  * iDistanceIndex;
        const int iNADistanceVer = pu.Cb().height * iDistanceIndex;

        for (int naspIdx = 0; naspIdx < numNACandidate[iDistanceIndex] && maxCandIdx < MAX_CCP_CAND_LIST_SIZE; naspIdx++)
        {
          switch (idxMap[iDistanceIndex][naspIdx])
          {
          case 0: offsetX0 = offsetX2 = 2 + iNADistanceHor; offsetY0 = offsetY2 = 2 + iNADistanceVer; offsetX1 = iNADistanceHor; offsetY1 = iNADistanceVer; break;
          case 1: offsetX0 = 2; offsetY0 = 0; offsetX1 = 0; offsetY1 = 2; break;
          case 2: offsetX0 = offsetX2; offsetY0 = 2 - offsetY3; offsetX1 = 2 - offsetX3; offsetY1 = offsetY2; break;
          default: printf("error!"); exit(0); break;
          }
        
          c0Avail = false;
          if (curSubPic.getTreatedAsPicFlag())
          {
            boundaryCond = ((posRB.x + offsetX0) <= (curSubPic.getSubPicRight() >> lumaScaleX)) && ((posRB.y + offsetY0) <= (curSubPic.getSubPicBottom() >> lumaScaleY));
          }
          else
          {
            boundaryCond = ((posRB.x + offsetX0) < (pcv.lumaWidth >> lumaScaleX)) && ((posRB.y + offsetY0) < (pcv.lumaHeight >> lumaScaleY));
          }
          if (boundaryCond)
          {
            posC0 = posRB.offset(offsetX0, offsetY0);
            c0Avail = true;
          }

          if (idxMap[iDistanceIndex][naspIdx] == 0)
          {
            c1Avail = false;
            if (curSubPic.getTreatedAsPicFlag())
            {
              boundaryCond = ((posCenter.x + offsetX1) <= (curSubPic.getSubPicRight() >> lumaScaleX) && (posCenter.y + offsetY1) <= (curSubPic.getSubPicBottom() >> lumaScaleY));
            }
            else
            {
              boundaryCond = ((posCenter.x + offsetX1) < (pcv.lumaWidth >> lumaScaleX)) && ((posCenter.y + offsetY1) < (pcv.lumaHeight >> lumaScaleY));
            }
            if (boundaryCond)
            {
              posC1 = posCenter.offset(offsetX1, offsetY1);
              c1Avail = true;
            }
          }
          else
          {
            c1Avail = false;
            if (curSubPic.getTreatedAsPicFlag())
            {
              boundaryCond = ((posRB.x + offsetX1) <= (curSubPic.getSubPicRight() >> lumaScaleX) && (posRB.y + offsetY1) <= (curSubPic.getSubPicBottom() >> lumaScaleY));
            }
            else
            {
              boundaryCond = ((posRB.x + offsetX1) < (pcv.lumaWidth >> lumaScaleX)) && ((posRB.y + offsetY1) < (pcv.lumaHeight >> lumaScaleY));
            }
            if (boundaryCond)
            {
              posC1 = posRB.offset(offsetX1, offsetY1);
              c1Avail = true;
            }
          }

          if (c0Avail || c1Avail)
          {
            int modelIdx = c0Avail? pColPic->cs->getCcpmIdxInfo(posC0) : pColPic->cs->getCcpmIdxInfo(posC1);
            if (modelIdx > 0)
            {
              const CCPModelCandidate currCCPModel = pColPic->cs->m_ccpModelLUT[modelIdx-1];
              CHECK(currCCPModel.type <= 0, "Invalid type");
              if (!tryToAddOneModel(currCCPModel))
              {
                continue;
              }
              if (!found1stCCLM && candList[maxCandIdx].type == CCP_TYPE_CCLM)
              {
                scaleCclm[0] = candList[maxCandIdx].params[0][0];
                shiftCclm[0] = candList[maxCandIdx].shift[0];
                scaleCclm[1] = candList[maxCandIdx].params[1][0];
                shiftCclm[1] = candList[maxCandIdx].shift[1];
                found1stCCLM = true;
              }
              maxCandIdx++;
              if (maxCandIdx == MAX_CCP_CAND_LIST_SIZE)
              {
                return maxCandIdx;
              }
            }
          }
        }
      }
    }
  }
#endif

  int offsetX = 0;  int offsetY = 0;
  int offsetX0 = 0; int offsetX1 = 0; int offsetX2 = pu.chType == CHANNEL_TYPE_LUMA ? pu.Y().width >> 1 : pu.Cb().width >> 1;
  int offsetY0 = 0; int offsetY1 = 0; int offsetY2 = pu.chType == CHANNEL_TYPE_LUMA ? pu.Y().height >> 1 : pu.Cb().height >> 1;

  const int horNAInterval = std::max((int)(pu.chType == CHANNEL_TYPE_LUMA ? pu.Y().width * 2 : pu.Cb().width * 2) >> 1, 4);
  const int verNAInterval = std::max((int)(pu.chType == CHANNEL_TYPE_LUMA ? pu.Y().height * 2 : pu.Cb().height * 2) >> 1, 4);
  const int numNACandidate[7] = { 5, 9, 9, 9, 9, 9, 9 };
  const int idxMap[7][9] =
  {
    { 0, 1, 2, 3, 4 },
    { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
    { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
    { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
    { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
    { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
    { 0, 1, 2, 3, 4, 5, 6, 7, 8 }
  };

  for (int iDistanceIndex = 0; iDistanceIndex < 7 && maxCandIdx < MAX_CCP_CAND_LIST_SIZE; iDistanceIndex++)
  {
    const int iNADistanceHor = horNAInterval * (iDistanceIndex + 1);
    const int iNADistanceVer = verNAInterval * (iDistanceIndex + 1);

    for (int naspIdx = 0; naspIdx < numNACandidate[iDistanceIndex] && maxCandIdx < MAX_CCP_CAND_LIST_SIZE; naspIdx++)
    {
      switch (idxMap[iDistanceIndex][naspIdx])
      {
      case 0: offsetX = offsetX0 = -iNADistanceHor - 1;                  offsetY = offsetY0 = verNAInterval + iNADistanceVer - 1;  break;
      case 1: offsetX = offsetX1 = horNAInterval + iNADistanceHor - 1;  offsetY = offsetY1 = -iNADistanceVer - 1;                  break;
      case 2: offsetX = offsetX2;       offsetY = offsetY1;       break;
      case 3: offsetX = offsetX0;       offsetY = offsetY2;       break;
      case 4: offsetX = offsetX0;       offsetY = offsetY1;       break;
      case 5: offsetX = -1;             offsetY = offsetY0;       break;
      case 6: offsetX = offsetX1;       offsetY = -1;             break;
      case 7: offsetX = offsetX0 >> 1;  offsetY = offsetY0;       break;
      case 8: offsetX = offsetX1;       offsetY = offsetY1 >> 1;  break;
      default: printf("error!"); exit(0); break;
      }

      Position posLT(pu.chromaPos().x + offsetX, pu.chromaPos().y + offsetY);

      const PredictionUnit *puRef = getPUFromPos(pu, CHANNEL_TYPE_CHROMA, posLT);

#if JVET_AF0073_INTER_CCP_MERGE
      if (puRef != nullptr && CU::isIntra(*puRef->cu) && puRef->curCand.type > 0)
#else
      if (puRef != nullptr && puRef->curCand.type > 0)
#endif
      {
        if (!tryToAddOnePU(puRef))
        {
          continue;
        }
        if (!found1stCCLM && candList[maxCandIdx].type == CCP_TYPE_CCLM)
        {
          scaleCclm[0] = candList[maxCandIdx].params[0][0];
          shiftCclm[0] = candList[maxCandIdx].shift[0];
          scaleCclm[1] = candList[maxCandIdx].params[1][0];
          shiftCclm[1] = candList[maxCandIdx].shift[1];
          found1stCCLM = true;
        }
        maxCandIdx++;
        if (maxCandIdx == MAX_CCP_CAND_LIST_SIZE)
        {
          return maxCandIdx;
        }
      }
#if JVET_AF0073_INTER_CCP_MERGE
      else if (puRef != nullptr && !CU::isIntra(*puRef->cu))
      {
        for (auto& tuRef : CU::traverseTUs(*puRef->cu))
        {
          if (tuRef.curCand.type > 0)
          {
            if (!tryToAddOneTU(&tuRef))
            {
              continue;
            }
            if (!found1stCCLM && candList[maxCandIdx].type == CCP_TYPE_CCLM)
            {
              scaleCclm[0] = candList[maxCandIdx].params[0][0];
              shiftCclm[0] = candList[maxCandIdx].shift[0];
              scaleCclm[1] = candList[maxCandIdx].params[1][0];
              shiftCclm[1] = candList[maxCandIdx].shift[1];
              found1stCCLM = true;
            }
            maxCandIdx++;
            if (maxCandIdx == MAX_CCP_CAND_LIST_SIZE)
            {
              return maxCandIdx;
            }
          }
        }
      }
#endif
    }
  }

  // Non-adjacent candidates round 2
  const int numNACandidate2[7] = { 4, 4, 4, 4, 4, 4, 4 };
  const int idxMap2[7][5]        = { { 0, 1, 2, 3 }, { 0, 1, 2, 3 }, { 0, 1, 2, 3 }, { 0, 1, 2, 3 },
                                     { 0, 1, 2, 3 }, { 0, 1, 2, 3 }, { 0, 1, 2, 3 } };

  for (int iDistanceIndex = 0; iDistanceIndex < 7 && maxCandIdx < MAX_CCP_CAND_LIST_SIZE; iDistanceIndex++)
  {
    const int horNADistance = horNAInterval * (iDistanceIndex + 1);
    const int verNADistance = verNAInterval * (iDistanceIndex + 1);

    for (int naspIdx = 0; naspIdx < numNACandidate2[iDistanceIndex] && maxCandIdx < MAX_CCP_CAND_LIST_SIZE; naspIdx++)
    {
      switch (idxMap2[iDistanceIndex][naspIdx])
      {
      case 0: offsetX = offsetX0 = -horNADistance - 1;          offsetY = offsetY2 + ((verNAInterval + verNADistance - 1 - offsetY2) >> 1); break;
      case 1: offsetX = offsetX2 + ((horNAInterval + horNADistance - 1 - offsetX2) >> 1); offsetY = offsetY0 = -verNADistance - 1; break;
      case 2: offsetX = offsetX0;                                offsetY = offsetY0 + ((offsetY2 - offsetY0) >> 1); break;
      case 3: offsetX = offsetX0 + ((offsetX2 - offsetX0) >> 1); offsetY = offsetY0; break;
      default: printf("error!"); exit(0); break;
      }

      Position posLT(pu.chromaPos().x + offsetX, pu.chromaPos().y + offsetY);

      const PredictionUnit *puRef = getPUFromPos(pu, CHANNEL_TYPE_CHROMA, posLT);

#if JVET_AF0073_INTER_CCP_MERGE
      if (puRef != nullptr && CU::isIntra(*puRef->cu) && puRef->curCand.type > 0)
#else
      if (puRef != nullptr && puRef->curCand.type > 0)
#endif
      {
        if (!tryToAddOnePU(puRef))
        {
          continue;
        }
        if (!found1stCCLM && candList[maxCandIdx].type == CCP_TYPE_CCLM)
        {
          scaleCclm[0] = candList[maxCandIdx].params[0][0];
          shiftCclm[0] = candList[maxCandIdx].shift[0];
          scaleCclm[1] = candList[maxCandIdx].params[1][0];
          shiftCclm[1] = candList[maxCandIdx].shift[1];
          found1stCCLM = true;
        }
        maxCandIdx++;
        if (maxCandIdx == MAX_CCP_CAND_LIST_SIZE)
        {
          return maxCandIdx;
        }
      }
#if JVET_AF0073_INTER_CCP_MERGE
      else if (puRef != nullptr && !CU::isIntra(*puRef->cu))
      {
        for (auto& tuRef : CU::traverseTUs(*puRef->cu))
        {
          if (tuRef.curCand.type > 0)
          {
            if (!tryToAddOneTU(&tuRef))
            {
              continue;
            }
            if (!found1stCCLM && candList[maxCandIdx].type == CCP_TYPE_CCLM)
            {
              scaleCclm[0] = candList[maxCandIdx].params[0][0];
              shiftCclm[0] = candList[maxCandIdx].shift[0];
              scaleCclm[1] = candList[maxCandIdx].params[1][0];
              shiftCclm[1] = candList[maxCandIdx].shift[1];
              found1stCCLM = true;
            }
            maxCandIdx++;
            if (maxCandIdx == MAX_CCP_CAND_LIST_SIZE)
            {
              return maxCandIdx;
            }
          }
        }
      }
#endif
    }
  }

#if JVET_Z0118_GDR  
  auto tryHistCCP = [&](const static_vector<CCPModelCandidate, MAX_NUM_HCCP_CANDS>& lut)
#else
  auto tryHistCCP = [&](const LutCCP& ccpLut)
#endif
  {
#if JVET_Z0118_GDR  
    for (int idx = 0; idx < lut.size(); idx++)
    {
      CCPModelCandidate curModel;
      pu.cs->getOneModelFromCCPLut(lut, curModel, idx);
#else
    for (int idx = 0; idx < ccpLut.lutCCP.size(); idx++)
    {
      CCPModelCandidate curModel;
      pu.cs->getOneModelFromCCPLut(ccpLut.lutCCP, curModel, idx);
#endif
      candList[maxCandIdx] = curModel;
      bool duplication = false;
      for (int j = 0; j < maxCandIdx; j++)
      {
        if (candList[maxCandIdx] == candList[j])
        {
          duplication = true;
          // THROW("Should not duplicaten");
          break;
        }
      }
      if (duplication)
      {
        continue;
      }
      if (!found1stCCLM && candList[maxCandIdx].type == CCP_TYPE_CCLM)
      {
        scaleCclm[0] = candList[maxCandIdx].params[0][0];
        shiftCclm[0] = candList[maxCandIdx].shift[0];
        scaleCclm[1] = candList[maxCandIdx].params[1][0];
        shiftCclm[1] = candList[maxCandIdx].shift[1];
        found1stCCLM = true;
      }
      maxCandIdx++;
      if (maxCandIdx == MAX_CCP_CAND_LIST_SIZE)
      {
        return maxCandIdx;
      }
    }
    return -1;
  };

#if JVET_Z0118_GDR
  int ret;

  if (pu.cs->isGdrEnabled() && pu.cs->isClean(pu))
  {
    ret = tryHistCCP(pu.cs->ccpLut.lutCCP1);  
  }
  else
  {
    ret = tryHistCCP(pu.cs->ccpLut.lutCCP0);
  }
#else
  int ret = tryHistCCP(pu.cs->ccpLut);
#endif

  if (ret != -1)
  {
    return ret;
  }

#if JVET_AE0043_CCP_MERGE_TEMPORAL
  // Temporal motion vector shifted candidates
  if (!slice.isIntra())
  {
    const Picture* const pColPic = slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0), slice.getColRefIdx()); 

    if( pColPic && !pColPic->isRefScaled( slice.getPPS() ) )
    {
      const PreCalcValues& pcv = *cs.pcv;

      bool c0Avail;
      bool c1Avail;
      bool boundaryCond;
      const SubPic& curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
      int lumaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, pu.chromaFormat );
      int lumaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, pu.chromaFormat );
      Position posRB = pu.Cb().bottomRight().offset(-1, -1);
      Position posCenter = pu.Cb().center();
      
      Position posC0;
      Position posC1;

      int offsetX0 = 0, offsetX1 = 0, offsetX2 = 0, offsetX3 = pu.Cb().width >> 1;
      int offsetY0 = 0, offsetY1 = 0, offsetY2 = 0, offsetY3 = pu.Cb().height >> 1;

      const int numNACandidate[5] = { 2, 2, 2, 2, 2 };
      const int idxMap[5][2] = { { 0, 1 },{ 0, 2 },{ 0, 2 },{ 0, 2 },{ 0, 2 } };

      const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;

      MotionInfo miNeigh;
      bool foundNeighMV = false;
      bool useL0;
      const int colPOC     = pColPic->getPOC();

      for (int posIdx = 0; posIdx < 5 && foundNeighMV == false; posIdx++)
      {
        const PredictionUnit *puRef = cs.getPURestricted(posCand[posIdx], pu, pu.chType);
        bool isAvailableNeigh = puRef && 
                                isDiffMER(pu.lumaPos(), posCand[posIdx], plevel) && 
#if JVET_Y0065_GPM_INTRA
                                puRef->getMotionInfo( posCand[posIdx] ).isInter &&
#endif
                                pu.cu != puRef->cu && CU::isInter(*puRef->cu);

        if (isAvailableNeigh)
        {
          miNeigh = puRef->getMotionInfo(posCand[posIdx]);
          for (int i = 0; i < 2 && foundNeighMV == false; i++)
          {
            int refIdx = miNeigh.refIdx[i];
            if (refIdx != -1)
            {
              const int currRefPOC = slice.getRefPic(RefPicList(i), refIdx)->getPOC();
              if (currRefPOC == colPOC)
              {
                foundNeighMV = true;
                useL0 = i == 0 ? 1 : 0;
              }
            }
          }
        }
      }
      
      Mv   shiftChromaMv;
      if (foundNeighMV)
      {
        shiftChromaMv = useL0 ? miNeigh.mv[0] : miNeigh.mv[1];
        shiftChromaMv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
        shiftChromaMv.hor = shiftChromaMv.hor >> lumaScaleX;
        shiftChromaMv.ver = shiftChromaMv.ver >> lumaScaleY;
      }
      else
      {
        shiftChromaMv.set(0,0);
      }

      for (int iDistanceIndex = 0; iDistanceIndex < 5 && maxCandIdx < MAX_CCP_CAND_LIST_SIZE; iDistanceIndex++)
      {
        const int iNADistanceHor = pu.Cb().width  * iDistanceIndex;
        const int iNADistanceVer = pu.Cb().height * iDistanceIndex;

        for (int naspIdx = 0; naspIdx < numNACandidate[iDistanceIndex] && maxCandIdx < MAX_CCP_CAND_LIST_SIZE; naspIdx++)
        {
          switch (idxMap[iDistanceIndex][naspIdx])
          {
          case 0: offsetX0 = offsetX2 = 2 + iNADistanceHor; offsetY0 = offsetY2 = 2 + iNADistanceVer; offsetX1 = iNADistanceHor; offsetY1 = iNADistanceVer; break;
          case 1: offsetX0 = 2; offsetY0 = 0; offsetX1 = 0; offsetY1 = 2; break;
          case 2: offsetX0 = offsetX2; offsetY0 = 2 - offsetY3; offsetX1 = 2 - offsetX3; offsetY1 = offsetY2; break;
          default: printf("error!"); exit(0); break;
          }
        
          c0Avail = false;
          if (curSubPic.getTreatedAsPicFlag())
          {
            boundaryCond = ((posRB.x + shiftChromaMv.hor + offsetX0) <= (curSubPic.getSubPicRight() >> lumaScaleX)) && ((posRB.y + shiftChromaMv.ver + offsetY0) <= (curSubPic.getSubPicBottom() >> lumaScaleY));
          }
          else
          {
            boundaryCond = ((posRB.x + shiftChromaMv.hor + offsetX0) < (pcv.lumaWidth >> lumaScaleX)) && ((posRB.y + shiftChromaMv.ver + offsetY0) < (pcv.lumaHeight >> lumaScaleY));
          }
          if (boundaryCond)
          {
            posC0 = posRB.offset(shiftChromaMv.hor + offsetX0, shiftChromaMv.ver + offsetY0);
            c0Avail = true;
          }

          if (idxMap[iDistanceIndex][naspIdx] == 0)
          {
            c1Avail = false;
            if (curSubPic.getTreatedAsPicFlag())
            {
              boundaryCond = ((posCenter.x + shiftChromaMv.hor + offsetX1) <= (curSubPic.getSubPicRight() >> lumaScaleX) && (posCenter.y + shiftChromaMv.ver + offsetY1) <= (curSubPic.getSubPicBottom() >> lumaScaleY));
            }
            else
            {
              boundaryCond = ((posCenter.x + shiftChromaMv.hor + offsetX1) < (pcv.lumaWidth >> lumaScaleX)) && ((posCenter.y + shiftChromaMv.ver + offsetY1) < (pcv.lumaHeight >> lumaScaleY));
            }
            if (boundaryCond)
            {
              posC1 = posCenter.offset(shiftChromaMv.hor + offsetX1, shiftChromaMv.ver + offsetY1);
              c1Avail = true;
            }
          }
          else
          {
            c1Avail = false;
            if (curSubPic.getTreatedAsPicFlag())
            {
              boundaryCond = ((posRB.x + shiftChromaMv.hor + offsetX1) <= (curSubPic.getSubPicRight() >> lumaScaleX) && (posRB.y + shiftChromaMv.ver + offsetY1) <= (curSubPic.getSubPicBottom() >> lumaScaleY));
            }
            else
            {
              boundaryCond = ((posRB.x + shiftChromaMv.hor + offsetX1) < (pcv.lumaWidth >> lumaScaleX)) && ((posRB.y + shiftChromaMv.ver + offsetY1) < (pcv.lumaHeight >> lumaScaleY));
            }
            if (boundaryCond)
            {
              posC1 = posRB.offset(shiftChromaMv.hor + offsetX1, shiftChromaMv.ver + offsetY1);          
              c1Avail = true;
            }
          }
          if (c0Avail || c1Avail)
          {
            int modelIdx = c0Avail? pColPic->cs->getCcpmIdxInfo(posC0) 
                            : pColPic->cs->getCcpmIdxInfo(posC1);
            if (modelIdx > 0)
            {
              const CCPModelCandidate currCCPModel = pColPic->cs->m_ccpModelLUT[modelIdx-1];
              CHECK(currCCPModel.type <= 0, "Invalid type");
              if (!tryToAddOneModel(currCCPModel))
              {
                continue;
              }
              if (!found1stCCLM && candList[maxCandIdx].type == CCP_TYPE_CCLM)
              {
                scaleCclm[0] = candList[maxCandIdx].params[0][0];
                shiftCclm[0] = candList[maxCandIdx].shift[0];
                scaleCclm[1] = candList[maxCandIdx].params[1][0];
                shiftCclm[1] = candList[maxCandIdx].shift[1];
                found1stCCLM = true;
              }
              maxCandIdx++;
              if (maxCandIdx == MAX_CCP_CAND_LIST_SIZE)
              {
                return maxCandIdx;
              }
            }
          }
        }
      }
    }
  }
#endif

  if (maxCandIdx < MAX_CCP_CAND_LIST_SIZE)
  {
    unsigned uiInternalBitDepth = pu.cs->sps->getBitDepth(CHANNEL_TYPE_CHROMA);
    const int defaultA[MAX_CCP_CAND_LIST_SIZE] = { 0, 1, -1, 2, -2, 3, -3, 4, -4, 5, -5, 6 };
    const int defaultB = 1 << (uiInternalBitDepth - 1);
    const int defaultShift = 3;

    for (int posIdx = 0; posIdx < MAX_CCP_CAND_LIST_SIZE; posIdx++)
    {
      CCPModelCandidate curModel;
      curModel.type         = CCP_TYPE_CCLM;
      curModel.params[0][0] = scaleCclm[0];
      curModel.params[0][1] = defaultB;
      curModel.shift[0]     = shiftCclm[0];
      curModel.params[1][0] = scaleCclm[1];
      curModel.params[1][1] = defaultB;
      curModel.shift[1]     = shiftCclm[1];

      if (found1stCCLM && defaultA[posIdx])
      {
        int dCb = curModel.params[0][0] > 0 ? -defaultA[posIdx] : defaultA[posIdx];
        if (curModel.shift[0] < defaultShift)
        {
          curModel.params[0][0] <<= (defaultShift - curModel.shift[0]);
          curModel.shift[0] = defaultShift;
        }
        else if (curModel.shift[0] > defaultShift)
        {
          dCb <<= (curModel.shift[0] - defaultShift);
        }
        curModel.params[0][0] += dCb;

        int dCr = curModel.params[1][0] > 0 ? -defaultA[posIdx] : defaultA[posIdx];
        if (curModel.shift[1] < defaultShift)
        {
          curModel.params[1][0] <<= (defaultShift - curModel.shift[1]);
          curModel.shift[1] = defaultShift;
        }
        else if (curModel.shift[1] > defaultShift)
        {
          dCr <<= (curModel.shift[1] - defaultShift);
        }
        curModel.params[1][0] += dCr;
      }
      else
      {
        curModel.params[0][0] = curModel.params[1][0] = defaultA[posIdx];
        curModel.shift[0] = curModel.shift[1] = defaultShift;
      }

      bool duplication = false;
      for (int j = 0; j < maxCandIdx; j++)
      {
        if (candList[j] == curModel)
        {
          duplication = true;
          // THROW("Should not duplicaten");
          break;
        }
      }
      if (duplication)
      {
        continue;
      }
      candList[maxCandIdx] = curModel;
      if (selIdx == maxCandIdx)
      {
        return maxCandIdx + 1;
      }
      maxCandIdx++;
      if (maxCandIdx == MAX_CCP_CAND_LIST_SIZE)
      {
        return maxCandIdx;
      }
    }
  }

  CHECK(maxCandIdx > MAX_CCP_CAND_LIST_SIZE, "Invlid number of non-adj CCCM candidates");
  return maxCandIdx;
}

void CU::saveModelsInHCCP(const CodingUnit &cu)
{
  bool lumaUsesISP = !CS::isDualITree(*cu.cs) && cu.ispMode;
#if JVET_AF0073_INTER_CCP_MERGE
  if (cu.chromaFormat == CHROMA_400 || (CS::isDualITree(*cu.cs) && cu.chType == CHANNEL_TYPE_LUMA))
#else
  if (cu.chromaFormat == CHROMA_400 || (CS::isDualITree(*cu.cs) && cu.chType == CHANNEL_TYPE_LUMA) || !CU::isIntra(cu))
#endif
  {
    return;
  }
  if (lumaUsesISP)
  {
    return;
  }
  const PredictionUnit &pu = *cu.firstPU;
  CodingStructure      &cs = *cu.cs;

#if JVET_AF0073_INTER_CCP_MERGE
  if (CU::isIntra(cu))
  {
#endif
  if (PU::isLMCMode(pu.intraDir[1]) && pu.curCand.type != CCP_TYPE_NONE)
  {
#if JVET_Z0118_GDR   
    if (pu.cs->isGdrEnabled() && pu.cs->isClean(pu))
    {      
      cs.addCCPToLut(cs.ccpLut.lutCCP1, pu.curCand, -1);     
    }

    cs.addCCPToLut(cs.ccpLut.lutCCP0, pu.curCand, -1);
#else
    cs.addCCPToLut(cs.ccpLut.lutCCP, pu.curCand, -1);
#endif
  }
#if JVET_AF0073_INTER_CCP_MERGE
  }
  else
  {
    for (auto& tu : CU::traverseTUs(cu))
    {
      if (tu.curCand.type != CCP_TYPE_NONE)
      {
#if JVET_Z0118_GDR   
        if (tu.cs->isGdrEnabled() && tu.cs->isClean(tu))
        {
          cs.addCCPToLut(cs.ccpLut.lutCCP1, tu.curCand, -1);
        }

        cs.addCCPToLut(cs.ccpLut.lutCCP0, tu.curCand, -1);
#else
        cs.addCCPToLut(cs.ccpLut.lutCCP, tu.curCand, -1);
#endif
      }
    }
  }
#endif
}

#if JVET_AG0059_CCP_MERGE_ENHANCEMENT
void CU::saveCcInsideFilterFlagInCCP(CodingUnit& cu)
{
  bool lumaUsesISP = !CS::isDualITree(*cu.cs) && cu.ispMode;
  if (cu.chromaFormat == CHROMA_400 || (CS::isDualITree(*cu.cs) && cu.chType == CHANNEL_TYPE_LUMA) || !CU::isIntra(cu))
  {
    return;
  }
  if (lumaUsesISP)
  {
    return;
  }
  PredictionUnit& pu = *cu.firstPU;
  if (PU::isLMCMode(pu.intraDir[1]) && pu.curCand.type != CCP_TYPE_NONE && pu.ccInsideFilter)
  {
    pu.curCand.ccInsideFilter = pu.ccInsideFilter;
  }
}
#endif

void PU::ccpParamsToCclmModel(const ComponentID compId, const CCPModelCandidate& params, CclmModel& cclmModel)
{
  cclmModel.a = int(params.params[compId - 1][0]);
  cclmModel.b = int(params.params[compId - 1][1]);
  cclmModel.shift = params.shift[compId - 1];
#if MMLM
  if (params.type & CCP_TYPE_MMLM)
  {
    cclmModel.a2 = int(params.params2[compId - 1][0]);
    cclmModel.b2 = int(params.params2[compId - 1][1]);
    cclmModel.shift2 = params.shift2[compId - 1];
    cclmModel.yThres = params.yThres;
  }
#endif
}

void PU::cclmModelToCcpParams(const ComponentID compId, CCPModelCandidate& params, const CclmModel& cclmModel)
{
  params.params[compId - 1][0] = cclmModel.a;
  params.params[compId - 1][1] = cclmModel.b;
  params.shift[compId - 1] = cclmModel.shift;
#if MMLM
  params.params2[compId - 1][0] = cclmModel.a2;
  params.params2[compId - 1][1] = cclmModel.b2;
  params.shift2[compId - 1] = cclmModel.shift2;
  params.yThres = cclmModel.yThres;
#endif
}

#if JVET_AB0174_CCCM_DIV_FREE
void PU::cccmModelToCcpParams(CCPModelCandidate& params, const CccmModel cccmModelCb[2], const CccmModel cccmModelCr[2], const int yThres, const int cccmLumaOffset)
#else
void PU::cccmModelToCcpParams(CCPModelCandidate& params, const CccmModel cccmModelCb[2], const CccmModel cccmModelCr[2], const int yThres)
#endif
{
  std::memcpy(params.params[0], cccmModelCb[0].params.data(), sizeof(TCccmCoeff) * cccmModelCb[0].getNumParams() );
  std::memcpy(params.params[1], cccmModelCr[0].params.data(), sizeof(TCccmCoeff) * cccmModelCr[0].getNumParams() );
  params.midVal = cccmModelCb[0].midVal;
  params.bd = cccmModelCb[0].bd;
#if JVET_AB0174_CCCM_DIV_FREE
  params.lumaOffset = cccmLumaOffset;
#endif
#if MMLM
  std::memcpy(params.params2[0], cccmModelCb[1].params.data(), sizeof(TCccmCoeff) * cccmModelCb[1].getNumParams() );
  std::memcpy(params.params2[1], cccmModelCr[1].params.data(), sizeof(TCccmCoeff) * cccmModelCr[1].getNumParams() );
  params.yThres = yThres;
#endif
}

void PU::ccpParamsToCccmModel(const CCPModelCandidate& params, CccmModel cccmModelCb[2], CccmModel cccmModelCr[2])
{
  std::memcpy(&cccmModelCb[0].params[0], params.params[0], sizeof(TCccmCoeff) * cccmModelCb[0].getNumParams() );
  std::memcpy(&cccmModelCr[0].params[0], params.params[1], sizeof(TCccmCoeff) * cccmModelCr[0].getNumParams() );
  cccmModelCb[0].midVal = cccmModelCr[0].midVal = params.midVal;
  cccmModelCb[0].bd = cccmModelCr[0].bd = params.bd;
#if MMLM
  if (params.type & CCP_TYPE_MMLM)
  {
    std::memcpy(&cccmModelCb[1].params[0], params.params2[0], sizeof(TCccmCoeff) * cccmModelCb[1].getNumParams() );
    std::memcpy(&cccmModelCr[1].params[0], params.params2[1], sizeof(TCccmCoeff) * cccmModelCr[1].getNumParams() );
    cccmModelCb[1].midVal = cccmModelCr[1].midVal = params.midVal;
    cccmModelCb[1].bd = cccmModelCr[1].bd = params.bd;
  }
#endif
}

#if JVET_AF0073_INTER_CCP_MERGE
#if JVET_AB0174_CCCM_DIV_FREE
void PU::cccmModelToCcpParams(CCPModelCandidate& params, const CccmModel& cccmModelCb, const CccmModel& cccmModelCr, const int cccmLumaOffset)
#else
void PU::cccmModelToCcpParams(CCPModelCandidate& params, const CccmModel& cccmModelCb, const CccmModel& cccmModelCr)
#endif
{
  std::memcpy(params.params[0], cccmModelCb.params.data(), sizeof(TCccmCoeff) * cccmModelCb.getNumParams() );
  std::memcpy(params.params[1], cccmModelCr.params.data(), sizeof(TCccmCoeff) * cccmModelCr.getNumParams() );
  params.midVal = cccmModelCb.midVal;
  params.bd = cccmModelCb.bd;
#if JVET_AB0174_CCCM_DIV_FREE
  params.lumaOffset = cccmLumaOffset;
#endif
}
void PU::ccpParamsToCccmModel(const CCPModelCandidate& params, CccmModel& cccmModelCb, CccmModel& cccmModelCr)
{
  std::memcpy(&cccmModelCb.params[0], params.params[0], sizeof(TCccmCoeff) * cccmModelCb.getNumParams() );
  std::memcpy(&cccmModelCr.params[0], params.params[1], sizeof(TCccmCoeff) * cccmModelCr.getNumParams() );
  cccmModelCb.midVal = cccmModelCr.midVal = params.midVal;
  cccmModelCb.bd = cccmModelCr.bd = params.bd;
}
#endif

#if JVET_AB0092_GLM_WITH_LUMA
#if JVET_AB0174_CCCM_DIV_FREE
void PU::glmModelToCcpParams(const ComponentID compId, CCPModelCandidate& params, const CccmModel& glmModel, const int lumaOffset)
#else
void PU::glmModelToCcpParams(const ComponentID compId, CCPModelCandidate& params, const CccmModel& glmModel)
#endif
{
  std::memcpy(params.params[compId - 1], glmModel.params.data(), sizeof(TCccmCoeff) * GLM_NUM_PARAMS);
  params.midVal = glmModel.midVal;
  params.bd = glmModel.bd;
#if JVET_AB0174_CCCM_DIV_FREE
  params.lumaOffset = lumaOffset;
#endif
}

void PU::ccpParamsToGlmModel(const ComponentID compId, const CCPModelCandidate& params, CccmModel& glmModel)
{
  std::memcpy(&glmModel.params[0], params.params[compId - 1], sizeof(TCccmCoeff) * GLM_NUM_PARAMS);
  glmModel.midVal = params.midVal;
  glmModel.bd = params.bd;
}
#endif
#endif

#if JVET_AB0155_SGPM
bool PU::isSgpm(const PredictionUnit &pu, const ChannelType &chType)
{
  if (chType == CHANNEL_TYPE_LUMA)
  {
    // Default case if chType is omitted.
    return pu.cu->sgpm;
  }
  else
  {
    return isDMChromaSgpm(pu) && (pu.intraDir[CHANNEL_TYPE_CHROMA] == DM_CHROMA_IDX);
  }
}
bool PU::isDMChromaSgpm(const PredictionUnit &pu)
{
  return false;
}
#endif

#if JVET_AB0155_SGPM
uint32_t PU::getIntraDirLuma(const PredictionUnit &pu, const int partIdx)
{
#if JVET_V0130_INTRA_TMP
  if (isMIP(pu) || isTmp(pu))
#else
  if (isMIP(pu))
#endif
  {
    return PLANAR_IDX;
  }
#if JVET_AG0058_EIP
  else if (isEIP(pu))
  {
    return PLANAR_IDX;
  }
#endif
  else
  {
    if (partIdx)
    {
#if JVET_AG0152_SGPM_ITMP_IBC
      if (pu.cu->sgpm && pu.cu->sgpmMode1 >= SGPM_BV_START_IDX)
      {
        return 0;
      }
#endif
      return pu.intraDir1[CHANNEL_TYPE_LUMA];
    }
#if JVET_AG0152_SGPM_ITMP_IBC
    if (pu.cu->sgpm && pu.cu->sgpmMode0 >= SGPM_BV_START_IDX)
    {
      return 0;
    }
#endif
    return pu.intraDir[CHANNEL_TYPE_LUMA];
  }
}
#else

uint32_t PU::getIntraDirLuma( const PredictionUnit &pu )
{
#if JVET_V0130_INTRA_TMP
	if (isMIP(pu) || isTmp(pu))
#else
  if (isMIP(pu))
#endif
  {
    return PLANAR_IDX;
  }
#if JVET_AG0058_EIP
  else if (isEIP(pu))
  {
    return PLANAR_IDX;
  }
#endif
  else
  {
    return pu.intraDir[CHANNEL_TYPE_LUMA];
  }
}
#endif

void PU::getIntraChromaCandModes(const PredictionUnit &pu, unsigned modeList[NUM_CHROMA_MODE])
{
  modeList[0] = PLANAR_IDX;
  modeList[1] = VER_IDX;
  modeList[2] = HOR_IDX;
  modeList[3] = DC_IDX;
  modeList[4] = LM_CHROMA_IDX;
#if MMLM
  modeList[5] = MDLM_L_IDX;
  modeList[6] = MDLM_T_IDX;
  modeList[7] = MMLM_CHROMA_IDX;
  modeList[8] = MMLM_L_IDX;
  modeList[9] = MMLM_T_IDX;
#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
  modeList[10] = DM_CHROMA_IDX;
  modeList[11] = DIMD_CHROMA_IDX;
#if JVET_AC0071_DBV
  modeList[12] = DBV_CHROMA_IDX;
#endif
#else
  modeList[10] = DM_CHROMA_IDX;
#if JVET_AC0071_DBV
  modeList[11] = DBV_CHROMA_IDX;
#endif
#endif
#else
  modeList[5] = MDLM_L_IDX;
  modeList[6] = MDLM_T_IDX;
#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
  modeList[7] = DM_CHROMA_IDX;
  modeList[8] = DIMD_CHROMA_IDX;
#if JVET_AC0071_DBV
  modeList[9] = DBV_CHROMA_IDX;
#endif
#else
  modeList[7] = DM_CHROMA_IDX;
#if JVET_AC0071_DBV
  modeList[8] = DBV_CHROMA_IDX;
#endif
#endif
#endif

  // If Direct Mode is MIP, mode cannot be already in the list.
  if (isDMChromaMIP(pu))
  {
    return;
  }
#if JVET_AB0155_SGPM
  if (isDMChromaSgpm(pu))
  {
    return;
  }
#endif

  const uint32_t lumaMode = getCoLocatedIntraLumaMode(pu);
  for (int i = 0; i < 4; i++)
  {
    if (lumaMode == modeList[i])
    {
      modeList[i] = VDIA_IDX;
      break;
    }
  }
}
  
#if JVET_Z0050_CCLM_SLOPE
  bool PU::hasCclmDeltaFlag(const PredictionUnit &pu, const int mode)
{
  int  chrMode      = mode < 0 ? pu.intraDir[1] : mode;
#if MMLM
  bool hasDeltaFlag = chrMode == LM_CHROMA_IDX || chrMode == MMLM_CHROMA_IDX;
#else
  bool hasDeltaFlag = chrMode == LM_CHROMA_IDX;
#endif
  hasDeltaFlag     &= pu.Cb().width * pu.Cb().height >= 128;
#if JVET_AA0126_GLM
  hasDeltaFlag     &= !pu.glmIdc.isActive();
#endif

  return hasDeltaFlag;
}
#endif

#if JVET_AA0126_GLM
bool PU::hasGlmFlag(const PredictionUnit &pu, const int mode)
{
  int  chrMode      = mode < 0 ? pu.intraDir[1] : mode;
  bool hasGlmFlag   = chrMode == LM_CHROMA_IDX || chrMode == MDLM_L_IDX || chrMode == MDLM_T_IDX;
#if JVET_AA0057_CCCM
  hasGlmFlag       &= !pu.cccmFlag;
#endif
  hasGlmFlag       &= pu.cu->chromaFormat == CHROMA_420; // GLM shall be disabled for non-4:2:0 format
  
  return hasGlmFlag;
}
#endif

bool PU::isLMCMode(unsigned mode)
{
#if MMLM
  return (mode >= LM_CHROMA_IDX && mode <= MMLM_T_IDX);
#else
  return (mode >= LM_CHROMA_IDX && mode <= MDLM_T_IDX);
#endif
}
#if MMLM
bool PU::isMultiModeLM(unsigned mode)
{
  return (mode == MMLM_CHROMA_IDX || mode == MMLM_L_IDX || mode == MMLM_T_IDX);
}
#endif
bool PU::isLMCModeEnabled(const PredictionUnit &pu, unsigned mode)
{
#if CCLM_LATENCY_RESTRICTION_RMV
  if (pu.cs->sps->getUseLMChroma() )
#else
  if ( pu.cs->sps->getUseLMChroma() && pu.cu->checkCCLMAllowed() )
#endif
  {
    return true;
  }
  return false;
}

int PU::getLMSymbolList(const PredictionUnit &pu, int *modeList)
{
  int idx = 0;

  modeList[idx++] = LM_CHROMA_IDX;
#if MMLM
  modeList[idx++] = MMLM_CHROMA_IDX;
#endif
  modeList[idx++] = MDLM_L_IDX;
  modeList[idx++] = MDLM_T_IDX;
#if MMLM
  modeList[idx++] = MMLM_L_IDX;
  modeList[idx++] = MMLM_T_IDX;
#endif
  return idx;
}

bool PU::isChromaIntraModeCrossCheckMode( const PredictionUnit &pu )
{
  return !pu.cu->bdpcmModeChroma && pu.intraDir[CHANNEL_TYPE_CHROMA] == DM_CHROMA_IDX;
}

#if JVET_AB0155_SGPM
uint32_t PU::getFinalIntraMode(const PredictionUnit &pu, const ChannelType &chType, const int partIdx)
{
  if (partIdx)
  {
    uint32_t uiIntraMode = pu.intraDir[chType];
    if (isLuma(chType))
    {
      uiIntraMode = pu.intraDir1[chType];
    }

    if (uiIntraMode == DM_CHROMA_IDX && !isLuma(chType))
    {
      uiIntraMode = getCoLocatedIntraLumaMode(pu, 1);
    }
    if (pu.chromaFormat == CHROMA_422 && !isLuma(chType)
        && uiIntraMode < NUM_LUMA_MODE)   // map directional, planar and dc
    {
      uiIntraMode = g_chroma422IntraAngleMappingTable[uiIntraMode];
    }
    return uiIntraMode;
  }
#else
uint32_t PU::getFinalIntraMode( const PredictionUnit &pu, const ChannelType &chType )
{
#endif
  uint32_t uiIntraMode = pu.intraDir[chType];

  if( uiIntraMode == DM_CHROMA_IDX && !isLuma( chType ) )
  {
    uiIntraMode = getCoLocatedIntraLumaMode(pu);
  }
  if( pu.chromaFormat == CHROMA_422 && !isLuma( chType ) && uiIntraMode < NUM_LUMA_MODE ) // map directional, planar and dc
  {
    uiIntraMode = g_chroma422IntraAngleMappingTable[uiIntraMode];
  }
#if JVET_Z0050_DIMD_CHROMA_FUSION && ENABLE_DIMD
  if (uiIntraMode == DIMD_CHROMA_IDX && !isLuma(chType))
  {
    uiIntraMode = pu.cu->dimdChromaMode;
  }
#endif
#if JVET_AC0071_DBV
  if (uiIntraMode == DBV_CHROMA_IDX && !isLuma(chType) && pu.bv == Mv())
  {
    uiIntraMode = PLANAR_IDX;
  }
#endif
  return uiIntraMode;
}

const PredictionUnit &PU::getCoLocatedLumaPU(const PredictionUnit &pu)
{
  Position              topLeftPos = pu.blocks[pu.chType].lumaPos();
  Position              refPos     = topLeftPos.offset(pu.blocks[pu.chType].lumaSize().width  >> 1,
                                                       pu.blocks[pu.chType].lumaSize().height >> 1);
#if INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  const PredictionUnit &lumaPU     = CS::isDualITree(*pu.cs) ? *pu.cs->picture->cs->getPU(refPos, CHANNEL_TYPE_LUMA)
                                                        : *pu.cs->getPU(topLeftPos, CHANNEL_TYPE_LUMA);
#else
  const PredictionUnit &lumaPU     = pu.cu->isSepTree() ? *pu.cs->picture->cs->getPU(refPos, CHANNEL_TYPE_LUMA)
                                                        : *pu.cs->getPU(topLeftPos, CHANNEL_TYPE_LUMA);
#endif
  return lumaPU;
}

#if JVET_AB0155_SGPM
uint32_t PU::getCoLocatedIntraLumaMode(const PredictionUnit &pu, const int partIdx)
{
#if JVET_AG0058_EIP
  if(PU::getCoLocatedLumaPU(pu).cu->eipFlag)
  {
    return PU::getCoLocatedLumaPU(pu).cu->eipModel.eipDimdMode;
  }
#endif
#if JVET_W0123_TIMD_FUSION
  if (PU::getCoLocatedLumaPU(pu).cu->timd)
  {
    return MAP131TO67(PU::getIntraDirLuma(PU::getCoLocatedLumaPU(pu), partIdx));
  }
#endif
#if JVET_AD0085_TMRL_EXTENSION
  if (PU::getCoLocatedLumaPU(pu).cu->tmrlFlag)
  {
    return MAP131TO67(PU::getIntraDirLuma(PU::getCoLocatedLumaPU(pu), partIdx));
  }
#endif
  return PU::getIntraDirLuma(PU::getCoLocatedLumaPU(pu), partIdx);
}
#else
uint32_t PU::getCoLocatedIntraLumaMode(const PredictionUnit &pu)
{
#if JVET_AG0058_EIP
  if(PU::getCoLocatedLumaPU(pu).cu->eipFlag)
  {
    return PU::getCoLocatedLumaPU(pu).cu->eipModel.eipDimdMode;
  }
#endif
#if JVET_W0123_TIMD_FUSION
  if (PU::getCoLocatedLumaPU(pu).cu->timd)
  {
    return MAP131TO67(PU::getIntraDirLuma(PU::getCoLocatedLumaPU(pu)));
  }
#endif
  return PU::getIntraDirLuma(PU::getCoLocatedLumaPU(pu));
}
#endif

#if JVET_AC0071_DBV
bool PU::dbvModeAvail(const PredictionUnit &pu)
{
#if JVET_AF0066_ENABLE_DBV_4_SINGLE_TREE
  if (!CS::isDualITree(*pu.cs))
  {
    const PredictionUnit &lumaPU = PU::getCoLocatedLumaPU(pu);
    return lumaPU.cu->tmpFlag || CU::isIBC(*lumaPU.cu);
  }
#endif
  CompArea lumaArea = CompArea(COMPONENT_Y, pu.chromaFormat, pu.Cb().lumaPos(), recalcSize(pu.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, pu.Cb().size()));
  lumaArea = clipArea(lumaArea, pu.cs->picture->block(COMPONENT_Y));
  Position posList[5] = { lumaArea.center(), lumaArea.topLeft(), lumaArea.topRight(), lumaArea.bottomLeft(), lumaArea.bottomRight() };
  for (int n = 0; n < NUM_DBV_POSITION; n++)
  {
    const PredictionUnit *lumaPU = pu.cs->picture->cs->getPU(posList[n], CHANNEL_TYPE_LUMA);
#if JVET_AB0061_ITMP_BV_FOR_IBC
    if (CU::isIBC(*lumaPU->cu) || isTmp(*lumaPU))
#else
    if (CU::isIBC(*lumaPU->cu))
#endif
    {
      return true;
    }
  }
  return false;
}

void PU::deriveChromaBv(PredictionUnit &pu)
{
#if JVET_AF0066_ENABLE_DBV_4_SINGLE_TREE
  if (!CS::isDualITree(*pu.cs))
  {
    return;
  }
#endif
  pu.bv.set(0, 0);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  pu.mv[0].setZero();
  int filterIdx = 0;
  uint8_t bkIntraDir = pu.intraDir[1];
  pu.intraDir[1] = DBV_CHROMA_IDX;
#endif
#if JVET_AA0070_RRIBC
  pu.cu->rribcFlipType = 0;
#endif
#if !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  const int shiftSampleHor = ::getComponentScaleX(COMPONENT_Cb, pu.chromaFormat);
  const int shiftSampleVer = ::getComponentScaleY(COMPONENT_Cb, pu.chromaFormat);
#endif
  CompArea lumaArea = CompArea(COMPONENT_Y, pu.chromaFormat, pu.Cb().lumaPos(), recalcSize(pu.chromaFormat, CHANNEL_TYPE_CHROMA, CHANNEL_TYPE_LUMA, pu.Cb().size()));
  lumaArea = clipArea(lumaArea, pu.cs->picture->block(COMPONENT_Y));

  Position posList[5] = { lumaArea.center(), lumaArea.topLeft(), lumaArea.topRight(), lumaArea.bottomLeft(), lumaArea.bottomRight() };
  for (int n = 0; n < NUM_DBV_POSITION; n++)
  {
    const PredictionUnit *lumaPU = pu.cs->picture->cs->getPU(posList[n], CHANNEL_TYPE_LUMA);
#if JVET_AB0061_ITMP_BV_FOR_IBC
    if (CU::isIBC(*lumaPU->cu) || isTmp(*lumaPU))
#else
    if (CU::isIBC(*lumaPU->cu))
#endif
    {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      Mv lumaBv = lumaPU->mv[0];
#else
      Mv lumaBv = lumaPU->bv;
#endif
#if JVET_AA0070_RRIBC
      lumaBv = adjustChromaBv(*lumaPU, lumaArea);
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      if (PU::checkIsChromaBvCandidateValid(pu, lumaBv, filterIdx))
#else
      Mv chromaBv = Mv(lumaBv.hor >> shiftSampleHor, lumaBv.ver >> shiftSampleVer);
      if (PU::checkIsChromaBvCandidateValid(pu, chromaBv))
#endif
      {
        pu.bv = lumaBv;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
        pu.bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
        pu.mv[0] = lumaBv;
#endif
#if JVET_AA0070_RRIBC
        pu.cu->rribcFlipType = lumaPU->cu->rribcFlipType;
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
        pu.intraDir[1] = bkIntraDir;
#endif
        return;
      }
#if JVET_AE0169_BIPREDICTIVE_IBC
      if (CU::isIBC(*lumaPU->cu) && lumaPU->interDir == 3)
      {
        lumaBv = lumaPU->mv[REF_PIC_LIST_1];
#if !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
        lumaBv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
#endif
#if JVET_AA0070_RRIBC
        lumaBv = adjustChromaBv(*lumaPU, lumaArea, REF_PIC_LIST_1);
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
        if (PU::checkIsChromaBvCandidateValid(pu, lumaBv, filterIdx))
#else
        Mv chromaBv = Mv(lumaBv.hor >> shiftSampleHor, lumaBv.ver >> shiftSampleVer);
        if (PU::checkIsChromaBvCandidateValid(pu, chromaBv))
#endif
        {
#if JVET_AA0070_RRIBC
          pu.cu->rribcFlipType = lumaPU->cu->rribcFlipType;
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
          pu.intraDir[1] = bkIntraDir;
#endif
          return;
        }
      }
#endif
    }
  }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  pu.intraDir[1] = bkIntraDir;
#endif
}

#if JVET_AA0070_RRIBC
#if JVET_AE0169_BIPREDICTIVE_IBC
Mv PU::adjustChromaBv(const PredictionUnit &parentPU, const CompArea &lumaArea, RefPicList list)
#else
Mv PU::adjustChromaBv(const PredictionUnit &parentPU, const CompArea &lumaArea)
#endif
{
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AE0169_BIPREDICTIVE_IBC
  Mv lumaBv = parentPU.mv[list];
#else
  Mv lumaBv = parentPU.mv[0];
#endif
#else
#if JVET_AE0169_BIPREDICTIVE_IBC
  Mv lumaBv = parentPU.mv[list];
  lumaBv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
#else
  Mv lumaBv = parentPU.bv;
#endif
#endif
  int flipType = parentPU.cu->rribcFlipType;
  Position parentCPos = parentPU.Y().center();
  Position curCPos = lumaArea.center();
  if (flipType)
  {
    if (flipType == 1)
    {
      int shift = (parentCPos.x - curCPos.x) << 1;
      if (shift)
      {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
        shift <<= MV_FRACTIONAL_BITS_INTERNAL;
#endif
        lumaBv.setHor(lumaBv.hor + shift);
      }
    }
    else if (flipType == 2)
    {
      int shift = (parentCPos.y - curCPos.y) << 1;
      if (shift)
      {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
        shift <<= MV_FRACTIONAL_BITS_INTERNAL;
#endif
        lumaBv.setVer(lumaBv.ver + shift);
      }
    }
  }
  return lumaBv;
}
#endif

bool PU::xCheckSimilarChromaBv(std::vector<Mv> &chromaBvList, const Mv chromaBv)
{
  for (int i = 0; i < chromaBvList.size(); i++)
  {
    if (chromaBv == chromaBvList[i])
    {
      return true;
    }
  }
  return false;
}

bool PU::checkIsChromaBvCandidateValid(const PredictionUnit &pu
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                     , const Mv mv
                                     , int filterIdx
#else
                                     , const Mv chromaBv
#endif
                                     , bool isRefTemplate, bool isRefAbove)
{
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  int roiWidth  = (isRefTemplate && !isRefAbove) ? DBV_TEMPLATE_SIZE : pu.Cb().width;
  int roiHeight = (isRefTemplate && isRefAbove ) ? DBV_TEMPLATE_SIZE : pu.Cb().height;
  uint32_t validType = checkValidBv(pu, COMPONENT_Cb, roiWidth, roiHeight, mv, true, filterIdx);
  return validType != IBC_BV_INVALID;
#else
  const int cuPelX = pu.Cb().x;
  const int cuPelY = pu.Cb().y;
  int roiWidth = (isRefTemplate && !isRefAbove) ? DBV_TEMPLATE_SIZE : pu.Cb().width;
  int roiHeight = (isRefTemplate && isRefAbove) ? DBV_TEMPLATE_SIZE : pu.Cb().height;
  int xPred = chromaBv.getHor();
  int yPred = chromaBv.getVer();
  if ((xPred + roiWidth) > 0 && (yPred + roiHeight) > 0)
  {
    return false;
  }
  int refRightX = cuPelX + xPred + roiWidth - 1;
  int refLeftX = cuPelX + xPred;
  int refBottomY = cuPelY + yPred + roiHeight - 1;
  int refTopY = cuPelY + yPred;
  const Position refPosLT(refLeftX, refTopY);
  const Position refPosBR(refRightX, refBottomY);
  if (!pu.cs->isDecomp(refPosBR, pu.chType))
  {
    return false;
  }
  if (!pu.cs->isDecomp(refPosLT, pu.chType))
  {
    return false;
  }
  return true;
#endif
}
#endif

int PU::getWideAngle( const TransformUnit &tu, const uint32_t dirMode, const ComponentID compID )
{
  //This function returns a wide angle index taking into account that the values 0 and 1 are reserved
  //for Planar and DC respectively, as defined in the Spec. Text.
  if( dirMode < 2 )
  {
    return ( int ) dirMode;
  }

  const CompArea&  area         = tu.cu->ispMode && isLuma(compID) ? tu.cu->blocks[compID] : tu.blocks[ compID ];
  int              width        = area.width;
  int              height       = area.height;
  int              modeShift[ ] = { 0, 6, 10, 12, 14, 15 };
  int              deltaSize    = abs( floorLog2( width ) - floorLog2( height ) );
  int              predMode     = dirMode;

  if( width > height && dirMode < 2 + modeShift[ deltaSize ] )
  {
    predMode += ( VDIA_IDX - 1 );
  }
  else if( height > width && predMode > VDIA_IDX - modeShift[ deltaSize ] )
  {
    predMode -= ( VDIA_IDX + 1 );
  }

  return predMode;
}

#if JVET_W0119_LFNST_EXTENSION
int PU::getLFNSTMatrixDim( int width, int height )
{
  int dimension = ( width == 8 && height == 8 ) ? 16 : ( ( ( width >= 8 ) && ( height >= 8 ) ) ? 32 : 16 );

  if( ( width >= 16 ) && ( height >= 16 ) )
  {
    dimension = L16H;
  }

  return dimension;
}

bool PU::getUseLFNST8( int width, int height )
{
  return ( width >= 8 ) && ( height >= 8 );
}

bool PU::getUseLFNST16( int width, int height )
{
  return ( width >= 16 ) && ( height >= 16 );
}

uint8_t PU::getLFNSTIdx( int intraMode, int mtsMode )
{
  return g_lfnstLut[ intraMode ];
}
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
void PU::addMergeHMVPCandSubTMVP(const CodingStructure &cs, MergeCtx &mrgCtx, const int &mrgCandIdx,
  const uint32_t maxNumMergeCand, int &cnt, const bool isAvailableA1,
  const MotionInfo miLeft, const bool isAvailableB1, const MotionInfo miAbove,
#if !JVET_Z0075_IBC_HMVP_ENLARGE
  , const bool ibcFlag
  , const bool isGt4x4
#endif
  const PredictionUnit &pu, int col
#if TM_MRG || (JVET_Z0084_IBC_TM && !JVET_Z0075_IBC_HMVP_ENLARGE)
  , const uint32_t mvdSimilarityThresh
#endif
)
{
  const Slice& slice = *cs.slice;
  MotionInfo miNeighbor;

  const Picture *pColPic = slice.getRefPic(RefPicList(col == 0 ? 1 - slice.getColFromL0Flag() : 1 - slice.getColFromL0Flag2nd()), col == 0 ? slice.getColRefIdx() : slice.getColRefIdx2nd());
#if JVET_Z0075_IBC_HMVP_ENLARGE
#if JVET_Z0118_GDR  
  bool isClean = cs.isClean(pu.cu->Y().bottomRight(), CHANNEL_TYPE_LUMA);
  auto &lut = (isClean) ? cs.motionLut.lut1 : cs.motionLut.lut0;
#else
  auto &lut = cs.motionLut.lut;
#endif
#else

#if JVET_Z0118_GDR  
  auto &lut = ibcFlag ? (isClean ? cs.motionLut.lutIbc1 : cs.motionLut.lutIbc0) : (isClean ? cs.motionLut.lut1 : cs.motionLut.lut0);
#else
  auto &lut = ibcFlag ? cs.motionLut.lutIbc : cs.motionLut.lut;
#endif

#endif // JVET_Z0075_IBC_HMVP_ENLARGE
  int numCandInLUT = (int)lut.size();

  for (int mrgIdx = 1; mrgIdx <= numCandInLUT; mrgIdx++)
  {
    miNeighbor = lut[numCandInLUT - mrgIdx];

#if !JVET_X0083_BM_AMVP_MERGE_MODE
#if JVET_Y0128_NON_CTC
    bool isValidAmMode = checkIsValidMergeMvCand(pu, miNeighbor.refIdx);
#else
    bool isValidAmMode = checkIsValidMergeMvCand(cs, pu, curPoc, amvpPoc, miNeighbor.refIdx);
#endif
    if (isValidAmMode &&
#if JVET_Z0075_IBC_HMVP_ENLARGE
    (mrgIdx > 2
#else
      (mrgIdx > 2 || ((mrgIdx > 1 || !isGt4x4) && ibcFlag)
#endif
        || ((!isAvailableA1 || (miLeft != miNeighbor)) && (!isAvailableB1 || (miAbove != miNeighbor))))
    )
#else
#if JVET_Z0075_IBC_HMVP_ENLARGE
    if (mrgIdx > 2
#else
    if (mrgIdx > 2 || ((mrgIdx > 1 || !isGt4x4) && ibcFlag)
#endif
      || ((!isAvailableA1 || (miLeft != miNeighbor)) && (!isAvailableB1 || (miAbove != miNeighbor))))
#endif
    {
      int index = 0;
      if ((miNeighbor.interDir & (1 << REF_PIC_LIST_0)) && slice.getRefPic(REF_PIC_LIST_0, miNeighbor.refIdx[0]) == pColPic)
      {
        index = 1;
      }
      else if (slice.isInterB() && (miNeighbor.interDir & (1 << REF_PIC_LIST_1)) && slice.getRefPic(REF_PIC_LIST_1, miNeighbor.refIdx[1]) == pColPic)
      {
        index = 2;
      }
      if (index > 0)
      {
        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = index;
        miNeighbor.mv[index - 1].roundToPrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
        if (index == 1)
        {
          mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[index - 1], miNeighbor.refIdx[index - 1]);
          mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(Mv(), NOT_VALID);
        }
        else
        {
          mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(Mv(), NOT_VALID);
          mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[index - 1], miNeighbor.refIdx[index - 1]);
        }

#if NON_ADJACENT_MRG_CAND
        if (mrgCtx.xCheckSimilarMotionSubTMVP(cnt
#if TM_MRG
          , mvdSimilarityThresh
#endif
        ))
        {
          continue;
        }
#endif
        cnt++;

        if (cnt == maxNumMergeCand)
        {
          break;
        }
      }
    }
    return;
  }
}
#endif
bool PU::addMergeHMVPCand(const CodingStructure &cs, MergeCtx &mrgCtx, const int &mrgCandIdx,
                          const uint32_t maxNumMergeCandMin1, int &cnt, const bool isAvailableA1,
                          const MotionInfo miLeft, const bool isAvailableB1, const MotionInfo miAbove
#if !JVET_Z0075_IBC_HMVP_ENLARGE
                        , const bool ibcFlag, const bool isGt4x4
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE || (JVET_Y0058_IBC_LIST_MODIFY && !JVET_Z0075_IBC_HMVP_ENLARGE)
                        , const PredictionUnit &pu
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
#if !JVET_Y0128_NON_CTC
                        , const int curPoc
                        , const int amvpPoc
#endif
#endif
#if TM_MRG || (JVET_Z0084_IBC_TM && !JVET_Z0075_IBC_HMVP_ENLARGE) || JVET_AE0046_BI_GPM
                        , const uint32_t mvdSimilarityThresh
#endif
  )
{
  const Slice& slice = *cs.slice;
  MotionInfo miNeighbor;

#if JVET_Z0118_GDR
  bool isClean = cs.isClean(pu.cu->Y().bottomRight(), CHANNEL_TYPE_LUMA);
#endif
#if JVET_Z0075_IBC_HMVP_ENLARGE
#if JVET_Z0118_GDR
  auto &lut = (isClean) ? cs.motionLut.lut1 : cs.motionLut.lut0;
#else
  auto &lut = cs.motionLut.lut;
#endif
#else
#if JVET_Z0118_GDR
  auto &lut = ibcFlag ? (isClean ? cs.motionLut.lutIbc1 : cs.motionLut.lutIbc0) : (isClean ? cs.motionLut.lut1 : cs.motionLut.lut0);
#else
  auto &lut = ibcFlag ? cs.motionLut.lutIbc : cs.motionLut.lut;
#endif
#endif

  int num_avai_candInLUT = (int)lut.size();

#if JVET_AA0070_RRIBC && !JVET_Z0075_IBC_HMVP_ENLARGE
  int cPosCurX = pu.lx() + (pu.lwidth() >> 1);
  int cPosCurY = pu.ly() + (pu.lheight() >> 1);
  int thW      = (pu.lwidth() >> 1) * 3;
  int thH      = (pu.lheight() >> 1) * 3;
#endif

  for (int mrgIdx = 1; mrgIdx <= num_avai_candInLUT; mrgIdx++)
  {
    miNeighbor = lut[num_avai_candInLUT - mrgIdx];

#if JVET_AA0070_RRIBC && !JVET_Z0075_IBC_HMVP_ENLARGE
    if (ibcFlag)
    {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      miNeighbor.useAltHpelIf = pu.mergeFlag ? miNeighbor.useAltHpelIf : (pu.cu->imv == IMV_HPEL);
#endif
      Position cPos(miNeighbor.centerPos.x, miNeighbor.centerPos.y);
      if (pu.mergeFlag || ((abs(cPosCurX - (int) cPos.x) <= thW) && (abs(cPosCurY - (int) cPos.y) <= thH)))
      {
        rribcAdjustMotion(pu, &cPos, miNeighbor);
      }
    }
#endif

#if JVET_X0083_BM_AMVP_MERGE_MODE
#if JVET_Y0128_NON_CTC
    bool isValidAmMode = checkIsValidMergeMvCand(pu, miNeighbor.refIdx);
#else
    bool isValidAmMode = checkIsValidMergeMvCand(cs, pu, curPoc, amvpPoc, miNeighbor.refIdx);
#endif
    if (isValidAmMode &&
#if JVET_Z0075_IBC_HMVP_ENLARGE
        ( mrgIdx > 2
#else
        ( mrgIdx > 2 || ((mrgIdx > 1 || !isGt4x4) && ibcFlag)
#endif
          || ((!isAvailableA1 || (miLeft != miNeighbor)) && (!isAvailableB1 || (miAbove != miNeighbor))) )
          )
#else
#if JVET_Z0075_IBC_HMVP_ENLARGE
    if ( mrgIdx > 2
#else
    if ( mrgIdx > 2 || ((mrgIdx > 1 || !isGt4x4) && ibcFlag)
#endif
      || ((!isAvailableA1 || (miLeft != miNeighbor)) && (!isAvailableB1 || (miAbove != miNeighbor))) )
#endif
    {
#if JVET_Y0058_IBC_LIST_MODIFY && JVET_Z0084_IBC_TM && !JVET_Z0075_IBC_HMVP_ENLARGE
      if (!ibcFlag || checkIsIBCCandidateValid(pu, miNeighbor))
      {
#endif
      mrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
#if JVET_Z0075_IBC_HMVP_ENLARGE || JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      mrgCtx.useAltHpelIf      [cnt] = miNeighbor.useAltHpelIf;
#else
      mrgCtx.useAltHpelIf      [cnt] = !ibcFlag && miNeighbor.useAltHpelIf;
#endif
      mrgCtx.bcwIdx            [cnt] = (miNeighbor.interDir == 3) ? miNeighbor.bcwIdx : BCW_DEFAULT;
#if INTER_LIC
      mrgCtx.licFlags          [cnt] = miNeighbor.usesLIC;
#if JVET_AA0070_RRIBC
      mrgCtx.rribcFlipTypes    [cnt] = 0;
#endif
#if !JVET_Z0075_IBC_HMVP_ENLARGE
      if (ibcFlag)
      {
#if JVET_AA0070_RRIBC
        mrgCtx.rribcFlipTypes[cnt] = !pu.tmMergeFlag ? miNeighbor.rribcFlipType : 0;
#endif
        CHECK(mrgCtx.licFlags[cnt], "addMergeHMVPCand: LIC is not used with IBC mode")
      }
#endif
#endif
#if JVET_AC0112_IBC_LIC
#if JVET_AA0070_RRIBC
      mrgCtx.ibcLicFlags       [cnt] = mrgCtx.rribcFlipTypes[cnt] ? false : miNeighbor.useIbcLic;
#else
      mrgCtx.ibcLicFlags       [cnt] = miNeighbor.useIbcLic;
#endif
#endif
#if JVET_AE0159_FIBC
#if JVET_AA0070_RRIBC
      mrgCtx.ibcFilterFlags    [cnt] = mrgCtx.rribcFlipTypes[cnt] ? false : miNeighbor.useIbcFilter;
#else
      mrgCtx.ibcFilterFlags    [cnt] = miNeighbor.useIbcFilter;
#endif
#endif

      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
      if (slice.isInterB())
      {
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);
      }
#if MULTI_HYP_PRED
      mrgCtx.addHypNeighbours[cnt] = miNeighbor.addHypData;
#endif
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
      if (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])
      {
        mrgCtx.interDirNeighbours[cnt] = pu.amvpMergeModeFlag[0] ? 1 : 2;
        mrgCtx.mvFieldNeighbours[(cnt << 1) + (pu.amvpMergeModeFlag[0] ? 1 : 0)].setMvField(Mv(), -1);
      }
#endif

#if JVET_Z0084_IBC_TM && !JVET_Z0075_IBC_HMVP_ENLARGE
      if (ibcFlag)
      {
        if (mrgCtx.xCheckSimilarIBCMotion(cnt, mvdSimilarityThresh))
        {
          continue;
        }
      }
#if NON_ADJACENT_MRG_CAND
      else
#endif
#endif
#if NON_ADJACENT_MRG_CAND
      if (mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG || JVET_AE0046_BI_GPM
                            , mvdSimilarityThresh
#endif
      ))
      {
#if JVET_AD0213_LIC_IMP
        mrgCtx.initMrgCand(cnt);
#endif
        continue;
      }
#endif
#if JVET_Y0058_IBC_LIST_MODIFY && !JVET_Z0075_IBC_HMVP_ENLARGE && !JVET_Z0084_IBC_TM
      if (ibcFlag)
      {
        if (!checkIsIBCCandidateValid(pu, miNeighbor))
        {
          continue;
        }
      }
#endif
      if (mrgCandIdx == cnt)
      {
        return true;
      }
      cnt ++;

      if (cnt  == maxNumMergeCandMin1)
      {
        break;
      }
#if JVET_Y0058_IBC_LIST_MODIFY && JVET_Z0084_IBC_TM && !JVET_Z0075_IBC_HMVP_ENLARGE
      }
#endif
    }
  }

  if (cnt < maxNumMergeCandMin1)
  {
    mrgCtx.useAltHpelIf[cnt] = false;
  }
  return false;
}

#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
inline void PU::getTemplateTop(const bool availableTop, const PredictionUnit &pu, CPelBuf pRecY, PelBuf pTempDest,
                               Position tempOffset, int tempWidth, int tempHeight)
{
  if (availableTop)
  {
    const Pel *pOrg = pRecY.bufAt(pu.blocks[COMPONENT_Y].pos().offset(tempOffset));
    for (int r = 0; r < tempHeight; r++)
    {
      for (int c = 0; c < tempWidth; c++)
      {
        *pTempDest.buf++ = pOrg[r * pRecY.stride + c];
      }
    }
  }
}

inline void PU::getTemplateLeft(const bool availableLeft, const PredictionUnit &pu, CPelBuf pRecY, PelBuf pTempDest,
                                Position tempOffset, int tempWidth, int tempHeight)
{
  if (availableLeft)
  {
    const Pel *pOrg = pRecY.bufAt(pu.blocks[COMPONENT_Y].pos().offset(tempOffset));
    for (int c = 0; c < tempWidth; c++)
    {
      for (int r = 0; r < tempHeight; r++)
      {
        *pTempDest.buf++ = pOrg[r * pRecY.stride + c];
      }
    }
  }
}

inline void PU::getTemplateRefTop(const PredictionUnit &pu, CPelBuf pRecY, PelBuf pTempDest, Mv CandMv, int tempWidth, int tempHeight
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                , InterPrediction *pcInter
#endif
)
{
  MotionInfo templateMv;
  int        offset = -AML_MERGE_TEMPLATE_SIZE;
  templateMv.mv[0]  = CandMv + Mv(0, offset << (MV_PRECISION_INTERNAL - MV_PRECISION_INT));
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  int nFilterIdx = 1;
#endif
  if (!PU::checkIsIBCCandidateValid(pu, templateMv
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                  , nFilterIdx
#endif
  ))
  {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    templateMv.mv[0] = CandMv;
#else
    offset = 0;
#endif
  }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  bool isFracMv = pu.cs->sps->getIBCFracFlag() && templateMv.mv[0].isFracMv();
  if (isFracMv)
  {
    PelUnitBuf pcBuf(pu.chromaFormat, pTempDest);
    pcInter->getPredIBCBlk(pu, COMPONENT_Y, pu.cu->slice->getPic(), templateMv.mv[0], pcBuf, nFilterIdx == 1
#if JVET_AC0112_IBC_LIC
                         , true
#endif
    );
  }
  else
  {
    templateMv.mv[0] >>= MV_FRACTIONAL_BITS_INTERNAL;
    PU::getTemplateTop(true, pu, pRecY, pTempDest, Position(templateMv.mv[0].hor, templateMv.mv[0].ver), tempWidth, tempHeight);
  }
#else
  CandMv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
  PU::getTemplateTop(true, pu, pRecY, pTempDest, Position(CandMv.hor, CandMv.ver + offset), tempWidth, tempHeight);
#endif
}

inline void PU::getTemplateRefLeft(const PredictionUnit &pu, CPelBuf pRecY, PelBuf pTempDest, Mv CandMv, int tempWidth,
                            int tempHeight
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                 , InterPrediction *pcInter
#endif
)
{
  MotionInfo templateMv;
  int        offset = -AML_MERGE_TEMPLATE_SIZE;
  templateMv.mv[0]  = CandMv + Mv(offset << (MV_PRECISION_INTERNAL - MV_PRECISION_INT), 0);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  int nFilterIdx = 1;
#endif
  if (!PU::checkIsIBCCandidateValid(pu, templateMv
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                  , nFilterIdx
#endif
  ))
  {
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    templateMv.mv[0] = CandMv;
#else
    offset = 0;
#endif
  }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  bool isFracMv = pu.cs->sps->getIBCFracFlag() && templateMv.mv[0].isFracMv();
  if(isFracMv)
  {
    PelUnitBuf pcBuf(pu.chromaFormat, pTempDest);
    pcInter->getPredIBCBlk(pu, COMPONENT_Y, pu.cu->slice->getPic(), templateMv.mv[0], pcBuf, nFilterIdx == 1
#if JVET_AC0112_IBC_LIC
                         , true
#endif
    );
  }
  else
  {
    templateMv.mv[0] >>= MV_FRACTIONAL_BITS_INTERNAL;
    PU::getTemplateLeft(true, pu, pRecY, pTempDest, Position(templateMv.mv[0].hor, templateMv.mv[0].ver), tempWidth, tempHeight);
  }
#else
  CandMv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
  PU::getTemplateLeft(true, pu, pRecY, pTempDest, Position(CandMv.hor + offset, CandMv.ver), tempWidth, tempHeight);
#endif
}

inline Distortion PU::getTMCost(const PredictionUnit &pu, CPelBuf pRecY, Mv CandMv, bool availableTmTop,
                                bool availableTmLeft,
                         InterPrediction *pcInter)
{
  Distortion uiCost = 0;
  if (availableTmTop)
  {
    PU::getTemplateRefTop(pu, pRecY, pu.cs->m_pcBufPredRefTop, CandMv, pu.cs->m_pcBufPredCurTop.width,
                          AML_MERGE_TEMPLATE_SIZE
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                        , pcInter
#endif
    );
    uiCost += pcInter->getTempCost(pu, pu.cs->m_pcBufPredCurTop, pu.cs->m_pcBufPredRefTop);
  }
  if (availableTmLeft)
  {
    PU::getTemplateRefLeft(pu, pRecY, pu.cs->m_pcBufPredRefLeft, CandMv, AML_MERGE_TEMPLATE_SIZE,
                           pu.cs->m_pcBufPredCurLeft.height
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                         , pcInter
#endif
    );
    uiCost += pcInter->getTempCost(pu, pu.cs->m_pcBufPredCurLeft, pu.cs->m_pcBufPredRefLeft);
  }
  return uiCost;
}
#endif

#if JVET_Z0075_IBC_HMVP_ENLARGE
bool PU::addIBCMergeHMVPCand(const CodingStructure &cs, MergeCtx &mrgCtx, const int &mrgCandIdx,
  const uint32_t maxNumMergeCandMin1, int &cnt
#if JVET_Y0058_IBC_LIST_MODIFY
  , const PredictionUnit &pu
#endif
#if TM_MRG || JVET_Z0084_IBC_TM
  , const uint32_t mvdSimilarityThresh
#endif
)
{
#if !JVET_Z0084_IBC_TM
  const Slice& slice = *cs.slice;
#endif
  MotionInfo miNeighbor;
#if JVET_Z0118_GDR
  bool isClean = cs.isClean(pu.cu->Y().bottomRight(), CHANNEL_TYPE_LUMA);
#endif

#if JVET_Z0118_GDR
  auto &lut = (isClean) ? cs.motionLut.lutIbc1 : cs.motionLut.lutIbc0;
#else
  auto &lut = cs.motionLut.lutIbc;
#endif
  int num_avai_candInLUT = (int)lut.size();
  int compareNum = cnt;

#if JVET_AA0070_RRIBC
  int cPosCurX = pu.lx() + (pu.lwidth() >> 1);
  int cPosCurY = pu.ly() + (pu.lheight() >> 1);
  int thW      = (pu.lwidth() >> 1) * 3;
  int thH      = (pu.lheight() >> 1) * 3;
#endif
  for (int mrgIdx = 1; mrgIdx <= num_avai_candInLUT; mrgIdx++)
  {
    miNeighbor = lut[num_avai_candInLUT - mrgIdx];
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    miNeighbor.useAltHpelIf = pu.mergeFlag ? miNeighbor.useAltHpelIf : (pu.cu->imv == IMV_HPEL);
#endif
#if JVET_AA0070_RRIBC
    Position cPos(miNeighbor.centerPos.x, miNeighbor.centerPos.y);
    if (pu.mergeFlag || ((abs(cPosCurX - (int) cPos.x) <= thW) && (abs(cPosCurY - (int) cPos.y) <= thH)))
    {
      rribcAdjustMotion(pu, &cPos, miNeighbor);
    }
#endif

#if JVET_Y0058_IBC_LIST_MODIFY && JVET_Z0084_IBC_TM
    if (checkIsIBCCandidateValid(pu, miNeighbor))
    {
#endif
      mrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
#if JVET_AA0070_RRIBC
#if IBC_TM_MRG
      mrgCtx.rribcFlipTypes[cnt] = !pu.tmMergeFlag ? miNeighbor.rribcFlipType : 0;
#else
      mrgCtx.rribcFlipTypes[cnt] = miNeighbor.rribcFlipType;
#endif
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      mrgCtx.useAltHpelIf      [cnt] = miNeighbor.useAltHpelIf;
#endif
#if !JVET_Z0084_IBC_TM
#if !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      mrgCtx.useAltHpelIf      [cnt] = false;
#endif
      mrgCtx.bcwIdx            [cnt] = (miNeighbor.interDir == 3) ? miNeighbor.bcwIdx : BCW_DEFAULT;
#if INTER_LIC
      mrgCtx.licFlags          [cnt] = miNeighbor.usesLIC;
      CHECK(mrgCtx.licFlags[cnt], "addIBCMergeHMVPCand: LIC is not used with IBC mode")
#endif
#endif

      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
#if JVET_AE0084_IBC_LIC_INHERITANCE
#if JVET_AC0112_IBC_LIC
#if JVET_AA0070_RRIBC
      mrgCtx.ibcLicFlags[cnt] = mrgCtx.rribcFlipTypes[cnt] == 0 ? miNeighbor.useIbcLic : false;
#else
      mrgCtx.ibcLicFlags[cnt] = miNeighbor.useIbcLic;
#endif
#endif
#endif
#if JVET_AE0159_FIBC
#if JVET_AA0070_RRIBC
      mrgCtx.ibcFilterFlags[cnt] = mrgCtx.rribcFlipTypes[cnt] == 0 ? miNeighbor.useIbcFilter : false;
#else
      mrgCtx.ibcFilterFlags[cnt] = miNeighbor.useIbcFilter;
#endif
#endif
#if !JVET_Z0084_IBC_TM
      if (slice.isInterB())
      {
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);
      }
#if MULTI_HYP_PRED
      mrgCtx.addHypNeighbours[cnt] = miNeighbor.addHypData;
#endif
#endif

#if JVET_Z0084_IBC_TM
      if (mrgCtx.xCheckSimilarIBCMotion(cnt, mvdSimilarityThresh, compareNum))
      {
        continue;
      }
#else
      if (mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG
        , mvdSimilarityThresh
#endif
        , compareNum
      ))
      {
        continue;
      }
#endif
#if JVET_Y0058_IBC_LIST_MODIFY && !JVET_Z0084_IBC_TM
      if (!checkIsIBCCandidateValid(pu, miNeighbor))
      {
        continue;
      }
#endif
      if (mrgCandIdx == cnt)
      {
        return true;
      }
      cnt++;

      if (cnt == maxNumMergeCandMin1)
      {
        break;
      }
#if JVET_Y0058_IBC_LIST_MODIFY && JVET_Z0084_IBC_TM
    }
#endif
  }

#if !JVET_Z0084_IBC_TM
  if (cnt < maxNumMergeCandMin1)
  {
    mrgCtx.useAltHpelIf[cnt] = false;
  }
#endif
  return false;
}
#endif

#if JVET_AA0070_RRIBC
void PU::rribcAdjustMotion(const PredictionUnit &pu, const Position *cPos, MotionInfo &miNeighbor)
{
#if IBC_TM_MRG
  if (pu.tmMergeFlag)
  {
    return;
  }
#endif

#if JVET_AA0061_IBC_MBVD
  if (pu.ibcMbvdMergeFlag)
  {
    return;
  }
#endif

  int curCPosX = pu.lx() + (pu.lwidth() >> 1);
  int curCPosY = pu.ly() + (pu.lheight() >> 1);

  if (miNeighbor.isIBCmot && miNeighbor.rribcFlipType && (pu.mergeFlag || (!pu.mergeFlag && (pu.cu->rribcFlipType == miNeighbor.rribcFlipType))))
  {
    if (miNeighbor.rribcFlipType == 1)
    {
      int shift = (cPos->x - curCPosX) << 1;
      if (shift)
      {
        int storeHor = miNeighbor.mv[0].getHor();
        miNeighbor.mv[0].setHor(storeHor + (shift << 4));
        if (!checkIsIBCCandidateValid(pu, miNeighbor))
        {
          miNeighbor.mv[0].setHor(storeHor);
        }
      }
    }
    else if (miNeighbor.rribcFlipType == 2)
    {
      int shift = (cPos->y - curCPosY) << 1;
      if (shift)
      {
        int storeVer = miNeighbor.mv[0].getVer();
        miNeighbor.mv[0].setVer(storeVer + (shift << 4));
        if (!checkIsIBCCandidateValid(pu, miNeighbor))
        {
          miNeighbor.mv[0].setVer(storeVer);
        }
      }
    }
  }
}
#endif

void PU::getIBCMergeCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx, const int& mrgCandIdx)
{
  const CodingStructure &cs = *pu.cs;
#if JVET_Z0075_IBC_HMVP_ENLARGE
  const uint32_t maxNumMergeCand = IBC_MRG_MAX_NUM_CANDS_MEM;
#else
  const uint32_t maxNumMergeCand = pu.cs->sps->getMaxNumIBCMergeCand();
#endif
#if JVET_Z0084_IBC_TM || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
#if IBC_TM_MRG
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  uint32_t       mvdSimilarityThresh = 1;
  if (pu.isBvpClusterApplicable())
  {
    mvdSimilarityThresh = (pu.tmMergeFlag && pu.mergeFlag) ? PU::getTMMvdThreshold(pu) : 1;
  }
  else
  {
#if JVET_AA0070_RRIBC
    mvdSimilarityThresh =
      ((pu.tmMergeFlag && pu.mergeFlag) || (!pu.mergeFlag && !pu.cu->rribcFlipType)) ? PU::getTMMvdThreshold(pu) : 1;
#else
    const uint32_t mvdSimilarityThresh = pu.tmMergeFlag ? PU::getTMMvdThreshold(pu) : 1;
#endif
  }
#else
#if JVET_AA0070_RRIBC
  const uint32_t mvdSimilarityThresh =
    ((pu.tmMergeFlag && pu.mergeFlag) || (!pu.mergeFlag && !pu.cu->rribcFlipType)) ? PU::getTMMvdThreshold(pu) : 1;
#else
  const uint32_t mvdSimilarityThresh = pu.tmMergeFlag ? PU::getTMMvdThreshold(pu) : 1;
#endif
#endif
#else
  const uint32_t mvdSimilarityThresh = 1;
#endif
#endif

  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
    mrgCtx.bcwIdx[ui] = BCW_DEFAULT;
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mvFieldNeighbours[ui * 2].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[ui * 2 + 1].refIdx = NOT_VALID;
    mrgCtx.useAltHpelIf[ui] = false;
#if JVET_AG0276_NLIC
    mrgCtx.altLMFlag[ui] = false;
    mrgCtx.altLMParaNeighbours[ui].resetAltLinearModel();
#endif
#if INTER_LIC
    mrgCtx.licFlags[ui] = false;
#endif
#if JVET_AC0112_IBC_LIC
    mrgCtx.ibcLicFlags[ui] =false;
#endif
#if JVET_AE0159_FIBC
    mrgCtx.ibcFilterFlags[ui] = false;
#endif
#if JVET_AA0070_RRIBC
    mrgCtx.rribcFlipTypes[ui] = 0;
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    mrgCtx.mvFieldNeighbours[(ui << 1)    ].setMvField(Mv(), -1);
    mrgCtx.mvFieldNeighbours[(ui << 1) + 1].setMvField(Mv(), -1);
#endif
  }

#if JVET_Z0075_IBC_HMVP_ENLARGE
  mrgCtx.numValidMergeCand = pu.cs->sps->getMaxNumIBCMergeCand();
#else 
  mrgCtx.numValidMergeCand = maxNumMergeCand;
#endif
  // compute the location of the current PU

  int cnt = 0;

#if JVET_Y0058_IBC_LIST_MODIFY
  const Position posLT = pu.Y().topLeft();
#endif
  const Position posRT = pu.Y().topRight();
  const Position posLB = pu.Y().bottomLeft();

  MotionInfo miAbove, miLeft, miAboveLeft, miAboveRight, miBelowLeft;

  //left
  const PredictionUnit* puLeft = cs.getPURestricted(posLB.offset(-1, 0), pu, pu.chType);
  bool isGt4x4 = pu.lwidth() * pu.lheight() > 16;
#if JVET_AE0094_IBC_NONADJACENT_SPATIAL_CANDIDATES
  if(pu.cs->sps->getUseIbcNonAdjCand())
  {
    isGt4x4 = true;
  }
#endif
#if JVET_AB0061_ITMP_BV_FOR_IBC
  const bool isAvailableA1 = puLeft && pu.cu != puLeft->cu && (CU::isIBC(*puLeft->cu) || puLeft->cu->tmpFlag);
#else
  const bool isAvailableA1 = puLeft && pu.cu != puLeft->cu && CU::isIBC(*puLeft->cu);
#endif
  if (isGt4x4 && isAvailableA1)
  {
    miLeft = puLeft->getMotionInfo(posLB.offset(-1, 0));
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    miLeft.useAltHpelIf = pu.mergeFlag ? miLeft.useAltHpelIf : (pu.cu->imv == IMV_HPEL);
#endif
#if JVET_AA0070_RRIBC
    Position cPos(puLeft->lx() + (puLeft->lwidth() >> 1), puLeft->ly() + (puLeft->lheight() >> 1));
    rribcAdjustMotion(pu, &cPos, miLeft);
#endif

#if JVET_Y0058_IBC_LIST_MODIFY
    if (checkIsIBCCandidateValid(pu, miLeft))
    {
#endif
    // get Inter Dir
#if JVET_AE0169_BIPREDICTIVE_IBC
    mrgCtx.interDirNeighbours[cnt] = 1;
#else
    mrgCtx.interDirNeighbours[cnt] = miLeft.interDir;
#endif
    // get Mv from Left
    mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miLeft.mv[0], miLeft.refIdx[0]);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    mrgCtx.useAltHpelIf[cnt] = miLeft.useAltHpelIf;
#endif
#if JVET_AA0070_RRIBC
#if IBC_TM_MRG
#if JVET_AA0061_IBC_MBVD
    mrgCtx.rribcFlipTypes[cnt] = (miLeft.isIBCmot && !pu.tmMergeFlag && !pu.ibcMbvdMergeFlag) ? miLeft.rribcFlipType : 0;
#else
    mrgCtx.rribcFlipTypes[cnt] = (miLeft.isIBCmot && !pu.tmMergeFlag) ? miLeft.rribcFlipType : 0;
#endif
#else
#if JVET_AA0061_IBC_MBVD
    mrgCtx.rribcFlipTypes[cnt] = (miLeft.isIBCmot && !pu.ibcMbvdMergeFlag) ? miLeft.rribcFlipType : 0;
#else
    mrgCtx.rribcFlipTypes[cnt] = miLeft.isIBCmot ? miLeft.rribcFlipType : 0;
#endif
#endif
#endif
#if JVET_AC0112_IBC_LIC
#if JVET_AA0070_RRIBC
    mrgCtx.ibcLicFlags[cnt] = miLeft.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miLeft.useIbcLic : false;
#else
    mrgCtx.ibcLicFlags[cnt] = miLeft.isIBCmot? miLeft.useIbcLic : false;
#endif
#endif
#if JVET_AE0159_FIBC
#if JVET_AA0070_RRIBC
    mrgCtx.ibcFilterFlags[cnt] = miLeft.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miLeft.useIbcFilter : false;
#else
    mrgCtx.ibcFilterFlags[cnt] = miLeft.isIBCmot? miLeft.useIbcFilter : false;
#endif
#endif
    if (mrgCandIdx == cnt)
    {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
      mrgCtx.numAMVPMergeCand = cnt;
#endif
      return;
    }
    cnt++;

    // early termination
    if (cnt == maxNumMergeCand)
    {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
      mrgCtx.numAMVPMergeCand = cnt;
#endif
      return;
    }
#if JVET_Y0058_IBC_LIST_MODIFY
    }
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (miLeft.isIBCmot && miLeft.interDir == 3)
    {
      MotionInfo miLeftL1 = miLeft;
      miLeftL1.mv[0] = miLeft.mv[1];
      miLeftL1.refIdx[1] = -1;
#if JVET_AA0070_RRIBC
      rribcAdjustMotion(pu, &cPos, miLeftL1);
#endif

#if JVET_Y0058_IBC_LIST_MODIFY
      if (checkIsIBCCandidateValid(pu, miLeftL1))
      {
#endif
      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = 1;
      // get Mv from Left
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miLeftL1.mv[0], miLeftL1.refIdx[0]);
#if JVET_AA0070_RRIBC
#if IBC_TM_MRG
#if JVET_AA0061_IBC_MBVD
      mrgCtx.rribcFlipTypes[cnt] = (miLeftL1.isIBCmot && !pu.tmMergeFlag && !pu.ibcMbvdMergeFlag) ? miLeftL1.rribcFlipType : 0;
#else
      mrgCtx.rribcFlipTypes[cnt] = (miLeftL1.isIBCmot && !pu.tmMergeFlag) ? miLeftL1.rribcFlipType : 0;
#endif
#else
#if JVET_AA0061_IBC_MBVD
      mrgCtx.rribcFlipTypes[cnt] = (miLeftL1.isIBCmot && !pu.ibcMbvdMergeFlag) ? miLeftL1.rribcFlipType : 0;
#else
      mrgCtx.rribcFlipTypes[cnt] = miLeftL1.isIBCmot ? miLeftL1.rribcFlipType : 0;
#endif
#endif
#endif
#if JVET_AC0112_IBC_LIC
#if JVET_AA0070_RRIBC
      mrgCtx.ibcLicFlags[cnt] = miLeftL1.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miLeftL1.useIbcLic : false;
#else
      mrgCtx.ibcLicFlags[cnt] = miLeftL1.isIBCmot? miLeftL1.useIbcLic : false;
#endif
#endif
#if JVET_AE0159_FIBC
#if JVET_AA0070_RRIBC
      mrgCtx.ibcFilterFlags[cnt] = miLeftL1.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miLeftL1.useIbcFilter : false;
#else
      mrgCtx.ibcFilterFlags[cnt] = miLeftL1.isIBCmot? miLeftL1.useIbcFilter : false;
#endif
#endif
      if (mrgCandIdx == cnt)
      {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
        mrgCtx.numAMVPMergeCand = cnt;
#endif
        return;
      }
      cnt++;

      // early termination
      if (cnt == maxNumMergeCand)
      {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
        mrgCtx.numAMVPMergeCand = cnt;
#endif
        return;
      }
#if JVET_Y0058_IBC_LIST_MODIFY
      }
#endif
    }
#endif
  }

  // above
  const PredictionUnit *puAbove = cs.getPURestricted(posRT.offset(0, -1), pu, pu.chType);
#if JVET_AB0061_ITMP_BV_FOR_IBC
  bool isAvailableB1 = puAbove && pu.cu != puAbove->cu && (CU::isIBC(*puAbove->cu) || puAbove->cu->tmpFlag);
#else
  bool isAvailableB1 = puAbove && pu.cu != puAbove->cu && CU::isIBC(*puAbove->cu);
#endif
  if (isGt4x4 && isAvailableB1)
  {
    miAbove = puAbove->getMotionInfo(posRT.offset(0, -1));
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    miAbove.useAltHpelIf = pu.mergeFlag ? miAbove.useAltHpelIf : (pu.cu->imv == IMV_HPEL);
#endif
#if JVET_AA0070_RRIBC
    Position cPos(puAbove->lx() + (puAbove->lwidth() >> 1), puAbove->ly() + (puAbove->lheight() >> 1));
    rribcAdjustMotion(pu, &cPos, miAbove);
#endif

    if (!isAvailableA1 || (miAbove != miLeft))
    {
#if JVET_Y0058_IBC_LIST_MODIFY
      if (checkIsIBCCandidateValid(pu, miAbove))
      {
#endif
      // get Inter Dir
#if JVET_AE0169_BIPREDICTIVE_IBC
      mrgCtx.interDirNeighbours[cnt] = 1;
#else
      mrgCtx.interDirNeighbours[cnt] = miAbove.interDir;
#endif
      // get Mv from Above
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAbove.mv[0], miAbove.refIdx[0]);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      mrgCtx.useAltHpelIf[cnt] = miAbove.useAltHpelIf;
#endif
#if JVET_AA0070_RRIBC
#if IBC_TM_MRG
#if JVET_AA0061_IBC_MBVD
      mrgCtx.rribcFlipTypes[cnt] = (miAbove.isIBCmot && !pu.tmMergeFlag && !pu.ibcMbvdMergeFlag) ? miAbove.rribcFlipType : 0;
#else
      mrgCtx.rribcFlipTypes[cnt] = (miAbove.isIBCmot && !pu.tmMergeFlag) ? miAbove.rribcFlipType : 0;
#endif
#else
#if JVET_AA0061_IBC_MBVD
      mrgCtx.rribcFlipTypes[cnt] = (miAbove.isIBCmot && !pu.ibcMbvdMergeFlag) ? miAbove.rribcFlipType : 0;
#else
      mrgCtx.rribcFlipTypes[cnt] = miAbove.isIBCmot ? miAbove.rribcFlipType : 0;
#endif
#endif
#endif
#if JVET_AC0112_IBC_LIC
#if JVET_AA0070_RRIBC
      mrgCtx.ibcLicFlags[cnt] = miAbove.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miAbove.useIbcLic : false;
#else
      mrgCtx.ibcLicFlags[cnt] = miAbove.isIBCmot ? miAbove.useIbcLic : false;
#endif
#endif
#if JVET_AE0159_FIBC
#if JVET_AA0070_RRIBC
      mrgCtx.ibcFilterFlags[cnt] = miAbove.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miAbove.useIbcFilter : false;
#else
      mrgCtx.ibcFilterFlags[cnt] = miAbove.isIBCmot ? miAbove.useIbcFilter : false;
#endif
#endif

#if JVET_Z0084_IBC_TM
      if( !mrgCtx.xCheckSimilarIBCMotion(cnt, mvdSimilarityThresh) )
#endif
      {
        if (mrgCandIdx == cnt)
        {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
          mrgCtx.numAMVPMergeCand = cnt;
#endif
          return;
        }
        cnt++;

        // early termination
        if (cnt == maxNumMergeCand)
        {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
          mrgCtx.numAMVPMergeCand = cnt;
#endif
          return;
        }
      }
#if JVET_Y0058_IBC_LIST_MODIFY
      }
#endif
    }
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (miAbove.isIBCmot && miAbove.interDir == 3 && (!isAvailableA1 || (miAbove != miLeft)))
    {
      MotionInfo miAboveL1 = miAbove;
      miAboveL1.mv[0] = miAbove.mv[1];
      miAboveL1.refIdx[1] = -1;
#if JVET_AA0070_RRIBC
      rribcAdjustMotion(pu, &cPos, miAboveL1);
#endif

#if JVET_Y0058_IBC_LIST_MODIFY
      if (checkIsIBCCandidateValid(pu, miAboveL1))
      {
#endif
        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = 1;
        // get Mv from Above
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAboveL1.mv[0], miAboveL1.refIdx[0]);
#if JVET_AA0070_RRIBC
#if IBC_TM_MRG
#if JVET_AA0061_IBC_MBVD
        mrgCtx.rribcFlipTypes[cnt] = (miAboveL1.isIBCmot && !pu.tmMergeFlag && !pu.ibcMbvdMergeFlag) ? miAboveL1.rribcFlipType : 0;
#else
        mrgCtx.rribcFlipTypes[cnt] = (miAboveL1.isIBCmot && !pu.tmMergeFlag) ? miAboveL1.rribcFlipType : 0;
#endif
#else
#if JVET_AA0061_IBC_MBVD
        mrgCtx.rribcFlipTypes[cnt] = (miAboveL1.isIBCmot && !pu.ibcMbvdMergeFlag) ? miAboveL1.rribcFlipType : 0;
#else
        mrgCtx.rribcFlipTypes[cnt] = miAboveL1.isIBCmot ? miAboveL1.rribcFlipType : 0;
#endif
#endif
#endif
#if JVET_AC0112_IBC_LIC
#if JVET_AA0070_RRIBC
        mrgCtx.ibcLicFlags[cnt] = miAboveL1.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miAboveL1.useIbcLic : false;
#else
        mrgCtx.ibcLicFlags[cnt] = miAboveL1.isIBCmot ? miAboveL1.useIbcLic : false;
#endif
#endif
#if JVET_AE0159_FIBC
#if JVET_AA0070_RRIBC
        mrgCtx.ibcFilterFlags[cnt] = miAboveL1.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miAboveL1.useIbcFilter : false;
#else
        mrgCtx.ibcFilterFlags[cnt] = miAboveL1.isIBCmot ? miAboveL1.useIbcFilter : false;
#endif
#endif

#if JVET_Z0084_IBC_TM
        if( !mrgCtx.xCheckSimilarIBCMotion(cnt, mvdSimilarityThresh) )
#endif
        {
          if (mrgCandIdx == cnt)
          {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
            mrgCtx.numAMVPMergeCand = cnt;
#endif
            return;
          }
          cnt++;

          // early termination
          if (cnt == maxNumMergeCand)
          {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
            mrgCtx.numAMVPMergeCand = cnt;
#endif
            return;
          }
        }
#if JVET_Y0058_IBC_LIST_MODIFY
      }
#endif
    }
#endif
  }

#if JVET_Y0058_IBC_LIST_MODIFY
  // above right
  const PredictionUnit *puAboveRight = cs.getPURestricted(posRT.offset(1, -1), pu, pu.chType);
#if JVET_AB0061_ITMP_BV_FOR_IBC
  bool isAvailableB0 = puAboveRight && pu.cu != puAboveRight->cu && (CU::isIBC(*puAboveRight->cu) || puAboveRight->cu->tmpFlag);
#else
  bool isAvailableB0 = puAboveRight && pu.cu != puAboveRight->cu && CU::isIBC(*puAboveRight->cu);
#endif
  if (isGt4x4 && isAvailableB0)
  {
    miAboveRight = puAboveRight->getMotionInfo(posRT.offset(1, -1));
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    miAboveRight.useAltHpelIf = pu.mergeFlag ? miAboveRight.useAltHpelIf : (pu.cu->imv == IMV_HPEL);
#endif
#if JVET_AA0070_RRIBC
    Position cPos(puAboveRight->lx() + (puAboveRight->lwidth() >> 1), puAboveRight->ly() + (puAboveRight->lheight() >> 1));
    rribcAdjustMotion(pu, &cPos, miAboveRight);
#endif

    if ((!isAvailableB1 || (miAbove != miAboveRight)) && (!isAvailableA1 || (miLeft != miAboveRight)))
    {
      if (checkIsIBCCandidateValid(pu, miAboveRight))
      {
        // get Inter Dir
#if JVET_AE0169_BIPREDICTIVE_IBC
        mrgCtx.interDirNeighbours[cnt] = 1;
#else
        mrgCtx.interDirNeighbours[cnt] = miAboveRight.interDir;
#endif
        // get Mv from Above-right
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAboveRight.mv[0], miAboveRight.refIdx[0]);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
        mrgCtx.useAltHpelIf[cnt] = miAboveRight.useAltHpelIf;
#endif
#if JVET_AA0070_RRIBC
#if IBC_TM_MRG
#if JVET_AA0061_IBC_MBVD
        mrgCtx.rribcFlipTypes[cnt] = (miAboveRight.isIBCmot && !pu.tmMergeFlag && !pu.ibcMbvdMergeFlag) ? miAboveRight.rribcFlipType : 0;
#else
        mrgCtx.rribcFlipTypes[cnt] = (miAboveRight.isIBCmot && !pu.tmMergeFlag) ? miAboveRight.rribcFlipType : 0;
#endif
#else
#if JVET_AA0061_IBC_MBVD
        mrgCtx.rribcFlipTypes[cnt] = (miAboveRight.isIBCmot && !pu.ibcMbvdMergeFlag) ? miAboveRight.rribcFlipType : 0;
#else
        mrgCtx.rribcFlipTypes[cnt] = miAboveRight.isIBCmot ? miAboveRight.rribcFlipType : 0;
#endif
#endif
#endif
#if JVET_AC0112_IBC_LIC
#if JVET_AA0070_RRIBC
        mrgCtx.ibcLicFlags[cnt] = miAboveRight.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miAboveRight.useIbcLic : false;
#else
        mrgCtx.ibcLicFlags[cnt] = miAboveRight.isIBCmot? miAboveRight.useIbcLic : false;
#endif
#endif
#if JVET_AE0159_FIBC
#if JVET_AA0070_RRIBC
        mrgCtx.ibcFilterFlags[cnt] = miAboveRight.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miAboveRight.useIbcFilter : false;
#else
        mrgCtx.ibcFilterFlags[cnt] = miAboveRight.isIBCmot? miAboveRight.useIbcFilter : false;
#endif
#endif

#if JVET_Z0084_IBC_TM
        if( !mrgCtx.xCheckSimilarIBCMotion(cnt, mvdSimilarityThresh) )
#endif
        {
          if (mrgCandIdx == cnt)
          {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
            mrgCtx.numAMVPMergeCand = cnt;
#endif
            return;
          }
          cnt++;

          // early termination
          if (cnt == maxNumMergeCand)
          {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
            mrgCtx.numAMVPMergeCand = cnt;
#endif
            return;
          }
        }
      }
    }
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (miAboveRight.isIBCmot && miAboveRight.interDir == 3 && (!isAvailableB1 || (miAbove != miAboveRight)) && (!isAvailableA1 || (miLeft != miAboveRight)))
    {
      MotionInfo miAboveRightL1 = miAboveRight;
      miAboveRightL1.mv[0] = miAboveRightL1.mv[1];
      miAboveRightL1.refIdx[1] = -1;
#if JVET_AA0070_RRIBC
      rribcAdjustMotion(pu, &cPos, miAboveRightL1);
#endif

      if (checkIsIBCCandidateValid(pu, miAboveRightL1))
      {
        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = 1;
        // get Mv from Above-right
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAboveRightL1.mv[0], miAboveRightL1.refIdx[0]);
#if JVET_AA0070_RRIBC
#if IBC_TM_MRG
#if JVET_AA0061_IBC_MBVD
        mrgCtx.rribcFlipTypes[cnt] = (miAboveRightL1.isIBCmot && !pu.tmMergeFlag && !pu.ibcMbvdMergeFlag) ? miAboveRightL1.rribcFlipType : 0;
#else
        mrgCtx.rribcFlipTypes[cnt] = (miAboveRightL1.isIBCmot && !pu.tmMergeFlag) ? miAboveRightL1.rribcFlipType : 0;
#endif
#else
#if JVET_AA0061_IBC_MBVD
        mrgCtx.rribcFlipTypes[cnt] = (miAboveRightL1.isIBCmot && !pu.ibcMbvdMergeFlag) ? miAboveRightL1.rribcFlipType : 0;
#else
        mrgCtx.rribcFlipTypes[cnt] = miAboveRightL1.isIBCmot ? miAboveRightL1.rribcFlipType : 0;
#endif
#endif
#endif
#if JVET_AC0112_IBC_LIC
#if JVET_AA0070_RRIBC
        mrgCtx.ibcLicFlags[cnt] = miAboveRightL1.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miAboveRightL1.useIbcLic : false;
#else
        mrgCtx.ibcLicFlags[cnt] = miAboveRightL1.isIBCmot? miAboveRightL1.useIbcLic : false;
#endif
#endif
#if JVET_AE0159_FIBC
#if JVET_AA0070_RRIBC
        mrgCtx.ibcFilterFlags[cnt] = miAboveRightL1.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miAboveRightL1.useIbcFilter : false;
#else
        mrgCtx.ibcFilterFlags[cnt] = miAboveRightL1.isIBCmot? miAboveRightL1.useIbcFilter : false;
#endif
#endif

#if JVET_Z0084_IBC_TM
        if( !mrgCtx.xCheckSimilarIBCMotion(cnt, mvdSimilarityThresh) )
#endif
        {
          if (mrgCandIdx == cnt)
          {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
            mrgCtx.numAMVPMergeCand = cnt;
#endif
            return;
          }
          cnt++;

          // early termination
          if (cnt == maxNumMergeCand)
          {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
            mrgCtx.numAMVPMergeCand = cnt;
#endif
            return;
          }
        }
      }
    }
#endif
  }

  //left bottom
  const PredictionUnit *puLeftBottom = cs.getPURestricted(posLB.offset(-1, 1), pu, pu.chType);
#if JVET_AB0061_ITMP_BV_FOR_IBC
  bool isAvailableA0 = puLeftBottom && pu.cu != puLeftBottom->cu && (CU::isIBC(*puLeftBottom->cu) || puLeftBottom->cu->tmpFlag);
#else
  bool isAvailableA0 = puLeftBottom && pu.cu != puLeftBottom->cu && CU::isIBC(*puLeftBottom->cu);
#endif
  if (isGt4x4 && isAvailableA0)
  {
    miBelowLeft = puLeftBottom->getMotionInfo(posLB.offset(-1, 1));
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    miBelowLeft.useAltHpelIf = pu.mergeFlag ? miBelowLeft.useAltHpelIf : (pu.cu->imv == IMV_HPEL);
#endif
#if JVET_AA0070_RRIBC
    Position cPos(puLeftBottom->lx() + (puLeftBottom->lwidth() >> 1), puLeftBottom->ly() + (puLeftBottom->lheight() >> 1));
    rribcAdjustMotion(pu, &cPos, miBelowLeft);
#endif

    if ((!isAvailableA1 || (miBelowLeft != miLeft)) && (!isAvailableB1 || (miBelowLeft != miAbove)) && (!isAvailableB0 || (miBelowLeft != miAboveRight)))
    {
      if (checkIsIBCCandidateValid(pu, miBelowLeft))
      {
        // get Inter Dir
#if JVET_AE0169_BIPREDICTIVE_IBC
        mrgCtx.interDirNeighbours[cnt] = 1;
#else
        mrgCtx.interDirNeighbours[cnt] = miBelowLeft.interDir;
#endif
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miBelowLeft.mv[0], miBelowLeft.refIdx[0]);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
        mrgCtx.useAltHpelIf[cnt] = miBelowLeft.useAltHpelIf;
#endif
#if JVET_AA0070_RRIBC
#if IBC_TM_MRG
#if JVET_AA0061_IBC_MBVD
        mrgCtx.rribcFlipTypes[cnt] = (miBelowLeft.isIBCmot && !pu.tmMergeFlag && !pu.ibcMbvdMergeFlag) ? miBelowLeft.rribcFlipType : 0;
#else
        mrgCtx.rribcFlipTypes[cnt] = (miBelowLeft.isIBCmot && !pu.tmMergeFlag) ? miBelowLeft.rribcFlipType : 0;
#endif
#else
#if JVET_AA0061_IBC_MBVD
        mrgCtx.rribcFlipTypes[cnt] = (miBelowLeft.isIBCmot && !pu.ibcMbvdMergeFlag) ? miBelowLeft.rribcFlipType : 0;
#else
        mrgCtx.rribcFlipTypes[cnt] = miBelowLeft.isIBCmot ? miBelowLeft.rribcFlipType : 0;
#endif
#endif
#endif
#if JVET_AC0112_IBC_LIC
#if JVET_AA0070_RRIBC
        mrgCtx.ibcLicFlags[cnt] = miBelowLeft.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miBelowLeft.useIbcLic : false;
#else
        mrgCtx.ibcLicFlags[cnt] = miBelowLeft.isIBCmot ? miBelowLeft.useIbcLic : false;
#endif
#endif
#if JVET_AE0159_FIBC
#if JVET_AA0070_RRIBC
        mrgCtx.ibcFilterFlags[cnt] = miBelowLeft.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miBelowLeft.useIbcFilter : false;
#else
        mrgCtx.ibcFilterFlags[cnt] = miBelowLeft.isIBCmot ? miBelowLeft.useIbcFilter : false;
#endif
#endif

#if JVET_Z0084_IBC_TM
        if( !mrgCtx.xCheckSimilarIBCMotion(cnt, mvdSimilarityThresh) )
#endif
        {
          if (mrgCandIdx == cnt)
          {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
            mrgCtx.numAMVPMergeCand = cnt;
#endif
            return;
          }
          cnt++;

          // early termination
          if (cnt == maxNumMergeCand)
          {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
            mrgCtx.numAMVPMergeCand = cnt;
#endif
            return;
          }
        }
      }
    }
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (miBelowLeft.isIBCmot && miBelowLeft.interDir == 3 && (!isAvailableA1 || (miBelowLeft != miLeft)) && (!isAvailableB1 || (miBelowLeft != miAbove)) && (!isAvailableB0 || (miBelowLeft != miAboveRight)))
    {
      MotionInfo miBelowLeftL1 = miBelowLeft;
      miBelowLeftL1.mv[0] = miBelowLeftL1.mv[1];
      miBelowLeftL1.refIdx[1] = -1;
#if JVET_AA0070_RRIBC
      rribcAdjustMotion(pu, &cPos, miBelowLeftL1);
#endif

      if (checkIsIBCCandidateValid(pu, miBelowLeftL1))
      {
        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = 1;
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miBelowLeftL1.mv[0], miBelowLeftL1.refIdx[0]);
#if JVET_AA0070_RRIBC
#if IBC_TM_MRG
#if JVET_AA0061_IBC_MBVD
        mrgCtx.rribcFlipTypes[cnt] = (miBelowLeftL1.isIBCmot && !pu.tmMergeFlag && !pu.ibcMbvdMergeFlag) ? miBelowLeftL1.rribcFlipType : 0;
#else
        mrgCtx.rribcFlipTypes[cnt] = (miBelowLeftL1.isIBCmot && !pu.tmMergeFlag) ? miBelowLeftL1.rribcFlipType : 0;
#endif
#else
#if JVET_AA0061_IBC_MBVD
        mrgCtx.rribcFlipTypes[cnt] = (miBelowLeftL1.isIBCmot && !pu.ibcMbvdMergeFlag) ? miBelowLeftL1.rribcFlipType : 0;
#else
        mrgCtx.rribcFlipTypes[cnt] = miBelowLeftL1.isIBCmot ? miBelowLeftL1.rribcFlipType : 0;
#endif
#endif
#endif
#if JVET_AC0112_IBC_LIC
#if JVET_AA0070_RRIBC
        mrgCtx.ibcLicFlags[cnt] = miBelowLeftL1.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miBelowLeftL1.useIbcLic : false;
#else
        mrgCtx.ibcLicFlags[cnt] = miBelowLeftL1.isIBCmot ? miBelowLeftL1.useIbcLic : false;
#endif
#endif
#if JVET_AE0159_FIBC
#if JVET_AA0070_RRIBC
        mrgCtx.ibcFilterFlags[cnt] = miBelowLeftL1.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miBelowLeftL1.useIbcFilter : false;
#else
        mrgCtx.ibcFilterFlags[cnt] = miBelowLeftL1.isIBCmot ? miBelowLeftL1.useIbcFilter : false;
#endif
#endif

#if JVET_Z0084_IBC_TM
        if( !mrgCtx.xCheckSimilarIBCMotion(cnt, mvdSimilarityThresh) )
#endif
        {
          if (mrgCandIdx == cnt)
          {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
            mrgCtx.numAMVPMergeCand = cnt;
#endif
            return;
          }
          cnt++;

          // early termination
          if (cnt == maxNumMergeCand)
          {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
            mrgCtx.numAMVPMergeCand = cnt;
#endif
            return;
          }
        }
      }
    }
#endif
  }

  // above left
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL && JVET_Z0084_IBC_TM && JVET_Z0075_IBC_HMVP_ENLARGE && JVET_W0090_ARMC_TM
  if ((cnt < 4 && pu.cs->sps->getUseAML() && pu.cs->sps->getTMnoninterToolsEnableFlag() && mrgCandIdx >= 0) || !(pu.cs->sps->getUseAML() && pu.cs->sps->getTMnoninterToolsEnableFlag())) //Only for AMVP case
#elif JVET_Z0084_IBC_TM && JVET_Z0075_IBC_HMVP_ENLARGE && JVET_W0090_ARMC_TM
  if ((cnt < 4 && pu.cs->sps->getUseAML() && mrgCandIdx >= 0)
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_W0090_ARMC_TM
         || !pu.cs->sps->getUseAML()
#endif
  ) //Only for AMVP case
#elif !JVET_Z0075_IBC_HMVP_ENLARGE
  if (cnt < 4)
#endif
  {
    const PredictionUnit *puAboveLeft = cs.getPURestricted(posLT.offset(-1, -1), pu, pu.chType);
#if JVET_AB0061_ITMP_BV_FOR_IBC
    bool isAvailableB2 = puAboveLeft && pu.cu != puAboveLeft->cu && (CU::isIBC(*puAboveLeft->cu) || puAboveLeft->cu->tmpFlag);
#else
    bool isAvailableB2 = puAboveLeft && pu.cu != puAboveLeft->cu && CU::isIBC(*puAboveLeft->cu);
#endif
    if (isGt4x4 && isAvailableB2)
    {
      miAboveLeft = puAboveLeft->getMotionInfo(posLT.offset(-1, -1));
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      miAboveLeft.useAltHpelIf = pu.mergeFlag ? miAboveLeft.useAltHpelIf : (pu.cu->imv == IMV_HPEL);
#endif
#if JVET_AA0070_RRIBC
      Position cPos(puAboveLeft->lx() + (puAboveLeft->lwidth() >> 1), puAboveLeft->ly() + (puAboveLeft->lheight() >> 1));
      rribcAdjustMotion(pu, &cPos, miAboveLeft);
#endif

      if ((!isAvailableA1 || (miLeft != miAboveLeft)) && (!isAvailableB1 || (miAbove != miAboveLeft)) && (!isAvailableA0 || (miBelowLeft != miAboveLeft)) && (!isAvailableB0 || (miAboveRight != miAboveLeft)))
      {
        if (checkIsIBCCandidateValid(pu, miAboveLeft))
        {
          // get Inter Dir
#if JVET_AE0169_BIPREDICTIVE_IBC
          mrgCtx.interDirNeighbours[cnt] = 1;
#else
          mrgCtx.interDirNeighbours[cnt] = miAboveLeft.interDir;
#endif
          mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAboveLeft.mv[0], miAboveLeft.refIdx[0]);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
          mrgCtx.useAltHpelIf[cnt] = miAboveLeft.useAltHpelIf;
#endif
#if JVET_AA0070_RRIBC
#if IBC_TM_MRG
#if JVET_AA0061_IBC_MBVD
          mrgCtx.rribcFlipTypes[cnt] = (miAboveLeft.isIBCmot && !pu.tmMergeFlag && !pu.ibcMbvdMergeFlag) ? miAboveLeft.rribcFlipType : 0;
#else
        mrgCtx.rribcFlipTypes[cnt] = (miAboveLeft.isIBCmot && !pu.tmMergeFlag) ? miAboveLeft.rribcFlipType : 0;
#endif
#else
#if JVET_AA0061_IBC_MBVD
          mrgCtx.rribcFlipTypes[cnt] = (miAboveLeft.isIBCmot && !pu.ibcMbvdMergeFlag) ? miAboveLeft.rribcFlipType : 0;
#else
        mrgCtx.rribcFlipTypes[cnt] = (miAboveLeft.isIBCmot) ? miAboveLeft.rribcFlipType : 0;
#endif
#endif
#endif
#if JVET_AC0112_IBC_LIC
#if JVET_AA0070_RRIBC
          mrgCtx.ibcLicFlags[cnt] = miAboveLeft.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miAboveLeft.useIbcLic : false;
#else
          mrgCtx.ibcLicFlags[cnt] = miAboveLeft.isIBCmot ? miAboveLeft.useIbcLic : false;
#endif
#endif
#if JVET_AE0159_FIBC
#if JVET_AA0070_RRIBC
          mrgCtx.ibcFilterFlags[cnt] = miAboveLeft.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miAboveLeft.useIbcFilter : false;
#else
          mrgCtx.ibcFilterFlags[cnt] = miAboveLeft.isIBCmot ? miAboveLeft.useIbcFilter : false;
#endif
#endif

#if JVET_Z0084_IBC_TM
          if( !mrgCtx.xCheckSimilarIBCMotion(cnt, mvdSimilarityThresh) )
#endif
          {
            if (mrgCandIdx == cnt)
            {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
              mrgCtx.numAMVPMergeCand = cnt;
#endif
              return;
            }
            cnt++;

            // early termination
            if (cnt == maxNumMergeCand)
            {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
              mrgCtx.numAMVPMergeCand = cnt;
#endif
              return;
            }
          }
        }
      }
#if JVET_AE0169_BIPREDICTIVE_IBC
      if (miAboveLeft.isIBCmot && miAboveLeft.interDir == 3 &&
        (!isAvailableA1 || (miLeft != miAboveLeft)) && (!isAvailableB1 || (miAbove != miAboveLeft)) && (!isAvailableA0 || (miBelowLeft != miAboveLeft)) && (!isAvailableB0 || (miAboveRight != miAboveLeft)))
      {
        MotionInfo miAboveLeftL1 = miAboveLeft;
        miAboveLeftL1.mv[0] = miAboveLeftL1.mv[1];
        miAboveLeftL1.refIdx[1] = -1;
#if JVET_AA0070_RRIBC
        rribcAdjustMotion(pu, &cPos, miAboveLeftL1);
#endif

        if (checkIsIBCCandidateValid(pu, miAboveLeftL1))
        {
          // get Inter Dir
          mrgCtx.interDirNeighbours[cnt] = 1;
          mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAboveLeftL1.mv[0], miAboveLeftL1.refIdx[0]);
#if JVET_AA0070_RRIBC
#if IBC_TM_MRG
#if JVET_AA0061_IBC_MBVD
          mrgCtx.rribcFlipTypes[cnt] = (miAboveLeftL1.isIBCmot && !pu.tmMergeFlag && !pu.ibcMbvdMergeFlag) ? miAboveLeftL1.rribcFlipType : 0;
#else
          mrgCtx.rribcFlipTypes[cnt] = (miAboveLeftL1.isIBCmot && !pu.tmMergeFlag) ? miAboveLeftL1.rribcFlipType : 0;
#endif
#else
#if JVET_AA0061_IBC_MBVD
          mrgCtx.rribcFlipTypes[cnt] = (miAboveLeftL1.isIBCmot && !pu.ibcMbvdMergeFlag) ? miAboveLeftL1.rribcFlipType : 0;
#else
          mrgCtx.rribcFlipTypes[cnt] = (miAboveLeftL1.isIBCmot) ? miAboveLeftL1.rribcFlipType : 0;
#endif
#endif
#endif
#if JVET_AC0112_IBC_LIC
#if JVET_AA0070_RRIBC
          mrgCtx.ibcLicFlags[cnt] = miAboveLeftL1.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miAboveLeftL1.useIbcLic : false;
#else
          mrgCtx.ibcLicFlags[cnt] = miAboveLeftL1.isIBCmot ? miAboveLeftL1.useIbcLic : false;
#endif
#endif
#if JVET_AE0159_FIBC
#if JVET_AA0070_RRIBC
          mrgCtx.ibcFilterFlags[cnt] = miAboveLeftL1.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miAboveLeftL1.useIbcFilter : false;
#else
          mrgCtx.ibcFilterFlags[cnt] = miAboveLeftL1.isIBCmot ? miAboveLeftL1.useIbcFilter : false;
#endif
#endif

#if JVET_Z0084_IBC_TM
          if( !mrgCtx.xCheckSimilarIBCMotion(cnt, mvdSimilarityThresh) )
#endif
          {
            if (mrgCandIdx == cnt)
            {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
              mrgCtx.numAMVPMergeCand = cnt;
#endif
              return;
            }
            cnt++;

            // early termination
            if (cnt == maxNumMergeCand)
            {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
              mrgCtx.numAMVPMergeCand = cnt;
#endif
              return;
            }
          }
        }
      }
#endif
    }
  }
#endif
#if JVET_AE0094_IBC_NONADJACENT_SPATIAL_CANDIDATES
  if (pu.cs->sps->getUseIbcNonAdjCand() && pu.mergeFlag && cnt != maxNumMergeCand)
  {
    MotionInfo miNonAdjacent;
    int        offsetX             = 0;
    int        offsetY             = 0;
    int        offsetX0            = 0;
    int        offsetX1            = 0;
    int        offsetX2            = pu.Y().width >> 1;
    int        offsetY0            = 0;
    int        offsetY1            = 0;
    int        offsetY2            = pu.Y().height >> 1;
    const int  iNonAdjCandNum[7] = { 5, 5, 5, 5, 5, 5, 5 };
    const int  idxMap[7][9]        = { { 0, 1, 2, 3, 4 },
                               { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
                               { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
                               { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
                               { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
                               { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
                               { 0, 1, 2, 3, 4, 5, 6, 7, 8 } };
#if JVET_AA0070_RRIBC
    int cPosCurX = pu.lx() + (pu.lwidth() >> 1);
    int cPosCurY = pu.ly() + (pu.lheight() >> 1);
    int thW      = (pu.lwidth() >> 1) * 3;
    int thH      = (pu.lheight() >> 1) * 3;
#endif
    for (int iDistanceIndex = 0; iDistanceIndex < 7 && cnt < maxNumMergeCand; iDistanceIndex++)
    {
      const int iNADistanceHor = pu.Y().width * (iDistanceIndex + 1);
      const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);
      for (int iNonAdjIdx = 0; iNonAdjIdx < iNonAdjCandNum[iDistanceIndex] && cnt < maxNumMergeCand; iNonAdjIdx++)
      {
        switch (idxMap[iDistanceIndex][iNonAdjIdx])
        {
        case 0:
          offsetX = offsetX0 = -iNADistanceHor - 1;
          offsetY = offsetY0 = pu.Y().height + iNADistanceVer - 1;
          break;
        case 1:
          offsetX = offsetX1 = pu.Y().width + iNADistanceHor - 1;
          offsetY = offsetY1 = -iNADistanceVer - 1;
          break;
        case 2:
          offsetX = offsetX2;
          offsetY = offsetY1;
          break;
        case 3:
          offsetX = offsetX0;
          offsetY = offsetY2;
          break;
        case 4:
          offsetX = offsetX0;
          offsetY = offsetY1;
          break;
        case 5:
          offsetX = -1;
          offsetY = offsetY0;
          break;
        case 6:
          offsetX = offsetX1;
          offsetY = -1;
          break;
        case 7:
          offsetX = offsetX0 >> 1;
          offsetY = offsetY0;
          break;
        case 8:
          offsetX = offsetX1;
          offsetY = offsetY1 >> 1;
          break;
        default:
          printf("error!");
          exit(0);
          break;
        }

        const PredictionUnit *puNonAdjacent = cs.getPURestricted(posLT.offset(offsetX, offsetY), pu, pu.chType);
        bool isAvailableNonAdjacent = puNonAdjacent && pu.cu != puNonAdjacent->cu && (CU::isIBC(*puNonAdjacent->cu) || puNonAdjacent->cu->tmpFlag);

        if (isGt4x4 && isAvailableNonAdjacent)
        {
          miNonAdjacent = puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY));
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
          miNonAdjacent.useAltHpelIf = pu.mergeFlag ? miNonAdjacent.useAltHpelIf : (pu.cu->imv == IMV_HPEL);
#endif
#if JVET_AA0070_RRIBC
          Position cPos(puNonAdjacent->lx() + (puNonAdjacent->lwidth() >> 1),
                        puNonAdjacent->ly() + (puNonAdjacent->lheight() >> 1));
          if (pu.mergeFlag || ((abs(cPosCurX - (int) cPos.x) <= thW) && (abs(cPosCurY - (int) cPos.y) <= thH)))
          {
            rribcAdjustMotion(pu, &cPos, miNonAdjacent);
          }
#endif
          if (checkIsIBCCandidateValid(pu, miNonAdjacent))
          {
            // get Inter Dir
#if JVET_AE0169_BIPREDICTIVE_IBC
            mrgCtx.interDirNeighbours[cnt] = 1;
#else
            mrgCtx.interDirNeighbours[cnt] = miNonAdjacent.interDir;
#endif
            mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNonAdjacent.mv[0], miNonAdjacent.refIdx[0]);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
            mrgCtx.useAltHpelIf[cnt] = miNonAdjacent.useAltHpelIf;
#endif
#if JVET_AA0070_RRIBC
#if IBC_TM_MRG
#if JVET_AA0061_IBC_MBVD
            mrgCtx.rribcFlipTypes[cnt] =
              (miNonAdjacent.isIBCmot && !pu.tmMergeFlag && !pu.ibcMbvdMergeFlag) ? miNonAdjacent.rribcFlipType : 0;
#else
            mrgCtx.rribcFlipTypes[cnt] = (miAboveLeft.isIBCmot && !pu.tmMergeFlag) ? miAboveLeft.rribcFlipType : 0;
#endif
#else
#if JVET_AA0061_IBC_MBVD
            mrgCtx.rribcFlipTypes[cnt] = (miAboveLeft.isIBCmot && !pu.ibcMbvdMergeFlag) ? miAboveLeft.rribcFlipType : 0;
#else
            mrgCtx.rribcFlipTypes[cnt] = (miAboveLeft.isIBCmot) ? miAboveLeft.rribcFlipType : 0;
#endif
#endif
#endif
#if JVET_AC0112_IBC_LIC
#if JVET_AA0070_RRIBC
            mrgCtx.ibcLicFlags[cnt] =
              miNonAdjacent.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miNonAdjacent.useIbcLic : false;
#else
            mrgCtx.ibcLicFlags[cnt]    = miNonAdjacent.isIBCmot ? miNonAdjacent.useIbcLic : false;
#endif
#endif
#if JVET_AE0159_FIBC
#if JVET_AA0070_RRIBC
            mrgCtx.ibcFilterFlags[cnt] = miNonAdjacent.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miNonAdjacent.useIbcFilter : false;
#else
            mrgCtx.ibcFilterFlags[cnt] = miNonAdjacent.isIBCmot ? miNonAdjacent.useIbcFilter : false;
#endif
#endif
#if JVET_Z0084_IBC_TM
            if (!mrgCtx.xCheckSimilarIBCMotion(cnt, mvdSimilarityThresh))
#endif
            {
              if (mrgCandIdx == cnt)
              {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                mrgCtx.numAMVPMergeCand = cnt;
#endif
                return;
              }
              cnt++;
              // early termination
              if (cnt == maxNumMergeCand)
              {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                mrgCtx.numAMVPMergeCand = cnt;
#endif
                return;
              }
            }
          }
#if JVET_AE0169_BIPREDICTIVE_IBC
          if (miNonAdjacent.isIBCmot && miNonAdjacent.interDir == 3)
          {
            MotionInfo miNonAdjacentL1 = miNonAdjacent;
            miNonAdjacentL1.mv[0]      = miNonAdjacentL1.mv[1];
            miNonAdjacentL1.refIdx[1]  = -1;
#if JVET_AA0070_RRIBC
            if (pu.mergeFlag || ((abs(cPosCurX - (int) cPos.x) <= thW) && (abs(cPosCurY - (int) cPos.y) <= thH)))
            {
              rribcAdjustMotion(pu, &cPos, miNonAdjacentL1);
            }
#endif

            if (checkIsIBCCandidateValid(pu, miNonAdjacentL1))
            {
              // get Inter Dir
              mrgCtx.interDirNeighbours[cnt] = 1;
              mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNonAdjacentL1.mv[0], miNonAdjacentL1.refIdx[0]);
#if JVET_AA0070_RRIBC
#if IBC_TM_MRG
#if JVET_AA0061_IBC_MBVD
              mrgCtx.rribcFlipTypes[cnt] =
                (miNonAdjacentL1.isIBCmot && !pu.tmMergeFlag && !pu.ibcMbvdMergeFlag) ? miNonAdjacentL1.rribcFlipType : 0;
#else
              mrgCtx.rribcFlipTypes[cnt] =
                (miNonAdjacentL1.isIBCmot && !pu.tmMergeFlag) ? miNonAdjacentL1.rribcFlipType : 0;
#endif
#else
#if JVET_AA0061_IBC_MBVD
              mrgCtx.rribcFlipTypes[cnt] =
                (miNonAdjacentL1.isIBCmot && !pu.ibcMbvdMergeFlag) ? miNonAdjacentL1.rribcFlipType : 0;
#else
              mrgCtx.rribcFlipTypes[cnt] = (miNonAdjacentL1.isIBCmot) ? miNonAdjacentL1.rribcFlipType : 0;
#endif
#endif
#endif
#if JVET_AC0112_IBC_LIC
#if JVET_AA0070_RRIBC
              mrgCtx.ibcLicFlags[cnt] =
                miNonAdjacentL1.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miNonAdjacentL1.useIbcLic : false;
#else
              mrgCtx.ibcLicFlags[cnt] = miNonAdjacentL1.isIBCmot ? miNonAdjacentL1.useIbcLic : false;
#endif
#endif
#if JVET_AE0159_FIBC
#if JVET_AA0070_RRIBC
              mrgCtx.ibcFilterFlags[cnt] = miNonAdjacentL1.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miNonAdjacentL1.useIbcFilter : false;
#else
              mrgCtx.ibcFilterFlags[cnt] = miNonAdjacentL1.isIBCmot ? miNonAdjacentL1.useIbcFilter : false;
#endif
#endif

#if JVET_Z0084_IBC_TM
              if (!mrgCtx.xCheckSimilarIBCMotion(cnt, mvdSimilarityThresh))
#endif
              {
                if (mrgCandIdx == cnt)
                {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                  mrgCtx.numAMVPMergeCand = cnt;
#endif
                  return;
                }
                cnt++;

                // early termination
                if (cnt == maxNumMergeCand)
                {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                  mrgCtx.numAMVPMergeCand = cnt;
#endif
                  return;
                }
              }
            }
          }
#endif
        }
      }
    }
  }
  if (pu.cs->sps->getUseIbcNonAdjCand() && !pu.mergeFlag && cnt != maxNumMergeCand)
  {
    MotionInfo miNonAdjacent;
    int        offsetX             = 0;
    int        offsetY             = 0;
    const int  iNonAdjCandNum[4] = { 5, 5, 5, 5 };
    const int  idxMap[4][5]        = { { 0, 1, 2, 3, 4 }, { 0, 1, 2, 3, 4 }, { 0, 1, 2, 3, 4 }, { 0, 1, 2, 3, 4 } };
#if JVET_AA0070_RRIBC
    int cPosCurX = pu.lx() + (pu.lwidth() >> 1);
    int cPosCurY = pu.ly() + (pu.lheight() >> 1);
    int thW      = (pu.lwidth() >> 1) * 3;
    int thH      = (pu.lheight() >> 1) * 3;
#endif
    for (int iDistanceIndex = 0; iDistanceIndex < NADISTANCE_LEVEL && cnt < maxNumMergeCand; iDistanceIndex++)
    {
      const int iNADistanceHor = pu.Y().width * (iDistanceIndex + 1);
      const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);
      for (int iNonAdjIdx = 0; iNonAdjIdx < iNonAdjCandNum[iDistanceIndex] && cnt < maxNumMergeCand; iNonAdjIdx++)
      {
        switch (idxMap[iDistanceIndex][iNonAdjIdx])
        {
        case 0:
          offsetX = -iNADistanceHor - 1;
          offsetY = pu.Y().height + iNADistanceVer - 1;
          break;
        case 1:
          offsetX = pu.Y().width + iNADistanceHor - 1;
          offsetY = -iNADistanceVer - 1;
          break;
        case 2:
          offsetX = pu.Y().width >> 1;
          offsetY = -iNADistanceVer - 1;
          break;
        case 3:
          offsetX = -iNADistanceHor - 1;
          offsetY = pu.Y().height >> 1;
          break;
        case 4:
          offsetX = -iNADistanceHor - 1;
          offsetY = -iNADistanceVer - 1;
          break;
        default:
          printf("error!");
          exit(0);
          break;
        }

        const PredictionUnit *puNonAdjacent = cs.getPURestricted(posLT.offset(offsetX, offsetY), pu, pu.chType);
        bool isAvailableNonAdjacent = puNonAdjacent && pu.cu != puNonAdjacent->cu && (CU::isIBC(*puNonAdjacent->cu) || puNonAdjacent->cu->tmpFlag);

        if (isGt4x4 && isAvailableNonAdjacent)
        {
          miNonAdjacent = puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY));
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
          miNonAdjacent.useAltHpelIf = pu.mergeFlag ? miNonAdjacent.useAltHpelIf : (pu.cu->imv == IMV_HPEL);
#endif
#if JVET_AA0070_RRIBC
          Position cPos(puNonAdjacent->lx() + (puNonAdjacent->lwidth() >> 1),
                        puNonAdjacent->ly() + (puNonAdjacent->lheight() >> 1));
          if (pu.mergeFlag || ((abs(cPosCurX - (int) cPos.x) <= thW) && (abs(cPosCurY - (int) cPos.y) <= thH)))
          {
            rribcAdjustMotion(pu, &cPos, miNonAdjacent);
          }
#endif
          if (checkIsIBCCandidateValid(pu, miNonAdjacent))
          {
            // get Inter Dir
#if JVET_AE0169_BIPREDICTIVE_IBC
            mrgCtx.interDirNeighbours[cnt] = 1;
#else
            mrgCtx.interDirNeighbours[cnt] = miNonAdjacent.interDir;
#endif
            mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNonAdjacent.mv[0], miNonAdjacent.refIdx[0]);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
            mrgCtx.useAltHpelIf[cnt] = miNonAdjacent.useAltHpelIf;
#endif
#if JVET_AA0070_RRIBC
#if IBC_TM_MRG
#if JVET_AA0061_IBC_MBVD
            mrgCtx.rribcFlipTypes[cnt] =
              (miNonAdjacent.isIBCmot && !pu.tmMergeFlag && !pu.ibcMbvdMergeFlag) ? miNonAdjacent.rribcFlipType : 0;
#else
            mrgCtx.rribcFlipTypes[cnt] = (miAboveLeft.isIBCmot && !pu.tmMergeFlag) ? miAboveLeft.rribcFlipType : 0;
#endif
#else
#if JVET_AA0061_IBC_MBVD
            mrgCtx.rribcFlipTypes[cnt] = (miAboveLeft.isIBCmot && !pu.ibcMbvdMergeFlag) ? miAboveLeft.rribcFlipType : 0;
#else
            mrgCtx.rribcFlipTypes[cnt] = (miAboveLeft.isIBCmot) ? miAboveLeft.rribcFlipType : 0;
#endif
#endif
#endif
#if JVET_AC0112_IBC_LIC
#if JVET_AA0070_RRIBC
            mrgCtx.ibcLicFlags[cnt] =
              miNonAdjacent.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miNonAdjacent.useIbcLic : false;
#else
            mrgCtx.ibcLicFlags[cnt]    = miNonAdjacent.isIBCmot ? miNonAdjacent.useIbcLic : false;
#endif
#endif
#if JVET_AE0159_FIBC
#if JVET_AA0070_RRIBC
            mrgCtx.ibcFilterFlags[cnt] = miNonAdjacent.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miNonAdjacent.useIbcFilter : false;
#else
            mrgCtx.ibcFilterFlags[cnt] = miNonAdjacent.isIBCmot ? miNonAdjacent.useIbcFilter : false;
#endif
#endif
#if JVET_Z0084_IBC_TM
            if (!mrgCtx.xCheckSimilarIBCMotion(cnt, mvdSimilarityThresh))
#endif
            {
              if (mrgCandIdx == cnt)
              {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                mrgCtx.numAMVPMergeCand = cnt;
#endif
                return;
              }
              cnt++;
              // early termination
              if (cnt == maxNumMergeCand)
              {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                mrgCtx.numAMVPMergeCand = cnt;
#endif
                return;
              }
            }
          }
#if JVET_AE0169_BIPREDICTIVE_IBC
          if (miNonAdjacent.isIBCmot && miNonAdjacent.interDir == 3)
          {
            MotionInfo miNonAdjacentL1 = miNonAdjacent;
            miNonAdjacentL1.mv[0]      = miNonAdjacentL1.mv[1];
            miNonAdjacentL1.refIdx[1]  = -1;
#if JVET_AA0070_RRIBC
            if (pu.mergeFlag || ((abs(cPosCurX - (int) cPos.x) <= thW) && (abs(cPosCurY - (int) cPos.y) <= thH)))
            {
              rribcAdjustMotion(pu, &cPos, miNonAdjacentL1);
            }
#endif

            if (checkIsIBCCandidateValid(pu, miNonAdjacentL1))
            {
              // get Inter Dir
              mrgCtx.interDirNeighbours[cnt] = 1;
              mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNonAdjacentL1.mv[0], miNonAdjacentL1.refIdx[0]);
#if JVET_AA0070_RRIBC
#if IBC_TM_MRG
#if JVET_AA0061_IBC_MBVD
              mrgCtx.rribcFlipTypes[cnt] = (miNonAdjacentL1.isIBCmot && !pu.tmMergeFlag && !pu.ibcMbvdMergeFlag)
                                             ? miNonAdjacentL1.rribcFlipType
                                             : 0;
#else
              mrgCtx.rribcFlipTypes[cnt] =
                (miNonAdjacentL1.isIBCmot && !pu.tmMergeFlag) ? miNonAdjacentL1.rribcFlipType : 0;
#endif
#else
#if JVET_AA0061_IBC_MBVD
              mrgCtx.rribcFlipTypes[cnt] =
                (miNonAdjacentL1.isIBCmot && !pu.ibcMbvdMergeFlag) ? miNonAdjacentL1.rribcFlipType : 0;
#else
              mrgCtx.rribcFlipTypes[cnt] = (miNonAdjacentL1.isIBCmot) ? miNonAdjacentL1.rribcFlipType : 0;
#endif
#endif
#endif
#if JVET_AC0112_IBC_LIC
#if JVET_AA0070_RRIBC
              mrgCtx.ibcLicFlags[cnt] =
                miNonAdjacentL1.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miNonAdjacentL1.useIbcLic : false;
#else
              mrgCtx.ibcLicFlags[cnt] = miNonAdjacentL1.isIBCmot ? miNonAdjacentL1.useIbcLic : false;
#endif
#endif
#if JVET_AE0159_FIBC
#if JVET_AA0070_RRIBC
              mrgCtx.ibcFilterFlags[cnt] = miNonAdjacentL1.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miNonAdjacentL1.useIbcFilter : false;
#else
              mrgCtx.ibcFilterFlags[cnt] = miNonAdjacentL1.isIBCmot ? miNonAdjacentL1.useIbcFilter : false;
#endif
#endif
#if JVET_Z0084_IBC_TM
              if (!mrgCtx.xCheckSimilarIBCMotion(cnt, mvdSimilarityThresh))
#endif
              {
                if (mrgCandIdx == cnt)
                {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                  mrgCtx.numAMVPMergeCand = cnt;
#endif
                  return;
                }
                cnt++;

                // early termination
                if (cnt == maxNumMergeCand)
                {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                  mrgCtx.numAMVPMergeCand = cnt;
#endif
                  return;
                }
              }
            }
          }
#endif
        }
      }
    }
  }
#endif
  if (cnt != maxNumMergeCand)
  {
#if JVET_Z0075_IBC_HMVP_ENLARGE
    bool bFound = addIBCMergeHMVPCand(cs, mrgCtx, mrgCandIdx, maxNumMergeCand, cnt
#if JVET_Y0058_IBC_LIST_MODIFY
                                     , pu
#endif
#if JVET_Z0084_IBC_TM
                                     , mvdSimilarityThresh
#endif
                                     );
#else
    bool bFound = addMergeHMVPCand(cs, mrgCtx, mrgCandIdx, maxNumMergeCand, cnt
#if JVET_Z0084_IBC_TM
                                  , isGt4x4 && isAvailableA1, miLeft, isGt4x4 && isAvailableB1, miAbove
#else
                                  , isAvailableA1, miLeft, isAvailableB1, miAbove
#endif
                                  , true, isGt4x4
#if JVET_X0083_BM_AMVP_MERGE_MODE || JVET_Y0058_IBC_LIST_MODIFY || JVET_Z0118_GDR
                                  , pu
#endif
#if JVET_Z0084_IBC_TM
#if JVET_X0083_BM_AMVP_MERGE_MODE
#if !JVET_Y0128_NON_CTC
                                  , 0, 0
#endif
#endif
                                  , mvdSimilarityThresh
#endif
                                  );
#endif

    if (bFound)
    {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
      mrgCtx.numAMVPMergeCand = cnt;
#endif
      return;
    }

#if JVET_Z0075_IBC_HMVP_ENLARGE
    if (cnt == maxNumMergeCand)
    {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
      mrgCtx.numAMVPMergeCand = cnt;
#endif
      return;
    }
#endif
  }

#if JVET_AG0091_ARBVP
  if (cnt > 0 && cnt < maxNumMergeCand)
  {
    MotionInfo miCascaded;
    int        offsetX = 0;
    int        offsetY = 0;
    Mv         cMv;
    Position posCand[5] = { pu.Y().center(), pu.Y().topLeft(), pu.Y().topRight(), pu.Y().bottomLeft(), pu.Y().bottomRight() };
#if JVET_AA0070_RRIBC
    int cPosCurX = pu.lx() + (pu.lwidth() >> 1);
    int cPosCurY = pu.ly() + (pu.lheight() >> 1);
    int thW = (pu.lwidth() >> 1) * 3;
    int thH = (pu.lheight() >> 1) * 3;
#endif
    int checkNum = cnt;
    for (int mergeIndex = 0; mergeIndex < checkNum; mergeIndex++)
    {
      cMv = mrgCtx.mvFieldNeighbours[mergeIndex << 1].mv;
      cMv.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
      offsetX = cMv.getHor();
      offsetY = cMv.getVer();
      for (int n = 0; n < 5; n++)
      {
        const PredictionUnit* puCascaded = cs.getPURestricted(posCand[n].offset(offsetX, offsetY), pu, pu.chType);
        bool                  isAvailableCascaded =
          puCascaded && pu.cu != puCascaded->cu && (CU::isIBC(*puCascaded->cu) || puCascaded->cu->tmpFlag);
        if (isGt4x4 && isAvailableCascaded)
        {
          miCascaded = puCascaded->getMotionInfo(posCand[n].offset(offsetX, offsetY));
#if JVET_AA0070_RRIBC
          Position cPos(puCascaded->lx() + (puCascaded->lwidth() >> 1),
            puCascaded->ly() + (puCascaded->lheight() >> 1));
          if (pu.mergeFlag || ((abs(cPosCurX - (int)cPos.x) <= thW) && (abs(cPosCurY - (int)cPos.y) <= thH)))
          {
            rribcAdjustMotion(pu, &cPos, miCascaded);
          }
#endif
          MotionInfo miCascadedL0 = miCascaded;
          miCascadedL0.mv[0] += mrgCtx.mvFieldNeighbours[mergeIndex << 1].mv;
          if ((miCascaded.refIdx[0] != -1) && checkIsIBCCandidateValid(pu, miCascadedL0))
          {
            // get Inter Dir
#if JVET_AE0169_BIPREDICTIVE_IBC
            mrgCtx.interDirNeighbours[cnt] = 1;
#else
            mrgCtx.interDirNeighbours[cnt] = miCascaded.interDir;
#endif
            mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miCascadedL0.mv[0], miCascaded.refIdx[0]);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
            mrgCtx.useAltHpelIf[cnt] = pu.mergeFlag ? false : (pu.cu->imv == IMV_HPEL);
#endif
#if JVET_AA0070_RRIBC
            mrgCtx.rribcFlipTypes[cnt] = 0;
#endif
#if JVET_AC0112_IBC_LIC
#if JVET_AA0070_RRIBC
            mrgCtx.ibcLicFlags[cnt] =
              miCascaded.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miCascaded.useIbcLic : false;
#else
            mrgCtx.ibcLicFlags[cnt] = miCascaded.isIBCmot ? miCascaded.useIbcLic : false;
#endif
            mrgCtx.ibcLicFlags[cnt] |= mrgCtx.ibcLicFlags[mergeIndex];
#endif
#if JVET_AE0159_FIBC
#if JVET_AA0070_RRIBC
            mrgCtx.ibcFilterFlags[cnt] = miCascaded.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miCascaded.useIbcFilter : false;
#else
            mrgCtx.ibcFilterFlags[cnt] = miCascaded.isIBCmot ? miCascaded.useIbcFilter : false;
#endif
            mrgCtx.ibcFilterFlags[cnt] |= mrgCtx.ibcFilterFlags[mergeIndex];
#endif
#if JVET_Z0084_IBC_TM
            if (!mrgCtx.xCheckSimilarIBCMotion(cnt, mvdSimilarityThresh))
#endif
            {
              if (mrgCandIdx == cnt)
              {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                mrgCtx.numAMVPMergeCand = cnt;
#endif
                return;
              }
              cnt++;
              // early termination
              if (cnt == maxNumMergeCand)
              {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                mrgCtx.numAMVPMergeCand = cnt;
#endif
                return;
              }
            }
          }
#if JVET_AE0169_BIPREDICTIVE_IBC
          if (miCascaded.isIBCmot && miCascaded.interDir == 3)
          {
            MotionInfo miCascadedL1 = miCascaded;
            miCascadedL1.mv[0] = miCascadedL1.mv[1];
            miCascadedL1.refIdx[1] = -1;
#if JVET_AA0070_RRIBC
            if (pu.mergeFlag || ((abs(cPosCurX - (int)cPos.x) <= thW) && (abs(cPosCurY - (int)cPos.y) <= thH)))
            {
              rribcAdjustMotion(pu, &cPos, miCascadedL1);
            }
#endif
            miCascadedL1.mv[0] += mrgCtx.mvFieldNeighbours[mergeIndex << 1].mv;

            if (checkIsIBCCandidateValid(pu, miCascadedL1))
            {
              // get Inter Dir
              mrgCtx.interDirNeighbours[cnt] = 1;
              mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miCascadedL1.mv[0], miCascadedL1.refIdx[0]);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
              mrgCtx.useAltHpelIf[cnt] = pu.mergeFlag ? false : (pu.cu->imv == IMV_HPEL);
#endif
#if JVET_AA0070_RRIBC
              mrgCtx.rribcFlipTypes[cnt] = 0;
#endif
#if JVET_AC0112_IBC_LIC
#if JVET_AA0070_RRIBC
              mrgCtx.ibcLicFlags[cnt] =
                miCascadedL1.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miCascadedL1.useIbcLic : false;
#else
              mrgCtx.ibcLicFlags[cnt] = miCascadedL1.isIBCmot ? miCascadedL1.useIbcLic : false;
#endif
              mrgCtx.ibcLicFlags[cnt] |= mrgCtx.ibcLicFlags[mergeIndex];
#endif
#if JVET_AE0159_FIBC
#if JVET_AA0070_RRIBC
              mrgCtx.ibcFilterFlags[cnt] = miCascadedL1.isIBCmot && mrgCtx.rribcFlipTypes[cnt] == 0 ? miCascadedL1.useIbcFilter : false;
#else
              mrgCtx.ibcFilterFlags[cnt] = miCascadedL1.isIBCmot ? miCascadedL1.useIbcFilter : false;
#endif
              mrgCtx.ibcFilterFlags[cnt] |= mrgCtx.ibcFilterFlags[mergeIndex];
#endif

#if JVET_Z0084_IBC_TM
              if (!mrgCtx.xCheckSimilarIBCMotion(cnt, mvdSimilarityThresh))
#endif
              {
                if (mrgCandIdx == cnt)
                {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                  mrgCtx.numAMVPMergeCand = cnt;
#endif
                  return;
                }
                cnt++;

                // early termination
                if (cnt == maxNumMergeCand)
                {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                  mrgCtx.numAMVPMergeCand = cnt;
#endif
                  return;
                }
              }
            }
          }
#endif
        } // if (isGt4x4 && isAvailableCascaded)
      }
    } // mergeIndex
  }
#endif

#if JVET_Y0058_IBC_LIST_MODIFY
  // pairwise-average candidates
  if (cnt>1 && cnt <maxNumMergeCand)
  {
    mrgCtx.mvFieldNeighbours[cnt * 2    ].setMvField(Mv(0, 0), NOT_VALID);
    mrgCtx.mvFieldNeighbours[cnt * 2 + 1].setMvField(Mv(0, 0), NOT_VALID);

    const Mv& MvI = mrgCtx.mvFieldNeighbours[0 * 2].mv;
    const Mv& MvJ = mrgCtx.mvFieldNeighbours[1 * 2].mv;
    // average two MVs
    Mv avgMv = MvI;

    avgMv += MvJ;
    roundAffineMv(avgMv.hor, avgMv.ver, 1);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    if(!pu.cs->sps->getIBCFracFlag())
    {
#endif
    avgMv.roundToPrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    }
#endif
    MotionInfo miAvg;
    miAvg.mv[0] = avgMv;
    miAvg.refIdx[0] = MAX_NUM_REF;
    if (checkIsIBCCandidateValid(pu, miAvg))
    {
      mrgCtx.mvFieldNeighbours[cnt * 2].setMvField(avgMv, MAX_NUM_REF);
      mrgCtx.interDirNeighbours[cnt] = 1;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AA0070_RRIBC
      mrgCtx.rribcFlipTypes[cnt] = 0;
#endif
      mrgCtx.useAltHpelIf[cnt] = pu.mergeFlag ? false : (pu.cu->imv == IMV_HPEL);
#endif
#if JVET_Z0084_IBC_TM
      if( !mrgCtx.xCheckSimilarIBCMotion(cnt, mvdSimilarityThresh) )
#else
      if (!mrgCtx.xCheckSimilarMotion(cnt))
#endif
      {
        if (mrgCandIdx == cnt)
        {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
          mrgCtx.numAMVPMergeCand = cnt;
#endif
          return;
        }
        cnt++;

        // early termination
        if (cnt == maxNumMergeCand)
        {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
          mrgCtx.numAMVPMergeCand = cnt;
#endif
          return;
        }
      }
    }
  }
#endif

#if JVET_Z0160_IBC_ZERO_PADDING
  int  validIdx = 0;
  Mv   validBvCand[IBC_MRG_MAX_NUM_CANDS];
  bool padList = true;

  if (cnt < maxNumMergeCand)
  {
    const int ctbSize     = pu.cs->sps->getCTUSize();
    const int log2CtbSize = floorLog2(ctbSize);
    const int cbX         = pu.lx() & (ctbSize - 1);
    const int cbY         = pu.ly() & (ctbSize - 1);
    const int cbWidth     = pu.lwidth();
    const int cbHeight    = pu.lheight();
#if !JVET_Z0153_IBC_EXT_REF
    const int leftEdge = -ctbSize
                         * (std::min(pu.Y().x >> log2CtbSize, (1 << ((MAX_CU_DEPTH - log2CtbSize) << 1))
                                                                - ((log2CtbSize < MAX_CU_DEPTH) ? 1 : 0)));
    int deltaX = cbX - leftEdge - cbWidth;
    int deltaY = cbY - cbHeight;
#else
    const int deltaX = ctbSize * (pu.lx() >> log2CtbSize) + cbX - cbWidth;
    const int deltaY = ((pu.ly() >> log2CtbSize) == 0)
                         ? (cbY - cbHeight)
                         : ((((pu.ly() >> log2CtbSize) == 1) ? 1 : 2) * ctbSize + cbY - cbHeight);
#endif

    if (deltaX >= 0 && deltaY >= 0)
    {
      validBvCand[0] = Mv(-cbWidth, -cbHeight);
      validBvCand[1] = Mv(-cbWidth, 0);
      validBvCand[2] = Mv(0, -cbHeight);
      validBvCand[3] = Mv(-cbWidth - (deltaX >> 1), -(cbHeight >> 1));
      validBvCand[4] = Mv(-(cbWidth >> 1), -cbHeight - (deltaY >> 1));
      validBvCand[5] = Mv(-cbWidth - (deltaX >> 1), -cbHeight - (deltaY >> 1));
    }
    else if (deltaY >= 0)
    {
      validBvCand[0] = Mv(-cbX, -cbHeight);
      validBvCand[1] = Mv(0, -cbHeight);
      validBvCand[2] = Mv(-(cbX >> 1), -cbHeight - (deltaY >> 1));
      validBvCand[3] = Mv(-cbX, -cbY);
      validBvCand[4] = Mv(0, -cbY);
      validBvCand[5] = Mv(cbWidth, -cbHeight - (deltaY >> 1));
    }
    else if (deltaX >= 0)
    {
      validBvCand[0] = Mv(-cbWidth, -cbY);
      validBvCand[1] = Mv(-cbWidth, 0);
      validBvCand[2] = Mv(-cbWidth - (deltaX >> 1), -(cbY >> 1));
      validBvCand[3] = Mv(-cbX, -cbY);
      validBvCand[4] = Mv(-cbX, 0);
      validBvCand[5] = Mv(-cbWidth - (deltaX >> 1), cbHeight);
    }
    else
    {
      padList = false;
    }
  }

  while (cnt < maxNumMergeCand && padList && validIdx < IBC_MRG_MAX_NUM_CANDS)
  {
#if !JVET_Z0084_IBC_TM
    bool duplicateBVP = false;
#endif
    Mv mvp = validBvCand[validIdx];
    mvp.changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);

#if !JVET_Z0084_IBC_TM
    for (int i = 0; i < cnt; i++)
    {
      if (mvp == mrgCtx.mvFieldNeighbours[i * 2].mv)
      {
        duplicateBVP = true;
        break;
      }
    }
#endif
    MotionInfo miCand;
    miCand.mv[0] = mvp;
    mrgCtx.mvFieldNeighbours[cnt * 2].setMvField(mvp, MAX_NUM_REF);
    mrgCtx.interDirNeighbours[cnt] = 1;
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    mrgCtx.useAltHpelIf[cnt] = pu.mergeFlag ? false : (pu.cu->imv == IMV_HPEL);
#endif
#if JVET_AC0112_IBC_LIC
    mrgCtx.ibcLicFlags[cnt] = false;
#endif
#if JVET_AE0159_FIBC
    mrgCtx.ibcFilterFlags[cnt] = false;
#endif
#if JVET_AA0070_RRIBC
    mrgCtx.rribcFlipTypes[cnt] = 0;
#endif

#if JVET_Z0084_IBC_TM
    if (!mrgCtx.xCheckSimilarIBCMotion(cnt, mvdSimilarityThresh) && checkIsIBCCandidateValid(pu, miCand))
#else
    if (!duplicateBVP && checkIsIBCCandidateValid(pu, miCand))
#endif
    {
      if (mrgCandIdx == cnt)
      {
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
        mrgCtx.numAMVPMergeCand = cnt;
#endif
        return;
      }
      cnt++;
    }
    validIdx++;
  }

#else
  while (cnt < maxNumMergeCand)
  {
    mrgCtx.interDirNeighbours[cnt] = 1;
    mrgCtx.mvFieldNeighbours[cnt * 2].setMvField(Mv(0, 0), MAX_NUM_REF);
#if JVET_AC0112_IBC_LIC
    mrgCtx.ibcLicFlags[cnt] = false;
#endif
#if JVET_AE0159_FIBC
    mrgCtx.ibcFilterFlags[cnt] = false;
#endif
#if JVET_AA0070_RRIBC
    mrgCtx.rribcFlipTypes[cnt] = 0;
#endif
    if (mrgCandIdx == cnt)
    {
      return;
    }
    cnt++;
  }
#endif

#if JVET_Z0084_IBC_TM && JVET_Z0075_IBC_HMVP_ENLARGE
  mrgCtx.numValidMergeCand = std::min((int)pu.cs->sps->getMaxNumIBCMergeCand(), cnt);
#elif !JVET_Z0075_IBC_HMVP_ENLARGE
  mrgCtx.numValidMergeCand = cnt;
#endif

#if (JVET_Z0160_IBC_ZERO_PADDING || JVET_Z0084_IBC_TM) && JVET_Z0075_IBC_HMVP_ENLARGE
  // Add a zero motion to be able to stop reordering
  if (cnt < maxNumMergeCand)
  {
    mrgCtx.interDirNeighbours[cnt] = 1;
    mrgCtx.mvFieldNeighbours[cnt * 2].setMvField(Mv(0, 0), MAX_NUM_REF);
  }
#endif
#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  mrgCtx.numAMVPMergeCand = cnt;
#endif
}

#if  JVET_Y0058_IBC_LIST_MODIFY || JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
bool PU::checkIsIBCCandidateValid(const PredictionUnit& pu
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                ,       MotionInfo miNeighbor
#else
                                , const MotionInfo miNeighbor
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                                ,       int filterIdx
#endif
                                ,       bool isRefTemplate
                                ,       bool isRefAbove
)
{
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  int roiWidth  = (isRefTemplate && !isRefAbove) ? AML_MERGE_TEMPLATE_SIZE : pu.lwidth();
  int roiHeight = (isRefTemplate &&  isRefAbove) ? AML_MERGE_TEMPLATE_SIZE : pu.lheight();
  uint32_t validType = checkValidBv(pu, COMPONENT_Y, roiWidth, roiHeight, miNeighbor.mv[REF_PIC_LIST_0], true, filterIdx);
  return validType != IBC_BV_INVALID;
#else
  Mv bv = miNeighbor.mv[REF_PIC_LIST_0];
  bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT); // used for only integer resolution
  const int cuPelX = pu.Y().x;
  const int cuPelY = pu.Y().y;
  int roiWidth  = (isRefTemplate && !isRefAbove) ? AML_MERGE_TEMPLATE_SIZE : pu.lwidth();
  int roiHeight = (isRefTemplate && isRefAbove) ? AML_MERGE_TEMPLATE_SIZE : pu.lheight();
  const int picWidth = pu.cs->slice->getPPS()->getPicWidthInLumaSamples();
  const int picHeight = pu.cs->slice->getPPS()->getPicHeightInLumaSamples();
  const unsigned int  lcuWidth = pu.cs->slice->getSPS()->getMaxCUWidth();
  int xPred = bv.getHor();
  int yPred = bv.getVer();

  if (searchBv(pu, cuPelX, cuPelY, roiWidth, roiHeight, picWidth, picHeight, xPred, yPred, lcuWidth)) // not valid bv derived
  {
    return true;
  }
  else
  {
    return false;
  }
#endif
}
#endif

#if JVET_AE0169_BIPREDICTIVE_IBC
bool PU::checkIsIBCCandidateValidBi(const PredictionUnit &pu, MotionInfo miNeighbor)
{
  bool ret = checkIsIBCCandidateValid(pu, miNeighbor);
  if (!ret || miNeighbor.interDir == 1)
  {
    return ret;
  }
  Mv mvbak = miNeighbor.mv[REF_PIC_LIST_0];
  miNeighbor.mv[REF_PIC_LIST_0] = miNeighbor.mv[REF_PIC_LIST_1];
  ret = checkIsIBCCandidateValid(pu, miNeighbor);
  miNeighbor.mv[REF_PIC_LIST_0] = mvbak;
  return ret;
}
#endif

#if JVET_Y0058_IBC_LIST_MODIFY || JVET_Z0084_IBC_TM
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
uint32_t PU::checkValidBvPU(const PredictionUnit& pu, ComponentID compID, Mv mv, bool ignoreFracMv, int filterIdx)
{
  return PU::checkValidBv(pu, compID, (int)pu.blocks[compID].width, (int)pu.blocks[compID].height, mv, ignoreFracMv, filterIdx);
}

uint32_t PU::checkValidBv(const PredictionUnit& pu, ComponentID compID, int compWidth, int compHeight, Mv mv, bool ignoreFracMv, int filterIdx
                        , bool isFinalMC
                        , bool checkAllRefValid
)
{
  const int picWidth   = pu.cs->slice->getPPS()->getPicWidthInLumaSamples();
  const int picHeight  = pu.cs->slice->getPPS()->getPicHeightInLumaSamples();
  const int lcuWidth   = pu.cs->slice->getSPS()->getMaxCUWidth();

  const int szShiftHor = ::getComponentScaleX(compID, pu.chromaFormat);
  const int szShiftVer = ::getComponentScaleY(compID, pu.chromaFormat);
  const int lumaPosX   = pu.blocks[compID].x << szShiftHor;
  const int lumaPosY   = pu.blocks[compID].y << szShiftVer;
  const int lumaWidth  = compWidth  << szShiftHor;
  const int lumaHeight = compHeight << szShiftVer;

  const int bvShiftHor = MV_FRACTIONAL_BITS_INTERNAL + szShiftHor;
  const int bvShiftVer = MV_FRACTIONAL_BITS_INTERNAL + szShiftVer;
  const int xFrac      = pu.cs->sps->getIBCFracFlag() ? (mv.hor & ((1 << bvShiftHor) - 1)) : 0;
  const int yFrac      = pu.cs->sps->getIBCFracFlag() ? (mv.ver & ((1 << bvShiftVer) - 1)) : 0;

  if ((!pu.cs->sps->getIBCFracFlag() || ignoreFracMv) || (xFrac == 0 && yFrac == 0))
  {
    int xLumaBv = (mv.hor >> bvShiftHor) << szShiftHor;
    int yLumaBv = (mv.ver >> bvShiftVer) << szShiftVer;

    return !PU::searchBv(pu, lumaPosX, lumaPosY, lumaWidth, lumaHeight, picWidth, picHeight, xLumaBv, yLumaBv, lcuWidth, 0, 0, compID)
         ? IBC_BV_INVALID
         : (xFrac != 0 || yFrac != 0 ? IBC_INT_BV_VALID : IBC_BV_VALID);
  }

  int xLumaBv = (mv.hor >> bvShiftHor) << szShiftHor;
  int yLumaBv = (mv.ver >> bvShiftVer) << szShiftVer;
  int xFilterTap = 0;
  int yFilterTap = 0;

  if (compID == COMPONENT_Y)
  {
    int filterTap = filterIdx == 1 ? 2 : NTAPS_LUMA_IBC;
    xFilterTap    = xFrac == 0 ? 0 : filterTap;
    yFilterTap    = yFrac == 0 ? 0 : filterTap;
  }
  else
  {
    int filterTap = filterIdx == 1 ? 2 : NTAPS_CHROMA;
    xFilterTap    = (xFrac == 0 ? 0 : filterTap) << szShiftHor;
    yFilterTap    = (yFrac == 0 ? 0 : filterTap) << szShiftVer;
  }

  if (PU::searchBv(pu, lumaPosX, lumaPosY, lumaWidth, lumaHeight, picWidth, picHeight, xLumaBv, yLumaBv, lcuWidth, xFilterTap, yFilterTap, compID))
  {
    return IBC_BV_VALID;
  }
  else
  {
    if (isFinalMC)
    {
      return IBC_INT_BV_VALID;
    }
    if (checkAllRefValid)
    {
      return IBC_BV_INVALID;
    }
    return PU::checkValidBv(pu, compID, compWidth, compHeight, mv, true);
  }
}
#endif

#if JVET_AE0159_FIBC
bool PU::checkIsIBCFilterCandidateValid(const PredictionUnit& pu, const MotionInfo miNeighbor, int filterIdx, bool isRefTemplate, bool isRefAbove)
{
  int roiWidth  = (isRefTemplate && !isRefAbove) ? FIBC_TEMPLATE_SIZE : pu.lwidth();
  int roiHeight = (isRefTemplate &&  isRefAbove) ? FIBC_TEMPLATE_SIZE : pu.lheight();
  uint32_t validType = checkValidBv(pu, COMPONENT_Y, roiWidth, roiHeight, miNeighbor.mv[REF_PIC_LIST_0], true, filterIdx);
  return validType != IBC_BV_INVALID;
}
#endif

bool PU::searchBv(const PredictionUnit& pu, int xPos, int yPos, int width, int height, int picWidth, int picHeight, int xBv, int yBv, int ctuSize
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
                , int xFilterTap, int yFilterTap, ComponentID compID
#endif
)
{
  const int ctuSizeLog2 = floorLog2(ctuSize);

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AC0071_DBV
  const bool isDBV     = (pu.cs->slice->getSliceType() == I_SLICE && CS::isDualITree(*pu.cs) && pu.cu->slice->getSPS()->getUseIntraDBV())
                      && compID != COMPONENT_Y && pu.intraDir[1] == DBV_CHROMA_IDX;
#endif
  const int szShiftHor = ::getComponentScaleX(compID, pu.chromaFormat);
  const int szShiftVer = ::getComponentScaleY(compID, pu.chromaFormat);
  int refRightX  = xPos + xBv + width  - 1 - szShiftHor;
  int refLeftX   = xPos + xBv;
  int refBottomY = yPos + yBv + height - 1 - szShiftVer;
  int refTopY    = yPos + yBv;

  if (pu.cs->sps->getIBCFracFlag())
  {
    CHECK((xFilterTap & 1) != 0 || (yFilterTap & 1) != 0, "The number of interpolation filter taps for IBC has to be an even number");
    if (xFilterTap > 0)
    {
      refLeftX  -= ((xFilterTap >> 1) - 1 - szShiftHor);
      refRightX +=  (xFilterTap >> 1);
    }
    if (yFilterTap > 0)
    {
      refTopY    -= ((yFilterTap >> 1) - 1 - szShiftVer);
      refBottomY +=  (yFilterTap >> 1);
    }
  }
#else
  int refRightX = xPos + xBv + width - 1;
  int refLeftX  = xPos + xBv;

  int refBottomY = yPos + yBv + height - 1;
  int refTopY    = yPos + yBv;
#endif

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (xPos < 0 || yPos < 0)
  {
    return false;
  }
#endif

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (refLeftX < 0)
#else
  if ((xPos + xBv) < 0)
#endif
  {
    return false;
  }
  if (refRightX >= picWidth)
  {
    return false;
  }

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (refTopY < 0)
#else
  if ((yPos + yBv) < 0)
#endif
  {
    return false;
  }
  if (refBottomY >= picHeight)
  {
    return false;
  }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (refRightX >= xPos && refBottomY >= yPos)
#else
  if ((xBv + width) > 0 && (yBv + height) > 0)
#endif
  {
    return false;
  }

#if !JVET_Z0153_IBC_EXT_REF
  // Don't search the above CTU row
  if (refTopY >> ctuSizeLog2 < yPos >> ctuSizeLog2
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && JVET_AC0071_DBV
      && !isDBV
#endif
    )
  {
    return false;
  }
#endif

  // Don't search the below CTU row
  if (refBottomY >> ctuSizeLog2 > yPos >> ctuSizeLog2)
  {
    return false;
  }

#if JVET_Z0084_IBC_TM || JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  unsigned curTileIdx = pu.cs->pps->getTileIdx(Position(xPos, yPos));
#else
  unsigned curTileIdx = pu.cs->pps->getTileIdx(pu.lumaPos());
#endif
  unsigned refTileIdx = pu.cs->pps->getTileIdx(Position(refLeftX, refTopY));
  if (curTileIdx != refTileIdx)
  {
    return false;
  }
  refTileIdx = pu.cs->pps->getTileIdx(Position(refLeftX, refBottomY));
  if (curTileIdx != refTileIdx)
  {
    return false;
  }
  refTileIdx = pu.cs->pps->getTileIdx(Position(refRightX, refTopY));
  if (curTileIdx != refTileIdx)
  {
    return false;
  }
  refTileIdx = pu.cs->pps->getTileIdx(Position(refRightX, refBottomY));
  if (curTileIdx != refTileIdx)
  {
    return false;
  }

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && JVET_AC0071_DBV
  if(compID == COMPONENT_Y || !isDBV)
  {
#endif
#if JVET_Z0153_IBC_EXT_REF
#if JVET_AA0106_IBCBUF_CTU256
  if(256 == ctuSize)
  {
    if ((refTopY >> ctuSizeLog2) + 1 < (yPos >> ctuSizeLog2))
    {
      return false;
    }
    if (((refTopY >> ctuSizeLog2) == (yPos >> ctuSizeLog2)) && ((refRightX >> ctuSizeLog2) > (xPos >> ctuSizeLog2)))
    {
      return false;
    }
    if (((refTopY >> ctuSizeLog2) + 1 == (yPos >> ctuSizeLog2)) && ((refLeftX >> ctuSizeLog2) + 1 < (xPos >> ctuSizeLog2)))
    {
      return false;
    }
  }
  else
  {
    if ((refTopY >> ctuSizeLog2) + 2 < (yPos >> ctuSizeLog2))
    {
      return false;
    }
    if (((refTopY >> ctuSizeLog2) == (yPos >> ctuSizeLog2)) && ((refRightX >> ctuSizeLog2) > (xPos >> ctuSizeLog2)))
    {
      return false;
    }
    if (((refTopY >> ctuSizeLog2) + 2 == (yPos >> ctuSizeLog2)) && ((refLeftX >> ctuSizeLog2) + 2 < (xPos >> ctuSizeLog2)))
    {
      return false;
    }
  }
#else
  if ((refTopY >> ctuSizeLog2) + 2 < (yPos >> ctuSizeLog2))
  {
    return false;
  }
  if (((refTopY >> ctuSizeLog2) == (yPos >> ctuSizeLog2)) && ((refRightX >> ctuSizeLog2) > (xPos >> ctuSizeLog2)))
  {
    return false;
  }
  if (((refTopY >> ctuSizeLog2) + 2 == (yPos >> ctuSizeLog2)) && ((refLeftX >> ctuSizeLog2) + 2 < (xPos >> ctuSizeLog2)))
  {
    return false;
  }
#endif
#else
  // in the same CTU line
#if CTU_256
  int numLeftCTUs = ( 1 << ( ( MAX_CU_DEPTH - ctuSizeLog2 ) << 1 ) ) - ( ( ctuSizeLog2 < MAX_CU_DEPTH ) ? 1 : 0 );
#else
  int numLeftCTUs = (1 << ((7 - ctuSizeLog2) << 1)) - ((ctuSizeLog2 < 7) ? 1 : 0);
#endif
  if ((refRightX >> ctuSizeLog2 <= xPos >> ctuSizeLog2) && (refLeftX >> ctuSizeLog2 >= (xPos >> ctuSizeLog2) - numLeftCTUs))
  {

    // in the same CTU, or left CTU
    // if part of ref block is in the left CTU, some area can be referred from the not-yet updated local CTU buffer
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && REMOVE_IBC_VPDU
    if(false)
    {
#endif
#if CTU_256
    if( ( ( refLeftX >> ctuSizeLog2 ) == ( ( xPos >> ctuSizeLog2 ) - 1 ) ) && ( ctuSizeLog2 == MAX_CU_DEPTH ) )
#else
    if (((refLeftX >> ctuSizeLog2) == ((xPos >> ctuSizeLog2) - 1)) && (ctuSizeLog2 == 7))
#endif
    {
      // ref block's collocated block in current CTU
#if JVET_Z0084_IBC_TM || JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      const Position refPosCol(refLeftX + ctuSize,  refTopY);
#else
      const Position refPosCol = pu.Y().topLeft().offset(xBv + ctuSize, yBv);
#endif
      int offset64x = (refPosCol.x >> (ctuSizeLog2 - 1)) << (ctuSizeLog2 - 1);
      int offset64y = (refPosCol.y >> (ctuSizeLog2 - 1)) << (ctuSizeLog2 - 1);
      const Position refPosCol64x64 = {offset64x, offset64y};
      if (pu.cs->isDecomp(refPosCol64x64, toChannelType(COMPONENT_Y)))
      {
        return false;
      }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      if (!pu.cs->pcv->isEncoder)
      {
#if JVET_Z0118_GDR
        PelBuf buf = m_ibcBuffer0.Y();
#else
        PelBuf buf = m_ibcBuffer.Y();
#endif
        CHECK(buf.at(refPosCol64x64.x & (m_IBCBufferWidth - 1), refPosCol64x64.y & (ctuSize - 1)) > -1, "InterPrediction::checkExtBv() refPosCol64x64 check error");
      }
#endif
      if (refPosCol64x64 == pu.Y().topLeft())
      {
        return false;
      }
    }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && REMOVE_IBC_VPDU
    }
#endif
  }
  else
  {
    return false;
  }
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS && JVET_AC0071_DBV
  }
#endif

  // in the same CTU, or valid area from left CTU. Check if the reference block is already coded
#if JVET_Z0084_IBC_TM || JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (compID != COMPONENT_Y)
  {
    refLeftX   >>= szShiftHor;
    refRightX  >>= szShiftHor;
    refTopY    >>= szShiftVer;
    refBottomY >>= szShiftVer;
  }
#endif
  const Position refPosLT(refLeftX,  refTopY);
  const Position refPosBR(refRightX, refBottomY);
#else
  const Position refPosLT = pu.Y().topLeft().offset(xBv, yBv);
  const Position refPosBR = pu.Y().bottomRight().offset(xBv, yBv);
#endif

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  const ChannelType      chType = toChannelType(compID);
#else
  const ChannelType      chType = toChannelType(COMPONENT_Y);
#endif
  if (!pu.cs->isDecomp(refPosBR, chType))
  {
    return false;
  }
  if (!pu.cs->isDecomp(refPosLT, chType))
  {
    return false;
  }
  return true;
}
#endif

#if JVET_AA0061_IBC_MBVD
void PU::getIbcMbvdMergeCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx, int numValidBv)
{
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
  if (pu.cu->slice->getSPS()->getUseIbcMbvdAdSearch())
  {
    getIbcAdaptiveMbvdMergeCandidates(pu, mrgCtx, numValidBv);
    return;
  }
#endif
  int refIdxList0;
  int k;
  int currBaseNum = 0;

  for( k = 0; k < numValidBv; k++ )
  {
    refIdxList0 = mrgCtx.mvFieldNeighbours[( k << 1 )].refIdx;

    if( refIdxList0 >= 0 )
    {
      mrgCtx.ibcMbvdBaseBv[currBaseNum][0] = mrgCtx.mvFieldNeighbours[( k << 1 )];
      mrgCtx.ibcMbvdBaseBv[currBaseNum][1] = MvField( Mv( 0, 0 ), -1 );
    }

    currBaseNum++;

    if( currBaseNum == IBC_MBVD_BASE_NUM )
    {
      break;
    }
  }
}
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
void PU::getIbcAdaptiveMbvdMergeCandidates(const PredictionUnit& pu, MergeCtx& mrgCtx, int numValidBv)
{
  int currBaseNum = 0;

  for (int k = 0; k < mrgCtx.numValidMergeCand; k++)
  {
    int refIdxList0 = mrgCtx.mvFieldNeighbours[(k << 1)].refIdx;
    if (refIdxList0 >= 0)
    {
      mrgCtx.ibcMbvdBaseBv[currBaseNum][0] = mrgCtx.mvFieldNeighbours[(k << 1)];
      mrgCtx.ibcMbvdBaseBv[currBaseNum][0].mv.roundToPrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
#if JVET_AE0169_BIPREDICTIVE_IBC // store original unrounded base for use as merge candidates
      mrgCtx.ibcMbvdBaseBvFrac[currBaseNum][0] = mrgCtx.mvFieldNeighbours[(k << 1)];
      mrgCtx.ibcMbvdBaseBvFrac[currBaseNum][1] = MvField(Mv(0, 0), -1);
#endif
      mrgCtx.ibcMbvdBaseBv[currBaseNum][1] = MvField(Mv(0, 0), -1);
      bool findSimilarBaseBv = false;
      for (int baseIdx = 0; baseIdx < currBaseNum; baseIdx++)
      {
        if (((mrgCtx.ibcMbvdBaseBv[baseIdx][0].mv.hor == mrgCtx.ibcMbvdBaseBv[currBaseNum][0].mv.hor)
          && (std::abs(mrgCtx.ibcMbvdBaseBv[baseIdx][0].mv.ver - mrgCtx.ibcMbvdBaseBv[currBaseNum][0].mv.ver) < IBC_MBVD_BASE_DIFF_TH))
          || ((mrgCtx.ibcMbvdBaseBv[baseIdx][0].mv.ver == mrgCtx.ibcMbvdBaseBv[currBaseNum][0].mv.ver)
            && (std::abs(mrgCtx.ibcMbvdBaseBv[baseIdx][0].mv.hor - mrgCtx.ibcMbvdBaseBv[currBaseNum][0].mv.hor) < IBC_MBVD_BASE_DIFF_TH)))
        {
          findSimilarBaseBv = true;
          break;
        }
      }
      currBaseNum += (findSimilarBaseBv == false);
    }
    if (currBaseNum == numValidBv)
    {
      mrgCtx.numValidMergeCand = currBaseNum;
      return;
    }
  }
  mrgCtx.numValidMergeCand = currBaseNum;
}
#endif

#if JVET_AE0169_BIPREDICTIVE_IBC
int32_t PU::getIbcMbvdEstBits(const PredictionUnit &pu, int mmvdMergeCand, int mmvdMergeCand1)
#else
int32_t PU::getIbcMbvdEstBits(const PredictionUnit &pu,unsigned int mmvdMergeCand)
#endif
{
#if JVET_AE0169_IBC_MBVD_LIST_DERIVATION
  const int mbvdsPerBase = pu.cu->slice->getSPS()->getUseIbcMbvdAdSearch() ? IBC_MBVD_SIZE_ENC : IBC_MBVD_MAX_REFINE_NUM;
#else
  const int mbvdsPerBase = IBC_MBVD_MAX_REFINE_NUM;
#endif
#if JVET_AE0169_BIPREDICTIVE_IBC
  int numCandminus1 = std::min(int(pu.cs->sps->getMaxNumIBCMergeCand()), IBC_MBVD_BASE_NUM) - 1;
  bool bimbvd = mmvdMergeCand1 >= IBC_MRG_MAX_NUM_CANDS;
  if (bimbvd)
  {
    mmvdMergeCand1 -= IBC_MRG_MAX_NUM_CANDS;
    int baseIdx0 = mmvdMergeCand / mbvdsPerBase;
    int baseIdx1 = mmvdMergeCand1 / mbvdsPerBase;
    int baseBits = (numCandminus1 == 0 ? 0 : baseIdx1 + (baseIdx1 == numCandminus1 ? 1: 2));
    baseBits -= (baseIdx0 == numCandminus1) ? 1 : 0;
    int ricePar = 1;
    unsigned int mvpIdx0 = mmvdMergeCand - baseIdx0 * mbvdsPerBase;
    unsigned int mmvdUnaryBits0 = mvpIdx0 >> ricePar;
    const int ibcMbvdSizeEnc0 = IBC_MBVD_SIZE_ENC-((baseIdx0 == baseIdx1) ? 1 : 0);
    int numCandStepMinus1_0 = ((ibcMbvdSizeEnc0+((1<<ricePar)-1)) >> ricePar) - 1;
    unsigned int riceBits0 = ((mmvdUnaryBits0 == numCandStepMinus1_0) && (ibcMbvdSizeEnc0&1)) ? 0 : 1;
    mmvdUnaryBits0 += (mmvdUnaryBits0 == numCandStepMinus1_0) ? 0 : 1;

    unsigned int mvpIdx1 = mmvdMergeCand1- baseIdx1 * mbvdsPerBase;
    if (baseIdx0 == baseIdx1)
    {
      mvpIdx1 -= (mvpIdx0+1);
    }
    unsigned int mmvdUnaryBits1 = mvpIdx1 >> ricePar;
    const int ibcMbvdSizeEnc1 = IBC_MBVD_SIZE_ENC-((baseIdx0 == baseIdx1) ? (mvpIdx0+1) : 0);
    int numCandStepMinus1_1 = ((ibcMbvdSizeEnc1+((1<<ricePar)-1)) >> ricePar) - 1;
    unsigned int riceBits1 = ((mmvdUnaryBits1 == numCandStepMinus1_1) && (ibcMbvdSizeEnc1&1)) ? 0 : 1;
    mmvdUnaryBits1 += (mmvdUnaryBits1 == numCandStepMinus1_1) ? 0 : 1;

    return baseBits + mmvdUnaryBits0 + riceBits0 + mmvdUnaryBits1 + riceBits1;
  }
  else
  {
#endif
  int baseIdx = mmvdMergeCand / mbvdsPerBase;
#if JVET_AE0169_BIPREDICTIVE_IBC
  int baseBits = (numCandminus1 == 0 ? 0 : baseIdx + (baseIdx == numCandminus1 ? 0: 1));
#else
  int baseBits = (IBC_MBVD_BASE_NUM == 1 ? 0 : baseIdx + (baseIdx == IBC_MBVD_BASE_NUM - 1 ? 0: 1));
#endif
  int ricePar = 1;
  unsigned int mvpIdx = mmvdMergeCand;
  mvpIdx -= baseIdx * mbvdsPerBase;
  mvpIdx >>= ricePar;
  int numCandStepMinus1 = (IBC_MBVD_SIZE_ENC >> ricePar) - 1;
  int mmvdUnaryBits = mvpIdx + (mvpIdx == numCandStepMinus1 ? 0 : 1);
#if JVET_AE0169_BIPREDICTIVE_IBC
    int32_t totalBits = baseBits + mmvdUnaryBits + ricePar;
    if (mmvdMergeCand1 >= 0) // mbvd + non-mbvd
    {
      totalBits += mmvdMergeCand1 + 1;
#if JVET_Z0084_IBC_TM
      if (mmvdMergeCand1 == pu.cs->sps->getMaxNumIBCMergeCand() - 1)
#else
      if (mmvdMergeCand1 == pu.cs->sps->getMaxNumMergeCand() - 1)
#endif
      {
        totalBits--;
      }
    }
    return totalBits;
  }
#else
  return baseBits + mmvdUnaryBits + ricePar;
#endif
}
#endif

#if MULTI_PASS_DMVR || JVET_W0097_GPM_MMVD_TM
uint32_t PU::getBDMVRMvdThreshold(const PredictionUnit &pu)
{
  uint32_t numPixels = pu.lwidth() * pu.lheight();
  if (numPixels < 64)
  {
    return (1 << MV_FRACTIONAL_BITS_INTERNAL) >> 2;
  }
  else if (numPixels < 256)
  {
    return (1 << MV_FRACTIONAL_BITS_INTERNAL) >> 1;
  }
  else
  {
    return (1 << MV_FRACTIONAL_BITS_INTERNAL) >> 0;
  }
}
#endif

#if JVET_AE0046_BI_GPM
uint32_t PU::getBiGpmThreshold(const PredictionUnit& pu)
{
  uint32_t numPixels = pu.lwidth() * pu.lheight();

  if ( numPixels < 256 )
  {
    return 1;
  }
  else
  {
    return (1 << MV_FRACTIONAL_BITS_INTERNAL);
  }
}
#endif

#if TM_MRG || TM_AMVP || JVET_Z0084_IBC_TM
uint32_t PU::getTMMvdThreshold(const PredictionUnit &pu)
{
  uint32_t numPixels = pu.lwidth() * pu.lheight();
  if (numPixels < 64)
  {
    return 1 << MV_FRACTIONAL_BITS_INTERNAL;
  }
  else if (numPixels < 256)
  {
    return 2 << MV_FRACTIONAL_BITS_INTERNAL;
  }
  else
  {
    return 4 << MV_FRACTIONAL_BITS_INTERNAL;
  }
}
#endif

#if TM_MRG
int PU::reorderInterMergeCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx, int numCand, uint32_t mvdSimilarityThresh)
{
  Slice& slice = *pu.cu->slice;

  if (numCand > 0 && slice.isInterB() && slice.getRefPOC(REF_PIC_LIST_0, 0) != slice.getRefPOC(REF_PIC_LIST_1, 0))
  {
    int numCandUni = 0;
    int numCandBi  = 0;

    // Promote bi candidates and collect uni candidates
    MergeCtx mrgCtxUni;

    for (int mergeCand = 0; mergeCand < numCand; ++mergeCand)
    {
      if (mrgCtx.interDirNeighbours[mergeCand] == 3)
      {
        mrgCtx.copyRegularMergeCand(numCandBi, mrgCtx, mergeCand);
        ++numCandBi;
      }
      else
      {
        mrgCtxUni.copyRegularMergeCand(numCandUni, mrgCtx, mergeCand);
        mrgCtxUni.convertRegularMergeCandToBi(numCandUni);
        ++numCandUni;
      }
    }

    // Insert uni-to-bi candidates
    for (int uniCandIdx = 0; uniCandIdx < numCandUni; ++uniCandIdx)
    {
      int mergeCand = numCandBi;
      mrgCtx.copyRegularMergeCand(mergeCand, mrgCtxUni, uniCandIdx);

      if (!mrgCtx.xCheckSimilarMotion(mergeCand, mvdSimilarityThresh))
      {
        ++numCandBi;
      }
#if JVET_AD0213_LIC_IMP
      else
      {
        mrgCtx.initMrgCand(mergeCand);
      }
#endif
    }

    return numCandBi;
  }
  else
  {
    return numCand;
  }
}
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
void PU::getInterMergeCandidatesSubTMVP(const PredictionUnit &pu, MergeCtx& mrgCtx,
  int col,
  const int& mrgCandIdx
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
  , MergeCtx* namvpMrgCtx
#endif
)
{
  const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
  const CodingStructure &cs = *pu.cs;
  const Slice &slice = *pu.cs->slice;
  const Picture *pColPic = slice.getRefPic(RefPicList(col == 0 ? 1 - slice.getColFromL0Flag() : 1 - slice.getColFromL0Flag2nd()), col == 0 ? slice.getColRefIdx() : slice.getColRefIdx2nd());
#if TM_MRG
  const uint32_t mvdSimilarityThresh = SUB_TMVP_MV_THRESHOLD;
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
  const uint32_t maxNumMergeCand = pu.tmMergeFlag ? TM_MRG_MAX_NUM_INIT_CANDS : SUB_TMVP_CANDIDATE_NUM;
#else
  const uint32_t maxNumMergeCand = pu.tmMergeFlag ? pu.cs->sps->getMaxNumTMMergeCand() : pu.cs->sps->getMaxNumMergeCand();
#endif
#else
  const uint32_t maxNumMergeCand = pu.cs->sps->getMaxNumMergeCand();
#endif
  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
    mrgCtx.bcwIdx[ui] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
    mrgCtx.altLMFlag[ui] = false;
    mrgCtx.altLMParaNeighbours[ui].resetAltLinearModel();
#endif
#if INTER_LIC
    mrgCtx.licFlags[ui] = false;
#endif
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mvFieldNeighbours[(ui << 1)].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
    mrgCtx.useAltHpelIf[ui] = false;
#if MULTI_HYP_PRED
    mrgCtx.addHypNeighbours[ui].clear();
#endif
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
    mrgCtx.candCost[ui] = MAX_UINT64;
#endif
  }
#if JVET_Z0102_NO_ARMC_FOR_ZERO_CAND
  mrgCtx.numCandToTestEnc = maxNumMergeCand;
#endif
  // compute the location of the current PU

  int cnt = 0;

  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  const Position posLB = pu.Y().bottomLeft();
  MotionInfo miAbove, miLeft, miAboveLeft, miAboveRight, miBelowLeft;

  // above
  const PredictionUnit *puAbove = cs.getPURestricted(posRT.offset(0, -1), pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
  bool isAvailableB1 = puAbove && isDiffMER(pu.lumaPos(), posRT.offset(0, -1), plevel) && pu.cu != puAbove->cu && CU::isInter(*puAbove->cu) && puAbove->getMotionInfo(posRT.offset(0, -1)).isInter;
#else
  bool isAvailableB1 = puAbove && isDiffMER(pu.lumaPos(), posRT.offset(0, -1), plevel) && pu.cu != puAbove->cu && CU::isInter(*puAbove->cu);
#endif

  if (isAvailableB1)
  {
    miAbove = puAbove->getMotionInfo(posRT.offset(0, -1));

    int index = 0;
    if ((miAbove.interDir & (1 << REF_PIC_LIST_0)) && slice.getRefPic(REF_PIC_LIST_0, miAbove.refIdx[0]) == pColPic)
    {
      index = 1;
    }
    else if (slice.isInterB() && (miAbove.interDir & (1 << REF_PIC_LIST_1)) && slice.getRefPic(REF_PIC_LIST_1, miAbove.refIdx[1]) == pColPic)
    {
      index = 2;
    }
    if (index > 0)
    {
      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = index;
      miAbove.mv[index - 1].roundToPrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
      if (index == 1)
      {
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAbove.mv[index - 1], miAbove.refIdx[index - 1]);
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(Mv(), NOT_VALID);
      }
      else
      {
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(Mv(), NOT_VALID);
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miAbove.mv[index - 1], miAbove.refIdx[index - 1]);
      }

#if NON_ADJACENT_MRG_CAND || TM_MRG
      if (!mrgCtx.xCheckSimilarMotionSubTMVP(cnt
#if TM_MRG
        , mvdSimilarityThresh
#endif
      ))
#endif
      {
        cnt++;
      }
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    mrgCtx.numValidMergeCand = cnt;
    return;
  }

  //left
  const PredictionUnit* puLeft = cs.getPURestricted(posLB.offset(-1, 0), pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
  const bool isAvailableA1 = puLeft && isDiffMER(pu.lumaPos(), posLB.offset(-1, 0), plevel) && pu.cu != puLeft->cu && CU::isInter(*puLeft->cu) && puLeft->getMotionInfo(posLB.offset(-1, 0)).isInter;
#else
  const bool isAvailableA1 = puLeft && isDiffMER(pu.lumaPos(), posLB.offset(-1, 0), plevel) && pu.cu != puLeft->cu && CU::isInter(*puLeft->cu);
#endif

  if (isAvailableA1)
  {
    miLeft = puLeft->getMotionInfo(posLB.offset(-1, 0));

    {
      int index = 0;
      if ((miLeft.interDir & (1 << REF_PIC_LIST_0)) && slice.getRefPic(REF_PIC_LIST_0, miLeft.refIdx[0]) == pColPic)
      {
        index = 1;
      }
      else if (slice.isInterB() && (miLeft.interDir & (1 << REF_PIC_LIST_1)) && slice.getRefPic(REF_PIC_LIST_1, miLeft.refIdx[1]) == pColPic)
      {
        index = 2;
      }
      if (index > 0)
      {
        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = index;
        miLeft.mv[index - 1].roundToPrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
        if (index == 1)
        {
          mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miLeft.mv[index - 1], miLeft.refIdx[index - 1]);
          mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(Mv(), NOT_VALID);
        }
        else
        {
          mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(Mv(), NOT_VALID);
          mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miLeft.mv[index - 1], miLeft.refIdx[index - 1]);
        }

#if NON_ADJACENT_MRG_CAND || TM_MRG
        if (!mrgCtx.xCheckSimilarMotionSubTMVP(cnt
#if TM_MRG
          , mvdSimilarityThresh
#endif
        ))
#endif
        {
          cnt++;
        }
      }
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    mrgCtx.numValidMergeCand = cnt;
    return;
  }

  // above right
  const PredictionUnit *puAboveRight = cs.getPURestricted(posRT.offset(1, -1), pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
  bool isAvailableB0 = puAboveRight && isDiffMER(pu.lumaPos(), posRT.offset(1, -1), plevel) && CU::isInter(*puAboveRight->cu) && puAboveRight->getMotionInfo(posRT.offset(1, -1)).isInter;
#else
  bool isAvailableB0 = puAboveRight && isDiffMER(pu.lumaPos(), posRT.offset(1, -1), plevel) && CU::isInter(*puAboveRight->cu);
#endif

  if (isAvailableB0)
  {
    miAboveRight = puAboveRight->getMotionInfo(posRT.offset(1, -1));

    {
      int index = 0;
      if ((miAboveRight.interDir & (1 << REF_PIC_LIST_0)) && slice.getRefPic(REF_PIC_LIST_0, miAboveRight.refIdx[0]) == pColPic)
      {
        index = 1;
      }
      else if (slice.isInterB() && (miAboveRight.interDir & (1 << REF_PIC_LIST_1)) && slice.getRefPic(REF_PIC_LIST_1, miAboveRight.refIdx[1]) == pColPic)
      {
        index = 2;
      }
      if (index > 0)
      {
        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = index;
        miAboveRight.mv[index - 1].roundToPrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
        if (index == 1)
        {
          mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAboveRight.mv[index - 1], miAboveRight.refIdx[index - 1]);
          mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(Mv(), NOT_VALID);
        }
        else
        {
          mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(Mv(), NOT_VALID);
          mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miAboveRight.mv[index - 1], miAboveRight.refIdx[index - 1]);
        }

#if NON_ADJACENT_MRG_CAND || TM_MRG
        if (!mrgCtx.xCheckSimilarMotionSubTMVP(cnt
#if TM_MRG
          , mvdSimilarityThresh
#endif
        ))
#endif
        {
          cnt++;
        }
      }
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    mrgCtx.numValidMergeCand = cnt;
    return;
  }

  //left bottom
  const PredictionUnit *puLeftBottom = cs.getPURestricted(posLB.offset(-1, 1), pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
  bool isAvailableA0 = puLeftBottom && isDiffMER(pu.lumaPos(), posLB.offset(-1, 1), plevel) && CU::isInter(*puLeftBottom->cu) && puLeftBottom->getMotionInfo(posLB.offset(-1, 1)).isInter;
#else
  bool isAvailableA0 = puLeftBottom && isDiffMER(pu.lumaPos(), posLB.offset(-1, 1), plevel) && CU::isInter(*puLeftBottom->cu);
#endif

  if (isAvailableA0)
  {
    miBelowLeft = puLeftBottom->getMotionInfo(posLB.offset(-1, 1));

    {
      int index = 0;
      if ((miBelowLeft.interDir & (1 << REF_PIC_LIST_0)) && slice.getRefPic(REF_PIC_LIST_0, miBelowLeft.refIdx[0]) == pColPic)
      {
        index = 1;
      }
      else if (slice.isInterB() && (miBelowLeft.interDir & (1 << REF_PIC_LIST_1)) && slice.getRefPic(REF_PIC_LIST_1, miBelowLeft.refIdx[1]) == pColPic)
      {
        index = 2;
      }
      if (index > 0)
      {
        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = index;
        miBelowLeft.mv[index - 1].roundToPrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
        if (index == 1)
        {
          mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miBelowLeft.mv[index - 1], miBelowLeft.refIdx[index - 1]);
          mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(Mv(), NOT_VALID);
        }
        else
        {
          mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(Mv(), NOT_VALID);
          mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miBelowLeft.mv[index - 1], miBelowLeft.refIdx[index - 1]);
        }

#if NON_ADJACENT_MRG_CAND || TM_MRG
        if (!mrgCtx.xCheckSimilarMotionSubTMVP(cnt
#if TM_MRG
          , mvdSimilarityThresh
#endif
        ))
#endif
        {
          cnt++;
        }
      }
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    mrgCtx.numValidMergeCand = cnt;
    return;
  }

  // above left
  if (cnt < 4)
  {
    const PredictionUnit *puAboveLeft = cs.getPURestricted(posLT.offset(-1, -1), pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
    bool isAvailableB2 = puAboveLeft && isDiffMER(pu.lumaPos(), posLT.offset(-1, -1), plevel) && CU::isInter(*puAboveLeft->cu) && puAboveLeft->getMotionInfo(posLT.offset(-1, -1)).isInter;
#else
    bool isAvailableB2 = puAboveLeft && isDiffMER(pu.lumaPos(), posLT.offset(-1, -1), plevel) && CU::isInter(*puAboveLeft->cu);
#endif

    if (isAvailableB2)
    {
      miAboveLeft = puAboveLeft->getMotionInfo(posLT.offset(-1, -1));

      {

        int index = 0;
        if ((miAboveLeft.interDir & (1 << REF_PIC_LIST_0)) && slice.getRefPic(REF_PIC_LIST_0, miAboveLeft.refIdx[0]) == pColPic)
        {
          index = 1;
        }
        else if (slice.isInterB() && (miAboveLeft.interDir & (1 << REF_PIC_LIST_1)) && slice.getRefPic(REF_PIC_LIST_1, miAboveLeft.refIdx[1]) == pColPic)
        {
          index = 2;
        }
        if (index > 0)
        {
          // get Inter Dir
          mrgCtx.interDirNeighbours[cnt] = index;
          miAboveLeft.mv[index - 1].roundToPrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
          if (index == 1)
          {
            mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAboveLeft.mv[index - 1], miAboveLeft.refIdx[index - 1]);
            mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(Mv(), NOT_VALID);
          }
          else
          {
            mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(Mv(), NOT_VALID);
            mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miAboveLeft.mv[index - 1], miAboveLeft.refIdx[index - 1]);
          }

#if NON_ADJACENT_MRG_CAND || TM_MRG
          if (!mrgCtx.xCheckSimilarMotionSubTMVP(cnt
#if TM_MRG
            , mvdSimilarityThresh
#endif
          ))
#endif
          {
            cnt++;
          }
        }
      }
    }
    // early termination
    if (cnt == maxNumMergeCand)
    {
      mrgCtx.numValidMergeCand = cnt;
      return;
    }
  }
#if NON_ADJACENT_MRG_CAND
  if (namvpMrgCtx != NULL)
  {
    for (uint32_t ui = 0; ui < namvpMrgCtx->numValidMergeCand && cnt < maxNumMergeCand; ++ui)
    {
      mrgCtx.interDirNeighbours[cnt] = namvpMrgCtx->interDirNeighbours[ui];
      mrgCtx.mvFieldNeighbours[cnt << 1] = namvpMrgCtx->mvFieldNeighbours[ui << 1];
      mrgCtx.mvFieldNeighbours[(cnt << 1) + 1] = namvpMrgCtx->mvFieldNeighbours[(ui << 1) + 1];

#if NON_ADJACENT_MRG_CAND || TM_MRG
      if (!mrgCtx.xCheckSimilarMotionSubTMVP(cnt
#if TM_MRG
        , mvdSimilarityThresh
#endif
      ))
      {
#endif
        mrgCtx.candCost[cnt] = namvpMrgCtx->candCost[ui];
        cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG
      }
#endif
    }
  }
#endif

  if (cnt != maxNumMergeCand)
  {
    addMergeHMVPCandSubTMVP(cs, mrgCtx, mrgCandIdx, maxNumMergeCand, cnt
      , isAvailableA1, miLeft, isAvailableB1, miAbove, pu, col
#if TM_MRG
      , mvdSimilarityThresh
#endif
    );
  }
  mrgCtx.numValidMergeCand = cnt;

  return;
}
#endif
void PU::getInterMergeCandidates( const PredictionUnit &pu, MergeCtx& mrgCtx,
                                 int mmvdList,
                                 const int& mrgCandIdx
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
                                , MergeCtx* tmvpMrgCtx
                                , MergeCtx* namvpMrgCtx
#endif
#if JVET_AE0046_BI_GPM
                                , bool enableTh4Gpm
#endif
)
{
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
  if (!(pu.cs->sps->getUseAML() && pu.cs->sps->getTMToolsEnableFlag())|| !pu.cs->sps->getUseTmvpNmvpReordering())
#else
  if (!pu.cs->sps->getUseAML() || !pu.cs->sps->getUseTmvpNmvpReordering())
#endif
  {
    tmvpMrgCtx  = nullptr;
    namvpMrgCtx = nullptr;
  }
#endif
  const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
  const CodingStructure &cs  = *pu.cs;
  const Slice &slice         = *pu.cs->slice;
#if TM_MRG
#if JVET_AE0046_BI_GPM
  const uint32_t mvdSimilarityThresh = pu.tmMergeFlag ? PU::getTMMvdThreshold(pu) : (enableTh4Gpm ? getBiGpmThreshold(pu) : 1);
#else
  const uint32_t mvdSimilarityThresh = pu.tmMergeFlag ? PU::getTMMvdThreshold(pu) : 1;
#endif
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC && JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
  uint32_t additionalMRGCand = 0;
  if (tmvpMrgCtx != NULL)
  {
    additionalMRGCand = tmvpMrgCtx->numValidMergeCand;
  }
  if (namvpMrgCtx != NULL)
  {
    additionalMRGCand += namvpMrgCtx->numValidMergeCand;
  }
  const uint32_t maxNumMergeCand =
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
                                   !(pu.cs->sps->getUseAML() && pu.cs->sps->getTMToolsEnableFlag())|| !pu.cs->sps->getUseTmvpNmvpReordering() ?
#else
                                   !pu.cs->sps->getUseAML() || !pu.cs->sps->getUseTmvpNmvpReordering() ?
#endif
                                   (pu.tmMergeFlag ? pu.cs->sps->getMaxNumTMMergeCand() : pu.cs->sps->getMaxNumMergeCand()) :
#endif
                                   (pu.tmMergeFlag ? std::min((int)(TM_MRG_MAX_NUM_INIT_CANDS        + additionalMRGCand), ((int)NUM_MERGE_CANDS - (3)))
                                                   : std::min((int)(pu.cs->sps->getMaxNumMergeCand() + additionalMRGCand), ((int)NUM_MERGE_CANDS - (3))));
#else
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
  const uint32_t maxNumMergeCand = pu.tmMergeFlag ? (( pu.cs->sps->getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
                                                  && pu.cs->sps->getTMToolsEnableFlag()
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
                                                  && pu.cs->sps->getUseTmvpNmvpReordering()
#endif
                                                    ) ? TM_MRG_MAX_NUM_INIT_CANDS : pu.cs->sps->getMaxNumTMMergeCand()) : pu.cs->sps->getMaxNumMergeCand();
#else
  const uint32_t maxNumMergeCand     = pu.tmMergeFlag ? pu.cs->sps->getMaxNumTMMergeCand() : pu.cs->sps->getMaxNumMergeCand();
#endif
#endif
#else
#if JVET_AE0046_BI_GPM
  const uint32_t mvdSimilarityThresh = enableTh4Gpm ? getBiGpmThreshold(pu) : 1;
#endif
  const uint32_t maxNumMergeCand = pu.cs->sps->getMaxNumMergeCand();
#endif
  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
    mrgCtx.bcwIdx[ui] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
    mrgCtx.altLMFlag[ui] = false;
    mrgCtx.altLMParaNeighbours[ui].resetAltLinearModel();
#endif
#if INTER_LIC
    mrgCtx.licFlags[ui] = false;
#endif
#if JVET_AC0112_IBC_LIC
    mrgCtx.ibcLicFlags[ui] = false;
#endif
#if JVET_AE0159_FIBC
    mrgCtx.ibcFilterFlags[ui] = false;
#endif
#if JVET_AA0070_RRIBC
    mrgCtx.rribcFlipTypes[ui] = 0;
#endif
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mvFieldNeighbours[(ui << 1)    ].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
    mrgCtx.useAltHpelIf[ui] = false;
#if MULTI_HYP_PRED
    mrgCtx.addHypNeighbours[ui].clear();
#endif
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
    mrgCtx.candCost[ui] = MAX_UINT64;
#endif
  }

  mrgCtx.numValidMergeCand = maxNumMergeCand;
#if JVET_Z0102_NO_ARMC_FOR_ZERO_CAND
  mrgCtx.numCandToTestEnc = maxNumMergeCand;
#endif
  // compute the location of the current PU
#if JVET_X0083_BM_AMVP_MERGE_MODE
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
  int amvpMergeCtxMergeDir = -1;
  RefPicList amvpRefList = REF_PIC_LIST_X;
  bool useAmvpMergeMode = false;
#endif
#if JVET_Y0128_NON_CTC
  if (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])
  {
    mrgCtx.numValidMergeCand = (mrgCandIdx + 1) > maxNumMergeCand ? maxNumMergeCand : (mrgCandIdx + 1);
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
    useAmvpMergeMode = true;
    if (pu.amvpMergeModeFlag[0] == true)
    {
      amvpMergeCtxMergeDir = 1;
      amvpRefList = REF_PIC_LIST_1;
    }
    else
    {
      amvpMergeCtxMergeDir = 2;
      amvpRefList = REF_PIC_LIST_0;
    }
#endif
  }
#else
  const int curPoc = slice.getPOC();
#if !JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
  RefPicList amvpRefList = REF_PIC_LIST_X;
#endif
  RefPicList mergeRefList = REF_PIC_LIST_X;
  int amvpPoc = -1;
  if (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])
  {
    mergeRefList = pu.amvpMergeModeFlag[0] ? REF_PIC_LIST_0 : REF_PIC_LIST_1;
    amvpRefList = RefPicList(1 - mergeRefList);
    amvpPoc = slice.getRefPOC(amvpRefList, pu.refIdx[amvpRefList]);
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
    useAmvpMergeMode = true;
    if (pu.amvpMergeModeFlag[0] == true)
    {
      amvpMergeCtxMergeDir = 1;
    }
    else
    {
      amvpMergeCtxMergeDir = 2;
    }
#endif
    mrgCtx.numValidMergeCand = (mrgCandIdx + 1) > maxNumMergeCand ? maxNumMergeCand : (mrgCandIdx + 1);
  }
#endif
#endif

  int cnt = 0;

  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  const Position posLB = pu.Y().bottomLeft();
  MotionInfo miAbove, miLeft, miAboveLeft, miAboveRight, miBelowLeft;

  // above
  const PredictionUnit *puAbove = cs.getPURestricted(posRT.offset(0, -1), pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
  bool isAvailableB1 = puAbove && isDiffMER(pu.lumaPos(), posRT.offset(0, -1), plevel) && pu.cu != puAbove->cu && CU::isInter(*puAbove->cu) && puAbove->getMotionInfo(posRT.offset(0, -1)).isInter;
#else
  bool isAvailableB1 = puAbove && isDiffMER(pu.lumaPos(), posRT.offset(0, -1), plevel) && pu.cu != puAbove->cu && CU::isInter(*puAbove->cu);
#endif

  if (isAvailableB1)
  {
    miAbove = puAbove->getMotionInfo(posRT.offset(0, -1));

#if JVET_X0083_BM_AMVP_MERGE_MODE
#if JVET_Y0128_NON_CTC
    bool isValidAmMode = checkIsValidMergeMvCand(pu, miAbove.refIdx);
#else
    bool isValidAmMode = checkIsValidMergeMvCand(cs, pu, curPoc, amvpPoc, miAbove.refIdx);
#endif
    if (isValidAmMode)
    {
#endif
    // get Inter Dir
    mrgCtx.interDirNeighbours[cnt] = miAbove.interDir;
    mrgCtx.useAltHpelIf[cnt] = miAbove.useAltHpelIf;
    // get Mv from Above
    mrgCtx.bcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAbove->cu->bcwIdx : BCW_DEFAULT;
#if INTER_LIC
#if JVET_AG0276_NLIC
    mrgCtx.licFlags[cnt] = miAbove.usesLIC || (puAbove->cu->altLMFlag && slice.getUseLIC());
#else
    mrgCtx.licFlags[cnt] = miAbove.usesLIC;
#endif
#endif
    mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAbove.mv[0], miAbove.refIdx[0]);

    if (slice.isInterB())
    {
      mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miAbove.mv[1], miAbove.refIdx[1]);
#if MULTI_HYP_PRED
      mrgCtx.addHypNeighbours[cnt] = puAbove->addHypData;
#endif
    }
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
    if (useAmvpMergeMode)
    {
      mrgCtx.interDirNeighbours[cnt] = amvpMergeCtxMergeDir;
      mrgCtx.mvFieldNeighbours[(cnt << 1) + amvpRefList].setMvField(Mv(), -1);
    }
#endif
#if NON_ADJACENT_MRG_CAND || TM_MRG || JVET_AE0046_BI_GPM
    if( !mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG || JVET_AE0046_BI_GPM
                           , mvdSimilarityThresh
#endif
    ) )
    {
#endif
    if (mrgCandIdx == cnt)
    {
#if TM_MRG
      if( !pu.tmMergeFlag )
#endif
      return;
    }

    cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG || JVET_AE0046_BI_GPM
    }
#if JVET_AD0213_LIC_IMP
    else
    {
      mrgCtx.initMrgCand(cnt);
    }
#endif
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
    } // if (isValidAmMode)
#endif
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
#if TM_MRG
    if (!pu.tmMergeFlag)
#endif
    return;
  }

  //left
  const PredictionUnit* puLeft = cs.getPURestricted(posLB.offset(-1, 0), pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
  const bool isAvailableA1 = puLeft && isDiffMER(pu.lumaPos(), posLB.offset(-1, 0), plevel) && pu.cu != puLeft->cu && CU::isInter(*puLeft->cu) && puLeft->getMotionInfo(posLB.offset(-1, 0)).isInter;
#else
  const bool isAvailableA1 = puLeft && isDiffMER(pu.lumaPos(), posLB.offset(-1, 0), plevel) && pu.cu != puLeft->cu && CU::isInter(*puLeft->cu);
#endif

  if (isAvailableA1)
  {
    miLeft = puLeft->getMotionInfo(posLB.offset(-1, 0));

#if JVET_X0083_BM_AMVP_MERGE_MODE
#if JVET_Y0128_NON_CTC
    bool isValidAmMode = checkIsValidMergeMvCand(pu, miLeft.refIdx);
#else
    bool isValidAmMode = checkIsValidMergeMvCand(cs, pu, curPoc, amvpPoc, miLeft.refIdx);
#endif
    if (isValidAmMode && (!isAvailableB1 || (miAbove != miLeft)))
#else
    if (!isAvailableB1 || (miAbove != miLeft))
#endif
    {
      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miLeft.interDir;
      mrgCtx.useAltHpelIf[cnt] = miLeft.useAltHpelIf;
      mrgCtx.bcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puLeft->cu->bcwIdx : BCW_DEFAULT;
#if INTER_LIC
#if JVET_AG0276_NLIC
      mrgCtx.licFlags[cnt] = miLeft.usesLIC || (puLeft->cu->altLMFlag && slice.getUseLIC());
#else
      mrgCtx.licFlags[cnt] = miLeft.usesLIC;
#endif
#endif
      // get Mv from Left
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miLeft.mv[0], miLeft.refIdx[0]);

      if (slice.isInterB())
      {
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miLeft.mv[1], miLeft.refIdx[1]);
#if MULTI_HYP_PRED
        mrgCtx.addHypNeighbours[cnt] = puLeft->addHypData;
#endif
      }
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
      if (useAmvpMergeMode)
      {
        mrgCtx.interDirNeighbours[cnt] = amvpMergeCtxMergeDir;
        mrgCtx.mvFieldNeighbours[(cnt << 1) + amvpRefList].setMvField(Mv(), -1);
      }
#endif
#if NON_ADJACENT_MRG_CAND || TM_MRG || JVET_AE0046_BI_GPM
      if( !mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG || JVET_AE0046_BI_GPM
                             , mvdSimilarityThresh
#endif
      ) )
      {
#endif
      if (mrgCandIdx == cnt)
      {
#if TM_MRG
        if (!pu.tmMergeFlag)
#endif
        return;
      }

      cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG || JVET_AE0046_BI_GPM
      }
#if JVET_AD0213_LIC_IMP
      else
      {
        mrgCtx.initMrgCand(cnt);
      }
#endif
#endif
    }
  }

  // early termination
  if( cnt == maxNumMergeCand )
  {
#if TM_MRG
    if (!pu.tmMergeFlag)
#endif
    return;
  }

  // above right
  const PredictionUnit *puAboveRight = cs.getPURestricted( posRT.offset( 1, -1 ), pu, pu.chType );

#if JVET_Y0065_GPM_INTRA
  bool isAvailableB0 = puAboveRight && isDiffMER( pu.lumaPos(), posRT.offset(1, -1), plevel) && CU::isInter( *puAboveRight->cu ) && puAboveRight->getMotionInfo( posRT.offset( 1, -1 ) ).isInter;
#else
  bool isAvailableB0 = puAboveRight && isDiffMER( pu.lumaPos(), posRT.offset(1, -1), plevel) && CU::isInter( *puAboveRight->cu );
#endif

  if( isAvailableB0 )
  {
    miAboveRight = puAboveRight->getMotionInfo( posRT.offset( 1, -1 ) );

#if JVET_X0083_BM_AMVP_MERGE_MODE
#if JVET_Y0128_NON_CTC
    bool isValidAmMode = checkIsValidMergeMvCand(pu, miAboveRight.refIdx);
#else
    bool isValidAmMode = checkIsValidMergeMvCand(cs, pu, curPoc, amvpPoc, miAboveRight.refIdx);
#endif
    if (isValidAmMode && (!isAvailableB1 || ( miAbove != miAboveRight )))
#else
    if( !isAvailableB1 || ( miAbove != miAboveRight ) )
#endif
    {

      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miAboveRight.interDir;
      mrgCtx.useAltHpelIf[cnt] = miAboveRight.useAltHpelIf;
      // get Mv from Above-right
      mrgCtx.bcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAboveRight->cu->bcwIdx : BCW_DEFAULT;
#if INTER_LIC
#if JVET_AG0276_NLIC
      mrgCtx.licFlags[cnt] = miAboveRight.usesLIC || (puAboveRight->cu->altLMFlag && slice.getUseLIC());
#else
      mrgCtx.licFlags[cnt] = miAboveRight.usesLIC;
#endif
#endif
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAboveRight.mv[0], miAboveRight.refIdx[0] );

      if( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miAboveRight.mv[1], miAboveRight.refIdx[1] );
#if MULTI_HYP_PRED
        mrgCtx.addHypNeighbours[cnt] = puAboveRight->addHypData;
#endif
      }
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
      if (useAmvpMergeMode)
      {
        mrgCtx.interDirNeighbours[cnt] = amvpMergeCtxMergeDir;
        mrgCtx.mvFieldNeighbours[(cnt << 1) + amvpRefList].setMvField(Mv(), -1);
      }
#endif

#if NON_ADJACENT_MRG_CAND || TM_MRG || JVET_AE0046_BI_GPM
      if( !mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG || JVET_AE0046_BI_GPM
                             , mvdSimilarityThresh
#endif
      ) )
      {
#endif
      if (mrgCandIdx == cnt)
      {
#if TM_MRG
        if (!pu.tmMergeFlag)
#endif
        return;
      }

      cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG || JVET_AE0046_BI_GPM
      }
#if JVET_AD0213_LIC_IMP
      else
      {
        mrgCtx.initMrgCand(cnt);
      }
#endif
#endif
    }
  }
  // early termination
  if( cnt == maxNumMergeCand )
  {
#if TM_MRG
    if (!pu.tmMergeFlag)
#endif
    return;
  }

  //left bottom
  const PredictionUnit *puLeftBottom = cs.getPURestricted( posLB.offset( -1, 1 ), pu, pu.chType );

#if JVET_Y0065_GPM_INTRA
  bool isAvailableA0 = puLeftBottom && isDiffMER( pu.lumaPos(), posLB.offset(-1, 1), plevel) && CU::isInter( *puLeftBottom->cu ) && puLeftBottom->getMotionInfo( posLB.offset( -1, 1 ) ).isInter;
#else
  bool isAvailableA0 = puLeftBottom && isDiffMER( pu.lumaPos(), posLB.offset(-1, 1), plevel) && CU::isInter( *puLeftBottom->cu );
#endif

  if( isAvailableA0 )
  {
    miBelowLeft = puLeftBottom->getMotionInfo( posLB.offset( -1, 1 ) );

#if JVET_X0083_BM_AMVP_MERGE_MODE
#if JVET_Y0128_NON_CTC
    bool isValidAmMode = checkIsValidMergeMvCand(pu, miBelowLeft.refIdx);
#else
    bool isValidAmMode = checkIsValidMergeMvCand(cs, pu, curPoc, amvpPoc, miBelowLeft.refIdx);
#endif
    if (isValidAmMode && (!isAvailableA1 || ( miBelowLeft != miLeft )))
#else
    if( !isAvailableA1 || ( miBelowLeft != miLeft ) )
#endif
    {
      // get Inter Dir
      mrgCtx.interDirNeighbours[cnt] = miBelowLeft.interDir;
      mrgCtx.useAltHpelIf[cnt] = miBelowLeft.useAltHpelIf;
      mrgCtx.bcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puLeftBottom->cu->bcwIdx : BCW_DEFAULT;
#if INTER_LIC
#if JVET_AG0276_NLIC
      mrgCtx.licFlags[cnt] = miBelowLeft.usesLIC || (puLeftBottom->cu->altLMFlag && slice.getUseLIC());
#else
      mrgCtx.licFlags[cnt] = miBelowLeft.usesLIC;
#endif
#endif
      // get Mv from Bottom-Left
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miBelowLeft.mv[0], miBelowLeft.refIdx[0] );

      if( slice.isInterB() )
      {
        mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miBelowLeft.mv[1], miBelowLeft.refIdx[1] );
#if MULTI_HYP_PRED
        mrgCtx.addHypNeighbours[cnt] = puLeftBottom->addHypData;
#endif
      }
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
      if (useAmvpMergeMode)
      {
        mrgCtx.interDirNeighbours[cnt] = amvpMergeCtxMergeDir;
        mrgCtx.mvFieldNeighbours[(cnt << 1) + amvpRefList].setMvField(Mv(), -1);
      }
#endif

#if NON_ADJACENT_MRG_CAND || TM_MRG || JVET_AE0046_BI_GPM
      if( !mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG || JVET_AE0046_BI_GPM
                             , mvdSimilarityThresh
#endif
      ) )
      {
#endif
      if (mrgCandIdx == cnt)
      {
#if TM_MRG
        if (!pu.tmMergeFlag)
#endif
        return;
      }

      cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG || JVET_AE0046_BI_GPM
      }
#if JVET_AD0213_LIC_IMP
      else
      {
        mrgCtx.initMrgCand(cnt);
      }
#endif
#endif
    }
  }
  // early termination
  if( cnt == maxNumMergeCand )
  {
#if TM_MRG
    if (!pu.tmMergeFlag)
#endif
    return;
  }

#if TM_MRG
  if (pu.tmMergeFlag)
  {
    cnt = PU::reorderInterMergeCandidates(pu, mrgCtx, cnt, mvdSimilarityThresh);

    if ((mrgCandIdx >= 0 && mrgCandIdx < cnt) || cnt >= maxNumMergeCand)
    {
      return;
    }
  }
#endif

  // above left
  if ( cnt < 4 )
  {
    const PredictionUnit *puAboveLeft = cs.getPURestricted( posLT.offset( -1, -1 ), pu, pu.chType );

#if JVET_Y0065_GPM_INTRA
    bool isAvailableB2 = puAboveLeft && isDiffMER( pu.lumaPos(), posLT.offset(-1, -1), plevel ) && CU::isInter( *puAboveLeft->cu ) && puAboveLeft->getMotionInfo( posLT.offset( -1, -1 ) ).isInter;
#else
    bool isAvailableB2 = puAboveLeft && isDiffMER( pu.lumaPos(), posLT.offset(-1, -1), plevel ) && CU::isInter( *puAboveLeft->cu );
#endif

    if( isAvailableB2 )
    {
      miAboveLeft = puAboveLeft->getMotionInfo( posLT.offset( -1, -1 ) );

#if JVET_X0083_BM_AMVP_MERGE_MODE
#if JVET_Y0128_NON_CTC
      bool isValidAmMode = checkIsValidMergeMvCand(pu, miAboveLeft.refIdx);
#else
      bool isValidAmMode = checkIsValidMergeMvCand(cs, pu, curPoc, amvpPoc, miAboveLeft.refIdx);
#endif
      if (isValidAmMode && ( !isAvailableA1 || ( miLeft != miAboveLeft ) ) && ( !isAvailableB1 || ( miAbove != miAboveLeft ) ))
#else
      if( ( !isAvailableA1 || ( miLeft != miAboveLeft ) ) && ( !isAvailableB1 || ( miAbove != miAboveLeft ) ) )
#endif
      {

        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miAboveLeft.interDir;
        mrgCtx.useAltHpelIf[cnt] = miAboveLeft.useAltHpelIf;
        mrgCtx.bcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAboveLeft->cu->bcwIdx : BCW_DEFAULT;
#if INTER_LIC
#if JVET_AG0276_NLIC
        mrgCtx.licFlags[cnt] = miAboveLeft.usesLIC || (puAboveLeft->cu->altLMFlag && slice.getUseLIC());
#else
        mrgCtx.licFlags[cnt] = miAboveLeft.usesLIC;
#endif
#endif
        // get Mv from Above-Left
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField( miAboveLeft.mv[0], miAboveLeft.refIdx[0] );

        if( slice.isInterB() )
        {
          mrgCtx.mvFieldNeighbours[( cnt << 1 ) + 1].setMvField( miAboveLeft.mv[1], miAboveLeft.refIdx[1] );
#if MULTI_HYP_PRED
          mrgCtx.addHypNeighbours[cnt] = puAboveLeft->addHypData;
#endif
        }
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
        if (useAmvpMergeMode)
        {
          mrgCtx.interDirNeighbours[cnt] = amvpMergeCtxMergeDir;
          mrgCtx.mvFieldNeighbours[(cnt << 1) + amvpRefList].setMvField(Mv(), -1);
        }
#endif

#if NON_ADJACENT_MRG_CAND || TM_MRG || JVET_AE0046_BI_GPM
        if( !mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG || JVET_AE0046_BI_GPM
                               , mvdSimilarityThresh
#endif
        ) )
        {
#endif
        if (mrgCandIdx == cnt)
        {
          return;
        }

        cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG || JVET_AE0046_BI_GPM
        }
#if JVET_AD0213_LIC_IMP
        else
        {
          mrgCtx.initMrgCand(cnt);
        }
#endif
#endif
      }
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

#if INTER_RM_SIZE_CONSTRAINTS
  if (slice.getPicHeader()->getEnableTMVPFlag())
#else
  if (slice.getPicHeader()->getEnableTMVPFlag() && (pu.lumaSize().width + pu.lumaSize().height > 12))
#endif
  {
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
    if (tmvpMrgCtx == NULL)
    {
#endif
    //>> MTK colocated-RightBottom
    // offset the pos to be sure to "point" to the same position the uiAbsPartIdx would've pointed to
    Position posRB = pu.Y().bottomRight().offset( -3, -3 );
    const PreCalcValues& pcv = *cs.pcv;

    Position posC0;
    Position posC1 = pu.Y().center();
    bool isC0Avail = false;
    bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
    const SubPic& curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
    if (curSubPic.getTreatedAsPicFlag())
    {
      boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
                      (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom());
    }
    if (boundaryCond)
    {
      int posYInCtu = posRB.y & pcv.maxCUHeightMask;
      if (posYInCtu + 4 < pcv.maxCUHeight)
      {
        posC0 = posRB.offset(4, 4);
        isC0Avail = true;
      }
    }

    Mv        cColMv;
    int       iRefIdx     = 0;
    int       dir         = 0;
    unsigned  uiArrayAddr = cnt;

    bool      bExistMV = (isC0Avail && getColocatedMVP(pu, REF_PIC_LIST_0, posC0, cColMv, iRefIdx, false
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
        , 0
#endif
        , &iRefIdx
#endif
      ))
        || getColocatedMVP(pu, REF_PIC_LIST_0, posC1, cColMv, iRefIdx, false
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
          , 0
#endif
          , &iRefIdx
#endif
        );
      if (bExistMV)
      {
        dir |= 1;
        mrgCtx.mvFieldNeighbours[2 * uiArrayAddr].setMvField(cColMv, iRefIdx);
      }

      if (slice.isInterB())
      {
        bExistMV = (isC0Avail && getColocatedMVP(pu, REF_PIC_LIST_1, posC0, cColMv, iRefIdx, false
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
          , 0
#endif
          , &iRefIdx
#endif
        ))
          || getColocatedMVP(pu, REF_PIC_LIST_1, posC1, cColMv, iRefIdx, false
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
            , 0
#endif
            , &iRefIdx
#endif
          );
        if (bExistMV)
        {
          dir |= 2;
          mrgCtx.mvFieldNeighbours[2 * uiArrayAddr + 1].setMvField(cColMv, iRefIdx);
        }
      }

#if JVET_X0083_BM_AMVP_MERGE_MODE
      int8_t tempRefIdx[2] = { mrgCtx.mvFieldNeighbours[2 * uiArrayAddr].refIdx, mrgCtx.mvFieldNeighbours[2 * uiArrayAddr + 1].refIdx };
#if JVET_Y0128_NON_CTC
      bool isValidAmMode = checkIsValidMergeMvCand(pu, tempRefIdx);
#else
      bool isValidAmMode = checkIsValidMergeMvCand(cs, pu, curPoc, amvpPoc, tempRefIdx);
#endif
      if (isValidAmMode && (dir != 0))
#else
      if (dir != 0)
#endif
      {
        bool addTMvp = true;
        if (addTMvp)
        {
          mrgCtx.interDirNeighbours[uiArrayAddr] = dir;
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
          if (useAmvpMergeMode)
          {
            mrgCtx.interDirNeighbours[cnt] = amvpMergeCtxMergeDir;
            mrgCtx.mvFieldNeighbours[(cnt << 1) + amvpRefList].setMvField(Mv(), -1);
          }
#endif
          mrgCtx.bcwIdx[uiArrayAddr] = BCW_DEFAULT;
#if INTER_LIC
          mrgCtx.licFlags[uiArrayAddr] = false;
#endif
          mrgCtx.useAltHpelIf[uiArrayAddr] = false;
#if MULTI_HYP_PRED
          mrgCtx.addHypNeighbours[uiArrayAddr].clear();
#endif
#if NON_ADJACENT_MRG_CAND || TM_MRG || JVET_AE0046_BI_GPM
          if (!mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG || JVET_AE0046_BI_GPM
            , mvdSimilarityThresh
#endif
          ))
          {
#endif
            if (mrgCandIdx == cnt)
            {
              return;
            }

            cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG || JVET_AE0046_BI_GPM
          }
#if JVET_AD0213_LIC_IMP
          else
          {
            mrgCtx.initMrgCand(cnt);
          }
#endif
#endif
        }
      }
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
    }
    else if (tmvpMrgCtx->numValidMergeCand > 0)
    {
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
#if TM_MRG
      if (!pu.tmMergeFlag
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
        || !pu.cs->sps->getUseTMMrgMode()
#endif
        )
#endif
      {
        for (uint32_t ui = 0; ui < tmvpMrgCtx->numValidMergeCand && cnt < maxNumMergeCand - 1; ++ui)
        {
          mrgCtx.bcwIdx[cnt] = tmvpMrgCtx->bcwIdx[ui];
#if INTER_LIC
          mrgCtx.licFlags[cnt] = tmvpMrgCtx->licFlags[ui];
#endif
          mrgCtx.interDirNeighbours[cnt] = tmvpMrgCtx->interDirNeighbours[ui];
          mrgCtx.mvFieldNeighbours[cnt << 1] = tmvpMrgCtx->mvFieldNeighbours[ui << 1];
          mrgCtx.useAltHpelIf[cnt] = tmvpMrgCtx->useAltHpelIf[ui];
          if (slice.isInterB())
          {
            mrgCtx.mvFieldNeighbours[(cnt << 1) + 1] = tmvpMrgCtx->mvFieldNeighbours[(ui << 1) + 1];
#if MULTI_HYP_PRED
            mrgCtx.addHypNeighbours[cnt] = tmvpMrgCtx->addHypNeighbours[ui];
#endif
          }

#if NON_ADJACENT_MRG_CAND || TM_MRG || JVET_AE0046_BI_GPM
          if (!mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG || JVET_AE0046_BI_GPM
            , mvdSimilarityThresh
#endif
          ))
          {
#endif
            mrgCtx.candCost[cnt] = tmvpMrgCtx->candCost[ui];
            if (mrgCandIdx == cnt)
            {
              return;
            }

            cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG || JVET_AE0046_BI_GPM
          }
#if JVET_AD0213_LIC_IMP
          else
          {
            mrgCtx.initMrgCand(cnt);
          }
#endif
#endif
        }
      }
#if TM_MRG
      else
      {
        mrgCtx.bcwIdx[cnt] = tmvpMrgCtx->bcwIdx[0];
#if INTER_LIC
        mrgCtx.licFlags[cnt] = tmvpMrgCtx->licFlags[0];
#endif
        mrgCtx.interDirNeighbours[cnt] = tmvpMrgCtx->interDirNeighbours[0];
        mrgCtx.mvFieldNeighbours[cnt << 1] = tmvpMrgCtx->mvFieldNeighbours[0];
        mrgCtx.useAltHpelIf[cnt] = tmvpMrgCtx->useAltHpelIf[0];
        if (slice.isInterB())
        {
          mrgCtx.mvFieldNeighbours[(cnt << 1) + 1] = tmvpMrgCtx->mvFieldNeighbours[1];
#if MULTI_HYP_PRED
          mrgCtx.addHypNeighbours[cnt] = tmvpMrgCtx->addHypNeighbours[0];
#endif
        }

#if NON_ADJACENT_MRG_CAND || TM_MRG || JVET_AE0046_BI_GPM
        if (!mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG || JVET_AE0046_BI_GPM
          , mvdSimilarityThresh
#endif
        ))
        {
#endif
          mrgCtx.candCost[cnt] = tmvpMrgCtx->candCost[0];
          if (mrgCandIdx == cnt)
          {
            return;
          }

          cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG || JVET_AE0046_BI_GPM
        }
#if JVET_AD0213_LIC_IMP
        else
        {
          mrgCtx.initMrgCand(cnt);
        }
#endif
#endif
      }
#endif
    }
#else
      mrgCtx.bcwIdx[cnt] = tmvpMrgCtx->bcwIdx[0];
#if INTER_LIC
      mrgCtx.licFlags[cnt] = tmvpMrgCtx->licFlags[0];
#endif
      mrgCtx.interDirNeighbours[cnt] = tmvpMrgCtx->interDirNeighbours[0];
      mrgCtx.mvFieldNeighbours[cnt << 1] = tmvpMrgCtx->mvFieldNeighbours[0];
      mrgCtx.useAltHpelIf[cnt] = tmvpMrgCtx->useAltHpelIf[0];
      if (slice.isInterB())
      {
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1] = tmvpMrgCtx->mvFieldNeighbours[1];
#if MULTI_HYP_PRED
        mrgCtx.addHypNeighbours[cnt] = tmvpMrgCtx->addHypNeighbours[0];
#endif
      }

#if NON_ADJACENT_MRG_CAND || TM_MRG || JVET_AE0046_BI_GPM
      if (!mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG || JVET_AE0046_BI_GPM
        , mvdSimilarityThresh
#endif
      ))
      {
#endif
        mrgCtx.candCost[cnt] = tmvpMrgCtx->candCost[0];
        if (mrgCandIdx == cnt)
        {
          return;
        }

        cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG || JVET_AE0046_BI_GPM
      }
#if JVET_AD0213_LIC_IMP
      else
      {
        mrgCtx.initMrgCand(cnt);
      }
#endif
#endif
    }
#endif
#endif
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }

#if NON_ADJACENT_MRG_CAND
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
  if (namvpMrgCtx == NULL)
  {
#endif
  MotionInfo miNeighbor;
  int offsetX = 0;
  int offsetY = 0;
  const int numNACandidate[4] = { 3, 5, 5, 5 };
  const int idxMap[4][5] = { { 0, 1, 4 },{ 0, 1, 2, 3, 4 },{ 0, 1, 2, 3, 4 },{ 0, 1, 2, 3, 4 } };

#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
  for (int iDistanceIndex = 0; iDistanceIndex < NADISTANCE_LEVEL && cnt < maxNumMergeCand; iDistanceIndex++)
#else
  for (int iDistanceIndex = 0; iDistanceIndex < NADISTANCE_LEVEL && cnt < maxNumMergeCand - 1; iDistanceIndex++)
#endif
  {
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
  if (!(pu.cs->sps->getUseAML() && pu.cs->sps->getTMToolsEnableFlag()) && cnt >= maxNumMergeCand - 1)
#else
    if (!pu.cs->sps->getUseAML() && cnt >= maxNumMergeCand - 1)
#endif
    {
      continue;
    }
#endif
    const int iNADistanceHor = pu.Y().width  * (iDistanceIndex + 1);
    const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);

#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
    for (int iNASPIdx = 0; iNASPIdx < numNACandidate[iDistanceIndex] && cnt < maxNumMergeCand; iNASPIdx++)
#else
    for (int iNASPIdx = 0; iNASPIdx < numNACandidate[iDistanceIndex] && cnt < maxNumMergeCand - 1; iNASPIdx++)
#endif
    {
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
  if (!(pu.cs->sps->getUseAML() && pu.cs->sps->getTMToolsEnableFlag()) && cnt >= maxNumMergeCand - 1)
#else
      if (!pu.cs->sps->getUseAML() && cnt >= maxNumMergeCand - 1)
#endif
      {
        continue;
      }
#endif
      switch (idxMap[iDistanceIndex][iNASPIdx])
      {
      case 0:offsetX = -iNADistanceHor - 1; offsetY = pu.Y().height + iNADistanceVer - 1; break;
      case 1:offsetX = pu.Y().width + iNADistanceHor - 1; offsetY = -iNADistanceVer - 1; break;
      case 2:offsetX = pu.Y().width >> 1;   offsetY = -iNADistanceVer - 1;    break;
      case 3:offsetX = -iNADistanceHor - 1; offsetY = pu.Y().height >> 1; break;
      case 4:offsetX = -iNADistanceHor - 1; offsetY = -iNADistanceVer - 1;    break;
      default: printf("error!"); exit(0); break;
      }

      const PredictionUnit *puNonAdjacent = cs.getPURestricted(posLT.offset(offsetX, offsetY), pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER( pu.lumaPos(), posLT.offset( offsetX, offsetY ) , plevel) && CU::isInter(*puNonAdjacent->cu) && puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY)).isInter;
#else
      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER( pu.lumaPos(), posLT.offset( offsetX, offsetY ) , plevel) && CU::isInter(*puNonAdjacent->cu);
#endif

      if (isAvailableNonAdjacent)
      {
        miNeighbor = puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY));


#if JVET_X0083_BM_AMVP_MERGE_MODE
#if JVET_Y0128_NON_CTC
        bool isValidAmMode = checkIsValidMergeMvCand(pu, miNeighbor.refIdx);
#else
        bool isValidAmMode = checkIsValidMergeMvCand(cs, pu, curPoc, amvpPoc, miNeighbor.refIdx);
#endif
        if (isValidAmMode)
        {
#endif
        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
        // get Mv from Above-Left
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
        mrgCtx.useAltHpelIf[cnt] = miNeighbor.useAltHpelIf;
        // get Mv from Above-right
        mrgCtx.bcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puNonAdjacent->cu->bcwIdx : BCW_DEFAULT;
#if INTER_LIC
#if JVET_AG0276_NLIC
        mrgCtx.licFlags[cnt] = miNeighbor.usesLIC || (puNonAdjacent->cu->altLMFlag && slice.getUseLIC());
#else
        mrgCtx.licFlags[cnt] = miNeighbor.usesLIC;
#endif
#endif
        if (slice.isInterB())
        {
          mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);
#if MULTI_HYP_PRED
          mrgCtx.addHypNeighbours[cnt] = puNonAdjacent->addHypData;
#endif
        }
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
        if (useAmvpMergeMode)
        {
          mrgCtx.interDirNeighbours[cnt] = amvpMergeCtxMergeDir;
          mrgCtx.mvFieldNeighbours[(cnt << 1) + amvpRefList].setMvField(Mv(), -1);
        }
#endif

#if NON_ADJACENT_MRG_CAND || TM_MRG || JVET_AE0046_BI_GPM
        if (!mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG || JVET_AE0046_BI_GPM
                               , mvdSimilarityThresh
#endif
        ))
#endif
        {
          if (mrgCandIdx == cnt)
          {
            return;
          }
          cnt++;
        }
#if JVET_AD0213_LIC_IMP
        else
        {
          mrgCtx.initMrgCand(cnt);
        }
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
        } // if (isValidAmMode)
#endif
      }

    }
  }
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
  }
  else
  {
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
    for (uint32_t ui = 0; ui < namvpMrgCtx->numValidMergeCand && cnt < maxNumMergeCand; ++ui)
#else
    for (uint32_t ui = 0; ui < namvpMrgCtx->numValidMergeCand && cnt < maxNumMergeCand - 1; ++ui)
#endif
    {
      mrgCtx.bcwIdx[cnt] = namvpMrgCtx->bcwIdx[ui];
#if INTER_LIC
      mrgCtx.licFlags[cnt] = namvpMrgCtx->licFlags[ui];
#endif
      mrgCtx.interDirNeighbours[cnt] = namvpMrgCtx->interDirNeighbours[ui];
      mrgCtx.mvFieldNeighbours[cnt << 1] = namvpMrgCtx->mvFieldNeighbours[ui << 1];
      mrgCtx.useAltHpelIf[cnt] = namvpMrgCtx->useAltHpelIf[ui];
      if (slice.isInterB())
      {
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1] = namvpMrgCtx->mvFieldNeighbours[(ui << 1) + 1];
#if MULTI_HYP_PRED
        mrgCtx.addHypNeighbours[cnt] = namvpMrgCtx->addHypNeighbours[ui];
#endif
      }

#if NON_ADJACENT_MRG_CAND || TM_MRG || JVET_AE0046_BI_GPM
      if (!mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG || JVET_AE0046_BI_GPM
        , mvdSimilarityThresh
#endif
      ))
      {
#endif
        mrgCtx.candCost[cnt] = namvpMrgCtx->candCost[ui];
        if (mrgCandIdx == cnt)
        {
          return;
        }

        cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG || JVET_AE0046_BI_GPM
      }
#if JVET_AD0213_LIC_IMP
      else
      {
        mrgCtx.initMrgCand(cnt);
      }
#endif
#endif
    }
  }
#endif
#endif

#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
  int maxNumMergeCandMin1 = maxNumMergeCand;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
  maxNumMergeCandMin1 -= !(pu.cs->sps->getUseAML() && pu.cs->sps->getTMToolsEnableFlag()) ? 1 : 0;
#else
  maxNumMergeCandMin1 -= !pu.cs->sps->getUseAML() ? 1 : 0;
#endif
#endif
#else
  int maxNumMergeCandMin1 = maxNumMergeCand - 1;
#endif
  if (cnt != maxNumMergeCandMin1)
  {
#if !JVET_Z0075_IBC_HMVP_ENLARGE
    bool isGt4x4 = true;
#endif
    bool bFound = addMergeHMVPCand(cs, mrgCtx, mrgCandIdx, maxNumMergeCandMin1, cnt
      , isAvailableA1, miLeft, isAvailableB1, miAbove
#if !JVET_Z0075_IBC_HMVP_ENLARGE
      , CU::isIBC(*pu.cu)
      , isGt4x4
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE || (JVET_Y0058_IBC_LIST_MODIFY && !JVET_Z0075_IBC_HMVP_ENLARGE) || JVET_Z0118_GDR
      , pu
#endif
#if JVET_X0083_BM_AMVP_MERGE_MODE
#if !JVET_Y0128_NON_CTC
      , curPoc
      , amvpPoc
#endif
#endif
#if TM_MRG || JVET_AE0046_BI_GPM
      , mvdSimilarityThresh
#endif
      );

    if (bFound)
    {
      return;
    }
  }

#if !JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC)
  // pairwise-average candidates
  {
    if (cnt > 1 && cnt < maxNumMergeCand
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && !(pu.cs->sps->getUseAML() && pu.cs->sps->getTMToolsEnableFlag())
#else
      && !pu.cs->sps->getUseAML()
#endif
#endif
      )
    {
      mrgCtx.mvFieldNeighbours[cnt * 2].setMvField( Mv( 0, 0 ), NOT_VALID );
      mrgCtx.mvFieldNeighbours[cnt * 2 + 1].setMvField( Mv( 0, 0 ), NOT_VALID );
#if INTER_LIC
      mrgCtx.licFlags[cnt] = false;
#endif
      mrgCtx.bcwIdx[cnt] = BCW_DEFAULT;
#if MULTI_HYP_PRED
      mrgCtx.addHypNeighbours[cnt].clear();
#endif
      // calculate average MV for L0 and L1 seperately
      unsigned char interDir = 0;
#if INTER_LIC
      bool averageUsed = false;
#endif

      mrgCtx.useAltHpelIf[cnt] = (mrgCtx.useAltHpelIf[0] == mrgCtx.useAltHpelIf[1]) ? mrgCtx.useAltHpelIf[0] : false;
      for( int refListId = 0; refListId < (slice.isInterB() ? 2 : 1); refListId++ )
      {
        const short refIdxI = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].refIdx;
        const short refIdxJ = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].refIdx;

        // both MVs are invalid, skip
        if( (refIdxI == NOT_VALID) && (refIdxJ == NOT_VALID) )
        {
          continue;
        }

        interDir += 1 << refListId;
        // both MVs are valid, average these two MVs
        if( (refIdxI != NOT_VALID) && (refIdxJ != NOT_VALID) )
        {
          const Mv& MvI = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].mv;
          const Mv& MvJ = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].mv;

          // average two MVs
          Mv avgMv = MvI;
          avgMv += MvJ;
          roundAffineMv(avgMv.hor, avgMv.ver, 1);

          mrgCtx.mvFieldNeighbours[cnt * 2 + refListId].setMvField( avgMv, refIdxI );
#if INTER_LIC
          mrgCtx.licFlags[cnt] = false;
          averageUsed = true;
#endif
        }
        // only one MV is valid, take the only one MV
        else if( refIdxI != NOT_VALID )
        {
          Mv singleMv = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].mv;
          mrgCtx.mvFieldNeighbours[cnt * 2 + refListId].setMvField( singleMv, refIdxI );
#if INTER_LIC
          if (!averageUsed)
          {
            mrgCtx.licFlags[cnt] |= mrgCtx.licFlags[0];
          }
#endif
        }
        else if( refIdxJ != NOT_VALID )
        {
          Mv singleMv = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].mv;
          mrgCtx.mvFieldNeighbours[cnt * 2 + refListId].setMvField( singleMv, refIdxJ );
#if INTER_LIC
          if (!averageUsed)
          {
            mrgCtx.licFlags[cnt] |= mrgCtx.licFlags[1];
          }
#endif
        }
      }

      mrgCtx.interDirNeighbours[cnt] = interDir;
      if( interDir > 0 )
      {
#if INTER_LIC
#if !JVET_AD0213_LIC_IMP
        if (interDir == 3)
        {
          mrgCtx.licFlags[cnt] = false;
        }
#endif
#endif
#if NON_ADJACENT_MRG_CAND || TM_MRG || JVET_AE0046_BI_GPM
        if( !mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG || JVET_AE0046_BI_GPM
                               , mvdSimilarityThresh
#endif
        ) )
#endif
        cnt++;
#if JVET_AD0213_LIC_IMP
        else
        {
          mrgCtx.initMrgCand(cnt);
        }
#endif
      }
    }

    // early termination
    if( cnt == maxNumMergeCand )
    {
      return;
    }
  }
#else
  // early termination
  if (cnt == maxNumMergeCand)
  {
    return;
  }
#endif

#if JVET_Z0139_HIST_AFF
  if (addMergeHMVPCandFromAffModel(pu, mrgCtx, mrgCandIdx, cnt
#if TM_MRG || JVET_AE0046_BI_GPM
    , mvdSimilarityThresh
#endif
  ))
  {
    return;
  }
#endif

#if JVET_Z0139_HIST_AFF
  int uiArrayAddr = cnt;
#else
  uint32_t uiArrayAddr = cnt;
#endif

#if JVET_Z0102_NO_ARMC_FOR_ZERO_CAND 
  mrgCtx.numCandToTestEnc = cnt;
#endif
   int iNumRefIdx = slice.isInterB() ? (useAmvpMergeMode ? slice.getNumRefIdx(RefPicList(1 - amvpRefList)) : std::min(slice.getNumRefIdx(REF_PIC_LIST_0), slice.getNumRefIdx(REF_PIC_LIST_1))) : slice.getNumRefIdx(REF_PIC_LIST_0);
#if JVET_X0083_BM_AMVP_MERGE_MODE
  if (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])
  {
    int mergeRefIdx = 0;
    const int targetMaxNumMergeCand = mrgCtx.numValidMergeCand;
    mrgCtx.numValidMergeCand = uiArrayAddr;
    while ((mrgCtx.numValidMergeCand < targetMaxNumMergeCand) && (mergeRefIdx < iNumRefIdx))
    {
#if JVET_Y0128_NON_CTC
      if (pu.cu->slice->getAmvpMergeModeValidCandPair(pu.amvpMergeModeFlag[0] ? mergeRefIdx  : pu.refIdx[0],
                                                      pu.amvpMergeModeFlag[0] ? pu.refIdx[1] : mergeRefIdx) == false)
#else
      const int mergePoc = slice.getRefPOC(mergeRefList, mergeRefIdx);
      if ((amvpPoc - curPoc) * (mergePoc - curPoc) > 0)
#endif
      {
        mergeRefIdx++;
        continue;
      }
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
      mrgCtx.interDirNeighbours [ mrgCtx.numValidMergeCand ] = amvpMergeCtxMergeDir;
#else
      mrgCtx.interDirNeighbours [ mrgCtx.numValidMergeCand ] = 3;
#endif
      mrgCtx.bcwIdx             [ mrgCtx.numValidMergeCand ] = BCW_DEFAULT;
#if INTER_LIC
      mrgCtx.licFlags           [ mrgCtx.numValidMergeCand ] = false;
#endif
      mrgCtx.useAltHpelIf       [ mrgCtx.numValidMergeCand ] = false;
#if MULTI_HYP_PRED
      mrgCtx.addHypNeighbours   [ mrgCtx.numValidMergeCand ].clear();
#endif
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
      mrgCtx.mvFieldNeighbours  [(mrgCtx.numValidMergeCand << 1) + (1 - amvpRefList)].setMvField(Mv(0, 0), mergeRefIdx);
      mrgCtx.mvFieldNeighbours  [(mrgCtx.numValidMergeCand << 1) + amvpRefList].setMvField(Mv(0, 0), -1);
#else
      mrgCtx.mvFieldNeighbours  [ mrgCtx.numValidMergeCand << 1].setMvField(Mv(0, 0), mergeRefIdx);
      mrgCtx.mvFieldNeighbours  [(mrgCtx.numValidMergeCand << 1) + 1].setMvField(Mv(0, 0), mergeRefIdx);
#endif
      mrgCtx.numValidMergeCand++;
      mergeRefIdx++;
    }
    return;
  }
#endif

  int r = 0;
  int refcnt = 0;
  while (uiArrayAddr < maxNumMergeCand)
  {
    mrgCtx.interDirNeighbours [uiArrayAddr     ] = 1;
    mrgCtx.bcwIdx             [uiArrayAddr     ] = BCW_DEFAULT;
#if INTER_LIC
    mrgCtx.licFlags           [uiArrayAddr     ] = false;
#endif
#if MULTI_HYP_PRED
    mrgCtx.addHypNeighbours[uiArrayAddr].clear();
#endif
    mrgCtx.mvFieldNeighbours  [uiArrayAddr << 1].setMvField(Mv(0, 0), r);
    mrgCtx.useAltHpelIf[uiArrayAddr] = false;

    if (slice.isInterB())
    {
      mrgCtx.interDirNeighbours [ uiArrayAddr          ] = 3;
      mrgCtx.mvFieldNeighbours  [(uiArrayAddr << 1) + 1].setMvField(Mv(0, 0), r);
    }

    uiArrayAddr++;

    if (refcnt == iNumRefIdx - 1)
    {
      r = 0;
    }
    else
    {
      ++r;
      ++refcnt;
    }

#if JVET_Z0139_HIST_AFF
    if (uiArrayAddr < maxNumMergeCand && uiArrayAddr == cnt + 1)
    {
      if (addMergeHMVPCandFromAffModel(pu, mrgCtx, mrgCandIdx, uiArrayAddr
#if TM_MRG || JVET_AE0046_BI_GPM
        , mvdSimilarityThresh
#endif
      ))
      {
        return;
      }
      if (uiArrayAddr >= mrgCtx.numValidMergeCand)
      {
        return;
      }
    }
#endif
  }
#if NON_ADJACENT_MRG_CAND
  CHECK(mrgCtx.numValidMergeCand != uiArrayAddr, "not enough number of merge candidates!");
#else
  mrgCtx.numValidMergeCand = uiArrayAddr;
#endif
}

#if JVET_Y0128_NON_CTC
bool  PU::isBiRefScaled(const CodingStructure& cs, const int refIdx0, const int refIdx1)
{
  const bool isResamplingPossible = cs.sps->getRprEnabledFlag();
  if (!isResamplingPossible)
  {
    return false;
  }

  const bool ref0IsScaled = refIdx0 < 0 || refIdx0 >= MAX_NUM_REF
    ? false
    : isResamplingPossible && cs.slice->getRefPic(REF_PIC_LIST_0, refIdx0)->isRefScaled(cs.pps);
  const bool ref1IsScaled = refIdx1 < 0 || refIdx1 >= MAX_NUM_REF
    ? false
    : isResamplingPossible && cs.slice->getRefPic(REF_PIC_LIST_1, refIdx1)->isRefScaled(cs.pps);

  return (ref0IsScaled || ref1IsScaled);
}
#endif

#if JVET_X0049_ADAPT_DMVR
bool PU::isBMMergeFlagCoded(const PredictionUnit& pu)
{

  if (pu.cs->slice->getSPS()->getUseDMVDMode()
#if JVET_Y0128_NON_CTC
    && pu.cs->slice->getUseBM()
#else
    && !pu.cs->slice->getCheckLDC()
#endif
    )
  {
    return (pu.cs->sps->getMaxNumBMMergeCand() > 0);
  }
  return false;
}

bool PU::isBiPredFromDifferentDirEqDistPoc(const PredictionUnit& pu, int refIdx0, int refIdx1)
{
  if (refIdx0 >= 0 && refIdx1 >= 0)
  {
    if (pu.cu->slice->getRefPic(REF_PIC_LIST_0, refIdx0)->longTerm
      || pu.cu->slice->getRefPic(REF_PIC_LIST_1, refIdx1)->longTerm)
    {
      return false;
    }
#if JVET_Y0128_NON_CTC
    if ( PU::isBiRefScaled( *pu.cs, refIdx0, refIdx1 ) )
    {
      return false;
    }
#endif
    const int poc0 = pu.cu->slice->getRefPOC(REF_PIC_LIST_0, refIdx0);
    const int poc1 = pu.cu->slice->getRefPOC(REF_PIC_LIST_1, refIdx1);
    const int poc = pu.cu->slice->getPOC();
    if ((poc - poc0)*(poc - poc1) < 0)
    {
      if (abs(poc - poc0) == abs(poc - poc1))
      {
        return true;
      }
    }
  }
  return false;
}

bool PU::addBMMergeHMVPCand(const CodingStructure &cs, MergeCtx &mrgCtx, const int &mrgCandIdx,
  const uint32_t maxNumMergeCandMin1, int &cnt, const bool isAvailableA1,
  const MotionInfo miLeft, const bool isAvailableB1, const MotionInfo miAbove
#if !JVET_Z0075_IBC_HMVP_ENLARGE
  , const bool ibcFlag, const bool isGt4x4
#endif
#if TM_MRG
  , const uint32_t mvdSimilarityThresh
#endif
)
{
  const Slice& slice = *cs.slice;
  MotionInfo miNeighbor;
#if JVET_Z0118_GDR
  bool isClean = cs.isClean(cs.area.Y().bottomRight(), CHANNEL_TYPE_LUMA);
#endif

#if JVET_Z0075_IBC_HMVP_ENLARGE
#if JVET_Z0118_GDR
  auto &lut = (isClean) ? cs.motionLut.lut1 : cs.motionLut.lut0;
#else
  auto &lut = cs.motionLut.lut;
#endif
#else
#if JVET_Z0118_GDR
  auto &lut = ibcFlag ? (isClean ? cs.motionLut.lutIbc1 : cs.motionLut.lutIbc0) : (isClean ? cs.motionLut.lut1 : cs.motionLut.lut0);
#else
  auto &lut = ibcFlag ? cs.motionLut.lutIbc : cs.motionLut.lut;
#endif
#endif

  int numCandInLUT = (int)lut.size();

  for (int mrgIdx = 1; mrgIdx <= numCandInLUT; mrgIdx++)
  {
    miNeighbor = lut[numCandInLUT - mrgIdx];

#if JVET_Z0075_IBC_HMVP_ENLARGE
    if ( mrgIdx > 2
#else
    if (mrgIdx > 2 || ((mrgIdx > 1 || !isGt4x4) && ibcFlag)
#endif
      || ((!isAvailableA1 || (miLeft != miNeighbor)) && (!isAvailableB1 || (miAbove != miNeighbor))))
    {
#if INTER_LIC
#if JVET_AD0213_LIC_IMP
      mrgCtx.licFlags[cnt] = false;
#else
      mrgCtx.licFlags[cnt] = miNeighbor.usesLIC;
#endif
#if !JVET_Z0075_IBC_HMVP_ENLARGE
      if (ibcFlag)
      {
        CHECK(mrgCtx.licFlags[cnt], "addMergeHMVPCand: LIC is not used with IBC mode")
      }
#endif
#endif

      bool isBmCand = false;
      int refIdx0 = miNeighbor.refIdx[0];
      int refIdx1 = miNeighbor.refIdx[1];
      if (refIdx0 >= 0 && refIdx1 >= 0 
#if !JVET_Y0089_DMVR_BCW
        && miNeighbor.bcwIdx == BCW_DEFAULT
#endif
#if MULTI_HYP_PRED
        && miNeighbor.addHypData.empty()
#endif
        )
      {
        if (cs.slice->getRefPic(REF_PIC_LIST_0, refIdx0)->longTerm
          || cs.slice->getRefPic(REF_PIC_LIST_1, refIdx1)->longTerm)
        {
          isBmCand = false;
        }
        const int poc0 = cs.slice->getRefPOC(REF_PIC_LIST_0, refIdx0);
        const int poc1 = cs.slice->getRefPOC(REF_PIC_LIST_1, refIdx1);
        const int poc = cs.slice->getPOC();
        if ((poc - poc0)*(poc - poc1) < 0)
        {
          if (abs(poc - poc0) == abs(poc - poc1))
          {
            isBmCand = true;
          }
        }
#if JVET_Y0128_NON_CTC
        if ( isBmCand && PU::isBiRefScaled( cs, refIdx0, refIdx1 ) )
        {
          isBmCand = false;
        }
#endif
      }
      if (!isBmCand)
      {
        continue;
      }
      mrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
#if JVET_Z0075_IBC_HMVP_ENLARGE
      mrgCtx.useAltHpelIf      [cnt] = miNeighbor.useAltHpelIf;
#else
      mrgCtx.useAltHpelIf[cnt] = !ibcFlag && miNeighbor.useAltHpelIf;
#endif

      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
      if (slice.isInterB())
      {
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);
#if JVET_Y0089_DMVR_BCW
        mrgCtx.bcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? miNeighbor.bcwIdx : BCW_DEFAULT;
#endif
      }
#if NON_ADJACENT_MRG_CAND
      if (mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG
        , mvdSimilarityThresh
#endif
      ))
      {
#if JVET_AD0213_LIC_IMP
        mrgCtx.initMrgCand(cnt);
#endif
        continue;
      }
#endif
      if (mrgCandIdx == cnt)
      {
        return true;
      }
      cnt++;

      if (cnt == maxNumMergeCandMin1)
      {
        break;
      }
    }
  }

  if (cnt < maxNumMergeCandMin1)
  {
    mrgCtx.useAltHpelIf[cnt] = false;
  }
  return false;
}
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
void computeDeltaAndShift(const Position posLT, Mv firstMv, std::vector<RMVFInfo> &mvpInfoVecOri)
{
  g_pelBufOP.computeDeltaAndShift(posLT, firstMv, mvpInfoVecOri);
}
void computeDeltaAndShiftAddi(const Position posLT, Mv firstMv, std::vector<RMVFInfo> &mvpInfoVecOri, std::vector<RMVFInfo> &mvpInfoVecRes)
{
  g_pelBufOP.computeDeltaAndShiftAddi(posLT, firstMv, mvpInfoVecOri, mvpInfoVecRes);
}
void buildRegressionMatrix(std::vector<RMVFInfo> &mvpInfoVecOri, 
#if JVET_AA0107_RMVF_AFFINE_OVERFLOW_FIX || JVET_AB0189_RMVF_BITLENGTH_CONTROL
  int64_t sumbb[2][3][3], int64_t sumeb[2][3],
#else
  int sumbb[2][3][3], int sumeb[2][3],
#endif
  uint16_t addedSize)
{
  g_pelBufOP.buildRegressionMatrix(mvpInfoVecOri, sumbb, sumeb, addedSize);
}
#if !JVET_AB0189_RMVF_BITLENGTH_CONTROL
int64_t det(int n, int64_t mat[3][3]) // n:the actual size of "mat"
{
  int p[2] = { 1, -1 };
  int64_t d = 0;
  int64_t submat[3][3];
  if (n == 2)
  {
    return((mat[0][0] * mat[1][1]) - (mat[1][0] * mat[0][1]));
  }
  else
  {
    for (int c = 0; c < n; c++)
    {
      int subi = 0;   //submatrix's i value
      for (int i = 1; i < n; i++)
      {
        int subj = 0;
        for (int j = 0; j < n; j++)
        {
          if (j == c)
          {
            continue;
          }
          submat[subi][subj] = mat[i][j];
          subj++;
        }
        subi++;
      }
      d += (p[c % 2] * mat[0][c] * det(n - 1, submat));
    }
  }
  return d;
}
#endif
static int getRMVFMSB(int64_t x)
{
  int msb = 0, bits = (sizeof(int64_t) << 3);
  int64_t y = 1;
  while (x > 1)
  {
    bits >>= 1;
    y = x >> bits;
    if (y)
    {
      x = y;
      msb += bits;
    }
  }
  msb += (int)y;
  return msb;
}

int64_t divideRMVF(int64_t numer, int64_t denom
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
  , int scale
  , bool isoffset
#endif
) // out = numer/denom
{
  if (numer == 0 || denom == 1)
  {
    return numer;
  }
  int64_t d;
  const int64_t iShiftA2 = 6;
  const int64_t iAccuracyShift = 15;
  const int64_t iMaxVal = 63;
  int64_t iScaleShiftA2 = 0;
  int64_t iScaleShiftA1 = 0;

  uint8_t signA1 = numer < 0;
  uint8_t signA2 = denom < 0;

  numer = (signA1) ? -numer : numer;
  denom = (signA2) ? -denom : denom;
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
  if (isoffset)
  {
    denom = denom >> (3 * scale);
    if (!denom)
    {
      return numer;
    }
  }
#endif
  iScaleShiftA2 = getRMVFMSB(denom) - iShiftA2;
#if !JVET_AB0189_RMVF_BITLENGTH_CONTROL
  iScaleShiftA1 = iScaleShiftA2 - 12;

  if (iScaleShiftA1 < 0)
  {
    iScaleShiftA1 = 0;
  }
#endif

  if (iScaleShiftA2 < 0)
  {
    iScaleShiftA2 = 0;
  }
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
  int numerBitlength = getRMVFMSB(abs(numer));
  if (numerBitlength > 48)
  {
    iScaleShiftA1 = numerBitlength - 48;
  }
#endif
  int64_t iScaleShiftA = iScaleShiftA2 + iAccuracyShift - iScaleShiftA1;

  int64_t a2s = (denom >> iScaleShiftA2) > iMaxVal ? iMaxVal : (denom >> iScaleShiftA2);
  int64_t a1s = (numer >> iScaleShiftA1);

#if JVET_AA0107_RMVF_AFFINE_OVERFLOW_FIX
  uint64_t aI64 = ((uint64_t)a1s * (uint64_t)g_rmvfMultApproxTbl[a2s]) >> iScaleShiftA;
  aI64 = aI64 > INT64_MAX ? INT64_MAX : aI64;
  d = (signA1 + signA2 == 1) ? -(int64_t)aI64 : (int64_t)aI64;
#else
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
  int64_t aI64 = (a1s * (int64_t)g_rmvfMultApproxTbl[a2s]) >> (iScaleShiftA - 8 < 0 ? 0 : iScaleShiftA - 8);
#else
  int64_t aI64 = (a1s * (int64_t)g_rmvfMultApproxTbl[a2s]) >> iScaleShiftA;
#endif

  d = (signA1 + signA2 == 1) ? -aI64 : aI64;
#endif
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
  return Clip3(int64_t(-RMVF_PARAM_THRED), int64_t(RMVF_PARAM_THRED - 1), d);
#else
  return d;
#endif
}
void PU::xCalcRMVFParameters(std::vector<RMVFInfo> &mvpInfoVec, int64_t dMatrix[2][4],
#if JVET_AA0107_RMVF_AFFINE_OVERFLOW_FIX || JVET_AB0189_RMVF_BITLENGTH_CONTROL
  int64_t sumbbfinal[2][3][3], int64_t sumebfinal[2][3],
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
  uint8_t shift,
#endif
#else
  int sumbbfinal[2][3][3], int sumebfinal[2][3],
#endif
  uint16_t addedSize)
{
#if !JVET_AB0189_RMVF_BITLENGTH_CONTROL
#if JVET_AA0107_RMVF_AFFINE_OVERFLOW_FIX
  int shift = 0;
#else
  int shift = 1;
#endif
#endif
  int iNum = int(mvpInfoVec.size());
#if JVET_AA0107_RMVF_AFFINE_OVERFLOW_FIX || JVET_AB0189_RMVF_BITLENGTH_CONTROL
#if !JVET_AB0189_RMVF_BITLENGTH_CONTROL
  int shiftDets = 0;
  int64_t m[3][3]; // parameter=det(md)/det(m)
#else
  uint8_t initShift = 3 * shift;
  uint8_t rightShift = 0;
#endif
  int64_t sumbb[2][3][3];
  int64_t sumeb[2][3];
#else
  int shiftDets = 5 * (getRMVFMSB(iNum) - 4);
  if (shiftDets < 0) shiftDets = 0;
  int sumbb[2][3][3];
  int sumeb[2][3];
#endif
  int64_t md[3][3];
  ////////////////// Extract statistics: Start
  //initialize values to zero
  if (!addedSize)
  {
    for (int c = 0; c < 2; c++)
    {
      for (int d = 0; d < 3; d++)
      {
        sumebfinal[c][d] = 0;
      }
      for (int d1 = 0; d1 < 3; d1++)
      {
        for (int d = 0; d < 3; d++)
        {
          sumbbfinal[c][d1][d] = 0;
        }
      }
    }
  }
  buildRegressionMatrix(mvpInfoVec, sumbbfinal, sumebfinal, addedSize);

  for (int c = 0; c < 2; c++)
  {
    for (int d = 0; d < 3; d++)
    {
#if JVET_AA0107_RMVF_AFFINE_OVERFLOW_FIX
      sumeb[c][d] = divideRMVF(sumebfinal[c][d], iNum);
#else
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
      sumeb[c][d] = sumebfinal[c][d];
#else
      sumeb[c][d] = sumebfinal[c][d] >> shift;
#endif
#endif
    }
    for (int d1 = 0; d1 < 3; d1++)
    {
      for (int d = 0; d < 3; d++)
      {
#if JVET_AA0107_RMVF_AFFINE_OVERFLOW_FIX
        sumbb[c][d1][d] = divideRMVF(sumbbfinal[c][d1][d], iNum);
#else
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
        sumbb[c][d1][d] = sumbbfinal[c][d1][d];
#else
        sumbb[c][d1][d] = sumbbfinal[c][d1][d] >> shift;
#endif
#endif
      }
    }
  }
#if JVET_AA0107_RMVF_AFFINE_OVERFLOW_FIX
  sumbb[0][2][2] = 1;
  sumbb[1][2][2] = 1;
  iNum = 1;
#endif
  ////////////////// Extract statistics: End
  ////////////////// Extract Weight: Start
#if !JVET_AB0189_RMVF_BITLENGTH_CONTROL
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      m[i][j] = sumbb[0][i][j];
    }
  }
#endif
  bool bBadMatrix = false;
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
  int64_t det1 = 0;
  int64_t det2 = (sumbb[0][2][0]) * ((sumbb[0][0][1] * sumbb[0][1][2] - sumbb[0][1][1] * sumbb[0][0][2]))
    - (sumbb[0][2][1]) * ((sumbb[0][0][0] * sumbb[0][1][2] - sumbb[0][1][0] * sumbb[0][0][2]))
    + (sumbb[0][2][2]) * ((sumbb[0][0][0] * sumbb[0][1][1] - sumbb[0][1][0] * sumbb[0][0][1]));
#else
  int64_t det2 = det(3, m);
  det2 >>= shiftDets;
#endif
  if (det2 == 0)
  {
    bBadMatrix = true;
  }
  for (int c = 0; c < 2; c++)
  {
    // 1) find matrix parameters with cross-component model
    // Find w[c][d]
#if !JVET_AB0189_RMVF_BITLENGTH_CONTROL
    int64_t dets[3];
#endif
    if (!bBadMatrix)
    {
      for (int d = 0; d < 3; d++)
      {
        // Initialize md matrix
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
        rightShift = d == 2 ? initShift : 0;
#endif
        for (int i = 0; i < 3; i++)
        {
          for (int j = 0; j < 3; j++)
          {
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
            md[i][j] = sumbb[0][i][j];
#else
            md[i][j] = m[i][j];
#endif
          }
        }
        // Replace coloumn D in md matrix
        for (int j = 0; j < 3; j++)
        {
          md[j][d] = sumeb[c][j];
        }
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
        det1 = (md[2][0]) * ((md[0][1] * md[1][2] - md[1][1] * md[0][2]) >> rightShift)
          - (md[2][1]) * ((md[0][0] * md[1][2] - md[1][0] * md[0][2]) >> rightShift)
          + (md[2][2]) * ((md[0][0] * md[1][1] - md[1][0] * md[0][1]) >> rightShift);
        dMatrix[c][d] = det1;
#else
        int64_t det1 = det(3, md);
        det1 >>= shiftDets;
        dets[d] = det1;
        dMatrix[c][d] = iNum * det1;
#endif

        // calcualte offset
        if (d == 2)
        {
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
          dMatrix[c][3] = det2;
#else
          dMatrix[c][2] = (det2 * sumeb[c][2] - dets[0] * sumbb[c][0][2] - dets[1] * sumbb[c][1][2]) << shift;
          dMatrix[c][3] = iNum * det2;
#endif
        }
      } // for d
    }

    if (bBadMatrix)
    {
      // 2) Find simple weight and offset, with non-corss component between x and y, i.e.: MVx=weight[0]*x+offset[0], MVy=weight[1]*y+offset[1]
      for (int d = 0; d < 3; d++)
      {
        dMatrix[c][d] = 0;
      }
      int64_t det1 = (iNum * sumeb[c][c] - sumbb[c][c][2] * sumeb[c][2]);
      int64_t det2 = (iNum * sumbb[c][c][c] - sumbb[c][c][2] * sumbb[c][c][2]);
      if (det2 == 0)
      {
        dMatrix[c][c] = 0;
        det1 = 0;
        det2 = 1;
        dMatrix[c][3] = iNum;
      }
      else
      {
        dMatrix[c][c] = iNum * det1;
        dMatrix[c][3] = iNum * det2;
      }
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
      dMatrix[c][2] = det2 * sumeb[c][2] - det1 * sumbb[c][c][2];
#else
      dMatrix[c][2] = (det2 * sumeb[c][2] - det1 * sumbb[c][c][2]) << shift;
#endif
    } // End "bBadMatrix"

  } // end "for c"
    ////////////////// Extract Weights: End
  return;
}
void PU::getRMVFAffineGuideCand(const PredictionUnit &pu, const PredictionUnit &abovePU, AffineMergeCtx &affMrgCtx
  , std::vector<RMVFInfo> mvp[NUM_REF_PIC_LIST_01][MAX_NUM_REF]
  , int mrgCandIdx
#if JVET_AG0276_NLIC
  , AltLMAffineMergeCtx* altAffineMergeCtx
#if JVET_AG0276_LIC_FLAG_SIGNALING
  , AltLMAffineMergeCtx* altBRAffineMergeCtx
#endif
#endif
)
{
  const CodingStructure &cs = *pu.cs;

  int iNumPredDir = cs.slice->isInterP() ? 1 : 2;
  Mv cMV[2][3];
  Mv cMVOri[2][3];

  std::vector<RMVFInfo> mvpInfoVec[2];
  std::vector<RMVFInfo> mvpInfoVecOri;

  bool available[2] = { false, false };
  //-- Collect non-adj affine subblock info
  Position anchorPosAbove = abovePU.Y().topLeft();
  Position neibPos;
  int stepsize = 1 << ATMVP_SUB_BLOCK_SIZE;
  int hSearchAbove = abovePU.Y().width;
  int vSearchAbove = abovePU.Y().height;
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
  size_t blkCnt = 0;
  size_t thred[2];

  const Position anchorPos = pu.Y().topLeft();
  bool enoughBlk[2] = { false, false };
  int startX = anchorPos.x - anchorPosAbove.x > RMVF_DISTANCE_THRED ? anchorPos.x - RMVF_DISTANCE_THRED - anchorPosAbove.x : 0;
  int endX = abovePU.Y().topRight().x - anchorPos.x > RMVF_DISTANCE_THRED ? hSearchAbove - (abovePU.Y().topRight().x - anchorPos.x - RMVF_DISTANCE_THRED) - 1 : hSearchAbove;
  int startY = anchorPos.y - anchorPosAbove.y > RMVF_DISTANCE_THRED ? anchorPos.y - RMVF_DISTANCE_THRED - anchorPosAbove.y : 0;
  int endY = abovePU.Y().bottomLeft().y - anchorPos.y > RMVF_DISTANCE_THRED ? vSearchAbove - (abovePU.Y().bottomLeft().y - anchorPos.y - RMVF_DISTANCE_THRED) - 1 : vSearchAbove;
  bool outsideX = anchorPos.x - abovePU.Y().topRight().x > RMVF_DISTANCE_THRED || anchorPosAbove.x - anchorPos.x >= RMVF_DISTANCE_THRED;
  bool outsideY = anchorPos.y - abovePU.Y().bottomLeft().y > RMVF_DISTANCE_THRED || anchorPosAbove.y - anchorPos.y >= RMVF_DISTANCE_THRED;
  PredictionUnit tempPU = abovePU;
  MotionBuf mb = tempPU.getMotionBuf();
  MotionInfo *mi = mb.buf;
  mi += (startY >> MIN_CU_LOG2) * mb.stride;
  if (!outsideX && !outsideY)
  {
    if (abovePU.interDir & 1)
    {
      thred[0] = RMVF_NUM_SUBBLK_THRED - mvp[REF_PIC_LIST_0][abovePU.refIdx[REF_PIC_LIST_0]].size();
      for (int i = startY; i < endY; i += stepsize)
      {
        for (int j = startX; j < endX; j += stepsize)
        {
          neibPos = anchorPosAbove.offset(j + 2, i + 2);
          mvpInfoVec[REF_PIC_LIST_0].push_back(RMVFInfo(mi[j >> MIN_CU_LOG2].mv[REF_PIC_LIST_0], neibPos, -1));
          blkCnt++;
          if (blkCnt == thred[0])
          {
            break;
          }
        }
        if (blkCnt == thred[0])
        {
          break;
        }
        mi += mb.stride;
      }
    }
    blkCnt = 0;
    mi = mb.buf;
    mi += (startY >> MIN_CU_LOG2) * mb.stride;
    if (abovePU.interDir & 2)
    {
      thred[1] = RMVF_NUM_SUBBLK_THRED - mvp[REF_PIC_LIST_1][abovePU.refIdx[REF_PIC_LIST_1]].size();
      for (int i = startY; i < endY; i += stepsize)
      {
        for (int j = startX; j < endX; j += stepsize)
        {
          neibPos = anchorPosAbove.offset(j + 2, i + 2);
          mvpInfoVec[REF_PIC_LIST_1].push_back(RMVFInfo(mi[j >> MIN_CU_LOG2].mv[REF_PIC_LIST_1], neibPos, -1));
          blkCnt++;
          if (blkCnt == thred[1])
          {
            break;
          }
        }
        if (blkCnt == thred[1])
        {
          break;
        }
        mi += mb.stride;
      }
    }
  }
#else
  if (abovePU.interDir == 3)
  {
    for (int i = 0; i < vSearchAbove; i += stepsize)
    {
      for (int j = 0; j < hSearchAbove; j += stepsize)
      {
        neibPos = anchorPosAbove.offset(j + 2, i + 2);
        const MotionInfo& neibMi = abovePU.getMotionInfo(neibPos);

        mvpInfoVec[REF_PIC_LIST_0].push_back(RMVFInfo(neibMi.mv[REF_PIC_LIST_0], neibPos, -1));
        mvpInfoVec[REF_PIC_LIST_1].push_back(RMVFInfo(neibMi.mv[REF_PIC_LIST_1], neibPos, -1));
      }
    }
  }
  else if (abovePU.interDir == 1)
  {
    for (int i = 0; i < vSearchAbove; i += stepsize)
    {
      for (int j = 0; j < hSearchAbove; j += stepsize)
      {
        neibPos = anchorPosAbove.offset(j + 2, i + 2);
        const MotionInfo& neibMi = abovePU.getMotionInfo(neibPos);

        mvpInfoVec[REF_PIC_LIST_0].push_back(RMVFInfo(neibMi.mv[REF_PIC_LIST_0], neibPos, -1));
      }
    }
  }
  else if (abovePU.interDir == 2)
  {
    for (int i = 0; i < vSearchAbove; i += stepsize)
    {
      for (int j = 0; j < hSearchAbove; j += stepsize)
      {
        neibPos = anchorPosAbove.offset(j + 2, i + 2);
        const MotionInfo& neibMi = abovePU.getMotionInfo(neibPos);

        mvpInfoVec[REF_PIC_LIST_1].push_back(RMVFInfo(neibMi.mv[REF_PIC_LIST_1], neibPos, -1));
      }
    }
  }
#endif
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
  if (mvpInfoVec[REF_PIC_LIST_0].size() > 3)
  {
    enoughBlk[0] = true;
  }
  if (mvpInfoVec[REF_PIC_LIST_1].size() > 3)
  {
    enoughBlk[1] = true;
  }
#endif
  for (unsigned int list = 0; list < iNumPredDir; ++list)
  {
    RefPicList eRefPicList = list == 0 ? REF_PIC_LIST_0 : REF_PIC_LIST_1;
    if (abovePU.interDir == 1 && eRefPicList == REF_PIC_LIST_1)
    {
      continue;
    }
    if (abovePU.interDir == 2 && eRefPicList == REF_PIC_LIST_0)
    {
      continue;
    }
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
    if (!enoughBlk[eRefPicList])
    {
      continue;
    }
#endif
    const Position posLT = pu.Y().topLeft();

    int64_t cMvX = 0, cMvY = 0;
    Mv cMv;
    Mv firstMv;
    int64_t parametersRMVF[2][4];
    firstMv.set(mvpInfoVec[eRefPicList][0].mvp.getHor(), mvpInfoVec[eRefPicList][0].mvp.getVer());
#if !JVET_AB0189_RMVF_BITLENGTH_CONTROL
    firstMv.hor = firstMv.hor >= 0 ? firstMv.hor << 2 : -(-firstMv.hor << 2);
    firstMv.ver = firstMv.ver >= 0 ? firstMv.ver << 2 : -(-firstMv.ver << 2);
#endif
    computeDeltaAndShift(posLT, firstMv, mvpInfoVec[eRefPicList]);

    //-- Model with Linear Regression
    //-- Calculate RMVF parameters:
#if JVET_AA0107_RMVF_AFFINE_OVERFLOW_FIX || JVET_AB0189_RMVF_BITLENGTH_CONTROL
    int64_t sumbb[2][3][3];
    int64_t sumeb[2][3];
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
    int denumShift = (getRMVFMSB(mvpInfoVec[eRefPicList].size()) - 5);
    if (denumShift < 0)
    {
      denumShift = 0;
    }
#endif
#else
    int sumbb[2][3][3];
    int sumeb[2][3];
#endif


    // top-left CPMV
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
    xCalcRMVFParameters(mvpInfoVec[eRefPicList], parametersRMVF, sumbb, sumeb, denumShift, 0);
    for (int i = 0; i < 2; i++)
    {
      parametersRMVF[0][i] = divideRMVF(parametersRMVF[0][i], parametersRMVF[0][3], denumShift, false);
      parametersRMVF[1][i] = divideRMVF(parametersRMVF[1][i], parametersRMVF[1][3], denumShift, false);
    }
    parametersRMVF[0][2] = divideRMVF(parametersRMVF[0][2], parametersRMVF[0][3], denumShift, true);
    parametersRMVF[1][2] = divideRMVF(parametersRMVF[1][2], parametersRMVF[1][3], denumShift, true);
    cMvX = parametersRMVF[0][2];
    cMvY = parametersRMVF[1][2];
    cMvX = (firstMv.getHor() << MAX_CU_DEPTH) + cMvX;
    cMvY = (firstMv.getVer() << MAX_CU_DEPTH) + cMvY;
    int iMvX = int(cMvX);
    int iMvY = int(cMvY);
    roundAffineMv(iMvX, iMvY, MAX_CU_DEPTH);
    cMv = Mv(iMvX, iMvY);
#else
    xCalcRMVFParameters(mvpInfoVec[eRefPicList], parametersRMVF, sumbb, sumeb, 0);
    cMvX = parametersRMVF[0][2];
    cMvY = parametersRMVF[1][2];
    cMvX = divideRMVF(cMvX, parametersRMVF[0][3]);
    cMvY = divideRMVF(cMvY, parametersRMVF[1][3]);
    cMvX += firstMv.getHor();
    cMvY += firstMv.getVer();
    cMv = Mv((int)(cMvX), (int)(cMvY));
    cMv.hor = cMv.hor >= 0 ? (cMv.hor + 1) >> 2 : (cMv.hor + 2) >> 2;
    cMv.ver = cMv.ver >= 0 ? (cMv.ver + 1) >> 2 : (cMv.ver + 2) >> 2;
#endif
    cMVOri[list][0] = cMv;
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
    cMVOri[list][0].clipToStorageBitDepth();
#endif

    // top-right CPMV
    cMvX = parametersRMVF[0][0] * pu.lumaSize().width + parametersRMVF[0][2];
    cMvY = parametersRMVF[1][0] * pu.lumaSize().width + parametersRMVF[1][2];
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
    cMvX = (firstMv.getHor() << MAX_CU_DEPTH) + cMvX;
    cMvY = (firstMv.getVer() << MAX_CU_DEPTH) + cMvY;
    iMvX = int(cMvX);
    iMvY = int(cMvY);
    roundAffineMv(iMvX, iMvY, MAX_CU_DEPTH);
    cMv = Mv(iMvX, iMvY);
#else
    cMvX = divideRMVF(cMvX, parametersRMVF[0][3]);
    cMvY = divideRMVF(cMvY, parametersRMVF[1][3]);
    cMvX += firstMv.getHor();
    cMvY += firstMv.getVer();
    cMv = Mv((int)(cMvX), (int)(cMvY));
    cMv.hor = cMv.hor >= 0 ? (cMv.hor + 1) >> 2 : (cMv.hor + 2) >> 2;
    cMv.ver = cMv.ver >= 0 ? (cMv.ver + 1) >> 2 : (cMv.ver + 2) >> 2;
#endif

    cMVOri[list][1] = cMv;
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
    cMVOri[list][1].clipToStorageBitDepth();
#endif


    // bottom-left CPMV
    cMvX = parametersRMVF[0][1] * pu.lumaSize().height + parametersRMVF[0][2];
    cMvY = parametersRMVF[1][1] * pu.lumaSize().height + parametersRMVF[1][2];

#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
    cMvX = (firstMv.getHor() << MAX_CU_DEPTH) + cMvX;
    cMvY = (firstMv.getVer() << MAX_CU_DEPTH) + cMvY;
    iMvX = int(cMvX);
    iMvY = int(cMvY);
    roundAffineMv(iMvX, iMvY, MAX_CU_DEPTH);
    cMv = Mv(iMvX, iMvY);
#else
    cMvX = divideRMVF(cMvX, parametersRMVF[0][3]);
    cMvY = divideRMVF(cMvY, parametersRMVF[1][3]);
    cMvX += firstMv.getHor();
    cMvY += firstMv.getVer();
    cMv = Mv((int)(cMvX), (int)(cMvY));
    cMv.hor = cMv.hor >= 0 ? (cMv.hor + 1) >> 2 : (cMv.hor + 2) >> 2;
    cMv.ver = cMv.ver >= 0 ? (cMv.ver + 1) >> 2 : (cMv.ver + 2) >> 2;
#endif

    cMVOri[list][2] = cMv;
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
    cMVOri[list][2].clipToStorageBitDepth();
#endif
    int refIdx = 0;

    refIdx = abovePU.refIdx[eRefPicList];
    if ((int)mvp[eRefPicList][refIdx].size() < 3)
    {
      available[eRefPicList] = true;
      cMV[list][0] = cMVOri[list][0];
      cMV[list][1] = cMVOri[list][1];
      cMV[list][2] = cMVOri[list][2];
      mvpInfoVec[eRefPicList].clear();
      mvpInfoVecOri.clear();
      continue;
    }
    for (int i = 0; i < (int)(mvp[eRefPicList][refIdx]).size(); i++)
    {
      mvpInfoVecOri.push_back(mvp[eRefPicList][refIdx][i]);
    }

    uint16_t addedSize = (uint16_t)mvpInfoVecOri.size();

    computeDeltaAndShiftAddi(posLT, firstMv, mvpInfoVecOri, mvpInfoVec[eRefPicList]);
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
    denumShift = (getRMVFMSB(mvpInfoVec[eRefPicList].size()) - 5);
    if (denumShift < 0)
    {
      denumShift = 0;
    }
#endif
    //-- Model with Linear Regression
    //-- Calculate RMVF parameters:
    //-- Generate prediction signal 
    cMvX = 0, cMvY = 0;
    // top-left CPMV
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
    xCalcRMVFParameters(mvpInfoVec[eRefPicList], parametersRMVF, sumbb, sumeb, denumShift, addedSize);
    for (int i = 0; i < 2; i++)
    {
      parametersRMVF[0][i] = divideRMVF(parametersRMVF[0][i], parametersRMVF[0][3], denumShift, false);
      parametersRMVF[1][i] = divideRMVF(parametersRMVF[1][i], parametersRMVF[1][3], denumShift, false);
    }
    parametersRMVF[0][2] = divideRMVF(parametersRMVF[0][2], parametersRMVF[0][3], denumShift, true);
    parametersRMVF[1][2] = divideRMVF(parametersRMVF[1][2], parametersRMVF[1][3], denumShift, true);
    cMvX = parametersRMVF[0][2];
    cMvY = parametersRMVF[1][2];
    cMvX = (firstMv.getHor() << MAX_CU_DEPTH) + cMvX;
    cMvY = (firstMv.getVer() << MAX_CU_DEPTH) + cMvY;
    iMvX = int(cMvX);
    iMvY = int(cMvY);
    roundAffineMv(iMvX, iMvY, MAX_CU_DEPTH);
    cMv = Mv(iMvX, iMvY);
#else
    xCalcRMVFParameters(mvpInfoVec[eRefPicList], parametersRMVF, sumbb, sumeb, addedSize);
    cMvX = parametersRMVF[0][2];
    cMvY = parametersRMVF[1][2];
    cMvX = divideRMVF(cMvX, parametersRMVF[0][3]);
    cMvY = divideRMVF(cMvY, parametersRMVF[1][3]);
    cMvX += firstMv.getHor();
    cMvY += firstMv.getVer();
    cMv = Mv((int)(cMvX), (int)(cMvY));
    cMv.hor = cMv.hor >= 0 ? (cMv.hor + 1) >> 2 : (cMv.hor + 2) >> 2;
    cMv.ver = cMv.ver >= 0 ? (cMv.ver + 1) >> 2 : (cMv.ver + 2) >> 2;
#endif
    cMV[list][0] = cMv;
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
    cMV[list][0].clipToStorageBitDepth();
#endif

    // top-right CPMV
    cMvX = parametersRMVF[0][0] * pu.lumaSize().width + parametersRMVF[0][2];
    cMvY = parametersRMVF[1][0] * pu.lumaSize().width + parametersRMVF[1][2];

#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
    cMvX = (firstMv.getHor() << MAX_CU_DEPTH) + cMvX;
    cMvY = (firstMv.getVer() << MAX_CU_DEPTH) + cMvY;
    iMvX = int(cMvX);
    iMvY = int(cMvY);
    roundAffineMv(iMvX, iMvY, MAX_CU_DEPTH);
    cMv = Mv(iMvX, iMvY);
#else
    cMvX = divideRMVF(cMvX, parametersRMVF[0][3]);
    cMvY = divideRMVF(cMvY, parametersRMVF[1][3]);
    cMvX += firstMv.getHor();
    cMvY += firstMv.getVer();
    cMv = Mv((int)(cMvX), (int)(cMvY));
    cMv.hor = cMv.hor >= 0 ? (cMv.hor + 1) >> 2 : (cMv.hor + 2) >> 2;
    cMv.ver = cMv.ver >= 0 ? (cMv.ver + 1) >> 2 : (cMv.ver + 2) >> 2;
#endif

    cMV[list][1] = cMv;
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
    cMV[list][1].clipToStorageBitDepth();
#endif  
    // bottom-left CPMV
    cMvX = parametersRMVF[0][1] * pu.lumaSize().height + parametersRMVF[0][2];
    cMvY = parametersRMVF[1][1] * pu.lumaSize().height + parametersRMVF[1][2];

#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
    cMvX = (firstMv.getHor() << MAX_CU_DEPTH) + cMvX;
    cMvY = (firstMv.getVer() << MAX_CU_DEPTH) + cMvY;
    iMvX = int(cMvX);
    iMvY = int(cMvY);
    roundAffineMv(iMvX, iMvY, MAX_CU_DEPTH);
    cMv = Mv(iMvX, iMvY);
#else
    cMvX = divideRMVF(cMvX, parametersRMVF[0][3]);
    cMvY = divideRMVF(cMvY, parametersRMVF[1][3]);
    cMvX += firstMv.getHor();
    cMvY += firstMv.getVer();
    cMv = Mv((int)(cMvX), (int)(cMvY));
    cMv.hor = cMv.hor >= 0 ? (cMv.hor + 1) >> 2 : (cMv.hor + 2) >> 2;
    cMv.ver = cMv.ver >= 0 ? (cMv.ver + 1) >> 2 : (cMv.ver + 2) >> 2;
#endif

    cMV[list][2] = cMv;
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
    cMV[list][2].clipToStorageBitDepth();
#endif
    available[eRefPicList] = true;

    mvpInfoVec[eRefPicList].clear();
    mvpInfoVecOri.clear();
  }
  int8_t referenceidx[2];
  referenceidx[0] = abovePU.refIdx[0];
  referenceidx[1] = abovePU.refIdx[1];
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
  if (enoughBlk[0] || enoughBlk[1])
  {
#endif
#if JVET_AG0276_NLIC
    if (PU::isValidAltMergeCandidate(abovePU) && (altAffineMergeCtx->numValidMergeCand < ALT_AFF_MRG_MAX_NUM_CANDS))
    {
      int cnt = altAffineMergeCtx->numValidMergeCand;
      altAffineMergeCtx->affineType[cnt] = AFFINEMODEL_6PARAM;
      for (int mvNum = 0; mvNum < 3; mvNum++)
      {
        altAffineMergeCtx->mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField(cMVOri[0][mvNum], referenceidx[0]);
        altAffineMergeCtx->mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField(cMVOri[1][mvNum], referenceidx[1]);
      }
      altAffineMergeCtx->interDirNeighbours[cnt] = abovePU.interDir;
      altAffineMergeCtx->bcwIdx[cnt] = abovePU.cu->bcwIdx;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
      altAffineMergeCtx->obmcFlags[cnt] = abovePU.cu->affine ? abovePU.cu->obmcFlag : true;
#endif
      altAffineMergeCtx->altLMParaNeighbours[cnt] = abovePU.cu->altLMParaUnit;
      if (altAffineMergeCtx->xCheckSameAffMotion(pu, cnt))
      {
        altAffineMergeCtx->numValidMergeCand++;
      }
    }
#if JVET_AG0276_LIC_FLAG_SIGNALING
    if (PU::isValidAltMergeCandidate(abovePU, true) && (altBRAffineMergeCtx->numValidMergeCand < ALT_AFF_MRG_MAX_NUM_CANDS))
    {
      int cnt = altBRAffineMergeCtx->numValidMergeCand;
      altBRAffineMergeCtx->affineType[cnt] = AFFINEMODEL_6PARAM;
      for (int mvNum = 0; mvNum < 3; mvNum++)
      {
        altBRAffineMergeCtx->mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField(cMVOri[0][mvNum], referenceidx[0]);
        altBRAffineMergeCtx->mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField(cMVOri[1][mvNum], referenceidx[1]);
      }
      altBRAffineMergeCtx->interDirNeighbours[cnt] = abovePU.interDir;
      altBRAffineMergeCtx->bcwIdx[cnt] = abovePU.cu->bcwIdx;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
      altBRAffineMergeCtx->obmcFlags[cnt] = abovePU.cu->affine ? abovePU.cu->obmcFlag : true;
#endif
      altBRAffineMergeCtx->altLMParaNeighbours[cnt] = abovePU.cu->altLMBRParaUnit;
      if (altBRAffineMergeCtx->xCheckSameAffMotion(pu, cnt))
      {
        altBRAffineMergeCtx->numValidMergeCand++;
      }
    }
#endif
#endif
    if (!xCPMVSimCheck(pu, affMrgCtx, cMVOri, abovePU.interDir, referenceidx, AFFINEMODEL_6PARAM, abovePU.cu->bcwIdx, abovePU.cu->licFlag))
    {
      int i = affMrgCtx.numValidMergeCand;
      for (int mvNum = 0; mvNum < 3; mvNum++)
      {
        affMrgCtx.mvFieldNeighbours[i << 1][mvNum].setMvField(cMVOri[0][mvNum], referenceidx[0]);
        affMrgCtx.mvFieldNeighbours[(i << 1) + 1][mvNum].setMvField(cMVOri[1][mvNum], referenceidx[1]);
      }
      affMrgCtx.interDirNeighbours[i] = abovePU.interDir;
      affMrgCtx.affineType[i] = AFFINEMODEL_6PARAM;
      affMrgCtx.mergeType[i] = MRG_TYPE_DEFAULT_N;
      affMrgCtx.bcwIdx[i] = abovePU.cu->bcwIdx;
  #if INTER_LIC
      affMrgCtx.licFlags[i] = abovePU.cu->licFlag;
  #endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
#if JVET_AD0213_LIC_IMP
      affMrgCtx.obmcFlags[i] = abovePU.cu->affine ? abovePU.cu->obmcFlag : true;
#else
      affMrgCtx.obmcFlags[i] = abovePU.cu->affine ? abovePU.cu->obmcFlag : !abovePU.cu->licFlag;
#endif
#endif
      if (affMrgCtx.numValidMergeCand == mrgCandIdx)
      {
        affMrgCtx.numValidMergeCand++;
        return;
      }
      affMrgCtx.numValidMergeCand++;
      if (affMrgCtx.numValidMergeCand == affMrgCtx.maxNumMergeCand)
      {
        return;
      }
    }
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
  }
#endif
  if (available[REF_PIC_LIST_0] || available[REF_PIC_LIST_1])
  {
#if JVET_AG0276_NLIC
    if (PU::isValidAltMergeCandidate(abovePU) && (altAffineMergeCtx->numValidMergeCand < ALT_AFF_MRG_MAX_NUM_CANDS))
    {
      int cnt = altAffineMergeCtx->numValidMergeCand;
      altAffineMergeCtx->affineType[cnt] = AFFINEMODEL_6PARAM;
      for (int mvNum = 0; mvNum < 3; mvNum++)
      {
        altAffineMergeCtx->mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField(cMV[0][mvNum], referenceidx[0]);
        altAffineMergeCtx->mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField(cMV[1][mvNum], referenceidx[1]);
      }
      altAffineMergeCtx->interDirNeighbours[cnt] = abovePU.interDir;
      altAffineMergeCtx->bcwIdx[cnt] = abovePU.cu->bcwIdx;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
      altAffineMergeCtx->obmcFlags[cnt] = abovePU.cu->affine ? abovePU.cu->obmcFlag : true;
#endif
      altAffineMergeCtx->altLMParaNeighbours[cnt] = abovePU.cu->altLMParaUnit;
      if (altAffineMergeCtx->xCheckSameAffMotion(pu, cnt))
      {
        altAffineMergeCtx->numValidMergeCand++;
      }
    }
#if JVET_AG0276_LIC_FLAG_SIGNALING
    if (PU::isValidAltMergeCandidate(abovePU, true) && (altBRAffineMergeCtx->numValidMergeCand < ALT_AFF_MRG_MAX_NUM_CANDS))
    {
      int cnt = altBRAffineMergeCtx->numValidMergeCand;
      altBRAffineMergeCtx->affineType[cnt] = AFFINEMODEL_6PARAM;
      for (int mvNum = 0; mvNum < 3; mvNum++)
      {
        altBRAffineMergeCtx->mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField(cMV[0][mvNum], referenceidx[0]);
        altBRAffineMergeCtx->mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField(cMV[1][mvNum], referenceidx[1]);
      }
      altBRAffineMergeCtx->interDirNeighbours[cnt] = abovePU.interDir;
      altBRAffineMergeCtx->bcwIdx[cnt] = abovePU.cu->bcwIdx;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
      altBRAffineMergeCtx->obmcFlags[cnt] = abovePU.cu->affine ? abovePU.cu->obmcFlag : true;
#endif
      altBRAffineMergeCtx->altLMParaNeighbours[cnt] = abovePU.cu->altLMBRParaUnit;
      if (altBRAffineMergeCtx->xCheckSameAffMotion(pu, cnt))
      {
        altBRAffineMergeCtx->numValidMergeCand++;
      }
    }
#endif
#endif
    if (!xCPMVSimCheck(pu, affMrgCtx, cMV, abovePU.interDir, referenceidx, AFFINEMODEL_6PARAM, BCW_DEFAULT, false))
    {
      int i = affMrgCtx.numValidMergeCand;
      for (int mvNum = 0; mvNum < 3; mvNum++)
      {
        affMrgCtx.mvFieldNeighbours[i << 1][mvNum].setMvField(cMV[0][mvNum], referenceidx[0]);
        affMrgCtx.mvFieldNeighbours[(i << 1) + 1][mvNum].setMvField(cMV[1][mvNum], referenceidx[1]);
      }
      affMrgCtx.interDirNeighbours[i] = abovePU.interDir;
      affMrgCtx.affineType[i] = AFFINEMODEL_6PARAM;
      affMrgCtx.mergeType[i] = MRG_TYPE_DEFAULT_N;
#if JVET_AD0213_LIC_IMP
      affMrgCtx.bcwIdx[i] = abovePU.cu->bcwIdx;
#else
      affMrgCtx.bcwIdx[i] = BCW_DEFAULT;
#endif
#if INTER_LIC
#if JVET_AD0213_LIC_IMP
      affMrgCtx.licFlags[i] = abovePU.cu->licFlag;
#else
      affMrgCtx.licFlags[i] = false;
#endif
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
      affMrgCtx.obmcFlags[i] = abovePU.cu->affine ? abovePU.cu->obmcFlag : true;
#endif
      if (affMrgCtx.numValidMergeCand == mrgCandIdx)
      {
        affMrgCtx.numValidMergeCand++;
        return;
      }
      affMrgCtx.numValidMergeCand++;
      if (affMrgCtx.numValidMergeCand == affMrgCtx.maxNumMergeCand)
      {
        return;
      }
    }
  }
}
void PU::xReturnMvpVec(std::vector<RMVFInfo> mvp[NUM_REF_PIC_LIST_01][MAX_NUM_REF], const PredictionUnit &pu, const Position &pos
#if !JVET_AB0189_RMVF_BITLENGTH_CONTROL
  , const MvpDir &eDir
#endif
  )
{
  CodingStructure &cs = *pu.cs;
  const PredictionUnit *neibPU = NULL;
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
  Position neibPos = pos;
#else
  Position neibPos;
  int offset = 2;
  switch (eDir)
  {
  case MD_LEFT:
    neibPos = pos.offset(-offset, 0);
    break;
  case MD_ABOVE:
    neibPos = pos.offset(0, -offset);
    break;
  case MD_ABOVE_RIGHT:
    neibPos = pos.offset(1, -1);
    break;
  case MD_BELOW_LEFT:
    neibPos = pos.offset(-1, 1);
    break;
  case MD_ABOVE_LEFT:
    neibPos = pos.offset(-1, -1);
    break;
  default:
    break;
  }
#endif
  neibPU = cs.getPURestricted(neibPos, pu, pu.chType);
  if (neibPU == NULL || !CU::isInter(*neibPU->cu))
  {
    return;
  }
  const MotionInfo& neibMi = neibPU->getMotionInfo(neibPos);
  if (neibMi.refIdx[0] >= 0)
  {
    mvp[0][neibMi.refIdx[0]].push_back(RMVFInfo(neibMi.mv[0], neibPos, neibMi.refIdx[0]));
  }
  if (neibMi.refIdx[1] >= 0)
  {
    mvp[1][neibMi.refIdx[1]].push_back(RMVFInfo(neibMi.mv[1], neibPos, neibMi.refIdx[1]));
  }
}
void PU::collectNeiMotionInfo(std::vector<RMVFInfo> mvpInfoVec[NUM_REF_PIC_LIST_01][MAX_NUM_REF], const PredictionUnit &pu)
{
  Position anchorPos = pu.Y().topLeft();
  Position neibPos;

  int imHeight = pu.cs->slice->getPic()->lheight();
  int imWidth = pu.cs->slice->getPic()->lwidth();
  int hSearch = pu.Y().width;
  int vSearch = pu.Y().height;
  int horSearch = hSearch >> 1;
  int verSearch = vSearch >> 1;
  int stepsize = 4;
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
  if (pu.lumaSize().width > RMVF_CUSIZE_THRED)
  {
    stepsize = 8;
  }
#endif
  neibPos = anchorPos;
  if (neibPos.y - 2 >= 0)
  {
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
    neibPos = anchorPos.offset(-2, -2);
#else
    neibPos = anchorPos.offset(-2, 0);
#endif
    for (int j = 0; j < hSearch; j += stepsize)
    {
      neibPos.x += stepsize;
      xReturnMvpVec(mvpInfoVec, pu, neibPos
        
#if !JVET_AB0189_RMVF_BITLENGTH_CONTROL
        , MD_ABOVE
#endif
      );
    }
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
    neibPos = anchorPos.offset(2, -2);
    int leftend = anchorPos.x - horSearch < 0 ? anchorPos.x : horSearch;
    for (int j = 0; j < leftend; j += stepsize)
#else
    neibPos = anchorPos.offset(2, 0);
    for (int j = 0; j < horSearch; j += stepsize)
#endif
    {
      neibPos.x -= stepsize;
#if !JVET_AB0189_RMVF_BITLENGTH_CONTROL
      if (neibPos.x < 0)
      {
        break;
      }
#endif
      xReturnMvpVec(mvpInfoVec, pu, neibPos
#if !JVET_AB0189_RMVF_BITLENGTH_CONTROL
        , MD_ABOVE
#endif
      );
    }
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
    neibPos = anchorPos.offset(hSearch - 2, -2);
    int rightend = pu.Y().topRight().x + horSearch > imWidth - 1 ? imWidth - 1 - pu.Y().topRight().x : horSearch;
    rightend = pu.lumaSize().width == RMVF_DISTANCE_THRED ? 0 : rightend;
    for (int j = 0; j < rightend; j += stepsize)
#else
    neibPos = anchorPos.offset(hSearch - 2, 0);
    for (int j = 0; j < horSearch; j += stepsize)
#endif
    {
      neibPos.x += stepsize;
#if !JVET_AB0189_RMVF_BITLENGTH_CONTROL
      if (neibPos.x > imWidth - 1)
      {
        break;
      }
#endif
      xReturnMvpVec(mvpInfoVec, pu, neibPos
#if !JVET_AB0189_RMVF_BITLENGTH_CONTROL
        , MD_ABOVE
#endif
      );
    }
  }
  neibPos = anchorPos;
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
  stepsize = 4;
  if (pu.lumaSize().height > RMVF_CUSIZE_THRED)
  {
    stepsize = 8;
  }
#endif
  if (neibPos.x - 2 >= 0)
  {
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
    neibPos = anchorPos.offset(-2, -2);
#else
    neibPos = anchorPos.offset(0, -2);
#endif
    for (int i = 0; i < vSearch; i += stepsize)
    {
      neibPos.y += stepsize;
      xReturnMvpVec(mvpInfoVec, pu, neibPos
#if !JVET_AB0189_RMVF_BITLENGTH_CONTROL
        , MD_LEFT
#endif
      );
    }
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
    neibPos = anchorPos.offset(-2, vSearch - 2);
    int belowend = pu.Y().bottomLeft().y + verSearch > imHeight - 1 ? imHeight - 1 - pu.Y().bottomLeft().y : verSearch;
    belowend = pu.lumaSize().height == RMVF_DISTANCE_THRED ? 0 : belowend;
    for (int j = 0; j < belowend; j += stepsize)
#else
    neibPos = anchorPos.offset(0, vSearch - 2);
    for (int i = 0; i < verSearch; i += stepsize)
#endif
    {
      neibPos.y += stepsize;
#if !JVET_AB0189_RMVF_BITLENGTH_CONTROL
      if (neibPos.y > imHeight - 1)
      {
        break;
      }
#endif
      xReturnMvpVec(mvpInfoVec, pu, neibPos
#if !JVET_AB0189_RMVF_BITLENGTH_CONTROL
        , MD_LEFT
#endif
      );
    }
  }
}
#endif
void PU::getInterBMCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx,
  const int& mrgCandIdx
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
  , MergeCtx* mvpMrgCtx1
  , MergeCtx* mvpMrgCtx2
#endif
)
{
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
  if (!(pu.cs->sps->getUseAML() && pu.cs->sps->getTMToolsEnableFlag()) || !pu.cs->sps->getUseTmvpNmvpReordering())
#else
  if (!pu.cs->sps->getUseAML() || !pu.cs->sps->getUseTmvpNmvpReordering())
#endif
  {
    mvpMrgCtx1 = nullptr;
    mvpMrgCtx2 = nullptr;
  }
#endif
  const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
  const CodingStructure &cs = *pu.cs;
  const Slice &slice = *pu.cs->slice;
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
  uint32_t additionalMRGCand = (mvpMrgCtx1 != NULL ? mvpMrgCtx1->numValidMergeCand : 0)
                             + (mvpMrgCtx2 != NULL ? mvpMrgCtx2->numValidMergeCand : 0);
#endif
  const uint32_t maxNumMergeCand = pu.cs->sps->getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
                                && pu.cs->sps->getTMToolsEnableFlag()
#endif
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
                                && pu.cs->sps->getUseTmvpNmvpReordering()
#endif
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
                                 ? std::min((int)(BM_MRG_MAX_NUM_INIT_CANDS + additionalMRGCand), ((int)NUM_MERGE_CANDS - (3)))
#else
                                 ? BM_MRG_MAX_NUM_INIT_CANDS
#endif
								 : pu.cs->sps->getMaxNumBMMergeCand();
#else
  const uint32_t maxNumMergeCand = pu.cs->sps->getMaxNumBMMergeCand();
#endif
  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
    mrgCtx.bcwIdx[ui] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
    mrgCtx.altLMFlag[ui] = false;
    mrgCtx.altLMParaNeighbours[ui].resetAltLinearModel();
#endif
#if INTER_LIC
    mrgCtx.licFlags[ui] = false;
#endif
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mvFieldNeighbours[(ui << 1)].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
    mrgCtx.useAltHpelIf[ui] = false;
#if MULTI_HYP_PRED
    mrgCtx.addHypNeighbours[ui].clear();
#endif
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
    mrgCtx.candCost[ui] = MAX_UINT64;
#endif
  }

  mrgCtx.numValidMergeCand = maxNumMergeCand;
  mrgCtx.numCandToTestEnc = maxNumMergeCand;
  // compute the location of the current PU

  int mvThreshod = 1;

  int cnt = 0;

  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  const Position posLB = pu.Y().bottomLeft();
  MotionInfo miAbove, miLeft, miAboveLeft, miAboveRight, miBelowLeft;

  // above
  const PredictionUnit *puAbove = cs.getPURestricted(posRT.offset(0, -1), pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
  bool isAvailableB1 = puAbove && isDiffMER(pu.lumaPos(), posRT.offset(0, -1), plevel) && pu.cu != puAbove->cu && CU::isInter(*puAbove->cu) && puAbove->getMotionInfo(posRT.offset(0, -1)).isInter;
#else
  bool isAvailableB1 = puAbove && isDiffMER(pu.lumaPos(), posRT.offset(0, -1), plevel) && pu.cu != puAbove->cu && CU::isInter(*puAbove->cu);
#endif

  if (isAvailableB1)
  {
    miAbove = puAbove->getMotionInfo(posRT.offset(0, -1));

    if (isBiPredFromDifferentDirEqDistPoc(pu, miAbove.refIdx[0], miAbove.refIdx[1])
      )
    {
      mrgCtx.interDirNeighbours[cnt] = miAbove.interDir;
      mrgCtx.useAltHpelIf[cnt] = miAbove.useAltHpelIf;
      mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAbove.mv[0], miAbove.refIdx[0]);
      mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miAbove.mv[1], miAbove.refIdx[1]);
#if JVET_Y0089_DMVR_BCW
      mrgCtx.bcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAbove->cu->bcwIdx : BCW_DEFAULT;
#endif
#if NON_ADJACENT_MRG_CAND || TM_MRG
      if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod))
      {
#endif
        if (mrgCandIdx == cnt)
        {
          mrgCtx.numValidMergeCand = cnt + 1;
          return;
        }
        cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG
      }
#if JVET_AD0213_LIC_IMP
      else
      {
        mrgCtx.initMrgCand(cnt);
      }
#endif
#endif
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    mrgCtx.numValidMergeCand = cnt;
    return;
  }

  //left
  const PredictionUnit* puLeft = cs.getPURestricted(posLB.offset(-1, 0), pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
  const bool isAvailableA1 = puLeft && isDiffMER(pu.lumaPos(), posLB.offset(-1, 0), plevel) && pu.cu != puLeft->cu && CU::isInter(*puLeft->cu) && puLeft->getMotionInfo(posLB.offset(-1, 0)).isInter;
#else
  const bool isAvailableA1 = puLeft && isDiffMER(pu.lumaPos(), posLB.offset(-1, 0), plevel) && pu.cu != puLeft->cu && CU::isInter(*puLeft->cu);
#endif

  if (isAvailableA1)
  {
    miLeft = puLeft->getMotionInfo(posLB.offset(-1, 0));

    if (!isAvailableB1 || (miAbove != miLeft))
    {
      if (isBiPredFromDifferentDirEqDistPoc(pu, miLeft.refIdx[0], miLeft.refIdx[1])
        )
      {
        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miLeft.interDir;
        mrgCtx.useAltHpelIf[cnt] = miLeft.useAltHpelIf;
        // get Mv from Left
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miLeft.mv[0], miLeft.refIdx[0]);
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miLeft.mv[1], miLeft.refIdx[1]);
#if JVET_Y0089_DMVR_BCW
        mrgCtx.bcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puLeft->cu->bcwIdx : BCW_DEFAULT;
#endif
#if NON_ADJACENT_MRG_CAND || TM_MRG
        if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod))
        {
#endif
          if (mrgCandIdx == cnt)
          {
            mrgCtx.numValidMergeCand = cnt + 1;
            return;
          }

          cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG
        }
#if JVET_AD0213_LIC_IMP
        else
        {
          mrgCtx.initMrgCand(cnt);
        }
#endif
#endif
      }
    }
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    mrgCtx.numValidMergeCand = cnt;
    return;
  }

  // above right
  const PredictionUnit *puAboveRight = cs.getPURestricted(posRT.offset(1, -1), pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
  bool isAvailableB0 = puAboveRight && isDiffMER(pu.lumaPos(), posRT.offset(1, -1), plevel) && CU::isInter(*puAboveRight->cu) && puAboveRight->getMotionInfo(posRT.offset(1, -1)).isInter;
#else
  bool isAvailableB0 = puAboveRight && isDiffMER(pu.lumaPos(), posRT.offset(1, -1), plevel) && CU::isInter(*puAboveRight->cu);
#endif

  if (isAvailableB0)
  {
    miAboveRight = puAboveRight->getMotionInfo(posRT.offset(1, -1));

    if (!isAvailableB1 || (miAbove != miAboveRight))
    {
      if (isBiPredFromDifferentDirEqDistPoc(pu, miAboveRight.refIdx[0], miAboveRight.refIdx[1])
        )
      {
        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miAboveRight.interDir;
        mrgCtx.useAltHpelIf[cnt] = miAboveRight.useAltHpelIf;
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAboveRight.mv[0], miAboveRight.refIdx[0]);
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miAboveRight.mv[1], miAboveRight.refIdx[1]);
#if JVET_Y0089_DMVR_BCW
        mrgCtx.bcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAboveRight->cu->bcwIdx : BCW_DEFAULT;
#endif

#if NON_ADJACENT_MRG_CAND || TM_MRG
        if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod))
        {
#endif
          if (mrgCandIdx == cnt)
          {
            mrgCtx.numValidMergeCand = cnt + 1;
            return;
          }
          cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG
        }
#if JVET_AD0213_LIC_IMP
        else
        {
          mrgCtx.initMrgCand(cnt);
        }
#endif
#endif
      }
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    mrgCtx.numValidMergeCand = cnt;
    return;
  }

  //left bottom
  const PredictionUnit *puLeftBottom = cs.getPURestricted(posLB.offset(-1, 1), pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
  bool isAvailableA0 = puLeftBottom && isDiffMER(pu.lumaPos(), posLB.offset(-1, 1), plevel) && CU::isInter(*puLeftBottom->cu) && puLeftBottom->getMotionInfo(posLB.offset(-1, 1)).isInter;
#else
  bool isAvailableA0 = puLeftBottom && isDiffMER(pu.lumaPos(), posLB.offset(-1, 1), plevel) && CU::isInter(*puLeftBottom->cu);
#endif

  if (isAvailableA0)
  {
    miBelowLeft = puLeftBottom->getMotionInfo(posLB.offset(-1, 1));

    if (!isAvailableA1 || (miBelowLeft != miLeft))
    {
      if (isBiPredFromDifferentDirEqDistPoc(pu, miBelowLeft.refIdx[0], miBelowLeft.refIdx[1])
        )
      {
        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miBelowLeft.interDir;
        mrgCtx.useAltHpelIf[cnt] = miBelowLeft.useAltHpelIf;
        // get Mv from Bottom-Left
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miBelowLeft.mv[0], miBelowLeft.refIdx[0]);
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miBelowLeft.mv[1], miBelowLeft.refIdx[1]);
#if JVET_Y0089_DMVR_BCW
        mrgCtx.bcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puLeftBottom->cu->bcwIdx : BCW_DEFAULT;
#endif

#if NON_ADJACENT_MRG_CAND || TM_MRG
        if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod))
        {
#endif
          if (mrgCandIdx == cnt)
          {
#if TM_MRG
            if (!pu.tmMergeFlag)
#endif
              return;
          }
          cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG
        }
#if JVET_AD0213_LIC_IMP
        else
        {
          mrgCtx.initMrgCand(cnt);
        }
#endif
#endif
      }
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    mrgCtx.numValidMergeCand = cnt;
    return;
  }


  const PredictionUnit *puAboveLeft = cs.getPURestricted(posLT.offset(-1, -1), pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
  bool isAvailableB2 = puAboveLeft && isDiffMER(pu.lumaPos(), posLT.offset(-1, -1), plevel) && CU::isInter(*puAboveLeft->cu) && puAboveLeft->getMotionInfo(posLT.offset(-1, -1)).isInter;
#else
  bool isAvailableB2 = puAboveLeft && isDiffMER(pu.lumaPos(), posLT.offset(-1, -1), plevel) && CU::isInter(*puAboveLeft->cu);
#endif

  if (isAvailableB2)
  {
    miAboveLeft = puAboveLeft->getMotionInfo(posLT.offset(-1, -1));

    if ((!isAvailableA1 || (miLeft != miAboveLeft)) && (!isAvailableB1 || (miAbove != miAboveLeft)))
    {
      if (isBiPredFromDifferentDirEqDistPoc(pu, miAboveLeft.refIdx[0], miAboveLeft.refIdx[1])
        )
      {

        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miAboveLeft.interDir;
        mrgCtx.useAltHpelIf[cnt] = miAboveLeft.useAltHpelIf;
        // get Mv from Above-Left
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAboveLeft.mv[0], miAboveLeft.refIdx[0]);
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miAboveLeft.mv[1], miAboveLeft.refIdx[1]);
#if JVET_Y0089_DMVR_BCW
        mrgCtx.bcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puAboveLeft->cu->bcwIdx : BCW_DEFAULT;
#endif

#if NON_ADJACENT_MRG_CAND || TM_MRG
        if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod))
        {
#endif
          if (mrgCandIdx == cnt)
          {
            mrgCtx.numValidMergeCand = cnt + 1;
            return;
          }

          cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG
        }
#if JVET_AD0213_LIC_IMP
        else
        {
          mrgCtx.initMrgCand(cnt);
        }
#endif
#endif
      }
    }
  }
  // early termination
  if (cnt == maxNumMergeCand)
  {
    mrgCtx.numValidMergeCand = cnt;
    return;
  }

#if INTER_RM_SIZE_CONSTRAINTS
  if (slice.getPicHeader()->getEnableTMVPFlag())
#else
  if (slice.getPicHeader()->getEnableTMVPFlag() && (pu.lumaSize().width + pu.lumaSize().height > 12))
#endif
  {
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
    if (mvpMrgCtx1 == NULL)
    {
#endif
    //>> MTK colocated-RightBottom
    // offset the pos to be sure to "point" to the same position the uiAbsPartIdx would've pointed to
    Position posRB = pu.Y().bottomRight().offset(-3, -3);
    const PreCalcValues& pcv = *cs.pcv;

    Position posC0;
    Position posC1 = pu.Y().center();
    bool isC0Avail = false;
    bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
    const SubPic& curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
    if (curSubPic.getTreatedAsPicFlag())
    {
      boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
        (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom());
    }
    if (boundaryCond)
    {
      int posYInCtu = posRB.y & pcv.maxCUHeightMask;
      if (posYInCtu + 4 < pcv.maxCUHeight)
      {
        posC0 = posRB.offset(4, 4);
        isC0Avail = true;
      }
    }

    Mv        cColMv;
    int       iRefIdx = 0;
    int       dir = 0;
    unsigned  uiArrayAddr = cnt;
    bool      bExistMV = (isC0Avail && getColocatedMVP(pu, REF_PIC_LIST_0, posC0, cColMv, iRefIdx, false))
      || getColocatedMVP(pu, REF_PIC_LIST_0, posC1, cColMv, iRefIdx, false);
    if (bExistMV)
    {
      dir |= 1;
      mrgCtx.mvFieldNeighbours[2 * uiArrayAddr].setMvField(cColMv, iRefIdx);
    }

    if (slice.isInterB())
    {
      bExistMV = (isC0Avail && getColocatedMVP(pu, REF_PIC_LIST_1, posC0, cColMv, iRefIdx, false))
        || getColocatedMVP(pu, REF_PIC_LIST_1, posC1, cColMv, iRefIdx, false);
      if (bExistMV)
      {
        dir |= 2;
        mrgCtx.mvFieldNeighbours[2 * uiArrayAddr + 1].setMvField(cColMv, iRefIdx);
      }
    }

    if (dir != 0)
    {
      bool addTMvp = isBiPredFromDifferentDirEqDistPoc(pu, mrgCtx.mvFieldNeighbours[2 * uiArrayAddr + 0].refIdx, mrgCtx.mvFieldNeighbours[2 * uiArrayAddr + 1].refIdx);
      if (addTMvp)
      {
        mrgCtx.interDirNeighbours[uiArrayAddr] = dir;
        mrgCtx.useAltHpelIf[uiArrayAddr] = false;
#if MULTI_HYP_PRED
        mrgCtx.addHypNeighbours[uiArrayAddr].clear();
#endif
#if NON_ADJACENT_MRG_CAND || TM_MRG
        if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod))
        {
#endif
          if (mrgCandIdx == cnt)
          {
            mrgCtx.numValidMergeCand = cnt + 1;
            return;
          }

          cnt++;
#if NON_ADJACENT_MRG_CAND || TM_MRG
        }
#if JVET_AD0213_LIC_IMP
        else
        {
          mrgCtx.initMrgCand(cnt);
        }
#endif
#endif
      }
    }
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
    }
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
    else if (mvpMrgCtx1->numValidMergeCand > 0)
    {
      for (int uiNumCand = 0; uiNumCand < mvpMrgCtx1->numValidMergeCand && cnt < maxNumMergeCand; uiNumCand++)
      {
        mrgCtx.interDirNeighbours[cnt] = mvpMrgCtx1->interDirNeighbours[uiNumCand];
        mrgCtx.mvFieldNeighbours[cnt << 1] = mvpMrgCtx1->mvFieldNeighbours[uiNumCand << 1];
        mrgCtx.useAltHpelIf[cnt] = mvpMrgCtx1->useAltHpelIf[uiNumCand];
        mrgCtx.mvFieldNeighbours[(cnt << 1) + 1] = mvpMrgCtx1->mvFieldNeighbours[(uiNumCand << 1) + 1];
#if JVET_Y0089_DMVR_BCW
        mrgCtx.bcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? mvpMrgCtx1->bcwIdx[uiNumCand] : BCW_DEFAULT;
#endif
#if NON_ADJACENT_MRG_CAND || TM_MRG
        if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod))
#endif
        {
          mrgCtx.candCost[cnt] = mvpMrgCtx1->candCost[uiNumCand];
          if (mrgCandIdx == cnt)
          {
            mrgCtx.numValidMergeCand = cnt + 1;
            return;
          }
          cnt++;
        }
#if JVET_AD0213_LIC_IMP
        else
        {
          mrgCtx.initMrgCand(cnt);
        }
#endif
      }
    }
#else
    else if (mvpMrgCtx1->numValidMergeCand > 0)
    {
      mrgCtx.interDirNeighbours[cnt] = mvpMrgCtx1->interDirNeighbours[0];
      mrgCtx.mvFieldNeighbours[cnt << 1] = mvpMrgCtx1->mvFieldNeighbours[0];
      mrgCtx.useAltHpelIf[cnt] = mvpMrgCtx1->useAltHpelIf[0];
      mrgCtx.mvFieldNeighbours[(cnt << 1) + 1] = mvpMrgCtx1->mvFieldNeighbours[1];
#if MULTI_HYP_PRED
      mrgCtx.addHypNeighbours[cnt] = mvpMrgCtx1->addHypNeighbours[0];
#endif
#if JVET_Y0089_DMVR_BCW
      mrgCtx.bcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? mvpMrgCtx1->bcwIdx[0] : BCW_DEFAULT;
#endif
#if NON_ADJACENT_MRG_CAND || TM_MRG
      if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod))
#endif
      {
        mrgCtx.candCost[cnt] = mvpMrgCtx1->candCost[0];
        if (mrgCandIdx == cnt)
        {
          mrgCtx.numValidMergeCand = cnt + 1;
          return;
        }
        cnt++;
      }
#if JVET_AD0213_LIC_IMP
      else
      {
        mrgCtx.initMrgCand(cnt);
      }
#endif
    }
#endif
#endif
  }

  // early termination
  if (cnt == maxNumMergeCand)
  {
    mrgCtx.numValidMergeCand = cnt;
    return;
  }

#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
  int maxNumMergeCandMin1 = maxNumMergeCand;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
  maxNumMergeCandMin1 -= !(pu.cs->sps->getUseAML() && pu.cs->sps->getTMToolsEnableFlag()) ? 1 : 0;
#else
  maxNumMergeCandMin1 -= !pu.cs->sps->getUseAML() ? 1 : 0;
#endif
#endif
#else
  int maxNumMergeCandMin1 = maxNumMergeCand - 1;
#endif
#if NON_ADJACENT_MRG_CAND
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
  if (mvpMrgCtx2 == NULL)
  {
#endif
  MotionInfo miNeighbor;
  int offsetX = 0;
  int offsetY = 0;
  const int numNACandidate[4] = { 3, 5, 5, 5 };
  const int idxMap[4][5] = { { 0, 1, 4 },{ 0, 1, 2, 3, 4 },{ 0, 1, 2, 3, 4 },{ 0, 1, 2, 3, 4 } };

  for (int iDistanceIndex = 0; iDistanceIndex < NADISTANCE_LEVEL && cnt < maxNumMergeCandMin1; iDistanceIndex++)
  {
    const int iNADistanceHor = pu.Y().width  * (iDistanceIndex + 1);
    const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);

    for (int iNASPIdx = 0; iNASPIdx < numNACandidate[iDistanceIndex] && cnt < maxNumMergeCandMin1; iNASPIdx++)
    {
      switch (idxMap[iDistanceIndex][iNASPIdx])
      {
      case 0:offsetX = -iNADistanceHor - 1; offsetY = pu.Y().height + iNADistanceVer - 1; break;
      case 1:offsetX = pu.Y().width + iNADistanceHor - 1; offsetY = -iNADistanceVer - 1; break;
      case 2:offsetX = pu.Y().width >> 1;   offsetY = -iNADistanceVer - 1;    break;
      case 3:offsetX = -iNADistanceHor - 1; offsetY = pu.Y().height >> 1; break;
      case 4:offsetX = -iNADistanceHor - 1; offsetY = -iNADistanceVer - 1;    break;
      default: printf("error!"); exit(0); break;
      }

      const PredictionUnit *puNonAdjacent = cs.getPURestricted(posLT.offset(offsetX, offsetY), pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(pu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent->cu) && puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY)).isInter;
#else
      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(pu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent->cu);
#endif

      if (isAvailableNonAdjacent)
      {
        miNeighbor = puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY));

        if( isBiPredFromDifferentDirEqDistPoc(pu, miNeighbor.refIdx[0], miNeighbor.refIdx[1]) )
        {
          // get Inter Dir
          mrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
          // get Mv from Above-Left
          mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
          mrgCtx.useAltHpelIf[cnt] = miNeighbor.useAltHpelIf;
          mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);
#if JVET_Y0089_DMVR_BCW
          mrgCtx.bcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puNonAdjacent->cu->bcwIdx : BCW_DEFAULT;
#endif

#if NON_ADJACENT_MRG_CAND || TM_MRG
          if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod))
#endif
          {
            if (mrgCandIdx == cnt)
            {
              mrgCtx.numValidMergeCand = cnt + 1;
              return;
            }
            cnt++;
          }
#if JVET_AD0213_LIC_IMP
          else
          {
            mrgCtx.initMrgCand(cnt);
          }
#endif
        }
      }
    }
  }
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
  }
  else
  {
    for (int uiNumCand = 0; uiNumCand < mvpMrgCtx2->numValidMergeCand && cnt < maxNumMergeCandMin1; uiNumCand++)
    {
      mrgCtx.interDirNeighbours[cnt] = mvpMrgCtx2->interDirNeighbours[uiNumCand];
      mrgCtx.mvFieldNeighbours[cnt << 1] = mvpMrgCtx2->mvFieldNeighbours[uiNumCand << 1];
      mrgCtx.useAltHpelIf[cnt] = mvpMrgCtx2->useAltHpelIf[uiNumCand];
      mrgCtx.mvFieldNeighbours[(cnt << 1) + 1] = mvpMrgCtx2->mvFieldNeighbours[(uiNumCand << 1) + 1];
#if JVET_Y0089_DMVR_BCW
      mrgCtx.bcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? mvpMrgCtx2->bcwIdx[uiNumCand] : BCW_DEFAULT;
#endif
#if NON_ADJACENT_MRG_CAND || TM_MRG
      if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod))
#endif
      {
        mrgCtx.candCost[cnt] = mvpMrgCtx2->candCost[uiNumCand];
        if (mrgCandIdx == cnt)
        {
          mrgCtx.numValidMergeCand = cnt + 1;
          return;
        }
        cnt++;
      }
#if JVET_AD0213_LIC_IMP
      else
      {
        mrgCtx.initMrgCand(cnt);
      }
#endif
    }
  }
#endif
#endif

  if (cnt != maxNumMergeCandMin1)
  {
#if !JVET_Z0075_IBC_HMVP_ENLARGE
    bool isGt4x4 = true;
#endif
    bool bFound = addBMMergeHMVPCand(cs, mrgCtx, mrgCandIdx, maxNumMergeCandMin1, cnt
      , isAvailableA1, miLeft, isAvailableB1, miAbove
#if !JVET_Z0075_IBC_HMVP_ENLARGE
      , CU::isIBC(*pu.cu)
      , isGt4x4
#endif
#if TM_MRG
      , mvThreshod
#endif
    );

    if (bFound)
    {
      return;
    }
  }

#if !JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC)
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
  if (!(pu.cs->sps->getUseAML() && pu.cs->sps->getTMToolsEnableFlag()))
#else
  if (!pu.cs->sps->getUseAML())
#endif
#endif
  {
    if (cnt > 1 && cnt < maxNumMergeCand)
    {
      mrgCtx.mvFieldNeighbours[cnt * 2].setMvField(Mv(0, 0), NOT_VALID);
      mrgCtx.mvFieldNeighbours[cnt * 2 + 1].setMvField(Mv(0, 0), NOT_VALID);
#if INTER_LIC
      mrgCtx.licFlags[cnt] = false;
#endif
#if MULTI_HYP_PRED
      mrgCtx.addHypNeighbours[cnt].clear();
#endif
      mrgCtx.bcwIdx[cnt] = BCW_DEFAULT;
      // calculate average MV for L0 and L1 seperately
      unsigned char interDir = 0;
      mrgCtx.useAltHpelIf[cnt] = (mrgCtx.useAltHpelIf[0] == mrgCtx.useAltHpelIf[1]) ? mrgCtx.useAltHpelIf[0] : false;
      for (int refListId = 0; refListId < (slice.isInterB() ? 2 : 1); refListId++)
      {
        const short refIdxI = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].refIdx;
        const short refIdxJ = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].refIdx;

        // both MVs are invalid, skip
        if (refIdxI != refIdxJ)
        {
          continue;
        }

        interDir += 1 << refListId;
        const Mv& mvI = mrgCtx.mvFieldNeighbours[0 * 2 + refListId].mv;
        const Mv& mvJ = mrgCtx.mvFieldNeighbours[1 * 2 + refListId].mv;

        // average two MVs
        Mv avgMv = mvI;
        avgMv += mvJ;
        roundAffineMv(avgMv.hor, avgMv.ver, 1);

        mrgCtx.mvFieldNeighbours[cnt * 2 + refListId].setMvField(avgMv, refIdxI);
      }

      mrgCtx.interDirNeighbours[cnt] = interDir;
      if (interDir == 3 && isBiPredFromDifferentDirEqDistPoc(pu, mrgCtx.mvFieldNeighbours[cnt * 2 + 0].refIdx, mrgCtx.mvFieldNeighbours[cnt * 2 + 1].refIdx))
      {
#if NON_ADJACENT_MRG_CAND || TM_MRG
        if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod))
#endif
          cnt++;
#if JVET_AD0213_LIC_IMP
        else
        {
          mrgCtx.initMrgCand(cnt);
        }
#endif
      }
    }

    // early termination
    if (cnt == maxNumMergeCand)
    {
      mrgCtx.numValidMergeCand = cnt;
      return;
    }
  }
#endif

  mrgCtx.numCandToTestEnc = cnt;

#if JVET_Y0128_NON_CTC
  while(cnt < maxNumMergeCand)
#else
  if (cnt < maxNumMergeCand)
#endif
  {
    mrgCtx.interDirNeighbours[cnt] = 3;
    mrgCtx.bcwIdx[cnt] = BCW_DEFAULT;
#if INTER_LIC
    mrgCtx.licFlags[cnt] = false;
#endif
#if MULTI_HYP_PRED
    mrgCtx.addHypNeighbours[cnt].clear();
#endif
    mrgCtx.useAltHpelIf[cnt] = false;
#if JVET_Y0128_NON_CTC
    mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(Mv(0, 0), pu.cs->slice->getBMDefaultRefIdx(0));
    mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(Mv(0, 0), pu.cs->slice->getBMDefaultRefIdx(1));
#else
    mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(Mv(0, 0), 0);
    mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(Mv(0, 0), 0);
#endif
#if !JVET_Y0128_NON_CTC
#if NON_ADJACENT_MRG_CAND || TM_MRG
    if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod))
#endif
#endif
    {
      if (mrgCandIdx == cnt)
      {
        mrgCtx.numValidMergeCand = cnt + 1;
        return;
      }
      cnt++;
    }
  }
  mrgCtx.numValidMergeCand = cnt;
}
#endif

#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
void PU::getTmvpBMCand(const PredictionUnit &pu, MergeCtx& mrgCtx)
{
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  if (!pu.cs->sps->getUseTmvpNmvpReordering())
  {
    mrgCtx.numValidMergeCand = 0;
    return;
  }
#endif

  const CodingStructure &cs = *pu.cs;
  const Slice &slice = *pu.cs->slice;

  const uint32_t mvThreshod = 1;
  const uint32_t maxNumMergeCand = NUM_TMVP_CANDS;
  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
    mrgCtx.bcwIdx[ui] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
    mrgCtx.altLMFlag[ui] = false;
    mrgCtx.altLMParaNeighbours[ui].resetAltLinearModel();
#endif
#if INTER_LIC
    mrgCtx.licFlags[ui] = false;
#endif
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mvFieldNeighbours[(ui << 1)].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
    mrgCtx.useAltHpelIf[ui] = false;
#if MULTI_HYP_PRED
    mrgCtx.addHypNeighbours[ui].clear();
#endif
    mrgCtx.candCost[ui] = MAX_UINT64;
  }

  int cnt = 0;

#if INTER_RM_SIZE_CONSTRAINTS
  if (slice.getPicHeader()->getEnableTMVPFlag())
#else
  if (slice.getPicHeader()->getEnableTMVPFlag() && (pu.lumaSize().width + pu.lumaSize().height > 12))
#endif
  {
    //>> MTK colocated-RightBottom
    // offset the pos to be sure to "point" to the same position the uiAbsPartIdx would've pointed to
    const PreCalcValues& pcv = *cs.pcv;
    bool isC0Avail;
    bool isC1Avail;
    bool boundaryCond;
    const SubPic& curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
    Position posRB = pu.Y().bottomRight().offset(-3, -3);
    Position posCenter = pu.Y().center();
    Position posC0;
    Position posC1;

    int       iRefIdx = 0;
    bool      bExistMV0, bExistMV1;
    Mv        cColMv0, cColMv1;
    int       dir;

    int offsetX0 = 0, offsetX1 = 0, offsetX2 = 0, offsetX3 = pu.Y().width >> 1;
    int offsetY0 = 0, offsetY1 = 0, offsetY2 = 0, offsetY3 = pu.Y().height >> 1;

    const int numNACandidate[5] = { 2, 2, 2, 2, 2 };
    const int idxMap[5][2] = { { 0, 1 },{ 0, 2 },{ 0, 2 },{ 0, 2 },{ 0, 2 } };
    for (int iDistanceIndex = 0; iDistanceIndex < TMVP_DISTANCE_LEVEL && cnt < maxNumMergeCand; iDistanceIndex++)
    {
      const int iNADistanceHor = pu.Y().width  * iDistanceIndex;
      const int iNADistanceVer = pu.Y().height * iDistanceIndex;

      for (int iNASPIdx = 0; iNASPIdx < numNACandidate[iDistanceIndex] && cnt < maxNumMergeCand; iNASPIdx++)
      {
        switch (idxMap[iDistanceIndex][iNASPIdx])
        {
        case 0: offsetX0 = offsetX2 = 4 + iNADistanceHor; offsetY0 = offsetY2 = 4 + iNADistanceVer; offsetX1 = iNADistanceHor; offsetY1 = iNADistanceVer; break;
        case 1: offsetX0 = 4; offsetY0 = 0; offsetX1 = 0; offsetY1 = 4; break;
        case 2: offsetX0 = offsetX2; offsetY0 = 4 - offsetY3; offsetX1 = 4 - offsetX3; offsetY1 = offsetY2; break;
        default: printf("error!"); exit(0); break;
        }
        isC0Avail = false;
        if (curSubPic.getTreatedAsPicFlag())
        {
          boundaryCond = ((posRB.x + offsetX0) <= curSubPic.getSubPicRight() && (posRB.y + offsetY0) <= curSubPic.getSubPicBottom());
        }
        else
        {
          boundaryCond = ((posRB.x + offsetX0) < pcv.lumaWidth) && ((posRB.y + offsetY0) < pcv.lumaHeight);
        }
        if (boundaryCond)
        {
          int posYInCtu = posRB.y & pcv.maxCUHeightMask;
          if (posYInCtu + offsetY0 < pcv.maxCUHeight)
          {
            posC0 = posRB.offset(offsetX0, offsetY0);
            isC0Avail = true;
          }
        }

        if (idxMap[iDistanceIndex][iNASPIdx] == 0)
        {
          isC1Avail = false;
          if (curSubPic.getTreatedAsPicFlag())
          {
            boundaryCond = ((posCenter.x + offsetX1) <= curSubPic.getSubPicRight() && (posCenter.y + offsetY1) <= curSubPic.getSubPicBottom());
          }
          else
          {
            boundaryCond = ((posCenter.x + offsetX1) < pcv.lumaWidth) && ((posCenter.y + offsetY1) < pcv.lumaHeight);
          }
          if (boundaryCond)
          {
            int posYInCtu = posCenter.y & pcv.maxCUHeightMask;
            if (posYInCtu + offsetY1 < pcv.maxCUHeight)
            {
              posC1 = posCenter.offset(offsetX1, offsetY1);
              isC1Avail = true; 
            }
          }
        }
        else
        {
          isC1Avail = false;
          if (curSubPic.getTreatedAsPicFlag())
          {
            boundaryCond = ((posRB.x + offsetX1) <= curSubPic.getSubPicRight() && (posRB.y + offsetY1) <= curSubPic.getSubPicBottom());
          }
          else
          {
            boundaryCond = ((posRB.x + offsetX1) < pcv.lumaWidth) && ((posRB.y + offsetY1) < pcv.lumaHeight);
          }
          if (boundaryCond)
          {
            int posYInCtu = posRB.y & pcv.maxCUHeightMask;
            if (posYInCtu + offsetY1 < pcv.maxCUHeight)
            {
              posC1 = posRB.offset(offsetX1, offsetY1);
              isC1Avail = true;
            }
          }
        }

        bExistMV0 = bExistMV1 = false;

        // Candidate with L0 and L1
        dir = 0;
        bExistMV0 = (isC0Avail && getColocatedMVP(pu, REF_PIC_LIST_0, posC0, cColMv0, iRefIdx, false
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
          , 0
#endif
        ))
          || (isC1Avail && getColocatedMVP(pu, REF_PIC_LIST_0, posC1, cColMv0, iRefIdx, false
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
            , 0
#endif
          ));
        if (bExistMV0)
        {
          dir |= 1;
          mrgCtx.mvFieldNeighbours[2 * cnt].setMvField(cColMv0, iRefIdx);
        }
        else
        {
          mrgCtx.mvFieldNeighbours[2 * cnt].setMvField(Mv(), NOT_VALID);
        }
        if (slice.isInterB())
        {
          bExistMV1 = (isC0Avail && getColocatedMVP(pu, REF_PIC_LIST_1, posC0, cColMv1, iRefIdx, false
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
            , 0
#endif
          ))
            || (isC1Avail && getColocatedMVP(pu, REF_PIC_LIST_1, posC1, cColMv1, iRefIdx, false
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
              , 0
#endif
            ));
          if (bExistMV1)
          {
            dir |= 2;
            mrgCtx.mvFieldNeighbours[2 * cnt + 1].setMvField(cColMv1, iRefIdx);
          }
          else
          {
            mrgCtx.mvFieldNeighbours[2 * cnt + 1].setMvField(Mv(), NOT_VALID);
          }
        }
        if (dir != 0)
        {
          bool addTMvp = isBiPredFromDifferentDirEqDistPoc(pu, mrgCtx.mvFieldNeighbours[2 * cnt + 0].refIdx, mrgCtx.mvFieldNeighbours[2 * cnt + 1].refIdx);
          if (addTMvp)
          {
            mrgCtx.interDirNeighbours[cnt] = dir;
            mrgCtx.useAltHpelIf[cnt] = false;
#if MULTI_HYP_PRED
            mrgCtx.addHypNeighbours[cnt].clear();
#endif
            if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod
            ))
            {
              cnt++;
              if (cnt == maxNumMergeCand)
              {
                break;
              }
            }
#if JVET_AD0213_LIC_IMP
            else
            {
              mrgCtx.initMrgCand(cnt);
            }
#endif
          }
        }
      }
    }
}

  mrgCtx.numValidMergeCand = cnt;
}
#endif

#if NON_ADJACENT_MRG_CAND && JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
void PU::getNonAdjacentBMCand(const PredictionUnit &pu, MergeCtx& mrgCtx)
{
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  if (!pu.cs->sps->getUseTmvpNmvpReordering())
  {
    mrgCtx.numValidMergeCand = 0;
    return;
  }
#endif
  const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
  const CodingStructure &cs = *pu.cs;

  const uint32_t mvThreshod = getBDMVRMvdThreshold(pu);

  const uint32_t maxNumMergeCand = NUM_NON_ADJ_CANDS;

  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
    mrgCtx.bcwIdx[ui] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
    mrgCtx.altLMFlag[ui] = false;
    mrgCtx.altLMParaNeighbours[ui].resetAltLinearModel();
#endif
#if INTER_LIC
    mrgCtx.licFlags[ui] = false;
#endif
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mvFieldNeighbours[(ui << 1)].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
    mrgCtx.useAltHpelIf[ui] = false;
#if MULTI_HYP_PRED
    mrgCtx.addHypNeighbours[ui].clear();
#endif
    mrgCtx.candCost[ui] = MAX_UINT64;
  }

  int cnt = 0;

  const Position posLT = pu.Y().topLeft();

  MotionInfo miNeighbor;
  int offsetX = 0;
  int offsetY = 0;
  int offsetX0 = 0; int offsetX1 = 0; int offsetX2 = pu.Y().width >> 1;
  int offsetY0 = 0; int offsetY1 = 0; int offsetY2 = pu.Y().height >> 1;

  const int numNACandidate[7] = { 5, 9, 9, 9, 9, 9, 9 };
  const int idxMap[7][9] = { { 0, 1, 2, 3, 4 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 } };

  for (int iDistanceIndex = 0; iDistanceIndex < 7 && cnt < maxNumMergeCand; iDistanceIndex++)
  {
    const int iNADistanceHor = pu.Y().width  * (iDistanceIndex + 1);
    const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);

    for (int iNASPIdx = 0; iNASPIdx < numNACandidate[iDistanceIndex] && cnt < maxNumMergeCand; iNASPIdx++)
    {
      switch (idxMap[iDistanceIndex][iNASPIdx])
      {
      case 0:offsetX = offsetX0 = -iNADistanceHor - 1;               offsetY = offsetY0 = pu.Y().height + iNADistanceVer - 1; break;
      case 1:offsetX = offsetX1 = pu.Y().width + iNADistanceHor - 1; offsetY = offsetY1 = -iNADistanceVer - 1;                break;
      case 2:offsetX = offsetX2;                                     offsetY = offsetY1;                                      break;
      case 3:offsetX = offsetX0;                                     offsetY = offsetY2;                                      break;
      case 4:offsetX = offsetX0;                                     offsetY = offsetY1;                                      break;
      case 5:offsetX = -1;                                           offsetY = offsetY0;                                      break;
      case 6:offsetX = offsetX1;                                     offsetY = -1;                                            break;
      case 7:offsetX = offsetX0 >> 1;                                offsetY = offsetY0;                                      break;
      case 8:offsetX = offsetX1;                                     offsetY = offsetY1 >> 1;                                 break;
      default: printf("error!"); exit(0); break;
      }

      const PredictionUnit *puNonAdjacent = cs.getPURestricted(posLT.offset(offsetX, offsetY), pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(pu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent->cu) && puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY)).isInter;
#else
      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(pu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent->cu);
#endif

      if (isAvailableNonAdjacent)
      {
        miNeighbor = puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY));

        if (isBiPredFromDifferentDirEqDistPoc(pu, miNeighbor.refIdx[0], miNeighbor.refIdx[1])
          )
        {
          // get Inter Dir
          mrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
          mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
          mrgCtx.useAltHpelIf[cnt] = miNeighbor.useAltHpelIf;
          mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);

          if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod))
          {
            cnt++;
          }
#if JVET_AD0213_LIC_IMP
          else
          {
            mrgCtx.initMrgCand(cnt);
          }
#endif
        }
      }
    }
  }

  const int numNACandidate2[7] = { 4, 4, 4, 4, 4, 4, 4 };
  const int idxMap2[7][5] = { { 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 } };

  for (int iDistanceIndex = 0; iDistanceIndex < 7 && cnt < maxNumMergeCand; iDistanceIndex++)
  {
    const int iNADistanceHor = pu.Y().width  * (iDistanceIndex + 1);
    const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);

    for (int iNASPIdx = 0; iNASPIdx < numNACandidate2[iDistanceIndex] && cnt < maxNumMergeCand; iNASPIdx++)
    {
      switch (idxMap2[iDistanceIndex][iNASPIdx])
      {
      case 0:offsetX = offsetX0 = -iNADistanceHor - 1;                                                         offsetY = offsetY2 + ((pu.Y().height + iNADistanceVer - 1 - offsetY2) >> 1); break;
      case 1:offsetX = offsetX2 + ((pu.Y().width + iNADistanceHor - 1 - offsetX2) >> 1);                       offsetY = offsetY0 = -iNADistanceVer - 1; break;
      case 2:offsetX = offsetX0;                                                                               offsetY = offsetY0 + ((offsetY2 - offsetY0) >> 1); break;
      case 3:offsetX = offsetX0 + ((offsetX2 - offsetX0) >> 1);                                                offsetY = offsetY0; break;
      default: printf("error!"); exit(0); break;
      }

      const PredictionUnit *puNonAdjacent = cs.getPURestricted(posLT.offset(offsetX, offsetY), pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(pu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent->cu) && puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY)).isInter;
#else
      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(pu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent->cu);
#endif

      if (isAvailableNonAdjacent)
      {
        miNeighbor = puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY));

        if (isBiPredFromDifferentDirEqDistPoc(pu, miNeighbor.refIdx[0], miNeighbor.refIdx[1])
          )
        {
          // get Inter Dir
          mrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
          // get Mv from Above-Left
          mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
          mrgCtx.useAltHpelIf[cnt] = miNeighbor.useAltHpelIf;
          mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);

          if (!mrgCtx.xCheckSimilarMotion(cnt, mvThreshod))
          {
            cnt++;
          }
#if JVET_AD0213_LIC_IMP
          else
          {
            mrgCtx.initMrgCand(cnt);
          }
#endif
        }
      }
    }
  }

  mrgCtx.numValidMergeCand = cnt;
}
#endif

#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
void PU::getTmvpMergeCand(const PredictionUnit &pu, MergeCtx& mrgCtx)
{
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  if (!pu.cs->sps->getUseTmvpNmvpReordering())
  {
    mrgCtx.numValidMergeCand = 0;
    return;
  }
#endif
  const CodingStructure &cs = *pu.cs;
  const Slice &slice = *pu.cs->slice;

#if TM_MRG
  const uint32_t mvdSimilarityThresh = 1;
#endif
#if TM_MRG && JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
  const uint32_t maxNumMergeCand =
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
                                   pu.cs->sps->getUseTMMrgMode() &&
#endif
                                   pu.tmMergeFlag ? 1 : NUM_TMVP_CANDS;
#else
  const uint32_t maxNumMergeCand = NUM_TMVP_CANDS;
#endif
  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
    mrgCtx.bcwIdx[ui] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
    mrgCtx.altLMFlag[ui] = false;
    mrgCtx.altLMParaNeighbours[ui].resetAltLinearModel();
#endif
#if INTER_LIC
    mrgCtx.licFlags[ui] = false;
#endif
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mvFieldNeighbours[(ui << 1)].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
    mrgCtx.useAltHpelIf[ui] = false;
#if MULTI_HYP_PRED
    mrgCtx.addHypNeighbours[ui].clear();
#endif
    mrgCtx.candCost[ui] = MAX_UINT64;
  }

  int cnt = 0;
#if INTER_RM_SIZE_CONSTRAINTS
  if (slice.getPicHeader()->getEnableTMVPFlag())
#else
  if (slice.getPicHeader()->getEnableTMVPFlag() && (pu.lumaSize().width + pu.lumaSize().height > 12))
#endif
  {
    //>> MTK colocated-RightBottom
    // offset the pos to be sure to "point" to the same position the uiAbsPartIdx would've pointed to
    const PreCalcValues& pcv = *cs.pcv;
    bool isC0Avail;
    bool isC1Avail;
    bool boundaryCond;
    const SubPic& curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
    Position posRB = pu.Y().bottomRight().offset(-3, -3);
    Position posCenter = pu.Y().center();
    Position posC0;
    Position posC1;

    int       iRefIdx = 0;
    bool      bExistMV0, bExistMV1;
    Mv        cColMv0, cColMv1;
    int       dir;

    int offsetX0 = 0, offsetX1 = 0, offsetX2 = 0, offsetX3 = pu.Y().width >> 1;
    int offsetY0 = 0, offsetY1 = 0, offsetY2 = 0, offsetY3 = pu.Y().height >> 1;

    const int numNACandidate[5] = { 2, 2, 2, 2, 2 };
    const int idxMap[5][2] = { { 0, 1 },{ 0, 2 },{ 0, 2 },{ 0, 2 },{ 0, 2 } };
    for (int iDistanceIndex = 0; iDistanceIndex < TMVP_DISTANCE_LEVEL && cnt < maxNumMergeCand; iDistanceIndex++)
    {
      const int iNADistanceHor = pu.Y().width  * iDistanceIndex;
      const int iNADistanceVer = pu.Y().height * iDistanceIndex;

      for (int iNASPIdx = 0; iNASPIdx < numNACandidate[iDistanceIndex] && cnt < maxNumMergeCand; iNASPIdx++)
      {
        switch (idxMap[iDistanceIndex][iNASPIdx])
        {
        case 0: offsetX0 = offsetX2 = 4 + iNADistanceHor; offsetY0 = offsetY2 = 4 + iNADistanceVer; offsetX1 = iNADistanceHor; offsetY1 = iNADistanceVer; break;
        case 1: offsetX0 = 4; offsetY0 = 0; offsetX1 = 0; offsetY1 = 4; break;
        case 2: offsetX0 = offsetX2; offsetY0 = 4 - offsetY3; offsetX1 = 4 - offsetX3; offsetY1 = offsetY2; break;
        default: printf("error!"); exit(0); break;
        }
        isC0Avail = false;
        if (curSubPic.getTreatedAsPicFlag())
        {
          boundaryCond = ((posRB.x + offsetX0) <= curSubPic.getSubPicRight() && (posRB.y + offsetY0) <= curSubPic.getSubPicBottom());
        }
        else
        {
          boundaryCond = ((posRB.x + offsetX0) < pcv.lumaWidth) && ((posRB.y + offsetY0) < pcv.lumaHeight);
        }
        if (boundaryCond)
        {
          int posYInCtu = posRB.y & pcv.maxCUHeightMask;
          if (posYInCtu + offsetY0 < pcv.maxCUHeight)
          {
            posC0 = posRB.offset(offsetX0, offsetY0);
            isC0Avail = true;
          }
        }

        if (idxMap[iDistanceIndex][iNASPIdx] == 0)
        {
          isC1Avail = false;
          if (curSubPic.getTreatedAsPicFlag())
          {
            boundaryCond = ((posCenter.x + offsetX1) <= curSubPic.getSubPicRight() && (posCenter.y + offsetY1) <= curSubPic.getSubPicBottom());
          }
          else
          {
            boundaryCond = ((posCenter.x + offsetX1) < pcv.lumaWidth) && ((posCenter.y + offsetY1) < pcv.lumaHeight);
          }
          if (boundaryCond)
          {
            int posYInCtu = posCenter.y & pcv.maxCUHeightMask;
            if (posYInCtu + offsetY1 < pcv.maxCUHeight)
            {
              posC1 = posCenter.offset(offsetX1, offsetY1);
              isC1Avail = true;
            }
          }
        }
        else
        {
          isC1Avail = false;
          if (curSubPic.getTreatedAsPicFlag())
          {
            boundaryCond = ((posRB.x + offsetX1) <= curSubPic.getSubPicRight() && (posRB.y + offsetY1) <= curSubPic.getSubPicBottom());
          }
          else
          {
            boundaryCond = ((posRB.x + offsetX1) < pcv.lumaWidth) && ((posRB.y + offsetY1) < pcv.lumaHeight);
          }
          if (boundaryCond)
          {
            int posYInCtu = posRB.y & pcv.maxCUHeightMask;
            if (posYInCtu + offsetY1 < pcv.maxCUHeight)
            {
              posC1 = posRB.offset(offsetX1, offsetY1);
              isC1Avail = true;
            }
          }
        }

        bExistMV0 = bExistMV1 = false;

        // Candidate with L0 and L1
        dir = 0;
        int refIdx[NUM_REF_PIC_LIST_01] = { 0, 0 };
        bExistMV0 = (isC0Avail && getColocatedMVP(pu, REF_PIC_LIST_0, posC0, cColMv0, iRefIdx, false,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
          0,
#endif
          &refIdx[REF_PIC_LIST_0]))
          || (isC1Avail && getColocatedMVP(pu, REF_PIC_LIST_0, posC1, cColMv0, iRefIdx, false,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
            0,
#endif
            &refIdx[REF_PIC_LIST_0]));

        if (bExistMV0)
        {
          dir |= 1;
          mrgCtx.mvFieldNeighbours[2 * cnt].setMvField(cColMv0, refIdx[REF_PIC_LIST_0]);
        }
        else
        {
          mrgCtx.mvFieldNeighbours[2 * cnt].setMvField(Mv(), NOT_VALID);
        }
        if (slice.isInterB())
        {
          bExistMV1 = (isC0Avail && getColocatedMVP(pu, REF_PIC_LIST_1, posC0, cColMv1, iRefIdx, false,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
            0,
#endif
            &refIdx[REF_PIC_LIST_1]))
            || (isC1Avail && getColocatedMVP(pu, REF_PIC_LIST_1, posC1, cColMv1, iRefIdx, false,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
              0,
#endif
              &refIdx[REF_PIC_LIST_1]));

          if (bExistMV1)
          {
            dir |= 2;
            mrgCtx.mvFieldNeighbours[2 * cnt + 1].setMvField(cColMv1, refIdx[REF_PIC_LIST_1]);
          }
          else
          {
            mrgCtx.mvFieldNeighbours[2 * cnt + 1].setMvField(Mv(), NOT_VALID);
          }
        }
        if (dir != 0)
        {
          mrgCtx.interDirNeighbours[cnt] = dir;
          mrgCtx.bcwIdx[cnt] = BCW_DEFAULT;
#if INTER_LIC
          mrgCtx.licFlags[cnt] = false;
#endif
          mrgCtx.useAltHpelIf[cnt] = false;
#if MULTI_HYP_PRED
          mrgCtx.addHypNeighbours[cnt].clear();
#endif
#if NON_ADJACENT_MRG_CAND || TM_MRG
          if (!mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG
            , mvdSimilarityThresh
#endif
          ))
          {
#endif
            cnt++;
            if (cnt == maxNumMergeCand)
            {
              break;
            }
#if NON_ADJACENT_MRG_CAND || TM_MRG
          }
#if JVET_AD0213_LIC_IMP
          else
          {
            mrgCtx.initMrgCand(cnt);
          }
#endif
#endif
        }
        if (!slice.getCheckLDC() && bExistMV0 && bExistMV1)
        {
          // Candidate without L1
          dir = 0;
          dir |= 1;
          mrgCtx.mvFieldNeighbours[2 * cnt].setMvField(cColMv0, refIdx[REF_PIC_LIST_0]);
          mrgCtx.mvFieldNeighbours[2 * cnt + 1].setMvField(Mv(), NOT_VALID);
          if (dir != 0)
          {
            mrgCtx.interDirNeighbours[cnt] = dir;
            mrgCtx.bcwIdx[cnt] = BCW_DEFAULT;
#if INTER_LIC
            mrgCtx.licFlags[cnt] = false;
#endif
            mrgCtx.useAltHpelIf[cnt] = false;
#if MULTI_HYP_PRED
            mrgCtx.addHypNeighbours[cnt].clear();
#endif
#if NON_ADJACENT_MRG_CAND || TM_MRG
            if (!mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG
              , mvdSimilarityThresh
#endif
            ))
            {
#endif
              cnt++;
              if (cnt == maxNumMergeCand) break;
#if NON_ADJACENT_MRG_CAND || TM_MRG
            }
#if JVET_AD0213_LIC_IMP
            else
            {
              mrgCtx.initMrgCand(cnt);
            }
#endif
#endif
          }

          // Candidate without L0
          dir = 0;
          mrgCtx.mvFieldNeighbours[2 * cnt].setMvField(Mv(), NOT_VALID);
          dir |= 2;
          mrgCtx.mvFieldNeighbours[2 * cnt + 1].setMvField(cColMv1, refIdx[REF_PIC_LIST_1]);
          if (dir != 0)
          {
            mrgCtx.interDirNeighbours[cnt] = dir;
            mrgCtx.bcwIdx[cnt] = BCW_DEFAULT;
#if INTER_LIC
            mrgCtx.licFlags[cnt] = false;
#endif
            mrgCtx.useAltHpelIf[cnt] = false;
#if MULTI_HYP_PRED
            mrgCtx.addHypNeighbours[cnt].clear();
#endif
#if NON_ADJACENT_MRG_CAND || TM_MRG
            if (!mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG
              , mvdSimilarityThresh
#endif
            ))
            {
#endif
              cnt++;
              if (cnt == maxNumMergeCand)
              {
                break;
              }
#if NON_ADJACENT_MRG_CAND || TM_MRG
            }
#if JVET_AD0213_LIC_IMP
            else
            {
              mrgCtx.initMrgCand(cnt);
            }
#endif
#endif
          }
        }
      }
    }
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    if (pu.tmMergeFlag)
    {
      offsetX0 = 0, offsetX1 = 0, offsetX2 = 0, offsetX3 = pu.Y().width >> 1;
      offsetY0 = 0, offsetY1 = 0, offsetY2 = 0, offsetY3 = pu.Y().height >> 1;

      for (int iDistanceIndex = 0; iDistanceIndex < TMVP_DISTANCE_LEVEL && cnt < (maxNumMergeCand + 1); iDistanceIndex++)
      {
        const int iNADistanceHor = pu.Y().width  * iDistanceIndex;
        const int iNADistanceVer = pu.Y().height * iDistanceIndex;

        for (int iNASPIdx = 0; iNASPIdx < numNACandidate[iDistanceIndex] && cnt < (maxNumMergeCand + 1); iNASPIdx++)
        {
          switch (idxMap[iDistanceIndex][iNASPIdx])
          {
          case 0: offsetX0 = offsetX2 = 4 + iNADistanceHor; offsetY0 = offsetY2 = 4 + iNADistanceVer; offsetX1 = iNADistanceHor; offsetY1 = iNADistanceVer; break;
          case 1: offsetX0 = 4; offsetY0 = 0; offsetX1 = 0; offsetY1 = 4; break;
          case 2: offsetX0 = offsetX2; offsetY0 = 4 - offsetY3; offsetX1 = 4 - offsetX3; offsetY1 = offsetY2; break;
          default: printf("error!"); exit(0); break;
          }
          isC0Avail = false;
          if (curSubPic.getTreatedAsPicFlag())
          {
            boundaryCond = ((posRB.x + offsetX0) <= curSubPic.getSubPicRight() && (posRB.y + offsetY0) <= curSubPic.getSubPicBottom());
          }
          else
          {
            boundaryCond = ((posRB.x + offsetX0) < pcv.lumaWidth) && ((posRB.y + offsetY0) < pcv.lumaHeight);
          }
          if (boundaryCond)
          {
            int posYInCtu = posRB.y & pcv.maxCUHeightMask;
            if (posYInCtu + offsetY0 < pcv.maxCUHeight)
            {
              posC0 = posRB.offset(offsetX0, offsetY0);
              isC0Avail = true;
            }
          }

          if (idxMap[iDistanceIndex][iNASPIdx] == 0)
          {
            isC1Avail = false;
            if (curSubPic.getTreatedAsPicFlag())
            {
              boundaryCond = ((posCenter.x + offsetX1) <= curSubPic.getSubPicRight() && (posCenter.y + offsetY1) <= curSubPic.getSubPicBottom());
            }
            else
            {
              boundaryCond = ((posCenter.x + offsetX1) < pcv.lumaWidth) && ((posCenter.y + offsetY1) < pcv.lumaHeight);
            }
            if (boundaryCond)
            {
              int posYInCtu = posCenter.y & pcv.maxCUHeightMask;
              if (posYInCtu + offsetY1 < pcv.maxCUHeight)
              {
                posC1 = posCenter.offset(offsetX1, offsetY1);
                isC1Avail = true;
              }
            }
          }
          else
          {
            isC1Avail = false;
            if (curSubPic.getTreatedAsPicFlag())
            {
              boundaryCond = ((posRB.x + offsetX1) <= curSubPic.getSubPicRight() && (posRB.y + offsetY1) <= curSubPic.getSubPicBottom());
            }
            else
            {
              boundaryCond = ((posRB.x + offsetX1) < pcv.lumaWidth) && ((posRB.y + offsetY1) < pcv.lumaHeight);
            }
            if (boundaryCond)
            {
              int posYInCtu = posRB.y & pcv.maxCUHeightMask;
              if (posYInCtu + offsetY1 < pcv.maxCUHeight)
              {
                posC1 = posRB.offset(offsetX1, offsetY1);
                isC1Avail = true;
              }
            }
          }

          bExistMV0 = bExistMV1 = false;

          // Candidate with L0 and L1
          dir = 0;
          int refIdx[NUM_REF_PIC_LIST_01] = { 0, 0 };
          bExistMV0 = (isC0Avail && getColocatedMVP(pu, REF_PIC_LIST_0, posC0, cColMv0, iRefIdx, false,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
            pu.cu->slice->isInterB(),
#endif
            &refIdx[REF_PIC_LIST_0]))
            || (isC1Avail && getColocatedMVP(pu, REF_PIC_LIST_0, posC1, cColMv0, iRefIdx, false,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
              pu.cu->slice->isInterB(),
#endif
              &refIdx[REF_PIC_LIST_0]));

          if (bExistMV0)
          {
            dir |= 1;
            mrgCtx.mvFieldNeighbours[2 * cnt].setMvField(cColMv0, refIdx[REF_PIC_LIST_0]);
          }
          else
          {
            mrgCtx.mvFieldNeighbours[2 * cnt].setMvField(Mv(), NOT_VALID);
          }
          if (slice.isInterB())
          {
            bExistMV1 = (isC0Avail && getColocatedMVP(pu, REF_PIC_LIST_1, posC0, cColMv1, iRefIdx, false,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
              1,
#endif
              &refIdx[REF_PIC_LIST_1]))
              || (isC1Avail && getColocatedMVP(pu, REF_PIC_LIST_1, posC1, cColMv1, iRefIdx, false,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
                1,
#endif
                &refIdx[REF_PIC_LIST_1]));

            if (bExistMV1)
            {
              dir |= 2;
              mrgCtx.mvFieldNeighbours[2 * cnt + 1].setMvField(cColMv1, refIdx[REF_PIC_LIST_1]);
            }
            else
            {
              mrgCtx.mvFieldNeighbours[2 * cnt + 1].setMvField(Mv(), NOT_VALID);
            }
          }
          if (dir != 0)
          {
            mrgCtx.interDirNeighbours[cnt] = dir;
            mrgCtx.bcwIdx[cnt] = BCW_DEFAULT;
#if INTER_LIC
            mrgCtx.licFlags[cnt] = false;
#endif
            mrgCtx.useAltHpelIf[cnt] = false;
#if MULTI_HYP_PRED
            mrgCtx.addHypNeighbours[cnt].clear();
#endif
#if NON_ADJACENT_MRG_CAND || TM_MRG
            if (!mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG
              , mvdSimilarityThresh
#endif
            ))
            {
#endif
              cnt++;
              if (cnt == (maxNumMergeCand + 1))
              {
                break;
              }
#if NON_ADJACENT_MRG_CAND || TM_MRG
            }
#endif
          }

          if (!slice.getCheckLDC() && bExistMV0 && bExistMV1)
          {
            // Candidate without L1
            dir = 0;
            dir |= 1;
            mrgCtx.mvFieldNeighbours[2 * cnt].setMvField(cColMv0, refIdx[REF_PIC_LIST_0]);
            mrgCtx.mvFieldNeighbours[2 * cnt + 1].setMvField(Mv(), NOT_VALID);
            if (dir != 0)
            {
              mrgCtx.interDirNeighbours[cnt] = dir;
              mrgCtx.bcwIdx[cnt] = BCW_DEFAULT;
#if INTER_LIC
              mrgCtx.licFlags[cnt] = false;
#endif
              mrgCtx.useAltHpelIf[cnt] = false;
#if MULTI_HYP_PRED
              mrgCtx.addHypNeighbours[cnt].clear();
#endif
#if NON_ADJACENT_MRG_CAND || TM_MRG
              if (!mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG
                , mvdSimilarityThresh
#endif
              ))
              {
#endif
                cnt++;
                if (cnt == (maxNumMergeCand + 1)) break;
#if NON_ADJACENT_MRG_CAND || TM_MRG
              }
#endif
            }

            // Candidate without L0
            dir = 0;
            mrgCtx.mvFieldNeighbours[2 * cnt].setMvField(Mv(), NOT_VALID);
            dir |= 2;
            mrgCtx.mvFieldNeighbours[2 * cnt + 1].setMvField(cColMv1, refIdx[REF_PIC_LIST_1]);
            if (dir != 0)
            {
              mrgCtx.interDirNeighbours[cnt] = dir;
              mrgCtx.bcwIdx[cnt] = BCW_DEFAULT;
#if INTER_LIC
              mrgCtx.licFlags[cnt] = false;
#endif
              mrgCtx.useAltHpelIf[cnt] = false;
#if MULTI_HYP_PRED
              mrgCtx.addHypNeighbours[cnt].clear();
#endif
#if NON_ADJACENT_MRG_CAND || TM_MRG
              if (!mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG
                , mvdSimilarityThresh
#endif
              ))
              {
#endif
                cnt++;
                if (cnt == (maxNumMergeCand + 1))
                {
                  break;
                }
#if NON_ADJACENT_MRG_CAND || TM_MRG
              }
#endif
            }
          }
        }
      }
    }
#endif
  }

  mrgCtx.numValidMergeCand = cnt;
}
#endif

#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM && NON_ADJACENT_MRG_CAND
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
void PU::getNonAdjacentMergeCandSubTMVP(const PredictionUnit &pu, MergeCtx& mrgCtx, int col)
{
  const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
  const CodingStructure &cs = *pu.cs;
  const Slice &slice = *pu.cs->slice;

  const uint32_t mvdSimilarityThresh = SUB_TMVP_MV_THRESHOLD;
  const uint32_t maxNumMergeCand = NUM_NON_ADJ_CANDS;

  const Picture *pColPic = slice.getRefPic(RefPicList(col == 0 ? 1 - slice.getColFromL0Flag() : 1 - slice.getColFromL0Flag2nd()), col == 0 ? slice.getColRefIdx() : slice.getColRefIdx2nd());

  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
    mrgCtx.bcwIdx[ui] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
    mrgCtx.altLMFlag[ui] = false;
    mrgCtx.altLMParaNeighbours[ui].resetAltLinearModel();
#endif
#if INTER_LIC
    mrgCtx.licFlags[ui] = false;
#endif
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mvFieldNeighbours[(ui << 1)].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
    mrgCtx.useAltHpelIf[ui] = false;
#if MULTI_HYP_PRED
    mrgCtx.addHypNeighbours[ui].clear();
#endif
    mrgCtx.candCost[ui] = MAX_UINT64;
  }

  int cnt = 0;

  const Position posLT = pu.Y().topLeft();

  MotionInfo miNeighbor;
  int offsetX = 0;
  int offsetY = 0;
  int offsetX0 = 0; int offsetX1 = 0; int offsetX2 = pu.Y().width >> 1;
  int offsetY0 = 0; int offsetY1 = 0; int offsetY2 = pu.Y().height >> 1;

  const int numNACandidate[7] = { 5, 9, 9, 9, 9, 9, 9 };
  const int idxMap[7][9] = { { 0, 1, 2, 3, 4 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 } };

  for (int iDistanceIndex = 0; iDistanceIndex < 7 && cnt < maxNumMergeCand; iDistanceIndex++)
  {
    const int iNADistanceHor = pu.Y().width  * (iDistanceIndex + 1);
    const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);

    for (int naSpIdx = 0; naSpIdx < numNACandidate[iDistanceIndex] && cnt < maxNumMergeCand; naSpIdx++)
    {
      switch( idxMap[iDistanceIndex][naSpIdx] )
      {
      case 0:offsetX = offsetX0 = -iNADistanceHor - 1;               offsetY = offsetY0 = pu.Y().height + iNADistanceVer - 1; break;
      case 1:offsetX = offsetX1 = pu.Y().width + iNADistanceHor - 1; offsetY = offsetY1 = -iNADistanceVer - 1;                break;
      case 2:offsetX = offsetX2;                                     offsetY = offsetY1;                                      break;
      case 3:offsetX = offsetX0;                                     offsetY = offsetY2;                                      break;
      case 4:offsetX = offsetX0;                                     offsetY = offsetY1;                                      break;
      case 5:offsetX = -1;                                           offsetY = offsetY0;                                      break;
      case 6:offsetX = offsetX1;                                     offsetY = -1;                                            break;
      case 7:offsetX = offsetX0 >> 1;                                offsetY = offsetY0;                                      break;
      case 8:offsetX = offsetX1;                                     offsetY = offsetY1 >> 1;                                 break;
      default: printf("error!"); exit(0); break;
      }

      const PredictionUnit *puNonAdjacent = cs.getPURestricted(posLT.offset(offsetX, offsetY), pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER( pu.lumaPos(), posLT.offset( offsetX, offsetY ), plevel ) && CU::isInter( *puNonAdjacent->cu ) && puNonAdjacent->getMotionInfo( posLT.offset( offsetX, offsetY ) ).isInter;
#else
      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER( pu.lumaPos(), posLT.offset( offsetX, offsetY ), plevel ) && CU::isInter( *puNonAdjacent->cu );
#endif

      if (isAvailableNonAdjacent)
      {
        miNeighbor = puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY));

        int index = 0;
        if ((miNeighbor.interDir & (1 << REF_PIC_LIST_0)) && slice.getRefPic(REF_PIC_LIST_0, miNeighbor.refIdx[0]) == pColPic)
        {
          index = 1;
        }
        else if (slice.isInterB() && (miNeighbor.interDir & (1 << REF_PIC_LIST_1)) && slice.getRefPic(REF_PIC_LIST_1, miNeighbor.refIdx[1]) == pColPic)
        {
          index = 2;
        }
        if (index > 0)
        {
          // get Inter Dir
          mrgCtx.interDirNeighbours[cnt] = index;
          miNeighbor.mv[index - 1].roundToPrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
          if (index == 1)
          {
            mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[index - 1], miNeighbor.refIdx[index - 1]);
            mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(Mv(), NOT_VALID);
          }
          else
          {
            mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(Mv(), NOT_VALID);
            mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[index - 1], miNeighbor.refIdx[index - 1]);
          }

#if NON_ADJACENT_MRG_CAND || TM_MRG
          if (!mrgCtx.xCheckSimilarMotionSubTMVP(cnt
#if TM_MRG
            , mvdSimilarityThresh
#endif
          ))
#endif
          {
            cnt++;
          }
        }
      }

    }
  }

  const int numNACandidate2[7] = { 4, 4, 4, 4, 4, 4, 4 };
  const int idxMap2[7][5] = { { 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 } };

  for (int iDistanceIndex = 0; iDistanceIndex < 7 && cnt < maxNumMergeCand; iDistanceIndex++)
  {
    const int iNADistanceHor = pu.Y().width  * (iDistanceIndex + 1);
    const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);

    for (int iNASPIdx = 0; iNASPIdx < numNACandidate2[iDistanceIndex] && cnt < maxNumMergeCand; iNASPIdx++)
    {
      switch (idxMap2[iDistanceIndex][iNASPIdx])
      {
      case 0:offsetX = offsetX0 = -iNADistanceHor - 1;                                                         offsetY = offsetY2 + ((pu.Y().height + iNADistanceVer - 1 - offsetY2) >> 1); break;
      case 1:offsetX = offsetX2 + ((pu.Y().width + iNADistanceHor - 1 - offsetX2) >> 1);                       offsetY = offsetY0 = -iNADistanceVer - 1; break;
      case 2:offsetX = offsetX0;                                                                               offsetY = offsetY0 + ((offsetY2 - offsetY0) >> 1); break;
      case 3:offsetX = offsetX0 + ((offsetX2 - offsetX0) >> 1);                                                offsetY = offsetY0; break;
      default: printf("error!"); exit(0); break;
      }

      const PredictionUnit *puNonAdjacent = cs.getPURestricted(posLT.offset(offsetX, offsetY), pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER( pu.lumaPos(), posLT.offset( offsetX, offsetY ), plevel ) && CU::isInter( *puNonAdjacent->cu ) && puNonAdjacent->getMotionInfo( posLT.offset( offsetX, offsetY ) ).isInter;
#else
      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(pu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent->cu);
#endif

      if (isAvailableNonAdjacent)
      {
        miNeighbor = puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY));

        int index = 0;
        if ((miNeighbor.interDir & (1 << REF_PIC_LIST_0)) && slice.getRefPic(REF_PIC_LIST_0, miNeighbor.refIdx[0]) == pColPic)
        {
          index = 1;
        }
        else if (slice.isInterB() && (miNeighbor.interDir & (1 << REF_PIC_LIST_1)) && slice.getRefPic(REF_PIC_LIST_1, miNeighbor.refIdx[1]) == pColPic)
        {
          index = 2;
        }
        if (index > 0)
        {
          // get Inter Dir
          mrgCtx.interDirNeighbours[cnt] = index;
          miNeighbor.mv[index - 1].roundToPrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
          if (index == 1)
          {
            mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[index - 1], miNeighbor.refIdx[index - 1]);
            mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(Mv(), NOT_VALID);
          }
          else
          {
            mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(Mv(), NOT_VALID);
            mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[index - 1], miNeighbor.refIdx[index - 1]);
          }
        }
#if NON_ADJACENT_MRG_CAND || TM_MRG
        if (!mrgCtx.xCheckSimilarMotionSubTMVP(cnt
#if TM_MRG
          , mvdSimilarityThresh
#endif
        ))
#endif
        {
          cnt++;
        }
      }
    }
  }

  mrgCtx.numValidMergeCand = cnt;
}
#endif
void PU::getNonAdjacentMergeCand(const PredictionUnit &pu, MergeCtx& mrgCtx)
{
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  if (!pu.cs->sps->getUseTmvpNmvpReordering())
  {
    mrgCtx.numValidMergeCand = 0;
    return;
  }
#endif
  const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
  const CodingStructure &cs = *pu.cs;
  const Slice &slice = *pu.cs->slice;

#if TM_MRG
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS || JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
  bool useMvdThreshold = true;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
  useMvdThreshold &= pu.cs->sps->getUseTMMrgMode();
#endif
#if JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
  useMvdThreshold &= pu.tmMergeFlag;
#endif
#endif
  const uint32_t mvdSimilarityThresh =
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS || JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC
                                       !useMvdThreshold ? 1 :
#endif
                                       getBDMVRMvdThreshold(pu);
#endif
  const uint32_t maxNumMergeCand = NUM_NON_ADJ_CANDS;

  for (uint32_t ui = 0; ui < maxNumMergeCand; ++ui)
  {
    mrgCtx.bcwIdx[ui] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
    mrgCtx.altLMFlag[ui] = false;
    mrgCtx.altLMParaNeighbours[ui].resetAltLinearModel();
#endif
#if INTER_LIC
    mrgCtx.licFlags[ui] = false;
#endif
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mvFieldNeighbours[(ui << 1)].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
    mrgCtx.useAltHpelIf[ui] = false;
#if MULTI_HYP_PRED
    mrgCtx.addHypNeighbours[ui].clear();
#endif
    mrgCtx.candCost[ui] = MAX_UINT64;
  }

  int cnt = 0;

  const Position posLT = pu.Y().topLeft();

  MotionInfo miNeighbor;
  int offsetX = 0;
  int offsetY = 0;
  int offsetX0 = 0; int offsetX1 = 0; int offsetX2 = pu.Y().width >> 1;
  int offsetY0 = 0; int offsetY1 = 0; int offsetY2 = pu.Y().height >> 1;

  const int numNACandidate[7] = { 5, 9, 9, 9, 9, 9, 9 };
  const int idxMap[7][9] = { { 0, 1, 2, 3, 4 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 } };

  for (int iDistanceIndex = 0; iDistanceIndex < 7 && cnt < maxNumMergeCand; iDistanceIndex++)
  {
    const int iNADistanceHor = pu.Y().width  * (iDistanceIndex + 1);
    const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);

    for (int iNASPIdx = 0; iNASPIdx < numNACandidate[iDistanceIndex] && cnt < maxNumMergeCand; iNASPIdx++)
    {
      switch (idxMap[iDistanceIndex][iNASPIdx])
      {
      case 0:offsetX = offsetX0 = -iNADistanceHor - 1;               offsetY = offsetY0 = pu.Y().height + iNADistanceVer - 1; break;
      case 1:offsetX = offsetX1 = pu.Y().width + iNADistanceHor - 1; offsetY = offsetY1 = -iNADistanceVer - 1;                break;
      case 2:offsetX = offsetX2;                                     offsetY = offsetY1;                                      break;
      case 3:offsetX = offsetX0;                                     offsetY = offsetY2;                                      break;
      case 4:offsetX = offsetX0;                                     offsetY = offsetY1;                                      break;
      case 5:offsetX = -1;                                           offsetY = offsetY0;                                      break;
      case 6:offsetX = offsetX1;                                     offsetY = -1;                                            break;
      case 7:offsetX = offsetX0 >> 1;                                offsetY = offsetY0;                                      break;
      case 8:offsetX = offsetX1;                                     offsetY = offsetY1 >> 1;                                 break;
      default: printf("error!"); exit(0); break;
      }

      const PredictionUnit *puNonAdjacent = cs.getPURestricted(posLT.offset(offsetX, offsetY), pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(pu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent->cu) && puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY)).isInter;
#else
      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(pu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent->cu);
#endif

      if (isAvailableNonAdjacent)
      {
        miNeighbor = puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY));

        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
        // get Mv from Above-Left
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
        mrgCtx.useAltHpelIf[cnt] = miNeighbor.useAltHpelIf;
        // get Mv from Above-right
        mrgCtx.bcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puNonAdjacent->cu->bcwIdx : BCW_DEFAULT;
#if INTER_LIC
#if JVET_AG0276_NLIC
        mrgCtx.licFlags[cnt] = miNeighbor.usesLIC || (puNonAdjacent->cu->altLMFlag && slice.getUseLIC());
#else
        mrgCtx.licFlags[cnt] = miNeighbor.usesLIC;
#endif
#endif
        if (slice.isInterB())
        {
          mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);
#if MULTI_HYP_PRED
          mrgCtx.addHypNeighbours[cnt] = puNonAdjacent->addHypData;
#endif
        }

#if NON_ADJACENT_MRG_CAND || TM_MRG
        if (!mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG
          , mvdSimilarityThresh
#endif
        ))
#endif
        {
          cnt++;
        }
#if JVET_AD0213_LIC_IMP
        else
        {
          mrgCtx.initMrgCand(cnt);
        }
#endif
      }

    }
  }

  const int numNACandidate2[7] = { 4, 4, 4, 4, 4, 4, 4 };
  const int idxMap2[7][5] = { { 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 } };

  for (int iDistanceIndex = 0; iDistanceIndex < 7 && cnt < maxNumMergeCand; iDistanceIndex++)
  {
    const int iNADistanceHor = pu.Y().width  * (iDistanceIndex + 1);
    const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);

    for (int iNASPIdx = 0; iNASPIdx < numNACandidate2[iDistanceIndex] && cnt < maxNumMergeCand; iNASPIdx++)
    {
      switch (idxMap2[iDistanceIndex][iNASPIdx])
      {
      case 0:offsetX = offsetX0 = -iNADistanceHor - 1;                                                         offsetY = offsetY2 + ((pu.Y().height + iNADistanceVer - 1 - offsetY2) >> 1); break;
      case 1:offsetX = offsetX2 + ((pu.Y().width + iNADistanceHor - 1 - offsetX2) >> 1);                       offsetY = offsetY0 = -iNADistanceVer - 1; break;
      case 2:offsetX = offsetX0;                                                                               offsetY = offsetY0 + ((offsetY2 - offsetY0) >> 1); break;
      case 3:offsetX = offsetX0 + ((offsetX2 - offsetX0) >> 1);                                                offsetY = offsetY0; break;
      default: printf("error!"); exit(0); break;
      }

      const PredictionUnit *puNonAdjacent = cs.getPURestricted(posLT.offset(offsetX, offsetY), pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(pu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent->cu) && puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY)).isInter;
#else
      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(pu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent->cu);
#endif

      if (isAvailableNonAdjacent)
      {
        miNeighbor = puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY));

        // get Inter Dir
        mrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
        // get Mv from Above-Left
        mrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
        mrgCtx.useAltHpelIf[cnt] = miNeighbor.useAltHpelIf;
        // get Mv from Above-right
        mrgCtx.bcwIdx[cnt] = (mrgCtx.interDirNeighbours[cnt] == 3) ? puNonAdjacent->cu->bcwIdx : BCW_DEFAULT;
#if INTER_LIC
#if JVET_AG0276_NLIC
        mrgCtx.licFlags[cnt] = miNeighbor.usesLIC || (puNonAdjacent->cu->altLMFlag && slice.getUseLIC());
#else
        mrgCtx.licFlags[cnt] = miNeighbor.usesLIC;
#endif
#endif
        if (slice.isInterB())
        {
          mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);
#if MULTI_HYP_PRED
          mrgCtx.addHypNeighbours[cnt] = puNonAdjacent->addHypData;
#endif
        }

#if NON_ADJACENT_MRG_CAND || TM_MRG
        if (!mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG
          , mvdSimilarityThresh
#endif
        ))
#endif
        {
          cnt++;
        }
#if JVET_AD0213_LIC_IMP
        else
        {
          mrgCtx.initMrgCand(cnt);
        }
#endif
      }
    }
  }

  mrgCtx.numValidMergeCand = cnt;
}
#endif

#if JVET_AG0276_NLIC
uint32_t PU::getAltMergeMvdThreshold(const PredictionUnit &pu)
{
  uint32_t numPixels = pu.lwidth() * pu.lheight();
  if (numPixels < 64)
  {
    return 1 << MV_FRACTIONAL_BITS_INTERNAL;
  }
  else if (numPixels < 256)
  {
    return 2 << MV_FRACTIONAL_BITS_INTERNAL;
  }
  else
  {
    return 4 << MV_FRACTIONAL_BITS_INTERNAL;
  }
}

#if JVET_AG0276_LIC_FLAG_SIGNALING
bool PU::isValidAltMergeCandidate(const PredictionUnit &pu, bool isBRCand)
#else
bool PU::isValidAltMergeCandidate(const PredictionUnit &pu)
#endif
{
  bool valid = true;
  if (!CU::isInter(*pu.cu))
  {
    valid = false;
  }
  if (pu.cu->geoFlag)
  {
    valid = false;
  }
  if (pu.mergeFlag && pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
  {
    valid = false;
  }

  if (!valid)
  {
    return false;
  }

#if JVET_AG0276_LIC_FLAG_SIGNALING
  bool trivial = false;
  if (!isBRCand)
  {
    trivial = (pu.cu->altLMParaUnit.scale[0] == 32 && pu.cu->altLMParaUnit.offset[0] == 0);
  }
  else
  {
    trivial = (pu.cu->altLMBRParaUnit.scale[0] == 32 && pu.cu->altLMBRParaUnit.offset[0] == 0);
  }
#else
  bool trivial = (pu.cu->altLMParaUnit.scale[0] == 32 && pu.cu->altLMParaUnit.offset[0] == 0);
#endif

  if (trivial)
  {
    return false;
  }
  return true;
}

void PU::getAltMergeCandidates(const PredictionUnit &pu, AltLMMergeCtx& cMrgCtx)
{
  const uint32_t         maxNumMergeCand = ALT_MRG_MAX_NUM_CANDS;
  const unsigned         plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
  const CodingStructure& cs = *pu.cs;
  const Slice&           slice = *pu.cs->slice;
  const uint32_t         mvdSimilarityThresh = getAltMergeMvdThreshold(pu);

  for (int i = 0; i < maxNumMergeCand; i++)
  {
    cMrgCtx.initAltLMMergeCtx(i);
  }

  int cnt = 0;

  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  const Position posLB = pu.Y().bottomLeft();

  // above
  const PredictionUnit *puAbove = cs.getPURestricted(posRT.offset(0, -1), pu, pu.chType);
  bool                 isAvailableB1 = puAbove && isDiffMER(pu.lumaPos(), posRT.offset(0, -1), plevel) && CU::isInter(*puAbove->cu);
  if (isAvailableB1 && PU::isValidAltMergeCandidate(*puAbove))
  {
    MotionInfo miAbove = puAbove->getMotionInfo(posRT.offset(0, -1));
    cMrgCtx.interDirNeighbours[cnt] = miAbove.interDir;
    cMrgCtx.useAltHpelIf[cnt] = miAbove.useAltHpelIf;
    cMrgCtx.bcwIdx[cnt] = (cMrgCtx.interDirNeighbours[cnt] == 3) ? puAbove->cu->bcwIdx : BCW_DEFAULT;
    cMrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAbove.mv[0], miAbove.refIdx[0]);
    if (slice.isInterB())
    {
      cMrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miAbove.mv[1], miAbove.refIdx[1]);
    }
    cMrgCtx.altLMParaNeighbours[cnt] = puAbove->cu->altLMParaUnit;
    if (!cMrgCtx.xCheckSameMotion(cnt, mvdSimilarityThresh))
    {
      cnt++;
    }
    else
    {
      cMrgCtx.initAltLMMergeCtx(cnt);
    }
  }

  if (cnt == maxNumMergeCand)
  {
    return;
  }

  //left
  const PredictionUnit* puLeft = cs.getPURestricted(posLB.offset(-1, 0), pu, pu.chType);
  const bool            isAvailableA1 = puLeft && isDiffMER(pu.lumaPos(), posLB.offset(-1, 0), plevel) && CU::isInter(*puLeft->cu);
  if (isAvailableA1 && PU::isValidAltMergeCandidate(*puLeft))
  {
    MotionInfo miLeft = puLeft->getMotionInfo(posLB.offset(-1, 0));
    cMrgCtx.interDirNeighbours[cnt] = miLeft.interDir;
    cMrgCtx.useAltHpelIf[cnt] = miLeft.useAltHpelIf;
    cMrgCtx.bcwIdx[cnt] = (cMrgCtx.interDirNeighbours[cnt] == 3) ? puLeft->cu->bcwIdx : BCW_DEFAULT;
    cMrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miLeft.mv[0], miLeft.refIdx[0]);
    if (slice.isInterB())
    {
      cMrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miLeft.mv[1], miLeft.refIdx[1]);
    }
    cMrgCtx.altLMParaNeighbours[cnt] = puLeft->cu->altLMParaUnit;
    if (!cMrgCtx.xCheckSameMotion(cnt, mvdSimilarityThresh))
    {
      cnt++;
    }
    else
    {
      cMrgCtx.initAltLMMergeCtx(cnt);
    }
  }

  if (cnt == maxNumMergeCand)
  {
    return;
  }

  // above right
  const PredictionUnit *puAboveRight = cs.getPURestricted(posRT.offset(1, -1), pu, pu.chType);
  const bool            isAvailableB0 = puAboveRight && isDiffMER(pu.lumaPos(), posRT.offset(1, -1), plevel) && CU::isInter(*puAboveRight->cu);
  if (isAvailableB0 && PU::isValidAltMergeCandidate(*puAboveRight))
  {
    MotionInfo miAboveRight = puAboveRight->getMotionInfo(posRT.offset(1, -1));
    cMrgCtx.interDirNeighbours[cnt] = miAboveRight.interDir;
    cMrgCtx.useAltHpelIf[cnt] = miAboveRight.useAltHpelIf;
    cMrgCtx.bcwIdx[cnt] = (cMrgCtx.interDirNeighbours[cnt] == 3) ? puAboveRight->cu->bcwIdx : BCW_DEFAULT;
    cMrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAboveRight.mv[0], miAboveRight.refIdx[0]);
    if (slice.isInterB())
    {
      cMrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miAboveRight.mv[1], miAboveRight.refIdx[1]);
    }
    cMrgCtx.altLMParaNeighbours[cnt] = puAboveRight->cu->altLMParaUnit;
    if (!cMrgCtx.xCheckSameMotion(cnt, mvdSimilarityThresh))
    {
      cnt++;
    }
    else
    {
      cMrgCtx.initAltLMMergeCtx(cnt);
    }
  }

  if (cnt == maxNumMergeCand)
  {
    return;
  }

  //left bottom
  const PredictionUnit *puLeftBottom = cs.getPURestricted(posLB.offset(-1, 1), pu, pu.chType);
  bool                 isAvailableA0 = puLeftBottom && isDiffMER(pu.lumaPos(), posLB.offset(-1, 1), plevel) && CU::isInter(*puLeftBottom->cu);
  if (isAvailableA0 && PU::isValidAltMergeCandidate(*puLeftBottom))
  {
    MotionInfo miBelowLeft = puLeftBottom->getMotionInfo(posLB.offset(-1, 1));
    cMrgCtx.interDirNeighbours[cnt] = miBelowLeft.interDir;
    cMrgCtx.useAltHpelIf[cnt] = miBelowLeft.useAltHpelIf;
    cMrgCtx.bcwIdx[cnt] = (cMrgCtx.interDirNeighbours[cnt] == 3) ? puLeftBottom->cu->bcwIdx : BCW_DEFAULT;
    cMrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miBelowLeft.mv[0], miBelowLeft.refIdx[0]);
    if (slice.isInterB())
    {
      cMrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miBelowLeft.mv[1], miBelowLeft.refIdx[1]);
    }
    cMrgCtx.altLMParaNeighbours[cnt] = puLeftBottom->cu->altLMParaUnit;
    if (!cMrgCtx.xCheckSameMotion(cnt, mvdSimilarityThresh))
    {
      cnt++;
    }
    else
    {
      cMrgCtx.initAltLMMergeCtx(cnt);
    }
  }

  if (cnt == maxNumMergeCand)
  {
    return;
  }

  // above left
  const PredictionUnit *puAboveLeft = cs.getPURestricted(posLT.offset(-1, -1), pu, pu.chType);
  bool                 isAvailableB2 = puAboveLeft && isDiffMER(pu.lumaPos(), posLT.offset(-1, -1), plevel) && CU::isInter(*puAboveLeft->cu);
  if (isAvailableB2 && PU::isValidAltMergeCandidate(*puAboveLeft))
  {
    MotionInfo miAboveLeft = puAboveLeft->getMotionInfo(posLT.offset(-1, -1));
    cMrgCtx.interDirNeighbours[cnt] = miAboveLeft.interDir;
    cMrgCtx.useAltHpelIf[cnt] = miAboveLeft.useAltHpelIf;
    cMrgCtx.bcwIdx[cnt] = (cMrgCtx.interDirNeighbours[cnt] == 3) ? puAboveLeft->cu->bcwIdx : BCW_DEFAULT;
    cMrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAboveLeft.mv[0], miAboveLeft.refIdx[0]);
    if (slice.isInterB())
    {
      cMrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miAboveLeft.mv[1], miAboveLeft.refIdx[1]);
    }
    cMrgCtx.altLMParaNeighbours[cnt] = puAboveLeft->cu->altLMParaUnit;
    if (!cMrgCtx.xCheckSameMotion(cnt, mvdSimilarityThresh))
    {
      cnt++;
    }
    else
    {
      cMrgCtx.initAltLMMergeCtx(cnt);
    }
  }

  if (cnt == maxNumMergeCand)
  {
    return;
  }

  int offsetX = 0;
  int offsetY = 0;
  int offsetX0 = 0; int offsetX1 = 0; int offsetX2 = pu.Y().width >> 1;
  int offsetY0 = 0; int offsetY1 = 0; int offsetY2 = pu.Y().height >> 1;

  const int numNACandidate[7] = { 5, 9, 9, 9, 9, 9, 9 };
  const int idxMap[7][9] = { { 0, 1, 2, 3, 4 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 } };

  for (int iDistanceIndex = 0; iDistanceIndex < 7 && cnt < maxNumMergeCand; iDistanceIndex++)
  {
    const int iNADistanceHor = pu.Y().width  * (iDistanceIndex + 1);
    const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);

    for (int iNASPIdx = 0; iNASPIdx < numNACandidate[iDistanceIndex] && cnt < maxNumMergeCand; iNASPIdx++)
    {
      switch (idxMap[iDistanceIndex][iNASPIdx])
      {
      case 0:offsetX = offsetX0 = -iNADistanceHor - 1;               offsetY = offsetY0 = pu.Y().height + iNADistanceVer - 1; break;
      case 1:offsetX = offsetX1 = pu.Y().width + iNADistanceHor - 1; offsetY = offsetY1 = -iNADistanceVer - 1;                break;
      case 2:offsetX = offsetX2;                                     offsetY = offsetY1;                                      break;
      case 3:offsetX = offsetX0;                                     offsetY = offsetY2;                                      break;
      case 4:offsetX = offsetX0;                                     offsetY = offsetY1;                                      break;
      case 5:offsetX = -1;                                           offsetY = offsetY0;                                      break;
      case 6:offsetX = offsetX1;                                     offsetY = -1;                                            break;
      case 7:offsetX = offsetX0 >> 1;                                offsetY = offsetY0;                                      break;
      case 8:offsetX = offsetX1;                                     offsetY = offsetY1 >> 1;                                 break;
      default: printf("error!"); exit(0); break;
      }

      const PredictionUnit *puNonAdjacent = cs.getPURestricted(posLT.offset(offsetX, offsetY), pu, pu.chType);

      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(pu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent->cu);
      if (isAvailableNonAdjacent && PU::isValidAltMergeCandidate(*puNonAdjacent))
      {
        MotionInfo miNeighbor = puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY));
        cMrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
        cMrgCtx.useAltHpelIf[cnt] = miNeighbor.useAltHpelIf;
        cMrgCtx.bcwIdx[cnt] = (cMrgCtx.interDirNeighbours[cnt] == 3) ? puNonAdjacent->cu->bcwIdx : BCW_DEFAULT;
        cMrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
        if (slice.isInterB())
        {
          cMrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);
        }
        cMrgCtx.altLMParaNeighbours[cnt] = puNonAdjacent->cu->altLMParaUnit;
        if (!cMrgCtx.xCheckSameMotion(cnt, mvdSimilarityThresh))
        {
          cnt++;
        }
        else
        {
          cMrgCtx.initAltLMMergeCtx(cnt);
        }
      }
    }
  }

  const int numNACandidate2[7] = { 4, 4, 4, 4, 4, 4, 4 };
  const int idxMap2[7][5] = { { 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 } };

  for (int iDistanceIndex = 0; iDistanceIndex < 7 && cnt < maxNumMergeCand; iDistanceIndex++)
  {
    const int iNADistanceHor = pu.Y().width  * (iDistanceIndex + 1);
    const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);

    for (int iNASPIdx = 0; iNASPIdx < numNACandidate2[iDistanceIndex] && cnt < maxNumMergeCand; iNASPIdx++)
    {
      switch (idxMap2[iDistanceIndex][iNASPIdx])
      {
      case 0:offsetX = offsetX0 = -iNADistanceHor - 1;                                                         offsetY = offsetY2 + ((pu.Y().height + iNADistanceVer - 1 - offsetY2) >> 1); break;
      case 1:offsetX = offsetX2 + ((pu.Y().width + iNADistanceHor - 1 - offsetX2) >> 1);                       offsetY = offsetY0 = -iNADistanceVer - 1; break;
      case 2:offsetX = offsetX0;                                                                               offsetY = offsetY0 + ((offsetY2 - offsetY0) >> 1); break;
      case 3:offsetX = offsetX0 + ((offsetX2 - offsetX0) >> 1);                                                offsetY = offsetY0; break;
      default: printf("error!"); exit(0); break;
      }

      const PredictionUnit *puNonAdjacent = cs.getPURestricted(posLT.offset(offsetX, offsetY), pu, pu.chType);

      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(pu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent->cu);
      if (isAvailableNonAdjacent && PU::isValidAltMergeCandidate(*puNonAdjacent))
      {
        MotionInfo miNeighbor = puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY));
        cMrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
        cMrgCtx.useAltHpelIf[cnt] = miNeighbor.useAltHpelIf;
        cMrgCtx.bcwIdx[cnt] = (cMrgCtx.interDirNeighbours[cnt] == 3) ? puNonAdjacent->cu->bcwIdx : BCW_DEFAULT;
        cMrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
        if (slice.isInterB())
        {
          cMrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);
        }
        cMrgCtx.altLMParaNeighbours[cnt] = puNonAdjacent->cu->altLMParaUnit;
        if (!cMrgCtx.xCheckSameMotion(cnt, mvdSimilarityThresh))
        {
          cnt++;
        }
        else
        {
          cMrgCtx.initAltLMMergeCtx(cnt);
        }
      }
    }
  }

  cMrgCtx.numValidMergeCand = cnt;
  return;
}
#if JVET_AG0276_LIC_FLAG_SIGNALING
void PU::getAltBRMergeCandidates(const PredictionUnit &pu, AltLMMergeCtx& cMrgCtx)
{
  const uint32_t         maxNumMergeCand = ALT_MRG_MAX_NUM_CANDS;
  const unsigned         plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
  const CodingStructure& cs = *pu.cs;
  const Slice&           slice = *pu.cs->slice;
  const uint32_t         mvdSimilarityThresh = getAltMergeMvdThreshold(pu);

  for (int i = 0; i < maxNumMergeCand; i++)
  {
    cMrgCtx.initAltLMMergeCtx(i);
  }

  int cnt = 0;

  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  const Position posLB = pu.Y().bottomLeft();

  const PredictionUnit *puAbove = cs.getPURestricted(posRT.offset(0, -1), pu, pu.chType);
  bool                 isAvailableB1 = puAbove && isDiffMER(pu.lumaPos(), posRT.offset(0, -1), plevel) && CU::isInter(*puAbove->cu);
  if (isAvailableB1 && PU::isValidAltMergeCandidate(*puAbove, true))
  {
    MotionInfo miAbove = puAbove->getMotionInfo(posRT.offset(0, -1));
    cMrgCtx.interDirNeighbours[cnt] = miAbove.interDir;
    cMrgCtx.useAltHpelIf[cnt] = miAbove.useAltHpelIf;
    cMrgCtx.bcwIdx[cnt] = (cMrgCtx.interDirNeighbours[cnt] == 3) ? puAbove->cu->bcwIdx : BCW_DEFAULT;
    cMrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAbove.mv[0], miAbove.refIdx[0]);
    if (slice.isInterB())
    {
      cMrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miAbove.mv[1], miAbove.refIdx[1]);
    }
    cMrgCtx.altLMParaNeighbours[cnt] = puAbove->cu->altLMBRParaUnit;
    if (!cMrgCtx.xCheckSameMotion(cnt, mvdSimilarityThresh))
    {
      cnt++;
    }
    else
    {
      cMrgCtx.initAltLMMergeCtx(cnt);
    }
  }

  if (cnt == maxNumMergeCand)
  {
    return;
  }

  const PredictionUnit* puLeft = cs.getPURestricted(posLB.offset(-1, 0), pu, pu.chType);
  const bool            isAvailableA1 = puLeft && isDiffMER(pu.lumaPos(), posLB.offset(-1, 0), plevel) && CU::isInter(*puLeft->cu);
  if (isAvailableA1 && PU::isValidAltMergeCandidate(*puLeft, true))
  {
    MotionInfo miLeft = puLeft->getMotionInfo(posLB.offset(-1, 0));
    cMrgCtx.interDirNeighbours[cnt] = miLeft.interDir;
    cMrgCtx.useAltHpelIf[cnt] = miLeft.useAltHpelIf;
    cMrgCtx.bcwIdx[cnt] = (cMrgCtx.interDirNeighbours[cnt] == 3) ? puLeft->cu->bcwIdx : BCW_DEFAULT;
    cMrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miLeft.mv[0], miLeft.refIdx[0]);
    if (slice.isInterB())
    {
      cMrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miLeft.mv[1], miLeft.refIdx[1]);
    }
    cMrgCtx.altLMParaNeighbours[cnt] = puLeft->cu->altLMBRParaUnit;
    if (!cMrgCtx.xCheckSameMotion(cnt, mvdSimilarityThresh))
    {
      cnt++;
    }
    else
    {
      cMrgCtx.initAltLMMergeCtx(cnt);
    }
  }

  if (cnt == maxNumMergeCand)
  {
    return;
  }

  const PredictionUnit *puAboveRight = cs.getPURestricted(posRT.offset(1, -1), pu, pu.chType);
  const bool            isAvailableB0 = puAboveRight && isDiffMER(pu.lumaPos(), posRT.offset(1, -1), plevel) && CU::isInter(*puAboveRight->cu);
  if (isAvailableB0 && PU::isValidAltMergeCandidate(*puAboveRight, true))
  {
    MotionInfo miAboveRight = puAboveRight->getMotionInfo(posRT.offset(1, -1));
    cMrgCtx.interDirNeighbours[cnt] = miAboveRight.interDir;
    cMrgCtx.useAltHpelIf[cnt] = miAboveRight.useAltHpelIf;
    cMrgCtx.bcwIdx[cnt] = (cMrgCtx.interDirNeighbours[cnt] == 3) ? puAboveRight->cu->bcwIdx : BCW_DEFAULT;
    cMrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAboveRight.mv[0], miAboveRight.refIdx[0]);
    if (slice.isInterB())
    {
      cMrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miAboveRight.mv[1], miAboveRight.refIdx[1]);
    }
    cMrgCtx.altLMParaNeighbours[cnt] = puAboveRight->cu->altLMBRParaUnit;
    if (!cMrgCtx.xCheckSameMotion(cnt, mvdSimilarityThresh))
    {
      cnt++;
    }
    else
    {
      cMrgCtx.initAltLMMergeCtx(cnt);
    }
  }

  if (cnt == maxNumMergeCand)
  {
    return;
  }

  const PredictionUnit *puLeftBottom = cs.getPURestricted(posLB.offset(-1, 1), pu, pu.chType);
  bool                 isAvailableA0 = puLeftBottom && isDiffMER(pu.lumaPos(), posLB.offset(-1, 1), plevel) && CU::isInter(*puLeftBottom->cu);
  if (isAvailableA0 && PU::isValidAltMergeCandidate(*puLeftBottom, true))
  {
    MotionInfo miBelowLeft = puLeftBottom->getMotionInfo(posLB.offset(-1, 1));
    cMrgCtx.interDirNeighbours[cnt] = miBelowLeft.interDir;
    cMrgCtx.useAltHpelIf[cnt] = miBelowLeft.useAltHpelIf;
    cMrgCtx.bcwIdx[cnt] = (cMrgCtx.interDirNeighbours[cnt] == 3) ? puLeftBottom->cu->bcwIdx : BCW_DEFAULT;
    cMrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miBelowLeft.mv[0], miBelowLeft.refIdx[0]);
    if (slice.isInterB())
    {
      cMrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miBelowLeft.mv[1], miBelowLeft.refIdx[1]);
    }
    cMrgCtx.altLMParaNeighbours[cnt] = puLeftBottom->cu->altLMBRParaUnit;
    if (!cMrgCtx.xCheckSameMotion(cnt, mvdSimilarityThresh))
    {
      cnt++;
    }
    else
    {
      cMrgCtx.initAltLMMergeCtx(cnt);
    }
  }

  if (cnt == maxNumMergeCand)
  {
    return;
  }

  const PredictionUnit *puAboveLeft = cs.getPURestricted(posLT.offset(-1, -1), pu, pu.chType);
  bool                 isAvailableB2 = puAboveLeft && isDiffMER(pu.lumaPos(), posLT.offset(-1, -1), plevel) && CU::isInter(*puAboveLeft->cu);
  if (isAvailableB2 && PU::isValidAltMergeCandidate(*puAboveLeft, true))
  {
    MotionInfo miAboveLeft = puAboveLeft->getMotionInfo(posLT.offset(-1, -1));
    cMrgCtx.interDirNeighbours[cnt] = miAboveLeft.interDir;
    cMrgCtx.useAltHpelIf[cnt] = miAboveLeft.useAltHpelIf;
    cMrgCtx.bcwIdx[cnt] = (cMrgCtx.interDirNeighbours[cnt] == 3) ? puAboveLeft->cu->bcwIdx : BCW_DEFAULT;
    cMrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miAboveLeft.mv[0], miAboveLeft.refIdx[0]);
    if (slice.isInterB())
    {
      cMrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miAboveLeft.mv[1], miAboveLeft.refIdx[1]);
    }
    cMrgCtx.altLMParaNeighbours[cnt] = puAboveLeft->cu->altLMBRParaUnit;
    if (!cMrgCtx.xCheckSameMotion(cnt, mvdSimilarityThresh))
    {
      cnt++;
    }
    else
    {
      cMrgCtx.initAltLMMergeCtx(cnt);
    }
  }

  if (cnt == maxNumMergeCand)
  {
    return;
  }

  int offsetX = 0;
  int offsetY = 0;
  int offsetX0 = 0; int offsetX1 = 0; int offsetX2 = pu.Y().width >> 1;
  int offsetY0 = 0; int offsetY1 = 0; int offsetY2 = pu.Y().height >> 1;

  const int numNACandidate[7] = { 5, 9, 9, 9, 9, 9, 9 };
  const int idxMap[7][9] = { { 0, 1, 2, 3, 4 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 } };

  for (int iDistanceIndex = 0; iDistanceIndex < 7 && cnt < maxNumMergeCand; iDistanceIndex++)
  {
    const int iNADistanceHor = pu.Y().width  * (iDistanceIndex + 1);
    const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);

    for (int iNASPIdx = 0; iNASPIdx < numNACandidate[iDistanceIndex] && cnt < maxNumMergeCand; iNASPIdx++)
    {
      switch (idxMap[iDistanceIndex][iNASPIdx])
      {
        case 0:offsetX = offsetX0 = -iNADistanceHor - 1;               offsetY = offsetY0 = pu.Y().height + iNADistanceVer - 1; break;
        case 1:offsetX = offsetX1 = pu.Y().width + iNADistanceHor - 1; offsetY = offsetY1 = -iNADistanceVer - 1;                break;
        case 2:offsetX = offsetX2;                                     offsetY = offsetY1;                                      break;
        case 3:offsetX = offsetX0;                                     offsetY = offsetY2;                                      break;
        case 4:offsetX = offsetX0;                                     offsetY = offsetY1;                                      break;
        case 5:offsetX = -1;                                           offsetY = offsetY0;                                      break;
        case 6:offsetX = offsetX1;                                     offsetY = -1;                                            break;
        case 7:offsetX = offsetX0 >> 1;                                offsetY = offsetY0;                                      break;
        case 8:offsetX = offsetX1;                                     offsetY = offsetY1 >> 1;                                 break;
        default: printf("error!"); exit(0); break;
      }

      const PredictionUnit *puNonAdjacent = cs.getPURestricted(posLT.offset(offsetX, offsetY), pu, pu.chType);

      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(pu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent->cu);
      if (isAvailableNonAdjacent && PU::isValidAltMergeCandidate(*puNonAdjacent, true))
      {
        MotionInfo miNeighbor = puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY));
        cMrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
        cMrgCtx.useAltHpelIf[cnt] = miNeighbor.useAltHpelIf;
        cMrgCtx.bcwIdx[cnt] = (cMrgCtx.interDirNeighbours[cnt] == 3) ? puNonAdjacent->cu->bcwIdx : BCW_DEFAULT;
        cMrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
        if (slice.isInterB())
        {
          cMrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);
        }
        cMrgCtx.altLMParaNeighbours[cnt] = puNonAdjacent->cu->altLMBRParaUnit;
        if (!cMrgCtx.xCheckSameMotion(cnt, mvdSimilarityThresh))
        {
          cnt++;
        }
        else
        {
          cMrgCtx.initAltLMMergeCtx(cnt);
        }
      }
    }
  }

  const int numNACandidate2[7] = { 4, 4, 4, 4, 4, 4, 4 };
  const int idxMap2[7][5] = { { 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 } };

  for (int iDistanceIndex = 0; iDistanceIndex < 7 && cnt < maxNumMergeCand; iDistanceIndex++)
  {
    const int iNADistanceHor = pu.Y().width  * (iDistanceIndex + 1);
    const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);

    for (int iNASPIdx = 0; iNASPIdx < numNACandidate2[iDistanceIndex] && cnt < maxNumMergeCand; iNASPIdx++)
    {
      switch (idxMap2[iDistanceIndex][iNASPIdx])
      {
        case 0:offsetX = offsetX0 = -iNADistanceHor - 1;                                                         offsetY = offsetY2 + ((pu.Y().height + iNADistanceVer - 1 - offsetY2) >> 1); break;
        case 1:offsetX = offsetX2 + ((pu.Y().width + iNADistanceHor - 1 - offsetX2) >> 1);                       offsetY = offsetY0 = -iNADistanceVer - 1; break;
        case 2:offsetX = offsetX0;                                                                               offsetY = offsetY0 + ((offsetY2 - offsetY0) >> 1); break;
        case 3:offsetX = offsetX0 + ((offsetX2 - offsetX0) >> 1);                                                offsetY = offsetY0; break;
        default: printf("error!"); exit(0); break;
      }

      const PredictionUnit *puNonAdjacent = cs.getPURestricted(posLT.offset(offsetX, offsetY), pu, pu.chType);

      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(pu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent->cu);
      if (isAvailableNonAdjacent && PU::isValidAltMergeCandidate(*puNonAdjacent, true))
      {
        MotionInfo miNeighbor = puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY));
        cMrgCtx.interDirNeighbours[cnt] = miNeighbor.interDir;
        cMrgCtx.useAltHpelIf[cnt] = miNeighbor.useAltHpelIf;
        cMrgCtx.bcwIdx[cnt] = (cMrgCtx.interDirNeighbours[cnt] == 3) ? puNonAdjacent->cu->bcwIdx : BCW_DEFAULT;
        cMrgCtx.mvFieldNeighbours[cnt << 1].setMvField(miNeighbor.mv[0], miNeighbor.refIdx[0]);
        if (slice.isInterB())
        {
          cMrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(miNeighbor.mv[1], miNeighbor.refIdx[1]);
        }
        cMrgCtx.altLMParaNeighbours[cnt] = puNonAdjacent->cu->altLMBRParaUnit;
        if (!cMrgCtx.xCheckSameMotion(cnt, mvdSimilarityThresh))
        {
          cnt++;
        }
        else
        {
          cMrgCtx.initAltLMMergeCtx(cnt);
        }
      }
    }
  }

  cMrgCtx.numValidMergeCand = cnt;
  return;
}
#endif
#endif

#if JVET_AG0276_NLIC
void PU::getAltLMAffineMergeCand(const PredictionUnit &pu, AltLMAffineMergeCtx& affMrgCtx)
{
  const uint32_t         maxNumMergeCand = ALT_AFF_MRG_MAX_NUM_CANDS;
  const unsigned         plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
  const CodingStructure& cs = *pu.cs;
  const Slice&           slice = *pu.cs->slice;

  for (int i = 0; i < maxNumMergeCand; i++)
  {
    affMrgCtx.initAltLMAffMergeCtx(i);
  }

  int cnt = 0;

  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  const Position posLB = pu.Y().bottomLeft();

  // above
  const PredictionUnit *puAbove = cs.getPURestricted(posRT.offset(0, -1), pu, pu.chType);
  bool isAvailableB1 = puAbove && isDiffMER(pu.lumaPos(), posRT.offset(0, -1), plevel) && CU::isInter(*puAbove->cu);
  if (isAvailableB1 && puAbove->cu->affine && (puAbove->mergeType == MRG_TYPE_DEFAULT_N) && PU::isValidAltMergeCandidate(*puAbove))
  {
    Mv cMv[2][3];
    affMrgCtx.affineType[cnt] = (EAffineModel)puAbove->cu->affineType;
    pu.cu->affineType = puAbove->cu->affineType;
    if (puAbove->interDir != 2)
    {
      xInheritedAffineMv(pu, puAbove, REF_PIC_LIST_0, cMv[0]);
    }
    if (slice.isInterB())
    {
      if (puAbove->interDir != 1)
      {
        xInheritedAffineMv(pu, puAbove, REF_PIC_LIST_1, cMv[1]);
      }
    }
    for (int mvNum = 0; mvNum < 3; mvNum++)
    {
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField(cMv[0][mvNum], puAbove->refIdx[0]);
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField(cMv[1][mvNum], puAbove->refIdx[1]);
    }
    affMrgCtx.interDirNeighbours[cnt] = puAbove->interDir;
    affMrgCtx.bcwIdx[cnt] = (affMrgCtx.interDirNeighbours[cnt] == 3) ? puAbove->cu->bcwIdx : BCW_DEFAULT;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
    affMrgCtx.obmcFlags[cnt] = puAbove->cu->obmcFlag;
#endif
    affMrgCtx.altLMParaNeighbours[cnt] = puAbove->cu->altLMParaUnit;
    if (affMrgCtx.xCheckSameAffMotion(pu, cnt))
    {
      cnt++;
    }
  }

  if (cnt == maxNumMergeCand)
  {
    return;
  }

  //left
  const PredictionUnit* puLeft = cs.getPURestricted(posLB.offset(-1, 0), pu, pu.chType);
  const bool isAvailableA1 = puLeft && isDiffMER(pu.lumaPos(), posLB.offset(-1, 0), plevel) && CU::isInter(*puLeft->cu);
  if (isAvailableA1 && puLeft->cu->affine && (puLeft->mergeType == MRG_TYPE_DEFAULT_N) && PU::isValidAltMergeCandidate(*puLeft))
  {
    Mv cMv[2][3];
    affMrgCtx.affineType[cnt] = (EAffineModel)puLeft->cu->affineType;
    pu.cu->affineType = puLeft->cu->affineType;
    if (puLeft->interDir != 2)
    {
      xInheritedAffineMv(pu, puLeft, REF_PIC_LIST_0, cMv[0]);
    }
    if (slice.isInterB())
    {
      if (puLeft->interDir != 1)
      {
        xInheritedAffineMv(pu, puLeft, REF_PIC_LIST_1, cMv[1]);
      }
    }
    for (int mvNum = 0; mvNum < 3; mvNum++)
    {
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField(cMv[0][mvNum], puLeft->refIdx[0]);
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField(cMv[1][mvNum], puLeft->refIdx[1]);
    }
    affMrgCtx.interDirNeighbours[cnt] = puLeft->interDir;
    affMrgCtx.bcwIdx[cnt] = (affMrgCtx.interDirNeighbours[cnt] == 3) ? puLeft->cu->bcwIdx : BCW_DEFAULT;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
    affMrgCtx.obmcFlags[cnt] = puLeft->cu->obmcFlag;
#endif
    affMrgCtx.altLMParaNeighbours[cnt] = puLeft->cu->altLMParaUnit;
    if (affMrgCtx.xCheckSameAffMotion(pu, cnt))
    {
      cnt++;
    }
  }

  if (cnt == maxNumMergeCand)
  {
    return;
  }

  // above right
  const PredictionUnit *puAboveRight = cs.getPURestricted(posRT.offset(1, -1), pu, pu.chType);
  const bool isAvailableB0 = puAboveRight && isDiffMER(pu.lumaPos(), posRT.offset(1, -1), plevel) && CU::isInter(*puAboveRight->cu);
  if (isAvailableB0 && puAboveRight->cu->affine && (puAboveRight->mergeType == MRG_TYPE_DEFAULT_N) && PU::isValidAltMergeCandidate(*puAboveRight))
  {
    Mv cMv[2][3];
    affMrgCtx.affineType[cnt] = (EAffineModel)puAboveRight->cu->affineType;
    pu.cu->affineType = puAboveRight->cu->affineType;
    if (puAboveRight->interDir != 2)
    {
      xInheritedAffineMv(pu, puAboveRight, REF_PIC_LIST_0, cMv[0]);
    }
    if (slice.isInterB())
    {
      if (puAboveRight->interDir != 1)
      {
        xInheritedAffineMv(pu, puAboveRight, REF_PIC_LIST_1, cMv[1]);
      }
    }
    for (int mvNum = 0; mvNum < 3; mvNum++)
    {
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField(cMv[0][mvNum], puAboveRight->refIdx[0]);
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField(cMv[1][mvNum], puAboveRight->refIdx[1]);
    }
    affMrgCtx.interDirNeighbours[cnt] = puAboveRight->interDir;
    affMrgCtx.bcwIdx[cnt] = (affMrgCtx.interDirNeighbours[cnt] == 3) ? puAboveRight->cu->bcwIdx : BCW_DEFAULT;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
    affMrgCtx.obmcFlags[cnt] = puAboveRight->cu->obmcFlag;
#endif
    affMrgCtx.altLMParaNeighbours[cnt] = puAboveRight->cu->altLMParaUnit;
    if (affMrgCtx.xCheckSameAffMotion(pu, cnt))
    {
      cnt++;
    }
  }

  if (cnt == maxNumMergeCand)
  {
    return;
  }

  //left bottom
  const PredictionUnit *puLeftBottom = cs.getPURestricted(posLB.offset(-1, 1), pu, pu.chType);
  bool isAvailableA0 = puLeftBottom && isDiffMER(pu.lumaPos(), posLB.offset(-1, 1), plevel) && CU::isInter(*puLeftBottom->cu);
  if (isAvailableA0 && puLeftBottom->cu->affine && (puLeftBottom->mergeType == MRG_TYPE_DEFAULT_N) && PU::isValidAltMergeCandidate(*puLeftBottom))
  {
    Mv cMv[2][3];
    affMrgCtx.affineType[cnt] = (EAffineModel)puLeftBottom->cu->affineType;
    pu.cu->affineType = puLeftBottom->cu->affineType;
    if (puLeftBottom->interDir != 2)
    {
      xInheritedAffineMv(pu, puLeftBottom, REF_PIC_LIST_0, cMv[0]);
    }
    if (slice.isInterB())
    {
      if (puLeftBottom->interDir != 1)
      {
        xInheritedAffineMv(pu, puLeftBottom, REF_PIC_LIST_1, cMv[1]);
      }
    }
    for (int mvNum = 0; mvNum < 3; mvNum++)
    {
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField(cMv[0][mvNum], puLeftBottom->refIdx[0]);
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField(cMv[1][mvNum], puLeftBottom->refIdx[1]);
    }
    affMrgCtx.interDirNeighbours[cnt] = puLeftBottom->interDir;
    affMrgCtx.bcwIdx[cnt] = (affMrgCtx.interDirNeighbours[cnt] == 3) ? puLeftBottom->cu->bcwIdx : BCW_DEFAULT;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
    affMrgCtx.obmcFlags[cnt] = puLeftBottom->cu->obmcFlag;
#endif
    affMrgCtx.altLMParaNeighbours[cnt] = puLeftBottom->cu->altLMParaUnit;
    if (affMrgCtx.xCheckSameAffMotion(pu, cnt))
    {
      cnt++;
    }
  }

  if (cnt == maxNumMergeCand)
  {
    return;
  }

  // above left
  const PredictionUnit *puAboveLeft = cs.getPURestricted(posLT.offset(-1, -1), pu, pu.chType);
  bool  isAvailableB2 = puAboveLeft && isDiffMER(pu.lumaPos(), posLT.offset(-1, -1), plevel) && CU::isInter(*puAboveLeft->cu);
  if (isAvailableB2 && puAboveLeft->cu->affine && (puAboveLeft->mergeType == MRG_TYPE_DEFAULT_N) && PU::isValidAltMergeCandidate(*puAboveLeft))
  {
    Mv cMv[2][3];
    affMrgCtx.affineType[cnt] = (EAffineModel)puAboveLeft->cu->affineType;
    pu.cu->affineType = puAboveLeft->cu->affineType;
    if (puAboveLeft->interDir != 2)
    {
      xInheritedAffineMv(pu, puAboveLeft, REF_PIC_LIST_0, cMv[0]);
    }
    if (slice.isInterB())
    {
      if (puAboveLeft->interDir != 1)
      {
        xInheritedAffineMv(pu, puAboveLeft, REF_PIC_LIST_1, cMv[1]);
      }
    }
    for (int mvNum = 0; mvNum < 3; mvNum++)
    {
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField(cMv[0][mvNum], puAboveLeft->refIdx[0]);
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField(cMv[1][mvNum], puAboveLeft->refIdx[1]);
    }
    affMrgCtx.interDirNeighbours[cnt] = puAboveLeft->interDir;
    affMrgCtx.bcwIdx[cnt] = (affMrgCtx.interDirNeighbours[cnt] == 3) ? puAboveLeft->cu->bcwIdx : BCW_DEFAULT;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
    affMrgCtx.obmcFlags[cnt] = puAboveLeft->cu->obmcFlag;
#endif
    affMrgCtx.altLMParaNeighbours[cnt] = puAboveLeft->cu->altLMParaUnit;
    if (affMrgCtx.xCheckSameAffMotion(pu, cnt))
    {
      cnt++;
    }
  }

  if (cnt == maxNumMergeCand)
  {
    return;
  }

  int offsetX = 0;
  int offsetY = 0;
  int offsetX0 = 0; int offsetX1 = 0; int offsetX2 = pu.Y().width >> 1;
  int offsetY0 = 0; int offsetY1 = 0; int offsetY2 = pu.Y().height >> 1;

  const int numNACandidate[7] = { 5, 9, 9, 9, 9, 9, 9 };
  const int idxMap[7][9] = { { 0, 1, 2, 3, 4 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 } };

  for (int iDistanceIndex = 0; iDistanceIndex < 7 && cnt < maxNumMergeCand; iDistanceIndex++)
  {
    const int iNADistanceHor = pu.Y().width  * (iDistanceIndex + 1);
    const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);

    for (int iNASPIdx = 0; iNASPIdx < numNACandidate[iDistanceIndex] && cnt < maxNumMergeCand; iNASPIdx++)
    {
      switch (idxMap[iDistanceIndex][iNASPIdx])
      {
      case 0:offsetX = offsetX0 = -iNADistanceHor - 1;               offsetY = offsetY0 = pu.Y().height + iNADistanceVer - 1; break;
      case 1:offsetX = offsetX1 = pu.Y().width + iNADistanceHor - 1; offsetY = offsetY1 = -iNADistanceVer - 1;                break;
      case 2:offsetX = offsetX2;                                     offsetY = offsetY1;                                      break;
      case 3:offsetX = offsetX0;                                     offsetY = offsetY2;                                      break;
      case 4:offsetX = offsetX0;                                     offsetY = offsetY1;                                      break;
      case 5:offsetX = -1;                                           offsetY = offsetY0;                                      break;
      case 6:offsetX = offsetX1;                                     offsetY = -1;                                            break;
      case 7:offsetX = offsetX0 >> 1;                                offsetY = offsetY0;                                      break;
      case 8:offsetX = offsetX1;                                     offsetY = offsetY1 >> 1;                                 break;
      default: printf("error!"); exit(0); break;
      }

      const PredictionUnit *puNonAdjacent = cs.getPURestricted(posLT.offset(offsetX, offsetY), pu, pu.chType);

      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(pu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent->cu);
      if (isAvailableNonAdjacent && puNonAdjacent->cu->affine && (puNonAdjacent->mergeType == MRG_TYPE_DEFAULT_N) && PU::isValidAltMergeCandidate(*puNonAdjacent))
      {
        Mv cMv[2][3];
        affMrgCtx.affineType[cnt] = (EAffineModel)puNonAdjacent->cu->affineType;
        pu.cu->affineType = puNonAdjacent->cu->affineType;
        if (puNonAdjacent->interDir != 2)
        {
          xInheritedAffineMv(pu, puNonAdjacent, REF_PIC_LIST_0, cMv[0]);
        }
        if (slice.isInterB())
        {
          if (puNonAdjacent->interDir != 1)
          {
            xInheritedAffineMv(pu, puNonAdjacent, REF_PIC_LIST_1, cMv[1]);
          }
        }
        for (int mvNum = 0; mvNum < 3; mvNum++)
        {
          affMrgCtx.mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField(cMv[0][mvNum], puNonAdjacent->refIdx[0]);
          affMrgCtx.mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField(cMv[1][mvNum], puNonAdjacent->refIdx[1]);
        }
        affMrgCtx.interDirNeighbours[cnt] = puNonAdjacent->interDir;
        affMrgCtx.bcwIdx[cnt] = (affMrgCtx.interDirNeighbours[cnt] == 3) ? puNonAdjacent->cu->bcwIdx : BCW_DEFAULT;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
        affMrgCtx.obmcFlags[cnt] = puNonAdjacent->cu->obmcFlag;
#endif
        affMrgCtx.altLMParaNeighbours[cnt] = puNonAdjacent->cu->altLMParaUnit;
        if (affMrgCtx.xCheckSameAffMotion(pu, cnt))
        {
          cnt++;
        }
      }
    }
  }

  const int numNACandidate2[7] = { 4, 4, 4, 4, 4, 4, 4 };
  const int idxMap2[7][5] = { { 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 } };

  for (int iDistanceIndex = 0; iDistanceIndex < 7 && cnt < maxNumMergeCand; iDistanceIndex++)
  {
    const int iNADistanceHor = pu.Y().width  * (iDistanceIndex + 1);
    const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);

    for (int iNASPIdx = 0; iNASPIdx < numNACandidate2[iDistanceIndex] && cnt < maxNumMergeCand; iNASPIdx++)
    {
      switch (idxMap2[iDistanceIndex][iNASPIdx])
      {
      case 0:offsetX = offsetX0 = -iNADistanceHor - 1;                                                         offsetY = offsetY2 + ((pu.Y().height + iNADistanceVer - 1 - offsetY2) >> 1); break;
      case 1:offsetX = offsetX2 + ((pu.Y().width + iNADistanceHor - 1 - offsetX2) >> 1);                       offsetY = offsetY0 = -iNADistanceVer - 1; break;
      case 2:offsetX = offsetX0;                                                                               offsetY = offsetY0 + ((offsetY2 - offsetY0) >> 1); break;
      case 3:offsetX = offsetX0 + ((offsetX2 - offsetX0) >> 1);                                                offsetY = offsetY0; break;
      default: printf("error!"); exit(0); break;
      }

      const PredictionUnit *puNonAdjacent = cs.getPURestricted(posLT.offset(offsetX, offsetY), pu, pu.chType);

      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(pu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent->cu);
      if (isAvailableNonAdjacent && puNonAdjacent->cu->affine && (puNonAdjacent->mergeType == MRG_TYPE_DEFAULT_N) && PU::isValidAltMergeCandidate(*puNonAdjacent))
      {
        Mv cMv[2][3];
        affMrgCtx.affineType[cnt] = (EAffineModel)puNonAdjacent->cu->affineType;
        pu.cu->affineType = puNonAdjacent->cu->affineType;
        if (puNonAdjacent->interDir != 2)
        {
          xInheritedAffineMv(pu, puNonAdjacent, REF_PIC_LIST_0, cMv[0]);
        }
        if (slice.isInterB())
        {
          if (puNonAdjacent->interDir != 1)
          {
            xInheritedAffineMv(pu, puNonAdjacent, REF_PIC_LIST_1, cMv[1]);
          }
        }
        for (int mvNum = 0; mvNum < 3; mvNum++)
        {
          affMrgCtx.mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField(cMv[0][mvNum], puNonAdjacent->refIdx[0]);
          affMrgCtx.mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField(cMv[1][mvNum], puNonAdjacent->refIdx[1]);
        }
        affMrgCtx.interDirNeighbours[cnt] = puNonAdjacent->interDir;
        affMrgCtx.bcwIdx[cnt] = (affMrgCtx.interDirNeighbours[cnt] == 3) ? puNonAdjacent->cu->bcwIdx : BCW_DEFAULT;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
        affMrgCtx.obmcFlags[cnt] = puNonAdjacent->cu->obmcFlag;
#endif
        affMrgCtx.altLMParaNeighbours[cnt] = puNonAdjacent->cu->altLMParaUnit;
        if (affMrgCtx.xCheckSameAffMotion(pu, cnt))
        {
          cnt++;
        }
      }
    }
  }

  affMrgCtx.numValidMergeCand = cnt;
  return;
}
#if JVET_AG0276_LIC_FLAG_SIGNALING
void PU::getAltLMBRAffineMergeCand(const PredictionUnit &pu, AltLMAffineMergeCtx& affMrgCtx)
{
  const uint32_t         maxNumMergeCand = ALT_AFF_MRG_MAX_NUM_CANDS;
  const unsigned         plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
  const CodingStructure& cs = *pu.cs;
  const Slice&           slice = *pu.cs->slice;

  for (int i = 0; i < maxNumMergeCand; i++)
  {
    affMrgCtx.initAltLMAffMergeCtx(i);
  }

  int cnt = 0;

  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  const Position posLB = pu.Y().bottomLeft();

  const PredictionUnit *puAbove = cs.getPURestricted(posRT.offset(0, -1), pu, pu.chType);
  bool isAvailableB1 = puAbove && isDiffMER(pu.lumaPos(), posRT.offset(0, -1), plevel) && CU::isInter(*puAbove->cu);
  if (isAvailableB1 && puAbove->cu->affine && (puAbove->mergeType == MRG_TYPE_DEFAULT_N) && PU::isValidAltMergeCandidate(*puAbove, true))
  {
    Mv cMv[2][3];
    affMrgCtx.affineType[cnt] = (EAffineModel)puAbove->cu->affineType;
    pu.cu->affineType = puAbove->cu->affineType;
    if (puAbove->interDir != 2)
    {
      xInheritedAffineMv(pu, puAbove, REF_PIC_LIST_0, cMv[0]);
    }
    if (slice.isInterB())
    {
      if (puAbove->interDir != 1)
      {
        xInheritedAffineMv(pu, puAbove, REF_PIC_LIST_1, cMv[1]);
      }
    }
    for (int mvNum = 0; mvNum < 3; mvNum++)
    {
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField(cMv[0][mvNum], puAbove->refIdx[0]);
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField(cMv[1][mvNum], puAbove->refIdx[1]);
    }
    affMrgCtx.interDirNeighbours[cnt] = puAbove->interDir;
    affMrgCtx.bcwIdx[cnt] = (affMrgCtx.interDirNeighbours[cnt] == 3) ? puAbove->cu->bcwIdx : BCW_DEFAULT;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
    affMrgCtx.obmcFlags[cnt] = puAbove->cu->obmcFlag;
#endif
    affMrgCtx.altLMParaNeighbours[cnt] = puAbove->cu->altLMBRParaUnit;
    if (affMrgCtx.xCheckSameAffMotion(pu, cnt))
    {
      cnt++;
    }
  }

  if (cnt == maxNumMergeCand)
  {
    return;
  }

  const PredictionUnit* puLeft = cs.getPURestricted(posLB.offset(-1, 0), pu, pu.chType);
  const bool isAvailableA1 = puLeft && isDiffMER(pu.lumaPos(), posLB.offset(-1, 0), plevel) && CU::isInter(*puLeft->cu);
  if (isAvailableA1 && puLeft->cu->affine && (puLeft->mergeType == MRG_TYPE_DEFAULT_N) && PU::isValidAltMergeCandidate(*puLeft, true))
  {
    Mv cMv[2][3];
    affMrgCtx.affineType[cnt] = (EAffineModel)puLeft->cu->affineType;
    pu.cu->affineType = puLeft->cu->affineType;
    if (puLeft->interDir != 2)
    {
      xInheritedAffineMv(pu, puLeft, REF_PIC_LIST_0, cMv[0]);
    }
    if (slice.isInterB())
    {
      if (puLeft->interDir != 1)
      {
        xInheritedAffineMv(pu, puLeft, REF_PIC_LIST_1, cMv[1]);
      }
    }
    for (int mvNum = 0; mvNum < 3; mvNum++)
    {
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField(cMv[0][mvNum], puLeft->refIdx[0]);
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField(cMv[1][mvNum], puLeft->refIdx[1]);
    }
    affMrgCtx.interDirNeighbours[cnt] = puLeft->interDir;
    affMrgCtx.bcwIdx[cnt] = (affMrgCtx.interDirNeighbours[cnt] == 3) ? puLeft->cu->bcwIdx : BCW_DEFAULT;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
    affMrgCtx.obmcFlags[cnt] = puLeft->cu->obmcFlag;
#endif
    affMrgCtx.altLMParaNeighbours[cnt] = puLeft->cu->altLMBRParaUnit;
    if (affMrgCtx.xCheckSameAffMotion(pu, cnt))
    {
      cnt++;
    }
  }

  if (cnt == maxNumMergeCand)
  {
    return;
  }

  const PredictionUnit *puAboveRight = cs.getPURestricted(posRT.offset(1, -1), pu, pu.chType);
  const bool isAvailableB0 = puAboveRight && isDiffMER(pu.lumaPos(), posRT.offset(1, -1), plevel) && CU::isInter(*puAboveRight->cu);
  if (isAvailableB0 && puAboveRight->cu->affine && (puAboveRight->mergeType == MRG_TYPE_DEFAULT_N) && PU::isValidAltMergeCandidate(*puAboveRight, true))
  {
    Mv cMv[2][3];
    affMrgCtx.affineType[cnt] = (EAffineModel)puAboveRight->cu->affineType;
    pu.cu->affineType = puAboveRight->cu->affineType;
    if (puAboveRight->interDir != 2)
    {
      xInheritedAffineMv(pu, puAboveRight, REF_PIC_LIST_0, cMv[0]);
    }
    if (slice.isInterB())
    {
      if (puAboveRight->interDir != 1)
      {
        xInheritedAffineMv(pu, puAboveRight, REF_PIC_LIST_1, cMv[1]);
      }
    }
    for (int mvNum = 0; mvNum < 3; mvNum++)
    {
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField(cMv[0][mvNum], puAboveRight->refIdx[0]);
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField(cMv[1][mvNum], puAboveRight->refIdx[1]);
    }
    affMrgCtx.interDirNeighbours[cnt] = puAboveRight->interDir;
    affMrgCtx.bcwIdx[cnt] = (affMrgCtx.interDirNeighbours[cnt] == 3) ? puAboveRight->cu->bcwIdx : BCW_DEFAULT;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
    affMrgCtx.obmcFlags[cnt] = puAboveRight->cu->obmcFlag;
#endif
    affMrgCtx.altLMParaNeighbours[cnt] = puAboveRight->cu->altLMBRParaUnit;
    if (affMrgCtx.xCheckSameAffMotion(pu, cnt))
    {
      cnt++;
    }
  }

  if (cnt == maxNumMergeCand)
  {
    return;
  }

  const PredictionUnit *puLeftBottom = cs.getPURestricted(posLB.offset(-1, 1), pu, pu.chType);
  bool isAvailableA0 = puLeftBottom && isDiffMER(pu.lumaPos(), posLB.offset(-1, 1), plevel) && CU::isInter(*puLeftBottom->cu);
  if (isAvailableA0 && puLeftBottom->cu->affine && (puLeftBottom->mergeType == MRG_TYPE_DEFAULT_N) && PU::isValidAltMergeCandidate(*puLeftBottom, true))
  {
    Mv cMv[2][3];
    affMrgCtx.affineType[cnt] = (EAffineModel)puLeftBottom->cu->affineType;
    pu.cu->affineType = puLeftBottom->cu->affineType;
    if (puLeftBottom->interDir != 2)
    {
      xInheritedAffineMv(pu, puLeftBottom, REF_PIC_LIST_0, cMv[0]);
    }
    if (slice.isInterB())
    {
      if (puLeftBottom->interDir != 1)
      {
        xInheritedAffineMv(pu, puLeftBottom, REF_PIC_LIST_1, cMv[1]);
      }
    }
    for (int mvNum = 0; mvNum < 3; mvNum++)
    {
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField(cMv[0][mvNum], puLeftBottom->refIdx[0]);
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField(cMv[1][mvNum], puLeftBottom->refIdx[1]);
    }
    affMrgCtx.interDirNeighbours[cnt] = puLeftBottom->interDir;
    affMrgCtx.bcwIdx[cnt] = (affMrgCtx.interDirNeighbours[cnt] == 3) ? puLeftBottom->cu->bcwIdx : BCW_DEFAULT;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
    affMrgCtx.obmcFlags[cnt] = puLeftBottom->cu->obmcFlag;
#endif
    affMrgCtx.altLMParaNeighbours[cnt] = puLeftBottom->cu->altLMBRParaUnit;
    if (affMrgCtx.xCheckSameAffMotion(pu, cnt))
    {
      cnt++;
    }
  }

  if (cnt == maxNumMergeCand)
  {
    return;
  }

  const PredictionUnit *puAboveLeft = cs.getPURestricted(posLT.offset(-1, -1), pu, pu.chType);
  bool  isAvailableB2 = puAboveLeft && isDiffMER(pu.lumaPos(), posLT.offset(-1, -1), plevel) && CU::isInter(*puAboveLeft->cu);
  if (isAvailableB2 && puAboveLeft->cu->affine && (puAboveLeft->mergeType == MRG_TYPE_DEFAULT_N) && PU::isValidAltMergeCandidate(*puAboveLeft, true))
  {
    Mv cMv[2][3];
    affMrgCtx.affineType[cnt] = (EAffineModel)puAboveLeft->cu->affineType;
    pu.cu->affineType = puAboveLeft->cu->affineType;
    if (puAboveLeft->interDir != 2)
    {
      xInheritedAffineMv(pu, puAboveLeft, REF_PIC_LIST_0, cMv[0]);
    }
    if (slice.isInterB())
    {
      if (puAboveLeft->interDir != 1)
      {
        xInheritedAffineMv(pu, puAboveLeft, REF_PIC_LIST_1, cMv[1]);
      }
    }
    for (int mvNum = 0; mvNum < 3; mvNum++)
    {
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField(cMv[0][mvNum], puAboveLeft->refIdx[0]);
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField(cMv[1][mvNum], puAboveLeft->refIdx[1]);
    }
    affMrgCtx.interDirNeighbours[cnt] = puAboveLeft->interDir;
    affMrgCtx.bcwIdx[cnt] = (affMrgCtx.interDirNeighbours[cnt] == 3) ? puAboveLeft->cu->bcwIdx : BCW_DEFAULT;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
    affMrgCtx.obmcFlags[cnt] = puAboveLeft->cu->obmcFlag;
#endif
    affMrgCtx.altLMParaNeighbours[cnt] = puAboveLeft->cu->altLMBRParaUnit;
    if (affMrgCtx.xCheckSameAffMotion(pu, cnt))
    {
      cnt++;
    }
  }

  if (cnt == maxNumMergeCand)
  {
    return;
  }

  int offsetX = 0;
  int offsetY = 0;
  int offsetX0 = 0; int offsetX1 = 0; int offsetX2 = pu.Y().width >> 1;
  int offsetY0 = 0; int offsetY1 = 0; int offsetY2 = pu.Y().height >> 1;

  const int numNACandidate[7] = { 5, 9, 9, 9, 9, 9, 9 };
  const int idxMap[7][9] = { { 0, 1, 2, 3, 4 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 },{ 0, 1, 2, 3, 4, 5, 6, 7, 8 } };

  for (int iDistanceIndex = 0; iDistanceIndex < 7 && cnt < maxNumMergeCand; iDistanceIndex++)
  {
    const int iNADistanceHor = pu.Y().width  * (iDistanceIndex + 1);
    const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);

    for (int iNASPIdx = 0; iNASPIdx < numNACandidate[iDistanceIndex] && cnt < maxNumMergeCand; iNASPIdx++)
    {
      switch (idxMap[iDistanceIndex][iNASPIdx])
      {
        case 0:offsetX = offsetX0 = -iNADistanceHor - 1;               offsetY = offsetY0 = pu.Y().height + iNADistanceVer - 1; break;
        case 1:offsetX = offsetX1 = pu.Y().width + iNADistanceHor - 1; offsetY = offsetY1 = -iNADistanceVer - 1;                break;
        case 2:offsetX = offsetX2;                                     offsetY = offsetY1;                                      break;
        case 3:offsetX = offsetX0;                                     offsetY = offsetY2;                                      break;
        case 4:offsetX = offsetX0;                                     offsetY = offsetY1;                                      break;
        case 5:offsetX = -1;                                           offsetY = offsetY0;                                      break;
        case 6:offsetX = offsetX1;                                     offsetY = -1;                                            break;
        case 7:offsetX = offsetX0 >> 1;                                offsetY = offsetY0;                                      break;
        case 8:offsetX = offsetX1;                                     offsetY = offsetY1 >> 1;                                 break;
        default: printf("error!"); exit(0); break;
      }

      const PredictionUnit *puNonAdjacent = cs.getPURestricted(posLT.offset(offsetX, offsetY), pu, pu.chType);

      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(pu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent->cu);
      if (isAvailableNonAdjacent && puNonAdjacent->cu->affine && (puNonAdjacent->mergeType == MRG_TYPE_DEFAULT_N) && PU::isValidAltMergeCandidate(*puNonAdjacent, true))
      {
        Mv cMv[2][3];
        affMrgCtx.affineType[cnt] = (EAffineModel)puNonAdjacent->cu->affineType;
        pu.cu->affineType = puNonAdjacent->cu->affineType;
        if (puNonAdjacent->interDir != 2)
        {
          xInheritedAffineMv(pu, puNonAdjacent, REF_PIC_LIST_0, cMv[0]);
        }
        if (slice.isInterB())
        {
          if (puNonAdjacent->interDir != 1)
          {
            xInheritedAffineMv(pu, puNonAdjacent, REF_PIC_LIST_1, cMv[1]);
          }
        }
        for (int mvNum = 0; mvNum < 3; mvNum++)
        {
          affMrgCtx.mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField(cMv[0][mvNum], puNonAdjacent->refIdx[0]);
          affMrgCtx.mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField(cMv[1][mvNum], puNonAdjacent->refIdx[1]);
        }
        affMrgCtx.interDirNeighbours[cnt] = puNonAdjacent->interDir;
        affMrgCtx.bcwIdx[cnt] = (affMrgCtx.interDirNeighbours[cnt] == 3) ? puNonAdjacent->cu->bcwIdx : BCW_DEFAULT;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
        affMrgCtx.obmcFlags[cnt] = puNonAdjacent->cu->obmcFlag;
#endif
        affMrgCtx.altLMParaNeighbours[cnt] = puNonAdjacent->cu->altLMBRParaUnit;
        if (affMrgCtx.xCheckSameAffMotion(pu, cnt))
        {
          cnt++;
        }
      }
    }
  }

  const int numNACandidate2[7] = { 4, 4, 4, 4, 4, 4, 4 };
  const int idxMap2[7][5] = { { 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 },{ 0, 1, 2, 3 } };

  for (int iDistanceIndex = 0; iDistanceIndex < 7 && cnt < maxNumMergeCand; iDistanceIndex++)
  {
    const int iNADistanceHor = pu.Y().width  * (iDistanceIndex + 1);
    const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);

    for (int iNASPIdx = 0; iNASPIdx < numNACandidate2[iDistanceIndex] && cnt < maxNumMergeCand; iNASPIdx++)
    {
      switch (idxMap2[iDistanceIndex][iNASPIdx])
      {
        case 0:offsetX = offsetX0 = -iNADistanceHor - 1;                                                         offsetY = offsetY2 + ((pu.Y().height + iNADistanceVer - 1 - offsetY2) >> 1); break;
        case 1:offsetX = offsetX2 + ((pu.Y().width + iNADistanceHor - 1 - offsetX2) >> 1);                       offsetY = offsetY0 = -iNADistanceVer - 1; break;
        case 2:offsetX = offsetX0;                                                                               offsetY = offsetY0 + ((offsetY2 - offsetY0) >> 1); break;
        case 3:offsetX = offsetX0 + ((offsetX2 - offsetX0) >> 1);                                                offsetY = offsetY0; break;
        default: printf("error!"); exit(0); break;
      }

      const PredictionUnit *puNonAdjacent = cs.getPURestricted(posLT.offset(offsetX, offsetY), pu, pu.chType);

      bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(pu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent->cu);
      if (isAvailableNonAdjacent && puNonAdjacent->cu->affine && (puNonAdjacent->mergeType == MRG_TYPE_DEFAULT_N) && PU::isValidAltMergeCandidate(*puNonAdjacent, true))
      {
        Mv cMv[2][3];
        affMrgCtx.affineType[cnt] = (EAffineModel)puNonAdjacent->cu->affineType;
        pu.cu->affineType = puNonAdjacent->cu->affineType;
        if (puNonAdjacent->interDir != 2)
        {
          xInheritedAffineMv(pu, puNonAdjacent, REF_PIC_LIST_0, cMv[0]);
        }
        if (slice.isInterB())
        {
          if (puNonAdjacent->interDir != 1)
          {
            xInheritedAffineMv(pu, puNonAdjacent, REF_PIC_LIST_1, cMv[1]);
          }
        }
        for (int mvNum = 0; mvNum < 3; mvNum++)
        {
          affMrgCtx.mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField(cMv[0][mvNum], puNonAdjacent->refIdx[0]);
          affMrgCtx.mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField(cMv[1][mvNum], puNonAdjacent->refIdx[1]);
        }
        affMrgCtx.interDirNeighbours[cnt] = puNonAdjacent->interDir;
        affMrgCtx.bcwIdx[cnt] = (affMrgCtx.interDirNeighbours[cnt] == 3) ? puNonAdjacent->cu->bcwIdx : BCW_DEFAULT;
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
        affMrgCtx.obmcFlags[cnt] = puNonAdjacent->cu->obmcFlag;
#endif
        affMrgCtx.altLMParaNeighbours[cnt] = puNonAdjacent->cu->altLMBRParaUnit;
        if (affMrgCtx.xCheckSameAffMotion(pu, cnt))
        {
          cnt++;
        }
      }
    }
  }

  affMrgCtx.numValidMergeCand = cnt;
  return;
}
#endif
#endif

bool PU::checkDMVRCondition(const PredictionUnit& pu)
{
#if !MULTI_PASS_DMVR
  if( pu.cs->sps->getUseDMVR() && !pu.cs->picHeader->getDisDmvrFlag() )
  {
    const int refIdx0 = pu.refIdx[REF_PIC_LIST_0];
    const int refIdx1 = pu.refIdx[REF_PIC_LIST_1];

    const WPScalingParam *wp0 = pu.cu->slice->getWpScaling( REF_PIC_LIST_0, refIdx0 );
    const WPScalingParam *wp1 = pu.cu->slice->getWpScaling( REF_PIC_LIST_1, refIdx1 );

    const bool isResamplingPossible = pu.cs->sps->getRprEnabledFlag();

    const bool ref0IsScaled = refIdx0 < 0 || refIdx0 >= MAX_NUM_REF
      ? false
      : isResamplingPossible && pu.cu->slice->getRefPic( REF_PIC_LIST_0, refIdx0 )->isRefScaled( pu.cs->pps );
    const bool ref1IsScaled = refIdx1 < 0 || refIdx1 >= MAX_NUM_REF
      ? false
      : isResamplingPossible && pu.cu->slice->getRefPic( REF_PIC_LIST_1, refIdx1 )->isRefScaled( pu.cs->pps );

    return pu.mergeFlag && pu.mergeType == MRG_TYPE_DEFAULT_N && !pu.ciipFlag && !pu.cu->affine && !pu.mmvdMergeFlag
      && !pu.cu->mmvdSkip && PU::isBiPredFromDifferentDirEqDistPoc( pu )
#if TM_MRG || (JVET_Z0084_IBC_TM && IBC_TM_MRG)
      && !pu.tmMergeFlag
#endif
      && ( pu.lheight() >= 8 ) && ( pu.lwidth() >= 8 )
      && ( ( pu.lheight() * pu.lwidth() ) >= 128 )
#if JVET_AD0213_LIC_IMP
      && !pu.cu->LICFlag
#endif
      && ( pu.cu->bcwIdx == BCW_DEFAULT )
      && !WPScalingParam::isWeighted( wp0 ) && !WPScalingParam::isWeighted( wp1 ) && !ref0IsScaled && !ref1IsScaled;
  }
  else
#endif
  {
    return false;
  }
}

#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
bool PU::checkBDMVR4Affine(const PredictionUnit& pu)
{
    // Different conditions for square and rectangular shapes
  if (pu.lwidth() == pu.lheight())
  {
    const int base32 = ((uint32_t)(pu.cs->picture->lwidth() * pu.cs->picture->lheight() * 0.00003546 + 55)) >> 5;
    const int maxSquareCuSizeABDMVR = std::max(32, std::min(base32 << 5, 256));

    if (pu.lwidth() > maxSquareCuSizeABDMVR)
    {
      return false;
    }
  }
  else
  {
    const int maxRectCuSizebase32 = ((uint32_t)(pu.cs->picture->lwidth() * pu.cs->picture->lheight() * 0.00001722 + 95)) >> 5;
    const int maxRectCuSizeABDMVR = std::max(32, std::min(maxRectCuSizebase32 << 5, 128));
    const int maxRectCuSizeRatiobase32 = ((uint32_t)(pu.cs->picture->lwidth() * pu.cs->picture->lheight() * 0.00000090 + 15)) >> 3;
    const int maxRectCuSizeRatio = std::max(8, std::min(maxRectCuSizeRatiobase32 << 3, 16));
    uint32_t maxLength = std::max(pu.lwidth(), pu.lheight());
    uint32_t minLength = std::min(pu.lwidth(), pu.lheight());

    if (maxLength > maxRectCuSizeABDMVR)
    {
      return false;
    }
    if (maxLength / minLength >= maxRectCuSizeRatio)
    {
      return false;
    }
  }
  return true;
}

bool PU::checkBDMVRCpmvRefinementPuUsage(const PredictionUnit& pu)
{
  // Different conditions for square and rectangular shapes
  if (pu.lwidth() == pu.lheight())
  {
    const int base32 = ((uint32_t) (pu.cs->picture->lwidth() * pu.cs->picture->lheight() * 0.000010558 + 60)) >> 5;
    const int maxSquareCuSizeABDMVR = std::max(32, std::min(base32 << 5, 128));

    if (pu.lwidth() > maxSquareCuSizeABDMVR)
    {
      return false;
    }
  }
  else
  {
    const int base32 = ((uint32_t)(pu.cs->picture->lwidth() * pu.cs->picture->lheight() * 0.0000100751 + 44)) >> 5;
    const int maxRectCuSizeABDMVR = std::max(32, std::min(base32 << 5, 128));
    const int maxRectCuSizeRatio = 8;
    uint32_t maxLength = std::max(pu.lwidth(), pu.lheight());
    uint32_t minLength = std::min(pu.lwidth(), pu.lheight());

    if (maxLength > maxRectCuSizeABDMVR)
    {
      return false;
    }
    if (maxLength / minLength >= maxRectCuSizeRatio)
    {
      return false;
    }
  }
  return true;
}
#endif

#if JVET_AF0163_TM_SUBBLOCK_REFINEMENT
bool PU::checkAffineTMCondition(const PredictionUnit& pu)
{
  Slice &pcSlice = *pu.cu->slice;
  int min = 0;
  if (pcSlice.getSliceType() == B_SLICE && pu.cu->slice->getCheckLDC())
  {
    min = MAX_INT;
    for (int k = 0; k < NUM_REF_PIC_LIST_01; k++)
    {
      for (int iRefIdx = 0; iRefIdx < pcSlice.getNumRefIdx((RefPicList)k); iRefIdx++)
      {
        if (pcSlice.getPOC() - pcSlice.getRefPic((RefPicList)k, iRefIdx)->getPOC() < min)
        {
          min = pcSlice.getPOC() - pcSlice.getRefPic((RefPicList)k, iRefIdx)->getPOC();
        }
      }
    }
  }
  return (min > 1) || !pu.cu->slice->getCheckLDC();
}
#endif
#if MULTI_PASS_DMVR
#if JVET_AE0046_BI_GPM
bool PU::checkBDMVRCondition(const PredictionUnit& pu, bool disregardGpmFlag)
#else
bool PU::checkBDMVRCondition(const PredictionUnit& pu)
#endif
{
  if( pu.cs->sps->getUseDMVDMode() )
  {
#if JVET_AE0046_BI_GPM
    if (disregardGpmFlag == false)
    {
      if (pu.cu->geoFlag)
      {
        return false;
      }
    }
#endif
    const int refIdx0 = pu.refIdx[REF_PIC_LIST_0];
    const int refIdx1 = pu.refIdx[REF_PIC_LIST_1];

    const WPScalingParam *wp0 = pu.cu->slice->getWpScaling( REF_PIC_LIST_0, refIdx0 );
    const WPScalingParam *wp1 = pu.cu->slice->getWpScaling( REF_PIC_LIST_1, refIdx1 );

    const bool isResamplingPossible = pu.cs->sps->getRprEnabledFlag();

    const bool ref0IsScaled = refIdx0 < 0 || refIdx0 >= MAX_NUM_REF
      ? false
      : isResamplingPossible && pu.cu->slice->getRefPic( REF_PIC_LIST_0, refIdx0 )->isRefScaled( pu.cs->pps );
    const bool ref1IsScaled = refIdx1 < 0 || refIdx1 >= MAX_NUM_REF
      ? false
      : isResamplingPossible && pu.cu->slice->getRefPic( REF_PIC_LIST_1, refIdx1 )->isRefScaled( pu.cs->pps );
#if JVET_AG0276_LIC_BDOF_BDMVR
    bool meetBdmvrCondition = ((pu.mergeFlag && pu.mergeType == MRG_TYPE_DEFAULT_N) || (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])) && !pu.ciipFlag && !pu.mmvdMergeFlag
      && !pu.cu->mmvdSkip
#if JVET_AG0067_DMVR_EXTENSIONS
      && (PU::isBiPredFromDifferentDirEqDistPoc( pu ) || (PU::isBiPredFromDifferentDirGenDistPoc( pu ) && !pu.cu->geoFlag))
#else
      && PU::isBiPredFromDifferentDirEqDistPoc(pu)
#endif
      && !WPScalingParam::isWeighted(wp0) && !WPScalingParam::isWeighted(wp1) && !ref0IsScaled && !ref1IsScaled;
    if (pu.cu->geoFlag || pu.cu->affine)
    {
      meetBdmvrCondition &= (pu.cu->licFlag == false);
    }
    if (pu.cu->licFlag == true)
    {
      meetBdmvrCondition &= (pu.tmMergeFlag == false);
    }
#if JVET_AG0276_NLIC
    if (pu.cu->altLMFlag)
    {
      meetBdmvrCondition = false;
    }
#endif
    return meetBdmvrCondition
#if JVET_AG0276_LIC_FLAG_SIGNALING
      && !PU::isOppositeLIC(pu)
#endif
      ;
#else
#if JVET_AB0112_AFFINE_DMVR
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
    return ((pu.mergeFlag && pu.mergeType == MRG_TYPE_DEFAULT_N) || (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])) && !pu.ciipFlag && !pu.mmvdMergeFlag
#else
    return ((pu.mergeFlag && pu.mergeType == MRG_TYPE_DEFAULT_N && (!pu.cu->affine || !pu.afMmvdFlag)) || (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])) && !pu.ciipFlag && !pu.mmvdMergeFlag
#endif
#elif JVET_X0083_BM_AMVP_MERGE_MODE
    return ((pu.mergeFlag && pu.mergeType == MRG_TYPE_DEFAULT_N) || (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])) && !pu.ciipFlag && !pu.cu->affine && !pu.mmvdMergeFlag
#else
    return pu.mergeFlag && pu.mergeType == MRG_TYPE_DEFAULT_N && !pu.ciipFlag && !pu.cu->affine && !pu.mmvdMergeFlag
#endif
      && !pu.cu->mmvdSkip
#if JVET_AG0067_DMVR_EXTENSIONS
      && (PU::isBiPredFromDifferentDirEqDistPoc( pu ) || (PU::isBiPredFromDifferentDirGenDistPoc( pu ) && !pu.cu->geoFlag))
#else
      && PU::isBiPredFromDifferentDirEqDistPoc( pu )
#endif
#if JVET_AG0276_NLIC
      && !pu.cu->altLMFlag
#endif
#if JVET_AD0213_LIC_IMP
      && !pu.cu->licFlag
#if JVET_AG0276_LIC_FLAG_SIGNALING
      && !PU::isOppositeLIC(pu)
#endif
#endif
#if !JVET_Y0089_DMVR_BCW
      && ( pu.cu->bcwIdx == BCW_DEFAULT )
#endif
      && !WPScalingParam::isWeighted( wp0 ) && !WPScalingParam::isWeighted( wp1 ) && !ref0IsScaled && !ref1IsScaled;
#endif
  }
  else
  {
    return false;
  }
}
#if JVET_AG0276_LIC_BDOF_BDMVR
bool PU::checkBDMVRCondition4Aff(const PredictionUnit& pu)
{
  if (pu.cs->sps->getUseDMVDMode())
  {
    const int refIdx0 = pu.refIdx[REF_PIC_LIST_0];
    const int refIdx1 = pu.refIdx[REF_PIC_LIST_1];

    const WPScalingParam *wp0 = pu.cu->slice->getWpScaling(REF_PIC_LIST_0, refIdx0);
    const WPScalingParam *wp1 = pu.cu->slice->getWpScaling(REF_PIC_LIST_1, refIdx1);

    const bool isResamplingPossible = pu.cs->sps->getRprEnabledFlag();

    const bool ref0IsScaled = refIdx0 < 0 || refIdx0 >= MAX_NUM_REF
      ? false
      : isResamplingPossible && pu.cu->slice->getRefPic(REF_PIC_LIST_0, refIdx0)->isRefScaled(pu.cs->pps);
    const bool ref1IsScaled = refIdx1 < 0 || refIdx1 >= MAX_NUM_REF
      ? false
      : isResamplingPossible && pu.cu->slice->getRefPic(REF_PIC_LIST_1, refIdx1)->isRefScaled(pu.cs->pps);
#if JVET_AB0112_AFFINE_DMVR
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
    return ((pu.mergeFlag && pu.mergeType == MRG_TYPE_DEFAULT_N) || (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])) && !pu.ciipFlag && !pu.mmvdMergeFlag
#else
    return ((pu.mergeFlag && pu.mergeType == MRG_TYPE_DEFAULT_N && (!pu.cu->affine || !pu.afMmvdFlag)) || (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])) && !pu.ciipFlag && !pu.mmvdMergeFlag
#endif
#elif JVET_X0083_BM_AMVP_MERGE_MODE
    return ((pu.mergeFlag && pu.mergeType == MRG_TYPE_DEFAULT_N) || (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])) && !pu.ciipFlag && !pu.cu->affine && !pu.mmvdMergeFlag
#else
    return pu.mergeFlag && pu.mergeType == MRG_TYPE_DEFAULT_N && !pu.ciipFlag && !pu.cu->affine && !pu.mmvdMergeFlag
#endif
      && !pu.cu->mmvdSkip
#if JVET_AG0067_DMVR_EXTENSIONS
      && (PU::isBiPredFromDifferentDirEqDistPoc(pu) || (PU::isBiPredFromDifferentDirGenDistPoc(pu) && !pu.cu->geoFlag))
#else
      && PU::isBiPredFromDifferentDirEqDistPoc(pu)
#endif
#if JVET_AG0276_NLIC
      && !pu.cu->altLMFlag
#endif
#if JVET_AG0276_LIC_FLAG_SIGNALING
      && !PU::isOppositeLIC(pu)
#endif
#if !JVET_Y0089_DMVR_BCW
      && (pu.cu->bcwIdx == BCW_DEFAULT)
#endif
      && !WPScalingParam::isWeighted(wp0) && !WPScalingParam::isWeighted(wp1) && !ref0IsScaled && !ref1IsScaled;
  }
  else
  {
    return false;
  }
}
#endif
#if JVET_AG0135_AFFINE_CIIP
#if JVET_AE0046_BI_GPM
bool PU::checkBDMVRConditionCIIPAffine(const PredictionUnit& pu, bool disregardGpmFlag)
#else
bool PU::checkBDMVRConditionCIIPAffine(const PredictionUnit& pu)
#endif
{
  if (pu.cs->sps->getUseDMVDMode())
  {
#if JVET_AE0046_BI_GPM
    if (disregardGpmFlag == false)
    {
      if (pu.cu->geoFlag)
      {
        return false;
      }
    }
#endif
    const int refIdx0 = pu.refIdx[REF_PIC_LIST_0];
    const int refIdx1 = pu.refIdx[REF_PIC_LIST_1];

    const WPScalingParam *wp0 = pu.cu->slice->getWpScaling(REF_PIC_LIST_0, refIdx0);
    const WPScalingParam *wp1 = pu.cu->slice->getWpScaling(REF_PIC_LIST_1, refIdx1);

    const bool isResamplingPossible = pu.cs->sps->getRprEnabledFlag();

    const bool ref0IsScaled = refIdx0 < 0 || refIdx0 >= MAX_NUM_REF
      ? false
      : isResamplingPossible && pu.cu->slice->getRefPic(REF_PIC_LIST_0, refIdx0)->isRefScaled(pu.cs->pps);
    const bool ref1IsScaled = refIdx1 < 0 || refIdx1 >= MAX_NUM_REF
      ? false
      : isResamplingPossible && pu.cu->slice->getRefPic(REF_PIC_LIST_1, refIdx1)->isRefScaled(pu.cs->pps);
#if JVET_AB0112_AFFINE_DMVR
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
    return ((pu.mergeFlag && pu.mergeType == MRG_TYPE_DEFAULT_N) || (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])) && !pu.mmvdMergeFlag
#else
    return ((pu.mergeFlag && pu.mergeType == MRG_TYPE_DEFAULT_N && (!pu.cu->affine || !pu.afMmvdFlag)) || (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])) && !pu.mmvdMergeFlag
#endif
#elif JVET_X0083_BM_AMVP_MERGE_MODE
    return ((pu.mergeFlag && pu.mergeType == MRG_TYPE_DEFAULT_N) || (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])) && !pu.cu->affine && !pu.mmvdMergeFlag
#else
    return pu.mergeFlag && pu.mergeType == MRG_TYPE_DEFAULT_N && !pu.cu->affine && !pu.mmvdMergeFlag
#endif
      && !pu.cu->mmvdSkip
#if JVET_AG0067_DMVR_EXTENSIONS
      && (PU::isBiPredFromDifferentDirEqDistPoc(pu) || (PU::isBiPredFromDifferentDirGenDistPoc(pu) && !pu.cu->geoFlag))
#else
      && PU::isBiPredFromDifferentDirEqDistPoc(pu)
#endif
#if JVET_AD0213_LIC_IMP && !JVET_AG0276_LIC_BDOF_BDMVR
      && !pu.cu->licFlag
#endif
#if JVET_AG0276_NLIC
      && !pu.cu->altLMFlag
#endif

#if !JVET_Y0089_DMVR_BCW
      && (pu.cu->bcwIdx == BCW_DEFAULT)
#endif
      && !WPScalingParam::isWeighted(wp0) && !WPScalingParam::isWeighted(wp1) && !ref0IsScaled && !ref1IsScaled;
  }
  else
  {
    return false;
  }
}
#endif
#endif

#if INTER_LIC && RPR_ENABLE
bool PU::checkRprLicCondition( const PredictionUnit & pu )
{
  if ( pu.cs->sps->getLicEnabledFlag() )
  {
#if JVET_Y0128_NON_CTC
#if JVET_AD0213_LIC_IMP
    if (pu.interDir != 1 && pu.interDir != 2 && pu.interDir != 3)
#else
    if(pu.interDir != 1 && pu.interDir != 2)
#endif
#else
#if JVET_AD0213_LIC_IMP
    if (pu.refIdx[0] < 0 && pu.refIdx[1] < 0)
#else
    if( pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0 )
#endif
#endif
    {
      return false;
    }

    bool  bLicEnabled = true;
#if JVET_AD0213_LIC_IMP
#if JVET_Y0128_NON_CTC
    if (pu.interDir == 3)
#else
    if (pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0)
#endif
    {
      const bool isResamplingPossible = pu.cs->sps->getRprEnabledFlag();
      for (int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
      {
        if (isResamplingPossible && pu.cu->slice->getRefPic(RefPicList(refList), pu.refIdx[refList])->isRefScaled(pu.cs->pps))
        {
          bLicEnabled = false;
          break;
        }
      }
    }
    else
    {
#endif
#if JVET_Y0128_NON_CTC
    int iRefIdx             = (pu.interDir == 1) ? pu.refIdx[0] : pu.refIdx[1];
    RefPicList  eRefPicList = (pu.interDir == 1) ? REF_PIC_LIST_0 : REF_PIC_LIST_1;
#else
    int iRefIdx             = (pu.refIdx[0] >= 0) ? pu.refIdx[0] : pu.refIdx[1];
    RefPicList  eRefPicList = (pu.refIdx[0] >= 0) ? REF_PIC_LIST_0 : REF_PIC_LIST_1;
#endif

    const bool isResamplingPossible = pu.cs->sps->getRprEnabledFlag();
    if( isResamplingPossible && pu.cu->slice->getRefPic( eRefPicList, iRefIdx )->isRefScaled( pu.cs->pps ) )
    {
      bLicEnabled = false;
    }
#if JVET_AD0213_LIC_IMP
    }
#endif
    return bLicEnabled;
  }
  else
  {
    return false;
  }
}
#endif

#if JVET_Y0128_NON_CTC || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && TM_AMVP)
bool PU::checkTmEnableCondition(const SPS* sps, const PPS* pps, const Picture* refPic)
{
#if TM_AMVP || TM_MRG || MULTI_PASS_DMVR
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && TM_AMVP
  if (sps->getUseTMAmvpMode())
#else
  if (sps->getUseDMVDMode())
#endif
  {
    if (sps->getRprEnabledFlag())
    {
      return !refPic->isRefScaled(pps);
    }
    else
    {
      return true;
    }
  }
  else
#endif
  {
    return false;
  }
}
#endif

#if JVET_Z0056_GPM_SPLIT_MODE_REORDERING
#if JVET_AG0164_AFFINE_GPM
bool PU::checkRprRefExistingInGpm(const PredictionUnit& pu, const MergeCtx& geoMrgCtx0, uint8_t candIdx0, const MergeCtx& geoMrgCtx1, uint8_t candIdx1, const AffineMergeCtx& affMergeCtx)
{
  if (pu.cs->sps->getRprEnabledFlag())
  {
    auto xCheckUseRprPerPart = [&pu](const MergeCtx& mrgCtx, const AffineMergeCtx& affCtx, uint8_t candIdx, uint8_t partIdx)
    {
#if JVET_Y0065_GPM_INTRA
      if (candIdx >= GEO_MAX_ALL_INTER_UNI_CANDS)
      {
        return false;
      }
#endif
      if (pu.affineGPM[partIdx])
      {
        int realCandIdx = candIdx;
        if (candIdx >= affCtx.m_indexOffset)
        {
          realCandIdx -= affCtx.m_indexOffset;
        }
#if JVET_AE0046_BI_GPM
        if (affCtx.interDirNeighbours[realCandIdx] == 3)
        {
          int refIdxL0 = affCtx.mvFieldNeighbours[(realCandIdx << 1)][0].refIdx;
          int refIdxL1 = affCtx.mvFieldNeighbours[(realCandIdx << 1) + 1][0].refIdx;
          return pu.cu->slice->getRefPic(REF_PIC_LIST_0, refIdxL0)->isRefScaled(pu.cs->pps) || pu.cu->slice->getRefPic(REF_PIC_LIST_1, refIdxL1)->isRefScaled(pu.cs->pps);
        }
#endif
        int refList = affCtx.interDirNeighbours[realCandIdx] - 1;
        int refIdx = affCtx.mvFieldNeighbours[(realCandIdx << 1) + refList][0].refIdx;
        return pu.cu->slice->getRefPic((RefPicList)refList, refIdx)->isRefScaled(pu.cs->pps);
      }
      else
      {
#if JVET_AE0046_BI_GPM
        if (mrgCtx.interDirNeighbours[candIdx] == 3)
        {
          int refIdxL0 = mrgCtx.mvFieldNeighbours[(candIdx << 1)].refIdx;
          int refIdxL1 = mrgCtx.mvFieldNeighbours[(candIdx << 1) + 1].refIdx;
          return pu.cu->slice->getRefPic(REF_PIC_LIST_0, refIdxL0)->isRefScaled(pu.cs->pps) || pu.cu->slice->getRefPic(REF_PIC_LIST_1, refIdxL1)->isRefScaled(pu.cs->pps);
        }
#endif
        int refList = mrgCtx.interDirNeighbours[candIdx] - 1;
        int refIdx = mrgCtx.mvFieldNeighbours[(candIdx << 1) + refList].refIdx;
        return pu.cu->slice->getRefPic((RefPicList)refList, refIdx)->isRefScaled(pu.cs->pps);
      }
    };
    return xCheckUseRprPerPart(geoMrgCtx0, affMergeCtx, candIdx0, 0) || xCheckUseRprPerPart(geoMrgCtx1, affMergeCtx,candIdx1,  1);
  }

  return false;
}
#endif
bool PU::checkRprRefExistingInGpm(const PredictionUnit& pu, const MergeCtx& geoMrgCtx0, uint8_t candIdx0, const MergeCtx& geoMrgCtx1, uint8_t candIdx1)
{
#if !JVET_AF0190_RPR_TMP_REORDER_LIC
  if (pu.cs->sps->getRprEnabledFlag())
  {
    auto xCheckUseRprPerPart = [&pu](const MergeCtx& mrgCtx, uint8_t candIdx)
    {
#if JVET_Y0065_GPM_INTRA
      if (candIdx >= GEO_MAX_NUM_UNI_CANDS)
      {
        return false;
      }
#endif
#if JVET_AE0046_BI_GPM
      if (mrgCtx.interDirNeighbours[candIdx] == 3)
      {
        int refIdxL0 = mrgCtx.mvFieldNeighbours[(candIdx << 1)].refIdx;
        int refIdxL1 = mrgCtx.mvFieldNeighbours[(candIdx << 1) + 1].refIdx;
        return pu.cu->slice->getRefPic(REF_PIC_LIST_0, refIdxL0)->isRefScaled(pu.cs->pps) || pu.cu->slice->getRefPic(REF_PIC_LIST_1, refIdxL1)->isRefScaled(pu.cs->pps);
      }
#endif
      int refList = mrgCtx.interDirNeighbours[candIdx] - 1;
      int refIdx  = mrgCtx.mvFieldNeighbours[(candIdx << 1) + refList].refIdx;
      return pu.cu->slice->getRefPic((RefPicList)refList, refIdx)->isRefScaled(pu.cs->pps);
    };

    return xCheckUseRprPerPart(geoMrgCtx0, candIdx0) || xCheckUseRprPerPart(geoMrgCtx1, candIdx1);
  }
#endif

  return false;
}
#endif

static int xGetDistScaleFactor(const int &iCurrPOC, const int &iCurrRefPOC, const int &iColPOC, const int &iColRefPOC)
{
  int iDiffPocD = iColPOC - iColRefPOC;
  int iDiffPocB = iCurrPOC - iCurrRefPOC;

  if (iDiffPocD == iDiffPocB)
  {
    return 4096;
  }
  else
  {
    int iTDB = Clip3(-128, 127, iDiffPocB);
    int iTDD = Clip3(-128, 127, iDiffPocD);
    int iX = (0x4000 + abs(iTDD / 2)) / iTDD;
    int iScale = Clip3(-4096, 4095, (iTDB * iX + 32) >> 6);
    return iScale;
  }
}

int convertMvFixedToFloat(int32_t val)
{
  int sign  = val >> 31;
  int scale = floorLog2((val ^ sign) | MV_MANTISSA_UPPER_LIMIT) - (MV_MANTISSA_BITCOUNT - 1);

  int exponent;
  int mantissa;
  if (scale >= 0)
  {
    int round = (1 << scale) >> 1;
    int n     = (val + round) >> scale;
    exponent  = scale + ((n ^ sign) >> (MV_MANTISSA_BITCOUNT - 1));
    mantissa  = (n & MV_MANTISSA_UPPER_LIMIT) | (sign << (MV_MANTISSA_BITCOUNT - 1));
  }
  else
  {
    exponent = 0;
    mantissa = val;
  }

  return exponent | (mantissa << MV_EXPONENT_BITCOUNT);
}

int convertMvFloatToFixed(int val)
{
  int exponent = val & MV_EXPONENT_MASK;
  int mantissa = val >> MV_EXPONENT_BITCOUNT;
  return exponent == 0 ? mantissa : (mantissa ^ MV_MANTISSA_LIMIT) << (exponent - 1);
}

int roundMvComp(int x)
{
  return convertMvFloatToFixed(convertMvFixedToFloat(x));
}

int PU::getDistScaleFactor(const int &currPOC, const int &currRefPOC, const int &colPOC, const int &colRefPOC)
{
  return xGetDistScaleFactor(currPOC, currRefPOC, colPOC, colRefPOC);
}

void PU::getInterMMVDMergeCandidates(const PredictionUnit &pu, MergeCtx& mrgCtx, const int& mrgCandIdx)
{
  int refIdxList0, refIdxList1;
  int k;
  int currBaseNum = 0;
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  const uint16_t maxNumMergeCand = std::min<int>(mrgCtx.numValidMergeCand, (pu.cs->sps->getUseTMMMVD() ? MMVD_BASE_MV_NUM : VVC_MMVD_BASE_MV_NUM));
#else
  const uint16_t maxNumMergeCand = std::min<int>(mrgCtx.numValidMergeCand, MMVD_BASE_MV_NUM);
#endif
#else
  const uint16_t maxNumMergeCand = mrgCtx.numValidMergeCand;
#endif

  for( k = 0; k < maxNumMergeCand; k++ )
  {
    refIdxList0 = mrgCtx.mvFieldNeighbours[( k << 1 )].refIdx;
    refIdxList1 = mrgCtx.mvFieldNeighbours[( k << 1 ) + 1].refIdx;

    if( ( refIdxList0 >= 0 ) && ( refIdxList1 >= 0 ) )
    {
      mrgCtx.mmvdBaseMv[currBaseNum][0] = mrgCtx.mvFieldNeighbours[( k << 1 )];
      mrgCtx.mmvdBaseMv[currBaseNum][1] = mrgCtx.mvFieldNeighbours[( k << 1 ) + 1];
    }
    else if( refIdxList0 >= 0 )
    {
      mrgCtx.mmvdBaseMv[currBaseNum][0] = mrgCtx.mvFieldNeighbours[( k << 1 )];
      mrgCtx.mmvdBaseMv[currBaseNum][1] = MvField( Mv( 0, 0 ), -1 );
    }
    else if( refIdxList1 >= 0 )
    {
      mrgCtx.mmvdBaseMv[currBaseNum][0] = MvField( Mv( 0, 0 ), -1 );
      mrgCtx.mmvdBaseMv[currBaseNum][1] = mrgCtx.mvFieldNeighbours[( k << 1 ) + 1];
    }
    mrgCtx.mmvdUseAltHpelIf[currBaseNum] = mrgCtx.useAltHpelIf[k];
    currBaseNum++;

#if !JVET_AA0132_CONFIGURABLE_TM_TOOLS
    if( currBaseNum == MMVD_BASE_MV_NUM )
    {
      break;
    }
#endif
  }
}
bool PU::getColocatedMVP(const PredictionUnit &pu, const RefPicList &eRefPicList, const Position &_pos, Mv& rcMv, const int &refIdx, bool sbFlag
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  , int col
#endif
  , int* targetRefIdx
#endif
)
{
  // don't perform MV compression when generally disabled or subPuMvp is used
  const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
  const unsigned mask  = ~( scale - 1 );

  const Position pos = Position{ PosType( _pos.x & mask ), PosType( _pos.y & mask ) };

  const Slice &slice = *pu.cs->slice;

  // use coldir.
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  const Picture* const pColPic = slice.getRefPic(RefPicList(col == 0 ? 1 - slice.getColFromL0Flag() : 1 - slice.getColFromL0Flag2nd()), col == 0 ? slice.getColRefIdx() : slice.getColRefIdx2nd());
#else
  const Picture* const pColPic = slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0), slice.getColRefIdx());
#endif

#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  bool isColPicScaled = pColPic->isRefScaled(pu.cs->pps);
  if (!pColPic || isColPicScaled)
#else
  if( !pColPic )
#endif
  {
    return false;
  }

  // Check the position of colocated block is within a subpicture
  const SubPic &curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
  if (curSubPic.getTreatedAsPicFlag())
  {
    if (!curSubPic.isContainingPos(pos))
    {
      return false;
    }
  }
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  int pocCur = slice.getPOC();
  int pocCol = pColPic->getPOC();
  RefPicList eColRefPicList = slice.getCheckLDC() ? eRefPicList : RefPicList(pocCur > pocCol);
#else
  RefPicList eColRefPicList = slice.getCheckLDC() ? eRefPicList : RefPicList(slice.getColFromL0Flag());
#endif

  const MotionInfo& mi = pColPic->cs->getMotionInfo( pos );

  if( !mi.isInter )
  {
    return false;
  }
  if (mi.isIBCmot)
  {
    return false;
  }
  if (CU::isIBC(*pu.cu))
  {
    return false;
  }
  int iColRefIdx = mi.refIdx[eColRefPicList];

  if (sbFlag && !slice.getCheckLDC())
  {
    eColRefPicList = eRefPicList;
    iColRefIdx = mi.refIdx[eColRefPicList];
    if (iColRefIdx < 0)
    {
      return false;
    }
  }
  else
  {
    if (iColRefIdx < 0)
    {
      eColRefPicList = RefPicList(1 - eColRefPicList);
      iColRefIdx = mi.refIdx[eColRefPicList];

      if (iColRefIdx < 0)
      {
        return false;
      }
    }
  }

  const Slice *pColSlice = nullptr;

  for( const auto s : pColPic->slices )
  {
    if( s->getIndependentSliceIdx() == mi.sliceIdx )
    {
      pColSlice = s;
      break;
    }
  }

  CHECK( pColSlice == nullptr, "Slice segment not found" );

  const Slice &colSlice = *pColSlice;
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
  if (targetRefIdx != nullptr)
  {
    *targetRefIdx = slice.getImRefIdx(mi.sliceIdx, eColRefPicList, eRefPicList, iColRefIdx
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
      , col
#endif
    );
    if (*targetRefIdx == -1)
    {
      return false;
    }

    const bool bIsCurrRefLongTerm = slice.getRefPic(eRefPicList, *targetRefIdx)->longTerm;
    const bool bIsColRefLongTerm = colSlice.getIsUsedAsLongTerm(eColRefPicList, iColRefIdx);
    if (bIsCurrRefLongTerm != bIsColRefLongTerm)
    {
      return false;
    }

    // Scale the vector.
    Mv cColMv = mi.mv[eColRefPicList];
    cColMv.setHor(roundMvComp(cColMv.getHor()));
    cColMv.setVer(roundMvComp(cColMv.getVer()));

    if (bIsCurrRefLongTerm /*|| bIsColRefLongTerm*/)
    {
      rcMv = cColMv;
      rcMv.clipToStorageBitDepth();
    }
    else
    {
      const int currPOC = slice.getPOC();
      const int colPOC = colSlice.getPOC();
      const int colRefPOC = colSlice.getRefPOC(eColRefPicList, iColRefIdx);
      const int currRefPOC = slice.getRefPic(eRefPicList, *targetRefIdx)->getPOC();
      const int distscale = xGetDistScaleFactor(currPOC, currRefPOC, colPOC, colRefPOC);

      if (distscale == 4096)
      {
        rcMv = cColMv;
        rcMv.clipToStorageBitDepth();
      }
      else
      {
        rcMv = cColMv.scaleMv(distscale);
      }
    }

    return true;
  }
  else
  {
#endif
  const bool bIsCurrRefLongTerm = slice.getRefPic(eRefPicList, refIdx)->longTerm;
  const bool bIsColRefLongTerm  = colSlice.getIsUsedAsLongTerm(eColRefPicList, iColRefIdx);

  if (bIsCurrRefLongTerm != bIsColRefLongTerm)
  {
    return false;
  }


  // Scale the vector.
  Mv cColMv = mi.mv[eColRefPicList];
  cColMv.setHor(roundMvComp(cColMv.getHor()));
  cColMv.setVer(roundMvComp(cColMv.getVer()));

  if (bIsCurrRefLongTerm /*|| bIsColRefLongTerm*/)
  {
    rcMv = cColMv;
    rcMv.clipToStorageBitDepth();
  }
  else
  {
    const int currPOC    = slice.getPOC();
    const int colPOC     = colSlice.getPOC();
    const int colRefPOC  = colSlice.getRefPOC(eColRefPicList, iColRefIdx);
    const int currRefPOC = slice.getRefPic(eRefPicList, refIdx)->getPOC();
    const int distscale  = xGetDistScaleFactor(currPOC, currRefPOC, colPOC, colRefPOC);

    if (distscale == 4096)
    {
      rcMv = cColMv;
      rcMv.clipToStorageBitDepth();
    }
    else
    {
      rcMv = cColMv.scaleMv(distscale);
    }
  }

  return true;
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
  }
#endif
}

bool PU::isDiffMER(const Position &pos1, const Position &pos2, const unsigned plevel)
{
  const unsigned xN = pos1.x;
  const unsigned yN = pos1.y;
  const unsigned xP = pos2.x;
  const unsigned yP = pos2.y;

  if ((xN >> plevel) != (xP >> plevel))
  {
    return true;
  }

  if ((yN >> plevel) != (yP >> plevel))
  {
    return true;
  }

  return false;
}

bool PU::isAddNeighborMv(const Mv& currMv, Mv* neighborMvs, int numNeighborMv)
{
  bool existed = false;
  for (uint32_t cand = 0; cand < numNeighborMv && !existed; cand++)
  {
    if (currMv == neighborMvs[cand])
    {
      existed = true;
    }
  }

  if (!existed)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void PU::getIbcMVPsEncOnly(PredictionUnit &pu, Mv* mvPred, int& nbPred)
{
  const PreCalcValues   &pcv = *pu.cs->pcv;
  const int  cuWidth = pu.blocks[COMPONENT_Y].width;
  const int  cuHeight = pu.blocks[COMPONENT_Y].height;
  const int  log2UnitWidth = floorLog2(pcv.minCUWidth);
  const int  log2UnitHeight = floorLog2(pcv.minCUHeight);
  const int  totalAboveUnits = (cuWidth >> log2UnitWidth) + 1;
  const int  totalLeftUnits = (cuHeight >> log2UnitHeight) + 1;

  nbPred = 0;
  Position posLT = pu.Y().topLeft();

#if JVET_Z0118_GDR
  bool isClean = pu.cs->isClean(pu.cu->Y().bottomRight(), CHANNEL_TYPE_LUMA);
#endif

  // above-left
  const PredictionUnit *aboveLeftPU = pu.cs->getPURestricted(posLT.offset(-1, -1), pu, CHANNEL_TYPE_LUMA);
  if (aboveLeftPU && CU::isIBC(*aboveLeftPU->cu))
  {
    if (isAddNeighborMv(aboveLeftPU->bv, mvPred, nbPred))
    {
      mvPred[nbPred++] = aboveLeftPU->bv;
    }
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (aboveLeftPU->interDir == 3)
    {
      Mv bv = aboveLeftPU->mv[REF_PIC_LIST_1];
      bv.changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
      if (isAddNeighborMv(bv, mvPred, nbPred))
      {
        mvPred[nbPred++] = bv;
      }
    }
#endif
  }

  // above neighbors
  for (uint32_t dx = 0; dx < totalAboveUnits && nbPred < IBC_NUM_CANDIDATES; dx++)
  {
    const PredictionUnit* tmpPU = pu.cs->getPURestricted(posLT.offset((dx << log2UnitWidth), -1), pu, CHANNEL_TYPE_LUMA);
    if (tmpPU && CU::isIBC(*tmpPU->cu))
    {
      if (isAddNeighborMv(tmpPU->bv, mvPred, nbPred))
      {
        mvPred[nbPred++] = tmpPU->bv;
      }
#if JVET_AE0169_BIPREDICTIVE_IBC
      if (tmpPU->interDir == 3)
      {
        Mv bv = tmpPU->mv[REF_PIC_LIST_1];
        bv.changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
        if (isAddNeighborMv(bv, mvPred, nbPred))
        {
          mvPred[nbPred++] = bv;
        }
      }
#endif
    }
  }

  // left neighbors
  for (uint32_t dy = 0; dy < totalLeftUnits && nbPred < IBC_NUM_CANDIDATES; dy++)
  {
    const PredictionUnit* tmpPU = pu.cs->getPURestricted(posLT.offset(-1, (dy << log2UnitHeight)), pu, CHANNEL_TYPE_LUMA);
    if (tmpPU && CU::isIBC(*tmpPU->cu))
    {
      if (isAddNeighborMv(tmpPU->bv, mvPred, nbPred))
      {
        mvPred[nbPred++] = tmpPU->bv;
      }
#if JVET_AE0169_BIPREDICTIVE_IBC
      if (tmpPU->interDir == 3)
      {
        Mv bv = tmpPU->mv[REF_PIC_LIST_1];
        bv.changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
        if (isAddNeighborMv(bv, mvPred, nbPred))
        {
          mvPred[nbPred++] = bv;
        }
      }
#endif
    }
  }

#if JVET_Z0118_GDR
  size_t numAvaiCandInLUT = isClean ? pu.cs->motionLut.lutIbc1.size() : pu.cs->motionLut.lutIbc0.size();
#else
  size_t numAvaiCandInLUT = pu.cs->motionLut.lutIbc.size();
#endif

  for (uint32_t cand = 0; cand < numAvaiCandInLUT && nbPred < IBC_NUM_CANDIDATES; cand++)
  {
#if JVET_Z0118_GDR
    MotionInfo neibMi = isClean ? pu.cs->motionLut.lutIbc1[cand] : pu.cs->motionLut.lutIbc0[cand];
#else
    MotionInfo neibMi = pu.cs->motionLut.lutIbc[cand];
#endif
    if (isAddNeighborMv(neibMi.bv, mvPred, nbPred))
    {
      mvPred[nbPred++] = neibMi.bv;
    }
  }

  bool isBvCandDerived[IBC_NUM_CANDIDATES];
  ::memset(isBvCandDerived, false, IBC_NUM_CANDIDATES);

  int curNbPred = nbPred;
  if (curNbPred < IBC_NUM_CANDIDATES)
  {
    do
    {
      curNbPred = nbPred;
      for (uint32_t idx = 0; idx < curNbPred && nbPred < IBC_NUM_CANDIDATES; idx++)
      {
        if (!isBvCandDerived[idx])
        {
          Mv derivedBv;
#if JVET_AE0169_BIPREDICTIVE_IBC
          Mv derivedBvL1 = Mv();
          if (getDerivedBV(pu, mvPred[idx], derivedBv, derivedBvL1))
#else
          if (getDerivedBV(pu, mvPred[idx], derivedBv))
#endif
          {
            if (isAddNeighborMv(derivedBv, mvPred, nbPred))
            {
              mvPred[nbPred++] = derivedBv;
            }
#if JVET_AE0169_BIPREDICTIVE_IBC
            if (nbPred < IBC_NUM_CANDIDATES && derivedBvL1 != Mv())
            {
              if (isAddNeighborMv(derivedBvL1, mvPred, nbPred))
              {
                mvPred[nbPred++] = derivedBvL1;
              }
            }
#endif
          }
          isBvCandDerived[idx] = true;
        }
      }
    } while (nbPred > curNbPred && nbPred < IBC_NUM_CANDIDATES);
  }
}

#if JVET_AE0169_BIPREDICTIVE_IBC
bool PU::getDerivedBV(PredictionUnit &pu, const Mv& currentMv, Mv& derivedMv, Mv& derivedMvL1)
#else
bool PU::getDerivedBV(PredictionUnit &pu, const Mv& currentMv, Mv& derivedMv)
#endif
{
  int   cuPelX = pu.lumaPos().x;
  int   cuPelY = pu.lumaPos().y;
  int rX = cuPelX + currentMv.getHor();
  int rY = cuPelY + currentMv.getVer();
  int offsetX = currentMv.getHor();
  int offsetY = currentMv.getVer();

  if( rX < 0 || rY < 0 || rX >= pu.cs->slice->getPPS()->getPicWidthInLumaSamples() || rY >= pu.cs->slice->getPPS()->getPicHeightInLumaSamples() )
  {
    return false;
  }

  const PredictionUnit *neibRefPU = NULL;
  neibRefPU = pu.cs->getPURestricted(pu.lumaPos().offset(offsetX, offsetY), pu, CHANNEL_TYPE_LUMA);

  bool isIBC = (neibRefPU) ? CU::isIBC(*neibRefPU->cu) : 0;
  if (isIBC)
  {
    derivedMv = neibRefPU->bv;
    derivedMv += currentMv;
#if JVET_AE0169_BIPREDICTIVE_IBC
    if (neibRefPU->interDir == 3)
    {
      derivedMvL1 = neibRefPU->mv[REF_PIC_LIST_1];
      derivedMvL1.changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
      derivedMvL1 += currentMv;
    }
#endif
  }
  return isIBC;
}

#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
#if JVET_AA0070_RRIBC || JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
inline void PU::getRribcBvpCand(PredictionUnit &pu, AMVPInfo *pInfo)
{
  pInfo->numCand = 2;
  if (pu.cu->rribcFlipType == 1)
  {
    pInfo->mvCand[0] = Mv(std::max(-static_cast<int>(pu.lwidth()), -pu.Y().x),0);
    pInfo->mvCand[0].changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
    pInfo->mvCand[1] = Mv(- pu.Y().x,0);
    pInfo->mvCand[1].changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
  }
  else if (pu.cu->rribcFlipType == 2)
  {
    const int ctbSize     = pu.cs->sps->getCTUSize();
    const int numCurrCtuY = (pu.Y().y >> (floorLog2(ctbSize)));
    unsigned int lcuWidth = pu.cs->slice->getSPS()->getMaxCUWidth();
    int rrTop;
    if (256 == lcuWidth)
    {
      rrTop = (numCurrCtuY < 2) ? -pu.Y().y : -((pu.Y().y & (ctbSize - 1)) + ctbSize);
    }
    else
    {
      rrTop = (numCurrCtuY < 3) ? -pu.Y().y : -((pu.Y().y & (ctbSize - 1)) + 2 * ctbSize);
    }
    pInfo->mvCand[0] = Mv(0,std::max(-static_cast<int>(pu.lheight()), rrTop));
    pInfo->mvCand[0].changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
    pInfo->mvCand[1] = Mv(0,rrTop);
    pInfo->mvCand[1].changePrecision(MV_PRECISION_INT, MV_PRECISION_INTERNAL);
  }
}
#endif

inline void PU::clusterBvpCand(const int cbWidth, const int cbHeight, AMVPInfo *pInfo)
{
  int                                                numGroups = 0;
  std::array<bool, (REGULAR_AMVP_MAX_NUM_CANDS + 1)> validCand;
  validCand.fill(true);
  Mv         bestCand;
  Distortion bestCost;
  int        sqrRadius;
  sqrRadius = std::max(2, floorLog2((cbWidth * cbHeight) >> MIN_PU_SIZE));
  sqrRadius = (sqrRadius * sqrRadius) << 8;
  // Clustering of AMVP candidates into two groups
  if (pInfo->numCand > AMVP_MAX_NUM_CANDS)
  {
    int numCand = std::min((int) pInfo->numCand, REGULAR_AMVP_MAX_NUM_CANDS + 1);
    for (int i = 0; i < numCand; i++)
    {
      if (validCand[i])
      {
        bestCand = pInfo->mvCand[i];
        bestCost = pInfo->mvCost[i];
      }
      else
      {
        continue;
      }
      for (int j = i + 1; j < numCand; j++)
      {
        if (validCand[j])
        {
          Mv mvDiff = pInfo->mvCand[i] - pInfo->mvCand[j];

          if ((mvDiff.getAbsHor() * mvDiff.getAbsHor() + mvDiff.getAbsVer() * mvDiff.getAbsVer())
              <= sqrRadius)   // next candidate is inside the radius
          {
            validCand[j] = false;
            if (bestCost > pInfo->mvCost[j])
            {
              bestCand = pInfo->mvCand[j];
              bestCost = pInfo->mvCost[j];
            }
          }
        }
      }
      pInfo->mvCand[numGroups] = bestCand;
      pInfo->mvCost[numGroups] = bestCost;
      if (++numGroups >= AMVP_MAX_NUM_CANDS)
      {
        break;
      }
    }
    pInfo->numCand = AMVP_MAX_NUM_CANDS;
  }
}
#endif

/**
 * Constructs a list of candidates for IBC AMVP (See specification, section "Derivation process for motion vector predictor candidates")
 */
#if (JVET_Z0084_IBC_TM && IBC_TM_AMVP) || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
#if JVET_AE0169_BIPREDICTIVE_IBC
void PU::fillIBCMvpCand(PredictionUnit &pu, AMVPInfo &amvpInfo, MergeCtx& mergeCtx, InterPrediction* pcInter)
#else
void PU::fillIBCMvpCand(PredictionUnit &pu, AMVPInfo &amvpInfo, InterPrediction* pcInter)
#endif
#else
void PU::fillIBCMvpCand(PredictionUnit &pu, AMVPInfo &amvpInfo)
#endif
{
  AMVPInfo *pInfo = &amvpInfo;
  pInfo->numCand  = 0;
#if !JVET_AE0169_BIPREDICTIVE_IBC
  MergeCtx mergeCtx;
#endif

#if JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
  if (pu.isBvpClusterApplicable())
  {
#if JVET_AA0070_RRIBC
    if (pu.cu->rribcFlipType)
    {
      getRribcBvpCand(pu, pInfo);
    }
    else
#endif
    {
      const int  cbWidth      = pu.lwidth();
      const int  cbHeight     = pu.lheight();
      const bool availableTop = (pu.cs->getPURestricted(pu.Y().topRight().offset(0, -1), pu, pu.chType)) ? true : false;
      const bool availableLeft =
        (pu.cs->getPURestricted(pu.Y().bottomLeft().offset(-1, 0), pu, pu.chType)) ? true : false;
      const CPelBuf pRecY           = pu.cs->picture->getRecoBuf(pu.cs->picture->blocks[COMPONENT_Y]);
      pInfo->maxSimilarityThreshold = 1;

      PU::getIBCMergeCandidates(pu, mergeCtx);
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      pu.cs->m_pcBufPredCurTop  = PelBuf(pcInter->getCurAMLTemplate< true, 0>(), cbWidth,                 AML_MERGE_TEMPLATE_SIZE);
      pu.cs->m_pcBufPredCurLeft = PelBuf(pcInter->getCurAMLTemplate<false, 0>(), AML_MERGE_TEMPLATE_SIZE, cbHeight               );
      pu.cs->m_pcBufPredRefTop  = PelBuf(pcInter->getRefAMLTemplate< true, 0>(), cbWidth,                 AML_MERGE_TEMPLATE_SIZE);
      pu.cs->m_pcBufPredRefLeft = PelBuf(pcInter->getRefAMLTemplate<false, 0>(), AML_MERGE_TEMPLATE_SIZE, cbHeight               );
#else
      pu.cs->createTMBuf(cbWidth, cbHeight);
#endif
      PU::getTemplateTop(availableTop, pu, pRecY, pu.cs->m_pcBufPredCurTop, Position(0, -AML_MERGE_TEMPLATE_SIZE),
                         cbWidth, AML_MERGE_TEMPLATE_SIZE);
      PU::getTemplateLeft(availableLeft, pu, pRecY, pu.cs->m_pcBufPredCurLeft, Position(-AML_MERGE_TEMPLATE_SIZE, 0),
                          AML_MERGE_TEMPLATE_SIZE, cbHeight);

      int candIdx = 0;
      while ((pInfo->numCand < (REGULAR_AMVP_MAX_NUM_CANDS + 1)) && (candIdx < mergeCtx.numAMVPMergeCand))
      {
        pInfo->mvCand[pInfo->numCand] = mergeCtx.mvFieldNeighbours[candIdx << 1].mv;
        pInfo->mvCost[pInfo->numCand] =
          getTMCost(pu, pRecY, pInfo->mvCand[pInfo->numCand], availableTop, availableLeft, pcInter);
        pInfo->mvCand[pInfo->numCand].roundIbcPrecInternal2Amvr(pu.cu->imv);
        if (!pInfo->xCheckSimilarMotion(pInfo->numCand))
        {
          pInfo->numCand++;
        }
        candIdx++;
      }
#if !JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
      pu.cs->destroyTMBuf();
#endif
      clusterBvpCand(cbWidth, cbHeight, pInfo);
    }
  }
  else
  {
#if JVET_Z0084_IBC_TM && IBC_TM_AMVP
#if JVET_AA0070_RRIBC
    pInfo->maxSimilarityThreshold =
      (pu.cs->sps->getUseDMVDMode() && pcInter && !pu.cu->rribcFlipType) ? PU::getTMMvdThreshold(pu) : 1;
#if IBC_TM_MRG
    if (!pu.cu->rribcFlipType)
    {
      pu.tmMergeFlag = true;
    }
#endif
#else
    pInfo->maxSimilarityThreshold = (pu.cs->sps->getUseDMVDMode() && pcInter) ? PU::getTMMvdThreshold(pu) : 1;
#if IBC_TM_MRG
    pu.tmMergeFlag = true;
#endif
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    pInfo->maxSimilarityThreshold = pu.cs->sps->getUseTMIbc() ? pInfo->maxSimilarityThreshold : 1;
#endif

#if JVET_Z0075_IBC_HMVP_ENLARGE
    PU::getIBCMergeCandidates(pu, mergeCtx, pu.cs->sps->getMaxNumIBCMergeCand());
#else
    PU::getIBCMergeCandidates(pu, mergeCtx);
#endif
#if IBC_TM_MRG
    pu.tmMergeFlag = false;
#endif

    int candIdx = 0;
    while ((pInfo->numCand < AMVP_MAX_NUM_CANDS_MEM) && (candIdx < mergeCtx.numValidMergeCand))
    {
      pInfo->mvCand[pInfo->numCand] = mergeCtx.mvFieldNeighbours[candIdx << 1].mv;
      pInfo->mvCand[pInfo->numCand].roundIbcPrecInternal2Amvr(pu.cu->imv);
      if (!pInfo->xCheckSimilarMotion(pInfo->numCand))
      {
        pInfo->numCand++;
      }
      candIdx++;
    }

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    if (pu.cs->sps->getUseTMIbc())
    {
#endif
#if JVET_AA0070_RRIBC
    if (pu.cs->sps->getUseDMVDMode() && pcInter && pInfo->numCand > 0 && !pu.cu->rribcFlipType)
#else
    if (pu.cs->sps->getUseDMVDMode() && pcInter && pInfo->numCand > 0)
#endif
    {
      struct amvpSort
      {
        Mv         amvpCand;
        Distortion cost;
      };
      amvpSort              temp;
      std::vector<amvpSort> input;
      const auto            costIncSort = [](const amvpSort &x, const amvpSort &y) { return x.cost < y.cost; };
      Distortion            tmCost      = std::numeric_limits<Distortion>::max();

      for (int candIdx = 0; candIdx < pInfo->numCand; ++candIdx)
      {
        tmCost = pcInter->deriveTMMv(pu, true, std::numeric_limits<Distortion>::max(), REF_PIC_LIST_0, MAX_NUM_REF,
                                     TM_MAX_NUM_OF_ITERATIONS, pInfo->mvCand[candIdx]);
        pInfo->mvCand[candIdx].roundIbcPrecInternal2Amvr(pu.cu->imv);
        temp.amvpCand = pInfo->mvCand[candIdx];
        temp.cost     = tmCost;
        input.push_back(temp);
      }

      stable_sort(input.begin(), input.end(), costIncSort);
      for (int candIdx = 0; candIdx < pInfo->numCand; ++candIdx)
      {
        pInfo->mvCand[candIdx] = input.at(candIdx).amvpCand;
      }
    }
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
    }
#endif

    if (pInfo->numCand > AMVP_MAX_NUM_CANDS)
    {
      pInfo->numCand = AMVP_MAX_NUM_CANDS;
    }

    while (pInfo->numCand < AMVP_MAX_NUM_CANDS)
    {
      pInfo->mvCand[pInfo->numCand++] = Mv(0, 0);
    }
#else
    PU::getIBCMergeCandidates(pu, mergeCtx, AMVP_MAX_NUM_CANDS - 1);
    int candIdx = 0;
    while (pInfo->numCand < AMVP_MAX_NUM_CANDS)
    {
      pInfo->mvCand[pInfo->numCand] = mergeCtx.mvFieldNeighbours[(candIdx << 1) + 0].mv;
      pInfo->numCand++;
      candIdx++;
    }

    for (Mv &mv: pInfo->mvCand)
    {
      mv.roundIbcPrecInternal2Amvr(pu.cu->imv);
    }
#endif
  }
#else
#if JVET_Z0084_IBC_TM && IBC_TM_AMVP
#if JVET_AA0070_RRIBC
  pInfo->maxSimilarityThreshold =
    (pu.cs->sps->getUseDMVDMode() && pcInter && !pu.cu->rribcFlipType) ? PU::getTMMvdThreshold(pu) : 1;
#if IBC_TM_MRG
  if (!pu.cu->rribcFlipType)
  {
    pu.tmMergeFlag = true;
  }
#endif
#else
  pInfo->maxSimilarityThreshold = (pu.cs->sps->getUseDMVDMode() && pcInter) ? PU::getTMMvdThreshold(pu) : 1;
#if IBC_TM_MRG
  pu.tmMergeFlag = true;
#endif
#endif
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  pInfo->maxSimilarityThreshold = pu.cs->sps->getUseTMIbc() ? pInfo->maxSimilarityThreshold : 1;
#endif

#if JVET_Z0075_IBC_HMVP_ENLARGE
  PU::getIBCMergeCandidates(pu, mergeCtx, pu.cs->sps->getMaxNumIBCMergeCand());
#else
  PU::getIBCMergeCandidates(pu, mergeCtx);
#endif
#if IBC_TM_MRG
  pu.tmMergeFlag = false;
#endif

  int candIdx = 0;
  while ((pInfo->numCand < AMVP_MAX_NUM_CANDS_MEM) && (candIdx < mergeCtx.numValidMergeCand))
  {
    pInfo->mvCand[pInfo->numCand] = mergeCtx.mvFieldNeighbours[candIdx << 1].mv;
    pInfo->mvCand[pInfo->numCand].roundIbcPrecInternal2Amvr(pu.cu->imv);
    if (!pInfo->xCheckSimilarMotion(pInfo->numCand))
    {
      pInfo->numCand++;
    }
    candIdx++;
  }

#if JVET_AA0070_RRIBC
#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
  if (pu.cs->sps->getUseTMIbc() && pu.cs->sps->getUseDMVDMode() && pcInter && pInfo->numCand > 0 && !pu.cu->rribcFlipType)
#else
  if (pu.cs->sps->getUseDMVDMode() && pcInter && pInfo->numCand > 0 && !pu.cu->rribcFlipType)
#endif
#else
  if (pu.cs->sps->getUseDMVDMode() && pcInter && pInfo->numCand > 0)
#endif
  {
    struct amvpSort
    {
      Mv amvpCand;
      Distortion cost;
    };
    amvpSort temp;
    std::vector<amvpSort> input;
    const auto costIncSort = [](const amvpSort &x, const amvpSort &y) { return x.cost < y.cost; };
    Distortion tmCost = std::numeric_limits<Distortion>::max();

    for (int candIdx = 0; candIdx < pInfo->numCand; ++candIdx)
    {
      tmCost = pcInter->deriveTMMv(pu, true, std::numeric_limits<Distortion>::max(), REF_PIC_LIST_0, MAX_NUM_REF,
                                   TM_MAX_NUM_OF_ITERATIONS, pInfo->mvCand[candIdx]);
      pInfo->mvCand[candIdx].roundIbcPrecInternal2Amvr(pu.cu->imv);
      temp.amvpCand = pInfo->mvCand[candIdx];
      temp.cost = tmCost;
      input.push_back(temp);
    }

    stable_sort(input.begin(), input.end(), costIncSort);
    for (int candIdx = 0; candIdx < pInfo->numCand; ++candIdx)
    {
      pInfo->mvCand[candIdx] = input.at(candIdx).amvpCand;
    }
  }

  if (pInfo->numCand > AMVP_MAX_NUM_CANDS)
  {
    pInfo->numCand = AMVP_MAX_NUM_CANDS;
  }

  while (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
    pInfo->mvCand[pInfo->numCand++] = Mv(0, 0);
  }
#else
  PU::getIBCMergeCandidates(pu, mergeCtx, AMVP_MAX_NUM_CANDS - 1);
  int candIdx = 0;
  while (pInfo->numCand < AMVP_MAX_NUM_CANDS)
  {
    pInfo->mvCand[pInfo->numCand] = mergeCtx.mvFieldNeighbours[(candIdx << 1) + 0].mv;
    ;
    pInfo->numCand++;
    candIdx++;
  }

  for (Mv &mv: pInfo->mvCand)
  {
    mv.roundIbcPrecInternal2Amvr(pu.cu->imv);
  }
#endif
#endif
}


/** Constructs a list of candidates for AMVP (See specification, section "Derivation process for motion vector predictor candidates")
* \param uiPartIdx
* \param uiPartAddr
* \param eRefPicList
* \param iRefIdx
* \param pInfo
*/
void PU::fillMvpCand(PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AMVPInfo &amvpInfo
#if TM_AMVP || JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV
                   , InterPrediction* interPred
#endif
)
{
  CodingStructure &cs = *pu.cs;
#if TM_AMVP
#if JVET_Y0128_NON_CTC || (JVET_AA0132_CONFIGURABLE_TM_TOOLS && TM_AMVP)
  interPred = PU::checkTmEnableCondition(pu.cs->sps, pu.cs->pps, pu.cu->slice->getRefPic(eRefPicList, refIdx)) ? interPred : nullptr;
#else
  interPred = pu.cu->cs->sps->getUseDMVDMode() ? interPred : nullptr;
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER // AmvpList might be already derived during ref. reordering
  if (interPred != nullptr && interPred->readTplAmvpBuffer(amvpInfo, *pu.cu, eRefPicList, refIdx))
#else
  if (cs.pcv->isEncoder && interPred != nullptr && interPred->readTplAmvpBuffer(amvpInfo, *pu.cu, eRefPicList, refIdx))
#endif
  {
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
    if (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])
    {
      amvpInfo.numCand = 2;
      amvpInfo.mvCand[1] = amvpInfo.mvCand[0];
    }
#endif
    return;
  }
#endif
  AMVPInfo *pInfo = &amvpInfo;

  pInfo->numCand = 0;
#if TM_AMVP
  pInfo->maxStorageSize         = interPred != nullptr ? REGULAR_AMVP_MAX_NUM_CANDS : AMVP_MAX_NUM_CANDS;
  pInfo->maxSimilarityThreshold = interPred != nullptr ? 1 : 0;
#endif

  if (refIdx < 0)
  {
    return;
  }

  //-- Get Spatial MV
  Position posLT = pu.Y().topLeft();
  Position posRT = pu.Y().topRight();
  Position posLB = pu.Y().bottomLeft();

  bool bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, *pInfo );

  if( !bAdded )
  {
    bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_LEFT, *pInfo );

  }

  // Above predictor search
  bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, *pInfo );

  if( !bAdded )
  {
    bAdded = addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE, *pInfo );

    if( !bAdded )
    {
      addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, *pInfo );
    }
  }

  for( int i = 0; i < pInfo->numCand; i++ )
  {
    pInfo->mvCand[i].roundTransPrecInternal2Amvr(pu.cu->imv);
  }

  if( pInfo->numCand == 2 )
  {
    if( pInfo->mvCand[0] == pInfo->mvCand[1] )
    {
      pInfo->numCand = 1;
    }
  }

#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
#if TM_AMVP
  if (cs.picHeader->getEnableTMVPFlag() && pInfo->numCand < pInfo->maxStorageSize && (pu.lumaSize().width + pu.lumaSize().height > 8))
#else
  if (cs.picHeader->getEnableTMVPFlag() && pInfo->numCand < AMVP_MAX_NUM_CANDS && (pu.lumaSize().width + pu.lumaSize().height > 8))
#endif
  {
    MergeCtx mrgCtxAll[2];
    mrgCtxAll[0].numValidMergeCand = 0;
    mrgCtxAll[1].numValidMergeCand = 0;

    MergeCtx mrgTmvp[2];
    mrgTmvp[0].numValidMergeCand = 0;
    mrgTmvp[1].numValidMergeCand = 0;

    MergeCtx namvpMergeCandCtx[2];
    namvpMergeCandCtx[0].numValidMergeCand = 0;
    namvpMergeCandCtx[1].numValidMergeCand = 0;

    bool isSaved = true;
    if (pu.cs->sps->getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
      && pu.cs->sps->getTMToolsEnableFlag()
#endif
      && interPred != NULL)
    {
      int nWidth = pu.lumaSize().width;
      int nHeight = pu.lumaSize().height;
      bool tplAvail = interPred->xAMLGetCurBlkTemplate(pu, nWidth, nHeight);
      if (tplAvail)
      {
        int poc0 = pu.cu->slice->getRefPic(RefPicList(1 - pu.cu->slice->getColFromL0Flag()), pu.cu->slice->getColRefIdx())->getPOC();
        int poc1 = pu.cu->slice->getRefPic(RefPicList(1 - pu.cu->slice->getColFromL0Flag2nd()), pu.cu->slice->getColRefIdx2nd())->getPOC();
        for (int col = 0; col < ((pu.cu->slice->getCheckLDC() || (poc0 == poc1)) ? 1 : 2); col++)
        {
          if (col == 0 || (!pu.cs->sps->getUseFastSubTmvp()))
          {
#if JVET_Z0054_BLK_REF_PIC_REORDER
            if (!interPred->readMergeBuffer(mrgCtxAll[0], mrgCtxAll[1], *pu.cu))
#else
            if (cs.pcv->isEncoder && interPred->readMergeBuffer(mrgCtxAll[0], mrgCtxAll[1], *pu.cu))
#endif
            {
              PU::getNonAdjacentMergeCandSubTMVP(pu, namvpMergeCandCtx[col], col);
              interPred->adjustMergeCandidatesInOneCandidateGroup(pu, namvpMergeCandCtx[col], 9);
              PU::getInterMergeCandidatesSubTMVP(pu, mrgCtxAll[col], col, -1, &namvpMergeCandCtx[col]);
              isSaved = false;
            }

            PU::getTMVPCandOpt(pu, eRefPicList, refIdx, mrgTmvp[col], mrgCtxAll[col], col);
            interPred->adjustMergeCandidatesInOneCandidateGroup(pu, mrgTmvp[col], AMVP_TMVP_INDEX);
            if (mrgTmvp[col].numValidMergeCand > 0)
            {
              Mv mvTmp = mrgTmvp[col].mvFieldNeighbours[eRefPicList].mv;
              mvTmp.roundTransPrecInternal2Amvr(pu.cu->imv);
#if TM_AMVP
              pInfo->mvCand[pInfo->numCand] = mvTmp;
              if (!pInfo->xCheckSimilarMotion(pInfo->numCand))
              {
                pInfo->numCand++;
              }
#else
              pInfo->mvCand[pInfo->numCand++] = cColMv;
#endif
            }
          }
          else
          {
            // Get Temporal Motion Predictor
            const int refIdxCol = refIdx;

            Position posRB = pu.Y().bottomRight().offset(-3, -3);

            const PreCalcValues& pcv = *cs.pcv;

            Position posC0;
            bool isC0Avail = false;
            Position posC1 = pu.Y().center();
            Mv cColMv;

            bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
            const SubPic &curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
            if (curSubPic.getTreatedAsPicFlag())
            {
              boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
                (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom());
            }
            if (boundaryCond)
            {
              int posYInCtu = posRB.y & pcv.maxCUHeightMask;
              if (posYInCtu + 4 < pcv.maxCUHeight)
              {
                posC0 = posRB.offset(4, 4);
                isC0Avail = true;
              }
            }
            for (int colIdx = 1; colIdx < (pu.cu->slice->isInterB() ? 2 : 1); colIdx++)
            {
              if ((isC0Avail && getColocatedMVP(pu, eRefPicList, posC0, cColMv, refIdxCol, false, colIdx)) || getColocatedMVP(pu, eRefPicList, posC1, cColMv, refIdxCol, false, colIdx))
              {
                cColMv.roundTransPrecInternal2Amvr(pu.cu->imv);
#if TM_AMVP
                pInfo->mvCand[pInfo->numCand] = cColMv;
                if (!pInfo->xCheckSimilarMotion(pInfo->numCand))
                {
                  pInfo->numCand++;
                }
#else
                pInfo->mvCand[pInfo->numCand++] = cColMv;
#endif
              }
            }
          }
        }
      }
      else
      {
        // Get Temporal Motion Predictor
        const int refIdxCol = refIdx;

        Position posRB = pu.Y().bottomRight().offset(-3, -3);

        const PreCalcValues& pcv = *cs.pcv;

        Position posC0;
        bool isC0Avail = false;
        Position posC1 = pu.Y().center();
        Mv cColMv;

        bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
        const SubPic &curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
        if (curSubPic.getTreatedAsPicFlag())
        {
          boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
            (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom());
        }
        if (boundaryCond)
        {
          int posYInCtu = posRB.y & pcv.maxCUHeightMask;
          if (posYInCtu + 4 < pcv.maxCUHeight)
          {
            posC0 = posRB.offset(4, 4);
            isC0Avail = true;
          }
        }
        for (int colIdx = 0; colIdx < (pu.cu->slice->isInterB() ? 2 : 1); colIdx++)
        {
          if ((isC0Avail && getColocatedMVP(pu, eRefPicList, posC0, cColMv, refIdxCol, false, colIdx)) || getColocatedMVP(pu, eRefPicList, posC1, cColMv, refIdxCol, false, colIdx))
          {
            cColMv.roundTransPrecInternal2Amvr(pu.cu->imv);
#if TM_AMVP
            pInfo->mvCand[pInfo->numCand] = cColMv;
            if (!pInfo->xCheckSimilarMotion(pInfo->numCand))
            {
              pInfo->numCand++;
            }
#else
            pInfo->mvCand[pInfo->numCand++] = cColMv;
#endif
          }
        }
      }
    }
    else
    {
      // Get Temporal Motion Predictor
      const int refIdxCol = refIdx;

      Position posRB = pu.Y().bottomRight().offset(-3, -3);

      const PreCalcValues& pcv = *cs.pcv;

      Position posC0;
      bool isC0Avail = false;
      Position posC1 = pu.Y().center();
      Mv cColMv;

      bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
      const SubPic &curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
      if (curSubPic.getTreatedAsPicFlag())
      {
        boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
          (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom());
      }
      if (boundaryCond)
      {
        int posYInCtu = posRB.y & pcv.maxCUHeightMask;
        if (posYInCtu + 4 < pcv.maxCUHeight)
        {
          posC0 = posRB.offset(4, 4);
          isC0Avail = true;
        }
      }
      for (int colIdx = 0; colIdx < (pu.cu->slice->isInterB() ? 2 : 1); colIdx++)
      {
        if ((isC0Avail && getColocatedMVP(pu, eRefPicList, posC0, cColMv, refIdxCol, false, colIdx)) || getColocatedMVP(pu, eRefPicList, posC1, cColMv, refIdxCol, false, colIdx))
        {
          cColMv.roundTransPrecInternal2Amvr(pu.cu->imv);
#if TM_AMVP
          pInfo->mvCand[pInfo->numCand] = cColMv;
          if (!pInfo->xCheckSimilarMotion(pInfo->numCand))
          {
            pInfo->numCand++;
          }
#else
          pInfo->mvCand[pInfo->numCand++] = cColMv;
#endif
        }
      }
    }
    if (!isSaved)
    {
      interPred->writeMergeBuffer(mrgCtxAll[0], mrgCtxAll[1], *pu.cu);
    }
  }
#else
#if TM_AMVP
  if (cs.picHeader->getEnableTMVPFlag() && pInfo->numCand < pInfo->maxStorageSize && (pu.lumaSize().width + pu.lumaSize().height > 12))
#else
  if (cs.picHeader->getEnableTMVPFlag() && pInfo->numCand < AMVP_MAX_NUM_CANDS && (pu.lumaSize().width + pu.lumaSize().height > 12))
#endif
  {
    // Get Temporal Motion Predictor
    const int refIdxCol = refIdx;

    Position posRB = pu.Y().bottomRight().offset(-3, -3);

    const PreCalcValues& pcv = *cs.pcv;

    Position posC0;
    bool isC0Avail = false;
    Position posC1 = pu.Y().center();
    Mv cColMv;

    bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
    const SubPic &curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
    if (curSubPic.getTreatedAsPicFlag())
    {
      boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
        (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom());
    }
    if (boundaryCond)
    {
      int posYInCtu = posRB.y & pcv.maxCUHeightMask;
      if (posYInCtu + 4 < pcv.maxCUHeight)
      {
        posC0 = posRB.offset(4, 4);
        isC0Avail = true;
      }
    }
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    for (int colIdx = 0; colIdx < (pu.cu->slice->isInterB() ? 2 : 1); colIdx++)
    {
#endif
      if ((isC0Avail && getColocatedMVP(pu, eRefPicList, posC0, cColMv, refIdxCol, false
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
        , colIdx
#endif
      )) || getColocatedMVP(pu, eRefPicList, posC1, cColMv, refIdxCol, false
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
        , colIdx
#endif
      ))
      {
        cColMv.roundTransPrecInternal2Amvr(pu.cu->imv);
#if TM_AMVP
        pInfo->mvCand[pInfo->numCand] = cColMv;
        if (!pInfo->xCheckSimilarMotion(pInfo->numCand))
        {
          pInfo->numCand++;
        }
#else
        pInfo->mvCand[pInfo->numCand++] = cColMv;
#endif
      }
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    }
#endif
  }
#endif

#if TM_AMVP
  // Non-adjacent candidates
  if (pInfo->numCand < pInfo->maxStorageSize && interPred != nullptr)
  {
    const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
    MotionInfo miNeighbor;
    int offsetX = 0;
    int offsetY = 0;
    const int numNACandidate[4] = { 3, 5, 5, 5 };
    const int idxMap[4][5] = { { 0, 1, 4 },{ 0, 1, 2, 3, 4 },{ 0, 1, 2, 3, 4 },{ 0, 1, 2, 3, 4 } };

    for (int iDistanceIndex = 0; iDistanceIndex < NADISTANCE_LEVEL && pInfo->numCand < pInfo->maxStorageSize; iDistanceIndex++)
    {
      const int iNADistanceHor = pu.Y().width  * (iDistanceIndex + 1);
      const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);

      for (int iNASPIdx = 0; iNASPIdx < numNACandidate[iDistanceIndex] && pInfo->numCand < pInfo->maxStorageSize; iNASPIdx++)
      {
        switch (idxMap[iDistanceIndex][iNASPIdx])
        {
        case 0: offsetX = -iNADistanceHor - 1;               offsetY = pu.Y().height + iNADistanceVer - 1; break;
        case 1: offsetX = pu.Y().width + iNADistanceHor - 1; offsetY = -iNADistanceVer - 1;                break;
        case 2: offsetX = pu.Y().width >> 1;                 offsetY = -iNADistanceVer - 1;                break;
        case 3: offsetX = -iNADistanceHor - 1;               offsetY = pu.Y().height >> 1;                 break;
        case 4: offsetX = -iNADistanceHor - 1;               offsetY = -iNADistanceVer - 1;                break;
        default: CHECK(true, "Unknown index for non-adjacent AMVP candidate"); break;
        }

        const PredictionUnit *puNonAdjacent = cs.getPURestricted(posLT.offset(offsetX, offsetY), pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
        bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(pu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent->cu) && puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY)).isInter;
#else
        bool isAvailableNonAdjacent = puNonAdjacent && isDiffMER(pu.lumaPos(), posLT.offset(offsetX, offsetY), plevel) && CU::isInter(*puNonAdjacent->cu);
#endif

        if (isAvailableNonAdjacent)
        {
          miNeighbor = puNonAdjacent->getMotionInfo(posLT.offset(offsetX, offsetY));

          const int        currRefPOC = cs.slice->getRefPic(eRefPicList, refIdx)->getPOC();
          const RefPicList eRefPicList2nd = (eRefPicList == REF_PIC_LIST_0) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

          for (int predictorSource = 0; predictorSource < 2; predictorSource++) // examine the indicated reference picture list, then if not available, examine the other list.
          {
            const RefPicList eRefPicListIndex = (predictorSource == 0) ? eRefPicList : eRefPicList2nd;
            const int        neibRefIdx = miNeighbor.refIdx[eRefPicListIndex];

            if (neibRefIdx >= 0 && currRefPOC == cs.slice->getRefPOC(eRefPicListIndex, neibRefIdx))
            {
              miNeighbor.mv[eRefPicListIndex].roundTransPrecInternal2Amvr(pu.cu->imv);
              pInfo->mvCand[(pInfo->numCand)] = miNeighbor.mv[eRefPicListIndex];
              if (!pInfo->xCheckSimilarMotion(pInfo->numCand))
              {
                pInfo->numCand++;
              }
            }
            if (pInfo->numCand > pInfo->maxStorageSize)
            {
              pInfo->numCand = pInfo->maxStorageSize;
            }
          }
        }
      }
    }
  }

  for (int i = 0; i < pInfo->numCand; i++)
  {
    pInfo->mvCand[i].roundTransPrecInternal2Amvr(pu.cu->imv);
  }
#endif

#if TM_AMVP
  if (pInfo->numCand < pInfo->maxStorageSize)
#else
  if (pInfo->numCand < AMVP_MAX_NUM_CANDS)
#endif
  {
    const int currRefPOC = cs.slice->getRefPic(eRefPicList, refIdx)->getPOC();
    addAMVPHMVPCand(pu, eRefPicList, currRefPOC, *pInfo);
  }
#if TM_AMVP
  if (pInfo->numCand > pInfo->maxStorageSize)
  {
    pInfo->numCand = pInfo->maxStorageSize;
  }
#else
  if (pInfo->numCand > AMVP_MAX_NUM_CANDS)
  {
    pInfo->numCand = AMVP_MAX_NUM_CANDS;
  }
#endif

#if TM_AMVP
  if (
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
    pu.cu->cs->sps->getUseTMAmvpMode()
#else
    pu.cu->cs->sps->getUseDMVDMode()
#endif
    && interPred != nullptr && pInfo->numCand > 0)
  {
    struct amvpSort
    {
      Mv amvpCand;
      Distortion cost;
    };
    amvpSort temp;
    std::vector<amvpSort> input;
    const auto costIncSort = [](const amvpSort &x, const amvpSort &y) { return x.cost < y.cost; };
    Distortion tmCost[REGULAR_AMVP_MAX_NUM_CANDS];
    for (int i = 0; i < REGULAR_AMVP_MAX_NUM_CANDS; i++)
    {
      tmCost[i] = std::numeric_limits<Distortion>::max();
    }

#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
    bool armcRefinedMotion = isArmcRefinedMotionEnabled(pu, 0);
    if (armcRefinedMotion)
    {
      pu.reduceTplSize = true;
    }
    if (armcRefinedMotion && pInfo->numCand > 1)
    {
      for (int candIdx = 0; candIdx < pInfo->numCand; ++candIdx)
      {
        tmCost[candIdx] = interPred->deriveTMMv(pu, true, std::numeric_limits<Distortion>::max(), eRefPicList, refIdx, 0, pInfo->mvCand[candIdx]);
        temp.amvpCand = pInfo->mvCand[candIdx];
        temp.cost = tmCost[candIdx];
        input.push_back(temp);
      }
      stable_sort(input.begin(), input.end(), costIncSort);
      for (int candIdx = 1; candIdx < pInfo->numCand; ++candIdx)
      {
        if (input.at(candIdx).cost > 5 * input.at(0).cost)
        {
          pInfo->numCand = candIdx + 1;
          break;
        }
      }
      input.clear();
    }
#endif
    for (int candIdx = 0; candIdx < pInfo->numCand; ++candIdx)
    {
      tmCost[candIdx] = interPred->deriveTMMv(pu, true, std::numeric_limits<Distortion>::max(), eRefPicList, refIdx
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
                                            , (armcRefinedMotion ? TM_MAX_NUM_OF_ITERATIONS : 0)
#else
                                            , 0
#endif
                                            , pInfo->mvCand[candIdx]);
      temp.amvpCand = pInfo->mvCand[candIdx];
      temp.cost = tmCost[candIdx];
      input.push_back(temp);
    }

    stable_sort(input.begin(), input.end(), costIncSort);
    for (int candIdx = 0; candIdx < pInfo->numCand; ++candIdx)
    {
      pInfo->mvCand[candIdx] = input.at(candIdx).amvpCand;
      tmCost[candIdx] = input.at(candIdx).cost;
    }
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
    pu.reduceTplSize = false;
#endif

    pInfo->numCand = 1;
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
    if (!armcRefinedMotion)
    {
#endif
#if JVET_AD0213_LIC_IMP
      if (pu.cu->licFlag)
      {
        interPred->deriveTMMv(pu, true, std::numeric_limits<Distortion>::max(), eRefPicList, refIdx, TM_MAX_NUM_OF_ITERATIONS, pInfo->mvCand[0]);
      }
      else
#endif
    interPred->deriveTMMv(pu, true, tmCost[0], eRefPicList, refIdx, TM_MAX_NUM_OF_ITERATIONS, pInfo->mvCand[0]);
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
    }
#endif
  }

  while (pInfo->numCand < pInfo->maxStorageSize)
#else
  while (pInfo->numCand < AMVP_MAX_NUM_CANDS)
#endif
  {
    pInfo->mvCand[pInfo->numCand] = Mv( 0, 0 );
    pInfo->numCand++;
  }
#if TM_AMVP
  pInfo->numCand = (
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
                    pu.cu->cs->sps->getUseTMAmvpMode() 
#else
                    pu.cu->cs->sps->getUseDMVDMode() 
#endif
                 && interPred != nullptr ? 1 : pInfo->numCand);
#endif

  for (Mv &mv : pInfo->mvCand)
  {
    mv.roundTransPrecInternal2Amvr(pu.cu->imv);
  }

#if TM_AMVP
#if JVET_Z0054_BLK_REF_PIC_REORDER
  if (interPred != nullptr)
#else
  if (cs.pcv->isEncoder && interPred != nullptr)
#endif
  {
    interPred->writeTplAmvpBuffer(*pInfo, *pu.cu, eRefPicList, refIdx);
  }
#endif
#if JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE
  if (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])
  {
    if (amvpInfo.numCand == 1)
    {
      amvpInfo.numCand = 2;
      amvpInfo.mvCand[1] = amvpInfo.mvCand[0];
    }
    else
    {
      CHECK(amvpInfo.numCand != 2, "this is not possible");
      amvpInfo.numCand = 3;
      amvpInfo.mvCand[2] = amvpInfo.mvCand[1];
      amvpInfo.mvCand[1] = amvpInfo.mvCand[0];
    }
  }
#endif
}

bool PU::addAffineMVPCandUnscaled( const PredictionUnit &pu, const RefPicList &refPicList, const int &refIdx, const Position &pos, const MvpDir &dir, AffineAMVPInfo &affiAMVPInfo 
#if JVET_Z0139_HIST_AFF
  , int aiNeibeInherited[5]
#endif
#if JVET_AG0164_AFFINE_GPM
  , int checkAffGPM
#endif
)
{
  CodingStructure &cs = *pu.cs;
  const PredictionUnit *neibPU = NULL;
  Position neibPos;

  switch ( dir )
  {
  case MD_LEFT:
    neibPos = pos.offset( -1, 0 );
    break;
  case MD_ABOVE:
    neibPos = pos.offset( 0, -1 );
    break;
  case MD_ABOVE_RIGHT:
    neibPos = pos.offset( 1, -1 );
    break;
  case MD_BELOW_LEFT:
    neibPos = pos.offset( -1, 1 );
    break;
  case MD_ABOVE_LEFT:
    neibPos = pos.offset( -1, -1 );
    break;
  default:
    break;
  }

  neibPU = cs.getPURestricted( neibPos, pu, pu.chType );

#if JVET_AG0164_AFFINE_GPM
  if (neibPU == NULL || !CU::isInter(*neibPU->cu)) 
  {
    return false;
  }       
  if (!checkAffGPM && (!neibPU->cu->affine || neibPU->mergeType != MRG_TYPE_DEFAULT_N))
  {
    return false;
  }
  if (checkAffGPM && !neibPU->cu->geoFlag)
  {
    return false;
  }
#else
  if (neibPU == NULL || !CU::isInter(*neibPU->cu) || !neibPU->cu->affine || neibPU->mergeType != MRG_TYPE_DEFAULT_N)
  {
    return false;
  }
#endif

  Mv outputAffineMv[3];
  const MotionInfo& neibMi = neibPU->getMotionInfo( neibPos );

#if JVET_AG0164_AFFINE_GPM
  int gpmPartIdx = neibMi.gpmPartIdx;
  PredictionUnit tmpPU;
  const PredictionUnit* neibPUSaved = neibPU;
  if (checkAffGPM)
  {
    if (gpmPartIdx == -1)
    {
      return false;
    }
    CHECK( gpmPartIdx != 0 && gpmPartIdx != 1, "Invalid gpmPartIdx");
    if (!neibPU->affineGPM[gpmPartIdx])
    {
      return false;
    }
    tmpPU = *neibPU;
  }
#endif

  const int        currRefPOC = cs.slice->getRefPic( refPicList, refIdx )->getPOC();
  const RefPicList refPicList2nd = (refPicList == REF_PIC_LIST_0) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

  for ( int predictorSource = 0; predictorSource < 2; predictorSource++ ) // examine the indicated reference picture list, then if not available, examine the other list.
  {
    const RefPicList eRefPicListIndex = (predictorSource == 0) ? refPicList : refPicList2nd;

#if JVET_AG0164_AFFINE_GPM
    neibPU = neibPUSaved;
    int neibRefIdx = neibMi.refIdx[eRefPicListIndex];
    if (checkAffGPM)
    {
      neibRefIdx = pu.gpmPartRefIdx[gpmPartIdx][eRefPicListIndex];
      if (neibRefIdx == -1 || pu.cu->slice->getRefPOC(eRefPicListIndex, neibRefIdx) != currRefPOC)
      {
        continue;
      }
    }
    else
#else
    const int        neibRefIdx = neibMi.refIdx[eRefPicListIndex];
#endif
    if ( ((neibPU->interDir & (eRefPicListIndex + 1)) == 0) || pu.cu->slice->getRefPOC( eRefPicListIndex, neibRefIdx ) != currRefPOC )
    {
      continue;
    }
#if JVET_AG0164_AFFINE_GPM
    if (checkAffGPM)
    { 
      tmpPU.refIdx[eRefPicListIndex] = neibRefIdx;
      tmpPU.interDir = eRefPicListIndex + 1;
      tmpPU.mvAffi[eRefPicListIndex][0] = neibPU->gpmPartmvAffi[gpmPartIdx][eRefPicListIndex][0];
      tmpPU.mvAffi[eRefPicListIndex][1] = neibPU->gpmPartmvAffi[gpmPartIdx][eRefPicListIndex][1];
      tmpPU.mvAffi[eRefPicListIndex][2] = neibPU->gpmPartmvAffi[gpmPartIdx][eRefPicListIndex][2];
      tmpPU.cu->affineType = neibPU->gpmPartAffType[gpmPartIdx];
      neibPU = &tmpPU;
    }
#endif
    xInheritedAffineMv( pu, neibPU, eRefPicListIndex, outputAffineMv );
    outputAffineMv[0].roundAffinePrecInternal2Amvr(pu.cu->imv);
    outputAffineMv[1].roundAffinePrecInternal2Amvr(pu.cu->imv);
    affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[0];
    affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[1];
    if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
    {
      outputAffineMv[2].roundAffinePrecInternal2Amvr(pu.cu->imv);
      affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[2];
    }
#if JVET_Z0139_HIST_AFF
    if (!checkLastAffineAMVPCandRedundancy(pu, affiAMVPInfo))
    {
      continue;
    }
#endif
    affiAMVPInfo.numCand++;
#if JVET_Z0139_HIST_AFF
    aiNeibeInherited[dir] = 1;
#endif
    return true;
  }

  return false;
}

void PU::xInheritedAffineMv( const PredictionUnit &pu, const PredictionUnit* puNeighbour, RefPicList eRefPicList, Mv rcMv[3] )
{
  int posNeiX = puNeighbour->Y().pos().x;
  int posNeiY = puNeighbour->Y().pos().y;
  int posCurX = pu.Y().pos().x;
  int posCurY = pu.Y().pos().y;

  int neiW = puNeighbour->Y().width;
  int curW = pu.Y().width;
  int neiH = puNeighbour->Y().height;
  int curH = pu.Y().height;

  Mv mvLT, mvRT, mvLB;
  mvLT = puNeighbour->mvAffi[eRefPicList][0];
  mvRT = puNeighbour->mvAffi[eRefPicList][1];
  mvLB = puNeighbour->mvAffi[eRefPicList][2];

#if !AFFINE_RM_CONSTRAINTS_AND_OPT
  bool isTopCtuBoundary = false;
  if ( (posNeiY + neiH) % pu.cs->sps->getCTUSize() == 0 && (posNeiY + neiH) == posCurY )
  {
    // use bottom-left and bottom-right sub-block MVs for inheritance
    const Position posRB = puNeighbour->Y().bottomRight();
    const Position posLB = puNeighbour->Y().bottomLeft();
    mvLT = puNeighbour->getMotionInfo( posLB ).mv[eRefPicList];
    mvRT = puNeighbour->getMotionInfo( posRB ).mv[eRefPicList];
    posNeiY += neiH;
    isTopCtuBoundary = true;
  }
#endif

  int shift = MAX_CU_DEPTH;
  int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;

  iDMvHorX = (mvRT - mvLT).getHor() << (shift - floorLog2(neiW));
  iDMvHorY = (mvRT - mvLT).getVer() << (shift - floorLog2(neiW));
#if AFFINE_RM_CONSTRAINTS_AND_OPT
  if ( puNeighbour->cu->affineType == AFFINEMODEL_6PARAM )
#else
  if ( puNeighbour->cu->affineType == AFFINEMODEL_6PARAM && !isTopCtuBoundary )
#endif
  {
    iDMvVerX = (mvLB - mvLT).getHor() << (shift - floorLog2(neiH));
    iDMvVerY = (mvLB - mvLT).getVer() << (shift - floorLog2(neiH));
  }
  else
  {
    iDMvVerX = -iDMvHorY;
    iDMvVerY = iDMvHorX;
  }

  int iMvScaleHor = mvLT.getHor() << shift;
  int iMvScaleVer = mvLT.getVer() << shift;
  int horTmp, verTmp;

  // v0
  horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY - posNeiY);
  verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY - posNeiY);
  roundAffineMv( horTmp, verTmp, shift );
  rcMv[0].hor = horTmp;
  rcMv[0].ver = verTmp;
  rcMv[0].clipToStorageBitDepth();

  // v1
  horTmp = iMvScaleHor + iDMvHorX * (posCurX + curW - posNeiX) + iDMvVerX * (posCurY - posNeiY);
  verTmp = iMvScaleVer + iDMvHorY * (posCurX + curW - posNeiX) + iDMvVerY * (posCurY - posNeiY);
  roundAffineMv( horTmp, verTmp, shift );
  rcMv[1].hor = horTmp;
  rcMv[1].ver = verTmp;
  rcMv[1].clipToStorageBitDepth();

  // v2
#if !JVET_Z0139_HIST_AFF
  if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
#endif
  {
    horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY + curH - posNeiY);
    verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY + curH - posNeiY);
    roundAffineMv( horTmp, verTmp, shift );
    rcMv[2].hor = horTmp;
    rcMv[2].ver = verTmp;
    rcMv[2].clipToStorageBitDepth();
  }
}

#if JVET_Z0139_HIST_AFF
void PU::deriveAffineParametersFromMVs(const PredictionUnit& pu, const Mv acMvTemp[3], int* affinePara, EAffineModel affModel)
{
  Mv mvLT = acMvTemp[0];
  Mv mvRT = acMvTemp[1];
  Mv mvLB = acMvTemp[2];
  const int iBit = MAX_CU_DEPTH;
  int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;
  iDMvHorX = (mvRT - mvLT).getHor() << (iBit - g_aucLog2[pu.lwidth()]);
  iDMvHorY = (mvRT - mvLT).getVer() << (iBit - g_aucLog2[pu.lwidth()]);
  if (affModel == AFFINEMODEL_6PARAM)
  {
    iDMvVerX = (mvLB - mvLT).getHor() << (iBit - g_aucLog2[pu.lheight()]);
    iDMvVerY = (mvLB - mvLT).getVer() << (iBit - g_aucLog2[pu.lheight()]);
  }
  else
  {
    iDMvVerX = -iDMvHorY;
    iDMvVerY = iDMvHorX;
  }

  affinePara[0] = iDMvHorX;
  affinePara[1] = iDMvVerX;
  affinePara[2] = iDMvHorY;
  affinePara[3] = iDMvVerY;
}
void PU::deriveMVsFromAffineParameters(const PredictionUnit & pu, Mv rcMv[3], int* affinePara, const Mv & cBaseMv, const Position & cBasePos)
{
  int posCurX = pu.Y().pos().x;
  int posCurY = pu.Y().pos().y;

  int curW = pu.Y().width;
  int curH = pu.Y().height;


  int shift = MAX_CU_DEPTH;
  int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;

  iDMvHorX = affinePara[0];
  iDMvVerX = affinePara[1];
  iDMvHorY = affinePara[2];
  iDMvVerY = affinePara[3];

  int iMvScaleHor = cBaseMv.getHor() << shift;
  int iMvScaleVer = cBaseMv.getVer() << shift;

  int horTmp, verTmp;

  // v0
  horTmp = iMvScaleHor + iDMvHorX * (posCurX - cBasePos.x) + iDMvVerX * (posCurY - cBasePos.y);
  verTmp = iMvScaleVer + iDMvHorY * (posCurX - cBasePos.x) + iDMvVerY * (posCurY - cBasePos.y);
  roundAffineMv(horTmp, verTmp, shift);

  rcMv[0].hor = horTmp;
  rcMv[0].ver = verTmp;


  // v1
  horTmp = iMvScaleHor + iDMvHorX * (posCurX + curW - cBasePos.x) + iDMvVerX * (posCurY - cBasePos.y);
  verTmp = iMvScaleVer + iDMvHorY * (posCurX + curW - cBasePos.x) + iDMvVerY * (posCurY - cBasePos.y);
  roundAffineMv(horTmp, verTmp, shift);

  rcMv[1].hor = horTmp;
  rcMv[1].ver = verTmp;


  // v2

  horTmp = iMvScaleHor + iDMvHorX * (posCurX - cBasePos.x) + iDMvVerX * (posCurY + curH - cBasePos.y);
  verTmp = iMvScaleVer + iDMvHorY * (posCurX - cBasePos.x) + iDMvVerY * (posCurY + curH - cBasePos.y);
  roundAffineMv(horTmp, verTmp, shift);

  rcMv[2].hor = horTmp;
  rcMv[2].ver = verTmp;

}

void PU::deriveCenterMVFromAffineParameters(const PredictionUnit& pu, Mv& rcMv, int* affinePara, const Mv& cBaseMv, const Position& cBasePos)
{

  int posCenterX = pu.Y().pos().x + (pu.Y().width >> 1);
  int posCenterY = pu.Y().pos().y + (pu.Y().height >> 1);

  int shift = MAX_CU_DEPTH;
  int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;

  iDMvHorX = affinePara[0];
  iDMvVerX = affinePara[1];
  iDMvHorY = affinePara[2];
  iDMvVerY = affinePara[3];

  int iMvScaleHor = cBaseMv.getHor() << shift;
  int iMvScaleVer = cBaseMv.getVer() << shift;

  int horTmp, verTmp;

  horTmp = iMvScaleHor + iDMvHorX * (posCenterX - cBasePos.x) + iDMvVerX * (posCenterY - cBasePos.y);
  verTmp = iMvScaleVer + iDMvHorY * (posCenterX - cBasePos.x) + iDMvVerY * (posCenterY - cBasePos.y);
  roundAffineMv(horTmp, verTmp, shift);

  rcMv.hor = horTmp;
  rcMv.ver = verTmp;
}

void PU::storeAffParas(int* affinePara)
{
  const int MinV = -(1 << (AFF_PARA_STORE_BITS - 1));
  const int MaxV = (1 << (AFF_PARA_STORE_BITS - 1)) - 1;
  const int roundingAdd = (1 << AFF_PARA_SHIFT) >> 1;
  affinePara[0] = Clip3(MinV, MaxV, affinePara[0] >= 0 ? ((affinePara[0] + roundingAdd) >> AFF_PARA_SHIFT) : -((-affinePara[0] + roundingAdd) >> AFF_PARA_SHIFT));
  affinePara[1] = Clip3(MinV, MaxV, affinePara[1] >= 0 ? ((affinePara[1] + roundingAdd) >> AFF_PARA_SHIFT) : -((-affinePara[1] + roundingAdd) >> AFF_PARA_SHIFT));
  affinePara[2] = Clip3(MinV, MaxV, affinePara[2] >= 0 ? ((affinePara[2] + roundingAdd) >> AFF_PARA_SHIFT) : -((-affinePara[2] + roundingAdd) >> AFF_PARA_SHIFT));
  affinePara[3] = Clip3(MinV, MaxV, affinePara[3] >= 0 ? ((affinePara[3] + roundingAdd) >> AFF_PARA_SHIFT) : -((-affinePara[3] + roundingAdd) >> AFF_PARA_SHIFT));
}


void PU::xGetAffineMvFromLUT(AffineMotionInfo * affHistInfo, int rParameters[4])
{
  rParameters[0] = affHistInfo->oneSetAffineParameters[0] << AFF_PARA_SHIFT;
  rParameters[1] = affHistInfo->oneSetAffineParameters[1] << AFF_PARA_SHIFT;
  rParameters[2] = affHistInfo->oneSetAffineParameters[2] << AFF_PARA_SHIFT;
  rParameters[3] = affHistInfo->oneSetAffineParameters[3] << AFF_PARA_SHIFT;
}

void PU::xGetAffineMvFromLUT(short affineParameters[4], int rParameters[4])
{
  rParameters[0] = affineParameters[0] << AFF_PARA_SHIFT;
  rParameters[1] = affineParameters[1] << AFF_PARA_SHIFT;
  rParameters[2] = affineParameters[2] << AFF_PARA_SHIFT;
  rParameters[3] = affineParameters[3] << AFF_PARA_SHIFT;
}


// Add affine candidates from LUT
// return true to early terminate

bool PU::checkLastAffineMergeCandRedundancy(const PredictionUnit& pu, AffineMergeCtx& affMrgCtx)
{
  if (affMrgCtx.numValidMergeCand < 1)
  {
    return true;
  }

  const int CPMV_SIMILARITY_THREH = 1;
  const int PARA_SIMILARITY_THREH = 1;

  const int lastIdx = affMrgCtx.numValidMergeCand;
  if (affMrgCtx.mergeType[lastIdx] == MRG_TYPE_SUBPU_ATMVP)
  {
    return true;
  }

  for (int idx = 0; idx < affMrgCtx.numValidMergeCand; idx++)
  {
    if (affMrgCtx.mergeType[idx] == MRG_TYPE_SUBPU_ATMVP)
    {
      continue;
    }
    if (affMrgCtx.affineType[idx] != affMrgCtx.affineType[lastIdx])
    {
      continue;
    }
    CHECK(affMrgCtx.affineType[idx] != AFFINEMODEL_6PARAM && affMrgCtx.affineType[idx] != AFFINEMODEL_4PARAM, "Invalid parameter" );

    if ( affMrgCtx.interDirNeighbours[idx] != affMrgCtx.interDirNeighbours[lastIdx])
    {
      continue;
    }
    if (affMrgCtx.interDirNeighbours[idx] == 3 && affMrgCtx.bcwIdx[idx] != affMrgCtx.bcwIdx[lastIdx])
    {
      continue;
    }
#if INTER_LIC
#if JVET_AD0213_LIC_IMP
    if (affMrgCtx.licFlags[idx] != affMrgCtx.licFlags[lastIdx])
#else
    if (affMrgCtx.interDirNeighbours[idx] != 3 && affMrgCtx.licFlags[idx] != affMrgCtx.licFlags[lastIdx])
#endif
    {
      continue;
    }
#endif

    if ((affMrgCtx.interDirNeighbours[lastIdx] & 1) != 0)
    {
      if (affMrgCtx.mvFieldNeighbours[(idx << 1)][0].refIdx != affMrgCtx.mvFieldNeighbours[(lastIdx << 1)][0].refIdx)
      {
        continue;
      }
      Mv acMvTemp[3];
      int affinePara[4], affineParaLast[4];
      acMvTemp[0] = affMrgCtx.mvFieldNeighbours[(idx << 1)][0].mv;
      acMvTemp[1] = affMrgCtx.mvFieldNeighbours[(idx << 1)][1].mv;
      acMvTemp[2] = affMrgCtx.mvFieldNeighbours[(idx << 1)][2].mv;
      deriveAffineParametersFromMVs(pu, acMvTemp,  affinePara, affMrgCtx.affineType[idx]);
      acMvTemp[0] = affMrgCtx.mvFieldNeighbours[(lastIdx << 1)][0].mv;
      acMvTemp[1] = affMrgCtx.mvFieldNeighbours[(lastIdx << 1)][1].mv;
      acMvTemp[2] = affMrgCtx.mvFieldNeighbours[(lastIdx << 1)][2].mv;
      deriveAffineParametersFromMVs(pu, acMvTemp, affineParaLast, affMrgCtx.affineType[idx]);

      if (
        abs(affMrgCtx.mvFieldNeighbours[(idx << 1)][0].mv.getHor() - affMrgCtx.mvFieldNeighbours[(lastIdx << 1)][0].mv.getHor()) > CPMV_SIMILARITY_THREH ||      
        abs(affMrgCtx.mvFieldNeighbours[(idx << 1)][0].mv.getVer() - affMrgCtx.mvFieldNeighbours[(lastIdx << 1)][0].mv.getVer()) > CPMV_SIMILARITY_THREH ||
        abs(affinePara[0] - affineParaLast[0]) > PARA_SIMILARITY_THREH ||
        abs(affinePara[2] - affineParaLast[2]) > PARA_SIMILARITY_THREH        
        )
      {
        continue;
      }


      if (affMrgCtx.affineType[idx] == AFFINEMODEL_6PARAM)
      {
        if( abs(affinePara[1] - affineParaLast[1]) > PARA_SIMILARITY_THREH ||
            abs(affinePara[3] - affineParaLast[3]) > PARA_SIMILARITY_THREH )
        {
          continue;
        }
      }
    }

    if ((affMrgCtx.interDirNeighbours[lastIdx] & 2) != 0)
    {
      if (affMrgCtx.mvFieldNeighbours[(idx << 1) + 1][0].refIdx != affMrgCtx.mvFieldNeighbours[(lastIdx << 1) + 1][0].refIdx)
      {
        continue;
      }
      Mv acMvTemp[3];
      int affinePara[4], affineParaLast[4];
      acMvTemp[0] = affMrgCtx.mvFieldNeighbours[(idx << 1) + 1][0].mv;
      acMvTemp[1] = affMrgCtx.mvFieldNeighbours[(idx << 1) + 1][1].mv;
      acMvTemp[2] = affMrgCtx.mvFieldNeighbours[(idx << 1) + 1][2].mv;
      deriveAffineParametersFromMVs(pu, acMvTemp, affinePara, affMrgCtx.affineType[idx]);
      acMvTemp[0] = affMrgCtx.mvFieldNeighbours[(lastIdx << 1) + 1][0].mv;
      acMvTemp[1] = affMrgCtx.mvFieldNeighbours[(lastIdx << 1) + 1][1].mv;
      acMvTemp[2] = affMrgCtx.mvFieldNeighbours[(lastIdx << 1) + 1][2].mv;
      deriveAffineParametersFromMVs(pu, acMvTemp, affineParaLast, affMrgCtx.affineType[idx]);

      if (
        abs(affMrgCtx.mvFieldNeighbours[(idx << 1) + 1][0].mv.getHor() - affMrgCtx.mvFieldNeighbours[(lastIdx << 1) + 1][0].mv.getHor()) > CPMV_SIMILARITY_THREH ||
        abs(affMrgCtx.mvFieldNeighbours[(idx << 1) + 1][0].mv.getVer() - affMrgCtx.mvFieldNeighbours[(lastIdx << 1) + 1][0].mv.getVer()) > CPMV_SIMILARITY_THREH ||
        abs(affinePara[0] - affineParaLast[0]) > PARA_SIMILARITY_THREH ||
        abs(affinePara[2] - affineParaLast[2]) > PARA_SIMILARITY_THREH)
      {
        continue;
      }

      if (affMrgCtx.affineType[idx] == AFFINEMODEL_6PARAM)
      {
        if (abs(affinePara[1] - affineParaLast[1]) > PARA_SIMILARITY_THREH ||
            abs(affinePara[3] - affineParaLast[3]) > PARA_SIMILARITY_THREH)
        {
          continue;
        }
      }
    }

    for (int mvNum = 0; mvNum < 3; mvNum++)
    {
      affMrgCtx.mvFieldNeighbours[(lastIdx << 1) + 0][mvNum].setMvField(Mv(), -1);
      affMrgCtx.mvFieldNeighbours[(lastIdx << 1) + 1][mvNum].setMvField(Mv(), -1);
    }
    affMrgCtx.interDirNeighbours[lastIdx] = 0;
    affMrgCtx.affineType[lastIdx] = AFFINEMODEL_4PARAM;
    affMrgCtx.mergeType[lastIdx] = MRG_TYPE_DEFAULT_N;
    affMrgCtx.bcwIdx[lastIdx] = BCW_DEFAULT;
#if INTER_LIC
    affMrgCtx.licFlags[lastIdx] = false;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
    affMrgCtx.obmcFlags[lastIdx] = true;
#endif
    return false;
  }
  return true;
}

bool PU::addMergeHMVPCandFromAffModel(const PredictionUnit& pu, MergeCtx& mrgCtx, const int& mrgCandIdx, int& cnt
#if TM_MRG || JVET_AE0046_BI_GPM
  , const uint32_t mvdSimilarityThresh
#endif
)
{
  if ((pu.cu->slice->getPicHeader()->getMvdL1ZeroFlag() || pu.cu->slice->getNumRefIdx(REF_PIC_LIST_1) ==0 )
    && abs(pu.cu->slice->getRefPOC( REF_PIC_LIST_0, 0) - pu.cu->slice->getPOC()) == 1)
  {
    return false;
  }

#if JVET_Z0118_GDR
  bool isClean = pu.cs->isClean(pu.cu->Y().bottomRight(), CHANNEL_TYPE_LUMA);
#endif

  const PredictionUnit* npuGroup1[5];
  const PredictionUnit* npuGroup2[5];
  Position posGroup1[5];
  Position posGroup2[5];
  int numGroup1, numGroup2;
  numGroup1 = numGroup2 = 0;

  const int CHECKED_NEI_NUM = 5;

  const Position posLB = pu.Y().bottomLeft();
  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();

  Position neiPositions[5] = { posLB.offset(-2, -1), posRT.offset(-1, -2), posRT.offset(3, -2), posLB.offset(-2, 3), posLT.offset(-2, -2) };

  int iTotalAffHMVPCandNum = 0;
  const int MAX_ALLOWED_AFF_HMVP_CAND = 1;

#if JVET_X0083_BM_AMVP_MERGE_MODE && !JVET_Y0128_NON_CTC
  RefPicList mergeRefList = REF_PIC_LIST_X;
  RefPicList amvpRefList = REF_PIC_LIST_X;
  int amvpPoc = -1;
  if (pu.amvpMergeModeFlag[0] || pu.amvpMergeModeFlag[1])
  {
    mergeRefList = pu.amvpMergeModeFlag[0] ? REF_PIC_LIST_0 : REF_PIC_LIST_1;
    amvpRefList = RefPicList(1 - mergeRefList);
    amvpPoc = pu.cu->slice->getRefPOC(amvpRefList, pu.refIdx[amvpRefList]);
  }
#endif
  for (int nei = 0; nei < CHECKED_NEI_NUM; nei++)
  {
    const PredictionUnit* puNei = pu.cs->getPURestricted(neiPositions[nei], pu, pu.chType);
    if (!puNei)
    {
      continue;
    }
    if (puNei->cu->predMode != MODE_INTER)
    {
      continue;
    }
    MotionInfo mvInfo = puNei->getMotionInfo(neiPositions[nei]);
    if (!mvInfo.isInter || mvInfo.interDir <= 0 || mvInfo.interDir > 3)
    {
      continue;
    }
    if (puNei->cu->affine && !(puNei->mergeFlag && puNei->mergeType != MRG_TYPE_DEFAULT_N))
    {
      posGroup1[numGroup1] = neiPositions[nei];
      npuGroup1[numGroup1++] = puNei;
    }
    else
    {
      posGroup2[numGroup2] = neiPositions[nei];
      npuGroup2[numGroup2++] = puNei;
    }
  }
  for (int iAffListIdx = 0; iAffListIdx < MAX_NUM_AFF_HMVP_CANDS && iTotalAffHMVPCandNum < MAX_ALLOWED_AFF_HMVP_CAND; iAffListIdx++)
  {
    for (int i = 0; i < numGroup1 && iTotalAffHMVPCandNum < MAX_ALLOWED_AFF_HMVP_CAND; i++)
    {
      const PredictionUnit* puNei = npuGroup1[i];
      MotionInfo mvInfo = puNei->getMotionInfo(posGroup1[i]);

#if JVET_Z0118_GDR
      if (addOneMergeHMVPCandFromAffModel(pu, mrgCtx, cnt, (isClean) ? pu.cs->motionLut.lutAff1 : pu.cs->motionLut.lutAff0, iAffListIdx, mvInfo, posGroup1[i], mvInfo.interDir == 3 ? puNei->cu->bcwIdx : BCW_DEFAULT
#else
      if (addOneMergeHMVPCandFromAffModel(pu, mrgCtx, cnt, pu.cs->motionLut.lutAff, iAffListIdx, mvInfo, posGroup1[i], mvInfo.interDir == 3 ? puNei->cu->bcwIdx : BCW_DEFAULT
#endif
#if INTER_LIC
#if JVET_AD0213_LIC_IMP
#if JVET_AG0276_NLIC
        , puNei->cu->licFlag || (puNei->cu->altLMFlag && puNei->cu->slice->getUseLIC())
#else
        , puNei->cu->licFlag
#endif
#else
        , mvInfo.interDir != 3 ? puNei->cu->licFlag : false
#endif
#endif
#if TM_MRG || JVET_AE0046_BI_GPM
        , mvdSimilarityThresh
#endif
      ))
      {

#if JVET_X0083_BM_AMVP_MERGE_MODE
        int8_t candRefIdx[2];
        candRefIdx[0] = mrgCtx.mvFieldNeighbours[(cnt << 1) + 0].refIdx;
        candRefIdx[1] = mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].refIdx;
#if JVET_Y0128_NON_CTC
        bool isValidAmMode = checkIsValidMergeMvCand(pu, candRefIdx);
#else
        bool isValidAmMode = checkIsValidMergeMvCand(*pu.cs, pu, pu.cu->slice->getPOC(), amvpPoc, candRefIdx);
#endif
        if (!isValidAmMode)
        {
          mrgCtx.interDirNeighbours[cnt] = 0;
          mrgCtx.mvFieldNeighbours[(cnt << 1) + 0].setMvField(Mv(), -1);
          mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(Mv(), -1);
          mrgCtx.bcwIdx[cnt] = BCW_DEFAULT;
#if INTER_LIC
          mrgCtx.licFlags[cnt] = false;
#endif
#if MULTI_HYP_PRED && JVET_AD0213_LIC_IMP
          mrgCtx.addHypNeighbours[cnt].clear();
#endif
          mrgCtx.useAltHpelIf[cnt] = false;
          continue;
        }
#if JVET_AD0213_LIC_IMP
        if ((pu.amvpMergeModeFlag[REF_PIC_LIST_0] || pu.amvpMergeModeFlag[REF_PIC_LIST_1]) && isValidAmMode)
        {
          int amvpMergeCtxMergeDir = ((pu.amvpMergeModeFlag[0] == true) ? 1 : 2);
          RefPicList amvpRefList = ((pu.amvpMergeModeFlag[0] == true) ? REF_PIC_LIST_1 : REF_PIC_LIST_0);;
          mrgCtx.interDirNeighbours[cnt] = amvpMergeCtxMergeDir;
          mrgCtx.mvFieldNeighbours[(cnt << 1) + amvpRefList].setMvField(Mv(), -1);
        }
#endif
#endif
        if (cnt == mrgCandIdx)
        {
#if TM_MRG
          if (!pu.tmMergeFlag)
#endif
            return true;
        }
        cnt++;
        if (cnt == mrgCtx.numValidMergeCand)
        {
          return true;
        }
        iTotalAffHMVPCandNum++;
      }
    }
    for (int i = 0; i < numGroup2 && iTotalAffHMVPCandNum < MAX_ALLOWED_AFF_HMVP_CAND; i++)
    {
      const PredictionUnit* puNei = npuGroup2[i];
      MotionInfo mvInfo = puNei->getMotionInfo(posGroup2[i]);

#if JVET_Z0118_GDR
      if (addOneMergeHMVPCandFromAffModel(pu, mrgCtx, cnt, (isClean) ? pu.cs->motionLut.lutAff1 : pu.cs->motionLut.lutAff0, iAffListIdx, mvInfo, posGroup2[i], mvInfo.interDir == 3 ? puNei->cu->bcwIdx : BCW_DEFAULT
#else
      if (addOneMergeHMVPCandFromAffModel(pu, mrgCtx, cnt, pu.cs->motionLut.lutAff, iAffListIdx, mvInfo, posGroup2[i], mvInfo.interDir == 3 ? puNei->cu->bcwIdx : BCW_DEFAULT
#endif
#if INTER_LIC
#if JVET_AD0213_LIC_IMP
#if JVET_AG0276_NLIC
        , puNei->cu->licFlag || (puNei->cu->altLMFlag && puNei->cu->slice->getUseLIC())
#else
        , puNei->cu->licFlag
#endif
#else
        , mvInfo.interDir != 3 ? puNei->cu->licFlag : false
#endif
#endif
#if TM_MRG || JVET_AE0046_BI_GPM
        , mvdSimilarityThresh
#endif
      ))
      {

#if JVET_X0083_BM_AMVP_MERGE_MODE
        int8_t candRefIdx[2];
        candRefIdx[0] = mrgCtx.mvFieldNeighbours[(cnt << 1) + 0].refIdx;
        candRefIdx[1] = mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].refIdx;
#if JVET_Y0128_NON_CTC
        bool isValidAmMode = checkIsValidMergeMvCand(pu, candRefIdx);
#else
        bool isValidAmMode = checkIsValidMergeMvCand(*pu.cs, pu, pu.cu->slice->getPOC(), amvpPoc, candRefIdx);
#endif
        if (!isValidAmMode)
        {
          mrgCtx.interDirNeighbours[cnt] = 0;
          mrgCtx.mvFieldNeighbours[(cnt << 1) + 0].setMvField(Mv(), -1);
          mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(Mv(), -1);
          mrgCtx.bcwIdx[cnt] = BCW_DEFAULT;
#if INTER_LIC
          mrgCtx.licFlags[cnt] = false;
#endif
#if MULTI_HYP_PRED && JVET_AD0213_LIC_IMP
          mrgCtx.addHypNeighbours[cnt].clear();
#endif
          mrgCtx.useAltHpelIf[cnt] = false;
          continue;
        }
#if JVET_AD0213_LIC_IMP
        if ((pu.amvpMergeModeFlag[REF_PIC_LIST_0] || pu.amvpMergeModeFlag[REF_PIC_LIST_1]) && isValidAmMode)
        {
          int amvpMergeCtxMergeDir = ((pu.amvpMergeModeFlag[0] == true) ? 1 : 2);
          RefPicList amvpRefList = ((pu.amvpMergeModeFlag[0] == true) ? REF_PIC_LIST_1 : REF_PIC_LIST_0);;
          mrgCtx.interDirNeighbours[cnt] = amvpMergeCtxMergeDir;
          mrgCtx.mvFieldNeighbours[(cnt << 1) + amvpRefList].setMvField(Mv(), -1);
        }
#endif
#endif
        if (cnt == mrgCandIdx)
        {
#if TM_MRG
          if (!pu.tmMergeFlag)
#endif
          {
            return true;
          }
        }
        cnt++;
        if (cnt == mrgCtx.numValidMergeCand)
        {
            return true;
        }
        iTotalAffHMVPCandNum++;
      }
    }
  }
  CHECK(iTotalAffHMVPCandNum > MAX_ALLOWED_AFF_HMVP_CAND, "Invalid number of aff-HMVP based merge candidates");
  return false;
}

bool PU::addOneMergeHMVPCandFromAffModel(const PredictionUnit& pu, MergeCtx& mrgCtx, int& cnt, static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS> *lutAff, int listIdx, const MotionInfo& mvInfo, Position neiPosition
  , int iGBiIdx 
#if INTER_LIC
  ,bool bICflag
#endif
#if TM_MRG || JVET_AE0046_BI_GPM
  , const uint32_t mvdSimilarityThresh
#endif
)
{
  Mv cMv[2];
  int HistParameters[2][4];
  mrgCtx.interDirNeighbours[cnt] = 0;
  mrgCtx.bcwIdx[cnt] = BCW_DEFAULT;
#if INTER_LIC
  mrgCtx.licFlags[cnt] = false;
#endif
  mrgCtx.useAltHpelIf[cnt] = false;
  mrgCtx.mvFieldNeighbours[(cnt << 1) + 0].setMvField(Mv(), -1);
  mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(Mv(), -1);

  if ((mvInfo.interDir & 1) != 0)
  {
    CHECK(mvInfo.refIdx[0] == -1, "invalid Refidx");
    int idxInLUT = std::min((int)mvInfo.refIdx[0], MAX_NUM_AFFHMVP_ENTRIES_ONELIST - 1);
    int lutSize = (int) lutAff[idxInLUT].size();
    if (listIdx >= lutSize || lutAff[idxInLUT][lutSize - 1 - listIdx].oneSetAffineParametersPattern == 0)
    {
      mrgCtx.mvFieldNeighbours[(cnt << 1) + 0].setMvField(Mv(), -1);
    }
    else
    {
      AffineMotionInfo& affHistInfo = lutAff[idxInLUT][lutSize - 1 - listIdx];
      mrgCtx.interDirNeighbours[cnt] |= 1;
      xGetAffineMvFromLUT(&affHistInfo, HistParameters[0]);
      deriveCenterMVFromAffineParameters(pu, cMv[0], HistParameters[0], mvInfo.mv[0], neiPosition);
      mrgCtx.mvFieldNeighbours[(cnt << 1) + 0].setMvField(cMv[0], mvInfo.refIdx[0]);
    }
  }
  if ((mvInfo.interDir & 2) != 0)
  {
    CHECK(mvInfo.refIdx[1] == -1, "invalid Refidx");
    int idxInLUT = MAX_NUM_AFFHMVP_ENTRIES_ONELIST + std::min((int)mvInfo.refIdx[1], MAX_NUM_AFFHMVP_ENTRIES_ONELIST - 1);
    int lutSize = (int)lutAff[idxInLUT].size();
    if (listIdx >= lutSize || lutAff[idxInLUT][lutSize - 1 - listIdx].oneSetAffineParametersPattern == 0)
    {
      mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(Mv(), -1);
    }
    else
    {
      AffineMotionInfo& affHistInfo = lutAff[idxInLUT][lutSize - 1 - listIdx];
      mrgCtx.interDirNeighbours[cnt] |= 2;
      xGetAffineMvFromLUT(&affHistInfo, HistParameters[1]);
      deriveCenterMVFromAffineParameters(pu, cMv[1], HistParameters[1], mvInfo.mv[1], neiPosition);
      mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(cMv[1], mvInfo.refIdx[1]);

    }
  }
  if (mrgCtx.interDirNeighbours[cnt] == 0)
  {
    mrgCtx.bcwIdx  [cnt] = BCW_DEFAULT;
#if INTER_LIC
    mrgCtx.licFlags[cnt] = false;
#endif
#if MULTI_HYP_PRED && JVET_AD0213_LIC_IMP
    mrgCtx.addHypNeighbours[cnt].clear();
#endif
    mrgCtx.useAltHpelIf[cnt] = false;
    mrgCtx.mvFieldNeighbours[(cnt << 1) + 0].setMvField(Mv(), -1);
    mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(Mv(), -1);
    return false;
  }
  mrgCtx.bcwIdx[cnt] = mrgCtx.interDirNeighbours[cnt] == 3 ? iGBiIdx : BCW_DEFAULT;

#if INTER_LIC
  mrgCtx.licFlags[cnt] = bICflag;
#if !JVET_AD0213_LIC_IMP
  CHECK(bICflag && mvInfo.interDir == 3, "LIC cannot be used for Bi");
#endif
#endif

#if NON_ADJACENT_MRG_CAND
  if (mrgCtx.xCheckSimilarMotion(cnt
#if TM_MRG || JVET_AE0046_BI_GPM
    , mvdSimilarityThresh
#endif
  ))
  {
    mrgCtx.interDirNeighbours[cnt] = 0;
    mrgCtx.bcwIdx[cnt] = BCW_DEFAULT;
#if INTER_LIC
    mrgCtx.licFlags[cnt] = false;
#endif
#if MULTI_HYP_PRED && JVET_AD0213_LIC_IMP
    mrgCtx.addHypNeighbours[cnt].clear();
#endif
    mrgCtx.useAltHpelIf[cnt] = false;
    mrgCtx.mvFieldNeighbours[(cnt << 1) + 0].setMvField(Mv(), -1);
    mrgCtx.mvFieldNeighbours[(cnt << 1) + 1].setMvField(Mv(), -1);
    return false;
  }
#endif
  return true;
}

bool PU::addOneAffineMergeHMVPCand(const PredictionUnit & pu, AffineMergeCtx & affMrgCtx, static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS>* lutAff, int listIdx, const MotionInfo & mvInfo, Position neiPosition, int iGBiIdx
#if INTER_LIC
                                 ,       bool bICflag
#endif
)
{
  Mv cMv[2][3];
  int aiHistParameters[2][4];
  affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = 0;
  if ((mvInfo.interDir & 1) != 0)
  {
    CHECK(mvInfo.refIdx[0] == -1, "invalid Refidx");
    int idxInLUT = std::min((int)mvInfo.refIdx[0], MAX_NUM_AFFHMVP_ENTRIES_ONELIST - 1);
    int lutSize = (int) lutAff[idxInLUT].size();
    if (listIdx >= lutSize || lutAff[idxInLUT][lutSize - 1 - listIdx].oneSetAffineParametersPattern == 0)
    {
      affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][0].setMvField(Mv(), -1);
      affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][1].setMvField(Mv(), -1);
      affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][2].setMvField(Mv(), -1);
    }
    else
    {
      AffineMotionInfo & affHistInfo = lutAff[idxInLUT][lutSize - 1 - listIdx];
      affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] |= 1;
      xGetAffineMvFromLUT(&affHistInfo, aiHistParameters[0]);
      deriveMVsFromAffineParameters(pu, cMv[0], aiHistParameters[0], mvInfo.mv[0], neiPosition);
      for (int mvNum = 0; mvNum < 3; mvNum++)
      {
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][mvNum].setMvField(cMv[0][mvNum], mvInfo.refIdx[0]);
      }
    }
  }
  if ((mvInfo.interDir & 2) != 0)
  {
    CHECK(mvInfo.refIdx[1] == -1, "invalid Refidx");
    int idxInLUT = MAX_NUM_AFFHMVP_ENTRIES_ONELIST + std::min((int)mvInfo.refIdx[1], MAX_NUM_AFFHMVP_ENTRIES_ONELIST - 1);
    int lutSize = (int) lutAff[idxInLUT].size();
    if (listIdx >= lutSize || lutAff[idxInLUT][lutSize - 1 - listIdx].oneSetAffineParametersPattern == 0)
    {
      affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][0].setMvField(Mv(), -1);
      affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][1].setMvField(Mv(), -1);
      affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][2].setMvField(Mv(), -1);
    }
    else
    {
      AffineMotionInfo& affHistInfo = lutAff[idxInLUT][lutSize - 1 - listIdx];
      affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] |= 2;
      xGetAffineMvFromLUT(&affHistInfo, aiHistParameters[1]);
      deriveMVsFromAffineParameters(pu, cMv[1], aiHistParameters[1], mvInfo.mv[1], neiPosition);
      for (int mvNum = 0; mvNum < 3; mvNum++)
      {
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][mvNum].setMvField(cMv[1][mvNum], mvInfo.refIdx[1]);
      }
    }
  }
  if (affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] == 0)
  {
    return false;
  }
  affMrgCtx.affineType[affMrgCtx.numValidMergeCand] = AFFINEMODEL_6PARAM;

  affMrgCtx.bcwIdx[affMrgCtx.numValidMergeCand] = (affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] == 3) ? iGBiIdx : BCW_DEFAULT;

#if INTER_LIC
  affMrgCtx.licFlags[affMrgCtx.numValidMergeCand] = bICflag;
#if !JVET_AD0213_LIC_IMP
  CHECK( bICflag && mvInfo.interDir == 3, "LIC cannot be used for Bi");
#endif
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
#if JVET_AD0213_LIC_IMP
  affMrgCtx.obmcFlags[affMrgCtx.numValidMergeCand] = true;
#else
  affMrgCtx.obmcFlags[affMrgCtx.numValidMergeCand] = !bICflag;
#endif
#endif

  if (checkLastAffineMergeCandRedundancy(pu, affMrgCtx))
  {
    return true;
  }
  return false;
}



bool PU::addOneInheritedHMVPAffineMergeCand(const PredictionUnit& pu, AffineMergeCtx& affMrgCtx, static_vector<AffineInheritInfo, MAX_NUM_AFF_INHERIT_HMVP_CANDS>& lutAffInherit, int listIdx)
{
  Mv cMv[2][3];
  int aiHistParameters[2][4];

  int lutSize = (int)lutAffInherit.size();

  if (listIdx >= lutSize)
  {
    return false;
  }
  AffineInheritInfo& affHistInfo = lutAffInherit[lutSize - 1 - listIdx];

  affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = 0;

  affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][0].setMvField(Mv(), -1);
  affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][1].setMvField(Mv(), -1);
  affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][2].setMvField(Mv(), -1);
  affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][0].setMvField(Mv(), -1);
  affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][1].setMvField(Mv(), -1);
  affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][2].setMvField(Mv(), -1);

  if (affHistInfo.oneSetAffineParametersPattern0 != 0 && affHistInfo.baseMV[0].refIdx != -1)
  {
    affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] |= 1;
    xGetAffineMvFromLUT(affHistInfo.oneSetAffineParameters0, aiHistParameters[0]);
    deriveMVsFromAffineParameters(pu, cMv[0], aiHistParameters[0], affHistInfo.baseMV[0].mv, affHistInfo.basePos);
    for (int mvNum = 0; mvNum < 3; mvNum++)
    {
      affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][mvNum].setMvField(cMv[0][mvNum], affHistInfo.baseMV[0].refIdx);
    }
  }

  if (affHistInfo.oneSetAffineParametersPattern1 != 0 && affHistInfo.baseMV[1].refIdx != -1)
  {
    affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] |= 2;
    xGetAffineMvFromLUT(affHistInfo.oneSetAffineParameters1, aiHistParameters[1]);
    deriveMVsFromAffineParameters(pu, cMv[1], aiHistParameters[1], affHistInfo.baseMV[1].mv, affHistInfo.basePos);
    for (int mvNum = 0; mvNum < 3; mvNum++)
    {
      affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][mvNum].setMvField(cMv[1][mvNum], affHistInfo.baseMV[1].refIdx);
    }
  }

  if (affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] == 0)
  {
    return false;
  }
  affMrgCtx.affineType[affMrgCtx.numValidMergeCand] = AFFINEMODEL_6PARAM;

  affMrgCtx.bcwIdx[affMrgCtx.numValidMergeCand] = BCW_DEFAULT;

#if INTER_LIC
  affMrgCtx.licFlags[affMrgCtx.numValidMergeCand] = false;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
  affMrgCtx.obmcFlags[affMrgCtx.numValidMergeCand] = true;
#endif

  if (checkLastAffineMergeCandRedundancy(pu, affMrgCtx))
  {
    return true;
  }
  return false;
}

bool PU::addSpatialAffineMergeHMVPCand(const PredictionUnit& pu, AffineMergeCtx& affMrgCtx, static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS>* lutAff, int affHMVPIdx, const PredictionUnit* neiPUs[], Position neiPositions[], int iNeiNum, const int mrgCandIdx)
{
  const Slice& slice = *pu.cs->slice;
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
#if JVET_W0090_ARMC_TM
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
  const uint32_t maxNumAffineMergeCand = slice.getPicHeader()->getMaxNumAffineMergeCand() + ((pu.cs->sps->getUseAML()&& pu.cs->sps->getTMToolsEnableFlag()) ? ADDITIONAL_AFFINE_CAND_NUM : 0);
#else
  const uint32_t maxNumAffineMergeCand = slice.getPicHeader()->getMaxNumAffineMergeCand() + (pu.cs->sps->getUseAML() ? ADDITIONAL_AFFINE_CAND_NUM : 0);
#endif
#else
  const uint32_t maxNumAffineMergeCand = slice.getPicHeader()->getMaxNumAffineMergeCand();
#endif
#else
  const uint32_t maxNumAffineMergeCand = slice.getPicHeader()->getMaxNumAffineMergeCand();
#endif

  for (int nei = 0; nei < iNeiNum; nei++)
  {
    const PredictionUnit* puNei = neiPUs[nei];
    CHECK(!neiPUs[nei], "Invalid neighbour PU");
    MotionInfo mvInfo = puNei->getMotionInfo(neiPositions[nei]);
    if (mvInfo.isIBCmot)
    {
      continue;
    }
    if (addOneAffineMergeHMVPCand(pu, affMrgCtx, lutAff, affHMVPIdx, mvInfo, neiPositions[nei], puNei->interDir == 3? puNei->cu->bcwIdx : BCW_DEFAULT
#if INTER_LIC
                                , puNei->cu->licFlag
#endif
    ))
    {
      if (affMrgCtx.numValidMergeCand == mrgCandIdx) // for decoder 
      {
        affMrgCtx.numValidMergeCand++;
        return true;
      }

      affMrgCtx.numValidMergeCand++;

      // early termination
      if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
      {
        return true;
      }
    }
  } //for nei
  return false;
}

bool PU::addSpatialAffineAMVPHMVPCand(PredictionUnit & pu, const RefPicList & eRefPicList, const int& refIdx, AffineAMVPInfo & affiAMVPInfo, static_vector<AffineMotionInfo, MAX_NUM_AFF_HMVP_CANDS>* lutAff, int iHMVPlistIdx,
  int neiIdx[], int iNeiNum, int aiNeibeInherited[], bool bFoundOne)
{
  if (affiAMVPInfo.numCand >= AMVP_MAX_NUM_CANDS)
  {
    return false;
  }

  const Slice& slice = *pu.cs->slice;
  const Position posLB = pu.Y().bottomLeft();
  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();

  Position neiPositions[5] = { posLB.offset(-2, -1), posRT.offset(-1, -2), posRT.offset(3, -2), posLB.offset(-2, 3), posLT.offset(-2, -2) };


  const int        currRefPOC = pu.cs->slice->getRefPic(eRefPicList, refIdx)->getPOC();

  AffineMotionInfo affHistInfo;
  int idxInLUT = (int)eRefPicList * MAX_NUM_AFFHMVP_ENTRIES_ONELIST + std::min(refIdx, MAX_NUM_AFFHMVP_ENTRIES_ONELIST - 1);
  if (iHMVPlistIdx >= lutAff[idxInLUT].size())
  {
    return false;
  }
  affHistInfo = lutAff[idxInLUT][lutAff[idxInLUT].size() - 1 - iHMVPlistIdx];
  if (affHistInfo.oneSetAffineParametersPattern == 0)
  {
    return false;
  }

  Mv cMv[3];
  int aiHistParameters[4];

  for (int idx = 0; idx < iNeiNum; idx++)
  {
    int nei = neiIdx[idx];
    if (aiNeibeInherited[nei])
    {
      continue;
    }
    aiNeibeInherited[nei] = 1;
    const PredictionUnit* puNei = pu.cs->getPURestricted(neiPositions[nei], pu, pu.chType);
    if (!puNei || !CU::isInter(*pu.cu))
    {
      continue;
    }

    MotionInfo mvInfo = puNei->getMotionInfo(neiPositions[nei]);
    if (mvInfo.isIBCmot)
    {
      continue;
    }
    if (!mvInfo.isInter || mvInfo.interDir <= 0 || mvInfo.interDir > 3)
    {
      continue;
    }
    RefPicList selRefPicListIndex = eRefPicList;
    if (((mvInfo.interDir & (selRefPicListIndex + 1)) == 0) || ((slice.getRefPic(selRefPicListIndex, mvInfo.refIdx[selRefPicListIndex])->getPOC()) != currRefPOC))
    {
      selRefPicListIndex = RefPicList(1 - eRefPicList);
      if (((mvInfo.interDir & (selRefPicListIndex + 1)) == 0) || ((slice.getRefPic(selRefPicListIndex, mvInfo.refIdx[selRefPicListIndex])->getPOC()) != currRefPOC))
      {
        continue;
      }
    }

    xGetAffineMvFromLUT(&affHistInfo, aiHistParameters);
    deriveMVsFromAffineParameters(pu, cMv, aiHistParameters, mvInfo.mv[selRefPicListIndex], neiPositions[nei]);

    if (pu.cu->imv == 0)
    {
      cMv[0].roundToPrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
      cMv[1].roundToPrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
      cMv[2].roundToPrecision(MV_PRECISION_INTERNAL, MV_PRECISION_QUARTER);
    }
    else if (pu.cu->imv == 2)
    {
      cMv[0].roundToPrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
      cMv[1].roundToPrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
      cMv[2].roundToPrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
    }

    affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = cMv[0];
    affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = cMv[1];
    affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = cMv[2];


    if (!checkLastAffineAMVPCandRedundancy(pu, affiAMVPInfo))
    {
      continue;
    }

    affiAMVPInfo.numCand++;

    if (affiAMVPInfo.numCand == AMVP_MAX_NUM_CANDS)
    {
      return true;
    }
    if (bFoundOne)
    {
      return false;
    }
  }

  return false;
}

#endif
#if JVET_Z0139_HIST_AFF || JVET_Z0139_NA_AFF
bool PU::checkLastAffineAMVPCandRedundancy(const PredictionUnit& pu, AffineAMVPInfo& affiAMVPInfo)
{
  if (affiAMVPInfo.numCand < 1)
  {
    return true;
  }

  const int CPMV_SIMILARITY_THREH = 0;
  const int lastIdx = affiAMVPInfo.numCand;

  for (int idx = 0; idx < affiAMVPInfo.numCand; idx++)
  {

    if (
      abs(affiAMVPInfo.mvCandLT[idx].getHor() - affiAMVPInfo.mvCandLT[lastIdx].getHor()) > CPMV_SIMILARITY_THREH ||
      abs(affiAMVPInfo.mvCandLT[idx].getVer() - affiAMVPInfo.mvCandLT[lastIdx].getVer()) > CPMV_SIMILARITY_THREH ||
      abs(affiAMVPInfo.mvCandRT[idx].getHor() - affiAMVPInfo.mvCandRT[lastIdx].getHor()) > CPMV_SIMILARITY_THREH ||
      abs(affiAMVPInfo.mvCandRT[idx].getVer() - affiAMVPInfo.mvCandRT[lastIdx].getVer()) > CPMV_SIMILARITY_THREH)
    {
      continue;
    }
    if (pu.cu->affineType == AFFINEMODEL_6PARAM)
    {
      if (
        abs(affiAMVPInfo.mvCandLB[idx].getHor() - affiAMVPInfo.mvCandLB[lastIdx].getHor()) > CPMV_SIMILARITY_THREH ||
        abs(affiAMVPInfo.mvCandLB[idx].getVer() - affiAMVPInfo.mvCandLB[lastIdx].getVer()) > CPMV_SIMILARITY_THREH)
      {
        continue;
      }
    }
    return false;
  }
  return true;
}
#endif
void PU::fillAffineMvpCand(PredictionUnit &pu, const RefPicList &eRefPicList, const int &refIdx, AffineAMVPInfo &affiAMVPInfo)
{
  affiAMVPInfo.numCand = 0;

  if (refIdx < 0)
  {
    return;
  }

#if JVET_Z0118_GDR
  bool isClean = pu.cs->isClean(pu.cu->Y().bottomRight(), CHANNEL_TYPE_LUMA);
#endif

  // insert inherited affine candidates
  Mv outputAffineMv[3];
  Position posLT = pu.Y().topLeft();
  Position posRT = pu.Y().topRight();
  Position posLB = pu.Y().bottomLeft();

#if JVET_Z0139_HIST_AFF
  int aiNeibeInherited[5];
  memset(aiNeibeInherited, 0, sizeof(aiNeibeInherited));
  Position neiPositions[5] = { posLB.offset(-2, -1), posRT.offset(-1, -2), posRT.offset(3, -2), posLB.offset(-2, 3), posLT.offset(-2, -2) };
  const PredictionUnit * neiPU[5];
  for (int nei = 0; nei < 5; nei++)
  {
    neiPU[nei] = pu.cs->getPURestricted(neiPositions[nei], pu, pu.chType);
  }
  int neiIdx[5] = { 0, 1, 2, 3, 4, };
  int leftNeiIdx[2];
  int leftAffNeiNum = 0;

  int aboveNeiIdx[3];
  int aboveAffNeiNum = 0;

  if (neiPU[3] && CU::isInter(*neiPU[3]->cu) && neiPU[3]->cu->affine && neiPU[3]->mergeType == MRG_TYPE_DEFAULT_N)
  {
    MotionInfo mvInfo = neiPU[3]->getMotionInfo(neiPositions[3]);
    if (mvInfo.isInter && mvInfo.interDir > 1 && mvInfo.interDir <= 3)
    {
      leftNeiIdx[leftAffNeiNum++] = 3;
    }
  }
  if (neiPU[0] && CU::isInter(*neiPU[0]->cu) && neiPU[0]->cu->affine && neiPU[0]->mergeType == MRG_TYPE_DEFAULT_N)
  {
    MotionInfo mvInfo = neiPU[0]->getMotionInfo(neiPositions[0]);
    if (mvInfo.isInter && mvInfo.interDir > 1 && mvInfo.interDir <= 3)
    {
      leftNeiIdx[leftAffNeiNum++] = 0;
    }
  }
  if (neiPU[2] && CU::isInter(*neiPU[2]->cu) && neiPU[2]->cu->affine && neiPU[2]->mergeType == MRG_TYPE_DEFAULT_N)
  {
    MotionInfo mvInfo = neiPU[2]->getMotionInfo(neiPositions[2]);
    if (mvInfo.isInter && mvInfo.interDir > 1 && mvInfo.interDir <= 3)
    {
      aboveNeiIdx[aboveAffNeiNum++] = 2;
    }
  }
  if (neiPU[1] && CU::isInter(*neiPU[1]->cu) && neiPU[1]->cu->affine && neiPU[1]->mergeType == MRG_TYPE_DEFAULT_N)
  {
    MotionInfo mvInfo = neiPU[1]->getMotionInfo(neiPositions[1]);
    if (mvInfo.isInter && mvInfo.interDir > 1 && mvInfo.interDir <= 3)
    {
      aboveNeiIdx[aboveAffNeiNum++] = 1;
    }
  }
  if (neiPU[4] && CU::isInter(*neiPU[4]->cu) && neiPU[4]->cu->affine && neiPU[4]->mergeType == MRG_TYPE_DEFAULT_N)
  {
    MotionInfo mvInfo = neiPU[4]->getMotionInfo(neiPositions[4]);
    if (mvInfo.isInter && mvInfo.interDir > 1 && mvInfo.interDir <= 3)
    {
      aboveNeiIdx[aboveAffNeiNum++] = 4;
    }
  }
  // check left neighbor 
  if (!addAffineMVPCandUnscaled(pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, affiAMVPInfo, aiNeibeInherited))
  {
    addAffineMVPCandUnscaled(pu, eRefPicList, refIdx, posLB, MD_LEFT, affiAMVPInfo, aiNeibeInherited);
  }
  leftAffNeiNum = 0;

  // check above neighbor
  if (!addAffineMVPCandUnscaled(pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, affiAMVPInfo, aiNeibeInherited))
  {
    if (!addAffineMVPCandUnscaled(pu, eRefPicList, refIdx, posRT, MD_ABOVE, affiAMVPInfo, aiNeibeInherited))
    {
      addAffineMVPCandUnscaled(pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, affiAMVPInfo, aiNeibeInherited);
    }
  }
  aboveAffNeiNum = 0;

#if JVET_AG0164_AFFINE_GPM
  if (!aiNeibeInherited[MD_BELOW_LEFT] && !aiNeibeInherited[MD_LEFT])
  {
    if (!addAffineMVPCandUnscaled(pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, affiAMVPInfo, aiNeibeInherited, 1))
    {
      addAffineMVPCandUnscaled(pu, eRefPicList, refIdx, posLB, MD_LEFT, affiAMVPInfo, aiNeibeInherited, 1);
    }
  }
  if (!aiNeibeInherited[MD_ABOVE_RIGHT] && !aiNeibeInherited[MD_ABOVE] && !aiNeibeInherited[MD_ABOVE_LEFT])
  {
    if (!addAffineMVPCandUnscaled(pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, affiAMVPInfo, aiNeibeInherited, 1))
    {
      if (!addAffineMVPCandUnscaled(pu, eRefPicList, refIdx, posRT, MD_ABOVE, affiAMVPInfo, aiNeibeInherited, 1))
      {
        addAffineMVPCandUnscaled(pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, affiAMVPInfo, aiNeibeInherited, 1);
      }
    }
  }
#endif

  for (int affHMVPIdx = 0; affHMVPIdx < 1; affHMVPIdx++)
  {
    if (affiAMVPInfo.numCand < AMVP_MAX_NUM_CANDS && leftAffNeiNum > 0)
    {
#if JVET_Z0118_GDR
      addSpatialAffineAMVPHMVPCand(pu, eRefPicList, refIdx, affiAMVPInfo, (isClean) ? pu.cs->motionLut.lutAff1 : pu.cs->motionLut.lutAff0, 0, leftNeiIdx, leftAffNeiNum, aiNeibeInherited, true);
#else
      addSpatialAffineAMVPHMVPCand(pu, eRefPicList, refIdx, affiAMVPInfo, pu.cs->motionLut.lutAff, 0, leftNeiIdx, leftAffNeiNum, aiNeibeInherited, true);
#endif
    }
    if (affiAMVPInfo.numCand < AMVP_MAX_NUM_CANDS && aboveAffNeiNum > 0)
    {
#if JVET_Z0118_GDR
      addSpatialAffineAMVPHMVPCand(pu, eRefPicList, refIdx, affiAMVPInfo, (isClean) ? pu.cs->motionLut.lutAff1 : pu.cs->motionLut.lutAff0, 0, aboveNeiIdx, aboveAffNeiNum, aiNeibeInherited, true);
#else
      addSpatialAffineAMVPHMVPCand(pu, eRefPicList, refIdx, affiAMVPInfo, pu.cs->motionLut.lutAff, 0, aboveNeiIdx, aboveAffNeiNum, aiNeibeInherited, true);
#endif
    }
  }
#else
  // check left neighbor
  if ( !addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, affiAMVPInfo ) )
  {
    addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_LEFT, affiAMVPInfo );
  }

  // check above neighbor
  if ( !addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, affiAMVPInfo ) )
  {
    if ( !addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE, affiAMVPInfo ) )
    {
      addAffineMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, affiAMVPInfo );
    }
  }
#endif

  if ( affiAMVPInfo.numCand >= AMVP_MAX_NUM_CANDS )
  {
    for (int i = 0; i < affiAMVPInfo.numCand; i++)
    {
      affiAMVPInfo.mvCandLT[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
      affiAMVPInfo.mvCandRT[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
      affiAMVPInfo.mvCandLB[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
    }
    return;
  }

  // insert constructed affine candidates
  int cornerMVPattern = 0;

  //-------------------  V0 (START) -------------------//
  AMVPInfo amvpInfo0;
  amvpInfo0.numCand = 0;

  // A->C: Above Left, Above, Left
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE_LEFT, amvpInfo0 );
  if ( amvpInfo0.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_ABOVE, amvpInfo0 );
  }
  if ( amvpInfo0.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posLT, MD_LEFT, amvpInfo0 );
  }
  cornerMVPattern = cornerMVPattern | amvpInfo0.numCand;

  //-------------------  V1 (START) -------------------//
  AMVPInfo amvpInfo1;
  amvpInfo1.numCand = 0;

  // D->E: Above, Above Right
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE, amvpInfo1 );
  if ( amvpInfo1.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posRT, MD_ABOVE_RIGHT, amvpInfo1 );
  }
  cornerMVPattern = cornerMVPattern | (amvpInfo1.numCand << 1);

  //-------------------  V2 (START) -------------------//
  AMVPInfo amvpInfo2;
  amvpInfo2.numCand = 0;

  // F->G: Left, Below Left
  addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_LEFT, amvpInfo2 );
  if ( amvpInfo2.numCand < 1 )
  {
    addMVPCandUnscaled( pu, eRefPicList, refIdx, posLB, MD_BELOW_LEFT, amvpInfo2 );
  }
  cornerMVPattern = cornerMVPattern | (amvpInfo2.numCand << 2);

  outputAffineMv[0] = amvpInfo0.mvCand[0];
  outputAffineMv[1] = amvpInfo1.mvCand[0];
  outputAffineMv[2] = amvpInfo2.mvCand[0];

  outputAffineMv[0].roundAffinePrecInternal2Amvr(pu.cu->imv);
  outputAffineMv[1].roundAffinePrecInternal2Amvr(pu.cu->imv);
  outputAffineMv[2].roundAffinePrecInternal2Amvr(pu.cu->imv);

  if ( cornerMVPattern == 7 || (cornerMVPattern == 3 && pu.cu->affineType == AFFINEMODEL_4PARAM) )
  {
    affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[0];
    affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[1];
    affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[2];

#if JVET_Z0139_HIST_AFF
    if (checkLastAffineAMVPCandRedundancy(pu, affiAMVPInfo))
#endif
    affiAMVPInfo.numCand++;
  }

#if JVET_Z0139_HIST_AFF
  for (int affHMVPIdx = 1; affHMVPIdx < MAX_NUM_AFF_HMVP_CANDS; affHMVPIdx++)
  {
    if (affiAMVPInfo.numCand < AMVP_MAX_NUM_CANDS && leftAffNeiNum > 0)
    {
#if JVET_Z0118_GDR
      addSpatialAffineAMVPHMVPCand(pu, eRefPicList, refIdx, affiAMVPInfo, (isClean) ? pu.cs->motionLut.lutAff1 : pu.cs->motionLut.lutAff0, 0, leftNeiIdx, leftAffNeiNum, aiNeibeInherited, true);
#else
      addSpatialAffineAMVPHMVPCand(pu, eRefPicList, refIdx, affiAMVPInfo, pu.cs->motionLut.lutAff, 0, leftNeiIdx, leftAffNeiNum, aiNeibeInherited, true);
#endif
    }
    if (affiAMVPInfo.numCand < AMVP_MAX_NUM_CANDS && aboveAffNeiNum > 0)
    {
#if JVET_Z0118_GDR
      addSpatialAffineAMVPHMVPCand(pu, eRefPicList, refIdx, affiAMVPInfo, (isClean) ? pu.cs->motionLut.lutAff1 : pu.cs->motionLut.lutAff0, 0, aboveNeiIdx, aboveAffNeiNum, aiNeibeInherited, true);
#else
      addSpatialAffineAMVPHMVPCand(pu, eRefPicList, refIdx, affiAMVPInfo, pu.cs->motionLut.lutAff, 0, aboveNeiIdx, aboveAffNeiNum, aiNeibeInherited, true);
#endif
    }
  }
#endif

  if ( affiAMVPInfo.numCand < 2 )
  {
    // check corner MVs
    for ( int i = 2; i >= 0 && affiAMVPInfo.numCand < AMVP_MAX_NUM_CANDS; i-- )
    {
      if ( cornerMVPattern & (1 << i) ) // MV i exist
      {
        affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = outputAffineMv[i];
        affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = outputAffineMv[i];
        affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = outputAffineMv[i];
#if JVET_Z0139_HIST_AFF
        if (checkLastAffineAMVPCandRedundancy(pu, affiAMVPInfo))
#endif
          affiAMVPInfo.numCand++;
      }
    }

    // Get Temporal Motion Predictor
    if ( affiAMVPInfo.numCand < 2 && pu.cs->picHeader->getEnableTMVPFlag() )
    {
      const int refIdxCol = refIdx;

      Position posRB = pu.Y().bottomRight().offset( -3, -3 );

      const PreCalcValues& pcv = *pu.cs->pcv;

      Position posC0;
      bool bC0Avail = false;
      Position posC1 = pu.Y().center();
      Mv cColMv;
      bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
      const SubPic &curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
      if (curSubPic.getTreatedAsPicFlag())
      {
        boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
          (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom());
      }
      if (boundaryCond)
      {
        int posYInCtu = posRB.y & pcv.maxCUHeightMask;
        if (posYInCtu + 4 < pcv.maxCUHeight)
        {
          posC0 = posRB.offset(4, 4);
          bC0Avail = true;
        }
      }
        if ((bC0Avail && getColocatedMVP(pu, eRefPicList, posC0, cColMv, refIdxCol, false)) || getColocatedMVP(pu, eRefPicList, posC1, cColMv, refIdxCol, false))
        {
          cColMv.roundAffinePrecInternal2Amvr(pu.cu->imv);
          affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand] = cColMv;
          affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand] = cColMv;
          affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand] = cColMv;
#if JVET_Z0139_HIST_AFF
          if (checkLastAffineAMVPCandRedundancy(pu, affiAMVPInfo))
#endif
            affiAMVPInfo.numCand++;
        }
    }

#if JVET_Z0139_NA_AFF
    if (affiAMVPInfo.numCand < AMVP_MAX_NUM_CANDS)
    {
      addNonAdjCstAffineMVPCandUnscaled(pu, eRefPicList, refIdx, affiAMVPInfo);
    }
#endif

#if JVET_Z0139_HIST_AFF
    if (affiAMVPInfo.numCand < AMVP_MAX_NUM_CANDS)
    {
      for (int affHMVPIdx = 0; affHMVPIdx < MAX_NUM_AFF_HMVP_CANDS; affHMVPIdx++)
      {
#if JVET_Z0118_GDR
        addSpatialAffineAMVPHMVPCand(pu, eRefPicList, refIdx, affiAMVPInfo, (isClean) ? pu.cs->motionLut.lutAff1 : pu.cs->motionLut.lutAff0, affHMVPIdx, neiIdx, 5, aiNeibeInherited, false);
#else
        addSpatialAffineAMVPHMVPCand(pu, eRefPicList, refIdx, affiAMVPInfo, pu.cs->motionLut.lutAff, affHMVPIdx, neiIdx, 5, aiNeibeInherited, false);
#endif
      }
    }
#endif
#if JVET_Z0139_HIST_AFF
    if (affiAMVPInfo.numCand < AMVP_MAX_NUM_CANDS)
#else
    if (affiAMVPInfo.numCand < 2)
#endif
    {
      // add zero MV
      for ( int i = affiAMVPInfo.numCand; i < AMVP_MAX_NUM_CANDS; i++ )
      {
        affiAMVPInfo.mvCandLT[affiAMVPInfo.numCand].setZero();
        affiAMVPInfo.mvCandRT[affiAMVPInfo.numCand].setZero();
        affiAMVPInfo.mvCandLB[affiAMVPInfo.numCand].setZero();
        affiAMVPInfo.numCand++;
      }
    }
  }

  for (int i = 0; i < affiAMVPInfo.numCand; i++)
  {
    affiAMVPInfo.mvCandLT[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
    affiAMVPInfo.mvCandRT[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
    affiAMVPInfo.mvCandLB[i].roundAffinePrecInternal2Amvr(pu.cu->imv);
  }
}

bool PU::addMVPCandUnscaled( const PredictionUnit &pu, const RefPicList &eRefPicList, const int &iRefIdx, const Position &pos, const MvpDir &eDir, AMVPInfo &info )
{
        CodingStructure &cs    = *pu.cs;
  const PredictionUnit *neibPU = NULL;
        Position neibPos;

  switch (eDir)
  {
  case MD_LEFT:
    neibPos = pos.offset( -1,  0 );
    break;
  case MD_ABOVE:
    neibPos = pos.offset(  0, -1 );
    break;
  case MD_ABOVE_RIGHT:
    neibPos = pos.offset(  1, -1 );
    break;
  case MD_BELOW_LEFT:
    neibPos = pos.offset( -1,  1 );
    break;
  case MD_ABOVE_LEFT:
    neibPos = pos.offset( -1, -1 );
    break;
  default:
    break;
  }

  neibPU = cs.getPURestricted( neibPos, pu, pu.chType );

#if JVET_Y0065_GPM_INTRA
  if( neibPU == NULL || !CU::isInter( *neibPU->cu ) || !neibPU->getMotionInfo( neibPos ).isInter)
#else
  if( neibPU == NULL || !CU::isInter( *neibPU->cu ) )
#endif
  {
    return false;
  }

  const MotionInfo& neibMi        = neibPU->getMotionInfo( neibPos );

  const int        currRefPOC     = cs.slice->getRefPic( eRefPicList, iRefIdx )->getPOC();
  const RefPicList eRefPicList2nd = ( eRefPicList == REF_PIC_LIST_0 ) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

  for( int predictorSource = 0; predictorSource < 2; predictorSource++ ) // examine the indicated reference picture list, then if not available, examine the other list.
  {
    const RefPicList eRefPicListIndex = ( predictorSource == 0 ) ? eRefPicList : eRefPicList2nd;
    const int        neibRefIdx       = neibMi.refIdx[eRefPicListIndex];

    if( neibRefIdx >= 0 && currRefPOC == cs.slice->getRefPOC( eRefPicListIndex, neibRefIdx ) )
    {
#if TM_AMVP
      info.mvCand[info.numCand] = neibMi.mv[eRefPicListIndex];
      if (!info.xCheckSimilarMotion(info.numCand))
      {
        info.numCand++;
        return true;
      }
#else
      info.mvCand[info.numCand++] = neibMi.mv[eRefPicListIndex];
      return true;
#endif
    }
  }

  return false;
}


void PU::addAMVPHMVPCand(const PredictionUnit &pu, const RefPicList eRefPicList, const int currRefPOC, AMVPInfo &info)
{
  const Slice &slice = *(*pu.cs).slice;

  MotionInfo neibMi;
#if JVET_Z0118_GDR
  bool isClean = pu.cs->isClean(pu.cu->Y().bottomRight(), CHANNEL_TYPE_LUMA);
#endif

#if JVET_Z0075_IBC_HMVP_ENLARGE
#if JVET_Z0118_GDR
  auto &lut = (isClean) ? pu.cs->motionLut.lut1 : pu.cs->motionLut.lut0;
#else
  auto &lut = pu.cs->motionLut.lut;
#endif
#else
#if JVET_Z0118_GDR
  auto &lut = CU::isIBC(*pu.cu) ? (isClean ? pu.cs->motionLut.lutIbc1 : pu.cs->motionLut.lutIbc0) : (isClean ? pu.cs->motionLut.lut1 : pu.cs->motionLut.lut0);
#else
  auto &lut = CU::isIBC(*pu.cu) ? pu.cs->motionLut.lutIbc : pu.cs->motionLut.lut;
#endif
#endif

  int num_avai_candInLUT = (int) lut.size();
  int num_allowedCand = std::min(MAX_NUM_HMVP_AVMPCANDS, num_avai_candInLUT);
  const RefPicList eRefPicList2nd = (eRefPicList == REF_PIC_LIST_0) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

  for (int mrgIdx = 1; mrgIdx <= num_allowedCand; mrgIdx++)
  {
#if TM_AMVP
    if (info.numCand >= info.maxStorageSize)
#else
    if (info.numCand >= AMVP_MAX_NUM_CANDS)
#endif
    {
      return;
    }
    neibMi = lut[mrgIdx - 1];

    for (int predictorSource = 0; predictorSource < 2; predictorSource++)
    {
      const RefPicList eRefPicListIndex = (predictorSource == 0) ? eRefPicList : eRefPicList2nd;
      const int        neibRefIdx = neibMi.refIdx[eRefPicListIndex];

#if JVET_Z0075_IBC_HMVP_ENLARGE
      if (neibRefIdx >= 0 && ((currRefPOC == slice.getRefPOC(eRefPicListIndex, neibRefIdx))))
#else
      if (neibRefIdx >= 0 && (CU::isIBC(*pu.cu) || (currRefPOC == slice.getRefPOC(eRefPicListIndex, neibRefIdx))))
#endif
      {
        Mv pmv = neibMi.mv[eRefPicListIndex];
        pmv.roundTransPrecInternal2Amvr(pu.cu->imv);

#if TM_AMVP
        info.mvCand[info.numCand] = pmv;
        if (!info.xCheckSimilarMotion(info.numCand))
        {
          info.numCand++;
        }

        if( info.numCand >= info.maxStorageSize )
        {
          return;
        }
#else
        info.mvCand[info.numCand++] = pmv;
        if (info.numCand >= AMVP_MAX_NUM_CANDS)
        {
          return;
        }
#endif
      }
    }
  }
}

bool PU::isBipredRestriction(const PredictionUnit &pu)
{
#if !INTER_RM_SIZE_CONSTRAINTS
  if(pu.cu->lumaSize().width == 4 && pu.cu->lumaSize().height ==4 )
  {
    return true;
  }
  /* disable bi-prediction for 4x8/8x4 */
  if ( pu.cu->lumaSize().width + pu.cu->lumaSize().height == 12 )
  {
    return true;
  }
#endif
  return false;
}

#if JVET_AE0046_BI_GPM
void PU::spanPuMv2DmvrBuffer(const PredictionUnit& pu, Mv* bdmvrSubPuMv0, Mv* bdmvrSubPuMv1)
{
  CHECK(bdmvrSubPuMv0 == nullptr || bdmvrSubPuMv1 == nullptr, "Error to span PU MV into empty array");

  for (int subPuIdx = 0; subPuIdx < MAX_NUM_SUBCU_DMVR; subPuIdx++)
  {
    bdmvrSubPuMv0[subPuIdx] = pu.mv[0];
    bdmvrSubPuMv1[subPuIdx] = pu.mv[1];
  }
}
#endif

#if JVET_Z0139_HIST_AFF
bool PU::getAffineControlPointCand(const PredictionUnit &pu, MotionInfo mi[4], bool isAvailable[4], int verIdx[4], int8_t bcwIdx, int modelIdx, int verNum, AffineMergeCtx& affMrgType)
#else
void PU::getAffineControlPointCand(const PredictionUnit &pu, MotionInfo mi[4], bool isAvailable[4], int verIdx[4], int8_t bcwIdx, int modelIdx, int verNum, AffineMergeCtx& affMrgType)
#endif
{
  int cuW = pu.Y().width;
  int cuH = pu.Y().height;
  int vx, vy;
  int shift = MAX_CU_DEPTH;
  int shiftHtoW = shift + floorLog2(cuW) - floorLog2(cuH);

  // motion info
  Mv cMv[2][4];
  int refIdx[2] = { -1, -1 };
  int dir = 0;
  EAffineModel curType = (verNum == 2) ? AFFINEMODEL_4PARAM : AFFINEMODEL_6PARAM;
#if INTER_LIC
  bool LICFlag = false;
#endif
  if ( verNum == 2 )
  {
    int idx0 = verIdx[0], idx1 = verIdx[1];
    if ( !isAvailable[idx0] || !isAvailable[idx1] )
    {
#if JVET_Z0139_HIST_AFF
      return false;
#else
      return;
#endif
    }

    for ( int l = 0; l < 2; l++ )
    {
      if ( mi[idx0].refIdx[l] >= 0 && mi[idx1].refIdx[l] >= 0 )
      {
        // check same refidx and different mv
        if ( mi[idx0].refIdx[l] == mi[idx1].refIdx[l])
        {
          dir |= (l + 1);
          refIdx[l] = mi[idx0].refIdx[l];
        }
      }
    }
#if INTER_LIC
    LICFlag = mi[idx0].usesLIC || mi[idx1].usesLIC;
#endif
  }
  else if ( verNum == 3 )
  {
    int idx0 = verIdx[0], idx1 = verIdx[1], idx2 = verIdx[2];
    if ( !isAvailable[idx0] || !isAvailable[idx1] || !isAvailable[idx2] )
    {
#if JVET_Z0139_HIST_AFF
      return false;
#else
      return;
#endif
    }

    for ( int l = 0; l < 2; l++ )
    {
      if ( mi[idx0].refIdx[l] >= 0 && mi[idx1].refIdx[l] >= 0 && mi[idx2].refIdx[l] >= 0 )
      {
        // check same refidx and different mv
        if ( mi[idx0].refIdx[l] == mi[idx1].refIdx[l] && mi[idx0].refIdx[l] == mi[idx2].refIdx[l])
        {
          dir |= (l + 1);
          refIdx[l] = mi[idx0].refIdx[l];
        }
      }
    }

#if INTER_LIC
    LICFlag = mi[idx0].usesLIC || mi[idx1].usesLIC || mi[idx2].usesLIC;
#endif
  }

  if ( dir == 0 )
  {
#if JVET_Z0139_HIST_AFF
    return false;
#else
    return;
#endif
  }

  for ( int l = 0; l < 2; l++ )
  {
    if ( dir & (l + 1) )
    {
      for ( int i = 0; i < verNum; i++ )
      {
        cMv[l][verIdx[i]] = mi[verIdx[i]].mv[l];
      }

      // convert to LT, RT[, [LB]]
      switch ( modelIdx )
      {
      case 0: // 0 : LT, RT, LB
        break;

      case 1: // 1 : LT, RT, RB
        cMv[l][2].hor = cMv[l][3].hor + cMv[l][0].hor - cMv[l][1].hor;
        cMv[l][2].ver = cMv[l][3].ver + cMv[l][0].ver - cMv[l][1].ver;
        cMv[l][2].clipToStorageBitDepth();
        break;

      case 2: // 2 : LT, LB, RB
        cMv[l][1].hor = cMv[l][3].hor + cMv[l][0].hor - cMv[l][2].hor;
        cMv[l][1].ver = cMv[l][3].ver + cMv[l][0].ver - cMv[l][2].ver;
        cMv[l][1].clipToStorageBitDepth();
        break;

      case 3: // 3 : RT, LB, RB
        cMv[l][0].hor = cMv[l][1].hor + cMv[l][2].hor - cMv[l][3].hor;
        cMv[l][0].ver = cMv[l][1].ver + cMv[l][2].ver - cMv[l][3].ver;
        cMv[l][0].clipToStorageBitDepth();
        break;

      case 4: // 4 : LT, RT
#if JVET_Z0139_HIST_AFF
      {
        Mv mvLT, mvRT, mvLB;
        mvLT = cMv[l][0];
        mvRT = cMv[l][1];
        int shift = MAX_CU_DEPTH;
        int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;

        iDMvHorX = (mvRT - mvLT).getHor() << (shift - floorLog2(pu.lwidth()));
        iDMvHorY = (mvRT - mvLT).getVer() << (shift - floorLog2(pu.lwidth()));
        iDMvVerX = -iDMvHorY;
        iDMvVerY = iDMvHorX;
        
        int iMvScaleHor = mvLT.getHor() << shift;
        int iMvScaleVer = mvLT.getVer() << shift;
        int horTmp, verTmp;

        horTmp = iMvScaleHor + iDMvVerX * pu.lheight();
        verTmp = iMvScaleVer + iDMvVerY * pu.lheight();
        roundAffineMv(horTmp, verTmp, shift);
        cMv[l][2].hor = horTmp;
        cMv[l][2].ver = verTmp;
        cMv[l][2].clipToStorageBitDepth();
      }
#endif
        break;

      case 5: // 5 : LT, LB
        vx = (cMv[l][0].hor << shift) + ((cMv[l][2].ver - cMv[l][0].ver) << shiftHtoW);
        vy = (cMv[l][0].ver << shift) - ((cMv[l][2].hor - cMv[l][0].hor) << shiftHtoW);
        roundAffineMv( vx, vy, shift );
        cMv[l][1].set( vx, vy );
        cMv[l][1].clipToStorageBitDepth();
        break;

      default:
        CHECK( 1, "Invalid model index!\n" );
        break;
      }
    }
    else
    {
      for ( int i = 0; i < 4; i++ )
      {
        cMv[l][i].hor = 0;
        cMv[l][i].ver = 0;
      }
    }
  }

  for ( int i = 0; i < 3; i++ )
  {
    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 0][i].mv = cMv[0][i];
    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 0][i].refIdx = refIdx[0];

    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 1][i].mv = cMv[1][i];
    affMrgType.mvFieldNeighbours[(affMrgType.numValidMergeCand << 1) + 1][i].refIdx = refIdx[1];
  }
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  affMrgType.colIdx[affMrgType.numValidMergeCand] = 0;
#endif
  affMrgType.interDirNeighbours[affMrgType.numValidMergeCand] = dir;
  affMrgType.affineType[affMrgType.numValidMergeCand] = curType;
  affMrgType.bcwIdx[affMrgType.numValidMergeCand] = (dir == 3) ? bcwIdx : BCW_DEFAULT;
#if INTER_LIC
  affMrgType.licFlags[affMrgType.numValidMergeCand] = LICFlag;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
#if JVET_AD0213_LIC_IMP
  affMrgType.obmcFlags[affMrgType.numValidMergeCand] = true;
#else
  affMrgType.obmcFlags[affMrgType.numValidMergeCand] = !LICFlag;
#endif
#endif
#if JVET_Z0139_HIST_AFF
  if (!checkLastAffineMergeCandRedundancy(pu, affMrgType))
  {
    return false;
  }
  return true;
#else
  affMrgType.numValidMergeCand++;
  return;
#endif
}
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
const int getNonAdjAvailableAffineNeighboursByDistance(const PredictionUnit &pu, const PredictionUnit *npu[], AffineMergeCtx &affMrgCtx, const int mrgCandStart, const int mrgCandIdx)
{
#if JVET_Z0118_GDR
  bool isClean = pu.cs->isClean(pu.cu->Y().bottomRight(), CHANNEL_TYPE_LUMA);
#endif
  const Position posLT = pu.Y().topLeft();
  const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
  int            num = mrgCandStart;
  uint8_t   cntLeftCand = 0;
  uint8_t  cntAboveCand = 0;
  int      log2CtuSize = floorLog2(pu.cs->sps->getCTUSize());
  int      ctuX = ((posLT.x >> log2CtuSize) << log2CtuSize);
  int      ctuY = ((posLT.y >> log2CtuSize) << log2CtuSize);

  int       offsetX = 0;
  int       offsetY = 0;
  for (int pos = 1; pos < (AFF_NON_ADJACENT_DIST + 1); pos++)
  {
    bool isAboveAva = (cntAboveCand < AFF_MAX_NON_ADJACENT_INHERITED_CANDS);
    bool isLeftAva = (cntLeftCand < AFF_MAX_NON_ADJACENT_INHERITED_CANDS);
    for (int posA = 0; ; posA++)
    {
      offsetX = ((int)pu.Y().width) * (pos + 1 - posA);
      offsetY = -((int)pu.Y().height * pos) - 1;
      if (offsetX < (-((int)pu.Y().width * pos) - 1))
      {
        break;
      }

      const Position posTemp = PU::convertNonAdjAffineBlkPos(posLT.offset(offsetX, offsetY), ctuX, ctuY);
      if (posTemp == Position(-1, -1))
      {
        break;
      }
      const PredictionUnit *puTemp = pu.cs->getPURestricted(posTemp, pu, pu.chType);
      if (puTemp && puTemp->cu->affine && puTemp->mergeType == MRG_TYPE_DEFAULT_N
        && PU::isDiffMER(pu.lumaPos(), posTemp, plevel))
      {
        bool redudant = false;
        for (int i = 0; i < num; i++)
        {
          if (puTemp == npu[i])
          {
            redudant = true;
            break;
          }
        }
        if (!redudant && isAboveAva)
        {
          npu[num++] = puTemp;
          cntAboveCand++;

          isAboveAva = (cntAboveCand < AFF_MAX_NON_ADJACENT_INHERITED_CANDS);
          if (!isAboveAva)
          {
            break;
          }
        }
      }
    }

    for (int posL = 0; ; posL++)
    {
      offsetX = -((int)pu.Y().width * pos);
      offsetY = ((int)pu.Y().height) * (pos + 1 - posL) - 1;
      if (offsetY < (-((int)pu.Y().height * pos) - 1))
      {
        break;
      }

      const Position posTemp = PU::convertNonAdjAffineBlkPos(posLT.offset(offsetX, offsetY), ctuX, ctuY);
      if (posTemp == Position(-1, -1))
      {
        break;
      }
      const PredictionUnit *puTemp = pu.cs->getPURestricted(posTemp, pu, pu.chType);
      if (puTemp && puTemp->cu->affine && puTemp->mergeType == MRG_TYPE_DEFAULT_N
        && PU::isDiffMER(pu.lumaPos(), posTemp, plevel))
      {
        bool redudant = false;
        for (int i = 0; i < num; i++)
        {
          if (puTemp == npu[i])
          {
            redudant = true;
            break;
          }
        }
        if (!redudant && isLeftAva)
        {
          npu[num++] = puTemp;
          cntLeftCand++;

          isLeftAva = (cntLeftCand < AFF_MAX_NON_ADJACENT_INHERITED_CANDS);
          if (!isLeftAva)
          {
            break;
          }
        }
      }
    }
  }
#if JVET_Z0118_GDR
  int lutSize = (isClean) ? (int)pu.cs->motionLut.lutAffInherit1.size() : (int)pu.cs->motionLut.lutAffInherit0.size();
#else
  int lutSize = (int)pu.cs->motionLut.lutAffInherit.size();
#endif
  for (int listIdx = 0; listIdx < lutSize; listIdx++)
  {
#if JVET_Z0118_GDR
    AffineInheritInfo& affHistInfo = (isClean) ? pu.cs->motionLut.lutAffInherit1[lutSize - 1 - listIdx] : pu.cs->motionLut.lutAffInherit0[lutSize - 1 - listIdx];
#else
    AffineInheritInfo& affHistInfo = pu.cs->motionLut.lutAffInherit[lutSize - 1 - listIdx];
#endif
    const Position posTemp = affHistInfo.basePos;
    const PredictionUnit *puTemp = pu.cs->getPURestricted(posTemp, pu, pu.chType);
    if (puTemp && puTemp->cu->affine && puTemp->mergeType == MRG_TYPE_DEFAULT_N
      && PU::isDiffMER(pu.lumaPos(), posTemp, plevel))
    {
      bool redudant = false;
      for (int i = 0; i < num; i++)
      {
        if (puTemp == npu[i])
        {
          redudant = true;
          break;
        }
      }
      if (!redudant)
      {
        npu[num++] = puTemp;
      }
    }
  }
  return num;
}
Position PU::convertNonAdjAffineBlkPos(const Position &pos, int curCtuX, int curCtuY)
{
  if (pos.x < 0 || pos.y < 0)
  {
    return Position(-1, -1);
  }

  PosType newX = pos.x;
  PosType newY = pos.y;
  if (newY < curCtuY && newX < curCtuX)
  {
    return Position(curCtuX - 1, curCtuY - 1);
  }

  if (newY < curCtuY && newX >= curCtuX)
  {
    return Position(newX, curCtuY - 1);
  }

  if (newY >= curCtuY && newX < curCtuX)
  {
    return Position(curCtuX - 1, newY);
  }

  newX = ((newX >> 4) << 4);
  newY = ((newY >> 4) << 4);

  return Position(newX, newY);
}
#endif
#if JVET_Z0139_NA_AFF
int PU::getMvDiffThresholdByWidthAndHeight(const PredictionUnit &pu, bool width)
{
  uint32_t numPixels = (width ? pu.lwidth() : pu.lheight());
  if (numPixels <= 8)
  {
    return (1 << MV_FRACTIONAL_BITS_INTERNAL) >> 4;
  }
  else if (numPixels <= 32)
  {
    return (1 << MV_FRACTIONAL_BITS_INTERNAL) >> 3;
  }
  else if (numPixels <= 64)
  {
    return (1 << MV_FRACTIONAL_BITS_INTERNAL) >> 2;
  }
  else
  {
    return (1 << MV_FRACTIONAL_BITS_INTERNAL) >> 1;
  }
}

bool PU::addNonAdjCstAffineMVPCandUnscaled(const PredictionUnit &pu, const RefPicList &refPicList, const int &refIdx, AffineAMVPInfo &affiAmvpInfo)
{
  const Position posLT[3] = { pu.Y().topLeft().offset( -1, -1 ), pu.Y().topLeft().offset( 0, -1 ), pu.Y().topLeft().offset( -1, 0 ) };
  const Position posRT[2] = { pu.Y().topRight().offset(0, -1), pu.Y().topRight().offset(1, -1) };
  const Position posLB[2] = { pu.Y().bottomLeft().offset(-1, 0), pu.Y().bottomLeft().offset(-1, 1) };
  const unsigned plevel   = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;

  for (int i = 1; i < (AFF_NON_ADJACENT_DIST + 1); i++)
  {
    MotionInfo miNew[3];
    Position   posNew[3];
    bool       isAvailableNew[3] = { false, false, false };

    const Position        posTRNew     = Position(posRT[0].x, posRT[0].y - (i * ((int) pu.Y().height)));
    const PredictionUnit *puNeighTRNew = pu.cs->getPURestricted(posTRNew, pu, pu.chType);
#if JVET_Y0065_GPM_INTRA
    if (puNeighTRNew && CU::isInter(*puNeighTRNew->cu) && puNeighTRNew->getMotionInfo(posTRNew).isInter && PU::isDiffMER(pu.lumaPos(), posTRNew, plevel))
#else
    if (puNeighTRNew && CU::isInter(*puNeighTRNew->cu) && PU::isDiffMER(pu.lumaPos(), posTRNew, plevel))
#endif
    {
      isAvailableNew[1] = true;
      miNew[1]          = puNeighTRNew->getMotionInfo(posTRNew);
      posNew[1]         = posTRNew;
      CHECK(posTRNew.x < 0 || posTRNew.y < 0, "posTRNew < 0");
    }

    for (int j = 1; j < (AFF_NON_ADJACENT_DIST + 1); j++)
    {
      isAvailableNew[0]                  = false;
      isAvailableNew[2]                  = false;
      const Position        posLBNew     = Position(posLB[0].x - (j * ((int) pu.Y().width)), posLB[0].y);
      const PredictionUnit *puNeighLBNew = pu.cs->getPURestricted(posLBNew, pu, pu.chType);
#if JVET_Y0065_GPM_INTRA
      if (puNeighLBNew && CU::isInter(*puNeighLBNew->cu) && puNeighLBNew->getMotionInfo(posLBNew).isInter && PU::isDiffMER(pu.lumaPos(), posLBNew, plevel))
#else
      if (puNeighLBNew && CU::isInter(*puNeighLBNew->cu) && PU::isDiffMER(pu.lumaPos(), posLBNew, plevel))
#endif
      {
        isAvailableNew[2] = true;
        miNew[2]          = puNeighLBNew->getMotionInfo(posLBNew);
        posNew[2]         = posLBNew;
        CHECK(posLBNew.x < 0 || posLBNew.y < 0, "posLBNew < 0");
      }

      PosType posX = isAvailableNew[2] ? posNew[2].x : (posLT[0].x - (j * ((int) pu.Y().width)));
      PosType posY = isAvailableNew[1] ? posNew[1].y : (posLT[0].y - (i * ((int) pu.Y().height)));
      if (posX < 0)
      {
        posX = isAvailableNew[1] ? posLT[1].x : -1;
      }
      if (posY < 0)
      {
        posY = isAvailableNew[2] ? posLT[2].y : -1;
      }

      const Position        posLTNew     = Position(posX, posY);
      const PredictionUnit *puNeighLTNew = pu.cs->getPURestricted(posLTNew, pu, pu.chType);
#if JVET_Y0065_GPM_INTRA
      if (puNeighLTNew && CU::isInter(*puNeighLTNew->cu) && puNeighLTNew->getMotionInfo(posLTNew).isInter && PU::isDiffMER(pu.lumaPos(), posLTNew, plevel))
#else
      if (puNeighLTNew && CU::isInter(*puNeighLTNew->cu) && PU::isDiffMER(pu.lumaPos(), posLTNew, plevel))
#endif
      {
        isAvailableNew[0] = true;
        miNew[0]          = puNeighLTNew->getMotionInfo(posLTNew);
        posNew[0]         = posLTNew;
        CHECK(posLTNew.x < 0 || posLTNew.y < 0, "posLTNew < 0");
      }

      if (addNonAdjCstAffineMVPConstructedCPMV(pu, miNew, isAvailableNew, posNew, refPicList, refIdx, affiAmvpInfo))
      {
        if (affiAmvpInfo.numCand >= AMVP_MAX_NUM_CANDS)
        {
          return true;
        }
      }
    }
  }

  return false;
}

int PU::getNonAdjAffParaDivFun(int num1, int num2)
{
  int divTable[16] = { 0, 7, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 1, 1, 0 };
  int x = floorLog2(num2);
  int normNum1 = (num2 << 4 >> x) & 15;
  int v = divTable[normNum1] | 8;
  x += (normNum1 != 0);
  int shift = 13 - x;
  int retVal = 0;
  if (shift < 0)
  {
    shift = -shift;
    int add = (1 << (shift - 1));
    retVal = (num1 * v + add) >> shift;
  }
  else
  {
    retVal = (num1 * v) << shift;
  }
  return (retVal>>(16-MAX_CU_DEPTH));
}

bool PU::addNonAdjCstAffineMVPConstructedCPMV( const PredictionUnit &pu, MotionInfo miNew[3], bool isAvaNew[3], Position pos[3], const RefPicList &refPicList, const int &refIdx, AffineAMVPInfo &affiAmvpInfo)
{
  if (!isAvaNew[0] || (!isAvaNew[1] && !isAvaNew[2]))
  {
    return false;
  }
  
  const int        currRefPOC    = pu.cs->slice->getRefPic(refPicList, refIdx)->getPOC();
  const RefPicList refPicList2nd = (refPicList == REF_PIC_LIST_0) ? REF_PIC_LIST_1 : REF_PIC_LIST_0;

  int  shift  = MAX_CU_DEPTH;
  int posNeiX = pos[0].x;
  int posNeiY = pos[0].y;
  int posCurX = pu.Y().pos().x;
  int posCurY = pu.Y().pos().y;

  int curW = pu.Y().width;
  int curH = pu.Y().height;
  int neiW = pos[1].x - pos[0].x;
  int neiH = pos[2].y - pos[0].y;
  if (!isAvaNew[1])
  {
    neiW = pu.Y().topRight().x - pos[0].x;
  }
  if (!isAvaNew[2])
  {
    neiH = pu.Y().bottomLeft().y - pos[0].y;
  }

  bool isConverted = false;

  for (int predictorSource = 0; predictorSource < 2; predictorSource++)
  {
    const RefPicList refListindex     = (predictorSource == 0) ? refPicList : refPicList2nd;
    const int        neibRefIdx = miNew[0].refIdx[refListindex];
    if (neibRefIdx < 0 || pu.cu->slice->getRefPOC(refListindex, neibRefIdx) != currRefPOC)
    {
      continue;
    }
    for (int modelIdx = 0; modelIdx < 3; modelIdx++)
    {
      if ((modelIdx == 1 && !isAvaNew[1]) || (modelIdx == 2 && !isAvaNew[2]))
      {
        continue;
      }
      Mv  outputAffineMv[3];
      Mv  mvLT, mvRT, mvLB;
      int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;
      int horTmp, verTmp;

      mvLT = miNew[0].mv[refListindex];
      mvRT = miNew[1].mv[refListindex];
      mvLB = miNew[2].mv[refListindex];

      int iMvScaleHor = mvLT.getHor() << shift;
      int iMvScaleVer = mvLT.getVer() << shift;
      iDMvHorX        = getNonAdjAffParaDivFun((mvRT - mvLT).getHor(), neiW);
      iDMvHorY        = getNonAdjAffParaDivFun((mvRT - mvLT).getVer(), neiW);
      iDMvVerX        = getNonAdjAffParaDivFun((mvLB - mvLT).getHor(), neiH);
      iDMvVerY        = getNonAdjAffParaDivFun((mvLB - mvLT).getVer(), neiH);

      if (!modelIdx && isAvaNew[0] && isAvaNew[1] && isAvaNew[2]
          && miNew[0].refIdx[refListindex] == miNew[1].refIdx[refListindex] 
          && miNew[0].refIdx[refListindex] == miNew[2].refIdx[refListindex])
      {
      }
      else if (modelIdx == 1 && isAvaNew[0] && isAvaNew[1]
               && miNew[0].refIdx[refListindex] == miNew[1].refIdx[refListindex])
      {
        iDMvVerX = -iDMvHorY;
        iDMvVerY = iDMvHorX;
      }
      else if (modelIdx == 2 && isAvaNew[0] && isAvaNew[2]
               && miNew[0].refIdx[refListindex] == miNew[2].refIdx[refListindex])
      {
        iDMvHorX = iDMvVerY;
        iDMvHorY = -iDMvVerX;
      }
      else
      {
        continue;
      }

      horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY - posNeiY);
      verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY - posNeiY);
      roundAffineMv(horTmp, verTmp, shift);
      outputAffineMv[0].hor = horTmp;
      outputAffineMv[0].ver = verTmp;
      outputAffineMv[0].clipToStorageBitDepth();

      horTmp = iMvScaleHor + iDMvHorX * (posCurX + curW - posNeiX) + iDMvVerX * (posCurY - posNeiY);
      verTmp = iMvScaleVer + iDMvHorY * (posCurX + curW - posNeiX) + iDMvVerY * (posCurY - posNeiY);
      roundAffineMv(horTmp, verTmp, shift);
      outputAffineMv[1].hor = horTmp;
      outputAffineMv[1].ver = verTmp;
      outputAffineMv[1].clipToStorageBitDepth();

      if (pu.cu->affineType == AFFINEMODEL_6PARAM)
      {
        horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY + curH - posNeiY);
        verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY + curH - posNeiY);
        roundAffineMv(horTmp, verTmp, shift);
        outputAffineMv[2].hor = horTmp;
        outputAffineMv[2].ver = verTmp;
        outputAffineMv[2].clipToStorageBitDepth();
      }

      outputAffineMv[0].roundAffinePrecInternal2Amvr(pu.cu->imv);
      outputAffineMv[1].roundAffinePrecInternal2Amvr(pu.cu->imv);
      if (pu.cu->affineType == AFFINEMODEL_6PARAM)
      {
        outputAffineMv[2].roundAffinePrecInternal2Amvr(pu.cu->imv);
      }
      affiAmvpInfo.mvCandLT[affiAmvpInfo.numCand] = outputAffineMv[0];
      affiAmvpInfo.mvCandRT[affiAmvpInfo.numCand] = outputAffineMv[1];
      affiAmvpInfo.mvCandLB[affiAmvpInfo.numCand] = Mv();
      if (pu.cu->affineType == AFFINEMODEL_6PARAM)
      {
        affiAmvpInfo.mvCandLB[affiAmvpInfo.numCand] = outputAffineMv[2];
      }
      if (!checkLastAffineAMVPCandRedundancy(pu, affiAmvpInfo))
      {
        affiAmvpInfo.mvCandLT[affiAmvpInfo.numCand] = Mv();
        affiAmvpInfo.mvCandRT[affiAmvpInfo.numCand] = Mv();
        affiAmvpInfo.mvCandLB[affiAmvpInfo.numCand] = Mv();
        continue;
      }
      affiAmvpInfo.numCand++;
      isConverted = true;
      if (affiAmvpInfo.numCand >= AMVP_MAX_NUM_CANDS)
      {
        return true;
      }
    }
  }

  return isConverted;
}

bool PU::xCPMVSimCheck(const PredictionUnit &pu, AffineMergeCtx &affMrgCtx, Mv curCpmv[2][3], unsigned char curdir, int8_t curRefIdx[2], EAffineModel curType, int bcwIdx, bool LICFlag)
{
  if (affMrgCtx.numValidMergeCand == 0 || affMrgCtx.mergeType[affMrgCtx.numValidMergeCand - 1] != MRG_TYPE_DEFAULT_N)
  {
    return false;
  }
  int iDMvHorX[2], iDMvHorY[2], iDMvVerX[2], iDMvVerY[2], iMvScaleHor[2], iMvScaleVer[2];
  int iDMvHorXCand, iDMvHorYCand, iDMvVerXCand, iDMvVerYCand, iMvScaleHorCand, iMvScaleVerCand;
  Mv  mvLT, mvRT, mvLB;

  const int mvDifThldWidth  = getMvDiffThresholdByWidthAndHeight(pu, true);
  const int mvDifThldHeight = getMvDiffThresholdByWidthAndHeight(pu, false);

  for (int l = 0; l < 2; l++)
  {
    if (curdir & (l + 1))
    {
      mvLT        = curCpmv[l][0];
      mvRT        = curCpmv[l][1];
      mvLB        = curCpmv[l][2];
      iDMvHorX[l] = (mvRT - mvLT).getHor();
      iDMvHorY[l] = (mvRT - mvLT).getVer();
      if (curType == AFFINEMODEL_6PARAM)
      {
        iDMvVerX[l] = (mvLB - mvLT).getHor();
        iDMvVerY[l] = (mvLB - mvLT).getVer();
      }
      else
      {
        iDMvVerX[l] = -iDMvHorY[l];
        iDMvVerY[l] = iDMvHorX[l];
      }

      iMvScaleHor[l] = mvLT.getHor();
      iMvScaleVer[l] = mvLT.getVer();
    }
    else
    {
      iDMvHorX[l]    = iDMvHorY[l] = 0;
      iDMvVerX[l]    = iDMvVerY[l] = 0;
      iMvScaleHor[l] = iMvScaleVer[l] = 0;
    }
  }


  for (uint32_t ui = 0; ui < affMrgCtx.numValidMergeCand; ui++)
  {
    bool isSimilar = true;
    unsigned char candDir    = affMrgCtx.interDirNeighbours[ui];
    int8_t        candRefIdx[2];
    if ((affMrgCtx.mergeType[ui] != MRG_TYPE_DEFAULT_N) || (candDir != curdir))
    {
      continue;
    }
    candRefIdx[0] = affMrgCtx.mvFieldNeighbours[(ui << 1) + 0][0].refIdx;
    candRefIdx[1] = affMrgCtx.mvFieldNeighbours[(ui << 1) + 1][0].refIdx;

    if ((candDir == 3 && (curRefIdx[0] != candRefIdx[0] || curRefIdx[1] != candRefIdx[1]))
      || (candDir == 1 && curRefIdx[0] != candRefIdx[0]) || (candDir == 2 && curRefIdx[1] != candRefIdx[1]))
    {
      continue;
    }
    EAffineModel candAffType = affMrgCtx.affineType[ui];
    for (int l = 0; l < 2; l++) 
    {
      if (curdir & (l + 1)) 
      {
        mvLT = affMrgCtx.mvFieldNeighbours[(ui << 1) + l][0].mv;
        mvRT = affMrgCtx.mvFieldNeighbours[(ui << 1) + l][1].mv;
        mvLB = affMrgCtx.mvFieldNeighbours[(ui << 1) + l][2].mv;
        iDMvHorXCand = (mvRT - mvLT).getHor();
        iDMvHorYCand = (mvRT - mvLT).getVer();
        if (candAffType == AFFINEMODEL_6PARAM)
        {
          iDMvVerXCand = (mvLB - mvLT).getHor();
          iDMvVerYCand = (mvLB - mvLT).getVer();
        }
        else
        {
          iDMvVerXCand = -iDMvHorYCand;
          iDMvVerYCand = iDMvHorXCand;
        }

        iMvScaleHorCand = mvLT.getHor();
        iMvScaleVerCand = mvLT.getVer();
      
        int diffHorX = iDMvHorX[l] - iDMvHorXCand;
        int diffHorY = iDMvHorY[l] - iDMvHorYCand;
        int diffVerX = iDMvVerX[l] - iDMvVerXCand;
        int diffVerY = iDMvVerY[l] - iDMvVerYCand;
        int diffiMvScaleHor = iMvScaleHor[l] - iMvScaleHorCand;
        int diffiMvScaleVer = iMvScaleVer[l] - iMvScaleVerCand;
        
        if (abs(diffHorX) >= mvDifThldWidth || abs(diffHorY) >= mvDifThldWidth || abs(diffVerX) >= mvDifThldHeight
          || abs(diffVerY) >= mvDifThldHeight || abs(diffiMvScaleHor) >= 1
          || abs(diffiMvScaleVer) >= 1)
        {
          isSimilar = false;
        }
        if (!isSimilar)
        {
          break;
        }
      }
    }
    if (isSimilar)
    {
      return true;
    }
  }

  return false;
}

bool PU::addNonAdjAffineConstructedCPMV(const PredictionUnit &pu, MotionInfo miNew[4], bool isAvaNew[4], Position pos[4], int8_t bcwId, AffineMergeCtx &affMrgCtx, int mrgCandIdx)
{
  if (!isAvaNew[0] || (!isAvaNew[1] && !isAvaNew[2]))
  {
    return true;
  }
  int  shift        = MAX_CU_DEPTH;

  int posNeiX = pos[0].x;
  int posNeiY = pos[0].y;
  int posCurX = pu.Y().pos().x;
  int posCurY = pu.Y().pos().y;

  int curW = pu.Y().width;
  int curH = pu.Y().height;
  int neiW = pos[1].x - pos[0].x;
  int neiH = pos[2].y - pos[0].y;
  if (!isAvaNew[1])
  {
    neiW = pu.Y().topRight().x - pos[0].x;
  }
  if (!isAvaNew[2])
  {
    neiH = pu.Y().bottomLeft().y - pos[0].y;
  }

  bool isConverted = false;
  for (int modelIdx = 0; modelIdx < 3; modelIdx++)
  {
    if ((modelIdx == 1 && !isAvaNew[1]) || (modelIdx == 2 && !isAvaNew[2]))
    {
      continue;
    }
    Mv cMv[2][3];
    int8_t refIdx[2] = { -1, -1 };
    int dir = 0;
    EAffineModel curType = AFFINEMODEL_6PARAM;
    bool bLICFlag = false;

    for (int l = 0; l < 2; l++)
    {
      Mv mvLT, mvRT, mvLB;
      int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;
      int horTmp, verTmp;

      mvLT = miNew[0].mv[l];
      mvRT = miNew[1].mv[l];
      mvLB = miNew[2].mv[l];

      int iMvScaleHor = mvLT.getHor() << shift;
      int iMvScaleVer = mvLT.getVer() << shift;
      iDMvHorX        = getNonAdjAffParaDivFun((mvRT - mvLT).getHor(), neiW);
      iDMvHorY        = getNonAdjAffParaDivFun((mvRT - mvLT).getVer(), neiW);
      iDMvVerX        = getNonAdjAffParaDivFun((mvLB - mvLT).getHor(), neiH);
      iDMvVerY        = getNonAdjAffParaDivFun((mvLB - mvLT).getVer(), neiH);

      if (!modelIdx && isAvaNew[0] && isAvaNew[1] && isAvaNew[2] && miNew[0].refIdx[l] >= 0
          && miNew[0].refIdx[l] == miNew[1].refIdx[l] && miNew[0].refIdx[l] == miNew[2].refIdx[l])
      {
#if INTER_LIC
        bLICFlag = bLICFlag || miNew[0].usesLIC || miNew[1].usesLIC || miNew[2].usesLIC;
#endif
      }
      else if (modelIdx == 1 && isAvaNew[0] && isAvaNew[1] && miNew[0].refIdx[l] >= 0
               && miNew[0].refIdx[l] == miNew[1].refIdx[l])
      {
        iDMvVerX = -iDMvHorY;
        iDMvVerY = iDMvHorX;
#if INTER_LIC
        bLICFlag = bLICFlag || miNew[0].usesLIC || miNew[1].usesLIC;
#endif
      }
      else if (modelIdx == 2 && isAvaNew[0] && isAvaNew[2] && miNew[0].refIdx[l] >= 0
               && miNew[0].refIdx[l] == miNew[2].refIdx[l])
      {
        iDMvHorX = iDMvVerY;
        iDMvHorY = -iDMvVerX;
#if INTER_LIC
        bLICFlag = bLICFlag || miNew[0].usesLIC || miNew[2].usesLIC;
#endif
      }
      else
      {
        continue;
      }

      dir |= (l + 1);
      refIdx[l] = miNew[0].refIdx[l];

      horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY - posNeiY);
      verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY - posNeiY);
      roundAffineMv(horTmp, verTmp, shift);
      cMv[l][0].hor = horTmp;
      cMv[l][0].ver = verTmp;
      cMv[l][0].clipToStorageBitDepth();

      horTmp = iMvScaleHor + iDMvHorX * (posCurX + curW - posNeiX) + iDMvVerX * (posCurY - posNeiY);
      verTmp = iMvScaleVer + iDMvHorY * (posCurX + curW - posNeiX) + iDMvVerY * (posCurY - posNeiY);
      roundAffineMv(horTmp, verTmp, shift);
      cMv[l][1].hor = horTmp;
      cMv[l][1].ver = verTmp;
      cMv[l][1].clipToStorageBitDepth();

      {
        horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY + curH - posNeiY);
        verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY + curH - posNeiY);
        roundAffineMv(horTmp, verTmp, shift);
        cMv[l][2].hor = horTmp;
        cMv[l][2].ver = verTmp;
        cMv[l][2].clipToStorageBitDepth();
      }
    }

    if (!dir)
    {
      continue;
    }
#if JVET_AD0213_LIC_IMP
    if (xCPMVSimCheck(pu, affMrgCtx, cMv, dir, refIdx, curType, (dir == 3) ? bcwId : BCW_DEFAULT, bLICFlag))
#else
    if (xCPMVSimCheck(pu, affMrgCtx, cMv, dir, refIdx, curType, (dir == 3) ? bcwId : BCW_DEFAULT, (dir != 3) ? bLICFlag : false))
#endif
    {
      continue;
    }
    isConverted = true;
    for (int i = 0; i < 3; i++)
    {
      affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][i].mv      = cMv[0][i];
      affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][i].refIdx = refIdx[0];

      affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][i].mv      = cMv[1][i];
      affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][i].refIdx = refIdx[1];
    }
    affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand]  = dir;
    affMrgCtx.affineType[affMrgCtx.numValidMergeCand]         = curType;
    affMrgCtx.mergeType[affMrgCtx.numValidMergeCand]           = MRG_TYPE_DEFAULT_N;
    affMrgCtx.bcwIdx[affMrgCtx.numValidMergeCand]              = (dir == 3) ? bcwId : BCW_DEFAULT;
#if INTER_LIC
#if JVET_AD0213_LIC_IMP
    affMrgCtx.licFlags[affMrgCtx.numValidMergeCand] = bLICFlag;
#else
    affMrgCtx.licFlags[affMrgCtx.numValidMergeCand] = (dir != 3) ? bLICFlag : false;
#endif
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
#if JVET_AD0213_LIC_IMP
    affMrgCtx.obmcFlags[affMrgCtx.numValidMergeCand] = true;
#else
    affMrgCtx.obmcFlags[affMrgCtx.numValidMergeCand] = (dir != 3) ? !bLICFlag : true;
#endif
#endif
    affMrgCtx.numValidMergeCand++;

    if (affMrgCtx.numValidMergeCand != 0 && affMrgCtx.numValidMergeCand - 1 == mrgCandIdx)
    {
      return false;
    }

    if (affMrgCtx.numValidMergeCand == affMrgCtx.maxNumMergeCand)
    {
      return false;
    }
    break;
  }

  return (!isConverted);
}

void PU::getNonAdjCstMergeCand(const PredictionUnit &pu, AffineMergeCtx &affMrgCtx, const int mrgCandIdx, bool isInitialized)
{
  const CodingStructure &cs    = *pu.cs;
  const Slice &          slice = *pu.cs->slice;
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
#if JVET_W0090_ARMC_TM
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
  const uint32_t maxNumAffineMergeCand = slice.getPicHeader()->getMaxNumAffineMergeCand() + ((pu.cs->sps->getUseAML()&& pu.cs->sps->getTMToolsEnableFlag()) ? ADDITIONAL_AFFINE_CAND_NUM : 0);
#else
  const uint32_t maxNumAffineMergeCand = slice.getPicHeader()->getMaxNumAffineMergeCand() + (pu.cs->sps->getUseAML() ? ADDITIONAL_AFFINE_CAND_NUM : 0);
#endif
#else
  const uint32_t maxNumAffineMergeCand = slice.getPicHeader()->getMaxNumAffineMergeCand();
#endif
#else
  const uint32_t maxNumAffineMergeCand = slice.getPicHeader()->getMaxNumAffineMergeCand();
#endif
  const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
  const Position posLT[3] = { pu.Y().topLeft().offset( -1, -1 ), pu.Y().topLeft().offset( 0, -1 ), pu.Y().topLeft().offset( -1, 0 ) };
  const Position posRT[2] = { pu.Y().topRight().offset(0, -1), pu.Y().topRight().offset(1, -1) };
  const Position posLB[2] = { pu.Y().bottomLeft().offset(-1, 0), pu.Y().bottomLeft().offset(-1, 1) };
  if (!isInitialized)
  {
    for (int i = 0; i < maxNumAffineMergeCand; i++)
    {
      for (int mvNum = 0; mvNum < 3; mvNum++)
      {
        affMrgCtx.mvFieldNeighbours[(i << 1) + 0][mvNum].setMvField(Mv(), -1);
        affMrgCtx.mvFieldNeighbours[(i << 1) + 1][mvNum].setMvField(Mv(), -1);
      }
      affMrgCtx.interDirNeighbours[i] = 0;
      affMrgCtx.affineType[i]         = AFFINEMODEL_4PARAM;
      affMrgCtx.mergeType[i]          = MRG_TYPE_DEFAULT_N;
      affMrgCtx.bcwIdx[i]             = BCW_DEFAULT;
#if JVET_AG0276_NLIC
      affMrgCtx.altLMFlag[i] = false;
      affMrgCtx.altLMParaNeighbours[i].resetAltLinearModel();
#endif
#if INTER_LIC
      affMrgCtx.licFlags[i] = false;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
      affMrgCtx.obmcFlags[i] = true;
#endif
    }

    affMrgCtx.numValidMergeCand = 0;
    affMrgCtx.maxNumMergeCand   = maxNumAffineMergeCand;
  }

  for (int i = 1; i < (AFF_NON_ADJACENT_DIST + 1); i++)
  {
    MotionInfo miNew[4];
    Position   posNew[4];
    bool       isAvailableNew[4] = { false, false, false, false };
    int8_t     neighBcwNew[2]    = { BCW_DEFAULT, BCW_DEFAULT };

    const Position        posTRNew     = Position(posRT[0].x, posRT[0].y - (i * ((int) pu.Y().height)));
    const PredictionUnit *puNeighTRNew = cs.getPURestricted(posTRNew, pu, pu.chType);
#if JVET_Y0065_GPM_INTRA
    if (puNeighTRNew && CU::isInter(*puNeighTRNew->cu) && puNeighTRNew->getMotionInfo(posTRNew).isInter && PU::isDiffMER(pu.lumaPos(), posTRNew, plevel))
#else
    if (puNeighTRNew && CU::isInter(*puNeighTRNew->cu) && PU::isDiffMER(pu.lumaPos(), posTRNew, plevel))
#endif
    {
      isAvailableNew[1] = true;
      miNew[1]          = puNeighTRNew->getMotionInfo(posTRNew);
      neighBcwNew[1]    = puNeighTRNew->cu->bcwIdx;
      posNew[1]         = posTRNew;
      CHECK(posTRNew.x < 0 || posTRNew.y < 0, "posTRNew < 0");
    }

    for (int j = 1; j < (AFF_NON_ADJACENT_DIST + 1); j++)
    {
      isAvailableNew[0]                  = false;
      isAvailableNew[2]                  = false;
      const Position        posLBNew     = Position(posLB[0].x - (j * ((int) pu.Y().width)), posLB[0].y);
      const PredictionUnit *puNeighLBNew = cs.getPURestricted(posLBNew, pu, pu.chType);
#if JVET_Y0065_GPM_INTRA
      if (puNeighLBNew && CU::isInter(*puNeighLBNew->cu) && puNeighLBNew->getMotionInfo(posLBNew).isInter && PU::isDiffMER(pu.lumaPos(), posLBNew, plevel))
#else
      if (puNeighLBNew && CU::isInter(*puNeighLBNew->cu) && PU::isDiffMER(pu.lumaPos(), posLBNew, plevel))
#endif
      {
        isAvailableNew[2] = true;
        miNew[2]          = puNeighLBNew->getMotionInfo(posLBNew);
        posNew[2]         = posLBNew;
        CHECK(posLBNew.x < 0 || posLBNew.y < 0, "posLBNew < 0");
      }

      PosType posX = isAvailableNew[2] ? posNew[2].x : (posLT[0].x - (j * ((int) pu.Y().width)));
      PosType posY = isAvailableNew[1] ? posNew[1].y : (posLT[0].y - (i * ((int) pu.Y().height)));
      if (posX < 0)
      {
        posX = isAvailableNew[1] ? posLT[1].x : -1;
      }
      if (posY < 0)
      {
        posY = isAvailableNew[2] ? posLT[2].y : -1;
      }

      const Position        posLTNew     = Position(posX, posY);
      const PredictionUnit *puNeighLTNew = cs.getPURestricted(posLTNew, pu, pu.chType);
#if JVET_Y0065_GPM_INTRA
      if (puNeighLTNew && CU::isInter(*puNeighLTNew->cu) && puNeighLTNew->getMotionInfo(posLTNew).isInter && PU::isDiffMER(pu.lumaPos(), posLTNew, plevel))
#else
      if (puNeighLTNew && CU::isInter(*puNeighLTNew->cu) && PU::isDiffMER(pu.lumaPos(), posLTNew, plevel))
#endif
      {
        isAvailableNew[0] = true;
        miNew[0]          = puNeighLTNew->getMotionInfo(posLTNew);
        neighBcwNew[0]    = puNeighLTNew->cu->bcwIdx;
        posNew[0]         = posLTNew;
        CHECK(posLTNew.x < 0 || posLTNew.y < 0, "posLTNew < 0");
      }

      if (!addNonAdjAffineConstructedCPMV(pu, miNew, isAvailableNew, posNew, neighBcwNew[0], affMrgCtx, mrgCandIdx))
      {
        if (affMrgCtx.numValidMergeCand != 0 && affMrgCtx.numValidMergeCand - 1 == mrgCandIdx)
        {
          return;
        }

        if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
        {
          return;
        }
      }
    }
  }
}
#endif

const int getAvailableAffineNeighboursForLeftPredictor( const PredictionUnit &pu, const PredictionUnit* npu[] 
#if JVET_Z0139_HIST_AFF
  , int neiIdx[]
#endif
)
{
  const Position posLB = pu.Y().bottomLeft();
  int num = 0;
  const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;

  const PredictionUnit *puLeftBottom = pu.cs->getPURestricted( posLB.offset( -1, 1 ), pu, pu.chType );
  if (puLeftBottom && puLeftBottom->cu->affine && puLeftBottom->mergeType == MRG_TYPE_DEFAULT_N
      && PU::isDiffMER(pu.lumaPos(), posLB.offset(-1, 1), plevel))
  {
#if JVET_Z0139_HIST_AFF
    neiIdx[num] = 3;
#endif
    npu[num++] = puLeftBottom;
    return num;
  }

  const PredictionUnit* puLeft = pu.cs->getPURestricted( posLB.offset( -1, 0 ), pu, pu.chType );
  if (puLeft && puLeft->cu->affine && puLeft->mergeType == MRG_TYPE_DEFAULT_N
      && PU::isDiffMER(pu.lumaPos(), posLB.offset(-1, 0), plevel))
  {
#if JVET_Z0139_HIST_AFF
    neiIdx[num] = 0;
#endif
    npu[num++] = puLeft;
    return num;
  }

  return num;
}

const int getAvailableAffineNeighboursForAbovePredictor( const PredictionUnit &pu, const PredictionUnit* npu[], int numAffNeighLeft 
#if JVET_Z0139_HIST_AFF
  , int neiIdx[]
#endif
)
{
  const Position posLT = pu.Y().topLeft();
  const Position posRT = pu.Y().topRight();
  const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
  int num = numAffNeighLeft;

  const PredictionUnit* puAboveRight = pu.cs->getPURestricted( posRT.offset( 1, -1 ), pu, pu.chType );
  if (puAboveRight && puAboveRight->cu->affine && puAboveRight->mergeType == MRG_TYPE_DEFAULT_N
      && PU::isDiffMER(pu.lumaPos(), posRT.offset(1, -1), plevel))
  {
#if JVET_Z0139_HIST_AFF
    neiIdx[num] = 2;
#endif
    npu[num++] = puAboveRight;
    return num;
  }

  const PredictionUnit* puAbove = pu.cs->getPURestricted( posRT.offset( 0, -1 ), pu, pu.chType );
  if (puAbove && puAbove->cu->affine && puAbove->mergeType == MRG_TYPE_DEFAULT_N
      && PU::isDiffMER(pu.lumaPos(), posRT.offset(0, -1), plevel))
  {
#if JVET_Z0139_HIST_AFF
    neiIdx[num] = 1;
#endif
    npu[num++] = puAbove;
    return num;
  }

  const PredictionUnit *puAboveLeft = pu.cs->getPURestricted( posLT.offset( -1, -1 ), pu, pu.chType );
  if (puAboveLeft && puAboveLeft->cu->affine && puAboveLeft->mergeType == MRG_TYPE_DEFAULT_N
      && PU::isDiffMER(pu.lumaPos(), posLT.offset(-1, -1), plevel))
  {
#if JVET_Z0139_HIST_AFF
    neiIdx[num] = 4;
#endif
    npu[num++] = puAboveLeft;
    return num;
  }

  return num;
}
void PU::getAffineMergeCand( const PredictionUnit &pu, AffineMergeCtx& affMrgCtx,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION   
  MergeCtx mrgCtxIn[2],
#endif
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION && JVET_W0090_ARMC_TM
  InterPrediction* m_pcInterSearch,
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
  AffineMergeCtx affineRMVFCtx,
  AffineMergeCtx affineRMVFOriCtx,
  uint16_t numCandtoAdd,
#endif
#if !AFFINE_MMVD
                             const int mrgCandIdx
#else
                                   int mrgCandIdx, bool isAfMmvd
#endif
#if JVET_Z0139_NA_AFF && JVET_W0090_ARMC_TM
                           ,       bool isZeroCandIdx
#endif
#if JVET_AG0164_AFFINE_GPM
                          , bool noSbTMVP
#endif
)
{
  const CodingStructure &cs = *pu.cs;
  const Slice &slice = *pu.cs->slice;
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
#if JVET_W0090_ARMC_TM
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
  const uint32_t maxNumAffineMergeCand = slice.getPicHeader()->getMaxNumAffineMergeCand() + ((pu.cs->sps->getUseAML()&& pu.cs->sps->getTMToolsEnableFlag()) ? ADDITIONAL_AFFINE_CAND_NUM : 0);
#else
  const uint32_t maxNumAffineMergeCand = slice.getPicHeader()->getMaxNumAffineMergeCand() + (pu.cs->sps->getUseAML() ? ADDITIONAL_AFFINE_CAND_NUM : 0);
#endif
#else
  const uint32_t maxNumAffineMergeCand = slice.getPicHeader()->getMaxNumAffineMergeCand();
#endif
#else
  const uint32_t maxNumAffineMergeCand = slice.getPicHeader()->getMaxNumAffineMergeCand();
#endif
  const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
#if JVET_Z0118_GDR
  bool isClean = pu.cs->isClean(pu.cu->Y().bottomRight(), CHANNEL_TYPE_LUMA);
#endif
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
  for (int i = 0; i < RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE; i++)
#else
  for (int i = 0; i < maxNumAffineMergeCand; i++)
#endif
  {
    for ( int mvNum = 0; mvNum < 3; mvNum++ )
    {
      affMrgCtx.mvFieldNeighbours[(i << 1) + 0][mvNum].setMvField( Mv(), -1 );
      affMrgCtx.mvFieldNeighbours[(i << 1) + 1][mvNum].setMvField( Mv(), -1 );
    }
    affMrgCtx.interDirNeighbours[i] = 0;
    affMrgCtx.affineType[i] = AFFINEMODEL_4PARAM;
    affMrgCtx.mergeType[i] = MRG_TYPE_DEFAULT_N;
    affMrgCtx.bcwIdx[i] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
    affMrgCtx.altLMFlag[i] = false;
    affMrgCtx.altLMParaNeighbours[i].resetAltLinearModel();
#endif
#if INTER_LIC
    affMrgCtx.licFlags[i] = false;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
    affMrgCtx.obmcFlags[i] = true;
#endif
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
    affMrgCtx.numAffCandToTestEnc = maxNumAffineMergeCand;
    affMrgCtx.candCost[i] = MAX_UINT64;
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    affMrgCtx.colIdx[i] = 0;
#endif
  }

  affMrgCtx.numValidMergeCand = 0;
  affMrgCtx.maxNumMergeCand = maxNumAffineMergeCand;

#if JVET_AG0135_AFFINE_CIIP
  bool sbTmvpEnableFlag = slice.getSPS()->getSbTMVPEnabledFlag() && ((pu.cu->slice->getTLayer() > 2 && PU::checkAffineTMCondition(pu)) || !PU::checkAffineTMCondition(pu))
#else
  bool sbTmvpEnableFlag = slice.getSPS()->getSbTMVPEnabledFlag()
#endif
                          && !(slice.getPOC() == slice.getRefPic(REF_PIC_LIST_0, 0)->getPOC() && slice.isIRAP());

#if JVET_AG0164_AFFINE_GPM
  if(noSbTMVP)
  {
    sbTmvpEnableFlag = false;
  }
#endif
  bool isAvailableSubPu = false;
  if (sbTmvpEnableFlag && slice.getPicHeader()->getEnableTMVPFlag())
  {
    MergeCtx mrgCtx = *affMrgCtx.mrgCtx;
    bool tmpLICFlag = false;

#if JVET_Z0139_NA_AFF && JVET_W0090_ARMC_TM
#if !JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
    if (cs.pcv->isEncoder || isZeroCandIdx)
#endif
    {
#endif    
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
      for (int i = 0; i < SUB_TMVP_NUM; i++)
      {
        CHECK(mrgCtx.subPuMvpMiBuf[i].area() == 0 || !mrgCtx.subPuMvpMiBuf[i].buf, "Buffer not initialized");
        mrgCtx.subPuMvpMiBuf[i].fill(MotionInfo());
      }
#else
      CHECK(mrgCtx.subPuMvpMiBuf.area() == 0 || !mrgCtx.subPuMvpMiBuf.buf, "Buffer not initialized");
      mrgCtx.subPuMvpMiBuf.fill(MotionInfo());
#endif

#if JVET_Z0139_NA_AFF && JVET_W0090_ARMC_TM
    }
#endif

    int pos = 0;
#if !JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    // Get spatial MV
    const Position posCurLB = pu.Y().bottomLeft();
    MotionInfo miLeft;

    //left
    const PredictionUnit* puLeft = cs.getPURestricted(posCurLB.offset(-1, 0), pu, pu.chType);
#if JVET_Y0065_GPM_INTRA
    const bool isAvailableA1 = puLeft && isDiffMER(pu.lumaPos(), posCurLB.offset(-1, 0), plevel) && pu.cu != puLeft->cu && CU::isInter(*puLeft->cu) && puLeft->getMotionInfo(posCurLB.offset(-1, 0)).isInter;
#else
    const bool isAvailableA1 = puLeft && isDiffMER(pu.lumaPos(), posCurLB.offset(-1, 0), plevel) && pu.cu != puLeft->cu && CU::isInter(*puLeft->cu);
#endif
    if (isAvailableA1)
    {
#if INTER_LIC
      affMrgCtx.mrgCtx->licFlags[pos] = false;
#endif

      miLeft = puLeft->getMotionInfo(posCurLB.offset(-1, 0));
      // get Inter Dir
      mrgCtx.interDirNeighbours[pos] = miLeft.interDir;

      // get Mv from Left
      mrgCtx.mvFieldNeighbours[pos << 1].setMvField(miLeft.mv[0], miLeft.refIdx[0]);

      if (slice.isInterB())
      {
        mrgCtx.mvFieldNeighbours[(pos << 1) + 1].setMvField(miLeft.mv[1], miLeft.refIdx[1]);
      }
      pos++;
    }

    mrgCtx.numValidMergeCand = pos;
#endif

#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    int poc0 = pu.cu->slice->getRefPic(RefPicList(1 - pu.cu->slice->getColFromL0Flag()), pu.cu->slice->getColRefIdx())->getPOC();
    int poc1 = pu.cu->slice->getRefPic(RefPicList(1 - pu.cu->slice->getColFromL0Flag2nd()), pu.cu->slice->getColRefIdx2nd())->getPOC();
    for (int index = 0; index < SUB_TMVP_INDEX; index++)
    {
      for (int colFrameIdx = 0; colFrameIdx < ((pu.cu->slice->getCheckLDC() || (poc0 == poc1)) ? 1 : 2); colFrameIdx++)
      {
        if (index > 0 && colFrameIdx == 1)
        {
          continue;
        }
#endif
#if JVET_Z0139_NA_AFF && JVET_W0090_ARMC_TM
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION && JVET_AF0163_TM_SUBBLOCK_REFINEMENT
        isAvailableSubPu = getInterMergeSubPuMvpCand(pu, mrgCtx, tmpLICFlag, 0, false, index, mrgCtxIn[colFrameIdx], colFrameIdx);
#elif JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
        isAvailableSubPu = getInterMergeSubPuMvpCand(pu, mrgCtx, tmpLICFlag, 0, index, mrgCtxIn[colFrameIdx], colFrameIdx);
#else
        isAvailableSubPu = getInterMergeSubPuMvpCand(pu, mrgCtx, tmpLICFlag, pos, (!isZeroCandIdx && !cs.pcv->isEncoder) ? 1 : 0);
#endif
#else
        isAvailableSubPu = getInterMergeSubPuMvpCand(pu, mrgCtx, tmpLICFlag, pos
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
          , colFrameIdx
#else
          , 0
#endif         
        );
#endif
        if (isAvailableSubPu)
        {
          for (int mvNum = 0; mvNum < 3; mvNum++)
          {
            affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][mvNum].setMvField(mrgCtx.mvFieldNeighbours[(pos << 1) + 0].mv, mrgCtx.mvFieldNeighbours[(pos << 1) + 0].refIdx);
            affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][mvNum].setMvField(mrgCtx.mvFieldNeighbours[(pos << 1) + 1].mv, mrgCtx.mvFieldNeighbours[(pos << 1) + 1].refIdx);
          }
          affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = mrgCtx.interDirNeighbours[pos];

          affMrgCtx.affineType[affMrgCtx.numValidMergeCand] = AFFINE_MODEL_NUM;
          affMrgCtx.mergeType[affMrgCtx.numValidMergeCand] = MRG_TYPE_SUBPU_ATMVP;
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
          affMrgCtx.colIdx[affMrgCtx.numValidMergeCand] = (index << 1) + colFrameIdx;
#endif
#if AFFINE_MMVD
          mrgCandIdx += (isAfMmvd && mrgCandIdx >= 0 ? 1 : 0);
#endif
          if (affMrgCtx.numValidMergeCand == mrgCandIdx)
          {
            return;
          }

          affMrgCtx.numValidMergeCand++;

#if JVET_Z0139_NA_AFF && JVET_W0090_ARMC_TM
          if (isZeroCandIdx && !cs.pcv->isEncoder)
          {
            return;
          }
#endif
          // early termination
          if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
          {
            return;
          }
        }
#if JVET_AF0163_TM_SUBBLOCK_REFINEMENT && JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
        if (colFrameIdx == 0 && index == 0 && pu.cs->slice->getSPS()->getTMToolsEnableFlag())
        {
          PredictionUnit puTemp = pu;

          puTemp.cu->bcwIdx = mrgCtxIn[colFrameIdx].bcwIdx[index];
#if INTER_LIC
          puTemp.cu->licFlag = mrgCtxIn[colFrameIdx].licFlags[index];
#endif
          puTemp.mv[0] = mrgCtxIn[colFrameIdx].mvFieldNeighbours[(index << 1) + 0].mv;
          puTemp.mv[1] = mrgCtxIn[colFrameIdx].mvFieldNeighbours[(index << 1) + 1].mv;
          puTemp.refIdx[0] = mrgCtxIn[colFrameIdx].mvFieldNeighbours[(index << 1) + 0].refIdx;
          puTemp.refIdx[1] = mrgCtxIn[colFrameIdx].mvFieldNeighbours[(index << 1) + 1].refIdx;
          puTemp.interDir = mrgCtxIn[colFrameIdx].interDirNeighbours[index];
          m_pcInterSearch->deriveSubTmvpTMMv2Pel(puTemp, 2);
          mrgCtxIn[colFrameIdx].mvFieldNeighbours[(index << 1) + 0].mv = puTemp.mv[0];
          mrgCtxIn[colFrameIdx].mvFieldNeighbours[(index << 1) + 1].mv = puTemp.mv[1];

          isAvailableSubPu = getInterMergeSubPuMvpCand(pu, mrgCtx, tmpLICFlag, 0, true, index, mrgCtxIn[colFrameIdx], colFrameIdx);
          if (isAvailableSubPu)
          {
            for (int mvNum = 0; mvNum < 3; mvNum++)
            {
              affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][mvNum].setMvField(mrgCtx.mvFieldNeighbours[(pos << 1) + 0].mv, mrgCtx.mvFieldNeighbours[(pos << 1) + 0].refIdx);
              affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][mvNum].setMvField(mrgCtx.mvFieldNeighbours[(pos << 1) + 1].mv, mrgCtx.mvFieldNeighbours[(pos << 1) + 1].refIdx);
            }
            affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = mrgCtx.interDirNeighbours[pos];

            affMrgCtx.affineType[affMrgCtx.numValidMergeCand] = AFFINE_MODEL_NUM;
            affMrgCtx.mergeType[affMrgCtx.numValidMergeCand] = MRG_TYPE_SUBPU_ATMVP;
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
            affMrgCtx.colIdx[affMrgCtx.numValidMergeCand] = 3;
#endif
#if AFFINE_MMVD
            mrgCandIdx += (isAfMmvd && mrgCandIdx >= 0 ? 1 : 0);
#endif
            if (affMrgCtx.numValidMergeCand == mrgCandIdx)
            {
              return;
            }

            affMrgCtx.numValidMergeCand++;

#if JVET_Z0139_NA_AFF && JVET_W0090_ARMC_TM
            if (isZeroCandIdx && !cs.pcv->isEncoder)
            {
              return;
            }
#endif
            // early termination
            if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
            {
              return;
            }
          }
        }
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
      }
    }
#endif
  }

  if ( slice.getSPS()->getUseAffine() )
  {
    ///> Start: inherited affine candidates
#if JVET_Z0139_HIST_AFF
    const int CHECKED_NEI_NUM = 7;
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
    const PredictionUnit *npu[CHECKED_NEI_NUM + AFF_MAX_NON_ADJACENT_INHERITED_CANDS + MAX_NUM_AFF_INHERIT_HMVP_CANDS];
#else
    const PredictionUnit* npu[CHECKED_NEI_NUM];
#endif
    const PredictionUnit* npuGroup2[CHECKED_NEI_NUM];
    Position posGroup2[CHECKED_NEI_NUM];
    int numGroup2;
    numGroup2 = 0;
    int neiIdx[CHECKED_NEI_NUM];
    int aiNeibeInherited[CHECKED_NEI_NUM];
    memset(aiNeibeInherited, 0, sizeof(aiNeibeInherited));

    const Position posLB = pu.Y().bottomLeft();
    const Position posLT = pu.Y().topLeft();
    const Position posRT = pu.Y().topRight();

    Position neiPositions[CHECKED_NEI_NUM] = { posLB.offset(-2, -1), posRT.offset(-1, -2), posRT.offset(3, -2), posLB.offset(-2, 3), posLT.offset(-2, -2),
    posLT.offset(-2, 1), posLT.offset(1, -2)};

    int numAffNeighLeft = getAvailableAffineNeighboursForLeftPredictor(pu, npu, neiIdx);
    if (numAffNeighLeft > 0)
    {
      aiNeibeInherited[neiIdx[0]] = 1;
    }
    int numAffNeigh = getAvailableAffineNeighboursForAbovePredictor(pu, npu, numAffNeighLeft, neiIdx);
    if (numAffNeigh > 0)
    {
      aiNeibeInherited[neiIdx[numAffNeigh - 1]] = 1;
    }
    for (int nei = 0; nei < CHECKED_NEI_NUM; nei++)
    {
      const PredictionUnit* puNei = pu.cs->getPURestricted(neiPositions[nei], pu, pu.chType);
      if (aiNeibeInherited[nei])
      {
        continue;
      }
      if (!puNei)
      {
        continue;
      }
      if (puNei->cu->predMode != MODE_INTER)
      {
        continue;
      }
      MotionInfo mvInfo = puNei->getMotionInfo(neiPositions[nei]);
      if (!mvInfo.isInter || mvInfo.interDir <= 0 || mvInfo.interDir > 3)
      {
        continue;
      }
      posGroup2[numGroup2] = neiPositions[nei];
      npuGroup2[numGroup2++] = puNei;
    }
#else
    const PredictionUnit* npu[5];
    int numAffNeighLeft = getAvailableAffineNeighboursForLeftPredictor( pu, npu );
    int numAffNeigh = getAvailableAffineNeighboursForAbovePredictor( pu, npu, numAffNeighLeft );
#endif
    for ( int idx = 0; idx < numAffNeigh; idx++ )
    {
      // derive Mv from Neigh affine PU
      Mv cMv[2][3];
      const PredictionUnit* puNeigh = npu[idx];
      pu.cu->affineType = puNeigh->cu->affineType;
      if ( puNeigh->interDir != 2 )
      {
        xInheritedAffineMv( pu, puNeigh, REF_PIC_LIST_0, cMv[0] );
      }
      if ( slice.isInterB() )
      {
        if ( puNeigh->interDir != 1 )
        {
          xInheritedAffineMv( pu, puNeigh, REF_PIC_LIST_1, cMv[1] );
        }
      }

      for ( int mvNum = 0; mvNum < 3; mvNum++ )
      {
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][mvNum].setMvField( cMv[0][mvNum], puNeigh->refIdx[0] );
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][mvNum].setMvField( cMv[1][mvNum], puNeigh->refIdx[1] );
      }
      affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = puNeigh->interDir;
      affMrgCtx.affineType[affMrgCtx.numValidMergeCand] = (EAffineModel)(puNeigh->cu->affineType);
      affMrgCtx.bcwIdx[affMrgCtx.numValidMergeCand] = puNeigh->cu->bcwIdx;
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
      affMrgCtx.colIdx[affMrgCtx.numValidMergeCand] = 0;
#endif
#if INTER_LIC
#if !JVET_AD0213_LIC_IMP
      CHECK( puNeigh->interDir == 3 && puNeigh->cu->licFlag, "LIC should not be enabled for affine bi-pred" );
#endif
      affMrgCtx.licFlags[affMrgCtx.numValidMergeCand] = puNeigh->cu->licFlag;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
#if JVET_AD0213_LIC_IMP
      affMrgCtx.obmcFlags[affMrgCtx.numValidMergeCand] = puNeigh->cu->affine ? puNeigh->cu->obmcFlag : true;
#else
      affMrgCtx.obmcFlags[affMrgCtx.numValidMergeCand] = puNeigh->cu->affine ? puNeigh->cu->obmcFlag : !puNeigh->cu->licFlag;
#endif
#endif
#if JVET_Z0139_HIST_AFF
      if (!checkLastAffineMergeCandRedundancy(pu, affMrgCtx))
      {
        continue;
      }
#endif
      if ( affMrgCtx.numValidMergeCand == mrgCandIdx )
      {
        affMrgCtx.numValidMergeCand++;
        return;
      }

      // early termination
      affMrgCtx.numValidMergeCand++;
      if ( affMrgCtx.numValidMergeCand == maxNumAffineMergeCand )
      {
        return;
      }
    }

#if JVET_AG0164_AFFINE_GPM
    for (int nIdx = 0; nIdx < numGroup2; nIdx++)
    {
      Mv cMv[2][3];
      PredictionUnit tmpPU = *npuGroup2[nIdx];
      Position       neiPos = posGroup2[nIdx];
      PredictionUnit* puNeigh = &tmpPU;
      if (puNeigh->cu->affine && puNeigh->mergeType == MRG_TYPE_DEFAULT_N)
      {
        pu.cu->affineType = puNeigh->cu->affineType;
      }
      else if (puNeigh->cu->geoFlag)
      {
        MotionInfo mi = puNeigh->getMotionInfo(neiPos);
        int gpmPartIdx = mi.gpmPartIdx;
        CHECK(gpmPartIdx != 0 && gpmPartIdx != 1, "Invalid gpmPartIdx");
        if (puNeigh->affineGPM[gpmPartIdx])
        {
          CHECK(puNeigh->gpmPartAffType[gpmPartIdx] == AFFINE_MODEL_NUM, "Invalid affine type");

          pu.cu->affineType = puNeigh->gpmPartAffType[gpmPartIdx];


          tmpPU.refIdx[0] = puNeigh->gpmPartRefIdx[gpmPartIdx][0];
          tmpPU.refIdx[1] = puNeigh->gpmPartRefIdx[gpmPartIdx][1];

          if (tmpPU.refIdx[0] != -1 && tmpPU.refIdx[1] != -1)
          {
            tmpPU.interDir = 3;
            tmpPU.mvAffi[0][0] = puNeigh->gpmPartmvAffi[gpmPartIdx][0][0];
            tmpPU.mvAffi[0][1] = puNeigh->gpmPartmvAffi[gpmPartIdx][0][1];
            tmpPU.mvAffi[0][2] = puNeigh->gpmPartmvAffi[gpmPartIdx][0][2];
            tmpPU.mvAffi[1][0] = puNeigh->gpmPartmvAffi[gpmPartIdx][1][0];
            tmpPU.mvAffi[1][1] = puNeigh->gpmPartmvAffi[gpmPartIdx][1][1];
            tmpPU.mvAffi[1][2] = puNeigh->gpmPartmvAffi[gpmPartIdx][1][2];
          }
          else if (tmpPU.refIdx[0] != -1)
          {
            tmpPU.interDir = 1;
            tmpPU.mvAffi[0][0] = puNeigh->gpmPartmvAffi[gpmPartIdx][0][0];
            tmpPU.mvAffi[0][1] = puNeigh->gpmPartmvAffi[gpmPartIdx][0][1];
            tmpPU.mvAffi[0][2] = puNeigh->gpmPartmvAffi[gpmPartIdx][0][2];
            tmpPU.mvAffi[1][0].setZero();
            tmpPU.mvAffi[1][1].setZero();
            tmpPU.mvAffi[1][2].setZero();
          }
          else if (tmpPU.refIdx[1] != -1)
          {
            tmpPU.interDir = 2;
            tmpPU.mvAffi[1][0] = puNeigh->gpmPartmvAffi[gpmPartIdx][1][0];
            tmpPU.mvAffi[1][1] = puNeigh->gpmPartmvAffi[gpmPartIdx][1][1];
            tmpPU.mvAffi[1][2] = puNeigh->gpmPartmvAffi[gpmPartIdx][1][2];
            tmpPU.mvAffi[0][0].setZero();
            tmpPU.mvAffi[0][1].setZero();
            tmpPU.mvAffi[0][2].setZero();
          }
          else
          {
            THROW("Invaid case");
          }
        }
        else
        {
          continue;
        }
      }
      else
      {
        continue;
      }
      if (puNeigh->interDir != 2)
      {
        xInheritedAffineMv(pu, puNeigh, REF_PIC_LIST_0, cMv[0]);
      }
      if (slice.isInterB())
      {
        if (puNeigh->interDir != 1)
        {
          xInheritedAffineMv(pu, puNeigh, REF_PIC_LIST_1, cMv[1]);
        }
      }

      for (int mvNum = 0; mvNum < 3; mvNum++)
      {
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][mvNum].setMvField(cMv[0][mvNum], puNeigh->refIdx[0]);
        affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][mvNum].setMvField(cMv[1][mvNum], puNeigh->refIdx[1]);
      }
      affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = puNeigh->interDir;
      affMrgCtx.affineType[affMrgCtx.numValidMergeCand] = (EAffineModel)pu.cu->affineType;
      affMrgCtx.bcwIdx[affMrgCtx.numValidMergeCand] = puNeigh->cu->bcwIdx;
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
      affMrgCtx.colIdx[affMrgCtx.numValidMergeCand] = 0;
#endif
#if INTER_LIC
#if !JVET_AD0213_LIC_IMP
      CHECK(puNeigh->interDir == 3 && puNeigh->cu->licFlag, "LIC should not be enabled for affine bi-pred");
#endif
      affMrgCtx.licFlags[affMrgCtx.numValidMergeCand] = puNeigh->cu->licFlag;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
#if JVET_AD0213_LIC_IMP
      affMrgCtx.obmcFlags[affMrgCtx.numValidMergeCand] = puNeigh->cu->affine ? puNeigh->cu->obmcFlag : true;
#else
      affMrgCtx.obmcFlags[affMrgCtx.numValidMergeCand] = puNeigh->cu->affine ? puNeigh->cu->obmcFlag : !puNeigh->cu->licFlag;
#endif
#endif
#if JVET_Z0139_HIST_AFF
      if (!checkLastAffineMergeCandRedundancy(pu, affMrgCtx))
      {
        continue;
      }
#endif
      if (affMrgCtx.numValidMergeCand == mrgCandIdx)
      {
        affMrgCtx.numValidMergeCand++;
        return;
      }

      // early termination
      affMrgCtx.numValidMergeCand++;
      if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
      {
        return;
      }
    }
#endif

#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
#if JVET_W0090_ARMC_TM
    int numAffNeighExtend2 = numCandtoAdd;
    AffineMergeCtx affMrgCtxTemp;
    for (int i = 0; i < RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE; i++)
    {
      for (int mvNum = 0; mvNum < 3; mvNum++)
      {
        affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 0][mvNum].setMvField(Mv(), -1);
        affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 1][mvNum].setMvField(Mv(), -1);
      }
      affMrgCtxTemp.interDirNeighbours[i] = 0;
      affMrgCtxTemp.affineType[i] = AFFINEMODEL_4PARAM;
      affMrgCtxTemp.mergeType[i] = MRG_TYPE_DEFAULT_N;
      affMrgCtxTemp.bcwIdx[i] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
      affMrgCtxTemp.altLMFlag[i] = false;
      affMrgCtxTemp.altLMParaNeighbours[i].resetAltLinearModel();
#endif
#if INTER_LIC
      affMrgCtxTemp.licFlags[i] = false;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
      affMrgCtxTemp.obmcFlags[i] = true;
#endif
      affMrgCtxTemp.candCost[i] = MAX_UINT64;
    }
    affMrgCtxTemp.numValidMergeCand = 0;
    affMrgCtxTemp.maxNumMergeCand = RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE;
    if (pu.cs->sps->getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
        && pu.cs->sps->getTMToolsEnableFlag()
#endif
      )
    {
      int counter = 0;
      affMrgCtxTemp = affineRMVFCtx;

      Mv cMv[2][3];
      int8_t referenceidx[2];
      for (int i = 0; i < affMrgCtxTemp.numValidMergeCand; i++)
      {
        cMv[0][0] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 0][0].mv;
        cMv[0][1] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 0][1].mv;
        cMv[0][2] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 0][2].mv;
        cMv[1][0] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 1][0].mv;
        cMv[1][1] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 1][1].mv;
        cMv[1][2] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 1][2].mv;
        referenceidx[0] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 0][0].refIdx;
        referenceidx[1] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 1][0].refIdx;
        if (xCPMVSimCheck(pu, affMrgCtx, cMv, affMrgCtxTemp.interDirNeighbours[i], referenceidx, EAffineModel(affMrgCtxTemp.affineType[i]), affMrgCtxTemp.bcwIdx[i], affMrgCtxTemp.licFlags[i]))
        {
          continue;
        }
        counter++;
        for (int mvNum = 0; mvNum < 3; mvNum++)
        {
          affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][mvNum].setMvField(cMv[0][mvNum], referenceidx[0]);
          affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][mvNum].setMvField(cMv[1][mvNum], referenceidx[1]);
        }
        affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = affMrgCtxTemp.interDirNeighbours[i];
        affMrgCtx.affineType[affMrgCtx.numValidMergeCand] = EAffineModel(affMrgCtxTemp.affineType[i]);
        affMrgCtx.mergeType[affMrgCtx.numValidMergeCand] = MRG_TYPE_DEFAULT_N;
        affMrgCtx.bcwIdx[affMrgCtx.numValidMergeCand] = affMrgCtxTemp.bcwIdx[i];
#if JVET_AG0276_NLIC
        affMrgCtx.altLMFlag[affMrgCtx.numValidMergeCand] = affMrgCtxTemp.altLMFlag[i];
        affMrgCtx.altLMParaNeighbours[affMrgCtx.numValidMergeCand] = affMrgCtxTemp.altLMParaNeighbours[i];
#endif
#if INTER_LIC
        affMrgCtx.licFlags[affMrgCtx.numValidMergeCand] = affMrgCtxTemp.licFlags[i];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
        affMrgCtx.obmcFlags[affMrgCtx.numValidMergeCand] = affMrgCtxTemp.obmcFlags[i];
#endif
        affMrgCtx.candCost[affMrgCtx.numValidMergeCand] = affMrgCtxTemp.candCost[i];

        if (affMrgCtx.numValidMergeCand == mrgCandIdx)
        {
          affMrgCtx.numValidMergeCand++;
          return;
        }

        // early termination
        affMrgCtx.numValidMergeCand++;
        if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
        {
          return;
        }
        if (counter >= numAffNeighExtend2)
        {
          break;
        }
      }
    }
    else
    {
#endif
      int counter = 0;
      affMrgCtxTemp = affineRMVFOriCtx;
      Mv cMv[2][3];
      int8_t referenceidx[2];
      for (int i = 0; i < affMrgCtxTemp.numValidMergeCand; i++)
      {
        cMv[0][0] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 0][0].mv;
        cMv[0][1] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 0][1].mv;
        cMv[0][2] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 0][2].mv;
        cMv[1][0] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 1][0].mv;
        cMv[1][1] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 1][1].mv;
        cMv[1][2] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 1][2].mv;
        referenceidx[0] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 0][0].refIdx;
        referenceidx[1] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 1][0].refIdx;
        for (int mvNum = 0; mvNum < 3; mvNum++)
        {
          affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][mvNum].setMvField(cMv[0][mvNum], referenceidx[0]);
          affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][mvNum].setMvField(cMv[1][mvNum], referenceidx[1]);
        }
        affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = affMrgCtxTemp.interDirNeighbours[i];
        affMrgCtx.affineType[affMrgCtx.numValidMergeCand] = EAffineModel(affMrgCtxTemp.affineType[i]);
        affMrgCtx.mergeType[affMrgCtx.numValidMergeCand] = MRG_TYPE_DEFAULT_N;
        affMrgCtx.bcwIdx[affMrgCtx.numValidMergeCand] = affMrgCtxTemp.bcwIdx[i];
#if JVET_AG0276_NLIC
        affMrgCtx.altLMFlag[affMrgCtx.numValidMergeCand] = affMrgCtxTemp.altLMFlag[i];
        affMrgCtx.altLMParaNeighbours[affMrgCtx.numValidMergeCand] = affMrgCtxTemp.altLMParaNeighbours[i];
#endif
#if INTER_LIC
        affMrgCtx.licFlags[affMrgCtx.numValidMergeCand] = affMrgCtxTemp.licFlags[i];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
        affMrgCtx.obmcFlags[affMrgCtx.numValidMergeCand] = affMrgCtxTemp.obmcFlags[i];
#endif
        affMrgCtx.candCost[affMrgCtx.numValidMergeCand] = affMrgCtxTemp.candCost[i];
        counter++;
        if (affMrgCtx.numValidMergeCand == mrgCandIdx)
        {
          affMrgCtx.numValidMergeCand++;
          return;
        }

        // early termination
        affMrgCtx.numValidMergeCand++;
        if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
        {
          return;
        }
        if (counter >= numAffNeighExtend2)
        {
          break;
        }
      }
#if JVET_W0090_ARMC_TM
    }
#endif
#endif
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
#if !JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
    int numAffNeighExtend2 = getNonAdjAvailableAffineNeighboursByDistance(pu, npu, affMrgCtx, 0, -1);
    if (numAffNeighExtend2 > 0)
    {
      AffineMergeCtx affMrgCtxTemp;
      for (int i = 0; i < RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE; i++)
      {
        for (int mvNum = 0; mvNum < 3; mvNum++)
        {
          affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 0][mvNum].setMvField(Mv(), -1);
          affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 1][mvNum].setMvField(Mv(), -1);
        }
        affMrgCtxTemp.interDirNeighbours[i] = 0;
        affMrgCtxTemp.affineType[i] = AFFINEMODEL_4PARAM;
        affMrgCtxTemp.mergeType[i] = MRG_TYPE_DEFAULT_N;
        affMrgCtxTemp.bcwIdx[i] = BCW_DEFAULT;
#if INTER_LIC
        affMrgCtxTemp.licFlags[i] = false;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
        affMrgCtxTemp.obmcFlags[i] = true;
#endif
        affMrgCtxTemp.candCost[i] = MAX_UINT64;
      }
      affMrgCtxTemp.numValidMergeCand = 0;
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
      if (pu.cs->sps->getUseFastSubTmvp())
      {
        affMrgCtxTemp.maxNumMergeCand = 17;
      }
      else
      {
        affMrgCtxTemp.maxNumMergeCand = 16;
      }
#else
      affMrgCtxTemp.maxNumMergeCand = RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE;
#endif

      std::vector<RMVFInfo> mvpInfoVec[NUM_REF_PIC_LIST_01][MAX_NUM_REF];
      collectNeiMotionInfo(mvpInfoVec, pu);

#if JVET_W0090_ARMC_TM
      if (pu.cs->sps->getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
          && pu.cs->sps->getTMToolsEnableFlag()
#endif
        )
      {
        for (int i = 0; i < numAffNeighExtend2; i++)
        {
          getRMVFAffineGuideCand(pu, *npu[i], affMrgCtxTemp, mvpInfoVec);
          if (affMrgCtxTemp.numValidMergeCand == affMrgCtxTemp.maxNumMergeCand)
          {
            break;
          }
        }
        PredictionUnit pu2 = pu;
        m_pcInterSearch->adjustAffineMergeCandidatesOneGroup(pu2, affMrgCtxTemp, affMrgCtxTemp.numValidMergeCand);
        int counter = 0;

        Mv cMv[2][3];
        int8_t referenceidx[2];
        for (int i = 0; i < affMrgCtxTemp.numValidMergeCand; i++)
        {
          cMv[0][0] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 0][0].mv;
          cMv[0][1] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 0][1].mv;
          cMv[0][2] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 0][2].mv;
          cMv[1][0] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 1][0].mv;
          cMv[1][1] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 1][1].mv;
          cMv[1][2] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 1][2].mv;
          referenceidx[0] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 0][0].refIdx;
          referenceidx[1] = affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 1][0].refIdx;
          if (xCPMVSimCheck(pu, affMrgCtx, cMv, affMrgCtxTemp.interDirNeighbours[i], referenceidx, EAffineModel(affMrgCtxTemp.affineType[i]), affMrgCtxTemp.bcwIdx[i], affMrgCtxTemp.licFlags[i]))
          {
            continue;
          }
          counter++;
          for (int mvNum = 0; mvNum < 3; mvNum++)
          {
            affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][mvNum].setMvField(cMv[0][mvNum], referenceidx[0]);
            affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][mvNum].setMvField(cMv[1][mvNum], referenceidx[1]);
          }
          affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = affMrgCtxTemp.interDirNeighbours[i];
          affMrgCtx.affineType[affMrgCtx.numValidMergeCand] = EAffineModel(affMrgCtxTemp.affineType[i]);
          affMrgCtx.mergeType[affMrgCtx.numValidMergeCand] = MRG_TYPE_DEFAULT_N;
          affMrgCtx.bcwIdx[affMrgCtx.numValidMergeCand] = affMrgCtxTemp.bcwIdx[i];
#if INTER_LIC
          affMrgCtx.licFlags[affMrgCtx.numValidMergeCand] = affMrgCtxTemp.licFlags[i];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
          affMrgCtx.obmcFlags[affMrgCtx.numValidMergeCand] = affMrgCtxTemp.obmcFlags[i];
#endif
          affMrgCtx.candCost[affMrgCtx.numValidMergeCand] = affMrgCtxTemp.candCost[i];

          if (affMrgCtx.numValidMergeCand == mrgCandIdx)
          {
            affMrgCtx.numValidMergeCand++;
            return;
          }

          // early termination
          affMrgCtx.numValidMergeCand++;
          if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
          {
            return;
          }
          if (counter >= numAffNeighExtend2)
          {
            break;
          }
        }
      }
      else
      {
#endif
        int counter = 0;
        int numAdded = 0;
        for (int i = 0; i < numAffNeighExtend2; i++)
        {
          numAdded = affMrgCtx.numValidMergeCand;
          getRMVFAffineGuideCand(pu, *npu[i], affMrgCtx, mvpInfoVec, mrgCandIdx);
          if (affMrgCtx.numValidMergeCand != 0 && (affMrgCtx.numValidMergeCand - 1 == mrgCandIdx))
          {
            return;
          }
          if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
          {
            return;
          }
          counter += affMrgCtx.numValidMergeCand - numAdded;
          if (counter >= numAffNeighExtend2)
          {
            break;
          }
        }
#if JVET_W0090_ARMC_TM
      }
#endif
    }
#endif
#endif
#if JVET_Z0139_HIST_AFF
    MotionInfo tmvpInfo;
    tmvpInfo.interDir = 0;
    tmvpInfo.refIdx[0] = tmvpInfo.refIdx[1] = -1;
    bool       isTmvpAvailable = false;
#endif
    ///> End: inherited affine candidates

    ///> Start: Constructed affine candidates
    {
      MotionInfo mi[4];
      bool isAvailable[4] = { false };

      int8_t neighBcw[2] = { BCW_DEFAULT, BCW_DEFAULT };
      // control point: LT B2->B3->A2
      const Position posLT[3] = { pu.Y().topLeft().offset( -1, -1 ), pu.Y().topLeft().offset( 0, -1 ), pu.Y().topLeft().offset( -1, 0 ) };
      for ( int i = 0; i < 3; i++ )
      {
        const Position pos = posLT[i];
        const PredictionUnit* puNeigh = cs.getPURestricted( pos, pu, pu.chType );

#if JVET_Y0065_GPM_INTRA
        if (puNeigh && CU::isInter(*puNeigh->cu) && puNeigh->getMotionInfo( pos ).isInter && PU::isDiffMER(pu.lumaPos(), pos, plevel))
#else
        if (puNeigh && CU::isInter(*puNeigh->cu) && PU::isDiffMER(pu.lumaPos(), pos, plevel))
#endif
        {
          isAvailable[0] = true;
          mi[0] = puNeigh->getMotionInfo( pos );
          neighBcw[0] = puNeigh->cu->bcwIdx;
#if JVET_Z0139_HIST_AFF
          if (puNeigh->interDir != 3) neighBcw[0] = BCW_DEFAULT;
#endif
          break;
        }
      }

      // control point: RT B1->B0
      const Position posRT[2] = { pu.Y().topRight().offset( 0, -1 ), pu.Y().topRight().offset( 1, -1 ) };
      for ( int i = 0; i < 2; i++ )
      {
        const Position pos = posRT[i];
        const PredictionUnit* puNeigh = cs.getPURestricted( pos, pu, pu.chType );

#if JVET_Y0065_GPM_INTRA
        if (puNeigh && CU::isInter(*puNeigh->cu) && puNeigh->getMotionInfo( pos ).isInter && PU::isDiffMER(pu.lumaPos(), pos, plevel))
#else
        if (puNeigh && CU::isInter(*puNeigh->cu) && PU::isDiffMER(pu.lumaPos(), pos, plevel))
#endif
        {
          isAvailable[1] = true;
          mi[1] = puNeigh->getMotionInfo( pos );
          neighBcw[1] = puNeigh->cu->bcwIdx;
#if JVET_Z0139_HIST_AFF
          if (puNeigh->interDir != 3) neighBcw[1] = BCW_DEFAULT;
#endif
          break;
        }
      }

      // control point: LB A1->A0
      const Position posLB[2] = { pu.Y().bottomLeft().offset( -1, 0 ), pu.Y().bottomLeft().offset( -1, 1 ) };
      for ( int i = 0; i < 2; i++ )
      {
        const Position pos = posLB[i];
        const PredictionUnit* puNeigh = cs.getPURestricted( pos, pu, pu.chType );

#if JVET_Y0065_GPM_INTRA
        if (puNeigh && CU::isInter(*puNeigh->cu) && puNeigh->getMotionInfo( pos ).isInter && PU::isDiffMER(pu.lumaPos(), pos, plevel))
#else
        if (puNeigh && CU::isInter(*puNeigh->cu) && PU::isDiffMER(pu.lumaPos(), pos, plevel))
#endif
        {
          isAvailable[2] = true;
          mi[2] = puNeigh->getMotionInfo( pos );
          break;
        }
      }

      // control point: RB
      if ( slice.getPicHeader()->getEnableTMVPFlag() )
      {
        //>> MTK colocated-RightBottom
        // offset the pos to be sure to "point" to the same position the uiAbsPartIdx would've pointed to
        Position posRB = pu.Y().bottomRight().offset( -3, -3 );

        const PreCalcValues& pcv = *cs.pcv;
        Position posC0;
        bool isC0Avail = false;

        bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
        const SubPic &curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
        if (curSubPic.getTreatedAsPicFlag())
        {
          boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
            (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom());
        }
        if (boundaryCond)
        {
          int posYInCtu = posRB.y & pcv.maxCUHeightMask;
          if (posYInCtu + 4 < pcv.maxCUHeight)
          {
            posC0 = posRB.offset(4, 4);
            isC0Avail = true;
          }
        }

        Mv        cColMv;
        int       refIdx = 0;
        bool      bExistMV = isC0Avail && getColocatedMVP(pu, REF_PIC_LIST_0, posC0, cColMv, refIdx, false
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
          , 0
#endif
          , &refIdx
#endif
        );
        if ( bExistMV )
        {
          mi[3].mv[0] = cColMv;
          mi[3].refIdx[0] = refIdx;
          mi[3].interDir = 1;
          isAvailable[3] = true;
#if JVET_Z0139_HIST_AFF
          tmvpInfo.mv[0] = cColMv;
          tmvpInfo.refIdx[0] = refIdx;
          tmvpInfo.interDir |= 1;
          isTmvpAvailable = true;
#endif
        }

        if ( slice.isInterB() )
        {
          bExistMV = isC0Avail && getColocatedMVP(pu, REF_PIC_LIST_1, posC0, cColMv, refIdx, false
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
            , 0
#endif
            , &refIdx
#endif
          );
          if ( bExistMV )
          {
            mi[3].mv[1] = cColMv;
            mi[3].refIdx[1] = refIdx;
            mi[3].interDir |= 2;
            isAvailable[3] = true;
#if JVET_Z0139_HIST_AFF
            tmvpInfo.mv[1] = cColMv;
            tmvpInfo.refIdx[1] = refIdx;
            tmvpInfo.interDir |= 2;
            isTmvpAvailable = true;
#endif
          }
        }
#if INTER_LIC
        mi[3].usesLIC = false;
#endif
      }

      //-------------------  insert model  -------------------//
      int order[6] = { 0, 1, 2, 3, 4, 5 };
      int modelNum = 6;
      int model[6][4] = {
        { 0, 1, 2 },          // 0:  LT, RT, LB
        { 0, 1, 3 },          // 1:  LT, RT, RB
        { 0, 2, 3 },          // 2:  LT, LB, RB
        { 1, 2, 3 },          // 3:  RT, LB, RB
        { 0, 1 },             // 4:  LT, RT
        { 0, 2 },             // 5:  LT, LB
      };

      int verNum[6] = { 3, 3, 3, 3, 2, 2 };
      int startIdx = pu.cs->sps->getUseAffineType() ? 0 : 4;
      for ( int idx = startIdx; idx < modelNum; idx++ )
      {
        int modelIdx = order[idx];
#if JVET_Z0139_HIST_AFF
        if (getAffineControlPointCand(pu, mi, isAvailable, model[modelIdx], ((modelIdx == 3) ? neighBcw[1] : neighBcw[0]), modelIdx, verNum[modelIdx], affMrgCtx))
        {
          if (affMrgCtx.numValidMergeCand == mrgCandIdx)
          {
            affMrgCtx.numValidMergeCand++;
            return;
          }
          affMrgCtx.numValidMergeCand ++;
          if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
          {
            return;
          }
        }
#else
        getAffineControlPointCand(pu, mi, isAvailable, model[modelIdx], ((modelIdx == 3) ? neighBcw[1] : neighBcw[0]), modelIdx, verNum[modelIdx], affMrgCtx);
        if ( affMrgCtx.numValidMergeCand != 0 && affMrgCtx.numValidMergeCand - 1 == mrgCandIdx )
        {
          return;
        }

        // early termination
        if ( affMrgCtx.numValidMergeCand == maxNumAffineMergeCand )
        {
          return;
        }
#endif
#if JVET_Z0139_HIST_AFF
        if (idx == startIdx)
        {
#if JVET_Z0118_GDR
          if (addSpatialAffineMergeHMVPCand(pu, affMrgCtx, (isClean) ? pu.cs->motionLut.lutAff1 : pu.cs->motionLut.lutAff0, 0, npuGroup2, posGroup2, numGroup2, mrgCandIdx))
#else
          if (addSpatialAffineMergeHMVPCand(pu, affMrgCtx, pu.cs->motionLut.lutAff, 0, npuGroup2, posGroup2, numGroup2, mrgCandIdx))
#endif
          {
            return;
          }

#if JVET_Z0118_GDR
          if (addOneInheritedHMVPAffineMergeCand(pu, affMrgCtx, (isClean) ? pu.cs->motionLut.lutAffInherit1 : pu.cs->motionLut.lutAffInherit0, 0))
#else
          if (addOneInheritedHMVPAffineMergeCand(pu, affMrgCtx, pu.cs->motionLut.lutAffInherit, 0))
#endif
          {
            if (affMrgCtx.numValidMergeCand == mrgCandIdx) // for decoder 
            {
              affMrgCtx.numValidMergeCand++;
              return;
            }

            affMrgCtx.numValidMergeCand++;

            // early termination
            if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
            {
              return;
            }
          }
        }
#endif
#if JVET_Z0139_NA_AFF
        if(!((pu.cu->slice->getPOC() - pu.cu->slice->getRefPOC( REF_PIC_LIST_0, 0)) == 1 && pu.cu->slice->getPicHeader()->getMvdL1ZeroFlag()))
        {
          if (idx == startIdx)
          {
            getNonAdjCstMergeCand(pu, affMrgCtx, mrgCandIdx, true);
            if (affMrgCtx.numValidMergeCand != 0 && affMrgCtx.numValidMergeCand - 1 == mrgCandIdx)
            {
              return;
            }

            if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
            {
              return;
            }
          }
        }
#endif
      }

#if JVET_Z0139_NA_AFF
      if((pu.cu->slice->getPOC() - pu.cu->slice->getRefPOC( REF_PIC_LIST_0, 0)) == 1 && pu.cu->slice->getPicHeader()->getMvdL1ZeroFlag())
      {
        getNonAdjCstMergeCand(pu, affMrgCtx, mrgCandIdx, true);
        if (affMrgCtx.numValidMergeCand != 0 && affMrgCtx.numValidMergeCand - 1 == mrgCandIdx)
        {
          return;
        }

        if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
        {
          return;
        }
      }
#endif
    }
    ///> End: Constructed affine candidates
#if JVET_Z0139_HIST_AFF 
    if (isTmvpAvailable)
    {
      Position posRB = pu.Y().bottomRight().offset(3, 3);
      if (addOneAffineMergeHMVPCand(pu, affMrgCtx
#if JVET_Z0118_GDR
                                 , (isClean) ? pu.cs->motionLut.lutAff1 : pu.cs->motionLut.lutAff0
#else
                                 , pu.cs->motionLut.lutAff
#endif
                                 , 0, tmvpInfo, posRB, BCW_DEFAULT
#if INTER_LIC
                                 , false
#endif
      ))
      {
        if (affMrgCtx.numValidMergeCand == mrgCandIdx) // for decoder 
        {
          affMrgCtx.numValidMergeCand++;
          return;
        }

        affMrgCtx.numValidMergeCand++;

        // early termination
        if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
        {
          return;
        }
      }
    }
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION && JVET_W0090_ARMC_TM
    if (pu.cs->sps->getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
        && pu.cs->sps->getTMToolsEnableFlag()
#endif
      )
    {
      affMrgCtx.numAffCandToTestEnc = affMrgCtx.numValidMergeCand;
    }
#endif
    for (int iAffListIdx = 1; iAffListIdx < MAX_NUM_AFF_HMVP_CANDS; iAffListIdx++)
    {
#if JVET_Z0118_GDR
      if (addSpatialAffineMergeHMVPCand(pu, affMrgCtx, (isClean) ? pu.cs->motionLut.lutAff1 : pu.cs->motionLut.lutAff0, iAffListIdx, npuGroup2, posGroup2, numGroup2, mrgCandIdx))
#else
      if (addSpatialAffineMergeHMVPCand(pu, affMrgCtx, pu.cs->motionLut.lutAff, iAffListIdx, npuGroup2, posGroup2, numGroup2, mrgCandIdx))
#endif
      {
        return;
      }
      if (isTmvpAvailable)
      {
        Position posRB = pu.Y().bottomRight().offset(3, 3);
        if (addOneAffineMergeHMVPCand(pu, affMrgCtx
#if JVET_Z0118_GDR
                                    , (isClean) ? pu.cs->motionLut.lutAff1 : pu.cs->motionLut.lutAff0
#else
                                    , pu.cs->motionLut.lutAff
#endif
                                    , iAffListIdx, tmvpInfo, posRB, BCW_DEFAULT
#if INTER_LIC
                                    , false
#endif
        ))
        {
          if (affMrgCtx.numValidMergeCand == mrgCandIdx) // for decoder 
          {
            affMrgCtx.numValidMergeCand++;
            return;
          }

          affMrgCtx.numValidMergeCand++;

          // early termination
          if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
          {
            return;
          }
        }
      }
    }

    for (int iAffListIdx = 1; iAffListIdx < MAX_NUM_AFF_INHERIT_HMVP_CANDS; iAffListIdx++)
    {
#if JVET_Z0118_GDR
      if (addOneInheritedHMVPAffineMergeCand(pu, affMrgCtx, (isClean) ? pu.cs->motionLut.lutAffInherit1 : pu.cs->motionLut.lutAffInherit0, iAffListIdx))
#else
      if (addOneInheritedHMVPAffineMergeCand(pu, affMrgCtx, pu.cs->motionLut.lutAffInherit, iAffListIdx))
#endif
      {
        if (affMrgCtx.numValidMergeCand == mrgCandIdx) // for decoder 
        {
          affMrgCtx.numValidMergeCand++;
          return;
        }

        affMrgCtx.numValidMergeCand++;

        // early termination
        if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
        {
          return;
        }
      }
    }
    const int MAX_PAIRWISE_NUM = 9;
    const int preDefinedPairs[MAX_PAIRWISE_NUM][2] = { {0, 1}, {0, 2}, {1, 2}, {0, 3}, {1, 3}, { 2, 3}, { 0, 4}, {1, 4}, { 2, 4} };
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    int iATMVPoffset = 0;
    for (int index = 0; index < affMrgCtx.numValidMergeCand; index++)
    {
      if (affMrgCtx.mergeType[index] == MRG_TYPE_SUBPU_ATMVP)
      {
        iATMVPoffset++;
      }
    }
#else
    int iATMVPoffset = affMrgCtx.mergeType[0] == MRG_TYPE_SUBPU_ATMVP ? 1 : 0;
#endif
    int currSize = affMrgCtx.numValidMergeCand;
    for (int pairIdx = 0; pairIdx < MAX_PAIRWISE_NUM; pairIdx++)
    {
      int idx0 = preDefinedPairs[pairIdx][0] + iATMVPoffset;
      int idx1 = preDefinedPairs[pairIdx][1] + iATMVPoffset;

      CHECK(affMrgCtx.mergeType[idx0] == MRG_TYPE_SUBPU_ATMVP || affMrgCtx.mergeType[idx1] == MRG_TYPE_SUBPU_ATMVP, "Invalid Index");

      if (idx0 >= currSize || idx1 >= currSize)
      {
        break;
      }
      if ((affMrgCtx.interDirNeighbours[idx0] & affMrgCtx.interDirNeighbours[idx1]) == 0)
      {
        continue;
      }

      affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] = 0;

      affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][0].setMvField(Mv(), -1);
      affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][1].setMvField(Mv(), -1);
      affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][2].setMvField(Mv(), -1);
      affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][0].setMvField(Mv(), -1);
      affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][1].setMvField(Mv(), -1);
      affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][2].setMvField(Mv(), -1);

      affMrgCtx.bcwIdx[affMrgCtx.numValidMergeCand] = BCW_DEFAULT;

#if INTER_LIC
      affMrgCtx.licFlags[affMrgCtx.numValidMergeCand] = false;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
      affMrgCtx.obmcFlags[affMrgCtx.numValidMergeCand] = true;
#endif

      affMrgCtx.affineType[affMrgCtx.numValidMergeCand] = AFFINEMODEL_6PARAM;

      int numCPMVs = 3;

      if (affMrgCtx.mvFieldNeighbours[(idx0 << 1)][0].refIdx != -1 && affMrgCtx.mvFieldNeighbours[(idx1 << 1)][0].refIdx == affMrgCtx.mvFieldNeighbours[(idx0 << 1)][0].refIdx)
      {
        affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] |= 1;

        for (int mvIdx = 0; mvIdx < numCPMVs; mvIdx++)
        {
          Mv avgMv = affMrgCtx.mvFieldNeighbours[(idx0 << 1)][mvIdx].mv;
          avgMv += affMrgCtx.mvFieldNeighbours[(idx1 << 1)][mvIdx].mv;
          roundAffineMv(avgMv.hor, avgMv.ver, 1);
          affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 0][mvIdx].setMvField(avgMv, affMrgCtx.mvFieldNeighbours[(idx0 << 1)][0].refIdx);
        }
      }
      if (affMrgCtx.mvFieldNeighbours[(idx0 << 1) + 1][0].refIdx != -1 && affMrgCtx.mvFieldNeighbours[(idx1 << 1) + 1][0].refIdx == affMrgCtx.mvFieldNeighbours[(idx0 << 1) + 1][0].refIdx)
      {
        affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] |= 2;

        for (int mvIdx = 0; mvIdx < numCPMVs; mvIdx++)
        {
          Mv avgMv = affMrgCtx.mvFieldNeighbours[(idx0 << 1) + 1][mvIdx].mv;
          avgMv += affMrgCtx.mvFieldNeighbours[(idx1 << 1) + 1][mvIdx].mv;
          roundAffineMv(avgMv.hor, avgMv.ver, 1);
          affMrgCtx.mvFieldNeighbours[(affMrgCtx.numValidMergeCand << 1) + 1][mvIdx].setMvField(avgMv, affMrgCtx.mvFieldNeighbours[(idx0 << 1) + 1][0].refIdx);
        }
      }

      if (affMrgCtx.interDirNeighbours[affMrgCtx.numValidMergeCand] == 0)
      {
        continue;
      }
      if (checkLastAffineMergeCandRedundancy(pu, affMrgCtx))
      {
        if (affMrgCtx.numValidMergeCand == mrgCandIdx) // for decoder 
        {
          affMrgCtx.numValidMergeCand++;
          return;
        }

        affMrgCtx.numValidMergeCand++;

        // early termination
        if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
        {
          return;
        }
      }
    }
#endif
  }

  ///> zero padding
  int cnt = affMrgCtx.numValidMergeCand;
  while ( cnt < maxNumAffineMergeCand )
  {
    for ( int mvNum = 0; mvNum < 3; mvNum++ )
    {
      affMrgCtx.mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField( Mv( 0, 0 ), 0 );
    }
    affMrgCtx.interDirNeighbours[cnt] = 1;
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    affMrgCtx.colIdx[cnt] = 0;
#endif

    if ( slice.isInterB() )
    {
      for ( int mvNum = 0; mvNum < 3; mvNum++ )
      {
        affMrgCtx.mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField( Mv( 0, 0 ), 0 );
      }
      affMrgCtx.interDirNeighbours[cnt] = 3;
    }
    affMrgCtx.affineType[cnt] = AFFINEMODEL_4PARAM;
    if ( cnt == mrgCandIdx )
    {
      affMrgCtx.numValidMergeCand++;
      return;
    }
    cnt++;
    affMrgCtx.numValidMergeCand++;
  }
}
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
bool PU::isAffBMMergeFlagCoded(const PredictionUnit& pu)
{
  if (pu.cs->slice->getSPS()->getUseDMVDMode()
#if JVET_Y0128_NON_CTC
    && pu.cs->slice->getUseBM()
#else
    && !pu.cs->slice->getCheckLDC()
#endif
    )
  {
    return (AFFINE_ADAPTIVE_DMVR_MAX_CAND > 0);
  }
  return false;
}
void PU::getBMAffineMergeCand(const PredictionUnit &pu, AffineMergeCtx& affineBMMergeCtx, AffineMergeCtx affineMergeRMVFCtx, int mrgCandIdx)
{
  const CodingStructure &cs = *pu.cs;
  const Slice &slice = *pu.cs->slice;
  const uint32_t maxNumAffineMergeCand = AFFINE_ADAPTIVE_DMVR_INIT_SIZE;

  const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
#if JVET_Z0118_GDR
  bool isClean = pu.cs->isClean(pu.cu->Y().bottomRight(), CHANNEL_TYPE_LUMA);
#endif
  for (int i = 0; i < maxNumAffineMergeCand; i++)
  {
    for (int mvNum = 0; mvNum < 3; mvNum++)
    {
      affineBMMergeCtx.mvFieldNeighbours[(i << 1) + 0][mvNum].setMvField(Mv(), -1);
      affineBMMergeCtx.mvFieldNeighbours[(i << 1) + 1][mvNum].setMvField(Mv(), -1);
    }
    affineBMMergeCtx.interDirNeighbours[i] = 0;
    affineBMMergeCtx.affineType[i] = AFFINEMODEL_4PARAM;
    affineBMMergeCtx.mergeType[i] = MRG_TYPE_DEFAULT_N;
    affineBMMergeCtx.bcwIdx[i] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
    affineBMMergeCtx.altLMFlag[i] = false;
    affineBMMergeCtx.altLMParaNeighbours[i].resetAltLinearModel();
#endif
#if INTER_LIC
    affineBMMergeCtx.licFlags[i] = false;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
    affineBMMergeCtx.obmcFlags[i] = true;
#endif
    affineBMMergeCtx.candCost[i] = MAX_UINT64;
  }
  affineBMMergeCtx.numAffCandToTestEnc = maxNumAffineMergeCand;
  affineBMMergeCtx.numValidMergeCand = 0;
  affineBMMergeCtx.maxNumMergeCand = maxNumAffineMergeCand;

  if (slice.getSPS()->getUseAffine())
  {
    ///> Start: inherited affine candidates
#if JVET_Z0139_HIST_AFF
    const int CHECKED_NEI_NUM = 7;
    const PredictionUnit* npu[CHECKED_NEI_NUM];
    const PredictionUnit* npuGroup2[CHECKED_NEI_NUM];
    Position posGroup2[CHECKED_NEI_NUM];
    int numGroup2;
    numGroup2 = 0;
    int neiIdx[CHECKED_NEI_NUM];
    int aiNeibeInherited[CHECKED_NEI_NUM];
    memset(aiNeibeInherited, 0, sizeof(aiNeibeInherited));

    const Position posLB = pu.Y().bottomLeft();
    const Position posLT = pu.Y().topLeft();
    const Position posRT = pu.Y().topRight();

    Position neiPositions[CHECKED_NEI_NUM] = { posLB.offset(-2, -1), posRT.offset(-1, -2), posRT.offset(3, -2), posLB.offset(-2, 3), posLT.offset(-2, -2),
    posLT.offset(-2, 1), posLT.offset(1, -2) };

    int numAffNeighLeft = getAvailableAffineNeighboursForLeftPredictor(pu, npu, neiIdx);
    if (numAffNeighLeft > 0)
    {
      aiNeibeInherited[neiIdx[0]] = 1;
    }
    int numAffNeigh = getAvailableAffineNeighboursForAbovePredictor(pu, npu, numAffNeighLeft, neiIdx);
    if (numAffNeigh > 0)
    {
      aiNeibeInherited[neiIdx[numAffNeigh - 1]] = 1;
    }
    for (int nei = 0; nei < CHECKED_NEI_NUM; nei++)
    {
      const PredictionUnit* puNei = pu.cs->getPURestricted(neiPositions[nei], pu, pu.chType);
      if (aiNeibeInherited[nei])
      {
        continue;
      }
      if (!puNei)
      {
        continue;
      }
      if (puNei->cu->predMode != MODE_INTER)
      {
        continue;
      }
      MotionInfo mvInfo = puNei->getMotionInfo(neiPositions[nei]);
      if (!mvInfo.isInter || mvInfo.interDir <= 0 || mvInfo.interDir > 3)
      {
        continue;
      }
      posGroup2[numGroup2] = neiPositions[nei];
      npuGroup2[numGroup2++] = puNei;
    }
#else
    const PredictionUnit* npu[5];
    int numAffNeighLeft = getAvailableAffineNeighboursForLeftPredictor(pu, npu);
    int numAffNeigh = getAvailableAffineNeighboursForAbovePredictor(pu, npu, numAffNeighLeft);
#endif

    for (int idx = 0; idx < numAffNeigh; idx++)
    {
      // derive Mv from Neigh affine PU
      Mv cMv[2][3];
      const PredictionUnit* puNeigh = npu[idx];
      pu.cu->affineType = puNeigh->cu->affineType;
      if (puNeigh->interDir != 2)
      {
        xInheritedAffineMv(pu, puNeigh, REF_PIC_LIST_0, cMv[0]);
      }
      if (slice.isInterB())
      {
        if (puNeigh->interDir != 1)
        {
          xInheritedAffineMv(pu, puNeigh, REF_PIC_LIST_1, cMv[1]);
        }
      }
      if (puNeigh->interDir == 3 && isBiPredFromDifferentDirEqDistPoc(pu, puNeigh->refIdx[0], puNeigh->refIdx[1]))
      {
        for (int mvNum = 0; mvNum < 3; mvNum++)
        {
          affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 0][mvNum].setMvField(cMv[0][mvNum], puNeigh->refIdx[0]);
          affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 1][mvNum].setMvField(cMv[1][mvNum], puNeigh->refIdx[1]);
        }
        affineBMMergeCtx.interDirNeighbours[affineBMMergeCtx.numValidMergeCand] = puNeigh->interDir;
        affineBMMergeCtx.affineType[affineBMMergeCtx.numValidMergeCand] = (EAffineModel)(puNeigh->cu->affineType);
        affineBMMergeCtx.bcwIdx[affineBMMergeCtx.numValidMergeCand] = puNeigh->cu->bcwIdx;
#if INTER_LIC
#if !JVET_AD0213_LIC_IMP
        CHECK(puNeigh->interDir == 3 && puNeigh->cu->licFlag, "LIC should not be enabled for affine bi-pred");
#endif
        affineBMMergeCtx.licFlags[affineBMMergeCtx.numValidMergeCand] = puNeigh->cu->licFlag;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
#if JVET_AD0213_LIC_IMP
        affineBMMergeCtx.obmcFlags[affineBMMergeCtx.numValidMergeCand] = puNeigh->cu->affine ? puNeigh->cu->obmcFlag : true;
#else
        affineBMMergeCtx.obmcFlags[affineBMMergeCtx.numValidMergeCand] = puNeigh->cu->affine ? puNeigh->cu->obmcFlag : !puNeigh->cu->licFlag;
#endif
#endif
#if JVET_Z0139_HIST_AFF
        if (!checkLastAffineMergeCandRedundancy(pu, affineBMMergeCtx))
        {
          continue;
        }
#endif
        // redundancy check
        if (affineBMMergeCtx.xCheckSimilarMotion(affineBMMergeCtx.numValidMergeCand, 0))
        {
          continue;
        }
        if (affineBMMergeCtx.numValidMergeCand == mrgCandIdx)
        {
          affineBMMergeCtx.numValidMergeCand++;
          return;
        }

        // early termination
        affineBMMergeCtx.numValidMergeCand++;
        if (affineBMMergeCtx.numValidMergeCand == maxNumAffineMergeCand)
        {
          return;
        }
      }
    }

    Mv cMv[2][3];
    int8_t referenceidx[2];
    for (int i = 0; i < affineMergeRMVFCtx.numValidMergeCand; i++)
    {
      if (affineMergeRMVFCtx.interDirNeighbours[i] == 3 && isBiPredFromDifferentDirEqDistPoc(pu, affineMergeRMVFCtx.mvFieldNeighbours[(i << 1) + 0][0].refIdx, affineMergeRMVFCtx.mvFieldNeighbours[(i << 1) + 1][0].refIdx))
      {
        cMv[0][0] = affineMergeRMVFCtx.mvFieldNeighbours[(i << 1) + 0][0].mv;
        cMv[0][1] = affineMergeRMVFCtx.mvFieldNeighbours[(i << 1) + 0][1].mv;
        cMv[0][2] = affineMergeRMVFCtx.mvFieldNeighbours[(i << 1) + 0][2].mv;
        cMv[1][0] = affineMergeRMVFCtx.mvFieldNeighbours[(i << 1) + 1][0].mv;
        cMv[1][1] = affineMergeRMVFCtx.mvFieldNeighbours[(i << 1) + 1][1].mv;
        cMv[1][2] = affineMergeRMVFCtx.mvFieldNeighbours[(i << 1) + 1][2].mv;
        referenceidx[0] = affineMergeRMVFCtx.mvFieldNeighbours[(i << 1) + 0][0].refIdx;
        referenceidx[1] = affineMergeRMVFCtx.mvFieldNeighbours[(i << 1) + 1][0].refIdx;
        if (xCPMVSimCheck(pu, affineBMMergeCtx, cMv, affineMergeRMVFCtx.interDirNeighbours[i], referenceidx, EAffineModel(affineMergeRMVFCtx.affineType[i]), affineMergeRMVFCtx.bcwIdx[i], affineMergeRMVFCtx.licFlags[i]))
        {
          continue;
        }
        for (int mvNum = 0; mvNum < 3; mvNum++)
        {
          affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 0][mvNum].setMvField(cMv[0][mvNum], referenceidx[0]);
          affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 1][mvNum].setMvField(cMv[1][mvNum], referenceidx[1]);
        }
        affineBMMergeCtx.interDirNeighbours[affineBMMergeCtx.numValidMergeCand] = affineMergeRMVFCtx.interDirNeighbours[i];
        affineBMMergeCtx.affineType[affineBMMergeCtx.numValidMergeCand] = EAffineModel(affineMergeRMVFCtx.affineType[i]);
        affineBMMergeCtx.mergeType[affineBMMergeCtx.numValidMergeCand] = MRG_TYPE_DEFAULT_N;
        affineBMMergeCtx.bcwIdx[affineBMMergeCtx.numValidMergeCand] = affineMergeRMVFCtx.bcwIdx[i];
#if JVET_AG0276_NLIC
        affineBMMergeCtx.altLMFlag[affineBMMergeCtx.numValidMergeCand] = affineMergeRMVFCtx.altLMFlag[i];
        affineBMMergeCtx.altLMParaNeighbours[affineBMMergeCtx.numValidMergeCand] = affineMergeRMVFCtx.altLMParaNeighbours[i];
#endif
#if INTER_LIC
        affineBMMergeCtx.licFlags[affineBMMergeCtx.numValidMergeCand] = affineMergeRMVFCtx.licFlags[i];
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
        affineBMMergeCtx.obmcFlags[affineBMMergeCtx.numValidMergeCand] = affineMergeRMVFCtx.obmcFlags[i];
#endif
        affineBMMergeCtx.candCost[affineBMMergeCtx.numValidMergeCand] = affineMergeRMVFCtx.candCost[i];

        // redundancy check
        if (affineBMMergeCtx.xCheckSimilarMotion(affineBMMergeCtx.numValidMergeCand, 0))
        {
          continue;
        }
        if (affineBMMergeCtx.numValidMergeCand == mrgCandIdx)
        {
          affineBMMergeCtx.numValidMergeCand++;
          return;
        }

        // early termination
        affineBMMergeCtx.numValidMergeCand++;
        if (affineBMMergeCtx.numValidMergeCand == maxNumAffineMergeCand)
        {
          return;
        }
      }
    }

#if JVET_Z0139_HIST_AFF
    MotionInfo tmvpInfo;
    tmvpInfo.interDir = 0;
    tmvpInfo.refIdx[0] = tmvpInfo.refIdx[1] = -1;
    bool       isTmvpAvailable = false;
#endif
    ///> End: inherited affine candidates

    ///> Start: Constructed affine candidates
    {
      MotionInfo mi[4];
      bool isAvailable[4] = { false };

      int8_t neighBcw[2] = { BCW_DEFAULT, BCW_DEFAULT };
      // control point: LT B2->B3->A2
      const Position posLT[3] = { pu.Y().topLeft().offset(-1, -1), pu.Y().topLeft().offset(0, -1), pu.Y().topLeft().offset(-1, 0) };
      for (int i = 0; i < 3; i++)
      {
        const Position pos = posLT[i];
        const PredictionUnit* puNeigh = cs.getPURestricted(pos, pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
        if (puNeigh && CU::isInter(*puNeigh->cu) && puNeigh->getMotionInfo(pos).isInter && PU::isDiffMER(pu.lumaPos(), pos, plevel))
#else
        if (puNeigh && CU::isInter(*puNeigh->cu) && PU::isDiffMER(pu.lumaPos(), pos, plevel))
#endif
        {
          isAvailable[0] = true;
          mi[0] = puNeigh->getMotionInfo(pos);
          neighBcw[0] = puNeigh->cu->bcwIdx;
#if JVET_Z0139_HIST_AFF
          if (puNeigh->interDir != 3) neighBcw[0] = BCW_DEFAULT;
#endif
          break;
        }
      }

      // control point: RT B1->B0
      const Position posRT[2] = { pu.Y().topRight().offset(0, -1), pu.Y().topRight().offset(1, -1) };
      for (int i = 0; i < 2; i++)
      {
        const Position pos = posRT[i];
        const PredictionUnit* puNeigh = cs.getPURestricted(pos, pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
        if (puNeigh && CU::isInter(*puNeigh->cu) && puNeigh->getMotionInfo(pos).isInter && PU::isDiffMER(pu.lumaPos(), pos, plevel))
#else
        if (puNeigh && CU::isInter(*puNeigh->cu) && PU::isDiffMER(pu.lumaPos(), pos, plevel))
#endif
        {
          isAvailable[1] = true;
          mi[1] = puNeigh->getMotionInfo(pos);
          neighBcw[1] = puNeigh->cu->bcwIdx;
#if JVET_Z0139_HIST_AFF
          if (puNeigh->interDir != 3) neighBcw[1] = BCW_DEFAULT;
#endif
          break;
        }
      }

      // control point: LB A1->A0
      const Position posLB[2] = { pu.Y().bottomLeft().offset(-1, 0), pu.Y().bottomLeft().offset(-1, 1) };
      for (int i = 0; i < 2; i++)
      {
        const Position pos = posLB[i];
        const PredictionUnit* puNeigh = cs.getPURestricted(pos, pu, pu.chType);

#if JVET_Y0065_GPM_INTRA
        if (puNeigh && CU::isInter(*puNeigh->cu) && puNeigh->getMotionInfo(pos).isInter && PU::isDiffMER(pu.lumaPos(), pos, plevel))
#else
        if (puNeigh && CU::isInter(*puNeigh->cu) && PU::isDiffMER(pu.lumaPos(), pos, plevel))
#endif
        {
          isAvailable[2] = true;
          mi[2] = puNeigh->getMotionInfo(pos);
          break;
        }
      }

      // control point: RB
      if (slice.getPicHeader()->getEnableTMVPFlag())
      {
        //>> MTK colocated-RightBottom
        // offset the pos to be sure to "point" to the same position the uiAbsPartIdx would've pointed to
        Position posRB = pu.Y().bottomRight().offset(-3, -3);

        const PreCalcValues& pcv = *cs.pcv;
        Position posC0;
        bool C0Avail = false;

        bool boundaryCond = ((posRB.x + pcv.minCUWidth) < pcv.lumaWidth) && ((posRB.y + pcv.minCUHeight) < pcv.lumaHeight);
        const SubPic &curSubPic = pu.cs->slice->getPPS()->getSubPicFromPos(pu.lumaPos());
        if (curSubPic.getTreatedAsPicFlag())
        {
          boundaryCond = ((posRB.x + pcv.minCUWidth) <= curSubPic.getSubPicRight() &&
            (posRB.y + pcv.minCUHeight) <= curSubPic.getSubPicBottom());
        }
        if (boundaryCond)
        {
          int posYInCtu = posRB.y & pcv.maxCUHeightMask;
          if (posYInCtu + 4 < pcv.maxCUHeight)
          {
            posC0 = posRB.offset(4, 4);
            C0Avail = true;
          }
        }

        Mv        cColMv;
        int       refIdx = 0;
        bool      bExistMV = C0Avail && getColocatedMVP(pu, REF_PIC_LIST_0, posC0, cColMv, refIdx, false
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
          , 0
          , &refIdx
#endif
        );
        if (bExistMV)
        {
          mi[3].mv[0] = cColMv;
          mi[3].refIdx[0] = refIdx;
          mi[3].interDir = 1;
          isAvailable[3] = true;
#if JVET_Z0139_HIST_AFF
          tmvpInfo.mv[0] = cColMv;
          tmvpInfo.refIdx[0] = refIdx;
          tmvpInfo.interDir |= 1;
          isTmvpAvailable = true;
#endif
        }

        if (slice.isInterB())
        {
          bExistMV = C0Avail && getColocatedMVP(pu, REF_PIC_LIST_1, posC0, cColMv, refIdx, false
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
            , 0
            , &refIdx
#endif
          );
          if (bExistMV)
          {
            mi[3].mv[1] = cColMv;
            mi[3].refIdx[1] = refIdx;
            mi[3].interDir |= 2;
            isAvailable[3] = true;
#if JVET_Z0139_HIST_AFF
            tmvpInfo.mv[1] = cColMv;
            tmvpInfo.refIdx[1] = refIdx;
            tmvpInfo.interDir |= 2;
            isTmvpAvailable = true;
#endif
          }
        }
#if INTER_LIC
        mi[3].usesLIC = false;
#endif
      }

      //-------------------  insert model  -------------------//
      int order[6] = { 0, 1, 2, 3, 4, 5 };
      int modelNum = 6;
      int model[6][4] = {
        { 0, 1, 2 },          // 0:  LT, RT, LB
        { 0, 1, 3 },          // 1:  LT, RT, RB
        { 0, 2, 3 },          // 2:  LT, LB, RB
        { 1, 2, 3 },          // 3:  RT, LB, RB
        { 0, 1 },             // 4:  LT, RT
        { 0, 2 },             // 5:  LT, LB
      };

      int verNum[6] = { 3, 3, 3, 3, 2, 2 };
      int startIdx = pu.cs->sps->getUseAffineType() ? 0 : 4;
      for (int idx = startIdx; idx < modelNum; idx++)
      {
        int modelIdx = order[idx];
#if JVET_Z0139_HIST_AFF
        if (getAffineControlPointCand(pu, mi, isAvailable, model[modelIdx], ((modelIdx == 3) ? neighBcw[1] : neighBcw[0]), modelIdx, verNum[modelIdx], affineBMMergeCtx))
        {
          if (affineBMMergeCtx.interDirNeighbours[affineBMMergeCtx.numValidMergeCand] == 3 && isBiPredFromDifferentDirEqDistPoc(pu, affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 0][0].refIdx, affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 1][0].refIdx))
          {
            // redundancy check
            if (affineBMMergeCtx.xCheckSimilarMotion(affineBMMergeCtx.numValidMergeCand, 0))
            {
              continue;
            }
            if (affineBMMergeCtx.numValidMergeCand == mrgCandIdx)
            {
              affineBMMergeCtx.numValidMergeCand++;
              return;
            }

            affineBMMergeCtx.numValidMergeCand++;
            if (affineBMMergeCtx.numValidMergeCand == maxNumAffineMergeCand)
            {
              return;
            }
          }
        }
#else
        getAffineControlPointCand(pu, mi, isAvailable, model[modelIdx], ((modelIdx == 3) ? neighBcw[1] : neighBcw[0]), modelIdx, verNum[modelIdx], affMrgCtx);
        // redundancy check
        if (affineBMMergeCtx.xCheckSimilarMotion(affineBMMergeCtx.numValidMergeCand, 0))
        {
          continue;
        }
        if (affMrgCtx.numValidMergeCand != 0 && affMrgCtx.numValidMergeCand - 1 == mrgCandIdx)
        {
          return;
        }

        // early termination
        if (affMrgCtx.numValidMergeCand == maxNumAffineMergeCand)
        {
          return;
        }
#endif
#if JVET_Z0139_HIST_AFF
        if (idx == startIdx)
        {
#if JVET_Z0118_GDR
          for (int nei = 0; nei < numGroup2; nei++)
          {
            const PredictionUnit* puNei = npuGroup2[nei];
            CHECK(!npuGroup2[nei], "Invalid neighbour PU");
            MotionInfo mvInfo = puNei->getMotionInfo(posGroup2[nei]);
            if (mvInfo.isIBCmot)
            {
              continue;
            }
            if (addOneAffineMergeHMVPCand(pu, affineBMMergeCtx, (isClean) ? pu.cs->motionLut.lutAff1 : pu.cs->motionLut.lutAff0, 0, mvInfo, posGroup2[nei], puNei->interDir == 3 ? puNei->cu->bcwIdx : BCW_DEFAULT
#if INTER_LIC
              , puNei->cu->licFlag
#endif
            ))
            {
              if (affineBMMergeCtx.interDirNeighbours[affineBMMergeCtx.numValidMergeCand] == 3 && isBiPredFromDifferentDirEqDistPoc(pu, affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 0][0].refIdx, affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 1][0].refIdx))
              {
                // redundancy check
                if (affineBMMergeCtx.xCheckSimilarMotion(affineBMMergeCtx.numValidMergeCand, 0))
                {
                  continue;
                }
                if (affineBMMergeCtx.numValidMergeCand == mrgCandIdx) // for decoder
                {
                  affineBMMergeCtx.numValidMergeCand++;
                  return;
                }

                affineBMMergeCtx.numValidMergeCand++;

                // early termination
                if (affineBMMergeCtx.numValidMergeCand == maxNumAffineMergeCand)
                {
                  return;
                }
              }
            }
          } //for nei
#else
          for (int nei = 0; nei < numGroup2; nei++)
          {
            const PredictionUnit* puNei = npuGroup2[nei];
            CHECK(!npuGroup2[nei], "Invalid neighbour PU");
            MotionInfo mvInfo = puNei->getMotionInfo(posGroup2[nei]);
            if (mvInfo.isIBCmot)
            {
              continue;
            }
            if (addOneAffineMergeHMVPCand(pu, affineBMMergeCtx, pu.cs->motionLut.lutAff, 0, mvInfo, posGroup2[nei], puNei->interDir == 3 ? puNei->cu->BcwIdx : BCW_DEFAULT
#if INTER_LIC
              , puNei->cu->LICFlag
#endif
            ))
            {
              if (affineBMMergeCtx.interDirNeighbours[affineBMMergeCtx.numValidMergeCand] == 3 && isBiPredFromDifferentDirEqDistPoc(pu, affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 0][0].refIdx, affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 1][0].refIdx))
              {
                // redundancy check
                if (affineBMMergeCtx.xCheckSimilarMotion(affineBMMergeCtx.numValidMergeCand, 0))
                {
                  continue;
                }
                if (affineBMMergeCtx.numValidMergeCand == mrgCandIdx) // for decoder
                {
                  affineBMMergeCtx.numValidMergeCand++;
                  return;
                }

                affineBMMergeCtx.numValidMergeCand++;

                // early termination
                if (affineBMMergeCtx.numValidMergeCand == maxNumAffineMergeCand)
                {
                  return;
                }
              }
            }
          } //for nei
#endif
#if JVET_Z0118_GDR
          if (addOneInheritedHMVPAffineMergeCand(pu, affineBMMergeCtx, (isClean) ? pu.cs->motionLut.lutAffInherit1 : pu.cs->motionLut.lutAffInherit0, 0))
#else
          if (addOneInheritedHMVPAffineMergeCand(pu, affineBMMergeCtx, pu.cs->motionLut.lutAffInherit, 0))
#endif
          {
            if (affineBMMergeCtx.interDirNeighbours[affineBMMergeCtx.numValidMergeCand] == 3 && isBiPredFromDifferentDirEqDistPoc(pu, affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 0][0].refIdx, affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 1][0].refIdx))
            {
              // redundancy check
              if (affineBMMergeCtx.xCheckSimilarMotion(affineBMMergeCtx.numValidMergeCand, 0))
              {
                continue;
              }
              if (affineBMMergeCtx.numValidMergeCand == mrgCandIdx) // for decoder
              {
                affineBMMergeCtx.numValidMergeCand++;
                return;
              }

              affineBMMergeCtx.numValidMergeCand++;

              // early termination
              if (affineBMMergeCtx.numValidMergeCand == maxNumAffineMergeCand)
              {
                return;
              }
            }
          }
        }
#endif
#if JVET_Z0139_NA_AFF
        if (!((pu.cu->slice->getPOC() - pu.cu->slice->getRefPOC(REF_PIC_LIST_0, 0)) == 1 && pu.cu->slice->getPicHeader()->getMvdL1ZeroFlag()))
        {
          if (idx == startIdx)
          {
            if (getBMNonAdjCstMergeCand(pu, affineBMMergeCtx))
            {
              return;
            }
          }
        }
#endif
      }

#if JVET_Z0139_NA_AFF
      if ((pu.cu->slice->getPOC() - pu.cu->slice->getRefPOC(REF_PIC_LIST_0, 0)) == 1 && pu.cu->slice->getPicHeader()->getMvdL1ZeroFlag())
      {
        if (getBMNonAdjCstMergeCand(pu, affineBMMergeCtx))
        {
          return;
        }
      }
#endif
    }
    ///> End: Constructed affine candidates
#if JVET_Z0139_HIST_AFF
    if (isTmvpAvailable)
    {
      Position posRB = pu.Y().bottomRight().offset(3, 3);
      if (addOneAffineMergeHMVPCand(pu, affineBMMergeCtx
#if JVET_Z0118_GDR
        , (isClean) ? pu.cs->motionLut.lutAff1 : pu.cs->motionLut.lutAff0
#else
        , pu.cs->motionLut.lutAff
#endif
        , 0, tmvpInfo, posRB, BCW_DEFAULT
#if INTER_LIC
        , false
#endif
      ))
      {
        if (affineBMMergeCtx.interDirNeighbours[affineBMMergeCtx.numValidMergeCand] == 3 && isBiPredFromDifferentDirEqDistPoc(pu, affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 0][0].refIdx, affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 1][0].refIdx))
        {
          // redundancy check
          if (!affineBMMergeCtx.xCheckSimilarMotion(affineBMMergeCtx.numValidMergeCand, 0))
          {
            if (affineBMMergeCtx.numValidMergeCand == mrgCandIdx) // for decoder
            {
              affineBMMergeCtx.numValidMergeCand++;
              return;
            }

            affineBMMergeCtx.numValidMergeCand++;

            // early termination
            if (affineBMMergeCtx.numValidMergeCand == maxNumAffineMergeCand)
            {
              return;
            }
          }
        }
      }
    }

    for (int iAffListIdx = 1; iAffListIdx < MAX_NUM_AFF_HMVP_CANDS; iAffListIdx++)
    {
#if JVET_Z0118_GDR
      for (int nei = 0; nei < numGroup2; nei++)
      {
        const PredictionUnit* puNei = npuGroup2[nei];
        CHECK(!npuGroup2[nei], "Invalid neighbour PU");
        MotionInfo mvInfo = puNei->getMotionInfo(posGroup2[nei]);
        if (mvInfo.isIBCmot)
        {
          continue;
        }
        if (addOneAffineMergeHMVPCand(pu, affineBMMergeCtx, (isClean) ? pu.cs->motionLut.lutAff1 : pu.cs->motionLut.lutAff0, iAffListIdx, mvInfo, posGroup2[nei], puNei->interDir == 3 ? puNei->cu->bcwIdx : BCW_DEFAULT
#if INTER_LIC
          , puNei->cu->licFlag
#endif
        ))
        {
          if (affineBMMergeCtx.interDirNeighbours[affineBMMergeCtx.numValidMergeCand] == 3 && isBiPredFromDifferentDirEqDistPoc(pu, affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 0][0].refIdx, affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 1][0].refIdx))
          {
            // redundancy check
            if (affineBMMergeCtx.xCheckSimilarMotion(affineBMMergeCtx.numValidMergeCand, 0))
            {
              continue;
            }
            if (affineBMMergeCtx.numValidMergeCand == mrgCandIdx) // for decoder
            {
              affineBMMergeCtx.numValidMergeCand++;
              return;
            }

            affineBMMergeCtx.numValidMergeCand++;

            // early termination
            if (affineBMMergeCtx.numValidMergeCand == maxNumAffineMergeCand)
            {
              return;
            }
          }
        }
      } //for nei
#else
      for (int nei = 0; nei < numGroup2; nei++)
      {
        const PredictionUnit* puNei = npuGroup2[nei];
        CHECK(!npuGroup2[nei], "Invalid neighbour PU");
        MotionInfo mvInfo = puNei->getMotionInfo(posGroup2[nei]);
        if (mvInfo.isIBCmot)
        {
          continue;
        }
        if (addOneAffineMergeHMVPCand(pu, affineBMMergeCtx, pu.cs->motionLut.lutAff, iAffListIdx, mvInfo, posGroup2[nei], puNei->interDir == 3 ? puNei->cu->BcwIdx : BCW_DEFAULT
#if INTER_LIC
          , puNei->cu->LICFlag
#endif
        ))
        {
          if (affineBMMergeCtx.interDirNeighbours[affineBMMergeCtx.numValidMergeCand] == 3 && isBiPredFromDifferentDirEqDistPoc(pu, affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 0][0].refIdx, affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 1][0].refIdx))
          {
            // redundancy check
            if (affineBMMergeCtx.xCheckSimilarMotion(affineBMMergeCtx.numValidMergeCand, 0))
            {
              continue;
            }
            if (affineBMMergeCtx.numValidMergeCand == mrgCandIdx) // for decoder
            {
              affineBMMergeCtx.numValidMergeCand++;
              return;
            }

            affineBMMergeCtx.numValidMergeCand++;

            // early termination
            if (affineBMMergeCtx.numValidMergeCand == maxNumAffineMergeCand)
            {
              return;
            }
          }
        }
      } //for nei
#endif

      if (isTmvpAvailable)
      {
        Position posRB = pu.Y().bottomRight().offset(3, 3);
        if (addOneAffineMergeHMVPCand(pu, affineBMMergeCtx
#if JVET_Z0118_GDR
          , (isClean) ? pu.cs->motionLut.lutAff1 : pu.cs->motionLut.lutAff0
#else
          , pu.cs->motionLut.lutAff
#endif
          , iAffListIdx, tmvpInfo, posRB, BCW_DEFAULT
#if INTER_LIC
          , false
#endif
        ))
        {
          if (affineBMMergeCtx.interDirNeighbours[affineBMMergeCtx.numValidMergeCand] == 3 && isBiPredFromDifferentDirEqDistPoc(pu, affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 0][0].refIdx, affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 1][0].refIdx))
          {
            // redundancy check
            if (affineBMMergeCtx.xCheckSimilarMotion(affineBMMergeCtx.numValidMergeCand, 0))
            {
              continue;
            }
            if (affineBMMergeCtx.numValidMergeCand == mrgCandIdx) // for decoder
            {
              affineBMMergeCtx.numValidMergeCand++;
              return;
            }

            affineBMMergeCtx.numValidMergeCand++;

            // early termination
            if (affineBMMergeCtx.numValidMergeCand == maxNumAffineMergeCand)
            {
              return;
            }
          }
        }
      }
    }

    for (int iAffListIdx = 1; iAffListIdx < MAX_NUM_AFF_INHERIT_HMVP_CANDS; iAffListIdx++)
    {
#if JVET_Z0118_GDR
      if (addOneInheritedHMVPAffineMergeCand(pu, affineBMMergeCtx, (isClean) ? pu.cs->motionLut.lutAffInherit1 : pu.cs->motionLut.lutAffInherit0, iAffListIdx))
#else
      if (addOneInheritedHMVPAffineMergeCand(pu, affineBMMergeCtx, pu.cs->motionLut.lutAffInherit, iAffListIdx))
#endif
      {
        if (affineBMMergeCtx.interDirNeighbours[affineBMMergeCtx.numValidMergeCand] == 3 && isBiPredFromDifferentDirEqDistPoc(pu, affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 0][0].refIdx, affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 1][0].refIdx))
        {
          // redundancy check
          if (affineBMMergeCtx.xCheckSimilarMotion(affineBMMergeCtx.numValidMergeCand, 0))
          {
            continue;
          }
          if (affineBMMergeCtx.numValidMergeCand == mrgCandIdx) // for decoder
          {
            affineBMMergeCtx.numValidMergeCand++;
            return;
          }

          affineBMMergeCtx.numValidMergeCand++;

          // early termination
          if (affineBMMergeCtx.numValidMergeCand == maxNumAffineMergeCand)
          {
            return;
          }
        }
      }
    }
    const int MAX_PAIRWISE_NUM = 9;
    const int preDefinedPairs[MAX_PAIRWISE_NUM][2] = { {0, 1}, {0, 2}, {1, 2}, {0, 3}, {1, 3}, { 2, 3}, { 0, 4}, {1, 4}, { 2, 4} };
    int iATMVPoffset = 0;
    int currSize = affineBMMergeCtx.numValidMergeCand;
    for (int pairIdx = 0; pairIdx < MAX_PAIRWISE_NUM; pairIdx++)
    {
      int idx0 = preDefinedPairs[pairIdx][0] + iATMVPoffset;
      int idx1 = preDefinedPairs[pairIdx][1] + iATMVPoffset;

      if (idx0 >= currSize || idx1 >= currSize)
      {
        break;
      }
      if ((affineBMMergeCtx.interDirNeighbours[idx0] & affineBMMergeCtx.interDirNeighbours[idx1]) == 0)
      {
        continue;
      }

      affineBMMergeCtx.interDirNeighbours[affineBMMergeCtx.numValidMergeCand] = 0;

      affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 0][0].setMvField(Mv(), -1);
      affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 0][1].setMvField(Mv(), -1);
      affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 0][2].setMvField(Mv(), -1);
      affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 1][0].setMvField(Mv(), -1);
      affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 1][1].setMvField(Mv(), -1);
      affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 1][2].setMvField(Mv(), -1);

      affineBMMergeCtx.bcwIdx[affineBMMergeCtx.numValidMergeCand] = BCW_DEFAULT;

#if INTER_LIC
      affineBMMergeCtx.licFlags[affineBMMergeCtx.numValidMergeCand] = false;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
      affineBMMergeCtx.obmcFlags[affineBMMergeCtx.numValidMergeCand] = true;
#endif
      affineBMMergeCtx.affineType[affineBMMergeCtx.numValidMergeCand] = AFFINEMODEL_6PARAM;

      int numCPMVs = 3;

      if (affineBMMergeCtx.mvFieldNeighbours[(idx0 << 1)][0].refIdx != -1 && affineBMMergeCtx.mvFieldNeighbours[(idx1 << 1)][0].refIdx == affineBMMergeCtx.mvFieldNeighbours[(idx0 << 1)][0].refIdx)
      {
        affineBMMergeCtx.interDirNeighbours[affineBMMergeCtx.numValidMergeCand] |= 1;

        for (int mvIdx = 0; mvIdx < numCPMVs; mvIdx++)
        {
          Mv avgMv = affineBMMergeCtx.mvFieldNeighbours[(idx0 << 1)][mvIdx].mv;
          avgMv += affineBMMergeCtx.mvFieldNeighbours[(idx1 << 1)][mvIdx].mv;
          roundAffineMv(avgMv.hor, avgMv.ver, 1);
          affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 0][mvIdx].setMvField(avgMv, affineBMMergeCtx.mvFieldNeighbours[(idx0 << 1)][0].refIdx);
        }
      }
      if (affineBMMergeCtx.mvFieldNeighbours[(idx0 << 1) + 1][0].refIdx != -1 && affineBMMergeCtx.mvFieldNeighbours[(idx1 << 1) + 1][0].refIdx == affineBMMergeCtx.mvFieldNeighbours[(idx0 << 1) + 1][0].refIdx)
      {
        affineBMMergeCtx.interDirNeighbours[affineBMMergeCtx.numValidMergeCand] |= 2;

        for (int mvIdx = 0; mvIdx < numCPMVs; mvIdx++)
        {
          Mv avgMv = affineBMMergeCtx.mvFieldNeighbours[(idx0 << 1) + 1][mvIdx].mv;
          avgMv += affineBMMergeCtx.mvFieldNeighbours[(idx1 << 1) + 1][mvIdx].mv;
          roundAffineMv(avgMv.hor, avgMv.ver, 1);
          affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 1][mvIdx].setMvField(avgMv, affineBMMergeCtx.mvFieldNeighbours[(idx0 << 1) + 1][0].refIdx);
        }
      }

      if (affineBMMergeCtx.interDirNeighbours[affineBMMergeCtx.numValidMergeCand] == 0)
      {
        continue;
      }
      if (affineBMMergeCtx.interDirNeighbours[affineBMMergeCtx.numValidMergeCand] == 3 && isBiPredFromDifferentDirEqDistPoc(pu, affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 0][0].refIdx, affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 1][0].refIdx))
      {
        if (checkLastAffineMergeCandRedundancy(pu, affineBMMergeCtx))
        {
          // redundancy check
          if (affineBMMergeCtx.xCheckSimilarMotion(affineBMMergeCtx.numValidMergeCand, 0))
          {
            continue;
          }

          if (affineBMMergeCtx.numValidMergeCand == mrgCandIdx) // for decoder
          {
            affineBMMergeCtx.numValidMergeCand++;
            return;
          }

          affineBMMergeCtx.numValidMergeCand++;

          // early termination
          if (affineBMMergeCtx.numValidMergeCand == maxNumAffineMergeCand)
          {
            return;
          }
        }
      }
    }
#endif
  }

  ///> zero padding
  int cnt = affineBMMergeCtx.numValidMergeCand;
  affineBMMergeCtx.numAffCandToTestEnc = cnt;
  while (cnt < maxNumAffineMergeCand)
  {
    for (int mvNum = 0; mvNum < 3; mvNum++)
    {
      affineBMMergeCtx.mvFieldNeighbours[(cnt << 1) + 0][mvNum].setMvField(Mv(0, 0), pu.cs->slice->getBMDefaultRefIdx(0));
    }
    affineBMMergeCtx.interDirNeighbours[cnt] = 1;

    if (slice.isInterB())
    {
      for (int mvNum = 0; mvNum < 3; mvNum++)
      {
        affineBMMergeCtx.mvFieldNeighbours[(cnt << 1) + 1][mvNum].setMvField(Mv(0, 0), pu.cs->slice->getBMDefaultRefIdx(1));
      }
      affineBMMergeCtx.interDirNeighbours[cnt] = 3;
    }
    affineBMMergeCtx.affineType[cnt] = AFFINEMODEL_4PARAM;
    affineBMMergeCtx.interDirNeighbours[cnt] = 3;
    affineBMMergeCtx.candCost[cnt] = MAX_UINT64;
    affineBMMergeCtx.mergeType[cnt] = MRG_TYPE_DEFAULT_N;
    affineBMMergeCtx.bcwIdx[cnt] = BCW_DEFAULT;
#if INTER_LIC
    affineBMMergeCtx.licFlags[cnt] = false;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
    affineBMMergeCtx.obmcFlags[cnt] = true;
#endif
    if (cnt == mrgCandIdx)
    {
      affineBMMergeCtx.numValidMergeCand++;
      return;
    }
    cnt++;
    affineBMMergeCtx.numValidMergeCand++;
  }
}
void PU::getRMVFAffineCand(const PredictionUnit &pu, AffineMergeCtx& affineMergeRMVFCtx, AffineMergeCtx& affineMergeRMVFOriCtx, InterPrediction* m_pcInterSearch
  , uint16_t &numCand
#if JVET_AG0276_NLIC
  , AltLMAffineMergeCtx& altAffineMergeRMVFCtx
#if JVET_AG0276_LIC_FLAG_SIGNALING
  , AltLMAffineMergeCtx& altBRAffineMergeRMVFCtx
#endif
#endif
)
{
  const int CHECKED_NEI_NUM = 7;
  const PredictionUnit *npu[CHECKED_NEI_NUM + AFF_MAX_NON_ADJACENT_INHERITED_CANDS + MAX_NUM_AFF_INHERIT_HMVP_CANDS];
  AffineMergeCtx affMrgCtxTemp;
  for (int i = 0; i < RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE; i++)
  {
    for (int mvNum = 0; mvNum < 3; mvNum++)
    {
      affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 0][mvNum].setMvField(Mv(), -1);
      affMrgCtxTemp.mvFieldNeighbours[(i << 1) + 1][mvNum].setMvField(Mv(), -1);
    }
    affMrgCtxTemp.interDirNeighbours[i] = 0;
    affMrgCtxTemp.affineType[i] = AFFINEMODEL_4PARAM;
    affMrgCtxTemp.mergeType[i] = MRG_TYPE_DEFAULT_N;
    affMrgCtxTemp.bcwIdx[i] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
    affMrgCtxTemp.altLMFlag[i] = false;
    affMrgCtxTemp.altLMParaNeighbours[i].resetAltLinearModel();
#endif
#if INTER_LIC
    affMrgCtxTemp.licFlags[i] = false;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
    affMrgCtxTemp.obmcFlags[i] = true;
#endif
    affMrgCtxTemp.candCost[i] = MAX_UINT64;
  }
  affMrgCtxTemp.numValidMergeCand = 0;
  affMrgCtxTemp.maxNumMergeCand = RMVF_AFFINE_MRG_MAX_CAND_LIST_SIZE;
  int numAffNeighExtend2 = getNonAdjAvailableAffineNeighboursByDistance(pu, npu, affMrgCtxTemp, 0, -1);
  numCand = numAffNeighExtend2;
  if (numAffNeighExtend2 > 0)
  {
    std::vector<RMVFInfo> mvpInfoVec[NUM_REF_PIC_LIST_01][MAX_NUM_REF];
    collectNeiMotionInfo(mvpInfoVec, pu);
#if JVET_W0090_ARMC_TM
    if (pu.cs->sps->getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
        && pu.cs->sps->getTMToolsEnableFlag()
#endif
      )
    {
      for (int i = 0; i < numAffNeighExtend2; i++)
      {
        getRMVFAffineGuideCand(pu, *npu[i], affMrgCtxTemp, mvpInfoVec, -1
#if JVET_AG0276_NLIC
          , &altAffineMergeRMVFCtx
#if JVET_AG0276_LIC_FLAG_SIGNALING
          , &altBRAffineMergeRMVFCtx
#endif
#endif
        );
        if (affMrgCtxTemp.numValidMergeCand == affMrgCtxTemp.maxNumMergeCand)
        {
          break;
        }
      }
      affineMergeRMVFOriCtx = affMrgCtxTemp;
      PredictionUnit pu2 = pu;
#if JVET_AG0164_AFFINE_GPM
      bool affineStored  = pu.cu->affine;
      int  affTypeStored = pu.cu->affineType;
      int  bcwStored     = pu.cu->bcwIdx;
#if INTER_LIC
      bool licStored     = pu.cu->licFlag;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
      bool obmcStored    = pu.cu->obmcFlag;
#endif
#endif
      m_pcInterSearch->adjustAffineMergeCandidatesOneGroup(pu2, affMrgCtxTemp, affMrgCtxTemp.numValidMergeCand, affineMergeRMVFOriCtx);
#if JVET_AG0164_AFFINE_GPM
      pu.cu->affine      = affineStored;
      pu.cu->affineType  = affTypeStored;
      pu.cu->bcwIdx      = bcwStored;
#if INTER_LIC
      pu.cu->licFlag     = licStored;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
      pu.cu->obmcFlag    = obmcStored;
#endif
#endif

      affineMergeRMVFCtx = affMrgCtxTemp;
    }
    else
    {
#endif
      for (int i = 0; i < numAffNeighExtend2; i++)
      {
        getRMVFAffineGuideCand(pu, *npu[i], affMrgCtxTemp, mvpInfoVec, -1
#if JVET_AG0276_NLIC
          , &altAffineMergeRMVFCtx
#if JVET_AG0276_LIC_FLAG_SIGNALING
          , &altBRAffineMergeRMVFCtx
#endif
#endif
        );
        if (affMrgCtxTemp.numValidMergeCand == affMrgCtxTemp.maxNumMergeCand)
        {
          break;
        }
      }
#if JVET_W0090_ARMC_TM
    }
#endif
  }
}
bool PU::getBMNonAdjCstMergeCand(const PredictionUnit &pu, AffineMergeCtx &affineBMMergeCtx, const int mrgCandIdx)
{
  const CodingStructure &cs = *pu.cs;
  const uint32_t maxNumAffineMergeCand = AFFINE_ADAPTIVE_DMVR_INIT_SIZE;

  const unsigned plevel = pu.cs->sps->getLog2ParallelMergeLevelMinus2() + 2;
  const Position posLT[3] = { pu.Y().topLeft().offset(-1, -1), pu.Y().topLeft().offset(0, -1), pu.Y().topLeft().offset(-1, 0) };
  const Position posRT[2] = { pu.Y().topRight().offset(0, -1), pu.Y().topRight().offset(1, -1) };
  const Position posLB[2] = { pu.Y().bottomLeft().offset(-1, 0), pu.Y().bottomLeft().offset(-1, 1) };

  for (int i = 1; i < (AFF_NON_ADJACENT_DIST + 1); i++)
  {
    MotionInfo miNew[4];
    Position   posNew[4];
    bool       isAvailableNew[4] = { false, false, false, false };
    int8_t     neighBcwNew[2] = { BCW_DEFAULT, BCW_DEFAULT };

    const Position        posTRNew = Position(posRT[0].x, posRT[0].y - (i * ((int)pu.Y().height)));
    const PredictionUnit *puNeighTRNew = cs.getPURestricted(posTRNew, pu, pu.chType);
#if JVET_Y0065_GPM_INTRA
    if (puNeighTRNew && CU::isInter(*puNeighTRNew->cu) && puNeighTRNew->getMotionInfo(posTRNew).isInter && PU::isDiffMER(pu.lumaPos(), posTRNew, plevel))
#else
    if (puNeighTRNew && CU::isInter(*puNeighTRNew->cu) && PU::isDiffMER(pu.lumaPos(), posTRNew, plevel))
#endif
    {
      isAvailableNew[1] = true;
      miNew[1] = puNeighTRNew->getMotionInfo(posTRNew);
      neighBcwNew[1] = puNeighTRNew->cu->bcwIdx;
      posNew[1] = posTRNew;
      CHECK(posTRNew.x < 0 || posTRNew.y < 0, "posTRNew < 0");
    }

    for (int j = 1; j < (AFF_NON_ADJACENT_DIST + 1); j++)
    {
      isAvailableNew[0] = false;
      isAvailableNew[2] = false;
      const Position        posLBNew = Position(posLB[0].x - (j * ((int)pu.Y().width)), posLB[0].y);
      const PredictionUnit *puNeighLBNew = cs.getPURestricted(posLBNew, pu, pu.chType);
#if JVET_Y0065_GPM_INTRA
      if (puNeighLBNew && CU::isInter(*puNeighLBNew->cu) && puNeighLBNew->getMotionInfo(posLBNew).isInter && PU::isDiffMER(pu.lumaPos(), posLBNew, plevel))
#else
      if (puNeighLBNew && CU::isInter(*puNeighLBNew->cu) && PU::isDiffMER(pu.lumaPos(), posLBNew, plevel))
#endif
      {
        isAvailableNew[2] = true;
        miNew[2] = puNeighLBNew->getMotionInfo(posLBNew);
        posNew[2] = posLBNew;
        CHECK(posLBNew.x < 0 || posLBNew.y < 0, "posLBNew < 0");
      }

      PosType posX = isAvailableNew[2] ? posNew[2].x : (posLT[0].x - (j * ((int)pu.Y().width)));
      PosType posY = isAvailableNew[1] ? posNew[1].y : (posLT[0].y - (i * ((int)pu.Y().height)));
      if (posX < 0)
      {
        posX = isAvailableNew[1] ? posLT[1].x : -1;
      }
      if (posY < 0)
      {
        posY = isAvailableNew[2] ? posLT[2].y : -1;
      }

      const Position        posLTNew = Position(posX, posY);
      const PredictionUnit *puNeighLTNew = cs.getPURestricted(posLTNew, pu, pu.chType);
#if JVET_Y0065_GPM_INTRA
      if (puNeighLTNew && CU::isInter(*puNeighLTNew->cu) && puNeighLTNew->getMotionInfo(posLTNew).isInter && PU::isDiffMER(pu.lumaPos(), posLTNew, plevel))
#else
      if (puNeighLTNew && CU::isInter(*puNeighLTNew->cu) && PU::isDiffMER(pu.lumaPos(), posLTNew, plevel))
#endif
      {
        isAvailableNew[0] = true;
        miNew[0] = puNeighLTNew->getMotionInfo(posLTNew);
        neighBcwNew[0] = puNeighLTNew->cu->bcwIdx;
        posNew[0] = posLTNew;
        CHECK(posLTNew.x < 0 || posLTNew.y < 0, "posLTNew < 0");
      }

      if (!isAvailableNew[0] || (!isAvailableNew[1] && !isAvailableNew[2]))
      {
        continue;
      }
      int  shift = MAX_CU_DEPTH;

      int posNeiX = posNew[0].x;
      int posNeiY = posNew[0].y;
      int posCurX = pu.Y().pos().x;
      int posCurY = pu.Y().pos().y;

      int curW = pu.Y().width;
      int curH = pu.Y().height;
      int neiW = posNew[1].x - posNew[0].x;
      int neiH = posNew[2].y - posNew[0].y;
      if (!isAvailableNew[1])
      {
        neiW = pu.Y().topRight().x - posNew[0].x;
      }
      if (!isAvailableNew[2])
      {
        neiH = pu.Y().bottomLeft().y - posNew[0].y;
      }

      for (int modelIdx = 0; modelIdx < 3; modelIdx++)
      {
        if ((modelIdx == 1 && !isAvailableNew[1]) || (modelIdx == 2 && !isAvailableNew[2]))
        {
          continue;
        }
        Mv cMv[2][3];
        int8_t refIdx[2] = { -1, -1 };
        int dir = 0;
        EAffineModel curType = AFFINEMODEL_6PARAM;
        bool bLICFlag = false;

        for (int l = 0; l < 2; l++)
        {
          Mv mvLT, mvRT, mvLB;
          int iDMvHorX, iDMvHorY, iDMvVerX, iDMvVerY;
          int horTmp, verTmp;

          mvLT = miNew[0].mv[l];
          mvRT = miNew[1].mv[l];
          mvLB = miNew[2].mv[l];

          int iMvScaleHor = mvLT.getHor() << shift;
          int iMvScaleVer = mvLT.getVer() << shift;
          iDMvHorX = getNonAdjAffParaDivFun((mvRT - mvLT).getHor(), neiW);
          iDMvHorY = getNonAdjAffParaDivFun((mvRT - mvLT).getVer(), neiW);
          iDMvVerX = getNonAdjAffParaDivFun((mvLB - mvLT).getHor(), neiH);
          iDMvVerY = getNonAdjAffParaDivFun((mvLB - mvLT).getVer(), neiH);

          if (!modelIdx && isAvailableNew[0] && isAvailableNew[1] && isAvailableNew[2] && miNew[0].refIdx[l] >= 0
            && miNew[0].refIdx[l] == miNew[1].refIdx[l] && miNew[0].refIdx[l] == miNew[2].refIdx[l])
          {
#if INTER_LIC
            bLICFlag = bLICFlag || miNew[0].usesLIC || miNew[1].usesLIC || miNew[2].usesLIC;
#endif
          }
          else if (modelIdx == 1 && isAvailableNew[0] && isAvailableNew[1] && miNew[0].refIdx[l] >= 0
            && miNew[0].refIdx[l] == miNew[1].refIdx[l])
          {
            iDMvVerX = -iDMvHorY;
            iDMvVerY = iDMvHorX;
#if INTER_LIC
            bLICFlag = bLICFlag || miNew[0].usesLIC || miNew[1].usesLIC;
#endif
          }
          else if (modelIdx == 2 && isAvailableNew[0] && isAvailableNew[2] && miNew[0].refIdx[l] >= 0
            && miNew[0].refIdx[l] == miNew[2].refIdx[l])
          {
            iDMvHorX = iDMvVerY;
            iDMvHorY = -iDMvVerX;
#if INTER_LIC
            bLICFlag = bLICFlag || miNew[0].usesLIC || miNew[2].usesLIC;
#endif
          }
          else
          {
            continue;
          }

          dir |= (l + 1);
          refIdx[l] = miNew[0].refIdx[l];

          horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY - posNeiY);
          verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY - posNeiY);
          roundAffineMv(horTmp, verTmp, shift);
          cMv[l][0].hor = horTmp;
          cMv[l][0].ver = verTmp;
          cMv[l][0].clipToStorageBitDepth();

          horTmp = iMvScaleHor + iDMvHorX * (posCurX + curW - posNeiX) + iDMvVerX * (posCurY - posNeiY);
          verTmp = iMvScaleVer + iDMvHorY * (posCurX + curW - posNeiX) + iDMvVerY * (posCurY - posNeiY);
          roundAffineMv(horTmp, verTmp, shift);
          cMv[l][1].hor = horTmp;
          cMv[l][1].ver = verTmp;
          cMv[l][1].clipToStorageBitDepth();
          horTmp = iMvScaleHor + iDMvHorX * (posCurX - posNeiX) + iDMvVerX * (posCurY + curH - posNeiY);
          verTmp = iMvScaleVer + iDMvHorY * (posCurX - posNeiX) + iDMvVerY * (posCurY + curH - posNeiY);
          roundAffineMv(horTmp, verTmp, shift);
          cMv[l][2].hor = horTmp;
          cMv[l][2].ver = verTmp;
          cMv[l][2].clipToStorageBitDepth();
        }

        if (dir != 3)
        {
          continue;
        }
        if (xCPMVSimCheck(pu, affineBMMergeCtx, cMv, dir, refIdx, curType, (dir == 3) ? neighBcwNew[0] : BCW_DEFAULT, (dir != 3) ? bLICFlag : false))
        {
          continue;
        }
        for (int i = 0; i < 3; i++)
        {
          affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 0][i].mv = cMv[0][i];
          affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 0][i].refIdx = refIdx[0];

          affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 1][i].mv = cMv[1][i];
          affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 1][i].refIdx = refIdx[1];
        }
        affineBMMergeCtx.interDirNeighbours[affineBMMergeCtx.numValidMergeCand] = dir;
        affineBMMergeCtx.affineType[affineBMMergeCtx.numValidMergeCand] = curType;
        affineBMMergeCtx.mergeType[affineBMMergeCtx.numValidMergeCand] = MRG_TYPE_DEFAULT_N;
        affineBMMergeCtx.bcwIdx[affineBMMergeCtx.numValidMergeCand] = (dir == 3) ? neighBcwNew[0] : BCW_DEFAULT;
#if INTER_LIC
#if JVET_AD0213_LIC_IMP
        affineBMMergeCtx.licFlags[affineBMMergeCtx.numValidMergeCand] = bLICFlag;
#else
        affineBMMergeCtx.licFlags[affineBMMergeCtx.numValidMergeCand] = (dir != 3) ? bLICFlag : false;
#endif
        
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
#if JVET_AD0213_LIC_IMP
        affineBMMergeCtx.obmcFlags[affineBMMergeCtx.numValidMergeCand] = true;
#else
        affineBMMergeCtx.obmcFlags[affineBMMergeCtx.numValidMergeCand] = (dir != 3) ? !bLICFlag : true;
#endif
#endif
        if (affineBMMergeCtx.interDirNeighbours[affineBMMergeCtx.numValidMergeCand] == 3 && isBiPredFromDifferentDirEqDistPoc(pu, affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 0][0].refIdx, affineBMMergeCtx.mvFieldNeighbours[(affineBMMergeCtx.numValidMergeCand << 1) + 1][0].refIdx))
        {

          // redundancy check
          if (affineBMMergeCtx.xCheckSimilarMotion(affineBMMergeCtx.numValidMergeCand, 0))
          {
            continue;
          }

          if (affineBMMergeCtx.numValidMergeCand == mrgCandIdx) // for decoder
          {
            affineBMMergeCtx.numValidMergeCand++;
            return true;
          }

          affineBMMergeCtx.numValidMergeCand++;

          // early termination
          if (affineBMMergeCtx.numValidMergeCand == maxNumAffineMergeCand)
          {
            return true;
          }
        }
      }
    }
  }
  return false;
}
#endif
#if AFFINE_MMVD
uint8_t PU::getMergeIdxFromAfMmvdBaseIdx(AffineMergeCtx& affMrgCtx, uint16_t afMmvdBaseIdx)
{
  uint8_t mergeIdx = (uint8_t)afMmvdBaseIdx;
  for (int i = 0; i < affMrgCtx.numValidMergeCand; i++)
  {
    if (affMrgCtx.mergeType[i] != MRG_TYPE_DEFAULT_N)
    {
      ++mergeIdx;
    }
    else
    {
      break;
    }
  }

  return mergeIdx;
}

void PU::getAfMmvdMvf(const PredictionUnit& pu, const AffineMergeCtx& affineMergeCtx, MvField mvfMmvd[2][3], const uint16_t afMmvdBase, const uint16_t offsetStep, const uint16_t offsetDir)
{
  CHECK(!pu.cu->affine, "Affine flag is not on for Affine MMVD mode!");
  CHECK(affineMergeCtx.mergeType[afMmvdBase] != MRG_TYPE_DEFAULT_N, "AFF_MMVD base candidate type is not regular Affine!");

#if JVET_AA0093_ENHANCED_MMVD_EXTENSION && !JVET_AA0132_CONFIGURABLE_TM_TOOLS
  static const int32_t refMvdCands[] = { 1, 2, 4, 8 };
#else
  static const int32_t refMvdCands[] = { 1, 2, 4, 8, 16 };
#endif

#if JVET_Y0128_NON_CTC
  const int32_t iPicSize = pu.cu->slice->getPic()->lumaSize().area();
  const int32_t mvShift  = iPicSize < 921600 ? 0 : ( iPicSize < 4096000 ? 2 : MV_FRACTIONAL_BITS_INTERNAL - 1); // 921600 = 1280x720, 4096000 = 2560x1600
#else
  static const int32_t iPicSize = pu.cu->slice->getPic()->lumaSize().area();
  static const int32_t mvShift  = iPicSize < 921600 ? 0 : ( iPicSize < 4096000 ? 2 : MV_FRACTIONAL_BITS_INTERNAL - 1); // 921600 = 1280x720, 4096000 = 2560x1600
#endif
  int step = refMvdCands[offsetStep] << mvShift;
  int affineType = affineMergeCtx.affineType[afMmvdBase];

  uint8_t interDir = affineMergeCtx.interDirNeighbours[afMmvdBase];
  int8_t refIdxL0 = affineMergeCtx.mvFieldNeighbours[(afMmvdBase << 1)    ][0].refIdx;
  int8_t refIdxL1 = affineMergeCtx.mvFieldNeighbours[(afMmvdBase << 1) + 1][0].refIdx;

  Mv baseMv[2][3];
  for (int i = 0; i < 3; i++)
  {
    baseMv[0][i] = affineMergeCtx.mvFieldNeighbours[(afMmvdBase << 1)    ][i].mv;
    baseMv[1][i] = affineMergeCtx.mvFieldNeighbours[(afMmvdBase << 1) + 1][i].mv;
    mvfMmvd[0][i].refIdx = refIdxL0;
    mvfMmvd[1][i].refIdx = refIdxL1;
    mvfMmvd[0][i].mv = Mv();
    mvfMmvd[1][i].mv = Mv();
  }

#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  const int xDir[] = {1, -1,  0,  0,  1, -1,  1, -1, 2, -2,  2, -2, 1,  1, -1, -1};
  const int yDir[] = {0,  0,  1, -1,  1, -1, -1,  1, 1,  1, -1, -1, 2, -2,  2, -2};
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
  int offsetX = xDir[offsetDir % 8]  * step;
  int offsetY = yDir[offsetDir % 8]  * step;
#else
  int offsetX = xDir[offsetDir]  * step;
  int offsetY = yDir[offsetDir] * step;
#endif
#else
  int magY = (offsetDir >> 1) & 0x1;
  int sign = (offsetDir & 0x1) ? -1 : 1;
  int offsetX = (1 - magY) * sign * step;
  int offsetY = (    magY) * sign * step;
#endif
  Mv offsetMv(offsetX, offsetY);

  int numCp = (affineType == AFFINEMODEL_4PARAM) ? 2 : 3;
  for (int cpIdx = 0; cpIdx < numCp; cpIdx++)
  {
    if (interDir == 1)
    {
      mvfMmvd[0][cpIdx].mv = baseMv[0][cpIdx] + offsetMv;
    }
    else if (interDir == 2)
    {
      mvfMmvd[1][cpIdx].mv = baseMv[1][cpIdx] + offsetMv;
    }
#if JVET_AA0093_ENHANCED_MMVD_EXTENSION
    else if (interDir == 3)
    {
      if (offsetDir >= 16
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS
        || !pu.cs->sps->getUseTMMMVD()
#endif
        )
      {
        int poc_cur = pu.cu->slice->getPOC();
        int poc_l0  = pu.cu->slice->getRefPOC(REF_PIC_LIST_0, refIdxL0);
        int poc_l1  = pu.cu->slice->getRefPOC(REF_PIC_LIST_1, refIdxL1);
        int distL0 = poc_l0 - poc_cur;
        int distL1 = poc_l1 - poc_cur;
        mvfMmvd[0][cpIdx].mv =                                                     baseMv[0][cpIdx] + offsetMv;
        mvfMmvd[1][cpIdx].mv = distL0 * distL1 < 0 ? baseMv[1][cpIdx] - offsetMv : baseMv[1][cpIdx] + offsetMv;
      }
      else if (offsetDir >= 8)
      {
        mvfMmvd[0][cpIdx].mv =                                                   baseMv[0][cpIdx] ;
        mvfMmvd[1][cpIdx].mv =                                                   baseMv[1][cpIdx] + offsetMv;
      }
      else
      {
        mvfMmvd[0][cpIdx].mv =                                                   baseMv[0][cpIdx] + offsetMv;
        mvfMmvd[1][cpIdx].mv =                                                   baseMv[1][cpIdx] ;
      }
    }
#else
    else if (interDir == 3)
    {
      int poc_cur = pu.cu->slice->getPOC();
      int poc_l0  = pu.cu->slice->getRefPOC(REF_PIC_LIST_0, refIdxL0);
      int poc_l1  = pu.cu->slice->getRefPOC(REF_PIC_LIST_1, refIdxL1);

      int distL0 = poc_l0 - poc_cur;
      int distL1 = poc_l1 - poc_cur;
      mvfMmvd[0][cpIdx].mv =                                                     baseMv[0][cpIdx] + offsetMv;
      mvfMmvd[1][cpIdx].mv = distL0 * distL1 < 0 ? baseMv[1][cpIdx] - offsetMv : baseMv[1][cpIdx] + offsetMv;
    }
#endif
  }
}

int32_t PU::getAfMmvdEstBits(const PredictionUnit &pu)
{
#if JVET_AA0132_CONFIGURABLE_TM_TOOLS && JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  if (!pu.cs->sps->getUseTMMMVD())
  {
    int baseBits = (ECM3_AF_MMVD_BASE_NUM == 1 ? 0 : pu.afMmvdBaseIdx + (pu.afMmvdBaseIdx == ECM3_AF_MMVD_BASE_NUM - 1 ? 0: 1));
    int stepBits = pu.afMmvdStep + (pu.afMmvdStep == ECM3_AF_MMVD_STEP_NUM - 1 ? 0 : 1);
    int dirBits  = gp_sizeIdxInfo->idxFrom(ECM3_AF_MMVD_OFFSET_DIR);
    return stepBits + dirBits + baseBits;
  }
#endif

  int baseBits = (AF_MMVD_BASE_NUM == 1 ? 0 : pu.afMmvdBaseIdx + (pu.afMmvdBaseIdx == AF_MMVD_BASE_NUM - 1 ? 0: 1));
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  int stepBits = pu.afMmvdMergeIdx + (pu.afMmvdMergeIdx == AF_MMVD_MAX_REFINE_NUM - 1 ? 0 : 1);
  return stepBits + baseBits;
#else
  int stepBits = pu.afMmvdStep + (pu.afMmvdStep == AF_MMVD_STEP_NUM - 1 ? 0 : 1);
  int dirBits  = gp_sizeIdxInfo->idxFrom(AF_MMVD_OFFSET_DIR);
  
  return stepBits + dirBits + baseBits;
#endif
}
#endif

void PU::setAllAffineMvField( PredictionUnit &pu, MvField *mvField, RefPicList eRefList )
{
  // Set Mv
  Mv mv[3];
  for ( int i = 0; i < 3; i++ )
  {
    mv[i] = mvField[i].mv;
  }
  setAllAffineMv( pu, mv[0], mv[1], mv[2], eRefList );

  // Set RefIdx
  CHECK( mvField[0].refIdx != mvField[1].refIdx || mvField[0].refIdx != mvField[2].refIdx, "Affine mv corners don't have the same refIdx." );
  pu.refIdx[eRefList] = mvField[0].refIdx;
}

void PU::setAllAffineMv(PredictionUnit& pu, Mv affLT, Mv affRT, Mv affLB, RefPicList eRefList, bool clipCPMVs)
{
  int width  = pu.Y().width;
  int shift = MAX_CU_DEPTH;
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
  const bool isTranslational = (pu.cu->affineType == AFFINEMODEL_6PARAM) ? (affLT == affRT && affLT == affLB) : (affLT == affRT);
#else
  const bool isTranslational = (affLT == affRT && affLT == affLB);
#endif

  if (clipCPMVs)
  {
    affLT.mvCliptoStorageBitDepth();
    affRT.mvCliptoStorageBitDepth();
    if (pu.cu->affineType == AFFINEMODEL_6PARAM)
    {
      affLB.mvCliptoStorageBitDepth();
    }
  }
  int deltaMvHorX, deltaMvHorY, deltaMvVerX, deltaMvVerY;
  deltaMvHorX = (affRT - affLT).getHor() << (shift - floorLog2(width));
  deltaMvHorY = (affRT - affLT).getVer() << (shift - floorLog2(width));
  int height = pu.Y().height;
  if ( pu.cu->affineType == AFFINEMODEL_6PARAM )
  {
    deltaMvVerX = (affLB - affLT).getHor() << (shift - floorLog2(height));
    deltaMvVerY = (affLB - affLT).getVer() << (shift - floorLog2(height));
  }
  else
  {
    deltaMvVerX = -deltaMvHorY;
    deltaMvVerY = deltaMvHorX;
  }

  int mvScaleHor = affLT.getHor() << shift;
  int mvScaleVer = affLT.getVer() << shift;

  int blockWidth = AFFINE_MIN_BLOCK_SIZE;
  int blockHeight = AFFINE_MIN_BLOCK_SIZE;
  const int halfBW = blockWidth >> 1;
  const int halfBH = blockHeight >> 1;

  MotionBuf mb = pu.getMotionBuf();
  int mvScaleTmpHor, mvScaleTmpVer;
#if !AFFINE_RM_CONSTRAINTS_AND_OPT
  const bool subblkMVSpreadOverLimit = InterPrediction::isSubblockVectorSpreadOverLimit( deltaMvHorX, deltaMvHorY, deltaMvVerX, deltaMvVerY, pu.interDir );
#endif

  if (isTranslational)
  {
    mvScaleTmpHor = mvScaleHor;
    mvScaleTmpVer = mvScaleVer;
    roundAffineMv(mvScaleTmpHor, mvScaleTmpVer, shift);
    Mv curMv(mvScaleTmpHor, mvScaleTmpVer);
    curMv.clipToStorageBitDepth();
    MotionInfo *mi = mb.buf;

    for (int y = 0; y < (pu.Y().height >> MIN_CU_LOG2); y++)
    {
      for (int x = 0; x < (pu.Y().width >> MIN_CU_LOG2); x++)
      {
        mi[x].mv[eRefList] = curMv;
      }
      mi += mb.stride;
    }
  }
#if !AFFINE_RM_CONSTRAINTS_AND_OPT
  else if (subblkMVSpreadOverLimit)
  {
      mvScaleTmpHor = mvScaleHor + deltaMvHorX * ( pu.Y().width >> 1 ) + deltaMvVerX * ( pu.Y().height >> 1 );
      mvScaleTmpVer = mvScaleVer + deltaMvHorY * ( pu.Y().width >> 1 ) + deltaMvVerY * ( pu.Y().height >> 1 );
      roundAffineMv(mvScaleTmpHor, mvScaleTmpVer, shift);
      Mv curMv(mvScaleTmpHor, mvScaleTmpVer);
      curMv.clipToStorageBitDepth();
      MotionInfo *mi = mb.buf;
      for (int y = 0; y < (pu.Y().height >> MIN_CU_LOG2); y++)
      {
        for (int x = 0; x < (pu.Y().width >> MIN_CU_LOG2); x++)
        {
          mi[x].mv[eRefList] = curMv;
        }
        mi += mb.stride;
      }
  }
#endif
  else
  {
    int mvScaleHorLine = mvScaleHor + deltaMvHorX * halfBW + deltaMvVerX * halfBH;
    int mvScaleVerLine = mvScaleVer + deltaMvHorY * halfBW + deltaMvVerY * halfBH;
    int deltaMvVerXLine = deltaMvVerX * blockHeight;
    int deltaMvVerYLine = deltaMvVerY * blockHeight;
    int deltaMvHorXBlk = deltaMvHorX * blockWidth;
    int deltaMvHorYBlk = deltaMvHorY * blockWidth;

    MotionInfo *mi = mb.buf;

    for( int h = 0; h < pu.Y().height; h += blockHeight )
    {
      int mvScaleHorBlk = mvScaleHorLine;
      int mvScaleVerBlk = mvScaleVerLine;

      for( int w = 0; w < pu.Y().width; w += blockWidth )
      {
        mvScaleTmpHor = mvScaleHorBlk;
        mvScaleTmpVer = mvScaleVerBlk;

        mvScaleHorBlk += deltaMvHorXBlk;
        mvScaleVerBlk += deltaMvHorYBlk;

        roundAffineMv( mvScaleTmpHor, mvScaleTmpVer, shift );
        Mv curMv( mvScaleTmpHor, mvScaleTmpVer );
        curMv.clipToStorageBitDepth();

        mi[w >> MIN_CU_LOG2].mv[eRefList] = curMv;
      }

      mi += ( blockHeight >> MIN_CU_LOG2 ) * mb.stride;
      mvScaleHorLine += deltaMvVerXLine;
      mvScaleVerLine += deltaMvVerYLine;
    }
  }

  pu.mvAffi[eRefList][0] = affLT;
  pu.mvAffi[eRefList][1] = affRT;
  pu.mvAffi[eRefList][2] = affLB;
}
#if JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT
void PU::setAffineBdofRefinedMotion(PredictionUnit &pu, Mv* mvBufDecAffineBDOF)
{
  MotionInfo *mi = pu.getMotionBuf().buf;
  const int mbStride = pu.getMotionBuf().stride;
  const int bioSubPuIdxInc = BDOF_SUBPU_STRIDE - (pu.lwidth() >> BDOF_SUBPU_DIM_LOG2);
  int bioSubPuIdx = 0;
  for (int mbBufPosY = 0; mbBufPosY < (pu.lheight() >> 2); mbBufPosY++)
  {
    for (int mbBufPosX = 0; mbBufPosX < (pu.lwidth() >> 2); mbBufPosX++)
    {
      mi[mbBufPosX].mv[0] += mvBufDecAffineBDOF[bioSubPuIdx];
      mi[mbBufPosX].mv[1] -= mvBufDecAffineBDOF[bioSubPuIdx];
      bioSubPuIdx++;
    }
    bioSubPuIdx += bioSubPuIdxInc;
    mi += mbStride;
  }
}
bool PU::checkDoAffineBdofRefine(const PredictionUnit &pu, InterPrediction *interPred)
{
#if JVET_AG0276_LIC_FLAG_SIGNALING
  if (pu.affineOppositeLic)
  {
    return false;
  }
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (pu.mergeType == MRG_TYPE_SUBPU_ATMVP || pu.amvpSbTmvpFlag)
#else
  if (pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
#endif
  {
    return true;
  }
  if (pu.cu->affine == false || pu.cu->bcwIdx != BCW_DEFAULT || pu.cu->licFlag == true)
  {
    return false;
  }
#if JVET_AG0276_NLIC && !JVET_AG0276_LIC_BDOF_BDMVR
  if (pu.cu->altLMFlag)
  {
    return false;
  }
#endif
  const WPScalingParam *wp0 = pu.cu->slice->getWpScaling( REF_PIC_LIST_0, pu.refIdx[0] );
  const WPScalingParam *wp1 = pu.cu->slice->getWpScaling( REF_PIC_LIST_1, pu.refIdx[1] );
  const bool isResamplingPossible = pu.cs->sps->getRprEnabledFlag();
  const bool ref0IsScaled = pu.refIdx[0] < 0 || pu.refIdx[0] >= MAX_NUM_REF
    ? false
    : isResamplingPossible && pu.cu->slice->getRefPic( REF_PIC_LIST_0, pu.refIdx[0] )->isRefScaled( pu.cs->pps );
  const bool ref1IsScaled = pu.refIdx[1] < 0 || pu.refIdx[1] >= MAX_NUM_REF
    ? false
    : isResamplingPossible && pu.cu->slice->getRefPic( REF_PIC_LIST_1, pu.refIdx[1] )->isRefScaled( pu.cs->pps );
  if (WPScalingParam::isWeighted( wp0 ) || WPScalingParam::isWeighted( wp1 ) || ref0IsScaled || ref1IsScaled)
  {
    return false;
  }
  if (pu.affBMMergeFlag)
  {
    return true;
  }
  if (pu.mergeFlag == false)
  {
    if (pu.interDir == 3)
    {
      return (pu.cu->slice->getPairEqualPocDist(pu.refIdx[0], pu.refIdx[1]) == true);
    }
    return false;
  }
  if (pu.interDir != 3 || pu.afMmvdFlag)
  {
    return false;
  }
  return (pu.cu->slice->getPairEqualPocDist(pu.refIdx[0], pu.refIdx[1]) == true);
}
#endif

#if JVET_AG0098_AMVP_WITH_SBTMVP
void PU::clipColPos(int& posX, int& posY, const PredictionUnit& pu)
#else
void clipColPos(int& posX, int& posY, const PredictionUnit& pu)
#endif
{
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  Position puPos = pu.lumaPos();
  int log2CtuSize = floorLog2(pu.cs->sps->getCTUSize());
  int ctuY = ((puPos.y >> log2CtuSize) << log2CtuSize);
  int horMax;
  int horMin;
  const SubPic &curSubPic = pu.cu->slice->getPPS()->getSubPicFromPos(puPos);
  if (curSubPic.getTreatedAsPicFlag())
  {
    horMax = (int)curSubPic.getSubPicRight();
    horMin = (int)curSubPic.getSubPicLeft();
  }
  else
  {
    horMax = (int)pu.cs->pps->getPicWidthInLumaSamples() - 1;
    horMin = 0;
  }
  int verMax = std::min((int)pu.cs->pps->getPicHeightInLumaSamples() - 1, ctuY + (int)pu.cs->sps->getCTUSize() - 1);
  int verMin = std::max((int)0, ctuY);

  posX = std::min(horMax, std::max(horMin, posX));
  posY = std::min(verMax, std::max(verMin, posY));
#else
  Position puPos = pu.lumaPos();
  int log2CtuSize = floorLog2(pu.cs->sps->getCTUSize());
  int ctuX = ((puPos.x >> log2CtuSize) << log2CtuSize);
  int ctuY = ((puPos.y >> log2CtuSize) << log2CtuSize);
  int horMax;
  const SubPic &curSubPic = pu.cu->slice->getPPS()->getSubPicFromPos(puPos);
  if (curSubPic.getTreatedAsPicFlag())
  {
    horMax = std::min((int)curSubPic.getSubPicRight(), ctuX + (int)pu.cs->sps->getCTUSize() + 3);
  }
  else
  {
    horMax = std::min((int)pu.cs->pps->getPicWidthInLumaSamples() - 1, ctuX + (int)pu.cs->sps->getCTUSize() + 3);
  }
  int horMin = std::max((int)0, ctuX);
  int verMax = std::min((int)pu.cs->pps->getPicHeightInLumaSamples() - 1, ctuY + (int)pu.cs->sps->getCTUSize() - 1);
  int verMin = std::max((int)0, ctuY);

  posX = std::min(horMax, std::max(horMin, posX));
  posY = std::min(verMax, std::max(verMin, posY));
#endif
}

#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
void PU::getTMVPCandOpt(const PredictionUnit &pu, RefPicList refList, int refIdx, MergeCtx& mrgCtx, MergeCtx mergeCtxIn, int col)
{
  const uint32_t maxNumMergeCand = NUM_TMVP_CANDS;
  for (uint32_t ui = 0; ui < pu.cs->sps->getMaxNumMergeCand(); ++ui)
  {
    mrgCtx.bcwIdx[ui] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
    mrgCtx.altLMFlag[ui] = false;
    mrgCtx.altLMParaNeighbours[ui].resetAltLinearModel();
#endif
#if INTER_LIC
    mrgCtx.licFlags[ui] = false;
#endif
    mrgCtx.interDirNeighbours[ui] = 0;
    mrgCtx.mvFieldNeighbours[(ui << 1)].refIdx = NOT_VALID;
    mrgCtx.mvFieldNeighbours[(ui << 1) + 1].refIdx = NOT_VALID;
    mrgCtx.useAltHpelIf[ui] = false;
#if MULTI_HYP_PRED
    mrgCtx.addHypNeighbours[ui].clear();
#endif
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
    mrgCtx.candCost[ui] = MAX_UINT64;
#endif
  }
  mrgCtx.numValidMergeCand = 0;
#if JVET_Z0102_NO_ARMC_FOR_ZERO_CAND
  mrgCtx.numCandToTestEnc = 0;
#endif

  const Slice   &slice = *pu.cs->slice;
  const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
  const unsigned mask = ~(scale - 1);
  const Picture *pColPic = slice.getRefPic(RefPicList(col == 0 ? 1 - slice.getColFromL0Flag() : 1 - slice.getColFromL0Flag2nd()), col == 0 ? slice.getColRefIdx() : slice.getColRefIdx2nd());

#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  if (!pColPic->isRefScaled(pu.cs->pps))
    {
#endif
  Mv cTMv;
  for (int i = 0; i < mergeCtxIn.numValidMergeCand; i++)
  {
    if ((mergeCtxIn.interDirNeighbours[i] & (1 << REF_PIC_LIST_0)) && slice.getRefPic(REF_PIC_LIST_0, mergeCtxIn.mvFieldNeighbours[i << 1].refIdx) == pColPic)
    {
      cTMv = mergeCtxIn.mvFieldNeighbours[i << 1].mv;
    }
    else if (slice.isInterB() && (mergeCtxIn.interDirNeighbours[i] & (1 << REF_PIC_LIST_1)) && slice.getRefPic(REF_PIC_LIST_1, mergeCtxIn.mvFieldNeighbours[(i << 1) + 1].refIdx) == pColPic)
    {
      cTMv = mergeCtxIn.mvFieldNeighbours[(i << 1) + 1].mv;
    }
    else
    {
      CHECK(true, "Wrong TMVP candidate");
    }

    ///////////////////////////////////////////////////////////////////////
    ////////          GET Initial Temporal Vector                  ////////
    ///////////////////////////////////////////////////////////////////////

    Mv cTempVector = cTMv;
    Mv cColMv;

    // compute the location of the current PU
    Position puPos = pu.lumaPos();
    Size puSize = pu.lumaSize();
    Position centerPos;
    cTempVector = cTMv;

    cTempVector.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
    int tempX = cTempVector.getHor();
    int tempY = cTempVector.getVer();

    centerPos.x = puPos.x + (puSize.width >> 1) + tempX;
    centerPos.y = puPos.y + (puSize.height >> 1) + tempY;

    clipColPos(centerPos.x, centerPos.y, pu);

    centerPos = Position{ PosType(centerPos.x & mask), PosType(centerPos.y & mask) };

    // derivation of center motion parameters from the collocated CU
    const MotionInfo &mi = pColPic->cs->getMotionInfo(centerPos);

    if (mi.isInter && mi.isIBCmot == false)
    {
      if (getColocatedMVP(pu, refList, centerPos, cColMv, refIdx, false
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
        , col
#endif
      ))
      {
        // set as default, for further motion vector field spanning
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
        mrgCtx.mvFieldNeighbours[(mrgCtx.numValidMergeCand << 1) + refList].setMvField(cColMv, refIdx);
        mrgCtx.mvFieldNeighbours[(mrgCtx.numValidMergeCand << 1) + 1 - refList].setMvField(Mv(), NOT_VALID);
#else
        mrgCtx.mvFieldNeighbours[(mrgCtx.numValidMergeCand << 1) + currRefListId].setMvField(cColMv, 0);
#endif
        mrgCtx.interDirNeighbours[mrgCtx.numValidMergeCand] = refList + 1;
        mrgCtx.bcwIdx[mrgCtx.numValidMergeCand] = BCW_DEFAULT;
      }
#if NON_ADJACENT_MRG_CAND || TM_MRG
      if (!mrgCtx.xCheckSimilarMotion(mrgCtx.numValidMergeCand))
      {
#endif
        mrgCtx.numValidMergeCand++;
        if (mrgCtx.numValidMergeCand == maxNumMergeCand)
        {
          break;
        }
#if NON_ADJACENT_MRG_CAND || TM_MRG
      }
#endif
      }
    }
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    }
#endif
    }
#endif

#if JVET_AG0098_AMVP_WITH_SBTMVP
void PU::getAmvpSbTmvp(PredictionUnit &pu, MergeCtx& mrgCtx, const Mv mvShift, const bool useAmvpSbTmvpBuf, const Position bufTL, bool* tmvpBufValid, MotionInfo* tmvpMotionBuf)
{
  const Slice   &slice = *pu.cs->slice;
  const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
  const unsigned mask = ~(scale - 1);
  const RefPicList colRefList = RefPicList(pu.colIdx == 0 ? (slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0) : (slice.isInterB() ? 1 - slice.getColFromL0Flag2nd() : 0));
  const Picture *pColPic = slice.getRefPic(colRefList, pu.colIdx == 0 ? slice.getColRefIdx() : slice.getColRefIdx2nd());

  Position puPos = pu.lumaPos();
  Size puSize = pu.lumaSize();
  Mv intMvShift = mvShift;
  intMvShift.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
  
  // get CU center tmvp
  Position centerPos;
  centerPos.x = puPos.x + (puSize.width >> 1) + intMvShift.hor;
  centerPos.y = puPos.y + (puSize.height >> 1) + intMvShift.ver;
  clipColPos(centerPos.x, centerPos.y, pu);
  centerPos = Position{ PosType(centerPos.x & mask), PosType(centerPos.y & mask) };
  const MotionInfo &mi = pColPic->cs->getMotionInfo(centerPos);
  Mv cColMv;
  int refIdx = 0;
  MotionInfo centerMi;

  if (useAmvpSbTmvpBuf)
  {
    int bufIdx = (centerPos.x >> ATMVP_SUB_BLOCK_SIZE) - bufTL.x + ((centerPos.y >> ATMVP_SUB_BLOCK_SIZE) - bufTL.y) * AMVP_SBTMVP_BUF_STRIDE;
    if (tmvpBufValid[bufIdx])
    {
      if (tmvpMotionBuf[bufIdx].isInter)
      {
        centerMi = tmvpMotionBuf[bufIdx];
      }
      else
      {
        centerMi.isInter = true;
        centerMi.sliceIdx = slice.getIndependentSliceIdx();
        centerMi.isIBCmot = false;
        centerMi.interDir = colRefList == REF_PIC_LIST_0 ? 1 : 2;
        centerMi.mv[colRefList] = mvShift;
        centerMi.mv[1 - colRefList] = Mv(0, 0);
        centerMi.refIdx[colRefList] = 0;
        centerMi.refIdx[1 - colRefList] = NOT_VALID;
      }
    }
    else
    {
      const MotionInfo &mi = pColPic->cs->getMotionInfo(centerPos);
      centerMi.isInter = true;
      centerMi.sliceIdx = slice.getIndependentSliceIdx();
      centerMi.isIBCmot = false;
      if (mi.isInter && mi.isIBCmot == false)
      {
        for (unsigned currRefListId = 0; currRefListId < (slice.isInterB() ? 2 : 1); currRefListId++)
        {
          RefPicList  currRefPicList = RefPicList(currRefListId);
          refIdx = 0;
          if (getColocatedMVP(pu, currRefPicList, centerPos, cColMv, refIdx, true, pu.colIdx))
          {
            centerMi.interDir |= (1 << currRefListId);
            centerMi.mv[currRefPicList] = cColMv;
            centerMi.refIdx[currRefPicList] = refIdx;
          }
        }
        tmvpMotionBuf[bufIdx] = centerMi;
      }
      else
      {
        centerMi.interDir = colRefList == REF_PIC_LIST_0 ? 1 : 2;
        centerMi.mv[colRefList] = mvShift;
        centerMi.mv[1 - colRefList] = Mv(0, 0);
        centerMi.refIdx[colRefList] = 0;
        centerMi.refIdx[1 - colRefList] = NOT_VALID;
        tmvpMotionBuf[bufIdx].isInter = false;
      }
      tmvpBufValid[bufIdx] = true;
    }
  }
  else
  {
    centerMi.isInter = true;
    centerMi.sliceIdx = slice.getIndependentSliceIdx();
    centerMi.isIBCmot = false;
    if (mi.isInter && mi.isIBCmot == false)
    {
      for (unsigned currRefListId = 0; currRefListId < (slice.isInterB() ? 2 : 1); currRefListId++)
      {
        RefPicList  currRefPicList = RefPicList(currRefListId);
        refIdx = 0;
        if (getColocatedMVP(pu, currRefPicList, centerPos, cColMv, refIdx, true, pu.colIdx))
        {
          centerMi.interDir |= (1 << currRefListId);
          centerMi.mv[currRefPicList] = cColMv;
          centerMi.refIdx[currRefPicList] = refIdx;
        }
      }
    }
    else
    {
      centerMi.interDir = colRefList == REF_PIC_LIST_0 ? 1 : 2;
      centerMi.mv[colRefList] = mvShift;
      centerMi.mv[1 - colRefList] = Mv(0, 0);
      centerMi.refIdx[colRefList] = 0;
      centerMi.refIdx[1 - colRefList] = NOT_VALID;
    }
  }
  
  int numPartLine = std::max(puSize.width >> ATMVP_SUB_BLOCK_SIZE, 1u);
  int numPartCol = std::max(puSize.height >> ATMVP_SUB_BLOCK_SIZE, 1u);
  int puHeight = numPartCol == 1 ? puSize.height : 1 << ATMVP_SUB_BLOCK_SIZE;
  int puWidth = numPartLine == 1 ? puSize.width : 1 << ATMVP_SUB_BLOCK_SIZE;
  int xOff = (puWidth >> 1) + intMvShift.hor;
  int yOff = (puHeight >> 1) + intMvShift.ver;
  if (useAmvpSbTmvpBuf)
  {
    MotionBuf mb = pu.getMotionBuf();
    for (int y = puPos.y; y < puPos.y + puSize.height; y += puHeight)
    {
      for (int x = puPos.x; x < puPos.x + puSize.width; x += puWidth)
      {
        Position colPos{ x + xOff, y + yOff };
        clipColPos(colPos.x, colPos.y, pu);
        colPos = Position{ PosType(colPos.x & mask), PosType(colPos.y & mask) };
        int bufIdx = (colPos.x >> ATMVP_SUB_BLOCK_SIZE) - bufTL.x + ((colPos.y >> ATMVP_SUB_BLOCK_SIZE) - bufTL.y) * AMVP_SBTMVP_BUF_STRIDE;
        MotionInfo mi;
        if (tmvpBufValid[bufIdx])
        {
          mi = tmvpMotionBuf[bufIdx].isInter ? tmvpMotionBuf[bufIdx] : centerMi;
        }
        else
        {
          const MotionInfo &colMi = pColPic->cs->getMotionInfo(colPos);
          if (colMi.isInter && colMi.isIBCmot == false)
          {
            mi.isInter = true;
            mi.sliceIdx = slice.getIndependentSliceIdx();
            mi.isIBCmot = false;
            for (unsigned currRefListId = 0; currRefListId < (slice.isInterB() ? 2 : 1); currRefListId++)
            {
              RefPicList currRefPicList = RefPicList(currRefListId);
              refIdx = 0;
              if (getColocatedMVP(pu, currRefPicList, colPos, cColMv, refIdx, true, pu.colIdx))
              {
                mi.refIdx[currRefListId] = refIdx;
                mi.mv[currRefListId] = cColMv;
              }
            }
            mi.interDir = (mi.refIdx[0] != -1 ? 1 : 0) + (mi.refIdx[1] != -1 ? 2 : 0);
            tmvpMotionBuf[bufIdx] = mi;
          }
          else
          {
            mi = centerMi;
            tmvpMotionBuf[bufIdx].isInter = false;
          }
          tmvpBufValid[bufIdx] = true;
        }
        mb.subBuf(g_miScaling.scale(Position{ x, y } -pu.lumaPos()), g_miScaling.scale(Size(puWidth, puHeight))).fill(mi);
      }
    }
  }
  else
  {
    MotionBuf &mb = mrgCtx.subPuMvpMiBuf[AMVP_SBTMVP_BUFFER_IDX];
    for (int y = puPos.y; y < puPos.y + puSize.height; y += puHeight)
    {
      for (int x = puPos.x; x < puPos.x + puSize.width; x += puWidth)
      {
        Position colPos{ x + xOff, y + yOff };
        clipColPos(colPos.x, colPos.y, pu);
        colPos = Position{ PosType(colPos.x & mask), PosType(colPos.y & mask) };
        const MotionInfo &colMi = pColPic->cs->getMotionInfo(colPos);

        MotionInfo mi;

        if (colMi.isInter && colMi.isIBCmot == false)
        {
          mi.isInter = true;
          mi.sliceIdx = slice.getIndependentSliceIdx();
          mi.isIBCmot = false;
          for (unsigned currRefListId = 0; currRefListId < (slice.isInterB() ? 2 : 1); currRefListId++)
          {
            RefPicList currRefPicList = RefPicList(currRefListId);
            refIdx = 0;
            if (getColocatedMVP(pu, currRefPicList, colPos, cColMv, refIdx, true, pu.colIdx))
            {
              mi.refIdx[currRefListId] = refIdx;
              mi.mv[currRefListId] = cColMv;
            }
          }
          mi.interDir = (mi.refIdx[0] != -1 ? 1 : 0) + (mi.refIdx[1] != -1 ? 2 : 0);
        }
        else
        {
          mi = centerMi;
        }
        mb.subBuf(g_miScaling.scale(Position{ x, y } -pu.lumaPos()), g_miScaling.scale(Size(puWidth, puHeight))).fill(mi);
      }
    }
  }
}
#endif

bool PU::getInterMergeSubPuMvpCand(const PredictionUnit &pu, MergeCtx& mrgCtx, bool& LICFlag, const int count
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION && JVET_AF0163_TM_SUBBLOCK_REFINEMENT
  , bool isRefined
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION 
  , int subIdx, MergeCtx mergeCtxIn
  , int col
#else
  , int mmvdList
#endif
)
{
  const Slice   &slice = *pu.cs->slice;
  const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
  const unsigned mask = ~(scale - 1);
#if INTER_LIC
  LICFlag = false;
#endif

#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  const Picture *pColPic = slice.getRefPic(RefPicList(col == 0 ? 1 - slice.getColFromL0Flag() : 1 - slice.getColFromL0Flag2nd()), col == 0 ? slice.getColRefIdx() : slice.getColRefIdx2nd());
  if (pColPic->isRefScaled(pu.cs->pps))
  {
    return false;
  }
#else
  const Picture *pColPic = slice.getRefPic(RefPicList(slice.isInterB() ? 1 - slice.getColFromL0Flag() : 0), slice.getColRefIdx());
#endif
  Mv cTMv;

#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  if (subIdx > mergeCtxIn.numValidMergeCand)
  {
    return false;
  }
  else if (subIdx < mergeCtxIn.numValidMergeCand)
  {
    if ((mergeCtxIn.interDirNeighbours[subIdx] & (1 << REF_PIC_LIST_0)) && slice.getRefPic(REF_PIC_LIST_0, mergeCtxIn.mvFieldNeighbours[subIdx << 1].refIdx) == pColPic)
    {
      cTMv = mergeCtxIn.mvFieldNeighbours[subIdx << 1].mv;
    }
    else if (slice.isInterB() && (mergeCtxIn.interDirNeighbours[subIdx] & (1 << REF_PIC_LIST_1)) && slice.getRefPic(REF_PIC_LIST_1, mergeCtxIn.mvFieldNeighbours[(subIdx << 1) + 1].refIdx) == pColPic)
    {
      cTMv = mergeCtxIn.mvFieldNeighbours[(subIdx << 1) + 1].mv;
    }
  }

#else
  if (count)
  {
    if ((mrgCtx.interDirNeighbours[0] & (1 << REF_PIC_LIST_0)) && slice.getRefPic(REF_PIC_LIST_0, mrgCtx.mvFieldNeighbours[REF_PIC_LIST_0].refIdx) == pColPic)
    {
      cTMv = mrgCtx.mvFieldNeighbours[REF_PIC_LIST_0].mv;
    }
    else if (slice.isInterB() && (mrgCtx.interDirNeighbours[0] & (1 << REF_PIC_LIST_1)) && slice.getRefPic(REF_PIC_LIST_1, mrgCtx.mvFieldNeighbours[REF_PIC_LIST_1].refIdx) == pColPic)
    {
      cTMv = mrgCtx.mvFieldNeighbours[REF_PIC_LIST_1].mv;
    }
  }
#endif

  ///////////////////////////////////////////////////////////////////////
  ////////          GET Initial Temporal Vector                  ////////
  ///////////////////////////////////////////////////////////////////////

  Mv cTempVector = cTMv;
  bool  tempLICFlag = false;

  // compute the location of the current PU
  Position puPos = pu.lumaPos();
  Size puSize = pu.lumaSize();
  int numPartLine = std::max(puSize.width >> ATMVP_SUB_BLOCK_SIZE, 1u);
  int numPartCol = std::max(puSize.height >> ATMVP_SUB_BLOCK_SIZE, 1u);
  int puHeight = numPartCol == 1 ? puSize.height : 1 << ATMVP_SUB_BLOCK_SIZE;
  int puWidth = numPartLine == 1 ? puSize.width : 1 << ATMVP_SUB_BLOCK_SIZE;

  Mv cColMv;
  int refIdx = 0;
  // use coldir.
  bool     bBSlice = slice.isInterB();

  Position centerPos;

  bool found = false;
  cTempVector = cTMv;

  cTempVector.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
  int tempX = cTempVector.getHor();
  int tempY = cTempVector.getVer();

  centerPos.x = puPos.x + (puSize.width >> 1) + tempX;
  centerPos.y = puPos.y + (puSize.height >> 1) + tempY;

  clipColPos(centerPos.x, centerPos.y, pu);

  centerPos = Position{ PosType(centerPos.x & mask), PosType(centerPos.y & mask) };

  // derivation of center motion parameters from the collocated CU
  const MotionInfo &mi = pColPic->cs->getMotionInfo(centerPos);

  if (mi.isInter && mi.isIBCmot == false)
  {
    mrgCtx.interDirNeighbours[count] = 0;

    for (unsigned currRefListId = 0; currRefListId < (bBSlice ? 2 : 1); currRefListId++)
    {
      RefPicList  currRefPicList = RefPicList(currRefListId);

      if (getColocatedMVP(pu, currRefPicList, centerPos, cColMv, refIdx, true
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
        , col
#endif
        , &refIdx
#endif
      ))
      {
        // set as default, for further motion vector field spanning
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
        mrgCtx.mvFieldNeighbours[(count << 1) + currRefListId].setMvField(cColMv, refIdx);
#else
        mrgCtx.mvFieldNeighbours[(count << 1) + currRefListId].setMvField(cColMv, 0);
#endif
        mrgCtx.interDirNeighbours[count] |= (1 << currRefListId);
        LICFlag = tempLICFlag;
        mrgCtx.bcwIdx[count] = BCW_DEFAULT;
        found = true;
      }
      else
      {
        mrgCtx.mvFieldNeighbours[(count << 1) + currRefListId].setMvField(Mv(), NOT_VALID);
        mrgCtx.interDirNeighbours[count] &= ~(1 << currRefListId);
      }
    }
  }
  if (!found)
  {
    return false;
  }
#if !JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  if (mmvdList != 1)
  {
#endif
    int xOff = (puWidth >> 1) + tempX;
    int yOff = (puHeight >> 1) + tempY;

#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
#if JVET_AF0163_TM_SUBBLOCK_REFINEMENT
    MotionBuf *mb;
    if (isRefined)
    {
      mb = &(mrgCtx.subPuMvpMiBuf[3]);
    }
    else
    {
      mb = &(mrgCtx.subPuMvpMiBuf[(subIdx << 1) + col]);
    }
#else
    MotionBuf &mb = mrgCtx.subPuMvpMiBuf[(subIdx << 1) + col];
#endif
#else
    MotionBuf &mb = mrgCtx.subPuMvpMiBuf;
#endif

    const bool isBiPred = isBipredRestriction(pu);

    for (int y = puPos.y; y < puPos.y + puSize.height; y += puHeight)
    {
      for (int x = puPos.x; x < puPos.x + puSize.width; x += puWidth)
      {
        Position colPos{ x + xOff, y + yOff };

        clipColPos(colPos.x, colPos.y, pu);

        colPos = Position{ PosType(colPos.x & mask), PosType(colPos.y & mask) };

        const MotionInfo &colMi = pColPic->cs->getMotionInfo(colPos);

        MotionInfo mi;

        found = false;
        mi.isInter = true;
        mi.sliceIdx = slice.getIndependentSliceIdx();
        mi.isIBCmot = false;
        if (colMi.isInter && colMi.isIBCmot == false)
        {
          for (unsigned currRefListId = 0; currRefListId < (bBSlice ? 2 : 1); currRefListId++)
          {
            RefPicList currRefPicList = RefPicList(currRefListId);
            if (getColocatedMVP(pu, currRefPicList, colPos, cColMv, refIdx, true
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
              , col
#endif
              , &refIdx
#endif
            ))
            {
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING
              mi.refIdx[currRefListId] = refIdx;
#else
              mi.refIdx[currRefListId] = 0;
#endif
              mi.mv[currRefListId] = cColMv;
              found = true;
            }
          }
        }
        if (!found)
        {
          mi.mv[0] = mrgCtx.mvFieldNeighbours[(count << 1) + 0].mv;
          mi.mv[1] = mrgCtx.mvFieldNeighbours[(count << 1) + 1].mv;
          mi.refIdx[0] = mrgCtx.mvFieldNeighbours[(count << 1) + 0].refIdx;
          mi.refIdx[1] = mrgCtx.mvFieldNeighbours[(count << 1) + 1].refIdx;
        }

        mi.interDir = (mi.refIdx[0] != -1 ? 1 : 0) + (mi.refIdx[1] != -1 ? 2 : 0);

        if (isBiPred && mi.interDir == 3)
        {
          mi.interDir = 1;
          mi.mv[1] = Mv();
          mi.refIdx[1] = NOT_VALID;
        }
        mi.bcwIdx = BCW_DEFAULT;
#if JVET_AF0163_TM_SUBBLOCK_REFINEMENT
        mb->subBuf(g_miScaling.scale(Position{ x, y } -pu.lumaPos()), g_miScaling.scale(Size(puWidth, puHeight)))
          .fill(mi);
#else
        mb.subBuf(g_miScaling.scale(Position{ x, y } -pu.lumaPos()), g_miScaling.scale(Size(puWidth, puHeight)))
          .fill(mi);
#endif
      }
    }
#if !JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  }
#endif
  return true;
}

#if MULTI_PASS_DMVR
void PU::spanMotionInfo(PredictionUnit &pu, const MergeCtx &mrgCtx,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  int colIdx,
#endif
  Mv* bdmvrSubPuMv0, Mv* bdmvrSubPuMv1, Mv* bdofSubPuMvOffset)
#else
void PU::spanMotionInfo( PredictionUnit &pu, const MergeCtx &mrgCtx )
#endif
{
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  CHECK(pu.cu->geoBlendFlag, "PU::spanMotionInfo( geoBlendFlag = 1 )");
#endif
#if !MULTI_PASS_DMVR
  MotionBuf mb = pu.getMotionBuf();
#endif
#if JVET_W0123_TIMD_FUSION
  IpmBuf ib = pu.getIpmBuf();
#endif

  if (!pu.mergeFlag || pu.mergeType == MRG_TYPE_DEFAULT_N || pu.mergeType == MRG_TYPE_IBC)
  {
    MotionInfo mi;

#if JVET_AB0061_ITMP_BV_FOR_IBC
    mi.isInter  = !CU::isIntra(*pu.cu) || pu.cu->tmpFlag;
    mi.isIBCmot = CU::isIBC(*pu.cu) || pu.cu->tmpFlag;
#else
    mi.isInter = !CU::isIntra(*pu.cu);
    mi.isIBCmot = CU::isIBC(*pu.cu);
#endif
    mi.sliceIdx = pu.cu->slice->getIndependentSliceIdx();
#if INTER_LIC
    mi.usesLIC = pu.cu->licFlag;
#endif
#if JVET_AC0112_IBC_LIC
#if JVET_AG0136_INTRA_TMP_LIC
    mi.useIbcLic = mi.isIBCmot ? (pu.cu->ibcLicFlag || pu.cu->tmpLicFlag): 0;
#else
    mi.useIbcLic = mi.isIBCmot ? pu.cu->ibcLicFlag : 0;
#endif
#endif
#if JVET_AE0159_FIBC
    mi.useIbcFilter = mi.isIBCmot ? pu.cu->ibcFilterFlag : false;
#endif
#if JVET_AA0070_RRIBC
    mi.rribcFlipType = mi.isIBCmot ? pu.cu->rribcFlipType : 0;
#endif
    mi.bcwIdx = pu.cu->bcwIdx;

    if( mi.isInter )
    {
      mi.interDir = pu.interDir;
      mi.useAltHpelIf = pu.cu->imv == IMV_HPEL;
      for( int i = 0; i < NUM_REF_PIC_LIST_01; i++ )
      {
        mi.mv[i]     = pu.mv[i];
        mi.refIdx[i] = pu.refIdx[i];
      }
      if (mi.isIBCmot)
      {
        mi.bv = pu.bv;
      }
    }

#if MULTI_PASS_DMVR
    if (pu.bdmvrRefine)
    {
      CHECK(bdmvrSubPuMv0 == nullptr, "this is not possible");
      const int dx = std::min<int>(pu.lwidth (), BDOF_SUBPU_DIM);
      const int dy = std::min<int>(pu.lheight(), BDOF_SUBPU_DIM);
      int subPuIdx = 0;
      const int bioSubPuIdxStrideIncr = BDOF_SUBPU_STRIDE - std::max(1, (int)(pu.lwidth() >> BDOF_SUBPU_DIM_LOG2));

      for (int yStart = 0; yStart < pu.lheight(); yStart += dy)
      {
        for (int xStart = 0; xStart < pu.lwidth(); xStart += dx)
        {
          const int bdmvrSubPuIdx = (yStart >> DMVR_SUBCU_HEIGHT_LOG2) * DMVR_SUBPU_STRIDE + (xStart >> DMVR_SUBCU_WIDTH_LOG2);
#if JVET_Y0089_DMVR_BCW
#if JVET_AG0067_DMVR_EXTENSIONS
          if ((pu.cu->bcwIdx != BCW_DEFAULT) || (PU::isBiPredFromDifferentDirGenDistPoc(pu) && (!PU::isBiPredFromDifferentDirEqDistPoc( pu ))))
#else
          if (pu.cu->bcwIdx != BCW_DEFAULT)
#endif
          {
            mi.mv[0] = bdmvrSubPuMv0[bdmvrSubPuIdx];
            mi.mv[1] = bdmvrSubPuMv1[bdmvrSubPuIdx];
          }
          else
#endif
          {
            mi.mv[0] = bdmvrSubPuMv0[bdmvrSubPuIdx] + bdofSubPuMvOffset[subPuIdx];
            mi.mv[1] = bdmvrSubPuMv1[bdmvrSubPuIdx] - bdofSubPuMvOffset[subPuIdx];
          }

          subPuIdx++;
          MotionBuf mb = pu.cs->getMotionBuf(Area(pu.lx() + xStart, pu.ly() + yStart, dx, dy));
          mb.fill(mi);
        }
        subPuIdx += bioSubPuIdxStrideIncr;
      }
#if JVET_W0123_TIMD_FUSION
      MotionBuf mb = pu.getMotionBuf();
      spanIpmInfoInter(pu, mb, ib);
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
      spanSCCInfo(pu);
#endif
      return;
    }
    MotionBuf mb = pu.getMotionBuf();
#endif
    if (pu.cu->affine)
    {
      mi.mv[0].setZero(); // to make sure filling of MV in unused reference list
      mi.mv[1].setZero();
      mb.fill(mi);
      if (pu.refIdx[0] >= 0)
      {
        PU::setAllAffineMv(pu, pu.mvAffi[0][0], pu.mvAffi[0][1], pu.mvAffi[0][2], REF_PIC_LIST_0);
      }
      if (pu.refIdx[1] >= 0)
      {
        PU::setAllAffineMv(pu, pu.mvAffi[1][0], pu.mvAffi[1][1], pu.mvAffi[1][2], REF_PIC_LIST_1);
      }
#if JVET_W0123_TIMD_FUSION
      spanIpmInfoInter(pu, mb, ib);
#endif
    }
#if JVET_AG0098_AMVP_WITH_SBTMVP
    else if (pu.amvpSbTmvpFlag)
    {
      CHECK(mrgCtx.subPuMvpMiBuf[AMVP_SBTMVP_BUFFER_IDX].area() == 0 || !mrgCtx.subPuMvpMiBuf[AMVP_SBTMVP_BUFFER_IDX].buf, "Buffer not initialized");
#if MULTI_PASS_DMVR
      MotionBuf mb = pu.getMotionBuf();
#endif
      mb.copyFrom(mrgCtx.subPuMvpMiBuf[AMVP_SBTMVP_BUFFER_IDX]);
#if JVET_W0123_TIMD_FUSION
      spanIpmInfoInter(pu, mb, ib);
#endif
    }
#endif
    else
    {
      mb.fill(mi);
#if JVET_W0123_TIMD_FUSION
      if (mi.isIBCmot
#if JVET_X0141_CIIP_TIMD_TM
        || pu.ciipFlag
#endif
        )
      {
#if JVET_AC0112_IBC_CIIP
        if (mi.isIBCmot)
        {
          spanIpmInfoIBC(pu, ib, mi.bv.getHor(), mi.bv.getVer());
        }
        else
#endif
        ib.fill(PLANAR_IDX);
      }
      else
      {
        spanIpmInfoInter(pu, mb, ib);
      }
#endif
    }
  }
  else if (pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
  {
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  for (int i = 0; i < SUB_TMVP_NUM; i++)
  {
    CHECK(mrgCtx.subPuMvpMiBuf[i].area() == 0 || !mrgCtx.subPuMvpMiBuf[i].buf, "Buffer not initialized");
  }
#else
  CHECK(mrgCtx.subPuMvpMiBuf.area() == 0 || !mrgCtx.subPuMvpMiBuf.buf, "Buffer not initialized");
#endif
#if MULTI_PASS_DMVR
    MotionBuf mb = pu.getMotionBuf();
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    mb.copyFrom(mrgCtx.subPuMvpMiBuf[pu.colIdx]);
#else
    mb.copyFrom(mrgCtx.subPuMvpMiBuf);
#endif
#if JVET_W0123_TIMD_FUSION
    spanIpmInfoInter(pu, mb, ib);
#endif
  }
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
  spanSCCInfo(pu);
#endif
}

#if JVET_W0123_TIMD_FUSION
#if MULTI_PASS_DMVR
void PU::spanMotionInfo2(PredictionUnit &pu, const MergeCtx &mrgCtx,
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
  int colIdx,
#endif
  Mv* bdmvrSubPuMv0, Mv* bdmvrSubPuMv1, Mv* bdofSubPuMvOffset)
#else
void PU::spanMotionInfo2( PredictionUnit &pu, const MergeCtx &mrgCtx )
#endif
{
#if !MULTI_PASS_DMVR
  MotionBuf mb = pu.getMotionBuf();
#endif

  if (!pu.mergeFlag || pu.mergeType == MRG_TYPE_DEFAULT_N || pu.mergeType == MRG_TYPE_IBC)
  {
    MotionInfo mi;

    mi.isInter = !CU::isIntra(*pu.cu);
    mi.isIBCmot = CU::isIBC(*pu.cu);
    mi.sliceIdx = pu.cu->slice->getIndependentSliceIdx();
#if INTER_LIC
    mi.usesLIC = pu.cu->licFlag;
#endif
    mi.bcwIdx  = pu.cu->bcwIdx;

    if( mi.isInter )
    {
      mi.interDir = pu.interDir;
      mi.useAltHpelIf = pu.cu->imv == IMV_HPEL;
      for( int i = 0; i < NUM_REF_PIC_LIST_01; i++ )
      {
        mi.mv[i]     = pu.mv[i];
        mi.refIdx[i] = pu.refIdx[i];
      }
      if (mi.isIBCmot)
      {
        mi.bv = pu.bv;
      }
    }

#if MULTI_PASS_DMVR
#if JVET_AE0046_BI_GPM
    if ( pu.gpmDmvrRefinePart0 || pu.gpmDmvrRefinePart1 )
    {
      const int dx = std::min<int>(pu.lwidth(), BDOF_SUBPU_DIM);
      const int dy = std::min<int>(pu.lheight(), BDOF_SUBPU_DIM);
      int subPuIdx = 0;
      const int bioSubPuIdxStrideIncr = BDOF_SUBPU_STRIDE - std::max(1, (int)(pu.lwidth() >> BDOF_SUBPU_DIM_LOG2));

      for (int yStart = 0; yStart < pu.lheight(); yStart += dy)
      {
        for (int xStart = 0; xStart < pu.lwidth(); xStart += dx)
        {
          mi.mv[0] = pu.mv[0] + bdofSubPuMvOffset[subPuIdx];
          mi.mv[1] = pu.mv[1] - bdofSubPuMvOffset[subPuIdx];

          subPuIdx++;
          MotionBuf mb = pu.cs->getMotionBuf(Area(pu.lx() + xStart, pu.ly() + yStart, dx, dy));
          mb.fill(mi);
        }
        subPuIdx += bioSubPuIdxStrideIncr;
      }
      return;
    }
#endif
    if (pu.bdmvrRefine)
    {
      CHECK(bdmvrSubPuMv0 == nullptr, "this is not possible");
      const int dx = std::min<int>(pu.lwidth (), BDOF_SUBPU_DIM);
      const int dy = std::min<int>(pu.lheight(), BDOF_SUBPU_DIM);
      int subPuIdx = 0;
      const int bioSubPuIdxStrideIncr = BDOF_SUBPU_STRIDE - std::max(1, (int)(pu.lwidth() >> BDOF_SUBPU_DIM_LOG2));

      for (int yStart = 0; yStart < pu.lheight(); yStart += dy)
      {
        for (int xStart = 0; xStart < pu.lwidth(); xStart += dx)
        {
          const int bdmvrSubPuIdx = (yStart >> DMVR_SUBCU_HEIGHT_LOG2) * DMVR_SUBPU_STRIDE + (xStart >> DMVR_SUBCU_WIDTH_LOG2);
#if JVET_AG0067_DMVR_EXTENSIONS
          if ((PU::isBiPredFromDifferentDirGenDistPoc(pu) && (!PU::isBiPredFromDifferentDirEqDistPoc( pu ))))
          {
            mi.mv[0] = bdmvrSubPuMv0[bdmvrSubPuIdx];
            mi.mv[1] = bdmvrSubPuMv1[bdmvrSubPuIdx];
          }
          else
          {
#endif
            mi.mv[0] = bdmvrSubPuMv0[bdmvrSubPuIdx] + bdofSubPuMvOffset[subPuIdx];
            mi.mv[1] = bdmvrSubPuMv1[bdmvrSubPuIdx] - bdofSubPuMvOffset[subPuIdx];
#if JVET_AG0067_DMVR_EXTENSIONS
          }
#endif

          subPuIdx++;
          MotionBuf mb = pu.cs->getMotionBuf(Area(pu.lx() + xStart, pu.ly() + yStart, dx, dy));
          mb.fill(mi);
        }
        subPuIdx += bioSubPuIdxStrideIncr;
      }
      return;
    }
    MotionBuf mb = pu.getMotionBuf();
#endif
    if (pu.cu->affine)
    {
      mi.mv[0].setZero(); // to make sure filling of MV in unused reference list
      mi.mv[1].setZero();
      mb.fill(mi);
      if (pu.refIdx[0] >= 0)
      {
        PU::setAllAffineMv(pu, pu.mvAffi[0][0], pu.mvAffi[0][1], pu.mvAffi[0][2], REF_PIC_LIST_0);
      }
      if (pu.refIdx[1] >= 0)
      {
        PU::setAllAffineMv(pu, pu.mvAffi[1][0], pu.mvAffi[1][1], pu.mvAffi[1][2], REF_PIC_LIST_1);
      }
    }
    else
    {
      mb.fill(mi);
    }
  }
  else if (pu.mergeType == MRG_TYPE_SUBPU_ATMVP)
  {
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    for (int i = 0; i < SUB_TMVP_NUM; i++)
    {
      CHECK(mrgCtx.subPuMvpMiBuf[i].area() == 0 || !mrgCtx.subPuMvpMiBuf[i].buf, "Buffer not initialized");
    }
#else
    CHECK(mrgCtx.subPuMvpMiBuf.area() == 0 || !mrgCtx.subPuMvpMiBuf.buf, "Buffer not initialized");
#endif
#if MULTI_PASS_DMVR
    MotionBuf mb = pu.getMotionBuf();
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    mb.copyFrom(mrgCtx.subPuMvpMiBuf[pu.colIdx]);
#else
    mb.copyFrom(mrgCtx.subPuMvpMiBuf);
#endif
  }
}

void PU::spanIpmInfoIntra( PredictionUnit &pu)
{
#if JVET_AB0155_SGPM
  if (pu.cu->sgpm)
  {
    spanIpmInfoSgpm(pu);
    return;
  }
#endif

  int ipm = PU::getIntraDirLuma(pu);

  if (pu.cu->timd)
  {
    ipm = MAP131TO67(ipm);
  }
#if JVET_AD0085_TMRL_EXTENSION
  if (pu.cu->tmrlFlag)
  {
    ipm = MAP131TO67(ipm);
  }
#endif
  IpmBuf ib = pu.getIpmBuf();
  ib.fill(ipm);
}
#if JVET_AB0155_SGPM
void PU::spanIpmInfoSgpm(PredictionUnit &pu)
{
  int sgpmMode0 = pu.cu->sgpmMode0;
  int sgpmMode1 = pu.cu->sgpmMode1;
#if JVET_AG0152_SGPM_ITMP_IBC
  if (sgpmMode0 >= SGPM_BV_START_IDX)
  {
    sgpmMode0 = 0;
  }
  if (sgpmMode1 >= SGPM_BV_START_IDX)
  {
    sgpmMode1 = 0;
  }
#endif
  int splitDir  = pu.cu->sgpmSplitDir;

  int16_t angle = g_geoParams[splitDir][0];
  int tpmMask = 0;
  int lookUpY = 0, motionIdx = 0;
  bool isFlip = angle >= 13 && angle <= 27;
  int distanceIdx = g_geoParams[splitDir][1];
  int distanceX = angle;
  int distanceY = (distanceX + (GEO_NUM_ANGLES >> 2)) % GEO_NUM_ANGLES;
  int offsetX = (-(int)pu.lwidth()) >> 1;
  int offsetY = (-(int)pu.lheight()) >> 1;

  if (distanceIdx > 0)
  {
    if( angle % 16 == 8 || ( angle % 16 != 0 && pu.lheight() >= pu.lwidth() ) )
    {
      offsetY += angle < 16 ? ( ( distanceIdx * pu.lheight() ) >> 3 ) : -( int ) ( ( distanceIdx * pu.lheight() ) >> 3 );
    }
    else
    {
      offsetX += angle < 16 ? ( ( distanceIdx * pu.lwidth() ) >> 3 ) : -( int ) ( ( distanceIdx * pu.lwidth() ) >> 3 );
    }
  }

  IpmBuf   ib      = pu.getIpmBuf();
  uint8_t *ipmSgpm = ib.buf;
  
  for (int y = 0; y < ib.height; y++)
  {
    lookUpY = (((4 * y + offsetY) << 1) + 5) * g_dis[distanceY];
    for (int x = 0; x < ib.width; x++)
    {
      motionIdx = (((4 * x + offsetX) << 1) + 5) * g_dis[distanceX] + lookUpY;
      tpmMask = motionIdx <= 0 ? (1 - isFlip) : isFlip;
      if (tpmMask == 0)
      {
        *ipmSgpm = sgpmMode0;
      }
      else
      {
        *ipmSgpm = sgpmMode1;
      }
    }
    ipmSgpm += ib.stride;
  }
}
#endif

#if RPR_ENABLE
void scalePositionInRef( PredictionUnit& pu, const PPS& pps, RefPicList refList, int refIdx, Position& PosY )
{
  const Picture* refPic = pu.cu->slice->getRefPic( refList, refIdx )->unscaledPic;
  const bool scaled = refPic->isRefScaled( &pps );
  if (scaled)
  {
    const SPS* sps = pu.cu->cs->sps;
    ChromaFormat chFmt = sps->getChromaFormatIdc();

    const std::pair<int, int> scalingRatio = pu.cu->slice->getScalingRatio( refList, refIdx );

    int64_t x0Int;
    int64_t y0Int;

    int posX = PosY.x;
    int posY = PosY.y;

    const int posShift = SCALE_RATIO_BITS - 4;

    int shiftHor = MV_FRACTIONAL_BITS_INTERNAL ;
    int shiftVer = MV_FRACTIONAL_BITS_INTERNAL ;
    int offX = 1 << (posShift - shiftHor - 1);
    int offY = 1 << (posShift - shiftVer - 1);

    x0Int = (posX << (4)) * (int64_t)scalingRatio.first ;
    x0Int = SIGN(x0Int) * ((llabs(x0Int) + ((long long)1 << 7)) >> 8) + ((refPic->getScalingWindow().getWindowLeftOffset() * SPS::getWinUnitX(chFmt)) << (posShift));

    y0Int = (posY << (4)) * (int64_t)scalingRatio.second ;
    y0Int = SIGN(y0Int) * ((llabs(y0Int) + ((long long)1 << 7)) >> 8) + ((refPic->getScalingWindow().getWindowTopOffset() * SPS::getWinUnitY(chFmt)) << (posShift));

    int xInt0 = ((int32_t)x0Int + offX) >> posShift;
    int yInt0 = ((int32_t)y0Int + offY) >> posShift;

    PosY.x = std::min( std::max(xInt0, 0), (int)refPic->unscaledPic->getPicWidthInLumaSamples()  - 1 );
    PosY.y = std::min( std::max(yInt0, 0), (int)refPic->unscaledPic->getPicHeightInLumaSamples() - 1 );
  }
  //else
  //  clipColPos( PosY.x, PosY.y, pu );
}
#endif

void PU::spanIpmInfoInter( PredictionUnit &pu, MotionBuf &mb, IpmBuf &ib)
{
  const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
  const unsigned mask = ~(scale - 1);
  Mv cMv;
  RefPicList refList;
  int refIdx;
  Position PosY;
  MotionInfo tempMi;
  MotionInfo mi0;
  MotionInfo mi1;
  Position PosY0;
  Position PosY1;
  Mv cMv0;
  Mv cMv1;
#if RPR_ENABLE
  const Picture* pRefPic0;
  const Picture* pRefPic1;
#else
  Picture* pRefPic0;
  Picture* pRefPic1;
#endif
  uint8_t* ii = ib.buf;
  int ibH = mb.height;
  int ibW = mb.width;
  for (int y = 0; y < ibH; y++)
  {
    for (int x = 0; x < ibW; x++)
    {
      uint8_t ipm = PLANAR_IDX;
      tempMi = mb.at(x, y);
      if (tempMi.interDir != 3)
      {
        if (tempMi.interDir != 2)
        {
          cMv = tempMi.mv[0];
          refList = REF_PIC_LIST_0;
          refIdx = tempMi.refIdx[0];
        }
        else
        {
          cMv = tempMi.mv[1];
          refList = REF_PIC_LIST_1;
          refIdx = tempMi.refIdx[1];
        }
        if (refList < 0 || refIdx < 0)
        {
          ipm = PLANAR_IDX;
        }
        else
        {
          cMv.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
          PosY.x = pu.Y().x + (x << MIN_CU_LOG2) + cMv.getHor();
          PosY.y = pu.Y().y + (y << MIN_CU_LOG2) + cMv.getVer();
          clipColPos(PosY.x, PosY.y, pu);
#if RPR_ENABLE
          scalePositionInRef( pu, *pu.cs->pps, refList, refIdx, PosY );
          PosY.x = (PosY.x & mask);
          PosY.y = (PosY.y & mask);
          ipm = pu.cu->slice->getRefPic(refList, refIdx)->unscaledPic->cs->getIpmInfo(PosY);
#else
          PosY.x = (PosY.x & mask);
          PosY.y = (PosY.y & mask);
          ipm = pu.cu->slice->getRefPic(refList, refIdx)->cs->getIpmInfo(PosY);
#endif
        }
      }
      else
      {
#if RPR_ENABLE
        pRefPic0 = pu.cu->slice->getRefPic(REF_PIC_LIST_0, tempMi.refIdx[0])->unscaledPic;
#else
        pRefPic0 = pu.cu->slice->getRefPic(REF_PIC_LIST_0, tempMi.refIdx[0]);
#endif
        cMv0 = tempMi.mv[0];
        cMv0.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
        PosY0.x = pu.Y().x + (x << MIN_CU_LOG2) + cMv0.getHor();
        PosY0.y = pu.Y().y + (y << MIN_CU_LOG2) + cMv0.getVer();
        clipColPos(PosY0.x, PosY0.y, pu);
#if RPR_ENABLE
        scalePositionInRef( pu, *pu.cs->pps, REF_PIC_LIST_0, tempMi.refIdx[0], PosY0 );
#endif
        PosY0.x = (PosY0.x & mask);
        PosY0.y = (PosY0.y & mask);
        mi0 = pRefPic0->cs->getMotionInfo(PosY0);
        int ipm0 = pRefPic0->cs->getIpmInfo(PosY0);
        int pocDiff0 = abs(pRefPic0->getPOC() - pu.cu->slice->getPOC());

#if RPR_ENABLE
        pRefPic1 = pu.cu->slice->getRefPic(REF_PIC_LIST_1, tempMi.refIdx[1])->unscaledPic;
#else
        pRefPic1 = pu.cu->slice->getRefPic(REF_PIC_LIST_1, tempMi.refIdx[1]);
#endif
        cMv1 = tempMi.mv[1];
        cMv1.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
        PosY1.x = pu.Y().x + (x << MIN_CU_LOG2) + cMv1.getHor();
        PosY1.y = pu.Y().y + (y << MIN_CU_LOG2) + cMv1.getVer();
        clipColPos(PosY1.x, PosY1.y, pu);
#if RPR_ENABLE
        scalePositionInRef( pu, *pu.cs->pps, REF_PIC_LIST_1, tempMi.refIdx[1], PosY1 );
#endif
        PosY1.x = (PosY1.x & mask);
        PosY1.y = (PosY1.y & mask);
        mi1 = pRefPic1->cs->getMotionInfo(PosY1);
        int ipm1 = pRefPic1->cs->getIpmInfo(PosY1);
        int pocDiff1 = abs(pRefPic1->getPOC() - pu.cu->slice->getPOC());

        if (!mi0.isInter && mi1.isInter)
        {
          ipm = ipm0;
        }
        else if (!mi1.isInter && mi0.isInter)
        {
          ipm = ipm1;
        }
        else if (ipm0 > DC_IDX && ipm1 <= DC_IDX)
        {
          ipm = ipm0;
        }
        else if (ipm0 <= DC_IDX && ipm1 > DC_IDX)
        {
          ipm = ipm1;
        }
        else if (pocDiff0 < pocDiff1)
        {
          ipm = ipm0;
        }
        else if (pocDiff1 < pocDiff0)
        {
          ipm = ipm1;
        }
        else if (pRefPic0->m_prevQP[0] > pRefPic1->m_prevQP[0])
        {
          ipm = ipm1;
        }
        else
        {
          ipm = ipm0;
        }
      }
      ii[x] = ipm;
    }
    ii += ib.stride;
  }
}
#endif

#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
void PU::spanSCCInfo(PredictionUnit &pu)
{
  if (!pu.cs->sps->getIBCFlag() && !pu.cs->sps->getPLTMode() && !pu.cs->sps->getBDPCMEnabledFlag())
  {
    return;
  }

  MotionBuf   mb         = pu.getMotionBuf();
  MotionInfo *motionInfo = mb.buf;

  const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
  const unsigned mask  = ~(scale - 1);
  Mv             cMv, cMv0, cMv1;
  RefPicList     refList;
  int            refIdx;
  Position       posY, posY0, posY1;
#if RPR_ENABLE
  const Picture *pRefPic0;
  const Picture *pRefPic1;
#else
  Picture *pRefPic0;
  Picture *pRefPic1;
#endif

  for (int y = 0; y < mb.height; y++)
  {
    for (int x = 0; x < mb.width; x++)
    {
      motionInfo[x].isRefSCC    = false;
      motionInfo[x].isRefRefSCC = false;
      motionInfo[x].isSCC       = CU::isIBC(*pu.cu) || CU::isPLT(*pu.cu) || (pu.cu->bdpcmMode > 0);

      if (CU::isInter(*pu.cu) && motionInfo[x].isInter && (motionInfo[x].interDir == 1 || motionInfo[x].interDir == 2))
      {
        cMv     = motionInfo[x].mv[motionInfo[x].interDir - 1];
        refList = motionInfo[x].interDir == 1 ? REF_PIC_LIST_0 : REF_PIC_LIST_1;
        refIdx  = motionInfo[x].refIdx[motionInfo[x].interDir - 1];
        CHECK(refIdx < 0, "unidirectional inter prediction should have a valid refIdx");
        cMv.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
        posY.x = pu.Y().x + (x << MIN_CU_LOG2) + cMv.getHor();
        posY.y = pu.Y().y + (y << MIN_CU_LOG2) + cMv.getVer();
        clipColPos(posY.x, posY.y, pu);
#if RPR_ENABLE
        scalePositionInRef(pu, *pu.cs->pps, refList, refIdx, posY);
        posY.x                  = (posY.x & mask);
        posY.y                  = (posY.y & mask);
        const MotionInfo &miRef = pu.cu->slice->getRefPic(refList, refIdx)->unscaledPic->cs->getMotionInfo(posY);
#else
        posY.x = (posY.x & mask);
        posY.y = (posY.y & mask);
        const MotionInfo &miRef = pu.cu->slice->getRefPic(refList, refIdx)->cs->getMotionInfo(posY);
#endif
        motionInfo[x].isRefSCC    = miRef.isSCC;
        motionInfo[x].isRefRefSCC = miRef.isRefSCC;
      }
      else if (CU::isInter(*pu.cu) && motionInfo[x].isInter && motionInfo[x].interDir == 3)
      {
#if RPR_ENABLE
        pRefPic0 = pu.cu->slice->getRefPic(REF_PIC_LIST_0, motionInfo[x].refIdx[0])->unscaledPic;
#else
        pRefPic0 = pu.cu->slice->getRefPic(REF_PIC_LIST_0, motionInfo[x].refIdx[0]);
#endif
        cMv0 = motionInfo[x].mv[0];
        cMv0.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
        posY0.x = pu.Y().x + (x << MIN_CU_LOG2) + cMv0.getHor();
        posY0.y = pu.Y().y + (y << MIN_CU_LOG2) + cMv0.getVer();
        clipColPos(posY0.x, posY0.y, pu);
#if RPR_ENABLE
        scalePositionInRef(pu, *pu.cs->pps, REF_PIC_LIST_0, motionInfo[x].refIdx[0], posY0);
#endif
        posY0.x = (posY0.x & mask);
        posY0.y = (posY0.y & mask);
        const MotionInfo &mi0Ref     = pRefPic0->cs->getMotionInfo(posY0);

#if RPR_ENABLE
        pRefPic1 = pu.cu->slice->getRefPic(REF_PIC_LIST_1, motionInfo[x].refIdx[1])->unscaledPic;
#else
        pRefPic1 = pu.cu->slice->getRefPic(REF_PIC_LIST_1, motionInfo[x].refIdx[1]);
#endif
        cMv1 = motionInfo[x].mv[1];
        cMv1.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
        posY1.x = pu.Y().x + (x << MIN_CU_LOG2) + cMv1.getHor();
        posY1.y = pu.Y().y + (y << MIN_CU_LOG2) + cMv1.getVer();
        clipColPos(posY1.x, posY1.y, pu);
#if RPR_ENABLE
        scalePositionInRef(pu, *pu.cs->pps, REF_PIC_LIST_1, motionInfo[x].refIdx[1], posY1);
#endif
        posY1.x = (posY1.x & mask);
        posY1.y = (posY1.y & mask);
        const MotionInfo &mi1Ref     = pRefPic1->cs->getMotionInfo(posY1);

        motionInfo[x].isRefSCC    = mi0Ref.isSCC || mi1Ref.isSCC;
        motionInfo[x].isRefRefSCC = mi0Ref.isRefSCC || mi1Ref.isRefSCC;
      }
    }
    motionInfo += mb.stride;
  }
}
#endif

#if !JVET_Z0054_BLK_REF_PIC_REORDER
void PU::applyImv( PredictionUnit& pu, MergeCtx &mrgCtx, InterPrediction *interPred )
{
  if( !pu.mergeFlag )
  {
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
    Mv cMvpL0;
#endif
    if( pu.interDir != 2 /* PRED_L1 */ )
    {
#if !JVET_Z0084_IBC_TM
      pu.mvd[0].changeTransPrecAmvr2Internal(pu.cu->imv);
#endif
      unsigned mvpIdx = pu.mvpIdx[0];
      AMVPInfo amvpInfo;
      if (CU::isIBC(*pu.cu))
      {
#if JVET_Z0084_IBC_TM
        pu.mvd[0].changeIbcPrecAmvr2Internal(pu.cu->imv);
#endif
        PU::fillIBCMvpCand(pu, amvpInfo
#if JVET_Z0084_IBC_TM && IBC_TM_AMVP
                         , interPred
#endif
        );
      }
      else
      {
#if JVET_Z0084_IBC_TM
        pu.mvd[0].changeTransPrecAmvr2Internal(pu.cu->imv);
#endif
        PU::fillMvpCand(pu, REF_PIC_LIST_0, pu.refIdx[0], amvpInfo
#if TM_AMVP
                      , interPred
#endif
          );
      }
      pu.mvpNum[0] = amvpInfo.numCand;
      pu.mvpIdx[0] = mvpIdx;
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AD0140_MVD_PREDICTION
      if (!CU::isIBC(*pu.cu))
      {
        RefPicList eRefList(REF_PIC_LIST_0);
        if (pu.isMvdPredApplicable() && pu.mvd[eRefList].isMvdPredApplicable())
        {
          if (pu.cu->smvdMode)
          {
            cMvpL0 = amvpInfo.mvCand[pu.mvpIdx[eRefList]];
          }
          else
          {
            std::vector<Mv> cMvdDerivedVec;
            interPred->deriveMvdSign(amvpInfo.mvCand[mvpIdx], pu.mvd[eRefList], pu, eRefList, pu.refIdx[eRefList], cMvdDerivedVec);
            CHECK(pu.mvsdIdx[eRefList] >= cMvdDerivedVec.size(), "");
            int mvsdIdx = pu.mvsdIdx[eRefList];
            Mv cMvd = interPred->deriveMVDFromMVSDIdxTrans(mvsdIdx, cMvdDerivedVec);
            CHECK(cMvd == Mv(0, 0), " zero MVD!");
            pu.mvd[eRefList] = cMvd;
          }
        }
      }
#endif
      pu.mv    [0] = amvpInfo.mvCand[mvpIdx] + pu.mvd[0];
      pu.mv[0].mvCliptoStorageBitDepth();
#if JVET_AA0070_RRIBC
      if (CU::isIBC(*pu.cu) && pu.cu->rribcFlipType == 1)
      {
        pu.mv[0].setVer(0);
      }
      else if (CU::isIBC(*pu.cu) && pu.cu->rribcFlipType == 2)
      {
        pu.mv[0].setHor(0);
      }
#endif
#if JVET_Z0160_IBC_ZERO_PADDING
      pu.bv = pu.mv[0];
      pu.bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
#endif
    }

    if (pu.interDir != 1 /* PRED_L0 */)
    {
      if( !( pu.cu->cs->picHeader->getMvdL1ZeroFlag() && pu.interDir == 3 ) && pu.cu->imv )/* PRED_BI */
      {
        pu.mvd[1].changeTransPrecAmvr2Internal(pu.cu->imv);
      }
      unsigned mvpIdx = pu.mvpIdx[1];
      AMVPInfo amvpInfo;
      PU::fillMvpCand(pu, REF_PIC_LIST_1, pu.refIdx[1], amvpInfo
#if TM_AMVP
                    , interPred
#endif
      );
      pu.mvpNum[1] = amvpInfo.numCand;
      pu.mvpIdx[1] = mvpIdx;
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED || JVET_AD0140_MVD_PREDICTION
      RefPicList eRefList(REF_PIC_LIST_1);
      if (pu.isMvdPredApplicable() && pu.mvd[eRefList].isMvdPredApplicable())
      {
        if (pu.cu->smvdMode)
        {
          std::vector<Mv> cMvdDerivedVec;
          interPred->deriveMvdSignSMVD(cMvpL0, amvpInfo.mvCand[pu.mvpIdx[1]], pu.mvd[REF_PIC_LIST_0], pu, cMvdDerivedVec);
          CHECK(pu.mvsdIdx[REF_PIC_LIST_0] >= cMvdDerivedVec.size(), "");
          int mvsdIdx = pu.mvsdIdx[REF_PIC_LIST_0];
          Mv cMvd = interPred->deriveMVDFromMVSDIdxTrans(mvsdIdx, cMvdDerivedVec);
          CHECK(cMvd == Mv(0, 0), " zero MVD for SMVD!");
          pu.mvd[REF_PIC_LIST_0] = cMvd;
          pu.mv[REF_PIC_LIST_0] = cMvpL0 + pu.mvd[REF_PIC_LIST_0];
          pu.mv[REF_PIC_LIST_0].mvCliptoStorageBitDepth();
          pu.mvd[REF_PIC_LIST_1].set(-pu.mvd[REF_PIC_LIST_0].hor, -pu.mvd[REF_PIC_LIST_0].ver);
        }
        else
        {
          std::vector<Mv> cMvdDerivedVec;
          interPred->deriveMvdSign(amvpInfo.mvCand[mvpIdx], pu.mvd[eRefList], pu, eRefList, pu.refIdx[eRefList], cMvdDerivedVec);
          CHECK(pu.mvsdIdx[eRefList] >= cMvdDerivedVec.size(), "");
          int mvsdIdx = pu.mvsdIdx[eRefList];
          Mv cMvd = interPred->deriveMVDFromMVSDIdxTrans(mvsdIdx, cMvdDerivedVec );
          CHECK(cMvd == Mv(0, 0), " zero MVD!");
          pu.mvd[eRefList] = cMvd;
        }
      }
#endif
      pu.mv    [1] = amvpInfo.mvCand[mvpIdx] + pu.mvd[1];
      pu.mv[1].mvCliptoStorageBitDepth();
    }
  }
  else
  {
    // this function is never called for merge
    THROW("unexpected");
    PU::getInterMergeCandidates(pu, mrgCtx, 0);

    mrgCtx.setMergeInfo( pu, pu.mergeIdx );
  }

  PU::spanMotionInfo( pu, mrgCtx );
}
#endif

#if JVET_AD0184_REMOVAL_OF_DIVISION_OPERATIONS
int PU::getMeanValue(int sum, int div)
{
  int sign = 1;
  if (sum < 0 )
  {
    sum  = -sum;
    sign = -1;
  }
  int divTable[16] = { 0, 7, 6, 5, 5, 4, 4, 3, 3, 2, 2, 1, 1, 1, 1, 0 };
  int x            = floorLog2(div);
  int normNum1     = (div << 4 >> x) & 15;
  int v            = divTable[normNum1] | 8;
  x += (normNum1 != 0);
  int shift  = 13 - x;
  int retVal = 0;
  if (shift < 0)
  {
    shift   = -shift;
    int add = (1 << (shift - 1));
    retVal  = (sum * v + add) >> shift;
  }
  else
  {
    retVal = (sum * v) << shift;
  }
  return sign * (retVal >> 16);
}
#endif

bool PU::isBiPredFromDifferentDirEqDistPoc(const PredictionUnit& pu)
{
#if JVET_AE0169_BIPREDICTIVE_IBC
  if (CU::isIBC(*pu.cu))
  {
    return false;
  }
#endif
  if (pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0)
  {
    if (pu.cu->slice->getRefPic(REF_PIC_LIST_0, pu.refIdx[0])->longTerm
      || pu.cu->slice->getRefPic(REF_PIC_LIST_1, pu.refIdx[1])->longTerm)
    {
      return false;
    }
#if JVET_Y0128_NON_CTC
    if ( PU::isBiRefScaled( *pu.cs, pu.refIdx[0], pu.refIdx[1] ) )
    {
      return false;
    }
#endif
    const int poc0 = pu.cu->slice->getRefPOC(REF_PIC_LIST_0, pu.refIdx[0]);
    const int poc1 = pu.cu->slice->getRefPOC(REF_PIC_LIST_1, pu.refIdx[1]);
    const int poc = pu.cu->slice->getPOC();
    if ((poc - poc0)*(poc - poc1) < 0)
    {
      if (abs(poc - poc0) == abs(poc - poc1))
      {
        return true;
      }
    }
  }
  return false;
}
#if JVET_AG0067_DMVR_EXTENSIONS
bool PU::isBiPredFromDifferentDirGenDistPoc(const PredictionUnit& pu)
{
#if JVET_AE0169_BIPREDICTIVE_IBC
  if (CU::isIBC(*pu.cu))
  {
    return false;
  }
#endif
  if (pu.refIdx[0] >= 0 && pu.refIdx[1] >= 0)
  {
    if (pu.cu->slice->getRefPic(REF_PIC_LIST_0, pu.refIdx[0])->longTerm
        || pu.cu->slice->getRefPic(REF_PIC_LIST_1, pu.refIdx[1])->longTerm)
    {
      return false;
    }
#if JVET_Y0128_NON_CTC
    if ( PU::isBiRefScaled( *pu.cs, pu.refIdx[0], pu.refIdx[1] ) )
    {
      return false;
    }
#endif
    const int poc0 = pu.cu->slice->getRefPOC(REF_PIC_LIST_0, pu.refIdx[0]);
    const int poc1 = pu.cu->slice->getRefPOC(REF_PIC_LIST_1, pu.refIdx[1]);
    const int poc = pu.cu->slice->getPOC();
    if ((poc - poc0)*(poc - poc1) < 0)
    {
      {
        return true;
      }
    }
  }
  return false;
}
#endif

void PU::restrictBiPredMergeCandsOne(PredictionUnit &pu)
{
  if (PU::isBipredRestriction(pu))
  {
    if (pu.interDir == 3)
    {
      pu.interDir = 1;
      pu.refIdx[1] = -1;
      pu.mv[1] = Mv(0, 0);
      pu.cu->bcwIdx = BCW_DEFAULT;
#if MULTI_HYP_PRED
      pu.addHypData.clear();
      pu.numMergedAddHyps = 0;
#endif
    }
  }
}

#if JVET_AE0046_BI_GPM
void PU::setGpmDirMode(PredictionUnit& pu)
{
  pu.gpmDirMode = 1;

  bool triggerUni = ((pu.lwidth() * pu.lheight()) < 256) ? true : false;

  if (triggerUni)
  {
    pu.gpmDirMode = 0;
  }
}
#endif

#if JVET_W0097_GPM_MMVD_TM
#if TM_MRG
#if JVET_AE0046_BI_GPM
void PU::getGeoMergeCandidates(PredictionUnit& pu, MergeCtx& geoMrgCtx, MergeCtx* mergeCtx, bool is4GPM)
#else
void PU::getGeoMergeCandidates(PredictionUnit &pu, MergeCtx& geoMrgCtx, MergeCtx* mergeCtx)
#endif
#else
#if JVET_AE0046_BI_GPM
void PU::getGeoMergeCandidates(const PredictionUnit& pu, MergeCtx& geoMrgCtx, MergeCtx* mergeCtx, bool is4GPM)
#else
void PU::getGeoMergeCandidates(const PredictionUnit &pu, MergeCtx& geoMrgCtx, MergeCtx* mergeCtx)
#endif
#endif
#else
void PU::getGeoMergeCandidates( const PredictionUnit &pu, MergeCtx& geoMrgCtx )
#endif
{
  MergeCtx tmpMergeCtx;

  const uint32_t maxNumMergeCand = pu.cs->sps->getMaxNumMergeCand();
#if JVET_Z0127_SPS_MHP_MAX_MRG_CAND
  const uint32_t maxNumGeoMergeCand = std::max(pu.cs->sps->getMaxNumMHPCand(), pu.cs->sps->getMaxNumGeoCand());
#endif
  geoMrgCtx.numValidMergeCand = 0;

#if JVET_Z0127_SPS_MHP_MAX_MRG_CAND
  for (int32_t i = 0; i < maxNumGeoMergeCand; i++)
#else
  for (int32_t i = 0; i < GEO_MAX_NUM_UNI_CANDS; i++)
#endif
  {
    geoMrgCtx.bcwIdx[i] = BCW_DEFAULT;
    geoMrgCtx.interDirNeighbours[i] = 0;
    geoMrgCtx.mvFieldNeighbours[(i << 1)].refIdx = NOT_VALID;
    geoMrgCtx.mvFieldNeighbours[(i << 1) + 1].refIdx = NOT_VALID;
    geoMrgCtx.mvFieldNeighbours[(i << 1)].mv = Mv();
    geoMrgCtx.mvFieldNeighbours[(i << 1) + 1].mv = Mv();
    geoMrgCtx.useAltHpelIf[i] = false;
#if JVET_AG0276_NLIC
    geoMrgCtx.altLMFlag[i] = false;
    geoMrgCtx.altLMParaNeighbours[i].resetAltLinearModel();
#endif
#if INTER_LIC
    geoMrgCtx.licFlags[i] = false;
#endif
  }
#if JVET_W0097_GPM_MMVD_TM
#if JVET_AE0046_BI_GPM
  if ( pu.gpmDirMode || (mergeCtx == NULL) )
#else
  if (mergeCtx == NULL)
#endif
  {
#if TM_MRG
    const bool tmMergeFlag = pu.tmMergeFlag;
    pu.tmMergeFlag = false;
#endif
#endif
#if JVET_AE0046_BI_GPM
#if JVET_Y0134_TMVP_NAMVP_CAND_REORDERING && JVET_W0090_ARMC_TM
    PU::getInterMergeCandidates(pu, tmpMergeCtx, 0, -1, nullptr, nullptr, is4GPM);
#else
    PU::getInterMergeCandidates(pu, tmpMergeCtx, 0, -1, is4GPM);
#endif
#else
  PU::getInterMergeCandidates(pu, tmpMergeCtx, 0);
#endif
#if JVET_W0097_GPM_MMVD_TM
#if TM_MRG
  pu.tmMergeFlag = tmMergeFlag;
#endif
  }
  else
  {
    memcpy(tmpMergeCtx.interDirNeighbours, mergeCtx->interDirNeighbours, maxNumMergeCand * sizeof(unsigned char));
    memcpy(tmpMergeCtx.mvFieldNeighbours, mergeCtx->mvFieldNeighbours, (maxNumMergeCand << 1) * sizeof(MvField));
  }
#endif

#if JVET_AE0046_BI_GPM
  if ( pu.gpmDirMode )
  {
    CHECK(!is4GPM, "is4GPM should be on when gpmDirMode is not 0");

    geoMrgCtx = tmpMergeCtx;
    geoMrgCtx.numValidMergeCand = maxNumMergeCand;

    for (int32_t i = 0; i < maxNumMergeCand; i++)
    {
      geoMrgCtx.useAltHpelIf[i] = false;
      geoMrgCtx.bcwIdx[i] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
      geoMrgCtx.altLMFlag[i] = false;
      geoMrgCtx.altLMParaNeighbours[i].resetAltLinearModel();
#endif
#if INTER_LIC
      geoMrgCtx.licFlags[i] = false;
#endif
#if MULTI_HYP_PRED
      geoMrgCtx.addHypNeighbours[i].clear();
#endif
    }
    return;
  }
#endif

  for (int32_t i = 0; i < maxNumMergeCand; i++)
  {
    int parity = i & 1;
    if( tmpMergeCtx.interDirNeighbours[i] & (0x01 + parity) )
    {
      geoMrgCtx.interDirNeighbours[geoMrgCtx.numValidMergeCand] = 1 + parity;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].mv = Mv(0, 0);
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].mv = tmpMergeCtx.mvFieldNeighbours[(i << 1) + parity].mv;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].refIdx = -1;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].refIdx = tmpMergeCtx.mvFieldNeighbours[(i << 1) + parity].refIdx;
#if JVET_W0097_GPM_MMVD_TM
      if (geoMrgCtx.xCheckSimilarMotion(geoMrgCtx.numValidMergeCand, PU::getBDMVRMvdThreshold(pu)))
      {
        continue;
      }
#endif
      geoMrgCtx.numValidMergeCand++;
#if JVET_Z0127_SPS_MHP_MAX_MRG_CAND
      if (geoMrgCtx.numValidMergeCand == maxNumGeoMergeCand)
#else
      if (geoMrgCtx.numValidMergeCand == GEO_MAX_NUM_UNI_CANDS)
#endif
      {
        return;
      }
      continue;
    }

    if (tmpMergeCtx.interDirNeighbours[i] & (0x02 - parity))
    {
      geoMrgCtx.interDirNeighbours[geoMrgCtx.numValidMergeCand] = 2 - parity;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].mv = tmpMergeCtx.mvFieldNeighbours[(i << 1) + !parity].mv;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].mv = Mv(0, 0);
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].refIdx = tmpMergeCtx.mvFieldNeighbours[(i << 1) + !parity].refIdx;
      geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].refIdx = -1;
#if JVET_W0097_GPM_MMVD_TM
      if (geoMrgCtx.xCheckSimilarMotion(geoMrgCtx.numValidMergeCand, PU::getBDMVRMvdThreshold(pu)))
      {
        continue;
      }
#endif
      geoMrgCtx.numValidMergeCand++;
#if JVET_Z0127_SPS_MHP_MAX_MRG_CAND
      if (geoMrgCtx.numValidMergeCand == maxNumGeoMergeCand)
#else
      if (geoMrgCtx.numValidMergeCand == GEO_MAX_NUM_UNI_CANDS)
#endif
      {
        return;
      }
    }
  }
#if JVET_W0097_GPM_MMVD_TM
  // add more parity based geo candidates, in an opposite parity rule
#if JVET_Z0127_SPS_MHP_MAX_MRG_CAND
  if (geoMrgCtx.numValidMergeCand < maxNumGeoMergeCand)
#else
  if (geoMrgCtx.numValidMergeCand < pu.cs->sps->getMaxNumGeoCand())
#endif
  {
    for (int32_t i = 0; i < maxNumMergeCand; i++)
    {
      int parity = i & 1;
      if (tmpMergeCtx.interDirNeighbours[i] == 3)
      {
        geoMrgCtx.interDirNeighbours[geoMrgCtx.numValidMergeCand] = 2 - parity;
        geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].mv = tmpMergeCtx.mvFieldNeighbours[(i << 1) + !parity].mv;
        geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].mv = Mv(0, 0);
        geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].refIdx = tmpMergeCtx.mvFieldNeighbours[(i << 1) + !parity].refIdx;
        geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].refIdx = -1;
        if (geoMrgCtx.xCheckSimilarMotion(geoMrgCtx.numValidMergeCand, PU::getBDMVRMvdThreshold(pu)))
        {
          continue;
        }
        geoMrgCtx.numValidMergeCand++;
#if JVET_Z0127_SPS_MHP_MAX_MRG_CAND
        if (geoMrgCtx.numValidMergeCand == maxNumGeoMergeCand)
#else
        if (geoMrgCtx.numValidMergeCand == pu.cs->sps->getMaxNumGeoCand())
#endif
        {
          return;
        }
      }
    }
  }

  // add at most two average based geo candidates
#if JVET_Z0127_SPS_MHP_MAX_MRG_CAND
  if (geoMrgCtx.numValidMergeCand < maxNumGeoMergeCand)
#else
  if (geoMrgCtx.numValidMergeCand < pu.cs->sps->getMaxNumGeoCand())
#endif
  {
    // add one L0 cand by averaging the first two available L0 candidates
    int cnt = 0;
    int firstAvailRefIdx = -1;
    Mv  avgMv;
    avgMv.setZero();
    for (int i = 0; i < geoMrgCtx.numValidMergeCand; i++)
    {
      if (cnt == 2)
      {
        break;
      }
      if (geoMrgCtx.interDirNeighbours[i] == 1)
      {
        avgMv += geoMrgCtx.mvFieldNeighbours[i * 2].mv;
        if (firstAvailRefIdx == -1)
        {
          firstAvailRefIdx = geoMrgCtx.mvFieldNeighbours[i * 2].refIdx;
        }
        cnt++;
      }
    }
    if (cnt == 2)
    {
      roundAffineMv(avgMv.hor, avgMv.ver, 1);
      geoMrgCtx.interDirNeighbours[geoMrgCtx.numValidMergeCand] = 1;
      geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand * 2].setMvField(avgMv, firstAvailRefIdx);
      geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand * 2 + 1].setMvField(Mv(0, 0), NOT_VALID);

      if (!geoMrgCtx.xCheckSimilarMotion(geoMrgCtx.numValidMergeCand, PU::getBDMVRMvdThreshold(pu)))
      {
        geoMrgCtx.numValidMergeCand++;
      }
#if JVET_Z0127_SPS_MHP_MAX_MRG_CAND
      if (geoMrgCtx.numValidMergeCand == maxNumGeoMergeCand)
#else
      if (geoMrgCtx.numValidMergeCand == pu.cs->sps->getMaxNumGeoCand())
#endif
      {
        return;
      }
    }

    // add one L1 cand by averaging the first two available L1 candidates
    cnt = 0;
    firstAvailRefIdx = -1;
    avgMv.setZero();
    for (int i = 0; i < geoMrgCtx.numValidMergeCand; i++)
    {
      if (cnt == 2)
      {
        break;
      }
      if (geoMrgCtx.interDirNeighbours[i] == 2)
      {
        avgMv += geoMrgCtx.mvFieldNeighbours[i * 2 + 1].mv;
        if (firstAvailRefIdx == -1)
        {
          firstAvailRefIdx = geoMrgCtx.mvFieldNeighbours[i * 2 + 1].refIdx;
        }
        cnt++;
      }
    }
    if (cnt == 2)
    {
      roundAffineMv(avgMv.hor, avgMv.ver, 1);
      geoMrgCtx.interDirNeighbours[geoMrgCtx.numValidMergeCand] = 2;
      geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand * 2 + 1].setMvField(avgMv, firstAvailRefIdx);
      geoMrgCtx.mvFieldNeighbours[geoMrgCtx.numValidMergeCand * 2].setMvField(Mv(0, 0), NOT_VALID);
      if (!geoMrgCtx.xCheckSimilarMotion(geoMrgCtx.numValidMergeCand, PU::getBDMVRMvdThreshold(pu)))
      {
        geoMrgCtx.numValidMergeCand++;
      }
#if JVET_Z0127_SPS_MHP_MAX_MRG_CAND
      if (geoMrgCtx.numValidMergeCand == maxNumGeoMergeCand)
#else
      if (geoMrgCtx.numValidMergeCand == pu.cs->sps->getMaxNumGeoCand())
#endif
      {
        return;
      }
    }
  }
#if JVET_Z0127_SPS_MHP_MAX_MRG_CAND
  if (geoMrgCtx.numValidMergeCand < maxNumGeoMergeCand)
#else
  if (geoMrgCtx.numValidMergeCand < pu.cs->sps->getMaxNumGeoCand())
#endif
  {
    const Slice &slice = *pu.cs->slice;
#if JVET_Y0065_GPM_INTRA
    int         iNumRefIdx = pu.cs->slice->isInterP() ? slice.getNumRefIdx(REF_PIC_LIST_0) : std::min(slice.getNumRefIdx(REF_PIC_LIST_0), slice.getNumRefIdx(REF_PIC_LIST_1));
#else
    int         iNumRefIdx = std::min(slice.getNumRefIdx(REF_PIC_LIST_0), slice.getNumRefIdx(REF_PIC_LIST_1));
#endif

    int r = 0;
    int refcnt = 0;

#if JVET_Z0127_SPS_MHP_MAX_MRG_CAND
    for (int32_t i = geoMrgCtx.numValidMergeCand; i < maxNumGeoMergeCand; i++)
#else
    for (int32_t i = geoMrgCtx.numValidMergeCand; i < pu.cs->sps->getMaxNumGeoCand(); i++)
#endif
    {
#if JVET_Y0065_GPM_INTRA
      int parity = pu.cs->slice->isInterP() ? 0 : (i & 1);
#else
      int parity = i & 1;
#endif
      if (0x01 + parity)
      {
        geoMrgCtx.interDirNeighbours[geoMrgCtx.numValidMergeCand] = 1 + parity;
        geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].mv = Mv(0, 0);
        geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].mv = Mv(0, 0);
        geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + !parity].refIdx = -1;
        geoMrgCtx.mvFieldNeighbours[(geoMrgCtx.numValidMergeCand << 1) + parity].refIdx = r;

        if (refcnt == iNumRefIdx - 1)
        {
          r = 0;
        }
        else
        {
          ++r;
          ++refcnt;
        }

        geoMrgCtx.numValidMergeCand++;
#if JVET_Z0127_SPS_MHP_MAX_MRG_CAND
        if (geoMrgCtx.numValidMergeCand == maxNumGeoMergeCand)
#else
        if (geoMrgCtx.numValidMergeCand == pu.cs->sps->getMaxNumGeoCand())
#endif
        {
          return;
        }
      }
    }
  }
#endif
}

void PU::spanGeoMotionInfo( PredictionUnit &pu, MergeCtx &geoMrgCtx, const uint8_t splitDir, const uint8_t candIdx0, const uint8_t candIdx1, const uint8_t *intraMPM
#if JVET_AG0164_AFFINE_GPM
  , AffineMergeCtx& geoAffMrgCtx
#endif
)
{
  pu.geoSplitDir  = splitDir;
  pu.geoMergeIdx0 = candIdx0;
  pu.geoMergeIdx1 = candIdx1;
  MotionBuf mb = pu.getMotionBuf();
#if JVET_W0123_TIMD_FUSION
  IpmBuf ib = pu.getIpmBuf();
#endif

#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  CHECK(pu.cu->geoBlendFlag && !pu.cs->pcv->isEncoder,"PU::spanGeoMotionInfo( geoBlendFlag = 1 ) !!!");
#endif
#if JVET_AG0164_AFFINE_GPM
  Mv affMvBuf0[2][(MAX_CU_SIZE >> 2) * (MAX_CU_SIZE >> 2)];
  Mv affMvBuf1[2][(MAX_CU_SIZE >> 2) * (MAX_CU_SIZE >> 2)];
  int affMvBufStride = MAX_CU_SIZE >> 2;
  if (pu.affineGPM[0])
  {
    pu.gpmPartAffType[0] = geoAffMrgCtx.affineType[candIdx0];
    CHECK(pu.gpmPartAffType[0] == AFFINE_MODEL_NUM, "Invalid affine type");
    pu.gpmPartRefIdx[0][0] = pu.gpmPartRefIdx[0][1] = -1;

    pu.cu->affineType = geoAffMrgCtx.affineType[candIdx0];
    MvField* cpmvMvField0 = geoAffMrgCtx.mvFieldNeighbours[candIdx0 << 1];
    if (cpmvMvField0[0].refIdx >= 0)
    {
      PU::setAllAffineMv(pu, cpmvMvField0[0].mv, cpmvMvField0[1].mv, cpmvMvField0[2].mv, REF_PIC_LIST_0);

      pu.gpmPartRefIdx[0][0]    = cpmvMvField0[0].refIdx;
      pu.gpmPartmvAffi[0][0][0] = cpmvMvField0[0].mv;
      pu.gpmPartmvAffi[0][0][1] = cpmvMvField0[1].mv;
      pu.gpmPartmvAffi[0][0][2] = cpmvMvField0[2].mv;
    }
    MvField* cpmvMvField1 = geoAffMrgCtx.mvFieldNeighbours[(candIdx0 << 1)+1];
    if (cpmvMvField1[0].refIdx >= 0)
    {
      PU::setAllAffineMv(pu, cpmvMvField1[0].mv, cpmvMvField1[1].mv, cpmvMvField1[2].mv, REF_PIC_LIST_1);

      pu.gpmPartRefIdx[0][1]    = cpmvMvField1[0].refIdx;
      pu.gpmPartmvAffi[0][1][0] = cpmvMvField1[0].mv;
      pu.gpmPartmvAffi[0][1][1] = cpmvMvField1[1].mv;
      pu.gpmPartmvAffi[0][1][2] = cpmvMvField1[2].mv;
    }
    MotionInfo* pMiBuf = mb.buf;
    Mv* pMvBuf0 = affMvBuf0[0];
    Mv* pMvBuf1 = affMvBuf0[1];
    for (int y = 0; y < mb.height; y++)
    {
      for (int x = 0; x < mb.width; x++)
      {
        pMvBuf0[x] = cpmvMvField0[0].refIdx >= 0 ? pMiBuf[x].mv[0] : Mv();
        pMvBuf1[x] = cpmvMvField1[0].refIdx >= 0 ? pMiBuf[x].mv[1] : Mv();
      }
      pMiBuf += mb.stride;
      pMvBuf0 += affMvBufStride;
      pMvBuf1 += affMvBufStride;
    }
  }
  if (pu.affineGPM[1])
  {
    pu.gpmPartAffType[1] = geoAffMrgCtx.affineType[candIdx1];
    CHECK(pu.gpmPartAffType[1] == AFFINE_MODEL_NUM, "Invalid affine type");
    pu.gpmPartRefIdx[1][0] = pu.gpmPartRefIdx[1][1] = -1;

    pu.cu->affineType = geoAffMrgCtx.affineType[candIdx1];
    MvField* cpmvMvField0 = geoAffMrgCtx.mvFieldNeighbours[(candIdx1 << 1)];
    if (cpmvMvField0[0].refIdx >= 0)
    {
      PU::setAllAffineMv(pu, cpmvMvField0[0].mv, cpmvMvField0[1].mv, cpmvMvField0[2].mv, REF_PIC_LIST_0);

      pu.gpmPartRefIdx[1][0]    = cpmvMvField0[0].refIdx;
      pu.gpmPartmvAffi[1][0][0] = cpmvMvField0[0].mv;
      pu.gpmPartmvAffi[1][0][1] = cpmvMvField0[1].mv;
      pu.gpmPartmvAffi[1][0][2] = cpmvMvField0[2].mv;

    }
    MvField* cpmvMvField1 = geoAffMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1];
    if (cpmvMvField1[0].refIdx >= 0)
    {
      PU::setAllAffineMv(pu, cpmvMvField1[0].mv, cpmvMvField1[1].mv, cpmvMvField1[2].mv, REF_PIC_LIST_1);

      pu.gpmPartRefIdx[1][1]    = cpmvMvField1[0].refIdx;
      pu.gpmPartmvAffi[1][1][0] = cpmvMvField1[0].mv;
      pu.gpmPartmvAffi[1][1][1] = cpmvMvField1[1].mv;
      pu.gpmPartmvAffi[1][1][2] = cpmvMvField1[2].mv;
    }
    MotionInfo* pMiBuf = mb.buf;
    Mv* pMvBuf0 = affMvBuf1[0];
    Mv* pMvBuf1 = affMvBuf1[1];
    for (int y = 0; y < mb.height; y++)
    {
      for (int x = 0; x < mb.width; x++)
      {
        pMvBuf0[x] = cpmvMvField0[0].refIdx >= 0 ? pMiBuf[x].mv[0] : Mv();
        pMvBuf1[x] = cpmvMvField1[0].refIdx >= 0 ? pMiBuf[x].mv[1] : Mv();
      }
      pMiBuf += mb.stride;
      pMvBuf0 += affMvBufStride;
      pMvBuf1 += affMvBufStride;
    }
  }
#endif

#if JVET_Y0065_GPM_INTRA
#if JVET_AG0164_AFFINE_GPM
  bool isIntra0 = candIdx0 >= GEO_MAX_ALL_INTER_UNI_CANDS;
  bool isIntra1 = candIdx1 >= GEO_MAX_ALL_INTER_UNI_CANDS;
#else
  bool isIntra0 = candIdx0 >= GEO_MAX_NUM_UNI_CANDS;
  bool isIntra1 = candIdx1 >= GEO_MAX_NUM_UNI_CANDS;
#endif
  uint32_t sliceIdx = pu.cs->slice->getIndependentSliceIdx();
#else
  MotionInfo biMv;
  biMv.isInter  = true;
  biMv.sliceIdx = pu.cs->slice->getIndependentSliceIdx();

  if( geoMrgCtx.interDirNeighbours[candIdx0] == 1 && geoMrgCtx.interDirNeighbours[candIdx1] == 2 )
  {
    biMv.interDir  = 3;
    biMv.mv[0]     = geoMrgCtx.mvFieldNeighbours[ candIdx0 << 1     ].mv;
    biMv.mv[1]     = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mv;
    biMv.refIdx[0] = geoMrgCtx.mvFieldNeighbours[ candIdx0 << 1     ].refIdx;
    biMv.refIdx[1] = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].refIdx;
  }
  else if( geoMrgCtx.interDirNeighbours[candIdx0] == 2 && geoMrgCtx.interDirNeighbours[candIdx1] == 1 )
  {
    biMv.interDir  = 3;
    biMv.mv[0]     = geoMrgCtx.mvFieldNeighbours[ candIdx1 << 1     ].mv;
    biMv.mv[1]     = geoMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].mv;
    biMv.refIdx[0] = geoMrgCtx.mvFieldNeighbours[ candIdx1 << 1     ].refIdx;
    biMv.refIdx[1] = geoMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].refIdx;
  }
  else if( geoMrgCtx.interDirNeighbours[candIdx0] == 1 && geoMrgCtx.interDirNeighbours[candIdx1] == 1 )
  {
    biMv.interDir = 1;
    biMv.mv[0] = geoMrgCtx.mvFieldNeighbours[candIdx1 << 1].mv;
    biMv.mv[1] = Mv(0, 0);
    biMv.refIdx[0] = geoMrgCtx.mvFieldNeighbours[candIdx1 << 1].refIdx;
    biMv.refIdx[1] = -1;
  }
  else if( geoMrgCtx.interDirNeighbours[candIdx0] == 2 && geoMrgCtx.interDirNeighbours[candIdx1] == 2 )
  {
    biMv.interDir = 2;
    biMv.mv[0] = Mv(0, 0);
    biMv.mv[1] = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mv;
    biMv.refIdx[0] = -1;
    biMv.refIdx[1] = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].refIdx;
  }
#endif

  int16_t angle = g_geoParams[splitDir][0];
  int tpmMask = 0;
  int lookUpY = 0, motionIdx = 0;
  bool isFlip = angle >= 13 && angle <= 27;
  int distanceIdx = g_geoParams[splitDir][1];
  int distanceX = angle;
  int distanceY = (distanceX + (GEO_NUM_ANGLES >> 2)) % GEO_NUM_ANGLES;
  int offsetX = (-(int)pu.lwidth()) >> 1;
  int offsetY = (-(int)pu.lheight()) >> 1;

  if (distanceIdx > 0)
  {
    if( angle % 16 == 8 || ( angle % 16 != 0 && pu.lheight() >= pu.lwidth() ) )
    {
      offsetY += angle < 16 ? ( ( distanceIdx * pu.lheight() ) >> 3 ) : -( int ) ( ( distanceIdx * pu.lheight() ) >> 3 );
    }
    else
    {
      offsetX += angle < 16 ? ( ( distanceIdx * pu.lwidth() ) >> 3 ) : -( int ) ( ( distanceIdx * pu.lwidth() ) >> 3 );
    }
  }

  MotionInfo *motionInfo = mb.buf;

#if JVET_AG0164_AFFINE_GPM
  Mv* pMvBufP0L0 = affMvBuf0[0];
  Mv* pMvBufP0L1 = affMvBuf0[1];
  Mv* pMvBufP1L0 = affMvBuf1[0];
  Mv* pMvBufP1L1 = affMvBuf1[1];
#endif

  for (int y = 0; y < mb.height; y++)
  {
    lookUpY = (((4 * y + offsetY) << 1) + 5) * g_dis[distanceY];
    for (int x = 0; x < mb.width; x++)
    {
      motionIdx = (((4 * x + offsetX) << 1) + 5) * g_dis[distanceX] + lookUpY;
      motionInfo[x].bcwIdx = pu.cu->bcwIdx;
#if JVET_Y0065_GPM_INTRA
      tpmMask = motionIdx <= 0 ? (1 - isFlip) : isFlip;
      if (tpmMask == 0 && isIntra0)
      {
        motionInfo[x].isInter = false;
        motionInfo[x].interDir = MAX_UCHAR;
        motionInfo[x].refIdx[0] = -1;
        motionInfo[x].refIdx[1] = -1;
        motionInfo[x].mv[0] = Mv();
        motionInfo[x].mv[1] = Mv();
        motionInfo[x].sliceIdx = sliceIdx;
      }
#else
      tpmMask = abs(motionIdx) < 32 ? 2 : (motionIdx <= 0 ? (1 - isFlip) : isFlip);
      if (tpmMask == 2)
      {
        motionInfo[x].isInter = true;
        motionInfo[x].interDir = biMv.interDir;
        motionInfo[x].refIdx[0] = biMv.refIdx[0];
        motionInfo[x].refIdx[1] = biMv.refIdx[1];
        motionInfo[x].mv[0] = biMv.mv[0];
        motionInfo[x].mv[1] = biMv.mv[1];
        motionInfo[x].sliceIdx = biMv.sliceIdx;
      }
#endif
      else if (tpmMask == 0)
      {
        motionInfo[x].isInter = true;
#if JVET_AG0164_AFFINE_GPM
        if (pu.affineGPM[0])
        {
          motionInfo[x].interDir = geoAffMrgCtx.interDirNeighbours[candIdx0];
          motionInfo[x].refIdx[0] = geoAffMrgCtx.mvFieldNeighbours[candIdx0 << 1][0].refIdx;
          motionInfo[x].refIdx[1] = geoAffMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1][0].refIdx;
          motionInfo[x].mv[0] = pMvBufP0L0[x];
          motionInfo[x].mv[1] = pMvBufP0L1[x];
        }
        else
        {
#endif
        motionInfo[x].interDir = geoMrgCtx.interDirNeighbours[candIdx0];
        motionInfo[x].refIdx[0] = geoMrgCtx.mvFieldNeighbours[candIdx0 << 1].refIdx;
        motionInfo[x].refIdx[1] = geoMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].refIdx;
        motionInfo[x].mv[0] = geoMrgCtx.mvFieldNeighbours[candIdx0 << 1].mv;
        motionInfo[x].mv[1] = geoMrgCtx.mvFieldNeighbours[(candIdx0 << 1) + 1].mv;
#if JVET_AG0164_AFFINE_GPM
        }
#endif
#if JVET_Y0065_GPM_INTRA
        motionInfo[x].sliceIdx = sliceIdx;
#else
        motionInfo[x].sliceIdx = biMv.sliceIdx;
#endif
      }
#if JVET_Y0065_GPM_INTRA
      else if (tpmMask == 1 && isIntra1)
      {
        motionInfo[x].isInter = false;
        motionInfo[x].interDir = MAX_UCHAR;
        motionInfo[x].refIdx[0] = -1;
        motionInfo[x].refIdx[1] = -1;
        motionInfo[x].mv[0] = Mv();
        motionInfo[x].mv[1] = Mv();
        motionInfo[x].sliceIdx = sliceIdx;
      }
#endif
      else
      {
        motionInfo[x].isInter = true;
#if JVET_AG0164_AFFINE_GPM
        if (pu.affineGPM[1])
        {
          motionInfo[x].interDir = geoAffMrgCtx.interDirNeighbours[candIdx1];
          motionInfo[x].refIdx[0] = geoAffMrgCtx.mvFieldNeighbours[candIdx1 << 1][0].refIdx;
          motionInfo[x].refIdx[1] = geoAffMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1][0].refIdx;
          motionInfo[x].mv[0] = pMvBufP1L0[x];
          motionInfo[x].mv[1] = pMvBufP1L1[x];
        }
        else
        {
#endif
        motionInfo[x].interDir = geoMrgCtx.interDirNeighbours[candIdx1];
        motionInfo[x].refIdx[0] = geoMrgCtx.mvFieldNeighbours[candIdx1 << 1].refIdx;
        motionInfo[x].refIdx[1] = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].refIdx;
        motionInfo[x].mv[0] = geoMrgCtx.mvFieldNeighbours[candIdx1 << 1].mv;
        motionInfo[x].mv[1] = geoMrgCtx.mvFieldNeighbours[(candIdx1 << 1) + 1].mv;
#if JVET_AG0164_AFFINE_GPM
        }
#endif
#if JVET_Y0065_GPM_INTRA
        motionInfo[x].sliceIdx = sliceIdx;
#else
        motionInfo[x].sliceIdx = biMv.sliceIdx;
#endif
      }
#if JVET_AG0164_AFFINE_GPM
      CHECK(tpmMask != 0 && tpmMask != 1, "Invalid TPM mask");
      motionInfo[x].gpmPartIdx = tpmMask;
#endif
    }
    motionInfo += mb.stride;

#if JVET_AG0164_AFFINE_GPM
    pMvBufP0L0 += affMvBufStride;
    pMvBufP0L1 += affMvBufStride;
    pMvBufP1L0 += affMvBufStride;
    pMvBufP1L1 += affMvBufStride;
#endif
  }
#if JVET_W0123_TIMD_FUSION
#if JVET_Y0065_GPM_INTRA
  if (pu.gpmIntraFlag)
  {
    const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
    const unsigned mask = ~(scale - 1);
    uint8_t* ii = ib.buf;
    int geoIpm[3];
#if JVET_AG0164_AFFINE_GPM
    geoIpm[0] = isIntra0 ? intraMPM[candIdx0 - GEO_MAX_ALL_INTER_UNI_CANDS] : -1;
    geoIpm[1] = isIntra1 ? intraMPM[candIdx1 - GEO_MAX_ALL_INTER_UNI_CANDS + GEO_MAX_NUM_INTRA_CANDS] : -1;
#else
    geoIpm[0] = isIntra0 ? intraMPM[candIdx0 - GEO_MAX_NUM_UNI_CANDS] : -1;
    geoIpm[1] = isIntra1 ? intraMPM[candIdx1 - GEO_MAX_NUM_UNI_CANDS + GEO_MAX_NUM_INTRA_CANDS] : -1;
#endif
    geoIpm[2] = (isIntra0 && isIntra1) ? ((candIdx1 < candIdx0) ? geoIpm[1] : geoIpm[0]) : isIntra0 ? geoIpm[0] : geoIpm[1];

    for (int y = 0; y < mb.height; y++)
    {
      lookUpY = (((4 * y + offsetY) << 1) + 5) * g_dis[distanceY];
      for (int x = 0; x < mb.width; x++)
      {
        motionIdx = (((4 * x + offsetX) << 1) + 5) * g_dis[distanceX] + lookUpY;
        tpmMask = motionIdx <= 0 ? (1 - isFlip) : isFlip;
        uint8_t ipm;
        if (geoIpm[tpmMask] >= 0)
        {
          ipm = (uint8_t)geoIpm[tpmMask];
        }
        else
        {
          MotionInfo tempMi = mb.at(x, y);
          if (tempMi.interDir != 3)
          {
            CHECK( tempMi.interDir != 1 && tempMi.interDir != 2, "Uncorrect interDir" );
            int list = tempMi.interDir-1;
            Mv cMv = tempMi.mv[list];
            int refIdx = tempMi.refIdx[list];

            cMv.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
            Position PosY;
            PosY.x = pu.Y().x + (x << MIN_CU_LOG2) + cMv.getHor();
            PosY.y = pu.Y().y + (y << MIN_CU_LOG2) + cMv.getVer();
            clipColPos(PosY.x, PosY.y, pu);
#if JVET_Z0067_RPR_ENABLE
            scalePositionInRef(pu, *pu.cs->pps, RefPicList(list), refIdx, PosY);
            PosY.x = (PosY.x & mask);
            PosY.y = (PosY.y & mask);
            ipm = pu.cu->slice->getRefPic(RefPicList(list), refIdx)->unscaledPic->cs->getIpmInfo(PosY);
#else
            PosY.x = (PosY.x & mask);
            PosY.y = (PosY.y & mask);
            ipm = pu.cu->slice->getRefPic(RefPicList(list), refIdx)->cs->getIpmInfo(PosY);
#endif
          }
          else
          {
#if JVET_Z0067_RPR_ENABLE
            const Picture* pRefPic0 = pu.cu->slice->getRefPic(REF_PIC_LIST_0, tempMi.refIdx[0])->unscaledPic;
#else
            Picture* pRefPic0 = pu.cu->slice->getRefPic(REF_PIC_LIST_0, tempMi.refIdx[0]);
#endif
            Mv cMv0 = tempMi.mv[0];
            cMv0.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
            Position PosY0;
            PosY0.x = pu.Y().x + (x << MIN_CU_LOG2) + cMv0.getHor();
            PosY0.y = pu.Y().y + (y << MIN_CU_LOG2) + cMv0.getVer();
            clipColPos(PosY0.x, PosY0.y, pu);
#if JVET_Z0067_RPR_ENABLE
            scalePositionInRef(pu, *pu.cs->pps, REF_PIC_LIST_0, tempMi.refIdx[0], PosY0);
#endif
            PosY0.x = (PosY0.x & mask);
            PosY0.y = (PosY0.y & mask);
            MotionInfo mi0 = pRefPic0->cs->getMotionInfo(PosY0);
            int ipm0 = pRefPic0->cs->getIpmInfo(PosY0);
            int pocDiff0 = abs(pRefPic0->getPOC() - pu.cu->slice->getPOC());

#if JVET_Z0067_RPR_ENABLE
            const Picture* pRefPic1 = pu.cu->slice->getRefPic(REF_PIC_LIST_1, tempMi.refIdx[1])->unscaledPic;
#else
            Picture* pRefPic1 = pu.cu->slice->getRefPic(REF_PIC_LIST_1, tempMi.refIdx[1]);
#endif
            Mv cMv1 = tempMi.mv[1];
            cMv1.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
            Position PosY1;
            PosY1.x = pu.Y().x + (x << MIN_CU_LOG2) + cMv1.getHor();
            PosY1.y = pu.Y().y + (y << MIN_CU_LOG2) + cMv1.getVer();
            clipColPos(PosY1.x, PosY1.y, pu);
#if JVET_Z0067_RPR_ENABLE
            scalePositionInRef(pu, *pu.cs->pps, REF_PIC_LIST_1, tempMi.refIdx[1], PosY1);
#endif
            PosY1.x = (PosY1.x & mask);
            PosY1.y = (PosY1.y & mask);
            MotionInfo mi1 = pRefPic1->cs->getMotionInfo(PosY1);
            int ipm1 = pRefPic1->cs->getIpmInfo(PosY1);
            int pocDiff1 = abs(pRefPic1->getPOC() - pu.cu->slice->getPOC());

            if (!mi0.isInter && mi1.isInter)
            {
              ipm = ipm0;
            }
            else if (!mi1.isInter && mi0.isInter)
            {
              ipm = ipm1;
            }
            else if (ipm0 > DC_IDX && ipm1 <= DC_IDX)
            {
              ipm = ipm0;
            }
            else if (ipm0 <= DC_IDX && ipm1 > DC_IDX)
            {
              ipm = ipm1;
            }
            else if (pocDiff0 < pocDiff1)
            {
              ipm = ipm0;
            }
            else if (pocDiff1 < pocDiff0)
            {
              ipm = ipm1;
            }
            else if (pRefPic0->m_prevQP[0] > pRefPic1->m_prevQP[0])
            {
              ipm = ipm1;
            }
            else
            {
              ipm = ipm0;
            }
          }
        }
        ii[x] = ipm;
      }
      ii += ib.stride;
    }
  }
  else
#endif
  spanIpmInfoInter(pu, mb, ib);
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
  spanSCCInfo(pu);
#endif
}

#if JVET_AE0169_GPM_IBC_IBC
void PU::spanGeoIBCMotionInfo(PredictionUnit &pu, MergeCtx &geoMrgCtx)
{
  uint8_t mergeIdx0 = pu.ibcGpmMergeIdx0;
  uint8_t mergeIdx1 = pu.ibcGpmMergeIdx1;
  bool    isIntra0  = mergeIdx0 >= GEO_MAX_NUM_UNI_CANDS;
  bool    isIntra1  = mergeIdx1 >= GEO_MAX_NUM_UNI_CANDS;

  uint8_t   splitDir = pu.ibcGpmSplitDir;
  MotionBuf mb       = pu.getMotionBuf();

  uint32_t sliceIdx = pu.cs->slice->getIndependentSliceIdx();

  MergeCtx *mergeCtx0 = &geoMrgCtx;
  MergeCtx *mergeCtx1 = &geoMrgCtx;

  int16_t angle   = g_geoParams[splitDir][0];
  int     tpmMask = 0;
  int     lookUpY = 0, motionIdx = 0;
  bool    isFlip      = angle >= 13 && angle <= 27;
  int     distanceIdx = g_geoParams[splitDir][1];
  int     distanceX   = angle;
  int     distanceY   = (distanceX + (GEO_NUM_ANGLES >> 2)) % GEO_NUM_ANGLES;
  int     offsetX     = (-(int) pu.lwidth()) >> 1;
  int     offsetY     = (-(int) pu.lheight()) >> 1;
  if (distanceIdx > 0)
  {
    if (angle % 16 == 8 || (angle % 16 != 0 && pu.lheight() >= pu.lwidth()))
    {
      offsetY += angle < 16 ? ((distanceIdx * pu.lheight()) >> 3) : -(int) ((distanceIdx * pu.lheight()) >> 3);
    }
    else
    {
      offsetX += angle < 16 ? ((distanceIdx * pu.lwidth()) >> 3) : -(int) ((distanceIdx * pu.lwidth()) >> 3);
    }
  }

  IpmBuf   ib = pu.getIpmBuf();
  uint8_t *ii = ib.buf;
  Mv bv[NUM_REF_PIC_LIST_01];
  if(!isIntra0)
  {
    bv[REF_PIC_LIST_0] = mergeCtx0->mvFieldNeighbours[mergeIdx0 << 1].mv;
    bv[REF_PIC_LIST_0].changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
  }
  if (!isIntra1)
  {
    bv[REF_PIC_LIST_1] = mergeCtx1->mvFieldNeighbours[mergeIdx1 << 1].mv;
    bv[REF_PIC_LIST_1].changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
  }
  const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
  const unsigned mask = ~(scale - 1);
  uint8_t ipmIbc[NUM_REF_PIC_LIST_01];
  for (int i = 0; i < NUM_REF_PIC_LIST_01; i++)
  {
    if((i==0 && !isIntra0) || (i==1 && !isIntra1))
    {
      Position PosY;
      PosY.x              = pu.Y().x + bv[i].getHor();
      PosY.y              = pu.Y().y + bv[i].getVer();
      PosY.x              = (PosY.x & mask);
      PosY.y              = (PosY.y & mask);
      CodingStructure *cs = pu.cs;
      while (cs != NULL && !cs->area.Y().contains(PosY))
      {
         cs = cs->parent;
      }
      ipmIbc[i] = cs->getIpmInfo(PosY);
    }
  }
  if (isIntra0)
  {
    ipmIbc[REF_PIC_LIST_0] = ipmIbc[REF_PIC_LIST_1];
  }
  if(isIntra1)
  {
    ipmIbc[REF_PIC_LIST_1] = ipmIbc[REF_PIC_LIST_0];
  }

  for (int y = 0; y < mb.height; y++)
  {
    lookUpY = (((4 * y + offsetY) << 1) + 5) * g_dis[distanceY];
    for (int x = 0; x < mb.width; x++)
    {
      motionIdx = (((4 * x + offsetX) << 1) + 5) * g_dis[distanceX] + lookUpY;
      tpmMask   = motionIdx <= 0 ? (1 - isFlip) : isFlip;
      int ipm;
      if (tpmMask == 0)
      {
        if (pu.ibcGpmMergeIdx0 >= IBC_GPM_MAX_NUM_UNI_CANDS)
        {
          mb.at(x, y).isInter      = false;
          mb.at(x, y).isIBCmot     = false;
          mb.at(x, y).interDir     = MAX_UCHAR;
          mb.at(x, y).refIdx[0]    = -1;
          mb.at(x, y).refIdx[1]    = -1;
          mb.at(x, y).mv[0]        = Mv();
          mb.at(x, y).mv[1]        = Mv();
          mb.at(x, y).bv           = Mv();
          mb.at(x, y).sliceIdx = sliceIdx;
          ipm = pu.intraDir[0];
        }
        else
        {
          mb.at(x, y).isInter      = true;
          mb.at(x, y).isIBCmot     = true;
          mb.at(x, y).interDir     = mergeCtx0->interDirNeighbours[mergeIdx0];
          mb.at(x, y).useAltHpelIf = pu.cu->imv == IMV_HPEL;
          mb.at(x, y).refIdx[0]    = mergeCtx0->mvFieldNeighbours[mergeIdx0 << 1].refIdx;
          mb.at(x, y).refIdx[1]    = mergeCtx0->mvFieldNeighbours[(mergeIdx0 << 1) + 1].refIdx;
          mb.at(x, y).mv[0]        = mergeCtx0->mvFieldNeighbours[mergeIdx0 << 1].mv;
          mb.at(x, y).mv[1]        = mergeCtx0->mvFieldNeighbours[(mergeIdx0 << 1) + 1].mv;
          mb.at(x, y).bv           = mb.at(x, y).mv[0];
          mb.at(x, y).bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);   // used for only integer resolution
          mb.at(x, y).sliceIdx = sliceIdx;
          ipm = ipmIbc[REF_PIC_LIST_0];
        }
      }
      else
      {
        if (pu.ibcGpmMergeIdx1 >= IBC_GPM_MAX_NUM_UNI_CANDS)
        {
          mb.at(x, y).isInter      = false;
          mb.at(x, y).isIBCmot     = false;
          mb.at(x, y).interDir     = MAX_UCHAR;
          mb.at(x, y).refIdx[0]    = -1;
          mb.at(x, y).refIdx[1]    = -1;
          mb.at(x, y).mv[0]        = Mv();
          mb.at(x, y).mv[1]        = Mv();
          mb.at(x, y).bv           = Mv();
          ipm = pu.intraDir[0];
          mb.at(x, y).sliceIdx = sliceIdx;
        }
        else
        {
          mb.at(x, y).isInter      = true;
          mb.at(x, y).isIBCmot     = true;
          mb.at(x, y).interDir     = mergeCtx1->interDirNeighbours[mergeIdx1];
          mb.at(x, y).useAltHpelIf = pu.cu->imv == IMV_HPEL;
          mb.at(x, y).refIdx[0]    = mergeCtx1->mvFieldNeighbours[mergeIdx1 << 1].refIdx;
          mb.at(x, y).refIdx[1]    = mergeCtx1->mvFieldNeighbours[(mergeIdx1 << 1) + 1].refIdx;
          mb.at(x, y).mv[0]        = mergeCtx1->mvFieldNeighbours[mergeIdx1 << 1].mv;
          mb.at(x, y).mv[1]        = mergeCtx1->mvFieldNeighbours[(mergeIdx1 << 1) + 1].mv;
          mb.at(x, y).bv           = mb.at(x, y).mv[0];
          mb.at(x, y).bv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);   // used for only integer resolution
          mb.at(x, y).sliceIdx = sliceIdx;
          ipm = ipmIbc[REF_PIC_LIST_1];
        }
      }
      ii[x] = ipm;
    }
    ii += ib.stride;
  }
}
#endif

#if JVET_W0097_GPM_MMVD_TM
#if TM_MRG
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
#if JVET_AE0046_BI_GPM
void PU::spanGeoMMVDMotionInfo(PredictionUnit& pu, MergeCtx& geoMrgCtx 
#if JVET_AG0164_AFFINE_GPM
  , AffineMergeCtx& geoAffMrgCtx
#endif
  , MergeCtx& geoTmMrgCtx0, MergeCtx& geoTmMrgCtx1, const uint8_t splitDir, const uint8_t mergeIdx0, const uint8_t mergeIdx1, const bool tmFlag0, const bool mmvdFlag0, const uint8_t mmvdIdx0, const bool tmFlag1, const bool mmvdFlag1, const uint8_t mmvdIdx1, const uint8_t bldIdx, const uint8_t *intraMPM,
  const bool dmvrPart0, const bool dmvrPart1, Mv* bdofSubPuMvOffsetPart0, Mv* bdofSubPuMvOffsetPart1)
#else
void PU::spanGeoMMVDMotionInfo(PredictionUnit &pu, MergeCtx &geoMrgCtx, MergeCtx &geoTmMrgCtx0, MergeCtx &geoTmMrgCtx1, const uint8_t splitDir, const uint8_t mergeIdx0, const uint8_t mergeIdx1, const bool tmFlag0, const bool mmvdFlag0, const uint8_t mmvdIdx0, const bool tmFlag1, const bool mmvdFlag1, const uint8_t mmvdIdx1, const uint8_t bldIdx, const uint8_t *intraMPM)
#endif
#else
void PU::spanGeoMMVDMotionInfo(PredictionUnit &pu, MergeCtx &geoMrgCtx, MergeCtx &geoTmMrgCtx0, MergeCtx &geoTmMrgCtx1, const uint8_t splitDir, const uint8_t mergeIdx0, const uint8_t mergeIdx1, const bool tmFlag0, const bool mmvdFlag0, const uint8_t mmvdIdx0, const bool tmFlag1, const bool mmvdFlag1, const uint8_t mmvdIdx1)
#endif
#else
#if JVET_AA0058_GPM_ADAPTIVE_BLENDING
#if JVET_AE0046_BI_GPM
void PU::spanGeoMMVDMotionInfo(PredictionUnit& pu, MergeCtx& geoMrgCtx, const uint8_t splitDir, const uint8_t mergeIdx0, const uint8_t mergeIdx1, const bool mmvdFlag0, const uint8_t mmvdIdx0, const bool mmvdFlag1, const uint8_t mmvdIdx1, const uint8_t bldIdx,
  const bool dmvrPart0, const bool dmvrPart1, Mv* bdofSubPuMvOffsetPart0, Mv* bdofSubPuMvOffsetPart1)
#else
void PU::spanGeoMMVDMotionInfo( PredictionUnit &pu, MergeCtx &geoMrgCtx, const uint8_t splitDir, const uint8_t mergeIdx0, const uint8_t mergeIdx1, const bool mmvdFlag0, const uint8_t mmvdIdx0, const bool mmvdFlag1, const uint8_t mmvdIdx1, const uint8_t bldIdx )
#endif
#else
void PU::spanGeoMMVDMotionInfo( PredictionUnit &pu, MergeCtx &geoMrgCtx, const uint8_t splitDir, const uint8_t mergeIdx0, const uint8_t mergeIdx1, const bool mmvdFlag0, const uint8_t mmvdIdx0, const bool mmvdFlag1, const uint8_t mmvdIdx1 )
#endif
#endif
{
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
  const int log2WeightBase = 5;  // BCW is 3, GEO is 5
  int16_t one   = 1 << log2WeightBase;
  int16_t half  = one >> 1;
  AffineBlendingModel& blendModel = pu.cu->blendModel;
  if ( pu.cu->geoBlendFlag )
  {
    bool bTest = tmFlag0 || mmvdFlag0 || tmFlag1 || mmvdFlag1 || splitDir != 0 || bldIdx != 0 ;
    bTest |=  pu.gpmIntraFlag || (mergeIdx0 >= GEO_MAX_NUM_UNI_CANDS) || (mergeIdx1 >= GEO_MAX_NUM_UNI_CANDS);
#if JVET_AG0164_AFFINE_GPM
    bTest |= pu.affineGPM[0] || pu.affineGPM[1];
#endif
    CHECK(bTest,"PU::spanGeoMMVDMotionInfo( geoBlendFlag = 1 ) failed.");
  }
#endif
  pu.geoSplitDir = splitDir;
  pu.geoMergeIdx0 = mergeIdx0;
  pu.geoMergeIdx1 = mergeIdx1;
  pu.geoMMVDFlag0 = mmvdFlag0;
  pu.geoMMVDIdx0 = mmvdIdx0;
  pu.geoMMVDFlag1 = mmvdFlag1;
  pu.geoMMVDIdx1 = mmvdIdx1;

#if TM_MRG
  pu.geoTmFlag0 = tmFlag0;
  pu.geoTmFlag1 = tmFlag1;
  MergeCtx *mergeCtx0 = (tmFlag0 ? &geoTmMrgCtx0 : &geoMrgCtx);
  MergeCtx *mergeCtx1 = (tmFlag1 ? &geoTmMrgCtx1 : &geoMrgCtx);
#else
  MergeCtx *mergeCtx0 = &geoMrgCtx;
  MergeCtx *mergeCtx1 = &geoMrgCtx;
#endif

  const int mvShift = MV_FRACTIONAL_BITS_DIFF;
  const bool extMMVD = pu.cs->picHeader->getGPMMMVDTableFlag();
  const int mmvdCands[8] = { 1 << mvShift , 2 << mvShift , 4 << mvShift , 8 << mvShift , 16 << mvShift , 32 << mvShift,  64 << mvShift , 128 << mvShift };
  const int extMmvdCands[9] = { 1 << mvShift , 2 << mvShift , 4 << mvShift , 8 << mvShift , 12 << mvShift , 16 << mvShift, 24 << mvShift, 32 << mvShift, 64 << mvShift };
  const int* refMvdCands = (extMMVD ? extMmvdCands : mmvdCands);
  Mv mvOffset0[2], mvOffset1[2], deltaMv;

#if JVET_AE0046_BI_GPM
#if JVET_AG0164_AFFINE_GPM
  int interDirIdx0 = (mergeIdx0 >= GEO_MAX_ALL_INTER_UNI_CANDS) ? -1 : mergeCtx0->interDirNeighbours[mergeIdx0];
#else
  int interDirIdx0 = (mergeIdx0 >= GEO_MAX_NUM_UNI_CANDS) ? -1 : mergeCtx0->interDirNeighbours[mergeIdx0];
#endif
  if (mmvdFlag0)
  {
    if (mergeCtx0->interDirNeighbours[mergeIdx0] == 3)
    {
      if (!pu.cs->slice->getCheckLDC())
      {
        interDirIdx0 = (mergeIdx0 % 2) + 1;
      }
    }
  }
#if JVET_AG0164_AFFINE_GPM
  int interDirIdx1 = (mergeIdx1 >= GEO_MAX_ALL_INTER_UNI_CANDS) ? -1 : mergeCtx1->interDirNeighbours[mergeIdx1];
#else
  int interDirIdx1 = (mergeIdx1 >= GEO_MAX_NUM_UNI_CANDS) ? -1 : mergeCtx1->interDirNeighbours[mergeIdx1];
#endif
  if (mmvdFlag1)
  {
    if (mergeCtx1->interDirNeighbours[mergeIdx1] == 3)
    {
      if (!pu.cs->slice->getCheckLDC())
      {
        interDirIdx1 = (mergeIdx1 % 2) + 1;
      }
    }
  }
#endif


  if (mmvdFlag0)
  {
    int fPosStep = (extMMVD ? (mmvdIdx0 >> 3) : (mmvdIdx0 >> 2));
    int fPosPosition = (extMMVD ? (mmvdIdx0 - (fPosStep << 3)) : (mmvdIdx0 - (fPosStep << 2)));

    if (fPosPosition == 0)
    {
      deltaMv = Mv(refMvdCands[fPosStep], 0);
    }
    else if (fPosPosition == 1)
    {
      deltaMv = Mv(-refMvdCands[fPosStep], 0);
    }
    else if (fPosPosition == 2)
    {
      deltaMv = Mv(0, refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 3)
    {
      deltaMv = Mv(0, -refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 4)
    {
      deltaMv = Mv(refMvdCands[fPosStep], refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 5)
    {
      deltaMv = Mv(refMvdCands[fPosStep], -refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 6)
    {
      deltaMv = Mv(-refMvdCands[fPosStep], refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 7)
    {
      deltaMv = Mv(-refMvdCands[fPosStep], -refMvdCands[fPosStep]);
    }

#if JVET_AE0046_BI_GPM
    if (interDirIdx0 == 1)
    {
      mvOffset0[0] = deltaMv;
    }
    else if (interDirIdx0 == 2)
    {
      mvOffset0[1] = deltaMv;
    }
#else
    if (mergeCtx0->interDirNeighbours[mergeIdx0] == 1)
    {
      mvOffset0[0] = deltaMv;
    }
    else
    {
      mvOffset0[1] = deltaMv;
    }
#endif
#if JVET_AE0046_BI_GPM
    else
    {
      CHECK(mergeCtx0->interDirNeighbours[mergeIdx0] == 2, "Error in inter dir when setting MMVD GPM");
      const int refListIdx0 = mergeCtx0->mvFieldNeighbours[mergeIdx0 << 1].refIdx;
      const int refListIdx1 = mergeCtx0->mvFieldNeighbours[(mergeIdx0 << 1) + 1].refIdx;

      const int poc0 = pu.cs->slice->getRefPOC(REF_PIC_LIST_0, refListIdx0);
      const int poc1 = pu.cs->slice->getRefPOC(REF_PIC_LIST_1, refListIdx1);
      const int currPoc = pu.cs->slice->getPOC();

      mvOffset0[0] = deltaMv;

      if ((poc0 - currPoc) == (poc1 - currPoc))
      {
        mvOffset0[1] = mvOffset0[0];
      }
      else if (abs(poc1 - currPoc) > abs(poc0 - currPoc))
      {
        const int scale = PU::getDistScaleFactor(currPoc, poc0, currPoc, poc1);
        mvOffset0[1] = mvOffset0[0];

        const bool isL0RefLongTerm = pu.cs->slice->getRefPic(REF_PIC_LIST_0, refListIdx0)->longTerm;
        const bool isL1RefLongTerm = pu.cs->slice->getRefPic(REF_PIC_LIST_1, refListIdx1)->longTerm;

        if (isL0RefLongTerm || isL1RefLongTerm)
        {
          if ((poc1 - currPoc) * (poc0 - currPoc) > 0)
          {
            mvOffset0[0] = mvOffset0[1];
          }
          else
          {
            mvOffset0[0].set(-1 * mvOffset0[1].getHor(), -1 * mvOffset0[1].getVer());
          }
        }
        else
        {
          mvOffset0[0] = mvOffset0[1].scaleMv(scale);
        }
      }
      else
      {
        const int scale = PU::getDistScaleFactor(currPoc, poc1, currPoc, poc0);
        const bool isL0RefLongTerm = pu.cs->slice->getRefPic(REF_PIC_LIST_0, refListIdx0)->longTerm;
        const bool isL1RefLongTerm = pu.cs->slice->getRefPic(REF_PIC_LIST_1, refListIdx1)->longTerm;
        if (isL0RefLongTerm || isL1RefLongTerm)
        {
          if ((poc1 - currPoc) * (poc0 - currPoc) > 0)
          {
            mvOffset0[1] = mvOffset0[0];
          }
          else
          {
            mvOffset0[1].set(-1 * mvOffset0[0].getHor(), -1 * mvOffset0[0].getVer());
          }
        }
        else
        {
          mvOffset0[1] = mvOffset0[0].scaleMv(scale);
        }
      }
    }
#endif
  }

  if (mmvdFlag1)
  {
    int fPosStep = (extMMVD ? (mmvdIdx1 >> 3) : (mmvdIdx1 >> 2));
    int fPosPosition = (extMMVD ? (mmvdIdx1 - (fPosStep << 3)) : (mmvdIdx1 - (fPosStep << 2)));

    if (fPosPosition == 0)
    {
      deltaMv = Mv(refMvdCands[fPosStep], 0);
    }
    else if (fPosPosition == 1)
    {
      deltaMv = Mv(-refMvdCands[fPosStep], 0);
    }
    else if (fPosPosition == 2)
    {
      deltaMv = Mv(0, refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 3)
    {
      deltaMv = Mv(0, -refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 4)
    {
      deltaMv = Mv(refMvdCands[fPosStep], refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 5)
    {
      deltaMv = Mv(refMvdCands[fPosStep], -refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 6)
    {
      deltaMv = Mv(-refMvdCands[fPosStep], refMvdCands[fPosStep]);
    }
    else if (fPosPosition == 7)
    {
      deltaMv = Mv(-refMvdCands[fPosStep], -refMvdCands[fPosStep]);
    }

#if JVET_AE0046_BI_GPM
    if (interDirIdx1 == 1)
    {
      mvOffset1[0] = deltaMv;
    }
    else if (interDirIdx1 == 2)
    {
      mvOffset1[1] = deltaMv;
    }
#else
    if (mergeCtx1->interDirNeighbours[mergeIdx1] == 1)
    {
      mvOffset1[0] = deltaMv;
    }
    else
    {
      mvOffset1[1] = deltaMv;
    }
#endif
#if JVET_AE0046_BI_GPM
    else
    {
      CHECK(mergeCtx1->interDirNeighbours[mergeIdx1] == 2, "Error in inter dir when setting MMVD GPM");
      const int refListIdx0 = mergeCtx1->mvFieldNeighbours[ mergeIdx1 << 1     ].refIdx;
      const int refListIdx1 = mergeCtx1->mvFieldNeighbours[(mergeIdx1 << 1) + 1].refIdx;

      const int poc0 = pu.cs->slice->getRefPOC(REF_PIC_LIST_0, refListIdx0);
      const int poc1 = pu.cs->slice->getRefPOC(REF_PIC_LIST_1, refListIdx1);
      const int currPoc = pu.cs->slice->getPOC();

      mvOffset1[0] = deltaMv;

      if ((poc0 - currPoc) == (poc1 - currPoc))
      {
        mvOffset1[1] = mvOffset1[0];
      }
      else if (abs(poc1 - currPoc) > abs(poc0 - currPoc))
      {
        const int scale = PU::getDistScaleFactor(currPoc, poc0, currPoc, poc1);
        mvOffset1[1] = mvOffset1[0];

        const bool isL0RefLongTerm = pu.cs->slice->getRefPic(REF_PIC_LIST_0, refListIdx0)->longTerm;
        const bool isL1RefLongTerm = pu.cs->slice->getRefPic(REF_PIC_LIST_1, refListIdx1)->longTerm;

        if (isL0RefLongTerm || isL1RefLongTerm)
        {
          if ((poc1 - currPoc) * (poc0 - currPoc) > 0)
          {
            mvOffset1[0] = mvOffset1[1];
          }
          else
          {
            mvOffset1[0].set(-1 * mvOffset1[1].getHor(), -1 * mvOffset1[1].getVer());
          }
        }
        else
        {
          mvOffset1[0] = mvOffset1[1].scaleMv(scale);
        }
      }
      else
      {
        const int scale = PU::getDistScaleFactor(currPoc, poc1, currPoc, poc0);
        const bool isL0RefLongTerm = pu.cs->slice->getRefPic(REF_PIC_LIST_0, refListIdx0)->longTerm;
        const bool isL1RefLongTerm = pu.cs->slice->getRefPic(REF_PIC_LIST_1, refListIdx1)->longTerm;
        if (isL0RefLongTerm || isL1RefLongTerm)
        {
          if ((poc1 - currPoc) * (poc0 - currPoc) > 0)
          {
            mvOffset1[1] = mvOffset1[0];
          }
          else
          {
            mvOffset1[1].set(-1 * mvOffset1[0].getHor(), -1 * mvOffset1[0].getVer());
          }
        }
        else
        {
          mvOffset1[1] = mvOffset1[0].scaleMv(scale);
        }
      }
    }
#endif
  }

  MotionBuf mb = pu.getMotionBuf();
#if JVET_AG0164_AFFINE_GPM
  Mv affMvBuf0[2][(MAX_CU_SIZE >> 2) * (MAX_CU_SIZE >> 2)];
  Mv affMvBuf1[2][(MAX_CU_SIZE >> 2) * (MAX_CU_SIZE >> 2)];
  int affMvBufStride = MAX_CU_SIZE >> 2;
  if (pu.affineGPM[0])
  {
    pu.gpmPartAffType[0] = geoAffMrgCtx.affineType[mergeIdx0];
    CHECK(pu.gpmPartAffType[0] == AFFINE_MODEL_NUM, "Invalid affine type");
    pu.gpmPartRefIdx[0][0] = pu.gpmPartRefIdx[0][1] = -1;

    pu.cu->affineType = geoAffMrgCtx.affineType[mergeIdx0];
    MvField* cpmvMvField0 = geoAffMrgCtx.mvFieldNeighbours[mergeIdx0 << 1];
    if (cpmvMvField0[0].refIdx >= 0)
    {
      PU::setAllAffineMv(pu, cpmvMvField0[0].mv + deltaMv, cpmvMvField0[1].mv + deltaMv, cpmvMvField0[2].mv + deltaMv, REF_PIC_LIST_0);

      pu.gpmPartRefIdx[0][0]    = cpmvMvField0[0].refIdx;
      pu.gpmPartmvAffi[0][0][0] = cpmvMvField0[0].mv + deltaMv;
      pu.gpmPartmvAffi[0][0][1] = cpmvMvField0[1].mv + deltaMv;
      pu.gpmPartmvAffi[0][0][2] = cpmvMvField0[2].mv + deltaMv;

    }
    MvField* cpmvMvField1 = geoAffMrgCtx.mvFieldNeighbours[(mergeIdx0 << 1) + 1];
    if (cpmvMvField1[0].refIdx >= 0)
    {
      PU::setAllAffineMv(pu, cpmvMvField1[0].mv + deltaMv, cpmvMvField1[1].mv + deltaMv, cpmvMvField1[2].mv + deltaMv, REF_PIC_LIST_1);

      pu.gpmPartRefIdx[0][1]    = cpmvMvField1[0].refIdx;
      pu.gpmPartmvAffi[0][1][0] = cpmvMvField1[0].mv + deltaMv;
      pu.gpmPartmvAffi[0][1][1] = cpmvMvField1[1].mv + deltaMv;
      pu.gpmPartmvAffi[0][1][2] = cpmvMvField1[2].mv + deltaMv;
    }
    MotionInfo* pMiBuf = mb.buf;
    Mv* pMvBuf0 = affMvBuf0[0];
    Mv* pMvBuf1 = affMvBuf0[1];
    for (int y = 0; y < mb.height; y++)
    {
      for (int x = 0; x < mb.width; x++)
      {
        pMvBuf0[x] = cpmvMvField0[0].refIdx >= 0 ? pMiBuf[x].mv[0] : Mv();
        pMvBuf1[x] = cpmvMvField1[0].refIdx >= 0 ? pMiBuf[x].mv[1] : Mv();
      }
      pMiBuf += mb.stride;
      pMvBuf0 += affMvBufStride;
      pMvBuf1 += affMvBufStride;
    }
  }
  if (pu.affineGPM[1])
  {
    pu.gpmPartAffType[1] = geoAffMrgCtx.affineType[mergeIdx1];
    CHECK(pu.gpmPartAffType[1] == AFFINE_MODEL_NUM, "Invalid affine type");
    pu.gpmPartRefIdx[1][0] = pu.gpmPartRefIdx[1][1] = -1;

    pu.cu->affineType = geoAffMrgCtx.affineType[mergeIdx1];
    MvField* cpmvMvField0 = geoAffMrgCtx.mvFieldNeighbours[(mergeIdx1 << 1)];
    if (cpmvMvField0[0].refIdx >= 0)
    {
      PU::setAllAffineMv(pu, cpmvMvField0[0].mv + deltaMv, cpmvMvField0[1].mv + deltaMv, cpmvMvField0[2].mv + deltaMv, REF_PIC_LIST_0);

      pu.gpmPartRefIdx[1][0]    = cpmvMvField0[0].refIdx;
      pu.gpmPartmvAffi[1][0][0] = cpmvMvField0[0].mv + deltaMv;
      pu.gpmPartmvAffi[1][0][1] = cpmvMvField0[1].mv + deltaMv;
      pu.gpmPartmvAffi[1][0][2] = cpmvMvField0[2].mv + deltaMv;
    }
    MvField* cpmvMvField1 = geoAffMrgCtx.mvFieldNeighbours[(mergeIdx1 << 1) + 1];
    if (cpmvMvField1[0].refIdx >= 0)
    {
      PU::setAllAffineMv(pu, cpmvMvField1[0].mv + deltaMv, cpmvMvField1[1].mv + deltaMv, cpmvMvField1[2].mv + deltaMv, REF_PIC_LIST_1);

      pu.gpmPartRefIdx[1][1]    = cpmvMvField1[0].refIdx;
      pu.gpmPartmvAffi[1][1][0] = cpmvMvField1[0].mv + deltaMv;
      pu.gpmPartmvAffi[1][1][1] = cpmvMvField1[1].mv + deltaMv;
      pu.gpmPartmvAffi[1][1][2] = cpmvMvField1[2].mv + deltaMv;
    }
    MotionInfo* pMiBuf = mb.buf;
    Mv* pMvBuf0 = affMvBuf1[0];
    Mv* pMvBuf1 = affMvBuf1[1];
    for (int y = 0; y < mb.height; y++)
    {
      for (int x = 0; x < mb.width; x++)
      {
        pMvBuf0[x] = cpmvMvField0[0].refIdx >= 0 ? pMiBuf[x].mv[0] : Mv();
        pMvBuf1[x] = cpmvMvField1[0].refIdx >= 0 ? pMiBuf[x].mv[1] : Mv();
      }
      pMiBuf += mb.stride;
      pMvBuf0 += affMvBufStride;
      pMvBuf1 += affMvBufStride;
    }
  }
#endif


#if JVET_Y0065_GPM_INTRA
#if JVET_AG0164_AFFINE_GPM
  bool isIntra0 = mergeIdx0 >= GEO_MAX_ALL_INTER_UNI_CANDS;
  bool isIntra1 = mergeIdx1 >= GEO_MAX_ALL_INTER_UNI_CANDS;
#else
  bool isIntra0 = mergeIdx0 >= GEO_MAX_NUM_UNI_CANDS;
  bool isIntra1 = mergeIdx1 >= GEO_MAX_NUM_UNI_CANDS;
#endif
  uint32_t sliceIdx = pu.cs->slice->getIndependentSliceIdx();
#else
#if !JVET_AE0046_BI_GPM
  MotionInfo biMv;
  biMv.isInter = true;
  biMv.sliceIdx = pu.cs->slice->getIndependentSliceIdx();

  if (mergeCtx0->interDirNeighbours[mergeIdx0] == 1 && mergeCtx1->interDirNeighbours[mergeIdx1] == 2)
  {
    biMv.interDir = 3;
    biMv.mv[0] = mergeCtx0->mvFieldNeighbours[mergeIdx0 << 1].mv + mvOffset0[0];
    biMv.mv[1] = mergeCtx1->mvFieldNeighbours[(mergeIdx1 << 1) + 1].mv + mvOffset1[1];
    biMv.refIdx[0] = mergeCtx0->mvFieldNeighbours[mergeIdx0 << 1].refIdx;
    biMv.refIdx[1] = mergeCtx1->mvFieldNeighbours[(mergeIdx1 << 1) + 1].refIdx;
  }
  else if (mergeCtx0->interDirNeighbours[mergeIdx0] == 2 && mergeCtx1->interDirNeighbours[mergeIdx1] == 1)
  {
    biMv.interDir = 3;
    biMv.mv[0] = mergeCtx1->mvFieldNeighbours[mergeIdx1 << 1].mv + mvOffset1[0];
    biMv.mv[1] = mergeCtx0->mvFieldNeighbours[(mergeIdx0 << 1) + 1].mv + mvOffset0[1];
    biMv.refIdx[0] = mergeCtx1->mvFieldNeighbours[mergeIdx1 << 1].refIdx;
    biMv.refIdx[1] = mergeCtx0->mvFieldNeighbours[(mergeIdx0 << 1) + 1].refIdx;
  }
  else if (mergeCtx0->interDirNeighbours[mergeIdx0] == 1 && mergeCtx1->interDirNeighbours[mergeIdx1] == 1)
  {
    biMv.interDir = 1;
    biMv.mv[0] = mergeCtx1->mvFieldNeighbours[mergeIdx1 << 1].mv + mvOffset1[0];
    biMv.mv[1] = Mv(0, 0);
    biMv.refIdx[0] = mergeCtx1->mvFieldNeighbours[mergeIdx1 << 1].refIdx;
    biMv.refIdx[1] = -1;
  }
  else if (mergeCtx0->interDirNeighbours[mergeIdx0] == 2 && mergeCtx1->interDirNeighbours[mergeIdx1] == 2)
  {
    biMv.interDir = 2;
    biMv.mv[0] = Mv(0, 0);
    biMv.mv[1] = mergeCtx1->mvFieldNeighbours[(mergeIdx1 << 1) + 1].mv + mvOffset1[1];
    biMv.refIdx[0] = -1;
    biMv.refIdx[1] = mergeCtx1->mvFieldNeighbours[(mergeIdx1 << 1) + 1].refIdx;
  }
#endif
#endif

  int16_t angle = g_geoParams[splitDir][0];
  int tpmMask = 0;
  int lookUpY = 0, motionIdx = 0;
  bool isFlip = angle >= 13 && angle <= 27;
  int distanceIdx = g_geoParams[splitDir][1];
  int distanceX = angle;
  int distanceY = (distanceX + (GEO_NUM_ANGLES >> 2)) % GEO_NUM_ANGLES;
  int offsetX = (-(int)pu.lwidth()) >> 1;
  int offsetY = (-(int)pu.lheight()) >> 1;
  if (distanceIdx > 0)
  {
    if (angle % 16 == 8 || (angle % 16 != 0 && pu.lheight() >= pu.lwidth()))
    {
      offsetY += angle < 16 ? ((distanceIdx * pu.lheight()) >> 3) : -(int)((distanceIdx * pu.lheight()) >> 3);
    }
    else
    {
      offsetX += angle < 16 ? ((distanceIdx * pu.lwidth()) >> 3) : -(int)((distanceIdx * pu.lwidth()) >> 3);
    }
  }

#if JVET_AG0164_AFFINE_GPM
  Mv* pMvBufP0L0 = affMvBuf0[0];
  Mv* pMvBufP0L1 = affMvBuf0[1];
  Mv* pMvBufP1L0 = affMvBuf1[0];
  Mv* pMvBufP1L1 = affMvBuf1[1];
#endif
  for (int y = 0; y < mb.height; y++)
  {
    lookUpY = (((4 * y + offsetY) << 1) + 5) * g_dis[distanceY];
    for (int x = 0; x < mb.width; x++)
    {
#if JVET_AE0046_BI_GPM
      Position samplePos(x << MIN_CU_LOG2, y << MIN_CU_LOG2);
      int bdofSubIdx = (samplePos.y >> BDOF_SUBPU_DIM_LOG2) * BDOF_SUBPU_STRIDE + (samplePos.x >> BDOF_SUBPU_DIM_LOG2);
#endif
      motionIdx = (((4 * x + offsetX) << 1) + 5) * g_dis[distanceX] + lookUpY;
      mb.at(x, y).bcwIdx = pu.cu->bcwIdx;
#if JVET_Y0065_GPM_INTRA
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
      if ( pu.cu->geoBlendFlag )
      {
        int iWeight = blendModel.compute( samplePos.x, samplePos.y );
        int weight0 = one - iWeight;
        tpmMask = (weight0 < half) ? 1 : 0;
      }
      else
#endif
      tpmMask = motionIdx <= 0 ? (1 - isFlip) : isFlip;
      if (tpmMask == 0 && isIntra0)
      {
        mb.at(x, y).isInter = false;
        mb.at(x, y).interDir = MAX_UCHAR;
        mb.at(x, y).refIdx[0] = -1;
        mb.at(x, y).refIdx[1] = -1;
        mb.at(x, y).mv[0] = Mv();
        mb.at(x, y).mv[1] = Mv();
        mb.at(x, y).sliceIdx = sliceIdx;
      }
#else
#if !JVET_AE0046_BI_GPM
      tpmMask = abs(motionIdx) < 32 ? 2 : (motionIdx <= 0 ? (1 - isFlip) : isFlip);
      if (tpmMask == 2)
      {
        mb.at(x, y).isInter = true;
        mb.at(x, y).interDir = biMv.interDir;
        mb.at(x, y).refIdx[0] = biMv.refIdx[0];
        mb.at(x, y).refIdx[1] = biMv.refIdx[1];
        mb.at(x, y).mv[0] = biMv.mv[0];
        mb.at(x, y).mv[1] = biMv.mv[1];
        mb.at(x, y).sliceIdx = biMv.sliceIdx;
      }
#endif
#endif
      else if (tpmMask == 0)
      {
        mb.at(x, y).isInter = true;
#if JVET_AG0164_AFFINE_GPM
        if (pu.affineGPM[0])
        {
          mb.at(x, y).interDir = geoAffMrgCtx.interDirNeighbours[mergeIdx0];
          mb.at(x, y).refIdx[0] = geoAffMrgCtx.mvFieldNeighbours[mergeIdx0 << 1][0].refIdx;
          mb.at(x, y).refIdx[1] = geoAffMrgCtx.mvFieldNeighbours[(mergeIdx0 << 1) + 1][0].refIdx;
          mb.at(x, y).mv[0] = pMvBufP0L0[x];
          mb.at(x, y).mv[1] = pMvBufP0L1[x];
        }
        else
        {
#endif
#if JVET_AE0046_BI_GPM
        mb.at(x, y).interDir = interDirIdx0;
        mb.at(x, y).refIdx[0] = (interDirIdx0 == 1 || interDirIdx0 == 3) ? mergeCtx0->mvFieldNeighbours[mergeIdx0 << 1].refIdx : -1;
        mb.at(x, y).refIdx[1] = (interDirIdx0 == 2 || interDirIdx0 == 3) ? mergeCtx0->mvFieldNeighbours[(mergeIdx0 << 1) + 1].refIdx : -1;
        mb.at(x, y).mv[0] = dmvrPart0 ? mergeCtx0->mvFieldNeighbours[mergeIdx0 << 1].mv + bdofSubPuMvOffsetPart0[bdofSubIdx] : mergeCtx0->mvFieldNeighbours[mergeIdx0 << 1].mv + mvOffset0[0];
        mb.at(x, y).mv[1] = dmvrPart0 ? mergeCtx0->mvFieldNeighbours[(mergeIdx0 << 1) + 1].mv - bdofSubPuMvOffsetPart0[bdofSubIdx] : mergeCtx0->mvFieldNeighbours[(mergeIdx0 << 1) + 1].mv + mvOffset0[1];
        if (interDirIdx0 == 1)
        {
          mb.at(x, y).mv[1] = Mv();
        }
        else if (interDirIdx0 == 2)
        {
          mb.at(x, y).mv[0] = Mv();
        }
#else
        mb.at(x, y).interDir = mergeCtx0->interDirNeighbours[mergeIdx0];
        mb.at(x, y).refIdx[0] = mergeCtx0->mvFieldNeighbours[mergeIdx0 << 1].refIdx;
        mb.at(x, y).refIdx[1] = mergeCtx0->mvFieldNeighbours[(mergeIdx0 << 1) + 1].refIdx;
        mb.at(x, y).mv[0] = mergeCtx0->mvFieldNeighbours[mergeIdx0 << 1].mv + mvOffset0[0];
        mb.at(x, y).mv[1] = mergeCtx0->mvFieldNeighbours[(mergeIdx0 << 1) + 1].mv + mvOffset0[1];
#endif
#if JVET_AG0164_AFFINE_GPM
        }
#endif
#if JVET_Y0065_GPM_INTRA || JVET_AE0046_BI_GPM
        mb.at(x, y).sliceIdx = sliceIdx;
#else
        mb.at(x, y).sliceIdx = biMv.sliceIdx;
#endif
      }
#if JVET_Y0065_GPM_INTRA
      else if (tpmMask == 1 && isIntra1)
      {
        mb.at(x, y).isInter = false;
        mb.at(x, y).interDir = MAX_UCHAR;
        mb.at(x, y).refIdx[0] = -1;
        mb.at(x, y).refIdx[1] = -1;
        mb.at(x, y).mv[0] = Mv();
        mb.at(x, y).mv[1] = Mv();
        mb.at(x, y).sliceIdx = sliceIdx;
      }
#endif
      else
      {
        mb.at(x, y).isInter = true;
#if JVET_AG0164_AFFINE_GPM
        if (pu.affineGPM[1])
        {
          mb.at(x, y).interDir = geoAffMrgCtx.interDirNeighbours[mergeIdx1];
          mb.at(x, y).refIdx[0] = geoAffMrgCtx.mvFieldNeighbours[mergeIdx1 << 1][0].refIdx;
          mb.at(x, y).refIdx[1] = geoAffMrgCtx.mvFieldNeighbours[(mergeIdx1 << 1) + 1][0].refIdx;
          mb.at(x, y).mv[0] = pMvBufP1L0[x];
          mb.at(x, y).mv[1] = pMvBufP1L1[x];
        }
        else
        {
#endif
#if JVET_AE0046_BI_GPM
        mb.at(x, y).interDir = interDirIdx1;
        mb.at(x, y).refIdx[0] = (interDirIdx1 == 1 || interDirIdx1 == 3) ? mergeCtx1->mvFieldNeighbours[mergeIdx1 << 1].refIdx : -1;
        mb.at(x, y).refIdx[1] = (interDirIdx1 == 2 || interDirIdx1 == 3) ? mergeCtx1->mvFieldNeighbours[(mergeIdx1 << 1) + 1].refIdx : -1;
        mb.at(x, y).mv[0] = dmvrPart1 ? mergeCtx1->mvFieldNeighbours[mergeIdx1 << 1].mv + bdofSubPuMvOffsetPart1[bdofSubIdx] : mergeCtx1->mvFieldNeighbours[mergeIdx1 << 1].mv + mvOffset1[0];
        mb.at(x, y).mv[1] = dmvrPart1 ? mergeCtx1->mvFieldNeighbours[(mergeIdx1 << 1) + 1].mv - bdofSubPuMvOffsetPart1[bdofSubIdx] : mergeCtx1->mvFieldNeighbours[(mergeIdx1 << 1) + 1].mv + mvOffset1[1];
        if (interDirIdx1 == 1)
        {
          mb.at(x, y).mv[1] = Mv();
        }
        else if (interDirIdx1 == 2)
        {
          mb.at(x, y).mv[0] = Mv();
        }
#else
        mb.at(x, y).interDir = mergeCtx1->interDirNeighbours[mergeIdx1];
        mb.at(x, y).refIdx[0] = mergeCtx1->mvFieldNeighbours[mergeIdx1 << 1].refIdx;
        mb.at(x, y).refIdx[1] = mergeCtx1->mvFieldNeighbours[(mergeIdx1 << 1) + 1].refIdx;
        mb.at(x, y).mv[0] = mergeCtx1->mvFieldNeighbours[mergeIdx1 << 1].mv + mvOffset1[0];
        mb.at(x, y).mv[1] = mergeCtx1->mvFieldNeighbours[(mergeIdx1 << 1) + 1].mv + mvOffset1[1];
#endif
#if JVET_AG0164_AFFINE_GPM
        }
#endif
#if JVET_Y0065_GPM_INTRA || JVET_AE0046_BI_GPM
        mb.at(x, y).sliceIdx = sliceIdx;
#else
        mb.at(x, y).sliceIdx = biMv.sliceIdx;
#endif
      }
#if JVET_AG0164_AFFINE_GPM
      CHECK(tpmMask != 0 && tpmMask != 1, "Invalid TPM mask");
      mb.at(x, y).gpmPartIdx = tpmMask;
#endif
    }
#if JVET_AG0164_AFFINE_GPM
    pMvBufP0L0 += affMvBufStride;
    pMvBufP0L1 += affMvBufStride;
    pMvBufP1L0 += affMvBufStride;
    pMvBufP1L1 += affMvBufStride;
#endif
  }

#if JVET_W0123_TIMD_FUSION
  IpmBuf ib = pu.getIpmBuf();
#if JVET_Y0065_GPM_INTRA
  if (pu.gpmIntraFlag)
  {
    const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
    const unsigned mask = ~(scale - 1);
    uint8_t* ii = ib.buf;
    int geoIpm[3];
#if JVET_AG0164_AFFINE_GPM
    geoIpm[0] = isIntra0 ? intraMPM[mergeIdx0 - GEO_MAX_ALL_INTER_UNI_CANDS] : -1;
    geoIpm[1] = isIntra1 ? intraMPM[mergeIdx1 - GEO_MAX_ALL_INTER_UNI_CANDS + GEO_MAX_NUM_INTRA_CANDS] : -1;
#else
    geoIpm[0] = isIntra0 ? intraMPM[mergeIdx0 - GEO_MAX_NUM_UNI_CANDS] : -1;
    geoIpm[1] = isIntra1 ? intraMPM[mergeIdx1 - GEO_MAX_NUM_UNI_CANDS + GEO_MAX_NUM_INTRA_CANDS] : -1;
#endif
    geoIpm[2] = (isIntra0 && isIntra1) ? ((mergeIdx1 < mergeIdx0) ? geoIpm[1] : geoIpm[0]) : isIntra0 ? geoIpm[0] : geoIpm[1];

    for (int y = 0; y < mb.height; y++)
    {
      lookUpY = (((4 * y + offsetY) << 1) + 5) * g_dis[distanceY];
      for (int x = 0; x < mb.width; x++)
      {
        motionIdx = (((4 * x + offsetX) << 1) + 5) * g_dis[distanceX] + lookUpY;
        tpmMask = motionIdx <= 0 ? (1 - isFlip) : isFlip;
        uint8_t ipm;
        if (geoIpm[tpmMask] >= 0)
        {
          ipm = (uint8_t)geoIpm[tpmMask];
        }
        else
        {
          MotionInfo tempMi = mb.at(x, y);
          if (tempMi.interDir != 3)
          {
            CHECK( tempMi.interDir != 1 && tempMi.interDir != 2, "Uncorrect interDir" );
            int list = tempMi.interDir-1;
            Mv cMv = tempMi.mv[list];
            int refIdx = tempMi.refIdx[list];

            cMv.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
            Position PosY;
            PosY.x = pu.Y().x + (x << MIN_CU_LOG2) + cMv.getHor();
            PosY.y = pu.Y().y + (y << MIN_CU_LOG2) + cMv.getVer();
            clipColPos(PosY.x, PosY.y, pu);
#if JVET_Z0067_RPR_ENABLE
            scalePositionInRef(pu, *pu.cs->pps, RefPicList(list), refIdx, PosY);
            PosY.x = (PosY.x & mask);
            PosY.y = (PosY.y & mask);
            ipm = pu.cu->slice->getRefPic(RefPicList(list), refIdx)->unscaledPic->cs->getIpmInfo(PosY);
#else
            PosY.x = (PosY.x & mask);
            PosY.y = (PosY.y & mask);
            ipm = pu.cu->slice->getRefPic(RefPicList(list), refIdx)->cs->getIpmInfo(PosY);
#endif
          }
          else
          {
#if JVET_Z0067_RPR_ENABLE
            const Picture* pRefPic0 = pu.cu->slice->getRefPic(REF_PIC_LIST_0, tempMi.refIdx[0])->unscaledPic;
#else
            Picture* pRefPic0 = pu.cu->slice->getRefPic(REF_PIC_LIST_0, tempMi.refIdx[0]);
#endif
            Mv cMv0 = tempMi.mv[0];
            cMv0.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
            Position PosY0;
            PosY0.x = pu.Y().x + (x << MIN_CU_LOG2) + cMv0.getHor();
            PosY0.y = pu.Y().y + (y << MIN_CU_LOG2) + cMv0.getVer();
            clipColPos(PosY0.x, PosY0.y, pu);
#if JVET_Z0067_RPR_ENABLE
            scalePositionInRef(pu, *pu.cs->pps, REF_PIC_LIST_0, tempMi.refIdx[0], PosY0);
#endif
            PosY0.x = (PosY0.x & mask);
            PosY0.y = (PosY0.y & mask);
            MotionInfo mi0 = pRefPic0->cs->getMotionInfo(PosY0);
            int ipm0 = pRefPic0->cs->getIpmInfo(PosY0);
            int pocDiff0 = abs(pRefPic0->getPOC() - pu.cu->slice->getPOC());

#if JVET_Z0067_RPR_ENABLE
            const Picture* pRefPic1 = pu.cu->slice->getRefPic(REF_PIC_LIST_1, tempMi.refIdx[1])->unscaledPic;
#else
            Picture* pRefPic1 = pu.cu->slice->getRefPic(REF_PIC_LIST_1, tempMi.refIdx[1]);
#endif
            Mv cMv1 = tempMi.mv[1];
            cMv1.changePrecision(MV_PRECISION_SIXTEENTH, MV_PRECISION_INT);
            Position PosY1;
            PosY1.x = pu.Y().x + (x << MIN_CU_LOG2) + cMv1.getHor();
            PosY1.y = pu.Y().y + (y << MIN_CU_LOG2) + cMv1.getVer();
            clipColPos(PosY1.x, PosY1.y, pu);
#if JVET_Z0067_RPR_ENABLE
            scalePositionInRef(pu, *pu.cs->pps, REF_PIC_LIST_1, tempMi.refIdx[1], PosY1);
#endif
            PosY1.x = (PosY1.x & mask);
            PosY1.y = (PosY1.y & mask);
            MotionInfo mi1 = pRefPic1->cs->getMotionInfo(PosY1);
            int ipm1 = pRefPic1->cs->getIpmInfo(PosY1);
            int pocDiff1 = abs(pRefPic1->getPOC() - pu.cu->slice->getPOC());

            if (!mi0.isInter && mi1.isInter)
            {
              ipm = ipm0;
            }
            else if (!mi1.isInter && mi0.isInter)
            {
              ipm = ipm1;
            }
            else if (ipm0 > DC_IDX && ipm1 <= DC_IDX)
            {
              ipm = ipm0;
            }
            else if (ipm0 <= DC_IDX && ipm1 > DC_IDX)
            {
              ipm = ipm1;
            }
            else if (pocDiff0 < pocDiff1)
            {
              ipm = ipm0;
            }
            else if (pocDiff1 < pocDiff0)
            {
              ipm = ipm1;
            }
            else if (pRefPic0->m_prevQP[0] > pRefPic1->m_prevQP[0])
            {
              ipm = ipm1;
            }
            else
            {
              ipm = ipm0;
            }
          }
        }
        ii[x] = ipm;
      }
      ii += ib.stride;
    }
  }
  else
#endif
  spanIpmInfoInter( pu, mb, ib );
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
  spanSCCInfo(pu);
#endif
}

#if JVET_AC0112_IBC_CIIP
void PU::spanIpmInfoIBC( PredictionUnit &pu, IpmBuf &ib, int bvx, int bvy )
{
  uint8_t* ii = ib.buf;
  int ibH = ib.height;
  int ibW = ib.width;
  const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
  const unsigned mask = ~(scale - 1);
  Position PosY;
  PosY.x = pu.Y().x + bvx;
  PosY.y = pu.Y().y + bvy;
  PosY.x = (PosY.x & mask);
  PosY.y = (PosY.y & mask);
  const PredictionUnit* puTarget = pu.cs->getPURestricted(PosY, pu, pu.chType);
  const uint8_t ipm = puTarget ? puTarget->getIpmInfo(PosY) : 0;
  for (int y = 0; y < ibH; y++)
  {
    for (int x = 0; x < ibW; x++)
    {
      ii[x] = ipm;
    }
    ii += ib.stride;
  }
}
#endif
#endif

#if JVET_Z0054_BLK_REF_PIC_REORDER
bool PU::useRefCombList(const PredictionUnit &pu)
{
  if (!pu.cs->sps->getUseARL())
  {
    return false;
  }
  if (CU::isIBC(*pu.cu))
  {
    return false;
  }
#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (pu.amvpSbTmvpFlag)
  {
    return false;
  }
#endif
  if (!pu.mergeFlag && (pu.interDir == 1 || pu.interDir == 2))
  {
    return true;
  }
#if JVET_X0083_BM_AMVP_MERGE_MODE
  if (pu.amvpMergeModeFlag[REF_PIC_LIST_0] || pu.amvpMergeModeFlag[REF_PIC_LIST_1])
  {
    return true;
  }
#endif
  return false;
}

bool PU::useRefPairList(const PredictionUnit &pu)
{
  if (!pu.cs->sps->getUseARL())
  {
    return false;
  }
  if (CU::isIBC(*pu.cu))
  {
    return false;
  }
  if (pu.interDir != 3 || pu.mergeFlag)
  {
    return false;
  }
  if (pu.cu->smvdMode)
  {
    return false;
  }
#if JVET_X0083_BM_AMVP_MERGE_MODE
  if (pu.amvpMergeModeFlag[REF_PIC_LIST_0] || pu.amvpMergeModeFlag[REF_PIC_LIST_1])
  {
    return false;
  }
#endif
  return true;
}
#endif

#if JVET_AF0073_INTER_CCP_MERGE
void CU::saveProCcpInfo(CodingUnit &cu)
{
  if (cu.chromaFormat == CHROMA_400 || (CS::isDualITree(*cu.cs) && cu.chType == CHANNEL_TYPE_LUMA))
  {
    return;
  }
  
  if (CU::isInter(cu)) // Inter - MV propagation
  {
    for (auto& tu : CU::traverseTUs(cu))
    {
      if (tu.curCand.type != CCP_TYPE_NONE) // if current cand is CCP coded (CCRM and interCCM)
      {
        continue;
      }
      else
      {
#if JVET_Y0065_GPM_INTRA
        if (tu.getMotionInfo().isInter)
        {
#endif
          CU::saveProCcpInfoInter(cu, tu);
#if JVET_Y0065_GPM_INTRA
        }
#endif
      }
    } // for each tu in the cu
  }
}

void CU::saveProCcpInfoInter(CodingUnit &cu, TransformUnit &tu)
{
  const unsigned scale = 4 * std::max<int>(1, 4 * AMVP_DECIMATION_FACTOR / 4);
  const unsigned mask = ~(scale - 1);

  PredictionUnit &pu = *cu.firstPU;
  CodingStructure      &cs = *cu.cs;

  int lumaScaleX = getChannelTypeScaleX( CHANNEL_TYPE_CHROMA, cs.sps->getChromaFormatIdc() );
  int lumaScaleY = getChannelTypeScaleY( CHANNEL_TYPE_CHROMA, cs.sps->getChromaFormatIdc() );

  CCPModelCandidate proModel;
  MotionInfo tempMi = tu.getMotionInfo();

  if (tempMi.interDir <= 0)
  {
    return;
  }
  else if (tempMi.interDir != 3) // uni-prediction
  {
    Mv cMv;
    RefPicList refList;
    int refIdx;
    Position posY;
    Position posC;

    if (tempMi.interDir != 2)
    {
      CHECK(tempMi.interDir != 1, "motion information problem");
      cMv = tempMi.mv[0];
      refList = REF_PIC_LIST_0;
      refIdx = tempMi.refIdx[0];
    }
    else
    {
      cMv = tempMi.mv[1];
      refList = REF_PIC_LIST_1;
      refIdx = tempMi.refIdx[1];
    }
    if (refList < 0 || refIdx < 0)
    {
      return;
    }
    else
    {
      cMv.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
      posY.x = tu.Y().x + cMv.getHor();
      posY.y = tu.Y().y + cMv.getVer();
#if JVET_AG0098_AMVP_WITH_SBTMVP
      PU::clipColPos(posY.x, posY.y, pu);
#else
      clipColPos(posY.x, posY.y, pu);
#endif
#if RPR_ENABLE
      scalePositionInRef( pu, *pu.cs->pps, refList, refIdx, posY );
#endif
      posY.x = (posY.x & mask);
      posY.y = (posY.y & mask);
      posC.x = posY.x >> lumaScaleX;
      posC.y = posY.y >> lumaScaleY;
      int idx = pu.cu->slice->getRefPic(refList, refIdx)->unscaledPic->cs->getCcpmIdxInfo(posC);
      if (idx > 0)
      {
        proModel = pu.cu->slice->getRefPic(refList, refIdx)->unscaledPic->cs->m_ccpModelLUT[idx-1];
        CHECK(proModel.type == 0, "invalid model");
        tu.curCand = proModel;
      }
      else
      {
        return;
      }
    }
  }
  else // bi-prediction, tempMi.interDir == 3
  {
    MotionInfo mi0;
    MotionInfo mi1;
    Position posY0;
    Position posC0;
    Position posY1;
    Position posC1;
    Mv cMv0;
    Mv cMv1;
    const Picture* pRefPic0;
    const Picture* pRefPic1;

    // L0 motion vector
    pRefPic0 = pu.cu->slice->getRefPic(REF_PIC_LIST_0, tempMi.refIdx[0])->unscaledPic;
    cMv0 = tempMi.mv[0];
    cMv0.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
    posY0.x = tu.Y().x + cMv0.getHor();
    posY0.y = tu.Y().y + cMv0.getVer();
#if JVET_AG0098_AMVP_WITH_SBTMVP
    PU::clipColPos(posY0.x, posY0.y, pu);
#else
    clipColPos(posY0.x, posY0.y, pu);
#endif
#if RPR_ENABLE
    scalePositionInRef( pu, *pu.cs->pps, REF_PIC_LIST_0, tempMi.refIdx[0], posY0 );
#endif
    posY0.x = (posY0.x & mask);
    posY0.y = (posY0.y & mask);
    posC0.x = posY0.x >> lumaScaleX;
    posC0.y = posY0.y >> lumaScaleY;
    int idx0 = pRefPic0->cs->getCcpmIdxInfo(posC0);
    CCPModelCandidate proModel0;
    if (idx0 > 0)
    {
      proModel0 = pRefPic0->cs->m_ccpModelLUT[idx0 - 1];
      CHECK(proModel0.type == 0, "invalid model");
    }
    int pocDiff0 = abs(pRefPic0->getPOC() - pu.cu->slice->getPOC());

    // L1 motion vector
    pRefPic1 = pu.cu->slice->getRefPic(REF_PIC_LIST_1, tempMi.refIdx[1])->unscaledPic;
    cMv1 = tempMi.mv[1];
    cMv1.changePrecision(MV_PRECISION_INTERNAL, MV_PRECISION_INT);
    posY1.x = tu.Y().x + cMv1.getHor();
    posY1.y = tu.Y().y + cMv1.getVer();
#if JVET_AG0098_AMVP_WITH_SBTMVP
    PU::clipColPos(posY1.x, posY1.y, pu);
#else
    clipColPos(posY1.x, posY1.y, pu);
#endif
#if RPR_ENABLE
    scalePositionInRef( pu, *pu.cs->pps, REF_PIC_LIST_1, tempMi.refIdx[1], posY1 );
#endif
    posY1.x = (posY1.x & mask);
    posY1.y = (posY1.y & mask);
    posC1.x = posY1.x >> lumaScaleX;
    posC1.y = posY1.y >> lumaScaleY;
    int idx1 = pRefPic1->cs->getCcpmIdxInfo(posC1);
    CCPModelCandidate proModel1;
    if (idx1 > 0)
    {
      proModel1 = pRefPic1->cs->m_ccpModelLUT[idx1 - 1];
      CHECK(proModel1.type == 0, "invalid model");
    }
    int pocDiff1 = abs(pRefPic1->getPOC() - pu.cu->slice->getPOC());

    // Select between L0 and L1
    if ((idx0 == 0) && (idx1 == 0)) // no model available
    {
      return;
    }
    else 
    {
      if ((idx0 > 0) && (idx1 == 0))
      {
        proModel = proModel0;
      }
      else if ((idx0 == 0) && (idx1 > 0))
      {
        proModel = proModel1;
      }
      else if (pocDiff0 < pocDiff1)
      {
        proModel = proModel0;
      }
      else if (pocDiff1 < pocDiff0)
      {
        proModel = proModel1;
      }
      else if (pRefPic0->m_prevQP[0] > pRefPic1->m_prevQP[0])
      {
        proModel = proModel1;
      }
      else
      {
        proModel = proModel0;
      }
      CHECK(proModel.type == 0, "invalid model");
      tu.curCand = proModel;
    }
  } // bi-prediction
}
#endif

bool CU::hasSubCUNonZeroMVd( const CodingUnit& cu )
{
#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (cu.firstPU->amvpSbTmvpFlag)
  {
    return (cu.firstPU->amvpSbTmvpMvdIdx == -1) ? false : true;
  }
#endif
  bool bNonZeroMvd = false;

  for( const auto &pu : CU::traversePUs( cu ) )
  {
    if( ( !pu.mergeFlag ) && ( !cu.skip ) )
    {
      if( pu.interDir != 2 /* PRED_L1 */ )
      {
        bNonZeroMvd |= pu.mvd[REF_PIC_LIST_0].getHor() != 0;
        bNonZeroMvd |= pu.mvd[REF_PIC_LIST_0].getVer() != 0;
      }
      if( pu.interDir != 1 /* PRED_L0 */ )
      {
        if( !pu.cu->cs->picHeader->getMvdL1ZeroFlag() || pu.interDir != 3 /* PRED_BI */ )
        {
          bNonZeroMvd |= pu.mvd[REF_PIC_LIST_1].getHor() != 0;
          bNonZeroMvd |= pu.mvd[REF_PIC_LIST_1].getVer() != 0;
        }
      }
    }
  }

  return bNonZeroMvd;
}

bool CU::hasSubCUNonZeroAffineMVd( const CodingUnit& cu )
{
  bool nonZeroAffineMvd = false;

  if ( !cu.affine || cu.firstPU->mergeFlag )
  {
    return false;
  }

  for ( const auto &pu : CU::traversePUs( cu ) )
  {
    if ( ( !pu.mergeFlag ) && ( !cu.skip ) )
    {
      if ( pu.interDir != 2 /* PRED_L1 */ )
      {
        for ( int i = 0; i < ( cu.affineType == AFFINEMODEL_6PARAM ? 3 : 2 ); i++ )
        {
          nonZeroAffineMvd |= pu.mvdAffi[REF_PIC_LIST_0][i].getHor() != 0;
          nonZeroAffineMvd |= pu.mvdAffi[REF_PIC_LIST_0][i].getVer() != 0;
        }
      }

      if ( pu.interDir != 1 /* PRED_L0 */ )
      {
        if ( !pu.cu->cs->picHeader->getMvdL1ZeroFlag() || pu.interDir != 3 /* PRED_BI */ )
        {
          for ( int i = 0; i < ( cu.affineType == AFFINEMODEL_6PARAM ? 3 : 2 ); i++ )
          {
            nonZeroAffineMvd |= pu.mvdAffi[REF_PIC_LIST_1][i].getHor() != 0;
            nonZeroAffineMvd |= pu.mvdAffi[REF_PIC_LIST_1][i].getVer() != 0;
          }
        }
      }
    }
  }

  return nonZeroAffineMvd;
}

uint8_t CU::getSbtInfo( uint8_t idx, uint8_t pos )
{
  return ( pos << 4 ) + ( idx << 0 );
}

uint8_t CU::getSbtIdx( const uint8_t sbtInfo )
{
  return ( sbtInfo >> 0 ) & 0xf;
}

uint8_t CU::getSbtPos( const uint8_t sbtInfo )
{
  return ( sbtInfo >> 4 ) & 0x3;
}

uint8_t CU::getSbtMode( uint8_t sbtIdx, uint8_t sbtPos )
{
  uint8_t sbtMode = 0;
  switch( sbtIdx )
  {
  case SBT_VER_HALF: sbtMode = sbtPos + SBT_VER_H0;  break;
  case SBT_HOR_HALF: sbtMode = sbtPos + SBT_HOR_H0;  break;
  case SBT_VER_QUAD: sbtMode = sbtPos + SBT_VER_Q0;  break;
  case SBT_HOR_QUAD: sbtMode = sbtPos + SBT_HOR_Q0;  break;
  default:           assert( 0 );
  }

  assert( sbtMode < NUMBER_SBT_MODE );
  return sbtMode;
}

uint8_t CU::getSbtIdxFromSbtMode( uint8_t sbtMode )
{
  if( sbtMode <= SBT_VER_H1 )
  {
    return SBT_VER_HALF;
  }
  else if( sbtMode <= SBT_HOR_H1 )
  {
    return SBT_HOR_HALF;
  }
  else if( sbtMode <= SBT_VER_Q1 )
  {
    return SBT_VER_QUAD;
  }
  else if( sbtMode <= SBT_HOR_Q1 )
  {
    return SBT_HOR_QUAD;
  }
  else
  {
    assert( 0 );
    return 0;
  }
}

uint8_t CU::getSbtPosFromSbtMode( uint8_t sbtMode )
{
  if( sbtMode <= SBT_VER_H1 )
  {
    return sbtMode - SBT_VER_H0;
  }
  else if( sbtMode <= SBT_HOR_H1 )
  {
    return sbtMode - SBT_HOR_H0;
  }
  else if( sbtMode <= SBT_VER_Q1 )
  {
    return sbtMode - SBT_VER_Q0;
  }
  else if( sbtMode <= SBT_HOR_Q1 )
  {
    return sbtMode - SBT_HOR_Q0;
  }
  else
  {
    assert( 0 );
    return 0;
  }
}

uint8_t CU::targetSbtAllowed( uint8_t sbtIdx, uint8_t sbtAllowed )
{
  uint8_t val = 0;
  switch( sbtIdx )
  {
  case SBT_VER_HALF: val = ( ( sbtAllowed >> SBT_VER_HALF ) & 0x1 ); break;
  case SBT_HOR_HALF: val = ( ( sbtAllowed >> SBT_HOR_HALF ) & 0x1 ); break;
  case SBT_VER_QUAD: val = ( ( sbtAllowed >> SBT_VER_QUAD ) & 0x1 ); break;
  case SBT_HOR_QUAD: val = ( ( sbtAllowed >> SBT_HOR_QUAD ) & 0x1 ); break;
  default:           CHECK( 1, "unknown SBT type" );
  }
  return val;
}

uint8_t CU::numSbtModeRdo( uint8_t sbtAllowed )
{
  uint8_t num = 0;
  uint8_t sum = 0;
  num = targetSbtAllowed( SBT_VER_HALF, sbtAllowed ) + targetSbtAllowed( SBT_HOR_HALF, sbtAllowed );
  sum += std::min( SBT_NUM_RDO, ( num << 1 ) );
  num = targetSbtAllowed( SBT_VER_QUAD, sbtAllowed ) + targetSbtAllowed( SBT_HOR_QUAD, sbtAllowed );
  sum += std::min( SBT_NUM_RDO, ( num << 1 ) );
  return sum;
}

bool CU::isSbtMode( const uint8_t sbtInfo )
{
  uint8_t sbtIdx = getSbtIdx( sbtInfo );
  return sbtIdx >= SBT_VER_HALF && sbtIdx <= SBT_HOR_QUAD;
}

bool CU::isSameSbtSize( const uint8_t sbtInfo1, const uint8_t sbtInfo2 )
{
  uint8_t sbtIdx1 = getSbtIdxFromSbtMode( sbtInfo1 );
  uint8_t sbtIdx2 = getSbtIdxFromSbtMode( sbtInfo2 );
  if( sbtIdx1 == SBT_HOR_HALF || sbtIdx1 == SBT_VER_HALF )
  {
    return sbtIdx2 == SBT_HOR_HALF || sbtIdx2 == SBT_VER_HALF;
  }
  else if( sbtIdx1 == SBT_HOR_QUAD || sbtIdx1 == SBT_VER_QUAD )
  {
    return sbtIdx2 == SBT_HOR_QUAD || sbtIdx2 == SBT_VER_QUAD;
  }
  else
  {
    return false;
  }
}

bool CU::isPredRegDiffFromTB(const CodingUnit &cu, const ComponentID compID)
{
  return (compID == COMPONENT_Y)
         && (cu.ispMode == VER_INTRA_SUBPARTITIONS
             && CU::isMinWidthPredEnabledForBlkSize(cu.blocks[compID].width, cu.blocks[compID].height));
}

bool CU::isMinWidthPredEnabledForBlkSize(const int w, const int h)
{
  return ((w == 8 && h > 4) || w == 4);
}

bool CU::isFirstTBInPredReg(const CodingUnit& cu, const ComponentID compID, const CompArea &area)
{
  return (compID == COMPONENT_Y) && cu.ispMode && ((area.topLeft().x - cu.Y().topLeft().x) % PRED_REG_MIN_WIDTH == 0);
}

void CU::adjustPredArea(CompArea &area)
{
  area.width = std::max<int>(PRED_REG_MIN_WIDTH, area.width);
}

#if MULTI_HYP_PRED
AMVPInfo PU::getMultiHypMVPCandsMerge(PredictionUnit &pu, const RefPicList eRefPicList, const int refIdx)
{
  CHECK(!pu.mergeFlag, "this function is only for merge");

  AMVPInfo amvpInfo;
  PU::fillMvpCand(pu, eRefPicList, refIdx, amvpInfo);

  return amvpInfo;
}

AMVPInfo PU::getMultiHypMVPCandsAMVP(PredictionUnit &pu, const RefPicList eRefPicList, const int refIdx)
{
  CHECK(pu.mergeFlag, "this function is only for AMVP");
  CHECK(pu.interDir != 3, "Multi-Hyp is only allowed for bi-predictive AMVP mode")

  const int refListForAMVP = eRefPicList;
  const int refIdxForAMVP = refIdx;

  AMVPInfo amvpInfo;
  if (pu.cu->affine) // TODO: check performance impact
  {
    AffineAMVPInfo affineAMVPInfo;
    PU::fillAffineMvpCand(pu, RefPicList(refListForAMVP), refIdxForAMVP, affineAMVPInfo);
    amvpInfo.numCand = affineAMVPInfo.numCand;
    memcpy(amvpInfo.mvCand, affineAMVPInfo.mvCandLT, sizeof(amvpInfo.mvCand));
  }
  else
  {
    PU::fillMvpCand(pu, RefPicList(refListForAMVP), refIdxForAMVP, amvpInfo);
  }

  return amvpInfo;
}

AMVPInfo PU::getMultiHypMVPCands(PredictionUnit &pu, const MultiHypPredictionData &mhData)
{
  CHECK(mhData.isMrg, "mhData is merge mode");
  const auto mhRefIdxForAMVPList = mhData.refIdx;
  const auto &MHRefPics = pu.cs->slice->getMultiHypRefPicList();
  CHECK(MHRefPics.empty(), "Multi Hyp: MHRefPics.empty()");
  const auto eRefPicList = RefPicList(MHRefPics[mhRefIdxForAMVPList].refList);
  const int iRefIdx = MHRefPics[mhRefIdxForAMVPList].refIdx;

  return (pu.mergeFlag ? PU::getMultiHypMVPCandsMerge : PU::getMultiHypMVPCandsAMVP)(pu, eRefPicList, iRefIdx);
}

Mv PU::getMultiHypMVP(PredictionUnit &pu, const MultiHypPredictionData &mhData)
{
  const auto amvpInfo = getMultiHypMVPCands(pu, mhData);
  CHECK(mhData.mvpIdx < 0, "Multi Hyp: mhData.mvpIdx < 0");
  CHECK(mhData.mvpIdx >= amvpInfo.numCand, "Multi Hyp: mhData.mvpIdx >= amvpInfo.numCand");
  const Mv mvp = amvpInfo.mvCand[mhData.mvpIdx];
  return mvp;
}
#endif


bool CU::isBcwIdxCoded( const CodingUnit &cu )
{
  if( cu.cs->sps->getUseBcw() == false )
  {
    CHECK(cu.bcwIdx != BCW_DEFAULT, "Error: cu.bcwIdx != BCW_DEFAULT");
    return false;
  }
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
  if (cu.skip)
  {
    return false;
  }
#endif
  if (cu.predMode == MODE_IBC)
  {
    return false;
  }

  if( cu.predMode == MODE_INTRA || cu.cs->slice->isInterP() )
  {
    return false;
  }

  if( cu.lwidth() * cu.lheight() < BCW_SIZE_CONSTRAINT )
  {
    return false;
  }

#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (cu.firstPU->amvpSbTmvpFlag)
  {
    return false;
  }
#endif

  if( !cu.firstPU->mergeFlag )
  {
    if( cu.firstPU->interDir == 3 )
    {
#if !JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
      const int refIdx0 = cu.firstPU->refIdx[REF_PIC_LIST_0];
      const int refIdx1 = cu.firstPU->refIdx[REF_PIC_LIST_1];
      
      const WPScalingParam *wp0 = cu.cs->slice->getWpScaling(REF_PIC_LIST_0, refIdx0);
      const WPScalingParam *wp1 = cu.cs->slice->getWpScaling(REF_PIC_LIST_1, refIdx1);
      
      return !(WPScalingParam::isWeighted(wp0) || WPScalingParam::isWeighted(wp1));
#else
      return true;
#endif
    }
  }

  return false;
}

uint8_t CU::getValidBcwIdx( const CodingUnit &cu )
{
  if( cu.firstPU->interDir == 3 && !cu.firstPU->mergeFlag )
  {
    return cu.bcwIdx;
  }
  else if( cu.firstPU->interDir == 3 && cu.firstPU->mergeFlag && cu.firstPU->mergeType == MRG_TYPE_DEFAULT_N )
  {
    // This is intended to do nothing here.
  }
  else if( cu.firstPU->mergeFlag && cu.firstPU->mergeType == MRG_TYPE_SUBPU_ATMVP )
  {
    CHECK(cu.bcwIdx != BCW_DEFAULT, " cu.bcwIdx != BCW_DEFAULT ");
  }
  else
  {
    CHECK(cu.bcwIdx != BCW_DEFAULT, " cu.bcwIdx != BCW_DEFAULT ");
  }

  return BCW_DEFAULT;
}

void CU::setBcwIdx( CodingUnit &cu, uint8_t uh )
{
  int8_t uhCnt = 0;

  if( cu.firstPU->interDir == 3 && !cu.firstPU->mergeFlag )
  {
    cu.bcwIdx = uh;
    ++uhCnt;
  }
  else if( cu.firstPU->interDir == 3 && cu.firstPU->mergeFlag && cu.firstPU->mergeType == MRG_TYPE_DEFAULT_N )
  {
    // This is intended to do nothing here.
  }
  else if( cu.firstPU->mergeFlag && cu.firstPU->mergeType == MRG_TYPE_SUBPU_ATMVP )
  {
    cu.bcwIdx = BCW_DEFAULT;
  }
  else
  {
    cu.bcwIdx = BCW_DEFAULT;
  }

  CHECK(uhCnt <= 0, " uhCnt <= 0 ");
}

uint8_t CU::deriveBcwIdx( uint8_t bcwLO, uint8_t bcwL1 )
{
  if( bcwLO == bcwL1 )
  {
    return bcwLO;
  }
  const int8_t w0 = getBcwWeight(bcwLO, REF_PIC_LIST_0);
  const int8_t w1 = getBcwWeight(bcwL1, REF_PIC_LIST_1);
  const int8_t th = g_bcwWeightBase >> 1;
  const int8_t off = 1;

  if( w0 == w1 || (w0 < (th - off) && w1 < (th - off)) || (w0 >(th + off) && w1 >(th + off)) )
  {
    return BCW_DEFAULT;
  }
  else
  {
    if( w0 > w1 )
    {
      return ( w0 >= th ? bcwLO : bcwL1 );
    }
    else
    {
      return ( w1 >= th ? bcwL1 : bcwLO );
    }
  }
}

bool CU::bdpcmAllowed( const CodingUnit& cu, const ComponentID compID )
{
  SizeType transformSkipMaxSize = 1 << cu.cs->sps->getLog2MaxTransformSkipBlockSize();

  bool bdpcmAllowed = cu.cs->sps->getBDPCMEnabledFlag();
       bdpcmAllowed &= CU::isIntra( cu );
       if (isLuma(compID))
       {
         bdpcmAllowed &= (cu.lwidth() <= transformSkipMaxSize && cu.lheight() <= transformSkipMaxSize);
#if ENABLE_DIMD
         bdpcmAllowed &= !cu.dimd;
#endif
       }
       else
       {
         bdpcmAllowed &=
           (cu.chromaSize().width <= transformSkipMaxSize && cu.chromaSize().height <= transformSkipMaxSize)
           && !cu.colorTransform;
       }
  return bdpcmAllowed;
}
#if JVET_AG0061_INTER_LFNST_NSPT
bool CU::isLfnstAllowed(const CodingUnit &cu, const ComponentID compID)
{
  bool lfnstAllowed = cu.chType == CHANNEL_TYPE_LUMA && compID == COMPONENT_Y;
  return lfnstAllowed;
}
#endif
bool CU::isMTSAllowed(const CodingUnit &cu, const ComponentID compID)
{
  SizeType tsMaxSize = 1 << cu.cs->sps->getLog2MaxTransformSkipBlockSize();
#if JVET_AA0133_INTER_MTS_OPT
  const int maxSize = CU::isIntra(cu) ? MTS_INTRA_MAX_CU_SIZE : cu.cs->sps->getInterMTSMaxSize();
#else
  const int maxSize  = CU::isIntra( cu ) ? MTS_INTRA_MAX_CU_SIZE : MTS_INTER_MAX_CU_SIZE;
#endif
  const int cuWidth  = cu.blocks[0].lumaSize().width;
  const int cuHeight = cu.blocks[0].lumaSize().height;
  bool mtsAllowed    = cu.chType == CHANNEL_TYPE_LUMA && compID == COMPONENT_Y;

  mtsAllowed &= CU::isIntra( cu ) ? cu.cs->sps->getUseIntraMTS() : cu.cs->sps->getUseInterMTS() && CU::isInter( cu );
  mtsAllowed &= cuWidth <= maxSize && cuHeight <= maxSize;
  mtsAllowed &= !cu.ispMode;
  mtsAllowed &= !cu.sbtInfo;
#if JVET_V0130_INTRA_TMP
#if !JVET_AC0115_INTRA_TMP_DIMD_MTS_LFNST
  mtsAllowed &= !cu.tmpFlag;
#endif
#endif
#if JVET_W0123_TIMD_FUSION
  mtsAllowed &= !(cu.timd && cu.firstPU->multiRefIdx);
#endif
  mtsAllowed &= !(cu.bdpcmMode && cuWidth <= tsMaxSize && cuHeight <= tsMaxSize);

  return mtsAllowed;
}

#if ENABLE_OBMC
unsigned int PU::getSameNeigMotion(PredictionUnit &pu, MotionInfo& mi, Position off, int iDir, int& iLength, int iMaxLength)
{

  PredictionUnit* tmpPu = nullptr;
#if JVET_AD0213_LIC_IMP
  PredictionUnit* tmpPu1 = nullptr;
#endif
  Position posNeighborMotion = Position(0, 0);
  const Position  posSubBlock(pu.lumaPos().offset(off));

  const unsigned int  uiMinCUW = pu.cs->pcv->minCUWidth;
  if (iDir == 0)
  {
    posNeighborMotion = posSubBlock.offset(0, -1);
  }
  else if (iDir == 1)
  {
    posNeighborMotion = posSubBlock.offset(-1, 0);
  }
  else
  {
    THROW("iDir not supported");
  }
  tmpPu = pu.cs->getPU(posNeighborMotion, pu.chType);

  //if neighbor pu is not available, return.
  if (!tmpPu)
  {
    return 0;
  }

  mi = tmpPu->getMotionInfo(posNeighborMotion);

  iLength = 1;
  if (mi.isInter && !mi.isIBCmot) // inter
  {
    MotionInfo currMi = pu.getMotionInfo(posSubBlock);
    // check similar MV

    bool bIsSimilarMV = PU::identicalMvOBMC(currMi, mi, pu.cs->slice->getCheckLDC());
#if JVET_AD0213_LIC_IMP
#if JVET_AF0190_RPR_TMP_REORDER_LIC
    if (bIsSimilarMV)
#else
    if (bIsSimilarMV && !tmpPu->cs->sps->getRprEnabledFlag())
#endif
    {
      if (mi.interDir == 3 && mi.bcwIdx != currMi.bcwIdx)
      {
        bIsSimilarMV = false;
      }
      if (bIsSimilarMV)
      {
        if (tmpPu->cu->licFlag != pu.cu->licFlag)
        {
          bIsSimilarMV = false;
        }
        else if (tmpPu->cu->licFlag)
        {
          for (int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
          {
#if JVET_AE0046_BI_GPM
            if (mi.refIdx[refList] >= 0)
#else
            if (tmpPu->refIdx[refList] >= 0)
#endif
            {
              if (tmpPu->cu->licScale[refList][COMPONENT_Y] != pu.cu->licScale[refList][COMPONENT_Y] || tmpPu->cu->licOffset[refList][COMPONENT_Y] != pu.cu->licOffset[refList][COMPONENT_Y])
              {
                bIsSimilarMV = false;
                break;
              }
            }
          }
        }
      }
    }
#endif
#if JVET_AG0276_NLIC
    if (bIsSimilarMV)
    {
      if (tmpPu->cu->altLMFlag != pu.cu->altLMFlag)
      {
        bIsSimilarMV = false;
      }
      else if (tmpPu->cu->altLMFlag)
      {
        if (tmpPu->cu->secAltLMParaUnit.scale[0] != pu.cu->altLMParaUnit.scale[0] || tmpPu->cu->secAltLMParaUnit.offset[0] != pu.cu->altLMParaUnit.offset[0])
        {
          bIsSimilarMV = false;
        }
      }
    }
#endif
    for (unsigned int idx = 1; idx < iMaxLength; idx++)
    {

      if (iDir == 0)//top
      {
        posNeighborMotion = posSubBlock.offset(idx*uiMinCUW, -1);
      }
      else//left
      {
        posNeighborMotion = posSubBlock.offset(-1, idx*uiMinCUW);
      }
#if JVET_AD0213_LIC_IMP
      tmpPu1 = pu.cs->getPU(posNeighborMotion, pu.chType);
      MotionInfo miNeigh = tmpPu1->getMotionInfo(posNeighborMotion);
#else
      tmpPu = pu.cs->getPU(posNeighborMotion, pu.chType);

      MotionInfo miNeigh = tmpPu->getMotionInfo(posNeighborMotion);
#endif

      bool       bSameMv = true;

#if JVET_AD0213_LIC_IMP
      if (mi.interDir != miNeigh.interDir || miNeigh.isIBCmot || !miNeigh.isInter)
#else
      if (mi.interDir != miNeigh.interDir || miNeigh.isIBCmot)
#endif
      {
        bSameMv = false;
      }
      else
      {
        for (unsigned int iRefList = 0; iRefList < 2; iRefList++)
        {
          if (miNeigh.interDir & (1 << iRefList))
          {
            if (!(miNeigh.mv[iRefList] == mi.mv[iRefList] && miNeigh.refIdx[iRefList] == mi.refIdx[iRefList]))
            {
              bSameMv = false; break;
            }
          }
        }
#if JVET_AD0213_LIC_IMP
        if (bSameMv)
        {
          if (mi.interDir == 3 && miNeigh.bcwIdx != mi.bcwIdx)
          {
            bSameMv = false;
          }
#if JVET_AF0190_RPR_TMP_REORDER_LIC
          if (bSameMv)
#else
          if (bSameMv && !tmpPu->cs->sps->getRprEnabledFlag())
#endif
          {
            if (tmpPu1->cu->licFlag != tmpPu->cu->licFlag)
            {
              bSameMv = false;
            }
            else if (tmpPu1->cu->licFlag)
            {
              for (int refList = 0; refList < NUM_REF_PIC_LIST_01; refList++)
              {
#if JVET_AE0046_BI_GPM
                if (miNeigh.refIdx[refList] >= 0)
#else
                if (tmpPu1->refIdx[refList] >= 0)
#endif
                {
                  if (tmpPu1->cu->licScale[refList][COMPONENT_Y] != tmpPu->cu->licScale[refList][COMPONENT_Y] || tmpPu1->cu->licOffset[refList][COMPONENT_Y] != tmpPu->cu->licOffset[refList][COMPONENT_Y])
                  {
                    bSameMv = false;
                    break;
                  }
                }
              }
            }
          }
        }
#endif
#if JVET_AG0276_NLIC
        if (bSameMv)
        {
          if (tmpPu1->cu->altLMFlag != tmpPu->cu->altLMFlag)
          {
            bSameMv = false;
          }
          else if (tmpPu1->cu->altLMFlag)
          {
            if (tmpPu1->cu->secAltLMParaUnit.scale[0] != tmpPu->cu->secAltLMParaUnit.scale[0] || tmpPu1->cu->secAltLMParaUnit.offset[0] != tmpPu->cu->secAltLMParaUnit.offset[0])
            {
              bSameMv = false;
            }
          }
        }
#endif
      }
#if JVET_AD0213_LIC_IMP
      if (bSameMv)
      {
        Position  posSubBlock1;
        if (iDir == 0)//top
        {
          posSubBlock1 = posSubBlock.offset(idx*uiMinCUW, 0);
        }
        else//left
        {
          posSubBlock1 = posSubBlock.offset(0, idx*uiMinCUW);
        }

        MotionInfo currMi1 = pu.getMotionInfo(posSubBlock1);
        CHECK(currMi1.isIBCmot || !currMi1.isInter, "currMi1.isIBCmot || !currMi1.isInter");
        if (currMi.interDir != currMi1.interDir)
        {
          bSameMv = false;
        }
        else
        {
          for (unsigned int iRefList = 0; iRefList < 2; iRefList++)
          {
            if (currMi1.interDir & (1 << iRefList))
            {
              if (!(currMi1.mv[iRefList] == currMi.mv[iRefList] && currMi1.refIdx[iRefList] == currMi.refIdx[iRefList]))
              {
                bSameMv = false;
                break;
              }
            }
          }
        }
      }
#endif 
     if (bSameMv)
      {
        iLength++;
      }
      else
      {
        break;
      }
    }
    return 2 + bIsSimilarMV; // 2: not skip; 3: skip
  }
  else             // intra
  {
    for (unsigned int idx = 1; idx < iMaxLength; idx++)
    {
      if (iDir == 0)
      {
        posNeighborMotion = posSubBlock.offset(idx*uiMinCUW, -1);
      }
      else
      {
        posNeighborMotion = posSubBlock.offset(-1, idx*uiMinCUW);
      }
      tmpPu = pu.cs->getPU(posNeighborMotion, pu.chType);

      if (!tmpPu->getMotionInfo(posNeighborMotion).isInter || tmpPu->getMotionInfo(posNeighborMotion).isIBCmot)
      {
        iLength++;
      }
      else
      {
        break;
      }
    }
    return 1;
  }
}

bool PU::identicalMvOBMC(MotionInfo curMI, MotionInfo neighMI, bool bLD)
{
  if (curMI.interDir != neighMI.interDir)
  {
    return false;
  }
  else
  {
    for (int i = 0; i < 2; i++)
    {
      if (curMI.refIdx[i] != -1)
      {
        if (!(curMI.mv[i] == neighMI.mv[i] && curMI.refIdx[i] == neighMI.refIdx[i]))
        {
          return false;
        }
      }
    }
    return true;
  }
}

bool PU::getNeighborMotion(PredictionUnit &pu, MotionInfo& mi, Position off, Size unitSize, int iDir)
{
  PredictionUnit* tmpPu = &pu;
  Position posNeighborMotion = Position(0, 0);
  const Position posSubBlock(pu.lumaPos().offset(off));


  if (iDir == 0) //above
  {
    posNeighborMotion = posSubBlock.offset(0, -1);
  }
  else if (iDir == 1) //left
  {
    posNeighborMotion = posSubBlock.offset(-1, 0);
  }
  else if (iDir == 2) //below
  {
    posNeighborMotion = posSubBlock.offset(0, unitSize.height);
    CHECK(pu.cu != tmpPu->cu, "Got a PU from a different CU when fetching a below PU");
  }
  else if (iDir == 3) //right
  {
    posNeighborMotion = posSubBlock.offset(unitSize.width, 0);
    CHECK(pu.cu != tmpPu->cu, "Got a PU from a different CU when fetching a right PU");
  }

  const bool bNoAdjacentMotion = !tmpPu || CU::isIntra(*tmpPu->cu);

  if (bNoAdjacentMotion)
  {
    return false;
  }

  mi = tmpPu->getMotionInfo(posNeighborMotion);
  const MotionInfo currMotion = pu.getMotionInfo(posSubBlock);

  if (mi.interDir)
  {
    if (mi.interDir != currMotion.interDir)
    {
      return true;
    }
    else
    {
      for (uint32_t iRefList = 0; iRefList < 2; iRefList++)
      {
        if (currMotion.interDir & (1 << iRefList))
        {
          Mv mvd = currMotion.mv[iRefList] - mi.mv[iRefList];

          if (!(mvd.getAbsHor() < 1 && mvd.getAbsVer() < 1 && currMotion.refIdx[iRefList] == mi.refIdx[iRefList]))
          {
            return true;
          }
        }
      }
      return false;
    }
  }
  else
  {
    return false;
  }
}
#endif

// TU tools

bool TU::isNonTransformedResidualRotated(const TransformUnit &tu, const ComponentID &compID)
{
  return tu.cs->sps->getSpsRangeExtension().getTransformSkipRotationEnabledFlag() && tu.blocks[compID].width == 4 && tu.cu->predMode == MODE_INTRA;
}

bool TU::getCbf( const TransformUnit &tu, const ComponentID &compID )
{
  return getCbfAtDepth( tu, compID, tu.depth );
}

bool TU::getCbfAtDepth(const TransformUnit &tu, const ComponentID &compID, const unsigned &depth)
{
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  if( !tu.blocks[compID].valid() )
  {
    CHECK(tu.cbf[compID] != 0, "cbf must be 0 if the component is not available");
  }
#endif
  return ((tu.cbf[compID] >> depth) & 1) == 1;
}

void TU::setCbfAtDepth(TransformUnit &tu, const ComponentID &compID, const unsigned &depth, const bool &cbf)
{
  // first clear the CBF at the depth
  tu.cbf[compID] &= ~(1  << depth);
  // then set the CBF
  tu.cbf[compID] |= ((cbf ? 1 : 0) << depth);
}

bool TU::isTSAllowed(const TransformUnit &tu, const ComponentID compID)
{
  const int maxSize = tu.cs->sps->getLog2MaxTransformSkipBlockSize();

  bool tsAllowed = tu.cs->sps->getTransformSkipEnabledFlag();
  tsAllowed &= ( !tu.cu->ispMode || !isLuma(compID) );
  SizeType transformSkipMaxSize = 1 << maxSize;
  tsAllowed &= !(tu.cu->bdpcmMode && isLuma(compID));
  tsAllowed &= !(tu.cu->bdpcmModeChroma && isChroma(compID));
  tsAllowed &= tu.blocks[compID].width <= transformSkipMaxSize && tu.blocks[compID].height <= transformSkipMaxSize;
  tsAllowed &= !tu.cu->sbtInfo;

  return tsAllowed;
}


int TU::getICTMode( const TransformUnit& tu, int jointCbCr )
{
  if( jointCbCr < 0 )
  {
    jointCbCr = tu.jointCbCr;
  }
  return g_ictModes[ tu.cs->picHeader->getJointCbCrSignFlag() ][ jointCbCr ];
}


bool TU::needsSqrt2Scale( const TransformUnit &tu, const ComponentID &compID )
{
  const Size &size=tu.blocks[compID];
  const bool isTransformSkip = (tu.mtsIdx[compID] == MTS_SKIP);
  return (!isTransformSkip) && (((floorLog2(size.width) + floorLog2(size.height)) & 1) == 1);
}

bool TU::needsBlockSizeTrafoScale( const TransformUnit &tu, const ComponentID &compID )
{
  return needsSqrt2Scale( tu, compID ) || isNonLog2BlockSize( tu.blocks[compID] );
}

TransformUnit* TU::getPrevTU( const TransformUnit &tu, const ComponentID compID )
{
  TransformUnit* prevTU = tu.prev;

  if( prevTU != nullptr && ( prevTU->cu != tu.cu || !prevTU->blocks[compID].valid() ) )
  {
    prevTU = nullptr;
  }

  return prevTU;
}

bool TU::getPrevTuCbfAtDepth( const TransformUnit &currentTu, const ComponentID compID, const int trDepth )
{
  const TransformUnit* prevTU = getPrevTU( currentTu, compID );
  return ( prevTU != nullptr ) ? TU::getCbfAtDepth( *prevTU, compID, trDepth ) : false;
}

#if JVET_AE0059_INTER_CCCM
bool TU::interCccmAllowed(const TransformUnit& tu)
{
  if (tu.cu->colorTransform)
  {
    return false;
  }
  if (!CU::interCccmSearchAllowed(*tu.cu))
  {
    return false;
  }
  if (!TU::getCbf(tu, COMPONENT_Y))
  {
    return false;
  }
#if JVET_AF0073_INTER_CCP_MERGE
  if (tu.interCcpMerge)
  {
    return false;
  }
#endif
  return true;
}
#endif

#if JVET_AF0073_INTER_CCP_MERGE
bool TU::interCcpMergeAllowed(const TransformUnit& tu)
{
  if (tu.cu->colorTransform)
  {
    return false;
  }
  if (!CU::interCcpMergeSearchAllowed(*tu.cu))
  {
    return false;
  }
  if (!TU::getCbf(tu, COMPONENT_Y))
  {
    return false;
  }
  return true;
}
#endif
// other tools

uint32_t getCtuAddr( const Position& pos, const PreCalcValues& pcv )
{
  return ( pos.x >> pcv.maxCUWidthLog2 ) + ( pos.y >> pcv.maxCUHeightLog2 ) * pcv.widthInCtus;
}

int getNumModesMip(const Size& block)
{
  switch( getMipSizeId(block) )
  {
  case 0: return 16;
  case 1: return  8;
  case 2: return  6;
  default: THROW( "Invalid mipSizeId" );
  }
}


int getMipSizeId(const Size& block)
{
  if( block.width == 4 && block.height == 4 )
  {
    return 0;
  }
  else if( block.width == 4 || block.height == 4 || (block.width == 8 && block.height == 8) )
  {
    return 1;
  }
  else
  {
    return 2;
  }

}

bool allowLfnstWithMip(const Size& block)
{
#if JVET_AB0067_MIP_DIMD_LFNST
  if (block.width >= 4 && block.height >= 4)
#else
  if (block.width >= 16 && block.height >= 16)
#endif
  {
    return true;
  }
  return false;
}
#if JVET_V0130_INTRA_TMP
bool allowLfnstWithTmp()
{
	return true;
}
#endif

#if JVET_AE0078_IBC_LIC_EXTENSION
int getLicDimBit(const CodingUnit& cu, const ComponentID compID)
{
  const int cuWidth = cu.blocks[compID].width;
  const int cuHeight = cu.blocks[compID].height;

  if (cu.ibcLicFlag)
  {
    if (cu.ibcLicIdx == IBC_LIC_IDX_T)
    {
      return floorLog2(cuWidth);
    }
    else if (cu.ibcLicIdx == IBC_LIC_IDX_L)
    {
      return floorLog2(cuHeight);
    }
  }

  return floorLog2(std::min(cuHeight, cuWidth));
}
#endif

#if INTER_LIC
bool CU::isLICFlagPresent(const CodingUnit& cu)
{
  if (CU::isIntra(cu) || !cu.slice->getUseLIC())
  {
    return false;
  }

#if JVET_AD0213_LIC_IMP
  if (cu.geoFlag || cu.firstPU->mergeFlag || CU::isIBC(cu) || cu.Y().area() < LIC_MIN_CU_PIXELS)
#else
  if (cu.geoFlag || cu.firstPU->mergeFlag || cu.firstPU->interDir == 3 || CU::isIBC(cu) || cu.Y().area() < LIC_MIN_CU_PIXELS)
#endif
  {
    return false;
  }
#if JVET_X0083_BM_AMVP_MERGE_MODE && JVET_AD0213_LIC_IMP
  if (cu.firstPU->amvpMergeModeFlag[0] || cu.firstPU->amvpMergeModeFlag[1])
  {
    return false;
  }
#endif
#if JVET_AG0098_AMVP_WITH_SBTMVP
  if (cu.firstPU->amvpSbTmvpFlag)
  {
    return false;
  }
#endif
#if JVET_Z0054_BLK_REF_PIC_REORDER && !JVET_AF0190_RPR_TMP_REORDER_LIC
  if (!cu.cs->pcv->isEncoder)
  {
    if (PU::useRefCombList(*cu.firstPU))
    {
      CHECK(cu.firstPU->refIdxLC < 0, "cu.firstPU->refIdxLC < 0");
      return (cu.firstPU->refIdxLC >= cu.cs->slice->getNumNonScaledRefPic() ? false : true);
    }
#if JVET_AD0213_LIC_IMP
    if (PU::useRefPairList(*cu.firstPU))
    {
      CHECK(cu.firstPU->refPairIdx < 0, "cu.firstPU->refPairIdx < 0");
      return (cu.firstPU->refPairIdx >= cu.cs->slice->getNumNonScaledRefPicPair() ? false : true);
    }
#endif
  }
#endif
#if JVET_Y0128_NON_CTC && !JVET_AF0190_RPR_TMP_REORDER_LIC
  if (!PU::checkRprLicCondition(*cu.firstPU))
  {
    return false;
  }
#endif

  return true;
}

void PU::spanLicFlags(PredictionUnit &pu, const bool LICFlag)
{
  MotionBuf mb = pu.getMotionBuf();
  MotionInfo *motionInfo = mb.buf;

  for (int y = 0; y < mb.height; y++)
  {
    for (int x = 0; x < mb.width; x++)
    {
      motionInfo[x].usesLIC = LICFlag;
    }
    motionInfo += mb.stride;
  }
}

#if JVET_AG0276_LIC_SLOPE_ADJUST
bool CU::isLicSlopeAllowed(const CodingUnit& cu)
{
  if (!cu.cs->sps->getLicSlopeAdjustEnabledFlag())
  {
    return false;
  }
  if (!cu.lx() && !cu.ly())
  {
    return false;
  }
  if (!licSlopeSizeTlCond(cu.lwidth(), cu.lheight(), cu.cs->slice->getTLayer()))
  {
    return false;
  }
  return true;
}
bool CU::licSlopeSizeTlCond(const int bw, const int bh, const int layerId)
{
  if (bw*bh > 4096 || bw*bh < 64 || bw >= 128 || bh >= 128 || bw == 4 || bh == 4 || (bw >= 64 && bh >=64))
  {
    return false;
  }
  if (layerId >= 5)
  {
    return false;
  }
  return true;
}
#endif
#endif

#if SIGN_PREDICTION
bool TU::getDelayedSignCoding( const TransformUnit &tu, const ComponentID compID )
{
  const uint32_t maxSize = CU::isIntra( *tu.cu ) ? SIGN_PRED_MAX_BS_INTRA : SIGN_PRED_MAX_BS_INTER;

  const uint32_t width = tu.blocks[compID].width;
  const uint32_t height = tu.blocks[compID].height;
  if( tu.cs->sps->getNumPredSigns() <= 0 || tu.mtsIdx[compID] == MTS_SKIP )
  {
    return false;
  }

  if( width < 4 || height < 4 || width > maxSize || height > maxSize )
  {
    return false;
  }
  return true;
}

bool TU::getUseSignPred( const TransformUnit &tu, const ComponentID compID )
{
#if JVET_Y0141_SIGN_PRED_IMPROVE
#if JVET_V0130_INTRA_TMP
  return TU::getDelayedSignCoding( tu, compID ) && (!tu.cu->lfnstIdx || !tu.cu->tmpFlag);
#else
  return TU::getDelayedSignCoding( tu, compID ) && !tu.cu->lfnstIdx;
#endif
#else
  return TU::getDelayedSignCoding( tu, compID ) && !tu.cu->lfnstIdx;
#endif
}

void TU::predBorderResi(const Position blkPos, const CPelBuf &recoBuf, const CPelBuf &predBuf, const ComponentID compID,
                        const uint32_t width, const uint32_t height, Pel *predResiBorder, const Pel defaultVal)
{
  const Pel *pReco = recoBuf.buf;
  const Pel *pPred = predBuf.buf;
  const ptrdiff_t strideReco = recoBuf.stride;
  const ptrdiff_t stridePred = predBuf.stride;

  const bool useLeft = blkPos.x != 0 || blkPos.y == 0;
  const bool useTop  = blkPos.y != 0 || blkPos.x == 0;

  Pel *dst = predResiBorder + height;

  if (useLeft)
  {
    if (blkPos.x != 0)
    {
      for (int32_t y = 0; y < height; y++)
      {
        Pel predVal = 2 * pReco[y * strideReco - 1] - pReco[y * strideReco - 2];
        dst[~y]     = predVal - pPred[y * stridePred];
      }
    }
    else
    {
      for (int32_t y = 0; y < height; y++)
      {
        dst[~y] = defaultVal - pPred[y * stridePred];
      }
    }
  }

  if (useTop)
  {
    if (blkPos.y != 0)
    {
      for (int32_t x = 0; x < width; x++)
      {
        Pel predVal = 2 * pReco[x - strideReco] - pReco[x - 2 * strideReco];
        dst[x]      = predVal - pPred[x];
      }
    }
    else
    {
      for (int32_t x = 0; x < width; x++)
      {
        dst[x] = defaultVal - pPred[x];
      }
    }
  }
}

Position TU::posSignHidingFirstCG( const TransformUnit &tu, ComponentID compID )
{
  Position pos( -1, -1 );
  int width = tu.blocks[compID].width;
  int height = tu.blocks[compID].height;
  if( tu.cu->cs->slice->getSignDataHidingEnabledFlag() && width >= 4 && height >= 4 )
  {
    // Mayte use 4x3 size is enough
    const ScanElement * scan = g_scanOrder[SCAN_GROUPED_4x4][SCAN_DIAG][gp_sizeIdxInfo->idxFrom( width )][gp_sizeIdxInfo->idxFrom( height )];
    const int cgSize = 16;
    int   lastNZPosInCG = -1, firstNZPosInCG = cgSize;
    Position posHiding( -1, -1 );
    const CCoeffBuf &buf = tu.getCoeffs( compID );

    for( auto n = cgSize - 1; n >= 0; --n )
    {
      ScanElement currSE = scan[n];
      Position    currPos = Position( currSE.x, currSE.y );

      if( buf.at( currPos ) )
      {
        lastNZPosInCG = n;
        posHiding = currPos;
        break;
      }
    }

    if( lastNZPosInCG != -1 )
    {
      for( auto n = 0; n < cgSize; n++ )
      {
        ScanElement currSE = scan[n];
        Position    currPos = Position( currSE.x, currSE.y );
        if( buf.at( currPos ) )
        {
          firstNZPosInCG = n;
          break;
        }
      }

    }
    if( firstNZPosInCG - lastNZPosInCG >= SBH_THRESHOLD )
    {
      return posHiding;
    }
  }
  return Position( -1, -1 );
}
#endif

#if JVET_X0083_BM_AMVP_MERGE_MODE
#if JVET_Y0128_NON_CTC
bool PU::checkIsValidMergeMvCand(const PredictionUnit &pu, int8_t mergeRefIdx[ NUM_REF_PIC_LIST_01 ])
{
  if (pu.amvpMergeModeFlag[REF_PIC_LIST_0] || pu.amvpMergeModeFlag[REF_PIC_LIST_1])
  {
    if (mergeRefIdx[pu.amvpMergeModeFlag[REF_PIC_LIST_0] ? 0 : 1] < 0)
    {
      return false;
    }
    return pu.cu->slice->getAmvpMergeModeValidCandPair(pu.amvpMergeModeFlag[REF_PIC_LIST_0] ? mergeRefIdx[0] : pu.refIdx[0],
                                                       pu.amvpMergeModeFlag[REF_PIC_LIST_0] ? pu.refIdx[1]   : mergeRefIdx[1]);
  }
  return true;
}
#else
bool PU::checkIsValidMergeMvCand(const CodingStructure &cs, const PredictionUnit &pu, const int curPoc, const int amvpPoc, int8_t mergeRefIdx[ NUM_REF_PIC_LIST_01 ])
{
  if (pu.amvpMergeModeFlag[REF_PIC_LIST_0])
  {
    if (mergeRefIdx[REF_PIC_LIST_0] < 0)
    {
      return false;
    }
    const int mergePoc = cs.slice->getRefPOC(REF_PIC_LIST_0, mergeRefIdx[REF_PIC_LIST_0]);
    if ((amvpPoc - curPoc) * (mergePoc -curPoc) > 0)
    {
      return false;
    }
  }
  if (pu.amvpMergeModeFlag[REF_PIC_LIST_1])
  {
    if (mergeRefIdx[REF_PIC_LIST_1] < 0)
    {
      return false;
    }
    const int mergePoc = cs.slice->getRefPOC(REF_PIC_LIST_1, mergeRefIdx[REF_PIC_LIST_1]);
    if ((amvpPoc - curPoc) * (mergePoc -curPoc) > 0)
    {
      return false;
    }
  }
  return true;
}
#endif
#if JVET_AA0093_REFINED_MOTION_FOR_ARMC
bool PU::isArmcRefinedMotionEnabled(const PredictionUnit &pu, unsigned mode)
{
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
  if (!(pu.cs->sps->getUseAML() && pu.cs->sps->getTMToolsEnableFlag()) || !pu.cs->sps->getUseArmcRefinedMotion())
#else
  if (!pu.cs->sps->getUseAML() || !pu.cs->sps->getUseArmcRefinedMotion())
#endif
  {
    return false;
  }
  int blkSize = pu.lwidth() * pu.lheight();
  if (mode == 0)
  {
    if (pu.cs->slice->getTLayer() < 5 && blkSize > 32 && blkSize <= 1024)
    {
      return true;
    }
  }
  else
  {
    if ((blkSize <= (mode == 1 ? 2048 : 4096)) && ( blkSize > 64))
    {
      return true;
    }
  }
  return false;
}
#endif
#endif

#if JVET_Z0135_TEMP_CABAC_WIN_WEIGHT
bool storeContexts( const Slice* slice, const int ctuXPosInCtus, const int ctuYPosInCtus )
{
  if( slice->getSPS()->getTempCabacInitMode() && !slice->isIntra() )
  {
    const PreCalcValues&  pcv = *slice->getPPS()->pcv;
    const int             ctuRsAddr = ctuXPosInCtus + ctuYPosInCtus * pcv.widthInCtus;
    return ctuRsAddr == pcv.sizeInCtus - 1;
  }
  return false;
}
#endif

#if JVET_AB0157_TMRL
bool CU::allowTmrl(const CodingUnit& cu)
{
  bool bReorder = true;
  if (!cu.Y().valid() || cu.predMode != MODE_INTRA || !isLuma(cu.chType) || cu.bdpcmMode
#if JVET_AD0082_TMRL_CONFIG
    || !cu.cs->sps->getUseTmrl() || MRL_NUM_REF_LINES <= 2 || cu.timd
#else
    || !cu.cs->sps->getUseMRL()
#endif
#if ENABLE_DIMD
    || cu.dimd
#endif
    )
  {
    bReorder = false;
  }

  bool isFirstLineOfCtu = (((cu.block(COMPONENT_Y).y) & ((cu.cs->sps)->getMaxCUWidth() - 1)) == 0);
  if (isFirstLineOfCtu)
  {
    bReorder = false;
  }

  return bReorder;
}
#endif
#if JVET_AC0144_AFFINE_DMVR_REGRESSION
int deriveAffineSubBlkSize(const int sz, const int minSbSz, const int deltaMvX, const int deltaMvY, const int shift)
{
  int sbSz = minSbSz;
  if (deltaMvX == 0 && deltaMvY == 0)
  {
    sbSz = sz;
  }
  else
  {
    int maxDmv = std::max(abs(deltaMvX), abs(deltaMvY)) * sbSz;
    int thred = 1 << (shift - 1);
    while (maxDmv < thred && sbSz < sz)
    {
      sbSz <<= 1;
      maxDmv <<= 1;
    }
  }
  return sbSz;
}

void PU::deriveAffineCandFromMvField(Position posLT, const int width, const int height, std::vector<RMVFInfo> mvInfoVec, Mv mvAffi[3])
{
  int64_t cMvX = 0, cMvY = 0;
  Mv cMv;
  Mv firstMv;
  int64_t parametersRMVF[2][4];

  firstMv.set(mvInfoVec[0].mvp.getHor(), mvInfoVec[0].mvp.getVer());
#if !JVET_AB0189_RMVF_BITLENGTH_CONTROL
  firstMv.hor = firstMv.hor >= 0 ? firstMv.hor << 2 : -(-firstMv.hor << 2);
  firstMv.ver = firstMv.ver >= 0 ? firstMv.ver << 2 : -(-firstMv.ver << 2);
#endif
  computeDeltaAndShift(posLT, firstMv, mvInfoVec);

  //-- Model with Linear Regression
  //-- Calculate RMVF parameters:
  int64_t sumbb[2][3][3];
  int64_t sumeb[2][3];

#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
  int denumShift = (getRMVFMSB(mvInfoVec.size()) - 5);
  if (denumShift < 0)
  {
    denumShift = 0;
  }

  xCalcRMVFParameters(mvInfoVec, parametersRMVF, sumbb, sumeb, denumShift, 0);
  for (int i = 0; i < 2; i++)
  {
    parametersRMVF[0][i] = divideRMVF(parametersRMVF[0][i], parametersRMVF[0][3], denumShift, false);
    parametersRMVF[1][i] = divideRMVF(parametersRMVF[1][i], parametersRMVF[1][3], denumShift, false);
  }
  parametersRMVF[0][2] = divideRMVF(parametersRMVF[0][2], parametersRMVF[0][3], denumShift, true);
  parametersRMVF[1][2] = divideRMVF(parametersRMVF[1][2], parametersRMVF[1][3], denumShift, true);
  cMvX = parametersRMVF[0][2];
  cMvY = parametersRMVF[1][2];
  cMvX = (firstMv.getHor() << MAX_CU_DEPTH) + cMvX;
  cMvY = (firstMv.getVer() << MAX_CU_DEPTH) + cMvY;
  int iMvX = int(cMvX);
  int iMvY = int(cMvY);
  roundAffineMv(iMvX, iMvY, MAX_CU_DEPTH);
  cMv = Mv(iMvX, iMvY);
  mvAffi[0] = cMv;
  mvAffi[0].clipToStorageBitDepth();

  cMvX = parametersRMVF[0][0] * width + parametersRMVF[0][2];
  cMvY = parametersRMVF[1][0] * width + parametersRMVF[1][2];
  cMvX = (firstMv.getHor() << MAX_CU_DEPTH) + cMvX;
  cMvY = (firstMv.getVer() << MAX_CU_DEPTH) + cMvY;
  iMvX = int(cMvX);
  iMvY = int(cMvY);
  roundAffineMv(iMvX, iMvY, MAX_CU_DEPTH);
  cMv = Mv(iMvX, iMvY);

  mvAffi[1] = cMv;
  mvAffi[1].clipToStorageBitDepth();


  // bottom-left CPMV
  cMvX = parametersRMVF[0][1] * height + parametersRMVF[0][2];
  cMvY = parametersRMVF[1][1] * height + parametersRMVF[1][2];

  cMvX = (firstMv.getHor() << MAX_CU_DEPTH) + cMvX;
  cMvY = (firstMv.getVer() << MAX_CU_DEPTH) + cMvY;
  iMvX = int(cMvX);
  iMvY = int(cMvY);
  roundAffineMv(iMvX, iMvY, MAX_CU_DEPTH);
  cMv = Mv(iMvX, iMvY);

  mvAffi[2] = cMv;
  mvAffi[2].clipToStorageBitDepth();
#else
  int shift = 2;
  int iNum = int(mvInfoVec.size());
  int shiftDets = 5 * (getRMVFMSB(iNum) - 4);
  if (shiftDets < 0) shiftDets = 0;
  int sumbb[2][3][3];
  int sumeb[2][3];
  int64_t m[3][3]; // parameter=det(md)/det(m)
  int64_t md[3][3];
  ////////////////// Extract statistics: Start
  //initialize values to zero
  memset(sumeb, 0, sizeof(sumeb));
  memset(sumbb, 0, sizeof(sumbb));
  buildRegressionMatrix(mvInfoVec, sumbb, sumeb, false);

  for (int c = 0; c < 2; c++)
  {
    for (int d = 0; d < 3; d++)
    {
      sumeb[c][d] >>= shift;
    }
    for (int d1 = 0; d1 < 3; d1++)
    {
      for (int d = 0; d < 3; d++)
      {
        sumbb[c][d1][d] >>= shift;
      }
    }
  }
  ////////////////// Extract statistics: End
  ////////////////// Extract Weight: Start
  memcpy(m, sumbb[0], sizeof(sumbb[0]));
  bool bBadMatrix = false;
  int64_t det2 = (sumbb[0][2][0]) * ((sumbb[0][0][1] * sumbb[0][1][2] - sumbb[0][1][1] * sumbb[0][0][2]))
    - (sumbb[0][2][1]) * ((sumbb[0][0][0] * sumbb[0][1][2] - sumbb[0][1][0] * sumbb[0][0][2]))
    + (sumbb[0][2][2]) * ((sumbb[0][0][0] * sumbb[0][1][1] - sumbb[0][1][0] * sumbb[0][0][1]));//det(3, m);
  det2 >>= shiftDets;
  if (det2 == 0)
  {
    bBadMatrix = true;
  }
  for (int c = 0; c < 2; c++)
  {
    // 1) find matrix parameters with cross-component model
    // Find w[c][d]
    //int64_t dets[3];
    if (!bBadMatrix)
    {
      for (int d = 0; d < 3; d++)
      {
        // Initialize md matrix
        memcpy(md, m, sizeof(m));
        // Replace coloumn D in md matrix
        for (int j = 0; j < 3; j++)
        {
          md[j][d] = sumeb[c][j];
        }

        int64_t det1 = (md[2][0]) * ((md[0][1] * md[1][2] - md[1][1] * md[0][2])/* >> rightShift*/)
          - (md[2][1]) * ((md[0][0] * md[1][2] - md[1][0] * md[0][2])/* >> rightShift*/)
          + (md[2][2]) * ((md[0][0] * md[1][1] - md[1][0] * md[0][1])/* >> rightShift*/);//det(3, md);
        det1 >>= shiftDets;

        //dets[d] = det1;
        parametersRMVF[c][d] = iNum * det1;

        // calcualte offset
        if (d == 2)
        {
          //parametersRMVF[c][2] = (det2 * sumeb[c][2] - dets[0] * sumbb[c][0][2] - dets[1] * sumbb[c][1][2]) << shift;
          parametersRMVF[c][3] = iNum * det2;
        }
      } // for d
    }

    if (bBadMatrix)
    {
      // 2) Find simple weight and offset, with non-corss component between x and y, i.e.: MVx=weight[0]*x+offset[0], MVy=weight[1]*y+offset[1]
      for (int d = 0; d < 3; d++)
      {
        parametersRMVF[c][d] = 0;
      }
      int64_t det1 = (iNum * sumeb[c][c] - sumbb[c][c][2] * sumeb[c][2]);
      int64_t det2 = (iNum * sumbb[c][c][c] - sumbb[c][c][2] * sumbb[c][c][2]);
      if (det2 == 0)
      {
        parametersRMVF[c][c] = 0;
        det1 = 0;
        det2 = 1;
        parametersRMVF[c][3] = iNum;
      }
      else
      {
        parametersRMVF[c][c] = iNum * det1;
        parametersRMVF[c][3] = iNum * det2;
      }
      parametersRMVF[c][2] = (det2 * sumeb[c][2] - det1 * sumbb[c][c][2]) << shift;
    } // End "bBadMatrix"

  } // end "for c"
    ////////////////// Extract Weights: End

  // top-left CPMV
  cMvX = parametersRMVF[0][2];
  cMvY = parametersRMVF[1][2];
  cMvX = divideRMVF(cMvX, parametersRMVF[0][3]);
  cMvY = divideRMVF(cMvY, parametersRMVF[1][3]);
  cMvX += firstMv.getHor();
  cMvY += firstMv.getVer();
  cMv = Mv((int)(cMvX), (int)(cMvY));
  cMv.hor = cMv.hor >= 0 ? (cMv.hor + 1) >> 2 : (cMv.hor + 2) >> 2;
  cMv.ver = cMv.ver >= 0 ? (cMv.ver + 1) >> 2 : (cMv.ver + 2) >> 2;
  mvAffi[0] = cMv;


  // top-right CPMV
  cMvX = parametersRMVF[0][0] * width + parametersRMVF[0][2];
  cMvY = parametersRMVF[1][0] * width + parametersRMVF[1][2];
  cMvX = divideRMVF(cMvX, parametersRMVF[0][3]);
  cMvY = divideRMVF(cMvY, parametersRMVF[1][3]);
  cMvX += firstMv.getHor();
  cMvY += firstMv.getVer();
  cMv = Mv((int)(cMvX), (int)(cMvY));
  cMv.hor = cMv.hor >= 0 ? (cMv.hor + 1) >> 2 : (cMv.hor + 2) >> 2;
  cMv.ver = cMv.ver >= 0 ? (cMv.ver + 1) >> 2 : (cMv.ver + 2) >> 2;

  mvAffi[1] = cMv;

  // bottom-left CPMV
  cMvX = parametersRMVF[0][1] * height + parametersRMVF[0][2];
  cMvY = parametersRMVF[1][1] * height + parametersRMVF[1][2];
  cMvX = divideRMVF(cMvX, parametersRMVF[0][3]);
  cMvY = divideRMVF(cMvY, parametersRMVF[1][3]);
  cMvX += firstMv.getHor();
  cMvY += firstMv.getVer();
  cMv = Mv((int)(cMvX), (int)(cMvY));
  cMv.hor = cMv.hor >= 0 ? (cMv.hor + 1) >> 2 : (cMv.hor + 2) >> 2;
  cMv.ver = cMv.ver >= 0 ? (cMv.ver + 1) >> 2 : (cMv.ver + 2) >> 2;
  mvAffi[2] = cMv;
#endif
}
#endif
#if JVET_AC0105_DIRECTIONAL_PLANAR
bool CU::isDirectionalPlanarAvailable(const CodingUnit &cu)
{
  PredictionUnit *pu = cu.firstPU;
  if (pu->multiRefIdx == 0 && cu.ispMode == 0 && cu.mipFlag == 0 && cu.dimd == 0 && cu.timd == 0 && cu.tmpFlag == 0 && cu.tmrlFlag == 0 && cu.sgpm == 0 && isLuma(cu.chType))
  {
    return true;
  }
  
  return false;
}
#endif
#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
bool CU::isGeoBlendAvailable( const CodingUnit& cu )
{
  return cu.slice->getSPS()->getUseGeoBlend(); 
}
#endif

#if GPM_CURVE
bool CU::isGeoCURVEAvailable(int splitDir,int cuSize)
{
  switch (cuSize)
  {
  case 16:
  case 32:
    if(splitDir == 14 || splitDir == 6 || splitDir == 28 || splitDir == 2 || splitDir == 32)
      return true;
    break;
  case 64:
    return true;
    break;
  default:
    break;
  }
  return false;
}
#endif

#if JVET_AC0130_NSPT
int PU::getNSPTMatrixDim( int width, int height )
{
  int dimension = ( width == 8 && height == 8 ) ? 32 : ( ( ( width == 4 && height == 8 ) || ( width == 8 && height == 4 ) ) ? 20 : 16 );

  dimension = ( ( width == 4 && height == 16 ) || ( width == 16 && height == 4 ) ) ? 24 : ( ( ( width == 8 && height == 16 ) || ( width == 16 && height == 8 ) ) ? 40 : dimension );

#if JVET_AE0086_LARGE_NSPT
  dimension = ( ( width == 4 && height == 32 ) || ( width == 32 && height == 4 ) ) ? 20 : ( ( ( width == 8 && height == 32 ) || ( width == 32 && height == 8 ) ) ? 24 : dimension );
#endif

  return dimension;
}

uint32_t PU::getFinalIntraModeForTransform( const TransformUnit &tu, const ComponentID compID )
{
  const CompArea& area = tu.blocks[ compID ];
  uint32_t intraMode = PU::getFinalIntraMode( *tu.cs->getPU( area.pos(), toChannelType( compID ) ), toChannelType( compID ) );

#if JVET_W0123_TIMD_FUSION
  if( compID != COMPONENT_Y && PU::isLMCMode( tu.cs->getPU( area.pos(), toChannelType( compID ) )->intraDir[ toChannelType( compID ) ] ) )
#else
  if( PU::isLMCMode( tu.cs->getPU( area.pos(), toChannelType( compID ) )->intraDir[ toChannelType( compID ) ] ) )
#endif
  {
    intraMode = PU::getCoLocatedIntraLumaMode( *tu.cs->getPU( area.pos(), toChannelType( compID ) ) );
  }
  if( PU::isMIP( *tu.cs->getPU( area.pos(), toChannelType( compID ) ), toChannelType( compID ) ) )
  {
#if JVET_AB0067_MIP_DIMD_LFNST
    intraMode = tu.cu->mipDimdMode;
#else
    intraMode = PLANAR_IDX;
#endif
  }
#if JVET_V0130_INTRA_TMP
  if( PU::isTmp( *tu.cs->getPU( area.pos(), toChannelType( compID ) ), toChannelType( compID ) ) )
  {
#if JVET_AC0115_INTRA_TMP_DIMD_MTS_LFNST
      intraMode = tu.cu->intraTmpDimdMode;
#else
      intraMode = PLANAR_IDX;
#endif
  }
#endif
#if JVET_AG0061_INTER_LFNST_NSPT
  if (CU::isInter(*tu.cu))
  {
    intraMode = tu.cu->dimdDerivedIntraDir;
  }
  if (tu.cu->geoFlag)
  {
    intraMode = g_geoAngle2IntraAng[g_geoParams[tu.cu->firstPU->geoSplitDir][0]];
  }
#endif
#if JVET_AB0155_SGPM
  if( PU::isSgpm( *tu.cs->getPU( area.pos(), toChannelType( compID ) ), toChannelType( compID ) ) )
  {
    intraMode = g_geoAngle2IntraAng[g_geoParams[tu.cu->sgpmSplitDir][0]];
  }
#endif
#if JVET_W0123_TIMD_FUSION
  if( tu.cu->timd && compID == COMPONENT_Y )
  {
    intraMode = MAP131TO67( intraMode );
  }
#endif
#if JVET_AC0105_DIRECTIONAL_PLANAR
  if (compID == COMPONENT_Y && intraMode == PLANAR_IDX)
  {
    if (tu.cu->plIdx == 2)
    {
      intraMode = HOR_IDX;
    }
    else if (tu.cu->plIdx == 1)
    {
      intraMode = VER_IDX;
    }
  }
#endif
#if JVET_AC0071_DBV
  if (compID != COMPONENT_Y && intraMode == DBV_CHROMA_IDX)
  {
    intraMode = PLANAR_IDX;
  }
#endif
#if JVET_AD0085_TMRL_EXTENSION
  if (tu.cu->tmrlFlag && compID == COMPONENT_Y)
  {
    intraMode = MAP131TO67(intraMode);
  }
#endif
#if JVET_AG0058_EIP
  if (PU::isEIP(*tu.cs->getPU(area.pos(), toChannelType(compID)), toChannelType(compID)))
  {
    intraMode = tu.cu->eipModel.eipDimdMode;
  }
#endif

  CHECK( intraMode >= NUM_INTRA_MODE - 1, "Invalid intra mode" );

  intraMode = PU::getNSPTIntraMode( PU::getWideAngle( tu, intraMode, compID ) );

  return intraMode;
}

uint32_t PU::getNSPTIntraMode( int wideAngPredMode )
{
  uint32_t intraMode;
  if( wideAngPredMode < 0 )
  {
    intraMode = ( uint32_t ) ( wideAngPredMode + ( NUM_EXT_LUMA_MODE >> 1 ) + NUM_LUMA_MODE );
  }
  else if( wideAngPredMode >= NUM_LUMA_MODE )
  {
    intraMode = ( uint32_t ) ( wideAngPredMode + ( NUM_EXT_LUMA_MODE >> 1 ) );
  }
  else
  {
    intraMode = ( uint32_t ) wideAngPredMode;
  }
  return intraMode;
}

bool CU::isNSPTAllowed( const TransformUnit &tu, const ComponentID compID, int width, int height, bool isIntra )
{
  bool allowNSPT = isIntra;
#if JVET_AG0061_INTER_LFNST_NSPT
  if (CU::isInter(*tu.cu))
  {
    allowNSPT = true;
  }
#endif
  if( allowNSPT )
  {
    allowNSPT = ( ( width == 4 && height ==  4 ) || ( width ==  8 && height == 8 ) || ( width == 4 && height ==  8 ) || ( width ==  8 && height == 4 ) ||
                  ( width == 4 && height == 16 ) || ( width == 16 && height == 4 ) || ( width == 8 && height == 16 ) || ( width == 16 && height == 8 )
#if JVET_AE0086_LARGE_NSPT
               || ( width == 4 && height == 32 ) || ( width == 32 && height == 4 ) || ( width == 8 && height == 32 ) || ( width == 32 && height == 8 )
#endif
                );
  }

  return allowNSPT;
}

bool CU::nsptApplyCond( const TransformUnit& tu, ComponentID compID, bool allowNSPT )
{
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
  bool cond = allowNSPT && tu.cu->lfnstIdx > 0 && ( tu.cu->isSepTree() ? true : isLuma( compID ) );
#else
  bool cond = allowNSPT && tu.cu->lfnstIdx > 0 && ( CS::isDualITree( *tu.cs ) ? true : isLuma( compID ) );
#endif

  return cond;
}
#endif

#if JVET_AD0085_TMRL_EXTENSION
int getSpatialIpm(const PredictionUnit& pu, uint8_t* spatialIpm, const int maxCands
#if JVET_AC0094_REF_SAMPLES_OPT
  , const bool& isForcedValid
#endif
  , bool extPrecision
#if JVET_AD0085_MPM_SORTING
  , IntraPrediction* pIntraPred/* = nullptr*/
#endif
)
{
#if JVET_AC0094_REF_SAMPLES_OPT
  uint8_t arrayReserved[NUM_LUMA_MODE];
  int     nbReserved{ 0 };
#endif
  bool includedMode[EXT_VDIA_IDX + 1]{ false };
  includedMode[PLANAR_IDX] = true;
  const CompArea& area = pu.block(COMPONENT_Y);
  const Position topLeft = area.topLeft();
  int numCand = 0;
  const ChannelType channelType = CHANNEL_TYPE_LUMA;
  auto getIpm = [&](Position pos) -> void
  {
    if (numCand >= maxCands)
    {
      return;
    }
    const PredictionUnit* neighborPu = pu.cs->getPURestricted(pos, pu, channelType);
    if (neighborPu)
    {
      bool found = false;
      if (CU::isIntra(*neighborPu->cu))
      {
        auto neighborMode = PU::getIntraDirLuma(*neighborPu);
        if (neighborPu->cu->sgpm)
        {
          neighborMode = g_geoAngle2IntraAng[g_geoParams[neighborPu->cu->sgpmSplitDir][0]];;
        }
#if JVET_AD0085_TMRL_EXTENSION
        if (extPrecision)
        {
          spatialIpm[numCand] = (neighborPu->cu->timd || neighborPu->cu->tmrlFlag) ? neighborMode : MAP67TO131(neighborMode);
        }
        else
        {
          spatialIpm[numCand] = (neighborPu->cu->timd || neighborPu->cu->tmrlFlag) ? MAP131TO67(neighborMode) : neighborMode;
        }
#else
        spatialIpm[numCand] = neighborPu->cu->timd ? MAP131TO67(neighborMode) : neighborMode;
#endif
        found = true;
      }
      if (CU::isInter(*neighborPu->cu) || CU::isIBC(*neighborPu->cu))
      {
        spatialIpm[numCand] = neighborPu->getIpmInfo(pos);
        if (neighborPu->cu->geoFlag)
        {
          spatialIpm[numCand] = g_geoAngle2IntraAng[g_geoParams[neighborPu->geoSplitDir][0]];
        }
#if JVET_AD0085_TMRL_EXTENSION
        if (extPrecision)
        {
          spatialIpm[numCand] = MAP67TO131(spatialIpm[numCand]);
        }
#endif
        found = true;
      }
      if (found && !includedMode[spatialIpm[numCand]])
      {
#if JVET_AC0094_REF_SAMPLES_OPT
        if (CU::isIdxModeValid((pu.cu)->areAboveRightUnavail, (pu.cu)->areBelowLeftUnavail,
          (pu.cu)->lheight(), (pu.cu)->lwidth(), extPrecision ? MAP131TO67(spatialIpm[numCand]) : spatialIpm[numCand], isForcedValid))
        {
#endif
          if (!includedMode[spatialIpm[numCand]])
          {
            includedMode[spatialIpm[numCand++]] = true;
          }
#if JVET_AC0094_REF_SAMPLES_OPT
        }
        else
        {
          arrayReserved[nbReserved] = spatialIpm[numCand];
          nbReserved++;
        }
#endif
      }
    }
  };

  // add spatial positions
  getIpm(topLeft.offset(-1, pu.Y().height - 1));
  getIpm(topLeft.offset(pu.Y().width - 1, -1));
  getIpm(topLeft.offset(-1, -1));
  getIpm(topLeft.offset(pu.Y().width, -1));
  getIpm(topLeft.offset(-1, pu.Y().height));

#if SECONDARY_MPM
  //adding dimd modes
  if (pu.cu->slice->getSPS()->getUseDimd())
  {
    if (pu.cu->dimdMode != -1 && numCand < maxCands)
    {
      auto currMode = pu.cu->dimdMode;
      if (extPrecision)
      {
        currMode = MAP67TO131(currMode);
      }
      spatialIpm[numCand] = currMode;
      if (!includedMode[spatialIpm[numCand]])
      {
        includedMode[spatialIpm[numCand++]] = true;
      }
    }

    if (pu.cu->dimdBlendMode[0] != -1 && numCand < maxCands)
    {
      auto currMode = pu.cu->dimdBlendMode[0];
      if (extPrecision)
      {
        currMode = MAP67TO131(currMode);
      }
      spatialIpm[numCand] = currMode;
      if (!includedMode[spatialIpm[numCand]])
      {
        includedMode[spatialIpm[numCand++]] = true;
      }
    }
  }
#endif

  // non-adjacent spatial candidates
  int offsetX = 0;
  int offsetY = 0;
  const int numNACandidate[4] = { 3, 5, 5, 5 };
  const int idxMap[4][5] = { { 0, 1, 4 },{ 0, 1, 2, 3, 4 },{ 0, 1, 2, 3, 4 },{ 0, 1, 2, 3, 4 } };
  for (int iDistanceIndex = 0; iDistanceIndex < NADISTANCE_LEVEL; iDistanceIndex++)
  {
    const int iNADistanceHor = pu.Y().width * (iDistanceIndex + 1);
    const int iNADistanceVer = pu.Y().height * (iDistanceIndex + 1);

    for (int iNASPIdx = 0; iNASPIdx < numNACandidate[iDistanceIndex]; iNASPIdx++)
    {
      switch (idxMap[iDistanceIndex][iNASPIdx])
      {
      case 0:offsetX = -iNADistanceHor - 1; offsetY = pu.Y().height + iNADistanceVer - 1; break;
      case 1:offsetX = pu.Y().width + iNADistanceHor - 1; offsetY = -iNADistanceVer - 1; break;
      case 2:offsetX = pu.Y().width >> 1;   offsetY = -iNADistanceVer - 1;    break;
      case 3:offsetX = -iNADistanceHor - 1; offsetY = pu.Y().height >> 1; break;
      case 4:offsetX = -iNADistanceHor - 1; offsetY = -iNADistanceVer - 1;    break;
      default: printf("error!"); exit(0); break;
      }
      getIpm(topLeft.offset(offsetX, offsetY));
    }
  }
#if JVET_AD0085_MPM_SORTING
  if (pIntraPred && numCand > ((pu.cs->slice->getSliceType() == I_SLICE) ? NUM_PRIMARY_MOST_PROBABLE_MODES - 1 : 1))
  {
    pIntraPred->deriveMPMSorted(pu, spatialIpm, numCand, 0);
  }
#endif

#if JVET_AC0094_REF_SAMPLES_OPT
  if (!isForcedValid)
  {
    for (int i{ 0 }; i < nbReserved && numCand < maxCands; i++)
    {
      auto currMode = arrayReserved[i];
      if (!includedMode[currMode])
      {
        spatialIpm[numCand] = currMode;
        includedMode[currMode] = true;
        numCand++;
      }
    }
  }
#endif

  CHECK(numCand > maxCands, "");

  return numCand;
}

void fillMPMList(const PredictionUnit& pu, uint8_t* mpm, const int maxCands, const int numCand, bool extPrecision)
{
  if (numCand >= maxCands)
  {
    return;
  }

  bool includedMode[EXT_VDIA_IDX + 1]{ false };
  CHECK(numCand > maxCands, "");
  int idx = 0;
  for (; idx < numCand; idx++)
  {
    includedMode[mpm[idx]] = true;
  }

  const int currNumCands = idx;
  const int offset = extPrecision ? EXT_VDIA_IDX - 5 : NUM_LUMA_MODE - 6;
  const int mod = offset + 3;
  for (int i = 0; i < currNumCands; i++)
  {
    if (mpm[i] <= DC_IDX)
    {
      continue;
    }
    for (int deltaAngular = 0; deltaAngular < 4; deltaAngular++)
    {
      if (idx < maxCands)
      {
        auto mode = ((mpm[i] + offset - deltaAngular) % mod) + 2;
        if (!includedMode[mode])
        {
          includedMode[mode] = true;
          mpm[idx++] = mode;
        }
      }

      if (idx < maxCands)
      {
        auto mode = ((mpm[i] - 1 + deltaAngular) % mod) + 2;
        if (!includedMode[mode])
        {
          includedMode[mode] = true;
          mpm[idx++] = mode;
        }
      }
    }
  }

  uint8_t mpmDefault[] = { DC_IDX, VER_IDX, HOR_IDX, VER_IDX - 4, VER_IDX + 4, 14, 22, 42, 58, 10, 26,
                            38, 62, 6, 30, 34, 66, 2, 48, 52, 16 };
  for (int i = 0; idx < maxCands; i++)
  {
    auto mode = mpmDefault[i];
    if (extPrecision)
    {
      mode = MAP67TO131(mode);
    }
    if (!includedMode[mode])
    {
      includedMode[mode] = true;
      mpm[idx++] = mode;
    }
  }
  CHECK(idx != maxCands, "");
}

void fillNonMPMList(uint8_t* mpm, uint8_t* non_mpm)
{
  bool includedMode[EXT_VDIA_IDX + 1]{ false };
  for (int i = 0; i < NUM_MOST_PROBABLE_MODES; i++)
  {
    auto mode = mpm[i];
    includedMode[mode] = true;
    CHECK(mode < PLANAR_IDX || mode >= NUM_LUMA_MODE, "");
  }

  int numNonMPM = 0;
  for (int i = 0; i < NUM_LUMA_MODE; i++)
  {
    if (!includedMode[i] && numNonMPM < NUM_NON_MPM_MODES)
    {
      non_mpm[numNonMPM++] = i;
    }
  }
  CHECK(numNonMPM != NUM_LUMA_MODE - NUM_MOST_PROBABLE_MODES, "");
}
#endif
#if JVET_AG0061_INTER_LFNST_NSPT
int buildHistogram(const Pel *pReco, int iStride, uint32_t uiHeight, uint32_t uiWidth, int *piHistogram, int direction,
                   int bw, int bh)
{
  const int wStep = 1, hStep = 1;
  int       angTable[17]   = { 0,     2048,  4096,  6144,  8192,  12288, 16384, 20480, 24576,
                       28672, 32768, 36864, 40960, 47104, 53248, 59392, 65536 };
  int       offsets[4]     = { HOR_IDX, HOR_IDX, VER_IDX, VER_IDX };
  int       dirs[4]        = { -1, 1, -1, 1 };
  int       mapXgrY1[2][2] = { { 1, 0 }, { 0, 1 } };
  int       mapXgrY0[2][2] = { { 2, 3 }, { 3, 2 } };

  for (uint32_t y = 0; y < uiHeight; y += hStep)
  {
    for (uint32_t x = 0; x < uiWidth; x += wStep)
    {
      if ((direction == 3) && x == (uiWidth - 1) && y == (uiHeight - 1))
      {
        continue;
      }

      const Pel *pRec = pReco + y * iStride + x;

      int iDy =
        pRec[-iStride - 1] + 2 * pRec[-1] + pRec[iStride - 1] - pRec[-iStride + 1] - 2 * pRec[+1] - pRec[iStride + 1];
      int iDx = pRec[iStride - 1] + 2 * pRec[iStride] + pRec[iStride + 1] - pRec[-iStride - 1] - 2 * pRec[-iStride]
                - pRec[-iStride + 1];

      if (iDy == 0 && iDx == 0)
      {
        continue;
      }

      int iAmp       = (int) (abs(iDx) + abs(iDy));
      int iAngUneven = -1;
      // for determining region
      if (iDx != 0 && iDy != 0)   // pure angles are not concerned
      {
        // get the region
        int signx  = iDx < 0 ? 1 : 0;
        int signy  = iDy < 0 ? 1 : 0;
        int absx   = iDx < 0 ? -iDx : iDx;
        int absy   = iDy < 0 ? -iDy : iDy;
        int gtY    = absx > absy ? 1 : 0;
        int region = gtY ? mapXgrY1[signy][signx] : mapXgrY0[signy][signx];
        // region = (region == 1 ? 2 : (region == 2 ? 1 : (region == 3 ? 4 : 3)));
#if JVET_X0149_TIMD_DIMD_LUT
        int s0   = gtY ? absy : absx;
        int s1   = gtY ? absx : absy;
        int x    = floorLog2(s1);
        int norm = (s1 << 4 >> x) & 15;
        int v    = g_gradDivTable[norm] | 8;
        x += (norm != 0);
        int shift = 13 - x;
        int ratio;
        if (shift < 0)
        {
          shift   = -shift;
          int add = (1 << (shift - 1));
          ratio   = (s0 * v + add) >> shift;
        }
        else
        {
          ratio = (s0 * v) << shift;
        }

        // iRatio after integerization can go beyond 2^16
#else
        float fRatio = gtY ? static_cast<float>(absy) / static_cast<float>(absx)
                           : static_cast<float>(absx) / static_cast<float>(absy);
        float fRatioScaled = fRatio * (1 << 16);
        int ratio = static_cast<int>(fRatioScaled);
#endif
        // get ang_idx
        int idx = 16;
        for (int i = 1; i < 17; i++)
        {
          if (ratio <= angTable[i])
          {
            idx = ratio - angTable[i - 1] < angTable[i] - ratio ? i - 1 : i;
            break;
          }
        }

        iAngUneven = offsets[region] + dirs[region] * idx;
        // iAngUneven = offsets[region - 1] + dirs[region - 1] * idx;
      }
      else
      {
        iAngUneven = iDx == 0 ? VER_IDX : HOR_IDX;
      }

      CHECK(iAngUneven < 0, "Wrong mode in DIMD histogram");
      CHECK(iAngUneven >= NUM_LUMA_MODE, "Wrong mode in DIMD histogram");

      piHistogram[iAngUneven] += iAmp;
    }
  }
  return 0;
}
#endif

#if JVET_AG0164_AFFINE_GPM
int  PU::getAffGPMCtxOffset(const PredictionUnit& pu)
{
  return DeriveCtx::CtxAffineFlag(*pu.cu);//CtxGPMAffineFlag(*pu.cu);
}

bool PU::isAffineGPMSizeValid(const PredictionUnit& pu)
{
  return true;
}

bool PU::isAffineGPMValid(const PredictionUnit& pu)
{
  if (!pu.cu->slice->getSPS()->getUseAffine() || !pu.cu->slice->getSPS()->getUseGeo())
  {
    return false;
  }
  if (pu.cu->slice->getSPS()->getMaxNumGpmAffCand() == 0)
  {
    return false;
  }
  if (!isAffineGPMSizeValid(pu) )
  {
    return false;
  }
  return true;
}

void PU::getGeoAffMergeCandidates(PredictionUnit& pu, AffineMergeCtx& gpmAffMrgCtx, InterPrediction* pcInterPred, AffineMergeCtx* affMergeCtx)
{
  AffineMergeCtx tmpMergeCtx;

  uint32_t maxNumAffineMergeCand = pu.cs->sps->getMaxNumAffineMergeCand();

  uint32_t maxNumGeoMergeCand = pu.cs->sps->getMaxNumGpmAffCand();

  if (pu.cs->sps->getUseAML()
#if JVET_AE0174_NONINTER_TM_TOOLS_CONTROL
    && pu.cs->sps->getTMToolsEnableFlag()
#endif
    )
  {
    maxNumGeoMergeCand = std::max((int)maxNumGeoMergeCand, GEO_MAX_NUM_UNI_AFF_CANDS_ARMC);
  }
  gpmAffMrgCtx.numValidMergeCand = 0;
  gpmAffMrgCtx.maxNumMergeCand = maxNumGeoMergeCand;
  gpmAffMrgCtx.m_isGPMAff = 1;

  for (int32_t i = 0; i < maxNumGeoMergeCand; i++)
  {
    for (int mvNum = 0; mvNum < 3; mvNum++)
    {
      gpmAffMrgCtx.mvFieldNeighbours[(i << 1) + 0][mvNum].setMvField(Mv(), -1);
      gpmAffMrgCtx.mvFieldNeighbours[(i << 1) + 1][mvNum].setMvField(Mv(), -1);
    }
    gpmAffMrgCtx.interDirNeighbours[i] = 0;
    gpmAffMrgCtx.affineType[i] = AFFINEMODEL_6PARAM;
    gpmAffMrgCtx.mergeType[i] = MRG_TYPE_DEFAULT_N;
    gpmAffMrgCtx.bcwIdx[i] = BCW_DEFAULT;
#if JVET_AG0276_NLIC
    gpmAffMrgCtx.altLMFlag[i] = false;
    gpmAffMrgCtx.altLMParaNeighbours[i].resetAltLinearModel();
#endif
#if INTER_LIC
    gpmAffMrgCtx.licFlags[i] = false;
#endif
#if JVET_AD0193_ADAPTIVE_OBMC_CONTROL
    gpmAffMrgCtx.obmcFlags[i] = false;
#endif
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
    gpmAffMrgCtx.numAffCandToTestEnc = maxNumAffineMergeCand;
    gpmAffMrgCtx.candCost[i] = MAX_UINT64;
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION
    gpmAffMrgCtx.colIdx[i] = 0;
#endif
  }

  if (affMergeCtx == NULL)
  {
#if TM_MRG
    const bool tmMergeFlag = pu.tmMergeFlag;
    pu.tmMergeFlag = false;
#endif
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION  
    MergeCtx mrgCtxAll[2];
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
    AffineMergeCtx affineRMVFCtx;
    AffineMergeCtx affineRMVFOriCtx;

#if JVET_AG0276_NLIC
    AltLMAffineMergeCtx altAffineRMVFCtx;
    altAffineRMVFCtx.init();
#if JVET_AG0276_LIC_FLAG_SIGNALING
    AltLMAffineMergeCtx altBRAffineRMVFCtx;
    altBRAffineRMVFCtx.init();
#endif
#endif
    uint16_t addNumRMVF = 0;
    PU::getRMVFAffineCand(pu, affineRMVFCtx, affineRMVFOriCtx, pcInterPred, addNumRMVF
#if JVET_AG0276_NLIC
    , altAffineRMVFCtx
#if JVET_AG0276_LIC_FLAG_SIGNALING
    , altBRAffineRMVFCtx
#endif
#endif
    );
#endif
    PU::getAffineMergeCand(pu, tmpMergeCtx
#if JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION  
      , mrgCtxAll
#endif
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
      , nullptr
#endif
#if JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS
      , affineRMVFCtx
      , affineRMVFOriCtx
      , addNumRMVF
#endif
#if AFFINE_MMVD
      , -1, false
#else
      ,-1
#endif
#if JVET_Z0139_NA_AFF && JVET_W0090_ARMC_TM
      ,false 
#endif
      , true
    );

#if TM_MRG
    pu.tmMergeFlag = tmMergeFlag;
#endif
  }
  else
  {
    tmpMergeCtx.numValidMergeCand = affMergeCtx->numValidMergeCand;
    memcpy(tmpMergeCtx.interDirNeighbours, affMergeCtx->interDirNeighbours, tmpMergeCtx.numValidMergeCand * sizeof(unsigned char));
    memcpy(tmpMergeCtx.mvFieldNeighbours, affMergeCtx->mvFieldNeighbours, (tmpMergeCtx.numValidMergeCand << 1) * 3 * sizeof(MvField));
    memcpy(tmpMergeCtx.affineType, affMergeCtx->affineType, tmpMergeCtx.numValidMergeCand * sizeof(EAffineModel));
    memcpy(tmpMergeCtx.bcwIdx, affMergeCtx->bcwIdx, tmpMergeCtx.numValidMergeCand * sizeof(uint8_t));
    memcpy(tmpMergeCtx.licFlags, affMergeCtx->licFlags, tmpMergeCtx.numValidMergeCand * sizeof(bool));
  }

  int affCandIdx = 0;
  for (int32_t i = 0; i < tmpMergeCtx.numValidMergeCand; i++)
  {
    if (tmpMergeCtx.mergeType[i] == MRG_TYPE_SUBPU_ATMVP)
    {
      continue;
    }
    int parity = affCandIdx & 1;
    affCandIdx++;
    if (tmpMergeCtx.interDirNeighbours[i] & (0x01 + parity))
    {
      gpmAffMrgCtx.interDirNeighbours[gpmAffMrgCtx.numValidMergeCand] = 1 + parity;
      gpmAffMrgCtx.affineType[gpmAffMrgCtx.numValidMergeCand] = tmpMergeCtx.affineType[i];

      for (int cpmvIdx = 0; cpmvIdx < 3; cpmvIdx++)
      {
        gpmAffMrgCtx.mvFieldNeighbours[(gpmAffMrgCtx.numValidMergeCand << 1) + (!parity)][cpmvIdx].mv = Mv(0, 0);
        gpmAffMrgCtx.mvFieldNeighbours[(gpmAffMrgCtx.numValidMergeCand << 1) + parity][cpmvIdx].mv = tmpMergeCtx.mvFieldNeighbours[(i << 1) + parity][cpmvIdx].mv;
        gpmAffMrgCtx.mvFieldNeighbours[(gpmAffMrgCtx.numValidMergeCand << 1) + (!parity)][cpmvIdx].refIdx = -1;
        gpmAffMrgCtx.mvFieldNeighbours[(gpmAffMrgCtx.numValidMergeCand << 1) + parity][cpmvIdx].refIdx = tmpMergeCtx.mvFieldNeighbours[(i << 1) + parity][cpmvIdx].refIdx;
      }

      if (gpmAffMrgCtx.xCheckSimilarMotion(gpmAffMrgCtx.numValidMergeCand))
      {
        continue;
      }

      gpmAffMrgCtx.numValidMergeCand++;

      if (gpmAffMrgCtx.numValidMergeCand == maxNumGeoMergeCand)
      {
        return;
      }
      continue;
    }

    if (tmpMergeCtx.interDirNeighbours[i] & (0x02 - parity))
    {
      gpmAffMrgCtx.interDirNeighbours[gpmAffMrgCtx.numValidMergeCand] = 2 - parity;
      gpmAffMrgCtx.affineType[gpmAffMrgCtx.numValidMergeCand] = tmpMergeCtx.affineType[i];

      for (int cpmvIdx = 0; cpmvIdx < 3; cpmvIdx++)
      {
        gpmAffMrgCtx.mvFieldNeighbours[(gpmAffMrgCtx.numValidMergeCand << 1) + (!parity)][cpmvIdx].mv = tmpMergeCtx.mvFieldNeighbours[(i << 1) + (!parity)][cpmvIdx].mv;
        gpmAffMrgCtx.mvFieldNeighbours[(gpmAffMrgCtx.numValidMergeCand << 1) + parity][cpmvIdx].mv = Mv(0, 0);
        gpmAffMrgCtx.mvFieldNeighbours[(gpmAffMrgCtx.numValidMergeCand << 1) + (!parity)][cpmvIdx].refIdx = tmpMergeCtx.mvFieldNeighbours[(i << 1) + (!parity)][cpmvIdx].refIdx;
        gpmAffMrgCtx.mvFieldNeighbours[(gpmAffMrgCtx.numValidMergeCand << 1) + parity][cpmvIdx].refIdx = -1;
      }
      if (gpmAffMrgCtx.xCheckSimilarMotion(gpmAffMrgCtx.numValidMergeCand))
      {
        continue;
      }

      gpmAffMrgCtx.numValidMergeCand++;
      if (gpmAffMrgCtx.numValidMergeCand == maxNumGeoMergeCand)
      {
        return;
      }
    }
  }

  // add more parity based geo candidates, in an opposite parity rule
  if (gpmAffMrgCtx.numValidMergeCand < maxNumGeoMergeCand)
  {
    affCandIdx = 0;
    for (int32_t i = 0; i < tmpMergeCtx.numValidMergeCand; i++)
    {
      if (tmpMergeCtx.mergeType[i] == MRG_TYPE_SUBPU_ATMVP)
      {
        continue;
      }
      int parity = affCandIdx & 1;
      affCandIdx++;
      if (tmpMergeCtx.interDirNeighbours[i] == 3)
      {
        gpmAffMrgCtx.interDirNeighbours[gpmAffMrgCtx.numValidMergeCand] = 2 - parity;

        gpmAffMrgCtx.affineType[gpmAffMrgCtx.numValidMergeCand] = tmpMergeCtx.affineType[i];

        for (int cpmvIdx = 0; cpmvIdx < 3; cpmvIdx++)
        {
          gpmAffMrgCtx.mvFieldNeighbours[(gpmAffMrgCtx.numValidMergeCand << 1) + (!parity)][cpmvIdx].mv = tmpMergeCtx.mvFieldNeighbours[(i << 1) + (!parity)][cpmvIdx].mv;
          gpmAffMrgCtx.mvFieldNeighbours[(gpmAffMrgCtx.numValidMergeCand << 1) + (parity)][cpmvIdx].mv = Mv(0, 0);
          gpmAffMrgCtx.mvFieldNeighbours[(gpmAffMrgCtx.numValidMergeCand << 1) + (!parity)][cpmvIdx].refIdx = tmpMergeCtx.mvFieldNeighbours[(i << 1) + (!parity)][cpmvIdx].refIdx;
          gpmAffMrgCtx.mvFieldNeighbours[(gpmAffMrgCtx.numValidMergeCand << 1) + (parity)][cpmvIdx].refIdx = -1;
        }
        if (gpmAffMrgCtx.xCheckSimilarMotion(gpmAffMrgCtx.numValidMergeCand))
        {
          continue;
        }
        gpmAffMrgCtx.numValidMergeCand++;

        if (gpmAffMrgCtx.numValidMergeCand == maxNumGeoMergeCand)
        {
          return;
        }
      }
    }
  }

  // add at most two average based geo candidates
  if (gpmAffMrgCtx.numValidMergeCand < maxNumGeoMergeCand)
  {
    // add one L0 cand by averaging the first two available L0 candidates
    int cnt = 0;
    int firstAvailRefIdx = -1;
    Mv  avgMv[3];
    avgMv[0].setZero();
    avgMv[1].setZero();
    avgMv[2].setZero();

    for (int i = 0; i < gpmAffMrgCtx.numValidMergeCand; i++)
    {
      if (cnt == 2)
      {
        break;
      }
      if (gpmAffMrgCtx.interDirNeighbours[i] == 1)
      {
        avgMv[0] += gpmAffMrgCtx.mvFieldNeighbours[i * 2][0].mv;
        avgMv[1] += gpmAffMrgCtx.mvFieldNeighbours[i * 2][1].mv;
        avgMv[2] += gpmAffMrgCtx.mvFieldNeighbours[i * 2][2].mv;
        if (firstAvailRefIdx == -1)
        {
          firstAvailRefIdx = gpmAffMrgCtx.mvFieldNeighbours[i * 2][0].refIdx;
        }
        cnt++;
      }
    }
    if (cnt == 2)
    {
      roundAffineMv(avgMv[0].hor, avgMv[0].ver, 1);
      roundAffineMv(avgMv[1].hor, avgMv[1].ver, 1);
      roundAffineMv(avgMv[2].hor, avgMv[2].ver, 1);
      gpmAffMrgCtx.interDirNeighbours[gpmAffMrgCtx.numValidMergeCand] = 1;
      gpmAffMrgCtx.mvFieldNeighbours[gpmAffMrgCtx.numValidMergeCand * 2][0].setMvField(avgMv[0], firstAvailRefIdx);
      gpmAffMrgCtx.mvFieldNeighbours[gpmAffMrgCtx.numValidMergeCand * 2][1].setMvField(avgMv[1], firstAvailRefIdx);
      gpmAffMrgCtx.mvFieldNeighbours[gpmAffMrgCtx.numValidMergeCand * 2][2].setMvField(avgMv[2], firstAvailRefIdx);
      gpmAffMrgCtx.mvFieldNeighbours[gpmAffMrgCtx.numValidMergeCand * 2 + 1][0].setMvField(Mv(0, 0), NOT_VALID);
      gpmAffMrgCtx.mvFieldNeighbours[gpmAffMrgCtx.numValidMergeCand * 2 + 1][1].setMvField(Mv(0, 0), NOT_VALID);
      gpmAffMrgCtx.mvFieldNeighbours[gpmAffMrgCtx.numValidMergeCand * 2 + 1][2].setMvField(Mv(0, 0), NOT_VALID);

      if (!gpmAffMrgCtx.xCheckSimilarMotion(gpmAffMrgCtx.numValidMergeCand))
      {
        gpmAffMrgCtx.numValidMergeCand++;
      }
      if (gpmAffMrgCtx.numValidMergeCand == maxNumGeoMergeCand)
      {
        return;
      }
    }

    // add one L1 cand by averaging the first two available L1 candidates
    cnt = 0;
    firstAvailRefIdx = -1;
    avgMv[0].setZero();
    avgMv[1].setZero();
    avgMv[2].setZero();
    for (int i = 0; i < gpmAffMrgCtx.numValidMergeCand; i++)
    {
      if (cnt == 2)
      {
        break;
      }
      if (gpmAffMrgCtx.interDirNeighbours[i] == 2)
      {
        avgMv[0] += gpmAffMrgCtx.mvFieldNeighbours[i * 2 + 1][0].mv;
        avgMv[1] += gpmAffMrgCtx.mvFieldNeighbours[i * 2 + 1][1].mv;
        avgMv[2] += gpmAffMrgCtx.mvFieldNeighbours[i * 2 + 1][2].mv;

        if (firstAvailRefIdx == -1)
        {
          firstAvailRefIdx = gpmAffMrgCtx.mvFieldNeighbours[i * 2 + 1][0].refIdx;
        }
        cnt++;
      }
    }
    if (cnt == 2)
    {
      roundAffineMv(avgMv[0].hor, avgMv[0].ver, 1);
      roundAffineMv(avgMv[1].hor, avgMv[1].ver, 1);
      roundAffineMv(avgMv[2].hor, avgMv[2].ver, 1);

      gpmAffMrgCtx.interDirNeighbours[gpmAffMrgCtx.numValidMergeCand] = 2;
      gpmAffMrgCtx.mvFieldNeighbours[gpmAffMrgCtx.numValidMergeCand * 2 + 1][0].setMvField(avgMv[0], firstAvailRefIdx);
      gpmAffMrgCtx.mvFieldNeighbours[gpmAffMrgCtx.numValidMergeCand * 2 + 1][1].setMvField(avgMv[1], firstAvailRefIdx);
      gpmAffMrgCtx.mvFieldNeighbours[gpmAffMrgCtx.numValidMergeCand * 2 + 1][2].setMvField(avgMv[2], firstAvailRefIdx);

      gpmAffMrgCtx.mvFieldNeighbours[gpmAffMrgCtx.numValidMergeCand * 2][0].setMvField(Mv(0, 0), NOT_VALID);
      gpmAffMrgCtx.mvFieldNeighbours[gpmAffMrgCtx.numValidMergeCand * 2][1].setMvField(Mv(0, 0), NOT_VALID);
      gpmAffMrgCtx.mvFieldNeighbours[gpmAffMrgCtx.numValidMergeCand * 2][2].setMvField(Mv(0, 0), NOT_VALID);

      if (!gpmAffMrgCtx.xCheckSimilarMotion(gpmAffMrgCtx.numValidMergeCand))
      {
        gpmAffMrgCtx.numValidMergeCand++;
      }

      if (gpmAffMrgCtx.numValidMergeCand == maxNumGeoMergeCand)
      {
        return;
      }
    }
  }
  if (gpmAffMrgCtx.numValidMergeCand < maxNumGeoMergeCand)
  {
    const Slice& slice = *pu.cs->slice;
#if JVET_Y0065_GPM_INTRA
    int         iNumRefIdx = pu.cs->slice->isInterP() ? slice.getNumRefIdx(REF_PIC_LIST_0) : std::min(slice.getNumRefIdx(REF_PIC_LIST_0), slice.getNumRefIdx(REF_PIC_LIST_1));
#else
    int         iNumRefIdx = std::min(slice.getNumRefIdx(REF_PIC_LIST_0), slice.getNumRefIdx(REF_PIC_LIST_1));
#endif

    int r = 0;
    int refcnt = 0;

    for (int32_t i = gpmAffMrgCtx.numValidMergeCand; i < maxNumGeoMergeCand; i++)
    {
#if JVET_Y0065_GPM_INTRA
      int parity = pu.cs->slice->isInterP() ? 0 : (i & 1);
#else
      int parity = i & 1;
#endif
      {
        gpmAffMrgCtx.interDirNeighbours[gpmAffMrgCtx.numValidMergeCand] = 1 + parity;

        for (int cpmvIdx = 0; cpmvIdx < 3; cpmvIdx++)
        {
          gpmAffMrgCtx.mvFieldNeighbours[(gpmAffMrgCtx.numValidMergeCand << 1) + !parity][cpmvIdx].mv = Mv(0, 0);
          gpmAffMrgCtx.mvFieldNeighbours[(gpmAffMrgCtx.numValidMergeCand << 1) + parity][cpmvIdx].mv = Mv(0, 0);
          gpmAffMrgCtx.mvFieldNeighbours[(gpmAffMrgCtx.numValidMergeCand << 1) + !parity][cpmvIdx].refIdx = -1;
          gpmAffMrgCtx.mvFieldNeighbours[(gpmAffMrgCtx.numValidMergeCand << 1) + parity][cpmvIdx].refIdx = r;
        }
        if (refcnt == iNumRefIdx - 1)
        {
          r = 0;
        }
        else
        {
          ++r;
          ++refcnt;
        }

        gpmAffMrgCtx.numValidMergeCand++;

        if (gpmAffMrgCtx.numValidMergeCand == maxNumGeoMergeCand)
        {
          return;
        }
      }
    }
  }
}

#endif

#if JVET_AG0058_EIP
bool PU::isEIP(const PredictionUnit& pu, const ChannelType& chType)
{
  return pu.cu->eipFlag && chType == CH_L;
}

Position getRecoLinesEIP(const CodingUnit& cu, const ComponentID compId)
{
  const int numRecoLines = std::min(cu.blocks[compId].width, cu.blocks[compId].height);
  const int above = std::min(cu.blocks[compId].y - EIP_FILTER_SIZE, numRecoLines);
  const int left = std::min(cu.blocks[compId].x - EIP_FILTER_SIZE, numRecoLines);
  CHECK(above < 1 || left < 1, "no reconstruction lines above or left.");

  return Position(above, left);
}

bool getAllowedEip(const CodingUnit& cu, const ComponentID compId)
{
  if (!cu.cs->sps->getUseEip())
  {
    return false;
  }

  const uint32_t width = cu.blocks[compId].width;
  const uint32_t height = cu.blocks[compId].height;

  if (width > MAX_EIP_SIZE || height > MAX_EIP_SIZE)
  {
    return false;
  }

  if (cu.blocks[compId].area() < 64)
  {
    return false;
  }

  const int numAboveAvail = cu.blocks[compId].y;
  const int numLeftAvail = cu.blocks[compId].x;
  const int tplSize = std::min(cu.blocks[compId].height, cu.blocks[compId].width) + EIP_FILTER_SIZE;
  if(numAboveAvail < tplSize || numLeftAvail < tplSize)
  {
    return false;
  }

  return true;
}

bool getAllowedEipMerge(const CodingUnit& cu, const ComponentID compId)
{
  if (!cu.cs->sps->getUseEip())
  {
    return false;
  }

  const uint32_t width = cu.blocks[compId].width;
  const uint32_t height = cu.blocks[compId].height;

  if (width > MAX_EIP_SIZE || height > MAX_EIP_SIZE)
  {
    return false;
  }

  const int numAboveAvail = cu.blocks[compId].y;
  const int numLeftAvail = cu.blocks[compId].x;

  if ((numAboveAvail >= (EIP_FILTER_SIZE + EIP_TPL_SIZE)) && (numLeftAvail >= (EIP_FILTER_SIZE + EIP_TPL_SIZE)))
  {
    return true;
  }

  return false;
}

int getAllowedCurEip(const CodingUnit &cu, const ComponentID compId, static_vector<EIPInfo, NUM_DERIVED_EIP>& eipInfoList)
{ 
  const int numOfCombEIP[4][4] = 
  {
    { 3, 3, 3, 2 },
    { 3, 5, 5, 5 },
    { 3, 5, 9, 9 },
    { 2, 5, 9, 9 },
  };
  const int log2Wm2 = floorLog2(cu.blocks[compId].width)  - 2;
  const int log2Hm2 = floorLog2(cu.blocks[compId].height) - 2;
  for (int i = 0; i < numOfCombEIP[log2Wm2][log2Hm2]; i++)
  {
    eipInfoList.push_back(g_eipInfoLut[log2Wm2][log2Hm2][i]);
  }

  return int(eipInfoList.size());
}

void CU::saveModelsInHEIP(const CodingUnit &cu)
{
  if (!cu.Y().valid())
  {
    return;
  }

  CodingStructure      &cs = *cu.cs;
  if (cu.eipFlag)
  {
#if JVET_Z0118_GDR   
    if (cu.cs->isGdrEnabled() && cu.cs->isClean(cu))
    {      
      cs.addEipToLut(cs.eipLut.lutEip1, cu.eipModel, -1);
    }

    cs.addEipToLut(cs.eipLut.lutEip0, cu.eipModel, -1);
#else
    cs.addEipToLut(cs.eipLut.lutEip, cu.eipModel, -1);
#endif
  }
}
#endif
