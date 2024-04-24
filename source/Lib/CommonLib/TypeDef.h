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

/** \file     TypeDef.h
    \brief    Define macros, basic types, new types and enumerations
*/

#ifndef __TYPEDEF__
#define __TYPEDEF__

#ifndef __COMMONDEF__
#error Include CommonDef.h not TypeDef.h
#endif

#include <vector>
#include <utility>
#include <sstream>
#include <cstddef>
#include <cstring>
#include <assert.h>
#include <cassert>
#include <cstdint>



#define BASE_ENCODER                                      1
#define BASE_NORMATIVE                                    1
#define TOOLS                                             1

#if BASE_ENCODER
// Lossy encoder speedups
#define AFFINE_ENC_OPT                                    1 // Affine encoder optimization
#define AMVR_ENC_OPT                                      1 // Encoder optimization for AMVR
#define MERGE_ENC_OPT                                     1 // Encoder optimization for merge modes. Regular merge, subblock based merge, CIIP and MMVD share the same full RD pool
#define ALF_SAO_TRUE_ORG                                  1 // using true original samples for SAO and ALF optimization
#define REMOVE_PCM                                        1 // Remove PCM related code for memory reduction and speedup
#define JVET_Y0152_TT_ENC_SPEEDUP                         1 // TT encoding speedup
#define JVET_AE0057_MTT_ET                                1 // JVET-AE0057: MTT early termination of 64x64 luma CU based on no-split RD cost 
#define JVET_AF0177_ALF_COV_FLOAT                         1 // JVET-AF0177: Use float for ALF and CCALF covariance matrix


// Software optimization
#define JVET_X0144_MAX_MTT_DEPTH_TID                      1 // JVET-X0144: max MTT hierarchy depth set by temporal ID
#if JVET_X0144_MAX_MTT_DEPTH_TID
#define JVET_AA0098_MTT_DEPTH_TID_BY_QP                   1 // JVET-AA0098: MTT depth based on QP
#endif
#define JVET_X0049_BDMVR_SW_OPT                           1 // JVET-X0049: software optimization for BDMVR (lossless)
#define INTRA_TRANS_ENC_OPT                               1 // JVET-Y0141: Software optimization, including TIMD/DIMD/MTS/LFNS encoder fast algorithm, SIMD implementation and CM initial value retraining 
#define JVET_AA0129_INTERHASH_OBMCOFF_RD                  1 // JVET-AA0129: improved encoder RDO of inter-hash based ME considering OBMC off

// SIMD optimizations
#define MCIF_SIMD_NEW                                     1 // SIMD for interpolation
#define DIST_SSE_ENABLE                                   1 // Enable SIMD for SSE
#define TRANSFORM_SIMD_OPT                                1 // SIMD optimization for transform
#endif

#if BASE_NORMATIVE
// Partition
#define CTU_256                                           1 // Add CTU 256
#if CTU_256
#define TU_256                                            1 // Add TU 256, removed MTS zero out
#endif
#define REMOVE_VPDU                                       1 // Remove VPDU restriction on BT/TT splitting
#if TU_256
#define LMCS_CHROMA_CALC_CU                               1 // Derive chroma LMCS parameter based on neighbor CUs. Needed by VPDU removal and 128x128 transform.
#endif

//-- intra
#define INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS             1 // Enable 2xN and Nx2 block by removing SCIPU constraints
#define CCLM_LATENCY_RESTRICTION_RMV                      1 // remove the latency between luma and chroma restriction of CCLM
#define LMS_LINEAR_MODEL                                  1 // LMS for parameters derivation of CCLM and MMLM mode, Remove constraint in derivation of neighbouring samples

//-- inter
#define CIIP_RM_BLOCK_SIZE_CONSTRAINTS                    1 // Remove the 64x64 restriction and enable 8x4/4x8 block for CIIP
#define BDOF_RM_CONSTRAINTS                               1
#define INTER_RM_SIZE_CONSTRAINTS                         1 // Remove size constraints for inter block
#define AFFINE_RM_CONSTRAINTS_AND_OPT                     1 // Remove affine constraints and optimization

// Loop filters
#define DB_PARAM_TID                                      1 // Adjust DB parameters based on temporal ID

#define CONVERT_NUM_TU_SPLITS_TO_CFG                      1 // Convert number of TU splits to config parameter to lower memory
#define DUMP_BEFORE_INLOOP                                1 // Dump reconstructed YUV before inloop filters, controlled by config parameter
#endif


/// Tools
#if TOOLS
// Intra
#define MMLM                                              1 // Add three MMLM modes
#define GRAD_PDPC                                         1 // Gradient PDPC extension for angular-intra modes for luma.
#define INTRA_6TAP                                        1 // 6TapCubic + 6 TapGaussian + left side 4 tap weak filtering for intra.
#define SECONDARY_MPM                                     1 // Primary MPM and secondary MPM: Add neighbouring modes into MPMs from positions AR, BL, AL, derived modes
#define ENABLE_DIMD                                       1 // Decoder side intra mode derivation
#if ENABLE_DIMD
#define JVET_V0087_DIMD_NO_ISP                            1 // JVET-V0087: Disallow combination of DIMD and ISP
#define JVET_X0124_TMP_SIGNAL                             1 // JVET-X0124: Cleanup on signalling of intra template matching
#endif

#define JVET_V0130_INTRA_TMP                              1 // JVET-V0130: Template matching prediction
#if JVET_V0130_INTRA_TMP
#define JVET_AB0130_ITMP_SAMPLING                         1 // JVET-AB0130: Speedup of IntraTMP by search range sampling and refinement
#if JVET_AB0130_ITMP_SAMPLING
#define TMP_FAST_ENC                                      1 // Encoder-only modification
#endif
#define JVET_W0069_TMP_BOUNDARY                           1 // JVET-W0069: Boundary handling for TMP
#define JVET_AB0061_ITMP_BV_FOR_IBC                       1 // JVET-AB0061: Storing IntraTMP BV for IBC BV prediction
#define JVET_AD0086_ENHANCED_INTRA_TMP                    1 // JVET-AD0086: Enhanced intra TMP
#define JVET_AE0077_EXT_INTRATMP                          1 // JVET-AE0077: Extended search areas for intraTMP
#define JVET_AF0079_STORING_INTRATMP                      1 // JVET-AF0079: Storing fractional-pel intraTMP BV
#define JVET_AG0136_INTRA_TMP_LIC                         1 // JVET-AG0136: Intra TMP extension to LIC
#define JVET_AG0151_INTRA_TMP_MERGE_MODE                  1 // JVET-AG0151: Intra TMP with merge candidates
#define JVET_AG0152_SGPM_ITMP_IBC                         1 // JVET-AG0152: SGPM with Intra TMP and IBC
#define JVET_AG0146_DIMD_ITMP_IBC                         1 // JVET-AG0146: DIMD with Intra TMP and IBC
#endif

#define JVET_W0123_TIMD_FUSION                            1 // JVET-W0123: Template based intra mode derivation and fusion
#if JVET_W0123_TIMD_FUSION
#define JVET_X0148_TIMD_PDPC                              1 // JVET-X0148: PDPC handling for TIMD
#define JVET_AG0092_ENHANCED_TIMD_FUSION                  1 // JVET-AG0092: TIMD fusion with non-angular predictor
#define JVET_AG0128_REF_LINE_OPT_TIMD_FUSION              1 // JVET-AG0128: TIMD fusion reference line determination

#if ENABLE_DIMD
#define JVET_AC0098_LOC_DEP_DIMD                          1 // JVET-AC0098: Location-dependent Decoder-side Intra Mode Derivation
#endif

#endif
#if ENABLE_DIMD || JVET_W0123_TIMD_FUSION
#define JVET_X0149_TIMD_DIMD_LUT                          1 // JVET-X0149: LUT-based derivation of DIMD and TIMD
#endif

#define JVET_Y0116_EXTENDED_MRL_LIST                      1 // JVET-Y0116: Extended MRL Candidate List
#define JVET_Z0050_DIMD_CHROMA_FUSION                     1 // JVET-Z0050: DIMD chroma mode and fusion of chroma intra prediction modes
#if JVET_Z0050_DIMD_CHROMA_FUSION
#define JVET_AC0119_LM_CHROMA_FUSION                      1 // JVET-AC0119: On Chroma Fusion improvement
#endif
#define JVET_Z0050_CCLM_SLOPE                             1 // JVET-Z0050: CCLM with slope adjustments
#define JVET_AA0057_CCCM                                  1 // JVET-AA0057: Convolutional cross-component model (CCCM)
#if JVET_AA0057_CCCM
#define JVET_AB0143_CCCM_TS                               1 // JVET-AB0143: CCCM template selection
#define JVET_AC0147_CCCM_NO_SUBSAMPLING                   1 // JVET-AC0147: Subsampling is not applied to CCCM
#define JVET_AC0054_GLCCCM                                1 // JVET-AC0054: Gradient and location based CCCM
#define JVET_AD0202_CCCM_MDF                              1 // JVET-AD0202: CCCM with multiple downsampling filters
#define JVET_AE0100_BVGCCCM                               1 // JVET-AE0100: Block vector guided CCCM
#endif
#define JVET_AA0126_GLM                                   1 // JVET-AA0126: Gradient linear model
#if JVET_AA0126_GLM
#define JVET_AB0092_GLM_WITH_LUMA                         1 // JVET-AB0092: Gradient linear model with luma value
#endif
#if JVET_AA0057_CCCM || JVET_AB0092_GLM_WITH_LUMA
#define JVET_AB0174_CCCM_DIV_FREE                         1 // JVET-AB0174: CCCM with division free operation
#define JVET_AC0053_GAUSSIAN_SOLVER                       1 // JVET-AC0053: Filter parameters using Gaussian elimination steps
#endif
#define JVET_AB0155_SGPM                                  1 // JVET-AB0155: Spatial geometric partitioning mode
#if JVET_AB0155_SGPM
#define JVET_AC0189_SGPM_NO_BLENDING                      1 // JVET-AC0189: Allow no blending for SGPM
#endif
#define JVET_AB0157_TMRL                                  1 // JVET-AB0157: Template-based multiple reference line intra prediction
#if JVET_AB0157_TMRL
#define JVET_AD0082_TMRL_CONFIG                           1 // JVET-AD0082: a configuration option for TMRL
#define JVET_AD0085_TMRL_EXTENSION                        1 // JVET-AD0085: TMRL angular extension and intra candidate list modifications
#endif
#define JVET_AB0157_INTRA_FUSION                          1 // JVET-AB0157: Intra prediction fusion
#define JVET_AC0094_REF_SAMPLES_OPT                       1 // JVET-AC0094: Optimizing the use of reference samples
#define JVET_AC0105_DIRECTIONAL_PLANAR                    1 // JVET-AC0105: Directional planar
#define JVET_AD0184_REMOVAL_OF_DIVISION_OPERATIONS        1 // JVET-AD0184: Removal of division operations
#define JVET_AD0085_MPM_SORTING                           1 // JVET-AD0085: Template-based intra MPM list construction
#define JVET_AD0188_CCP_MERGE                             1 // JVET-AD0188: Non-local cross-component prediction and cross-component merge mode
#if JVET_AD0188_CCP_MERGE
#define JVET_AE0097_RM_OFFSET_UPDATE_IN_CCP_MERGE         1 // JVET-AE0097: Remove offset update process in CCP merge
#endif
#define JVET_AD0120_LBCCP                                 1 // JVET-AD0120: Local-Boosting Cross-Component Prediction, wherein the template part is controlled by CCCM SPS
#define JVET_AE0043_CCP_MERGE_TEMPORAL                    1 // JVET-AE0043: Cross-component merge mode with temporal candidates
#define JVET_AG0058_EIP                                   1 // JVET-AG0058: Extrapolation filter-based intra prediction mode
#define JVET_AG0154_DECODER_DERIVED_CCP_FUSION            1 // JVET-AG0154: Decoder derived CCP mode with fusion candidates
#define JVET_AG0059_CCP_MERGE_ENHANCEMENT                 1 // JVET-AG0059: Enhancements on CCP merge for chroma intra coding

//IBC
#define JVET_Y0058_IBC_LIST_MODIFY                        1 // JVET-Y0058: Modifications of IBC merge/AMVP list construction, ARMC-TM-IBC part is included under JVET_W0090_ARMC_TM
#define JVET_Z0075_IBC_HMVP_ENLARGE                       1 // JVET-Z0075: Enlarged HMVP table for IBC
#define JVET_Z0084_IBC_TM                                 1 // JVET-Z0084: Add template matching in IBC modes. [0 = disable, 1 (default) = enable for IBC-AMVP and IBC-MRG, 2 = enable for IBC-AMVP, 3 = enable for IBC-MRG]
#if JVET_Z0084_IBC_TM
#define IBC_TM_AMVP                                       ((JVET_Z0084_IBC_TM) == 1 || (JVET_Z0084_IBC_TM) == 2) // Add template matching to IBC to refine IBC AMVP candidates
#define IBC_TM_MRG                                        ((JVET_Z0084_IBC_TM) == 1 || (JVET_Z0084_IBC_TM) == 3) // Add template matching to IBC to refine IBC merge candidates
#endif
#define JVET_Z0131_IBC_BVD_BINARIZATION                   1 // JVET-Z0131: Block vector difference binarization
#define JVET_Z0153_IBC_EXT_REF                            1 // JVET-Z0153: Extend reference area for IBC
#define JVET_Z0160_IBC_ZERO_PADDING                       1 // JVET-Z0160: Replacement of zero-padding candidates
#define JVET_AA0106_IBCBUF_CTU256                         1 // JVET-AA0106: Adjust IBC reference area to 2*128 rows above the current CTU
#define JVET_AA0061_IBC_MBVD                              1 // JVET-AA0061: IBC merge mode with block vector differences
#if JVET_AA0061_IBC_MBVD
#define JVET_AE0169_IBC_MBVD_LIST_DERIVATION              1 // JVET-AE0169: IBC MBVD list derivation
#endif
#define JVET_AA0070_RRIBC                                 1 // JVET-AA0070: Reconstruction-Reordered IBC
#define JVET_AC0112_IBC_CIIP                              1 // JVET-AC0112: Combined IBC and intra prediction (IBC-CIIP)
#define JVET_AC0112_IBC_GPM                               1 // JVET-AC0112: IBC with geometry partitioning mode (IBC-GPM)
#define JVET_AC0112_IBC_LIC                               1 // JVET-AC0112: IBC with local illumination compensation (IBC-LIC)
#if JVET_AC0112_IBC_LIC
#define JVET_AE0078_IBC_LIC_EXTENSION                     1 // JVET-AE0078: IBC-LIC extension
#endif
#define JVET_AC0104_IBC_BVD_PREDICTION                    1 // JVET-AC0104: IBC block vector difference prediction (part of JVET-AC0113 Test 3.5a) 
#define JVET_AC0060_IBC_BVP_CLUSTER_RRIBC_BVD_SIGN_DERIV  1 // JVET-AC0060: IBC BVP candidates clustering and BVD sign derivation for BV with one zero component (part of JVET-AC0113 Test 3.5a) 
#define JVET_AC0071_DBV                                   1 // JVET-AC0071: Direct block vector mode for chroma prediction
#define JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS   1 // JVET-AD0208: IBC adaptation for camera-captured contents and IBC extension to fractional-pel BV
#define JVET_AE0084_IBC_LIC_INHERITANCE                   1 // JVET-AE0084: Harmonization of IBC HMVP and IBC-LIC
#define JVET_AE0159_FIBC                                  1 // JVET-AE0159: Filtered Intra Block Copy (FIBC)
#define JVET_AE0169_GPM_IBC_IBC                           1 // JVET-AE0169: Bi-predictive IBC GPM
#define JVET_AE0169_BIPREDICTIVE_IBC                      1 // JVET-AE0169: IBC BVP-merge and bi-predictive IBC merge

#define JVET_AE0174_NONINTER_TM_TOOLS_CONTROL             1 // JVET-AE0174: Add non-inter TM sps flag to control whether template matching is used for non-inter (Intra and IBC) tools
#define JVET_AE0094_IBC_NONADJACENT_SPATIAL_CANDIDATES    1 // JVET-AE0094: IBC with non-adjacent spatial candidates
#define JVET_AG0091_ARBVP                                 1 // JVET-AG0091: Auto-relocated block vector prediction

#if JVET_AC0071_DBV && JVET_V0130_INTRA_TMP
#define JVET_AF0066_ENABLE_DBV_4_SINGLE_TREE              1 // JVET-AF0066: Enable DBV mode in single tree configuration
#endif

// Inter
#define CIIP_PDPC                                         1 // Apply pdpc to megre prediction as a new CIIP mode (CIIP_PDPC) additional to CIIP mode
#define JVET_X0090_CIIP_FIX                               1 // JVET-X0090: combination of CIIP, OBMC and LMCS
#define SAMPLE_BASED_BDOF                                 1 // Sample based BDOF
#define MULTI_PASS_DMVR                                   1 // Multi-pass DMVR
#define AFFINE_MMVD                                       1 // Add MMVD to affine merge mode
#define INTER_LIC                                         1 // Add LIC to non-subblock inter
#if INTER_LIC
#define JVET_AD0213_LIC_IMP                               1 // JVET-AD0213: bi-predictive LIC and the combination of LIC and OBMC
#define JVET_AF0128_LIC_MERGE_TM                          1 // JVET-AF0128: LIC flag derivation for merge candidates with template costs
#define JVET_AG0276_NLIC                                  1 // JVET-AG0276: non-local LIC
#define JVET_AG0276_LIC_FLAG_SIGNALING                    1 // JVET-AG0276: signal LIC flag for merge mode
#define JVET_AG0276_LIC_BDOF_BDMVR                        1 // JVET-AG0276: LIC BDOF and BDMVR
#define JVET_AG0276_LIC_SLOPE_ADJUST                      1 // JVET-AG0276: LIC with slope adjustment
#endif
#define NON_ADJACENT_MRG_CAND                             1 // Add non-adjacent merge candidates
#define MULTI_HYP_PRED                                    1 // Multiple hypothesis prediction
#if MULTI_HYP_PRED
#define JVET_Z0127_SPS_MHP_MAX_MRG_CAND                   1 // JVET-Z0127: Signal number of MHP candidates
#endif
#define IF_12TAP                                          1 // 12-tap IF
#define JVET_AB0082                                       1 // JVET-AB0082: AMRC fix and fix to use 12-tap MC for upscaling with flexibility to select other filters for upscaling
#define JVET_Z0117_CHROMA_IF                              1 // JVET-Z0117: 6-tap interpolation filter for chroma MC
#define JVET_AA0042_RPR_FILTERS                           1 // JVET-AA0042: luma 12-tap non-affine 10-tap affine, chroma 6-tap      
#define ENABLE_OBMC                                       1 // Enable Overlapped Block Motion Compensation
#if ENABLE_OBMC
#define JVET_AC0335_CONTENT_ADAPTIVE_OBMC_ENABLING        1 // JVET-AC0335: Content adaptive OBMC enabling
#define JVET_AD0193_ADAPTIVE_OBMC_CONTROL                 1 // JVET-AD0193: Adaptive OBMC control
#endif

#if JVET_X0049_BDMVR_SW_OPT
#define JVET_X0049_ADAPT_DMVR                             1 // JVET-X0049: Adaptive DMVR
#endif
#define JVET_X0056_DMVD_EARLY_TERMINATION                 1 // JVET-X0056: Early termination for DMVR and TM
#define JVET_X0083_BM_AMVP_MERGE_MODE                     1 // JVET-X0083: AMVP-merge mode
#if JVET_X0083_BM_AMVP_MERGE_MODE
#define JVET_Y0129_MVD_SIGNAL_AMVP_MERGE_MODE             1 // JVET-Y0129: MVD signalling for AMVP-merge mode
#define JVET_Z0085_AMVPMERGE_DMVD_OFF                     1 // JVET-Z0085: Enabling amvpMerge when DMVD is off
#if JVET_Z0085_AMVPMERGE_DMVD_OFF
#define JVET_AA0124_AMVPMERGE_DMVD_OFF_RPR_ON             1
#endif
#define JVET_AB0078_AMVPMERGE_LDB                         1 // JVET-AB0078: Enabling AmvpMerge for low delay picture
#endif
#define JVET_Y0089_DMVR_BCW                               1 // JVET-Y0089: DMVR with BCW enabled
#define JVET_Y0065_GPM_INTRA                              1 // JVET-Y0065: Intra prediction for GPM
#define JVET_Z0136_OOB                                    1 // JVET-Z0136 test2.2a: Enhanced bi-prediction with out-of-boundary prediction samples
#define JVET_Z0083_PARSINGERROR_FIX                       1 // JVET-Z0083: Fix on MHP parsing condition
#define JVET_Z0139_HIST_AFF                               1 // JVET-Z0139: Affine HMVP 
#define JVET_Z0139_NA_AFF                                 1 // JVET-Z0139: Constructed non-adjacent spatial neighbors for affine mode
#define JVET_AA0058_GPM_ADAPTIVE_BLENDING                 1 // JVET-AA0058: GPM adaptive blending
#define GPM_CURVE                                         0 // GPM curve split mode
#define GPM_BLEND                                         1 // GPM advanced blending
#define JVET_AA0146_WRAP_AROUND_FIX                       1 // JVET-AA0146: bugfix&cleanup for wrap around motion compensation
#define JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION          1 // JVET-AA0107 Regression based affine merge candidate derivation
#if JVET_AA0107_RMVF_AFFINE_MERGE_DERIVATION
#define JVET_AB0189_RMVF_BITLENGTH_CONTROL                1 // JVET-AB0189: Bit length control over regression based affine merge derivation method
#if JVET_AB0189_RMVF_BITLENGTH_CONTROL
#define JVET_AA0107_RMVF_AFFINE_OVERFLOW_FIX              0 // Overflow fix
#else
#define JVET_AA0107_RMVF_AFFINE_OVERFLOW_FIX              1 // Overflow fix
#endif
#define JVET_AA0128_AFFINE_MERGE_CTX_INC                  1 // JVET-AA0128 test b: Increased number of CABAC contexts of affine merge index
#endif
#define JVET_AA0096_MC_BOUNDARY_PADDING                   1 // JVET-AA0096: motion compensated picture boundary padding
#define JVET_AB0112_AFFINE_DMVR                           1 // JVET-AB0112: DMVR (baseMV refinement) for affine
#define JVET_AC0144_AFFINE_DMVR_REGRESSION                1 // JVET-AC0144: DMVR for affine with regression refinement
#define JVET_AD0182_AFFINE_DMVR_PLUS_EXTENSIONS           1 // JVET-AD0182: Affine parameter refinement and adaptive affine DMVR mode extension
#define JVET_AC0158_PIXEL_AFFINE_MC                       1 // JVET-AC0158: Pixel based affine motion compensation
#define JVET_AD0195_HIGH_PRECISION_BDOF_CORE              1 // JVET-AD0195: High-Precision MV Refinement for BDOF
#define JVET_AE0091_ITERATIVE_BDOF                        1 // JVET-AE0091: Iterative BDOF part of JVET-AE0091
#define JVET_AD0123_REF_PICTURE_PADDING_FOR_GDR           1 // JVET-AD0123: Reference picture padding for GDR
#define JVET_AE0059_INTER_CCCM                            1 // JVET-AE0059: Convolutional cross-component model for inter prediction
#define JVET_AE0046_BI_GPM                                1 // JVET-AE0046: Allow bi-MVs for inter partitions in GPM
#if IF_12TAP && JVET_AA0042_RPR_FILTERS && JVET_Z0117_CHROMA_IF
#define JVET_AE0150_SMALL_SCALE_RPR_FILTERS               1 // RPR filters and thresholds for scale factors 1.1x to 1.35x
#endif
#define JVET_AF0190_RPR_TMP_REORDER_LIC                   1 // JVET-AF0190: enabling template-based reordering tools and LIC for scaled pictures in the RPR
#define JVET_AF0073_INTER_CCP_MERGE                       1 // JVET-AF0073: Cross-component prediction merge mode for inter prediction
#define JVET_AF0159_AFFINE_SUBPU_BDOF_REFINEMENT          1 // JVET-AF0159: Affine subblock BDOF refinement
#define JVET_AF0057                                       1 // JVET-AF0057: Encoder only. DMVR with robust MV derivation.
#define JVET_AG0112_REGRESSION_BASED_GPM_BLENDING         1 // JVET-AG0112: Regression-based GPM blending
#define JVET_AG0135_AFFINE_CIIP                           1 // JVET-AG0135: CIIP with affine prediction 
#define JVET_AG0164_AFFINE_GPM                            1 // JVET-AG0164: GPM with affine prediction
#define JVET_AG0098_AMVP_WITH_SBTMVP                      1 // JVET-AG0098: AMVP with SbTMVP mode
#define JVET_AG0067_DMVR_EXTENSIONS                       1 // JVET-AG0067: On DMVR Extensions

// Inter template matching tools
#define ENABLE_INTER_TEMPLATE_MATCHING                    1 // It controls whether template matching is enabled for inter prediction
#if ENABLE_INTER_TEMPLATE_MATCHING
#define TM_AMVP                                           1 // Add template matching to non-subblock inter to refine regular AMVP candidates
#define TM_MRG                                            1 // Add template matching to non-subblock inter to refine regular merge candidates
#define JVET_W0090_ARMC_TM                                1 // JVET-W0090: Adaptive reordering of merge candidates with template matching
#define JVET_Z0056_GPM_SPLIT_MODE_REORDERING              1 // JVET-Z0056: Template matching based reordering for GPM split modes
#if ENABLE_OBMC
#define JVET_Z0061_TM_OBMC                                1 // JVET-Z0061: Template matching based OBMC
#endif
#define JVET_AA0132_CONFIGURABLE_TM_TOOLS                 1 // JVET-AA0132: Configuration parameters and SPS flags for template matching tools
#endif
#define JVET_W0097_GPM_MMVD_TM                            1 // JVET-W0097: GPM-MMVD and GPM-TM, GPM-TM part is controlled by TM_MRG
#define JVET_X0141_CIIP_TIMD_TM                           1 // JVET-X0141: CIIP with TIMD and TM merge, CIIP-TM part is controlled by TM_MRG, and CIIP-TIMD part is controlled by JVET_W0123_TIMD_FUSION
#define JVET_Y0134_TMVP_NAMVP_CAND_REORDERING             1 // JVET-Y0134: MV candidate reordering for TMVP and NAMVP types (controlled by JVET_W0090_ARMC_TM), and reference picture selection for TMVP 
#define JVET_AF0163_TM_SUBBLOCK_REFINEMENT                1 // JVET-AF0163: TM based subblock motion refinement
#if JVET_W0090_ARMC_TM
#define JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED            1 // JVET-Y0067: TM based reordering for MMVD and affine MMVD and MVD sign prediction
#define JVET_AD0140_MVD_PREDICTION                        1 // JVET-AD0140: MVD (sign and suffix) prediction
#if JVET_Y0067_ENHANCED_MMVD_MVD_SIGN_PRED
#define JVET_AA0093_ENHANCED_MMVD_EXTENSION               1 // JVET-AA0093: Test 2.3 part: Extension on TM based reordering for MMVD and affine MMVD
#endif
#define JVET_Z0102_NO_ARMC_FOR_ZERO_CAND                  1 // JVET-Z0102: No ARMC for the zero candidates of regular, TM and BM merge modes
#define JVET_Z0054_BLK_REF_PIC_REORDER                    1 // JVET-Z0054: Block level TM based reordering of reference pictures
#define JVET_AA0093_REFINED_MOTION_FOR_ARMC               1 // JVET-AA0093: Refined motion for ARMC
#define JVET_AA0093_DIVERSITY_CRITERION_FOR_ARMC          1 // JVET-AA0093: Diversity criterion for ARMC reordering
#define JVET_AB0079_TM_BCW_MRG                            1 // JVET-AB0079: Template matching based BCW index derivation for merge mode with positive weights only
#define JVET_AC0185_ENHANCED_TEMPORAL_MOTION_DERIVATION   1 // JVET-AC0185: Enhanced temporal motion information derivation
#define JVET_AE0091_HIGH_ACCURACY_TEMPLATE_MATCHING       1 // JVET-AE0091: High-Accuracy template matching
#endif

// Transform and coefficient coding
#define TCQ_8STATES                                       1
#define JVET_W0119_LFNST_EXTENSION                        1 // JVET-W0119: LFNST extension with large kernel
#if JVET_W0119_LFNST_EXTENSION
#define EXTENDED_LFNST                                    0
#else
#define EXTENDED_LFNST                                    1 // Extended LFNST
#endif
#define SIGN_PREDICTION                                   1 // Transform coefficients sign prediction
#if SIGN_PREDICTION
#define JVET_Y0141_SIGN_PRED_IMPROVE                      1 // JVET-Y0141 test3: Sign prediction improvement                          
#define JVET_AD0105_ASPECT1_NUM_SIGN_PRED_BY_QP           1 // JVET-AD0105 Aspect1: NumSignPred based on QP
#endif
#define JVET_W0103_INTRA_MTS                              1 // JVET-W0103: Extended Intra MTS
#if JVET_W0103_INTRA_MTS
#define JVET_Y0142_ADAPT_INTRA_MTS                        1 // JVET-Y0142: Adaptive Intra MTS
#if JVET_Y0142_ADAPT_INTRA_MTS
#define JVET_Y0159_INTER_MTS                              1 // JVET-Y0159: Inter MTS uses fixed 4 candidates
#endif
#endif
#define JVET_AA0133_INTER_MTS_OPT                         1 // JVET-AA0133: Inter MTS optimization
#define JVET_AB0067_MIP_DIMD_LFNST                        1 // JVET-AB0067: Modification of LFNST for MIP coded block
#define JVET_AC0130_NSPT                                  1 // JVET-AC0130: NSPT replacing DCT-II + LFNST for certain block shapes
#if JVET_AC0130_NSPT
#define JVET_AE0086_LARGE_NSPT                            1 // JVET-AE0086: Large NSPT kernels (4x32, 8x32)
#endif
#define JVET_AC0115_INTRA_TMP_DIMD_MTS_LFNST              1 // JVET-AC0115: Modifications of MTS/LFNST for Intra TMP coded block
#define JVET_AE0125_SHIFT_QUANTIZATION_CENTER             1 // JVET-AE0125: Shifting quantization center
#define JVET_AE0102_LFNST_CTX                             1 // JVET-AE0102: Context modelling for transform coefficients for LFNST/NSPT
#define JVET_AG0061_INTER_LFNST_NSPT                      1 // JVET-AG0061: 3.3 Utilizing LFNST/NSPT for inter coding
#define JVET_AG0100_TRANSFORM_COEFFICIENT_CODING          1 // JVET_AG0100: 3.2b Transform coefficient coding
#define JVET_AG0143_INTER_INTRA                           1 // JVET_AG0143: 3.1c CABAC inter/intra model switch

// Entropy Coding
#define EC_HIGH_PRECISION                                 1 // CABAC high precision
#define SLICE_TYPE_WIN_SIZE                               1 // Context window initialization based on slice type
#define JVET_Z0135_TEMP_CABAC_WIN_WEIGHT                  1 // JVET-Z0135 Test 4.3b: Temporal CABAC, weighted states, windows adjustment
#define JVET_AD0206_CABAC_INIT_AT_GDR                     1 // JVET-AD0206: Cabac initialization at GDR picture
#define JVET_AF0133_RETRAINING_ISLICE_CTX                 1 // JVET-AF0133: retrained I-slice context model
#define JVET_ECM12_CABAC_RETRAIN                          1 // use retrained contexts for ecm-12.0
#define JVET_AG0196_CABAC_RETRAIN                         1 // JVET-AG0196 5.1 bin dumper
#define JVET_AG0196_WINDOWS_OFFSETS_SLICETYPE             1 // JVET-AG0196 5.2: window offsets per slice type
#define JVET_AG0117_CABAC_SPATIAL_TUNING                  1 // JVET-AG0117 5.3: spatial CABAC tuning

// Loop filters
#define ALF_IMPROVEMENT                                   1 // ALF improvement
#define EMBEDDED_APS                                      1 // Embed APS into picture header
#define JVET_V0094_BILATERAL_FILTER                       1 // Bilateral filter
#define JVET_X0071_CHROMA_BILATERAL_FILTER                1 // JVET-X0071/JVET-X0067: Chroma bilateral filter
#define JVET_W0066_CCSAO                                  1 // JVET-W0066: Cross-component sample adaptive offset
#define JVET_X0071_LONGER_CCALF                           1 // JVET-X0071/JVET-X0045: Longer filter for CCALF
#define JVET_X0071_ALF_BAND_CLASSIFIER                    1 // JVET-X0071/JVET-X0070: Alternative band classifier for ALF
#define JVET_Y0106_CCSAO_EDGE_CLASSIFIER                  1 // JVET-Y0106: Edge based classifier for CCSAO
#define JVET_Z0105_LOOP_FILTER_VIRTUAL_BOUNDARY           1 // JVET-Z0105: Enable virtual boundary processing for in-loop filters
#define JVET_AA0095_ALF_LONGER_FILTER                     1 // JVET-AA0095: Longer luma filter shape 
#define JVET_AA0095_ALF_WITH_SAMPLES_BEFORE_DBF           1 // JVET-AA0095: Using samples before deblocking filter for ALF
#define JVET_AB0184_ALF_MORE_FIXED_FILTER_OUTPUT_TAPS     1 // JVET-AB0184: Extended Fixed-Filter-Output based Taps for ALF
#define JVET_AB0171_ASYMMETRIC_DB_FOR_GDR                 1 // JVET-AB0171: Asymmetric Deblocking for GDR
#define JVET_AC0162_ALF_RESIDUAL_SAMPLES_INPUT            1 // JVET-AC0162: ALF using residaul samples as additional inputs
#define JVET_AD0222_ALF_LONG_FIXFILTER                    1 // JVET-AD0222: More taps for fixed filter outputs
#define JVET_AD0222_ALF_RESI_CLASS                        1 // JVET-AD0222: Residual-based classifier
#define JVET_AD0222_ADDITONAL_ALF_FIXFILTER               1 // JVET-AD0222: Additional ALF fixed filter
#define JVET_AE0139_ALF_IMPROVED_FIXFILTER                1 // JVET-AE0139: Improved ALF fixed filter
#define JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO      1 // JVET-AE0151: CCSAO with history offsets and extended edge classifiers
#define JVET_AF0197_LUMA_RESIDUAL_TAP_IN_CCALF            1 // JVET-AF0197: Luma Residual Tap in CCALF
#define JVET_AF0112_BIF_DYNAMIC_SCALING                   1 // JVET-AF0112: Dynamic TU scale factor for BIF with LUTs interpolation
#define JVET_AG0157_ALF_CHROMA_FIXED_FILTER               1 // JVET-AG0157: Fixed filter for chroma ALF
#define JVET_AG0158_ALF_LUMA_COEFF_PRECISION              1 // JVET-AG0158: adaptive precision for luma ALF coefficients
#define JVET_AG0145_ADAPTIVE_CLIPPING                     1 // JVET-AG0145: Adaptive clipping with signalled lower and upper bounds

// SIMD optimizations
#if IF_12TAP
#define IF_12TAP_SIMD                                     1 // Enable SIMD for 12-tap filter
#if IF_12TAP_SIMD
#define SIMD_4x4_12                                       1 // Enable 4x4-block combined passes for 12-tap filters
#endif
#endif
#if JVET_V0130_INTRA_TMP
#define ENABLE_SIMD_TMP                                   1
#endif
#if JVET_V0094_BILATERAL_FILTER
#define ENABLE_SIMD_BILATERAL_FILTER                      1
#endif
#if JVET_X0071_CHROMA_BILATERAL_FILTER
#define JVET_X0071_CHROMA_BILATERAL_FILTER_ENABLE_SIMD    1
#endif
#endif // tools

// Software extensions
#define RPR_ENABLE                                        1 // JVET-X0121: Fixes for RRP
#define JVET_Y0128_NON_CTC                                1 // JVET-Y0128: Fixing issues for RPR enabling and non-CTC configuration
#define JVET_Y0240_BIM                                    1 // JVET-Y0240: Block importance mapping
#define JVET_Z0067_RPR_ENABLE                             1 // JVET-Z0067: Fixes for RPR
#define JVET_Z0150_MEMORY_USAGE_PRINT                     1 // JVET-Z0150: Print memory usage
#define JVET_Z0118_GDR                                    1 // JVET-Z0118: GDR
#define JVET_AD0169_SMALL_SCALE_DOWNSAMPLING              1 // JVET-AD0169: Downsampling filters in range 1.1 to 1.35 based on Kaiser(7) windowed sinc

#if JVET_Z0118_GDR
#define GDR_LEAK_TEST                                     0
#define GDR_ENC_TRACE                                     0
#define GDR_DEC_TRACE                                     0
#endif

#define JVET_AC0096                                       1 // JVET-AC0096: RPR functionality testing
#define JVET_AG0116                                       1 // JVET-AG0116: GOP-based RPR encoder control from VTM







// clang-format off
#define JVET_V0056                                        1 // JVET-V0056: MCTF changes

//########### place macros to be removed in next cycle below this line ###############
#define JVET_W0134_UNIFORM_METRICS_LOG                    1 // change metrics output for easy parsing
#define MSSIM_UNIFORM_METRICS_LOG                         1 // add MS-SSIM support to uniform logs

//########### place macros to be removed in next cycle below this line ###############
#define JVET_S0058_GCI                                    1 // no_mtt_constraint_flag and no_weightedpred_constraint_flag

#define JVET_R0341_GCI                                    1 // JVET-R0341: on constraint flag for local chroma QP control

#define JVET_S0203                                        1 // JVET-S0203 (aspects 1 & 2): change the signalling of sublayer_level_idc[ i ] and ptl_sublayer_level_present_flag[ i ] to be in descending order

#define JVET_S0066_GCI                                    1 // JVET-S0066: Signal new GCI flags gci_three_minus_max_log2_ctu_size_constraint_idc and gci_no_luma_transform_size_64_constraint_flag (no_explicit_scaling_list_constraint_flag already included as part of JVET-S0050)

#define JVET_S0193_NO_OUTPUT_PRIOR_PIC                    1 // JVET-S0193: Move ph_no_output_of_prior_pics_flag to SH

#define JVET_S_PROFILES                                   1 // Profile definitions

#define JVET_S_SUB_PROFILE                                1 // Move signalling of ptl_num_sub_profiles

#define JVET_R0324_REORDER                                1 // Reordering of syntax elements JVET-R0324/JVET-R2001-v2

#define JVET_S0219_ASPECT2_CHANGE_ORDER_APS_PARAMS_TYPE   1 // JVET-S0219 aspect2: change the order to put the aps_params_type before the aps_adaptation_parameter_set_id.

#define JVET_R0270                                        1 // JVET-S0270: Treating picture with mixed RASL and RADL slices as RASL picture

#define JVET_S0081_NON_REFERENCED_PIC                     1 // JVET-S0081: exclude non-referenced picture to be used as prevTid0 picture

#define JVET_R0433                                        1 // JVET-R0433: APS signaling and semantics cleanup

#define JVET_S0076_ASPECT1                                1 // JVET-S0076: aspect 1: Move ph_non_ref_pic_flag to earlier position

#define JVET_S0045_SIGN                                   1 // JVET-S0045: semantics of strp_entry_sign_flag

#define JVET_S0133_PH_SYNTAX_OVERRIDE_ENC_FIX             1 // JVET-S0133: Encoder-only fix on the override of partition constriants in PH

#define JVET_S0266_VUI_length                             1 // JVET-S0266: VUI modifications including signalling of VUI length

#define JVET_S0179_CONDITIONAL_SIGNAL_GCI                 1 // JVET-S0179: Conditional signalling of GCI fields

#define JVET_S0049_ASPECT4                                1 // JVET-S0049 aspect 4: Constrain the value of pps_alf_info_in_ph_flag to be equal to 0 when the PH is in the SH

#define JVET_S0258_SUBPIC_CONSTRAINTS                     1 // JVET-S0258: sub-picture constraints

#define JVET_S0074_SPS_REORDER                            1 // JVET-S0074: aspect 1, rearrange some syntax elements in SPS

#define JVET_S0234_ACT_CRS_FIX                            1 // JVET-S0234: perform chroma residual scaling in RGB domain when ACT is on

#define JVET_S0094_CHROMAFORMAT_BITDEPTH_CONSTRAINT       1 // JVET-S0094: 0 for constraint flags for chroma format and bit depth mean unconstrained, by coding these constraints as subtractive

#define JVET_S0132_HLS_REORDER                            1 // Rearrange syntax elements in SPS and PPS

#define JVET_S0221_NUM_VB_CHECK                           1 // JVET_S0221: Constraints on the number of virtual boundaries

#define JVET_S0052_RM_SEPARATE_COLOUR_PLANE               1 // JVET-S0052: Remove separate colour plane coding from VVC version 1

#define JVET_S0123_IDR_UNAVAILABLE_REFERENCE              1 // JVET-S0123: Invoke the generation of unavailable reference picture for an IDR picture that has RPLs.
                                                            //             Change the process for deriving empty RPLs when sps_idr_rpl_present_flag is equal to 0 and nal_unit_type is equal to IDR_W_RADL or IDR_N_LP to involve pps_rpl_info_in_ph_flag.
#define JVET_S0124_UNAVAILABLE_REFERENCE                  1 // JVET-S0124: Add TemporalId, ph_non_ref_pic_flag, and ph_pic_parameter_set_id for generating unavailable reference pictures

#define JVET_S0063_VPS_SIGNALLING                         1 // Modifications to VPS signalling - conditionally signal vps_num_ptls_minus1

#define JVET_S0065_SPS_INFERENCE_RULE                     1 // JVET_S0065_PROPOSAL1: Inference rule for sps_virtual_boundaries_present_flag

#define JVET_S0155_EOS_NALU_CHECK                         1 // JVET-S0155: Constraints on EOS NAL units

#define JVET_R0093_SUBPICS_AND_CONF_WINDOW                1 // JVET-R0093 and JVET-R0294: Constraint on subpictures and conformance cropping window, and rewriting of conformance cropping window in subpicture extraction

#define JVET_S0160_ASPECT1_ASPECT9                        1 // JVET-S0160: Aspect 1 Infer the value of pps_loop_filter_across_tiles_enabled_flag to be equal to 0 (instead of 1) when not present
                                                            //             Aspect 9 The value of ph_poc_msb_cycle_present_flag is required to be equal to 0 when vps_independent_layer_flag[GeneralLayerIdx[nuh_layer_id]] is equal to 0 and there is an ILRP entry in RefPicList[0] or RefPicList[1] of a slice of the current picture

#define JVET_S0071_SAME_SIZE_SUBPIC_LAYOUT                1 // JVET-S0071 : shortcut when all subpictures have the same size

#define JVET_S0098_SLI_FRACTION                           1 // JVET-S0098 Item 3: Add non_subpic_layers_fraction syntax element

#define JVET_S0048_SCALING_OFFSET                         1 // JVET-S0048 Aspect2: change the constraint on the value ranges of scaling window offsets to be more flexible

#define JVET_S0248_HRD_CLEANUP                            1 // JVET-S0248 Aspect7: When bp_alt_cpb_params_present_flag is equal to 1, the value of bp_du_hrd_params_present_flag shall be equal to 0.

#define JVET_S0100_ASPECT3                                1 // JVET-S0100 Aspect 3: constraints on vps_dpb_max_tid and vps_hrd_max_tid depending on vps_ptl_max_tid

#define JVET_S0156_LEVEL_DEFINITION                       1 // JVET-S0156: On level definitions

#define JVET_S0064_SEI_BUFFERING_PERIOD_CLEANUP           1 // JVET-S0064: Conditionally signal bp_sublayer_dpb_output_offsets_present_flag

#define JVET_S0185_PROPOSAL2_SEI_CLEANUP                  1 // JVET-S0185_PROPOSAL2: Move signalling of syntax element bp_alt_cpb_params_present_flag

#define JVET_R0294_SUBPIC_HASH                            1 // JVET-R0294: Allow decoded picture hash SEI messages to be nested in subpicture context

#define JVET_S0181_PROPOSAL1                              1 // JVET-0181_Proposal1: Conditionally signal bp_sublayer_initial_cpb_removal_delay_present_flag

#define JVET_Q0406_CABAC_ZERO                             1 // JVET-Q0406: signal cabac_zero_words per sub-picture

#define JVET_S0177_SCALABLE_NESTING_SEI                   1 // JVET-S0177: Constraints on the scalable nesting SEI message

#define JVET_R0068_ASPECT6_ENC_RESTRICTION                1 // encoder restriction for JVET-R0068 apsect 6

#define JVET_S0178_GENERAL_SEI_CHECK                      1 // JVET-S0178: General SEI semantics and constraints

#define JVET_S0176_SLI_SEI                                1 // JVET-S0176: On the subpicture level information SEI message

#define JVET_S0186_SPS_CLEANUP                            1 // JVET-S0186: Proposal 1, move sps_chroma_format_idc and sps_log2_ctu_size_minus5 to take place sps_reserved_zero_4bits

#define JVET_S0181_PROPOSAL2_BUFFERING_PERIOD_CLEANUP     1 // JVET-S0181 Proposal2: Move signalling of bp_max_sublayers_minus1 and conditionally signal bp_cpb_removal_delay_deltas_present_flag, bp_num_cpb_removal_delay_deltas_minus1, and bp_cpb_removal_delay

#define JVET_S0050_GCI                                    1 // JVET-S0050: Signal new GCI flags no_virtual_boundaries_constraint_flag and no_explicit_scaling_list_constraint_flag
                                                            //             Constrain the value of one_subpic_per_pic_constraint_flag, one_slice_per_pic_constraint_flag and no_aps_constraint_flag
                                                            //             Remove all constraints that require GCI fields to be equal to a value that imposes a constraint

#define JVET_S0138_GCI_PTL                                1 // JVET-S_Notes_d9: move frame_only_constraint_flag and single_layer_constraint_flag into PTL for easy access

#define JVET_S0113_S0195_GCI                              1 // JVET-S0113: no_rectangular_slice_constraint_flag to constrain pps_rect_slice_flag
                                                            //             one_slice_per_subpicture_constraint_flag to constrain pps_single_slice_per_subpic_flag
                                                            // JVET-S0195: replace one_subpic_per_pic_constraint_flag with no_subpic_info_constraint_flag and its semantics
                                                            //             add no_idr_rpl_constraint_flag

#define JVET_S0182_RPL_SIGNALLING                         1 // JVET-S0182: modifications to rpl information signalling

#define JVET_S0185_PROPOSAl1_PICTURE_TIMING_CLEANUP       1 // JVET-S0185: Proposal 1, put syntax element pt_cpb_removal_delay_minus1[] first, followed by similar information for sub-layers, followed by pt_dpb_output_delay

#define JVET_S0183_VPS_INFORMATION_SIGNALLING             1 // JVET-S0183: Proposal 1, signal vps_num_output_layer_sets_minus1 as vps_num_output_layer_sets_minus2

#define JVET_S0184_VIRTUAL_BOUNDARY_CONSTRAINT            1 // JVET-S0184: Conformance constraints regarding virtual boundary signalling when subpictures are present

#define JVET_Q0114_ASPECT5_GCI_FLAG                       1 // JVET-Q0114 Aspect 5: Add a general constraint on no reference picture resampling

#define JVET_S0105_GCI_REORDER_IN_CATEGORY                1 // JVET-S0105: reorder and categorize GCI flags (assumes the following macros set to 1: JVET_S0050_GCI, JVET_S0113_S0195_GCI, JVET_S0066_GCI, JVET_S0138_GCI_PTL, JVET_S0058_GCI, JVET_R0341_GCI, JVET_Q0114_ASPECT5_GCI_FLAG)

//########### place macros to be be kept below this line ###############
#define JVET_S0257_DUMP_360SEI_MESSAGE                    1 // Software support of 360 SEI messages

#define JVET_R0351_HIGH_BIT_DEPTH_SUPPORT                 1 // JVET-R0351: high bit depth coding support (syntax changes, no mathematical differences for CTCs)
#define JVET_R0351_HIGH_BIT_DEPTH_SUPPORT_VS              1 // JVET-R0351: high bit depth coding support (syntax changes for Visual Studio)
#define JVET_R0351_HIGH_BIT_DEPTH_ENABLED                 0 // JVET-R0351: high bit depth coding enabled (increases accuracies of some calculations, e.g. transforms)

#define JVET_R0164_MEAN_SCALED_SATD                       1 // JVET-R0164: Use a mean scaled version of SATD in encoder decisions

#define JVET_M0497_MATRIX_MULT                            0 // 0: Fast method; 1: Matrix multiplication

#define JVET_R0107_BITSTREAM_EXTACTION                    1 // JVET-R0107 Proposal 3:Bitsteam extraction modifications

#define APPLY_SBT_SL_ON_MTS                               1 // apply save & load fast algorithm on inter MTS when SBT is on

typedef std::pair<int, bool> TrMode;
typedef std::pair<int, int>  TrCost;

#define REUSE_CU_RESULTS                                  1
#if REUSE_CU_RESULTS
#if !CONVERT_NUM_TU_SPLITS_TO_CFG
#define REUSE_CU_RESULTS_WITH_MULTIPLE_TUS                1
#endif
#endif
// clang-format on

#ifndef JVET_J0090_MEMORY_BANDWITH_MEASURE
#define JVET_J0090_MEMORY_BANDWITH_MEASURE                0
#endif

#ifndef EXTENSION_360_VIDEO
#define EXTENSION_360_VIDEO                               0   ///< extension for 360/spherical video coding support; this macro should be controlled by makefile, as it would be used to control whether the library is built and linked
#endif

#ifndef EXTENSION_HDRTOOLS
#define EXTENSION_HDRTOOLS                                0 //< extension for HDRTools/Metrics support; this macro should be controlled by makefile, as it would be used to control whether the library is built and linked
#endif

#define JVET_O0756_CONFIG_HDRMETRICS                      1
#if EXTENSION_HDRTOOLS
#define JVET_O0756_CALCULATE_HDRMETRICS                   1
#endif

#ifndef ENABLE_SPLIT_PARALLELISM
#define ENABLE_SPLIT_PARALLELISM                          0
#endif
#if ENABLE_SPLIT_PARALLELISM
#define PARL_SPLIT_MAX_NUM_JOBS                           6                             // number of parallel jobs that can be defined and need memory allocated
#define NUM_RESERVERD_SPLIT_JOBS                        ( PARL_SPLIT_MAX_NUM_JOBS + 1 )  // number of all data structures including the merge thread (0)
#define PARL_SPLIT_MAX_NUM_THREADS                        PARL_SPLIT_MAX_NUM_JOBS
#define NUM_SPLIT_THREADS_IF_MSVC                         4

#endif

// clang-format on

// ====================================================================================================================
// General settings
// ====================================================================================================================

#ifndef ENABLE_TRACING
#define ENABLE_TRACING                                    0 // DISABLE by default (enable only when debugging, requires 15% run-time in decoding) -- see documentation in 'doc/DTrace for NextSoftware.pdf'
#endif

#if ENABLE_TRACING
#define K0149_BLOCK_STATISTICS                            1 // enables block statistics, which can be analysed with YUView (https://github.com/IENT/YUView)
#if K0149_BLOCK_STATISTICS
#define BLOCK_STATS_AS_CSV                                0 // statistics will be written in a comma separated value format. this is not supported by YUView
#endif
#endif

#define WCG_EXT                                           1
#define WCG_WPSNR                                         WCG_EXT

#define KEEP_PRED_AND_RESI_SIGNALS                        0

// ====================================================================================================================
// Debugging
// ====================================================================================================================

// most debugging tools are now bundled within the ENABLE_TRACING macro -- see documentation to see how to use

#define PRINT_MACRO_VALUES                                1 ///< When enabled, the encoder prints out a list of the non-environment-variable controlled macros and their values on startup

#define INTRA_FULL_SEARCH                                 0 ///< enables full mode search for intra estimation

// TODO: rename this macro to DECODER_DEBUG_BIT_STATISTICS (may currently cause merge issues with other branches)
// This can be enabled by the makefile
#ifndef RExt__DECODER_DEBUG_BIT_STATISTICS
#define RExt__DECODER_DEBUG_BIT_STATISTICS                0 ///< 0 (default) = decoder reports as normal, 1 = decoder produces bit usage statistics (will impact decoder run time by up to ~10%)
#endif

#ifndef RExt__DECODER_DEBUG_TOOL_MAX_FRAME_STATS
#define RExt__DECODER_DEBUG_TOOL_MAX_FRAME_STATS         (1 && RExt__DECODER_DEBUG_BIT_STATISTICS )   ///< 0 (default) = decoder reports as normal, 1 = decoder produces max frame bit usage statistics
#endif

#define TR_ONLY_COEFF_STATS                              (1 && RExt__DECODER_DEBUG_BIT_STATISTICS )   ///< 0 combine TS and non-TS decoder debug statistics. 1 = separate TS and non-TS decoder debug statistics.
#define EPBINCOUNT_FIX                                   (1 && RExt__DECODER_DEBUG_BIT_STATISTICS )   ///< 0 use count to represent number of calls to decodeBins. 1 = count and bins for EP bins are the same.

#ifndef RExt__DECODER_DEBUG_TOOL_STATISTICS
#define RExt__DECODER_DEBUG_TOOL_STATISTICS               0 ///< 0 (default) = decoder reports as normal, 1 = decoder produces tool usage statistics
#endif

#if RExt__DECODER_DEBUG_BIT_STATISTICS || RExt__DECODER_DEBUG_TOOL_STATISTICS
#define RExt__DECODER_DEBUG_STATISTICS                    1
#endif

// ====================================================================================================================
// Tool Switches - transitory (these macros are likely to be removed in future revisions)
// ====================================================================================================================

#define DECODER_CHECK_SUBSTREAM_AND_SLICE_TRAILING_BYTES  1 ///< TODO: integrate this macro into a broader conformance checking system.
#define T0196_SELECTIVE_RDOQ                              1 ///< selective RDOQ
#define U0040_MODIFIED_WEIGHTEDPREDICTION_WITH_BIPRED_AND_CLIPPING 1
#define U0033_ALTERNATIVE_TRANSFER_CHARACTERISTICS_SEI    1 ///< Alternative transfer characteristics SEI message (JCTVC-U0033, with syntax naming from V1005)
#define X0038_LAMBDA_FROM_QP_CAPABILITY                   1 ///< This approach derives lambda from QP+QPoffset+QPoffset2. QPoffset2 is derived from QP+QPoffset using a linear model that is clipped between 0 and 3.
                                                            // To use this capability enable config parameter LambdaFromQpEnable

// ====================================================================================================================
// Tool Switches
// ====================================================================================================================


// This can be enabled by the makefile
#ifndef RExt__HIGH_BIT_DEPTH_SUPPORT
#if JVET_R0351_HIGH_BIT_DEPTH_ENABLED
#define RExt__HIGH_BIT_DEPTH_SUPPORT                      1 ///< 0 (default) use data type definitions for 8-10 bit video, 1 = use larger data types to allow for up to 16-bit video (originally developed as part of N0188)
#else
#define RExt__HIGH_BIT_DEPTH_SUPPORT                      0 ///< 0 (default) use data type definitions for 8-10 bit video, 1 = use larger data types to allow for up to 16-bit video (originally developed as part of N0188)
#endif
#endif

// SIMD optimizations
#define SIMD_ENABLE                                       1
#define ENABLE_SIMD_OPT                                 ( SIMD_ENABLE && !RExt__HIGH_BIT_DEPTH_SUPPORT )    ///< SIMD optimizations, no impact on RD performance
#define ENABLE_SIMD_OPT_MCIF                            ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for the interpolation filter, no impact on RD performance
#define ENABLE_SIMD_OPT_BUFFER                          ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for the buffer operations, no impact on RD performance
#define ENABLE_SIMD_OPT_DIST                            ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for the distortion calculations(SAD,SSE,HADAMARD), no impact on RD performance
#define ENABLE_SIMD_OPT_AFFINE_ME                       ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for affine ME, no impact on RD performance
#define ENABLE_SIMD_OPT_ALF                             ( 1 && ENABLE_SIMD_OPT )                            ///< SIMD optimization for ALF
#if ENABLE_SIMD_OPT_BUFFER
#define ENABLE_SIMD_OPT_BCW                               1                                                 ///< SIMD optimization for Bcw
#endif

// End of SIMD optimizations


#define ME_ENABLE_ROUNDING_OF_MVS                         1 ///< 0 (default) = disables rounding of motion vectors when right shifted,  1 = enables rounding

#define RDOQ_CHROMA_LAMBDA                                1 ///< F386: weighting of chroma for RDOQ

#define U0132_TARGET_BITS_SATURATION                      1 ///< Rate control with target bits saturation method
#ifdef  U0132_TARGET_BITS_SATURATION
#define V0078_ADAPTIVE_LOWER_BOUND                        1 ///< Target bits saturation with adaptive lower bound
#endif
#define W0038_DB_OPT                                      1 ///< adaptive DB parameter selection, LoopFilterOffsetInPPS and LoopFilterDisable are set to 0 and DeblockingFilterMetric=2;
#define W0038_CQP_ADJ                                     1 ///< chroma QP adjustment based on TL, CQPTLAdjustEnabled is set to 1;

#define SHARP_LUMA_DELTA_QP                               1 ///< include non-normative LCU deltaQP and normative chromaQP change
#define ER_CHROMA_QP_WCG_PPS                              1 ///< Chroma QP model for WCG used in Anchor 3.2
#define ENABLE_QPA                                        1 ///< Non-normative perceptual QP adaptation according to JVET-H0047 and JVET-K0206. Deactivated by default, activated using encoder arguments --PerceptQPA=1 --SliceChromaQPOffsetPeriodicity=1
#define ENABLE_QPA_SUB_CTU                              ( 1 && ENABLE_QPA ) ///< when maximum delta-QP depth is greater than zero, use sub-CTU QPA


#define RDOQ_CHROMA                                       1 ///< use of RDOQ in chroma

#define QP_SWITCHING_FOR_PARALLEL                         1 ///< Replace floating point QP with a source-file frame number. After switching POC, increase base QP instead of frame level QP.

#define LUMA_ADAPTIVE_DEBLOCKING_FILTER_QP_OFFSET         1 /// JVET-L0414 (CE11.2.2) with explicit signalling of num interval, threshold and qpOffset

#define Y4M_SUPPORT                                       1
// ====================================================================================================================
// Derived macros
// ====================================================================================================================

#if RExt__HIGH_BIT_DEPTH_SUPPORT
#define FULL_NBIT                                         1 ///< When enabled, use distortion measure derived from all bits of source data, otherwise discard (bitDepth - 8) least-significant bits of distortion
#define RExt__HIGH_PRECISION_FORWARD_TRANSFORM            1 ///< 0 use original 6-bit transform matrices for both forward and inverse transform, 1 (default) = use original matrices for inverse transform and high precision matrices for forward transform
#else
#define FULL_NBIT                                         1 ///< When enabled, use distortion measure derived from all bits of source data, otherwise discard (bitDepth - 8) least-significant bits of distortion
#define RExt__HIGH_PRECISION_FORWARD_TRANSFORM            0 ///< 0 (default) use original 6-bit transform matrices for both forward and inverse transform, 1 = use original matrices for inverse transform and high precision matrices for forward transform
#endif

#if FULL_NBIT
#define DISTORTION_PRECISION_ADJUSTMENT(x)                0
#else
#define DISTORTION_ESTIMATION_BITS                        8
#define DISTORTION_PRECISION_ADJUSTMENT(x)                ((x>DISTORTION_ESTIMATION_BITS)? ((x)-DISTORTION_ESTIMATION_BITS) : 0)
#endif

// ====================================================================================================================
// Error checks
// ====================================================================================================================

#if ((RExt__HIGH_PRECISION_FORWARD_TRANSFORM != 0) && (RExt__HIGH_BIT_DEPTH_SUPPORT == 0))
#error ERROR: cannot enable RExt__HIGH_PRECISION_FORWARD_TRANSFORM without RExt__HIGH_BIT_DEPTH_SUPPORT
#endif

// ====================================================================================================================
// Named numerical types
// ====================================================================================================================

#if RExt__HIGH_BIT_DEPTH_SUPPORT
typedef       int             Pel;               ///< pixel type
typedef       int64_t           TCoeff;            ///< transform coefficient
typedef       int             TMatrixCoeff;      ///< transform matrix coefficient
typedef       int16_t           TFilterCoeff;      ///< filter coefficient
typedef       int64_t           Intermediate_Int;  ///< used as intermediate value in calculations
typedef       uint64_t          Intermediate_UInt; ///< used as intermediate value in calculations
#else
typedef       int16_t           Pel;               ///< pixel type
typedef       int             TCoeff;            ///< transform coefficient
typedef       int16_t           TMatrixCoeff;      ///< transform matrix coefficient
typedef       int16_t           TFilterCoeff;      ///< filter coefficient
typedef       int             Intermediate_Int;  ///< used as intermediate value in calculations
typedef       uint32_t            Intermediate_UInt; ///< used as intermediate value in calculations
#endif

typedef       uint64_t          SplitSeries;       ///< used to encoded the splits that caused a particular CU size
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
typedef       uint64_t          ModeTypeSeries;    ///< used to encoded the ModeType at different split depth
#endif
typedef       uint64_t        Distortion;        ///< distortion measurement

// ====================================================================================================================
// Enumeration
// ====================================================================================================================

#if JVET_AA0096_MC_BOUNDARY_PADDING
enum PadDirection
{
  PAD_TOP           = 0,
  PAD_BOTTOM        = 1,
  PAD_RIGHT         = 2,
  PAD_LEFT          = 3,
  MAX_PAD_DIRECTION = 4
};
#endif

#if JVET_AD0208_IBC_ADAPT_FOR_CAM_CAPTURED_CONTENTS
enum IbcBvStatus
{
  IBC_BV_INVALID,
  IBC_BV_VALID,
  IBC_INT_BV_VALID
};
#endif

enum ApsType
{
  ALF_APS = 0,
  LMCS_APS = 1,
  SCALING_LIST_APS = 2,
};

enum QuantFlags
{
  Q_INIT           = 0x0,
  Q_USE_RDOQ       = 0x1,
  Q_RDOQTS         = 0x2,
  Q_SELECTIVE_RDOQ = 0x4,
};

//EMT transform tags
enum TransType
{
#if JVET_W0103_INTRA_MTS
#if JVET_AA0133_INTER_MTS_OPT
  DCT2 = 0,
  DCT8 = 1,
  DST7 = 2,
  DCT5 = 3,
  DST4 = 4,
  DST1 = 5,
  IDTR = 6,
  KLT0 = 7,
  KLT1 = 8,
  NUM_TRANS_TYPE = 9,
  DCT2_EMT = 10
#else
  DCT2 = 0,
  DCT8 = 1,
  DST7 = 2,
  DCT5 = 3,
  DST4 = 4,
  DST1 = 5,
  IDTR = 6,
  NUM_TRANS_TYPE = 7,
  DCT2_EMT = 8
#endif
#else
  DCT2 = 0,
  DCT8 = 1,
  DST7 = 2,
  NUM_TRANS_TYPE = 3,
  DCT2_EMT = 4
#endif
};

enum MTSIdx
{
  MTS_DCT2_DCT2 = 0,
  MTS_SKIP = 1,
  MTS_DST7_DST7 = 2,
  MTS_DCT8_DST7 = 3,
  MTS_DST7_DCT8 = 4,
  MTS_DCT8_DCT8 = 5
};

enum ISPType
{
  NOT_INTRA_SUBPARTITIONS       = 0,
  HOR_INTRA_SUBPARTITIONS       = 1,
  VER_INTRA_SUBPARTITIONS       = 2,
  NUM_INTRA_SUBPARTITIONS_MODES = 3,
  INTRA_SUBPARTITIONS_RESERVED  = 4
};

#if JVET_W0123_TIMD_FUSION || (JVET_Z0056_GPM_SPLIT_MODE_REORDERING && JVET_Y0065_GPM_INTRA)
enum TemplateType
{
  NO_NEIGHBOR         = 0,
  LEFT_NEIGHBOR       = 1,
  ABOVE_NEIGHBOR      = 2,
  LEFT_ABOVE_NEIGHBOR = 3
};
#endif

enum SbtIdx
{
  SBT_OFF_DCT  = 0,
  SBT_VER_HALF = 1,
  SBT_HOR_HALF = 2,
  SBT_VER_QUAD = 3,
  SBT_HOR_QUAD = 4,
  NUMBER_SBT_IDX,
  SBT_OFF_MTS, //note: must be after all SBT modes, only used in fast algorithm to discern the best mode is inter EMT
};

enum SbtPos
{
  SBT_POS0 = 0,
  SBT_POS1 = 1,
  NUMBER_SBT_POS
};

enum SbtMode
{
  SBT_VER_H0 = 0,
  SBT_VER_H1 = 1,
  SBT_HOR_H0 = 2,
  SBT_HOR_H1 = 3,
  SBT_VER_Q0 = 4,
  SBT_VER_Q1 = 5,
  SBT_HOR_Q0 = 6,
  SBT_HOR_Q1 = 7,
  NUMBER_SBT_MODE
};

/// supported slice type
enum SliceType
{
  B_SLICE               = 0,
  P_SLICE               = 1,
  I_SLICE               = 2,
  NUMBER_OF_SLICE_TYPES = 3
};

/// chroma formats (according to how the monochrome or the color planes are intended to be coded)
enum ChromaFormat
{
  CHROMA_400        = 0,
  CHROMA_420        = 1,
  CHROMA_422        = 2,
  CHROMA_444        = 3,
  NUM_CHROMA_FORMAT = 4
};

enum ChannelType
{
  CHANNEL_TYPE_LUMA    = 0,
  CHANNEL_TYPE_CHROMA  = 1,
  MAX_NUM_CHANNEL_TYPE = 2
};
#if JVET_W0069_TMP_BOUNDARY
enum RefTemplateType
{
  L_SHAPE_TEMPLATE = 1,
  LEFT_TEMPLATE    = 2,
  ABOVE_TEMPLATE   = 3,
  NO_TEMPLATE      = 4
};
#endif
#if JVET_AD0086_ENHANCED_INTRA_TMP
enum TmpSubPelDirType
{
  LEFT_POS         = 0,
  RIGHT_POS        = 1,
  ABOVE_POS        = 2,
  BOTTOM_POS       = 3,
  ABOVE_LEFT_POS   = 4,
  ABOVE_RIGHT_POS  = 5,
  LEFT_BOTTOM_POS  = 6,
  RIGHT_BOTTOM_POS = 7,
};
#endif
#if !INTRA_RM_SMALL_BLOCK_SIZE_CONSTRAINTS
enum TreeType
{
  TREE_D = 0, //default tree status (for single-tree slice, TREE_D means joint tree; for dual-tree I slice, TREE_D means TREE_L for luma and TREE_C for chroma)
  TREE_L = 1, //separate tree only contains luma (may split)
  TREE_C = 2, //separate tree only contains chroma (not split), to avoid small chroma block
};

enum ModeType
{
  MODE_TYPE_ALL = 0, //all modes can try
  MODE_TYPE_INTER = 1, //can try inter
  MODE_TYPE_INTRA = 2, //can try intra, ibc, palette
};
#endif
#define CH_L CHANNEL_TYPE_LUMA
#define CH_C CHANNEL_TYPE_CHROMA

enum ComponentID
{
  COMPONENT_Y         = 0,
#if JVET_W0066_CCSAO
  MAX_NUM_LUMA_COMP   = 1,
#endif
  COMPONENT_Cb        = 1,
  COMPONENT_Cr        = 2,
  MAX_NUM_COMPONENT   = 3,
  JOINT_CbCr          = MAX_NUM_COMPONENT,
  MAX_NUM_TBLOCKS     = MAX_NUM_COMPONENT
};

#define MAP_CHROMA(c) (ComponentID(c))

enum InputColourSpaceConversion // defined in terms of conversion prior to input of encoder.
{
  IPCOLOURSPACE_UNCHANGED               = 0,
  IPCOLOURSPACE_YCbCrtoYCrCb            = 1, // Mainly used for debug!
  IPCOLOURSPACE_YCbCrtoYYY              = 2, // Mainly used for debug!
  IPCOLOURSPACE_RGBtoGBR                = 3,
  NUMBER_INPUT_COLOUR_SPACE_CONVERSIONS = 4
};

enum MATRIX_COEFFICIENTS // Table E.5 (Matrix coefficients)
{
  MATRIX_COEFFICIENTS_RGB                           = 0,
  MATRIX_COEFFICIENTS_BT709                         = 1,
  MATRIX_COEFFICIENTS_UNSPECIFIED                   = 2,
  MATRIX_COEFFICIENTS_RESERVED_BY_ITUISOIEC         = 3,
  MATRIX_COEFFICIENTS_USFCCT47                      = 4,
  MATRIX_COEFFICIENTS_BT601_625                     = 5,
  MATRIX_COEFFICIENTS_BT601_525                     = 6,
  MATRIX_COEFFICIENTS_SMPTE240                      = 7,
  MATRIX_COEFFICIENTS_YCGCO                         = 8,
  MATRIX_COEFFICIENTS_BT2020_NON_CONSTANT_LUMINANCE = 9,
  MATRIX_COEFFICIENTS_BT2020_CONSTANT_LUMINANCE     = 10,
};

enum DeblockEdgeDir
{
  EDGE_VER     = 0,
  EDGE_HOR     = 1,
  NUM_EDGE_DIR = 2
};

/// supported prediction type
enum PredMode
{
  MODE_INTER                 = 0,     ///< inter-prediction mode
  MODE_INTRA                 = 1,     ///< intra-prediction mode
  MODE_IBC                   = 2,     ///< ibc-prediction mode
  MODE_PLT                   = 3,     ///< plt-prediction mode
  NUMBER_OF_PREDICTION_MODES = 4,
};

/// reference list index
enum RefPicList
{
  REF_PIC_LIST_0               = 0,   ///< reference list 0
  REF_PIC_LIST_1               = 1,   ///< reference list 1
  NUM_REF_PIC_LIST_01          = 2,
  REF_PIC_LIST_X               = 100  ///< special mark
};

#define L0 REF_PIC_LIST_0
#define L1 REF_PIC_LIST_1
#if (JVET_W0097_GPM_MMVD_TM && TM_MRG) || JVET_Y0065_GPM_INTRA
enum GeoTmMvCand
{
  GEO_TM_OFF = 0,
  GEO_TM_SHAPE_AL,
  GEO_TM_SHAPE_A,
  GEO_TM_SHAPE_L,
  GEO_NUM_TM_MV_CAND
};
#endif
/// distortion function index
enum DFunc
{
  DF_SSE             = 0,             ///< general size SSE
  DF_SSE2            = DF_SSE+1,      ///<   2xM SSE
  DF_SSE4            = DF_SSE+2,      ///<   4xM SSE
  DF_SSE8            = DF_SSE+3,      ///<   8xM SSE
  DF_SSE16           = DF_SSE+4,      ///<  16xM SSE
  DF_SSE32           = DF_SSE+5,      ///<  32xM SSE
  DF_SSE64           = DF_SSE+6,      ///<  64xM SSE
  DF_SSE16N          = DF_SSE+7,      ///< 16NxM SSE

  DF_SAD             = 8,             ///< general size SAD
  DF_SAD2            = DF_SAD+1,      ///<   2xM SAD
  DF_SAD4            = DF_SAD+2,      ///<   4xM SAD
  DF_SAD8            = DF_SAD+3,      ///<   8xM SAD
  DF_SAD16           = DF_SAD+4,      ///<  16xM SAD
  DF_SAD32           = DF_SAD+5,      ///<  32xM SAD
  DF_SAD64           = DF_SAD+6,      ///<  64xM SAD
  DF_SAD16N          = DF_SAD+7,      ///< 16NxM SAD

  DF_HAD             = 16,            ///< general size Hadamard
  DF_HAD2            = DF_HAD+1,      ///<   2xM HAD
  DF_HAD4            = DF_HAD+2,      ///<   4xM HAD
  DF_HAD8            = DF_HAD+3,      ///<   8xM HAD
  DF_HAD16           = DF_HAD+4,      ///<  16xM HAD
  DF_HAD32           = DF_HAD+5,      ///<  32xM HAD
  DF_HAD64           = DF_HAD+6,      ///<  64xM HAD
  DF_HAD16N          = DF_HAD+7,      ///< 16NxM HAD

  DF_SAD12           = 24,
  DF_SAD24           = 25,
  DF_SAD48           = 26,

  DF_MRSAD           = 27,            ///< general size MR SAD
  DF_MRSAD2          = DF_MRSAD+1,    ///<   2xM MR SAD
  DF_MRSAD4          = DF_MRSAD+2,    ///<   4xM MR SAD
  DF_MRSAD8          = DF_MRSAD+3,    ///<   8xM MR SAD
  DF_MRSAD16         = DF_MRSAD+4,    ///<  16xM MR SAD
  DF_MRSAD32         = DF_MRSAD+5,    ///<  32xM MR SAD
  DF_MRSAD64         = DF_MRSAD+6,    ///<  64xM MR SAD
  DF_MRSAD16N        = DF_MRSAD+7,    ///< 16NxM MR SAD

  DF_MRHAD           = 35,            ///< general size MR Hadamard
  DF_MRHAD2          = DF_MRHAD+1,    ///<   2xM MR HAD
  DF_MRHAD4          = DF_MRHAD+2,    ///<   4xM MR HAD
  DF_MRHAD8          = DF_MRHAD+3,    ///<   8xM MR HAD
  DF_MRHAD16         = DF_MRHAD+4,    ///<  16xM MR HAD
  DF_MRHAD32         = DF_MRHAD+5,    ///<  32xM MR HAD
  DF_MRHAD64         = DF_MRHAD+6,    ///<  64xM MR HAD
  DF_MRHAD16N        = DF_MRHAD+7,    ///< 16NxM MR HAD

  DF_MRSAD12         = 43,
  DF_MRSAD24         = 44,
  DF_MRSAD48         = 45,

  DF_SAD_FULL_NBIT    = 46,
  DF_SAD_FULL_NBIT2   = DF_SAD_FULL_NBIT+1,    ///<   2xM SAD with full bit usage
  DF_SAD_FULL_NBIT4   = DF_SAD_FULL_NBIT+2,    ///<   4xM SAD with full bit usage
  DF_SAD_FULL_NBIT8   = DF_SAD_FULL_NBIT+3,    ///<   8xM SAD with full bit usage
  DF_SAD_FULL_NBIT16  = DF_SAD_FULL_NBIT+4,    ///<  16xM SAD with full bit usage
  DF_SAD_FULL_NBIT32  = DF_SAD_FULL_NBIT+5,    ///<  32xM SAD with full bit usage
  DF_SAD_FULL_NBIT64  = DF_SAD_FULL_NBIT+6,    ///<  64xM SAD with full bit usage
  DF_SAD_FULL_NBIT16N = DF_SAD_FULL_NBIT+7,    ///< 16NxM SAD with full bit usage

  DF_SSE_WTD          = 54,                ///< general size SSE
  DF_SSE2_WTD         = DF_SSE_WTD+1,      ///<   4xM SSE
  DF_SSE4_WTD         = DF_SSE_WTD+2,      ///<   4xM SSE
  DF_SSE8_WTD         = DF_SSE_WTD+3,      ///<   8xM SSE
  DF_SSE16_WTD        = DF_SSE_WTD+4,      ///<  16xM SSE
  DF_SSE32_WTD        = DF_SSE_WTD+5,      ///<  32xM SSE
  DF_SSE64_WTD        = DF_SSE_WTD+6,      ///<  64xM SSE
  DF_SSE16N_WTD       = DF_SSE_WTD+7,      ///< 16NxM SSE
  DF_DEFAULT_ORI      = DF_SSE_WTD+8,

  DF_SAD_INTERMEDIATE_BITDEPTH = 63,

  DF_SAD_WITH_MASK   = 64,
#if TM_AMVP || TM_MRG || JVET_Z0084_IBC_TM
  DF_TM_A_WSAD_FULL_NBIT,
  DF_TM_L_WSAD_FULL_NBIT,
  DF_TM_A_WMRSAD_FULL_NBIT,
  DF_TM_L_WMRSAD_FULL_NBIT,
  DF_TOTAL_FUNCTIONS
#else
  DF_TOTAL_FUNCTIONS = 65
#endif
};

/// motion vector predictor direction used in AMVP
enum MvpDir
{
  MD_LEFT = 0,          ///< MVP of left block
  MD_ABOVE,             ///< MVP of above block
  MD_ABOVE_RIGHT,       ///< MVP of above right block
  MD_BELOW_LEFT,        ///< MVP of below left block
  MD_ABOVE_LEFT         ///< MVP of above left block
};

enum TransformDirection
{
  TRANSFORM_FORWARD              = 0,
  TRANSFORM_INVERSE              = 1,
  TRANSFORM_NUMBER_OF_DIRECTIONS = 2
};

/// supported ME search methods
enum MESearchMethod
{
  MESEARCH_FULL              = 0,
  MESEARCH_DIAMOND           = 1,
  MESEARCH_SELECTIVE         = 2,
  MESEARCH_DIAMOND_ENHANCED  = 3,
  MESEARCH_NUMBER_OF_METHODS = 4
};

/// coefficient scanning type used in ACS
enum CoeffScanType
{
  SCAN_DIAG = 0,        ///< up-right diagonal scan
  SCAN_TRAV_HOR = 1,
  SCAN_TRAV_VER = 2,
  SCAN_NUMBER_OF_TYPES
};

enum CoeffScanGroupType
{
  SCAN_UNGROUPED   = 0,
  SCAN_GROUPED_4x4 = 1,
  SCAN_NUMBER_OF_GROUP_TYPES = 2
};

enum ScalingListMode
{
  SCALING_LIST_OFF,
  SCALING_LIST_DEFAULT,
  SCALING_LIST_FILE_READ
};

enum ScalingListSize
{
  SCALING_LIST_1x1 = 0,
  SCALING_LIST_2x2,
  SCALING_LIST_4x4,
  SCALING_LIST_8x8,
  SCALING_LIST_16x16,
  SCALING_LIST_32x32,
  SCALING_LIST_64x64,
  SCALING_LIST_128x128,
  SCALING_LIST_SIZE_NUM,
  //for user define matrix
  SCALING_LIST_FIRST_CODED = SCALING_LIST_2x2,
  SCALING_LIST_LAST_CODED = SCALING_LIST_64x64
};

enum ScalingList1dStartIdx
{
  SCALING_LIST_1D_START_2x2    = 0,
  SCALING_LIST_1D_START_4x4    = 2,
  SCALING_LIST_1D_START_8x8    = 8,
  SCALING_LIST_1D_START_16x16  = 14,
  SCALING_LIST_1D_START_32x32  = 20,
  SCALING_LIST_1D_START_64x64  = 26,
};

// For use with decoded picture hash SEI messages, generated by encoder.
enum HashType
{
  HASHTYPE_MD5             = 0,
  HASHTYPE_CRC             = 1,
  HASHTYPE_CHECKSUM        = 2,
  HASHTYPE_NONE            = 3,
  NUMBER_OF_HASHTYPES      = 4
};

enum SAOMode //mode
{
  SAO_MODE_OFF = 0,
  SAO_MODE_NEW,
  SAO_MODE_MERGE,
  NUM_SAO_MODES
};

enum SAOModeMergeTypes
{
  SAO_MERGE_LEFT =0,
  SAO_MERGE_ABOVE,
  NUM_SAO_MERGE_TYPES
};


enum SAOModeNewTypes
{
  SAO_TYPE_START_EO =0,
  SAO_TYPE_EO_0 = SAO_TYPE_START_EO,
  SAO_TYPE_EO_90,
  SAO_TYPE_EO_135,
  SAO_TYPE_EO_45,

  SAO_TYPE_START_BO,
  SAO_TYPE_BO = SAO_TYPE_START_BO,

  NUM_SAO_NEW_TYPES
};
#define NUM_SAO_EO_TYPES_LOG2 2

enum SAOEOClasses
{
  SAO_CLASS_EO_FULL_VALLEY = 0,
  SAO_CLASS_EO_HALF_VALLEY = 1,
  SAO_CLASS_EO_PLAIN       = 2,
  SAO_CLASS_EO_HALF_PEAK   = 3,
  SAO_CLASS_EO_FULL_PEAK   = 4,
  NUM_SAO_EO_CLASSES,
};

#define NUM_SAO_BO_CLASSES_LOG2  5
#define NUM_SAO_BO_CLASSES       (1<<NUM_SAO_BO_CLASSES_LOG2)

#if JVET_AE0151_CCSAO_HISTORY_OFFSETS_AND_EXT_EO
enum CCSAOSetTypes
{
  CCSAO_SET_TYPE_BAND = 0,
  CCSAO_SET_TYPE_EDGE = 1,
  NUM_CCSAO_SET_TYPES = 2
};
#endif

namespace Profile
{
  enum Name
  {
#if JVET_S_PROFILES
    NONE                                 = 0,
    STILL_PICTURE                        = 64,
    MAIN_10                              = 1,
    MAIN_10_STILL_PICTURE                = MAIN_10 | STILL_PICTURE,
    MULTILAYER_MAIN_10                   = 17,
    MULTILAYER_MAIN_10_STILL_PICTURE     = MULTILAYER_MAIN_10 | STILL_PICTURE,
    MAIN_10_444                          = 33,
    MAIN_10_444_STILL_PICTURE            = MAIN_10_444 | STILL_PICTURE,
    MULTILAYER_MAIN_10_444               = 49,
    MULTILAYER_MAIN_10_444_STILL_PICTURE = MULTILAYER_MAIN_10_444 | STILL_PICTURE,
#else
    NONE        = 0,
    MAIN_10     = 1,
    MAIN_444_10 = 2
#endif
  };
}

namespace Level
{
  enum Tier
  {
    MAIN = 0,
    HIGH = 1,
    NUMBER_OF_TIERS=2
  };

  enum Name
  {
    // code = (major_level * 16 + minor_level * 3)
    NONE     = 0,
    LEVEL1   = 16,
    LEVEL2   = 32,
    LEVEL2_1 = 35,
    LEVEL3   = 48,
    LEVEL3_1 = 51,
    LEVEL4   = 64,
    LEVEL4_1 = 67,
    LEVEL5   = 80,
    LEVEL5_1 = 83,
    LEVEL5_2 = 86,
    LEVEL6   = 96,
    LEVEL6_1 = 99,
    LEVEL6_2 = 102,
    LEVEL15_5 = 255,
  };
}

enum CostMode
{
  COST_STANDARD_LOSSY              = 0,
  COST_SEQUENCE_LEVEL_LOSSLESS     = 1,
  COST_LOSSLESS_CODING             = 2,
  COST_MIXED_LOSSLESS_LOSSY_CODING = 3
};

enum WeightedPredictionMethod
{
  WP_PER_PICTURE_WITH_SIMPLE_DC_COMBINED_COMPONENT                          =0,
  WP_PER_PICTURE_WITH_SIMPLE_DC_PER_COMPONENT                               =1,
  WP_PER_PICTURE_WITH_HISTOGRAM_AND_PER_COMPONENT                           =2,
  WP_PER_PICTURE_WITH_HISTOGRAM_AND_PER_COMPONENT_AND_CLIPPING              =3,
  WP_PER_PICTURE_WITH_HISTOGRAM_AND_PER_COMPONENT_AND_CLIPPING_AND_EXTENSION=4
};

enum FastInterSearchMode
{
  FASTINTERSEARCH_DISABLED = 0,
  FASTINTERSEARCH_MODE1    = 1, // TODO: assign better names to these.
  FASTINTERSEARCH_MODE2    = 2,
  FASTINTERSEARCH_MODE3    = 3
};

enum SPSExtensionFlagIndex
{
  SPS_EXT__REXT           = 0,
//SPS_EXT__MVHEVC         = 1, //for use in future versions
//SPS_EXT__SHVC           = 2, //for use in future versions
  SPS_EXT__NEXT           = 3,
  NUM_SPS_EXTENSION_FLAGS = 8
};

enum PPSExtensionFlagIndex
{
  PPS_EXT__REXT           = 0,
//PPS_EXT__MVHEVC         = 1, //for use in future versions
//PPS_EXT__SHVC           = 2, //for use in future versions
  NUM_PPS_EXTENSION_FLAGS = 8
};

// TODO: Existing names used for the different NAL unit types can be altered to better reflect the names in the spec.
//       However, the names in the spec are not yet stable at this point. Once the names are stable, a cleanup
//       effort can be done without use of macros to alter the names used to indicate the different NAL unit types.
enum NalUnitType
{
  NAL_UNIT_CODED_SLICE_TRAIL = 0,   // 0
  NAL_UNIT_CODED_SLICE_STSA,        // 1
  NAL_UNIT_CODED_SLICE_RADL,        // 2
  NAL_UNIT_CODED_SLICE_RASL,        // 3

  NAL_UNIT_RESERVED_VCL_4,
  NAL_UNIT_RESERVED_VCL_5,
  NAL_UNIT_RESERVED_VCL_6,

  NAL_UNIT_CODED_SLICE_IDR_W_RADL,  // 7
  NAL_UNIT_CODED_SLICE_IDR_N_LP,    // 8
  NAL_UNIT_CODED_SLICE_CRA,         // 9
  NAL_UNIT_CODED_SLICE_GDR,         // 10

  NAL_UNIT_RESERVED_IRAP_VCL_11,
  NAL_UNIT_RESERVED_IRAP_VCL_12,
  NAL_UNIT_DCI,                     // 13
  NAL_UNIT_VPS,                     // 14
  NAL_UNIT_SPS,                     // 15
  NAL_UNIT_PPS,                     // 16
  NAL_UNIT_PREFIX_APS,              // 17
  NAL_UNIT_SUFFIX_APS,              // 18
  NAL_UNIT_PH,                      // 19
  NAL_UNIT_ACCESS_UNIT_DELIMITER,   // 20
  NAL_UNIT_EOS,                     // 21
  NAL_UNIT_EOB,                     // 22
  NAL_UNIT_PREFIX_SEI,              // 23
  NAL_UNIT_SUFFIX_SEI,              // 24
  NAL_UNIT_FD,                      // 25

  NAL_UNIT_RESERVED_NVCL_26,
  NAL_UNIT_RESERVED_NVCL_27,

  NAL_UNIT_UNSPECIFIED_28,
  NAL_UNIT_UNSPECIFIED_29,
  NAL_UNIT_UNSPECIFIED_30,
  NAL_UNIT_UNSPECIFIED_31,
  NAL_UNIT_INVALID
};

#if SHARP_LUMA_DELTA_QP
enum LumaLevelToDQPMode
{
  LUMALVL_TO_DQP_DISABLED   = 0,
  LUMALVL_TO_DQP_AVG_METHOD = 1, // use average of CTU to determine luma level
  LUMALVL_TO_DQP_NUM_MODES  = 2
};
#endif

enum MergeType
{
  MRG_TYPE_DEFAULT_N        = 0, // 0
  MRG_TYPE_SUBPU_ATMVP,
  MRG_TYPE_IBC,
  NUM_MRG_TYPE                   // 5
};

#if SIGN_PREDICTION
enum SIGN_PRED_TYPE : uint8_t
{
  SIGN_PRED_BYPASS   = 0,
  SIGN_PRED_POSITIVE = 1,
  SIGN_PRED_NEGATIVE = 2,
  SIGN_PRED_HIDDEN   = 3,
};
#endif

//////////////////////////////////////////////////////////////////////////
// Encoder modes to try out
//////////////////////////////////////////////////////////////////////////

enum EncModeFeature
{
  ENC_FT_FRAC_BITS = 0,
  ENC_FT_DISTORTION,
  ENC_FT_RD_COST,
  ENC_FT_ENC_MODE_TYPE,
  ENC_FT_ENC_MODE_OPTS,
  ENC_FT_ENC_MODE_PART,
  NUM_ENC_FEATURES
};

enum ImvMode
{
  IMV_OFF = 0,
  IMV_FPEL,
  IMV_4PEL,
  IMV_HPEL,
  NUM_IMV_MODES
};


// ====================================================================================================================
// Type definition
// ====================================================================================================================

/// parameters for adaptive loop filter
class PicSym;

#define MAX_NUM_SAO_CLASSES  32  //(NUM_SAO_EO_GROUPS > NUM_SAO_BO_GROUPS)?NUM_SAO_EO_GROUPS:NUM_SAO_BO_GROUPS

struct SAOOffset
{
  SAOMode modeIdc; // NEW, MERGE, OFF
  int typeIdc;     // union of SAOModeMergeTypes and SAOModeNewTypes, depending on modeIdc.
  int typeAuxInfo; // BO: starting band index
  int offset[MAX_NUM_SAO_CLASSES];

  SAOOffset();
  ~SAOOffset();
  void reset();

  const SAOOffset& operator= (const SAOOffset& src);
};

struct SAOBlkParam
{

  SAOBlkParam();
  ~SAOBlkParam();
  void reset();
  const SAOBlkParam& operator= (const SAOBlkParam& src);
  SAOOffset& operator[](int compIdx){ return offsetParam[compIdx];}
  const SAOOffset& operator[](int compIdx) const { return offsetParam[compIdx];}
private:
  SAOOffset offsetParam[MAX_NUM_COMPONENT];

};

#if JVET_V0094_BILATERAL_FILTER
class BifParams
{
public:
  int frmOn;                // slice_bif_enabled_flag
  int allCtuOn;             // slice_bif_all_ctb_enabled_flag
  int numBlocks;
  std::vector<int> ctuOn;   // bif_ctb_flag[][]
};
#endif

struct CclmModel
{
  // First model
  int a        = 0;
  int b        = 0;
  int shift    = 0;
  int midLuma  = 0;
  
#if MMLM
  // Second model
  int a2       = 0;
  int b2       = 0;
  int shift2   = 0;
  int midLuma2 = 0;
  int yThres   = 0;
#endif
  
  void setFirstModel (int xa, int xb, int xshift)           { a  = xa; b  = xb; shift  = xshift; }
#if MMLM
  void setSecondModel(int xa, int xb, int xshift, int xthr) { a2 = xa; b2 = xb; shift2 = xshift; yThres = xthr; }
#endif
};

#if JVET_Z0050_CCLM_SLOPE
struct CclmOffsets
{
  int8_t cb0 = 0;
  int8_t cr0 = 0;
  int8_t cb1 = 0;
  int8_t cr1 = 0;
  
  bool isActive() const                           { return cb0 || cr0 || cb1 || cr1;        }
  void setAllZero()                               { cb0 = 0;  cr0 = 0;  cb1 = 0;  cr1 = 0;  }
  void setOffsets(int b0, int r0, int b1, int r1) { cb0 = b0; cr0 = r0; cb1 = b1; cr1 = r1; }
  void setOffset(ComponentID c, int model, int v)
  {
    if ( c == COMPONENT_Cb )
    {
      if ( model == 0 )
      {
        cb0 = v;
      }
      else
      {
        cb1 = v;
      }
    }
    else
    {
      if ( model == 0 )
      {
        cr0 = v;
      }
      else
      {
        cr1 = v;
      }
    }
  }
};
#endif

#if JVET_AA0126_GLM
struct GlmIdc
{
  int8_t cb0 = 0, cr0 = 0;
  int8_t cb1 = 0, cr1 = 0;
  
  bool isActive()                         const { return cb0 || cr0 || cb1 || cr1;                                                           }
  bool isActive(ComponentID c)            const { return (c == COMPONENT_Cb) ? (cb0 || cb1) : (cr0 || cr1);                                  }
  void setAllZero()                             { cb0 = 0;  cr0 = 0;  cb1 = 0;  cr1 = 0;                                                     }
  void setIdcs(int b0, int r0, int b1, int r1)  { cb0 = b0; cr0 = r0; cb1 = b1; cr1 = r1;                                                    }
  void setIdc(ComponentID c, int model, int v)  { (c == COMPONENT_Cb) ? (model == 0 ? cb0 = v : cb1 = v) : (model == 0 ? cr0 = v : cr1 = v); }
  int  getIdc(ComponentID c, int model)   const { return (c == COMPONENT_Cb) ? (model == 0 ? cb0 : cb1) : (model == 0 ? cr0 : cr1);          }
};
#endif

struct BitDepths
{
  int recon[MAX_NUM_CHANNEL_TYPE]; ///< the bit depth as indicated in the SPS
};

enum PLTRunMode
{
  PLT_RUN_INDEX = 0,
  PLT_RUN_COPY  = 1,
  NUM_PLT_RUN   = 2
};
/// parameters for deblocking filter
struct LFCUParam
{
  bool internalEdge;                     ///< indicates internal edge
  bool leftEdge;                         ///< indicates left edge
  bool topEdge;                          ///< indicates top edge
};

#if JVET_Z0054_BLK_REF_PIC_REORDER
struct RefListAndRefIdx
{
  RefPicList refList;
  int8_t     refIdx;
  uint32_t   pocDist;
  Distortion cost;
};
struct RefPicPair
{
  int8_t refIdx[2];
  uint32_t pocDist;
  Distortion cost;
};
#endif

struct PictureHash
{
  std::vector<uint8_t> hash;

  bool operator==(const PictureHash &other) const
  {
    if (other.hash.size() != hash.size())
    {
      return false;
    }
    for(uint32_t i=0; i<uint32_t(hash.size()); i++)
    {
      if (other.hash[i] != hash[i])
      {
        return false;
      }
    }
    return true;
  }

  bool operator!=(const PictureHash &other) const
  {
    return !(*this == other);
  }
};

struct SEITimeSet
{
  SEITimeSet() : clockTimeStampFlag(false),
                     numUnitFieldBasedFlag(false),
                     countingType(0),
                     fullTimeStampFlag(false),
                     discontinuityFlag(false),
                     cntDroppedFlag(false),
                     numberOfFrames(0),
                     secondsValue(0),
                     minutesValue(0),
                     hoursValue(0),
                     secondsFlag(false),
                     minutesFlag(false),
                     hoursFlag(false),
                     timeOffsetLength(0),
                     timeOffsetValue(0)
  { }
  bool clockTimeStampFlag;
  bool numUnitFieldBasedFlag;
  int  countingType;
  bool fullTimeStampFlag;
  bool discontinuityFlag;
  bool cntDroppedFlag;
  int  numberOfFrames;
  int  secondsValue;
  int  minutesValue;
  int  hoursValue;
  bool secondsFlag;
  bool minutesFlag;
  bool hoursFlag;
  int  timeOffsetLength;
  int  timeOffsetValue;
};

struct SEIMasteringDisplay
{
  bool      colourVolumeSEIEnabled;
  uint32_t      maxLuminance;
  uint32_t      minLuminance;
  uint16_t    primaries[3][2];
  uint16_t    whitePoint[2];
};

#if SHARP_LUMA_DELTA_QP
struct LumaLevelToDeltaQPMapping
{
  LumaLevelToDQPMode                 mode;             ///< use deltaQP determined by block luma level
  double                             maxMethodWeight;  ///< weight of max luma value when mode = 2
  std::vector< std::pair<int, int> > mapping;          ///< first=luma level, second=delta QP.
#if ENABLE_QPA
  bool isEnabled() const { return (mode != LUMALVL_TO_DQP_DISABLED && mode != LUMALVL_TO_DQP_NUM_MODES); }
#else
  bool isEnabled() const { return mode!=LUMALVL_TO_DQP_DISABLED; }
#endif
};
#endif

#if ER_CHROMA_QP_WCG_PPS
struct WCGChromaQPControl
{
  bool isEnabled() const { return enabled; }
  bool   enabled;         ///< Enabled flag (0:default)
  double chromaCbQpScale; ///< Chroma Cb QP Scale (1.0:default)
  double chromaCrQpScale; ///< Chroma Cr QP Scale (1.0:default)
  double chromaQpScale;   ///< Chroma QP Scale (0.0:default)
  double chromaQpOffset;  ///< Chroma QP Offset (0.0:default)
};
#endif

class ChromaCbfs
{
public:
  ChromaCbfs()
    : Cb(true), Cr(true)
  {}
  ChromaCbfs( bool _cbf )
    : Cb( _cbf ), Cr( _cbf )
  {}
public:
  bool sigChroma( ChromaFormat chromaFormat ) const
  {
    if( chromaFormat == CHROMA_400 )
    {
      return false;
    }
    return   ( Cb || Cr );
  }
  bool& cbf( ComponentID compID )
  {
    bool *cbfs[MAX_NUM_TBLOCKS] = { nullptr, &Cb, &Cr };

    return *cbfs[compID];
  }
public:
  bool Cb;
  bool Cr;
};


enum MsgLevel
{
  SILENT  = 0,
  ERROR   = 1,
  WARNING = 2,
  INFO    = 3,
  NOTICE  = 4,
  VERBOSE = 5,
  DETAILS = 6
};
enum RESHAPE_SIGNAL_TYPE
{
  RESHAPE_SIGNAL_SDR = 0,
  RESHAPE_SIGNAL_PQ  = 1,
  RESHAPE_SIGNAL_HLG = 2,
  RESHAPE_SIGNAL_NULL = 100,
};

#if JVET_AG0058_EIP
enum EIP_REFERENCE_TYPE
{
  EIP_A = 0,
  EIP_AL = 1,
  EIP_L = 2,
  EIP_AL_A = 3,
  EIP_AL_L = 4,
  EIP_A_L = 5,
  EIP_AL_A_L = 6,
  NUM_EIP_TPL_TYPE = 7,
};

enum EIP_FILTER_TYPE
{
  EIP_FILTER_S  = 0,
  EIP_FILTER_V  = 1,
  EIP_FILTER_H  = 2,
  NUM_EIP_SHAPE = 3,
};

struct EIPInfo
{
  int recoType;
  int filterShape;

  EIPInfo() : recoType(0), filterShape(0) {}
  EIPInfo(int _recoType, int _filterShape) : recoType(_recoType), filterShape(_filterShape) {}
};
#endif

#if JVET_AB0155_SGPM && !JVET_AG0152_SGPM_ITMP_IBC
struct SgpmInfo
{
  int sgpmSplitDir;
  int sgpmMode0;
  int sgpmMode1;

  SgpmInfo() : sgpmSplitDir(0), sgpmMode0(0), sgpmMode1(0) {}
  SgpmInfo(const int sd, const int sm0, const int sm1) : sgpmSplitDir(sd), sgpmMode0(sm0), sgpmMode1(sm1) {}
  SgpmInfo &operator=(const SgpmInfo &other)
  {
    sgpmSplitDir = other.sgpmSplitDir;
    sgpmMode0    = other.sgpmMode0;
    sgpmMode1    = other.sgpmMode1;
    return *this;
  }
};
#endif

#if JVET_AG0112_REGRESSION_BASED_GPM_BLENDING
struct AffineBlendingModel
{
  bool  valid;
  int params[3];
  int shift;
  int offset;
  int min, max;

  void init()
  {
    valid = false;
    shift = 1;
    params[0] = 1 << (shift - 1);
    params[1] = 1 << (shift - 1);
    params[2] = 0;
  }

  AffineBlendingModel()
  {
    init();
  }

  AffineBlendingModel( const int s, const int _min, const int _max )
  {
    shift = s;
    params[0] = 1 << (shift - 1);
    params[1] = 1 << (shift - 1);
    params[2] = 0;
    min = _min;
    max = _max;
  }
  void copy( const AffineBlendingModel& other )
  {
    if ( other.valid )
    {
      *this = other;
    }
    else
    {
      init();
    }
  }

  int compute( const int x, const int y, bool bClip=true )
  {
    int weight = (params[0] * x + params[1] * y + params[2] + offset) >> shift ;
    if ( bClip )
    {
      return (weight > max ? max : (weight < min ? min : weight));
    }
    else
    {
      return weight;
    }
  }

  bool isSame( const AffineBlendingModel& other )
  {
    bool  bSame = true;

    bSame &= other.valid == valid;
    bSame &= other.params[0] == params[0];
    bSame &= other.params[1] == params[1];
    bSame &= other.params[2] == params[2];
    bSame &= other.shift == shift;
    bSame &= other.offset == offset;
    bSame &= other.min == min;
    bSame &= other.max == max;

    return bSame;
  }
};
#endif

// ---------------------------------------------------------------------------
// exception class
// ---------------------------------------------------------------------------

class Exception : public std::exception
{
public:
  Exception( const std::string& _s ) : m_str( _s ) { }
  Exception( const Exception& _e ) : std::exception( _e ), m_str( _e.m_str ) { }
  virtual ~Exception() noexcept { };
  virtual const char* what() const noexcept { return m_str.c_str(); }
  Exception& operator=( const Exception& _e ) { std::exception::operator=( _e ); m_str = _e.m_str; return *this; }
  template<typename T> Exception& operator<<( T t ) { std::ostringstream oss; oss << t; m_str += oss.str(); return *this; }
private:
  std::string m_str;
};

// if a check fails with THROW or CHECK, please check if ported correctly from assert in revision 1196)
#define THROW(x)            throw( Exception( "\nERROR: In function \"" ) << __FUNCTION__ << "\" in " << __FILE__ << ":" << __LINE__ << ": " << x )
#define CHECK(c,x)          if(c){ THROW(x); }
#define EXIT(x)             throw( Exception( "\n" ) << x << "\n" )
#define CHECK_NULLPTR(_ptr) CHECK( !( _ptr ), "Accessing an empty pointer pointer!" )

#if !NDEBUG  // for non MSVC compiler, define _DEBUG if in debug mode to have same behavior between MSVC and others in debug
#ifndef _DEBUG
#define _DEBUG 1
#endif
#endif

#if defined( _DEBUG )
#define CHECKD(c,x)         if(c){ THROW(x); }
#else
#define CHECKD(c,x)
#endif // _DEBUG

// ---------------------------------------------------------------------------
// static vector
// ---------------------------------------------------------------------------

template<typename T, size_t N>
class static_vector
{
  T _arr[ N ];
  size_t _size;

public:

  typedef T         value_type;
  typedef size_t    size_type;
  typedef ptrdiff_t difference_type;
  typedef T&        reference;
  typedef T const&  const_reference;
  typedef T*        pointer;
  typedef T const*  const_pointer;
  typedef T*        iterator;
  typedef T const*  const_iterator;

  static const size_type max_num_elements = N;

  static_vector() : _size( 0 )                                 { }
  static_vector( size_t N_ ) : _size( N_ )                     { }
  static_vector( size_t N_, const T& _val ) : _size( 0 )       { resize( N_, _val ); }
  template<typename It>
  static_vector( It _it1, It _it2 ) : _size( 0 )               { while( _it1 < _it2 ) _arr[ _size++ ] = *_it1++; }
  static_vector( std::initializer_list<T> _il ) : _size( 0 )
  {
    typename std::initializer_list<T>::iterator _src1 = _il.begin();
    typename std::initializer_list<T>::iterator _src2 = _il.end();

    while( _src1 < _src2 ) _arr[ _size++ ] = *_src1++;

    CHECKD( _size > N, "capacity exceeded" );
  }
  static_vector& operator=( std::initializer_list<T> _il )
  {
    _size = 0;

    typename std::initializer_list<T>::iterator _src1 = _il.begin();
    typename std::initializer_list<T>::iterator _src2 = _il.end();

    while( _src1 < _src2 ) _arr[ _size++ ] = *_src1++;

    CHECKD( _size > N, "capacity exceeded" );
  }

  void resize( size_t N_ )                      { CHECKD( N_ > N, "capacity exceeded" ); while(_size < N_) _arr[ _size++ ] = T() ; _size = N_; }
  void resize( size_t N_, const T& _val )       { CHECKD( N_ > N, "capacity exceeded" ); while(_size < N_) _arr[ _size++ ] = _val; _size = N_; }
  void reserve( size_t N_ )                     { CHECKD( N_ > N, "capacity exceeded" ); }
  void push_back( const T& _val )               { CHECKD( _size >= N, "capacity exceeded" ); _arr[ _size++ ] = _val; }
  void push_back( T&& val )                     { CHECKD( _size >= N, "capacity exceeded" ); _arr[ _size++ ] = std::forward<T>( val ); }
  void pop_back()                               { CHECKD( _size == 0, "calling pop_back on an empty vector" ); _size--; }
  void pop_front()                              { CHECKD( _size == 0, "calling pop_front on an empty vector" ); _size--; for( int i = 0; i < _size; i++ ) _arr[i] = _arr[i + 1]; }
  void clear()                                  { _size = 0; }
  reference       at( size_t _i )               { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  const_reference at( size_t _i ) const         { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  reference       operator[]( size_t _i )       { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  const_reference operator[]( size_t _i ) const { CHECKD( _i >= _size, "Trying to access an out-of-bound-element" ); return _arr[ _i ]; }
  reference       front()                       { CHECKD( _size == 0, "Trying to access the first element of an empty vector" ); return _arr[ 0 ]; }
  const_reference front() const                 { CHECKD( _size == 0, "Trying to access the first element of an empty vector" ); return _arr[ 0 ]; }
  reference       back()                        { CHECKD( _size == 0, "Trying to access the last element of an empty vector" );  return _arr[ _size - 1 ]; }
  const_reference back() const                  { CHECKD( _size == 0, "Trying to access the last element of an empty vector" );  return _arr[ _size - 1 ]; }
  pointer         data()                        { return _arr; }
  const_pointer   data() const                  { return _arr; }
  iterator        begin()                       { return _arr; }
  const_iterator  begin() const                 { return _arr; }
  const_iterator  cbegin() const                { return _arr; }
  iterator        end()                         { return _arr + _size; }
  const_iterator  end() const                   { return _arr + _size; };
  const_iterator  cend() const                  { return _arr + _size; };
  size_type       size() const                  { return _size; };
  size_type       byte_size() const             { return _size * sizeof( T ); }
  bool            empty() const                 { return _size == 0; }

  size_type       capacity() const              { return N; }
  size_type       max_size() const              { return N; }
  size_type       byte_capacity() const         { return sizeof(_arr); }

  iterator        insert( const_iterator _pos, const T& _val )
                                                { CHECKD( _size >= N, "capacity exceeded" );
                                                  for( difference_type i = _size - 1; i >= _pos - _arr; i-- ) _arr[i + 1] = _arr[i];
                                                  *const_cast<iterator>( _pos ) = _val;
                                                  _size++;
                                                  return const_cast<iterator>( _pos ); }

  iterator        insert( const_iterator _pos, T&& _val )
                                                { CHECKD( _size >= N, "capacity exceeded" );
                                                  for( difference_type i = _size - 1; i >= _pos - _arr; i-- ) _arr[i + 1] = _arr[i];
                                                  *const_cast<iterator>( _pos ) = std::forward<T>( _val );
                                                  _size++; return const_cast<iterator>( _pos ); }
  template<class InputIt>
  iterator        insert( const_iterator _pos, InputIt first, InputIt last )
                                                { const difference_type numEl = last - first;
                                                  CHECKD( _size + numEl > N, "capacity exceeded" );
                                                  for( difference_type i = _size - 1; i >= _pos - _arr; i-- ) _arr[i + numEl] = _arr[i];
                                                  iterator it = const_cast<iterator>( _pos ); _size += numEl;
                                                  while( first != last ) *it++ = *first++;
                                                  return const_cast<iterator>( _pos ); }

  iterator        insert( const_iterator _pos, size_t numEl, const T& val )
                                                { //const difference_type numEl = last - first;
                                                  CHECKD( _size + numEl > N, "capacity exceeded" );
                                                  for( difference_type i = _size - 1; i >= _pos - _arr; i-- ) _arr[i + numEl] = _arr[i];
                                                  iterator it = const_cast<iterator>( _pos ); _size += numEl;
                                                  for ( int k = 0; k < numEl; k++) *it++ = val;
                                                  return const_cast<iterator>( _pos ); }

  void            erase( const_iterator _pos )  { iterator it   = const_cast<iterator>( _pos ) - 1;
                                                  iterator last = end() - 1;
                                                  while( ++it != last ) *it = *( it + 1 );
                                                  _size--; }
};


// ---------------------------------------------------------------------------
// dynamic cache
// ---------------------------------------------------------------------------

template<typename T>
class dynamic_cache
{
  std::vector<T*> m_cache;
#if ENABLE_SPLIT_PARALLELISM
  int64_t         m_cacheId;
#endif

public:

#if ENABLE_SPLIT_PARALLELISM
  dynamic_cache()
  {
    static int cacheId = 0;
    m_cacheId = cacheId++;
  }

#endif
  ~dynamic_cache()
  {
    deleteEntries();
  }

  void deleteEntries()
  {
    for( auto &p : m_cache )
    {
      delete p;
      p = nullptr;
    }

    m_cache.clear();
  }

  T* get()
  {
    T* ret;

    if( !m_cache.empty() )
    {
      ret = m_cache.back();
      m_cache.pop_back();
#if ENABLE_SPLIT_PARALLELISM
      CHECK( ret->cacheId != m_cacheId, "Putting item into wrong cache!" );
      CHECK( !ret->cacheUsed,           "Fetched an element that should've been in cache!!" );
#endif
    }
    else
    {
      ret = new T;
    }

#if ENABLE_SPLIT_PARALLELISM
    ret->cacheId   = m_cacheId;
    ret->cacheUsed = false;

#endif
    return ret;
  }

  void cache( T* el )
  {
#if ENABLE_SPLIT_PARALLELISM
    CHECK( el->cacheId != m_cacheId, "Putting item into wrong cache!" );
    CHECK( el->cacheUsed,            "Putting cached item back into cache!" );

    el->cacheUsed = true;

#endif
    m_cache.push_back( el );
  }

  void cache( std::vector<T*>& vel )
  {
#if ENABLE_SPLIT_PARALLELISM
    for( auto el : vel )
    {
      CHECK( el->cacheId != m_cacheId, "Putting item into wrong cache!" );
      CHECK( el->cacheUsed,            "Putting cached item back into cache!" );

      el->cacheUsed = true;
    }

#endif
    m_cache.insert( m_cache.end(), vel.begin(), vel.end() );
    vel.clear();
  }
};

typedef dynamic_cache<struct CodingUnit    > CUCache;
typedef dynamic_cache<struct PredictionUnit> PUCache;
typedef dynamic_cache<struct TransformUnit > TUCache;

struct XUCache
{
  CUCache cuCache;
  PUCache puCache;
  TUCache tuCache;
};

#define SIGN(x) ( (x) >= 0 ? 1 : -1 )


//! \}

#endif


