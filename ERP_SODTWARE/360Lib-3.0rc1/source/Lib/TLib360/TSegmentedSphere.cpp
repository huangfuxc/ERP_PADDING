/* The copyright in this software is being made available under the BSD
* License, included below. This software may be subject to other third party
* and contributor rights, including patent rights, and no such rights are
* granted under this license.
*
* Copyright (c) 2010-2015, ITU/ISO/IEC
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

/** \file     TSegmentedSphere.cpp
    \brief    SegmentedSphere class
*/

#include <assert.h>
#include <math.h>
#include "TSegmentedSphere.h"

#if EXTENSION_360_VIDEO

/***********************************************
  Segmented Sphere geometry related functions;
***********************************************/

TSegmentedSphere::TSegmentedSphere(SVideoInfo& sVideoInfo, InputGeoParam *pInGeoParam) : TCubeMap(sVideoInfo, pInGeoParam)
{
    assert(sVideoInfo.geoType == SVIDEO_SEGMENTEDSPHERE);
    //assert(sVideoInfo.iNumFaces == 6);
    geoInit(sVideoInfo, pInGeoParam);
}

TSegmentedSphere::~TSegmentedSphere()
{}

Void TSegmentedSphere::map2DTo3D(SPos& IPosIn, SPos *pSPosOut)
{
    POSType u, v,
        square = m_sVideoInfo.iFaceHeight,
        width = m_sVideoInfo.iFaceWidth,
        pitch, yaw;
    //u = IPosIn.x;
    u = IPosIn.x + (POSType)(0.5);
    v = IPosIn.y + (POSType)(0.5);

#if SVIDEO_SSP_PADDING_FIX
#else
    if ((u < 0 || u >= m_sVideoInfo.iFaceWidth) && (v >= 0 && v < m_sVideoInfo.iFaceHeight * 2 / 3))
    {
        u = u < 0 ? m_sVideoInfo.iFaceWidth + u : (u - m_sVideoInfo.iFaceWidth);
    }
    else if (v < 0)
    {
        v = -v;
        u = u + (m_sVideoInfo.iFaceWidth >> 1);
        u = u >= m_sVideoInfo.iFaceWidth ? u - m_sVideoInfo.iFaceWidth : u;
    }
    else if (v >= m_sVideoInfo.iFaceHeight)
    {
        v = (m_sVideoInfo.iFaceHeight * 2 / 3 << 1) - v;
        u = u + (m_sVideoInfo.iFaceWidth >> 1);
        u = u >= m_sVideoInfo.iFaceWidth ? u - m_sVideoInfo.iFaceWidth : u;
    }
#endif
    POSType pole_x, pole_y, pole_d;
    if (IPosIn.faceIdx == 0)
    {
        pole_x = u - width / 2;
        pole_y = v - square / 2;
        pole_d = ssqrt(pole_x*pole_x + pole_y*pole_y);
        yaw = (pole_d > 0) ? acos(pole_y / pole_d) : 0;
        yaw = (pole_x < 0) ? S_PI*2 - yaw : yaw;
        pitch = S_PI_2 - pole_d * S_PI_2 / square;
    }
    else if (IPosIn.faceIdx == 1)
    {
        pole_x = u - width / 2;
        pole_y = v - square / 2;
        pole_d = ssqrt(pole_x*pole_x + pole_y*pole_y);
        yaw = (pole_d > 0) ? satan2(pole_y, pole_x) + S_PI_2 : 0;
        pitch = pole_d * S_PI_2 / square - S_PI_2;
    }
    else if (IPosIn.faceIdx == 2)
    {
        yaw = (POSType)(u*S_PI_2 / m_sVideoInfo.iFaceWidth - S_PI);
        pitch = (POSType)(S_PI_2 - (v + square / 2)*S_PI / square / 2);
    }
    else if (IPosIn.faceIdx == 3)
    {
        yaw = (POSType)((u + square)*S_PI_2 / m_sVideoInfo.iFaceWidth - S_PI);
        pitch = (POSType)(S_PI_2 - (v + square / 2)*S_PI / square / 2);
    }
    else if (IPosIn.faceIdx == 4)
    {
        yaw = (POSType)((u + 2*square)*S_PI_2 / m_sVideoInfo.iFaceWidth - S_PI);
        pitch = (POSType)(S_PI_2 - (v + square / 2)*S_PI / square / 2);
    }
    else if (IPosIn.faceIdx == 5)
    {
        yaw = (POSType)((u + 3*square)*S_PI_2 / m_sVideoInfo.iFaceWidth - S_PI);
        pitch = (POSType)(S_PI_2 - (v + square / 2)*S_PI / square / 2);
    }
    else
    {
        assert(!"Face index Error!\n");
    }

    pSPosOut->faceIdx = IPosIn.faceIdx;
    pSPosOut->x = (POSType)(scos(pitch)*scos(yaw));
    pSPosOut->y = (POSType)(ssin(pitch));
    pSPosOut->z = -(POSType)(scos(pitch)*ssin(yaw));
}

Void TSegmentedSphere::map3DTo2D(SPos *pSPosIn, SPos *pSPosOut)
{
    POSType x = pSPosIn->x;
    POSType y = pSPosIn->y;
    POSType z = pSPosIn->z;

    POSType len = ssqrt(x*x + y*y + z*z);
    POSType square = m_sVideoInfo.iFaceHeight;
    POSType yaw = (POSType)(satan2(-z, x)),
        pitch = (POSType)(sasin(y / len));

    pSPosOut->z = 0;
    if (y > len*ssqrt(2) / 2)
    {
        pSPosOut->faceIdx = 0;
        pSPosOut->x = square * ssin(yaw) * (S_PI_2 - pitch) / S_PI_2 +
            m_sVideoInfo.iFaceWidth / 2 - 0.5;
        pSPosOut->y = square / 2 * (1 + scos(yaw) * 2 * (S_PI_2 - pitch) / S_PI_2) - 0.5;
    }
    else if (y < -len*ssqrt(2) / 2)
    {
        pSPosOut->faceIdx = 1;
        pSPosOut->x = square * ssin(yaw) * (S_PI_2 + pitch) / S_PI_2 +
            m_sVideoInfo.iFaceWidth / 2 - 0.5;
        pSPosOut->y = square / 2 * (1 - scos(yaw) * 2 * (S_PI_2 + pitch) / S_PI_2) - 0.5;
    }
    else if (z >= 0 && x < 0)
    {
        pSPosOut->faceIdx = 2;
        pSPosOut->x = (S_PI + yaw)*m_sVideoInfo.iFaceWidth / S_PI_2 - 0.5;
        pSPosOut->y = (S_PI_2 - pitch) * square / S_PI_2 - square / 2 - 0.5;
    }
    else if (z > 0 && x >= 0)
    {
        pSPosOut->faceIdx = 3;
        pSPosOut->x = (S_PI + yaw)*m_sVideoInfo.iFaceWidth / S_PI_2 - m_sVideoInfo.iFaceWidth - 0.5;
        pSPosOut->y = (S_PI_2 - pitch) * square / S_PI_2 - square / 2 - 0.5;
    }
    else if (z <= 0 && x > 0)
    {
        pSPosOut->faceIdx = 4;
        pSPosOut->x = (S_PI + yaw)*m_sVideoInfo.iFaceWidth / S_PI_2 - 2 * m_sVideoInfo.iFaceWidth - 0.5;
        pSPosOut->y = (S_PI_2 - pitch) * square / S_PI_2 - square / 2 - 0.5;
    }
    else if (z < 0 && x <= 0)
    {
        pSPosOut->faceIdx = 5;
        pSPosOut->x = (S_PI + yaw)*m_sVideoInfo.iFaceWidth / S_PI_2 - 3 * m_sVideoInfo.iFaceWidth - 0.5;
        pSPosOut->y = (S_PI_2 - pitch) * square / S_PI_2 - square / 2 - 0.5;
    }
}

#if SVIDEO_SSP_VERT
//90 anti clockwise: source -> destination;
/*Void TSegmentedSphere::rot90(Pel *pSrcBuf, Int iStrideSrc, Int iWidth, Int iHeight, Int iNumSamples, Pel *pDst, Int iStrideDst)
{
    Pel *pSrcCol = pSrcBuf + (iWidth - 1)*iNumSamples;
    for (Int j = 0; j<iWidth; j++)
    {
        Pel *pSrc = pSrcCol;
        for (Int i = 0; i<iHeight; i++, pSrc += iStrideSrc)
        {
            memcpy(pDst + i*iNumSamples, pSrc, iNumSamples * sizeof(Pel));
        }
        pDst += iStrideDst;
        pSrcCol -= iNumSamples;
    }
}*/

Int TSegmentedSphere::getRot(Int faceIdx)
{
    return m_sVideoInfo.framePackStruct.faces[m_facePos[faceIdx][0]][m_facePos[faceIdx][1]].rot;
}
#endif

Bool TSegmentedSphere::insideFace(Int fId, Int x, Int y, ComponentID chId, ComponentID origchId)
{
    assert(m_sVideoInfo.iFaceHeight == m_sVideoInfo.iFaceWidth && "SSP Face shall be square.");
#if SVIDEO_SSP_VERT
    Int margin = 0;
#else
    Int margin = 4;
#endif
    if (
        y<0 || y >= (m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId)) ||
        x<0 || x >= (m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId))
        )
        return false;

    if (fId == 0 || fId == 1)
    {
        Int radius = (m_sVideoInfo.iFaceHeight >> 1);
        Double x_L = radius - (x << getComponentScaleX(chId)) - 0.5;
        Double y_L = radius - (y << getComponentScaleY(chId)) - 0.5;
        Double d = ssqrt(x_L*x_L + y_L*y_L);
        
        if (m_chromaFormatIDC == CHROMA_444 || origchId == COMPONENT_Y)
        {
            return d <= radius + margin;
        }
        else  // chroma in 420
        {
            return d/2 <= (radius + margin)>>1;
        }
    }
    else
    {
        return (x >= 0 && x<(m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId)) && y >= 0 && y<(m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId))); 
    }
}

#if SVIDEO_SSP_PADDING_FIX
Void TSegmentedSphere::spherePadding(Bool bEnforced)
{
    if (!bEnforced && m_bPadded)
    {
        return;
    }
    m_bPadded = false;

#if SVIDEO_DEBUG
    //dump to file;
    static Bool bFirstDumpBeforePading = true;
    dumpAllFacesToFile("equirect_before_padding", false, !bFirstDumpBeforePading);
    bFirstDumpBeforePading = false;
#endif

    TGeometry::spherePadding(bEnforced);

    Int nFaces = m_sVideoInfo.iNumFaces;

    for (Int ch = 0; ch<(getNumChannels()); ch++)
    {
        ComponentID chId = (ComponentID)ch;
        Int nWidth = m_sVideoInfo.iFaceWidth >> getComponentScaleX(chId);
        Int nHeight = m_sVideoInfo.iFaceHeight >> getComponentScaleY(chId);
        Int nMarginX = m_iMarginX >> getComponentScaleX(chId);
        Int iStride = getStride(ComponentID(ch));

        //Equatorial area, face 2,3,4,5
        Pel **pSrc = new Pel*[nFaces];
        Pel **pDst = new Pel*[nFaces];
        for (Int faceIdx = 0; faceIdx<nFaces; faceIdx++)
        {
            pSrc[faceIdx] = m_pFacesOrig[faceIdx][ch];
            pDst[faceIdx] = pSrc[faceIdx] + nWidth;
        }
        
        sPadH(pSrc[3], pDst[2], nMarginX, nHeight, iStride);
        sPadH(pSrc[4], pDst[3], nMarginX, nHeight, iStride);
        sPadH(pSrc[5], pDst[4], nMarginX, nHeight, iStride);
        sPadH(pSrc[2], pDst[5], nMarginX, nHeight, iStride);

        delete[] pSrc;
        delete[] pDst;
    }
    m_bPadded = true;

#if SVIDEO_DEBUG
    //dump to file;
    static Bool bFirstDumpAfterPading = true;
    dumpAllFacesToFile("equirect_after_padding", true, !bFirstDumpAfterPading);
    bFirstDumpAfterPading = false;
#endif
}

Void TSegmentedSphere::sPadH(Pel *pSrc, Pel *pDst, Int iCount, Int iVCnt, Int iStride)
{
    for (Int j = 0; j < iVCnt; j++)
    {
        for (Int i = 1; i <= iCount; i++)
        {
            pDst[i - 1] = pSrc[i - 1];
            pSrc[-i] = pDst[-i];
        }
        pSrc += iStride;
        pDst += iStride;
    }
}
#endif
#endif