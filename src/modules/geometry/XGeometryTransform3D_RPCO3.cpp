//*************************************************************************
//
// Copyright (C) 2015, LIESMARS, Wuhan University
//
// License:  LGPL
//
//*************************************************************************
//
// $Id: orsXGeometryTransform3D.cpp 2008/10/30 11:42 JWS $
//
// Project: OpenRS 
//
// Purpose:  
//
// Author: JIANG Wanshou, jws@lmars.whu.edu.cn
//
//*************************************************************************
//
// $Log: orsXGeometryTransform3D.cpp,v $
//
// Revision 1.0 date: 2008/10/30 by JIANG Wanshou
// new
//
#include "stdafx.h"
#include <math.h>
#include "orsMath/orsIMatrixService.h"
#include "orsXGeometryTransform3D.h"
#include "orsImage/orsIImageService.h"
#include "orsImageGeometry/orsIImageGeometryService.h"
#include "orsGeoData/orsIGeoDataService.h"

#include "orsImage/orsIAlgImageStatistics.h"

#include "orsImage/orsIImageSource.h"

ORS_GET_IMAGE_SERVICE_IMPL();
ORS_GET_IMAGEGEOMETRY_SERVICE_IMPL();
ORS_GET_GEODATA_SERVICE_IMPL();

orsPOINT2D Average(const orsPOINT2D *pts, int n);

orsPOINT3D Average3D(const orsPOINT3D *pts, int n);



///////////////////////////////////////////////////////////////////////////////
//		  a1+a2*( xSrc-xcSrc)+a3*(ySrc-ycSrc)+a4*(zSrc-zcSrc)+a5*( xSrc-xcSrc)*(ySrc-ycSrc)+
//        a6*( xSrc-xcSrc)*(zSrc-zcSrc)+a7*(ySrc-ycSrc)*(zSrc-zcSrc)+a8*( xSrc-xcSrc)*( xSrc-xcSrc)+
//        a9*(ySrc-ycSrc)*(ySrc-ycSrc)+a10*(zSrc-zcSrc)*(zSrc-zcSrc)+a11*( xSrc-xcSrc)*( xSrc-xcSrc)*( xSrc-xcSrc)+
//        a12*(ySrc-ycSrc)*(ySrc-ycSrc)*(ySrc-ycSrc)+a13*(zSrc-zcSrc)*(zSrc-zcSrc)*(zSrc-zcSrc)+
//        a14*( xSrc-xcSrc)*(ySrc-ycSrc)*(zSrc-zcSrc)+a15*( xSrc-xcSrc)*( xSrc-xcSrc)*(ySrc-ycSrc)+
//        a16*( xSrc-xcSrc)*( xSrc-xcSrc)*(zSrc-zcSrc)+a17*( xSrc-xcSrc)*(ySrc-ycSrc)*(ySrc-ycSrc)+
//        a18*(ySrc-ycSrc)*(ySrc-ycSrc)*(zSrc-zcSrc)+a19*( xSrc-xcSrc)*(zSrc-zcSrc)*(zSrc-zcSrc)+
//        a20*(ySrc-ycSrc)*(zSrc-zcSrc)*(zSrc-zcSrc)
// xDst - xcDst = ------------------------------------------------------------------------------------
//		  b1+b2*( xSrc-xcSrc)+b3*(ySrc-ycSrc)+b4*(zSrc-zcSrc)+b5*( xSrc-xcSrc)*(ySrc-ycSrc)+
//        b6*( xSrc-xcSrc)*(zSrc-zcSrc)+b7*(ySrc-ycSrc)*(zSrc-zcSrc)+b8*( xSrc-xcSrc)*( xSrc-xcSrc)+
//        b9*(ySrc-ycSrc)*(ySrc-ycSrc)+b10*(zSrc-zcSrc)*(zSrc-zcSrc)+b11*( xSrc-xcSrc)*( xSrc-xcSrc)*( xSrc-xcSrc)+
//        b12*(ySrc-ycSrc)*(ySrc-ycSrc)*(ySrc-ycSrc)+b13*(zSrc-zcSrc)*(zSrc-zcSrc)*(zSrc-zcSrc)+
//        b14*( xSrc-xcSrc)*(ySrc-ycSrc)*(zSrc-zcSrc)+b15*( xSrc-xcSrc)*( xSrc-xcSrc)*(ySrc-ycSrc)+
//        b16*( xSrc-xcSrc)*( xSrc-xcSrc)*(zSrc-zcSrc)+b17*( xSrc-xcSrc)*(ySrc-ycSrc)*(ySrc-ycSrc)+
//        b18*(ySrc-ycSrc)*(ySrc-ycSrc)*(zSrc-zcSrc)+b19*( xSrc-xcSrc)*(zSrc-zcSrc)*(zSrc-zcSrc)+
//        b20*(ySrc-ycSrc)*(zSrc-zcSrc)*(zSrc-zcSrc)
//
//
//		          c1+c2*L+c3*P+c4*H+c5*L*P+c6*L*H+c7*P*H+c8*L*L+c9*P*P+c10*H*H+c11*P*L*H+c12*L*L*L+
//                c13*L*P*P+c14*L*H*H+c15*L*L*P+c16*P*P*P+c17*P*H*H+c18*L*L*H+c19*P*P*H+c20*H*H*H
// yDst - ycDst = ------------------------------------------------------------------------------------
//		          d1+d2*L+d3*P+d4*H+d5*L*P+d6*L*H+d7*P*H+d8*L*L+d9*P*P+d10*H*H+d11*P*L*H+d12*L*L*L+
//                d13*L*P*P+d14*L*H*H+d15*L*L*P+d16*P*P*P+d17*P*H*H+d18*L*L*H+d19*P*P*H+d20*H*H*H

/////////////////////////////////////////////////////////////////////////////////
orsRPC_O3::orsRPC_O3()
{
//	m_l[10] = 1;
}



void orsRPC_O3::Initialize(const orsPOINT3D *ptsSrc, int n, const orsPOINT2D *ptsDst, float *weights)
{
	// 	assert( n >= 8 );

	m_pcSrc = Average3D(ptsSrc, n);
	m_pcDst = Average(ptsDst, n);

	m_xScale = m_yScale = 0;
	m_XScale = m_YScale = m_ZScale = 0;

	int k;

	for (k = 0; k < n; k++)
	{
		if (m_xScale < fabs(ptsDst[k].x - m_pcDst.x))
			m_xScale = fabs(ptsDst[k].x - m_pcDst.x);

		if (m_yScale < fabs(ptsDst[k].y - m_pcDst.y))
			m_yScale = fabs(ptsDst[k].y - m_pcDst.y);

		if (m_XScale < fabs(ptsSrc[k].X - m_pcSrc.X))
			m_XScale = fabs(ptsSrc[k].X - m_pcSrc.X);
		if (m_YScale < fabs(ptsSrc[k].Y - m_pcSrc.Y))
			m_YScale = fabs(ptsSrc[k].Y - m_pcSrc.Y);
		if (m_ZScale < fabs(ptsSrc[k].Z - m_pcSrc.Z))
			m_ZScale = fabs(ptsSrc[k].Z - m_pcSrc.Z);
	}

	////////////////////////////////////
	int i, j;
	double a[39], b[39], dxSrc, dySrc, dzSrc, dxDst, dyDst, lx, ly;

	orsMatrixD	AA(39, 39), BB(39, 39);
	orsVectorD	AL(39), BL(39);

	AA.Zero();	AL.Zero();
	BB.Zero();	BL.Zero();

	memset(a, 0, 39 * sizeof(double));
	memset(b, 0, 39 * sizeof(double));

	for (k = 0; k<n; k++)
	{
		dxSrc = (ptsSrc[k].X - m_pcSrc.X) / m_XScale;
		dySrc = (ptsSrc[k].Y - m_pcSrc.Y) / m_YScale;
		dzSrc = (ptsSrc[k].Z - m_pcSrc.Z) / m_ZScale;

		dxDst = (ptsDst[k].x - m_pcDst.x) / m_xScale;
		dyDst = (ptsDst[k].y - m_pcDst.y) / m_yScale;

		a[0] = b[0] = 1;	 a[1] = b[1] = dxSrc;  a[2] = b[2] = dySrc;  a[3] = b[3] = dzSrc;
		a[4] = b[4] = dxSrc*dySrc;  a[5] = b[5] = dxSrc*dzSrc;  a[6] = b[6] = dySrc*dzSrc;
		a[7] = b[7] = dxSrc*dxSrc;  a[8] = b[8] = dySrc*dySrc;  a[9] = b[9] = dzSrc*dzSrc;
		a[10] = b[10] = dxSrc*dxSrc*dxSrc;   a[11] = b[11] = dySrc*dySrc*dySrc;
		a[12] = b[12] = dzSrc*dzSrc*dzSrc;   a[13] = b[13] = dxSrc*dySrc*dzSrc;
		a[14] = b[14] = dxSrc*dxSrc*dySrc;   a[15] = b[15] = dxSrc*dxSrc*dzSrc;
		a[16] = b[16] = dxSrc*dySrc*dySrc;   a[17] = b[17] = dySrc*dySrc*dzSrc;
		a[18] = b[18] = dxSrc*dzSrc*dzSrc;   a[19] = b[19] = dySrc*dzSrc*dzSrc;

		a[20] = -dxDst*dxSrc;  a[21] = -dxDst*dySrc;  a[22] = -dxDst*dzSrc;
		a[23] = -dxDst*dxSrc*dySrc;  a[24] = -dxDst*dxSrc*dzSrc;  a[25] = -dxDst*dySrc*dzSrc;
		a[26] = -dxDst*dxSrc*dxSrc;  a[27] = -dxDst*dySrc*dySrc;  a[28] = -dxDst*dzSrc*dzSrc;
		a[29] = -dxDst*dxSrc*dxSrc*dxSrc;   a[30] = -dxDst*dySrc*dySrc*dySrc;
		a[31] = -dxDst*dzSrc*dzSrc*dzSrc;   a[32] = -dxDst*dxSrc*dySrc*dzSrc;
		a[33] = -dxDst*dxSrc*dxSrc*dySrc;   a[34] = -dxDst*dxSrc*dxSrc*dzSrc;
		a[35] = -dxDst*dxSrc*dySrc*dySrc;   a[36] = -dxDst*dySrc*dySrc*dzSrc;
		a[37] = -dxDst*dxSrc*dzSrc*dzSrc;   a[38] = -dxDst*dySrc*dzSrc*dzSrc;
		lx = dxDst;

		b[20] = -dyDst*dxSrc; b[21] = -dyDst*dySrc; b[22] = -dyDst*dzSrc;
		b[23] = -dyDst*dxSrc*dySrc; b[24] = -dyDst*dxSrc*dzSrc; b[25] = -dyDst*dySrc*dzSrc;
		b[26] = -dyDst*dxSrc*dxSrc; b[27] = -dyDst*dySrc*dySrc; b[28] = -dyDst*dzSrc*dzSrc;
		b[29] = -dyDst*dxSrc*dxSrc*dxSrc;   b[30] = -dyDst*dySrc*dySrc*dySrc;
		b[31] = -dyDst*dzSrc*dzSrc*dzSrc;   b[32] = -dyDst*dxSrc*dySrc*dzSrc;
		b[33] = -dyDst*dxSrc*dxSrc*dySrc;   b[34] = -dyDst*dxSrc*dxSrc*dzSrc;
		b[35] = -dyDst*dxSrc*dySrc*dySrc;   b[36] = -dyDst*dySrc*dySrc*dzSrc;
		b[37] = -dyDst*dxSrc*dzSrc*dzSrc;   b[38] = -dyDst*dySrc*dzSrc*dzSrc;
		ly = dyDst;

		for (i = 0; i<39; i++)
		{
			for (j = 0; j<39; j++)	{
				AA[i][j] += a[i] * a[j];
				BB[i][j] += b[i] * b[j];
			}

			AL[i] += a[i] * lx;
			BL[i] += b[i] * ly;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	orsVectorD	X(39), Y(39);
	getMatrixService()->SolveLinearEqs_K(AA, AL, X, 0.0001);
	getMatrixService()->SolveLinearEqs_K(BB, BL, Y, 0.0001);

	X.CopyData(m_a);
	Y.CopyData(m_b);

	//////////////////////////////////////////////////////////////////////////

	m_mx = m_my = 0;
	double vx, vy,mx,my, wx, wy;
	for (k = 0; k<n; k++)
	{
		dxSrc = (ptsSrc[k].X - m_pcSrc.X) / m_XScale;
		dySrc = (ptsSrc[k].Y - m_pcSrc.Y) / m_YScale;
		dzSrc = (ptsSrc[k].Z - m_pcSrc.Z) / m_ZScale;

		mx = m_a[0] + m_a[1] * dxSrc + m_a[2] * dySrc + m_a[3] * dzSrc + m_a[4] * dxSrc*dySrc
			+ m_a[5] * dxSrc*dzSrc + m_a[6] * dySrc*dzSrc + m_a[7] * dxSrc*dxSrc + m_a[8] * dySrc*dySrc
			+ m_a[9] * dzSrc*dzSrc + m_a[10] * dxSrc*dxSrc*dxSrc + m_a[11] * dySrc*dySrc*dySrc
			+ m_a[12] * dzSrc*dzSrc*dzSrc + m_a[13] * dxSrc*dySrc*dzSrc + m_a[14] * dxSrc*dxSrc*dySrc
			+ m_a[15] * dxSrc*dxSrc*dzSrc + m_a[16] * dxSrc*dySrc*dySrc + m_a[17] * dySrc*dySrc*dzSrc
			+ m_a[18] * dxSrc*dzSrc*dzSrc + m_a[19] * dySrc*dzSrc*dzSrc;
		wx = 1 + m_a[20] * dxSrc + m_a[21] * dySrc + m_a[22] * dzSrc + m_a[23] * dxSrc*dySrc
			+ m_a[24] * dxSrc*dzSrc + m_a[25] * dySrc*dzSrc + m_a[26] * dxSrc*dxSrc + m_a[27] * dySrc*dySrc
			+ m_a[28] * dzSrc*dzSrc + m_a[29] * dxSrc*dxSrc*dxSrc + m_a[30] * dySrc*dySrc*dySrc
			+ m_a[31] * dzSrc*dzSrc*dzSrc + m_a[32] * dxSrc*dySrc*dzSrc + m_a[33] * dxSrc*dxSrc*dySrc
			+ m_a[34] * dxSrc*dxSrc*dzSrc + m_a[35] * dxSrc*dySrc*dySrc + m_a[36] * dySrc*dySrc*dzSrc
			+ m_a[37] * dxSrc*dzSrc*dzSrc + m_a[38] * dySrc*dzSrc*dzSrc;

		my = m_b[0] + m_b[1] * dxSrc + m_b[2] * dySrc + m_b[3] * dzSrc + m_b[4] * dxSrc*dySrc
			+ m_b[5] * dxSrc*dzSrc + m_b[6] * dySrc*dzSrc + m_b[7] * dxSrc*dxSrc + m_b[8] * dySrc*dySrc
			+ m_b[9] * dzSrc*dzSrc + m_b[10] * dxSrc*dxSrc*dxSrc + m_b[11] * dySrc*dySrc*dySrc
			+ m_b[12] * dzSrc*dzSrc*dzSrc + m_b[13] * dxSrc*dySrc*dzSrc + m_b[14] * dxSrc*dxSrc*dySrc
			+ m_b[15] * dxSrc*dxSrc*dzSrc + m_b[16] * dxSrc*dySrc*dySrc + m_b[17] * dySrc*dySrc*dzSrc
			+ m_b[18] * dxSrc*dzSrc*dzSrc + m_b[19] * dySrc*dzSrc*dzSrc;
		wy = 1 + m_b[20] * dxSrc + m_b[21] * dySrc + m_b[22] * dzSrc + m_b[23] * dxSrc*dySrc
			+ m_b[24] * dxSrc*dzSrc + m_b[25] * dySrc*dzSrc + m_b[26] * dxSrc*dxSrc + m_b[27] * dySrc*dySrc
			+ m_b[28] * dzSrc*dzSrc + m_b[29] * dxSrc*dxSrc*dxSrc + m_b[30] * dySrc*dySrc*dySrc
			+ m_b[31] * dzSrc*dzSrc*dzSrc + m_b[32] * dxSrc*dySrc*dzSrc + m_b[33] * dxSrc*dxSrc*dySrc
			+ m_b[34] * dxSrc*dxSrc*dzSrc + m_b[35] * dxSrc*dySrc*dySrc + m_b[36] * dySrc*dySrc*dzSrc
			+ m_b[37] * dxSrc*dzSrc*dzSrc + m_b[38] * dySrc*dzSrc*dzSrc;
		
		vx = ptsDst[k].x - m_pcDst.x - m_xScale*( mx / wx);
		vy = ptsDst[k].y - m_pcDst.y - m_yScale*(my / wy);

		m_mx += vx*vx;
		m_my += vy*vy;
	}

	m_mx = sqrt(m_mx / (n - 39));
	m_my = sqrt(m_my / (n - 39));
}



void orsRPC_O3::GetResidual(const orsPOINT3D *ptsSrc, int n, const orsPOINT2D *ptsDst, orsPOINT2D *pVxys)
{
	int k;
	double mx,my,wx, wy;
	double	dxSrc, dySrc, dzSrc, dxDst, dyDst;

	for (k = 0; k<n; k++)
	{
		dxSrc = (ptsSrc->X - m_pcSrc.X) / m_XScale;
		dySrc = (ptsSrc->Y - m_pcSrc.Y) / m_YScale;
		dzSrc = (ptsSrc->Z - m_pcSrc.Z) / m_ZScale;

		mx = m_a[0] + m_a[1] * dxSrc + m_a[2] * dySrc + m_a[3] * dzSrc + m_a[4] * dxSrc*dySrc
			+ m_a[5] * dxSrc*dzSrc + m_a[6] * dySrc*dzSrc + m_a[7] * dxSrc*dxSrc + m_a[8] * dySrc*dySrc
			+ m_a[9] * dzSrc*dzSrc + m_a[10] * dxSrc*dxSrc*dxSrc + m_a[11] * dySrc*dySrc*dySrc
			+ m_a[12] * dzSrc*dzSrc*dzSrc + m_a[13] * dxSrc*dySrc*dzSrc + m_a[14] * dxSrc*dxSrc*dySrc
			+ m_a[15] * dxSrc*dxSrc*dzSrc + m_a[16] * dxSrc*dySrc*dySrc + m_a[17] * dySrc*dySrc*dzSrc
			+ m_a[18] * dxSrc*dzSrc*dzSrc + m_a[19] * dySrc*dzSrc*dzSrc;
		wx = 1 + m_a[20] * dxSrc + m_a[21] * dySrc + m_a[22] * dzSrc + m_a[23] * dxSrc*dySrc
			+ m_a[24] * dxSrc*dzSrc + m_a[25] * dySrc*dzSrc + m_a[26] * dxSrc*dxSrc + m_a[27] * dySrc*dySrc
			+ m_a[28] * dzSrc*dzSrc + m_a[29] * dxSrc*dxSrc*dxSrc + m_a[30] * dySrc*dySrc*dySrc
			+ m_a[31] * dzSrc*dzSrc*dzSrc + m_a[32] * dxSrc*dySrc*dzSrc + m_a[33] * dxSrc*dxSrc*dySrc
			+ m_a[34] * dxSrc*dxSrc*dzSrc + m_a[35] * dxSrc*dySrc*dySrc + m_a[36] * dySrc*dySrc*dzSrc
			+ m_a[37] * dxSrc*dzSrc*dzSrc + m_a[38] * dySrc*dzSrc*dzSrc;

		my = m_b[0] + m_b[1] * dxSrc + m_b[2] * dySrc + m_b[3] * dzSrc + m_b[4] * dxSrc*dySrc
			+ m_b[5] * dxSrc*dzSrc + m_b[6] * dySrc*dzSrc + m_b[7] * dxSrc*dxSrc + m_b[8] * dySrc*dySrc
			+ m_b[9] * dzSrc*dzSrc + m_b[10] * dxSrc*dxSrc*dxSrc + m_b[11] * dySrc*dySrc*dySrc
			+ m_b[12] * dzSrc*dzSrc*dzSrc + m_b[13] * dxSrc*dySrc*dzSrc + m_b[14] * dxSrc*dxSrc*dySrc
			+ m_b[15] * dxSrc*dxSrc*dzSrc + m_b[16] * dxSrc*dySrc*dySrc + m_b[17] * dySrc*dySrc*dzSrc
			+ m_b[18] * dxSrc*dzSrc*dzSrc + m_b[19] * dySrc*dzSrc*dzSrc;
		wy = 1 + m_b[20] * dxSrc + m_b[21] * dySrc + m_b[22] * dzSrc + m_b[23] * dxSrc*dySrc
			+ m_b[24] * dxSrc*dzSrc + m_b[25] * dySrc*dzSrc + m_b[26] * dxSrc*dxSrc + m_b[27] * dySrc*dySrc
			+ m_b[28] * dzSrc*dzSrc + m_b[29] * dxSrc*dxSrc*dxSrc + m_b[30] * dySrc*dySrc*dySrc
			+ m_b[31] * dzSrc*dzSrc*dzSrc + m_b[32] * dxSrc*dySrc*dzSrc + m_b[33] * dxSrc*dxSrc*dySrc
			+ m_b[34] * dxSrc*dxSrc*dzSrc + m_b[35] * dxSrc*dySrc*dySrc + m_b[36] * dySrc*dySrc*dzSrc
			+ m_b[37] * dxSrc*dzSrc*dzSrc + m_b[38] * dySrc*dzSrc*dzSrc;

		pVxys->x = ptsDst->x - m_pcDst.x - m_xScale* mx / wx;
		pVxys->y = ptsDst->y - m_pcDst.y - m_yScale* my / wy;

		ptsSrc++;	ptsDst++;	pVxys++;
	}
}



void orsRPC_O3::Transform(const orsPOINT3D *ptsSrc, int n, orsPOINT2D *ptsDst)
{
	int k;

	double mx,my, wx, wy;
	double	dxSrc, dySrc, dzSrc, dxDst, dyDst;

	for (k = 0; k<n; k++)
	{
		dxSrc = (ptsSrc->X - m_pcSrc.X) / m_XScale;
		dySrc = (ptsSrc->Y - m_pcSrc.Y) / m_YScale;
		dzSrc = (ptsSrc->Z - m_pcSrc.Z) / m_ZScale;

		mx = m_a[0] + m_a[1] * dxSrc + m_a[2] * dySrc + m_a[3] * dzSrc + m_a[4] * dxSrc*dySrc
			+ m_a[5] * dxSrc*dzSrc + m_a[6] * dySrc*dzSrc + m_a[7] * dxSrc*dxSrc + m_a[8] * dySrc*dySrc
			+ m_a[9] * dzSrc*dzSrc + m_a[10] * dxSrc*dxSrc*dxSrc + m_a[11] * dySrc*dySrc*dySrc
			+ m_a[12] * dzSrc*dzSrc*dzSrc + m_a[13] * dxSrc*dySrc*dzSrc + m_a[14] * dxSrc*dxSrc*dySrc
			+ m_a[15] * dxSrc*dxSrc*dzSrc + m_a[16] * dxSrc*dySrc*dySrc + m_a[17] * dySrc*dySrc*dzSrc
			+ m_a[18] * dxSrc*dzSrc*dzSrc + m_a[19] * dySrc*dzSrc*dzSrc;
		wx = 1 + m_a[20] * dxSrc + m_a[21] * dySrc + m_a[22] * dzSrc + m_a[23] * dxSrc*dySrc
			+ m_a[24] * dxSrc*dzSrc + m_a[25] * dySrc*dzSrc + m_a[26] * dxSrc*dxSrc + m_a[27] * dySrc*dySrc
			+ m_a[28] * dzSrc*dzSrc + m_a[29] * dxSrc*dxSrc*dxSrc + m_a[30] * dySrc*dySrc*dySrc
			+ m_a[31] * dzSrc*dzSrc*dzSrc + m_a[32] * dxSrc*dySrc*dzSrc + m_a[33] * dxSrc*dxSrc*dySrc
			+ m_a[34] * dxSrc*dxSrc*dzSrc + m_a[35] * dxSrc*dySrc*dySrc + m_a[36] * dySrc*dySrc*dzSrc
			+ m_a[37] * dxSrc*dzSrc*dzSrc + m_a[38] * dySrc*dzSrc*dzSrc;

		my = m_b[0] + m_b[1] * dxSrc + m_b[2] * dySrc + m_b[3] * dzSrc + m_b[4] * dxSrc*dySrc
			+ m_b[5] * dxSrc*dzSrc + m_b[6] * dySrc*dzSrc + m_b[7] * dxSrc*dxSrc + m_b[8] * dySrc*dySrc
			+ m_b[9] * dzSrc*dzSrc + m_b[10] * dxSrc*dxSrc*dxSrc + m_b[11] * dySrc*dySrc*dySrc
			+ m_b[12] * dzSrc*dzSrc*dzSrc + m_b[13] * dxSrc*dySrc*dzSrc + m_b[14] * dxSrc*dxSrc*dySrc
			+ m_b[15] * dxSrc*dxSrc*dzSrc + m_b[16] * dxSrc*dySrc*dySrc + m_b[17] * dySrc*dySrc*dzSrc
			+ m_b[18] * dxSrc*dzSrc*dzSrc + m_b[19] * dySrc*dzSrc*dzSrc;
		wy = 1 + m_b[20] * dxSrc + m_b[21] * dySrc + m_b[22] * dzSrc + m_b[23] * dxSrc*dySrc
			+ m_b[24] * dxSrc*dzSrc + m_b[25] * dySrc*dzSrc + m_b[26] * dxSrc*dxSrc + m_b[27] * dySrc*dySrc
			+ m_b[28] * dzSrc*dzSrc + m_b[29] * dxSrc*dxSrc*dxSrc + m_b[30] * dySrc*dySrc*dySrc
			+ m_b[31] * dzSrc*dzSrc*dzSrc + m_b[32] * dxSrc*dySrc*dzSrc + m_b[33] * dxSrc*dxSrc*dySrc
			+ m_b[34] * dxSrc*dxSrc*dzSrc + m_b[35] * dxSrc*dySrc*dySrc + m_b[36] * dySrc*dySrc*dzSrc
			+ m_b[37] * dxSrc*dzSrc*dzSrc + m_b[38] * dySrc*dzSrc*dzSrc;
 
		ptsDst->x = m_pcDst.x + m_xScale* mx / wx;
 		ptsDst->y = m_pcDst.y + m_yScale* my / wy;

		ptsSrc++;	ptsDst++;
	}
}



void orsRPC_O3::GetMeanError(double *mx, double *my)
{
	*mx = m_mx;
	*my = m_my;
}



void orsRPC_O3::GetParameter(orsPOINT3D *pcSrc, orsPOINT2D *pcDst, double *a, double *b, double *c)
{
	memcpy(a, m_a, 20 * sizeof(double));
	a[20] = 1;
	memcpy(a+21, m_a+20, 19 * sizeof(double));

	memcpy(b, m_b, 20 * sizeof(double));
	b[20] = 1;
	memcpy(b + 21, m_b + 20, 19 * sizeof(double));
}



// 取变换参数，中心点计入参数
void orsRPC_O3::GetParameter(double *a, double *b, double *c)
{
	assert(false);
}

//写出参数
#include <stdio.h>
void orsRPC_O3::Writerpc(const char *outputfile)
{
	orsString FileName = outputfile;

	int pos = FileName.reverseFind('_');
	orsString mainName = FileName.left(pos);
	orsString ImgName = mainName + ".TIF";
	ref_ptr<orsIImageSource> orthImg = getImageService()->openImageFile(ImgName);

	orsIImageGeometry *othGeo = orthImg->GetImageGeometry();

	orsISpatialReference *orgSRS = othGeo->GetSpatialReference();

	ref_ptr<orsIOGRString> hcsWkt;
	orgSRS->exportToWkt(hcsWkt);

	const char *Wkt = hcsWkt->getStr();
	
	FILE *fp = fopen(FileName, "wt");

	if (NULL == fp)
	{
		getPlatform()->logPrint(ORS_LOG_ERROR, "Can not create %s", FileName.c_str() );
		return;
	}

	fprintf(fp, "LINE_OFF: %lf  pixels\n", m_pcDst.y);
	fprintf(fp, "SAMP_OFF: %lf  pixels\n", m_pcDst.x);
	fprintf(fp, "LAT_OFF: %lf  degrees\n", m_pcSrc.Y);
	fprintf(fp, "LONG_OFF: %lf  degrees\n", m_pcSrc.X);
	fprintf(fp, "HEIGHT_OFF: %lf  meters\n", m_pcSrc.Z);
	fprintf(fp, "LINE_SCALE: %lf  pixels\n", m_yScale);
	fprintf(fp, "SAMP_SCALE: %lf  pixels\n", m_xScale);
	fprintf(fp, "LAT_SCALE: %lf  degrees\n", m_YScale);
	fprintf(fp, "LONG_SCALE: %lf  degrees\n", m_XScale);
	fprintf(fp, "HEIGHT_SCALE: %lf  meters\n", m_ZScale);
	int i;
	for (i = 0; i < 20;i++)
	{
		fprintf(fp, "LINE_NUM_COEFF_%d: %.16le\n", i+1, m_b[i]);
	}

	fprintf(fp, "LINE_DEN_COEFF_%d: %.16le\n", 1,1.0);
	for (i = 0; i < 19; i++)
	{
		fprintf(fp, "LINE_DEN_COEFF_%d: %.16le\n", i + 2, m_b[20+i]);
	}
	for (i = 0; i < 20; i++)
	{
		fprintf(fp, "SAMP_NUM_COEFF_%d: %.16le\n", i + 1, m_a[i]);
	}

	fprintf(fp, "SAMP_DEN_COEFF_%d: %.16le\n", 1, 1.0);
	for (i = 0; i < 19; i++)
	{
		fprintf(fp, "SAMP_DEN_COEFF_%d: %.16le\n", i + 2, m_a[20+i]);
	}

	fprintf(fp, "wktHCS: %s",Wkt);
	fclose(fp);
}
