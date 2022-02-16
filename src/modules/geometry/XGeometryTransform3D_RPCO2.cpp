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

orsPOINT2D Average(const orsPOINT2D *pts, int n);

orsPOINT3D Average3D(const orsPOINT3D *pts, int n);

///////////////////////////////////////////////////////////////////////////////
//		       a1+a2*( xSrc-xcSrc)+a3*(ySrc-ycSrc)+a4*(zSrc-zcSrc)+a5*( xSrc-xcSrc)*(ySrc-ycSrc)+
//             a6*( xSrc-xcSrc)*(zSrc-zcSrc)+a7*(ySrc-ycSrc)*(zSrc-zcSrc)+a8*( xSrc-xcSrc)*( xSrc-xcSrc)+
//             a9*(ySrc-ycSrc)*(ySrc-ycSrc)+a10*(zSrc-zcSrc)*(zSrc-zcSrc)
// xDst - xcDst = ------------------------------------------------------------------------------------
//    		   b1+b2*( xSrc-xcSrc)+b3*(ySrc-ycSrc)+b4*(zSrc-zcSrc)+b5*( xSrc-xcSrc)*(ySrc-ycSrc)+
//             b6*( xSrc-xcSrc)*(zSrc-zcSrc)+b7*(ySrc-ycSrc)*(zSrc-zcSrc)+b8*( xSrc-xcSrc)*( xSrc-xcSrc)+
//             b9*(ySrc-ycSrc)*(ySrc-ycSrc)+b10*(zSrc-zcSrc)*(zSrc-zcSrc)
//
//		          c1+c2*L+c3*P+c4*H+c5*L*P+c6*L*H+c7*P*H+c8*L*L+c9*P*P+c10*H*H
// yDst - ycDst = ------------------------------------------------------------------------------------
//		          d1+d2*L+d3*P+d4*H+d5*L*P+d6*L*H+d7*P*H+d8*L*L+d9*P*P+d10*H*H

/////////////////////////////////////////////////////////////////////////////////
orsRPC_O2::orsRPC_O2()
{
//	m_l[10] = 1;
}



void orsRPC_O2::Initialize(const orsPOINT3D *ptsSrc, int n, const orsPOINT2D *ptsDst, float *weights)
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
	double a[19], b[19], dxSrc, dySrc, dzSrc, dxDst, dyDst, lx, ly;

	orsMatrixD	AA(19, 19), BB(19, 19);
	orsVectorD	AL(19), BL(19);

	AA.Zero();	AL.Zero();
	BB.Zero();	BL.Zero();

	memset(a, 0, 19 * sizeof(double));
	memset(b, 0, 19 * sizeof(double));

	for (k = 0; k<n; k++)
	{
		// 3D
		dxSrc = (ptsSrc[k].X - m_pcSrc.X) / m_XScale;
		dySrc = (ptsSrc[k].Y - m_pcSrc.Y) / m_YScale;
		dzSrc = (ptsSrc[k].Z - m_pcSrc.Z) / m_ZScale;

		// 2D
		dxDst = (ptsDst[k].x - m_pcDst.x) / m_xScale;
		dyDst = (ptsDst[k].y - m_pcDst.y) / m_yScale;

		// 分子
		a[0] = 1;
		a[1] = dxSrc;		 a[2] = dySrc;		  a[3] = dzSrc;
		a[4] = dxSrc*dySrc;  a[5] = dxSrc*dzSrc;  a[6] = dySrc*dzSrc;
		a[7] = dxSrc*dxSrc;  a[8] = dySrc*dySrc;  a[9] = dzSrc*dzSrc;

		memcpy(b, a, 10 * sizeof(double));

		// 分母
		int i;
		for (i = 1; i < 10; i++)	
		{
			a[9 + i] = -dxDst*a[i];
			b[9 + i] = -dyDst*b[i];
		}

		lx = dxDst;
		ly = dyDst;

		for (i = 0; i<19; i++)
		{
			for (j = 0; j<19; j++)	{
				AA[i][j] += a[i] * a[j];
				BB[i][j] += b[i] * b[j];
			}

			AL[i] += a[i] * lx;
			BL[i] += b[i] * ly;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	orsVectorD	X(19), Y(19);
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

		mx = m_a[0] + m_a[1] * dxSrc + m_a[2] * dySrc + m_a[3] * dzSrc 
			+ m_a[4] * dxSrc*dySrc + m_a[5] * dxSrc*dzSrc + m_a[6] * dySrc*dzSrc 
			+ m_a[7] * dxSrc*dxSrc + m_a[8] * dySrc*dySrc + m_a[9] * dzSrc*dzSrc;

		wx = 1 + m_a[10] * dxSrc + m_a[11] * dySrc + m_a[12] * dzSrc 
			+ m_a[13] * dxSrc*dySrc	+ m_a[14] * dxSrc*dzSrc + m_a[15] * dySrc*dzSrc 
			+ m_a[16] * dxSrc*dxSrc + m_a[17] * dySrc*dySrc	+ m_a[18] * dzSrc*dzSrc;

		my = m_b[0] + m_b[1] * dxSrc + m_b[2] * dySrc + m_b[3] * dzSrc 
			+ m_b[4] * dxSrc*dySrc + m_b[5] * dxSrc*dzSrc + m_b[6] * dySrc*dzSrc 
			+ m_b[7] * dxSrc*dxSrc + m_b[8] * dySrc*dySrc + m_b[9] * dzSrc*dzSrc;

		wy = 1 + m_b[10] * dxSrc + m_b[11] * dySrc + m_b[12] * dzSrc 
			+ m_b[13] * dxSrc*dySrc	+ m_b[14] * dxSrc*dzSrc + m_b[15] * dySrc*dzSrc 
			+ m_b[16] * dxSrc*dxSrc + m_b[17] * dySrc*dySrc	+ m_b[18] * dzSrc*dzSrc;
		
		vx = ptsDst[k].x - m_pcDst.x - m_xScale*( mx / wx);
		vy = ptsDst[k].y - m_pcDst.y - m_yScale*(my / wy);

		m_mx += vx*vx;
		m_my += vy*vy;
	}

	m_mx = sqrt(m_mx / (n - 19));
	m_my = sqrt(m_my / (n - 19));
}



void orsRPC_O2::GetResidual(const orsPOINT3D *ptsSrc, int n, const orsPOINT2D *ptsDst, orsPOINT2D *pVxys)
{
	int k;
	double mx,my,wx, wy;
	double	dxSrc, dySrc, dzSrc, dxDst, dyDst;

	for (k = 0; k<n; k++)
	{
		dxSrc = (ptsSrc->X - m_pcSrc.X) / m_XScale;
		dySrc = (ptsSrc->Y - m_pcSrc.Y) / m_YScale;
		dzSrc = (ptsSrc->Z - m_pcSrc.Z) / m_ZScale;

		mx = m_a[0] + m_a[1] * dxSrc + m_a[2] * dySrc + m_a[3] * dzSrc 
			+ m_a[4] * dxSrc*dySrc + m_a[5] * dxSrc*dzSrc + m_a[6] * dySrc*dzSrc 
			+ m_a[7] * dxSrc*dxSrc + m_a[8] * dySrc*dySrc + m_a[9] * dzSrc*dzSrc;

		wx = 1 + m_a[10] * dxSrc + m_a[11] * dySrc + m_a[12] * dzSrc 
			+ m_a[13] * dxSrc*dySrc	+ m_a[14] * dxSrc*dzSrc + m_a[15] * dySrc*dzSrc 
			+ m_a[16] * dxSrc*dxSrc + m_a[17] * dySrc*dySrc	+ m_a[18] * dzSrc*dzSrc;

		my = m_b[0] + m_b[1] * dxSrc + m_b[2] * dySrc + m_b[3] * dzSrc 
			+ m_b[4] * dxSrc*dySrc + m_b[5] * dxSrc*dzSrc + m_b[6] * dySrc*dzSrc 
			+ m_b[7] * dxSrc*dxSrc + m_b[8] * dySrc*dySrc + m_b[9] * dzSrc*dzSrc;

		wy = 1 + m_b[10] * dxSrc + m_b[11] * dySrc + m_b[12] * dzSrc 
			+ m_b[13] * dxSrc*dySrc	+ m_b[14] * dxSrc*dzSrc + m_b[15] * dySrc*dzSrc 
			+ m_b[16] * dxSrc*dxSrc + m_b[17] * dySrc*dySrc	+ m_b[18] * dzSrc*dzSrc;

		pVxys->x = ptsDst->x - m_pcDst.x - m_xScale* mx / wx;
		pVxys->y = ptsDst->y - m_pcDst.y - m_yScale* my / wy;

		ptsSrc++;	ptsDst++;	pVxys++;
	}
}


void orsRPC_O2::Transform(const orsPOINT3D *ptsSrc, int n, orsPOINT2D *ptsDst)
{
	int k;

	double mx,my, wx, wy;
	double	dxSrc, dySrc, dzSrc, dxDst, dyDst;

	for (k = 0; k<n; k++)
	{
		dxSrc = (ptsSrc->X - m_pcSrc.X) / m_XScale;
		dySrc = (ptsSrc->Y - m_pcSrc.Y) / m_YScale;
		dzSrc = (ptsSrc->Z - m_pcSrc.Z) / m_ZScale;

		mx = m_a[0] + m_a[1] * dxSrc + m_a[2] * dySrc + m_a[3] * dzSrc 
			+ m_a[4] * dxSrc*dySrc + m_a[5] * dxSrc*dzSrc + m_a[6] * dySrc*dzSrc 
			+ m_a[7] * dxSrc*dxSrc + m_a[8] * dySrc*dySrc + m_a[9] * dzSrc*dzSrc;

		wx = 1 + m_a[10] * dxSrc + m_a[11] * dySrc + m_a[12] * dzSrc 
			+ m_a[13] * dxSrc*dySrc	+ m_a[14] * dxSrc*dzSrc + m_a[15] * dySrc*dzSrc 
			+ m_a[16] * dxSrc*dxSrc + m_a[17] * dySrc*dySrc	+ m_a[18] * dzSrc*dzSrc;

		my = m_b[0] + m_b[1] * dxSrc + m_b[2] * dySrc + m_b[3] * dzSrc 
			+ m_b[4] * dxSrc*dySrc + m_b[5] * dxSrc*dzSrc + m_b[6] * dySrc*dzSrc 
			+ m_b[7] * dxSrc*dxSrc + m_b[8] * dySrc*dySrc + m_b[9] * dzSrc*dzSrc;

		wy = 1 + m_b[10] * dxSrc + m_b[11] * dySrc + m_b[12] * dzSrc 
			+ m_b[13] * dxSrc*dySrc	+ m_b[14] * dxSrc*dzSrc + m_b[15] * dySrc*dzSrc 
			+ m_b[16] * dxSrc*dxSrc + m_b[17] * dySrc*dySrc	+ m_b[18] * dzSrc*dzSrc;
 
		ptsDst->x = m_pcDst.x + m_xScale* mx / wx;
 		ptsDst->y = m_pcDst.y + m_yScale* my / wy;

		ptsSrc++;	ptsDst++;
	}
}


void orsRPC_O2::GetMeanError(double *mx, double *my)
{
	*mx = m_mx;
	*my = m_my;
}


void orsRPC_O2::GetParameter(orsPOINT3D *pcSrc, orsPOINT2D *pcDst, double *a, double *b, double *c)
{
	memcpy(a, m_a, 10 * sizeof(double));
	a[10] = 1;
	memcpy(a + 11, m_a + 10, 9 * sizeof(double));

	memcpy(b, m_b, 10 * sizeof(double));
	b[10] = 1;
	memcpy(b + 11, m_b + 10, 9 * sizeof(double));
}


// 取变换参数，中心点计入参数
void orsRPC_O2::GetParameter(double *a, double *b, double *c)
{
	assert(false);
}

//写出参数
#include <stdio.h>
void orsRPC_O2::Writerpc(const char *outputfile)
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
	for (i = 0; i < 10;i++)
	{
		fprintf(fp, "LINE_NUM_COEFF_%d: %.16le\n", i+1, m_b[i]);
	}
	for (i = 10; i < 20; i++)
	{
		fprintf(fp, "LINE_NUM_COEFF_%d: %.16le\n", i + 1, 0.0);
	}

	fprintf(fp, "LINE_DEN_COEFF_%d: %.16le\n", 0+1, 1.0);
	for (i = 0; i < 9; i++)
	{
		fprintf(fp, "LINE_DEN_COEFF_%d: %.16le\n", i + 2, m_b[10+i]);
	}
	for (i = 10; i < 20; i++)
	{
		fprintf(fp, "LINE_DEN_COEFF_%d: %.16le\n", i + 1, 0.0);
	}

	//////////////////////////////////////////////////////////////////////////
	for (i = 0; i < 10; i++)
	{
		fprintf(fp, "SAMP_NUM_COEFF_%d: %.16le\n", i + 1, m_a[i]);
	}
	for (i = 10; i < 20; i++)
	{
		fprintf(fp, "SAMP_NUM_COEFF_%d: %.16le\n", i + 1, 0.0);
	}

	fprintf(fp, "SAMP_DEN_COEFF_%d: %.16le\n", 0 + 1, 1.0);
	for (i = 0; i < 9; i++)
	{
		fprintf(fp, "SAMP_DEN_COEFF_%d: %.16le\n", i + 2, m_a[10+i]);
	}
	for (i = 10; i < 20; i++)
	{
		fprintf(fp, "SAMP_DEN_COEFF_%d: %.16le\n", i + 1, 0.0);
	}

	fprintf(fp, "wktHCS: %s",Wkt);
	fclose(fp);
}
