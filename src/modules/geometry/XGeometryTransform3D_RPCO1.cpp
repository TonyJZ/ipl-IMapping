#include <math.h>


//#include "orsMath/orsIMatrixService.h"
#include "XGeometryTransform3D.h"

iplPOINT2D Average( const iplPOINT2D *pts, int n);

iplPOINT3D Average3D( const iplPOINT3D *pts, int n);


using namespace ipl;

//////////////////////////////////////////////////////////////////////////
//				   L0*( xSrc-xcSrc) + L1*(ySrc-ycSrc) + L2*(zSrc-zcSrc) + L3
// xDst - xcDst = -----------------------------------------------------------
//				   L4*( xSrc-xcSrc) + L5*(ySrc-ycSrc) + L6*(zSrc-zcSrc) + 1
//
//				   L7*( xSrc-xcSrc) + L8*(ySrc-ycSrc) + L9*(zSrc-zcSrc) + L10
// yDst - ycDst = ------------------------------------------------------------
//				   L11*( xSrc-xcSrc) + L12*(ySrc-ycSrc) + L13*(zSrc-zcSrc) + 1
//
//				   
// dxDst*(l6*dxSrc + l7*dySrc + 1) = l0*dxSrc + l1*dySrc + l2
//
// dyDst*(l6*dxSrc + l7*dySrc + 1) = l3*dxSrc + l4*dySrc + l5
//
//////////////////////////////////////////////////////////////////////////
XRPC_O1::XRPC_O1()
{
	m_l[10] = 1;
}



void XRPC_O1::Initialize( const iplPOINT3D *ptsSrc, int n,  const iplPOINT2D *ptsDst, float *weights )
{
// 	assert( n >= 8 );

	m_pcSrc = Average3D( ptsSrc, n );
	m_pcDst = Average( ptsDst, n );

	m_xScale = m_yScale = 0;
	m_XScale = m_YScale = m_ZScale = 0;

	int k;

	for( k = 0; k < n; k++  )
	{
		if( m_xScale < fabs( ptsDst[k].x - m_pcDst.x ) )
			m_xScale = fabs( ptsDst[k].x - m_pcDst.x );

		if( m_yScale < fabs( ptsDst[k].y - m_pcDst.y ) )
			m_yScale = fabs( ptsDst[k].y - m_pcDst.y );

		if( m_XScale < fabs( ptsSrc[k].X - m_pcSrc.X ) )
			m_XScale = fabs( ptsSrc[k].X - m_pcSrc.X );
		if( m_YScale < fabs( ptsSrc[k].Y - m_pcSrc.Y ) )
			m_YScale = fabs( ptsSrc[k].Y - m_pcSrc.Y );
		if( m_ZScale < fabs( ptsSrc[k].Z - m_pcSrc.Z ) )
			m_ZScale = fabs( ptsSrc[k].Z - m_pcSrc.Z );
	}

	////////////////////////////////////
	int i,j;
	double a[7], b[7], dxSrc, dySrc, dzSrc, dxDst, dyDst, lx, ly;

	orsMatrixD	AA(7,7), BB(7,7);
	orsVectorD	AL(7), BL(7);

	AA.Zero();	AL.Zero();
	BB.Zero();	BL.Zero();

	memset(a, 0, 7*sizeof(double) );
	memset(b, 0, 7*sizeof(double) );

	for( k=0; k<n ; k++)
	{ 
		dxSrc = (ptsSrc[k].X - m_pcSrc.X)/m_XScale;	
		dySrc = (ptsSrc[k].Y - m_pcSrc.Y)/m_YScale;
		dzSrc = (ptsSrc[k].Z - m_pcSrc.Z)/m_ZScale;

		dxDst = (ptsDst[k].x - m_pcDst.x)/m_xScale;	
		dyDst = (ptsDst[k].y - m_pcDst.y)/m_yScale;	

		a[0] = dxSrc;	a[1] = dySrc;	a[2] = dzSrc;	a[3] = 1;
		a[4] = -dxDst*dxSrc;	a[5] = -dxDst*dySrc;	a[6] = -dxDst*dzSrc;
		lx = dxDst;

		b[0] = dxSrc;	b[1] = dySrc;	b[2] = dzSrc;	b[3] = 1;
		b[4] = -dyDst*dxSrc;	b[5] = -dyDst*dySrc;	b[6] = -dyDst*dzSrc;		
		ly = dyDst;

		for( i=0; i<7; i++)
		{
			for( j=0; j<7; j++)	{
				AA[i][j] += a[i]*a[j];
				BB[i][j] += b[i]*b[j];
			}

			AL[i] += a[i]* lx;
			BL[i] += b[i]* ly;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	orsVectorD	X(7), Y(7);
	getMatrixService()->SolveLinearEqs_K( AA, AL, X, 0.0001 );
	getMatrixService()->SolveLinearEqs_K( BB, BL, Y, 0.0001 );
	
	X.CopyData( m_l );
	Y.CopyData( m_l+7 );

	//////////////////////////////////////////////////////////////////////////

	m_mx = m_my = 0;
	double vx, vy, wx, wy;
	for( k=0; k<n ; k++)
	{
		dxSrc = (ptsSrc[k].X - m_pcSrc.X)/m_XScale;	
		dySrc = (ptsSrc[k].Y - m_pcSrc.Y)/m_YScale;
		dzSrc = (ptsSrc[k].Z - m_pcSrc.Z)/m_ZScale;
		
		wx = m_l[4]*dxSrc + m_l[5]*dySrc + m_l[6]*dzSrc +  1;
		wy = m_l[11]*dxSrc + m_l[12]*dySrc + m_l[13]*dzSrc +  1;

		vx = ptsDst[k].x-m_pcDst.x - m_xScale*( m_l[0]*dxSrc + m_l[1]*dySrc + m_l[2]*dzSrc + m_l[3] ) / wx ;
		vy = ptsDst[k].y-m_pcDst.y - m_yScale*( m_l[7]*dxSrc + m_l[8]*dySrc + m_l[9]*dzSrc + m_l[10] ) / wy ;

		m_mx += vx*vx;
		m_my += vy*vy;
	}

	m_mx = sqrt( m_mx / (n-7) );
	m_my = sqrt( m_my / (n-7) );
}



void XRPC_O1::GetResidual( const iplPOINT3D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys )
{
	int k;
	double wx, wy;
	double	dxSrc, dySrc, dzSrc;

	for( k=0; k<n ; k++)
	{
		dxSrc = (ptsSrc->X - m_pcSrc.X)/m_XScale;	
		dySrc = (ptsSrc->Y - m_pcSrc.Y)/m_YScale;
		dzSrc = (ptsSrc->Z - m_pcSrc.Z)/m_ZScale;
		
		wx = m_l[4]*dxSrc + m_l[5]*dySrc + m_l[6]*dzSrc +  1;
		wy = m_l[11]*dxSrc + m_l[12]*dySrc + m_l[13]*dzSrc +  1;
		
		pVxys->x = ptsDst->x-m_pcDst.x - m_xScale*( m_l[0]*dxSrc + m_l[1]*dySrc + m_l[2]*dzSrc + m_l[3] ) / wx ;
		pVxys->y = ptsDst->y-m_pcDst.y - m_yScale*( m_l[7]*dxSrc + m_l[8]*dySrc + m_l[9]*dzSrc + m_l[10] ) / wy ;
		
		ptsSrc++;	ptsDst++;	pVxys++;
	}
}



void XRPC_O1::Transform( const iplPOINT3D *ptsSrc, int n, iplPOINT2D *ptsDst )
{
	int k;

	double wx, wy;
	double	dxSrc, dySrc, dzSrc, dxDst, dyDst;

	for( k=0; k<n ; k++)
	{
		dxSrc = (ptsSrc->X - m_pcSrc.X)/m_XScale;	
		dySrc = (ptsSrc->Y - m_pcSrc.Y)/m_YScale;
		dzSrc = (ptsSrc->Z - m_pcSrc.Z)/m_ZScale;
				
		wx = m_l[4]*dxSrc + m_l[5]*dySrc + m_l[6]*dzSrc +  1;
		wy = m_l[11]*dxSrc + m_l[12]*dySrc + m_l[13]*dzSrc +  1;
		
		ptsDst->x = m_pcDst.x + m_xScale*( m_l[0]*dxSrc + m_l[1]*dySrc + m_l[2]*dzSrc + m_l[3] ) / wx ;
		ptsDst->y = m_pcDst.y + m_yScale*( m_l[7]*dxSrc + m_l[8]*dySrc + m_l[9]*dzSrc + m_l[10] ) / wy ;
						
		ptsSrc++;	ptsDst++;
	}
}



void XRPC_O1::GetMeanError( double *mx,double *my )
{
	*mx = m_mx;
	*my = m_my;
}



void XRPC_O1::GetParameter( iplPOINT3D *pcSrc, iplPOINT2D *pcDst, double *a, double *b, double *c )
{
	*pcSrc = m_pcSrc;
	*pcDst = m_pcDst;

	double l[14];

	l[0] = m_l[0]*m_xScale/m_XScale;
	l[1] = m_l[1]*m_xScale/m_YScale;
	l[2] = m_l[2]*m_xScale/m_ZScale;
	l[3] = m_l[3]*m_xScale;

	l[4] = m_l[8]/m_XScale;
	l[5] = m_l[9]/m_YScale;
	l[6] = m_l[10]/m_ZScale;
	
	l[7] = m_l[7]*m_yScale/m_XScale;
	l[8] = m_l[8]*m_yScale/m_YScale;
	l[9] = m_l[9]*m_yScale/m_ZScale;
	l[10] = m_l[10]*m_yScale;
	
	l[11] = m_l[11]/m_XScale;
	l[12] = m_l[12]/m_YScale;
	l[13] = m_l[13]/m_ZScale;

	memcpy( a, l, 14*sizeof(double));
}



// 取变换参数，中心点计入参数
void XRPC_O1::GetParameter( double *a, double *b, double *c )
{
	int i;

	double l[14];

	l[0] = m_l[0]*m_xScale/m_XScale;
	l[1] = m_l[1]*m_xScale/m_YScale;
	l[2] = m_l[2]*m_xScale/m_ZScale;
	l[3] = m_l[3]*m_xScale;

	l[4] = m_l[4]/m_XScale;
	l[5] = m_l[5]/m_YScale;
	l[6] = m_l[6]/m_ZScale;
	
	l[7] = m_l[7]*m_yScale/m_XScale;
	l[8] = m_l[8]*m_yScale/m_YScale;
	l[9] = m_l[9]*m_yScale/m_ZScale;
	l[10] = m_l[10]*m_yScale;
	
	l[11] = m_l[11]/m_XScale;
	l[12] = m_l[12]/m_YScale;
	l[13] = m_l[13]/m_ZScale;

//////////////////////////////////////////////////////////////////////////
//				   l0*( xSrc-xcSrc) + l1*(ySrc-ycSrc) +  l2*(zSrc-zcSrc) + l3
// xDst - xcDst = ---------------------------------------------------------------------
//				   l4*( xSrc-xcSrc) + l5*(ySrc-ycSrc) + l6*(zSrc-zcSrc) + 1
// ==>
//		           l0*xSrc + l1*ySrc + l2*zSrc +  (l3 - l0*xcSrc - l1*ycSrc - l2*zcSrc)
// xDst = xcDst + ----------------------------------------------------------------------
//		           l4*xSrc + l5*ySrc + l6*zSrc + (1  - l4*xcSrc - l5*ycSrc - l6*zcSrc )
//
// ==>
//
//		   ( l0 + l4*xcDst) * *xSrc + ( l1 + l5*xcDst)* ySrc + ( l2 + l6*xcDst)* zSrc + (l3 - l0*xcSrc - l1*ycSrc - l2*zcSrc) + xcDst*l7x 
// xDst = ----------------------------------------------------------------------------------------------------------------------------------
//		    l4*xSrc + l5*ySrc + l6*zSrc + (1  - l4*xcSrc - l5*ycSrc - l6*zcSrc )-->l7x
	
	double xc, yc;
	
	xc = m_pcDst.x;
	yc = m_pcDst.y;
	
	double l7x = 1 - l[4]*m_pcSrc.X - l[5]*m_pcSrc.Y - l[6]*m_pcSrc.Z;
	
	//////////////////////////////////////////////////////////////////////////
	// 不能置后， 否则l[0],l[1]已被修改
	l[3] += l7x*xc - l[0]*m_pcSrc.X - l[1]*m_pcSrc.Y - l[2]*m_pcSrc.Z;
	l[0] += l[4]*xc;
	l[1] += l[5]*xc;
	l[2] += l[6]*xc;

	for( i=0; i<7; i++)
		l[i] /= l7x;

	//////////////////////////////////////////////////////////////////////////
	double l7y = 1 - l[11]*m_pcSrc.X - l[12]*m_pcSrc.Y - l[13]*m_pcSrc.Z;

	l[10] += l7y*yc - l[7]*m_pcSrc.X - l[8]*m_pcSrc.Y - l[9]*m_pcSrc.Z;
	l[7] += l[11]*yc;
	l[8] += l[12]*yc;
	l[9] += l[13]*yc;

	for( i=7; i<14; i++)
		l[i] /= l7y;

	memcpy( a, l, 14*sizeof(double));
}


//写出参数
#include <stdio.h>

#include "orsImage/orsIImageService.h"


void XRPC_O1::Writerpc(const char *outputfile)
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
		getPlatform()->logPrint(ORS_LOG_ERROR, "Can not create %s", FileName.c_str());
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
	//////////////////////////////// LINE /////////////////////////////////////////
	fprintf(fp, "LINE_NUM_COEFF_1: %.16le\n",  m_l[10]);
	for (i = 0; i < 3; i++)
	{
		fprintf(fp, "LINE_NUM_COEFF_%d: %.16le\n", i + 2, m_l[7+i]);	// l7,l8,l9
	}
	for (i = 4; i < 20; i++)
	{
		fprintf(fp, "LINE_NUM_COEFF_%d: %.16le\n", i + 1, 0);
	}

	///////////////////////////////// LINE /////////////////////////////////////////
	fprintf(fp, "LINE_DEN_COEFF_1: %.16le\n", 1.0);
	for (i = 0; i < 3; i++)
	{
		fprintf(fp, "LINE_DEN_COEFF_%d: %.16le\n", i + 2, m_l[11+i]);	// l11,l12,l13
	}

	for (i = 4; i < 20; i++)
	{
		fprintf(fp, "LINE_DEN_COEFF_%d: %.16le\n", i + 1, 0);
	}

	////////////////////////////////// SAMPLE ////////////////////////////////////////
	fprintf(fp, "SAMP_NUM_COEFF_1: %.16le\n", m_l[3]);				// l3
	for (i = 0; i < 3; i++)
	{
		fprintf(fp, "SAMP_NUM_COEFF_%d: %.16le\n", i + 2, m_l[i]);	// l0,l1,l2
	}
	for (i = 4; i < 20; i++)
	{
		fprintf(fp, "SAMP_NUM_COEFF_%d: %.16le\n", i + 1, 0.0);
	}

	/////////////////////////////////// SAMPLE ///////////////////////////////////////
	fprintf(fp, "SAMP_NUM_COEFF_1: %.16le\n", 1.0);
	for (i = 0; i < 3; i++)
	{
		fprintf(fp, "SAMP_DEN_COEFF_%d: %.16le\n", i + 2, m_l[4 + i]);	// l4, l5, l6
	}
	for (i = 4; i < 20; i++)
	{
		fprintf(fp, "SAMP_DEN_COEFF_%d: %.16le\n", i + 1, 0.0);
	}

	fprintf(fp, "wktHCS: %s", Wkt);
	fclose(fp);
}


