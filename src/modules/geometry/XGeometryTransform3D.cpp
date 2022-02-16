#include <math.h>
#include <Eigen/Dense>

//#include "orsMath/orsIMatrixService.h"
#include "XGeometryTransform3D.h"

iplPOINT2D Average( const iplPOINT2D *pts, int n);

iplPOINT3D Average3D( const iplPOINT3D *pts, int n)
{
	iplPOINT3D sum;

	sum.X = sum.Y = sum.Z = 0;
	for(int i=0; i<n; i++)
	{
		sum.X += pts->X;
		sum.Y += pts->Y;
		sum.Z += pts->Z;
		pts++;
	}

	sum.X /= n;
	sum.Y /= n;
	sum.Z /= n;

	return sum;
}

//////////////////////////////////////////////////////////////////////////

using namespace ipl;

//////////////////////////////////////////////////////////////////////////
// xDst - xcDst = a0*( xSrc-xcSrc) + a1*(ySrc-ycSrc) + a2*(zSrc - zcSrc)
// yDst - ycDst = b0*( xSrc-xcSrc) + b1*(ySrc-ycSrc) + b2*(zSrc - zcSrc)
// zDst - zcDst = c0*( xSrc-xcSrc) + c1*(ySrc-ycSrc) + c2*(zSrc - zcSrc)
//////////////////////////////////////////////////////////////////////////

void XAffineTransform3D::Initialize( const iplPOINT3D *ptsSrc, int n, const iplPOINT3D *ptsDst, float *weights )
{
	/////////////////////////////
	m_pcSrc = Average3D( ptsSrc, n);
	m_pcDst = Average3D( ptsDst, n);
	
	////////////////////////////////////
	int i,j, k;
	double a[3];
	Eigen::Matrix3d AA;
	Eigen::Vector3d AL, BL, CL;

// 	orsMatrixD	AA(3,3);
// 	orsVectorD  AL(3), BL(3), CL(3);

	AA = Eigen::Matrix3d::Zero();
	AL[0] = AL[1] = 0;
	BL[0] = BL[1] = 0;
	CL[0] = CL[1] = 0;

	for( k=0; k<n ; k++)
	{
		a[0] = ptsSrc[k].X-m_pcSrc.X;	
		a[1] = ptsSrc[k].Y-m_pcSrc.Y;
		a[2] = ptsSrc[k].Z-m_pcSrc.Z;

		for( i=0; i<3; i++)
		{
			for( j=0; j<3; j++)
				AA(i, j) += a[i]*a[j];

			AL[i] += a[i]* ( ptsDst[k].X - m_pcDst.X );
			BL[i] += a[i]* ( ptsDst[k].Y - m_pcDst.Y );
			CL[i] += a[i]* ( ptsDst[k].Z - m_pcDst.Z );
		}
	}

	//////////////////////////////////////////////////////////////////////////
	Eigen::Matrix3d AI = AA.inverse();
	Eigen::Vector3d X;

// 	orsMatrixD AI = AA;	
// 	orsVectorD X( 3 );
	
// 	getMatrixService()->MatrixInverse( AI );
// 	getMatrixService()->MatrixMultiplyVector( AI, AL, X ); X.CopyData( m_a );
// 	getMatrixService()->MatrixMultiplyVector( AI, BL, X ); X.CopyData( m_b );
// 	getMatrixService()->MatrixMultiplyVector( AI, CL, X ); X.CopyData( m_c );

	X = AI*AL;
	memcpy(m_a, &X[0], sizeof(double) * 3);
	X = AI*BL;
	memcpy(m_b, &X[0], sizeof(double) * 3);
	X = AI*CL;
	memcpy(m_c, &X[0], sizeof(double) * 3);


	m_mx = m_my = m_mz = 0;
	double vx, vy, vz, dx, dy;
	for( k=0; k<n ; k++)
	{
		dx = (ptsSrc[k].X - m_pcSrc.X);	dy = (ptsSrc[k].Y - m_pcSrc.Y);

		vx = ptsDst[k].X - ( m_pcDst.X  + m_a[0]*dx + m_a[1]*dy );
		vy = ptsDst[k].Y - ( m_pcDst.Y  + m_b[0]*dx + m_b[1]*dy );
		vz = ptsDst[k].Z - ( m_pcDst.Z  - m_pcSrc.Z  + ptsSrc[k].Z + m_c[0]*dx + m_c[1]*dy );

		m_mx += vx*vx;
		m_my += vy*vy;
		m_mz += vz*vz;
	}

	m_mx = sqrt( m_mx / (n-3) );
	m_my = sqrt( m_my / (n-3) );
	m_mz = sqrt( m_mz / (n-3) );
}

void XAffineTransform3D::GetResidual( const iplPOINT3D *ptsSrc, int n, const iplPOINT3D *ptsDst, iplPOINT3D *pVxys )
{
	int k;
	double dx, dy, dz;

	for( k=0; k<n ; k++)
	{
		dx = (ptsSrc[k].X - m_pcSrc.X);	
		dy = (ptsSrc[k].Y - m_pcSrc.Y);
		dz = (ptsSrc[k].Z - m_pcSrc.Z);

		pVxys->X = ptsDst[k].X - ( m_pcDst.X + m_a[0]*dx + m_a[1]*dy + m_a[2]*dz );
		pVxys->Y = ptsDst[k].Y - ( m_pcDst.Y + m_b[0]*dx + m_b[1]*dy + m_b[2]*dz );
		pVxys->Z = ptsDst[k].Z - ( m_pcDst.Y + m_c[0]*dx + m_c[1]*dy + m_c[2]*dz  );
		
		pVxys++;
	}
}

void XAffineTransform3D::Transform( const iplPOINT3D *ptsSrc, int n, iplPOINT3D *ptsDst )
{
	int k;
	double dx, dy,dz;

	for( k=0; k<n ; k++)
	{
		dx = (ptsSrc->X - m_pcSrc.X);	
		dy = (ptsSrc->Y - m_pcSrc.Y);
		dz = (ptsSrc->Z - m_pcSrc.Z);
		
		ptsDst->X = m_pcDst.X + m_a[0]*dx + m_a[1]*dy + m_a[2]*dz;
		ptsDst->Y = m_pcDst.Y + m_b[0]*dx + m_b[1]*dy + m_b[2]*dz;	
		ptsDst->Z = ptsSrc->Z + m_c[0]*dx + m_c[1]*dy + m_a[3]*dz;

		ptsSrc++;	ptsDst++;
	}
}


void XAffineTransform3D::GetMeanError( double *mx,double *my, double *mz )
{
	*mx = m_mx;
	*my = m_my;
	*mz = m_mz;
}



//////////////////////////////////////////////////////////////////////////
// xDst - xcDst = a0*( xSrc-xcSrc) + a1*(ySrc-ycSrc) + a2*(zSrc-zcSrc)
// yDst - ycDst = b0*( xSrc-xcSrc) + b1*(ySrc-ycSrc) + b2*(zSrc-zcSrc)
// zDst - zcDst = c0*( xSrc-xcSrc) + c1*(ySrc-ycSrc) + c2*(zSrc-zcSrc)
//////////////////////////////////////////////////////////////////////////
void XAffineTransform3D::GetParameter( iplPOINT3D *pcSrc, iplPOINT3D *pcDst,double *a, double *b,  double *c )
{	
	*pcSrc = m_pcSrc;
	*pcDst = m_pcDst;
	
	for( int i=0; i<3; i++)
	{
		a[i] = m_a[i];
		b[i] = m_b[i];
		c[i] = m_c[i];
	}
};

//
// xDst = xcDst + a0*( xSrc-xcSrc) + a1*(ySrc-ycSrc) + a2*(zSrc-zcSrc)
//		= xcDst - ( a0*xcSrc + a1*ycSrc + a2*zcSrc) + ( a0*xSrc + a1*ySrc + a2*zSrc);
//
// yDst = ycDst + b0*( xSrc-xcSrc) + b1*(ySrc-ycSrc) + b2*(zSrc-zcSrc)
//		= ycDst - ( b0*xcSrc + b1*ycSrc + b2*zcSrc) + ( b0*xSrc + b1*ySrc + b2*zSrc);
//
// zDst = zcDst + c0*( xSrc-xcSrc) + c1*(ySrc-ycSrc) + c2*(zSrc-zcSrc)
//		= zcDst - ( c0*xcSrc + c1*ycSrc + c2*zcSrc) + ( c0*xSrc + c1*ySrc + c2*zSrc);
//
void XAffineTransform3D::GetParameter( double *a, double *b, double *c )
{	
	a[1] = m_a[0];	a[2] = m_a[1];	a[3] = m_a[2];
	b[1] = m_b[0];	b[2] = m_b[1];	b[3] = m_b[2];
	c[1] = m_b[0];	c[2] = m_c[1];	c[3] = m_c[2];
	
	a[0] = m_pcDst.X - a[1]*m_pcSrc.X - a[2]*m_pcSrc.Y - a[3]*m_pcSrc.Z;
	b[0] = m_pcDst.Y - b[1]*m_pcSrc.Y - b[2]*m_pcSrc.Y - b[3]*m_pcSrc.Z;
	c[0] = m_pcDst.Z - c[1]*m_pcSrc.Y - c[2]*m_pcSrc.Y - c[3]*m_pcSrc.Z;
};



///


//////////////////////////////////////////////////////////////////////////
// simplified XAffineTransform3D:	a2=b2=0, c2=1
//		xDst - xcDst = a0*( xSrc-xcSrc) + a1*(ySrc-ycSrc)
//		yDst - ycDst = b0*( xSrc-xcSrc) + b1*(ySrc-ycSrc)
//		zDst - zcDst = c0*( xSrc-xcSrc) + c1*(ySrc-ycSrc) + zSrc - zcSrc
//////////////////////////////////////////////////////////////////////////

void XAffineTransform25D::Initialize( const iplPOINT3D *ptsSrc, int n, const iplPOINT3D *ptsDst, float *weights )
{
	/////////////////////////////
	m_pcSrc = Average3D( ptsSrc, n);	//m_pcSrc.Z = 0;
	m_pcDst = Average3D( ptsDst, n);	//m_pcDst.Z = 0;
	
	////////////////////////////////////
	int i,j, k;
	double a[2];
	Eigen::Matrix2d AA;
	Eigen::Vector2d AL, BL, CL;

// 	orsMatrixD	AA(2,2);
// 	orsVectorD  AL(2), BL(2), CL(2);

	AA = Eigen::Matrix2d::Zero();
	AL[0] = AL[1] = 0;
	BL[0] = BL[1] = 0;
	CL[0] = CL[1] = 0;

	for( k=0; k<n ; k++)
	{
		a[0] = ptsSrc[k].X-m_pcSrc.X;	a[1] = ptsSrc[k].Y-m_pcSrc.Y;

		for( i=0; i<2; i++)
		{
			for( j=0; j<2; j++)
				AA(i,j) += a[i]*a[j];

			AL[i] += a[i]* ( ptsDst[k].X - m_pcDst.X );
			BL[i] += a[i]* ( ptsDst[k].Y - m_pcDst.Y );
			CL[i] += a[i]* ( ptsDst[k].Z - ptsSrc[k].Z );
		}
	}

	//////////////////////////////////////////////////////////////////////////
	Eigen::Matrix2d AI = AA.inverse();
	Eigen::Vector2d X;

// 	orsMatrixD AI = AA;	
// 	orsVectorD X( 2 );
	
// 	getMatrixService()->MatrixInverse( AI );
// 	getMatrixService()->MatrixMultiplyVector( AI, AL, X ); X.CopyData( m_a );
// 	getMatrixService()->MatrixMultiplyVector( AI, BL, X ); X.CopyData( m_b );
// 	getMatrixService()->MatrixMultiplyVector( AI, CL, X ); X.CopyData( m_c );

	X = AI*AL;
	memcpy(m_a, &X[0], sizeof(double) * 2);
	X = AI*BL;
	memcpy(m_b, &X[0], sizeof(double) * 2);
	X = AI*CL;
	memcpy(m_c, &X[0], sizeof(double) * 2);

	m_mx = m_my = m_mz = 0;
	double vx, vy, vz, dx, dy, dz;
	for( k=0; k<n ; k++)
	{
		dx = (ptsSrc[k].X - m_pcSrc.X);	
		dy = (ptsSrc[k].Y - m_pcSrc.Y);
		dz = (ptsSrc[k].Z - m_pcSrc.Z);

		vx = ptsDst[k].X - ( m_pcDst.X  + m_a[0]*dx + m_a[1]*dy );
		vy = ptsDst[k].Y - ( m_pcDst.Y  + m_b[0]*dx + m_b[1]*dy );
		vz = ptsDst[k].Z - ( m_pcDst.Z  + m_c[0]*dx + m_c[1]*dy + dz );

		m_mx += vx*vx;
		m_my += vy*vy;
		m_mz += vz*vz;
	}

	m_mx = sqrt( m_mx / (n-3) );
	m_my = sqrt( m_my / (n-3) );
	m_mz = sqrt( m_mz / (n-3) );
}

void XAffineTransform25D::GetResidual( const iplPOINT3D *ptsSrc, int n, const iplPOINT3D *ptsDst, iplPOINT3D *pVxys )
{
	int k;
	double dx, dy, dz;

	for( k=0; k<n ; k++)
	{
		dx = (ptsSrc[k].X - m_pcSrc.X);	
		dy = (ptsSrc[k].Y - m_pcSrc.Y);
		dz = (ptsSrc[k].Z - m_pcSrc.Z);

		pVxys->X = ptsDst[k].X - ( m_pcDst.X + m_a[0]*dx + m_a[1]*dy );
		pVxys->Y = ptsDst[k].Y - ( m_pcDst.Y + m_b[0]*dx + m_b[1]*dy );
		pVxys->Z = ptsDst[k].Z - ( m_pcDst.Z + m_c[0]*dx + m_c[1]*dy + dz );
		
		pVxys++;
	}
}

void XAffineTransform25D::Transform( const iplPOINT3D *ptsSrc, int n, iplPOINT3D *ptsDst )
{
	int k;
	double dx, dy, dz;

	for( k=0; k<n ; k++)
	{
		dx = (ptsSrc->X - m_pcSrc.X);	
		dy = (ptsSrc->Y - m_pcSrc.Y);
		dz = (ptsSrc->Z - m_pcSrc.Z);
		
		ptsDst->X = m_pcDst.X + m_a[0]*dx + m_a[1]*dy;
		ptsDst->Y = m_pcDst.Y + m_b[0]*dx + m_b[1]*dy;	
		ptsDst->Z = m_pcDst.Z + m_c[0]*dx + m_c[1]*dy + dz;

		ptsSrc++;	ptsDst++;
	}
}


void XAffineTransform25D::GetMeanError( double *mx,double *my, double *mz )
{
	*mx = m_mx;
	*my = m_my;
	*mz = m_mz;
}



//////////////////////////////////////////////////////////////////////////
// xDst - xcDst = a0*( xSrc-xcSrc) + a1*(ySrc-ycSrc)
// yDst - ycDst = b0*( xSrc-xcSrc) + b1*(ySrc-ycSrc)
//
//////////////////////////////////////////////////////////////////////////
void XAffineTransform25D::GetParameter( iplPOINT3D *pcSrc, iplPOINT3D *pcDst,double *a, double *b,  double *c )
{	
	*pcSrc = m_pcSrc;
	*pcDst = m_pcDst;
	
	for( int i=0; i<2; i++)
	{
		a[i] = m_a[i];
		b[i] = m_b[i];
		c[i] = m_c[i];
	}
};

//
// xDst = xcDst + a0*( xSrc-xcSrc) + a1*(ySrc-ycSrc)
//		= xcDst -(a0*xcSrc + a1*ycSrc) + a0*xSrc + a1*ySrc;
//
// yDst = ycDst + b0*( xSrc-xcSrc) + b1*(ySrc-ycSrc)
//		= ycDst -(b0*xcSrc + b1*ycSrc) + b0*xSrc + b1*ySrc;
//
// zDst = zcDst + c0*( xSrc-xcSrc) + c1*(ySrc-ycSrc) + (zSrc-zcSrc)
//		= zcDst -(c0*xcSrc + c1*ycSrc + zcSrc ) + c0*xSrc + c1*ySrc + zSrc;
//
void XAffineTransform25D::GetParameter( double *a, double *b, double *c )
{	
	a[1] = m_a[0];	a[2] = m_a[1];
	b[1] = m_b[0];	b[2] = m_b[1];
	c[1] = m_c[0];	c[2] = m_c[1];

	a[0] = m_pcDst.X - a[1]*m_pcSrc.X - a[2]*m_pcSrc.Y;
	b[0] = m_pcDst.Y - b[1]*m_pcSrc.X - b[2]*m_pcSrc.Y;
	c[0] = m_pcDst.Z - c[1]*m_pcSrc.X - c[2]*m_pcSrc.Y - m_pcSrc.Z;
};







//////////////////////////////////////////////////////////////////////////
//				   L0*( xSrc-xcSrc) + L1*(ySrc-ycSrc) + L2*(zSrc-zcSrc) + L3
// xDst - xcDst = -----------------------------------------------------------
//				   L8*( xSrc-xcSrc) + L9*(ySrc-ycSrc) + L10*(zSrc-zcSrc) + 1
//
//				   L4*( xSrc-xcSrc) + L5*(ySrc-ycSrc) + L6*(zSrc-zcSrc) + L7
// yDst - ycDst = ------------------------------------------------------------
//				   L8*( xSrc-xcSrc) + L9*(ySrc-ycSrc) + L10*(zSrc-zcSrc) + 1
//
//				   
// dxDst*(l6*dxSrc + l7*dySrc + 1) = l0*dxSrc + l1*dySrc + l2
//
// dyDst*(l6*dxSrc + l7*dySrc + 1) = l3*dxSrc + l4*dySrc + l5
//
//////////////////////////////////////////////////////////////////////////
XDLTTransform_3D::XDLTTransform_3D()
{
	m_l[10] = 1;
}



void XDLTTransform_3D::Initialize( const iplPOINT3D *ptsSrc, int n,  const iplPOINT2D *ptsDst, float *weights )
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
	double a[11], b[11],  dxSrc, dySrc, dzSrc, dxDst, dyDst, lx, ly;

	Eigen::MatrixXd AA(11, 11);
	Eigen::VectorXd AL(11);

// 	orsMatrixD	AA(11,11);
// 	orsVectorD	AL(11);

	AA = Eigen::MatrixXd::Zero(11, 11);	
	AL = Eigen::VectorXd::Zero(11);

	memset(a, 0, 11*sizeof(double) );
	memset(b, 0, 11*sizeof(double) );

	for( k=0; k<n ; k++)
	{ 
		dxSrc = (ptsSrc[k].X - m_pcSrc.X)/m_XScale;	
		dySrc = (ptsSrc[k].Y - m_pcSrc.Y)/m_YScale;
		dzSrc = (ptsSrc[k].Z - m_pcSrc.Z)/m_ZScale;

		dxDst = (ptsDst[k].x - m_pcDst.x)/m_xScale;	
		dyDst = (ptsDst[k].y - m_pcDst.y)/m_yScale;	

		a[0] = dxSrc;	a[1] = dySrc;	a[2] = dzSrc;	a[3] = 1;
		a[8] = -dxDst*dxSrc;	a[9] = -dxDst*dySrc;	a[10] = -dxDst*dzSrc;
		lx = dxDst;

		b[4] = dxSrc;	b[5] = dySrc;	b[6] = dzSrc;	b[7] = 1;
		b[8] = -dyDst*dxSrc;	b[8] = -dyDst*dySrc;	b[10] = -dyDst*dzSrc;		
		ly = dyDst;

		for( i=0; i<11; i++)
		{
			for( j=0; j<11; j++)
				AA(i, j) += a[i]*a[j] + b[i]*b[j];

			AL[i] += a[i]* lx + b[i]* ly;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	Eigen::VectorXd X(11);

	double det = AA.determinant();
	if (fabs(det) > 1e-6)
		X = AA.colPivHouseholderQr().solve(AL); //是否稳定可靠，未经验证，AA可能奇异
	else
		X = AA.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(AL);

	memcpy(m_l, &X[0], sizeof(double) * 11);
		
// 	orsVectorD	X(11);
// 	getMatrixService()->SolveLinearEqs_K( AA, AL, X, 0.0001 );
// 	
// 	X.CopyData( m_l );

	//////////////////////////////////////////////////////////////////////////

	m_mx = m_my = 0;
	double vx, vy, w;
	for( k=0; k<n ; k++)
	{
		dxSrc = (ptsSrc[k].X - m_pcSrc.X)/m_XScale;	
		dySrc = (ptsSrc[k].Y - m_pcSrc.Y)/m_YScale;
		dzSrc = (ptsSrc[k].Z - m_pcSrc.Z)/m_ZScale;
		
		w = m_l[8]*dxSrc + m_l[9]*dySrc + m_l[10]*dzSrc +  1;

		vx = ptsDst[k].x-m_pcDst.x - m_xScale*( m_l[0]*dxSrc + m_l[1]*dySrc + m_l[2]*dzSrc + m_l[3] ) / w ;
		vy = ptsDst[k].y-m_pcDst.y - m_yScale*( m_l[4]*dxSrc + m_l[5]*dySrc + m_l[6]*dzSrc + m_l[7] ) / w ;

		m_mx += vx*vx;
		m_my += vy*vy;
	}

	m_mx = sqrt( m_mx / (n-6) );
	m_my = sqrt( m_my / (n-6) );
}



void XDLTTransform_3D::GetResidual( const iplPOINT3D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys )
{
	int k;
	double w;
	double	dxSrc, dySrc, dzSrc, dxDst, dyDst;

	for( k=0; k<n ; k++)
	{
		dxSrc = (ptsSrc->X - m_pcSrc.X)/m_XScale;	
		dySrc = (ptsSrc->Y - m_pcSrc.Y)/m_YScale;
		dzSrc = (ptsSrc->Z - m_pcSrc.Z)/m_ZScale;
		
		w = m_l[8]*dxSrc + m_l[9]*dySrc + m_l[10]*dzSrc +  1;
		
		pVxys->x = ptsDst->x - m_pcDst.x - m_xScale*( m_l[0]*dxSrc + m_l[1]*dySrc + m_l[2]*dzSrc + m_l[3] ) / w;
		pVxys->y = ptsDst->y - m_pcDst.y - m_yScale*( m_l[4]*dxSrc + m_l[5]*dySrc + m_l[6]*dzSrc + m_l[7] ) / w;
		
		ptsSrc++;	ptsDst++;	pVxys++;
	}
}



void XDLTTransform_3D::Transform( const iplPOINT3D *ptsSrc, int n, iplPOINT2D *ptsDst )
{
	int k;

	double w;
	double	dxSrc, dySrc, dzSrc, dxDst, dyDst;

	for( k=0; k<n ; k++)
	{
		dxSrc = (ptsSrc->X - m_pcSrc.X)/m_XScale;	
		dySrc = (ptsSrc->Y - m_pcSrc.Y)/m_YScale;
		dzSrc = (ptsSrc->Z - m_pcSrc.Z)/m_ZScale;
				
		w = m_l[8]*dxSrc + m_l[9]*dySrc + m_l[10]*dzSrc +  1;
		
		dxDst = ( m_l[0]*dxSrc + m_l[1]*dySrc + m_l[2]*dzSrc + m_l[3] ) / w;
		dyDst = ( m_l[4]*dxSrc + m_l[5]*dySrc + m_l[6]*dzSrc + m_l[7] ) / w;

		ptsDst->x = m_pcDst.x + m_xScale*dxDst;	
		ptsDst->y = m_pcDst.y + m_yScale*dyDst;	
				
		ptsSrc++;	ptsDst++;
	}
}



void XDLTTransform_3D::GetMeanError( double *mx,double *my )
{
	*mx = m_mx;
	*my = m_my;
}



void XDLTTransform_3D::GetParameter( iplPOINT3D *pcSrc, iplPOINT2D *pcDst, double *a, double *b, double *c )
{
	*pcSrc = m_pcSrc;
	*pcDst = m_pcDst;

	double l[12];

	l[0] = m_l[0]*m_xScale/m_XScale;
	l[1] = m_l[1]*m_xScale/m_YScale;
	l[2] = m_l[2]*m_xScale/m_ZScale;
	l[3] = m_l[3]*m_xScale;
	
	l[4] = m_l[4]*m_yScale/m_XScale;
	l[5] = m_l[5]*m_yScale/m_YScale;
	l[6] = m_l[6]*m_yScale/m_ZScale;
	l[7] = m_l[7]*m_yScale;
	
	l[8] = m_l[8]/m_XScale;
	l[9] = m_l[9]/m_YScale;
	l[10] = m_l[10]/m_ZScale;
	l[11] = 1;

	if( NULL == b )	{
		memcpy( a, l, 12*sizeof(double));
	}
	else	{
		memcpy( a, l, 4*sizeof(double));
		memcpy( b, l+4, 4*sizeof(double));
		memcpy( c, l+8, 4*sizeof(double));
	}
}



// 取变换参数，中心点计入参数
void XDLTTransform_3D::GetParameter( double *a, double *b, double *c )
{
	int i;

	double l[12];

	l[0] = m_l[0]*m_xScale/m_XScale;
	l[1] = m_l[1]*m_xScale/m_YScale;
	l[2] = m_l[2]*m_xScale/m_ZScale;
	l[3] = m_l[3]*m_xScale;
	
	l[4] = m_l[4]*m_yScale/m_XScale;
	l[5] = m_l[5]*m_yScale/m_YScale;
	l[6] = m_l[6]*m_yScale/m_ZScale;
	l[7] = m_l[7]*m_yScale;
	
	l[8] = m_l[8]/m_XScale;
	l[9] = m_l[9]/m_YScale;
	l[10] = m_l[10]/m_ZScale;

//////////////////////////////////////////////////////////////////////////
//				   l0*( xSrc-xcSrc) + l1*(ySrc-ycSrc) +  l2*(zSrc-zcSrc) + l3
// xDst - xcDst = ---------------------------------------------------------------------
//				   l8*( xSrc-xcSrc) + l9*(ySrc-ycSrc) + l10*(zSrc-zcSrc) + 1
// ==>
//		           l0*xSrc + l1*ySrc + l2*zSrc +  (l3 - l0*xcSrc - l1*ycSrc - l2*zcSrc)
// xDst = xcDst + ----------------------------------------------------------------------
//		           l8*xSrc + l9*ySrc + l10*zSrc + (1  - l8*xcSrc - l9*ycSrc - l10*zcSrc )
//
// ==>
//
//		   (l0 + l8*xcDst)*xSrc + (l1 + l9*xcDst)*ySrc + (l2 + l10*xcDst)*zSrc + (l3 - l0*xcSrc - l1*ycSrc - l2*zcSrc) + xcDst*l11 
// xDst = ----------------------------------------------------------------------------------------------------------------------------------
//		    l8*xSrc + l9*ySrc + l10*zSrc + (1 - l8*xcSrc - l9*ycSrc - l10*zcSrc )=l11
	
	double xc, yc;
	
	xc = m_pcDst.x;
	yc = m_pcDst.y;
	
	double l11 = 1 - l[8]*m_pcSrc.X - l[9]*m_pcSrc.Y - l[10]*m_pcSrc.Z;
	
	// 不能置后， 否则l[0],l[1]已被修改
	l[3] += l11*xc - l[0]*m_pcSrc.X - l[1]*m_pcSrc.Y - l[2]*m_pcSrc.Z;
	l[0] += l[8]*xc;
	l[1] += l[9]*xc;
	l[2] += l[10]*xc;

	l[7] += l11*yc - l[4]*m_pcSrc.X - l[5]*m_pcSrc.Y - l[6]*m_pcSrc.Z;
	l[4] += l[8]*yc;
	l[5] += l[9]*yc;
	l[6] += l[10]*yc;

	for( i=0; i<11; i++)
		l[i] /= l11;

	l[11] = 1;

	if( NULL == b )	{
		memcpy( a, l, 12*sizeof(double));
	}
	else	{
		memcpy( a, l, 4*sizeof(double));
		memcpy( b, l+4, 4*sizeof(double));
		memcpy( c, l+8, 4*sizeof(double));
	}
}




///////////////////////////////////////////////////////////////////////////////
//				   l0*(xSrc-xcSrc) + l1*(ySrc-ycSrc) + l2*(zSrc-zcSrc) + l3
// xDst - xcDst = ------------------------------------------------------------
//				   l8*(xSrc-xcSrc) + l9*(ySrc-ycSrc) + l10*(zSrc-zcSrc) + 1
//
//				   l4*(xSrc-xcSrc) + l5*(ySrc-ycSrc) + l6*(zSrc-zcSrc) + l7
// yDst - ycDst = ------------------------------------------------------------ + l11*(xDst-xcDst)*(yDst-ycDst)
//				   l8*(xSrc-xcSrc) + l9*(ySrc-ycSrc) + l10*(zSrc-zcSrc) + 1
/////////////////////////////////////////////////////////////////////////////////
XSDLTTransform_3D::XSDLTTransform_3D()
{

}

void XSDLTTransform_3D::Initialize( const iplPOINT3D *ptsSrc, int n,  const iplPOINT2D *ptsDst, float *weights )
{
// 	assert( n >= 7 );
	
	int i, j, k;
	m_pcSrc = Average3D( ptsSrc, n );
	m_pcDst = Average( ptsDst, n );

	m_xScale = m_yScale = 0;
	m_XScale = m_YScale = m_ZScale = 0;

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

	double a[12], b[12], m_l0[12];
	double dxSrc, dySrc, dzSrc, dxDst, dyDst, dxDst0, dyDst0, lx, ly, w;

	Eigen::MatrixXd AA(12, 12);
	Eigen::VectorXd AL(12), vX(12);
// 	orsMatrixD	AA(12,12);
// 	orsVectorD	AL(12);
// 	orsVectorD	vX(12);

	memset(m_l0, 0, 12*sizeof(double));
	memset(m_l, 0, 12*sizeof(double));

	int iter;
	for(iter=0; iter<30; iter++)
	{
		AA = Eigen::MatrixXd::Zero(12, 12);	
		AL = Eigen::VectorXd::Zero(12);
		
		memset(a, 0, 12*sizeof(double) );
		memset(b, 0, 12*sizeof(double) );
		
		for( k=0; k<n; k++ )
		{ 
 			dxSrc = (ptsSrc[k].X - m_pcSrc.X)/m_XScale;	
 			dySrc = (ptsSrc[k].Y - m_pcSrc.Y)/m_YScale;
 			dzSrc = (ptsSrc[k].Z - m_pcSrc.Z)/m_ZScale;
 			
 			dxDst = (ptsDst[k].x - m_pcDst.x)/m_xScale;	
 			dyDst = (ptsDst[k].y - m_pcDst.y)/m_yScale;
			
			w = 1.0 / (m_l[8]*dxSrc + m_l[9]*dySrc + m_l[10]*dzSrc + 1);
			
			a[0] = b[4] = dxSrc*w;
			a[1] = b[5] = dySrc*w;
			a[2] = b[6] = dzSrc*w;
			a[3] = b[7] = w;

			a[8] = -(dxDst*dxSrc)*w;	a[9] = -(dxDst*dySrc)*w;	a[10] = -(dxDst*dzSrc)*w;
			a[11] = 0;	lx = dxDst*w;

			b[8] = -(dyDst*dxSrc)*w;	b[9] = -(dyDst*dySrc)*w;	b[10] = -(dyDst*dzSrc)*w;		
			b[11] = dxDst*dyDst;	ly = dyDst*w;
			
			for( i=0; i<12; i++)
			{
				for( j=0; j<12; j++)
					AA(i,j) += a[i]*a[j] + b[i]*b[j];
				
				AL[i] += a[i]* lx + b[i]* ly;
			}
		}
		double det = AA.determinant();
		if (fabs(det) > 1e-6)
			vX = AA.colPivHouseholderQr().solve(AL); //是否稳定可靠，未经验证，AA可能奇异
		else
			vX = AA.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(AL);
		memcpy(m_l, &vX[0], sizeof(double) * 12);

// 		getMatrixService()->SolveLinearEqs_K( AA, AL, vX, 0.0001 );
// 		vX.CopyData( m_l );

		if(iter > 0) {
			double delmax = 0;
			m_mx = m_my = 0;

			for( k=0; k<n; k++ )
			{
				dxSrc = (ptsSrc[k].X - m_pcSrc.X)/m_XScale;
				dySrc = (ptsSrc[k].Y - m_pcSrc.Y)/m_YScale;
				dzSrc = (ptsSrc[k].Z - m_pcSrc.Z)/m_ZScale;

				w = 1.0 / (m_l0[8]*dxSrc + m_l0[9]*dySrc + m_l0[10]*dzSrc + 1);
				
				dxDst0 = ( m_l0[0]*dxSrc + m_l0[1]*dySrc + m_l0[2]*dzSrc + m_l0[3] ) * w;
				dyDst0 = ( m_l0[4]*dxSrc + m_l0[5]*dySrc + m_l0[6]*dzSrc + m_l0[7] ) * w;
				dyDst0 = dyDst0 / (1.0 - m_l0[11]*dxDst0);
				
				w = 1.0 / (m_l[8]*dxSrc + m_l[9]*dySrc + m_l[10]*dzSrc + 1);
				
				dxDst = ( m_l[0]*dxSrc + m_l[1]*dySrc + m_l[2]*dzSrc + m_l[3] ) * w;
				dyDst = ( m_l[4]*dxSrc + m_l[5]*dySrc + m_l[6]*dzSrc + m_l[7] ) * w;
				dyDst = dyDst / (1.0 - m_l[11]*dxDst);
				
				lx = (dxDst - dxDst0)*m_xScale;
				ly = (dxDst - dxDst0)*m_yScale;

				if( delmax < fabs(lx) ) delmax = fabs(lx);
				if( delmax < fabs(ly) ) delmax = fabs(ly);

				lx = ptsDst[k].x - m_pcDst.x - m_xScale*dxDst;
				ly = ptsDst[k].y - m_pcDst.y - m_yScale*dyDst;
				
				m_mx += lx*lx;
				m_my += ly*ly;
			}

			if( delmax < 1e-6 )
				break;
		}

		memcpy(m_l0, m_l, 12*sizeof(double));
	}

	m_mx = sqrt( m_mx / (n-6) );
	m_my = sqrt( m_my / (n-6) );
}

void XSDLTTransform_3D::GetResidual( const iplPOINT3D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys )
{
	int k;
	double w;
	double	dxSrc, dySrc, dzSrc, dxDst, dyDst;

	for( k=0; k<n ; k++)
	{
		dxSrc = (ptsSrc->X - m_pcSrc.X)/m_XScale;	
		dySrc = (ptsSrc->Y - m_pcSrc.Y)/m_YScale;
		dzSrc = (ptsSrc->Z - m_pcSrc.Z)/m_ZScale;
		
		w = 1.0 / (m_l[8]*dxSrc + m_l[9]*dySrc + m_l[10]*dzSrc + 1);
			
		dxDst = ( m_l[0]*dxSrc + m_l[1]*dySrc + m_l[2]*dzSrc + m_l[3] ) * w;
		dyDst = ( m_l[4]*dxSrc + m_l[5]*dySrc + m_l[6]*dzSrc + m_l[7] ) * w;
		dyDst = dyDst / (1.0 - m_l[11]*dxDst);
		
		pVxys->x = ptsDst->x - m_pcDst.x - m_xScale*dxDst;
		pVxys->y = ptsDst->y - m_pcDst.y - m_yScale*dyDst;
		
		ptsSrc++;	ptsDst++;	pVxys++;
	}
}

void XSDLTTransform_3D::Transform( const iplPOINT3D *ptsSrc, int n, iplPOINT2D *ptsDst )
{
	int k;

	double w;
	double	dxSrc, dySrc, dzSrc, dxDst, dyDst;

	for( k=0; k<n ; k++)
	{
		dxSrc = (ptsSrc->X - m_pcSrc.X)/m_XScale;	
		dySrc = (ptsSrc->Y - m_pcSrc.Y)/m_YScale;
		dzSrc = (ptsSrc->Z - m_pcSrc.Z)/m_ZScale;
				
		w = 1.0 / (m_l[8]*dxSrc + m_l[9]*dySrc + m_l[10]*dzSrc + 1);
			
		dxDst = ( m_l[0]*dxSrc + m_l[1]*dySrc + m_l[2]*dzSrc + m_l[3] ) * w;
		dyDst = ( m_l[4]*dxSrc + m_l[5]*dySrc + m_l[6]*dzSrc + m_l[7] ) * w;
		dyDst = dyDst / (1.0 - m_l[11]*dxDst);

		ptsDst->x = m_pcDst.x + m_xScale*dxDst;	
		ptsDst->y = m_pcDst.y + m_yScale*dyDst;	
				
		ptsSrc++;	ptsDst++;
	}
}

void XSDLTTransform_3D::GetMeanError( double *mx,double *my )
{
	*mx = m_mx;
	*my = m_my;
}

void XSDLTTransform_3D::GetParameter( iplPOINT3D *pcSrc, iplPOINT2D *pcDst, double *a, double *b, double *c )
{
	*pcSrc = m_pcSrc;
	*pcDst = m_pcDst;

	double l[12];

	l[0] = m_l[0]*m_xScale/m_XScale;
	l[1] = m_l[1]*m_xScale/m_YScale;
	l[2] = m_l[2]*m_xScale/m_ZScale;
	l[3] = m_l[3]*m_xScale;
	
	l[4] = m_l[4]*m_yScale/m_XScale;
	l[5] = m_l[5]*m_yScale/m_YScale;
	l[6] = m_l[6]*m_yScale/m_ZScale;
	l[7] = m_l[7]*m_yScale;
	
	l[8] = m_l[8]/m_XScale;
	l[9] = m_l[9]/m_YScale;
	l[10] = m_l[10]/m_ZScale;
	l[11] = m_l[11]/m_xScale;

	if( NULL == b ) { // 返回SDLT中心化参数
		memcpy( a, l, 12*sizeof(double));
	}
	else { // 返回RPC01中心化参数
		memcpy( a, l, 4*sizeof(double));
		memcpy( a+4, l+8, 3*sizeof(double));

		memcpy( b, l+4, 4*sizeof(double));
		memcpy( b+4, l+8, 3*sizeof(double));

		b[4] -= l[11]*l[0];
		b[5] -= l[11]*l[1];
		b[6] -= l[11]*l[2];

		double t = 1.0 / (1.0 - l[11]*l[3]);

		for(int i=0; i<7; i++)
			b[i] *= t;
	}
}

// 取变换参数，中心点计入参数
void XSDLTTransform_3D::GetParameter( double *a, double *b, double *c )
{
	if( NULL == b ) {
		assert(false); // 无法将内部SDLT中心化参数直接转换为SDLT非中心化参数
	}
	else { // 返回RPC01非中心化参数
		double l[14];
		iplPOINT3D pcSrc;
		iplPOINT2D pcDst;

		GetParameter(&pcSrc, &pcDst, l, l+7);

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
//		   (l0 + l4*xcDst)*xSrc + (l1 + l5*xcDst)*ySrc + (l2 + l6*xcDst)*zSrc + (l3 - l0*xcSrc - l1*ycSrc - l2*zcSrc) + xcDst*l7x 
// xDst = ----------------------------------------------------------------------------------------------------------------------------------
//		    l4*xSrc + l5*ySrc + l6*zSrc + (1 - l4*xcSrc - l5*ycSrc - l6*zcSrc )-->l7x

		int i;
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
		
		memcpy( a, l, 7*sizeof(double));
		memcpy( b, l+7, 7*sizeof(double));
	}
}

