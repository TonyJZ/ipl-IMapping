#include <math.h>
#include <Eigen/Dense>
#include <Eigen/SVD>



//#include "orsMath/orsIMatrixService.h"
#include "XGeometryTransform2D.h"

double Average(double *x, int n)
{
	double sum;

	sum = 0;
	for (int i = 0; i < n; i++)
		sum += *x++;

	return sum / n;
}


iplPOINT2D Average(const iplPOINT2D *pts, int n)
{
	iplPOINT2D sum;

	sum.x = sum.y = 0;
	for (int i = 0; i < n; i++)
	{
		sum.x += pts->x;
		sum.y += pts->y;
		pts++;
	}

	sum.x /= n;
	sum.y /= n;

	return sum;
}

using namespace ipl;
//using namespace Eigen;

void XTranslationTransform2D::Initialize(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights)
{
	int k;
	double w;
	double atemp = 0, btemp = 0, wtemp = 0;;
	if (NULL != weights)	{
		for (k = 0; k < n; k++)
		{
			w = weights[k];
			wtemp += w;

			atemp += w*(ptsDst[k].x - ptsSrc[k].x);
			btemp += w*(ptsDst[k].y - ptsSrc[k].y);
		}
		m_a[0] = atemp / wtemp;
		m_b[0] = btemp / wtemp;
	}
	else	{
		for (k = 0; k < n; k++)
		{

			atemp += (ptsDst[k].x - ptsSrc[k].x);
			btemp += (ptsDst[k].y - ptsSrc[k].y);
		}
		m_a[0] = atemp / n;
		m_b[0] = btemp / n;
	}

	m_mx = m_my = 0;
	double vx, vy;
	for (k = 0; k < n; k++)
	{
		vx = (ptsDst[k].x - ptsSrc[k].x) - m_a[0];
		vy = (ptsDst[k].y - ptsSrc[k].y) - m_a[1];

		m_mx += vx*vx;
		m_my += vy*vy;
	}

	if (n > 1)	{
		m_mx = sqrt(m_mx / (n - 1));
		m_my = sqrt(m_my / (n - 1));
	}
	else	{
		m_mx = m_my = 0;
	}

}

void XTranslationTransform2D::GetResidual(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys)
{
	int k;

	for (k = 0; k < n; k++)
	{
		pVxys->x = (ptsDst[k].x - ptsSrc[k].x) - m_a[0];
		pVxys->y = (ptsDst[k].y - ptsSrc[k].y) - m_b[0];

		pVxys++;
	}
}

void XTranslationTransform2D::Transform(const iplPOINT2D *ptsSrc, int n, iplPOINT2D *ptsDst)
{
	int k;

	for (k = 0; k < n; k++)
	{
		ptsDst->x = m_a[0] + ptsSrc->x;
		ptsDst->y = m_b[0] + ptsSrc->y;

		ptsSrc++;	ptsDst++;
	}
}

void XTranslationTransform2D::Transform(double *x, double *y)
{
	*x = m_a[0] + *x;
	*y = m_b[0] + *y;
}

void XTranslationTransform2D::GetMeanError(double *mx, double *my)
{
	*mx = m_mx;
	*my = m_my;
}



//////////////////////////////////////////////////////////////////////////
// xDst - xcDst = a0*( xSrc-xcSrc) + a1*(ySrc-ycSrc)
// yDst - ycDst = b0*( xSrc-xcSrc) + b1*(ySrc-ycSrc)
//
//////////////////////////////////////////////////////////////////////////
void XTranslationTransform2D::GetParameter(iplPOINT2D *pcSrc, iplPOINT2D *pcDst, double *a, double *b)
{
	pcSrc->x = 0;
	pcSrc->y = 0;
	pcDst->x = 0;
	pcDst->y = 0;

	a[0] = m_a[0];
	b[0] = m_b[0];
};

//
// xDst = xcDst + a0*( xSrc-xcSrc) + a1*(ySrc-ycSrc)
//		= xcDst - a0*xcSrc - a1*ycSrc + a0*xSrc + a1*ySrc;
//
// yDst = ycDst + b0*( xSrc-xcSrc) + b1*(ySrc-ycSrc)
//		= ycDst - b0*xcSrc - b1*ycSrc + a0*xSrc + a1*ySrc;
//
void XTranslationTransform2D::GetParameter(double *a, double *b)
{
	a[0] = m_a[0];	a[1] = 1; a[2] = 0;
	b[0] = m_b[0];	b[1] = 0; b[2] = 1;
};


void XAffineTransform2D::Initialize(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights)
{
	/////////////////////////////
	m_pcSrc = Average(ptsSrc, n);
	m_pcDst = Average(ptsDst, n);

	////////////////////////////////////
	int i, j, k;
	double a[2], w;
	Eigen::Matrix<double, 2, 2> AA;
	Eigen::Vector2d AL, BL, CL;

// 	orsMatrixD	AA(2, 2);
// 	orsVectorD  AL(2), BL(2), CL(2);
	
	AA = Eigen::MatrixXd::Zero(2, 2);
	AL[0] = AL[1] = 0;
	BL[0] = BL[1] = 0;
	CL[0] = CL[1] = 0;

	if (NULL != weights)	{
		for (k = 0; k < n; k++)
		{
			a[0] = ptsSrc[k].x - m_pcSrc.x;	a[1] = ptsSrc[k].y - m_pcSrc.y;
			w = weights[k];
			for (i = 0; i < 2; i++)
			{
				for (j = 0; j < 2; j++)
					AA(i, j) += w*a[i] * a[j];//BTPB

				AL[i] += w*a[i] * (ptsDst[k].x - m_pcDst.x);//BTPL
				BL[i] += w*a[i] * (ptsDst[k].y - m_pcDst.y);
			}
		}
	}
	else	{
		for (k = 0; k < n; k++)
		{
			a[0] = ptsSrc[k].x - m_pcSrc.x;	a[1] = ptsSrc[k].y - m_pcSrc.y;

			for (i = 0; i < 2; i++)
			{
				for (j = 0; j < 2; j++)
					AA(i, j) += a[i] * a[j];//BTPB

				AL[i] += a[i] * (ptsDst[k].x - m_pcDst.x);//BTPL
				BL[i] += a[i] * (ptsDst[k].y - m_pcDst.y);
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////
	Eigen::Matrix<double, 2, 2> AI = AA.inverse();
	Eigen::Vector2d X;

	X = AI*AL;
	memcpy(m_a, &X[0], sizeof(double) * 2);
//	m_a[0] = X[0]; m_a[1] = X[1];

	X = AI*BL;
	m_b[0] = X[0]; m_b[1] = X[1];
// 	orsMatrixD AI = AA;
// 	orsVectorD X(2);
// 	getMatrixService()->MatrixInverse(AI);
// 	getMatrixService()->MatrixMultiplyVector(AI, AL, X); X.CopyData(m_a);
// 	getMatrixService()->MatrixMultiplyVector(AI, BL, X); X.CopyData(m_b);

	m_mx = m_my = 0;
	double vx, vy, dx, dy;
	for (k = 0; k < n; k++)
	{
		dx = (ptsSrc[k].x - m_pcSrc.x);	dy = (ptsSrc[k].y - m_pcSrc.y);

		vx = (ptsDst[k].x - m_pcDst.x) - (m_a[0] * dx + m_a[1] * dy);
		vy = (ptsDst[k].y - m_pcDst.y) - (m_b[0] * dx + m_b[1] * dy);

		m_mx += vx*vx;
		m_my += vy*vy;
	}

	if (n > 3)	{
		m_mx = sqrt(m_mx / (n - 3));
		m_my = sqrt(m_my / (n - 3));
	}
	else	{
		m_mx = m_my = 0;
	}

}

void XAffineTransform2D::GetResidual(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys)
{
	int k;
	double dx, dy;

	for (k = 0; k < n; k++)
	{
		dx = (ptsSrc[k].x - m_pcSrc.x);	dy = (ptsSrc[k].y - m_pcSrc.y);

		pVxys->x = (ptsDst[k].x - m_pcDst.x) - (m_a[0] * dx + m_a[1] * dy);
		pVxys->y = (ptsDst[k].y - m_pcDst.y) - (m_b[0] * dx + m_b[1] * dy);

		pVxys++;
	}
}

void XAffineTransform2D::Transform(const iplPOINT2D *ptsSrc, int n, iplPOINT2D *ptsDst)
{
	int k;
	double dx, dy;

	for (k = 0; k < n; k++)
	{
		dx = (ptsSrc->x - m_pcSrc.x);	dy = (ptsSrc->y - m_pcSrc.y);

		ptsDst->x = m_pcDst.x + (m_a[0] * dx + m_a[1] * dy);
		ptsDst->y = m_pcDst.y + (m_b[0] * dx + m_b[1] * dy);

		ptsSrc++;	ptsDst++;
	}
}

void XAffineTransform2D::Transform(double *x, double *y)
{
	double dx, dy;

	dx = *x - m_pcSrc.x;	dy = *y - m_pcSrc.y;

	*x = m_pcDst.x + (m_a[0] * dx + m_a[1] * dy);
	*y = m_pcDst.y + (m_b[0] * dx + m_b[1] * dy);
}

void XAffineTransform2D::GetMeanError(double *mx, double *my)
{
	*mx = m_mx;
	*my = m_my;
}



//////////////////////////////////////////////////////////////////////////
// xDst - xcDst = a0*( xSrc-xcSrc) + a1*(ySrc-ycSrc)
// yDst - ycDst = b0*( xSrc-xcSrc) + b1*(ySrc-ycSrc)
//
//////////////////////////////////////////////////////////////////////////
void XAffineTransform2D::GetParameter(iplPOINT2D *pcSrc, iplPOINT2D *pcDst, double *a, double *b)
{
	*pcSrc = m_pcSrc;
	*pcDst = m_pcDst;

	for (int i = 0; i < 2; i++)
	{
		a[i] = m_a[i];
		b[i] = m_b[i];
	}
};

//
// xDst = xcDst + a0*( xSrc-xcSrc) + a1*(ySrc-ycSrc)
//		= xcDst - a0*xcSrc - a1*ycSrc + a0*xSrc + a1*ySrc;
//
// yDst = ycDst + b0*( xSrc-xcSrc) + b1*(ySrc-ycSrc)
//		= ycDst - b0*xcSrc - b1*ycSrc + a0*xSrc + a1*ySrc;
//
void XAffineTransform2D::GetParameter(double *a, double *b)
{
	a[1] = m_a[0];	a[2] = m_a[1];
	b[1] = m_b[0];	b[2] = m_b[1];

	a[0] = m_pcDst.x - a[1] * m_pcSrc.x - a[2] * m_pcSrc.y;
	b[0] = m_pcDst.y - b[1] * m_pcSrc.x - b[2] * m_pcSrc.y;
};



//////////////////////////////////////////////////////////////////////////
// xDst - xcDst =  a0*( xSrc-xcSrc) + a1*(ySrc-ycSrc)
// yDst - ycDst = -a1*( xSrc-xcSrc) + a0*(ySrc-ycSrc)
//
//////////////////////////////////////////////////////////////////////////

void XSimilarityTransform::Initialize(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights)
{
	m_pcSrc = Average(ptsSrc, n);
	m_pcDst = Average(ptsDst, n);

	//////////////////////////////////////////////////////////////////////////

	int i, j, k;
	double a[2], b[2], dx, dy, lx, ly;

	Eigen::Matrix2d AA;
	Eigen::Vector2d AL;

// 	orsMatrixD	AA(2, 2);
// 	orsVectorD  AL(2);

	AA= Eigen::Matrix2d::Zero();	AL= Eigen::Vector2d::Zero();
	for (k = 0; k < n; k++)
	{
		dx = (ptsSrc[k].x - m_pcSrc.x);	dy = (ptsSrc[k].y - m_pcSrc.y);

		a[0] = dx;	a[1] = dy;	lx = ptsDst[k].x - m_pcDst.x;
		b[0] = dy;	b[1] = -dx;	ly = ptsDst[k].y - m_pcDst.y;

		for (i = 0; i < 2; i++)
		{
			for (j = 0; j < 2; j++)
				AA(i, j) += a[i] * a[j] + b[i] * b[j];

			AL[i] += a[i] * lx + b[i] * ly;
		}
	}

	Eigen::Matrix2d AI = AA.inverse();
// 	orsMatrixD AI = AA;
// 	getMatrixService()->MatrixInverse(AI);

	Eigen::Vector2d X;
	X = AI*AL;
	memcpy(m_a, &X[0], sizeof(double) * 2);
//	m_a[0] = X[0];	m_a[1] = X[1];
// 	orsVectorD X(2);
// 	getMatrixService()->MatrixMultiplyVector(AI, AL, X);
// 	X.CopyData(m_a);

	m_mx = m_my = 0;
	double vx, vy;
	for (k = 0; k < n; k++)
	{
		dx = (ptsSrc[k].x - m_pcSrc.x);	dy = (ptsSrc[k].y - m_pcSrc.y);

		vx = (ptsDst[k].x - m_pcDst.x) - (m_a[0] * dx + m_a[1] * dy);
		vy = (ptsDst[k].y - m_pcDst.y) - (-m_a[1] * dx + m_a[0] * dy);

		m_mx += vx*vx;
		m_my += vy*vy;
	}

	if (n > 2)	{
		m_mx = sqrt(m_mx / (n - 2));
		m_my = sqrt(m_my / (n - 2));
	}
	else	{
		m_mx = m_my = 0;
	}
}

void XSimilarityTransform::GetMeanError(double *mx, double *my)
{
	*mx = m_mx;
	*my = m_my;
}


// 参数归化成仿射变换形式
void XSimilarityTransform::GetParameter(iplPOINT2D *pcSrc, iplPOINT2D *pcDst, double *a, double *b)
{
	*pcSrc = m_pcSrc;
	*pcDst = m_pcDst;

	a[0] = m_a[0];	a[1] = m_a[1];
	b[0] = -m_a[1];	b[1] = m_a[0];
};


// 参数归化成仿射变换形式
void XSimilarityTransform::GetParameter(double *a, double *b)
{
	a[1] = m_a[0];	a[2] = m_a[1];
	b[1] = -m_a[1];	b[2] = m_a[0];

	a[0] = m_pcDst.x - (a[1] * m_pcSrc.x + a[2] * m_pcSrc.y);
	b[0] = m_pcDst.y - (b[1] * m_pcSrc.x + b[2] * m_pcSrc.y);
};



void XSimilarityTransform::Transform(const iplPOINT2D *ptsSrc, int n, iplPOINT2D *ptsDst)
{
	int k;
	double dx, dy;

	for (k = 0; k < n; k++)
	{
		dx = (ptsSrc->x - m_pcSrc.x);	dy = (ptsSrc->y - m_pcSrc.y);

		ptsDst->x = m_pcDst.x + (m_a[0] * dx + m_a[1] * dy);
		ptsDst->y = m_pcDst.y + (-m_a[1] * dx + m_a[0] * dy);

		ptsSrc++;	ptsDst++;
	}
}

void XSimilarityTransform::Transform(double *x, double *y)
{
	double dx, dy;

	dx = *x - m_pcSrc.x;	dy = *y - m_pcSrc.y;

	*x = m_pcDst.x + (m_a[0] * dx + m_a[1] * dy);
	*y = m_pcDst.y + (-m_a[1] * dx + m_a[0] * dy);
}


void XSimilarityTransform::GetResidual(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys)
{
	int k;
	double dx, dy;

	for (k = 0; k < n; k++)
	{
		dx = (ptsSrc[k].x - m_pcSrc.x);	dy = (ptsSrc[k].y - m_pcSrc.y);

		pVxys->x = (ptsDst[k].x - m_pcDst.x) - (m_a[0] * dx + m_a[1] * dy);
		pVxys->y = (ptsDst[k].y - m_pcDst.y) - (-m_a[1] * dx + m_a[0] * dy);

		pVxys++;
	}
}


//////////////////////////////////////////////////////////////////////////
// xDst - xcDst = a0 + a1*( xSrc-xcSrc) + a2*(ySrc-ycSrc) + a3*( xSrc-xcSrc)*(ySrc-ycSrc)
// yDst - ycDst = b0 + b1*( xSrc-xcSrc) + b2*(ySrc-ycSrc) + b3*( xSrc-xcSrc)*(ySrc-ycSrc)
//
//////////////////////////////////////////////////////////////////////////

void XBilinearTransform::Initialize(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights)
{
	m_pcSrc = Average(ptsSrc, n);
	m_pcDst = Average(ptsDst, n);

	////////////////////////////////////
	int i, j, k;
	double a[4];//, b[4];

	Eigen::Matrix4d AA;
	Eigen::Vector4d AL, BL;

// 	orsMatrixD AA(4, 4);
// 	orsVectorD AL(4), BL(4);

	AA = Eigen::Matrix4d::Zero();
	AL = Eigen::Vector4d::Zero();
	BL = Eigen::Vector4d::Zero();

	for (k = 0; k < n; k++)
	{
		a[0] = 1;	a[1] = ptsSrc[k].x - m_pcSrc.x;	a[2] = ptsSrc[k].y - m_pcSrc.y;	a[3] = a[1] * a[2];

		for (i = 0; i < 4; i++)
		{
			for (j = 0; j < 4; j++)
				AA(i, j) += a[i] * a[j];

			AL[i] += a[i] * (ptsDst[k].x - m_pcDst.x);
			BL[i] += a[i] * (ptsDst[k].y - m_pcDst.y);
		}
	}

	//////////////////////////////////////////////////////////////////////////
	Eigen::Matrix4d AI = AA.inverse();

// 	orsMatrixD AI = AA;
// 	getMatrixService()->MatrixInverse(AI);

	Eigen::Vector4d X;
	X = AI*AL;
	memcpy(m_a, &X[0], sizeof(double) * 4);
//	m_a[0] = X[0]; m_a[1] = X[1]; m_a[2] = X[2]; m_a[3] = X[3];

	X = AI*BL;
	memcpy(m_b, &X[0], sizeof(double) * 4);
//	m_b[0] = X[0]; m_b[1] = X[1]; m_b[2] = X[2]; m_b[3] = X[3];

// 	orsVectorD	X(4);
// 	getMatrixService()->MatrixMultiplyVector(AI, AL, X); X.CopyData(m_a);
// 	getMatrixService()->MatrixMultiplyVector(AI, BL, X); X.CopyData(m_b);

	//////////////////////////////////////////////////////////////////////////

	m_mx = m_my = 0;
	double vx, vy, dx, dy;
	for (k = 0; k < n; k++)
	{
		dx = (ptsSrc[k].x - m_pcSrc.x);
		dy = (ptsSrc[k].y - m_pcSrc.y);

		vx = (ptsDst[k].x - m_pcDst.x) - (m_a[0] + m_a[1] * dx + m_a[2] * dy + m_a[3] * dx*dy);
		vy = (ptsDst[k].y - m_pcDst.y) - (m_b[0] + m_b[1] * dx + m_b[2] * dy + m_b[3] * dx*dy);

		m_mx += vx*vx;
		m_my += vy*vy;
	}

	if (n > 4)	{
		m_mx = sqrt(m_mx / (n - 4));
		m_my = sqrt(m_my / (n - 4));
	}
	else
		m_mx = m_my = 0;

}


void XBilinearTransform::GetResidual(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys)
{
	int k;
	double dx, dy;

	for (k = 0; k < n; k++)
	{
		dx = (ptsSrc[k].x - m_pcSrc.x);	dy = (ptsSrc[k].y - m_pcSrc.y);

		pVxys->x = (ptsDst[k].x - m_pcDst.x) - (m_a[0] + m_a[1] * dx + m_a[2] * dy + m_a[3] * dx*dy);
		pVxys->y = (ptsDst[k].y - m_pcDst.y) - (m_b[0] + m_b[1] * dx + m_b[2] * dy + m_b[3] * dx*dy);

		pVxys++;
	}
}

void XBilinearTransform::Transform(const iplPOINT2D *ptsSrc, int n, iplPOINT2D *ptsDst)
{
	int k;
	double dx, dy;

	for (k = 0; k < n; k++)
	{
		dx = (ptsSrc->x - m_pcSrc.x);	dy = (ptsSrc->y - m_pcSrc.y);

		ptsDst->x = m_pcDst.x + (m_a[0] + m_a[1] * dx + m_a[2] * dy + m_a[3] * dx*dy);
		ptsDst->y = m_pcDst.y + (m_b[0] + m_b[1] * dx + m_b[2] * dy + m_b[3] * dx*dy);

		ptsSrc++;	ptsDst++;
	}
}


void XBilinearTransform::Transform(double *x, double *y)
{
	double dx, dy;

	dx = *x - m_pcSrc.x;	dy = *y - m_pcSrc.y;

	*x = m_pcDst.x + (m_a[0] + m_a[1] * dx + m_a[2] * dy + m_a[3] * dx*dy);
	*y = m_pcDst.y + (m_b[0] + m_b[1] * dx + m_b[2] * dy + m_b[3] * dx*dy);
}


void XBilinearTransform::GetMeanError(double *mx, double *my)
{
	*mx = m_mx;
	*my = m_my;
}



// 取变换参数
void XBilinearTransform::GetParameter(iplPOINT2D *pcSrc, iplPOINT2D *pcDst, double *a, double *b)
{
	*pcSrc = m_pcSrc;
	*pcDst = m_pcDst;

	int i;
	for (i = 0; i < 4; i++)
		a[i] = m_a[i];

	for (i = 0; i < 4; i++)
		b[i] = m_b[i];
}

// 取变换参数
void XBilinearTransform::GetParameter(double *a, double *b)
{
	int i;

	for (i = 0; i < 4; i++)
		a[i] = m_a[i];

	for (i = 0; i < 4; i++)
		b[i] = m_b[i];

	// xDst - xcDst = a0 + a1*( xSrc-xcSrc) + a2*(ySrc-ycSrc) + a3*( xSrc-xcSrc)*(ySrc-ycSrc)
	//              = a0 + a1*xSrc - a1*xcSrc + a2*ySrc - a2*ycSrc + a3*xSrc*ySrc 
	//					+  a3*xSrc*ySrc - a3*xSrc*ycSrc - a3*xcSrc*ySrc + a3*xcSrc*ycSrc;

	a[0] += m_pcDst.x - a[1] * m_pcSrc.x - a[2] * m_pcSrc.y + a[3] * m_pcSrc.x*m_pcSrc.y;
	a[1] -= a[3] * m_pcSrc.y;
	a[2] -= a[3] * m_pcSrc.x;

	b[0] += m_pcDst.y - b[1] * m_pcSrc.x - b[2] * m_pcSrc.y + b[3] * m_pcSrc.x*m_pcSrc.y;
	b[1] -= b[3] * m_pcSrc.y;
	b[2] -= b[3] * m_pcSrc.x;
}


//////////////////////////////////////////////////////////////////////////
//				   l0*( xSrc-xcSrc) + l1*(ySrc-ycSrc) + l2
// xDst - xcDst = ------------------------------------------
//				   l6*( xSrc-xcSrc) + l7*(ySrc-ycSrc) + 1
//
//				   l3*( xSrc-xcSrc) + l4*(ySrc-ycSrc) + l5
// yDst - ycDst = ------------------------------------------
//				   l6*( xSrc-xcSrc) + l7*(ySrc-ycSrc) + 1
//
//				   
// dxDst*(l6*dxSrc + l7*dySrc + 1) = l0*dxSrc + l1*dySrc + l2
//
// dyDst*(l6*dxSrc + l7*dySrc + 1) = l3*dxSrc + l4*dySrc + l5
//
//////////////////////////////////////////////////////////////////////////
XDLTTransform_2D::XDLTTransform_2D()
{
	m_l[8] = 1;
}



void XDLTTransform_2D::Initialize(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights)
{
	m_pcSrc = Average(ptsSrc, n);
	m_pcDst = Average(ptsDst, n);

	////////////////////////////////////
	int i, j, k;
	double a[8], b[8], dxSrc, dySrc, dxDst, dyDst, lx, ly, w;

	Eigen::Matrix<double, 8, 8> AA;
	Eigen::VectorXd AL(8);

// 	orsMatrixD	AA(8, 8);
// 	orsVectorD	AL(8);

	AA = Eigen::MatrixXd::Zero(8,8);
	AL = Eigen::VectorXd::Zero(8);

	memset(a, 0, 8 * sizeof(double));
	memset(b, 0, 8 * sizeof(double));

	if (NULL != weights)	{
		for (k = 0; k < n; k++)
		{
			dxSrc = ptsSrc[k].x - m_pcSrc.x;		dySrc = ptsSrc[k].y - m_pcSrc.y;
			dxDst = ptsDst[k].x - m_pcDst.x;		dyDst = ptsDst[k].y - m_pcDst.y;

			a[0] = dxSrc;	a[1] = dySrc;	a[2] = 1;
			a[6] = -dxDst*dxSrc;	a[7] = -dxDst*dySrc;		lx = dxDst;

			b[3] = dxSrc;	b[4] = dySrc;	b[5] = 1;
			b[6] = -dyDst*dxSrc;	b[7] = -dyDst*dySrc;		ly = dyDst;

			w = weights[k];

			for (i = 0; i < 8; i++)
			{
				for (j = 0; j < 8; j++)
					AA(i, j) += w*(a[i] * a[j] + b[i] * b[j]);

				AL[i] += w*(a[i] * lx + b[i] * ly);
			}
		}
	}
	else	{
		for (k = 0; k < n; k++)
		{
			dxSrc = ptsSrc[k].x - m_pcSrc.x;		dySrc = ptsSrc[k].y - m_pcSrc.y;
			dxDst = ptsDst[k].x - m_pcDst.x;		dyDst = ptsDst[k].y - m_pcDst.y;

			a[0] = dxSrc;	a[1] = dySrc;	a[2] = 1;
			a[6] = -dxDst*dxSrc;	a[7] = -dxDst*dySrc;		lx = dxDst;

			b[3] = dxSrc;	b[4] = dySrc;	b[5] = 1;
			b[6] = -dyDst*dxSrc;	b[7] = -dyDst*dySrc;		ly = dyDst;

			for (i = 0; i < 8; i++)
			{
				for (j = 0; j < 8; j++)
					AA(i, j) += a[i] * a[j] + b[i] * b[j];

				AL[i] += a[i] * lx + b[i] * ly;
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////
	Eigen::VectorXd X(8);
//	AA.jacobiSvd().solve(AL);
    X = AA.colPivHouseholderQr().solve(AL);

	memcpy(m_l, &X[0], sizeof(double) * 8);
	//getMatrixService()->SolveLinearEqs_Gauss(AA, AL);	AL.CopyData(m_l);


	//////////////////////////////////////////////////////////////////////////

	m_mx = m_my = 0;
	double vx, vy, s;
	for (k = 0; k < n; k++)
	{
		dxSrc = ptsSrc[k].x - m_pcSrc.x;
		dySrc = ptsSrc[k].y - m_pcSrc.y;

		s = m_l[6] * dxSrc + m_l[7] * dySrc + 1;

		vx = (ptsDst[k].x - m_pcDst.x) - (m_l[0] * dxSrc + m_l[1] * dySrc + m_l[2]) / s;
		vy = (ptsDst[k].y - m_pcDst.y) - (m_l[3] * dxSrc + m_l[4] * dySrc + m_l[5]) / s;

		m_mx += vx*vx;
		m_my += vy*vy;
	}

	if (n > 4)	{
		m_mx = sqrt(m_mx / (n - 4));
		m_my = sqrt(m_my / (n - 4));
	}
	else
		m_mx = m_my = 0;
}



void XDLTTransform_2D::GetResidual(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys)
{
	int k;
	double dx, dy, w;

	for (k = 0; k < n; k++)
	{
		dx = ptsSrc[k].x - m_pcSrc.x;
		dy = ptsSrc[k].y - m_pcSrc.y;

		w = m_l[6] * dx + m_l[7] * dy + 1;

		pVxys->x = (ptsDst[k].x - m_pcDst.x) - (m_l[0] * dx + m_l[1] * dy + m_l[2]) / w;
		pVxys->y = (ptsDst[k].y - m_pcDst.y) - (m_l[3] * dx + m_l[4] * dy + m_l[5]) / w;

		pVxys++;
	}
}



void XDLTTransform_2D::Transform(const iplPOINT2D *ptsSrc, int n, iplPOINT2D *ptsDst)
{
	int k;
	double dx, dy, w;

	for (k = 0; k < n; k++)
	{
		dx = (ptsSrc->x - m_pcSrc.x);	dy = (ptsSrc->y - m_pcSrc.y);

		w = m_l[6] * dx + m_l[7] * dy + 1;

		ptsDst->x = m_pcDst.x + (m_l[0] * dx + m_l[1] * dy + m_l[2]) / w;
		ptsDst->y = m_pcDst.y + (m_l[3] * dx + m_l[4] * dy + m_l[5]) / w;

		ptsSrc++;	ptsDst++;
	}
}

void XDLTTransform_2D::Transform(double *x, double *y)
{
	double dx, dy;

	dx = *x - m_pcSrc.x;	dy = *y - m_pcSrc.y;

	double w = m_l[6] * dx + m_l[7] * dy + 1;

	*x = m_pcDst.x + (m_l[0] * dx + m_l[1] * dy + m_l[2]) / w;
	*y = m_pcDst.y + (m_l[3] * dx + m_l[4] * dy + m_l[5]) / w;
}


void XDLTTransform_2D::GetMeanError(double *mx, double *my)
{
	*mx = m_mx;
	*my = m_my;
}



void XDLTTransform_2D::GetParameter(iplPOINT2D *pcSrc, iplPOINT2D *pcDst, double *l, double *b)
{
	*pcSrc = m_pcSrc;
	*pcDst = m_pcDst;

	for (int i = 0; i < 9; i++)
		l[i] = m_l[i];
}



// 取变换参数，中心点计入参数
void XDLTTransform_2D::GetParameter(double *l, double *b)
{
	int i;


	//////////////////////////////////////////////////////////////////////////
	//				   l0*( xSrc-xcSrc) + l1*(ySrc-ycSrc) + l2
	// xDst - xcDst = ------------------------------------------
	//				   l6*( xSrc-xcSrc) + l7*(ySrc-ycSrc) + 1
	// ==>
	//		           l0*xSrc + l1*ySrc + (l2 - l0*xcSrc - l1*ycSrc)
	// xDst = xcDst + -------------------------------------------------
	//		           l6*xSrc + l7*ySrc + (1  - l6*xcSrc - l7*ycSrc)=l8
	//
	// ==>
	//
	//		   ( l0 + l6*xcDst) * *xSrc + ( l1 + l7*xcDst)* ySrc + (l2 - l0*xcSrc - l1*ycSrc)+ xcDst*l8 
	// xDst = -----------------------------------------------------------------------------------------------------------------
	//		   l6*xSrc + l7*ySrc + (1  - l6*xcSrc - l7*ycSrc)

	for (i = 0; i < 8; i++)
		l[i] = m_l[i];

	double l8 = 1 - l[6] * m_pcSrc.x - l[7] * m_pcSrc.y;

	// 不能置后， 否则l[0],l[1]已被修改
	l[2] += l8*m_pcDst.x - l[0] * m_pcSrc.x - l[1] * m_pcSrc.y;
	l[0] += l[6] * m_pcDst.x;
	l[1] += l[7] * m_pcDst.x;

	l[5] += l8*m_pcDst.y - l[3] * m_pcSrc.x - l[4] * m_pcSrc.y;
	l[3] += l[6] * m_pcDst.y;
	l[4] += l[7] * m_pcDst.y;

	for (i = 0; i < 8; i++)
		l[i] /= l8;

	l[8] = 1;
}

//////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////
// xDst - xcDst = a0 + a1*( xSrc-xcSrc) + a2*(ySrc-ycSrc) + a3*xx + a5*xy + a6*yy
// yDst - ycDst = b0 + b1*( xSrc-xcSrc) + b2*(ySrc-ycSrc) + b3*xx + b5*xy + b6*yy
//
//////////////////////////////////////////////////////////////////////////

void XPolynomialTransform2::Initialize(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights)
{
	m_pcSrc = Average(ptsSrc, n);
	m_pcDst = Average(ptsDst, n);

	////////////////////////////////////
	int i, j, k;
	double a[6], w;

	Eigen::MatrixXd AA(6, 6);
	Eigen::VectorXd AL(6), BL(6);

// 	orsMatrixD AA(6, 6);
// 	orsVectorD AL(6), BL(6);

	AA = Eigen::MatrixXd::Zero(6, 6);	
	AL = Eigen::VectorXd::Zero(6);	
	BL = Eigen::VectorXd::Zero(6);

	if (NULL != weights)	{
		for (k = 0; k < n; k++)
		{
			a[0] = 1;			a[1] = ptsSrc[k].x - m_pcSrc.x;	a[2] = ptsSrc[k].y - m_pcSrc.y;
			a[3] = a[1] * a[1];	a[4] = a[1] * a[2];				a[5] = a[2] * a[2];

			w = weights[k];

			for (i = 0; i < 6; i++)
			{
				for (j = 0; j < 6; j++)
					AA(i, j) += w*a[i] * a[j];

				AL[i] += w*a[i] * (ptsDst[k].x - m_pcDst.x);
				BL[i] += w*a[i] * (ptsDst[k].y - m_pcDst.y);
			}
		}
	}
	else	{
		for (k = 0; k < n; k++)
		{
			a[0] = 1;			a[1] = ptsSrc[k].x - m_pcSrc.x;	a[2] = ptsSrc[k].y - m_pcSrc.y;
			a[3] = a[1] * a[1];	a[4] = a[1] * a[2];				a[5] = a[2] * a[2];

			for (i = 0; i < 6; i++)
			{
				for (j = 0; j < 6; j++)
					AA(i, j) += a[i] * a[j];

				AL[i] += a[i] * (ptsDst[k].x - m_pcDst.x);
				BL[i] += a[i] * (ptsDst[k].y - m_pcDst.y);
			}
		}
	}


	//////////////////////////////////////////////////////////////////////////
	Eigen::MatrixXd AI = AA.inverse();

// 	orsMatrixD AI = AA;
// 	getMatrixService()->MatrixInverse(AI);

	Eigen::VectorXd X(6);
	X = AI*AL;
	memcpy(m_a, &X[0], sizeof(double) * 6);

	X = AI*BL;
	memcpy(m_b, &X[0], sizeof(double) * 6);
// 	orsVectorD	X(6);
// 	getMatrixService()->MatrixMultiplyVector(AI, AL, X); X.CopyData(m_a);
// 	getMatrixService()->MatrixMultiplyVector(AI, BL, X); X.CopyData(m_b);

	//////////////////////////////////////////////////////////////////////////

	m_mx = m_my = 0;
	double vx, vy, dx, dy, dx2, dy2, dxy;
	for (k = 0; k < n; k++)
	{
		dx = (ptsSrc[k].x - m_pcSrc.x);
		dy = (ptsSrc[k].y - m_pcSrc.y);

		dx2 = dx*dx;
		dy2 = dy*dy;
		dxy = dx*dy;

		vx = (ptsDst[k].x - m_pcDst.x) - (m_a[0] + m_a[1] * dx + m_a[2] * dy + m_a[3] * dx2 + m_a[4] * dxy + m_a[5] * dy2);
		vy = (ptsDst[k].y - m_pcDst.y) - (m_b[0] + m_b[1] * dx + m_b[2] * dy + m_b[3] * dx2 + m_b[4] * dxy + m_b[5] * dy2);

		m_mx += vx*vx;
		m_my += vy*vy;
	}

	if (n > 6)	{
		m_mx = sqrt(m_mx / (n - 6));
		m_my = sqrt(m_my / (n - 6));
	}
	else	{
		m_mx = m_my = 0;
	}

}


void XPolynomialTransform2::GetResidual(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys)
{
	int k;
	double dx, dy, dx2, dy2, dxy;
	/*
		for( k=0; k<n ; k++)
		{
		dx = (ptsSrc[k].x - m_pcSrc.x);	dy = (ptsSrc[k].y - m_pcSrc.y);

		pVxys->x = (ptsDst[k].x-m_pcDst.x) - ( m_a[0] + m_a[1]*dx + m_a[2]*dy + m_a[3]*dx*dx + m_a[4]*dx*dy + m_a[5]*dy*dy );
		pVxys->y = (ptsDst[k].y-m_pcDst.y) - ( m_b[0] + m_b[1]*dx + m_b[2]*dy + m_b[3]*dx*dx + m_b[4]*dx*dy + m_b[5]*dy*dy);

		pVxys++;
		}
		*/
	double a[6], b[6];

	GetParameter(a, b);

	for (k = 0; k < n; k++)
	{
		dx = ptsSrc[k].x;	dy = ptsSrc[k].y;
		dx2 = dx*dx;		dy2 = dy*dy;		dxy = dx*dy;

		pVxys->x = ptsDst[k].x - (a[0] + a[1] * dx + a[2] * dy + a[3] * dx2 + a[4] * dxy + a[5] * dy2);
		pVxys->y = ptsDst[k].y - (b[0] + b[1] * dx + b[2] * dy + b[3] * dx2 + b[4] * dxy + b[5] * dy2);

		pVxys++;
	}

}

void XPolynomialTransform2::Transform(const iplPOINT2D *ptsSrc, int n, iplPOINT2D *ptsDst)
{
	int k;
	double dx, dy, dx2, dy2, dxy;

	for (k = 0; k < n; k++)
	{
		dx = (ptsSrc->x - m_pcSrc.x);	dy = (ptsSrc->y - m_pcSrc.y);
		dx2 = dx*dx;			dy2 = dy*dy;			dxy = dx*dy;

		ptsDst->x = m_pcDst.x + (m_a[0] + m_a[1] * dx + m_a[2] * dy + m_a[3] * dx2 + m_a[4] * dxy + m_a[5] * dy2);
		ptsDst->y = m_pcDst.y + (m_b[0] + m_b[1] * dx + m_b[2] * dy + m_b[3] * dx2 + m_b[4] * dxy + m_b[5] * dy2);

		ptsSrc++;	ptsDst++;
	}
}

void XPolynomialTransform2::Transform(double *x, double *y)
{
	double dx, dy, dx2, dy2, dxy;

	dx = *x - m_pcSrc.x;	dy = *y - m_pcSrc.y;
	dx2 = dx*dx;			dy2 = dy*dy;			dxy = dx*dy;

	*x = m_pcDst.x + (m_a[0] + m_a[1] * dx + m_a[2] * dy + m_a[3] * dx2 + m_a[4] * dxy + m_a[5] * dy2);
	*y = m_pcDst.y + (m_b[0] + m_b[1] * dx + m_b[2] * dy + m_b[3] * dx2 + m_b[4] * dxy + m_b[5] * dy2);
}

void XPolynomialTransform2::GetMeanError(double *mx, double *my)
{
	*mx = m_mx;
	*my = m_my;
}



// 取变换参数
void XPolynomialTransform2::GetParameter(iplPOINT2D *pcSrc, iplPOINT2D *pcDst, double *a, double *b)
{
	*pcSrc = m_pcSrc;
	*pcDst = m_pcDst;

	int i;
	for (i = 0; i < 6; i++)
		a[i] = m_a[i];

	for (i = 0; i < 6; i++)
		b[i] = m_b[i];
}

// 取变换参数
void XPolynomialTransform2::GetParameter(double *a, double *b)
{
	int i;

	for (i = 0; i < 6; i++)
		a[i] = m_a[i];

	for (i = 0; i < 6; i++)
		b[i] = m_b[i];

	// xDst - xcDst = a0 + a1*( xSrc-xcSrc) + a2*(ySrc-ycSrc) + a3*( xSrc-xcSrc)*(xSrc-xcSrc) + a4*( xSrc-xcSrc)*(ySrc-ycSrc) + a5*( ySrc-ycSrc)*(ySrc-ycSrc)
	//              = a0 - a1*xcSrc - a2*ycSrc + a3*xcSrc*xcSrc + a4*xcScrc*ycSrc + a5*ycSrc*ycSrc 
	//				 + ( a1 - 2*a3*xcSrc -  a4*ycSrc)*xSrc
	//				 + ( a2              - *a4*xcSrc -2*a5*ycSrc)*ySrc
	//				 + a3*xSrc*xSrc
	//				 + a4*xSrc*ySrc
	//				 + a5*ySrc*ySrc;

	a[0] += m_pcDst.x - a[1] * m_pcSrc.x - a[2] * m_pcSrc.y + a[3] * m_pcSrc.x*m_pcSrc.x
		+ a[4] * m_pcSrc.x*m_pcSrc.y
		+ a[5] * m_pcSrc.y*m_pcSrc.y;
	a[1] -= 2 * a[3] * m_pcSrc.x + a[4] * m_pcSrc.y;
	a[2] -= a[4] * m_pcSrc.x + 2 * a[5] * m_pcSrc.y;

	b[0] += m_pcDst.y - b[1] * m_pcSrc.x - b[2] * m_pcSrc.y + b[3] * m_pcSrc.x*m_pcSrc.x
		+ b[4] * m_pcSrc.x*m_pcSrc.y
		+ b[5] * m_pcSrc.y*m_pcSrc.y;
	b[1] -= 2 * b[3] * m_pcSrc.x + b[4] * m_pcSrc.y;
	b[2] -= b[4] * m_pcSrc.x + 2 * b[5] * m_pcSrc.y;
}

#include <vector>
using namespace std;
void XPolynomialTransformTPSpline::Initialize(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights)
{
	m_nCntrPts = n;

	m_ptsSrc = new iplPOINT2D[n];
	memcpy(m_ptsSrc, ptsSrc, n*sizeof(iplPOINT2D));

	int nCoefficents;

	nCoefficents = m_nCntrPts + 3;

	m_a = new double[nCoefficents];
	m_b = new double[nCoefficents];

	////////////////////////////////////
	int i, k;
	// 	FILE *fp;
	// 	fp = fopen("E:\\point.txt","wt");
	// 	for ( i =0;i<m_nCntrPts;i++)
	// 	{
	// 		fprintf(fp,"%lf %lf \n",ptsSrc[i].x,ptsSrc[i].y);
	// 	}
	// 	fprintf(fp,"\n\n");
	// 	for ( i =0;i<m_nCntrPts;i++)
	// 	{
	// 		fprintf(fp,"%lf %lf \n",ptsDst[i].x,ptsDst[i].y);
	// 	}
	// 	fclose(fp);

	Eigen::MatrixXd a(nCoefficents, nCoefficents);
	Eigen::VectorXd lx(nCoefficents), ly(nCoefficents);
// 	orsMatrixD a(nCoefficents, nCoefficents);
// 	orsVectorD lx(nCoefficents);
// 	orsVectorD ly(nCoefficents);

	double *suma = new double[nCoefficents];
	double sumlx = 0;
	double sumly = 0;
	memset(suma, 0, sizeof(double)*nCoefficents);

	Eigen::VectorXd AL(nCoefficents), BL(nCoefficents);
//	orsVectorD AL(nCoefficents), BL(nCoefficents);

	AL = Eigen::VectorXd::Zero(nCoefficents);
	BL = Eigen::VectorXd::Zero(nCoefficents);

	double dx, dy, r;
	for (k = 0; k < nCoefficents; k++)
	{
		if (k < m_nCntrPts)	//前N个系数
		{
			for (i = 0; i < m_nCntrPts; i++)
			{
				if (i != k)
				{
					dx = ptsSrc[k].x - ptsSrc[i].x;
					dy = ptsSrc[k].y - ptsSrc[i].y;

					r = dx*dx + dy*dy;

					a(k, i) = r * log(r);
				}
				else
					a(k, i) = 0;

				suma[i] += a(k, i);
			}

			a(k, m_nCntrPts) = 1; a(k, m_nCntrPts + 1) = ptsSrc[k].x; a(k, m_nCntrPts + 2) = ptsSrc[k].y;
			suma[m_nCntrPts] += a(k, m_nCntrPts); suma[m_nCntrPts + 1] += a(k, m_nCntrPts + 1);
			suma[m_nCntrPts + 2] += a(k, m_nCntrPts + 2);

			lx[k] = ptsDst[k].x;
			ly[k] = ptsDst[k].y;

			sumlx += ptsDst[k].x;
			sumly += ptsDst[k].y;
		}

		if (k == m_nCntrPts)
		{
			for (i = 0; i < m_nCntrPts; i++)
			{
				a(k, i) = 1;
				suma[i] += a(k, i);
			}

			a(k, m_nCntrPts) = 0; a(k, m_nCntrPts + 1) = 0; a(k, m_nCntrPts + 2) = 0;
			suma[m_nCntrPts] += a(k, m_nCntrPts); suma[m_nCntrPts + 1] += a(k, m_nCntrPts + 1);
			suma[m_nCntrPts + 2] += a(k, m_nCntrPts + 2);
			lx[k] = 0;
			ly[k] = 0;
			sumlx += lx[k];
			sumly += ly[k];
		}
		if (k == m_nCntrPts + 1)
		{
			for (i = 0; i < m_nCntrPts; i++)
			{
				a(k, i) = ptsSrc[i].x; suma[i] += a(k, i);
			}
			a(k, m_nCntrPts) = 0; a(k, m_nCntrPts + 1) = 0; a(k, m_nCntrPts + 2) = 0;
			suma[m_nCntrPts] += a(k, m_nCntrPts); suma[m_nCntrPts + 1] += a(k, m_nCntrPts + 1);
			suma[m_nCntrPts + 2] += a(k, m_nCntrPts + 2);
			lx[k] = 0;
			ly[k] = 0;
			sumlx += lx[k];
			sumly += ly[k];
		}
		if (k == m_nCntrPts + 2)
		{
			for (i = 0; i < m_nCntrPts; i++)
			{
				a(k, i) = ptsSrc[i].y; suma[i] += a(k, i);
			}
			a(k, m_nCntrPts) = 0; a(k, m_nCntrPts + 1) = 0; a(k, m_nCntrPts + 2) = 0;
			suma[m_nCntrPts] += a(k, m_nCntrPts); suma[m_nCntrPts + 1] += a(k, m_nCntrPts + 1);
			suma[m_nCntrPts + 2] += a(k, m_nCntrPts + 2);
			lx[k] = 0;
			ly[k] = 0;
			sumlx += lx[k];
			sumly += ly[k];
		}
	}

	for (k = 0; k < nCoefficents; k++)
		suma[k] /= nCoefficents;

	sumlx /= nCoefficents;
	sumly /= nCoefficents;

	double deta;
	Eigen::MatrixXd AI = a;
	deta = AI.determinant();

// 	orsMatrixD AI = a;
// 	getMatrixService()->MatrixDeterminant(AI, deta);
	if (fabs(deta) < 1e-6)
	{
		for (k = 0; k < nCoefficents; k++)
		{
			vector<double> uu;
			for (i = 0; i < nCoefficents; i++)
			{
				a(k, i) -= suma[i];
				uu.push_back(a(k, i));
			}

			lx[k] -= sumlx;
			ly[k] -= sumly;
			vector<double> llx;
			vector<double> lly;
			llx.push_back(lx[k]);
			lly.push_back(ly[k]);
		}
	}

	AI = a.inverse();
//	getMatrixService()->MatrixInverse(AI);

	Eigen::VectorXd	X(nCoefficents);

	X = AI*lx;
	memcpy(m_a, &X[0], sizeof(double)*nCoefficents);
	X = AI*ly;
	memcpy(m_b, &X[0], sizeof(double)*nCoefficents);

// 	getMatrixService()->MatrixMultiplyVector(AI, lx, X); X.CopyData(m_a);
// 	getMatrixService()->MatrixMultiplyVector(AI, ly, X); X.CopyData(m_b);

	//////////////////////////////////////////////////////////////////////////
	m_mx = m_my = 0;
	double vx, vy;
	vx = 0;
	if (deta != 0)
	{
		for (k = 0; k < m_nCntrPts; k++)
		{
			vx = 0; vy = 0;
			for (i = 0; i < nCoefficents; i++)
			{
				vx += m_a[i] * (a(k, i));
				vy += m_b[i] * (a(k, i));
			}
			vx = (lx[k]) - vx;
			vy = (ly[k]) - vy;
			m_mx += vx*vx;
			m_my += vy*vy;
		}
	}
	else
	{
		for (k = 0; k < m_nCntrPts; k++)
		{
			vx = 0; vy = 0;
			for (i = 0; i < nCoefficents; i++)
			{
				vx += m_a[i] * (a(k, i) + suma[i]);
				vy += m_b[i] * (a(k, i) + suma[i]);

			}
			vx = (lx[k] + sumlx) - vx;
			vy = (ly[k] + sumly) - vy;
			m_mx += vx*vx;
			m_my += vy*vy;
		}
	}

	/*if( n > 6 )	{*/
	m_mx = sqrt(m_mx / n);
	m_my = sqrt(m_my / n);
	/*}*/
	//else	{
	//	m_mx = m_my = 0;
	//}
// 	a.DeAlloc();
// 	lx.DeAlloc();
// 	ly.DeAlloc();
	delete[]suma;

}


void XPolynomialTransformTPSpline::GetResidual(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys)
{
	int k, i;

	assert(m_nCntrPts == n);

	double x, y, dx, dy, r;

	for (k = 0; k < m_nCntrPts; k++)
	{
		x = ptsSrc[k].x;
		y = ptsSrc[k].y;

		// 仿射变换
		pVxys->x = m_a[m_nCntrPts];
		pVxys->y = m_b[m_nCntrPts];

		pVxys->x += m_a[m_nCntrPts + 1] * x;
		pVxys->y += m_b[m_nCntrPts + 1] * x;

		pVxys->x += m_a[m_nCntrPts + 2] * y;
		pVxys->y += m_b[m_nCntrPts + 2] * y;

		// 径向基函数加权
		for (i = 0; i < m_nCntrPts; i++)
		{
			if (i != k)
			{
				dx = x - ptsSrc[i].x;
				dy = y - ptsSrc[i].y;

				r = dx*dx + dy*dy;

				if (r > 0)	{
					r = r * log(r);

					pVxys->x += r * m_a[i];
					pVxys->y += r * m_b[i];
				}
			}
		}

		pVxys->x = ptsDst[k].x - pVxys->x;
		pVxys->y = ptsDst[k].y - pVxys->y;

		pVxys++;
	}
}

void XPolynomialTransformTPSpline::Transform(const iplPOINT2D *ptsSrc, int n, iplPOINT2D *ptsDst)
{
	int k, i;

	double x, y, dx, dy, r, x1, y1;

	for (k = 0; k < n; k++)
	{
		x = ptsSrc[k].x;
		y = ptsSrc[k].y;

		// 仿射变换
		x1 = m_a[m_nCntrPts];
		y1 = m_b[m_nCntrPts];

		x1 += m_a[m_nCntrPts + 1] * x;
		y1 += m_b[m_nCntrPts + 1] * x;

		x1 += m_a[m_nCntrPts + 2] * y;
		y1 += m_b[m_nCntrPts + 2] * y;

		// 径向基函数加权
		for (i = 0; i < m_nCntrPts; i++)
		{
			dx = x - m_ptsSrc[i].x;
			dy = y - m_ptsSrc[i].y;

			r = dx*dx + dy*dy;

			if (r > 0)	{
				r = r * log(r);

				x1 += r * m_a[i];
				y1 += r * m_b[i];
			}
		}

		ptsDst[k].x = x1;
		ptsDst[k].y = y1;
	}
}


void XPolynomialTransformTPSpline::Transform(double *x, double *y)
{
	double dx, dy;

	double r, x1, y1;

	int i;

	// 仿射变换
	x1 = m_a[m_nCntrPts];
	y1 = m_b[m_nCntrPts];

	x1 += m_a[m_nCntrPts + 1] * *x;
	y1 += m_b[m_nCntrPts + 1] * *x;

	x1 += m_a[m_nCntrPts + 2] * *y;
	y1 += m_b[m_nCntrPts + 2] * *y;

	// 径向基函数加权
	for (i = 0; i < m_nCntrPts; i++)
	{
		dx = *x - m_ptsSrc[i].x;
		dy = *y - m_ptsSrc[i].y;

		r = dx*dx + dy*dy;

		if (r > 0)	{
			r = r * log(r);

			x1 += r * m_a[i];
			y1 += r * m_b[i];
		}
	}

	*x = x1;
	*y = y1;
}

void XPolynomialTransformTPSpline::GetMeanError(double *mx, double *my)
{
	*mx = m_mx;
	*my = m_my;
}



// 取变换参数
void XPolynomialTransformTPSpline::GetParameter(iplPOINT2D *pcSrc, iplPOINT2D *pcDst, double *a, double *b)
{
	pcSrc->x = pcSrc->y = 0;
	pcDst->x = pcDst->y = 0;

	memcpy(a, m_a, (m_nCntrPts + 3)*sizeof(double));
	memcpy(b, m_b, (m_nCntrPts + 3)*sizeof(double));
}

// 取变换参数
void XPolynomialTransformTPSpline::GetParameter(double *a, double *b)
{
	memcpy(a, m_a, (m_nCntrPts + 3)*sizeof(double));
	memcpy(b, m_b, (m_nCntrPts + 3)*sizeof(double));
}


//////////////////////////////////////////////////////////////////////////
//	dx = xSrc-xcSrc		
//  dy = ySrc-ycSrc
// xDst - xcDst = a0 + a1*dx + a2*dy + a3*dx*dx + a4*dx*dy + a5*dy*dy + a6*dx*dx*dx + a7*dx*dx*dy + a8*dx*dy*dy + a9*dy*dy*dy
// yDst - ycDst = b0 + b1*dx + b2*dy + b3*dx*dx + b5*dx*dy + b5*dy*dy + b6*dx*dx*dx + b7*dx*dx*dy + b8*dx*dy*dy + b9*dy*dy*dy
//////////////////////////////////////////////////////////////////////////

void XPolynomialTransform3::Initialize(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights)
{
	m_pcSrc = Average(ptsSrc, n);
	m_pcDst = Average(ptsDst, n);

	////////////////////////////////////
	int i, j, k;
	double a[10], b[10];

	Eigen::MatrixXd AA(10, 10);
	Eigen::VectorXd AL(10), BL(10);

// 	orsMatrixD AA(10, 10);
// 	orsVectorD AL(10), BL(10);

	AA = Eigen::MatrixXd::Zero(10,10);	
	AL = Eigen::VectorXd::Zero(10);	
	BL = Eigen::VectorXd::Zero(10);

	for (k = 0; k < n; k++)
	{
		double dx, dy;

		dx = ptsSrc[k].x - m_pcSrc.x;
		dy = ptsSrc[k].y - m_pcSrc.y;

		a[0] = 1;			a[1] = dx;			a[2] = dy;
		a[3] = dx*dx;		a[4] = dx*dy;		a[5] = dy*dy;
		a[6] = a[3] * dx;	a[7] = a[3] * dy;	a[8] = dx*a[5];	a[9] = a[5] * dy;

		memcpy(b, a, 10 * sizeof(double));

		double lx, ly;

		lx = (ptsDst[k].x - m_pcDst.x);
		ly = (ptsDst[k].y - m_pcDst.y);

		for (i = 0; i < 10; i++)
		{
			for (j = 0; j < 10; j++)
				AA(i, j) += a[i] * a[j];

			AL[i] += a[i] * lx;
			BL[i] += b[i] * ly;
		}
	}

	//////////////////////////////////////////////////////////////////////////
	Eigen::MatrixXd AI = AA.inverse();
// 	orsMatrixD AI = AA;
// 	getMatrixService()->MatrixInverse(AI);

	Eigen::VectorXd X(10); 
	X = AI*AL;
	memcpy(m_a, &X[0], sizeof(double) * 10);
	X = AI*BL;
	memcpy(m_b, &X[0], sizeof(double) * 10);

//	orsVectorD	X(10);
// 	getMatrixService()->MatrixMultiplyVector(AI, AL, X); X.CopyData(m_a);
// 	getMatrixService()->MatrixMultiplyVector(AI, BL, X); X.CopyData(m_b);

	//////////////////////////////////////////////////////////////////////////

	m_mx = m_my = 0;
	for (k = 0; k < n; k++)
	{
		double vx, vy, dx, dy, dx2, dxy, dy2, dx3, dx2y, dxdy2, dy3;

		dx = (ptsSrc[k].x - m_pcSrc.x);
		dy = (ptsSrc[k].y - m_pcSrc.y);

		dx2 = dx*dx; dxy = dx*dy;	dy2 = dy*dy;
		dx3 = dx2*dx;	dx2y = dx2*dy;	dxdy2 = dx*dy2;	dy3 = dy2*dy;

		vx = (ptsDst[k].x - m_pcDst.x) -
			(m_a[0] + m_a[1] * dx + m_a[2] * dy + m_a[3] * dx2 + m_a[4] * dxy + m_a[5] * dy2 + m_a[6] * dx3 + m_a[7] * dx2y + m_a[8] * dxdy2 + m_a[9] * dy3);
		vy = (ptsDst[k].y - m_pcDst.y) -
			(m_b[0] + m_b[1] * dx + m_b[2] * dy + m_b[3] * dx2 + m_b[4] * dxy + m_b[5] * dy2 + m_b[6] * dx3 + m_b[7] * dx2y + m_b[8] * dxdy2 + m_b[9] * dy3);

		m_mx += vx*vx;
		m_my += vy*vy;
	}

	m_mx = sqrt(m_mx / (n - 10));
	m_my = sqrt(m_my / (n - 10));

}


void XPolynomialTransform3::GetResidual(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys)
{
	int k;

	for (k = 0; k < n; k++)
	{
		double dx, dy, dx2, dxy, dy2, dx3, dx2y, dxdy2, dy3;

		dx = (ptsSrc[k].x - m_pcSrc.x);
		dy = (ptsSrc[k].y - m_pcSrc.y);

		dx2 = dx*dx; dxy = dx*dy;	dy2 = dy*dy;
		dx3 = dx2*dx;	dx2y = dx2*dy;	dxdy2 = dx*dy2;	dy3 = dy2*dy;

		pVxys->x = (ptsDst[k].x - m_pcDst.x) -
			(m_a[0] + m_a[1] * dx + m_a[2] * dy + m_a[3] * dx2 + m_a[4] * dxy + m_a[5] * dy2 + m_a[6] * dx3 + m_a[7] * dx2y + m_a[8] * dxdy2 + m_a[9] * dy3);
		pVxys->y = (ptsDst[k].y - m_pcDst.y) -
			(m_b[0] + m_b[1] * dx + m_b[2] * dy + m_b[3] * dx2 + m_b[4] * dxy + m_b[5] * dy2 + m_b[6] * dx3 + m_b[7] * dx2y + m_b[8] * dxdy2 + m_b[9] * dy3);

		pVxys++;
	}
}

void XPolynomialTransform3::Transform(const iplPOINT2D *ptsSrc, int n, iplPOINT2D *ptsDst)
{
	int k;
	double dx, dy;

	for (k = 0; k < n; k++)
	{
		double dx, dy, dx2, dxy, dy2, dx3, dx2y, dxdy2, dy3;

		dx = (ptsSrc->x - m_pcSrc.x);
		dy = (ptsSrc->y - m_pcSrc.y);

		dx2 = dx*dx; dxy = dx*dy;	dy2 = dy*dy;
		dx3 = dx2*dx;	dx2y = dx2*dy;	dxdy2 = dx*dy2;	dy3 = dy2*dy;

		ptsDst->x = m_pcDst.x + (m_a[0] + m_a[1] * dx + m_a[2] * dy + m_a[3] * dx2 + m_a[4] * dxy + m_a[5] * dy2 + m_a[6] * dx3 + m_a[7] * dx2y + m_a[8] * dxdy2 + m_a[9] * dy3);
		ptsDst->y = m_pcDst.y + (m_b[0] + m_b[1] * dx + m_b[2] * dy + m_b[3] * dx2 + m_b[4] * dxy + m_b[5] * dy2 + m_b[6] * dx3 + m_b[7] * dx2y + m_b[8] * dxdy2 + m_b[9] * dy3);

		ptsSrc++;	ptsDst++;
	}
}




void XPolynomialTransform3::Transform(double *x, double *y)
{
	double dx, dy, dx2, dxy, dy2, dx3, dx2y, dxdy2, dy3;

	dx = *x - m_pcSrc.x;	dy = *y - m_pcSrc.y;

	dx2 = dx*dx; dxy = dx*dy;	dy2 = dy*dy;
	dx3 = dx2*dx;	dx2y = dx2*dy;	dxdy2 = dx*dy2;	dy3 = dy2*dy;

	*x = m_pcDst.x + (m_a[0] + m_a[1] * dx + m_a[2] * dy + m_a[3] * dx2 + m_a[4] * dxy + m_a[5] * dy2 + m_a[6] * dx3 + m_a[7] * dx2y + m_a[8] * dxdy2 + m_a[9] * dy3);
	*y = m_pcDst.y + (m_b[0] + m_b[1] * dx + m_b[2] * dy + m_b[3] * dx2 + m_b[4] * dxy + m_b[5] * dy2 + m_b[6] * dx3 + m_b[7] * dx2y + m_b[8] * dxdy2 + m_b[9] * dy3);
}


void XPolynomialTransform3::GetMeanError(double *mx, double *my)
{
	*mx = m_mx;
	*my = m_my;
}

// 取变换参数
void XPolynomialTransform3::GetParameter(iplPOINT2D *pcSrc, iplPOINT2D *pcDst, double *a, double *b)
{
	*pcSrc = m_pcSrc;
	*pcDst = m_pcDst;

	memcpy(a, m_a, 10 * sizeof(double));
	memcpy(b, m_b, 10 * sizeof(double));
}

// 取变换参数
void XPolynomialTransform3::GetParameter(double *a, double *b)
{
	double dx, dy, dx2, dxy, dy2, dx3, dx2y, dxdy2, dy3;

	// 常数项
	dx = -m_pcSrc.x;	dy = -m_pcSrc.y;
	dx2 = dx*dx; dxy = dx*dy;	dy2 = dy*dy;
	dx3 = dx2*dx;	dx2y = dx2*dy;	dxdy2 = dx*dy2;	dy3 = dy2*dy;;

	a[0] = m_pcDst.x + (m_a[0] + m_a[1] * dx + m_a[2] * dy + m_a[3] * dx2 + m_a[4] * dxy + m_a[5] * dy2 + m_a[6] * dx3 + m_a[7] * dx2y + m_a[8] * dxdy2 + m_a[9] * dy3);
	a[1] = m_a[1] + m_a[3] * 2 * dx + m_a[4] * dy + m_a[6] * 3 * dx2 + m_a[7] * 2 * dxy + m_a[8] * dy2;
	a[2] = m_a[2] + m_a[4] * dx + m_a[5] * 2 * dy + m_a[7] * dx2 + m_a[8] * 2 * dxy + m_a[9] * 3 * dy2;

	a[3] = m_a[3] + 3 * m_a[6] * dx + m_a[7] * dy;
	a[4] = m_a[4] + 2 * m_a[7] * dx + 2 * m_a[8] * dy;
	a[5] = m_a[5] + m_a[8] * dx + 3 * m_a[9] * dy;

	a[6] = m_a[6];
	a[7] = m_a[7];
	a[8] = m_a[8];
	a[9] = m_a[9];

	b[0] = m_pcDst.y + (m_b[0] + m_b[1] * dx + m_b[2] * dy + m_b[3] * dx2 + m_b[4] * dxy + m_b[5] * dy2 + m_b[6] * dx3 + m_b[7] * dx2y + m_b[8] * dxdy2 + m_b[9] * dy3);
	b[1] = m_b[1] + m_b[3] * 2 * dx + m_b[4] * dy + m_b[6] * 3 * dx2 + m_b[7] * 2 * dxy + m_b[8] * dy2;
	b[2] = m_b[2] + m_b[4] * dx + m_b[5] * 2 * dy + m_b[7] * dx2 + m_b[8] * 2 * dxy + m_b[9] * 3 * dy2;

	b[3] = m_b[3] + 3 * m_b[6] * dx + m_b[7] * dy;
	b[4] = m_b[4] + 2 * m_b[7] * dx + 2 * m_b[8] * dy;
	b[5] = m_b[5] + m_b[8] * dx + 3 * m_b[9] * dy;

	b[6] = m_b[6];
	b[7] = m_b[7];
	b[8] = m_b[8];
	b[9] = m_b[9];

	// test
// 	dx = m_pcSrc.x;	dy = m_pcSrc.y;
// 	dx2 = dx*dx; dxy = dx*dy;	dy2 = dy*dy;
// 	dx3 = dx2*dx;	dx2y = dx2*dy;	dxdy2 = dx*dy2;	dy3 = dy2*dy;
// 
// 	double x1 = m_pcDst.x + (m_a[0] + m_a[1] * dx + m_a[2] * dy + m_a[3] * dx2 + m_a[4] * dxy + m_a[5] * dy2 + m_a[6] * dx3 + m_a[7] * dx2y + m_a[8] * dxdy2 + m_a[9] * dy3);
// 
// 	dx = 2*m_pcSrc.x;	dy = 2*m_pcSrc.y;
// 	dx2 = dx*dx; dxy = dx*dy;	dy2 = dy*dy;
// 	dx3 = dx2*dx;	dx2y = dx2*dy;	dxdy2 = dx*dy2;	dy3 = dy2*dy;
// 
// 	double x2 = a[0] + a[1] * dx + a[2] * dy + a[3] * dx2 + a[4] * dxy + a[5] * dy2 + a[6] * dx3 + a[7] * dx2y + a[8] * dxdy2 + a[9] * dy3;


}




//////////////////////////////////////////////////////////////////////////
// xDst - xcDst = a0 + a1*( xSrc-xcSrc) + a2*(ySrc-ycSrc) + a3*xx + a4*xy + a5*yy + a6*xxx
// yDst - ycDst = b0 + b1*( xSrc-xcSrc) + b2*(ySrc-ycSrc) + b3*xx + b4*xy + b5*yy
//
//////////////////////////////////////////////////////////////////////////

void XPolynomialTransform2_xDist::Initialize(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights)
{
	m_pcSrc = Average(ptsSrc, n);
	m_pcDst = Average(ptsDst, n);

	////////////////////////////////////
	int i, j, k;
	double a[9];

	Eigen::MatrixXd AA(7, 7), BB(6, 6);
	Eigen::VectorXd AL(7), BL(6);

// 	orsMatrixD AA(7, 7), BB(6, 6);
// 	orsVectorD AL(7), BL(6);

	AA = Eigen::MatrixXd::Zero(7, 7);	
	BB = Eigen::MatrixXd::Zero(6, 6);	
	AL = Eigen::VectorXd::Zero(7);	
	BL = Eigen::VectorXd::Zero(6);

	for (k = 0; k < n; k++)
	{
		a[0] = 1;			a[1] = ptsSrc[k].x - m_pcSrc.x;	a[2] = ptsSrc[k].y - m_pcSrc.y;
		a[3] = a[1] * a[1];	a[4] = a[1] * a[2];		a[5] = a[2] * a[2];

		// 畸变项 r3, r5, r7
		a[6] = a[1] * a[3];		//a[7] = a[6] * a[3];		a[8] = a[7] * a[3];		

		for (i = 0; i < 7; i++)
		{
			for (j = 0; j < 7; j++)
				AA(i, j) += a[i] * a[j];

			AL[i] += a[i] * (ptsDst[k].x - m_pcDst.x);
		}

		for (i = 0; i < 6; i++)
		{
			for (j = 0; j < 6; j++)
				BB(i, j) += a[i] * a[j];

			BL[i] += a[i] * (ptsDst[k].y - m_pcDst.y);
		}
	}

	//////////////////////////////////////////////////////////////////////////
	{
		Eigen::MatrixXd AI = AA.inverse();
// 		orsMatrixD AI = AA;
// 		getMatrixService()->MatrixInverse(AI);

		Eigen::VectorXd X(7);
		X = AI*AL;
		memcpy(m_a, &X[0], sizeof(double) * 7);
// 		orsVectorD	X(7);
// 		getMatrixService()->MatrixMultiplyVector(AI, AL, X);	X.CopyData(m_a);
	}
	{
		Eigen::MatrixXd AI = BB.inverse();
// 		orsMatrixD AI = BB;
// 		getMatrixService()->MatrixInverse(AI);

		Eigen::VectorXd Y(6);
		Y = AI*BL;
		memcpy(m_b, &Y[0], sizeof(double) * 6);
// 		orsVectorD	Y(6);
// 		getMatrixService()->MatrixMultiplyVector(AI, BL, Y); Y.CopyData(m_b);
	}

	//////////////////////////////////////////////////////////////////////////

	m_mx = m_my = 0;
	double vx, vy, dx, dy, dy2, dxy, dx2, dx3;	// dx5, dx7;
	for (k = 0; k < n; k++)
	{
		dx = (ptsSrc[k].x - m_pcSrc.x);
		dy = (ptsSrc[k].y - m_pcSrc.y);

		dx2 = dx*dx; dx3 = dx2*dx;	//dx5 = dx3*dx2;	dx7=dx5*dx2;
		dxy = dx*dy;
		dy2 = dy*dy;

		vx = (ptsDst[k].x - m_pcDst.x) - (m_a[0] + m_a[1] * dx + m_a[2] * dy + m_a[3] * dx2 + m_a[4] * dxy + m_a[5] * dy2)
			- (m_a[6] * dx3);// + m_a[7]*dx5 + m_a[8]*dx7);

		vy = (ptsDst[k].y - m_pcDst.y) - (m_b[0] + m_b[1] * dx + m_b[2] * dy + m_b[3] * dx2 + m_b[4] * dxy + m_b[5] * dy2);

		m_mx += vx*vx;
		m_my += vy*vy;
	}

	if (n > 7)	{
		m_mx = sqrt(m_mx / (n - 7));
		m_my = sqrt(m_my / (n - 6));
	}
	else	{
		m_mx = m_my = 0;
	}
}


void XPolynomialTransform2_xDist::GetResidual(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys)
{
	int k;
	double vx, vy, dx, dy, dx2, dx3, dx5, dx7, dy2, dxy;
	/*
		for( k=0; k<n ; k++)
		{
		dx = (ptsSrc[k].x - m_pcSrc.x);
		dy = (ptsSrc[k].y - m_pcSrc.y);

		dx2 = dx*dx; dx3 = dx2*dx;
		dxy = dx*dy;	dy2 = dy*dy;

		vx = ( ptsDst[k].x - m_pcDst.x) - ( m_a[0] + m_a[1]*dx + m_a[2]*dy + m_a[3]*dx2 + m_a[4]*dxy + m_a[5]*dy2 )
		- ( m_a[6]*dx3 );

		vy = ( ptsDst[k].y - m_pcDst.y) - ( m_b[0] + m_b[1]*dx + m_b[2]*dy + m_b[3]*dx2 + m_b[4]*dxy + m_b[5]*dy2 );

		pVxys++;
		}
		*/
	double a[7], b[6];

	GetParameter(a, b);

	for (k = 0; k < n; k++)
	{
		dx = ptsSrc[k].x;	dy = ptsSrc[k].y;
		dx2 = dx*dx;	dx3 = dx2*dx;
		dxy = dx*dy;	dy2 = dy*dy;

		pVxys->x = ptsDst[k].x - (a[0] + a[1] * dx + a[2] * dy + a[3] * dx2 + a[4] * dxy + a[5] * dy2 + a[6] * dx3);
		pVxys->y = ptsDst[k].y - (b[0] + b[1] * dx + b[2] * dy + b[3] * dx2 + b[4] * dxy + b[5] * dy2);

		pVxys++;
	}

}

void XPolynomialTransform2_xDist::Transform(const iplPOINT2D *ptsSrc, int n, iplPOINT2D *ptsDst)
{
	int k;
	double dx, dy, dx2, dxy, dy2, dx3;

	for (k = 0; k < n; k++)
	{
		dx = (ptsSrc->x - m_pcSrc.x);	dy = (ptsSrc->y - m_pcSrc.y);
		dx2 = dx*dx;	dy2 = dy*dy;
		dxy = dx*dy;	dx3 = dx*dx2;

		ptsDst->x = m_pcDst.x + (m_a[0] + m_a[1] * dx + m_a[2] * dy + m_a[3] * dx2 + m_a[4] * dxy + m_a[5] * dy2 + m_a[6] * dx3);
		ptsDst->y = m_pcDst.y + (m_b[0] + m_b[1] * dx + m_b[2] * dy + m_b[3] * dx2 + m_b[4] * dxy + m_b[5] * dy2);

		ptsSrc++;	ptsDst++;
	}
}




void XPolynomialTransform2_xDist::Transform(double *x, double *y)
{
	double dx, dy, dxy, dx2, dy2, dx3;

	dx = *x - m_pcSrc.x;	dy = *y - m_pcSrc.y;

	dx2 = dx*dx;	dy2 = dy*dy;
	dxy = dx*dy;	dx3 = dx*dx2;

	*x = m_pcDst.x + (m_a[0] + m_a[1] * dx + m_a[2] * dy + m_a[3] * dx2 + m_a[4] * dxy + m_a[5] * dy2 + m_a[6] * dx3);
	*y = m_pcDst.y + (m_b[0] + m_b[1] * dx + m_b[2] * dy + m_b[3] * dx2 + m_b[4] * dxy + m_b[5] * dy2);
}


void XPolynomialTransform2_xDist::GetMeanError(double *mx, double *my)
{
	*mx = m_mx;
	*my = m_my;
}



// 取变换参数
void XPolynomialTransform2_xDist::GetParameter(iplPOINT2D *pcSrc, iplPOINT2D *pcDst, double *a, double *b)
{
	*pcSrc = m_pcSrc;
	*pcDst = m_pcDst;

	int i;
	for (i = 0; i < 7; i++)
		a[i] = m_a[i];

	for (i = 0; i < 6; i++)
		b[i] = m_b[i];
}

// 取变换参数
void XPolynomialTransform2_xDist::GetParameter(double *a, double *b)
{
	int i;

	for (i = 0; i < 7; i++)
		a[i] = m_a[i];

	for (i = 0; i < 6; i++)
		b[i] = m_b[i];

	// xDst - xcDst = a0 + a1*( xSrc-xcSrc) + a2*(ySrc-ycSrc) + a3*( xSrc-xcSrc)*(xSrc-xcSrc) + a4*( xSrc-xcSrc)*(ySrc-ycSrc) + a5*( ySrc-ycSrc)*(ySrc-ycSrc) + a6*( xSrc-xcSrc)*(xSrc-xcSrc)*(xSrc-xcSrc)
	//              = a0 - a1*xcSrc - a2*ycSrc + a3*xcSrc*xcSrc + a4*xcScrc*ycSrc + a5*ycSrc*ycSrc - a6*xcSrc*xcSrc*xcSrc
	//				 + ( a1 - 2*a3*xcSrc -  a4*ycSrc)*xSrc
	//				 + ( a2              - *a4*xcSrc -2*a5*ycSrc)*ySrc
	//				 + a3*xSrc*xSrc - 3*a6*xcSrc
	//				 + a4*xSrc*ySrc
	//				 + a5*ySrc*ySrc;

	a[0] += m_pcDst.x - a[1] * m_pcSrc.x - a[2] * m_pcSrc.y + a[3] * m_pcSrc.x*m_pcSrc.x
		+ a[4] * m_pcSrc.x*m_pcSrc.y
		+ a[5] * m_pcSrc.y*m_pcSrc.y
		- a[6] * m_pcSrc.x*m_pcSrc.x*m_pcSrc.x;

	a[1] -= 2 * a[3] * m_pcSrc.x + a[4] * m_pcSrc.y - 3 * a[6] * m_pcSrc.x*m_pcSrc.x;
	a[2] -= a[4] * m_pcSrc.x + 2 * a[5] * m_pcSrc.y;
	a[3] -= 3 * a[6] * m_pcSrc.x;

	////////////
	b[0] += m_pcDst.y - b[1] * m_pcSrc.x - b[2] * m_pcSrc.y + b[3] * m_pcSrc.x*m_pcSrc.x
		+ b[4] * m_pcSrc.x*m_pcSrc.y
		+ b[5] * m_pcSrc.y*m_pcSrc.y;
	b[1] -= 2 * b[3] * m_pcSrc.x + b[4] * m_pcSrc.y;
	b[2] -= b[4] * m_pcSrc.x + 2 * b[5] * m_pcSrc.y;
}



//////////////////////////////////////////////////////////////////////////


