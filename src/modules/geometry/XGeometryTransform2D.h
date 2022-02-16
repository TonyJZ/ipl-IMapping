#pragma once

#include "core/iplmacro.h"
#include "geometry/interface/IGeometryTransform.h"

namespace ipl
{
	//////////////////////////////////////////////////////////////////////////
	// xDst - xcDst = a0
	// yDst - ycDst = b0
	// 刚性平移变换
	//////////////////////////////////////////////////////////////////////////
	class XTranslationTransform2D : public IGeometryTransform2D
	{
	public:
		int GetLeastNumberOfObservations() { return 1; }

		void Initialize(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights = NULL);
		void GetResidual(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys);
		void GetMeanError(double *mx, double *my);

		// 中心化的参数 
		void GetParameter(iplPOINT2D *pcSrc, iplPOINT2D *pcDst, double *a, double *b = NULL);

		// 不需中心化的参数
		void GetParameter(double *a, double *b = NULL);

		void Transform(const iplPOINT2D *ptsSrc, int n, iplPOINT2D *ptsDst);
		void Transform(double *x, double *y);

	private:
		double m_a[1];
		double m_b[1];

		double m_mx;
		double m_my;

		iplPOINT2D	m_pcSrc;
		iplPOINT2D	m_pcDst;

	public:
		IPL_OBJECT_IMP1(XTranslationTransform2D, IGeometryTransform2D, "translation", "2D translation Transform")
	};

	//////////////////////////////////////////////////////////////////////////
	// xDst - xcDst = a0*( xSrc-xcSrc) + a1*(ySrc-ycSrc)
	// yDst - ycDst = b0*( xSrc-xcSrc) + b1*(ySrc-ycSrc)
	//
	//////////////////////////////////////////////////////////////////////////
	class XAffineTransform2D : public IGeometryTransform2D
	{
	public:
		int GetLeastNumberOfObservations() { return 3; }

		void Initialize(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights = NULL);
		void GetResidual(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys);
		void GetMeanError(double *mx, double *my);

		// 中心化的参数 
		void GetParameter(iplPOINT2D *pcSrc, iplPOINT2D *pcDst, double *a, double *b = NULL);

		// 不需中心化的参数
		void GetParameter(double *a, double *b = NULL);

		void Transform(const iplPOINT2D *ptsSrc, int n, iplPOINT2D *ptsDst);
		void Transform(double *x, double *y);

	private:
		double m_a[2];
		double m_b[2];

		double m_mx;
		double m_my;

		iplPOINT2D	m_pcSrc;
		iplPOINT2D	m_pcDst;

	public:
		IPL_OBJECT_IMP1(XAffineTransform2D, IGeometryTransform2D, "affine", "2D Affine Transform")
	};


	//////////////////////////////////////////////////////////////////////////
	// xDst - xcDst =  a0*( xSrc-xcSrc) + a1*(ySrc-ycSrc)
	// yDst - ycDst = -a1*( xSrc-xcSrc) + a0*(ySrc-ycSrc)
	//
	//////////////////////////////////////////////////////////////////////////
	class XSimilarityTransform : public IGeometryTransform2D
	{
	public:
		int GetLeastNumberOfObservations() { return 2; }

		void Initialize(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights = NULL);
		void GetResidual(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys);
		void GetMeanError(double *mx, double *my);

		// 中心化的参数 
		void GetParameter(iplPOINT2D *pcSrc, iplPOINT2D *pcDst, double *a, double *b = NULL);

		// 不需中心化的参数
		void GetParameter(double *a, double *b = NULL);

		void Transform(const iplPOINT2D *ptsSrc, int n, iplPOINT2D *ptsDst);
		void Transform(double *x, double *y);

	private:
		double m_a[2];

		iplPOINT2D	m_pcSrc;
		iplPOINT2D	m_pcDst;

		double m_mx, m_my;

	public:
		IPL_OBJECT_IMP1(XSimilarityTransform, IGeometryTransform2D, "similarity", "2D Similarity Transform")

	};



	//////////////////////////////////////////////////////////////////////////
	// xDst - xcDst = a0 + a1*( xSrc-xcSrc) + a2*(ySrc-ycSrc) + a3*( xSrc-xcSrc)*(ySrc-ycSrc)
	// yDst - ycDst = b0 + b1*( xSrc-xcSrc) + b2*(ySrc-ycSrc) + b3*( xSrc-xcSrc)*(ySrc-ycSrc)
	//
	//////////////////////////////////////////////////////////////////////////
	class XBilinearTransform : public IGeometryTransform2D
	{
	public:
		int GetLeastNumberOfObservations() { return 4; }

		void Initialize(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights = NULL);
		void GetResidual(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys);
		void GetMeanError(double *mx, double *my);

		// 中心化的参数
		void GetParameter(iplPOINT2D *pcSrc, iplPOINT2D *pcDst, double *a, double *b = NULL);

		// 不需中心化的参数
		void GetParameter(double *a, double *b = NULL);

		void Transform(const iplPOINT2D *ptsSrc, int n, iplPOINT2D *ptsDst);
		void Transform(double *x, double *y);

	private:
		double m_a[4];
		double m_b[4];

		iplPOINT2D	m_pcSrc;
		iplPOINT2D	m_pcDst;

		double m_mx, m_my;

	public:
		IPL_OBJECT_IMP1(XBilinearTransform, IGeometryTransform2D, "bilinear", "2D Bilinear Transform")

	};


	//////////////////////////////////////////////////////////////////////////
	//	dx = xSrc-xcSrc		
	//  dy = ySrc-ycSrc
	//
	// xDst - xcDst = a0 + a1*dx + a2*dy + a3*dx*dx + a4*dx*dy + a5*dy*dy
	// yDst - ycDst = b0 + b1*dx + b2*dy + b3*dx*dx + b5*dx*dy + b5*dy*dy
	//
	// 二次多项式
	//////////////////////////////////////////////////////////////////////////
	class XPolynomialTransform2 : public IGeometryTransform2D
	{
	public:
		int GetLeastNumberOfObservations() { return 6; }

		void Initialize(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights = NULL);
		void GetResidual(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys);
		void GetMeanError(double *mx, double *my);

		// 中心化的参数
		void GetParameter(iplPOINT2D *pcSrc, iplPOINT2D *pcDst, double *a, double *b = NULL);

		// 不需中心化的参数
		void GetParameter(double *a, double *b = NULL);

		void Transform(const iplPOINT2D *ptsSrc, int n, iplPOINT2D *ptsDst);
		void Transform(double *x, double *y);

	private:
		double m_a[6];
		double m_b[6];

		iplPOINT2D	m_pcSrc;
		iplPOINT2D	m_pcDst;

		double m_mx, m_my;

	public:
		IPL_OBJECT_IMP1(XPolynomialTransform2, IGeometryTransform2D, "polynomial2", "Second Order Polynomial Transform")
	};

	//////////////////////////////////////////////////////////////////////////
	//薄板样条

	class XPolynomialTransformTPSpline : public IGeometryTransform2D
	{
	public:
		XPolynomialTransformTPSpline()
		{
			m_ptsSrc = NULL;

			m_a = NULL;
			m_b = NULL;
		}

		~XPolynomialTransformTPSpline()
		{
			if (NULL != m_ptsSrc)
				delete m_ptsSrc;

			if (NULL != m_a)
				delete m_a;

			if (NULL != m_b)
				delete m_b;
		}

		int GetLeastNumberOfObservations() { return 9; }

		void Initialize(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights = NULL);
		void GetResidual(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys);
		void GetMeanError(double *mx, double *my);

		// 中心化的参数
		void GetParameter(iplPOINT2D *pcSrc, iplPOINT2D *pcDst, double *a, double *b = NULL);

		// 不需中心化的参数
		void GetParameter(double *a, double *b = NULL);

		void Transform(const iplPOINT2D *ptsSrc, int n, iplPOINT2D *ptsDst);
		void Transform(double *x, double *y);

	private:
		double *m_a;
		double *m_b;

		int m_nCntrPts;//控制点个数

		iplPOINT2D	*m_ptsSrc;
		// 	iplPOINT2D	m_pcDst;

		double m_mx, m_my;

	public:
		IPL_OBJECT_IMP1(XPolynomialTransformTPSpline, IGeometryTransform2D, _T("TPSpline"), _T("Thin Plate Spline Transform"))
	};

	//////////////////////////////////////////////////////////////////////////
	//	dx = xSrc-xcSrc		
	//  dy = ySrc-ycSrc
	// xDst - xcDst = a0 + a1*dx + a2*dy + a3*dx*dx + a4*dx*dy + a5*dy*dy + a6*dx*dx*dx
	// yDst - ycDst = b0 + b1*dx + b2*dy + b3*dx*dx + b5*dx*dy + b5*dy*dy
	// 二次多项式+x方向畸变
	//////////////////////////////////////////////////////////////////////////
	class XPolynomialTransform2_xDist : public IGeometryTransform2D
	{
	public:

		int GetLeastNumberOfObservations() { return 9; }

		void Initialize(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights = NULL);
		void GetResidual(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys);
		void GetMeanError(double *mx, double *my);

		// 中心化的参数
		void GetParameter(iplPOINT2D *pcSrc, iplPOINT2D *pcDst, double *a, double *b = NULL);

		// 不需中心化的参数
		void GetParameter(double *a, double *b = NULL);

		void Transform(const iplPOINT2D *ptsSrc, int n, iplPOINT2D *ptsDst);
		void Transform(double *x, double *y);

	private:
		double m_a[9];
		double m_b[6];

		iplPOINT2D	m_pcSrc;
		iplPOINT2D	m_pcDst;

		double m_mx, m_my;

	public:
		IPL_OBJECT_IMP1(XPolynomialTransform2_xDist, IGeometryTransform2D, "polynomial2_xDist", "Second Order Polynomial Transform with Lens distorton in x direction")

	};




	//////////////////////////////////////////////////////////////////////////
	//	dx = xSrc-xcSrc		
	//  dy = ySrc-ycSrc
	// xDst - xcDst = a0 + a1*dx + a2*dy + a3*dx*dx + a4*dx*dy + a5*dy*dy + a6*dx*dx*dx + a7*dx*dx*dy + a8*dx*dy*dy + a9*dy*dy*dy
	// yDst - ycDst = b0 + b1*dx + b2*dy + b3*dx*dx + b5*dx*dy + b5*dy*dy + b6*dx*dx*dx + b7*dx*dx*dy + b8*dx*dy*dy + b9*dy*dy*dy
	//////////////////////////////////////////////////////////////////////////
	class XPolynomialTransform3 : public IGeometryTransform2D
	{
	public:
		int GetLeastNumberOfObservations() { return 20; }

		void Initialize(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights = NULL);
		void GetResidual(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys);
		void GetMeanError(double *mx, double *my);

		// 中心化的参数
		void GetParameter(iplPOINT2D *pcSrc, iplPOINT2D *pcDst, double *a, double *b = NULL);

		// 不需中心化的参数
		void GetParameter(double *a, double *b = NULL);

		void Transform(const iplPOINT2D *ptsSrc, int n, iplPOINT2D *ptsDst);
		void Transform(double *x, double *y);

	private:
		double m_a[10];
		double m_b[10];

		iplPOINT2D	m_pcSrc;
		iplPOINT2D	m_pcDst;

		double m_mx, m_my;

	public:
		IPL_OBJECT_IMP1(XPolynomialTransform3, IGeometryTransform2D, "polynomial3", "Third Order Polynomial Transform")
	};



	//////////////////////////////////////////////////////////////////////////
	//				   l0*( xSrc-xcSrc) + l1*(ySrc-ycSrc) + l2
	// xDst - xcDst = ------------------------------------------
	//				   l6*( xSrc-xcSrc) + l7*(ySrc-ycSrc) + 1
	//
	//				   l3*( xSrc-xcSrc) + l4*(ySrc-ycSrc) + l5
	// yDst - ycDst = ------------------------------------------
	//				   l6*( xSrc-xcSrc) + l7*(ySrc-ycSrc) + 1
	//////////////////////////////////////////////////////////////////////////
	class XDLTTransform_2D : public IGeometryTransform2D
	{
	public:
		XDLTTransform_2D();

		int GetLeastNumberOfObservations() { return 4; }

		void Initialize(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights = NULL);
		void GetResidual(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys);
		void GetMeanError(double *mx, double *my);

		// 中心化的参数
		void GetParameter(iplPOINT2D *pcSrc, iplPOINT2D *pcDst, double *a, double *b = NULL);

		// 不需中心化的参数
		void GetParameter(double *a, double *b = NULL);

		void Transform(const iplPOINT2D *ptsSrc, int n, iplPOINT2D *ptsDst);
		void Transform(double *x, double *y);

	private:
		double m_l[9];

		iplPOINT2D	m_pcSrc;
		iplPOINT2D	m_pcDst;

		double m_mx, m_my;

	public:
		IPL_OBJECT_IMP1(XDLTTransform_2D, IGeometryTransform2D, "dlt", "2D DLT")
	};

}
