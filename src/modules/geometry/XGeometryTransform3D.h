#pragma once


#include "geometry/interface/IGeometryTransform.h"


namespace ipl
{
	//////////////////////////////////////////////////////////////////////////
	// xDst - xcDst = a0*( xSrc-xcSrc) + a1*(ySrc-ycSrc) + a2*(zSrc - zcSrc)
	// yDst - ycDst = b0*( xSrc-xcSrc) + b1*(ySrc-ycSrc) + b2*(zSrc - zcSrc)
	// zDst - zcDst = c0*( xSrc-xcSrc) + c1*(ySrc-ycSrc) + c2*(zSrc - zcSrc)
	//////////////////////////////////////////////////////////////////////////
	class XAffineTransform3D : public IGeometryTransform3D
	{
	public:
		int GetLeastNumberOfObservations() { return 3; }

		void Initialize(const iplPOINT3D *ptsSrc, int n, const iplPOINT3D *ptsDst, float *weights = NULL);
		void GetResidual(const iplPOINT3D *ptsSrc, int n, const iplPOINT3D *ptsDst, iplPOINT3D *pVxys);
		void GetMeanError(double *mx, double *my, double *mz);

		// 中心化的参数: a[3],b[3],c[3]
		void GetParameter(iplPOINT3D *pcSrc, iplPOINT3D *pcDst, double *a, double *b, double *c);

		// 不需中心化的参数: a[4],b[4],c[4]
		void GetParameter(double *a, double *b, double *c);

		void Transform(const iplPOINT3D *ptsSrc, int n, iplPOINT3D *ptsDst);

	private:
		double m_a[4];
		double m_b[4];
		double m_c[4];

		double m_mx;
		double m_my;
		double m_mz;

		iplPOINT3D	m_pcSrc;
		iplPOINT3D	m_pcDst;

	public:
		IPL_OBJECT_IMP1(XAffineTransform3D, IGeometryTransform3D, "affine", "3D Affine Transform")
	};




	///////////////////////////////////////////////////////////////////////
	// simplified XAffineTransform3D:	a2=b2=0, c2=1
	//
	//		xDst - xcDst = a0*( xSrc-xcSrc) + a1*(ySrc-ycSrc)
	//		yDst - ycDst = b0*( xSrc-xcSrc) + b1*(ySrc-ycSrc)
	//		zDst - zcDst = c0*( xSrc-xcSrc) + c1*(ySrc-ycSrc) + (zSrc - zcSrc)
	//////////////////////////////////////////////////////////////////////////
	class XAffineTransform25D : public IGeometryTransform3D
	{
	public:
		int GetLeastNumberOfObservations() { return 6; }

		void Initialize(const iplPOINT3D *ptsSrc, int n, const iplPOINT3D *ptsDst, float *weights = NULL);
		void GetResidual(const iplPOINT3D *ptsSrc, int n, const iplPOINT3D *ptsDst, iplPOINT3D *pVxys);
		void GetMeanError(double *mx, double *my, double *mz);

		// 中心化的参数: a[2],b[2],c[2]
		void GetParameter(iplPOINT3D *pcSrc, iplPOINT3D *pcDst, double *a, double *b, double *c);

		// 不需中心化的参数: a[3],b[2],c[3]
		void GetParameter(double *a, double *b, double *c);

		void Transform(const iplPOINT3D *ptsSrc, int n, iplPOINT3D *ptsDst);

	private:
		double m_a[3];
		double m_b[3];
		double m_c[3];

		double m_mx;
		double m_my;
		double m_mz;

		iplPOINT3D	m_pcSrc;
		iplPOINT3D	m_pcDst;

	public:
		IPL_OBJECT_IMP1(XAffineTransform25D, IGeometryTransform3D, "affine25D", "2.5D Affine Transform")
	};




	///////////////////////////////////////////////////////////////////////////////
	//				   l0*( xSrc-xcSrc) + l1*(ySrc-ycSrc) + l2*(zSrc-zcSrc) + l3
	// xDst - xcDst = ------------------------------------------------------------
	//				   l8*( xSrc-xcSrc) + l9*(ySrc-ycSrc) + l10*(zSrc-zcSrc) + 1
	//
	//				   l4*( xSrc-xcSrc)  + l5*(ySrc-ycSrc)  + l6*(zSrc-zcSrc) + l7
	// yDst - ycDst = --------------------------------------------------------------
	//				   l8*( xSrc-xcSrc) + l9*(ySrc-ycSrc) + l10*(zSrc-zcSrc) + 1
	/////////////////////////////////////////////////////////////////////////////////
	class XDLTTransform_3D : public IGeometryTransform3D_2D
	{
	public:
		XDLTTransform_3D();

		int GetLeastNumberOfObservations() { return 6; }

		void Initialize(const iplPOINT3D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights = NULL);
		void GetResidual(const iplPOINT3D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys);
		void GetMeanError(double *mx, double *my);

		// 中心化的参数
		void GetParameter(iplPOINT3D *pcSrc, iplPOINT2D *pcDst, double *a, double *b = NULL, double *c = NULL);

		// 不需中心化的参数
		void GetParameter(double *a, double *b = NULL, double *c = NULL);

		void Transform(const iplPOINT3D *ptsSrc, int n, iplPOINT2D *ptsDst);

		void Writerpc(const char *outputfile)
		{
			assert(false);
		};

	private:
		double m_l[11];

		iplPOINT3D	m_pcSrc;
		iplPOINT2D	m_pcDst;

		double		m_xScale;
		double		m_yScale;

		double		m_XScale;
		double		m_YScale;
		double		m_ZScale;

		double m_mx, m_my;

	public:
		IPL_OBJECT_IMP1(XDLTTransform_3D, IGeometryTransform3D_2D, "dlt", "3D DLT")

	};




	///////////////////////////////////////////////////////////////////////////////
	// (Younian Wang, 1999)以线阵CCD影像的严格几何模型为基础, 推导出了自检校型直接线性变换模型SDLT
	//
	//				   l0*(xSrc-xcSrc) + l1*(ySrc-ycSrc) + l2*(zSrc-zcSrc) + l3
	// xDst - xcDst = ------------------------------------------------------------
	//				   l8*(xSrc-xcSrc) + l9*(ySrc-ycSrc) + l10*(zSrc-zcSrc) + 1
	//
	//				   l4*(xSrc-xcSrc)  + l5*(ySrc-ycSrc) + l6*(zSrc-zcSrc) + l7
	// yDst - ycDst = ------------------------------------------------------------ + l11*(xDst-xcDst)*(yDst-ycDst)
	//				   l8*(xSrc-xcSrc) + l9*(ySrc-ycSrc) + l10*(zSrc-zcSrc) + 1
	//
	// 将l11项移至左侧，合并同类项，再将第一个公式代入，可知SDLT实际上是RPC01的特殊情况
	/////////////////////////////////////////////////////////////////////////////////
	class XSDLTTransform_3D : public IGeometryTransform3D_2D
	{
	public:
		XSDLTTransform_3D();

		int GetLeastNumberOfObservations() { return 6; }

		void Initialize(const iplPOINT3D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights = NULL);
		void GetResidual(const iplPOINT3D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys);
		void GetMeanError(double *mx, double *my);

		// 中心化的参数
		void GetParameter(iplPOINT3D *pcSrc, iplPOINT2D *pcDst, double *a, double *b = NULL, double *c = NULL);

		// 不需中心化的参数
		void GetParameter(double *a, double *b = NULL, double *c = NULL);

		void Transform(const iplPOINT3D *ptsSrc, int n, iplPOINT2D *ptsDst);

		virtual void Writerpc(const char *outputfile)
		{
			assert(false);
		};

	private:
		double m_l[12];

		iplPOINT3D	m_pcSrc;
		iplPOINT2D	m_pcDst;

		double		m_xScale;
		double		m_yScale;

		double		m_XScale;
		double		m_YScale;
		double		m_ZScale;

		double m_mx, m_my;

	public:
		IPL_OBJECT_IMP1(XSDLTTransform_3D, IGeometryTransform3D_2D, "sdlt", "3D SDLT")

	};




	///////////////////////////////////////////////////////////////////////////////
	//				   l0*( xSrc-xcSrc) + l1*(ySrc-ycSrc) + l2*(zSrc-zcSrc) + l3
	// xDst - xcDst = ------------------------------------------------------------
	//				   l4*( xSrc-xcSrc) + l5*(ySrc-ycSrc) + l6*(zSrc-zcSrc) + 1
	//
	//				   l7*( xSrc-xcSrc)  + l8*(ySrc-ycSrc)  + l8*(zSrc-zcSrc) + l10
	// yDst - ycDst = --------------------------------------------------------------
	//				   l11*( xSrc-xcSrc) + l12*(ySrc-ycSrc) + l13*(zSrc-zcSrc)+ 1
	/////////////////////////////////////////////////////////////////////////////////
	class XRPC_O1 : public IGeometryTransform3D_2D
	{
	public:
		XRPC_O1();

		int GetLeastNumberOfObservations() { return 7; }

		void Initialize(const iplPOINT3D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights = NULL);
		void GetResidual(const iplPOINT3D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys);
		void GetMeanError(double *mx, double *my);

		// 中心化的参数
		void GetParameter(iplPOINT3D *pcSrc, iplPOINT2D *pcDst, double *a, double *b = NULL, double *c = NULL);

		// 不需中心化的参数
		void GetParameter(double *a, double *b = NULL, double *c = NULL);

		void Transform(const iplPOINT3D *ptsSrc, int n, iplPOINT2D *ptsDst);

		void Writerpc(const char *outputfile);

	private:
		double m_l[14];

		iplPOINT3D	m_pcSrc;
		iplPOINT2D	m_pcDst;

		double		m_xScale;
		double		m_yScale;

		double		m_XScale;
		double		m_YScale;
		double		m_ZScale;

		double m_mx, m_my;

	public:
		IPL_OBJECT_IMP1(XRPC_O1, IGeometryTransform3D_2D, "rpcO1", "First Order RPC")

	};

	///////////////////////////////////////////////////////////////////////////////
	//		 Numl(P,L,H)
	// X = --------------
	//		 Denl(P,L,H)
	//
	//		Nums(P,L,H)
	// Y = --------------
	//		Dens(P,L,H)
	//Numl(P,L,H)=a1+a2*L+a3*P+a4*H+a5*L*P+a6*L*H+a7*P*H+a8*L*L+a9*P*P+a10*H*H+
	//a11*P*L*H+a12*L*L*L+a13*L*P*P+a14*L*H*H+a15*L*L*P+a16*P*P*P+a17*P*H*H+a18*L*L*H+a19*P*P*H+a20*H*H*H
	//Numl(P,L,H)=b1+b2*L+b3*P+b4*H+b5*L*P+b6*L*H+b7*P*H+b8*L*L+b9*P*P+b10*H*H+
	//b11*P*L*H+b12*L*L*L+b13*L*P*P+b14*L*H*H+b15*L*L*P+b16*P*P*P+b17*P*H*H+b18*L*L*H+b19*P*P*H+b20*H*H*H
	//Numl(P,L,H)=c1+c2*L+c3*P+c4*H+c5*L*P+c6*L*H+c7*P*H+c8*L*L+c9*P*P+c10*H*H+
	//c11*P*L*H+c12*L*L*L+c13*L*P*P+c14*L*H*H+c15*L*L*P+c16*P*P*P+c17*P*H*H+c18*L*L*H+c19*P*P*H+c20*H*H*H
	//Numl(P,L,H)=d1+d2*L+d3*P+d4*H+d5*L*P+d6*L*H+d7*P*H+d8*L*L+d9*P*P+d10*H*H+
	//d11*P*L*H+d12*L*L*L+d13*L*P*P+d14*L*H*H+d15*L*L*P+d16*P*P*P+d17*P*H*H+d18*L*L*H+d19*P*P*H+d20*H*H*H
	/////////////////////////////////////////////////////////////////////////////////


	// second order only
	class XRPC_O2 : public IGeometryTransform3D_2D
	{
	public:
		XRPC_O2();

		int GetLeastNumberOfObservations() { return 7; }

		void Initialize(const iplPOINT3D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights = NULL);
		void GetResidual(const iplPOINT3D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys);
		void GetMeanError(double *mx, double *my);

		// 中心化的参数
		void GetParameter(iplPOINT3D *pcSrc, iplPOINT2D *pcDst, double *a, double *b = NULL, double *c = NULL);

		// 不需中心化的参数
		void GetParameter(double *a, double *b = NULL, double *c = NULL);

		// 	//写出rpc参数
		void Writerpc(const char *outputfile);

		void Transform(const iplPOINT3D *ptsSrc, int n, iplPOINT2D *ptsDst);

	private:
		double m_a[19];
		double m_b[19];

		iplPOINT3D	m_pcSrc;
		iplPOINT2D	m_pcDst;

		double		m_xScale;
		double		m_yScale;

		double		m_XScale;
		double		m_YScale;
		double		m_ZScale;

		double m_mx, m_my;
		//	UnknownType m_l;
	public:
		IPL_OBJECT_IMP1(XRPC_O2, IGeometryTransform3D_2D, "rpcO2", "Sencond Order RPC")
	};



	class XRPC_O3 : public IGeometryTransform3D_2D
	{
	public:
		XRPC_O3();

		int GetLeastNumberOfObservations() { return 7; }

		void Initialize(const iplPOINT3D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights = NULL);
		void GetResidual(const iplPOINT3D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys);
		void GetMeanError(double *mx, double *my);

		// 中心化的参数
		void GetParameter(iplPOINT3D *pcSrc, iplPOINT2D *pcDst, double *a, double *b = NULL, double *c = NULL);

		// 不需中心化的参数
		void GetParameter(double *a, double *b = NULL, double *c = NULL);

		// 	//写出rpc参数
		void Writerpc(const char *outputfile);

		void Transform(const iplPOINT3D *ptsSrc, int n, iplPOINT2D *ptsDst);

	private:
		double m_a[39];
		double m_b[39];

		iplPOINT3D	m_pcSrc;
		iplPOINT2D	m_pcDst;

		double		m_xScale;
		double		m_yScale;

		double		m_XScale;
		double		m_YScale;
		double		m_ZScale;

		double m_mx, m_my;
		//	UnknownType m_l;
	public:
		IPL_OBJECT_IMP1(XRPC_O3, IGeometryTransform3D_2D, "rpcO3", "Third Order RPC")

	};

}
