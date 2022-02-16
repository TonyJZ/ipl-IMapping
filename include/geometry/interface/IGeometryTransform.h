#pragma once

#include "core/ipldef.h"
#include "core/iplmacro.h"
#include "core/interface/IObject.h"


namespace ipl
{
	interface IGeometryTransform2D : public IObject
	{
	public:
		virtual int GetLeastNumberOfObservations() = 0;

		virtual void Initialize(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights = NULL) = 0;
		virtual void GetResidual(const iplPOINT2D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys) = 0;
		virtual void GetMeanError(double *mx, double *my) = 0;

		// 中心化的参数
		virtual void GetParameter(iplPOINT2D *pcSrc, iplPOINT2D *pcDst, double *a, double *b = NULL) = 0;

		// 不需中心化的参数
		virtual void GetParameter(double *a, double *b = NULL) = 0;

		virtual void Transform(const iplPOINT2D *ptsSrc, int n, iplPOINT2D *ptsDst) = 0;

		virtual void Transform(double *x, double *y) = 0;

		IPL_INTERFACE_DEF(IObject, "geometry.transform2D");
	};

	interface IGeometryTransform3D : public IObject
	{
	public:
		virtual int GetLeastNumberOfObservations() = 0;

		virtual void Initialize(const iplPOINT3D *ptsSrc, int n, const iplPOINT3D *ptsDst, float *weights = NULL) = 0;
		virtual void GetResidual(const iplPOINT3D *ptsSrc, int n, const iplPOINT3D *ptsDst, iplPOINT3D *pVxys) = 0;
		virtual void GetMeanError(double *mx, double *my, double *mz) = 0;

		// 中心化的参数
		virtual void GetParameter(iplPOINT3D *pcSrc, iplPOINT3D *pcDst, double *a, double *b, double *c) = 0;

		// 不需中心化的参数
		virtual void GetParameter(double *a, double *b, double *c) = 0;

		virtual void Transform(const iplPOINT3D *ptsSrc, int n, iplPOINT3D *ptsDst) = 0;

		IPL_INTERFACE_DEF(IObject, "geometry.transform3D");
	};


	interface IGeometryTransform3D_2D : public IObject
	{
	public:
		virtual int GetLeastNumberOfObservations() = 0;

		virtual void Initialize(const iplPOINT3D *ptsSrc, int n, const iplPOINT2D *ptsDst, float *weights = NULL) = 0;
		virtual void GetResidual(const iplPOINT3D *ptsSrc, int n, const iplPOINT2D *ptsDst, iplPOINT2D *pVxys) = 0;
		virtual void GetMeanError(double *mx, double *my) = 0;

		// 中心化的参数
		virtual void GetParameter(iplPOINT3D *pcSrc, iplPOINT2D *pcDst, double *a, double *b = NULL, double *c = NULL) = 0;

		// 不需中心化的参数
		virtual void GetParameter(double *a, double *b = NULL, double *c = NULL) = 0;

		virtual void Transform(const iplPOINT3D *ptsSrc, int n, iplPOINT2D *ptsDst) = 0;

		virtual void Writerpc(const char * ImgName) = 0;

		IPL_INTERFACE_DEF(IObject, "geometry.transform3D_2D");
	};

#define IPL_GEOMETRY_TRANSFORM2D_TRANSLATION		"ipl.geometry.transform2D.translation"
#define IPL_GEOMETRY_TRANSFORM2D_AFFINE		"ipl.geometry.transform2D.affine"
#define IPL_GEOMETRY_TRANSFORM2D_SIMILARITY "ipl.geometry.transform2D.similarity"
#define IPL_GEOMETRY_TRANSFORM2D_BILINEAR	"ipl.geometry.transform2D.bilinear"
#define IPL_GEOMETRY_TRANSFORM2D_DLT		"ipl.geometry.transform2D.dlt"

#define IPL_GEOMETRY_TRANSFORM2D_POLYNOMIAL2	"ipl.geometry.transform2D.polynomial2"
#define IPL_GEOMETRY_TRANSFORM2D_POLYNOMIAL3	"ipl.geometry.transform2D.polynomial3"

#define IPL_GEOMETRY_TRANSFORM2D_TPSPLINE		"ipl.geometry.transform2D.TPSpline"

#define IPL_GEOMETRY_TRANSFORM3D_AFFINE			"ipl.geometry.transform3D.affine"
#define IPL_GEOMETRY_TRANSFORM3D_AFFINE25D		"ipl.geometry.transform3D.affine25D"

#define IPL_GEOMETRY_TRANSFORM3D_DLT		"ipl.geometry.transform3D_2D.dlt"
#define IPL_GEOMETRY_TRANSFORM3D_SDLT		"ipl.geometry.transform3D_2D.sdlt"

// #define IPL_GEOMETRY_TRANSFORM3D_RPCO1		"ipl.transform3D_2D.rpcO1"
// #define IPL_GEOMETRY_TRANSFORM3D_RPCO2		"ipl.transform3D_2D.rpcO2"
// #define IPL_GEOMETRY_TRANSFORM3D_RPCO3		"ipl.transform3D_2D.rpcO3"

}

