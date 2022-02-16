#pragma once

#include "spatialreference/interface/ISpatialReference.h"

namespace ipl
{
	interface ICoordinateTransform : public IObject
	{
	public:
		ICoordinateTransform() {};
		virtual ~ICoordinateTransform() {};

		virtual void Initialize(ISpatialReference *from, ISpatialReference *to) = 0;
		virtual bool InitFastTransform(double minX, double minY, double maxX, double maxY, double maxErr) = 0;

		virtual void EnableFast(bool bEnable = true) = 0;

		// 正变换
		virtual long Transform(const iplPOINT3D *ptsFrom, int n, iplPOINT3D *ptsTo) = 0;

		// 反变换
		virtual long Inverse(const iplPOINT3D *ptsFrom, int n, iplPOINT3D *ptsTo) = 0;

		/////////////////////// 便利函数
		long Transform(const iplPOINT3D &ptFrom, iplPOINT3D *ptTo)
		{
			return Transform(&ptFrom, 1, ptTo);
		};

		long Transform(iplPOINT3D *pts, int n = 1)
		{
			return Transform(pts, n, pts);
		};

		long Inverse(const iplPOINT3D &ptFrom, iplPOINT3D *ptTo)
		{
			return Inverse(&ptFrom, 1, ptTo);
		};

		long Inverse(iplPOINT3D *pts, int n = 1)
		{
			return Inverse(pts, n, pts);
		};

		IPL_INTERFACE_DEF(IObject, _T("CoordinateTransform"))
	};

#define IPL_CoordinateTransform_DEF		"ipl.CoordinateTransform.default"
}

