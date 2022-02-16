#pragma once

#include "spatialreference/interface/ICoordinateTransform.h"
#include "XSpatialReference.h"

namespace ipl
{
	// affine transformation for fast processing
	struct	affineTransPARA {
		iplPOINT3D pcSrc;
		iplPOINT3D pcDst;
		double a[2];
		double b[2];
		double c[2];
	};

	struct fastTransDATA {
		double minX, minY;
		double maxX, maxY;
		double dx, dy;
		int	   nx, ny;

		affineTransPARA *grids;
	};


	class XCoordinateTransform : public ICoordinateTransform
	{
	private:
		static XEllipsoid m_WGS84;

		bool	m_ftAvailable;
		bool	m_bFastEnabled;

		fastTransDATA	m_forward;
		fastTransDATA	m_inverse;

	private:
		bool	m_bSameCS;		// same coordinate system
		bool	m_bSameVCS;		// same vertical coordinate system
		bool	m_bSameDatum;	// same Datum

		XSpatialReference *m_fromCS;
		XSpatialReference *m_toCS;

// 		ref_ptr<ISpatialReference>	m_fromCS0;
// 		ref_ptr<ISpatialReference>	m_toCS0;

		ISpatialReference*	m_fromCS0;
		ISpatialReference*	m_toCS0;


	private:
		bool MakeAffineParameters(fastTransDATA *trfData, double maxErr, bool isInverse = false);

		bool Transform(XSpatialReference *from, XSpatialReference *to, iplPOINT3D *pt);

		// 分开定义快慢变换，避免循环调用
		long Transform_slow(const iplPOINT3D *ptsFrom, int n, iplPOINT3D *ptsTo);
		long Inverse_slow(const iplPOINT3D *ptsFrom, int n, iplPOINT3D *ptsTo);

		long Transform_fast(const iplPOINT3D *ptsFrom, int n, iplPOINT3D *ptsTo);
		long Inverse_fast(const iplPOINT3D *ptsFrom, int n, iplPOINT3D *ptsTo);

	public:
		XCoordinateTransform();
		virtual ~XCoordinateTransform();

		virtual void Initialize(ISpatialReference *from, ISpatialReference *to);

		virtual bool InitFastTransform(double minX, double minY, double maxX, double maxY, double maxErr);

		virtual void EnableFast(bool bEnable = true) { m_bFastEnabled = bEnable; };

		// 只供外部调用，内部不能调用，否则可能死循环
		virtual long Transform(const iplPOINT3D *ptsFrom, int n, iplPOINT3D *ptsTo);
		virtual long Inverse(const iplPOINT3D *ptsFrom, int n, iplPOINT3D *ptsTo);

		IPL_OBJECT_IMP1(XCoordinateTransform, ICoordinateTransform, "default", "Spatial Coordinate Transformer")
	};


}
