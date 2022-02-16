#pragma once

/***************************************************************************/
/* RSC IDENTIFIER:  GEOCENTRIC
*
* ABSTRACT
*
*    This component provides conversions between Geodetic coordinates (latitude,
*    longitude in radians and height in meters) and Geocentric coordinates
*    (X, Y, Z) in meters.
*
* ERROR HANDLING
*
*    This component checks parameters for valid values.  If an invalid value
*    is found, the error code is combined with the current error code using
*    the bitwise or.  This combining allows multiple error codes to be
*    returned. The possible error codes are:
*
*      DTCC_NO_ERROR        : No errors occurred in function
*      DTCC_LAT_ERROR       : Latitude out of valid range
*                                 (-90 to 90 degrees)
*      DTCC_LON_ERROR       : Longitude out of valid range
*                                 (-180 to 360 degrees)
*      DTCC_A_ERROR         : Semi-major axis lessthan or equal to zero
*      DTCC_B_ERROR         : Semi-minor axis lessthan or equal to zero
*      DTCC_A_LESS_B_ERROR  : Semi-major axis less than semi-minor axis
*
*
* REUSE NOTES
*
*    GEOCENTRIC is intended for reuse by any application that performs
*    coordinate conversions between geodetic coordinates and geocentric
*    coordinates.
*
*
* REFERENCES
*
*    An Improved Algorithm for Geocentric to Geodetic Coordinate Conversion,
*    Ralph Toms, February 1996  UCRL-JC-123138.
*
*    Further information on GEOCENTRIC can be found in the Reuse Manual.
*
*    GEOCENTRIC originated from : U.S. Army Topographic Engineering Center
*                                 Geospatial Information Division
*                                 7701 Telegraph Road
*                                 Alexandria, VA  22310-3864
*
* LICENSES
*
*    None apply to this component.
*
* RESTRICTIONS
*
*    GEOCENTRIC has no restrictions.
*
*
***************************************************************************/

#include "core/ipldef.h"
#include "core/interface/IPlatform.h"
#include "spatialreference/interface/IEllipsoid.h"

namespace ipl
{
//	extern IPlatform	*g_orsPlatform;
//	extern ILogService	*g_orsProcess;

	class XEllipsoid : public IEllipsoid
	{
	private:
		double m_a;
		double m_b;
		double m_a2;
		double m_b2;
		double m_e2;
		double m_ep2;

		// for reducing error message after several times
		int			m_errorCount;

	public:
		XEllipsoid();
		virtual	~XEllipsoid() {};

		virtual bool SetParameter(double a, double b);

		virtual double GetSemiMajor()	const { return m_a; };
		virtual double GetSemiMinor()	const { return m_b; };

		virtual bool Geodetic_To_Geocentric(double latitude, double longitude, double height, double *X, double *Y, double *Z);
		virtual bool Geocentric_To_Geodetic(double X, double Y, double Z, double *latitude, double *longitude, double *height);

		/*!
		* METHOD: nearestIntersection()
		* Returns the point of nearest intersection of the ray with the ellipsoid.
		* computes the ray's intersection with a surface at some offset outside (for positive offset) of the ellipsoid (think
		* elevation).
		*/
		bool NearestIntersection(const iplRAY &ray, const double &height, iplPOINT3D *rtnPt) const;

		// Object方法实现
		IPL_OBJECT_IMP1(XEllipsoid, IEllipsoid, "default", "Ellipsoid")
	};

	extern XEllipsoid	_epWgs84;

	class XTangentPlane : public ITangentPlane
	{
	private:
		double m_a;			// major axis
		double m_b;			// minor axis

		double  m_lat0;		// Latitude of the Anchor point [rad]
		double  m_lon0;		// Longitude of the Anchor point [rad]
		double	m_hei0;

		XEllipsoid m_ellipsoid;

		bool	m_bSingleModel;

	private:
		double a2;
		double b2;
		double e2;
		double ep2;
		//	double R;					// radius for single model, use N0

		double cosLat0, sinLat0;
		double cosLon0, sinLon0;
		double N0;

		double m_Xg0, m_Yg0, m_Zg0;	// 切平面远点的地心坐标

	private:

	public:
		XTangentPlane();
		virtual ~XTangentPlane() {};

		// default WGS84
		void InitializeEllipsoid(double a, double b);

		void SetAnchorPoint(double lat0, double lon0, double h0);

		void GetAnchorPoint(double *lat0, double *lon0, double *h0) const
		{
			*lat0 = m_lat0;
			*lon0 = m_lon0;
			*h0 = m_hei0;
		}

		void EnableSimpleModel();
		void DisableSimpleModel();

		void Geocentric2TangentPlane(double Xc, double Yc, double Zc, double *X, double *Y, double *Z) const;
		void TangentPlane2Geocentric(double X, double Y, double Z, double *Xc, double *Yc, double *Zc) const;

		void Geographic2TangentPlane(double lat, double lon, double h, double *X, double *Y, double *Z) const;
		void TangentPlane2Geographic(double X, double Y, double Z, double *lat, double *lon, double *h) const;

		IPL_OBJECT_IMP1(XTangentPlane, ITangentPlane, "default", "Tangent Plane")
	};

}

