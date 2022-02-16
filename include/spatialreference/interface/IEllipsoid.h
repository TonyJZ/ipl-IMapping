#pragma once

#include "core/ipldef.h"
#include "core/iplmacro.h"

#include "core/interface/IObject.h"

namespace ipl
{
	interface IEllipsoid : public IObject
	{
	public:
		//IEllipsoid();
		//virtual ~IEllipsoid();

		virtual bool SetParameter(double a, double b) = 0;

		virtual double GetSemiMajor() const = 0;
		virtual double GetSemiMinor() const = 0;

		virtual bool Geodetic_To_Geocentric(double latitude, double longitude, double height, double *X, double *Y, double *Z) = 0;
		virtual bool Geocentric_To_Geodetic(double X, double Y, double Z, double *latitude, double *longitude, double *height) = 0;

		virtual bool NearestIntersection(const iplRAY &ray, const double &height, iplPOINT3D *rtnPt) const = 0;

		IPL_INTERFACE_DEF(IObject, _T("ellipsoid"));
	};


	class ITangentPlane : public IObject
	{
	public:
		//ITangentPlane();

		// default WGS84
		virtual void InitializeEllipsoid(double a, double b) = 0;

		virtual void SetAnchorPoint(double lat0, double long0, double h0) = 0;
		virtual void GetAnchorPoint(double *lat0, double *long0, double *h0) const = 0;

		virtual void EnableSimpleModel() = 0;
		virtual void DisableSimpleModel() = 0;

		virtual void Geocentric2TangentPlane(double Xc, double Yc, double Zc, double *X, double *Y, double *Z) const = 0;
		virtual void TangentPlane2Geocentric(double X, double Y, double Z, double *Xc, double *Yc, double *Zc) const = 0;

		virtual void Geographic2TangentPlane(double lat, double lon, double h, double *X, double *Y, double *Z) const = 0;
		virtual void TangentPlane2Geographic(double X, double Y, double Z, double *lat, double *lon, double *h) const = 0;

		IPL_INTERFACE_DEF(IObject, _T("TangentPlane"));
	};

#define IPL_Ellipsoid_DEF			"ipl.ellipsoid.default"
#define IPL_TangentPlane_DEF		"ipl.TangentPlane.default"
}


