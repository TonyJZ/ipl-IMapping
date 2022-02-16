#pragma once

#include "core/ipldef.h"
#include "core/iplmacro.h"
#include "core/interface/IObject.h"

namespace ipl
{
	//
	// IGeoid is to support conversions between ellipsoid heights and geoid heights.
	//
	interface IGeoid : public IObject
	{
		//IGeoid();
		virtual ~IGeoid() {};

		virtual	bool Initialize(const std::string & wktGeoid) = 0;

		/*
		* GetDeltaHeight returns the height of the WGS84 geiod above or below the WGS84 ellipsoid,
		* at the specified geodetic coordinates, using a grid of height adjustments from
		* the specified gravity model, such as EGM96.
		*
		* input:
		*		latitude    : Geodetic latitude in radians
		*		longitude   : Geodetic longitude in radians
		* output
		*		dh         : Height Adjustment, in meters
		*
		*/
		virtual	bool GetDeltaHeight(double latitudeInRadian, double longitudeInRadian, double *dh) = 0;

		/*
		* The function Ellipsoid2Geoid converts the specified WGS84 ellipsoid height
		* at the specified geodetic coordinates to the equivalent geoid height
		*
		* input:
		*		latitude           : Geodetic latitude in radians
		*		longitude          : Geodetic longitude in radians
		*		ellipsoidHeight    : Ellipsoid height, in meters
		* output
		*		geoidHeight        : Geoid height, in meters.
		*/
		virtual	bool Ellipsoid2Geoid(double latitudeInRadian, double longitudeInRadian, double ellipsoidHeight, double *geoidHeight) = 0;

		/*
		* The function Ellipsoid2Geoid converts the specified WGS84 ellipsoid height
		* at the specified geodetic coordinates to the equivalent geoid height
		*
		* input:
		*		latitude			: Geodetic latitude in radians
		*		longitude			: Geodetic longitude in radians
		*		geoidHeight			: Geoid height, in meters.
		* output
		*		ellipsoidHeight		: Ellipsoid height, in meters
		*/
		virtual	bool Geoid2Ellipsoid(double latitudeInRadian, double longitudeInRadian, double geoidHeight, double *ellipsoidHeight) = 0;


		IPL_INTERFACE_DEF(IObject, _T("Geoid"));
	};

#define IPL_Geoid_DEF		"ipl.Geoid.default"
}

