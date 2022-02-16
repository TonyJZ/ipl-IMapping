#pragma once


#include "simplefeature/interface/ISFGeometry.h"

namespace ipl
{
	class XSFPoint : public ISFPoint
	{
	public:
		XSFPoint();
		virtual ~XSFPoint();

		virtual void getEnvelope(iplEnvelope * psEnvelope) const
		{
			psEnvelope->minX = psEnvelope->maxX = x;
			psEnvelope->minY = psEnvelope->maxY = y;
		};

		IPL_wkbGeometryType getGeometryType() const
		{
			if (m_coordDimension < 3)
				return IPL_wkbPoint;
			else
				return IPL_wkbPoint25D;
		}

		virtual const char *getGeometryName() const
		{
			return "POINT";
		}

		virtual IGeoObject *Clone() const;

		//relation of Geo
// 		virtual int Intersects(SFIGeometry *pGeoOther) const;
// 		virtual int Equals(SFIGeometry *pGeoOther) const;
// 		virtual int Disjoint(SFIGeometry *pGeoOther) const;
// 		virtual int Touches(SFIGeometry *pGeoOther) const;
// 		virtual int Crosses(SFIGeometry *pGeoOther) const;
// 		virtual int Within(SFIGeometry *pGeoOther) const;
// 		virtual int Contains(SFIGeometry *pGeoOther) const;
// 		virtual int Overlaps(SFIGeometry *pGeoOther) const;

	public:
		IPL_OBJECT_IMP1(XSFPoint, ISFPoint, "default", "SF Point")
	};
}


