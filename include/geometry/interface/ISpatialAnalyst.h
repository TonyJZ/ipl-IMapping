#pragma once

#include "core/ipldef.h"
#include "core/iplmacro.h"
#include "core/interface/IObject.h"
#include "geometry/interface/IGeoObject.h"

namespace ipl
{
	interface ISpatialAnalyst : public IObject
	{
	public:
		//relation of Geo
		virtual int Intersects(IGeoObject *pGeo1, IGeoObject *pGeo2) const = 0;
		virtual int Equals(IGeoObject *pGeo1, IGeoObject *pGeo2) const = 0;
		virtual int Disjoint(IGeoObject *pGeo1, IGeoObject *pGeo2) const = 0;
		virtual int Touches(IGeoObject *pGeo1, IGeoObject *pGeo2) const = 0;
		virtual int Crosses(IGeoObject *pGeo1, IGeoObject *pGeo2) const = 0;
		virtual int Within(IGeoObject *pGeo1, IGeoObject *pGeo2) const = 0;
		virtual int Contains(IGeoObject *pGeo1, IGeoObject *pGeo2) const = 0;
		virtual int Overlaps(IGeoObject *pGeo1, IGeoObject *pGeo2) const = 0;


		IPL_INTERFACE_DEF(IObject, "spatialAnalyst");
	};

}
