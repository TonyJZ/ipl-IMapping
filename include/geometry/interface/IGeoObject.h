#pragma once


#include "core/ipldef.h"
#include "core/iplmacro.h"
#include "core/interface/IObject.h"

#include "geometry/GeometryDef.h"


namespace ipl
{
	//几何对象的抽象接口
	interface IGeoObject : public IObject
	{
	public:
		ipl_int32 m_coordDimension;

	public:
		virtual int getDimension() const = 0;

		virtual void getEnvelope(iplEnvelope * psEnvelope) const = 0;
		virtual IPL_wkbGeometryType getGeometryType() const = 0;
		virtual const char *getGeometryName() const = 0;

		virtual IGeoObject *Clone() const = 0;
		//virtual void empty() = 0;

		IPL_INTERFACE_DEF(IObject, "GeoObject");
	};

}


