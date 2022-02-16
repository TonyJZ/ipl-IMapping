#pragma once

#include <assert.h>
#include "core/iplstd.h"
#include "simplefeature/interface/ISFGeometry.h"


namespace ipl
{
	class XSFPolygon : public ISFPolygon
	{
	private:
		iplArray <ref_ptr<ISFLineRing> > m_Rings;

	public:
		XSFPolygon();
		virtual ~XSFPolygon();

		virtual int getDimension() const { return 2; };

		virtual void getEnvelope(iplEnvelope * psEnvelope) const;

		virtual IPL_wkbGeometryType getGeometryType() const
		{
			if (m_coordDimension < 3)
				return IPL_wkbPolygon;
			else
				return IPL_wkbPolygon25D;
		}
		virtual const char *getGeometryName() const
		{
			return "POLYGON";
		}

	public:
		virtual double getArea() const;

		virtual void addRing(const ref_ptr<ISFLineRing> pRing);
		virtual void addRingDirectly(ref_ptr<ISFLineRing> pRing);

		virtual ref_ptr<ISFLineRing> getExteriorRing();
		virtual const ref_ptr<ISFLineRing> getExteriorRing() const;

		virtual int  getNumInteriorRings() const;

		virtual	ref_ptr<ISFLineRing> getInteriorRing(int i);
		virtual	const ref_ptr<ISFLineRing> getInteriorRing(int i) const ;

		virtual void  closeRings();

		virtual IGeoObject *Clone() const;

	public:
		IPL_OBJECT_IMP1(XSFPolygon, ISFPolygon, "default", "SF Polygon")

	};

}

