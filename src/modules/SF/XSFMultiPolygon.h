#pragma once

#include "simplefeature/interface/ISFGeometry.h"

namespace ipl
{
	class XSFMultiPolygon : public ISFMultiPolygon
	{
	private:
		iplArray <ref_ptr<ISFPolygon> > m_geometries;

	public:
		XSFMultiPolygon();
		virtual ~XSFMultiPolygon();

		virtual int getDimension() const { return 2; };

		virtual void getEnvelope(iplEnvelope * psEnvelope) const;

		virtual IPL_wkbGeometryType getGeometryType() const
		{
			if (m_coordDimension == 2)
				return IPL_wkbMultiPolygon;
			else
				return IPL_wkbMultiPolygon25D;
		}

		virtual const char *getGeometryName() const
		{
			return "MULTIPOLYGON";
		}

	public:
		virtual int	getNumGeometries() const { return m_geometries.size(); };
		virtual ref_ptr<IGeoObject>	getGeometryRef(int index) 
		{ 
			ref_ptr<IGeoObject>  nullRing;
			nullRing.reset();

			if (index<0 || index>m_geometries.size() - 1)
				return nullRing;

			return ipl_static_pointer_cast<IGeoObject>( m_geometries[index] );
		};

		virtual SFERR addGeometry(const ref_ptr<IGeoObject> geom);
		virtual SFERR addGeometryDirectly(ref_ptr<IGeoObject> geom);

		virtual SFERR removeGeometry(int, int bDelete);

		virtual IGeoObject *Clone() const;

	public:
		virtual double  getArea() const;

	public:
		IPL_OBJECT_IMP1(XSFMultiPolygon, ISFMultiPolygon, "default", "SF MultiPolygon")
	};
}
