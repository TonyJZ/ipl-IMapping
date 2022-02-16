#pragma once


#include "simplefeature/interface/ISFGeometry.h"

namespace ipl
{
	class XSFGeometryCollection : public ISFGeometryCollection
	{
	protected:
		iplArray<ref_ptr<IGeoObject> >m_geometries;

	public:
		XSFGeometryCollection();
		virtual ~XSFGeometryCollection();

		virtual int getDimension() const { return 2; };

		virtual void getEnvelope(iplEnvelope * psEnvelope) const;

		virtual IPL_wkbGeometryType getGeometryType() const
		{
			if (m_coordDimension == 2)
				return IPL_wkbGeometryCollection;
			else
				return IPL_wkbGeometryCollection25D;
		}

		virtual const char *getGeometryName() const
		{
			return "GEOMETRYCOLLECTION";
		}

	public:

		virtual int	getNumGeometries() const { return m_geometries.size(); };
		virtual ref_ptr<IGeoObject>	getGeometryRef(int index) 
		{
			ref_ptr<IGeoObject>  nullRing;
			nullRing.reset();

			if (index<0 || index>m_geometries.size() - 1)
				return nullRing;

			return m_geometries[index]; 
		};

		virtual SFERR addGeometry(const ref_ptr<IGeoObject> geom);
		virtual SFERR addGeometryDirectly(ref_ptr<IGeoObject> geom);

		virtual SFERR removeGeometry(int, int bDelete);

		virtual IGeoObject *Clone() const;

	public:
		IPL_OBJECT_IMP1(XSFGeometryCollection, ISFGeometryCollection, "default", "SF GeometryCollection")
	};

}

