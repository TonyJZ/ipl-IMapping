#pragma once

#include <vector>
#include <string>

//gdal
#include <GDAL/ogr_api.h>

#include "core/iplstd.h"
#include "core/iplmacro.h"
#include "simplefeature/interface/ISFFeatureDefn.h"
#include "XSFFieldDefn.h"


namespace ipl
{
	class XSFFeatureDefn : public ISFFeatureDefn
	{
	private:
		//OGRFeatureDefnH * m_poFeatureDefn;
		// int         nRefCount;

		iplArray< ref_ptr<ISFFieldDefn> > m_vFiledDefn;

		IPL_wkbGeometryType m_geometryType;

		std::string	m_featureClassName;

	public:
		XSFFeatureDefn();
		virtual ~XSFFeatureDefn();

		void SetOGRFeatureDefn(OGRFeatureDefnH pofeatureDF);

		virtual void SetName(const char  *name);
		virtual const char  *GetName();

		virtual size_t         GetFieldCount() { return m_vFiledDefn.size(); };
		virtual ISFFieldDefn *GetFieldDefn(int i);
		virtual int         GetFieldIndex(const char *);

		virtual void        AddFieldDefn(ISFFieldDefn *);

		virtual IPL_wkbGeometryType GetGeomType()
		{
			return m_geometryType;
		};

		virtual void        SetGeomType(IPL_wkbGeometryType type) { m_geometryType = type; }

//		virtual ISFFeatureDefn *Clone() const;

	public:
		IPL_OBJECT_IMP1(XSFFeatureDefn, ISFFeatureDefn, "default", "SF Feature Define")
	};

}

