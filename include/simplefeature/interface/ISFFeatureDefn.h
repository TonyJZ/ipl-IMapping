#pragma once

#include "geometry/GeometryDef.h"
#include "simplefeature/SFBaseDef.h"
//#include "simplefeature/interface/SFIGeometry.h"
#include "simplefeature/interface/ISFFieldDefn.h"


namespace ipl
{
	interface ISFFeatureDefn : public IObject
	{
	public:
		virtual void SetName(const char  *name) = 0;
		virtual const char  *GetName() = 0;// { return pszFeatureClassName; }

		virtual size_t         GetFieldCount() = 0;// { return nFieldCount; }
		virtual ISFFieldDefn *GetFieldDefn(int i) = 0;
		virtual int         GetFieldIndex(const char *) = 0;

		virtual void        AddFieldDefn(ISFFieldDefn *) = 0;

		virtual IPL_wkbGeometryType GetGeomType() = 0;
		virtual void        SetGeomType(IPL_wkbGeometryType) = 0;

//		virtual ISFFeatureDefn *Clone() const = 0;
		//     virtual int         Reference()  = 0;
		//     virtual int         Dereference() =0;
		//     virtual void        Release() = 0;

		//     virtual int         GetReferenceCount() = 0;

		IPL_INTERFACE_DEF(IObject, "SFFeatureDefn");
	};

}

