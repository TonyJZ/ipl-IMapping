#pragma once

#include "core/ipldef.h"

#include "simplefeature/SFBaseDef.h"
//#include "simplefeature/interface/SFIGeometry.h"
#include "simplefeature/interface/ISFFeatureDefn.h"
#include "simplefeature/interface/ISFFieldDefn.h"
#include "geometry/interface/IGeoObject.h"


namespace ipl
{

	interface ISFFeatureDefn;
	interface ISFFieldDefn;

	/************************************************************************/
	/*                              ISFFeature                              */
	/************************************************************************/

	/**
	* A simple feature, including geometry and attributes.
	*/

	interface ISFFeature : public IObject
	{
	public:
		virtual int IsFieldSet(int iField) const = 0;
		// 	virtual const char *GetStyleString() = 0;
		// 	virtual void SetStyleString(const char* pszString) = 0;

		virtual long GetID() const = 0;
		virtual SFERR SetID(long id) = 0;

		virtual void SetField(int i, SFField * puValue) = 0;

		virtual ISFFeatureDefn     *GetDefnRef() = 0;

		virtual SFERR              SetGeometry(const ref_ptr<IGeoObject> poGeomIn) = 0;
		virtual SFERR              SetGeometryDirectly(ref_ptr<IGeoObject> poGeomIn) = 0;

		virtual IGeoObject        *GetGeometryRef() = 0;

		virtual ref_ptr<IGeoObject> StealGeometry() = 0;

		virtual ISFFeature         *Clone() const = 0;

		virtual size_t                 GetFieldCount() = 0;

		virtual ISFFieldDefn       *GetFieldDefnRef(int iField) = 0;

		virtual int                 GetFieldIndex(const char * pszName) = 0;

		virtual SFField           *GetRawFieldRef(int i) = 0;

		IPL_INTERFACE_DEF(IObject, "SFFeature");
	};

}

