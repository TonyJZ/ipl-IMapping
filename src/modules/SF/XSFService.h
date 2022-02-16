#pragma once


#include "simplefeature/interface/ISFService.h"

// #include "simplefeature/SFFieldDefn.h"
// #include "simplefeature/SFFeatureDefn.h"



namespace ipl
{
	class XSFService : public ISFService
	{
	public:
		virtual iplFileFormatList getSupposedSFFormats();

		virtual ISFVectorSourceReader  *OpenSFFile(const iplChar *filename, bool bUpdate);
		virtual ISFVectorSourceWriter  *CreateSFFile(const iplChar *pszName, const iplChar *format/*, ref_ptr<ISpatialReference> &pSRS*/);

		//virtual int DeleteDataSource(const iplChar *pszName);
		//     virtual ISFVectorSource *CopyDataSource( ISFVectorSource *poSrcDS, 
		// 		const char *pszNewName, 
		// 		char **papszOptions = NULL );

		virtual ISFVectorSource* createSFVectorSource();
		virtual ISFVectorLayer* createSFVectorLayer();

		ISFFeature* createFeature(ref_ptr<ISFFeatureDefn> poDefnIn);
		ISFFieldDefn *createFieldDefn();
		ISFFeatureDefn *createFeatureDefn();

		IGeoObject* createGeometry(IPL_wkbGeometryType dataType);

		ISFPoint* createPoint();
		ISFLineString* createLineString();
		ISFLineRing* createLinearRing();
		ISFPolygon   * createPolygon();
		ISFGeometryCollection *createGeometryCollection();
		ISFMultiPoint *createMultiPoint();
		ISFMultiLineString *createMultiLineString();
		ISFMultiPolygon *createMultiPolygon();

// 		ISFFieldDefn *CreateFieldDefn()
// 		{
// 			return new XSFFieldDefn;
// 		}


		//////////////////////////////////////////////////////////////////////////

		virtual bool startup(IPlatform *platform);
		virtual void shutdown();
		virtual bool isok();

		//////////////////////////////////////////////////////////////////////////
		//relation of Geo (暂时在这里没有实现，看是否需要)
		virtual int Intersects(IGeoObject *pGeo, IGeoObject *pGeoOther);
		virtual int Equals(IGeoObject *pGeo, IGeoObject *pGeoOther);
		virtual int Disjoint(IGeoObject *pGeo, IGeoObject *pGeoOther);
		virtual int Touches(IGeoObject *pGeo, IGeoObject *pGeoOther);
		virtual int Crosses(IGeoObject *pGeo, IGeoObject *pGeoOther);
		virtual int Within(IGeoObject *pGeo, IGeoObject *pGeoOther);
		virtual int Contains(IGeoObject *pGeo, IGeoObject *pGeoOther);
		virtual int Overlaps(IGeoObject *pGeo, IGeoObject *pGeoOther);
		//Object方法实现
		IPL_OBJECT_IMP2(XSFService, ISFService, IService, "default", "Simple Feature Service")
	};
}

