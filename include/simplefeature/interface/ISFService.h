#pragma once

#include "core/interface/IPlatform.h"
#include "core/interface/IService.h"
#include "simplefeature/SFBaseDef.h"
#include "simplefeature/interface/ISFFeature.h"
#include "simplefeature/interface/ISFGeometry.h"
#include "simplefeature/interface/ISFVectorSource.h"
#include "simplefeature/interface/ISFVectorSourceReader.h"
#include "simplefeature/interface/ISFVectorSourceWriter.h"

namespace ipl
{
	class ISFService : public IService
	{
	public:
		virtual iplFileFormatList getSupposedSFFormats() = 0;

		virtual ISFVectorSourceReader* OpenSFFile(const iplChar *filename, bool bUpdate) = 0;
		virtual ISFVectorSourceWriter* CreateSFFile(const iplChar *pszName, const iplChar *format/*, ref_ptr<ISpatialReference> &pSRS*/) = 0;

		//virtual int DeleteDataSource(const char *pszName) = 0;

		//     virtual ISFVectorSource *CopyDataSource( ISFVectorSource *poSrcDS, 
		// 		const char *pszNewName, 
		// 		char **papszOptions ) = 0;

		//relation of Geo
		virtual int Intersects(IGeoObject *pGeo, IGeoObject *pGeoOther) = 0;
		virtual int Equals(IGeoObject *pGeo, IGeoObject *pGeoOther) = 0;
		virtual int Disjoint(IGeoObject *pGeo, IGeoObject *pGeoOther) = 0;
		virtual int Touches(IGeoObject *pGeo, IGeoObject *pGeoOther) = 0;
		virtual int Crosses(IGeoObject *pGeo, IGeoObject *pGeoOther) = 0;
		virtual int Within(IGeoObject *pGeo, IGeoObject *pGeoOther) = 0;
		virtual int Contains(IGeoObject *pGeo, IGeoObject *pGeoOther) = 0;
		virtual int Overlaps(IGeoObject *pGeo, IGeoObject *pGeoOther) = 0;


		virtual ISFVectorSource* createSFVectorSource() = 0;
		virtual ISFVectorLayer* createSFVectorLayer() = 0;

		virtual ISFFeature* createFeature(ref_ptr<ISFFeatureDefn> poDefnIn) = 0;

		virtual ISFFieldDefn *createFieldDefn() = 0;

		virtual ISFFeatureDefn *createFeatureDefn() = 0;

		virtual IGeoObject* createGeometry(IPL_wkbGeometryType dataType) = 0;

		virtual ISFPoint* createPoint() = 0;

		virtual ISFLineString* createLineString() = 0;
		virtual ISFLineRing* createLinearRing() = 0;

		virtual ISFPolygon   * createPolygon() = 0;

		virtual ISFGeometryCollection *createGeometryCollection() = 0;
		virtual ISFMultiPoint *createMultiPoint() = 0;
		virtual ISFMultiLineString *createMultiLineString() = 0;
		virtual ISFMultiPolygon *createMultiPolygon() = 0;

		virtual bool startup(IPlatform *platform) = 0;
		virtual void shutdown() = 0;
		virtual bool isok() = 0;

		IPL_INTERFACE_DEF(IService, "SF")
	};

#define IPL_SERVICE_SF			"ipl.service.SF"
#define IPL_SERVICE_SF_DEFAULT	"ipl.service.SF.default"




	// 获取日志服务的宏定义
	ISFService *getSFService();

#define IPL_GET_SF_SERVICE_IMPL()	\
	static ISFService *s_sfService = NULL;\
	ISFService *getSFService()\
{\
	if( NULL != s_sfService )\
	return s_sfService;\
	\
	s_sfService =\
	IPL_PTR_CAST( ISFService, getPlatform()->getService( IPL_SERVICE_SF) );\
	\
	return s_sfService;\
}


}
