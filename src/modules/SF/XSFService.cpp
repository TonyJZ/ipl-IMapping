#include "XSFService.h"
#include "XSFFeature.h"
#include "XSFFieldDefn.h"
#include "XSFFeatureDefn.h"
#include "XSFPoint.h"
#include "XSFLineRing.h"
#include "XSFPolygon.h"
#include "XSFGeometryCollection.h"
#include "XSFMultiPoint.h"
#include "XSFMultiLineString.h"
#include "XSFMultiPolygon.h"
#include "XSFOGRVectorSource.h"
#include "XSFOGRVectorLayer.h"

#include "core/interface/IPlatform.h"
#include "core/interface/IRegisterService.h"

#include "simplefeature/SF2OGR.h"

extern ipl::IPlatform* ipl::getPlatform();

using namespace ipl;
ISFVectorSourceReader *XSFService::OpenSFFile( const iplChar *filename, bool bUpdate )
{
	ref_ptr<IRegisterService> registerService = getPlatform()->getRegisterService();

	iplArray<IObjectDesc* > descriptes =
		registerService->getObjectDescsByInterface("IVectorSourceReader");

	if( descriptes.size() == 0 )	{
		getPlatform()->logPrint( IPL_LOG_INFO, "Can not find IVectorSourceReader" );
		return NULL;
	}

	for(int i=0;i<descriptes.size();i++)
	{
		IObjectDesc *desc = descriptes[i];

		ISFVectorSourceReader *reader = IPL_CREATE_OBJECT(ISFVectorSourceReader,desc->getID());

		if( reader != NULL && reader->Open( filename, bUpdate ) ) 
		{
			return reader;
		}

		if (reader != NULL)//the format is not supported 
			delete reader;
	}

	return NULL;
}

ISFVectorSourceWriter *XSFService::CreateSFFile(const iplChar *pszName, const iplChar *format/*, ref_ptr<ISpatialReference> &pSRS*/)
{
	IRegisterService *registerService = getPlatform()->getRegisterService().get();

	iplArray<IObjectDesc* > descriptes =
		registerService->getObjectDescsByInterface("ISFVectorSourceWriter");

	if (descriptes.size() == 0) {
		getPlatform()->logPrint(IPL_LOG_INFO, "Can not find ISFVectorSourceWriter");
		return NULL;
	}

	for (int i = 0; i < descriptes.size(); i++)
	{
		IObjectDesc *desc = descriptes[i];

		ISFVectorSourceWriter *writer = IPL_CREATE_OBJECT(ISFVectorSourceWriter, desc->getID());

		if (writer != NULL) {
			if (writer->Create(pszName, (char *)format/*, pSRS*/))
				return writer;
		}

		if (writer != NULL)//the format is not supported 
			delete writer;
	}

	return NULL;
}

ISFVectorSource* XSFService::createSFVectorSource()
{
	return new XSFOGRVectorSource;
}

ISFVectorLayer* XSFService::createSFVectorLayer()
{
	return new XSFOGRVectorLayer;
}

iplFileFormatList XSFService::getSupposedSFFormats()
{
	iplFileFormat f;
	iplFileFormatList l;

	f.name = "Shape File";	f.ext = "shp";	l.push_back( f );
	f.name = "kml file";	f.ext = "kml";	l.push_back( f );

	return l;
}

// int XSFService::DeleteDataSource( const iplChar *pszName )
// {
// 	IRegisterService *registerService = getPlatform()->getRegisterService().get();
// 
// 	iplArray<IObjectDesc* > descriptes =
// 		registerService->getObjectDescsByInterface("ISFVectorSource");
// 
// 	if (descriptes.size() == 0) {
// 		getPlatform()->logPrint(IPL_LOG_INFO, "Can not find osfIVectorSource");
// 		return NULL;
// 	}
// 
// 	for (int i = 0; i < descriptes.size(); i++)
// 	{
// 		IObjectDesc *desc = descriptes[i];
// 
// 		ISFVectorSource *reader = IPL_CREATE_OBJECT(ISFVectorSource, desc->getID());
// 
// 		if (reader != NULL) {
// 			return reader->DeleteDataSource(pszName);
// 		}
// 	}
// 	return SFERR_FAILURE;
// }

// SFOGRVectorSource *XSFService::CopyDataSource( SFOGRVectorSource *poSrcDS,
// 		const char *pszNewName,
// 		char **papszOptions )
// {
// 	return NULL;
// }

bool XSFService::startup(IPlatform *platform)
{
    return true;
}

void ipl::XSFService::shutdown()
{
    ;
}
bool ipl::XSFService::isok()
{
	return true;
}

//////////////////////////////////////////////////////////////////////////

IGeoObject *XSFService::createGeometry(IPL_wkbGeometryType dataType)
{
	switch( dataType )
	{
	case IPL_wkbUnknown:
		return NULL;

	case IPL_wkbPoint:
		return createPoint();

	case IPL_wkbPoint25D:
		return createPoint();

	case IPL_wkbLineString:
		return createLineString();

	case IPL_wkbLineString25D:
		return createLineString();

	case IPL_wkbLinearRing:
		return createLinearRing();

	case IPL_wkbPolygon:
		return createPolygon();

	case IPL_wkbPolygon25D:
		return createPolygon();

	case IPL_wkbMultiPoint:
		return createMultiPoint();

	case IPL_wkbMultiPoint25D:
		return createMultiPoint();

	case IPL_wkbMultiLineString:
		return createMultiLineString();

	case IPL_wkbMultiLineString25D:
		return createMultiLineString();

	case IPL_wkbMultiPolygon:
		return createMultiPolygon();

	case IPL_wkbMultiPolygon25D:
		return createMultiPolygon();

	case IPL_wkbGeometryCollection:
		return createGeometryCollection();

	case IPL_wkbGeometryCollection25D:
		return createGeometryCollection();

	case IPL_wkbNone:
		return NULL;

	default:
		{
			return NULL;
		}
	}
}

ISFFeature* XSFService::createFeature(ref_ptr<ISFFeatureDefn> poDefnIn)
{
	return new XSFFeature(poDefnIn);
}

ISFFieldDefn *XSFService::createFieldDefn()
{
	return new XSFFieldDefn;
}

ISFFeatureDefn *XSFService::createFeatureDefn()
{
	return new XSFFeatureDefn;
}

//IGeoObject* createGeometry(SF_wkbGeometryType dataType);
ISFPoint* XSFService::createPoint()
{
	return new XSFPoint;
}

ISFLineString* XSFService::createLineString()
{
	XSFLineRing *ring = new XSFLineRing;
	ring->SetAsRing(false);

	return ring;
}

ISFLineRing* XSFService::createLinearRing()
{
	return new XSFLineRing;
}

ISFPolygon* XSFService::createPolygon()
{
	return new XSFPolygon;
}

ISFGeometryCollection* XSFService::createGeometryCollection()
{
	return new XSFGeometryCollection;
}

ISFMultiPoint* XSFService::createMultiPoint()
{
	return new XSFMultiPoint;
}
ISFMultiLineString* XSFService::createMultiLineString()
{
	return new XSFMultiLineString;
}
ISFMultiPolygon* XSFService::createMultiPolygon()
{
	return new XSFMultiPolygon;
}


//////////////////////////////////////////////////////////////////////////
//relation of geos 
int XSFService::Intersects (IGeoObject *pGeo,IGeoObject *pGeoOther)
{
	if (pGeo == NULL ||pGeoOther == NULL)
		return true;

	iplEnvelope oEnv1,oEnv2;

	pGeo->getEnvelope(&oEnv1);
	pGeoOther->getEnvelope(&oEnv2);

	if( oEnv1.maxX < oEnv2.minX
        || oEnv1.maxY < oEnv2.minY
        || oEnv2.maxX < oEnv1.minX
        || oEnv2.maxY < oEnv1.minY )
        return false;

	OGRGeometryH pOgrGeom;
	OGRGeometryH pOgrGeomOther;

	pOgrGeom = CreateOgrGeom(pGeo);
	pOgrGeomOther = CreateOgrGeom(pGeoOther);

	if (NULL == pOgrGeom ||NULL == pOgrGeomOther)
		return 0;

	int st = OGR_G_Intersect( pOgrGeom, pOgrGeomOther );

	OGRFree( pOgrGeom );
	OGRFree( pOgrGeomOther );

	return st;
}

int XSFService::Equals (IGeoObject *pGeo,IGeoObject *pGeoOther)
{
	if (pGeo == NULL ||pGeoOther == NULL)
		return true;

	iplEnvelope oEnv1,oEnv2;

	pGeo->getEnvelope(&oEnv1);
	pGeoOther->getEnvelope(&oEnv2);

	if( oEnv1.maxX < oEnv2.minX
        || oEnv1.maxY < oEnv2.minY
        || oEnv2.maxX < oEnv1.minX
        || oEnv2.maxY < oEnv1.minY )
        return false;

	OGRGeometryH pOgrGeom, pOgrGeomOther;
	pOgrGeom = CreateOgrGeom(pGeo);

	pOgrGeomOther = CreateOgrGeom(pGeoOther);
	if (NULL == pOgrGeom ||NULL == pOgrGeomOther)
		return 0;

	int st = OGR_G_Equals( pOgrGeom, pOgrGeomOther);

	OGRFree( pOgrGeom );
	OGRFree( pOgrGeomOther );

	return st;
}

int XSFService::Disjoint (IGeoObject *pGeo,IGeoObject *pGeoOther)
{
	if (pGeo == NULL ||pGeoOther == NULL)
		return true;

	iplEnvelope oEnv1,oEnv2;

	pGeo->getEnvelope(&oEnv1);
	pGeoOther->getEnvelope(&oEnv2);

	if( oEnv1.maxX < oEnv2.minX
        || oEnv1.maxY < oEnv2.minY
        || oEnv2.maxX < oEnv1.minX
        || oEnv2.maxY < oEnv1.minY )
        return true;

	OGRGeometryH pOgrGeom, pOgrGeomOther;
	pOgrGeom = CreateOgrGeom(pGeo);
	pOgrGeomOther = CreateOgrGeom(pGeoOther);
	if (NULL == pOgrGeom ||NULL == pOgrGeomOther)
		return 0;

	int st = OGR_G_Disjoint( pOgrGeom, pOgrGeomOther);
	
	OGRFree( pOgrGeom );
	OGRFree( pOgrGeomOther );

	return st;
}

int XSFService::Touches (IGeoObject *pGeo,IGeoObject *pGeoOther)
{
	if (pGeo == NULL ||pGeoOther == NULL)
		return true;

	iplEnvelope oEnv1,oEnv2;

	pGeo->getEnvelope(&oEnv1);
	pGeoOther->getEnvelope(&oEnv2);

	if( oEnv1.maxX < oEnv2.minX
        || oEnv1.maxY < oEnv2.minY
        || oEnv2.maxX < oEnv1.minX
        || oEnv2.maxY < oEnv1.minY )
        return false;

	OGRGeometryH pOgrGeom, pOgrGeomOther;

	pOgrGeom = CreateOgrGeom(pGeo);
	pOgrGeomOther = CreateOgrGeom(pGeoOther);

	int st = OGR_G_Touches( pOgrGeom, pOgrGeomOther );

	if (NULL == pOgrGeom ||NULL == pOgrGeomOther)
		return 0;

	OGRFree( pOgrGeom );
	OGRFree( pOgrGeomOther );

	return st;
}

int XSFService::Crosses (IGeoObject *pGeo,IGeoObject *pGeoOther)
{
	if (pGeo == NULL ||pGeoOther == NULL)
		return true;

	iplEnvelope oEnv1,oEnv2;

	pGeo->getEnvelope(&oEnv1);
	pGeoOther->getEnvelope(&oEnv2);

	if( oEnv1.maxX < oEnv2.minX
        || oEnv1.maxY < oEnv2.minY
        || oEnv2.maxX < oEnv1.minX
        || oEnv2.maxY < oEnv1.minY )
        return false;

	OGRGeometryH pOgrGeom, pOgrGeomOther;
	
	pOgrGeom = CreateOgrGeom(pGeo);
	pOgrGeomOther = CreateOgrGeom(pGeoOther);

	if (NULL == pOgrGeom ||NULL == pOgrGeomOther)
		return 0;

	int st = OGR_G_Crosses( pOgrGeom, pOgrGeomOther);
	
	OGRFree( pOgrGeom );
	OGRFree( pOgrGeomOther );
	
	return st;
}

int XSFService::Within (IGeoObject *pGeo,IGeoObject *pGeoOther)
{
	if (pGeo == NULL ||pGeoOther == NULL)
		return true;

	iplEnvelope oEnv1,oEnv2;

	pGeo->getEnvelope(&oEnv1);
	pGeoOther->getEnvelope(&oEnv2);

	if( oEnv1.maxX < oEnv2.minX
        || oEnv1.maxY < oEnv2.minY
        || oEnv2.maxX < oEnv1.minX
        || oEnv2.maxY < oEnv1.minY )
        return false;

	OGRGeometryH pOgrGeom, pOgrGeomOther;
	pOgrGeom = CreateOgrGeom(pGeo);
	pOgrGeomOther = CreateOgrGeom(pGeoOther);

	int st = OGR_G_Within( pOgrGeom, pOgrGeomOther);
	
	if (NULL == pOgrGeom ||NULL == pOgrGeomOther)
		return 0;

	OGRFree( pOgrGeom );
	OGRFree( pOgrGeomOther );

	return st;
}


int XSFService::Contains (IGeoObject *pGeo,IGeoObject *pGeoOther)
{
	if (pGeo == NULL ||pGeoOther == NULL)
		return true;

	iplEnvelope oEnv1,oEnv2;

	pGeo->getEnvelope(&oEnv1);
	pGeoOther->getEnvelope(&oEnv2);

	if( oEnv1.maxX < oEnv2.minX
        || oEnv1.maxY < oEnv2.minY
        || oEnv2.maxX < oEnv1.minX
        || oEnv2.maxY < oEnv1.minY )
        return false;

	OGRGeometryH pOgrGeom, pOgrGeomOther;
	
	pOgrGeom = CreateOgrGeom(pGeo);
	pOgrGeomOther = CreateOgrGeom(pGeoOther);

	if (NULL == pOgrGeom ||NULL == pOgrGeomOther)
		return 0;

	int st = OGR_G_Contains( pOgrGeom, pOgrGeomOther);
	
	OGRFree( pOgrGeom );
	OGRFree( pOgrGeomOther );	

	return st;
}
int XSFService::Overlaps (IGeoObject *pGeo,IGeoObject *pGeoOther)
{
	if (pGeo == NULL ||pGeoOther == NULL)
		return true;

	iplEnvelope oEnv1,oEnv2;

	pGeo->getEnvelope(&oEnv1);
	pGeoOther->getEnvelope(&oEnv2);

	if( oEnv1.maxX < oEnv2.minX
        || oEnv1.maxY < oEnv2.minY
        || oEnv2.maxX < oEnv1.minX
        || oEnv2.maxY < oEnv1.minY )
        return false;

	OGRGeometryH pOgrGeom;
	OGRGeometryH pOgrGeomOther;

	pOgrGeom = CreateOgrGeom(pGeo);
	pOgrGeomOther = CreateOgrGeom(pGeoOther);

	if (NULL == pOgrGeom ||NULL == pOgrGeomOther)
		return 0;

	int st = OGR_G_Overlaps( pOgrGeom, pOgrGeomOther);

	OGRFree( pOgrGeom );
	OGRFree( pOgrGeomOther );	

	return st;
}

