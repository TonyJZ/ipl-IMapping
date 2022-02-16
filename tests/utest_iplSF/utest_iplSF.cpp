// utest_SF.cpp : Defines the entry point for the console application.
//
#include "core/iplPlatformUtil.h"
#include "core/interface/IPlatform.h"
#include "spatialreference/interface/ISpatialReferenceService.h"
#include "simplefeature/interface/ISFService.h"
#include "simplefeature/interface/ISFVectorLayer.h"
#include "simplefeature/interface/ISFVectorSource.h"

ipl::IPlatform *g_pPlatform = NULL;

ipl::IPlatform *ipl::getPlatform()
{
	return g_pPlatform;
}

namespace ipl
{
	IPL_GET_SRS_SERVICE_IMPL();
	IPL_GET_SF_SERVICE_IMPL();
}

using namespace ipl;

static ref_ptr<ISFVectorSource> gVectorSource;

void readVector(const char *input, const char *output)
{
	ref_ptr<ISFVectorSourceReader> pVectorReader;
	pVectorReader = ref_ptr<ISFVectorSourceReader>(getSFService()->OpenSFFile(input, false));

	gVectorSource.reset();
	gVectorSource = pVectorReader->GetVectorSource();

	size_t nLayers = gVectorSource->GetLayerCount();
	size_t nGeoObjs = 0;
	for (size_t i = 0; i < nLayers; ++i)
	{
		ISFVectorLayer *pLayer = gVectorSource->GetLayer(i);
		const char* layerName = pLayer->GetLayerName();

		pLayer = gVectorSource->GetLayerByName(layerName);

		IPL_wkbGeometryType type = pLayer->GetGeometryType();

		ref_ptr<ISpatialReference> pSR = pLayer->GetSpatialReference();

		if (pSR.get() != NULL)
		{
			ref_ptr<IOGRString> wktStr;
			pSR->exportToWkt(wktStr);
		}

		size_t fnum = pLayer->GetFeatureCount();
		iplEnvelope envelope;
		pLayer->GetExtent(&envelope);

		//重置, 重新读取feature
		pLayer->ResetReading();
		ISFFeature *feature;
		pLayer->ResetReading();

		while ((feature = pLayer->GetNextFeature()) != NULL)
		{
			ISFFeatureDefn* featDefn = feature->GetDefnRef();
			int fid = feature->GetID();

			int nfield = feature->GetFieldCount();
			for (int j = 0; j < nfield; ++j)
			{
				ISFFieldDefn* fDefn = feature->GetFieldDefnRef(j);
				SFField *field = feature->GetRawFieldRef(j);
			}

			IGeoObject *geom;

			geom = feature->GetGeometryRef();

			if (NULL == geom)
				continue;

			IPL_wkbGeometryType gType = geom->getGeometryType();

			//assert(gType == OSF_wkbPolygon);

			int numOfPts;

			if (gType == IPL_wkbLineString || gType == IPL_wkbLineString25D)
			{
				ISFLineString *poLineString = static_cast<ISFLineString*>(geom);
			}
			else if (gType == IPL_wkbLinearRing)
			{
				ISFLineRing *poLineString = static_cast<ISFLineRing*>(geom);
			}
			else if (gType == IPL_wkbPolygon || gType == IPL_wkbPolygon25D)
			{
				ISFPolygon *poPolygon = static_cast<ISFPolygon*>(geom);
			}
			else if (gType == IPL_wkbMultiLineString || gType == IPL_wkbMultiLineString25D)
			{
				ISFMultiLineString *poMultiLine = static_cast<ISFMultiLineString*>(geom);
			}
			else if (gType == IPL_wkbMultiPolygon || gType == IPL_wkbMultiPolygon25D)
			{
				ISFMultiPolygon *poMultiPolygon = static_cast<ISFMultiPolygon*>(geom);
			}

			nGeoObjs++;
		}

	}

	pVectorReader->Close();
	pVectorReader.reset();

	ref_ptr<ISFVectorSourceWriter> pVectorWriter;
	pVectorWriter = ref_ptr<ISFVectorSourceWriter>(getSFService()->CreateSFFile(output, SF_FILE_FORMAT_SHP));

	pVectorWriter->SetVectorSource(gVectorSource);
	pVectorWriter->Close();
	pVectorWriter.reset();
	
}

void CreateVectorSource(const char* filename)
{
	char wktStr[2048];

	sprintf(wktStr, "PROJCS[\"WGS 84 / UTM zone %d%s\", \
																   GEOGCS[\"WGS 84\", \
																   DATUM[\"WGS_1984\", \
																   SPHEROID[\"WGS 84\", 6378137, 298.257223563, \
																   AUTHORITY[\"EPSG\", \"7030\"]], \
																   AUTHORITY[\"EPSG\", \"6326\"]], \
																   PRIMEM[\"Greenwich\", 0, \
																   AUTHORITY[\"EPSG\", \"8901\"]], \
																   UNIT[\"degree\", 0.0174532925199433, \
																   AUTHORITY[\"EPSG\", \"9122\"]], \
																   AUTHORITY[\"EPSG\", \"4326\"]], \
																   PROJECTION[\"Transverse_Mercator\"], \
																   PARAMETER[\"latitude_of_origin\", 0], \
																   PARAMETER[\"central_meridian\", %d], \
																   PARAMETER[\"scale_factor\", 0.9996], \
																   PARAMETER[\"false_easting\", 500000], \
																   PARAMETER[\"false_northing\", 0], \
																   UNIT[\"metre\", 1, \
																   AUTHORITY[\"EPSG\", \"9001\"]]]",
		// AUTHORITY[\"EPSG\", \"32650\"]] ",		// 这是标准的UTM 50N代码，不能加
		0, 0, 0);

	//服务的调用方式 1 需要定义IPL_GET_SRS_SERVICE_IMPL
	ISpatialReferenceService *g_pSRService = getSRService();


	//可执行对象的调用方式1 通过服务创建
	ref_ptr<ISpatialReference> geoSR = ref_ptr<ISpatialReference>(g_pSRService->CreateSpatialReference(wktStr));
	
	ref_ptr<ISFVectorSource> pSFSource;

	bool bUseMultiLayer = false;

	pSFSource = ref_ptr<ISFVectorSource>(getSFService()->createSFVectorSource());
	{
		if (pSFSource.get() == NULL)
			return;

		if (!bUseMultiLayer)
		{
			ref_ptr<ISFVectorLayer> pVecLayer;

			//_T("polygon layer"), IPL_wkbPolygon25D
			pVecLayer = ref_ptr<ISFVectorLayer>(getSFService()->createSFVectorLayer());

			pVecLayer->SetLayerName(_T("polygon layer"));
			pVecLayer->SetGeometryType(IPL_wkbPolygon25D);

			ref_ptr<ISFFeatureDefn>   featDefn = ref_ptr<ISFFeatureDefn>(getSFService()->createFeatureDefn());

			ref_ptr<ISFFieldDefn> oFieldOrder = ref_ptr<ISFFieldDefn>(getSFService()->createFieldDefn());
			

			oFieldOrder->Set("name", OFTString, 32, 0, SF_JUndefined);
			featDefn->AddFieldDefn(oFieldOrder->Clone());
			
			
			oFieldOrder->Set("PolygonID", OFTInteger, 8, 0, SF_JUndefined);
			featDefn->AddFieldDefn(oFieldOrder->Clone());

			pVecLayer->SetFeatureDefn(featDefn);
			
			ref_ptr<ISFFeature> feature = ref_ptr<ISFFeature>(getSFService()->createFeature(featDefn));
			SFField sf;
			sf.String = "outter";
			feature->SetField(0, &sf);
			sf.Integer = 1;
			feature->SetField(1, &sf);

			ref_ptr<ISFPolygon> poly = ref_ptr<ISFPolygon>(getSFService()->createPolygon());
			ref_ptr<ISFLineRing> ring = ref_ptr<ISFLineRing>(getSFService()->createLinearRing());

			iplPOINT3D pt;
			pt.X = -50, pt.Y = -50;
			ring->addPoint(pt);
			pt.X = 50, pt.Y = -50;
			ring->addPoint(pt);
			pt.X = 50, pt.Y = 50;
			ring->addPoint(pt);
			pt.X = -50, pt.Y = 50;
			ring->addPoint(pt);
			ring->closeRing();
			poly->addRing(ring);

			ring = NULL;
			ring = ref_ptr<ISFLineRing>(getSFService()->createLinearRing());
			pt.X = -30, pt.Y = -30;
			ring->addPoint(pt);
			pt.X = 30, pt.Y = -30;
			ring->addPoint(pt);
			pt.X = 30, pt.Y = 30;
			ring->addPoint(pt);
			pt.X = -30, pt.Y = 30;
			ring->addPoint(pt);
			ring->closeRing();
			poly->addRing(ring);


			feature->SetGeometry(poly);
			//SetGeometry(const IGeoObject * poGeomIn);
			pVecLayer->AppendFeature(feature.get());
			pVecLayer->SetSpatialReference(geoSR);

			pSFSource->addLayer(pVecLayer);
		}
		else
		{
			;
		}
	}

	ref_ptr<ISFVectorSourceWriter> pVectorWriter;
	pVectorWriter = ref_ptr<ISFVectorSourceWriter>(getSFService()->CreateSFFile(filename, SF_FILE_FORMAT_SHP));

	pVectorWriter->SetVectorSource(pSFSource);
	pVectorWriter->Close();
}

void geoAnalyst()
{
	if (gVectorSource.get() == NULL)
		return;
	
	size_t nLayers = gVectorSource->GetLayerCount();
	size_t nGeoObjs = 0;
	for (size_t i = 0; i < nLayers; ++i)
	{
		ISFVectorLayer *pLayer = gVectorSource->GetLayer(i);
		

		size_t fnum = pLayer->GetFeatureCount();

		if(fnum <= 1)
			continue;
	
		//重置, 重新读取feature
		pLayer->ResetReading();
		ISFFeature *feature = pLayer->GetNextFeature();
		if(feature == NULL)
			continue;

		IGeoObject *pObj1 = feature->GetGeometryRef();
		size_t nfield = feature->GetFieldCount();
		for (int j = 0; j < nfield; ++j)
		{
			SFField  *fd = feature->GetRawFieldRef(j);
		}

		while ((feature = pLayer->GetNextFeature()) != NULL)
		{
			IGeoObject *pObj2 = feature->GetGeometryRef();
			for (int j = 0; j < nfield; ++j)
			{
				SFField  *fd = feature->GetRawFieldRef(j);
			}

			int ret = getSFService()->Touches(pObj1, pObj2);

			printf("touch: %d\n", ret);
		}

	}
}

//1. read / write
//2. spatial analyst
//3. create vector source by users

//char input[] = "D:\\iplTestData\\VectorMap\\province\\bou2_4p.shp";
//char output[] = "D:\\iplTestData\\utest\\test_SF\\bou2_4p_writer.shp";
//char createfile[] = "D:\\iplTestData\\utest\\test_SF\\createfile.shp";

int main(int argc, char * argv[])
{
	iplString errorinfo;
	g_pPlatform = iplInitialize(errorinfo);

	char *input = argv[1];
	char *output = argv[2];
	char *createfile = argv[3];

	readVector(input, output);
	geoAnalyst();
	CreateVectorSource(createfile);
	

	gVectorSource.reset();
	iplUninitialize();
    return 0;
}

