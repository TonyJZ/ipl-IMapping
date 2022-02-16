// utest_CoordnateTransformSHP.cpp : Defines the entry point for the console application.
//

#include "core/iplPlatformUtil.h"
#include "core/interface/IPlatform.h"
#include "spatialreference/interface/ISpatialReferenceService.h"
#include "simplefeature/interface/ISFService.h"
#include "simplefeature/interface/ISFVectorLayer.h"
#include "simplefeature/interface/ISFVectorSource.h"

#include "core/iplcore.h"
#include "commonAPIs/iplstring.h"
#include "io/CoordinateOffsetFileIO.h"
#include "reconstruction/reconstructionDef.h"

#include "GDAL/gdal_alg.h"
#include "GDAL/gdal_priv.h"   //添加GDAL库函数的头文件
#include "GDAL/ogrsf_frmts.h"

#include <fstream>

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

void usage(bool wait = false)
{
	printf("coordinate transform V1.0: Appropolis Inc.\n");
	printf("usage:\n");
	printf("utest_CoordinateTransformSHP offset.txt translist.txt --org2dst \n");
	printf("---------------------------------------------\n");
	if (wait)
	{
		printf("<press ENTER>\n");
		getc(stdin);
	}
	exit(1);
}

static void byebye(bool wait = false)
{
	if (wait)
	{
		fprintf(stderr, "<press ENTER>\n");
		getc(stdin);
	}
	/*	exit(1);*/
}

using namespace ipl;
static ref_ptr<ISFVectorSource> gVectorSource;
static Eigen::Vector3d offset(0, 0, 0);

int transformSHP(const std::string pInput, const std::string pOutput)
{
	//shp 读写
	ref_ptr<ISFVectorSourceReader> pVectorReader;
	pVectorReader = ref_ptr<ISFVectorSourceReader>(getSFService()->OpenSFFile(pInput.c_str(), false));

	gVectorSource.reset();
	gVectorSource = pVectorReader->GetVectorSource();

	size_t nLayers = gVectorSource->GetLayerCount();
	size_t nGeoObjs = 0;
	for (size_t i = 0; i < nLayers; ++i)
	{
		ISFVectorLayer *pLayer = gVectorSource->GetLayer(i);
		const char* layerName = pLayer->GetLayerName();

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
		//pLayer->ResetReading();

		while ((feature = pLayer->GetNextFeature()) != NULL)
		{
			ISFFeatureDefn* featDefn = feature->GetDefnRef();
			int fid = feature->GetID();

			int nfield = feature->GetFieldCount();
			for (int j = 0; j < nfield; ++j)
			{
				ISFFieldDefn* fDefn = feature->GetFieldDefnRef(j);
				const char *fieldName = fDefn->GetNameRef();

				if (strcmp(fieldName, rec::FieldDefn_ROOF_HEIGHT) == 0)
				{
					SFField *field = feature->GetRawFieldRef(j);
					field->Real += offset[2];
				}
				else if (strcmp(fieldName, rec::FieldDefn_FLOOR_HEIGHT) == 0)
				{
					SFField *field = feature->GetRawFieldRef(j);
					field->Real += offset[2];
				}
				else if (strcmp(fieldName, rec::FieldDefn_PROFILE_HEIGHT) == 0)
				{
					SFField *field = feature->GetRawFieldRef(j);
					field->Real += offset[2];
				}
			}

			IGeoObject *geom;

			geom = feature->GetGeometryRef();

			if (NULL == geom)
				continue;

			IPL_wkbGeometryType gType = geom->getGeometryType();

			//assert(gType == OSF_wkbPolygon);

			int numOfPts;

			if (gType == IPL_wkbPoint)
			{
				ISFPoint *poPoint = static_cast<ISFPoint*>(geom);
				poPoint->x += offset[0];
				poPoint->y += offset[1];
				poPoint->z += offset[2];
			}
			else if (gType == IPL_wkbMultiPoint)
			{
				ISFMultiPoint *poMultiPoint = static_cast<ISFMultiPoint*>(geom);
				int nGeos = poMultiPoint->getNumGeometries();
				for (int iGeo = 0; iGeo < nGeos; ++iGeo)
				{
					ref_ptr<IGeoObject> geoRef = poMultiPoint->getGeometryRef(iGeo);
					ISFPoint *poPoint = static_cast<ISFPoint*>(geoRef.get());
					poPoint->x += offset[0];
					poPoint->y += offset[1];
					poPoint->z += offset[2];
				}
			}
			else if (gType == IPL_wkbLineString || gType == IPL_wkbLineString25D)
			{
				ISFLineString *poLineString = static_cast<ISFLineString*>(geom);
				for (int ipt = 0; ipt < poLineString->getNumPoints(); ++ipt)
				{
					iplPOINT3D pt;
					poLineString->getPoint(ipt, &pt);
					pt.X += offset[0];
					pt.Y += offset[1];
					pt.Z += offset[2];

					poLineString->setPoint(ipt, pt);
				}
			}
			else if (gType == IPL_wkbLinearRing)
			{
				ISFLineRing *poLineString = static_cast<ISFLineRing*>(geom);
				for (int ipt = 0; ipt < poLineString->getNumPoints(); ++ipt)
				{
					iplPOINT3D pt;
					poLineString->getPoint(ipt, &pt);
					pt.X += offset[0];
					pt.Y += offset[1];
					pt.Z += offset[2];

					poLineString->setPoint(ipt, pt);
				}
			}
			else if (gType == IPL_wkbPolygon || gType == IPL_wkbPolygon25D)
			{
				ISFPolygon *poPolygon = static_cast<ISFPolygon*>(geom);
				ref_ptr<ISFLineRing> ring = poPolygon->getExteriorRing();
				for (int ipt = 0; ipt < ring->getNumPoints(); ++ipt)
				{
					iplPOINT3D pt;
					ring->getPoint(ipt, &pt);
					pt.X += offset[0];
					pt.Y += offset[1];
					pt.Z += offset[2];

					ring->setPoint(ipt, pt);
				}

				for (int ir = 0; ir < poPolygon->getNumInteriorRings(); ++ir)
				{
					ref_ptr<ISFLineRing> ring = poPolygon->getInteriorRing(ir);
					for (int ipt = 0; ipt < ring->getNumPoints(); ++ipt)
					{
						iplPOINT3D pt;
						ring->getPoint(ipt, &pt);
						pt.X += offset[0];
						pt.Y += offset[1];
						pt.Z += offset[2];

						ring->setPoint(ipt, pt);
					}
				}

			}
			else if (gType == IPL_wkbMultiLineString || gType == IPL_wkbMultiLineString25D)
			{
				ISFMultiLineString *poMultiLine = static_cast<ISFMultiLineString*>(geom);
				int nGeos = poMultiLine->getNumGeometries();
				for (int iGeo = 0; iGeo < nGeos; ++iGeo)
				{
					ref_ptr<IGeoObject> geoRef = poMultiLine->getGeometryRef(iGeo);
					ISFLineRing *poLineString = static_cast<ISFLineRing*>(geoRef.get());
					for (int ipt = 0; ipt < poLineString->getNumPoints(); ++ipt)
					{
						iplPOINT3D pt;
						poLineString->getPoint(ipt, &pt);
						pt.X += offset[0];
						pt.Y += offset[1];
						pt.Z += offset[2];

						poLineString->setPoint(ipt, pt);
					}
				}

			}
			else if (gType == IPL_wkbMultiPolygon || gType == IPL_wkbMultiPolygon25D)
			{
				ISFMultiPolygon *poMultiPolygon = static_cast<ISFMultiPolygon*>(geom);
				int nGeos = poMultiPolygon->getNumGeometries();
				for (int iGeo = 0; iGeo < nGeos; ++iGeo)
				{
					ref_ptr<IGeoObject> geoRef = poMultiPolygon->getGeometryRef(iGeo);
					ISFPolygon *poPolygon = static_cast<ISFPolygon*>(geoRef.get());
					ref_ptr<ISFLineRing> ring = poPolygon->getExteriorRing();
					for (int ipt = 0; ipt < ring->getNumPoints(); ++ipt)
					{
						iplPOINT3D pt;
						ring->getPoint(ipt, &pt);
						pt.X += offset[0];
						pt.Y += offset[1];
						pt.Z += offset[2];

						ring->setPoint(ipt, pt);
					}

					for (int ir = 0; ir < poPolygon->getNumInteriorRings(); ++ir)
					{
						ref_ptr<ISFLineRing> ring = poPolygon->getInteriorRing(ir);
						for (int ipt = 0; ipt < ring->getNumPoints(); ++ipt)
						{
							iplPOINT3D pt;
							ring->getPoint(ipt, &pt);
							pt.X += offset[0];
							pt.Y += offset[1];
							pt.Z += offset[2];

							ring->setPoint(ipt, pt);
						}
					}
				}
			}

			nGeoObjs++;
		}

	}

	pVectorReader->Close();
	pVectorReader.reset();

	ref_ptr<ISFVectorSourceWriter> pVectorWriter;
	pVectorWriter = ref_ptr<ISFVectorSourceWriter>(getSFService()->CreateSFFile(pOutput.c_str(), SF_FILE_FORMAT_SHP));

	pVectorWriter->SetVectorSource(gVectorSource);
	pVectorWriter->Close();
	pVectorWriter.reset();

	return 0;
}

int main(int argc, char * argv[])
{
	if (argc < 4)
	{
		usage();
	}

	//	assert(false);
	if (strcmp(argv[1], "-h") == 0)
	{
		usage();
	}

	int i = 1;
	char *pOffsetFile = argv[i]; i++;
	char *pInputFile = argv[i]; i++;
//	char *pOutputFile = argv[i]; i++;

	enum OffsetType
	{
		OFFSET_UNKNOWN = 0,
		OFFSET_ORG2DST = 1,
		OFFSET_DST2ORG
	};

	OffsetType type = OFFSET_UNKNOWN;
	for (; i < argc; i++)
	{
		if (strcmp(argv[i], "--org2dst") == 0)
		{
			type = OFFSET_ORG2DST;
		}
		else if (strcmp(argv[i], "--dst2org") == 0)
		{
			type = OFFSET_DST2ORG;
		};
	}

	iplString errorinfo;
	g_pPlatform = iplInitialize(errorinfo);
	
	Eigen::Vector3d org2dst, dst2org;
	
	load_OffsetParam(pOffsetFile, org2dst, dst2org);
	if (type == OFFSET_ORG2DST)
		offset = org2dst;
	else if (type == OFFSET_DST2ORG)
		offset = dst2org;

	std::ifstream  ifs;
	ifs.open(pInputFile);

	if (!ifs.is_open())
	{
		std::cout << "can not open " << pInputFile << std::endl;
		return -1;
	}

	std::string file_desc;
	int fileNum;
	ifs >> fileNum;

	std::vector<std::string>  inputList, outputList;
	for (int i = 0; i < fileNum; ++i)
	{
		ifs >> file_desc;
		inputList.push_back(file_desc);

		std::string result_name, suffix_name;
		extract_file_name(file_desc, result_name, suffix_name);

		std::string offsetASName = result_name + rec::BBName_Offset + suffix_name;
		outputList.push_back(offsetASName);
	}

	ifs.close();

	for (int i = 0; i < fileNum; ++i)
	{
		transformSHP(inputList[i], outputList[i]);
//		break;
	}

	gVectorSource.reset();
	iplUninitialize();
    return 0;
}

