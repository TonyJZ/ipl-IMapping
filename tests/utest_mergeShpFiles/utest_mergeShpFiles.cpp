// utest_mergeShpFiles.cpp : Defines the entry point for the console application.
//

#include "core/iplPlatformUtil.h"
#include "core/interface/IPlatform.h"
#include "spatialreference/interface/ISpatialReferenceService.h"
#include "simplefeature/interface/ISFService.h"
#include "simplefeature/interface/ISFVectorLayer.h"
#include "simplefeature/interface/ISFVectorSource.h"

#include "core/iplcore.h"
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
	printf("utest_mergeShpFiles mergelist.txt output.shp \n");
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

int mergePolygon(std::vector<std::string>  inputList)
{
	for (int iFile = 0; iFile < inputList.size(); ++iFile)
	{
		std::string filename = inputList[iFile];
		ref_ptr<ISFVectorSourceReader> pVectorReader;
		pVectorReader = ref_ptr<ISFVectorSourceReader>(getSFService()->OpenSFFile(filename.c_str(), false));

		ref_ptr<ISFVectorSource> pVectorSource;
		if (gVectorSource.get() == NULL)
		{
			gVectorSource = pVectorReader->GetVectorSource();
			continue;
		}
		else
			pVectorSource = pVectorReader->GetVectorSource();

		ISFVectorLayer *pWriteLayer = gVectorSource->GetLayer(0);

		size_t nLayers = pVectorSource->GetLayerCount();
		size_t nGeoObjs = 0;
		for (size_t i = 0; i < nLayers; ++i)
		{
			ISFVectorLayer *pLayer = pVectorSource->GetLayer(i);
			const char* layerName = pLayer->GetLayerName();

			//pLayer = gVectorSource->GetLayerByName(layerName);

			IPL_wkbGeometryType type = pLayer->GetGeometryType();

			if (type != IPL_wkbPolygon &&
				type != IPL_wkbPolygon25D &&
				type != IPL_wkbMultiPolygon &&
				type != IPL_wkbMultiPolygon25D)
				continue;

// 			ref_ptr<ISpatialReference> pSR = pLayer->GetSpatialReference();
// 
// 			if (pSR.get() != NULL)
// 			{
// 				ref_ptr<IOGRString> wktStr;
// 				pSR->exportToWkt(wktStr);
// 			}

			size_t fnum = pLayer->GetFeatureCount();
// 			iplEnvelope envelope;
// 			pLayer->GetExtent(&envelope);

			//重置, 重新读取feature
			pLayer->ResetReading();
			ISFFeature *feature;
			pLayer->ResetReading();

			while ((feature = pLayer->GetNextFeature()) != NULL)
			{
				pWriteLayer->AppendFeature(feature);
			}

		}
	}

	return 0;
}

int writePolygon(const char *pFileName)
{
	ref_ptr<ISFVectorSourceWriter> pVectorWriter;
	pVectorWriter = ref_ptr<ISFVectorSourceWriter>(getSFService()->CreateSFFile(pFileName, SF_FILE_FORMAT_SHP));

	pVectorWriter->SetVectorSource(gVectorSource);
	pVectorWriter->Close();
	pVectorWriter.reset();

	return 0;
}

//2018.06.04 仅合并文件中的polygon
int main(int argc, char * argv[])
{
	if (argc < 3)
	{
		usage();
	}

	//	assert(false);
	if (strcmp(argv[1], "-h") == 0)
	{
		usage();
	}

	int i = 1;
	//char *pOffsetFile = argv[i]; i++;
	char *pInputFile = argv[i]; i++;
	char *pOutputFile = argv[i]; i++;

	
	for (; i < argc; i++)
	{
		if (strcmp(argv[i], "--org2dst") == 0)
		{
			//type = OFFSET_ORG2DST;
		}
		else if (strcmp(argv[i], "--dst2org") == 0)
		{
			//type = OFFSET_DST2ORG;
		};
	}

	iplString errorinfo;
	g_pPlatform = iplInitialize(errorinfo);

	std::ifstream  ifs;
	ifs.open(pInputFile);

	if (!ifs.is_open())
	{
		std::cout << "can not open " << pInputFile << std::endl;
		return -1;
	}
	std::vector<std::string>  inputList;
	while (!ifs.eof())
	{
		std::string file_desc;
		ifs >> file_desc;
		inputList.push_back(file_desc);
	}
	ifs.close();

	mergePolygon(inputList);

	writePolygon(pOutputFile);

	gVectorSource.reset();
	iplUninitialize();
    return 0;
}

