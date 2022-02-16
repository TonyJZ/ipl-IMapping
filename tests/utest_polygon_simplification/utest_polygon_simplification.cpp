// utest_polygon_simplification.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include <GDAL/gdal_alg.h>
#include <GDAL/gdal_priv.h>   
#include <GDAL/ogrsf_frmts.h>

#include "geometry/polygon/PolygonSimplification.h"

void usage(bool wait = false)
{
	printf("polygon simplification V1.0: Appropolis Inc.\n");
	printf("usage:\n");
	printf("utest_polygon_simplification input.shp output_shp dTolerance\n");
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
	char *pInput = argv[i]; i++;
	char *pOutput = argv[i]; i++;
	double dTolerance = atof(argv[i]); i++;
// 	char *pfloorfile = argv[i]; i++;
// 	double floor_hei = 5.0;

	GDALAllRegister();
	GDALDataset       *poDS;
	poDS = (GDALDataset*)GDALOpenEx(pInput, GDAL_OF_VECTOR, NULL, NULL, NULL);
	if (poDS == NULL)
	{
		printf("Open  /'%s/' failed.\n", pInput);
		exit(1);
	}

	const char *pszDriverName = "ESRI Shapefile";
	GDALDriver *poDriver;
	
	poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
	if (poDriver == NULL)
	{
		printf("%s driver not available.\n", pszDriverName);
		exit(1);
	}
	GDALDataset *poSimpDS;   //简化后的矢量图层
	poSimpDS = poDriver->Create(pOutput, 0, 0, 0, GDT_Unknown, NULL); //创建数据源
	if (poSimpDS == NULL)
	{
		printf("Creation of /'%s/' file failed.\n", pOutput);
		exit(1);
	}

	int nLayer = poDS->GetLayerCount();
	for (int i = 0; i < nLayer; i++)
	{
		OGRLayer  *poLayer;
		poLayer = poDS->GetLayer(i);

		const char *pLayerName = poLayer->GetName();

		OGRLayer *poSimpLayer;
		poSimpLayer = poSimpDS->CopyLayer(poLayer, pLayerName, NULL);
		if (poSimpLayer == NULL)
		{
			printf("Layer creation failed.\n");
			continue;
		}

		OGRFeature *poFeature;
		poLayer->ResetReading();
		while ((poFeature = poLayer->GetNextFeature()) != NULL)
		{
// 			OGRFeatureDefn *poFDefn = poLayer->GetLayerDefn();
// 			int iField;
// 			for (iField = 0; iField < poFDefn->GetFieldCount(); iField++)
// 			{
// 				OGRFieldDefn *poFieldDefn = poFDefn->GetFieldDefn(iField);
// 				if (poFieldDefn->GetType() == OFTInteger)
// 					printf("%d,", poFeature->GetFieldAsInteger(iField));
// 				else if (poFieldDefn->GetType() == OFTInteger64)
// 					printf(CPL_FRMT_GIB ",", poFeature->GetFieldAsInteger64(iField));
// 				else if (poFieldDefn->GetType() == OFTReal)
// 					printf("%.3f,", poFeature->GetFieldAsDouble(iField));
// 				else if (poFieldDefn->GetType() == OFTString)
// 					printf("%s,", poFeature->GetFieldAsString(iField));
// 				else
// 					printf("%s,", poFeature->GetFieldAsString(iField));
// 			}
			OGRGeometry *orgGeometry;
			orgGeometry = poFeature->GetGeometryRef();
			if (orgGeometry != NULL
				&& wkbFlatten(orgGeometry->getGeometryType()) == wkbPolygon)
			{
// 				OGRPoint *poPoint = (OGRPoint *)poGeometry;
// 				printf("%.3f,%3.f\n", poPoint->getX(), poPoint->getY());

				//OGRGeometry *simpGeo = orgGeometry->Simplify(dTolerance);
				//OGRLinearRing *ring = ((OGRPolygon*)orgGeometry)->getExteriorRing();

// 				int ptNum = ring->getNumPoints();
// 				printf("points: %d\n", ptNum);
// 				OGRPoint pt;
// 				for (int j = 0; j < ptNum; j++)
// 				{
// 					ring->getPoint(j, &pt);
// 					printf("%.3f, %.3f\n", pt.getX(), pt.getY());
// 				}

				//method 1: Preserve Topology
				//OGRPolygon *simpGeo = (OGRPolygon*)(orgGeometry->SimplifyPreserveTopology(/*dTolerance*/0.5));

				//method 2: DP
				//OGRPolygon *simpGeo = (OGRPolygon*)(orgGeometry->Simplify(/*dTolerance*/1.0));

// 				ring = simpGeo->getExteriorRing();
// 				ptNum = ring->getNumPoints();
// 				printf("points: %d\n", ptNum);
// 				for (int j = 0; j < ptNum; j++)
// 				{
// 					ring->getPoint(j, &pt);
// 					printf("%.3f,%.3f\n", pt.getX(), pt.getY());
// 				}

				//method3: CGAL
  				OGRPolygon simpGeo;
  				ipl::simplify_polygon_CGAL((OGRPolygon*)orgGeometry, &simpGeo, dTolerance);

				poFeature->SetGeometry(&simpGeo);
				poSimpLayer->SetFeature(poFeature);

// 				orgGeometry = poFeature->GetGeometryRef();
// 				ring = ((OGRPolygon*)orgGeometry)->getExteriorRing();
// 
// 				ptNum = ring->getNumPoints();
// 				printf("points: %d\n", ptNum);
// 				for (int j = 0; j < ptNum; j++)
// 				{
// 					ring->getPoint(j, &pt);
// 					printf("%.3f, %.3f\n", pt.getX(), pt.getY());
// 				}
			}
// 			else
// 			{
// 				printf("no point geometry\n");
// 			}
//			OGRFeature::DestroyFeature(poFeature);
		}

		
	}

	GDALClose(poDS);
	GDALClose(poSimpDS);
    return 0;
}

