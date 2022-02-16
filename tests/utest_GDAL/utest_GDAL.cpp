// utest_GDAL.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include "gdal_alg.h"
#include "gdal_priv.h"   //添加GDAL库函数的头文件
#include "ogrsf_frmts.h"

#include <iostream>

using namespace std;


int WritePolygonShp()
{
	const char *pszDriverName = "ESRI Shapefile";
	GDALDriver *poDriver;
	GDALAllRegister();
	poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
	if (poDriver == NULL)
	{
		printf("%s driver not available.\n", pszDriverName);
		exit(1);
	}
	GDALDataset *poDS;
	poDS = poDriver->Create("D:\\iplTestData\\utest\\test_skeleton\\test2.shp", 0, 0, 0, GDT_Unknown, NULL); //创建数据源
	if (poDS == NULL)
	{
		printf("Creation of output file failed.\n");
		exit(1);
	}
	OGRLayer *poLayer;
	poLayer = poDS->CreateLayer("point_out", NULL, wkbPolygon, NULL); //创建图层
	if (poLayer == NULL)
	{
		printf("Layer creation failed.\n");
		exit(1);
	}

	OGRFieldDefn oField("Name", OFTString); //创建属性
	oField.SetWidth(32);
	if (poLayer->CreateField(&oField) != OGRERR_NONE)
	{
		printf("Creating Name field failed.\n");
		exit(1);
	}

	OGRFieldDefn oField1("PointX", OFTReal);
	oField1.SetPrecision(3);
	if (poLayer->CreateField(&oField1) != OGRERR_NONE)
	{
		cout << "Creating Name1 field failed." << endl;
		return 0;
	}

	OGRFeature *poFeature;
	poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
	poFeature->SetField(0, "good");
	poFeature->SetField(1, 1);

	OGRLinearRing out_ring, inner_ring;

	int i = 10;
	//1. 逆时针
// 	out_ring.addPoint(-50, -50);
// 	out_ring.addPoint(50, -50);
// 	out_ring.addPoint(50, 50);
// 	out_ring.addPoint(-50, 50);

	//2. 顺时针
	out_ring.addPoint(-50, 50);
	out_ring.addPoint(50, 50);
	out_ring.addPoint(50, -50);
	out_ring.addPoint(-50, -50);
	
	out_ring.closeRings();//首尾点重合形成闭合环 
	OGRPolygon poly;

	poly.addRing(&out_ring);

	//3. hole 逆时针
	inner_ring.addPoint(-30, -30);
	inner_ring.addPoint(30, -30);
	inner_ring.addPoint(30, 30);
	inner_ring.addPoint(-30, 30);	

	//4. hole 顺时针
// 	inner_ring.addPoint(-30, 30);
// 	inner_ring.addPoint(30, 30);
// 	inner_ring.addPoint(30, -30);
// 	inner_ring.addPoint(-30, -30);
	
	inner_ring.closeRings();

	poly.addRing(&inner_ring);


	poFeature->SetGeometry(&poly);
//	poLayer->SetFeature(poFeature);

	if (poLayer->CreateFeature(poFeature) != OGRERR_NONE)
	{
		printf("Failed to create feature in shapefile.\n");
		exit(1);
	}
	OGRFeature::DestroyFeature(poFeature);

	//OGRFeature *poFeature;
// 	poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
// 	poFeature->SetField(0, "normal");
// 	poFeature->SetField(1, 2);

// 	ring.empty();
// 
// 	ring.addPoint(30, 20);
// 	ring.addPoint(80, 70);
// 	ring.addPoint(100, 45);
// 	ring.addPoint(100, 20);
// 	ring.addPoint(100, 45);
// 	ring.closeRings();
// 	poly.addRing(&ring);
// 	poFeature->SetGeometry(&poly);
// 	if (poLayer->CreateFeature(poFeature) != OGRERR_NONE)
// 	{
// 		printf("Failed to create feature in shapefile.\n");
// 		exit(1);
// 	}
	
//	OGRFeature::DestroyFeature(poFeature);

	GDALClose(poDS);

	return 0;
}

int WriteLineStringShp()
{
	const char *pszDriverName = "ESRI Shapefile";
	GDALDriver *poDriver;
	GDALAllRegister();
	poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
	if (poDriver == NULL)
	{
		printf("%s driver not available.\n", pszDriverName);
		exit(1);
	}
	GDALDataset *poDS;
	poDS = poDriver->Create("D:\\temp\\linestring_out.shp", 0, 0, 0, GDT_Unknown, NULL);
	if (poDS == NULL)
	{
		printf("Creation of output file failed.\n");
		exit(1);
	}
	OGRLayer *poLayer;
	poLayer = poDS->CreateLayer("point_out", NULL, wkbLineString, NULL);
	if (poLayer == NULL)
	{
		printf("Layer creation failed.\n");
		exit(1);
	}

	OGRFieldDefn oField("Name", OFTString);
	oField.SetWidth(32);
	if (poLayer->CreateField(&oField) != OGRERR_NONE)
	{
		printf("Creating Name field failed.\n");
		exit(1);
	}

	OGRFieldDefn oField1("PointX", OFTReal);
	oField1.SetPrecision(3);
	if (poLayer->CreateField(&oField1) != OGRERR_NONE)
	{
		cout << "Creating Name1 field failed." << endl;
		return 0;
	}

	OGRFeature *poFeature;
	poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
	poFeature->SetField(0, "good");
	poFeature->SetField(1, 1);

	OGRMultiLineString  polylines;

	OGRLineString *lines = new OGRLineString();

	lines->addPoint(100, 100);
	lines->addPoint(100, 200);

	polylines.addGeometry(lines);
	
	lines->empty();
	lines->addPoint(200, 150);
	lines->addPoint(400, 700);
	polylines.addGeometry(lines);

	poFeature->SetGeometry(&polylines);

// 	lines->empty();
// 	lines->addPoint(100, 100);
// 	lines->addPoint(100, 200);
// 	poFeature->SetGeometry(lines);

	if (poLayer->CreateFeature(poFeature) != OGRERR_NONE)
	{
		printf("Failed to create feature in shapefile.\n");
		exit(1);
	}
	OGRFeature::DestroyFeature(poFeature);
	GDALClose(poDS);

	return 0;
}

int Writepolygon_kml()
{
	const char *pszDriverName = "KML";
	GDALDriver *poDriver;
	GDALAllRegister();
	poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
	if (poDriver == NULL)
	{
		printf("%s driver not available.\n", pszDriverName);
		exit(1);
	}
	GDALDataset *poDS;
	poDS = poDriver->Create("D:\\temp\\3Dpolygon_out.kml", 0, 0, 0, GDT_Unknown, NULL); //创建数据源
	if (poDS == NULL)
	{
		printf("Creation of output file failed.\n");
		exit(1);
	}
	OGRLayer *poLayer;
	poLayer = poDS->CreateLayer("building_boundary", NULL, wkbPolygon, NULL); //创建图层
	if (poLayer == NULL)
	{
		printf("Layer creation failed.\n");
		exit(1);
	}

	OGRFieldDefn oField("Facet Name", OFTString); //创建属性
	oField.SetWidth(32);
	if (poLayer->CreateField(&oField) != OGRERR_NONE)
	{
		printf("Creating Name field failed.\n");
		exit(1);
	}

	OGRFieldDefn oField1("Facet ID", OFTInteger);
	//oField1.SetPrecision(3);
	if (poLayer->CreateField(&oField1) != OGRERR_NONE)
	{
		cout << "Creating Name1 field failed." << endl;
		return 0;
	}

	OGRFeature *poFeature;
	poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
	poFeature->SetField(0, "good");
	poFeature->SetField(1, 1);

	OGRLinearRing ring;

	int i = 10;

	ring.addPoint(-114.0769187663662, 51.05120827725572,100);
	ring.addPoint(-114.0772182810246, 51.04888991863528,50);
	ring.addPoint(-114.0732675198972, 51.04888991863528,0);

	ring.closeRings();//首尾点重合形成闭合环 
	OGRPolygon poly;

	poly.addRing(&ring);

	poFeature->SetGeometry(&poly);
	if (poLayer->CreateFeature(poFeature) != OGRERR_NONE)
	{
		printf("Failed to create feature in shapefile.\n");
		exit(1);
	}
	OGRFeature::DestroyFeature(poFeature);
	GDALClose(poDS);

	return 0;
}

int WriteLineString_kmz()
{
	const char *pszDriverName = "LIBKML";
	GDALDriver *poDriver;
	GDALAllRegister();
	poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
	if (poDriver == NULL)
	{
		printf("%s driver not available.\n", pszDriverName);
		exit(1);
	}
	GDALDataset *poDS;
	poDS = poDriver->Create("D:\\temp\\linestring_out.kmz", 0, 0, 0, GDT_Unknown, NULL);
	if (poDS == NULL)
	{
		printf("Creation of output file failed.\n");
		exit(1);
	}
	OGRLayer *poLayer;
	poLayer = poDS->CreateLayer("point_out", NULL, wkbLineString, NULL);
	if (poLayer == NULL)
	{
		printf("Layer creation failed.\n");
		exit(1);
	}

	OGRFieldDefn oField("Name", OFTString);
	oField.SetWidth(32);
	if (poLayer->CreateField(&oField) != OGRERR_NONE)
	{
		printf("Creating Name field failed.\n");
		exit(1);
	}

	OGRFieldDefn oField1("PointX", OFTReal);
	oField1.SetPrecision(3);
	if (poLayer->CreateField(&oField1) != OGRERR_NONE)
	{
		cout << "Creating Name1 field failed." << endl;
		return 0;
	}

	OGRFeature *poFeature;
	poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
	poFeature->SetField(0, "good");
	poFeature->SetField(1, 1);

	OGRLineString *lines = new OGRLineString();

	lines->addPoint(100, 100);

	lines->addPoint(100, 200);
	lines->addPoint(200, 150);

	poFeature->SetGeometry(lines);

	if (poLayer->CreateFeature(poFeature) != OGRERR_NONE)
	{
		printf("Failed to create feature in shapefile.\n");
		exit(1);
	}
	OGRFeature::DestroyFeature(poFeature);
	GDALClose(poDS);

	return 0;
}

int read_kmz()
{
//	char filename[] = "D:/code_ipl/3rdparty/Reconstruction/source/libkml-2.2/testdata/kmz/camels.kml";
	char filename[] = "D:/temp/03/doc.kml";
//	char filename[] = "D:/temp/03.kmz";
	GDALAllRegister();
	GDALDataset       *poDS;
	poDS = (GDALDataset*)GDALOpenEx(filename, GDAL_OF_VECTOR, NULL, NULL, NULL);
	if (poDS == NULL)
	{
		printf("Open failed.\n");
		exit(1);
	}

	int nLayer = poDS->GetLayerCount();
	for (int i = 0; i < nLayer; ++i)
	{
		OGRLayer  *poLayer;
		
		//poLayer = poDS->GetLayerByName("point");
		poLayer = poDS->GetLayer(i);
		const char *name = poLayer->GetName();

		OGRwkbGeometryType type = poLayer->GetGeomType();
		OGRFeature *poFeature;
		poLayer->ResetReading();
		while ((poFeature = poLayer->GetNextFeature()) != NULL)
		{
			OGRFeatureDefn *poFDefn = poLayer->GetLayerDefn();
			int iField;
			for (iField = 0; iField < poFDefn->GetFieldCount(); iField++)
			{
				OGRFieldDefn *poFieldDefn = poFDefn->GetFieldDefn(iField);
				if (poFieldDefn->GetType() == OFTInteger)
					printf("%d,", poFeature->GetFieldAsInteger(iField));
				else if (poFieldDefn->GetType() == OFTInteger64)
					printf(CPL_FRMT_GIB ",", poFeature->GetFieldAsInteger64(iField));
				else if (poFieldDefn->GetType() == OFTReal)
					printf("%.3f,", poFeature->GetFieldAsDouble(iField));
				else if (poFieldDefn->GetType() == OFTString)
					printf("%s,", poFeature->GetFieldAsString(iField));
				else
					printf("%s,", poFeature->GetFieldAsString(iField));
			}
			OGRGeometry *poGeometry;
			poGeometry = poFeature->GetGeometryRef();
			if (poGeometry != NULL
				&& wkbFlatten(poGeometry->getGeometryType()) == wkbPoint)
			{
				OGRPoint *poPoint = (OGRPoint *)poGeometry;
				printf("%.3f,%3.f\n", poPoint->getX(), poPoint->getY());
			}
			else
			{
				printf("no point geometry\n");
			}
			OGRFeature::DestroyFeature(poFeature);
		}
	}

	GDALClose(poDS);
	return 0;
}

int main(int argc, char * argv[])
{

	read_kmz();
// 	WritePolygonShp();
// 	WriteLineStringShp();

//	Writepolygon_kml();
//	WriteLineString_kmz();

    return 0;
}

