// utest_geos.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include <iostream>
#include "ogrsf_frmts.h"

using namespace std;


int main(int argc, char * argv[])
{
	GDALAllRegister();
	GDALDataset   *poDS;
	CPLSetConfigOption("SHAPE_ENCODING", "");  //解决中文乱码问题
	
	//读取shp文件 "D:/temp/SHP/province/bou2_4p.shp"
	poDS = (GDALDataset*)GDALOpenEx(argv[1], GDAL_OF_VECTOR, NULL, NULL, NULL);

	if (poDS == NULL)
	{
		printf("Open failed.\n%s");
		exit(0);
	}

	int nLayers = poDS->GetLayerCount();

	OGRLayer  *poLayer;
	poLayer = poDS->GetLayer(0); //读取层
	poLayer->ResetReading();
	OGRFeature *poFeature1, *poFeature2, *poFeature3;
	poFeature1 = poLayer->GetFeature(0); //四川省
	poFeature2 = poLayer->GetFeature(1); //黑龙江省
	poFeature3 = poLayer->GetFeature(2); //青海省
	OGRGeometry *p1 = poFeature1->GetGeometryRef();
	OGRGeometry *p3 = poFeature2->GetGeometryRef();
	OGRGeometry *p2 = poFeature3->GetGeometryRef();
	cout << p1->IsEmpty() << endl   //图形是否为空
		<< p1->IsSimple() << endl  //是否是单个几何图形
		<< p1->getGeometryType() << endl   //几何图形的类型，polygon返回3
		<< p1->getGeometryName() << endl   //几何图形的名称
		<< p1->getDimension() << endl     //图形的维度
		<< p1->getCoordinateDimension() << endl   //坐标的维度
		<< p1->getSpatialReference() << endl;    //空间参考
	if (p2->Disjoint(p1))
		cout << "不相交" << endl;
	else
	{
		if (p2->Touches(p1))
			cout << "接触" << endl;
		else if (p2->Overlaps(p1))
			cout << "部分重叠" << endl;
		else if (p2->Contains(p1))
			cout << "包含" << endl;
		else
			cout << "unknown" << endl;
	}

	system("pause");

	return 1;
}

