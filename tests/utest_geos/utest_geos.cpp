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
	CPLSetConfigOption("SHAPE_ENCODING", "");  //���������������
	
	//��ȡshp�ļ� "D:/temp/SHP/province/bou2_4p.shp"
	poDS = (GDALDataset*)GDALOpenEx(argv[1], GDAL_OF_VECTOR, NULL, NULL, NULL);

	if (poDS == NULL)
	{
		printf("Open failed.\n%s");
		exit(0);
	}

	int nLayers = poDS->GetLayerCount();

	OGRLayer  *poLayer;
	poLayer = poDS->GetLayer(0); //��ȡ��
	poLayer->ResetReading();
	OGRFeature *poFeature1, *poFeature2, *poFeature3;
	poFeature1 = poLayer->GetFeature(0); //�Ĵ�ʡ
	poFeature2 = poLayer->GetFeature(1); //������ʡ
	poFeature3 = poLayer->GetFeature(2); //�ຣʡ
	OGRGeometry *p1 = poFeature1->GetGeometryRef();
	OGRGeometry *p3 = poFeature2->GetGeometryRef();
	OGRGeometry *p2 = poFeature3->GetGeometryRef();
	cout << p1->IsEmpty() << endl   //ͼ���Ƿ�Ϊ��
		<< p1->IsSimple() << endl  //�Ƿ��ǵ�������ͼ��
		<< p1->getGeometryType() << endl   //����ͼ�ε����ͣ�polygon����3
		<< p1->getGeometryName() << endl   //����ͼ�ε�����
		<< p1->getDimension() << endl     //ͼ�ε�ά��
		<< p1->getCoordinateDimension() << endl   //�����ά��
		<< p1->getSpatialReference() << endl;    //�ռ�ο�
	if (p2->Disjoint(p1))
		cout << "���ཻ" << endl;
	else
	{
		if (p2->Touches(p1))
			cout << "�Ӵ�" << endl;
		else if (p2->Overlaps(p1))
			cout << "�����ص�" << endl;
		else if (p2->Contains(p1))
			cout << "����" << endl;
		else
			cout << "unknown" << endl;
	}

	system("pause");

	return 1;
}

