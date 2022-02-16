#pragma once
#include "core/iplcore.h"

namespace ipl
{
	namespace rec
	{
		//2άpolygon�������ֶ����� (��Ҫ��arcScan�е������ֶδ���ģ��һ��)
		//note: shp��ʽ�ֶ���10�ַ�����
		const char FieldDefn_Polygon_ID[] = "PgnID";  //polygone ID
		const char FieldDefn_Name[] = "Name";  //����������
		const char FieldDefn_ROOF_HEIGHT[] = "RoofH";    //�����ﶥ��߶�
		const char FieldDefn_FLOOR_HEIGHT[] = "FloorH";  //������ǽ�ǵ�߶�

		const char FieldDefn_PROFILE_HEIGHT[] = "PrfH";  //��ȡboundary�ĸ߳�����
		
		//�ļ�����
		const char FolderName_AShape[] = "AShape";
		const char FolderName_Walls[] = "Walls";
		const char FolderName_Roofs[] = "Roofs";

		//�ļ��� building boudary names
		const char BBName_ORG[] = "orgAShape.shp";
		const char BBName_RFN[] = "rfnAShape.shp";
		const char BBName_Simplified[] = "_simplified";
		const char BBName_Offset[] = "_offset";
	}


}

