#pragma once
#include "core/iplcore.h"

namespace ipl
{
	namespace rec
	{
		//2维polygon的属性字段名称 (需要和arcScan中的属性字段创建模板一致)
		//note: shp格式字段有10字符限制
		const char FieldDefn_Polygon_ID[] = "PgnID";  //polygone ID
		const char FieldDefn_Name[] = "Name";  //建筑物名称
		const char FieldDefn_ROOF_HEIGHT[] = "RoofH";    //建筑物顶面高度
		const char FieldDefn_FLOOR_HEIGHT[] = "FloorH";  //建筑物墙角点高度

		const char FieldDefn_PROFILE_HEIGHT[] = "PrfH";  //提取boundary的高程剖面
		
		//文件夹名
		const char FolderName_AShape[] = "AShape";
		const char FolderName_Walls[] = "Walls";
		const char FolderName_Roofs[] = "Roofs";

		//文件名 building boudary names
		const char BBName_ORG[] = "orgAShape.shp";
		const char BBName_RFN[] = "rfnAShape.shp";
		const char BBName_Simplified[] = "_simplified";
		const char BBName_Offset[] = "_offset";
	}


}

