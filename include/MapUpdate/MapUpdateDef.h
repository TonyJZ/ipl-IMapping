#pragma once
#include "core/iplcore.h"
#include "fitting/geoModelDef.h"
#include "spatialindexing/iploctree_key.h"
//#include <pcl/octree/octree_key.h>
#include <boost/unordered_map.hpp>

namespace ipl
{
	//discard 2018-04-14  Tony
	//不同类型的CCL(wall, ceiling)在不同的gcm中处理
//	template<typename PointT>
// 	struct CandidateChangeLayer   //CCL
// 	{
// 		ref_ptr<geoModel::PointGroup<PointT> > ceilingGroup;
// 		ref_ptr<geoModel::PointGroup<PointT> > wallGroup;
// 	};


	//GCM中的块索引，记录每块中出现的CCL名字
	typedef std::vector<std::string>  CCLNameList;

	//全局变化图GCM，将各个CandidateChangeLayer合并
	typedef boost::unordered_map<iplOctreeKey, CCLNameList> ChangeVoxelMap;

	const char CeilingFolder[] = "ceiling";
	const char WallFolder[] = "wall";

	const char CCLName[] = "CCL";
	const char GCMIndicesFolder[] = "GCMIndices";
	const char GCMParamsName[] = "GCMParams.txt"; //GCM参数文件名, 不同GCM的voxelPointIndices不可混用
	
	const char CCLIndiceFile_Suffix[] = ".idx";          //索引文件后缀
	const char CCLLutFile_Suffix[] = ".lut";            //点云查找表文件后缀
	const char GCMFile_Suffix[] = ".gcm";               //global change map 文件后缀  用于GlobalChangeDetection
	const char CSMFile_Suffix[] = ".csm";				//changed segment map 文件后缀  GlobalChangeDetection导出的结果
}
