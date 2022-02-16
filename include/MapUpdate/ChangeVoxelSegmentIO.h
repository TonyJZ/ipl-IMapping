#pragma once
#include "core/iplcore.h"
#include "MapUpdate/MapUpdateDef.h"
#include "spatialindexing/SpatialindexDef.h"

namespace ipl
{
	//读取指定目录下的点云索引文件，文件后缀idx，文件名是key值
	//note: 要求索引文件存放在CCL/ceiling/GCMIndices或CCL/wall/GCMIndices下
	IPL_BASE_API int load_voxelPointIndices(const std::string &indices_path, double ori[3], double vsize[3], VoxelKeyMap &vmap);

	IPL_BASE_API int save_voxelPointIndices(const std::string &indices_path, const double ori[3], const double vsize[3], VoxelKeyMap &vmap);

	//文件后缀: .csm   采用文本格式存储
	IPL_BASE_API int load_ChangeVoxelSegments(const std::string &filename, std::vector<ChangeVoxelMap> &cseg_map);

	IPL_BASE_API int save_ChangeVoxelSegments(const std::string &filename, std::vector<ChangeVoxelMap> &cseg_map);
}

