#pragma once
#include "core/iplcore.h"
#include "MapUpdate/MapUpdateDef.h"
#include "spatialindexing/SpatialindexDef.h"

namespace ipl
{
	//��ȡָ��Ŀ¼�µĵ��������ļ����ļ���׺idx���ļ�����keyֵ
	//note: Ҫ�������ļ������CCL/ceiling/GCMIndices��CCL/wall/GCMIndices��
	IPL_BASE_API int load_voxelPointIndices(const std::string &indices_path, double ori[3], double vsize[3], VoxelKeyMap &vmap);

	IPL_BASE_API int save_voxelPointIndices(const std::string &indices_path, const double ori[3], const double vsize[3], VoxelKeyMap &vmap);

	//�ļ���׺: .csm   �����ı���ʽ�洢
	IPL_BASE_API int load_ChangeVoxelSegments(const std::string &filename, std::vector<ChangeVoxelMap> &cseg_map);

	IPL_BASE_API int save_ChangeVoxelSegments(const std::string &filename, std::vector<ChangeVoxelMap> &cseg_map);
}

