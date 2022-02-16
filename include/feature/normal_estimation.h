#pragma once
#include "core/iplcore.h"
#include "core/ipldef.h"


namespace ipl
{
	template <typename PointT>
	int estimate_normals_CGAL_PCA(const iplPointCloud<PointT> &cloud, const std::vector<int> *indices, 
		std::vector<iplNORMAL3D> &normals);

}

#include "feature/impl/normal_estimation.hpp"
