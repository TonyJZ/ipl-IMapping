#pragma once
#include "core/iplcore.h"
#include "fitting/geoModelDef.h"


namespace ipl
{
	//modelType: 2 for line, 3 for plane
	template <typename PointT>
	int geometry_detect_by_JLinkage(pcl::PointCloud<PointT> &cloud, std::vector<std::vector<int>> &clusters, geoModelType modelType,
		float mInlierThreshold, int nDNeigh = 10, float nPClose = 0.8, float nPFar = 0.2, int nSampleModels = 5000);

}

#include "fitting/impl/geometry_detection_by_Jlinkage.hpp"

#endif