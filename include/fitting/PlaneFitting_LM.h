#pragma once
#include "core/iplcore.h"
#include "feature/PointGeoSegment.h"

namespace ipl
{
	//2018-04-10 Tony: 未进行选权迭代，
	template<typename PointT>
	int plane_fitting_LM(std::vector<ref_ptr<PointGeoSegment<PointT> > > &clusters, geoModel::geoModelInfo &refined_coef);

}

#include "fitting/impl/PlaneFitting_LM.hpp"
