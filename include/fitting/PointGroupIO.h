#pragma once

#include "core/iplcore.h"
#include "fitting/geoModelDef.h"

namespace ipl
{
	template <typename PointT>
	int  load_PointGroup(const std::string &path,
		geoModel::PointGroup<PointT> &ptGroup);

	template <typename PointT>
	int  save_PointGroup(const std::string &path,
		const geoModel::PointGroup<PointT> &ptGroup);

}

#include "fitting/impl/PointGroupIO.hpp"
