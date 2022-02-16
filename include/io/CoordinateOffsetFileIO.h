#pragma once
#include "core/iplcore.h"

#include <Eigen/Dense>

namespace ipl
{
	IPL_BASE_API bool load_OffsetParam(const std::string filename, Eigen::Vector3d &org2dst, Eigen::Vector3d &dst2org);
	IPL_BASE_API bool save_OffsetParam(const std::string filename, Eigen::Vector3d org2dst, Eigen::Vector3d dst2org);

}

