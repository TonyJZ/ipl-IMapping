#pragma once

#include "core/iplcore.h"

#include <Eigen/Dense>

namespace ipl
{
	//Eigen::Matrix4d a;
	//Eigen::Affine3d b;
	//b.matrix() = a;    or  b = a;

	//f = R*X+t
	IPL_BASE_API bool load_RigidTransParam(const std::string filename, Eigen::Matrix4d &trans_param, double &rms);
	IPL_BASE_API bool save_RigidTransParam(const std::string filename, Eigen::Matrix4d trans_param, double rms);



}

