#pragma once
#include "core/iplcore.h"
#include <Eigen/Dense>

namespace ipl
{
	template <typename PointT>
	void pcTransformAffine(const iplPointCloud<PointT>&cloud_in, iplPointCloud<PointT> &cloud_out, 
		/*const double orgin[3], */const Eigen::Matrix4d &trans_param);
	
// 
// 	template <typename PointT>
// 	IPL_BASE_API void pcTransformRigid3D();
}

#include "registration/impl/pointcloudGeoTransformation.hpp"
