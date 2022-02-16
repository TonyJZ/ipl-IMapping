#pragma once
/*
* Software License Agreement (Proprietary License)
*
*
*  Copyright (c) 2017-, Appropolis, Inc.
*
*  All rights reserved.
*
*  rights described below...
*
*/

#include "core/iplcore.h"
#include "fitting/geoModelDef.h"

namespace ipl
{
	/** \brief detect planes in a given point set
	* \param[in] cloud: point cloud 
	* \param[in] indices: the indices of points which need to be processed
	*.\param[in] has_normal: set to true if the point cloud has normal 
	* \param[out] sorted_cloud: the eRansac will sort the point cloud 
	* \param[out] inliers: the indices of plane points, this indices is corresponding to sorted_cloud  
	*.\param[out] planeCoeffs: the coefficients of detected planes
	*.\param[in] minModelPts: the minimum number of points in one plane
	*.\param[in] epsTh: the threshold for plane fitting
	*.\param[in] epsConnective: the threshold for points connectivity
	*.\param[in] normalTh: the threshold for included angle of point normals, it is degree of angle %Default value: 10 degrees.
	* \return
	*  * < 0 (-1) on error
	*  * == 0 on success
	*/
	template <typename PointT>
	int detect_planes_by_eRansac(const iplPointCloud<PointT> &cloud, std::vector<int> *indices, bool has_normal,
		iplPointCloud<PointT> &sorted_cloud,
		std::vector<std::vector<int> > &inliers,
		std::vector<geoModel::geoModelInfo> &planeCoeffs,
		int minModelPts = 200, double epsTh = 0.05, double epsConnective = 0.2, double normalRadianTh = 10);
}


#include "fitting/impl/pcPlaneDetection.hpp"
