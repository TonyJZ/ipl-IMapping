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

#include <core/iplcore.h>

namespace ipl
{

	/** \brief classify point cloud as wall or ceiling by analyzing the normal of each point
	* \param[in] cloud: the point cloud   NOTE: the input point cloud must have 'normal' field
	* \param[in] ver_degTh:  the angle threshold for vertical points which measures the included angle to vertical direction
	* \param[in] hor_degTh:  the angle threshold for horizontal points which measures the included angle to vertical direction
	* \param[out] indices:  indices[0], wall; indices[1], ceiling; indices[2], others 
	*/
	//discard interface 2018-03-20  
	//采用下面的函数组合来实现
	template <typename PointT>
	int   classify_wall_ceiling_by_normal(iplPointCloud<PointT> *cloud, std::vector<std::vector <int> > &indices,
		float ver_degTh = 15.0, float hor_degTh = 80.0/*, float hei_interval = 0.1*/);


	template <typename PointT>
	int   classify_vertical_horizontal_by_normal(iplPointCloud<PointT> *cloud, const std::vector<int> &indices,
		std::vector<std::vector <int> > &cls_indices, float ver_degTh = 15.0, float hor_degTh = 80.0/*, float hei_interval = 0.1*/);

	template <typename PointT>
	int  extract_ceiling_by_normal(iplPointCloud<PointT> *cloud, const std::vector<int> &indices, 
		std::vector<std::vector <int> > &cls_indices, float degTh = 10.0);
}


#include <Classifier/impl/pcClassifyWallCeiling.hpp>
