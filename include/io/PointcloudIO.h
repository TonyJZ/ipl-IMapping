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

	/** \brief read pointcloud file
	* \param[in] file_name
	* \param[out] cloud   point cloud data 
	* \param[in] offset  the offset in the file where to expect the true header to begin.
	* \return
	*  * < 0 (-1) on error
	*  * == 0 on success
	*/
	template<typename PointT>
	int  read_PointCloud(const std::string &file_name, 
		iplPointCloud<PointT> &cloud, const int offset = 0);

	/** \brief write pointcloud file
	* \param[in] file_name
	* \param[out] cloud   point cloud data
	* \param[in] indices  the indices of pointcloud that want to write out.
	*.\param[in] binary set to true if the file is to be written in a binary
	* format, false (default) for ASCII
	* \return
	*  * < 0 (-1) on error
	*  * == 0 on success
	*/
	template<typename PointT>
	int  write_PointCloud(const std::string &file_name,
		const iplPointCloud<PointT> &cloud,
		const std::vector<int> &indices,
		const bool binary = false);

	/** \brief write pointcloud file
	* \param[in] file_name
	* \param[out] cloud   point cloud data
	*.\param[in] binary set to true if the file is to be written in a binary
	* format, false (default) for ASCII
	* \return
	*  * < 0 (-1) on error
	*  * == 0 on success
	*/
	template<typename PointT>
	int  write_PointCloud(const std::string &file_name,
		const iplPointCloud<PointT> &cloud,
		const bool binary = false);
}


#include <io/impl/PointcloudIO.hpp>
