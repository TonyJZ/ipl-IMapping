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


#include "io/PointcloudIO.h"
#include "commonAPIs/iplutility.h"
#include "commonAPIs/iplstring.h"
//#include "core/iplfiles.h"
//#include "core/iplstring.h"

#if HAVE_PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/ascii_io.h>
#endif

template<typename PointT>
int  ipl::read_PointCloud(const std::string &file_name,
	iplPointCloud<PointT> &cloud, const int offset)
{
	std::string pname, sname, lower_str;
	ipl::extract_pure_file_name(file_name, pname, sname);
	ipl::convert2lower(sname, lower_str);

	if (lower_str.compare(".pcd")==0)
	{
		pcl::PCDReader reader;
		return reader.read(file_name, cloud, offset);
	}
	else if (lower_str.compare(".ply")==0)
	{
		pcl::PLYReader reader;
		return reader.read(file_name, cloud, offset);
	}
	else if (lower_str.compare(".obj") == 0)
	{
		pcl::OBJReader reader;
		return reader.read(file_name, cloud, offset);
	}
	else if (lower_str.compare(".txt"))
	{
		pcl::ASCIIReader reader;
		return reader.read(file_name, cloud, offset);
	}

	return (-1);
}

template<typename PointT>
int  ipl::write_PointCloud(const std::string &file_name,
	const iplPointCloud<PointT> &cloud,
	const std::vector<int> &indices,
	const bool binary)
{
	std::string pname, sname, lower_str;
	ipl::extract_pure_file_name(file_name, pname, sname);
	ipl::convert2lower(sname, lower_str);

	if (lower_str.compare(".pcd") == 0)
	{
		pcl::PCDWriter writer;
		return writer.write(file_name, cloud, indices, binary);

	}
	else if (lower_str.compare(".ply") == 0)
	{
		pcl::PLYWriter writer;
		pcl::PointCloud<PointT> sub_cloud;
		pcl::copyPointCloud(cloud, indices, sub_cloud);

		return writer.write(file_name, sub_cloud, binary);
	}
// 	else if (lower_str.compare(".obj") == 0)
// 	{
// 		pcl::OBJWriter reader;
// 		return reader.read(file_name, cloud, offset);
// 	}
// 	else if (lower_str.compare(".txt"))
// 	{
// 		pcl::ASCIIWriter reader;
// 		return reader.read(file_name, cloud, offset);
// 	}

	return (-1);
}

template<typename PointT>
int  ipl::write_PointCloud(const std::string &file_name,
	const iplPointCloud<PointT> &cloud,
	const bool binary)
{
	std::vector<int> indices;
	indices.resize(cloud.size());
	for (size_t i = 0; i < indices.size(); ++i) { indices[i] = static_cast<int>(i); }

	return ipl::write_PointCloud(file_name, cloud, indices, binary);
}
