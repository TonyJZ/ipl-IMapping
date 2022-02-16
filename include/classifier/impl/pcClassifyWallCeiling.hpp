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

#include "classifier/pcClassifyWallCeiling.h"

#if HAVE_PCL
#include <pcl/common/angles.h>
#include <pcl/conversions.h>
#endif

//using namespace ipl;

template <typename PointT>
int ipl::classify_wall_ceiling_by_normal(iplPointCloud<PointT> *cloud, std::vector<std::vector <int> > &indices,
	float ver_degTh/* = 15.0*/, float hor_degTh/* = 80.0*//*, float hei_interval = 0.1*/)
{
	bool has_normal = false;

	std::vector<pcl::PCLPointField>  fields;
	pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));

	
	// Get the index we need
	for (size_t d = 0; d < fields.size(); ++d)
		if (fields[d].name == "normal_x")
		{
			has_normal = true;
			break;
		}

	if (!has_normal)
	{
		std::cout << "this point cloud don't contain normal!" << std::endl;
		return 0;
	}
	
	//按法方向分类，提取顶面和立面
	std::vector <int> idx_hor, idx_ver;  //水平面, 垂直面
	Eigen::Vector3f  nVertical = Eigen::Vector3f(0, 0, 1);

//	float vDegThreshold_ = 15.0, fDegThreshold_ = 80.0;
	float threshold_r = cosf(pcl::deg2rad(ver_degTh));
	float threshold_f = cosf(pcl::deg2rad(hor_degTh));

	double minr_h, maxr_h;
	minr_h = std::numeric_limits<double>::max();
	maxr_h = std::numeric_limits<double>::lowest();

	for (int i = 0; i < cloud->size(); ++i)
	{
		Eigen::Map<Eigen::Vector3f> nghbr_normal(static_cast<float*> (cloud->points[i].normal));
		float dot_product = fabsf(nghbr_normal.dot(nVertical));
		if (dot_product > threshold_r)
		{
			idx_hor.push_back(i);

			if (cloud->points[i].z > maxr_h)
				maxr_h = cloud->points[i].z;
			if (cloud->points[i].z < minr_h)
				minr_h = cloud->points[i].z;
		}
		else if (dot_product < threshold_f)
		{
			idx_ver.push_back(i);
		}
	}

// 	int nstep = 30;
// 	double interval = (maxr_h - minr_h) / nstep;
	float hei_interval = 0.1;

	minr_h = minr_h - 0.5*hei_interval;
	int nstep = ceil((maxr_h - minr_h) / hei_interval);

	std::vector<int> zHist;
	zHist.resize(nstep + 2, 0);

	for (int i = 0; i < idx_hor.size(); i++)
	{
		int id = idx_hor[i];
		double z = cloud->points[id].z;

		int iStep = static_cast<int> (floor((z - minr_h) / hei_interval));
		zHist[iStep]++;
	}

	int imax, maxPts = 0;
	for (int i = 0; i < zHist.size(); i++)
	{
		if (zHist[i] > maxPts)
		{
			maxPts = zHist[i];
			imax = i;
		}
	}

	double sec_floor, sec_ceil; //屋顶的高度区间

	double roof_meanZ = minr_h + imax*hei_interval + 0.5*hei_interval;
	double buf_size = 0.1;  //0.1m
	sec_floor = roof_meanZ - buf_size;
	sec_ceil = /*roof_meanZ + buf_size;*/maxr_h;

	std::vector <int> idx_ceiling, idx_unclassified; //屋顶面索引
	for (int i = 0; i < idx_hor.size(); i++)
	{
		int id = idx_hor[i];
		double z = cloud->points[id].z;

		if (z > sec_floor && z < sec_ceil)
			idx_ceiling.push_back(id);
		else
			idx_unclassified.push_back(id);
	}

	indices.push_back(idx_ver);
	indices.push_back(idx_ceiling);
	indices.push_back(idx_unclassified);

	return (1);
}


template <typename PointT>
int ipl::classify_vertical_horizontal_by_normal(iplPointCloud<PointT> *cloud, const std::vector<int> &indices,
	std::vector<std::vector <int> > &cls_indices, float ver_degTh/* = 15.0*/, float hor_degTh/* = 80.0*//*, float hei_interval = 0.1*/)
{
	bool has_normal = false;

	std::vector<pcl::PCLPointField>  fields;
	pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));

	// Get the index we need
	for (size_t d = 0; d < fields.size(); ++d)
		if (fields[d].name == "normal_x")
		{
			has_normal = true;
			break;
		}

	if (!has_normal)
	{
		std::cout << "this point cloud don't contain normal!" << std::endl;
		return 0;
	}

	//按法方向分类，提取顶面和立面
	std::vector <int> idx_hor, idx_ver;  //水平面, 垂直面
	Eigen::Vector3f  nVertical = Eigen::Vector3f(0, 0, 1);

	//	float vDegThreshold_ = 15.0, fDegThreshold_ = 80.0;
	float threshold_r = cosf(pcl::deg2rad(ver_degTh));
	float threshold_f = cosf(pcl::deg2rad(hor_degTh));

	double minr_h, maxr_h;
	minr_h = std::numeric_limits<double>::max();
	maxr_h = std::numeric_limits<double>::lowest();

	for (int i = 0; i < indices.size(); ++i)
	{
		int id = indices[i];

		Eigen::Map<Eigen::Vector3f> nghbr_normal(static_cast<float*> (cloud->points[id].normal));
		float dot_product = fabsf(nghbr_normal.dot(nVertical));
		if (dot_product > threshold_r)
		{
			idx_hor.push_back(id);

// 			if (cloud->points[i].z > maxr_h)
// 				maxr_h = cloud->points[i].z;
// 			if (cloud->points[i].z < minr_h)
// 				minr_h = cloud->points[i].z;
		}
		else if (dot_product < threshold_f)
		{
			idx_ver.push_back(id);
		}
	}

	cls_indices.push_back(idx_ver);
	cls_indices.push_back(idx_hor);

	return (1);
}


template <typename PointT>
int ipl::extract_ceiling_by_normal(iplPointCloud<PointT> *cloud, const std::vector<int> &indices,
	std::vector<std::vector <int> > &cls_indices, float degTh)
{
	bool has_normal = false;

	std::vector<pcl::PCLPointField>  fields;
	pcl::for_each_type<typename pcl::traits::fieldList<PointT>::type>(pcl::detail::FieldAdder<PointT>(fields));

	// Get the index we need
	for (size_t d = 0; d < fields.size(); ++d)
		if (fields[d].name == "normal_x")
		{
			has_normal = true;
			break;
		}

	if (!has_normal)
	{
		std::cout << "this point cloud don't contain normal!" << std::endl;
		return 0;
	}

	//按法方向分类，提取顶面和立面
	std::vector <int> idx_ceiling, idx_unknown;  
	Eigen::Vector3f  nVertical = Eigen::Vector3f(0, 0, 1);

	//	float vDegThreshold_ = 15.0, fDegThreshold_ = 80.0;
	float vth = cosf(pcl::deg2rad(degTh));
	
	for (int i = 0; i < indices.size(); ++i)
	{
		int id = indices[i];

		Eigen::Map<Eigen::Vector3f> nghbr_normal(static_cast<float*> (cloud->points[id].normal));
		float dot_product = nghbr_normal.dot(nVertical);

		if (fabsf(dot_product) > vth && dot_product < 0)
		{
			idx_ceiling.push_back(id);
		}
		else
		{
			idx_unknown.push_back(id);
		}
	}

	cls_indices.push_back(idx_ceiling);
	cls_indices.push_back(idx_unknown);

	return (1);
}
