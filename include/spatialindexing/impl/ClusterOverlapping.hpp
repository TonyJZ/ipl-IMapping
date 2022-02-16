#pragma once
#include "spatialindexing/ClusterOverlapping.h"


template <typename PointT> void
ipl::ClusterOverlapping<PointT>::setFirstCluster(const PointCloudConstPtr &cloud, const IndicesPtr &indices)
{
	first_input_ = cloud;
	first_indices_ = indices;
}

template <typename PointT> void
ipl::ClusterOverlapping<PointT>::setFirstCluster(const PointCloudConstPtr &cloud)
{
	first_input_ = cloud;
	first_indices_.reset(new std::vector<int>);
	//first_indices_->resize(first_input_->size());
	for (int i = 0; i < first_input_->size(); i++)
		first_indices_->push_back(i);
}

template <typename PointT> void
ipl::ClusterOverlapping<PointT>::setSecondCluster(const PointCloudConstPtr &cloud, const IndicesPtr &indices)
{
	second_input_ = cloud;
	second_indices_ = indices;
}

template <typename PointT> void
ipl::ClusterOverlapping<PointT>::setSecondCluster(const PointCloudConstPtr &cloud)
{
	second_input_ = cloud;
	second_indices_.reset(new std::vector<int>);
	//first_indices_->resize(first_input_->size());
	for (int i = 0; i < second_input_->size(); i++)
		second_indices_->push_back(i);
}

template <typename PointT> int
ipl::ClusterOverlapping<PointT>::apply(float vsize)
{
	Eigen::Vector4f f_min_p, f_max_p;
	Eigen::Vector4f s_min_p, s_max_p;

	pcl::getMinMax3D<PointT>(*first_input_, *first_indices_, f_min_p, f_max_p);
	pcl::getMinMax3D<PointT>(*second_input_, *second_indices_, s_min_p, s_max_p);

	Eigen::Vector4f bbmin, bbmax;

	bbmin[0] = (f_min_p[0] < s_min_p[0]) ? f_min_p[0] : s_min_p[0];
	bbmin[1] = (f_min_p[1] < s_min_p[1]) ? f_min_p[1] : s_min_p[1];
	bbmin[2] = (f_min_p[2] < s_min_p[2]) ? f_min_p[2] : s_min_p[2];

	bbmax[0] = (f_max_p[0] > s_max_p[0]) ? f_max_p[0] : s_max_p[0];
	bbmax[1] = (f_max_p[1] > s_max_p[1]) ? f_max_p[1] : s_max_p[1];
	bbmax[2] = (f_max_p[2] > s_max_p[2]) ? f_max_p[2] : s_max_p[2];

	PointVoxelization<PointT> pv1, pv2;

	pv1.setInputCloud(first_input_);
	pv1.setIndices(first_indices_);
	pv1.setBBox(bbmin, bbmax);
	pv1.apply(vsize, vsize, vsize);

	pv2.setInputCloud(second_input_);
	pv2.setIndices(second_indices_);
	pv2.setBBox(bbmin, bbmax);
	pv2.apply(vsize, vsize, vsize);

//	ipl::VoxelList *voxel_list1, *voxel_list2;
	ipl::VoxelKeyMap  *vID_map1, *vID_map2;

//	voxel_list1 = pv1.getVoxelList();
	vID_map1 = pv1.getVoxelKeyMap(); 

//	voxel_list2 = pv2.getVoxelList();
	vID_map2 = pv2.getVoxelKeyMap();

	f_voxel_num_ = vID_map1->size();
	s_voxel_num_ = vID_map2->size();
	o_voxel_num_ = 0;

	f_overlapping_indices_.reset(new std::vector<int>);
	s_overlapping_indices_.reset(new std::vector<int>);

	for (VoxelKeyMap::iterator vmIter1 = vID_map1->begin();
		vmIter1 != vID_map1->end();
		++vmIter1)
	{//±éÀúfirst pointcloud
		iplOctreeKey key_arg = vmIter1->first;

		VoxelKeyMap::iterator vmIter2;
		vmIter2 = vID_map2->find(key_arg);
		if (vmIter2 == vID_map2->end())
			continue;  //non-overlapping

		o_voxel_num_++;

//		int vID1 = vmIter1->second;
//		std::vector<int>* vIds1 = voxel_list1->at(vID1).getPointIndices();
		std::vector<int>* vIds1 = vmIter1->second.getPointIndices();
	
		f_overlapping_indices_->insert(f_overlapping_indices_->end(), vIds1->begin(), vIds1->end());

// 		int vID2 = vmIter2->second;
// 		std::vector<int>* vIds2 = voxel_list2->at(vID2).getPointIndices();
		std::vector<int>* vIds2 = vmIter2->second.getPointIndices();

		s_overlapping_indices_->insert(s_overlapping_indices_->end(), vIds2->begin(), vIds2->end());
	}

	return (0);
}


