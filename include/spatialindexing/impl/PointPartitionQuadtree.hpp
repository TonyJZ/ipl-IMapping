#pragma once
#include "spatialindexing/PointPartitionQuadtree.h"
#include <pcl/common/common.h>

template <typename PointT> void 
ipl::PointPartitionQuadtree<PointT>::getBBox(Eigen::Vector4f &bbmin, Eigen::Vector4f &bbmax)
{
	bbmin = min_p;
	bbmax = max_p;
}

template <typename PointT> void
ipl::PointPartitionQuadtree<PointT>::setBBox(const Eigen::Vector4f bbmin, const Eigen::Vector4f bbmax)
{
	// 	min_p[0] = bbmin[0]; min_p[1] = bbmin[1]; min_p[2] = bbmin[2];
	// 	max_p[0] = bbmax[0]; max_p[1] = bbmax[1]; max_p[2] = bbmax[2];

	min_p = bbmin;
	max_p = bbmax;
	bSetbb_ = true;
}

template <typename PointT> void
ipl::PointPartitionQuadtree<PointT>::setBBox(const double bbmin[3], const double bbmax[3])
{
	min_p[0] = bbmin[0]; min_p[1] = bbmin[1]; min_p[2] = bbmin[2]; min_p[3] = 1.0;
	max_p[0] = bbmax[0]; max_p[1] = bbmax[1]; max_p[2] = bbmax[2]; max_p[3] = 1.0;
	bSetbb_ = true;
}

template <typename PointT> int
ipl::PointPartitionQuadtree<PointT>::apply(float vsize)
{
	bool bReady = initCompute();
	if (!bReady)
	{
		deinitCompute();
		return (-1);
	}

	leaf_size_[0] = vsize; leaf_size_[1] = vsize; leaf_size_[2] = vsize;
	// Avoid division errors
	//	if (leaf_size_[3] == 0)
	leaf_size_[3] = 1;
	// Use multiplications instead of divisions
	inverse_leaf_size_ = Eigen::Array4f::Ones() / leaf_size_.array();


	// Get the minimum and maximum dimensions
	if(!bSetbb_)
		pcl::getMinMax3D<PointT>(*input_, *indices_, min_p, max_p);

	// 	m_bbmin[0] = min_p[0]; m_bbmin[1] = min_p[1]; m_bbmin[2] = min_p[2];
	// 	m_bbmax[0] = max_p[0]; m_bbmax[1] = max_p[1]; m_bbmax[2] = max_p[2];

	// Check that the leaf size is not too small, given the size of the data
	int64_t dx = static_cast<int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0]) + 1;
	int64_t dy = static_cast<int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1]) + 1;
//	int64_t dz = static_cast<int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2]) + 1;

	if ((dx*dy) > static_cast<int64_t>(std::numeric_limits<int32_t>::max()))
	{
		PCL_WARN("[pcl::UrbanRec::VoxelFCGraph] Leaf size is too small for the input dataset. Integer indices would overflow.");
		//output = *input_;
		return (-1);
	}

	vNumX_ = dx; vNumY_ = dy; 

//	voxel_list_.clear();
	vID_map_.clear();
	int vID;
	for (std::vector<int>::const_iterator it = indices_->begin(); it != indices_->end(); ++it)
	{
		double x, y;
		x = input_->points[*it].x;
		y = input_->points[*it].y;
		//z = input_->points[*it].z;

		int ijk0 = static_cast<int> (floor((x - min_p[0]) * inverse_leaf_size_[0]));
		int ijk1 = static_cast<int> (floor((y - min_p[1]) * inverse_leaf_size_[1]));
		//int ijk2 = static_cast<int> (floor((z - min_p[2]) * inverse_leaf_size_[2]));

		if (ijk0 < 0 || ijk0 >= vNumX_
			|| ijk1 < 0 || ijk1 >= vNumY_
			)
			continue;

		QuadtreeKey	key_arg;
		key_arg.x = ijk0; key_arg.y = ijk1;

		CellKeyMap::iterator it_vm;
		it_vm = vID_map_.find(key_arg);

		if (it_vm != vID_map_.end())
		{//exist voxel
// 			vID = it_vm->second;
// 			voxel_list_[vID].addPointIndex(*it);
			it_vm->second.addPointIndex(*it);
		}
		else
		{//add new voxel
			ipl::VoxelContainerPointIndices newVoxel;
			// 			newVoxel.occupyFlag = pcl::UrbanRec::occupied_voxel;
			// 			newVoxel.voxel_key = key_arg;
			newVoxel.addPointIndex(*it);
// 			newVoxel.feat.vRefPos[0] = min_p[0] + ijk0*leaf_size_[0] + leaf_size_[0] * 0.5;
// 			newVoxel.feat.vRefPos[1] = min_p[1] + ijk1*leaf_size_[1] + leaf_size_[1] * 0.5;

			vID_map_.insert(std::make_pair(key_arg, newVoxel));
//			voxel_list_.push_back(newVoxel);
		}
	}

	return (0);
}

template <typename PointT> float
ipl::PointPartitionQuadtree<PointT>::getMeanPtsNumofCell()
{
	if (vID_map_.size() == 0)
		return 0;


	size_t cellNum = vID_map_.size();
	int ptNum = 0;
	CellKeyMap::iterator iter;

	for (iter = vID_map_.begin(); iter != vID_map_.end(); ++iter)
	{
		ptNum += iter->second.getSize();
	}

	return (float(ptNum)) / cellNum;
}

