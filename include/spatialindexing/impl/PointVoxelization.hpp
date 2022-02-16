#pragma once
#include "spatialindexing/PointVoxelization.h"

#include <pcl/common/common.h>

template <typename PointT> void
ipl::PointVoxelization<PointT>::setBBox(const Eigen::Vector4f bbmin, const Eigen::Vector4f bbmax)
{
// 	min_p_[0] = bbmin[0]; min_p_[1] = bbmin[1]; min_p_[2] = bbmin[2];
// 	max_p_[0] = bbmax[0]; max_p_[1] = bbmax[1]; max_p_[2] = bbmax[2];

	min_p_ = bbmin;
	max_p_ = bbmax;
	bSetbb_ = true;
}

template <typename PointT> int
ipl::PointVoxelization<PointT>::apply(float voxelX, float voxelY, float voxelZ)
{
	bool bReady = initCompute();
	if (!bReady)
	{
		deinitCompute();
		return (-1);
	}

	leaf_size_[0] = voxelX; leaf_size_[1] = voxelY; leaf_size_[2] = voxelZ;
	// Avoid division errors
	//	if (leaf_size_[3] == 0)
	leaf_size_[3] = 1;
	// Use multiplications instead of divisions
	inverse_leaf_size_ = Eigen::Array4f::Ones() / leaf_size_.array();


	// Get the minimum and maximum dimensions
	if(!bSetbb_)
		pcl::getMinMax3D<PointT>(*input_, *indices_, min_p_, max_p_);

	if (!bSetOrg_)
		origin_ = min_p_;
	
	// 	m_bbmin[0] = min_p_[0]; m_bbmin[1] = min_p_[1]; m_bbmin[2] = min_p_[2];
	// 	m_bbmax[0] = max_p_[0]; m_bbmax[1] = max_p_[1]; m_bbmax[2] = max_p_[2];

	// Check that the leaf size is not too small, given the size of the data
	
	maxBBi_[0] = static_cast<int32_t>(ceil((max_p_[0] - origin_[0]) * inverse_leaf_size_[0]));
	maxBBi_[1] = static_cast<int32_t>(ceil((max_p_[1] - origin_[1]) * inverse_leaf_size_[1]));
	maxBBi_[2] = static_cast<int32_t>(ceil((max_p_[2] - origin_[2]) * inverse_leaf_size_[2]));

	minBBi_[0] = static_cast<int32_t>(floor((min_p_[0] - origin_[0]) * inverse_leaf_size_[0]));
	minBBi_[1] = static_cast<int32_t>(floor((min_p_[1] - origin_[1]) * inverse_leaf_size_[1]));
	minBBi_[2] = static_cast<int32_t>(floor((min_p_[2] - origin_[2]) * inverse_leaf_size_[2]));

	int32_t dx = maxBBi_[0] - minBBi_[0] + 1;
	int32_t dy = maxBBi_[1] - minBBi_[1] + 1;
	int32_t dz = maxBBi_[2] - minBBi_[2] + 1;

	if ((dx*dy*dz) > static_cast<int32_t>(std::numeric_limits<int32_t>::max()))
	{
		PCL_WARN("[pcl::UrbanRec::VoxelFCGraph] Leaf size is too small for the input dataset. Integer indices would overflow.");
		//output = *input_;
		return (-1);
	}

	vNumX_ = dx; vNumY_ = dy; vNumZ_ = dz;

//	voxel_list_.clear();
	vID_map_.clear();
	int vID;
	for (std::vector<int>::const_iterator it = indices_->begin(); it != indices_->end(); ++it)
	{
		double x, y, z;
		x = input_->points[*it].x;
		y = input_->points[*it].y;
		z = input_->points[*it].z;

		int ijk0 = static_cast<int32_t> (floor((x - origin_[0]) * inverse_leaf_size_[0]));
		int ijk1 = static_cast<int32_t> (floor((y - origin_[1]) * inverse_leaf_size_[1]));
		int ijk2 = static_cast<int32_t> (floor((z - origin_[2]) * inverse_leaf_size_[2]));

		if(ijk0<minBBi_[0] || ijk0>maxBBi_[0]
			|| ijk1<minBBi_[1] || ijk1> maxBBi_[1]
			|| ijk2<minBBi_[2] || ijk2> maxBBi_[2])
			continue;

		iplOctreeKey	key_arg;
		key_arg.x = ijk0; key_arg.y = ijk1; key_arg.z = ijk2;

		VoxelKeyMap::iterator it_vm;
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

// 			newVoxel.feat.vRefPos[0] = min_p_[0] + ijk0*leaf_size_[0] + leaf_size_[0] * 0.5;
// 			newVoxel.feat.vRefPos[1] = min_p_[1] + ijk1*leaf_size_[1] + leaf_size_[1] * 0.5;
// 			newVoxel.feat.vRefPos[2] = min_p_[2] + ijk2*leaf_size_[2] + leaf_size_[2] * 0.5;

			vID_map_.insert(std::make_pair(key_arg, newVoxel));
//			voxel_list_.push_back(newVoxel);
		}
	}

	return (0);
}

