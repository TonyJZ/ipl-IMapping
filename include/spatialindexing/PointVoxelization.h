#pragma once
#include "core/iplcore.h"
#include "spatialindexing/SpatialindexDef.h"

#include <Eigen/Dense>

namespace ipl
{
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/** \brief partition points into voxels.
	*  \
	*/
	template <typename PointT>
	class PointVoxelization : public ipl::iplPointCluster<PointT>
	{
	public:
		typedef ipl::iplPointCloud< PointT > PointCloud;
		typedef typename PointCloud::ConstPtr PointCloudConstPtr;
		typedef typename PointCloud::Ptr PointCloudPtr;

		using ipl::iplPointCluster<PointT>::indices_;
		using pcl::PCLBase<PointT>::input_;

	public:
		PointVoxelization()
		{
			bSetbb_ = false;
			bSetOrg_ = false;
			minBBi_ = Eigen::Vector3i::Zero();
			maxBBi_ = Eigen::Vector3i::Zero();
			vNumX_ = vNumY_ = vNumZ_ = 0;
		}

		virtual ~PointVoxelization()
		{
		}

		//对指定范围内的点进行voxelization
		void setBBox(const Eigen::Vector4f bbmin, const Eigen::Vector4f bbmax);
		void getBBox(Eigen::Vector4f &bbmin, Eigen::Vector4f &bbmax)
		{
			bbmin = min_p_;
			bbmax = max_p_;
		}

		//设置原点
		void setOrigin(const float x0, const float y0, const float z0)
		{
			origin_[0] = x0; origin_[1] = y0; origin_[2] = z0; origin_[3] = 1.0;
			bSetOrg_ = true;
		}

		virtual int apply(float vsize)
		{
			return apply(vsize, vsize, vsize);
		}

		virtual int apply(float voxelX, float voxelY, float voxelZ);

//		ipl::VoxelList* getVoxelList() { return &voxel_list_; };
		ipl::VoxelKeyMap* getVoxelKeyMap() { return &vID_map_; };

		Eigen::Vector3i getVoxelScope() { return Eigen::Vector3i(vNumX_, vNumY_, vNumZ_); };
		void getVoxelScope(Eigen::Vector3i &minBB, Eigen::Vector3i &maxBB) 
		{ 
			minBB = minBBi_;
			maxBB = maxBBi_;
			return ;
		};

	protected:
		

	private:
//		ipl::VoxelList  voxel_list_;
		ipl::VoxelKeyMap   vID_map_; //voxel的八叉树编码 - vID

		int vNumX_, vNumY_, vNumZ_;
		Eigen::Vector3i minBBi_, maxBBi_;
		Eigen::Vector4f leaf_size_;
		Eigen::Array4f inverse_leaf_size_;
		Eigen::Vector4f min_p_, max_p_; //bounding box 

		Eigen::Vector4f origin_;	//原点

		bool bSetbb_;
		bool bSetOrg_;
	};
}

#include "spatialindexing/impl/PointVoxelization.hpp"
