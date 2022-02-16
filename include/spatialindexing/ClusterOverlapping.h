#pragma once

#include "core/iplcore.h"
#include "spatialindexing/PointVoxelization.h"

namespace ipl
{
	//两个cluster之间的空间重叠关系
	//利用voxel来确定
	template <typename PointT>
	class ClusterOverlapping
	{
	public:
		typedef iplPointCloud< PointT > PointCloud;
		typedef typename PointCloud::ConstPtr PointCloudConstPtr;
		typedef ref_ptr <std::vector<int> > IndicesPtr;
		

//		using iplPointCluster<PointT>::initCompute;
//		using iplPointCluster<PointT>::deinitCompute;
//		using iplPointCluster<PointT>::indices_;
//		using iplPointCluster<PointT>::input_;

	public:
		ClusterOverlapping()
		{
			
		}

		virtual ~ClusterOverlapping()
		{
			first_input_.reset();
			first_indices_.reset();

			second_input_.reset();
			second_indices_.reset();

			f_overlapping_indices_.reset();
			s_overlapping_indices_.reset();
		}


		void setFirstCluster(const PointCloudConstPtr &cloud, const IndicesPtr &indices);
		void setFirstCluster(const PointCloudConstPtr &cloud);

		void setSecondCluster(const PointCloudConstPtr &cloud, const IndicesPtr &indices);
		void setSecondCluster(const PointCloudConstPtr &cloud);

		virtual int apply(float vsize);

		void getOverlappingVoxelNum(int &fVoxels, int &sVoxels, int &oVoxels)
		{
			fVoxels = f_voxel_num_;
			sVoxels = s_voxel_num_;
			oVoxels = o_voxel_num_;
		}

		void getOverlappingIndices(IndicesPtr &findices, IndicesPtr &sindices)
		{
			findices = f_overlapping_indices_;
			sindices = s_overlapping_indices_;
		}

	protected:
		

	private:
		PointCloudConstPtr first_input_;
		IndicesPtr first_indices_;
		
		PointCloudConstPtr second_input_;
		IndicesPtr second_indices_;

		int f_voxel_num_, s_voxel_num_, o_voxel_num_;

		IndicesPtr  f_overlapping_indices_;
		IndicesPtr  s_overlapping_indices_;
	};

}



#include "spatialindexing/impl/ClusterOverlapping.hpp"
