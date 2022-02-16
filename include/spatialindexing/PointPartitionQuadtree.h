#pragma once
#include "core/iplcore.h"
#include "spatialindexing/SpatialindexDef.h"

namespace ipl
{
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/** \brief partition points into 2D grids
	*  \
	*/
	template <typename PointT>
	class PointPartitionQuadtree : public ipl::iplPointCluster<PointT>
	{//2D partition
	public:
		typedef ipl::iplPointCloud< PointT > PointCloud;
		typedef typename PointCloud::ConstPtr PointCloudConstPtr;
		typedef typename PointCloud::Ptr PointCloudPtr;

		using ipl::iplPointCluster<PointT>::indices_;
		using pcl::PCLBase<PointT>::input_;

	public:
		PointPartitionQuadtree()
		{
			bSetbb_ = false;
		}

		virtual ~PointPartitionQuadtree()
		{
		}

		//对指定范围内的点进行voxelization
		void setBBox(const Eigen::Vector4f bbmin, const Eigen::Vector4f bbmax);

		void setBBox(const double bbmin[3], const double bbmax[3]);

		void getBBox(Eigen::Vector4f &bbmin, Eigen::Vector4f &bbmax);

		virtual int apply(float vsize);

//		virtual int apply(float vx, float vy);

		inline void getCenterPosition(const ipl::QuadtreeKey key, double &xc, double &yc)
		{
			xc = key.x*leaf_size_[0] + min_p[0] + 0.5*leaf_size_[0];
			yc = key.y*leaf_size_[1] + min_p[1] + 0.5*leaf_size_[1];
		};

		inline void getRect(const ipl::QuadtreeKey key, iplRECT<double> &rect)
		{
			rect.m_xmin = key.x*leaf_size_[0] + min_p[0];
			rect.m_xmax = rect.m_xmin + leaf_size_[0];
			rect.m_ymin = key.y*leaf_size_[1] + min_p[1];
			rect.m_ymax = rect.m_ymin + leaf_size_[1];
		};

//		CellList* getCellList() { return &voxel_list_; };
		CellKeyMap* getCellKeyMap() { return &vID_map_; };

		float getMeanPtsNumofCell();

	protected:


	private:
//		CellList  voxel_list_;
		CellKeyMap   vID_map_; //voxel的八叉树编码 - vID

		int vNumX_, vNumY_/*, vNumZ_*/;
		Eigen::Vector4f leaf_size_;
		Eigen::Array4f inverse_leaf_size_;
		Eigen::Vector4f min_p, max_p;

		bool bSetbb_;
	};
}

#include "spatialindexing/impl/PointPartitionQuadtree.hpp"
