#pragma once
#include "core/iplcore.h"
#include "spatialindexing/SpatialindexDef.h"
#include "spatialindexing/PointVoxelization.h"

#include "feature/PointGeoSegment.h"

namespace ipl
{
	//多个cluster之间的重叠度统计
	//利用hash map和PointVoxelization实现
	template<typename PointT>
	class ClusterOverlappingByVoxelMap
	{
		typedef boost::unordered_map<iplOctreeKey, std::vector<int> > GVMAP;

	public:
		ClusterOverlappingByVoxelMap(const double org[3], const double vsize[3]);
		~ClusterOverlappingByVoxelMap();

		void reinitialize();
		void setClusters(std::vector<ref_ptr<PointGeoSegment<PointT> > > &clusters);

		int apply();

		const OVERLAPMap& const getOverlappingClusters() { return overlap_map_; };

	protected:


	private:
		double origin_[3];
		double voxel_size_[3];
		ref_ptr<PointVoxelization<PointT> > pv_;
		std::vector<ref_ptr<PointGeoSegment<PointT> > > clusters_;

		GVMAP  global_voxel_map_;
		OVERLAPMap  overlap_map_;
	};
}

#include "spatialindexing/impl/ClusterOverlappingByVoxelMap.hpp"
