#pragma once
#include "spatialindexing/ClusterOverlappingByVoxelMap.h"


namespace ipl
{

template<typename PointT>
ClusterOverlappingByVoxelMap<PointT>::ClusterOverlappingByVoxelMap(const double org[3], const double vsize[3])
{
	origin_[0] = org[0]; origin_[1] = org[1]; origin_[2] = org[2];
	voxel_size_[0] = vsize[0]; voxel_size_[1] = vsize[1]; voxel_size_[2] = vsize[2];
}

template<typename PointT>
ClusterOverlappingByVoxelMap<PointT>::~ClusterOverlappingByVoxelMap()
{

}

template<typename PointT> void
ClusterOverlappingByVoxelMap<PointT>::reinitialize()
{

}

template<typename PointT> void
ClusterOverlappingByVoxelMap<PointT>::setClusters(std::vector<ref_ptr<PointGeoSegment<PointT>>> &clusters)
{
	clusters_ = clusters;
}

template<typename PointT> int
ClusterOverlappingByVoxelMap<PointT>::apply()
{
	pv_.reset(new PointVoxelization<PointT>);
	pv_->setOrigin(static_cast<float>(origin_[0]), static_cast<float>(origin_[1]), static_cast<float>(origin_[2]));

	global_voxel_map_.clear();
	for (size_t i = 0; i < clusters_.size(); ++i)
	{
		pv_->setInputCloud(clusters_[i]->getInputCloud());
		pv_->setIndices(clusters_[i]->getIndices());

		pv_->apply(static_cast<float>(voxel_size_[0]), static_cast<float>(voxel_size_[1]), static_cast<float>(voxel_size_[2]));

		VoxelKeyMap *vIDmap;
		vIDmap = pv_->getVoxelKeyMap();
		VoxelKeyMap::iterator vkm_it = vIDmap->begin();
		for (; vkm_it != vIDmap->end(); ++vkm_it)
		{
			iplOctreeKey key = vkm_it->first;
			GVMAP::iterator  gvm_it = global_voxel_map_.find(key);
			if (gvm_it != global_voxel_map_.end())
			{
				gvm_it->second.push_back(i);
			}
			else
			{
				std::vector<int> newItem;
				newItem.push_back(i);
				global_voxel_map_.insert(std::make_pair(key, newItem));
			}
		}
	}

	overlap_map_.clear();
	OVERLAPMap::iterator  om_it;

	GVMAP::iterator  gvm_it = global_voxel_map_.begin();
	for (; gvm_it != global_voxel_map_.end(); ++gvm_it)
	{
		for (size_t i = 0; i < gvm_it->second.size(); ++i)
		{
			int c1 = gvm_it->second[i];

			om_it = overlap_map_.find(std::make_pair(c1, c1));
			if (om_it != overlap_map_.end())
			{
				om_it->second++;
			}
			else
			{
				overlap_map_.insert(std::make_pair(std::make_pair(c1, c1), 1));
			}

			for (size_t j = i + 1; j < gvm_it->second.size(); ++j)
			{
				int c2 = gvm_it->second[j];
				om_it = overlap_map_.find(std::make_pair(c1, c2));
				if (om_it != overlap_map_.end())
				{
					om_it->second++;
				}
				else
				{
					overlap_map_.insert(std::make_pair(std::make_pair(c1, c2), 1));
				}
			}
		}
	}

	return 0;
}


}
