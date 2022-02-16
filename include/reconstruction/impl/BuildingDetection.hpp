#pragma once
#include "reconstruction/BuildingDetection.h"
#include "commonAPIs/iplutility.h"
#include "classifier/ProfileExtraction.h"
#include "spatialindexing/PointPartitionQuadtree.h"

//#include "reconstruction/BoundaryReconstruction.h"
#include "classifier/alg/clusteringAlg.h"


namespace ipl
{

template<typename PointT> int
BuildingDetection<PointT>::exportBuildingSegments(const std::string dir)
{
	bool bRet = ipl::create_folder(dir);

	if (bRet == false)
	{
		std::cout << "can not create folder: " << dir << std::endl;
		return -1;
	}


	for (int i = 0; i < building_boundary_grids_.size(); i++)
	{
		//accept
		std::string out_name, out_param;
		char buf[32], buf_para[32];
		sprintf(buf, "building_%04d.pcd", i);
		//		sprintf(buf_para, "wall_model_%04d%s", ic, geoModel::ModelParamFile_Suffix);

		out_name = dir;
		out_name += "/";
		//out_name += result_name;
		out_name += buf;

		std::vector<int> indices;
		for (int j = 0; j < building_boundary_grids_[i].size(); ++j)
		{
			QuadtreeKey key = building_boundary_grids_[i].at(j);

			CellKeyMap::iterator it;
			it = gridMap_->find(key);
			if (it != gridMap_->end())
			{
				indices.insert(indices.end(), it->second.getPointIndices()->begin(), it->second.getPointIndices()->end());
			}
		}
		// 		std::vector <int> segIndice;
		// 		for (CellKeyMap::iterator seg_it = seg.begin(); seg_it != seg.end(); ++seg_it)
		// 		{
		// 			segIndice.insert(segIndice.end(), seg_it->second.getPointIndices()->begin(),
		// 				seg_it->second.getPointIndices()->end());
		// 		}

		//writer.write<PointT>(out_name, *sort_cloud, inliers[i], true);
		ipl::write_PointCloud(out_name, *input_, indices, true);
	}
	return 0;
}

// template <typename PointT> int 
// BuildingDetection<PointT>::exportBuildingSegments(const std::string dir)
// {
// 	for (int i = 0; i < building_segs_.size(); i++)
// 	{
// 		//accept
// 		std::string out_name, out_param;
// 		char buf[32], buf_para[32];
// 		sprintf(buf, "building_%04d.pcd", i);
// 		//		sprintf(buf_para, "wall_model_%04d%s", ic, geoModel::ModelParamFile_Suffix);
// 
// 		out_name = dir;
// 		out_name += "/";
// 		//out_name += result_name;
// 		out_name += buf;
// 
// 		ipl::write_PointCloud(out_name, *input_, building_segs_[i], true);
// 	}
// 	return 0;
// }

template <typename PointT>
BuildingDetection<PointT>::BuildingDetection()
{

}

template <typename PointT>
BuildingDetection<PointT>::~BuildingDetection()
{

}

template <typename PointT> int
BuildingDetection<PointT>::Partition(float vsize)
{
	v_size_ = vsize;

	bool bReady = initCompute();
	if (!bReady)
	{
		deinitCompute();
		return -1;
	}

	//1. 建立八叉树
// 	std::cout << "create Octree..." << std::endl;
// 	PointVoxelization<PointT> octree_partition;
// 	octree_partition.setInputCloud(input_);
// 	octree_partition.setIndices(indices_);
// 
// 	octree_partition.apply(v_size_);
// 	vMap_.reset(new VoxelKeyMap);
// 	*vMap_ = *(octree_partition.getVoxelKeyMap());
// 	std::cout << " octree partition is done!" << std::endl;

	//2. 统计投影四叉树格网
// 	gridMap_.reset(new CellKeyMap);
// 	VoxelKeyMap::iterator it_vm;
// 	for (it_vm = vMap_->begin(); it_vm != vMap_->end(); ++it_vm)
// 	{
// 		QuadtreeKey grid_key;
// 		grid_key.x = it_vm->first.x;
// 		grid_key.y = it_vm->first.y;
// 
// 		CellKeyMap::iterator it_find = gridMap_->find(grid_key);
// 		if (it_find != gridMap_->end())
// 		{
// 			std::vector<int>* indices = it_find->second.getPointIndices();
// 			indices->insert(indices->end(), it_vm->second.getPointIndices()->begin(), 
// 				it_vm->second.getPointIndices()->end());
// 		}
// 		else
// 		{
// 			gridMap_->insert(std::make_pair(grid_key, it_vm->second));
// 		}
// 	}

	//直接建立四叉树
	std::cout << "create quadtree..." << std::endl;
	PointPartitionQuadtree<PointT> quadtree_partition;
	quadtree_partition.setInputCloud(input_);
	quadtree_partition.setIndices(indices_);

	quadtree_partition.apply(v_size_);
	gridMap_.reset(new CellKeyMap);
	*gridMap_ = *(quadtree_partition.getCellKeyMap());
	std::cout << " octree partition is done!" << std::endl;

	//3. 统计属性
	//	float heiTh = vsize*slopeTh;
	attMap_.reset(new GridAttMap);
	CellKeyMap::iterator it_KM;
	for (it_KM = gridMap_->begin(); it_KM != gridMap_->end(); ++it_KM)
	{
		std::vector<int>* cell_indices = it_KM->second.getPointIndices();

		float zmax, zmin;
		zmin = std::numeric_limits<float>::max();
		zmax = std::numeric_limits<float>::lowest();

		std::vector<float> zList;
		for (int i = 0; i < cell_indices->size(); ++i)
		{
			int id = cell_indices->at(i);
			float z = input_->points[id].z;
			zList.push_back(z);

			if (zmax < z)
				zmax = z;
			if (zmin > z)
				zmin = z;
		}

// 		std::sort(zList.begin(), zList.end());
// 		int mpos = zList.size() / 2;

		GRIDAtt  att;
		att.maxHei = zmax;
		att.minHei = zmin;
//		att.medianHei = zList[mpos];
		att.flag = GF_Undefine;

// 		if (zmax - zmin < heiTh)
// 			att.flag = GF_SolidSurface;
// 		else
// 			att.flag = GF_Edge;

		attMap_->insert(std::make_pair(it_KM->first, att));
	}

	return 0;
}

template <typename PointT> int 
BuildingDetection<PointT>::exportHeightSection(const std::string filename, float zmin, float zmax)
{
	bool bReady = initCompute();
	if (!bReady)
	{
		deinitCompute();
		return -1;
	}

	std::vector <int> indices;
	ipl::ProfileExtraction<PointT> pe;

	pe.setInputCloud(input_);
	pe.setIndices(indices_);
	pe.extractHeightProfile(zmin, zmax, indices);

// 	std::string out_name, out_param;
// 	char buf[32], buf_para[32];
// 	sprintf(buf, "profile.pcd");
// 	//sprintf(buf_para, "wall_model_%04d%s", ic, geoModel::ModelParamFile_Suffix);
// 
// 	out_name = /*pOutDir*/pOutputDir;
// 	out_name += "/";
// 	//out_name += result_name;
// 	out_name += buf;

	//writer.write<PointT>(out_name, *sort_cloud, inliers[i], true);
	ipl::write_PointCloud(filename, *input_, indices, true);

	return 0;
}

template <typename PointT> int 
BuildingDetection<PointT>::DetectEruptions(float secHeiTh, float heiDiffTh, float minAreaTh)
{
	GridAttMap seed_grids;
	
	std::vector<QuadtreeKey> neighbourhood;
	int neigh4 = 4, neigh8 = 8;
	neighbourhood.resize(neigh8);
	{
		neighbourhood[0].x = 1;		neighbourhood[0].y = 0;
		neighbourhood[1].x = -1;	neighbourhood[1].y = 0;
		neighbourhood[2].x = 0;		neighbourhood[2].y = 1;
		neighbourhood[3].x = 0;		neighbourhood[3].y = -1;
		neighbourhood[4].x = 1;		neighbourhood[4].y = 1;
		neighbourhood[5].x = 1;		neighbourhood[5].y = -1;
		neighbourhood[6].x = -1;	neighbourhood[6].y = 1;
		neighbourhood[7].x = -1;	neighbourhood[7].y = -1;
	}

	//检测边缘 (建筑物种子提取)
	GridAttMap::iterator it;
	for (it = attMap_->begin(); it != attMap_->end(); ++it)
	{
		//高程剖面过滤
		if(it->second.maxHei < secHeiTh)
			continue;

		//阶跃(有墙面点)
		if (it->second.maxHei - it->second.minHei > heiDiffTh)
		{
			seed_grids.insert(std::make_pair(it->first, it->second));
			it->second.flag = GF_Edge;
			continue;
		}
			
		//墙面缺失
// 		if (it->second.maxHei > minHei)
// 		{
// 			for (int i = 0; i < neigh8; ++i)
// 			{
// 				QuadtreeKey newKey = it->first + neighbourhood[i];
// 				GridAttMap::iterator it_find = attMap_->find(newKey);
// 
// 				if (it_find == attMap_->end())
// 				{//存在空洞
// 					candidate_grids.insert(std::make_pair(it->first, it->second));
// 					break;
// 				}
// 			}
// 		}
	}

	// 阶跃种子的连通性分割
	building_boundary_grids_.clear();
	GridAttMap vmapDup = *attMap_;

	while (seed_grids.size() > 0)
	{//遍历所有种子，进行基于种子生长的分割

		GridAttMap::iterator it_key;
		it_key = seed_grids.begin();

		std::vector<ipl::QuadtreeKey> curSeg;

		//region growing
		std::stack<QuadtreeKey> keyStack;

		keyStack.push(it_key->first); // 插入种子

		seed_grids.erase(it_key);
		vmapDup.erase(it_key->first);

		while (!keyStack.empty())
		{//种子生长
			QuadtreeKey topKey = keyStack.top();
			keyStack.pop();

			curSeg.push_back(topKey); //新的聚类
//			insert(std::make_pair(topKey, keymap[topKey]));

			for (size_t i = 0; i < neigh8; ++i)
			{
				QuadtreeKey neighKey = topKey + neighbourhood[i];
				GridAttMap::iterator it_find = vmapDup.find(neighKey);

				if (it_find != vmapDup.end())
				{//存在邻居
					if (it_find->second.flag == GF_Edge)
					{//两个边缘块
// 						if (fabs((*attMap_)[topKey].maxHei - it_find->second.maxHei) < v_size_)
// 						{
							keyStack.push(neighKey);
							seed_grids.erase(neighKey);
							vmapDup.erase(neighKey);
//						}
					}
					else
					{//边缘块和非边缘块
						if (it_find->second.maxHei > secHeiTh &&
							fabs((*attMap_)[topKey].maxHei - it_find->second.maxHei) < 2*v_size_)
						{
							keyStack.push(neighKey);
							vmapDup.erase(neighKey);
						}
					}
				}
			}
		}

		//过滤掉小的突出物
		if (curSeg.size()*v_size_*v_size_ < minAreaTh)
			continue;

		building_boundary_grids_.push_back(curSeg);
	}

	return 0;
}

// template <typename PointT> int
// BuildingDetection<PointT>::SegmentBuildings()
// {
// 	bool bReady = initCompute();
// 	if (!bReady)
// 	{
// 		deinitCompute();
// 		return -1;
// 	}
// 
// 	std::vector<int> peaks_indices;
// 	std::vector<int> cluster_centers;
// 	std::vector<uint32_t> slabels;	//labels for current slice points 
// 	std::vector<double> rhos;
// 
// 	clustering_by_DBSCAN(*input_, indices_.get(), 4.0, 4, cluster_centers, slabels, rhos);
// 
// 	//统计分割数
// 	int segNum = 0;
// 	for (int i = 0; i < slabels.size(); ++i)
// 	{
// 		if (slabels[i] > segNum)
// 			segNum = slabels[i];
// 	}
// 
// 	building_segs_.resize(segNum);
// 
// 	assert(indices_->size() == slabels.size());
// 
// 	for (int i = 0; i < slabels.size(); ++i)
// 	{
// 		if (slabels[i] > 0)
// 		{
// 			int segId = slabels[i];
// 			int pid = indices_->at(i);
// 
// 			building_segs_[segId].push_back(pid);
// 		}
// 	}
// 
// 	return 0;
// }

template <typename PointT> size_t
BuildingDetection<PointT>::getBuildingCount()
{
	return building_boundary_grids_.size();
}

template <typename PointT> size_t
BuildingDetection<PointT>::getBuildingPointIndices(int iBuilding, std::vector<int> &indices)
{
	if (iBuilding<0 || iBuilding>building_boundary_grids_.size() - 1)
		return 0;

	indices.clear();
	for (int j = 0; j < building_boundary_grids_[iBuilding].size(); ++j)
	{
		QuadtreeKey key = building_boundary_grids_[iBuilding].at(j);

		CellKeyMap::iterator it;
		it = gridMap_->find(key);
		if (it != gridMap_->end())
		{
			indices.insert(indices.end(), it->second.getPointIndices()->begin(), it->second.getPointIndices()->end());
		}
	}
	
	return indices.size();
}

}

