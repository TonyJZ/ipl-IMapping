#pragma once
#include "fitting/PlaneRefinement.h"
#include "fitting/PlaneFitting_LM.h"

#include <stack>

namespace ipl
{

template<typename PointT> 
PlaneRefinement<PointT>::PlaneRefinement()
{
	connective_ratio_ = 0.8;
}

template<typename PointT>
PlaneRefinement<PointT>::~PlaneRefinement()
{

}

template<typename PointT> void
PlaneRefinement<PointT>::reinitialize()
{

	clusters_.clear();
}

template<typename PointT> void 
PlaneRefinement<PointT>::addPlane(ref_ptr<PointGeoSegment<PointT>> &cluster)
{
	clusters_.push_back(cluster);
}

template<typename PointT> void
PlaneRefinement<PointT>::addPlanes(std::vector<ref_ptr<PointGeoSegment<PointT>>> &clusters)
{
	clusters_.insert(clusters_.end(), clusters.begin(), clusters.end());
}

template<typename PointT> int
PlaneRefinement<PointT>::refine(double vsize)
{
	//1. 提取邻接的分割对 (pair)
	geoModel::BoundingBox bbox;
	bbox.min_pt[0] = bbox.min_pt[1] = bbox.min_pt[2] = std::numeric_limits<double>::max();
	bbox.max_pt[0] = bbox.max_pt[1] = bbox.max_pt[2] = std::numeric_limits<double>::lowest();

	for (size_t i = 0; i < clusters_.size(); ++i)
	{
		geoModel::geoModelInfo coef = clusters_[i]->getModelCoef();

		if (bbox.min_pt[0] > coef.bbox.min_pt[0])
			bbox.min_pt[0] = coef.bbox.min_pt[0];
		if (bbox.min_pt[1] > coef.bbox.min_pt[1])
			bbox.min_pt[1] = coef.bbox.min_pt[1];
		if (bbox.min_pt[2] > coef.bbox.min_pt[2])
			bbox.min_pt[2] = coef.bbox.min_pt[2];

		if (bbox.max_pt[0] < coef.bbox.max_pt[0])
			bbox.max_pt[0] = coef.bbox.max_pt[0];
		if (bbox.max_pt[1] < coef.bbox.max_pt[1])
			bbox.max_pt[1] = coef.bbox.max_pt[1];
		if (bbox.max_pt[2] < coef.bbox.max_pt[2])
			bbox.max_pt[2] = coef.bbox.max_pt[2];
	}

	double center[3], voxel_size[3];
	center[0] = 0.5*(bbox.min_pt[0] + bbox.max_pt[0]);
	center[1] = 0.5*(bbox.min_pt[1] + bbox.max_pt[1]);
	center[2] = 0.5*(bbox.min_pt[2] + bbox.max_pt[2]);
	voxel_size[0] = voxel_size[1] = voxel_size[2] = vsize;

	connMap_.reset(new ClusterOverlappingByVoxelMap<PointT>(center, voxel_size));
	connMap_->setClusters(clusters_);
	connMap_->apply();
	const OVERLAPMap overlappings = connMap_->getOverlappingClusters();

	int num_cls = clusters_.size();
	std::vector<int> clsVoxels;
	clsVoxels.resize(num_cls);
	for (int i = 0; i < num_cls; ++i)
	{
		std::pair<int, int> key = std::make_pair(i, i);
		clsVoxels[i] = overlappings.find(key)->second;
	}

	//2. 计算邻接表，并确定融合列表
	Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> adj_table;
	adj_table = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>::Zero(num_cls, num_cls);

	for (int i = 0; i < num_cls; ++i)
	{
		adj_table(i, i) = true;
		
		for (int j = i+1; j < num_cls; ++j)
		{
			OVERLAPMap::const_iterator  om_it = overlappings.find(std::make_pair(i, j));
			if (om_it == overlappings.end())
				continue;
			
			int overlap_degree = om_it->second;

			if (overlap_degree > connective_ratio_*clsVoxels[i]
				|| overlap_degree > connective_ratio_*clsVoxels[j])
			{
				adj_table(i, j) = true;
				adj_table(j, i) = true;  //对称矩阵
			}
		}

	}

	std::vector<std::vector<int> >  mergeList;
	for (int i = 0; i < num_cls; ++i)
	{
		//当前行是否在前面已经被处理过
		bool bProcessed = false;
		for (int j = 0; j < i; ++j)
		{
			if (adj_table(i, j) == true)
			{
				bProcessed = true;
				break;
			}
		}

		if (bProcessed)
			continue;

		std::vector<int>  newList;
		newList.push_back(i);

		//找到当前行号所有邻接的pair
		//直接邻接通过for循环遍历，间接邻接通过stack查找
		std::stack<int> clsID_stack;
		for (int j = i + 1; j < num_cls; ++j)
		{
			if (adj_table(i, j) == true)
			{
				clsID_stack.push(j);
				newList.push_back(j);
			}
		}

		while (!clsID_stack.empty())
		{
			int irow = clsID_stack.top();
			clsID_stack.pop();

			for (int icol = irow + 1; icol < num_cls; ++icol)
			{
				if (adj_table(i, icol) == true) 
					continue;

				newList.push_back(icol);
				clsID_stack.push(icol);
			}
		}

		mergeList.push_back(newList);
	}

	//3. merge planes
	refined_clusters_.clear();
	for (size_t i = 0; i < mergeList.size(); ++i)
	{
		if (mergeList[i].size() == 1)
		{
			int clsID = mergeList[i].at(0);
			refined_clusters_.push_back(clusters_[clsID]);
		}
		else
		{
			std::vector<ref_ptr<PointGeoSegment<PointT> > > selected_clusters;
			for (size_t j = 0; j < mergeList[i].size(); ++j)
			{
				int clsID = mergeList[i].at(j);
				selected_clusters.push_back(clusters_[clsID]);
			}

			geoModel::geoModelInfo refined_coef;
			int ret = plane_fitting_LM(selected_clusters, refined_coef);
			if (ret != 0)
			{
				std::cout << "error! can not fit plane " << i << std::endl;
				continue;
			}
			
			ref_ptr<PointGeoSegment<PointT> > newCluster(new PointGeoSegment<PointT>);
			ref_ptr<iplPointCloud<PointT> > merged_cloud(new iplPointCloud<PointT>);

			for (size_t j = 0; j < selected_clusters.size(); ++j)
			{
				iplPointCloud<PointT>::ConstPtr cloud = selected_clusters[i]->getInputCloud();

				merged_cloud->points.insert(merged_cloud->points.end(), cloud->points.begin(), cloud->points.end());
			}
			merged_cloud->height = 1;
			merged_cloud->width = merged_cloud->size();

			newCluster->setInputCloud(merged_cloud);
			newCluster->setModelCoef(refined_coef);
			newCluster->reinitialize();

			refined_clusters_.push_back(newCluster);
		}
	}

	return 0;
}

}

