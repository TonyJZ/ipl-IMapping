#pragma once

#include "spatialindexing/ClusterConnectivityVoxelNeighbourhood.h"
#include "spatialindexing/PointVoxelization.h"


template <typename PointT> void
ipl::ClusterConnectivityVoxelNeighbourhood<PointT>::getBBox(Eigen::Vector3d &bbmin, Eigen::Vector3d &bbmax)
{
	bool bReady = initCompute();
	if (!bReady)
	{
		deinitCompute();
		return;
	}

	std::vector<double> bbox_seg_;
	bbox_seg_.resize(6, 0);
	bbox_seg_[0] = bbox_seg_[1] = bbox_seg_[2] = std::numeric_limits<float>::max();
	bbox_seg_[3] = bbox_seg_[4] = bbox_seg_[5] = std::numeric_limits<float>::lowest();

	int nSample;
	if (indices_.get() == 0)
		nSample = input_->size();
	else
		nSample = indices_->size();

	for (int i = 0; i < nSample; i++)
	{
		int id = i;
		if (indices_.get())
			id = indices_->at(i);

		PointT pt = input_->points[id];
		if (pt.x < bbox_seg_[0])
			bbox_seg_[0] = pt.x;
		if (pt.x > bbox_seg_[3])
			bbox_seg_[3] = pt.x;

		if (pt.y < bbox_seg_[1])
			bbox_seg_[1] = pt.y;
		if (pt.y > bbox_seg_[4])
			bbox_seg_[4] = pt.y;

		if (pt.z < bbox_seg_[2])
		{
			bbox_seg_[2] = pt.z;
		}
		if (pt.z > bbox_seg_[5])
		{
			bbox_seg_[5] = pt.z;
		}
	}

	bbmin[0] = bbox_seg_[0];
	bbmin[1] = bbox_seg_[1];
	bbmin[2] = bbox_seg_[2];

	bbmax[0] = bbox_seg_[3];
	bbmax[1] = bbox_seg_[4];
	bbmax[2] = bbox_seg_[5];
	return;
}


template <typename PointT> const bool
ipl::ClusterConnectivityVoxelNeighbourhood<PointT>::isClusterInVoxel3D(const int cID_arg, const PointT& point_arg)
{
	iplOctreeKey key_arg;
	int ret = getVoxelKey(point_arg, key_arg);
	if (ret == -1)
		return false;

	std::vector<int> cID_indices = vID_cID_table_[key_arg];

	bool bFind = false;
	for (int i = 0; i < cID_indices.size(); i++)
	{
		if (cID_arg == cID_indices[i])
		{
			bFind = true;
			break;
		}
	}

	return bFind;
}

template <typename PointT> const bool
ipl::ClusterConnectivityVoxelNeighbourhood<PointT>::isClusterInVoxel2D(const int cID_arg, const PointT& point_arg)
{
	double x, y;
	x = point_arg.x;
	y = point_arg.y;
	//	z = point_arg.z;

	int ijk0 = static_cast<int> (floor((x - min_p[0]) * inverse_leaf_size_[0]));
	int ijk1 = static_cast<int> (floor((y - min_p[1]) * inverse_leaf_size_[1]));
	//	int ijk2 = static_cast<int> (floor((z - min_p[2]) * inverse_leaf_size_[2]));

	iplOctreeKey	key_arg;
	key_arg.x = ijk0; key_arg.y = ijk1;

	if (key_arg.x < 0 || key_arg.x > vNumX_ - 1
		|| key_arg.y < 0 || key_arg.y > vNumY_ - 1)
		return false;

	//	bool bFind = false;
	for (int ijk2 = 0; ijk2 < vNumZ_; ++ijk2)
	{
		key_arg.z = ijk2;

		VoxelKeyMap::iterator it_vm;
		it_vm = vID_map_->find(key_arg);
		if (it_vm == vID_map_->end())
			continue;  //null voxel
		else
		{
			//int vID = it_vm->second;
			std::vector<int> cID_indices = vID_cID_table_[key_arg];
			for (int i = 0; i < cID_indices.size(); i++)
			{
				if (cID_arg == cID_indices[i])
				{
					return true;
				}
			}
		}
	}

	return false;
}

template <typename PointT> int
ipl::ClusterConnectivityVoxelNeighbourhood<PointT>::getVoxelKey(const PointT& point_arg, iplOctreeKey &key)
{
	double x, y, z;
	x = point_arg.x;
	y = point_arg.y;
	z = point_arg.z;

	int ijk0 = static_cast<int> (floor((x - min_p[0]) * inverse_leaf_size_[0]));
	int ijk1 = static_cast<int> (floor((y - min_p[1]) * inverse_leaf_size_[1]));
	int ijk2 = static_cast<int> (floor((z - min_p[2]) * inverse_leaf_size_[2]));

//	pcl::octree::OctreeKey	key_arg;
	key.x = ijk0; key.y = ijk1; key.z = ijk2;

	if (key.x < 0 || key.x > vNumX_ - 1
		|| key.y < 0 || key.y > vNumY_ - 1
		|| key.z < 0 || key.z > vNumZ_ - 1)
		return -1;

	VoxelMap::iterator it_vm;
	it_vm = vID_map_.find(key);
	if (it_vm == vID_map_.end())
		return -1;  //null voxel
	else
		return 0;
}

template <typename PointT> void
ipl::ClusterConnectivityVoxelNeighbourhood<PointT>::voxelize(/*float vsize*/)
{
	bool bReady = initCompute();
	if (!bReady)
	{
		deinitCompute();
		return;
	}
	
	pv_.setInputCloud(input_);
	pv_.setIndices(indices_);

	// Get the minimum and maximum dimensions
	pcl::getMinMax3D<PointT>(*input_, *indices_, min_p, max_p);

	pv_.setBBox(min_p, max_p);
	pv_.apply(voxel_size_, voxel_size_, voxel_size_);

//	voxel_list_ = pv_.getVoxelList();
	vID_map_ = pv_.getVoxelKeyMap(); //voxel的八叉树编码 - vID


	int64_t dx = static_cast<int64_t>((max_p[0] - min_p[0]) * inverse_leaf_size_[0]) + 1;
	int64_t dy = static_cast<int64_t>((max_p[1] - min_p[1]) * inverse_leaf_size_[1]) + 1;
	int64_t dz = static_cast<int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2]) + 1;

	if ((dx*dy*dz) > static_cast<int64_t>(std::numeric_limits<int32_t>::max()))
	{
		PCL_WARN("[pcl::UrbanRec::VoxelFCGraph] Leaf size is too small for the input dataset. Integer indices would overflow.");
		//output = *input_;
		return;
	}

	vNumX_ = dx; vNumY_ = dy; vNumZ_ = dz;

	//build vID_cID table
	vID_cID_table_.clear();
//	vID_cID_table_.resize(vID_map_->size());
	//for (int i = 0; i < voxel_list_->size(); i++)
	for(VoxelKeyMap::iterator it_vm = vID_map_->begin(); it_vm != vID_map_->end(); ++it_vm)
	{
		iplOctreeKey key = it_vm->first;
		std::vector<int>* vPtIndices = it_vm->second.getPointIndices();

		std::vector<int> cID_list;
		int cID;
		for (std::vector<int>::iterator it = vPtIndices->begin();
			it != vPtIndices->end();
			++it)
		{
			cID = pID_cID_table_->at(*it);
			cID_list.push_back(cID);
		}

		std::sort(cID_list.begin(), cID_list.end());

		std::vector<int>  vCIndices;  //voxel - cluster indices

		cID = cID_list[0];
		vCIndices.push_back(cID);
		for (int j = 0; j < cID_list.size(); j++)
		{
			if (cID != cID_list[j])
			{
				cID = cID_list[j];
				vCIndices.push_back(cID);
			}
		}

		//vID_cID_table_[key] = vCIndices;
		vID_cID_table_.insert(std::make_pair(key, vCIndices));
	}

}

template <typename PointT> void
ipl::ClusterConnectivityVoxelNeighbourhood<PointT>::apply()
{
	voxelize();

	adj_graph_.reset();
	adj_graph_ = boost::shared_ptr< CCGraph >(new CCGraph(/*cluster_amount_*/));

	VertexDescriptor vertex_descriptor(0);
	vertices_.clear();
	vertices_.resize(cluster_amount_, vertex_descriptor);

	for (int ic = 0; ic < cluster_amount_; ic++)
		vertices_[ic] = boost::add_vertex(*adj_graph_);

	//	igl::VertexDescriptor cID1, cID2;

	std::vector<iplOctreeKey> nvoxel_indices; //邻接voxel ID

	for (ipl::VoxelKeyMap::iterator vmIter = vID_map_->begin();
		vmIter != vID_map_->end();
		++vmIter)
	{//遍历每个voxel
		iplOctreeKey key_arg = vmIter->first;
		//int vID1 = vmIter->second;
		std::vector<int> v1_cID_indices = vID_cID_table_[key_arg];

		//voxel中有多个cluster时，连接所有的cluster
		for (int i = 0; i < v1_cID_indices.size() - 1; i++)
		{
			for (int j = i + 1; j < v1_cID_indices.size(); j++)
			{
				// 				cID1 = boost::vertex(v1_cID_indices[i], *adj_graph_);
				// 				cID2 = boost::vertex(v1_cID_indices[j], *adj_graph_);
				// 				one = vertex(1, undigraph);
				// 				two = vertex(2, undigraph);
				// 				add_edge(zero, one, undigraph);
				// 				add_edge(zero, two, undigraph);
				// 				add_edge(one, two, undigraph);

				boost::add_edge(vertices_[v1_cID_indices[i]], vertices_[v1_cID_indices[j]], *adj_graph_);
			}
		}

		search_Voxel_Neighbours(key_arg, nvoxel_indices);
		for (std::vector<iplOctreeKey>::iterator vIter = nvoxel_indices.begin();
			vIter != nvoxel_indices.end();
			++vIter)
		{
			iplOctreeKey vID2 = *vIter;
			std::vector<int> v2_cID_indices = vID_cID_table_[vID2];
			//连接相邻voxel中的所有cluster

			for (int i = 0; i < v1_cID_indices.size(); i++)
			{
				for (int j = 0; j < v2_cID_indices.size(); j++)
				{
					// 					cID1 = boost::vertex(v1_cID_indices[i], *adj_graph_);
					// 					cID2 = boost::vertex(v2_cID_indices[j], *adj_graph_);
					if (v1_cID_indices[i] == v2_cID_indices[j]) continue;

					boost::add_edge(vertices_[v1_cID_indices[i]], vertices_[v2_cID_indices[j]], *adj_graph_);
				}
			}
		}

	}
}

// template <typename PointT> CCGraph*
// ipl::ClusterConnectivityVoxelNeighbourhood<PointT>::getConnectedGraph()
// {
// 	return adj_graph_.get();
// }

template <typename PointT> int
ipl::ClusterConnectivityVoxelNeighbourhood<PointT>::search_Voxel_Neighbours(ipl::iplOctreeKey key_arg, std::vector<ipl::iplOctreeKey> &Voxel_indices)
{
	Voxel_indices.clear();

	std::vector<iplOctreeKey> neighbors;
	int nn = 10;  //邻域数
				  //	neighbors = Eigen::MatrixXd::Constant(nn, 3, 0);

	neighbors.resize(nn);
	//10邻域定义
	neighbors[0].x = key_arg.x - 1;		neighbors[0].y = key_arg.y;			neighbors[0].z = key_arg.z;
	neighbors[1].x = key_arg.x + 1;		neighbors[1].y = key_arg.y;			neighbors[1].z = key_arg.z;
	neighbors[2].x = key_arg.x;			neighbors[2].y = key_arg.y - 1;		neighbors[2].z = key_arg.z;
	neighbors[3].x = key_arg.x;			neighbors[3].y = key_arg.y + 1;		neighbors[3].z = key_arg.z;
	neighbors[4].x = key_arg.x;			neighbors[4].y = key_arg.y;			neighbors[4].z = key_arg.z - 1;
	neighbors[5].x = key_arg.x;			neighbors[5].y = key_arg.y;			neighbors[5].z = key_arg.z + 1;

	neighbors[6].x = key_arg.x - 1;		neighbors[6].y = key_arg.y - 1;		neighbors[6].z = key_arg.z;
	neighbors[7].x = key_arg.x - 1;		neighbors[7].y = key_arg.y + 1;		neighbors[7].z = key_arg.z;
	neighbors[8].x = key_arg.x + 1;		neighbors[8].y = key_arg.y - 1;		neighbors[8].z = key_arg.z;
	neighbors[9].x = key_arg.x + 1;		neighbors[9].y = key_arg.y + 1;		neighbors[9].z = key_arg.z;


	for (int i = 0; i < nn; i++)
	{
		if (neighbors[i].x < 0 || neighbors[i].x > vNumX_ - 1
			|| neighbors[i].y < 0 || neighbors[i].y > vNumY_ - 1
			|| neighbors[i].z < 0 || neighbors[i].z > vNumZ_ - 1)
			continue;

		VoxelKeyMap::iterator it_vm;
		it_vm = vID_map_->find(neighbors[i]);
		if (it_vm == vID_map_->end())
			continue;  //null voxel

		Voxel_indices.push_back(neighbors[i]);

	}

	return Voxel_indices.size();
}

