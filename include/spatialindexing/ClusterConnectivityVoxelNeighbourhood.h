#pragma once
#include "core/iplcore.h"

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

#include "spatialindexing/SpatialindexDef.h"
#include "spatialindexing/VoxelContainer.h"
#include "spatialindexing/PointVoxelization.h"


namespace ipl
{
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/** \brief construct voxel neighbourhood for a batch of point clusters
	*  \
	*/
	template <typename PointT>
	class ClusterConnectivityVoxelNeighbourhood : public ipl::iplPointCluster<PointT>
	{
	public:
		typedef typename boost::property <boost::edge_weight_t, std::vector<float> > Weight;
		typedef typename boost::adjacency_list <boost::hash_setS, boost::vecS, boost::undirectedS, boost::no_property, Weight> CCGraph;
		typedef typename boost::adjacency_list_traits< boost::vecS, boost::vecS, boost::undirectedS> Traits;
		typedef Traits::vertex_descriptor VertexDescriptor;
		typedef typename boost::graph_traits< CCGraph >::edge_descriptor EdgeDescriptor;
		typedef typename boost::graph_traits< CCGraph >::out_edge_iterator OutEdgeIterator;
		typedef typename boost::graph_traits< CCGraph >::in_edge_iterator InEdgeIterator;
		typedef typename boost::graph_traits< CCGraph >::vertex_iterator VertexIterator;
		typedef typename boost::graph_traits< CCGraph >::adjacency_iterator AdjacencyIterator;

	public:
		typedef iplPointCloud< PointT > PointCloud;
		typedef typename PointCloud::ConstPtr PointCloudConstPtr;
		

		using iplPointCluster<PointT>::initCompute;
		using iplPointCluster<PointT>::deinitCompute;
		using iplPointCluster<PointT>::indices_;
		using iplPointCluster<PointT>::input_;

	public:
		ClusterConnectivityVoxelNeighbourhood()
		{
			pID_cID_table_ = nullptr;

			voxel_size_ = 0.0f;
			vNumX_ = vNumY_ = vNumZ_ = 0;

			cluster_amount_ = 0;
		}

		ClusterConnectivityVoxelNeighbourhood(const float vsize, const std::vector<int> *p2c_Lut)
		{
			pID_cID_table_ = p2c_Lut;
			//cluster ID: from 0 to n, consecutive numbering, no jumping

			int ptNum = p2c_Lut->size();
			cluster_amount_ = p2c_Lut->at(ptNum - 1) + 1;

			voxel_size_ = vsize;

			leaf_size_[0] = vsize; leaf_size_[1] = vsize; leaf_size_[2] = vsize;
			leaf_size_[3] = 1;
			
			inverse_leaf_size_ = Eigen::Array4f::Ones() / leaf_size_.array();
		}

		virtual ~ClusterConnectivityVoxelNeighbourhood()
		{
			if (adj_graph_ != 0)
				adj_graph_.reset();
		}

		virtual void apply();

		virtual CCGraph* getConnectedGraph()
		{
				return adj_graph_.get();
		}

//		void setPointClusterLut(std::vector<int>* p2c_Lut);

		void getBBox(Eigen::Vector3d &bbmin, Eigen::Vector3d &bbmax);

		

		//根据查询点坐标，提取voxel Key.  no voxel return -1 
		int getVoxelKey(const PointT& point_arg, iplOctreeKey &key);

		const bool isClusterInVoxel2D(const int cID_arg, const PointT& point_arg);

		const bool isClusterInVoxel3D(const int cID_arg, const PointT& point_arg);

	protected:
		void voxelize(/*float vsize*/);

		int search_Voxel_Neighbours(iplOctreeKey key_arg, std::vector<iplOctreeKey> &Voxel_indices);


	private:
		//lut
		const std::vector<int>*  pID_cID_table_;  //point-cluster lut
		boost::unordered_map<iplOctreeKey, std::vector<int>> vID_cID_table_;  //每个voxel中的cluster编号

													   //graph
		std::vector< VertexDescriptor > vertices_; //唯一的顶点数组，避免重复插入顶点
		boost::shared_ptr<CCGraph> adj_graph_;	//adjacency graph for clusters


		//voxels
		ipl::PointVoxelization<PointT>  pv_;
//		ipl::VoxelList *voxel_list_;
		ipl::VoxelKeyMap  *vID_map_; //voxel的八叉树编码 - vID

		float voxel_size_;
 		int vNumX_, vNumY_, vNumZ_;
 		Eigen::Vector4f leaf_size_;
 		Eigen::Array4f inverse_leaf_size_;
 		Eigen::Vector4f min_p, max_p;

		//聚类总数 从point-cluster lut中确定
		int		cluster_amount_;


	};
}


#include "spatialindexing/impl/ClusterConnectivityVoxelNeighbourhood.hpp"
