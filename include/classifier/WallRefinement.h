#pragma once

#include "core/iplcore.h"
#include "reconstruction/BuildingModelDef.h"
//#include <model_reconstruction/building_model_def.h>

#include "feature/PointGeoSegment.h"
#include "spatialindexing/ClusterConnectivityVoxelNeighbourhood.h"

namespace ipl
{
	//提取连续的最大高层区间 直方图统计法
	template <typename PointT>
	float getConsecutiveHeightScope(const iplPointCloud<PointT> &cloud, const std::vector<int> *indices=NULL, const float hTh = 0.025); 

	//主要功能：
	//1. 剔除不满足条件的墙面
	//2. 合并双面墙
	
	template <typename PointT>
	class WallRefinement
	{
		typedef ipl::iplPointCloud< PointT > PointCloud;
		typedef boost::shared_ptr <std::vector<int> > IndicesPtr;
		const int ISOLATED = -1;
		const int NONWALL = -2;
		//		using namespace ipl::bm;

	public:
		//点云的采样间隔,  墙面的厚度
		WallRefinement(float sampling_interval/*, float vsize*/);
		WallRefinement();
		virtual ~WallRefinement();

		int loadCandidateWalls(const std::string &dir, const std::string &ext);
		void setHeightScope(float floorHei, float ceilingHei);

		virtual void reinitialize();

		int removeUncertainWalls(float areaTh = 4.0, float sparseTh = 0.6, float rmsTh=0.1, float maxGirderTh = 0.3);

		//vsize: 允许的误差范围
		int mergeWalls(float vsize);  //仅标记不真正合并


		int exportAcceptedWalls(const std::string &dir, const std::string &ext);

	protected:
		float floor_hei_, ceiling_hei_;   //墙面的高程范围

	private:
		boost::shared_ptr<ipl::ClusterConnectivityVoxelNeighbourhood<PointT> > connectivity_;
		std::vector<ipl::PointGeoSegment<PointT> > segments_;
		boost::shared_ptr<iplPointCloud<PointT> > cloud_;
		//ipl::iplPointCloud<PointT>::Ptr cloud_;
		std::vector<IndicesPtr> seg_indices_;  //每个seg对应全局cloud_的indices
		std::vector<int>   p2cLut_;  //点云对应的聚类号

		float vsize_;
		float s_interval_;

		std::vector<ipl::bim::WallType>  walls_type_;
		std::vector<int> tracking_list_;  //记录合并后的cluster关系
		std::vector<std::vector<int> >  merged_indices_;
	};


}


#include <classifier/impl/wallRefinement.hpp>

