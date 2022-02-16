#pragma once

#include "core/iplcore.h"
#include "reconstruction/BuildingModelDef.h"
//#include <model_reconstruction/building_model_def.h>

#include "feature/PointGeoSegment.h"
#include "spatialindexing/ClusterConnectivityVoxelNeighbourhood.h"

namespace ipl
{
	//��ȡ���������߲����� ֱ��ͼͳ�Ʒ�
	template <typename PointT>
	float getConsecutiveHeightScope(const iplPointCloud<PointT> &cloud, const std::vector<int> *indices=NULL, const float hTh = 0.025); 

	//��Ҫ���ܣ�
	//1. �޳�������������ǽ��
	//2. �ϲ�˫��ǽ
	
	template <typename PointT>
	class WallRefinement
	{
		typedef ipl::iplPointCloud< PointT > PointCloud;
		typedef boost::shared_ptr <std::vector<int> > IndicesPtr;
		const int ISOLATED = -1;
		const int NONWALL = -2;
		//		using namespace ipl::bm;

	public:
		//���ƵĲ������,  ǽ��ĺ��
		WallRefinement(float sampling_interval/*, float vsize*/);
		WallRefinement();
		virtual ~WallRefinement();

		int loadCandidateWalls(const std::string &dir, const std::string &ext);
		void setHeightScope(float floorHei, float ceilingHei);

		virtual void reinitialize();

		int removeUncertainWalls(float areaTh = 4.0, float sparseTh = 0.6, float rmsTh=0.1, float maxGirderTh = 0.3);

		//vsize: �������Χ
		int mergeWalls(float vsize);  //����ǲ������ϲ�


		int exportAcceptedWalls(const std::string &dir, const std::string &ext);

	protected:
		float floor_hei_, ceiling_hei_;   //ǽ��ĸ̷߳�Χ

	private:
		boost::shared_ptr<ipl::ClusterConnectivityVoxelNeighbourhood<PointT> > connectivity_;
		std::vector<ipl::PointGeoSegment<PointT> > segments_;
		boost::shared_ptr<iplPointCloud<PointT> > cloud_;
		//ipl::iplPointCloud<PointT>::Ptr cloud_;
		std::vector<IndicesPtr> seg_indices_;  //ÿ��seg��Ӧȫ��cloud_��indices
		std::vector<int>   p2cLut_;  //���ƶ�Ӧ�ľ����

		float vsize_;
		float s_interval_;

		std::vector<ipl::bim::WallType>  walls_type_;
		std::vector<int> tracking_list_;  //��¼�ϲ����cluster��ϵ
		std::vector<std::vector<int> >  merged_indices_;
	};


}


#include <classifier/impl/wallRefinement.hpp>

