#pragma once
#include "core/iplcore.h"
#include "MapUpdate/MapUpdateDef.h"
#include "feature/PointGeoSegment.h"
#include "fitting/PlaneRefinement.h"

namespace ipl
{
	template<typename PointT>
	class MapUpdater
	{
		
		typedef std::vector<iplOctreeKey>  CCLKeyList;
		typedef boost::unordered_map<std::string, CCLKeyList> SegmentMap;

	public:
		MapUpdater();
		~MapUpdater();

		//���ù��˲�����ȥ�������������ı仯�ָ�
		void setFilters(int minSeg = 100);

		//����changed segment map
		int loadCSM(std::string filename);

		//segment refineʱ�������ݣ�ÿ��ֻ�����һ��segment�е�����
		int refineSegments(double vsize = 0.2);

		//����������ģ��
		int saveResults(std::string output_folder);

	protected:
		//��ȡѡ�е�voxel�а����ĵ�������
		int load_Indices(std::string &pgName, CCLKeyList &keylist, std::vector<int> &indices);

		//��������ֵ��ȡPointGroup�б�ѡ�е�cluster (����Ԥ����ȡ��ģ��)
		int get_SelectedPointClusters(const std::string &pgName, const std::vector<int> &sel_indices,
			std::vector<ref_ptr<PointGeoSegment<PointT> > > &clusters );

	private:
//		std::string folder_name_;   //ģ�͵���Ŀ¼
		int minSeg_;

		std::vector<ChangeVoxelMap>		changed_segments_;   //��voxel��֯��segments
		std::vector<SegmentMap>			organized_segments_;  //��ccl��֯��segments

		ref_ptr<PlaneRefinement<PointT> >  plane_refiner_;   //ƽ�澫����
		std::vector<ref_ptr<PointGeoSegment<PointT> > > refined_clusters_; //�������ģ��
	};

	
}

#include "MapUpdate/impl/MapUpdater.hpp"
