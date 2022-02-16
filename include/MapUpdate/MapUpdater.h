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

		//设置过滤参数，去除不满足条件的变化分割
		void setFilters(int minSeg = 100);

		//加载changed segment map
		int loadCSM(std::string filename);

		//segment refine时加载数据，每次只会加载一个segment中的数据
		int refineSegments(double vsize = 0.2);

		//导出检测出的模型
		int saveResults(std::string output_folder);

	protected:
		//提取选中的voxel中包含的点索引号
		int load_Indices(std::string &pgName, CCLKeyList &keylist, std::vector<int> &indices);

		//根据索引值提取PointGroup中被选中的cluster (符合预先提取的模型)
		int get_SelectedPointClusters(const std::string &pgName, const std::vector<int> &sel_indices,
			std::vector<ref_ptr<PointGeoSegment<PointT> > > &clusters );

	private:
//		std::string folder_name_;   //模型导出目录
		int minSeg_;

		std::vector<ChangeVoxelMap>		changed_segments_;   //按voxel组织的segments
		std::vector<SegmentMap>			organized_segments_;  //按ccl组织的segments

		ref_ptr<PlaneRefinement<PointT> >  plane_refiner_;   //平面精化器
		std::vector<ref_ptr<PointGeoSegment<PointT> > > refined_clusters_; //精化后的模型
	};

	
}

#include "MapUpdate/impl/MapUpdater.hpp"
