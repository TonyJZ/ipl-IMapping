#pragma once
#include <core/iplcore.h>
#include "spatialindexing/PointVoxelization.h"


namespace ipl
{
// 	template <typename PointT>
// 	int  detect_buildings(ref_ptr<iplPointCloud<PointT> > cloud, std::vector<std::vector <int> > &buildingIndices, 
// 		std::vector<std::vector <int> > &treeIndices, float vsize, float heiTh, float minAreaTh);

	template <typename PointT>
	class BuildingDetection : public iplPointCluster<PointT>
	{
		enum GRIDFLAG
		{
			GF_Undefine = 0,
			GF_SolidSurface,
			GF_Edge
		};
		//网格点统计属性
		typedef struct GRIDAtt
		{
			float maxHei, minHei;
			float medianHei;

			GRIDFLAG flag; //0: undefine; 1: solid surface; 2: edge; 
		};
		typedef boost::unordered_map<QuadtreeKey, GRIDAtt> GridAttMap; 

	public:
		BuildingDetection();
		~BuildingDetection();

		//输出高程断面
		int exportHeightSection(const std::string filename, float zmin, float zmax);
		
		//voxel partition & att. stat.
		int Partition(float vsize);

		
		//检测突出物 (边界+顶面)
		//从断面 secHeiTh 开始检测
		int DetectEruptions(float secHeiTh, float heiDiffTh, float minAreaTh);

		//分割独立的建筑物
		//int SegmentBuildings();

//		int getBuildingIndices(std::vector<std::vector <int> > &buildingIndices);

		
		//导出建筑物点云
		int exportBuildingSegments(const std::string dir);

		size_t getBuildingCount();

		size_t getBuildingPointIndices(int iBuilding, std::vector<int> &indices);

	protected:


	private:
		float v_size_;

//		float zProfile_min_, zProfile_max_;

		ref_ptr<VoxelKeyMap>   vMap_;
		ref_ptr<CellKeyMap>   gridMap_;  //octree 的平面投影
		ref_ptr<GridAttMap>   attMap_;

		std::vector<std::vector<ipl::QuadtreeKey> > building_boundary_grids_;
//		std::vector<std::vector <int> > building_indices_;
		std::vector<std::vector<int> > building_segs_;
	};


}

#include "reconstruction/impl/BuildingDetection.hpp"
