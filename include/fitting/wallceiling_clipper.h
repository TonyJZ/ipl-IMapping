#pragma once
#include "core/iplcore.h"
#include "fitting/geoModelDef.h"
#include "feature/PointGeoSegment.h"

//#include <pcl/search/kdtree.h>

namespace ipl
{
	//模型图层裁剪器,同时裁剪wall & ceiling 主要面向registration设计
	template <typename PointT>
	class WallCeilingClipper
	{
		//用label来查找对应的平面参数
// 		typedef pcl::search::Search <PointT> KdTree;
// 		typedef typename KdTree::Ptr KdTreePtr;

		//typedef pcl::PointXYZRGBNormal PointT;
	public:

		WallCeilingClipper();
		~WallCeilingClipper();


		void set_refData(const std::vector<std::string> &pszCeilingFiles, const std::vector<std::string> &pszWallFiles);
		void set_tarData(const std::vector<std::string> &pszCeilingFiles, const std::vector<std::string> &pszWallFiles);
		void set_Transformation(const std::string filename);
		void set_search_radius(double radius = 2.0) { search_buf_size_ = radius; };

		void do_clip();               //根据target数据范围，裁剪所需的参考数据

		void do_clip(const ipl::geoModel::BoundingBox &bbox);   //直接根据boundingbox裁剪参考数据


		//提取裁剪结果
		void getClippedCeilings(geoModel::PointGroup<PointT> &clipped_group);
		void getClippedWalls(geoModel::PointGroup<PointT> &clipped_group);

		//提取target点集
		void getTargetCeilings(geoModel::PointGroup<PointT> &clipped_group);
		void getTargetWalls(geoModel::PointGroup<PointT> &clipped_group);

	private:
		std::vector<std::string> ref_ceilings_name_;
		std::vector<std::string> ref_walls_name_;

		std::vector<std::string> tar_ceilings_name_;
		std::vector<std::string> tar_walls_name_;

		//target 数据集
		std::vector<ipl::PointGeoSegment<PointT> > tar_ceiling_segments_;
		std::vector<ipl::PointGeoSegment<PointT> > tar_wall_segments_;

		//裁剪后得到的参考数据
		std::vector<ipl::PointGeoSegment<PointT> > ref_ceiling_segments_;
		std::vector<ipl::PointGeoSegment<PointT> > ref_wall_segments_;

		bool bRefClipped_;        //ref. data is clipped or not
		geoModel::BoundingBox target_bbox_;     //bounding box of target walls
		geoModel::BoundingBox clip_box_;        //clip box of ref. data (do init transformation)

		Eigen::Affine3d  transform_;  //the target-to-reference transformation

		double search_buf_size_;
	};

}

#include "fitting/impl/wallceiling_clipper.hpp"
