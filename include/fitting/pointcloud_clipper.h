#pragma once
#include "core/iplcore.h"
#include "fitting/geoModelDef.h"
#include "feature/PointGeoSegment.h"


namespace ipl
{
	//通用的点云裁剪器
	template <typename PointT>
	class PointCloudClipper
	{
		//用label来查找对应的平面参数
		// 		typedef pcl::search::Search <PointT> KdTree;
		// 		typedef typename KdTree::Ptr KdTreePtr;

		//typedef pcl::PointXYZRGBNormal PointT;
	public:

		PointCloudClipper();
		~PointCloudClipper();


		void set_refData(const std::vector<std::string> &pszRefFiles);
		void set_tarData(const std::vector<std::string> &pszTarFiles);
		void set_Transformation(const std::string filename);
		void set_search_radius(double radius = 0.0) { search_buf_size_ = radius; };

		void do_clip();               //根据target数据范围，裁剪所需的参考数据

		void do_clip(const ipl::geoModel::BoundingBox &bbox);   //直接根据boundingbox裁剪参考数据


		//提取裁剪结果
		void getClippedRefGroup(geoModel::PointGroup<PointT> &clipped_group);
		//提取target group
		void getTargetGroup(geoModel::PointGroup<PointT> &tar_group);

		geoModel::BoundingBox getTargetBBox() { return target_bbox_; };
		geoModel::BoundingBox getCropBBox() { return clip_box_; };  //crop box 通常是对target box做转换和扩大以后得到
		
		Eigen::Matrix4d getTransformation() { return transform_.matrix(); };

	private:
		std::vector<std::string> ref_name_;
		//std::vector<std::string> ref_walls_name_;

		std::vector<std::string> tar_name_;
		//std::vector<std::string> tar_walls_name_;

		//target 数据集
		std::vector<ipl::PointGeoSegment<PointT> > tar_segments_;
		//std::vector<ipl::PointGeoSegment<PointT> > tar_wall_segments_;

		//裁剪后得到的参考数据
		std::vector<ipl::PointGeoSegment<PointT> > ref_segments_;
		//std::vector<ipl::PointGeoSegment<PointT> > ref_wall_segments_;

		bool bRefClipped_;        //ref. data is clipped or not
		geoModel::BoundingBox target_bbox_;     //bounding box of target walls
		geoModel::BoundingBox clip_box_;        //clip box of ref. data (do init transformation)

		Eigen::Affine3d  transform_;  //the target-to-reference transformation

		double search_buf_size_;
	};

}

#include "fitting/impl/pointcloud_clipper.hpp"


