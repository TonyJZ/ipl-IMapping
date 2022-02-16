#pragma once
#include "core/iplcore.h"
#include "fitting/geoModelDef.h"
#include "feature/PointGeoSegment.h"


namespace ipl
{
	//ͨ�õĵ��Ʋü���
	template <typename PointT>
	class PointCloudClipper
	{
		//��label�����Ҷ�Ӧ��ƽ�����
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

		void do_clip();               //����target���ݷ�Χ���ü�����Ĳο�����

		void do_clip(const ipl::geoModel::BoundingBox &bbox);   //ֱ�Ӹ���boundingbox�ü��ο�����


		//��ȡ�ü����
		void getClippedRefGroup(geoModel::PointGroup<PointT> &clipped_group);
		//��ȡtarget group
		void getTargetGroup(geoModel::PointGroup<PointT> &tar_group);

		geoModel::BoundingBox getTargetBBox() { return target_bbox_; };
		geoModel::BoundingBox getCropBBox() { return clip_box_; };  //crop box ͨ���Ƕ�target box��ת���������Ժ�õ�
		
		Eigen::Matrix4d getTransformation() { return transform_.matrix(); };

	private:
		std::vector<std::string> ref_name_;
		//std::vector<std::string> ref_walls_name_;

		std::vector<std::string> tar_name_;
		//std::vector<std::string> tar_walls_name_;

		//target ���ݼ�
		std::vector<ipl::PointGeoSegment<PointT> > tar_segments_;
		//std::vector<ipl::PointGeoSegment<PointT> > tar_wall_segments_;

		//�ü���õ��Ĳο�����
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


