#pragma once
#include "fitting/pointcloud_clipper.h"
#include "io/PointcloudIO.h"
#include "fitting/ModelParamsIO.h"

namespace ipl
{
	template <typename PointT>
	PointCloudClipper<PointT>::PointCloudClipper()
	{
		transform_ = Eigen::Matrix4d::Identity();

		search_buf_size_ = 0.0;

		bRefClipped_ = false;

		target_bbox_.min_pt[0] = target_bbox_.min_pt[1] = target_bbox_.min_pt[2] = std::numeric_limits<double>::max();
		target_bbox_.max_pt[0] = target_bbox_.max_pt[1] = target_bbox_.max_pt[2] = std::numeric_limits<double>::lowest();

		clip_box_.min_pt[0] = clip_box_.min_pt[1] = clip_box_.min_pt[2] = std::numeric_limits<double>::max();
		clip_box_.max_pt[0] = clip_box_.max_pt[1] = clip_box_.max_pt[2] = std::numeric_limits<double>::lowest();
	}

	template <typename PointT>
	PointCloudClipper<PointT>::~PointCloudClipper()
	{

	}

	template <typename PointT> void
		PointCloudClipper<PointT>::set_refData(const std::vector<std::string> &pszRefFiles)
	{//只记录, 裁剪时才提取数据
		ref_name_ = pszRefFiles;
	}

	template <typename PointT> void
		PointCloudClipper<PointT>::set_tarData(const std::vector<std::string> &pszTarFiles)
	{
		char strblank[128];
		memset(strblank, ' '/*32*/, sizeof(char) * 128);

		tar_name_ = pszTarFiles;

		std::string rname, sname;
		std::string paraname;

		double bbmin[3], bbmax[3];
		ipl::geoModel::geoModelInfo mCoef;

		tar_segments_.clear();
		size_t nWalls = tar_name_.size();
		for (size_t i = 0; i < nWalls; i++)
		{
			std::cout << "loading target walls.... " << i + 1 << " / " << nWalls << "\r";

			//提取分割点云和模型参数
			ipl::extract_file_name(tar_name_[i], rname, sname);
			paraname = rname;
			paraname += geoModel::ModelParamFile_Suffix;

			ipl::load_geoModel_params(paraname, /*bbmin, bbmax,*/ mCoef);
			memcpy(bbmin, mCoef.bbox.min_pt, sizeof(double) * 3);
			memcpy(bbmax, mCoef.bbox.max_pt, sizeof(double) * 3);

			//计算clipping box
			if (target_bbox_.min_pt[0] > bbmin[0])
				target_bbox_.min_pt[0] = bbmin[0];
			if (target_bbox_.min_pt[1] > bbmin[1])
				target_bbox_.min_pt[1] = bbmin[1];
			if (target_bbox_.min_pt[2] > bbmin[2])
				target_bbox_.min_pt[2] = bbmin[2];

			if (target_bbox_.max_pt[0] < bbmax[0])
				target_bbox_.max_pt[0] = bbmax[0];
			if (target_bbox_.max_pt[1] < bbmax[1])
				target_bbox_.max_pt[1] = bbmax[1];
			if (target_bbox_.max_pt[2] < bbmax[2])
				target_bbox_.max_pt[2] = bbmax[2];


			//提取分割点云
			pcl::PointCloud<PointT>::Ptr seg_cloud(new pcl::PointCloud <PointT>);
			ipl::read_PointCloud(tar_name_[i], *seg_cloud);

			ipl::PointGeoSegment<PointT> seg;

			seg.setInputCloud(seg_cloud);

			seg.setModelCoef(mCoef);

			tar_segments_.push_back(seg);
		}

		std::cout << std::endl;

		std::cout << "bounding box:" << std::endl;
		std::cout << target_bbox_.min_pt[0] << " " << target_bbox_.min_pt[1] << " " << target_bbox_.min_pt[2] << std::endl;
		std::cout << target_bbox_.max_pt[0] << " " << target_bbox_.max_pt[1] << " " << target_bbox_.max_pt[2] << std::endl;

		return;
	}

	template <typename PointT> void
		PointCloudClipper<PointT>::set_Transformation(const std::string filename)
	{
		Eigen::Matrix4d trans_mat;
		double rms;
		bool bRet = load_RigidTransParam(filename, /*origin_, */trans_mat, rms);

		if (!bRet)
			std::cout << "error! can't read file: " << filename << std::endl;

		transform_ = trans_mat;
	}

	template <typename PointT> void
		PointCloudClipper<PointT>::do_clip()
	{
		if (bRefClipped_)
			return;

		//计算clipping box  需要对target的bbox进行变换
		Eigen::Vector4d bb_pts[4], trans_bb_pts[4];  //boundingbox的4个端点

		bb_pts[0](0) = target_bbox_.min_pt[0];// -origin_[0];
		bb_pts[0](1) = target_bbox_.min_pt[1];// -origin_[1];
		bb_pts[0](2) = 0;
		bb_pts[0](3) = 1;

		bb_pts[1](0) = target_bbox_.max_pt[0];// -origin_[0];
		bb_pts[1](1) = target_bbox_.min_pt[1];// -origin_[1];
		bb_pts[1](2) = 0;
		bb_pts[1](3) = 1;

		bb_pts[2](0) = target_bbox_.max_pt[0];// -origin_[0];
		bb_pts[2](1) = target_bbox_.max_pt[1];// -origin_[1];
		bb_pts[2](2) = 0;
		bb_pts[2](3) = 1;

		bb_pts[3](0) = target_bbox_.min_pt[0];// -origin_[0];
		bb_pts[3](1) = target_bbox_.max_pt[1];// -origin_[1];
		bb_pts[3](2) = 0;
		bb_pts[3](3) = 1;


		clip_box_.max_pt[0] = std::numeric_limits<double>::lowest();
		clip_box_.max_pt[1] = std::numeric_limits<double>::lowest();
		clip_box_.min_pt[0] = std::numeric_limits<double>::max();
		clip_box_.min_pt[1] = std::numeric_limits<double>::max();

		for (int i = 0; i < 4; ++i)
		{
			trans_bb_pts[i] = transform_*bb_pts[i];

			if (clip_box_.min_pt[0] > trans_bb_pts[i](0))
				clip_box_.min_pt[0] = trans_bb_pts[i](0);
			if (clip_box_.min_pt[1] > trans_bb_pts[i](1))
				clip_box_.min_pt[1] = trans_bb_pts[i](1);
			if (clip_box_.max_pt[0] < trans_bb_pts[i](0))
				clip_box_.max_pt[0] = trans_bb_pts[i](0);
			if (clip_box_.max_pt[1] < trans_bb_pts[i](1))
				clip_box_.max_pt[1] = trans_bb_pts[i](1);
		}

		clip_box_.min_pt[0] -= search_buf_size_;
		clip_box_.min_pt[1] -= search_buf_size_;
		clip_box_.max_pt[0] += search_buf_size_;
		clip_box_.max_pt[1] += search_buf_size_;

		return do_clip(clip_box_);
	}

	template <typename PointT> void
		PointCloudClipper<PointT>::do_clip(const ipl::geoModel::BoundingBox &bbox)
	{
		if (bRefClipped_)
			return;

		memcpy(clip_box_.min_pt, bbox.min_pt, sizeof(double) * 3);
		memcpy(clip_box_.max_pt, bbox.max_pt, sizeof(double) * 3);

		iplRECT<double> clip_rect;
		clip_rect.m_xmin = clip_box_.min_pt[0];
		clip_rect.m_ymin = clip_box_.min_pt[1];
		clip_rect.m_xmax = clip_box_.max_pt[0];
		clip_rect.m_ymax = clip_box_.max_pt[1];

		std::string rname, sname;
		std::string paraname;

		size_t total_num = 0, clipped_num = 0;
		double bbmin[3], bbmax[3];
		ipl::geoModel::geoModelInfo mCoef;
		size_t nWalls = ref_name_.size();
		ref_segments_.clear();
		for (size_t i = 0; i < nWalls; ++i)
		{
			std::cout << "clip reference walls.... " << i + 1 << " / " << nWalls << "\r";

			//提取分割点云和模型参数
			ipl::extract_file_name(ref_name_[i], rname, sname);
			paraname = rname;
			paraname += geoModel::ModelParamFile_Suffix;

			ipl::load_geoModel_params(paraname, /*bbmin, bbmax,*/ mCoef);
			memcpy(bbmin, mCoef.bbox.min_pt, sizeof(double) * 3);
			memcpy(bbmax, mCoef.bbox.max_pt, sizeof(double) * 3);
			//计算clipping box
			iplRECT<double> wrect;
			wrect.m_xmin = bbmin[0];
			wrect.m_ymin = bbmin[1];
			wrect.m_xmax = bbmax[0];
			wrect.m_ymax = bbmax[1];

			if (!clip_rect.isIntersect(wrect))
				continue;

			//提取分割点云
			ipl::PointGeoSegment<PointT> seg;
			seg.setModelCoef(mCoef);

			pcl::PointCloud<PointT>::Ptr seg_cloud(new pcl::PointCloud <PointT>);
			ipl::read_PointCloud(ref_name_[i], *seg_cloud);

			total_num += seg_cloud->size();

			if (bbmin[0] > clip_box_.min_pt[0]
				&& bbmin[1] > clip_box_.min_pt[1]
				&& bbmax[0] < clip_box_.max_pt[0]
				&& bbmax[1] < clip_box_.max_pt[1])
			{
				seg.setInputCloud(seg_cloud);
				ref_segments_.push_back(seg);
				clipped_num += seg_cloud->size();
			}
			else
			{//clip the target ceiling points
				std::cout << std::endl << "clip points... ";
				ref_ptr<iplPointCloud<PointT> > clip_cloud(new iplPointCloud <PointT>);
				for (size_t j = 0; j < seg_cloud->size(); ++j)
				{
					PointT pt = seg_cloud->points[j];
					if (pt.x < clip_box_.min_pt[0]
						|| pt.x > clip_box_.max_pt[0]
						|| pt.y < clip_box_.min_pt[1]
						|| pt.y > clip_box_.max_pt[1])
						continue;

					clip_cloud->push_back(pt);
				}
				std::cout << clip_cloud->size() << "/" << seg_cloud->size() << std::endl;

				if (clip_cloud->size() > 0)
				{
					seg.setInputCloud(clip_cloud);
					ref_segments_.push_back(seg);
					clipped_num += clip_cloud->size();
				}
			}

		}

		std::cout << std::endl;
		std::cout << "clipped: " << ref_segments_.size() << "/" << nWalls
			<< " clipped points: " << clipped_num << "/" << total_num << std::endl << std::endl;

		bRefClipped_ = true;
	}

	template <typename PointT> void
		PointCloudClipper<PointT>::getTargetGroup(geoModel::PointGroup<PointT> &tar_group)
	{
		tar_group.cloud.reset(new iplPointCloud<PointT>);
		//tar_group.cloud->clear();
		tar_group.lut.clear();
		tar_group.modelParams.clear();

		for (size_t i = 0; i < tar_segments_.size(); ++i)
		{
			ipl::geoModel::geoModelInfo mCoef;
			mCoef = tar_segments_[i].getModelCoef();
			ref_ptr<iplPointCloud<PointT> const> cloud = tar_segments_[i].getInputCloud();

			tar_group.modelParams.push_back(mCoef);
			for (size_t j = 0; j < cloud->size(); ++j)
			{
				tar_group.cloud->points.push_back(cloud->points[j]);
				tar_group.lut.push_back(i);
			}
		}

		tar_group.cloud->height = 1;
		tar_group.cloud->width = tar_group.cloud->size();
	}

	template <typename PointT> void
		PointCloudClipper<PointT>::getClippedRefGroup(geoModel::PointGroup<PointT> &clipped_group)
	{
		clipped_group.cloud.reset(new iplPointCloud<PointT>);
		//clipped_group.cloud->clear();
		clipped_group.lut.clear();
		clipped_group.modelParams.clear();

		for (size_t i = 0; i < ref_segments_.size(); ++i)
		{
			ipl::geoModel::geoModelInfo mCoef;
			mCoef = ref_segments_[i].getModelCoef();
			ref_ptr<iplPointCloud<PointT> const> cloud = ref_segments_[i].getInputCloud();

			clipped_group.modelParams.push_back(mCoef);
			for (size_t j = 0; j < cloud->size(); ++j)
			{
				clipped_group.cloud->points.push_back(cloud->points[j]);
				clipped_group.lut.push_back(i);
			}
		}

		clipped_group.cloud->height = 1;
		clipped_group.cloud->width = clipped_group.cloud->size();
	}
}

