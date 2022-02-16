#pragma once
#include "MapUpdate/CandidateChangeExtractor.h"

#include <Eigen/Dense>
#include <pcl/common/transforms.h>


namespace ipl
{
	template <typename PointT>
	CandidateChangeExtractor<PointT>::CandidateChangeExtractor()
	{
		transform_ = Eigen::Matrix4d::Identity();
		
		disTH_ = 0.2;
	
		data_clipper_.reset(new PointCloudClipper<PointT>);
	}

	template <typename PointT>
	CandidateChangeExtractor<PointT>::~CandidateChangeExtractor()
	{

	}

	template <typename PointT> void
		CandidateChangeExtractor<PointT>::set_refData(const std::vector<std::string> &pszRefFiles)
	{
		data_clipper_->set_refData(pszRefFiles);
	}

	template <typename PointT> void
		CandidateChangeExtractor<PointT>::set_tarData(const std::vector<std::string> &pszTarFiles)
	{
		data_clipper_->set_tarData(pszTarFiles);
	}

	template <typename PointT> void
		CandidateChangeExtractor<PointT>::set_Transformation(const std::string filename)
	{
		data_clipper_->set_Transformation(filename);

		transform_.matrix() = data_clipper_->getTransformation();
	}

	template <typename PointT> void
		CandidateChangeExtractor<PointT>::extractCandidateChange(geoModel::PointGroup<PointT> &cand_group)
	{
		geoModel::PointGroup<PointT> ref_group, tar_group;

		data_clipper_->do_clip();
		data_clipper_->getClippedRefGroup(ref_group);
		data_clipper_->getTargetGroup(tar_group);

		trans_cloud_.reset(new iplPointCloud<PointT>);
		pcl::transformPointCloud(*(tar_group.cloud), *trans_cloud_, transform_);

		search_.reset(new pcl::search::KdTree<PointT>);
		search_->setInputCloud(ref_group.cloud);

		cand_group.cloud.reset(new iplPointCloud<PointT>);
//		cand_group.cloud->clear();
		cand_group.lut.clear();
		cand_group.modelParams = tar_group.modelParams;

		//≤È’“unregistration point
		double sqr_disTh = disTH_*disTH_;
		for (size_t i = 0; i < tar_group.cloud->size(); ++i)
		{
			PointT searchPoint;
			searchPoint = trans_cloud_->points[i];
	
			std::vector<int> k_indices;
			std::vector<float> k_sqr_distances;
			search_->nearestKSearch(searchPoint, 1, k_indices, k_sqr_distances);

			if (k_sqr_distances[0] > sqr_disTh)
			{
				cand_group.cloud->push_back(searchPoint);
				cand_group.lut.push_back(tar_group.lut[i]);
			}
		}

		cand_group.cloud->height = 1;
		cand_group.cloud->width = cand_group.cloud->size();

		return;
	}
}
