#pragma once

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>
//#include <pcl/io/pcd_io.h>
#include "pcl/impl/instantiate.hpp"

#include <Eigen/Dense>

#include "registration/ipGraphMatching.h"
#include "geometry/RotateMat.h"
#include "feature/FeatureIO.h"
#include "fitting/ModelParamsIO.h"
#include "io/PointcloudIO.h"

#include "commonAPIs/iplstring.h"

//PCL_INSTANTIATE(ptSegment, pcl::PointXYZ);
//template class ptSegment<pcl::PointXYZ>;

namespace ipl
{
	ipGraphMatching::ipGraphMatching()
	{
		search_radius_ = 5.0;

		cur_matchID_ = -1;
		cur_match_cost_ = std::numeric_limits<double>::max();

		//	bfine_trans = false;
		transform_ = Eigen::Matrix4d::Identity();
		//	icp_transform_ = Eigen::Matrix4d::Identity();
		//	fine_transform_ = Eigen::Matrix4d::Identity();
	}

	ipGraphMatching::ipGraphMatching(std::string refname, std::string targetname)
	{
		ref_ipfname_ = refname;
		target_ipfname_ = targetname;

		ipGraphMatching();
	}

	ipGraphMatching::~ipGraphMatching()
	{
		ref_ip2ds_.clear();
		target_ip2ds_.clear();

		ref_segfiles_.clear();
		target_segfiles_.clear();

		ref_segments_.clear();
		target_segments_.clear();

		if (search_ != 0)
			search_.reset();

		if (ipt_cloud_ != 0)
			ipt_cloud_.reset();

		cand_indices_.clear();
	}

	void ipGraphMatching::set_ipf(std::string refname, std::string targetname)
	{
		ref_ipfname_ = refname;
		target_ipfname_ = targetname;
	}

	void ipGraphMatching::set_search_radius(float r)
	{
		search_radius_ = r;
	}

	bool ipGraphMatching::start_match()
	{
		int bRefOK, btargetOK;

		//1. load ipt
		bRefOK = load_IntersectionPoint2D(ref_ipfname_.c_str(), ref_segfiles_, 
			ref_bbox_.min_pt, ref_bbox_.max_pt, ref_ip2ds_);
		
		btargetOK = load_IntersectionPoint2D(target_ipfname_.c_str(), target_segfiles_, 
			target_bbox_.min_pt, target_bbox_.max_pt, target_ip2ds_);

		if (bRefOK != 0 || btargetOK != 0)
			return false;

		//2. load segment coefficients
		int nRefSegs = ref_segfiles_.size();
		ref_segments_.resize(nRefSegs);

		for (int i = 0; i < nRefSegs; i++)
		{
			//提取分割点云和模型参数
			std::string rname, sname;
			extract_file_name(ref_segfiles_[i].c_str(), rname, sname);
			std::string paraname = rname;
			paraname += geoModel::ModelParamFile_Suffix;

			geoModel::geoModelInfo mCoef;
			//geoModel::geoModelType type;
			double bbmin[3], bbmax[3];
			load_geoModel_params(paraname.c_str(), /*bbmin, bbmax,*/ mCoef);
			memcpy(bbmin, mCoef.bbox.min_pt, sizeof(double) * 3);
			memcpy(bbmax, mCoef.bbox.max_pt, sizeof(double) * 3);
			//		planeCoeffs.push_back(mCoef);

			//提取分割点云
			//pcl::PCDReader pcdReader;
// 			ref_ptr<iplPointCloud<iplPointXYZ> > seg_cloud(new iplPointCloud <iplPointXYZ>);
// 			read_PointCloud(ref_segfiles_[i], *seg_cloud);
// 
// 			PointGeoSegment<iplPointXYZ> seg;
// 
// 			seg.setInputCloud(seg_cloud);
// 
// 			seg.setModelCoef(mCoef);

			ref_segments_[i] = mCoef;
		}

		int ntargetSegs = target_segfiles_.size();
		target_segments_.resize(ntargetSegs);

		for (int i = 0; i < ntargetSegs; i++)
		{
			//提取分割点云和模型参数
			std::string rname, sname;
			extract_file_name(target_segfiles_[i].c_str(), rname, sname);
			std::string paraname = rname;
			paraname += geoModel::ModelParamFile_Suffix;

			geoModel::geoModelInfo mCoef;
			//geoModel::geoModelType type;
			double bbmin[3], bbmax[3];
			load_geoModel_params(paraname.c_str(), /*bbmin, bbmax,*/ mCoef);
			memcpy(bbmin, mCoef.bbox.min_pt, sizeof(double) * 3);
			memcpy(bbmax, mCoef.bbox.max_pt, sizeof(double) * 3);
			//		planeCoeffs.push_back(mCoef);

			//提取分割点云
// 			ref_ptr<iplPointCloud<iplPointXYZ> > seg_cloud(new iplPointCloud <iplPointXYZ>);
// 			read_PointCloud(target_segfiles_[i], *seg_cloud);
// 			
// 			PointGeoSegment<iplPointXYZ> seg;
// 
// 			seg.setInputCloud(seg_cloud);
// 
// 			seg.setModelCoef(mCoef);

			target_segments_[i] = mCoef;
		}

		//clip the ref. ip2ds
		BoundingBox clip_bb;
		clip_bb = target_bbox_;

		clip_bb.min_pt[0] -= search_radius_;
		clip_bb.min_pt[1] -= search_radius_;
		clip_bb.max_pt[0] += search_radius_;
		clip_bb.max_pt[1] += search_radius_;

		int n_refipt = ref_ip2ds_.size();
		clipped_ref_ip2ds_.clear();
		for (int i = 0; i < n_refipt; i++)
		{
			if (ref_ip2ds_[i].p[0] < clip_bb.min_pt[0]
				|| ref_ip2ds_[i].p[0] > clip_bb.max_pt[0]
				|| ref_ip2ds_[i].p[1] < clip_bb.min_pt[1]
				|| ref_ip2ds_[i].p[1] > clip_bb.max_pt[1])
				continue;

			clipped_ref_ip2ds_.push_back(ref_ip2ds_[i]);
		}

		//3. build ref ipt tree
		if (ipt_cloud_ == 0)
			ipt_cloud_ = ref_ptr<pcl::PointCloud<pcl::PointXYZ> >(new pcl::PointCloud<pcl::PointXYZ>);

		size_t n_clipped_ip2ds = clipped_ref_ip2ds_.size();
		ipt_cloud_->points.resize(n_clipped_ip2ds);
		for (int i = 0; i < n_clipped_ip2ds; i++)
		{
			//memcpy(ipt_cloud_->points[i].data, ref_ip2ds_[i].p, sizeof(float) * 4);
			
			ipt_cloud_->points[i].x = clipped_ref_ip2ds_[i].p[0];
 			ipt_cloud_->points[i].y = clipped_ref_ip2ds_[i].p[1];
 			ipt_cloud_->points[i].z = clipped_ref_ip2ds_[i].p[2];
		}
		ipt_cloud_->height = 1;
		ipt_cloud_->width = ipt_cloud_->size();

		if (search_ == 0)
			search_ = ref_ptr<pcl::search::Search<pcl::PointXYZ> >(new pcl::search::KdTree<pcl::PointXYZ>);

		search_->setInputCloud(ipt_cloud_);

		//4.find the principal point in unregistering ipt
		target_principal_ = target_ip2ds_[0];

		//find the principal edge of target ip
		double maxEdge = std::numeric_limits<double>::lowest();
		double ptS[3], ptE[3];
		int princEdgeID = -1;
		for (int i = 0; i < target_principal_.connIndices.size(); i++)
		{//选择长边作为principal edge
			int edgeId = target_principal_.connIndices[i];

			iplPOINT3D diff_coord;
			diff_coord.X = target_principal_.connAtts[i].ptE[0] - target_principal_.connAtts[i].ptS[0];
			diff_coord.Y = target_principal_.connAtts[i].ptE[1] - target_principal_.connAtts[i].ptS[1];
			diff_coord.Z = target_principal_.connAtts[i].ptE[2] - target_principal_.connAtts[i].ptS[2];

			double edge_len = diff_coord.length2();
			if (maxEdge < edge_len)
			{
				maxEdge = edge_len;
				princEdgeID = edgeId;
				memcpy(ptS, target_principal_.connAtts[i].ptS, sizeof(double) * 3);
				memcpy(ptE, target_principal_.connAtts[i].ptE, sizeof(double) * 3);
			}
		}

		//确定principal edge 的方向向量
		double org[3];
		//	Eigen::Vector3f pe_normal;//principal edge's direction
		org[0] = target_principal_.p[0];
		org[1] = target_principal_.p[1];
		org[2] = target_principal_.p[2];

		get_ray_direction(org, ptS, ptE, target_segments_[princEdgeID].coef, princ_normal_);

		//5. clip candidates
		pcl::PointXYZ searchPoint;
		searchPoint.x = target_ip2ds_[0].p[0];
		searchPoint.y = target_ip2ds_[0].p[1];
		searchPoint.z = target_ip2ds_[0].p[2];
		//memcpy(searchPoint.data, target_ip2ds_[0].p, sizeof(float) * 4);  //the first point is the principal point

		std::vector<float> pointNKNSquaredDistance;
		cand_indices_.clear();
		search_->radiusSearch(searchPoint, search_radius_, cand_indices_, pointNKNSquaredDistance);

		std::cout << "candidates: " << cand_indices_.size() << std::endl;

		
		//6. preparations for ip2d matching
		cur_matchID_ = -1;
		cur_match_cost_ = std::numeric_limits<double>::max();

		transform_ = Eigen::Matrix4d::Identity();

		size_t nTar_ip2d = target_ip2ds_.size();
		Vx_.resize(nTar_ip2d);
		Vy_.resize(nTar_ip2d);

		WeightX_.resize(nTar_ip2d);
		WeightY_.resize(nTar_ip2d);
		nor_WeightX_.resize(nTar_ip2d);
		nor_WeightY_.resize(nTar_ip2d);
		sumWX_ = sumWY_ = 0;
		for (size_t i = 0; i < nTar_ip2d; ++i)
		{
			WeightX_[i] = 1.0 / target_ip2ds_[i].coefVar[0];
			WeightY_[i] = 1.0 / target_ip2ds_[i].coefVar[1];

			sumWX_ += WeightX_[i];
			sumWY_ += WeightY_[i];
		}

		for (size_t i = 0; i < nTar_ip2d; ++i)
		{
			nor_WeightX_[i] = WeightX_[i] / sumWX_;
			nor_WeightY_[i] = WeightY_[i] / sumWY_;
		}

		//	icp_transform_ = Eigen::Matrix4d::Identity();

		return true;
	}

	//贪婪法逐个与cand_indices_中的点比较
	bool ipGraphMatching::match_next()
	{
		if (cur_matchID_ < -1 || cur_matchID_ >= static_cast<int> (cand_indices_.size() - 1))
			return false;

		cur_matchID_++;

		feature::IntersectionPoint   cand_ip;
		cand_ip = clipped_ref_ip2ds_[cand_indices_[cur_matchID_]];

		double xOffset, yOffset, zOffset;
		xOffset = cand_ip.p[0];
		yOffset = cand_ip.p[1];
		zOffset = /*cand_ip.p[2]*/0;


		cur_match_cost_ = std::numeric_limits<double>::max();
		double mCur_Cost;
// 		pcl::PointXYZ org;
// 		org.x = cand_ip.p[0]; org.y = cand_ip.p[1]; org.z = cand_ip.p[2];
		for (int i = 0; i < cand_ip.connIndices.size(); i++)
		{//和每一条edge匹配，并计算匹配代价，将匹配代价最小最为当前点的最佳匹配
			int edgeId = cand_ip.connIndices[i];

			//计算当前参考边的方向
			Eigen::Vector2d cur_eNormal;//current edge's direction

			get_ray_direction(cand_ip.p, cand_ip.connAtts[i].ptS, cand_ip.connAtts[i].ptS, ref_segments_[edgeId].coef,
				cur_eNormal);

			//ref_segments_[edgeId].getProjectionLineDirection(org, cur_eNormal);

			//计算绕Z轴的旋转角
			double kappa = get2DRotateAngle(princ_normal_, cur_eNormal);

			//构造转换矩阵
			Eigen::Affine3d  cur_transform = Eigen::Affine3d::Identity();
			//		Eigen::Matrix3d  rotate_mat;

			Eigen::AngleAxisd rotZ(kappa, Eigen::Vector3d::UnitZ());
			Eigen::AngleAxisd rotY(0, Eigen::Vector3d::UnitY());
			Eigen::AngleAxisd rotX(0, Eigen::Vector3d::UnitX());

			Eigen::Quaternion<double> q = rotZ * rotY * rotX;
			Eigen::Matrix3d rotationMatrix = q.matrix();

			Eigen::Vector3d org, deltaT;
			org[0] = -target_ip2ds_[0].p[0];
			org[1] = -target_ip2ds_[0].p[1];
			org[2] = -target_ip2ds_[0].p[2];

			deltaT = rotationMatrix*org;
			//xOffset -= deltaT[0]; yOffset -= deltaT[1]; zOffset -= deltaT[2];

			cur_transform(0, 3) = xOffset + deltaT[0];
			cur_transform(1, 3) = yOffset + deltaT[1];
			cur_transform(2, 3) = zOffset + deltaT[2];
			cur_transform(3, 3) = 1;

			cur_transform.matrix().topLeftCorner(3, 3) = q.toRotationMatrix();
// 			cur_transform.translation() << xOffset, yOffset, zOffset;
// 			cur_transform.rotate(Eigen::AngleAxisd(kappa, Eigen::Vector3d::UnitZ()));

			std::vector<feature::IntersectionPoint> trans_ipts;
			transform_ipts(target_ip2ds_, trans_ipts, cur_transform);

			//计算当前转换矩阵下的匹配代价
			mCur_Cost = 0;
			for (int j = 0; j < trans_ipts.size(); j++)
			{
				pcl::PointXYZ searchPoint;
				searchPoint.x = trans_ipts[j].p[0];
				searchPoint.y = trans_ipts[j].p[1];
				searchPoint.z = trans_ipts[j].p[2];
				//memcpy(searchPoint.data, trans_ipts[j].p, sizeof(float) * 4);  //the first point is the principal point

				std::vector<int> k_indices;
				std::vector<float> k_sqr_distances;
				search_->nearestKSearch(searchPoint, 1, k_indices, k_sqr_distances);

				mCur_Cost += k_sqr_distances[0];
			}
			mCur_Cost /= trans_ipts.size();
			mCur_Cost = sqrt(mCur_Cost);

			if (cur_match_cost_ > mCur_Cost)
			{
				cur_match_cost_ = mCur_Cost;
				transform_ = cur_transform;
			}
		}

		return true;
	}

	bool ipGraphMatching::weighted_match_next()
	{
		if (cur_matchID_ < -1 || cur_matchID_ >= static_cast<int> (cand_indices_.size() - 1))
			return false;

		cur_matchID_++;

		feature::IntersectionPoint   cand_ip;
		cand_ip = clipped_ref_ip2ds_[cand_indices_[cur_matchID_]];

		double xOffset, yOffset, zOffset;
		xOffset = cand_ip.p[0];
		yOffset = cand_ip.p[1];
		zOffset = cand_ip.p[2];


		cur_match_cost_ = std::numeric_limits<double>::max();
		double mCur_Cost, wei_Cost;
		std::vector<double> curVx, curVy;
		size_t ntaget_ip2d = target_ip2ds_.size();

		curVx.resize(ntaget_ip2d);
		curVy.resize(ntaget_ip2d);
		// 		pcl::PointXYZ org;
		// 		org.x = cand_ip.p[0]; org.y = cand_ip.p[1]; org.z = cand_ip.p[2];
		for (int i = 0; i < cand_ip.connIndices.size(); i++)
		{//和每一条edge匹配，并计算匹配代价，将匹配代价最小最为当前点的最佳匹配
			int edgeId = cand_ip.connIndices[i];

			//计算当前参考边的方向
			Eigen::Vector2d cur_eNormal;//current edge's direction

			get_ray_direction(cand_ip.p, cand_ip.connAtts[i].ptS, cand_ip.connAtts[i].ptS, ref_segments_[edgeId].coef,
				cur_eNormal);

			//ref_segments_[edgeId].getProjectionLineDirection(org, cur_eNormal);

			//计算绕Z轴的旋转角
			double kappa = get2DRotateAngle(princ_normal_, cur_eNormal);

			//构造转换矩阵
			Eigen::Affine3d  cur_transform = Eigen::Affine3d::Identity();
			//		Eigen::Matrix3d  rotate_mat;

			// 		RotateMat_Z(kappa, rotate_mat);
			// 
			// 		cur_transform(0, 0) = rotate_mat(0, 0);
			// 		cur_transform(0, 1) = rotate_mat(0, 1);
			// 		cur_transform(0, 2) = rotate_mat(0, 2);
			// 		cur_transform(1, 0) = rotate_mat(1, 0);
			// 		cur_transform(1, 1) = rotate_mat(1, 1);
			// 		cur_transform(1, 2) = rotate_mat(1, 2);
			// 		cur_transform(2, 0) = rotate_mat(2, 0);
			// 		cur_transform(2, 1) = rotate_mat(2, 1);
			// 		cur_transform(2, 2) = rotate_mat(2, 2);
			// 
			// 		cur_transform(0, 3) = xOffset;
			// 		cur_transform(1, 3) = yOffset;
			// 		cur_transform(2, 3) = zOffset;

			//		Eigen::Affine3d transform_2 = Eigen::Affine3d::Identity();
			Eigen::AngleAxisd rotZ(kappa, Eigen::Vector3d::UnitZ());
			Eigen::AngleAxisd rotY(0, Eigen::Vector3d::UnitY());
			Eigen::AngleAxisd rotX(0, Eigen::Vector3d::UnitX());

			Eigen::Quaternion<double> q = rotZ * rotY * rotX;
			Eigen::Matrix3d rotationMatrix = q.matrix();
	
			Eigen::Vector3d org, deltaT;
			org[0] = -target_ip2ds_[0].p[0];
			org[1] = -target_ip2ds_[0].p[1]; 
			org[2] = -target_ip2ds_[0].p[2];

			deltaT = rotationMatrix*org;
			//xOffset -= deltaT[0]; yOffset -= deltaT[1]; zOffset -= deltaT[2];
			
			cur_transform(0, 3) = xOffset + deltaT[0];
			cur_transform(1, 3) = yOffset + deltaT[1];
			cur_transform(2, 3) = zOffset + deltaT[2];
			cur_transform(3, 3) = 1;

			cur_transform.matrix().topLeftCorner(3, 3) = q.toRotationMatrix();
			

			std::vector<feature::IntersectionPoint> trans_ipts;
			transform_ipts(target_ip2ds_, trans_ipts, cur_transform);

			//计算当前转换矩阵下的匹配代价
			mCur_Cost = 0;
			wei_Cost = 0;
			for (int j = 0; j < ntaget_ip2d; j++)
			{
				pcl::PointXYZ searchPoint;
				searchPoint.x = trans_ipts[j].p[0];
				searchPoint.y = trans_ipts[j].p[1];
				searchPoint.z = trans_ipts[j].p[2];
				//memcpy(searchPoint.data, trans_ipts[j].p, sizeof(float) * 4);  //the first point is the principal point

				std::vector<int> k_indices;
				std::vector<float> k_sqr_distances;
				search_->nearestKSearch(searchPoint, 1, k_indices, k_sqr_distances);

				mCur_Cost += k_sqr_distances[0];

				curVx[i] = trans_ipts[j].p[0] - clipped_ref_ip2ds_[k_indices[0]].p[0];
				curVy[i] = trans_ipts[j].p[1] - clipped_ref_ip2ds_[k_indices[0]].p[1];

				wei_Cost += curVx[i] * curVx[i] * nor_WeightX_[i];
				wei_Cost += curVy[i] * curVy[i] * nor_WeightY_[i];
			}
			//等权sigma2
			mCur_Cost /= trans_ipts.size();
			mCur_Cost = sqrt(mCur_Cost);

			//weighted-sigma2
			//wei_Cost /= ntaget_ip2d;
			wei_Cost = sqrt(wei_Cost);

			if (cur_match_cost_ > wei_Cost)
			{
				cur_match_cost_ = wei_Cost;
				transform_ = cur_transform;
			}
		}

		return true;
	}

	inline void ipGraphMatching::transform_ipts(std::vector<feature::IntersectionPoint> src_ipts,
		std::vector<feature::IntersectionPoint> &dst_ipts, Eigen::Affine3d trans_mat)
	{
		pcl::PointCloud<pcl::PointXYZ> input, output;

		input.points.resize(src_ipts.size());
		input.width = input.size();
		input.height = 1;
		for (int i = 0; i < src_ipts.size(); i++)
		{
			input.points[i].x = src_ipts[i].p[0]/* - src_ipts[0].p[0]*/;
			input.points[i].y = src_ipts[i].p[1]/* - src_ipts[0].p[1]*/;
			input.points[i].z = src_ipts[i].p[2]/* - src_ipts[0].p[2]*/;
		}

		pcl::transformPointCloud(input, output, trans_mat);

		dst_ipts = src_ipts;
		for (int i = 0; i < dst_ipts.size(); i++)
		{
			dst_ipts[i].p[0] = output.points[i].x;
			dst_ipts[i].p[1] = output.points[i].y;
			dst_ipts[i].p[2] = output.points[i].z;
		}
	}

	// template <typename PointT> void
	// ipGraphMatching::transformPointCloud(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out)
	// {
	// 	for (int i = 0; i < cloud_in.size(); i++)
	// 	{
	// 		cloud_in.points[i].data[0] -= target_ip2ds_[0].p[0];
	// 		cloud_in.points[i].data[1] -= target_ip2ds_[0].p[1];
	// 		cloud_in.points[i].data[2] -= target_ip2ds_[0].p[2];
	// 	}
	// 
	// 	pcl::transformPointCloud(cloud_in, cloud_out, transform_);
	// }

	inline void ipGraphMatching::get_ray_direction(const double pt[3], const double ptS[3], const double ptE[3],
		const iplModelCoeffs coef, Eigen::Vector2d &normal)
	{
		Eigen::Vector2d t_normal;

		//initial direction
		normal[0] = -coef.values[1];
		normal[1] = coef.values[0];
		//	normal[2] = 1.0;

		double dis2 = (ptS[0] - pt[0])*(ptS[0] - pt[0]) + (ptS[1] - pt[1])*(ptS[1] - pt[1]);

		//找出长边方向
		if (dis2 < (ptE[0] - pt[0])*(ptE[0] - pt[0]) + (ptE[1] - pt[1])*(ptE[1] - pt[1]))
		{
			t_normal[0] = ptE[0] - pt[0];
			t_normal[1] = ptE[1] - pt[1];
		}
		else
		{
			t_normal[0] = ptS[0] - pt[0];
			t_normal[1] = ptS[1] - pt[1];
		}

		//利用点积判断方向
		if (normal[0] * t_normal[0] + normal[1] * t_normal[1] < 0)
		{
			normal[0] *= -1;
			normal[1] *= -1;
		}
		return;
	}

}
