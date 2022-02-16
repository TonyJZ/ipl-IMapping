#pragma once

#include "core/iplcore.h"
#include "feature/FeatureDef.h"
#include "fitting/geoModelDef.h"
//#include "feature/PointGeoSegment.h"

//PCL
#include <pcl/search/search.h>

namespace ipl
{
	class IPL_BASE_API ipGraphMatching
	{
	public:
		//	typedef pcl::PointXYZ pcl::PointXYZ;
		typedef struct 
		{
			double min_pt[3];
			double max_pt[3];
		} BoundingBox;

		typedef pcl::search::Search <pcl::PointXYZ> KdTree;
		typedef typename KdTree::Ptr KdTreePtr;
		typedef boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > PointCloudPtr;

		ipGraphMatching(std::string refname, std::string targetname);
		ipGraphMatching();
		~ipGraphMatching();

		void set_ipf(std::string refname, std::string targetname);
		void set_search_radius(float r);

		double get_match_cost() {
			return cur_match_cost_;
		};

		Eigen::Affine3d  get_transform() {
			return transform_;
		};

		void get_normalized_org(double org[3]) {
			org[0] = target_ip2ds_[0].p[0];
			org[1] = target_ip2ds_[0].p[1];
			org[2] = target_ip2ds_[0].p[2];
		};

		//匹配前必须先调用
		bool start_match();

		//视同等精度观测
		bool match_next();

		//顾及观测值的精度: 精度推导 (plane  ->  intersection  ->  2D regist result)
		bool weighted_match_next();

		// 	template <typename PointT> void
		// 		transformPointCloud(const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out);

		//	bool do_ICP();

	protected:
		inline void transform_ipts(std::vector<feature::IntersectionPoint> src_ipts, std::vector<feature::IntersectionPoint> &dst_ipts, 
			Eigen::Affine3d  trans_mat);

		//提取射线方向
		inline void get_ray_direction(const double pt[3], const double ptS[3], const double ptE[3], 
			const iplModelCoeffs coef, Eigen::Vector2d &normal);

	private:
		//ipf name
		std::string ref_ipfname_;
		std::string target_ipfname_;

		//seg names
		std::vector<std::string> ref_segfiles_;
		std::vector<std::string> target_segfiles_;

		//bounding box 
		BoundingBox ref_bbox_;
		BoundingBox target_bbox_;

		//ipt
		std::vector<feature::IntersectionPoint> ref_ip2ds_;
		std::vector<feature::IntersectionPoint> clipped_ref_ip2ds_;
		std::vector<feature::IntersectionPoint> target_ip2ds_;

		//the principal point of unregistering ipt
		feature::IntersectionPoint target_principal_;
		Eigen::Vector2d princ_normal_;


		//segs
// 		std::vector<ipl::PointGeoSegment<pcl::PointXYZ> > ref_segments_;
// 		std::vector<ipl::PointGeoSegment<pcl::PointXYZ> > target_segments_;

		//plane coefficients
		std::vector<ipl::geoModel::geoModelInfo> ref_segments_;
		std::vector<ipl::geoModel::geoModelInfo> target_segments_;

		//ref candidates
		KdTreePtr search_;  //for all ref ipts
		PointCloudPtr  ipt_cloud_;

		float search_radius_;
		std::vector<int> cand_indices_;

		int cur_matchID_;  //对应的ref ipt ID
		double cur_match_cost_;   //stddev of the distances between corresponding points

		std::vector<double> Vx_, Vy_; // residual errors of registration
		std::vector<double> WeightX_, WeightY_;
		std::vector<double> nor_WeightX_, nor_WeightY_;   //normalized weight
		double sumWX_, sumWY_;

								  //coarse to fine registration
								  //	bool bfine_trans;
		Eigen::Affine3d  transform_;  //the intersection matching result
									  //	Eigen::Affine3d  fine_transform_;    //the icp matching result

									  //	Eigen::Affine3d icp_transform_;  //fine registering result by ICP
	};

}

