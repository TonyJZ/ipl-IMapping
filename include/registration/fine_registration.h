#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>

#include <Eigen/Dense>
// This disables some useless Warnings on MSVC.
// It is intended to be done for this test only.
#include <Eigen/src/Core/util/DisableStupidWarnings.h>


#include "core/iplcore.h"
#include "commonAPIs/iplutility.h"
//#include "feature/PointGeoSegment.h"
#include "fitting/wallceiling_clipper.h"
#include "optimization/LevenbergMarquardt.h"



namespace ipl
{
	typedef struct
	{
		double a, b, c, d;

	} PlaneParams;

	//点面对应
	typedef struct
	{
		double x, y, z;
		double a, b, c, d;
		double w;  //由于要调用Eigen的解算 P = W^T*W, 
	}Point2PlaneCorrespondence;

	//点点对应
	typedef struct
	{
		double x, y, z;
		double xr, yr, zr;
		double w;  //由于要调用Eigen的解算 P = W^T*W, 
	}Point2PointCorrespondence;

	typedef struct
	{
		double corresponding_mse_threshold;//残差
		double cur_corresponding_mse;
		int max_iterations;//迭代次数
		int iterations;
		double relative_unknowns_mse_threshold;//未知数改正数
		double cur_relative_unknowns_mse;
	}ConvergenceCriteria;



	//the solver for Point-to-Plane 
	struct OptimizationFunctor_Point2Plane : LM_Functor<double>
	{
		std::vector<Point2PlaneCorrespondence> correspondences_;

		/** LM_Functor constructor
		* \param[in] m_data_points the number of data points to evaluate
		* \param[in,out] estimator pointer to the estimator object
		*/
		OptimizationFunctor_Point2Plane(int nUnknowns, std::vector<Point2PlaneCorrespondence> correspondences)
			: LM_Functor<double>(nUnknowns, static_cast<int>(correspondences.size()))
		{
			correspondences_ = correspondences;
		}

		/** Copy constructor
		* \param[in] src the optimization functor to copy into this
		*/
		
		/** \brief Destructor. */
//		virtual ~OptimizationFunctor() {}

		/** Fill fvec from x. For the current state vector x fill the f values
		* \param[in] x state vector
		* \param[out] fvec f values vector
		*/
		int operator () (const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const;

		int df(const Eigen::VectorXd &x, Eigen::MatrixXd &fjac) const;

// 		void setCorrespondences(std::vector<Point2PlaneCorrespondence> correspondence)
// 		{
// 			correspondences_ = correspondence;
// 		}
		//const TransformationEstimationLM<PointSource, PointTarget, MatScalar> *estimator_;
	};

	//the solver for Point-to-Point 
	struct OptimizationFunctor_Point2Point : LM_Functor<double>
	{
		std::vector<Point2PointCorrespondence> correspondences_;

		/** LM_Functor constructor
		* \param[in] m_data_points the number of data points to evaluate
		* \param[in,out] estimator pointer to the estimator object
		*/
		OptimizationFunctor_Point2Point(int nUnknowns, std::vector<Point2PointCorrespondence> correspondences)
			: LM_Functor<double>(nUnknowns, static_cast<int>(correspondences.size()))
		{
			correspondences_ = correspondences;
		}

		/** Copy constructor
		* \param[in] src the optimization functor to copy into this
		*/

		/** \brief Destructor. */
		//		virtual ~OptimizationFunctor() {}

		/** Fill fvec from x. For the current state vector x fill the f values
		* \param[in] x state vector
		* \param[out] fvec f values vector
		*/
		int operator () (const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const;

		int df(const Eigen::VectorXd &x, Eigen::MatrixXd &fjac) const;

		// 		void setCorrespondences(std::vector<Point2PlaneCorrespondence> correspondence)
		// 		{
		// 			correspondences_ = correspondence;
		// 		}
		//const TransformationEstimationLM<PointSource, PointTarget, MatScalar> *estimator_;
	};

	//indoor point cloud fine registering (by ICP)
	class IPL_BASE_API ipcFineRegistering
	{
		typedef struct
		{
			double min_pt[3];
			double max_pt[3];
		} BoundingBox;

		//用label来查找对应的平面参数
		typedef pcl::search::Search <pcl::PointXYZL> KdTree;
		typedef typename KdTree::Ptr KdTreePtr;

		typedef pcl::PointXYZRGBNormal PointT;
	public:
		
		ipcFineRegistering();
		~ipcFineRegistering();


		void set_refData(const std::vector<std::string> &pszCeilingFiles, const std::vector<std::string> &pszWallFiles);
		void set_tarData(const std::vector<std::string> &pszCeilingFiles, const std::vector<std::string> &pszWallFiles);

		void set_initTransformation(const std::string filename);
		void set_search_radius(double radius = 2.0);

		//设置墙面点和顶面点的采样率
		void set_sampling_rate(float w_rate, float c_rate = 0.2);

		void ceiling_switch(bool turn_on = true) { use_ceiling_ = turn_on; };
		void set_debug_path(std::string &path) 
		{ 
			debug_path_ = path;
			if (!check_exist(debug_path_))
				create_folder(debug_path_);
		};

		//register model: point-to-plane
		double do_ICP_PointToPlane(double corresponding_mse=0.001, double unknowns_mse = 1e-6, int maxIter = 10);

		//register model: point-to-point
		double do_ICP_PointToPoint(double corresponding_mse = 0.001, double unknowns_mse = 1e-6, int maxIter = 10);

		void get_init_transform(/*double origin[3], */Eigen::Affine3d &trans)
		{
		//	origin[0] = origin_[0]; origin[1] = origin_[1]; origin[2] = origin_[2];
			trans = init_transform_;
		};

		void get_final_transform(/*double origin[3], */Eigen::Affine3d &trans) 
		{
		//	origin[0] = origin_[0]; origin[1] = origin_[1]; origin[2] = origin_[2];
			trans = fine_transform_;
		};

		//提取配准误差
		double get_registration_rms() { return reg_rms_; };

//		Eigen::Affine3d get_final_transform();
		
//		double get_ICP_score() { return icp_fit_score_; };

	protected:
		void resample_target_points(geoModel::PointGroup<PointT> &tar_ceiling_group, geoModel::PointGroup<PointT> &tar_wall_group);

		void transform_pointcloud(const pcl::PointCloud<PointT> &org_cloud, pcl::PointCloud<PointT> &dst_cloud,
			const Eigen::Affine3d transfom);

		void transform_pointcloud(const pcl::PointCloud<PointT> &org_cloud, pcl::PointCloud<PointT> &dst_cloud,
			const double x[6]);

		void ICP_preparation();


	private:
		bool use_ceiling_;  //是否使用顶面数据
		std::string debug_path_;

		ref_ptr<WallCeilingClipper<PointT> > data_clipper_; //数据裁剪器

// 		std::vector<std::string> ref_ceilings_name_;
// 		std::vector<std::string> ref_walls_name_;
// 
// 		std::vector<std::string> tar_ceilings_name_;
// 		std::vector<std::string> tar_walls_name_;
// 
// 		std::vector<ipl::PointGeoSegment<PointT> > tar_ceiling_segments_;
// 		std::vector<ipl::PointGeoSegment<PointT> > tar_wall_segments_;

		//裁剪后得到的参考数据
// 		std::vector<ipl::PointGeoSegment<PointT> > ref_ceiling_segments_;
// 		std::vector<ipl::PointGeoSegment<PointT> > ref_wall_segments_;

// 		bool bRefClipped_;        //ref. data is clipped or not
// 		BoundingBox bbox_;        //bounding box of target walls
// 		BoundingBox clip_box_;    //clip box of ref. data (do init transformation)

		//resample points from target point cloud
		ref_ptr<iplPointCloud<PointT> >  resampled_wall_cloud_;
		ref_ptr<iplPointCloud<PointT> >  resampled_ceiling_cloud_;

		//采样点的权值
		std::vector<double>  resampled_wall_weights_;
		std::vector<double>  resampled_ceiling_weights_;

		//转换后的点云坐标
		ref_ptr<iplPointCloud<PointT> >  trans_wall_cloud_;  
		ref_ptr<iplPointCloud<PointT> >  trans_ceiling_cloud_;

		//NN搜索
		KdTreePtr wall_search_;  //for all wall ref points
		KdTreePtr ceiling_search_;  //for all ceiling ref points

		//参考点集
		ref_ptr<iplPointCloud<pcl::PointXYZL> >  ref_wall_cloud_;
		ref_ptr<iplPointCloud<pcl::PointXYZL> >  ref_ceiling_cloud_;

		//参考平面参数
		std::vector<PlaneParams>  ref_wall_params_;
		std::vector<PlaneParams>  ref_ceiling_params_;

		ConvergenceCriteria convergence_criteria_;

		//std::vector<Point2PlaneCorrespondence> correspondences_;

		//double origin_[3];		//the origin of the target dataset
		Eigen::Affine3d  init_transform_;  //the initial matching result
		Eigen::Affine3d  fine_transform_;    //the ICP matching result


//		float ceiling_lifted_;    //lift the ceiling for ICP

		double search_buf_size_;
		float wall_resampling_rate_, ceiling_resampling_rate;					//sampling rate for target points  which is ranged from 0 to 1
		
//		size_t min_sampling_num_;  //最小采样点数

		double reg_rms_; //配准中误差
	};

}
