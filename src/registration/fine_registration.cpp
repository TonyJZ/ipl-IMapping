#include "registration/fine_registration.h"
#include "core/ipldef.h"
#include "io/PointcloudIO.h"
#include "io/GeoTransFileIO.h"
#include "commonAPIs/iplstring.h"
#include "fitting/ModelParamsIO.h"
#include "geometry/RotateMat.h"

#include <boost/random.hpp>

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/common/transforms.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>

// #include <pcl/registration/icp.h>
// #include <pcl/registration/icp_nl.h>
// #include <pcl/registration/correspondence_rejection_one_to_one.h>
// #include <pcl/registration/correspondence_rejection_median_distance.h>
// #include <pcl/registration/correspondence_rejection_sample_consensus.h>
//#include <pcl/registration/correspondence_rejection_var_trimmed.h>
//#include <pcl/registration/correspondence_rejection_trimmed.h>
//#include <pcl/registration/correspondence_rejection_surface_normal.h>
//#include <pcl/registration/transformation_estimation_point_to_plane.h>


//调试开关，用于输出中间结果 
//#define __TEST   
//等权观测
//#define __EQUALWEIGHT

namespace ipl
{

	//////////////////////////////////////////////////////////////////////////////////////////////
	int OptimizationFunctor_Point2Plane::operator () (const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
	{
		double tx, ty, tz, roll, pitch, yaw;
		tx = x[0]; ty = x[1]; tz = x[2];
		roll = x[3]; pitch = x[4]; yaw = x[5];

// 		double cos_r, sin_r, cos_p, sin_p, cos_y, sin_y;
// 		cos_r = cos(roll);		sin_r = sin(roll);
// 		cos_p = cos(pitch);		sin_p = sin(pitch);
// 		cos_y = cos(yaw);		sin_y = sin(yaw);

		Eigen::Matrix3d rotMat;
		Eigen::Vector3d pos, trans_pos, T;
		RotateMat_XYZ(roll, pitch, yaw, rotMat);

		T(0) = tx; T(1) = ty; T(2) = tz;

		// Transform each source point and compute its distance to the corresponding target point
//		assert(inputs() == correspondences_.size());
		int n = values();

		for (int i = 0; i < correspondences_.size(); ++i)
		{
			double x, y, z;
			double a, b, c, d;
			double w;

			x = correspondences_[i].x;
			y = correspondences_[i].y;
			z = correspondences_[i].z;
			a = correspondences_[i].a;
			b = correspondences_[i].b;
			c = correspondences_[i].c;
			d = correspondences_[i].d;
			w = correspondences_[i].w;

			pos(0) = x; pos(1) = y; pos(2) = z;
			trans_pos = rotMat*pos + T;

			double a2b2c2 = a*a + b*b + c*c;

			double dis = 1.0 / a2b2c2*iplsqr(a*trans_pos(0) + b*trans_pos(1) + c*trans_pos(2) + d);

			fvec[i] = dis*w;   //LB = WL
		}
		return (0);
	}

	int OptimizationFunctor_Point2Plane::df(const Eigen::VectorXd &x, Eigen::MatrixXd &fjac) const
	{
		double tx, ty, tz, roll, pitch, yaw;
		tx = x[0]; ty = x[1]; tz = x[2];
		roll = x[3]; pitch = x[4]; yaw = x[5];

		double cos_r, sin_r, cos_p, sin_p, cos_y, sin_y;
		cos_r = cos(roll);		sin_r = sin(roll);
		cos_p = cos(pitch);		sin_p = sin(pitch);
		cos_y = cos(yaw);		sin_y = sin(yaw);
		
		//此处建立雅可比矩阵[一阶导数]
		for (int i = 0; i < correspondences_.size(); ++i)
		{
			double x, y, z;
			double a, b, c, d;
			double w;

			x = correspondences_[i].x;
			y = correspondences_[i].y;
			z = correspondences_[i].z;
			a = correspondences_[i].a;
			b = correspondences_[i].b;
			c = correspondences_[i].c;
			d = correspondences_[i].d;
			w = correspondences_[i].w;

			double a2b2c2 = a*a + b*b + c*c;
			double a2b2c2_inv = 1.0 / a2b2c2;

			//dtx
			fjac(i, 0) = a2b2c2_inv*((2 * ((z*(a*cos_y + sin_y*b)*cos_r + y*(a*cos_y + sin_y*b)*sin_r - c*x)*sin_p + (-a*sin_y*y + b*y*cos_y + cos_p*c*z)*cos_r + (a*sin_y*z - b*z*cos_y + cos_p*c*y)*sin_r + cos_y*cos_p*a*x + cos_p*sin_y*b*x + a*tx + c*tz + b*ty + d))*a);
		
			//dty
			fjac(i, 1) = a2b2c2_inv*((2 * ((z*(a*cos_y + sin_y*b)*cos_r + y*(a*cos_y + sin_y*b)*sin_r - c*x)*sin_p + (-a*sin_y*y + b*y*cos_y + cos_p*c*z)*cos_r + (a*sin_y*z - b*z*cos_y + cos_p*c*y)*sin_r + cos_y*cos_p*a*x + cos_p*sin_y*b*x + a*tx + c*tz + b*ty + d))*b);
			
			//dtz
			fjac(i, 2) = a2b2c2_inv*((2 * ((z*(a*cos_y + sin_y*b)*cos_r + y*(a*cos_y + sin_y*b)*sin_r - c*x)*sin_p + (-a*sin_y*y + b*y*cos_y + cos_p*c*z)*cos_r + (a*sin_y*z - b*z*cos_y + cos_p*c*y)*sin_r + cos_y*cos_p*a*x + cos_p*sin_y*b*x + a*tx + c*tz + b*ty + d))*c);
			
			//droll
			fjac(i, 3) = a2b2c2_inv*((2 * (a*(cos_y*cos_p*x + (-sin_y*cos_r + cos_y*sin_p*sin_r)*y + (sin_y*sin_r + cos_y*sin_p*cos_r)*z + tx) + b*(sin_y*cos_p*x + (cos_y*cos_r + sin_y*sin_p*sin_r)*y + (-cos_y*sin_r + sin_y*sin_p*cos_r)*z + ty) + c*(-sin_p*x + cos_p*sin_r*y + cos_p*cos_r*z + tz) + d))*(a*((sin_y*sin_r + cos_y*sin_p*cos_r)*y + (sin_y*cos_r - cos_y*sin_p*sin_r)*z) + b*((-cos_y*sin_r + sin_y*sin_p*cos_r)*y + (-cos_y*cos_r - sin_y*sin_p*sin_r)*z) + c*(cos_p*cos_r*y - cos_p*sin_r*z)));
			
			//dpitch
			fjac(i, 4) = a2b2c2_inv*((2 * (a*(cos_y*cos_p*x + (-sin_y*cos_r + cos_y*sin_p*sin_r)*y + (sin_y*sin_r + cos_y*sin_p*cos_r)*z + tx) + b*(sin_y*cos_p*x + (cos_y*cos_r + sin_y*sin_p*sin_r)*y + (-cos_y*sin_r + sin_y*sin_p*cos_r)*z + ty) + c*(-sin_p*x + cos_p*sin_r*y + cos_p*cos_r*z + tz) + d))*(a*(-cos_y*sin_p*x + cos_y*cos_p*sin_r*y + cos_y*cos_p*cos_r*z) + b*(-sin_y*sin_p*x + sin_y*cos_p*sin_r*y + sin_y*cos_p*cos_r*z) + c*(-cos_p*x - sin_p*sin_r*y - sin_p*cos_r*z)));
			
			//dyaw
			fjac(i, 5) = a2b2c2_inv*((2 * (a*(cos_y*cos_p*x + (-sin_y*cos_r + cos_y*sin_p*sin_r)*y + (sin_y*sin_r + cos_y*sin_p*cos_r)*z + tx) + b*(sin_y*cos_p*x + (cos_y*cos_r + sin_y*sin_p*sin_r)*y + (-cos_y*sin_r + sin_y*sin_p*cos_r)*z + ty) + c*(-sin_p*x + cos_p*sin_r*y + cos_p*cos_r*z + tz) + d))*(a*(-sin_y*cos_p*x + (-cos_y*cos_r - sin_y*sin_p*sin_r)*y + (cos_y*sin_r - sin_y*sin_p*cos_r)*z) + b*(cos_y*cos_p*x + (-sin_y*cos_r + cos_y*sin_p*sin_r)*y + (sin_y*sin_r + cos_y*sin_p*cos_r)*z)));

			for (int j = 0; j < 6; ++j)
			{
				fjac(i, j) *= w;  //B = WA
			}
		}

		return 0;
	}

	//////////////////////////////////////////////////////////////////////////////////////////////
	int OptimizationFunctor_Point2Point::operator () (const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
	{
		double tx, ty, tz, roll, pitch, yaw;
		tx = x[0]; ty = x[1]; tz = x[2];
		roll = x[3]; pitch = x[4]; yaw = x[5];

// 		double cos_r, sin_r, cos_p, sin_p, cos_y, sin_y;
// 		cos_r = cos(roll);		sin_r = sin(roll);
// 		cos_p = cos(pitch);		sin_p = sin(pitch);
// 		cos_y = cos(yaw);		sin_y = sin(yaw);

		Eigen::Matrix3d rotMat;
		Eigen::Vector3d pos, trans_pos, T;
		RotateMat_XYZ(roll, pitch, yaw, rotMat);
		
		T(0) = tx; T(1) = ty; T(2) = tz;

		// Transform each source point and compute its distance to the corresponding target point
//		assert(inputs() == correspondences_.size());
		for (int i = 0; i < correspondences_.size(); ++i)
		{
			double x, y, z;
			double xr, yr, zr;
			//double a, b, c, d;
			double w;
			double Xt, Yt, Zt;

			x = correspondences_[i].x;
			y = correspondences_[i].y;
			z = correspondences_[i].z;
			w = correspondences_[i].w;
			xr = correspondences_[i].xr;
			yr = correspondences_[i].yr;
			zr = correspondences_[i].zr;

// 			Xt = cos_p*cos_y*x + (sin_r*sin_p*cos_y - cos_r*sin_y)*y + (cos_r*sin_p*cos_y + sin_r*sin_y)*z + tx;
// 			Yt = cos_p*sin_y*x + (sin_r*sin_p*sin_y + cos_r*cos_y)*y + (cos_r*sin_p*sin_y - sin_r*cos_y)*z + ty;
// 			Zt = -sin_p*x + sin_r*cos_p*y + cos_r*cos_p*z + tz;
// 
// 			//LB = WL
// 			fvec[i * 3] = (Xt - xr)*w;
// 			fvec[i * 3 + 1] = (Yt - yr)*w;
// 			fvec[i * 3 + 2] = (Zt - zr)*w;

			//LB = WL
			
			pos(0) = x; pos(1) = y; pos(2) = z;
			trans_pos = rotMat*pos + T;

// 			double temp1 = sqr(trans_pos(0) - xr);
// 			double temp2 = sqr(trans_pos(1) - yr);
// 			double temp3 = sqr(trans_pos(2) - zr);
// 
// 			double temp = sqr(trans_pos(0) - xr) + sqr(trans_pos(1) - yr) + sqr(trans_pos(2) - zr);

			fvec(i) = iplsqr(trans_pos(0) - xr) + iplsqr(trans_pos(1) - yr) + iplsqr(trans_pos(2) - zr);
			fvec(i) *= w;
		}
		return (0);
	}

	int OptimizationFunctor_Point2Point::df(const Eigen::VectorXd &x, Eigen::MatrixXd &fjac) const
	{
		double tx, ty, tz, roll, pitch, yaw;
		tx = x[0]; ty = x[1]; tz = x[2];
		roll = x[3]; pitch = x[4]; yaw = x[5];

		double cos_r, sin_r, cos_p, sin_p, cos_y, sin_y;
		cos_r = cos(roll);		sin_r = sin(roll);
		cos_p = cos(pitch);		sin_p = sin(pitch);
		cos_y = cos(yaw);		sin_y = sin(yaw);

		//此处建立雅可比矩阵[一阶导数]
		for (int i = 0; i < correspondences_.size(); ++i)
		{
			double x, y, z;
			double xr, yr, zr;
			//double a, b, c, d;
			double w;

			x = correspondences_[i].x;
			y = correspondences_[i].y;
			z = correspondences_[i].z;
			w = correspondences_[i].w;
			xr = correspondences_[i].xr;
			yr = correspondences_[i].yr;
			zr = correspondences_[i].zr;

			//dXt
			fjac(i, 0) = 2 * cos_p*cos_y*x + (2 * (sin_r*sin_p*cos_y - cos_r*sin_y))*y + 
				(2 * (cos_r*sin_p*cos_y + sin_r*sin_y))*z + 2 * tx - 2 * xr;
			fjac(i, 1) = 2 * cos_p*sin_y*x + (2 * (sin_r*sin_p*sin_y + cos_r*cos_y))*y + 
				(2 * (cos_r*sin_p*sin_y - sin_r*cos_y))*z + 2 * ty - 2 * yr;
			fjac(i, 2) = -2 * sin_p*x + 2 * sin_r*cos_p*y + 2 * cos_r*cos_p*z + 2 * tz - 2 * zr;
			fjac(i, 3) = (2 * (cos_p*cos_y*x + (sin_r*sin_p*cos_y - cos_r*sin_y)*y + 
				(cos_r*sin_p*cos_y + sin_r*sin_y)*z + tx - xr))*
				((cos_r*sin_p*cos_y + sin_r*sin_y)*y + (-sin_r*sin_p*cos_y + cos_r*sin_y)*z) + 
				(2 * (cos_p*sin_y*x + (sin_r*sin_p*sin_y + cos_r*cos_y)*y + 
				(cos_r*sin_p*sin_y - sin_r*cos_y)*z + ty - yr))*
					((cos_r*sin_p*sin_y - sin_r*cos_y)*y + (-sin_r*sin_p*sin_y - cos_r*cos_y)*z) + 
				(2 * (-sin_p*x + sin_r*cos_p*y + cos_r*cos_p*z + tz - zr))*(cos_r*cos_p*y - sin_r*cos_p*z);
			fjac(i, 4) = (2 * (cos_p*cos_y*x + (sin_r*sin_p*cos_y - cos_r*sin_y)*y + 
				(cos_r*sin_p*cos_y + sin_r*sin_y)*z + tx - xr))*
				(-sin_p*cos_y*x + sin_r*cos_p*cos_y*y + cos_r*cos_p*cos_y*z) + 
				(2 * (cos_p*sin_y*x + (sin_r*sin_p*sin_y + cos_r*cos_y)*y + 
				(cos_r*sin_p*sin_y - sin_r*cos_y)*z + ty - yr))*(-sin_p*sin_y*x + 
					sin_r*cos_p*sin_y*y + cos_r*cos_p*sin_y*z) + 
					(2 * (-sin_p*x + sin_r*cos_p*y + cos_r*cos_p*z + tz - zr))*
				(-cos_p*x - sin_r*sin_p*y - cos_r*sin_p*z);
			fjac(i, 5) = (2 * (cos_p*cos_y*x + (sin_r*sin_p*cos_y - cos_r*sin_y)*y + 
				(cos_r*sin_p*cos_y + sin_r*sin_y)*z + tx - xr))*(-cos_p*sin_y*x + 
				(-sin_r*sin_p*sin_y - cos_r*cos_y)*y + (-cos_r*sin_p*sin_y + sin_r*cos_y)*z) + 
					(2 * (cos_p*sin_y*x + (sin_r*sin_p*sin_y + cos_r*cos_y)*y + 
				(cos_r*sin_p*sin_y - sin_r*cos_y)*z + ty - yr))*(cos_p*cos_y*x + 
						(sin_r*sin_p*cos_y - cos_r*sin_y)*y + (cos_r*sin_p*cos_y + sin_r*sin_y)*z);
			//B = WA
			fjac(i, 0) *= w;
			fjac(i, 1) *= w;
			fjac(i, 2) *= w;
			fjac(i, 3) *= w;
			fjac(i, 4) *= w;
			fjac(i, 5) *= w;

// 			//dYt
// 			fjac(i * 3 + 1, 0) = 0.0;
// 			fjac(i * 3 + 1, 1) = 1.0;
// 			fjac(i * 3 + 1, 2) = 0.0;
// 			fjac(i * 3 + 1, 3) = (cos_r*sin_p*sin_y - sin_r*cos_y)*y + (-sin_r*sin_p*sin_y - cos_r*cos_y)*z;
// 			fjac(i * 3 + 1, 4) = -sin_p*sin_y*x + sin_r*cos_p*sin_y*y + cos_r*cos_p*sin_y*z;
// 			fjac(i * 3 + 1, 5) = cos_p*cos_y*x + (sin_r*sin_p*cos_y - cos_r*sin_y)*y + (cos_r*sin_p*cos_y + sin_r*sin_y)*z;
// 			//B = WA
// 			fjac(i * 3 + 1, 0) *= w;
// 			fjac(i * 3 + 1, 1) *= w;
// 			fjac(i * 3 + 1, 2) *= w;
// 			fjac(i * 3 + 1, 3) *= w;
// 			fjac(i * 3 + 1, 4) *= w;
// 			fjac(i * 3 + 1, 5) *= w;
// 
// 			//dZt
// 			fjac(i * 3 + 2, 0) = 0.0;
// 			fjac(i * 3 + 2, 1) = 0.0;
// 			fjac(i * 3 + 2, 2) = 1.0;
// 			fjac(i * 3 + 2, 3) = cos_r*cos_p*y - sin_r*cos_p*z;
// 			fjac(i * 3 + 2, 4) = -cos_p*x - sin_r*sin_p*y - cos_r*sin_p*z;
// 			fjac(i * 3 + 2, 5) = 0.0;
// 			//B = WA
// 			fjac(i * 3 + 2, 0) *= w;
// 			fjac(i * 3 + 2, 1) *= w;
// 			fjac(i * 3 + 2, 2) *= w;
// 			fjac(i * 3 + 2, 3) *= w;
// 			fjac(i * 3 + 2, 4) *= w;
// 			fjac(i * 3 + 2, 5) *= w;

		}

		return 0;
	}

	ipcFineRegistering::ipcFineRegistering()
	{
		init_transform_ = Eigen::Matrix4d::Identity();
		fine_transform_ = Eigen::Matrix4d::Identity();

		search_buf_size_ = 2.0;
		wall_resampling_rate_ = 1.0;
		ceiling_resampling_rate = 0.2;  //默认为0.2*wall_resampling_rate_
		reg_rms_ = std::numeric_limits<double>::max();

// 		bRefClipped_ = false;
// 		bbox_.min_pt[0] = bbox_.min_pt[1] = bbox_.min_pt[2] = std::numeric_limits<double>::max();
// 		bbox_.max_pt[0] = bbox_.max_pt[1] = bbox_.max_pt[2] = std::numeric_limits<double>::lowest();
		
		data_clipper_.reset(new WallCeilingClipper<PointT>);

		use_ceiling_ = true;
	}

	ipcFineRegistering::~ipcFineRegistering()
	{
		// 	if (clipped_ref_ceiling_search_ != 0)
		// 		clipped_ref_ceiling_search_.reset();
		// 
		// 	if (clipped_ref_wall_search_ != 0)
		// 		clipped_ref_wall_search_.reset();

	}

	void ipcFineRegistering::set_refData(const std::vector<std::string> &pszCeilingFiles, 
		const std::vector<std::string> &pszWallFiles)
	{
		data_clipper_->set_refData(pszCeilingFiles, pszWallFiles);
	}

	void ipcFineRegistering::set_tarData(const std::vector<std::string> &pszCeilingFiles, 
		const std::vector<std::string> &pszWallFiles)
	{
		data_clipper_->set_tarData(pszCeilingFiles, pszWallFiles);
	}

	void ipcFineRegistering::set_initTransformation(const std::string filename)
	{
		Eigen::Matrix4d trans_mat;
		//double rms;
		bool bRet = load_RigidTransParam(filename, /*origin_, */trans_mat, reg_rms_);

		if (!bRet)
			std::cout << "error! can't read file: " << filename << std::endl;

		init_transform_ = trans_mat;

		data_clipper_->set_Transformation(filename);
	}

	void ipcFineRegistering::set_search_radius(double radius)
	{
		search_buf_size_ = radius;
		data_clipper_->set_search_radius(radius);
	}

	void ipcFineRegistering::set_sampling_rate(float w_rate, float c_rate)
	{
		if (w_rate > 0 && w_rate <= 1.0)
			wall_resampling_rate_ = w_rate;

		if (c_rate > 0 && c_rate <= 1.0)
			ceiling_resampling_rate = c_rate;
	}

	void ipcFineRegistering::transform_pointcloud(const pcl::PointCloud<PointT> &org_cloud, pcl::PointCloud<PointT> &dst_cloud,
		const Eigen::Affine3d transfom)
	{
		pcl::transformPointCloud(org_cloud, dst_cloud, transfom);
	}

	void ipcFineRegistering::transform_pointcloud(const pcl::PointCloud<PointT> &org_cloud, pcl::PointCloud<PointT> &dst_cloud,
		const double x[6])
	{
		double tx, ty, tz, roll, pitch, yaw;
		tx = x[0]; ty = x[1]; tz = x[2];
		roll = x[3]; pitch = x[4]; yaw = x[5];

		Eigen::Matrix3d rotMat;

		RotateMat_XYZ(roll, pitch, yaw, rotMat);
		
		size_t npts = org_cloud.size();
		dst_cloud.points.resize(npts);
		dst_cloud.height = org_cloud.height;
		dst_cloud.width = org_cloud.width;
		for (size_t i = 0; i < npts; ++i)
		{
			double x, y, z;
			x = org_cloud.points[i].x;
			y = org_cloud.points[i].y;
			z = org_cloud.points[i].z;

			dst_cloud.points[i] = org_cloud.points[i];
			dst_cloud.points[i].x = rotMat(0, 0)*x + rotMat(0, 1)*y + rotMat(0, 2)*z + tx;
			dst_cloud.points[i].y = rotMat(1, 0)*x + rotMat(1, 1)*y + rotMat(1, 2)*z + ty;
			dst_cloud.points[i].z = rotMat(2, 0)*x + rotMat(2, 1)*y + rotMat(2, 2)*z + tz;
		}

	}

	void ipcFineRegistering::resample_target_points(geoModel::PointGroup<PointT> &tar_ceiling_group, geoModel::PointGroup<PointT> &tar_wall_group)
	{
		resampled_wall_cloud_.reset(new iplPointCloud<PointT>);
		resampled_ceiling_cloud_.reset(new iplPointCloud<PointT>);

		size_t nWalls = tar_wall_group.modelParams.size();

		//int n_resampling_wall = ceil(n_wall * sampling_rate_);

		//analyse the distribution of normals 
		
		///////////////////////////////////////////////////////////////////////
		////////1. 方向直方图统计   ///////////////////////////////////////////
		Eigen::Vector2d  axisX = Eigen::Vector2d(1.0, 0.0);
		double th_pi_4 = cosf(M_PI / 4);

		std::vector<size_t> dirHists; //方向直方图
		dirHists.resize(4, 0);

		std::vector<int> plane2QuadLut;  //平面与象限对应
		plane2QuadLut.resize(nWalls);
		
		std::vector<iplPointCloud<PointT> > quadCloudset; //方向点集
		quadCloudset.resize(4);

		std::vector<std::vector<double> > quadCloudWeights;//各象限点云的权值
		quadCloudWeights.resize(4);

		std::vector<double> planeW; //平面内的所有点等权
		planeW.resize(nWalls);

		//计算各平面所属象限
		for (size_t i = 0; i < nWalls; ++i)
		{
			ipl::geoModel::geoModelInfo mCoef;
			mCoef = tar_wall_group.modelParams[i];
			double pw = 1.0 / mCoef.obsSD;  //point weight
			planeW[i] = pw;

			Eigen::Vector2d proj_line_normal;
			proj_line_normal[0] = -mCoef.coef.values[1];
			proj_line_normal[1] = mCoef.coef.values[0];

			double cos_theta = proj_line_normal.dot(axisX);

			//(0, PI/4), (PI/4, PI/2), (PI/2, 3*PI/4), (3*PI/4, PI)
			int iQuad = 0;
			if (cos_theta > th_pi_4)
			{
				iQuad = 0;
			}
			else if (cos_theta > 0)
			{
				iQuad = 1;
			}
			else if (cos_theta > -th_pi_4)
			{
				iQuad = 2;
			}
			else if (cos_theta > -1)
			{
				iQuad = 3;
			}

			plane2QuadLut[i] = iQuad;			
		}

		//统计各象限点数
		for (size_t i = 0; i < tar_wall_group.cloud->size(); ++i)
		{
			int iplane = tar_wall_group.lut[i];
			int iQuad = plane2QuadLut[iplane];
			
			dirHists[iQuad]++;
			quadCloudset[iQuad].points.push_back(tar_wall_group.cloud->points[i]);
			quadCloudWeights[iQuad].push_back(planeW[iplane]);
		}
		

		///////////////////////////////////////////////////////////
		//////////////2. 均匀采样  ////////////////////////////////
		//均匀采样保证在各段采样率一致，拼接不同平面后，各平面采样率也是均匀的
		//////////////////////////////////////////////////////////
		size_t total_pts = 0;
		size_t expected_samplings[4];     //每象限期望的采样点数为1/4*total_pts*resampling_rate
		for (size_t i = 0; i < 4; ++i)
		{
			total_pts += dirHists[i];
		}

		expected_samplings[0] = expected_samplings[1]
			= expected_samplings[2] = expected_samplings[3] = floor(total_pts*0.25*wall_resampling_rate_);

		//根据分布调整实际采样点数
		for (size_t i = 0; i < 4; ++i)
		{
			if (expected_samplings[i] > dirHists[i])
				expected_samplings[i] = dirHists[i];
		}

		std::cout << "wall resampling: " << "\n"
			<< "Quadrant 1: " << expected_samplings[0] << "/" << dirHists[0] << "\n"
			<< "Quadrant 2: " << expected_samplings[1] << "/" << dirHists[1] << "\n"
			<< "Quadrant 3: " << expected_samplings[2] << "/" << dirHists[2] << "\n"
			<< "Quadrant 4: " << expected_samplings[3] << "/" << dirHists[3]
			<< std::endl;

		resampled_wall_weights_.clear();
		for (int iQuad = 0; iQuad < 4; ++iQuad)
		{
			if (quadCloudset[iQuad].size() < 10)
				continue;
				
			//uniform_int  
			boost::mt19937 rng(time(0));
			boost::uniform_int<> ui(0, quadCloudset[iQuad].size() - 1);
			std::vector<bool> flag;  //避免采样点重复，保证后面计算时为行满秩矩阵。
			flag.resize(quadCloudset[iQuad].size(), false);

			for (size_t i = 0; i < expected_samplings[iQuad]; ++i)
			{
				int id = ui(rng);
				while (flag[id])
				{
					id = ui(rng);
				}

				resampled_wall_cloud_->points.push_back(quadCloudset[iQuad].points[id]);
				resampled_wall_weights_.push_back(quadCloudWeights[iQuad].at(id));
				flag[id] = true;
			}
		}
		resampled_wall_cloud_->height = 1;
		resampled_wall_cloud_->width = resampled_wall_cloud_->size();

		size_t nSample_ceiling = floor(resampled_wall_cloud_->size() * ceiling_resampling_rate);

		size_t nCeilings = tar_ceiling_group.modelParams.size();
		planeW.resize(nCeilings);
		std::vector<double> merged_ceiling_weights;
		for (size_t i = 0; i < tar_ceiling_group.modelParams.size(); ++i)
		{
			ipl::geoModel::geoModelInfo mCoef;
			mCoef = tar_ceiling_group.modelParams[i];
			double pw = 1.0 / mCoef.obsSD;  //point weight
			planeW[i] = pw;
		}
		
		std::cout << "ceiling resampling: "
			<< nSample_ceiling << "/" << tar_ceiling_group.cloud->size()
			<< std::endl;

		boost::mt19937 rng(time(0));
		boost::uniform_int<> ui(0, tar_ceiling_group.cloud->size() - 1);
		std::vector<bool> flag;  //避免采样点重复，保证后面计算时为行满秩矩阵。
		flag.resize(tar_ceiling_group.cloud->size(), false);
		for (size_t i = 0; i < nSample_ceiling; ++i)
		{
			int id = ui(rng);
			while (flag[id])
			{
				id = ui(rng);
			}

			resampled_ceiling_cloud_->points.push_back(tar_ceiling_group.cloud->points[id]);
			int iplane = tar_ceiling_group.lut[i];
			double pw = planeW[iplane];

			resampled_ceiling_weights_.push_back(pw);
			flag[id] = true;
		}

		resampled_ceiling_cloud_->height = 1;
		resampled_ceiling_cloud_->width = resampled_ceiling_cloud_->size();

//		write_PointCloud("D:/iplTestData/TargetLocalization/target_dataset1/result/eRansac/resamp_wall.pcd", *resampled_wall_cloud_, true);
//		write_PointCloud("D:/iplTestData/TargetLocalization/target_dataset1/result/eRansac/resamp_ceiling.pcd", *resampled_ceiling_cloud_, true);
		return;
	}

	void ipcFineRegistering::ICP_preparation()
	{
		//1. clip reference data and create kd tree
		data_clipper_->do_clip();
		geoModel::PointGroup<PointT> ref_ceiling_group, ref_wall_group;

		data_clipper_->getClippedCeilings(ref_ceiling_group);
		data_clipper_->getClippedWalls(ref_wall_group);

// 		ipl::write_PointCloud("D:/iplTestData/TargetLocalization/target_dataset1/result/clipped_ceiling.pcd",
// 			ref_ceiling_group.cloud, true);
// 		ipl::write_PointCloud("D:/iplTestData/TargetLocalization/target_dataset1/result/clipped_wall.pcd",
// 			ref_wall_group.cloud, true);

		//////////////////////////////////////
		//////  create kdtrees /////////////
		wall_search_.reset(new pcl::search::KdTree<pcl::PointXYZL>);
		ceiling_search_.reset(new pcl::search::KdTree<pcl::PointXYZL>);

		ref_wall_cloud_.reset(new iplPointCloud<pcl::PointXYZL>);
		ref_ceiling_cloud_.reset(new iplPointCloud<pcl::PointXYZL>);

		ref_wall_params_.resize(ref_wall_group.modelParams.size());
		for (size_t i = 0; i < ref_wall_group.modelParams.size(); ++i)
		{
			ipl::geoModel::geoModelInfo mCoef;
			//mCoef = ref_wall_segments_[i].getModelCoef();
			mCoef = ref_wall_group.modelParams[i];
			ref_wall_params_[i].a = mCoef.coef.values[0];
			ref_wall_params_[i].b = mCoef.coef.values[1];
			ref_wall_params_[i].c = mCoef.coef.values[2];
			ref_wall_params_[i].d = mCoef.coef.values[3];
		}

		for (size_t j = 0; j < ref_wall_group.cloud->size(); ++j)
		{
			pcl::PointXYZL pt;
			pt.x = ref_wall_group.cloud->points[j].x;
			pt.y = ref_wall_group.cloud->points[j].y;
			pt.z = ref_wall_group.cloud->points[j].z;
			pt.label = ref_wall_group.lut[j];
			ref_wall_cloud_->push_back(pt);
		}

		ref_ceiling_params_.resize(ref_ceiling_group.modelParams.size());
		for (size_t i = 0; i < ref_ceiling_group.modelParams.size(); ++i)
		{
			ipl::geoModel::geoModelInfo mCoef;
			mCoef = ref_ceiling_group.modelParams[i];
			ref_ceiling_params_[i].a = mCoef.coef.values[0];
			ref_ceiling_params_[i].b = mCoef.coef.values[1];
			ref_ceiling_params_[i].c = mCoef.coef.values[2];
			ref_ceiling_params_[i].d = mCoef.coef.values[3];
		}

		for (size_t j = 0; j < ref_ceiling_group.cloud->size(); ++j)
		{
			pcl::PointXYZL pt;
			pt.x = ref_ceiling_group.cloud->points[j].x;
			pt.y = ref_ceiling_group.cloud->points[j].y;
			pt.z = ref_ceiling_group.cloud->points[j].z;
			pt.label = ref_ceiling_group.lut[j];
			ref_ceiling_cloud_->push_back(pt);
		}

#ifdef __TEST
		// 		ipl::write_PointCloud("D:/iplTestData/TargetLocalization/target_dataset1/result/clipped_ceiling.pcd",
		// 			*ref_ceiling_cloud_);
		// 		ipl::write_PointCloud("D:/iplTestData/TargetLocalization/target_dataset1/result/clipped_wall.pcd",
		// 			*ref_wall_cloud_);
#endif // __TEST

		wall_search_->setInputCloud(ref_wall_cloud_);
		ceiling_search_->setInputCloud(ref_ceiling_cloud_);


		//2. resample target data
		geoModel::PointGroup<PointT> tar_ceiling_group, tar_wall_group;
		data_clipper_->getTargetCeilings(tar_ceiling_group);
		data_clipper_->getTargetWalls(tar_wall_group);

		resample_target_points(tar_ceiling_group, tar_wall_group);

		return;
	}

	double ipcFineRegistering::do_ICP_PointToPlane(double corresponding_mse, double unknowns_mse, int maxIter)
	{
		ICP_preparation();

		//////////////////////////////////////////////////////////////////////////////////////////////////
		////////  2. find NN corresponding (ICP) + estimate transformation in an iterated way ////////////
		
		fine_transform_ = init_transform_;
		convergence_criteria_.max_iterations = maxIter;
		convergence_criteria_.corresponding_mse_threshold = corresponding_mse;
		convergence_criteria_.relative_unknowns_mse_threshold = unknowns_mse;

		convergence_criteria_.iterations = 0;
		convergence_criteria_.cur_corresponding_mse = std::numeric_limits<double>::max();
		convergence_criteria_.cur_relative_unknowns_mse = std::numeric_limits<double>::max();

		int min_number_correspondences = 6;  //最小对应点数 (观测值不可小于未知数个数)
		double corr_dist_threshold = search_buf_size_ * 0.6;
		corr_dist_threshold *= corr_dist_threshold;

		trans_wall_cloud_.reset(new iplPointCloud<PointT>);
		trans_ceiling_cloud_.reset(new iplPointCloud<PointT>);
		std::cout << "iterated 3D regisatration..." << std::endl;
		do 
		{
			transform_pointcloud(*resampled_wall_cloud_, *trans_wall_cloud_, fine_transform_);
			
			if(use_ceiling_)
				transform_pointcloud(*resampled_ceiling_cloud_, *trans_ceiling_cloud_, fine_transform_);

#ifdef __TEST
//			std::string path = "D:/iplTestData/TargetLocalization/target_dataset1/result/";
			std::string filename;
			char buf[64];
			sprintf(buf, "resampled_wall_%02d.pcd", convergence_criteria_.iterations);
			filename = debug_path_ + "/" + buf;
			ipl::write_PointCloud(filename, *trans_wall_cloud_);

			if (use_ceiling_)
			{
				sprintf(buf, "resampled_ceiling_%02d.pcd", convergence_criteria_.iterations);
				filename = debug_path_ + "/" + buf;
				ipl::write_PointCloud(filename, *trans_ceiling_cloud_);
			}
#endif

#ifdef __TEST
			ref_ptr<iplPointCloud<pcl::PointXYZL> > cor_wall_cloud(new iplPointCloud<pcl::PointXYZL>);
			ref_ptr<iplPointCloud<pcl::PointXYZL> > cor_ceiling_cloud(new iplPointCloud<pcl::PointXYZL>);
#endif

			std::vector<double> residuals;
			double sumWTW = 0;
			std::vector<Point2PlaneCorrespondence> correspondences;
			size_t nwall_cor = 0;
			//提取墙面对应
			for (size_t i = 0; i < trans_wall_cloud_->size(); ++i)
			{
				pcl::PointXYZL searchPoint;
				searchPoint.x = trans_wall_cloud_->points[i].x;
				searchPoint.y = trans_wall_cloud_->points[i].y;
				searchPoint.z = trans_wall_cloud_->points[i].z;
				
				std::vector<int> k_indices;
				std::vector<float> k_sqr_distances;
				wall_search_->nearestKSearch(searchPoint, 1, k_indices, k_sqr_distances);

				if (k_sqr_distances[0] < corr_dist_threshold)
				{
					Point2PlaneCorrespondence  corr;
					corr.x = searchPoint.x;
					corr.y = searchPoint.y;
					corr.z = searchPoint.z;
					corr.w = resampled_wall_weights_[i];

					sumWTW += corr.w*corr.w;

					int ref_id = k_indices[0];
					uint32_t planeId = ref_wall_cloud_->points[ref_id].label;

					corr.a = ref_wall_params_[planeId].a;
					corr.b = ref_wall_params_[planeId].b;
					corr.c = ref_wall_params_[planeId].c;
					corr.d = ref_wall_params_[planeId].d;

					correspondences.push_back(corr);
					residuals.push_back(k_sqr_distances[0]);
					nwall_cor++;

#ifdef __TEST
					cor_wall_cloud->points.push_back(ref_wall_cloud_->points[ref_id]);
#endif
				}			
			}

			size_t nceiling_cor = 0;
			if(use_ceiling_){
			
			//提取顶面对应
			for (size_t i = 0; i < trans_ceiling_cloud_->size(); ++i)
			{
				pcl::PointXYZL searchPoint;
				searchPoint.x = trans_ceiling_cloud_->points[i].x;
				searchPoint.y = trans_ceiling_cloud_->points[i].y;
				searchPoint.z = trans_ceiling_cloud_->points[i].z;

				std::vector<int> k_indices;
				std::vector<float> k_sqr_distances;
				ceiling_search_->nearestKSearch(searchPoint, 1, k_indices, k_sqr_distances);

				if (k_sqr_distances[0] < corr_dist_threshold)
				{
					Point2PlaneCorrespondence  corr;
					corr.x = searchPoint.x;
					corr.y = searchPoint.y;
					corr.z = searchPoint.z;
					corr.w = resampled_ceiling_weights_[i];

					sumWTW += corr.w*corr.w;

					int ref_id = k_indices[0];
					uint32_t planeId = ref_ceiling_cloud_->points[ref_id].label;

					corr.a = ref_ceiling_params_[planeId].a;
					corr.b = ref_ceiling_params_[planeId].b;
					corr.c = ref_ceiling_params_[planeId].c;
					corr.d = ref_ceiling_params_[planeId].d;

					correspondences.push_back(corr);
					residuals.push_back(k_sqr_distances[0]);
					nceiling_cor++;

#ifdef __TEST
					cor_ceiling_cloud->points.push_back(ref_ceiling_cloud_->points[ref_id]);
#endif
				}
			}

			}

#ifdef __TEST
			cor_wall_cloud->height = 1;
			cor_wall_cloud->width = cor_wall_cloud->size();

// 			std::string path1 = "D:/iplTestData/TargetLocalization/target_dataset1/result/";
// 			std::string filename;
// 			char buf1[32];
			sprintf(buf, "correspond_wall_cloud_%02d.pcd", convergence_criteria_.iterations);
			filename = debug_path_ + "/" + buf;
			ipl::write_PointCloud(filename, *cor_wall_cloud);

			if(use_ceiling_){
			cor_ceiling_cloud->height = 1;
			cor_ceiling_cloud->width = cor_ceiling_cloud->size();
			sprintf(buf, "correspond_ceiling_cloud_%02d.pcd", convergence_criteria_.iterations);
			filename = debug_path_ + "/" + buf;
			ipl::write_PointCloud(filename, *cor_ceiling_cloud);
			}
#endif
			
			double nor_factor = 1.0 / sqrt(sumWTW);  //权归一化因子  P = WT*W
			
			size_t nCor = correspondences.size();
			if (nCor < min_number_correspondences)
			{
				std::cout << "the correspondence is less than the minimal requirements!" << std::endl;
				break;
			}

			std::cout << "find corresponding pairs: " << nCor << ";";
			std::cout << " wall: " << nwall_cor << " (" << trans_wall_cloud_->size() << ")" << ";";

			if (use_ceiling_)
				std::cout << " ceiling: " << nceiling_cor << " (" << trans_ceiling_cloud_->size() << ")";
			std::cout << std::endl;

			//权归一化
			for (size_t i = 0; i < nCor; i++)
			{
#ifdef __EQUALWEIGHT
				correspondences[i].w = 1.0;
#else
				correspondences[i].w *= nor_factor;
#endif
// 				if (!use_ceiling_)
// 				{
// 					correspondences[i].z = correspondences[i].zr = 0;
// 				}
			}

			if (convergence_criteria_.iterations == 0)
			{
				double initRms = 0;
				for (size_t i = 0; i < residuals.size(); i++)
				{
					initRms += residuals[i] * residuals[i] * correspondences[i].w*correspondences[i].w;
				}

#ifdef __EQUALWEIGHT
				initRms = sqrt(initRms / residuals.size());
#else
				initRms = sqrt(initRms);
#endif
				std::cout << "initial rms: " << initRms << std::endl;
			}

			//参数估计采用Eigen::LevenbergMarquardt
			OptimizationFunctor_Point2Plane functor(6, correspondences);
			//Eigen::NumericalDiff<OptimizationFunctor<double> > num_diff(functor);

			Eigen::LevenbergMarquardt<OptimizationFunctor_Point2Plane> lm(functor);

			Eigen::VectorXd x(6);
			x.setZero();

			lm.parameters.xtol = 1e-8;
			lm.parameters.ftol = 1e-8;

			int info = lm.minimize(x);

			// Copy the rotation and translation components
			Eigen::Matrix4d transform_matrix;
			transform_matrix.setZero();
			transform_matrix(0, 3) = x[0];
			transform_matrix(1, 3) = x[1];
			transform_matrix(2, 3) = x[2];
			transform_matrix(3, 3) = 1;

			// Compute w from the unit quaternion
// 			Eigen::Quaternion<double> q(0, x[3], x[4], x[5]);
// 			q.w() = static_cast<double> (sqrt(1 - q.dot(q)));
// 			q.normalize();
			Eigen::Matrix3d rotMat;
			rotMat = Eigen::AngleAxisd(x[5], Eigen::Vector3d::UnitZ())
				* Eigen::AngleAxisd(x[4], Eigen::Vector3d::UnitY())
				* Eigen::AngleAxisd(x[3], Eigen::Vector3d::UnitX());
			transform_matrix.topLeftCorner(3, 3) = rotMat;

			fine_transform_.matrix() = transform_matrix * fine_transform_.matrix();

#ifdef __TEST
			ref_ptr<iplPointCloud<PointT> > dst_cloud(new iplPointCloud<PointT>);
			transform_pointcloud(*trans_wall_cloud_, *dst_cloud, &x[0]);
			sprintf(buf, "result_wall_cloud_%02d.pcd", convergence_criteria_.iterations);
			filename = debug_path_ + "/" + buf;
			ipl::write_PointCloud(filename, *dst_cloud);

			if (use_ceiling_) {
				transform_pointcloud(*trans_ceiling_cloud_, *dst_cloud, &x[0]);
				sprintf(buf, "result_ceiling_cloud_%02d.pcd", convergence_criteria_.iterations);
				filename = debug_path_ + "/" + buf;
				ipl::write_PointCloud(filename, *dst_cloud);
			}
#endif

			convergence_criteria_.iterations++;
			convergence_criteria_.cur_corresponding_mse = lm.fvec.norm()/*/sqrt(nCor)*/;
#ifdef __EQUALWEIGHT
			convergence_criteria_.cur_corresponding_mse /= sqrt(nCor);
#endif
			convergence_criteria_.cur_relative_unknowns_mse = lm.parameters.xtol;

			std::cout << "iterator: " << convergence_criteria_.iterations 
				<< "\tmse: " << convergence_criteria_.cur_corresponding_mse << std::endl;
			std::cout << "tx, ty, tz, roll, pitch, yaw" << std::endl;
			std::cout << x[0] << "\t" << x[1] << "\t" << x[2] << "\t" << x[3] << "\t" << x[4] << "\t" << x[5] << std::endl;

			if(convergence_criteria_.iterations > convergence_criteria_.max_iterations
				|| convergence_criteria_.cur_corresponding_mse < convergence_criteria_.corresponding_mse_threshold
				|| convergence_criteria_.cur_relative_unknowns_mse < convergence_criteria_.cur_relative_unknowns_mse)
				break;

		} while (true);
 
		reg_rms_ = convergence_criteria_.cur_corresponding_mse;
		return reg_rms_;
	}

	double ipcFineRegistering::do_ICP_PointToPoint(double corresponding_mse, double unknowns_mse, int maxIter)
	{
		ICP_preparation();

		//////////////////////////////////////////////////////////////////////////////////////////////////
		////////  2. find NN corresponding (ICP) + estimate transformation in an iterated way ////////////

		fine_transform_ = init_transform_;
		convergence_criteria_.max_iterations = maxIter;
		convergence_criteria_.corresponding_mse_threshold = corresponding_mse;
		convergence_criteria_.relative_unknowns_mse_threshold = unknowns_mse;

		convergence_criteria_.iterations = 0;
		convergence_criteria_.cur_corresponding_mse = std::numeric_limits<double>::max();
		convergence_criteria_.cur_relative_unknowns_mse = std::numeric_limits<double>::max();

		int min_number_correspondences = 6;  //最小对应点数 (观测值不可小于未知数个数)
		double corr_dist_threshold = search_buf_size_ * 0.6;
		corr_dist_threshold *= corr_dist_threshold;

		trans_wall_cloud_.reset(new iplPointCloud<PointT>);
		trans_ceiling_cloud_.reset(new iplPointCloud<PointT>);
		std::cout << "iterated 3D regisatration..." << std::endl;
		do
		{
			transform_pointcloud(*resampled_wall_cloud_, *trans_wall_cloud_, fine_transform_);
			if(use_ceiling_)
				transform_pointcloud(*resampled_ceiling_cloud_, *trans_ceiling_cloud_, fine_transform_);

#ifdef __TEST
//			std::string path = "D:/iplTestData/TargetLocalization/target_dataset1/result/";
			std::string filename;
			char buf[64];
			sprintf(buf, "resampled_wall_%02d.pcd", convergence_criteria_.iterations);
			filename = debug_path_ + "/" + buf;
			ipl::write_PointCloud(filename, *trans_wall_cloud_);

			if(use_ceiling_){
			sprintf(buf, "resampled_ceiling_%02d.pcd", convergence_criteria_.iterations);
			filename = debug_path_ + "/" + buf;
			ipl::write_PointCloud(filename, *trans_ceiling_cloud_);
			}
#endif

#ifdef __TEST
			ref_ptr<iplPointCloud<pcl::PointXYZL> > cor_wall_cloud(new iplPointCloud<pcl::PointXYZL>);
			ref_ptr<iplPointCloud<pcl::PointXYZL> > cor_ceiling_cloud(new iplPointCloud<pcl::PointXYZL>);
#endif

			std::vector<double> residuals;
			double sumWTW = 0;
			std::vector<Point2PointCorrespondence> correspondences;
			size_t nwall_cor = 0;
			//提取墙面对应
			for (size_t i = 0; i < trans_wall_cloud_->size(); ++i)
			{
				pcl::PointXYZL searchPoint;
				searchPoint.x = trans_wall_cloud_->points[i].x;
				searchPoint.y = trans_wall_cloud_->points[i].y;
				searchPoint.z = trans_wall_cloud_->points[i].z;

				std::vector<int> k_indices;
				std::vector<float> k_sqr_distances;
				wall_search_->nearestKSearch(searchPoint, 1, k_indices, k_sqr_distances);

				if (k_sqr_distances[0] < corr_dist_threshold)
				{
					Point2PointCorrespondence  corr;
					corr.x = searchPoint.x;
					corr.y = searchPoint.y;
					corr.z = searchPoint.z;
					corr.w = resampled_wall_weights_[i];

					sumWTW += corr.w*corr.w;

					int ref_id = k_indices[0];
					//uint32_t planeId = ref_wall_cloud_->points[ref_id].label;

					corr.xr = ref_wall_cloud_->points[ref_id].x;
					corr.yr = ref_wall_cloud_->points[ref_id].y;
					corr.zr = ref_wall_cloud_->points[ref_id].z;
					//corr.d = ref_wall_params_[planeId].d;

					correspondences.push_back(corr);
					residuals.push_back(k_sqr_distances[0]);
					nwall_cor++;

#ifdef __TEST
					cor_wall_cloud->points.push_back(ref_wall_cloud_->points[ref_id]);
#endif
				}
			}

			size_t nceiling_cor = 0;
			if(use_ceiling_){
			//提取顶面对应
			for (size_t i = 0; i < trans_ceiling_cloud_->size(); ++i)
			{
				pcl::PointXYZL searchPoint;
				searchPoint.x = trans_ceiling_cloud_->points[i].x;
				searchPoint.y = trans_ceiling_cloud_->points[i].y;
				searchPoint.z = trans_ceiling_cloud_->points[i].z;

				std::vector<int> k_indices;
				std::vector<float> k_sqr_distances;
				ceiling_search_->nearestKSearch(searchPoint, 1, k_indices, k_sqr_distances);

				if (k_sqr_distances[0] < corr_dist_threshold)
				{
					Point2PointCorrespondence  corr;
					corr.x = searchPoint.x;
					corr.y = searchPoint.y;
					corr.z = searchPoint.z;
					corr.w = resampled_ceiling_weights_[i];

					sumWTW += corr.w*corr.w;

					int ref_id = k_indices[0];
					//uint32_t planeId = ref_ceiling_cloud_->points[ref_id].label;

					corr.xr = ref_ceiling_cloud_->points[ref_id].x;
					corr.yr = ref_ceiling_cloud_->points[ref_id].y;
					corr.zr = ref_ceiling_cloud_->points[ref_id].z;

					correspondences.push_back(corr);
					residuals.push_back(k_sqr_distances[0]);
					nceiling_cor++;

#ifdef __TEST
					cor_ceiling_cloud->points.push_back(ref_ceiling_cloud_->points[ref_id]);
#endif
				}
			}

			}

#ifdef __TEST
			cor_wall_cloud->height = 1;
			cor_wall_cloud->width = cor_wall_cloud->size();

			// 			std::string path1 = "D:/iplTestData/TargetLocalization/target_dataset1/result/";
			// 			std::string filename;
			// 			char buf1[32];
			sprintf(buf, "correspond_wall_cloud_%02d.pcd", convergence_criteria_.iterations);
			filename = debug_path_ + "/" + buf;
			ipl::write_PointCloud(filename, *cor_wall_cloud);

			if(use_ceiling_){
			cor_ceiling_cloud->height = 1;
			cor_ceiling_cloud->width = cor_ceiling_cloud->size();
			sprintf(buf, "correspond_ceiling_cloud_%02d.pcd", convergence_criteria_.iterations);
			filename = debug_path_ + "/" + buf;
			ipl::write_PointCloud(filename, *cor_ceiling_cloud);
			}
#endif

			double nor_factor = 1.0 / sqrt(sumWTW);  //权归一化因子  P = WT*W

			size_t nCor = correspondences.size();
			if (nCor < min_number_correspondences)
			{
				std::cout << "the correspondence is less than the minimal requirements!" << std::endl;
				break;
			}
			std::cout << "find corresponding pairs: " << nCor << ";";
			std::cout << " wall: " << nwall_cor << ";";

			if (use_ceiling_)
				std::cout << " ceiling: " << nceiling_cor;
			std::cout << std::endl;

//			double VTV0 = 0;
			//权归一化
			for (size_t i = 0; i < nCor; i++)
			{
#ifdef __EQUALWEIGHT
				correspondences[i].w = 1.0;
#else
				correspondences[i].w *= nor_factor;
#endif
				if (!use_ceiling_)
				{
					correspondences[i].z = correspondences[i].zr = 0;
				}
//				VTV0 += residuals[i] * residuals[i] * correspondences[i].w*correspondences[i].w;
			}

			if (convergence_criteria_.iterations == 0)
			{
				double initRms = 0;
				for (size_t i = 0; i < residuals.size(); i++)
				{
					initRms += residuals[i] * residuals[i] * correspondences[i].w*correspondences[i].w;
				}

#ifdef __EQUALWEIGHT
				initRms = sqrt(initRms/residuals.size());
#else
				initRms = sqrt(initRms);
#endif
				std::cout << "initial rms: " << initRms << std::endl;
			}

//			VTV0 = sqrt(VTV0);

			//参数估计采用Eigen::LevenbergMarquardt
			OptimizationFunctor_Point2Point functor(6, correspondences);
			//Eigen::NumericalDiff<OptimizationFunctor<double> > num_diff(functor);

			Eigen::LevenbergMarquardt<OptimizationFunctor_Point2Point> lm(functor);

			Eigen::VectorXd x(6);
			x.setZero();

			lm.parameters.xtol = 1e-8;
			lm.parameters.ftol = 1e-8;

// 			lm.minimizeOneStep(x);
// 			double init_norm = lm.fvec.norm();

			int info = lm.minimize(x);

			// Copy the rotation and translation components
			Eigen::Matrix4d transform_matrix;
			transform_matrix.setZero();
			transform_matrix(0, 3) = x[0];
			transform_matrix(1, 3) = x[1];
			transform_matrix(2, 3) = x[2];
			transform_matrix(3, 3) = 1;

			// Compute w from the unit quaternion
// 			Eigen::Quaternion<double> q(0, x[3], x[4], x[5]);
// 			q.w() = static_cast<double> (sqrt(1 - q.dot(q)));
// 			q.normalize();
// 			transform_matrix.topLeftCorner(3, 3) = q.toRotationMatrix();

			Eigen::Matrix3d rotMat;
			rotMat = Eigen::AngleAxisd(x[5], Eigen::Vector3d::UnitZ())
 			* Eigen::AngleAxisd(x[4], Eigen::Vector3d::UnitY())
 			* Eigen::AngleAxisd(x[3], Eigen::Vector3d::UnitX());

// 			Eigen::Matrix3d rotMat1;
// 			RotateMat_XYZ(x[3], x[4], x[5], rotMat1);

// 			Eigen::Matrix3d m1, m2, m3;
// 			m1 = Eigen::AngleAxisd(x[5], Eigen::Vector3d::UnitZ());
// 			m2 = Eigen::AngleAxisd(x[4], Eigen::Vector3d::UnitY());
// 			m3 = Eigen::AngleAxisd(x[3], Eigen::Vector3d::UnitX());
// 
// 			Eigen::Matrix3d rm1, rm2, rm3;
// 			RotateMat_X(x[3], rm3);
// 			RotateMat_Y(x[4], rm2);
// 			RotateMat_Z(x[5], rm1);

			transform_matrix.topLeftCorner(3, 3) = rotMat;

#ifdef __TEST
// 			double roll, pitch, yaw;
// 			roll = x[3]; pitch = -x[4]; yaw = x[5];
// 
// 			double cos_r, sin_r, cos_p, sin_p, cos_y, sin_y;
// 			cos_r = cos(roll);		sin_r = sin(roll);
// 			cos_p = cos(pitch);		sin_p = sin(pitch);
// 			cos_y = cos(yaw);		sin_y = sin(yaw);
// 
// 			transform_matrix(0, 0) = cos_p*cos_y;
// 			transform_matrix(0, 1) = cos_p*sin_y;
// 			transform_matrix(0, 2) = -sin_p;
// 			transform_matrix(1, 0) = sin_r*sin_p*cos_y - cos_r*sin_y;
// 			transform_matrix(1, 1) = sin_r*sin_p*sin_y + cos_r*cos_y;
// 			transform_matrix(1, 2) = sin_r*cos_p;
// 			transform_matrix(2, 0) = cos_r*sin_p*cos_y + sin_r*sin_y;
// 			transform_matrix(2, 1) = cos_r*sin_p*sin_y - sin_r*cos_y;
// 			transform_matrix(2, 2) = cos_r*cos_p;
#endif
			
			/////////////////////////////////////////////////////////////////////
			fine_transform_.matrix() = transform_matrix * fine_transform_.matrix();
			/////////////////////////////////////////////////////////////////////

#ifdef __TEST
			ref_ptr<iplPointCloud<PointT> > dst_cloud(new iplPointCloud<PointT>);
			transform_pointcloud(*trans_wall_cloud_, *dst_cloud, &x[0]);
			sprintf(buf, "result_wall_cloud_%02d.pcd", convergence_criteria_.iterations);
			filename = debug_path_ + "/" + buf;
			ipl::write_PointCloud(filename, *dst_cloud);

			if(use_ceiling_){
			transform_pointcloud(*trans_ceiling_cloud_, *dst_cloud, &x[0]);
			sprintf(buf, "result_ceiling_cloud_%02d.pcd", convergence_criteria_.iterations);
			filename = debug_path_ + "/" + buf;
			ipl::write_PointCloud(filename, *dst_cloud);
			}
#endif

			convergence_criteria_.iterations++;
			convergence_criteria_.cur_corresponding_mse = lm.fvec.norm()/*/sqrt(nCor)*/;
#ifdef __EQUALWEIGHT
			convergence_criteria_.cur_corresponding_mse /= sqrt(nCor);
#endif
			convergence_criteria_.cur_relative_unknowns_mse = lm.parameters.xtol;

			std::cout << "iterator: " << convergence_criteria_.iterations
				<< "\tmse: " << convergence_criteria_.cur_corresponding_mse << std::endl;
			std::cout << "tx, ty, tz, roll, pitch, yaw" << std::endl;
			std::cout << x[0] << "\t" << x[1] << "\t" << x[2] << "\t" << x[3] << "\t" << x[4] << "\t" << x[5] << std::endl;

			if (convergence_criteria_.iterations > convergence_criteria_.max_iterations
				|| convergence_criteria_.cur_corresponding_mse < convergence_criteria_.corresponding_mse_threshold
				|| convergence_criteria_.cur_relative_unknowns_mse < convergence_criteria_.cur_relative_unknowns_mse)
				break;

		} while (true);

		reg_rms_ = convergence_criteria_.cur_corresponding_mse;
		return reg_rms_;
	}

}

