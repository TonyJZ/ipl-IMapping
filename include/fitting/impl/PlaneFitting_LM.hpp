#pragma once
#include "fitting/PlaneFitting_LM.h"
#include "optimization/LevenbergMarquardt.h"
#include "fitting/fitting_assessment.h"

namespace ipl
{
#ifndef _POINT3D_W_Struc_
#define _POINT3D_W_Struc_
	struct Point3D_W
	{
		double x, y, z;
		double w;   //weight
	};
#endif // !POINT3DStruc

	
	//the solver for Point-to-Plane 
	struct Fitting_Point2Plane : LM_Functor<double>
	{
		/** LM_Functor constructor
		* \param[in] m_data_points the number of data points to evaluate
		* \param[in,out] estimator pointer to the estimator object
		*/
		const std::vector<Point3D_W> *pts_;

		Fitting_Point2Plane(int nUnknowns, const std::vector<Point3D_W> *pts)
			: LM_Functor<double>(nUnknowns, static_cast<int>(pts->size()))
		{
			pts_ = pts;
		}

		/** Fill fvec from x. For the current state vector x fill the f values
		* \param[in] x state vector
		* \param[out] fvec f values vector
		*/
		int operator () (const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
		{
			double a, b, c, d;
			a = x[0]; b = x[1]; c = x[2]; d = x[3];

			double a2b2c2_inv = 1.0 / (a*a + b*b + c*c);

			for (int i = 0; i < pts_->size(); ++i)
			{
				double x, y, z, w;
				
				x = (*pts_)[i].x;
				y = (*pts_)[i].y;
				z = (*pts_)[i].z;
				w = (*pts_)[i].w;

				double dis2 = iplsqr(a*x + b*y + c*z + d)*a2b2c2_inv;

				fvec[i] = dis2*w;   //LB = WL
			}
			return (0);
		};

		int df(const Eigen::VectorXd &x, Eigen::MatrixXd &fjac) const
		{
			double a, b, c, d;
			a = x[0]; b = x[1]; c = x[2]; d = x[3];
			double a2b2c2_inv = 1.0 / (a*a + b*b + c*c);
			double a2b2c2_inv2 = iplsqr(a2b2c2_inv);

			//此处建立雅可比矩阵[一阶导数]
			for (int i = 0; i < pts_->size(); ++i)
			{
				double x, y, z;
				double w;

				x = (*pts_)[i].x;
				y = (*pts_)[i].y;
				z = (*pts_)[i].z;
				w = (*pts_)[i].w;

				double dis = a*x + b*y + c*z + d;
				double dis2 = iplsqr(dis);
				//da
				fjac(i, 0) = a2b2c2_inv * 2 * dis*x - a2b2c2_inv2 * 2 * dis2*a;

				//db
				fjac(i, 1) = a2b2c2_inv * 2 * dis*y - a2b2c2_inv2 * 2 * dis2*b;
				
				//dc
				fjac(i, 2) = a2b2c2_inv * 2 * dis*z - a2b2c2_inv2 * 2 * dis2*c;

				//dd
				fjac(i, 3) = a2b2c2_inv * 2 * dis;

				for (int j = 0; j < 4; ++j)
				{
					fjac(i, j) *= w;  //B = WA
				}
			}

			return 0;
		};

		// 		void setCorrespondences(std::vector<Point2PlaneCorrespondence> correspondence)
		// 		{
		// 			correspondences_ = correspondence;
		// 		}
		//const TransformationEstimationLM<PointSource, PointTarget, MatScalar> *estimator_;
	};

	template<typename PointT>
	int plane_fitting_LM(std::vector<ref_ptr<PointGeoSegment<PointT> > > &clusters, geoModel::geoModelInfo &refined_coef)
	{
		std::vector<Point3D_W>  pts;
		Eigen::VectorXd x(4);
		//double x[4];  //选择平面初值
		x.setZero();


		//1. 准备观测值和未知数
		size_t maxNum = 0;
		for (size_t i = 0; i < clusters.size(); ++i)
		{
			geoModel::geoModelInfo coef = clusters[i]->getModelCoef();
			double pw = 1.0 / coef.obsSD;  //point weight

			size_t curSegNum = 0;
			
			iplPointCloud<PointT>::ConstPtr cloud = clusters[i]->getInputCloud();
			pcl::IndicesConstPtr indices = clusters[i]->getIndices();

			if (indices != NULL)
			{//根据索引取点
				curSegNum = indices->size();
				for (size_t j = 0; j < indices->size(); ++j)
				{
					int id = (*indices)[j];
					Point3D_W pt;
					pt.x = cloud->points[id].x;
					pt.y = cloud->points[id].y;
					pt.z = cloud->points[id].z;
					pt.w = pw;

					pts.push_back(pt);
				}
			}
			else
			{//直接取点
				curSegNum = cloud->size();
				for (size_t j = 0; j < cloud->size(); ++j)
				{
					Point3D_W pt;
					pt.x = cloud->points[j].x;
					pt.y = cloud->points[j].y;
					pt.z = cloud->points[j].z;
					pt.w = pw;

					pts.push_back(pt);
				}
			}

			if (curSegNum > maxNum)
			{
				x[0] = coef.coef.values[0];
				x[1] = coef.coef.values[1];
				x[2] = coef.coef.values[2];
				x[3] = coef.coef.values[3];
			}
		}

		//2. 求解平面参数
		if (pts.size() < 4)
		{
			std::cout << "not enough points to fit a plane!" << std::endl;
			return -1;
		}
			
		//参数估计采用Eigen::LevenbergMarquardt
		Fitting_Point2Plane functor(4, &pts);
		Eigen::LevenbergMarquardt<Fitting_Point2Plane> lm(functor);

		lm.parameters.xtol = 1e-8;
		lm.parameters.ftol = 1e-8;

		int info = lm.minimize(x);


		//计算参数估计的协方差
		refined_coef.bbox.min_pt[0] = refined_coef.bbox.min_pt[1] = refined_coef.bbox.min_pt[2] = std::numeric_limits<double>::max();
		refined_coef.bbox.max_pt[0] = refined_coef.bbox.max_pt[1] = refined_coef.bbox.max_pt[2] = std::numeric_limits<double>::lowest();
		std::vector<double> X_Arr, Y_Arr, Z_Arr;
		for (size_t i = 0; i < pts.size(); ++i)
		{
			if (refined_coef.bbox.min_pt[0] > pts[i].x)
				refined_coef.bbox.min_pt[0] = pts[i].x;
			if (refined_coef.bbox.max_pt[0] < pts[i].x)
				refined_coef.bbox.max_pt[0] = pts[i].x;
			if (refined_coef.bbox.min_pt[1] > pts[i].y)
				refined_coef.bbox.min_pt[1] = pts[i].y;
			if (refined_coef.bbox.max_pt[1] < pts[i].y)
				refined_coef.bbox.max_pt[1] = pts[i].y;
			if (refined_coef.bbox.min_pt[2] > pts[i].z)
				refined_coef.bbox.min_pt[2] = pts[i].z;
			if (refined_coef.bbox.max_pt[2] < pts[i].z)
				refined_coef.bbox.max_pt[2] = pts[i].z;

			X_Arr.push_back(pts[i].x);
			Y_Arr.push_back(pts[i].y);
			Z_Arr.push_back(pts[i].z);
		}

		refined_coef.type = geoModel::gMT_PLANE;
		refined_coef.coef.values.resize(4);
		refined_coef.coef.values[0] = x[0];
		refined_coef.coef.values[1] = x[1];
		refined_coef.coef.values[2] = x[2];
		refined_coef.coef.values[3] = x[3];

		int ret = plane_fitting_covariance(&X_Arr[0], &Y_Arr[0], &Z_Arr[0], pts.size(), refined_coef);
		if (ret != 0)
		{
			std::cout << "can not get plane fitting covariance!" << std::endl;
			return ret;
		}

		return 0;
	}
}

