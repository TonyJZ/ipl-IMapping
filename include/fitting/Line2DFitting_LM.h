#pragma once
#include "core/iplcore.h"
#include "optimization/LevenbergMarquardt.h"

namespace ipl
{
#ifndef _POINT2D_W_Struc_
#define _POINT2D_W_Struc_
	struct Point2D_W
	{
		double x, y;
		double w;   //weight
	};
#endif // !POINT3DStruc

	//the solver for Point-to-line2D 
	struct Fitting_Point2Line2D : LM_Functor<double>
	{
		/** LM_Functor constructor
		* \param[in] m_data_points the number of data points to evaluate
		* \param[in,out] estimator pointer to the estimator object
		*/
		const std::vector<Point2D_W> *pts_;
		int nUnknowns_;
		//nUnknowns = 3,  a*x+b*y+c=0
		Fitting_Point2Line2D(int nUnknowns, const std::vector<Point2D_W> *pts)
			: LM_Functor<double>(nUnknowns, static_cast<int>(pts->size()))
		{
			nUnknowns_ = nUnknowns;
			pts_ = pts;
		}

		/** Fill fvec from x. For the current state vector x fill the f values
		* \param[in] x state vector
		* \param[out] fvec f values vector
		*/
		int operator () (const Eigen::VectorXd &x, Eigen::VectorXd &fvec) const
		{
			double a, b, c;
			a = x[0]; b = x[1]; c = x[2];

			double a2b2_inv = 1.0 / (a*a + b*b);

			for (int i = 0; i < pts_->size(); ++i)
			{
				double x, y, w;

				x = (*pts_)[i].x;
				y = (*pts_)[i].y;
				//z = (*pts_)[i].z;
				w = (*pts_)[i].w;

				double dis2 = iplsqr(a*x + b*y + c)*a2b2_inv;

				fvec[i] = dis2*w;   //LB = WL
			}
			return (0);
		};

		int df(const Eigen::VectorXd &x, Eigen::MatrixXd &fjac) const
		{
			double a, b, c;
			a = x[0]; b = x[1]; c = x[2]; 
			double a2b2_inv = 1.0 / (a*a + b*b);
			double a2b2_inv2 = iplsqr(a2b2_inv);

			//此处建立雅可比矩阵[一阶导数]
			for (int i = 0; i < pts_->size(); ++i)
			{
				double x, y;
				double w;

				x = (*pts_)[i].x;
				y = (*pts_)[i].y;
				//z = (*pts_)[i].z;
				w = (*pts_)[i].w;

				double dis = a*x + b*y + c;
				double dis2 = iplsqr(dis);
				//da
				fjac(i, 0) = a2b2_inv * 2 * dis*x - a2b2_inv2 * 2 * dis2*a;

				//db
				fjac(i, 1) = a2b2_inv * 2 * dis*y - a2b2_inv2 * 2 * dis2*b;

				//dc
				fjac(i, 2) = a2b2_inv * 2 * dis;

				for (int j = 0; j < nUnknowns_; ++j)
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

	//LS算法的局限性: https://zhuanlan.zhihu.com/p/36429715  
	inline bool fit_line2D_by_LM(double *xCorrds, double *yCorrds, int n, iplModelCoeffs *linecoef)
	{
		std::vector<Point2D_W>  pts;
		Eigen::VectorXd x(3);
		//double x[4];  //选择平面初值
		x.setZero();

		//coef: ax+by+cz+d=0;
		x[0] = linecoef->values[0];
		x[1] = linecoef->values[1];
		x[2] = linecoef->values[3];
//		x[3] = linecoef->coef.values[3];

		//1. 准备观测值和未知数
		size_t maxNum = 0;
		for (size_t i = 0; i < n; ++i)
		{
			//geoModel::geoModelInfo coef = clusters[i]->getModelCoef();
			double pw = 1.0;  //point weight

			Point2D_W pt;
			pt.x = xCorrds[i];
			pt.y = yCorrds[i];
			pt.w = pw;

			pts.push_back(pt);
		}

		//2. 求解平面参数
		if (pts.size() < 3)
		{
			std::cout << "not enough points to fit a plane!" << std::endl;
			return -1;
		}

		//参数估计采用Eigen::LevenbergMarquardt
		Fitting_Point2Line2D functor(3, &pts);
		Eigen::LevenbergMarquardt<Fitting_Point2Line2D> lm(functor);

		lm.parameters.xtol = 1e-8;
		lm.parameters.ftol = 1e-8;

		int info = lm.minimize(x);

//		linecoef->type = geoModel::gMT_LINE3D;
//		linecoef->coef.values.resize(4);
		linecoef->values[0] = x[0];
		linecoef->values[1] = x[1];
		linecoef->values[2] = 0;
		linecoef->values[3] = x[2];

		double sumE = 0;
		for (int i = 0; i < n; ++i)
		{
			sumE += lm.fvec[i];
		}
		double rms = sqrt(sumE / n);

		std::cout << "fitting RMS: " << rms << std::endl;

		return true;
	};

}

