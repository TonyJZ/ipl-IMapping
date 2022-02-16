#pragma once
#include "core/iplcore.h"
#include "optimization/LevenbergMarquardt.h"

#include <Eigen/Dense>
#include <Eigen/SVD>


namespace ipl
{
	/** \brief fit 2d line with a given point set, estimate line function by least squares adjustment.
	* \param[in] x:  the x-coordinate array
	* \param[in] y:  the y-coordinate array
	*.\param[in] n: number of points
	* \param[out] linecoef: the estimated line parameters
	* \return
	*  * == false on error
	*  * == true on success
	*/
	inline bool fit_line2D_by_LS(double *x, double *y, int n, iplModelCoeffs *linecoef)
	{
		using namespace Eigen;
// 		using namespace Eigen::internal;
// 		using namespace Eigen::Architecture;

		if (n < 2)
		{
			std::cout << "can't fit a line with " << n << " points!" << std::endl;
			return false;
		}
			

// 		n = 4;
// 
// 		//(x, y, 1)*(a,b,c)^T = L   note: 平面方程的一般形式无唯一解   K(ax+by+c)=0, 需要用
// 		MatrixXd A(n, 2);   //y=kx+b
// 		MatrixXd L(n, 1);
// 
// 		for (int i = 0; i < n; ++i)
// 		{
// 			A(i, 0) = x[i];
// 			A(i, 1) = 1;
// 			//A(i, 2) = 1;
// 
// 			L(i, 0) = y[i];
// 		}
// 			
// 		//MatrixXd AT = A.transpose();
// 
// 		MatrixXd X(2, 1);
// 
// 		//X = (AT * A).inverse() * AT * L; //Least squre
// 		X = A.jacobiSvd(ComputeThinU | ComputeThinV).solve(L);

		float a, b, c; //ax+by=c
		a = 0.0f;
		b = 0.0f;
		c = 0.0f;

		double sX = 0, sY = 0;
		double mX, mY, sXX = 0.0, sXY = 0.0, sYY = 0.0;

		//Calculate sums of X and Y
		for (int i = 0; i < n; i++)
		{
			sX += x[i];
			sY += y[i];
			
		}

		//Calculate X and Y means (sample means)
		mX = sX / n;
		mY = sY / n;

		//Calculate sum of X squared, sum of Y squared and sum of X * Y
		//(components of the scatter matrix)
		for (int i = 0; i < n; i++)
		{
			sXX += (x[i] - mX) * (x[i] - mX);
			sXY += (x[i] - mX) * (y[i] - mY);
			sYY += (y[i] - mY) * (y[i] - mY);
		}

		bool isVertical = (fabs(sXY) <1e-6) && (sXX < sYY);
		bool isHorizontal = (fabs(sXY) <1e-6) && (sXX > sYY);
		bool isIndeterminate = (fabs(sXY) <1e-6) && (fabs(sXX - sYY) < 1e-6);
		
		double slope;
		double intercept;

		if (isVertical)
		{
			a = 1.0f;
			b = 0.0f;
			c = (float)mX;
		}
		else if (isHorizontal)
		{
			a = 0.0f;
			b = 1.0f;
			c = (float)mY;
		}
		else if (isIndeterminate)
		{
			a = 0;
			b = 0;
			c = 0;
		}
		else
		{
			slope = (sYY - sXX + sqrt((sYY - sXX) * (sYY - sXX) + 4.0 * sXY * sXY)) / (2.0 * sXY);
			intercept = mY - slope * mX;
			double normFactor = (intercept >= 0.0 ? 1.0 : -1.0) * sqrt(slope * slope + 1.0);
			a = (float)(-slope / normFactor);
			b = (float)(1.0 / normFactor);
			c = (float)(intercept / normFactor);
		}

		linecoef->values.resize(4);  //ax+by+c=0
		linecoef->values[0] = a;
		linecoef->values[1] = b;
		linecoef->values[2] = 0;
		linecoef->values[3] = -c;

		return true;
	};

}
