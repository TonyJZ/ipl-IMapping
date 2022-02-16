#pragma once

#include "core/iplcore.h"
#include "fitting/geoModelDef.h"
#include "fitting/fitting_assessment.h"

#include <Eigen/Dense>

namespace ipl
{
	/** \brief fit 3d plane with a given point set, estimate line function by least squares adjustment.
	*.\ so that the summed squared distance to all points is minimzized.
	* \param[in] x:  the x-coordinate array
	* \param[in] y:  the y-coordinate array
	*.\param[in] z:  the z-coordinate array
	*.\param[in] n: number of points
	* \param[out] linecoef: the estimated line parameters
	* \return
	*  * == false on error
	*  * == true on success
	*/
	bool fit_plane_by_LS(double *x, double *y, double *z, int n, geoModel::geoModelInfo &coef)
	{
		if (n < 4)
		{
			std::cout << "can't fit a plane with " << n << " points!" << std::endl;
			return false;
		}
			
		double sX = 0, sY = 0, sZ = 0;
		double mX, mY, mZ;

		for (int i = 0; i < n; i++)
		{
			sX += x[i];
			sY += y[i];
			sZ += z[i];
		}

		mX = sX / n;
		mY = sY / n;
		mZ = sZ / n;

		// Calc full 3x3 covariance matrix, excluding symmetries:
		double xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;

		for (int i = 0; i < n; i++)
		{
			xx += (x[i] - mX)*(x[i] - mX);
			xy += (x[i] - mX)*(y[i] - mY);
			xz += (x[i] - mX)*(z[i] - mZ);
			yy += (y[i] - mY)*(y[i] - mY);
			yz += (y[i] - mY)*(z[i] - mZ);
			zz += (z[i] - mZ)*(z[i] - mZ);
		}

		double det_x, det_y, det_z;
		det_x = yy*zz - yz*yz;
		det_y = xx*zz - xz*xz;
		det_z = xx*yy - xy*xy;


		double det_max = std::max(std::max(det_x, det_y), det_z);
		int longest_axis;  
		if (det_x > det_y && det_x > det_z)
			longest_axis = 0;
		else if (det_y > det_x && det_y > det_z)
			longest_axis = 1;
		else if (det_z > det_x && det_z > det_y)
			longest_axis = 2;
		else
			longest_axis = 0; //ball

		if (!(det_max > 0))
		{
			std::cout << "The points don't span a plane" << std::endl;
			return false;
		}

		// Pick path with best conditioning:
		Eigen::Vector3d dir;
		
		if(longest_axis == 0) 
		{
			dir = Eigen::Vector3d(det_x, xz*yz - xy*zz, xy*yz - xz*yy);
		}
		else if(longest_axis == 1)
		{
			dir = Eigen::Vector3d(xz*yz - xy*zz, det_y, xy*xz - yz*xx);
		}
		else if(longest_axis == 2)
		{
			dir = Eigen::Vector3d(xy*yz - xz*yy, xy*xz - yz*xx, det_z);
		}

		dir.normalize();

		coef.type = geoModel::gMT_PLANE;
		coef.coef.values.resize(4);  //ax+by+c=0
		coef.coef.values[0] = dir(0);
		coef.coef.values[1] = dir(1);
		coef.coef.values[2] = dir(2);
		coef.coef.values[3] = -(dir(0)*mX+dir(1)*mY+dir(2)*mZ);

		plane_fitting_covariance(x, y, z, n, coef);
		return true;
	};

}

