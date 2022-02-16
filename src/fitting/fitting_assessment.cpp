#include "fitting/fitting_assessment.h"
#include <Eigen/Dense>

namespace ipl
{
	int plane_fitting_covariance(double *x, double *y, double *z, size_t num_pts,
		geoModel::geoModelInfo &pInfo)
	{
		int maxIter = 10;
		float crossTh = cosf(iplDEG2RAD*5.0);

		if (num_pts < 4)
			return -1;

		if (pInfo.type != geoModel::gMT_PLANE || pInfo.coef.values.size() != 4)
			return -1;

		Eigen::VectorXd coorX(num_pts), coorY(num_pts), coorZ(num_pts);

		double meanX, meanY, meanZ;
		meanX = meanY = meanZ = 0.0;
		for (size_t i = 0; i < num_pts; ++i)
		{
			meanX += x[i];
			meanY += y[i];
			meanZ += z[i];
		}

		meanX /= num_pts;
		meanY /= num_pts;
		meanZ /= num_pts;

		for (size_t i = 0; i < num_pts; ++i)
		{
			coorX(i) = x[i] - meanX;
			coorY(i) = y[i] - meanY;
			coorZ(i) = z[i] - meanZ;
		}
		
		Eigen::VectorXd Vdis(num_pts);
		double sumSD;
		int nUnknowns = 4;   //4直接求, 5拉格朗日乘数法
		//Eigen::MatrixXd Vdis2(1, num_pts);  //
		Eigen::MatrixXd Jac(num_pts, nUnknowns);
		Eigen::VectorXd L(num_pts);
 		Eigen::VectorXd DX(nUnknowns); //da, db, dc, dd
 		Eigen::VectorXd X(nUnknowns);

		Vdis = Eigen::VectorXd::Zero(num_pts);
		Jac = Eigen::MatrixXd::Zero(num_pts, nUnknowns);

		//a = pInfo.coef.values[0]; b = pInfo.coef.values[1]; c = pInfo.coef.values[2]; d = pInfo.coef.values[3];
		X = Eigen::VectorXd::Zero(nUnknowns);
		X(0) = pInfo.coef.values[0]; 
		X(1) = pInfo.coef.values[1];
		X(2) = pInfo.coef.values[2];
		X(3) = pInfo.coef.values[3] + X(0)*meanX + X(1)*meanY + X(2)*meanZ;
		//X.normalize();
		Eigen::Vector3f  n0(X(0), X(1), X(2)), n1;
		n0.normalize();

		int iter = 0;
		float dot_product = 0;
		Eigen::MatrixXd ATA(nUnknowns, nUnknowns), ATL(nUnknowns, 1);
		Eigen::MatrixXd ATA_inv(nUnknowns, nUnknowns);
		do 
		{
			double a, b, c, d, lambda;
			double sum_a2b2c2 = 0, sqrt_sum_a2b2c2 = 0; //sqrt(a2+b2+c2)
			a = X(0); b = X(1); c = X(2); d = X(3); //lambda = X(4);
			sum_a2b2c2 += a*a;
			sum_a2b2c2 += b*b;
			sum_a2b2c2 += c*c;

			sqrt_sum_a2b2c2 = sqrt(sum_a2b2c2);
			double sqrt_sum_a2b2c2_inv = 1.0 / sqrt_sum_a2b2c2;
			double sqrt_sum_a2b2c2_inv3 = sqrt_sum_a2b2c2_inv*sqrt_sum_a2b2c2_inv*sqrt_sum_a2b2c2_inv;

			sumSD = 0;
			for (size_t i = 0; i < num_pts; ++i)
			{
				double p = coorX[i] * a + coorY[i] * b + coorZ[i] * c + d;
				//double p2 = p*p;

				L(i) = -p*sqrt_sum_a2b2c2_inv/*+lambda*(sum_a2b2c2-1.0)*/;

				Vdis(i) = p*sqrt_sum_a2b2c2_inv;
				sumSD += Vdis(i)*Vdis(i);

				Jac(i, 0) = x[i] * sqrt_sum_a2b2c2_inv - a*p*sqrt_sum_a2b2c2_inv3/* - 2*a*lambda*/;
				Jac(i, 1) = y[i] * sqrt_sum_a2b2c2_inv - b*p*sqrt_sum_a2b2c2_inv3/* - 2*b*lambda*/;
				Jac(i, 2) = z[i] * sqrt_sum_a2b2c2_inv - c*p*sqrt_sum_a2b2c2_inv3/* - 2*c*lambda*/;
				Jac(i, 3) = sqrt_sum_a2b2c2_inv;
				//Jac(i, 4) = 1.0 - sum_a2b2c2;
			}

			
			ATA = Jac.transpose()*Jac;
			ATL = Jac.transpose()*L;

			ATA_inv = ATA.inverse();
			DX = ATA_inv*ATL;

			X += DX;
			//X.normalize();
			n1 = Eigen::Vector3f(X(0), X(1), X(2));
			n1.normalize();
			//X(0) = n1(0); X(1) = n1(1); X(2) = n1(2);


			dot_product = fabsf(n1.dot(n0));

			if(sqrt(sumSD/(num_pts-4))<1e-3)
				break;

			iter++;

		} while (/*dot_product < crossTh && */iter < maxIter);

		Eigen::MatrixXd Hess(4, 4);
		// 		Hess = WA.transpose()*WA;
		// 
		// 		double det = Hess.determinant();

		n1.normalize();

// 		pInfo.coef.values[0] = n1(0);
// 		pInfo.coef.values[1] = n1(1);
// 		pInfo.coef.values[2] = n1(2);
// 		pInfo.coef.values[3] = X(3);

// 		Hess = Jac.transpose()*Jac;
// 		double det = Hess.determinant();  //矩阵可逆条件, 行列式值不为0
//		assert(fabs(det) > 1e-6);

		Eigen::MatrixXd Q(4, 4);
		Q = ATA_inv;

		pInfo.obsSD = sqrt(sumSD / (num_pts - 4)); //无偏估计
		pInfo.coefVar.resize(4);
		for (size_t i = 0; i < 4; ++i)
		{
			pInfo.coefVar[i] = Q(i, i);
		}

		return 0;
	}

}


