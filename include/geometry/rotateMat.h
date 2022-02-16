#pragma once

#include <math.h>

#include <Eigen/Dense>

/* ����������������ת�ǣ���ָ������v1��ʼ����ʱ����ת��ת������v2ʱ����ת���ĽǶȣ���Χ�� 0 ~360��
* �㷨���£�
* ����ͨ����˺�arccosine�ĵõ���������֮��ļн�
* Ȼ���ж�ͨ��������ж���������֮���λ�ù�ϵ
* ���v2��v1����ʱ�뷽��, ����arccose�ĽǶ�ֵ, ��Χ0 ~180.0(�������ֶ���, ���Թ����������)
* ���򷵻� 360.0 - arecose��ֵ, ����180��360(�������ֶ���, ���Ϊ��)
* ���ػ���ֵ
*/

namespace ipl
{
	inline double get2DRotateAngle(Eigen::Vector2d v1, Eigen::Vector2d v2)
	{
		const double epsilon = 1.0e-6;
		const double nyPI = acos(-1.0);
		double angle;

		double dd = v1.dot(v2);

		if (fabs(dd - 1.0) <= epsilon)
			angle = 0.0;
		else if (fabs(dd + 1.0) <= epsilon)
			angle = nyPI;
		else
		{
			//Eigen::Vector2f cross;

			angle = acos(dd);
			//cross product
			//cross = v1.cross(v2);
			double cross = v1[0] * v2[1] - v1[1] * v2[0];
			// vector p2 is clockwise from vector p1 
			// with respect to the origin (0.0)
			if (cross < 0) {
				angle = 2 * nyPI - angle;
			}
		}

		//	degree = angle *  180.0 / nyPI;
		return angle;
	};

	inline double get2DRotateAngle(Eigen::Vector2f v1, Eigen::Vector2f v2)
	{
		const double epsilon = 1.0e-6;
		const double nyPI = acos(-1.0);
		double angle;

		double dd = v1.dot(v2);

		if (fabs(dd - 1.0) <= epsilon)
			angle = 0.0;
		else if (fabs(dd + 1.0) <= epsilon)
			angle = nyPI;
		else
		{
			//Eigen::Vector2f cross;

			angle = acos(dd);
			//cross product
			//cross = v1.cross(v2);
			double cross = v1[0] * v2[1] - v1[1] * v2[0];
			// vector p2 is clockwise from vector p1 
			// with respect to the origin (0.0)
			if (cross < 0) {
				angle = 2 * nyPI - angle;
			}
		}

		//	degree = angle *  180.0 / nyPI;
		return angle;
	};

	inline void RotateMat_X(double angle, Eigen::Matrix3d &Mat)
	{
		Mat = Eigen::Matrix3d::Identity();

		Mat(1, 1) = cos(angle);
		Mat(1, 2) = -sin(angle);
		Mat(2, 1) = sin(angle);
		Mat(2, 2) = cos(angle);
	};

	inline void RotateMat_Y(double angle, Eigen::Matrix3d &Mat)
	{
		Mat = Eigen::Matrix3d::Identity();

		Mat(0, 0) = cos(angle);
		Mat(0, 2) = sin(angle);
		Mat(2, 0) = -sin(angle);
		Mat(2, 2) = cos(angle);
	};

	inline void RotateMat_Z(double angle, Eigen::Matrix3d &Mat)
	{
		Mat = Eigen::Matrix3d::Identity();

		Mat(0, 0) = cos(angle);
		Mat(0, 1) = -sin(angle);
		Mat(1, 0) = sin(angle);
		Mat(1, 1) = cos(angle);
	};

	//rotate order: X --> Y --> Z
	inline void RotateMat_XYZ(double alpha, double beta, double gamma, Eigen::Matrix3d &Mat)
	{
		Eigen::Matrix3d matX, matY, matZ;
		RotateMat_X(alpha, matX);
		RotateMat_Y(beta, matY);
		RotateMat_Z(gamma, matZ);

		Mat = matZ*matY*matX;
	};

}
