#pragma once
#include <core/iplcore.h>
#include "fitting/geoModelDef.h"
#include "spatialindexing/PointVoxelization.h"

namespace ipl
{
	//this class is created for analyzing the distribution of points on a wall
	//将面特征提取全部放到这个类中实现  2018.03.08 Tony
	//将WallPointDistribution 改写成统一的PlanePointDistribution      2018.04.12   Tony
	//任意平面都可以经过轴-角旋转到与XOZ平面平行
	template <typename PointT>
	class PlanePointDistribution : public ipl::iplPointCluster<PointT>
		{
	public:
		using ipl::iplPointCluster<PointT>::indices_;
		typedef ipl::iplPointCloud<PointT> PointCloud;
		typedef typename PointCloud::Ptr PointCloudPtr;


	public:
		PlanePointDistribution();
		~PlanePointDistribution();
		void setPlaneCoef(geoModel::geoModelInfo coef);
		//void setHeightScope(float floorHei, float ceilingHei);

		//1. 不需要Axis-Aligned Transformation的特征统计
		//统计点面距离, 
		double distance2plane(std::vector<double> &distances);
		int getPointNum();
		double getPlaneRMS();
//		void  getHeight(float &floorhei, float &ceilinghei);//提取最大最小高度
//		void getConsecutiveHeightScope(float &heiScope, const float hTh = 0.025); //提取连续的最大高层区间 直方图统计法

		//2. 需要Axis-Aligned Transformation (AAT)的特征统计
		//
		bool reinitializeAAT(float vsize = 0.2);

		//2.1. 平面特征
		//平面稀疏度
		float getSparseDegree(); /*{ return sparse_degree_; };*/
		//包络矩形面积
		float getBoundRectArea();
		//忽略内部的空洞区域
		float getOccupiedArea(); /*{ return occupied_area_; };*/

		//2.2 水平面投影直线特征
		//投影线段长度
		float getProjectedLineLength();
		//投影直线上最远两个端点
		void getFurthestPointsOnProjectedLine(iplPOINT3D &ptS, iplPOINT3D &ptE);
		//最大连通线段长度, 并提取最大连通线段的端点
		float getProjectedLineMaxConnectedLength(iplPOINT3D &ptS, iplPOINT3D &ptE);
		//最大连通线段的端点
		//void getFurthestConnectedPointsOnProjectedLine(iplPOINT3D &ptS, iplPOINT3D &ptE);
	private:
		bool bAxisAligned_;               //是否完成轴对齐变换
		geoModel::geoModelInfo coef_;     //模型参数

		bool bGetMinMaxPts_;			//是否获取到了原始数据范围
		Eigen::Vector4f min_pt_, max_pt_; //原数据坐标范围

		Eigen::Vector4f trans_min_pt_, trans_max_pt_;  //变换后数据的坐标范围

// 		float sparse_degree_;
// 		float connected_degree_;
// 		float occupied_area_;
// 		float total_area_;

		ref_ptr<PointCloud>			trans_cloud_;//转换成轴对齐后的点云
		Eigen::Affine3d				transMat_;  //转换矩阵
		Eigen::Matrix<float, 4, 1> centroid_;   //原始数据的中心

		ref_ptr<PointVoxelization<PointT> > voxeler_;
		float vsizeX_, vsizeY_, vsizeZ_;
	};
}


#include "feature/impl/PlanePointDistribution.hpp"
