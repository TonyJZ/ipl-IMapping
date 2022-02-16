#pragma once
#include "core/iplcore.h"
#include "fitting/geoModelDef.h"
//#include "feature/WallPointDistribution.h"

#include <pcl/search/search.h>

namespace ipl
{
	//PointGeoSegment is a class that contains a set of points belong to a certain geo-model
	//it is created for statistic the features of points
	template <typename PointT>
	class PointGeoSegment : public ipl::iplPointCluster<PointT>
	{
	public:
		typedef ipl::iplPointCloud< PointT > PointCloud;
		typedef typename PointCloud::ConstPtr PointCloudConstPtr;
		typedef typename PointCloud::Ptr PointCloudPtr;

// 		using ipl::iplPointCluster<PointT>::initCompute;
// 		using ipl::iplPointCluster<PointT>::deinitCompute;
		using ipl::iplPointCluster<PointT>::indices_;
		//	using pcl::PCLBase<PointT>::input_;

		typedef pcl::search::Search <PointT> KdTree;
		typedef typename KdTree::Ptr KdTreePtr;

	public:
		PointGeoSegment();
		~PointGeoSegment();

		// 		virtual void
		// 			setInputCloud(const PointCloudConstPtr &cloud);

		void setModelCoef(/*ipl::geoModel::geoModelType type, */ipl::geoModel::geoModelInfo coef);

		const ipl::geoModel::geoModelType getModelType()
		{
			return coef_.type;
		};

		const ipl::geoModel::geoModelInfo getModelCoef()
		{
			return coef_;
		};

		//重新初始化各类特征Flag，进行新一次的统计
		//初始化indices
		void reinitialize();

		void getBBox(Eigen::Vector3d &bbmin, Eigen::Vector3d &bbmax);

		//取法向量均值 （所有点法向量的均值）
		const Eigen::Vector3f getMeanNormal();

		double get3DDiagonal();

		double get2DDiagonal();

		double getFurthestPointsByPCL(PointT &pmin, PointT &pmax);

		double getFurthestPointsApproximate(PointT &pmin, PointT &pmax);

		//nsample: 随机采样点数
		double getProjectionDensity(int nsample, double dc);

		//提取投影直线方程 (平面点拟合直线)
		int getProjectedLineCeof(iplModelCoeffs *coef);

		//计算联通度，分析segment的空间分布情况
//		inline double get2DConnectivity(float vSize, double &maxConnDis);

		//提取投影射线的方向向量, orgPt为给定射线原点
//		void getProjectionLineDirection(PointT orgPt, Eigen::Vector2f &normal);

//		double distance2Model(std::vector<double> &distances);

// 	protected:
// 		PointCloudPtr input_;

// 	private:
// 		ref_ptr<WallPointDistribution<PointT> >   

	private:
		KdTreePtr search_;

		//ipl::geoModel::geoModelType mt_; 
		ipl::geoModel::geoModelInfo coef_;     //模型参数

		bool bFindFurthestPair;
		PointT ptmin_, ptmax_;
		double maxDis_;

		//feaures
		bool bFindbbox;
		std::vector<double> bbox_seg_; //xmin,ymin,zmin,xmax,ymax,zmax
		Eigen::Vector3f low_pt_, high_pt_;//, lb_pt_, rt_pt_; //low, high, leftbottom, righttop 

		Eigen::Vector3f mean_normal_;  //点云的法向量均值
	};
}


#include "feature/impl/PointGeoSegment.hpp"
