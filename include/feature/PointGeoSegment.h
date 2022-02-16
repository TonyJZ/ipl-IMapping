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

		//���³�ʼ����������Flag��������һ�ε�ͳ��
		//��ʼ��indices
		void reinitialize();

		void getBBox(Eigen::Vector3d &bbmin, Eigen::Vector3d &bbmax);

		//ȡ��������ֵ �����е㷨�����ľ�ֵ��
		const Eigen::Vector3f getMeanNormal();

		double get3DDiagonal();

		double get2DDiagonal();

		double getFurthestPointsByPCL(PointT &pmin, PointT &pmax);

		double getFurthestPointsApproximate(PointT &pmin, PointT &pmax);

		//nsample: �����������
		double getProjectionDensity(int nsample, double dc);

		//��ȡͶӰֱ�߷��� (ƽ������ֱ��)
		int getProjectedLineCeof(iplModelCoeffs *coef);

		//������ͨ�ȣ�����segment�Ŀռ�ֲ����
//		inline double get2DConnectivity(float vSize, double &maxConnDis);

		//��ȡͶӰ���ߵķ�������, orgPtΪ��������ԭ��
//		void getProjectionLineDirection(PointT orgPt, Eigen::Vector2f &normal);

//		double distance2Model(std::vector<double> &distances);

// 	protected:
// 		PointCloudPtr input_;

// 	private:
// 		ref_ptr<WallPointDistribution<PointT> >   

	private:
		KdTreePtr search_;

		//ipl::geoModel::geoModelType mt_; 
		ipl::geoModel::geoModelInfo coef_;     //ģ�Ͳ���

		bool bFindFurthestPair;
		PointT ptmin_, ptmax_;
		double maxDis_;

		//feaures
		bool bFindbbox;
		std::vector<double> bbox_seg_; //xmin,ymin,zmin,xmax,ymax,zmax
		Eigen::Vector3f low_pt_, high_pt_;//, lb_pt_, rt_pt_; //low, high, leftbottom, righttop 

		Eigen::Vector3f mean_normal_;  //���Ƶķ�������ֵ
	};
}


#include "feature/impl/PointGeoSegment.hpp"
