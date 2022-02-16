#pragma once
#include <core/iplcore.h>
#include "fitting/geoModelDef.h"
#include "spatialindexing/PointVoxelization.h"

namespace ipl
{
	//this class is created for analyzing the distribution of points on a wall
	//����������ȡȫ���ŵ��������ʵ��  2018.03.08 Tony
	//��WallPointDistribution ��д��ͳһ��PlanePointDistribution      2018.04.12   Tony
	//����ƽ�涼���Ծ�����-����ת����XOZƽ��ƽ��
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

		//1. ����ҪAxis-Aligned Transformation������ͳ��
		//ͳ�Ƶ������, 
		double distance2plane(std::vector<double> &distances);
		int getPointNum();
		double getPlaneRMS();
//		void  getHeight(float &floorhei, float &ceilinghei);//��ȡ�����С�߶�
//		void getConsecutiveHeightScope(float &heiScope, const float hTh = 0.025); //��ȡ���������߲����� ֱ��ͼͳ�Ʒ�

		//2. ��ҪAxis-Aligned Transformation (AAT)������ͳ��
		//
		bool reinitializeAAT(float vsize = 0.2);

		//2.1. ƽ������
		//ƽ��ϡ���
		float getSparseDegree(); /*{ return sparse_degree_; };*/
		//����������
		float getBoundRectArea();
		//�����ڲ��Ŀն�����
		float getOccupiedArea(); /*{ return occupied_area_; };*/

		//2.2 ˮƽ��ͶӰֱ������
		//ͶӰ�߶γ���
		float getProjectedLineLength();
		//ͶӰֱ������Զ�����˵�
		void getFurthestPointsOnProjectedLine(iplPOINT3D &ptS, iplPOINT3D &ptE);
		//�����ͨ�߶γ���, ����ȡ�����ͨ�߶εĶ˵�
		float getProjectedLineMaxConnectedLength(iplPOINT3D &ptS, iplPOINT3D &ptE);
		//�����ͨ�߶εĶ˵�
		//void getFurthestConnectedPointsOnProjectedLine(iplPOINT3D &ptS, iplPOINT3D &ptE);
	private:
		bool bAxisAligned_;               //�Ƿ���������任
		geoModel::geoModelInfo coef_;     //ģ�Ͳ���

		bool bGetMinMaxPts_;			//�Ƿ��ȡ����ԭʼ���ݷ�Χ
		Eigen::Vector4f min_pt_, max_pt_; //ԭ�������귶Χ

		Eigen::Vector4f trans_min_pt_, trans_max_pt_;  //�任�����ݵ����귶Χ

// 		float sparse_degree_;
// 		float connected_degree_;
// 		float occupied_area_;
// 		float total_area_;

		ref_ptr<PointCloud>			trans_cloud_;//ת����������ĵ���
		Eigen::Affine3d				transMat_;  //ת������
		Eigen::Matrix<float, 4, 1> centroid_;   //ԭʼ���ݵ�����

		ref_ptr<PointVoxelization<PointT> > voxeler_;
		float vsizeX_, vsizeY_, vsizeZ_;
	};
}


#include "feature/impl/PlanePointDistribution.hpp"
