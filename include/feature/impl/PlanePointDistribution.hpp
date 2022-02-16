#pragma once
#include "feature/PlanePointDistribution.h"


#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <Eigen/dense>
#include <Eigen/Geometry>

template <typename PointT>
ipl::PlanePointDistribution<PointT>::PlanePointDistribution()
{
	bAxisAligned_ = false;
	bGetMinMaxPts_ = false;

	vsizeX_ = vsizeY_ = vsizeZ_ = 0.0;
}

template <typename PointT>
ipl::PlanePointDistribution<PointT>::~PlanePointDistribution()
{
	if (trans_cloud_ != 0)
		trans_cloud_.reset();

	if (voxeler_ != 0)
		voxeler_.reset();
}

template <typename PointT> void
ipl::PlanePointDistribution<PointT>::setPlaneCoef(geoModel::geoModelInfo coef)
{
	coef_ = coef;
}

// template <typename PointT> void
// ipl::PlanePointDistribution<PointT>::getHeight(float &floorhei, float &ceilinghei)
// {
// 	bool bReady = initCompute();
// 	if (!bReady)
// 	{
// 		deinitCompute();
// 		return;
// 	}
// 
// 	if (!bGetMinMaxPts_)
// 	{
// 		pcl::getMinMax3D(*input_, *indices_, min_pt_, max_pt_);
// 		bGetMinMaxPts_ = true;
// 	}
// 	floorhei = min_pt_[2];
// 	ceilinghei = max_pt_[2];
// }

// template <typename PointT> void
// ipl::PlanePointDistribution<PointT>::getConsecutiveHeightScope(float &heiScope, const float hTh)
// {
// 	float hei_interval = 0.05;  
// 
// 	float min_h, max_h;
// 
// 	bool bReady = initCompute();
// 	if (!bReady)
// 	{
// 		deinitCompute();
// 		return;
// 	}
// 
// 	if (!bGetMinMaxPts_)
// 	{
// 		pcl::getMinMax3D(*input_, *indices_, min_pt_, max_pt_);
// 		bGetMinMaxPts_ = true;
// 	}
// 
// 	min_h = min_pt_[2] - 0.5*hei_interval;
// 	max_h = max_pt_[2] + 0.5*hei_interval;
// 
// 	int nstep = ceil((max_h - min_h) / hei_interval);
// 
// 	std::vector<int> zHist;
// 	std::vector<int> zCumulativeHist;
// 	zHist.resize(nstep + 2, 0);
// 	zCumulativeHist.resize(nstep + 2, 0);
// 
// 	int npts = indices_->size();
// 	for (int i = 0; i < indices_->size(); i++)
// 	{
// 		int id = indices_->at(i);
// 		double z = input_->points[id].z;
// 
// 		int iStep = static_cast<int> (floor((z - min_h) / hei_interval));
// 		zHist[iStep]++;
// 	}
// 
// 	zCumulativeHist[0] = zHist[0];
// 	for (int i = 1; i < zHist.size(); i++)
// 	{
// 		zCumulativeHist[i] = zCumulativeHist[i - 1] + zHist[i];
// 	}
// 
// 	bool bfind_zi = false, bfind_zj = false;
// 	int zi, zj;
// 	zi = zj = 0;
// 	for (int i = 0; i < zCumulativeHist.size(); i++)
// 	{
// 		float ratio = zCumulativeHist[i];
// 		ratio /= npts;
// 
// 		if (!bfind_zi)
// 		{
// 			if (ratio > hTh)
// 			{
// 				zi = i;
// 				bfind_zi = true;
// 			}
// 		}
// 
// 		if (!bfind_zj)
// 		{
// 			if (ratio > 1.0 - hTh)
// 			{
// 				zj = i;
// 				bfind_zj = true;
// 			}
// 		}
// 	}
// 
// 	heiScope = (zj - zi)*hei_interval;
// }

template <typename PointT> double
ipl::PlanePointDistribution<PointT>::distance2plane(std::vector<double> &distances)
{
	double rms = 0;
	double vmod;

	bool bReady = initCompute();
	if (!bReady)
	{
		deinitCompute();
		return std::numeric_limits<double>::min();
	}

	//double x, y, z;
	vmod = sqrt(coef_.coef.values[0] * coef_.coef.values[0] + coef_.coef.values[1] * coef_.coef.values[1]
		+ coef_.coef.values[2] * coef_.coef.values[2]);

	distances.clear();
	for (int i = 0; i < indices_->size(); i++)
	{
		int id = indices_->at(i);
		double dis;

		dis = fabs(input_->points[id].x*coef_.coef.values[0] + input_->points[id].y*coef_.coef.values[1] 
			+ input_->points[id].z*coef_.coef.values[2] + coef_.coef.values[3])
			/ vmod;

		rms += dis*dis;
		distances.push_back(dis);
	}

	if (distances.size() == 0)
		rms = 0;
	else
		rms = sqrt(rms / distances.size());
	
	return rms;
}

template <typename PointT> int
ipl::PlanePointDistribution<PointT>::getPointNum()
{
	bool bReady = initCompute();
	if (!bReady)
	{
		deinitCompute();
		return 0;
	}

	return indices_->size();
}

template <typename PointT> double
ipl::PlanePointDistribution<PointT>::getPlaneRMS()
{
	return coef_.obsSD;
}

template <typename PointT> bool
ipl::PlanePointDistribution<PointT>::reinitializeAAT(float vsize /* = 0.2 */)
{
	bool bReady = initCompute();
	if (!bReady)
	{
		deinitCompute();
		return false;
	}

	if (!bGetMinMaxPts_)
	{
		pcl::getMinMax3D(*input_, *indices_, min_pt_, max_pt_);
		bGetMinMaxPts_ = true;
	}

	pcl::compute3DCentroid(*input_, *indices_, centroid_);

	pcl::PointCloud<PointT>  decentre_cloud;

	for (int i = 0; i < indices_->size(); i++)
	{
		int id = indices_->at(i);
		PointT pt = input_->points[id];

		pt.x -= centroid_[0];
		pt.y -= centroid_[1];
		decentre_cloud.points.push_back(pt);
	}

	decentre_cloud.height = 1;
	decentre_cloud.width = decentre_cloud.size();

	//转换矩阵，将去中心后的数据转换成轴对齐
	//平面法向量与Y轴对齐
	Eigen::Affine3d  transMat_inverse;
	Eigen::Vector3d plane_normal, Y_axis;
	
	Y_axis = Eigen::Vector3d(0, 1, 0);
	plane_normal = Eigen::Vector3d(coef_.coef.values[0], coef_.coef.values[1], coef_.coef.values[2]);

	Eigen::Vector3d rotAxis = plane_normal.cross(Y_axis);
	rotAxis.normalize();
	double dot_product = plane_normal.dot(Y_axis);
	double angle_in_radian = acos(dot_product);


// 	double yaw;
// 	yaw = atan(-coef_.values[0] / coef_.values[1]);

// 	transMat_ = Eigen::Affine3d::Identity();
// 	transMat_.translation() << 0, 0, 0;
// 	transMat_.rotate(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())
// 		* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
// 		* Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));
	transMat_ = Eigen::AngleAxisd(angle_in_radian, rotAxis);


	transMat_inverse = transMat_.inverse();

	if (trans_cloud_.get() == 0)
		trans_cloud_ = ref_ptr<PointCloud>(new PointCloud);

	pcl::transformPointCloud(decentre_cloud, *trans_cloud_, /*transMat_inverse*/transMat_);

// 	std::string name = "D:/iplTestData/TargetLocalization/target_dataset1/result/filtered/ATT/cloud.pcd";
// 	write_PointCloud(name, *trans_cloud_, true);

	//	Eigen::Vector4f min_pt, max_pt;
	pcl::getMinMax3D(*trans_cloud_, trans_min_pt_, trans_max_pt_);

	bAxisAligned_ = true;

#ifdef _TEST_AAT_
////////////////////////for test
		Eigen::Vector3d  p1, p2;
		p1[0] = decentre_cloud.points[0].x;
		p1[1] = decentre_cloud.points[0].y;
		p1[2] = decentre_cloud.points[0].z;

		p2[0] = trans_cloud_->points[0].x;
		p2[1] = trans_cloud_->points[0].y;
		p2[2] = trans_cloud_->points[0].z;

		//正变换
		Eigen::Vector3d p1c;
		p1c = p1;
// 		p1c[0] -= centroid_[0];
// 		p1c[1] -= centroid_[1];

		Eigen::Vector3d p1t;
		p1t = transMat_inverse*p1c;

		//逆变换
		Eigen::Vector3d p2t;
		p2t = transMat_ * p2;

		Eigen::Vector3d p2c;
		p2c = p2t;
		p2c[0] += centroid_[0];
		p2c[1] += centroid_[1];

#endif
	
	float vx = vsize; 
	float vy = trans_max_pt_[1] - trans_min_pt_[1];

//	assert(vy < 2 * vsize); //检查平面拟合的精度

	float vz = vsize;
//	float varea = vx*vz;

	vsizeX_ = vx; vsizeY_ = vy; vsizeZ_ = vz;

	//if (voxeler_.get() == 0)
	voxeler_.reset(new PointVoxelization<PointT>);  //释放原来的引用计数，添加新的引用计数

	voxeler_->setInputCloud(trans_cloud_);
	voxeler_->setBBox(trans_min_pt_, trans_max_pt_);
	voxeler_->apply(vx, vy, vz);


	return true;
}


template <typename PointT> float
ipl::PlanePointDistribution<PointT>::getSparseDegree()
{
	float total_area, occupied_area;
	float sparse_degree;

	float varea = vsizeX_*vsizeZ_;

	Eigen::Vector3i vscope = voxeler_->getVoxelScope();
	total_area = vscope[0] * vscope[2] * varea;
	
	ipl::VoxelKeyMap* vmap = voxeler_->getVoxelKeyMap();
	occupied_area = vmap->size() * varea;

	sparse_degree = occupied_area / total_area;

	return sparse_degree;
}

template <typename PointT> float
ipl::PlanePointDistribution<PointT>::getBoundRectArea()
{
	float total_area;
	//float sparse_degree;

	float varea = vsizeX_*vsizeZ_;

	Eigen::Vector3i vscope = voxeler_->getVoxelScope();
	total_area = vscope[0] * vscope[2] * varea;

	return total_area;
}

template <typename PointT> float
ipl::PlanePointDistribution<PointT>::getOccupiedArea()
{
	float occupied_area;
	
	float varea = vsizeX_*vsizeZ_;

	ipl::VoxelKeyMap* vmap = voxeler_->getVoxelKeyMap();
	occupied_area = vmap->size() * varea;

	return occupied_area;
}

template <typename PointT> float
ipl::PlanePointDistribution<PointT>::getProjectedLineLength()
{
	float len;

	Eigen::Vector3i vscope = voxeler_->getVoxelScope();
	len = vscope[0] * vsizeX_;

	return len;
}

template <typename PointT> void
ipl::PlanePointDistribution<PointT>::getFurthestPointsOnProjectedLine(iplPOINT3D &ptS, iplPOINT3D &ptE)
{
	Eigen::Vector3i vscope = voxeler_->getVoxelScope();

	Eigen::Vector3d ptTrans, ptOrg;
	ptTrans[0] = trans_min_pt_[0] + 0.5*vsizeX_;
	ptTrans[1] = trans_min_pt_[1] + 0.5*vsizeY_;
	ptTrans[2] = 0.0;

	ptOrg = transMat_*ptTrans;
	ptS.X = ptOrg[0] + centroid_[0];
	ptS.Y = ptOrg[1] + centroid_[1];
	ptS.Z = 0;

	ptTrans[0] = trans_min_pt_[0] + vscope[0]*vsizeX_ + 0.5*vsizeX_;
	ptTrans[1] = trans_min_pt_[1] + 0.5*vsizeY_;
	ptTrans[2] = 0.0;

	ptOrg = transMat_*ptTrans;
	ptE.X = ptOrg[0] + centroid_[0];
	ptE.Y = ptOrg[1] + centroid_[1];
	ptE.Z = 0;

	return;
}

template <typename PointT> float
ipl::PlanePointDistribution<PointT>::getProjectedLineMaxConnectedLength(iplPOINT3D &ptS, iplPOINT3D &ptE)
{
	float maxLen = 0.0;
	ipl::VoxelKeyMap* vmap = voxeler_->getVoxelKeyMap();
	Eigen::Vector3i vscope = voxeler_->getVoxelScope();

	std::vector<int> voxel_hist;
	
	voxel_hist.resize(vscope[0], 0);
	for (VoxelKeyMap::iterator vmIter = vmap->begin();
		vmIter != vmap->end();
		++vmIter)
	{//遍历first pointcloud
		iplOctreeKey key_arg = vmIter->first;

		voxel_hist[key_arg.x]++;
	}

	int i, j;
	int maxStep = 0;
	int iS, iE;
	iS = iE = 0;
	for (i = 0, j = 0; j < voxel_hist.size(); ++j)
	{
		if (voxel_hist[j] == 0)
		{
			if (j - i > maxStep)
			{
				maxStep = j - i;
				iS = i;
				iE = j;
			}

			while (voxel_hist[j] == 0)
				j++;

			i = j;
		}
	}

	maxLen = maxStep * vsizeX_;

	assert(iS < iE);
	Eigen::Vector3d ptTrans, ptOrg;
	ptTrans[0] = trans_min_pt_[0] + iS * vsizeX_ + 0.5*vsizeX_;
	ptTrans[1] = trans_min_pt_[1] + 0.5*vsizeY_;
	ptTrans[2] = 0.0;

	ptOrg = transMat_*ptTrans;
	ptS.X = ptOrg[0] + centroid_[0];
	ptS.Y = ptOrg[1] + centroid_[1];
	ptS.Z = 0;

	ptTrans[0] = trans_min_pt_[0] + iE * vsizeX_ + 0.5*vsizeX_;
	ptTrans[1] = trans_min_pt_[1] + 0.5*vsizeY_;
	ptTrans[2] = 0.0;

	ptOrg = transMat_*ptTrans;
	ptE.X = ptOrg[0] + centroid_[0];
	ptE.Y = ptOrg[1] + centroid_[1];
	ptE.Z = 0;

	return maxLen;
}
