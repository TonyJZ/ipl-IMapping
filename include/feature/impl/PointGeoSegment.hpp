#pragma once

#include "feature/PointGeoSegment.h"
#include "fitting/Line2DFitting_LS.h"
#include "fitting/Line2DFitting_LM.h"

#include <boost/random.hpp>


#include <pcl/common/distances.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/impl/extract_clusters.hpp>

template <typename PointT>
ipl::PointGeoSegment<PointT>::PointGeoSegment() :
	search_()
//	,input_()
{
	//	mt_ = sMT_undefined;
	bbox_seg_.clear();
	bFindFurthestPair = false;
	bFindbbox = false;
	maxDis_ = 0.0;
}

template <typename PointT>
ipl::PointGeoSegment<PointT>::~PointGeoSegment()
{
	bbox_seg_.clear();

	if (search_ != 0)
		search_.reset();

	if (input_ != 0)
		input_.reset();

	if (indices_ != 0)
		indices_.reset();
}

// template <typename PointT> void
// ipl::PointGeoSegment<PointT>::setInputCloud(const PointCloudConstPtr &cloud)
// {
// 	if (input_ == 0)
// 		input_ = boost::shared_ptr<pcl::PointCloud <PointT>>(new pcl::PointCloud <PointT>);
// 
// 	*input_ = *cloud;
// 
// 	bFindFurthestPair = false;
// 	bFindbbox = false;
// 	bbox_seg_.clear();
// 	maxDis_ = 0.0;
// }

template <typename PointT> void
ipl::PointGeoSegment<PointT>::reinitialize()
{
	bFindFurthestPair = false;
	bFindbbox = false;
	bbox_seg_.clear();
	maxDis_ = 0.0;

	initCompute();
}

template <typename PointT> void
ipl::PointGeoSegment<PointT>::setModelCoef(/*ipl::geoModel::geoModelType type, */ipl::geoModel::geoModelInfo coef)
{
//	mt_ = type;
	coef_ = coef;
}

template <typename PointT> void
ipl::PointGeoSegment<PointT>::getBBox(Eigen::Vector3d &bbmin, Eigen::Vector3d &bbmax)
{
	if (bFindbbox)
	{
		bbmin[0] = bbox_seg_[0];
		bbmin[1] = bbox_seg_[1];
		bbmin[2] = bbox_seg_[2];

		bbmax[0] = bbox_seg_[3];
		bbmax[1] = bbox_seg_[4];
		bbmax[2] = bbox_seg_[5];
		return;
	}

	bbox_seg_.resize(6, 0);
	bbox_seg_[0] = bbox_seg_[1] = bbox_seg_[2] = std::numeric_limits<double>::max();
	bbox_seg_[3] = bbox_seg_[4] = bbox_seg_[5] = std::numeric_limits<double>::lowest();

	int nSample;
	if (indices_.get() == 0)
		nSample = input_->size();
	else
		nSample = indices_->size();

	for (int i = 0; i < nSample; i++)
	{
		int id = i;
		if (indices_.get())
			id = indices_->at(i);

		PointT pt = input_->points[id];
		if (pt.x < bbox_seg_[0])
			bbox_seg_[0] = pt.x;
		if (pt.x > bbox_seg_[3])
			bbox_seg_[3] = pt.x;

		if (pt.y < bbox_seg_[1])
			bbox_seg_[1] = pt.y;
		if (pt.y > bbox_seg_[4])
			bbox_seg_[4] = pt.y;

		if (pt.z < bbox_seg_[2])
		{
			bbox_seg_[2] = pt.z;
			low_pt_[0] = pt.x;
			low_pt_[1] = pt.y;
			low_pt_[2] = pt.z;
		}
		if (pt.z > bbox_seg_[5])
		{
			bbox_seg_[5] = pt.z;
			high_pt_[0] = pt.x;
			high_pt_[1] = pt.y;
			high_pt_[2] = pt.z;
		}
	}

	bbmin[0] = bbox_seg_[0];
	bbmin[1] = bbox_seg_[1];
	bbmin[2] = bbox_seg_[2];

	bbmax[0] = bbox_seg_[3];
	bbmax[1] = bbox_seg_[4];
	bbmax[2] = bbox_seg_[5];

	bFindbbox = true;
	return;
}

template <typename PointT> const Eigen::Vector3f
ipl::PointGeoSegment<PointT>::getMeanNormal()
{
	//判断PointT是否有normal field   
	//boost::mpl::if_
	//how to do ?
	mean_normal_ = Eigen::Vector3f::Zero();

	int nSample;
// 	if (indices_.get() == 0)
// 		nSample = input_->size();
// 	else
		nSample = indices_->size();

	for (int i = 0; i < nSample; i++)
	{
		int id = i;
// 		if (indices_.get())
		id = indices_->at(i);

		mean_normal_[0] += input_->points[id].normal_x;
		mean_normal_[1] += input_->points[id].normal_y;
		mean_normal_[2] += input_->points[id].normal_z;
	}

	mean_normal_[0] /= nSample;
	mean_normal_[1] /= nSample;
	mean_normal_[2] /= nSample;

	return mean_normal_;
}

template <typename PointT> double
ipl::PointGeoSegment<PointT>::get3DDiagonal()
{
	Eigen::Vector3d bbmin, bbmax;

	getBBox(bbmin, bbmax);

	double diagonal = sqrt((bbmax[0] - bbmin[0])*(bbmax[0] - bbmin[0])
		+ (bbmax[1] - bbmin[1])*(bbmax[1] - bbmin[1])
		+ (bbmax[2] - bbmin[2])*(bbmax[2] - bbmin[2]));

	return diagonal;
}

template <typename PointT> double
ipl::PointGeoSegment<PointT>::get2DDiagonal()
{
	Eigen::Vector3d bbmin, bbmax;

	getBBox(bbmin, bbmax);

	double diagonal = sqrt((bbmax[0] - bbmin[0])*(bbmax[0] - bbmin[0])
		+ (bbmax[1] - bbmin[1])*(bbmax[1] - bbmin[1]));

	return diagonal;
}

template <typename PointT> double
ipl::PointGeoSegment<PointT>::getProjectionDensity(int nsample, double dc)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2D(new pcl::PointCloud <pcl::PointXYZ>);
	pcl::copyPointCloud(*input_, *indices_, *cloud2D);

	// XYplane projection
	for (int i = 0; i < cloud2D->size(); i++)
	{
		cloud2D->points[i].z = 0;
	}

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
	kdtree->setInputCloud(cloud2D);

	std::vector<int> k_indices;
	std::vector<float> k_sqr_distances;
	boost::mt19937 rng(time(0));
	// 1. uniform_int  
	boost::uniform_int<> ui(0, cloud2D->size() - 1);
	double aPD = 0;
	for (int i = 0; i < nsample; i++)
	{
		int id = ui(rng);

		pcl::PointXYZ pt = cloud2D->points[id];

		kdtree->radiusSearch(pt, dc, k_indices, k_sqr_distances);

		int nn = k_indices.size() - 1; //去掉查找点本身

		assert(nn >= 0);

		aPD += nn;
	}

	aPD /= nsample;

	return aPD;
}

template <typename PointT> double
ipl::PointGeoSegment<PointT>::getFurthestPointsByPCL(PointT &pmin, PointT &pmax)
{
	if (bFindFurthestPair)
	{
		pmin = ptmin_;
		pmax = ptmax_;

		return maxDis_;
	}
	maxDis_ = pcl::getMaxSegment(*input_, *indices_, pmin, pmax);
	pmin = ptmin_;
	pmax = ptmax_;
	bFindFurthestPair = true;

	return maxDis_;
}

template <typename PointT> double
ipl::PointGeoSegment<PointT>::getFurthestPointsApproximate(PointT &pmin, PointT &pmax)
{
	if (bFindFurthestPair)
	{
		pmin = ptmin_;
		pmax = ptmax_;

		return maxDis_;
	}
	Eigen::Vector3d bbmin, bbmax;
	getBBox(bbmin, bbmax);

	if (search_ == 0)
		search_ = boost::shared_ptr<pcl::search::Search<PointT> >(new pcl::search::KdTree<PointT>);

	search_->setInputCloud(input_, indices_);

	std::vector<int> pointIdxNKNSearch(1);
	std::vector<float> pointNKNSquaredDistance(1);

	PointT searchPoint;
	searchPoint.x = bbmin[0];	searchPoint.y = bbmin[1];	searchPoint.z = bbmin[2];
	search_->nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
	ptmin_ = input_->points[pointIdxNKNSearch[0]];

	searchPoint.x = bbmax[0];	searchPoint.y = bbmax[1];	searchPoint.z = bbmax[2];
	search_->nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
	ptmax_ = input_->points[pointIdxNKNSearch[0]];

	maxDis_ = sqrt((ptmax_.x - ptmin_.x)*(ptmax_.x - ptmin_.x)
		+ (ptmax_.y - ptmin_.y)*(ptmax_.y - ptmin_.y)
		+ (ptmax_.z - ptmin_.z)*(ptmax_.z - ptmin_.z));

	pmin = ptmin_;
	pmax = ptmax_;
	bFindFurthestPair = true;

	return maxDis_;
}

template <typename PointT> int
ipl::PointGeoSegment<PointT>::getProjectedLineCeof(iplModelCoeffs *coef)
{
	bool bReady = initCompute();
	if (!bReady)
	{
		deinitCompute();
		return -1;
	}
	

	std::vector<double> x, y;
	int ptNum = indices_->size();
	x.resize(ptNum);
	y.resize(ptNum);
	for (int j = 0; j < ptNum; j++)
	{
		int id = indices_->at(j);
		x[j] = input_->points[id].x;
		y[j] = input_->points[id].y;
	}

	*coef = coef_.coef;
	//fit_line2D_by_LM(&x[0], &y[0], ptNum, coef);
	fit_line2D_by_LS(&x[0], &y[0], ptNum, coef);

	return 0;
}

// template <typename PointT> inline double
// ipl::PointGeoSegment<PointT>::get2DConnectivity(float vSize, double &maxConnDis)
// {
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2D(new pcl::PointCloud <pcl::PointXYZ>);
// 	pcl::copyPointCloud(*input_, *indices_, *cloud2D);
// 
// 	for (int i = 0; i < cloud2D->size(); i++)
// 	{
// 		cloud2D->points[i].z = 0;
// 	}
// 
// 	double voxelSize = vSize;
// 	pcl::octree::OctreePointCloud<pcl::PointXYZ> octree(voxelSize);
// 	octree.setInputCloud(cloud2D);
// 	octree.addPointsFromInputCloud();
// 	pcl::PointCloud<pcl::PointXYZ>::Ptr resampled_cloud2D(new pcl::PointCloud <pcl::PointXYZ>);
// 	octree.getOccupiedVoxelCenters(resampled_cloud2D->points);
// 	resampled_cloud2D->width = resampled_cloud2D->points.size();
// 	resampled_cloud2D->height = 1;
// 
// 	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
// 	tree->setInputCloud(resampled_cloud2D);
// 	std::vector<pcl::PointIndices> cluster_indices;
// 	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
// 	ec.setClusterTolerance(/*0.02*/vSize*1.5); // 2cm
// 	ec.setSearchMethod(tree);
// 	ec.setInputCloud(resampled_cloud2D);
// 	ec.extract(cluster_indices);
// 	int maxCls = 0;
// 	int maxID = -1;
// 	for (int i = 0; i < cluster_indices.size(); i++)
// 	{
// 		if (maxCls < cluster_indices[i].indices.size())
// 		{
// 			maxID = i;
// 			maxCls = cluster_indices[i].indices.size();
// 		}
// 	}
// 	pcl::PointXYZ pmin, pmax;
// 	maxConnDis = pcl::getMaxSegment(*resampled_cloud2D, cluster_indices[maxID].indices, pmin, pmax);
// 
// 	double max_dis = pcl::getMaxSegment(*resampled_cloud2D, pmin, pmax);
// 
// 	return maxConnDis / max_dis;
// }

// template <typename PointT> void
// ipl::PointGeoSegment<PointT>::getProjectionLineDirection(PointT orgPt, Eigen::Vector2f &normal)
// {
// 	Eigen::Vector2f t_normal;
// 
// 	//initial direction
// 	normal[0] = -coef_.coef.values[1];
// 	normal[1] = coef_.coef.values[0];
// 	//	normal[2] = 1.0;
// 
// 	PointT pmin, pmax;
// 	getFurthestPointsApproximate(pmin, pmax);
// 
// 	float dis2 = (pmin.x - orgPt.x)*(pmin.x - orgPt.x) + (pmin.y - orgPt.y)*(pmin.y - orgPt.y);
// 
// 	//找出长边方向
// 	if (dis2 < (pmax.x - orgPt.x)*(pmax.x - orgPt.x) + (pmax.y - orgPt.y)*(pmax.y - orgPt.y))
// 	{
// 		t_normal[0] = pmax.x - orgPt.x;
// 		t_normal[1] = pmax.y - orgPt.y;
// 	}
// 	else
// 	{
// 		t_normal[0] = pmin.x - orgPt.x;
// 		t_normal[1] = pmin.y - orgPt.y;
// 	}
// 
// 	//利用点积判断方向
// 	if (normal[0] * t_normal[0] + normal[1] * t_normal[1] < 0)
// 	{
// 		normal[0] *= -1;
// 		normal[1] *= -1;
// 	}
// 
// 	return;
// }

///////////////////distance to model
//line:    fabs(A*x0+B*y0+C)/sqrt(A^2+B^2)
//plane:   fabs(A*x0+B*y0+C*z0+D)/sqrt(A^2+B^2+C^2)
// template <typename PointT> double
// ipl::PointGeoSegment<PointT>::distance2Model(std::vector<double> &distances)
// {
// 	double rms = 0;
// 	double vmod;
// 	//double x, y, z;
// 	if (mt_ == gMT_LINE)
// 	{
// 		vmod = sqrt(coef_.values[0]*coef_.values[0]+coef_.values[1]*coef_.values[1]);
// 	}
// 	else if (mt_ == gMT_PLANE)
// 	{
// 		vmod = sqrt(coef_.values[0] * coef_.values[0] + coef_.values[1] * coef_.values[1]+coef_.values[2]*coef_.values[2]);
// 	}
// 
// 	distances.clear();
// 	for (int i = 0; i < indices_->size(); i++)
// 	{
// 		int id = indices_->at(i);
// 		double dis;
// 		
// 		if (mt_ == gMT_LINE)
// 			dis = fabs(input_->points[id].x*coef_.values[0] + input_->points[id].y*coef_.values[1] + coef_.values[2])
// 			/ vmod;
// 		else if (mt_ == gMT_PLANE)
// 			dis = fabs(input_->points[id].x*coef_.values[0] + input_->points[id].y*coef_.values[1] + input_->points[id].z*coef_.values[2] + coef_.values[3])
// 			/ vmod;
// 
// 		rms += dis*dis;
// 		distances.push_back(dis);
// 	}
// 
// 	if (distances.size() == 0)
// 		rms = 0;
// 	else
// 		rms = sqrt(rms / distances.size());
// 
// 	return rms;
// }

