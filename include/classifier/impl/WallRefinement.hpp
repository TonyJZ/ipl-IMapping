#include "classifier/WallRefinement.h"
#include "commonAPIs/iplutility.h"
//#include "io/iplio.h"
#include "spatialindexing/ClusterOverlapping.h"
#include "feature/PlanePointDistribution.h"

#include "fitting/ModelParamsIO.h"
#include "fitting/PlaneFitting.h"
#include "io/PointcloudIO.h"

#include <Eigen/dense>

//cgal
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/intersections.h>
#include <CGAL/number_utils.h>

//pcl
#include <pcl/common/angles.h>
#include <pcl/features/normal_3d.h>
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>

namespace ipl
{
template<typename PointT> 
float getConsecutiveHeightScope(const iplPointCloud<PointT> &cloud, const std::vector<int> *indices/* =NULL */, const float hTh /* = 0.025 */)
{
	ref_ptr<std::vector<int> >  selected_indices(new std::vector<int>);

	if (!indices)
	{
		selected_indices->resize(cloud.points.size());
		for (size_t i = 0; i < selected_indices->size(); ++i) { (*selected_indices)[i] = static_cast<int>(i); }
	}
	else
	{
		selected_indices->insert(selected_indices->end(), indices->begin(), indices->end());
	}

	float hei_interval = 0.05;

	float min_h, max_h;

	Eigen::Vector4f min_pt, max_pt;
	pcl::getMinMax3D(cloud, *selected_indices, min_pt, max_pt);

	min_h = min_pt[2] - 0.5*hei_interval;
	max_h = max_pt[2] + 0.5*hei_interval;

	int nstep = ceil((max_h - min_h) / hei_interval);

	std::vector<int> zHist;
	std::vector<int> zCumulativeHist;
	zHist.resize(nstep + 2, 0);
	zCumulativeHist.resize(nstep + 2, 0);

	int npts = selected_indices->size();
	for (int i = 0; i < selected_indices->size(); i++)
	{
		int id = selected_indices->at(i);
		double z = cloud.points[id].z;

		int iStep = static_cast<int> (floor((z - min_h) / hei_interval));
		zHist[iStep]++;
	}

	zCumulativeHist[0] = zHist[0];
	for (int i = 1; i < zHist.size(); i++)
	{
		zCumulativeHist[i] = zCumulativeHist[i - 1] + zHist[i];
	}

	bool bfind_zi = false, bfind_zj = false;
	int zi, zj;
	zi = zj = 0;
	for (int i = 0; i < zCumulativeHist.size(); i++)
	{
		float ratio = zCumulativeHist[i];
		ratio /= npts;

		if (!bfind_zi)
		{
			if (ratio > hTh)
			{
				zi = i;
				bfind_zi = true;
			}
		}

		if (!bfind_zj)
		{
			if (ratio > 1.0 - hTh)
			{
				zj = i;
				bfind_zj = true;
			}
		}
	}

	float heiScope = (zj - zi)*hei_interval;
	return heiScope;
}

}


template <typename PointT>
ipl::WallRefinement<PointT>::WallRefinement(float sampling_interval/*, float vsize*/) :
	s_interval_(sampling_interval)
	//, vsize_(vsize)
{

}

template <typename PointT>
ipl::WallRefinement<PointT>::WallRefinement() : 
	s_interval_(0.02)
	//, vsize_(0.2)
{

}

template <typename PointT>
ipl::WallRefinement<PointT>::~WallRefinement()
{
	cloud_.reset();
	connectivity_.reset();
	segments_.clear();
	p2cLut_.clear();
	seg_indices_.clear();
}

template <typename PointT> void
ipl::WallRefinement<PointT>::setHeightScope(float floorHei, float ceilingHei)
{
	floor_hei_ = floorHei;
	ceiling_hei_ = ceilingHei;
}

template <typename PointT> int
ipl::WallRefinement<PointT>::loadCandidateWalls(const std::string &dir, const std::string &ext)
{
	pcl::PointCloud<PointT>::Ptr seg_cloud(new pcl::PointCloud <PointT>);

	p2cLut_.clear();
	cloud_.reset();
	seg_indices_.clear();
	cloud_ = pcl::PointCloud<PointT>::Ptr (new pcl::PointCloud <PointT>);

	std::vector<std::string> filenames;
	ipl::scan_files(dir, ext, filenames);

	int nSegs = filenames.size();
	if (nSegs <= 0)
	{
		std::cout << "can't find " << ext << " in " << dir << std::endl;
		return (-1);
	}

	std::cout << "begin to load candidate walls: " << nSegs << std::endl;
	seg_indices_.resize(nSegs);
	int curptId = 0;
	for (int i = 0; i < nSegs; i++)
	{
		std::cout << "loading: " << i << "\r";
		//提取分割点云和模型参数
		std::string rname, sname;

		ipl::extract_file_name(filenames[i], rname, sname);
		std::string paraname = rname;
		paraname += /*".param"*/geoModel::ModelParamFile_Suffix;

		ipl::geoModel::geoModelInfo info;

//		ipl::iplModelCoeffs mCoef;
		ipl::geoModel::geoModelType type;

		double bbmin[3], bbmax[3];
		load_geoModel_params(paraname, /*bbmin, bbmax,*/ info);

//		mCoef = info.coef;

		//提取分割点云
		read_PointCloud(filenames[i], *seg_cloud);

		seg_indices_[i] = IndicesPtr(new std::vector<int>);
		for (int j = 0; j < seg_cloud->size(); j++)
		{//合并点云并记录每个分割的索引
			cloud_->points.push_back(seg_cloud->points[j]);

			p2cLut_.push_back(i);

			seg_indices_[i]->push_back(curptId);
			curptId++;
		}

		ipl::PointGeoSegment<PointT> seg;

		//seg.setInputCloud(seg_cloud);

		seg.setModelCoef(info);

		segments_.push_back(seg);
	}
	std::cout << "loading completed" << std::endl;

	cloud_->height = 1;
	cloud_->width = cloud_->size();

	//初始化segments
	for (int i = 0; i < nSegs; i++)
	{
		segments_[i].setInputCloud(cloud_);
		segments_[i].setIndices(seg_indices_[i]);
	}

	//初始化segment 类型
	walls_type_.resize(segments_.size());
	for (int i = 0; i < walls_type_.size(); i++)
	{
		walls_type_[i] = ipl::bim::WT_UNKNOWN;
	}

	//for test
// 	for (int i = 0; i < segments_.size(); i++)
// 	{
// 		ipl::iplModelCoeffs coef = segments_[i].getModelCoef();
// 
// 
// 	}

	return (0);
}

template <typename PointT> int
ipl::WallRefinement<PointT>::removeUncertainWalls(float areaTh, float sparseTh, float rmsTh, float maxGirderTh)
{
	double dc = /*0.1*/ 5*s_interval_;
	double connTh = 10.0*s_interval_;
	double lowrhoTh, highrhoTh;  //用来过滤低密度点(非稳定的墙面)
// 	double vSize = 0.02; //原始数据的voxel size
// 	double lowZTh = 0.5, highZTh = 2.0; //可接受的最小墙面高

// 	lowrhoTh = 2 * dc*maxGirderTh / (s_interval_*s_interval_);
// 	highrhoTh = 2 * dc*minWallTh / (s_interval_*s_interval_);

	/*printf("plane filtering: \n");*/

	
	int nSegs = segments_.size();
	std::cout << "begin to analyze candidate walls: " << nSegs << std::endl;
	int accepted = 0;
	for (int i = 0; i < nSegs; i++)
	{
		std::cout << "process: " << i << " wall\r";

		Eigen::Vector3d bbmin, bbmax;
		segments_[i].getBBox(bbmin, bbmax);

		PlanePointDistribution<PointT> dwall;
		dwall.setInputCloud(segments_[i].getInputCloud());
		dwall.setIndices(segments_[i].getIndices());
		dwall.setPlaneCoef(segments_[i].getModelCoef());

		std::vector<double> distances;

		double rms = dwall.distance2plane(distances);

		if (rms > rmsTh)
		{
			walls_type_[i] = ipl::bim::WT_NONWALL;
			tracking_list_[i] = NONWALL;
			continue;
		}

		dwall.reinitializeAAT(/*float vsize = 0.2*/);

// 		float floorhei, ceilinghei;
// 		dwall.getHeight(floorhei, ceilinghei);

		float hei_scope;
//		dwall.getConsecutiveHeightScope(hei_scope);
		hei_scope = getConsecutiveHeightScope(*(segments_[i].getInputCloud()), segments_[i].getIndices().get());
		
		float sparse_deg = dwall.getSparseDegree();
		float occupied_area = dwall.getOccupiedArea();
		

// 		double area = sqrt((bbmin[0] - bbmax[0])*(bbmin[0] - bbmax[0]) + (bbmin[1] - bbmax[1])*(bbmin[1] - bbmax[1]))
// 			*(bbmax[2] - bbmin[2]);

		if (occupied_area < areaTh)
		{
			walls_type_[i] = ipl::bim::WT_NONWALL;
			tracking_list_[i] = NONWALL;
			continue;
		}

		if (/*ceilinghei - floorhei*/hei_scope < maxGirderTh)
		{
			walls_type_[i] = ipl::bim::WT_NONWALL;
			tracking_list_[i] = NONWALL;
			continue;
		}

		if (sparse_deg < sparseTh)
		{
			walls_type_[i] = ipl::bim::WT_NONWALL;
			tracking_list_[i] = NONWALL;
			continue;
		}

// 		double diagLen = segments_[i].get2DDiagonal();
// 		int nsample = 20;
// 		double dens = segments_[i].getProjectionDensity(nsample, dc);
// 
// 		if (diagLen < /*2.0*/lenTh)
// 		{
// 			walls_type_[i] = ipl::bm::WT_NONWALL;
// 			tracking_list_[i] = NONWALL;
// 			continue;
// 		}
// 			
// 
// 		if (diagLen < /*3.0*/1.5*lenTh && dens < lowrhoTh)
// 		{
// 			walls_type_[i] = ipl::bm::WT_NONWALL;
// 			tracking_list_[i] = NONWALL;
// 			continue;
// 		}
// 			
// 		double maxConnDis;
// 		double connp = segments_[i].get2DConnectivity(/*0.2*/connTh, maxConnDis);
// 		if (connp < 0.5 && maxConnDis < /*2.0*/lenTh)
// 		{
// 			walls_type_[i] = ipl::bm::WT_NONWALL;
// 			tracking_list_[i] = NONWALL;
// 			continue;
// 		}
		accepted++;
	}

	std::cout << "complete 100%. accepted walls: " << accepted << std::endl;
	return (0);
}

template <typename PointT> int
ipl::WallRefinement<PointT>::mergeWalls(float vsize)
{
	vsize_ = vsize;

	//建立voxel connectivity
	IndicesPtr accepted_indices = IndicesPtr(new std::vector<int>);

	int nSegs = segments_.size();
	int nAccepted = 0;
	for (int i = 0; i < nSegs; i++)
	{
		if (walls_type_[i] == ipl::bim::WT_NONWALL)
		{
			continue;
//			tracking_list_[i] = NONWALL;
		}
		
		nAccepted++;
		accepted_indices->insert(accepted_indices->end(), seg_indices_[i]->begin(), seg_indices_[i]->end());
	}
	std::cout << "candidate " << nAccepted << " wall merging ... " << std::endl;

	std::cout << "build Cluster Connectivity... ";
	//仅对接受的墙面建立连通关系
	connectivity_.reset(new ipl::ClusterConnectivityVoxelNeighbourhood<PointT>(vsize_, &p2cLut_));
//	connectivity_ = boost::shared_ptr<ipl::ClusterConnectivityVoxelNeighbourhood<PointT> >(new ipl::ClusterConnectivityVoxelNeighbourhood<PointT>(vsize_, &p2cLut_));
	connectivity_->setInputCloud(cloud_);
	connectivity_->setIndices(accepted_indices);

	connectivity_->apply();
	ClusterConnectivityVoxelNeighbourhood<PointT>::CCGraph* adjSegs = connectivity_->getConnectedGraph();
	std::cout << "done!" << std::endl;

	typedef CGAL::Exact_predicates_exact_constructions_kernel K;
	typedef K::Point_2 Point_2;
	typedef K::Segment_2 Segment_2;
	typedef K::Line_2 Line_2;
	typedef K::Intersect_2 Intersect_2;

// 	typedef K::Line_3	Line_3;
// 	typedef K::Plane_3	Plane_3;

	boost::shared_ptr<ClusterOverlapping<PointT> > overlapping_detector = boost::shared_ptr<ClusterOverlapping<PointT> >(new ClusterOverlapping<PointT>);
	ClusterConnectivityVoxelNeighbourhood<PointT>::VertexIterator vi, vi_end;
	int nvertices = 0;

	std::cout << "traverse wall cluster...  " << std::endl;
	float includedTh = cosf(pcl::deg2rad(15.0));  //夹角阈值15度·
	for (boost::tie(vi, vi_end) = boost::vertices(*adjSegs); vi != vi_end; ++vi)
	{//遍历顶点
		nvertices++;
		std::cout << "wall: " << nvertices << "\r";

		ClusterConnectivityVoxelNeighbourhood<PointT>::VertexDescriptor vd = *vi;

		ClusterConnectivityVoxelNeighbourhood<PointT>::AdjacencyIterator ai, ai_end;
		boost::tie(ai, ai_end) = boost::adjacent_vertices(vd, *adjSegs);
		for (; ai != ai_end; ++ai)
		{//遍历邻接节点
			ClusterConnectivityVoxelNeighbourhood<PointT>::VertexDescriptor vt = *ai;

			if (vd < vt) //避免重复遍历邻接对象
			{//判断邻接平面是否是同一面墙
	 			
				//计算两个平面方程的夹角
				ipl::iplModelCoeffs coef1 = segments_[vd].getModelCoef().coef;
				ipl::iplModelCoeffs coef2 = segments_[vt].getModelCoef().coef;

				Eigen::Vector3f normal1, normal2;
				float dot_product;

				normal1[0] = coef1.values[0]; normal1[1] = coef1.values[1]; normal1[2] = coef1.values[2];
				normal2[0] = coef2.values[0]; normal2[1] = coef2.values[1]; normal2[2] = coef2.values[2];
				dot_product = normal1.dot(normal2);

				if (fabsf(dot_product) < includedTh)
				{//平面间夹角大于阈值
					continue;
				}

				boost::shared_ptr<const iplPointCloud<PointT> >  cloud1, cloud2;
				boost::shared_ptr <std::vector<int> >  indice1, indice2;

				cloud1 = segments_[vd].getInputCloud();
				cloud2 = segments_[vt].getInputCloud();

				indice1 = segments_[vd].getIndices();
				indice2 = segments_[vt].getIndices();
				
				overlapping_detector->setFirstCluster(cloud1, indice1);
				overlapping_detector->setSecondCluster(cloud2, indice2);

				overlapping_detector->apply(vsize_);

				int v1_num, v2_num, o_num;
				overlapping_detector->getOverlappingVoxelNum(v1_num, v2_num, o_num);  //occupied voxels

				float overlap_deg1, overlap_deg2;

				overlap_deg1 = float(o_num) / v1_num;
				overlap_deg2 = float(o_num) / v2_num;

				if (overlap_deg1 > 0.8 || overlap_deg2 > 0.8)
				{
					normal1 = segments_[vd].getMeanNormal();
					normal2 = segments_[vt].getMeanNormal();
					dot_product = normal1.dot(normal2);

					if (dot_product > 0)
					{
						walls_type_[vd] = bim::WT_ONESIDE;
						walls_type_[vt] = bim::WT_ONESIDE;
					}
					else
					{
						walls_type_[vd] = bim::WT_DOUBLESIDE;
						walls_type_[vt] = bim::WT_DOUBLESIDE;
					}

					int sCID = vd;
					while (tracking_list_[sCID] > ISOLATED)
					{
						sCID = tracking_list_[sCID];
					}

					tracking_list_[vt] = sCID;

					merged_indices_[sCID].push_back(vt);
					std::cout << "merge: (" << vd << ", " << vt << ")" << std::endl;
				}
			}

		}
	}

	return (0);
}

template <typename PointT> void
ipl::WallRefinement<PointT>::reinitialize()
{
	int nSegs = segments_.size();
	walls_type_.resize(nSegs, bim::WT_UNKNOWN);
	tracking_list_.resize(nSegs, ISOLATED);   //isolated seg. marked -1

	merged_indices_.resize(nSegs);
	for (int i = 0; i < nSegs; i++)
	{
		std::vector<int> idx;
		idx.push_back(i);
		merged_indices_[i] = idx;
	}
}

template <typename PointT> int
ipl::WallRefinement<PointT>::exportAcceptedWalls(const std::string &dir, const std::string &ext)
{
	ipl::create_folder(dir);
	
	std::cout << "export accepted walls..." << std::endl;
	int curID = 0;
	for (int i = 0; i < segments_.size(); i++)
	{
		if (walls_type_[i] == ipl::bim::WT_NONWALL)
			continue;
		
		if (tracking_list_[i] != ISOLATED)
			continue;

		//accept
		std::string out_name, out_param;
		char buf[32], buf_para[32];
		sprintf(buf, "wall_refined_%04d", curID);
		sprintf(buf_para, "wall_refined_%04d%s", curID, geoModel::ModelParamFile_Suffix);


		out_name = /*pOutDir*/dir;
		out_name += "/";
		//out_name += result_name;
		out_name += buf;
		out_name += ext;

		out_param = /*pOutDir*/dir;
		out_param += "/";
		//out_param += result_name;
		out_param += buf_para;

		boost::shared_ptr<PointCloud> merged_cloud = boost::shared_ptr<PointCloud>(new PointCloud);
		std::vector<double> x, y, z;

		for (int j = 0; j < merged_indices_[i].size(); j++)
		{//合并segs
			int cls_ID = merged_indices_[i].at(j);

			boost::shared_ptr<const PointCloud> seg_cloud = segments_[cls_ID].getInputCloud();
			
			for (int ipt = 0; ipt < seg_indices_[cls_ID]->size(); ipt++)
			{
				int id = seg_indices_[cls_ID]->at(ipt);

				merged_cloud->points.push_back(seg_cloud->points[id]);
				x.push_back(seg_cloud->points[id].x);
				y.push_back(seg_cloud->points[id].y);
				z.push_back(seg_cloud->points[id].z);
			}
// 			merged_cloud->points.insert(merged_cloud->points.end(),
// 				seg_cloud->points.begin(), seg_cloud->points.end());
		}

		merged_cloud->height = 1;
		merged_cloud->width = merged_cloud->size();

		
		ipl::geoModel::geoModelInfo coef;
		//ipl::geoModel::geoModelType mtype;
		if (merged_indices_[i].size() > 1)
		{//计算合并后点云的平面参数

			fit_plane_by_LS(&x[0], &y[0], &z[0], merged_cloud->size(), coef);

			//note: 利用协方差的计算方法，当分布不均匀，特别是有较大间隙时，拟合的平面参数会失败
			// Use Least-Squares to fit the plane through all the given sample points and find out its coefficients
// 			EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
// 			Eigen::Vector4f xyz_centroid;
// 			Eigen::Vector4f plane_parameters;
// 			
// 			pcl::computeMeanAndCovarianceMatrix(*merged_cloud, covariance_matrix, xyz_centroid);
// 
// 			// Compute the model coefficients
// 			EIGEN_ALIGN16 Eigen::Vector3f::Scalar eigen_value;
// 			EIGEN_ALIGN16 Eigen::Vector3f eigen_vector;
// 			pcl::eigen33(covariance_matrix, eigen_value, eigen_vector);
// 
// 			// Hessian form (D = nc . p_plane (centroid here) + p)
// 			plane_parameters[0] = eigen_vector[0];
// 			plane_parameters[1] = eigen_vector[1];
// 			plane_parameters[2] = eigen_vector[2];
// 			plane_parameters[3] = 0;
// 			plane_parameters[3] = -1 * plane_parameters.dot(xyz_centroid);
// 
// 			mtype = geoModel::gMT_PLANE;
// 			coef.values.resize(4);
// 			coef.values[0] = plane_parameters[0];
// 			coef.values[1] = plane_parameters[1];
// 			coef.values[2] = plane_parameters[2];
// 			coef.values[3] = plane_parameters[3];

		}
		else
		{
			int cls_ID = merged_indices_[i].at(0);
			coef = segments_[cls_ID].getModelCoef();
			//mtype = segments_[cls_ID].getModelType();
		}

		std::cout << "plane " << i << "\r";

		write_PointCloud(out_name, *merged_cloud, true);
		Eigen::Vector4f bbmin, bbmax;
		pcl::getMinMax3D(*merged_cloud, bbmin, bbmax);
		double min_pt[3], max_pt[3];
		min_pt[0] = bbmin[0]; min_pt[1] = bbmin[1]; min_pt[2] = bbmin[2];
		max_pt[0] = bbmax[0]; max_pt[1] = bbmax[1]; max_pt[2] = bbmax[2];
		memcpy(coef.bbox.min_pt, min_pt, sizeof(double) * 3);
		memcpy(coef.bbox.max_pt, max_pt, sizeof(double) * 3);
		save_geoModel_params(out_param.c_str(), /*min_pt, max_pt,*/ coef);
		
		curID++;
	}

	std::cout << "export completed. total walls: " << curID << std::endl;

	return (0);
}

