#pragma once
#include "feature/IntersectionPoint2DExtractor.h"
#include "feature/PlanePointDistribution.h"

#include "spatialindexing/ClusterConnectivityVoxelNeighbourhood.h"

#include "commonAPIs/iplutility.h"
#include "fitting/ModelParamsIO.h"
#include "io/PointcloudIO.h"

//eigen
#include <Eigen/Dense>

//cgal
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/intersections.h>
#include <CGAL/number_utils.h>

template <typename PointT>
int ipl::IntersectionPoint2DExtractor<PointT>::extract()
{
// 	char *wall_dir;
// 	char *file_ext; 
// 	std::vector<std::string> &filenames;
	

	//std::vector<iglIntersectionPoint> &ipts;

	//typedef pcl::PointXYZRGBNormal PointT;

	//TraversFolderForCertainFiles(wall_dir, file_ext, filenames);

	//pcl::PCDReader pcdReader;
	pcl::PointCloud<PointT>::Ptr cloud_in_all(new pcl::PointCloud <PointT>); //合并所有seg点云
																			 //	std::vector<pcl::ModelCoefficients> planeCoeffs;
	std::vector<std::vector<int>> segGroup_indices;
	int nSegs = filenames_.size();
	if (nSegs)
		segGroup_indices.resize(nSegs);

	//segNames = filenames;

	//std::vector<ptSegment<PointT>>  segments;

	segments_.clear();

	std::vector<ipl::feature::ProjectedLineAttribute>  planeAttList;

	std::vector<int>   p2cLut;  //点云对应的聚类号
	int curptId = 0;
	//std::cout << "loading walls... ";
	for (int i = 0; i < nSegs; i++)
	{
		std::cout << "loading walls... " << i + 1 << " / " << nSegs << "\r";
		//提取分割点云和模型参数
		std::string rname, sname;

		ipl::extract_file_name(filenames_[i], rname, sname);
		std::string paraname = rname;
		paraname += /*".param"*/geoModel::ModelParamFile_Suffix;

		ipl::geoModel::geoModelInfo mCoef;
		//ipl::geoModel::geoModelType type;

		double bbmin[3], bbmax[3];
		ipl::load_geoModel_params(paraname, /*bbmin, bbmax,*/ mCoef);
		
		//提取分割点云
		pcl::PointCloud<PointT>::Ptr seg_cloud(new pcl::PointCloud <PointT>);
		ipl::read_PointCloud(filenames_[i], *seg_cloud);
		
		for (int j = 0; j < seg_cloud->size(); j++)
		{//合并点云并记录每个分割的索引
			cloud_in_all->points.push_back(seg_cloud->points[j]);

			p2cLut.push_back(i);

			segGroup_indices[i].push_back(curptId);
			curptId++;
		}

		ipl::PointGeoSegment<PointT> seg;

		seg.setInputCloud(seg_cloud);

		seg.setModelCoef(mCoef);

		segments_.push_back(seg);
	}

	std::cout << std::endl;

	//	segments[0].get2DConnectivity(1.0);

	cloud_in_all->height = 1;
	cloud_in_all->width = cloud_in_all->size();

	//建立voxel connectivity
	float vsize = 1.0;
	ref_ptr<ipl::ClusterConnectivityVoxelNeighbourhood<PointT> > voxelConn(new ipl::ClusterConnectivityVoxelNeighbourhood<PointT>(vsize, &p2cLut));

	voxelConn->setInputCloud(cloud_in_all);
//	voxelConn->setPointClusterLut(&p2cLut);

	// 	Eigen::Vector3f bbmin, bbmax;
	// 	voxelConn->getBBox(bbmin, bbmax);
	// 
	// 	bbmin[0] -= (bbmax[0] - bbmin[0])*0.5;
	// 	bbmin[1] -= (bbmax[1] - bbmin[1])*0.5;
	// 	bbmax[0] += (bbmax[0] - bbmin[0])*0.5;
	// 	bbmax[1] += (bbmax[1] - bbmin[1])*0.5;

	voxelConn->apply();

	ClusterConnectivityVoxelNeighbourhood<PointT>::CCGraph* adjSegs = voxelConn->getConnectedGraph();

	//找到相邻的segments并计算交点
	typedef CGAL::Exact_predicates_exact_constructions_kernel K;
	typedef K::Point_2 Point_2;
	typedef K::Segment_2 Segment_2;
	typedef K::Line_2 Line_2;
	typedef K::Intersect_2 Intersect_2;

	typedef K::Line_3	Line_3;
	typedef K::Plane_3	Plane_3;

	//std::vector<iglIntersectionPoint>  ipts;

	features_.reset();
	features_ = boost::shared_ptr< IntersectionPoint2DSet >(new IntersectionPoint2DSet(/*cluster_amount_*/));
	//ipts.clear();

	std::cout << "begin to extract intersection points... " << std::endl;
	ClusterConnectivityVoxelNeighbourhood<PointT>::VertexIterator vi, vi_end;
	int nvertices = 0;
	int nIPs = 0;
	for (boost::tie(vi, vi_end) = boost::vertices(*adjSegs); vi != vi_end; ++vi)
	{//遍历顶点
		nvertices++;

		ClusterConnectivityVoxelNeighbourhood<PointT>::VertexDescriptor vd = *vi;

		ClusterConnectivityVoxelNeighbourhood<PointT>::AdjacencyIterator ai, ai_end;
		boost::tie(ai, ai_end) = boost::adjacent_vertices(vd, *adjSegs);
		for (; ai != ai_end; ++ai)
		{//遍历邻接节点
			ClusterConnectivityVoxelNeighbourhood<PointT>::VertexDescriptor vt = *ai;

			if (vd < vt)
			{//计算交点

				std::cout << "connected: (" << vd << ", " << vt << ")" << std::endl;

				ipl::iplModelCoeffs vdCeof, vtCeof;

				vdCeof = segments_[vd].getModelCoef().coef;
				vtCeof = segments_[vt].getModelCoef().coef;

				Line_2 lin1(vdCeof.values[0], vdCeof.values[1], vdCeof.values[3]);
				Line_2 lin2(vtCeof.values[0], vtCeof.values[1], vtCeof.values[3]);

				CGAL::cpp11::result_of<Intersect_2(Line_2, Line_2)>::type
					result = intersection(lin1, lin2);
				if (result)
				{
					if (const Line_2* l = boost::get<Line_2>(&*result))
					{
						std::cout << "line:" << *l << std::endl;
					}
					else
					{
						const Point_2* p = boost::get<Point_2 >(&*result);
						std::cout << "point:" << *p << std::endl;

						PointT pt;
						pt.x = to_double(p->x());
						pt.y = to_double(p->y());
						pt.z = PROJECTED2D_Z;

						bool bFindvd = voxelConn->isClusterInVoxel2D(vd, pt);
						bool bFindvt = voxelConn->isClusterInVoxel2D(vt, pt);

						if (bFindvd || bFindvt)
						{//交点所在的voxel中包含目标分割

							//iglIntersectionPoint ipt;
							ipl::POSITION ipt;
							ipt.resize(4);
							ipt[0] = pt.x;
							ipt[1] = pt.y;
							ipt[2] = PROJECTED2D_Z;
							ipt[3] = 1.0;

							ipl::edgeINDICE  connIndices;
							connIndices.push_back(vd);
							connIndices.push_back(vt);

							//计算交点的权重
							PointT vd_pmin, vd_pmax, vt_pmin, vt_pmax;
							double vd_dis_2D, vt_dis_2D;

							segments_[vd].getFurthestPointsApproximate(vd_pmin, vd_pmax);
							segments_[vt].getFurthestPointsApproximate(vt_pmin, vt_pmax);

							vd_dis_2D = std::sqrt((vd_pmax.x - vd_pmin.x)*(vd_pmax.x - vd_pmin.x)
								+ (vd_pmax.y - vd_pmin.y)*(vd_pmax.y - vd_pmin.y));
							vt_dis_2D = std::sqrt((vt_pmax.x - vt_pmin.x)*(vt_pmax.x - vt_pmin.x)
								+ (vt_pmax.y - vt_pmin.y)*(vt_pmax.y - vt_pmin.y));

							//maxEdge = 1.0;

							Eigen::Vector2f vd_line_normal, vt_line_normal;
							vd_line_normal[0] = -vdCeof.values[1];
							vd_line_normal[1] = vdCeof.values[0];

							vt_line_normal[0] = -vtCeof.values[1];
							vt_line_normal[1] = vtCeof.values[0];

							float cos_theta = vd_line_normal.dot(vt_line_normal);
							float sin_theta = sqrt(1.0 - cos_theta*cos_theta);

							DESCRIPTOR weight;

							if (vd_dis_2D < vt_dis_2D)
								std::swap(vd_dis_2D, vt_dis_2D);
							weight.push_back(0.5*(vd_dis_2D + vt_dis_2D*sin_theta));

							features_->addOnePosition(ipt);
							features_->addOneDescriptor(weight);
							features_->addEdgeIndice(connIndices);
							//ipts.push_back(ipt);

							std::vector<double> var;
							EstimatorVariance(vd, vt, var);
							features_->addIntersectionVariance(var);
							
							nIPs++;

							//std::cout << "find " << nIPs << " intersection \r";
						}

					}
				}
			}

		}
	}

	std::cout << "find " << nIPs << " intersections." <<std::endl;

	//统计连接边的属性
	std::vector<ipl::feature::ProjectedLineAttribute>  attList;
	for (size_t i = 0; i < segments_.size(); ++i)
	{
		std::cout << "estimate the attributes of walls... " << i+1 << " / " << segments_.size() << "\r";
		PlanePointDistribution<PointT> dwall;

		dwall.setInputCloud(segments_[i].getInputCloud());
		dwall.setIndices(segments_[i].getIndices());
		dwall.setPlaneCoef(segments_[i].getModelCoef());

		dwall.reinitializeAAT();
		iplPOINT3D ptS, ptE;
		dwall.getFurthestPointsOnProjectedLine(ptS, ptE);

		ipl::feature::ProjectedLineAttribute att;
		att.ptS[0] = ptS.X; att.ptS[1] = ptS.Y; att.ptS[2] = ptS.Z;
		att.ptE[0] = ptE.X; att.ptE[1] = ptE.Y; att.ptE[2] = ptE.Z;

		attList.push_back(att);
	}

	std::vector<ipl::feature::edgeINDICE> conne_indices = features_->getEdgeIndices();
	
	//size_t nIPs = features_->getPositions().size();
	assert(nIPs == features_->getPositions().size());

	for (size_t i = 0; i < nIPs; ++i)
	{
		ipl::feature::edgeINDICE   edgeIndice = conne_indices[i];
		ipl::feature::edgeATTRIBUTES edgeAtts;
		for (size_t j = 0; j < edgeIndice.size(); ++j)
		{
			int edgeID = edgeIndice[j];

			edgeAtts.push_back(attList[edgeID]);
		}
		features_->addEdgeAtt(edgeAtts);
	}

	return (0);
}

template <typename PointT>
int ipl::IntersectionPoint2DExtractor<PointT>::EstimatorVariance(int i, int j, std::vector<double> &intersectionVar)
{
	ipl::geoModel::geoModelInfo planeInfo_i, planeInfo_j;

	if (i > j)
		std::swap(i, j);

	if (i < 0 || j >= segments_.size())
		return -1;

	planeInfo_i = segments_[i].getModelCoef();
	planeInfo_j = segments_[j].getModelCoef();

	double a1, b1, a2, b2;
	a1 = planeInfo_i.coef.values[0];
	b1 = planeInfo_i.coef.values[1];

	a2 = planeInfo_j.coef.values[0];
	b2 = planeInfo_j.coef.values[1];


	Eigen::MatrixXd B(2, 2);
	B(0, 0) = a1; B(0, 1) = b1;
	B(1, 0) = a2; B(1, 1) = b2;

	double sigma1 = planeInfo_i.obsSD;
	sigma1 *= sigma1;
	double sigma2 = planeInfo_j.obsSD;
	sigma2 *= sigma2;

//	Eigen::MatrixXd BTB(2, 2);
	Eigen::MatrixXd D(2, 2);

// 	BTB = B.transpose()*B;
// 	D = BTB.inverse();
// 
// 	intersectionVar.resize(2);
// 	intersectionVar[0] = D(0, 0) / sigma1;
// 	intersectionVar[1] = D(1, 1) / sigma2;


	Eigen::MatrixXd BTP(2, 2), BTPB(2, 2);
	Eigen::MatrixXd P(2, 2);

	P = Eigen::Matrix<double, 2, 2>::Identity();
	P(0, 0) = 1.0 / sigma1;
	P(1, 1) = 1.0 / sigma2;

	BTP = B.transpose()*P;

	BTPB = BTP*B;
	D = BTPB.inverse();

	intersectionVar.resize(2);
	intersectionVar[0] = D(0, 0);
	intersectionVar[1] = D(1, 1);

	return (0);
}
