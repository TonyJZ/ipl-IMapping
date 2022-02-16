#include "reconstruction/FloormapReconstruction.h"
#include "commonAPIs/iplutility.h"

//#include <io/iplio.h>
#include "io/PointcloudIO.h"
#include "io/PolyhedronOFF.h"
#include "io/PolylineIO.h"
#include "fitting/ModelParamsIO.h"

#include "reconstruction/GridIntersection.h"
#include "reconstruction/BuildingModelDef.h"
#include "optimization/IndoorLayoutOptimizationByGC.h"

// #include "io/polyline_io.h"
// #include "io/polyhedron_off.h"

#include "geometry/polygon/SelfintersectionReparation.h"
//#include "fitting/Line2DFitting.h"
#include "fitting/Line2DFitting_LM.h"

//CGAL
// #include <CGAL/Polyhedron_incremental_builder_3.h>
// #include<CGAL/IO/Polyhedron_iostream.h>
#include <CGAL/Polygon_2.h>

//PCL
#include <pcl/common/angles.h>
// #define PCL_NO_PRECOMPILE
// #include <pcl/sample_consensus/method_types.h>
// #include <pcl/sample_consensus/model_types.h>
// #include <pcl/segmentation/sac_segmentation.h>

//GDAL
#include <gdal/gdal_alg.h>
#include <gdal/gdal_priv.h>   
#include <gdal/ogrsf_frmts.h>

#ifdef HAVE_OpenMP
#include <omp.h>
#endif

#ifdef _DEBUG
#define TEST_VERSION	
#define OUTPUT_IMMEDIATEFILE
#endif


ipl::FloorMapReconstruction::FloorMapReconstruction()
{
	alpha_ = 1.0;
	beta_ = 0.08/*0.5*/;

	inner_edge_Th_ = 2.0; //内部边大于2.0m的保留
	polygon_area_Th_ = 3.0;

	border_edge_Th_ = 4.0;
}

ipl::FloorMapReconstruction::~FloorMapReconstruction()
{

}

void 
ipl::FloorMapReconstruction::setOutputDir(const char* pDir)
{
	outputDir_ = pDir;
}

int
ipl::FloorMapReconstruction::loadSegments(const std::string &wall_dir, const std::string &ext, const std::string &nonwall)
{
	pcl::PointCloud<PointT>::Ptr seg_cloud(new pcl::PointCloud <PointT>);

	p2cLut_.clear();
	wall_cloud_.reset(new pcl::PointCloud <PointT>);
	nonwall_cloud_.reset(new pcl::PointCloud <PointT>);
	seg_indices_.clear();

	std::vector<std::string> filenames;
	ipl::scan_files(wall_dir, ext, filenames);

	int nSegs = filenames.size();
	if (nSegs <= 0)
	{
		std::cout << "can't find " << ext << " in " << wall_dir << std::endl;
		return (-1);
	}

	std::cout << "load point cloud ... " << "wall number: " << nSegs << std::endl;
	seg_indices_.resize(nSegs);
	int curptId = 0;
	for (int i = 0; i < nSegs; i++)
	{
		std::cout << "loading wall " << i << "\r";
		//提取分割点云和模型参数
		std::string rname, sname;

		ipl::extract_file_name(filenames[i], rname, sname);
		std::string paraname = rname;
		paraname += geoModel::ModelParamFile_Suffix;

		ipl::geoModel::geoModelInfo mCoef;
		//ipl::geoModel::geoModelType type;

		double bbmin[3], bbmax[3];
		ipl::load_geoModel_params(paraname, /*bbmin, bbmax,*/ mCoef);
		memcpy(bbmin, mCoef.bbox.min_pt, sizeof(double) * 3);
		memcpy(bbmax, mCoef.bbox.max_pt, sizeof(double) * 3);

		//提取分割点云
		ipl::read_PointCloud(filenames[i], *seg_cloud);

		seg_indices_[i] = IndicesPtr(new std::vector<int>);
		for (int j = 0; j < seg_cloud->size(); j++)
		{//合并点云并记录每个分割的索引
			wall_cloud_->points.push_back(seg_cloud->points[j]);

			p2cLut_.push_back(i);

			seg_indices_[i]->push_back(curptId);
			curptId++;
		}

		ipl::PointGeoSegment<PointT> seg;

		//seg.setInputCloud(seg_cloud);

		seg.setModelCoef(mCoef);

		segments_.push_back(seg);
	}
	std::cout << "loading walls done!" << std::endl;

	wall_cloud_->height = 1;
	wall_cloud_->width = wall_cloud_->size();

	//初始化segments
	for (int i = 0; i < nSegs; i++)
	{
		segments_[i].setInputCloud(wall_cloud_);
		segments_[i].setIndices(seg_indices_[i]);
	}

	std::cout << "loading ceiling... ";
	read_PointCloud(nonwall, *nonwall_cloud_);
	std::cout << "done!" << std::endl;

	getProjectedLineCeoffs();

	return (0);

}

int
ipl::FloorMapReconstruction::getProjectedLineCeoffs()
{
	std::cout << "estimate the projected line function for each wall..." << std::endl;

// 	pcl::PointIndices inliers;
// 	// Create the segmentation object
// 	pcl::SACSegmentation<ipl::iplPointXY> seg;
// 	// Optional
// 	seg.setOptimizeCoefficients(true);
// 	seg.setModelType(pcl::SACMODEL_LINE);
// 	seg.setMethodType(pcl::SAC_RANSAC);
// 	seg.setDistanceThreshold(0.2);

	boost::shared_ptr<const iplPointCloud<PointT> > cloud;
	boost::shared_ptr <std::vector<int> > indice;

	projected_lines_.clear();
	int nWalls = segments_.size();
	for (int i = 0; i < nWalls; i++)
	{
		std::cout << "processing: " << i << "/" << nWalls << "\r";
		iplModelCoeffs coef;
		cloud = segments_[i].getInputCloud();
		indice = segments_[i].getIndices();
		
		std::vector<double> x, y;
		int ptNum = indice->size();
		x.resize(ptNum);
		y.resize(ptNum);
		for (int j = 0; j < ptNum; j++)
		{
			int id = indice->at(j);
			x[j] = cloud->points[id].x;
			y[j] = cloud->points[id].y;
		}

		coef = segments_[i].getModelCoef().coef;
		fit_line2D_by_LM(&x[0], &y[0], ptNum, &coef);
		
		projected_lines_.push_back(coef);

	}
	std::cout << "processing: " << nWalls << "/" << nWalls << std::endl;

	return (0);
}

int 
ipl::FloorMapReconstruction::lineArrangement()
{
	//为保证lineArrangement的数据范围和计算属性的范围一致，直接用统一的wall_bbmin_, wall_bbmax_
	std::vector<simpleLineSeg> linesegs;
	simpleLineSeg  lseg;

	arr_.clear();

//  	Segment_2 s1(Point_2(wall_bbmin_[0] , wall_bbmin_[1]), Point_2(wall_bbmax_[0], wall_bbmin_[1]));
//  	insert(arr_, s1);
// 
//  	Segment_2 s2(Point_2(wall_bbmax_[0], wall_bbmin_[1]), Point_2(wall_bbmax_[0], wall_bbmax_[1]));
//  	insert(arr_, s2);
//  
//  	Segment_2 s3(Point_2(wall_bbmax_[0], wall_bbmax_[1]), Point_2(wall_bbmin_[0], wall_bbmax_[1]));
//  	insert(arr_, s3);
//  
//  	Segment_2 s4(Point_2(wall_bbmin_[0], wall_bbmax_[1]), Point_2(wall_bbmin_[0], wall_bbmin_[1]));
//  	insert(arr_, s4);

	std::cout << "create line arrangement... " << std::endl;
	int nWalls = segments_.size();
	for (int i = 0; i < nWalls; i++)
	{//计算每个segment与bbox的交点
		iplModelCoeffs mCoef = /*segments_[i].getModelCoef();*/projected_lines_[i];

#ifdef TEST_VERSION
		Eigen::Vector3f  nVertical = Eigen::Vector3f(0, 0, 1);
		Eigen::Vector3f plane_normal(mCoef.values[0], mCoef.values[1], mCoef.values[2]);
		float dot_product = fabsf(plane_normal.dot(nVertical));

		float crossA = pcl::rad2deg(acosf(dot_product));
		std::cout << i << " included angle between plane normal and vertical: " << crossA << std::endl;

		
		Eigen::Vector3f projected_line = Eigen::Vector3f(projected_lines_[i].values[0], projected_lines_[i].values[1], projected_lines_[i].values[2]);
		dot_product = fabsf(plane_normal.dot(projected_line));

		crossA = pcl::rad2deg(acosf(dot_product));
		std::cout << i << " included angle between projected line and plane normal: " << crossA << std::endl;

		mCoef = projected_lines_[i];
#endif
		//line's equation  mCoef.values[0], mCoef.values[1], mCoef.values[3]
		Point_2  p1, p2;
		double x, y;
		bool bHorizontal = false, bVertical = false;

		if (mCoef.values[0] == 0)
		{//水平线
			bHorizontal = true;
			x = wall_bbmin_[0];
			y = -mCoef.values[3] / mCoef.values[1];
			p1 = Point_2(x, y);

			x = wall_bbmax_[0];
			y = -mCoef.values[3] / mCoef.values[1];
			p2 = Point_2(x, y);
		}
		else if (mCoef.values[1] == 0)
		{//垂直线
			bVertical = true;
			x = -mCoef.values[3] / mCoef.values[0];
			y = wall_bbmin_[1];
			p1 = Point_2(x, y);

			x = -mCoef.values[3] / mCoef.values[0];
			y = wall_bbmax_[1];
			p2 = Point_2(x, y);
		}
		else
		{
			if (fabs(mCoef.values[0] / mCoef.values[1]) > 1)
			{
				x = -(mCoef.values[1] * wall_bbmin_[1] + mCoef.values[3]) / mCoef.values[0];
				y = wall_bbmin_[1];
				p1 = Point_2(x, y);

				x = -(mCoef.values[1] * wall_bbmax_[1] + mCoef.values[3]) / mCoef.values[0];
				y = wall_bbmax_[1];
				p2 = Point_2(x, y);
			}
			else
			{
				x = wall_bbmin_[0];
				y = -(mCoef.values[0] * wall_bbmin_[0] + mCoef.values[3]) / mCoef.values[1];
				p1 = Point_2(x, y);

				x = wall_bbmax_[0];
				y = -(mCoef.values[0] * wall_bbmax_[0] + mCoef.values[3]) / mCoef.values[1];
				p2 = Point_2(x, y);
			}
		}

		double x1, y1, x2, y2;
		if (!bHorizontal && !bVertical)
		{
			//flags: left, right, top, bottom
			bool bp1_xL, bp1_xR, bp1_yT, bp1_yB;
			bool bp2_xL, bp2_xR, bp2_yT, bp2_yB;

			bp1_xL = bp1_xR = bp1_yT = bp1_yB = false;
			bp2_xL = bp2_xR = bp2_yT = bp2_yB = false;

			x1 = p1.x().to_double();
			y1 = p1.y().to_double();
			x2 = p2.x().to_double();
			y2 = p2.y().to_double();

			if (p1.x().to_double() < wall_bbmin_[0])
				bp1_xL = true;
			if (p1.x().to_double() > wall_bbmax_[0])
				bp1_xR = true;
			if (p1.y().to_double() < wall_bbmin_[1])
				bp1_yB = true;
			if (p1.y().to_double() > wall_bbmax_[1])
				bp1_yT = true;

			if (p2.x().to_double() < wall_bbmin_[0])
				bp2_xL = true;
			if (p2.x().to_double() > wall_bbmax_[0])
				bp2_xR = true;
			if (p2.y().to_double() < wall_bbmin_[1])
				bp2_yB = true;
			if (p2.y().to_double() > wall_bbmax_[1])
				bp2_yT = true;

			if (bp1_xL || bp1_xR || bp1_yB || bp1_yT
				|| bp2_xL || bp2_xR || bp2_yB || bp2_yT)
			{
				if (bp1_xL)
				{
					x = wall_bbmin_[0];
					y = -(mCoef.values[0] * x + mCoef.values[3]) / mCoef.values[1];
					p1 = Point_2(x, y);
				}

				if (bp1_xR)
				{
					x = wall_bbmax_[0];
					y = -(mCoef.values[0] * x + mCoef.values[3]) / mCoef.values[1];
					p1 = Point_2(x, y);
				}

				if (bp2_xL)
				{
					x = wall_bbmin_[0];
					y = -(mCoef.values[0] * x + mCoef.values[3]) / mCoef.values[1];
					p2 = Point_2(x, y);
				}

				if (bp2_xR)
				{
					x = wall_bbmax_[0];
					y = -(mCoef.values[0] * x + mCoef.values[3]) / mCoef.values[1];
					p2 = Point_2(x, y);
				}

				if (bp1_yB)
				{
					y = wall_bbmin_[1];
					x = -(mCoef.values[1] * y + mCoef.values[3]) / mCoef.values[0];
					p1 = Point_2(x, y);
				}

				if (bp1_yT)
				{
					y = wall_bbmax_[1];
					x = -(mCoef.values[1] * y + mCoef.values[3]) / mCoef.values[0];
					p1 = Point_2(x, y);
				}

				if (bp2_yB)
				{
					y = wall_bbmin_[1];
					x = -(mCoef.values[1] * y + mCoef.values[3]) / mCoef.values[0];
					p2 = Point_2(x, y);
				}

				if (bp2_yT)
				{
					y = wall_bbmax_[1];
					x = -(mCoef.values[1] * y + mCoef.values[3]) / mCoef.values[0];
					p2 = Point_2(x, y);
				}
			}
		}

		x1 = p1.x().to_double();
		y1 = p1.y().to_double();
		x2 = p2.x().to_double();
		y2 = p2.y().to_double();

		Segment_2 s(p1, p2);
		insert(arr_, s);

#ifdef TEST_VERSION
		lseg.x[0] = s.source().x().to_double();
		lseg.x[1] = s.target().x().to_double();
		lseg.y[0] = s.source().y().to_double();
		lseg.y[1] = s.target().y().to_double();
		linesegs.push_back(lseg);

#endif // TEST_VERSION
	}

	std::cout << "done!" << std::endl;

/*	Halfedge_iterator heit;
	std::cout << arr_.number_of_halfedges() << " edges:" << std::endl;
	int num_antenas = 0;
	std::vector<Halfedge_handle> removal_edge_list;
	for (heit = arr_.halfedges_begin(); heit != arr_.halfedges_end(); ++heit)
	{
		typename Arrangement_2::Face_handle inner_face = heit->twin()->face();
		if (inner_face == heit->face())
		{
			removal_edge_list.push_back(heit);
			num_antenas++;
		}
	}

	for (int i = 0; i < removal_edge_list.size(); i++)
	{
		arr_.remove_edge(removal_edge_list[i]);
	}

	std::cout << num_antenas << " antenas removed." << std::endl;*/

#ifdef TEST_VERSION
	{
		std::string  filename = outputDir_;
		filename += "/lineSegs_beforeArr.txt";
		write_2Dpolylines_TXT(filename, linesegs);
	}
#endif // TEST_VERSION

	
	return (0);
}

int 
ipl::FloorMapReconstruction::createLayout(const float wGridsize, const float cGridsize)
{
	wall_gridsize_ = wGridsize;
	ceiling_gridsize_ = cGridsize;


	boost::unordered_map<int, LayoutAtt>::iterator  attIter;
	
	double cInverse_leaf_size = 1.0 / cGridsize;
	
	pcl::getMinMax3D(*wall_cloud_, wall_bbmin_, wall_bbmax_); //以墙面点的数据范围为基准
	//边界扩大，避免产生毛刺
	wall_bbmin_[0] -= 1;
	wall_bbmin_[1] -= 1;
	wall_bbmax_[0] += 1;
	wall_bbmax_[1] += 1;

	//1. 构造点云格网，进行数据统计
	std::cout << "create quadtree partition for ceiling points...";

	PointPartitionQuadtree<PointT> quadtree_partition;

	quadtree_partition.setInputCloud(nonwall_cloud_);
	quadtree_partition.setBBox(wall_bbmin_, wall_bbmax_);
	quadtree_partition.apply(ceiling_gridsize_);
//	cell_list_nonwall_ = *(quadtree_partition.getCellList());
	keymap_nonwall_ = *(quadtree_partition.getCellKeyMap());
	std::cout << " done!" << std::endl;

	std::cout << "create quadtree partition for wall points...";
	quadtree_partition.setInputCloud(wall_cloud_);
	quadtree_partition.setBBox(wall_bbmin_, wall_bbmax_);
	quadtree_partition.apply(wall_gridsize_);
//	cell_list_wall_ = *(quadtree_partition.getCellList());
	keymap_wall_ = *(quadtree_partition.getCellKeyMap());
	std::cout << " done!" << std::endl;

	//2. 空间划分
	lineArrangement();
	Landmarks_pl landmarks_pl;
	landmarks_pl.attach(arr_);
	

	//为vertex, edge, facet添加索引号
	//计算facet和edge的LayoutAtt
	int nvertices = arr_.number_of_vertices();
	int nvertices_inf = arr_.number_of_vertices_at_infinity();
	nvertices -= nvertices_inf;

	std::cout << "arrangement information: " << std::endl;

	int idx;
	////////////////////vertices///////////////////////////////////////////
	points_.resize(arr_.number_of_vertices());
	Vertex_handle vit;
	std::cout << arr_.number_of_vertices() << " vertices with " << arr_.number_of_vertices_at_infinity()
		<< " infinity points." << std::endl;
	for (vit = arr_.vertices_begin(), idx=0; vit != arr_.vertices_end(); ++vit) 
	{
		vit->set_data(idx);

		double x, y, z;
		x = vit->point().x().to_double();
		y = vit->point().y().to_double();
		z = 0;
		Polyhedron::Point_3 p(x, y, z);
		points_[idx] = p;

		idx++;
	}

	////////////////////////////edge/////////////////////////////////////////////
	int nhalfedges = arr_.number_of_halfedges();
	
	//halfedge_att_list_.resize(nhalfedges, att);
	Halfedge_iterator heit;
	std::cout << nhalfedges << " halfedges with " << arr_.number_of_curves() << " curves." << std::endl;
	for (heit = arr_.halfedges_begin(), idx=0; heit != arr_.halfedges_end(); ++heit, ++idx)
	{
		int eid = heit->data();

		heit->set_data(idx);
		//eit->twin()->set_data(idx);
	}
		
	/////////////////////////////face/////////////////////////////////////
	int nfaces = arr_.number_of_faces();
	int nfaces_unbounded = arr_.number_of_unbounded_faces();
	//nfaces -= nfaces_unbounded;
	
	//face_att_list_.resize(nfaces, att);

	facet_area_list_.resize(nfaces, 0);
	//face_halfedge_list_.resize(nfaces);

	// Print the arrangement faces.
	Face_iterator fit;
	std::cout << arr_.number_of_faces() << " faces with " << arr_.number_of_unbounded_faces() << " unbounded faces." << std::endl;
	for (fit = arr_.faces_begin(), idx = 1; fit != arr_.faces_end(); ++fit)
	{
#ifdef TEST_VERSION
		if (!fit->is_unbounded())
		{
			typedef CGAL::Polygon_2<Kernel> Polygon_2;
			Polygon_2 poly;

			Arrangement_2::Ccb_halfedge_circulator  circ = fit->outer_ccb();
			Arrangement_2::Ccb_halfedge_circulator  curr = circ;

			//int vid = curr->source()->data();
			//vlist.push_back(vid);
		
			do
			{
				//Arrangement_2::Halfedge_const_handle he = curr->handle();
				Arrangement_2::Halfedge_handle he = curr;

	// 			int eid = he->data();
	// 			fe_list.push_back(eid);
	// 
	// 			int vid = he->target()->data();

			
				Point_2 p = he->target()->point();

				poly.push_back(p);
			
				curr = curr->next();

			} while (curr != circ);

			double area = poly.area().to_double();
			facet_area_list_[idx] = area;
		}
		//face_halfedge_list_[idx] = fe_list;
#endif
		
		if (!fit->is_unbounded())
		{
			fit->set_data(idx);
			++idx;
		}
		else
			fit->set_data(0); //unbounded face labels 0
	}

	face_flags_.clear();
	face_flags_.resize(nfaces, false);  //inner: true;  outer: false 
		
#ifdef OUTPUT_IMMEDIATEFILE
	std::string outputname = outputDir_;
	outputname += "/init_cells.shp";
	exportAllPolygons(outputname, 5, true);

	outputname = outputDir_;
	outputname += "/polylines.shp";
	exportInnerPolylines(outputname, 5, false);
#endif
//	exportLineArrangement("D:/code_indoor_location/sample_data/sample1/ref_dataset/result/linearrange.off", 5.0);

	///////////////////////////////////////////////////////////////////////////////////////
	//3. 为face添加属性值
	int   vNumX = static_cast<int>((wall_bbmax_[0] - wall_bbmin_[0]) / ceiling_gridsize_) + 1;
	int   vNumY = static_cast<int>((wall_bbmax_[1] - wall_bbmin_[1]) / ceiling_gridsize_) + 1;
	int cell_num = vNumX*vNumY;
	std::cout << "ceiling grid: " << cell_num << std::endl;
	std::cout << "calculate features for each face..." << std::endl;

	LayoutAtt att;
	att.occupied = 0;
	att.empty = 0;
	for (int i = 0; i < nfaces; i++)
	{
		face_att_list_.insert(std::make_pair(i, att));
	}
	for (int i = 0; i < nhalfedges; i++)
	{
		halfedge_att_list_.insert(std::make_pair(i, att));
	}

	//遍历arrangement中的face, 利用GridIntersection计算落入face中的grid数量
	GridIntersection grid_intersect;
	grid_intersect.InitialGrid(wall_bbmin_, wall_bbmax_, ceiling_gridsize_, ceiling_gridsize_);

#ifdef HAVE_OpenMP
	#pragma omp parallel private(fit)
	{
#endif // HAVE_OpenMP
	for (fit = arr_.faces_begin(); fit != arr_.faces_end(); ++fit)
	{
#ifdef HAVE_OpenMP
		#pragma omp single nowait
		{
#endif // HAVE_OpenMP

		int idx = fit->data();
		std::cout << "processing: " << idx << "/" << nfaces << "\r";
		if (!fit->is_unbounded())
		{
			ipl::GridIntersection::Polygon_2 poly;

			Arrangement_2::Ccb_halfedge_circulator  circ = fit->outer_ccb();
			Arrangement_2::Ccb_halfedge_circulator  curr = circ;

			do
			{
				Arrangement_2::Halfedge_handle he = curr;

				ipl::GridIntersection::Point_2 p(he->target()->point().x().to_double(), he->target()->point().y().to_double());

				poly.push_back(p);

				curr = curr->next();

			} while (curr != circ);

			CellKeyMap intersected_grids;
			grid_intersect.getIntersectedGrids(poly, intersected_grids); //扫描线求交
			for (CellKeyMap::iterator it_intersected = intersected_grids.begin();
				it_intersected != intersected_grids.end();
				++it_intersected)
			{
				QuadtreeKey key_arg = it_intersected->first;

				CellKeyMap::iterator it_vm;
				it_vm = keymap_nonwall_.find(key_arg);
				if (it_vm != keymap_nonwall_.end())
				{//存在非墙面点
				 //				occupied = true;
					face_att_list_[idx].occupied++;
				}
				else
				{
					//				occupied = false;
					face_att_list_[idx].empty++;
				}
			}
		}
		else
		{
			face_att_list_[idx].empty = 1;
			face_att_list_[idx].occupied = 0;
		}
#ifdef HAVE_OpenMP
		}
#endif
	}
#if HAVE_OpenMP
	}
#endif
	std::cout << "processing " << nfaces << "/" << nfaces << std::endl;

/*	for (int ix = 0; ix < vNumX; ix++)
	{
		for (int iy = 0; iy < vNumY; iy++)
		{
			std::cout << "processing: " << ix*vNumY + iy << "/" << cell_num << "\r";

			bool occupied; //标记当前格网是否有数据
			QuadtreeKey key_arg;
			key_arg.x = ix;
			key_arg.y = iy;

			CellKeyMap::iterator it_vm;
			it_vm = keymap_nonwall_.find(key_arg);
			if (it_vm != keymap_nonwall_.end())
			{//存在非墙面点
				occupied = true;
			}
			else
			{
				occupied = false;
			}

			//找到所属face
			double x = wall_bbmin_[0] + ix*ceiling_gridsize_ + 0.5*ceiling_gridsize_;
			double y = wall_bbmin_[1] + iy*ceiling_gridsize_ + 0.5*ceiling_gridsize_;

			CGAL::Arr_point_location_result<Arrangement_2>::Type obj = 
				landmarks_pl.locate(Point_2(x, y));

// 			const Vertex_const_handle*   v;
// 			const Halfedge_const_handle* e;
 			const Face_const_handle*     f;
			//std::cout << "The point (" << q << ") is located ";
			if (f = boost::get<Face_const_handle>(&obj)) // located inside a face
			{
				if (!(*f)->is_unbounded())
				{
					idx = (*f)->data();
					
					if (occupied)
					{
						face_att_list_[idx].occupied++;
					}
					else
					{
						face_att_list_[idx].empty++;
					}
				}
			}
		}
	}
	//the unbounded face
	face_att_list_[0].occupied = 0;
	face_att_list_[0].empty = 1;

// 	std::cout << std::endl;
// 	std::cout << "face 125: " << face_att_list_[125].occupied << "," << face_att_list_[125].empty << std::endl;

	std::cout << "processing: " << cell_num << "/" << cell_num << std::endl;
	//当facet非常小，存在漏掉的情况（在一个网格中，但网格中心不在facet内）
	//add the lost facets
	std::cout << "check small faces..." << std::endl;
	for (fit = arr_.faces_begin(); fit != arr_.faces_end(); ++fit)
	{
		if (fit->is_unbounded())
		{
			continue;
		}

		idx = fit->data();

		std::cout << "processing " << idx << "/" << nfaces << "\r";

		if(face_att_list_[idx].empty != 0 || face_att_list_[idx].occupied != 0)
			continue;

		double x = fit->outer_ccb()->target()->point().x().to_double();
		double y = fit->outer_ccb()->target()->point().y().to_double();

		int ijk0 = static_cast<int> (floor((x - wall_bbmin_[0]) * cInverse_leaf_size));
		int ijk1 = static_cast<int> (floor((y - wall_bbmin_[1]) * cInverse_leaf_size));

		QuadtreeKey	key_arg;
		key_arg.x = ijk0; key_arg.y = ijk1;

		CellKeyMap::iterator it_vm;
		it_vm = keymap_nonwall_.find(key_arg);

		if (it_vm != keymap_nonwall_.end())
		{//exist voxel
			face_att_list_[idx].empty = 0;
			face_att_list_[idx].occupied = 1;
		}
		else
		{
			face_att_list_[idx].empty = 1;
			face_att_list_[idx].occupied = 0;
		}
	}
	std::cout << "processing " << nfaces << "/" << nfaces << std::endl;*/
	
	//利用grid_intersection计算edge的LayoutAtt
	//GridIntersection grid_intersect;
	grid_intersect.InitialGrid(wall_bbmin_, wall_bbmax_, wall_gridsize_, wall_gridsize_);

	geometry::LineSeg2D seg;
	std::vector<bool> halfedge_traverse_flag;

	std::cout << "calculate features for each edge..." << std::endl;
	halfedge_traverse_flag.resize(nhalfedges, false);

	for (heit = arr_.halfedges_begin(); heit != arr_.halfedges_end(); ++heit)
	{
		idx = heit->data();
		std::cout << "processing: " << idx << "/" << nhalfedges << "\r";

		if (halfedge_traverse_flag[idx])
			continue;

		int twin_idx = heit->twin()->data();

		halfedge_traverse_flag[idx] = halfedge_traverse_flag[twin_idx] = true;

		Point_2 p = heit->source()->point();
		seg.sp[0] = p.x().to_double();
		seg.sp[1] = p.y().to_double();

		p = heit->target()->point();
		seg.ep[0] = p.x().to_double();
		seg.ep[1] = p.y().to_double();

		CellKeyMap intersected_grids;
		grid_intersect.getIntersectedGrids(seg, intersected_grids); //扫描线求交
		for (CellKeyMap::iterator it_intersected = intersected_grids.begin();
			it_intersected != intersected_grids.end();
			++it_intersected)
		{
			QuadtreeKey key_arg = it_intersected->first;

			CellKeyMap::iterator it_vm;
			it_vm = keymap_wall_.find(key_arg);
			if (it_vm != keymap_wall_.end())
			{//存在非墙面点
//				occupied = true;
				halfedge_att_list_[idx].occupied++;
				halfedge_att_list_[twin_idx].occupied++;
			}
			else
			{
//				occupied = false;
				halfedge_att_list_[idx].empty++;
				halfedge_att_list_[twin_idx].empty++;
			}
		}
	}

	std::cout << "processing: " << nhalfedges << "/" << nhalfedges << std::endl;

#ifdef TEST_VERSION
	for (int i = 0; i < face_att_list_.size(); i++)
	{
		if (face_att_list_[i].occupied == 0 && face_att_list_[i].empty == 0)
		{
			std::cout << "error!" << std::endl;
		}
	}

	for (int i = 0; i < halfedge_att_list_.size(); i++)
	{
		if (halfedge_att_list_[i].occupied == 0 && halfedge_att_list_[i].empty == 0)
		{
			std::cout << "error!" << std::endl;
		}
	}

#endif
	

	//构造IndoorLayoutOptimization_GraphCut并求解
	IndoorLayoutOptimizationByGC  opt_gc;
	std::cout << "select a solver: GraphCut" << std::endl;

	opt_gc.beginBuildGraph(nfaces);
	for (fit = arr_.faces_begin(), idx = 0; fit != arr_.faces_end(); ++fit)
	{
		double source_weight, sink_weight;
		if (!fit->is_unbounded())
		{
			idx = fit->data();
			double fv = face_att_list_[idx].occupied / (face_att_list_[idx].occupied + face_att_list_[idx].empty);
			source_weight = alpha_*exp(-fv);
			//source_weight = alpha_*fv;
			sink_weight = 1.0 - source_weight;
		}
		else
		{
			idx = fit->data();
			source_weight = alpha_;
			sink_weight = 1.0 - alpha_;
		}

//		source_weight = 1;
//		sink_weight = 0;
		opt_gc.add_UnaryCost(idx, source_weight, sink_weight);
	}
	
// 	std::vector<double>  edgeWList;
// 	edgeWList.resize(nfaces*nfaces, -1);

	Face_handle unb_face = arr_.unbounded_face();
	halfedge_traverse_flag.assign(nhalfedges, false);
	for (heit = arr_.halfedges_begin(), idx = 0; heit != arr_.halfedges_end(); ++heit, ++idx)
	{
		int eidx = heit->data();
		int twin_eidx = heit->twin()->data();
		int source_key, target_key;
		double edge_weight;

		if (halfedge_traverse_flag[eidx])
			continue;

		halfedge_traverse_flag[eidx] = halfedge_traverse_flag[twin_eidx] = true;

		//Halfedge_const_handle he = eit;
// 		if ((heit->face() != unb_face) && (heit->twin()->face()) != unb_face)
// 		{
			if (heit->face() == heit->twin()->face())
			{
				int fid = heit->face()->data();
				int tfid = heit->twin()->face()->data();
				continue;
			}

			source_key = heit->face()->data();
			target_key = heit->twin()->face()->data();

			double evw = halfedge_att_list_[eidx].occupied / (halfedge_att_list_[eidx].occupied + halfedge_att_list_[eidx].empty);
			double fv = face_att_list_[source_key].occupied / (face_att_list_[source_key].occupied + face_att_list_[source_key].empty);
			double fw = face_att_list_[target_key].occupied / (face_att_list_[target_key].occupied + face_att_list_[target_key].empty);

			edge_weight = beta_*exp(-0.5*(evw + fabs(fv - fw)));
			//edge_weight = beta_*exp(-(evw * fabs(fv - fw)));
			//edge_weight = beta_*exp(0.5*(evw+fabs(fv - fw)));
			//edge_weight = beta_*(evw+fabs(fv - fw))*0.5;
			opt_gc.add_DirectedBinaryCost(source_key, target_key, edge_weight);

			evw = halfedge_att_list_[twin_eidx].occupied / (halfedge_att_list_[twin_eidx].occupied + halfedge_att_list_[twin_eidx].empty);
			fv = face_att_list_[target_key].occupied / (face_att_list_[target_key].occupied + face_att_list_[target_key].empty);
			fw = face_att_list_[source_key].occupied / (face_att_list_[source_key].occupied + face_att_list_[source_key].empty);

			edge_weight = beta_*exp(-0.5*(evw + fabs(fv - fw)));
			//edge_weight = beta_*exp(-(evw * fabs(fv - fw)));
			//edge_weight = beta_*exp(0.5*(evw + fabs(fv - fw)));
			//edge_weight = beta_*(evw + fabs(fv - fw))*0.5;
			opt_gc.add_DirectedBinaryCost(target_key, source_key, edge_weight);

// 			if (edgeWList[source_key*nfaces + target_key] < 0)
// 			{
// 				edgeWList[source_key*nfaces + target_key] = edge_weight;
// 				edgeWList[target_key*nfaces + source_key] = edge_weight;
// 			}
// 			else
// 			{
// 				std::cout << "duplicated edge: " << source_key << "/" << target_key << std::endl;
// 				std::cout << "former weight: " << edgeWList[source_key*nfaces + target_key] << " newer: "
// 					<< edge_weight << std::endl;
// 			}
			
//		}
	}

	opt_gc.endBuildGraph();
	std::cout << "done!" << std::endl;

	face_lables_.clear();
	face_lables_ = opt_gc.getLabels();

	int nmarkedface = face_lables_[0].size() + face_lables_[1].size();
	assert(nmarkedface == nfaces);
	//std::vector<bool> face_flags_;
	face_flags_.clear();
	face_flags_.resize(nmarkedface, false);  //inner: true;  outer: false 
	for (int i = 0; i < face_lables_[1].size(); i++)
	{
		int id = face_lables_[1].at(i);

		face_flags_[id] = true;
	}
	face_flags_[0] = false;

	return (0);
}


int 
ipl::FloorMapReconstruction::export2DPolygon_txt(Arrangement_2 *arr, const int fid, const std::string &filename)
{
	//std::vector<double> xlist, ylist;
	std::vector<simpleLineSeg> linesegs;
	simpleLineSeg lseg;

	int nvertices = arr->number_of_vertices();
	int nvertices_inf = arr->number_of_vertices_at_infinity();
	nvertices -= nvertices_inf;

	Face_handle f;
	for (f = arr->faces_begin(); f != arr->faces_end(); ++f)
	{//提取面的顶点列表
	 //std::cout << "traverse face: " << f->data() << std::endl;

		if (f->is_unbounded())
		{
			continue;
		}

		int id = f->data();
		if (id != fid)
			continue;

		//outer border
		Arrangement_2::Ccb_halfedge_circulator  circ = f->outer_ccb();
		Arrangement_2::Ccb_halfedge_circulator  curr = circ;

		int vid = 0;

		//int vid = curr->source()->data();
		//vlist.push_back(vid);

		do
		{
			//Arrangement_2::Halfedge_const_handle he = curr->handle();
			Arrangement_2::Halfedge_handle he = curr;
			// 			Polyhedron::Point_3 p3(he->target()->point().x().to_double(), he->target()->point().y().to_double(), floor_hei);
			// 
			// 			points.push_back(p3);

			vid = he->target()->data();
			
			lseg.x[0] = he->source()->point().x().to_double();
			lseg.x[1] = he->target()->point().x().to_double();
			lseg.y[0] = he->source()->point().y().to_double();
			lseg.y[1] = he->target()->point().y().to_double();
			linesegs.push_back(lseg);

			//vid++;

			curr = curr->next();

		} while (curr != circ);

		write_2Dpolylines_TXT(filename, linesegs);

		//holes (inner border)   need to test: how to create a suface mesh with holes
		if (true) {
			int ihole = 0;
			Arrangement_2::Hole_iterator hi;
			for (hi = f->holes_begin(); hi != f->holes_end(); ++hi, ++ihole)
			{
				std::vector<simpleLineSeg> holeSegs;
				Arrangement_2::Ccb_halfedge_circulator circ = *hi;
				Arrangement_2::Ccb_halfedge_circulator curr = circ;

				do
				{
					Arrangement_2::Halfedge_handle he = curr;
					// 				Polyhedron::Point_3 p3(he->target()->point().x().to_double(), he->target()->point().y().to_double(), floor_hei);
					// 				
					// 				points.push_back(p3);
					//find adjacent facets
					vid = he->target()->data();
					lseg.x[0] = he->source()->point().x().to_double();
					lseg.x[1] = he->target()->point().x().to_double();
					lseg.y[0] = he->source()->point().y().to_double();
					lseg.y[1] = he->target()->point().y().to_double();
					holeSegs.push_back(lseg);

					curr = curr->next();
				} while (curr != circ);
				

				std::string rname, sname;

				ipl::extract_file_name(filename, rname, sname);
				std::string paraname = rname;
				char buf[256];
				sprintf(buf, "_hole_%02d.txt", ihole);
				paraname += buf;

				write_2Dpolylines_TXT(paraname, holeSegs);
			}
		}
		break;
	}

	return (0);
}

int
ipl::FloorMapReconstruction::exportAll2DPolygon_txt(Arrangement_2 *arr, const std::string &dir)
{
	//std::vector<double> xlist, ylist;
	std::vector<simpleLineSeg> linesegs;
	simpleLineSeg lseg;

	int nvertices = arr->number_of_vertices();
	int nvertices_inf = arr->number_of_vertices_at_infinity();
	nvertices -= nvertices_inf;

	Face_handle f;
	for (f = arr->faces_begin(); f != arr->faces_end(); ++f)
	{//提取面的顶点列表
	 //std::cout << "traverse face: " << f->data() << std::endl;

		if (f->is_unbounded())
		{
			continue;
		}

		int id = f->data();
		linesegs.clear();

		char buf[128];
		sprintf(buf, "face_%03d.txt", id);
		std::string filename;
		filename = dir;
		filename += buf;

// 		if (id != fid)
// 			continue;

		//outer border
		Arrangement_2::Ccb_halfedge_circulator  circ = f->outer_ccb();
		Arrangement_2::Ccb_halfedge_circulator  curr = circ;

		int vid = 0;

		//int vid = curr->source()->data();
		//vlist.push_back(vid);

		do
		{
			//Arrangement_2::Halfedge_const_handle he = curr->handle();
			Arrangement_2::Halfedge_handle he = curr;
			// 			Polyhedron::Point_3 p3(he->target()->point().x().to_double(), he->target()->point().y().to_double(), floor_hei);
			// 
			// 			points.push_back(p3);

			vid = he->target()->data();
			lseg.x[0] = he->source()->point().x().to_double();
			lseg.x[1] = he->target()->point().x().to_double();
			lseg.y[0] = he->source()->point().y().to_double();
			lseg.y[1] = he->target()->point().y().to_double();
			linesegs.push_back(lseg);

			//vid++;

			curr = curr->next();

		} while (curr != circ);

// 		xlist.push_back(circ->target()->point().x().to_double());
// 		ylist.push_back(circ->target()->point().y().to_double());
		write_2Dpolylines_TXT(filename, linesegs);

		//holes (inner border)   need to test: how to create a suface mesh with holes
		if (true) {
			int ihole = 0;
			Arrangement_2::Hole_iterator hi;
			for (hi = f->holes_begin(); hi != f->holes_end(); ++hi, ++ihole)
			{
				std::vector<simpleLineSeg> holeSegs;
				Arrangement_2::Ccb_halfedge_circulator circ = *hi;
				Arrangement_2::Ccb_halfedge_circulator curr = circ;

				do
				{
					Arrangement_2::Halfedge_handle he = curr;
					// 				Polyhedron::Point_3 p3(he->target()->point().x().to_double(), he->target()->point().y().to_double(), floor_hei);
					// 				
					// 				points.push_back(p3);
					//find adjacent facets
					vid = he->target()->data();
					lseg.x[0] = he->source()->point().x().to_double();
					lseg.x[1] = he->target()->point().x().to_double();
					lseg.y[0] = he->source()->point().y().to_double();
					lseg.y[1] = he->target()->point().y().to_double();
					holeSegs.push_back(lseg);

					curr = curr->next();
				} while (curr != circ);
				
				std::string rname, sname;

				ipl::extract_file_name(filename, rname, sname);
				std::string paraname = rname;
				char buf[256];
				sprintf(buf, "_hole_%02d.txt", ihole);
				paraname += buf;

				write_2Dpolylines_TXT(paraname, holeSegs);
			}
		}
//		break;
	}

	return (0);
}

int
ipl::FloorMapReconstruction::exportFaceofLineArrangement_off(Arrangement_2 *arr, const int fid, const std::string &filename, const double floor_hei)
{
	//构造Polyhedron，输出边界
	//std::vector<Polyhedron::Point_3>  points;
	std::vector<std::vector<int>> flist;	//facet vertex list  counter-clockwise
	std::vector<std::vector<int>> hlist;	//holes

	std::vector<int>  elist;

	int nvertices = arr->number_of_vertices();
	int nvertices_inf = arr->number_of_vertices_at_infinity();
	nvertices -= nvertices_inf;

	//points.resize(nvertices);

	/*int idx, ncount = 0;
	Vertex_const_handle vit;
	std::cout << arr->number_of_vertices() << " vertices:" << std::endl;
	for (vit = arr->vertices_begin(); vit != arr->vertices_end(); ++vit)
	{//提取顶点
		ncount++;

		idx = vit->data();

		if (vit->is_isolated())
		{
			std::cout << "logic error! isolated vertices is not allowed: idx " << idx << std::endl;
			assert(false);
		}
		else
		{
			//std::cout << idx << " - degree " << vit->degree() << std::endl;
			double x, y, z;
			x = vit->point().x().to_double();
			y = vit->point().y().to_double();
			z = floor_hei;
			Polyhedron::Point_3 p(x, y, z);
			points[idx] = p;
		}
	}*/

	Face_handle f;
	for (f = arr->faces_begin(); f != arr->faces_end(); ++f)
	{//提取面的顶点列表
	 //std::cout << "traverse face: " << f->data() << std::endl;

		if (f->is_unbounded())
		{
			continue;
		}

		int id = f->data();
		if(id!=fid)
			continue;
		
		std::vector<int>  vlist;//当前face的顶点列表

		//outer border
		Arrangement_2::Ccb_halfedge_circulator  circ = f->outer_ccb();
		Arrangement_2::Ccb_halfedge_circulator  curr = circ;

		int vid = 0;

		//int vid = curr->source()->data();
		//vlist.push_back(vid);

		do
		{
			//Arrangement_2::Halfedge_const_handle he = curr->handle();
			Arrangement_2::Halfedge_handle he = curr;
// 			Polyhedron::Point_3 p3(he->target()->point().x().to_double(), he->target()->point().y().to_double(), floor_hei);
// 
// 			points.push_back(p3);

			vid = he->target()->data();

			vlist.push_back(vid);

			elist.push_back(he->data());

			//vid++;

			curr = curr->next();

		} while (curr != circ);

		flist.push_back(vlist);

		//holes (inner border)   need to test: how to create a suface mesh with holes
		if (false) {
			Arrangement_2::Hole_iterator hi;
			for (hi = f->holes_begin(); hi != f->holes_end(); ++hi)
			{
				Arrangement_2::Ccb_halfedge_circulator circ = *hi;
				Arrangement_2::Ccb_halfedge_circulator curr = circ;

				std::vector<int> hole_vlist;
				//int vid = curr->source()->data();
				//hole_vlist.push_back(vid);
				do
				{
					Arrangement_2::Halfedge_handle he = curr;
					// 				Polyhedron::Point_3 p3(he->target()->point().x().to_double(), he->target()->point().y().to_double(), floor_hei);
					// 				
					// 				points.push_back(p3);
									//find adjacent facets
					vid = he->target()->data();
					hole_vlist.push_back(vid);
					//vid++;

					curr = curr->next();
				} while (curr != circ);
				hlist.push_back(hole_vlist);
			}
		}
		break;
	}

	flist.insert(flist.end(), hlist.begin(), hlist.end());


	// build a polyhedron from the loaded arrays
// 	Polyhedron P;
// 	polyhedron_builder<HalfedgeDS> builder(points_, flist, hlist.size());
// 	P.delegate(builder);
// 
// 	// write the polyhedron out as a .OFF file
// 	std::ofstream os(filename);
// 	os << P;
// 	os.close();

	//typedef CGAL::Simple_cartesian<double>		Kernel;
	typedef CGAL::Cartesian<CGAL::Exact_rational>	Kernel;
	typedef CGAL::Polyhedron_3<Kernel>			Polyhedron;
	typedef Polyhedron::HalfedgeDS				HalfedgeDS;
	//typedef Polyhedron::Point_3					Point_3;

	write_Polyhedron_off<HalfedgeDS, Polyhedron, Polyhedron::Point_3>(filename, points_, flist, hlist.size());

	return (0);
}

int 
ipl::FloorMapReconstruction::extractFloormap()
{
//	std::vector<bool> face_flags_;
	double floor_hei = 5.0;

#ifdef TEST_VERSION  //记录arrangement中的vertices, halfedges, faces
	{
	int idx;
	vertices_list_.resize(arr_.number_of_vertices());

	points_.resize(arr_.number_of_vertices());

	////////////////////vertices///////////////////////////////////////////
	Vertex_handle vit;
	std::cout << arr_.number_of_vertices() << " vertices:" << std::endl;
	for (vit = arr_.vertices_begin(), idx = 0; vit != arr_.vertices_end(); ++vit)
	{
		idx = vit->data();
		/*std::cout << idx <<"(" << vit->point() << ")";
		if (vit->is_isolated())
		std::cout << " - Isolated." << std::endl;
		else
		std::cout << " - degree " << vit->degree() << std::endl;*/
		vertices_list_[idx] = vit->point();

		double x, y, z;
		x = vit->point().x().to_double();
		y = vit->point().y().to_double();
		z = floor_hei;
		Polyhedron::Point_3 p(x, y, z);
		points_[idx] = p;
	}


	////////////////////////halfedge///////////////////////////////////////
	int henum = arr_.number_of_halfedges();
	halfedge_face_list_.resize(henum);
	Halfedge_iterator heit;

	for (heit = arr_.halfedges_begin(), idx = 0; heit != arr_.halfedges_end(); ++heit, ++idx)
	{//为halfedge设置id会改变edge的id
		int he_id = heit->data();
		//heit->set_data(idx);
		if (heit->face()->is_unbounded())
			halfedge_face_list_[idx] = -1;
		else
			halfedge_face_list_[idx] = heit->face()->data();
	}

	halfedge_twin_list_.resize(henum);
	for (heit = arr_.halfedges_begin(), idx = 0; heit != arr_.halfedges_end(); ++heit, ++idx)
	{
		halfedge_twin_list_[idx] = heit->twin()->data();
	}

	/////////////////////////////face/////////////////////////////////////
	int nfaces = arr_.number_of_faces();
	int nfaces_unbounded = arr_.number_of_unbounded_faces();
//	nfaces -= nfaces_unbounded;

	face_halfedge_list_.resize(nfaces);

	// Print the arrangement faces.
	Face_iterator fit;
	std::cout << arr_.number_of_faces() << " faces:" << std::endl;
	for (fit = arr_.faces_begin(), idx = 0; fit != arr_.faces_end(); ++fit)
	{
		if (fit->is_unbounded())
		{
			continue;
		}

		Arrangement_2::Ccb_halfedge_circulator  circ = fit->outer_ccb();
		Arrangement_2::Ccb_halfedge_circulator  curr = circ;

		//int vid = curr->source()->data();
		//vlist.push_back(vid);
		std::vector<int> fe_list;

		do
		{
			//Arrangement_2::Halfedge_const_handle he = curr->handle();
			Arrangement_2::Halfedge_handle he = curr;

			int eid = he->data();
			fe_list.push_back(eid);

			int vid = he->target()->data();

			curr = curr->next();

		} while (curr != circ);

		idx = fit->data();
		face_halfedge_list_[idx] = fe_list;
	}
	}

	std::string out_dir = outputDir_;
	out_dir += "/lineArr_beformerge.txt"; 
	exportLineArrangement_TXT(&arr_, out_dir, 5);
#endif

	//合并相邻的具有相同标记的facets
	Arrangement_2::Face_handle f;
	int nfaces = arr_.number_of_faces();
	std::cout << nfaces << " faces:" << std::endl;
	int nunbfaces = 0;

	int fidx, eidx, vidx;
	std::vector<bool> face_exist_flags;
	std::vector<bool> halfedge_exist_flags;

	face_exist_flags.resize(nfaces, true);
	int halfedge_num = arr_.number_of_halfedges();
	halfedge_exist_flags.resize(halfedge_num, true);

	//std::vector<bool> face_traverse_flag;
	std::vector<bool> halfedge_traverse_flag;

	std::cout << "------------------------------" << std::endl;
	std::cout << "vertice, halfedge, face: ";
	std::cout << "(" << arr_.number_of_vertices() << ", " << arr_.number_of_halfedges()
		<< ", " << arr_.number_of_faces() << ")" << std::endl;

	std::cout << "begin to merge faces" << std::endl;

	///////////////////////////// traverse edges to merge faces //////////////////

	
	Edge_iterator eit;
	std::set<Halfedge_handle> removal_edge_list;   //按边遍历不会产生重复，按面遍历会产生重复边
	for (eit = arr_.edges_begin(); eit != arr_.edges_end(); ++eit) 
	{
		Halfedge_handle he = eit;

// 		if (he->data() == 310)
// 		{
// 			;
// 		}

		if (he->face()->is_unbounded() && he->twin()->face()->is_unbounded())
		{//antenna
			removal_edge_list.insert(he);
			continue;
		}

		int fid_i = he->face()->data();
		int fid_j = he->twin()->face()->data();

		if (face_flags_[fid_i] == face_flags_[fid_j])
		{//内部边
			if(face_flags_[fid_i])
			{//inner
				int eid = he->data();
				double sparseDeg = halfedge_att_list_[eid].occupied 
					/ (halfedge_att_list_[eid].occupied + halfedge_att_list_[eid].empty);
					
				double dx, dy;
				dx = he->source()->point().x().to_double() - he->target()->point().x().to_double();
				dy = he->source()->point().y().to_double() - he->target()->point().y().to_double();
				double edgeLen = sqrt(dx*dx + dy*dy);

// 					if (edgeLen > 4.0)
// 					{
// 						std::cout << eid << " source:(" << he->source()->point().x().to_double() << "," << he->source()->point().y().to_double() << "), "
// 							<< "target:(" << he->target()->point().x().to_double() << "," << he->target()->point().y().to_double() << ")" << std::endl;
// 					}

				if (edgeLen*sparseDeg < inner_edge_Th_)
					removal_edge_list.insert(he);
				else
					std::cout << "edge " << eid << "/" << he->twin()->data()
					<< " is an inner wall" << std::endl;
			}
			else
			{//outter
				//可能存在标记错误, 用edge来约束

				int eid = he->data();
				double sparseDeg = halfedge_att_list_[eid].occupied
					/ (halfedge_att_list_[eid].occupied + halfedge_att_list_[eid].empty);

				double dx, dy;
				dx = he->source()->point().x().to_double() - he->target()->point().x().to_double();
				dy = he->source()->point().y().to_double() - he->target()->point().y().to_double();
				double edgeLen = sqrt(dx*dx + dy*dy);

				double fdeg_i = face_att_list_[fid_i].occupied / (face_att_list_[fid_i].occupied + face_att_list_[fid_i].empty);
				double fdeg_j = face_att_list_[fid_j].occupied / (face_att_list_[fid_j].occupied + face_att_list_[fid_j].empty);

// 					if (edgeLen > 4.0)
// 					{
// 						std::cout << eid << " source:(" << he->source()->point().x().to_double() << "," << he->source()->point().y().to_double() << "), "
// 							<< "target:(" << he->target()->point().x().to_double() << "," << he->target()->point().y().to_double() << ")" << std::endl;
// 					}

				if (edgeLen*sparseDeg > border_edge_Th_ /*&&(fdeg_i > 0.5 || fdeg_j > 0.5)*/)
				{
					std::cout << "edge " << eid << "/" << he->twin()->data()
						<< " is an border wall" << std::endl;

// 					if (fdeg_i > 0.3)
// 						face_flags_[fid_i] = true;
// 					if (fdeg_j > 0.3)
// 						face_flags_[fid_j] = true;
				}
 				else
 					removal_edge_list.insert(he);
			}
		}
	}

// 		if(removal_edge_list.size()==0)
// 			break;

#ifdef	OUTPUT_IMMEDIATEFILE
	{
		std::string outputname = outputDir_;
		outputname += "/polygon_beforeEdgeRemoval.shp";
		exportAllPolygons(outputname, 5, true);

		outputname = outputDir_;
		outputname += "/polylines_beforeEdgeRemoval.shp";
		exportInnerPolylines(outputname, 5);
	}
#endif
	
	removeEdges(&arr_, &removal_edge_list);

#ifdef	OUTPUT_IMMEDIATEFILE
	{
		std::string outputname = outputDir_;
		outputname += "/polygon_beforeTinyRemoval.shp";
		exportAllPolygons(outputname, 5, true);

		outputname = outputDir_;
		outputname += "/polylines_beforeTinyRemoval.shp";
		exportInnerPolylines(outputname, 5);
	}
#endif

	removal_edge_list.clear();

	std::cout << "------------------------------" << std::endl;
	std::cout << "vertice, halfedge, face: ";
	std::cout << "(" << arr_.number_of_vertices() << ", " << arr_.number_of_halfedges()
		<< ", " << arr_.number_of_faces() << ")" << std::endl;

	//过滤微小多边形
	std::set<Face_handle>  merged_face_list;
	Face_iterator fit;
	for (fit = arr_.faces_begin(); fit != arr_.faces_end(); ++fit)
	{
		int face_id = fit->data();
		if(fit->is_unbounded())
			continue;

		//std::vector<Halfedge_handle> face_bounder;

		typedef CGAL::Polygon_2<Kernel> Polygon_2;
		Polygon_2 poly;

		Arrangement_2::Ccb_halfedge_circulator  circ = fit->outer_ccb();
		Arrangement_2::Ccb_halfedge_circulator  curr = circ;

		do
		{
			Arrangement_2::Halfedge_handle he = curr;
			int hid = he->data();
			int thid = he->twin()->data();

			//face_bounder.push_back(he);

			Point_2 p = he->target()->point();

			poly.push_back(p);

			curr = curr->next();

		} while (curr != circ);

		double area = poly.area().to_double();
		if (area < polygon_area_Th_)
		{//待删除的小多边形
			//删除策略: 合并到最相似的面中
			merged_face_list.insert(fit);
			std::cout << "find tiny polygon: " << fit->data()
				<< " area: " << area << std::endl;
		}
	}

//	removeEdges(&arr_, &removal_edge_list);
	mergeFaces(&arr_, &merged_face_list);

#ifdef OUTPUT_IMMEDIATEFILE
	{
		std::string outputname = outputDir_;
		outputname += "/polylines_afterTinyRemoval.shp";
		exportInnerPolylines(outputname, 5);
	}
	
#endif

	//merge collinear edges
	mergeCollinearEdges(&arr_);
	/////////////////////////////////////////////////////////////
	//repair the self-intersection polygons


#ifdef TEST_VERSION
	std::cout << "------------------------------" << std::endl;
	std::cout << "vertice, halfedge, face: ";
	std::cout << "(" << arr_.number_of_vertices() << ", " << arr_.number_of_halfedges()
		<< ", " << arr_.number_of_faces() << ")" << std::endl;
	for (f = arr_.faces_begin(); f != arr_.faces_end(); ++f)
	{
		if (f->is_unbounded())
		{
			std::cout << "Unbounded face. " << std::endl;
			continue;
		}

		fidx = f->data();
		std::string out_name, out_param;
		char buf[32], buf_para[32];
		sprintf(buf, "/face_%03d.txt", fidx);

		out_name = outputDir_;
		//out_name += result_name;
		out_name += buf;

		export2DPolygon_txt(&arr_, fidx, out_name);
	}
#endif // TEST_VERSION


//	exportLineArrangement(&arr_, filename, floor_hei);

	/*halfedge_traverse_flag.resize(halfedge_num, false);
	for (f = arr_.faces_begin(); f != arr_.faces_end(); ++f)
	{
		if (f->is_unbounded())
		{
			std::cout << "Unbounded face. " << std::endl;
			continue;
		}

		fidx = f->data();

		//所有的边只允许遍历一次
		//face_traverse_flag.resize(nfaces, false);
		halfedge_traverse_flag.assign(halfedge_num, false);

		std::cout << "------------------------------------------" << std::endl;
		std::cout << "traverse face: " << f->data() << std::endl;

//		int cur_face_num = arr_.number_of_faces();

		//for debug
// 		if (fidx == 4)
// 		{
// 			Arrangement_2::Ccb_halfedge_circulator  circ = f->outer_ccb();
// 			Arrangement_2::Ccb_halfedge_circulator  curr = circ;
// 			
// 			do
// 			{
// 				Arrangement_2::Halfedge_handle he = curr;
// 
// 				int id = he->data();
// 				curr = he->next();
// 
// 			} while (true);
// 
// 			curr = circ;
// 			do
// 			{
// 				Arrangement_2::Halfedge_handle he = curr;
// 
// 				int id = he->data();
// 				curr = he->prev();
// 
// 			} while (true);
// 		}
		
		//std::vector<int> merged_face_id;
		
		// Print the outer boundary.
		
		
		assert(face_exist_flags[fidx]);

		std::cout << "Outer boundary: " << std::endl;
		Arrangement_2::Ccb_halfedge_circulator  circ = f->outer_ccb();
		Arrangement_2::Ccb_halfedge_circulator  curr = circ;

		std::cout << "halfedge circulator start: " << circ->data() << std::endl;

		//int s_id = circ->data();

		//std::cout << "(" << circ->source()->point() << ")";
		do
		{
			//Arrangement_2::Halfedge_const_handle he = curr->handle();
			Arrangement_2::Halfedge_handle he = curr;

			eidx = he->data();
			int opp_eidx = he->twin()->data();

			assert(!halfedge_traverse_flag[eidx]);
			halfedge_traverse_flag[eidx] = true;

			assert(halfedge_exist_flags[eidx]);
			assert(halfedge_exist_flags[opp_eidx]);

			//std::cout << "current edge ( " << eidx << "," << opp_eidx << ")" << std::endl;

			if (!he->face()->is_unbounded() && !he->twin()->face()->is_unbounded())
			{
				//find adjacent facets
				int fid_i = he->face()->data();
				int fid_j = he->twin()->face()->data();

				assert(fid_i == f->data());
				//std::cout << "adjacent facets: " << fid_i << "," << fid_j << "  ";

				if (face_flags_[fid_i] == face_flags_[fid_j])
				{
					Halfedge_handle todelete_he = he;
					if (todelete_he->next() == todelete_he->twin() && todelete_he->twin()->next() == todelete_he)
					{
						// Self-Intersection
						std::cout << "self-intersection edge: (" << todelete_he->data() << "," << todelete_he->twin()->data()
							<< ")" << std::endl;
						//arr_.remove_edge(todelete_he);   //删除边
						break;
					}

					while (todelete_he == circ || todelete_he->twin() == circ)
					{
						circ = circ->prev();
						std::cout << "halfedge circulator start point modified: " << circ->data() << std::endl;
					}
							

					he = todelete_he->prev();
					if (he == todelete_he->twin())
						he = he->prev();

					int previd = he->data();
					int nextid = he->next()->data();

					arr_.remove_edge(todelete_he);   //删除边
					nextid = he->next()->data();
					//std::cout << "removel halfedge: " << eidx << ", " << opp_eidx << std::endl;
					//merged_face_id.push_back(fid_j);

					face_exist_flags[fid_j] = false;
					halfedge_exist_flags[eidx] = false;
					halfedge_exist_flags[opp_eidx] = false;
				}
				else
				{
					int previd = he->prev()->data();
					int nextid = he->next()->data();
					std::cout << "border edge: (" << eidx << "," << opp_eidx
						<<") between faces (" << fid_i << "," << fid_j << ")" << std::endl;
				}
				//std::cout << std::endl;
			}
			else
			{
				int previd = he->prev()->data();
				int nextid = he->next()->data();
				std::cout << "border edge: (" << eidx << "," << opp_eidx
					<< ") between faces (" << fidx << "," << -1 << ")" << std::endl;
			}
				
			curr = he->next();

		} while (curr != circ);
		std::cout << std::endl;
		


//		exportFaceofLineArrangement(&arr_, 0, "D:/code_indoor_location/sample_data/sample1/ref_dataset/result/face1.off", 6.0);

		// process the holes.
		std::cout << "face " << f->data() << " contains (" << f->number_of_holes() << ") holes" << std::endl;
		Arrangement_2::Hole_iterator hi;
		int index = 0;
		std::cout << f->number_of_holes() << " holes in facet: " << f->data() << std::endl;
// 		Arrangement_2::Hole_iterator hbegin, hend;
// 		hbegin = f->holes_begin();
// 		hend = f->holes_end();
		if(false){
		for (hi = f->holes_begin(); hi != f->holes_end(); ++hi, ++index)
		{
			std::cout << " Hole # " << index << std::endl;

			Arrangement_2::Ccb_halfedge_circulator circ = *hi;
			Arrangement_2::Ccb_halfedge_circulator curr = circ;

			//circ = circ->twin();   //对洞内的face进行合并
			
			//std::cout << "(" << curr->source()->point() << ")";

			std::cout << "halfedge circulator start: " << circ->data() << std::endl;

			//int s_id = circ->data();

			//std::cout << "(" << circ->source()->point() << ")";
			do
			{
				//Arrangement_2::Halfedge_const_handle he = curr->handle();
				Arrangement_2::Halfedge_handle he = curr;

				eidx = he->data();
				int opp_eidx = he->twin()->data();

				assert(!halfedge_traverse_flag[eidx]);
				halfedge_traverse_flag[eidx] = true;

				assert(halfedge_exist_flags[eidx]);
				assert(halfedge_exist_flags[opp_eidx]);

				//std::cout << "current edge ( " << eidx << "," << opp_eidx << ")" << std::endl;

				if (!he->face()->is_unbounded() && !he->twin()->face()->is_unbounded())
				{
					//find adjacent facets
					int fid_i = he->face()->data();
					int fid_j = he->twin()->face()->data();

					assert(fid_i == f->data());
					//std::cout << "adjacent facets: " << fid_i << "," << fid_j << "  ";

					if (face_flags_[fid_i] == face_flags_[fid_j])
					{
						//bool bSelfIntersection = false;

						Halfedge_handle todelete_he = he;

						if (todelete_he->next() == todelete_he->twin() && todelete_he->twin()->next() == todelete_he)
						{
							// Self-Intersection
							std::cout << "self-intersection edge: (" << todelete_he->data() << "," << todelete_he->twin()->data()
								<< ")" << std::endl;
							arr_.remove_edge(todelete_he);   //删除边
							break;
						}

						while (todelete_he == circ || todelete_he->twin() == circ)
						{
							circ = circ->prev();
							std::cout << "halfedge circulator start point modified: " << circ->data() << std::endl;
						}

						he = todelete_he->prev();
						if (he == todelete_he->twin())
							he = he->prev();

						int previd = he->data();
						int nextid = he->next()->data();

						arr_.remove_edge(todelete_he);   //删除边
						//nextid = he->next()->data();
						//std::cout << "removel halfedge: " << eidx << ", " << opp_eidx << std::endl;
						//merged_face_id.push_back(fid_j);

						face_exist_flags[fid_j] = false;
						halfedge_exist_flags[eidx] = false;
						halfedge_exist_flags[opp_eidx] = false;
					}
					else
					{
						int previd = he->prev()->data();
						int nextid = he->next()->data();
						std::cout << "border edge: (" << eidx << "," << opp_eidx
							<< ") between faces (" << fid_i << "," << fid_j << ")" << std::endl;
					}
					//std::cout << std::endl;
				}
				else
				{
					int previd = he->prev()->data();
					int nextid = he->next()->data();
					std::cout << "border edge: (" << eidx << "," << opp_eidx
						<< ") between faces (" << fidx << "," << -1 << ")" << std::endl;
				}

				curr = he->next();

			} while (curr != circ);
			std::cout << std::endl;

			if(hi==f->holes_end())
				break;
		}
		}

		std::string out_name_txt, out_name_off;
		char buf[32], buf_para[32];
		
		sprintf(buf, "face_%03d.txt", fidx);
		out_name_txt = "D:/code_indoor_location/sample_data/sample1/ref_dataset/result/";
		//out_name += result_name;
		out_name_txt += buf;

		sprintf(buf, "face_%03d.off", fidx);
		out_name_off = "D:/code_indoor_location/sample_data/sample1/ref_dataset/result/";
		//out_name += result_name;
		out_name_off += buf;

		//exportFaceofLineArrangement(&arr_, fidx, out_name_off, 6.0);
		export2DPolygon(&arr_, fidx, out_name_txt);

		if(f == arr_.faces_end())
			break;
	}*/
		
	return (0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//修复自相交的多边形(一种拓扑错误)
//1.计算winding number; 2. 找到自相交的ring; 3. 删除ring
int 
ipl::FloorMapReconstruction::repairLineArrangement_selfintersection(Arrangement_2 *arr, PolygonList &polygons)
{
	typedef std::pair<bool, int>                        Data;
	typedef boost::unordered_map<int, Data>				WNMap;    //winding number map
	typedef boost::unordered_map<int, Vertex_handle>	VhMap;		//vertex handle map

	Winding_number<Arrangement_2> winding_number(*arr);

	polygons.clear();
	//holes.clear();

	std::cout << arr->number_of_faces() << " faces with " << arr->number_of_unbounded_faces() << " unbounded faces" << std::endl;
	typename Arrangement_2::Face_iterator fi;
	for (fi = arr->faces_begin(); fi != arr->faces_end(); ++fi)
	{
		if(fi->is_unbounded())
			continue;

		int fid = fi->data();
		std::cout << "face " << fid;
		WNMap::iterator it_wm;
		it_wm = winding_number._windingmap.find(fid);

		assert(it_wm != winding_number._windingmap.end());

		std::cout << " winding number = " << it_wm->second.second;

		std::vector<int> vlist;
		std::vector<int> hlist;
		if ((it_wm->second.second % 2) != 0)
		{//valid
			std::cout << " is valid" << std::endl;
			typename Arrangement_2::Ccb_halfedge_circulator cco = fi->outer_ccb();
			do {
				vlist.push_back(cco->target()->data());

			} while (++cco != fi->outer_ccb());
		}
		else
		{//self-intersection polygon
			std::cout << " need to repair" << std::endl;
			VhMap	vmap;
			typename Arrangement_2::Ccb_halfedge_circulator cco = fi->outer_ccb();
			int vnum = 0;
			do {
				int vid = cco->target()->data();

				VhMap::iterator it_vm;
				it_vm = vmap.find(vid);

				if (it_vm != vmap.end())
				{//catch a ring
					hlist.insert(hlist.begin(), vid);
					while (vlist.back()!=vid)
					{
						hlist.insert(hlist.begin(), vlist.back());

						vlist.pop_back();
						vnum--;
					}

					//holes.push_back(hlist);
				}
				else
				{
					vmap.insert(std::make_pair(vid, cco->target()));
					vlist.push_back(vid);
					vnum++;
				}
			} while (++cco != fi->outer_ccb());
		}

		if(vlist.size()>0)
			polygons.insert(std::make_pair(fid, vlist));

	}

	return (0);
}

///////////////////////////////////////////////////////////////////
//修复多边形中的ring (拓扑错误的self-intersection, 或拓扑正确的ring)
//keep_antenna: 是否保留antenna
int
ipl::FloorMapReconstruction::repairLineArrangement_ring(Arrangement_2 *arr, PolygonList &polygons, bool keep_antenna)
{
	//typedef std::pair<bool, int>                        Data;
	//typedef boost::unordered_map<int, Data>				WNMap;    //winding number map
	typedef boost::unordered_map<int, Vertex_handle>	VhMap;		//vertex handle map

//	Winding_number<Arrangement_2> winding_number(*arr);

	polygons.clear();
	//holes.clear();

	int nfaces = arr->number_of_faces();
	std::cout << arr->number_of_faces() << " faces with " << arr->number_of_unbounded_faces() << " unbounded faces" << std::endl;
	typename Arrangement_2::Face_iterator fi;
	for (fi = arr->faces_begin(); fi != arr->faces_end(); ++fi)
	{
		if (fi->is_unbounded())
			continue;

		int fid = fi->data();
		std::cout << "repair face: " << fid << "/" << nfaces << "\r";

		std::vector<int> vlist;
		std::vector<int> hlist;
		VhMap	vmap;
		typename Arrangement_2::Ccb_halfedge_circulator cco = fi->outer_ccb();
		int vnum = 0;
		do {
			int vid = cco->target()->data();

			VhMap::iterator it_vm;
			it_vm = vmap.find(vid);

			if (it_vm != vmap.end())
			{//catch a ring
				if (keep_antenna && cco->face()==cco->twin()->face())
				{
					std::cout << "find an antenna:  " << vid << std::endl;
					vlist.push_back(vid);
					vnum++;
				}
				else
				{
					hlist.insert(hlist.begin(), vid);
					while (vlist.back() != vid)
					{
						hlist.insert(hlist.begin(), vlist.back());

						vlist.pop_back();
						vnum--;
					}
				}
				

				//holes.push_back(hlist);
			}
			else
			{
				vmap.insert(std::make_pair(vid, cco->target()));
				vlist.push_back(vid);
				vnum++;
			}
		} while (++cco != fi->outer_ccb());

		if (vlist.size() > 0)
			polygons.insert(std::make_pair(fid, vlist));

	}
	std::cout << "repair face: done! " << nfaces << "/" << nfaces << std::endl;

	return (0);
}

int
ipl::FloorMapReconstruction::exportLineArrangement_off(Arrangement_2 *arr, const std::string &filename, const double floor_hei)
{
	//构造Polyhedron，输出边界
	std::vector<Polyhedron::Point_3>  points;
	std::vector<std::vector<int> > flist;	//facet vertex list  counter-clockwise
	std::vector<std::vector<int> > hlist;	//holes

	/*int nvertices = arr->number_of_vertices();
	int nvertices_inf = arr->number_of_vertices_at_infinity();
	nvertices -= nvertices_inf;

	points.resize(nvertices);

	int idx, ncount = 0;
	Vertex_const_handle vit;
	std::cout << arr->number_of_vertices() << " vertices:" << std::endl;
	for (vit = arr->vertices_begin(); vit != arr->vertices_end(); ++vit)
	{//提取顶点
		ncount++;

		idx = vit->data();

		if (vit->is_isolated())
		{
			std::cout << "logic error! isolated vertices is not allowed: idx " << idx << std::endl;
			assert(false);
		}
		else
		{
			//std::cout << idx << " - degree " << vit->degree() << std::endl;
			double x, y, z;
			x = vit->point().x().to_double();
			y = vit->point().y().to_double();
			z = floor_hei;
			Polyhedron::Point_3 p(x, y, z);
			points[idx] = p;
		}
	}

	assert(ncount == nvertices);
	assert(nvertices_inf == 0);

	int nfaces = arr->number_of_faces();
	int nfaces_unbounded = arr->number_of_unbounded_faces();
	nfaces -= nfaces_unbounded;

	int ncount = 0;
	flist.resize(nfaces);
	Face_handle f;
	int idx = 0;
	for (f = arr->faces_begin(); f != arr->faces_end(); ++f)
	{//提取面的顶点列表
	 //std::cout << "traverse face: " << f->data() << std::endl;

		if (f->is_unbounded())
		{
			continue;
		}

		ncount++;

		//int idx = f->data();

		std::vector<int>  vlist;//当前face的顶点列表
								//outer border
		Arrangement_2::Ccb_halfedge_circulator  circ = f->outer_ccb();
		Arrangement_2::Ccb_halfedge_circulator  curr = circ;

		int vid = curr->source()->data();
		//vlist.push_back(vid);

		do
		{
			//Arrangement_2::Halfedge_const_handle he = curr->handle();
			Arrangement_2::Halfedge_handle he = curr;
			vid = he->target()->data();
			vlist.push_back(vid);

			curr = curr->next();

		} while (curr != circ);

		flist[idx] = vlist;
		++idx;

		//holes (inner border)   need to test: how to create a suface mesh with holes
		if(false) {
		Arrangement_2::Hole_iterator hi;
		for (hi = f->holes_begin(); hi != f->holes_end(); ++hi)
		{
			Arrangement_2::Ccb_halfedge_circulator circ = *hi;
			Arrangement_2::Ccb_halfedge_circulator curr = circ;

			std::vector<int> hole_vlist;
			int vid = curr->source()->data();
			hole_vlist.push_back(vid);
			do
			{
				Arrangement_2::Halfedge_handle he = curr;
				//find adjacent facets
				vid = he->target()->data();
				hole_vlist.push_back(vid);

				curr = curr->next();
			} while (curr != circ);
			hlist.push_back(hole_vlist);
		}
		}
	}

	assert(ncount == nfaces);

	flist.insert(flist.end(), hlist.begin(), hlist.end());*/


//	repairLineArrangement_selfintersection(arr, flist, hlist);

	//typedef CGAL::Simple_cartesian<double>		Kernel;
	typedef CGAL::Cartesian<CGAL::Exact_rational>	Kernel;
	typedef CGAL::Polyhedron_3<Kernel>			Polyhedron;
	typedef Polyhedron::HalfedgeDS				HalfedgeDS;
	//typedef Polyhedron::Point_3					Point_3;
	// build a polyhedron from the loaded arrays
	write_Polyhedron_off<HalfedgeDS, Polyhedron, Polyhedron::Point_3>(filename, points_, flist, hlist.size());
// 	Polyhedron P;
// 	polyhedron_builder<HalfedgeDS> builder(points_, flist, hlist.size());
// 	P.delegate(builder);
// 
// 	// write the polyhedron out as a .OFF file
// 	std::ofstream os(filename);
// 	os << P;
// 	os.close();

	return (0);
}

int
ipl::FloorMapReconstruction::exportLineArrangement_TXT(Arrangement_2 *arr, const std::string &filename, const double floor_hei)
{
	std::vector<simpleLineSeg> linesegs;
	simpleLineSeg lseg;

	Edge_iterator eit;
	for (eit = arr->edges_begin(); eit != arr->edges_end(); ++eit)
	{
		Halfedge_handle he = eit;
		
		lseg.x[0] = he->source()->point().x().to_double();
		lseg.x[1] = he->target()->point().x().to_double();
		lseg.y[0] = he->source()->point().y().to_double();
		lseg.y[1] = he->target()->point().y().to_double();
		linesegs.push_back(lseg);
	}

	write_2Dpolylines_TXT(filename, linesegs);

	return (0);
}

int
ipl::FloorMapReconstruction::exportOutterBoundary(const std::string &filename, const double floor_hei)
{
	const char *pszDriverName = "ESRI Shapefile";
	GDALDriver *poDriver;
	GDALAllRegister();
	poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
	if (poDriver == NULL)
	{
		std::cout << pszDriverName << " driver not available" << std::endl;
		return (-1);
	}

	GDALDataset *poDS;
	poDS = poDriver->Create(filename.c_str(), 0, 0, 0, GDT_Unknown, NULL); //创建数据源
	if (poDS == NULL)
	{
		std::cout << "can't create output file: " << filename << std::endl;
		return (-1);
	}
	OGRLayer *poLayer;
	poLayer = poDS->CreateLayer("building boundary", NULL, wkbPolygon, NULL); //创建图层
	if (poLayer == NULL)
	{
		std::cout << "Layer creation failed" << std::endl;
		return (-1);
	}

	OGRFieldDefn oField("Name", OFTString); //创建属性
	oField.SetWidth(32);
	if (poLayer->CreateField(&oField) != OGRERR_NONE)
	{
		std::cout << "Creating PolygonName field failed" << std::endl;
		return (-1);
	}

	OGRFieldDefn oField1("PolygonID", OFTInteger64);
	//oField1.SetPrecision(3);
	if (poLayer->CreateField(&oField1) != OGRERR_NONE)
	{
		std::cout << "Creating PolygonID field failed." << std::endl;
		return (-1);
	}

	Face_handle unf = arr_.unbounded_face();
	
	int fid = 0;
	char fname[32];

	// Traverse the inner boundary (holes).
	Hole_iterator hit;
	for (hit = unf->holes_begin(); hit != unf->holes_end(); ++hit, ++fid)
	{
		//fid = hit->data();
		sprintf(fname, "polygon_%03d", fid);

		OGRFeature *poFeature;
		poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
		poFeature->SetField(0, fname);
		poFeature->SetField(1, fid);

		OGRLinearRing ring;

		typename Arrangement_2::Ccb_halfedge_circulator cch = *hit;
		do {
			typename Arrangement_2::Face_handle inner_face = cch->twin()->face();
			if (inner_face == cch->face() )
				continue;        // discard antennas

			int id = cch->face()->data();
			int innerfid = cch->twin()->face()->data();

			int eid = cch->data();
			int teid = cch->twin()->data();

			double x, y;
			x = cch->target()->point().x().to_double(); 
			y = cch->target()->point().y().to_double();
			ring.addPoint(x, y, floor_hei);

		} while (++cch != *hit);

		ring.closeRings();//首尾点重合形成闭合环 
		OGRPolygon poly;
		poly.addRing(&ring);

		poFeature->SetGeometry(&poly);
		if (poLayer->CreateFeature(poFeature) != OGRERR_NONE)
		{
			std::cout << "Failed to create feature: " << fname << std::endl;
		}
		OGRFeature::DestroyFeature(poFeature);
	}

	GDALClose(poDS);
	return (0);
}

int 
ipl::FloorMapReconstruction::exportAllPolygons(const std::string &filename, const double floor_hei, const bool output_holes)
{
	PolygonList polygons;
//	repairLineArrangement_selfintersection(&arr_, polygons);

	repairLineArrangement_ring(&arr_, polygons);
	const char *pszDriverName = "ESRI Shapefile";
	GDALDriver *poDriver;
	GDALAllRegister();
	poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
	if (poDriver == NULL)
	{
		std::cout << pszDriverName << " driver not available" << std::endl;
		return (-1);
	}

	GDALDataset *poDS;
	poDS = poDriver->Create(filename.c_str(), 0, 0, 0, GDT_Unknown, NULL); //创建数据源
	if (poDS == NULL)
	{
		std::cout << "can't create output file: " << filename << std::endl;
		return (-1);
	}
	OGRLayer *poLayer;
	poLayer = poDS->CreateLayer("building boundary", NULL, wkbPolygon, NULL); //创建图层
	if (poLayer == NULL)
	{
		std::cout << "Layer creation failed" << std::endl;
		return (-1);
	}

	OGRFieldDefn oField("Name", OFTString); //创建属性
	oField.SetWidth(32);
	if (poLayer->CreateField(&oField) != OGRERR_NONE)
	{
		std::cout << "Creating PolygonName field failed" << std::endl;
		return (-1);
	}

	OGRFieldDefn oField1("PolygonID", OFTInteger64);
	//oField1.SetPrecision(3);
	if (poLayer->CreateField(&oField1) != OGRERR_NONE)
	{
		std::cout << "Creating PolygonID field failed." << std::endl;
		return (-1);
	}

	OGRFieldDefn oField2("Type", OFTString);
	//oField1.SetPrecision(3);
	if (poLayer->CreateField(&oField2) != OGRERR_NONE)
	{
		std::cout << "Creating Polygon Type field failed." << std::endl;
		return (-1);
	}
		
	// 	if (nmarkedface != (arr_.number_of_faces() - arr_.number_of_unbounded_faces()))
	// 		return (-1);

	Face_handle unf = arr_.unbounded_face();
	Hole_iterator hit;
	int fid = 0;
	char fname[32];

	PolygonList::iterator pit;
	for (pit = polygons.begin(); pit != polygons.end(); ++pit)
	{
		fid = pit->first;
		sprintf(fname, "polygon_%03d", fid);

		if (!face_flags_[fid] && !output_holes)
 			continue;	//不输出空洞多边形

		OGRFeature *poFeature;
		poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
		poFeature->SetField(0, fname);
		poFeature->SetField(1, fid);

		if (face_flags_[fid])
			poFeature->SetField(2, bim::BoundaryTypeDesc[1]);
		else
		{
			poFeature->SetField(2, bim::BoundaryTypeDesc[0]);
		}
			

		OGRLinearRing ring;

		VertexIDList::iterator vit = pit->second.begin();
		do {
			int id = *vit;
			double x, y;
			x = points_[id].x().to_double();
			y = points_[id].y().to_double();
			ring.addPoint(x, y, floor_hei);
		} while (++vit != pit->second.end());


		ring.closeRings();//首尾点重合形成闭合环 
		OGRPolygon poly;
		poly.addRing(&ring);

		poFeature->SetGeometry(&poly);
		if (poLayer->CreateFeature(poFeature) != OGRERR_NONE)
		{
			std::cout << "Failed to create feature: " << fname << std::endl;
		}
		OGRFeature::DestroyFeature(poFeature);
	}

	GDALClose(poDS);
	return (0);
}

int 
ipl::FloorMapReconstruction::exportInnerPolylines(const std::string &filename, const double floor_hei, const bool bfiltered)
{
// 	PolygonList polygons;
// 	repairLineArrangement_selfintersection(&arr_, polygons);

	const char *pszDriverName = "ESRI Shapefile";
	GDALDriver *poDriver;
	GDALAllRegister();
	poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
	if (poDriver == NULL)
	{
		std::cout << pszDriverName << " driver not available" << std::endl;
		return (-1);
	}

	GDALDataset *poDS;
	poDS = poDriver->Create(filename.c_str(), 0, 0, 0, GDT_Unknown, NULL); //创建数据源
	if (poDS == NULL)
	{
		std::cout << "can't create output file: " << filename << std::endl;
		return (-1);
	}
	OGRLayer *poLayer;
	poLayer = poDS->CreateLayer("building boundary", NULL, wkbLineString, NULL); //创建图层
	if (poLayer == NULL)
	{
		std::cout << "Layer creation failed" << std::endl;
		return (-1);
	}

	OGRFieldDefn oField("Name", OFTString); //创建属性
	oField.SetWidth(32);
	if (poLayer->CreateField(&oField) != OGRERR_NONE)
	{
		std::cout << "Creating PolygonName field failed" << std::endl;
		return (-1);
	}

	OGRFieldDefn oField1("LineID", OFTInteger64);
	//oField1.SetPrecision(3);
	if (poLayer->CreateField(&oField1) != OGRERR_NONE)
	{
		std::cout << "Creating PolygonID field failed." << std::endl;
		return (-1);
	}

	OGRFieldDefn oField2("PolygonID", OFTInteger64);
	//oField1.SetPrecision(3);
	if (poLayer->CreateField(&oField2) != OGRERR_NONE)
	{
		std::cout << "Creating PolygonID field failed." << std::endl;
		return (-1);
	}

	int eid = 0;
	char ename[32];
	OGRMultiLineString  polylines;

	Face_handle unfh = arr_.unbounded_face();
	Edge_iterator eit;
	for (eit = arr_.edges_begin(); eit != arr_.edges_end(); ++eit)
	{
		Halfedge_handle he = eit;
		int heid = he->data();
		int t_heid = he->twin()->data();

		int fid = he->face()->data();
		 
		sprintf(ename, "line_%03d\/%03d", heid, t_heid);

		OGRFeature *poFeature;
		poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
		poFeature->SetField(0, ename);
		poFeature->SetField(1, heid);
		poFeature->SetField(2, fid);

		if (bfiltered)
		{
			if (eit->face() == unfh && eit->twin()->face() == unfh)
				continue; //antenna

			if (eit->next() == eit->twin() && eit->next()->next() == eit)
				continue; //self circle
		}
 		
		OGRLineString lines;

		double x = eit->source()->point().x().to_double();
		double y = eit->source()->point().y().to_double();
		lines.addPoint(x, y, floor_hei);

		x = eit->target()->point().x().to_double();
		y = eit->target()->point().y().to_double();
		lines.addPoint(x, y, floor_hei);

		poFeature->SetGeometry(&lines);

		polylines.addGeometry(&lines);

		if (poLayer->CreateFeature(poFeature) != OGRERR_NONE)
		{
			std::cout << "Failed to create feature: lines" << std::endl;
		}
		OGRFeature::DestroyFeature(poFeature);
	}	

	GDALClose(poDS);
	return (0);
}

void
ipl::FloorMapReconstruction::removeEdges(Arrangement_2 *arr, std::set<Halfedge_handle> *removal_list)
{
	std::set<Halfedge_handle>::iterator iter;

	for (iter = removal_list->begin(); iter != removal_list->end(); ++iter)
	{
		Halfedge_handle he = *iter;
		int eid = he->data();
		int teid = he->twin()->data();

		std::cout << "delete edge (" << eid << "/" << teid << "), ";
		int fid = he->face()->data();
		int tfid = he->twin()->face()->data();

		if (face_flags_[fid] != face_flags_[tfid])
			continue;  //边界边不能删除
		
		std::cout << "in face " << fid << std::endl;

		arr->remove_edge(he);
	}
}

//合并面元时，需要判断与相邻面元的关系，合并到最相似的面元中
void
ipl::FloorMapReconstruction::mergeFaces(Arrangement_2 *arr, std::set<Face_handle> *face_list)
{
	typedef std::vector<Halfedge_handle>  EdgeList;

	// 边合并后长度已经改变，需要重新统计属性
	// 实际上每条边的起止结点都没有改变
/*	GridIntersection grid_intersect;
	grid_intersect.InitialGrid(wall_bbmin_, wall_bbmax_, gridsize_, gridsize_);

	geometry::LineSeg2D seg;
	CellKeyMap intersected_grids;
	
	std::vector<bool> halfedge_traverse_flag;

	int org_halfedge_num = halfedge_att_list_.size();
	halfedge_traverse_flag.resize(org_halfedge_num, false);

	std::cout << "recalculate features for each edge..." << std::endl;
	int nhalfedges = arr->number_of_halfedges();
	Halfedge_iterator heit;
	for (heit = arr->halfedges_begin(); heit != arr->halfedges_end(); ++heit)
	{
		int idx = heit->data();
		std::cout << "processing: " << idx << "/" << nhalfedges << "\r";

		if (halfedge_traverse_flag[idx])
			continue;

		int twin_idx = heit->twin()->data();

		halfedge_traverse_flag[idx] = halfedge_traverse_flag[twin_idx] = true;

		halfedge_att_list_[idx].occupied = 0;
		halfedge_att_list_[idx].empty = 0;
		halfedge_att_list_[twin_idx].occupied = 0;
		halfedge_att_list_[twin_idx].empty = 0;

		Point_2 p = heit->source()->point();
		seg.sp[0] = p.x().to_double();
		seg.sp[1] = p.y().to_double();

		p = heit->target()->point();
		seg.ep[0] = p.x().to_double();
		seg.ep[1] = p.y().to_double();

		grid_intersect.getIntersectedGrids(seg, intersected_grids); //扫描线求交
		for (CellKeyMap::iterator it_intersected = intersected_grids.begin();
			it_intersected != intersected_grids.end();
			++it_intersected)
		{
			QuadtreeKey key_arg = it_intersected->first;

			CellKeyMap::iterator it_vm;
			it_vm = keymap_wall_.find(key_arg);
			if (it_vm != keymap_wall_.end())
			{//存在非墙面点
			 //				occupied = true;
				halfedge_att_list_[idx].occupied++;
				halfedge_att_list_[twin_idx].occupied++;
			}
			else
			{
				//				occupied = false;
				halfedge_att_list_[idx].empty++;
				halfedge_att_list_[twin_idx].empty++;
			}
		}
	}
	std::cout << "processing: " << nhalfedges << "/" << nhalfedges << std::endl;*/


	std::set<Halfedge_handle> removed_halfedges;  //需要删除的边集合,  不可重复
	std::set<Halfedge_handle>::iterator removeIter;

	std::set<Face_handle>::iterator fIter;
	for (fIter = face_list->begin(); fIter != face_list->end(); ++fIter)
	{
		Face_handle fit = *fIter;
		int fid = fit->data();
		std::cout << "face: " << fid;

		boost::unordered_map<int, EdgeList>	neighbor_edges;   //邻接face id, 邻接边链表
		boost::unordered_map<int, EdgeList>::iterator map_iter;

		Arrangement_2::Ccb_halfedge_circulator  circ = fit->outer_ccb();
		Arrangement_2::Ccb_halfedge_circulator  curr = circ;
		do
		{
			Halfedge_handle he = curr;
			assert(fid == he->face()->data());

			int nfid = he->twin()->face()->data();

			map_iter = neighbor_edges.find(nfid);
			if (map_iter != neighbor_edges.end())
			{
				map_iter->second.push_back(he);
			}
			else
			{
				EdgeList  elist;
				elist.push_back(he);
				neighbor_edges.insert(std::make_pair(nfid, elist));
			}

			curr = curr->next();

		} while (curr != circ);

		int mergeTo = -1;     //合并到哪个face
		float max_empty = -1;
		EdgeList::iterator eiter;
		for (map_iter = neighbor_edges.begin(); map_iter != neighbor_edges.end(); ++map_iter)
		{
			float emptyDeg = 0;
			for (eiter = map_iter->second.begin(); eiter != map_iter->second.end(); ++eiter)
			{
				Halfedge_handle he = *eiter;
				int eid = he->data();

				emptyDeg += halfedge_att_list_[eid].empty;
			}

			if (emptyDeg > max_empty)
			{
				mergeTo = map_iter->first;
				max_empty = emptyDeg;
			}
		}

		if (mergeTo == -1)
			continue;

		std::cout << "remove face " << fid << " merge to" << mergeTo << std::endl;

		map_iter = neighbor_edges.find(mergeTo);

		for (eiter = map_iter->second.begin(); eiter != map_iter->second.end(); ++eiter)
		{
			Halfedge_handle he = *eiter;
			removeIter = removed_halfedges.find(he);
			if (removeIter != removed_halfedges.end())
				continue;

			removeIter = removed_halfedges.find(he->twin());
			if (removeIter != removed_halfedges.end())
				continue;

			removed_halfedges.insert(he);

			std::cout << "remove edge: " << he->data() << std::endl;
			arr->remove_edge(he);
		}
	}
}

void
ipl::FloorMapReconstruction::mergeCollinearEdges(Arrangement_2 *arr)
{
// 	bool are_mergeable(Halfedge_const_handle e1, Halfedge_const_handle e2) const;

	Edge_iterator eit;
	Arrangement_2::Originating_curve_iterator     ocit;
	for (eit = arr->edges_begin(); eit != arr->edges_end(); ++eit)
	{
		if(arr->are_mergeable(eit, eit->next()))
		{
			arr->merge_edge(eit, eit->next());
			//continue;
		}
		
/*		Vertex_handle vit = eit->target();

		

		int deg = vit->degree();

// 		if (vit->degree() != 2)
// 			continue;

		Arrangement_2::Halfedge_around_vertex_circulator  circ = vit->incident_halfedges();
		Arrangement_2::Ccb_halfedge_circulator cch = circ;
		int nedge = 0;
		do 
		{
			nedge++;
		} while (++cch !=circ);

		if (nedge != 2)
		{
			continue;
		}

		cch = circ;
		++cch;
		
		int n1 = arr->number_of_originating_curves(cch);
		int n2 = arr->number_of_originating_curves(circ);
		if (n1 == 1 && n2 == 1)
		{
			if (arr->originating_curves_begin(cch) == arr->originating_curves_begin(circ))
			{
				arr->merge_edge(cch, circ);
			}
		}*/
	}
}

int 
ipl::FloorMapReconstruction::getOutterBoundary(OGRMultiPolygon &polygons, const double floor_hei/* =0 */)
{
	Face_handle unf = arr_.unbounded_face();

	polygons.empty();

	int fid = 0;
	//char fname[32];

	// Traverse the inner boundary (holes).
	Hole_iterator hit;
	for (hit = unf->holes_begin(); hit != unf->holes_end(); ++hit, ++fid)
	{
		//fid = hit->data();
// 		sprintf(fname, "polygon_%03d", fid);
// 
// 		OGRFeature *poFeature;
// 		poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
// 		poFeature->SetField(0, fname);
// 		poFeature->SetField(1, fid);

		OGRLinearRing ring;

		typename Arrangement_2::Ccb_halfedge_circulator cch = *hit;
		do {
			typename Arrangement_2::Face_handle inner_face = cch->twin()->face();
			if (inner_face == cch->face())
				continue;        // discard antennas

			int id = cch->face()->data();
			int innerfid = cch->twin()->face()->data();

			int eid = cch->data();
			int teid = cch->twin()->data();

			double x, y;
			x = cch->target()->point().x().to_double();
			y = cch->target()->point().y().to_double();
			ring.addPoint(x, y, floor_hei);

		} while (++cch != *hit);

		ring.closeRings();//首尾点重合形成闭合环 
		OGRPolygon poly;
		poly.addRing(&ring);

		OGRErr err = polygons.addGeometry(&poly);
	}

	return (0);
}

int 
ipl::FloorMapReconstruction::getAllPolygons(OGRMultiPolygon &polygons, const double floor_hei /* = 0 */)
{
	polygons.empty();

	PolygonList plist;
	repairLineArrangement_ring(&arr_, plist);

	Face_handle unf = arr_.unbounded_face();
	Hole_iterator hit;
	int fid = 0;
	char fname[32];

	PolygonList::iterator pit;
	for (pit = plist.begin(); pit != plist.end(); ++pit)
	{
		fid = pit->first;
		sprintf(fname, "polygon_%03d", fid);

// 		OGRFeature *poFeature;
// 		poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
// 		poFeature->SetField(0, fname);
// 		poFeature->SetField(1, fid);
// 
// 		if (face_flags_[fid])
// 			poFeature->SetField(2, bim::BoundaryTypeDesc[1]);
// 		else
// 			poFeature->SetField(2, bim::BoundaryTypeDesc[0]);

		OGRLinearRing ring;

		VertexIDList::iterator vit = pit->second.begin();
		do {
			int id = *vit;
			double x, y;
			x = points_[id].x().to_double();
			y = points_[id].y().to_double();
			ring.addPoint(x, y, floor_hei);
		} while (++vit != pit->second.end());
		ring.closeRings();//首尾点重合形成闭合环 
		OGRPolygon poly;
		poly.addRing(&ring);
		
		polygons.addGeometry(&poly);
	}

//	GDALClose(poDS);
	return (0);
}


