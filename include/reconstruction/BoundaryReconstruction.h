#pragma once
#include "core/iplcore.h"
#include "commonAPIs/iplutility.h"
#include "geometry/GeometryDef.h"
#include "spatialindexing/SpatialindexDef.h"
#include "fitting/geoModelDef.h"
#include "reconstruction/reconstructionDef.h"

//cgal
// #include <CGAL/Cartesian.h>
// #include <CGAL/Exact_rational.h>
// #include <CGAL/Polygon_2.h>
// #include <CGAL/Arr_segment_traits_2.h>
// #include <CGAL/Arr_extended_dcel.h>
// //#include <CGAL/Arrangement_2.h>
// #include <CGAL/Arrangement_with_history_2.h>
// #include <CGAL/IO/Arr_text_formatter.h>
// #include <CGAL/IO/Arr_iostream.h>
// //#include <CGAL/Arr_simple_point_location.h>
// #include <CGAL/Arr_landmarks_point_location.h>
// #include <CGAL/Polyhedron_3.h>

//GDAL
#include <GDAL/gdal_alg.h>
#include <GDAL/gdal_priv.h>   //添加GDAL库函数的头文件
#include <GDAL/ogrsf_frmts.h>


namespace ipl
{
	template <typename PointT>
	class BoundaryReconstruction : public iplPointCluster<PointT>
	{
	public:
// 		typedef CGAL::Cartesian<CGAL::Exact_rational>							Kernel;
// 		typedef CGAL::Arr_segment_traits_2<Kernel>								Traits_2;
// 		typedef Traits_2::Point_2												Point_2;
// 		typedef Traits_2::Curve_2												Segment_2;
// 		typedef CGAL::Polygon_2<Kernel>											Polygon_2;
// 		typedef CGAL::Arr_extended_dcel<Traits_2, int, int, int>	Dcel;    //vertex id, edge id, face id
// 		typedef CGAL::Arrangement_with_history_2<Traits_2, Dcel>				Arrangement_2;
// 		typedef CGAL::Arr_landmarks_point_location<Arrangement_2>				Landmarks_pl;
// 		//typedef CGAL::Arr_extended_dcel_text_formatter<Arrangement_2>  Formatter;
// 
// 		typedef Arrangement_2::Curve_const_handle						Curve_const_handle;
// 		typedef Arrangement_2::Vertex_const_handle						Vertex_const_handle;
// 		typedef Arrangement_2::Halfedge_const_handle					Halfedge_const_handle;
// 		typedef Arrangement_2::Face_const_handle						Face_const_handle;
// 
// 		typedef Arrangement_2::Curve_handle								Curve_handle;
// 		typedef Arrangement_2::Vertex_handle							Vertex_handle;
// 		typedef Arrangement_2::Halfedge_handle							Halfedge_handle;
// 		typedef Arrangement_2::Face_handle								Face_handle;
// 
// 		typedef Arrangement_2::Curve_iterator		Curve_iterator;
// 		typedef Arrangement_2::Edge_iterator		Edge_iterator;
// 		typedef Arrangement_2::Halfedge_iterator	Halfedge_iterator;
// 		typedef Arrangement_2::Face_iterator		Face_iterator;
// 		typedef Arrangement_2::Hole_iterator		Hole_iterator;

	public:
		BoundaryReconstruction();
		~BoundaryReconstruction();

		//用于控制检测剖面高度, 调节建筑物的完整性 ratio [0, 1], 绝对楼高的比例因子
		void setZProfileRatio(float ratio)
		{
			if (ratio < 0 || ratio > 1)
			{
				std::cout << "Z profile ration must between [0, 1]." << std::endl;
			}
			ZPrf_ratio_ = ratio;
		}

		int extractAlphaShape(float gsize, float simTh);

		int detectWalls(float minWallHei);

		int detectPlanes(float minWallHei, float maxRoofHei);

		//临时方案
		int getRoofFloorHeight(float vsize, float &roofHei, float &floorHei);

		void setIntermediateFolder(const std::string folder)
		{
			IntermediateFolder_ = folder;

			AShapeFolder_ = IntermediateFolder_ + "/" + rec::FolderName_AShape;
			WallsFolder_ = IntermediateFolder_ + "/" + rec::FolderName_Walls;
			RoofsFolder_ = IntermediateFolder_ + "/" + rec::FolderName_Roofs;

			create_folder(AShapeFolder_);
			create_folder(WallsFolder_);
			create_folder(RoofsFolder_);
		}

	protected:
		int exportPolygons(const std::string polyname, std::vector<OGRPolygon> &Poly, 
			float roof_hei, float floor_hei, float profile_hei);

		int exportPolylines(const std::string polyname, OGRMultiLineString *lines, 
			float roof_hei, float floor_hei, float profile_hei);

		int exportWallSegments(const std::string dir);

		int exportRoofSegments(const std::string dir);

	private:
		std::string IntermediateFolder_;
		std::string AShapeFolder_;
		std::string WallsFolder_;
		std::string RoofsFolder_;

		ref_ptr<CellKeyMap>   gridMap_;

		ref_ptr<iplPointCloud <PointT> > sorted_wall_cloud_;
		std::vector<std::vector<int> > wall_inliers_;
		std::vector<ipl::geoModel::geoModelInfo> wall_coeffs_;

		ref_ptr<iplPointCloud <PointT> > sorted_roof_cloud_;
		std::vector<std::vector<int> > roof_inliers_;
		std::vector<ipl::geoModel::geoModelInfo> roof_coeffs_;

	private:
		float ZPrf_ratio_;   //用于提取boundary的起算高度
	};

}

#include "reconstruction/impl/BoundaryReconstruction.hpp"
