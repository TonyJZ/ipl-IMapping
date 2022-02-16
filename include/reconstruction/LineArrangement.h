#pragma once
#include "core/iplcore.h"
#include "geometry/geometryDef.h"

//GDAL
#include <gdal/gdal_alg.h>
#include <gdal/gdal_priv.h>   
#include <gdal/ogrsf_frmts.h>

//Eigen
#include <Eigen/dense>

//cgal
#include <CGAL/Cartesian.h>
#include <CGAL/Exact_rational.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arr_extended_dcel.h>
//#include <CGAL/Arrangement_2.h>
#include <CGAL/Arrangement_with_history_2.h>
#include <CGAL/Arr_landmarks_point_location.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polygon_2.h>

namespace ipl
{
	class IPL_BASE_API LineArrangement
	{
	public:
		typedef CGAL::Cartesian<CGAL::Exact_rational>							Kernel;
		//typedef CGAL::Exact_predicates_inexact_constructions_kernel				Kernel;
		//typedef CGAL::Exact_predicates_exact_constructions_kernel				Kernel;
		typedef CGAL::Arr_segment_traits_2<Kernel>								Traits_2;
		typedef Traits_2::FT													FT;
		typedef Traits_2::Point_2												Point_2;
		typedef Traits_2::Curve_2												Segment_2;
		//typedef Traits_2::X_monotone_curve_2									Segment_2;
		typedef CGAL::Arr_extended_dcel<Traits_2, int, int, int>	Dcel;    //vertex id, edge id, face id
		typedef CGAL::Arrangement_with_history_2<Traits_2, Dcel>				Arrangement_2;
		typedef CGAL::Arr_landmarks_point_location<Arrangement_2>				Landmarks_pl;
		//typedef CGAL::Arr_extended_dcel_text_formatter<Arrangement_2>  Formatter;

		typedef CGAL::Polyhedron_3<Kernel>										Polyhedron;
		typedef Polyhedron::Point_3												Point_3;
		typedef CGAL::Polygon_2<Kernel>											Polygon_2;

		typedef Arrangement_2::Curve_const_handle						Curve_const_handle;
		typedef Arrangement_2::Vertex_const_handle						Vertex_const_handle;
		typedef Arrangement_2::Halfedge_const_handle					Halfedge_const_handle;
		typedef Arrangement_2::Face_const_handle						Face_const_handle;

		typedef Arrangement_2::Curve_handle								Curve_handle;
		typedef Arrangement_2::Vertex_handle							Vertex_handle;
		typedef Arrangement_2::Halfedge_handle							Halfedge_handle;
		typedef Arrangement_2::Face_handle								Face_handle;

		typedef Arrangement_2::Curve_iterator		Curve_iterator;
		typedef Arrangement_2::Edge_iterator		Edge_iterator;
		typedef Arrangement_2::Halfedge_iterator	Halfedge_iterator;
		typedef Arrangement_2::Face_iterator		Face_iterator;
		typedef Arrangement_2::Hole_iterator		Hole_iterator;

	public:
		LineArrangement();
		~LineArrangement();

		int Initialize(const OGRMultiLineString *line_segs);

		//int Initialize();

		int insertLines(std::vector<geometry::LineSeg2D> &lineSegs);

		Arrangement_2* getArrangment();

		int getOutterBoundary(std::vector<OGRPolygon> &polygons, const double floor_hei = 0);
		int getAllPolygons(std::vector<OGRPolygon> &polygons, const double floor_hei = 0);

	protected:



	private:

		Arrangement_2    arr_;

	};


}

