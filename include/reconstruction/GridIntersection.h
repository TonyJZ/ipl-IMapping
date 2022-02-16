#pragma once
#include "core/iplcore.h"
#include "geometry/geometryDef.h"
#include "spatialindexing/SpatialindexDef.h"

//cgal
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Exact_rational.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/squared_distance_2.h>

namespace ipl
{
	typedef boost::unordered_map<ipl::QuadtreeKey, ipl::geometry::LineSeg2D> LineSectionKeyMap;
	typedef boost::unordered_map<ipl::QuadtreeKey, double> IntersectionAreaMap;

	//计算给定线段穿过的网格
	class IPL_BASE_API GridIntersection
	{
	public:
		//typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
		//typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
		typedef CGAL::Cartesian<CGAL::Exact_rational>				Kernel;
		typedef Kernel::Point_2										Point_2;
		typedef CGAL::Polygon_2<Kernel>								Polygon_2;
		typedef CGAL::Polygon_with_holes_2<Kernel>					Polygon_with_holes_2;

		GridIntersection()
		{

		}

		virtual ~GridIntersection()
		{

		}

		int InitialGrid(const double bbmin[3], const double bbmax[3], const float size_x, const float size_y);

		int InitialGrid(const Eigen::Vector4f bbmin, const Eigen::Vector4f bbmax, 
			const float size_x, const float size_y);

		int getIntersectedGrids(const ipl::geometry::LineSeg2D seg, CellKeyMap &grid_indices);

		//提取与线段相交的网格，并提取落入每个网格中的分段
		int getIntersectedGridsAndLineSections(const ipl::geometry::LineSeg2D seg, CellKeyMap &grid_indices,
			 LineSectionKeyMap &lineSections);

		int getIntersectedGrids(const Polygon_2 &polygon, CellKeyMap &grid_indices);

		//提取与多边形相交的网格，并提取每个网格的重叠面积
		int getIntersectedGridsAndOverlappingArea(const Polygon_2 &polygon, CellKeyMap &grid_indices,
			IntersectionAreaMap &overlapMap);

	protected:
		//提取位于指定矩形块内的线段
		int getInnerLineSection(double xs, double ys, double xe, double ye, 
			iplRECT<double> rect, ipl::geometry::LineSeg2D &line);

	private:
		Eigen::Vector4f bbmin_, bbmax_;  //格网的数据范围
		int vNumX_, vNumY_/*, vNumZ_*/;
		Eigen::Vector4f leaf_size_;
		Eigen::Array4f inverse_leaf_size_;
	};

}

