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

	//��������߶δ���������
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

		//��ȡ���߶��ཻ�����񣬲���ȡ����ÿ�������еķֶ�
		int getIntersectedGridsAndLineSections(const ipl::geometry::LineSeg2D seg, CellKeyMap &grid_indices,
			 LineSectionKeyMap &lineSections);

		int getIntersectedGrids(const Polygon_2 &polygon, CellKeyMap &grid_indices);

		//��ȡ�������ཻ�����񣬲���ȡÿ��������ص����
		int getIntersectedGridsAndOverlappingArea(const Polygon_2 &polygon, CellKeyMap &grid_indices,
			IntersectionAreaMap &overlapMap);

	protected:
		//��ȡλ��ָ�����ο��ڵ��߶�
		int getInnerLineSection(double xs, double ys, double xe, double ye, 
			iplRECT<double> rect, ipl::geometry::LineSeg2D &line);

	private:
		Eigen::Vector4f bbmin_, bbmax_;  //���������ݷ�Χ
		int vNumX_, vNumY_/*, vNumZ_*/;
		Eigen::Vector4f leaf_size_;
		Eigen::Array4f inverse_leaf_size_;
	};

}

