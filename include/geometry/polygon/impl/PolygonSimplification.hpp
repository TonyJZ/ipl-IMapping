#pragma once

#include "geometry/polygon/PolygonSimplification.h"

//CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polyline_simplification_2/simplify.h>

//GEOS
// #include <GEOS/geos/simplify/DouglasPeuckerSimplifier.h>
// #include <GEOS/geos/simplify/TopologyPreservingSimplifier.h>



int ipl::simplify_polygon_CGAL(OGRPolygon *orgPoly, OGRPolygon *dstPoly, double costTh)
{
	namespace PS = CGAL::Polyline_simplification_2;
	typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
	typedef CGAL::Polygon_2<K>                   Polygon_2;
//	typedef PS::Stop_below_count_threshold		 Stop;
//	typedef PS::Stop_below_count_ratio_threshold Stop;
	typedef PS::Stop_above_cost_threshold		 Stop;
//	typedef PS::Squared_distance_cost            Cost;
//	typedef PS::Scaled_squared_distance_cost     Cost;
	typedef PS::Hybrid_squared_distance_cost<K::FT>     Cost;

	 
	dstPoly->empty();
	for (int iR = -1; iR < orgPoly->getNumInteriorRings(); ++iR)
	{
		OGRLinearRing *orgRing = NULL;
		if (iR == -1)
			orgRing = orgPoly->getExteriorRing();
		else
			orgRing = orgPoly->getInteriorRing(iR);

		int ptNum = orgRing->getNumPoints();
		if (ptNum < 3)
		{
			*dstPoly = *orgPoly;
			return (0);
		}

		std::cout << "-----------------org polygon------------" << std::endl;
		Polygon_2 cpolygon;		//CGAL polygon
		for (int i = 1; i < ptNum; i++)
		{
			OGRPoint pt;
			orgRing->getPoint(i, &pt);
			std::cout << pt.getX() << ", " << pt.getY() << std::endl;

			Polygon_2::Point_2 p(pt.getX(), pt.getY());
			cpolygon.push_back(p);
		}

		Cost cost(costTh);
		Polygon_2 simpPolygon = PS::simplify(cpolygon, cost, Stop(costTh));

		std::cout << "-----------------dst polygon------------" << std::endl;
		OGRLinearRing ring;
		Polygon_2::Vertex_iterator  viter;
		for (viter = simpPolygon.vertices_begin(); viter != simpPolygon.vertices_end(); ++viter)
		{
			ring.addPoint(viter->x(), viter->y());
			std::cout << viter->x() << ", " << viter->y() << std::endl;
		}

		
		ring.closeRings();//首尾点重合形成闭合环 
		dstPoly->addRing(&ring);
	}
	
	return 0;
}

int ipl::simplify_polygon_CGAL(OGRMultiPolygon *orgPolys, OGRMultiPolygon *dstPolys, double costTh)
{
	int nGeoCount = orgPolys->getNumGeometries();
	int iPoly = 0;

	dstPolys->empty();
	const OGRGeometry *poPolyGeometry;
	for (iPoly = 0; iPoly < nGeoCount; iPoly++)
	{
		poPolyGeometry = orgPolys->getGeometryRef(iPoly);

		OGRPolygon *poPoly = (OGRPolygon *)poPolyGeometry;
		

		OGRPolygon dstpoPoly;

		simplify_polygon_CGAL(poPoly, &dstpoPoly, costTh);

		dstPolys->addGeometry(&dstpoPoly);
	}

	return 0;
}

int ipl::simplify_polygon_CGAL(std::vector<OGRPolygon> &orgPolys, std::vector<OGRPolygon> &dstPolys, double costTh)
{
	int nGeoCount = orgPolys.size();
	int iPoly = 0;

	dstPolys.clear();
	const OGRGeometry *poPolyGeometry;
	for (iPoly = 0; iPoly < nGeoCount; iPoly++)
	{
		OGRPolygon dstpoPoly;

		simplify_polygon_CGAL(&(orgPolys[iPoly]), &dstpoPoly, costTh);

		dstPolys.push_back(dstpoPoly);
	}

	return 0;
}
