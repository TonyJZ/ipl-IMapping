#pragma once

#include "core/iplcore.h"
#include "geometry/GeometryDef.h"

//CGAL
#include <CGAL/Polygon_2.h>

//GDAL
#include <GDAL/gdal_alg.h>
#include <GDAL/gdal_priv.h>   //添加GDAL库函数的头文件
#include <GDAL/ogrsf_frmts.h>

namespace ipl {

	/** \brief The method implemented is based on "Simultaneous curve simplification" by Dyken et al.
	//It can simplify any set of polylines, open or closed, and possibly intersecting themselves or each other.
	* \param[in] orgPoly:  the original polyline
	* \param[out] dstPoly: the simplified polyline
	* \return
	*  * < 0 (-1) on error
	*  * == 0 on success
	*/
 	int simplify_polygon_CGAL(OGRPolygon *orgPoly, OGRPolygon *dstPoly, double costTh = 0.5);

	int simplify_polygon_CGAL(OGRMultiPolygon *orgPolys, OGRMultiPolygon *dstPolys, double costTh = 0.5);

	int simplify_polygon_CGAL(std::vector<OGRPolygon> &orgPolys, std::vector<OGRPolygon> &dstPolys, double costTh = 0.5);

// 	typename <typename kernel>
// 	int simplify_polygon_CGAL(CGAL::Polygon_2<Kernel> &orgPoly, CGAL::Polygon_2<Kernel> &dstPoly);
}


#include "geometry/polygon/impl/PolygonSimplification.hpp"
