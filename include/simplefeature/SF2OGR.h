#pragma once

#include <GDAL/ogr_api.h>
#include "simplefeature/interface/ISFGeometry.h"
#include "simplefeature/interface/ISFFeature.h"

namespace ipl
{
	OGRGeometryH CreateOgrGeom(IGeoObject *poGeometry);

	OGRGeometryH CreateOgrPoint(ISFPoint *poPoint);
	OGRGeometryH CreateOgrLineString(ISFLineString *poLine);
	OGRGeometryH CreateOgrLinearR(ISFLineRing *poLinearR);
	OGRGeometryH CreateOgrPolygon(ISFPolygon *poPolygon);
	OGRGeometryH CreateOgrMultiPoint(ISFMultiPoint *poMultiPoint);
	OGRGeometryH CreateOgrMultiLineString(ISFMultiLineString *poMultiLine);
	OGRGeometryH CreateOgrMultiPolygon(ISFMultiPolygon *poMultiPolygon);
	OGRGeometryH CreateOgrGeoCollection(ISFGeometryCollection *poGeoc);

	OGRFeatureDefnH CreateOgrFeatureDefn(ISFFeatureDefn *poFDefn);
	OGRFieldDefnH CreateOgrFieldDefn(ISFFieldDefn *poFieldDefn);

}

#include "simplefeature/SF2OGR.hpp"
