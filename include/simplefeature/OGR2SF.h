#pragma once

#include <GDAL/ogr_api.h>
#include "simplefeature/interface/ISFGeometry.h"


namespace ipl
{
	IGeoObject *CreateGeometry(OGRGeometryH poGeometry);

	ISFPoint *CreatePoint(OGRGeometryH poPoint);
	ISFLineString *CreateLineString(OGRGeometryH poLineString);
	ISFLineRing *CreateLinearRing(OGRGeometryH poRing);
	ISFPolygon *CreatePolgon(OGRGeometryH poPolygon);

	ISFGeometryCollection *CreateGeoCollection(OGRGeometryH poGeoC);
	ISFMultiPoint *CreateMultiPoint(OGRGeometryH poMultiPoint);
	ISFMultiLineString *CreateMultiLineString(OGRGeometryH poMultiLine);
	ISFMultiPolygon *CreateMultiPolygon(OGRGeometryH poMultiPolygon);
}

#include "simplefeature/OGR2SF.hpp"
