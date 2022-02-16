#pragma once

#include "OGR2SF.h"
#include "simplefeature/interface/ISFService.h"

//extern ipl::ISFService *g_osfService;

namespace ipl
{
	ISFPoint *CreatePoint(OGRGeometryH poOgrPoint)
	{
		if (NULL == poOgrPoint)
			return NULL;

		ISFPoint* poGeometry = getSFService()->createPoint();

		poGeometry->x = OGR_G_GetX(poOgrPoint, 0);
		poGeometry->y = OGR_G_GetY(poOgrPoint, 0);
		poGeometry->z = OGR_G_GetZ(poOgrPoint, 0);

		return poGeometry;
	}

	ISFLineString *CreateLineString(OGRGeometryH poOgrLineString)
	{
		if (NULL == poOgrLineString)
			return NULL;

		ISFLineString* poGeometry = getSFService()->createLineString();

		int numOfPoints = OGR_G_GetPointCount(poOgrLineString);

		poGeometry->setNumPoints(numOfPoints);

		iplPOINT3D point;
		for (int i = 0; i < numOfPoints; i++)
		{
			OGR_G_GetPoint(poOgrLineString, i, &point.X, &point.Y, &point.Z);
			poGeometry->setPoint(i, point);
		}

		return poGeometry;
	}

	ISFLineRing *CreateLinearRing(OGRGeometryH poOgrRing)
	{
		if (NULL == poOgrRing)
			return NULL;

		ISFLineRing* poGeometry = getSFService()->createLinearRing();

		int numOfPoints = OGR_G_GetPointCount(poOgrRing);

		poGeometry->setNumPoints(numOfPoints);

		iplPOINT3D point;

		for (int i = 0; i < numOfPoints; i++)
		{
			OGR_G_GetPoint(poOgrRing, i, &point.X, &point.Y, &point.Z);

			poGeometry->setPoint(i, point);
		}
		return poGeometry;
	}

	ISFPolygon *CreatePolgon(OGRGeometryH poOgrPolygon)
	{
		if (NULL == poOgrPolygon)
			return NULL;

		ISFPolygon *poGeometry = getSFService()->createPolygon();

		//OGRLinearRing *poOgrLineRing = poOgrPolygon->getExteriorRing();

		OGRGeometryH poOgrLineRing = OGR_G_GetGeometryRef(poOgrPolygon, 0);

		poGeometry->addRingDirectly(ref_ptr<ISFLineRing>(CreateLinearRing(poOgrLineRing)));

		int nRings = OGR_G_GetGeometryCount(poOgrPolygon);

		for (int i = 1; i < nRings; i++)
		{
			poOgrLineRing = OGR_G_GetGeometryRef(poOgrPolygon, i);

			poGeometry->addRingDirectly(ref_ptr<ISFLineRing>(CreateLinearRing(poOgrLineRing)));
		}

		return poGeometry;
	}

	ISFGeometryCollection *CreateGeoCollection(OGRGeometryH poOgrGeoC)
	{
		if (NULL == poOgrGeoC)
			return NULL;

		ISFGeometryCollection *poGeometry = getSFService()->createGeometryCollection();

		int nSubs = OGR_G_GetGeometryCount(poOgrGeoC);

		OGRGeometryH poOgrGeometry;
		for (int i = 0; i < nSubs; i++)
		{
			poOgrGeometry = OGR_G_GetGeometryRef(poOgrGeoC, i);

			poGeometry->addGeometryDirectly(ref_ptr<IGeoObject>(CreateGeometry(poOgrGeometry)));
		}

		return poGeometry;
	}

	ISFMultiPoint *CreateMultiPoint(OGRGeometryH poOgrMultiPoint)
	{
		if (NULL == poOgrMultiPoint)
			return NULL;

		ISFMultiPoint *poGeometry = getSFService()->createMultiPoint();

		int nSubs = OGR_G_GetGeometryCount(poOgrMultiPoint);

		OGRGeometryH poOgrGeometry;
		for (int i = 0; i < nSubs; i++)
		{
			//poOgrGeometry = poOgrMultiPoint->getGeometryRef(i);

			poOgrGeometry = OGR_G_GetGeometryRef(poOgrMultiPoint, i);

			poGeometry->addGeometryDirectly(ref_ptr<IGeoObject>(CreateGeometry(poOgrGeometry)));
		}

		return poGeometry;
	}

	ISFMultiLineString *CreateMultiLineString(OGRGeometryH poOgrMultiLine)
	{
		if (NULL == poOgrMultiLine)
			return NULL;

		ISFMultiLineString *poGeometry = getSFService()->createMultiLineString();

		int nSubs = OGR_G_GetGeometryCount(poOgrMultiLine);

		OGRGeometryH poOgrGeometry;
		for (int i = 0; i < nSubs; i++)
		{
			//poOgrGeometry = poOgrMultiLine->getGeometryRef(i);

			poOgrGeometry = OGR_G_GetGeometryRef(poOgrMultiLine, i);

			poGeometry->addGeometryDirectly(ref_ptr<IGeoObject>(CreateGeometry(poOgrGeometry)));
		}

		return poGeometry;
	}

	ISFMultiPolygon *CreateMultiPolygon(OGRGeometryH poOgrMultiPolygon)
	{
		if (NULL == poOgrMultiPolygon)
			return NULL;

		ISFMultiPolygon *poGeometry = getSFService()->createMultiPolygon();

		int nSubs = OGR_G_GetGeometryCount(poOgrMultiPolygon);

		OGRGeometryH poOgrGeometry;
		for (int i = 0; i < nSubs; i++)
		{
			//poOgrGeometry = poOgrMultiPolygon->getGeometryRef(i);

			poOgrGeometry = OGR_G_GetGeometryRef(poOgrMultiPolygon, i);

			poGeometry->addGeometryDirectly(ref_ptr<IGeoObject>(CreateGeometry(poOgrGeometry)));
		}

		return poGeometry;
	}

	IGeoObject *CreateGeometry(OGRGeometryH poOgrGeometry)
	{
		if (NULL == poOgrGeometry)
			return NULL;

		switch (static_cast<IPL_wkbGeometryType> (OGR_G_GetGeometryType(poOgrGeometry)))
		{
		case IPL_wkbUnknown:
			return NULL;

		case IPL_wkbPoint:
		case IPL_wkbPoint25D:
		{
			return CreatePoint(poOgrGeometry);
		}

		case IPL_wkbLineString:
		case IPL_wkbLineString25D:
		{
			return CreateLineString(poOgrGeometry);
		}

		case IPL_wkbLinearRing:
		{
			return CreateLinearRing(poOgrGeometry);
		}

		case IPL_wkbPolygon:
		case IPL_wkbPolygon25D:
		{
			return CreatePolgon(poOgrGeometry);
		}

		case IPL_wkbMultiPoint:
		case IPL_wkbMultiPoint25D:
		{
			return CreateMultiPoint(poOgrGeometry);
		}

		case IPL_wkbMultiLineString:
		case IPL_wkbMultiLineString25D:
		{
			return CreateMultiLineString(poOgrGeometry);
		}

		case IPL_wkbMultiPolygon:
		case IPL_wkbMultiPolygon25D:
		{
			return CreateMultiPolygon(poOgrGeometry);
		}

		case IPL_wkbGeometryCollection:
		case IPL_wkbGeometryCollection25D:
		{
			return CreateGeoCollection(poOgrGeometry);
		}

		case IPL_wkbNone:
			return NULL;

		default:
		{
			return NULL;
		}
		}
	}

}

