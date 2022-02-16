#include "SF2OGR.h"
#include "simplefeature/interface/ISFService.h"

namespace ipl
{
	OGRGeometryH CreateOgrGeom(IGeoObject *poGeometry)
	{
		if (NULL == poGeometry)
			return NULL;

		OGRwkbGeometryType type = (OGRwkbGeometryType)poGeometry->getGeometryType();

		//OGRGeometryH ogrGeog;

		switch (type)
		{
		case wkbUnknown:
			return NULL;

		case wkbPoint:
		case wkbPoint25D:
		{
			ISFPoint* poPoint = (ISFPoint *)poGeometry;

			return CreateOgrPoint(poPoint);
		}

		case wkbLineString:
		case wkbLineString25D:
		{
			ISFLineString *poLineString = (ISFLineString *)poGeometry;
			return CreateOgrLineString(poLineString);
		}

		case wkbLinearRing:
		{
			ISFLineRing *poLineString = (ISFLineRing *)poGeometry;
			return CreateOgrLinearR(poLineString);
		}

		case wkbPolygon:
		case wkbPolygon25D:
		{
			ISFPolygon *poPolygon = (ISFPolygon *)poGeometry;
			return CreateOgrPolygon(poPolygon);
		}

		case wkbMultiPoint:
		case wkbMultiPoint25D:
		{
			ISFMultiPoint *poMultiPoint = (ISFMultiPoint *)poGeometry;
			return CreateOgrMultiPoint(poMultiPoint);
		}

		case wkbMultiLineString:
		case wkbMultiLineString25D:
		{
			ISFMultiLineString *poMultiLine = (ISFMultiLineString *)poGeometry;
			return CreateOgrMultiLineString(poMultiLine);
		}

		case wkbMultiPolygon:
		case wkbMultiPolygon25D:
		{
			ISFMultiPolygon *poMultiPolygon = (ISFMultiPolygon *)poGeometry;
			return CreateOgrMultiPolygon(poMultiPolygon);
		}

		case wkbGeometryCollection:
		case wkbGeometryCollection25D:
		{
			ISFGeometryCollection *poGeoC = (ISFGeometryCollection *)poGeometry;
			return CreateOgrGeoCollection(poGeoC);
		}

		case wkbNone:
			return NULL;

		default:
		{
			return NULL;
		}
		}
	}

	OGRGeometryH CreateOgrPoint(ISFPoint *poPoint)
	{
		if (NULL == poPoint)
			return NULL;

		OGRGeometryH poOgrPoint;

		poOgrPoint = OGR_G_CreateGeometry((OGRwkbGeometryType)(poPoint->getGeometryType()));

		OGR_G_SetPoint(poOgrPoint, 0, poPoint->x, poPoint->y, poPoint->z);

		return poOgrPoint;

	}

	OGRGeometryH CreateOgrLineString(ISFLineString *poLine)
	{
		if (NULL == poLine)
			return NULL;

		OGRGeometryH poOgrLineString;

		//poOgrLineString = (OGRLineString *)OGRGeometryFactory::createGeometry((OGRwkbGeometryType)(poLine->getGeometryType()));
		poOgrLineString = OGR_G_CreateGeometry((OGRwkbGeometryType)(poLine->getGeometryType()));

		int numOfPoints = poLine->getNumPoints();

		// poOgrLineString->setNumPoints(numOfPoints);

		iplPOINT3D point;
		for (int i = numOfPoints - 1; i >= 0; i--)
		{
			poLine->getPoint(i, &point);
			//poOgrLineString->setPoint(i,point.X,point.Y,point.Z);

			OGR_G_SetPoint(poOgrLineString, i, point.X, point.Y, point.Z);

		}
		return poOgrLineString;
	}

	OGRGeometryH CreateOgrLinearR(ISFLineRing *poLinearR)
	{
		if (NULL == poLinearR)
			return NULL;

		OGRGeometryH poOgrLinearRing;

		//poOgrLinearRing = (OGRLinearRing *)OGRGeometryFactory::createGeometry((OGRwkbGeometryType)(poLinearR->getGeometryType()));
		poOgrLinearRing = OGR_G_CreateGeometry((OGRwkbGeometryType)(poLinearR->getGeometryType()));

		int numOfPoints = poLinearR->getNumPoints();

		//poOgrLinearRing->setNumPoints(numOfPoints);

		iplPOINT3D point;
		for (int i = numOfPoints - 1; i >= 0; i--)
		{
			poLinearR->getPoint(i, &point);

			//poOgrLinearRing->setPoint(i,point.X,point.Y,point.Z);

			OGR_G_SetPoint(poOgrLinearRing, i, point.X, point.Y, point.Z);

		}
		return poOgrLinearRing;
	}

	OGRGeometryH CreateOgrPolygon(ISFPolygon *poPolygon)
	{
		if (NULL == poPolygon)
			return NULL;

		OGRGeometryH poOgrPolygon;
		//poOgrPolygon = (OGRPolygon *)OGRGeometryFactory::createGeometry((OGRwkbGeometryType)(poPolygon->getGeometryType()));

		poOgrPolygon = OGR_G_CreateGeometry((OGRwkbGeometryType)(poPolygon->getGeometryType()));

		ref_ptr<ISFLineRing> poLinearR = poPolygon->getExteriorRing();

		OGRGeometryH poOgrLineRing = CreateOgrLinearR(poLinearR.get());

		//poOgrPolygon->addRingDirectly( poOgrLineRing );

		assert(OGRERR_NONE == OGR_G_AddGeometryDirectly(poOgrPolygon, poOgrLineRing));

		for (int i = 0; i< poPolygon->getNumInteriorRings(); i++)
		{
			poLinearR = poPolygon->getInteriorRing(i);

			poOgrLineRing = CreateOgrLinearR(poLinearR.get());
			//poOgrLineRing = OGR_G_CreateGeometry((OGRwkbGeometryType)(poLinearR->getGeometryType()));

			//poOgrPolygon->addRingDirectly(poOgrLineRing);
			assert(OGRERR_NONE == OGR_G_AddGeometryDirectly(poOgrPolygon, poOgrLineRing));
		}

		return poOgrPolygon;
	}

	OGRGeometryH CreateOgrGeoCollection(ISFGeometryCollection *poGeoc)
	{
		if (NULL == poGeoc)
			return NULL;

		OGRGeometryH poOgrGeoc;
		//poOgrGeoc = (OGRGeometryCollection *)OGRGeometryFactory::createGeometry((OGRwkbGeometryType)(poGeoc->getGeometryType()));

		poOgrGeoc = OGR_G_CreateGeometry((OGRwkbGeometryType)(poGeoc->getGeometryType()));

		int numOfGeom = poGeoc->getNumGeometries();

		for (int i = numOfGeom - 1; i >= 0; i--)
		{
			IGeoObject *poGeom = poGeoc->getGeometryRef(i).get();

			OGRGeometryH poOgrGeom = CreateOgrGeom(poGeom);

			//poOgrGeoc->addGeometryDirectly(poOgrGeom);

			assert(OGRERR_NONE == OGR_G_AddGeometryDirectly(poOgrGeoc, poOgrGeom));
			//delete poOgrGeom;
		}

		return poOgrGeoc;
	}

	OGRGeometryH CreateOgrMultiPoint(ISFMultiPoint *poMultiPoint)
	{
		if (NULL == poMultiPoint)
			return NULL;

		OGRGeometryH poOgrMultiPoint;
		//poOgrMultiPoint = (OGRMultiPoint *)OGRGeometryFactory::createGeometry((OGRwkbGeometryType)(poMultiPoint->getGeometryType()));

		poOgrMultiPoint = OGR_G_CreateGeometry((OGRwkbGeometryType)(poMultiPoint->getGeometryType()));

		int numOfGeom = poMultiPoint->getNumGeometries();

		for (int i = numOfGeom - 1; i >= 0; i--)
		{
			ISFPoint *poGeom = (ISFPoint *)poMultiPoint->getGeometryRef(i).get();

			OGRGeometryH poOgrGeom = CreateOgrPoint(poGeom);
			//poOgrMultiPoint->addGeometryDirectly(poOgrGeom);

			assert(OGRERR_NONE == OGR_G_AddGeometryDirectly(poOgrMultiPoint, poOgrGeom));
		}

		return poOgrMultiPoint;
	}

	OGRGeometryH CreateOgrMultiLineString(ISFMultiLineString *poMultiLine)
	{
		if (NULL == poMultiLine)
			return NULL;

		OGRGeometryH poOgrLineString;
		//poOgrLineString = (OGRGeometryH )OGRGeometryFactory::createGeometry((OGRwkbGeometryType)(poMultiLine->getGeometryType()));

		poOgrLineString = OGR_G_CreateGeometry((OGRwkbGeometryType)(poMultiLine->getGeometryType()));

		int numOfGeom = poMultiLine->getNumGeometries();

		for (int i = numOfGeom - 1; i >= 0; i--)
		{
			ISFLineString *poGeom = (ISFLineString *)poMultiLine->getGeometryRef(i).get();

			OGRGeometryH poOgrGeom = CreateOgrLineString(poGeom);

			//poOgrLineString->addGeometryDirectly(poOgrGeom);

			assert(OGRERR_NONE == OGR_G_AddGeometryDirectly(poOgrLineString, poOgrGeom));

			//delete poOgrGeom;
		}

		return poOgrLineString;

	}

	OGRGeometryH CreateOgrMultiPolygon(ISFMultiPolygon *poMultiPolygon)
	{
		if (NULL == poMultiPolygon)
			return NULL;

		OGRGeometryH poOgrMultiPolygon;
		//poOgrMultiPolygon = (OGRGeometryH )OGRGeometryFactory::createGeometry((OGRwkbGeometryType)(poMultiPolygon->getGeometryType()));

		poOgrMultiPolygon = OGR_G_CreateGeometry((OGRwkbGeometryType)(poMultiPolygon->getGeometryType()));

		int numOfGeom = poMultiPolygon->getNumGeometries();

		for (int i = numOfGeom - 1; i >= 0; i--)
		{
			ISFPolygon *poGeom = (ISFPolygon *)poMultiPolygon->getGeometryRef(i).get();

			OGRGeometryH poOgrGeom = CreateOgrPolygon(poGeom);

			//poOgrMultiPolygon->addGeometryDirectly(poOgrGeom);
			assert(OGRERR_NONE == OGR_G_AddGeometryDirectly(poOgrMultiPolygon, poOgrGeom));
		}

		return poOgrMultiPolygon;
	}

	// 

	/* OGRFieldDefn */
	// 
	// OGRFieldDefnH CPL_DLL OGR_Fld_Create( const char *, OGRFieldType );
	// void   CPL_DLL OGR_Fld_Destroy( OGRFieldDefnH );
	// 
	// void   CPL_DLL OGR_Fld_SetName( OGRFieldDefnH, const char * );
	// const char CPL_DLL *OGR_Fld_GetNameRef( OGRFieldDefnH );
	// OGRFieldType CPL_DLL OGR_Fld_GetType( OGRFieldDefnH );
	// void   CPL_DLL OGR_Fld_SetType( OGRFieldDefnH, OGRFieldType );
	// OGRJustification CPL_DLL OGR_Fld_GetJustify( OGRFieldDefnH );
	// void   CPL_DLL OGR_Fld_SetJustify( OGRFieldDefnH, OGRJustification );
	// int    CPL_DLL OGR_Fld_GetWidth( OGRFieldDefnH );
	// void   CPL_DLL OGR_Fld_SetWidth( OGRFieldDefnH, int );
	// int    CPL_DLL OGR_Fld_GetPrecision( OGRFieldDefnH );
	// void   CPL_DLL OGR_Fld_SetPrecision( OGRFieldDefnH, int );
	// void   CPL_DLL OGR_Fld_Set( OGRFieldDefnH, const char *, OGRFieldType, 
	// 						   int, int, OGRJustification );
	// 
	// const char CPL_DLL *OGR_GetFieldTypeName( OGRFieldType );

	OGRFieldDefnH CreateOgrFieldDefn(ISFFieldDefn *poFieldDefn)
	{
		if (NULL == poFieldDefn)
			return NULL;

		OGRFieldDefnH poOgrFieldDefn;

		//poOgrFieldDefn = new OGRFieldDefn(poFieldDefn->GetNameRef(), OGRFieldType(poFieldDefn->GetType()));

		poOgrFieldDefn = OGR_FD_Create(poFieldDefn->GetNameRef());

		//OGR_Fld_SetName( poOgrFieldDefn,  poFieldDefn->GetNameRef() );

		OGR_Fld_SetType(poOgrFieldDefn, (OGRFieldType)poFieldDefn->GetType());
		OGR_Fld_SetWidth(poOgrFieldDefn, poFieldDefn->GetWidth());
		OGR_Fld_SetPrecision(poOgrFieldDefn, poFieldDefn->GetPrecision());
		OGR_Fld_SetJustify(poOgrFieldDefn, (OGRJustification)poFieldDefn->GetJustify());

		return poOgrFieldDefn;
	}


	// OGRFeatureDefnH CPL_DLL OGR_FD_Create( const char * );
	// void   CPL_DLL OGR_FD_Destroy( OGRFeatureDefnH );
	// void   CPL_DLL OGR_FD_Release( OGRFeatureDefnH );
	// const char CPL_DLL *OGR_FD_GetName( OGRFeatureDefnH );
	// int    CPL_DLL OGR_FD_GetFieldCount( OGRFeatureDefnH );
	// OGRFieldDefnH CPL_DLL OGR_FD_GetFieldDefn( OGRFeatureDefnH, int );
	// int    CPL_DLL OGR_FD_GetFieldIndex( OGRFeatureDefnH, const char * );
	// void   CPL_DLL OGR_FD_AddFieldDefn( OGRFeatureDefnH, OGRFieldDefnH );
	// OGRwkbGeometryType CPL_DLL OGR_FD_GetGeomType( OGRFeatureDefnH );
	// void   CPL_DLL OGR_FD_SetGeomType( OGRFeatureDefnH, OGRwkbGeometryType );
	// int    CPL_DLL OGR_FD_Reference( OGRFeatureDefnH );
	// int    CPL_DLL OGR_FD_Dereference( OGRFeatureDefnH );
	// int    CPL_DLL OGR_FD_GetReferenceCount( OGRFeatureDefnH );

	OGRFeatureDefnH CreateOgrFeatureDefn(ISFFeatureDefn *poFDefn)
	{
		if (NULL == poFDefn)
			return NULL;

		OGRFeatureDefnH poOgrFDefn;
		//poOgrFDefn = OGRFeatureDefn::CreateFeatureDefn( poFDefn->GetName() );

		poOgrFDefn = OGR_FD_Create(poFDefn->GetName());

		//poOgrFDefn->SetGeomType( OGRwkbGeometryType(poFDefn->GetGeomType()));

		OGR_FD_SetGeomType(poOgrFDefn, OGRwkbGeometryType(poFDefn->GetGeomType()));

		ISFFieldDefn *poFieldDefn;
		for (int i = poFDefn->GetFieldCount() - 1; i >= 0; i--)
		{
			poFieldDefn = poFDefn->GetFieldDefn(i);
			OGRFieldDefnH poOgrFieldDefn = CreateOgrFieldDefn(poFieldDefn);

			if (poOgrFieldDefn)
			{
				//poOgrFDefn->AddFieldDefn(poOgrFieldDefn);

				OGR_FD_AddFieldDefn(poOgrFDefn, poOgrFieldDefn);

				OGRFree(poOgrFieldDefn);
			}
		}

		return poOgrFDefn;
	}

}
	
