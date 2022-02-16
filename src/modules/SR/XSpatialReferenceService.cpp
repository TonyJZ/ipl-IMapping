#include "XSpatialReferenceService.h"
#include "XSpatialReference.h"
#include "XCoordinateTransform.h"

using namespace ipl;

XSpatialReferenceService::XSpatialReferenceService()
{
};


XSpatialReferenceService::~XSpatialReferenceService()
{
};


bool XSpatialReferenceService::startup(IPlatform *platform)
{
	return true;
}

void XSpatialReferenceService::shutdown()
{
	return;
}

bool XSpatialReferenceService::isok()
{
	return true;
}

IGeoid *XSpatialReferenceService::CreateGeoid( const char *wktGeoidName ) const 
{
	IGeoid *geoid = static_cast<IGeoid*>(new XGeoid());

	if( NULL != geoid ) {
		geoid->Initialize( wktGeoidName );
		return geoid;
	}

	return NULL;
}

IEllipsoid *XSpatialReferenceService::CreateEllipsoid( const char *wktEllipsoidName ) const 
{
	IEllipsoid *pEllipsoid = static_cast<IEllipsoid*>(new XEllipsoid());

	if( NULL != pEllipsoid )
		return pEllipsoid;

	return NULL;
}

ITangentPlane *XSpatialReferenceService::CreateTangentPlane() const
{
	return new XTangentPlane();

	return NULL;
}


ISpatialReference *XSpatialReferenceService::CreateSpatialReference( const char *wktHcs, const char *wktVcs ) const 
{
	ISpatialReference *pSRS = new XSpatialReference( false );

	if( wktHcs )
		pSRS->importFromWkt( wktHcs );
	
	if( wktVcs )
		pSRS->importFromVcsWkt( wktVcs );

	return pSRS;
}

ICoordinateTransform *XSpatialReferenceService::CreateCoordinateTransform() const 
{
	return new XCoordinateTransform();

}


// orsISrsDlg *XSpatialReferenceService::CreateSrsDlg() const
// {
// 	return ORS_PTR_CAST( orsISrsDlg, getPlatform()->createObject( ORS_EXTENSION_SRS_DLG_DEFALUT ) );
// }


bool XSpatialReferenceService::GetAvailableGeoidNames( iplArray <iplString> &geoidNames )
{
	return false;

}

bool XSpatialReferenceService::GetAvailableEllipsoidNames( iplArray <iplString> &ellipsoidNames )
{
	return false;

}

// query services for config
bool XSpatialReferenceService::GetAvailableDatumNames( iplArray <iplString> &pcsNames )
{
	return false;
}

// Geographic Coordinate Systems
bool XSpatialReferenceService::GetAvailableGcsNames( iplArray <iplString> &pcsNames )
{
	return false;

}

// Projected Coordinate System
bool XSpatialReferenceService::GetAvailablePcsNames( iplArray <iplString> &pcsNames )
{
	return false;

}

// Vertical Coordinate System
bool XSpatialReferenceService::GetAvailableVcsNames( iplArray <iplString> &pcsNames )
{
	return false;

}



//////////////////////////////////////////////////////////////////////////
