#include <math.h>
#include "XSFMultiPoint.h"

using namespace ipl;

XSFMultiPoint::XSFMultiPoint()
{
	m_coordDimension = 2;
}

XSFMultiPoint::~XSFMultiPoint()
{
}
void XSFMultiPoint::getEnvelope( iplEnvelope * psEnvelope ) const
{
	iplEnvelope oGeomEnv;
    
    if( m_geometries.size() == 0 )
        return;

    m_geometries[0].get()->getEnvelope( psEnvelope );

    for( int iGeom = 1; iGeom < m_geometries.size(); iGeom++ )
    {
        m_geometries[iGeom].get()->getEnvelope( &oGeomEnv );

        if( psEnvelope->minX > oGeomEnv.minX )
            psEnvelope->minX = oGeomEnv.minX;
        if( psEnvelope->minY > oGeomEnv.minY )
            psEnvelope->minY = oGeomEnv.minY;
        if( psEnvelope->maxX < oGeomEnv.maxX )
            psEnvelope->maxX = oGeomEnv.maxX;
        if( psEnvelope->maxY < oGeomEnv.maxY )
            psEnvelope->maxY = oGeomEnv.maxY;
    }	
}



SFERR XSFMultiPoint::addGeometry( const ref_ptr<IGeoObject> geom )
{
	IPL_wkbGeometryType type = geom->getGeometryType();
	if (type != IPL_wkbPoint && type != IPL_wkbPoint25D)
		return SFERR_FAILURE;
	
	ref_ptr<ISFPoint> poPoint= ref_ptr<ISFPoint>((ISFPoint *)geom->Clone());
	m_geometries.push_back( poPoint);
	
	return SFERR_NONE;	
}


SFERR XSFMultiPoint::addGeometryDirectly( ref_ptr<IGeoObject> geom )
{
	IPL_wkbGeometryType type = geom->getGeometryType();
	if (type != IPL_wkbPoint && type != IPL_wkbPoint25D)
		return SFERR_FAILURE;

	//ref_ptr<ISFPoint> poPoint= ref_ptr<ISFPoint>((ISFPoint *)geom);
	m_geometries.push_back( ipl_static_pointer_cast<ISFPoint>(geom));
	
	return SFERR_NONE;	
}


SFERR XSFMultiPoint::removeGeometry( int i, int bDelete )
{
	if (i < 0||i>m_geometries.size() -1)
		return SFERR_FAILURE;
	
	iplArray <ref_ptr<ISFPoint> >::iterator iter = m_geometries.begin();

	m_geometries.erase( iter+i );

	return SFERR_NONE;
}

IGeoObject *XSFMultiPoint::Clone() const
{
	ISFMultiPoint *poMultiPt;

	poMultiPt = new XSFMultiPoint;

	for (int i = 0; i < m_geometries.size(); i++)
	{
		poMultiPt->addGeometry(m_geometries[i]);
	}

	return poMultiPt;
}

