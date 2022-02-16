#include <math.h>

#include "XSFGeometryCollection.h"

using namespace ipl;

XSFGeometryCollection::XSFGeometryCollection()
{
	m_coordDimension = 2;
}
XSFGeometryCollection::~XSFGeometryCollection()
{
}

void XSFGeometryCollection::getEnvelope( iplEnvelope * psEnvelope ) const
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

SFERR XSFGeometryCollection::addGeometry( const ref_ptr<IGeoObject> geom )
{
	ref_ptr<IGeoObject> poGeom= ref_ptr<IGeoObject>(geom->Clone());
	m_geometries.push_back( poGeom );
	
	return SFERR_NONE;
}


SFERR XSFGeometryCollection::addGeometryDirectly( ref_ptr<IGeoObject> geom )
{
	//ref_ptr<IGeoObject> poGeom= ref_ptr<IGeoObject>(geom);
	m_geometries.push_back( geom );
	
	return SFERR_NONE;
}


SFERR XSFGeometryCollection::removeGeometry( int i, int bDelete )
{	
	if (i < 0||i>m_geometries.size() -1)
		return SFERR_FAILURE;

	std::vector <ref_ptr<IGeoObject> >::iterator iter;
	iter = m_geometries.begin();

	m_geometries.erase(iter+i);
	return SFERR_NONE;
}

IGeoObject *XSFGeometryCollection::Clone() const 
{
	ISFGeometryCollection *pOGeoCol;

	pOGeoCol = new XSFGeometryCollection;
	
	for (int i = 0; i < m_geometries.size(); i++)
	{
		pOGeoCol->addGeometry(m_geometries[i]);
	}

	return pOGeoCol;
}

