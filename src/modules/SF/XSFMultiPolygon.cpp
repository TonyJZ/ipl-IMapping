#include <math.h>

#include "XSFMultiPolygon.h"

using namespace ipl;

XSFMultiPolygon::XSFMultiPolygon()
{
	m_coordDimension = 2;
}
XSFMultiPolygon::~XSFMultiPolygon()
{
}
void XSFMultiPolygon::getEnvelope( iplEnvelope * psEnvelope ) const
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

SFERR XSFMultiPolygon::addGeometry( const ref_ptr<IGeoObject> geom )
{
	IPL_wkbGeometryType type = geom->getGeometryType();
	if (type != IPL_wkbPolygon && type != IPL_wkbPolygon25D)
		return SFERR_FAILURE;
	
	ref_ptr<ISFPolygon> poPolygon = ref_ptr<ISFPolygon>(( ISFPolygon *)geom->Clone());
	m_geometries.push_back( poPolygon );
	
	return SFERR_NONE;	
}

SFERR XSFMultiPolygon::addGeometryDirectly( ref_ptr<IGeoObject> geom )
{
	IPL_wkbGeometryType type = geom->getGeometryType();
	if (type != IPL_wkbPolygon && type != IPL_wkbPolygon25D)
		return SFERR_FAILURE;

//	ref_ptr<ISFPolygon> poPolygon = ref_ptr<ISFPolygon>(( ISFPolygon *)geom);
	m_geometries.push_back( ipl_static_pointer_cast<ISFPolygon>(geom) );

	return SFERR_NONE;	
}


SFERR XSFMultiPolygon::removeGeometry( int i, int bDelete )
{
	if (i < 0||i>m_geometries.size() -1)
		return SFERR_FAILURE;
	
	std::vector <ref_ptr<ISFPolygon> >::iterator iter = m_geometries.begin();

	m_geometries.erase( iter+i );

	return SFERR_NONE;
}

double XSFMultiPolygon::getArea() const
{
	double dfArea = 0.0;
    int iPoly;

    for( iPoly = 0; iPoly < getNumGeometries(); iPoly++ )
    {
        dfArea += m_geometries[iPoly].get()->getArea();
    }

    return dfArea;
}

IGeoObject *XSFMultiPolygon::Clone() const
{
	ISFMultiPolygon *pOMultiPolygon;

	pOMultiPolygon = new XSFMultiPolygon;

	for (int i = 0; i < m_geometries.size(); i++)
	{
		pOMultiPolygon->addGeometry(m_geometries[i]);
	}

	return pOMultiPolygon;
}

