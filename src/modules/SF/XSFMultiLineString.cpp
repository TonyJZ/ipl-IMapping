#include <math.h>
#include "XSFMultiLineString.h"

using namespace ipl;

XSFMultiLineString::XSFMultiLineString()
{
	m_coordDimension = 2;
}
XSFMultiLineString::~XSFMultiLineString()
{
}


void XSFMultiLineString::getEnvelope( iplEnvelope * psEnvelope ) const
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


SFERR XSFMultiLineString::addGeometry( const ref_ptr<IGeoObject> geom )
{
	IPL_wkbGeometryType type = geom->getGeometryType();
	if (type != IPL_wkbLineString && type != IPL_wkbLineString25D)
		return SFERR_FAILURE;
	
	ref_ptr<ISFLineString> poLine = ref_ptr<ISFLineString>((ISFLineString *)geom->Clone());
	m_geometries.push_back( poLine );
	
	return SFERR_NONE;	
}



SFERR XSFMultiLineString::addGeometryDirectly( ref_ptr<IGeoObject> geom )
{
	IPL_wkbGeometryType type = geom->getGeometryType();
	if (type != IPL_wkbLineString && type != IPL_wkbLineString25D)
		return SFERR_FAILURE;

	//ref_ptr<ISFLineString> poLine = ref_ptr<ISFLineString>((ISFLineString *)geom);
	m_geometries.push_back(ipl_static_pointer_cast<ISFLineString>(geom));
	
	return SFERR_NONE;	
}


SFERR XSFMultiLineString::removeGeometry( int i, int bDelete )
{
	if (i < 0||i>m_geometries.size() -1)
		return SFERR_FAILURE;

	std::vector <ref_ptr<ISFLineString> >::iterator iter = m_geometries.begin();

	m_geometries.erase( iter+i );

	return SFERR_NONE;
}


IGeoObject *XSFMultiLineString::Clone() const
{
	ISFMultiLineString *pOMultiLineString;

	pOMultiLineString = new XSFMultiLineString();

	for (int i = 0; i < m_geometries.size(); i++)
	{
		pOMultiLineString->addGeometry(m_geometries[i]);
	}

	return pOMultiLineString;

}

