#include <math.h>

#include "XSFPolygon.h"

using namespace ipl;
XSFPolygon::XSFPolygon()
{
	m_coordDimension = 2;
}
XSFPolygon::~XSFPolygon()
{
}
void XSFPolygon::getEnvelope( iplEnvelope * psEnvelope ) const
{
	iplEnvelope  oRingEnv;

	if( m_Rings.size() == 0 )
		return;

	m_Rings[0]->getEnvelope( psEnvelope );

	for( int iRing = 1; iRing < m_Rings.size(); iRing++ )
	{
		m_Rings[iRing]->getEnvelope( &oRingEnv );

		if( psEnvelope->minX > oRingEnv.minX )
			psEnvelope->minX = oRingEnv.minX;
		if( psEnvelope->minY > oRingEnv.minY )
			psEnvelope->minY = oRingEnv.minY;
		if( psEnvelope->maxX < oRingEnv.maxX )
			psEnvelope->maxX = oRingEnv.maxX;
		if( psEnvelope->maxY < oRingEnv.maxY )
			psEnvelope->maxY = oRingEnv.maxY;
	}
}


double XSFPolygon::getArea() const
{
	double dfArea = 0.0;

	if( getExteriorRing() != NULL )
	{
		int iRing;

		dfArea = getExteriorRing()->getArea();

		for( iRing = 0; iRing < getNumInteriorRings(); iRing++ )
			dfArea -= getInteriorRing( iRing )->getArea();
	}

	return dfArea;
}

void  XSFPolygon::addRing (const ref_ptr<ISFLineRing> pRing)
{
	ref_ptr<ISFLineRing> poRing = ref_ptr<ISFLineRing>((ISFLineRing*)pRing->Clone());
	m_Rings.push_back(poRing);
}

void  XSFPolygon::addRingDirectly(ref_ptr<ISFLineRing> pRing)
{
	//ref_ptr<ISFLineRing> poRing = ref_ptr<ISFLineRing>(pRing);
	m_Rings.push_back(pRing);
}


ref_ptr<ISFLineRing> XSFPolygon::getExteriorRing () 
{
	ref_ptr<ISFLineRing>  nullRing;
	nullRing.reset();

	if(m_Rings.size() == 0)
		return nullRing;

	return m_Rings[0];
}


const ref_ptr<ISFLineRing> XSFPolygon::getExteriorRing() const
{
	ref_ptr<ISFLineRing>  nullRing;
	nullRing.reset();

	if( m_Rings.size() == 0 )
		return nullRing;

	return m_Rings[0];
}


int  XSFPolygon::getNumInteriorRings () const 
{
	return m_Rings.size() - 1;
}

ref_ptr<ISFLineRing> XSFPolygon::getInteriorRing (int i) 
{
	ref_ptr<ISFLineRing>  nullRing;
	nullRing.reset();

	if(i<0||i>m_Rings.size() - 2)
		return nullRing;

	return m_Rings[i+1];
}

const ref_ptr<ISFLineRing> XSFPolygon::getInteriorRing (int i) const
{
	ref_ptr<ISFLineRing>  nullRing;
	nullRing.reset();

	if(i<0||i>m_Rings.size() - 2)
		return nullRing;

	return m_Rings[i+1];
}


void  XSFPolygon::closeRings ()
{
	for( int iRing = 0; iRing < m_Rings.size(); iRing++ )
		m_Rings[iRing].get()->closeRing();
}

IGeoObject *XSFPolygon::Clone() const
{
	ISFPolygon *poPolygon;
	
	poPolygon = new XSFPolygon;
	
	for (int i = 0; i < m_Rings.size(); i++)
	{
		 poPolygon->addRing(ref_ptr<ISFLineRing>(static_cast<ISFLineRing*>(m_Rings[i]->Clone())));
	}

	return poPolygon;
}

