#include <math.h>

#include "XSFPoint.h"


using namespace ipl;

XSFPoint::XSFPoint()
{
	x= y = z = 0;
	m_coordDimension = 0;
}

XSFPoint::~XSFPoint()
{
}

IGeoObject *ipl::XSFPoint::Clone() const
{
	ISFPoint  *poPoint;
	
    poPoint = new XSFPoint();

	poPoint->x = x;
	poPoint->y = y;
	poPoint->z = z;
	
    return poPoint;
}

