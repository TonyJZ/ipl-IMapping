#include <math.h>
#include "XSFLineRing.h"

using namespace ipl;

XSFLineRing::~XSFLineRing()
{
}


void XSFLineRing::getEnvelope( iplEnvelope * psEnvelope )const
{
	double dfMinX, dfMinY, dfMaxX, dfMaxY;

	if( m_points.size() == 0 )
		return;

	dfMinX = dfMaxX = m_points[0].X;
	dfMinY = dfMaxY = m_points[0].Y;

	for( int iPoint = 1; iPoint < m_points.size(); iPoint++ )
	{
		if( dfMaxX < m_points[iPoint].X )
			dfMaxX = m_points[iPoint].X;
		if( dfMaxY < m_points[iPoint].Y )
			dfMaxY = m_points[iPoint].Y;
		if( dfMinX > m_points[iPoint].X )
			dfMinX = m_points[iPoint].X;
		if( dfMinY > m_points[iPoint].Y )
			dfMinY = m_points[iPoint].Y;
	}
	psEnvelope->minX = dfMinX;
	psEnvelope->maxX = dfMaxX;
	psEnvelope->minY = dfMinY;
	psEnvelope->maxY = dfMaxY;
}


IPL_wkbGeometryType XSFLineRing::getGeometryType() const
{
	if (m_bIsRing)
		return IPL_wkbLinearRing;
	if( m_coordDimension < 3 )
		return IPL_wkbLineString;
	else
		return IPL_wkbLineString25D;
}


const char *XSFLineRing::getGeometryName() const
{
	if (m_bIsRing)
		return "LINERING";
	return "LINESTRING";
}

// 曲线长度
double XSFLineRing::getLength() const
{
	double      dfLength = 0;
	int i;
	double      dfDeltaX, dfDeltaY;
	for( i = 0;i<m_points.size() - 2 ;i++ )
	{
		dfDeltaX = m_points[i+1].X - m_points[i].X;
		dfDeltaY = m_points[i+1].Y - m_points[i].Y;
		dfLength += sqrt(dfDeltaX*dfDeltaX + dfDeltaY*dfDeltaY);
	}
    if (m_bIsRing)
	{
		dfDeltaX = m_points[0].X - m_points[m_points.size() - 1].X;
		dfDeltaY = m_points[0].Y - m_points[m_points.size() - 1].Y;
		dfLength += sqrt(dfDeltaX*dfDeltaX + dfDeltaY*dfDeltaY);
	}
	return dfLength;
}

// 起点
void XSFLineRing::startPoint(iplPOINT3D *point) 
{
	*point = m_points[0];
}

// 终点
void XSFLineRing::endPoint(iplPOINT3D *point) 
{
	*point = m_points[m_points.size() - 1];
}

// 是否闭合
int XSFLineRing::isClosed()  
{
	return (m_points[0] == m_points[m_points.size()-1]);
}


int XSFLineRing::getNumPoints() const
{
	return m_points.size();
}


void XSFLineRing::setNumPoints (int numPoints)
{
	m_points.resize(numPoints);
}


const iplPOINT3D *XSFLineRing::getPtsBuf() const
{
	if( 0 == m_points.size() )
		return NULL;

	return &m_points[0];
}


void XSFLineRing::getPoint(int index, iplPOINT3D *points )
{
	if(m_points.size() == 0)
		points = NULL;

	if (index<0||index>=m_points.size())
		points = NULL;
	
	*points = m_points[index];
	
}


void XSFLineRing::setPoint(int index, const iplPOINT3D point)
{
	if (index<0||index>=m_points.size())
		return;

	m_points[index] = point;
}
void XSFLineRing::setPoint( int index, double x, double y, double z )
{
	iplPOINT3D point;
	if (index<0||index>=m_points.size())
		return;
	point.X =x;
	point.Y = y;
	point.Z = z;
	m_points[index] = point;
 
}
void XSFLineRing::setPoints(iplPOINT3D *points,int num)
{
	m_points.empty();
	for (int i = 0;i<num;i++)
		m_points.push_back(points[i]);
}

void XSFLineRing::addPoint( const iplPOINT3D point )
{
	m_points.push_back(point);
}

void XSFLineRing::removePoint( int index  )
{
	if (index<0 || index>=m_points.size())
		return;
	
	m_points.erase(m_points.begin() + index);
}

int XSFLineRing::isClockwise() const 
{
	if(!m_bIsRing)
		return 0;
	double dfSum = 0.0;
	
	for( int iVert = 0; iVert < m_points.size()-1; iVert++ )
	{
		dfSum += m_points[iVert].X * m_points[iVert+1].Y
			- m_points[iVert].Y * m_points[iVert+1].X;
	}

	dfSum += m_points[m_points.size()-1].X * m_points[0].Y
		- m_points[m_points.size()-1].Y * m_points[0].X;

	return dfSum < 0.0;
}

void XSFLineRing::closeRing()
{
	if(!m_bIsRing)
		return ;
	if( m_points.size() < 2 )
		return;

	if( m_points[0] != m_points[m_points.size() -1])
		addPoint(m_points[0]);

}

double XSFLineRing::getArea() const 
{
	double dfAreaSum = 0.0;
	int i;

	for( i = 0; i < m_points.size()-1; i++ )
	{
		dfAreaSum += 0.5 * ( m_points[i].X * m_points[i+1].Y 
			- m_points[i+1].X * m_points[i].Y );
	}

	dfAreaSum += 0.5 * ( m_points[m_points.size()-1].X * m_points[0].Y 
		- m_points[0].X * m_points[m_points.size()-1].Y );

	return fabs(dfAreaSum);
}

IGeoObject *XSFLineRing::Clone() const 
{
	XSFLineRing *poRing;

	poRing = new XSFLineRing;
	poRing->SetAsRing(m_bIsRing);

	int n = getNumPoints();

	poRing->setNumPoints(n);

	for (int i = 0; i < n; i++)
	{
		poRing->setPoint(i, m_points[i]);
	}

	return poRing;
}

