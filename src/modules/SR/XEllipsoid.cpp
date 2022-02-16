/***************************************************************************/
/* RSC IDENTIFIER:  GEOCENTRIC
 *
 * ABSTRACT
 *
 *    This component provides conversions between Geodetic coordinates (latitude,
 *    longitude in radians and height in meters) and Geocentric coordinates
 *    (X, Y, Z) in meters.
 *
 * ERROR HANDLING
 *
 *    This component checks parameters for valid values.  If an invalid value
 *    is found, the error code is combined with the current error code using 
 *    the bitwise or.  This combining allows multiple error codes to be
 *    returned. The possible error codes are:
 *
 *      DTCC_NO_ERROR        : No errors occurred in function
 *      DTCC_LAT_ERROR       : Latitude out of valid range
 *                                 (-90 to 90 degrees)
 *      DTCC_LON_ERROR       : Longitude out of valid range
 *                                 (-180 to 360 degrees)
 *      DTCC_A_ERROR         : Semi-major axis lessthan or equal to zero
 *      DTCC_B_ERROR         : Semi-minor axis lessthan or equal to zero
 *      DTCC_A_LESS_B_ERROR  : Semi-major axis less than semi-minor axis
 *
 *
 * REUSE NOTES
 *
 *    GEOCENTRIC is intended for reuse by any application that performs
 *    coordinate conversions between geodetic coordinates and geocentric
 *    coordinates.
 *    
 *
 * REFERENCES
 *    
 *    An Improved Algorithm for Geocentric to Geodetic Coordinate Conversion,
 *    Ralph Toms, February 1996  UCRL-JC-123138.
 *    
 *    Further information on GEOCENTRIC can be found in the Reuse Manual.
 *
 *    GEOCENTRIC originated from : U.S. Army Topographic Engineering Center
 *                                 Geospatial Information Division
 *                                 7701 Telegraph Road
 *                                 Alexandria, VA  22310-3864
 *
 * LICENSES
 *
 *    None apply to this component.
 *
 * RESTRICTIONS
 *
 *    GEOCENTRIC has no restrictions.
 *
/***************************************************************************/

#include <math.h>

#include <string.h>
#include <stdio.h>
#include <iostream>

#include "core/interface/IPlatform.h"
#include "core/iplmacro.h"
#include "XEllipsoid.h"

//XEllipsoid	_epWgs84;

//////////////////////////////////////////////////////////////////////////

#define PI         3.1415926535897932384626433832795e0
#define PI_OVER_2  (PI / 2.0e0)
#define COS_67P5   0.38268343236508977  /* cosine of 67.5 degrees */
#define AD_C       1.0026000            /* Toms region 1 constant */

using namespace ipl;

XEllipsoid::XEllipsoid()
{
	/* Ellipsoid parameters, default to WGS 84 */

	m_a = 6378137.0;				/* Semi-major axis of ellipsoid in meters */
	m_b = 6356752.3142;				/* Semi-minor axis of ellipsoid           */
	
	m_a2 = 40680631590769.0;        /* Square of semi-major axis */
	m_b2 = 40408299984087.05;       /* Square of semi-minor axis */
	m_e2 = 0.0066943799901413800;   /* Eccentricity squared  */
	m_ep2 = 0.00673949675658690300; /* 2nd eccentricity squared */

	m_errorCount = 0 ;
}


bool XEllipsoid::SetParameter( double a, double b)
{
	char msg[256];

	if (a <= 0.0 && NULL != getPlatform()) {
		sprintf(msg, "orsXEllipsoid: Invalid semi-major : %.5lf", a);
		getPlatform()->logPrint(IPL_LOG_ERROR, msg);
		return false;
	}

	if (b <= 0.0 && NULL != getPlatform()) {
		sprintf(msg, "orsXEllipsoid: Invalid semi-minor : %.5lf", b);
		getPlatform()->logPrint(IPL_LOG_ERROR, msg);
		return false;
	}

	if (a < b && NULL != getPlatform()) {
		sprintf(msg, "orsXEllipsoid: semi-major %.5lf < semi-minor %.5lf", a, b);
		getPlatform()->logPrint(IPL_LOG_ERROR, msg);
		return false;
	}

	m_a = a;	m_a2 = a * a;
	m_b = b;	m_b2 = b * b;
	m_e2 = (m_a2 - m_b2) / m_a2;
	m_ep2 = (m_a2 - m_b2) / m_b2;

	return true;
}




/****************************************************************************
* The function Convert_Geodetic_To_Geocentric converts geodetic coordinates
* (latitude, longitude, and height) to geocentric coordinates (X, Y, Z),
* according to the current ellipsoid parameters.
*
*    Latitude  : Geodetic latitude in radians                     (input)
*    Longitude : Geodetic longitude in radians                    (input)
*    Height    : Geodetic height, in meters                       (input)
*    X         : Calculated Geocentric X coordinate, in meters    (output)
*    Y         : Calculated Geocentric Y coordinate, in meters    (output)
*    Z         : Calculated Geocentric Z coordinate, in meters    (output)
*
*****************************************************************************/
bool XEllipsoid::Geodetic_To_Geocentric (double latitude, double longitude,double height, double *X, double *Y, double *Z)
{
	double Rn;            /*  Earth radius at location  */
	double sin_lat;       /*  sin(latitude)  */
	double sin2_lat;      /*  square of sin(latitude)  */
	double cos_lat;       /*  cos(latitude)  */

	char msg[256];
	
	if ((latitude < -PI_OVER_2) || (latitude > PI_OVER_2))
	{
		if (m_errorCount < 2 && NULL != getPlatform()) {
			sprintf(msg, "Geodetic_To_Geocentric: latitude out of range : %.7lf", latitude);
			getPlatform()->logPrint(IPL_LOG_ERROR, msg);
		}
		m_errorCount++;
		return false;
	}

	if ((longitude < -2 * PI) || (longitude > (2 * PI)))
	{
		if (m_errorCount < 2 && NULL != getPlatform()) {
			sprintf(msg, "Geodetic_To_Geocentric: longitude out of range : %.7lf", longitude);
			getPlatform()->logPrint(IPL_LOG_ERROR, msg);
		}
		m_errorCount++;
		return false;
	}

	if( longitude > 2*PI )
		longitude -= (2*PI);

	sin_lat = sin(latitude);
	cos_lat = cos(latitude);

	sin2_lat = sin_lat * sin_lat;

	Rn = m_a / (sqrt(1.0e0 - m_e2 * sin2_lat));

	*X = (Rn + height) * cos_lat * cos(longitude);
	*Y = (Rn + height) * cos_lat * sin(longitude);
	*Z = ((Rn * (1 - m_e2)) + height) * sin_lat;
	
	return true;
}


/********************************************************************
 * The function Convert_Geocentric_To_Geodetic converts geocentric
 * coordinates (X, Y, Z) to geodetic coordinates (latitude, longitude, 
 * and height), according to the current ellipsoid parameters.
 *
 *    X         : Geocentric X coordinate, in meters.         (input)
 *    Y         : Geocentric Y coordinate, in meters.         (input)
 *    Z         : Geocentric Z coordinate, in meters.         (input)
 *    Latitude  : Calculated latitude value in radians.       (output)
 *    Longitude : Calculated longitude value in radians.      (output)
 *    Height    : Calculated height value, in meters.         (output)
 *
 * The method used here is derived from 'An Improved Algorithm for
 * Geocentric to Geodetic Coordinate Conversion', by Ralph Toms, Feb 1996
 *
 *  Note: Variable names follow the notation used in Toms, Feb 1996 
 *********************************************************************/
#define USE_ITERATIVE_METHOD

bool XEllipsoid::Geocentric_To_Geodetic (double X, double Y, double Z, double *latitude, double *longitude,double *height)
{

#if !defined(USE_ITERATIVE_METHOD)
	double W;        /* distance from Z axis */
	double W2;       /* square of distance from Z axis */
	double T0;       /* initial estimate of vertical component */
	double T1;       /* corrected estimate of vertical component */
	double s0;       /* initial estimate of horizontal component */
	double s1;       /* corrected estimate of horizontal component */
	double sin_B0;   /* sin(B0), B0 is estimate of Bowring aux variable */
	double sin3_B0;  /* cube of sin(B0) */
	double cos_B0;   /* cos(B0) */
	double sin_p1;   /* sin(phi1), phi1 is estimated latitude */
	double cos_p1;   /* cos(phi1) */
	double Rn;       /* Earth radius at location */
	double sum;      /* numerator of cos(phi1) */

	bool at_Pole;     /* indicates location is in polar region */
	
	at_Pole = false;
	if (X != 0.0)
	{
		*longitude = atan2(Y,X);
	}
	else
	{
		if (Y > 0)
		{
			*longitude = PI_OVER_2;
		}
		else if (Y < 0)
		{
			*longitude = -PI_OVER_2;
		}
		else
		{
			at_Pole = true;
			*longitude = 0.0;
			if (Z > 0.0)
			{  /* north pole */
				*latitude = PI_OVER_2;
			}
			else if (Z < 0.0)
			{  /* south pole */
				*latitude = -PI_OVER_2;
			}
			else
			{  /* center of earth */
				*latitude = PI_OVER_2;
				*height = -m_b;
				return true;
			} 
		}
	}
	W2 = X*X + Y*Y;
	W = sqrt(W2);
	T0 = Z * AD_C;
	s0 = sqrt(T0 * T0 + W2);
	sin_B0 = T0 / s0;
	cos_B0 = W / s0;
	sin3_B0 = sin_B0 * sin_B0 * sin_B0;
	T1 = Z + m_b * m_ep2 * sin3_B0;
	sum = W - m_a * m_e2 * cos_B0 * cos_B0 * cos_B0;
	s1 = sqrt(T1*T1 + sum * sum);
	sin_p1 = T1 / s1;
	cos_p1 = sum / s1;
	Rn = m_a / sqrt(1.0 - m_e2 * sin_p1 * sin_p1);

	if (cos_p1 >= COS_67P5)
	{
		*height = W / cos_p1 - Rn;
	}
	else if (cos_p1 <= -COS_67P5)
	{
		*height = W / -cos_p1 - Rn;
	}
	else
	{
		*height = Z / sin_p1 + Rn * (m_e2 - 1.0);
	}

	if ( !at_Pole )
		*latitude = atan(sin_p1 / cos_p1);

	return true;

#else /* defined(USE_ITERATIVE_METHOD) */
/*
* Reference...
* ============
* Wenzel, H.-G.(1985): Hochaufl枚sende Kugelfunktionsmodelle f眉r
* das Gravitationspotential der Erde. Wiss. Arb. Univ. Hannover
* Nr. 137, p. 130-131.

* Programmed by GGA- Leibniz-Institue of Applied Geophysics
*               Stilleweg 2
*               D-30655 Hannover
*               Federal Republic of Germany
*               Internet: www.gga-hannover.de
*
*               Hannover, March 1999, April 2004.
*               see also: comments in statements
* remarks:
* Mathematically exact and because of symmetry of rotation-ellipsoid,
* each point (X,Y,Z) has at least two solutions (Latitude1,Longitude1,Height1) and
* (Latitude2,Longitude2,Height2). Is point=(0.,0.,Z) (P=0.), so you get even
* four solutions,	every two symmetrical to the semi-minor axis.
* Here Height1 and Height2 have at least a difference in order of
* radius of curvature (e.g. (0,0,b)=> (90.,0.,0.) or (-90.,0.,-2b);
* (a+100.)*(sqrt(2.)/2.,sqrt(2.)/2.,0.) => (0.,45.,100.) or
* (0.,225.,-(2a+100.))).
* The algorithm always computes (Latitude,Longitude) with smallest |Height|.
* For normal computations, that means |Height|<10000.m, algorithm normally
* converges after to 2-3 steps!!!
* But if |Height| has the amount of length of ellipsoid's axis
* (e.g. -6300000.m),	algorithm needs about 15 steps.
*/

/* local defintions and variables */
/* end-criterium of loop, accuracy of sin(Latitude) */
#define genau   1.E-12
#define genau2  (genau*genau)
#define maxiter 30

    double P;        /* distance between semi-minor axis and location */
    double RR;       /* distance between center and location */
    double CT;       /* sin of geocentric latitude */
    double ST;       /* cos of geocentric latitude */
    double RX;
    double RK;
    double RN;       /* Earth radius at location */
    double CPHI0;    /* cos of start or old geodetic latitude in iterations */
    double SPHI0;    /* sin of start or old geodetic latitude in iterations */
    double CPHI;     /* cos of searched geodetic latitude */
    double SPHI;     /* sin of searched geodetic latitude */
    double SDPHI;    /* end-criterium: addition-theorem of sin(Latitude(iter)-Latitude(iter-1)) */
    bool At_Pole;     /* indicates location is in polar region */
    int iter;        /* # of continous iteration, max. 30 is always enough (s.a.) */

    At_Pole = false;
    P = sqrt(X*X+Y*Y);
    RR = sqrt(X*X+Y*Y+Z*Z);

/*	special cases for latitude and longitude */
    if (P/m_a < genau) {

/*  special case, if P=0. (X=0., Y=0.) */
        At_Pole = true;
	*longitude = 0.;

/*  if (X,Y,Z)=(0.,0.,0.) then Height becomes semi-minor axis
 *  of ellipsoid (=center of mass), Latitude becomes PI/2 */
        if (RR/m_a < genau) {
            *latitude = PI_OVER_2;
            *height   = -m_b;
            return true;

        }
    }
    else {
/*  ellipsoidal (geodetic) longitude
 *  interval: -PI < Longitude <= +PI */
        *longitude=atan2(Y,X);
    }

/* --------------------------------------------------------------
 * Following iterative algorithm was developped by
 * "Institut f眉r Erdmessung", University of Hannover, July 1988.
 * Internet: www.ife.uni-hannover.de
 * Iterative computation of CPHI,SPHI and Height.
 * Iteration of CPHI and SPHI to 10**-12 radian resp.
 * 2*10**-7 arcsec.
 * --------------------------------------------------------------
 */
    CT = Z/RR;
    ST = P/RR;
    RX = 1.0/sqrt(1.0-m_e2*(2.0-m_e2)*ST*ST);
    CPHI0 = ST*(1.0-m_e2)*RX;
    SPHI0 = CT*RX;
    iter = 0;

/* loop to find sin(Latitude) resp. Latitude
 * until |sin(Latitude(iter)-Latitude(iter-1))| < genau */
    do
    {
        iter++;
        RN = m_a/sqrt(1.0-m_e2*SPHI0*SPHI0);

/*  ellipsoidal (geodetic) height */
        *height = P*CPHI0+Z*SPHI0-RN*(1.0-m_e2*SPHI0*SPHI0);

        RK = m_e2*RN/(RN+*height);
        RX = 1.0/sqrt(1.0-RK*(2.0-RK)*ST*ST);
        CPHI = ST*(1.0-RK)*RX;
        SPHI = CT*RX;
        SDPHI = SPHI*CPHI0-CPHI*SPHI0;
        CPHI0 = CPHI;
        SPHI0 = SPHI;
    }
    while (SDPHI*SDPHI > genau2 && iter < maxiter);

/*	ellipsoidal (geodetic) latitude */
    *latitude=atan(SPHI/fabs(CPHI));

    return true;
#endif /* defined(USE_ITERATIVE_METHOD) */

} /* END OF Convert_Geocentric_To_Geodetic */



//*****************************************************************************
//  copied from OSSIM and modified nearest intersection parameter t
//
//  METHOD: ossimEllipsoid::nearestIntersection
//  
//   geographic objects that are derive this class will asssume that
//   the reference datum is wgs84 and that the ray origin is a
//   geocentric coordinate relative to the wgs84 datum.  Will return
//   true if the object was intersected and false otherwise.
//  
//   The nearest intersection will use the ray sphere intersection
//   found in most ray tracers.  We will take a Ray defined by the
//   parametric equation:
//  
//     x = x0 + dx*t
//     y = y0 + dy*t
//     z = z0 + dz*t
//  
//   and intersect this with the equation of a spheroid:
//  
//     x^2/theXRadius^2 + y^2/theYRadius^2 + z^2/theZRadius^2 = 1
//  
//  
//   the intersection is achived by substituting the parametric line
//   into the equation of the sphereroid.  By doing this you should
//   get a quadratic in t and the equation should look like this:
//  
//    a*t^2 + b*t + c = 0
//  
//      let a = dx^2/theXRadius^2 + dy^2/theYRadius^2 + dz^2/theZRadius^2
//      let b = 2*(x0*dx/theXRadius^2 +y0*dy/theYRadius^2 + z0*dz/theZRadius^2
//      let c = x0^2/theXRadius^2 + y0^2/theYRadius^2 + z0^2/theZRadius^2 - 1
//  
//  
//    Now solve the quadratic (-b +- sqrt(b^2 - 4ac) ) / 2a
//  
//    After solving for t, the parameter is applied to the ray to determine
//    the 3D point position in X,Y,Z, passed back in rtnPt. The boolean
//    "true" is returned if an intersection was found.
//
//	
//*****************************************************************************
bool XEllipsoid::NearestIntersection( const iplRAY &ray, const double &height, iplPOINT3D *rtnPt) const
{
	double a2 = (m_a + height)*(m_a + height);
	double b2 = (m_b + height)*(m_b + height);
	
	//***
	// Solve m_coefficents of m_ quadratic formula
	//***
	double a =       (ray.dX * ray.dX)/a2 + (ray.dY * ray.dY)/a2 + (ray.dZ * ray.dZ)/b2;
	
	double b = 2.0*( (ray.X0 * ray.dX)/a2 + (ray.Y0 * ray.dY)/a2 + (ray.Z0 * ray.dZ)/b2 );
	
	double c =       (ray.X0 * ray.X0)/a2 + (ray.Y0 * ray.Y0)/a2 + (ray.Z0 * ray.Z0)/b2 - 1.0;
	
	//***
	// solve m_ quadratic
	//***
	double root = b*b - 4*a*c;
	
	double t;
	
	if( root < 0.0 )
	{
		// no intersection
		return false;
	}
	else
	{
		double squareRoot = sqrt(root);
		double t1 = (-b + squareRoot ) / (2.0*a);
		double t2 = (-b - squareRoot ) / (2.0*a);
		
		//
		// which is the nearest Point?
		//		dist2 = dX2+dY2+dZ2 = ( ray.dX2 + ray.dY2 + ray.dZ2 )* t2
		//	==> min { fabs(t1), fabs(t2) }
		//
		if( fabs(t2) < fabs(t1) )
			t = t2;
		else
			t = t1;

		rtnPt->X  = ray.X0 + ray.dX*t; 
		rtnPt->Y  = ray.Y0 + ray.dY*t;
		rtnPt->Z  = ray.Z0 + ray.dZ*t;
	}
	
	return true; 
}



//////////////////////////////////////////////////////////////////////////
//
//	椭球面到切平面的坐标系转换，根据王之卓《摄影测量原理》编码
//	
//	江万寿，2007，12，20
//  坐标轴方向： 东-北-天
//////////////////////////////////////////////////////////////////////////

XTangentPlane::XTangentPlane()
{
	InitializeEllipsoid( 6378137.0, 6356752.3142 );	// WGS84

	m_bSingleModel = false;
}


void XTangentPlane::InitializeEllipsoid( double a, double b )
{
	m_a = a;
	m_b = b;
//	R = (a+b)/2;
	
	m_ellipsoid.SetParameter( a, b );

	a2 = a*a;        /* Square of semi-major axis */
	b2 = b*b;       /* Square of semi-minor axis */
	e2 = ( a2 - b2 ) / a2;   /* Eccentricity squared  */
	ep2 = (a2 - b2 ) / b2;	 /* 2nd eccentricity squared */
}

void XTangentPlane::SetAnchorPoint( double lat0, double lon0, double h0 )
{
	m_lat0 = lat0;
	m_lon0 = lon0;
	m_hei0 = h0;
	
	cosLat0 = cos( lat0 );	sinLat0 = sin( lat0 );
	cosLon0 = cos( lon0 );	sinLon0 = sin( lon0 );
	
	N0 = m_a/sqrt( 1 - e2*sinLat0*sinLat0 );

	m_ellipsoid.Geodetic_To_Geocentric ( m_lat0, m_lon0, m_hei0, &m_Xg0, &m_Yg0, &m_Zg0 );

}


void XTangentPlane::EnableSimpleModel()
{
	m_bSingleModel = true;
}

void XTangentPlane::DisableSimpleModel()
{
	m_bSingleModel = false;
}


// 无需迭代
void XTangentPlane::Geographic2TangentPlane( double lat, double lon, double h, double *X, double *Y, double *Z ) const
{
	double sinLat = sin( lat );
	double cosLat = cos( lat );

	double	dLon = lon - m_lon0;
	double sinDLon = sin( dLon );
	double cosDLon = cos( dLon );

	if( m_bSingleModel ) {
		*X = ( N0 + h )*  cosLat*sinDLon;
		*Y = ( N0 + h )*( sinLat*cosLat0 - cosLat*sinLat0*sinDLon );
		*Z = ( N0 + h )*( sinLat*sinLat0 + cosLat*cosLat0*cosDLon ) - ( N0 + m_hei0 );
	}
	else	{
		double N = m_a/sqrt( 1 - e2*sinLat*sinLat );
		double D = e2*( N0*sinLat0 - N*sinLat );

		*X = ( N + h )*   cosLat*sinDLon;
		*Y = ( N + h )* ( sinLat*cosLat0 - cosLat*sinLat0*cosDLon ) + D*cosLat0;
		*Z = ( N + h )* ( sinLat*sinLat0 + cosLat*cosLat0*cosDLon ) - ( N0 + m_hei0) + D*sinLat0 ;
	}
}


void XTangentPlane::TangentPlane2Geographic( double X, double Y, double Z, double *lat, double *lon, double *h ) const
{
	double A, B, C, lat1, h1;

	A = ( 1 + m_lat0 / N0 )*cosLat0 - sinLat0 / N0 * Y + cosLat0 / N0 * Z;
	B = ( 1 + m_lat0 / N0 )*sinLat0 + cosLat0 / N0 * Y + sinLat0 / N0 * Z;
	C = sqrt( A*A + (X/N0)*(X/N0) );

	// 球面近似
	lat1 = atan2( B, C );
	*lon = m_lon0 + atan2( X, N0*A );

	double  NZh = ( N0 + Z + m_hei0);
	double  XYNZh = sqrt( X*X + Y*Y + NZh*NZh );
	
	h1 = XYNZh - N0;

	if( m_bSingleModel )
		return ;

	//////////////////////////////////////////////////////////////////////////
	// 迭代精化
	double sinLat, N, D, E, DNh;

	for( int times=1; ; times++ ) 
	{
		sinLat = sin( lat1 );

		N = m_a / sqrt( 1 - e2*sinLat*sinLat );	
		D = e2*( N0*sinLat0 - N*sinLat );

		DNh = D/(N + h1 );
		E = sqrt( 1 + (2*sinLat + DNh)* DNh );

		*lat = atan2( B - D/N0, C );
		*h = XYNZh / E - N;

		if( fabs(lat1 - *lat) < 1e-11 )
			break;

		lat1 = *lat;	h1 = *h;
	} 
}



void XTangentPlane::Geocentric2TangentPlane(
	double Xg, 	double Yg, 	double Zg,          // The geocentrical coords
	double *Xt, double *Yt, double *Zt ) const		// The tangent plane coords
{
	double dx, dy, dz;

	dx = Xg - m_Xg0;
	dy = Yg - m_Yg0;
	dz = Zg - m_Zg0;

	*Xt = -sinLon0 * dx + cosLon0 * dy;
	*Yt = -sinLat0 * ( cosLon0*dx + sinLon0*dy ) + cosLat0*dz;
	*Zt =  cosLat0 * ( cosLon0*dx + sinLon0*dy ) + sinLat0*dz;
}



void XTangentPlane::TangentPlane2Geocentric(
	double Xt,	double Yt,	double Zt,		// The tangent plane coords
	double *Xg, double *Yg, double *Zg ) const   // The geocentric coords
{	
	*Xg = m_Xg0 - sinLon0*Xt - cosLon0 * ( sinLat0*Yt - cosLat0 * Zt );
	*Yg = m_Yg0 + cosLon0*Xt - sinLon0 * ( sinLat0*Yt - cosLat0 * Zt );
	*Zg = m_Zg0              + cosLat0 * Yt + sinLat0 * Zt;
}