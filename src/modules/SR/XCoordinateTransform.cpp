#include <proj/proj_api.h>

#include "core/interface/IPlatform.h"
#include "spatialreference/interface/IGeoid.h"
#include "geometry/interface/IGeometryTransform.h"

#include "XCoordinateTransform.h"

#include <string.h>
#include <assert.h>


ipl::XEllipsoid ipl::XCoordinateTransform::m_WGS84;		// default is WGS84


using namespace ipl;

XCoordinateTransform::XCoordinateTransform()
{
	m_ftAvailable = false;
	m_bFastEnabled = false;

	m_fromCS = NULL;
	m_toCS = NULL;
	m_bSameCS = true;
	m_bSameVCS = true;

	memset( &m_forward, 0, sizeof( fastTransDATA ) );
	memset( &m_inverse, 0, sizeof( fastTransDATA ) );
}


XCoordinateTransform::~XCoordinateTransform()
{
	if( m_forward.grids )
		delete m_forward.grids;

	if( m_inverse.grids )
		delete m_inverse.grids;

//	if( m_ogrCT )
//		OCTDestroyCoordTransform_impl( m_ogrCT);

//	if( m_ogrCTInverse )
//		OCTDestroyCoordTransform_impl( m_ogrCTInverse );

}


void XCoordinateTransform::Initialize( ISpatialReference *fromCS, ISpatialReference *toCS )
{
	////////////////////// free resources //////////////
	if( m_forward.grids )
		delete m_forward.grids;

	if( m_inverse.grids )
		delete m_inverse.grids;

	memset( &m_forward, 0, sizeof( fastTransDATA ) );
	memset( &m_inverse, 0, sizeof( fastTransDATA ) );

	////////////////////////////////////////////
// 	m_fromCS0 = ref_ptr<ISpatialReference>(fromCS);
// 	m_toCS0 = ref_ptr<ISpatialReference>(toCS);
	m_fromCS0 = fromCS;
	m_toCS0 = toCS;

	m_fromCS = static_cast<XSpatialReference*> (fromCS);
	m_toCS = static_cast<XSpatialReference *> (toCS);

	m_bSameCS = (bool)m_fromCS->IsSameOgrCS( m_toCS );
	m_bSameDatum = (bool)m_fromCS->IsSameOgrGeogCS ( m_toCS );

	if( fromCS->VcsType() == toCS->VcsType() )
		m_bSameVCS = true;
	else
		m_bSameVCS = false;
}


/////////////////////////////////////////////////////////////////////////////////////////////
// 考虑垂直参考系统的坐标转换
//
//		地理坐标，地心坐标是中间的过渡
//
//	geoid is refereced to WGS84 ?
//		由于各基于各椭球的地理坐标相差不过几十米，高程异常改正误差可忽略不计？
//////////////////////////////////////////////////////////////////////////////////////////////
/* 
			proj 4的实现流程	proj 4 pj_transform( PJ *srcdefn, PJ *dstdefn, ...) 

1. Transform unusual input coordinate axis orientation to standard form if needed.                                        
2. Transform Z to meters if it isn't already.           
	3. Transform geocentric source coordinates to lat/long.  
	4. Transform source points to lat/long, if they aren't already.
		5. But if they are already lat long, adjust for the prime meridian if there is one in effect. 
			6. Do we need to translate from geoid to ellipsoidal vertical datum?   
       			7. Convert datums if needed, and possible.
				// 此处，geoid没有考虑是否在wgs84下 !!!!! 可能的解释是geoid是平滑的，且分辨率很低
			8. Do we need to translate from geoid to ellipsoidal vertical datum? 
		9. But if they are staying lat long, adjust for the prime meridian if there is one in effect. 
	10.Transform destination latlong to geocentric if required.
	11. Transform destination points to projection coordinates, if desired.
12. If a wrapping center other than 0 is provided, rewrap around the suggested center (for latlong coordinate systems only). 
13. Transform Z from meters if needed.                             
14. Transform normalized axes into unusual output coordinate axis orientation if needed.                                          
*/

/* 原有代码实现 
bool XCoordinateTransform::Transform(XSpatialReference *fromCS , XSpatialReference *toCS, iplPOINT3D *pt )
{
	double X, Y, Z;

	Z = pt->Z;

	if( fromCS->IsGeographic() ) {
		X = pt->X *DEG_TO_RAD;
		Y = pt->Y *DEG_TO_RAD;
	}
	else	{
		X = pt->X;
		Y = pt->Y;
	}

	// 残留问题，高程系统没有设置
	pj_transform( fromCS->GetPJ(), toCS->GetPJ(), 1, 0, &X, &Y, &Z );
	
	if( toCS->IsGeographic() ) {
		pt->X = X*RAD_TO_DEG;
		pt->Y = Y*RAD_TO_DEG;
	}
	else {
		pt->X = X;
		pt->Y = Y;
	}
	
	pt->Z = Z;

	return true;

	//////////////////////////////////////////////////////////////////////////
	//getPlatform()->logPrint( ORS_LOG_DEBUG, "enter XCoordinateTransform::Transform");

	double latitude, longtitude, height, xg, yg, zg;
	double WGS84_height, geoid_height;

	// 投影坐标到地理坐标
	if( fromCS->IsProjected() ) {
		if( fromCS->Projected_To_Geodetic( pt->X, pt->Y, &latitude, &longtitude) == false )
			return false;
	}
	else	{
		latitude   = pt->Y *DEG_TO_RAD;
		longtitude = pt->X *DEG_TO_RAD;
	}

	// Datum Transformation Stage
	WGS84_height = geoid_height = height = pt->Z;

	if( !m_bSameDatum || !m_bSameVCS )
	{
		// 椭球变换
		// Shift to WGS84
		if( false == m_bSameDatum )
		{
			// If this datum requires grid shifts, then apply it to geodetic coordinates.
			// 有现成的格网， 精度较高
// 			if( fromCS->DatumType() == PJD_GRIDSHIFT ) {
// 				// proj4.5 use (longtitude, latitude),
// 				assert(false);
// 				// proj4.6
// 				// pj_apply_gridshift( pj_param( fromCS->GetPJ()->params,"snadgrids").s, 0, 1, 1, &longtitude, &latitude, &WGS84_height );
// 			}
// 			else	
			{
				// 使用地心坐标变换
				if( fromCS->Geodetic_To_Geocentric( latitude, longtitude, height, &xg, &yg, &zg ) == false )
					return false;
				if( fromCS->Geocentric_To_WGS84( &xg, &yg, &zg ) == false )
					return false;
				if( m_WGS84.Geocentric_To_Geodetic( xg, yg, zg, &latitude, &longtitude, &WGS84_height ) == false )
					return false;
			}
			height = WGS84_height;
		}

		// 大地水准面高程改化为WGS84椭球高
		// apply geoid correction, from geoid to WGS84

		if( !m_bSameVCS ) {
			WGS84_height = geoid_height;
			if( fromCS->GetGeoid() )	{
				if( !fromCS->GetGeoid()->Geoid2Ellipsoid( latitude, longtitude, geoid_height, &WGS84_height) == false )
					return false;
				height = WGS84_height;
			}

			// WGS84椭球高改化为大地水准面高程
			// apply geoid correction, from WGS84 to geoid
			if( toCS->GetGeoid() )	{
				if( !toCS->GetGeoid()->Geoid2Ellipsoid( latitude, longtitude,WGS84_height, &geoid_height) )
					return false;
				height = geoid_height;
			}
		}

		// 椭球变换
		// Shift from WGS84
		if( !m_bSameDatum ) {
			// 有现成的格网
// 			if( toCS->DatumType() == PJD_GRIDSHIFT )
// 			{
// 				// proj4.5 use (longtitude, latitude),
// 
// 				assert(false);
// 				// proj4.6
// 				// pj_apply_gridshift( pj_param( toCS->GetPJ()->params,"snadgrids").s, 1, 1, 1, &longtitude, &latitude, &height );
// 			}
// 			else	
			{
				if( m_WGS84.Geodetic_To_Geocentric( latitude, longtitude, WGS84_height, &xg, &yg, &zg ) == false )
					return false;

				if( toCS->Geocentric_From_WGS84( &xg, &yg, &zg ) == false )
					return false;

				if( toCS->Geocentric_To_Geodetic( xg, yg, zg, &latitude, &longtitude, &height ) == false )
					return false;

				if (toCS->VcsType() == vcsGEOID )
					height = geoid_height;
				else if (toCS->VcsType() == vcsNONE)
					height = 0.0;
			}
		}
	}

	// 地理坐标到投影
	if( toCS->IsProjected() )
		toCS->Geodetic_To_Projected( latitude, longtitude, &pt->X, &pt->Y );
	else	{
		pt->X = longtitude*RAD_TO_DEG;
		pt->Y = latitude*RAD_TO_DEG;
	}
	pt->Z = height;

	//getPlatform()->logPrint( ORS_LOG_DEBUG, "leaving XCoordinateTransform::Transform");
	
	return true;
}

*/


bool XCoordinateTransform::Transform(XSpatialReference *fromCS , XSpatialReference *toCS, iplPOINT3D *pt )
{
	if( srsTangentPLANE == fromCS->SrsType() || srsGEOCENTRIC == fromCS->SrsType() ||
		srsTangentPLANE == toCS->SrsType() || srsGEOCENTRIC == toCS->SrsType() ) 
	{	
		// 只适合比较简单的或者同一参考椭球的情况，因为椭球变换没有实现
		fromCS->ToGeocentric_WGS84( *pt, pt );
		toCS->FromGeocentric_WGS84( *pt, pt );
	}
	else	{	// 地理坐标或投影坐标
		double X, Y, Z;

		if( fromCS->IsGeographic() ) {
			X = pt->X *DEG_TO_RAD;
			Y = pt->Y *DEG_TO_RAD;
		}
		else	{
			X = pt->X;
			Y = pt->Y;
		}

		////////////////////////////////////////////////////
		// 大地水准面高程改化为WGS84椭球高
		// apply geoid correction, from geoid to WGS84
		if( fromCS->GetGeoid() )	{
			double lat, lon, h;

			if( fromCS->IsGeographic() ) {
				lat = Y;
				lon = X;
			}
			else	{
				fromCS->ToGeoGraphic( X, Y, Z, &lon, &lat, &h );

				lat = DEG_TO_RAD*lat;
				lon = DEG_TO_RAD*lon;
				
				assert( Z == h );
			}

			if( !fromCS->GetGeoid()->Geoid2Ellipsoid( lat, lon, Z, &Z) == false )
				return false;
		}
		else
			Z = pt->Z;	
		
		/////////////////////////////////////////////////////////////////
		pj_transform( fromCS->GetPJ(), toCS->GetPJ(), 1, 0, &X, &Y, &Z );
		/////////////////////////////////////////////////////////////////
		
		////////////////////////////////////////////////////
		// WGS84椭球高改化为大地水准面高程
		// apply geoid correction, from WGS84 to geoid
		if( toCS->GetGeoid() )	{
			double lat, lon, h;

			if( toCS->IsGeographic() ) {
				lat = Y;
				lon = X;
			}
			else	{
				fromCS->ToGeoGraphic( X, Y, Z, &lon, &lat, &h );
				
				lat = DEG_TO_RAD*lat;
				lon = DEG_TO_RAD*lon;
				
				assert( Z == h );
			}

			if( !toCS->GetGeoid()->Ellipsoid2Geoid( lat, lon, Z, &Z) )
				return false;
		}
		
		////////////////////////////////////////////////////
		if( toCS->IsGeographic() ) {
			pt->X = X*RAD_TO_DEG;
			pt->Y = Y*RAD_TO_DEG;
		}
		else {
			pt->X = X;
			pt->Y = Y;
		}
		
		pt->Z = Z;
	}
	
	return true;
}



long XCoordinateTransform::Transform_slow( const iplPOINT3D *ptsFrom, int n, iplPOINT3D *ptsTo)
{
	int i;
	for( i=0; i<n; i++)	
	{
		ptsTo[i] = ptsFrom[i];
		
		if( Transform( m_fromCS, m_toCS, ptsTo+i ) == false )
			return false ;
	}

	return true;
};


long XCoordinateTransform::Transform( const iplPOINT3D *ptsFrom, int n, iplPOINT3D *ptsTo)
{
	assert( m_fromCS != NULL );
	assert( m_toCS != NULL );

	if( m_ftAvailable && m_bFastEnabled )
		return	Transform_fast( ptsFrom, n, ptsTo );

	if( ptsFrom != ptsTo )
		memcpy(ptsTo, ptsFrom, n*sizeof(iplPOINT3D));

	if( m_bSameCS && m_bSameVCS )
		return true;

	return Transform_slow( ptsFrom,  n, ptsTo );
};


long XCoordinateTransform::Inverse_slow( const iplPOINT3D *ptsFrom, int n, iplPOINT3D *ptsTo)
{
	int i;
	for( i=0; i<n; i++) {
		ptsTo[i] = ptsFrom[i];
		if( Transform( m_toCS, m_fromCS, ptsTo+i ) == false )
			return false;
	}

	return true;
};


long XCoordinateTransform::Inverse( const iplPOINT3D *ptsFrom, int n, iplPOINT3D *ptsTo)
{
	assert( m_fromCS != NULL );
	assert( m_toCS != NULL );

	if( m_ftAvailable && m_bFastEnabled )
		return Inverse_fast( ptsFrom, n, ptsTo );

	if( ptsFrom != ptsTo )
		memcpy( ptsTo, ptsFrom, n*sizeof(iplPOINT3D));

	if( m_bSameCS && m_bSameVCS )
		return true;

	return Inverse_slow( ptsFrom, n, ptsTo );

};


//////////////////////////////////////////////////////////////////////////

bool XCoordinateTransform::MakeAffineParameters( fastTransDATA *trfData, double maxErr, bool bIsInverse )
{
	iplPOINT3D ptsFrom[5], ptsTo[5];
	
	bool bSrcGraphic, bDstGraphic;
	
	if( !bIsInverse ) {
		bSrcGraphic = m_fromCS->IsGeographic();
		bDstGraphic = m_toCS->IsGeographic();
	}
	else {
		bSrcGraphic = m_toCS->IsGeographic();
		bDstGraphic = m_fromCS->IsGeographic();
	}

	double dxy, dxy1;

	if( bSrcGraphic )
		dxy = 0.002;	// angle
	else
		dxy = 250;	// length

	//getPlatform()->logPrint(ORS_LOG_DEBUG, "InitFastTransform: create orsIGeometryTransform3D " );
	
	ref_ptr<IGeometryTransform3D> theAffine(IPL_CREATE_OBJECT(IGeometryTransform3D, IPL_GEOMETRY_TRANSFORM3D_AFFINE25D));

	//////////////////////////////////////////////////////////////////////////
	// 迭代求最佳逼近分块大小
	{
		dxy1 = dxy;
		double mx, my, mz;

		for( int i=0; i<50; i++ )
		{
			double x = trfData->minX;
			double y = trfData->minY;

			ptsFrom[0].X = x+dxy/2;		ptsFrom[0].Y = y+dxy/2;
			ptsFrom[1].X = x;			ptsFrom[1].Y = y;
			ptsFrom[2].X = x+dxy;		ptsFrom[2].Y = y;
			ptsFrom[3].X = x+dxy;		ptsFrom[3].Y = y+dxy;
			ptsFrom[4].X = x;			ptsFrom[4].Y = y+dxy;
			
			int k;
			for( k=0; k<5; k++ )
				ptsFrom[k].Z = 100;
			
			// 正算			
			if( !bIsInverse ) {
				if( Transform_slow( ptsFrom, 5, ptsTo ) == false )
					return false;
			}
			// 反算
			else	{
				if( Inverse_slow( ptsFrom, 5, ptsTo ) == false )
					return false;
			}
			
			theAffine->Initialize( ptsFrom, 5, ptsTo );
			theAffine->GetMeanError( &mx, &my, &mz );
			
			if( mx > maxErr || my > maxErr )
				break;
			
			//////////////////////////////////////////////////////////////////////////
			// 右上角
			x = trfData->maxX;
			y = trfData->maxY;
			
			ptsFrom[0].X = x+dxy/2;		ptsFrom[0].Y = y+dxy/2;
			ptsFrom[1].X = x;			ptsFrom[1].Y = y;
			ptsFrom[2].X = x+dxy;		ptsFrom[2].Y = y;
			ptsFrom[3].X = x+dxy;		ptsFrom[3].Y = y+dxy;
			ptsFrom[4].X = x;			ptsFrom[4].Y = y+dxy;		
			
			// 正算			
			if( !bIsInverse ) {
				if( Transform_slow( ptsFrom, 5, ptsTo ) == false )
					return false;
			}
			// 反算
			else	{
				if( Inverse_slow( ptsFrom, 5, ptsTo ) == false )
					return false;
			}
			
			theAffine->Initialize( ptsFrom, 5, ptsTo );
			theAffine->GetMeanError( &mx, &my, &mz );
			
			if( mx > maxErr || my > maxErr || mz > maxErr )
				break;

			dxy1 = dxy;
			dxy *= 2;
		}
	}

	trfData->dx = trfData->dy = dxy1;

	//////////////////////////////////////////////////////////////////////////

	int i,j;

	// set origin
	trfData->minX = trfData->minX - fmod( trfData->minX, trfData->dx );
	trfData->minY = trfData->minY - fmod( trfData->minY, trfData->dy );
	// calculate number of grids
	trfData->nx = int( ( trfData->maxX - trfData->minX ) / trfData->dx + 1 );
	trfData->ny = int( ( trfData->maxY - trfData->minY ) / trfData->dy + 1 );

	if( trfData->nx < 1 )
		trfData->nx = 1;

	if( trfData->ny < 1 )
		trfData->ny = 1;

	//
	trfData->grids = new affineTransPARA[ trfData->nx*trfData->ny ];

	///////////////////////////////////////////////////////////
	double x,y,  mx, my, mz;

	//////////////////////////////////////


	affineTransPARA *grids = trfData->grids;
	y = trfData->minY;
	for( i = 0; i<trfData->ny; i++)
	{
		x = trfData->minX;
		for( j = 0; j < trfData->nx; j++)
		{
			// inner point
			//memset( ptsFrom, 0, 5*sizeof( iplPOINT3D ));

			ptsFrom[0].X = x+trfData->dx/2;		ptsFrom[0].Y = y+trfData->dy/2;
			ptsFrom[1].X = x;					ptsFrom[1].Y = y;
			ptsFrom[2].X = x+trfData->dx;		ptsFrom[2].Y = y;
			ptsFrom[3].X = x+trfData->dx;		ptsFrom[3].Y = y+trfData->dy;
			ptsFrom[4].X = x;					ptsFrom[4].Y = y+trfData->dy;

			int k;
			for( k=0; k<5; k++ )
				ptsFrom[k].Z = 100;

			// 正算

			if( !bIsInverse ) {
				//getPlatform()->logPrint(ORS_LOG_DEBUG, "Transform_slow" );

				if( Transform_slow( ptsFrom, 5, ptsTo ) == false )
					return false;
			}
			// 反算
			else	{
				//getPlatform()->logPrint(ORS_LOG_DEBUG, "Inverse_slow" );

				if( Inverse_slow( ptsFrom, 5, ptsTo ) == false )
					return false;
			}

			theAffine->Initialize( ptsFrom, 5, ptsTo );
			theAffine->GetParameter( &grids->pcSrc, &grids->pcDst, grids->a, grids->b,  grids->c );

			theAffine->GetMeanError( &mx, &my, &mz );

			// 增加Z方向的考虑，比如切平面高程和椭球高程的转换限差

			if( mx < maxErr && my < maxErr && mz < maxErr )
				return false;

			grids++;	x += trfData->dx;
		}
		y += trfData->dy;
	}

	return true;
}

///////////////////////////////////////////////////////////////////////////
//
//	fast transformation between tangent plane coordinates, geographic coordinates
//	1. (X, Y) - (long, lat)
//  2. (X, Y)1 - (X,Y)2
//
// other transformation are not allowed
////////////////////////////////////////////////////////////////////////////
bool XCoordinateTransform::InitFastTransform( double minX, double minY, double maxX, double maxY, double maxErr)
{
	int i;

	//getPlatform()->logPrint( ORS_LOG_DEBUG, "enter XCoordinateTransform::InitFastTransform");

	if( m_bSameCS && m_bSameVCS )
		return true;

	// 释放原先的资源
	if( m_ftAvailable )	{
		if( m_forward.grids )	{
			delete m_forward.grids;
			m_forward.grids = NULL;
		}
		if( m_inverse.grids )	{
			delete m_inverse.grids;
			m_inverse.grids = NULL;
		}
		m_ftAvailable = false;
	}

	//////////////////////////////////////////////////////////////////////////

	m_forward.minX = minX;	m_forward.minY = minY;
	m_forward.maxX = maxX;	m_forward.maxY = maxY;

	//////////////////////////////////////////////////////////////////////////
	double dx0 = maxX - minX;
	double dy0 = maxY - minY;

	iplPOINT3D ptsFrom[5], ptsTo[5];
	
	memset( ptsFrom, 0, 5*sizeof( iplPOINT3D ));
	
	ptsFrom[0].X = minX;	ptsFrom[0].Y = minY;
	ptsFrom[1].X = maxX;	ptsFrom[1].Y = minY;
	ptsFrom[2].X = maxX;	ptsFrom[2].Y = maxY;
	ptsFrom[3].X = minX;	ptsFrom[3].Y = maxY;
	
	if( Transform_slow( ptsFrom, 4, ptsTo ) == false )
		return false;
	
	minX = minY = 99999999;
	maxX = maxY = -99999999;
	for( i=0; i<4; i++)
	{
		if( minX > ptsTo[i].X )
			minX = ptsTo[i].X;
		else if( maxX < ptsTo[i].X )
			maxX = ptsTo[i].X;
		
		if( minY > ptsTo[i].Y )
			minY = ptsTo[i].Y;
		else if( maxY < ptsTo[i].Y )
			maxY = ptsTo[i].Y;
	}

	double dx1 = maxX - minX;
	double dy1 = maxY - minY;
	double scale = sqrt( (dx1*dx1+dy1*dy1)/(dx0*dx0+dy0*dy0) );

	//////////////////////////////////////////////////////////////////////////

	if( MakeAffineParameters( &m_forward, scale*maxErr ) == false ) {
		getPlatform()->logPrint( IPL_LOG_ERROR, "XCoordinateTransform: MakeAffineParameters failed");
		//std::cout << "XCoordinateTransform: MakeAffineParameters failed" << std::endl;
		return false;
	}

	//////////////////////////////// for inverse ///////////////

	//////////////////////
	m_inverse.minX = minX;	m_inverse.minY = minY;
	m_inverse.maxX = maxX;	m_inverse.maxY = maxY;

	if( MakeAffineParameters( &m_inverse, maxErr, true ) == false ) {
		getPlatform()->logPrint( IPL_LOG_ERROR, "XCoordinateTransform: MakeAffineParameters failed");
		return false;
	}

	///////////////////////////////////////////////////////////

	m_ftAvailable = true;
	m_bFastEnabled = true;

	//getPlatform()->logPrint( ORS_LOG_DEBUG, "leaving XCoordinateTransform::InitFastTransform");

	return true;
}


long XCoordinateTransform::Transform_fast( const iplPOINT3D *ptsFrom, int n, iplPOINT3D *ptsTo )
{
	int i,j, k;

	double dx, dy, dz;

	for( k=0; k<n; k++)
	{
		j = int( ( ptsFrom->X - m_forward.minX ) / m_forward.dx );
		i = int( ( ptsFrom->Y - m_forward.minY ) / m_forward.dy );

		if( i < 0 || j < 0 || j >= m_forward.nx || i >= m_forward.ny )
			return	Transform_slow( ptsFrom, 1, ptsTo );

		affineTransPARA *tra = m_forward.grids + i*m_forward.nx + j;

		dx = ptsFrom[k].X - tra->pcSrc.X;
		dy = ptsFrom[k].Y - tra->pcSrc.Y;
		dz = ptsFrom[k].Z - tra->pcSrc.Z;

		ptsTo->X = tra->pcDst.X + tra->a[0]*dx + tra->a[1]*dy;
		ptsTo->Y = tra->pcDst.Y + tra->b[0]*dx + tra->b[1]*dy;

		// 增加改正量
		ptsTo->Z = tra->pcDst.Z + tra->c[0]*dx + tra->c[1]*dy + dz;

		ptsFrom++;		ptsTo++;
	}

	return true;
}



long XCoordinateTransform::Inverse_fast( const iplPOINT3D *ptsFrom, int n, iplPOINT3D *ptsTo )
{
	int i,j, k;

	double dx, dy, dz;

	for( k=0; k<n; k++)
	{
		j = int( ( ptsFrom->X - m_inverse.minX ) / m_inverse.dx );
		i = int( ( ptsFrom->Y - m_inverse.minY ) / m_inverse.dy );

		if( i < 0 || j < 0 || j >= m_inverse.nx || i >= m_inverse.ny )
			return	Inverse_slow( ptsFrom, 1, ptsTo );

		affineTransPARA *tra = m_inverse.grids + i*m_inverse.nx + j;

		dx = ptsFrom[k].X - tra->pcSrc.X;
		dy = ptsFrom[k].Y - tra->pcSrc.Y;
		dz = ptsFrom[k].Z - tra->pcSrc.Z;

		ptsTo->X = tra->pcDst.X + tra->a[0]*dx + tra->a[1]*dy;
		ptsTo->Y = tra->pcDst.Y + tra->b[0]*dx + tra->b[1]*dy;
		ptsTo->Z = tra->pcDst.Z + tra->c[0]*dx + tra->c[1]*dy + dz;


		ptsFrom++;		ptsTo++;
	}

	return true;
}


ICoordinateTransform *CreateCoordTransform()
{
	return new XCoordinateTransform;
}
