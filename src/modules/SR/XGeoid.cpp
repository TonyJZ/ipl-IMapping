#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>

#include "commonAPIs/iplutility.h"
#include "XGeoid.h"

#define PI 3.14159265358979323e0

using namespace ipl;

XGeoid::XGeoid()
{
	m_pData = NULL;
	m_errCount = 0;
}


ipl::XGeoid::~XGeoid()
{
	if( NULL != m_pData )
		delete m_pData;
}


bool ipl::XGeoid::Initialize( const std::string & wktGeoid )
{
	if( wktGeoid == "EGM96" || wktGeoid == "egm96"  )
		return LoadData_EGM96();

	return LoadData_Leica( wktGeoid );
}



bool ipl::XGeoid::GetDeltaHeight( double lat_r, double lon_r, double *dh )
{
	if( NULL == m_pData )
	{
		getPlatform()->logPrint(IPL_LOG_ERROR, "geoid is not initialized");
		return false;
	}

	if (( lat_r < m_lat0 ) || (lat_r > m_lat1 ))
	{
		if( m_errCount < 2 ) {
			getPlatform()->logPrint(IPL_LOG_ERROR, "geoid: latitude out of range ");
			m_errCount++;
		}
		return false;
	}
	if ((lon_r < m_lat0 ) || (lon_r > m_lat1 ))
	{
		if( m_errCount < 2 ) {
			getPlatform()->logPrint(IPL_LOG_ERROR, "geoid: Longtitude out of range ");
			m_errCount++;
		}
		return false;
	}

	///////////////////
	// 度
	double lat, lon;

	lat = 90.0 - lat_r * 180.0 / PI;
	lon = lon_r * 180.0 / PI;

	if( lon < 0.0 )
		lon += 360.0;

	/////////////////////////////////////////////////////////////
	// 格网坐标，(0,0) of Geoid Height grid is at Northwest corner
	double x, y;

	x = lon / m_dLong;
	y = lat / m_dLat;

	// 格网“左上”角
	int row0, col0;

	col0 = x;
	if( (col0 + 1) == m_nCols )
		col0--;

	row0 = y;
	if( (row0 + 1) == m_nRows )
		row0--;

	// 四个角点的高程值
	float h00, h01, h10, h11;

	// 左上角索引
	long index = row0*m_nCols + col0;
	h00 = m_pData[ index ];	h01 = m_pData[ index+ 1 ];

	// 左下角索引
	index += m_nCols;
	h10 = m_pData[ index ];	h11 = m_pData[ index + 1 ];

	//////////////////////////////////////////////////////////////////////////
	//  Perform Bi-Linear Interpolation to compute Height above Ellipsoid
	// 格网单元内偏移
	double dx, dy;

	dx = x - col0;
	dy = y - row0;

	double h0, h1;

	h0 = h00 + dx * ( h01 - h00 );
	h1 = h10 + dx * ( h11 - h10 );

	*dh = h0 + dy*( h1 - h0 );

	return true;
}



bool ipl::XGeoid::Ellipsoid2Geoid(double latitude, double longitude, double ellipsoidHeight, double *geoidHeight)
{
	if( m_pData == NULL )
		return false;

	double dh;

	if( !GetDeltaHeight( latitude, longitude, &dh ) )
		return false;

	*geoidHeight = ellipsoidHeight - dh;

	return true;
}


bool ipl::XGeoid::Geoid2Ellipsoid(double latitude, double longitude, double geoidHeight, double *ellipsoidHeight )
{
	if( NULL == m_pData )
		return false;

	double dh;

	if( !GetDeltaHeight ( latitude, longitude, &dh ) )
		return false;

	*ellipsoidHeight = geoidHeight + dh;

	return true;
}


std::string ipl::XGeoid::GetPath()
{
	char    *envPathName = getenv( "GEOID_DATA" );

	std::string pathName;

	// Check the environment for a user provided path, else current directory;
	// Build a File Name, including specified or default path:
	if( envPathName != NULL )
	{
		pathName = pathName + "/";
	}
	else
	{
		pathName = "../etc/geoid/";
	}

	return pathName;
}

//////////////////////////////////////////////////////////////////////////
/*
* The function Initialize_Geiud reads geoid separation data from a file in
* the current directory and builds the geoid separation table from it.
* If the separation file can not be found or accessed, an error code of
* GEOID_FILE_OPEN_ERROR is returned, If the separation file is incomplete
* or improperly formatted, an error code of GEOID_INITIALIZE_ERROR is returned,
* otherwise GEOID_NO_ERROR is returned.
*/

struct  egm96GeoidHEAHER
{
	float	lat0;
	float	lat1;
	float	lon0;
	float	lon1;
	float	dLat;
	float	dLon;
};


// for 4 bytes types
void SwapBytes( unsigned char *values, int n, int itemBytes )
{
	unsigned char temp;

	for( int i=0; i<n; i++ )
	{
		unsigned char* swap = values;

		temp = swap[0];	swap[0] = swap[3];	swap[3] = temp;
		temp = swap[1];	swap[1] = swap[2];	swap[2] = temp;

		values += itemBytes;
	}
}

#ifndef LITTLE_ENDIAN
#define LITTLE_ENDIAN
#endif // !LITTLE_ENDIAN

bool ipl::XGeoid::LoadData_EGM96()
{
	if( NULL != m_pData )
		return true;

	//////////////////////////////////////////////////////////////////////////

	std::string etcDir;

	get_directory_etc( etcDir );

	std::string fileName = etcDir + "/geoid/";

	fileName += "egm96.grd";

	// 已改为二进制文件
	FILE  *fp;
	if (( fp = fopen( fileName.c_str(), "rb" ) ) == NULL) {
		char msg[256];
		sprintf( "Can not open %s", fileName.c_str() );
		getPlatform()->logPrint(IPL_LOG_ERROR, msg);
		return false;
	}

	m_lat0 = -90;	m_lat1 = 90;
	m_long0 = 0;	m_long1 = 360;
	m_nCols = 1441;	m_nRows = 721;
	m_dLat = m_dLong = 1.0 / 4;

	//////////////////////////////////////////////////////////////////////////
	egm96GeoidHEAHER h;
	fread( &h, sizeof(egm96GeoidHEAHER), 1, fp );

	// egm96.grd 是按
#ifdef LITTLE_ENDIAN
	SwapBytes( (unsigned char *)&h, 4, sizeof(float) );
	SwapBytes( (unsigned char *)&h.dLat, 2, sizeof(float) );
#endif

	//  Determine if header read properly, or NOT
	if( h.lat0 != -90.0 ||	h.lat1 !=   90.0 ||
		h.lon0 !=   0.0 ||	h.lon1 !=  360.0 ||
		h.dLat != m_dLat || h.dLon != m_dLong )
	{
		fclose(fp);
		getPlatform()->logPrint(IPL_LOG_ERROR, "geoid initialization error");
		return false;
	}

	//////////////////////////////////////////////////////////////////////////
	// Extract geoid delta height from the file:

	int nGeoidElevs = m_nRows*m_nCols;

	m_pData = new float[nGeoidElevs];
	int nRead = fread( m_pData, sizeof(float), nGeoidElevs, fp);
	fclose(fp);

	//  Determine if all values of file read properly
	if ( nRead != nGeoidElevs)
	{
		delete m_pData;	m_pData = NULL;
		getPlatform()->logPrint(IPL_LOG_ERROR, "geoid initialization error");
		return false;
	}

#ifdef LITTLE_ENDIAN
	SwapBytes( (unsigned char *)m_pData, nGeoidElevs, sizeof(float ) );
#endif

	return true;
}


//////////////////////////////////////////////////////////////////////////
struct  leicaGeoidHEAHER
{
	double		lat0;
	double		long0;
	double		dLat;
	double		dLong;
	ipl_int32	nRows;
	ipl_int32	nCols;
	ipl_int32	nKind;
};


bool ipl::XGeoid::LoadData_Leica( const std::string &fileName )
{
	std::string filePath = GetPath();

	filePath += fileName;

	// 已改为二进制文件
	FILE  *fp;
	if (( fp = fopen( filePath.c_str(), "rb" ) ) == NULL) {
		char msg[256];
		sprintf(msg, "Can not open %s", filePath.c_str() );
		getPlatform()->logPrint(IPL_LOG_ERROR, msg);
		return false;
	}

	//
	leicaGeoidHEAHER h;
	fread( &h,sizeof(leicaGeoidHEAHER), 1, fp );

#ifdef BIG_ENDIAN
	SwapBytes( (unsigned char *)&h.lat0, 4, sizeof(double) );
	SwapBytes( (unsigned char *)&h.nRows, 3, sizeof(ipl_int32) );
#endif

	m_lat0 = h.lat0;	m_long0 = h.long0;
	m_dLat = h.dLat;	m_dLong = h.dLong;
	m_nRows = h.nRows;	m_nCols = h.nCols;

	m_lat1  = m_lat0  + (m_nRows-1) * m_dLat;
	m_long1 = h.long0 + (m_nCols-1) * m_dLong;

	//////////////////////////////////////////////////////////////////////////
	int nGeoidElevs = m_nRows*m_nCols;

	m_pData = new float[nGeoidElevs];
	int nRead = fread( m_pData, sizeof(float), nGeoidElevs, fp);
	fclose(fp);

	//  Determine if all values of file read properly
	if ( nRead != nGeoidElevs)
	{
		delete m_pData;	m_pData = NULL;
		getPlatform()->logPrint(IPL_LOG_ERROR, "geoid initialization error");
		return false;
	}

#ifdef BIG_ENDIAN
	SwapBytes( (unsigned char *)m_pData, nGeoidElevs, sizeof(float ) );
#endif

	return true;
}
