#pragma once
#include <vector>

namespace ipl
{
	namespace geometry
	{
		struct LineSeg2D
		{
			double sp[2];
			double ep[2];
		};

		struct EDGE2D
		{
			LineSeg2D  ls;
			void *att;
		};

		typedef std::vector<ipl::geometry::EDGE2D>  POLYGON_MAP_MODEL;
	}

#ifndef MAX
#  define MIN(a,b)      ((a<b) ? a : b)
#  define MAX(a,b)      ((a>b) ? a : b)
#endif

#ifndef ABS
#  define ABS(x)        ((x<0) ? (-1*(x)) : x)
#endif


	struct iplEnvelope
	{
	public:
		double      minX;
		double      maxX;
		double      minY;
		double      maxY;

	public:
		iplEnvelope()
		{
			minX = maxX = minY = maxY = 0;
		}

		int  IsInit() const { return minX != 0 || minY != 0 || maxX != 0 || maxY != 0; }

		void merge(iplEnvelope const& sOther) {
			if (IsInit())
			{
				minX = MIN(minX, sOther.minX);
				maxX = MAX(maxX, sOther.maxX);
				minY = MIN(minY, sOther.minY);
				maxY = MAX(maxY, sOther.maxY);
			}
			else
			{
				minX = sOther.minX;
				maxX = sOther.maxX;
				minY = sOther.minY;
				maxY = sOther.maxY;
			}
		}
		void merge(double dfX, double dfY) {
			if (IsInit())
			{
				minX = MIN(minX, dfX);
				maxX = MAX(maxX, dfX);
				minY = MIN(minY, dfY);
				maxY = MAX(maxY, dfY);
			}
			else
			{
				minX = maxX = dfX;
				minY = maxY = dfY;
			}
		}

		int Intersects(iplEnvelope const& other) const
		{
			return minX <= other.maxX && maxX >= other.minX &&
				minY <= other.maxY && maxY >= other.minY;
		}

		int Contains(iplEnvelope const& other) const
		{
			return minX <= other.minX && minY <= other.minY &&
				maxX >= other.maxX && maxY >= other.maxY;
		}
	};

	/* -------------------------------------------------------------------- */
	/*      ogr_geometry.h related definitions.                             */
	/* -------------------------------------------------------------------- */
	/**
	* List of well known binary geometry types.  These are used within the BLOBs
	* but are also returned from OGRGeometry::m_geometryType to identify the
	* type of a geometry object.
	*/

	typedef enum
	{
		IPL_wkbUnknown = 0,         /**< unknown type, non-standard */
		IPL_wkbPoint = 1,           /**< 0-dimensional geometric object, standard WKB */
		IPL_wkbLineString = 2,      /**< 1-dimensional geometric object with linear
								   *   interpolation between Points, standard WKB */
		IPL_wkbPolygon = 3,         /**< planar 2-dimensional geometric object defined
								   *   by 1 exterior boundary and 0 or more interior
								   *   boundaries, standard WKB */
		IPL_wkbMultiPoint = 4,      /**< GeometryCollection of Points, standard WKB */
		IPL_wkbMultiLineString = 5, /**< GeometryCollection of LineStrings, standard WKB */
		IPL_wkbMultiPolygon = 6,    /**< GeometryCollection of Polygons, standard WKB */
		IPL_wkbGeometryCollection = 7, /**< geometric object that is a collection of 1
									  or more geometric objects, standard WKB */
		IPL_wkbNone = 100,          /**< non-standard, for pure attribute records */
		IPL_wkbLinearRing = 101,    /**< non-standard, just for createGeometry() */
		IPL_wkbPoint25D = 0x80000001, /**< 2.5D extension as per 99-402 */
		IPL_wkbLineString25D = 0x80000002, /**< 2.5D extension as per 99-402 */
		IPL_wkbPolygon25D = 0x80000003, /**< 2.5D extension as per 99-402 */
		IPL_wkbMultiPoint25D = 0x80000004, /**< 2.5D extension as per 99-402 */
		IPL_wkbMultiLineString25D = 0x80000005, /**< 2.5D extension as per 99-402 */
		IPL_wkbMultiPolygon25D = 0x80000006, /**< 2.5D extension as per 99-402 */
		IPL_wkbGeometryCollection25D = 0x80000007 /**< 2.5D extension as per 99-402 */
	} IPL_wkbGeometryType;



#define IPL_wkb25DBit 0x80000000
#define IPL_wkbFlatten(x)  ((IPL_wkbGeometryType) ((x) & (~IPL_wkb25DBit)))
}

