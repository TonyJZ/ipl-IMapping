#pragma once

/*
1 Coordinate System WKT
2 <coordinate system> = <horz cs> | <geocentric cs> | <vert cs> | <compd cs> | <fitted cs> | <local cs>
3 <horz cs>         = <geographic cs> | <projected cs>
4 <projected cs>     = PROJCS["", <geographic cs>, <projection>, {<parameter>,}* <linear unit> {,<twin axes>}{,<authority>}]
5 <projection>     = PROJECTION["" {,<authority>}]
6 <geographic cs>   = GEOGCS["", <datum>, <prime meridian>, <angular unit> {,<twin axes>} {,<authority>}]
7 <datum>          = DATUM["", <spheroid> {,<to wgs84>} {,<authority>}]
8 <spheroid>        = SPHEROID["", <semi-major axis>, <inverse flattening> {,<authority>}]
9 <semi-major axis>    = <number>
10 <inverse flattening> = <number>
11 <prime meridian>     = PRIMEM["", <longitude> {,<authority>}]
12 <longitude>          = <number>
13 <angular unit>       = <unit>
14 <linear unit>        = <unit>
15 <unit>               = UNIT["", <conversion factor> {,<authority>}]
16 <conversion factor>  = <number>
17 <geocentric cs>      = GEOCCS["", <datum>, <prime meridian>, <linear unit> {,<axis>, <axis>, <axis>} {,<authority>}]
18 <authority>          = AUTHORITY["", ""]
19 <vert cs>            = VERT_CS["", <vert datum>, <linear unit>, {<axis>,} {,<authority>}]
20 <vert datum>         = VERT_DATUM["", <datum type> {,<authority>}]
21 <datum type>         = <number>
22 <compd cs>           = COMPD_CS["", <head cs>, <tail cs> {,<authority>}]
23 <head cs>            = <coordinate system>
24 <tail cs>            = <coordinate system>
25 <twin axes>          = <axis>, <axis>
26 <axis>               = AXIS["", NORTH | SOUTH | EAST | WEST | UP | DOWN | OTHER]
27 <to wgs84s>          = TOWGS84[<seven param>]
28 <seven param>        = <dx>, <dy>, <dz>, <ex>, <ey>, <ez>, <ppm>
29 <dx>                 = <number>
30 <dy>                 = <number>
31 <dz>                 = <number>
32 <ex>                 = <number>
33 <ey>                 = <number>
34 <ez>                 = <number>
35 <ppm>                = <number>
36 <fitted cs>          = FITTED_CS["", <to base>, <base cs>]
37 <to base>            = <math transform>
38 <base cs>            = <coordinate system>
39 <local cs>           = LOCAL_CS["", <local datum>, <unit>, <axis>, {,<axis>}* {,<authority>}]
40 <local datum>        = LOCAL_DATUM["", <datum type> {,<authority>}]

*/

#include <GDAL/cpl_conv.h>
#include <GDAL/ogr_srs_api.h>
#include <proj/proj_api.h>

#include "core/ipldef.h"
#include "spatialreference/interface/ISpatialReference.h"

#include "XEllipsoid.h"
#include "XGeoid.h"

namespace ipl
{

#define MAX_GTIF_PROJPARMS 	10
#define undefined -1

	enum csWktTYPE {	// <horz cs> | <geocentric cs> | <vert cs> | <compd cs> | <fitted cs> | <local cs>
		cswGEOCENTRIC,
		cswHORZ,
		cswVERT,
		cswCOMPD,
		cswFITTED,
		cswLOCAL
	};

	enum hcsTYPE {
		hcsGEOGRAPHIC = 1,
		hcsPROJECTED = 2
	};


	enum csAXIS {
		NORTH, SOUTH,
		EAST, WEST,
		UP, DOWN,
		OTHER
	};

	struct SPHEROID {
		double semiMajor;
		double inverFlattening;
		double semiMinor;
	};


	struct toWGS84 {
		double dx, dy, dz;
		double ex, ey, ez;
		double ppm;
	};


	struct DATUM {
		long	epsgCode;
		SPHEROID spheroid;
		toWGS84  toWgs84;
	};


	struct geographicCS {
		long	epsgCode;
		DATUM	datum;
		double pmLongitude;
		double UOMLengthInMeters;
		double UOMAngleInDegrees;
		csAXIS xAxis, yAxis;
	};


	enum projTYPE {

	};

	struct projectedCS {
		long epsgCode;

		geographicCS gcs;

		projTYPE projType;

		double para[7];

		double UOMLengthInMeters;
		csAXIS xAxis, yAxis;
	};

	struct geocentricCS {
		long epsgCode;
		DATUM datum;
		double pmLongitude;
		double UOMLengthInMeters;
		csAXIS xAxis, yAxis, zAxis;
	};

	struct hcsINFO {
		hcsTYPE	  hcsType;

		union {
			geographicCS geog;
			projectedCS proj;
			geocentricCS geoc;
		};

	};


	struct vertDATUM {
		long code;
		long type;
	};


	struct vcsINFO {
		long code;
		vertDATUM datum;
		double UOMLengthInMeters;
		csAXIS axis;
	};


	// all code are from EPSG
	struct wktCsINFO {
		csWktTYPE cswType;

		union {
			hcsINFO hcs;
			vcsINFO vcs;
		};
	};


	struct GTIFDefn
	{
		srsTYPE srsType;

		long	PCS;		// Projected Type Coordinate Reference Type
		long	GCS;		// GGeographic Type Coordinate Reference Type

		long	UOMLength;
		double	UOMLengthInMeters;

		long    UOMAngle;
		double  UOMAngleInDegrees;

		long	Datum;
		long	PM;		// Prime meridian

						/** Decimal degrees of longitude between this prime meridian and
						Greenwich.  Prime meridians to the west of Greenwich are negative. */
		double	PMLongToGreenwich;

		long	Ellipsoid;

		double	SemiMajor;		// The length of the semi major ellipse axis in meters
		double	SemiMinor;		// The length of the semi minor ellipse axis in meters.

		long	ProjCode;			// Projection id, Proj_UTM_11S
		long	Projection;		// EPSG identifier for underlying projection method

								/** GeoTIFF identifier for underlying projection method.  While some of
								these values have corresponding vlaues in EPSG (Projection field),
								others do not.  For example CT_TransverseMercator. */
		long	CTProjection;		//

									/** Number of projection parameters in ProjParm and ProjParmId. */
		int		nParms;

		/** Projection parameter value.  The identify of this parameter
		is established from the corresponding entry in ProjParmId.  The
		value will be measured in meters, or decimal degrees if it is a
		linear or angular measure. */
		double	ProjParm[MAX_GTIF_PROJPARMS];

		/** Projection parameter identifier.  For example ProjFalseEastingGeoKey.
		The value will be 0 for unused table entries. */
		int		ProjParmId[MAX_GTIF_PROJPARMS]; /* geokey identifier,
												eg. ProjFalseEastingGeoKey*/

												/** Special zone map system code (MapSys_UTM_South, MapSys_UTM_North,
												MapSys_State_Plane or KvUserDefined if none apply. */
		int		MapSys;

		int		Zone;    // UTM, or State Plane Zone number, zero if not known.

	};

	///////////////////////////////////////////////////////////////


	enum crsTYPE {
		crsGEOCENTRIC,
		crsGEOGRAPHIC_3D,
		crsGEOGRAPHIC_2D,
		crsPROJECTED,
		crsVERTICAL,
		crsCOMPOUND,
		crsENGINEERING,
		crsLOCAL
	};


	/* datum_type values */
	enum datumTYPE {
		datumUNKNOWN = 0,
		datum3PARAM = 1,
		datum7PARAM = 2,
		datumGRIDSHIFT = 3,
		datumWGS84 = 4   /* WGS84 (or anything considered equivelent) */
	};



	// OGR 字符串，实现安全模式的释放功能
	class XOGRString : public IOGRString
	{
	public:
		char *m_ogrStr;
	public:
		XOGRString() { m_ogrStr = NULL; };
		~XOGRString();

		const char *getStr() { return m_ogrStr; };

		IPL_OBJECT_IMP1(XOGRString, IOGRString, "default", "OGRString")
	};

	class XOGRValueArray : public IOGRValueArray
	{
	public:
		double *m_ogrValues;
	public:
		XOGRValueArray() { m_ogrValues = NULL; };
		~XOGRValueArray();
		const double *getValues() { return m_ogrValues; };

		IPL_OBJECT_IMP1(XOGRValueArray, IOGRValueArray, "default", "OGRValueArray")
	};
	
	///////////////////////////////////////////////////////
	class  XSpatialReference : public ISpatialReference
	{
	private:
		// for reducing error message after several times
		int			m_errorCount;

	private:
		srsTYPE	m_srsType;
		vcsTYPE m_heightType;

		char	m_vcsWkt[256];

	private:
		XEllipsoid	m_ellipsoid;

		OGRSpatialReferenceH m_hOgrSR;

		// proj4
		projPJ			m_pj;

		ref_ptr <IGeoid>	m_geoid;

		ref_ptr <ITangentPlane>	m_tangentPlane;

	private:
		// 	char *GetProj4Defn( GTIFDefn * psDefn );
		//
		// 	short GetEPSG_Code( const OGR_SRSNode *node );
		//
		// 	void SetGTIFFDefn_ProjectionMethod(const char *projectionMethodName );
		// 	void SetGTIFFDefn_GeogCS();
		// 	void SetGTIFDefnValues();
		///	//////////////////////////////////////////////////////////

	public:
		XSpatialReference(bool bRegister);
		virtual ~XSpatialReference();

		virtual bool ToGeocentric(const iplPOINT3D &from, iplPOINT3D *geoC);
		virtual bool FromGeocentric(const iplPOINT3D &geoC, iplPOINT3D *to);

		// 和WGS84椭球地心坐标的转换
		virtual bool ToGeocentric_WGS84(const iplPOINT3D &from, iplPOINT3D *geoC);
		virtual bool FromGeocentric_WGS84(const iplPOINT3D &geoC, iplPOINT3D *to);

		// 和自己椭球地理坐标的转换，按经纬度顺序
		virtual bool ToGeoGraphic(double X, double Y, double Z, double *longtitude, double *latitude, double *h);
		virtual bool FromGeoGraphic(double longtitude, double latitude, double h, double *x, double *y, double *z);

	public:

		// 空间参考的初始化集成OGRSpatialReference
		// 利用OGRSpatialReference出示proj4
		bool InitProj4();
		// proj4 库指针
		projPJ GetPJ() { return m_pj; };

	public:
		// 投影坐标与大地坐标的转换
		virtual bool Projected_To_Geodetic(double x, double y, double *latitude, double *longtitude);
		virtual bool Geodetic_To_Projected(double latitude, double longtitude, double *x, double *y);

		// 地心坐标与WGS84地心坐标的转换
		virtual bool Geocentric_To_WGS84(double *x, double *y, double *z);
		virtual bool Geocentric_From_WGS84(double *x, double *y, double *z);

		//datumTYPE	DatumType();

		IEllipsoid *GetEllipsoid()
		{
			return &m_ellipsoid;
		}

	public:

		// 	bool SetWith_OGRSpatialReference( OGRSpatialReference *ogrSR );
		// 	OGRSpatialReference *GetSpatialReference()	{ return this;	};

		virtual bool SetWithSpatialReference(ISpatialReference *pSpatialReference);

		virtual bool SetVcs(vcsTYPE vcsType, const char *vcsWkt = NULL);

	public:

		// 坐标系类型
		virtual srsTYPE SrsType() const { return m_srsType; };

		virtual vcsTYPE VcsType() const { return m_heightType; };

		virtual IGeoid *GetGeoid() { return m_geoid.get(); };

		virtual bool IsSameOgrCS(ISpatialReference *toCS) const
		{
			if (((XSpatialReference*)toCS)->m_srsType != m_srsType)
				return false;

			return  OSRIsSame(m_hOgrSR, ((XSpatialReference*)toCS)->m_hOgrSR);
		};

		virtual bool IsSameOgrGeogCS(ISpatialReference *toCS) const
		{
			return OSRIsSameGeogCS(m_hOgrSR, ((XSpatialReference*)toCS)->m_hOgrSR);
		};

		bool IsSameSRS(ISpatialReference *toCS) const;

		virtual bool IsProjected() const;

		virtual bool IsGeographic() const;

		virtual double GetSemiMajor() const
		{
			OGRErr err;

			return OSRGetSemiMajor(m_hOgrSR, &err);

		};

		virtual double GetSemiMinor() const
		{
			OGRErr err;

			return OSRGetSemiMinor(m_hOgrSR, &err);
		};

		//////////////////////////////////////////////////////////////////////////
		virtual bool importFromOGR(const OGRSpatialReference *pOgrSR);

		virtual const OGRSpatialReference *GetOGRSpatialReference()
		{
			return (OGRSpatialReference *)m_hOgrSR;
		}

		virtual bool importFromWkt(const char *pHcsWkt);

		virtual bool importFromVcsWkt(const char *vcsWkt);

		virtual bool importFromProj4(const char *pszProj4)
		{
			return !OSRImportFromProj4(m_hOgrSR, pszProj4);
		};

		virtual bool importFromEPSG(int epsgCode);

		virtual bool importFromESRI(const char *pszPrj)
		{
			char *pszPrj_t = (char *)pszPrj;
			//vc90调试时失败！ zj, 2010.11.26
			if (OGRERR_NONE == OSRImportFromESRI(m_hOgrSR, &pszPrj_t)) {
				InitProj4();
				return true;
			}

			return false;
		};

		virtual bool importFromPCI(const char *pszProj,
			const char *pszUnits = NULL, double *padfPrjParams = NULL)
		{
			return !OSRImportFromPCI(m_hOgrSR, pszProj, pszUnits, padfPrjParams);
		};

		virtual bool importFromUSGS(long iProjSys, long iZone, double *padfPrjParams, long iDatum)
		{
			return !OSRImportFromUSGS(m_hOgrSR, iProjSys, iZone, padfPrjParams, iDatum);
		};

		virtual bool importFromPanorama(long iProjSys, long iDatum, long iEllips, long iZone,
			double dfStdP1, double dfStdP2, double dfCenterLat, double dfCenterLong)
		{
			// 		return !OSRImportFromPanorama( iProjSys, iDatum, iEllips, iZone,
			//                    dfStdP1,  dfStdP2,  dfCenterLat,  dfCenterLong );
			return true;
		};

		//     virtual bool importFromWMSAUTO( const char *pszAutoDef )
		// 	{
		// 		return !OSRImportFromWMSAUTO( m_hOgrSR, pszAutoDef );
		// 	};

		virtual bool importFromXML(const char * pszXML)
		{
			return !OSRImportFromXML(m_hOgrSR, pszXML);
		};
		virtual bool importFromDict(const char *pszDict, const char *pszCode)
		{
			return !OSRImportFromDict(m_hOgrSR, pszDict, pszCode);
		};
		//     virtual bool importFromURN( const char *pszURN )
		// 	{
		// 		return !OSRImportFromURN( m_hOgrSR, pszURN );
		// 	};

		virtual bool SetProjection(const char *pcs)
		{
			return !OSRSetProjection(m_hOgrSR, pcs);
		}

		virtual bool SetGeogCS(const char * pszGeogName,
			const char * pszDatumName, const char * pszEllipsoidName,
			double dfSemiMajor, double dfInvFlattening,
			const char * pszPMName = NULL, double dfPMOffset = 0.0, const char * pszUnits = NULL,
			double dfConvertToRadians = 0.0)
		{
			return !OSRSetGeogCS(m_hOgrSR, pszGeogName,
				pszDatumName, pszEllipsoidName, dfSemiMajor, dfInvFlattening,
				pszPMName, dfPMOffset, pszUnits, dfConvertToRadians);
		}

		//////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////

		virtual bool exportToWkt(ref_ptr<IOGRString> &hcsWkt) const;

		virtual bool exportToVcsWkt(ref_ptr<IOGRString> &vcsWkt) const;

		virtual bool exportToPrettyWkt(ref_ptr<IOGRString> &ppszResult, bool bSimplify = false) const
		{
			XOGRString *result = new XOGRString;

			bool bRet = !OSRExportToPrettyWkt(m_hOgrSR, &result->m_ogrStr, bSimplify);

			ppszResult = ref_ptr<IOGRString>(result);

			return bRet;
		};

		virtual bool exportToProj4(ref_ptr<IOGRString> &ppszProj4) const
		{
			XOGRString *proj4 = new XOGRString;

			bool bRet = !OSRExportToProj4(m_hOgrSR, &proj4->m_ogrStr);

			ppszProj4 = ref_ptr<IOGRString>(proj4);

			return bRet;
		};

		virtual bool exportToPCI(ref_ptr<IOGRString> &ppszProj, ref_ptr<IOGRString> &ppszUnits, ref_ptr<IOGRValueArray> &ppadfPrjParams) const
		{
			XOGRString *proj4 = new XOGRString;
			XOGRString *units = new XOGRString;
			XOGRValueArray *params = new XOGRValueArray;

			bool bRet = !OSRExportToPCI(m_hOgrSR, &proj4->m_ogrStr, &units->m_ogrStr, &params->m_ogrValues);

			ppszProj = ref_ptr<IOGRString>(proj4);
			ppszUnits = ref_ptr<IOGRString>(units);
			ppadfPrjParams = ref_ptr<IOGRValueArray>(params);

			return bRet;
		};

		virtual bool exportToUSGS(long *piProjSys, long *piZone, ref_ptr<IOGRValueArray> &ppadfPrjParams, long *piDatum) const
		{
			XOGRValueArray *params = new XOGRValueArray;

			bool bRet = OSRExportToUSGS(m_hOgrSR, piProjSys, piZone, &params->m_ogrValues, piDatum);

			ppadfPrjParams = ref_ptr<IOGRValueArray>(params);

			return bRet;
		};


		virtual bool exportToXML(ref_ptr<IOGRString> &ppszRawXML, char *pszDialect = NULL) const
		{
			XOGRString *xml = new XOGRString;

			bool bRet = !OSRExportToXML(m_hOgrSR, &xml->m_ogrStr, pszDialect);

			ppszRawXML = ref_ptr<IOGRString>(xml);

			return bRet;
		}

		virtual bool exportToPanorama(long *piProjSys, long *piDatum, long *piEllips, long *piZone,
			double *pdfStdP1, double *pdfStdP2, double *pdfCenterLat, double *pdfCenterLong) const
		{
			// 		return !OSRExportToPanorama( piProjSys, piDatum, piEllips, piZone, pdfStdP1, pdfStdP2, pdfCenterLat, pdfCenterLong );
			return true;
		};

		//////////////////////////////////////////////////////////////////////////

		bool SetTOWGS84(double dfDX, double dfDY, double dfDZ,
			double dfEX, double dfEY, double dfEZ, double dfPPM)
		{
			return !OSRSetTOWGS84(m_hOgrSR, dfDX, dfDY, dfDZ, dfEX, dfEY, dfEZ, dfPPM);
		};

		bool GetTOWGS84(double *padfCoef, int nCoeff) const
		{
			return !OSRGetTOWGS84(m_hOgrSR, padfCoef, nCoeff);
		};

		IPL_OBJECT_IMP1(XSpatialReference, ISpatialReference, "default", "Spatial Reference")
	};

}
