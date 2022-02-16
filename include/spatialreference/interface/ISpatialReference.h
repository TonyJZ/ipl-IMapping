#pragma once

#include "spatialreference/interface/IGeoid.h"

namespace ipl
{
	/**
	* Holds a definition of a coordinate system in normalized form, from GTIFF
	*/
	enum	srsTYPE {
		srsNONE = 0,
		srsGEOGRAPHIC = 1,
		srsPROJECTED = 2,
		srsGEOCENTRIC = 3,
		srsTangentPLANE = 4,
		srsLOCAL_CS = 5

	};


	// Height System Type
	enum vcsTYPE
	{
		vcsNONE = 0,
		vcsELLIPSOID = 1,
		vcsGEOID = 2
	};

#define wktVcsNONE		_T("NONE")
#define wktVcsELLIPSOID _T("Ellipsoid")
#define wktVcsEGM96		_T("EGM96")


	class OGRSpatialReference;


	// OGR �ַ�����ʵ�ְ�ȫģʽ���ͷŹ���
	interface IOGRString : public IObject
	{
		virtual const char *getStr() = 0;

		IPL_INTERFACE_DEF(IObject, _T("OGRString"))
	};

	interface IOGRValueArray : public IObject
	{
		virtual const double *getValues() = 0;

		IPL_INTERFACE_DEF(IObject, _T("OGRValueArray"))
	};


	interface IEllipsoid;

	interface  ISpatialReference : public IObject
	{
	public:
		ISpatialReference() {};
		virtual ~ISpatialReference() {};

		// ����任����Ҫ�ӿڣ�������ϵ�Դ����ڲ�����
		// �����������(lon, lat��h)˳��

		// ���Լ�������������ת��
		virtual bool ToGeocentric(const iplPOINT3D &from, iplPOINT3D *geoC) = 0;
		virtual bool FromGeocentric(const iplPOINT3D &geoC, iplPOINT3D *to) = 0;

		// ��WGS84������������ת��
		virtual bool ToGeocentric_WGS84(const iplPOINT3D &from, iplPOINT3D *geoC) = 0;
		virtual bool FromGeocentric_WGS84(const iplPOINT3D &geoC, iplPOINT3D *to) = 0;

		// ���Լ�������������ת��������γ��˳��; ��γ���Զ�Ϊ��λ
		virtual bool ToGeoGraphic(double X, double Y, double Z, double *longtitude, double *latitude, double *h) = 0;
		virtual bool FromGeoGraphic(double longtitude, double latitude, double h, double *x, double *y, double *z) = 0;

		// ����������WGS84���������ת��
		// 	virtual bool Geocentric_To_WGS84( double *x, double *y, double *z ) = 0;
		// 	virtual bool Geocentric_From_WGS84( double *x, double *y, double *z ) = 0;
		// 
		// ���������������ص�ת��

		// ͶӰ�������������ת��
		// 	virtual bool Projected_To_Geodetic( double x, double y, double *latitude, double *longtitude ) = 0;
		// 	virtual bool Geodetic_To_Projected( double latitude, double longtitude, double *x, double *y ) = 0;
		// 
		// 	// �����������������ת��
		// 	virtual bool Geodetic_To_Geocentric (double latitude, double longitude,double height, double *X, double *Y, double *Z) = 0;
		// 	virtual bool Geocentric_To_Geodetic (double X, double Y, double Z, double *latitude, double *longitude,double *height) = 0;
		// 
		virtual	IEllipsoid *GetEllipsoid() = 0;

	public:	// Set Attributes
		virtual bool SetWithSpatialReference(ISpatialReference *pSpatialReference) = 0;

		virtual bool SetVcs(vcsTYPE vcsType, const char *vcsWkt = NULL) = 0;

	public:	// get attributes

		virtual bool IsSameSRS(ISpatialReference *toCS) const = 0;
		virtual bool IsSameOgrCS(ISpatialReference *toCS) const = 0;
		virtual bool IsSameOgrGeogCS(ISpatialReference *toCS) const = 0;

		virtual bool IsProjected() const = 0;
		virtual bool IsGeographic() const = 0;
		virtual double GetSemiMajor() const = 0;
		virtual double GetSemiMinor() const = 0;

		// ����ϵ����
		virtual srsTYPE SrsType() const = 0;

		// �߳�����
		virtual vcsTYPE VcsType() const = 0;

		// ���ˮ׼�������
		virtual IGeoid *GetGeoid() = 0;

		//////////////////////////////////////////////////////////////////////////
		// �����ĵ���
		virtual bool importFromOGR(const OGRSpatialReference *pOgrSR) = 0;

		virtual bool importFromWkt(const char *hcsWkt) = 0;
		virtual bool importFromVcsWkt(const char *vcsWkt) = 0;

		virtual bool importFromProj4(const char *pszProj4) = 0;
		virtual bool importFromEPSG(int epsgCode) = 0;
		virtual bool importFromESRI(const char *papszPrj) = 0;
		virtual bool importFromPCI(const char *pszProj,
			const char *pszUnits = NULL, double *padfPrjParams = NULL) = 0;
		virtual bool importFromUSGS(long iProjSys, long iZone, double *padfPrjParams, long iDatum) = 0;
		virtual bool importFromPanorama(long iProjSys, long iDatum, long iEllips, long iZone,
			double dfStdP1, double dfStdP2, double dfCenterLat, double dfCenterLong) = 0;
		//    virtual bool importFromWMSAUTO( const char *pszAutoDef )=0;
		virtual bool importFromXML(const char * pszXML) = 0;
		virtual bool importFromDict(const char *pszDict, const char *pszCode) = 0;
		//virtual bool importFromURN( const char *pszURN )=0;

		///////////////////////////////////////////////
		virtual bool SetProjection(const char *) = 0;
		virtual bool SetGeogCS(const char * pszGeogName,
			const char * pszDatumName, const char * pszEllipsoidName,
			double dfSemiMajor, double dfInvFlattening,
			const char * pszPMName = NULL, double dfPMOffset = 0.0, const char * pszUnits = NULL,
			double dfConvertToRadians = 0.0) = 0;

		//////////////////////////////////////////////////////////////////////////
		// �����ĵ���
		virtual const OGRSpatialReference *GetOGRSpatialReference() = 0;

		virtual bool exportToWkt(ref_ptr<IOGRString> &hcsWkt) const = 0;
		virtual bool exportToVcsWkt(ref_ptr<IOGRString> &vcsWkt) const = 0;
		virtual bool exportToPrettyWkt(ref_ptr<IOGRString> &ppszResult, bool bSimplify = false) const = 0;
		virtual bool exportToProj4(ref_ptr<IOGRString> &ppszProj4) const = 0;
		virtual bool exportToPCI(ref_ptr<IOGRString> &ppszProj, ref_ptr<IOGRString> &ppszUnits, ref_ptr<IOGRValueArray> &ppadfPrjParams) const = 0;
		virtual bool exportToUSGS(long *piProjSys, long *piZone, ref_ptr<IOGRValueArray> &ppadfPrjParams, long *piDatum) const = 0;
		virtual bool exportToXML(ref_ptr<IOGRString> &ppszRawXML, char *pszDialect = NULL) const = 0;
		virtual bool exportToPanorama(long *piProjSys, long *piDatum, long *piEllips, long *piZone,
			double *pdfStdP1, double *pdfStdP2, double *pdfCenterLat, double *pdfCenterLong) const = 0;

		//////////////////////////////////////////////////////////////////////////
		virtual bool SetTOWGS84(double, double, double, double = 0.0, double = 0.0, double = 0.0, double = 0.0) = 0;
		virtual bool GetTOWGS84(double *padfCoef, int nCoeff = 7) const = 0;


		IPL_INTERFACE_DEF(IObject, _T("spatialReference"))
	};

#define IPL_SpatialReference_DEF		"ipl.spatialReference.default"
}

