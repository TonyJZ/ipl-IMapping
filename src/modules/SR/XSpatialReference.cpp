//std
#include <assert.h>
#include <string>
#include <iostream>

//boost
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string_regex.hpp>

//gdal
#include <GDAL/cpl_string.h>


#include "XSpatialReference.h"

///////////////////////////////////////////////////////////////////////

char *csTypeNAMEs[] = {
	"geographic 2D",
	"geographic 3D",
	"projected 2D",
	"compound",
	"vertical",
	"local"
};

extern ipl::IPlatform* ipl::getPlatform();

using namespace ipl;
//////////////////////////////////////////////////////////////////////////


// OGR 字符串，实现安全模式的释放功能

XOGRString::~XOGRString()
{
	if (NULL != m_ogrStr)
		CPLFree(m_ogrStr);
};


XOGRValueArray::~XOGRValueArray()
{
	if (NULL != m_ogrValues)
		CPLFree(m_ogrValues);
};


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

XSpatialReference::XSpatialReference(bool bRegister)
	:ISpatialReference()
{
	m_pj = NULL;
	m_errorCount = 0;

	m_srsType = srsNONE;
	m_heightType = vcsELLIPSOID;

	m_hOgrSR = NULL;

	if (!bRegister) {
		m_hOgrSR = OSRNewSpatialReference(NULL); // new OGRSpatialReference;	
		strcpy(m_vcsWkt, wktVcsELLIPSOID);
	}
}


XSpatialReference::~XSpatialReference()
{
	if (m_pj != NULL)
		pj_free(m_pj);

	if (m_hOgrSR)
		OSRDereference(m_hOgrSR);

}


bool XSpatialReference::ToGeocentric(const iplPOINT3D &from, iplPOINT3D *geoC)
{
	switch (m_srsType) {
	case srsGEOCENTRIC:
		*geoC = from;
		break;
	case srsGEOGRAPHIC:

		m_ellipsoid.Geodetic_To_Geocentric(DEG_TO_RAD*from.Y, DEG_TO_RAD*from.X, from.Z, &geoC->X, &geoC->Y, &geoC->Z);

		break;
	case srsPROJECTED:
	{
		double lat, lon;

		Projected_To_Geodetic(from.X, from.Y, &lat, &lon);
		m_ellipsoid.Geodetic_To_Geocentric(lat, lon, from.Z, &geoC->X, &geoC->Y, &geoC->Z);
	}

	break;

	case srsTangentPLANE:
		m_tangentPlane->TangentPlane2Geocentric(from.X, from.Y, from.Z, &geoC->X, &geoC->Y, &geoC->Z);
		break;
	}

	return true;
}

bool XSpatialReference::FromGeocentric(const iplPOINT3D &geoC, iplPOINT3D *to)
{
	switch (m_srsType) {
	case srsGEOCENTRIC:
		*to = geoC;
		break;
	case srsGEOGRAPHIC:
	{
		double lat, lon, hei;

		m_ellipsoid.Geocentric_To_Geodetic(geoC.X, geoC.Y, geoC.Z, &lat, &lon, &hei);

		to->lon = RAD_TO_DEG*lon;
		to->lat = RAD_TO_DEG*lat;
		to->h = hei;
	}

	break;
	case srsPROJECTED:
	{
		double lat, lon, hei;

		m_ellipsoid.Geocentric_To_Geodetic(geoC.X, geoC.Y, geoC.Z, &lat, &lon, &hei);

		// 弧度
		Geodetic_To_Projected(lat, lon, &to->X, &to->Y);

		to->h = hei;
	}

	break;
	case srsTangentPLANE:
		m_tangentPlane->Geocentric2TangentPlane(geoC.X, geoC.Y, geoC.Z, &to->X, &to->Y, &to->Z);
		break;
	}

	return true;
}


// 和自己椭球地理坐标的转换，按经纬度顺序
bool XSpatialReference::ToGeoGraphic(double X, double Y, double Z, double *lon, double *lat, double *h)
{
	switch (m_srsType) {
	case srsGEOCENTRIC:
		m_ellipsoid.Geocentric_To_Geodetic(X, Y, Z, lat, lon, h);

		*lat = RAD_TO_DEG* *lat;
		*lon = RAD_TO_DEG* *lon;

		break;
	case srsGEOGRAPHIC:
		*lon = X;
		*lat = Y;
		*h = Z;

		break;
	case srsPROJECTED:
	{
		Projected_To_Geodetic(X, Y, lat, lon);

		*lon = RAD_TO_DEG* *lon;
		*lat = RAD_TO_DEG* *lat;
		*h = Z;
	}

	break;

	case srsTangentPLANE:
		m_tangentPlane->TangentPlane2Geographic(X, Y, Z, lat, lon, h);

		*lon = RAD_TO_DEG* *lon;
		*lat = RAD_TO_DEG* *lat;
		break;
	}

	return true;
}


bool XSpatialReference::FromGeoGraphic(double lon, double lat, double h, double *X, double *Y, double *Z)
{
	switch (m_srsType) {
	case srsGEOCENTRIC:
		m_ellipsoid.Geodetic_To_Geocentric(DEG_TO_RAD*lat, DEG_TO_RAD*lon, h, X, Y, Z);
		break;
	case srsGEOGRAPHIC:
	{
		*X = lon;
		*Y = lat;
		*Z = h;
	}

	break;
	case srsPROJECTED:
	{
		//double lat, lon, hei;

		Geodetic_To_Projected(DEG_TO_RAD*lat, DEG_TO_RAD*lon, X, Y);

		*Z = h;
	}

	break;
	case srsTangentPLANE:
		m_tangentPlane->Geographic2TangentPlane(DEG_TO_RAD*lat, DEG_TO_RAD*lon, h, X, Y, Z);
		break;
	}

	return true;
}

// 和WGS84椭球地心坐标的转换
bool XSpatialReference::ToGeocentric_WGS84(const iplPOINT3D &from, iplPOINT3D *geoC)
{
	ToGeocentric(from, geoC);

	Geocentric_To_WGS84(&geoC->X, &geoC->Y, &geoC->Z);

	return true;
}

bool XSpatialReference::FromGeocentric_WGS84(const iplPOINT3D &geoC, iplPOINT3D *to)
{
	*to = geoC;

	Geocentric_From_WGS84(&to->X, &to->Y, &to->Z);

	FromGeocentric(*to, to);

	return true;
}

// 
// datumTYPE XSpatialReference::DatumType()
// {
// 	if( m_pj == NULL )	{
// 		getPlatform()->logPrint( IPL_LOG_ERROR, "Projection are not initialized");
// 		return datumUNKNOWN;
// 	}
// 
// 	return (datumTYPE)  m_pj->datum_type;
// }


//
// 利用OGRSpatialReference_impl信息初始化Proj4
//
bool XSpatialReference::InitProj4()
{
	// copy and modified from OGRProj4CT::Initialize

	if (srsNONE == m_srsType) {
		if (OSRIsGeographic(m_hOgrSR))
			m_srsType = srsGEOGRAPHIC;
		else if (OSRIsProjected(m_hOgrSR))
			m_srsType = srsPROJECTED;
		else if (OSRIsLocal(m_hOgrSR))
			m_srsType = srsLOCAL_CS;
		else {
			assert(false);
		}
	}

	/* -------------------------------------------------------------------- */
	/*      Establish PROJ.4 handle for source if projection.               */
	/* -------------------------------------------------------------------- */
	char *pszProj4Defn, **papszArgs;

	if (OSRExportToProj4(m_hOgrSR, &pszProj4Defn) != OGRERR_NONE)
		return FALSE;

	papszArgs = CSLTokenizeStringComplex(pszProj4Defn, " +", TRUE, FALSE);

	if (m_pj)
		pj_free(m_pj);

	m_pj = pj_init(CSLCount(papszArgs), papszArgs);

	if (m_pj == NULL)
	{
		if (pj_errno != 0) {
			char msg[256];
			sprintf(msg, "Failed to initialize PROJ.4 with `%s'.\n%s", pszProj4Defn, pj_strerrno(pj_errno));
			getPlatform()->logPrint(IPL_LOG_WARNING, msg);  // modify by zhangguo   IPL_LOG_ERROR  -->IPL_LOG_WARNING 20090926
			//std::cout << msg << std::endl;
		}
		else
			getPlatform()->logPrint(IPL_LOG_WARNING, "Failed to initialize PROJ.4");
			//std::cout << "Failed to initialize PROJ.4" << std::endl;
		//// modify by zhangguo   IPL_LOG_ERROR  -->IPL_LOG_WARNING 20090926
	}

	m_ellipsoid.SetParameter(GetSemiMajor(), GetSemiMinor());

	CSLDestroy(papszArgs);
	CPLFree(pszProj4Defn);

	return true;
}


bool XSpatialReference::Projected_To_Geodetic(double x, double y, double *latitude, double *longtitude)
{
	if (m_pj == NULL) {
		if (m_errorCount < 2) {
			getPlatform()->logPrint(IPL_LOG_ERROR, "Projected_To_Geodetic: Projection are not initialized");
			//std::cout << "Projected_To_Geodetic: Projection are not initialized" << std::endl;
			m_errorCount++;
		}
		return false;
	}

	projXY projected_loc;
	projLP geodetic_loc;

	projected_loc.u = x;
	projected_loc.v = y;

	geodetic_loc = pj_inv(projected_loc, m_pj);

	//	变量pj_errno 不可靠
	// 	if( pj_errno != 0 )  {
	// 		if( m_errorCount < 2 ) {
	// 			char msg[256];
	// 			sprintf(msg, "Failed Converting to geodetic by PROJ.45 \n%s", pj_strerrno( pj_errno ) );
	// 			getPlatform()->logPrint( IPL_LOG_ERROR,  msg );
	// 			m_errorCount++;
	// 		}
	// 
	// 		return false;
	// 	}

	// proj4.5 use (longtitude, latitue)
	*longtitude = geodetic_loc.u;
	*latitude = geodetic_loc.v;

	return true;
};



bool XSpatialReference::Geodetic_To_Projected(double latitude, double longtitude, double *x, double *y)
{
	if (m_pj == NULL) {
		if (m_errorCount < 2) {
			getPlatform()->logPrint(IPL_LOG_ERROR, "Projection are not initialized");
			//std::cout << "Projection are not initialized" << std::endl;
			m_errorCount++;
		}
		return false;
	}

	projXY projected_loc;
	projLP geodetic_loc;

	// proj4.5 use (longtitude, latitue)
	geodetic_loc.u = longtitude;
	geodetic_loc.v = latitude;

	projected_loc = pj_fwd(geodetic_loc, m_pj);

	//	变量pj_errno 不可靠
	// 	if( pj_errno != 0 )  {
	// 		if( m_errorCount < 2 ) {
	// 			char msg[256];
	// 			sprintf(msg, "Failed Converting to projected by PROJ.45 \n%s", pj_strerrno( pj_errno ) );
	// 			getPlatform()->logPrint( IPL_LOG_WARNING, msg );
	// 			m_errorCount++;
	// 		}
	// 		return false;
	// 	}

	*x = projected_loc.u;
	*y = projected_loc.v;

	return true;
};



bool XSpatialReference::importFromOGR(const OGRSpatialReference *pOgrSR)
{
	OSRDereference(m_hOgrSR);

	m_hOgrSR = NULL;

	m_hOgrSR = OSRClone((OGRSpatialReference *)pOgrSR);

	return InitProj4();
}


bool XSpatialReference::SetWithSpatialReference(ISpatialReference *pSR)
{
	ref_ptr<IOGRString> vcs;

	if (pSR->exportToVcsWkt(vcs))
		importFromVcsWkt(vcs->getStr());

	return importFromOGR(pSR->GetOGRSpatialReference());
}


bool XSpatialReference::Geocentric_To_WGS84(double *x, double *y, double *z)
{
	if (m_pj == NULL) {
		getPlatform()->logPrint(IPL_LOG_ERROR, "Coordinate System is not initilized");
		//std::cout << "Coordinate System is not initilized" << std::endl;
		return false;
	}

	// proj 4.8中这个功能被取消了，比较遗憾
	//if( m_pj )
	// 		pj_geocentric_to_wgs84( m_pj, 1, 1, x, y, z );

	return true;
}


bool XSpatialReference::Geocentric_From_WGS84(double *x, double *y, double *z)
{
	if (m_pj == NULL) {
		getPlatform()->logPrint(IPL_LOG_ERROR, "Coordinate System is not initilized");
		//std::cout << "Coordinate System is not initilized" << std::endl;
		return false;
	}

	// proj 4.8中这个功能被取消了，比较遗憾
	//if( m_pj )
	//	pj_geocentric_from_wgs84( m_pj, 1, 1, x, y, z );

	return true;
}



bool XSpatialReference::importFromWkt(const char *pHcsWkt)
{
	getPlatform()->logPrint( IPL_LOG_DEBUG, "importFromWkt %s", pHcsWkt );
	if (NULL == pHcsWkt) {
		m_srsType = srsNONE;
		return false;
	}
	// 	char pHcsWktTemp[2048];
	// 	strcpy(pHcsWktTemp, pHcsWkt);

	std::string pHcsWktTemp = pHcsWkt;

	char *hcsWkt = (char *)pHcsWktTemp.data();
	//char *hcsWkt;

	char *correctedWkt = NULL;
	if (std::string::npos != pHcsWktTemp.find(_T("TangentPLANE")))
	{
		m_srsType = srsTangentPLANE;

		m_tangentPlane = ref_ptr< XTangentPlane >(new XTangentPlane);

		//std::string trimTemp = boost::algorithm::trim_left_copy_if(pHcsWktTemp, boost::algorithm::is_any_of("GEOGCS["));
		boost::iterator_range<std::string::iterator> result = boost::algorithm::find_first(pHcsWktTemp, _T("GEOGCS["));

		size_t pos = result.begin() - pHcsWktTemp.begin();
		std::string trimTemp(result.begin(), pHcsWktTemp.end());

		//hcsWkt = (char *)orsString::findSubStr_i((char *)pHcsWktTemp, _T("GEOGCS["));

		//trimTemp.copy(hcsWkt, 2048);
		hcsWkt = (char*)trimTemp.data();

		int len = iplStrLen(hcsWkt);

		if (']' == hcsWkt[len - 1])
			hcsWkt[len - 1] = 0;

	}
	else if (std::string::npos != pHcsWktTemp.find(_T("PROJCS")))
	{
		//const char *projWkt = orsString::findSubStr_i((char *)pHcsWktTemp, _T("PROJECTION["));
		boost::iterator_range<std::string::iterator> result = boost::algorithm::find_first(pHcsWktTemp, _T("PROJECTION["));
		size_t pos = result.begin() - pHcsWktTemp.begin();

		//std::string projWkt = pHcsWktTemp.substr(pos);
		//assert(result.empty());
		std::string projWkt(result.begin(), pHcsWktTemp.end());

		//printf("proj Wkt: %s\n", projWkt);

		// 检查wkt中是的单位是否正确
		//const char *unitStr = orsString::findSubStr_i(projWkt, _T("UNIT"));
		result = boost::algorithm::find_first(projWkt, _T("UNIT"));

		if (!result.empty()) { //find "UNIT"
			std::string unitStr(result.begin(), projWkt.end());
			//std::string::iterator iterUnit = result.begin();
			int unitLen = result.begin() - projWkt.begin();

			result = boost::algorithm::find_first(unitStr, _T("metre"));

			if (result.empty()) { //not find "meter"
				int len = unitLen + pos;

				correctedWkt = new char[len + 1 + strlen("UNIT[\"metre\", 1, AUTHORITY[\"EPSG\",\"9001\"]]]")];
				memcpy(correctedWkt, pHcsWktTemp.data(), len);
				correctedWkt[len] = 0;
				strcpy(correctedWkt + len, "UNIT[\"metre\", 1, AUTHORITY[\"EPSG\",\"9001\"]]]");

				//memcpy(hcsWkt, correctedWkt, iplStrLen(correctedWkt));
				hcsWkt = correctedWkt;

				//printf("Corrected Wkt: %s\n", hcsWkt);
			}
		}
		else {
			// 默认单位为米
			printf("Unit unspecified in hcsWkt\n");

			//int len = strlen(pHcsWktTemp);
			int len = pHcsWktTemp.size();

			correctedWkt = new char[len + strlen(",UNIT[\"metre\", 1, AUTHORITY[\"EPSG\",\"9001\"]]]")];

			memcpy(correctedWkt, pHcsWktTemp.data(), len - 1);
			strcpy(correctedWkt + len - 1, ",UNIT[\"metre\", 1, AUTHORITY[\"EPSG\",\"9001\"]]]");

			//memcpy(hcsWkt, correctedWkt, iplStrLen(correctedWkt));
			hcsWkt = correctedWkt;

			//printf("Corrected Wkt: %s\n", hcsWkt);
		}
	}
	else if (std::string::npos != pHcsWktTemp.find(_T("GEOGCS")))
	{

	}
	else if (0 == boost::algorithm::equals(pHcsWktTemp, "wgs84"))
	{
		hcsWkt = "GEOGCS[\"WGS 1984\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.01745329251994328,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]]";
		//strcpy(hcsWkt, "GEOGCS[\"WGS 1984\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.01745329251994328,AUTHORITY[\"EPSG\",\"9122\"]],AUTHORITY[\"EPSG\",\"4326\"]]");
	}

	if (OGRERR_NONE == OSRImportFromWkt(m_hOgrSR, &hcsWkt)) {
		if (correctedWkt)
			delete correctedWkt;

		if (InitProj4()) {
			if (srsTangentPLANE == m_srsType) {
				m_tangentPlane->InitializeEllipsoid(GetSemiMajor(), GetSemiMinor());

				boost::iterator_range<std::string::iterator> result = boost::algorithm::find_first(pHcsWktTemp, _T("AnchorPOINT["));

				//char *pAnchorStr = (char *)orsString::findSubStr_i((char *)pHcsWktTemp, "AnchorPOINT[") + strlen("AnchorPOINT[");
				char *pAnchorStr = (char*)(pHcsWktTemp.data() + (result.end() - pHcsWktTemp.begin()));

				double lon, lat, h;

				// AnchorPOINT[114.3222933,22.71760494,0]
				sscanf(pAnchorStr, "%lf,%lf,%lf]", &lon, &lat, &h);

				m_tangentPlane->SetAnchorPoint(DEG_TO_RAD*lat, DEG_TO_RAD*lon, h);
			}

			return true;
		}
		else {
			std::cout << "InitProj4 failed: %s" << std::endl;
		}
	}
	else {
		std::cout << "importFromWkt failed: %s" << std::endl;
	}

	return false;
}

bool XSpatialReference::SetVcs(vcsTYPE vcsType, const char *vcsWkt)
{
	m_heightType = vcsType;

	switch (vcsType) {
	case vcsNONE:
		strcpy(m_vcsWkt, wktVcsNONE);
		break;
	case vcsELLIPSOID:
		strcpy(m_vcsWkt, wktVcsELLIPSOID);
		break;
		// 	case vcsEGM96:
		// 		strcpy( m_vcsWkt, wktVcsEGM96 );
		// 		break;
	case vcsGEOID:
		assert(vcsWkt != NULL);
		if (vcsWkt == NULL)
			return false;

		strcpy(m_vcsWkt, vcsWkt);
		break;
	default:
		return false;
	}

	return true;
}


bool XSpatialReference::importFromVcsWkt(const char *vcsWkt)
{
	if (NULL == vcsWkt) {
		m_heightType = vcsNONE;
		return false;
	}

	// 跳过空格
	while (*vcsWkt == ' ')	vcsWkt++;

	strcpy(m_vcsWkt, vcsWkt);

	m_heightType = vcsNONE;

	std::string str = m_vcsWkt;
	if (std::string::npos != str.find(wktVcsELLIPSOID))
		//	  (	orsString::findSubStr_i( m_vcsWkt, wktVcsELLIPSOID) )	
	{
		m_heightType = vcsELLIPSOID;
	}
	else if (std::string::npos != str.find(wktVcsEGM96))
		//	( orsString::findSubStr_i( m_vcsWkt, wktVcsEGM96 )	)	
	{
		m_heightType = vcsGEOID;
	}

	// 需要为不同的Geoid完善程序，目前只支持EGM96

	return false;
};


bool XSpatialReference::importFromEPSG(int epsgCode)
{
	if (OGRERR_NONE == OSRImportFromEPSG(m_hOgrSR, epsgCode)) {
		InitProj4();
		return true;
	}
	else
		getPlatform()->logPrint(IPL_LOG_ERROR, "importFromEPSG failed: %d", epsgCode);
		//std::cout << "importFromEPSG failed: " << epsgCode << std::endl;

	return false;
}

bool XSpatialReference::exportToWkt(ref_ptr<IOGRString> &hcsWkt) const
{
	bool bRet = false;

	switch (SrsType()) {
	case srsTangentPLANE:
	{
		ref_ptr<XOGRString> wkt0 = ref_ptr<XOGRString>(new XOGRString);
		if (0 == OSRExportToWkt(m_hOgrSR, &wkt0->m_ogrStr)) {
			double lat0, lon0, h0;

			m_tangentPlane->GetAnchorPoint(&lat0, &lon0, &h0);

			XOGRString *wkt = new XOGRString;

			wkt->m_ogrStr = (char *)OGRMalloc(strlen(wkt0->getStr()) + 80);

			lat0 = RAD_TO_DEG*lat0;
			lon0 = RAD_TO_DEG*lon0;
			sprintf((char *)wkt->getStr(), "TangentPLANE[AnchorPOINT[%.8lf,%.8lf%,%.3lf%],%s]", lon0, lat0, h0, wkt0->getStr());

			hcsWkt = ref_ptr<IOGRString>((IOGRString *)wkt);

			bRet = true;
		}
	}

	break;

	case srsGEOGRAPHIC:
	case srsPROJECTED:
	case srsLOCAL_CS:
	{
		XOGRString *wkt = new XOGRString;

		bRet = !OSRExportToWkt(m_hOgrSR, &wkt->m_ogrStr);
		hcsWkt = ref_ptr<IOGRString>((IOGRString *)wkt);

		bRet = true;
	}
	break;

	default:
		assert(false);
	}

	return bRet;
};

bool XSpatialReference::exportToVcsWkt(ref_ptr<IOGRString> &vcsWkt) const
{
	XOGRString *wkt = new XOGRString;

	wkt->m_ogrStr = CPLStrdup(m_vcsWkt);

	vcsWkt = ref_ptr<IOGRString>((IOGRString *)wkt);

	return true;
};

//////////////////////////////////////////////////////////////////////////

bool XSpatialReference::IsSameSRS(ISpatialReference *toCS) const
{
	if (SrsType() != toCS->SrsType())
		return false;

	switch (m_srsType) {
	case srsTangentPLANE:
	{
		ref_ptr<IOGRString> hcsWkt0, hcsWkt1;

		exportToWkt(hcsWkt0);
		toCS->exportToWkt(hcsWkt1);

		if (0 == stricmp(hcsWkt0->getStr(), hcsWkt1->getStr()))
			return true;

		return false;
	}

	break;
	case srsGEOCENTRIC:
		assert(false);

		break;
	case srsPROJECTED:
	case srsGEOGRAPHIC:
		// 待完善，高程系统是否一致

		// if( GetGeoid() != toCS  )

		if (!IsSameOgrCS(toCS))
			return false;
	}

	return true;
}

bool XSpatialReference::IsProjected() const
{
	return (srsPROJECTED == m_srsType);

	//return OSRIsProjected( m_hOgrSR );
};

bool XSpatialReference::IsGeographic() const
{
	return (srsGEOGRAPHIC == m_srsType);

	//return OSRIsGeographic( m_hOgrSR );
};
