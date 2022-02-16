// utest_SR.cpp : Defines the entry point for the console application.
//

//#include "spatialreference/SpatialReference.h"
#include "core/iplPlatformUtil.h"
#include "core/interface/IPlatform.h"
#include "spatialreference/interface/ICoordinateTransform.h"
#include "spatialreference/interface/ISpatialReference.h"
#include "spatialreference/interface/ISpatialReferenceService.h"

#include "geometry/interface/IGeometryTransform.h"


ipl::IPlatform *g_pPlatform = NULL;

ipl::IPlatform *ipl::getPlatform()
{
	return g_pPlatform;
}

namespace ipl
{
	IPL_GET_SRS_SERVICE_IMPL();
}

using namespace ipl;
int main(int argc, char * argv[])
{
	iplString errorinfo;
	g_pPlatform = iplInitialize(errorinfo);

	
	char wktStr[2048];

	sprintf(wktStr, "PROJCS[\"WGS 84 / UTM zone %d%s\", \
																   GEOGCS[\"WGS 84\", \
																   DATUM[\"WGS_1984\", \
																   SPHEROID[\"WGS 84\", 6378137, 298.257223563, \
																   AUTHORITY[\"EPSG\", \"7030\"]], \
																   AUTHORITY[\"EPSG\", \"6326\"]], \
																   PRIMEM[\"Greenwich\", 0, \
																   AUTHORITY[\"EPSG\", \"8901\"]], \
																   UNIT[\"degree\", 0.0174532925199433, \
																   AUTHORITY[\"EPSG\", \"9122\"]], \
																   AUTHORITY[\"EPSG\", \"4326\"]], \
																   PROJECTION[\"Transverse_Mercator\"], \
																   PARAMETER[\"latitude_of_origin\", 0], \
																   PARAMETER[\"central_meridian\", %d], \
																   PARAMETER[\"scale_factor\", 0.9996], \
																   PARAMETER[\"false_easting\", 500000], \
																   PARAMETER[\"false_northing\", 0], \
																   UNIT[\"metre\", 1, \
																   AUTHORITY[\"EPSG\", \"9001\"]]]",
		// AUTHORITY[\"EPSG\", \"32650\"]] ",		// 这是标准的UTM 50N代码，不能加
		0, 0, 0);

	//服务的调用方式 1 需要定义IPL_GET_SRS_SERVICE_IMPL
	ISpatialReferenceService *g_pSRService = getSRService(); 


	// 调用方式2 
	g_pSRService = IPL_PTR_CAST(ISpatialReferenceService, getPlatform()->getService(IPL_SERVICE_SRS)); 
	printf("ISpatialReferenceService ok\n");

	//可执行对象的调用方式1 通过服务创建
	ref_ptr<ISpatialReference> geoSR = ref_ptr<ISpatialReference>(g_pSRService->CreateSpatialReference(wktStr));

	printf("ISpatialReference ok\n");

	ref_ptr<IGeometryTransform2D> transform;

	transform = ref_ptr<IGeometryTransform2D>(IPL_CREATE_OBJECT(IGeometryTransform2D, IPL_GEOMETRY_TRANSFORM2D_DLT));
	printf("IGeometryTransform2D ok\n");
	//可执行对象调用方式2  直接创建
	ref_ptr<ITangentPlane> tPlane;
	tPlane = ref_ptr<ITangentPlane>(IPL_CREATE_OBJECT(ITangentPlane, IPL_TangentPlane_DEF));
	printf("ITangentPlane ok\n");

	printf("%s\n", tPlane->getClassDesc().c_str()); 
	printf("%s\n", tPlane->getClassID().c_str());
	
	tPlane->queryInterface("ITangentPlane");

	tPlane->InitializeEllipsoid(0, 0);

	//在平台释放前，释放所有平台创建的可执行对象
	geoSR.reset();
	transform.reset();
	tPlane.reset();

	iplUninitialize();

//	tPlane->getClassID();
    return 0;
}

