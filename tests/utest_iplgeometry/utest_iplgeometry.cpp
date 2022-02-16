// utest_iplgeometry.cpp : Defines the entry point for the console application.
//

#include "core/iplPlatformUtil.h"
#include "core/interface/IPlatform.h"
#include "core/interface/IRegisterService.h"
#include "geometry/interface/IGeometryTransform.h"

//#include <boost/filesystem.hpp>

extern bool dataSimulate(char * filename, std::vector<iplPOINT3D> &src, std::vector<iplPOINT3D> &dst);

ipl::IPlatform *g_pPlatform = NULL;

ipl::IPlatform *ipl::getPlatform()
{
	return g_pPlatform;
}

using namespace ipl;


static std::vector<iplPOINT3D> srcPts, dstPts;

bool getDLTTransform()
{
	ref_ptr<IGeometryTransform2D> transform;

	transform = ref_ptr<IGeometryTransform2D>(IPL_CREATE_OBJECT(IGeometryTransform2D, IPL_GEOMETRY_TRANSFORM2D_DLT));

	printf("DLT transform\n");

	iplPOINT2D *ptsSrc;
	iplPOINT2D *ptsDst;

	size_t npts = srcPts.size();
	ptsSrc = new iplPOINT2D[npts];
	ptsDst = new iplPOINT2D[npts];


	double mx, my;

	int i;

	
	for (i = 0; i < npts; i++)
	{
		ptsSrc[i].x = srcPts[i].X;
		ptsSrc[i].y = srcPts[i].Y;

		ptsDst[i].x = dstPts[i].X;
		ptsDst[i].y = dstPts[i].Y;
	}
	


	transform->Initialize(ptsSrc, npts, ptsDst);
	double a[9];
	transform->GetParameter(a, NULL);

	//		transform->GetResidual(ptsDst, n, ptsSrc , ptsVxy );

	transform->GetMeanError(&mx, &my);

	printf("mx=%.3f, my=%.3f\n", mx, my);

	delete ptsSrc;
	delete ptsDst;

	return true;
}

bool getAffineTransform()
{
	ref_ptr<IGeometryTransform2D> transform;

	transform = ref_ptr<IGeometryTransform2D>(IPL_CREATE_OBJECT(IGeometryTransform2D, IPL_GEOMETRY_TRANSFORM2D_AFFINE));

	printf("Affine transform\n");

	iplPOINT2D *ptsSrc;
	iplPOINT2D *ptsDst;

	size_t npts = srcPts.size();
	ptsSrc = new iplPOINT2D[npts];
	ptsDst = new iplPOINT2D[npts];


	double mx, my;

	int i;

	
	for (i = 0; i < npts; i++)
	{
		ptsSrc[i].x = srcPts[i].X;
		ptsSrc[i].y = srcPts[i].Y;

		ptsDst[i].x = dstPts[i].X;
		ptsDst[i].y = dstPts[i].Y;
	}
	


	transform->Initialize(ptsSrc, npts, ptsDst);

	double a[9], b[9];
	transform->GetParameter(a, b);

	//		transform->GetResidual(ptsDst, n, ptsSrc , ptsVxy );

	transform->GetMeanError(&mx, &my);

	printf("mx=%.3f, my=%.3f\n", mx, my);

	delete ptsSrc;
	delete ptsDst;

	return true;
}

int main(int argc, char * argv[])
{
	static ref_ptr <iplPOINT2D> ptsSrc2D, ptsDst2D;
	static ref_ptr <iplPOINT3D> ptsSrc3D, ptsDst3D;

	dataSimulate(argv[1], srcPts, dstPts);


	iplString errorinfo;
	g_pPlatform = iplInitialize(errorinfo);

	getAffineTransform();
	getDLTTransform();

	iplUninitialize();
    return 0;
}

