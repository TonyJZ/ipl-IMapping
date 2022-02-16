#include "core/interface/IPlatform.h"
#include "core/interface/IPlugin.h"
#include "core/interface/IRegisterService.h"


ipl::IPlatform *g_pPlatform = NULL;


// orsIMatrixService *g_matrixService = NULL;
// orsIMatrixService *getMatrixService()
// {
// 	if( NULL == g_matrixService )
// 		g_matrixService = IPL_PTR_CAST( orsIMatrixService, g_pPlatform->getService( ORS_SERVICE_MATRIX ) );
// 	
// 	return g_matrixService;
// }



//#include "XGeometryService.h"
#include "XGeometryTransform2D.h"
#include "XGeometryTransform3D.h"

using namespace ipl;

IObject* createAffineTransform2D(bool bForRegister)
{
	XAffineTransform2D* obj = new XAffineTransform2D;
	
	return IPL_PTR_CAST(IObject,obj);
}
IObject* createTranslationTransform2D(bool bForRegister)
{
	XTranslationTransform2D* obj = new XTranslationTransform2D;

	return IPL_PTR_CAST(IObject,obj);
}

IObject* createAffineTransform25D(bool bForRegister)
{
	XAffineTransform25D* obj = new XAffineTransform25D;
	
	return IPL_PTR_CAST(IObject,obj);
}


IObject* createAffineTransform3D(bool bForRegister)
{
	XAffineTransform3D* obj = new XAffineTransform3D;
	
	return IPL_PTR_CAST(IObject,obj);
}

IObject* createSimilarityTransform(bool bForRegister)
{
	XSimilarityTransform *obj = new XSimilarityTransform;
	
	return IPL_PTR_CAST(IObject,obj);
}


IObject* createBilinearTransform(bool bForRegister)
{
	XBilinearTransform *obj = new XBilinearTransform;
	
	return IPL_PTR_CAST(IObject,obj);
}

IObject* createPolynomialTransform2(bool bForRegister)
{
	XPolynomialTransform2 *obj = new XPolynomialTransform2;
	
	return IPL_PTR_CAST(IObject,obj);
}

IObject* createPolynomialTransformTPSpline(bool bForRegister)
{
	XPolynomialTransformTPSpline *obj = new XPolynomialTransformTPSpline;

	return IPL_PTR_CAST(IObject,obj);
}

IObject* createPolynomialTransform2_xDist(bool bForRegister)
{
	XPolynomialTransform2_xDist *obj = new XPolynomialTransform2_xDist;
	
	return IPL_PTR_CAST(IObject,obj);
}

IObject* createPolynomialTransform3(bool bForRegister)
{
	XPolynomialTransform3 *obj = new XPolynomialTransform3;
	
	return IPL_PTR_CAST(IObject,obj);
}

IObject* createDLT2D(bool bForRegister)
{
	XDLTTransform_2D *obj = new XDLTTransform_2D;
	
	return IPL_PTR_CAST(IObject,obj);
}


IObject* createDLT3D(bool bForRegister)
{
	XDLTTransform_3D *obj = new XDLTTransform_3D;
	
	return IPL_PTR_CAST(IObject,obj);
}

IObject* createSDLT3D(bool bForRegister)
{
	XSDLTTransform_3D *obj = new XSDLTTransform_3D;
	
	return IPL_PTR_CAST(IObject,obj);
}

// IObject* createRPCO1(bool bForRegister)
// {
// 	orsRPC_O1 *obj = new orsRPC_O1;
// 
// 	return IPL_PTR_CAST(IObject,obj);
// }
// 
// 
// IObject* createRPCO2(bool bForRegister)
// {
// 	orsRPC_O2 *obj = new orsRPC_O2;
// 
// 	return IPL_PTR_CAST(IObject, obj);
// }
// 
// 
// IObject* createRPCO3(bool bForRegister)
// {
// 	orsRPC_O3 *obj = new orsRPC_O3;
// 
// 	return IPL_PTR_CAST(IObject, obj);
// }


// #include "orsXPolygonClip.h"
// 
// IObject* createPolygonClip(bool bForRegister)
// {
// 	orsXPolygonClip *obj = new orsXPolygonClip(bForRegister);
// 	
// 	return IPL_PTR_CAST(IObject,obj);
// }

class XPlugin: public IPlugin
{
public:
	virtual const iplChar *getID()	
	{
		return "appropolis.ipl.Geometry"; 
	};

	virtual	const iplChar *getName()	
	{	
		return "Geometry Process"; 
	};
	
	virtual const iplChar *getProvider() 
	{	
		return "appropolis.calgary"; 
	};
	
	virtual const iplChar *getVersion()
	{
		return "0.1";
	}
	
	virtual  bool initialize(IPlatform*  platform)
	{
		g_pPlatform = platform;

		ref_ptr<IRegisterService> pRegister = platform->getRegisterService();
		
		//platform->registerService( IPL_SERVICE_GEOMETRY, new orsXGeometryService );
		
		pRegister->registerObject( createTranslationTransform2D );
		pRegister->registerObject( createAffineTransform2D );
		pRegister->registerObject( createAffineTransform3D );

		pRegister->registerObject( createAffineTransform25D );

		pRegister->registerObject( createSimilarityTransform  );
		
		pRegister->registerObject( createBilinearTransform );

		pRegister->registerObject( createPolynomialTransform2 );
		pRegister->registerObject( createPolynomialTransformTPSpline );
		pRegister->registerObject( createPolynomialTransform3 );

		pRegister->registerObject( createDLT2D );

		pRegister->registerObject( createDLT3D );

		pRegister->registerObject( createSDLT3D );

// 		pRegister->registerObject( createRPCO1 );
// 
// 		pRegister->registerObject(createRPCO2);
// 
// 		pRegister->registerObject(createRPCO3);

		pRegister->registerObject( createPolynomialTransform2_xDist );

//		pRegister->registerObject( createPolygonClip );

		return true;
	}
	
	virtual void finalize()
	{
		
	}
};

namespace ipl
{
	IPL_REGISTER_PLUGIN(XPlugin)
}


