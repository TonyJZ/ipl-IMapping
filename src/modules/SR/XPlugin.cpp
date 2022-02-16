#include "XSpatialReferenceService.h"
#include "XEllipsoid.h"
#include "XCoordinateTransform.h"
#include "XGeoid.h"
#include "XSpatialReference.h"

#include "core/interface/IPlatform.h"
#include "core/interface/IPlugin.h"
#include "core/interface/IRegisterService.h"

//ORS_GET_UTILITY_SERVICE_IMPL()
ipl::IPlatform *g_pPlatform = NULL;
// ipl::IPlatform *ipl::getPlatform()
// {
// 	return g_pPlatform;
// }

using namespace ipl;

IObject* createEllipsoid(bool bForRegister)
{
	XEllipsoid* obj = new XEllipsoid;

	return IPL_PTR_CAST(IObject, obj);
}
IObject* createTangentPlane(bool bForRegister)
{
	XTangentPlane* obj = new XTangentPlane;

	return IPL_PTR_CAST(IObject, obj);
}
IObject* createGeoid(bool bForRegister)
{
	XGeoid* obj = new XGeoid;

	return IPL_PTR_CAST(IObject, obj);
}

IObject* createSpatialReference(bool bForRegister)
{
	XSpatialReference* obj = new XSpatialReference(bForRegister);

	return IPL_PTR_CAST(IObject, obj);
}

class XPlugin: public IPlugin
{
public:
	virtual const iplChar *getID()	
	{
		return "appropolis.ipl.SRS"; 
	};

	virtual	const iplChar *getName()	
	{	
		return "Spatial Reference Service"; 
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

		platform->registerService( IPL_SERVICE_SRS, new XSpatialReferenceService );

		ref_ptr<IRegisterService> pRegister = platform->getRegisterService();

		//platform->registerService( IPL_SERVICE_GEOMETRY, new orsXGeometryService );

		pRegister->registerObject(createEllipsoid);
		pRegister->registerObject(createTangentPlane);
		pRegister->registerObject(createGeoid);
		pRegister->registerObject(createSpatialReference);


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


