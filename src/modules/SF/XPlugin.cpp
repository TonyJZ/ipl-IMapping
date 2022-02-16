//////////////////////////////////////////////////////////////////////////
#include "XSFService.h"
#include "spatialreference/interface/ISpatialReferenceService.h"
#include "core/interface/IPlatform.h"
#include "core/interface/IPlugin.h"
#include "core/interface/IRegisterService.h"

namespace ipl
{
	IPL_GET_SRS_SERVICE_IMPL();
}

ipl::ISFService *g_osfService = NULL;
//////////////////////////////////////////////////////////////////////////

#include "XSFOGRVectorSource.h"
using namespace ipl;

IObject* createVectorDataSource(bool bForRegister )
{
	XSFOGRVectorSource* obj = new XSFOGRVectorSource;
	
	return IPL_PTR_CAST(IObject,obj);
}

class XPlugin: public IPlugin
{
public:
	virtual const iplChar *getID()	
	{
		return "appropolis.ipl.SimpleFeature"; 
	};

	virtual	const iplChar *getName()
	{	
		return "OGC Simple Feature Service"; 
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
		g_osfService = new XSFService;
		
		platform->registerService(IPL_SERVICE_SF, g_osfService);
		
		platform->getRegisterService()->registerObject( createVectorDataSource);

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


