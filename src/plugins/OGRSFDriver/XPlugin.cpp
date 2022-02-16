#include "core/interface/IPlatform.h"
#include "core/interface/IObject.h"
#include "core/interface/IPlugin.h"
#include "core/interface/IRegisterService.h"
#include "simplefeature/interface/ISFService.h"
#include "spatialreference/interface/ISpatialReferenceService.h"

#include "XSFVectorSourceReader.h"
#include "XSFVectorSourceWriter.h"

namespace ipl
{
	IPL_GET_SF_SERVICE_IMPL();
	IPL_GET_SRS_SERVICE_IMPL();
}

//ipl::ISFService *g_osfService = NULL;
/////////////////////////////////////////////////////////////////////////

using namespace ipl;

IObject* createOGRReader( bool bForRegister )
{
	return new XSFVectorSourceReader();
}

IObject* createOGRWriter(bool bForRegister)
{
	return new XSFVectorSourceWriter();
}

class XPlugin: public IPlugin
{
public:
	virtual const iplChar *getID()
	{
		return "appropolis.ipl.OGRSFDriver";
	};

	virtual	const iplChar *getName()
	{
		return "OGR SF Driver";
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
		ref_ptr<IRegisterService> pRegister = platform->getRegisterService();

		pRegister->registerObject(createOGRReader);
		pRegister->registerObject(createOGRWriter);
		
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


