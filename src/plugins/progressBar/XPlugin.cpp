#include "core/interface/IPlatform.h"
#include "core/interface/IObject.h"
#include "core/interface/IPlugin.h"
#include "core/interface/IRegisterService.h"

#include "XProgressMsg_cmd.h"

ipl::IPlatform *s_platform = NULL;

ipl::IPlatform * _getPlatform()
{
	return s_platform;
}

/////////////////////////////////////////////////////////////////////////

using namespace ipl;

IObject* createcmdProgressMsg(bool bForRegister)
{
	return new XProgressMsg;
}


IObject* createcmdProgressMsg_ms(bool bForRegister)
{
	return new XProgressMsgMultiStage;
}


class XPlugin: public IPlugin
{
public:
	virtual const iplChar *getID()
	{
		return "appropolis.ipl.progressBar";
	};

	virtual	const iplChar *getName()
	{
		return "progress bar";
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
		s_platform = platform;

		ref_ptr<IRegisterService> pRegister = platform->getRegisterService();

		pRegister->registerObject(createcmdProgressMsg);
		pRegister->registerObject(createcmdProgressMsg_ms);
		
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


