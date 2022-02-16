// utest_iplcore.cpp : Defines the entry point for the console application.
//

#include "core/iplPlatformUtil.h"
#include "core/interface/IPlatform.h"
#include "core/interface/IRegisterService.h"

//#include <boost/filesystem.hpp>



ipl::IPlatform *g_pPlatform = NULL;

ipl::IPlatform *ipl::getPlatform()
{
	return g_pPlatform;
}

using namespace ipl;
int main(int argc, char * argv[])
{
	iplString errorinfo;

	printf("hello ipl\n");

	g_pPlatform = iplInitialize(errorinfo);

	iplString strVersion = getPlatform()->getVersion();

// 	ILogService *logService = getPlatform()->getLogService().get();
// 	logService->setTaskName("ipl", "D:/code_ipl/bin/vs2015/debug/ipl.log");
// 
// 	logService->warn("iplCore", "utest iplCore");

	getPlatform()->logPrint(IPL_LOG_ERROR, "this is testing logPrint");

	iplArray<IObjectDesc* > descripts =
		getPlatform()->getRegisterService()->getAllObjectDescs();

	printf("ipl plug-in objects: \n");
	for (size_t i = 0; i < descripts.size(); i++)
	{
		IObjectDesc* objDesc = descripts[i];

		iplString objectID = objDesc->getID();
		printf("%s\n", objectID.c_str());
		iplString objName = objDesc->getName();
		printf("%s\n", objName.c_str());
		iplString desc = objDesc->getDesc();
		printf("%s\n", desc.c_str());
		iplString pluginName = objDesc->getPluginName();
		printf("%s\n", pluginName.c_str());
		iplArray<iplString> interfaces = objDesc->getInterfaceNames();
	}

	iplUninitialize();
}

