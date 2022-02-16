#include "core/iplPlatformUtil.h"

// #include <vector>
// 
// #include <assert.h>

#include "commonAPIs/iplutility.h"

#include "XPlatform.h"
#include "XRegisterService.h"
//#include "orsXLastErrorService.h"
//#include "orsXBinarySerialize.h"
#include "XLogService.h"
#include "XProperty.h"
#include "XXMLSerialize.h"
//#include "orsXRDFService.h"

//#include "orsXExe.h"

//////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////
using namespace ipl;

//////////////////////////////////////////////////////////////////////////
static XPlatform	*s_platform = NULL;

IPlatform *_getPlatform()
{
	
	return s_platform;

	//return (IPlatform*)(&s_platform);
}



IObject* create_keywordProperty(bool bForRegister)
{
	return new XProperty;
}

/*
orsIObject* create_binaryPropertySerialize(bool bForRegister)
{
	return new orsXBinarySerialize;
}*/

IObject* create_xmlPropertySerialize(bool bForRegister)
{
	return new XXMLSerialize;
}

// IObject* create_cmdProcessMsg(bool bForRegister)
// {
// 	return new XProcessMsg;
// }
// 
// 
// IObject* create_cmdProcessMsg_ms(bool bForRegister)
// {
// 	return new XProcessMsg_multiStage;
// }


// #include "orsXUndoManager.h"
// 
// orsIObject* create_undoManager(bool bForRegister)
// {
// 	return new orsXUndoManager;
// }
// 
// #include "orsXUtilityService.h"


//ϵͳ�ڲ�����ע��
void iplRegistCommonObjects()
{
	//ϵͳ���ö��󹤳�ע��
	s_platform->getRegisterService()->registerObject(create_keywordProperty);
	//s_platform->getRegisterService()->registerObject(create_binaryPropertySerialize);
	s_platform->getRegisterService()->registerObject(create_xmlPropertySerialize);
}


//ע���ڲ�������ķ���
void iplRegistCoreServices()
{
	s_platform->registerService( IPL_logServiceName, new XLogService);

	//ϵͳ����ע��
	XRegisterService* registerService(new XRegisterService);
	//ÿ�������������Ӧһ�������Ķ��󣬲���һ��������ڶ�������������������ָ�����������
	s_platform->registerService( IPL_registerServiceName, registerService );
//	s_platform->registerService( IPL_pluginManagerName,  registerService );


	//s_platform->registerService( IPL_lastErrorServiceName, new orsXLastErrorService);

// 	s_platform->registerService( IPL_SERVICE_RDF, new orsXRDFService );
// 	s_platform->registerService( IPL_SERVICE_UTILITY, new orsXUtilityService );

// 	s_platform->getRegisterService()->registerObject( create_cmdProcessMsg );
// 	s_platform->getRegisterService()->registerObject( create_cmdProcessMsg_ms );
// 
// 	s_platform->getRegisterService()->registerObject( create_undoManager );
}


//////////////////////////////////////////////////////////////////////////
#ifdef WIN32

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files:
#include <windows.h>
#include <winsock2.h>

extern HMODULE g_hModule;
std::string GetSelfDllDir()
{
	char exeDir[512]= {0};
	char selfDir[512] = {0};
	
	HMODULE hHandle = g_hModule;
	
	GetModuleFileName( hHandle, exeDir, 512);
	char* temp_str = strrchr(exeDir,'\\');
	memcpy(selfDir,exeDir,temp_str-exeDir);
	return selfDir;
}
#endif

void iplSetEnv()
{
	//���̶���λ�ü���ϵͳ����Path��
	//char pathenv[4096];
	//GetEnvironmentVariable("Path",pathenv,4096);
	
	char *pathEnv;
	pathEnv = getenv( "PATH" );

	
	iplString exeDir;
	get_directory_exe(exeDir);

	iplString  plugindir = exeDir + "/plugins";
	iplString  moudledir = exeDir + "/modules";
	iplString  dependentsdir = exeDir + "/dependents";
	iplString  mydir = exeDir;

	iplString  newPathEnv = "PATH=";
	newPathEnv = newPathEnv + mydir + ";" + newPathEnv + plugindir + ";" + moudledir + ";" + dependentsdir;
	
	if(pathEnv != NULL)
		newPathEnv = newPathEnv + ";" + pathEnv;

    //pathEnv = newPathEnv.c_str();
	//SetEnvironmentVariable("Path",pathenvs.c_str());
	char *writable = new char[newPathEnv.size() + 1];
	std::copy(newPathEnv.begin(), newPathEnv.end(), writable);
	writable[newPathEnv.size()] = '\0'; // don't forget the terminating 0
	putenv(writable);

	if (writable)	delete writable; writable = NULL;

	// 
	char *setEnv = getenv( "GDAL_DATA" );

	if( NULL == setEnv )	{
		
		iplString etcDir;
		get_directory_etc(etcDir);

		iplString gdal_data;
		gdal_data = etcDir + _T("/gdal_data");

		iplString setStr = _T("GDAL_DATA=") + gdal_data;

		writable = new char[setStr.size() + 1];
		std::copy(setStr.begin(), setStr.end(), writable);
		writable[setStr.size()] = '\0'; // don't forget the terminating 0
		putenv(writable);

		if (writable)	delete writable;	writable = NULL;
// 		orsIUtilityService *pUtilService = (orsIUtilityService *)_getPlatform()->getService( IPL_SERVICE_UTILITY );
// 		
// 		if( NULL != pUtilService )	{
// 			orsString gdal_data;
// 			pUtilService->GetDirectory_ETC( gdal_data );
// 			
// 			gdal_data = gdal_data + _T("/gdal_data");
// 			
// 			orsString setStr = _T("GDAL_DATA=") + gdal_data;
// 			
// 			putenv( (char*)(setStr.c_str()) );			
// 		}
	}
}

extern bool g_lazyLoad;

IPL_EXTERN_C IPlatform* ipl::iplInitialize( iplString &errorinfo, bool bInteractiveLog )
{
	s_platform = new XPlatform;

	//ע���ڲ�����
	iplRegistCoreServices();

	iplSetEnv();

	s_platform->getLogService()->enableInteractive( bInteractiveLog );

	iplRegistCommonObjects();

	// �ǽ���������������

	g_lazyLoad = !bInteractiveLog;

	g_lazyLoad = false;

	if( !s_platform->startup(errorinfo) )
		return NULL;

	return s_platform;
}


IPL_EXTERN_C void ipl::iplUninitialize()
{
	//���ò���ĳ�ʼ������

	s_platform->shutdown();

	delete s_platform;
}


