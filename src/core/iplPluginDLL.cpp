#include "iplPluginDLL.h"

#include "core/interface/IPlatform.h"
#include "core/interface/ILogService.h"

using namespace ipl;
extern IPlatform * _getPlatform();

iplPluginDLL::iplPluginDLL(const iplChar * name, const iplChar * path)
{
	m_pluginName = name;
	m_pathName = path;

	m_hDLL = NULL;

	m_plugin = NULL;
}

iplPluginDLL::~iplPluginDLL()
{
	unload();
}



typedef IPlugin *(*iplGetPluginInstancePtr)(IPlatform* platform);
typedef const char *(*iplgetPlatformMajorVersionPtr)();

bool iplPluginDLL::load(IPlatform* platform)
{
	printf("load plugin :%s\n",m_pathName.c_str());


	//1)加载动态库
	m_hDLL = loadDynamicLib( m_pathName.c_str() );
	if( m_hDLL  == NULL)
		return false;


	iplString  msg = m_pathName + " loaded" ;
	platform->getLogService()->debug( "iplCore", msg.c_str() );


	//2)加载symbol函数
	iplgetPlatformMajorVersionPtr versionFun =
		(iplgetPlatformMajorVersionPtr)getDynamicSymbol( m_hDLL , IPL_PLUGIN_MAJOR_VERSION_GET);


	iplGetPluginInstancePtr	instanceFun =
		(iplGetPluginInstancePtr)getDynamicSymbol( m_hDLL , IPL_PLUGIN_INSTANCE_GET);
	if(versionFun == NULL || instanceFun == NULL){
		platform->getLogService()->warn("host", "versionFun == NULL || instanceFun == NULL");
		return false;
	}


	iplString platformMajorVersion;
	platformMajorVersion = versionFun();
	m_plugin = instanceFun( platform );

	//3) 插件加载?
	//判断插件的主平台号和当前平台号是否相符合
	if(platformMajorVersion != IPL_PLATFORM_VERSION)
	{
		_getPlatform()->logPrint(IPL_LOG_ERROR, _T("Version of %s differs from platform"), m_pluginName.c_str());
		return false;
	}

	if( !m_plugin->initialize( _getPlatform() ) )
	{
		_getPlatform()->logPrint(IPL_LOG_ERROR, _T("Failed Initializing %s "), m_pluginName.c_str());
		return false;
	}

	return true;
}

void iplPluginDLL::unload()
{
	if( NULL != m_hDLL )	{
		if(m_plugin != NULL)
		{
			m_plugin->finalize();
		}

		unloadDynamicLib( m_hDLL );

		iplString  msg = m_pathName + " unloaded" ;
		_getPlatform()->getLogService()->debug( "iplCore", msg.c_str() );

		m_plugin = NULL;
		m_hDLL = NULL;
	}
}
