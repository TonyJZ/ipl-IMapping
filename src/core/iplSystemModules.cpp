//*************************************************************************
//
// Copyright (C) 2008-2018, LIESMARS, Wuhan University
//
// License:  LGPL
//
//*************************************************************************
//
// $Id: orsSystemModules.cpp 2009/8/31 11:40 JWS $
//
// Project: OpenRS
//
// Purpose: 
//
// Author: GUO Wei, Wanshou Jiang, jws@lmars.whu.edu.cn
//
//*************************************************************************
//
// $Log: orsSystemModules.cpp,v $
//
// Revision 1.0 date: 2009/8/31 by JIANG Wanshou
// 改用orsPlugDLL进行管理
//
// Revision 1.0 date: 2008/9/23 by GUO Wei
// new
//

//#include <vector>
#include "commonAPIs/iplutility.h"
#include "core/interface/IPlatform.h"
#include "iplPluginDLL.h"
#include "iplCommon.h"


using namespace ipl;
extern IPlatform * _getPlatform();

#define IPL_MODULE_INIT_DIR		"modules"

// #define IPL_MODULE_INIT_FUN		"orsModuleInitialize"
// #define IPL_MODULE_UNINIT_FUN	"orsModuleUninitialize"
// 
// typedef void (*orsModuleInitializeFun)(orsIPlatform* platform);
// typedef void (*orsModuleUninitializeFun)();
// 
// struct SystemModuleInfo
// {
// 	orsPluginHandle handle;
// 	orsModuleInitializeFun initFun;
// 	orsModuleUninitializeFun uninitFun;
// };
// 
// 
// std::vector<SystemModuleInfo> s_moduleHandles;

iplPluginDLLMap *s_loadedModules=NULL;

//////////////////////////////////////////////////////////////////////////

const iplChar *g_curModuleName = NULL;

bool LoadModule(const iplChar * pluginName )
{

	if(s_loadedModules==NULL)
	{
		s_loadedModules=new iplPluginDLLMap;
	}

	//得到插件的路径名
	iplString pathName = pluginName;	//getPluginPathByName(pluginName);
	
	//创建插件
	ref_ptr<iplPluginDLL> plugin( new iplPluginDLL( pluginName, pathName.c_str() ));
	
	g_curModuleName = pathName.c_str();

	// 加载插件
	if( plugin->load( _getPlatform() ) )
	{
		//放入到插件实体map中
		s_loadedModules->erase( pluginName );
		s_loadedModules->insert( iplPluginDLLMap::value_type( pluginName, plugin) );
		
		return true;
	}
	else	{
		iplString msg = "Fail to add plugin: ";
		msg = msg + pluginName;
		
		_getPlatform()->logPrint( IPL_LOG_WARNING, msg.c_str() );
		return false;
	}
}


//加载系统级模块
void LoadSystemModules()
{
	iplString  sysModuleDir;
	get_directory_exe(sysModuleDir);

	sysModuleDir = sysModuleDir + "/" + IPL_MODULE_INIT_DIR;

	//找到目录下所有的dll
	iplArray<iplString> allDlls;
	
	findAllFiles( sysModuleDir.c_str(), PLUGIN_EXTENSION, allDlls );

	//加载插件
	ipl_uint32 i = 0;
	for(i=0;i<allDlls.size();i++)
	{
		LoadModule( allDlls[i].c_str() );
	}
}

void unLoadSystemModules()
{
	delete s_loadedModules;
}