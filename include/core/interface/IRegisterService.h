#pragma once

#include "core/ipldef.h"
#include "core/iplstd.h"
#include "core/iplmacro.h"
#include "core/interface/IPlugin.h"
#include "core/interface/IService.h"
#include "core/interface/IObject.h"

namespace ipl
{
	class iplPluginInfo
	{
	public:
		iplString	pluginName;
		iplString	pluginVersion;
		iplString	pluginID;
		iplString	pluginProvider;
	};

	// 对象注册服务
	interface IRegisterService : public IService
	{
	public:
		//注册插件
		virtual bool registerObject(IPL_OBJECT_CREATOR_FUN objCreator) = 0;

		//加入插件，如果isForced开关被打开，将强制更新插件XML树
		virtual bool addPlugin(const iplChar * pluginName, bool isForced = false) = 0;

		//移除插件，如果isForced开关被打开，将强制更新插件XML树
		virtual bool removePlugin(const iplChar * pluginName, bool isForced = false) = 0;

		// 通过对象名称得到描述
		virtual IObjectDesc* getObjectDescByID(const iplChar * objID) = 0;

		// 得到所有的对象描述
		virtual iplArray<IObjectDesc* > getAllObjectDescs() = 0;

		// 得到兼容某个接口的算法描述
		//!!!!!注意，interfaceClass是interface的类名称，而不是接口的名字
		virtual iplArray<IObjectDesc* > getObjectDescsByInterface(const iplChar * interfaceName) = 0;

		// 创建对象实例
		virtual IObject *createObject(const iplChar *objectID) = 0;

		//接口名称
		IPL_INTERFACE_DEF(IService, _T("register"))
	};



	interface IPluginManager : public IObject
	{
	public:
		//加入插件，如果isForced开关被打开，将强制更新插件XML树
		virtual bool addPlugin(const iplChar * pluginName, bool isForced = false) = 0;

		//移除插件，如果isForced开关被打开，将强制更新插件XML树
		virtual bool removePlugin(const iplChar * pluginName, bool isForced = false) = 0;

		//scan文件夹内部所有的插件
		virtual bool scanDir(const iplChar * dirName, bool isForced = false) = 0;

		//得到插件相关信息：（插件版本号，主版本号）
		virtual iplArray<iplPluginInfo> &getPluginInfos() = 0;
		virtual bool getPluginInfo(const iplChar *pluginName, iplPluginInfo &info) = 0;

		IPL_INTERFACE_DEF(IObject, _T("pluginManager"))
	};


}
