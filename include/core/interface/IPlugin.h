#pragma once

#include "core/ipldef.h"

namespace ipl
{
#define IPL_PLUGIN_INSTANCE_GET			"iplGetPluginInstance"
#define IPL_PLUGIN_MAJOR_VERSION_GET	"iplGetPlatformMajorVersion"

	// 插件导出函数声明
#define PLUGIN_API IPL_EXPORT


	interface IObject;

	// 对象创建函数指针类型
	typedef IObject* (*IPL_OBJECT_CREATOR_FUN)(bool bForRegister);

	enum iplLicenseTYPE {
		iplltFREE,				// 免费，无锁
		iplltBUSINESS			// 商业，
	};

	interface IPlatform;

	interface IPlugin
	{
		// plugin id, 
		virtual const iplChar *getID() = 0;

		// plugin mame
		virtual	const iplChar *getName() = 0;

		// 得到插件版本号
		virtual const iplChar *getVersion() = 0;

		// "org.openRS...."
		virtual const iplChar *getProvider() = 0;

		//插件初始化,用户可以在该函数内，注册插件对象
		virtual bool initialize(IPlatform*  platform) = 0;

		//卸载插件时，做相关clear工作
		virtual void finalize() = 0;

		//////////////////////////////////////////////////////////////////////////`
		virtual iplLicenseTYPE GetLicenseType() { return iplltFREE; };

		// 从插件取加密锁的ID, 加密、解密由插件完成，此函数在license服务器上调用
		virtual bool GetLockID(const char *cryptKey, char cryptedLockID[2048])
		{
			cryptedLockID[0] = 0;
			return false;
		};
	};

}
