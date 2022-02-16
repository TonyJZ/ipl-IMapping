#pragma once

//只提供给系统开发者，不提供给插件开发者，系统DLL唯一导出，系统开发者通过平台指针调用

#include "core/ipldef.h"
#include "core/iplstd.h"
#include "core/interface/IPlatform.h"
//#include "orsBase/orsString.h"
#include "core/interface/ILogService.h"

namespace ipl
{
	//启动openRS主体
	IPL_EXTERN_C IPL_BASE_API IPlatform* iplInitialize(iplString &errorinfo, bool bInteractiveLog = false);

	//关闭openRS主体并释放内存
	IPL_EXTERN_C IPL_BASE_API void iplUninitialize();

	//////////////////////////////////////////////////////////////////////////

#define IplLOG_DEBUG( strModule, message) { \
	getLogService()->debug( strModule, message, __FILE__, __LINE__); }

#define IplLOG_INFO(strModule, message) { \
	getLogService()->info( strModule, message, __FILE__, __LINE__); }

#define IplLOG_WARN(strModule, message) { \
	getLogService()->warn( strModule, message, __FILE__, __LINE__); }

#define IplLOG_ERROR(strModule, message) { \
	getLogService()->error( strModule, message, __FILE__, __LINE__); }

#define IplLOG_FATAL(strModule, message) { \
	getLogService()->fatal( strModule, message, __FILE__, __LINE__); }


}
