#pragma once

#include "core/interface/IService.h"

namespace ipl
{
	// 日志插件对象接口
	interface ILogger : public IObject
	{
	public:
		// 任务名称和日志文件，只需最开始设置一次
		virtual void setTaskName(const iplChar *taskName) = 0;

		// 如果设置了文件名，即为文件方式，否则为控制台方式或自定义方式
		virtual void setLogFile(const iplChar *logFile, bool bAppend = true) = 0;

		// 设置日志样式
		virtual void setLayout(const iplChar *layout, const char *conversionPattern = NULL) = 0;

	public:
		//致命错误信息输出
		virtual void fatal(const iplChar *  msg) = 0;

		//错误信息输出
		virtual void error(const iplChar *  msg) = 0;

		//警告信息输出
		virtual void warn(const iplChar *  msg) = 0;

		//一般信息输出
		virtual void info(const iplChar *  msg) = 0;

		//调试信息输出
		virtual void debug(const iplChar *  msg) = 0;

		//接口名称
		IPL_INTERFACE_DEF(IObject, _T("logger"))
	};



	//日志服务
	interface ILogService : public IService
	{
	public:
		// 任务名称和日志文件，只需最开始设置一次
		virtual void setTaskName(const iplChar *taskName, const iplChar *logFile = NULL) = 0;

	public:
		//致命错误信息输出
		virtual void fatal(const iplChar *  strModule, const iplChar *  msg, const iplChar *  file = NULL, int row = 0) = 0;

		//错误信息输出
		virtual void error(const iplChar *  strModule, const iplChar *  msg, const iplChar *  file = NULL, int row = 0) = 0;

		//警告信息输出
		virtual void warn(const iplChar *  strModule, const iplChar *  msg, const iplChar *  file = NULL, int row = 0) = 0;

		//一般信息输出
		virtual void info(const iplChar *  strModule, const iplChar *  msg, const iplChar *  file = NULL, int row = 0) = 0;

		//调试信息输出
		virtual void debug(const iplChar *  strModule, const iplChar *  msg, const iplChar *  file = NULL, int row = 0) = 0;

		// 是否允许交互式消息框
		virtual void enableInteractive(bool bEnable = true) = 0;

		//接口名称
		IPL_INTERFACE_DEF(IService, _T("log"))
	};

#define IPL_SERVICE_LOG _T("ipl.service.log")


	// 获取日志服务的宏定义
	ILogService *getLogService();

#define IPL_GET_LOG_SERVICE_IMPL()	\
static ILogService *s_logService = NULL;\
ILogService *getLogService()\
{\
	if( NULL != s_logService )\
	return s_logService;\
	\
	s_logService =\
	IPL_PTR_CAST( ILogService, getPlatform()->getService( IPL_SERVICE_LOG) );\
	\
	return s_logService;\
}


}
