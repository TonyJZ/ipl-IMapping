#pragma once


#include "core/interface/IPlatform.h"
#include "core/interface/ILogService.h"
#include "core/iplmacro.h"

//#include <vector>

namespace ipl
{
	struct  log4AppenderINFO
	{
		iplString appenerID;
		iplString logFileName;
		bool	  bAppend;
		iplString layout;
		iplString conversionPattern;
	};


	//日志服务
	class XLogService : public ILogService
	{
	private:
		//	ref_ptr<orsIGuiLogExt>	m_guiLogger;

		// 是否允许对话框
		bool m_bInteractive;

		std::vector<log4AppenderINFO>		m_vAppendInfos;

		std::vector<ref_ptr<ILogger> >	m_vAppenders;

		FILE		*m_logFile;
		iplString	m_taskName;

		iplLogLEVEL m_logLevel;

	public:
		XLogService()
		{
			m_bInteractive = false;

			m_logFile = NULL;
		};

		//开启
		virtual bool startup(IPlatform *platform);

		//关闭时被调用
		virtual void shutdown();

		//是否启动成功
		virtual bool isok();

	public:
		// 任务名称，只需最开始设置一次
		virtual void setTaskName(const iplChar *taskName, const iplChar *logFile = NULL);

		//致命错误信息输出
		virtual void fatal(const iplChar * strModule, const iplChar * msg, const iplChar * file = NULL, int row = 0);

		//错误信息输出
		virtual void error(const iplChar * strModule, const iplChar * msg, const iplChar * file = NULL, int row = 0);

		//警告信息输出
		virtual void warn(const iplChar * strModule, const iplChar * msg, const iplChar * file = NULL, int row = 0);

		//一般信息输出
		virtual void info(const iplChar * strModule, const iplChar * msg, const iplChar * file = NULL, int row = 0);

		//调试信息输出
		virtual void debug(const iplChar * strModule, const iplChar * msg, const iplChar * file = NULL, int row = 0);

		virtual void enableInteractive(bool bEnable = true) { m_bInteractive = bEnable; };

	private:

		IPL_OBJECT_IMP2(XLogService, ILogService, IService, "stdprint", "stdprint")

	};


#define IPL_SERVICE_LOG_DEFAULT "ipl.service.log.log4cxx"


}
