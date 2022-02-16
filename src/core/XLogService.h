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


	//��־����
	class XLogService : public ILogService
	{
	private:
		//	ref_ptr<orsIGuiLogExt>	m_guiLogger;

		// �Ƿ�����Ի���
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

		//����
		virtual bool startup(IPlatform *platform);

		//�ر�ʱ������
		virtual void shutdown();

		//�Ƿ������ɹ�
		virtual bool isok();

	public:
		// �������ƣ�ֻ���ʼ����һ��
		virtual void setTaskName(const iplChar *taskName, const iplChar *logFile = NULL);

		//����������Ϣ���
		virtual void fatal(const iplChar * strModule, const iplChar * msg, const iplChar * file = NULL, int row = 0);

		//������Ϣ���
		virtual void error(const iplChar * strModule, const iplChar * msg, const iplChar * file = NULL, int row = 0);

		//������Ϣ���
		virtual void warn(const iplChar * strModule, const iplChar * msg, const iplChar * file = NULL, int row = 0);

		//һ����Ϣ���
		virtual void info(const iplChar * strModule, const iplChar * msg, const iplChar * file = NULL, int row = 0);

		//������Ϣ���
		virtual void debug(const iplChar * strModule, const iplChar * msg, const iplChar * file = NULL, int row = 0);

		virtual void enableInteractive(bool bEnable = true) { m_bInteractive = bEnable; };

	private:

		IPL_OBJECT_IMP2(XLogService, ILogService, IService, "stdprint", "stdprint")

	};


#define IPL_SERVICE_LOG_DEFAULT "ipl.service.log.log4cxx"


}
