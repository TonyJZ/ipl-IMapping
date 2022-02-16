#pragma once

#include "core/interface/IService.h"

namespace ipl
{
	// ��־�������ӿ�
	interface ILogger : public IObject
	{
	public:
		// �������ƺ���־�ļ���ֻ���ʼ����һ��
		virtual void setTaskName(const iplChar *taskName) = 0;

		// ����������ļ�������Ϊ�ļ���ʽ������Ϊ����̨��ʽ���Զ��巽ʽ
		virtual void setLogFile(const iplChar *logFile, bool bAppend = true) = 0;

		// ������־��ʽ
		virtual void setLayout(const iplChar *layout, const char *conversionPattern = NULL) = 0;

	public:
		//����������Ϣ���
		virtual void fatal(const iplChar *  msg) = 0;

		//������Ϣ���
		virtual void error(const iplChar *  msg) = 0;

		//������Ϣ���
		virtual void warn(const iplChar *  msg) = 0;

		//һ����Ϣ���
		virtual void info(const iplChar *  msg) = 0;

		//������Ϣ���
		virtual void debug(const iplChar *  msg) = 0;

		//�ӿ�����
		IPL_INTERFACE_DEF(IObject, _T("logger"))
	};



	//��־����
	interface ILogService : public IService
	{
	public:
		// �������ƺ���־�ļ���ֻ���ʼ����һ��
		virtual void setTaskName(const iplChar *taskName, const iplChar *logFile = NULL) = 0;

	public:
		//����������Ϣ���
		virtual void fatal(const iplChar *  strModule, const iplChar *  msg, const iplChar *  file = NULL, int row = 0) = 0;

		//������Ϣ���
		virtual void error(const iplChar *  strModule, const iplChar *  msg, const iplChar *  file = NULL, int row = 0) = 0;

		//������Ϣ���
		virtual void warn(const iplChar *  strModule, const iplChar *  msg, const iplChar *  file = NULL, int row = 0) = 0;

		//һ����Ϣ���
		virtual void info(const iplChar *  strModule, const iplChar *  msg, const iplChar *  file = NULL, int row = 0) = 0;

		//������Ϣ���
		virtual void debug(const iplChar *  strModule, const iplChar *  msg, const iplChar *  file = NULL, int row = 0) = 0;

		// �Ƿ�������ʽ��Ϣ��
		virtual void enableInteractive(bool bEnable = true) = 0;

		//�ӿ�����
		IPL_INTERFACE_DEF(IService, _T("log"))
	};

#define IPL_SERVICE_LOG _T("ipl.service.log")


	// ��ȡ��־����ĺ궨��
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
