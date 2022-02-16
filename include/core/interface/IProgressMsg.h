#pragma once
#include "core/interface/IObject.h"
#include "core/iplmacro.h"

namespace ipl
{
	//////////////////////////////////////////////////////////////////////////
	//
	// ��������������Ϣ��
	//
	interface IProgressMsg : public IObject
	{
	public:
		//���ȣ���0.0-1.0֮��, �������ʾ ȡ������
		virtual bool process(double status) = 0;

		//��Ϣ��ӡ
		virtual int logPrint(iplLogLEVEL loglevel, const iplChar *fmt, ...) = 0;

		//�ӿ�����
		IPL_INTERFACE_DEF(IObject, _T("progressMsg"))
	};

#define IPL_PROGRESSMSG_CMD	_T("ipl.progressMsg.cmd")

	//////////////////////////////////////////////////////////////////////////
	// ��׶δ�����Ϣ���������������յ���Ϣ������

	interface IProgressMsgMultiStage : public IProgressMsg
	{
	public:
		virtual void setProcessMsg(ref_ptr<IProgressMsg> &pMsg) = 0;
		virtual void setStage(double progressOffset, double progressRatio) = 0;

		//�ӿ�����
		IPL_INTERFACE_DEF(IProgressMsg, _T("multiStage"))
	};


#define IPL_PROGRESSMSG_MULTISTAGE	_T("ors.progress.multiStage.default")
}

