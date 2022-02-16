#pragma once

#include "core/iplmacro.h"
#include "core/interface/IProgressMsg.h"

#include <stdarg.h>

namespace ipl
{
	class XProgressMsg : public IProgressMsg
	{
	public:
		//���ȣ���0.0-1.0֮��
		bool process(double status);

		//��Ϣ��ӡ
		int logPrint(iplLogLEVEL loglevel, const iplChar *fmt, ...);

		//�ӿں궨��
		IPL_OBJECT_IMP1(XProgressMsg, IProgressMsg, _T("cmd"), _T("cmd"))
	};


	class XProgressMsgMultiStage : public IProgressMsgMultiStage
	{
	private:
		ref_ptr<IProgressMsg> m_preMsgr;

		double m_progressOffset;
		double m_progressRatio;

		int		m_curProgress;

	public:

		//��Ϣ��ӡ
		int logPrint(iplLogLEVEL loglevel, const iplChar *fmt, ...);

		//���ȣ���0.0-1.0֮��
		bool process(double status);

		void setProcessMsg(ref_ptr<IProgressMsg> &pMsg);
	
		void setStage(double progressOffset, double progressRatio);

	public:
		//�ӿں궨��
		IPL_OBJECT_IMP2(XProgressMsgMultiStage, IProgressMsgMultiStage, IProgressMsg, _T("default"), _T("default multi stage warper"))
	};

}

