#pragma once

#include "core/iplmacro.h"
#include "core/interface/IProgressMsg.h"

#include <stdarg.h>

namespace ipl
{
	class XProgressMsg : public IProgressMsg
	{
	public:
		//进度，在0.0-1.0之间
		bool process(double status);

		//消息打印
		int logPrint(iplLogLEVEL loglevel, const iplChar *fmt, ...);

		//接口宏定义
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

		//消息打印
		int logPrint(iplLogLEVEL loglevel, const iplChar *fmt, ...);

		//进度，在0.0-1.0之间
		bool process(double status);

		void setProcessMsg(ref_ptr<IProgressMsg> &pMsg);
	
		void setStage(double progressOffset, double progressRatio);

	public:
		//接口宏定义
		IPL_OBJECT_IMP2(XProgressMsgMultiStage, IProgressMsgMultiStage, IProgressMsg, _T("default"), _T("default multi stage warper"))
	};

}

