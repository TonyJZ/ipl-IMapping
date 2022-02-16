#pragma once
#include "core/interface/IObject.h"
#include "core/iplmacro.h"

namespace ipl
{
	//////////////////////////////////////////////////////////////////////////
	//
	// 进度条，错误信息等
	//
	interface IProgressMsg : public IObject
	{
	public:
		//进度，在0.0-1.0之间, 返回零表示 取消任务
		virtual bool process(double status) = 0;

		//消息打印
		virtual int logPrint(iplLogLEVEL loglevel, const iplChar *fmt, ...) = 0;

		//接口名称
		IPL_INTERFACE_DEF(IObject, _T("progressMsg"))
	};

#define IPL_PROGRESSMSG_CMD	_T("ipl.progressMsg.cmd")

	//////////////////////////////////////////////////////////////////////////
	// 多阶段处理消息，必须先设置最终的消息处理器

	interface IProgressMsgMultiStage : public IProgressMsg
	{
	public:
		virtual void setProcessMsg(ref_ptr<IProgressMsg> &pMsg) = 0;
		virtual void setStage(double progressOffset, double progressRatio) = 0;

		//接口名称
		IPL_INTERFACE_DEF(IProgressMsg, _T("multiStage"))
	};


#define IPL_PROGRESSMSG_MULTISTAGE	_T("ors.progress.multiStage.default")
}

