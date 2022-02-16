#include "XProgressMsg_cmd.h"
#include "core/interface/IPlatform.h"


extern ipl::IPlatform * _getPlatform();

using namespace ipl;

//���ȣ���0.0-1.0֮��
bool XProgressMsg::process(double status)
{
	_getPlatform()->logPrint(IPL_LOG_INFO, _T("process %.1lf%%"), status * 100);

	return true;
}


#include <stdarg.h>


//��Ϣ��ӡ
int XProgressMsg::logPrint(iplLogLEVEL loglevel, const iplChar *fmt, ...)
{
	iplChar buf[IPL_MAX_LOG_ITEM_SIZE];

	int count;
	va_list args;
	va_start(args, fmt);
	count = vsprintf(buf, fmt, args);
	va_end(args);

	return _getPlatform()->logPrint(loglevel, _T("%s"), buf);
}


int XProgressMsgMultiStage::logPrint(iplLogLEVEL loglevel, const iplChar *fmt, ...)
{
	assert(NULL != m_preMsgr.get());

	iplChar buf[IPL_MAX_LOG_ITEM_SIZE];

	int count;
	va_list args;
	va_start(args, fmt);
	count = vsprintf(buf, fmt, args);

	va_end(args);

	return m_preMsgr->logPrint(loglevel, buf);
}

//���ȣ���0.0-1.0֮��
bool XProgressMsgMultiStage::process(double status)
{
	assert(NULL != m_preMsgr.get());

	double progress = m_progressOffset + m_progressRatio*status;

	// ÿ�ν��������ﵽ0.1����Ӧһ��
	if ((int)(1000 * progress) > m_curProgress) {
		m_curProgress = int(1000 * progress);
		return m_preMsgr->process(0.001*m_curProgress);
	}

	return true;
}

void XProgressMsgMultiStage::setProcessMsg(ref_ptr<IProgressMsg> &pMsg)
{
	m_preMsgr = pMsg;
}

void XProgressMsgMultiStage::setStage(double progressOffset, double progressRatio)
{
	m_progressOffset = progressOffset;
	m_progressRatio = progressRatio;

	m_curProgress = 1000 * progressOffset;
}

