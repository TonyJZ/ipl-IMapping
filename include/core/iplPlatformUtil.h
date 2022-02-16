#pragma once

//ֻ�ṩ��ϵͳ�����ߣ����ṩ����������ߣ�ϵͳDLLΨһ������ϵͳ������ͨ��ƽָ̨�����

#include "core/ipldef.h"
#include "core/iplstd.h"
#include "core/interface/IPlatform.h"
//#include "orsBase/orsString.h"
#include "core/interface/ILogService.h"

namespace ipl
{
	//����openRS����
	IPL_EXTERN_C IPL_BASE_API IPlatform* iplInitialize(iplString &errorinfo, bool bInteractiveLog = false);

	//�ر�openRS���岢�ͷ��ڴ�
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
