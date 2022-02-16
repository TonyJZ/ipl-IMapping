#pragma once

#include "core/ipldef.h"
#include "core/iplstd.h"
#include "core/iplmacro.h"
#include "core/interface/IObject.h"

namespace ipl
{
	interface IPlatform;

	interface IService : public IObject
	{
	public:
		//����ʱ������
		virtual bool startup( IPlatform *platform ) = 0;

		//�ر�ʱ������
		virtual void shutdown() = 0;

		//�Ƿ������ɹ�
		virtual bool isok() = 0;

		//����
		//virtual std::string	getDesc() = 0;

		//����
		IPL_INTERFACE_DEF(IObject, _T("service"))
	};

#define IPL_SERVICE _T("ipl.service")
}

