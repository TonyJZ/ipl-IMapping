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
		//启动时被调用
		virtual bool startup( IPlatform *platform ) = 0;

		//关闭时被调用
		virtual void shutdown() = 0;

		//是否启动成功
		virtual bool isok() = 0;

		//名称
		//virtual std::string	getDesc() = 0;

		//名称
		IPL_INTERFACE_DEF(IObject, _T("service"))
	};

#define IPL_SERVICE _T("ipl.service")
}

