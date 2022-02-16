#pragma once

#include "core/interface/IObject.h"
#include "core/interface/IProperty.h"
#include "core/interface/IDataSource.h"


namespace ipl
{
	//
	// 算法对象接口
	//
	interface IAlgorithm : public IObject
	{
	public:
		virtual bool setInputSource(iplArray<ref_ptr<IDataSource> > &input) = 0;

		virtual bool getOutputSource(iplArray<ref_ptr<IDataSource> > &output) = 0;

		virtual bool setArguments(ref_ptr<IProperty> &parameterArgs) = 0;

		//测试算法与硬件系统的兼容性 (需要硬件加速支持的算法)
		virtual int TestCapability(const char *pszCapability) = 0;

		IPL_INTERFACE_DEF(IObject, _T("algorithm"))
	};
}

