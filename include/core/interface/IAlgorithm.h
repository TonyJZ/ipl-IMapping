#pragma once

#include "core/interface/IObject.h"
#include "core/interface/IProperty.h"
#include "core/interface/IDataSource.h"


namespace ipl
{
	//
	// �㷨����ӿ�
	//
	interface IAlgorithm : public IObject
	{
	public:
		virtual bool setInputSource(iplArray<ref_ptr<IDataSource> > &input) = 0;

		virtual bool getOutputSource(iplArray<ref_ptr<IDataSource> > &output) = 0;

		virtual bool setArguments(ref_ptr<IProperty> &parameterArgs) = 0;

		//�����㷨��Ӳ��ϵͳ�ļ����� (��ҪӲ������֧�ֵ��㷨)
		virtual int TestCapability(const char *pszCapability) = 0;

		IPL_INTERFACE_DEF(IObject, _T("algorithm"))
	};
}

