#pragma once

#include "core/ipldef.h"
#include "core/iplstd.h"
#include "core/iplmacro.h"
#include "core/interface/IPlugin.h"
#include "core/interface/IService.h"
#include "core/interface/IObject.h"

namespace ipl
{
	class iplPluginInfo
	{
	public:
		iplString	pluginName;
		iplString	pluginVersion;
		iplString	pluginID;
		iplString	pluginProvider;
	};

	// ����ע�����
	interface IRegisterService : public IService
	{
	public:
		//ע����
		virtual bool registerObject(IPL_OBJECT_CREATOR_FUN objCreator) = 0;

		//�����������isForced���ر��򿪣���ǿ�Ƹ��²��XML��
		virtual bool addPlugin(const iplChar * pluginName, bool isForced = false) = 0;

		//�Ƴ���������isForced���ر��򿪣���ǿ�Ƹ��²��XML��
		virtual bool removePlugin(const iplChar * pluginName, bool isForced = false) = 0;

		// ͨ���������Ƶõ�����
		virtual IObjectDesc* getObjectDescByID(const iplChar * objID) = 0;

		// �õ����еĶ�������
		virtual iplArray<IObjectDesc* > getAllObjectDescs() = 0;

		// �õ�����ĳ���ӿڵ��㷨����
		//!!!!!ע�⣬interfaceClass��interface�������ƣ������ǽӿڵ�����
		virtual iplArray<IObjectDesc* > getObjectDescsByInterface(const iplChar * interfaceName) = 0;

		// ��������ʵ��
		virtual IObject *createObject(const iplChar *objectID) = 0;

		//�ӿ�����
		IPL_INTERFACE_DEF(IService, _T("register"))
	};



	interface IPluginManager : public IObject
	{
	public:
		//�����������isForced���ر��򿪣���ǿ�Ƹ��²��XML��
		virtual bool addPlugin(const iplChar * pluginName, bool isForced = false) = 0;

		//�Ƴ���������isForced���ر��򿪣���ǿ�Ƹ��²��XML��
		virtual bool removePlugin(const iplChar * pluginName, bool isForced = false) = 0;

		//scan�ļ����ڲ����еĲ��
		virtual bool scanDir(const iplChar * dirName, bool isForced = false) = 0;

		//�õ���������Ϣ��������汾�ţ����汾�ţ�
		virtual iplArray<iplPluginInfo> &getPluginInfos() = 0;
		virtual bool getPluginInfo(const iplChar *pluginName, iplPluginInfo &info) = 0;

		IPL_INTERFACE_DEF(IObject, _T("pluginManager"))
	};


}
