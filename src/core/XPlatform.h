#pragma once

#include <string>
#include <map>


#include "core/interface/IPlatform.h"
#include "core/interface/IObject.h"
#include "core/interface/IService.h"

#include "iplCommon.h"
// 
// 
// #include "orsBase/orsUtil.h"

#include "XProperty.h"


namespace ipl
{
	class XPlatform : public IPlatform
	{
		typedef std::map<iplString, ref_ptr<IService> > ServiceMap;

	public:
		XPlatform() {
			int i = 0;
// 			m_logService = NULL;
// 			m_registerService = NULL;
		};

		//�õ��汾��
		virtual const iplChar *getVersion() { return IPL_PLATFORM_VERSION; }

		//ע�����ÿ��serviceNameֻ��Ӧһ����ʾ��
		virtual bool registerService(const iplChar * serviceName, IService* service);

		//��ע�����
		virtual void unregisterService(const iplChar * serviceName);

		//�õ����еķ���
		virtual iplArray< ref_ptr<IService> > getAllService();

		//ͨ���������õ�������
		virtual ref_ptr<IService> getService(const iplChar * serviceName);

		//�õ������б�
		virtual iplArray<iplString> getServiceNames();

		//
		// �õ������̶����񣨼ӿ�����ٶ�), ��Ϊ��ָͨ�룬��singleton�����û������ͷ�
		// ȫ����־����
		virtual ref_ptr<ILogService> getLogService();

		//�õ�ע�����
		virtual ref_ptr<IRegisterService> getRegisterService();

		//�õ�����鿴����
		//virtual orsILastErrorService *getLastErrorService();

	public:
		// ͨ�� ע��Ķ������� ��������, ��������Ҫ��ref_ptr����  ��Ҫ����IPlatform�ڲ�����tony 2018-01-18
		virtual IObject *createObject(const iplChar * objName);

		// �������Զ���
		virtual IProperty *createProperty();

		// ���� ��������� �Զ����� ��Ӧ������ڵ㡱�Ķ���
		//virtual orsIConnectableObject *CreateConnectableObject(orsIObject *logicObject);

	public:
		//��ӡ��־
		int logPrint(iplLogLEVEL loglevel, const iplChar *fmt, ...);

		// ȡ�������͵���Чֵ
		ipl_float64 getDefaultNULL(iplDataTYPE type)
		{
			return iplGetDefaultNULL(type);
		}

		// ȡ���͵ĵķ�Χ
		ipl_float64 getDefaultMin(iplDataTYPE type)
		{
			return iplGetDefaultMin(type);
		}

		ipl_float64 getDefaultMax(iplDataTYPE type)
		{
			return iplGetDefaultMax(type);
		}

		//�����������õ����͵��ֽ���sizeof
		int getSizeOfType(iplDataTYPE type)
		{
			return iplGetSizeOfType(type);
		}

// 		void *malloc_(size_t size)
// 		{
// 			return malloc(size);
// 		}
// 
// 		void free_(void *ptr)
// 		{
// 			free(ptr);
// 		}

	public:
		//����ƽ̨
		bool startup(iplString &errorInfo);

		//�ر�ƽ̨��ж����Դ
		void shutdown();

	protected:
		//�����Ӧ��
		ServiceMap m_services;

		//ȱʡlog����
		ref_ptr<ILogService> m_logService;

		//ȱʡע�����
		ref_ptr<IRegisterService> m_registerService;

		//����������
//		ref_ptr<ILastErrorService> m_lastErrorService;

		//�Ƿ�����
		bool m_bStartup;
	};

}

