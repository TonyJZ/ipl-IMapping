#pragma once

#include "core/ipldef.h"

namespace ipl
{
#define IPL_PLUGIN_INSTANCE_GET			"iplGetPluginInstance"
#define IPL_PLUGIN_MAJOR_VERSION_GET	"iplGetPlatformMajorVersion"

	// ���������������
#define PLUGIN_API IPL_EXPORT


	interface IObject;

	// ���󴴽�����ָ������
	typedef IObject* (*IPL_OBJECT_CREATOR_FUN)(bool bForRegister);

	enum iplLicenseTYPE {
		iplltFREE,				// ��ѣ�����
		iplltBUSINESS			// ��ҵ��
	};

	interface IPlatform;

	interface IPlugin
	{
		// plugin id, 
		virtual const iplChar *getID() = 0;

		// plugin mame
		virtual	const iplChar *getName() = 0;

		// �õ�����汾��
		virtual const iplChar *getVersion() = 0;

		// "org.openRS...."
		virtual const iplChar *getProvider() = 0;

		//�����ʼ��,�û������ڸú����ڣ�ע��������
		virtual bool initialize(IPlatform*  platform) = 0;

		//ж�ز��ʱ�������clear����
		virtual void finalize() = 0;

		//////////////////////////////////////////////////////////////////////////`
		virtual iplLicenseTYPE GetLicenseType() { return iplltFREE; };

		// �Ӳ��ȡ��������ID, ���ܡ������ɲ����ɣ��˺�����license�������ϵ���
		virtual bool GetLockID(const char *cryptKey, char cryptedLockID[2048])
		{
			cryptedLockID[0] = 0;
			return false;
		};
	};

}
