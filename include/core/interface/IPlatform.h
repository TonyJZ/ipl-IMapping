#pragma once

#	if defined _WIN32 &&  _MSC_VER > 1000
#   if !defined(_DLL) || !defined(_MT)
#       error "Only multi-threaded C++ runtime DLL libraries can be used with OpenRS!"
#   endif
#	endif

// #include <assert.h>
// #include <stdio.h>
//#include "warningdisable.h"

#include "core/ipldef.h"
#include "core/iplmacro.h"
#include "core/interface/IObject.h"

namespace ipl
{
// 	interface orsIPlatform;
// 	//interface orsIObject;
// 	interface orsIPropertySerialize;
// 	interface orsIProcessMsg;
 	interface IService;
 	interface ILogService;
 	interface IRegisterService;
// 	interface orsILastErrorService;
 	interface IProperty;
// 	interface orsIConnectableObject;



#define IPL_PLATFORM_VERSION	"0.1.0"



// 	enum orsLicenseServiceTYPE {
// 		orsLstNodeLOCK,		// ����
// 		orsLstFLOATING			// ����
// 	};




	//ƽ̨����ע������������ڷ���������֣�������������UUID����������������ģ���ʶ
	interface IPlatform
	{
	public:
		//�õ��汾��
		virtual const iplChar *getVersion() = 0;

		// 	//����ƽ̨
		// 	virtual const iplChar *startup( bool &ret ) = 0;
		// 
		// 	//�ر�ƽ̨��ж����Դ
		// 	virtual void shutdown() = 0;

	public:
		//���÷���ʱ����ƽ̨�����ڴ�ռ䣬�������ⲿ�ͷŷ���

		//ע�����(ÿ��moudleֻ��Ӧһ����ʾ)
		//service���ⲿ��������IPlatform�ڲ������������ⲿ�ͷ�
		virtual bool registerService(const iplChar * service_name, IService* service) = 0;

		// 	//�õ������б�
		// 	virtual orsArray<const iplChar *> getServiceNames() = 0;

		//ͨ���������õ�������
		virtual ref_ptr<IService> getService(const iplChar * serviceName) = 0;

		//�õ������̶�����(�ӿ�����ٶ�)
		//ȫ����־����
		virtual ref_ptr<ILogService> getLogService() = 0;
		//�õ�ע�����
		virtual ref_ptr<IRegisterService> getRegisterService() = 0;
		//�õ�����鿴����
		//virtual orsILastErrorService *getLastErrorService() = 0;

	public:
		//������ִ�ж���ʱ���е����߹����ڴ�ռ�, ����������ָ���Ŵ����Ķ���
		//�����߸����ͷ��ڴ�ռ䣬�ұ�����ƽ̨�ر�ǰ�ͷŵ�����ƽ̨�����Ķ���

		// ͨ�� ע��Ķ������� ��������
		virtual IObject *createObject(const iplChar * objIdStr) = 0;

		// �������Զ���
		virtual IProperty *createProperty() = 0;

		// ���� ��������� �Զ����� ��Ӧ������ڵ㡱�Ķ���
		//virtual orsIConnectableObject *CreateConnectableObject(orsIObject *logicObject) = 0;


		// utilities
	public:
		// �ɱ������־
		virtual int logPrint(iplLogLEVEL loglevel, const iplChar *fmt, ...) = 0;

		// ȡ�������͵���Чֵ
		virtual ipl_float64 getDefaultNULL(iplDataTYPE type) = 0;
		// ȡ���͵ĵķ�Χ
		virtual ipl_float64 getDefaultMin(iplDataTYPE type) = 0;
		virtual ipl_float64 getDefaultMax(iplDataTYPE type) = 0;

		//�����������õ����͵��ֽ���sizeof
		virtual int getSizeOfType(iplDataTYPE type) = 0;

// 		virtual void *malloc_(size_t size) = 0;
// 		virtual void free_(void *ptr) = 0;

	};


	// ȡƽ̨����
	IPlatform* getPlatform();

// #define orsMalloc( size )	getPlatform()->malloc_(size)
// #define orsFree( ptr )	getPlatform()->free_( ptr )

	template<class T>
	T* ipl_create_object(T* nullval, const iplChar * id, IPlatform *pPlatform, const iplChar * interfaceName)
	{
		assert(NULL != pPlatform);
		//	printf("ID:%s\n", id);
		IObject *obj = pPlatform->createObject(id);
		if (obj != NULL)
			return (T*)obj->queryInterface(interfaceName);
		else
			return NULL;
	}

#define IPL_CREATE_OBJECT(T, id) (ipl_create_object((T*)NULL, id, getPlatform(), #T))

}
