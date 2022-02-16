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
// 		orsLstNodeLOCK,		// 本地
// 		orsLstFLOATING			// 浮动
// 	};




	//平台服务，注意服务名不等于服务本身的名字，服务本身名称是UUID，而服务名代表功能模块标识
	interface IPlatform
	{
	public:
		//得到版本号
		virtual const iplChar *getVersion() = 0;

		// 	//启动平台
		// 	virtual const iplChar *startup( bool &ret ) = 0;
		// 
		// 	//关闭平台，卸载资源
		// 	virtual void shutdown() = 0;

	public:
		//调用服务时，由平台管理内存空间，不可在外部释放服务

		//注册服务(每个moudle只对应一个表示)
		//service由外部创建，在IPlatform内部管理，不可在外部释放
		virtual bool registerService(const iplChar * service_name, IService* service) = 0;

		// 	//得到服务列表
		// 	virtual orsArray<const iplChar *> getServiceNames() = 0;

		//通过服务名得到服务单例
		virtual ref_ptr<IService> getService(const iplChar * serviceName) = 0;

		//得到几个固定服务(加快访问速度)
		//全局日志服务
		virtual ref_ptr<ILogService> getLogService() = 0;
		//得到注册服务
		virtual ref_ptr<IRegisterService> getRegisterService() = 0;
		//得到错误查看服务
		//virtual orsILastErrorService *getLastErrorService() = 0;

	public:
		//创建可执行对象时，有调用者管理内存空间, 建议用智能指针存放创建的对象
		//调用者负责释放内存空间，且必须在平台关闭前释放掉所有平台创建的对象

		// 通过 注册的对象名称 创建对象
		virtual IObject *createObject(const iplChar * objIdStr) = 0;

		// 创建属性对象
		virtual IProperty *createProperty() = 0;

		// 根据 “处理对象” 自动创建 对应“处理节点”的对象
		//virtual orsIConnectableObject *CreateConnectableObject(orsIObject *logicObject) = 0;


		// utilities
	public:
		// 可变参数日志
		virtual int logPrint(iplLogLEVEL loglevel, const iplChar *fmt, ...) = 0;

		// 取给点类型的无效值
		virtual ipl_float64 getDefaultNULL(iplDataTYPE type) = 0;
		// 取类型的的范围
		virtual ipl_float64 getDefaultMin(iplDataTYPE type) = 0;
		virtual ipl_float64 getDefaultMax(iplDataTYPE type) = 0;

		//辅助方法，得到类型的字节数sizeof
		virtual int getSizeOfType(iplDataTYPE type) = 0;

// 		virtual void *malloc_(size_t size) = 0;
// 		virtual void free_(void *ptr) = 0;

	};


	// 取平台对象
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
