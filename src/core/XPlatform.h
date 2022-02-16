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

		//得到版本号
		virtual const iplChar *getVersion() { return IPL_PLATFORM_VERSION; }

		//注册服务（每个serviceName只对应一个表示）
		virtual bool registerService(const iplChar * serviceName, IService* service);

		//反注册服务
		virtual void unregisterService(const iplChar * serviceName);

		//得到所有的服务
		virtual iplArray< ref_ptr<IService> > getAllService();

		//通过服务名得到服务单例
		virtual ref_ptr<IService> getService(const iplChar * serviceName);

		//得到服务列表
		virtual iplArray<iplString> getServiceNames();

		//
		// 得到几个固定服务（加快访问速度), 改为普通指针，这singleton服务用户不能释放
		// 全局日志服务
		virtual ref_ptr<ILogService> getLogService();

		//得到注册服务
		virtual ref_ptr<IRegisterService> getRegisterService();

		//得到错误查看服务
		//virtual orsILastErrorService *getLastErrorService();

	public:
		// 通过 注册的对象名称 创建对象, 调用者需要用ref_ptr保存  需要考虑IPlatform内部管理tony 2018-01-18
		virtual IObject *createObject(const iplChar * objName);

		// 创建属性对象
		virtual IProperty *createProperty();

		// 根据 “处理对象” 自动创建 对应“处理节点”的对象
		//virtual orsIConnectableObject *CreateConnectableObject(orsIObject *logicObject);

	public:
		//打印日志
		int logPrint(iplLogLEVEL loglevel, const iplChar *fmt, ...);

		// 取给点类型的无效值
		ipl_float64 getDefaultNULL(iplDataTYPE type)
		{
			return iplGetDefaultNULL(type);
		}

		// 取类型的的范围
		ipl_float64 getDefaultMin(iplDataTYPE type)
		{
			return iplGetDefaultMin(type);
		}

		ipl_float64 getDefaultMax(iplDataTYPE type)
		{
			return iplGetDefaultMax(type);
		}

		//辅助方法，得到类型的字节数sizeof
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
		//启动平台
		bool startup(iplString &errorInfo);

		//关闭平台，卸载资源
		void shutdown();

	protected:
		//服务对应表
		ServiceMap m_services;

		//缺省log服务
		ref_ptr<ILogService> m_logService;

		//缺省注册服务
		ref_ptr<IRegisterService> m_registerService;

		//最近错误服务
//		ref_ptr<ILastErrorService> m_lastErrorService;

		//是否启动
		bool m_bStartup;
	};

}

