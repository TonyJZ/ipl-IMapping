#include "core/interface/IRegisterService.h"
#include "core/interface/ILogService.h"


#include "XPlatform.h"
#include "XProperty.h"

using namespace ipl;

//反注册服务
void XPlatform::unregisterService( const iplChar * serviceName)
{
	ServiceMap::iterator iter = m_services.find( serviceName );
	if(iter != m_services.end())
	{
		IService* service = iter->second.get();
		service->shutdown();
		m_services.erase(iter);
	}
}

iplArray<iplString> XPlatform::getServiceNames()
{
	iplArray<iplString> servicelist;
	ServiceMap::iterator iter = m_services.begin();
	while(iter != m_services.end()){
		servicelist.push_back(iter->first.c_str());
		iter++;
	}
	return servicelist;
}

//得到所有的服务
iplArray<ref_ptr<IService> > XPlatform::getAllService()
{
	iplArray<ref_ptr<IService> > servicelist;
	ServiceMap::iterator iter = m_services.begin();

	while(iter != m_services.end()){
		servicelist.push_back( iter->second );
		iter++;
	}
	return servicelist;
}

ref_ptr<IService> XPlatform::getService(const iplChar * serviceName)
{
	ServiceMap::iterator iter = m_services.find( serviceName );
	if(iter == m_services.end()) {
		logPrint( IPL_LOG_ERROR,  _T("Can not get service: %s"), serviceName );
		return NULL;
	}
	else{
		return iter->second;
	}
}

ref_ptr<ILogService> XPlatform::getLogService()
{
	if( NULL == m_logService )
		m_logService = ipl_static_pointer_cast<ILogService>(getService(IPL_logServiceName));

	return m_logService;
}

ref_ptr<IRegisterService> XPlatform::getRegisterService()
{
	if(  NULL == m_registerService.get() )
		m_registerService = ipl_static_pointer_cast<IRegisterService>(getService(IPL_registerServiceName));

	return m_registerService;
}

// orsILastErrorService *XPlatform::getLastErrorService()
// {
// 	if(  NULL == m_lastErrorService.get() )
// 	 	m_lastErrorService = (orsILastErrorService*)getService(IPL_lastErrorServiceName);
// 
// 	return m_lastErrorService.get();
// }



IObject* XPlatform::createObject(const iplChar * name)
{
	return m_registerService->createObject(name); 
}

IProperty *XPlatform::createProperty()
{
	return new XProperty();
}


#define _IPL_CREATE_OBJECT(T, id) (ipl_create_object((T*)NULL, id, _getPlatform(), #T))

// orsIConnectableObject *XPlatform::CreateConnectableObject( IObject *logicObject )
// {
// 	IRegisterService *registerService = _getPlatform()->getRegisterService();
// 
// 	iplArray<ref_ptr<IObjectDesc> > objDescs =
// 		registerService->getObjectDescsByInterface( "orsIConnectableObject" );
// 
// 	// 逐个试探
// 	for(unsigned int i=0;i<objDescs.size();i++)
// 	{
// 		IObjectDesc *desc = objDescs[i].get();
// 		orsIConnectableObject *connectObj =
// 			_IPL_CREATE_OBJECT( orsIConnectableObject, desc->getID() );
// 
// 		if(connectObj != NULL){
// 			if( connectObj->setLogicObject( logicObject ) )
// 				return connectObj;
// 		}
// 	}
// 
// 	return NULL;
// }
// 

//////////////////////////////////////////////////////////////////////////

bool XPlatform::registerService( const iplChar * serviceName, IService* service)
{
	unregisterService( serviceName );
	m_services.erase( serviceName );
	m_services.insert( ServiceMap::value_type(serviceName, ref_ptr<IService>(service)));

	if( NULL == m_logService ) {
		iplString msg = service->getClassID() + ": registered";
//		m_logService->debug( "iplCore", msg.c_str() );
	}
	else	{
// 		iplString msg = service->getClassID() + ": registered";
// 
// 		printf("debug: %s\n", msg.c_str() );
	}

	bool needStartup = m_bStartup;

	//判断之前是否已经有该服务的指针
	ServiceMap::iterator iter;
	for (iter = m_services.begin();iter != m_services.end();iter++)
	{
		IService* oldservice = iter->second.get();
		if(oldservice == service)
		{
			needStartup = false;
			break;
		}
	}

	if(needStartup){
		//需要错误处理
		if( !service->startup( this ) ) {
			iplString error_str = service->getClassID() + ": startup failed";

			m_logService->warn( "iplCore", error_str.c_str() );

			//getLastErrorService()->setErrorInfo(0,  error_str.c_str() );
			return false;
		}
		else	{
			iplString msg = service->getClassID() + ": startup";
			m_logService->debug( "iplCore", msg.c_str() );
		}
	}

	//得到固定的服务名
	if( NULL == m_logService )
		m_logService = getLogService();

// 	m_registerService = (IRegisterService*)getService(IPL_registerServiceName);
// 	m_lastErrorService = (orsILastErrorService*)getService(IPL_lastErrorServiceName);

	return true;
}

void LoadSystemModules();

bool XPlatform::startup(iplString &errorInfo)
{
	//得到固定的服务名
	if(NULL == m_logService)
		m_logService = getLogService();
	
	if (NULL == m_registerService)
		m_registerService = getRegisterService();
	//m_lastErrorService = (orsILastErrorService*)getService(IPL_lastErrorServiceName);

	if(m_logService == NULL)
	{
		errorInfo = "log service can not be found";
		return false;
	}

	if(m_registerService == NULL)
	{
		errorInfo = "register service can not be found";
		return false;
	}

// 	if(m_lastErrorService == NULL)
// 	{
// 		errorInfo = "last error service can not be found";
// 		return false;
// 	}

	//最先启动日志服务
	m_logService->startup(this);

	// 	//////////////////////////////////////////////////////////////////////////
	// 	// 对于注册过程可能无法监视
	// 	
	// 	m_logService->loadExtension( this );
	
	LoadSystemModules();

	//循环调用
	std::vector<IService *> servicesToBeStart;

	ServiceMap::iterator iter = m_services.begin();
	while(iter != m_services.end()) {
		IService *service = iter->second.get();

		// 不是logService
		if( service != m_logService.get())
		{
			bool needStartup = true;

			// 已经在列表中 
			for (unsigned int i=0; i< servicesToBeStart.size(); i++)
			{
				if( service == servicesToBeStart[i] )
				{
					needStartup = false;
					break;
				}
			}

			// 放入列表
			if( needStartup )
				servicesToBeStart.push_back(service);
		}
		iter++;
	}

	for ( unsigned int i=0; i<servicesToBeStart.size(); i++)
	{
		IService* service = servicesToBeStart[i];

		//需要错误处理
		if( !service->startup( this ) ) {
			iplString error_str = service->getClassID() + ": startup failed";
			m_logService->info( "iplCore", error_str.c_str() );
			return false;
		}
		else	{
			iplString msg = service->getClassID() + ": startup";
			m_logService->debug( "iplCore", msg.c_str() );
		}
	}

	m_bStartup = true;
	return true;
}


void unLoadSystemModules();
void XPlatform::shutdown()
{
	std::vector< IService* > stopservices;

//	m_logService->unloadExtension();

	ServiceMap::iterator iter = m_services.begin();
	while(iter != m_services.end()){
		IService* service = iter->second.get();
		if(service != m_logService.get())
		{
			bool needShutdown = true;
			for ( unsigned int i=0;i<stopservices.size();i++)
			{
				if(service == stopservices[i])
				{
					needShutdown = false;
					break;
				}
			}

			if(needShutdown)
				stopservices.push_back(service);
		}
		iter++;
	}

	for ( unsigned int i=0;i<stopservices.size();i++)
	{
		IService* service = stopservices[i];
		if(service != m_registerService.get() &&
			service != m_logService.get())
			service->shutdown();
	}

	stopservices.clear();
	m_services.clear();

	//////////////////////////////////////////////////////////////////////////
	unLoadSystemModules();
	//////////////////////////////////////////////////////////////////////////

	// 最后终止日志服务和注册服务
	m_logService->shutdown();
	m_registerService->shutdown();

//	m_registerService = NULL;
//	m_lastErrorService = NULL;
//	m_logService = NULL;

	m_bStartup = false;
}


#include <stdarg.h>

int XPlatform::logPrint(iplLogLEVEL loglevel, const iplChar *fmt, ...)
{
	IPlatform *platform = this;

	char buf[IPL_MAX_LOG_ITEM_SIZE];

	int count;
	va_list args;
	va_start(args,fmt);
	count = vsprintf(buf,fmt,args);
	va_end(args);

	ILogService *service =  platform->getLogService().get();
	if(service != NULL)
	{
		switch(loglevel)
		{
		case IPL_LOG_WARNING:
			service->warn("iplCore",buf);
			break;
		case IPL_LOG_INFO:
			service->info("iplCore",buf);
			break;
		case IPL_LOG_ERROR:
			service->error("iplCore",buf);
			break;
		case IPL_LOG_FATAL:
			service->fatal("iplCore",buf);
			break;
		case IPL_LOG_DEBUG:
			service->debug("iplCore",buf);
			break;
		default:
			break;
		}
	}

	return count;
}


