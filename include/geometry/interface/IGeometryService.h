#pragma once

#include "core/interface/IPlatform.h"
#include "core/interface/IService.h"

// #include "orsBase/orsMatrix.h"
// 
// #include "orsGeometry/orsIRegion.h"

namespace ipl
{

	interface IGeometryService : public IService
	{
		//目前不知道要做什么  Tony 2018-01-19 
	public:
		

	public:
		IPL_INTERFACE_DEF(IService, "geometry")
	};


#define IPL_SERVICE_GEOMETRY	"ipl.service.geometry"


	// 获取几何服务的宏定义
	IGeometryService *getGeometryService();

#define IPL_GET_GEOMETRY_SERVICE_IMPL()	\
	static IGeometryService *s_geometryService = NULL;\
	IGeometryService *getGeometryService()\
{\
	if( NULL != s_geometryService )\
	return s_geometryService;\
	\
	s_geometryService =\
	IPL_PTR_CAST(IGeometryService, getPlatform()->getService( IPL_SERVICE_GEOMETRY) );\
	\
	return s_geometryService;\
}

}

//#include "geometry/interface/IGeometry.h"
