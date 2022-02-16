#pragma once

#include "core/ipldef.h"
#include "core/interface/IService.h"

#include "spatialreference/interface/IGeoid.h"
#include "spatialreference/interface/IEllipsoid.h"
#include "spatialreference/interface/ISpatialReference.h"
#include "spatialreference/interface/ICoordinateTransform.h"

namespace ipl
{
	interface  ISpatialReferenceService : public IService
	{
		// create services
		virtual IGeoid *CreateGeoid(const char *wktGeoidName) const = 0;
		virtual IEllipsoid *CreateEllipsoid(const char *wktEllipsoidName = NULL) const = 0;
		virtual ITangentPlane *CreateTangentPlane() const = 0;
		virtual ISpatialReference *CreateSpatialReference(const char *wktHcs = NULL, const char *wktVcs = NULL) const = 0;
		virtual ICoordinateTransform *CreateCoordinateTransform() const = 0;

		//virtual orsISrsDlg *CreateSrsDlg() const = 0;

		//////////////////////////////////////////////////////////////////////////
		// query services
		virtual bool GetAvailableGeoidNames(iplArray <iplString> &geoidNames) = 0;
		virtual bool GetAvailableEllipsoidNames(iplArray <iplString> &ellipsoidNames) = 0;

		// query services for config
		virtual bool GetAvailableDatumNames(iplArray <iplString> &pcsNames) = 0;

		// Geographic Coordinate Systems
		virtual bool GetAvailableGcsNames(iplArray <iplString> &pcsNames) = 0;

		// Projected Coordinate System
		virtual bool GetAvailablePcsNames(iplArray <iplString> &pcsNames) = 0;

		// Vertical Coordinate System
		virtual bool GetAvailableVcsNames(iplArray <iplString> &pcsNames) = 0;

		IPL_INTERFACE_DEF(IService, _T("SRS"));
	};

#define IPL_SERVICE_SRS			_T("ipl.service.SRS")
#define IPL_SERVICE_SRS_DEFAULT	_T("ipl.service.SRS.default")


	// 获取日志服务的宏定义
	ISpatialReferenceService *getSRService();

#define IPL_GET_SRS_SERVICE_IMPL()	\
	static ISpatialReferenceService *s_SRSService = NULL;\
	ISpatialReferenceService *getSRService()\
{\
	if( NULL != s_SRSService )\
	return s_SRSService;\
	\
	s_SRSService =\
	IPL_PTR_CAST( ISpatialReferenceService, getPlatform()->getService( IPL_SERVICE_SRS ) );\
	\
	return s_SRSService;\
}

}

