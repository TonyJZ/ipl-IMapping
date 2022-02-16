#pragma once

#include "core/interface/IPlatform.h"
#include "spatialreference/interface/ISpatialReferenceService.h"

//////////////////////////////////////////////////////////////////////////

namespace ipl
{
	class XSpatialReferenceService : public ISpatialReferenceService
	{
	public:
		XSpatialReferenceService();
		~XSpatialReferenceService();

	public:
		//启动时被调用
		virtual bool startup(IPlatform *platform);

		//关闭时被调用
		virtual void shutdown();

		//是否启动成功
		virtual bool isok();

	public:
		// create services
		IGeoid *CreateGeoid(const char *wktGeoidName) const;
		IEllipsoid *CreateEllipsoid(const char *wktEllipsoidName = NULL) const;
		ITangentPlane *CreateTangentPlane() const;
		ISpatialReference *CreateSpatialReference(const char *wktHcs = NULL, const char *wktVcs = NULL) const;

		ICoordinateTransform *CreateCoordinateTransform() const;

		//	orsISrsDlg *CreateSrsDlg() const;

	public:
		// query services
		bool GetAvailableGeoidNames(iplArray <iplString> &geoidNames);
		bool GetAvailableEllipsoidNames(iplArray <iplString> &ellipsoidNames);

		// query services for config
		bool GetAvailableDatumNames(iplArray <iplString> &pcsNames);

		// Geographic Coordinate Systems
		bool GetAvailableGcsNames(iplArray <iplString> &pcsNames);

		// Projected Coordinate System
		bool GetAvailablePcsNames(iplArray <iplString> &pcsNames);

		// Vertical Coordinate System
		bool GetAvailableVcsNames(iplArray <iplString> &pcsNames);


		IPL_OBJECT_DEF_NORMAL(ISpatialReferenceService, "default", "Spatial Reference Service")
			IPL_BEGIN_VTABLE_MAP(XSpatialReferenceService)
			IPL_INTERFACE_ENTRY2(IObject, XSpatialReferenceService)
			IPL_INTERFACE_ENTRY(ISpatialReferenceService)
			//		IPL_INTERFACE_ENTRY(orsIPluginManager)
			//		IPL_INTERFACE_ENTRY(orsIService)
			IPL_END_VTABLE_MAP

			// Object方法实现
			//	IPL_OBJECT_IMP2( XSpatialReferenceService,
			//		ISpatialReferenceService, orsIService, "default", "Spatial Reference Service")
	};

}
