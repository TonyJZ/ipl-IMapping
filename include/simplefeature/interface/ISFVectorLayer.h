#pragma once

#include "geometry/interface/IGeoObject.h"
#include "spatialreference/interface/ISpatialReference.h"
#include "simplefeature/interface/ISFFeature.h"
#include "simplefeature/SFStyleTable.h"

namespace ipl
{
	/************************************************************************/
	/*                               ISFVectorLayer                         */
	/************************************************************************/

	/**
	* This class represents a layer of simple features, with access methods.
	*/
	//ʡ��ILayer�㣬Ŀǰ��ʸ����ͼ������������Դ��û�б�Ҫ��
	interface ISFVectorLayer : public IObject   
	{
	public:
		virtual IGeoObject *GetSpatialFilter() = 0;
		virtual void SetSpatialFilter(ref_ptr<IGeoObject> ) = 0;
		virtual void SetSpatialFilterRect(double dfMinX, double dfMinY, double dfMaxX, double dfMaxY) = 0;

		virtual void SetLayerName(const char *pszName) = 0;
		virtual const char* GetLayerName() const = 0;

		virtual void SetGeometryType(IPL_wkbGeometryType geoType) = 0;
		virtual IPL_wkbGeometryType GetGeometryType() const = 0;

		//virtual ref_ptr<ISFFeature>& AppendFeature(IPL_wkbGeometryType type, iplPOINT3D *pts, int n) = 0;

		//�ⲿ����poFeature
		virtual bool AppendFeature(const ISFFeature* poFeature) = 0;
		//�ⲿ������poFeature
		virtual bool AppendFeatureDirectly(ISFFeature* poFeature) = 0;

		//virtual SFERR      SetFeature(ISFFeature *poFeature) = 0;
		//virtual SFERR      CreateFeature(ISFFeature *poFeature) = 0;

		//virtual SFERR StoreFeature(long id) = 0;

		virtual void        ResetReading() = 0;
		virtual ISFFeature* GetNextFeature() = 0;
		virtual SFERR      SetNextByIndex(long nIndex) = 0;
		virtual ISFFeature* GetFeature(long nFID) = 0;

		
		virtual SFERR      DeleteFeature(long nFID) = 0;
		//virtual SFERR		SyncToDisk() = 0;

		//virtual ISFFeatureDefn *GetLayerDefn() = 0;

		virtual size_t         GetFeatureCount(int bForce = true) = 0;
		virtual SFERR      GetExtent(iplEnvelope *psExtent, int bForce = TRUE) = 0;

		//virtual int         TestCapability(const char *) = 0;
		// 	virtual const char *GetInfo( const char * ) = 0; //Fetch metadata from layer.

// 		virtual SFERR      CreateField(ISFFieldDefn *poField,
// 			int bApproxOK) = 0;

		// 	virtual osfStyleTable *GetStyleTable() = 0;
		// 	virtual SetStyleTable(osfStyleTable *) = 0;
		// 	virtual OSF_RenderType GetRenderType() = 0;
		// 	virtual void SetRenderType(OSF_RenderType rendtype) = 0;

		// 	virtual const char *GetFIDColumn() = 0;
		// 	virtual const char *GetGeometryColumn()= 0;

		/************************************************************************/
		/* ���½�ͼ�������˳�δ����ʱDBF�ļ�Ϊ��,�ٴδ򿪲�����ʱ�ͻᷢ���쳣
		������ΪOGRVC161D�⺯�� OGRErr SHPWriteOGRFeature( SHPHandle hSHP, DBFHandle hDBF,OGRFeatureDefn *poFeatureDefn,
		OGRFeature *poFeature )δ�ж�hDBF�Ƿ�Ϊ�յ������
		������ͨ�������½�һ���ֶ��Ƿ�ɹ����ж�DBF�ļ��Ƿ���ȷ
		*/
		/************************************************************************/
//		virtual int			TestDBF() = 0;

		virtual void SetSpatialReference(ref_ptr<ISpatialReference> pSRS) = 0;
		virtual ref_ptr<ISpatialReference> GetSpatialReference() const = 0;

		virtual void SetFeatureDefn(ref_ptr<ISFFeatureDefn> featDefn) = 0;
		virtual ref_ptr<ISFFeatureDefn> GetFeatureDefn() const = 0;

	public:
		IPL_INTERFACE_DEF(IObject, "SFVectorLayer");
	};

#define IPL_VECTORLAYER_SF_DEFAULT		"ipl.SFVectorLayer.OGR"
}
