#pragma once

//GDAL
#include <GDAL/ogr_api.h>

#include "simplefeature/interface/ISFVectorLayer.h"

namespace ipl
{
	class XSFOGRVectorLayer : public ISFVectorLayer
	{
	public:
//		XSFOGRVectorLayer(OGRLayerH ogrLayer);
		XSFOGRVectorLayer();
		virtual ~XSFOGRVectorLayer();

	public:	// implemented interfaces
		virtual IGeoObject *GetSpatialFilter();

		virtual void SetLayerName(const char *pszName);
		virtual const char* GetLayerName() const;

		virtual void SetGeometryType(IPL_wkbGeometryType geoType);
		virtual IPL_wkbGeometryType GetGeometryType() const;

		//virtual ref_ptr<ISFFeature>& AppendFeature(IPL_wkbGeometryType type, iplPOINT3D *pts, int n);

		virtual bool AppendFeature(const ISFFeature* poFeature);

		virtual bool AppendFeatureDirectly(ISFFeature* poFeature);

		//virtual SFERR StoreFeature(long id);

		virtual void SetSpatialFilter(ref_ptr<IGeoObject> );
		virtual void SetSpatialFilterRect(double dfMinX, double dfMinY, double dfMaxX, double dfMaxY);

		virtual void        ResetReading();
		virtual ISFFeature *GetNextFeature();
		virtual SFERR      SetNextByIndex(long nIndex);
		virtual ISFFeature *GetFeature(long nFID);

// 		virtual int         TestCapability(const char *);
// 
// 		virtual int			TestDBF();

//		virtual SFERR      SetFeature(ISFFeature *poFeature);
//		virtual SFERR      CreateFeature(ISFFeature *poFeature);
		virtual SFERR      DeleteFeature(long nFID);
//		virtual SFERR		SyncToDisk()
// 		{
// 			return OGR_L_SyncToDisk(m_ogrLayer);	//-> SyncToDisk();
// 		};

		virtual size_t         GetFeatureCount(int bForce = TRUE);

		virtual SFERR      GetExtent(iplEnvelope *psExtent, int bForce = TRUE);

		//Fetch metadata from layer.
		//	virtual const char *GetInfo( const char * ) ;

//		virtual SFERR CreateField(ISFFieldDefn *poField, int bApproxOK = TRUE);

		// 	virtual osfStyleTable *GetStyleTable()
		// 	{
		// 		return m_styleTable;
		// 	}
		// 	virtual SetStyleTable(osfStyleTable *StyleTable)
		// 	{
		// 		if (m_styleTable != NULL)
		// 		{
		// 			delete m_styleTable;
		// 			m_styleTable = NULL;
		// 		}
		// 		m_styleTable = StyleTable;
		// 	}

		// 	virtual const char *GetFIDColumn() ;
		//
		// 	virtual const char *GetGeometryColumn();

		virtual void SetSpatialReference(ref_ptr<ISpatialReference> pSRS);
		virtual ref_ptr<ISpatialReference> GetSpatialReference() const
		{
			return m_pSRS;
		};

		virtual void SetFeatureDefn(ref_ptr<ISFFeatureDefn> featDefn);
		virtual ref_ptr<ISFFeatureDefn> GetFeatureDefn() const
		{
			return m_featureDefn;
		}


	private:	// import from OGR

		//ISFFeature *CreateFeatureFromOgr(OGRFeatureH poFeature);
		//OGRFeatureH CreateOgrFeature(ISFFeature *poFeature);

	private:
		bool			m_modified;

		iplString		m_LayerName;

		IPL_wkbGeometryType   m_gType;

//		OGRLayerH		m_ogrLayer;

		ref_ptr<ISpatialReference>	m_pSRS;

	protected:

		ref_ptr<ISFFeatureDefn>	m_featureDefn;

		iplArray<ref_ptr<ISFFeature> >  m_features;
		iplArray<ref_ptr<ISFFeature> >::iterator m_iter;

		//char *pszFullName;

	private:
		ref_ptr<IGeoObject> m_pfilterGeom;
		iplEnvelope m_filterEnvelope;

		bool m_FilterIsEnvelope;
		//是否使用用空间过滤
		bool m_IsFilter;
		//通过Filter后的Feature数
		int m_FeatureCount;

	private:
		// 	osfStyleTable		*m_styleTable;
		// 	OSF_RenderType m_rendType;
		// 	void SetDefRendType();

	private:
		int FilterGeometry(IGeoObject *pGeom);

	private:

// 		void SetFeatureDefn(OGRLayerH pOgrLayer);
// 		void SetSpatialRef(OGRLayerH pOgrLayer);

	public:
		// 得到detailed description
		virtual std::string	getDesc() const { return std::string("OGR vector layer"); };

	public:
		IPL_OBJECT_IMP1(XSFOGRVectorLayer, ISFVectorLayer, "OGR", "OGR VectorLayer")
	};
}
