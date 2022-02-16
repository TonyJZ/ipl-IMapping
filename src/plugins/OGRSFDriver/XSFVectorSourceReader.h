#pragma once
#include <GDAL/ogr_api.h>
#include <GDAL/ogr_geometry.h>
#include <GDAL/ogr_feature.h>

#include "simplefeature/interface/ISFVectorSourceReader.h"
#include "spatialreference/interface/ISpatialReference.h"
#include "simplefeature/interface/ISFFeature.h"
#include "simplefeature/interface/ISFFeatureDefn.h"

namespace ipl
{
	class XSFVectorSourceReader : public ISFVectorSourceReader
	{
	private:
		//iplArray <ref_ptr<XSFOGRVectorLayer> > m_Layers;

		ref_ptr<ISFVectorSource>   m_pVecSource;
		ref_ptr<ISpatialReference>	m_pSRS;

		ref_ptr<ISFFeatureDefn>		m_curLayerFeatDefn;

		iplString				m_filePath;

		OGRSFDriverH		m_poDriver;
		OGRDataSourceH		m_pOgrDataSource;

	protected:
		ref_ptr<ISFVectorLayer> initLayers(OGRLayerH ogrLayer);
		ref_ptr<ISFFeature> getFeatureFromOgr(OGRFeatureH poFeature);
		ref_ptr<ISpatialReference> getLayerSpatialRefFromOgr(OGRLayerH pOgrLayer);
		ref_ptr<ISFFeatureDefn> getLayerFeatureDefnFromOgr(OGRLayerH pOgrLayer);

	public:
		XSFVectorSourceReader();
		//XSFVectorSourceReader(OGRDataSourceH DataSource);
		virtual ~XSFVectorSourceReader();

		// 得到detailed description
		virtual iplString	getDesc() const { return "OGR Vector Data Source Reader"; };
		

		virtual iplFileFormatList GetSupportedFormats();
		virtual const iplChar *getFilePath() const;

	public:
		virtual bool Open(const char *pszName, int bUpdate);
		//virtual bool Create(const char *pszDestDataSource, const char *pszFormat );

		virtual bool IsOpen() const { return m_pOgrDataSource != NULL; }
		virtual void Close();

// 		virtual int TestCapability(const char *);
// 		virtual int	TestDBF();

	public:
// 		virtual size_t			GetLayerCount();
// 		virtual ISFVectorLayer    *GetLayer(int);

		virtual ref_ptr<ISFVectorSource> GetVectorSource();

// 		virtual SFERR DeleteLayer(int);
// 
// 		// 自己创建时，每个dataSource 采用同一个 ISpatialReference ?
// 		virtual ISFVectorLayer *CreateLayer(const char *pszName,
// 			IPL_wkbGeometryType eGType, char ** papszOptions = NULL);//,ISpatialReference *poSpatialRef = NULL );

	public:
		IPL_OBJECT_IMP2(XSFVectorSourceReader, ISFVectorSourceReader, IVectorSourceReader, "OGR", "OGR VectorSourceReader")
	};
}

