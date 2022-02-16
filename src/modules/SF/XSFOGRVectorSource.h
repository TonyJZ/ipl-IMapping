#pragma once

//#include "ogrsf_frmts.h"
#include <GDAL/ogr_api.h>
#include <GDAL/ogr_geometry.h>
#include <GDAL/ogr_feature.h>

//#include "orsSF/osfIFrmts.h"

//#include "orsSF/osfIFeatureDefn.h"
#include "core/ipldef.h"
#include "simplefeature/interface/ISFVectorSource.h"

namespace ipl
{
	interface ISpatialReference;
//	class XSFOGRVectorLayer;

	class XSFOGRVectorSource : public ISFVectorSource
	{
	private:
		iplArray <ref_ptr<ISFVectorLayer> > m_Layers;

//		ref_ptr<ISpatialReference>	m_pSRS;

		iplString				m_filePath;

// 		OGRSFDriverH		m_poDriver;
// 		OGRDataSourceH		m_pOgrDataSource;

	private:
//		void initialize();


	public:
		XSFOGRVectorSource();
//		XSFOGRVectorSource(OGRDataSourceH DataSource);
		virtual ~XSFOGRVectorSource();

		// 得到detailed description
		virtual iplString	getDesc() const { return "OGR Vector Data Source"; };
		virtual const iplChar *getFilePath() const;

		//virtual const iplDSType getType() const { return IPL_DS_SIMPLEFEATURE; };

		virtual iplFileFormatList GetSupportedFormats();

	public:
//		virtual bool Open(const char *pszName, int bUpdate);
		//virtual bool Create(const char *pszDestDataSource, const char *pszFormat );

//		virtual bool IsOpen() const { return m_pOgrDataSource != NULL; }
//		virtual void Close();

//		virtual bool Create(const char *pszName, char *pszFormat, ref_ptr<ISpatialReference> &poSpatialRef);
//		virtual int DeleteDataSource(const char *pszName);

//		virtual int TestCapability(const char *);

	public:
		virtual size_t			GetLayerCount();
		virtual ISFVectorLayer    *GetLayer(int);
		virtual void addLayer(ref_ptr<ISFVectorLayer> pLayer);

		virtual ISFVectorLayer *GetLayerByName(const char *);

		virtual SFERR DeleteLayer(int);

		// 自己创建时，每个dataSource 采用同一个 ISpatialReference ?
// 		virtual ISFVectorLayer *CreateLayer(const char *pszName,
// 			IPL_wkbGeometryType eGType, char ** papszOptions = NULL);//,ISpatialReference *poSpatialRef = NULL );

	public:
		IPL_OBJECT_IMP3(XSFOGRVectorSource, ISFVectorSource, IVectorSource, IDataSource, "OGR", "OGR VectorSource")
	};
}


