#include <GDAL/ogr_srs_api.h>
#include "XSFVectorSourceReader.h"
#include "spatialreference/interface/ISpatialReference.h"
#include "spatialreference/interface/ISpatialReferenceService.h"
#include "simplefeature/interface/ISFService.h"
#include "simplefeature/interface/ISFVectorLayer.h"
#include "core/interface/IPlatform.h"
#include "commonAPIs/iplutility.h"

#include "simplefeature/OGR2SF.h"

//extern ipl::IPlatform* ipl::getPlatform();
//extern ipl::ISFService *g_osfService;


using namespace ipl;

//iml VectorReader
XSFVectorSourceReader::XSFVectorSourceReader()
{
	m_pOgrDataSource = NULL;
	OGRRegisterAll();
}

// XSFVectorSourceReader::XSFVectorSourceReader(OGRDataSourceH DataSource)
// {
// 	m_pOgrDataSource = DataSource;
// 	initialize();
// }

XSFVectorSourceReader::~XSFVectorSourceReader()
{
	Close();
}

iplFileFormatList XSFVectorSourceReader::GetSupportedFormats()
{
	iplFileFormat	f;
	iplFileFormatList l;

	f.name = "Shape File";	f.ext = "shp";	l.push_back(f);

	return l;
}

const iplChar *XSFVectorSourceReader::getFilePath() const
{
	return m_filePath.c_str();
}

// int XSFVectorSourceReader::TestCapability(const char * pszCapability)
// {
// 	return OGR_DS_TestCapability(m_pOgrDataSource, pszCapability);
// }

// ��OGRFeature ���� ISFFeature
ref_ptr<ISFFeature> XSFVectorSourceReader::getFeatureFromOgr(OGRFeatureH poOgrFeature)
{
	ref_ptr<ISFFeature> poFeature = ref_ptr<ISFFeature>(getSFService()->createFeature(m_curLayerFeatDefn));

	OGRGeometryH ogrGeometry = OGR_F_GetGeometryRef(poOgrFeature);

	poFeature->SetGeometryDirectly(ref_ptr<IGeoObject>(CreateGeometry(ogrGeometry)));
	poFeature->SetID(OGR_F_GetFID(poOgrFeature));

	// 	poFeature->SetStyleString(poOgrFeature->GetStyleString());
	int nFieldCount = OGR_F_GetFieldCount(poOgrFeature);

	SFField nullF;
	nullF.Set.nMarker1 = OGRNullMarker;
	nullF.Set.nMarker2 = OGRNullMarker;
	nullF.Set.nMarker3 = OGRNullMarker;

	for (int i = 0; i < nFieldCount; i++)
	{
		if (OGR_F_IsFieldNull(poOgrFeature, i) == TRUE)
			poFeature->SetField(i, &nullF);
		else
			poFeature->SetField(i, (SFField *)OGR_F_GetRawFieldRef(poOgrFeature, i));
	}

	return poFeature;
}

// ���ÿռ�ο�
ref_ptr<ISpatialReference> XSFVectorSourceReader::getLayerSpatialRefFromOgr(OGRLayerH pOgrLayer)
{
	OGRSpatialReferenceH pOgrSR = OGR_L_GetSpatialRef(pOgrLayer);
	ref_ptr<ISpatialReference> pSRS;

	if (NULL != pOgrSR) {

		pSRS = ref_ptr<ISpatialReference>(getSRService()->CreateSpatialReference());
		
		char *wkt;

		//pOgrLayer->GetSpatialRef()->exportToWkt( &wkt );

		OSRExportToWkt(pOgrSR, &wkt);

		pSRS->importFromWkt(wkt);

		// ������delete
		CPLFree(wkt);
	}
	else {
		// ����
		//m_pSRS->importFromEPSG( 4326 );	// wgs84
	}

	return pSRS;
}

// ����Feature ���壬����osfFeature����
ref_ptr<ISFFeatureDefn> XSFVectorSourceReader::getLayerFeatureDefnFromOgr(OGRLayerH pOgrLayer)
{
	OGRFeatureDefnH ogrFeatureDefn = OGR_L_GetLayerDefn(pOgrLayer);

	int i;
	OGRFieldDefnH ogrFieldDefn;

	// ����
	ref_ptr<ISFFeatureDefn> featureDefn = ref_ptr<ISFFeatureDefn>(getSFService()->createFeatureDefn());
	featureDefn->SetName(OGR_FD_GetName(ogrFeatureDefn));

	// ����
	featureDefn->SetGeomType(static_cast<IPL_wkbGeometryType>(OGR_FD_GetGeomType(ogrFeatureDefn)));

	int nFieldCount = OGR_FD_GetFieldCount(ogrFeatureDefn);

	for (i = 0; i < nFieldCount; i++)
	{
		ogrFieldDefn = OGR_FD_GetFieldDefn(ogrFeatureDefn, i);

		ISFFieldDefn *fieldDefn = getSFService()->createFieldDefn();

		fieldDefn->Set(
			OGR_Fld_GetNameRef(ogrFieldDefn),
			(SF_FieldType)OGR_Fld_GetType(ogrFieldDefn),
			OGR_Fld_GetWidth(ogrFieldDefn),
			OGR_Fld_GetPrecision(ogrFieldDefn),
			(SF_Justification)OGR_Fld_GetJustify(ogrFieldDefn));

		featureDefn->AddFieldDefn(fieldDefn);
	}

	return featureDefn;
}


ref_ptr<ISFVectorLayer> XSFVectorSourceReader::initLayers(OGRLayerH ogrLayer)
{
// 	m_ogrLayer = NULL;
// 	m_pfilterGeom = NULL;
	//���Ի�ȡOGR�е�StyleTable
	// 	m_styleTable = NULL;

	ref_ptr<ISFVectorLayer> pLayer;

	assert(NULL != ogrLayer);

	if (NULL == ogrLayer)
		return pLayer;

// 	m_ogrLayer = ogrLayer;
// 
// 	m_IsFilter = false;
// 	m_FilterIsEnvelope = false;
// 
// 	m_FeatureCount = -1;
// 
// 	m_pfilterGeom = NULL;

	pLayer = ref_ptr<ISFVectorLayer>(getSFService()->createSFVectorLayer());

	//////////////////////////////////////////////////////////////////////////

	const char* pszName = OGR_L_GetName(ogrLayer);
	pLayer->SetLayerName(pszName);

	OGRwkbGeometryType eGType = OGR_L_GetGeomType(ogrLayer);
	pLayer->SetGeometryType(IPL_wkbGeometryType(eGType));


	//m_pSRS = GetSpatialReferenceService()->CreateSpatialReference();
	//SetSpatialRef(ogrLayer, pLayer);
	pLayer->SetSpatialReference(getLayerSpatialRefFromOgr(ogrLayer));

// 	ref_ptr<ISFFeatureDefn> featureDefn = ref_ptr<ISFFeatureDefn>(g_osfService->createFeatureDefn());
// 	SetFeatureDefn(m_ogrLayer);

	m_curLayerFeatDefn = getLayerFeatureDefnFromOgr(ogrLayer);
	pLayer->SetFeatureDefn(m_curLayerFeatDefn);

	size_t nFeatureCount;
	nFeatureCount = OGR_L_GetFeatureCount(ogrLayer, TRUE);//GetFeatureCount()����OGR���ɾ����δ�����feature

//	m_features.resize(nFeatureCount);

	OGR_L_ResetReading(ogrLayer);

	//////////////////////////////////////////////////////////////////////////
	// ����ȫ��Feature
	int numOfDeleteFeature = 0;
	for (int i = 0; i < nFeatureCount; i++)
	{
		OGRFeatureH ogrFeature;
		ogrFeature = OGR_L_GetNextFeature(ogrLayer);

		if (NULL != ogrFeature) {
			//m_features[i] = ref_ptr<ISFFeature>(CreateFeatureFromOgr(ogrFeature));
			pLayer->AppendFeature(getFeatureFromOgr(ogrFeature).get());
			OGR_F_Destroy(ogrFeature);
		}
		else
			numOfDeleteFeature++;
	}

// 	if (numOfDeleteFeature > 0)
// 		m_features.resize(m_features.size() - numOfDeleteFeature);
// 
// 	m_modified = false;

	return pLayer;
}

bool XSFVectorSourceReader::Open(const char *pszName, int bUpdate)
{
	OGRSFDriverH *pahDriver = NULL;

	m_pOgrDataSource = OGROpen(pszName, bUpdate, pahDriver);

	if (m_pOgrDataSource == NULL)
		return false;

//	initialize();

	m_filePath = pszName;

	return true;
}


void XSFVectorSourceReader::Close()
{
	if (NULL != m_pOgrDataSource)
	{
		//m_Layers.clear();
		m_pVecSource.reset();
		//	m_pOgrDataSource->ExecuteSQL("REPACK  ", null, "");
		// OGRDataSource::DestroyDataSource( m_pOgrDataSource );
		OGR_DS_Destroy(m_pOgrDataSource);

		m_pOgrDataSource = NULL;
	}
}

ref_ptr<ISFVectorSource> XSFVectorSourceReader::GetVectorSource()
{
	if (m_pOgrDataSource == NULL)
		return ref_ptr<ISFVectorSource>(NULL);

	m_pVecSource.reset();

	m_pVecSource = ref_ptr<ISFVectorSource>(getSFService()->createSFVectorSource());

	int nLayers = OGR_DS_GetLayerCount(m_pOgrDataSource);

	//m_Layers.resize(nLayers);

	OGRLayerH Layer;

	for (int i = 0; i < nLayers; i++)
	{
		Layer = OGR_DS_GetLayer(m_pOgrDataSource, i);

		//m_Layers[i] = ref_ptr<XSFOGRVectorLayer>(new XSFOGRVectorLayer(Layer));
		m_pVecSource->addLayer(initLayers(Layer));
	}

	return m_pVecSource;
}
