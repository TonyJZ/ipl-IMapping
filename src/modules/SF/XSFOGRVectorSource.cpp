//#include "GDAL/ogr_srs_api.h"

#include "XSFOGRVectorSource.h"
#include "XSFOGRVectorLayer.h"
#include "spatialreference/interface/ISpatialReference.h"
#include "core/interface/IPlatform.h"
#include "commonAPIs/iplutility.h"

extern ipl::IPlatform* ipl::getPlatform();

OGRSpatialReferenceH CreateOgrSpatialReference(ipl::ISpatialReference *poRefer)
{
	OGRSpatialReferenceH poOgrSRS;
	
	if( poRefer )	{


		ipl::ref_ptr<ipl::IOGRString> wkt;
		
		poRefer->exportToWkt( wkt );

		char *ogrWkt = (char *)wkt->getStr();

		//poOgrSRS->importFromWkt( &ogrWkt );

		poOgrSRS = OSRNewSpatialReference( ogrWkt );

		if( wkt->getStr() != ogrWkt )
			OGRFree( ogrWkt );

	}
	else	{
		poOgrSRS = NULL;
	}
	
	return poOgrSRS;
}

using namespace ipl;
//////////////////////////////////////////////////////////////////////////

//iml VectorReader
XSFOGRVectorSource::XSFOGRVectorSource()
{
//	m_pOgrDataSource = NULL;
}

// XSFOGRVectorSource::XSFOGRVectorSource(OGRDataSourceH DataSource)
// {
// 	m_pOgrDataSource = DataSource;
// 	initialize();
// }

XSFOGRVectorSource::~XSFOGRVectorSource()
{
//	Close();
}

iplFileFormatList XSFOGRVectorSource::GetSupportedFormats()
{
	iplFileFormat	f;
	iplFileFormatList l;

	f.name = "Shape File";	f.ext = "shp";	l.push_back( f );

	return l;
}

// void XSFOGRVectorSource::initialize()
// {
// 	if (m_pOgrDataSource == NULL)
// 		return;
// 
// 	int nLayers = OGR_DS_GetLayerCount(m_pOgrDataSource);
// 
// 	m_Layers.resize( nLayers );
// 
// 	OGRLayerH Layer;
// 
// 	for (int i = 0;i<nLayers;i++)
// 	{
// 		Layer = OGR_DS_GetLayer( m_pOgrDataSource, i );
// 
// 		m_Layers[i] = ref_ptr<XSFOGRVectorLayer>(new XSFOGRVectorLayer(Layer));
// 	}
// }

// bool XSFOGRVectorSource::Open( const char *pszName, int bUpdate)
// {
// 	OGRSFDriverH *pahDriver = NULL;
// 	
//     m_pOgrDataSource = OGROpen( pszName, bUpdate, pahDriver );
// 	
// 	if( m_pOgrDataSource == NULL )
// 		return false;
// 	
// 	initialize();
// 	
// 	m_filePath  = pszName;
// 	
// 	return true;
// }



const iplChar *XSFOGRVectorSource::getFilePath() const
{
	return m_filePath.c_str();
}


// void XSFOGRVectorSource::Close()
// {
// 	if( NULL != m_pOgrDataSource )
// 	{
// 		m_Layers.clear();
// 
// 		//	m_pOgrDataSource->ExecuteSQL("REPACK  ", null, "");
// 		// OGRDataSource::DestroyDataSource( m_pOgrDataSource );
// 		OGR_DS_Destroy( m_pOgrDataSource );
// 
// 		m_pOgrDataSource = NULL;
// 	}
// }

size_t XSFOGRVectorSource::GetLayerCount()
{
	return m_Layers.size();
}

ISFVectorLayer *XSFOGRVectorSource::GetLayer(int index )
{
	return IPL_PTR_CAST( ISFVectorLayer, m_Layers[index].get() );
}

void XSFOGRVectorSource::addLayer(ref_ptr<ISFVectorLayer> pLayer)
{
	m_Layers.push_back(pLayer);
}

ISFVectorLayer *XSFOGRVectorSource::GetLayerByName(const char *pzLayerName)
{
	if (NULL == pzLayerName)
		return NULL;

	std::vector <ref_ptr<ISFVectorLayer> >::iterator iter;
	iter = m_Layers.begin();
	do
	{
		if( strcmp(pzLayerName, iter->get()->GetLayerName()) == 0 )
			return (*iter).get();
		iter++;

	} while (iter != m_Layers.end());

	return NULL;
}

SFERR XSFOGRVectorSource::DeleteLayer(int i)
{
	SFERR Err = SFERR_NONE/*OGR_DS_DeleteLayer( m_pOgrDataSource, i )*/;

	if (Err == SFERR_NONE)
	{
		std::vector <ref_ptr<ISFVectorLayer> >::iterator iterDel;
		iterDel = m_Layers.begin();
		iterDel +=i;
		m_Layers.erase(iterDel);
	}

	return Err;
}

// ISFVectorLayer *XSFOGRVectorSource::CreateLayer( const char *pszName,
// 		IPL_wkbGeometryType eGType, char ** papszOptions )
// {
//     OGRLayerH Layer;
// 
// 	OGRSpatialReferenceH poOgrSRS = NULL;
// 
// 	if( NULL != m_pSRS.get() )	
// 		poOgrSRS = (OGRSpatialReferenceH)m_pSRS->GetOGRSpatialReference();
// 
// 	Layer = OGR_DS_CreateLayer( m_pOgrDataSource, pszName, poOgrSRS, (OGRwkbGeometryType)eGType, papszOptions);
// 
// 	if (NULL == Layer)
// 		return NULL;
// 
// 	XSFOGRVectorLayer *poLayer;
// 	poLayer = new XSFOGRVectorLayer(Layer);
// 
// 	m_Layers.push_back(ref_ptr<XSFOGRVectorLayer>(poLayer));
// 
// 	return poLayer;
// }

// int XSFOGRVectorSource::TestCapability( const char * pszCapability)
// {
// 	return OGR_DS_TestCapability( m_pOgrDataSource, pszCapability);
// }

// #include "orsBase/orsIUtilityService.h"
// 
// ORS_GET_UTILITY_SERVICE_IMPL();

// bool XSFOGRVectorSource::Create( const char *pszName, char *pszFormat, ref_ptr<ISpatialReference> &pSRS )
// {
// // 	OGRSFDriverRegistrarH poR = OGRSFDriverRegistrar
// // 
// // 	OGRSFDriverH     poDriver = NULL;
// // 	int              iDriver;
// // 	
// // 	for( iDriver = 0; iDriver < poR->GetDriverCount() && poDriver == NULL; iDriver++ )
// // 	{
// // 		if( EQUAL( poR->GetDriver(iDriver)->GetName(), pszFormat) )
// // 		{
// // 			poDriver = poR->GetDriver(iDriver);
// // 		}
// // 	}
// 
// 	OGRSFDriverH poDriver = OGRGetDriverByName( pszFormat );	
// 
// 	if( poDriver == NULL )
// 		return NULL;
// 
//     if( !OGR_Dr_TestCapability( poDriver, ODrCCreateDataSource ) )
//     {
// 		char msg[256];
//         sprintf(msg, "%s driver does not support data source creation.\n", pszFormat );
// 		
// 		getPlatform()->logPrint( IPL_LOG_ERROR, msg );
// 		
// 		return NULL;
//     }
// 	
// 	//////////////////////////////////////////////////////////////////////////
// 	// ÏÈÇå¿Õ
// 	// OGR_Dr_DeleteDataSource( poDriver, pszName );
// // 	if( getUtilityService()->CheckFolderExist( pszName ) )
// // 	{
// // 		getUtilityService()->EmptyFolder( pszName );
// // 	}
// 
// 	if (check_exist(pszName))
// 	{
// 		empty_folder(pszName);
// 	}
// 
// 	// Create the output data source.                                 
//     // options		
// 	char *papszDSCO[] = {NULL};
// 	
// 	m_pOgrDataSource = OGR_Dr_CreateDataSource( poDriver, pszName, papszDSCO );
// 	
//     if( m_pOgrDataSource == NULL )
//     {
// 		char msg[256];
//         sprintf(msg, "%s driver failed to create %s\n", pszFormat, pszName );
// 		getPlatform()->logPrint( IPL_LOG_ERROR, msg );
//         return false;
//     }
// 
// 	m_pSRS = pSRS;
// 	m_filePath  = pszName;
// 
// 	m_poDriver = poDriver;
// 
// 	return true;
// }

// int XSFOGRVectorSource::DeleteDataSource( const char *pszName )
// {
// 	if( NULL == m_poDriver )
// 		return OGRERR_FAILURE;
// 
// 	return OGR_Dr_DeleteDataSource( m_poDriver, pszName );
// }
