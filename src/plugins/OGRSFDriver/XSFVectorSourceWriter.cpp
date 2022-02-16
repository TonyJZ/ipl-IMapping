#include <GDAL/ogr_srs_api.h>
#include "XSFVectorSourceWriter.h"
#include "spatialreference/interface/ISpatialReference.h"
#include "spatialreference/interface/ISpatialReferenceService.h"
#include "simplefeature/interface/ISFService.h"
#include "simplefeature/interface/ISFVectorLayer.h"
#include "core/interface/IPlatform.h"
#include "commonAPIs/iplutility.h"

#include "simplefeature/SF2OGR.h"

//extern ipl::IPlatform* ipl::getPlatform();
//extern ipl::ISFService *g_osfService;


// OGRSpatialReferenceH CreateOgrSpatialReference(ipl::ISpatialReference *poRefer)
// {
// 	OGRSpatialReferenceH poOgrSRS;
// 
// 	if (poRefer) {
// 
// 
// 		ipl::ref_ptr<ipl::IOGRString> wkt;
// 
// 		poRefer->exportToWkt(wkt);
// 
// 		char *ogrWkt = (char *)wkt->getStr();
// 
// 		//poOgrSRS->importFromWkt( &ogrWkt );
// 
// 		poOgrSRS = OSRNewSpatialReference(ogrWkt);
// 
// 		if (wkt->getStr() != ogrWkt)
// 			OGRFree(ogrWkt);
// 
// 	}
// 	else {
// 		poOgrSRS = NULL;
// 	}
// 
// 	return poOgrSRS;
// }

using namespace ipl;

XSFVectorSourceWriter::XSFVectorSourceWriter()
{
	m_pOgrDataSource = NULL;
	OGRRegisterAll();
}

XSFVectorSourceWriter::~XSFVectorSourceWriter()
{
	Close();
}

iplFileFormatList XSFVectorSourceWriter::GetSupportedFormats()
{
	iplFileFormat	f;
	iplFileFormatList l;

	f.name = "Shape File";	f.ext = "shp";	l.push_back(f);

	return l;
}

const iplChar *XSFVectorSourceWriter::getFilePath() const
{
	return m_filePath.c_str();
}

void XSFVectorSourceWriter::Close()
{
	if (NULL != m_pOgrDataSource)
	{
		m_pVecSource.reset();

		//	m_pOgrDataSource->ExecuteSQL("REPACK  ", null, "");
		// OGRDataSource::DestroyDataSource( m_pOgrDataSource );
		OGR_DS_Destroy(m_pOgrDataSource);

		m_pOgrDataSource = NULL;
	}
}

bool XSFVectorSourceWriter::Create(const char *pszName, const char *pszFormat/*, ref_ptr<ISpatialReference> pSRS*/)
{
	// 	OGRSFDriverRegistrarH poR = OGRSFDriverRegistrar
	// 
	// 	OGRSFDriverH     poDriver = NULL;
	// 	int              iDriver;
	// 	
	// 	for( iDriver = 0; iDriver < poR->GetDriverCount() && poDriver == NULL; iDriver++ )
	// 	{
	// 		if( EQUAL( poR->GetDriver(iDriver)->GetName(), pszFormat) )
	// 		{
	// 			poDriver = poR->GetDriver(iDriver);
	// 		}
	// 	}

	OGRSFDriverH poDriver = OGRGetDriverByName(pszFormat);

	if (poDriver == NULL)
		return false;

	if (!OGR_Dr_TestCapability(poDriver, ODrCCreateDataSource))
	{
		char msg[256];
		sprintf(msg, "%s driver does not support data source creation.\n", pszFormat);

		getPlatform()->logPrint(IPL_LOG_ERROR, msg);

		return false;
	}

	//////////////////////////////////////////////////////////////////////////
	// ÏÈÇå¿Õ
	// OGR_Dr_DeleteDataSource( poDriver, pszName );
	// 	if( getUtilityService()->CheckFolderExist( pszName ) )
	// 	{
	// 		getUtilityService()->EmptyFolder( pszName );
	// 	}

	if (check_exist(pszName))
	{
		empty_folder(pszName);
	}

	// Create the output data source.                                 
	// options		
	char *papszDSCO[] = { NULL };

	m_pOgrDataSource = OGR_Dr_CreateDataSource(poDriver, pszName, papszDSCO);

	if (m_pOgrDataSource == NULL)
	{
		char msg[256];
		sprintf(msg, "%s driver failed to create %s\n", pszFormat, pszName);
		getPlatform()->logPrint(IPL_LOG_ERROR, msg);
		return false;
	}

	//m_pSRS = pSRS;
	m_filePath = pszName;

	m_poDriver = poDriver;

	return true;
}

OGRFeatureDefnH XSFVectorSourceWriter::getLayerFeatureDefn(ISFVectorLayer *sflayer)
{
	ref_ptr<ISFFeatureDefn> fDefn = sflayer->GetFeatureDefn();

	const char *pszName = fDefn->GetName();

	OGRFeatureDefnH ogrFeatureDefn = OGR_FD_Create(pszName);

	size_t nField = fDefn->GetFieldCount();
	for (int i = 0; i < nField; ++i)
	{
		ISFFieldDefn *fieldDefn = fDefn->GetFieldDefn(i);
		const char* pfiledName = fieldDefn->GetNameRef();
		SF_FieldType type = fieldDefn->GetType();
		int wid = fieldDefn->GetWidth();
		int precision = fieldDefn->GetPrecision();
		SF_Justification just = fieldDefn->GetJustify();

		OGRFieldDefnH ogrFieldDefn = OGR_Fld_Create(pfiledName, OGRFieldType(type));
		OGR_Fld_Set(ogrFieldDefn, pfiledName, OGRFieldType(type), wid, precision, OGRJustification(just));
		
		OGR_FD_AddFieldDefn(ogrFeatureDefn, ogrFieldDefn);
	}

	return ogrFeatureDefn;
}

void XSFVectorSourceWriter::setFieldValue(SFField  *value, int i, OGRFeatureH &poFeature)
{
	if(OGR_RawField_IsNull((OGRField*)value) == TRUE)
		OGR_F_SetFieldRaw(poFeature, i, NULL);
	else
		OGR_F_SetFieldRaw(poFeature, i, (OGRField*)value);

	return;
}

void XSFVectorSourceWriter::createLayerField(OGRLayerH ogrLayer, OGRFieldDefnH fDefn, int bApproxOK)
{
	OGRErr err = OGR_L_CreateField(ogrLayer, fDefn, bApproxOK);
	return;
}


bool XSFVectorSourceWriter::SetVectorSource(ref_ptr<ISFVectorSource> pSource)
{
	if (m_pOgrDataSource == NULL)
		return false;

	m_pVecSource.reset();
	m_pVecSource = pSource;

	size_t nLayers = m_pVecSource->GetLayerCount();

	for (int iLayer = 0; iLayer < nLayers; ++iLayer)
	{
		OGRLayerH ogrLayer;

		OGRSpatialReferenceH poOgrSRS = NULL;

		ISFVectorLayer *sfLayer = m_pVecSource->GetLayer(iLayer);

		if (NULL != sfLayer->GetSpatialReference().get())
			poOgrSRS = (OGRSpatialReferenceH)(sfLayer->GetSpatialReference()->GetOGRSpatialReference());

		const char* pszName = sfLayer->GetLayerName();
		IPL_wkbGeometryType eGType = sfLayer->GetGeometryType();

		ogrLayer = OGR_DS_CreateLayer(m_pOgrDataSource, pszName, poOgrSRS, (OGRwkbGeometryType)eGType, NULL);

		if (NULL == ogrLayer)
			return false;

		OGRFeatureDefnH ogrFeatureDefn = getLayerFeatureDefn(sfLayer);

		int fcount = OGR_FD_GetFieldCount(ogrFeatureDefn);
		for (int i = 0; i < fcount; ++i)
		{
			OGRFieldDefnH fieldDefn = OGR_FD_GetFieldDefn(ogrFeatureDefn, i);
			createLayerField(ogrLayer, fieldDefn, TRUE);
		}
		
// 		OGR_L_CreateField(OGRLayerH, OGRFieldDefnH, int)
// 
// 		OGR_L_CreateField(ogrLayer, ogrFeatureDefn, FALSE); //Create a new field on a layer

		sfLayer->ResetReading();
		ISFFeature* feature = NULL;
		while (NULL != (feature = sfLayer->GetNextFeature()))
		{
			OGRFeatureH poFeature = OGR_F_Create(ogrFeatureDefn);

			//////att
			size_t nField = feature->GetFieldCount();
			for (int iField = 0; iField < nField; ++iField)
			{
				SFField  *sffield = feature->GetRawFieldRef(iField);

				setFieldValue(sffield, iField, poFeature);
			}
			
			//////geometry
			OGRGeometryH poOGRGeometry = NULL;

			IGeoObject *pGeo = feature->GetGeometryRef();

			poOGRGeometry = CreateOgrGeom(pGeo);

			OGR_F_SetGeometryDirectly(poFeature, poOGRGeometry);

			//OGR_L_SetFeature(ogrLayer, poFeature); //Rewrite an existing feature
			SFERR Err = OGR_L_CreateFeature(ogrLayer, poFeature); //Create and write a new feature within a layer
		
			//OGR_F_Destroy(poFeature);
		}

	}

	return true;
}

