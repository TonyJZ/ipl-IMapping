#include "XSFOGRVectorLayer.h"
#include "simplefeature/interface/ISFService.h"

#include "XSFFieldDefn.h"


// #include "OGR2SF.h"
// #include "SF2OGR.h"

#include "GDAL/cpl_conv.h"
#include "GDAL/ogr_srs_api.h"

//////////////////////////////////////////////////////////////////////////
extern ipl::ISFService *g_osfService;

using namespace ipl;

//////////////////////////////////////////////////////////////////////////
XSFOGRVectorLayer::XSFOGRVectorLayer()
{
//	m_ogrLayer = NULL;
	m_pfilterGeom = NULL;

	m_IsFilter = false;
	m_FilterIsEnvelope = false;

	m_FeatureCount = -1;

	m_pfilterGeom = NULL;

	m_LayerName = "";
	m_gType = IPL_wkbUnknown;
}

// XSFOGRVectorLayer::XSFOGRVectorLayer(OGRLayerH ogrLayer)
// {	
// 	m_ogrLayer = NULL; 
// 	m_pfilterGeom = NULL;
// 	//可以获取OGR中的StyleTable
// 	// 	m_styleTable = NULL;
// 	
// 	assert( NULL != ogrLayer );
// 	
// 	if( NULL == ogrLayer )
// 		return ;
// 	
// 	m_ogrLayer = ogrLayer;
// 	
// 	m_IsFilter = false;
// 	m_FilterIsEnvelope = false;
// 	
// 	m_FeatureCount = -1;
// 	
// 	m_pfilterGeom = NULL;
// 
// 	//////////////////////////////////////////////////////////////////////////
// 
// 	//m_pSRS = GetSpatialReferenceService()->CreateSpatialReference();
// 	SetSpatialRef(m_ogrLayer);
// 	
// 	m_featureDefn = ref_ptr<ISFFeatureDefn>(g_osfService->createFeatureDefn());
// 	SetFeatureDefn( m_ogrLayer );
// 	
// 	int nFeatureCount;
// 	nFeatureCount = OGR_L_GetFeatureCount( m_ogrLayer, TRUE );
// 
//     m_features.resize( nFeatureCount );//GetFeatureCount()包含OGR标记删除但未清理的feature
// 	
// 	OGR_L_ResetReading( m_ogrLayer );
// 	
// 	//////////////////////////////////////////////////////////////////////////
// 	// 缓冲全部Feature
// 	int numOfDeleteFeature = 0;
// 	for( int i = 0;i< m_features.size(); i++)
// 	{
// 		OGRFeatureH ogrFeature;
// 		ogrFeature = OGR_L_GetNextFeature( m_ogrLayer );
// 		
// 		if( NULL != ogrFeature )	{
// 			m_features[i] = ref_ptr<ISFFeature>(CreateFeatureFromOgr( ogrFeature ));
// 			
// 			OGR_F_Destroy( ogrFeature );
// 		}
// 		else
// 			numOfDeleteFeature++;
// 	}
// 
// 	if(numOfDeleteFeature > 0)
// 		m_features.resize(m_features.size() - numOfDeleteFeature);
// 
// 	m_modified = false;
// }


XSFOGRVectorLayer::~XSFOGRVectorLayer()
{

}

void XSFOGRVectorLayer::SetLayerName(const char *pszName)
{
	m_LayerName = pszName;
}

const char* XSFOGRVectorLayer::GetLayerName() const
{
	return m_LayerName.c_str();
}

void XSFOGRVectorLayer::SetGeometryType(IPL_wkbGeometryType geoType)
{
	m_gType = geoType;
}

IPL_wkbGeometryType XSFOGRVectorLayer::GetGeometryType() const
{
	return m_gType;
}

// 从OGRFeature 创建 ISFFeature
// ISFFeature *XSFOGRVectorLayer::CreateFeatureFromOgr( OGRFeatureH poOgrFeature )
// {
// 	ISFFeature *poFeature = g_osfService->createFeature( GetLayerDefn() );
// 	
// 	OGRGeometryH ogrGeometry = OGR_F_GetGeometryRef( poOgrFeature );
// 	
// 	poFeature->SetGeometryDirectly( CreateGeometry( ogrGeometry ) );
// 	poFeature->SetID( OGR_F_GetFID( poOgrFeature ) );
// 	
// 	// 	poFeature->SetStyleString(poOgrFeature->GetStyleString());
// 	int nFieldCount = OGR_F_GetFieldCount( poOgrFeature );
// 	
// 	for( int i = 0; i< nFieldCount; i++)
// 	{
// 		poFeature->SetField( i, (SFField *)OGR_F_GetRawFieldRef( poOgrFeature, i) );
// 	}
// 
// 	return poFeature;
// 	
// }



// 设置空间参考
// void XSFOGRVectorLayer::SetSpatialRef(ref_ptr<ISpatialReference> pSRS)
// {
// 	OGRSpatialReferenceH pOgrSR = OGR_L_GetSpatialRef( pOgrLayer );
// 
// 	if( NULL != pOgrSR )	{
// 		if( NULL == m_pSRS.get() )
// 			m_pSRS = ref_ptr<ISpatialReference>(getSRService()->CreateSpatialReference());
// 		
// 		char *wkt;
// 
// 		//pOgrLayer->GetSpatialRef()->exportToWkt( &wkt );
// 		
// 		OSRExportToWkt( pOgrSR, &wkt );
// 		
// 		m_pSRS->importFromWkt( wkt );
// 		
// 		// 不能用delete
// 		CPLFree( wkt );
// 	}
// 	else	{	
// 		// 不变
// 		//m_pSRS->importFromEPSG( 4326 );	// wgs84
// 	}
// }


// 设置Feature 定义，建立osfFeature定义
void XSFOGRVectorLayer::SetFeatureDefn(ref_ptr<ISFFeatureDefn> featDefn)
{
	m_featureDefn.reset();
	m_featureDefn = featDefn;

// 	OGRFeatureDefnH ogrFeatureDefn = OGR_L_GetLayerDefn( pOgrLayer );
// 	
// 	int i;
// 	OGRFieldDefnH ogrFieldDefn;
// 	
// 	// 名称
// 	m_featureDefn->SetName( OGR_FD_GetName( ogrFeatureDefn ) );
// 
// 	// 类型
// 	m_featureDefn->SetGeomType( (IPL_wkbGeometryType) OGR_FD_GetGeomType( ogrFeatureDefn ) );
// 
// 	int nFieldCount = OGR_FD_GetFieldCount( ogrFeatureDefn );
// 
// 	for( i=0; i<nFieldCount ; i++)
// 	{
// 		ogrFieldDefn = OGR_FD_GetFieldDefn( ogrFeatureDefn, i );
// 		
// 		ISFFieldDefn *fieldDefn = g_osfService->createFieldDefn();
// 		
// 		fieldDefn->Set( 
// 			OGR_Fld_GetNameRef( ogrFieldDefn ), 
// 			(SF_FieldType)OGR_Fld_GetType( ogrFieldDefn ), 
// 			OGR_Fld_GetWidth(ogrFieldDefn), 
// 			OGR_Fld_GetPrecision(ogrFieldDefn), 
// 			(SF_Justification)OGR_Fld_GetJustify(ogrFieldDefn) );
// 		
// 		m_featureDefn->AddFieldDefn( fieldDefn );
// 	}
}



//////////////////////////////////////////////////////////////////////////

IGeoObject *XSFOGRVectorLayer::GetSpatialFilter () 
{
	return m_pfilterGeom.get();
}
	
void XSFOGRVectorLayer::SetSpatialFilter(ref_ptr<IGeoObject> pGeoIN) 
{
	if ((m_pfilterGeom.get()) !=NULL)
		m_pfilterGeom = NULL;
	
	if (pGeoIN == NULL)
	{
		m_IsFilter = false;
		m_FeatureCount = -1;
	}
	else{
		pGeoIN->getEnvelope(&m_filterEnvelope);
		m_pfilterGeom = pGeoIN;
		m_FeatureCount = -1;
		m_FilterIsEnvelope = false;
		m_IsFilter = true;
	}	
}

void XSFOGRVectorLayer::SetSpatialFilterRect(double dfMinX, double dfMinY, double dfMaxX, double dfMaxY)
{
	m_filterEnvelope.maxX = dfMaxX;
	m_filterEnvelope.maxY = dfMaxY;
	m_filterEnvelope.minX = dfMinX;
	m_filterEnvelope.minY = dfMinY;

	ref_ptr<ISFPolygon> pPolygon = ref_ptr<ISFPolygon>(g_osfService->createPolygon());
	ref_ptr<ISFLineRing> pLinearRing = ref_ptr<ISFLineRing>(g_osfService->createLinearRing());

	pLinearRing->setNumPoints(4);
	iplPOINT3D point;
	point.X = dfMaxX;point.Y = dfMaxY;point.Z = 0.0;
	pLinearRing->setPoint(0,point);
	point.X = dfMaxX;point.Y = dfMinY;point.Z = 0.0;
	pLinearRing->setPoint(1,point);
	point.X = dfMinX;point.Y = dfMinY;point.Z = 0.0;
	pLinearRing->setPoint(2,point);
	point.X = dfMinX;point.Y = dfMaxY;point.Z = 0.0;
	pLinearRing->setPoint(3,point);
	point.X = dfMaxX;point.Y = dfMaxY;point.Z = 0.0;
	pLinearRing->setPoint(4,point);

	pPolygon->addRing(pLinearRing);

	m_pfilterGeom = pPolygon;
	m_FeatureCount = -1;
	m_FilterIsEnvelope = true;  
	m_IsFilter = true;
}

int XSFOGRVectorLayer::FilterGeometry(IGeoObject *pGeom)
{
	if(pGeom == NULL||m_pfilterGeom.get() == NULL)
		return true;
	
	iplEnvelope sGeomEnv;
	pGeom->getEnvelope(&sGeomEnv);
		
	if( sGeomEnv.maxX < m_filterEnvelope.minX
        || sGeomEnv.maxY < m_filterEnvelope.minY
        || m_filterEnvelope.maxX < sGeomEnv.minX
        || m_filterEnvelope.maxY < sGeomEnv.minY )
	{
		return false;
	}
	else
	{
		if (m_FilterIsEnvelope)
			return true;
		else
			return g_osfService->Intersects(m_pfilterGeom.get(), pGeom);
			//return m_pfilterGeom.get()->Intersects( pGeom );	
	}
}

void XSFOGRVectorLayer::ResetReading()
{
	m_iter = m_features.begin();
}

ISFFeature *XSFOGRVectorLayer::GetNextFeature() 
{
	if (!m_IsFilter)
	{
		if (m_iter != m_features.end())
			return (*m_iter++).get();
		return NULL;
	}
	
	while (m_iter != m_features.end())
	{
		IGeoObject *pGeom = (*m_iter).get()->GetGeometryRef();
		if ( FilterGeometry( pGeom ))
			return (*m_iter++).get();

		*m_iter++;
	}
// 	do 
// 	{
// 		//OGR中删除元素依然存在但指针为空
// 		if (m_iter != m_features.end() && (*m_iter).get())
// 		{
// 			IGeoObject *pGeom = (*m_iter).get()->GetGeometryRef();
// 			if ( FilterGeometry( pGeom ))
// 				return (*m_iter++).get();	
// 			*m_iter++;
// 		}
// 		else
// 			continue;
// 		
// 	} while (1);
	
	return NULL;
}

SFERR XSFOGRVectorLayer::SetNextByIndex( long nIndex )
{
	if(m_FeatureCount == -1)
	{
		if (nIndex<0||nIndex>m_features.size()-1)
			return SFERR_FAILURE;
		m_iter = m_features.begin();
		m_iter +=nIndex;
		
		return SFERR_NONE;
	}

	return SFERR_FAILURE;
}

ISFFeature *XSFOGRVectorLayer::GetFeature( long nFID ) 
{
	std::vector<ref_ptr<ISFFeature> >::iterator iterSet;
	iterSet = m_features.begin();

	while(iterSet != m_features.end())
	{
		if ((*iterSet).get()->GetID() == nFID)
		{
			return (*iterSet).get();
		}
		iterSet++;

	} 

	return NULL;
}


//////////////////////////////////////////////////////////////////////////



// 设置orsIFeature
// SFERR XSFOGRVectorLayer::SetFeature( ISFFeature *poFeature ) 
// {
// 	OGRFeatureH poOgrFeature = CreateOgrFeature( poFeature );
// 
// 	//SFERR Err = m_ogrLayer->SetFeature( poOgrFeature );
// 
// 	SFERR Err = OGR_L_SetFeature( m_ogrLayer, poOgrFeature );
// 	
// 	OGR_F_Destroy( poOgrFeature );
// 
// 	if( Err == SFERR_NONE)
// 	{
// 		std::vector<ref_ptr<ISFFeature> >::iterator iterSet;
// 		iterSet = m_features.begin();
// 		
// 		do 
// 		{
// 			if ((*iterSet).get()->GetID() == poFeature->GetID())
// 			{
// 				*iterSet = ref_ptr<ISFFeature>(poFeature);
// 				break;
// 			}
// 			iterSet++;
// 			
// 		} while (iterSet != m_features.end());
// 	}
// 		
// 	return Err;
// }



// 从 ISFFeature  建立 OGR Feature
// OGRFeatureH XSFOGRVectorLayer::CreateOgrFeature(ISFFeature *poFeature)
// {
// 	if (NULL == poFeature)
// 		return NULL;
// 	
// 	OGRFeatureH poOgrFeature;
// 	ISFFeatureDefn *poDefn;
// 
// 	OGRFeatureDefnH poOgrFeatureDefn;
// 	
// 	poDefn = poFeature->GetDefnRef();
// 
// 	poOgrFeatureDefn = CreateOgrFeatureDefn(poDefn);
// 
// 	//poOgrFeatureDefn为CreateFeature中的引用，不需要删除
// 	// OGRFeature::CreateFeature( poOgrFeatureDefn );
// 	poOgrFeature = OGR_F_Create( poOgrFeatureDefn );
// 	
// 	if (NULL == poOgrFeatureDefn)
// 		return NULL;
// 	
// 	OGRGeometryH Geom = CreateOgrGeom(poFeature->GetGeometryRef());
// 	// poOgrFeature->SetGeometryDirectly(Geom);
// 
// 	OGR_F_SetGeometryDirectly( poOgrFeature, Geom );
// 	//poOgrFeature->SetGeometry(Geom);
// 	//delete Geom;
// 	
// 	OGR_F_SetFID( poOgrFeature, poFeature->GetID() );
// 	
// 	for (int i = 0;i<poFeature->GetFieldCount();i++)
// 	{
// 		//poOgrFeature->SetField(   i, (OGRField *)poFeature->GetRawFieldRef(i));
// 
// 		OGR_F_SetFieldRaw( poOgrFeature, i, (OGRField *)poFeature->GetRawFieldRef(i) );
// 
// 	}
// 	
// 	return poOgrFeature;
// }

//////////////////////////////////////////////////////////////////////////
// 从外部 Feature 创建内部拷贝

// SFERR XSFOGRVectorLayer::CreateFeature( ISFFeature *poFeature )
// {
// 	try
// 	{
// 		OGRFeatureH poOgrFeature;
// 		poOgrFeature = CreateOgrFeature(poFeature);
// 		
// 		//SFERR Err = m_ogrLayer->CreateFeature(poOgrFeature);
// 		SFERR Err = OGR_L_CreateFeature( m_ogrLayer, poOgrFeature);
// 		
// 		if (Err == OGRERR_NONE)
// 		{
// 			ref_ptr<ISFFeature> feature = ref_ptr<ISFFeature>(CreateFeatureFromOgr(poOgrFeature));
// 			m_features.push_back(feature);
// 		}
// 		
// 		OGR_F_Destroy( poOgrFeature );
// 		return Err;
// 	}
// 	catch (...)
// 	{
// 		const char *msg = CPLGetLastErrorMsg();
// 		if(msg != NULL)
// 			getPlatform()->logPrint(IPL_LOG_ERROR,"%s",msg);
// 		return OGRERR_FAILURE;
// 	}
// 
// }


//////////////////////////////////////////////////////////////////////////
// 根据ID删除 Feature
SFERR XSFOGRVectorLayer::DeleteFeature( long nFID )
{
	SFERR Err;
	
	if (nFID < 0)//缓存中的临时要素
		Err = SFERR_NONE;
// 	else
// 		// m_ogrLayer->DeleteFeature(nFID);
// 		Err = OGR_L_DeleteFeature( m_ogrLayer, nFID );

	if (SFERR_NONE == Err)
	{
		std::vector<ref_ptr<ISFFeature> >::iterator iterSet;
		iterSet = m_features.begin();
		
		do 
		{
			if ((*iterSet).get()->GetID() == nFID)
			{
				m_features.erase(iterSet);
				break;
			}
			iterSet++;
			
		} while (iterSet != m_features.end());
	}
	
	return Err;
}

//////////////////////////////////////////////////////////////////////////
// 取过滤范围内的Feature个数
size_t XSFOGRVectorLayer::GetFeatureCount(int bForce)
{
	if (m_IsFilter)
	{
		if (m_FeatureCount != -1)
			return m_FeatureCount;
		else
		{
			m_FeatureCount = 0;
			ResetReading();

			ISFFeature *poFeature;
			while( (poFeature = GetNextFeature()) != NULL )
			{
				m_FeatureCount++;
			}
			ResetReading();

			return m_FeatureCount;
		}
	}
	return m_features.size();
}


// 取坐标范围
SFERR XSFOGRVectorLayer::GetExtent(iplEnvelope *psExtent, int bForce)
{
	OGREnvelope Envelope;

	SFERR Err = /*OGR_L_GetExtent( m_ogrLayer, &Envelope, bForce)*/OGRERR_NONE;

	ResetReading();

	ISFFeature *feat = GetNextFeature();
	if (feat != NULL)
	{
		feat->GetGeometryRef()->getEnvelope(psExtent);
	}

	feat = GetNextFeature();
	if (feat != NULL)
	{
		iplEnvelope envelope;
		feat->GetGeometryRef()->getEnvelope(&envelope);

		if (psExtent->minX > envelope.minX)
			psExtent->minX = envelope.minX;
		if (psExtent->minY > envelope.minY)
			psExtent->minY = envelope.minY;
		if (psExtent->maxX < envelope.maxX)
			psExtent->maxX = envelope.maxX;
		if (psExtent->maxY < envelope.maxY)
			psExtent->maxY = envelope.maxY;
	}

	return Err;
}

// 建立域
// SFERR XSFOGRVectorLayer::CreateField( ISFFieldDefn *poField,
//                                      int bApproxOK )
// {
//     OGRFieldDefnH poOgrFieldDefn = CreateOgrFieldDefn( poField );
// 	
// 	//SFERR Err = m_ogrLayer->CreateField( poOgrFieldDefn, bApproxOK );
// 	SFERR Err = OGR_L_CreateField( m_ogrLayer, poOgrFieldDefn, bApproxOK );
// 	
// 	OGR_FD_Destroy( poOgrFieldDefn );
// 
// 	if (Err == OGRERR_NONE)
// 	{
// 		m_featureDefn.get()->AddFieldDefn(poField);
// 	}
// 	
// 	return Err;
// }

// int XSFOGRVectorLayer::TestCapability( const char * pszCapability)
// {
// 	//return m_ogrLayer->TestCapability(pszCapability);
// 
// 	return OGR_L_TestCapability( m_ogrLayer, pszCapability );
// }
// 
// int XSFOGRVectorLayer::TestDBF()
// {
// 	if (GetLayerDefn()->GetFieldCount() > 0)
// 		return TRUE;
// 	
// 	ref_ptr<ISFFieldDefn> fieldDefn( new XSFFieldDefn);
// 	
// 	fieldDefn.get()->Set("FID", SF_ftInteger, 11, 0, SF_JUndefined);
// 
// 	try
// 	{
// 		this->CreateField(fieldDefn.get());
// 	}
// 	catch(...)
// 	{
// 		return FALSE;
// 	}
// 	
// 	return TRUE;
// }
//////////////////////////////////////////////////////////////////////////
//
// 应当调用 OSF 的函数，自动更新到 OGR 外存
//	 先生成OGRFeature，再添加到缓存
//
// ISFFeature * XSFOGRVectorLayer::AppendFeature( IPL_wkbGeometryType type, iplPOINT3D *pts, int n )
// {
// 	CPLErrorReset();
// 
// 	OGRFeatureH poFeature = OGR_F_Create( OGR_L_GetLayerDefn( m_ogrLayer ) );
// 	
// 	//assert(padfX != NULL && padfY != NULL && padfZ != NULL);
// 
// #ifdef ORS_PLATFORM_WINDOWS
// 	DWORD nErrorNo = GetLastError (); // 得到错误代码
// 	LPSTR lpBuffer;    
// 	FormatMessage ( FORMAT_MESSAGE_ALLOCATE_BUFFER  | 
// 		          FORMAT_MESSAGE_IGNORE_INSERTS  | 
// 			      FORMAT_MESSAGE_FROM_SYSTEM,
// 		          NULL,
// 		          nErrorNo, // 此乃错误代码，通常在程序中可由 GetLastError()得之
// 		          LANG_NEUTRAL,
// 		          (LPTSTR) & lpBuffer,
// 		           0 ,
// 		          NULL );
// 
// 	orsString	 strErrorCause =  lpBuffer  ?  _T(lpBuffer) : _T( " Sorry, cannot find this error info. " );
// #endif
// 	
// 
// 	OGRGeometryH poOGRGeometry = NULL;
// 	
// 	switch( type ) {
// 	case wkbPoint25D:
// 		{
// // 			OGRPoint *poOGRPoint = new OGRPoint( *padfX, *padfY, *padfZ );
// // 			poOGRGeometry = poOGRPoint;
// 			
// 			poOGRGeometry = OGR_G_CreateGeometry( wkbPoint25D );
// 			OGR_G_SetPoint( poOGRGeometry, 0, pts->X, pts->Y, pts->Z );
// 		}
// 		break;
// 		
// 	case wkbLineString25D:
// 		{
// 			//OGRLineString *poOGRLine = new OGRLineString();
// 			//poOGRLine->setPoints( n, padfX, padfY, padfZ );
// 
// 			poOGRGeometry = OGR_G_CreateGeometry( wkbLineString25D );
// 
// 			for( int i=n-1; i>=0; i-- )
// 				OGR_G_SetPoint( poOGRGeometry, i, pts[i].X, pts[i].Y, pts[i].Z );
// 		}
// 		break;
// 
// 	case wkbPolygon25D:
// 		{
// 			poOGRGeometry = OGR_G_CreateGeometry( wkbPolygon25D );
// 			
// 			OGRGeometryH	poRing = OGR_G_CreateGeometry( wkbLinearRing );
// 
// 			//poRing->setPoints( n, padfX, padfY, padfZ );
// 
// 			for( int i=n-1; i>=0; i-- )
// 				OGR_G_SetPoint( poRing, i, pts[i].X, pts[i].Y, pts[i].Z );
// 
// 			OGR_G_AddGeometryDirectly( poOGRGeometry, poRing );	
// 		}
// 		break;
// 	}
// 
// 	// poFeature->SetField( "elevation", padfZ[0] );
// 	/////////////////////////////////////////////////////////////////////////
// 	
// 	// do not delete poOGR, it's not clone
// 
// 	OGR_F_SetGeometryDirectly( poFeature, poOGRGeometry );
// 		
// 	// create to OGR lib
// 	if( OGR_L_CreateFeature( m_ogrLayer, poFeature ) != OGRERR_NONE )  {
// 		OGR_F_Destroy( poFeature );
// 		return false;
// 	}
// 
// 	ref_ptr<ISFFeature> pFeature = ref_ptr<ISFFeature>(CreateFeatureFromOgr( poFeature ));
// 
// 	OGR_F_Destroy( poFeature );
// 
// 	m_features.push_back( pFeature );
// 	
// 	m_modified = true;
// 
// 	return pFeature.get();
// }

bool XSFOGRVectorLayer::AppendFeature(const ISFFeature* poFeature)
{
	if (poFeature)
	{
		m_features.push_back(ref_ptr<ISFFeature>(poFeature->Clone()));
		return true;
	}
	
	return false;
}

bool XSFOGRVectorLayer::AppendFeatureDirectly(ISFFeature* poFeature)
{
	if (poFeature)
	{
		m_features.push_back(ref_ptr<ISFFeature>(poFeature));
		return true;
	}
	
	return false;
}

// SFERR XSFOGRVectorLayer::StoreFeature(long id)
// {
// 	ISFFeature *poFeature = GetFeature(id);
// 	if (NULL == poFeature)
// 	{
// 		return SFERR_CORRUPT_DATA;
// 	}
// 	
// 	OGRFeatureH poOgrFeature = CreateOgrFeature(poFeature);
// 	if (NULL == poOgrFeature)
// 	{
// 		return SFERR_CORRUPT_DATA;
// 	}
// 	
// 	SFERR Err;
// 
// 	//ID<0为缓冲中的临时数据
// 	if (id < 0)
// 	{
// 		Err = OGR_L_CreateFeature( m_ogrLayer, poOgrFeature );
// 	}
// 	else	
// 	{
// 		Err = OGR_L_SetFeature( m_ogrLayer, poOgrFeature );
// 	}
// 	
// 	if (OGRERR_NONE == Err)
// 	{
// 		poFeature->SetID( OGR_F_GetFID(poOgrFeature) );
// 	}
// 
// 	OGR_F_Destroy( poOgrFeature );
// 	
// 	return Err;
// }

void XSFOGRVectorLayer::SetSpatialReference(ref_ptr<ISpatialReference> pSRS)
{
	m_pSRS.reset();

	m_pSRS = pSRS;
}
