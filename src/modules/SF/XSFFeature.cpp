#include <GDAL/cpl_conv.h>
#include <GDAL/cpl_string.h>
#include <GDAL/ogr_core.h>

#include "XSFFeature.h"

using namespace ipl;

XSFFeature::XSFFeature( ref_ptr<ISFFeatureDefn> poDefnIn)
{
// 	m_styleString = NULL;
/*  m_styleTable = NULL;*/
    m_tmpFieldValue = NULL;

    //m_defnIn->Reference();

    m_defn = poDefnIn;

    m_id = SF_nullFID;
    
    m_geometry = NULL;

	m_fields = NULL;

    // we should likely be initializing from the defaults, but this will
    // usually be a waste.

	if (m_defn->GetFieldCount() > 0)
	{
		int cnt = m_defn->GetFieldCount();
		m_fields = new SFField[cnt];

		for( int i = 0; i < cnt; i++ )
		{
			m_fields[i].Set.nMarker1 = SF_unSetMarker;
			m_fields[i].Set.nMarker2 = SF_unSetMarker;
		}
	}
}

XSFFeature::~XSFFeature()
{	
	if(m_fields) {
		for( int i = 0; i < m_defn->GetFieldCount(); i++ )
		{
			if( !IsFieldSet(i) )
				continue;

			ISFFieldDefn *poFDefn = m_defn->GetFieldDefn(i);
			
			switch( poFDefn->GetType() )
			{
			case OFTString:
				CPLFree(m_fields[i].String);
				break;
			case OFTBinary:
				CPLFree( m_fields[i].Binary.paData );
				break;
			case OFTStringList:
				CSLDestroy( m_fields[i].StringList.paList );
				break;
			case OFTIntegerList:
				CPLFree( m_fields[i].IntegerList.paList );
				break;
			case OFTRealList:
				CPLFree( m_fields[i].RealList.paList );
				break;
			default:
				// should add support for wide strings.
				break;
			}
		}

		delete[] m_fields;
	}

//    m_defn->Release();

//     delete (m_styleString);
    if(m_tmpFieldValue) delete m_tmpFieldValue;

}


SFERR  XSFFeature::SetGeometryDirectly( ref_ptr<IGeoObject> poGeomIn )
{
	//delete m_geometry;
    m_geometry = poGeomIn;

// I should be verifying that the geometry matches the defn's type.  

    return SFERR_NONE;
}


ipl::SFERR  ipl::XSFFeature::SetGeometry( const ref_ptr<IGeoObject> poGeomIn)
{
	if( poGeomIn != NULL )
        m_geometry = ref_ptr<IGeoObject>(poGeomIn->Clone());
    else
        m_geometry = NULL;
    // I should be verifying that the geometry matches the defn's type.
    
    return SFERR_NONE;
}


IGeoObject *XSFFeature::GetGeometryRef()
{
	return m_geometry.get();
}

ref_ptr<IGeoObject> ipl::XSFFeature::StealGeometry()
{
	ref_ptr<IGeoObject> poReturn = m_geometry;
    
	m_geometry = NULL;

    return poReturn;
}


ISFFeature  *XSFFeature::Clone() const
{
	ISFFeature  *poNew = new XSFFeature( m_defn );
	
    poNew->SetGeometry(ref_ptr<IGeoObject>(m_geometry->Clone()) );
	
    for( int i = 0; i < m_defn->GetFieldCount(); i++ )
    {
        poNew->SetField( i, m_fields + i );
    }
	
//     if( GetStyleString() != NULL )
//         poNew->SetStyleString(GetStyleString());
	
    poNew->SetID( GetID() );
	
    return poNew;
}

size_t XSFFeature::GetFieldCount() 
{
	return m_defn->GetFieldCount(); 
}

ISFFieldDefn *XSFFeature::GetFieldDefnRef( int iField )
{
	return m_defn->GetFieldDefn(iField);
}

int  XSFFeature::GetFieldIndex( const char * pszName)
{
	return m_defn->GetFieldIndex(pszName);
}

SFField *XSFFeature::GetRawFieldRef( int i ) 
{
	return m_fields + i;
}

void XSFFeature::SetField( int i, ipl::SFField * puValue )
{
	if (puValue == NULL)
		return;
	
	ISFFieldDefn *poFDefn = m_defn->GetFieldDefn(i);

    if( poFDefn->GetType() == OFTInteger || poFDefn->GetType() == OFTInteger64)
    {
        m_fields[i] = *puValue;
    }
    else if( poFDefn->GetType() == OFTReal )
    {
        m_fields[i] = *puValue;
    }
    else if( poFDefn->GetType() == OFTString )
    {
        if( IsFieldSet( i ) )
            CPLFree( m_fields[i].String );

		if (puValue->Set.nMarker1 == OGRNullMarker &&
			puValue->Set.nMarker2 == OGRNullMarker &&
			puValue->Set.nMarker3 == OGRNullMarker)
			m_fields[i].String = NULL;
		else if( puValue->String == NULL )
            m_fields[i].String = NULL;
        else if( puValue->Set.nMarker1 == SF_unSetMarker
                 && puValue->Set.nMarker2 == SF_unSetMarker )
            m_fields[i] = *puValue;
        else
            m_fields[i].String = CPLStrdup( puValue->String );
    }
    else if( poFDefn->GetType() == OFTDate
             || poFDefn->GetType() == OFTTime
             || poFDefn->GetType() == OFTDateTime )
    {
        memcpy( m_fields+i, puValue, sizeof(OGRField) );
    }
    else if( poFDefn->GetType() == OFTIntegerList )
    {
        int     nCount = puValue->IntegerList.nCount;
        
        if( IsFieldSet( i ) )
            CPLFree( m_fields[i].IntegerList.paList );
        
        if( puValue->Set.nMarker1 == OGRUnsetMarker
            && puValue->Set.nMarker2 == OGRUnsetMarker )
        {
            m_fields[i] = *puValue;
        }
        else
        {
            m_fields[i].IntegerList.paList =
                (int *) CPLMalloc(sizeof(int) * nCount);
            memcpy( m_fields[i].IntegerList.paList,
                    puValue->IntegerList.paList,
                    sizeof(int) * nCount );
            m_fields[i].IntegerList.nCount = nCount;
        }
    }
    else if( poFDefn->GetType() == OFTRealList )
    {
        int     nCount = puValue->RealList.nCount;

        if( IsFieldSet( i ) )
            CPLFree( m_fields[i].RealList.paList );

        if( puValue->Set.nMarker1 == OGRUnsetMarker
            && puValue->Set.nMarker2 == OGRUnsetMarker )
        {
            m_fields[i] = *puValue;
        }
        else
        {
            m_fields[i].RealList.paList =
                (double *) CPLMalloc(sizeof(double) * nCount);
            memcpy( m_fields[i].RealList.paList,
                    puValue->RealList.paList,
                    sizeof(double) * nCount );
            m_fields[i].RealList.nCount = nCount;
        }
    }
    else if( poFDefn->GetType() == OFTStringList )
    {
        if( IsFieldSet( i ) )
            CSLDestroy( m_fields[i].StringList.paList );
        
        if( puValue->Set.nMarker1 == OGRUnsetMarker
            && puValue->Set.nMarker2 == OGRUnsetMarker )
        {
            m_fields[i] = *puValue;
        }
        else
        {
            m_fields[i].StringList.paList =
                CSLDuplicate( puValue->StringList.paList );
            
            m_fields[i].StringList.nCount = puValue->StringList.nCount;
            CPLAssert( CSLCount(puValue->StringList.paList)
                       == puValue->StringList.nCount );
        }
    }
    else if( poFDefn->GetType() == OFTBinary )
    {
        if( IsFieldSet( i ) )
            CPLFree( m_fields[i].Binary.paData );
        
        if( puValue->Set.nMarker1 == SF_unSetMarker
            && puValue->Set.nMarker2 == SF_unSetMarker )
        {
            m_fields[i] = *puValue;
        }
        else
        {
            m_fields[i].Binary.nCount = puValue->Binary.nCount;
            m_fields[i].Binary.paData = 
                (GByte *) CPLMalloc(puValue->Binary.nCount);
            memcpy( m_fields[i].Binary.paData, 
                    puValue->Binary.paData, 
                    puValue->Binary.nCount );
        }
    }
    else
        /* do nothing for other field types */;
}
