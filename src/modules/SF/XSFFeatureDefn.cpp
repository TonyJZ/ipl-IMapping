#include "XSFFeatureDefn.h"


using namespace ipl;
//////////////////////////////////////////////////////////////////////////
//XSFFeatureDefn
//////////////////////////////////////////////////////////////////////////
XSFFeatureDefn::XSFFeatureDefn()
{
	
}

XSFFeatureDefn::~XSFFeatureDefn()
{
	
}

void XSFFeatureDefn::SetOGRFeatureDefn(OGRFeatureDefnH pofeatureDF)
{
	//m_poFeatureDefn = pofeatureDF;
}


void XSFFeatureDefn::SetName( const char  *name )
{
	m_featureClassName = name;	
}


const char* XSFFeatureDefn::GetName()
{
	return m_featureClassName.c_str();
}

// int  XSFFeatureDefn::GetFieldCount()
// {
// 	return( m_poFeatureDefn->GetFieldCount() );
// }


ISFFieldDefn *XSFFeatureDefn::GetFieldDefn( int i )
{
	if( i < m_vFiledDefn.size() )
		return m_vFiledDefn[i].get();

	return NULL;
}


int  XSFFeatureDefn::GetFieldIndex( const char *pszName )
{
	int i;
	for( i=0; i<m_vFiledDefn.size(); i++ )	{
		if( strcmp( m_vFiledDefn[i]->GetNameRef(), pszName ) == 0 )
			return i;
	}

	return -1;
}


void  XSFFeatureDefn::AddFieldDefn( ISFFieldDefn *piFieldGefn )
{
	m_vFiledDefn.push_back( ref_ptr<ISFFieldDefn>(piFieldGefn));
}



// ISFFeatureDefn * XSFFeatureDefn::Clone() const
// {
// 	OGRFeatureDefnH* poFD = m_poFeatureDefn->Clone();
// 	if (poFD != NULL)
// 	{
// 		XSFFeatureDefn *poxFD = new XSFFeatureDefn();
// 		poxFD->SetOGRFeatureDefn(poFD);
// 		return poxFD;
// 	}
// 	else
// 		return NULL;
// }

