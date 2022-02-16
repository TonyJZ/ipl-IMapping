#include "XSFFieldDefn.h"

using namespace ipl;

//////////////////////////////////////////////////////////////////////////
//XSFFieldDefn
//////////////////////////////////////////////////////////////////////////
XSFFieldDefn::XSFFieldDefn()
{

}

XSFFieldDefn::~XSFFieldDefn()
{
}

void XSFFieldDefn::SetName( const char *pszName )
{
	m_name = pszName;
}

const char * XSFFieldDefn::GetNameRef()
{
	return m_name.c_str();
}


void XSFFieldDefn::SetDefault( const SFField *posfF )
{
	m_default = *posfF;
}

const SFField* XSFFieldDefn::GetDefaultRef()
{
	return &m_default;
}


void XSFFieldDefn::Set( const char * pszName, SF_FieldType type,  int nWidthIn, int nPrecisionIn, SF_Justification justification )
{
	m_name = pszName;
	m_type = type;

	m_width = nWidthIn;
	m_precision = nPrecisionIn;
	m_justify = justification;
}

ISFFieldDefn* XSFFieldDefn::Clone() const
{
	ISFFieldDefn  *poNew = new XSFFieldDefn();
	poNew->Set(m_name.c_str(), m_type, m_width, m_precision, m_justify);

	poNew->SetDefault(&m_default);

	return poNew;
}
