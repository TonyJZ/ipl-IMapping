
//#include "orsBase/orsUtil.h"
#include "XProperty.h"
#include "iplCommon.h"


using namespace ipl;

struct blobDATA {
	ipl_int32 length;
	ipl_byte  data[1];
};

// 内存块
class iplBlob : public iplBLOB
{
private:
	blobDATA *alloc( ipl_int32 length );

public:
	iplBlob()	{ pData = NULL; }
	~iplBlob()	{ free( pData );	}

	bool SetData( const ipl_byte *data, ipl_int32 length );

	bool SetString( const iplChar * str );

	void setVector( const iplVectorBase &value );
	void setMatrix( const iplMatrixBase &value );

	const iplBlob &operator = (const iplBlob &v)
	{
		SetData( v.GetData(), v.GetLength() );
		return *this;
	}

	ipl_int32 GetLength()  const
	{
		return ((blobDATA *)pData)->length;
	}

	const ipl_byte *GetData() const
	{
		return ((blobDATA *)pData)->data;
	};

	const iplChar * GetString() const
	{
		if(pData == NULL)
			return NULL;
		return (const iplChar *)((blobDATA *)pData)->data;
	};

	bool getVector( iplVectorBase &value ) const;
	bool getMatrix( iplMatrixBase &value ) const;

};

blobDATA *iplBlob::alloc( ipl_int32 length )
{
	if( NULL != pData )	{
		blobDATA *blob = (blobDATA *)pData;
		if( blob->length < length )	{
			free( pData );
			pData = NULL;
		}
		else
			return (blobDATA *)pData;
	}

	if( NULL != pData )
		return (blobDATA *)pData;

	pData = malloc( length + sizeof(ipl_int32) );
	blobDATA *blob = (blobDATA *)pData;
	blob->length = length;

	return blob;
}


bool iplBlob::SetData( const ipl_byte *data, ipl_int32 length )
{
	blobDATA *blob = alloc( length );

	if( NULL != blob ) {
		memcpy( blob->data, data, length );
		return true;
	}

	return false;
}

bool iplBlob::SetString( const iplChar * str )
{
	if(str == NULL)
		return false;
	int len = strlen(str);
	blobDATA *blob = alloc( len + 1 );

	if( blob )	{
		memcpy( blob->data, str, len );
		blob->data[len] = 0;
		return true;
	}

	return false;
}

void iplBlob::setVector( const iplVectorBase &v )
{
	int headBytes = 2*sizeof( ipl_int32 );
	int dataBytes = v.Rows()*iplGetSizeOfType( v.GetDataType() );

	blobDATA *blob = alloc( headBytes + dataBytes );

	ipl_int32 *pInfo = (ipl_int32 *)blob->data;

	pInfo[0] = v.GetDataType();	
	pInfo[1] = v.Rows();

	memcpy( blob->data + headBytes, v.Buf(), dataBytes );
}

void iplBlob::setMatrix( const iplMatrixBase &m )
{
	int headBytes = 3*sizeof( ipl_int32 );
	int dataBytes = m.Rows()*m.Cols() * iplGetSizeOfType( m.GetDataType() );

	blobDATA *blob = alloc( headBytes + dataBytes );
	ipl_int32 *pInfo = (ipl_int32 *)blob->data;
	
	pInfo[0] = m.GetDataType();	
	pInfo[1] = m.Rows();
	pInfo[2] = m.Cols();
	
	memcpy( blob->data + headBytes, m.Buf(), dataBytes );
}

bool iplBlob::getVector( iplVectorBase &vectorOut ) const
{
	blobDATA *blob = (blobDATA *)pData;
	ipl_int32 *pInfo = (ipl_int32 *)blob->data;

	iplDataTYPE type = (iplDataTYPE)pInfo[0];
	ipl_int32 nRows =  pInfo[1];

	int headBytes = 2*sizeof( ipl_int32 );
	int dataBytes = nRows* iplGetSizeOfType( type );

	vectorOut.SetDataType( type );

	switch( type)	{
	case IPL_DT_FLOAT32:
		{
			iplVector<ipl_float32>	*tVector = (iplVector<ipl_float32>	*)&vectorOut;
			tVector->Alloc( nRows );
		}
		break;
	case IPL_DT_FLOAT64:
		{
			iplVector<ipl_float64>	*tVector = (iplVector<ipl_float64>	*)&vectorOut;
			tVector->Alloc( nRows );
		}
		break;
	default:
		assert( false );
	}

	memcpy( vectorOut.Buf(), (BYTE *)blob->data+headBytes, dataBytes );

	return true;
}

bool iplBlob::getMatrix( iplMatrixBase &matrixOut ) const
{
	blobDATA *blob = (blobDATA *)pData;
	ipl_int32 *pInfo = (ipl_int32 *)blob->data;
	
	iplDataTYPE type = (iplDataTYPE)pInfo[0];
	ipl_int32 nRows =  pInfo[1];
	ipl_int32 nCols =  pInfo[2];
	
	int headBytes = 3*sizeof( ipl_int32 );
	int dataBytes = nRows*nCols*iplGetSizeOfType( type );
	
	matrixOut.SetDataType( type );
	
	switch( type)	{
	case IPL_DT_FLOAT32:
		{
			iplMatrix<ipl_float32>	*tMatrix = (iplMatrix<ipl_float32>	*)&matrixOut;
			tMatrix->Alloc( nRows, nCols );
		}
		break;
	case IPL_DT_FLOAT64:
		{
			iplMatrix<ipl_float64>	*tMatrix = (iplMatrix<ipl_float64>	*)&matrixOut;
			tMatrix->Alloc( nRows, nCols );
		}
		break;
	default:
		assert( false );
	}

	memcpy( matrixOut.Buf(), (BYTE *)blob->data+headBytes, dataBytes );

	return true;
}

//////////////////////////////////////////////////////////////////////////

iplVariant::iplVariant()
{
	memset( this, 0, sizeof(iplVariant) );
// 	type = IPL_V_UNKOWN;
// 	vInt32 = 0;
// 	vInt16 = 0;
// 	vFloat64 = 0;
// 	b_value_ptr = NULL;
// 	b_value_length = 0;
// 	pProperty = NULL;
}

iplVariant::iplVariant (iplVariant & var)
{
	type = var.type ;
	switch (var.type) {
	case IPL_V_I2 :
		vInt16 = var.vInt16 ;
		break;
	case IPL_V_I4 :
		vInt32 = var.vInt32 ;
		break ;
	case IPL_V_R8 :
		vFloat64 = var.vFloat64 ;
		break ;
	case IPL_V_STR :
		set( var.getString() );
		break;
	case IPL_V_BLOB :
		{
			ipl_int32 len;
			const ipl_byte *buf= var.getBlob( len );
			set( buf, len );
		}
		break ;
	case IPL_V_CHILD :
		set( var.getProperty() );
		break ;
	case IPL_V_OBJECT:
		set( /*(IObject *)NULL*/var.getObject() );

	default:
		assert( false );
	}
}


iplVariant::~iplVariant()
{
	switch( type )	{
	case IPL_V_OBJECT:
// 		if( vObject )
// 			vObject->release();
		break;
	case IPL_V_CHILD :
// 		if( vProperty )
// 			vProperty->release();
		break;
	case IPL_V_STR:
	case IPL_V_BLOB:
		//delete (ipl_byte*)vBlob.pData;
		free( (ipl_byte*)vBlob.pData );
	}
}

void iplVariant::set( ref_ptr<IProperty> property )
{
	if( property.get() == vProperty.get() )
		return ;

// 	if( vProperty )
// 		vProperty->release();
	vProperty.reset();
	vProperty = property;

// 	if( vProperty )
// 		vProperty->addRef();
}

void iplVariant::set( ref_ptr<IObject> pObject )
{
	if( vObject.get() == pObject.get() )
		return ;

// 	if( vObject )
// 		vObject->release();
	
	vObject.reset();
	vObject = pObject;

// 	if( vObject )
// 		vObject->addRef();
}


void iplVariant::set( const ipl_byte *data, ipl_int32 length )
{
	iplBlob *pBlob = (iplBlob *)&vBlob;
	pBlob->SetData( data, length );
}

void iplVariant::set( const iplChar * str )
{
	iplBlob *pBlob = (iplBlob *)&vBlob;
	pBlob->SetString( str );
}


void iplVariant::set( const iplVectorBase &vector )
{
	iplBlob *pBlob = (iplBlob *)&vBlob;
	pBlob->setVector( vector );
}

void iplVariant::set( const iplMatrixBase &matrix )
{
	iplBlob *pBlob = (iplBlob *)&vBlob;
	pBlob->setMatrix( matrix );
}



void iplVariant::getVector( iplVectorBase &vector )
{
	iplBlob *pBlob = (iplBlob *)&vBlob;
	pBlob->getVector( vector );
}

void iplVariant::getMatrix( iplMatrixBase &matrix )
{
	iplBlob *pBlob = (iplBlob *)&vBlob;
	pBlob->getMatrix( matrix );
}


const ipl_byte *iplVariant::getBlob( ipl_int32 &length )
{
	iplBlob *pBlob = (iplBlob *)&vBlob;

	length = pBlob->GetLength();

	return pBlob->GetData();
}

const iplChar * iplVariant::getString()
{
	iplBlob *pBlob = (iplBlob *)&vBlob;
	return pBlob->GetString();
}

PropertyRecord::~PropertyRecord()
{
// 	for( unsigned int i=0; i<size();i++)
// 	{
// 		delete GetVariant( i );
// 	}
	variants.clear();
}

////////////////////////////////////XProperty/////////////////////////////

// 防止属性名带空格
iplString ToRegularName( const iplChar *name1 )
{	
	iplString name = name1;

	iplChar *s = (iplChar *)name.c_str();
	while( *s )	{
		switch( *s ) {
		case ' ':
		case '(':
		case ')':
		case '%':
			*s = '_'; 
		}
		s++; 
	};

	return name;
}



typedef PropertyRecordMap::value_type		_recordType;
typedef PropertyRecordMap::iterator			_recordIterator;
typedef PropertyRecordMap::const_iterator	_recordConstIterator;

XProperty::XProperty()//:m_listener(NULL)
{
}

XProperty::~XProperty()
{
	removeAll();
}

bool XProperty::removeAll()
{
// 	_recordIterator iter;
// 	
// 	for(iter = m_records.begin();iter != m_records.end();iter++)
// 	{
// 		PropertyRecord *pRecord = iter->second;
// 		
// 		delete pRecord;
// 	}

	m_records.clear();

	m_attrNames.clear();

	return true;
}


bool XProperty::remove( const iplChar * name )
{
	bool bRet = false;

	std::vector<iplString>::iterator it;

	for( it = m_attrNames.begin(); it<m_attrNames.end(); it++ )
	{
		if( 0 == it->compare( name ) ) {
			m_attrNames.erase( it );
			bRet = true;
			break;
		}
	}

	m_records.erase( name);
	
	return bRet;
}

//////////////////////////////////////////////////////////////////////////

#define ADD_RECORD0(name1, value)\
if( bUnique ) {\
	if( setAttr( name1, value ) )\
		return ;\
}\
\
iplString name = ToRegularName( name1 );\
\
_recordIterator pointer = m_records.find(name);\
PropertyRecord *pRecord = NULL;\
\
if(pointer == m_records.end()){\
	pRecord = new PropertyRecord;\
	m_records.insert(_recordType(name, ref_ptr<PropertyRecord>(pRecord)));\
	\
	m_attrNames.push_back( name );\
} else {\
	pRecord = pointer->second.get();\
}

//////////////////////////////////////////////////////////////////////////

#define ADD_RECORD1(name1, value)\
	if( bUnique ) {\
	if( setAttr( name1, value, 0 ) )\
	return ;\
	}\
	\
	iplString name = ToRegularName( name1 );\
	\
	_recordIterator pointer = m_records.find(name);\
	PropertyRecord *pRecord = NULL;\
	\
	if(pointer == m_records.end()){\
	pRecord = new PropertyRecord;\
	m_records.insert(_recordType(name, ref_ptr<PropertyRecord>(pRecord)));\
	\
	m_attrNames.push_back( name );\
	} else {\
	pRecord = pointer->second.get();\
}

//////////////////////////////////////////////////////////////////////////

void XProperty::addAttr(const iplChar * name1, bool value, bool bUnique )
{
	ADD_RECORD0(name1,value);
	
	ref_ptr<iplVariant>	var( new iplVariant);
	//iplVariant	*var = pRecord;
	
	var->type = IPL_V_BOOL;
	var->set( value );
	
	pRecord->AddVariant(var);
}


void XProperty::addAttr(const iplChar * name1,ipl_int16 value, bool bUnique )
{
	ADD_RECORD1( name1, value);
	
	ref_ptr<iplVariant>	var(new iplVariant);
	//iplVariant	*var = pRecord;
	
	var->type = IPL_V_I2;
	var->set( value );
	
	pRecord->AddVariant(var);
}


void XProperty::addAttr(const iplChar * name1, ipl_int32 value, bool bUnique )
{
	ADD_RECORD1(name1,value);
	
	ref_ptr<iplVariant>	var(new iplVariant);
	//iplVariant	*var = pRecord;
	
	var->type = IPL_V_I4;
	var->set( value );
	
	pRecord->AddVariant( var );
}



void XProperty::addAttr(const iplChar * name1, ipl_float64 value, bool bUnique )
{
	ADD_RECORD1(name1,value);

	ref_ptr<iplVariant>	var(new iplVariant);
	//iplVariant	*var = pRecord;

	var->type = IPL_V_R8;
	var->set( value );

	pRecord->AddVariant( var );
}

void XProperty::addAttr(const iplChar * name1, const iplChar * value, bool bUnique )
{
	ADD_RECORD1(name1,value);

	ref_ptr<iplVariant>	var(new iplVariant);
	//iplVariant	*var = pRecord;

	var->type = IPL_V_STR;
	var->set( value );

	pRecord->AddVariant(var);
}
void XProperty::addAttribute(const iplChar * name1, const iplChar * value, bool bUnique )
{
	ADD_RECORD1(name1,value);

	ref_ptr<iplVariant>	var(new iplVariant);
	//iplVariant	*var = pRecord;

	var->type = IPL_V_Attribute;
	var->set( value );

	pRecord->AddVariant(var);
}
void XProperty::addAttr(const iplChar * name1, const iplVectorBase &value , bool bUnique )
{
	ADD_RECORD0(name1,value);

	ref_ptr<iplVariant>	var(new iplVariant);
	//iplVariant	*var = pRecord;

	var->type = IPL_V_VECTOR;
	var->set( value );

	pRecord->AddVariant( var );
}

void XProperty::addAttr(const iplChar * name1, const iplMatrixBase &value, bool bUnique )
{
	ADD_RECORD0(name1,value);
	
	ref_ptr<iplVariant>	var(new iplVariant);
	//iplVariant	*var = pRecord;
	
	var->type = IPL_V_MATRIX;
	var->set( value );
	
	pRecord->AddVariant( var );
	
}

void XProperty::addAttr(const iplChar * name1, const ipl_byte *pValue, ipl_int32 nLength, bool bUnique )
{
	ADD_RECORD0(name1, pValue);

	ref_ptr<iplVariant>	var(new iplVariant);
	//iplVariant	*var = pRecord;

	var->type = IPL_V_BLOB;
	var->set( pValue, nLength );

	pRecord->AddVariant(var);
}



//////////////////////////////////////////////////////////////////////////

#define GET_RECORD(name1, value)\
iplString name = ToRegularName( name1 );\
\
_recordIterator iter;\
iter = m_records.find(name);\
\
if(iter == m_records.end())\
return false;\
\
PropertyRecord* pRecord = iter->second.get();

//////////////////////////////////////////////////////////////////////////

bool XProperty::setAttr(const iplChar * name1, bool value)
{
	GET_RECORD( name1, value);
	
	ref_ptr<iplVariant> var = pRecord->GetVariant( 0 );
	
	switch( var->type )
	{
	case IPL_V_BOOL:
		var->set( value );
		break;
	default:
		return false;
	}
		
	return true;
}


bool XProperty::setAttr(const iplChar * name1, ipl_int32 value, ipl_uint32 index)
{
	GET_RECORD( name1, value);
		
	if( index >= pRecord->size())
		return false;
	
	ref_ptr<iplVariant> var = pRecord->GetVariant( index );
	
	switch( var->type )
	{
	case IPL_V_I4:
		var->set( value );
		break;
	case IPL_V_I2:
		var->set( (ipl_int16)value );
	default:
		return false;
	}

	return true;
}

bool XProperty::setAttr(const iplChar * name1, ipl_int16 value, ipl_uint32 index)
{
	GET_RECORD( name1, value);
	
	if(index >= pRecord->size())
		return false;
	
	ref_ptr<iplVariant> var = pRecord->GetVariant(index);
	
	switch( var->type )
	{
	case IPL_V_I2:
		var->set( value );
		break;
	case IPL_V_I4:
		var->set( (ipl_int32)value );
	default:
		return false;
	}

	return true;
}

bool XProperty::setAttr(const iplChar * name1, const iplChar * value, ipl_uint32 index)
{
	GET_RECORD( name1, value);
	
	if( index >= pRecord->size() )
		return false;
	
	ref_ptr<iplVariant> var = pRecord->GetVariant(index);
	
	if(var->type == IPL_V_STR)
		var->set( value );
	else
		return false;

	return true;
}


bool XProperty::setAttr(const iplChar * name1, const iplVectorBase &value )
{
	GET_RECORD( name1, value);

	ref_ptr<iplVariant> var = pRecord->GetVariant(0);

	var->type = IPL_V_VECTOR;

	var->set( value );
	
	return true;
}

bool XProperty::setAttr(const iplChar * name1, const iplMatrixBase &value )
{
	GET_RECORD( name1, value);

	ref_ptr<iplVariant> var = pRecord->GetVariant(0);
	
	var->type = IPL_V_MATRIX;
	var->set( value );

	return true;
}

bool XProperty::setAttr(const iplChar * name1, ipl_float64 value, ipl_uint32 index)
{
	GET_RECORD( name1, value);
	
	if(index >= pRecord->size())
		return false;
	
	ref_ptr<iplVariant> var = pRecord->GetVariant(index);
	
	if(var->type == IPL_V_I2)
	{
		var->set( (ipl_int16)value );
	}
	else if(var->type == IPL_V_I4)
	{
		var->set( (ipl_int32)value );
	}
	else if(var->type == IPL_V_R8)
	{
		var->set( value );
	}
	else
		return false;

	return true;
}

bool XProperty::setAttr(const iplChar * name1, const ipl_byte* pValue, ipl_int32 nLength,
								ipl_uint32 index)
{
	GET_RECORD( name1, value);
	
	if(index >= pRecord->size())
		return false;
	
	ref_ptr<iplVariant> var = pRecord->GetVariant(index);
	
	if( var->type == IPL_V_BLOB )
	{
		var->set( pValue, nLength );
	}
	else
		return false;

	return true;
}

bool XProperty::setAttr(const iplChar * name1, ref_ptr<IProperty> value, ipl_uint32 index )
{
	GET_RECORD( name1, value);
	
	if( index >= pRecord->size() )
		return false;
	
	ref_ptr<iplVariant> var = pRecord->GetVariant( index );
	
	if( var->type == IPL_V_CHILD )
	{
		var->set( value );
	}
	else
		return false;

	return true;
}


//////////////////////////////////////////////////////////////////////////

bool XProperty::getAttr( const iplChar * name1, bool & value )  const
{
	iplString name = ToRegularName( name1 );

	PropertyRecord* pRecord;
	_recordConstIterator it = m_records.find(name) ;
	
	if (it != m_records.end()) {
		pRecord = it->second.get() ;
		for ( unsigned i = 0 ; i < pRecord->size() ; i++) {
			switch( pRecord->GetVariant(i)->type )
			{
			case IPL_V_BOOL:
				value = pRecord->GetVariant(i)->getBool();
				break;
			case IPL_V_STR:
				{
					//if( iplString::findSubStr_i( pRecord->GetVariant(i)->getString(), "true" ) )
					iplString str = pRecord->GetVariant(i)->getString();
					if (iplString::npos != str.find("true"))
						value = true;
					else if( iplString::npos != str.find( "false" ) )
						value = false;
					else
						return false;
				}
				break;
			}
		}
		
		return true ;
	}
	return false ;
}

bool XProperty::getAttr(const iplChar * name1, iplArray<ipl_int16> &values) const
{
	iplString name = ToRegularName( name1 );

	PropertyRecord* pRecord;
	_recordConstIterator it = m_records.find(name) ;

	if (it != m_records.end()) {
		values.clear();

		pRecord = it->second.get() ;
		for ( unsigned i = 0 ; i < pRecord->size() ; i++) {
			switch( pRecord->GetVariant(i)->type )
			{
			case IPL_V_I2:
				values.push_back( pRecord->GetVariant(i)->getInt16() );
				break;
			case IPL_V_I4:
				values.push_back( pRecord->GetVariant(i)->getInt32() );
				break;
			case IPL_V_STR:
				{
					int value;
					
					if( 1 == sscanf( pRecord->GetVariant(i)->getString(), "%d", &value ) )
						values.push_back( (ipl_int16)value );
				}
				break;
			default:
				return false;
				break;
			}
		}
		return true ;
	}
	return false ;
}


bool XProperty::getAttr(const iplChar * name1, iplArray<ipl_int32> &values) const
{
	iplString name = ToRegularName( name1 );

	PropertyRecord* pRecord;
	_recordConstIterator it = m_records.find(name) ;

	if (it != m_records.end()) {
		values.clear();

		pRecord = it->second.get() ;
		for ( unsigned i = 0 ; i < pRecord->size() ; i++) {
			switch(pRecord->GetVariant(i)->type)
			{
			case IPL_V_I2:
				values.push_back(pRecord->GetVariant(i)->getInt16()) ;
				break;
			case IPL_V_I4:
				values.push_back(pRecord->GetVariant(i)->getInt32()) ;
				break;
			case IPL_V_STR:
				{
					int value;
					
					if( 1 == sscanf( pRecord->GetVariant(i)->getString(), "%d", &value ) )
						values.push_back( (ipl_int32)value );
				}
				break;
			default:
				return false;
				break;
			}
		}
		return true ;
	}
	return false ;
}


bool
XProperty::getAttr(const iplChar * name1,iplArray<ipl_float64> &values) const
{
	iplString name = ToRegularName( name1 );

	PropertyRecord* pRecord;
	_recordConstIterator it = m_records.find(name) ;

	if (it != m_records.end()) {
		values.clear();

		pRecord = it->second.get() ;
		for ( unsigned i = 0 ; i < pRecord->size() ; i++)
		{
			switch( pRecord->GetVariant(i)->type )
			{
			case IPL_V_I2:
				values.push_back( pRecord->GetVariant(i)->getInt16() ) ;
				break;
			case IPL_V_I4:
				values.push_back( pRecord->GetVariant(i)->getInt32() ) ;
				break;
			case IPL_V_R8:
				values.push_back( pRecord->GetVariant(i)->getFloat64() ) ;
				break;
			case IPL_V_STR:
				{
					double value;
					
					if( 1 == sscanf( pRecord->GetVariant(i)->getString(), "%lf", &value ) )
						values.push_back( (ipl_float64)value );
				}
				break;
			}
		}

		return true ;
	}

	return false ;
}

bool XProperty::getAttr(const iplChar * name1, iplArray<const ipl_byte *> &values, iplArray<ipl_int32> &vLength) const
{
	iplString name = ToRegularName( name1 );

	PropertyRecord * pRecord;
	_recordConstIterator it = m_records.find(name) ;

	if (it != m_records.end()) {
		values.clear();
		vLength.clear();

		pRecord = it->second.get() ;
		for ( unsigned i = 0 ; i < pRecord->size() ; i++)
		{
			switch( pRecord->GetVariant(i)->type )	{
			case IPL_V_BLOB:
			case IPL_V_VECTOR:
			case IPL_V_MATRIX:
				{
					ipl_int32 len;
					values.push_back( pRecord->GetVariant(i)->getBlob( len ) ) ;
					vLength.push_back( len );
				}
				break;
			}			
		}

		return true ;
	}

	return false ;
}

bool XProperty::getAttr(const iplChar * name1, iplArray<iplString> &values) const
{
	iplString name = ToRegularName( name1 );

	PropertyRecord* pRecord;

	_recordConstIterator it = m_records.find( name ) ;

	if( it != m_records.end() ) {
		values.clear();

		pRecord = it->second.get() ;
		for (unsigned i = 0 ; i < pRecord->size() ; i++)
		{
			switch( pRecord->GetVariant(i)->type ) {
			case IPL_V_STR:
				{
					const iplChar * str = pRecord->GetVariant(i)->getString();
					if(str == NULL){
						iplString nullstr;
						values.push_back(nullstr);
					}else {
						values.push_back(str);
					}
				}
				break;
			case IPL_V_Attribute:
				{
					const iplChar * str = pRecord->GetVariant(i)->getString();
					if(str == NULL){
						iplString nullstr;
						values.push_back(nullstr);
					}else {
						values.push_back(str);
					}
				}
				break;
			case IPL_V_BLOB:
				{
					ref_ptr<iplVariant> var = pRecord->GetVariant( i );

					ipl_int32 len;
					const ipl_byte *buf = var->getBlob( len );

					char * temp_str = new char[len+1];
					memcpy( temp_str, buf, len );

					temp_str[len] = '\0';

					values.push_back( temp_str );

					delete temp_str;
				}
			default:
				return false;
			}
		}
		return true ;
	}
	return false ;
}

bool
XProperty::getAttr(const iplChar * name1,iplArray< ref_ptr<IProperty> > &values ) const
{
	iplString name = ToRegularName( name1 );

	PropertyRecord* pRecord;
	_recordConstIterator it = m_records.find(name) ;

	if (it != m_records.end()) {
		values.clear();

		pRecord = it->second.get() ;
		for (unsigned i = 0 ; i < pRecord->size() ; i++)
		{
			if( IPL_V_CHILD == pRecord->GetVariant(i)->type )
				values.push_back( ref_ptr<IProperty>(pRecord->GetVariant(i)->getProperty()) );
		}
		return true ;
	}
	return false ;
}



/////////////////////////////// 单接口: 复接口的特例 /////////////////////////////////////

bool XProperty::getAttr( const iplChar * name1, ipl_float64 &value) const
{
	iplString name = ToRegularName( name1 );

	iplArray<ipl_float64> v;
	if (getAttr(name.c_str(),v)) {
		if (!v.size()) return false ;
		value = v[0] ;
		return true ;
	}
	return false ;
}


bool XProperty::getAttr(const iplChar * name1, ipl_int32 &value) const
{
	iplString name = ToRegularName( name1 );

	iplArray<ipl_int32> v;

	if( getAttr(name.c_str(),v) ) {
		if( 0 == v.size() )
			return false ;

		value = v[0];

		return true ;
	}
	return false ;
}



bool XProperty::getAttr( const iplChar * name1, ipl_int16 & value )  const
{
	iplString name = ToRegularName( name1 );

	iplArray<ipl_int16> v;
	
	if( getAttr(name.c_str(), v) ) {
		if( 0 == v.size() )
			return false ;
		
		value = v[0] ;

		return true ;
	}
	return false ;
}


bool XProperty::getAttr( const iplChar * name1, iplString &value )  const
{
//	printf("now in XProperty::getAttr\n");
	iplString name = ToRegularName( name1 );

//	printf("name: %s\n", name.c_str());

	PropertyRecord* pRecord;

	_recordConstIterator it = m_records.find( name ) ;

	if( it != m_records.end() ) {
		pRecord = it->second.get() ;

		switch( pRecord->GetVariant(0)->type ) {
		case IPL_V_STR:
			{
				const iplChar *str = pRecord->GetVariant(0)->getString();
				if(str != NULL)
					value = str;
			}
			break;
		case IPL_V_Attribute:
			{
				const iplChar *str = pRecord->GetVariant(0)->getString();
				if(str != NULL)
					value = str;
			}
			break;
		case IPL_V_BLOB:
			{
				ref_ptr<iplVariant> var = pRecord->GetVariant( 0 );

				ipl_int32 len;
				const ipl_byte *buf = var->getBlob( len );

				char * temp_str = new char[len+1];
				memcpy( temp_str, buf, len );

				temp_str[len] = '\0';

				value = temp_str;

				delete temp_str;
			}
		default:
			return false;
		}

		return true;
	}
	return false ;
}


bool XProperty::getAttr(const iplChar * name1, iplVectorBase &vector ) const
{
	iplString name = ToRegularName( name1 );

	PropertyRecord* pRecord;

	_recordConstIterator it = m_records.find( name ) ;

	if( it != m_records.end() ) {
		pRecord = it->second.get() ;

		switch( pRecord->GetVariant(0)->type ) {
		case IPL_V_VECTOR:
			pRecord->GetVariant(0)->getVector( vector );
			return true;
		}
	}

	return false;
}


bool XProperty::getAttr(const iplChar * name1, iplMatrixBase &matrix ) const
{
	iplString name = ToRegularName( name1 );

	PropertyRecord* pRecord;

	_recordConstIterator it = m_records.find( name ) ;

	if( it != m_records.end() ) {
		pRecord = it->second.get() ;

		switch( pRecord->GetVariant(0)->type ) {
		case IPL_V_MATRIX:
			pRecord->GetVariant(0)->getMatrix( matrix );
			return true;
		}
	}

	return false;
}

bool XProperty::getAttr(const iplChar * name1, const ipl_byte * &value, ipl_int32 &nLength)  const
{
	iplString name = ToRegularName( name1 );

	iplArray<const ipl_byte *> vChar;
	iplArray<ipl_int32> vLong ;

	if( getAttr( name.c_str(), vChar, vLong) )
	{
		if( 0 == vChar.size() )
			return false ;

		value = vChar[0] ;
		nLength = vLong[0] ;

		return true;
	}
	return false;
}

bool XProperty::getAttr(const iplChar * name1, ref_ptr<IProperty> &value) const
{
	iplString name = ToRegularName( name1 );

	iplArray< ref_ptr<IProperty> > v ;
	if( getAttr(name.c_str(), v) ) {
		if (!v.size()) return false ;
		value = v[0];
		return true ;
	}
	return false ;
}

//////////////////////////////////////////////////////////////////////////

void XProperty::addAttr(const iplChar * name1, ref_ptr<IProperty> value, bool bUnique )
{
	if( bUnique ) {
		if( setAttr( name1, value, 0 ) )
			return ;
	}

	iplString name = ToRegularName( name1 );
	
	_recordIterator pointer = m_records.find(name);
	
	PropertyRecord *pRecord = NULL;
	
	if(pointer == m_records.end()){
		pRecord = new PropertyRecord;
		m_records.insert(_recordType(name, ref_ptr<PropertyRecord>(pRecord)));
		m_attrNames.push_back( name );
	} else
		pRecord = pointer->second.get();
	
	ref_ptr<iplVariant>	var(new iplVariant);
	//iplVariant	*var = pRecord;
	
	var->type = IPL_V_CHILD ;
	var->set( value );
	
	pRecord->AddVariant( var );
}

extern IPlatform * _getPlatform();

ref_ptr<IProperty> XProperty::createChild(const iplChar * name1, bool bUnique )
{
	iplString name = ToRegularName( name1 );
	
	_recordIterator pointer = m_records.find(name);

	ref_ptr<IProperty> pChild;

	if( pointer == m_records.end() || !bUnique ) {
		pChild = ref_ptr<IProperty>(_getPlatform()->createProperty());
		addAttr( name1, pChild );
	} 
	else	{
		getAttr( name1, pChild );
	}

	return pChild;
}

void XProperty::addAttr(const iplChar * name1, ref_ptr<IObject> objPtr, bool bUnique )
{
	if( IPL_PTR_CAST( IProperty, objPtr ) ) {
		ref_ptr<IProperty> property = ipl_static_pointer_cast<IProperty>(objPtr);
		addAttr( name1, property, bUnique );
		return ;
	}

	if( bUnique ) {
		if( setAttr( name1, objPtr  ) )
			return ;
	}

	iplString name = ToRegularName( name1 );
	
	_recordIterator pointer = m_records.find(name);
	
	PropertyRecord *pRecord = NULL;
	
	if(pointer == m_records.end()){
		pRecord = new PropertyRecord;
		m_records.insert(_recordType(name, ref_ptr<PropertyRecord>(pRecord)));
		m_attrNames.push_back( name );
	} else
		pRecord = pointer->second.get();
	
	ref_ptr<iplVariant>	var(new iplVariant);
	//iplVariant	*var = pRecord;
	
	var->type = IPL_V_OBJECT ;
	var->set( objPtr );
	
	pRecord->AddVariant( var );	
}

bool XProperty::setAttr(const iplChar * name1, ref_ptr<IObject> objPtr )
{
	iplString name = ToRegularName( name1 );
	
	_recordIterator iter;
	
	iter = m_records.find(name);
	
	if( iter == m_records.end() )
		return false;
	
	PropertyRecord *pRecord = iter->second.get();
	
	ref_ptr<iplVariant> var = pRecord->GetVariant( 0 );
	
	if( var->type == IPL_V_OBJECT )
	{
		var->set( objPtr );
	}
	else
		return false;
		
	return true;
}



bool
XProperty::getAttr(const iplChar * name1, ref_ptr<IObject> &value ) const
{
	iplString name = ToRegularName( name1 );
	
	PropertyRecord* pRecord;
	_recordConstIterator it = m_records.find(name) ;
	
	if (it != m_records.end()) {
		pRecord = it->second.get() ;
		for (unsigned i = 0 ; i < pRecord->size() ; i++)
		{
			if( IPL_V_OBJECT == pRecord->GetVariant(i)->type )	{
				value = pRecord->GetVariant(i)->getObject();
				break;
			}
		}
		return true ;
	}
	return false ;
}


//////////////////////////////////////////////////////////////////////////

void XProperty::addAttr(const iplChar * name1, ref_ptr<PropertyListener> objPtr, bool bUnique )
{
	if( bUnique ) {
		if( setAttr( name1, objPtr ) )
			return ;
	}

	iplString name = ToRegularName( name1 );
	
	_recordIterator pointer = m_records.find(name);
	
	PropertyRecord *pRecord = NULL;
	
	if(pointer == m_records.end()){
		pRecord = new PropertyRecord;
		m_records.insert(_recordType(name, ref_ptr<PropertyRecord>(pRecord)));
		m_attrNames.push_back( name );
	} else
		pRecord = pointer->second.get();
	
	ref_ptr<iplVariant>	var(new iplVariant);
	//iplVariant	*var = pRecord;
	
	var->type = IPL_V_LISTNER ;
	var->set( objPtr );
	
	pRecord->AddVariant( var );	
}

bool XProperty::setAttr(const iplChar * name1, ref_ptr<PropertyListener> objPtr )
{
	iplString name = ToRegularName( name1 );
	
	_recordIterator iter;
	
	iter = m_records.find(name);
	
	if( iter == m_records.end() )
		return false;
	
	PropertyRecord *pRecord = iter->second.get();
	
	// 	if( index >= pRecord->size() )
	// 		return false;
	
	ref_ptr<iplVariant> var = pRecord->GetVariant( 0 );
	
	if( var->type == IPL_V_LISTNER )
	{
		var->set( objPtr );
	}
	else
		return false;
	
	return true;
}



bool
XProperty::getAttr(const iplChar * name1, ref_ptr<PropertyListener> &value ) const
{
	iplString name = ToRegularName( name1 );
	
	PropertyRecord* pRecord;
	_recordConstIterator it = m_records.find(name) ;
	
	if (it != m_records.end()) {
		pRecord = it->second.get() ;
		for (unsigned i = 0 ; i < pRecord->size() ; i++)
		{
			if( IPL_V_LISTNER == pRecord->GetVariant(i)->type )	{
				value = pRecord->GetVariant(i)->getPropListener();
				break;
			}
		}
		return true ;
	}
	return false ;
}


//////////////////////////////////////////////////////////////////////////

void XProperty::copy( const ref_ptr<IProperty> pProperty )
{
	if(pProperty.get() == NULL)
		return ;
	int i;
	
	iplString name;
	IPLVariantType type;
	ipl_int32 valuenum;

	for( i=0; i<pProperty->size(); i++ )
	{
		pProperty->getAttributeInfo(i, name, type, valuenum);
		
		switch(type){
		case IPL_V_BOOL:
			{
				bool value;
				pProperty->getAttr(name.c_str(), value );
				
				addAttr( name.c_str(), value );
			}
			break;
		case IPL_V_I2:
			{
				iplArray<ipl_int16> values;
				pProperty->getAttr(name.c_str(), values);
				
				for (int j=0;j<values.size();j++)
					addAttr( name.c_str(), values[j] );
			}
			break;
		case IPL_V_I4:
			{
				iplArray<ipl_int32> values;
				
				pProperty->getAttr(name.c_str(), values);
				
				for (int j=0;j<values.size();j++)
					addAttr( name.c_str(), values[j]);
			}
			break;
		case IPL_V_R8:
			{
				iplArray<ipl_float64> values;
				
				pProperty->getAttr(name.c_str(), values);
				
				for (int j=0;j<values.size();j++)
					addAttr( name.c_str(), values[j] );
			}
			break;
		case IPL_V_STR:
			{
				iplArray<iplString> values;
				
				pProperty->getAttr(name.c_str(), values);
				
				for (int j=0;j<values.size();j++)
					addAttr( name.c_str(), values[j].c_str());
			}
			break;
		case IPL_V_BLOB:
			{
				iplArray<const ipl_byte *> values;
				iplArray<ipl_int32> nLengths;
				
				pProperty->getAttr(name.c_str(), values,nLengths);
				
				for (int j=0;j<values.size();j++)
				{
					addAttr( name.c_str(), values[j], nLengths[j] );
				}
			}
			break;
		case IPL_V_CHILD:
			{
				iplArray<ref_ptr<IProperty> > children;
				pProperty->getAttr(name.c_str(), children );
				
				for (int j=0; j<children.size();j++)
				{
					ref_ptr<IProperty> newChild(_getPlatform()->createProperty());

					newChild->copy(  children[j] );
					addAttr( name.c_str(), newChild );
				}
			}
			break;

		case IPL_V_MATRIX:
			{
				iplMatrixD value;
				
				pProperty->getAttr(name.c_str(), value );
				
				addAttr( name.c_str(), value );
			}
			break;
		case IPL_V_VECTOR:
			{
				iplVectorD value;
				
				pProperty->getAttr(name.c_str(), value );
				
				addAttr( name.c_str(), value );
			}
			break;
		case IPL_V_LISTNER:
			{
// 				PropertyListener *pListner;
// 				
// 				pProperty->getAttr(name.c_str(), pListner );
// 				
// 				addAttr( name, pListner );
			}
			break;
		case IPL_V_OBJECT:
			{
				ref_ptr<IObject> value;
				
				pProperty->getAttr(name.c_str(), value );
				
				addAttr( name.c_str(), value );
			}
		default:
			assert( false ) ;
		}
	}
}


//////////////////////////////////////////////////////////////////////////


// 只更新自己有的
void XProperty::update( const ref_ptr<IProperty> pProperty )
{
	if(pProperty.get() == NULL)
		return ;
	int i;

	iplString name;
	IPLVariantType type;
	ipl_int32 valuenum;

	for( i=0; i<size(); i++ )
	{
		getAttributeInfo(i, name, type, valuenum);
		
		switch(type){
		case IPL_V_BOOL:
			{
				bool value;
				
				if( pProperty->getAttr(name.c_str(), value ) )
					setAttr( name.c_str(), value );
			}
			break;
		case IPL_V_I2:
			{
 				iplArray<ipl_int16> values;
 				pProperty->getAttr(name.c_str(), values);

				for (int j=0;j<values.size();j++)
					setAttr( name.c_str(), values[j], j );
			}
			break;
		case IPL_V_I4:
			{
				iplArray<ipl_int32> values;
				
				pProperty->getAttr(name.c_str(), values);
				
				for (int j=0;j<values.size();j++)
					setAttr( name.c_str(), values[j], j );
			}
			break;
		case IPL_V_R8:
			{
 				iplArray<ipl_float64> values;

				pProperty->getAttr(name.c_str(), values);
				
				for (int j=0;j<values.size();j++)
					setAttr( name.c_str(), values[j], j );
			}
			break;
		case IPL_V_STR:
			{
				iplArray<iplString> values;

				pProperty->getAttr(name.c_str(), values);
				
				for (int j=0;j<values.size();j++)
					setAttr( name.c_str(), values[j].c_str(), j );
			}
			break;
 		case IPL_V_BLOB:
 			{
 				iplArray<const ipl_byte *> values;
 				iplArray<ipl_int32> nLengths;

 				pProperty->getAttr(name.c_str(), values,nLengths);

 				for (int j=0;j<values.size();j++)
 				{
					setAttr( name.c_str(), values[j], nLengths[j], j );
 				}
 			}
 			break;
 		case IPL_V_CHILD:
			{
				ref_ptr<IProperty> theProp;
				ref_ptr<IProperty> inProp;

				pProperty->getAttr( name.c_str(), inProp );

				if( inProp.get() )	{
					getAttr( name.c_str(), theProp );
					theProp->update( inProp );
				}
			}
			break;
		case IPL_V_MATRIX:
			{
				iplMatrixD value;
				
				if( pProperty->getAttr(name.c_str(), value ) )
					setAttr( name.c_str(), value );
			}
			break;
		case IPL_V_VECTOR:
			{
				iplVectorD value;
				
				if( pProperty->getAttr(name.c_str(), value ) )
					setAttr( name.c_str(), value );
			}
			break;
		case IPL_V_OBJECT:
			{
				ref_ptr<IObject> pObj;
					
				pProperty->getAttr( name.c_str(), pObj );
							
				setAttr( name.c_str(), pObj );
			}
			break;
		case IPL_V_LISTNER:
// 			{
// 				PropertyListener *pListner;
// 				
// 				pProperty->getAttr(name.c_str(), pListner );
// 				
// 				setAttr( name, pListner );
// 			}
			break;
			
		default:
			assert( false ) ;
		}
	}

}



//////////////////////////////////////////////////////////////////////////
void XProperty::getAttributeInfo( ipl_uint32 index, iplString &name, IPLVariantType &type,
									ipl_int32 &numOfValues, bool bSort ) const
{
	assert( index < size() && index >=0 );

	if( !bSort )	{
		name = m_attrNames[index].c_str();
		
		_recordConstIterator iter = m_records.find( name.c_str() ) ;
		if( iter != m_records.end() ) {
			ref_ptr<iplVariant> var = iter->second->GetVariant( 0 );
			for (int i=0;i<iter->second->size();i++)
			{
				ref_ptr<iplVariant> vv = iter->second->GetVariant(i);
				int ee;
				ee = 3;
			}
			type = var->type;
			numOfValues = iter->second->size();
		}
		else
			assert( false );
	}
	else	{
		_recordConstIterator iter = m_records.begin();
		 
		for( int i=0; i<index; i++)
			iter ++;
		 
		name = iter->first.c_str();
		
		ref_ptr<iplVariant> var = iter->second->GetVariant( 0 );
		
		type = var->type;
		
		numOfValues = iter->second->size();
	}
}


IPLVariantType XProperty::getAttrType( const iplChar *attrName ) const
{
	_recordConstIterator iter = m_records.find( attrName ) ;
	
	if( iter != m_records.end() ) {
		ref_ptr<iplVariant> var = iter->second->GetVariant( 0 );
		return var->type;
	}

	return IPL_V_UNKOWN;
}
bool XProperty::findAttrName(const iplChar * name)//判断是否存在节点name
{
	iplString name0 = ToRegularName( name );
	_recordConstIterator it = m_records.find( name ) ;
	if( it != m_records.end() ) return true;
	else return false;
}

//#include "orsBase/iplString.h"
ref_ptr<IProperty> XProperty::findNode( const iplChar *nodePath, ref_ptr<IProperty> &pParent )
{
	assert( nodePath[0] == '/' );	//&& nodePath[1] == '/'  );

	if( nodePath[1] == '/' )
		nodePath += strlen( "//" );
	else
		nodePath += strlen( "/" );

	IProperty *pNode = this;

	ref_ptr<IProperty> pNodeMatch = NULL;

	do 	{
		iplString node = nodePath;
		
		size_t pos = node.find( '/' );
		iplString key;

		if( -1 == pos )		// 最后一个了
			key = node;
		else
			key = node.substr(0, pos);
		
		nodePath += pos + 1;

		if( *nodePath == '/' )
			nodePath++;
		
		pParent = pNodeMatch;
		pNodeMatch.reset();

		int i;
		for( i=0; i< pNode->size(); i++ )
		{
			iplString name;
			IPLVariantType type;
			ipl_int32 numOfValues = 0;
			
			pNode->getAttributeInfo( i, name, type, numOfValues );
			
			if( name == key ) {
				if( IPL_V_CHILD == type )
					pNode->getAttr( name.c_str(), pNodeMatch );

				break;
			}
		}

		if( NULL == pNodeMatch.get() )
			return NULL;

		pNode = pNodeMatch.get();

		if( -1 == pos || 0 == nodePath[0] )
			break;

	} while (1);

	return pNodeMatch;
}