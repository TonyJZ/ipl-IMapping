/********************************************************************
	purpose:
				属性序列化

  BOM（Byte Order Mark）是一个字符，它表明UNICODE文本的UTF-16,UTF-32的编码字节顺序（高字节低字节顺序）和编码方式
  Encoding Representation 
		UTF-8 EF BB BF 
		UTF-16 Big Endian FE FF 
		UTF-16 Little Endian FF FE 
		UTF-32 Big Endian 00 00 FE FF 
		UTF-32 Little Endian FF FE 00 00
	
  其中UTF-8编码是字节顺序无关的

  有些utf8编码没有这个BOM,该怎么区分了,是utf8还是ansi(根本就没有BOM这个东西),下面先了解下utf8:
  UTF-8是UNICODE的一种变长字符编码，由Ken Thompson于1992年创建。现在已经标准化为RFC 3629。UTF-8用1到6个字节编码UNICODE字符。
  如果UNICODE字符由2个字节表示，则编码成UTF-8很可能需要3个字节，而如果UNICODE字符由4个字节表示，则编码成UTF-8可能需要6个字节。
*********************************************************************/

//#include <string>
#ifdef WIN32

//#include "warningdisable.h"

//#include "targetver.h"

#define WIN32_LEAN_AND_MEAN             // Exclude rarely-used stuff from Windows headers
// Windows Header Files:
#include <windows.h>

#include <winsock2.h>

#endif

// #include "iplBase/IProperty.h"
#include "iplCommon.h"

#include "XXMLSerialize.h"
#include "XProperty.h"

#include "tinyxml/tinyxml.h"

#ifdef IPL_PLATFORM_WINDOWS

std::string UTF8ToGBK(const std::string& strUTF8)
{
	int len = MultiByteToWideChar(CP_UTF8, 0, strUTF8.c_str(), -1, NULL, 0);
	unsigned short * wszGBK = new unsigned short[len + 1];
	memset(wszGBK, 0, len * 2 + 2);
	MultiByteToWideChar(CP_UTF8, 0, (LPCTSTR)strUTF8.c_str(), -1, (LPWSTR)wszGBK, len);

	len = WideCharToMultiByte(CP_ACP, 0,(LPCWSTR)wszGBK, -1, NULL, 0, NULL, NULL);
	char *szGBK = new char[len + 1];
	memset(szGBK, 0, len + 1);
	WideCharToMultiByte(CP_ACP,0, (LPCWSTR)wszGBK, -1, szGBK, len, NULL, NULL);
	
	//strUTF8 = szGBK;

	std::string strTemp(szGBK);
	
	delete[]szGBK;
	delete[]wszGBK;

	return strTemp;
}


std::string GBKToUTF8(const std::string& mbcsStr)
{
	wchar_t*  wideStr; 
	char*   utf8Str; 
	int   charLen;
	
	charLen = MultiByteToWideChar(936, 0, mbcsStr.c_str(), -1, NULL, 0);    ////////936 ----- gb2312
	wideStr = (wchar_t*) malloc(sizeof(wchar_t)*charLen); 

	MultiByteToWideChar(936, 0, mbcsStr.c_str(), -1, wideStr, charLen);
	
	charLen = WideCharToMultiByte(CP_UTF8, 0, wideStr, -1, NULL, 0, NULL, NULL);
	
	utf8Str = (char*)malloc(charLen);
	
	WideCharToMultiByte(CP_UTF8, 0, wideStr, -1, utf8Str, charLen, NULL, NULL);
	
	std::string strTemp( utf8Str );

	free( wideStr );
	free( utf8Str );

	return strTemp;	
} 

#else


#include <iconv.h>

std::string UTF8ToGBK(const std::string& strUTF8)
{
	iconv_t cd = iconv_open( "GBK", "UTF-8");

	const char *pin = strUTF8.c_str();
	int inlen = strlen(pin);

	size_t outlen = inlen *4;

	char *outbuf = (char *)malloc( outlen );

	bzero( outbuf, outlen );

	char *in = (char *)pin;
	char *out = outbuf;

	// 注意in和out会变化
	
	iconv( cd, &in, (size_t *)&inlen, &out, &outlen );

	std::string strTemp( outbuf );

	free(outbuf);

	iconv_close(cd);

	return strTemp;
}


std::string GBKToUTF8(const std::string& strGBK )
{
	iconv_t cd = iconv_open("UTF-8",  "GBK");
	
	const char *pin = strGBK.c_str();
	int inlen = strlen(pin);
	
	size_t outlen = inlen *4;
	
	char *outbuf = (char *)malloc( outlen );
	
	bzero( outbuf, outlen );
	
	char *in = (char *)pin;
	char *out = outbuf;
	
	// 注意in和out会变化
	
	iconv( cd, &in, (size_t *)&inlen, &out, &outlen );
	
	std::string strTemp( outbuf );
	
	free(outbuf);
	
	iconv_close(cd);
	
	return strTemp;
}


#endif

using namespace ipl;

bool XXMLSerialize::import( const char *bytes,long length, IProperty *info)
{
	TiXmlDocument xmlDoc;

	unsigned char *pBytes = (unsigned char *)bytes;

	if( strstr(bytes, "UTF-8" ) || ( 0xEF == pBytes[0] && 0xBB == pBytes[1] && 0xBF == pBytes[2] ) )	
	{
		m_bFromUTF8 = true;

/*		std::string gbkStr;
		
		// UTF_8 has BOM?: EF BB BF				
		// BOM:
		if( 0xEF == pBytes[0] && 0xBB == pBytes[1] && 0xBF == pBytes[2] )
			gbkStr = UTF8ToGBK( bytes + 3);	// skip BOM: 
		else
			gbkStr = UTF8ToGBK( bytes );

		xmlDoc.Parse( gbkStr.c_str() ) ;	
*/
	}
	else	{
		m_bFromUTF8 = false;
//		xmlDoc.Parse( bytes ) ;
	}

	xmlDoc.Parse( bytes ) ;

	readXmlNode( &xmlDoc, info );

	return true ;
}


bool XXMLSerialize::outport(char *&bytes,long *length, const IProperty *info)
{
	m_bFromUTF8 = true;

	TiXmlDocument xmlDoc;

	writeXmlNode(&xmlDoc,info);

	TiXmlPrinter printer;

	printer.SetIndent( "\t" );
	xmlDoc.Accept( &printer );
	const char* charXmlDoc = printer.CStr();
	*length =   strlen(charXmlDoc) ;

	const char *pHead = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";

	int headLen = strlen(pHead);

	bytes = new char[ headLen + *length + 1];
	strcpy( bytes, pHead);
	memcpy (bytes+ headLen, charXmlDoc, *length) ;

	*length += headLen;

	bytes[*length] = '\0';

	return true ;
}

bool XXMLSerialize::outport_noUTF_8Head(char *&bytes,long *length, const IProperty *info)
{
	m_bFromUTF8 = true;

	TiXmlDocument xmlDoc;

	writeXmlNode(&xmlDoc,info);

	TiXmlPrinter printer;

	printer.SetIndent( "\t" );
	xmlDoc.Accept( &printer );
	const char* charXmlDoc = printer.CStr();
	*length =   strlen(charXmlDoc) ;

	bytes = new char[ *length + 1];
	memcpy (bytes, charXmlDoc, *length) ;

	bytes[*length] = '\0';

	return true ;
}

bool XXMLSerialize::free(char *&bytes)
{
	delete bytes;

	return true;
}


// static char *NodeTypes[] =
// {
// 	"TINYXML_DOCUMENT",
// 		"TINYXML_ELEMENT",
// 		"TINYXML_COMMENT",
// 		"TINYXML_UNKNOWN",
// 		"TINYXML_TEXT",
// 		"TINYXML_DECLARATION",
// 		"TINYXML_TYPECOUNT"
// };




bool HasElementChildren( TiXmlNode * pNode )
{
	TiXmlNode *pChildNode = pNode->FirstChild();

	while( pChildNode )	{
		if( TiXmlNode::TINYXML_ELEMENT == pChildNode->Type() )
			return true;

		pChildNode = pChildNode->NextSibling();
	}

	return false;
}

void XXMLSerialize::AddAttr_UTF8( ipl::IProperty *pProp, const char *name, const char *pStr )
{
	if( NULL == pStr || strlen(pStr)==0)//当为空时，直接添加， by whx
	{
		pProp->addAttr( name,pStr); 
		return ;
	}

	if( m_bFromUTF8 )	{
		std::string gbk = UTF8ToGBK( pStr );
		pProp->addAttr( name, gbk.c_str() );
	}
	else 
		pProp->addAttr( name, pStr );
}

void XXMLSerialize::AddAttribute_UTF8( IProperty *pProp, const char *name, const char *pStr )
{
	if( NULL == pStr || strlen(pStr)==0 ) //当pStr为“”时，UTF8ToGBK函数会出错，所以这里进行判断，by whx，2017.1.25
	{
		pProp->addAttribute( name, pStr );
		return ;
	}

	if( m_bFromUTF8 )	{
		std::string gbk = UTF8ToGBK( pStr );
		pProp->addAttribute( name, gbk.c_str() );
	}
	else 
		pProp->addAttribute( name, pStr );
}

int XXMLSerialize::readXmlNode( TiXmlNode * pNode, IProperty *record)
{
	TiXmlElement* xmlElement = 0 ;
	const char*	key;
	const char*	type;
	const char* text;
	
	while (pNode) {
		
		int nodeType = pNode->Type();
		
//		printf("Node type : %s, value = %s\n", NodeTypes[nodeType], pNode->Value() );

		switch( nodeType )
		{
		case TiXmlNode::TINYXML_DOCUMENT:
			readXmlNode( pNode->FirstChild(), record );
			return 0;
			
		case TiXmlNode::TINYXML_COMMENT:
			{
				TiXmlComment *xmlComment = pNode->ToComment();
				const char *value = xmlComment->Value();
			}
			break;
		case TiXmlNode::TINYXML_UNKNOWN:
			break;
		case TiXmlNode::TINYXML_TEXT:
			{
				TiXmlText *xmlText = pNode->ToText();
				const char *value = xmlText->Value();

				// never go here, 出错了，
				assert(false);

// 				char *end = strstr( value, "</");
// 
// 				if( NULL != end ) {
// 					end[strlen( end )-1] = 0;
// 					
// 					*end = 0;
// 					
// 					key = end + 2;
// 					
// 					record->addAttr( key, value );
// 				}
			}
			break;
		case TiXmlNode::TINYXML_DECLARATION:
			{
				TiXmlDeclaration *xmlDeclr = pNode->ToDeclaration();
				const char *value = xmlDeclr->Value();
			}
			break;			
		case TiXmlNode::TINYXML_ELEMENT:
			if( HasElementChildren( pNode )  )	{

				// 嵌套子节点
				xmlElement = pNode->ToElement();
				key = xmlElement->Value();
			
				/////////////////////////////////////////

				ref_ptr<IProperty> children = ref_ptr<IProperty>(new XProperty);
				
				// 按第一个传递，会自动遍历兄弟
				int nErr = readXmlNode( pNode->FirstChild(), children.get() );
				
				if( nErr == 1 )
					return nErr;
				
				///////////////////////////////////
				// 允许有attribute，转换为子属性
				const TiXmlAttribute *attr;
				
				attr = xmlElement->FirstAttribute();
				
				while( attr )	{
					// 假定都是字符串
					// children->addAttr( attr->Name(), attr->Value() );
					AddAttribute_UTF8( children.get(), attr->Name(), attr->Value() );
					
					attr = attr->Next();
				}

				//////////////////////////////////
				record->addAttr( key, children );	
			}
			else	{
				//叶节点
				xmlElement = pNode->ToElement();
				key = xmlElement->Value();
				text = xmlElement->GetText();
				type = xmlElement->Attribute("type");
				
				// 无类型, 按照字符串存储
				if( type == NULL) {
					// 分有attribute和无attribute 2种情况
					if( NULL != xmlElement->FirstAttribute() )	{
						
						ref_ptr<IProperty> child_value = ref_ptr<IProperty>(new XProperty);
						
						// 看有没有attribute, 作为child属性来实现
						const TiXmlAttribute *attr;
						
						attr = xmlElement->FirstAttribute();
						
						while( attr )	{
							// 假定都是字符串
							if( NULL != attr->Value() && iplStrLen( attr->Value() ) > 0 )	
							{
								//child_value->addAttr( attr->Name(), attr->Value() );
								AddAttribute_UTF8( child_value.get(), attr->Name(), attr->Value() );
							}
							attr = attr->Next();
						}
						
						// 如果有文本，改为value保存
						//if( NULL != text && strlen(text) > 0 )
						//{
							//child_value->addAttr( _T("value"), text ) ;
							AddAttr_UTF8( child_value.get(),  _T("value"), text );
						//}

						record->addAttr( key, child_value );
					}
					else	{
						//if( NULL != text && strlen(text) > 0 )
						//{
							//record->addAttr(key,text);
							AddAttr_UTF8( record, key, text );
						//}
					}
				}
				else	{
					//根据节点数据类型读取
					if (0==stricmp(type,"bool")) {
						char buf[32];
						sscanf(text,"%s", buf);
						
						bool value;
						
						if( 0 == _tcsicmp( buf, "true") )
							value = true;
						else
							value = false;
						
						record->addAttr(key,value);
					}
					else if (0==stricmp(type,"int16")) {
						int value;
						sscanf(text,"%d",&value);
						record->addAttr(key,(ipl_int16)value);
					}
					else if (0==stricmp(type,"int32")){
						ipl_int32 value;
						sscanf(text,"%d",&value);
						record->addAttr(key,value);
					}
					else if (0==stricmp(type,"float64")) {
						ipl_float64 value;
						sscanf(text,"%lf",&value);
						record->addAttr(key,value);
					}
					else if (0==stricmp(type,"string")) 
					{
						//record->addAttr( key, text );
						AddAttr_UTF8( record, key, text );
					}
					else if( 0==stricmp(type,"blob") || 0==stricmp(type,"vector") || 0==stricmp(type,"matrix") ) {
						ipl_byte * val = NULL;
						ipl_int32 len = 0;
						
						// 如果后面没有内容，则忽略
						if( NULL != text )	{
							int v_len = iplBase64::getDataLength(strlen(text));
							unsigned char* data = new unsigned char[ v_len ];
						
							int realLen = iplBase64::decode(text, strlen(text), data );
							
							if( 0==stricmp(type,"blob") )
								record->addAttr(key,data,v_len);
							else if( 0==stricmp(type,"vector") ) {
								ipl_int32 *pInfo = (ipl_int32 *)data;
								iplDataTYPE type = (iplDataTYPE)pInfo[0];
								ipl_int32 nRows =pInfo[1];
								
								int headBytes = 2*sizeof( ipl_int32 );
								int dataBytes = nRows*iplGetSizeOfType( type );
								
								switch( type)	{
								case IPL_DT_FLOAT32:
									{
										iplVector<ipl_float32>	vector;
										vector.Alloc( nRows );
										memcpy( vector.Buf(), data+headBytes, dataBytes );
										
										record->addAttr(key, vector );
									}
									break;
								case IPL_DT_FLOAT64:
									{
										iplVector<ipl_float64> vector;
										vector.Alloc( nRows );
										memcpy( vector.Buf(), data+headBytes, dataBytes );
										
										record->addAttr(key, vector );
									}
									break;
								default:
									assert( false );
								}
							}
							else if( 0==stricmp(type,"matrix") ) {
								ipl_int32 *pInfo = (ipl_int32 *)data;
								iplDataTYPE type = (iplDataTYPE)pInfo[0];
								ipl_int32 nRows =pInfo[1];
								ipl_int32 nCols =pInfo[2];
								
								int headBytes = 3*sizeof( ipl_int32 );
								int dataBytes = nRows*nCols*iplGetSizeOfType( type );
								
								switch( type)	{
								case IPL_DT_FLOAT32:
									{
										iplMatrix<ipl_float32>	matrix;
										matrix.Alloc( nRows, nCols );
										memcpy( matrix.Buf(), data+headBytes, dataBytes );
										
										record->addAttr(key, matrix );
									}
									break;
								case IPL_DT_FLOAT64:
									{
										iplMatrix<ipl_float64>matrix;
										matrix.Alloc( nRows, nCols );
										memcpy( matrix.Buf(), data+headBytes, dataBytes );
										record->addAttr(key, matrix );
									}
									break;
								default:
									assert( false );
								}
							}						
							delete []data;
						}
					}
				}
			}
			break;
		}

		// 遍历兄弟
		pNode = pNode->NextSibling();
	}

	return 0 ;
}

int XXMLSerialize::writeXmlNode(TiXmlNode * pNode, const IProperty *record)
{
	unsigned int i=0;
	for (i=0;i<record->size();i++)
	{
		iplString name;
		IPLVariantType type;
		ipl_int32 valuenum;
		record->getAttributeInfo(i,name,type,valuenum, false);

		switch(type){
		case IPL_V_BOOL:
			{
				bool value;
				record->getAttr(name.c_str(), value);

				TiXmlElement* pChildeElm = new TiXmlElement(name.c_str()) ;
				pChildeElm->SetAttribute("type", "bool");

				char text[32];
				memset(text,0,32);

				if( value )
					sprintf(text,"true");
				else
					sprintf(text,"false");

				TiXmlText* pText = new TiXmlText(text);
				pChildeElm->InsertEndChild(*pText);
				pNode->InsertEndChild(*pChildeElm);

				delete pChildeElm;
				delete pText;
			}
			break;
		case IPL_V_I2:
			{
				iplArray<ipl_int16> values;
				record->getAttr(name.c_str(), values);
				for (unsigned j=0;j<values.size();j++)
				{
					TiXmlElement* pChildeElm = new TiXmlElement(name.c_str()) ;
					pChildeElm->SetAttribute("type", "int16");
					ipl_int16 i_value = values[j];
					char text[32];
					memset(text,0,32);
					sprintf(text,"%d",i_value);
					TiXmlText* pText = new TiXmlText(text);
					pChildeElm->InsertEndChild(*pText);
					pNode->InsertEndChild(*pChildeElm);
					delete pChildeElm;
					delete pText;
				}
			}
			break;
		case IPL_V_I4:
			{
				iplArray<ipl_int32> values;
				record->getAttr(name.c_str(), values);
				for (unsigned j=0;j<values.size();j++)
				{
					TiXmlElement* pChildeElm = new TiXmlElement(name.c_str()) ;
					pChildeElm->SetAttribute("type", "int32");
					ipl_int32 i_value = values[j];
					char text[32];
					memset(text,0,32);
					sprintf(text,"%d",i_value);
					TiXmlText* pText = new TiXmlText(text);
					pChildeElm->InsertEndChild(*pText);
					pNode->InsertEndChild(*pChildeElm);
					delete pChildeElm;
					delete pText;
				}
			}
			break;
		case IPL_V_R8:
			{
				iplArray<ipl_float64> values;
				record->getAttr(name.c_str(), values);
				for (unsigned j=0;j<values.size();j++)
				{
					TiXmlElement* pChildeElm = new TiXmlElement(name.c_str()) ;
					pChildeElm->SetAttribute("type", "float64");
					ipl_float64 i_value = values[j];
					char text[200];
					memset(text,0,200);
					sprintf(text,"%.8lf",i_value);
					TiXmlText* pText = new TiXmlText(text);
					pChildeElm->InsertEndChild(*pText);
					pNode->InsertEndChild(*pChildeElm);
					delete pChildeElm;
					delete pText;
				}
			}
			break;
		case IPL_V_STR:
			{
				iplArray<iplString> values;
				
				record->getAttr(name.c_str(), values);

				for (unsigned j=0;j<values.size();j++)
				{
					if( !values[j].empty() )	{
						TiXmlElement* pChildeElm = new TiXmlElement(name.c_str()) ;

						//默认就是string, 减少空间浪费；还可和其它软件兼容
						//pChildeElm->SetAttribute("type", "string");	
						
						std::string s_value = GBKToUTF8( values[j].c_str() );
						
						TiXmlText* pText = NULL;
						
						if( 0 == s_value.length() )
						{
							pText = new TiXmlText("");
						}
						else
							pText = new TiXmlText(s_value.c_str());
						
						pChildeElm->InsertEndChild(*pText);
						pNode->InsertEndChild(*pChildeElm);
						
						delete pChildeElm;
						delete pText;						
					}
					else //当value为空时，by whx,2016/01/21
					{
						TiXmlElement* pChildeElm = new TiXmlElement(name.c_str()) ;

						TiXmlText* pText = NULL;

						pText = new TiXmlText("");

						pChildeElm->InsertEndChild(*pText);
						pNode->InsertEndChild(*pChildeElm);

						delete pChildeElm;
						delete pText;
					}
				}
			}
			break;
		case IPL_V_BLOB:
			{
				iplArray<const ipl_byte *> values;
				iplArray<ipl_int32> nLengths;
				record->getAttr(name.c_str(), values,nLengths);

				for (unsigned j=0;j<values.size();j++)
				{
					TiXmlElement* pChildeElm = new TiXmlElement(name.c_str()) ;
					pChildeElm->SetAttribute("type", "blob");
					const ipl_byte *val = values[j];
					ipl_int32 len = nLengths[j];
					std::string blob_str;
					iplBase64::encode(val,len,blob_str);
					TiXmlText* pText = new TiXmlText(blob_str.c_str());
					pChildeElm->InsertEndChild(*pText);
					pNode->InsertEndChild(*pChildeElm);
					delete pChildeElm;
					delete pText;
				}
			}
			break;
		case IPL_V_VECTOR:
		case IPL_V_MATRIX:
			{
				const ipl_byte *value;
				ipl_int32 nLength;

				record->getAttr(name.c_str(), value, nLength);

				TiXmlElement* pChildeElm = new TiXmlElement(name.c_str()) ;

				if( IPL_V_VECTOR == type )
					pChildeElm->SetAttribute("type", "vector");
				else
					pChildeElm->SetAttribute("type", "matrix");

				std::string blob_str;
				iplBase64::encode(value, nLength, blob_str);

				TiXmlText* pText = new TiXmlText( blob_str.c_str() );

				pChildeElm->InsertEndChild(*pText);
				pNode->InsertEndChild(*pChildeElm);

				delete pChildeElm;
				delete pText;
			}
			break;

		case IPL_V_CHILD:
			{
				iplArray<ref_ptr<IProperty> > children;
				record->getAttr(name.c_str(), children );
				for (unsigned i=0;i<children.size();i++)
				{
					TiXmlElement* pChildeElm = new TiXmlElement(name.c_str()) ;
					TiXmlNode *pChildNode = pNode->InsertEndChild(*pChildeElm);

					writeXmlNode(pChildNode, children[i].get() );

					delete pChildeElm;
				}
			}
			break;
		case IPL_V_LISTNER:
			break;
		case IPL_V_OBJECT:
			{
				ref_ptr<IObject> obj;
				record->getAttr(name.c_str(),obj);
				
				TiXmlElement* pChildeElm = new TiXmlElement(name.c_str()) ;
				pChildeElm->SetAttribute("type", "object");
				iplString s_value;
				TiXmlText* pText = NULL;
				
				if( s_value.empty() )
				{
					pText = new TiXmlText("");
				}
				else
					pText = new TiXmlText(s_value.c_str());
				
				pChildeElm->InsertEndChild(*pText);
				pNode->InsertEndChild(*pChildeElm);
				delete pChildeElm;
				delete pText;
			}
			break;
		case IPL_V_Attribute:
			{
				iplArray<iplString> values;

				record->getAttr(name.c_str(), values);

				for (unsigned j=0;j<values.size();j++)
				{
					if( !values[j].empty() )	{
						TiXmlElement *xmlElement = pNode->ToElement();

						//默认就是string, 减少空间浪费；还可和其它软件兼容
						xmlElement->SetAttribute(name.c_str(), values[j].c_str());	

					}
				}
			}
			break;
		default:
			//assert( false );

			break ;
		}
	}

	return 0 ;
}
