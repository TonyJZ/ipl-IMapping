#pragma once

#include <string>

#include "core/interface/IProperty.h"
#include "core/interface/IPlatform.h"
#include "core/iplmacro.h"

class TiXmlNode;  //define in tinyXML

namespace ipl
{
	class /*ORS_BASE_API*/ XXMLSerialize : public IPropertySerialize
	{
	private:
		bool m_bFromUTF8;

	private:

		void AddAttr_UTF8(IProperty *pProp, const char *name, const char *pStr);
		//添加真实的属性，如：<task name="ImageGeoQualityCheck">
		void AddAttribute_UTF8(IProperty *pProp, const char *name, const char *pStr);

	public:
		virtual bool import(const char *bytes, long length, IProperty *info);
		virtual bool outport(char *&bytes, long *length, const IProperty *info);
		virtual bool outport_noUTF_8Head(char *&bytes, long *length, const IProperty *info);

		virtual bool free(char *&bytes);

	private:
		int readXmlNode(TiXmlNode * pNode, IProperty *record);
		int writeXmlNode(TiXmlNode * pNode, const IProperty *record);

	public:	
		//接口宏定义
		IPL_OBJECT_IMP1(XXMLSerialize, IPropertySerialize, _T("xml"), _T("xmlSerializer"))
	};

}
