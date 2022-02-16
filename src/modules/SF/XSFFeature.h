#pragma once



#include "geometry/interface/IGeoObject.h"
#include "simplefeature/interface/ISFFeature.h"
#include "simplefeature/interface/ISFFeatureDefn.h"
#include "simplefeature/interface/ISFFieldDefn.h"


namespace ipl
{
	class XSFFeature : public ISFFeature
	{
	private:
		long							m_id;
		ref_ptr<ISFFeatureDefn>			m_defn;
		SFField							*m_fields;

		ref_ptr<IGeoObject>	m_geometry;

	protected:
		//    char				*m_styleString;
		//  osfStyleTable		*m_styleTable;
		char				*m_tmpFieldValue;

	public:
		XSFFeature(ref_ptr<ISFFeatureDefn> poDefnIn);
		virtual ~XSFFeature();

		virtual int IsFieldSet(int iField) const
		{
			return	m_fields[iField].Set.nMarker1 != SF_unSetMarker
				|| m_fields[iField].Set.nMarker2 != SF_unSetMarker;
		}

		// 	virtual const char *GetStyleString()
		// 	{
		// 		return m_styleString;
		// 	}
		// 
		// 	virtual void SetStyleString(const char *pszString)
		// 	{
		// 		if (pszString == NULL)
		// 		{
		// 			m_styleString = NULL;
		// 			return;
		// 		}
		// 		if( strlen(m_styleString) < strlen(pszString) )
		// 		{
		// 			delete m_styleString;
		// 			m_styleString = NULL;
		// 		}
		// 		if( NULL == m_styleString && NULL != pszString )	{
		// 			m_styleString = new char[strlen(pszString)];
		// 			strcpy( m_styleString, pszString );
		// 		}
		// 	}

		virtual long GetID() const { return m_id; }
		virtual SFERR SetID(long id) { m_id = id;	return SFERR_NONE; }

		virtual void SetField(int i, SFField * puValue);

		virtual ISFFeatureDefn *GetDefnRef() { return m_defn.get(); }

		// 设置拷贝
		virtual SFERR SetGeometry(const ref_ptr<IGeoObject> poGeomIn);

		// 直接设置引用
		virtual SFERR SetGeometryDirectly(ref_ptr<IGeoObject> poGeomIn);

		virtual IGeoObject *GetGeometryRef();

		// 偷取几何对象，原有的会丢失, 必须加锁，否则丢失
		virtual ref_ptr<IGeoObject> StealGeometry();

		virtual ISFFeature *Clone() const;

		//  virtual bool  Equal( XSFFeature * poFeature );

		virtual size_t GetFieldCount();

		virtual ISFFieldDefn *GetFieldDefnRef(int iField);

		virtual int GetFieldIndex(const char * pszName);

		virtual SFField *GetRawFieldRef(int i);

	public:
		IPL_OBJECT_IMP1(XSFFeature, ISFFeature, "default", "SF Feature")
	};
}


