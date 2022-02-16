#pragma once

//gdal
#include <GDAL/ogr_api.h>

#include <string>

#include "simplefeature/SFBaseDef.h"
#include "simplefeature/interface/ISFFieldDefn.h"

namespace ipl
{
	class XSFFieldDefn : public ISFFieldDefn
	{
	private:
		iplString			m_name;
		SF_FieldType        m_type;
		SF_Justification    m_justify;

		short               m_width;                 /* zero is variable */
		short               m_precision;
		SFField            m_default;

	public:
		XSFFieldDefn();
		virtual ~XSFFieldDefn();

		// 	void SetOGRFieldDefn(OGRFieldDefn* pogrFieldDefn);
		// 	OGRFieldDefn* GetOGRFieldDefn();

		virtual void                SetName(const char *);
		virtual const char         *GetNameRef();

		virtual SF_FieldType        GetType() { return m_type; };
		virtual void                SetType(SF_FieldType type) { m_type = type; };

		virtual int                 GetWidth() { return m_width; }
		virtual void                SetWidth(int width) { m_width = width; };

		virtual int                 GetPrecision() { return m_precision; };
		virtual void                SetPrecision(int nPrecisionIn) { m_precision = nPrecisionIn; };

		virtual void                Set(const char *, SF_FieldType, int = 0, int = 0,
			SF_Justification just = SF_JUndefined);

		virtual void                SetDefault(const SFField *);
		virtual const SFField     *GetDefaultRef();

		virtual void            SetJustify(SF_Justification eJustifyln) { m_justify = eJustifyln; }
		virtual SF_Justification GetJustify() { return m_justify; }

		virtual ISFFieldDefn         *Clone() const;

	public:
		IPL_OBJECT_IMP1(XSFFieldDefn, ISFFieldDefn, "default", "SF Field Define")
	};
}

