#pragma once

#include "simplefeature/SFBaseDef.h"
//#include "simplefeature/interface/SFIGeometry.h"
#include "core/interface/IObject.h"
#include "core/iplmacro.h"

namespace ipl
{
	interface ISFFieldDefn : public IObject
	{
	public:
		virtual void			SetName(const char *) = 0;
		virtual const char		*GetNameRef() = 0;

		virtual SF_FieldType	GetType() = 0;
		virtual void			SetType(SF_FieldType eTypeIn) = 0;
		virtual int				GetWidth() = 0;
		virtual void			SetWidth(int nWidthIn) = 0;

		virtual int				GetPrecision() = 0;
		virtual void			SetPrecision(int nPrecisionIn) = 0;

		virtual void			Set(const char * pszName, SF_FieldType type, int nWidthIn, int nPrecisionIn, SF_Justification justification) = 0;;

		virtual void			SetDefault(const SFField *) = 0;;
		virtual const SFField	*GetDefaultRef() = 0;

		virtual void            SetJustify(SF_Justification eJustifyln) = 0;
		virtual SF_Justification GetJustify() = 0;

		virtual ISFFieldDefn         *Clone() const = 0;

		IPL_INTERFACE_DEF(IObject, "SFFieldDefn");
	};

}
