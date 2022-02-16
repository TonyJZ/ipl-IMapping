#pragma once
#include "core/ipldef.h"
#include "core/iplstd.h"
#include "core/iplmacro.h"
#include "core/interface/IObject.h"

namespace ipl
{
	interface IDataSource : public IObject
	{
	public:
		//¿¼ÂÇ·ÏÆú½Ó¿Ú  2018-02-12  
		virtual const iplChar *getFilePath() const = 0;

		//virtual const iplDSType getType() const = 0;

		IPL_INTERFACE_DEF(IObject, _T("dataSource"))
	};



}

