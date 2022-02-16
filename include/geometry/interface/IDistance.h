#pragma once

#include "core/ipldef.h"
#include "core/iplmacro.h"
#include "core/interface/IObject.h"
#include "geometry/interface/IGeoObject.h"


namespace ipl
{
	//���ξ������ӿ�
	interface IDistance : public IObject
	{
	public:
		virtual ipl_float64 getDistance(const IGeoObject *pObj1, const IGeoObject *pObj2) const = 0;

		IPL_INTERFACE_DEF(IObject, "distance");
	};

}
