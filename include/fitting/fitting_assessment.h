#pragma once


#include "core/iplcore.h"
#include "fitting/geoModelDef.h"

namespace ipl
{
	IPL_BASE_API int plane_fitting_covariance(double *x, double *y, double *z, size_t num_pts, 
		geoModel::geoModelInfo &pInfo);

}

