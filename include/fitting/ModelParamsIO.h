#pragma once
/*
* Software License Agreement (Proprietary License)
*
*
*  Copyright (c) 2017-, Appropolis, Inc.
*
*  All rights reserved.
*
*  rights described below...
*
*/

#include "core/iplcore.h"
#include "fitting/geoModelDef.h"

namespace ipl
{
	/** \brief read geometric model parameters from a file
	* \param[in] filename
	* \param[out] mType: the flag to designate the model type  
	*.\param[out] mCoef: geometric model parameters
	* \return
	*  * < 0 (-1) on error
	*  * == 0 on success
	*/
	IPL_BASE_API int  load_geoModel_params(const std::string &filename,
							ipl::geoModel::geoModelInfo &info);

	/** \brief save geometric model parameters in a file
	* \param[in] filename
	* \param[in] mType: the flag to designate the model type
	*.\param[in] mCoef: geometric model parameters
	* \return
	*  * < 0 (-1) on error
	*  * == 0 on success
	*/
	IPL_BASE_API int  save_geoModel_params(const std::string &filename,
							const ipl::geoModel::geoModelInfo &info);

}

