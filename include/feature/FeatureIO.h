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
#include "feature/FeatureDef.h"

namespace ipl
{
	//file format description for intersection 2D point
/******************************************************
file suffix: .ip2d
file flag: intersection file V1.0
number of planes
plane i_th path
boundbox
number of intersections
intersection i_th type  (note: the first intersection is the principal intersection)
                  coordinate[4]
				  number of connected planes
				  connected plane ID list
				  connected plane attributes list

				  intersection weight
				  variance of coordinate x, y
******************************************************/
	/** \brief read 2D intersection points from a file    the file extension is ".ip2d"
	* \param[in] filename
	* \param[out] model_names: the model linked to intersection points
	*.\param[out] ipt2ds: 2d intersection points
	* \return
	*  * < 0 (-1) on error
	*  * == 0 on success
	*/
	IPL_BASE_API int  load_IntersectionPoint2D(const std::string filename,
											std::vector<std::string> &model_names, 
											double min_pt[3], double max_pt[3],
											std::vector<ipl::feature::IntersectionPoint> &ipt2ds);

	
	/** \brief w.rite 2D intersection points into a file    the file extension is ".ip2d"
	* \param[in] filename
	* \param[in] model_names: the model linked to intersection points
	*.\param[in] ipt2ds: ipt2ds: 2d intersection points
	* \return
	*  * < 0 (-1) on error
	*  * == 0 on success
	*/
	IPL_BASE_API int  save_IntersectionPoint2D(const std::string filename,
											const std::vector<std::string> model_names,
											const std::vector<ipl::feature::IntersectionPoint> &ipt2ds);


	IPL_BASE_API int load_PlaneAttribute(const std::string filename, feature::ProjectedLineAttribute &att);

	IPL_BASE_API int save_PlaneAttribute(const std::string filename, const feature::ProjectedLineAttribute &att);
}

