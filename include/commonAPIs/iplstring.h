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
#include <boost/algorithm/string.hpp>

#include <core/ipldef.h>
#include <core/iplstd.h>

namespace ipl
{
	/** \brief covert string to lowercase string
	* \param[in] str input string
	* \param[out] lower_str  lowercase string
	*/
	IPL_BASE_API void convert2lower(const std::string str, std::string &lower_str);

	/** \brief covert string to uppercase string
	* \param[in] str input string
	* \param[out] upper_str  uppercase string
	*/
	IPL_BASE_API void convert2upper(const std::string str, std::string &upper_str);


	/** \brief extract file name and suffix from a input string
	* \param[in] file_name the file name with path and suffix
	* \param[out] result_name the file name with path
	* \param[out] suffix_name the suffix name. note the suffix contains '.'
	*/
	IPL_BASE_API void  extract_file_name(const std::string file_name, std::string &result_name, std::string &suffix_name);

	/** \brief extract pure file name and suffix from a input string
	* \param[in] file_name the file name with path and suffix
	* \param[out] pure_name the pure file name
	* \param[out] suffix_name the suffix name. note the suffix contains '.'
	*/
	IPL_BASE_API void  extract_pure_file_name(const std::string file_name, std::string &pure_name, std::string &suffix_name);

	/** \brief get directory path from a giving string
	* \param[in] path
	* \param[out] dir: the parent folder of the path itself
	*/
	IPL_BASE_API void  get_dir_from_path(const std::string path, std::string &dir);

	
};

