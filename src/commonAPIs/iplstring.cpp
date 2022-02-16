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

#include "commonAPIs/iplstring.h"
#include <boost/filesystem.hpp>
#include <boost/algorithm/string_regex.hpp>

using namespace std;
using namespace boost::filesystem;
using namespace boost::algorithm;
using namespace ipl;


void ipl::convert2lower(const std::string str, std::string &lower_str)
{
	lower_str = boost::algorithm::to_lower_copy(str);
}


void ipl::convert2upper(const std::string str, std::string &upper_str)
{
	upper_str = boost::algorithm::to_upper_copy(str);
}


void ipl::extract_file_name(const std::string file_name, std::string &result_name, std::string &suffix_name)
{
	boost::filesystem::path p(file_name);
	result_name = p.parent_path().string();
	result_name += '/';
	result_name += p.stem().string();

	suffix_name = p.extension().string();
}

void ipl::extract_pure_file_name(const std::string file_name, std::string &pure_name, std::string &suffix_name)
{
	boost::filesystem::path p(file_name);
	pure_name = p.stem().string();
	suffix_name = p.extension().string();
}

void ipl::get_dir_from_path(const std::string path, std::string &dir)
{
	string standard_path = replace_all_copy(path, "\\", "/");
	boost::filesystem::path p(standard_path);

	if (!exists(p))
		return;

	if (!is_directory(p))        // ÊÇ·ñÊÇÄ¿Â¼
	{
		dir = p.parent_path().string();
	}
	else
	{
		std::string stem = p.stem().string();
		if (stem == ".")
			dir = p.parent_path().string();
		else
			dir = p.string();
	}


}
