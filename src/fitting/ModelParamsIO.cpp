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
#include "fitting/ModelParamsIO.h"
#include <fstream>
//#include <stdlib>

using namespace ipl::geoModel;

int ipl::load_geoModel_params(const std::string &filename, ipl::geoModel::geoModelInfo &info)
{
	std::ifstream  ifs;
	ifs.open(filename);

	if (!ifs.is_open())
	{
		std::cout << "error: can't open " << filename << std::endl;
		return (-1);
	}

	std::string aline;
	std::string model_flag;
	ifs >> model_flag;

	if (model_flag.compare(gMFlag_PLANE) == 0)
	{
		//mType = gMT_PLANE;
		info.type = gMT_PLANE;
		info.coef.values.resize(4, 0.0);
		ifs >> info.coef.values[0] >> info.coef.values[1] >> info.coef.values[2] >> info.coef.values[3];
	}
	else if (model_flag.compare(gMFlag_LINE) == 0)
	{
		//mType = gMT_LINE;
		info.type = gMT_LINE3D;
		info.coef.values.resize(6, 0.0);
		//x0, y0, z0, a, b, c
		// x = x0 + a*t
		// y = y0 + b*t
		// z = z0 + c*t
		ifs >> info.coef.values[0] >> info.coef.values[1] >> info.coef.values[2]
			>> info.coef.values[3] >> info.coef.values[4] >> info.coef.values[5];
	}
	else if (model_flag.compare(gMFlag_UNDEFINED) == 0)
	{
		//unknown model type
		//mType = gMT_UNDEFINED;
		info.type = gMT_UNDEFINED;
		info.coef.values.clear();

		while (!ifs.eof())
		{
			float v;
		
			ifs >> aline;
			if (aline.compare(gInfoFlag_Section_OBSSD) != 0 )
			{
				v = atof(aline.c_str());
				info.coef.values.push_back(v);
			}
			else
				break;
		}
	}

	std::string strDesc;
	ifs >> strDesc;
	ifs >> info.bbox.min_pt[0] >> info.bbox.min_pt[1] >> info.bbox.min_pt[2];
	ifs >> info.bbox.max_pt[0] >> info.bbox.max_pt[1] >> info.bbox.max_pt[2];

	if (!ifs.eof())
	{
		if (aline.empty())
		{
			ifs >> model_flag;
		}
		else
		{
			model_flag = aline;
		}

		if (model_flag.compare(gInfoFlag_Section_OBSSD) == 0)
		{
			ifs >> info.obsSD;
		}

		ifs >> model_flag;
		if (model_flag.compare(gInfoFlag_Section_COEFVAR) == 0)
		{
			info.coefVar.clear();
			while (!ifs.eof())
			{
				double v;

				ifs >> v;
				info.coefVar.push_back(v);
			}
		}
	}
	
	ifs.close();

	return (0);
}

int ipl::save_geoModel_params(const std::string &filename, const ipl::geoModel::geoModelInfo &info)
{
	std::ofstream  ofs;
	ofs.open(filename);

	if (!ofs.is_open())
	{
		std::cout << "error: can't open " << filename << std::endl;
		return (-1);
	}

	if (info.type == gMT_LINE3D)
	{
		ofs << gMFlag_LINE << std::endl;

		assert(info.coef.values.size() == 6);
	}
	else if (info.type == gMT_PLANE)
	{
		ofs << gMFlag_PLANE << std::endl;
		assert(info.coef.values.size() == 4);
	}
	else if(info.type == gMT_UNDEFINED)
	{
		ofs << gMFlag_UNDEFINED << std::endl;
		
	}

	for (int j = 0; j < info.coef.values.size(); j++)
	{
		ofs << info.coef.values[j] << std::endl;
	}

	ofs << "BoundBox:" << std::endl;
	ofs << info.bbox.min_pt[0] << " " << info.bbox.min_pt[1] << " " << info.bbox.min_pt[2] << std::endl;
	ofs << info.bbox.max_pt[0] << " " << info.bbox.max_pt[1] << " " << info.bbox.max_pt[2] << std::endl;

	ofs << gInfoFlag_Section_OBSSD << std::endl;
	ofs << info.obsSD << std::endl;

	ofs << gInfoFlag_Section_COEFVAR << std::endl;
	for (size_t i = 0; i < info.coefVar.size(); ++i)
	{
		ofs << info.coefVar[i] << std::endl;
	}

	ofs.close();

	return (0);
}

