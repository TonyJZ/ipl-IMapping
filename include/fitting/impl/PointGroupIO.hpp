#pragma once

#include "fitting/PointGroupIO.h"
#include "commonAPIs/iplutility.h"
#include "fitting/ModelParamsIO.h"

#include <fstream>

namespace ipl
{
	template <typename PointT>
	int  load_PointGroup(const std::string &path, geoModel::PointGroup<PointT> &ptGroup)
	{
		std::string in_name;
		char buf[32];
		// 		sprintf(buf, "wall_refined_%04d", curID);
		// 		sprintf(buf_para, "wall_refined_%04d.param", curID);

	
		in_name = path;
		in_name += "/";
		in_name += geoModel::PointGroup_CloudName;

		ptGroup.cloud.reset(new iplPointCloud<PointT>);
		//读点云
		if (read_PointCloud(in_name, *(ptGroup.cloud)) != 0)
		{
			std::cout << "error: can't open " << in_name << std::endl;
			return (-1);
		}

		//读lut
		in_name = path;
		in_name += "/";
		in_name += geoModel::PointGroup_LutName;
		
		std::vector<std::string>  modelNameList;     //模型名列表，模型ID与模型名的对应
		
		std::ifstream  ifs;
		ifs.open(in_name);

		if (!ifs.is_open())
		{
			std::cout << "error: can't open " << in_name << std::endl;
			return (-1);
		}

		std::string aline;
		//		std::string model_flag;
		ifs >> aline;

		int num_model;
		ifs >> num_model;
		modelNameList.resize(num_model);
		for (size_t i = 0; i < num_model; ++i)
		{
			ifs >> modelNameList[i];
		}

		int num_pt;
		ifs >> num_pt;
		ptGroup.lut.resize(num_pt);
		for (size_t i = 0; i < num_pt; ++i)
		{
			ifs >> ptGroup.lut[i];
		}
		ifs.close();

		assert(ptGroup.lut.size() == ptGroup.cloud->size());

		//读model params
// 		in_name = path;
// 		in_name += "/";
// 		in_name += geoModel::PointGroup_ModelParamsFolder;
// 
// 		std::vector<std::string> filenames;
// 		ipl::scan_files(in_name, geoModel::ModelParamFile_Suffix, filenames);

		size_t nModels = modelNameList.size();
		ptGroup.modelParams.resize(nModels);
		for (size_t i = 0; i < nModels; ++i)
		{
			if (load_geoModel_params(modelNameList[i], ptGroup.modelParams[i]) != 0)
			{
				std::cout << "error: can't open " << modelNameList[i] << std::endl;
				return (-1);
			}
		}

		ptGroup.pathName = path;

		return (0);
	}

	template <typename PointT>
	int  save_PointGroup(const std::string &path, const geoModel::PointGroup<PointT> &ptGroup)
	{
		create_folder(path);
		empty_folder(path);

		std::string out_name;
		char buf[32];

		out_name = path;
		out_name += "/";
		out_name += geoModel::PointGroup_CloudName;

		//写点云
		if (write_PointCloud(out_name, *(ptGroup.cloud), true) != 0)
		{
			std::cout << "error: can't open " << out_name << std::endl;
			return (-1);
		}

		//写model params
		out_name = path;
		out_name += "/";
		out_name += geoModel::PointGroup_ModelParamsFolder;

		create_folder(out_name);
		std::vector<std::string>  modelNameList;     //模型名列表，模型ID与模型名的对应
		for (size_t i = 0; i < ptGroup.modelParams.size(); ++i)
		{
			std::string modelfile = out_name;
			sprintf(buf, "/modelParam_%04d%s", i, geoModel::ModelParamFile_Suffix);
			modelfile += buf;

			modelNameList.push_back(modelfile);

			if (save_geoModel_params(modelfile, ptGroup.modelParams[i]) != 0)
			{
				std::cout << "error: can't open " << modelfile << std::endl;
				return (-1);
			}
		}


		//写lut
		out_name = path;
		out_name += "/";
		out_name += geoModel::PointGroup_LutName;

		
		std::ofstream  ofs;
		ofs.open(out_name);

		if (!ofs.is_open())
		{
			std::cout << "error: can't open " << out_name << std::endl;
			return (-1);
		}

//		std::string aline;
//		std::string model_flag;
		ofs << "This_is_point_model_LUT" << std::endl;
		//modelID - model name
		ofs << modelNameList.size() << std::endl;
		for (size_t i = 0; i < modelNameList.size(); ++i)
		{
			ofs << modelNameList[i] << std::endl;
		}

		//point - modelID
		ofs << ptGroup.lut.size() << std::endl;
		for (size_t i = 0; i < ptGroup.lut.size(); ++i)
		{
			ofs << ptGroup.lut[i] << " ";
		}
		ofs.close();

		return (0);
	}

}
