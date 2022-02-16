#include "io/GeoTransFileIO.h"

#include <fstream>

namespace ipl
{
	bool load_RigidTransParam(const std::string filename, Eigen::Matrix4d &trans_param, double &rms)
	{
		std::ifstream  ifs;
		ifs.open(filename);

		if (!ifs.is_open())
		{
			return false;
		}

		//	std::string file_desc;
		//	file_desc.resize(100);
		std::string file_desc;
// 		ifs >> file_desc;
// 		ifs >> orgin[0] >> orgin[1] >> orgin[2];

		ifs >> file_desc;

		for (int i = 0; i < 16; i++)
			ifs >> trans_param(i);

		ifs >> file_desc;
		ifs >> rms;

		ifs.close();

//		trans_param.transposeInPlace();

		return true;
	}

	bool save_RigidTransParam(const std::string filename, Eigen::Matrix4d trans_param, double rms)
	{
		std::ofstream  ofs;
		ofs.open(filename);

		if (!ofs.is_open())
		{
			return false;
		}

// 		ofs << "This_is_the_transformation_origin:" << std::endl;
// 		ofs << orgin[0] << " " << orgin[1] << " " << orgin[2] << std::endl;		ofs << "This_is_the_transformation_matrix:" << std::endl;
		
		ofs << "This_is_the_transformation_matrix:" << std::endl;
		//note: eigen默认按列优先存储，为了与外部处理一致，写入文件时对矩阵进行转置
		ofs << trans_param.transpose();
		//ofs << trans_param;
		ofs << std::endl;

		ofs << "This_is_RMS:" << std::endl;
		ofs << rms << std::endl;

		ofs.close();

		return true;
	}



}


