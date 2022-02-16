#include "io/CoordinateOffsetFileIO.h"
#include <fstream>

namespace ipl
{
	const std::string ORG2DST = "ORG2DST";
	const std::string DST2ORG = "DST2ORG";

	bool load_OffsetParam(const std::string filename, Eigen::Vector3d &org2dst, Eigen::Vector3d &dst2org)
	{
		std::ifstream  ifs;
		ifs.open(filename);

		if (!ifs.is_open())
		{
			return false;
		}

		std::string file_desc;

		ifs >> file_desc;
		for (int i = 0; i < 3; ++i)
		{
			ifs >> org2dst[i];
		}

		ifs >> file_desc;
		for (int i = 0; i < 3; ++i)
		{
			ifs >> dst2org[i];
		}

		ifs.close();

		return true;
	}

	bool save_OffsetParam(const std::string filename, Eigen::Vector3d org2dst, Eigen::Vector3d dst2org)
	{
		std::ofstream  ofs;
		ofs.open(filename);

		if (!ofs.is_open())
		{
			return false;
		}

		ofs << ORG2DST << std::endl;
		for (int i = 0; i < 3; ++i)
		{
			ofs << org2dst[i];
			ofs << std::endl;
		}
		
		ofs << DST2ORG << std::endl;
		for (int i = 0; i < 3; ++i)
		{
			ofs << dst2org[i];
			ofs << std::endl;
		}

		ofs.close();

		return true;
	}
}

