#include "io/ImageGeoIO.h"
#include <fstream>

namespace ipl
{

	bool load_bundleFile(const std::string filename, std::vector<simpleCameraModel> &cams, std::vector<tiePoint> &tps)
	{
		std::ifstream  ifs;
		ifs.open(filename);

		if (!ifs.is_open())
		{
			return false;
		}

		cams.clear();
		tps.clear();

		char file_desc[1024];
		ifs.getline(file_desc, 1024);

		int num_cams, num_tps;

		ifs >> num_cams >> num_tps;

		for (int i = 0; i < num_cams; i++)
		{
			simpleCameraModel cam;

			ifs >> cam.f >> cam.k1 >> cam.k2;

			cam.mRC = Eigen::Matrix<double, 4, 4>::Identity();
			ifs >> cam.mRC.block<3, 3>(0, 0)(0, 0)
				>> cam.mRC.block<3, 3>(0, 0)(0, 1)
				>> cam.mRC.block<3, 3>(0, 0)(0, 2);

			ifs >> cam.mRC.block<3, 3>(0, 0)(1, 0)
				>> cam.mRC.block<3, 3>(0, 0)(1, 1)
				>> cam.mRC.block<3, 3>(0, 0)(1, 2);

			ifs >> cam.mRC.block<3, 3>(0, 0)(2, 0)
				>> cam.mRC.block<3, 3>(0, 0)(2, 1)
				>> cam.mRC.block<3, 3>(0, 0)(2, 2);

			ifs >> cam.mRC.block<3, 1>(0, 3)(0)
				>> cam.mRC.block<3, 1>(0, 3)(1)
				>> cam.mRC.block<3, 1>(0, 3)(2);

			//		std::cout << cam.mRC << std::endl;

			cams.push_back(cam);
		}

		for (int i = 0; i < num_tps; i++)
		{
			tiePoint tp;

			ifs >> tp.pos[0]
				>> tp.pos[1]
				>> tp.pos[2];

			ifs >> tp.color[0]
				>> tp.color[1]
				>> tp.color[2];

			int nv = 0;
			ifs >> nv;

			for (int j = 0; j < nv; j++)
			{
				imagePoint ipt;

				ifs >> ipt.imgID >> ipt.keyID >> ipt.img_coord[0] >> ipt.img_coord[1];

				tp.viewList.push_back(ipt);
			}

			tps.push_back(tp);
		}

		ifs.close();

		return true;
	}

}

