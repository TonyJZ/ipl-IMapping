#pragma once

#include "core/iplcore.h"


namespace ipl
{
	//based on bundle.out
	typedef struct
	{
		double f;
		double k1, k2;

		Eigen::Matrix<double, 4, 4> mRC; //��ת��ƽ��  ͶӰ���� P = KR[I  -C]�� KΪ���������

	} simpleCameraModel;

	typedef struct
	{
		int imgID;
		int keyID;
		double img_coord[2]; //ȥ��������������
	} imagePoint;

	typedef struct
	{
		double pos[3];  //position
		int color[3]; //RGB
		std::vector<imagePoint> viewList;  //a list of views the point is visible in

	} tiePoint;


	//bundle.out
	IPL_BASE_API bool load_bundleFile(const std::string filename, 
		std::vector<simpleCameraModel> &cams, std::vector<tiePoint> &tps);
}

