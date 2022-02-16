#pragma once

#include "core/iplcore.h"


namespace ipl
{
	//based on bundle.out
	typedef struct
	{
		double f;
		double k1, k2;

		Eigen::Matrix<double, 4, 4> mRC; //旋转和平移  投影矩阵 P = KR[I  -C]， K为摄像机矩阵

	} simpleCameraModel;

	typedef struct
	{
		int imgID;
		int keyID;
		double img_coord[2]; //去主点后的像素坐标
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

