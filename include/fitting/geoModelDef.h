#pragma once

#include "core/iplcore.h"

namespace ipl 
{
	namespace geoModel 
	{
		typedef struct
		{
			double min_pt[3];
			double max_pt[3];
		} BoundingBox;

		//geometric model type
		enum geoModelType
		{
			gMT_UNDEFINED = 0,
			gMT_LINE3D,
			gMT_PLANE
		};

		const char gMFlag_UNDEFINED[] = "UNDEFINED";
		const char gMFlag_PLANE[] = "PLANE";
		const char gMFlag_LINE[] = "LINE";

		const char gInfoFlag_Section_OBSSD[] = "OBSERVATION_STANDARD_ERROR";
		const char gInfoFlag_Section_COEFVAR[] = "ESTIMATOR_VARIANCE";


		struct geoModelInfo 
		{
			geoModelType  type;
			iplModelCoeffs   coef;   //ax+by+cz+d=0  
			double obsSD;	//观测值标准差  sqrt(D^2/(n-t)), 无偏估计  RMS = sqrt(D^2/n), 有偏估计 
			std::vector<double> coefVar;  //估计参数的方差
			BoundingBox bbox;
		};

		const char ModelParamFile_Suffix[] = ".param";

		//多个几何模型点的集合
		template<typename PointT>
		struct PointGroup
		{
			std::string pathName;  //全路径名
			ref_ptr<iplPointCloud<PointT> > cloud; //点集
			std::vector<int> lut;        //点对应的模型ID查找表
			std::vector<geoModel::geoModelInfo> modelParams;  //模型参数
		};

		const char PointGroup_CloudName[] = "cloud.pcd";
		const char PointGroup_LutName[] = "cloud.lut";
		const char PointGroup_ModelParamsFolder[] = "modelParams";
		const char PointGroup_GCMIndicesFolder[] = "GCMIndices";
	}
	
}


