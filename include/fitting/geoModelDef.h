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
			double obsSD;	//�۲�ֵ��׼��  sqrt(D^2/(n-t)), ��ƫ����  RMS = sqrt(D^2/n), ��ƫ���� 
			std::vector<double> coefVar;  //���Ʋ����ķ���
			BoundingBox bbox;
		};

		const char ModelParamFile_Suffix[] = ".param";

		//�������ģ�͵�ļ���
		template<typename PointT>
		struct PointGroup
		{
			std::string pathName;  //ȫ·����
			ref_ptr<iplPointCloud<PointT> > cloud; //�㼯
			std::vector<int> lut;        //���Ӧ��ģ��ID���ұ�
			std::vector<geoModel::geoModelInfo> modelParams;  //ģ�Ͳ���
		};

		const char PointGroup_CloudName[] = "cloud.pcd";
		const char PointGroup_LutName[] = "cloud.lut";
		const char PointGroup_ModelParamsFolder[] = "modelParams";
		const char PointGroup_GCMIndicesFolder[] = "GCMIndices";
	}
	
}


