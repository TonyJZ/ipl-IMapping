#pragma once
#include "core/iplcore.h"
#include "fitting/geoModelDef.h"
#include "spatialindexing/iploctree_key.h"
//#include <pcl/octree/octree_key.h>
#include <boost/unordered_map.hpp>

namespace ipl
{
	//discard 2018-04-14  Tony
	//��ͬ���͵�CCL(wall, ceiling)�ڲ�ͬ��gcm�д���
//	template<typename PointT>
// 	struct CandidateChangeLayer   //CCL
// 	{
// 		ref_ptr<geoModel::PointGroup<PointT> > ceilingGroup;
// 		ref_ptr<geoModel::PointGroup<PointT> > wallGroup;
// 	};


	//GCM�еĿ���������¼ÿ���г��ֵ�CCL����
	typedef std::vector<std::string>  CCLNameList;

	//ȫ�ֱ仯ͼGCM��������CandidateChangeLayer�ϲ�
	typedef boost::unordered_map<iplOctreeKey, CCLNameList> ChangeVoxelMap;

	const char CeilingFolder[] = "ceiling";
	const char WallFolder[] = "wall";

	const char CCLName[] = "CCL";
	const char GCMIndicesFolder[] = "GCMIndices";
	const char GCMParamsName[] = "GCMParams.txt"; //GCM�����ļ���, ��ͬGCM��voxelPointIndices���ɻ���
	
	const char CCLIndiceFile_Suffix[] = ".idx";          //�����ļ���׺
	const char CCLLutFile_Suffix[] = ".lut";            //���Ʋ��ұ��ļ���׺
	const char GCMFile_Suffix[] = ".gcm";               //global change map �ļ���׺  ����GlobalChangeDetection
	const char CSMFile_Suffix[] = ".csm";				//changed segment map �ļ���׺  GlobalChangeDetection�����Ľ��
}
