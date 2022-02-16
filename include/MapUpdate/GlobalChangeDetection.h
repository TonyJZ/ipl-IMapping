#pragma once
#include "core/iplcore.h"
#include "MapUpdate/MapUpdateDef.h"
#include "fitting/geoModelDef.h"
#include "spatialindexing/PointVoxelization.h"

namespace ipl
{
	//global change voxel map�ļ����ö����Ƹ�ʽ
	const char GCVMFILE_Suffix[] = ".gcvm";   //gcvm�ļ���׺

	template <typename PointT>
	class GlobalChangeDetection
	{
		typedef boost::unordered_map<iplOctreeKey, int> ChangeKeyMap; //0 unused; 1 in stack; 2 segmented

	public:
		GlobalChangeDetection();
		~GlobalChangeDetection();


		void createGCM(double org[3], double vsize[3]);
		
		int openGCM(std::string filename);

		int saveGCM(std::string filename);

		//��GCM�����һ��CCL
		int addCandidateChangeLayer(const std::string &ccl_name);


		int detect(std::vector<ChangeVoxelMap> &changeSegs, uint32_t reliabilityTh = 3);


	protected:
		//��CCL�е�PointGroup���䵽��ǰGCM��ÿ��element��
		int voxlize_PointGroup(geoModel::PointGroup<PointT> *ptGroup);
		

	private:
		double origin_[3];
		double voxel_size_[3];

		ChangeVoxelMap		gcm_;
		ref_ptr<PointVoxelization<PointT> > pv_;

		uint32_t reliabilityTh_;

//		std::vector<ChangeVoxelMap>  change_segments_;
	};
}

#include "MapUpdate/impl/GlobalChangeDetection.hpp"
