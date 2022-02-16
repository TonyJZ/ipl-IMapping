#pragma once
#include "MapUpdate/GlobalChangeDetection.h"

//#include "MapUpdate/CandidateChangeLayerIO.h"
#include "spatialindexing/SpatialindexDef.h"
#include "MapUpdate/ChangeVoxelSegmentIO.h"
#include "commonAPIs/iplutility.h"
#include "fitting/PointGroupIO.h"

#include <stdio.h>
#include <stack>

#define _USE_DYNAMIC_HASHMAP_ALG_


namespace ipl
{
	template<typename PointT>
	GlobalChangeDetection<PointT>::GlobalChangeDetection()
	{

	}

	template<typename PointT>
	GlobalChangeDetection<PointT>::~GlobalChangeDetection()
	{
		gcm_.clear();
		pv_.reset();
	}

	template<typename PointT> void
	GlobalChangeDetection<PointT>::createGCM(double org[3], double vsize[3])
	{
		origin_[0] = org[0]; origin_[1] = org[1]; origin_[2] = org[2];
		voxel_size_[0] = vsize[0]; voxel_size_[1] = vsize[1]; voxel_size_[2] = vsize[2];

		gcm_.clear();
		pv_.reset(new PointVoxelization<PointT>);

//		pv_->setOrigin(static_cast<float>(origin_[0]), static_cast<float>(origin_[1]), static_cast<float>(origin_[2]));

	}


	template<typename PointT> int
		GlobalChangeDetection<PointT>::addCandidateChangeLayer(const std::string &ccl_name)
	{
		int Ret;
		ref_ptr<geoModel::PointGroup<PointT> > ccl(new geoModel::PointGroup<PointT>);
//		CandidateChangeLayer<PointT> ccl;

		Ret = load_PointGroup(ccl_name, *ccl);
		if (Ret != 0)
		{
			std::cout << "can not open " << ccl_name << std::endl;
			return Ret;
		}

		voxlize_PointGroup(ccl.get());
		
		return 0;
	}


	template<typename PointT> int
		GlobalChangeDetection<PointT>::voxlize_PointGroup(geoModel::PointGroup<PointT> *ptGroup)
	{
		if (ptGroup == NULL)
			return -1;

		pv_->setOrigin(static_cast<float>(origin_[0]), static_cast<float>(origin_[1]), static_cast<float>(origin_[2]));
		pv_->setInputCloud(ptGroup->cloud);
		pv_->apply(static_cast<float>(voxel_size_[0]), static_cast<float>(voxel_size_[1]), static_cast<float>(voxel_size_[2]));

		VoxelKeyMap *vIDmap;
		vIDmap = pv_->getVoxelKeyMap();

		std::string GCMIndices_path;
		GCMIndices_path = ptGroup->pathName + "/" + geoModel::PointGroup_GCMIndicesFolder;
		//清空原有的索引文件
		create_folder(GCMIndices_path);
		empty_folder(GCMIndices_path);

		save_voxelPointIndices(GCMIndices_path, origin_, voxel_size_, *vIDmap);  //保存索引文件

		//更新GCM
		VoxelKeyMap::iterator vIt;
		for (vIt = vIDmap->begin(); vIt != vIDmap->end(); ++vIt)
		{
			iplOctreeKey key_arg = vIt->first;

			ChangeVoxelMap::iterator it_gcm;
			it_gcm = gcm_.find(key_arg);

			if (it_gcm != gcm_.end())
			{//existing
				it_gcm->second.push_back(ptGroup->pathName);
			}
			else
			{
				CCLNameList newItem;
				newItem.push_back(ptGroup->pathName);
				gcm_.insert(std::make_pair(key_arg, newItem));
			}
		}

		return 0;
	}

	template<typename PointT> int
		GlobalChangeDetection<PointT>::openGCM(std::string filename)
	{
		FILE *fp = NULL;

		fp = fopen(filename.c_str(), "rb");
		if (fp == NULL)
		{
			std::cout << "can not open " << filename << std::endl;
			return -1;
		}

		fread(origin_, sizeof(double), 3, fp);
		fread(voxel_size_, sizeof(double), 3, fp);

		gcm_.clear();

		uint32_t num;
		fread(&num, sizeof(num), 1, fp);

		for (uint32_t i = 0; i < num; ++i)
		{
			iplOctreeKey key;
			fread(key.key_, sizeof(key.key_), 1, fp);

			uint32_t nccl;  //number of ccl
			CCLNameList value;

			fread(&nccl, sizeof(nccl), 1, fp);
			for (uint32_t j = 0; j < nccl; ++j)
			{
				char filename[256];
				fread(filename, sizeof(char), 256, fp); //每个ccl名只保留256个字符，不可超过
				std::string ccl_name = filename;

				value.push_back(ccl_name);
			}

			gcm_.insert(std::make_pair(key, value));
		}

		fclose(fp);
		fp = NULL;

		return 0;
	}

	template<typename PointT> int
		GlobalChangeDetection<PointT>::saveGCM(std::string filename)
	{
		FILE *fp = NULL;
		fp = fopen(filename.c_str(), "wb");
		if (fp == NULL)
		{
			std::cout << "can not open " << filename << std::endl;
			return -1;
		}

		fwrite(origin_, sizeof(double), 3, fp);
		fwrite(voxel_size_, sizeof(double), 3, fp);

		uint32_t num = gcm_.size();
		fwrite(&num, sizeof(num), 1, fp);

		ChangeVoxelMap::iterator it;
		for (it = gcm_.begin(); it != gcm_.end(); ++it)
		{
			iplOctreeKey key = it->first;
			fwrite(key.key_, sizeof(int32_t), 3, fp);

			uint32_t nccl = it->second.size();  //number of ccl
			fwrite(&nccl, sizeof(nccl), 1, fp);
			for (uint32_t j = 0; j < nccl; ++j)
			{
				char filename[256]="\0";
				strcpy(filename, it->second[j].c_str());
				fwrite(filename,sizeof(char), 256, fp); //每个ccl名只保留256个字符，不可超过
			}
		}

		fclose(fp);
		fp = NULL;

		return 0;
	}

#ifdef _USE_FLAG_HASHMAP_ALG_
	template<typename PointT> int
		GlobalChangeDetection<PointT>::detect(std::vector<ChangeVoxelMap> &changeSegs, uint32_t reliabilityTh)
	{
		reliabilityTh_ = reliabilityTh;

		//1. 根据reliability提取变化的voxel
		ChangeKeyMap keyMap;

		ChangeVoxelMap::iterator  it_gcm;
		for (it_gcm = gcm_.begin(); it_gcm != gcm_.end(); ++it_gcm)
		{
			if (it_gcm->second.size() > reliabilityTh)
			{
				keyMap.insert(std::make_pair(it_gcm->first, 0));
			}
		}

		//2. region growing 将相邻的voxel存入segments
		//领域系统采用26领域
		std::vector<iplOctreeKey> neighbourhood;
		neighbourhood.resize(26);
		{
			neighbourhood[0].x = 1;		neighbourhood[0].y = 0;		neighbourhood[0].z = 0;
			neighbourhood[1].x = -1;	neighbourhood[1].y = 0;		neighbourhood[1].z = 0;
			neighbourhood[2].x = 0;		neighbourhood[2].y = 1;		neighbourhood[2].z = 0;
			neighbourhood[3].x = 0;		neighbourhood[3].y = -1;	neighbourhood[3].z = 0;
			neighbourhood[4].x = 1;		neighbourhood[4].y = 1;		neighbourhood[4].z = 0;
			neighbourhood[5].x = 1;		neighbourhood[5].y = -1;	neighbourhood[5].z = 0;
			neighbourhood[6].x = -1;	neighbourhood[6].y = 1;		neighbourhood[6].z = 0;
			neighbourhood[7].x = -1;	neighbourhood[7].y = -1;	neighbourhood[7].z = 0;

			neighbourhood[8].x = 0;		neighbourhood[8].y = 0;		neighbourhood[8].z = 1;
			neighbourhood[9].x = 1;		neighbourhood[9].y = 0;		neighbourhood[9].z = 1;
			neighbourhood[10].x = -1;	neighbourhood[10].y = 0;	neighbourhood[10].z = 1;
			neighbourhood[11].x = 0;	neighbourhood[11].y = 1;	neighbourhood[11].z = 1;
			neighbourhood[12].x = 0;	neighbourhood[12].y = -1;	neighbourhood[12].z = 1;
			neighbourhood[13].x = 1;	neighbourhood[13].y = 1;	neighbourhood[13].z = 1;
			neighbourhood[14].x = 1;	neighbourhood[14].y = -1;	neighbourhood[14].z = 1;
			neighbourhood[15].x = -1;	neighbourhood[15].y = 1;	neighbourhood[15].z = 1;
			neighbourhood[16].x = -1;	neighbourhood[16].y = -1;	neighbourhood[16].z = 1;

			neighbourhood[17].x = 0;	neighbourhood[17].y = 0;	neighbourhood[17].z = -1;
			neighbourhood[18].x = 1;	neighbourhood[18].y = 0;	neighbourhood[18].z = -1;
			neighbourhood[19].x = -1;	neighbourhood[19].y = 0;	neighbourhood[19].z = -1;
			neighbourhood[20].x = 0;	neighbourhood[20].y = 1;	neighbourhood[20].z = -1;
			neighbourhood[21].x = 0;	neighbourhood[21].y = -1;	neighbourhood[21].z = -1;
			neighbourhood[22].x = 1;	neighbourhood[22].y = 1;	neighbourhood[22].z = -1;
			neighbourhood[23].x = 1;	neighbourhood[23].y = -1;	neighbourhood[23].z = -1;
			neighbourhood[24].x = -1;	neighbourhood[24].y = 1;	neighbourhood[24].z = -1;
			neighbourhood[25].x = -1;	neighbourhood[25].y = -1;	neighbourhood[25].z = -1;
		}

		changeSegs.clear();

		ChangeKeyMap::iterator it_km;
		for (it_km = keyMap.begin(); it_km != keyMap.end(); ++it_km)
		{
			ChangeVoxelMap seg;

			//region growing
			std::stack<iplOctreeKey> keyStack;
			if (it_km->second == 2)
				continue; //当前key已被处理

			assert(it_km->second != 1);

			keyStack.push(it_km->first); // 插入种子
			it_km->second = 1;  //在栈标记

			while (!keyStack.empty())
			{//种子生长
				iplOctreeKey topKey = keyStack.top();
				keyStack.pop();

				seg.insert(std::make_pair(topKey, gcm_[topKey]));
				keyMap[topKey] = 2; //已分割标记

				for (size_t i = 0; i < neighbourhood.size(); ++i)
				{
					iplOctreeKey newKey = topKey + neighbourhood[i];
					ChangeKeyMap::iterator it_find = keyMap.find(newKey);

					if (it_find != keyMap.end())
					{//存在的key
						if (it_find->second == 0)
						{//
							keyStack.push(it_find->first);
							it_find->second = 1;
						}
					}

				}
			}

			if (seg.size() > 0)
				changeSegs.push_back(seg);
		}

		return 0;
	}
#endif
	
#ifdef _USE_DYNAMIC_HASHMAP_ALG_
	template<typename PointT> int
		GlobalChangeDetection<PointT>::detect(std::vector<ChangeVoxelMap> &changeSegs, uint32_t reliabilityTh)
	{
		reliabilityTh_ = reliabilityTh;

		//1. 根据reliability提取变化的voxel
		ChangeKeyMap keyMap;

		ChangeVoxelMap::iterator  it_gcm;
		for (it_gcm = gcm_.begin(); it_gcm != gcm_.end(); ++it_gcm)
		{
			if (it_gcm->second.size() > reliabilityTh)
			{
				keyMap.insert(std::make_pair(it_gcm->first, 0));
			}
		}

		//2. region growing 将相邻的voxel存入segments
		//领域系统采用26领域
		std::vector<iplOctreeKey> neighbourhood;
		neighbourhood.resize(26);
		{
			neighbourhood[0].x = 1;		neighbourhood[0].y = 0;		neighbourhood[0].z = 0;
			neighbourhood[1].x = -1;	neighbourhood[1].y = 0;		neighbourhood[1].z = 0;
			neighbourhood[2].x = 0;		neighbourhood[2].y = 1;		neighbourhood[2].z = 0;
			neighbourhood[3].x = 0;		neighbourhood[3].y = -1;	neighbourhood[3].z = 0;
			neighbourhood[4].x = 1;		neighbourhood[4].y = 1;		neighbourhood[4].z = 0;
			neighbourhood[5].x = 1;		neighbourhood[5].y = -1;	neighbourhood[5].z = 0;
			neighbourhood[6].x = -1;	neighbourhood[6].y = 1;		neighbourhood[6].z = 0;
			neighbourhood[7].x = -1;	neighbourhood[7].y = -1;	neighbourhood[7].z = 0;

			neighbourhood[8].x = 0;		neighbourhood[8].y = 0;		neighbourhood[8].z = 1;
			neighbourhood[9].x = 1;		neighbourhood[9].y = 0;		neighbourhood[9].z = 1;
			neighbourhood[10].x = -1;	neighbourhood[10].y = 0;	neighbourhood[10].z = 1;
			neighbourhood[11].x = 0;	neighbourhood[11].y = 1;	neighbourhood[11].z = 1;
			neighbourhood[12].x = 0;	neighbourhood[12].y = -1;	neighbourhood[12].z = 1;
			neighbourhood[13].x = 1;	neighbourhood[13].y = 1;	neighbourhood[13].z = 1;
			neighbourhood[14].x = 1;	neighbourhood[14].y = -1;	neighbourhood[14].z = 1;
			neighbourhood[15].x = -1;	neighbourhood[15].y = 1;	neighbourhood[15].z = 1;
			neighbourhood[16].x = -1;	neighbourhood[16].y = -1;	neighbourhood[16].z = 1;

			neighbourhood[17].x = 0;	neighbourhood[17].y = 0;	neighbourhood[17].z = -1;
			neighbourhood[18].x = 1;	neighbourhood[18].y = 0;	neighbourhood[18].z = -1;
			neighbourhood[19].x = -1;	neighbourhood[19].y = 0;	neighbourhood[19].z = -1;
			neighbourhood[20].x = 0;	neighbourhood[20].y = 1;	neighbourhood[20].z = -1;
			neighbourhood[21].x = 0;	neighbourhood[21].y = -1;	neighbourhood[21].z = -1;
			neighbourhood[22].x = 1;	neighbourhood[22].y = 1;	neighbourhood[22].z = -1;
			neighbourhood[23].x = 1;	neighbourhood[23].y = -1;	neighbourhood[23].z = -1;
			neighbourhood[24].x = -1;	neighbourhood[24].y = 1;	neighbourhood[24].z = -1;
			neighbourhood[25].x = -1;	neighbourhood[25].y = -1;	neighbourhood[25].z = -1;
		}

		changeSegs.clear();

		ChangeKeyMap::iterator it_km;
		while (keyMap.size()>0)
		{
			it_km = keyMap.begin();
			
			ChangeVoxelMap seg;

			//region growing
			std::stack<iplOctreeKey> keyStack;
// 			if (it_km->second == 2)
// 				continue; //当前key已被处理
// 
// 			assert(it_km->second != 1);

			keyStack.push(it_km->first); // 插入种子
//			it_km->second = 1;  //在栈标记
			keyMap.erase(it_km);

			while (!keyStack.empty())
			{//种子生长
				iplOctreeKey topKey = keyStack.top();
				keyStack.pop();

				seg.insert(std::make_pair(topKey, gcm_[topKey]));
//				keyMap[topKey] = 2; //已分割标记

				for (size_t i = 0; i < neighbourhood.size(); ++i)
				{
					iplOctreeKey newKey = topKey + neighbourhood[i];
					ChangeKeyMap::iterator it_find = keyMap.find(newKey);

					if (it_find != keyMap.end())
					{//存在的key
						keyStack.push(it_find->first);
						keyMap.erase(it_find);
					}
				}
			}

			if (seg.size() > 0)
				changeSegs.push_back(seg);
		}
		
		return 0;
	}
#endif
	

}
