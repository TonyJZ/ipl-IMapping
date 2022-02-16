#pragma once
#include "MapUpdate/MapUpdater.h"
#include "commonAPIs/iplutility.h"
#include "fitting/PointGroupIO.h"
#include "io/PointcloudIO.h"
#include "fitting/ModelParamsIO.h"
#include "MapUpdate/ChangeVoxelSegmentIO.h"

#if HAVE_PCL
#include <pcl/common/angles.h>
#include <pcl/conversions.h>
#endif

#include <Eigen/Dense>

#include <fstream>

//#define _DEBUGGING_

namespace ipl
{
template<typename PointT>
MapUpdater<PointT>::MapUpdater()
{
//	folder_name_ = output_folder;
	minSeg_ = 100;
}

template<typename PointT>
MapUpdater<PointT>::~MapUpdater()
{

}

template<typename PointT> void 
MapUpdater<PointT>::setFilters(int minSeg /* = 100 */)
{
	minSeg_ = minSeg;
}


template<typename PointT> int
MapUpdater<PointT>::loadCSM(std::string filename)
{
	int ret;

	changed_segments_.clear();
	ret = load_ChangeVoxelSegments(filename, changed_segments_);
	if (ret != 0)
	{
		std::cout << "can not open " << filename << std::endl;
		return ret;
	}

	//按照segment --> CCL --> voxel 重新整理
	organized_segments_.clear();
	size_t nSegs = changed_segments_.size();
	organized_segments_.resize(nSegs);

	for (size_t i = 0; i < nSegs; ++i)
	{//i-th segment
		ChangeVoxelMap::iterator vm_it;
		for (vm_it = changed_segments_[i].begin(); vm_it != changed_segments_[i].end(); ++vm_it)
		{//traverse voxels, each voxel contains some CCLs
			iplOctreeKey key = vm_it->first;
			size_t nCCLs = vm_it->second.size();

			for (size_t iCCL = 0; iCCL < nCCLs; ++iCCL)
			{
				std::string ccl_name = vm_it->second[iCCL];

				SegmentMap::iterator s_it = organized_segments_[i].find(ccl_name);
				if (s_it != organized_segments_[i].end())
				{
					s_it->second.push_back(key);
				}
				else
				{
					CCLKeyList newItem;
					newItem.push_back(key);
					organized_segments_[i].insert(std::make_pair(ccl_name, newItem));
				}
			}
			
		}
	}

	return 0;
}

template<typename PointT> int
MapUpdater<PointT>::refineSegments(double vsize)
{
	plane_refiner_.reset(new PlaneRefinement<PointT>);
	refined_clusters_.clear();
	
	size_t nSegs = organized_segments_.size();

	for (size_t i = 0; i < nSegs; ++i)
	{
		size_t nCCLs = organized_segments_[i].size(); //当前seg包含的CCL数 
		plane_refiner_->reinitialize();

		SegmentMap::iterator sit = organized_segments_[i].begin();
		for (; sit != organized_segments_[i].end(); ++sit)
		{//提取ccl_i的PointGeoSegment
			std::string pgname = sit->first;
			std::vector<int> indices;
			int ret = load_Indices(pgname, sit->second, indices);
			if (ret != 0)
			{
				std::cout << "can not load indices in " << pgname << std::endl;
				continue;
			}

			std::vector<ref_ptr<PointGeoSegment<PointT> > > clusters;
			ret = get_SelectedPointClusters(pgname, indices, clusters);
			if (ret != 0)
			{
				std::cout << "can not get point cluster: " << pgname << std::endl;
				continue;
			}

			plane_refiner_->addPlanes(clusters);

#ifdef _DEBUGGING_
			std::string debug_dir = "D:/iplTestData/TargetLocalization/mapupdate/gcm/debug";
			for (int ic=0; ic < clusters.size(); ++ic)
			{
				char buf[32];
				sprintf(buf, "seg_%d_%d.pcd", i, ic);
				std::string segName;
				segName = debug_dir + "/" + buf;

				write_PointCloud(segName, *(clusters[ic]->getInputCloud()), *(clusters[ic]->getIndices()), true);
			}

#endif
		}

		plane_refiner_->refine(vsize);
		const std::vector<ref_ptr<PointGeoSegment<PointT> > > refined_clusters = plane_refiner_->getRefinedClusters();

		refined_clusters_.insert(refined_clusters_.end(), refined_clusters.begin(), refined_clusters.end());
	}

	return 0;
}


template<typename PointT> int
MapUpdater<PointT>::load_Indices(std::string &pgName, CCLKeyList &keylist, std::vector<int> &indices)
{
	indices.clear();

	if (!check_exist(pgName))
	{
		std::cout << pgName << " is not existing!" << std::endl;
		return -1;
	}

	std::string idxfolder;

	idxfolder = pgName + "/" + GCMIndicesFolder;
	if (!check_exist(idxfolder))
	{
		std::cout << idxfolder << " is not existing!" << std::endl;
		return -1;
	}

	std::string idx_name;
	for (size_t i = 0; i < keylist.size(); ++i)
	{
		char buf[32];
		iplOctreeKey key = keylist[i];
		sprintf(buf, "key_%d_%d_%d%s", key.x, key.y, key.z, CCLIndiceFile_Suffix);

		idx_name = idxfolder + "/" + buf;

		std::ifstream  ifs;
		ifs.open(idx_name);
		if (!ifs.is_open())
		{
			std::cout << "error: can't create " << idx_name << std::endl;
			return (-1);
		}

		int num;
		ifs >> num;
		for (size_t j = 0; j < num; ++j)
		{
			int v = -1;
			ifs >> v;
			indices.push_back(v);
		}
		ifs.close();
	}

	return 0;
}

template<typename PointT> int
MapUpdater<PointT>::get_SelectedPointClusters(const std::string &pgName, const std::vector<int> &sel_indices,
	std::vector<ref_ptr<PointGeoSegment<PointT> > > &clusters)
{
	geoModel::PointGroup<PointT> ptGroup;

	int ret = load_PointGroup(pgName, ptGroup);
	if (ret != 0)
	{
		std::cout << "can not open " << pgName << std::endl;
		return ret;
	}

	//model ID, indices
	typedef boost::unordered_map<int, ref_ptr<std::vector<int> > > CLSIndicesMap;
	CLSIndicesMap indiceMap;
	CLSIndicesMap::iterator it_CI; 

	for (size_t i = 0; i < sel_indices.size(); ++i)
	{
		int pID = sel_indices[i];      // selected point ID
		int mID = ptGroup.lut[pID];    // model ID

		it_CI = indiceMap.find(mID);
		if (it_CI != indiceMap.end())
		{
			it_CI->second->push_back(pID);
		}
		else
		{
			ref_ptr<std::vector<int> > newItem(new std::vector<int>);
			newItem->push_back(pID);
			indiceMap.insert(std::make_pair(mID, newItem));
		}
	}

	size_t clsNum = indiceMap.size();
	clusters.clear();
	for (it_CI = indiceMap.begin(); it_CI != indiceMap.end(); ++it_CI)
	{
		if(it_CI->second->size()<minSeg_)
			continue;

		ref_ptr<PointGeoSegment<PointT> > pgSeg(new PointGeoSegment<PointT>);

		pgSeg->setInputCloud(ptGroup.cloud);
		pgSeg->setIndices(it_CI->second);
		pgSeg->setModelCoef(ptGroup.modelParams[it_CI->first]);

		clusters.push_back(pgSeg);
	}

	return 0;
}

template<typename PointT> int
MapUpdater<PointT>::saveResults(std::string output_folder)
{
	if(!check_exist(output_folder))
		create_folder(output_folder);

	empty_folder(output_folder);

	std::vector<ref_ptr<PointGeoSegment<PointT> > > ceilingCls, wallCls;

	Eigen::Vector3f  nVertical = Eigen::Vector3f(0, 0, 1);

	float ver_degTh = 15.0, hor_degTh = 80.0;
	float th_ver = cosf(pcl::deg2rad(ver_degTh));
	float th_hor = cosf(pcl::deg2rad(hor_degTh));

	//分组
	for (size_t i = 0; i < refined_clusters_.size(); ++i)
	{
		geoModel::geoModelInfo coef = refined_clusters_[i]->getModelCoef();

		Eigen::Map<Eigen::Vector3f> plane_normal(static_cast<float*> (&coef.coef.values[0]));
		float dot_product = fabsf(plane_normal.dot(nVertical));
		if (dot_product > th_ver)
		{
			ceilingCls.push_back(refined_clusters_[i]);
		}
		else if (dot_product < th_hor)
		{
			wallCls.push_back(refined_clusters_[i]);
		}
	}

	if (ceilingCls.size() > 0)
	{
		std::string folderName = output_folder /*+ "/" + CeilingFolder*/;
		create_folder(folderName);
		empty_folder(folderName);

		for (size_t i = 0; i < ceilingCls.size(); ++i)
		{
			std::string pt_name, param_name;
			char buf[32], buf_para[32];
			sprintf(buf, "update_%04d.pcd", i);
			sprintf(buf_para, "update_%04d%s", i, geoModel::ModelParamFile_Suffix);

			pt_name = folderName + "/" + buf;
			param_name = folderName + "/" + buf_para;

			//writer.write<PointT>(out_name, *sort_cloud, inliers[i], true);
			write_PointCloud(pt_name, *ceilingCls[i]->getInputCloud(), *ceilingCls[i]->getIndices(), true);

			save_geoModel_params(param_name, ceilingCls[i]->getModelCoef());
		}
	}

	if (wallCls.size() > 0)
	{
		std::string folderName = output_folder/* + "/" + WallFolder*/;
		create_folder(folderName);
		empty_folder(folderName);

		for (size_t i = 0; i < wallCls.size(); ++i)
		{
			std::string pt_name, param_name;
			char buf[32], buf_para[32];
			sprintf(buf, "update_%04d.pcd", i);
			sprintf(buf_para, "update_%04d%s", i, geoModel::ModelParamFile_Suffix);

			pt_name = folderName + "/" + buf;
			param_name = folderName + "/" + buf_para;

			//writer.write<PointT>(out_name, *sort_cloud, inliers[i], true);
			write_PointCloud(pt_name, *wallCls[i]->getInputCloud(), *wallCls[i]->getIndices(), true);

			save_geoModel_params(param_name, wallCls[i]->getModelCoef());
		}
	}

	return 0;
}

}

