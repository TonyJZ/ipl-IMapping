#include "MapUpdate/ChangeVoxelSegmentIO.h"
#include "commonAPIs/iplutility.h"

#include <fstream>

const char SEGMENT_FLAG[] = "_segment_";

namespace ipl
{
	int load_voxelPointIndices(const std::string &indices_path, double ori[3], double vsize[3], VoxelKeyMap &vmap)
	{
		if (!check_exist(indices_path))
		{
			std::cout << indices_path << " is not existing!" << std::endl;
			return -1;
		}


		//1. 读GCM参数
		std::string gcmParamfile;

		gcmParamfile = indices_path + "/" + GCMParamsName;
		std::ifstream  ifs;
		ifs.open(gcmParamfile);

		if (!ifs.is_open())
		{
			std::cout << "error: can't open " << gcmParamfile << std::endl;
			return (-1);
		}

		std::string aline;

		ifs >> aline; // "This_is_GCM_origin:" 
		ifs >> ori[0] >> ori[1] >> ori[2];
		ifs >> aline; // "This_is_GCM_voxel_size:"
		ifs >> vsize[0] >> vsize[1] >> vsize[2];
		ifs.close();

		//2. 读voxel索引
		std::vector<std::string>  idx_names;
		scan_files(indices_path, CCLIndiceFile_Suffix, idx_names);
		vmap.clear();
		for (size_t i = 0; i < idx_names.size(); ++i)
		{
			char buf1[32], buf2[32], buf3[32];
			iplOctreeKey key;

			sscanf(idx_names[i].c_str(), "_%s_%s_%s.idx", buf1, buf2, buf3);
			key.x = atoi(buf1);
			key.y = atoi(buf2);
			key.z = atoi(buf3);

			ipl::VoxelContainerPointIndices newVoxel;

			ifs.open(idx_names[i]);
			if (!ifs.is_open())
			{
				std::cout << "error: can't create " << idx_names[i] << std::endl;
				return (-1);
			}

			int num;
			ifs >> num;
			for (size_t i = 0; i < num; i++)
			{
				int v = -1;
				ifs >> v;
				newVoxel.addPointIndex(v);
			}
			ifs.close();

			vmap.insert(std::make_pair(key, newVoxel));
		}

		return 0;
	}

	int save_voxelPointIndices(const std::string &indices_path, const double ori[3], const double vsize[3], VoxelKeyMap &vmap)
	{
		if (!check_exist(indices_path))
			create_folder(indices_path);

		//1. 写GCM参数
		std::string gcmParamfile;
		gcmParamfile = indices_path + "/" + GCMParamsName;

		std::ofstream  ofs;
		ofs.open(gcmParamfile);

		if (!ofs.is_open())
		{
			std::cout << "error: can't create " << gcmParamfile << std::endl;
			return (-1);
		}

		ofs << "This_is_GCM_origin:" << std::endl;
		ofs << ori[0] << " " << ori[1] << " " << ori[2] << std::endl;
		ofs << "This_is_GCM_voxel_size:" << std::endl;
		ofs << vsize[0] << " " << vsize[1] << " " << vsize[2] << std::endl;
		ofs.close();

		//2. 写voxel索引
		std::string idx_name;
		for (VoxelKeyMap::iterator it = vmap.begin(); it != vmap.end(); ++it)
		{
			char buf[32];
			iplOctreeKey key = it->first;
			sprintf(buf, "key_%d_%d_%d%s", key.x, key.y, key.z, CCLIndiceFile_Suffix);

			idx_name = indices_path + "/" + buf;
			ofs.open(idx_name);
			if (!ofs.is_open())
			{
				std::cout << "error: can't create " << idx_name << std::endl;
				return (-1);
			}

			std::vector<int>* indices = it->second.getPointIndices();
			ofs << indices->size() << std::endl;
			for (size_t i = 0; i < indices->size(); ++i)
			{
				ofs << (*indices)[i] << " ";
			}

			ofs.close();
		}

		return 0;
	}

	int load_ChangeVoxelSegments(const std::string &filename, std::vector<ChangeVoxelMap> &cseg_map)
	{
		std::ifstream  ifs;
		ifs.open(filename);

		if (!ifs.is_open())
		{
			std::cout << "error: can't open " << filename << std::endl;
			return (-1);
		}

		cseg_map.clear();

		std::string strFlag;
		ifs >> strFlag;

		while (!ifs.eof())
		{
			ifs >> strFlag;

			if (strFlag.compare(SEGMENT_FLAG) == 0)
			{
				ChangeVoxelMap  seg;
				int num_voxel;
				ifs >> num_voxel;

				for(int i=0; i<num_voxel; ++i)
				{
					iplOctreeKey key;
					int num_survey;

					ifs >> key.x >> key.y >> key.z >> num_survey;
					CCLNameList surList;
					for (int j = 0; j < num_survey; j++)
					{
						std::string name;
						ifs >> name;
						surList.push_back(name);
					}

					seg.insert(std::make_pair(key, surList));
				}
				
				cseg_map.push_back(seg);
			}
		}
		ifs.close();

		return 0;
	}

	int save_ChangeVoxelSegments(const std::string &filename, std::vector<ChangeVoxelMap> &cseg_map)
	{
		std::ofstream  ofs;
		ofs.open(filename);

		if (!ofs.is_open())
		{
			std::cout << "error: can't create " << filename << std::endl;
			return (-1);
		}

		ofs << "This_is_CSM_file" << std::endl;

		for (size_t i = 0; i < cseg_map.size(); ++i)
		{
			ofs << SEGMENT_FLAG << std::endl;
			ofs << cseg_map[i].size() << std::endl; //包含多少个voxel

			ChangeVoxelMap::iterator it = cseg_map[i].begin();
			while (it != cseg_map[i].end())
			{
				ofs << it->first.x << " "
					<< it->first.y << " "
					<< it->first.z << " "
					<< it->second.size() << std::endl; //key, nsurvey


				for (size_t j = 0; j < it->second.size(); ++j)
				{
					ofs << it->second[j] << std::endl;   //文件名中不可有空格
				}
					
				++it;
			}
		}

		ofs.close();

		return 0;
	}
}
