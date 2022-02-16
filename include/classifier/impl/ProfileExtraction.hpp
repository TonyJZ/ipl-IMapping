#pragma once
#include "classifier/ProfileExtraction.h"

namespace ipl
{
	template <typename PointT>
	ProfileExtraction<PointT>::ProfileExtraction()
	{

	}

	template <typename PointT>
	ProfileExtraction<PointT>::~ProfileExtraction()
	{

	}

	template <typename PointT> int
		ProfileExtraction<PointT>::extractHeightProfile(float zmin, float zmax, std::vector<int> &profile_Indices)
	{
		bool bReady = initCompute();
		if (!bReady)
		{
			deinitCompute();
			return -1;
		}

		profile_Indices.clear();
		for (int i = 0; i < indices_->size(); ++i)
		{
			int id = indices_->at(i);
			PointT pt = input_->points[id];

			if (pt.z > zmin && pt.z < zmax)
				profile_Indices.push_back(id);
		}

		return 0;
	}
}

