#pragma once
#include "core/iplcore.h"


namespace ipl
{
	template <typename PointT>
	class ProfileExtraction : public iplPointCluster<PointT>
	{
	public:
		ProfileExtraction();
		~ProfileExtraction();

		int extractHeightProfile(float zmin, float zmax, std::vector <int> &profile_Indices);

	};
}

#include "classifier/impl/ProfileExtraction.hpp"
