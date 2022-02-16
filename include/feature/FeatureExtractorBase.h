#pragma once

#include "core/iplcore.h"
#include "feature/FeatureSetBase.h"

namespace ipl
{
//	typedef boost::shared_ptr<const FeatureSetBase> ConstFeaturePtr;
	//a base class for feature extraction
	template <typename PointT>
	class FeatureExtractor : public ipl::iplPointCluster<PointT>
	{
	public:
		FeatureExtractor()
		{

		}

// 		virtual ~FeatureExtractor()
// 		{
// 			feature_.reset();
// 		}

		virtual int extract() = 0;

// 		virtual ConstFeaturePtr getFeatures()
// 		{
// 			return features_;
// 		}

	protected:
//		boost::shared_ptr<FeatureSetBase> features_;
	};
}

