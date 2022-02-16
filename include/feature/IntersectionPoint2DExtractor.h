#pragma once
#include "core/iplcore.h"
#include "feature/FeatureExtractorBase.h"
#include "feature/IntersectionPoint2DSet.h"
#include "feature/PointGeoSegment.h"

namespace ipl
{
	//typedef boost::shared_ptr<const IntersectionPoint2DSet> ConstFeaturePtr;
	//2D ͶӰ���ͳһ���ֵ
	const float PROJECTED2D_Z = 10.0;

	template <typename PointT>
	class IntersectionPoint2DExtractor : public ipl::FeatureExtractor<PointT>
	{
	public:
		IntersectionPoint2DExtractor(const std::vector<std::string> &filenames)
		{
			filenames_ = filenames;
		}

		virtual ~IntersectionPoint2DExtractor()
		{
			features_.reset();
		}

		virtual int extract();

		virtual IntersectionPoint2DSet* getFeatures()
		{
			return features_.get();
		}

	protected:
		//����seg i �� seg j ��������ķ���
		int EstimatorVariance(int i, int j, std::vector<double> &intersectionVar); 

	protected:
		std::vector<ipl::PointGeoSegment<PointT> > segments_;

		ref_ptr<IntersectionPoint2DSet> features_;

	private:
		std::vector<std::string>  filenames_;
	};
}


#include <feature/impl/IntersectionPoint2DExtractor.hpp>
