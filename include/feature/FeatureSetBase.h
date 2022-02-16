#pragma once

#include <Feature/featureDef.h>

namespace ipl
{
	using namespace ipl::feature;
// 	struct Feature
// 	{
// 		std::vector<double> position_;
// 		std::vector<float> description_;
// 	};

	//a general class for feature set
	class FeatureSetBase
	{
	public:
		FeatureSetBase()
		{

		}

// 		FeatureBase(const FeatureBase& base)
// 			: position_(base.position_)
// 			, description_(base.description_)
// 		{
// 
// 		}

		inline std::vector<POSITION> const
			getPositions() const { return (pos_); }

		inline std::vector<DESCRIPTOR> const
			getDescriptors() const { return (desc_); }

		inline void addPositions(const std::vector<POSITION> &pos)
		{ 
			pos_.insert(pos_.end(), pos.begin(), pos.end()); 
		}

		inline void addOnePosition(const POSITION &p)
		{
			pos_.push_back(p); 
		}

		inline void addDescriptors(const std::vector<DESCRIPTOR> &desc)
		{
			desc_.insert(desc_.end(), desc.begin(), desc.end());
		}

		inline void addOneDescriptor(const DESCRIPTOR &d)
		{
			desc_.push_back(d);
		}

		inline void addOneFeature(const POSITION &p, const DESCRIPTOR &d)
		{
			addOnePosition(p);
			addOneDescriptor(d);
		}

		inline void addFeatures(const std::vector<POSITION> &pos, const std::vector<DESCRIPTOR> &desc)
		{
			addPositions(pos);
			addDescriptors(desc);
		}

	protected:
		std::vector<POSITION> pos_;
		std::vector<DESCRIPTOR> desc_;

// 	public:
// 		typedef boost::shared_ptr< ::pcl::ModelCoefficients> Ptr;
// 		typedef boost::shared_ptr< ::pcl::ModelCoefficients  const> ConstPtr;
 	}; 


} // namespace ipl

