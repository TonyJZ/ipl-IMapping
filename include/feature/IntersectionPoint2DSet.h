#pragma once
#include <core/iplcore.h>
#include <feature/FeatureSetBase.h>

namespace ipl {

	//class for intersection point feature set
	class IPL_BASE_API IntersectionPoint2DSet : public ipl::FeatureSetBase
	{
	public:
		IntersectionPoint2DSet();
		~IntersectionPoint2DSet();

		void addEdgeIndice(const edgeINDICE &indice)
		{
			connected_indices_.push_back(indice);
		};

		inline std::vector<edgeINDICE> const
			getEdgeIndices() const { return (connected_indices_); };

		void addEdgeAtt(const edgeATTRIBUTES &att)
		{
			connected_atts_.push_back(att);
		};

		inline std::vector<edgeATTRIBUTES> const
			getEdgeAttributes() const { return connected_atts_; };

		void addIntersectionVariance(std::vector<double> var)
		{
			intersection_variances_.push_back(var);
		};

		inline std::vector<std::vector<double> > const
			getIntersectionVariances() const { return (intersection_variances_); };

	
	protected:
		//std::vector<float>  weights_;
//		std::vector<std::string> plane_names_;
		std::vector<edgeINDICE> connected_indices_; //the connected edge's id
		std::vector<edgeATTRIBUTES> connected_atts_;
		std::vector<std::vector<double> > intersection_variances_;

	private:

	};

	/** \brief convert IntersectionPoint2DSet to std::vector<IntersectionPoint>
	* \param[in] feat_set
	* \param[out] ipt2ds: 
	* \return
	*  * < 0 (-1) on error
	*  * == 0 on success
	*/
	IPL_BASE_API int convert_FeatureStructure(const IntersectionPoint2DSet &feat_set, std::vector<IntersectionPoint> &ipt2ds);
}

