#pragma once
#include <core/iplcore.h>

namespace ipl
{
	//the base class for label optimization
	class LabelOptimization
	{
	public:
		LabelOptimization()
		{

		}

		virtual ~LabelOptimization()
		{

		}

		virtual std::vector<std::vector<int> > getLabels() = 0;
		
		virtual bool beginBuildGraph(int vextex_num) = 0;

		virtual bool endBuildGraph() = 0;

		virtual bool
			add_UnaryCost(int key, double source_weight, double sink_weight) = 0;

		virtual bool
			add_UndirectedBinaryCost(int source_key, int target_key, double edge_weight) = 0;

		virtual bool
			add_DirectedBinaryCost(int source_key, int target_key, double edge_weight) = 0;

	protected:

		/** \brief This method analyzes the residual network and assigns a label to every point in the cloud.
		* \param[in] residual_capacity residual network that was obtained during the segmentation
		*/
// 		virtual void
// 			assembleLabels(/*ResidualCapacityMap& residual_capacity*/) = 0;

	private:

	};
}

