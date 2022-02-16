#pragma once
#include "core/iplcore.h"
#include "feature/PointGeoSegment.h"
#include "spatialindexing/ClusterOverlappingByVoxelMap.h"

namespace ipl
{
	template<typename PointT>
	class PlaneRefinement
	{
	public:
		PlaneRefinement();
		~PlaneRefinement();

		void reinitialize();

		void addPlane(ref_ptr<PointGeoSegment<PointT> > &cluster);
		void addPlanes(std::vector<ref_ptr<PointGeoSegment<PointT> > > &clusters);

		int refine(double vsize = 0.2);

		const std::vector<ref_ptr<PointGeoSegment<PointT> > >& getRefinedClusters() const
		{
			return refined_clusters_;
		};

	protected:


	private:
		float connective_ratio_;
		
		std::vector<ref_ptr<PointGeoSegment<PointT> > > clusters_; 
		ref_ptr<ClusterOverlappingByVoxelMap<PointT> >  connMap_;

		std::vector<ref_ptr<PointGeoSegment<PointT> > > refined_clusters_;

	};
}

#include "fitting/impl/PlaneRefinement.hpp"
