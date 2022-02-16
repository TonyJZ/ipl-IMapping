#pragma once
#include "core/iplcore.h"
#include "fitting/pointcloud_clipper.h"
#include "feature/PointGeoSegment.h"

#include <pcl/search/kdtree.h>

namespace ipl
{
	template <typename PointT>
	class CandidateChangeExtractor
	{
		typedef pcl::search::Search <PointT> KdTree;
		typedef typename KdTree::Ptr KdTreePtr;

	public:
		CandidateChangeExtractor();
		~CandidateChangeExtractor();

		void set_refData(const std::vector<std::string> &pszRefFiles);
		void set_tarData(const std::vector<std::string> &pszTarFiles);
		void set_Transformation(const std::string filename);
		void set_distance_threshold(double th = 0.2) { disTH_ = th; };

		void extractCandidateChange(geoModel::PointGroup<PointT> &cand_group);

	protected:


	private:
		Eigen::Affine3d  transform_;  //the initial matching result

		ref_ptr<PointCloudClipper<PointT> > data_clipper_; //Êý¾Ý²Ã¼ôÆ÷
		double disTH_;

		KdTreePtr search_;  
		ref_ptr<iplPointCloud<PointT> > trans_cloud_; 
	};
}


#include "MapUpdate/impl/CandidateChangeExtractor.hpp"
