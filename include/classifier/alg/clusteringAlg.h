#pragma once
#include "classifier/alg/DBSCAN.h"
//#include "classifier/alg/OPTICS.h"
//#include "classifier/alg/clustering_density_peaks.h"

#include "core/iplcore.h"

namespace ipl
{
	template <typename PointT>
	int clustering_by_DBSCAN(const pcl::PointCloud<PointT> &cloud, std::vector<int> *indices, double Eps, int MinPts,
		std::vector<int> &cluster_center_idx, std::vector<uint32_t> &labels, std::vector<double> &rhosout)
	{
		int nSamples;
		if (indices)
			nSamples = indices->size();
		else
			nSamples = cloud.size();

		DBS::point_t *points = new DBS::point_t[nSamples];

		for (int i = 0; i < nSamples; i++)
		{
			int id;

			if (indices)
				id = indices->at(i);
			else
				id = i;

			points[i].x = cloud.points[id].x;
			points[i].y = cloud.points[id].y;
			points[i].z = cloud.points[id].z;
			points[i].cluster_id = UNCLASSIFIED;
		}


		DBS::dbscan(points, nSamples, Eps, MinPts, DBS::euclidean_dist);


		labels.resize(nSamples);
		for (int i = 0; i < nSamples; i++)
		{
			labels[i] = points[i].cluster_id;
		}

		rhosout.assign(nSamples, 0);

		if (points)	delete[] points;   points = NULL;

		return 0;
	}

// 	template <typename PointT>
// 	int clustering_by_optics(const pcl::PointCloud<PointT> &cloud, double Eps, int MinPts,
// 		std::vector<int> &cluster_center_idx, std::vector<uint32_t> &labels, std::vector<double> &rhosout)
// 	{
// 		int nSamples = cloud.points.size();
// 
// 		opt_point_t *points = new opt_point_t[nSamples];
// 
// 		for (int i = 0; i < nSamples; i++)
// 		{
// 			points[i].x = cloud.points[i].x;
// 			points[i].y = cloud.points[i].y;
// 			points[i].z = cloud.points[i].z;
// 			points[i].cluster_id = UNCLASSIFIED;
// 		}
// 
// 
// 		optics(points, nSamples, Eps, MinPts, euclidean_dist);
// 
// 
// 		labels.resize(nSamples);
// 		for (int i = 0; i < nSamples; i++)
// 		{
// 			labels[i] = points[i].cluster_id;
// 		}
// 
// 		rhosout.assign(nSamples, 0);
// 
// 		if (points)	delete[] points;   points = NULL;
// 
// 		return 0;
// 	}

}
