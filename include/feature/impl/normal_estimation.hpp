#pragma once
#include "feature/normal_estimation.h"

//CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/pca_estimate_normals.h>
#include <CGAL/mst_orient_normals.h>
#include <CGAL/property_map.h>
//#include <CGAL/IO/read_xyz_points.h>
#include <utility> // defines std::pair
#include <list>
#include <fstream>


namespace ipl
{
template <typename PointT>
int estimate_normals_CGAL_PCA(const iplPointCloud<PointT> &cloud, const std::vector<int> *indices,
	std::vector<iplNORMAL3D> &normals)
{
	// Types
	typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
	typedef Kernel::Point_3 Point;
	typedef Kernel::Vector_3 Vector;
	// Point with normal vector stored in a std::pair.
	typedef std::pair<Point, Vector> PointVectorPair;
	// Concurrency
#ifdef CGAL_LINKED_WITH_TBB
	typedef CGAL::Parallel_tag Concurrency_tag;
#else
	typedef CGAL::Sequential_tag Concurrency_tag;
#endif

	std::vector<PointVectorPair> points;

	size_t nSample = 0;
	if (indices)
		nSample = indices->size();
	else
		nSample = cloud.size();

	points.reserve(nSample);
	Vector normal0 = CGAL::NULL_VECTOR;
	for (int i = 0; i < nSample; i++)
	{
		int id = i;
		if (indices)
			id = indices->at(i);

		Point pt;
		pt = Point(cloud.points[id].x, cloud.points[id].y, cloud.points[id].z);
		points.push_back(std::make_pair(pt, normal0));
	}

	// Estimates normals direction.
	// Note: pca_estimate_normals() requiresa range of points
	// as well as property maps to access each point's position and normal.
	const int nb_neighbors = 18; // K-nearest neighbors = 3 rings
	CGAL::pca_estimate_normals<Concurrency_tag>(points.begin(), points.end(),
		CGAL::First_of_pair_property_map<PointVectorPair>(),
		CGAL::Second_of_pair_property_map<PointVectorPair>(),
		nb_neighbors);

	normals.resize(nSample);
	for (int i = 0; i < nSample; i++)
	{
		normals[i].nx = points[i].second.x();
		normals[i].ny = points[i].second.y();
		normals[i].nz = points[i].second.z();
	}

	return 0;
}

}

