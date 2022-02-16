#pragma once
/*
* Software License Agreement (Proprietary License)
*
*
*  Copyright (c) 2017-, Appropolis, Inc.
*
*  All rights reserved.
*
*  rights described below...
*
*/
#include "core/ipldef.h"
#include "fitting/pcPlaneDetection.h"
#include "fitting/fitting_assessment.h"
#include "feature/normal_estimation.h"

#if HAVE_CGAL
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//#include <CGAL/IO/read_xyz_points.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/property_map.h>
#include <CGAL/Timer.h>
#include <CGAL/number_utils.h>
#include <CGAL/Shape_detection_3.h>
#endif

//#include <iostream>
//#include <fstream>
namespace ipl
{
template <typename PointT>
int detect_planes_by_eRansac(const iplPointCloud<PointT> &cloud, std::vector<int> *indices, bool has_normal,
	iplPointCloud<PointT> &sorted_cloud,
	std::vector<std::vector<int> > &inliers,
	std::vector<ipl::geoModel::geoModelInfo> &planeCoeffs,
	int minModelPts/* = 200*/, double epsTh/* = 0.05*/, double epsConnective/* = 0.2*/, double normalRadianTh/* = 0.9*/)
{
	// Type declarations
	typedef CGAL::Exact_predicates_inexact_constructions_kernel		Kernel;
	typedef Kernel::FT												FT;
	typedef Kernel::Point_3											Point;
	typedef Kernel::Vector_3										Vector;
	//typedef std::pair<Point, int>									PointID;
	typedef std::pair<Point, Vector>								Point_with_normal;
	typedef std::vector<Point_with_normal>						Pwn_vector;
	typedef CGAL::First_of_pair_property_map<Point_with_normal>	Point_map;
	typedef CGAL::Second_of_pair_property_map<Point_with_normal>	Normal_map;
	// In Efficient_RANSAC_traits the basic types, i.e., Point and Vector types
	// as well as iterator type and property maps, are defined.
	typedef CGAL::Shape_detection_3::Efficient_RANSAC_traits<Kernel, Pwn_vector, Point_map, Normal_map> Traits;
	typedef CGAL::Shape_detection_3::Efficient_RANSAC<Traits>	Efficient_ransac;
	typedef CGAL::Shape_detection_3::Plane<Traits>              Plane;

	// Points with normals.
	Pwn_vector points;
	//	std::vector<Point> points1;
	std::vector<iplNORMAL3D> myNormals;

	if (!has_normal)
		estimate_normals_CGAL_PCA(cloud, indices, myNormals);

	size_t nSample = 0;
	if (indices)
		nSample = indices->size();
	else
		nSample = cloud.size();

	points.reserve(nSample);
	//points.resize(10);
	for (int i = 0; i < nSample; i++)
	{
		int id = i;
		if (indices)
			id = indices->at(i);

		Point pt;
		Vector normal = CGAL::NULL_VECTOR;

		// 		eRansac::Point Pt;
		// 		//default normal
		// 		Pt.normal[0] = 0.0;
		// 		Pt.normal[1] = 0.0;
		// 		Pt.normal[2] = 0.0;

		//pt = PointID(Point(FT(cloud.points[id].x), FT(cloud.points[id].y), FT(cloud.points[id].z)), i);
		pt = Point(FT(cloud.points[id].x), FT(cloud.points[id].y), FT(cloud.points[id].z));

		//FT temp = pt.x();

		if (has_normal)
		{
			normal = Vector(cloud.points[id].normal_x, cloud.points[id].normal_y, cloud.points[id].normal_z);
			// 			Pt.normal[0] = static_cast<float>(cloud.points[id].normal_x);
			// 			Pt.normal[1] = static_cast<float>(cloud.points[id].normal_y);
			// 			Pt.normal[2] = static_cast<float>(cloud.points[id].normal_z);
		}
		else
		{
			normal = Vector(myNormals[i].nx, myNormals[i].ny, myNormals[i].nz);
		}
		//		points1.push_back(pt);
		points.push_back(Point_with_normal(pt, normal));
	}

	// Instantiates shape detection engine.
	Efficient_ransac ransac;
	// Provides the input data.
	ransac.set_input(points);
	// Registers detection of planes
	ransac.add_shape_factory<Plane>();
	// Measures time before setting up the shape detection.
	CGAL::Timer time;
	time.start();
	// Build internal data structures.
	ransac.preprocess();
	// Measures time after preprocessing.
	time.stop();
	std::cout << "preprocessing took: " << time.time() * 1000 << "ms" << std::endl;
	// Perform detection several times and choose result with highest coverage.
	// Sets parameters for shape detection.
	Efficient_ransac::Parameters parameters;
	// Sets probability to miss the largest primitive at each iteration.
	parameters.probability = 0.05;
	// Detect shapes with at least 500 points.
	parameters.min_points = minModelPts;
	// Sets maximum Euclidean distance between a point and a shape.
	parameters.epsilon = epsTh;
	// Sets maximum Euclidean distance between points to be clustered.
	parameters.cluster_epsilon = epsConnective;
	// Sets maximum normal deviation.
	// 0.9 < dot(surface_normal, point_normal); 
	parameters.normal_threshold = cos(normalRadianTh*iplDEG2RAD);
	// Detects shapes
	ransac.detect(parameters);

	typedef pcl::PointXYZRGBNormal PCLPointT;

	//�ش�����
	sorted_cloud.clear();
	sorted_cloud.reserve(points.size());
	for (Pwn_vector::iterator ptIt = points.begin();
		ptIt != points.end(); ++ptIt)
	{
		PCLPointT pcl_pt;
		pcl_pt.x = ptIt->first.x();
		pcl_pt.y = ptIt->first.y();
		pcl_pt.z = ptIt->first.z();

		if (has_normal)
		{
			pcl_pt.normal_x = ptIt->second.x();
			pcl_pt.normal_y = ptIt->second.y();
			pcl_pt.normal_z = ptIt->second.z();
		}

		sorted_cloud.push_back(pcl_pt);
	}

	Efficient_ransac::Shape_range shapes = ransac.shapes();
	/*	FT best_coverage = 0;
	for (size_t i = 0; i < 3; i++)
	{
	// Reset timer.
	time.reset();
	time.start();
	// Detects shapes.
	ransac.detect();
	// Measures time after detection.
	time.stop();
	// Compute coverage, i.e. ratio of the points assigned to a shape.
	FT coverage = FT(points.size() - ransac.number_of_unassigned_points())
	/ FT(points.size());
	// Prints number of assigned shapes and unsassigned points.
	std::cout << "time: " << time.time() * 1000 << "ms" << std::endl;
	std::cout << ransac.shapes().end() - ransac.shapes().begin() << " primitives, "
	<< coverage << " coverage" << std::endl;

	// Choose result with highest coverage.
	if (coverage > best_coverage) {
	best_coverage = coverage;
	// Efficient_ransac::shapes() provides
	// an iterator range to the detected shapes.
	shapes = ransac.shapes();
	}
	}*/
	Efficient_ransac::Shape_range::iterator it = shapes.begin();

	std::vector<double> Xcoords, Ycoords, Zcoords;
	while (it != shapes.end())
	{
		boost::shared_ptr<Efficient_ransac::Shape> shape = *it;
		// Using Shape_base::info() for printing 
		// the parameters of the detected shape.
		std::cout << (*it)->info();
		if (Plane* plane = dynamic_cast<Plane*>(it->get()))
		{
			Kernel::Vector_3 normal = plane->plane_normal();
			std::cout << "Plane with normal " << normal << std::endl;
			ipl::geoModel::geoModelInfo  pInfo;

			pInfo.type = ipl::geoModel::gMT_PLANE;

			//���ƽ�����
			iplModelCoeffs coef;
			coef.values.resize(4);
			coef.values[0] = normal.hx();
			coef.values[1] = normal.hy();
			coef.values[2] = normal.hz();
			coef.values[3] = plane->d();

			pInfo.coef = coef;

			// planeCoeffs.push_back(coef);
			// Plane shape can also be converted to Kernel::Plane_3
			//std::cout << "Kernel::Plane_3: " << static_cast<Kernel::Plane_3>(*plane) << std::endl;

			// Sums distances of points to detected shapes.
			FT sum_dis2 = 0;
			// Iterates through point indices assigned to each detected shape.
			std::vector<std::size_t>::const_iterator
				index_it = (*it)->indices_of_assigned_points().begin();
			std::vector<int> plane_ptIdx;

			while (index_it != (*it)->indices_of_assigned_points().end())
			{
				int id;
				// 				if (indices)
				// 					id = indices->at(*index_it);
				// 				else
				id = *index_it;

				plane_ptIdx.push_back(id);
				// Retrieves point
				const Point_with_normal &p = *(points.begin() + (*index_it));
				//const Point &pt3 = p.first.first;
				// Adds Euclidean distance between point and shape.
				//sum_distances += CGAL::sqrt((*it)->squared_distance(p.first));
				sum_dis2 += (*it)->squared_distance(p.first);
				// Proceeds with next point.
				index_it++;
			}
			// Computes and prints average distance.
			FT se = CGAL::sqrt(sum_dis2 / (shape->indices_of_assigned_points().size() - 4));  //4����С�۲�
			std::cout << " average distance: " << se << std::endl;

			//pInfo.fittingSE = se;

			size_t num_plane_pts = plane_ptIdx.size();
			if (num_plane_pts > minModelPts)
			{
				inliers.push_back(plane_ptIdx);

				Xcoords.resize(num_plane_pts);
				Ycoords.resize(num_plane_pts);
				Zcoords.resize(num_plane_pts);

				for (size_t i = 0; i < num_plane_pts; ++i)
				{
					int id = plane_ptIdx[i];
					Xcoords[i] = sorted_cloud.points[id].x;
					Ycoords[i] = sorted_cloud.points[id].y;
					Zcoords[i] = sorted_cloud.points[id].z;
				}

				plane_fitting_covariance(&Xcoords[0], &Ycoords[0], &Zcoords[0], num_plane_pts,
					pInfo);

				planeCoeffs.push_back(pInfo);
			}

		}

		// Proceeds with next detected shape.
		it++;
	}

	return 0;
}
}
