#pragma once
#include "fitting/alpha_shapes_2d.h"


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Weighted_alpha_shape_euclidean_traits_2.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Alpha_shape_2.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <list>


namespace ipl
{
template <typename PointT>
int alpha_shape_2d(const pcl::PointCloud<PointT> &cloud, std::vector<int> *indices, bool has_weight,
	std::vector<float> &weights,
	OGRPolygon &line_segs,
	float alpha)
{
// 	typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
// 	typedef K::FT FT;
// 	typedef K::Point_2 Point_base;
// 	typedef K::Weighted_point_2  Point;
// 	typedef CGAL::Weighted_alpha_shape_euclidean_traits_2<K> Gt;
// 	typedef CGAL::Regular_triangulation_vertex_base_2<Gt> Rvb;
// 	typedef CGAL::Alpha_shape_vertex_base_2<Gt, Rvb> Vb;
// 	typedef CGAL::Regular_triangulation_face_base_2<Gt> Rf;
// 	typedef CGAL::Alpha_shape_face_base_2<Gt, Rf>  Fb;
// 	typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
// 	typedef CGAL::Regular_triangulation_2<Gt, Tds> Triangulation_2;
// 	typedef CGAL::Alpha_shape_2<Triangulation_2>  Alpha_shape_2;
// 	typedef Alpha_shape_2::Alpha_shape_edges_iterator Alpha_shape_edges_iterator;

	typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
	typedef K::FT                                                FT;
	typedef K::Weighted_point_2                                  Point;
	typedef K::Segment_2                                         Segment;
	typedef CGAL::Alpha_shape_vertex_base_2<K>                   Vb;
	typedef CGAL::Alpha_shape_face_base_2<K>                     Fb;
	typedef CGAL::Triangulation_data_structure_2<Vb, Fb>          Tds;
	typedef CGAL::Delaunay_triangulation_2<K, Tds>                Triangulation_2;
	typedef CGAL::Alpha_shape_2<Triangulation_2>                 Alpha_shape_2;
	typedef Alpha_shape_2::Alpha_shape_edges_iterator            Alpha_shape_edges_iterator;

	std::vector<Point> points;

	int nSample;
	if (indices)
		nSample = indices->size();
	else
		nSample = cloud.size();

	for (int i = 0; i < nSample; i++)
	{
		int id;

		if (indices)
			id = indices->at(i);
		else
			id = i;

		Point p = Point(cloud.points[id].x, cloud.points[id].y);
		FT p_w = FT(10);
		if (has_weight)
			p_w = FT(weights[id]);

		points.push_back(Point(p, p_w));
	}


//	std::vector<Gt::Segment_2> segments;

	Alpha_shape_2 A(points.begin(), points.end());

	/*
	In general, an alpha shape can be disconnected and contain many singular edges or vertices.
	Its regularized version is formed by the set of regular edges and their vertices.
	*/
	//	if (has_weight)
	//	{
	A.set_mode(Alpha_shape_2::REGULARIZED);
	//	}
	// 	else
	// 	{
	// 		A.set_mode(Alpha_shape_2::GENERAL);
	// 	};


	if (alpha < 0)
	{
		Alpha_shape_2::Alpha_iterator opt = A.find_optimal_alpha(1);
		A.set_alpha((*opt)*2.0f);
	}
	else
		A.set_alpha(/*Alpha*/alpha);

	//����edge
	/*int r = 0, s = 0, i = 0, e = 0;
	for (Alpha_shape_2::Finite_edges_iterator it = A.finite_edges_begin();
	it != A.finite_edges_end();
	++it)
	{
	switch (A.classify(*it))
	{
	case Alpha_shape_2::REGULAR:  ++r; break;
	case Alpha_shape_2::SINGULAR: ++s; break;
	case Alpha_shape_2::EXTERIOR: ++e; break;
	case Alpha_shape_2::INTERIOR: ++i; break;
	}
	}
	std::cout << "EIRS " << e << " " << i << " " << r << " " << s << std::endl;*/

	line_segs.empty();
	int nREGULAR = 0;
	int nSINGULAR = 0;
	int nEXTERIOR = 0;
	int nINTERIOR = 0;
	int nALL = 0;
	OGRLinearRing ring;
	for (Alpha_shape_edges_iterator it = A.alpha_shape_edges_begin();
		it != A.alpha_shape_edges_end();
		++it)
	{
		nALL++;
		// Get a vertex from the edge  by face
		/*		Alpha_shape_2::Triangulation::Face& f = *(it->first);
		int i = it->second;
		Vertex vs = f.vertex(f.cw(i));
		Vertex vt = f.vertex(f.ccw(i));*/

		//Gt::Segment_2 s = A.segment(*it);
		Segment s = A.segment(*it);
		Point p = s.source();
		
//		line_segs.push_back(p);

		const Point& p1 = s.point(0);
		const Point& p2 = s.point(1);

		PointT  ptxy;
		switch (A.classify(*it))
		{
		case Alpha_shape_2::REGULAR:
//			line_segs.push_back(p);
			ring.addPoint(p2.x(), p2.y(), 0);
			nREGULAR++;
			break;
		case Alpha_shape_2::SINGULAR:
			nSINGULAR++;
			//ring.addPoint(p2.x(), p2.y(), 0);

//			line_segs.push_back(p);
// 			ipl::geometry::EDGE2D  edge;
// 
// 			edge.ls.sp[0] = p1.x();
// 			edge.ls.sp[1] = p1.y();
// 			edge.ls.ep[0] = p2.x();
// 			edge.ls.ep[1] = p2.y();
// 
// 			line_segs.push_back(edge);

			// 			ptxy.x = p1.x();
			// 			ptxy.y = p1.y();
			// 
			// 			line_segs->points.push_back(ptxy);
			// 
			// 			ptxy.x = p2.x();
			// 			ptxy.y = p2.y();
			// 			line_segs->points.push_back(ptxy);

			break;
		case Alpha_shape_2::EXTERIOR:
			nEXTERIOR++;
			break;
		case Alpha_shape_2::INTERIOR:
			nINTERIOR++;
			break;
		}
	}


	//��������
	for (std::vector<Point>::iterator it = points.begin(); it != points.end(); ++it)
	{
		//std::cout << "*it "<<*it<<std::endl;
		switch (A.classify(*it))
		{
		case Alpha_shape_2::REGULAR:
			ring.addPoint(it->x(), it->y(), 0);
			break;
		case Alpha_shape_2::SINGULAR:

			break;
		case Alpha_shape_2::EXTERIOR:
			break;
		case Alpha_shape_2::INTERIOR:
			break;
		}
	}

	ring.closeRings();//��β���غ��γɱպϻ� 
	line_segs.addRing(&ring);

//	std::cout << segments.size() << " alpha shape edges." << std::endl;
	return 0;
}

template <typename PointT>
int alpha_shape_2d(const pcl::PointCloud<PointT> &cloud, std::vector<int> *indices, bool has_weight,
	std::vector<float> &weights,
	OGRMultiLineString &line_segs,
	float alpha)
{
	// 	typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
	// 	typedef K::FT FT;
	// 	typedef K::Point_2 Point_base;
	// 	typedef K::Weighted_point_2  Point;
	// 	typedef CGAL::Weighted_alpha_shape_euclidean_traits_2<K> Gt;
	// 	typedef CGAL::Regular_triangulation_vertex_base_2<Gt> Rvb;
	// 	typedef CGAL::Alpha_shape_vertex_base_2<Gt, Rvb> Vb;
	// 	typedef CGAL::Regular_triangulation_face_base_2<Gt> Rf;
	// 	typedef CGAL::Alpha_shape_face_base_2<Gt, Rf>  Fb;
	// 	typedef CGAL::Triangulation_data_structure_2<Vb, Fb> Tds;
	// 	typedef CGAL::Regular_triangulation_2<Gt, Tds> Triangulation_2;
	// 	typedef CGAL::Alpha_shape_2<Triangulation_2>  Alpha_shape_2;
	// 	typedef Alpha_shape_2::Alpha_shape_edges_iterator Alpha_shape_edges_iterator;

	typedef CGAL::Exact_predicates_inexact_constructions_kernel  K;
	typedef K::FT                                                FT;
	typedef K::Weighted_point_2                                  Point;
	typedef K::Segment_2                                         Segment;
	typedef CGAL::Alpha_shape_vertex_base_2<K>                   Vb;
	typedef CGAL::Alpha_shape_face_base_2<K>                     Fb;
	typedef CGAL::Triangulation_data_structure_2<Vb, Fb>          Tds;
	typedef CGAL::Delaunay_triangulation_2<K, Tds>                Triangulation_2;
	typedef CGAL::Alpha_shape_2<Triangulation_2>                 Alpha_shape_2;
	typedef Alpha_shape_2::Alpha_shape_edges_iterator            Alpha_shape_edges_iterator;

	std::vector<Point> points;

	int nSample;
	if (indices)
		nSample = indices->size();
	else
		nSample = cloud.size();

	for (int i = 0; i < nSample; i++)
	{
		int id;

		if (indices)
			id = indices->at(i);
		else
			id = i;

		Point p = Point(cloud.points[id].x, cloud.points[id].y);
		FT p_w = FT(10);
		if (has_weight)
			p_w = FT(weights[id]);

		points.push_back(Point(p, p_w));
	}


	//	std::vector<Gt::Segment_2> segments;

	Alpha_shape_2 A(points.begin(), points.end());

	/*
	In general, an alpha shape can be disconnected and contain many singular edges or vertices.
	Its regularized version is formed by the set of regular edges and their vertices.
	*/
	//	if (has_weight)
	//	{
	A.set_mode(Alpha_shape_2::REGULARIZED);
	//	}
	// 	else
	// 	{
	// 		A.set_mode(Alpha_shape_2::GENERAL);
	// 	};


	if (alpha < 0)
	{
		Alpha_shape_2::Alpha_iterator opt = A.find_optimal_alpha(1);
		A.set_alpha((*opt)*2.0f);
	}
	else
		A.set_alpha(/*Alpha*/alpha);

	//����edge
	/*int r = 0, s = 0, i = 0, e = 0;
	for (Alpha_shape_2::Finite_edges_iterator it = A.finite_edges_begin();
	it != A.finite_edges_end();
	++it)
	{
	switch (A.classify(*it))
	{
	case Alpha_shape_2::REGULAR:  ++r; break;
	case Alpha_shape_2::SINGULAR: ++s; break;
	case Alpha_shape_2::EXTERIOR: ++e; break;
	case Alpha_shape_2::INTERIOR: ++i; break;
	}
	}
	std::cout << "EIRS " << e << " " << i << " " << r << " " << s << std::endl;*/

	line_segs.empty();
	OGRLineString line;
	for (Alpha_shape_edges_iterator it = A.alpha_shape_edges_begin();
		it != A.alpha_shape_edges_end();
		++it)
	{
		// Get a vertex from the edge  by face
		/*		Alpha_shape_2::Triangulation::Face& f = *(it->first);
		int i = it->second;
		Vertex vs = f.vertex(f.cw(i));
		Vertex vt = f.vertex(f.ccw(i));*/

		//Gt::Segment_2 s = A.segment(*it);
		Segment s = A.segment(*it);
		Point p = s.source();

		//		line_segs.push_back(p);

		const Point& p1 = s.point(0);
		const Point& p2 = s.point(1);

		PointT  ptxy;
		switch (A.classify(*it))
		{
		case Alpha_shape_2::REGULAR:
			//			line_segs.push_back(p);
			line.empty();
			line.addPoint(p1.x(), p1.y());
			line.addPoint(p2.x(), p2.y());

			line_segs.addGeometry(&line);

			break;
		case Alpha_shape_2::SINGULAR:

			line.empty();
			line.addPoint(p1.x(), p1.y());
			line.addPoint(p2.x(), p2.y());

			line_segs.addGeometry(&line);

			//ring.addPoint(p2.x(), p2.y(), 0);

			//			line_segs.push_back(p);
			// 			ipl::geometry::EDGE2D  edge;
			// 
			// 			edge.ls.sp[0] = p1.x();
			// 			edge.ls.sp[1] = p1.y();
			// 			edge.ls.ep[0] = p2.x();
			// 			edge.ls.ep[1] = p2.y();
			// 
			// 			line_segs.push_back(edge);

			// 			ptxy.x = p1.x();
			// 			ptxy.y = p1.y();
			// 
			// 			line_segs->points.push_back(ptxy);
			// 
			// 			ptxy.x = p2.x();
			// 			ptxy.y = p2.y();
			// 			line_segs->points.push_back(ptxy);

			break;
		case Alpha_shape_2::EXTERIOR:
			break;
		case Alpha_shape_2::INTERIOR:
			break;
		}
	}
//	ring.closeRings();//��β���غ��γɱպϻ� 
//	line_segs.addRing(&ring);

	//��������
	for (std::vector<Point>::iterator it = points.begin(); it != points.end(); ++it)
	{
		//std::cout << "*it "<<*it<<std::endl;
		switch (A.classify(*it))
		{
		case Alpha_shape_2::REGULAR:
			;
		case Alpha_shape_2::SINGULAR:

			break;
		case Alpha_shape_2::EXTERIOR:
			break;
		case Alpha_shape_2::INTERIOR:
			break;
		}
	}


	//	std::cout << segments.size() << " alpha shape edges." << std::endl;
	return 0;
}
}

