// utest_LineArrangement.cpp : Defines the entry point for the console application.
//

#include <CGAL/Cartesian.h>
#include <CGAL/Exact_rational.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_on_surface_with_history_2.h>
#include <CGAL/Arrangement_with_history_2.h>
#include <CGAL/Arr_simple_point_location.h>
#include <CGAL/Arr_extended_dcel.h>
#include "point_location_utils.h"

typedef CGAL::Cartesian<CGAL::Exact_rational>             Kernel;
typedef CGAL::Arr_segment_traits_2<Kernel>                Traits_2;
typedef Traits_2::Point_2                                 Point_2;
typedef Traits_2::Curve_2                                 Segment_2;
typedef Traits_2::Line_2								  Line_2;
typedef CGAL::Arr_extended_dcel<Traits_2, int, int, int>	Dcel;    //vertex id, edge id, face id
typedef CGAL::Arrangement_with_history_2<Traits_2, Dcel>        Arr_with_hist_2;
typedef Arr_with_hist_2::Curve_handle                     Curve_handle;
typedef CGAL::Arr_simple_point_location<Arr_with_hist_2>  Point_location;
typedef Arr_with_hist_2::Halfedge_handle					Halfedge_handle;
typedef Arr_with_hist_2::Vertex_handle						Vertex_handle;

typedef CGAL::Arr_walk_along_line_point_location<Arr_with_hist_2> Walk_pl;

int testSelfIntersection()
{
	Arr_with_hist_2   arr;
	// Insert s1, s2 and s3 incrementally:

	
	std::vector<Segment_2> segments;

	segments.push_back(Segment_2(Point_2(-2, -2), Point_2(2, -2)));
	segments.push_back(Segment_2(Point_2(2, -2), Point_2(2, 2)));
	segments.push_back(Segment_2(Point_2(2, 2), Point_2(-2, 2)));
	segments.push_back(Segment_2(Point_2(-2, 2), Point_2(-2, -2)));

	segments.push_back(Segment_2(Point_2(-1, -1), Point_2(1, -1)));
	segments.push_back(Segment_2(Point_2(1, -1), Point_2(1, 1)));
	segments.push_back(Segment_2(Point_2(1, 1), Point_2(-1, 1)));
	segments.push_back(Segment_2(Point_2(-1, 1), Point_2(-1, -1)));

	/*	segments.push_back(Segment_2(Point_2(-1, 0), Point_2(1, 0)));
	segments.push_back(Segment_2(Point_2(0, -1), Point_2(0, 1)));

	segments.push_back(Segment_2(Point_2(-1.5, 1.5), Point_2(-1.5, 1)));
	segments.push_back(Segment_2(Point_2(-1.5, 1), Point_2(-1.1, 1)));
	segments.push_back(Segment_2(Point_2(-1.1, 1), Point_2(-1.1, 1.5)));
	segments.push_back(Segment_2(Point_2(-1.1, 1.5), Point_2(-1.5, 1.5)));*/

	insert(arr, segments[0]);
	insert(arr, segments[1]);
	insert(arr, segments[2]);
	insert(arr, segments[3]);

	insert(arr, segments[4]);
	insert(arr, segments[5]);
	insert(arr, segments[6]);
	insert(arr, segments[7]);

	// 	insert(arr, segments[3]);
	// 	insert(arr, segments[4]);
	// 	insert(arr, segments[5]);
	// 	insert(arr, segments[6]);




	//	insert(arr, segments.begin(), segments.end());


	std::cout << "vertice, halfedge, face: ";
	std::cout << "(" << arr.number_of_vertices() << ", " << arr.number_of_halfedges()
		<< ", " << arr.number_of_faces() << ")" << std::endl;

	Arr_with_hist_2::Face_iterator fit;
	int idx = 0;
	for (fit = arr.faces_begin(); fit != arr.faces_end(); ++fit, idx++)
	{
		if (fit->is_unbounded())
			continue;

		Arr_with_hist_2::Ccb_halfedge_circulator  circ = fit->outer_ccb();
		Arr_with_hist_2::Ccb_halfedge_circulator  curr = circ;

		std::cout << "-----------------------------" << std::endl;
		std::cout << "face: " << idx << std::endl;
		do
		{
			//Arrangement_2::Halfedge_const_handle he = curr->handle();
			Arr_with_hist_2::Halfedge_handle he = curr;

			std::cout << "(" << he->target()->point().x().to_double() << ", "
				<< he->target()->point().y().to_double() << ")" << std::endl;

			curr = curr->next();

		} while (curr != circ);

		std::cout << "holes: "
			<< fit->number_of_holes() << std::endl;

		int hidx = 0;
		//holes (inner border)   need to test: how to create a suface mesh with holes
		Arr_with_hist_2::Hole_iterator hi;
		typedef std::vector<Arr_with_hist_2::Halfedge_handle> hole_edge_list;
		typedef std::vector<hole_edge_list> holes_list;

		holes_list  list;
		if (fit->number_of_holes() > 0)
			list.resize(fit->number_of_holes());

		for (hi = fit->holes_begin(), hidx = 0; hi != fit->holes_end(); ++hi, ++hidx)
		{
			Arr_with_hist_2::Ccb_halfedge_circulator circ = *hi;
			Arr_with_hist_2::Ccb_halfedge_circulator curr = circ;

			std::cout << "hole: " << hidx << std::endl;

			do
			{
				Arr_with_hist_2::Halfedge_handle he = curr;
				list[hidx].push_back(he);

				std::cout << "(" << he->target()->point().x().to_double() << ", "
					<< he->target()->point().y().to_double() << ")" << std::endl;

				curr = curr->next();
			} while (curr != circ);

		}

		for (hidx = 0; hidx<list.size(); ++hidx)
		{
			for (int i = 0; i < list[hidx].size(); i++)
			{
				Arr_with_hist_2::Halfedge_handle he = list[hidx].at(i);

				std::cout << "(" << he->target()->point().x().to_double() << ", "
					<< he->target()->point().y().to_double() << ")" << std::endl;

				arr.remove_edge(he);   //É¾³ı±ß
			}
		}

		std::cout << std::endl;
	}


	std::cout << "------------------------------" << std::endl;
	std::cout << "vertice, halfedge, face: ";
	std::cout << "(" << arr.number_of_vertices() << ", " << arr.number_of_halfedges()
		<< ", " << arr.number_of_faces() << ")" << std::endl;

	idx = 0;
	for (fit = arr.faces_begin(); fit != arr.faces_end(); ++fit, idx++)
	{
		if (fit->is_unbounded())
			continue;

		Arr_with_hist_2::Ccb_halfedge_circulator  circ = fit->outer_ccb();
		Arr_with_hist_2::Ccb_halfedge_circulator  curr = circ;

		std::cout << "-----------------------------" << std::endl;
		std::cout << "face: " << idx << std::endl;
		do
		{
			//Arrangement_2::Halfedge_const_handle he = curr->handle();
			Arr_with_hist_2::Halfedge_handle he = curr;

			std::cout << "(" << he->target()->point().x().to_double() << ", "
				<< he->target()->point().y().to_double() << ")" << std::endl;

			curr = curr->next();

		} while (curr != circ);

		std::cout << "holes: "
			<< fit->number_of_holes() << std::endl;
	}

	return 0;
}

int testUnbounded()
{
	Arr_with_hist_2   arr;
	Walk_pl pl(arr);
	// Insert s1, s2 and s3 incrementally:
	std::vector<Segment_2> lines;

	lines.push_back(Segment_2(Point_2(-2, 1), Point_2(2, 1)));
	lines.push_back(Segment_2(Point_2(-2, -1), Point_2(2, -1)));
	lines.push_back(Segment_2(Point_2(-1, 2), Point_2(-1, -2)));
	lines.push_back(Segment_2(Point_2(1, 2), Point_2(1, -2)));

// 	lines.push_back(Segment_2(Point_2(-1, -1), Point_2(1, -1)));
// 	lines.push_back(Segment_2(Point_2(1, -1), Point_2(1, 1)));
// 	lines.push_back(Segment_2(Point_2(1, 1), Point_2(-1, 1)));
// 	lines.push_back(Segment_2(Point_2(-1, 1), Point_2(-1, -1)));

	/*	segments.push_back(Segment_2(Point_2(-1, 0), Point_2(1, 0)));
	segments.push_back(Segment_2(Point_2(0, -1), Point_2(0, 1)));

	segments.push_back(Segment_2(Point_2(-1.5, 1.5), Point_2(-1.5, 1)));
	segments.push_back(Segment_2(Point_2(-1.5, 1), Point_2(-1.1, 1)));
	segments.push_back(Segment_2(Point_2(-1.1, 1), Point_2(-1.1, 1.5)));
	segments.push_back(Segment_2(Point_2(-1.1, 1.5), Point_2(-1.5, 1.5)));*/

	insert(arr, lines[0]);
	insert(arr, lines[1]);
	insert(arr, lines[2]);
	insert(arr, lines[3]);

// 	insert_non_intersecting_curve(arr, lines[0], pl);
// 	insert_non_intersecting_curve(arr, lines[1], pl);
// 	insert_non_intersecting_curve(arr, lines[2], pl);
// 	insert_non_intersecting_curve(arr, lines[3], pl);

// 	insert(arr, lines[4]);
// 	insert(arr, lines[5]);
// 	insert(arr, lines[6]);
// 	insert(arr, lines[7]);

	std::cout << "vertice, halfedge, face: ";
	std::cout << "(" << arr.number_of_vertices() << ", " << arr.number_of_halfedges()
		<< ", " << arr.number_of_faces() << ")" << std::endl;

	
	int idx = 0;

	std::cout << "-----------------------------" << std::endl;
	std::cout << "Vertices:" << std::endl;
	Arr_with_hist_2::Vertex_iterator vit;
	for (vit = arr.vertices_begin(), idx=0; vit != arr.vertices_end(); ++vit)
	{
		std::cout << idx << ":" << vit->point().x().to_double() << ", " << vit->point().y().to_double() << std::endl;
		vit->set_data(idx);
		idx++;
	}

	std::cout << "-----------------------------" << std::endl;
	std::cout << "Edges:" << std::endl;
	Arr_with_hist_2::Halfedge_iterator eit;
	for (eit = arr.halfedges_begin(), idx=0; eit != arr.halfedges_end(); ++eit)
	{
		std::cout << idx;
		std::cout << "(" << eit->source()->point().x().to_double() << ", " << eit->source()->point().y().to_double() << "), "
			<< "(" << eit->target()->point().x().to_double() << ", " << eit->target()->point().y().to_double() << ")" << std::endl;
		
		eit->set_data(idx);
		idx++;
	}


	idx = 0;
	Arr_with_hist_2::Face_iterator fit;
	for (fit = arr.faces_begin(); fit != arr.faces_end(); ++fit, idx++)
	{
		if (fit->is_unbounded())
		{
			Arr_with_hist_2::Hole_iterator hit;
			int ih = 0;
			for (hit = fit->holes_begin(); hit != fit->holes_end(); ++hit, ih++)
			{
				std::cout << "this is hole " << ih << std::endl;
				typename Arr_with_hist_2::Ccb_halfedge_circulator cch = *hit;
				do {
					
					typename Arr_with_hist_2::Face_handle inner_face = cch->twin()->face();
					if (inner_face == cch->face())
					{
						std::cout << "edge: " << "(" << cch->data() << "/" << cch->twin()->data() << ")"
							<< "is an antena" << std::endl;
						continue;        // discard antenas
					}

					std::cout << cch->data() << " :";
					std::cout << "(" << cch->target()->point().x().to_double() << ", "
						<< cch->target()->point().y().to_double() << ")" << std::endl;


				} while (++cch != *hit);
			}
			continue;
		}
			

		Arr_with_hist_2::Ccb_halfedge_circulator  circ = fit->outer_ccb();
		Arr_with_hist_2::Ccb_halfedge_circulator  curr = circ;

		std::cout << "-----------------------------" << std::endl;
		std::cout << "face: " << idx << std::endl;
		do
		{
			//Arrangement_2::Halfedge_const_handle he = curr->handle();
			Arr_with_hist_2::Halfedge_handle he = curr;

			std::cout << curr->data() << " :";
			std::cout << "(" << he->target()->point().x().to_double() << ", "
				<< he->target()->point().y().to_double() << ")" << std::endl;

			curr = curr->next();

		} while (curr != circ);

		std::cout << "holes: "
			<< fit->number_of_holes() << std::endl;
	}

	

	return 0;
}

int main(int argc, char * argv[])
{
	testUnbounded();
	testSelfIntersection();
	return 0;
}

