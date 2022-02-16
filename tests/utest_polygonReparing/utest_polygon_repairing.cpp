// utest_polygonReparing.cpp : Defines the entry point for the console application.
//


#include<list>
#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Cartesian.h>
#include <CGAL/Exact_rational.h>

#include <CGAL/Arr_accessor.h>
// #include "read_objects.h"
// #include "polygon_repairing.h"

#include "geometry/polygon/SelfintersectionReparation.h"

//#include "model_reconstruction/floormap_reconstruction.h"

//typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef CGAL::Cartesian<CGAL::Exact_rational> Kernel;
typedef CGAL::Arr_segment_traits_2<Kernel> Traits;
typedef Traits::Point_2 Point;
typedef Traits::X_monotone_curve_2 Segment;

typedef CGAL::Arr_extended_dcel<Traits, int, int, int>	Dcel;
typedef CGAL::Arrangement_with_history_2<Traits, Dcel>    Arrangement;
typedef Arrangement::Halfedge                   DHalfedge;

typedef CGAL::Arr_accessor<Arrangement>			Arr_accessor;

int exportLineArrangement(Arrangement *arr, const std::string &filename, const double floor_hei);



int main(int argc, char* argv[])
{
	//std::list<Point> points;
	//const char* filename = (argc > 1) ? argv[1] : "polygon . dat";

	//read_objects<Point>(filename, std::back_inserter(points));
	//CGAL_assertion(points.size() >= 3);
	std::list<Segment> segments;
	//std::list<Point>::const_iterator it = points.begin();
	// 	const Point& first_point = *it++;
	// 	const Point* prev_point = &first_point;
	// 
	// 	while (it != points.end()) {
	// 		const Point& point = *it++;
	// 		segments.push_back(Segment(*prev_point, point));
	// 		prev_point = &point;
	// 	}segments.push_back(Segment(*prev_point, first_point));

/*	segments.push_back(Segment(Point(3, 1), Point(7, 1)));
	segments.push_back(Segment(Point(7, 1), Point(4, 4)));
	//segments.push_back(Segment(Point(5, 3), Point(4, 4)));
	segments.push_back(Segment(Point(4, 4), Point(4, 1.5)));
	segments.push_back(Segment(Point(4, 1.5), Point(7, 5)));
	//segments.push_back(Segment(Point(5, 3), Point(7, 5)));
	segments.push_back(Segment(Point(7, 5), Point(3, 5)));
	segments.push_back(Segment(Point(3, 5), Point(1, 3)));
	segments.push_back(Segment(Point(1, 3), Point(3, 1)));*/

/*	segments.push_back(Segment(Point(3, 1), Point(7, 1)));
	segments.push_back(Segment(Point(7, 1), Point(5, 3)));
	segments.push_back(Segment(Point(5, 3), Point(7, 5)));
	segments.push_back(Segment(Point(7, 5), Point(3, 5)));
	segments.push_back(Segment(Point(3, 5), Point(1, 3)));
	segments.push_back(Segment(Point(1, 3), Point(3, 1)));

	segments.push_back(Segment(Point(5, 3), Point(4, 4)));
	segments.push_back(Segment(Point(4, 4), Point(4, 1.5)));
	segments.push_back(Segment(Point(4, 1.5), Point(5, 3)));*/

	segments.push_back(Segment(Point(-1, 1), Point(-1, -1)));
	segments.push_back(Segment(Point(-1, -1), Point(1, 1)));
	segments.push_back(Segment(Point(1, 1), Point(1, -1)));
	segments.push_back(Segment(Point(1, -1), Point(-1, 1)));

	Arrangement arr;

	insert(arr, segments.begin(), segments.end());

	Arrangement::Vertex_handle vit;
	std::cout << arr.number_of_vertices() << " vertices:" << std::endl;
	int idx;
	for (vit = arr.vertices_begin(), idx = 0; vit != arr.vertices_end(); ++vit, ++idx)
	{
		vit->set_data(idx);
		std::cout << "vertice: " << idx << " (" << vit->point().x().to_double()
			<< ", " << vit->point().y().to_double() << ")" << std::endl;
	}

	Arrangement::Halfedge_iterator hit;
	std::cout << arr.number_of_halfedges() << " halfedges:" << std::endl;
	for (hit = arr.halfedges_begin(), idx = 0; hit != arr.halfedges_end(); ++hit, ++idx)
	{
		hit->set_data(idx);
		std::cout << "halfedge: " << idx << " from v" << hit->source()->data() << " to v" << hit->target()->data()
			<< std::endl;
	}

	for (hit = arr.halfedges_begin(), idx = 0; hit != arr.halfedges_end(); ++hit, ++idx)
	{
		Arrangement::Halfedge_around_vertex_circulator  inhe;
		inhe = hit->target()->incident_halfedges();

		int eid = inhe->data();

		std::cout << "halfedge: " << hit->data() << " to v " << hit->target()->data() 
			<< " incident_halfedges " << eid << std::endl;
	}
	Arrangement::Face_iterator fit;
	std::cout << arr.number_of_faces() << " faces:" << std::endl;
	for (fit = arr.faces_begin(), idx = 0; fit != arr.faces_end(); ++fit, ++idx)
	{
		fit->set_data(idx);
	}

	for (fit = arr.faces_begin(); fit != arr.faces_end(); ++fit)
	{//提取面的顶点列表

		std::cout << "-------------------------------" << std::endl;
		std::cout << "face: " << fit->data() << std::endl;
		if (fit->is_unbounded())
		{
			typename Arrangement::Hole_iterator hit;
			for (hit = fit->holes_begin(); hit != fit->holes_end(); ++hit)
			{
				typename Arrangement::Ccb_halfedge_circulator cch = *hit;
				do {

					int fid = cch->face()->data();
					int innerfid = cch->twin()->face()->data();

					int eid = cch->data();
					int teid = cch->twin()->data();
					std::cout << "halfedge: " << eid << " border of face (" << fid << ", " << innerfid << ")" << std::endl;
				} while (++cch != *hit);
			}
			continue;
		}

		// 		if (idx != 2)
		// 			continue;

				//int idx = f->data();

				//outer border
		Arrangement::Ccb_halfedge_circulator  circ = fit->outer_ccb();
		Arrangement::Ccb_halfedge_circulator  curr = circ;
		do
		{
			//Arrangement_2::Halfedge_const_handle he = curr->handle();
			Arrangement::Halfedge_handle he = curr;
			int eid = he->data();

			int fid = he->face()->data();
			int innerfid = he->twin()->face()->data();

			std::cout << "halfedge: " << eid << " border of face (" << fid << ", " << innerfid << ")" << std::endl;
			curr = curr->next();
		} while (curr != circ);
	}


	//test split 
	Arrangement::Halfedge_handle h6, h8, h13, h15;
	//DHalfedge *h6, *h8, *h13, *h15;
	for (hit = arr.halfedges_begin(); hit != arr.halfedges_end(); ++hit)
	{
		int eid = hit->data();
		int previd = hit->prev()->data();
		int nextid = hit->next()->data();

		switch (eid)
		{
		case 6:
			h6 = hit;
			break;
		case 8:
			h8 = hit;
			break;
		case 13:
			h13 = hit;
			break;
		case 15:
			h15 = hit;
			break;
		}
	}
	
	Arr_accessor  accessor(arr);

	
/*	h8->set_next(&(*h6));
	h6->set_prev(h8);
	h13->set_next(h15);
	h15->set_prev(h13);*/

	//ipl::FloorMapRec  rec;
//	exportLineArrangement(&arr, "D:/temp/test.off", 5.0);


	std::list<std::list<Segment> > polygons;
	ipl::polygon_repairing(segments.begin(), segments.end(), polygons, Traits());

	std::list<std::list <Segment> >::const_iterator pit;
	for (pit = polygons.begin(); pit != polygons.end(); ++pit) {
		std::copy(pit->begin(), pit->end(),
			std::ostream_iterator<Segment>(std::cout, "\n"));
		std::cout << std::endl;
	}

	return 0;
}

#include <CGAL/Polyhedron_incremental_builder_3.h>
#include<CGAL/IO/Polyhedron_iostream.h>

typedef CGAL::Polyhedron_3<Kernel>			Polyhedron;
typedef Polyhedron::HalfedgeDS				HalfedgeDS;
//typedef Polyhedron::Point_3					Point_3;

// A modifier creating a triangle with the incremental builder.
template<class HDS>
class polyhedron_builder : public CGAL::Modifier_base<HDS>
{
public:
	std::vector<Polyhedron::Point_3> vertex_;
	std::vector<std::vector<int> >    tris_;
	int nhole_;

	polyhedron_builder(std::vector<Polyhedron::Point_3> vertex, std::vector<std::vector<int> > tris, int nhole = 0) :
		vertex_(vertex), tris_(tris), nhole_(nhole) {}

	void operator()(HDS& hds)
	{
		typedef typename HDS::Vertex   Vertex;
		typedef typename Vertex::Point Point;

		// create a cgal incremental builder
		CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
		B.begin_surface(vertex_.size(), tris_.size(), nhole_);

		// add the polyhedron vertices
		for (int i = 0; i < (int)vertex_.size(); i++)
		{
			B.add_vertex(vertex_[i]);
		}

		// add the polygon mesh
		for (int i = 0; i < (int)tris_.size(); i++)
		{
			B.begin_facet();
			std::vector<std::vector<int> >::iterator iter = tris_.begin() + i;

			for (int j = 0; j < iter->size(); j++)
			{
				B.add_vertex_to_facet(iter->at(j));
			}
			B.end_facet();
		}

		// finish up the surface
		B.end_surface();
	}
};

int exportLineArrangement(Arrangement *arr, const std::string &filename, const double floor_hei)
{
	//构造Polyhedron，输出边界
	std::vector<Polyhedron::Point_3>  points;
	std::vector<std::vector<int>> flist;	//facet vertex list  counter-clockwise
	std::vector<std::vector<int>> hlist;	//holes

	int nvertices = arr->number_of_vertices();
	int nvertices_inf = arr->number_of_vertices_at_infinity();
	nvertices -= nvertices_inf;

	points.resize(nvertices);

	int idx, ncount = 0;
	Arrangement::Vertex_const_handle vit;
	std::cout << arr->number_of_vertices() << " vertices:" << std::endl;
	for (vit = arr->vertices_begin(); vit != arr->vertices_end(); ++vit)
	{//提取顶点
		idx = vit->data();
				
		//std::cout << idx << " - degree " << vit->degree() << std::endl;
		double x, y, z;
		x = vit->point().x().to_double();
		y = vit->point().y().to_double();
		z = floor_hei;
		Polyhedron::Point_3 p(x, y, z);
		points[idx] = p;
	}

// 	assert(ncount == nvertices);
// 	assert(nvertices_inf == 0);

	int nfaces = arr->number_of_faces();
	int nfaces_unbounded = arr->number_of_unbounded_faces();
	nfaces -= nfaces_unbounded;

	
	ncount = 0;
	//flist.resize(nfaces);
	Arrangement::Face_handle f;
	
	idx = 0;
	for (f = arr->faces_begin(); f != arr->faces_end(); ++f, ++idx)
	{//提取面的顶点列表
	 //std::cout << "traverse face: " << f->data() << std::endl;
		
		if (f->is_unbounded())
		{
			continue;
		}

		
			

		//int idx = f->data();

		std::vector<int>  vlist;//当前face的顶点列表

		if (idx == 2)
		{
			vlist.push_back(2);
			vlist.push_back(0);
			vlist.push_back(1);
			vlist.push_back(6);
			vlist.push_back(5);
			vlist.push_back(7);
			flist.push_back(vlist);
			continue;
		}
		
		//outer border
		Arrangement::Ccb_halfedge_circulator  circ = f->outer_ccb();
		Arrangement::Ccb_halfedge_circulator  curr = circ;

		int vid = curr->source()->data();
		//vlist.push_back(vid);

		do
		{
			//Arrangement_2::Halfedge_const_handle he = curr->handle();
			Arrangement::Halfedge_handle he = curr;

			vid = he->target()->data();
			vlist.push_back(vid);

			int eid = he->data();

			curr = curr->next();

		} while (curr != circ);

		//flist[idx] = vlist;
		
		flist.push_back(vlist);

		//holes (inner border)   need to test: how to create a suface mesh with holes
		if (false) {
			Arrangement::Hole_iterator hi;
			for (hi = f->holes_begin(); hi != f->holes_end(); ++hi)
			{
				Arrangement::Ccb_halfedge_circulator circ = *hi;
				Arrangement::Ccb_halfedge_circulator curr = circ;

				std::vector<int> hole_vlist;
				int vid = curr->source()->data();
				hole_vlist.push_back(vid);
				do
				{
					Arrangement::Halfedge_handle he = curr;
					//find adjacent facets
					vid = he->target()->data();
					hole_vlist.push_back(vid);

					curr = curr->next();
				} while (curr != circ);
				hlist.push_back(hole_vlist);
			}
		}
	}

//	assert(ncount == nfaces);

	flist.insert(flist.end(), hlist.begin(), hlist.end());


	// build a polyhedron from the loaded arrays
	Polyhedron P;
	polyhedron_builder<HalfedgeDS> builder(points, flist, hlist.size());
	P.delegate(builder);

	// write the polyhedron out as a .OFF file
	std::ofstream os(filename);
	os << P;
	os.close();

	return (0);
}

