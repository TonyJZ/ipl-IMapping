#pragma once

#include <utility>

#include <CGAL/basic.h>
#include <CGAL/enum.h>

#include <CGAL/Arrangement_with_history_2.h>
#include <CGAL/Arr_extended_dcel.h>

namespace ipl
{
template <typename Arrangement> class Winding_number {
private:
	Arrangement& _arr;
	typename Arrangement::Traits_2::Compare_endpoints_xy_2 _cmp_endpoints;

	// The Boolean flag indicates whether the face has been discovered already
	// during the traversal. The integral field stores the winding number.
	typedef std::pair<bool, int>                        Data;
	//typedef std::pair<int, Data>                              IndexedData;
	typedef boost::unordered_map<int, Data>				WNMap;    //winding number map

public:
	WNMap			_windingmap;
public:
	Winding_number(Arrangement& arr) : _arr(arr)
	{
		// Initialize the winding numbers of all faces.

// 		int nface = _arr.number_of_faces();
// 
// 		Arrangement::Halfedge_iterator hi;
// 		int eid = 0;
// 		for (hi = _arr.halfedges_begin(); hi != _arr.halfedges_end(); ++hi)
// 		{
// 			std::cout << '(' << hi->source()->point().x().to_double() << ',' << hi->source()->point().y().to_double() << ')'
// 				<< '(' << hi->target()->point().x().to_double() << ',' << hi->target()->point().y().to_double() << ')' << std::endl;
// 
// 			hi->set_data(eid);
// 
// 			int t = hi->data();
// 			eid++;
// 		}

		int id = 0;
		typename Arrangement::Face_iterator fi;
		for (fi = _arr.faces_begin(); fi != _arr.faces_end(); ++fi)
		{
			int fidx = fi->data();
			//fi->set_data(fidx);

			_windingmap.insert(std::make_pair(fidx, Data(false, 0)));
			id++;
		}

		/////////////////////////////////for test//////////////////////////////////////
		
		_cmp_endpoints = _arr.traits()->compare_endpoints_xy_2_object(); /* \label{lst:winding_number:a} */
		propagate_face(_arr.unbounded_face(), 0);   // compute the winding numbers
	}

private:
	// Count the net change to the winding number when crossing a halfedge.
	int count(typename Arrangement::Halfedge_handle he)
	{
		int num_cur = _arr.number_of_curves();

		int onum = _arr.number_of_originating_curves(he);

		bool l2r = he->direction() == CGAL::ARR_LEFT_TO_RIGHT;
		std::cout << he->data() << ": " << '(' << he->target()->point().x().to_double() << ','
			<< he->target()->point().y().to_double() << ')' << std::endl;

		typename Arrangement::Originating_curve_iterator ocit;
		int num = 0;
		for (ocit = _arr.originating_curves_begin(he);
			ocit != _arr.originating_curves_end(he); ++ocit)
		{
			// 			double xs = ocit->source()->point().x().to_double();
			// 			double ys = ocit->source()->point().y().to_double();
			// 			double xt = ocit->target()->point().x().to_double();
			// 			double yt = ocit->target()->point().y().to_double();

// 			std::cout << "Curve [" << *ocit << "] induces "
// 				<< _arr.number_of_induced_edges(ocit) << " edges." << std::endl;

			(l2r == (_cmp_endpoints(*ocit) == CGAL::SMALLER)) ? ++num : --num; /* \label{lst:winding_number:b} */
		}

		return num;
	}

	// Traverse all faces neighboring the given face and compute the
	// winding numbers of all faces while traversing the arrangement.
	void propagate_face(typename Arrangement::Face_handle fh, int num)
	{
		int fidx = fh->data();
		WNMap::iterator it_wm;
		it_wm = _windingmap.find(fidx);


		if (it_wm->second.first)
			return;

		it_wm->second = Data(true, num);
		//fh->set_data(Data(true, num));

		// Traverse the inner boundary (holes).
		typename Arrangement::Hole_iterator hit;
		for (hit = fh->holes_begin(); hit != fh->holes_end(); ++hit)
		{
			typename Arrangement::Ccb_halfedge_circulator cch = *hit;
			do {
				int fid = cch->face()->data();
				int innerfid = cch->twin()->face()->data();

				int eid = cch->data();
				int teid = cch->twin()->data();

				typename Arrangement::Face_handle inner_face = cch->twin()->face();
				if (inner_face == cch->face())
					continue;        // discard antenas

				propagate_face(inner_face, num + count(cch->twin()));
			} while (++cch != *hit);
		}

		// Traverse the outer boundary.
		if (fh->is_unbounded()) return;

		typename Arrangement::Ccb_halfedge_circulator cco = fh->outer_ccb();
		do {
			typename Arrangement::Face_handle outer_face = cco->twin()->face();
			propagate_face(outer_face, num + count(cco->twin()));
		} while (++cco != fh->outer_ccb());
	}
};


template <typename Traits, typename Input_iterator, typename Container>
void polygon_repairing(Input_iterator begin, Input_iterator end,
	Container& res, const Traits& traits)
{
	// Each face is extended with a pair of a Boolean flag and an integral
	// field: The former indicates whether the face has been discovered
	// already during the traversal. The latter stores the winding number.
	typedef std::pair<bool, int>                              Data;
	//typedef CGAL::Arr_face_extended_dcel<Traits, int>         Dcel;
	typedef CGAL::Arr_extended_dcel<Traits, int, int, int>	Dcel;
	typedef CGAL::Arrangement_with_history_2<Traits, Dcel>    Arrangement;
	typedef boost::unordered_map<int, Data>				WNMap;    //winding number map

	Arrangement arr(&traits);
	insert(arr, begin, end);

	std::cout << "The arrangement contains "
		<< arr.number_of_faces() << " faces:" << std::endl;

	// Print out the curves and the number of edges each one induces.
	Arrangement::Curve_iterator              cit;
	std::cout << "The arrangement contains "
		<< arr.number_of_curves() << " curves:" << std::endl;
	for (cit = arr.curves_begin(); cit != arr.curves_end(); ++cit)
	{
		// 		std::cout << "Curve [" << *cit << "] "
		// 			<< cit->data() << std::endl;
		// 		
		std::cout << "Curve [" << *cit << "] induces "
			<< arr.number_of_induced_edges(cit) << " edges." << std::endl;
	}

	// Print the arrangement edges along with the list of curves that
	// induce each edge.
	Arrangement::Edge_iterator               eit;
	Arrangement::Originating_curve_iterator  ocit;

	std::cout << "The arrangement is comprised of "
		<< arr.number_of_edges() << " edges:" << std::endl;
	for (eit = arr.edges_begin(); eit != arr.edges_end(); ++eit)
	{
		std::cout << "[" << eit->curve() << "]. Originating curves: ";
		for (ocit = arr.originating_curves_begin(eit);
			ocit != arr.originating_curves_end(eit); ++ocit)
			std::cout << " [" << *ocit << "]" << std::flush;
		std::cout << std::endl;
	}

	Winding_number<Arrangement> winding_number(arr);

	typename Arrangement::Face_iterator fi;
	for (fi = arr.faces_begin(); fi != arr.faces_end(); ++fi)
	{
		int fid = fi->data();
		WNMap::iterator it_wm;
		it_wm = winding_number._windingmap.find(fid);

		if ((it_wm->second.second % 2) == 0) continue;

		CGAL_assertion(!fi->is_unbounded());
		typename Container::value_type polygon;


		typename Arrangement::Ccb_halfedge_circulator cco = fi->outer_ccb();
		do {
			polygon.push_back(cco->curve());
		} while (++cco != fi->outer_ccb());


		res.push_back(polygon);
	}
}

}
