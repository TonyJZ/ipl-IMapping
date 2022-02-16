#pragma once

#include <CGAL/Polyhedron_incremental_builder_3.h>
#include<CGAL/IO/Polyhedron_iostream.h>


namespace ipl
{
	//typedef CGAL::Simple_cartesian<double>		Kernel;
// 	typedef CGAL::Cartesian<CGAL::Exact_rational>	Kernel;
// 	typedef CGAL::Polyhedron_3<Kernel>			Polyhedron;
// 	typedef Polyhedron::HalfedgeDS				HalfedgeDS;
	//typedef Polyhedron::Point_3					Point_3;

	// A modifier creating a triangle with the incremental builder.
	template<class HDS, class Polyhedron, class Point>
	class polyhedron_builder : public CGAL::Modifier_base<HDS>
	{
	public:
		std::vector<Point> vertex_;
		std::vector<std::vector<int> >    tris_;
		int nhole_;

		polyhedron_builder(std::vector<Point> vertex, std::vector<std::vector<int> > tris, int nhole = 0) :
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

	template<class HDS, class Polyhedron, class Point>
	int write_Polyhedron_off(const std::string &filename, std::vector<Point> &vertex, std::vector<std::vector<int> > &tris, int nhole = 0)
	{
		Polyhedron P;
		polyhedron_builder<HDS, Polyhedron, Point> builder(vertex, tris, nhole);
		P.delegate(builder);

		// write the polyhedron out as a .OFF file
		std::ofstream os(filename);
		os << P;
		os.close();
		
		return (0);
	}

}

