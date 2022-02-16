// utest_skeleton.cpp : Defines the entry point for the console application.
//
#include "core/iplPlatformUtil.h"
#include "core/interface/IPlatform.h"
#include "spatialreference/interface/ISpatialReferenceService.h"
#include "simplefeature/interface/ISFService.h"
#include "simplefeature/interface/ISFVectorLayer.h"
#include "simplefeature/interface/ISFVectorSource.h"

#include "shapeanalysis/interface/IAlgSkeletonExtractor.h"

ipl::IPlatform *g_pPlatform = NULL;

ipl::IPlatform *ipl::getPlatform()
{
	return g_pPlatform;
}

namespace ipl
{
	IPL_GET_SRS_SERVICE_IMPL();
	IPL_GET_SF_SERVICE_IMPL();
}

#include<boost/shared_ptr.hpp>
#include<CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include<CGAL/Polygon_with_holes_2.h>
#include<CGAL/create_straight_skeleton_from_polygon_with_holes_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2                    Point;
typedef CGAL::Polygon_2<K>            Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes;
typedef CGAL::Straight_skeleton_2<K>  Ss;
typedef boost::shared_ptr<Ss> SsPtr;

template<class K>
void print_straight_skeleton(CGAL::Straight_skeleton_2<K> const& ss)
{
	typedef CGAL::Straight_skeleton_2<K> Ss;

	typedef typename Ss::Vertex_const_handle     Vertex_const_handle;
	typedef typename Ss::Halfedge_const_handle   Halfedge_const_handle;
	typedef typename Ss::Halfedge_const_iterator Halfedge_const_iterator;

	Halfedge_const_handle null_halfedge;
	Vertex_const_handle   null_vertex;

	std::cout << "Straight skeleton with " << ss.size_of_vertices()
		<< " vertices, " << ss.size_of_halfedges()
		<< " halfedges and " << ss.size_of_faces()
		<< " faces" << std::endl;

	for (Halfedge_const_iterator hit = ss.halfedges_begin(); hit != ss.halfedges_end(); ++hit)
	{
		Halfedge_const_handle h = hit;

		if (h->is_bisector() && ((h->id() % 2) == 0) && !h->has_infinite_time() && !h->opposite()->has_infinite_time())
		{
			std::cout << "skeleton..." << std::endl
				<< h->vertex()->point().x()
				<< " "
				<< h->vertex()->point().y()
				<< " "
				<< h->opposite()->vertex()->point().x()
				<< " "
				<< h->opposite()->vertex()->point().y()
				<< " E\n";
		}
	}
}

void testCGALSs()
{
	Polygon_2 outer;

// 	outer.push_back(Point(-1, -1));
// 	outer.push_back(Point(0, -12));
// 	outer.push_back(Point(1, -1));
// 	outer.push_back(Point(12, 0));
// 	outer.push_back(Point(1, 1));
// 	outer.push_back(Point(0, 12));
// 	outer.push_back(Point(-1, 1));
// 	outer.push_back(Point(-12, 0));

	outer.push_back(Point(-50, 50));
	outer.push_back(Point(50, 50));
	outer.push_back(Point(50, -50));
	outer.push_back(Point(-50, -50));

	Polygon_2 hole;

	hole.push_back(Point(-1, 0));
	hole.push_back(Point(0, 1));
	hole.push_back(Point(1, 0));
	hole.push_back(Point(0, -1));

	Polygon_with_holes poly(outer);

//	poly.add_hole(hole);

	// You can pass the polygon via an iterator pair
	SsPtr iss = CGAL::create_interior_straight_skeleton_2(poly);
	// Or you can pass the polygon directly, as below.

	// To create an exterior straight skeleton you need to specify a maximum offset.
// 	double lMaxOffset = 5;
// 	SsPtr oss = CGAL::create_exterior_straight_skeleton_2(lMaxOffset, poly);

	print_straight_skeleton(*iss);
//	print_straight_skeleton(*oss);
}

using namespace ipl;
int main()
{
// 	testCGALSs();
// 	return 0;

	char filename[] = "D:/iplTestData/utest/test_skeleton//manual_multipolys.shp";
	char outputname[] = "D:/iplTestData/utest/test_skeleton/manual_multipolys_ske.shp";

	iplString errorinfo;
	g_pPlatform = iplInitialize(errorinfo);

	ref_ptr<ISFVectorSourceReader> pVectorReader;
	pVectorReader = ref_ptr<ISFVectorSourceReader>(getSFService()->OpenSFFile(filename, false));

	ref_ptr<ISFVectorSource> inputSource = pVectorReader->GetVectorSource();
	iplArray<ref_ptr<IDataSource> > sources;
	sources.push_back(inputSource);

	pVectorReader->Close();

	ref_ptr<IAlgSkeletonExtractor> SkeExtractor;

	SkeExtractor = ref_ptr<IAlgSkeletonExtractor>(IPL_CREATE_OBJECT(IAlgSkeletonExtractor, IPL_SKELETONEXTRACTOR_DEFAULT));

	SkeExtractor->setInputSource(sources);

	ref_ptr<IProgressMsg> prg = ref_ptr<IProgressMsg>(IPL_CREATE_OBJECT(IProgressMsg, IPL_PROGRESSMSG_CMD));
	SkeExtractor->process(prg.get());

	iplArray<ref_ptr<IDataSource> > dstSources;
	SkeExtractor->getOutputSource(dstSources);

	ref_ptr<ISFVectorSourceWriter> pVectorWriter;
	pVectorWriter = ref_ptr<ISFVectorSourceWriter>(getSFService()->CreateSFFile(outputname, SF_FILE_FORMAT_SHP));

	ref_ptr<ISFVectorSource> dstSkeSource = ipl_static_pointer_cast<ISFVectorSource>(dstSources[0]);
	pVectorWriter->SetVectorSource(dstSkeSource);
	pVectorWriter->Close();
	pVectorWriter.reset();


	sources.clear();
	inputSource.reset();
	pVectorReader.reset();

	prg.reset();
	SkeExtractor.reset();

	dstSources.clear();
	dstSkeSource.reset();
	pVectorWriter.reset();
	iplUninitialize();

	return 0;
}

