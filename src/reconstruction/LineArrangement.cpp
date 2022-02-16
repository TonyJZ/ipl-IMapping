#include "reconstruction/LineArrangement.h"

namespace ipl
{
LineArrangement::LineArrangement()
{

}

LineArrangement::~LineArrangement()
{

}

int LineArrangement::Initialize(const OGRMultiLineString *line_segs)
{
	arr_.clear();

	//读取OGRMultiLineString
	int nGeoCount = line_segs->getNumGeometries();
	int iLine = 0;

	const OGRGeometry *poLineGeometry;
	for (iLine = 0; iLine < nGeoCount; iLine++)
	{
		poLineGeometry = line_segs->getGeometryRef(iLine);

		OGRLineString *poLineString = (OGRLineString *)poLineGeometry;

		int nnCount = poLineString->getNumPoints();

		for (int j = 0; j < nnCount-1; j++)
		{
			double x1, x2, y1, y2;

			x1 = poLineString->getX(j);
			y1 = poLineString->getY(j);
			x2 = poLineString->getX(j+1);
			y2 = poLineString->getY(j+1);
			
			Point_2 p1(x1, y1), p2(x2, y2);

			Segment_2 s(p1, p2);
			insert(arr_, s);
		}

	}

	return 0;
}

// int LineArrangement::Initialize()
// {
// 	Landmarks_pl landmarks_pl;
// 	landmarks_pl.attach(arr_);
// 
// 	return 0;
// }

int LineArrangement::insertLines(std::vector<geometry::LineSeg2D> &lineSegs)
{
	for (int i = 0; i < lineSegs.size(); ++i)
	{
		double x1, x2, y1, y2;

		x1 = lineSegs[i].sp[0];
		y1 = lineSegs[i].sp[1];
		x2 = lineSegs[i].ep[0];
		y2 = lineSegs[i].ep[1];

		Point_2 p1(x1, y1), p2(x2, y2);

		Segment_2 s(p1, p2);
		insert(arr_, s);
	}

	return 0;
}

LineArrangement::Arrangement_2* LineArrangement::getArrangment()
{
	Landmarks_pl landmarks_pl;
	landmarks_pl.attach(arr_);

	return &arr_;
}

int LineArrangement::getOutterBoundary(std::vector<OGRPolygon> &polygons, const double floor_hei /* = 0 */)
{
	polygons.clear();

	Face_handle unf = arr_.unbounded_face();

	int fid = 0;
	char fname[32];

	// Traverse the inner boundary (holes).
	Hole_iterator hit;
	for (hit = unf->holes_begin(); hit != unf->holes_end(); ++hit, ++fid)
	{
		//fid = hit->data();
//		sprintf(fname, "polygon_%03d", fid);

// 		OGRFeature *poFeature;
// 		poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
// 		poFeature->SetField(0, fname);
// 		poFeature->SetField(1, fid);

		OGRLinearRing ring;

		typename Arrangement_2::Ccb_halfedge_circulator cch = *hit;
		do {
			typename Arrangement_2::Face_handle inner_face = cch->twin()->face();
			if (inner_face == cch->face())
				continue;        // discard antennas

			int id = cch->face()->data();
			int innerfid = cch->twin()->face()->data();

			int eid = cch->data();
			int teid = cch->twin()->data();

			double x, y;
			x = CGAL::to_double(cch->target()->point().x())/*.to_double()*/;
			y = CGAL::to_double(cch->target()->point().y())/*.to_double()*/;
			ring.addPoint(x, y, floor_hei);

		} while (++cch != *hit);

		ring.closeRings();//首尾点重合形成闭合环 
		OGRPolygon poly;
		poly.addRing(&ring);

		polygons.push_back(poly);
	}

	return 0;
}

int LineArrangement::getAllPolygons(std::vector<OGRPolygon> &polygons, const double floor_hei /* = 0 */)
{

	return 0;
}

}


