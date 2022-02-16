#include "feature/IntersectionPoint2DSet.h"

using namespace ipl;
using namespace ipl::feature;

ipl::IntersectionPoint2DSet::IntersectionPoint2DSet()
{

}

ipl::IntersectionPoint2DSet::~IntersectionPoint2DSet()
{

}

// void 
// ipl::IntersectionPoint2DSet::addEdgeIndice(const edgeINDICE &indice)
// {
// 	connected_indices_.push_back(indice);
// }

int 
ipl::convert_FeatureStructure(const IntersectionPoint2DSet &feat_set, std::vector<IntersectionPoint> &ipt2ds)
{
	std::vector<POSITION> pos = feat_set.getPositions();
	std::vector<DESCRIPTOR> desc = feat_set.getDescriptors();
	std::vector<edgeINDICE> conne_indices = feat_set.getEdgeIndices();
	std::vector<edgeATTRIBUTES> conne_atts = feat_set.getEdgeAttributes();
	std::vector<std::vector<double> > intersection_variances = feat_set.getIntersectionVariances();

	size_t ptNum = pos.size();
	if (ptNum != desc.size() || ptNum != conne_indices.size())
		return (-1);

	ipt2ds.clear();
	ipt2ds.resize(ptNum);
	for (size_t i = 0; i < ptNum; i++)
	{
		IntersectionPoint pt;
		
		pt.type = FT_Intersection2D;

		pt.p[0] = pos[i].at(0);
		pt.p[1] = pos[i].at(1);
		pt.p[2] = pos[i].at(2);
		pt.p[3] = pos[i].at(3);

		pt.connIndices = conne_indices[i];
		pt.connAtts = conne_atts[i];
		pt.weight = desc[i].at(0);

		pt.coefVar = intersection_variances[i];

		ipt2ds[i] = pt;
	}

	return (0);
}

void 
ipl::getBBox(const std::vector<feature::IntersectionPoint> &ip2ds, double min_pt[3], double max_pt[3])
{
	min_pt[0] = min_pt[1] = min_pt[2] = std::numeric_limits<double>::max();
	max_pt[0] = max_pt[1] = max_pt[2] = std::numeric_limits<double>::lowest();

	for (size_t i = 0; i < ip2ds.size(); ++i)
	{
		const double *pt = ip2ds[i].p;

		if (pt[0] < min_pt[0])
			min_pt[0] = pt[0];
		if (pt[0] > max_pt[0])
			max_pt[0] = pt[0];

		if (pt[1] < min_pt[1])
			min_pt[1] = pt[1];
		if (pt[1] > max_pt[1])
			max_pt[1] = pt[1];

		if (pt[2] < min_pt[2])
			min_pt[2] = pt[2];
		if (pt[2] > max_pt[2])
			max_pt[2] = pt[2];
	}

	return;
}

