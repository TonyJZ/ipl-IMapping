#include "XAlgSkeletonExtractor.h"
#include "spatialreference/interface/ISpatialReference.h"
#include "spatialreference/interface/ISpatialReferenceService.h"
#include "simplefeature/SFBaseDef.h"
#include "simplefeature/interface/ISFService.h"
#include "simplefeature/interface/ISFVectorLayer.h"
#include "simplefeature/interface/ISFVectorSource.h"

//cgal
#include<CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include<CGAL/Polygon_with_holes_2.h>
#include<CGAL/create_straight_skeleton_from_polygon_with_holes_2.h>

using namespace ipl;

XAlgSkeletonExtractor::XAlgSkeletonExtractor()
{

}

XAlgSkeletonExtractor::~XAlgSkeletonExtractor()
{
	m_srcDataSources.clear();
	m_dstDataSources.clear();
	m_paramArgs.reset();
}

int XAlgSkeletonExtractor::TestCapability(const char *pszCapability)
{

	return 0;
}

bool XAlgSkeletonExtractor::setInputSource(iplArray<ref_ptr<IDataSource> > &input)
{
	m_srcDataSources = input;
	return true;
}

bool XAlgSkeletonExtractor::getOutputSource(iplArray<ref_ptr<IDataSource> > &output)
{
	output = m_dstDataSources;
	return true;
}

bool XAlgSkeletonExtractor::setArguments(ref_ptr<IProperty> &parameterArgs)
{
	m_paramArgs = parameterArgs;
	return true;
}

int XAlgSkeletonExtractor::process(IProgressMsg *prg)
{
	size_t nSource = m_srcDataSources.size();

	if (nSource == 0)
		return 0;

	if (prg != NULL)
	{
		prg->logPrint(IPL_LOG_INFO, "skeleton extracting %d ...", nSource);
		prg->process(0.0);
	}
		
	m_dstDataSources.clear();
	m_dstDataSources.resize(nSource);
	for (size_t i = 0; i < nSource; ++i)
	{//只对封闭的多边形进行处理

		prg->process(float(i)/nSource);

		ISFVectorSource* pSrcSource = IPL_PTR_CAST(ISFVectorSource, m_srcDataSources[i].get());
		if (pSrcSource == NULL)
		{
			prg->logPrint(IPL_LOG_WARNING, "the %d is not a simple feature!", i);
			continue;
		}

		int nLayer = pSrcSource->GetLayerCount();

		ref_ptr<ISFVectorSource> pDstSource = ref_ptr<ISFVectorSource>(getSFService()->createSFVectorSource());
		if (pDstSource.get() == NULL)
		{
			prg->logPrint(IPL_LOG_ERROR, "can't create SF Vector Source!", i);
			return -1;
		}
		
		for (size_t iLayer = 0; iLayer < nLayer; ++iLayer)
		{//逐层提取
			ISFVectorLayer *pLayer = pSrcSource->GetLayer(iLayer);
			if (pLayer != NULL)
			{
				ref_ptr<ISpatialReference> pSRS = pLayer->GetSpatialReference();

				IPL_wkbGeometryType geoType = pLayer->GetGeometryType();

				iplString srcLayerName = pLayer->GetLayerName();
				
				ref_ptr<ISFVectorLayer> pDstLayer;
				pDstLayer = ref_ptr<ISFVectorLayer>(getSFService()->createSFVectorLayer());
				pDstLayer->SetSpatialReference(pSRS);
				pDstSource->addLayer(pDstLayer);

				if (geoType != IPL_wkbPolygon && geoType != IPL_wkbMultiPolygon
					&& geoType != IPL_wkbPolygon25D && geoType != IPL_wkbMultiPolygon25D)
				{
					pDstLayer->SetLayerName(srcLayerName.c_str());
					pDstLayer->SetGeometryType(IPL_wkbUnknown);
					
					continue;
				}

				iplString dstLayerName = srcLayerName + " skeleton";
				pDstLayer->SetLayerName(dstLayerName.c_str());
				pDstLayer->SetGeometryType(IPL_wkbLineString25D);

				ref_ptr<ISFFeatureDefn>   featDefn = ref_ptr<ISFFeatureDefn>(getSFService()->createFeatureDefn());

				ref_ptr<ISFFieldDefn> oFieldOrder = ref_ptr<ISFFieldDefn>(getSFService()->createFieldDefn());


				oFieldOrder->Set("name", OFTString, 32, 0, SF_JUndefined);
				featDefn->AddFieldDefn(oFieldOrder->Clone());
				oFieldOrder->Set("PolygonID", OFTInteger, 8, 0, SF_JUndefined);
				featDefn->AddFieldDefn(oFieldOrder->Clone());

				pDstLayer->SetFeatureDefn(featDefn);

				ISFFeature *feature;
				pLayer->ResetReading();
				int polyid = 0;
				while ((feature = pLayer->GetNextFeature()) != NULL)
				{//遍历多边形比提取skeleton
					IGeoObject *geom;

					geom = feature->GetGeometryRef();

					if (NULL == geom)
						continue;

					IPL_wkbGeometryType gType = geom->getGeometryType();
					IGeoObject *pSkt = NULL;

					//assert(gType == OSF_wkbPolygon);
					if (gType == IPL_wkbPolygon || gType == IPL_wkbPolygon25D)
					{
						ISFPolygon *poPolygon = static_cast<ISFPolygon*>(geom);

						pSkt = static_cast<IGeoObject*>(getSkeleton(poPolygon));
					}
					else if (gType == IPL_wkbMultiPolygon || gType == IPL_wkbMultiPolygon25D)
					{
						ISFMultiPolygon *poMultiPolygon = static_cast<ISFMultiPolygon*>(geom);

						pSkt = static_cast<IGeoObject*>(getSkeleton(poMultiPolygon));
					}

					ref_ptr<ISFFeature> sktfeat = ref_ptr<ISFFeature>(getSFService()->createFeature(featDefn));
					SFField sf;
					sf.String = "skeleton";
					sktfeat->SetField(0, &sf);
					sf.Integer = polyid;
					sktfeat->SetField(1, &sf);
					polyid++;

					sktfeat->SetGeometry(ref_ptr<IGeoObject>(pSkt));

					pDstLayer->AppendFeature(sktfeat.get());
				}
			}
		}

		m_dstDataSources[i] = pDstSource;
	}

	return 1;
}

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2                    Point;
typedef CGAL::Polygon_2<K>            Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes;
typedef CGAL::Straight_skeleton_2<K>  Ss;
typedef boost::shared_ptr<Ss> SsPtr;

void getLineStringsFromSkeleton(const SsPtr &iss, ISFMultiLineString *pLineStrings)
{
	typedef typename Ss::Vertex_const_handle     Vertex_const_handle;
	typedef typename Ss::Halfedge_const_handle   Halfedge_const_handle;
	typedef typename Ss::Halfedge_const_iterator Halfedge_const_iterator;

	Halfedge_const_handle null_halfedge;
	Vertex_const_handle   null_vertex;

	//pts.clear();

	std::cout << "Straight skeleton with " << iss->size_of_vertices()
		<< " vertices, " << iss->size_of_halfedges()
		<< " halfedges and " << iss->size_of_faces()
		<< " faces" << std::endl;

	for (Halfedge_const_iterator hit = iss->halfedges_begin(); hit != iss->halfedges_end(); ++hit)
	{
		Halfedge_const_handle h = hit;

		if (h->is_bisector() && ((h->id() % 2) == 0) 
			&& !h->has_infinite_time() && !h->opposite()->has_infinite_time())
		{
			ISFLineString *pLine = getSFService()->createLineString();
			if (pLine == NULL)
			{
				return ;
			}

			iplPOINT3D ptS, ptE;
			ptS.X = h->vertex()->point().x();
			ptS.Y = h->vertex()->point().y();
			ptS.Z = 0;

			ptE.X = h->opposite()->vertex()->point().x();
			ptE.Y = h->opposite()->vertex()->point().y();
			ptE.Z = 0;

			pLine->addPoint(ptS);
			pLine->addPoint(ptE);
			
			pLineStrings->addGeometryDirectly(ref_ptr<IGeoObject>(pLine));
		}
	}

}

ISFMultiLineString* XAlgSkeletonExtractor::getSkeleton(ISFPolygon *poly)
{
	if (poly == NULL)
		return NULL;

	ref_ptr<ISFLineRing> ext_ring = poly->getExteriorRing();
	const iplPOINT3D *pts = ext_ring->getPtsBuf();
	int npts = ext_ring->getNumPoints();

	Polygon_2 outer;
	//note: shp中外环顺时针，内环逆时针; shp重复记录首顶点形成闭环，cgal不重复
	for (int i = npts - 2; i >=0; --i)
	{
		outer.push_back(Point(pts[i].X, pts[i].Y));
	}
	Polygon_with_holes cgal_poly(outer);

	int nholes = poly->getNumInteriorRings();
	for (int ih = 0; ih < nholes; ++ih)
	{
		ref_ptr<ISFLineRing> inne_ring = poly->getInteriorRing(ih);
	
		Polygon_2 hole;

		const iplPOINT3D *pts = inne_ring->getPtsBuf();
		int npts = inne_ring->getNumPoints();
		for (int i = npts - 2; i >= 0; --i)
		{
			hole.push_back(Point(pts[i].X, pts[i].Y));
		}

		cgal_poly.add_hole(hole);
	}

	SsPtr iss = CGAL::create_interior_straight_skeleton_2(cgal_poly);

	ISFMultiLineString *pLineStrings = getSFService()->createMultiLineString();
	if (pLineStrings == NULL)
	{
		return NULL;
	}
		

	//std::vector<iplPOINT3D> sktpts;
	getLineStringsFromSkeleton(iss, pLineStrings);

	//pLineString->setPoints(&sktpts[0], sktpts.size());

	return pLineStrings;
}

ISFMultiLineString* XAlgSkeletonExtractor::getSkeleton(ISFMultiPolygon *multipoly)
{
	if (multipoly == NULL)
		return NULL;

	ISFMultiLineString *pMultiLineString = getSFService()->createMultiLineString();
	if (pMultiLineString == NULL)
	{
		return NULL;
	}

	int npoly = multipoly->getNumGeometries();
	for (int ipoly = 0; ipoly < npoly; ++ipoly)
	{
		ref_ptr<ISFPolygon> polygon = ipl_static_pointer_cast<ISFPolygon>(multipoly->getGeometryRef(ipoly));

		ref_ptr<ISFMultiLineString> pLineStrings = ref_ptr<ISFMultiLineString>(getSkeleton(polygon.get()));

		int nline = pLineStrings->getNumGeometries();
		for (int i = 0; i < nline; ++i)
		{
			ref_ptr<IGeoObject> pObj = pLineStrings->getGeometryRef(i);
			pMultiLineString->addGeometryDirectly(pObj);
		}
	}

	return pMultiLineString;
}
