#pragma once
#include "core/iplmacro.h"
#include "ShapeAnalysis/interface/IAlgSkeletonExtractor.h"
#include "simplefeature/interface/ISFGeometry.h"

namespace ipl
{
	class XAlgSkeletonExtractor : public IAlgSkeletonExtractor
	{
	public:
		XAlgSkeletonExtractor();
		virtual ~XAlgSkeletonExtractor();

		virtual int process(IProgressMsg *prg);

		virtual bool setInputSource(iplArray<ref_ptr<IDataSource> > &input);

		virtual bool getOutputSource(iplArray<ref_ptr<IDataSource> > &output);

		virtual bool setArguments(ref_ptr<IProperty> &parameterArgs);

		virtual int TestCapability(const char *pszCapability);

	private:
		ISFMultiLineString* getSkeleton(ISFPolygon *poly);
		ISFMultiLineString* getSkeleton(ISFMultiPolygon *multipoly);

	private:
		iplArray<ref_ptr<IDataSource> >  m_srcDataSources;
		iplArray<ref_ptr<IDataSource> >  m_dstDataSources;
		ref_ptr<IProperty>		m_paramArgs;

	public:
		IPL_OBJECT_IMP2(XAlgSkeletonExtractor, IAlgSkeletonExtractor, IAlgorithm, "CGAL", "CGAL VectorLayer")
	};
}

