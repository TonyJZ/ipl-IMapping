#pragma once
#include "core/iplmacro.h"
#include "core/interface/IAlgorithm.h"
#include "core/interface/IProgressMsg.h"

namespace ipl
{
	interface IAlgSkeletonExtractor : public IAlgorithm
	{
	public:

		virtual int process(IProgressMsg *prg) = 0;

		IPL_INTERFACE_DEF(IAlgorithm, "skeletonextractor");
	};

#define IPL_SKELETONEXTRACTOR_DEFAULT		"ipl.algorithm.skeletonextractor.CGAL"
}
