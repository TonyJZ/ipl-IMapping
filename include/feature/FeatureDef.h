#pragma once

#include <Core/iplcore.h>

namespace ipl
{
	namespace feature
	{
		typedef std::vector<double>  POSITION;
		typedef std::vector<float>   DESCRIPTOR;
		typedef std::vector<int> edgeINDICE; //the edge indices connected to intersection point

		//list supported feature types below
		enum FeatureType
		{
			FT_Unknown = 0,
			FT_Intersection2D, 
			FT_Intersection3D
		};

		//平面投影直线的属性
		typedef struct
		{
			double ptS[3], ptE[3]; //直线上最远点对坐标

			
		} ProjectedLineAttribute;

		typedef std::vector<ProjectedLineAttribute> edgeATTRIBUTES;

		//the intersection point structure
		typedef struct
		{
			FeatureType type;

			double  p[4];   //coordinates

			edgeINDICE connIndices;    //connected model's ID
			edgeATTRIBUTES  connAtts;   //connected model's attributes

			float weight;    //the weight of intersection point. 

			std::vector<double> coefVar;  //估计参数的方差
		} IntersectionPoint;

	}

	IPL_BASE_API void getBBox(const std::vector<feature::IntersectionPoint> &ip2ds, double min_pt[3], double max_pt[3]);
}

