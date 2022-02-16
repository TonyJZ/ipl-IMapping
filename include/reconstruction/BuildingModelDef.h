#pragma once
#include <vector>

//#include <Core/iplgeometry_def.h>



namespace ipl
{
	namespace bim   //building information model
	{
		enum WallType
		{
			WT_NONWALL = -1,    //  not considered as a wall
			WT_UNKNOWN = 0,
			WT_FAKE,	       // a hypothetical wall 
			WT_ONESIDE,        //one-side wall  generally means the boundary of the building
			WT_DOUBLESIDE      //double-side wall the inner separated walls
		};

		////////////// geometry structure //////////////////
		//primitive

		//element


		///////////////  semantics definition   //////////////////////
		//typedef char  Description;
		
		/////////////////////////////////////////
		enum BoundaryType
		{
			BT_Outter = 0,
			BT_Inner
		};
		const char *BoundaryTypeDesc[] = { "Outter","Inner"};

		///////////////////////////////////////////
		enum SurfaceType
		{
			ST_RoofSurface = 0,
			ST_WallSurface,
			ST_GroundSurface,
			ST_ClosureSurface,
			ST_FloorSurface,
			ST_InteriorWallSurface,
			ST_CeilingSurface,
			ST_Door,
			ST_Window
		};
		const char *SurfaceTypeDesc[] = { "RoofSurface",
												"WallSurface",
												"GroundSurface",
												"ClosureSurface",
												"FloorSurface",
												"InteriorWallSurface",
												"CeilingSurface",
												"Door",
												"Window"};

		///////////////////////////////////////////
		enum ElementType
		{

		};

		//////////////////////////////////////////
		enum PrimitiveType
		{

		};
	}
}


