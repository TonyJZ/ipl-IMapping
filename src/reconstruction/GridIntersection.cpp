#include "reconstruction/GridIntersection.h"

//CGAL
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/Polygon_2_algorithms.h>
#include <CGAL/intersections.h>

int 
ipl::GridIntersection::InitialGrid(const double bbmin[3], const double bbmax[3], const float size_x, const float size_y)
{
	Eigen::Vector4f bbmin4f, bbmax4f;

	bbmin4f[0] = bbmin[0]; bbmin4f[1] = bbmin[1]; bbmin4f[2] = bbmin[2]; bbmin4f[3] = 1.0;
	bbmax4f[0] = bbmax[0]; bbmax4f[1] = bbmax[1]; bbmax4f[2] = bbmax[2]; bbmax4f[3] = 1.0;

	return InitialGrid(bbmin4f, bbmax4f, size_x, size_y);
}

int
ipl::GridIntersection::InitialGrid(const Eigen::Vector4f bbmin, const Eigen::Vector4f bbmax, 
	const float size_x, const float size_y)
{
	bbmin_ = bbmin;
	bbmax_ = bbmax;

	leaf_size_[0] = size_x; leaf_size_[1] = size_y; 
	// Avoid division errors
	//	if (leaf_size_[3] == 0)
	leaf_size_[2] = 1;
	leaf_size_[3] = 1;
	// Use multiplications instead of divisions
	inverse_leaf_size_ = Eigen::Array4f::Ones() / leaf_size_.array();


	// Check that the leaf size is not too small, given the size of the data
	int64_t dx = static_cast<int64_t>((bbmax_[0] - bbmin_[0]) * inverse_leaf_size_[0]) + 1;
	int64_t dy = static_cast<int64_t>((bbmax_[1] - bbmin_[1]) * inverse_leaf_size_[1]) + 1;
	//	int64_t dz = static_cast<int64_t>((max_p[2] - min_p[2]) * inverse_leaf_size_[2]) + 1;

	if ((dx*dy) > static_cast<int64_t>(std::numeric_limits<int32_t>::max()))
	{
		std::cout<<"[pcl::UrbanRec::VoxelFCGraph] Leaf size is too small for the input dataset. Integer indices would overflow." 
			<< std::endl;
		//output = *input_;
		return (-1);
	}

	vNumX_ = dx; vNumY_ = dy;

	return (0);
}

int 
ipl::GridIntersection::getIntersectedGrids(const Polygon_2 &polygon, ipl::CellKeyMap &grid_indices)
{
	CGAL::Bbox_2 bbox = polygon.bbox();
	double xmin = bbox.xmin();
	double xmax = bbox.xmax();
	double ymin = bbox.ymin();
	double ymax = bbox.ymax();

	int iStart, jStart, iEnd, jEnd; //起止标号

	iStart = static_cast<int> (floor((xmin - bbmin_[0]) * inverse_leaf_size_[0]));
	jStart = static_cast<int> (floor((ymin - bbmin_[1]) * inverse_leaf_size_[1]));

	iEnd = static_cast<int> (floor((xmax - bbmin_[0]) * inverse_leaf_size_[0]));
	jEnd = static_cast<int> (floor((ymax - bbmin_[1]) * inverse_leaf_size_[1]));

	if (iStart < 0)
		iStart = 0;
	if (iEnd >= vNumX_)
		iEnd = vNumX_ - 1;
	if (jStart < 0)
		jStart = 0;
	if (jEnd >= vNumY_)
		jEnd = vNumY_ - 1;

	grid_indices.clear();
	int ix, iy;

	for (ix = iStart; ix <= iEnd; ++ix)
	{
		for (iy = jStart; iy <= jEnd; ++iy)
		{
			Polygon_2 cube;

			double x, y;
			
			//ix, iy
			x = bbmin_[0] + ix*leaf_size_[0];
			y = bbmin_[1] + iy*leaf_size_[1];
			cube.push_back(Point_2(x, y));

			//ix+1, iy
			x = bbmin_[0] + (ix+1)*leaf_size_[0];
			y = bbmin_[1] + iy*leaf_size_[1];
			cube.push_back(Point_2(x, y));

			//ix+1, iy+1
			x = bbmin_[0] + (ix+1)*leaf_size_[0];
			y = bbmin_[1] + (iy+1)*leaf_size_[1];
			cube.push_back(Point_2(x, y));

			//ix, iy+1
			x = bbmin_[0] + ix*leaf_size_[0];
			y = bbmin_[1] + (iy+1)*leaf_size_[1];
			cube.push_back(Point_2(x, y));

			if ((CGAL::do_intersect(polygon, cube)))
			{
				QuadtreeKey key_arg;
				key_arg.x = ix;
				key_arg.y = iy;

				ipl::VoxelContainerPointIndices dummyIndices;
				grid_indices.insert(std::make_pair(key_arg, dummyIndices));
			}
			
		}
	}

	return (0);
}

int
ipl::GridIntersection::getIntersectedGridsAndOverlappingArea(const Polygon_2 &polygon, CellKeyMap &grid_indices,
	IntersectionAreaMap &overlapMap)
{
	CGAL::Bbox_2 bbox = polygon.bbox();
	double xmin = bbox.xmin();
	double xmax = bbox.xmax();
	double ymin = bbox.ymin();
	double ymax = bbox.ymax();

	int iStart, jStart, iEnd, jEnd; //起止标号

	iStart = static_cast<int> (floor((xmin - bbmin_[0]) * inverse_leaf_size_[0]));
	jStart = static_cast<int> (floor((ymin - bbmin_[1]) * inverse_leaf_size_[1]));

	iEnd = static_cast<int> (floor((xmax - bbmin_[0]) * inverse_leaf_size_[0]));
	jEnd = static_cast<int> (floor((ymax - bbmin_[1]) * inverse_leaf_size_[1]));

	if (iStart < 0)
		iStart = 0;
	if (iEnd >= vNumX_)
		iEnd = vNumX_ - 1;
	if (jStart < 0)
		jStart = 0;
	if (jEnd >= vNumY_)
		jEnd = vNumY_ - 1;

	grid_indices.clear();
	overlapMap.clear();

	int ix, iy;

	for (ix = iStart; ix <= iEnd; ++ix)
	{
		for (iy = jStart; iy <= jEnd; ++iy)
		{
			Polygon_2 cube;

			double x, y;

			//ix, iy
			x = bbmin_[0] + ix*leaf_size_[0];
			y = bbmin_[1] + iy*leaf_size_[1];
			cube.push_back(Point_2(x, y));

			//ix+1, iy
			x = bbmin_[0] + (ix + 1)*leaf_size_[0];
			y = bbmin_[1] + iy*leaf_size_[1];
			cube.push_back(Point_2(x, y));

			//ix+1, iy+1
			x = bbmin_[0] + (ix + 1)*leaf_size_[0];
			y = bbmin_[1] + (iy + 1)*leaf_size_[1];
			cube.push_back(Point_2(x, y));

			//ix, iy+1
			x = bbmin_[0] + ix*leaf_size_[0];
			y = bbmin_[1] + (iy + 1)*leaf_size_[1];
			cube.push_back(Point_2(x, y));

			if ((CGAL::do_intersect(polygon, cube)))
			{
				QuadtreeKey key_arg;
				key_arg.x = ix;
				key_arg.y = iy;

				ipl::VoxelContainerPointIndices dummyIndices;
				grid_indices.insert(std::make_pair(key_arg, dummyIndices));

				std::list<Polygon_with_holes_2> polyI;
				CGAL::intersection(polygon, cube, std::back_inserter(polyI));

				double totalArea = 0;
				typedef std::list<Polygon_with_holes_2>::iterator LIT;
				for (LIT lit = polyI.begin(); lit != polyI.end(); lit++) 
				{
					totalArea += CGAL::to_double(lit->outer_boundary().area());
				}

				overlapMap.insert(std::make_pair(key_arg, totalArea));
			}

		}
	}

	return (0);
}

int 
ipl::GridIntersection::getIntersectedGrids(const ipl::geometry::LineSeg2D seg, ipl::CellKeyMap &grid_indices)
{
	double xs, ys, xe, ye;
	double xmin, xmax, ymin, ymax;

	//保证x方向为正方向
	if (seg.sp[0] < seg.ep[0])
	{
		xs = seg.sp[0];
		ys = seg.sp[1];
		xe = seg.ep[0];
		ye = seg.ep[1];
	}
	else
	{
		xs = seg.ep[0];
		ys = seg.ep[1];
		xe = seg.sp[0];
		ye = seg.sp[1];
	}
	//计算斜率
	double dx = xe - xs;
	double dy = ye - ys;

	xmin = xs; 	xmax = xe;
	ymin = ys;	ymax = ye;
	if (ys > ye)
		std::swap(ymin, ymax);

	int iStart, jStart, iEnd, jEnd; //起止标号
// 	int ijk0, ijk1;
// 
// 	ijk0 = static_cast<int> (floor((xs - bbmin_[0]) * inverse_leaf_size_[0]));
// 	ijk1 = static_cast<int> (floor((ys - bbmin_[1]) * inverse_leaf_size_[1]));
// 	
// 	iStart = iEnd = ijk0;
// 	jStart = jEnd = ijk1;
// 
// 	ijk0 = static_cast<int> (floor((xe - bbmin_[0]) * inverse_leaf_size_[0]));
// 	ijk1 = static_cast<int> (floor((ye - bbmin_[1]) * inverse_leaf_size_[1]));
// 	 	
// 	if (iStart > ijk0)
// 		iStart = ijk0;
// 	if (iEnd < ijk0)
// 		iEnd = ijk0;
// 	if (jStart > ijk1)
// 		jStart = ijk1;
// 	if (jEnd < ijk1)
// 		jEnd = ijk1;

	iStart = static_cast<int> (floor((xmin - bbmin_[0]) * inverse_leaf_size_[0]));
	jStart = static_cast<int> (floor((ymin - bbmin_[1]) * inverse_leaf_size_[1]));

	iEnd = static_cast<int> (ceil((xmax - bbmin_[0]) * inverse_leaf_size_[0]));
	jEnd = static_cast<int> (ceil((ymax - bbmin_[1]) * inverse_leaf_size_[1]));

	if (iStart < 0)
		iStart = 0;
	if (iEnd >= vNumX_)
		iEnd = vNumX_ - 1;
	if (jStart < 0)
		jStart = 0;
	if (jEnd >= vNumY_)
		jEnd = vNumY_ - 1;
	
// 	int wid = clip_bbmax[0] - clip_bbmin[0];
// 	int hei = clip_bbmax[1] - clip_bbmin[1];

	grid_indices.clear(); //线段穿过的格网列表

	//特殊线段：极小的线段, 只存在一个gridcell中
	if (iStart == iEnd && jStart == jEnd)
	{
		QuadtreeKey key_arg;
		key_arg.x = iStart;
		key_arg.y = jStart;

		ipl::VoxelContainerPointIndices dummyIndices;
		grid_indices.insert(std::make_pair(key_arg, dummyIndices));
		
		return (0);
	}

	int ix, iy;
	if (fabs(dx) > fabs(dy))
	{//水平方向优先

		for (ix = iStart; ix <= iEnd; ++ix)
		{
			double x = bbmin_[0] + ix*leaf_size_[0];
			
			if ( x<xs || x>xe )
				continue;
			
			double y;

			if (dy == 0)
			{
				assert(jStart == jEnd);
				iy = jStart;
			}
			else
			{
				y = ys + (x - xs) / dx*dy;        //直线与扫描线的交点
				iy = static_cast<int> (floor((y - bbmin_[1]) * inverse_leaf_size_[1]));
			}

			if(iy<jStart || iy>jEnd)
				continue;

			QuadtreeKey key_arg;
			key_arg.x = ix;
			key_arg.y = iy;
			
			//std::cout << "find cell: " << ix  << ", " << iy << std::endl;

			CellKeyMap::iterator it_vm;
			it_vm = grid_indices.find(key_arg);
			if (it_vm == grid_indices.end())
			{
				ipl::VoxelContainerPointIndices dummyIndices;
				grid_indices.insert(std::make_pair(key_arg, dummyIndices));
			}

			double y0 = bbmin_[1] + iy * inverse_leaf_size_[1];
			double y1 = bbmin_[1] + (iy+1) * inverse_leaf_size_[1];

			if (fabs(y - y0) < 1e-6 || fabs(y - y1) < 1e-6)
				continue; //对角线上，只有一个网格

			key_arg.x = ix - 1;
			key_arg.y = iy;
			if(key_arg.x<iStart||key_arg.x>iEnd)
				continue;
			
			it_vm = grid_indices.find(key_arg);
			if (it_vm == grid_indices.end())
			{
				ipl::VoxelContainerPointIndices dummyIndices;
				grid_indices.insert(std::make_pair(key_arg, dummyIndices));
			}
		}
	}
	else
	{//垂直方向优先
		double ymin, ymax;
		if (ys < ye)
		{
			ymin = ys;
			ymax = ye;
		}
		else
		{
			ymin = ye;
			ymax = ys;
		}

		for (iy = jStart; iy <= jEnd; ++iy)
		{
			double x;
			double y = bbmin_[1] + iy*leaf_size_[1];

			if(y<ymin || y>ymax)
				continue;

			if (dx == 0)
			{
				assert(iStart == iEnd);
				ix = iStart;
			}
			else
			{
				x = xs + (y - ys) * dx / dy;        //直线与扫描线的交点
				ix = static_cast<int> (floor((x - bbmin_[0]) * inverse_leaf_size_[0]));
			}

			if(ix<iStart || ix>iEnd)
				continue;

			QuadtreeKey key_arg;
			key_arg.x = ix;
			key_arg.y = iy;

			//std::cout << "find cell: " << ix << ", " << iy << std::endl;

			CellKeyMap::iterator it_vm;
			it_vm = grid_indices.find(key_arg);
			if (it_vm == grid_indices.end())
			{
				ipl::VoxelContainerPointIndices dummyIndices;
				grid_indices.insert(std::make_pair(key_arg, dummyIndices));
			}

			double x0 = bbmin_[0] + ix * inverse_leaf_size_[0];
			double x1 = bbmin_[0] + (ix+1) * inverse_leaf_size_[0];

			if (fabs(x - x0) < 1e-6 || fabs(x - x1) < 1e-6)
				continue;

			key_arg.x = ix;
			key_arg.y = iy - 1;

			if(key_arg.y<jStart || key_arg.y>jEnd)
				continue;

			it_vm = grid_indices.find(key_arg);
			if (it_vm == grid_indices.end())
			{
				ipl::VoxelContainerPointIndices dummyIndices;
				grid_indices.insert(std::make_pair(key_arg, dummyIndices));
			}
		}
	}

	//补上第一块和最后一块
	QuadtreeKey key_arg;
	CellKeyMap::iterator it_vm;

	ix = static_cast<int> (floor((xs - bbmin_[0]) * inverse_leaf_size_[0]));
	iy = static_cast<int> (floor((ys - bbmin_[1]) * inverse_leaf_size_[1]));
	key_arg.x = ix;
	key_arg.y = iy;
	it_vm = grid_indices.find(key_arg);
	if (it_vm == grid_indices.end())
	{
		ipl::VoxelContainerPointIndices dummyIndices;
		grid_indices.insert(std::make_pair(key_arg, dummyIndices));
	}

	ix = static_cast<int> (floor((xe - bbmin_[0]) * inverse_leaf_size_[0]));
	iy = static_cast<int> (floor((ye - bbmin_[1]) * inverse_leaf_size_[1]));
	key_arg.x = ix;
	key_arg.y = iy;
	it_vm = grid_indices.find(key_arg);
	if (it_vm == grid_indices.end())
	{
		ipl::VoxelContainerPointIndices dummyIndices;
		grid_indices.insert(std::make_pair(key_arg, dummyIndices));
	}

	return (0);
}

int
ipl::GridIntersection::getIntersectedGridsAndLineSections(const ipl::geometry::LineSeg2D seg, ipl::CellKeyMap &grid_indices,
	ipl::LineSectionKeyMap &lineSections)
{
	double xs, ys, xe, ye;
	double xmin, xmax, ymin, ymax;

	//保证x方向为正方向
	if (seg.sp[0] < seg.ep[0])
	{
		xs = seg.sp[0];
		ys = seg.sp[1];
		xe = seg.ep[0];
		ye = seg.ep[1];
	}
	else
	{
		xs = seg.ep[0];
		ys = seg.ep[1];
		xe = seg.sp[0];
		ye = seg.sp[1];
	}
	//计算斜率
	double dx = xe - xs;
	double dy = ye - ys;

	xmin = xs; 	xmax = xe;
	ymin = ys;	ymax = ye;
	if (ys > ye)
		std::swap(ymin, ymax);

	int iStart, jStart, iEnd, jEnd; //起止标号

	iStart = static_cast<int> (floor((xmin - bbmin_[0]) * inverse_leaf_size_[0]));
	jStart = static_cast<int> (floor((ymin - bbmin_[1]) * inverse_leaf_size_[1]));

	iEnd = static_cast<int> (ceil((xmax - bbmin_[0]) * inverse_leaf_size_[0]));
	jEnd = static_cast<int> (ceil((ymax - bbmin_[1]) * inverse_leaf_size_[1]));

	if (iStart < 0)
		iStart = 0;
	if (iEnd >= vNumX_)
		iEnd = vNumX_ - 1;
	if (jStart < 0)
		jStart = 0;
	if (jEnd >= vNumY_)
		jEnd = vNumY_ - 1;

	grid_indices.clear(); //线段穿过的格网列表
	lineSections.clear();

	//特殊线段：极小的线段, 只存在一个gridcell中
	if (iStart == iEnd && jStart == jEnd)
	{
		QuadtreeKey key_arg;
		key_arg.x = iStart;
		key_arg.y = jStart;

		ipl::VoxelContainerPointIndices dummyIndices;
		grid_indices.insert(std::make_pair(key_arg, dummyIndices));

		ipl::geometry::LineSeg2D line;
		line.sp[0] = xs; line.sp[1] = ys;
		line.ep[0] = xe; line.ep[1] = ye;

		lineSections.insert(std::make_pair(key_arg, line));

		return (0);
	}

	int ix, iy;
	if (fabs(dx) > fabs(dy))
	{//水平方向优先

		for (ix = iStart; ix <= iEnd; ++ix)
		{
			double x = bbmin_[0] + ix*leaf_size_[0];

			if (x<xs || x>xe)
				continue;

			double y;

			if (dy == 0)
			{
				assert(jStart == jEnd);
				iy = jStart;
			}
			else
			{
				y = ys + (x - xs) / dx*dy;        //直线与扫描线的交点
				iy = static_cast<int> (floor((y - bbmin_[1]) * inverse_leaf_size_[1]));
			}

			if (iy<jStart || iy>jEnd)
				continue;

			QuadtreeKey key_arg;
			key_arg.x = ix;
			key_arg.y = iy;

			//std::cout << "find cell: " << ix  << ", " << iy << std::endl;

			CellKeyMap::iterator it_vm;
			it_vm = grid_indices.find(key_arg);
			if (it_vm == grid_indices.end())
			{
				ipl::VoxelContainerPointIndices dummyIndices;
				grid_indices.insert(std::make_pair(key_arg, dummyIndices));

				iplRECT<double> rect;
				rect.m_xmin = bbmin_[0] + key_arg.x*leaf_size_[0];
				rect.m_xmax = rect.m_xmin + leaf_size_[0];
				rect.m_ymin = bbmin_[1] + key_arg.y*leaf_size_[1];
				rect.m_ymax = rect.m_ymin + leaf_size_[1];

				ipl::geometry::LineSeg2D line;
				getInnerLineSection(xs, ys, xe, ye, rect, line);
				lineSections.insert(std::make_pair(key_arg, line));
			}

			double y0 = bbmin_[1] + iy * inverse_leaf_size_[1];
			double y1 = bbmin_[1] + (iy + 1) * inverse_leaf_size_[1];

			if (fabs(y - y0) < 1e-6 || fabs(y - y1) < 1e-6)
				continue; //对角线上，只有一个网格

			key_arg.x = ix - 1;
			key_arg.y = iy;
			if (key_arg.x<iStart || key_arg.x>iEnd)
				continue;

			it_vm = grid_indices.find(key_arg);
			if (it_vm == grid_indices.end())
			{
				ipl::VoxelContainerPointIndices dummyIndices;
				grid_indices.insert(std::make_pair(key_arg, dummyIndices));

				iplRECT<double> rect;
				rect.m_xmin = bbmin_[0] + key_arg.x*leaf_size_[0];
				rect.m_xmax = rect.m_xmin + leaf_size_[0];
				rect.m_ymin = bbmin_[1] + key_arg.y*leaf_size_[1];
				rect.m_ymax = rect.m_ymin + leaf_size_[1];

				ipl::geometry::LineSeg2D line;
				getInnerLineSection(xs, ys, xe, ye, rect, line);
				lineSections.insert(std::make_pair(key_arg, line));
			}
		}
	}
	else
	{//垂直方向优先

		for (iy = jStart; iy <= jEnd; ++iy)
		{
			double x;
			double y = bbmin_[1] + iy*leaf_size_[1];

			if (y<ymin || y>ymax)
				continue;

			if (dx == 0)
			{
				assert(iStart == iEnd);
				ix = iStart;
			}
			else
			{
				x = xs + (y - ys) * dx / dy;        //直线与扫描线的交点
				ix = static_cast<int> (floor((x - bbmin_[0]) * inverse_leaf_size_[0]));
			}

			if (ix<iStart || ix>iEnd)
				continue;

			QuadtreeKey key_arg;
			key_arg.x = ix;
			key_arg.y = iy;

			//std::cout << "find cell: " << ix << ", " << iy << std::endl;

			CellKeyMap::iterator it_vm;
			it_vm = grid_indices.find(key_arg);
			if (it_vm == grid_indices.end())
			{
				ipl::VoxelContainerPointIndices dummyIndices;
				grid_indices.insert(std::make_pair(key_arg, dummyIndices));

				iplRECT<double> rect;
				rect.m_xmin = bbmin_[0] + key_arg.x*leaf_size_[0];
				rect.m_xmax = rect.m_xmin + leaf_size_[0];
				rect.m_ymin = bbmin_[1] + key_arg.y*leaf_size_[1];
				rect.m_ymax = rect.m_ymin + leaf_size_[1];

				ipl::geometry::LineSeg2D line;
				getInnerLineSection(xs, ys, xe, ye, rect, line);
				lineSections.insert(std::make_pair(key_arg, line));
			}

			double x0 = bbmin_[0] + ix * inverse_leaf_size_[0];
			double x1 = bbmin_[0] + (ix + 1) * inverse_leaf_size_[0];

			if (fabs(x - x0) < 1e-6 || fabs(x - x1) < 1e-6)
				continue;

			key_arg.x = ix;
			key_arg.y = iy - 1;

			if (key_arg.y<jStart || key_arg.y>jEnd)
				continue;

			it_vm = grid_indices.find(key_arg);
			if (it_vm == grid_indices.end())
			{
				ipl::VoxelContainerPointIndices dummyIndices;
				grid_indices.insert(std::make_pair(key_arg, dummyIndices));

				iplRECT<double> rect;
				rect.m_xmin = bbmin_[0] + key_arg.x*leaf_size_[0];
				rect.m_xmax = rect.m_xmin + leaf_size_[0];
				rect.m_ymin = bbmin_[1] + key_arg.y*leaf_size_[1];
				rect.m_ymax = rect.m_ymin + leaf_size_[1];

				ipl::geometry::LineSeg2D line;
				getInnerLineSection(xs, ys, xe, ye, rect, line);
				lineSections.insert(std::make_pair(key_arg, line));
			}
		}
	}

	//补上第一块和最后一块
	QuadtreeKey key_arg;
	CellKeyMap::iterator it_vm;

	ix = static_cast<int> (floor((xs - bbmin_[0]) * inverse_leaf_size_[0]));
	iy = static_cast<int> (floor((ys - bbmin_[1]) * inverse_leaf_size_[1]));
	key_arg.x = ix;
	key_arg.y = iy;
	it_vm = grid_indices.find(key_arg);
	if (it_vm == grid_indices.end())
	{
		ipl::VoxelContainerPointIndices dummyIndices;
		grid_indices.insert(std::make_pair(key_arg, dummyIndices));

		iplRECT<double> rect;
		rect.m_xmin = bbmin_[0] + key_arg.x*leaf_size_[0];
		rect.m_xmax = rect.m_xmin + leaf_size_[0];
		rect.m_ymin = bbmin_[1] + key_arg.y*leaf_size_[1];
		rect.m_ymax = rect.m_ymin + leaf_size_[1];

		ipl::geometry::LineSeg2D line;
		getInnerLineSection(xs, ys, xe, ye, rect, line);
		lineSections.insert(std::make_pair(key_arg, line));
	}

	ix = static_cast<int> (floor((xe - bbmin_[0]) * inverse_leaf_size_[0]));
	iy = static_cast<int> (floor((ye - bbmin_[1]) * inverse_leaf_size_[1]));
	key_arg.x = ix;
	key_arg.y = iy;
	it_vm = grid_indices.find(key_arg);
	if (it_vm == grid_indices.end())
	{
		ipl::VoxelContainerPointIndices dummyIndices;
		grid_indices.insert(std::make_pair(key_arg, dummyIndices));

		iplRECT<double> rect;
		rect.m_xmin = bbmin_[0] + key_arg.x*leaf_size_[0];
		rect.m_xmax = rect.m_xmin + leaf_size_[0];
		rect.m_ymin = bbmin_[1] + key_arg.y*leaf_size_[1];
		rect.m_ymax = rect.m_ymin + leaf_size_[1];

		ipl::geometry::LineSeg2D line;
		getInnerLineSection(xs, ys, xe, ye, rect, line);
		lineSections.insert(std::make_pair(key_arg, line));
	}

	return (0);
}

int
ipl::GridIntersection::getInnerLineSection(double xs, double ys, double xe, double ye,
	iplRECT<double> rect, ipl::geometry::LineSeg2D &line)
{
	if (xs > xe)
	{
		std::swap(xs, xe);
		std::swap(ys, ye);
	}

	double ymin, ymax;
	if (ys < ye)
	{
		ymin = ys;
		ymax = ye;
	}
	else
	{
		ymin = ye;
		ymax = ys;
	}

	double dx = xe - xs;
	double dy = ye - ys;

	if (fabs(dx) < 1e-6)
	{//垂直线
		line.sp[0] = xs;
		line.sp[1] = rect.m_ymin;
		line.ep[0] = xs;
		line.ep[1] = rect.m_ymax;

		if (line.sp[1] < ymin)
			line.sp[1] = ymin;
		if (line.ep[1] > ymax)
			line.ep[1] = ymax;
	}
	else if (fabs(dy) < 1e-6)
	{//水平线
		line.sp[0] = rect.m_xmin;
		line.sp[1] = ys;
		line.ep[0] = rect.m_xmax;
		line.ep[1] = ys;

		if (line.sp[0] < xs)
			line.sp[0] = xs;
		if (line.ep[0] > xe)
			line.ep[0] = xe;
	}
	else
	{
		double x[4], y[4]; //从全部四个交点中选取在rect中的点

		x[0] = rect.m_xmin;
		y[0] = ys + (x[0] - xs) / dx*dy;   

		x[1] = rect.m_xmax;
		y[1] = ys + (x[1] - xs) / dx*dy;

		y[2] = rect.m_ymin;
		x[2] = xs + (y[2] - ys) * dx / dy;

		y[3] = rect.m_ymax;
		x[3] = xs + (y[3] - ys) * dx / dy;

		std::vector<int> indices;
		for (int i = 0; i < 4; ++i)
		{
			if(x[i]<rect.m_xmin
				|| x[i]>rect.m_xmax
				|| y[i]<rect.m_ymin
				|| y[i]>rect.m_ymax)
				continue;
			indices.push_back(i);
		}

		assert(indices.size() == 2);

		line.sp[0] = x[indices[0]];
		line.sp[1] = y[indices[0]];
		line.ep[0] = x[indices[1]];
		line.ep[1] = y[indices[1]];

		if (line.sp[0] > line.ep[0])
		{
			std::swap(line.sp[0], line.ep[0]);
			std::swap(line.sp[1], line.ep[1]);
		}

		if (line.sp[0] < xs)
		{
			line.sp[0] = xs;
			line.sp[1] = ys;
		}

		if (line.ep[0] > xe)
		{
			line.ep[0] = xe;
			line.ep[1] = ye;
		}
	}

	return 0;
}

