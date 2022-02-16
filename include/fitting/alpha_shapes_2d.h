#pragma once
#include "core/iplcore.h"
#include <CGAL/Polygon_2.h>

//GDAL
#include <GDAL/gdal_alg.h>
#include <GDAL/gdal_priv.h>   //添加GDAL库函数的头文件
#include <GDAL/ogrsf_frmts.h>

namespace ipl
{
	//weights和cloud对应
	//line_segs: 顺序排列的点链
	// 不能直接得到有序点链, 需要利用Arrangement算法处理
	// http://cgal-discuss.949826.n4.nabble.com/Convert-Alpha-Segments-to-an-ordered-Polygon-td3575098.html
	template <typename PointT>
	int alpha_shape_2d(const pcl::PointCloud<PointT> &cloud, std::vector<int> *indices, bool has_weight,
		std::vector<float> &weights,
		OGRPolygon &line_segs,
		float alpha = -1);

	template <typename PointT>
	int alpha_shape_2d(const pcl::PointCloud<PointT> &cloud, std::vector<int> *indices, bool has_weight,
		std::vector<float> &weights,
		OGRMultiLineString &line_segs,
		float alpha = -1);
}

#include "fitting/impl/alpha_shapes_2d.hpp"

