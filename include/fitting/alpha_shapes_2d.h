#pragma once
#include "core/iplcore.h"
#include <CGAL/Polygon_2.h>

//GDAL
#include <GDAL/gdal_alg.h>
#include <GDAL/gdal_priv.h>   //���GDAL�⺯����ͷ�ļ�
#include <GDAL/ogrsf_frmts.h>

namespace ipl
{
	//weights��cloud��Ӧ
	//line_segs: ˳�����еĵ���
	// ����ֱ�ӵõ��������, ��Ҫ����Arrangement�㷨����
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

