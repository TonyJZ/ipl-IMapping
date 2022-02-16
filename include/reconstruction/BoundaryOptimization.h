#pragma once
#include "core/iplcore.h"
#include "feature/PointGeoSegment.h"
#include "reconstruction/LineArrangement.h"
#include "spatialindexing/PointPartitionQuadtree.h"

//GDAL
#include <GDAL/gdal_alg.h>
#include <GDAL/gdal_priv.h>   //添加GDAL库函数的头文件
#include <GDAL/ogrsf_frmts.h>


namespace ipl
{
	template <typename PointT>
	class BoundaryOptimization
	{
	public:
		typedef ref_ptr <std::vector<int> > IndicesPtr;

		typedef struct BuildingCombo
		{
			std::string building_points_filename;

			float zRoof, zWallbase, zProfile;

			std::vector<int> poly_Indices;
			std::vector<int> roof_Indices;
			std::vector<int> wall_Indices;

			geoModel::BoundingBox bbox;
		};

// 		struct LayoutAtt     //facet, edge 的附加属性
// 		{
// 			//int id;          //primitive's ID
// 			float occupied;  //有点的面积/长度
// 			float empty;    //总的面积/长度
// 		};

		struct LayoutFaceAtt     //facet的附加属性
		{
			//int id;          //primitive's ID
// 			float occupied;  //有点的面积/长度
// 			float empty;    //总的面积/长度
			float sparsity;  //稀疏度sparseness
			double area;     //face 的几何面积
		};
 
		struct LayoutEdgeAtt     //edge 的附加属性
		{
			//int id;          //primitive's ID
// 			float occupied;  //有点的面积/长度
// 			float empty;    //总的面积/长度
			float sparsity;
			double length;   //几何长度 note: 用实际的几何长度，可以避免微小线段的影响(极端情况是重合的端点)
		};

		typedef std::vector<int>			VertexIDList;
		typedef boost::unordered_map<int, VertexIDList>	PolygonList;

		typedef std::vector<LineArrangement::Vertex_handle>  VertexHandleList;

	public:
		BoundaryOptimization();
		~BoundaryOptimization();

		void initialize(const std::string input_folder, const std::string output_folder);

		//添加1.boundary polygon; 2. wall projected lines 建立平面layout
		int createLayout2D(float bufSize = 2.0);

		int computeAttributesForArrangement(const float wall_Gridsize, const float roof_Gridsize);

		int optimize(/*double inner_edge_Th_, double border_edge_Th_,*/ double polygon_area_Th_);

		int exportOutterBoundary(const std::string &filename);

		int exportAllPolygons(const std::string &filename, const bool output_holes = false);

	protected:

		void getWallKeyMap(CellKeyMap *keymap);

		//RoofFlag: 0, profile points; 1, roof segments
		void getRoofKeyMap(CellKeyMap *keymap, int RoofFlag=0);


		void release();

		int loadPointSegments(const std::string dir, BuildingCombo &bc);
		int loadPolygons(const std::string dir, BuildingCombo &bc);

		int exportPolygons(const std::string filename, const std::vector<OGRPolygon> &Polys,
			std::vector<float> rHeis, std::vector<float> fHeis);

		int exportKeyMap(const std::string filename, const std::vector<OGRPolygon> &Polys,
			const std::vector<std::pair<QuadtreeKey, VoxelContainerPointIndices> >  keyCells);

		int exportLineSegments(const std::string filename, const std::vector<geometry::LineSeg2D> &lineSegs);

		int exportLineSegments(const std::string filename, const LineArrangement::Arrangement_2 *arr);


		//从墙面中提取投影直线方程
		int extractProjectedLinesFromPlanes();

		int addPolygonToArrangment(OGRPolygon &poly, geoModel::BoundingBox bbox,
			std::vector<geometry::LineSeg2D> &lineSegs);

		//直线与BBox相交的线段
		int getLineSeg(const double sp[2], const double ep[2], const geoModel::BoundingBox &bbox,
			geometry::LineSeg2D &lineSeg);

		int getLineSeg(const iplModelCoeffs &mCoef, const geoModel::BoundingBox &bbox,
			geometry::LineSeg2D &lineSeg);

		void removeEdges(LineArrangement::Arrangement_2 *arr, std::set<LineArrangement::Halfedge_handle> *removal_list);  //删除列表中的边

		void mergeFaces(LineArrangement::Arrangement_2 *arr, std::set<LineArrangement::Face_handle> *face_list); //合并列表中的面

		void mergeCollinearEdges(LineArrangement::Arrangement_2 *arr);     //合并共线的边


		//repair self-intersection polygons in line arrangement; discard
		int repairLineArrangement_selfintersection(LineArrangement::Arrangement_2 *arr, PolygonList &polygons);  
		//修复拓扑正确但存在环的多边形
		int repairLineArrangement_ring(LineArrangement::Arrangement_2 *arr, PolygonList &polygons, bool keep_antenna = false); 

		//修复多边形中的天线和环
		int repairPolygon(LineArrangement::Arrangement_2 *arr, LineArrangement::Face_handle fi, VertexHandleList &polygons);


	private:
		std::string    input_dir_;
		std::string    output_dir_;

		std::vector<PointGeoSegment<PointT> > wall_segments_;
		std::vector<PointGeoSegment<PointT> > roof_segments_;

		ref_ptr<iplPointCloud<PointT> > roof_cloud_;
		ref_ptr<iplPointCloud<PointT> > wall_cloud_;

		std::vector<IndicesPtr> wall_seg_indices_;
		std::vector<IndicesPtr> roof_seg_indices_;

		std::vector<int>   wall_p2cLut_;  //墙面点云对应的聚类号
		std::vector<int>   roof_p2cLut_;  //顶面点云对应的聚类号

		std::vector<OGRPolygon> aShapes_;    //初始边框

		std::vector<BuildingCombo>  building_combos_;

		std::vector<iplModelCoeffs> projected_lines_;

		geoModel::BoundingBox       scene_bbox_;	//整个区域的范围

	private:
		ref_ptr<LineArrangement>        LA_;
		LineArrangement::Arrangement_2	*arr2_;

		std::vector<std::vector<int> > face_lables_; //[0]: outter; [1]: inner
		std::vector<bool> face_flags_;				//inner:true;  outter: false
		
		//std::vector<bool> halfedge_traverse_flag_;

		int init_nvertices_, init_nhalfedges_, init_nfaces_; //初始的顶点、半边、面数


		boost::unordered_map<int, LayoutEdgeAtt> halfedge_att_list_;  //边属性列表
		boost::unordered_map<int, LayoutFaceAtt> face_att_list_;      //面属性列表

		//for debug, 仅为了方便分析算法
		std::vector<LayoutEdgeAtt>  edge_att_vec_;
		std::vector<LayoutFaceAtt>  face_att_vec_;

//		std::vector<LayoutEdgeAtt> edgeAttList;

		CellKeyMap   keymap_wall_, keymap_roof_;
		float  wall_gridsize_, roof_gridsize_;
		float mean_pts_num_wall_, mean_pts_num_roof_;

	private:
		double alpha_, beta_;  //能量函数调整

		std::vector<LineArrangement::Point_3>  points_;
	};
}

#include "reconstruction/impl/BoundaryOptimization.hpp"
