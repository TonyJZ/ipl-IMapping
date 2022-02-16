#pragma once
#include "core/iplcore.h"
#include "feature/PointGeoSegment.h"
#include "spatialindexing/PointPartitionQuadtree.h"

#include <Eigen/dense>

//cgal
#include <CGAL/Cartesian.h>
#include <CGAL/Exact_rational.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arr_extended_dcel.h>
//#include <CGAL/Arrangement_2.h>
#include <CGAL/Arrangement_with_history_2.h>
#include <CGAL/IO/Arr_text_formatter.h>
#include <CGAL/IO/Arr_iostream.h>
//#include <CGAL/Arr_simple_point_location.h>
#include <CGAL/Arr_landmarks_point_location.h>
#include <CGAL/Polyhedron_3.h>

class OGRMultiPolygon;
namespace ipl
{
	class IPL_BASE_API FloorMapReconstruction
	{
	public:
		struct LayoutAtt     //facet, edge �ĸ�������
		{
			//int id;          //primitive's ID
			float occupied;  //�е�����/����
			float empty;    //�ܵ����/����
		};

		typedef iplPointXYZRGBNormal  PointT;
		typedef boost::shared_ptr <std::vector<int> > IndicesPtr;

		
		typedef CGAL::Cartesian<CGAL::Exact_rational>							Kernel;
		typedef CGAL::Arr_segment_traits_2<Kernel>								Traits_2;
		typedef Traits_2::Point_2												Point_2;
		typedef Traits_2::Curve_2												Segment_2;
		typedef CGAL::Arr_extended_dcel<Traits_2, int, int, int>	Dcel;    //vertex id, edge id, face id
		typedef CGAL::Arrangement_with_history_2<Traits_2, Dcel>				Arrangement_2;
		typedef CGAL::Arr_landmarks_point_location<Arrangement_2>				Landmarks_pl;
		//typedef CGAL::Arr_extended_dcel_text_formatter<Arrangement_2>  Formatter;

		typedef Arrangement_2::Curve_const_handle						Curve_const_handle;
		typedef Arrangement_2::Vertex_const_handle						Vertex_const_handle;
		typedef Arrangement_2::Halfedge_const_handle					Halfedge_const_handle;
		typedef Arrangement_2::Face_const_handle						Face_const_handle;

		typedef Arrangement_2::Curve_handle								Curve_handle;
		typedef Arrangement_2::Vertex_handle							Vertex_handle;
		typedef Arrangement_2::Halfedge_handle							Halfedge_handle;
		typedef Arrangement_2::Face_handle								Face_handle;
		
		typedef Arrangement_2::Curve_iterator		Curve_iterator;
		typedef Arrangement_2::Edge_iterator		Edge_iterator;
		typedef Arrangement_2::Halfedge_iterator	Halfedge_iterator;
		typedef Arrangement_2::Face_iterator		Face_iterator;
		typedef Arrangement_2::Hole_iterator		Hole_iterator;

		typedef CGAL::Polyhedron_3<Kernel>  Polyhedron;

		typedef std::vector<int>			VertexIDList;
		typedef boost::unordered_map<int, VertexIDList>	PolygonList;
		//typedef Polyhedron::Vertex_handle   Vertex_handle;

	public:
		FloorMapReconstruction();
		virtual ~FloorMapReconstruction();

		int loadSegments(const std::string &wall_dir, const std::string &ext, const std::string &nonwall);

		//wGridsize��������Ҫ����ǽ����Ƶ���Χ���ο�WallRefinement��mergeDoubleSideWalls������
		//cGridsize��������Ҫ����
		int createLayout(const float wGridsize = 0.2, const float cGridsize = 0.2); // indoor layout extracted by graph-cut

		int extractFloormap();

		void setOutputDir(const char* pDir);
		int exportOutterBoundary(const std::string &filename, const double floor_hei);
		int exportInnerPolylines(const std::string &filename, const double floor_hei, const bool bfiltered = true);
		int exportAllPolygons(const std::string &filename, const double floor_hei, const bool output_holes=false);
		

	protected:
		int lineArrangement(); //���ֿռ�

		int getOutterBoundary(OGRMultiPolygon &polygons, const double floor_hei = 0);
		int getAllPolygons(OGRMultiPolygon &polygons, const double floor_hei = 0);

		int repairLineArrangement_selfintersection(Arrangement_2 *arr, PolygonList &polygons);  //repair self-intersection polygons in line arrangement;

		int repairLineArrangement_ring(Arrangement_2 *arr, PolygonList &polygons, bool keep_antenna=false); //�޸�������ȷ�����ڻ��Ķ����

		void removeEdges(Arrangement_2 *arr, std::set<Halfedge_handle> *removal_list);  //ɾ���б��еı�

		void mergeFaces(Arrangement_2 *arr, std::set<Face_handle> *face_list); //�ϲ��б��е���

		void mergeCollinearEdges(Arrangement_2 *arr);     //�ϲ����ߵı�

		//����ͶӰֱ�߷��̵Ĳ�����ֱ����ƽ�淽�̴��ڴ��󣬴�ֱ���費����ʱ������ֱ����ƽ�淽�̵õ���ͶӰֱ������ʵλ��֮�����ƽ��2017.12.22
		int getProjectedLineCeoffs();  


	private:
		//for test
		int exportLineArrangement_off(Arrangement_2 *arr, const std::string &filename, const double floor_hei);

		int exportLineArrangement_TXT(Arrangement_2 *arr, const std::string &filename, const double floor_hei);
		
		int exportFaceofLineArrangement_off(Arrangement_2 *arr, const int fid, const std::string &filename, const double floor_hei);

		int export2DPolygon_txt(Arrangement_2 *arr, const int fid, const std::string &filename);

		int exportAll2DPolygon_txt(Arrangement_2 *arr, const std::string &dir);

	private:
		boost::shared_ptr<iplPointCloud<PointT> > nonwall_cloud_;
		boost::shared_ptr<iplPointCloud<PointT> > wall_cloud_;
		std::vector<IndicesPtr> seg_indices_;
		std::vector<PointGeoSegment<PointT> > segments_;
		std::vector<iplModelCoeffs> projected_lines_;

		std::vector<int>   p2cLut_;  //���ƶ�Ӧ�ľ����


		Eigen::Vector4f wall_bbmin_, wall_bbmax_;
		float  wall_gridsize_, ceiling_gridsize_;

		std::string		outputDir_;

	private:
		Arrangement_2   arr_;    //plane subdivision

		//CellList  cell_list_wall_, cell_list_nonwall_;
		CellKeyMap   keymap_wall_, keymap_nonwall_; 
		boost::unordered_map<int, LayoutAtt> halfedge_att_list_;
		boost::unordered_map<int, LayoutAtt> face_att_list_;

		std::vector<double> facet_area_list_;  //facet �����

		double alpha_, beta_;  //������������

		std::vector<std::vector<int> > face_lables_; //[0]: outter; [1]: inner
		std::vector<bool> face_flags_;    //inner:true;  outter: false

		double inner_edge_Th_;
		double border_edge_Th_;
		double polygon_area_Th_;

	private:
		//v, e, f in arrangement
		std::vector<Point_2>  vertices_list_;
		std::vector<int>  halfedge_face_list_;  //halfedge��Ӧ��face�����б�
		std::vector<int>  halfedge_twin_list_;
		std::vector<std::vector<int> > face_halfedge_list_;  //face������edge�����б�
		//holes  ��ʱ������
		std::vector<Polyhedron::Point_3>  points_;
	};
}

