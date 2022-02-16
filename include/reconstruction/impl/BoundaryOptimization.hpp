#pragma once
#include "reconstruction/BoundaryOptimization.h"
#include "commonAPIs/iplutility.h"
#include "commonAPIs/iplstring.h"
#include "io/PointcloudIO.h"
#include "reconstruction/reconstructionDef.h"
#include "fitting/ModelParamsIO.h"
#include "reconstruction/GridIntersection.h"
#include "optimization/IndoorLayoutOptimizationByGC.h"
#include "reconstruction/BuildingModelDef.h"


#ifdef HAVE_OpenMP
#include <omp.h>
#endif

#define OUTPUT_DEBUG_INFO

namespace ipl
{

	template <typename PointT>
	BoundaryOptimization<PointT>::BoundaryOptimization()
	{
		alpha_ = 1.0;
		beta_ = /*0.08*//*-0.5*/0.08;
	}

	template <typename PointT>
	BoundaryOptimization<PointT>::~BoundaryOptimization()
	{
		release();
	}

	template <typename PointT> void
		BoundaryOptimization<PointT>::release()
	{
		wall_cloud_.reset(new iplPointCloud <PointT>);
		roof_cloud_.reset(new iplPointCloud <PointT>);

		wall_segments_.clear();
		roof_segments_.clear();

		wall_seg_indices_.clear();
		roof_seg_indices_.clear();

		wall_p2cLut_.clear();
		roof_p2cLut_.clear();

		projected_lines_.clear();

		aShapes_.clear();

		building_combos_.clear();
	}

	template <typename PointT> int
		BoundaryOptimization<PointT>::loadPolygons(const std::string dir, BuildingCombo &bc)
	{
		std::string orgShapeFile = dir + "/" + rec::FolderName_AShape + "/" + rec::BBName_RFN;

		std::string result_name, suffix_name;
		extract_file_name(orgShapeFile, result_name, suffix_name);

		std::string simplifiedShapeFile = result_name + rec::BBName_Simplified + suffix_name;

		GDALAllRegister();
		GDALDataset       *poDS;
		poDS = (GDALDataset*)GDALOpenEx(simplifiedShapeFile.c_str(), GDAL_OF_VECTOR, NULL, NULL, NULL);
		if (poDS == NULL)
		{
			std::cout << "can't open file: " << simplifiedShapeFile << std::endl;
			return -1;
		}

		int nLayer = poDS->GetLayerCount();
		for (int i = 0; i < nLayer; ++i)
		{
			OGRLayer  *poLayer;

			//poLayer = poDS->GetLayerByName("point");
			poLayer = poDS->GetLayer(i);
			//const char *name = poLayer->GetName();

			OGRwkbGeometryType type = poLayer->GetGeomType();
			if (type == wkbPolygon || type == wkbMultiPolygon
				|| type == wkbPolygon25D || type == wkbMultiPolygon25D)
			{
				OGRFeature *poFeature;
				poLayer->ResetReading();
				while ((poFeature = poLayer->GetNextFeature()) != NULL)
				{
					int nfield = poFeature->GetFieldCount();
					for (int j = 0; j < nfield; ++j)
					{
						OGRFieldDefn* fDefn = poFeature->GetFieldDefnRef(j);
						const char *fieldName = fDefn->GetNameRef();

						if (strcmp(fieldName, rec::FieldDefn_ROOF_HEIGHT) == 0)
						{
							OGRField *field = poFeature->GetRawFieldRef(j);
							bc.zRoof = field->Real;
						}
						else if (strcmp(fieldName, rec::FieldDefn_FLOOR_HEIGHT) == 0)
						{
							OGRField *field = poFeature->GetRawFieldRef(j);
							bc.zWallbase = field->Real;
						}
						else if (strcmp(fieldName, rec::FieldDefn_PROFILE_HEIGHT) == 0)
						{
							OGRField *field = poFeature->GetRawFieldRef(j);
							bc.zProfile = field->Real;
						}
					}

					OGRGeometry *poGeometry;
					poGeometry = poFeature->GetGeometryRef();

					if (poGeometry == NULL)
						continue;
					if (wkbFlatten(poGeometry->getGeometryType()) == wkbPolygon
						|| wkbFlatten(poGeometry->getGeometryType()) == wkbPolygon25D)
					{
						OGRPolygon *poPoly = (OGRPolygon *)poGeometry;
						//polygons.addGeometry(poPoly);
						bc.poly_Indices.push_back(aShapes_.size());
						aShapes_.push_back(*poPoly);

						OGREnvelope3D envelop;
						poPoly->getEnvelope(&envelop);
						if (bc.bbox.min_pt[0] > envelop.MinX)
							bc.bbox.min_pt[0] = envelop.MinX;
						if (bc.bbox.min_pt[1] > envelop.MinY)
							bc.bbox.min_pt[1] = envelop.MinY;
						if (bc.bbox.min_pt[2] > envelop.MinZ)
							bc.bbox.min_pt[2] = envelop.MinZ;
						if (bc.bbox.max_pt[0] < envelop.MaxX)
							bc.bbox.max_pt[0] = envelop.MaxX;
						if (bc.bbox.max_pt[1] < envelop.MaxY)
							bc.bbox.max_pt[1] = envelop.MaxY;
						if (bc.bbox.max_pt[2] < envelop.MaxZ)
							bc.bbox.max_pt[2] = envelop.MaxZ;
					}
					else if (wkbFlatten(poGeometry->getGeometryType()) == wkbMultiPolygon
						|| wkbFlatten(poGeometry->getGeometryType()) == wkbMultiPolygon25D)
					{
						OGRMultiPolygon *poMultiPoly = (OGRMultiPolygon *)poGeometry;

						int nGeos = poMultiPoly->getNumGeometries();
						for (int iGeo = 0; iGeo < nGeos; ++iGeo)
						{
							OGRPolygon *poPoly = (OGRPolygon *)(poMultiPoly->getGeometryRef(iGeo));

							bc.poly_Indices.push_back(aShapes_.size());
							aShapes_.push_back(*poPoly);

							OGREnvelope3D envelop;
							poPoly->getEnvelope(&envelop);
							if (bc.bbox.min_pt[0] > envelop.MinX)
								bc.bbox.min_pt[0] = envelop.MinX;
							if (bc.bbox.min_pt[1] > envelop.MinY)
								bc.bbox.min_pt[1] = envelop.MinY;
							if (bc.bbox.min_pt[2] > envelop.MinZ)
								bc.bbox.min_pt[2] = envelop.MinZ;
							if (bc.bbox.max_pt[0] < envelop.MaxX)
								bc.bbox.max_pt[0] = envelop.MaxX;
							if (bc.bbox.max_pt[1] < envelop.MaxY)
								bc.bbox.max_pt[1] = envelop.MaxY;
							if (bc.bbox.max_pt[2] < envelop.MaxZ)
								bc.bbox.max_pt[2] = envelop.MaxZ;
						}
					}
					else
					{
						std::cout << "this feature is not a polygon!" << std::endl;
					}

					OGRFeature::DestroyFeature(poFeature);
				}
			}
		}

		GDALClose(poDS);

		return 0;
	}

	template <typename PointT> int
		BoundaryOptimization<PointT>::loadPointSegments(const std::string dir, BuildingCombo &bc)
	{
		std::string walls_dir = dir + "/" + rec::FolderName_Walls;
		std::string roofs_dir = dir + "/" + rec::FolderName_Roofs;

		ref_ptr<iplPointCloud<PointT> > seg_cloud(new iplPointCloud <PointT>);

		std::vector<std::string> filenames;

		//1. load wall segments 
		ipl::scan_files(walls_dir, ".pcd", filenames);

		int nSegs = filenames.size();

		std::cout << "load wall segments: " << walls_dir << std::endl;
		std::cout << "walls number: " << nSegs << std::endl;

		//seg_indices_.resize(nSegs);
		int curptId = wall_cloud_->size();
		int curSegId = wall_seg_indices_.size();

		for (int i = 0; i < nSegs; i++)
		{
			std::cout << "loading wall " << i << "\r";
			//提取分割点云和模型参数
			std::string rname, sname;

			ipl::extract_file_name(filenames[i], rname, sname);
			std::string paraname = rname;
			paraname += geoModel::ModelParamFile_Suffix;

			ipl::geoModel::geoModelInfo mCoef;
			//ipl::geoModel::geoModelType type;

			load_geoModel_params(paraname, /*bbmin, bbmax,*/ mCoef);

			//提取分割点云
			ipl::read_PointCloud(filenames[i], *seg_cloud);

			IndicesPtr indices(new std::vector<int>);
			for (int j = 0; j < seg_cloud->size(); j++)
			{//合并点云并记录每个分割的索引
				wall_cloud_->points.push_back(seg_cloud->points[j]);

				wall_p2cLut_.push_back(curSegId + i);

				indices->push_back(curptId);
				curptId++;
			}

			wall_seg_indices_.push_back(indices);

			ipl::PointGeoSegment<PointT> seg;

			seg.setModelCoef(mCoef);
			seg.setInputCloud(wall_cloud_);
			seg.setIndices(wall_seg_indices_.back());

			bc.wall_Indices.push_back(wall_segments_.size());
			wall_segments_.push_back(seg);

			if (bc.bbox.min_pt[0] > mCoef.bbox.min_pt[0])
				bc.bbox.min_pt[0] = mCoef.bbox.min_pt[0];
			if (bc.bbox.min_pt[1] > mCoef.bbox.min_pt[1])
				bc.bbox.min_pt[1] = mCoef.bbox.min_pt[1];
			if (bc.bbox.min_pt[2] > mCoef.bbox.min_pt[2])
				bc.bbox.min_pt[2] = mCoef.bbox.min_pt[2];
			if (bc.bbox.max_pt[0] < mCoef.bbox.max_pt[0])
				bc.bbox.max_pt[0] = mCoef.bbox.max_pt[0];
			if (bc.bbox.max_pt[1] < mCoef.bbox.max_pt[1])
				bc.bbox.max_pt[1] = mCoef.bbox.max_pt[1];
			if (bc.bbox.max_pt[2] < mCoef.bbox.max_pt[2])
				bc.bbox.max_pt[2] = mCoef.bbox.max_pt[2];
		}
		std::cout << "loading walls done!" << std::endl;
		wall_cloud_->height = 1;
		wall_cloud_->width = wall_cloud_->size();

		//初始化segments
		// 	for (int i = 0; i < nSegs; i++)
		// 	{
		// 		segments_[i].setInputCloud(wall_cloud_);
		// 		segments_[i].setIndices(seg_indices_[i]);
		// 	}


		//2. load roof segments 
		ipl::scan_files(roofs_dir, ".pcd", filenames);

		nSegs = filenames.size();

		std::cout << "load roof segments: " << roofs_dir << std::endl;
		std::cout << "roofs number: " << nSegs << std::endl;

		//seg_indices_.resize(nSegs);
		curptId = roof_cloud_->size();
		curSegId = roof_seg_indices_.size();

		for (int i = 0; i < nSegs; i++)
		{
			std::cout << "loading roof " << i << "\r";
			//提取分割点云和模型参数
			std::string rname, sname;

			ipl::extract_file_name(filenames[i], rname, sname);
			std::string paraname = rname;
			paraname += geoModel::ModelParamFile_Suffix;

			ipl::geoModel::geoModelInfo mCoef;
			//ipl::geoModel::geoModelType type;

			load_geoModel_params(paraname, /*bbmin, bbmax,*/ mCoef);

			//提取分割点云
			ipl::read_PointCloud(filenames[i], *seg_cloud);

			IndicesPtr indices(new std::vector<int>);
			for (int j = 0; j < seg_cloud->size(); j++)
			{//合并点云并记录每个分割的索引
				roof_cloud_->points.push_back(seg_cloud->points[j]);

				roof_p2cLut_.push_back(curSegId + i);

				indices->push_back(curptId);
				curptId++;
			}

			roof_seg_indices_.push_back(indices);

			ipl::PointGeoSegment<PointT> seg;

			seg.setModelCoef(mCoef);
			seg.setInputCloud(roof_cloud_);
			seg.setIndices(roof_seg_indices_.back());

			bc.roof_Indices.push_back(roof_segments_.size());
			roof_segments_.push_back(seg);

			if (bc.bbox.min_pt[0] > mCoef.bbox.min_pt[0])
				bc.bbox.min_pt[0] = mCoef.bbox.min_pt[0];
			if (bc.bbox.min_pt[1] > mCoef.bbox.min_pt[1])
				bc.bbox.min_pt[1] = mCoef.bbox.min_pt[1];
			if (bc.bbox.min_pt[2] > mCoef.bbox.min_pt[2])
				bc.bbox.min_pt[2] = mCoef.bbox.min_pt[2];
			if (bc.bbox.max_pt[0] < mCoef.bbox.max_pt[0])
				bc.bbox.max_pt[0] = mCoef.bbox.max_pt[0];
			if (bc.bbox.max_pt[1] < mCoef.bbox.max_pt[1])
				bc.bbox.max_pt[1] = mCoef.bbox.max_pt[1];
			if (bc.bbox.max_pt[2] < mCoef.bbox.max_pt[2])
				bc.bbox.max_pt[2] = mCoef.bbox.max_pt[2];
		}
		std::cout << "loading roofs done!" << std::endl;
		roof_cloud_->height = 1;
		roof_cloud_->width = roof_cloud_->size();

		return (0);
	}

	template <typename PointT> int
		BoundaryOptimization<PointT>::exportKeyMap(const std::string filename, const std::vector<OGRPolygon> &Polys,
			const std::vector<std::pair<QuadtreeKey, VoxelContainerPointIndices>> keyCells)
	{
		const char *pszDriverName = "ESRI Shapefile";
		GDALDriver *poDriver;
		GDALAllRegister();
		poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
		if (poDriver == NULL)
		{
			std::cout << pszDriverName << " driver not available" << std::endl;
			return (-1);
		}

		GDALDataset *poDS;
		poDS = poDriver->Create(filename.c_str(), 0, 0, 0, GDT_Unknown, NULL); //创建数据源
		if (poDS == NULL)
		{
			std::cout << "can't create output file: " << filename << std::endl;
			return (-1);
		}
		OGRLayer *poLayer_polygon;
		poLayer_polygon = poDS->CreateLayer("building boundary", NULL, wkbPolygon, NULL); //创建图层
		if (poLayer_polygon == NULL)
		{
			std::cout << "Layer creation failed" << std::endl;
			return (-1);
		}

		OGRFieldDefn oField(rec::FieldDefn_Polygon_ID, OFTInteger64);
		//oField1.SetPrecision(3);
		if (poLayer_polygon->CreateField(&oField) != OGRERR_NONE)
		{
			std::cout << "Creating PolygonID field failed." << std::endl;
			return (-1);
		}

		OGRFieldDefn oField1(rec::FieldDefn_Name, OFTString); //创建属性
		oField.SetWidth(256);
		if (poLayer_polygon->CreateField(&oField1) != OGRERR_NONE)
		{
			std::cout << "Creating PolygonName field failed" << std::endl;
			return (-1);
		}

		OGRFieldDefn oField2("GridX", OFTInteger64);
		//oField1.SetPrecision(3);
		if (poLayer_polygon->CreateField(&oField2) != OGRERR_NONE)
		{
			std::cout << "Creating PolygonID field failed." << std::endl;
			return (-1);
		}

		OGRFieldDefn oField3("GridY", OFTInteger64);
		//oField1.SetPrecision(3);
		if (poLayer_polygon->CreateField(&oField3) != OGRERR_NONE)
		{
			std::cout << "Creating PolygonID field failed." << std::endl;
			return (-1);
		}

		OGRFieldDefn oField4("PtNum", OFTInteger64);
		//oField1.SetPrecision(3);
		if (poLayer_polygon->CreateField(&oField4) != OGRERR_NONE)
		{
			std::cout << "Creating PolygonID field failed." << std::endl;
			return (-1);
		}

		assert(Polys.size() == keyCells.size());
		for (int i = 0; i < Polys.size(); ++i)
		{
			int fid = 0;
			char fname[32] = "";
			//	sprintf(fname, "building_%03d", fid);
			std::pair<QuadtreeKey, VoxelContainerPointIndices> cellAtt = keyCells[i];

			OGRFeature *poFeature;
			poFeature = OGRFeature::CreateFeature(poLayer_polygon->GetLayerDefn());
			poFeature->SetField(0, fname);
			poFeature->SetField(1, fid);
			int ix = cellAtt.first.x;
			int iy = cellAtt.first.y;
			int ptNum = cellAtt.second.getSize();

			poFeature->SetField(2, ix);
			poFeature->SetField(3, iy);
			poFeature->SetField(4, ptNum);

			poFeature->SetGeometry(&Polys[i]);
			if (poLayer_polygon->CreateFeature(poFeature) != OGRERR_NONE)
			{
				std::cout << "Failed to create feature: " << fname << std::endl;
			}
			OGRFeature::DestroyFeature(poFeature);
		}

		GDALClose(poDS);

		return (0);
	}

	template <typename PointT> int
		BoundaryOptimization<PointT>::exportPolygons(const std::string filename, const std::vector<OGRPolygon> &Polys,
			std::vector<float> rHeis, std::vector<float> fHeis)
	{
		const char *pszDriverName = "ESRI Shapefile";
		GDALDriver *poDriver;
		GDALAllRegister();
		poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
		if (poDriver == NULL)
		{
			std::cout << pszDriverName << " driver not available" << std::endl;
			return (-1);
		}

		GDALDataset *poDS;
		poDS = poDriver->Create(filename.c_str(), 0, 0, 0, GDT_Unknown, NULL); //创建数据源
		if (poDS == NULL)
		{
			std::cout << "can't create output file: " << filename << std::endl;
			return (-1);
		}
		OGRLayer *poLayer_polygon;
		poLayer_polygon = poDS->CreateLayer("building boundary", NULL, wkbPolygon, NULL); //创建图层
		if (poLayer_polygon == NULL)
		{
			std::cout << "Layer creation failed" << std::endl;
			return (-1);
		}

		OGRFieldDefn oField(rec::FieldDefn_Polygon_ID, OFTInteger64);
		//oField1.SetPrecision(3);
		if (poLayer_polygon->CreateField(&oField) != OGRERR_NONE)
		{
			std::cout << "Creating PolygonID field failed." << std::endl;
			return (-1);
		}

		OGRFieldDefn oField1(rec::FieldDefn_Name, OFTString); //创建属性
		oField.SetWidth(256);
		if (poLayer_polygon->CreateField(&oField1) != OGRERR_NONE)
		{
			std::cout << "Creating PolygonName field failed" << std::endl;
			return (-1);
		}

		OGRFieldDefn oField2(rec::FieldDefn_ROOF_HEIGHT, OFTReal);
		//oField1.SetPrecision(3);
		if (poLayer_polygon->CreateField(&oField2) != OGRERR_NONE)
		{
			std::cout << "Creating PolygonID field failed." << std::endl;
			return (-1);
		}

		OGRFieldDefn oField3(rec::FieldDefn_FLOOR_HEIGHT, OFTReal);
		//oField1.SetPrecision(3);
		if (poLayer_polygon->CreateField(&oField3) != OGRERR_NONE)
		{
			std::cout << "Creating PolygonID field failed." << std::endl;
			return (-1);
		}

		for (int i = 0; i < Polys.size(); ++i)
		{
			int fid = 0;
			char fname[32] = "";
			//	sprintf(fname, "building_%03d", fid);

			OGRFeature *poFeature;
			poFeature = OGRFeature::CreateFeature(poLayer_polygon->GetLayerDefn());
			poFeature->SetField(0, fname);
			poFeature->SetField(1, fid);
			poFeature->SetField(2, rHeis[i]);
			poFeature->SetField(3, fHeis[i]);

			poFeature->SetGeometry(&Polys[i]);
			if (poLayer_polygon->CreateFeature(poFeature) != OGRERR_NONE)
			{
				std::cout << "Failed to create feature: " << fname << std::endl;
			}
			OGRFeature::DestroyFeature(poFeature);
		}

		GDALClose(poDS);
		return (0);
	}

	template <typename PointT> int 
		BoundaryOptimization<PointT>::exportLineSegments(const std::string filename, const LineArrangement::Arrangement_2 *arr)
	{
		const char *pszDriverName = "ESRI Shapefile";
		GDALDriver *poDriver;
		GDALAllRegister();
		poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
		if (poDriver == NULL)
		{
			std::cout << pszDriverName << " driver not available" << std::endl;
			return (-1);
		}

		GDALDataset *poDS;
		poDS = poDriver->Create(filename.c_str(), 0, 0, 0, GDT_Unknown, NULL); //创建数据源
		if (poDS == NULL)
		{
			std::cout << "can't create output file: " << filename << std::endl;
			return (-1);
		}
		OGRLayer *poLayer;
		poLayer = poDS->CreateLayer("building boundary", NULL, wkbLineString25D, NULL); //创建图层
		if (poLayer == NULL)
		{
			std::cout << "Layer creation failed" << std::endl;
			return (-1);
		}

		OGRFieldDefn oField("Name", OFTString); //创建属性
		oField.SetWidth(32);
		if (poLayer->CreateField(&oField) != OGRERR_NONE)
		{
			std::cout << "Creating PolygonName field failed" << std::endl;
			return (-1);
		}

		OGRFieldDefn oField1("LineID", OFTInteger64);
		//oField1.SetPrecision(3);
		if (poLayer->CreateField(&oField1) != OGRERR_NONE)
		{
			std::cout << "Creating PolygonID field failed." << std::endl;
			return (-1);
		}

		OGRFieldDefn oField2("PID", OFTInteger64);
		//oField1.SetPrecision(3);
		if (poLayer->CreateField(&oField2) != OGRERR_NONE)
		{
			std::cout << "Creating PolygonID field failed." << std::endl;
			return (-1);
		}

		OGRFieldDefn oField3("T_PID", OFTInteger64);
		//oField1.SetPrecision(3);
		if (poLayer->CreateField(&oField3) != OGRERR_NONE)
		{
			std::cout << "Creating PolygonID field failed." << std::endl;
			return (-1);
		}

		OGRFieldDefn oField4("SPRS", OFTReal);
		//oField1.SetPrecision(3);
		if (poLayer->CreateField(&oField4) != OGRERR_NONE)
		{
			std::cout << "Creating PolygonID field failed." << std::endl;
			return (-1);
		}

		LineArrangement::Halfedge_iterator heit;
		std::vector<bool> halfedge_traverse_flag;

		std::cout << "calculate features for each edge..." << std::endl;
		halfedge_traverse_flag.resize(init_nhalfedges_, false);

		int eid;
		for (heit = arr2_->halfedges_begin(); heit != arr2_->halfedges_end(); ++heit)
		{
			eid = heit->data();
			std::cout << "processing: " << eid << "/" << init_nhalfedges_ << "\r";

			if (halfedge_traverse_flag[eid])
				continue;

			int t_eid = heit->twin()->data();

			halfedge_traverse_flag[eid] = halfedge_traverse_flag[t_eid] = true;

			int fid = heit->face()->data();
			int ft_id = heit->twin()->face()->data();

			OGRFeature *poFeature;
			poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
			poFeature->SetField(1, eid);
			poFeature->SetField(2, fid);
			poFeature->SetField(3, ft_id);

			boost::unordered_map<int, LayoutEdgeAtt>::iterator it_find = halfedge_att_list_.find(eid);
			if (it_find != halfedge_att_list_.end())
			{
				float sparse = halfedge_att_list_[eid].sparsity;
// 					halfedge_att_list_[eid].occupied /
// 					(halfedge_att_list_[eid].occupied + halfedge_att_list_[eid].empty);

				poFeature->SetField(4, sparse);
			}
			else
				poFeature->SetField(4, 0);

			OGRLineString line;

			double x, y;
			x = CGAL::to_double(heit->source()->point().x());
			y = CGAL::to_double(heit->source()->point().y());
			line.addPoint(x, y, 50);
			
			x = CGAL::to_double(heit->target()->point().x());
			y = CGAL::to_double(heit->target()->point().y());
			line.addPoint(x, y, 50);

			poFeature->SetGeometry(&line);

			if (poLayer->CreateFeature(poFeature) != OGRERR_NONE)
			{
				std::cout << "Failed to create feature: lines" << std::endl;
			}
			OGRFeature::DestroyFeature(poFeature);
		}

		GDALClose(poDS);

		return (0);
	}

	template <typename PointT> int 
		BoundaryOptimization<PointT>::exportLineSegments(const std::string filename, const std::vector<geometry::LineSeg2D> &lineSegs)
	{
		const char *pszDriverName = "ESRI Shapefile";
		GDALDriver *poDriver;
		GDALAllRegister();
		poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
		if (poDriver == NULL)
		{
			std::cout << pszDriverName << " driver not available" << std::endl;
			return (-1);
		}

		GDALDataset *poDS;
		poDS = poDriver->Create(filename.c_str(), 0, 0, 0, GDT_Unknown, NULL); //创建数据源
		if (poDS == NULL)
		{
			std::cout << "can't create output file: " << filename << std::endl;
			return (-1);
		}
		OGRLayer *poLayer;
		poLayer = poDS->CreateLayer("building boundary", NULL, wkbLineString25D, NULL); //创建图层
		if (poLayer == NULL)
		{
			std::cout << "Layer creation failed" << std::endl;
			return (-1);
		}

		OGRFieldDefn oField("Name", OFTString); //创建属性
		oField.SetWidth(32);
		if (poLayer->CreateField(&oField) != OGRERR_NONE)
		{
			std::cout << "Creating PolygonName field failed" << std::endl;
			return (-1);
		}

		OGRFieldDefn oField1("LineID", OFTInteger64);
		//oField1.SetPrecision(3);
		if (poLayer->CreateField(&oField1) != OGRERR_NONE)
		{
			std::cout << "Creating PolygonID field failed." << std::endl;
			return (-1);
		}

		OGRFieldDefn oField2("PolygonID", OFTInteger64);
		//oField1.SetPrecision(3);
		if (poLayer->CreateField(&oField2) != OGRERR_NONE)
		{
			std::cout << "Creating PolygonID field failed." << std::endl;
			return (-1);
		}

		//OGRMultiLineString  polylines;
		for (int i = 0; i < lineSegs.size(); i++)
		{
			OGRFeature *poFeature;
			poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
			poFeature->SetField(1, i);

			OGRLineString line;

			line.addPoint(lineSegs[i].sp[0], lineSegs[i].sp[1], 50);
			line.addPoint(lineSegs[i].ep[0], lineSegs[i].ep[1], 50);

			poFeature->SetGeometry(&line);

			if (poLayer->CreateFeature(poFeature) != OGRERR_NONE)
			{
				std::cout << "Failed to create feature: lines" << std::endl;
			}
			OGRFeature::DestroyFeature(poFeature);
		}
		
		GDALClose(poDS);

		return (0);
	}

	template <typename PointT> void
		BoundaryOptimization<PointT>::initialize(const std::string input_folder, const std::string output_folder)
	{
		input_dir_ = input_folder;
		output_dir_ = output_folder;

		release();
		create_folder(output_dir_);

		scene_bbox_.min_pt[0] = scene_bbox_.min_pt[1] = scene_bbox_.min_pt[2] = std::numeric_limits<double>::max();
		scene_bbox_.max_pt[0] = scene_bbox_.max_pt[1] = scene_bbox_.max_pt[2] = std::numeric_limits<double>::lowest();

		std::vector <std::string> subFolders;
		scan_folders(input_folder, subFolders);
		for (int i = 0; i < subFolders.size(); ++i)
		{
			BuildingCombo bc;
			bc.bbox.min_pt[0] = bc.bbox.min_pt[1] = bc.bbox.min_pt[2] = std::numeric_limits<double>::max();
			bc.bbox.max_pt[0] = bc.bbox.max_pt[1] = bc.bbox.max_pt[2] = std::numeric_limits<double>::lowest();

			std::string pure_name, suffix_name;
			ipl::extract_pure_file_name(subFolders[i], pure_name, suffix_name);
			bc.building_points_filename = input_dir_ + "/" + pure_name + ".pcd";
	
			loadPointSegments(subFolders[i], bc);
			loadPolygons(subFolders[i], bc);

			building_combos_.push_back(bc);

			if (scene_bbox_.min_pt[0] > bc.bbox.min_pt[0])
				scene_bbox_.min_pt[0] = bc.bbox.min_pt[0];
			if (scene_bbox_.min_pt[1] > bc.bbox.min_pt[1])
				scene_bbox_.min_pt[1] = bc.bbox.min_pt[1];
			if (scene_bbox_.min_pt[2] > bc.bbox.min_pt[2])
				scene_bbox_.min_pt[2] = bc.bbox.min_pt[2];
			if (scene_bbox_.max_pt[0] < bc.bbox.max_pt[0])
				scene_bbox_.max_pt[0] = bc.bbox.max_pt[0];
			if (scene_bbox_.max_pt[1] < bc.bbox.max_pt[1])
				scene_bbox_.max_pt[1] = bc.bbox.max_pt[1];
			if (scene_bbox_.max_pt[2] < bc.bbox.max_pt[2])
				scene_bbox_.max_pt[2] = bc.bbox.max_pt[2];
		}

		/*
		//for test
		////////////////////////////////////////////////////////////
		for (int i = 0; i < wall_segments_.size(); ++i)
		{//output all wall segs
		char buf[32];
		sprintf(buf, "/wall_seg_%04d.pcd", i);

		std::string out_name = output_dir_ + buf;
		write_PointCloud(out_name, *(wall_segments_[i].getInputCloud()), *(wall_segments_[i].getIndices()), true);
		}

		for (int i = 0; i < roof_segments_.size(); ++i)
		{//output all roof segs
		char buf[32];
		sprintf(buf, "/roof_seg_%04d.pcd", i);

		std::string out_name = output_dir_ + buf;
		write_PointCloud(out_name, *(roof_segments_[i].getInputCloud()), *(roof_segments_[i].getIndices()), true);
		}

		//output all polygons
		int polyNum = aShapes_.size();
		std::vector<float> fHeis, rHeis;
		fHeis.resize(polyNum, 0);
		rHeis.resize(polyNum, 0);
		std::string out_name = output_dir_ + "/all_polygons.shp";
		exportPolygons(out_name, aShapes_, rHeis, fHeis);
		*/

#ifdef OUTPUT_DEBUG_INFO
// 		std::vector<OGRPolygon> Polys;
// 		std::vector<float> rHeis, fHeis;
// 		for (int iB = 0; iB < building_combos_.size(); ++iB)
// 		{
// 			OGRLinearRing ring;
// 			ring.addPoint(building_combos_[iB].bbox.min_pt[0], building_combos_[iB].bbox.min_pt[1], 0);
// 			ring.addPoint(building_combos_[iB].bbox.max_pt[0], building_combos_[iB].bbox.min_pt[1], 0);
// 			ring.addPoint(building_combos_[iB].bbox.max_pt[0], building_combos_[iB].bbox.max_pt[1], 0);
// 			ring.addPoint(building_combos_[iB].bbox.min_pt[0], building_combos_[iB].bbox.max_pt[1], 0);
// 
// 			ring.closeRings();//首尾点重合形成闭合环 
// 			OGRPolygon poly;
// 			poly.addRing(&ring);
// 
// 			Polys.push_back(poly);
// 			rHeis.push_back(building_combos_[iB].bbox.max_pt[2]);
// 			fHeis.push_back(building_combos_[iB].bbox.min_pt[2]);
// 		}
// 		std::string filename = output_dir_ + "/buildingBBox.shp";
// 
// 		exportPolygons(filename, Polys, rHeis, fHeis);
		
#endif // OUTPUT_DEBUG_INFO


		extractProjectedLinesFromPlanes();

	}

	template <typename PointT> int
		BoundaryOptimization<PointT>::extractProjectedLinesFromPlanes()
	{
		projected_lines_.clear();

		for (int i = 0; i < wall_segments_.size(); ++i)
		{
			iplModelCoeffs coef;
			wall_segments_[i].getProjectedLineCeof(&coef);
			projected_lines_.push_back(coef);
		}

		return 0;
	}

	template <typename PointT> int
		BoundaryOptimization<PointT>::createLayout2D(float bufSize)
	{
		//layout2D的范围(加了buffer)  >  scene_bbox_(实际的数据范围)
		scene_bbox_.min_pt[0] -= bufSize; scene_bbox_.min_pt[1] -= bufSize; scene_bbox_.min_pt[2] -= bufSize;
		scene_bbox_.max_pt[0] += bufSize; scene_bbox_.max_pt[1] += bufSize; scene_bbox_.max_pt[2] += bufSize;

		LA_.reset(new LineArrangement);

		//1. create layout2D
		std::cout << "create Layout 2D..." << std::endl;
		int buildingNum = building_combos_.size();
		for (int i = 0; i < buildingNum; ++i)
		{
			std::vector<geometry::LineSeg2D> BuSegs; //for test 每个building的lineSegs

			std::cout << "add building " << i << "/" << buildingNum << "\r";
			BuildingCombo bc = building_combos_[i];

			geoModel::BoundingBox  bound = bc.bbox;
			bound.min_pt[0] -= bufSize; bound.min_pt[1] -= bufSize; bound.min_pt[2] -= bufSize;
			bound.max_pt[0] += bufSize; bound.max_pt[1] += bufSize; bound.max_pt[2] += bufSize;
			//1. polygons
			for (int iPoly = 0; iPoly < bc.poly_Indices.size(); ++iPoly)
			{
				int polyID = bc.poly_Indices[iPoly];
				std::vector<geometry::LineSeg2D> lineSegs;

				addPolygonToArrangment(aShapes_[polyID], bound, lineSegs);

				BuSegs.insert(BuSegs.end(), lineSegs.begin(), lineSegs.end());

				LA_->insertLines(lineSegs);
			}

			//2. projected lines
			std::vector<geometry::LineSeg2D> lineSegs;
			for (int iWall = 0; iWall < bc.wall_Indices.size(); ++iWall)
			{
				int wallID = bc.wall_Indices[iWall];

				geometry::LineSeg2D lSeg;
				getLineSeg(/*wall_segments_[wallID].getModelCoef().coef*/projected_lines_[wallID], bound, lSeg);
				lineSegs.push_back(lSeg);
			}

			BuSegs.insert(BuSegs.end(), lineSegs.begin(), lineSegs.end());

			LA_->insertLines(lineSegs);

#ifdef OUTPUT_DEBUG_INFO
//			if (i == 2)
//			{
				std::string filename = output_dir_ + "/wall_lineSegs.shp";
				exportLineSegments(filename, BuSegs);

				filename = output_dir_ + "/ashapes.shp";
				std::vector<float> rHeis, fHeis;
				rHeis.resize(aShapes_.size(), 0);
				fHeis.resize(aShapes_.size(), 0);
				exportPolygons(filename, aShapes_, rHeis, fHeis);
//			}

#endif // OUTPUT_DEBUG_INFO

		}
		std::cout << "add building " << buildingNum << "/" << buildingNum << std::endl;

		arr2_ = LA_->getArrangment();

		////////////////////////////////////
		//clean arrangement
		//(1) 多个局部LineArrangement合并时会出现antenna
		//(2) duplicate vertex in arrangement  (原因未知, 暂不处理)
		std::set<LineArrangement::Halfedge_handle> removal_edge_list;
		LineArrangement::Edge_iterator eit;
		int nOutter_ant = 0, nInner_ant = 0;
		std::cout << "clean antennae ... " << std::endl;
		for (eit = arr2_->edges_begin(); eit != arr2_->edges_end(); ++eit)
		{
			LineArrangement::Halfedge_handle he = eit;

			if (he->face()->is_unbounded() && he->twin()->face()->is_unbounded())
			{//antenna
				removal_edge_list.insert(he);
				nOutter_ant++;
				std::cout << "find outter/inner antenna: " << nOutter_ant << "/" << nInner_ant << "\r";
				continue;
			}

// 			int fid_i = he->face()->data();
// 			int fid_j = he->twin()->face()->data();

			if (he->face() == he->twin()->face())
			{//内部antenna

				removal_edge_list.insert(he);
				nInner_ant++;
				std::cout << "find outter/inner antenna: " << nOutter_ant << "/" << nInner_ant << "\r";
				continue;
			}
		}

		std::cout << "find outter/inner antenna: " << nOutter_ant << "/" << nInner_ant << std::endl;

//		removeEdges(arr2_, &removal_edge_list);
		std::set<LineArrangement::Halfedge_handle>::iterator iter;
		for (iter = removal_edge_list.begin(); iter != removal_edge_list.end(); ++iter)
		{
			LineArrangement::Halfedge_handle he = *iter;
			arr2_->remove_edge(he);
		}
		removal_edge_list.clear();

// 		nvertices = arr2_->number_of_vertices();
// 		nhalfedges = arr2_->number_of_halfedges();
// 		nfaces = arr2_->number_of_faces();

		init_nvertices_ = arr2_->number_of_vertices();
		init_nhalfedges_ = arr2_->number_of_halfedges();
		init_nfaces_ = arr2_->number_of_faces();

		// 	int nvertices = arr2_->number_of_vertices();
		// 	int nvertices_inf = arr2_->number_of_vertices_at_infinity();

		//2. 为vertex, edge, facet添加索引号
		//计算facet和edge的LayoutAtt
		int nvertices = arr2_->number_of_vertices();
		int nvertices_inf = arr2_->number_of_vertices_at_infinity();
		nvertices -= nvertices_inf;

		int idx;
		////////////////////vertices///////////////////////////////////////////
		points_.resize(arr2_->number_of_vertices());
		LineArrangement::Vertex_handle vit;
		std::cout << arr2_->number_of_vertices() << " vertices with " << arr2_->number_of_vertices_at_infinity()
			<< " infinity points." << std::endl;
		for (vit = arr2_->vertices_begin(), idx = 0; vit != arr2_->vertices_end(); ++vit)
		{
			vit->set_data(idx);

			double x, y, z;
			x = CGAL::to_double(vit->point().x())/*.to_double()*/;
			y = CGAL::to_double(vit->point().y())/*.to_double()*/;
			z = 0;
			LineArrangement::Point_3 p(x, y, z);
			points_[idx] = p;

			idx++;
		}

		////////////////////////////edge/////////////////////////////////////////////
		int nhalfedges = arr2_->number_of_halfedges();

		//halfedge_att_list_.resize(nhalfedges, att);
		LineArrangement::Halfedge_iterator heit;
		std::cout << nhalfedges << " halfedges with " << arr2_->number_of_curves() << " curves." << std::endl;
		for (heit = arr2_->halfedges_begin(), idx = 0; heit != arr2_->halfedges_end(); ++heit, ++idx)
		{
			int eid = heit->data();

			heit->set_data(idx);
			//eit->twin()->set_data(idx);
		}

		/////////////////////////////face/////////////////////////////////////
		int nfaces = arr2_->number_of_faces();
		int nfaces_unbounded = arr2_->number_of_unbounded_faces();
		//nfaces -= nfaces_unbounded;

		//face_att_list_.resize(nfaces, att);

		//facet_area_list_.resize(nfaces, 0);
		//face_halfedge_list_.resize(nfaces);

		// Print the arrangement faces.
		LineArrangement::Face_iterator fit;
		std::cout << arr2_->number_of_faces() << " faces with " << arr2_->number_of_unbounded_faces() << " unbounded faces." << std::endl;
		for (fit = arr2_->faces_begin(), idx = 1; fit != arr2_->faces_end(); ++fit)
		{
			if (!fit->is_unbounded())
			{
				fit->set_data(idx);
				++idx;
			}
			else
				fit->set_data(0); //unbounded face labels 0

#ifdef OUTPUT_DEBUG_INFO
// 			if (!fit->is_unbounded())
// 			{
// 				int idx = fit->data();
// 				GridIntersection::Polygon_2 poly;
// 
// 				LineArrangement::Arrangement_2::Ccb_halfedge_circulator  circ = fit->outer_ccb();
// 				LineArrangement::Arrangement_2::Ccb_halfedge_circulator  curr = circ;
// 
// 				do
// 				{
// 					LineArrangement::Arrangement_2::Halfedge_handle he = curr;
// 
// 					int eid = he->data();
// 					int vtid = he->target()->data();
// 					int vsid = he->source()->data();
// 
// 					GridIntersection::Point_2 p(he->target()->point().x()/*.to_double()*/, he->target()->point().y()/*.to_double()*/);
// 
// 					poly.push_back(p);
// 
// 					curr = curr->next();
// 
// 				} while (curr != circ);
// 			}

#endif // OUTPUT_DEBUG_INFO

		}

#ifdef OUTPUT_DEBUG_INFO
		{
			std::string filename = output_dir_ + "/init_polylines.shp";
			exportLineSegments(filename, arr2_);
		}
#endif

		face_flags_.clear();
		face_flags_.resize(nfaces, false);  //inner: true;  outer: false 

		return 0;
	}

	template <typename PointT> int
		BoundaryOptimization<PointT>::addPolygonToArrangment(OGRPolygon &poly, geoModel::BoundingBox bbox,
			std::vector<geometry::LineSeg2D> &lineSegs)
	{
		lineSegs.clear();

		// 	OGREnvelope envelop;
		// 	poly.getEnvelope(&envelop);
		// 
		// 	geoModel::BoundingBox bbox;
		// 	bbox.min_pt[0] = envelop.MinX;
		// 	bbox.min_pt[1] = envelop.MinY;
		// 	bbox.max_pt[0] = envelop.MaxX;
		// 	bbox.max_pt[1] = envelop.MaxY;

		OGRLinearRing *ring = poly.getExteriorRing();
		int ptNum = ring->getNumPoints();
		if (ptNum < 3)
		{
			std::cout << "point number is less than 3" << std::endl;
			return (-1);
		}

		for (int i = 0; i < ptNum - 1; i++)
		{
			double sp[2], ep[2];
			OGRPoint pt;
			ring->getPoint(i, &pt);
			sp[0] = pt.getX();
			sp[1] = pt.getY();

			ring->getPoint(i + 1, &pt);
			ep[0] = pt.getX();
			ep[1] = pt.getY();

			geometry::LineSeg2D seg;

// 			seg.ep[0] = ep[0]; seg.ep[1] = ep[1];
// 			seg.sp[0] = sp[0]; seg.sp[1] = sp[1];
 			getLineSeg(sp, ep, bbox, seg);
			lineSegs.push_back(seg);
		}

		for (int ir = 0; ir < poly.getNumInteriorRings(); ++ir)
		{
			OGRLinearRing *ring = poly.getInteriorRing(ir);
			for (int i = 0; i < ptNum - 1; i++)
			{
				double sp[2], ep[2];
				OGRPoint pt;
				ring->getPoint(i, &pt);
				sp[0] = pt.getX();
				sp[1] = pt.getY();

				ring->getPoint(i + 1, &pt);
				ep[0] = pt.getX();
				ep[1] = pt.getY();

				geometry::LineSeg2D seg;

				getLineSeg(sp, ep, bbox, seg);
				lineSegs.push_back(seg);
			}
		}

		return 0;
	}

	template <typename PointT> int
		BoundaryOptimization<PointT>::getLineSeg(const double sp[2], const double ep[2], const geoModel::BoundingBox &bbox, geometry::LineSeg2D &lineSeg)
	{
		iplModelCoeffs mCoef;

		mCoef.values.resize(4, 0);

		double dx = ep[0] - sp[0];
		double dy = ep[1] - sp[1];

		mCoef.values[0] = dy;
		mCoef.values[1] = -dx;
		mCoef.values[3] = dx*sp[1] - dy*sp[0];

		return getLineSeg(mCoef, bbox, lineSeg);
	}

	template <typename PointT> int
		BoundaryOptimization<PointT>::getLineSeg(const iplModelCoeffs &mCoef, const geoModel::BoundingBox &bbox, geometry::LineSeg2D &lineSeg)
	{
		//double x, y;
		bool bHorizontal = false, bVertical = false;

		if (mCoef.values[0] == 0)
		{//水平线
			bHorizontal = true;
			lineSeg.sp[0] = bbox.min_pt[0];
			lineSeg.sp[1] = -mCoef.values[3] / mCoef.values[1];

			lineSeg.ep[0] = bbox.max_pt[0];
			lineSeg.ep[1] = -mCoef.values[3] / mCoef.values[1];
		}
		else if (mCoef.values[1] == 0)
		{//垂直线
			bVertical = true;
			lineSeg.sp[0] = -mCoef.values[3] / mCoef.values[0];
			lineSeg.sp[1] = bbox.min_pt[1];

			lineSeg.ep[0] = -mCoef.values[3] / mCoef.values[0];
			lineSeg.ep[1] = bbox.max_pt[1];
		}
		else
		{
			if (fabs(mCoef.values[0] / mCoef.values[1]) > 1)
			{
				lineSeg.sp[0] = -(mCoef.values[1] * bbox.min_pt[1] + mCoef.values[3]) / mCoef.values[0];
				lineSeg.sp[1] = bbox.min_pt[1];

				lineSeg.ep[0] = -(mCoef.values[1] * bbox.max_pt[1] + mCoef.values[3]) / mCoef.values[0];
				lineSeg.ep[1] = bbox.max_pt[1];
			}
			else
			{
				lineSeg.sp[0] = bbox.min_pt[0];
				lineSeg.sp[1] = -(mCoef.values[0] * bbox.min_pt[0] + mCoef.values[3]) / mCoef.values[1];

				lineSeg.ep[0] = bbox.max_pt[0];
				lineSeg.ep[1] = -(mCoef.values[0] * bbox.max_pt[0] + mCoef.values[3]) / mCoef.values[1];
			}
		}

		double x1, y1, x2, y2;
		double x, y;
		if (!bHorizontal && !bVertical)
		{
			//flags: left, right, top, bottom
			bool bp1_xL, bp1_xR, bp1_yT, bp1_yB;
			bool bp2_xL, bp2_xR, bp2_yT, bp2_yB;

			bp1_xL = bp1_xR = bp1_yT = bp1_yB = false;
			bp2_xL = bp2_xR = bp2_yT = bp2_yB = false;

			x1 = lineSeg.sp[0];
			y1 = lineSeg.sp[1];
			x2 = lineSeg.ep[0];
			y2 = lineSeg.ep[1];

			if (x1 < bbox.min_pt[0])
				bp1_xL = true;
			if (x1 > bbox.max_pt[0])
				bp1_xR = true;
			if (y1 < bbox.min_pt[1])
				bp1_yB = true;
			if (y1 > bbox.max_pt[1])
				bp1_yT = true;

			if (x2 < bbox.min_pt[0])
				bp2_xL = true;
			if (x2 > bbox.max_pt[0])
				bp2_xR = true;
			if (y2 < bbox.min_pt[1])
				bp2_yB = true;
			if (y2 > bbox.max_pt[1])
				bp2_yT = true;

			if (bp1_xL || bp1_xR || bp1_yB || bp1_yT
				|| bp2_xL || bp2_xR || bp2_yB || bp2_yT)
			{
				if (bp1_xL)
				{
					x = bbox.min_pt[0];
					y = -(mCoef.values[0] * x + mCoef.values[3]) / mCoef.values[1];
					//p1 = Point_2(x, y);
					lineSeg.sp[0] = x;
					lineSeg.sp[1] = y;
				}

				if (bp1_xR)
				{
					x = bbox.max_pt[0];
					y = -(mCoef.values[0] * x + mCoef.values[3]) / mCoef.values[1];
					//p1 = Point_2(x, y);
					lineSeg.sp[0] = x;
					lineSeg.sp[1] = y;
				}

				if (bp2_xL)
				{
					x = bbox.min_pt[0];
					y = -(mCoef.values[0] * x + mCoef.values[3]) / mCoef.values[1];
					//p2 = Point_2(x, y);
					lineSeg.ep[0] = x;
					lineSeg.ep[1] = y;
				}

				if (bp2_xR)
				{
					x = bbox.max_pt[0];
					y = -(mCoef.values[0] * x + mCoef.values[3]) / mCoef.values[1];
					//p2 = Point_2(x, y);
					lineSeg.ep[0] = x;
					lineSeg.ep[1] = y;
				}

				if (bp1_yB)
				{
					y = bbox.min_pt[1];
					x = -(mCoef.values[1] * y + mCoef.values[3]) / mCoef.values[0];
					//p1 = Point_2(x, y);
					lineSeg.sp[0] = x;
					lineSeg.sp[1] = y;
				}

				if (bp1_yT)
				{
					y = bbox.max_pt[1];
					x = -(mCoef.values[1] * y + mCoef.values[3]) / mCoef.values[0];
					//p1 = Point_2(x, y);
					lineSeg.sp[0] = x;
					lineSeg.sp[1] = y;
				}

				if (bp2_yB)
				{
					y = bbox.min_pt[1];
					x = -(mCoef.values[1] * y + mCoef.values[3]) / mCoef.values[0];
					//p2 = Point_2(x, y);
					lineSeg.ep[0] = x;
					lineSeg.ep[1] = y;
				}

				if (bp2_yT)
				{
					y = bbox.max_pt[1];
					x = -(mCoef.values[1] * y + mCoef.values[3]) / mCoef.values[0];
					//p2 = Point_2(x, y);
					lineSeg.ep[0] = x;
					lineSeg.ep[1] = y;
				}
			}
		}

		return 0;
	}

	template <typename PointT> void 
		BoundaryOptimization<PointT>::getWallKeyMap(CellKeyMap *keymap)
	{
		PointPartitionQuadtree<PointT> quadtree_partition;

		std::cout << "create quadtree partition for wall points " << wall_cloud_->size() << " ...";
		quadtree_partition.setInputCloud(wall_cloud_);
		quadtree_partition.setBBox(scene_bbox_.min_pt, scene_bbox_.max_pt);
		quadtree_partition.apply(wall_gridsize_);
		//	cell_list_wall_ = *(quadtree_partition.getCellList());
		*keymap = *(quadtree_partition.getCellKeyMap());
		std::cout << " done!" << std::endl;

		mean_pts_num_wall_ = quadtree_partition.getMeanPtsNumofCell();
		std::cout << "mean pts number in a wall cell is " << mean_pts_num_wall_ << std::endl;

#ifdef OUTPUT_DEBUG_INFO
		std::string filename = output_dir_ + "/wall_pts.pcd";
		write_PointCloud(filename, *wall_cloud_, true);
#endif

#ifdef OUTPUT_DEBUG_INFO
		{
			std::vector<OGRPolygon> Polys;
			std::vector<std::pair<QuadtreeKey, VoxelContainerPointIndices> >  keyCells;
			CellKeyMap::iterator it_km = keymap->begin();
			for (; it_km != keymap->end(); ++it_km)
			{
				QuadtreeKey key = it_km->first;
				iplRECT<double> rect;
				quadtree_partition.getRect(key, rect);

				OGRLinearRing ring;

				ring.addPoint(rect.m_xmin, rect.m_ymin, 0);
				ring.addPoint(rect.m_xmax, rect.m_ymin, 0);
				ring.addPoint(rect.m_xmax, rect.m_ymax, 0);
				ring.addPoint(rect.m_xmin, rect.m_ymax, 0);

				ring.closeRings();//首尾点重合形成闭合环 
				OGRPolygon poly;
				poly.addRing(&ring);

				keyCells.push_back(*it_km);
				Polys.push_back(poly);
			}

			std::string filename = output_dir_ + "/wallkeymap.shp";
			exportKeyMap(filename, Polys, keyCells);
		}

#endif
	}

	template <typename PointT> void
		BoundaryOptimization<PointT>::getRoofKeyMap(CellKeyMap *keymap, int RoofFlag)
	{
		PointPartitionQuadtree<PointT> quadtree_partition;

		if (RoofFlag == 0)
		{
			ref_ptr<iplPointCloud<PointT> > buildings_cloud(new iplPointCloud<PointT>);
			std::cout << "loading building point cloud ... " << std::endl;
			for (int i = 0; i < building_combos_.size(); ++i)
			{
				ref_ptr<iplPointCloud<PointT> > icloud(new iplPointCloud<PointT>);

				if (read_PointCloud(building_combos_[i].building_points_filename, *icloud) != 0)
				{
					std::cout << "can not open: " << building_combos_[i].building_points_filename << std::endl;
					continue;
				}

				std::cout << "loading: " << building_combos_[i].building_points_filename << "points number: " << icloud->size()
					<< std::endl;


				for (int j = 0; j < icloud->size(); j++)
				{
					if(icloud->points[j].z > building_combos_[i].zProfile)
						buildings_cloud->points.push_back(icloud->points[j]);
				}
			}
			//std::cout << std::endl;
			std::cout << "loading building point cloud ... done! " << std::endl;

			buildings_cloud->height = 1;
			buildings_cloud->width = buildings_cloud->size();

			std::cout << "create quadtree partition for profile points " << buildings_cloud->size() << " ...";
			quadtree_partition.setInputCloud(buildings_cloud);
			quadtree_partition.setBBox(scene_bbox_.min_pt, scene_bbox_.max_pt);
			quadtree_partition.apply(roof_gridsize_);

#ifdef OUTPUT_DEBUG_INFO
			{
				std::string filename = output_dir_ + "/roof_pts.pcd";
				write_PointCloud(filename, *buildings_cloud, true);
			}
#endif
		}
		else if (RoofFlag == 1)
		{
			std::cout << "create quadtree partition for roof points " << roof_cloud_->size() << " ...";
			quadtree_partition.setInputCloud(roof_cloud_);
			quadtree_partition.setBBox(scene_bbox_.min_pt, scene_bbox_.max_pt);
			quadtree_partition.apply(roof_gridsize_);

#ifdef OUTPUT_DEBUG_INFO
			{
				std::string filename = output_dir_ + "/roof_pts.pcd";
				write_PointCloud(filename, *roof_cloud_, true);
			}
#endif
		}
		//	cell_list_nonwall_ = *(quadtree_partition.getCellList());

		*keymap = *(quadtree_partition.getCellKeyMap());
		std::cout << " done!" << std::endl;

		mean_pts_num_roof_ = quadtree_partition.getMeanPtsNumofCell();
		std::cout << "mean pts number in a roof cell is " << mean_pts_num_roof_ << std::endl;

#ifdef OUTPUT_DEBUG_INFO
		{
			std::vector<OGRPolygon> Polys;
			std::vector<std::pair<QuadtreeKey, VoxelContainerPointIndices> >  keyCells;
			CellKeyMap::iterator it_km = keymap->begin();
			for ( ; it_km != keymap->end(); ++it_km)
			{
				QuadtreeKey key = it_km->first;
				iplRECT<double> rect;
				quadtree_partition.getRect(key, rect);

				OGRLinearRing ring;

				ring.addPoint(rect.m_xmin, rect.m_ymin, 0);
				ring.addPoint(rect.m_xmax, rect.m_ymin, 0);
				ring.addPoint(rect.m_xmax, rect.m_ymax, 0);
				ring.addPoint(rect.m_xmin, rect.m_ymax, 0);

				ring.closeRings();//首尾点重合形成闭合环 
				OGRPolygon poly;
				poly.addRing(&ring);

				keyCells.push_back(*it_km);
				Polys.push_back(poly);
			}

			std::string filename = output_dir_ + "/roofkeymap.shp";
			exportKeyMap(filename, Polys, keyCells);
		}

#endif

	}


	template <typename PointT> int
		BoundaryOptimization<PointT>::computeAttributesForArrangement(const float wall_Gridsize, const float roof_Gridsize)
	{
		std::cout << "arrangement information: " << std::endl;
		wall_gridsize_ = wall_Gridsize;
		roof_gridsize_ = roof_Gridsize;

		///////////////////////////////////////////////////////////////////////////////////////
		//2. 计算face, edge属性值
		//2.1 构造点云格网，进行数据统计

		getWallKeyMap(&keymap_wall_);
		getRoofKeyMap(&keymap_roof_, 0);


		int   vNumX = static_cast<int>((scene_bbox_.max_pt[0] - scene_bbox_.min_pt[0]) / roof_gridsize_) + 1;
		int   vNumY = static_cast<int>((scene_bbox_.max_pt[1] - scene_bbox_.min_pt[1]) / roof_gridsize_) + 1;
		int cell_num = vNumX*vNumY;
		std::cout << "roof grid: " << cell_num << std::endl;
		std::cout << "calculate features for each face..." << std::endl;

		LineArrangement::Vertex_handle vit;
		LineArrangement::Halfedge_iterator heit;
		LineArrangement::Face_iterator fit;

		int nvertices = arr2_->number_of_vertices();
		int nvertices_inf = arr2_->number_of_vertices_at_infinity();
		nvertices -= nvertices_inf;

		int nhalfedges = arr2_->number_of_halfedges();
		int nfaces = arr2_->number_of_faces();
		int nfaces_unbounded = arr2_->number_of_unbounded_faces();

		face_att_list_.clear();
		halfedge_att_list_.clear();

// #ifdef OUTPUT_DEBUG_INFO
// 		
// 		edgeAttList.resize(nhalfedges);
// #endif
		
		LayoutFaceAtt f_att;
// 		f_att.occupied = 0;
// 		f_att.empty = 0;
 		for (int i = 0; i < nfaces; i++)
		{
			face_att_list_.insert(std::make_pair(i, f_att));
		}
		face_att_vec_.resize(nfaces);

		LayoutEdgeAtt e_att;
// 		e_att.occupied = 0;
// 		e_att.empty = 0;
		for (int i = 0; i < nhalfedges; i++)
		{
			halfedge_att_list_.insert(std::make_pair(i, e_att));
		}
		edge_att_vec_.resize(nhalfedges);

		//遍历arrangement中的face, 利用GridIntersection计算落入face中的grid数量
		GridIntersection grid_intersect;
		grid_intersect.InitialGrid(scene_bbox_.min_pt, scene_bbox_.max_pt, roof_gridsize_, roof_gridsize_);

#ifdef HAVE_OpenMP
#pragma omp parallel private(fit)
		{
#endif // HAVE_OpenMP
			for (fit = arr2_->faces_begin(); fit != arr2_->faces_end(); ++fit)
			{
#ifdef HAVE_OpenMP
#pragma omp single nowait
				{
#endif // HAVE_OpenMP

					int idx = fit->data();
					std::cout << "processing: " << idx << "/" << nfaces << "\r";
					if (!fit->is_unbounded())
					{
						VertexHandleList polygons;
						repairPolygon(arr2_, fit, polygons);

						GridIntersection::Polygon_2 poly;
						for (VertexHandleList::iterator vh_it = polygons.begin(); vh_it != polygons.end(); ++vh_it)
						{
							GridIntersection::Point_2 p((*vh_it)->point().x()/*.to_double()*/, (*vh_it)->point().y()/*.to_double()*/);
							poly.push_back(p);
						}

						double f_area = CGAL::to_double(poly.area());
						face_att_list_[idx].area = f_area;
// 						LineArrangement::Arrangement_2::Ccb_halfedge_circulator  circ = fit->outer_ccb();
// 						LineArrangement::Arrangement_2::Ccb_halfedge_circulator  curr = circ;
// 
// 						do
// 						{
// 							LineArrangement::Arrangement_2::Halfedge_handle he = curr;
// 
// 							GridIntersection::Point_2 p(he->target()->point().x()/*.to_double()*/, he->target()->point().y()/*.to_double()*/);
// 
// 							poly.push_back(p);
// 
// 							curr = curr->next();
// 
// 						} while (curr != circ);

						CellKeyMap intersected_grids;
						IntersectionAreaMap overlapMap;
						//grid_intersect.getIntersectedGrids(poly, intersected_grids); //扫描线求交
						grid_intersect.getIntersectedGridsAndOverlappingArea(poly, intersected_grids, overlapMap);

						if (intersected_grids.size() == 0)
						{//polygon 不在数据范围内
// 							face_att_list_[idx].empty = 1;
// 							face_att_list_[idx].occupied = 0;
							face_att_list_[idx].sparsity = 0;

							face_att_vec_[idx] = face_att_list_[idx];
						}
						else
						{//polygon 与数据有overlap
							int gNum = 0; //overlaping grid number
							float sum_occupied_area_w = 0;
							for (CellKeyMap::iterator it_intersected = intersected_grids.begin();
								it_intersected != intersected_grids.end();
								++it_intersected)
							{
								gNum++; //网格总数

								QuadtreeKey key_arg = it_intersected->first;

								CellKeyMap::iterator it_vm;
								it_vm = keymap_roof_.find(key_arg);
								if (it_vm != keymap_roof_.end())
								{//存在非墙面点
								//	occupied = true;
								//	face_att_list_[idx].occupied++;
									float wi = it_vm->second.getSize();
									wi = wi / mean_pts_num_roof_;

									if (wi > 1.0)
										wi = 1.0;
									
									double o_area = overlapMap[key_arg];
									sum_occupied_area_w += wi*o_area;
								}
// 								else
// 								{
// 									//				occupied = false;
// 									face_att_list_[idx].empty++;
// 								}
							}
							face_att_list_[idx].sparsity = sum_occupied_area_w / face_att_list_[idx].area;

							face_att_vec_[idx] = face_att_list_[idx];
						}
					}
					else
					{//unbounded face
// 						face_att_list_[idx].empty = 1;
// 						face_att_list_[idx].occupied = 0;
						face_att_list_[idx].sparsity = 0;
						face_att_list_[idx].area = DBL_MAX;

						face_att_vec_[idx] = face_att_list_[idx];
					}
#ifdef HAVE_OpenMP
				}
#endif
			}
#if HAVE_OpenMP
		}
#endif
		std::cout << "processing: " << nfaces << "/" << nfaces << std::endl;

		//利用grid_intersection计算edge的LayoutAtt
		//GridIntersection grid_intersect;
		grid_intersect.InitialGrid(scene_bbox_.min_pt, scene_bbox_.max_pt, wall_gridsize_, wall_gridsize_);

		geometry::LineSeg2D seg;
		std::vector<bool> halfedge_traverse_flag;

		std::cout << "calculate features for each edge..." << std::endl;
		halfedge_traverse_flag.resize(init_nhalfedges_, false);

		int idx;
		for (heit = arr2_->halfedges_begin(); heit != arr2_->halfedges_end(); ++heit)
		{
			idx = heit->data();
			std::cout << "processing: " << idx << "/" << nhalfedges << "\r";

			if (halfedge_traverse_flag[idx])
				continue;

			int twin_idx = heit->twin()->data();

			halfedge_traverse_flag[idx] = halfedge_traverse_flag[twin_idx] = true;

			double squared_dis = CGAL::to_double(CGAL::squared_distance(heit->source()->point(), heit->target()->point()));
			halfedge_att_list_[idx].length = sqrt(squared_dis);
			halfedge_att_list_[twin_idx].length = halfedge_att_list_[idx].length;

			LineArrangement::Point_2 p = heit->source()->point();
			seg.sp[0] = CGAL::to_double(p.x())/*.to_double()*/;
			seg.sp[1] = CGAL::to_double(p.y())/*.to_double()*/;

			p = heit->target()->point();
			seg.ep[0] = CGAL::to_double(p.x())/*.to_double()*/;
			seg.ep[1] = CGAL::to_double(p.y())/*.to_double()*/;

			CellKeyMap intersected_grids;
			LineSectionKeyMap lineSections;
			grid_intersect.getIntersectedGridsAndLineSections(seg, intersected_grids, lineSections); //扫描线求交
			if (intersected_grids.size() == 0)
			{
// 				halfedge_att_list_[idx].empty=1;
// 				halfedge_att_list_[idx].occupied = 0;

				halfedge_att_list_[idx].sparsity = 0;
				halfedge_att_list_[twin_idx] = halfedge_att_list_[idx];

				edge_att_vec_[idx] = halfedge_att_list_[idx];
				edge_att_vec_[twin_idx] = halfedge_att_list_[twin_idx];
			}
			else
			{
				double sum_disW = 0;
				for (CellKeyMap::iterator it_intersected = intersected_grids.begin();
					it_intersected != intersected_grids.end();
					++it_intersected)
				{
					QuadtreeKey key_arg = it_intersected->first;

					CellKeyMap::iterator it_vm;
					it_vm = keymap_wall_.find(key_arg);
					if (it_vm != keymap_wall_.end())
					{//存在非墙面点
					 //				occupied = true;
						float wi = it_vm->second.getSize();
						wi = wi / mean_pts_num_wall_;

						if (wi > 1.0)
							wi = 1.0;
						
						ipl::geometry::LineSeg2D line = lineSections[key_arg];
						LineArrangement::Point_2 p1(line.sp[0], line.sp[1]);
						LineArrangement::Point_2 p2(line.ep[0], line.ep[1]);
						
						double squared_dis = CGAL::to_double(CGAL::squared_distance(p1, p2));
						sum_disW += sqrt(squared_dis)*wi;
					}
// 					else
// 					{
// 						//				occupied = false;
// 						halfedge_att_list_[idx].empty++;
// 						halfedge_att_list_[twin_idx].empty++;
// 					}
				}

				halfedge_att_list_[idx].sparsity = sum_disW / halfedge_att_list_[idx].length;
				halfedge_att_list_[twin_idx].sparsity = halfedge_att_list_[idx].sparsity;

				edge_att_vec_[idx] = halfedge_att_list_[idx];
				edge_att_vec_[twin_idx] = halfedge_att_list_[twin_idx];
			}
		}

// #ifdef OUTPUT_DEBUG_INFO
// 		for (int i = 0; i < nhalfedges; ++i)
// 		{
// 			edgeAttList[i] = halfedge_att_list_[i];
// 		}
// #endif

		std::cout << "processing: " << nhalfedges << "/" << nhalfedges << std::endl;

		return 0;
	}

	template <typename PointT> int
		BoundaryOptimization<PointT>::optimize(/*double inner_edge_Th_, double border_edge_Th_,*/ double polygon_area_Th_)
	{
		int nfaces = arr2_->number_of_faces();
		assert(init_nfaces_ == nfaces); //确定face编号没有改变

		//构造IndoorLayoutOptimization_GraphCut并求解
		IndoorLayoutOptimizationByGC  opt_gc;
		std::cout << "select a solver: GraphCut" << std::endl;

		opt_gc.beginBuildGraph(nfaces);
		LineArrangement::Face_iterator fit;
		int idx;

		std::vector<double> UnaryCosts;  //for debug, 调试maxflow中的错误
		UnaryCosts.resize(nfaces, 0);

		for (fit = arr2_->faces_begin(), idx = 0; fit != arr2_->faces_end(); ++fit)
		{
			double source_weight, sink_weight;
			if (!fit->is_unbounded())
			{
				idx = fit->data();
				double fv = face_att_list_[idx].sparsity;// *face_att_list_[idx].area;
					//face_att_list_[idx].occupied / (face_att_list_[idx].occupied + face_att_list_[idx].empty);
				source_weight = alpha_*exp(-fv);
				//source_weight = alpha_*fv;
				sink_weight = 1.0 - source_weight;
			}
			else
			{
				idx = fit->data();
				source_weight = alpha_;
				sink_weight = 1.0 - alpha_;
			}

			//		source_weight = 1;
			//		sink_weight = 0;
			opt_gc.add_UnaryCost(idx, source_weight, sink_weight);

			UnaryCosts[idx] = source_weight;
		}

		// 	std::vector<double>  edgeWList;
		// 	edgeWList.resize(nfaces*nfaces, -1);

		LineArrangement::Face_handle unb_face = arr2_->unbounded_face();
		std::vector<bool> halfedge_traverse_flag;
		int nhalfedges = arr2_->number_of_halfedges();
		halfedge_traverse_flag.assign(init_nhalfedges_, false);
		LineArrangement::Halfedge_iterator heit;

		//OVERLAPMap AdjFaceMap;  //面邻接表，是否存在两个面之间有多条边的情况？
		//face ID pair <int, int>; 公共边ids std::vector<int>
		typedef boost::unordered_map<std::pair<int, int>, std::vector<int> > FACEPAIREDGEMap;
		FACEPAIREDGEMap adjFaceEdgeMap;
		for (heit = arr2_->halfedges_begin(), idx = 0; heit != arr2_->halfedges_end(); ++heit, ++idx)
		{
			int eidx = heit->data();
			int twin_eidx = heit->twin()->data();
			int source_key, target_key;
			double edge_weight;

			if (halfedge_traverse_flag[eidx])
				continue;

			halfedge_traverse_flag[eidx] = halfedge_traverse_flag[twin_eidx] = true;

			//Halfedge_const_handle he = eit;
			// 		if ((heit->face() != unb_face) && (heit->twin()->face()) != unb_face)
			// 		{
			if (heit->face() == heit->twin()->face())
			{//antenna
				int fid = heit->face()->data();
				int tfid = heit->twin()->face()->data();
				std::cout << "find an antenna! halfedge " << heit->data() << " in face " << heit->face()->data() << std::endl;
				continue;
			}

			source_key = heit->face()->data();
			target_key = heit->twin()->face()->data();

			std::pair<int, int> comb_key;
			if (source_key < target_key)
				comb_key = std::make_pair(source_key, target_key);
			else
				comb_key = std::make_pair(target_key, source_key);


			FACEPAIREDGEMap::iterator it_find = adjFaceEdgeMap.find(comb_key);
			if (it_find != adjFaceEdgeMap.end())
			{
// 				it_find->second.first++;
// 				it_find->second.second += evw;
				it_find->second.push_back(eidx);
			}
			else
			{
// 				std::pair<int, double> val = std::make_pair(1, evw);
// 				adjFaceCostMap.insert(std::make_pair(comb_key, val));
				std::vector<int> edgeList;
				edgeList.push_back(eidx);
				adjFaceEdgeMap.insert(std::make_pair(comb_key, edgeList));
			}
		}


		struct BinaryCost {
			int sID, tID;
			double cost;
		} ;
		std::vector<BinaryCost>   dbcosts1, dbcosts2;
		FACEPAIREDGEMap::iterator it_FP;
		for(it_FP= adjFaceEdgeMap.begin(); it_FP!= adjFaceEdgeMap.end(); ++it_FP)
		{
			std::pair<int, int> comb_key = it_FP->first;
			int source_key = comb_key.first;
			int target_key = comb_key.second;

			std::vector<int> edgeList = it_FP->second;
			double sumLen = 0;
			double sumevw = 0;
			for (int i = 0; i < edgeList.size(); i++)
			{
				int eid = edgeList[i];
				double evw = halfedge_att_list_[eid].sparsity;
// 					halfedge_att_list_[eid].occupied /
// 					(halfedge_att_list_[eid].occupied + halfedge_att_list_[eid].empty);

				sumevw += evw*halfedge_att_list_[eid].length;
				sumLen += halfedge_att_list_[eid].length;
			}

			double mean_evw = sumevw/sumLen;
			double fv = face_att_list_[source_key].sparsity;
				//face_att_list_[source_key].occupied / (face_att_list_[source_key].occupied + face_att_list_[source_key].empty);
			double fw = face_att_list_[target_key].sparsity;
				//face_att_list_[target_key].occupied / (face_att_list_[target_key].occupied + face_att_list_[target_key].empty);

			double edge_weight = beta_*exp(-0.5*(mean_evw + fabs(fv - fw)));
			//double edge_weight = beta_*exp(0.5*(mean_evw + fabs(fv - fw)));
			//edge_weight = beta_*exp(-(evw * fabs(fv - fw)));
			//edge_weight = beta_*exp(0.5*(evw+fabs(fv - fw)));
			//edge_weight = beta_*(evw+fabs(fv - fw))*0.5;
			opt_gc.add_DirectedBinaryCost(source_key, target_key, edge_weight);

			BinaryCost bcost;
			bcost.sID = source_key;
			bcost.tID = target_key;
			bcost.cost = edge_weight;
			dbcosts1.push_back(bcost);

			//evw = halfedge_att_list_[twin_eidx].occupied / (halfedge_att_list_[twin_eidx].occupied + halfedge_att_list_[twin_eidx].empty);
			fv = face_att_list_[target_key].sparsity;
				//face_att_list_[target_key].occupied / (face_att_list_[target_key].occupied + face_att_list_[target_key].empty);
			fw = face_att_list_[source_key].sparsity;
				//face_att_list_[source_key].occupied / (face_att_list_[source_key].occupied + face_att_list_[source_key].empty);

			edge_weight = beta_*exp(-0.5*(mean_evw + fabs(fv - fw)));
			//edge_weight = beta_*exp(0.5*(mean_evw + fabs(fv - fw)));
			//edge_weight = beta_*exp(-(evw * fabs(fv - fw)));
			//edge_weight = beta_*exp(0.5*(evw + fabs(fv - fw)));
			//edge_weight = beta_*(evw + fabs(fv - fw))*0.5;
			opt_gc.add_DirectedBinaryCost(target_key, source_key, edge_weight);

			bcost.sID = target_key;
			bcost.tID = source_key;
			bcost.cost = edge_weight;
			dbcosts2.push_back(bcost);
		}

		opt_gc.endBuildGraph();
		std::cout << "done!" << std::endl;

		face_lables_.clear();
		face_lables_ = opt_gc.getLabels();

		int nmarkedface = face_lables_[0].size() + face_lables_[1].size();
		assert(nmarkedface == nfaces);
		//std::vector<bool> face_flags_;
		face_flags_.clear();
		face_flags_.resize(nmarkedface, false);  //inner: true;  outer: false 
		for (int i = 0; i < face_lables_[1].size(); i++)
		{
			int id = face_lables_[1].at(i);

			face_flags_[id] = true;
		}
		face_flags_[0] = false;

#ifdef OUTPUT_DEBUG_INFO
		{
			std::string filename = output_dir_ + "/labeled_faces.shp";
			exportAllPolygons(filename, false);
 			filename = output_dir_ + "/line_sparse.shp";
 			exportLineSegments(filename, arr2_); 
		}
#endif

		//////////////////////////////////////////////////
		//合并相邻的具有相同标记的facets
		LineArrangement::Arrangement_2::Face_handle f;
		nfaces = arr2_->number_of_faces();

		std::cout << " faces:" << nfaces << std::endl;
		//std::cout << nfaces << " faces:" << std::endl;
		int nunbfaces = 0;

		int fidx, eidx, vidx;
		std::vector<bool> face_exist_flags;
		face_exist_flags.resize(nfaces, true);

//		std::vector<bool> halfedge_exist_flags;
// 		int halfedge_num = arr2_->number_of_halfedges();
// 		halfedge_exist_flags.resize(halfedge_num, true);

		//std::vector<bool> face_traverse_flag;
		//std::vector<bool> halfedge_traverse_flag;

		std::cout << "------------------------------" << std::endl;
		std::cout << "vertice, halfedge, face: ";
		std::cout << "(" << arr2_->number_of_vertices() << ", " << arr2_->number_of_halfedges()
			<< ", " << arr2_->number_of_faces() << ")" << std::endl;

		std::cout << "begin to merge faces" << std::endl;

		///////////////////////////// traverse edges to merge faces //////////////////
		LineArrangement::Edge_iterator eit;
		std::set<LineArrangement::Halfedge_handle> removal_edge_list;   //按边遍历不会产生重复，按面遍历会产生重复边
		for (eit = arr2_->edges_begin(); eit != arr2_->edges_end(); ++eit)
		{
			LineArrangement::Halfedge_handle he = eit;

			// 		if (he->data() == 310)
			// 		{
			// 			;
			// 		}

			if (he->face()->is_unbounded() && he->twin()->face()->is_unbounded())
			{//antenna
				removal_edge_list.insert(he);
				continue;
			}

			int fid_i = he->face()->data();
			int fid_j = he->twin()->face()->data();

			if (face_flags_[fid_i] == face_flags_[fid_j])
			{//内部边

				removal_edge_list.insert(he);

// 				if (face_flags_[fid_i])
// 				{//inner
// 					int eid = he->data();
// 					double sparseDeg = halfedge_att_list_[eid].occupied
// 						/ (halfedge_att_list_[eid].occupied + halfedge_att_list_[eid].empty);
// 
// 					double dx, dy;
// 					dx = CGAL::to_double(he->source()->point().x()/*.to_double()*/ - he->target()->point().x())/*.to_double()*/;
// 					dy = CGAL::to_double(he->source()->point().y()/*.to_double()*/ - he->target()->point().y())/*.to_double()*/;
// 					double edgeLen = sqrt(dx*dx + dy*dy);
// 
// 					// 					if (edgeLen > 4.0)
// 					// 					{
// 					// 						std::cout << eid << " source:(" << he->source()->point().x().to_double() << "," << he->source()->point().y().to_double() << "), "
// 					// 							<< "target:(" << he->target()->point().x().to_double() << "," << he->target()->point().y().to_double() << ")" << std::endl;
// 					// 					}
// 
// 					if (edgeLen*sparseDeg < inner_edge_Th_)
// 						removal_edge_list.insert(he);
// 					else
// 						std::cout << "edge " << eid << "/" << he->twin()->data()
// 						<< " is an inner wall" << std::endl;
// 				}
// 				else
// 				{//outter
// 				 //可能存在标记错误, 用edge来约束
// 
// 					int eid = he->data();
// 					double sparseDeg = halfedge_att_list_[eid].occupied
// 						/ (halfedge_att_list_[eid].occupied + halfedge_att_list_[eid].empty);
// 
// 					double dx, dy;
// 					dx = CGAL::to_double(he->source()->point().x()/*.to_double()*/ - he->target()->point().x())/*.to_double()*/;
// 					dy = CGAL::to_double(he->source()->point().y()/*.to_double()*/ - he->target()->point().y())/*.to_double()*/;
// 					double edgeLen = sqrt(dx*dx + dy*dy);
// 
// 					double fdeg_i = face_att_list_[fid_i].occupied / (face_att_list_[fid_i].occupied + face_att_list_[fid_i].empty);
// 					double fdeg_j = face_att_list_[fid_j].occupied / (face_att_list_[fid_j].occupied + face_att_list_[fid_j].empty);
// 
// 					// 					if (edgeLen > 4.0)
// 					// 					{
// 					// 						std::cout << eid << " source:(" << he->source()->point().x().to_double() << "," << he->source()->point().y().to_double() << "), "
// 					// 							<< "target:(" << he->target()->point().x().to_double() << "," << he->target()->point().y().to_double() << ")" << std::endl;
// 					// 					}
// 
// 					if (edgeLen*sparseDeg > border_edge_Th_ /*&&(fdeg_i > 0.5 || fdeg_j > 0.5)*/)
// 					{
// 						std::cout << "edge " << eid << "/" << he->twin()->data()
// 							<< " is an border wall" << std::endl;
// 
// 						// 					if (fdeg_i > 0.3)
// 						// 						face_flags_[fid_i] = true;
// 						// 					if (fdeg_j > 0.3)
// 						// 						face_flags_[fid_j] = true;
// 					}
// 					else
// 						removal_edge_list.insert(he);
//				}
			}
		}

		removeEdges(arr2_, &removal_edge_list);

		removal_edge_list.clear();

#ifdef OUTPUT_DEBUG_INFO
		{
			std::string filename = output_dir_ + "/polygons_before_mergeFaces.shp";
			exportAllPolygons(filename, true);
			filename = output_dir_ + "/polylines_before_mergeFaces.shp";
			exportLineSegments(filename, arr2_);
		}
#endif

		std::cout << "------------------------------" << std::endl;
		std::cout << "vertice, halfedge, face: ";
		std::cout << "(" << arr2_->number_of_vertices() << ", " << arr2_->number_of_halfedges()
			<< ", " << arr2_->number_of_faces() << ")" << std::endl;

		//过滤微小多边形
		std::set<LineArrangement::Face_handle>  merged_face_list;
//		LineArrangement::Face_iterator fit;
		for (fit = arr2_->faces_begin(); fit != arr2_->faces_end(); ++fit)
		{
			int face_id = fit->data();
			if (fit->is_unbounded())
				continue;

			//std::vector<Halfedge_handle> face_bounder;

			//typedef CGAL::Polygon_2<Kernel> Polygon_2;
			LineArrangement::Polygon_2 poly;

			LineArrangement::Arrangement_2::Ccb_halfedge_circulator  circ = fit->outer_ccb();
			LineArrangement::Arrangement_2::Ccb_halfedge_circulator  curr = circ;

			do
			{
				LineArrangement::Arrangement_2::Halfedge_handle he = curr;
				int hid = he->data();
				int thid = he->twin()->data();

				//face_bounder.push_back(he);

				LineArrangement::Point_2 p = he->target()->point();

				poly.push_back(p);

				curr = curr->next();

			} while (curr != circ);

			double area = CGAL::to_double(poly.area())/*.to_double()*/;
			if (area < polygon_area_Th_)
			{//待删除的小多边形
			 //删除策略: 合并到最相似的面中
				merged_face_list.insert(fit);
				std::cout << "find tiny polygon: " << fit->data()
					<< " area: " << area << std::endl;
			}
		}

		//	removeEdges(&arr2_, &removal_edge_list);
		mergeFaces(arr2_, &merged_face_list);

#ifdef OUTPUT_DEBUG_INFO
		{
			std::string filename = output_dir_ + "/polygons_before_mergeCollinearEdges.shp";
			exportAllPolygons(filename, true); 
			filename = output_dir_ + "/polylines_before_mergeCollinearEdges.shp";
			exportLineSegments(filename, arr2_); 
		}
#endif

		//merge collinear edges
		mergeCollinearEdges(arr2_);

		return 0;
	}


	template <typename PointT> void
		BoundaryOptimization<PointT>::removeEdges(LineArrangement::Arrangement_2 *arr, std::set<LineArrangement::Halfedge_handle> *removal_list)
	{
		std::set<LineArrangement::Halfedge_handle>::iterator iter;

		for (iter = removal_list->begin(); iter != removal_list->end(); ++iter)
		{
			LineArrangement::Halfedge_handle he = *iter;
			int eid = he->data();
			int teid = he->twin()->data();

			std::cout << "delete edge (" << eid << "/" << teid << "), ";
			int fid = he->face()->data();
			int tfid = he->twin()->face()->data();

			if (face_flags_[fid] != face_flags_[tfid])
				continue;  //边界边不能删除

			std::cout << "in face " << fid << std::endl;

			arr->remove_edge(he);
		}

	}

	template <typename PointT> void
		BoundaryOptimization<PointT>::mergeFaces(LineArrangement::Arrangement_2 *arr, std::set<LineArrangement::Face_handle> *face_list)
	{
		typedef std::vector<LineArrangement::Halfedge_handle>  EdgeList;

		std::set<LineArrangement::Halfedge_handle> removed_halfedges;  //需要删除的边集合,  不可重复
		std::set<LineArrangement::Halfedge_handle>::iterator removeIter;

		std::set<LineArrangement::Face_handle>::iterator fIter;
		for (fIter = face_list->begin(); fIter != face_list->end(); ++fIter)
		{
			LineArrangement::Face_handle fit = *fIter;
			int fid = fit->data();
			std::cout << "face: " << fid << " ";

			boost::unordered_map<int, EdgeList>	neighbor_edges;   //邻接face id, 邻接边链表
			boost::unordered_map<int, EdgeList>::iterator map_iter;

			LineArrangement::Arrangement_2::Ccb_halfedge_circulator  circ = fit->outer_ccb();
			LineArrangement::Arrangement_2::Ccb_halfedge_circulator  curr = circ;
			do
			{
				LineArrangement::Halfedge_handle he = curr;
				assert(fid == he->face()->data());

				int nfid = he->twin()->face()->data();

				map_iter = neighbor_edges.find(nfid);
				if (map_iter != neighbor_edges.end())
				{
					map_iter->second.push_back(he);
				}
				else
				{
					EdgeList  elist;
					elist.push_back(he);
					neighbor_edges.insert(std::make_pair(nfid, elist));
				}

				curr = curr->next();

			} while (curr != circ);

			int mergeTo = -1;     //合并到哪个face
//			float max_empty = -1;
			float min_w_length = std::numeric_limits<float>::max();
			EdgeList::iterator eiter;
			for (map_iter = neighbor_edges.begin(); map_iter != neighbor_edges.end(); ++map_iter)
			{
				float emptyDeg = 0;
				for (eiter = map_iter->second.begin(); eiter != map_iter->second.end(); ++eiter)
				{
					LineArrangement::Halfedge_handle he = *eiter;
					int eid = he->data();

					emptyDeg += halfedge_att_list_[eid].length*halfedge_att_list_[eid].sparsity;
				}

				if (emptyDeg < min_w_length)
				{
					mergeTo = map_iter->first;
					min_w_length = emptyDeg;
				}
			}

			if (mergeTo == -1)
				continue;

			std::cout << " remove face: " << fid << " merge to: " << mergeTo << std::endl;

			map_iter = neighbor_edges.find(mergeTo);

			for (eiter = map_iter->second.begin(); eiter != map_iter->second.end(); ++eiter)
			{
				LineArrangement::Halfedge_handle he = *eiter;
				removeIter = removed_halfedges.find(he);
				if (removeIter != removed_halfedges.end())
					continue;

				removeIter = removed_halfedges.find(he->twin());
				if (removeIter != removed_halfedges.end())
					continue;

				removed_halfedges.insert(he);

				std::cout << "remove edge: " << he->data() << std::endl;
				arr->remove_edge(he);
			}
		}
	}

	template <typename PointT> void
		BoundaryOptimization<PointT>::mergeCollinearEdges(LineArrangement::Arrangement_2 *arr)
	{
		// 	bool are_mergeable(Halfedge_const_handle e1, Halfedge_const_handle e2) const;

		LineArrangement::Edge_iterator eit;
		LineArrangement::Arrangement_2::Originating_curve_iterator     ocit;
		for (eit = arr->edges_begin(); eit != arr->edges_end(); ++eit)
		{
			if (arr->are_mergeable(eit, eit->next()))
			{
				arr->merge_edge(eit, eit->next());
				//continue;
			}

			/*		Vertex_handle vit = eit->target();



			int deg = vit->degree();

			// 		if (vit->degree() != 2)
			// 			continue;

			Arrangement_2::Halfedge_around_vertex_circulator  circ = vit->incident_halfedges();
			Arrangement_2::Ccb_halfedge_circulator cch = circ;
			int nedge = 0;
			do
			{
			nedge++;
			} while (++cch !=circ);

			if (nedge != 2)
			{
			continue;
			}

			cch = circ;
			++cch;

			int n1 = arr->number_of_originating_curves(cch);
			int n2 = arr->number_of_originating_curves(circ);
			if (n1 == 1 && n2 == 1)
			{
			if (arr->originating_curves_begin(cch) == arr->originating_curves_begin(circ))
			{
			arr->merge_edge(cch, circ);
			}
			}*/
		}
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////
	//修复自相交的多边形(一种拓扑错误)
	//1.计算winding number; 2. 找到自相交的ring; 3. 删除ring
	template <typename PointT> int
		BoundaryOptimization<PointT>::repairLineArrangement_selfintersection(LineArrangement::Arrangement_2 *arr, LineArrangement::PolygonList &polygons)
	{
		typedef std::pair<bool, int>                        Data;
		typedef boost::unordered_map<int, Data>				WNMap;    //winding number map
		typedef boost::unordered_map<int, Vertex_handle>	VhMap;		//vertex handle map

		Winding_number<Arrangement_2> winding_number(*arr);

		polygons.clear();
		//holes.clear();

		std::cout << arr->number_of_faces() << " faces with " << arr->number_of_unbounded_faces() << " unbounded faces" << std::endl;
		typename Arrangement_2::Face_iterator fi;
		for (fi = arr->faces_begin(); fi != arr->faces_end(); ++fi)
		{
			if (fi->is_unbounded())
				continue;

			int fid = fi->data();
			std::cout << "face " << fid;
			WNMap::iterator it_wm;
			it_wm = winding_number._windingmap.find(fid);

			assert(it_wm != winding_number._windingmap.end());

			std::cout << " winding number = " << it_wm->second.second;

			std::vector<int> vlist;
			std::vector<int> hlist;
			if ((it_wm->second.second % 2) != 0)
			{//valid
				std::cout << " is valid" << std::endl;
				typename Arrangement_2::Ccb_halfedge_circulator cco = fi->outer_ccb();
				do {
					vlist.push_back(cco->target()->data());

				} while (++cco != fi->outer_ccb());
			}
			else
			{//self-intersection polygon
				std::cout << " need to repair" << std::endl;
				VhMap	vmap;
				typename Arrangement_2::Ccb_halfedge_circulator cco = fi->outer_ccb();
				int vnum = 0;
				do {
					int vid = cco->target()->data();

					VhMap::iterator it_vm;
					it_vm = vmap.find(vid);

					if (it_vm != vmap.end())
					{//catch a ring
						hlist.insert(hlist.begin(), vid);
						while (vlist.back() != vid)
						{
							hlist.insert(hlist.begin(), vlist.back());

							vlist.pop_back();
							vnum--;
						}

						//holes.push_back(hlist);
					}
					else
					{
						vmap.insert(std::make_pair(vid, cco->target()));
						vlist.push_back(vid);
						vnum++;
					}
				} while (++cco != fi->outer_ccb());
			}

			if (vlist.size() > 0)
				polygons.insert(std::make_pair(fid, vlist));

		}

		return (0);
	}

	///////////////////////////////////////////////////////////////////
	//修复多边形中的ring (拓扑错误的self-intersection, 或拓扑正确的ring)
	//keep_antenna: 是否保留antenna
	template <typename PointT> int
		BoundaryOptimization<PointT>::repairLineArrangement_ring(LineArrangement::Arrangement_2 *arr, PolygonList &polygons, bool keep_antenna /* = false */)
	{
		//typedef std::pair<bool, int>                        Data;
		//typedef boost::unordered_map<int, Data>				WNMap;    //winding number map
		typedef boost::unordered_map<int, LineArrangement::Vertex_handle>	VhMap;		//vertex handle map
	//	Winding_number<Arrangement_2> winding_number(*arr);

		polygons.clear();
		//holes.clear();

		int nfaces = arr->number_of_faces();
		std::cout << arr->number_of_faces() << " faces with " << arr->number_of_unbounded_faces() << " unbounded faces" << std::endl;
		typename LineArrangement::Arrangement_2::Face_iterator fi;
		for (fi = arr->faces_begin(); fi != arr->faces_end(); ++fi)
		{
			if (fi->is_unbounded())
				continue;

			int fid = fi->data();
			std::cout << "repair face: " << fid << "..." << std::endl;

			std::vector<int> vlist;
			std::vector<int> hlist;
			VhMap	vmap;
			typename LineArrangement::Arrangement_2::Ccb_halfedge_circulator cco = fi->outer_ccb();
			int vnum = 0;
			do {
				int vid = cco->target()->data();

				std::cout << "eid: " << vid << std::endl;

				VhMap::iterator it_vm;
				it_vm = vmap.find(vid);

				if (it_vm != vmap.end())
				{//catch a ring
					if (keep_antenna && cco->face() == cco->twin()->face())
					{
						std::cout << "find an antenna:  " << vid << std::endl;
						vlist.push_back(vid);
						vnum++;
					}
					else
					{
						std::cout << "find an ring: ";
						hlist.insert(hlist.begin(), vid);
						while (vlist.back() != vid)
						{
							hlist.insert(hlist.begin(), vlist.back());

							std::cout << hlist.front() << " ";

							vlist.pop_back();
							vnum--;
						}
						std::cout << std::endl;
						
					}


					//holes.push_back(hlist);
				}
				else
				{
					vmap.insert(std::make_pair(vid, cco->target()));
					vlist.push_back(vid);
					vnum++;
				}
			} while (++cco != fi->outer_ccb());

			if (vlist.size() > 0)
				polygons.insert(std::make_pair(fid, vlist));

		}
		std::cout << "repair face: done! " << nfaces << "/" << nfaces << std::endl;

		return (0);
	}

	template <typename PointT> int
		BoundaryOptimization<PointT>::repairPolygon(LineArrangement::Arrangement_2 *arr, LineArrangement::Face_handle fi, 
			VertexHandleList &polygons)
	{
		typedef boost::unordered_map<int, LineArrangement::Vertex_handle>	VhMap;		//vertex handle map
		//	Winding_number<Arrangement_2> winding_number(*arr);

		polygons.clear();
		//holes.clear();

// 		int nfaces = arr->number_of_faces();
// 		std::cout << arr->number_of_faces() << " faces with " << arr->number_of_unbounded_faces() << " unbounded faces" << std::endl;
// 		typename LineArrangement::Arrangement_2::Face_iterator fi;
// 		for (fi = arr->faces_begin(); fi != arr->faces_end(); ++fi)
//		{
			if (fi->is_unbounded())
				return 0;

			int fid = fi->data();
			//std::cout << "repair face: " << fid << " ... ";

			std::vector<int> vlist;
			std::vector<int> hlist;
			VhMap	vmap;
			typename LineArrangement::Arrangement_2::Ccb_halfedge_circulator cco = fi->outer_ccb();
			int vnum = 0;
			do {
				int vid = cco->target()->data();

				VhMap::iterator it_vm;
				it_vm = vmap.find(vid);

				if (it_vm != vmap.end())
				{//catch a ring
// 					if (keep_antenna && cco->face() == cco->twin()->face())
// 					{
// 						std::cout << "find an antenna:  " << vid << std::endl;
// 						vlist.push_back(vid);
// 						vnum++;
// 					}
// 					else
// 					{
						hlist.insert(hlist.begin(), vid);
						while (vlist.back() != vid)
						{
							hlist.insert(hlist.begin(), vlist.back());

							vlist.pop_back();
							vnum--;
						}
//					}


					//holes.push_back(hlist);
				}
				else
				{
					vmap.insert(std::make_pair(vid, cco->target()));
					vlist.push_back(vid);
					vnum++;
				}
			} while (++cco != fi->outer_ccb());

			for (int i = 0; i < vlist.size(); ++i)
			{
				int vid = vlist[i];
				LineArrangement::Vertex_handle vh = vmap[vid];
				polygons.push_back(vh);
			}

//		}
		//std::cout << "done! " << std::endl;
		return 0;
	}

	template <typename PointT> int
		BoundaryOptimization<PointT>::exportOutterBoundary(const std::string &filename)
	{
		const char *pszDriverName = "ESRI Shapefile";
		GDALDriver *poDriver;
		GDALAllRegister();
		poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
		if (poDriver == NULL)
		{
			std::cout << pszDriverName << " driver not available" << std::endl;
			return (-1);
		}

		GDALDataset *poDS;
		poDS = poDriver->Create(filename.c_str(), 0, 0, 0, GDT_Unknown, NULL); //创建数据源
		if (poDS == NULL)
		{
			std::cout << "can't create output file: " << filename << std::endl;
			return (-1);
		}
		OGRLayer *poLayer;
		poLayer = poDS->CreateLayer("building boundary", NULL, wkbPolygon, NULL); //创建图层
		if (poLayer == NULL)
		{
			std::cout << "Layer creation failed" << std::endl;
			return (-1);
		}

		OGRFieldDefn oField("Name", OFTString); //创建属性
		oField.SetWidth(32);
		if (poLayer->CreateField(&oField) != OGRERR_NONE)
		{
			std::cout << "Creating PolygonName field failed" << std::endl;
			return (-1);
		}

		OGRFieldDefn oField1("PolygonID", OFTInteger64);
		//oField1.SetPrecision(3);
		if (poLayer->CreateField(&oField1) != OGRERR_NONE)
		{
			std::cout << "Creating PolygonID field failed." << std::endl;
			return (-1);
		}

		Face_handle unf = arr2_.unbounded_face();

		int fid = 0;
		char fname[32];

		// Traverse the inner boundary (holes).
		Hole_iterator hit;
		for (hit = unf->holes_begin(); hit != unf->holes_end(); ++hit, ++fid)
		{
			//fid = hit->data();
			sprintf(fname, "polygon_%03d", fid);

			OGRFeature *poFeature;
			poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
			poFeature->SetField(0, fname);
			poFeature->SetField(1, fid);

			OGRLinearRing ring;

			typename Arrangement_2::Ccb_halfedge_circulator cch = *hit;
			do {
				typename Arrangement_2::Face_handle inner_face = cch->twin()->face();
				if (inner_face == cch->face())
					continue;        // discard antennas

				int id = cch->face()->data();
				int innerfid = cch->twin()->face()->data();

				int eid = cch->data();
				int teid = cch->twin()->data();

				double x, y;
				x = cch->target()->point().x().to_double();
				y = cch->target()->point().y().to_double();
				ring.addPoint(x, y, floor_hei);

			} while (++cch != *hit);

			ring.closeRings();//首尾点重合形成闭合环 
			OGRPolygon poly;
			poly.addRing(&ring);

			poFeature->SetGeometry(&poly);
			if (poLayer->CreateFeature(poFeature) != OGRERR_NONE)
			{
				std::cout << "Failed to create feature: " << fname << std::endl;
			}
			OGRFeature::DestroyFeature(poFeature);
		}

		GDALClose(poDS);
		return (0);
	}

	template <typename PointT> int
		BoundaryOptimization<PointT>::exportAllPolygons(const std::string &filename, const bool output_holes /* = false */)
	{
		PolygonList polygons;
		//	repairLineArrangement_selfintersection(&arr2_, polygons);

		repairLineArrangement_ring(arr2_, polygons);
		const char *pszDriverName = "ESRI Shapefile";
		GDALDriver *poDriver;
		GDALAllRegister();
		poDriver = GetGDALDriverManager()->GetDriverByName(pszDriverName);
		if (poDriver == NULL)
		{
			std::cout << pszDriverName << " driver not available" << std::endl;
			return (-1);
		}

		GDALDataset *poDS;
		poDS = poDriver->Create(filename.c_str(), 0, 0, 0, GDT_Unknown, NULL); //创建数据源
		if (poDS == NULL)
		{
			std::cout << "can't create output file: " << filename << std::endl;
			return (-1);
		}
		OGRLayer *poLayer;
		poLayer = poDS->CreateLayer("building boundary", NULL, wkbPolygon, NULL); //创建图层
		if (poLayer == NULL)
		{
			std::cout << "Layer creation failed" << std::endl;
			return (-1);
		}

		OGRFieldDefn oField("Name", OFTString); //创建属性
		oField.SetWidth(32);
		if (poLayer->CreateField(&oField) != OGRERR_NONE)
		{
			std::cout << "Creating PolygonName field failed" << std::endl;
			return (-1);
		}

		OGRFieldDefn oField1("PolygonID", OFTInteger64);
		//oField1.SetPrecision(3);
		if (poLayer->CreateField(&oField1) != OGRERR_NONE)
		{
			std::cout << "Creating PolygonID field failed." << std::endl;
			return (-1);
		}

		OGRFieldDefn oField2("Type", OFTString);
		//oField1.SetPrecision(3);
		if (poLayer->CreateField(&oField2) != OGRERR_NONE)
		{
			std::cout << "Creating Polygon Type field failed." << std::endl;
			return (-1);
		}

		// 	if (nmarkedface != (arr2_.number_of_faces() - arr2_.number_of_unbounded_faces()))
		// 		return (-1);

		LineArrangement::Face_handle unf = arr2_->unbounded_face();
		LineArrangement::Hole_iterator hit;
		int fid = 0;
		char fname[32];

		PolygonList::iterator pit;
		for (pit = polygons.begin(); pit != polygons.end(); ++pit)
		{
			fid = pit->first;
			sprintf(fname, "polygon_%03d", fid);

			if (!face_flags_[fid] && !output_holes)
				continue;	//不输出空洞多边形

			OGRFeature *poFeature;
			poFeature = OGRFeature::CreateFeature(poLayer->GetLayerDefn());
			poFeature->SetField(0, fname);
			poFeature->SetField(1, fid);

			if (face_flags_[fid])
				poFeature->SetField(2, bim::BoundaryTypeDesc[1]);
			else
			{
				poFeature->SetField(2, bim::BoundaryTypeDesc[0]);
			}


			OGRLinearRing ring;

			VertexIDList::iterator vit = pit->second.begin();
			do {
				int id = *vit;
				double x, y;
				x = CGAL::to_double(points_[id].x())/*.to_double()*/;
				y = CGAL::to_double(points_[id].y())/*.to_double()*/;
				ring.addPoint(x, y, /*floor_hei*/0);
			} while (++vit != pit->second.end());


			ring.closeRings();//首尾点重合形成闭合环 
			OGRPolygon poly;
			poly.addRing(&ring);

			poFeature->SetGeometry(&poly);
			if (poLayer->CreateFeature(poFeature) != OGRERR_NONE)
			{
				std::cout << "Failed to create feature: " << fname << std::endl;
			}
			OGRFeature::DestroyFeature(poFeature);
		}

		GDALClose(poDS);
		return (0);
	}

}

