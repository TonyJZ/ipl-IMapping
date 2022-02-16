#pragma once
#include "reconstruction/BoundaryReconstruction.h"
#include "commonAPIs/iplstring.h"
#include "fitting/alpha_shapes_2d.h"
#include "geometry/polygon/PolygonSimplification.h"
#include "spatialindexing/PointVoxelization.h"
#include "spatialindexing/PointPartitionQuadtree.h"
#include "fitting/pcPlaneDetection.h"
#include "fitting/geoModelDef.h"
#include "io/PointcloudIO.h"
#include "fitting/ModelParamsIO.h"
#include "reconstruction/LineArrangement.h"
#include "reconstruction/GridIntersection.h"

//Eigen
#include <Eigen/Dense>


#define _Output_Intermediate_

namespace ipl
{

template <typename PointT> int
BoundaryReconstruction<PointT>::exportPolygons(const std::string polyname, std::vector<OGRPolygon> &Poly,
	float roof_hei, float floor_hei, float profile_hei)
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
	poDS = poDriver->Create(polyname.c_str(), 0, 0, 0, GDT_Unknown, NULL); //创建数据源
	if (poDS == NULL)
	{
		std::cout << "can't create output file: " << polyname << std::endl;
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

	OGRFieldDefn oField4(rec::FieldDefn_PROFILE_HEIGHT, OFTReal);
	//oField1.SetPrecision(3);
	if (poLayer_polygon->CreateField(&oField4) != OGRERR_NONE)
	{
		std::cout << "Creating PolygonID field failed." << std::endl;
		return (-1);
	}

	int fid = 0;
	char fname[32]="";
//	sprintf(fname, "building_%03d", fid);

	for (int i = 0; i < Poly.size(); ++i)
	{
		OGRFeature *poFeature;
		poFeature = OGRFeature::CreateFeature(poLayer_polygon->GetLayerDefn());
		poFeature->SetField(0, fname);
		poFeature->SetField(1, fid);
		poFeature->SetField(2, roof_hei);
		poFeature->SetField(3, floor_hei);
		poFeature->SetField(4, profile_hei);

		poFeature->SetGeometry(&(Poly[i]));
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
BoundaryReconstruction<PointT>::exportPolylines(const std::string polyname, OGRMultiLineString *lines, 
	float roof_hei, float floor_hei, float profile_hei)
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
	OGRLayer *poLayer_polyline;
	poLayer_polyline = poDS->CreateLayer("boundary lines", NULL, wkbMultiLineString, NULL); //创建图层
	if (poLayer_polyline == NULL)
	{
		std::cout << "Layer creation failed" << std::endl;
		return (-1);
	}

	OGRFieldDefn oField(rec::FieldDefn_Polygon_ID, OFTInteger64);
	//oField1.SetPrecision(3);
	if (poLayer_polyline->CreateField(&oField) != OGRERR_NONE)
	{
		std::cout << "Creating PolygonID field failed." << std::endl;
		return (-1);
	}


	OGRFieldDefn oField1(rec::FieldDefn_Name, OFTString); //创建属性
	oField.SetWidth(256);
	if (poLayer_polyline->CreateField(&oField1) != OGRERR_NONE)
	{
		std::cout << "Creating PolygonName field failed" << std::endl;
		return (-1);
	}

	OGRFieldDefn oField2(rec::FieldDefn_ROOF_HEIGHT, OFTReal);
	//oField1.SetPrecision(3);
	if (poLayer_polyline->CreateField(&oField2) != OGRERR_NONE)
	{
		std::cout << "Creating PolygonID field failed." << std::endl;
		return (-1);
	}

	OGRFieldDefn oField3(rec::FieldDefn_FLOOR_HEIGHT, OFTReal);
	//oField1.SetPrecision(3);
	if (poLayer_polyline->CreateField(&oField3) != OGRERR_NONE)
	{
		std::cout << "Creating PolygonID field failed." << std::endl;
		return (-1);
	}

	OGRFieldDefn oField4(rec::FieldDefn_PROFILE_HEIGHT, OFTReal);
	//oField1.SetPrecision(3);
	if (poLayer_polygon->CreateField(&oField4) != OGRERR_NONE)
	{
		std::cout << "Creating PolygonID field failed." << std::endl;
		return (-1);
	}

	// 	int fid = 0;
	// 	char fname[32];
	sprintf(fname, "lines_%03d", fid);

	//	OGRFeature *poFeature;
	poFeature = OGRFeature::CreateFeature(poLayer_polyline->GetLayerDefn());
	poFeature->SetField(0, fname);
	poFeature->SetField(1, fid);
	poFeature->SetField(2, roof_hei);
	poFeature->SetField(3, floor_hei);
	poFeature->SetField(4, profile_hei);

	poFeature->SetGeometry(lines);
	if (poLayer_polyline->CreateFeature(poFeature) != OGRERR_NONE)
	{
		std::cout << "Failed to create feature: " << fname << std::endl;
	}
	OGRFeature::DestroyFeature(poFeature);

	GDALClose(poDS);
	return (0);
}

template <typename PointT> int 
BoundaryReconstruction<PointT>::exportWallSegments(const std::string dir)
{
	int iModel = 0;
	int iWall = 0, iRoof = 0;

	float ver_degTh = 20.0, hor_degTh = 80.0;
	float threshold_r = cosf(ver_degTh*iplDEG2RAD);
	float threshold_f = cosf(hor_degTh*iplDEG2RAD);
	Eigen::Vector3f  nVertical = Eigen::Vector3f(0, 0, 1);
	int flag; //0, 1:wall; 2:roof

	//pcl::PCDWriter writer;
	for (int i = 0; i < wall_inliers_.size(); i++)
	{
		if (wall_inliers_[i].size() == 0)
			continue;

		//判断是否垂直
		Eigen::Map<Eigen::Vector3f> nghbr_normal(static_cast<float*> (&(wall_coeffs_[i].coef.values[0])));
		float dot_product = fabsf(nghbr_normal.dot(nVertical));
		if (dot_product > threshold_r)
		{
			flag = 2;  //roof
			continue;
		}
		else if (dot_product < threshold_f)
		{
			flag = 1;  //wall
		}
		else
		{
			flag = 0;
			continue;
		}

		
		//accept
		std::string out_name, out_param;
		char buf[32], buf_para[32];
		
		if (flag == 1)
		{
			sprintf(buf, "wall_model_%04d.pcd", iWall);
			sprintf(buf_para, "wall_model_%04d%s", iWall, geoModel::ModelParamFile_Suffix);
			iWall++;
		}
		else if (flag == 2)
		{
			sprintf(buf, "roof_model_%04d.pcd", iRoof);
			sprintf(buf_para, "roof_model_%04d%s", iRoof, geoModel::ModelParamFile_Suffix);
			iRoof++;
		}

		out_name = /*pOutDir*/dir;
		out_name += "/";
		//out_name += result_name;
		out_name += buf;

		out_param = /*pOutDir*/dir;
		out_param += "/";
		//out_param += result_name;
		out_param += buf_para;

		//writer.write<PointT>(out_name, *sort_cloud, inliers[i], true);
		write_PointCloud(out_name, *sorted_wall_cloud_, wall_inliers_[i], true);

		Eigen::Vector4f bbmin, bbmax;
		pcl::getMinMax3D(*sorted_wall_cloud_, wall_inliers_[i], bbmin, bbmax);
		double min_pt[3], max_pt[3];
		min_pt[0] = bbmin[0]; min_pt[1] = bbmin[1]; min_pt[2] = bbmin[2];
		max_pt[0] = bbmax[0]; max_pt[1] = bbmax[1]; max_pt[2] = bbmax[2];
		//Save_ParamFile(out_param.c_str(), sMT_plane, planeCoeffs[i]);
		memcpy(wall_coeffs_[i].bbox.min_pt, min_pt, sizeof(double) * 3);
		memcpy(wall_coeffs_[i].bbox.max_pt, max_pt, sizeof(double) * 3);
		save_geoModel_params(out_param.c_str(), /*min_pt, max_pt,*/ wall_coeffs_[i]);

		iModel++;
	}

	return 0;
}

template <typename PointT> int
BoundaryReconstruction<PointT>::exportRoofSegments(const std::string dir)
{
	int iModel = 0;
	int iWall = 0, iRoof = 0;

	float ver_degTh = 20.0, hor_degTh = 80.0;
	float threshold_r = cosf(ver_degTh*iplDEG2RAD);
	float threshold_f = cosf(hor_degTh*iplDEG2RAD);
	Eigen::Vector3f  nVertical = Eigen::Vector3f(0, 0, 1);
	int flag; //0, 1:wall; 2:roof

			  //pcl::PCDWriter writer;
	for (int i = 0; i < roof_inliers_.size(); i++)
	{
		if (roof_inliers_[i].size() == 0)
			continue;

		//判断是否垂直
		Eigen::Map<Eigen::Vector3f> nghbr_normal(static_cast<float*> (&(roof_coeffs_[i].coef.values[0])));
		float dot_product = fabsf(nghbr_normal.dot(nVertical));
		if (dot_product > threshold_r)
		{
			flag = 2;  //roof
		}
		else if (dot_product < threshold_f)
		{
			flag = 1;  //wall
			continue;
		}
		else
		{
			flag = 0;
			continue;
		}


		//accept
		std::string out_name, out_param;
		char buf[32], buf_para[32];

		if (flag == 1)
		{
			sprintf(buf, "wall_model_%04d.pcd", iWall);
			sprintf(buf_para, "wall_model_%04d%s", iWall, geoModel::ModelParamFile_Suffix);
			iWall++;
		}
		else if (flag == 2)
		{
			sprintf(buf, "roof_model_%04d.pcd", iRoof);
			sprintf(buf_para, "roof_model_%04d%s", iRoof, geoModel::ModelParamFile_Suffix);
			iRoof++;
		}

		out_name = /*pOutDir*/dir;
		out_name += "/";
		//out_name += result_name;
		out_name += buf;

		out_param = /*pOutDir*/dir;
		out_param += "/";
		//out_param += result_name;
		out_param += buf_para;

		//writer.write<PointT>(out_name, *sort_cloud, inliers[i], true);
		write_PointCloud(out_name, *sorted_roof_cloud_, roof_inliers_[i], true);

		Eigen::Vector4f bbmin, bbmax;
		pcl::getMinMax3D(*sorted_roof_cloud_, roof_inliers_[i], bbmin, bbmax);
		double min_pt[3], max_pt[3];
		min_pt[0] = bbmin[0]; min_pt[1] = bbmin[1]; min_pt[2] = bbmin[2];
		max_pt[0] = bbmax[0]; max_pt[1] = bbmax[1]; max_pt[2] = bbmax[2];
		//Save_ParamFile(out_param.c_str(), sMT_plane, planeCoeffs[i]);
		memcpy(roof_coeffs_[i].bbox.min_pt, min_pt, sizeof(double) * 3);
		memcpy(roof_coeffs_[i].bbox.max_pt, max_pt, sizeof(double) * 3);
		save_geoModel_params(out_param.c_str(), /*min_pt, max_pt,*/ roof_coeffs_[i]);

		iModel++;
	}

	return 0;
}

template <typename PointT>
BoundaryReconstruction<PointT>::BoundaryReconstruction()
{
	ZPrf_ratio_ = 0; // 从墙角点开始提取
}

template <typename PointT>
BoundaryReconstruction<PointT>::~BoundaryReconstruction()
{

}

template <typename PointT> int
BoundaryReconstruction<PointT>::getRoofFloorHeight(float vsize, float &roofHei, float &floorHei)
{
//	float fHei = 0;

	ref_ptr<PointVoxelization<PointT> > voxeler(new PointVoxelization<PointT>);
//	float vsize = 1.0;

	voxeler->setInputCloud(input_);
	voxeler->setIndices(indices_);
	voxeler->apply(vsize, vsize, vsize);

	Eigen::Vector3i minBB, maxBB;
	voxeler->getVoxelScope(minBB, maxBB);
	int zNomin = minBB[2];
	int zNomax = maxBB[2];

	Eigen::Vector4f bbmin, bbmax;
	voxeler->getBBox(bbmin, bbmax);

	VoxelKeyMap *vmap = voxeler->getVoxelKeyMap();
	VoxelKeyMap vmapDup = *vmap;

	int hist_num = 8;
	if (zNomax - zNomin < hist_num)
		hist_num = zNomax - zNomin;

	std::vector<int> zHistogram;
	std::vector<int> zWallHist;
	zHistogram.resize(hist_num + 1, 0);

	float interval = float(zNomax - zNomin) / hist_num;

	//选择墙面
//	std::vector<iplOctreeKey>  candWallKeys;
 	int totalWallGrid = 0;
// 	floorHei = 0;
	zWallHist.resize(zNomax - zNomin + 1, 0);
	while (vmapDup.size() > 0)
	{
		VoxelKeyMap::iterator iter = vmapDup.begin();

		iplOctreeKey key = iter->first;
//		int nOccupied = 0;
//		std::vector<iplOctreeKey> cur_volume;
		int zmin_i, zmax_i;
		zmin_i = zNomax; zmax_i = zNomin;
		std::vector<int> cur_column;
		for (int zNo = zNomin; zNo <= zNomax; zNo++)
		{
			iplOctreeKey newkey = key;
			newkey.z = zNo;

			VoxelKeyMap::iterator it_find = vmapDup.find(newkey);
			if (it_find != vmapDup.end())
			{//存在的key
// 				nOccupied++;
// 				cur_volume.push_back(newkey);
				cur_column.push_back(zNo);

				if (zmin_i > zNo)
					zmin_i = zNo;
				if (zmax_i < zNo)
					zmax_i = zNo;

				int iStep = ceil((zNo - zNomin) / interval);
				zHistogram[iStep]++;

				vmapDup.erase(newkey);
			}

			if ((zmax_i - zmin_i)*vsize > 3.0)
			{//标准层高 3.0m
				totalWallGrid += cur_column.size();

				for (int i = 0; i < cur_column.size(); ++i)
				{
					int zNo = cur_column[i];
					zWallHist[zNo]++;
				}
			}
		}
	}

	//floorhei 只利用墙面点
	int aver_layerNum = floor(totalWallGrid/zWallHist.size()); //每个高程断面上的平均格网数
	
	int nLayer = 0;
	totalWallGrid = 0;
	for (int i = 0; i < zWallHist.size(); ++i)
	{//对outliers的处理
		if(zWallHist[i]<0.1*aver_layerNum)
			continue;

		nLayer++;
		totalWallGrid += zWallHist[i];
	}

	aver_layerNum = totalWallGrid / nLayer;

	int accumHist = 0;
	int minNo;
	for (int i = 0; i < zWallHist.size(); ++i)
	{
		accumHist += zWallHist[i];

		if (accumHist > aver_layerNum)
		{
			minNo = i;
			break;
		}
	}

	floorHei = bbmin[2] + minNo*vsize + 0.5*vsize;

//	floorHei /= wallGrid;

	//roofhei, 需要利用屋顶面网格
	int maxHist = 0;
	int maxNo;
	int totalHist = 0;
	for (int i = 0; i < hist_num + 1; ++i)
	{
		totalHist += zHistogram[i];
		if (maxHist < zHistogram[i])
		{
			maxHist = zHistogram[i];
			maxNo = i;
		}
	}

	roofHei = bbmin[2] + (maxNo*interval + zNomin)*vsize + 0.5*vsize;
	
	return 0;
}

template <typename PointT> int 
BoundaryReconstruction<PointT>::extractAlphaShape(float gsize, float simTh)
{
	bool bReady = initCompute();
	if (!bReady)
	{
		deinitCompute();
		return -1;
	}

	//1. 提取楼顶高度
	float roofHei, floorHei;
	getRoofFloorHeight(1.0, roofHei, floorHei);
	float zDiff = roofHei - floorHei;

	float  zProfile = floorHei + zDiff * ZPrf_ratio_;

	//2. 筛选剖面以上的点
	ref_ptr<iplPointCloud<PointT> > sel_cloud(new iplPointCloud <PointT>);
	for (int i = 0; i < indices_->size(); ++i)
	{
		int id = indices_->at(i);
		
		if(input_->points[id].z < zProfile)
			continue;

		PointT pt;
		pt = input_->points[id];
	
		sel_cloud->points.push_back(pt);
	}
	sel_cloud->height = 1;
	sel_cloud->width = sel_cloud->size();

	//2. 建立四叉树
	std::cout << "create quadtree..." << std::endl;
	PointPartitionQuadtree<PointT> quadtree_partition;
	quadtree_partition.setInputCloud(sel_cloud);
//	quadtree_partition.setIndices(indices_);

	quadtree_partition.apply(gsize);
	gridMap_.reset(new CellKeyMap);
	*gridMap_ = *(quadtree_partition.getCellKeyMap());
	std::cout << " quadtree partition is done!" << std::endl;

	ref_ptr<iplPointCloud<iplPointXY> > cent_cloud(new iplPointCloud <iplPointXY>);
	CellKeyMap::iterator iter;
	for (iter = gridMap_->begin(); iter != gridMap_->end(); ++iter)
	{
		QuadtreeKey key = iter->first;
		double xc, yc;
		quadtree_partition.getCenterPosition(key, xc, yc);

		iplPointXY pt;
		pt.x = xc;
		pt.y = yc;
		//		pt.z = 0;

		cent_cloud->points.push_back(pt);
	}
	cent_cloud->height = 1;
	cent_cloud->width = cent_cloud->size();

	std::cout << cent_cloud->size() << " grids with size " << gsize << std::endl;
	std::cout << "extract original boundary from grids ... ";
	//3. 从格网中心点中提取初始alpha shape
	std::vector<float> weights;
	OGRMultiLineString lineS;
	ipl::alpha_shape_2d(*cent_cloud, NULL, false, weights, lineS, /*1.0/(2*gsize)*/2 * gsize);

	LineArrangement  LA;
	std::vector<OGRPolygon>  orgPolys;
	LA.Initialize(&lineS);
	LA.getOutterBoundary(orgPolys, roofHei);
	std::cout << "done!" << std::endl;

	//4. 利用初始alpha shape 获取更精细的边界点，内部仍填充网格中心点 (效率考虑)
	Eigen::Vector4f bbmin, bbmax;
	quadtree_partition.getBBox(bbmin, bbmax);

	GridIntersection grid_intersect;
	grid_intersect.InitialGrid(bbmin, bbmax, gsize, gsize);

	CellKeyMap dupGridMap = *(quadtree_partition.getCellKeyMap());
	CellKeyMap intersected_grids;
	for (int iPoly = 0; iPoly < orgPolys.size(); ++iPoly)
	{//提取初始boundary穿过的网格
		OGRPolygon *polygon = &orgPolys[iPoly];

		for (int iR = -1; iR < polygon->getNumInteriorRings(); ++iR)
		{
			OGRLinearRing *ring = NULL;
			if (iR == -1)
				ring = polygon->getExteriorRing();
			else
				ring = polygon->getInteriorRing(iR);

			int ptNum = ring->getNumPoints();
		
			GridIntersection::Polygon_2 cpolygon;		//CGAL polygon
			for (int i = 0; i < ptNum-1; i++)
			{
				geometry::LineSeg2D lseg;

				OGRPoint pt;
				ring->getPoint(i, &pt);
				lseg.sp[0] = pt.getX();
				lseg.sp[1] = pt.getY();

				ring->getPoint(i + 1, &pt);
				lseg.ep[0] = pt.getX();
				lseg.ep[1] = pt.getY();

				CellKeyMap grid_indices;
				grid_intersect.getIntersectedGrids(lseg, grid_indices); //扫描线求交
		
				intersected_grids.insert(grid_indices.begin(), grid_indices.end());
			}
		}
	}

	ref_ptr<iplPointCloud<iplPointXYZ> > cand_cloud(new iplPointCloud <iplPointXYZ>);
	std::cout << "get candidate points... ";
	for (CellKeyMap::iterator it_cm = intersected_grids.begin();
		it_cm != intersected_grids.end();
		++it_cm)
	{
		QuadtreeKey key_arg = it_cm->first;

		CellKeyMap::iterator it_find = dupGridMap.find(key_arg);
		if (it_find == dupGridMap.end())
			continue;

		std::vector<int>* indPtr = it_find->second.getPointIndices();
		for (int i = 0; i < indPtr->size(); ++i)
		{
			int id = indPtr->at(i);
			PointT p0 = sel_cloud->points[id];

			if (p0.z < zProfile)
				continue;

			iplPointXYZ pt;
			pt.x = p0.x;
			pt.y = p0.y;
			pt.z = 0;

			cand_cloud->points.push_back(pt);
		}
		dupGridMap.erase(it_find);
	}
	//填补网格中心点
	for (iter = dupGridMap.begin(); iter != dupGridMap.end(); ++iter)
	{
		QuadtreeKey key = iter->first;
		double xc, yc;
		quadtree_partition.getCenterPosition(key, xc, yc);

		iplPointXYZ pt;
		pt.x = xc;
		pt.y = yc;
		pt.z = 0;

		cand_cloud->points.push_back(pt);
	}

	cand_cloud->height = 1;
	cand_cloud->width = cand_cloud->size();

	std::cout << "done!" << " point number: " << cand_cloud->size() << std::endl;

#ifdef _Output_Intermediate_
	std::string candName = IntermediateFolder_ + "/cand.pcd";
	ipl::write_PointCloud(candName, *cand_cloud, true);
#endif // _Output_Intermediate_

	ipl::alpha_shape_2d(*cand_cloud, NULL, false, weights, lineS, /*1.0/(2*gsize)*/2 * gsize);

	std::vector<OGRPolygon>  refinedPolys;
	LA.Initialize(&lineS);
	LA.getOutterBoundary(refinedPolys, roofHei);


	std::vector<OGRPolygon>  dstPolys;
	if (orgPolys.size() > 0)
	{
		simplify_polygon_CGAL(refinedPolys, dstPolys, simTh);
	}

#ifdef _Output_Intermediate_
	if (orgPolys.size() > 0)
	{
		std::string orgASName = AShapeFolder_ + "/" + rec::BBName_ORG;
		exportPolygons(orgASName, orgPolys, roofHei, floorHei, zProfile);

		std::string rfnASName = AShapeFolder_ + "/" + rec::BBName_RFN;
		exportPolygons(rfnASName, refinedPolys, roofHei, floorHei, zProfile);

		std::string result_name, suffix_name;
		extract_file_name(rfnASName, result_name, suffix_name);

		std::string simplifiedASName = result_name + rec::BBName_Simplified + suffix_name;
		exportPolygons(simplifiedASName, dstPolys, roofHei, floorHei, zProfile);
	}
#endif // _Output_Intermediate_


	return 0;
}

template <typename PointT> int
BoundaryReconstruction<PointT>::detectWalls(float minWallHei)
{
	ref_ptr<PointVoxelization<PointT> > voxeler(new PointVoxelization<PointT>);
	float vsize = 0.5;

	voxeler->setInputCloud(input_);
	voxeler->setIndices(indices_);
	voxeler->apply(vsize, vsize, vsize);

	Eigen::Vector3i minBB, maxBB;
	voxeler->getVoxelScope(minBB, maxBB);
	int zmin = minBB[2];
	int zmax = maxBB[2];

	VoxelKeyMap *vmap = voxeler->getVoxelKeyMap();
	VoxelKeyMap vmapDup = *vmap;

	//选择墙面
	std::vector<iplOctreeKey>  candWallKeys;
	while (vmapDup.size() > 0)
	{
		VoxelKeyMap::iterator iter = vmapDup.begin();

		iplOctreeKey key = iter->first;
		int nOccupied = 0;
		std::vector<iplOctreeKey> cur_volume;
		for (int zNo = zmin; zNo <= zmax; zNo++)
		{
			iplOctreeKey newkey = key;
			newkey.z = zNo;

			VoxelKeyMap::iterator it_find = vmapDup.find(newkey);
			if (it_find != vmapDup.end())
			{//存在的key
				nOccupied++;
				cur_volume.push_back(newkey);
			}
		}

		if (nOccupied*vsize > minWallHei)
		{
			candWallKeys.insert(candWallKeys.end(), cur_volume.begin(), cur_volume.end());
		}

		for (int i = 0; i < cur_volume.size(); ++i)
		{
			iplOctreeKey newkey = cur_volume[i];
			vmapDup.erase(newkey);
		}
	}

	//提取墙面点索引
	std::vector<int> wall_indices;
	for (int i = 0; i < candWallKeys.size(); ++i)
	{
		iplOctreeKey key = candWallKeys[i];

		VoxelKeyMap::iterator iter;
		iter = vmap->find(key);
		if (iter != vmap->end())
		{
			wall_indices.insert(wall_indices.end(), iter->second.getPointIndices()->begin(), iter->second.getPointIndices()->end());
		}
	}

// 	std::string wallName = IntermediateFolder_ + "/wall.pcd";
// 	ipl::write_PointCloud(wallName, *input_, wall_indices, true);

	//检测墙面
	sorted_wall_cloud_.reset(new iplPointCloud <PointT>);
	wall_inliers_.clear();
	wall_coeffs_.clear();

	detect_planes_by_eRansac(*input_, &wall_indices, false, *sorted_wall_cloud_, wall_inliers_, wall_coeffs_, 3000, 0.3, 0.5, 25);

#ifdef _Output_Intermediate_
	
	exportWallSegments(WallsFolder_);

#endif

	return 0;
}

template <typename PointT> int
BoundaryReconstruction<PointT>::detectPlanes(float minWallHei, float maxRoofHei)
{
	ref_ptr<PointVoxelization<PointT> > voxeler(new PointVoxelization<PointT>);
	float vsize = 0.5;

	int minWallVNum = floor(minWallHei / vsize) + 1;
	int maxRoofVNum = floor(maxRoofHei / vsize) + 1;

	voxeler->setInputCloud(input_);
	voxeler->setIndices(indices_);
	voxeler->apply(vsize, vsize, vsize);

	Eigen::Vector3i minBB, maxBB;
	voxeler->getVoxelScope(minBB, maxBB);
	int zmin = minBB[2];
	int zmax = maxBB[2];

	VoxelKeyMap *vmap = voxeler->getVoxelKeyMap();
	VoxelKeyMap vmapDup = *vmap;

	//选择墙面, 顶面
	std::vector<iplOctreeKey>  candWallKeys;
	std::vector<iplOctreeKey>  candRoofKeys;
	while (vmapDup.size() > 0)
	{
		VoxelKeyMap::iterator iter = vmapDup.begin();

		iplOctreeKey key = iter->first;
		int nOccupied = 0;
		std::vector<iplOctreeKey> cur_volume;
		for (int zNo = zmin; zNo <= zmax; zNo++)
		{
			iplOctreeKey newkey = key;
			newkey.z = zNo;

			VoxelKeyMap::iterator it_find = vmapDup.find(newkey);
			if (it_find != vmapDup.end())
			{//存在的key
				nOccupied++;
				cur_volume.push_back(newkey);
			}
		}

		if (nOccupied > minWallVNum)
		{
			candWallKeys.insert(candWallKeys.end(), cur_volume.begin(), cur_volume.end());
		}
		else if (nOccupied < maxRoofVNum)
		{
			candRoofKeys.insert(candRoofKeys.end(), cur_volume.begin(), cur_volume.end());
		}

		for (int i = 0; i < cur_volume.size(); ++i)
		{
			iplOctreeKey newkey = cur_volume[i];
			vmapDup.erase(newkey);
		}
	}

	//提取墙面点索引
	std::vector<int> wall_indices;
	for (int i = 0; i < candWallKeys.size(); ++i)
	{
		iplOctreeKey key = candWallKeys[i];

		VoxelKeyMap::iterator iter;
		iter = vmap->find(key);
		if (iter != vmap->end())
		{
			wall_indices.insert(wall_indices.end(), iter->second.getPointIndices()->begin(), iter->second.getPointIndices()->end());
		}
	}

	//提取顶面点索引
	std::vector<int> roof_indices;
	for (int i = 0; i < candRoofKeys.size(); ++i)
	{
		iplOctreeKey key = candRoofKeys[i];

		VoxelKeyMap::iterator iter;
		iter = vmap->find(key);
		if (iter != vmap->end())
		{
			roof_indices.insert(roof_indices.end(), iter->second.getPointIndices()->begin(), iter->second.getPointIndices()->end());
		}
	}
#ifdef _Output_Intermediate_

	std::string wallName = IntermediateFolder_ + "/wall.pcd";
	ipl::write_PointCloud(wallName, *input_, wall_indices, true);

	std::string roofName = IntermediateFolder_ + "/roof.pcd";
	ipl::write_PointCloud(roofName, *input_, roof_indices, true);

#endif
 	
	//检测墙面
	sorted_wall_cloud_.reset(new iplPointCloud <PointT>);
	wall_inliers_.clear();
	wall_coeffs_.clear();

	detect_planes_by_eRansac(*input_, &wall_indices, false, *sorted_wall_cloud_, wall_inliers_, wall_coeffs_, 3000, 0.3, 0.5, 25);
	exportWallSegments(WallsFolder_);

	//检测顶面
	sorted_roof_cloud_.reset(new iplPointCloud <PointT>);
	roof_inliers_.clear();
	roof_coeffs_.clear();

	detect_planes_by_eRansac(*input_, &roof_indices, false, *sorted_roof_cloud_, roof_inliers_, roof_coeffs_, 2000, 0.3, 0.5, 15);
	exportRoofSegments(RoofsFolder_);

	return 0;
}

}

