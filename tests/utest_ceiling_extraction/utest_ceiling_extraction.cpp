// utest_ceiling_extraction.cpp : Defines the entry point for the console application.
//
// utest_iplclassifier.cpp : Defines the entry point for the console application.
//

//#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

#include "classifier/pcClassifyWallCeiling.h"
#include "core/iplcore.h"
#include "commonAPIs/iplutility.h"
#include "core/iplstd.h"
#include "io/PointcloudIO.h"

#include "fitting/geoModelDef.h"
#include "fitting/pcPlaneDetection.h"
#include "fitting/ModelParamsIO.h"

void usage(bool wait = false)
{
	printf("indoor pointcloud clustering V1.0: Appropolis Inc.\n");
	printf("usage:\n");
	printf("utest_clustering input.pcd output_dir\n");
	printf("---------------------------------------------\n");
	if (wait)
	{
		printf("<press ENTER>\n");
		getc(stdin);
	}
	exit(1);
}

static void byebye(bool wait = false)
{
	if (wait)
	{
		fprintf(stderr, "<press ENTER>\n");
		getc(stdin);
	}
	/*	exit(1);*/
}
using namespace ipl;
using namespace std;

typedef iplPointXYZRGBNormal PointT;
int main(int argc, char * argv[])
{
	if (argc < 4)
	{
		usage();
	}

	//	assert(false);
	if (strcmp(argv[1], "-h") == 0)
	{
		usage();
	}

	int i = 1;
	char *pInputFileName = argv[i]; i++;
	char *pOutDir = argv[i]; i++;

	bool doRansac = false;

	for (; i < argc; i++)
	{
		if (strcmp(argv[i], "--doRANSAC") == 0)
			doRansac = true;
	}

	//pcl::PCDReader pcdReader;
	if (doRansac) {

		ipl::create_folder(/*pOutDir*/pOutDir);

		printf("plane detection by efficient RANSAC: \n");

		//plane detection
		iplPointCloud<PointT>::Ptr cloud(new iplPointCloud <PointT>);

		//pcdReader.read(pWallFileName, *cloud);
		read_PointCloud(pInputFileName, *cloud);
		std::vector<int> indices;
		indices.resize(cloud->size());

		for (size_t i = 0; i < indices.size(); ++i) { indices[i] = static_cast<int>(i); }

		//1. extract ceiling points
		std::vector<std::vector<int>> cls_indices;
		extract_ceiling_by_normal(cloud.get(), indices, cls_indices);

//		write_PointCloud("D:/iplTestData/TargetLocalization/target_dataset1/result/ceiling_pts.pcd", *cloud, cls_indices[0], true);

		//2. detect planes by RANSAC
		iplPointCloud<PointT>::Ptr sort_cloud(new iplPointCloud <PointT>);
		std::vector<std::vector<int> > inliers;
		std::vector<ipl::geoModel::geoModelInfo> planeCoeffs;

		//plane_detection_CGAL(*cloud, NULL, true, sort_cloud.get(), inliers, planeCoeffs);
		detect_planes_by_eRansac(*cloud, &cls_indices[0], true, *sort_cloud, inliers, planeCoeffs);

		int iModel = 0;

		//pcl::PCDWriter writer;
		for (int i = 0; i < inliers.size(); i++)
		{
			if (inliers[i].size() == 0)
				continue;

			//accept
			std::string out_name, out_param;
			char buf[32], buf_para[32];
			sprintf(buf, "ceiling_model_%04d.pcd", iModel);
			sprintf(buf_para, "ceiling_model_%04d%s", iModel, geoModel::ModelParamFile_Suffix);

			out_name = /*pOutDir*/pOutDir;
			out_name += "/";
			//out_name += result_name;
			out_name += buf;

			out_param = /*pOutDir*/pOutDir;
			out_param += "/";
			//out_param += result_name;
			out_param += buf_para;

			//writer.write<PointT>(out_name, *sort_cloud, inliers[i], true);
			write_PointCloud(out_name, *sort_cloud, inliers[i], true);

			Eigen::Vector4f bbmin, bbmax;
			pcl::getMinMax3D(*sort_cloud, inliers[i], bbmin, bbmax);
			double min_pt[3], max_pt[3];
			min_pt[0] = bbmin[0]; min_pt[1] = bbmin[1]; min_pt[2] = bbmin[2];
			max_pt[0] = bbmax[0]; max_pt[1] = bbmax[1]; max_pt[2] = bbmax[2];
			//Save_ParamFile(out_param.c_str(), sMT_plane, planeCoeffs[i]);
			memcpy(planeCoeffs[i].bbox.min_pt, min_pt, sizeof(double) * 3);
			memcpy(planeCoeffs[i].bbox.max_pt, max_pt, sizeof(double) * 3);
			save_geoModel_params(out_param.c_str(), /*min_pt, max_pt, */planeCoeffs[i]);

			iModel++;
		}
	}


	byebye(argc == 1);
	return 0;
}


