// utest_wall_extraction.cpp : Defines the entry point for the console application.
//

#include "core/iplcore.h"
#include "commonAPIs/iplutility.h"
#include "core/iplstd.h"

#include "fitting/pcPlaneDetection.h"
#include "fitting/ModelParamsIO.h"
#include "io/PointcloudIO.h"

#include <pcl/common/common.h>


void usage(bool wait = false)
{
	printf("wall extraction V1.0: Appropolis Inc.\n");
	printf("usage:\n");
	printf("utest_wall_extraction vertical_pts.pcd eRANSAC_dir filtered_dir --doRANSAC(optional)\n");
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

typedef iplPointXYZRGBNormal PointT;
//template class ptSegment<PointT>;

//#define PointT pcl::PointXYZRGBNormal
int main(int argc, char * argv[])
{
	if (argc < 3)
	{
		usage();
	}

	//	assert(false);
	if (strcmp(argv[1], "-h") == 0)
	{
		usage();
	}

	int i = 1;
	char *pWallFileName = argv[i]; i++;
	char *pRansacOutDir = argv[i]; i++;
	//	char *pFilterOutDir = argv[i]; i++;

	bool doRansac = false;

	for (; i < argc; i++)
	{
		if (strcmp(argv[i], "--doRANSAC") == 0)
			doRansac = true;
		// 		else if (strcmp(argv[i], "-z0") == 0 || strcmp(argv[i], "-Z0") == 0) {
		// 			i++;
		// 			z0 = atof(argv[i]);
		// 		}
		// 		else if (strcmp(argv[i], "-from") == 0)
		// 		{
		// 			i++;
		// 			prjFrom = argv[i];
		// 		}
		// 		else if (strcmp(argv[i], "-from") == 0)
		// 		{
		// 			i++;
		// 			prjTo = argv[i];
		// 		}
	}

	//pcl::PCDReader pcdReader;
	if (doRansac) {

		ipl::create_folder(/*pOutDir*/pRansacOutDir);

		printf("plane detection by efficient RANSAC: \n");

		//plane detection
		iplPointCloud<PointT>::Ptr cloud(new iplPointCloud <PointT>);

		//pcdReader.read(pWallFileName, *cloud);
		read_PointCloud(pWallFileName, *cloud);

		iplPointCloud<PointT>::Ptr sort_cloud(new iplPointCloud <PointT>);
		std::vector<std::vector<int> > inliers;
		std::vector<ipl::geoModel::geoModelInfo> planeCoeffs;

		//plane_detection_CGAL(*cloud, NULL, true, sort_cloud.get(), inliers, planeCoeffs);
		detect_planes_by_eRansac(*cloud, NULL, true, *sort_cloud, inliers, planeCoeffs);

		int iModel = 0;

		//pcl::PCDWriter writer;
		for (int i = 0; i < inliers.size(); i++)
		{
			if (inliers[i].size() == 0)
				continue;

			//accept
			std::string out_name, out_param;
			char buf[32], buf_para[32];
			sprintf(buf, "wall_model_%04d.pcd", iModel);
			sprintf(buf_para, "wall_model_%04d%s", iModel, geoModel::ModelParamFile_Suffix);

			out_name = /*pOutDir*/pRansacOutDir;
			out_name += "/";
			//out_name += result_name;
			out_name += buf;

			out_param = /*pOutDir*/pRansacOutDir;
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
			save_geoModel_params(out_param.c_str(), /*min_pt, max_pt,*/ planeCoeffs[i]);

			iModel++;
		}
	}


	byebye(argc == 1);
	return 0;
}

