// utest_ProfileExtraction.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include <core/iplcore.h>
#include <core/iplstd.h>
#include <commonAPIs/iplutility.h>
#include "io/PointcloudIO.h"

#include "classifier/ProfileExtraction.h"

void usage(bool wait = false)
{
	printf("profile extraction V1.0: Appropolis Inc.\n");
	printf("usage:\n");
	printf("utest_ProfileExtraction input.pcd output_dir\n");
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

typedef ipl::iplPointXYZRGBNormal PointT;
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
	char *pUAVPointCloud = argv[i]; i++;
	// 	char *pwall_ext = argv[i]; i++;
	// 	char *pnonwall = argv[i]; i++;
	char *pOutputDir = argv[i]; i++;
	
//	double floor_hei = 5.0;
	double zmin, zmax;
	for (; i < argc; i++)
	{
		if (strcmp(argv[i], "-zmin") == 0 ) {
			i++;
			zmin = atof(argv[i]);
		}
		else if (strcmp(argv[i], "-zmax") == 0) {
			i++;
			zmax = atof(argv[i]);
		}
	}

	ipl::create_folder(pOutputDir);

	ipl::iplPointCloud<PointT>::Ptr cloud(new pcl::PointCloud <PointT>);
	ipl::read_PointCloud(pUAVPointCloud, *cloud);

	std::vector <int> indices;
	ipl::ProfileExtraction<PointT> pe;

	pe.setInputCloud(cloud);
	pe.extractHeightProfile(zmin, zmax, indices);

	std::string out_name, out_param;
	char buf[32], buf_para[32];
	sprintf(buf, "profile.pcd");
	//		sprintf(buf_para, "wall_model_%04d%s", ic, geoModel::ModelParamFile_Suffix);

	out_name = /*pOutDir*/pOutputDir;
	out_name += "/";
	//out_name += result_name;
	out_name += buf;

	//writer.write<PointT>(out_name, *sort_cloud, inliers[i], true);
	ipl::write_PointCloud(out_name, *cloud, indices, true);


	byebye(argc == 1);

	return 0;
}

