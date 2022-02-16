// utest_BuildingBoundaryReconstruction.cpp : Defines the entry point for the console application.
//

#include "core/iplcore.h"
#include "core/iplstd.h"
#include "commonAPIs/iplutility.h"
#include "commonAPIs/iplstring.h"
#include "io/PointcloudIO.h"

//#include "classifier/BuildingDetection.h"
#include "reconstruction/BoundaryReconstruction.h"

void usage(bool wait = false)
{
	printf("boundary reconstruction V1.0: Appropolis Inc.\n");
	printf("usage:\n");
	printf("utest_BuildingBoundaryReconstruction input.pcd output_dir\n");
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
	char *pCloudFolder = argv[i]; i++;
	char *suffix = argv[i]; i++;
	// 	char *pwall_ext = argv[i]; i++;
	// 	char *pnonwall = argv[i]; i++;
	char *pOutputDir = argv[i]; i++;

	for (; i < argc; i++)
	{
		;
	}

	std::vector<std::string> buildingList;
	ipl::scan_files(pCloudFolder, suffix, buildingList);

	int nSegs = buildingList.size();
	for (int i = 0; i < nSegs; ++i)
	{
		std::string result_name, suffix_name;
		ipl::extract_file_name(buildingList[i], result_name, suffix_name);
		
		ipl::iplPointCloud<PointT>::Ptr cloud(new pcl::PointCloud <PointT>);
		ipl::read_PointCloud(buildingList[i], *cloud);

		ipl::BoundaryReconstruction<PointT> BR;
		BR.setIntermediateFolder(result_name);
		BR.setInputCloud(cloud);
		BR.setZProfileRatio(0.7);

		BR.extractAlphaShape(0.5, 0.5);

//		BR.detectPlanes(1.5, 0.5);
	}


	byebye(argc == 1);

	return 0;
}


