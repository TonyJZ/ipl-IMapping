// utest_BuildingDetection.cpp : Defines the entry point for the console application.
//
#include <core/iplcore.h>
#include <core/iplstd.h>
#include <commonAPIs/iplutility.h>
#include "io/PointcloudIO.h"

#include "reconstruction/BuildingDetection.h"

void usage(bool wait = false)
{
	printf("building detection V1.0: Appropolis Inc.\n");
	printf("usage:\n");
	printf("utest_BuildingDetection input.pcd output_dir\n");
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

	double floor_hei = 5.0;

	ipl::create_folder(pOutputDir);

	for (; i < argc; i++)
	{
		;
	}

	ipl::iplPointCloud<PointT>::Ptr cloud(new pcl::PointCloud <PointT>);
	ipl::read_PointCloud(pUAVPointCloud, *cloud);

	std::vector<std::vector <int> > indices;
	ipl::BuildingDetection<PointT> BD;

	BD.setInputCloud(cloud);

	// 	char profile_name[] = "D:/iplTestData/mappingExperiment/uav_pointcloud/shanghai3/profile/profile55-100.pcd";
	// 	BD.exportHeightSection(profile_name, 55, 100);

	BD.Partition(1.0);

	BD.DetectEruptions(51.5/*最小高度*/, 2.0/*突出物高差*/, 80/*最小面积*/);

	std::string buildingFolder = pOutputDir;
	buildingFolder += "/building";

	BD.exportBuildingSegments(buildingFolder);

    /*
// 	BD.SegmentBuildings();
// 	BD.exportBuildingSegments(buildingFolder);
    */

	byebye(argc == 1);

	return 0;
}

