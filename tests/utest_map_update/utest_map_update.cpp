// utest_map_update.cpp : Defines the entry point for the console application.
//
#include <Eigen/Dense>

#include "commonAPIs/iplstring.h"
#include "commonAPIs/iplutility.h"
#include "io/GeoTransFileIO.h"
#include "feature/FeatureIO.h"

#include "fitting/PointGroupIO.h"
#include "MapUpdate/MapUpdater.h"

void usage(bool wait = false)
{
	printf("Map updater V1.0: Appropolis Inc.\n");
	printf("usage:\n");
	printf("--iCSM <filename>:  load csm \n");
	printf("--oDir <dir>: output results\n");
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

typedef pcl::PointXYZRGBNormal PointT;

using namespace ipl;
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

	char *csmName = NULL;
	char *output_dir = NULL;

	for (; i < argc; )
	{
		if (strcmp(argv[i], "--iCSM") == 0)
		{
			i++;
			csmName = argv[i]; i++;
		}
		else if (strcmp(argv[i], "--oDir") == 0)
		{
			i++;
			output_dir = argv[i]; i++;
		}
		else
		{
			i++;
		}
	}

	MapUpdater<PointT>   updater;

	updater.loadCSM(csmName);
	updater.refineSegments();

	updater.saveResults(output_dir);

	byebye(argc == 1);

	return 0;
}



