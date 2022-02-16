// utest_BoundaryOptimization.cpp : Defines the entry point for the console application.
//

#include "core/iplcore.h"
#include "core/iplstd.h"
#include "commonAPIs/iplutility.h"
#include "commonAPIs/iplstring.h"
#include "io/PointcloudIO.h"

//#include "classifier/BuildingDetection.h"
#include "reconstruction/BoundaryOptimization.h"


void usage(bool wait = false)
{
	printf("boundary optimization V1.0: Appropolis Inc.\n");
	printf("usage:\n");
	printf("utest_BoundaryOptimization input_dir output_dir\n");
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
	char *pInputDir = argv[i]; i++;
	//char *suffix = argv[i]; i++;
	// 	char *pwall_ext = argv[i]; i++;
	// 	char *pnonwall = argv[i]; i++;
	char *pOutputDir = argv[i]; i++;

	for (; i < argc; i++)
	{
		;
	}

	ipl::BoundaryOptimization<PointT>  BO;
	std::string filename;

	BO.initialize(pInputDir, pOutputDir);
	BO.createLayout2D();

	filename = pOutputDir;
	filename += "/opt_initLayout.shp";
	BO.exportAllPolygons(filename, true);

	BO.computeAttributesForArrangement(1.0, 2.0);

	double inner_edge_Th_ = 2.0; //内部边大于2.0m的保留
	double polygon_area_Th_ = 6.0;
	double border_edge_Th_ = 4.0;
	
	BO.optimize(/*inner_edge_Th_, border_edge_Th_,*/ polygon_area_Th_);

	filename = pOutputDir;
	filename += "/opt_allpolygons.shp";
	BO.exportAllPolygons(filename, true);


    return 0;
}

