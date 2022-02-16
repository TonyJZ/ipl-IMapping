// utest_floormapRec.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"

//#include <pcl/io/pcd_io.h>
#include "io/PointcloudIO.h"
#include "reconstruction/FloormapReconstruction.h"

#include <gdal/gdal_alg.h>
#include <gdal/gdal_priv.h>   //添加GDAL库函数的头文件
#include <gdal/ogrsf_frmts.h>

void usage(bool wait = false)
{
	printf("floor map reconstruction V1.0: Appropolis Inc.\n");
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
	char *pwall_dir = argv[i]; i++;
	char *pwall_ext = argv[i]; i++;
	char *pnonwall = argv[i]; i++;
	char *pOutputDir = argv[i]; i++;
	
	double floor_hei = 5.0;

	ipl::create_folder(pOutputDir);

	for (; i < argc; i++)
	{
		;
	}

	std::string filename;

	ipl::FloorMapReconstruction   floor_rec;

	floor_rec.setOutputDir(pOutputDir);
	floor_rec.loadSegments(pwall_dir, pwall_ext, pnonwall);

//	floor_rec.lineArrangement();
//	floor_rec.exportLineArrangement("D:/code_indoor_location/sample_data/sample1/ref_dataset/result/linearrange.off", 5.0);

	floor_rec.createLayout(0.2, 0.2); // indoor layout extracted by graph-cut

	filename = pOutputDir;
	filename += "/cells.shp";
	floor_rec.exportAllPolygons(filename, 5);

	filename = pOutputDir;
	filename += "/polylines.shp";
	floor_rec.exportInnerPolylines(filename, floor_hei);
	
	floor_rec.extractFloormap(/*pfloorfile, floor_hei*/);

	filename = pOutputDir;
	filename += "/boundary.shp";
	floor_rec.exportOutterBoundary(filename, floor_hei);

	filename = pOutputDir;
	filename += "/all_polygons.shp";
	floor_rec.exportAllPolygons(filename, floor_hei, true);

	filename = pOutputDir;
	filename += "/inner_polylines.shp";
	floor_rec.exportInnerPolylines(filename, floor_hei);

	byebye(argc == 1);
	return 0;
}

