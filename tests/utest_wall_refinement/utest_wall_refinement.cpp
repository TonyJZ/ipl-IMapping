// utest_wall_refinement.cpp : Defines the entry point for the console application.
//

#include <core/iplcore.h>
#include <classifier/WallRefinement.h>

void usage(bool wait = false)
{
	printf("wall refinement V1.0: Appropolis Inc.\n");
	printf("usage:\n");
	printf("utest_wall_refinement input_dir file_ext filtered_dir \n");
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
	//	char *pWallFileName = argv[i]; i++;
	char *pInputDir = argv[i]; i++;
	char *pExterntion = argv[i]; i++;
	char *pRefinedDir = argv[i]; i++;

	ipl::create_folder(pRefinedDir);

	float samInterval = 0.02; //interval of point cloud
	float voxelSize = 0.3;    //thickness of walls

	ipl::WallRefinement<PointT> wall_refinement(samInterval/*, voxelSize*/);

	wall_refinement.loadCandidateWalls(pInputDir, pExterntion);

	wall_refinement.reinitialize();

	double dc = 0.1;
	double lowrhoTh, highrhoTh;  //用来过滤低密度点(非稳定的墙面)
	double vSize = 0.02; //原始数据的voxel size
	double lowZTh = 0.3, highZTh = 1.5; //可接受的最小墙面高

	wall_refinement.removeUncertainWalls(2.0, 0.2);

	wall_refinement.mergeWalls(voxelSize);

	wall_refinement.exportAcceptedWalls(pRefinedDir, pExterntion);

	byebye(argc == 1);
	return 0;
}

