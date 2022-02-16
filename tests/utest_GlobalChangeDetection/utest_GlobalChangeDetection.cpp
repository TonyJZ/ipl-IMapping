// utest_GlobalChangeDetection.cpp : Defines the entry point for the console application.
//
#include <Eigen/Dense>

#include "commonAPIs/iplstring.h"
#include "commonAPIs/iplutility.h"
#include "io/GeoTransFileIO.h"
#include "feature/FeatureIO.h"

#include "fitting/PointGroupIO.h"
#include "MapUpdate/CandidateChangeExtractor.h"
#include "MapUpdate/GlobalChangeDetection.h"
#include "MapUpdate/ChangeVoxelSegmentIO.h"


void usage(bool wait = false)
{
	printf("Global Change Detection V1.0: Appropolis Inc.\n");
	printf("usage:\n");
	printf("--create_gcm: create global change map <gcmname>  \n");
	printf("--open_gcm: open existing global change map <gcmname>  \n");
	printf("--origin: origin coordinates  <x, y, z>  \n");
	printf("--voxel_size: voxel size <sx, sy, sz>  \n");
	printf("--reliability_Th: reliability threshold <rTh>  \n");
	printf("--save_csm: out put the changed segment map <csmname>  \n");
	printf("--iccl: insert one candidate change layer <cclname>  \n");
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

	char *gcmName = NULL;
	double org[3], vsize[3];
	int reliabilityTh = 0;
	char *csmResult = NULL;
	std::vector<std::string>  cclNames;

	bool bCreate = false;
	for (; i < argc; )
	{
		if (strcmp(argv[i], "--create_gcm") == 0)
		{
			i++;
			gcmName = argv[i]; i++;
			bCreate = true;
		}
		else if (strcmp(argv[i], "--open_gcm") == 0)
		{
			i++;
			gcmName = argv[i]; i++;
			bCreate = false;
		}
		else if (strcmp(argv[i], "--reliability_Th") == 0)
		{
			i++;
			reliabilityTh = atoi(argv[i]); i++;
		}
		else if (strcmp(argv[i], "--save_csm") == 0)
		{
			i++;
			csmResult = argv[i]; i++;
		}
		else if (strcmp(argv[i], "--iccl") == 0)
		{
			i++;
			cclNames.push_back(argv[i]); i++;
		}
		else if (strcmp(argv[i], "--origin") == 0)
		{
			i++;
			org[0] = atof(argv[i]); i++;
			org[1] = atof(argv[i]); i++;
			org[2] = atof(argv[i]); i++;
		}
		else if (strcmp(argv[i], "--voxel_size") == 0)
		{
			i++;
			vsize[0] = atof(argv[i]); i++;
			vsize[1] = atof(argv[i]); i++;
			vsize[2] = atof(argv[i]); i++;
		}
		else
		{
			i++;
		}
	}

	GlobalChangeDetection<PointT> gcd;
	

	if(bCreate)
		gcd.createGCM(org, vsize);
	else
		gcd.openGCM(gcmName);

	for (size_t i = 0; i < cclNames.size(); ++i)
	{
		//向GCM中添加一层CCL
		gcd.addCandidateChangeLayer(cclNames[i]);
	}

	std::vector<ChangeVoxelMap> changeSegs;
	gcd.detect(changeSegs, reliabilityTh);

	save_ChangeVoxelSegments(csmResult, changeSegs);

	gcd.saveGCM(gcmName);


	byebye(argc == 1);

	return 0;
}

