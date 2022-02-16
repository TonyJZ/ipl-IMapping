// utest_CandidateChangeExtraction.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"

#include <Eigen/Dense>

#include "commonAPIs/iplstring.h"
#include "commonAPIs/iplutility.h"
#include "io/GeoTransFileIO.h"
#include "feature/FeatureIO.h"

#include "MapUpdate/CandidateChangeExtractor.h"
#include "fitting/PointGroupIO.h"

void usage(bool wait = false)
{
	printf("candidate change extraction V1.0: Appropolis Inc.\n");
	printf("usage:\n");
	printf("refwall_dird refceiling_dir tarwall_dir tarceiling_dir suffix trans_file cand_dir\n");
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

int main(int argc, char * argv[])
{
	if (argc < 6)
	{
		usage();
	}

	//	assert(false);
	if (strcmp(argv[1], "-h") == 0)
	{
		usage();
	}

	int i = 1;
	char *ref_dir = argv[i]; i++;
	//char *ref_ceiling_dir = argv[i]; i++;

	char *tar_dir = argv[i]; i++;
	//char *tar_ceiling_dir = argv[i]; i++;

	char *suffix = argv[i]; i++;
	char *trans_name = argv[i]; i++;
	
	char *cand_dir = argv[i]; i++;

	float radius = 0.1;
	
	for (; i < argc; i++)
	{
		if (strcmp(argv[i], "--distance_th") == 0)
		{
			i++;
			radius = atof(argv[i]);
		}
// 		else if (strcmp(argv[i], "--resampling_rate") == 0)
// 		{
// 			i++;
// 			resampling_rate = atof(argv[i]);
// 		}
// 		else if (strcmp(argv[i], "--doTrans") == 0)
// 		{
// 			doTrans = true;
// 			/*i++;*/
// 		}
	}

// 	Eigen::Affine3d init_trans, final_trans;
// 	double norm_org[3];

	bool bRet;

//	double min_pt[3], max_pt[3];

	std::vector<std::string> ref_files;
	std::vector<std::string> tar_files;

	ipl::scan_files(ref_dir, suffix, ref_files);
//	ipl::scan_files(ref_ceiling_dir, suffix, refCeilings);

	ipl::scan_files(tar_dir, suffix, tar_files);
//	ipl::scan_files(tar_ceiling_dir, suffix, tarCeilings);

	ipl::CandidateChangeExtractor<PointT>  cce;

	//¼ì²â
	cce.set_refData(ref_files);
	cce.set_tarData(tar_files);
	cce.set_Transformation(trans_name);
	cce.set_distance_threshold(radius);

	ipl::geoModel::PointGroup<PointT> cand_group;
	cce.extractCandidateChange(cand_group);

	//±£´æ
	ipl::save_PointGroup(cand_dir, cand_group);

	byebye(argc == 1);

    return 0;
}

