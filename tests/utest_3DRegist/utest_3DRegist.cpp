// utest_3DRegist.cpp : Defines the entry point for the console application.
//

#include <Eigen/Dense>

#include "commonAPIs/iplstring.h"
#include "commonAPIs/iplutility.h"
#include "io/GeoTransFileIO.h"
#include "feature/FeatureIO.h"
#include "registration/fine_registration.h"

void usage(bool wait = false)
{
	printf("indoor registration V1.0: Appropolis Inc.\n");
	printf("usage:\n");
	printf("refwall_dir refceiling_dir tarwall_dir tarceiling_dir suffix init_trans_file final_trans_file --search_radius 2.0 --resampling_rate 0.1\n");
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
	if (argc < 8)
	{
		usage();
	}

	//	assert(false);
	if (strcmp(argv[1], "-h") == 0)
	{
		usage();
	}

	int i = 1;
	char *ref_wall_dir = argv[i]; i++;
	char *ref_ceiling_dir = argv[i]; i++;

	char *tar_wall_dir = argv[i]; i++;
	char *tar_ceiling_dir = argv[i]; i++;

	char *suffix = argv[i]; i++;

	char *init_trans_name = argv[i]; i++;
	char *final_trans_name = argv[i]; i++;

	
	float radius = 10.0;
	float resampling_rate = 0.2;
//	double init_mTh = 0.2;

// 	bool doICP = false;
// 	bool doTrans = false;
	for (; i < argc; i++)
	{
		if (strcmp(argv[i], "--search_radius") == 0)
		{
			i++;
			radius = atof(argv[i]);
		}
		else if (strcmp(argv[i], "--resampling_rate") == 0)
		{
			i++;
			resampling_rate = atof(argv[i]);
		}
// 		else if (strcmp(argv[i], "--doTrans") == 0)
// 		{
// 			doTrans = true;
// 			/*i++;*/
// 		}
	}

	Eigen::Affine3d init_trans, final_trans;
	double norm_org[3];

	bool bRet;

	double min_pt[3], max_pt[3];
	
	std::vector<std::string> refWalls, refCeilings;
	std::vector<std::string> tarWalls, tarCeilings;

	ipl::scan_files(ref_wall_dir, suffix, refWalls);
	ipl::scan_files(ref_ceiling_dir, suffix, refCeilings);

	ipl::scan_files(tar_wall_dir, suffix, tarWalls);
	ipl::scan_files(tar_ceiling_dir, suffix, tarCeilings);

	ipl::ipcFineRegistering  icpReg;

	icpReg.set_refData(refCeilings, refWalls);
	icpReg.set_tarData(tarCeilings, tarWalls);
	icpReg.set_initTransformation(init_trans_name);
	icpReg.set_search_radius(radius);
	icpReg.set_sampling_rate(resampling_rate);

//	icpReg.ceiling_switch(false);
//	icpReg.set_debug_path(std::string("D:/iplTestData/TargetLocalization/mapupdate/T4/result/debug"));

//	double rms = icpReg.do_ICP_PointToPoint();
	double rms = icpReg.do_ICP_PointToPlane();

	icpReg.get_init_transform(init_trans);
	icpReg.get_final_transform(final_trans);

	if (final_trans_name)
		ipl::save_RigidTransParam(final_trans_name, final_trans.matrix(), rms);

	byebye(argc == 1);
	return 0;
}

