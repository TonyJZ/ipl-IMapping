
//#include "stdafx.h"
#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>

#include "registration/ipGraphMatching.h"
#include "registration/fine_registration.h"

#include "commonAPIs/iplstring.h"
#include "io/GeoTransFileIO.h"

void usage(bool wait = false)
{
	printf("2D registration: Appropolis Inc.\n");
	printf("usage:\n");
	printf("ref.ipf target.ipf trans_file --search_radius 10.0 \n");
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

double intersection_graph_match(const char *ref_g_name, const char *unreg_g_name, float buf_radius, double mTh,
	Eigen::Affine3d &init_trans, double norm_org[3])
{
	ipl::ipGraphMatching ip_matcher;


	ip_matcher.set_ipf(ref_g_name, unreg_g_name);
	ip_matcher.set_search_radius(buf_radius);

	double bestM_cost = std::numeric_limits<double>::max();
	Eigen::Affine3d bestM_trans = Eigen::Affine3d::Identity();
	init_trans = bestM_trans;

	if (!ip_matcher.start_match())
		return bestM_cost;

	int iter = 0;
	//while (ip_matcher.match_next())
	while (ip_matcher.weighted_match_next())
	{
		double cost = ip_matcher.get_match_cost();

		printf("iter: %d, cost: %f \n", iter++, cost);

		if (cost < bestM_cost)
		{
			bestM_cost = cost;
			bestM_trans = ip_matcher.get_transform();
		}

		if (bestM_cost < mTh)
			break;
	}

	ip_matcher.get_normalized_org(norm_org);
	init_trans = bestM_trans;
	return bestM_cost;
}

typedef pcl::PointXYZRGBNormal PointT;

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
	char *ref_ipf_name = argv[i]; i++;
	char *tar_ipf_name = argv[i]; i++;
	char *trans_name = argv[i]; i++;
	char *ref_ceiling_name = NULL;
	char *ref_wall_name = NULL;
	char *tar_ceiling_name = NULL;
	char *tar_wall_name = NULL;

	float init_r = 10.0;
	double init_mTh = 0.01;

	bool doICP = false;
	bool doTrans = false;
	for (; i < argc; i++)
	{
		if (strcmp(argv[i], "--search_radius") == 0)
		{
			i++;
			init_r = atof(argv[i]);
		}
		else if (strcmp(argv[i], "--doICP") == 0)
		{
			doICP = true;
			i++;
			ref_ceiling_name = argv[i]; i++;
			ref_wall_name = argv[i]; i++;
			tar_ceiling_name = argv[i]; i++;
			tar_wall_name = argv[i]; /*i++;*/
		}
		else if (strcmp(argv[i], "--doTrans") == 0)
		{
			doTrans = true;
			/*i++;*/
		}
	}

	Eigen::Affine3d init_trans;
	double norm_org[3];

	printf("intersection graph matching: \n");
	double mCost = intersection_graph_match(ref_ipf_name, tar_ipf_name, init_r, init_mTh, init_trans, norm_org);

	if (trans_name)
		ipl::save_RigidTransParam(trans_name, /*norm_org, */init_trans.matrix(), mCost);

	byebye(argc == 1);
	return 0;
}

