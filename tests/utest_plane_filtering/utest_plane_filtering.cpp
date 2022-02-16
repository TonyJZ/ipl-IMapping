// utest_plane_filtering.cpp : Defines the entry point for the console application.
//

#include "core/iplcore.h"
#include "core/iplstd.h"
#include "commonAPIs/iplutility.h"
#include "commonAPIs/iplstring.h"

#include "fitting/pcPlaneDetection.h"
#include "fitting/ModelParamsIO.h"
#include "io/PointcloudIO.h"
#include "feature/PlanePointDistribution.h"

#include <Eigen/Dense>
#include <pcl/common/angles.h>


void usage(bool wait = false)
{
	printf("plane filtering V1.0: Appropolis Inc.\n");
	printf("usage:\n");
	printf("utest_plane_filtering eRANSAC_dir filtered_dir \n");
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

using namespace ipl;

typedef iplPointXYZRGBNormal PointT;
//template class ptSegment<PointT>;

//#define PointT pcl::PointXYZRGBNormal
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
	char *pOutputDir = argv[i]; i++;

	float fitting_RMS_Th = 0.1;
	int  plane_number_Th = 100;
	float area_Th = 1.0;
	float sparseDeg_Th = 0.5;

	
	for (; i < argc; i++)
	{
		if (strcmp(argv[i], "--fitting_RMS") == 0)
		{
			i++;
			fitting_RMS_Th = atof(argv[i]);
		}
		else if (strcmp(argv[i], "--plane_number") == 0)
		{
			i++;
			plane_number_Th = atof(argv[i]);
		}
		else if (strcmp(argv[i], "--occupied_area") == 0)
		{
			i++;
			area_Th = atof(argv[i]);
		}
		else if (strcmp(argv[i], "--sparse_degree") == 0)
		{
			i++;
			sparseDeg_Th = atof(argv[i]);
		}
	}


	if (check_exist(pOutputDir))
		empty_folder(pOutputDir);
	else
		ipl::create_folder(pOutputDir);

	//wall, ceiling 分开分析
	float ver_degTh = 15.0, hor_degTh = 80.0;
	float verTh = cosf(pcl::deg2rad(ver_degTh));
	float horTh = cosf(pcl::deg2rad(hor_degTh));
	Eigen::Vector3f  nVertical = Eigen::Vector3f(0, 0, 1);

	std::vector<std::string> filenames;
	ipl::scan_files(pInputDir, pExterntion, filenames);

	int nSegs = filenames.size();
	if (nSegs <= 0)
	{
		std::cout << "can't find " << pExterntion << " in " << pInputDir << std::endl;
		return (-1);
	}

	std::cout << "begin to process candidate planes: " << nSegs << std::endl;
	int curptId = 0;
	for (int i = 0; i < nSegs; i++)
	{
		ref_ptr<PlanePointDistribution<PointT> >  ppDistribute(new PlanePointDistribution<PointT>);
		ref_ptr<iplPointCloud<PointT> > cloud(new iplPointCloud<PointT>);

		std::cout << "loading: " << i << "\r";
		//提取分割点云和模型参数
		std::string rname, sname;

		ipl::extract_file_name(filenames[i], rname, sname);
		std::string paraname = rname;
		paraname += /*".param"*/geoModel::ModelParamFile_Suffix;

		ipl::geoModel::geoModelInfo info;

		//		ipl::iplModelCoeffs mCoef;
		ipl::geoModel::geoModelType type;

		double bbmin[3], bbmax[3];
		load_geoModel_params(paraname, /*bbmin, bbmax,*/ info);
		if (info.type != geoModel::gMT_PLANE)
			continue;

		//		mCoef = info.coef;

		//提取分割点云
		read_PointCloud(filenames[i], *cloud);

		ppDistribute->setInputCloud(cloud);
		ppDistribute->setPlaneCoef(info);

		if(ppDistribute->getPointNum() < plane_number_Th)
			continue;
		if (ppDistribute->getPlaneRMS() > fitting_RMS_Th)
			continue;

		int flag = 0;//0: undefined; 1: vertical; 2: horizontal
		Eigen::Map<Eigen::Vector3f> plane_normal(static_cast<float*> (&info.coef.values[0]));
		float dot_product = fabsf(plane_normal.dot(nVertical));
		if (dot_product > verTh)
		{
			flag = 2;
		}
		else if (dot_product < horTh)
		{
			flag = 1;
		}
		
		if(flag == 0)
			continue;

		if (flag == 1)
		{//过滤墙面
			ppDistribute->reinitializeAAT();

			if (ppDistribute->getOccupiedArea() < area_Th)
				continue;

			if(ppDistribute->getSparseDegree() < sparseDeg_Th)
				continue;
		}
		else if (flag == 2)
		{//过滤顶面

		}

		//accept
		//提取分割模型参数
		std::string newPCName, newParamName;
		ipl::extract_pure_file_name(filenames[i], rname, sname);

		newPCName = pOutputDir;
		newPCName = newPCName +"/" + rname + sname;
		newParamName = pOutputDir;
		newParamName = newParamName + "/" + rname + geoModel::ModelParamFile_Suffix;

		ipl::copy_file(filenames[i], newPCName);
		ipl::copy_file(paraname, newParamName);
	}

	std::cout << "processing completed" << std::endl;

	byebye(argc == 1);
	return 0;
}


