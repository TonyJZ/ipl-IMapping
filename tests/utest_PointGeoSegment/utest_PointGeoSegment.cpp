// utest_PointGeoSegment.cpp : Defines the entry point for the console application.
//
#include "core/iplcore.h"
#include "core/iplfiles.h"
#include "core/iplstd.h"

#include "io/PointcloudIO.h"
#include "fitting/pcPlaneDetection.h"
#include "fitting/ModelParamsIO.h"

#include "feature/PointGeoSegment.h"


void usage(bool wait = false)
{
	printf("plane filtering V1.0: Appropolis Inc.\n");
	printf("usage:\n");
	printf("utest_PointGeoSegment input_dir file_ext filtered_dir \n");
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
	char *pRansacOutDir = argv[i]; i++;
	char *pExterntion = argv[i]; i++;
	char *pFilterOutDir = argv[i]; i++;

	ipl::createFolder(pFilterOutDir);

	std::vector <std::string> filenames;
	//char pExterntion[] = ".pcd";

	//TraversFolderForCertainFiles(pRansacOutDir, pExterntion, filenames);
	ipl::scanFiles(pRansacOutDir, pExterntion, filenames);

	pcl::PointCloud<PointT>::Ptr seg_cloud(new pcl::PointCloud <PointT>);
	ipl::PointGeoSegment<PointT>  seg;

	double dc = 0.1;
	double lowrhoTh, highrhoTh;  //用来过滤低密度点(非稳定的墙面)
	double vSize = 0.02; //原始数据的voxel size
	double lowZTh = 0.5, highZTh = 2.0; //可接受的最小墙面高

	lowrhoTh = 2 * dc*lowZTh / (vSize*vSize);
	highrhoTh = 2 * dc*highZTh / (vSize*vSize);

	printf("plane filtering: \n");

	int nSegs = filenames.size();
	for (int i = 0; i < nSegs; i++)
	{
		//提取分割点云
		read_PointCloud(filenames[i], *seg_cloud);
		//pcdReader.read(filenames[i], *seg_cloud);
		seg.setInputCloud(seg_cloud);
		seg.reinitialize();
		double diagLen = seg.get2DDiagonal();
		int nsample = 20;
		double dens = seg.getProjectionDensity(nsample, dc);

		if (diagLen < 2.0)
			continue;

		if (diagLen < 3.0 && dens < lowrhoTh)
			continue;


		double maxConnDis;
		double connp = seg.get2DConnectivity(0.2, maxConnDis);
		if (connp < 0.5 && maxConnDis < 2.0)
			continue;


		//accept
		//提取分割模型参数
		std::string rname, sname;
		ipl::extractPureFileName(filenames[i].c_str(), rname, sname);

		std::cout << "accept: " << rname << std::endl;
		//printf("accept: %s\n", rname);

		std::string newptname = pFilterOutDir;
		newptname += '/';
		newptname += rname;
		std::string newparamname = newptname;
		newptname += ".pcd";
		newparamname += geoModel::ModelParamFile_Suffix;

		ipl::extractFileName(filenames[i].c_str(), rname, sname);
		std::string paramname = rname;
		paramname += geoModel::ModelParamFile_Suffix;

		boost::filesystem::copy_file(filenames[i].c_str(), newptname.c_str(),
			boost::filesystem::copy_option::overwrite_if_exists);
		boost::filesystem::copy_file(paramname.c_str(), newparamname.c_str(),
			boost::filesystem::copy_option::overwrite_if_exists);

	}

	


	byebye(argc == 1);
	return 0;
}

