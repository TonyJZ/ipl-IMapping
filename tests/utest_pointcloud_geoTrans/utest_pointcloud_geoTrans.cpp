// utest_pointcloud_geoTrans.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"

#include "core/iplcore.h"
#include "io/PointcloudIO.h"
#include "io/GeoTransFileIO.h"
#include "registration/pointcloudGeoTransformation.h"

typedef pcl::PointXYZRGBNormal PointT;
int main(int argc, char * argv[])
{
	int i = 1;
	char *pInputPointCloud = argv[i]; i++;
	char *pOutPointCloud = argv[i]; i++;
	char *pTransFile = argv[i]; i++;

	ipl::ref_ptr<ipl::iplPointCloud<PointT> > cloud_in(new pcl::PointCloud <PointT>);
	ipl::ref_ptr<ipl::iplPointCloud<PointT> > cloud_out(new pcl::PointCloud <PointT>);
	
	ipl::read_PointCloud(pInputPointCloud, *cloud_in);

	double org[3];
	Eigen::Matrix4d trans_param;
	double rms;
	ipl::load_RigidTransParam(pTransFile, /*org, */trans_param, rms);

	ipl::pcTransformAffine(*cloud_in, *cloud_out, /*org, */trans_param);

	ipl::write_PointCloud(pOutPointCloud, *cloud_out, true);

    return 0;
}

