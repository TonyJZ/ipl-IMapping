// utest_PointCloud3DTo2D.cpp : Defines the entry point for the console application.
//
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
//	char *pTransFile = argv[i]; i++;

	ipl::ref_ptr<ipl::iplPointCloud<PointT> > cloud(new pcl::PointCloud <PointT>);
//	ipl::ref_ptr<ipl::iplPointCloud<PointT> > cloud_out(new pcl::PointCloud <PointT>);

	ipl::read_PointCloud(pInputPointCloud, *cloud);
	for (int i = 0; i < cloud->size(); ++i)
	{
		cloud->points[i].z = 0;
	}

	ipl::write_PointCloud(pOutPointCloud, *cloud, true);
	return 0;
}

