// utest_iplclassifier.cpp : Defines the entry point for the console application.
//

//#include <pcl/io/pcd_io.h>

#include <classifier/pcClassifyWallCeiling.h>
#include <core/iplcore.h>
#include <commonAPIs/iplutility.h>
#include <core/iplstd.h>
#include <io/PointcloudIO.h>

void usage(bool wait = false)
{
	printf("indoor pointcloud clustering V1.0: Appropolis Inc.\n");
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
using namespace ipl;
using namespace std;
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
	char *pInputFileName = argv[i]; i++;
	char *pOutDir = argv[i]; i++;

	for (; i < argc; i++)
	{
		;
	}

	ipl::create_folder(pOutDir);

	ipl::iplPointCloud<ipl::iplPointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGBNormal>);
	//pcl::PCDReader Reader;
	//string filename = "D:/indoor_data/test/to_register/test2/cloud0.pcd";
	//Reader.read(pInputFileName, *cloud);
	read_PointCloud(pInputFileName, *cloud);

	std::vector<int> indices;
	indices.resize(cloud->size());

	for (size_t i = 0; i < indices.size(); ++i) { indices[i] = static_cast<int>(i); }

	//按法方向分类，提取垂直点和水平点
	std::vector<std::vector<int>> cls_indices;
	//classify_wall_ceiling_by_normal(cloud.get(), cls_indices);
	classify_vertical_horizontal_by_normal(cloud.get(), indices, cls_indices);

	//pcl::PCDWriter writer;
	std::string out_name;
	char buf[32];

	sprintf(buf, "vertical_pts.pcd");
	out_name = pOutDir;
	out_name += "/";
	out_name += buf;
	//writer.write<pcl::PointXYZRGBNormal>(out_name, *cloud, cls_indices[0], true);
	write_PointCloud(out_name, *cloud, cls_indices[0], true);

	sprintf(buf, "horizontal_pts.pcd");
	out_name = pOutDir;
	out_name += "/";
	out_name += buf;
	//writer.write<pcl::PointXYZRGBNormal>(out_name, *cloud, cls_indices[1], true);
	write_PointCloud(out_name, *cloud, cls_indices[1], true);

// 	sprintf(buf, "outlier_pts.pcd");
// 	out_name = pOutDir;
// 	out_name += "/";
// 	out_name += buf;
// 	//writer.write<pcl::PointXYZRGBNormal>(out_name, *cloud, cls_indices[2], true);
// 	write_PointCloud(out_name, *cloud, cls_indices[2], true);

// 	std::vector<int> nonWallIndices;
// 	nonWallIndices = cls_indices[1];
// 	nonWallIndices.insert(nonWallIndices.end(), cls_indices[2].begin(), cls_indices[2].end());
// 
// 	sprintf(buf, "nonwall_pts.pcd");
// 	out_name = pOutDir;
// 	out_name += "/";
// 	out_name += buf;
// 	//writer.write<pcl::PointXYZRGBNormal>(out_name, *cloud, cls_indices[2], true);
// 	write_PointCloud(out_name, *cloud, nonWallIndices, true);


	byebye(argc == 1);
	return 0;
}


