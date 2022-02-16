#include "registration/pointcloudGeoTransformation.h"

#include <pcl/common/transforms.h>

namespace ipl
{
	template <typename PointT> 
	void pcTransformAffine(const iplPointCloud<PointT>&cloud_in, iplPointCloud<PointT> &cloud_out,
		/*const double orgin[3], */const Eigen::Matrix4d &trans_param)
	{
// 		ref_ptr<iplPointCloud<PointT> > decentered_cloud(new iplPointCloud<PointT>);
// 
// 		size_t npts = cloud_in.size();
// 		decentered_cloud->points.resize(npts);
// 		decentered_cloud->height = 1; 
// 		decentered_cloud->width = npts;
// 		for (size_t i = 0; i < npts; i++)
// 		{
// 			decentered_cloud->points[i] = cloud_in.points[i];
// 			decentered_cloud->points[i].x -= orgin[0];
// 			decentered_cloud->points[i].y -= orgin[1];
// 			decentered_cloud->points[i].z -= orgin[2];
// 		}
		
		pcl::transformPointCloud(cloud_in, cloud_out, trans_param);

		//先选择后平移，新的原点已经记录在 Eigen::Affine3f中，不需要再平移

// 		for (size_t i = 0; i < npts; ++i)
// 		{
// 			cloud_out.points[i].x += orgin[0];
// 			cloud_out.points[i].y += orgin[1];
// 			cloud_out.points[i].z += orgin[2];
// 		}

		return ;
	}
}
