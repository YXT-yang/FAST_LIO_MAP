#ifndef DATA_TYPE_DATA_
#define DATA_TYPE_DATA_

#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace lo_space
{
  // Eigen
  using V3D = Eigen::Vector3d;

  // PCL
  using PointType = pcl::PointXYZINormal;
  using PointCloudXYZI = pcl::PointCloud<PointType>;
  using PointCloudPtr = PointCloudXYZI::Ptr;
}

#endif /// DATA_TYPE_DATA_