#include <pcl/io/pcd_io.h>
#include "utils/data_type.hpp"
#include <pcl/visualization/pcl_visualizer.h>

using namespace lo_space;

int main(int argc, char **argv)
{
  PointCloudXYZI cloud;
  cloud.width = 1600;
  cloud.height = 1;
  cloud.points.resize(1600 * 1);

  // 生成 xz 墙面
  int num_tab = 0;
  for (float i = -10; i < 10; i += 0.5)
  {
    for (float j = -10; j < 10; j += 1.)
    {
      cloud.points[num_tab].x = (i) + rand() / (RAND_MAX + .0F) / 5.0;
      cloud.points[num_tab].y = .0 + rand() / (RAND_MAX + .0F) / 10.0;
      cloud.points[num_tab].z = (j) + rand() / (RAND_MAX + .0F) / 2.0;
      cloud.points[num_tab].intensity = 1.0; // 使用高强度值来区分
      num_tab++;
    }
  }

  // 生成 yz 墙面
  for (float i = -10; i < 10; i += 0.5)
  {
    for (float j = -10; j < 10; j += 1.)
    {
      cloud.points[num_tab].y = (i) + rand() / (RAND_MAX + .0F) / 5.0;
      cloud.points[num_tab].x = .0 + rand() / (RAND_MAX + .0F) / 10.0;
      cloud.points[num_tab].z = (j) + rand() / (RAND_MAX + .0F) / 2.0;
      cloud.points[num_tab].intensity = 1.0; // 使用高强度值来区分
      num_tab++;
    }
  }

  PointCloudPtr cloud_ptr(new PointCloudXYZI);
  *cloud_ptr = cloud;

  pcl::io::savePCDFileASCII("/home/whu/ThirdLib/MASS/FAST-LIO-MAP/data/initial.pcd", cloud);
  pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Viewer"));
  pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormal> intensity_distribution(cloud_ptr, "intensity");
  viewer->addPointCloud<pcl::PointXYZINormal>(cloud_ptr, intensity_distribution, "mergedCloud");

  while (!viewer->wasStopped())
  {
    viewer->spinOnce();
  }

  // std::cout << rand() / (RAND_MAX + .0F) << "\n";

  return 0;
}