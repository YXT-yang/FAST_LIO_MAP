#include "map-Maintain/map_Maintain.hpp"
#include <pcl/io/pcd_io.h>
using namespace lo_space;

int main(int argc, char **argv)
{
  PointCloudPtr cloud_initial(new PointCloudXYZI);
  PointCloudPtr cloud_increment(new PointCloudXYZI);
  pcl::io::loadPCDFile<PointType>("/home/whu/ThirdLib/MASS/FAST-LIO-MAP/data/initial.pcd", *cloud_initial);
  pcl::io::loadPCDFile<PointType>("/home/whu/ThirdLib/MASS/FAST-LIO-MAP/data/increment.pcd", *cloud_increment);

  std::cout << "cloud_initial: " << cloud_initial->size() << "\n";
  std::cout << "cloud_increment: " << cloud_increment->size() << "\n";

  V3D pose1(.0, .0, .0);
  V3D pose2(.0, .0, 10.0);

  MapMaintain::Ptr mapMantain = std::make_shared<MapMaintain>();
  mapMantain->MapFovSegment(pose1);
  mapMantain->IkdTreeInitial(cloud_initial);

  mapMantain->MapFovSegment(pose1);
  mapMantain->SearchPointCloudInTree(cloud_initial);
  mapMantain->MapIncrement();
  std::cout << "getAddPointsNumLate: " << mapMantain->getAddPointsNumLate() << "\n";

  mapMantain->MapFovSegment(pose2);
  mapMantain->SearchPointCloudInTree(cloud_increment);
  mapMantain->MapIncrement();
  std::cout << "getAddPointsNumLate: " << mapMantain->getAddPointsNumLate() << "\n";

  std::cout
      << "测试运行完毕!" << "\n";
  return 0;
}