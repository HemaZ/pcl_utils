#include <pcl_utils.hpp>

int main() {

  std::string input = "../data/0000000000.bin";
  std::string output = "../0000000000.pcd";

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud =
      pcl_utils::bin_to_pcd(input, output);
  pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
  viewer.showCloud(cloud);
  while (!viewer.wasStopped()) {
  }
  return 0;
}