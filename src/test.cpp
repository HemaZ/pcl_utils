#include <pcl_utils.hpp>

int main() {

  std::string input1 = "../data/0000000000.bin";
  std::string input2 = "../data/0000000006.bin";
  std::string output1 = "../0000000000.pcd";
  std::string output2 = "../0000000006.pcd";

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1 =
      pcl_utils::bin_to_pcd(input1, output1);

  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2 =
      pcl_utils::bin_to_pcd(input2, output2);

  pcl::PointCloud<pcl::PointXYZI>::Ptr aligend_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  Eigen::Matrix4f tf = pcl_utils::icp(cloud1, cloud2, aligend_cloud);
  // pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
  pcl::visualization::PCLVisualizer viewer("Simple Cloud Viewer");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> colors1(
      cloud1, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> colors2(
      cloud2, 0, 255, 0);
  viewer.addPointCloud<pcl::PointXYZI>(cloud1, colors1, "Cloud1");
  viewer.addPointCloud<pcl::PointXYZI>(aligend_cloud, colors2, "cloud2");

  while (!viewer.wasStopped()) {
    viewer.spinOnce();
  }
  return 0;
}