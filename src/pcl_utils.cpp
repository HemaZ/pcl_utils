#include <pcl_utils.hpp>
namespace pcl_utils {

pcl::PointCloud<pcl::PointXYZI>::Ptr bin_to_pcd(std::string input_path,
                                                std::string out_path) {
  std::fstream input(input_path.c_str(), std::ios::in | std::ios::binary);
  if (!input.good()) {
    std::cerr << "Couldn't read the file " << input_path << std::endl;
    return NULL;
  }
  input.seekg(0, std::ios::beg);
  pcl::PointCloud<pcl::PointXYZI>::Ptr output(
      new pcl::PointCloud<pcl::PointXYZI>);
  int idx = 0;
  while (input.good() && !input.eof()) {
    pcl::PointXYZI point;
    input.read((char *)&point.x, 3 * sizeof(float));
    input.read((char *)&point.intensity, sizeof(float));
    output->push_back(point);
    idx++;
  }
  input.close();
  cout << "Read KTTI point cloud with " << idx << " points, writing to "
       << out_path << endl;
  pcl::PCDWriter writer;
  writer.write(out_path, *output, false);
  return output;
}

Eigen::Matrix4f icp(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr aligend_cloud,
                    double max_dist, int max_iter, double tf_epsilon,
                    double ed_epislon) {

  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setInputSource(cloud1);
  icp.setInputTarget(cloud2);
  icp.setMaxCorrespondenceDistance(max_dist);
  icp.setMaximumIterations(max_iter);
  icp.setTransformationEpsilon(tf_epsilon);
  icp.setEuclideanFitnessEpsilon(ed_epislon);
  icp.align(*aligend_cloud);
  std::cout << "has converged:" << icp.hasConverged()
            << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  return icp.getFinalTransformation();
}

} // namespace pcl_utils