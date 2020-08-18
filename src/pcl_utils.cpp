#include <pcl_utils.hpp>
namespace pcl_utils
{
pcl::PointCloud<pcl::PointXYZI>::Ptr bin_to_pcd(std::string input_path, std::string out_path)
{
  std::fstream input(input_path.c_str(), std::ios::in | std::ios::binary);
  if (!input.good())
  {
    std::cerr << "Couldn't read the file " << input_path << std::endl;
    return NULL;
  }
  input.seekg(0, std::ios::beg);
  pcl::PointCloud<pcl::PointXYZI>::Ptr output(new pcl::PointCloud<pcl::PointXYZI>);
  int idx = 0;
  while (input.good() && !input.eof())
  {
    pcl::PointXYZI point;
    input.read((char *)&point.x, 3 * sizeof(float));
    input.read((char *)&point.intensity, sizeof(float));
    output->push_back(point);
    idx++;
  }
  input.close();
  cout << "Read KTTI point cloud with " << idx << " points, writing to " << out_path << endl;
  pcl::PCDWriter writer;
  writer.write(out_path, *output, false);
  return output;
}
/**
 * Find the transforamtion between two point clouds
 * and transfrom the source cloud to target cloud.
 * @param cloud1 target pointcloud.
 * @param cloud2 source pointcloud.
 * @param aligend_cloud source point cloud transformed in the target pointcloud frame.
 * @param max_dist maximum distance threshold between two correspondent points in source <-> target.
 * @param max_iter the maximum number of iterations the internal optimization should run for.
 * @param tf_epsilon maximum allowable translation squared difference between two consecutive transformations.
 * @param ed_epislon maximum allowed Euclidean error between two consecutive steps in the ICP loop, before the algorithm
 * is considered to have converged.
 * @return The final transformation matrix estimated between source and target clouds.
 */
Eigen::Matrix4f icp(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr aligend_cloud, double max_dist, int max_iter,
                    double tf_epsilon, double ed_epislon)
{
  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setInputSource(cloud2);
  icp.setInputTarget(cloud1);
  icp.setMaxCorrespondenceDistance(max_dist);
  icp.setMaximumIterations(max_iter);
  icp.setTransformationEpsilon(tf_epsilon);
  icp.setEuclideanFitnessEpsilon(ed_epislon);
  icp.align(*aligend_cloud);
  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;
  *aligend_cloud += *cloud1;
  return icp.getFinalTransformation();
}

}  // namespace pcl_utils