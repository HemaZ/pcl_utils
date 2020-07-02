#include <fstream>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>
namespace pcl_utils {

pcl::PointCloud<pcl::PointXYZI>::Ptr bin_to_pcd(std::string input_path,
                                                std::string out_path);

Eigen::Matrix4f icp(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud2,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr aligend_cloud,
                    double max_dist = 0.05, int max_iter = 50,
                    double tf_epsilon = 1e-8, double ed_epislon = 1);
} // namespace pcl_utils
