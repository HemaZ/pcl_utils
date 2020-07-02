#include <fstream>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>
namespace pcl_utils {

pcl::PointCloud<pcl::PointXYZI>::Ptr bin_to_pcd(std::string input_path,
                                                std::string out_path);
}
