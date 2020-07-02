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
} // namespace pcl_utils