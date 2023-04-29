#ifndef COMMON_H
#define COMMON_H

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/auto_io.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/geometry.h>
#include <pcl/filters/passthrough.h>

#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>

#include <iomanip>
#include <iostream>
#include <fstream>
#include <mutex>
#include <chrono>
#include <thread>
#include <filesystem>
#include <limits>

typedef pcl::PointXYZI PCFORMAT;
using namespace std;

void read_parameters_from_file(const std::string filename, std::map<std::string, std::string> &parameter_list);

void add_point_cloud_visualization(pcl::visualization::PCLVisualizer::Ptr viewer_ptr, pcl::PointCloud<PCFORMAT>::Ptr m_cloud_ptr);

// need to convert before visualization
void write_point_cloud_binary(pcl::PointCloud<PCFORMAT>::Ptr m_cloud_ptr, const std::string filename);

// dont need to convert before visualization
int write_point_cloud_acsii(pcl::PointCloud<PCFORMAT>::Ptr m_cloud_ptr, const std::string filename);
int write_point_cloud_acsii(pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_ptr, const std::string filename);

std::string get_current_date(std::vector<int> &current_date);

#endif // COMMON_H