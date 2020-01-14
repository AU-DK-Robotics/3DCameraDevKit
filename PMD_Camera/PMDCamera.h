#pragma once

#include <pcl/common/common_headers.h>

#include <royale.hpp>
#include <iostream>
#include <fstream>
#include <mutex>
#include <Windows.h>

#define NOMINMAX

using namespace royale;
using namespace std;

struct pointXYZ
{
	pointXYZ(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
	float x;
	float y;
	float z;
};

class ListenerPointCloud : public ISparsePointCloudListener
{
public:
	ListenerPointCloud();

	~ListenerPointCloud();

	void onNewData(const royale::SparsePointCloud *data) override;

	pcl::PointCloud<pcl::PointXYZ>::Ptr get_cloud_ptr() const;

	std::vector<pointXYZ> get_points()const;

private:

	// Save point cloud in royale style
	void save_royale_xyzcPoints(const royale::SparsePointCloud * data, const string& filename, float write_confidence = 0.2);

	// Save current frame point cloud to class member
	void save_royale_xyzcPoints(const royale::SparsePointCloud * data, vector<pointXYZ>&points, float write_confidence = 0.0);

	pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_ptr;

	std::vector<pointXYZ> m_points;
};

class ListenerDepth : public IDepthImageListener
{
public:
	ListenerDepth();

	~ListenerDepth();

	void onNewData(const royale::DepthImage *data) override;
};

class PMDCamera
{
public:
	PMDCamera();

	~PMDCamera();

	int get_camera_size(size_t &camera_size);

	// init the specific camera
	/*
	\param[in] camera_index before it, you have to know which camera you want to initialize
	\param[in] operate_mode See "Getting_Started_CamBoard_pico_monstar.pdf"

	0: indoor room reconstruction, MODE_9_5FPS_1900 
	1: Room scanning indoor navigation, MODE_9_10FPS_900
	2: 3D object reconstruction, MODE_9_15FPS_600
	3: Medium size object recongnition or face reconstruction, MODE_9_25FPS_300
	4: Remote collaboration, MODE_5_35PFS_500
	5: Small object, MODe_5_45FPS_400
	6: Hand tracking, MODE_5_60FPS

	you should use 'get_camera_size' to see camera_index[from 0 to camera_size].
	*/
	int init_camera(size_t camera_index, size_t operate_mode = 5);
	
	// Set the listen mode of camera
	/*
	belows are the meaning of data_mode.
	0: retrieve pointcloud
	1: retrieve depth image
	*/
	int set_camera_data_mode(size_t data_mode);

	// start to capture, and Listener* callbacks function will be called.
	int start_capture();

	// It should be stopped after use. Otherwise, it will result in failure to start again.
	int stop_capture();

	pcl::PointCloud<pcl::PointXYZ>::Ptr get_cloud_ptr() const;

private:

	ListenerDepth m_listener_depth;

	ListenerPointCloud m_listener_point_cloud;

	CameraManager m_camera_manager;

	std::unique_ptr<ICameraDevice> m_camera_device;

	royale::Vector<royale::String> m_camera_list;

};

