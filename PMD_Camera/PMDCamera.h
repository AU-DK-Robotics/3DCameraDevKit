#pragma once

#include <royale.hpp>
#include <iostream>
#include <fstream>
#include <mutex>
#include <Windows.h>

#define NOMINMAX

using namespace royale;
using namespace std;

class ListenerPointCloud : public ISparsePointCloudListener
{
public:
	ListenerPointCloud();

	~ListenerPointCloud();

	void onNewData(const royale::SparsePointCloud *data) override;
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
	you should use 'get_camera_size' to see camera_index[from 0 to camera_size].
	*/
	int init_camera(size_t camera_index, size_t operate_mode = 0);
	
	// Set the listen mode of camera
	/*
	belows are the meaning of data_mode.
	0: retrieve depth image
	1: retrieve pointcloud
	*/
	int set_camera_data_mode(size_t data_mode);

	// start to capture, and Listener* callbacks function will be called.
	int start_capture();

	// It should be stopped after use. Otherwise, it will result in failure to start again.
	int stop_capture();

private:

	ListenerDepth m_listener_depth;

	ListenerPointCloud m_listener_point_cloud;

	CameraManager m_camera_manager;

	std::unique_ptr<ICameraDevice> m_camera_device;

	royale::Vector<royale::String> m_camera_list;
};

