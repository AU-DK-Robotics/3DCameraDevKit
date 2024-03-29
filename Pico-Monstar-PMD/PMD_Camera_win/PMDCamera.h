#ifndef PMDCAMERA_H
#define PMDCAMERA_H

#include "../../common/common.h"

#include <Windows.h>

#include <royale.hpp>
#define NOMINMAX

extern bool SAVEPOINTCLOUD;
extern std::mutex pointcloud_mutex;

using namespace royale;

class ListenerPointCloud : public ISparsePointCloudListener
{
public:
	ListenerPointCloud();

	~ListenerPointCloud();

	void set_viewer_ptr(pcl::visualization::PCLVisualizer::Ptr viewer_ptr);

	void onNewData(const royale::SparsePointCloud *data) override;
	
	std::vector<pcl::PointCloud<PCFORMAT>::Ptr> & get_cloud_ptr();

	void set_saving_type(const std::string & type);

	void set_capture_range(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z);

	void set_directory(std::string dir);

	void set_capture_interval(float second);
private:
	void save_royale_xyzcPoints(const royale::SparsePointCloud * data);

	void filter_point_cloud();

	std::vector<pcl::PointCloud<PCFORMAT>::Ptr> m_cloud_ptr_vec;

	pcl::visualization::PCLVisualizer::Ptr m_viewer_ptr;

	size_t m_frame_count;
	float m_last_second;
	float m_interval_second;

	std::string m_saving_type;

	std::vector<float> m_capture_range;

	std::string m_directory_name;
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
	PMDCamera(pcl::visualization::PCLVisualizer::Ptr viewer_ptr);

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
	7: Mixed Mode
	8: Mixed Mode
	9: low noise extended
	10: Fast acquistion
	11: very fast acquistion

	you should use 'get_camera_size' to see camera_index[from 0 to camera_size].
	*/
	int init_camera(size_t camera_index, size_t operate_mode = 6);
	
	// Set the listen mode of camera
	/*
	belows are the meaning of data_mode.
	0: retrieve pointcloud
	1: retrieve depth image
	*/
	int set_camera_data_mode(size_t data_mode);

	// start to capture, and Listener* callbacks function will be called.
	// Before starting, it will stop the capturing anyway.
	int start_capture();

	// It should be stopped after use. Otherwise, it will result in failure to start again.
	int stop_capture();

	pcl::PointCloud<PCFORMAT>::Ptr get_visualization_cloud_ptr();

	// only "txt" or "bin"
	void set_saving_type(const std::string type);

	void set_capture_range(float min_x, float max_x, float min_y, float max_y, float min_z, float max_z);

	void set_directory(std::string directory);

	void set_capture_interval(float second);
private:

	ListenerDepth m_listener_depth;

	ListenerPointCloud m_listener_point_cloud;

	CameraManager m_camera_manager;

	std::unique_ptr<ICameraDevice> m_camera_device;

	royale::Vector<royale::String> m_camera_list;
};

#endif // PMDCAMERA_H