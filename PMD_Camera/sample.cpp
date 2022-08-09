#include "PMDCamera.h"


void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
	
	if (event.getKeySym() == "s" && event.keyDown())
	{
		SAVEPOINTCLOUD = true;
	}
}

int main(int argc, char *argv[])
{
	if (argc != 10) return -1;

	float 
		min_x = std::stof(argv[1]),
		max_x = std::stof(argv[2]),
		min_y = std::stof(argv[3]),
		max_y = std::stof(argv[4]),
		min_z = std::stof(argv[5]),
		max_z = std::stof(argv[6]);
	std::string directory = std::string(argv[7]);
	std::string save_type = std::string(argv[8]);

	// only for "auto"
	float interval_second = std::stof(argv[9]);

	size_t camera_size = 0;
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	PMDCamera pmd_camera(viewer);
	pmd_camera.get_camera_size(camera_size);
	pmd_camera.set_saving_type(save_type);
	pmd_camera.set_capture_range(min_x, max_x, min_y, max_y, min_z, max_z);
	pmd_camera.set_directory(directory);

	/*
		parameters description
	*/

	cout
		<< "detected " << camera_size << " camera" << endl
		<< "point cloud range:" << min_x << " " << max_x << " " << min_y << " " << max_y << " " << min_z << " " << max_z << "m" << endl
		<< "'s'  to save current frame, 'q' to quit." << std::endl;
	if (save_type == "txt")
	{
		std::cout << "Saving point cloud will be saved in ascii." << std::endl;
	}
	else if (save_type == "bin")
	{
		std::cout << "Saving point cloud will be saved in binary." << std::endl;
	}
	else if (save_type == "auto")
	{
		SAVEPOINTCLOUD = true;
		pmd_camera.set_capture_interval(interval_second);
		std::cout << "Saving point cloud will be saved in binary automatically." << std::endl;
	}
	else
	{
		std::cout << "Please input \"txt\" or \"bin\" as a type of saving file" << std::endl;
		return -1;
	}


	/*
	\param[in] camera_index before it, you have to know which camera you want to initialize
	\param[in] operate_mode See "Getting_Started_CamBoard_pico_monstar.pdf"

	0: indoor room reconstruction, MODE_9_5FPS_1900
	1: Room scanning indoor navigation, MODE_9_10FPS_900
	2: 3D object reconstruction, MODE_9_15FPS_600
	3: Medium size object recognition or face reconstruction, MODE_9_25FPS_300
	4: Remote collaboration, MODE_5_35PFS_500
	5: Small object, MODe_5_45FPS_400
	6: Hand tracking, MODE_5_60FPS
	7: Mixed Mode
	8: Mixed Mode
	9: low noise extended
	10: Fast acquisition
	11: very fast acquisition

	you should use 'get_camera_size' to see camera_index[from 0 to camera_size].
	*/
	pmd_camera.init_camera(0, 1);

	pmd_camera.set_camera_data_mode(0);

	viewer->setBackgroundColor(0, 0, 0);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	viewer->addPointCloud<pcl::PointXYZI>(cloud, "cloud");
	// it might be a bug cause if dont update first the update in other functions will be freezed.
	viewer->updatePointCloud<pcl::PointXYZI>(cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
	//viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

	pmd_camera.start_capture();

	while (!viewer->wasStopped())
	{
		pcl::PointCloud<PCFORMAT>::Ptr to_show_point_cloud(new pcl::PointCloud<PCFORMAT>);
		pcl::PointCloud<PCFORMAT>::Ptr vi_cloud = pmd_camera.get_visualization_cloud_ptr();
		{
			std::lock_guard<std::mutex> guard(pointcloud_mutex);
			pcl::copyPointCloud(*vi_cloud, *to_show_point_cloud);
		}
		to_show_point_cloud->height = 1;
		to_show_point_cloud->width = to_show_point_cloud->points.size();
		add_point_cloud_visualization(viewer, to_show_point_cloud);
		viewer->spinOnce(33);
	}
	pmd_camera.stop_capture();
	return 0;
}
