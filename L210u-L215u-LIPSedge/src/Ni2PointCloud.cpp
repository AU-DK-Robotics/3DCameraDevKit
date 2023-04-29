#include "../include/L215u.h"

int main(int argc, char* argv[])
{
	std::map<std::string, std::string> parameter_list;
	read_parameters_from_file("camera_parameters_l215u.txt", parameter_list);
	std::string all_directory = parameter_list["saved_directory_name"];
	std::string save_type = parameter_list["saved_format"];
	float interval_second = std::stof(parameter_list["interval_time"]);
	size_t auto_number = (save_type == "auto") ? std::stol(parameter_list["auto_number"]) : INT_MAX;
	//bool AUTOPOINTCLOUD = (save_type == "auto") ? true : false;

	std::vector<std::string> directoies;
	boost::split(directoies, all_directory, boost::is_any_of("#"));
	size_t index_dir = 0;
	std::string curr_dir = directoies[index_dir];
	std::cout << "current directory is " << curr_dir << std::endl;

	//for (auto s: directoies)
	//	std::cout << s << std::endl;
	//return 1;

	float hfov, vfov;
	Status status;
	bool isColorValid = false;

	if (STATUS_OK != OpenNI::initialize())
	{
		cout << "After initialization: " << OpenNI::getExtendedError() << endl << endl;
		return 1;
	}

	Device devDevice;
	if (STATUS_OK != devDevice.open(ANY_DEVICE))
	{
		cout << "ERROR: Cannot open device: " << OpenNI::getExtendedError() << endl << endl;
		return 1;
	}

	VideoMode mode;
	VideoStream vsDepth;
	VideoStream vsColor;

	// Create and setup depth stream
	if (STATUS_OK != vsDepth.create(devDevice, SENSOR_DEPTH))
	{
		cout << "ERROR: Cannot create depth stream on device" << endl << endl;
		return 1;
	}
	vsDepth.setMirroringEnabled(false);

	mode = vsDepth.getVideoMode();
	cout << "Depth VideoMode: " << mode.getResolutionX() << " x " << mode.getResolutionY() << " @ " << mode.getFps() << " FPS";
	cout << ", Unit is ";
	if (mode.getPixelFormat() == PIXEL_FORMAT_DEPTH_1_MM)
	{
		cout << "1mm";
	}
	else if (mode.getPixelFormat() == PIXEL_FORMAT_DEPTH_100_UM)
	{
		cout << "100um";
	}
	cout << endl;

	status = vsDepth.getProperty<float>(ONI_STREAM_PROPERTY_HORIZONTAL_FOV, &hfov);
	status = vsDepth.getProperty<float>(ONI_STREAM_PROPERTY_VERTICAL_FOV, &vfov);

	// Create and setup color stream
	if (STATUS_OK != vsColor.create(devDevice, SENSOR_COLOR))
	{
		cout << "ERROR: Cannot create color stream on device" << endl << endl;
	}
	else
	{
		vsColor.setMirroringEnabled(false);
	}

	// Check and enable Depth-To-Color image registration
	if (vsColor.isValid())
	{
		if (devDevice.isImageRegistrationModeSupported(IMAGE_REGISTRATION_DEPTH_TO_COLOR))
		{
			if (STATUS_OK == devDevice.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR))
			{
				isColorValid = true;
			}
			else
			{
				cout << "ERROR: Failed to set imageRegistration mode" << endl << endl;
			}
		}
		else
		{
			cout << "ERROR: ImageRegistration mode is not supported" << endl << endl;
		}
	}

	// Start streams
	if (STATUS_OK != vsDepth.start())
	{
		cout << "ERROR: Cannot start depth stream on device" << endl << endl;
		return 1;
	}
	if (isColorValid)
	{
		if (STATUS_OK != vsColor.start())
		{
			cout << "ERROR: Cannot start color stream on device" << endl << endl;
			return 1;
		}
	}

	VideoFrameRef dFrame;
	VideoFrameRef cFrame;

	Mat mPointCloud;
	Mat imgDepth, imgColor;
	//pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<PCFORMAT>::Ptr pointCloud(new pcl::PointCloud<PCFORMAT>);

	vtkObject::GlobalWarningDisplayOff();
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::PointCloud<PCFORMAT>::Ptr cloud(new pcl::PointCloud<PCFORMAT>);
	viewer->addPointCloud<PCFORMAT>(cloud, "cloud");
	// it might be a bug cause if dont update first the update in other functions will be freezed.
	viewer->updatePointCloud<PCFORMAT>(cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
	viewer->addCoordinateSystem(0.5);
	viewer->initCameraParameters();
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

	size_t frame_count = 0;
	double current_second = clock();

	Eigen::Matrix4f tsfm(Eigen::Matrix4f::Identity());
	tsfm(0, 0) = -1;
	tsfm(1, 1) = -1;

	/*
	-1 0 0 0
	0 -1 0 0
	0  0 1 0
	0  0 0 1
	*/
	std::vector< pcl::PointCloud<PCFORMAT>::Ptr> to_save_points;
	std::vector<std::string> to_save_points_filename;
	CameraParam camera_param;
	loadCameraIntrinsicParam("camera_intrinsicParam.txt", camera_param);
	
	while (!viewer->wasStopped())
	{
		generateOneFramePointCloud(imgDepth, imgColor, mPointCloud, dFrame, cFrame, vsDepth, vsColor, pointCloud, isColorValid, camera_param);

		pcl::PointCloud<PCFORMAT>::Ptr to_show_point_cloud(new pcl::PointCloud<PCFORMAT>);
		{
			std::lock_guard<std::mutex> guard(pointcloud_mutex);
			pcl::copyPointCloud(*pointCloud, *to_show_point_cloud);
		}
		to_show_point_cloud->height = 1;
		to_show_point_cloud->width = to_show_point_cloud->points.size();
		pcl::transformPointCloud(*to_show_point_cloud, *to_show_point_cloud, tsfm);
		add_point_cloud_visualization(viewer, to_show_point_cloud);
		//std::cout << "Point size:" << to_show_point_cloud->width << std::endl;
		//std::cout << "frame_count: " << frame_count << std::endl;

		if (SAVEPOINTCLOUD || AUTOPOINTCLOUD)
		{
			std::vector<int> current_date;
			std::string save_filename, final_save_filename;
			save_filename = get_current_date(current_date) + "_" + std::to_string(frame_count);
			save_filename = curr_dir + "/" + save_filename;

			double last_second = clock();

			if (save_type == "txt")
			{
				final_save_filename = save_filename + ".txt";
				write_point_cloud_acsii(to_show_point_cloud, final_save_filename);
				++frame_count;
			}
			else if (save_type == "auto" && ((last_second - current_second) > interval_second))
			{
				if (frame_count == auto_number)
				{
					std::cout << "frame number limited: break & saving point clouds ..." << std::endl;
					
					std::cout <<"to_save_points.size():" << to_save_points.size() << std::endl;
					for (size_t i = 0; i < to_save_points.size(); i++)
					{
						write_point_cloud_acsii(to_save_points[i], to_save_points_filename[i]);
						std::cout << "[" << i << "/" << to_save_points.size() << "] saved: " << to_save_points_filename[i] << std::endl;
					}
					std::cout << "point clouds are saved" << std::endl;

					index_dir = (index_dir + 1) == directoies.size() ? directoies.size() - 1 : index_dir + 1;
					curr_dir = directoies[index_dir];
					std::cout << "current directory is " << curr_dir << std::endl;

					AUTOPOINTCLOUD = false;
					frame_count = 0;
					to_save_points.clear();
					to_save_points_filename.clear();
					continue;
				}

				current_second = last_second;
				final_save_filename = save_filename + ".txt";
				//write_point_cloud_acsii(to_show_point_cloud, final_save_filename);
				//std::cout
				//	<< '[' << frame_count << '/' << auto_number << ']'
				//	<< "saved to " << final_save_filename << std::endl;
				to_save_points.push_back(to_show_point_cloud);
				to_save_points_filename.push_back(final_save_filename);
				std::cout << "captured [" << frame_count <<"] with \"" <<final_save_filename<<"\"" << std::endl;

				++frame_count;
			}
			SAVEPOINTCLOUD = false;
		}

		viewer->spinOnce(33);
	}

	

	vsDepth.stop();
	vsDepth.destroy();
	if (isColorValid)
	{
		vsColor.stop();
		vsColor.destroy();
	}

	devDevice.close();
	OpenNI::shutdown();

	return 0;
}
