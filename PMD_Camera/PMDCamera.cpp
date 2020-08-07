#include "PMDCamera.h"

bool SAVEPOINTCLOUD = false;

PMDCamera::PMDCamera(pcl::visualization::PCLVisualizer::Ptr viewer_ptr)
{
	m_listener_point_cloud.set_viewer_ptr(viewer_ptr);
}


PMDCamera::~PMDCamera()
{
}

int PMDCamera::get_camera_size(size_t &camera_size)
{
	m_camera_list = m_camera_manager.getConnectedCameraList();

	camera_size =  m_camera_list.size();

	return 0;
}

int PMDCamera::init_camera(size_t camera_index, size_t operate_mode)
{
	m_camera_device = m_camera_manager.createCamera(m_camera_list[camera_index]);

	royale::CameraStatus status = m_camera_device->initialize();

	if (status != CameraStatus::SUCCESS)
	{
		cerr << "Cannot initialize the camera device, error string : " << getErrorString(status) << endl;
		return 1;
	}

	royale::Vector<royale::String> useCases;

	status = m_camera_device->getUseCases(useCases);

	if (status != royale::CameraStatus::SUCCESS || useCases.empty())
	{
		cerr << "No use cases are available" << endl;
		cerr << "getUseCases() returned: " << getErrorString(status) << endl;
		return 1;
	}

	for (size_t i = 0; i < useCases.size(); ++i)
	{
		if (i == operate_mode)
		{
			cout << "Select the " << useCases[i] << "mode" << endl;
		}
	}
	// set an operation mode
	if (m_camera_device->setUseCase(useCases.at(operate_mode)) != royale::CameraStatus::SUCCESS)
	{
		cerr << "Error setting use case" << endl;
		return 1;
	}
	return 0;

}

int PMDCamera::set_camera_data_mode(size_t data_mode)
{
	if (data_mode == 0)
	{
		if (m_camera_device->registerSparsePointCloudListener(&m_listener_point_cloud) != CameraStatus::SUCCESS)
		{
			cerr << "Error registering point cloud listener" << endl;
			return 1;
		}
	}

	else if (data_mode == 1)
	{
		if (m_camera_device->registerDepthImageListener(&m_listener_depth) != CameraStatus::SUCCESS)
		{
			cerr << "Error registering depth listener" << endl;
			return 1;
		}
	}
	return 0;
}

int PMDCamera::start_capture()
{
	this->stop_capture();

	if (m_camera_device->startCapture() != CameraStatus::SUCCESS)
	{
		cerr << "Error starting the capturing" << endl;
		return 1;
	}
	return 0;
}

int PMDCamera::stop_capture()
{
	if (m_camera_device->stopCapture() != CameraStatus::SUCCESS)
	{
		cerr << "Error stopping the capturing" << endl;
		return 1;
	}
	return 0;
}

void ListenerPointCloud::save_royale_xyzcPoints(const royale::SparsePointCloud * data)
{
	PCFORMAT p;
	for (size_t i = 0; i < data->xyzcPoints.size(); i += 4)
	{
		p.x = data->xyzcPoints.at(i),
		p.y = data->xyzcPoints.at(i + 1),
		p.z = data->xyzcPoints.at(i + 2),
		p.intensity = data->xyzcPoints.at(i + 3);

		m_cloud_ptr->points.push_back(p);
	}
	//std::cout << "point size=" << m_cloud_ptr->points.size() << std::endl;
}

ListenerPointCloud::ListenerPointCloud()
	:m_cloud_ptr(new pcl::PointCloud<PCFORMAT>())
{

}


ListenerPointCloud::~ListenerPointCloud()
{

}

void ListenerPointCloud::set_viewer_ptr(pcl::visualization::PCLVisualizer::Ptr viewer_ptr)
{
	m_viewer_ptr = viewer_ptr;
}

void ListenerPointCloud::onNewData(const royale::SparsePointCloud * data)
{
	save_royale_xyzcPoints(data);

	add_point_cloud_visualization(m_viewer_ptr, m_cloud_ptr);

	if (SAVEPOINTCLOUD)
	{
		std::string save_filename = get_current_date() + ".bin";
		write_point_cloud_binary(m_cloud_ptr, save_filename);
		std::cout << "write to " << save_filename << "(" << m_cloud_ptr->points.size() << ")" << std::endl;
		SAVEPOINTCLOUD = false;
	}

	// clear points
	m_cloud_ptr->clear();
}

pcl::PointCloud<PCFORMAT>::Ptr ListenerPointCloud::get_cloud_ptr() const
{
	return this->m_cloud_ptr;
}

ListenerDepth::ListenerDepth()
{
}

ListenerDepth::~ListenerDepth()
{
}

void ListenerDepth::onNewData(const royale::DepthImage * data)
{
	cout << "retrieve depth image ..." << endl;
}

void add_point_cloud_visualization(pcl::visualization::PCLVisualizer::Ptr viewer_ptr, pcl::PointCloud<PCFORMAT>::Ptr m_cloud_ptr)
{
	if (!m_cloud_ptr->points.empty())
	{
		m_cloud_ptr->width = m_cloud_ptr->points.size();
		m_cloud_ptr->height = 1;
		// Keep the same name as shown in main function
		viewer_ptr->updatePointCloud<PCFORMAT>(m_cloud_ptr, "cloud");
	}
}

void write_point_cloud_binary(pcl::PointCloud<PCFORMAT>::Ptr m_cloud_ptr, const std::string filename)
{
	FILE * stream = fopen(filename.c_str(), "wb");
	fwrite((void *)&m_cloud_ptr->points.at(0), sizeof(float), 4 * m_cloud_ptr->points.size(), stream);
	fclose(stream);
}

std::string get_current_date()
{
	time_t tt = time(NULL);
	struct tm *stm = localtime(&tt);

	char tmp[32];
	sprintf(tmp, "%04d-%02d-%02d-%02d-%02d-%02d", 1900 + stm->tm_year, 1 + stm->tm_mon, stm->tm_mday, stm->tm_hour,
		stm->tm_min, stm->tm_sec);

	return std::string(tmp);
}
