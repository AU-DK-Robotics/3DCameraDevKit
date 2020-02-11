#include "PMDCamera.h"

PMDCamera::PMDCamera() 
{
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
			cout << "Choose the " << useCases[i] << "mode" << endl;
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

void PMDCamera::set_trigger_count(size_t _count)
{
	m_listener_point_cloud.set_trigger_count(_count);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PMDCamera::get_cloud_ptr() const
{
	return this->m_listener_point_cloud.get_cloud_ptr();
}

void ListenerPointCloud::save_royale_xyzcPoints(const royale::SparsePointCloud * data, const string& filename, float write_confidence)
{
	ofstream ofile;

	ofile.open(filename, std::ios::out);

	if (!ofile.is_open())
	{
		return;
	}

	for (size_t i = 0; i < data->xyzcPoints.size(); i+=4)
	{
		if (data->xyzcPoints.at(i + 3) > write_confidence)
		{
			ofile
				<< data->xyzcPoints.at(i) << " "
				<< data->xyzcPoints.at(i + 1) << " "
				<< data->xyzcPoints.at(i + 2)
				<< endl;
		}
	}
	ofile.close();
}

void ListenerPointCloud::save_royale_xyzcPoints(const royale::SparsePointCloud * data, vector<pointXYZ>& points, float write_confidence)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

	for (size_t i = 0; i < data->xyzcPoints.size(); i+=4)
	{
		float c = data->xyzcPoints.at(i + 3);

		if (c > write_confidence)
		{
			float
				x = data->xyzcPoints.at(i),
				y = data->xyzcPoints.at(i + 1),
				z = data->xyzcPoints.at(i + 2);

			points.push_back(pointXYZ(x, y, z));

			cloud_ptr->points.push_back(pcl::PointXYZ(x, y, z));
			//m_cloud_ptr->points.push_back(pcl::PointXYZ(x, y, z));
		}
	}
			
	m_cloud_ptr = cloud_ptr;
}

ListenerPointCloud::ListenerPointCloud():
	m_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>),
	m_local_trigger_count(0)
{

}

ListenerPointCloud::~ListenerPointCloud()
{

}

void ListenerPointCloud::onNewData(const royale::SparsePointCloud * data)
{
	cout << "retrieve point cloud ..." << endl;
	
	// save_royale_xyzcPoints(data, "current_frame.txt");

	if (++m_local_trigger_count == m_trigger_count)
	{
		save_royale_xyzcPoints(data, this->m_points, 0.5);
		m_local_trigger_count = 0;
	}
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ListenerPointCloud::get_cloud_ptr() const
{
	return this->m_cloud_ptr;
}

std::vector<pointXYZ> ListenerPointCloud::get_points() const
{
	return this->m_points;
}

void ListenerPointCloud::set_trigger_count(size_t _trigger_count)
{
	this->m_trigger_count = _trigger_count;
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
