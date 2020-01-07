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

ListenerPointCloud::ListenerPointCloud()
{
}

ListenerPointCloud::~ListenerPointCloud()
{
}

void ListenerPointCloud::onNewData(const royale::SparsePointCloud * data)
{
	cout << "retrieve point cloud ..." << endl;
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
