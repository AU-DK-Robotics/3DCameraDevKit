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
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

	PMDCamera pmd_camera(viewer);

	size_t camera_size = 0;

	pmd_camera.get_camera_size(camera_size);

	cout << "detected " << camera_size << " camera" << endl;

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
	pmd_camera.init_camera(0, 6);

	pmd_camera.set_camera_data_mode(0);

	viewer->setBackgroundColor(0, 0, 0);
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
	viewer->addPointCloud<pcl::PointXYZI>(cloud, "cloud");
	// it might be a bug cause if dont update first the update in other functions will be freezed.
	viewer->updatePointCloud<pcl::PointXYZI>(cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

	pmd_camera.start_capture();

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
	}

	pmd_camera.stop_capture();
	return 0;
}
