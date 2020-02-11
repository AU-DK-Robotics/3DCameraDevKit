#define SHOWCLOUD

#ifdef SHOWCLOUD
#include <pcl/visualization/pcl_visualizer.h>
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);
	//if (event.getKeySym() == "r" && event.keyDown())
	//	std::cout << "'r' was pressed" << std::endl;
	if (event.getKeySym() == "s" && event.keyDown())
		std::cout << "'s' was pressed" << std::endl;
}
#endif // SHOWCLOUD

#include "PMDCamera.h"

int main(int argc, char *argv[])
{
	PMDCamera pmd_camera;
	
	size_t camera_size = 0;

	pmd_camera.get_camera_size(camera_size);

	cout << "detected " << camera_size << " camera" << endl;

	pmd_camera.init_camera(0, 6);

	pmd_camera.set_trigger_count(10);

	pmd_camera.set_camera_data_mode(0);

	pmd_camera.start_capture();

#ifdef SHOWCLOUD
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud");
	//viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&viewer);

	while (!viewer->wasStopped())
	{
		viewer->updatePointCloud<pcl::PointXYZ>(pmd_camera.get_cloud_ptr(), "cloud");
		viewer->spinOnce(100);
	}

#endif // SHOWCLOUD

	pmd_camera.stop_capture();

	return 0;
}
