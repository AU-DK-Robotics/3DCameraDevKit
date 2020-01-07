#include "PMDCamera.h"

int main(int argc, char *argv[])
{
	PMDCamera pmd_camera;
	
	size_t camera_size = 0;

	pmd_camera.get_camera_size(camera_size);

	cout << "detected " << camera_size << " camera" << endl;

	pmd_camera.init_camera(0);

	pmd_camera.set_camera_data_mode(0);

	pmd_camera.start_capture();

	Sleep(5000);

	pmd_camera.stop_capture();

	return 0;
}
