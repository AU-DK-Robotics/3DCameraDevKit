#include "../include/L215u.h"

int main(int argc, char* argv[])
{
	std::map<std::string, std::string> parameter_list;
	read_parameters_from_file("camera_parameters_l215u.txt", parameter_list);
	std::string directory = parameter_list["saved_directory_name"];
	std::string save_type = parameter_list["saved_format"];
	float interval_second = std::stof(parameter_list["interval_time"]);
	size_t auto_number = (save_type == "auto") ? std::stol(parameter_list["auto_number"]) : INT_MAX;
	bool AUTOPOINTCLOUD = (save_type == "auto") ? true : false;

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
	vsDepth.setMirroringEnabled(true);

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
		vsColor.setMirroringEnabled(true);
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

	VideoFrameRef cFrame;
	/*Mat imgColor;
	getRGBImage(imgColor, vsColor, cFrame);*/

	cv::namedWindow("videodemo");

	size_t frame_count = 0;

	while (true)
	{
		Mat imgColor;
		getRGBImage(imgColor, vsColor, cFrame);
		imshow("videodemo", imgColor);

		char key_board = cv::waitKey(33);
		if ('q' == key_board)
		{
			break;
		}
		else if ('s' == key_board)
		{
			std::vector<int> current_date;
			std::string save_filename, final_save_filename;
			save_filename = get_current_date(current_date) + "_" + std::to_string(frame_count++);
			save_filename = directory + "/" + save_filename;
			cv::imwrite(save_filename + ".jpg", imgColor);
			std::cout << "saved: " << save_filename << std::endl;
		}
	}

	//vsDepth.stop();
	//vsDepth.destroy();
	if (isColorValid)
	{
		vsColor.stop();
		vsColor.destroy();
	}

	devDevice.close();
	OpenNI::shutdown();

	return 0;
}
