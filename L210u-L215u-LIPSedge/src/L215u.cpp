#include "../include/L215u.h"

bool SAVEPOINTCLOUD = false;
bool AUTOPOINTCLOUD = false;
std::mutex pointcloud_mutex;
bool CALIBRATED = false;
cv::Rect ValidRoi;
cv::Mat map1, map2;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* viewer_void)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);

	if (event.getKeySym() == "s" && event.keyDown())
	{
		SAVEPOINTCLOUD = true;
	}

	if (event.getKeySym() == "n" && event.keyDown())
	{
		AUTOPOINTCLOUD = true;
	}
	
}

//Mat niComputeCloud(const Mat depthMap, const VideoStream& depthStream)
//{
//	Size nsize = depthMap.size();
//	vector<Mat> output(3);
//	output[0] = Mat(nsize, CV_32F);
//	output[1] = Mat(nsize, CV_32F);
//	output[2] = Mat(nsize, CV_32F);
//
//	const DepthPixel* pDepthArray = (const DepthPixel*)depthMap.data;
//
//	for (int y = 0; y < depthMap.rows; y++)
//	{
//		for (int x = 0; x < depthMap.cols; x++)
//		{
//			float fX, fY, fZ;
//			CoordinateConverter::convertDepthToWorld(depthStream, x, y, *pDepthArray++, &fX, &fY, &fZ);
//			output[0].at<float>(y, x) = fX;
//			output[1].at<float>(y, x) = fY;
//			output[2].at<float>(y, x) = fZ;
//		}
//	}
//
//	Mat outMat;
//	merge(output, outMat);
//	return outMat;
//}

int niComputeCloud(
	const Mat depthMap, const VideoStream& depthStream, pcl::PointCloud<PCFORMAT>& points,
	const CameraParam& camera_param, cv::Rect & validRect)
{
	const DepthPixel* pDepthArray = (const DepthPixel*)depthMap.data;
	points.clear();

	// 1.
	// This is a method using converDepthToWorld below, but it is not accurate
	//for (int y = 0; y < depthMap.rows; y++)
	//{
	//	for (int x = 0; x < depthMap.cols; x++)
	//	{
	//		float fX, fY, fZ;
	//		CoordinateConverter::convertDepthToWorld(depthStream, x, y, *pDepthArray++, &fX, &fY, &fZ);
	//		//output[0].at<float>(y, x) = fX;
	//		//output[1].at<float>(y, x) = fY;
	//		//output[2].at<float>(y, x) = fZ;

	//		if (fX != 0 && fY != 0 && fZ != 0)
	//		{
	//			// to unit:m
	//			points.push_back(pcl::PointXYZ(fX / 1000.0, fY / 1000.0, fZ / 1000.0));
	//		}
	//	}
	//}

	// 2. 
	// Ref: https://github.com/DavidTrimoulet/PR2-Kinetic-Xenial/blob/fd1840105f/src/image_pipeline/depth_image_proc/include/depth_image_proc/depth_traits.h
	// Ref: https://github.com/DavidTrimoulet/PR2-Kinetic-Xenial/blob/fd1840105fd6c344519a21a83c2308db3cb26c58/src/image_pipeline/depth_image_proc/include/depth_image_proc/depth_conversions.h
	float min_Depth = 200; // 0.2 m
	float max_Depth = 2000; // 2 m
	double unit_scaling = 0.001;

	float constant_x = unit_scaling / camera_param.Fx;//f/dx
	float constant_y = unit_scaling / camera_param.Fy;//f/dy
	float world_x = 0.0, world_y = 0.0, world_z = 0.0;
	int width = depthMap.cols, height = depthMap.rows;

	//std::cout << "width:" << width << " height:" << height << std::endl;

	PCFORMAT p;
	for (int v = validRect.y; v < validRect.height; ++v)
	{
		for (int u = validRect.x; u < validRect.width; ++u)
		{
			uint16_t Depth = pDepthArray[v * width + u];
			
			if (Depth <= 0)
				continue;

			p.x = (u - camera_param.Cx) * float(Depth) * constant_x;
			p.y = (v - camera_param.Cy) * float(Depth) * constant_y;
			p.z = float(Depth) * unit_scaling;
			p.intensity = 1.0;

			points.push_back(p);
		}
	}

	return 0;
}

int convert2pclRGB(cv::Mat& points, cv::Mat& color, pcl::PointCloud<pcl::PointXYZRGB>& pointRGB)
{
	if (points.size() != color.size()) return -1;

	int width = points.size().width;
	int height = points.size().height;

	pointRGB.resize(width * height);

	for (size_t i = 0; i < pointRGB.size(); i++)
	{
		pointRGB[i].x = points.at<Vec3f>(i / width, i % height)[0];
		pointRGB[i].y = points.at<Vec3f>(i / width, i % height)[1];
		pointRGB[i].z = points.at<Vec3f>(i / width, i % height)[2];

		pointRGB[i].r = color.at<Vec3b>(i / width, i % height)[0];
		pointRGB[i].g = color.at<Vec3b>(i / width, i % height)[1];
		pointRGB[i].b = color.at<Vec3b>(i / width, i % height)[2];
	}

	return 0;
}

int loadCameraIntrinsicParam(const std::string& filename, CameraParam& camera_param)
{
	std::map<std::string, std::string> parameter_map;
	read_parameters_from_file(filename, parameter_map);

	assert(parameter_map.size() == 11);

	camera_param.Fx = std::stof(parameter_map["Fx"]);
	camera_param.Fy = std::stof(parameter_map["Fy"]);
	camera_param.Cx = std::stof(parameter_map["Cx"]);
	camera_param.Cy = std::stof(parameter_map["Cy"]);

	camera_param.Rx = std::stof(parameter_map["Rx"]);
	camera_param.Ry = std::stof(parameter_map["Ry"]);

	camera_param.K1 = std::stof(parameter_map["K1"]);
	camera_param.K2 = std::stof(parameter_map["K2"]);
	camera_param.K3 = std::stof(parameter_map["K3"]);

	camera_param.P1 = std::stof(parameter_map["P1"]);
	camera_param.P2 = std::stof(parameter_map["P2"]);

	return 0;
}


int generateOneFramePointCloud(
	Mat& imgDepth, Mat& imgColor, Mat& mPointCloud,
	VideoFrameRef& dFrame, VideoFrameRef& cFrame,
	VideoStream& vsDepth, VideoStream& vsColor,
	pcl::PointCloud<PCFORMAT>::Ptr pointCloud, bool isColorValid,
	CameraParam& camera_param)
{
	// Depth
	if (vsDepth.isValid())
	{
		if (STATUS_OK == vsDepth.readFrame(&dFrame))
		{
			imgDepth = Mat(dFrame.getHeight(), dFrame.getWidth(), CV_16UC1, (void*)dFrame.getData());
			// ignore below
			if (!isColorValid)
			{
				imgDepth.convertTo(imgColor, CV_8U, 255.0 / 4096);
				applyColorMap(imgColor, imgColor, COLORMAP_JET);
			}
		}
	}
	// Color
	if (isColorValid && vsColor.isValid())
	{
		if (STATUS_OK == vsColor.readFrame(&cFrame))
		{
			Mat imgRGBColor(cFrame.getHeight(), cFrame.getWidth(), CV_8UC3, (void*)cFrame.getData());
			cvtColor(imgRGBColor, imgColor, CV_RGB2BGR);
		}
	}
	// PointCloud
	if (!imgDepth.empty() && !imgColor.empty())
	{
		const cv::Mat K = (cv::Mat_<double>(3, 3) << camera_param.Fx, 0.0, camera_param.Cx, 0.0, camera_param.Fy, camera_param.Cy, 0.0, 0.0, 1.0);
		const cv::Mat D = (cv::Mat_<double>(5, 1) << camera_param.K1, camera_param.K2, camera_param.P1, camera_param.P2, camera_param.K3);
		cv::Mat UndistortImage, validImage;

		if (CALIBRATED == false)
		{
			cv::Size imageSize(imgDepth.cols, imgDepth.rows);
			const double alpha = 0;
			//std::cout << K << "\n" << D << std::endl;
			cv::Mat NewCameraMatrix = getOptimalNewCameraMatrix(K, D, imageSize, alpha, imageSize, &ValidRoi, false);
			//initUndistortRectifyMap(K, D, cv::Mat(), NewCameraMatrix, imageSize, CV_16SC2, map1, map2);

			//std::cout << "NewCameraMatrix:\n" << NewCameraMatrix << std::endl;

			//std::cout << "NewCameraMatrix.at<double>(0, 0):" << NewCameraMatrix.at<double>(0, 0) << std::endl;
			//std::cout << "NewCameraMatrix.at<double>(1, 1):" << NewCameraMatrix.at<double>(1, 1) << std::endl;
			//std::cout << "NewCameraMatrix.at<double>(0, 2):" << NewCameraMatrix.at<double>(0, 2) << std::endl;
			//std::cout << "NewCameraMatrix.at<double>(1, 2):" << NewCameraMatrix.at<double>(1, 2) << std::endl;

			// update intrinsic parameters of camera 
			camera_param.Fx = NewCameraMatrix.at<double>(0, 0);
			camera_param.Fy = NewCameraMatrix.at<double>(1, 1);
			camera_param.Cx = NewCameraMatrix.at<double>(0, 2);
			camera_param.Cy = NewCameraMatrix.at<double>(1, 2);

			//cv::undistort(imgDepth, UndistortImage, K, D, NewCameraMatrix);
			//cv::undistort(imgDepth, UndistortImage, K, D);
			//cv::imwrite("imgDepth.jpg", imgDepth);
			//cv::imwrite("UndistortImage.jpg", UndistortImage);
			CALIBRATED = true;
		}
		else
		{
			//cv::undistort(imgDepth, UndistortImage, K, D);
			//cv::imwrite("imgDepth_2.jpg", imgDepth);
			//cv::imwrite("UndistortImage_2.jpg", UndistortImage);
		}
		//remap(imgDepth, UndistortImage, map1, map2, cv::INTER_LINEAR);
		//cv::imwrite("imgDepth.jpg", imgDepth);
		//cv::imwrite("UndistortImage.jpg", UndistortImage);
		//validImage = UndistortImage(ValidRoi);
		//cv::imwrite("validImage.jpg", validImage);
		mPointCloud = niComputeCloud(imgDepth, vsDepth, *pointCloud, camera_param, ValidRoi);
		//mPointCloud = niComputeCloud(imgDepth, vsDepth, *pointCloud, camera_param);
		//write_point_cloud_acsii(pointCloud, "test.txt");
	}
	return 0;
}


int getRGBImage(cv::Mat& imgColor, VideoStream& vsColor, VideoFrameRef& cFrame)
{
	if (STATUS_OK == vsColor.readFrame(&cFrame))
	{
		Mat imgRGBColor(cFrame.getHeight(), cFrame.getWidth(), CV_8UC3, (void*)cFrame.getData());
		cvtColor(imgRGBColor, imgColor, CV_RGB2BGR);
	}
	return 0;
}