#include "L215u.h"

bool SAVEPOINTCLOUD = false;
std::mutex pointcloud_mutex;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *>(viewer_void);

	if (event.getKeySym() == "s" && event.keyDown())
	{
		SAVEPOINTCLOUD = true;
	}
}

Mat niComputeCloud(const Mat depthMap, const VideoStream& depthStream)
{
	Size nsize = depthMap.size();
	vector<Mat> output(3);
	output[0] = Mat(nsize, CV_32F);
	output[1] = Mat(nsize, CV_32F);
	output[2] = Mat(nsize, CV_32F);

	const DepthPixel* pDepthArray = (const DepthPixel*)depthMap.data;

	for (int y = 0; y < depthMap.rows; y++)
	{
		for (int x = 0; x < depthMap.cols; x++)
		{
			float fX, fY, fZ;
			CoordinateConverter::convertDepthToWorld(depthStream, x, y, *pDepthArray++, &fX, &fY, &fZ);
			output[0].at<float>(y, x) = fX;
			output[1].at<float>(y, x) = fY;
			output[2].at<float>(y, x) = fZ;
		}
	}

	Mat outMat;
	merge(output, outMat);
	return outMat;
}

int niComputeCloud(const Mat depthMap, const VideoStream& depthStream, pcl::PointCloud<pcl::PointXYZ>& points)
{
	//Size nsize = depthMap.size();
	//vector<Mat> output(3);
	//output[0] = Mat(nsize, CV_32F);
	//output[1] = Mat(nsize, CV_32F);
	//output[2] = Mat(nsize, CV_32F);

	const DepthPixel* pDepthArray = (const DepthPixel*)depthMap.data;
	points.clear();

	for (int y = 0; y < depthMap.rows; y++)
	{
		for (int x = 0; x < depthMap.cols; x++)
		{
			float fX, fY, fZ;
			CoordinateConverter::convertDepthToWorld(depthStream, x, y, *pDepthArray++, &fX, &fY, &fZ);
			//output[0].at<float>(y, x) = fX;
			//output[1].at<float>(y, x) = fY;
			//output[2].at<float>(y, x) = fZ;

			if (fX != 0 && fY != 0 && fZ != 0)
			{
				// to unit:m
				points.push_back(pcl::PointXYZ(fX / 1000.0, fY / 1000.0, fZ / 1000.0));
			}
		}
	}

	//Mat outMat;
	//merge(output, outMat);
	//return outMat;
	return 0;
}

int convert2pclRGB(cv::Mat & points, cv::Mat & color, pcl::PointCloud<pcl::PointXYZRGB>& pointRGB)
{
	if (points.size() != color.size()) return -1;

	int width = points.size().width;
	int height = points.size().height;

	pointRGB.resize(width * height);

	for (size_t i = 0; i< pointRGB.size(); i++)
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


int generateOneFramePointCloud(
	Mat &imgDepth, Mat &imgColor, Mat &mPointCloud,
	VideoFrameRef &dFrame, VideoFrameRef &cFrame,
	VideoStream & vsDepth, VideoStream & vsColor,
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, bool isColorValid)
{
	if (vsDepth.isValid())
	{
		if (STATUS_OK == vsDepth.readFrame(&dFrame))
		{
			imgDepth = Mat(dFrame.getHeight(), dFrame.getWidth(), CV_16UC1, (void*)dFrame.getData());
			if (!isColorValid)
			{
				imgDepth.convertTo(imgColor, CV_8U, 255.0 / 4096);
				applyColorMap(imgColor, imgColor, COLORMAP_JET);
			}
		}
	}
	if (isColorValid && vsColor.isValid())
	{
		if (STATUS_OK == vsColor.readFrame(&cFrame))
		{
			Mat imgRGBColor(cFrame.getHeight(), cFrame.getWidth(), CV_8UC3, (void*)cFrame.getData());
			cvtColor(imgRGBColor, imgColor, CV_RGB2BGR);
		}
	}

	if (!imgDepth.empty() && !imgColor.empty())
	{
		mPointCloud = niComputeCloud(imgDepth, vsDepth, *pointCloud);

		//write_point_cloud_acsii(pointCloud, "test.txt");
	}
	return 0;
}