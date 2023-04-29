#ifndef L215U_H
#define L215U_H

#include "../../common/common.h"
#include "LIPSNICustomProperty.h"

#include <OpenNI.h>
#include <opencv2/opencv.hpp>
//#include <opencv2/viz/vizcore.hpp>

using namespace std;
using namespace openni;
using namespace cv;

extern bool SAVEPOINTCLOUD;
extern bool AUTOPOINTCLOUD;
extern std::mutex pointcloud_mutex;
extern bool CALIBRATED;


struct CameraParam 
{
	// Ref: https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html

	// Intrinsic parameters
	double Fx, Fy, Cx, Cy;
	// Resolution of camera
	double Rx, Ry;
	// Distortion coefficients: 
	double K1, K2, K3, P1, P2;
};

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void);

//Mat niComputeCloud(const Mat depthMap, const VideoStream& depthStream);
int niComputeCloud(
	const Mat depthMap, const VideoStream& depthStream, 
	pcl::PointCloud<PCFORMAT>& points, const CameraParam& camera_param, cv::Rect& validRect);

int generateOneFramePointCloud(
	Mat& imgDepth, Mat& imgColor, Mat& mPointCloud, VideoFrameRef& dFrame, VideoFrameRef& cFrame, VideoStream& vsDepth,
	VideoStream& vsColor, pcl::PointCloud<PCFORMAT>::Ptr pointCloud, bool isColorValid, CameraParam& camera_param);

int getRGBImage(cv::Mat& imgColor, VideoStream& vsColor, VideoFrameRef& cFrame);

int convert2pclRGB(cv::Mat & points, cv::Mat & color, pcl::PointCloud<pcl::PointXYZRGB> & pointRGB );

int loadCameraIntrinsicParam(const std::string& filename, CameraParam & camera_param);

#endif
