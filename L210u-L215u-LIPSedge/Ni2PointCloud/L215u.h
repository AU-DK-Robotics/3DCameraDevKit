#ifndef L215U_H
#define L215U_H

#include "../../common/common.h"

#include <OpenNI.h>
#include <opencv2/opencv.hpp>
//#include <opencv2/viz/vizcore.hpp>

using namespace std;
using namespace openni;
using namespace cv;

extern bool SAVEPOINTCLOUD;
extern std::mutex pointcloud_mutex;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* viewer_void);

Mat niComputeCloud(const Mat depthMap, const VideoStream& depthStream);
int niComputeCloud(const Mat depthMap, const VideoStream& depthStream, pcl::PointCloud<pcl::PointXYZ>& points);

int generateOneFramePointCloud(Mat &imgDepth, Mat &imgColor, Mat &mPointCloud, VideoFrameRef &dFrame, VideoFrameRef &cFrame, VideoStream & vsDepth, VideoStream & vsColor, pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud, bool isColorValid);

int convert2pclRGB(cv::Mat & points, cv::Mat & color, pcl::PointCloud<pcl::PointXYZRGB> & pointRGB );

#endif
