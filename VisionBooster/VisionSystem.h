#pragma once
#include "stdafx.h"

//const int CAPTURE_TIME_OUT = 6000; 
// Scan once
#define VST_SINGLE_SCAN			 
//#define VST_UnorderedPointClouds // Unordered points

struct cropSize_t
{
	cropSize_t() 
		:min_x(DBL_MAX), min_y(DBL_MAX), min_z(DBL_MAX),
		max_x(DBL_MIN), max_y(DBL_MIN), max_z(DBL_MIN) {}
	double min_x, min_y, min_z;
	double max_x, max_y, max_z;
};

class VisionSystem
{
public:
	VisionSystem(std::string installPath);

	~VisionSystem();

	void initVisionSystem();

	bool scanOnce(std::vector<VST3D_PT> & VSTPoints);
	
	void cropPointCloud(const cropSize_t & cropSize, const std::vector<VST3D_PT> & VSTPoints, std::vector<VST3D_PT> & new_VSTPoints);

	const cropSize_t calBoundingBox(const std::vector<VST3D_PT> & VSTPoints);
	const cropSize_t calBoundingBox(const std::string pointsFilename);

	void fittingCylidner(const std::string pointsFilename, Eigen::Vector3f &point, Eigen::Vector3f &axis);

	Eigen::Matrix4f generateRMatrixAlongAxis(Eigen::Vector3f point, Eigen::Vector3f axis, float angle);

	void retryCon(const std::string errorStr, const int codeLine, const std::string codeFile);

	void disConnect();

private:
	std::string installPath;

	// Default Capture Method: 2
	const int captureMethod;

	float ToRadians(float deg);

	void loadPointsFromFile(const std::string pointsFilename, std::vector<VST3D_PT>& VSTPoints);
};

