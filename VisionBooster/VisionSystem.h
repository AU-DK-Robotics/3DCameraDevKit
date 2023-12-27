#pragma once
#include "stdafx.h"

const int CAPTURE_TIME_OUT = 6000; 
// Scan once
#define VST_SINGLE_SCAN			 
//#define VST_UnorderedPointClouds // Unordered points

class VisionSystem
{
public:
	VisionSystem(std::string installPath);

	~VisionSystem();

	void initVisionSystem();

	bool scanOnce(std::vector<VST3D_PT> & VSTPoints);

	void retryCon(const std::string errorStr, const int codeLine, const std::string codeFile);

	void disConnect();

private:
	std::string installPath;

	// Default Capture Method: 2
	const int captureMethod;
};

