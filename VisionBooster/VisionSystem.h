#pragma once
#include "stdafx.h"

// 获取点云超时
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

	void scanOnce();

	void retryCon();
private:
	std::string installPath;

};

