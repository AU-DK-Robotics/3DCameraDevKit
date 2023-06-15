#pragma once

//#include "targetver.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdlib.h>
#include <tchar.h>
#include <vector>
#include "windows.h"
#include "VisionBooster.h"

#ifdef _WIN64
#ifdef _DEBUG
#pragma comment(lib, ".\\x64\\Debug\\VisionBooster.lib")
#else
#pragma comment(lib, ".\\x64\\Release\\VisionBooster.lib")
#endif
// #else
// #ifdef _DEBUG
// #pragma comment(lib,"..\\win32\\Debug\\VisionBooster.lib")
// #else
// #pragma comment(lib,"..\\win32\\Release\\VisionBooster.lib")
// #endif
#endif