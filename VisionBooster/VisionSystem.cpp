#include "VisionSystem.h"

VisionSystem::VisionSystem(std::string installPath)
{
	this->installPath = installPath;
}

VisionSystem::~VisionSystem()
{

}

void VisionSystem::initVisionSystem()
{
	int result = VST3D_Init(this->installPath.data, true);  // default path, start without software's window
	if (result != VST3D_RESULT_OK)
	{
		printf("Could not Start Scanner software.\n");
		VST3D_Exit();
		//return -1;
	}

	result = VST3D_Connect(); // Self-check and connect
	if (result != VST3D_RESULT_OK)
	{
		printf("Could not connect to Scanner.\n");
		VST3D_Exit();
		//return -1;
	}
	VST3D_SetCameraID(0); // 多组3D 扫描头时，可以根据ID号切换。
}

void VisionSystem::scanOnce()
{
	int result = VST3D_Scan();		//  开始单次扫描
	//result = Capture3DData();		//  开始多线程单次扫描，设置超时
	if (result != VST3D_RESULT_OK)
	{
		printf("Scan error...\n");
		retryCon();
	}
	
	int totalNum = 0, lossN = 0;
	result = VST3D_GetNumPoints(totalNum); // 得到当前单次扫描总点数
	if (result != VST3D_RESULT_OK)
	{
		lossN++;
		printf("Frame loss in 3D acquisition, try it again...\n");
	}
	else
	{
		char filename[64];
		sprintf_s(filename, "./points%d.txt", N); // 3D points 临时保存为文件
		//fstream file(filename, ios::out);
		int totalNum = 0;
		VST3D_PT *pPointClouds = nullptr;
		result = VST3D_GetPointClouds(totalNum, &pPointClouds); // 得到当前单次扫描总点数和乱序的点云

		// method 1 ： copy all points
		VST3D_PT *myPts = new VST3D_PT[totalNum];
		memcpy((void *)myPts, (void *)(pPointClouds), totalNum * sizeof(VST3D_PT));
		for (int i = 0; i < totalNum; i++) // 遍历拷贝的采集到的所有乱序点云，得到点云坐标等信息，通过索引方式[0 1 2 ....]
		{
			VST3D_PT &pt = myPts[i];
			float x, y, z, nx, ny, nz;
			float cr, cg, cb;
			x = pt.x;
			y = pt.y;
			z = pt.z;
			nx = pt.nx;
			ny = pt.ny;
			nz = pt.nz;
			cr = (float)pt.cr / 255;
			cg = (float)pt.cg / 255;
			cb = (float)pt.cb / 255;
			file << x << " " << y << " " << z << " " << cr << " " << cg << " " << cb << " " << nx << " " << ny << " " << nz << endl;
		}

		if (myPts)
		{

			delete[] myPts;
		}
	}
}

void VisionSystem::retryCon()
{
	int nreset = 0;
	while (true)
	{
		// 每次重新连接前延时几秒，相机，光栅硬件驱动加载需要时间
		Sleep(CAPTURE_TIME_OUT); 

		// 单机扫描情况
		int result = VST3D_Reset(this -> installPath.data()); // 单机扫描重新连接扫描仪系统，重启

		if (result != VST3D_RESULT_OK)
		{
			printf("Check cables connected to Scanner.\n"); // 重新连接失败，继续尝试
		}
		else
		{
			// 重新连接成功，重新扫描
			break; 
		}
		if (nreset++ > 5) // 做5次重新连接尝试，如果不能正常连接扫描仪，退出软件系统
		{
			VST3D_Exit();
			//return result;
		}
	}
}
