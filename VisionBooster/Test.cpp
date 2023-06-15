#include "stdafx.h"
using namespace std;

#define CAPTURE_TIME_OUT 6000 // 获取点云超时
#define VST_SINGLE_SCAN			 // Scan once
#define VST_UnorderedPointClouds // Unordered points 

const char *install_path = "C:\\Program Files\\VST\\VisenTOP Studio\\VisenTOP Studio.exe";

//void WINAPI CaptureThreadFunc(LPVOID lpParameter)
//{
//	int *pResult = (int *)lpParameter;
//	*pResult = VST3D_Scan();
//}
//
//int Capture3DData()
//{
//	HANDLE hThread;
//	DWORD ThreadID;
//	int VST3D_RESULT = VST3D_RESULT_OK;
//	hThread = CreateThread(NULL, 0, (LPTHREAD_START_ROUTINE)CaptureThreadFunc, &VST3D_RESULT, 0, &ThreadID);
//	DWORD hResult = WaitForSingleObject(hThread, CAPTURE_TIME_OUT); // Set waiting time
//	if (hResult == WAIT_OBJECT_0)
//	{
//		return VST3D_RESULT;
//	}
//	else if (hResult == WAIT_TIMEOUT)
//	{
//		return VST3D_RESULT_ERROR; // 超时
//	}
//	else
//	{
//		return VST3D_RESULT_ERROR;
//	}
//}

#ifdef VST_SINGLE_SCAN
int _tmain(int argc, _TCHAR *argv[])
{
	//int result = VST3D_Init(); // default path, start with software's window
	int result = VST3D_Init(install_path, true);  // default path, start without software's window
	if (result != VST3D_RESULT_OK)
	{
		printf("Could not Start Scanner software.\n");
		VST3D_Exit();
		return -1;
	}

	result = VST3D_Connect(); // Self-check and connect
	if (result != VST3D_RESULT_OK)
	{
		printf("Could not connect to Scanner.\n");
		VST3D_Exit();
		return -1;
	}

	int N = 0;
	int lossN = 0;

	int totalGroup = 1; // 设置连续扫描的次数
	vector<int> vGroupNumRecord(totalGroup);

	/////////////////////////////////////获取相机内参和外参数////////////////////////////////////
	//
	//	double* calparas = nullptr;
	//	VST3D_Output_CalPara(&calparas);

	//	// 130相机Sensor像素大小 dx = 4.8μm
	//	double dx, dy;
	//	dx = dy = 4.8;

	//	// 左相机内参 KK
	//	double cx_l = calparas[6];
	//	double cy_l = calparas[7];

	//	double fx_l = calparas[8];  // 像素焦距
	//	double fy_l = calparas[9];

	//	double fx_lmm = (fx_l*dx) / 1000;   // 实际焦距 mm
	//	double fy_lmm = (fy_l*dy) / 1000;

	//	// 左相机畸变参数 径向畸变
	//	double k1_l = calparas[10];
	//	double k2_l = calparas[11];

	//	// 左相机畸变参数 切向畸变
	//	double p1_l = calparas[12];
	//	double p2_l = calparas[13];

	//	// 左相机姿态
	//	double pose_l[6] = { 0 };

	//	//*****************************************************

	//	// 右相机内参 KK
	//	double cx_r = calparas[14];
	//	double cy_r = calparas[15];

	//	double fx_r = calparas[16];
	//	double fy_r = calparas[17];

	//	double fx_rmm = (fx_r * dx) / 1000;   // 实际焦距 mm
	//	double fy_rmm = (fy_r * dy) / 1000;

	//	// 右相机畸变参数 径向畸变
	//	double k1_r = calparas[18];
	//	double k2_r = calparas[19];

	//	// 右相机畸变参数 切向畸变
	//	double p1_r = calparas[20];
	//	double p2_r = calparas[21];

	//	// 右相机姿态  model: Pr = R*Pl + T     pose {Tx,Ty,Tz,Omx,Omy,Omz} {平移向量 + 旋转向量}表示法
	//	double pose_r[6] = { calparas[3], calparas[4], calparas[5], calparas[0], calparas[1], calparas[2] };

	//	/////////////////////////////////////////////////////////////
	// #ifdef VST_SNAPMARK
	// result = VST3D_SetCircleMarkSnap();   //  扫描时，输出圆标记点空间位置信息，调用两次可以取消这个功能。
	// if (result != VST3D_RESULT_OK)
	//{
	//	return -1;
	// }
	// #endif

	while (N < totalGroup)
	{
		VST3D_SetCameraID((N % 0)); // 多组3D 扫描头时，可以根据ID号切换。
		result = VST3D_Scan();		//  开始单次扫描
		//result = Capture3DData();	//  开始多线程单次扫描，设置超时

		if (result != VST3D_RESULT_OK)
		{
			printf("Scan error...\n");
			int nreset = 0;
			while (true)
			{
				Sleep(CAPTURE_TIME_OUT); // 每次重新连接前延时几秒，相机，光栅硬件驱动加载需要时间

				// 单机扫描情况
				result = VST3D_Reset(install_path); // 单机扫描重新连接扫描仪系统，重启

				if (result != VST3D_RESULT_OK)
				{
					printf("Check cables connected to Scanner.\n"); // 重新连接失败，继续尝试
				}
				else
				{
					break; // 重新连接成功，重新扫描
				}
				if (nreset++ > 5) // 做5次重新连接尝试，如果不能正常连接扫描仪，退出软件系统
				{
					VST3D_Exit();
					return result;
				}
			}
		}

		int totalNum = 0;
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
			fstream file(filename, ios::out);

#ifdef VST_SNAPMARK
			//////////////////////////获得空间标记点3D坐标/////////////////////////////////////////
			int totalMarker = 0;
			VST3D_PT *pMarkers = nullptr;
			result = VST3D_GetMarkersData(totalMarker, &pMarkers); // 得到当前单次扫描总标记点数和点坐标和法向量

			VST3D_PT *myMarkers = new VST3D_PT[totalMarker];
			memcpy((void *)myMarkers, (void *)(pMarkers), totalMarker * sizeof(VST3D_PT));
			for (int i = 0; i < totalMarker; i++) // 遍历拷贝采集到的所有3D Marker信息，通过索引方式[0 1 2 ....]
			{
				VST3D_PT &mark = myMarkers[i];
				float x, y, z, nx, ny, nz;
				x = mark.x;
				y = mark.y;
				z = mark.z;
				nx = mark.nx;
				ny = mark.ny;
				nz = mark.nz;
				cout << x << "," << y << "," << z << "," << nx << "," << ny << "," << nz << endl;
				// system("pause");
			}
			if (myMarkers) // 清内存
			{
				delete[] myMarkers;
			}
#endif

			/////////////////////////////////////////////////////////////////////////////////

			///////////////////////// 方法1：获得乱序点云///////////////////////////////////////
#ifdef VST_UnorderedPointClouds
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
				// cout << x << "," << y << "," << z << endl;
				// system("pause");
			}
			if (myPts)
			{
				delete[] myPts;
			}

			// method 2: Get each point by index
			vGroupNumRecord[N] = totalNum;
			for (int i = 0; i < totalNum; i++) // 遍历单次采集到的所有乱序点云，得到点云坐标等信息，通过索引方式[0 1 2 ....]
			{
				VST3D_PT *pt = nullptr;
				VST3D_GetEachPointByIndex(i, &pt);
				float x, y, z, nx, ny, nz;
				float cr, cg, cb;
				x = pt->x;
				y = pt->y;
				z = pt->z;
				nx = pt->nx;
				ny = pt->ny;
				nz = pt->nz;
				cr = (float)pt->cr / 255;
				cg = (float)pt->cg / 255;
				cb = (float)pt->cb / 255;
				file << x << " " << y << " " << z << endl;
				//			file << x << " " << y << " " << z << " " << nx << " " << ny << " " << nz << endl;
				//			file << x << " " << y << " " << z << " " << cr << " " << cg << " " << cb << " " << nx << " " << ny << " " << nz << endl;
			}
			// file.close();
			////////////////////////////////////////////////////////////////////////
			// char Imagename[64];
			// sprintf_s(Imagename, "d:\\test\\Image%d.bmp", N);  // 采集的左相机图片 临时保存为bmp文件
			// unsigned char* pImag = nullptr;
			// int width = 0;
			// int height = 0;
			// VST3D_GetImage(&pImag, width, height);  // 获取单次采集的左相机图片
			// Write24BitImg2BmpFile(pImag, width, height, Imagename); // 保存图片到相应的位置，可以用来进行2D图像内容检测分析，得到点云在3D中位置。

#endif
			///////////////////////// 方法2：获取左相机照片和点云图对应关系///////////////////////////////////////
#ifdef VST_OrganizedPointClouds

			int width = 0;
			int height = 0;
			VST3D_GeneratePointMap(); // 生成点云和图像对应关系点云图

			float *xMap = nullptr;
			float *yMap = nullptr;
			float *zMap = nullptr;

			VST3D_Output_PointMap(&xMap, &yMap, &zMap, width, height, true);

			int cnt = 0;
			for (int i = 0; i < height; i++)
			{
				for (int j = 0; j < width; j++)
				{
					int idx = i * width + j;
					if (!_isnan(xMap[idx]))
					{
						cnt++;
						// std::cout << "["<<i<<","<<j<<"]:"<<xMap[idx] << "," << yMap[idx] << "," << zMap[idx] << endl;
						file << xMap[idx] << " " << yMap[idx] << " " << zMap[idx] << endl;
					}
				}
			}

			file.close();
			cout << "# " << N << " PointMap num is :" << cnt << endl;

			////////////////////////////////////////////////////////////////////////
			//  得到具体像素坐标处的点云图
			char Imagename[64];
			sprintf_s(Imagename, "d:\\test\\Image%d.bmp", N); // 采集的左相机图片 临时保存为bmp文件
			unsigned char *pImag = nullptr;
			VST3D_GetImage(&pImag, width, height);					// 获取单次采集的左相机图片
			Write24BitImg2BmpFile(pImag, width, height, Imagename); // 保存图片到相应的位置，可以用来进行2D图像内容检测分析，得到点云在3D中位置。

#endif
			//////////////////////////////////////////////////////////////////////////////////
		}
		cout << "# " << N << " total num is :" << totalNum << endl;
		N++;
	}

	cout << endl;
	cout << "# Total Loss Frame Num is :" << lossN << endl;

	VST3D_Exit(); // 退出扫描操作，卸载设备软件
	system("pause");

	return 0;
}
#endif

#ifdef VST_ALIGN_SCAN
int _tmain(int argc, _TCHAR *argv[])
{

	// int result = VST3D_Init(); //      缺省安装目录启动，正常窗口启动
	int result = VST3D_Init(install_path, false); // 缺省安装目录启动,隐藏软件窗口
	if (result != VST3D_RESULT_OK)
	{
		printf("Could not Start Scanner software.\n");
		VST3D_Exit();
		return -1;
	}
	result = VST3D_Connect_ALIGN(); // 正常单幅扫描一次，并识别标记点，同时进行点云旋转对齐
	if (result != VST3D_RESULT_OK)
	{
		printf("Could not connect to Scanner.\n");
		VST3D_Exit();
		return -1;
	}

	// float* plane = nullptr;
	// result = VST3D_DelBackground(&plane);  // 设置扫描基准平面后，调用去除基准面以下的杂点
	// if (result != VST3D_RESULT_OK)
	//{
	//	printf("Could not Set backgroud palne parameters.\n");
	//	VST3D_Exit();
	//	return -1;
	// }
	// float cen[3] = { 0.0f };  // 平面的中心坐标
	// float normals[3] = { 0.0f }; // 平面的法向量
	// cen[0] = plane[3];
	// cen[1] = plane[4];
	// cen[2] = plane[5];
	// normals[0] = plane[0];
	// normals[1] = plane[1];
	// normals[2] = plane[2];

	int N = 0;
	int lossN = 0;

	int totalGroup = 5;
	vector<int> vGroupNumRecord(totalGroup);
	vector<vector<VST3D_PT>> vGroupPts(totalGroup);
	while (N < totalGroup)
	{
		// result = VST3D_Scan(); //      开始单次扫描
		result = Capture3DData(); //     开始多线程单次扫描，设置超时
		if (result != VST3D_RESULT_OK)
		{
			printf("Scan error...\n");
			int nreset = 0;
			while (true)
			{
				Sleep(CAPTURE_TIME_OUT); // 每次重新连接前延时几秒，相机，光栅硬件驱动加载需要时间

				// 单机拼接扫描情况
				result = VST3D_Reset_ALIGN(); // 单机扫描重新连接扫描仪系统，重启

				if (result != VST3D_RESULT_OK)
				{
					printf("Check cables connected to Scanner.\n"); // 重新连接失败，继续尝试
				}
				else
				{
					break; // 重新连接成功，重新扫描
				}
				if (nreset++ > 5) // 做5次重新连接尝试，如果不能正常连接扫描仪，退出软件系统
				{
					VST3D_Exit();
					return result;
				}
			}
		}
		int totalNum = 0;
		result = VST3D_GetNumPoints(totalNum); // 得到当前单次扫描总点数
		if (result != VST3D_RESULT_OK)
		{
			lossN++;
			printf("Frame loss in 3D acquisition, try it again...\n");
		}
		else
		{
			///////////////////////// 获得乱序点云///////////////////////////////////////
			int totalNum = 0;
			VST3D_PT *pPointClouds = nullptr;
			result = VST3D_GetPointClouds(totalNum, &pPointClouds); // 得到当前单次扫描总点数和乱序的点云
			vGroupNumRecord[N] = totalNum;
			for (int i = 0; i < totalNum; i++) // 遍历单次采集到的所有乱序点云，得到点云坐标等信息，通过索引方式[0 1 2 ....]
			{
				VST3D_PT *pt = nullptr;
				VST3D_GetEachPointByIndex(i, &pt);
				VST3D_PT cur_pt;
				cur_pt.x = pt->x;
				cur_pt.y = pt->y;
				cur_pt.z = pt->z;
				cur_pt.nx = pt->nx;
				cur_pt.ny = pt->ny;
				cur_pt.nz = pt->nz;
				cur_pt.cr = pt->cr;
				cur_pt.cg = pt->cg;
				cur_pt.cb = pt->cb;
				vGroupPts[N].push_back(cur_pt);
			}
			/////////////////////////////////////////////////////////////////////////////////////////////////
		}
		std::cout << "# " << N << " total num is :" << totalNum << endl;

		std::system("pause"); // 停一下，可以旋转物体角度，再次扫描

		N++;
	}

	///////////////// Get Rotation and Translation in align scaning //获取每次移动时的旋转和平移关系
	double *para = nullptr;
	double *para_eular = nullptr;
	result = VST3D_GetOdoMeterRotationTrans(&para);
	if (result != VST3D_RESULT_OK)
	{
		printf("Could not Get Rotation and Translation vector...\n");
		VST3D_Exit();
		return -1;
	}
	for (size_t i = 0; i < totalGroup; i++)
	{
		double *buff = reinterpret_cast<double *>((unsigned char *)para + i * 12 * sizeof(double));
		double R[3][3] = {0.0};
		double T[3] = {0.0};
		for (size_t n = 0; n < 3; n++)
		{
			for (size_t m = 0; m < 3; m++)
			{
				R[n][m] = buff[n * 3 + m];
			}
			T[n] = buff[9 + n];
		}
	}

	result = VST3D_GetOdoMeterEularTrans(&para_eular); // RPY Z-Y-X
	if (result != VST3D_RESULT_OK)
	{
		printf("Could not Get Eular and Translation vector...\n");
		VST3D_Exit();
		return -1;
	}

	for (size_t i = 0; i < totalGroup; i++)
	{
		double *buff = reinterpret_cast<double *>((unsigned char *)para_eular + i * 6 * sizeof(double));
		double eular[3] = {0.0};
		double T[3] = {0.0};
		for (size_t n = 0; n < 3; n++)
		{
			eular[n] = buff[n];
			T[n] = buff[3 + n];
		}
	}

	////////////////////////////////////////////////////////////////////

	// Remove all overlay points from multiple surface scanning
	result = VST3D_RemoveOverLayPoints(); // 开始去除重叠点云
	if (result != VST3D_RESULT_OK)
	{
		printf("Could not scan...\n");
		VST3D_Exit();
		return -1;
	}

	char filename[64];
	sprintf_s(filename, "d:\\FilterdPoints.asc"); // 3D points
	fstream file(filename, ios::out);

	bool *state = nullptr;
	VST3D_GetTotalPointState(&state); // 去除重叠点云后，判断点是否被保留，或者是否被去除
	int idx = 0;
	int icounter = 0;
	for (size_t i = 0; i < totalGroup; i++)
	{
		int totalNum = vGroupNumRecord[i];
		for (size_t j = 0; j < totalNum; j++)
		{
			bool curentState = *(reinterpret_cast<bool *>((unsigned char *)state + idx * sizeof(bool) + j * sizeof(bool)));
			if (curentState) // 保留下来的点
			{
				icounter++;
				VST3D_PT &cur_pt = vGroupPts[i][j];
				file << cur_pt.x << " " << cur_pt.y << " " << cur_pt.z << endl;
			}
		}
		idx += totalNum;
	}
	file.close();
	std::cout << endl;
	std::cout << "# Total Loss Frame Num is :" << lossN << endl;
	std::system("pause");

	VST3D_Exit(); // 退出扫描操作，卸载设备软件
	return 0;
}
#endif

#ifdef VST_ARRAY_SCAN
int _tmain(int argc, _TCHAR *argv[])
{
	// int result = VST3D_Init(); //      缺省安装目录启动，正常窗口启动
	int result = VST3D_Init(install_path, true); // 缺省安装目录启动,隐藏软件窗口
	if (result != VST3D_RESULT_OK)
	{
		printf("Could not Start Scanner software.\n");
		VST3D_Exit();
		return -1;
	}

	result = VST3D_Connect_ARRAY(); // 正常阵列扫描，连上多台扫描仪并进行自检
	if (result != VST3D_RESULT_OK)
	{
		printf("Could not connect to Scanner.\n");
		VST3D_Exit();
		return -1;
	}

	int N = 0;
	int lossN = 0;

	int totalGroup = 10;
	vector<int> vGroupNumRecord(totalGroup);

	while (N < totalGroup)
	{
		// result = VST3D_Scan();  //  开始单次扫描
		result = Capture3DData(); // 开始多线程单次扫描，设置超时
		if (result != VST3D_RESULT_OK)
		{
			printf("Scan error...\n");
			int nreset = 0;
			while (true)
			{
				Sleep(CAPTURE_TIME_OUT); // 每次重新连接前延时几秒，相机，光栅硬件驱动加载需要时间

				// 阵列扫描情况
				result = VST3D_Connect_ARRAY(); // 阵列扫描重新尝试扫描，不重启扫描软件，速度快
				// result = VST3D_Reset_ARRAY();  // 阵列扫描重新连接扫描仪系统，重启扫描软件，速度慢

				if (result != VST3D_RESULT_OK)
				{
					printf("Check cables connected to Scanner.\n"); // 重新连接失败，继续尝试
				}
				else
				{
					break; // 重新连接成功，重新扫描
				}
				if (nreset++ > 5) // 做5次重新连接尝试，如果不能正常连接扫描仪，退出软件系统
				{
					VST3D_Exit();
					return result;
				}
			}
		}
		// Sleep(RECONNECT_TIME_OUT); // 测试单次单幅扫描掉线，重新连接前延时几秒，各类硬件驱动加载需要时间

		int totalNum = 0;
		result = VST3D_GetNumPoints(totalNum); // 得到当前单次扫描总点数
		if (result != VST3D_RESULT_OK)
		{
			lossN++;
			printf("Frame loss in 3D acquisition, try it again...\n");
		}
		else
		{
			char filename[64];
			sprintf_s(filename, "d:\\test\\points%d.asc", N); // 3D points
			fstream file(filename, ios::out);

			///////////////////////// 获得阵列乱序点云///////////////////////////////////////
			int totalNum = 0;
			VST3D_PT *pPointClouds = nullptr;
			result = VST3D_GetPointClouds(totalNum, &pPointClouds); // 得到当前单次扫描总点数和乱序的点云
			vGroupNumRecord[N] = totalNum;
			for (int i = 0; i < totalNum; i++) // 遍历单次采集到的所有乱序点云，得到点云坐标等信息，通过索引方式[0 1 2 ....]
			{
				VST3D_PT *pt = nullptr;
				VST3D_GetEachPointByIndex(i, &pt);
				float x, y, z, nx, ny, nz;
				float cr, cg, cb;
				x = pt->x;
				y = pt->y;
				z = pt->z;
				nx = pt->nx;
				ny = pt->ny;
				nz = pt->nz;
				cr = (float)pt->cr / 255;
				cg = (float)pt->cg / 255;
				cb = (float)pt->cb / 255;
				file << x << " " << y << " " << z << endl;
				//	file << x << " " << y << " " << z << " " << cr << " " << cg << " " << cb << " " << nx << " " << ny << " " << nz << endl;
			}
			file.close();
			/////////////////////////////////////////////////////////////////////////////////////////////////
		}
		cout << "# " << N << " total num is :" << totalNum << endl;
		N++;
	}
	cout << endl;
	cout << "# Total Loss Frame Num is :" << lossN << endl;

	VST3D_Exit(); // 退出扫描操作，卸载设备软件
	system("pause");
	return 0;
}
#endif

//////   Testint points from file//////////
// int ptSize = 0;
// VST3D_PT * pts = nullptr;
// ReadPointsFromFile("points0.asc", ptSize, &pts);     // note after operation, points memory need to releaseed.
// for (size_t i = 0; i < ptSize; i++)
//{
//	VST3D_PT& pt = pts[i];
//	float x, y, z/*, nx, ny, nz*/;
//	// float cr, cg, cb;
//	x = pt.x;
//	y = pt.y;
//	z = pt.z;
// }
// if (pts != NULL)       // release memory
//{
//	delete[] pts;
//	pts = nullptr;
// }
//
///////////////
//
// const char* exename = "C:\\code\\VS2017\\Falcon\\x64\\Release\\VisenTOP Studio.exe";
