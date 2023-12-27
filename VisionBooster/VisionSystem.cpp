#include "VisionSystem.h"

VisionSystem::VisionSystem(std::string installPath)
	:captureMethod(2)
{
	this->installPath = installPath;
}

VisionSystem::~VisionSystem()
{

}

void VisionSystem::initVisionSystem()
{
	int result = VST3D_Init(this->installPath.data(), true); // default path, start without software's window
	if (result != VST3D_RESULT_OK)
	{
		printf("Could not Start Scanner software.\n");
		VST3D_Exit();
	}

	result = VST3D_Connect(); // Self-check and connect
	if (result != VST3D_RESULT_OK)
	{
		printf("Could not connect to Scanner.\n");
		VST3D_Exit();
	}
	VST3D_SetCameraID(0); // check camera by ID
}

bool VisionSystem::scanOnce(std::vector<VST3D_PT> & VSTPoints)
{
	int result = VST3D_Scan(); //  Start scanning once
	if (result != VST3D_RESULT_OK)
	{
		retryCon("VST3D_Scan", __LINE__, __FILE__);
		return false;
	}

	int totalNum = 0;
	result = VST3D_GetNumPoints(totalNum); // obtain number of point at once
	if (result != VST3D_RESULT_OK)
	{
		retryCon("VST3D_GetNumPoints", __LINE__, __FILE__);
		return false;
	}

	VST3D_PT *pPointClouds = nullptr;
	result = VST3D_GetPointClouds(totalNum, &pPointClouds); // �õ���ǰ����ɨ���ܵ���������ĵ���

	// Method 1
	if (captureMethod == 1)
	{
		VST3D_PT * myPts = new VST3D_PT[totalNum];
		if (myPts)
		{
			memcpy((void *)myPts, (void *)(pPointClouds), totalNum * sizeof(VST3D_PT));
			VST3D_PT tmp;
			for (int i = 0; i < totalNum; i++) // ���������Ĳɼ���������������ƣ��õ������������Ϣ��ͨ��������ʽ[0 1 2 ....]
			{
				VST3D_PT &pt = myPts[i];
				//float x, y, z, nx, ny, nz;
				//float cr, cg, cb;
				tmp.x = pt.x;
				tmp.y = pt.y;
				tmp.z = pt.z;
				tmp.nx = pt.nx;
				tmp.ny = pt.ny;
				tmp.nz = pt.nz;
				tmp.cr = (float)pt.cr / 255;
				tmp.cg = (float)pt.cg / 255;
				tmp.cb = (float)pt.cb / 255;
				//file << x << " " << y << " " << z << " " << cr << " " << cg << " " << cb << " " << nx << " " << ny << " " << nz << endl;
				VSTPoints.push_back(tmp);
			}
			delete[] myPts;
		}
	}
	else if (captureMethod == 2)
	{
		// Method 2
		VST3D_PT tmp;
		for (int i = 0; i < totalNum; i++) // �������βɼ���������������ƣ��õ������������Ϣ��ͨ��������ʽ[0 1 2 ....]
		{
			VST3D_PT *pt = nullptr;
			VST3D_GetEachPointByIndex(i, &pt);
			//float x, y, z, nx, ny, nz;
			//float cr, cg, cb;
			tmp.x = pt->x;
			tmp.y = pt->y;
			tmp.z = pt->z;
			tmp.nx = pt->nx;
			tmp.ny = pt->ny;
			tmp.nz = pt->nz;
			tmp.cr = (float)pt->cr / 255;
			tmp.cg = (float)pt->cg / 255;
			tmp.cb = (float)pt->cb / 255;
			VSTPoints.push_back(tmp);
		}
	}

	return true;
}

void VisionSystem::retryCon(const std::string errorStr, const int codeLine, const std::string codeFile)
{
	printf("Error(%s) at line number %d in file %s\n", errorStr.data(), __LINE__, __FILE__);

	int nreset = 0;
	while (true)
	{
		// ÿ����������ǰ��ʱ���룬�������դӲ������������Ҫʱ��
		Sleep(CAPTURE_TIME_OUT);

		// ����ɨ�����
		int result = VST3D_Reset(this->installPath.data()); // ����ɨ����������ɨ����ϵͳ������

		if (result != VST3D_RESULT_OK)
		{
			printf("Check cables connected to Scanner.\n"); // ��������ʧ�ܣ���������
		}
		else
		{
			// �������ӳɹ�������ɨ��
			break;
		}
		if (nreset++ > 5) // ��5���������ӳ��ԣ����������������ɨ���ǣ��˳����ϵͳ
		{
			VST3D_Exit();
			// return result;
		}
	}
}

void VisionSystem::disConnect()
{
	VST3D_Exit();
}
