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
	VST3D_SetCameraID(0); // ����3D ɨ��ͷʱ�����Ը���ID���л���
}

void VisionSystem::scanOnce()
{
	int result = VST3D_Scan();		//  ��ʼ����ɨ��
	//result = Capture3DData();		//  ��ʼ���̵߳���ɨ�裬���ó�ʱ
	if (result != VST3D_RESULT_OK)
	{
		printf("Scan error...\n");
		retryCon();
	}
	
	int totalNum = 0, lossN = 0;
	result = VST3D_GetNumPoints(totalNum); // �õ���ǰ����ɨ���ܵ���
	if (result != VST3D_RESULT_OK)
	{
		lossN++;
		printf("Frame loss in 3D acquisition, try it again...\n");
	}
	else
	{
		char filename[64];
		sprintf_s(filename, "./points%d.txt", N); // 3D points ��ʱ����Ϊ�ļ�
		//fstream file(filename, ios::out);
		int totalNum = 0;
		VST3D_PT *pPointClouds = nullptr;
		result = VST3D_GetPointClouds(totalNum, &pPointClouds); // �õ���ǰ����ɨ���ܵ���������ĵ���

		// method 1 �� copy all points
		VST3D_PT *myPts = new VST3D_PT[totalNum];
		memcpy((void *)myPts, (void *)(pPointClouds), totalNum * sizeof(VST3D_PT));
		for (int i = 0; i < totalNum; i++) // ���������Ĳɼ���������������ƣ��õ������������Ϣ��ͨ��������ʽ[0 1 2 ....]
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
		// ÿ����������ǰ��ʱ���룬�������դӲ������������Ҫʱ��
		Sleep(CAPTURE_TIME_OUT); 

		// ����ɨ�����
		int result = VST3D_Reset(this -> installPath.data()); // ����ɨ����������ɨ����ϵͳ������

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
			//return result;
		}
	}
}
