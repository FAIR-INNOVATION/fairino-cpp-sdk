#include "libfairino/robot.h"
#ifdef WINDOWS_OPTION
#include <string.h>
#include <windows.h>
#elif LINUX_OPTION
#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <cstring>
#include <unistd.h>
#endif

#include <chrono>
#include <thread>

using namespace std;

int main(void)
{
	FRRobot robot;                 
	robot.RPC("192.168.58.2");     

	JointPos j1, j2, j3, j4;
	DescPose desc_pos1, desc_pos2, desc_pos3, desc_pos4, offset_pos;
	ExaxisPos  epos;

	memset(&j1, 0, sizeof(JointPos));
	memset(&j2, 0, sizeof(JointPos));
	memset(&j3, 0, sizeof(JointPos));
	memset(&j4, 0, sizeof(JointPos));
	memset(&desc_pos1, 0, sizeof(DescPose));
	memset(&desc_pos2, 0, sizeof(DescPose));
	memset(&desc_pos3, 0, sizeof(DescPose));
	memset(&desc_pos4, 0, sizeof(DescPose));
	memset(&offset_pos, 0, sizeof(DescPose));
	memset(&epos, 0, sizeof(ExaxisPos));

	j1 = { { 72.577, -67.27, 119.251, -136.726, -84.359, 9.484 } };
	desc_pos1.tran.x = -43;
	desc_pos1.tran.y = -510.509;
	desc_pos1.tran.z = 124.369;
	desc_pos1.rpy.rx = 173.868;
	desc_pos1.rpy.ry = 4.672;
	desc_pos1.rpy.rz = 152.584;

	j2 = { { 96.764, -79.41, 119.251, -124.725, -86.395, 9.491 } };
	desc_pos2.tran.x = 163.37;
	desc_pos2.tran.y = -458.036;
	desc_pos2.tran.z = 208.202;
	desc_pos2.rpy.rx = 174.353;
	desc_pos2.rpy.ry = 2.698;
	desc_pos2.rpy.rz = 176.979;

	j3 = { { 127.379, -97.378, 119.244, -109.431, -80.629, 9.568 } };
	desc_pos3.tran.x = 342.727;
	desc_pos3.tran.y = -253.774;
	desc_pos3.tran.z = 323.464;
	desc_pos3.rpy.rx = 176.001;
	desc_pos3.rpy.ry = 8.822;
	desc_pos3.rpy.rz = -152.698;

	j4 = { { 154.671, -64.046, 112.177, -149.892, -87.904, 9.549 } };
	desc_pos4.tran.x = 560.2845;
	desc_pos4.tran.y = -148.293;
	desc_pos4.tran.z = 162.943;
	desc_pos4.rpy.rx = -168.724;
	desc_pos4.rpy.ry = 3.963;
	desc_pos4.rpy.rz = -124.27;

	int tool = 0;
	int user = 0;
	float vel = 100.0;
	float acc = 100.0;
	float ovl = 100.0;
	float blendT = -1.0;
	float blendR = 0.0;
	uint8_t flag = 0;
	int type = 1;

	robot.SetSpeed(20);

	int err1 = robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
	printf("movej errcode:%d\n", err1);
	int ret = robot.NewSplineStart(type);
	printf("NewSplineStart ret = %d\n", ret);
	ret = robot.NewSplinePoint(&j1, &desc_pos1, tool, user, vel, acc, ovl, blendR, 0);
	printf("NewSplinePoint ret = %d\n", ret);

	ret = robot.NewSplinePoint(&j2, &desc_pos2, tool, user, vel, acc, ovl, blendR, 0);
	printf("NewSplinePoint ret = %d\n", ret);

	ret = robot.NewSplinePoint(&j3, &desc_pos3, tool, user, vel, acc, ovl, blendR, 0);
	printf("NewSplinePoint ret = %d\n", ret);

	ret = robot.NewSplinePoint(&j4, &desc_pos4, tool, user, vel, acc, ovl, blendR, 1);
	printf("NewSplinePoint ret = %d\n", ret);
	ret = robot.NewSplineEnd();
	printf("NewSplineEnd ret = %d\n", ret);

	return 0;
}