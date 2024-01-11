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

	j1 = { 114.578, -117.798, -97.745, -54.436, 90.053, -45.216 };
	desc_pos1.tran.x = -140.418;
	desc_pos1.tran.y = 619.351;
	desc_pos1.tran.z = 198.369;
	desc_pos1.rpy.rx = -179.948;
	desc_pos1.rpy.ry = 0.023;
	desc_pos1.rpy.rz = 69.793;

	j2 = { 115.401, -105.206, -117.959, -49.727, 90.054, -45.222 };
	desc_pos2.tran.x = -95.586;
	desc_pos2.tran.y = 504.143;
	desc_pos2.tran.z = 186.880;
	desc_pos2.rpy.rx = 178.001;
	desc_pos2.rpy.ry = 2.091;
	desc_pos2.rpy.rz = 70.585;

	j3 = { 135.609, -103.249, -120.211, -49.715, 90.058, -45.219 };
	desc_pos3.tran.x = -252.429;
	desc_pos3.tran.y = 428.903;
	desc_pos3.tran.z = 188.492;
	desc_pos3.rpy.rx = 177.804;
	desc_pos3.rpy.ry = 2.294;
	desc_pos3.rpy.rz = 90.782;

	j4 = { 154.766, -87.036, -135.672, -49.045, 90.739, -45.223 };
	desc_pos4.tran.x = -277.255;
	desc_pos4.tran.y = 272.958;
	desc_pos4.tran.z = 205.452;
	desc_pos4.rpy.rx = 179.289;
	desc_pos4.rpy.ry = 1.765;
	desc_pos4.rpy.rz = 109.966;

	int tool = 0;
	int user = 0;
	float vel = 100.0;
	float acc = 100.0;
	float ovl = 100.0;
	float blendT = -1.0;
	uint8_t flag = 0;

	robot.SetSpeed(20);

	int err1 = robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
	printf("movej errcode:%d\n", err1);
	robot.SplineStart();
	robot.SplinePTP(&j1, &desc_pos1, tool, user, vel, acc, ovl);
	robot.SplinePTP(&j2, &desc_pos2, tool, user, vel, acc, ovl);
	robot.SplinePTP(&j3, &desc_pos3, tool, user, vel, acc, ovl);
	robot.SplinePTP(&j4, &desc_pos4, tool, user, vel, acc, ovl);
	robot.SplineEnd();

	return 0;
}