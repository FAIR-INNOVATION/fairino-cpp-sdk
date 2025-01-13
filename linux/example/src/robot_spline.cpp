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
	FRRobot robot;                 //实例化机器人对象
	robot.RPC("192.168.58.2");     //与机器人控制器建立通信连接

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

	j1.jPos[0] = -78.011;
	j1.jPos[1] = -93.495;
	j1.jPos[2] = -110.064;
	j1.jPos[3] = -60.077;
	j1.jPos[4] = 65.601;
	j1.jPos[5] = 21.277;
	desc_pos1.tran.x = -36.435;
	desc_pos1.tran.y = -518.325;
	desc_pos1.tran.z = 339.133;
	desc_pos1.rpy.rx = -164.426;
	desc_pos1.rpy.ry = -20.019;
	desc_pos1.rpy.rz = 166.57;

	j2.jPos[0] = -34.08;
	j2.jPos[1] = -93.504;
	j2.jPos[2] = -110.073;
	j2.jPos[3] = -66.142;
	j2.jPos[4] = 91.713;
	j2.jPos[5] = 21.167;
	desc_pos2.tran.x = 350.773;
	desc_pos2.tran.y = -356.852;
	desc_pos2.tran.z = 318.755;
	desc_pos2.rpy.rx = 179.643;
	desc_pos2.rpy.ry = 1.699;
	desc_pos2.rpy.rz = -145.248;

	j3.jPos[0] = -18.239;
	j3.jPos[1] = -104.603;
	j3.jPos[2] = -99.5;
	j3.jPos[3] = -64.648;
	j3.jPos[4] = 92.438;
	j3.jPos[5] = 21.167;
	desc_pos3.tran.x = 512.541;
	desc_pos3.tran.y = -271.823;
	desc_pos3.tran.z = 304.302;
	desc_pos3.rpy.rx = -179.714;
	desc_pos3.rpy.ry = 2.725;
	desc_pos3.rpy.rz = -129.373;

	j4.jPos[0] = -4.627;
	j4.jPos[1] = -93.756;
	j4.jPos[2] = -114.223;
	j4.jPos[3] = -64.012;
	j4.jPos[4] = 92.451;
	j4.jPos[5] = 21.192;
	desc_pos4.tran.x = 465.707;
	desc_pos4.tran.y = -135.736;
	desc_pos4.tran.z = 287.378;
	desc_pos4.rpy.rx = 177.256;
	desc_pos4.rpy.ry = 1.564;
	desc_pos4.rpy.rz = -115.899;

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