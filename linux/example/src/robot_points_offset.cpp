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

	JointPos j1, j2;
	DescPose desc_pos1, desc_pos2, offset_pos, offset_pos1;
	ExaxisPos  epos;

	memset(&j1, 0, sizeof(JointPos));
	memset(&j2, 0, sizeof(JointPos));
	memset(&desc_pos1, 0, sizeof(DescPose));
	memset(&desc_pos2, 0, sizeof(DescPose));
	memset(&offset_pos, 0, sizeof(DescPose));
	memset(&offset_pos1, 0, sizeof(DescPose));
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

	offset_pos1.tran.x = 0;
	offset_pos1.tran.y = 0;
	offset_pos1.tran.z = 100;
	offset_pos1.rpy.rx = 0;
	offset_pos1.rpy.ry = 0;
	offset_pos1.rpy.rz = 0;

	int tool = 0;
	int user = 0;
	float vel = 70.0;
	float acc = 100.0;
	float ovl = 70.0;
	float blendT = -1.0;
	float blendR = 0.0;
	uint8_t flag = 0;
	int type = 0;

	robot.SetSpeed(20);

	int ret = 0;
	ret = robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
	printf("MoveJ ret = %d\n", ret);
	ret = robot.MoveJ(&j2, &desc_pos2, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
	printf("MoveJ ret = %d\n", ret);

	std::this_thread::sleep_for(std::chrono::milliseconds(2000));    //ms
	ret = robot.PointsOffsetEnable(type, &offset_pos1);
	printf("PointsOffsetEnable ret is:%d\n", ret);

	ret = robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
	printf("MoveJ ret = %d\n", ret);
	ret = robot.MoveJ(&j2, &desc_pos2, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
	printf("MoveJ ret = %d\n", ret);

	ret = robot.PointsOffsetDisable();
	printf("PointsOffsetDisable ret is:%d\n", ret);

	return 0;
}