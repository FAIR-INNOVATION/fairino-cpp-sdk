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

	DescPose desc_pos1, desc_pos2, desc_pos3;
	memset(&desc_pos1, 0, sizeof(DescPose));
	memset(&desc_pos2, 0, sizeof(DescPose));
	memset(&desc_pos3, 0, sizeof(DescPose));
	JointPos j1, j2, j3, j4;

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
	// desc_pos3.tran.x = 512.541;
	desc_pos3.tran.x = 1000000;
	desc_pos3.tran.y = -271.823;
	desc_pos3.tran.z = 304.302;
	desc_pos3.rpy.rx = -179.714;
	desc_pos3.rpy.ry = 2.725;
	desc_pos3.rpy.rz = -129.373;
	int tool = 0;
	int user = 0;
	float vel = 100.0;
	float acc = 100.0;
	float ovl = 100.0;
	float blendT = -1.0;
	float blendT1 = 0.0;
	int config = -1;
	int retval = 0;

	retval = robot.SetSpeed(20);
	
	retval = robot.MoveCart(&desc_pos1, tool, user, vel, acc, ovl, blendT, config);
	printf("movecart retval is: %d\n", retval);

	retval = robot.MoveCart(&desc_pos2, tool, user, vel, acc, ovl, blendT, config);
	printf("movecart retval is: %d\n", retval);

	retval = robot.MoveCart(&desc_pos3, tool, user, vel, acc, ovl, blendT1, config);
	printf("movecart retval is: %d\n", retval);

	return 0;
}