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

	DescPose desc_pos1, desc_pos2, desc_pos3;
	memset(&desc_pos1, 0, sizeof(DescPose));
	memset(&desc_pos2, 0, sizeof(DescPose));
	memset(&desc_pos3, 0, sizeof(DescPose));

	desc_pos1.tran.x = 75.414;
	desc_pos1.tran.y = 568.526;
	desc_pos1.tran.z = 338.135;
	desc_pos1.rpy.rx = -178.348;
	desc_pos1.rpy.ry = -0.930;
	desc_pos1.rpy.rz = 52.611;

	desc_pos2.tran.x = -273.856;
	desc_pos2.tran.y = 643.260;
	desc_pos2.tran.z = 259.235;
	desc_pos2.rpy.rx = -177.972;
	desc_pos2.rpy.ry = -1.494;
	desc_pos2.rpy.rz = 80.866;

	desc_pos3.tran.x = -423.044;
	desc_pos3.tran.y = 229.703;
	desc_pos3.tran.z = 241.080;
	desc_pos3.rpy.rx = -173.990;
	desc_pos3.rpy.ry = -5.772;
	desc_pos3.rpy.rz = 123.971;

	int tool = 0;
	int user = 0;
	float vel = 100.0;
	float acc = 100.0;
	float ovl = 100.0;
	float blendT = -1.0;
	float blendT1 = 0.0;
	int config = -1;

	robot.SetSpeed(20);
	robot.MoveCart(&desc_pos1, tool, user, vel, acc, ovl, blendT, config);
	robot.MoveCart(&desc_pos2, tool, user, vel, acc, ovl, blendT, config);
	robot.MoveCart(&desc_pos3, tool, user, vel, acc, ovl, blendT1, config);

	return 0;
}