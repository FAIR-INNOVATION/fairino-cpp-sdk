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

	char name[30] = "tpd2023";
	int tool = 0;
	int user = 0;
	float vel = 50.0;
	float acc = 100.0;
	float ovl = 100.0;
	float blendT = -1.0;
	int config = -1;
	uint8_t blend = 0;
	int retval = 0;

	DescPose desc_pose;
	memset(&desc_pose, 0, sizeof(DescPose));
	DescPose start_pose;
	memset(&start_pose, 0, sizeof(DescPose));

	desc_pose.tran.x = 358.820099;
	desc_pose.tran.y = -419.684113;
	desc_pose.tran.z = 525.055115;
	desc_pose.rpy.rx = -85.994499;
	desc_pose.rpy.ry = -28.797600;
	desc_pose.rpy.rz = -133.960007;

	retval = robot.LoadTPD(name);
	printf("LoadTPD retval is: %d\n", retval);
	//robot.MoveCart(&desc_pose, tool, user, vel, acc, ovl, blendT, config);
	
	robot.GetTPDStartPose(name, &start_pose);
	printf("start pose, xyz is: %f %f %f. rpy is: %f %f %f \n", start_pose.tran.x, start_pose.tran.y, start_pose.tran.z, start_pose.rpy.rx, start_pose.rpy.ry, start_pose.rpy.rz);
	std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	
	retval = robot.MoveTPD(name, blend, ovl);
	printf("MoveTPD retval is: %d\n", retval);
	std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	return 0;
}