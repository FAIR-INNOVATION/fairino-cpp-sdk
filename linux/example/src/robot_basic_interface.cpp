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
	printf("start to debug\n");
	FRRobot robot;                 
	robot.RPC("192.168.58.2");     //Establish a communication connection with the robot controller

	char ip[64] = "";
	char version[64] = "";
	uint8_t state;

	robot.GetSDKVersion(version);
	printf("SDK version:%s\n", version);
	robot.GetControllerIP(ip);
	printf("controller ip:%s\n", ip);

	robot.Mode(1);
	this_thread::sleep_for(std::chrono::milliseconds(1000));   //unit: ms
	robot.DragTeachSwitch(1);
	robot.IsInDragTeach(&state);
	printf("drag state :%u\n", state);
	this_thread::sleep_for(std::chrono::milliseconds(3000));
	robot.DragTeachSwitch(0);
	this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.IsInDragTeach(&state);
	printf("drag state :%u\n", state);
	this_thread::sleep_for(std::chrono::milliseconds(3000));

	robot.RobotEnable(0);
	this_thread::sleep_for(std::chrono::milliseconds(3000));
	robot.RobotEnable(1);

	robot.Mode(0);
	this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.Mode(1);

	robot.CloseRPC();
	return 0;
}