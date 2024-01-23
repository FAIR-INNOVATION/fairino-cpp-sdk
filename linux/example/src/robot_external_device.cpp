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

	int company = 4;
	int device = 0;
	int softversion = 0;
	int bus = 1;
	int index = 1;
	int act = 0;
	int max_time = 30000;
	uint8_t block = 0;
	uint16_t fault = 0;
	uint8_t status = 0;
	uint16_t active_status = 0;
	uint8_t current_pos = 0;
	int8_t current = 0;
	int voltage = 0;
	int temp = 0;
	int8_t speed = 0;

	if (0)
	{
		robot.SetGripperConfig(company, device, softversion, bus);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		robot.GetGripperConfig(&company, &device, &softversion, &bus);
		printf("gripper config:%d,%d,%d,%d\n", company, device, softversion, bus);

		robot.ActGripper(index, act);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		robot.GetGripperActivateStatus(&fault, &active_status);
		printf("gripper active fault is: %u, status is: %u\n", fault, active_status);
		std::this_thread::sleep_for(std::chrono::milliseconds(3000));

		act = 1;
		robot.ActGripper(index, act);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		robot.GetGripperActivateStatus(&fault, &active_status);
		printf("gripper active fault is: %u, status is: %u\n", fault, active_status);
		std::this_thread::sleep_for(std::chrono::milliseconds(5000));

		// first move
		robot.MoveGripper(index, 100, 10, 10, max_time, block);
		robot.GetGripperCurPosition(&fault, &current_pos);
		printf("fault is:%u, current position is: %u\n", fault, current_pos);

		robot.GetGripperCurCurrent(&fault, &current);
		printf("fault is:%u, current current is: %d\n", fault, current);

		robot.GetGripperVoltage(&fault, &voltage);
		printf("fault is:%u, current voltage is: %d \n", fault, voltage);

		robot.GetGripperTemp(&fault, &temp);
		printf("fault is:%u, current temperature is: %d\n", fault, temp);

		robot.GetGripperCurSpeed(&fault, &speed);
		printf("fault is:%u, current speed is: %d\n", fault, speed);

		std::this_thread::sleep_for(std::chrono::milliseconds(3000));
		status = 0;
		robot.GetGripperMotionDone(&fault, &status);
		printf("motion status:%u,%u\n", fault, status);

		printf("\n");

		// secode move
		block = 1;
		robot.MoveGripper(index, 0, 10, 0, max_time, block);

		robot.GetGripperCurSpeed(&fault, &speed);
		printf("fault is:%u, current speed is: %d\n", fault, speed);

		robot.GetGripperCurPosition(&fault, &current_pos);
		printf("fault is:%u, current position is: %u\n", fault, current_pos);

		robot.GetGripperCurCurrent(&fault, &current);
		printf("fault is:%u, current current is: %d\n", fault, current);

		robot.GetGripperVoltage(&fault, &voltage);
		printf("fault is:%u, current voltage is: %d \n", fault, voltage);

		robot.GetGripperTemp(&fault, &temp);
		printf("fault is:%u, current temperature is: %d\n", fault, temp);

		printf("\n");

		std::this_thread::sleep_for(std::chrono::milliseconds(10000));
		status = 0;
		robot.GetGripperMotionDone(&fault, &status);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		printf("motion status:%u,%u\n", fault, status);

		act = 0;
		robot.ActGripper(index, act);
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		robot.GetGripperActivateStatus(&fault, &active_status);
		printf("gripper active fault is: %u, status is: %u\n", fault, active_status);
	}

	if (1)
	{
		int retval = 0;
		DescPose prepick_pose;
		DescPose postpick_pose;
		memset(&prepick_pose, 0, sizeof(DescPose));
		memset(&postpick_pose, 0, sizeof(DescPose));

		DescPose desc_p1;
		desc_p1.tran.x = -351.553;
		desc_p1.tran.y = 87.913;
		desc_p1.tran.z = 354.175;
		desc_p1.rpy.rx = -179.680;
		desc_p1.rpy.ry = -0.133;
		desc_p1.rpy.rz = 2.472;

		DescPose desc_p2;
		desc_p2.tran.x = -351.535;
		desc_p2.tran.y = -247.222;
		desc_p2.tran.z = 354.173;
		desc_p2.rpy.rx = -179.680;
		desc_p2.rpy.ry = -0.137;
		desc_p2.rpy.rz = 2.473;

		retval = robot.ComputePrePick(&desc_p1, 10, 0, &prepick_pose);
		printf("ComputePrePick retval is: %d\n", retval);
		printf("xyz is: %f, %f, %f; rpy is: %f, %f, %f\n", prepick_pose.tran.x, prepick_pose.tran.y, prepick_pose.tran.z,
			   prepick_pose.rpy.rx, prepick_pose.rpy.ry, prepick_pose.rpy.rz);

		retval = robot.ComputePostPick(&desc_p2, -10, 0, &postpick_pose);
		printf("ComputePostPick retval is: %d\n", retval);
		printf("xyz is: %f, %f, %f; rpy is: %f, %f, %f\n", postpick_pose.tran.x, postpick_pose.tran.y, postpick_pose.tran.z,
			   postpick_pose.rpy.rx, postpick_pose.rpy.ry, postpick_pose.rpy.rz);
	}

	return 0;
}