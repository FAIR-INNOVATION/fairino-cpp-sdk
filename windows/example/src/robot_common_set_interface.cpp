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
	FRRobot robot;			   // 实例化机器人对象
	robot.RPC("192.168.58.2"); // 与机器人控制器建立通信连接

	int i;
	float value;
	int tool_id, etool_id, user_id;
	int type;
	int install;
	int retval = 0;

	DescTran coord;
	DescPose t_coord, etcp, etool, w_coord;
	memset(&coord, 0, sizeof(DescTran));
	memset(&t_coord, 0, sizeof(DescPose));
	memset(&etcp, 0, sizeof(DescPose));
	memset(&etool, 0, sizeof(DescPose));
	memset(&w_coord, 0, sizeof(DescPose));

	if (1)
	{
		DescPose tool0_pose;
		memset(&tool0_pose, 0, sizeof(DescPose));
		//
		printf("SetToolPoint start\n");
		std::this_thread::sleep_for(std::chrono::seconds(3));
		for (int i = 1; i < 7; i++)
		{
			retval = robot.SetToolPoint(i);
			printf("SetToolPoint retval is: %d\n", retval);
			// std::this_thread::sleep_for(std::chrono::seconds(3));
		}
		// std::this_thread::sleep_for(std::chrono::seconds(3));
		printf("SetToolPoint end\n");
		retval = robot.ComputeTool(&tool0_pose);
		printf("ComputeTool retval is: %d\n", retval);
		printf("xyz is: %f, %f, %f; rpy is: %f, %f, %f\n", tool0_pose.tran.x, tool0_pose.tran.y, tool0_pose.tran.z,
			   tool0_pose.rpy.rx, tool0_pose.rpy.ry, tool0_pose.rpy.rz);

		DescPose tcp4_0_pose;
		memset(&tcp4_0_pose, 0, sizeof(DescPose));
		//
		for (int i = 1; i < 5; i++)
		{
			retval = robot.SetTcp4RefPoint(i);
			printf("SetTcp4RefPoint retval is: %d\n", retval);
		}
		retval = robot.ComputeTcp4(&tcp4_0_pose);
		printf("ComputeTcp4 retval is: %d\n", retval);
		printf("xyz is: %f, %f, %f; rpy is: %f, %f, %f\n", tcp4_0_pose.tran.x, tcp4_0_pose.tran.y, tcp4_0_pose.tran.z,
			   tcp4_0_pose.rpy.rx, tcp4_0_pose.rpy.ry, tcp4_0_pose.rpy.rz);

		DescPose extcp_0_pose;
		memset(&extcp_0_pose, 0, sizeof(DescPose));
		printf("SetExTCPPoint start\n");
		// std::this_thread::sleep_for(std::chrono::seconds(3));
		for (int i = 1; i < 7; i++)
		{
			retval = robot.SetExTCPPoint(i);
			printf("SetExTCPPoint retval is: %d\n", retval);
			//std::this_thread::sleep_for(std::chrono::seconds(3));
		}
		// std::this_thread::sleep_for(std::chrono::seconds(3));
		printf("SetExTCPPoint end\n");
		retval = robot.ComputeExTCF(&extcp_0_pose);
		printf("ComputeExTCF retval is: %d\n", retval);
		printf("xyz is: %f, %f, %f; rpy is: %f, %f, %f\n", extcp_0_pose.tran.x, extcp_0_pose.tran.y, extcp_0_pose.tran.z,
			   extcp_0_pose.rpy.rx, extcp_0_pose.rpy.ry, extcp_0_pose.rpy.rz);

		DescPose wobj_0_pose;
		memset(&wobj_0_pose, 0, sizeof(DescPose));
		for (int i = 1; i < 4; i++)
		{
			retval = robot.SetWObjCoordPoint(i);
			printf("SetWObjCoordPoint retval is: %d\n", retval);
		}
		retval = robot.ComputeWObjCoord(0, &wobj_0_pose);
		printf("ComputeWObjCoord retval is: %d\n", retval);
		printf("xyz is: %f, %f, %f; rpy is: %f, %f, %f\n", wobj_0_pose.tran.x, wobj_0_pose.tran.y, wobj_0_pose.tran.z,
			   wobj_0_pose.rpy.rx, wobj_0_pose.rpy.ry, wobj_0_pose.rpy.rz);
	}
	if (0)
	{
		retval = robot.SetLoadWeight(0.1);
		printf("SetLoadWeight retval is: %d\n", retval);

		coord.x = 0.1;
		coord.y = 0.1;
		coord.z = 0.1;

		retval = robot.SetLoadCoord(&coord);
		printf("SetLoadCoord retval is: %d\n", retval);
	}
	if (0)
	{

		retval = robot.SetSpeed(40);
		printf("setspeed retval is: %d\n", retval);

		for (i = 1; i < 21; i++)
		{
			robot.SetSysVarValue(i, i + 0.5);
			std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 单位ms
		}

		for (i = 1; i < 21; i++)
		{
			robot.GetSysVarValue(i, &value);
			printf("sys value:%f\n", value);
		}

		tool_id = 10;
		t_coord.tran.x = 1.0;
		t_coord.tran.y = 2.0;
		t_coord.tran.z = 3.0;
		t_coord.rpy.rx = 4.0;
		t_coord.rpy.ry = 5.0;
		t_coord.rpy.rz = 6.0;
		type = 0;
		install = 0;
		int ret = robot.SetToolCoord(tool_id, &t_coord, type, install);
		printf("SetToolCoord ret = %d\n", ret);
		ret = robot.SetToolList(tool_id, &t_coord, type, install);
		printf("SetToolCoord ret = %d\n", ret);

		tool_id = 11;
		etcp.tran.x = 100;
		etool.tran.x = 100;
		retval = robot.SetExToolCoord(tool_id, &etcp, &etool);
		printf("SetExToolCoord retval is: %d\n", retval);

		retval = robot.SetExToolList(tool_id, &etcp, &etool);
		printf("SetExToolList retval is: %d\n", retval);

		w_coord.tran.x = 11.0;
		w_coord.tran.y = 12.0;
		w_coord.tran.z = 13.0;
		w_coord.rpy.rx = 14.0;
		w_coord.rpy.ry = 15.0;
		w_coord.rpy.rz = 16.0;
		user_id = 12;
		retval = robot.SetWObjCoord(user_id, &w_coord);
		printf("SetWObjCoord retval is: %d\n", retval);

		retval = robot.SetWObjList(user_id, &w_coord);
		printf("SetWObjList retval is: %d\n", retval);

		retval = robot.SetRobotInstallAngle(0.0, 0.0);
		printf("SetRobotInstallAngle retval is: %d\n", retval);
		retval = robot.SetRobotInstallPos(1);
		printf("SetRobotInstallPos retval is: %d\n", retval);
	}

	return 0;
}