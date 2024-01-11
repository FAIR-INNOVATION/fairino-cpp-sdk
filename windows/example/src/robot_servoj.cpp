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
#include <string.h>
#endif

#include <chrono>
#include <thread>

using namespace std;

int main(void)
{
	FRRobot robot;			   
	robot.RPC("192.168.58.2"); 

	JointPos j;

	memset(&j, 0, sizeof(JointPos));

	float vel = 0.0;
	float acc = 0.0;
	float cmdT = 0.004;
	float filterT = 0.0;
	float gain = 0.0;
	uint8_t flag = 1;
	int count = 1000;
	double dt = 0.1;

	int ret = robot.GetActualJointPosDegree(flag, &j);
	if (ret == 0)
	{
		ret = robot.ServoMoveStart();
		if (0 != ret)
		{
			printf("ServoMoveStart errcode:%d\n", ret);
			return 0;
		}
		printf("servoJ ServoMoveStart start...\n");
		while (count)
		{
			int ret = robot.ServoJ(&j, acc, vel, cmdT, filterT, gain);
			j.jPos[0] += dt;
			count -= 1;
		}
		ret = robot.ServoMoveEnd();
		if (0 != ret)
		{
			printf("ServoMoveStart errcode:%d\n", ret);
		}
		printf("servoJ ServoMoveEnd\n");
	}
	else
	{
		printf("GetActualJointPosDegree errcode:%d\n", ret);
	}
	return 0;
}