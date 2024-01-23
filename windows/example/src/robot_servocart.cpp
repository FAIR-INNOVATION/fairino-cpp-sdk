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

	DescPose desc_pos_dt;
	memset(&desc_pos_dt, 0, sizeof(DescPose));

	desc_pos_dt.tran.z = -0.5;
	float pos_gain[6] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 };
	int mode = 2;
	float vel = 0.0;
	float acc = 0.0;
	float cmdT = 0.008;
	float filterT = 0.0;
	float gain = 0.0;
	uint8_t flag = 0;
	int count = 100;

	robot.SetSpeed(50);

	while (count)
	{
		robot.ServoCart(mode, &desc_pos_dt, pos_gain, acc, vel, cmdT, filterT, gain);
		count -= 1;
		robot.WaitMs(cmdT * 1000);
	}

	return 0;
}