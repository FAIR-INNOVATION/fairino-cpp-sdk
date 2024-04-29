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
	//robot.RPC("192.168.58.2");     //与机器人控制器建立通信连接

	robot.SetSpeed(50);   //设置全局速度

	robot.StartJOG(0, 1, 0, 20.0, 20.0, 30.0);   //单关节运动，StartJOG为非阻塞指令，运动状态下接收其他运动指令（包含StartJOG）会被丢弃
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));   //单位ms
	//robot.StopJOG(1)  //机器人单轴点动减速停止
	robot.ImmStopJOG();  //机器人单轴点动立即停止
	robot.StartJOG(0, 2, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(0, 3, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(0, 4, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(0, 5, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(0, 6, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();

	robot.StartJOG(2, 1, 0, 20.0, 20.0, 30.0);   //基坐标系下点动
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//robot.StopJOG(3)  //机器人单轴点动减速停止
	robot.ImmStopJOG();  //机器人单轴点动立即停止
	robot.StartJOG(2, 2, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(2, 3, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(2, 4, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(2, 5, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(2, 6, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();

	robot.StartJOG(4, 1, 0, 20.0, 20.0, 30.0);   //工具坐标系下点动
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//robot.StopJOG(5)  //机器人单轴点动减速停止
	robot.ImmStopJOG();  //机器人单轴点动立即停止
	robot.StartJOG(4, 2, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(4, 3, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(4, 4, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(4, 5, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(4, 6, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();

	robot.StartJOG(8, 1, 0, 20.0, 20.0, 30.0);   //工件坐标系下点动
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//robot.StopJOG(9)  //机器人单轴点动减速停止
	robot.ImmStopJOG();  //机器人单轴点动立即停止
	robot.StartJOG(8, 2, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(8, 3, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(8, 4, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(8, 5, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(8, 6, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//robot.ImmStopJOG();
	robot.StopJOG(9);

	return 0;
}