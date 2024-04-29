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

    int retval = 0;

    retval = robot.LoadIdentifyDynFilterInit();
    printf("LoadIdentifyDynFilterInit retval is: %d \n", retval);

    retval = robot.LoadIdentifyDynVarInit();
    printf("LoadIdentifyDynVarInit retval is: %d \n", retval);

    double joint_toq[6] = {0};
    double joint_pos[6] = {0};
    retval = robot.LoadIdentifyMain(joint_toq, joint_pos,1);
    printf("LoadIdentifyMain retval is: %d \n", retval);

    double gain[12] = {0};
    double weight = 0;
    DescTran load_pos;
    memset(&load_pos, 0, sizeof(DescTran));
    retval = robot.LoadIdentifyGetResult(gain, &weight, &load_pos);
    printf("LoadIdentifyGetResult retval is: %d \n", retval);
    printf("weight is: %f, load pose is: %f, %f, %f\n", weight, load_pos.x, load_pos.y, load_pos.z);

    retval = robot.WaitMs(10);
    printf("WaitMs retval is: %d \n", retval);


}



