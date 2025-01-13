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
    float param[6] ={1,10000,200,0,0,20};

    retval = robot.ConveyorSetParam(param);
    printf("ConveyorSetParam retval is: %d\n", retval);
    
    double cmp[3] = {0.0, 0.0, 0.0};
    retval = robot.ConveyorCatchPointComp(cmp);
    printf("ConveyorCatchPointComp retval is: %d\n", retval);

    return 0;
}
