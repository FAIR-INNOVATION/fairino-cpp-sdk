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

    retval = robot.ConveyorStartEnd(1);
    printf("ConveyorStartEnd retval is: %d\n", retval);

    retval = robot.ConveyorPointIORecord();
    printf("ConveyorPointIORecord retval is: %d\n", retval);

    retval = robot.ConveyorPointARecord();
    printf("ConveyorPointARecord retval is: %d\n", retval);

    retval = robot.ConveyorRefPointRecord();
    printf("ConveyorRefPointRecord retval is: %d\n", retval);

    retval = robot.ConveyorPointBRecord();
    printf("ConveyorPointBRecord retval is: %d\n", retval);

    retval = robot.ConveyorStartEnd(0);
    printf("ConveyorStartEnd retval is: %d\n", retval);
    
    return 0;
}
