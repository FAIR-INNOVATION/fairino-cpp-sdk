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

    int mode = 0;
    int config = 1;
    float level1[6] = {1.0,2.0,3.0,4.0,5.0,6.0};
    float level2[6] = {50.0,20.0,30.0,40.0,50.0,60.0};
    int retval = 0;

    retval = robot.SetAnticollision(mode, level1, config);

    mode = 1;
    retval = robot.SetAnticollision(mode, level2, config);
    printf("SetAnticollision retval is: %d\n", retval);
    retval = robot.SetCollisionStrategy(1);
    printf("SetCollisionStrategy retval is: %d\n", retval);

    float plimit[6] = {0.0,0.0,150.0,80.0,170.0,160.0};
    retval = robot.SetLimitPositive(plimit);
    printf("SetLimitPositive retval is: %d\n", retval);
    float nlimit[6] = {-170.0,-260.0,-150.0,-260.0,-170.0,-160.0};
    retval = robot.SetLimitNegative(nlimit);
    printf("SetLimitNegative retval is: %d\n", retval);

    retval = robot.ResetAllError();
    printf("ResetAllError retval is: %d\n", retval);

    float lcoeff[6] = {1,1,1,1,1,1};
    float wcoeff[6] = {1,1,1,1,1,1};
    float ccoeff[6] = {1,1,1,1,1,1};
    float fcoeff[6] = {1,1,1,1,1,1};
    retval = robot.FrictionCompensationOnOff(1);
    printf("FrictionCompensationOnOff retval is: %d\n", retval);

    retval = robot.SetFrictionValue_level(lcoeff);
    printf("SetFrictionValue_level retval is: %d\n", retval);

    retval = robot.SetFrictionValue_wall(wcoeff);
    printf("SetFrictionValue_wall retval is: %d\n", retval);

    retval = robot.SetFrictionValue_ceiling(ccoeff);
    printf("SetFrictionValue_ceiling retval is: %d\n", retval);

    retval = robot.SetFrictionValue_freedom(fcoeff);
    printf("SetFrictionValue_freedom retval is: %d\n", retval);

    return 0;
}