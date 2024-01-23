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

    int retval = 0;
    float param[6] ={1,10000,200,0,0,20};

    retval = robot.ConveyorSetParam(param);
    printf("ConveyorSetParam retval is: %d\n", retval);
    
    double cmp[3] = {0.0, 0.0, 0.0};
    retval = robot.ConveyorCatchPointComp(cmp);
    printf("ConveyorCatchPointComp retval is: %d\n", retval);

    return 0;
}
