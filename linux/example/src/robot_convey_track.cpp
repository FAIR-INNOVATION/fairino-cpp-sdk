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

	int index = 1;
	int act = 0;
	int max_time = 30000;
	uint8_t block = 0;
    int retval = 0;
    
    DescPose desc_p1;
    desc_p1.tran.x = -351.553;
    desc_p1.tran.y = 87.913;
    desc_p1.tran.z = 354.175;
    desc_p1.rpy.rx = -179.680;
    desc_p1.rpy.ry =  -0.133;
    desc_p1.rpy.rz = 2.472;

    // /[-333.625, -229.039, 404.340, -179.141, -0.778, 91.276]
    DescPose desc_p2;
    desc_p2.tran.x = -351.535;
    desc_p2.tran.y = -247.222;
    desc_p2.tran.z = 354.173;
    desc_p2.rpy.rx = -179.680;
    desc_p2.rpy.ry =  -0.137;
    desc_p2.rpy.rz = 2.473;


    retval = robot.MoveCart(&desc_p1, 1, 0, 100.0, 100.0, 100.0, -1.0, -1);
    printf("MoveCart retval is: %d\n", retval);

    retval = robot.WaitMs(1);
    printf("WaitMs retval is: %d\n", retval);

    retval = robot.ConveyorIODetect(10000);
    printf("ConveyorIODetect retval is: %d\n", retval);

    retval = robot.ConveyorGetTrackData(1);
    printf("ConveyorGetTrackData retval is: %d\n", retval);

    retval = robot.ConveyorTrackStart(1);
    printf("ConveyorTrackStart retval is: %d\n", retval);

    retval = robot.TrackMoveL("cvrCatchPoint",  1, 0, 100, 100, 100, -1.0, 0, 0);
    printf("TrackMoveL retval is: %d\n", retval);

    retval = robot.MoveGripper(index, 51, 40, 30, max_time, block);
    printf("MoveGripper retval is: %d\n", retval);

    retval = robot.TrackMoveL("cvrRaisePoint", 1, 0, 100, 100, 100, -1.0, 0, 0);
    printf("TrackMoveL retval is: %d\n", retval);

    retval = robot.ConveyorTrackEnd();
    printf("ConveyorTrackEnd retval is: %d\n", retval);

    robot.MoveCart(&desc_p2, 1, 0, 100.0, 100.0, 100.0, -1.0, -1);

    retval = robot.MoveGripper(index, 100, 40, 10, max_time, block);
    printf("MoveGripper retval is: %d\n", retval);

    return 0;
}
