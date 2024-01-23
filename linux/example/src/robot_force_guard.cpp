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

    uint8_t flag = 1;
    uint8_t sensor_id = 1;
    uint8_t select[6] = {1,1,1,1,1,1};
    float max_threshold[6] = {20.0,20.0,20.0,20.0,20.0,20.0};
    float min_threshold[6] = {20.0,20.0,20.0,20.0,20.0,20.0};

    ForceTorque ft;
    DescPose desc_p1, desc_p2, desc_p3;
    memset(&ft, 0, sizeof(ForceTorque));
    memset(&desc_p1, 0, sizeof(DescPose));
    memset(&desc_p2, 0, sizeof(DescPose));
    memset(&desc_p3, 0, sizeof(DescPose));

    desc_p1.tran.x = -433.06;
    desc_p1.tran.y = 212.862;
    desc_p1.tran.z = 419.955;
    desc_p1.rpy.rx = 179.056;
    desc_p1.rpy.ry = 0.422;
    desc_p1.rpy.rz = 48.117;

    desc_p2.tran.x = -341.078;
    desc_p2.tran.y = -98.77;
    desc_p2.tran.z = 369.866;
    desc_p2.rpy.rx = -178.888;
    desc_p2.rpy.ry = 0.441;
    desc_p2.rpy.rz = 84.562;

    desc_p3.tran.x = -442.474;
    desc_p3.tran.y = -290.893;
    desc_p3.tran.z = 324.923;
    desc_p3.rpy.rx = -174.106;
    desc_p3.rpy.ry = -1.298;
    desc_p3.rpy.rz = 109.225;

    if (1)
    {
        robot.FT_Guard(flag, sensor_id, select, &ft, max_threshold, min_threshold);
        robot.MoveCart(&desc_p1, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
        robot.MoveCart(&desc_p2, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
        robot.MoveCart(&desc_p3, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
        flag = 0;
        robot.FT_Guard(flag, sensor_id, select, &ft, max_threshold, min_threshold);
    }

    //
    if (0)
    {
        int retval = robot.FT_SpiralSearch(0, 1, 10, 5000, 3);
        printf("FT_SpiralSearch retval is:%d \n", retval);

        retval = robot.FT_RotInsertion(0, 10, 10, 100, 1, 0, 1);
        printf("FT_RotInsertion retval is:%d \n", retval);

        retval = robot.FT_LinInsertion(0, 20, 10, 0, 10, 1);
        printf("FT_LinInsertion retval is:%d \n", retval);

        retval = robot.FT_FindSurface(1, 1, 1, 10, 0, 300, 20);
        printf("FT_FindSurface retval is:%d \n", retval);

        retval = robot.FT_CalCenterStart();
        printf("FT_CalCenterStart retval is:%d \n", retval);

        DescPose center_pose;
        memset(&center_pose, 0, sizeof(DescPose));
        retval = robot.FT_CalCenterEnd(&center_pose);
        printf("FT_CalCenterEnd retval is:%d \n", retval);
        printf("xyz is: %f, %f, %f; rpy is: %f, %f, %f\n", center_pose.tran.x, center_pose.tran.y, center_pose.tran.z,  \
													center_pose.rpy.rx, center_pose.rpy.ry, center_pose.rpy.rz);

    }

    return 0;
}