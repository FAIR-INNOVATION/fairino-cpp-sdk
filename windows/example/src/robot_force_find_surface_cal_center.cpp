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

    int rcs = 0;
    uint8_t dir = 1;
    uint8_t axis = 1;
    float lin_v = 5.0;
    float lin_a = 0.0;
    float maxdis = 50.0;
    float ft_goal = 2.0;

    DescPose desc_pos, xcenter, ycenter;
    ForceTorque ft;
    memset(&desc_pos, 0, sizeof(DescPose));
    memset(&xcenter, 0, sizeof(DescPose));
    memset(&ycenter, 0, sizeof(DescPose));
    memset(&ft, 0, sizeof(ForceTorque));

    desc_pos.tran.x = -208.495;
    desc_pos.tran.y = -349.847;
    desc_pos.tran.z = 61.434;
    desc_pos.rpy.rx = 178.459;
    desc_pos.rpy.ry = -0.518;
    desc_pos.rpy.rz = 90.318;

    ft.fx = -2.0;

    robot.MoveCart(&desc_pos, 9,0,100.0,100.0,100.0,-1.0,-1);

    robot.FT_CalCenterStart();
    robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);
    robot.MoveCart(&desc_pos, 9,0,100.0,100.0,100.0,-1.0,-1);
    robot.WaitMs(1000);

    dir = 2;
    robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);
    robot.FT_CalCenterEnd(&xcenter);
    printf("xcenter:%f,%f,%f,%f,%f,%f\n",xcenter.tran.x,xcenter.tran.y,xcenter.tran.z,xcenter.rpy.rx,xcenter.rpy.ry,xcenter.rpy.rz);
    robot.MoveCart(&xcenter, 9,0,60.0,50.0,50.0,-1.0,-1);

    robot.FT_CalCenterStart();
    dir = 1;
    axis = 2;
    lin_v = 6.0;
    maxdis = 150.0;
    robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);
    robot.MoveCart(&desc_pos, 9,0,100.0,100.0,100.0,-1.0,-1);
    robot.WaitMs(1000);

    dir = 2;
    robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);  
    robot.FT_CalCenterEnd(&ycenter);
    printf("ycenter:%f,%f,%f,%f,%f,%f\n",ycenter.tran.x,ycenter.tran.y,ycenter.tran.z,ycenter.rpy.rx,ycenter.rpy.ry,ycenter.rpy.rz);
    robot.MoveCart(&ycenter, 9,0,60.0,50.0,50.0,0.0,-1);   

    return 0;
}