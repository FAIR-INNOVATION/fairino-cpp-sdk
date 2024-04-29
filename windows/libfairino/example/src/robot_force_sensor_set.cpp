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

    /* 调整位姿 */
    int tool, user;
    DescPose desc_pos;
    memset(&desc_pos, 0, sizeof(DescPose));
    robot.GetActualTCPPose(0, &desc_pos);
    printf("desc_pos:%f,%f,%f,%f,%f,%f\n",desc_pos.tran.x,desc_pos.tran.y,desc_pos.tran.z,desc_pos.rpy.rx,desc_pos.rpy.ry,desc_pos.rpy.rz);
    desc_pos.rpy.rx = -179.9;
    desc_pos.rpy.ry = 0.0;
    robot.GetActualTCPNum(1, &tool);
    robot.GetActualWObjNum(1, &user);
    robot.MoveCart(&desc_pos, tool, user, 50, 50, 50, -1.0, -1);

    int company = 17;
    int device = 0;
    int softversion = 0;
    int bus = 1;
    int index = 1;
    int act = 0;

    robot.FT_SetConfig(company, device, softversion, bus);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    robot.FT_GetConfig(&company, &device, &softversion, &bus);
    printf("FT config:%d,%d,%d,%d\n", company, device, softversion, bus);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    robot.FT_Activate(act);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    act = 1;
    robot.FT_Activate(act);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    robot.SetLoadWeight(0.0);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    DescTran coord;
    memset(&coord, 0, sizeof(DescTran));
    robot.SetLoadCoord(&coord);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    robot.FT_SetZero(0);
    std::this_thread::sleep_for(std::chrono::seconds(1));

    ForceTorque ft0;
    memset(&ft0, 0, sizeof(ForceTorque));
    int retval = robot.FT_GetForceTorqueRCS(1, &ft0);
    printf("ft0 retval: %d, origin:%f,%f,%f,%f,%f,%f\n",retval, ft0.fx,ft0.fy,ft0.fz,ft0.tx,ft0.ty,ft0.tz);

    ForceTorque ft;
    memset(&ft, 0, sizeof(ForceTorque));
    robot.FT_GetForceTorqueOrigin(1, &ft);
    printf("ft origin:%f,%f,%f,%f,%f,%f\n", ft.fx,ft.fy,ft.fz,ft.tx,ft.ty,ft.tz);
    robot.FT_SetZero(1);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    memset(&ft, 0, sizeof(ForceTorque));
    printf("ft rcs:%f,%f,%f,%f,%f,%f\n",ft.fx,ft.fy,ft.fz,ft.tx,ft.ty,ft.tz);

    return 0;
}