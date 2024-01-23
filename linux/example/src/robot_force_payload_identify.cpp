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

    float weight;
    int retval = 0;

    DescPose tcoord, desc_p1, desc_p2, desc_p3;
    memset(&tcoord, 0, sizeof(DescPose));
    memset(&desc_p1, 0, sizeof(DescPose));
    memset(&desc_p2, 0, sizeof(DescPose));
    memset(&desc_p3, 0, sizeof(DescPose));

    /* Clear the load */
    robot.SetLoadWeight(0.0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    DescTran coord;
    memset(&coord, 0, sizeof(DescTran));
    robot.SetLoadCoord(&coord);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));    

    robot.FT_SetRCS(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    int tool = 0;
    int user = 0;
    DescPose desc_pos;
    memset(&desc_pos, 0, sizeof(DescPose));
    robot.GetActualTCPPose(0, &desc_pos);
    printf("desc_pos:%f,%f,%f,%f,%f,%f\n",desc_pos.tran.x,desc_pos.tran.y,desc_pos.tran.z,desc_pos.rpy.rx,desc_pos.rpy.ry,desc_pos.rpy.rz);
    desc_pos.rpy.rx = -90;
    desc_pos.rpy.ry = 0.0;
    robot.GetActualTCPNum(1, &tool);
    robot.GetActualWObjNum(1, &user);
    robot.MoveCart(&desc_pos, tool, user, 30, 50, 30, -1.0, -1);    

    tcoord.tran.z = 10.0;
    robot.SetToolCoord(10, &tcoord, 1, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    robot.FT_PdIdenRecord(10);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    retval = robot.FT_PdIdenCompute(&weight);
    printf("payload retval is: %d, weight:%f\n", retval, weight);


    JointPos j;
    memset(&j, 0, sizeof(JointPos));
    memset(&desc_pos, 0, sizeof(DescPose));
    robot.GetActualJointPosDegree(1, &j);
    j.jPos[4] += 10;
    robot.GetForwardKin(&j, &desc_pos);
    robot.MoveCart(&desc_pos, tool, user, 30, 100, 30, -1, -1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    robot.FT_PdCogIdenRecord(10, 1);
    robot.WaitMs(1);

    j.jPos[4] -= 100;
    robot.GetForwardKin(&j, &desc_pos);
    robot.MoveCart(&desc_pos, tool, user, 30, 100, 30, -1, -1);   
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    robot.FT_PdCogIdenRecord(10, 2);
    robot.WaitMs(1);
    j.jPos[3] += 10;
    robot.GetForwardKin(&j, &desc_pos);
    robot.MoveCart(&desc_pos, tool, user, 30, 100, 30, -1, -1);   
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    robot.FT_PdCogIdenRecord(10, 3);
    robot.WaitMs(1);
    DescTran cog;
    memset(&cog, 0, sizeof(DescTran));
    retval = robot.FT_PdCogIdenCompute(&cog);
    printf("retval is: %d, cog:%f,%f,%f\n",retval, cog.x, cog.y, cog.z);

    robot.SetLoadWeight(weight);
    robot.SetLoadCoord(&cog);

    return 0;
}