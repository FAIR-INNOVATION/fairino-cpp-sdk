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
    int sensor_id = 1;
    uint8_t select[6] = {1,1,1,0,0,0};
    float ft_pid[6] = {0.0005,0.0,0.0,0.0,0.0,0.0};
    uint8_t adj_sign = 0;
    uint8_t ILC_sign = 0;
    float max_dis = 100.0;
    float max_ang = 0.0;

    ForceTorque ft;
    DescPose desc_p1, desc_p2, offset_pos;
    ExaxisPos epos;
    JointPos j1, j2;
    memset(&ft, 0, sizeof(ForceTorque));
    memset(&desc_p1, 0, sizeof(DescPose));
    memset(&desc_p2, 0, sizeof(DescPose));
    memset(&offset_pos, 0, sizeof(DescPose));
    memset(&j1, 0, sizeof(JointPos));
    memset(&j2, 0, sizeof(JointPos));
    memset(&epos, 0, sizeof(ExaxisPos));

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

    ft.fx = -10.0;
    ft.fy = -10.0;
    ft.fz = -10.0;
    robot.FT_Control(flag, sensor_id, select, &ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang);  
    float p = 0.00005;
    float force = 30.0; 
    robot.FT_ComplianceStart(p, force); 
    int count = 15;
    while (count)
    {
        robot.MoveCart(&desc_p1, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
        robot.MoveCart(&desc_p2, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
        count -= 1;
    }
    robot.FT_ComplianceStop();
    flag = 0;
    robot.FT_Control(flag, sensor_id, select, &ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang);

    return 0;
}