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
    uint8_t select[6] = {0,0,1,0,0,0};
    float ft_pid[6] = {0.0005,0.0,0.0,0.0,0.0,0.0};
    uint8_t adj_sign = 0;
    uint8_t ILC_sign = 0;
    float max_dis = 100.0;
    float max_ang = 0.0;

    ForceTorque ft;
    DescPose desc_p1, desc_p2, desc_p3, desc_p4, offset_pos;
    JointPos j1,j2,j3,j4;
    ExaxisPos epos;
    memset(&ft, 0, sizeof(ForceTorque));
    memset(&desc_p1, 0, sizeof(DescPose));
    memset(&desc_p2, 0, sizeof(DescPose));
    memset(&desc_p3, 0, sizeof(DescPose));
    memset(&desc_p4, 0, sizeof(DescPose));
    memset(&offset_pos, 0, sizeof(DescPose));
    memset(&epos, 0, sizeof(ExaxisPos));
    memset(&j1, 0, sizeof(JointPos));
    memset(&j2, 0, sizeof(JointPos));
    memset(&j3, 0, sizeof(JointPos));
    memset(&j4, 0, sizeof(JointPos));

    j1 = {-62.634,-100.187,-107.248,-64.857,88.565,30.584};
    j2 = {-106.298,-111.065,-106.369,-52.017,88.515,30.285};
    j3 = {-107.686,-106.385,-114.325,-48.895,89.167,30.258};
    j4 = {-63.536,-91.994,-122.111,-55.805,89.952,28.389};

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

    desc_p4.tran.x = 81.396;
    desc_p4.tran.y = -455.715;
    desc_p4.tran.z = 100.408;
    desc_p4.rpy.rx = -179.899;
    desc_p4.rpy.ry = -0.0;
    desc_p4.rpy.rz = 178.075;

    ft.fz = -10.0;

    // robot.MoveJ(&j1,&desc_p1,9,0,100.0,180.0,100.0,&epos,-1.0,0,&offset_pos);
    robot.MoveCart(&desc_p1, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
    robot.FT_Control(flag, sensor_id, select, &ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang);
    // robot.MoveL(&j2,&desc_p2,9,0,100.0,180.0,20.0,-1.0,&epos,0,0,&offset_pos);
    // robot.MoveL(&j3,&desc_p3,9,0,100.0,180.0,20.0,-1.0,&epos,0,0,&offset_pos);
    // robot.MoveL(&j4,&desc_p4,9,0,100.0,180.0,20.0,-1.0,&epos,0,0,&offset_pos);
    robot.MoveCart(&desc_p2, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
    robot.MoveCart(&desc_p3, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
    flag = 0;
    robot.FT_Control(flag, sensor_id, select, &ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang);

    return 0;
}