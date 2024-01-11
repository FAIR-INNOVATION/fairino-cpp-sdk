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

    char file_path[256] = "/fruser/traj/test_computermd5.txt.txt";
    char md5[256] = {0};
    uint8_t emerg_state = 0;
    uint8_t si0_state = 0;
    uint8_t si1_state = 0;
    int sdk_com_state = 0;

    char ssh_keygen[1024] = {0};
    int retval = robot.GetSSHKeygen(ssh_keygen);
    printf("GetSSHKeygen retval is: %d\n", retval);
    printf("ssh key is: %s \n", ssh_keygen);

    char ssh_name[32] = "fr";
    char ssh_ip[32] = "192.168.58.44";
    char ssh_route[128] = "/home/fr";
    char ssh_robot_url[128] = "/root/robot/dhpara.config";
    retval = robot.SetSSHScpCmd(1, ssh_name, ssh_ip, ssh_route, ssh_robot_url);
    printf("SetSSHScpCmd retval is: %d\n", retval);
    printf("robot url is: %s\n", ssh_robot_url);

    robot.ComputeFileMD5(file_path, md5);
    printf("md5 is: %s \n", md5);

    robot.GetRobotEmergencyStopState(&emerg_state);
    printf("emergency state is: %u \n", emerg_state);

    robot.GetSafetyStopState(&si0_state, &si1_state);
    printf("safety stop state is: %u, %u \n", si0_state, si1_state);

    robot.GetSDKComState(&sdk_com_state);
    printf("sdk com state is: %d", sdk_com_state);
    return 0;
}