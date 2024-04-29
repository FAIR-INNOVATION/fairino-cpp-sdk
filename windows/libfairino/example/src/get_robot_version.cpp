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

    int retval = 0;
    char robotModel[64] = {0};
    char webversion[64] = {0};
    char controllerVersion[64] = {0};

    char ctrlBoxBoardversion[128] = {0};
    char driver1version[128] = {0};
    char driver2version[128] = {0};
    char driver3version[128] = {0};
    char driver4version[128] = {0};
    char driver5version[128] = {0};
    char driver6version[128] = {0};
    char endBoardversion[128] = {0};

    retval = robot.GetSoftwareVersion(robotModel, webversion, controllerVersion);
    printf("Getsoftwareversion retval is: %d\n", retval);
    printf("robotmodel is: %s, webversion is: %s, controllerVersion is: %s \n", robotModel, webversion, controllerVersion);

    retval = robot.GetHardwareVersion(ctrlBoxBoardversion,  driver1version,  driver2version,
                                     driver3version,  driver4version,  driver5version,
                                     driver6version,  endBoardversion);
    printf("GetHardwareversion retval is: %d\n", retval);
    printf("GetHardwareversion get hardware versoin is: %s, %s, %s, %s, %s, %s, %s, %s\n", ctrlBoxBoardversion, 
                                                            driver1version, driver2version,
                                                            driver3version, driver4version,
                                                            driver5version, driver6version, endBoardversion);

    retval = robot.GetFirmwareVersion(ctrlBoxBoardversion,  driver1version,  driver2version,
                                     driver3version,  driver4version,  driver5version,
                                     driver6version,  endBoardversion);
    printf("GetFirmwareversion retval is: %d\n", retval);
    printf("GetHardwareversion get hardware versoin is: %s, %s, %s, %s, %s, %s, %s, %s\n", ctrlBoxBoardversion, 
                                                            driver1version, driver2version,
                                                            driver3version, driver4version,
                                                            driver5version, driver6version, endBoardversion);

    return 0;
}