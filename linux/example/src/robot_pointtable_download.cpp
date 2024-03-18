#include "libfairino/robot.h"
#ifdef WINDOWS_OPTION
#include <string.h>
#include <string>
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
#include <fstream>

using namespace std;

int main(void)
{
    FRRobot robot;
    robot.RPC("192.168.58.2");

    string save_path = "D://sharkLog/";
    string point_table_name = "point_table_a.db";

    int retval = robot.PointTableDownLoad(point_table_name, save_path);
    cout << "download : " << point_table_name << " fail: " << retval << endl;
}