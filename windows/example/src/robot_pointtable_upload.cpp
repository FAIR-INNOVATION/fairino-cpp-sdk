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

using namespace std;

int main(void)
{
	FRRobot robot;                
	robot.RPC("192.168.58.2");     

	string save_path = "D://sharkLog/0.db";

	int retval = robot.PointTableUpLoad(save_path);
	cout << "retval is: "<<retval<<endl;

}