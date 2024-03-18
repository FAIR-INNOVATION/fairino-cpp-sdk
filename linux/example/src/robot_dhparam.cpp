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

    double dh[6] = {0};
    int retval = 0;
    retval = robot.GetDHCompensation(dh);
    cout << "retval is: " << retval << endl;
    cout << "dh is: " << dh[0] << " " << dh[1] << " "<< dh[2] << " "<< dh[3] << " "<< dh[4] << " "<< dh[5] << endl;

}