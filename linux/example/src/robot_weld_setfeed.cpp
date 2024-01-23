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
	printf("start to debug\n");
	FRRobot robot;                 
	robot.RPC("192.168.58.2");     

	this_thread::sleep_for(chrono::seconds(3));

    int retval = robot.SetForwardWireFeed(1, 1);
    cout << "SetForwardWireFeed retval is: " << retval << endl;

    this_thread::sleep_for(chrono::seconds(3));

    retval = robot.SetForwardWireFeed(1, 0);
    cout << "SetForwardWireFeed retval is: " << retval << endl;

    retval = robot.SetReverseWireFeed(1, 1);
    cout << "SetReverseWireFeed retval is: " << retval << endl;

    this_thread::sleep_for(chrono::seconds(3));

    retval = robot.SetReverseWireFeed(1, 0);
    cout << "SetReverseWireFeed retval is: " << retval << endl;

    retval = robot.SetAspirated(1, 1);
    cout << "SetAspirated retval " << retval << endl;

	this_thread::sleep_for(chrono::seconds(2));

    retval = robot.SetAspirated(1, 0);
    cout << "SetAspirated retval " << retval << endl;
    
    return 0;
}
