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

// Note: Before running this program, verify that the end load is configured correctly and that you can switch into drag teach-in mode
int main(void)
{
	FRRobot robot;                
	robot.RPC("192.168.58.2");     

	int type = 1;
	char name[30] = "tpd2023";
	int period_ms = 4;
	uint16_t di_choose = 0;
	uint16_t do_choose = 0;

	robot.SetTPDParam(type, name, period_ms, di_choose, do_choose);

	robot.Mode(1);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.DragTeachSwitch(1);
	robot.SetTPDStart(type, name, period_ms, di_choose, do_choose);
	std::this_thread::sleep_for(std::chrono::milliseconds(30000));
	robot.SetWebTPDStop();
	robot.DragTeachSwitch(0);

	//robot.SetTPDDelete(name);

	return 0;
}