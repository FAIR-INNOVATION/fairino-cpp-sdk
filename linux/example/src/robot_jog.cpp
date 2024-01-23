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
	//robot.RPC("192.168.58.2");    

	robot.SetSpeed(50);   //set global speed

	robot.StartJOG(0, 1, 0, 20.0, 20.0, 30.0);   //For single-joint movement, StartJOG is a non-blocking command, and other motion commands (including StartJOG) received in the motion state will be discarded
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));   //unit:ms
	//robot.StopJOG(1)  //The robot stops with single-axis jog deceleration
	robot.ImmStopJOG();  //The single-axis jog of the robot stops immediately
	robot.StartJOG(0, 2, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(0, 3, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(0, 4, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(0, 5, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(0, 6, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();

	robot.StartJOG(2, 1, 0, 20.0, 20.0, 30.0);   
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//robot.StopJOG(3)  
	robot.ImmStopJOG();  
	robot.StartJOG(2, 2, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(2, 3, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(2, 4, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(2, 5, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(2, 6, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();

	robot.StartJOG(4, 1, 0, 20.0, 20.0, 30.0);   
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//robot.StopJOG(5)  
	robot.ImmStopJOG(); 
	robot.StartJOG(4, 2, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(4, 3, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(4, 4, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(4, 5, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(4, 6, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();

	robot.StartJOG(8, 1, 0, 20.0, 20.0, 30.0);   //Jog in the workpiece coordinate system
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//robot.StopJOG(9)  
	robot.ImmStopJOG();  
	robot.StartJOG(8, 2, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(8, 3, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(8, 4, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(8, 5, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	robot.ImmStopJOG();
	robot.StartJOG(8, 6, 1, 20.0, 20.0, 30.0);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	//robot.ImmStopJOG();
	robot.StopJOG(9);

	return 0;
}