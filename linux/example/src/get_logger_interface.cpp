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
	robot.LoggerInit(2, "../abcd.log", 2);
	//robot.LoggerInit(0);
	robot.SetLoggerLevel(3);
	robot.RPC("192.168.58.2");    

	uint8_t status = 0;
	uint8_t smooth = 0;
	uint8_t block = 0;
	uint8_t di = 0, tool_di = 0;
	float ai = 0.0, tool_ai = 0.0;
	float value = 0.0;
	int i;
	uint8_t do_state_h = 0;
	uint8_t do_state_l = 0;
	uint8_t tool_do_state = 0;

	for (i = 0; i < 16; i++)
	{
		robot.SetDO(i, status, smooth, block);
		robot.WaitMs(1000);
	}
	printf("set DO status to 0\n");
	robot.GetDO(&do_state_h, &do_state_l);
	printf("DO state high is: %u \n DO state low is: %u\n", do_state_h, do_state_l);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	printf("\n");

	status = 1;

	for (i = 0; i < 16; i++)
	{
		robot.SetDO(i, status, smooth, block);
		robot.WaitMs(1000);
	}
	printf("set DO status to 1\n");
	robot.GetDO(&do_state_h, &do_state_l);
	printf("DO state high is: %u \n DO state low is: %u\n", do_state_h, do_state_l);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	printf("\n");

	status = 0;

	for (i = 0; i < 2; i++)
	{
		robot.SetToolDO(i, status, smooth, block);
		robot.WaitMs(1000);
	}
	printf("set tool DO status to 0\n");
	robot.GetToolDO(&tool_do_state);
	printf("tool DO state is: %u\n", tool_do_state);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	printf("\n");

	status = 1;

	for (i = 0; i < 2; i++)
	{
		robot.SetToolDO(i, status, smooth, block);
		robot.WaitMs(1000);
	}
	printf("set tool DO status to 1\n");
	robot.GetToolDO(&tool_do_state);
	printf("tool DO state is: %u\n", tool_do_state);
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	printf("\n");

	value = 50.0;
	robot.SetAO(0, value, block);
	value = 100.0;
	robot.SetAO(1, value, block);
	robot.WaitMs(1000);
	value = 0.0;
	robot.SetAO(0, value, block);
	value = 0.0;
	robot.SetAO(1, value, block);

	value = 100.0;
	robot.SetToolAO(0, value, block);
	robot.WaitMs(1000);
	value = 0.0;
	robot.SetToolAO(0, value, block);

	robot.GetDI(0, block, &di);
	printf("di0:%u\n", di);
	robot.WaitDI(0, 1, 1000, 1);            
	robot.WaitMultiDI(1, 3, 3, 10000, 1);   
	tool_di = robot.GetToolDI(1, block, &tool_di);
	printf("tool_di1:%u\n", tool_di);
	robot.WaitToolDI(1, 1, 0, 1);         

	robot.GetAI(0, block, &ai);
	printf("ai0:%f\n", ai);
	robot.WaitAI(0, 0, 50, 1000, 1);         
	robot.WaitToolAI(0, 0, 50, 1000, 1);       
	tool_ai = robot.GetToolAI(0, block, &tool_ai);
	printf("tool_ai0:%f\n", tool_ai);

    uint8_t _button_state = 0;
    int times = 0;
    do
    {
        robot.GetAxlePointRecordBtnState(&_button_state);
        printf("_button_state is: %u\n", _button_state);
        // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        times++;
    } while (times < 500000);
    

	return 0;
}