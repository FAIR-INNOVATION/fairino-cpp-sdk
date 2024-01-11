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

	char program_name[64] = "/fruser/pptest.lua";
	char loaded_name[64] = "";
	uint8_t state;
	int line;
	uint8_t default_load_program_flag = 0;

	robot.Mode(0);

	default_load_program_flag = 1;
	robot.LoadDefaultProgConfig(default_load_program_flag, program_name);

	robot.ProgramLoad(program_name);
	robot.ProgramRun();
	std::this_thread::sleep_for(std::chrono::milliseconds(3000));     //ms
	robot.ProgramPause();
	robot.GetProgramState(&state);
	printf("program state:%u\n", state);
	robot.GetCurrentLine(&line);
	printf("current line:%d\n", line);
	robot.GetLoadedProgram(loaded_name);
	printf("program name:%s\n", loaded_name);
	std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	robot.ProgramResume();
	std::this_thread::sleep_for(std::chrono::milliseconds(5000));
	robot.ProgramStop();
	std::this_thread::sleep_for(std::chrono::milliseconds(2000));

	return 0;
}