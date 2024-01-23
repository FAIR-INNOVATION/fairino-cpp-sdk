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

	DescPose start_descpose;
	start_descpose.rpy.rx = 2.243;
	start_descpose.rpy.ry = 0.828;
	start_descpose.rpy.rz = -148.894;
	start_descpose.tran.x = -208.064;
	start_descpose.tran.y = 412.155;
	start_descpose.tran.z = 1.926;

	JointPos start_jointpose;
	start_jointpose.jPos[0] = -51.489;
	start_jointpose.jPos[1] = -105.721;
	start_jointpose.jPos[2] = 130.695;
	start_jointpose.jPos[3] = -108.338;
	start_jointpose.jPos[4] = -91.356;
	start_jointpose.jPos[5] = 62.014;

	DescPose end_descpose;
	end_descpose.rpy.rx = 2.346;
	end_descpose.rpy.ry = -3.633;
	end_descpose.rpy.rz = -106.313;
	end_descpose.tran.x = -425.087;
	end_descpose.tran.y = 389.637;
	end_descpose.tran.z = -9.249;

	JointPos end_jointpose;
	end_jointpose.jPos[0] = -47.137;
	end_jointpose.jPos[1] = -102.345;
	end_jointpose.jPos[2] = 127.607;
	end_jointpose.jPos[3] = -108.526;
	end_jointpose.jPos[4] = -91.407;
	end_jointpose.jPos[5] = 23.537;

    ExaxisPos ex_axis_pose;
    memset(&ex_axis_pose, 0, sizeof(ExaxisPos));
    DescPose offset_pose;
    memset(&offset_pose, 0, sizeof(DescPose));
    int retval = 0;

    robot.SetSpeed(50);

    retval = robot.MoveJ(&start_jointpose, &start_descpose, 1, 0, 50, 50, 50, &ex_axis_pose, 0, 0, &offset_pose);
    if(retval != 0)
    {
        cout << "movej fail " << retval << endl;
        return 0;
    }

    retval = robot.ARCStart(1, 0, 5000);
    if(retval != 0)
    {
        cout << "ARCStart fail " << retval << endl;
        return 0;
    }

    retval = robot.WeaveStart(0);
    if(retval != 0)
    {
        cout << "WeaveStart fail " << retval << endl;
        robot.ARCEnd(0, 0, 0);
        return 0;
    }

    retval = robot.MoveL(&end_jointpose,&end_descpose, 1, 0, 30, 50, 50, 0, &ex_axis_pose, 0, 0, &offset_pose);
    if(retval != 0)
    {
        cout << "MoveL fail " << retval << endl;
        robot.ARCEnd(0, 0, 0);
        robot.WeaveEnd(0);
        return 0;
    }

    robot.ARCEnd(1, 0, 5000);
    robot.WeaveEnd(0);

    return 0;
}