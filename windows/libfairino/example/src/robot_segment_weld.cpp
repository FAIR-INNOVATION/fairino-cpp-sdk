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
	start_descpose.rpy.rx = 7.178;
	start_descpose.rpy.ry = -0.809;
	start_descpose.rpy.rz = -133.134;
	start_descpose.tran.x = -135.56;
	start_descpose.tran.y = 373.448;
	start_descpose.tran.z = 36.767;

	JointPos start_jointpose;
	start_jointpose.jPos[0] = -70.228;
	start_jointpose.jPos[1] = -130.911;
	start_jointpose.jPos[2] = 134.147;
	start_jointpose.jPos[3] = -83.379;
	start_jointpose.jPos[4] = -95.656;
	start_jointpose.jPos[5] = 27.74;

	DescPose end_descpose;
	end_descpose.rpy.rx = -4.586;
	end_descpose.rpy.ry = -10.926;
	end_descpose.rpy.rz = -124.298;
	end_descpose.tran.x = -380.207;
	end_descpose.tran.y = 371.358;
	end_descpose.tran.z = 55.898;

	JointPos end_jointpose;
	end_jointpose.jPos[0] = -50.247;
	end_jointpose.jPos[1] = -113.273;
	end_jointpose.jPos[2] = 125.856;
	end_jointpose.jPos[3] = -100.351;
	end_jointpose.jPos[4] = -80.702;
	end_jointpose.jPos[5] = 38.478;
    ExaxisPos ex_axis_pose;
    memset(&ex_axis_pose, 0, sizeof(ExaxisPos));
    DescPose offset_pose;
    memset(&offset_pose, 0, sizeof(DescPose));
    int retval = 0;

    retval = robot.SegmentWeldStart(&start_descpose, &end_descpose, &start_jointpose, &end_jointpose, 20, 20, 1, 0, 5000, 1, 0, 1, 0, 
                                    20, 50, 50, 0, &ex_axis_pose, 0, 0, &offset_pose);
    if(0 != retval)
    {
        cout << "SegmentWeldStart end " << retval << endl;
    }

    return 0;
}