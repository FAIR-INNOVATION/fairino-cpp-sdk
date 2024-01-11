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

	JointPos j;
	DescPose desc_pos, offset_pos1, offset_pos2;
	ExaxisPos epos;
	SpiralParam sp;

	memset(&j, 0, sizeof(JointPos));
	memset(&desc_pos, 0, sizeof(DescPose));
	memset(&offset_pos1, 0, sizeof(DescPose));
	memset(&offset_pos2, 0, sizeof(DescPose));
	memset(&epos, 0, sizeof(ExaxisPos));
	memset(&sp, 0, sizeof(SpiralParam));

	j = { { -30.926, -102.968, -149.089, -16.774, 88.187, -62.993 } };
	offset_pos1.tran.x = 10.0;
	offset_pos1.rpy.rx = -10.0;
	offset_pos2.tran.x = 50.0;
	offset_pos2.rpy.rx = -5.0;

	sp.circle_num = 5;
	sp.circle_angle = 5.0;
	sp.rad_init = 50.0;
	sp.rad_add = 10.0;
	sp.rotaxis_add = 10.0;
	sp.rot_direction = 0;

	int tool = 0;
	int user = 0;
	float vel = 30.0;
	float acc = 30.0;
	float ovl = 100.0;
	float blendT = 0.0;
	uint8_t flag = 2;

	uint8_t result = 0;
	uint8_t block = 0;
	float acc_deg[6];
	memset(acc_deg, 0, sizeof(acc_deg));

	robot.SetSpeed(20);

	robot.GetActualJointAccDegree(block, acc_deg);
	printf("acc for every joint is: %f %f %f %f %f %f  deg/s^2\n", acc_deg[0], acc_deg[1], acc_deg[2], acc_deg[3], acc_deg[4], acc_deg[5]);

	int ret = robot.GetForwardKin(&j, &desc_pos); // In cases where there is only joint position, Cartesian coordinates can be solved using the Positive Kinematics interface
	printf("GetForwardKin ret is: %d \n",ret);
	if (ret == 0)
	{
		ret = robot.GetInverseKinHasSolution(1, &offset_pos1, &j, &result); // According to the coordinates of the reference joint, determine whether the target pose has a solution
		printf("GetInverseKinHasSolution ret: %d\n", result);
		if (0 == result)
		{
			printf("pose1 can not be solved\n");
		}
		else
		{
			printf("pose1 has solution, use moveJ to plan \n");
			int err1 = robot.MoveJ(&j, &desc_pos, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos1);
			printf("movej errcode:%d\n", err1);
			robot.GetActualJointAccDegree(block, acc_deg);
			printf("acc for every joint is:: %f %f %f %f %f %f  deg/s^2\n", acc_deg[0], acc_deg[1], acc_deg[2], acc_deg[3], acc_deg[4], acc_deg[5]);
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(3000));

		int err2 = robot.NewSpiral(&j, &desc_pos, tool, user, vel, acc, &epos, ovl, flag, &offset_pos2, sp);
		printf("newspiral errcode:%d\n", err2);
	}
	else
	{
		printf("GetForwardKin errcode:%d\n", ret);
	}

	return 0;
}