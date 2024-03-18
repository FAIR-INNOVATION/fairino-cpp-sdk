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
	robot.LoggerInit(1);
	robot.SetLoggerLevel(3);
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

	j = { { -39.666, -96.491, -79.531, -94.251, 90.961, -58.714 } };
	offset_pos1.tran.x = 10.0;
	offset_pos1.rpy.rx = -10.0;
	offset_pos2.tran.x = 30.0;
	offset_pos2.rpy.rx = -5.0;

	sp.circle_num = 3;
	sp.circle_angle = 5.0;
	sp.rad_init = 50.0;
	sp.rad_add = 10.0;
	sp.rotaxis_add = 5.0;
	sp.rot_direction = 0;

	int tool = 0;
	int user = 0;
	float vel = 70.0;
	float acc = 30.0;
	float ovl = 100.0;
	float blendT = 0.0;
	uint8_t flag = 2;

	uint8_t result = 0;
	uint8_t block = 0;
	float acc_deg[6];
	memset(acc_deg, 0, sizeof(acc_deg));

	robot.SetSpeed(50);

	robot.GetActualJointAccDegree(block, acc_deg);
	printf("acc for every joint is: %f %f %f %f %f %f  deg/s^2\n", acc_deg[0], acc_deg[1], acc_deg[2], acc_deg[3], acc_deg[4], acc_deg[5]);

	int ret = robot.GetForwardKin(&j, &desc_pos); 
	printf("GetForwardKin ret is: %d \n", ret);
	printf("GetForwardKin result:%f,%f,%f,%f,%f,%f\n", desc_pos.tran.x, desc_pos.tran.y, desc_pos.tran.z, \
											desc_pos.rpy.rx, desc_pos.rpy.ry, desc_pos.rpy.rz);

	int retval = 0;
	JointPos start_joint_pose;
	memset(&start_joint_pose, 0, sizeof(JointPos));
	retval = robot.GetInverseKinRef(0, &desc_pos, &j, &start_joint_pose);
	printf("GetInverseKinRef retval is: %d \n", retval);
	printf("joint is: %f, %f, %f,%f, %f, %f\n", start_joint_pose.jPos[0], start_joint_pose.jPos[1], start_joint_pose.jPos[2],
		   start_joint_pose.jPos[3], start_joint_pose.jPos[4], start_joint_pose.jPos[5]);

	if (ret == 0)
	{
		ret = robot.GetInverseKinHasSolution(1, &offset_pos1, &j, &result); 
		printf("GetInverseKinHasSolution ret: %d\n", result);
		if (0 == result)
		{
			printf("pose1 can not be solved\n");
		}
		else
		{
			memset(&start_joint_pose, 0, sizeof(JointPos));
			retval = robot.GetInverseKin(1, &offset_pos1, -1, &start_joint_pose);
			printf("GetInverseKin retval is: %d \n", retval);
			printf("GetInverseKin result is: %f, %f, %f, %f, %f, %f\n", start_joint_pose.jPos[0], start_joint_pose.jPos[1], start_joint_pose.jPos[2],
				   start_joint_pose.jPos[3], start_joint_pose.jPos[4], start_joint_pose.jPos[5]);

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

	DescPose desc_pos_dt;
	memset(&desc_pos_dt, 0, sizeof(DescPose));

	desc_pos_dt.tran.z = -0.5;
	float pos_gain[6] = { 0.0, 0.0, 1.0, 0.0, 0.0, 0.0 };
	int mode = 2;
	float vell = 0.0;
	float accl = 0.0;
	float cmdT = 0.008;
	float filterT = 0.0;
	float gain = 0.0;
	int count = 100;

	robot.SetSpeed(10);
	while (count)
	{
		robot.ServoCart(mode, &desc_pos_dt, pos_gain, accl, vell, cmdT, filterT, gain);
		count -= 1;
		robot.WaitMs(cmdT * 1000);
	}

	return 0;
}