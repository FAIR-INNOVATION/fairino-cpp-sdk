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

	float yangle, zangle;
	int flag = 0;
	JointPos j_deg, j_rad;
	DescPose tcp, flange, tcp_offset, wobj_offset;
	DescTran cog;
	int id;
	float torques[6] = { 0.0 };
	float weight;
	float neg_deg[6] = { 0.0 }, pos_deg[6] = { 0.0 };
	float t_ms;
	int config;
	float vel;

	memset(&j_deg, 0, sizeof(JointPos));
	memset(&j_rad, 0, sizeof(JointPos));
	memset(&tcp, 0, sizeof(DescPose));
	memset(&flange, 0, sizeof(DescPose));
	memset(&tcp_offset, 0, sizeof(DescPose));
	memset(&wobj_offset, 0, sizeof(DescPose));
	memset(&cog, 0, sizeof(DescTran));

	robot.GetRobotInstallAngle(&yangle, &zangle);
	printf("yangle:%f,zangle:%f\n", yangle, zangle);

	robot.GetActualJointPosDegree(flag, &j_deg);
	printf("joint pos deg:%f,%f,%f,%f,%f,%f\n", j_deg.jPos[0], j_deg.jPos[1], j_deg.jPos[2], j_deg.jPos[3], j_deg.jPos[4], j_deg.jPos[5]);

	robot.GetActualJointPosRadian(flag, &j_rad);
	printf("joint pos rad:%f,%f,%f,%f,%f,%f\n", j_rad.jPos[0], j_rad.jPos[1], j_rad.jPos[2], j_rad.jPos[3], j_rad.jPos[4], j_rad.jPos[5]);

	robot.GetActualTCPPose(flag, &tcp);
	printf("tcp pose:%f,%f,%f,%f,%f,%f\n", tcp.tran.x, tcp.tran.y, tcp.tran.z, tcp.rpy.rx, tcp.rpy.ry, tcp.rpy.rz);

	robot.GetActualToolFlangePose(flag, &flange);
	printf("flange pose:%f,%f,%f,%f,%f,%f\n", flange.tran.x, flange.tran.y, flange.tran.z, flange.rpy.rx, flange.rpy.ry, flange.rpy.rz);

	robot.GetActualTCPNum(flag, &id);
	printf("tcp num:%d\n", id);

	robot.GetActualWObjNum(flag, &id);
	printf("wobj num:%d\n", id);

	robot.GetJointTorques(flag, torques);
	printf("torques:%f,%f,%f,%f,%f,%f\n", torques[0], torques[1], torques[2], torques[3], torques[4], torques[5]);

	robot.GetTargetPayload(flag, &weight);
	printf("payload weight:%f\n", weight);

	robot.GetTargetPayloadCog(flag, &cog);
	printf("payload cog:%f,%f,%f\n", cog.x, cog.y, cog.z);

	robot.GetTCPOffset(flag, &tcp_offset);
	printf("tcp offset:%f,%f,%f,%f,%f,%f\n", tcp_offset.tran.x, tcp_offset.tran.y, tcp_offset.tran.z, tcp_offset.rpy.rx, tcp_offset.rpy.ry, tcp_offset.rpy.rz);

	robot.GetWObjOffset(flag, &wobj_offset);
	printf("wobj offset:%f,%f,%f,%f,%f,%f\n", wobj_offset.tran.x, wobj_offset.tran.y, wobj_offset.tran.z, wobj_offset.rpy.rx, wobj_offset.rpy.ry, wobj_offset.rpy.rz);

	robot.GetJointSoftLimitDeg(flag, neg_deg, pos_deg);
	printf("neg limit deg:%f,%f,%f,%f,%f,%f\n", neg_deg[0], neg_deg[1], neg_deg[2], neg_deg[3], neg_deg[4], neg_deg[5]);
	printf("pos limit deg:%f,%f,%f,%f,%f,%f\n", pos_deg[0], pos_deg[1], pos_deg[2], pos_deg[3], pos_deg[4], pos_deg[5]);

	robot.GetSystemClock(&t_ms);
	printf("system clock:%f\n", t_ms);

	robot.GetRobotCurJointsConfig(&config);
	printf("joint config:%d\n", config);

	robot.GetDefaultTransVel(&vel);
	printf("trans vel:%f\n", vel);

	if (0)
	{
		int retval = 0;
		float joint_speed_deg[6] = {0};
		retval = robot.GetActualJointSpeedsDegree(1, joint_speed_deg);
		printf("GetActualJointSpeedsDegree retval is: %d \n", retval);
		printf("joint degree speed is: %f, %f, %f, %f, %f, %f \n", joint_speed_deg[0], joint_speed_deg[1], joint_speed_deg[2],\
															joint_speed_deg[3], joint_speed_deg[4], joint_speed_deg[5]);


		float joint_acc_deg[6] = {0};
		retval = robot.GetActualJointAccDegree(1, joint_acc_deg);
		printf("GetActualJointAccDegree retval is: %d \n", retval);
		printf("joint degree acc is: %f, %f, %f, %f, %f, %f \n", joint_acc_deg[0], joint_acc_deg[1], joint_acc_deg[2],\
															joint_acc_deg[3], joint_acc_deg[4], joint_acc_deg[5]);

		float tcp_speed = 0;
		float ori_speed = 0;
		retval = robot.GetTargetTCPCompositeSpeed(1, &tcp_speed, &ori_speed);
		printf("GetTargetTCPCompositeSpeed retval is: %d \n", retval);
		printf("tcp_speed is:%f, ori_speed is: %f \n", tcp_speed, ori_speed);


		retval = robot.GetActualTCPCompositeSpeed(1, &tcp_speed, &ori_speed);
		printf("GetActualTCPCompositeSpeed retval is: %d \n", retval);
		printf("tcp_speed is:%f, ori_speed is: %f \n", tcp_speed, ori_speed);

		float targer_tcp_speed[6] = {0};
		retval = robot.GetTargetTCPSpeed(1, targer_tcp_speed);
		printf("GetTargetTCPSpeed retval is: %d \n", retval);
		printf("xyz is: %f, %f, %f; rpy is: %f, %f, %f\n", targer_tcp_speed[0], targer_tcp_speed[1], targer_tcp_speed[2],  \
													targer_tcp_speed[3], targer_tcp_speed[4], targer_tcp_speed[5]);


		float actual_tcp_speed[6] = {0};
		robot.GetActualTCPSpeed(1, actual_tcp_speed);
		printf("GetActualTCPSpeed retval is: %d \n", retval);
		printf("xyz is: %f, %f, %f; rpy is: %f, %f, %f\n", actual_tcp_speed[0], actual_tcp_speed[1], actual_tcp_speed[2],  \
													actual_tcp_speed[3], actual_tcp_speed[4], actual_tcp_speed[5]);

		JointPos start_joint_pose = {{0}};
		DescPose start_cart_pose;
		memset(&start_cart_pose, 0, sizeof(DescPose));

		retval = robot.GetForwardKin(&start_joint_pose, &start_cart_pose);
		printf("GetForwardKin retval is: %d \n", retval);
		printf("desc_pos:%f,%f,%f,%f,%f,%f\n",start_cart_pose.tran.x,start_cart_pose.tran.y,start_cart_pose.tran.z,
											start_cart_pose.rpy.rx,start_cart_pose.rpy.ry,start_cart_pose.rpy.rz);

		uint8_t solve = 0;
		retval = robot.GetInverseKinHasSolution(0, &start_cart_pose, &start_joint_pose, &solve);
		printf("GetInverseKinHasSolution retval is: %d, solve is: %u\n", retval, solve);

		retval = robot.GetInverseKinRef(0, &start_cart_pose, &start_joint_pose, &start_joint_pose);
		printf("GetInverseKinRef retval is: %d \n", retval);
		printf("xyz is: %f, %f, %f; rpy is: %f, %f, %f\n", start_joint_pose.jPos[0], start_joint_pose.jPos[1], start_joint_pose.jPos[2],  \
													start_joint_pose.jPos[3], start_joint_pose.jPos[4], start_joint_pose.jPos[5]);


		memset(&start_joint_pose, 0, sizeof(JointPos));
		retval = robot.GetInverseKin(0, &start_cart_pose, -1, &start_joint_pose);
		printf("GetInverseKin retval is: %d \n", retval);
		printf("xyz is: %f, %f, %f; rpy is: %f, %f, %f\n", start_joint_pose.jPos[0], start_joint_pose.jPos[1], start_joint_pose.jPos[2],  \
													start_joint_pose.jPos[3], start_joint_pose.jPos[4], start_joint_pose.jPos[5]);
		
		uint8_t motion_done = 0;
		retval = robot.GetRobotMotionDone(&motion_done);
		printf("GetRobotMotionDone retval is: %d , motion done is: %u\n", retval, motion_done);

		int main_code = 0;
		int sub_code = 0;
		retval = robot.GetRobotErrorCode(&main_code, &sub_code);
		printf("GetRobotMotionDone retval is: %d , amin code is:%d, sub code is: %d\n", retval, main_code, sub_code);

		int que_len = 0;
		retval = robot.GetMotionQueueLength(&que_len);
		printf("GetMotionQueueLength retval is: %d, queue length is: %d \n", retval, que_len);
		
		char point_name[64] = "111";
		float point_data[20] = {0};
		retval = robot.GetRobotTeachingPoint(point_name, point_data);
		printf("GetRobotTeachingPoint retval is: %d \n", retval);
		for(int i=0; i<20; i++)
		{
			if(0 == i%3){
				printf("\n");
			}
			printf("point data %d is: %f.", i, point_data[i]);
		}


		for (int i = 1; i < 21; i++)
		{
			robot.SetSysVarValue(i, i + 0.5);
			std::this_thread::sleep_for(std::chrono::milliseconds(10)); // 单位ms
		}

		float sys_value = 0;
		for (int i = 1; i < 21; i++)
		{
			robot.GetSysVarValue(i, &sys_value);
			printf("sys value:%f\n", sys_value);
		}
	}

	return 0;
}