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

    int retval = 0;
    char traj_file_name[30] = "/fruser/traj/tra我.txt";
    retval = robot.LoadTrajectoryJ(traj_file_name, 100, 1);
    printf("LoadTrajectoryJ %s, retval is: %d\n",traj_file_name, retval);

    DescPose traj_start_pose;
    memset(&traj_start_pose, 0, sizeof(DescPose));
    retval = robot.GetTrajectoryStartPose(traj_file_name, &traj_start_pose);
    printf("GetTrajectoryStartPose is: %d\n", retval);
    printf("desc_pos:%f,%f,%f,%f,%f,%f\n",traj_start_pose.tran.x,traj_start_pose.tran.y,traj_start_pose.tran.z,
											traj_start_pose.rpy.rx,traj_start_pose.rpy.ry,traj_start_pose.rpy.rz);

    std::this_thread::sleep_for(std::chrono::seconds(5));

    robot.SetSpeed(50);
    robot.MoveCart(&traj_start_pose, 0, 0, 100, 100, 100, -1, -1);

    int traj_num = 0;
    retval = robot.GetTrajectoryPointNum(&traj_num);
    printf("GetTrajectoryStartPose retval is: %d, traj num is: %d\n", retval, traj_num);

    retval = robot.SetTrajectoryJSpeed(50.0);
    printf("SetTrajectoryJSpeed is: %d\n", retval);

    ForceTorque traj_force;
    memset(&traj_force, 0, sizeof(ForceTorque));
    traj_force.fx = 10;
    retval = robot.SetTrajectoryJForceTorque(&traj_force);
    printf("SetTrajectoryJForceTorque retval is: %d\n", retval);

    retval = robot.SetTrajectoryJForceFx(10.0);
    printf("SetTrajectoryJForceFx retval is: %d\n", retval);

    retval = robot.SetTrajectoryJForceFy(0.0);
    printf("SetTrajectoryJForceFy retval is: %d\n", retval);

    retval = robot.SetTrajectoryJForceFz(0.0);
    printf("SetTrajectoryJForceFz retval is: %d\n", retval);

    retval = robot.SetTrajectoryJTorqueTx(10.0);
    printf("SetTrajectoryJTorqueTx retval is: %d\n", retval);

    retval = robot.SetTrajectoryJTorqueTy(10.0);
    printf("SetTrajectoryJTorqueTy retval is: %d\n", retval);

    retval = robot.SetTrajectoryJTorqueTz(10.0);
    printf("SetTrajectoryJTorqueTz retval is: %d\n", retval);

    retval = robot.MoveTrajectoryJ();
    printf("MoveTrajectoryJ retval is: %d\n", retval);

}
