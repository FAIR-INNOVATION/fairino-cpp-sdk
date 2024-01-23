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

    //Constant force parameters
    uint8_t status = 1;  
    int sensor_num = 1; 
    float gain[6] = {0.0001,0.0,0.0,0.0,0.0,0.0};  //
    uint8_t adj_sign = 0;  
    uint8_t ILC_sign = 0; 
    float max_dis = 100.0; 
    float max_ang = 5.0;  

    ForceTorque ft;
    memset(&ft, 0, sizeof(ForceTorque));

    //Spiral exploration parameters
    int rcs = 0;  
    float dr = 0.7; 
    float fFinish = 1.0; 
    float t = 60000.0; 
    float vmax = 5.0; 

    //Straight line insertion parameter
    float force_goal = 50.0;  
    float lin_v = 3.0; 
    float lin_a = 0.0; 
    float disMax = 100.0; 
    uint8_t linorn = 1; 

    //Rotate the insertion parameters
    float angVelRot = 2.0; 
    float forceInsertion = 1.0;
    int angleMax= 45; 
    uint8_t orn = 1; 
    float angAccmax = 0.0;
    uint8_t rotorn = 2; 

    uint8_t select0[6] = {1,1,1,0,0,0};  //Six degrees of freedom to choose [fx, fy, fz, mx, my, mz], 0 - not in effect, 1 - in effect
    gain[0] = 0.0001;
    ft.fz = -20.0;
    status = 1;
    robot.FT_Control(status,sensor_num,select0,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);
    robot.FT_LinInsertion(rcs,force_goal,lin_v,lin_a,disMax,linorn);
    status = 0;
    robot.FT_Control(status,sensor_num,select0,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);

    uint8_t select1[6] = {1,1,1,0,0,0}; //Six degrees of freedom to choose [fx, fy, fz, mx, my, mz], 0 - not in effect, 1 - in effect
    ft.fz = -10.0;
    robot.FT_Control(status,sensor_num,select1,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);
    robot.FT_SpiralSearch(rcs,dr,fFinish,t,vmax);
    status = 0;
    robot.FT_Control(status,sensor_num,select1,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);

    uint8_t select2[6] = {1,1,1,0,0,0};  //Six degrees of freedom to choose [fx, fy, fz, mx, my, mz], 0 - not in effect, 1 - in effect
    gain[0] = 0.00005;
    ft.fz = -30.0;
    status = 1;
    robot.FT_Control(status,sensor_num,select2,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);
    robot.FT_LinInsertion(rcs,force_goal,lin_v,lin_a,disMax,linorn);
    status = 0;
    robot.FT_Control(status,sensor_num,select2,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);

    uint8_t select3[6] = {0,0,1,1,1,0};  //Six degrees of freedom to choose [fx, fy, fz, mx, my, mz], 0 - not in effect, 1 - in effect
    ft.fz = -10.0;
    gain[0] = 0.0001;
    status = 1;
    robot.FT_Control(status,sensor_num,select3,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);
    robot.FT_RotInsertion(rcs,angVelRot,forceInsertion,angleMax,orn,angAccmax,rotorn);
    status = 0;
    robot.FT_Control(status,sensor_num,select3,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);

    uint8_t select4[6] = {1,1,1,0,0,0};  //Six degrees of freedom to choose [fx, fy, fz, mx, my, mz], 0 - not in effect, 1 - in effect
    ft.fz = -30.0;
    status = 1;
    robot.FT_Control(status,sensor_num,select4,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);
    robot.FT_LinInsertion(rcs,force_goal,lin_v,lin_a,disMax,linorn);
    status = 0;
    robot.FT_Control(status,sensor_num,select4,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);

    return 0;
}