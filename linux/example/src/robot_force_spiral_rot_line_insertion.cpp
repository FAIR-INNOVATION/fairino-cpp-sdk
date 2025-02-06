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
    FRRobot robot;                 //实例化机器人对象
    robot.RPC("192.168.58.2");     //与机器人控制器建立通信连接

    //恒力参数
    uint8_t status = 1;  //恒力控制开启标志，0-关，1-开
    int sensor_num = 1; //力传感器编号
    float gain[6] = {0.0001,0.0,0.0,0.0,0.0,0.0};  //最大阈值
    uint8_t adj_sign = 0;  //自适应启停状态，0-关闭，1-开启
    uint8_t ILC_sign = 0;  //ILC控制启停状态，0-停止，1-训练，2-实操
    float max_dis = 100.0;  //最大调整距离
    float max_ang = 5.0;  //最大调整角度

    ForceTorque ft;
    memset(&ft, 0, sizeof(ForceTorque));

    //螺旋线探索参数
    int rcs = 0;  //参考坐标系，0-工具坐标系，1-基坐标系
    float dr = 0.7;  //每圈半径进给量，单位mm
    float fFinish = 1.0; //力或力矩阈值（0~100），单位N或Nm
    float t = 60000.0; //最大探索时间，单位ms
    float vmax = 5.0; //线速度最大值，单位mm/s

    //直线插入参数
    float force_goal = 50.0;  //力或力矩阈值（0~100），单位N或Nm
    float lin_v = 3.0; //直线速度，单位mm/s
    float lin_a = 0.0; //直线加速度，单位mm/s^2,暂不使用
    float disMax = 100.0; //最大插入距离，单位mm
    uint8_t linorn = 1; //插入方向，1-正方向，2-负方向

    //旋转插入参数
    float angVelRot = 2.0;  //旋转角速度，单位°/s
    float forceInsertion = 1.0; //力或力矩阈值（0~100），单位N或Nm
    int angleMax= 45; //最大旋转角度，单位°
    uint8_t orn = 1; //力的方向，1-fz,2-mz
    float angAccmax = 0.0; //最大旋转角加速度，单位°/s^2,暂不使用
    uint8_t rotorn = 2; //旋转方向，1-顺时针，2-逆时针

    uint8_t select0[6] = {1,1,1,0,0,0};  //六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
    gain[0] = 0.0001;
    ft.fz = -20.0;
    status = 1;
    robot.FT_Control(status,sensor_num,select0,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);
    robot.FT_LinInsertion(rcs,force_goal,lin_v,lin_a,disMax,linorn);
    status = 0;
    robot.FT_Control(status,sensor_num,select0,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);

    uint8_t select1[6] = {1,1,1,0,0,0}; //六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
    ft.fz = -10.0;
    robot.FT_Control(status,sensor_num,select1,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);
    robot.FT_SpiralSearch(rcs,dr,fFinish,t,vmax);
    status = 0;
    robot.FT_Control(status,sensor_num,select1,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);

    uint8_t select2[6] = {1,1,1,0,0,0};  //六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
    gain[0] = 0.00005;
    ft.fz = -30.0;
    status = 1;
    robot.FT_Control(status,sensor_num,select2,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);
    robot.FT_LinInsertion(rcs,force_goal,lin_v,lin_a,disMax,linorn);
    status = 0;
    robot.FT_Control(status,sensor_num,select2,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);

    uint8_t select3[6] = {0,0,1,1,1,0};  //六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
    ft.fz = -10.0;
    gain[0] = 0.0001;
    status = 1;
    robot.FT_Control(status,sensor_num,select3,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);
    robot.FT_RotInsertion(rcs,angVelRot,forceInsertion,angleMax,orn,angAccmax,rotorn);
    status = 0;
    robot.FT_Control(status,sensor_num,select3,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);

    uint8_t select4[6] = {1,1,1,0,0,0};  //六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
    ft.fz = -30.0;
    status = 1;
    robot.FT_Control(status,sensor_num,select4,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);
    robot.FT_LinInsertion(rcs,force_goal,lin_v,lin_a,disMax,linorn);
    status = 0;
    robot.FT_Control(status,sensor_num,select4,&ft,gain,adj_sign,ILC_sign,max_dis,max_ang);

    return 0;
}