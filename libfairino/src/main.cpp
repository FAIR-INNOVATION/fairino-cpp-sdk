#include "robot.h"
#ifdef WIN32
#include <string.h>
#include <windows.h>
#include <chrono>
#include <ws2tcpip.h>
#else
#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <cstring>
#include <unistd.h>
#endif

#include <cmath>
#include <chrono>
#include <thread>
#include "md5.hpp"
#include "FRTcpClient.h"
#include <bitset>
#include <algorithm>
#include <atomic>

using namespace std;
using std::chrono::duration_cast;
using std::chrono::system_clock;
using std::chrono::milliseconds;
using std::chrono::seconds;

int Sleep(int ms)
{
#ifdef WIN32
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
#else
    usleep(ms * 1000);
#endif
    return 0;
}

void UDPFrameCallBack(int srcType, int count, int cmdID, int len, std::string content)
{
    cout << "recv cmd: cmdID:  " << to_string(cmdID) << "  content is " << content << "  count is " << count << endl;;
    
    return;
}

#pragma region 机器人基础

int TestRobotCtrl(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    robot.SetReConnectParam(true, 30000, 500);
    int rtn = robot.RPC("192.168.57.2");
    
    char ip[64] = "";
    char version[64] = "";
    uint8_t state;
    robot.GetSDKVersion(version);
    printf("SDK version:%s\n", version);
    robot.GetControllerIP(ip);
    printf("controller ip:%s\n", ip);
    robot.Mode(1);
    robot.Sleep(1000);
    robot.DragTeachSwitch(1);
    robot.Sleep(1000);
    robot.IsInDragTeach(&state);
    printf("drag state :%u\n", state);
    robot.Sleep(3000);
    robot.DragTeachSwitch(0);
    robot.Sleep(1000);
    robot.IsInDragTeach(&state);
    printf("drag state :%u\n", state);
    robot.Sleep(3000);
    robot.RobotEnable(0);
    robot.Sleep(3000);
    robot.RobotEnable(1);
    robot.Mode(0);
    robot.Sleep(2000);
    robot.Mode(1);
    robot.Sleep(1000);
    rtn = robot.HiSpeedManualSwitch(1);
    printf("change high speed mode %d\n", rtn);
    robot.Sleep(3000);
    rtn = robot.HiSpeedManualSwitch(0);
    printf("change low speed mode %d\n", rtn);
    robot.Sleep(3000);
    robot.ShutDownRobotOS();
    robot.CloseRPC();
    return 0;
}

int TestGetVersions(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    char robotModel[64] = { 0 };
    char webversion[64] = { 0 };
    char controllerVersion[64] = { 0 };
    char ctrlBoxBoardversion[128] = { 0 };
    char driver1version[128] = { 0 };
    char driver2version[128] = { 0 };
    char driver3version[128] = { 0 };
    char driver4version[128] = { 0 };
    char driver5version[128] = { 0 };
    char driver6version[128] = { 0 };
    char endBoardversion[128] = { 0 };
    rtn = robot.GetSoftwareVersion(robotModel, webversion, controllerVersion);
    printf("Getsoftwareversion rtn is: %d\n", rtn);
    printf("robotmodel is: %s, webversion is: %s, controllerVersion is: %s \n\n", robotModel, webversion, controllerVersion);
    rtn = robot.GetHardwareVersion(ctrlBoxBoardversion, driver1version, driver2version, driver3version, driver4version, driver5version, driver6version, endBoardversion);
    printf("GetHardwareversion rtn is: %d\n", rtn);
    printf("GetHardwareversion get hardware versoin is: %s, %s, %s, %s, %s, %s, %s, %s\n\n", ctrlBoxBoardversion, driver1version, driver2version, driver3version, driver4version, driver5version, driver6version, endBoardversion);
    rtn = robot.GetFirmwareVersion(ctrlBoxBoardversion, driver1version, driver2version, driver3version, driver4version, driver5version, driver6version, endBoardversion);
    printf("GetFirmwareversion rtn is: %d\n", rtn);
    printf("GetHardwareversion get hardware versoin is: %s, %s, %s, %s, %s, %s, %s, %s\n\n", ctrlBoxBoardversion, driver1version, driver2version, driver3version, driver4version, driver5version, driver6version, endBoardversion);
    robot.CloseRPC();
    return 0;
}


/**
 * @brief 测试设置与读取机器人系统时间
 * @note QNX同步精度为分钟级
 */
void TestSetAndGetRobotTime() 
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    robot.SetReConnectParam(true, 30000, 500);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0) {
        return;
    }
    float t_ms = 0.0f;
    int ret = robot.GetSystemClock(&t_ms);
    if (ret == 0) {
        constexpr int64_t OFFSET_8H_MS = 8LL * 3600LL * 1000LL;
        int64_t rawMs = static_cast<int64_t>(t_ms);
        int64_t fixedMs = rawMs - OFFSET_8H_MS;
        std::time_t sec = static_cast<std::time_t>(fixedMs / 1000LL);
        std::tm utcTm {};
#if defined (_WIN32) || defined (_WIN64)
        localtime_s(&utcTm, &sec);
#else
        localtime_r(&sec, &utcTm);
#endif
        char utcBuf[64] = { 0 };
        strftime(utcBuf, sizeof(utcBuf), "%Y-%m-%d %H:%M:%S", &utcTm);
        printf("system clock : %.0f\n", t_ms);
        printf("BEFORE UTC Time : %s\n", utcBuf);
    }
    else {
        printf("GetSystemClock failed, ret:%d\n", ret);
    }

    robot.SetRobotTime();
    robot.Sleep(1000);
    float t_ms_after = 0.0f;
    ret = robot.GetSystemClock(&t_ms_after);
    if (ret == 0) {
        printf("system clock : %.0f\n", t_ms_after);

        constexpr int64_t OFFSET_8H_MS = 8LL * 3600LL * 1000LL;
        int64_t robotMs = static_cast<int64_t>(t_ms_after);
        int64_t robotMsFixed = robotMs - OFFSET_8H_MS;

        auto nowSys = std::chrono::system_clock::now();
        int64_t pcMs = std::chrono::duration_cast<std::chrono::milliseconds>(nowSys.time_since_epoch()).count();
        constexpr int64_t MS_PER_MINUTE = 60LL * 1000LL;
        bool isConsistent = ((robotMsFixed / MS_PER_MINUTE) == (pcMs / MS_PER_MINUTE));

        std::time_t robotSec = robotMsFixed / 1000LL;
        std::tm robotTm {};
#if defined (_WIN32) || defined (_WIN64)
        localtime_s(&robotTm, &robotSec);
#else
        localtime_r(&robotSec, &robotTm);
#endif

        std::time_t pcSec = pcMs / 1000LL;
        std::tm pcTm {};
#if defined (_WIN32) || defined (_WIN64)
        localtime_s(&pcTm, &pcSec);
#else
        localtime_r(&pcSec, &pcTm);
#endif

        char pcBuf[64] = { 0 };
        char robotBuf[64] = { 0 };
        strftime(pcBuf, sizeof(pcBuf), "%Y-%m-%d %H:%M", &pcTm);
        strftime(robotBuf, sizeof(robotBuf), "%Y-%m-%d %H:%M", &robotTm);
        if (isConsistent) {
            printf("Consistent | PC time: %s | Robot time: %s\n", pcBuf, robotBuf);
        }
        else {
            printf("[Inconsistent] | PC time: %s | Robot time: %s\n", pcBuf, robotBuf);
        }
    }
    else {
        printf("GetSystemClock failed, ret:%d\n", ret);
    }
    robot.Sleep(1000);
    robot.CloseRPC();
    robot.Sleep(1000);
}




#pragma endregion
#pragma region 机器人运动

int TestJOG(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    for (int i = 0; i < 6; i++)
    {
        robot.StartJOG(0, i + 1, 0, 20.0, 20.0, 30.0);
        robot.Sleep(1000);
        robot.ImmStopJOG();
        robot.Sleep(1000);
    }
    for (int i = 0; i < 6; i++)
    {
        robot.StartJOG(2, i + 1, 0, 20.0, 20.0, 30.0);
        robot.Sleep(1000);
        robot.ImmStopJOG();
        robot.Sleep(1000);
    }
    for (int i = 0; i < 6; i++)
    {
        robot.StartJOG(4, i + 1, 0, 20.0, 20.0, 30.0);
        robot.Sleep(1000);
        robot.StopJOG(5);
        robot.Sleep(1000);
    }
    for (int i = 0; i < 6; i++)
    {
        robot.StartJOG(8, i + 1, 0, 20.0, 20.0, 30.0);
        robot.Sleep(1000);
        robot.StopJOG(9);
        robot.Sleep(1000);
    }
    robot.CloseRPC();
    return 0;
}

int TestMove(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;

    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    robot.SetReConnectParam(true, 30000, 500);
    int rtn = robot.RPC("192.168.57.2");
    if (rtn != 0)
    {
        return -1;
    }

    JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
    JointPos j2(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
    JointPos j3(-29.777, -84.536, 109.275, -114.075, -86.655, 74.257);
    JointPos j4(-31.154, -95.317, 94.276, -88.079, -89.740, 74.256);
    DescPose desc_pos1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
    DescPose desc_pos2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
    DescPose desc_pos3(-487.434, 154.362, 308.576, 176.600, 0.268, -14.061);
    DescPose desc_pos4(-443.165, 147.881, 480.951, 179.511, -0.775, -15.409);
    DescPose offset_pos(0, 0, 0, 0, 0, 0);
    ExaxisPos epos(0, 0, 0, 0);
    int tool = 0;
    int user = 0;
    float vel = 100.0;
    float acc = 100.0;
    float ovl = 100.0;
    float oacc = 100.0;
    float blendT = 0.0;
    float blendR = 0.0;
    uint8_t flag = 0;
    uint8_t search = 0;
    int blendMode = 0;
    int velAccMode = 0;
    robot.SetSpeed(20);
    rtn = robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
    printf("movej errcode:%d\n", rtn);
    rtn = robot.MoveL(&j2, &desc_pos2, tool, user, vel, acc, ovl, blendR, blendMode, &epos, search, flag, &offset_pos, oacc, velAccMode);
    printf("movel errcode:%d\n", rtn);
    rtn = robot.MoveC(&j3, &desc_pos3, tool, user, vel, acc, &epos, flag, &offset_pos, &j4, &desc_pos4, tool, user, vel, acc, &epos, flag, &offset_pos, ovl, blendR, oacc, velAccMode);
    printf("movec errcode:%d\n", rtn);
    rtn = robot.MoveJ(&j2, &desc_pos2, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
    printf("movej errcode:%d\n", rtn);
    rtn = robot.Circle(&j3, &desc_pos3, tool, user, vel, acc, &epos, &j1, &desc_pos1, tool, user, vel, acc, &epos, ovl, flag, &offset_pos, oacc, -1, velAccMode);
    printf("circle errcode:%d\n", rtn);
    rtn = robot.MoveCart(&desc_pos4, tool, user, vel, acc, ovl, blendT, -1);
    printf("MoveCart errcode:%d\n", rtn);
    rtn = robot.MoveJ(&j1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
    printf("movej errcode:%d\n", rtn);
    rtn = robot.MoveL(&desc_pos2, tool, user, vel, acc, ovl, blendR, blendMode, &epos, search, flag, &offset_pos, -1, velAccMode);
    printf("movel errcode:%d\n", rtn);
    rtn = robot.MoveC(&desc_pos3, tool, user, vel, acc, &epos, flag, &offset_pos, &desc_pos4, tool, user, vel, acc, &epos, flag, &offset_pos, ovl, blendR, -1, velAccMode);
    printf("movec errcode:%d\n", rtn);
    rtn = robot.MoveJ(&j2, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
    printf("movej errcode:%d\n", rtn);
    rtn = robot.Circle(&desc_pos3, tool, user, vel, acc, &epos, &desc_pos1, tool, user, vel, acc, &epos, ovl, flag, &offset_pos, oacc, blendR, -1, velAccMode);
    printf("circle errcode:%d\n", rtn);
    robot.CloseRPC();
    return 0;
}

int TestSpiral(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    JointPos j(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
    DescPose desc_pos(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
    DescPose offset_pos1(50, 0, 0, -30, 0, 0);
    DescPose offset_pos2(50, 0, 0, -5, 0, 0);
    ExaxisPos epos(0, 0, 0, 0);
    SpiralParam sp;
    sp.circle_num = 5;
    sp.circle_angle = 5.0;
    sp.rad_init = 50.0;
    sp.rad_add = 10.0;
    sp.rotaxis_add = 10.0;
    sp.rot_direction = 0;
    int tool = 0;
    int user = 0;
    float vel = 100.0;
    float acc = 100.0;
    float ovl = 100.0;
    float blendT = 0.0;
    uint8_t flag = 2;
    robot.SetSpeed(20);
    rtn = robot.MoveJ(&j, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos1);
    printf("movej errcode:%d\n", rtn);
    rtn = robot.NewSpiral(&desc_pos, tool, user, vel, acc, &epos, ovl, flag, &offset_pos2, sp);
    printf("newspiral errcode:%d\n", rtn);
    robot.CloseRPC();
    return 0;
}

int TestServoJ(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    JointPos j(0, 0, 0, 0, 0, 0);
    ExaxisPos epos(0, 0, 0, 0);
    float vel = 0.0;
    float acc = 0.0;
    float cmdT = 0.008;
    float filterT = 0.0;
    float gain = 0.0;
    uint8_t flag = 0;
    int count = 500;
    float dt = 0.1;
    int cmdID = 0;
    int ret = robot.GetActualJointPosDegree(flag, &j);
    if (ret == 0)
    {
        robot.ServoMoveStart();
        while (count)
        {
            robot.ServoJ(&j, &epos, acc, vel, cmdT, filterT, gain, cmdID);
            j.jPos[0] += dt;
            count -= 1;
            robot.WaitMs(cmdT * 1000);
        }
        robot.ServoMoveEnd();
    }
    else
    {
        printf("GetActualJointPosDegree errcode:%d\n", ret);
    }
    robot.CloseRPC();
    return 0;
}

int TestServoJUDP(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    int rtn = 0;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    rtn = robot.SetCmdRpyCallback(UDPFrameCallBack);
    printf("SetCmdRpyCallback rtn is %d\n", rtn);
    rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 50);
    JointPos j(0, -90, 90, 0, 0, 0);
    ExaxisPos epos(0, 0, 0, 0);
    DescPose offset_pos(0, 0, 0, 0, 0, 0);
    while (true)
    {
        robot.MoveJ(&j, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        float vel = 0.0;
        float acc = 0.0;
        float cmdT = 0.016;
        float filterT = 0.0;
        float gain = 0.0;
        uint8_t flag = 0;
        float dt = 0.1;
        int cmdID = 0;
        int ret = robot.GetActualJointPosDegree(flag, &j);
        if (ret != 0)
        {
            printf("GetActualJointPosDegree errcode:%d\n", ret);
        }
        int comType = 1;
        int count = 300;
        rtn = robot.ServoMoveStart(comType);
        printf("ServoMoveStart rtn is %d\n", rtn);
        while (count)
        {
            rtn = robot.ServoJ(&j, &epos, acc, vel, cmdT, filterT, gain, cmdID, comType);
            printf("ServoJ rtn is %d\n", rtn);
            j.jPos[0] += dt;
            j.jPos[1] += dt;
            j.jPos[2] += dt;
            j.jPos[3] += dt;
            j.jPos[4] += dt;
            j.jPos[5] += dt;
            //epos.ePos[0] += dt;
            count -= 1;
            robot.Sleep(15);
        }
        robot.ServoMoveEnd(comType);
        printf("ServoMoveEnd rtn is %d\n", rtn);
        count = 300;
        robot.ServoMoveStart(comType);
        printf("ServoMoveStart rtn is %d\n", rtn);
        while (count)
        {
            robot.ServoJ(&j, &epos, acc, vel, cmdT, filterT, gain, cmdID, comType);
            printf("ServoJ rtn is %d\n", rtn);
            j.jPos[0] -= dt;
            j.jPos[1] -= dt;
            j.jPos[2] -= dt;
            j.jPos[3] -= dt;
            j.jPos[4] -= dt;
            j.jPos[5] -= dt;
            //epos.ePos[0] -= dt;
            count -= 1;
            robot.Sleep(15);
        }
        robot.ServoMoveEnd(comType);
        printf("ServoMoveEnd rtn is %d\n", rtn);
    }
    robot.Sleep(4000);
    robot.CloseRPC();
    return 0;
}

int TestServoJMultiPos()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    robot.SetReConnectParam(true, 30000, 500);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }

    std::ifstream fin("C://Users/fr/Desktop/ServoJPath.txt");
    if (!fin.is_open())
    {
        std::cerr << "无法打开文件 jianzhipath.txt\n";
        return -1;
    }

    std::vector<JointPos> all_joint_data;
    std::string line;
    while (std::getline(fin, line))
    {
        std::istringstream iss(line);
        double val;
        int col_idx = 0;
        std::vector<double> one_pose;
        JointPos onePose = {};
        while (iss >> val)
        {
            if (col_idx >= 1 && col_idx <= 6)
            {
                one_pose.push_back(val);
            }
            col_idx++;
        }

        if (one_pose.size() == 6)
        {
            onePose.jPos[0] = one_pose[0];
            onePose.jPos[1] = one_pose[1];
            onePose.jPos[2] = one_pose[2];
            onePose.jPos[3] = one_pose[3];
            onePose.jPos[4] = one_pose[4];
            onePose.jPos[5] = one_pose[5];
            all_joint_data.push_back(onePose);
        }
    }

    fin.close();
    std::cout << "一共读取到 " << all_joint_data.size() << " 组关节位置\n";

    std::vector<JointPos> back_forth_path;
    back_forth_path.insert(back_forth_path.end(), all_joint_data.begin(), all_joint_data.end());
    for (auto it = all_joint_data.rbegin() + 1; it != all_joint_data.rend(); ++it)
    {
        back_forth_path.push_back(*it);
    }

    ExaxisPos epos = {};
    DescPose offset_pos = {};
    robot.Sleep(1000);

    while (1)
    {
        robot.ResetAllError();
        robot.MoveJ(&all_joint_data[0], 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        int moveCount = 0;
        while (moveCount < back_forth_path.size() - 10)
        {
            robot.GetRobotRealTimeState(&pkg);

            int singleServoJCount = 50 - pkg.mc_queue_len;
            if (singleServoJCount <= 0)
            {
                robot.Sleep(100);
                continue;
            }

            if (singleServoJCount > 10)
            {
                singleServoJCount = 10;
            }

            std::vector<JointPos> joint_pos = {};
            for (int j = 0; j < singleServoJCount; j++)
            {
                joint_pos.push_back(back_forth_path[moveCount]);
                moveCount++;
            }

            ExaxisPos axisPos = { 0.0, 0.0, 0.0, 0.0 };
            int servoJCmdCount = 0;
            rtn = robot.ServoJ(joint_pos, &axisPos, 100.0, 100.0, 0.008, 0.008, 1.0, servoJCmdCount);
            if (rtn != 0)
            {
                break;
            }
        }

        robot.Sleep(4000);
    }

    robot.Sleep(1000);
    robot.CloseRPC();
    robot.Sleep(1000);
    return 0;
}

int TestServoJT(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    robot.DragTeachSwitch(1);
    float torques[] = { 0, 0, 0, 0, 0, 0 };
    robot.GetJointTorques(1, torques);
    int count = 100;
    robot.ServoJTStart();
    int error = 0;
    while (count > 0)
    {
        error = robot.ServoJT(torques, 0.001);
        count = count - 1;
        robot.Sleep(1);
    }
    error = robot.ServoJTEnd();
    robot.DragTeachSwitch(0);
    robot.CloseRPC();
    return 0;
}

int ServoJTWithSafety(FRRobot* robot)
{
    robot->ResetAllError();
    robot->Sleep(500);
    float torques[] = { 0, 0, 0, 0, 0, 0 };
    robot->GetJointTorques(1, torques);
    robot->ServoJTStart();
    ROBOT_STATE_PKG pkg = {};
    robot->DragTeachSwitch(1);
    int checkFlag = 3;
    //double jPowerLimit[6] = {1, 1, 1, 1, 1, 1};
    double jPowerLimit[6] = { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 };
    double jVelLimit[6] = { 181, 80, 80, 80, 80, 80 };
    int count = 800000;
    int error = 0;
    while (count > 0)
    {
        torques[2] = torques[2] + 0.01;
        error = robot->ServoJT(torques, 0.008, checkFlag, jPowerLimit, jVelLimit);
        if (error != 0)
        {
            robot->ServoJTEnd();
        }
        printf("ServoJT rtn is %d\n", error);
        count = count - 1;
        robot->Sleep(1);
        robot->GetRobotRealTimeState(&pkg);
        printf("maincode %d, subcode %d\n", pkg.main_code, pkg.sub_code);
    }
    robot->DragTeachSwitch(0);
    error = robot->ServoJTEnd();
    return 0;
}


int TestServoJTUDP(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    robot.SetCmdRpyCallback(UDPFrameCallBack);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    JointPos j(0, -90, 90, 0, 0, 0);
    ExaxisPos epos(0, 0, 0, 0);
    DescPose offset_pos(0, 0, 0, 0, 0, 0);
    robot.MoveJ(&j, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.Sleep(1000);
    robot.DragTeachSwitch(1);
    float torques[] = { 0, 0, 0, 0, 0, 0 };
    robot.GetJointTorques(1, torques);
    int comType = 0;
    int count = 100;
    int checkFlag = 3;
    double jPowerLimit[6] = { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 };
    double jVelLimit[6] = { 80, 80, 80, 80, 80, 80 };
    rtn = robot.ServoJTStart(comType);
    printf("ServoJTStart rtn is %d\n", rtn);
    while (true)
    {
        torques[0] = 0.05;
        rtn = robot.ServoJT(torques, 0.001, checkFlag, jPowerLimit, jVelLimit, comType);
        printf("ServoJT rtn is %d\n", rtn);
        robot.Sleep(1);
        robot.GetRobotRealTimeState(&pkg);
        if (pkg.jt_cur_pos[0] > 30)
        {
            break;
        }
    }
    while (true)
    {
        torques[0] = -0.03;
        rtn = robot.ServoJT(torques, 0.001, checkFlag, jPowerLimit, jVelLimit, comType);
        printf("ServoJT rtn is %d\n", rtn);
        robot.Sleep(1);
        robot.GetRobotRealTimeState(&pkg);
        if (pkg.jt_cur_pos[0] < 0 || pkg.jt_cur_pos[1] < -110)
        {
            break;
        }
    }
    rtn = robot.ServoJTEnd(comType);
    printf("ServoJTEnd rtn is %d\n", rtn);
    robot.DragTeachSwitch(0);
    robot.Sleep(1000);
    robot.CloseRPC();
    return 0;
}

int TestServoCart(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    DescPose desc_pos_dt = { -129.207, -514.784 , 265.821 , 175.028 , -6.671 , -167.027 };
    robot.GetActualTCPPose(0, &desc_pos_dt);
    ExaxisPos exaxis = { 0.0, 0.0, 0.0, 0.0 };
    float pos_gain[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    int mode = 0;
    float vel = 0.0;
    float acc = 0.0;
    float cmdT = 0.008;
    float filterT = 0.0;
    float gain = 0.0;
    uint8_t flag = 0;
    int count = 5000;
    robot.SetSpeed(20);
    while (count)
    {
        rtn = robot.ServoCart(mode, &desc_pos_dt, exaxis, pos_gain, acc, vel, cmdT, filterT, gain);
        printf("ServoCart rtn is %d\n", rtn);
        count -= 1;
        desc_pos_dt.tran.x += 0.01;
        //exaxis.ePos[0] += 0.01;
    }
    robot.CloseRPC();
    return 0;
}

int TestSpline(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
    JointPos j2(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
    JointPos j3(-61.954, -84.409, 108.153, -116.316, -91.283, 74.260);
    JointPos j4(-89.575, -80.276, 102.713, -116.302, -91.284, 74.267);
    DescPose desc_pos1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
    DescPose desc_pos2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
    DescPose desc_pos3(-327.622, 402.230, 320.402, -178.067, 2.127, -46.207);
    DescPose desc_pos4(-104.066, 544.321, 327.023, -177.715, 3.371, -73.818);
    DescPose offset_pos(0, 0, 0, 0, 0, 0);
    ExaxisPos epos(0, 0, 0, 0);
    int tool = 0;
    int user = 0;
    float vel = 100.0;
    float acc = 100.0;
    float ovl = 100.0;
    float blendT = -1.0;
    uint8_t flag = 0;
    robot.SetSpeed(20);
    int err1 = robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
    printf("movej errcode:%d\n", err1);
    robot.SplineStart();
    robot.SplinePTP(&j1, &desc_pos1, tool, user, vel, acc, ovl);
    robot.SplinePTP(&j2, &desc_pos2, tool, user, vel, acc, ovl);
    robot.SplinePTP(&j3, &desc_pos3, tool, user, vel, acc, ovl);
    robot.SplinePTP(&j4, &desc_pos4, tool, user, vel, acc, ovl);
    robot.SplineEnd();
    err1 = robot.MoveJ(&j1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
    printf("movej errcode:%d\n", err1);
    robot.SplineStart();
    robot.SplinePTP(&j1, tool, user, vel, acc, ovl);
    robot.SplinePTP(&j2, tool, user, vel, acc, ovl);
    robot.SplinePTP(&j3, tool, user, vel, acc, ovl);
    robot.SplinePTP(&j4, tool, user, vel, acc, ovl);
    robot.SplineEnd();
    robot.CloseRPC();
    return 0;
}

int TestNewSpline(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
    JointPos j2(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
    JointPos j3(-61.954, -84.409, 108.153, -116.316, -91.283, 74.260);
    JointPos j4(-89.575, -80.276, 102.713, -116.302, -91.284, 74.267);
    JointPos j5(-95.228, -54.621, 73.691, -112.245, -91.280, 74.268);
    DescPose desc_pos1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
    DescPose desc_pos2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
    DescPose desc_pos3(-327.622, 402.230, 320.402, -178.067, 2.127, -46.207);
    DescPose desc_pos4(-104.066, 544.321, 327.023, -177.715, 3.371, -73.818);
    DescPose desc_pos5(-33.421, 732.572, 275.103, -177.907, 2.709, -79.482);
    DescPose offset_pos(0, 0, 0, 0, 0, 0);
    ExaxisPos epos(0, 0, 0, 0);
    int tool = 0;
    int user = 0;
    float vel = 100.0;
    float acc = 100.0;
    float ovl = 100.0;
    float blendT = -1.0;
    uint8_t flag = 0;
    robot.SetSpeed(20);
    int err1 = robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
    printf("movej errcode:%d\n", err1);
    robot.NewSplineStart(1, 2000);
    robot.NewSplinePoint(&j1, &desc_pos1, tool, user, vel, acc, ovl, -1, 0);
    robot.NewSplinePoint(&j2, &desc_pos2, tool, user, vel, acc, ovl, -1, 0);
    robot.NewSplinePoint(&j3, &desc_pos3, tool, user, vel, acc, ovl, -1, 0);
    robot.NewSplinePoint(&j4, &desc_pos4, tool, user, vel, acc, ovl, -1, 0);
    robot.NewSplinePoint(&j5, &desc_pos5, tool, user, vel, acc, ovl, -1, 0);
    robot.NewSplineEnd();
    err1 = robot.MoveJ(&j1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
    printf("movej errcode:%d\n", err1);
    robot.NewSplineStart(1, 2000);
    robot.NewSplinePoint(&desc_pos1, tool, user, vel, acc, ovl, -1, 0);
    robot.NewSplinePoint(&desc_pos2, tool, user, vel, acc, ovl, -1, 0);
    robot.NewSplinePoint(&desc_pos3, tool, user, vel, acc, ovl, -1, 0);
    robot.NewSplinePoint(&desc_pos4, tool, user, vel, acc, ovl, -1, 0);
    robot.NewSplinePoint(&desc_pos5, tool, user, vel, acc, ovl, -1, 0);
    robot.NewSplineEnd();
    robot.CloseRPC();
    return 0;
}

int TestPause(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
    JointPos j5(-95.228, -54.621, 73.691, -112.245, -91.280, 74.268);
    DescPose desc_pos1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
    DescPose desc_pos5(-33.421, 732.572, 275.103, -177.907, 2.709, -79.482);
    DescPose offset_pos(0, 0, 0, 0, 0, 0);
    ExaxisPos epos(0, 0, 0, 0);
    int tool = 0;
    int user = 0;
    float vel = 100.0;
    float acc = 100.0;
    float ovl = 100.0;
    float blendT = -1.0;
    uint8_t flag = 0;
    robot.SetSpeed(20);
    rtn = robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
    rtn = robot.MoveJ(&j5, &desc_pos5, tool, user, vel, acc, ovl, &epos, 1, flag, &offset_pos);
    robot.Sleep(1000);
    robot.PauseMotion();
    robot.Sleep(1000);
    robot.ResumeMotion();
    robot.Sleep(1000);
    robot.StopMotion();
    robot.Sleep(1000);
    robot.CloseRPC();
    return 0;
}

int TestOffset(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
    JointPos j2(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
    DescPose desc_pos1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
    DescPose desc_pos2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
    DescPose offset_pos(0, 0, 0, 0, 0, 0);
    DescPose offset_pos1(0, 0, 50, 0, 0, 0);
    ExaxisPos epos(0, 0, 0, 0);
    int tool = 0;
    int user = 0;
    float vel = 100.0;
    float acc = 100.0;
    float ovl = 100.0;
    float blendT = -1.0;
    uint8_t flag = 0;
    robot.SetSpeed(20);
    robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
    robot.MoveJ(&j2, &desc_pos2, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
    robot.Sleep(1000);
    robot.PointsOffsetEnable(0, &offset_pos1);
    robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
    robot.MoveJ(&j2, &desc_pos2, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
    robot.PointsOffsetDisable();
    robot.CloseRPC();
    return 0;
}

int TestMoveAO(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
    JointPos j2(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
    DescPose desc_pos1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
    DescPose desc_pos2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
    DescPose offset_pos(0, 0, 0, 0, 0, 0);
    DescPose offset_pos1(0, 0, 50, 0, 0, 0);
    ExaxisPos epos(0, 0, 0, 0);
    int tool = 0;
    int user = 0;
    float vel = 20.0;
    float acc = 20.0;
    float ovl = 100.0;
    float blendT = -1.0;
    uint8_t flag = 0;
    robot.SetSpeed(20);
    robot.MoveAOStart(0, 100, 100, 20);
    robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
    robot.MoveJ(&j2, &desc_pos2, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
    robot.MoveAOStop();
    robot.Sleep(1000);
    robot.MoveToolAOStart(0, 100, 100, 20);
    robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
    robot.MoveJ(&j2, &desc_pos2, tool, user, vel, acc, ovl, &epos, blendT, flag, &offset_pos);
    robot.MoveToolAOStop();
    robot.CloseRPC();
    return 0;
}

void TestIntersectLineMove()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return;
    }
    robot.SetReConnectParam(true, 30000, 500);
    DescPose mainPoint[6] = {};
    DescPose piecePoint[6] = {};
    ExaxisPos mainExaxisPos[6] = {};
    ExaxisPos pieceExaxisPos[6] = {};
    int extAxisFlag = 0;
    ExaxisPos exaxisPos[4] = {};
    DescPose offset = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };


    mainPoint[0] = { -411.572, -516.869, 197.724, -111.821, 31.353, -145.537 };
    mainPoint[1] = { -430.242, -575.160, 205.215, -107.763, 47.877, -141.814 };
    mainPoint[2] = { -443.560, -608.068, 180.211, -139.983, 78.547, -170.874 };
    mainPoint[3] = { -443.718, -608.250, 130.382, -155.397, 80.964, 173.955 };
    mainPoint[4] = { -436.198, -582.428, 100.045, 174.899, 72.468, 157.366 };
    mainPoint[5] = { -420.815, -527.510, 106.649, 123.128, 67.885, 110.539 };


    mainExaxisPos[0] = { 0.000, 0.000, 0.000, 0.000 };
    mainExaxisPos[1] = { 0.000, 0.000, 0.000, 0.000 };
    mainExaxisPos[2] = { 0.000, 0.000, 0.000, 0.000 };
    mainExaxisPos[3] = { 0.000, 0.000, 0.000, 0.000 };
    mainExaxisPos[4] = { 0.000, 0.000, 0.000, 0.000 };
    mainExaxisPos[5] = { 0.000, 0.000, 0.000, 0.000 };

    piecePoint[0] = {-341.600, -568.334, 327.186, 5.404, -3.657, -145.629};
    piecePoint[1] = {-319.224, -619.882, 330.833, 2.439, -3.294, -141.933};
    piecePoint[2] = {-278.636, -609.413, 329.042, 4.194, -7.682, -138.522};
    piecePoint[3] = {-270.948, -567.929, 326.010, 1.932, -4.908, -138.190};
    piecePoint[4] = {-291.152, -544.315, 324.130, -1.220, -5.373, -139.433};
    piecePoint[5] = {-316.419, -543.041, 324.621, 0.387, -5.188, -142.384};


    pieceExaxisPos[0] = { 0.000, -0.000, 0.000, 0.000 };
    pieceExaxisPos[1] = { 0.000, -0.000, 0.000, 0.000 };
    pieceExaxisPos[2] = { 0.000, -0.000, 0.000, 0.000 };
    pieceExaxisPos[3] = { 0.000, -0.000, 0.000, 0.000 };
    pieceExaxisPos[4] = { 0.000, -0.000, 0.000, 0.000 };
    pieceExaxisPos[5] = { 0.000, -0.000, 0.000, 0.000 };

    exaxisPos[0] = { 0.000, 0.000, 0.000, 0.000 };
    exaxisPos[1] = { 0.000, 0.000, 0.000, 0.000 };
    exaxisPos[2] = { 0.000, 0.000, 0.000, 0.000 };
    exaxisPos[3] = { 0.000, 0.000, 0.000, 0.000 };
    int tool = 2;
    int wobj = 0;
    double vel = 100.0;
    double acc = 100.0;
    double ovl = 12.0;
    double oacc = 12.0;
    int moveType = 0;
    int moveDirection = 0;
    rtn = robot.MoveToIntersectLineStart(mainPoint, mainExaxisPos, piecePoint, pieceExaxisPos, extAxisFlag, exaxisPos[0], tool, wobj, vel, acc, ovl, oacc, moveType, moveDirection, offset);
    printf("MoveToIntersectLineStart rtn is %d\n", rtn);
    rtn = robot.MoveIntersectLine(mainPoint, mainExaxisPos, piecePoint, pieceExaxisPos, extAxisFlag, exaxisPos, tool, wobj, vel, acc, 5.0, 5.0, moveDirection, offset);
    printf("MoveIntersectLine rtn is %d\n", rtn);
    robot.CloseRPC();
    return;
}

int TestFIR(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    JointPos startjointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
    JointPos midjointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
    JointPos endjointPos(-29.777, -84.536, 109.275, -114.075, -86.655, 74.257);
    DescPose startdescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
    DescPose middescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
    DescPose enddescPose(-487.434, 154.362, 308.576, 176.600, 0.268, -14.061);
    ExaxisPos exaxisPos(0, 0, 0, 0);
    DescPose offdese(0, 0, 0, 0, 0, 0);
    rtn = robot.PtpFIRPlanningStart(1000, 1000);
    cout << "PtpFIRPlanningStart rtn is " << rtn << endl;
    robot.MoveJ(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.MoveJ(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.PtpFIRPlanningEnd();
    cout << "PtpFIRPlanningEnd rtn is " << rtn << endl;
    robot.LinArcFIRPlanningStart(1000, 1000, 1000, 1000);
    cout << "LinArcFIRPlanningStart rtn is " << rtn << endl;
    robot.MoveL(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
    robot.MoveC(&midjointPos, &middescPose, 0, 0, 100, 100, &exaxisPos, 0, &offdese, &endjointPos, &enddescPose, 0, 0, 100, 100, &exaxisPos, 0, &offdese, 100, -1);
    robot.LinArcFIRPlanningEnd();
    cout << "LinArcFIRPlanningEnd rtn is " << rtn << endl;
    robot.CloseRPC();
    return 0;
}

int TestAccSmooth(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    JointPos startjointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
    JointPos endjointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
    DescPose startdescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
    DescPose enddescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
    ExaxisPos exaxisPos(0, 0, 0, 0);
    DescPose offdese(0, 0, 0, 0, 0, 0);
    rtn = robot.AccSmoothStart(0);
    cout << "AccSmoothStart rtn is " << rtn << endl;
    robot.MoveJ(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.MoveJ(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    rtn = robot.AccSmoothEnd(0);
    cout << "AccSmoothEnd rtn is " << rtn << endl;
    robot.CloseRPC();
    return 0;
}

int TestAngularSpeed(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    JointPos startjointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
    JointPos endjointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
    DescPose startdescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
    DescPose enddescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
    ExaxisPos exaxisPos(0, 0, 0, 0);
    DescPose offdese(0, 0, 0, 0, 0, 0);
    rtn = robot.AngularSpeedStart(50);
    cout << "AngularSpeedStart rtn is " << rtn << endl;
    robot.MoveJ(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.MoveJ(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    rtn = robot.AngularSpeedEnd();
    cout << "AngularSpeedEnd rtn is " << rtn << endl;
    robot.CloseRPC();
    return 0;
}

int TestSingularAvoid(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    JointPos startjointPos(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
    JointPos endjointPos(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
    DescPose startdescPose(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
    DescPose enddescPose(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
    ExaxisPos exaxisPos(0, 0, 0, 0);
    DescPose offdese(0, 0, 0, 0, 0, 0);
    rtn = robot.SingularAvoidStart(2, 10, 5, 5);
    cout << "SingularAvoidStart rtn is " << rtn << endl;
    robot.MoveJ(&startjointPos, &startdescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.MoveJ(&endjointPos, &enddescPose, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    rtn = robot.SingularAvoidEnd();
    cout << "SingularAvoidEnd rtn is " << rtn << endl;
    robot.CloseRPC();
    return 0;
}



int TestOriginPointWeave()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    JointPos j(39.886, -98.580, -124.032, -47.393, 90.000, 40.842);
    ExaxisPos epos(0, 0, 0, 0);
    DescPose offset_pos(0, 0, 0, 0, 0, 0);
    DescPose refPoint = { 425.381,214.211,104.811,179.997,-0.003,-90.956 };
    robot.MoveJ(&j, 1, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.OriginPointWeaveStart(0, 0, refPoint, 3);
    robot.MoveStationary();
    robot.OriginPointWeaveEnd();
    robot.Sleep(2000);
    robot.MoveJ(&j, 1, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.OriginPointWeaveStart(0, 1, refPoint, 3);
    robot.MoveStationary();
    robot.OriginPointWeaveEnd();
    robot.CloseRPC();
    robot.Sleep(1000);
    return 0;
}

int TestOriginPointWeaveWithLaser()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    JointPos j(39.886, -98.580, -124.032, -47.393, 90.000, 40.842);
    ExaxisPos epos1(0, 0, 0, 0);
    DescPose offset_pos(0, 0, 0, 0, 0, 0);
    ExaxisPos epos2(5, 0.000, 0.000, 0.000);
    DescPose refPoint(400.021, 300.022, 299.996, 179.997, -0.003, -90.956);
    robot.LaserTrackingSensorConfig("192.168.58.20", 5020);
    robot.LaserTrackingSensorSamplePeriod(20);
    robot.LoadPosSensorDriver(101);
    robot.ExtDevLoadUDPDriver();
    rtn = robot.SetExAxisCmdDoneTime(5000.0);
    printf("SetExAxisCmdDoneTime rtn is %d\n", rtn);
    rtn = robot.ExtAxisServoOn(1, 1);
    printf("ExtAxisServoOn axis id 1 rtn is %d\n", rtn);
    rtn = robot.ExtAxisServoOn(2, 1);
    printf("ExtAxisServoOn axis id 2 rtn is %d\n", rtn);
    robot.Sleep(2000);
    robot.ExtAxisSetHoming(1, 0, 10, 2);
    rtn = robot.LaserTrackingLaserOnOff(1, 0);
    printf("LaserTrackingLaserOnOff id 2 rtn is %d\n", rtn);
    robot.LaserTrackingTrackOnOff(1, 4);
    robot.Sleep(200);
    robot.OriginPointWeaveStart(0, 0, refPoint, 10);
    robot.MoveStationary();
    robot.OriginPointWeaveEnd();
    robot.LaserTrackingTrackOnOff(0, 4);

    robot.Sleep(2000);

    robot.ExtAxisMove(epos1, 100, -1);
    robot.LaserTrackingTrackOnOff(1, 4);
    robot.OriginPointWeaveStart(0, 0, refPoint, 20);
    robot.ExtAxisMove(epos2, 100, -1);
    robot.OriginPointWeaveEnd();
    robot.LaserTrackingTrackOnOff(0, 4);
    robot.CloseRPC();
    robot.Sleep(1000);
    return 0;
}

int ServoJVtest()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    robot.SetReConnectParam(true, 300000, 500);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    double joint_vel[6] = { 10.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double exis_vel[4]{ 0.0, 0.0, 0.0, 0.0 };
    float acc = 0.0f;
    float vel = 0.0f;
    float cmdT = 0.008f;
    float filterT = 0.0f;
    float gain = 0.0f;
    int cnt = 0;
    while (cnt < 200)
    {
        int error = robot.ServoJV(joint_vel, exis_vel, acc, vel, cmdT, filterT, gain);
        printf("ServoJV rtn is %d\n", error);
        cnt++;
    }
    robot.CloseRPC();
    robot.Sleep(1000000);
    return 0;
}

int ServoMITtest()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    robot.SetReConnectParam(true, 30000, 500);
    int rtn = robot.SetCmdRpyCallback(UDPFrameCallBack);
    printf("SetCmdRpyCallback rtn is %d\n", rtn);
    rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    while (true)
    {
        robot.ResetAllError();
        robot.Sleep(500);
        double posGain[6] = { 0.0 };
        double desPos[6] = { 0.0 };
        double velGain[6] = { 0.0 };
        double desVel[6] = { 0.0 };
        double torques[6] = { 0.0 };
        float curTorque[6] = { 0.0 };
        robot.GetJointTorques(1, curTorque);
        for (int i = 0; i < 6; i++)
        {
            torques[i] = curTorque[i];
        }
        robot.ServoMITStart(0);
        ROBOT_STATE_PKG pkg = {};
        robot.DragTeachSwitch(1);
        double intev = 0.008;
        double jPowerLimit[6] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
        double jVelLimit[6] = { 50, 50, 50, 50, 50, 50 };
        int error = 0;
        while (true)
        {
            torques[5] = 0.02;
            error = robot.ServoMIT(posGain, desPos, velGain, desVel, torques, intev, 0);
            robot.Sleep(1);
            robot.GetRobotRealTimeState(&pkg);
            printf("pkg.jt_cur_pos[5]: %f\n", pkg.jt_cur_pos[5]);
            if (pkg.jt_cur_pos[5] > 30)
            {
                break;
            }
        }
        while (true)
        {
            torques[5] = -0.02;
            error = robot.ServoMIT(posGain, desPos, velGain, desVel, torques, intev, 0);
            robot.Sleep(1);
            robot.GetRobotRealTimeState(&pkg);
            printf("pkg.jt_cur_pos[5]:%f\n", pkg.jt_cur_pos[5]);
            if (pkg.jt_cur_pos[5] < 0)
            {
                break;
            }
        }
        robot.DragTeachSwitch(0);
        error = robot.ServoMITEnd(0);
    }
    robot.CloseRPC();
    robot.Sleep(1000000);
    return 0;
}

int TestSplineWeaveBackCenter()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    robot.SetReConnectParam(true, 30000, 500);
    int rtn = robot.SetCmdRpyCallback(UDPFrameCallBack);
    printf("SetCmdRpyCallback rtn is %d\n", rtn);
    rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }

    JointPos j1(9.000, -66.067, 67.706, -103.217, -90.151, 100.669);
    JointPos j2(-4.660, -107.973, 103.734, -76.214, -89.999, 90.886);
    JointPos j3(-36.762, -77.380, 91.364, -127.159, -90.024, 54.833);
    JointPos j4(-62.875, -89.460, 86.437, -77.030, -90.012, 31.539);
    DescPose desc_pos1(-654.129, -235.344, 246.543, 6.010, -11.535, -176.787);
    DescPose desc_pos2(-273.710, -100.871, 280.935, 5.692, 9.522, 179.512);
    DescPose desc_pos3(-566.093, 311.278, 215.008, -10.453, -17.486, -174.209);
    DescPose desc_pos4(-246.558, 328.240, 292.173, 13.912, 4.437, -179.067);
    DescPose offset_pos(0, 0, 0, 0, 0, 0);
    ExaxisPos epos(0, 0, 0, 0);
    int tool = 2;
    int user = 0;
    float vel = 100.0;
    float acc = 100.0;
    float ovl = 20;
    float oacc = 100.0;
    float blendT = 0.0;
    float blendR = 0.0;
    uint8_t flag = 0;
    uint8_t search = 0;
    int blendMode = 0;
    int velAccMode = 0;
    robot.SetSpeed(1);

    robot.SetWeaveBackCenterConfig(1);
    int weaveBackConfig = 0;
    robot.GetWeaveBackCenterConfig(weaveBackConfig);
    printf("GetWeavebackCenterConfig  %d: \n", weaveBackConfig);

    rtn = robot.MoveJ(&j1, &desc_pos1, tool, user, vel, acc, 100.0, &epos, blendT, flag, &offset_pos);

    robot.WeaveStart(0);
    robot.NewSplineStart(0, 6000);
    robot.NewSplinePoint(&j1, &desc_pos1, tool, user, vel, acc, ovl, -1, 0);
    robot.NewSplinePoint(&j2, &desc_pos2, tool, user, vel, acc, ovl, -1, 0);
    robot.NewSplinePoint(&j3, &desc_pos3, tool, user, vel, acc, ovl, -1, 0);
    robot.NewSplinePoint(&j4, &desc_pos4, tool, user, vel, acc, ovl, -1, 1);
    robot.NewSplineEnd();
}

//ty = 0
//MoveJ(, , 2, 0, 100, 100, 100, 0.000, 0.000, 0.000, 0.000, -1, 0, 0, 0, 0, 0, 0, 0)
//WeaveStart(ty)
//NewSplineStart(0, 6000)
//NewSplinePoint(9.000, -66.067, 67.706, -103.217, -90.151, 100.669, -654.129, -235.344, 246.543, 6.010, -11.535, -176.787, 2, 0, 100, 100, 100, 20, 0)
//NewSplinePoint(, , 2, 0, 100, 100, 100, 20, 0)
//NewSplinePoint(, , 2, 0, 100, 100, 100, 20, 0)
//NewSplinePoint(,, 2, 0, 100, 100, 100, 20, 1)
//NewSplineEnd()
//WeaveEnd(ty)

int TestLaserStationary(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return 0;
    }
    robot.SetReConnectParam(true, 30000, 500);
    rtn = robot.LaserSensorRecordandReplay(0, 10, 1, 0, 0.1, 1, 0, 100, 100);
    printf("LaserSensorRecordandReplay rtn is %d\n", rtn);
    rtn = robot.MoveStationary();
    printf("MoveStationary rtn is %d\n", rtn);
    rtn = robot.LaserSensorRecord1(0, 10);
    printf("LaserSensorRecordandReplay rtn is %d\n", rtn);
    robot.CloseRPC();
    robot.Sleep(9999999);
    return 0;
}


/**
 * 摆动过程中调速和实时偏移测试
 * WeaveStart -> MoveL -> SetSpeed调速 -> SetWeaveOffsetRT偏移(50次) -> WeaveEnd
 */
int TestWeaveSpeedAndOffset()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    robot.SetReConnectParam(true, 30000, 500);
    int rtn = robot.RPC("192.168.58.2");

    if (rtn != 0)
    {
        return 0;
    }
    
    std::cout << "============================================================" << std::endl;
    std::cout << "  摆动调速与实时偏移测试" << std::endl;
    std::cout << "============================================================" << std::endl;

    ExaxisPos epos(0, 0, 0, 0);
    DescPose offset_pos(0, 0, 0, 0, 0, 0);

    JointPos j1(5.027, -84.331, -75.139, -103.690, 86.379, 20.794);
    DescPose d1(324.752, -83.339, 366.314, -172.321, -0.936, -106.047);

    JointPos j2(-35.335, -117.598, -57.174, -95.234, 90.001, -19.560);
    DescPose d2(324.999, -355.439, 260.000, 179.995, 0.003, -105.775);

    JointPos j3(59.787, -117.594, -57.183, -95.222, 90.006, 75.562);
    DescPose d3(324.998, 355.441, 260.002, 179.995, 0.003, -105.775);

    // ---- Step 1: MoveJ到起始点 ----
    std::cout << "\nStep 1: MoveJ to start point" << std::endl;
    rtn = robot.MoveJ(&j1, &d1, 1, 0, 100, 100, 50, &epos, -1, 0, &offset_pos);
    std::cout << "  MoveJ(j1) rtn=" << rtn << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // ---- Step 2: MoveJ到摆动入口 ----
    std::cout << "\nStep 2: MoveJ to weave entry point" << std::endl;
    rtn = robot.MoveJ(&j2, &d2, 1, 0, 100, 100, 50, &epos, -1, 0, &offset_pos);
    std::cout << "  MoveJ(j2) rtn=" << rtn << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // ---- Step 3: WeaveStart, 启动摆动MoveL线程 ----
    std::cout << "\nStep 3: WeaveStart + MoveL in background thread" << std::endl;
    robot.WeaveStart(0);

    std::atomic<bool> weaveRunning(true);
    std::thread weaveThread([&]() {
        rtn = robot.MoveL(&j3, &d3, 1, 0, 100, 100, 5, -1, 0, &epos, 0, 0, &offset_pos, 5, 0, 0, 10);
        std::cout << "  MoveL(weave) thread finished, rtn=" << rtn << std::endl;
        weaveRunning = false;
        });
    weaveThread.detach();  // 后台运行
    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // 等待运动开始

    // ---- Step 4: 调速测试(主线程，摆动MoveL后台运行) ----
    std::cout << "\nStep 4: SetSpeed test during weaving" << std::endl;
    std::vector<int> speedValues = { 20, 50, 80, 30, 60, 10 };
    for (int speed : speedValues)
    {
        if (!weaveRunning.load()) break;
        rtn = robot.SetSpeedInstant(speed);
        robot.GetRobotRealTimeState(&pkg);
        std::cout << "  SetSpeed(" << speed << ") -> rtn=" << rtn
            << ", TCP_CmpSpeed=" << pkg.target_TCP_CmpSpeed << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    // ---- Step 5: SetWeaveOffsetRT偏移测试(主线程，摆动MoveL后台运行) ----
    std::cout << "\nStep 5: SetWeaveOffsetRT test (50 iterations, delta=0.1)" << std::endl;
    double accumOffset = 0.0;
    for (int i = 0; i < 50 && weaveRunning.load(); i++)
    {
        accumOffset += 1;
        DescPose weaveOffset(0, 0, accumOffset, 0, 0, 0);
        rtn = robot.SetWeaveOffsetRT(weaveOffset);
        robot.GetRobotRealTimeState(&pkg);
        std::cout << "  [" << (i + 1) << "/50] SetWeaveOffsetRT(x=" << accumOffset << ") -> rtn=" << rtn
            << ", TCP_pos=(" << pkg.tl_cur_pos[0] << "," << pkg.tl_cur_pos[1] << "," << pkg.tl_cur_pos[2] << ")"
            << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    // ---- Step 6: 等待摆动MoveL完成, WeaveEnd ----
    std::cout << "\nStep 6: Wait for weave MoveL, then WeaveEnd" << std::endl;
    // 由于使用了 detach，这里需要等待 weaveRunning 变为 false
    while (weaveRunning.load()) 
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    robot.WeaveEnd(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // ---- Step 7: MoveL返回起始点 ----
    std::cout << "\nStep 7: MoveL back to start" << std::endl;
    rtn = robot.MoveL(&j1, &d1, 1, 0, 100, 100, 50, -1, 0, &epos, 0, 0, &offset_pos, 50, 0, 0, 10);
    std::cout << "  MoveL(back) rtn=" << rtn << std::endl;

    robot.GetRobotRealTimeState(&pkg);
    std::cout << "\n  Final robot state: main_code=" << pkg.main_code
        << ", sub_code=" << pkg.sub_code << std::endl;
    std::cout << "============================================================" << std::endl;
    std::cout << "  摆动调速与实时偏移测试 完成" << std::endl;
    std::cout << "============================================================" << std::endl;
}

int TestWorkPieceTrsf()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    robot.SetReConnectParam(true, 30000, 500);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return 0;
    }

    JointPos j1(-11.188, -64.165, -107.299, -76.706, 89.590, 92.983);
    DescPose desc1(225.986, 190.694, 394.238, -6.230, -23.797, -98.972);
    JointPos j2(-38.148, -97.408, -133.704, -30.999, 89.584, 92.986);
    DescPose desc2(52.741, 262.917, 30.824, -5.696, -9.864, -126.092);
    JointPos j3(-25.561, -123.131, -85.736, -94.911, 89.582, 93.006);
    DescPose desc3(70.455, 88.410, 45.299, -4.101, 31.775, -113.199);
    JointPos j4(-8.013, -125.881, -79.196, -84.440, 89.564, 93.005);
    DescPose desc4(209.453, -73.895, 56.416, -4.727, 17.523, -95.906);
    JointPos j5(-2.722, -94.518, -119.965, -54.518, 89.563, 93.005);
    DescPose desc5(274.800, 81.106, 102.977, -5.467, -2.980, -90.711);
    JointPos j6(-2.671, -56.234, -138.914, -25.099, 95.355, 92.967);
    DescPose desc6(300.392, 177.281, 300.926, -1.909, -51.894, -89.703);
    JointPos j7(-1.229, -121.184, -63.201, -122.331, 93.045, 93.019);
    DescPose desc7(296.856, -31.294, 215.698, -0.589, 34.594, -88.954);

    ExaxisPos exaxis = {0.0, 0.0, 0.0, 0.0};
    DescPose offset(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    int tool = 1;
    int workpiece = 1;
    double blend = 5.0;

    robot.MoveJ(&j1, &desc1, tool, workpiece, 100, 100, 100, &exaxis, -1, 0, &offset);
    robot.MoveJ(&j2, &desc2, tool, workpiece, 100, 100, 100, &exaxis, blend, 0, &offset);
    robot.MoveL(&j3, &desc3, tool, workpiece, 10, 100, 100, blend, 0, &exaxis, 0, 1, &offset);
    robot.MoveC(&j4, &desc4, tool, workpiece, 100, 100, &exaxis, 0, &offset, &j5, &desc5, tool, workpiece, 100, 100, &exaxis, 0, &offset, 10, blend);
    robot.Circle(&j6, &desc6, tool, workpiece, 100, 100, &exaxis, &j7, &desc7, tool, workpiece, 100, 100, &exaxis, 10, 0, &offset, 100.0, blend);
    
    rtn = robot.WorkPieceTrsfStart(2);
    printf("WorkPieceTrsfStart rtn is %d\n", rtn);
    robot.MoveJ(&j1, &desc1, tool, workpiece, 100, 100, 100, &exaxis, -1, 0, &offset);
    robot.MoveJ(&j2, &desc2, tool, workpiece, 100, 100, 100, &exaxis, blend, 0, &offset);
    robot.MoveL(&j3, &desc3, tool, workpiece, 10, 100, 100, blend, 0, &exaxis, 0, 1, &offset);
    robot.MoveC(&j4, &desc4, tool, workpiece, 100, 100, &exaxis, 0, &offset, &j5, &desc5, tool, workpiece, 100, 100, &exaxis, 0, &offset, 10, blend);
    robot.Circle(&j6, &desc6, tool, workpiece, 100, 100, &exaxis, &j7, &desc7, tool, workpiece, 100, 100, &exaxis, 10, 0, &offset, 100.0, blend);    
    rtn = robot.WorkPieceTrsfEnd();
    printf("WorkPieceTrsfEnd rtn is %d\n", rtn);
    robot.CloseRPC();
    robot.Sleep(2000);
}

#pragma endregion
#pragma region 机器人IO

int TestAODO(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    uint8_t status = 1;
    uint8_t smooth = 0;
    uint8_t block = 0;
    for (int i = 0; i < 16; i++)
    {
        robot.SetDO(i, status, smooth, block);
        robot.Sleep(300);
    }
    status = 0;
    for (int i = 0; i < 16; i++)
    {
        robot.SetDO(i, status, smooth, block);
        robot.Sleep(300);
    }
    status = 1;
    for (int i = 0; i < 2; i++)
    {
        robot.SetToolDO(i, status, smooth, block);
        robot.Sleep(1000);
    }
    status = 0;
    for (int i = 0; i < 2; i++)
    {
        robot.SetToolDO(i, status, smooth, block);
        robot.Sleep(1000);
    }
    for (int i = 0; i < 100; i++)
    {
        robot.SetAO(0, i * 40.96, block);
        robot.Sleep(30);
    }
    for (int i = 0; i < 100; i++)
    {
        robot.SetToolAO(0, i * 40.96, block);
        robot.Sleep(30);
    }
    robot.CloseRPC();
    return 0;
}

int TestGetDIAI(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    uint8_t status = 1;
    uint8_t smooth = 0;
    uint8_t block = 0;
    uint8_t di = 0, tool_di = 0;
    float ai = 0.0, tool_ai = 0.0;
    float value = 0.0;
    robot.GetDI(0, block, &di);
    printf("di0:%u\n", di);
    tool_di = robot.GetToolDI(1, block, &tool_di);
    printf("tool_di1:%u\n", tool_di);
    robot.GetAI(0, block, &ai);
    printf("ai0:%f\n", ai);
    tool_ai = robot.GetToolAI(0, block, &tool_ai);
    printf("tool_ai0:%f\n", tool_ai);
    uint8_t _button_state = 0;
    robot.GetAxlePointRecordBtnState(&_button_state);
    printf("_button_state is: %u\n", _button_state);
    uint8_t tool_do_state = 0;
    robot.GetToolDO(&tool_do_state);
    printf("tool DO state is: %u\n", tool_do_state);
    uint8_t do_state_h = 0;
    uint8_t do_state_l = 0;
    robot.GetDO(&do_state_h, &do_state_l);
    printf("DO state high is: %u \n DO state low is: %u\n", do_state_h, do_state_l);
    robot.CloseRPC();
    return 0;
}

int TestWaitDIAI(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    uint8_t status = 1;
    uint8_t smooth = 0;
    uint8_t block = 0;
    uint8_t di = 0, tool_di = 0;
    float ai = 0.0, tool_ai = 0.0;
    float value = 0.0;
    rtn = robot.WaitDI(0, 1, 1000, 0);
    cout << "WaitDI over; rtn is: " << rtn << endl;
    robot.WaitMultiDI(1, 3, 3, 1000, 1);
    cout << "WaitMultiDI over; rtn is: " << rtn << endl;
    robot.WaitToolDI(0, 1, 1000, 1);
    cout << "WaitToolDI over; rtn is: " << rtn << endl;
    robot.WaitAI(0, 0, 50, 1000, 1);
    cout << "WaitAI over; rtn is: " << rtn << endl;
    robot.WaitToolAI(0, 0, 50, 1000, 1);
    cout << "WaitToolAI over; rtn is: " << rtn << endl;
    robot.CloseRPC();
    return 0;
}

int TestDOReset(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(3);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    for (int i = 0; i < 16; i++)
    {
        robot.SetDO(i, 1, 0, 0);
        robot.Sleep(200);
    }
    int resetFlag = 0;
    int resumeReloadFlag = 0;
    rtn = robot.SetOutputResetCtlBoxDO(resetFlag, resumeReloadFlag);
    robot.SetOutputResetCtlBoxAO(resetFlag, resumeReloadFlag);
    robot.SetOutputResetAxleDO(resetFlag, resumeReloadFlag);
    robot.SetOutputResetAxleAO(resetFlag, resumeReloadFlag);
    robot.SetOutputResetExtDO(resetFlag, resumeReloadFlag);
    robot.SetOutputResetExtAO(resetFlag, resumeReloadFlag);
    robot.SetOutputResetSmartToolDO(resetFlag, resumeReloadFlag);
    robot.ProgramLoad("test.lua");
    robot.ProgramRun();
    robot.Sleep(2000);
    robot.PauseMotion();
    robot.Sleep(2000);
    robot.ResumeMotion();
    robot.Sleep(2000);
    robot.CloseRPC();
    return 0;
}

int TestIOConfig()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    int setDIConfig[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
    int getDIConfig[8] = { 0 };
    rtn = robot.SetDIConfig(setDIConfig);
    printf("SetDIConfig rtn is %d\n", rtn);
    rtn = robot.GetDIConfig(getDIConfig);
    printf("GetDIConfig rtn is %d, value is %d %d %d %d %d %d %d %d \n", rtn,
        getDIConfig[0], getDIConfig[1], getDIConfig[2], getDIConfig[3], getDIConfig[4], getDIConfig[5], getDIConfig[6], getDIConfig[7]);
    int setDOConfig[8] = { 9, 10, 11, 12, 13, 14, 15, 16 };
    int getDOConfig[8] = { 0 };
    rtn = robot.SetDOConfig(setDOConfig);
    printf("SetDOConfig rtn is %d\n", rtn);
    rtn = robot.GetDOConfig(getDOConfig);
    printf("GetDOConfig rtn is %d, value is %d %d %d %d %d %d %d %d \n", rtn,
        getDOConfig[0], getDOConfig[1], getDOConfig[2], getDOConfig[3], getDOConfig[4], getDOConfig[5], getDOConfig[6], getDOConfig[7]);
    int setToolDIConfig[2] = { 17, 18 };
    int getToolDIConfig[2] = { 0 };
    rtn = robot.SetToolDIConfig(setToolDIConfig);
    printf("SetToolDIConfig rtn is %d\n", rtn);
    rtn = robot.GetToolDIConfig(getToolDIConfig);
    printf("GetToolDIConfig rtn is %d, value is %d %d \n", rtn, getToolDIConfig[0], getToolDIConfig[1]);
    int setDIConfigLevel[8] = { 1, 1, 1, 1, 0, 0, 0, 0 };
    int getDIConfigLevel[8] = { 0 };
    rtn = robot.SetDIConfigLevel(setDIConfigLevel);
    printf("SetDIConfigLevel rtn is %d\n", rtn);
    rtn = robot.GetDIConfigLevel(getDIConfigLevel);
    printf("GetDIConfigLevel rtn is %d, value is %d %d %d %d %d %d %d %d \n", rtn,
        getDIConfigLevel[0], getDIConfigLevel[1], getDIConfigLevel[2], getDIConfigLevel[3], getDIConfigLevel[4], getDIConfigLevel[5], getDIConfigLevel[6], getDIConfigLevel[7]);
    int setDOConfigLevel[8] = { 0, 0, 0, 0, 1, 1, 1, 1 };
    int getDOConfigLevel[8] = { 0 };
    rtn = robot.SetDOConfigLevel(setDOConfigLevel);
    printf("SetDOConfigLevel rtn is %d\n", rtn);
    rtn = robot.GetDOConfigLevel(getDOConfigLevel);
    printf("GetDOConfigLevel rtn is %d, value is %d %d %d %d %d %d %d %d \n", rtn,
        getDOConfigLevel[0], getDOConfigLevel[1], getDOConfigLevel[2], getDOConfigLevel[3], getDOConfigLevel[4], getDOConfigLevel[5], getDOConfigLevel[6], getDOConfigLevel[7]);
    int setToolDIConfigLevel[2] = { 1, 0 };
    int getToolDIConfigLevel[2] = { 0 };
    rtn = robot.SetToolDIConfigLevel(setToolDIConfigLevel);
    printf("SetToolDIConfigLevel rtn is %d\n", rtn);
    rtn = robot.GetToolDIConfigLevel(getToolDIConfigLevel);
    printf("GetToolDIConfigLevel rtn is %d, value is %d %d \n", rtn, getToolDIConfigLevel[0], getToolDIConfigLevel[1]);
    int setStandardDILevel[8] = { 1, 1, 1, 1, 0, 0, 0, 0 };
    int getStandardDILevel[8] = { 0 };
    rtn = robot.SetStandardDILevel(setStandardDILevel);
    printf("SetStandardDILevel rtn is %d\n", rtn);
    rtn = robot.GetStandardDILevel(getStandardDILevel);
    printf("GetStandardDILevel rtn is %d, value is %d %d %d %d %d %d %d %d \n", rtn,
        getStandardDILevel[0], getStandardDILevel[1], getStandardDILevel[2], getStandardDILevel[3], getStandardDILevel[4], getStandardDILevel[5], getStandardDILevel[6], getStandardDILevel[7]);
    int setStandardDOLevel[8] = { 0, 0, 0, 0, 1, 1, 1, 1 };
    int getStandardDOLevel[8] = { 0 };
    rtn = robot.SetStandardDOLevel(setStandardDOLevel);
    printf("SetStandardDOLevel rtn is %d\n", rtn);
    rtn = robot.GetStandardDOLevel(getStandardDOLevel);
    printf("GetStandsrdDOLevel rtn is %d, value is %d %d %d %d %d %d %d %d \n", rtn,
        getStandardDOLevel[0], getStandardDOLevel[1], getStandardDOLevel[2], getStandardDOLevel[3], getStandardDOLevel[4], getStandardDOLevel[5], getStandardDOLevel[6], getStandardDOLevel[7]);
    robot.Sleep(2000);
    robot.CloseRPC();
    robot.Sleep(1000);
    return 0;
}

int TestSafetyIOConfig()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    robot.SetReConnectParam(true, 30000, 500);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    
    int getDIConfig[8] = { 0 };
    rtn = robot.SetSafetyDIConfig(0, 201);
    printf("SetSafetyDIConfig rtn is %d\n", rtn);
    rtn = robot.SetSafetyDIConfig(1, 202);
    printf("SetSafetyDIConfig rtn is %d\n", rtn);
    rtn = robot.SetSafetyDIConfig(2, 203);
    printf("SetSafetyDIConfig rtn is %d\n", rtn);
    rtn = robot.SetSafetyDIConfig(3, 204);
    printf("SetSafetyDIConfig rtn is %d\n", rtn);
    rtn = robot.GetDIConfig(getDIConfig);
    printf("GetDIConfig rtn is %d, value is %d %d %d %d %d %d %d %d \n", rtn,
        getDIConfig[0], getDIConfig[1], getDIConfig[2], getDIConfig[3], getDIConfig[4], getDIConfig[5], getDIConfig[6], getDIConfig[7]);

    rtn = robot.SetSafetyDIConfig(0, 0);
    printf("SetSafetyDIConfig rtn is %d\n", rtn);
    rtn = robot.SetSafetyDIConfig(1, 0);
    printf("SetSafetyDIConfig rtn is %d\n", rtn);
    rtn = robot.SetSafetyDIConfig(2, 0);
    printf("SetSafetyDIConfig rtn is %d\n", rtn);
    rtn = robot.SetSafetyDIConfig(3, 0);
    printf("SetSafetyDIConfig rtn is %d\n", rtn);
    rtn = robot.GetDIConfig(getDIConfig);
    printf("GetDIConfig rtn is %d, value is %d %d %d %d %d %d %d %d \n", rtn,
        getDIConfig[0], getDIConfig[1], getDIConfig[2], getDIConfig[3], getDIConfig[4], getDIConfig[5], getDIConfig[6], getDIConfig[7]);


    int getDOConfig[8] = { 0 };
    rtn = robot.SetSafetyDOConfig(0, 204);
    printf("SetSafetyDOConfig rtn is %d\n", rtn);
    rtn = robot.SetSafetyDOConfig(1, 205);
    printf("SetSafetyDOConfig rtn is %d\n", rtn);
    rtn = robot.SetSafetyDOConfig(2, 206);
    printf("SetSafetyDOConfig rtn is %d\n", rtn);
    rtn = robot.SetSafetyDOConfig(3, 207);
    printf("SetSafetyDOConfig rtn is %d\n", rtn);
    rtn = robot.GetDOConfig(getDOConfig);
    printf("GetDOConfig rtn is %d, value is %d %d %d %d %d %d %d %d \n", rtn,
        getDOConfig[0], getDOConfig[1], getDOConfig[2], getDOConfig[3], getDOConfig[4], getDOConfig[5], getDOConfig[6], getDOConfig[7]);
   
    rtn = robot.SetSafetyDOConfig(0, 0);
    printf("SetSafetyDOConfig rtn is %d\n", rtn);
    rtn = robot.SetSafetyDOConfig(1, 0);
    printf("SetSafetyDOConfig rtn is %d\n", rtn);
    rtn = robot.SetSafetyDOConfig(2, 0);
    printf("SetSafetyDOConfig rtn is %d\n", rtn);
    rtn = robot.SetSafetyDOConfig(3, 0);
    printf("SetSafetyDOConfig rtn is %d\n", rtn);
    rtn = robot.GetDOConfig(getDOConfig);
    printf("GetDOConfig rtn is %d, value is %d %d %d %d %d %d %d %d \n", rtn,
        getDOConfig[0], getDOConfig[1], getDOConfig[2], getDOConfig[3], getDOConfig[4], getDOConfig[5], getDOConfig[6], getDOConfig[7]);

    robot.Sleep(2000);
    robot.CloseRPC();
    robot.Sleep(1000);
    return 0;
}
#pragma endregion
#pragma region 机器人常用设置

int TestExtCoord(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    DescPose p1Desc(-89.606, 779.517, 193.516, 178.000, 0.476, -92.484);
    JointPos p1Joint(-108.145, -50.137, 85.818, -125.599, -87.946, 74.329);
    DescPose p2Desc(-24.656, 850.384, 191.361, 177.079, -2.058, -95.355);
    JointPos p2Joint(-111.024, -41.538, 69.222, -114.913, -87.743, 74.329);
    DescPose p3Desc(-99.813, 766.661, 241.878, -176.817, 1.917, -91.604);
    JointPos p3Joint(-107.266, -56.116, 85.971, -122.560, -92.548, 74.331);
    ExaxisPos exaxisPos(0, 0, 0, 0);
    DescPose offdese(0, 0, 0, 0, 0, 0);
    DescPose posTCP[3] = { p1Desc , p2Desc , p3Desc };
    DescPose coordRtn = {};
    robot.MoveJ(&p1Joint, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.SetExTCPPoint(1);
    robot.MoveJ(&p2Joint, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.SetExTCPPoint(2);
    robot.MoveJ(&p3Joint, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.SetExTCPPoint(3);
    rtn = robot.ComputeExTCF(&coordRtn);
    printf("ComputeExTCF          %d coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);
    robot.SetExToolCoord(21, &coordRtn, &offdese);
    robot.SetExToolList(21, &coordRtn, &offdese);
    robot.CloseRPC();
    return 0;
}

int TestTCPCompute(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    DescPose p1Desc(186.331, 487.913, 209.850, 149.030, 0.688, -114.347);
    JointPos p1Joint(-127.876, -75.341, 115.417, -122.741, -59.820, 74.300);
    DescPose p2Desc(69.721, 535.073, 202.882, -144.406, -14.775, -89.012);
    JointPos p2Joint(-101.780, -69.828, 110.917, -125.740, -127.841, 74.300);
    DescPose p3Desc(146.861, 578.426, 205.598, 175.997, -36.178, -93.437);
    JointPos p3Joint(-112.851, -60.191, 86.566, -80.676, -97.463, 74.300);
    DescPose p4Desc(136.284, 509.876, 225.613, 178.987, 1.372, -100.696);
    JointPos p4Joint(-116.397, -76.281, 113.845, -128.611, -88.654, 74.299);
    DescPose p5Desc(138.395, 505.972, 298.016, 179.134, 2.147, -101.110);
    JointPos p5Joint(-116.814, -82.333, 109.162, -118.662, -88.585, 74.302);
    DescPose p6Desc(105.553, 454.325, 232.017, -179.426, 0.444, -99.952);
    JointPos p6Joint(-115.649, -84.367, 122.447, -128.663, -90.432, 74.303);
    ExaxisPos exaxisPos(0, 0, 0, 0);
    DescPose offdese(0, 0, 0, 0, 0, 0);
    JointPos posJ[6] = { p1Joint , p2Joint , p3Joint , p4Joint , p5Joint , p6Joint };
    DescPose coordRtn = {};
    rtn = robot.ComputeToolCoordWithPoints(1, posJ, coordRtn);
    printf("ComputeToolCoordWithPoints    %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);
    robot.MoveJ(&p1Joint, &p1Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.SetToolPoint(1);
    robot.MoveJ(&p2Joint, &p2Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.SetToolPoint(2);
    robot.MoveJ(&p3Joint, &p3Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.SetToolPoint(3);
    robot.MoveJ(&p4Joint, &p4Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.SetToolPoint(4);
    robot.MoveJ(&p5Joint, &p5Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.SetToolPoint(5);
    robot.MoveJ(&p6Joint, &p6Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.SetToolPoint(6);
    rtn = robot.ComputeTool(&coordRtn);
    printf("6 Point ComputeTool        %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);
    robot.SetToolList(1, &coordRtn, 0, 0, 0);
    robot.MoveJ(&p1Joint, &p1Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.SetTcp4RefPoint(1);
    robot.MoveJ(&p2Joint, &p2Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.SetTcp4RefPoint(2);
    robot.MoveJ(&p3Joint, &p3Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.SetTcp4RefPoint(3);
    robot.MoveJ(&p4Joint, &p4Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.SetTcp4RefPoint(4);
    rtn = robot.ComputeTcp4(&coordRtn);
    printf("4 Point ComputeTool        %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);
    robot.SetToolCoord(2, &coordRtn, 0, 0, 1, 0);
    DescPose getCoord = {};
    rtn = robot.GetTCPOffset(0, &getCoord);
    printf("GetTCPOffset    %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);
    robot.CloseRPC();
    return 0;
}

int TestWobjCoord(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    DescPose p1Desc(-89.606, 779.517, 193.516, 178.000, 0.476, -92.484);
    JointPos p1Joint(-108.145, -50.137, 85.818, -125.599, -87.946, 74.329);
    DescPose p2Desc(-24.656, 850.384, 191.361, 177.079, -2.058, -95.355);
    JointPos p2Joint(-111.024, -41.538, 69.222, -114.913, -87.743, 74.329);
    DescPose p3Desc(-99.813, 766.661, 241.878, -176.817, 1.917, -91.604);
    JointPos p3Joint(-107.266, -56.116, 85.971, -122.560, -92.548, 74.331);
    ExaxisPos exaxisPos(0, 0, 0, 0);
    DescPose offdese(0, 0, 0, 0, 0, 0);
    DescPose posTCP[3] = { p1Desc , p2Desc , p3Desc };
    DescPose coordRtn = {};
    rtn = robot.ComputeWObjCoordWithPoints(1, posTCP, 0, coordRtn);
    printf("ComputeWObjCoordWithPoints    %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);
    robot.MoveJ(&p1Joint, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.SetWObjCoordPoint(1);
    robot.MoveJ(&p2Joint, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.SetWObjCoordPoint(2);
    robot.MoveJ(&p3Joint, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.SetWObjCoordPoint(3);
    rtn = robot.ComputeWObjCoord(1, 0, &coordRtn);
    printf("ComputeWObjCoord                   %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);
    robot.SetWObjCoord(1, &coordRtn, 0);
    robot.SetWObjList(1, &coordRtn, 0);
    DescPose getWobjDesc = {};
    rtn = robot.GetWObjOffset(0, &getWobjDesc);
    printf("GetWObjOffset                   %d  coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);
    robot.CloseRPC();
    return 0;
}


int TestLoadInstall(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    for (int i = 1; i < 100; i++)
    {
        robot.SetSpeed(i);
        robot.SetOaccScale(i);
        robot.Sleep(30);
    }
    float defaultVel = 0.0;
    robot.GetDefaultTransVel(&defaultVel);
    printf("GetDefaultTransVel is %f\n", defaultVel);
    for (int i = 1; i < 21; i++)
    {
        robot.SetSysVarValue(i, i + 0.5);
        robot.Sleep(100);
    }
    for (int i = 1; i < 21; i++)
    {
        float value = 0;
        robot.GetSysVarValue(i, &value);
        printf("sys value  %d is :%f\n", i, value);
        robot.Sleep(100);
    }
    robot.SetLoadWeight(0, 2.5);
    DescTran loadCoord = {};
    loadCoord.x = 3.0;
    loadCoord.y = 4.0;
    loadCoord.z = 5.0;
    robot.SetLoadCoord(&loadCoord);
    robot.Sleep(1000);
    float getLoad = 0.0;
    robot.GetTargetPayload(0, &getLoad);
    DescTran getLoadTran = {};
    robot.GetTargetPayloadCog(0, &getLoadTran);
    printf("get load is %f; get load cog is %f %f %f\n", getLoad, getLoadTran.x, getLoadTran.y, getLoadTran.z);
    robot.SetRobotInstallPos(0);
    robot.SetRobotInstallAngle(15.0, 25.0);
    float anglex = 0.0;
    float angley = 0.0;
    robot.GetRobotInstallAngle(&anglex, &angley);
    printf("GetRobotInstallAngle x:  %f;  y:  %f\n", anglex, angley);
    robot.CloseRPC();
    return 0;
}

int TestFocus()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    DescPose p1Desc(186.331, 487.913, 209.850, 149.030, 0.688, -114.347);
    JointPos p1Joint(-127.876, -75.341, 115.417, -122.741, -59.820, 74.300);
    DescPose p2Desc(69.721, 535.073, 202.882, -144.406, -14.775, -89.012);
    JointPos p2Joint(-101.780, -69.828, 110.917, -125.740, -127.841, 74.300);
    DescPose p3Desc(146.861, 578.426, 205.598, 175.997, -36.178, -93.437);
    JointPos p3Joint(-112.851, -60.191, 86.566, -80.676, -97.463, 74.300);
    DescPose p4Desc(136.284, 509.876, 225.613, 178.987, 1.372, -100.696);
    JointPos p4Joint(-116.397, -76.281, 113.845, -128.611, -88.654, 74.299);
    DescPose p5Desc(138.395, 505.972, 298.016, 179.134, 2.147, -101.110);
    JointPos p5Joint(-116.814, -82.333, 109.162, -118.662, -88.585, 74.302);
    DescPose p6Desc(105.553, 454.325, 232.017, -179.426, 0.444, -99.952);
    JointPos p6Joint(-115.649, -84.367, 122.447, -128.663, -90.432, 74.303);
    ExaxisPos exaxisPos(0, 0, 0, 0);
    DescPose offdese(0, 0, 100, 0, 0, 0);
    robot.MoveJ(&p1Joint, &p1Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.SetTcp4RefPoint(1);
    robot.MoveJ(&p2Joint, &p2Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.SetTcp4RefPoint(2);
    robot.MoveJ(&p3Joint, &p3Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.SetTcp4RefPoint(3);
    robot.MoveJ(&p4Joint, &p4Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.SetTcp4RefPoint(4);
    DescPose coordRtn = {};
    rtn = robot.ComputeTcp4(&coordRtn);
    printf("4 Point ComputeTool    %d coord is %f %f %f %f %f %f \n", rtn, coordRtn.tran.x, coordRtn.tran.y, coordRtn.tran.z, coordRtn.rpy.rx, coordRtn.rpy.ry, coordRtn.rpy.rz);
    robot.SetToolCoord(1, &coordRtn, 0, 0, 1, 0);
    robot.GetForwardKin(&p1Joint, &p1Desc);
    robot.GetForwardKin(&p2Joint, &p2Desc);
    robot.GetForwardKin(&p3Joint, &p3Desc);
    robot.SetFocusCalibPoint(1, p1Desc);
    robot.SetFocusCalibPoint(2, p2Desc);
    robot.SetFocusCalibPoint(3, p3Desc);
    DescTran resultPos = {};
    float accuracy = 0.0;
    rtn = robot.ComputeFocusCalib(3, resultPos, accuracy);
    printf("ComputeFocusCalib coord is %d %f %f %f accuracy is %f\n", rtn, resultPos.x, resultPos.y, resultPos.z, accuracy);
    rtn = robot.SetFocusPosition(resultPos);
    robot.GetForwardKin(&p5Joint, &p5Desc);
    robot.GetForwardKin(&p6Joint, &p6Desc);
    robot.MoveL(&p5Joint, &p5Desc, 1, 0, 10, 100, 100, -1, 0, &exaxisPos, 0, 1, &offdese);
    robot.MoveL(&p6Joint, &p6Desc, 1, 0, 10, 100, 100, -1, 0, &exaxisPos, 0, 1, &offdese);
    robot.FocusStart(50, 19, 710, 90, 0);
    robot.MoveL(&p5Joint, &p5Desc, 1, 0, 10, 100, 100, -1, 0, &exaxisPos, 0, 1, &offdese);
    robot.MoveL(&p6Joint, &p6Desc, 1, 0, 10, 100, 100, -1, 0, &exaxisPos, 0, 1, &offdese);
    robot.FocusEnd();
    robot.CloseRPC();
    return 0;
}

int TestFriction(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;

    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    float lcoeff[6] = { 0.9,0.9,0.9,0.9,0.9,0.9 };
    float wcoeff[6] = { 0.4,0.4,0.4,0.4,0.4,0.4 };
    float ccoeff[6] = { 0.6,0.6,0.6,0.6,0.6,0.6 };
    float fcoeff[6] = { 0.89,0.89,0.89,0.78,0.78,0.78 };
    rtn = robot.FrictionCompensationOnOff(1);
    printf("FrictionCompensationOnOff rtn is %d\n", rtn);
    rtn = robot.SetFrictionValue_level(lcoeff);
    printf("SetFrictionValue_level rtn is %d\n", rtn);
    rtn = robot.SetFrictionValue_wall(wcoeff);
    printf("SetFrictionValue_wall rtn is %d\n", rtn);
    rtn = robot.SetFrictionValue_ceiling(ccoeff);
    printf("SetFrictionValue_ceiling rtn is %d\n", rtn);
    rtn = robot.SetFrictionValue_freedom(fcoeff);
    printf("SetFrictionValue_freedom rtn is %d\n", rtn);
    robot.CloseRPC();
    return 0;
}

int TestGetError(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    int maincode, subcode;
    robot.GetRobotErrorCode(&maincode, &subcode);
    printf("robot maincode is %d; subcode is %d\n", maincode, subcode);
    robot.ResetAllError();
    robot.Sleep(1000);
    robot.GetRobotErrorCode(&maincode, &subcode);
    printf("robot maincode is %d; subcode is %d\n", maincode, subcode);
    robot.CloseRPC();
    return 0;
}

int TestWideVoltageCtrlBoxtemp(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    printf("robot rpc rtn is %d\n", rtn);
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    robot.SetWideBoxTempFanMonitorParam(1, 2);
    int enable = 0;
    int period = 0;
    robot.GetWideBoxTempFanMonitorParam(enable, period);
    printf("GetWideBoxTempFanMonitorParam enable is %d   period is %d\n", enable, period);
    for (int i = 0; i < 100; i++)
    {
        robot.GetRobotRealTimeState(&pkg);
        printf("robot ctrl box temp is %f,  fan current is %d\n", pkg.wideVoltageCtrlBoxTemp, pkg.wideVoltageCtrlBoxFanCurrent);
        robot.Sleep(100);
    }
    rtn = robot.SetWideBoxTempFanMonitorParam(0, 2);
    printf("SetWideBoxTempFanMonitorParam rtn is %d\n", rtn);
    enable = 0;
    period = 0;
    robot.GetWideBoxTempFanMonitorParam(enable, period);
    printf("GetWideBoxTempFanMonitorParam enable is %d   period is %d\n", enable, period);
    for (int i = 0; i < 100; i++)
    {
        robot.GetRobotRealTimeState(&pkg);
        printf("robot ctrl box temp is %f,  fan current is %d\n", pkg.wideVoltageCtrlBoxTemp, pkg.wideVoltageCtrlBoxFanCurrent);
        robot.Sleep(100);
    }
    robot.CloseRPC();
    robot.Sleep(2000);
    return 0;
}

int TestSensitivityCalib()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    robot.SetReConnectParam(true, 30000, 500);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return 0;
    }
    rtn = robot.JointSensitivityEnable(0);
    rtn = robot.JointSensitivityEnable(1);
    printf("JointSensitivityEnable rtn is %d\n", rtn);
    JointPos curJPos = {};
    robot.GetActualJointPosDegree(0, &curJPos);
    ExaxisPos epos = { 0,0,0,0 };
    DescPose offset_pos = { 0,0,0,0,0,0 };
    JointPos jointPos1 = { curJPos.jPos[0], 0, 0, -90, 0.02, curJPos.jPos[5] };
    DescPose descPos1 = {};
    robot.GetForwardKin(&jointPos1, &descPos1);
    robot.MoveJ(&jointPos1, &descPos1, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.Sleep(200);
    rtn = robot.JointSensitivityCollect();
    printf("JointSensitivityCollect 1 rtn is %d\n", rtn);
    robot.Sleep(100);
    JointPos jointPos2 = { curJPos.jPos[0], -30, 0, -90, 0.02, curJPos.jPos[5] };
    DescPose descPos2 = {};
    robot.GetForwardKin(&jointPos2, &descPos2);
    robot.MoveJ(&jointPos2, &descPos2, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.Sleep(100);
    rtn = robot.JointSensitivityCollect();
    printf("JointSensitivityCollect 2 rtn is %d\n", rtn);
    robot.Sleep(100);
    JointPos jointPos3 = { curJPos.jPos[0], -60, 0, -90, 0.02, curJPos.jPos[5] };
    DescPose descPos3 = {};
    robot.GetForwardKin(&jointPos3, &descPos3);
    robot.MoveJ(&jointPos3, &descPos3, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.Sleep(100);
    rtn = robot.JointSensitivityCollect();
    printf("JointSensitivityCollect 3 rtn is %d\n", rtn);
    robot.Sleep(100);
    JointPos jointPos4 = { curJPos.jPos[0], -90, 0, -90, 0.02, curJPos.jPos[5] };
    DescPose descPos4 = {};
    robot.GetForwardKin(&jointPos4, &descPos4);
    robot.MoveJ(&jointPos4, &descPos4, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.Sleep(100);
    rtn = robot.JointSensitivityCollect();
    printf("JointSensitivityCollect 4 rtn is %d\n", rtn);
    robot.Sleep(100);
    JointPos jointPos5 = { curJPos.jPos[0], -120, 0, -90, 0.02, curJPos.jPos[5] };
    DescPose descPos5 = {};
    robot.GetForwardKin(&jointPos5, &descPos5);
    robot.MoveJ(&jointPos5, &descPos5, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.Sleep(100);
    rtn = robot.JointSensitivityCollect();
    printf("JointSensitivityCollect 5 rtn is %d\n", rtn);
    robot.Sleep(100);
    JointPos jointPos6 = { curJPos.jPos[0], -150, 0, -90, 0.02, curJPos.jPos[5] };
    DescPose descPos6 = {};
    robot.GetForwardKin(&jointPos6, &descPos6);
    robot.MoveJ(&jointPos6, &descPos6, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.Sleep(100);
    rtn = robot.JointSensitivityCollect();
    printf("JointSensitivityCollect 6 rtn is %d\n", rtn);
    robot.Sleep(100);
    JointPos jointPos7 = { curJPos.jPos[0], -180, 0, -90, 0.02, curJPos.jPos[5] };
    DescPose descPos7 = {};
    robot.GetForwardKin(&jointPos7, &descPos7);
    robot.MoveJ(&jointPos7, &descPos7, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.Sleep(100);
    rtn = robot.JointSensitivityCollect();
    printf("JointSensitivityCollect 7 rtn is %d\n", rtn);
    robot.Sleep(100);
    //-------------------
    robot.MoveJ(&jointPos6, &descPos6, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.Sleep(100);
    rtn = robot.JointSensitivityCollect();
    printf("JointSensitivityCollect 8 rtn is %d\n", rtn);
    robot.Sleep(100);
    robot.MoveJ(&jointPos5, &descPos5, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.Sleep(100);
    rtn = robot.JointSensitivityCollect();
    printf("JointSensitivityCollect 9 rtn is %d\n", rtn);
    robot.Sleep(100);
    robot.MoveJ(&jointPos4, &descPos4, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.Sleep(100);
    rtn = robot.JointSensitivityCollect();
    printf("JointSensitivityCollect 10 rtn is %d\n", rtn);
    robot.Sleep(100);
    robot.MoveJ(&jointPos3, &descPos3, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.Sleep(100);
    rtn = robot.JointSensitivityCollect();
    printf("JointSensitivityCollect 11 rtn is %d\n", rtn);
    robot.Sleep(100);
    robot.MoveJ(&jointPos2, &descPos2, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.Sleep(100);
    rtn = robot.JointSensitivityCollect();
    printf("JointSensitivityCollect 12 rtn is %d\n", rtn);
    robot.Sleep(100);
    robot.MoveJ(&jointPos1, &descPos1, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.Sleep(200);
    rtn = robot.JointSensitivityCollect();
    printf("JointSensitivityCollect 13 rtn is %d\n", rtn);
    robot.Sleep(100);
    double calibResult[6] = { 0.0 };
    double linearity[6] = { 0.0 };
    rtn = robot.JointSensitivityCalibration(calibResult, linearity);
    printf("JointSensitivityCalibration rtn is %d\n", rtn);
    rtn = robot.JointSensitivityEnable(0);
    printf("JointSensitivityEnable rtn is %d\n", rtn);
    printf("jointSensor Calib result is %f %f %f %f %f %f\njointSensor linearity is %f %f %f %f %f %f\n",
        calibResult[0], calibResult[1], calibResult[2],
        calibResult[3], calibResult[4], calibResult[5],
        linearity[0], linearity[1], linearity[2],
        linearity[3], linearity[4], linearity[5]);
    double hysteresisError[6] = { 0.0 };
    rtn = robot.JointHysteresisError(hysteresisError);
    printf("JointHysteresisError result is %f %f %f %f %f %f\n",
        hysteresisError[0], hysteresisError[1], hysteresisError[2],
        hysteresisError[3], hysteresisError[4], hysteresisError[5]);
    double repeatability[6] = { 0.0 };
    rtn = robot.JointRepeatability(repeatability);
    printf("JointRepeatability result is %f %f %f %f %f %f\n",
        repeatability[0], repeatability[1], repeatability[2],
        repeatability[3], repeatability[4], repeatability[5]);
    double M[6] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
    double B[6] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
    double K[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    double threshold[6] = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };
    int setZeroFlag = 1;
    rtn = robot.SetAdmittanceParams(M, B, K, threshold, calibResult, setZeroFlag);
    printf("SetAdmittanceParams rtn is %d\n", rtn);
    robot.CloseRPC();
}

int TestSlavePortErr()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return 0;
    }
    robot.SetReConnectParam(true, 30000, 500);
    int inRecvErr[8] = { 0 };
    int inCRCErr[8] = { 0 };
    int inTransmitErr[8] = { 0 };
    int inLinkErr[8] = { 0 };
    int outRecvErr[8] = { 0 };
    int outCRCErr[8] = { 0 };
    int outTransmitErr[8] = { 0 };
    int outLinkErr[8] = { 0 };
    robot.GetSlavePortErrCounter(inRecvErr, inCRCErr, inTransmitErr, inLinkErr,
        outRecvErr, outCRCErr, outTransmitErr, outLinkErr);
    for (int i = 0; i < 8; i++)
    {
        if (inRecvErr[i] != 0)
        {
            printf("inRecvErr %d is %d\n", i, inRecvErr[i]);
        }
        if (inCRCErr[i] != 0)
        {
            printf("inCRCErr %d is %d\n", i, inCRCErr[i]);
        }
        if (inTransmitErr[i] != 0)
        {
            printf("inTransmitErr %d is %d\n", i, inTransmitErr[i]);
        }
        if (inLinkErr[i] != 0)
        {
            printf("inLinkErr %d is %d\n", i, inLinkErr[i]);
        }
        if (outRecvErr[i] != 0)
        {
            printf("outRecvErr %d is %d\n", i, outRecvErr[i]);
        }
        if (outCRCErr[i] != 0)
        {
            printf("outCRCErr %d is %d\n", i, outCRCErr[i]);
        }
        if (outTransmitErr[i] != 0)
        {
            printf("outTransmitErr %d is %d\n", i, outTransmitErr[i]);
        }
        if (outLinkErr[i] != 0)
        {
            printf("outLinkErr %d is %d\n", i, outLinkErr[i]);
        }
    }
    printf("others has no err!\n");
    for (int i = 0; i < 8; i++)
    {
        robot.SlavePortErrCounterClear(i);
    }
    robot.CloseRPC();
    return 0;
}

int TestVelFeedForwardRatio()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return 0;
    }
    robot.SetReConnectParam(true, 30000, 500);
    double setRadio[6] = { 0.23, 0.89, 0.28, 1.0, 1.0, 0.76 };
    robot.SetVelFeedForwardRatio(setRadio);
    double getRadio[6] = { 0.0 };
    robot.GetVelFeedForwardRatio(getRadio);
    printf(" %f %f %f %f %f %f\n", getRadio[0], getRadio[1], getRadio[2], getRadio[3], getRadio[4], getRadio[5]);
    robot.CloseRPC();
    return 0;
}

int TestPhotoelectricSensorTCPCalib(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return 0;
    }
    robot.SetReConnectParam(true, 30000, 500);
    DescTran offset = { 10.0, 10.0, 3.0 };
    DescPose TCP = {};
    rtn = robot.PhotoelectricSensorTCPCalibration("FR_CalibrateTheToolTcp-061101.lua", offset, TCP);
    printf("PhotoelectricSensorTCPCalibration rtn is  %d %f %f %f %f %f %f \n", rtn, TCP.tran.x, TCP.tran.y, TCP.tran.z, TCP.rpy.rx, TCP.rpy.ry, TCP.rpy.rz);
    robot.CloseRPC();
    robot.Sleep(9999999);
    return 0;
}


#pragma endregion
#pragma region 机器人安全设置

int TestCollision(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    int mode = 0;
    int config = 1;
    float level1[6] = { 1.0,2.0,3.0,4.0,5.0,6.0 };
    float level2[6] = { 0.5,0.2,0.3,0.4,0.5,0.6 };
    rtn = robot.SetAnticollision(mode, level1, config);
    printf("SetAnticollision mode 0 rtn is %d\n", rtn);
    mode = 1;
    rtn = robot.SetAnticollision(mode, level2, config);
    printf("SetAnticollision mode 1 rtn is %d\n", rtn);
    JointPos p1Joint(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
    JointPos p2Joint(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
    DescPose p1Desc(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
    DescPose p2Desc(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
    ExaxisPos exaxisPos(0.0, 0.0, 0.0, 0.0);
    DescPose offdese(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    robot.MoveL(&p2Joint, &p2Desc, 0, 0, 100, 100, 100, 2, &exaxisPos, 0, 0, &offdese);
    robot.ResetAllError();
    int safety[6] = { 5,5,5,5,5,5 };
    rtn = robot.SetCollisionStrategy(3, 1000, 150, 250, safety);
    printf("SetCollisionStrategy rtn is %d\n", rtn);
    double jointDetectionThreshould[6] = { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 };
    double tcpDetectionThreshould[6] = { 60,60,60,60,60,60 };
    rtn = robot.CustomCollisionDetectionStart(3, jointDetectionThreshould, tcpDetectionThreshould, 0);
    cout << "CustomCollisionDetectionStart rtn is " << rtn << endl;
    robot.MoveL(&p1Joint, &p1Desc, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
    robot.MoveL(&p2Joint, &p2Desc, 0, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
    rtn = robot.CustomCollisionDetectionEnd();
    cout << "CustomCollisionDetectionEnd rtn is " << rtn << endl;
    robot.CloseRPC();
    return 0;
}

int TestLimit(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    float plimit[6] = { 170.0,80.0,150.0,80.0,170.0,160.0 };
    robot.SetLimitPositive(plimit);
    float nlimit[6] = { -170.0,-260.0,-150.0,-260.0,-170.0,-160.0 };
    robot.SetLimitNegative(nlimit);
    float neg_deg[6] = { 0.0 }, pos_deg[6] = { 0.0 };
    robot.GetJointSoftLimitDeg(0, neg_deg, pos_deg);
    printf("neg limit deg:%f,%f,%f,%f,%f,%f\n", neg_deg[0], neg_deg[1], neg_deg[2], neg_deg[3], neg_deg[4], neg_deg[5]);
    printf("pos limit deg:%f,%f,%f,%f,%f,%f\n", pos_deg[0], pos_deg[1], pos_deg[2], pos_deg[3], pos_deg[4], pos_deg[5]);
    robot.CloseRPC();
    return 0;
}

int TestCollisionMethod(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    rtn = robot.SetCollisionDetectionMethod(0, 0);
    printf("SetCollisionDetectionMethod rtn is %d\n", rtn);
    rtn = robot.SetStaticCollisionOnOff(1);
    printf("SetStaticCollisionOnOff On rtn is %d\n", rtn);
    rtn = robot.Sleep(5000);
    rtn = robot.SetStaticCollisionOnOff(0);
    printf("SetStaticCollisionOnOff Off rtn is %d\n", rtn);
    robot.CloseRPC();
    return 0;
}

int TestPowerLimit(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    robot.DragTeachSwitch(1);
    robot.SetPowerLimit(1, 0.03);
    float torques[] = { 0, 0, 0, 0, 0, 0 };
    robot.GetJointTorques(1, torques);
    int count = 100;
    robot.ServoJTStart();
    int error = 0;
    while (count > 0)
    {
        error = robot.ServoJT(torques, 0.001);
        count = count - 1;
        robot.Sleep(1);
    }
    error = robot.ServoJTEnd();
    robot.DragTeachSwitch(0);
    robot.CloseRPC();
    return 0;
}

int TestSetVelReducePara()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    JointPos j1(0, -90, 90, 0, 0, 0);
    JointPos j2(90, -90, 90, 0, 0, 0);
    ExaxisPos epos(0, 0, 0, 0);
    DescPose offset_pos(0, 0, 0, 0, 0, 0);
    robot.SetSpeed(80);
    rtn = robot.SetVelReducePara(2, 30, 1);
    printf("SetVelReducePara param error rtn is %d\n", rtn);
    rtn = robot.SetVelReducePara(0, 30, 1);
    printf("SetVelReducePara disable reduce vel rtn is %d\n", rtn);
    robot.MoveJ(&j1, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.MoveJ(&j2, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    rtn = robot.SetVelReducePara(1, 30, 1);
    printf("SetVelReducePara reduce vel rtn is %d\n", rtn);
    robot.MoveJ(&j1, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.MoveJ(&j2, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    rtn = robot.SetVelReducePara(2, 30, 2);
    printf("SetVelReducePara disable robot rtn is %d\n", rtn);
    robot.MoveJ(&j1, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.MoveJ(&j2, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.Sleep(2000);
    robot.ResetAllError();
    robot.RobotEnable(1);
    robot.Sleep(1000);
    rtn = robot.SetVelReducePara(2, 30, 0);
    printf("SetVelReducePara report error rtn is %d\n", rtn);
    robot.MoveJ(&j1, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.MoveJ(&j2, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
   
    robot.Sleep(2000);
    robot.CloseRPC();
    return 0;
}

int TestSetJointVelReducePara()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    robot.SetReConnectParam(true, 30000, 500);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    
    JointPos j1(10.220, -11.121, -118.086, -46.739, 82.036, 131.503);
    JointPos j2(89.782, -11.122, -118.086, -46.740, 82.036, 131.504);
    ExaxisPos epos(0, 0, 0, 0);
    DescPose offset_pos(0, 0, 0, 0, 0, 0);
    robot.SetSpeed(20);

    std::vector<double> maxJointVelA = {100.0, 100.0, 100.0, 100.0, 100.0, 100.0 };
    rtn = robot.SetVelReducePara(2, 200, 0, maxJointVelA);
    printf("SetVelReducePara param error rtn is %d\n", rtn);
    robot.MoveJ(&j1, 1, 2, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.MoveJ(&j2, 1, 2, 100, 100, 100, &epos, -1, 0, &offset_pos);

    std::vector<double> maxJointVelB = { 20.0, 20.0, 20.0, 20.0, 20.0, 20.0 };
    rtn = robot.SetVelReducePara(2, 200, 0, maxJointVelB);
    printf("SetVelReducePara reduce vel rtn is %d\n", rtn);
    robot.MoveJ(&j1, 1, 2, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.MoveJ(&j2, 1, 2, 100, 100, 100, &epos, -1, 0, &offset_pos);
   
    robot.Sleep(2000);
    robot.CloseRPC();
    return 0;
}

int TestSafetyParamsCheckSum()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    robot.SetReConnectParam(true, 30000, 500);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }

    int status = 0;
    uint32_t checksum = 0;

    rtn = robot.GetSafetyParamsCheckSum(status, checksum);
    printf("GetSafetyParamsCheckSum: error=%d, status=%d, hex_code=  0x%x  \n", rtn, status, checksum);
    robot.Sleep(3000);

    rtn = robot.SafetyOPPasswordCheck(0, "123");
    printf("SafetyOPPasswordCheck: error= %d\n", rtn);


    float level1[6] = { 2.0, 2.0, 2.0, 2.0, 2.0, 2.0 };
    rtn = robot.SetAnticollision(0, level1, 1);
    printf("SetAnticollision: error=%d\n", rtn);

    int safety[6] = { 10, 10, 10, 10, 10, 10 };
    rtn = robot.SetCollisionStrategy(0, 1000, 150, 0, safety);
    printf("SetCollisionStrategy rtn is %d\n", rtn);
    

    robot.Sleep(1000);

    rtn = robot.GetSafetyParamsCheckSum(status, checksum);
    printf("GetSafetyParamsCheckSum(again): error=%d, status=%d, hex_code=  0x%x  \n", rtn, status, checksum);

    robot.Sleep(2000);
    robot.CloseRPC();
    return 0;
}

#pragma endregion
#pragma region 机器人状态查询

int TestGetStatus(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    float yangle, zangle;
    robot.GetRobotInstallAngle(&yangle, &zangle);
    printf("yangle:%f,zangle:%f\n", yangle, zangle);
    JointPos j_deg = {};
    robot.GetActualJointPosDegree(0, &j_deg);
    printf("joint pos deg:%f,%f,%f,%f,%f,%f\n", j_deg.jPos[0], j_deg.jPos[1], j_deg.jPos[2], j_deg.jPos[3], j_deg.jPos[4], j_deg.jPos[5]);
    float jointSpeed[6] = { 0.0 };
    robot.GetActualJointSpeedsDegree(0, jointSpeed);
    printf("joint speeds deg:%f,%f,%f,%f,%f,%f\n", jointSpeed[0], jointSpeed[1], jointSpeed[2], jointSpeed[3], jointSpeed[4], jointSpeed[5]);
    float jointAcc[6] = { 0.0 };
    robot.GetActualJointAccDegree(0, jointAcc);
    printf("joint acc deg:%f,%f,%f,%f,%f,%f\n", jointAcc[0], jointAcc[1], jointAcc[2], jointAcc[3], jointAcc[4], jointAcc[5]);
    float tcp_speed = 0.0;
    float ori_speed = 0.0;
    robot.GetTargetTCPCompositeSpeed(0, &tcp_speed, &ori_speed);
    printf("GetTargetTCPCompositeSpeed tcp %f; ori %f\n", tcp_speed, ori_speed);
    robot.GetActualTCPCompositeSpeed(0, &tcp_speed, &ori_speed);
    printf("GetActualTCPCompositeSpeed tcp %f; ori %f\n", tcp_speed, ori_speed);
    float targetSpeed[6] = { 0.0 };
    robot.GetTargetTCPSpeed(0, targetSpeed);
    printf("GetTargetTCPSpeed %f,%f,%f,%f,%f,%f\n", targetSpeed[0], targetSpeed[1], targetSpeed[2], targetSpeed[3], targetSpeed[4], targetSpeed[5]);
    float actualSpeed[6] = { 0.0 };
    robot.GetActualTCPSpeed(0, actualSpeed);
    printf("GetTargetTCPSpeed %f,%f,%f,%f,%f,%f\n", actualSpeed[0], actualSpeed[1], actualSpeed[2], actualSpeed[3], actualSpeed[4], actualSpeed[5]);
    DescPose tcp = {};
    robot.GetActualTCPPose(0, &tcp);
    printf("tcp pose:%f,%f,%f,%f,%f,%f\n", tcp.tran.x, tcp.tran.y, tcp.tran.z, tcp.rpy.rx, tcp.rpy.ry, tcp.rpy.rz);
    DescPose flange = {};
    robot.GetActualToolFlangePose(0, &flange);
    printf("flange pose:%f,%f,%f,%f,%f,%f\n", flange.tran.x, flange.tran.y, flange.tran.z, flange.rpy.rx, flange.rpy.ry, flange.rpy.rz);
    int id = 0;
    robot.GetActualTCPNum(0, &id);
    printf("tcp num:%d\n", id);
    robot.GetActualWObjNum(0, &id);
    printf("wobj num:%d\n", id);
    float jtorque[6] = { 0.0 };
    robot.GetJointTorques(0, jtorque);
    printf("torques:%f,%f,%f,%f,%f,%f\n", jtorque[0], jtorque[1], jtorque[2], jtorque[3], jtorque[4], jtorque[5]);
    float t_ms = 0.0;
    robot.GetSystemClock(&t_ms);
    printf("system clock:%f\n", t_ms);
    int config = 0;
    robot.GetRobotCurJointsConfig(&config);
    printf("joint config:%d\n", config);
    uint8_t motionDone = 0;
    robot.GetRobotMotionDone(&motionDone);
    printf("GetRobotMotionDone :%d\n", motionDone);
    int len = 0;
    robot.GetMotionQueueLength(&len);
    printf("GetMotionQueueLength :%d\n", len);
    uint8_t emergState = 0;
    robot.GetRobotEmergencyStopState(&emergState);
    printf("GetRobotEmergencyStopState :%d\n", emergState);
    int comstate = 0;
    robot.GetSDKComState(&comstate);
    printf("GetSDKComState :%d\n", comstate);
    uint8_t si0_state, si1_state;
    robot.GetSafetyStopState(&si0_state, &si1_state);
    printf("GetSafetyStopState :%d %d\n", si0_state, si1_state);
    double temp[6] = { 0.0 };
    robot.GetJointDriverTemperature(temp);
    printf("Temperature:%f,%f,%f,%f,%f,%f\n", temp[0], temp[1], temp[2], temp[3], temp[4], temp[5]);
    double torque[6] = { 0.0 };
    robot.GetJointDriverTorque(torque);
    printf("torque:%f,%f,%f,%f,%f,%f\n", torque[0], torque[1], torque[2], torque[3], torque[4], torque[5]);
    robot.GetRobotRealTimeState(&pkg);
    robot.CloseRPC();
    return 0;
}


int TestGetTeachPoint(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    char name[64] = "P1";
    float data[20] = { 0 };
    rtn = robot.GetRobotTeachingPoint(name, data);
    printf(" %d name is: %s \n", rtn, name);
    for (int i = 0; i < 20; i++)
    {
        printf("data is: %f \n", data[i]);
    }
    int que_len = 0;
    rtn = robot.GetMotionQueueLength(&que_len);
    printf("GetMotionQueueLength rtn is: %d, queue length is: %d \n", rtn, que_len);
    double dh[6] = { 0 };
    int retval = 0;
    retval = robot.GetDHCompensation(dh);
    cout << "retval is: " << retval << endl;
    cout << "dh is: " << dh[0] << " " << dh[1] << " " << dh[2] << " " << dh[3] << " " << dh[4] << " " << dh[5] << endl;
    string SN = "";
    robot.GetRobotSN(SN);
    cout << "robot SN is " << SN << endl;
    robot.CloseRPC();
    return 0;
}


int TestInverseKinExaxis()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return 0;
    }
    robot.SetReConnectParam(true, 30000, 500);
    DescPose desc(99.957, -0.002, 29.994, -176.569, -6.757, -167.462);
    ExaxisPos exaxis(100.0, 0.0, 0.0, 0.0);
    JointPos jointPos = {};
    DescPose offsetPos = {};
    robot.GetRobotRealTimeState(&pkg);
    int toolnum = pkg.tool;
    int workPcsNum = pkg.user;
    robot.GetInverseKinExaxis(0, desc, exaxis, toolnum, workPcsNum, jointPos);
    printf("GetInverseKinExaxis joint is %f, %f, %f, %f, %f, %f\n", jointPos.jPos[0], jointPos.jPos[1], jointPos.jPos[2], jointPos.jPos[3], jointPos.jPos[4], jointPos.jPos[5]);
    robot.ExtAxisMove(exaxis, 100, -1);
    robot.MoveJ(&jointPos, &desc, toolnum, workPcsNum, 100.0, 100.0, 100.0, &exaxis, -1, 0, &offsetPos);
    robot.CloseRPC();
    robot.Sleep(9999999);
    return 0;
}

int TestInverseKin(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
    DescPose desc_pos1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
    JointPos inverseRtn = {};
    robot.GetInverseKin(0, &desc_pos1, -1, &inverseRtn);
    printf("dcs1 GetInverseKin rtn is %f %f %f %f %f %f \n", inverseRtn.jPos[0], inverseRtn.jPos[1], inverseRtn.jPos[2], inverseRtn.jPos[3], inverseRtn.jPos[4], inverseRtn.jPos[5]);
    robot.GetInverseKinRef(0, &desc_pos1, &j1, &inverseRtn);
    printf("dcs1 GetInverseKinRef rtn is %f %f %f %f %f %f \n", inverseRtn.jPos[0], inverseRtn.jPos[1], inverseRtn.jPos[2], inverseRtn.jPos[3], inverseRtn.jPos[4], inverseRtn.jPos[5]);
    uint8_t hasResut = 0;
    robot.GetInverseKinHasSolution(0, &desc_pos1, &j1, &hasResut);
    printf("dcs1 GetInverseKinRef result %d\n", hasResut);
    DescPose forwordResult = {};
    robot.GetForwardKin(&j1, &forwordResult);
    printf("jpos1 forwordResult rtn is %f %f %f %f %f %f \n", forwordResult.tran.x, forwordResult.tran.y, forwordResult.tran.z, forwordResult.rpy.rx, forwordResult.rpy.ry, forwordResult.rpy.rz);
    robot.CloseRPC();
    return 0;
}

int TestCoord()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return 0;
    }
    robot.SetReConnectParam(true, 30000, 500);
    robot.Sleep(2000);
    int id = 1;
    DescPose toolCoord = {};
    DescPose extoolCoord = {};
    DescPose exworkpieceCoord = {};
    DescPose wobjCoord = {};
    DescPose exAxisCoord = {};
    int type = 0, install = 0, toolID = 0, loadNo = 0;
    robot.GetToolCoordWithID(id, toolCoord, type, install, toolID, loadNo);
    printf("GetToolCoordWithID %d, %f %f %f %f %f %f,  type = %d, install = %d, toolID = %d, loadNo = %d\n", id,
        toolCoord.tran.x, toolCoord.tran.y, toolCoord.tran.z,
        toolCoord.rpy.rx, toolCoord.rpy.ry, toolCoord.rpy.rz, type, install, toolID, loadNo);
    int refFrame = 0;
    robot.GetWObjCoordWithID(id, wobjCoord, refFrame);
    printf("GetWObjCoordWithID %d, %f %f %f %f %f %f, refFrame = %d\n", id,
        wobjCoord.tran.x, wobjCoord.tran.y, wobjCoord.tran.z,
        wobjCoord.rpy.rx, wobjCoord.rpy.ry, wobjCoord.rpy.rz, refFrame);


    robot.GetExToolCoordWithID(21, extoolCoord, exworkpieceCoord);
    printf("GetExToolCoordWithID %d, %f %f %f %f %f %f\n", id,
        extoolCoord.tran.x, extoolCoord.tran.y, extoolCoord.tran.z,
        extoolCoord.rpy.rx, extoolCoord.rpy.ry, extoolCoord.rpy.rz,
        exworkpieceCoord.tran.x, exworkpieceCoord.tran.y, exworkpieceCoord.tran.z,
        exworkpieceCoord.rpy.rx, exworkpieceCoord.rpy.ry, exworkpieceCoord.rpy.rz);

    int axisCoordNum = 0, calibFlag = 0;
    robot.GetExAxisCoordWithID(id, exAxisCoord, axisCoordNum, calibFlag);
    printf("GetExAxisCoordWithID %d, %f %f %f %f %f %f, axisCoordNum = %d, calibFlag = %d\n", id,
        exAxisCoord.tran.x, exAxisCoord.tran.y, exAxisCoord.tran.z,
        exAxisCoord.rpy.rx, exAxisCoord.rpy.ry, exAxisCoord.rpy.rz, axisCoordNum, calibFlag);

    double weight = 0.0;
    DescTran cog = {};
    robot.GetTargetPayloadWithID(id, weight, cog);
    printf("GetTargetPayloadWithID %d, %f %f %f %f\n", id, weight,
        cog.x, cog.y, cog.z);
    robot.GetCurToolCoord(toolCoord);
    printf("GetCurToolCoord %f %f %f %f %f %f\n",
        toolCoord.tran.x, toolCoord.tran.y, toolCoord.tran.z,
        toolCoord.rpy.rx, toolCoord.rpy.ry, toolCoord.rpy.rz);
    robot.GetCurWObjCoord(wobjCoord);
    printf("GetCurWObjCoord %f %f %f %f %f %f\n",
        wobjCoord.tran.x, wobjCoord.tran.y, wobjCoord.tran.z,
        wobjCoord.rpy.rx, wobjCoord.rpy.ry, wobjCoord.rpy.rz);
    robot.GetCurExToolCoord(extoolCoord);
    printf("GetExToolCoordWithID %f %f %f %f %f %f\n",
        extoolCoord.tran.x, extoolCoord.tran.y, extoolCoord.tran.z,
        extoolCoord.rpy.rx, extoolCoord.rpy.ry, extoolCoord.rpy.rz);
    robot.GetCurExAxisCoord(exAxisCoord);
    printf("GetCurExAxisCoord %f %f %f %f %f %f\n",
        exAxisCoord.tran.x, exAxisCoord.tran.y, exAxisCoord.tran.z,
        exAxisCoord.rpy.rx, exAxisCoord.rpy.ry, exAxisCoord.rpy.rz);
    float weightT = 0.0;
    DescTran cogT = {};
    robot.GetTargetPayload(0, &weightT);
    robot.GetTargetPayloadCog(0, &cogT);
    printf("GetTargetPayload %f %f %f %f\n", weightT,
        cogT.x, cogT.y, cogT.z);
    DescPose coordSet(0, 1, 2, 3, 4, 5);
    robot.SetToolCoord(1, &coordSet, 0, 0, 1, 0);
    robot.SetWObjCoord(1, &coordSet, 0);
    robot.SetLoadWeight(1, 1.3);
    //DescTran cog = {};
    cog.x = 10;
    cog.y = 20;
    cog.z = 30;
    robot.SetLoadCoord(1, &cog);
    DescPose etcp(0, 0, 100, 0, 0, 0);
    DescPose etool(0, 0, 50, 0, 0, 0);
    rtn = robot.SetExToolCoord(21, &etcp, &etool);
    printf("SetExToolCoord rtn is %d\n", rtn);
    robot.ExtAxisActiveECoordSys(1, 1, coordSet, 1);
    robot.CloseRPC();
    return 0;
}

#pragma endregion
#pragma region 机器人轨迹复现

int TestTPD(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    int type = 1;
    char name[30] = "tpd2025";
    int period_ms = 4;
    uint16_t di_choose = 0;
    uint16_t do_choose = 0;
    robot.SetTPDParam(type, name, period_ms, di_choose, do_choose);
    robot.Mode(1);
    robot.Sleep(1000);
    robot.DragTeachSwitch(1);
    robot.SetTPDStart(type, name, period_ms, di_choose, do_choose);
    robot.Sleep(3000);
    robot.SetWebTPDStop();
    robot.DragTeachSwitch(0);
    robot.Sleep(1000);
    float ovl = 100.0;
    uint8_t blend = 0;
    DescPose start_pose = {};
    rtn = robot.LoadTPD(name);
    printf("LoadTPD rtn is: %d\n", rtn);
    robot.GetTPDStartPose(name, &start_pose);
    printf("start pose, xyz is: %f %f %f. rpy is: %f %f %f \n", start_pose.tran.x, start_pose.tran.y, start_pose.tran.z, start_pose.rpy.rx, start_pose.rpy.ry, start_pose.rpy.rz);
    rtn = robot.MoveToTPDStart(name, 0, 100);
    printf("MoveToTPDStart rtn is: %d\n", rtn);
    rtn = robot.MoveTPD(name, blend, ovl);
    printf("MoveTPD rtn is: %d\n", rtn);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    robot.SetTPDDelete(name);
    robot.Sleep(1000);
    robot.CloseRPC();
    return 0;
}

int TestSetTrajectoryJSpeed()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    robot.SetReConnectParam(true, 30000, 500);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }

    rtn = robot.TrajectoryJUpLoad("D://zUP/horse.txt");
    printf("Upload TrajectoryJ A %d\n", rtn);
    char traj_file_name[90] = "horse.txt";
    rtn = robot.LoadTrajectoryJ(traj_file_name, 100, 1);
    printf("LoadTrajectoryJ %s, rtn is: %d\n", traj_file_name, rtn);
    DescPose traj_start_pose;
    memset(&traj_start_pose, 0, sizeof(DescPose));
    rtn = robot.GetTrajectoryStartPose(traj_file_name, &traj_start_pose);
    printf("GetTrajectoryStartPose is: %d\n", rtn);
    printf("desc_pos:%f,%f,%f,%f,%f,%f\n", traj_start_pose.tran.x, traj_start_pose.tran.y, traj_start_pose.tran.z, traj_start_pose.rpy.rx, traj_start_pose.rpy.ry, traj_start_pose.rpy.rz);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    robot.SetSpeed(50);
    robot.MoveCart(&traj_start_pose, 0, 0, 100, 100, 100, -1, -1);
    int traj_num = 0;
    rtn = robot.GetTrajectoryPointNum(&traj_num);
    printf("GetTrajectoryStartPose rtn is: %d, traj num is: %d\n", rtn, traj_num);
    rtn = robot.MoveTrajectoryJ();
    printf("MoveTrajectoryJ rtn is: %d\n", rtn);
    robot.Sleep(1000);
    robot.GetRobotRealTimeState(&pkg);
    int trajspeedMode = 1;
    while (pkg.motion_done == 0)
    {
        robot.GetRobotRealTimeState(&pkg);
        rtn = robot.SetTrajectoryJSpeed(10.0, trajspeedMode);
        printf("SetTrajectoryJSpeed is: %d\n", rtn);
        robot.Sleep(1000);
        rtn = robot.SetTrajectoryJSpeed(80.0, trajspeedMode);
        printf("SetTrajectoryJSpeed is: %d\n", rtn);
        robot.Sleep(1000);
    }
    robot.CloseRPC();
    robot.Sleep(1000000);
    return 0;
}

int TestTraj(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    rtn = robot.TrajectoryJUpLoad("D://zUP/trajHelix_aima_1.txt");
    printf("Upload TrajectoryJ A %d\n", rtn);
    char traj_file_name[30] = "trajHelix_aima_1.txt";
    rtn = robot.LoadTrajectoryJ(traj_file_name, 100, 1);
    printf("LoadTrajectoryJ %s, rtn is: %d\n", traj_file_name, rtn);
    DescPose traj_start_pose;
    memset(&traj_start_pose, 0, sizeof(DescPose));
    rtn = robot.GetTrajectoryStartPose(traj_file_name, &traj_start_pose);
    printf("GetTrajectoryStartPose is: %d\n", rtn);
    printf("desc_pos:%f,%f,%f,%f,%f,%f\n", traj_start_pose.tran.x, traj_start_pose.tran.y, traj_start_pose.tran.z, traj_start_pose.rpy.rx, traj_start_pose.rpy.ry, traj_start_pose.rpy.rz);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    robot.SetSpeed(50);
    robot.MoveCart(&traj_start_pose, 0, 0, 100, 100, 100, -1, -1);
    int traj_num = 0;
    rtn = robot.GetTrajectoryPointNum(&traj_num);
    printf("GetTrajectoryStartPose rtn is: %d, traj num is: %d\n", rtn, traj_num);
    rtn = robot.SetTrajectoryJSpeed(50.0);
    printf("SetTrajectoryJSpeed is: %d\n", rtn);
    ForceTorque traj_force;
    memset(&traj_force, 0, sizeof(ForceTorque));
    traj_force.fx = 10;
    rtn = robot.SetTrajectoryJForceTorque(&traj_force);
    printf("SetTrajectoryJForceTorque rtn is: %d\n", rtn);
    rtn = robot.SetTrajectoryJForceFx(10.0);
    printf("SetTrajectoryJForceFx rtn is: %d\n", rtn);
    rtn = robot.SetTrajectoryJForceFy(0.0);
    printf("SetTrajectoryJForceFy rtn is: %d\n", rtn);
    rtn = robot.SetTrajectoryJForceFz(0.0);
    printf("SetTrajectoryJForceFz rtn is: %d\n", rtn);
    rtn = robot.SetTrajectoryJTorqueTx(10.0);
    printf("SetTrajectoryJTorqueTx rtn is: %d\n", rtn);
    rtn = robot.SetTrajectoryJTorqueTy(10.0);
    printf("SetTrajectoryJTorqueTy rtn is: %d\n", rtn);
    rtn = robot.SetTrajectoryJTorqueTz(10.0);
    printf("SetTrajectoryJTorqueTz rtn is: %d\n", rtn);
    rtn = robot.MoveTrajectoryJ();
    printf("MoveTrajectoryJ rtn is: %d\n", rtn);
    robot.CloseRPC();
    return 0;
}


int TestLoadTrajLA(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    rtn = robot.TrajectoryJUpLoad("D://zUP/trajHelix_aima_1.txt");
    printf("Upload TrajectoryJ A %d\n", rtn);
    char traj_file_name[30] = "trajHelix_aima_1.txt";
    rtn = robot.LoadTrajectoryLA(traj_file_name, 1, 2, 0, 2, 100, 200, 1000);
    printf("LoadTrajectoryLA %s, rtn is: %d\n", traj_file_name, rtn);
    DescPose traj_start_pose;
    memset(&traj_start_pose, 0, sizeof(DescPose));
    rtn = robot.GetTrajectoryStartPose(traj_file_name, &traj_start_pose);
    printf("GetTrajectoryStartPose is: %d\n", rtn);
    printf("desc_pos:%f,%f,%f,%f,%f,%f\n", traj_start_pose.tran.x, traj_start_pose.tran.y, traj_start_pose.tran.z, traj_start_pose.rpy.rx, traj_start_pose.rpy.ry, traj_start_pose.rpy.rz);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    robot.SetSpeed(50);
    robot.MoveCart(&traj_start_pose, 0, 0, 100, 100, 100, -1, -1);
    rtn = robot.MoveTrajectoryLA();
    printf("MoveTrajectoryLA rtn is: %d\n", rtn);
    robot.CloseRPC();
    return 0;
}

#pragma endregion
#pragma region 机器人webapp程序

int TestLuaOp(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    char program_name[64] = "test.lua";
    char loaded_name[64] = "";
    uint8_t state;
    int line;
    robot.Mode(0);
    robot.LoadDefaultProgConfig(1, program_name);
    robot.ProgramLoad(program_name);
    robot.ProgramRun();
    robot.Sleep(1000);
    robot.ProgramPause();
    robot.GetProgramState(&state);
    printf("program state:%u\n", state);
    robot.GetCurrentLine(&line);
    printf("current line:%d\n", line);
    robot.GetLoadedProgram(loaded_name);
    printf("program name:%s\n", loaded_name);
    robot.Sleep(1000);
    robot.ProgramResume();
    robot.Sleep(1000);
    robot.ProgramStop();
    robot.Sleep(1000);
    robot.CloseRPC();
    return 0;
}

int TestLUAUpDownLoad(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    list<std::string> luaNames;
    rtn = robot.GetLuaList(&luaNames);
    std::cout << "res is: " << rtn << std::endl;
    std::cout << "size is: " << luaNames.size() << std::endl;
    for (auto it = luaNames.begin(); it != luaNames.end(); it++)
    {
        std::cout << it->c_str() << std::endl;
    }

    robot.Sleep(2000);
    rtn = robot.LuaDownLoad("test.lua", "D://zDOWN/");
    printf("LuaDownLoad rtn is %d\n", rtn);
    rtn = robot.LuaUpload("D://zUP/airlab.lua");
    printf("LuaUpload rtn is %d\n", rtn);
    rtn = robot.LuaDelete("test.lua");
    printf("LuaDelete rtn is %d\n", rtn);
    robot.CloseRPC();
    return 0;
}



#pragma endregion
#pragma region 机器人外设

int TestGripper(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    int company = 6;
    int device = 1;
    int softversion = 0;
    int bus = 1;
    int index = 1;
    int act = 0;
    int max_time = 30000;
    uint8_t block = 0;
    uint8_t status;
    uint16_t fault;
    uint16_t active_status = 0;
    uint8_t current_pos = 0;
    int8_t current = 0;
    int voltage = 0;
    int temp = 0;
    int8_t speed = 0;
    robot.SetGripperConfig(company, device, softversion, bus);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    robot.GetGripperConfig(&company, &device, &softversion, &bus);
    printf("gripper config:%d,%d,%d,%d\n", company, device, softversion, bus);
    robot.ActGripper(index, act);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    act = 1;
    robot.ActGripper(index, act);
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));
    rtn = robot.MoveGripper(index, 15, 50, 50, max_time, block, 0, 0, 0, 0);
    printf("MoveGripper to 100 %d\n", rtn);
    std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    rtn = robot.MoveGripper(index, 100, 50, 50, max_time, block, 0, 0, 0, 0);
    printf("MoveGripper to 15 %d\n", rtn);
    robot.GetGripperMotionDone(&fault, &status);
    printf("motion status:%u,%u\n", fault, status);
    robot.GetGripperActivateStatus(&fault, &active_status);
    printf("gripper active fault is: %u, status is: %u\n", fault, active_status);
    robot.GetGripperCurPosition(&fault, &current_pos);
    printf("fault is:%u, current position is: %u\n", fault, current_pos);
    robot.GetGripperCurCurrent(&fault, &current);
    printf("fault is:%u, current current is: %d\n", fault, current);
    robot.GetGripperVoltage(&fault, &voltage);
    printf("fault is:%u, current voltage is: %d \n", fault, voltage);
    robot.GetGripperTemp(&fault, &temp);
    printf("fault is:%u, current temperature is: %d\n", fault, temp);
    robot.GetGripperCurSpeed(&fault, &speed);
    printf("fault is:%u, current speed is: %d\n", fault, speed);
    int retval = 0;
    DescPose prepick_pose = {};
    DescPose postpick_pose = {};
    DescPose p1Desc(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
    DescPose p2Desc(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
    retval = robot.ComputePrePick(&p1Desc, 10, 0, &prepick_pose);
    printf("ComputePrePick retval is: %d\n", retval);
    printf("xyz is: %f, %f, %f; rpy is: %f, %f, %f\n", prepick_pose.tran.x, prepick_pose.tran.y, prepick_pose.tran.z, prepick_pose.rpy.rx, prepick_pose.rpy.ry, prepick_pose.rpy.rz);
    retval = robot.ComputePostPick(&p2Desc, -10, 0, &postpick_pose);
    printf("ComputePostPick retval is: %d\n", retval);
    printf("xyz is: %f, %f, %f; rpy is: %f, %f, %f\n", postpick_pose.tran.x, postpick_pose.tran.y, postpick_pose.tran.z, postpick_pose.rpy.rx, postpick_pose.rpy.ry, postpick_pose.rpy.rz);
    robot.CloseRPC();
    return 0;
}


int TestRotGripperState(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    uint16_t fault = 0;
    double rotNum = 0.0;
    int rotSpeed = 0;
    int rotTorque = 0;
    robot.GetGripperRotNum(&fault, &rotNum);
    robot.GetGripperRotSpeed(&fault, &rotSpeed);
    robot.GetGripperRotTorque(&fault, &rotTorque);
    printf("gripper rot num : %lf, gripper rotSpeed : %d, gripper rotTorque : %d\n", rotNum, rotSpeed, rotTorque);
    robot.CloseRPC();
    return 0;
}

int TestConveyor()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);

    DescPose pos1(-354.549, 63.914, 270.176, -179.679, -0.134, 2.468);
    DescPose pos2(-351.203, -213.393, 351.054, -179.932, -0.508, 2.472);


    double cmp[3] = { 0.0, 0.0, 0.0 };
    rtn = robot.ConveyorCatchPointComp(cmp);//设置传动带抓取点补偿
    if (rtn != 0)
    {
        return 0;
    }
    printf("ConveyorCatchPointComp: %d  \n" , rtn);

    rtn = robot.MoveCart(&pos1, 1, 0, (float)30.0, (float)180.0, (float)100.0, (float)-1.0, -1);
    printf("MoveCart: %d\n" , rtn);

    rtn = robot.ConveyorIODetect(10000);//传送带工件IO检测
    printf("ConveyorIODetect: %d\n   " , rtn);

    robot.ConveyorGetTrackData(1);//配置传送带跟踪抓取
    rtn = robot.ConveyorTrackStart(1);//跟踪开始
    printf("ConveyorTrackStart: %d\n  " , rtn);

    rtn = robot.TrackMoveL("cvrCatchPoint", 1, 0, (float)100.0, (float)0.0, (float)100.0, (float)-1.0, 0, 0);
    printf("ConveyorTrackMoveL: %d\n  " , rtn);

    rtn = robot.MoveGripper(2, 30, 60, 30, 30000, 0, 0, 0, 50, 50);
   printf("ConveyorTrackMoveL: %d\n  " , rtn);


    rtn = robot.TrackMoveL("cvrRaisePoint", 1, 0, (float)100.0, (float)0.0, (float)100.0, (float)-1.0, 0, 0);
   printf("ConveyorTrackMoveL: %d\n   " , rtn);

    rtn = robot.ConveyorTrackEnd();//传送带跟踪停止
   printf("ConveyorTrackEnd: %d\n " , rtn);

    rtn = robot.MoveCart(&pos2, 1, 0, (float)30.0, (float)180.0, (float)100.0, (float)-1.0, -1);
   printf("MoveCart: %d\n " , rtn);

    rtn = robot.MoveGripper(2, 100, 60, 30, 30000, 0, 0, 0, 50, 50);
   printf("MoveGripper: %d\n " , rtn);
   return 0;

}


/// <summary>
/// 测试静止跟踪 (SetStationaryTrackPara + MoveStationary)
/// SetDO(6,1) → ConveyorTrackStart → ConveyorIODetect → ConveyorGetTrackData
/// → SetStationaryTrackPara → MoveStationary → ConveyorTrackEnd → SetDO(6,0)
/// </summary>
int TestStationaryTrack()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);

    printf("\n========== 传送带静止跟踪测试 ==========");

    JointPos j1(-35.146, -102.684, 120.805, -100.401, -90.295, 150.105);
    DescPose d1(-121.814, -348.341, 209.978, -173.152, -3.585, -5.446);

    ExaxisPos ex(0, 0, 0, 0);
    DescPose zeroOff(0, 0, 0, 0, 0, 0);

    int tool = 1;
    int workpiece = 1;
    float conveyorParam[6] = { 0, 10000, 200, 0, 0, 10 };
    rtn = robot.ConveyorSetParam(conveyorParam);

    robot.MoveJ(&j1, &d1, tool, workpiece, 100, 100, 100, &ex, -1, 0, &zeroOff);

    // Step 1: SetDO 控制信号
    printf("--- Step 1: SetDO(6,1) ---\n");
    rtn = robot.SetDO(6, 1, 0, 0);
    printf("  SetDO(6,1) rtn={0}\n", rtn);

    // Step 2: 传送带跟踪开始
    printf("--- Step 2: ConveyorTrackStart(2) ---\n");
    rtn = robot.ConveyorTrackStart(2);
    printf("  ConveyorTrackStart(2) rtn={0}\n", rtn);

    // Step 3: 工件IO检测
    printf("--- Step 3: ConveyorIODetect(10000) ---\n");
    rtn = robot.ConveyorIODetect(10000);
    printf("  ConveyorIODetect(10000) rtn={0}\n", rtn);

    // Step 4: 获取跟踪数据
    printf("--- Step 4: ConveyorGetTrackData(2) ---\n");
    rtn = robot.ConveyorGetTrackData(2);
    printf("  ConveyorGetTrackData(2) rtn={0}\n", rtn);

    // Step 5: 静止跟踪参数配置 (时间模式, 200s, 距离5)
    printf("--- Step 5: SetStationaryTrackPara(0,200,5) ---\n");
    rtn = robot.SetStationaryTrackPara(0, 5, 5);
    printf("  SetStationaryTrackPara(0,200,5) rtn={0}\n", rtn);

    // Step 6: 执行静止跟踪运动
    printf("--- Step 6: MoveStationary() ---\n");
    rtn = robot.MoveStationary();
    rtn = robot.WaitStationaryMotionDone();
    printf("  MoveStationary() rtn={0}\n", rtn);

    // Step 7: 传送带跟踪结束
    printf("--- Step 7: ConveyorTrackEnd() ---\n");
    rtn = robot.ConveyorTrackEnd();
    printf("  ConveyorTrackEnd() rtn={0}\n", rtn);

    // Step 8: SetDO 关闭信号
    printf("--- Step 8: SetDO(6,0) ---\n");
    rtn = robot.SetDO(6, 0, 0, 0);
    printf("  SetDO(6,0) rtn={0}\n", rtn);

    printf("\n========== 静止跟踪测试完成 ==========\n");
    return 0;
}


int TestAxleSensor(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    robot.AxleSensorConfig(18, 0, 0, 1);
    int company = -1;
    int type = -1;
    robot.AxleSensorConfigGet(company, type);
    printf("company is %d, type is %d\n", company, type);
    rtn = robot.AxleSensorActivate(1);
    printf("AxleSensorActivate rtn is %d\n", rtn);
    robot.Sleep(1000);
    rtn = robot.AxleSensorRegWrite(1, 4, 6, 1, 0, 0, 0);
    printf("AxleSensorRegWrite rtn is %d\n", rtn);
    robot.CloseRPC();
    return 0;
}


int TestExDevProtocol(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    int protocol = 4096;
    rtn = robot.SetExDevProtocol(protocol);
    std::cout << "SetExDevProtocol rtn " << rtn << std::endl;
    rtn = robot.GetExDevProtocol(&protocol);
    std::cout << "GetExDevProtocol rtn " << rtn << " protocol is: " << protocol << std::endl;
    robot.CloseRPC();
    return 0;
}

void TestSucker()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    uint8_t ctrl[20];
    uint8_t state;
    int pressVlaue;
    int error;

    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return;
    }
    robot.SetReConnectParam(true, 30000, 500);
    //上传并加载开放协议文件
    robot.OpenLuaUpload("D://zUP/CtrlDev_sucker.lua");
    robot.Sleep(2000);
    robot.SetCtrlOpenLUAName(1, "CtrlDev_sucker.lua");
    robot.UnloadCtrlOpenLUA(1);
    robot.LoadCtrlOpenLUA(1);
    robot.Sleep(1000);

    //控制吸盘广播模式下，按照最大能力吸附
    ctrl[0] = 1;
    robot.SetSuckerCtrl(0, 1, ctrl);

    //循环监控1号吸盘和12号吸盘的状态
    for (int i = 0; i < 100; i++)
    {
        robot.GetSuckerState(1, &state, &pressVlaue, &error);
        printf("sucker1 state is %d, pressVlaue is %d, error num is %d\n", state, pressVlaue, error);
        robot.GetSuckerState(12, &state, &pressVlaue, &error);
        printf("sucker12 state is %d, pressVlaue is %d, error num is %d\n", state, pressVlaue, error);
        robot.Sleep(100);
    }

    //等待1号吸盘是否为吸附到物体的状态，等待时间100ms
    int ret = robot.WaitSuckerState(1, 1, 100);
    printf("WaitSuckerState result is  %d\n", ret);

    //单播模式关闭1号和12号吸盘
    ctrl[0] = 3;
    robot.SetSuckerCtrl(1, 1, ctrl);
    robot.SetSuckerCtrl(12, 1, ctrl);

    robot.CloseRPC();
}

int TestAxleGenCom()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    int led_on[6] = { 0xAB, 0xBA, 0x12, 0x01, 0x01, 0x79 };
    int led_off[6] = { 0xAB, 0xBA, 0x12, 0x01, 0x00, 0x78 };
    int version[5] = { 0xAB, 0xBA, 0x11, 0x00, 0x76 };
    int state[6] = { 0xAB, 0xBA, 0x1B, 0x01, 0xAA, 0x2B };
    int cycleState[6] = { 0xAB, 0xBA, 0x12, 0x01, 0x00, 0x78 };
    int rcvdata[16] = { 0 };
    int ret = 0;
    int cnt = 1;
    JointPos p1Joint(88.708, -86.178, 140.989, -141.825, -89.162, -49.879);
    DescPose p1Desc(188.007, -377.850, 260.207, 178.715, 2.823, -131.466);
    JointPos p2Joint(112.131, -75.554, 126.989, -139.027, -88.044, -26.477);
    DescPose p2Desc(368.003, -377.848, 260.211, 178.715, 2.823, -131.465);
    ExaxisPos exaxisPos(0, 0, 0, 0);
    DescPose offdese(0, 0, 0, 0, 0, 0);
    //开启末端透传功能
    robot.SetAxleGenComEnable(1);
    robot.SetAxleLuaEnable(1);
    while (cnt <= 10000)
    {
        //读取版本号
        ret = robot.SndRcvAxleGenComCmdData(5, version, 10, rcvdata);
        printf(" hard version : %d,hard code:%d, soft version:%d %d, soft code:%d \n", rcvdata[4], rcvdata[5], rcvdata[6], rcvdata[7], rcvdata[8]);
        if (ret != 0)
        {
            //break;
        }
        robot.Sleep(1000);
        //读取艾灸头在位状态
        ret = robot.SndRcvAxleGenComCmdData(6, state, 6, rcvdata);
        printf(" state : %d \n", rcvdata[4]);
        robot.Sleep(1000);
        //开启艾灸头激光
        ret = robot.SndRcvAxleGenComCmdData(6, led_on, 6, rcvdata);
        printf("led on rcv data is: %d, %d, %d, %d, %d, %d  \n", rcvdata[0], rcvdata[1], rcvdata[2], rcvdata[3], rcvdata[4], rcvdata[5]);
        robot.MoveJ(&p1Joint, &p1Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        robot.Sleep(4000);
        //关闭艾灸头激光
        ret = robot.SndRcvAxleGenComCmdData(6, led_off, 6, rcvdata);
        printf("led off rcv data is: %d, %d, %d, %d, %d, %d \n", rcvdata[0], rcvdata[1], rcvdata[2], rcvdata[3], rcvdata[4], rcvdata[5]);
        robot.MoveJ(&p2Joint, &p2Desc, 0, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
        robot.Sleep(1000);
        printf("***********************complate No. %d SDK test*****************************\n", cnt);
        cnt++;
    }
    robot.CloseRPC();
}
int TestCtrlOpenLuaOperate()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return 0;
    }
    robot.SetReConnectParam(true, 30000, 500);
    rtn = robot.OpenLuaUpload("D://zUP/openlua/CtrlDev_WELDING_A.lua");
    printf("OpenLuaUpload rtn is %d\n", rtn);
    robot.Sleep(2000);
    rtn = robot.OpenLuaUpload("D://zUP/openlua/CtrlDev_SWDPOLISH.lua");
    printf("OpenLuaUpload rtn is %d\n", rtn);
    robot.Sleep(2000);
    rtn = robot.OpenLuaDownload("CtrlDev_WELDING_A.lua", "D://zDOWN/");
    printf("OpenLuaDownload rtn is %d\n", rtn);
    robot.Sleep(5000);
    //rtn = robot.OpenLuaDownload("CtrlDev_SWDPOLISH.lua", "D://zDOWN/");
    //printf("OpenLuaDownload rtn is %d\n", rtn);
    rtn = robot.SetCtrlOpenLUAName(0, "CtrlDev_WELDING_A.lua");
    printf("SetCtrlOpenLUAName rtn is %d\n", rtn);
    robot.Sleep(2000);
    rtn = robot.SetCtrlOpenLUAName(1, "CtrlDev_SWDPOLISH.lua");
    printf("SetCtrlOpenLUAName rtn is %d\n", rtn);
    std::string name[4] = {};
    rtn = robot.GetCtrlOpenLUAName(name);
    printf("ctrl open lua names : %s, %s, %s, %s\n", name[0].c_str(), name[1].c_str(), name[2].c_str(), name[3].c_str());
    rtn = robot.LoadCtrlOpenLUA(1);
    printf("LoadCtrlOpenLUA rtn is %d\n", rtn);
    robot.Sleep(2000);
    rtn = robot.UnloadCtrlOpenLUA(1);
    printf("UnloadCtrlOpenLUA rtn is %d\n", rtn);
    rtn = robot.OpenLuaDelete("CtrlDev_WELDING_A.lua");
    printf("OpenLuaDelete rtn is %d\n", rtn);
    rtn = robot.AllOpenLuaDelete();
    printf("AllOpenLuaDelete rtn is %d\n", rtn);
    robot.CloseRPC();
    robot.Sleep(1000);
    return 0;
}

void TestLaserConfig()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    uint8_t ctrl[20];
    uint8_t state;
    int pressVlaue;
    int error;
    robot.CloseRPC();
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return;
    }
    robot.SetReConnectParam(true, 30000, 500);
    //设置IP地址和端口号
    robot.LaserTrackingSensorConfig("192.168.58.120", 502);
    //设置采样周期
    robot.LaserTrackingSensorSamplePeriod(20);
    //加载驱动
    robot.LoadPosSensorDriver(103);
    //关闭激光外设
    robot.LaserTrackingLaserOnOff(0, 0);
    robot.Sleep(3000);
    //打开激光外设
    robot.LaserTrackingLaserOnOff(1, 0);
    robot.Sleep(3000);
    robot.LaserTrackingLaserOnOff(0, 0);
    robot.Sleep(3000);
    //打开激光外设
    robot.LaserTrackingLaserOnOff(1, 0);
    robot.CloseRPC();
}

void TestLaserRecordAndReplay()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    uint8_t ctrl[20];
    uint8_t state;
    int pressVlaue;
    int error;
    robot.CloseRPC();
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return;
    }
    robot.SetReConnectParam(true, 30000, 500);

    //上传并加载开放协议文件
    //robot.OpenLuaUpload("D://zUP/CtrlDev_laser_ruiniu-0117.lua");
    //robot.Sleep(2000);
    //robot.SetCtrlOpenLUAName(0, "CtrlDev_laser_ruiniu-0117.lua");
    //robot.UnloadCtrlOpenLUA(0);
    //robot.LoadCtrlOpenLUA(0);
    //robot.Sleep(8000);
    int cnt = 1;
    while (cnt < 31)
    {
        //运动到扫描的起点
        JointPos startjointPos(58.830, -92.757, 86.939, -81.135, -90.548, 26.358);
        DescPose startdescPose(-74.319, -312.541, 39.168, 177.512, -1.843, 122.527);
        ExaxisPos exaxisPos(0, 0, 0, 0);
        DescPose offdese(0, 0, 0, 0, 0, 0);
        robot.MoveL(&startjointPos, &startdescPose, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 0, 0);
        //开始轨迹记录
        robot.LaserSensorRecord1(2, 10);
        //运动到需要记录的终点
        JointPos endjointPos(76.229, -78.219, 71.540, -82.615, -88.277, 42.332);
        DescPose enddescPose(17.298, -408.461, 40.967, 178.317, 0.798, 123.875);
        robot.MoveL(&endjointPos, &enddescPose, 1, 0, 30, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 0, 0);
        //停止记录
        robot.LaserSensorRecord1(0, 10);
        //运动到记录的焊缝起点
        robot.MoveToLaserRecordStart(1, 30);
        //开始轨迹复现
        robot.LaserSensorReplay(10, 100);
        robot.MoveLTR();
        //停止轨迹复现
        robot.LaserSensorRecord1(0, 10);
        printf("激光扫描+轨迹复现稳定性测试第%d次\n", cnt);
        cnt++;
    }
    robot.CloseRPC();
}

void TestLasertrack()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot = {};
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    robot.SetReConnectParam(true, 30000, 500);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return;
    }
    

    //上传并加载开放协议文件
    //robot.OpenLuaUpload("E://openlua/CtrlDev_laser_ruiniu-0117.lua");
    //robot.Sleep(2000);
    //robot.SetCtrlOpenLUAName(0, "CtrlDev_laser_ruiniu-0117.lua");
    //robot.UnloadCtrlOpenLUA(0);
    //robot.LoadCtrlOpenLUA(0);
    //robot.Sleep(8000);
    int cnt = 0;
    while (cnt < 2)
    {
        //运动到需要寻位的起始点
        JointPos startjointPos(58.830, -92.757, 86.939, -81.135, -90.548, 26.358);
        DescPose startdescPose(-74.319, -312.541, 39.168, 177.512, -1.843, 122.527);
        ExaxisPos exaxisPos(0, 0, 0, 0);
        DescPose offdese(0, 0, 0, 0, 0, 0);
        DescTran directionPoint;
        robot.MoveL(&startjointPos, &startdescPose, 1, 0, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);

        //沿着-y方向开始寻位
        int ret = robot.LaserTrackingSearchStart_xyz(0, 10, 300, 10000, 2);
        robot.LaserTrackingSearchStop();
        //如果寻位成功
        
        if (ret == 0)
        {
            //运动到寻位点
            robot.MoveToLaserSeamPos(1, 30, 0, 0, 0, offdese);
            printf("robot search end0\n");
            //开始沿着寻位点进行激光跟踪
            robot.LaserTrackingTrackOnOff(1, 2);
            JointPos endjointPos(76.229, -78.219, 71.540, -82.615, -88.277, 42.332);
            DescPose enddescPose(17.298, -408.461, 40.967, 178.317, 0.798, 123.875);
            robot.MoveL(&endjointPos, &enddescPose, 1, 0, 5, 100, 100, -1, &exaxisPos, 0, 0, &offdese, 1, 1);
            printf("robot search end1\n");
            //停止跟踪
            robot.LaserTrackingTrackOnOff(0, 2);

        }
        cnt++;
    }
    robot.CloseRPC();
}


void testLasertrackandExitAxis()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    uint8_t ctrl[20];
    uint8_t state;
    int pressVlaue;
    int error;
    robot.CloseRPC();
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");

    if (rtn != 0)
    {
        return;
    }
    robot.SetReConnectParam(true, 30000, 500);

    ExaxisPos startexaxisPos = { 0,0,0,0 };
    ExaxisPos seamexaxisPos = { 10,0,0,0 };
    ExaxisPos endexaxisPos = { 15, 0, 0, 0 };
    DescPose offdese = { 0, 0, 0, 0, 0, 0 };
    JointPos seamjointPos(0, 0, 0, 0, 0, 0);
    DescPose seamdescPose(0, 0, 0, 0, 0, 0);

    int cnt = 1;
    while (cnt < 2)
    {
        //运动到需要寻位的起始点
        JointPos startjointPos(58.830, -92.757, 86.939, -81.135, -90.548, 26.358);
        DescPose startdescPose(-74.319, -312.541, 39.168, 177.512, -1.843, 122.527);
        robot.ExtAxisSyncMoveJ(startjointPos, startdescPose, 1, 0, 100, 100, 100, startexaxisPos, -1, 0, offdese);

        //沿着-y方向开始寻位
        int ret = robot.LaserTrackingSearchStart_xyz(0, 100, 300, 1000, 2);
        robot.LaserTrackingSearchStop();
        int tool = 0;
        int user = 0;
        robot.GetLaserSeamPos(0, offdese, seamjointPos, seamdescPose, tool, user, startexaxisPos);
        printf("%f, %f, %f,%f, %f, %f,%f, %f, %f,%f, %f, %f\n", seamjointPos.jPos[0], seamjointPos.jPos[1], seamjointPos.jPos[2], seamjointPos.jPos[3], seamjointPos.jPos[4], seamjointPos.jPos[5], seamdescPose.tran.x, seamdescPose.tran.y, seamdescPose.tran.z, seamdescPose.rpy.rx, seamdescPose.rpy.ry, seamdescPose.rpy.rz);

        //如果寻位成功
        if (ret == 0)
        {
            //机器人和扩展轴同步运动到寻位点
            robot.ExtAxisSyncMoveJ(seamjointPos, seamdescPose, 1, 0, 100, 100, 100, seamexaxisPos, -1, 0, offdese);

            //开始沿着寻位点进行激光跟踪并与扩展轴同步运动
            robot.LaserTrackingTrackOnOff(1, 2);
            JointPos endjointPos(67.362, -88.180, 83.000, -81.723, -90.086, 34.885);
            DescPose enddescPose(-32.721, -341.899, 36.602, 177.508, -1.841, 122.519);
            robot.ExtAxisSyncMoveL(endjointPos, enddescPose, 1, 0, 20, 100, 100, -1, endexaxisPos, 0, offdese);;
            //停止跟踪
            robot.LaserTrackingTrackOnOff(0, 2);
        }
        cnt++;
        printf("扩展轴与机器人同步进行激光跟踪  第%d次\n", cnt);
    }
    robot.CloseRPC();
}


// Laser record replay + extended axis asynchronous motion + fixed‑point weave
void TestLaserRecordReplayExaxisWithWave()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot = {};
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    robot.SetReConnectParam(true, 30000, 500);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return;
    }

    JointPos startjointPos = JointPos(105.600, -65.393, -93.638, -79.687, 79.175, 134.046);
    DescPose startdescPose = DescPose(42.376, 533.597, 362.564, -34.957, -0.564, 169.063);
    JointPos endjointPos = JointPos(105.600, -65.393, -93.638, -79.687, 79.175, 134.046);
    DescPose enddescPose = DescPose(42.376, 533.597, 362.564, -34.957, -0.564, 169.063);
    DescPose offdese = DescPose(0, 0, 0, 0, 0, 0);
    ExaxisPos exaxis0Pos(0, 134.296, 0, 0);
    ExaxisPos exaxis1Pos(0, 74.335, 0, 0);
    // MoveJ to safe point(Exaxis 0,174.957,0,0)
    rtn = robot.MoveJ(&startjointPos, &startdescPose, 5, 0, 100, 100, 50, &exaxis0Pos, -1, 0, &offdese);
    printf("MoveJ start: %d\n", rtn);

    // Exaxis async move to start position 105.003
    rtn = robot.ExtAxisMove(exaxis1Pos, 50, -1);
    printf("ExtAxisMove 105.003: %d\n", rtn);

    robot.Sleep(3000);

    // MoveL to start point
    rtn = robot.MoveL(&endjointPos, &enddescPose, 5, 0, 100, 100, 50, -1, 0, &exaxis1Pos, 0, 0, &offdese, 100, 0, 0, 10);
    printf("MoveL end: %d\n", rtn);

    // Start laser record
    rtn = robot.LaserSensorRecord1(2, 10);
    printf("LaserSensorRecord start: %d\n", rtn);

    // Exaxis move to 174.957 during recording
    rtn = robot.ExtAxisMove(exaxis0Pos, 50, -1);
    printf("ExtAxisMove 174.957: %d\n", rtn);

    robot.Sleep(3000);

    // Stop laser record
    rtn = robot.LaserSensorRecord1(0, 10);
    printf("LaserSensorRecord stop: %d\n", rtn);

    // Exaxis move back to 105.003, MoveL return to start point
    rtn = robot.ExtAxisMove(exaxis1Pos, 50, -1);
    printf("ExtAxisMove back: %d\n", rtn);

    // MoveL to start point
    rtn = robot.MoveL(&endjointPos, &enddescPose, 5, 0, 100, 100, 50, -1, 0, &exaxis1Pos, 0, 0, &offdese, 100, 0, 0, 10);
    printf("MoveL back: %d\n", rtn);

    // PTP move to laser record weld start point
    rtn = robot.MoveToLaserRecordStart(0, 30);
    printf("MoveToLaserRecordStart: %d\n", rtn);

    // Start replay
    rtn = robot.LaserSensorRecord1(3, 10);
    printf("LaserSensorRecord replay: %d\n", rtn);

    // Start fixed‑point weave
    DescPose refPoint = DescPose(49.381, 533.608, 362.556, -34.961, -0.564, 169.062);
    rtn = robot.OriginPointWeaveStart(0, 1, refPoint, 5);
    printf("OriginPointWeaveStart: %d\n", rtn);

    // Exaxis move to 174.957 during weaving
    rtn = robot.ExtAxisMove(exaxis0Pos, 50, -1);
    printf("ExtAxisMove replay: %d\n", rtn);

    // Stop weave
    rtn = robot.OriginPointWeaveEnd();
    printf("OriginPointWeaveEnd: %d\n", rtn);

    // Stop replay
    rtn = robot.LaserSensorRecord1(0, 10);
    printf("LaserSensorRecord stop: %d\n", rtn);

    robot.CloseRPC();
    robot.Sleep(1000);
}

// Laser record replay + normal weave
void TestLaserReproduceNormalWeave()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot = {};
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    robot.SetReConnectParam(true, 30000, 500);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return;
    }

    JointPos startjointPos = JointPos(68.930, -70.217, -121.821, -75.522, 91.216, 137.289);
    DescPose startdescPose = DescPose(216.097, 304.517, 34.164, -3.871, 0.792, 132.864);
    JointPos endjointPos = JointPos(58.092, -78.843, -115.569, -73.420, 91.662, 126.457);
    DescPose enddescPose = DescPose(296.276, 307.453, 34.803, -3.868, 0.783, 132.864);
    DescPose offdese = DescPose(0, 0, 0, 0, 0, 0);
    ExaxisPos exaxis0Pos(0, 174.957, 0, 0);
    ExaxisPos exaxis1Pos(0, 105.003, 0, 0);

    robot.Sleep(1000);

    // MoveL to start position(Exaxis 0,174.957,0,0)
    rtn = robot.MoveL(&startjointPos, &startdescPose, 5, 0, 100, 100, 100, -1, 0, &exaxis0Pos, 0, 0, &offdese, 100, 0, 0, 10);
    printf("MoveL start: %d\n", rtn);

    // Start laser record
    rtn = robot.LaserSensorRecord1(2, 10);
    printf("LaserSensorRecord start: %d\n", rtn);

    // MoveL to end position
    rtn = robot.MoveL(&endjointPos, &enddescPose, 5, 0, 100, 100, 100, -1, 0, &exaxis0Pos, 0, 0, &offdese, 100, 0, 0, 10);
    printf("MoveL end: %d\n", rtn);

    // Stop laser record
    rtn = robot.LaserSensorRecord1(0, 10);
    printf("LaserSensorRecord stop: %d\n", rtn);

    // MoveL return to start position
    rtn = robot.MoveL(&startjointPos, &startdescPose, 5, 0, 100, 100, 100, -1, 0, &exaxis0Pos, 0, 0, &offdese, 100, 0, 0, 10);
    printf("MoveL back: %d\n", rtn);

    // LIN move to laser record weld start point
    rtn = robot.MoveToLaserRecordStart(1, 30);
    printf("MoveToLaserRecordStart: %d\n", rtn);

    // Start normal weave
    rtn = robot.WeaveStart(0);
    printf("WeaveStart: %d\n", rtn);

    // Start record replay
    rtn = robot.LaserSensorRecord1(3, 10);
    printf("LaserSensorRecord replay: %d\n", rtn);

    // Laser track replay motion
    rtn = robot.MoveLTR();
    printf("MoveLTR: %d\n", rtn);

    robot.Sleep(3000);

    // Stop record replay
    rtn = robot.LaserSensorRecord1(0, 10);
    printf("LaserSensorRecord stop: %d\n", rtn);

    // Stop normal weave
    rtn = robot.WeaveEnd(0);
    printf("WeaveEnd: %d\n", rtn);

    robot.CloseRPC();
    robot.Sleep(1000);
}



void TestFieldBusBoard()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    uint8_t type = 0, version = 0, connState = 0;
    uint8_t ctrl[8] = {0};
    double ctrlAO[8] = {0.0};
    uint8_t DI[8] = {0};
    double AI[8] = {0.0};
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return;
    }
    robot.SetReConnectParam(true, 30000, 500);
    //上传并加载开放协议文件
    //robot.OpenLuaUpload("E://项目/外设SDK/CtrlDev_field.lua");
    //robot.Sleep(2000);
    //robot.SetCtrlOpenLUAName(3, "CtrlDev_field.lua");
    //robot.UnloadCtrlOpenLUA(3);
    //robot.LoadCtrlOpenLUA(3);
    //robot.Sleep(8000);

    //获取从站板卡的协议类型、软件版本、与PLC的连接状态
    robot.GetFieldBusConfig(&type, &version, &connState);
    printf("type is %d, version is %d,connState is %d\n", type, version, connState);

    //写入DO0 = 1、DO1 = 0、DO2 = 1
    ctrl[0] = 0;
    ctrl[1] = 1;
    ctrl[2] = 1;
    robot.FieldBusSlaveWriteDO(0, 3, ctrl);

    //写入AO2 = 0x1000
    ctrlAO[0] = 0x1005;
    robot.FieldBusSlaveWriteAO(2, 1, ctrlAO);

    //循环监控DI0~DI3 AI0~AI2
    for (int i = 0; i < 100; i++)
    {
        robot.FieldBusSlaveReadDI(0, 4, DI);
        printf("DI0 is %d, DI1 is %d,DI2 is %d,DI3 is %d\n\n", DI[0], DI[1], DI[2], DI[3]);
        robot.FieldBusSlaveReadAI(0, 3, AI);
        printf("AI0 is %f, AI1 is %f,AI2 is %f\n\n", AI[0], AI[1], AI[2]);
        robot.Sleep(100);
    }

    //等待DI0是否为1，等待时间100ms，并打印结果
    int ret = robot.FieldBusSlaveWaitDI(0, 1, 100);
    printf("FieldBusSlaveWaitDI result is  %d\n", ret);

    //等待AI0是否大于400，等待时间100ms，并打印结果
    ret = robot.FieldBusSlaveWaitAI(0, 0, 400.00, 100);
    printf("FieldBusSlaveWaitAI result is  %d\n", ret);

    robot.CloseRPC();
}

int TsetSmarttoolState(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;

    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    robot.SetReConnectParam(true, 30000, 500);

    while (true)
    {
        int btn = 0;
        robot.GetSmarttoolBtnState(btn);
        cout << "smarttool " << std::bitset<sizeof(btn) * 8>(btn) << endl;

        Sleep(100);
    }
}


int TestAxleLua(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    robot.AxleLuaUpload("D://zUP/AXLE_LUA_End_DaHuan.lua");
    AxleComParam param(7, 8, 1, 0, 5, 3, 1);
    robot.SetAxleCommunicationParam(param);
    AxleComParam getParam;
    robot.GetAxleCommunicationParam(&getParam);
    printf("GetAxleCommunicationParam param is %d %d %d %d %d %d %d\n", getParam.baudRate, getParam.dataBit, getParam.stopBit, getParam.verify, getParam.timeout, getParam.timeoutTimes, getParam.period);
    robot.SetAxleLuaEnable(1);
    int luaEnableStatus = 0;
    robot.GetAxleLuaEnableStatus(&luaEnableStatus);
    robot.SetAxleLuaEnableDeviceType(0, 1, 0, 0);
    int forceEnable = 0;
    int gripperEnable = 0;
    int ioEnable = 0;
    int dexhandEnable = 0;
    robot.GetAxleLuaEnableDeviceType(&forceEnable, &gripperEnable, &ioEnable, &dexhandEnable);
    printf("GetAxleLuaEnableDeviceType param is %d %d %d %d\n", forceEnable, gripperEnable, ioEnable, dexhandEnable);
    int func[32] = { 0,1,1,1,1,0,1,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
    rtn = robot.SetAxleLuaGripperFunc(2, func);
    printf("SetAxleLuaGripperFunc rtn is %d\n", rtn);
    int getFunc[32] = { 0 };
    rtn = robot.GetAxleLuaGripperFunc(2, getFunc);
    printf("GetAxleLuaGripperFunc rtn is %d\n", rtn);
    int getforceEnable[16] = { 0 };
    int getgripperEnable[16] = { 0 };
    int getioEnable[16] = { 0 };
    int dexhandEnable1[16] = { 0 };
    robot.GetAxleLuaEnableDevice(getforceEnable, getgripperEnable, getioEnable, dexhandEnable1);
    printf("\ngetforceEnable status : ");
    for (int i = 0; i < 16; i++)
    {
        printf("%d,", getforceEnable[i]);
    }
    printf("\ngetgripperEnable status : ");
    for (int i = 0; i < 16; i++)
    {
        printf("%d,", getgripperEnable[i]);
    }
    printf("\ngetioEnable status : ");
    for (int i = 0; i < 16; i++)
    {
        printf("%d,", getioEnable[i]);
    }
    printf("\n");
    robot.ActGripper(2, 0);
    robot.Sleep(2000);
    robot.ActGripper(2, 1);
    robot.Sleep(2000);
    robot.MoveGripper(2, 1, 10, 100, 50000, 0, 0, 0, 0, 0);
    int pos = 0;
    while (true)
    {
        robot.GetRobotRealTimeState(&pkg);
        printf("gripper pos is %u\n", pkg.gripper_position);
        robot.Sleep(100);
    }
    robot.CloseRPC();
    return 0;
}

int TestDexterousHands()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;

    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);

    int id = 1;               // 从站号
    int slaveNum = 4;         // 控制4个手指
    int max_time = 8000;      // 最大等待时间 8秒
    int speed[16] = { 0 };      // 速度数组，全0表示使用默认速度
    int force[16] = { 0 };      // 力矩数组

    // 初始化力矩数组：前4个手指设为50%，其余为0（通过Move指令下发数值）
    for (int i = 0; i < 16; i++)
    {
        force[i] = (i < 4) ? 50 : 0;
    }

    // 辅助函数：设置位置数组（前4个手指有效）
    double pos[16] = { 0.0 };


    JointPos j1(-91.876, -85.920, 109.279, -86.239, -96.664, -28.563);
    JointPos j2(-40.954, -85.920, 109.279, -86.239, -96.664, -28.563);
    ExaxisPos epos(0, 0, 0, 0);
    DescPose offset_pos(0, 0, 0, 0, 0, 0);

    printf("===== 灵巧手完整功能测试开始 =====\n");

    // 1. 清除错误
    int ret = robot.ClearDexterousHandsError();
    printf("ClearDexterousHandsError rtn %d\n", ret);

    // ========== 2. 设置功能开关 ==========
    int setFunc[32] = { 0 };
    setFunc[2] = 1;   // 启用位置设置功能
    setFunc[4] = 1;   // 启用力矩设置功能
    setFunc[9] = 1;   // 读位置
    setFunc[10] = 1;   // 读力矩
    setFunc[11] = 1;   // 读状态
    setFunc[22] = 1;   // 单轴运动状态

    ret = robot.SetDexterousHandsFunc(id, setFunc);
    printf("SetDexterousHandsFunc(使能+初始化+位置/速度/力矩功能启用) rtn %d\n", ret);

    // ========== 3. 读取功能状态（验证设置是否生效） ==========
    int getFunc[32] = { 0 };  // GetDexterousHandsFunc 返回32个整数
    ret = robot.GetDexterousHandsFunc(id, getFunc);
    printf("GetDexterousHandsFunc rtn %d\n", ret);
    if (ret == 0)
    {
        // 打印全部32个数值
        printf("GetDexterousHandsFunc 返回的全部32个数值:");
        for (int i = 0; i < 32; i++)
        {
            printf("  [%d]={%d}", i, getFunc[i]);
            if ((i + 1) % 8 == 0)
            {
                printf("\n");
            }
            else if (i < 31)
            {
                printf(", ");
            }
        }
    }

    // ========== 4. 激活灵巧手 ==========
    ret = robot.SetDexterousHandsAct(id, 1);
    printf("SetDexterousHandsAct(激活) rtn %d\n", ret);
    if (ret != 0)
    {
        printf("激活失败，测试中止");
        return -1;
    }

    // ========== 5. 初始移动到 20°（通过Move指令下发位置和力矩数值） ==========
    memset(pos, 0, sizeof(pos));
    pos[0] = 20;
    pos[1] = 20;
    pos[2] = 20;
    pos[3] = 20;
    ret = robot.SetDexterousHandsMove(id, slaveNum, pos, speed, force, max_time);
    printf("初始移动 20° -> %d\n", ret);
    robot.Sleep(5000);
    // ========== 6. 往复运动10次（10° ↔ 50°） ==========
    printf("开始往复运动10次...");
    for (int iteration = 1; iteration <= 10; iteration++)
    {
        robot.MoveJ(&j1, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        memset(pos, 0, sizeof(pos));
        pos[0] = 10;
        pos[1] = 10;
        pos[2] = 10;
        pos[3] = 10;
        ret = robot.SetDexterousHandsMove(id, slaveNum, pos, speed, force, max_time);
        printf("移动到 10° -> %d\n", ret);
        robot.Sleep(1000);

        robot.MoveJ(&j2, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);

        memset(pos, 0, sizeof(pos));
        pos[0] = 50;
        pos[1] = 50;
        pos[2] = 50;
        pos[3] = 50;
        ret = robot.SetDexterousHandsMove(id, slaveNum, pos, speed, force, max_time);
        printf("移动到 50° -> %d\n", ret);
        robot.Sleep(1000);
    }

    printf("测试完成（功能开关设置/读取 + 激活 + 10次往复运动）。");
    return 0;
}


/**
 * @brief 五指灵巧手完整功能测试
 *
 * 测试流程：
 * 1. 初始化机器人连接并配置参数
 * 2. 设置灵巧手功能开关（使能、初始化、位置/速度/力矩控制、多轴同步）
 * 3. 读取并验证功能开关状态
 * 4. 激活灵巧手
 * 5. 执行往复运动测试：机器人交替移动至两个位姿，灵巧手在 A/B/C 三组目标位置间切换
 *
 * @return int 0-测试成功，-1-连接失败，-2-激活失败
 */
int TestFiveDexterousHands()
{
    // ==================== 1. 初始化与连接 ====================
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;

    robot.LoggerInit();
    robot.SetLoggerLevel(1);

    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;  // 连接失败
    }
    robot.SetReConnectParam(true, 30000, 500);

    // ==================== 2. 运动参数配置 ====================
    const int DEXTEROUS_ID = 1;          // 从站号（灵巧手）
    const int FINGER_COUNT = 12;          // 控制手指数量
    const int MOVE_TIMEOUT_MS = 12000;    // 单次运动最大等待时间（毫秒）

    // 速度/力矩数组（12个手指，后4个留空）
    int speed[16] = { 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 0, 0, 0, 0 };
    int force[16] = { 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 0, 0, 0, 0 };

    // 三组目标位置（角度）
    double posA[16] = { 5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5,  5, 0, 0, 0, 0 };
    double posB[16] = { 60, 10, 70, 30, 70, 70, 10, 10, 10, 10, 10, 10, 0, 0, 0, 0 };
    double posC[16] = { 50, 50, 20, 20, 0,  0,  0,  0,  70, 70, 70, 70, 0, 0, 0, 0 };

    // 机器人两个测试位姿

    JointPos j1(-172.132, -90.455, -102.422, -67.864, 95.273, -21.129);
    JointPos j2(-173.180, -106.578, -83.661, -70.600, 95.440, -22.167);

    ExaxisPos epos(0, 0, 0, 0);
    DescPose offset_pos(0, 0, 0, 0, 0, 0);

    printf("===== 灵巧手完整功能测试开始 =====\n");

    // ==================== 3. 清除错误状态 ====================
    rtn = robot.ClearDexterousHandsError();
    printf("[清除错误] rtn = %d\n", rtn);

    // ==================== 4. 设置功能开关 ====================
    int setFuncA[32] = { 0 };
    setFuncA[2] = 1;
    setFuncA[3] = 1;
    setFuncA[4] = 1;
    setFuncA[9] = 1;
    setFuncA[10] = 1;
    setFuncA[11] = 1;
    setFuncA[20] = 1;//多轴同步运动
    setFuncA[22] = 1;

    int setFuncB[32] = { 0 };
    setFuncB[2] = 1;
    setFuncB[3] = 1;
    setFuncB[4] = 1;
    setFuncB[9] = 1;
    setFuncB[10] = 1;
    setFuncB[11] = 1;
    setFuncB[22] = 1;

    // 主站设置功能 A
    rtn = robot.SetDexterousHandsFunc(DEXTEROUS_ID, setFuncA);
    printf("[设置主站功能] rtn = %d\n", rtn);

    // 从站（手指2~12）设置功能 B
    for (int i = 2; i <= FINGER_COUNT; i++)
    {
        rtn = robot.SetDexterousHandsFunc(i, setFuncB);
    }
    printf("[设置从站功能（2~12）] rtn = %d\n", rtn);

    // ==================== 5. 读取并验证功能状态 ====================
    int getFunc[32] = { 0 };
    rtn = robot.GetDexterousHandsFunc(DEXTEROUS_ID, getFunc);
    printf("[读取功能状态] rtn = %d\n", rtn);

    if (rtn == 0)
    {
        printf("功能开关状态（32位）:\n  ");
        for (int i = 0; i < 32; i++)
        {
            printf("[%d]=%d", i, getFunc[i]);
            if ((i + 1) % 8 == 0 && i < 31)
                printf("\n  ");
            else if (i < 31)
                printf(", ");
        }
        printf("\n");
    }

    // ==================== 6. 激活灵巧手 ====================
    rtn = robot.SetDexterousHandsAct(DEXTEROUS_ID, 1);
    printf("[激活灵巧手] rtn = %d\n", rtn);
    if (rtn != 0)
    {
        printf("激活失败，测试中止\n");
        return -2;
    }

    robot.Sleep(5000);  // 等待激活完成

    // ==================== 7. 往复运动测试（10次循环） ====================
    printf("\n开始往复运动测试（共10次循环）...\n");
    printf("  位姿1: j1（左）  位姿2: j2（右）\n");
    printf("  手指目标: A→B→A→C（每组4个动作）\n\n");

    for (int iteration = 1; iteration <= 10; iteration++)
    {
        printf("--- 第 %2d 次循环 ---\n", iteration);

        // 动作1：移至 j1 + 手指 A
        robot.MoveJ(&j1, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        rtn = robot.SetDexterousHandsMove(DEXTEROUS_ID, FINGER_COUNT, posA, speed, force, MOVE_TIMEOUT_MS);
        printf("  j1 + posA → %d\n", rtn);
        robot.Sleep(1000);

        // 动作2：移至 j2 + 手指 B
        robot.MoveJ(&j2, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        rtn = robot.SetDexterousHandsMove(DEXTEROUS_ID, FINGER_COUNT, posB, speed, force, MOVE_TIMEOUT_MS);
        printf("  j2 + posB → %d\n", rtn);
        robot.Sleep(1000);

        // 动作3：移至 j1 + 手指 A
        robot.MoveJ(&j1, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        rtn = robot.SetDexterousHandsMove(DEXTEROUS_ID, FINGER_COUNT, posA, speed, force, MOVE_TIMEOUT_MS);
        printf("  j1 + posA → %d\n", rtn);
        robot.Sleep(1000);

        // 动作4：移至 j2 + 手指 C
        robot.MoveJ(&j2, 0, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
        rtn = robot.SetDexterousHandsMove(DEXTEROUS_ID, FINGER_COUNT, posC, speed, force, MOVE_TIMEOUT_MS);
        printf("  j2 + posC → %d\n", rtn);
        robot.Sleep(1000);
    }

    // ==================== 8. 测试完成 ====================
    printf("\n===== 测试完成 =====\n");
    printf("  功能开关设置/读取  ✓\n");
    printf("  灵巧手激活        ✓\n");
    printf("  10次往复运动      ✓\n");
    return 0;
}

int TestGripperWaitMotionDone()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    robot.SetReConnectParam(true, 30000, 500);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    for (int i = 0; i < 10; i++)
    {
        rtn = robot.MoveGripper(1, 0, 10, 100, 30000, 0, 0, 0, 0, 0);
        printf("MoveGripper(Open) ret=%d\n", rtn);
        rtn = robot.GripperWaitMotionDone(1, 10000, 0, 0);
        printf("GripperWaitMotionDone(Wait for motion completed without object detection) ret=%d\n", rtn);
        rtn = robot.MoveGripper(1, 90, 10, 100, 30000, 0, 0, 0, 0, 0);
        printf("MoveGripper(Close) ret=%d\n", rtn);
        rtn = robot.GripperWaitMotionDone(1, 10000, 0, 0);
        printf("GripperWaitMotionDone(Wait for motion completed without object detection) ret=%d\n", rtn);
    }
    robot.Sleep(1000);
    robot.CloseRPC();
    robot.Sleep(1000);
}


#pragma endregion
#pragma region 机器人力控
int TestFTInit(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    int company = 22;
    int device = 0;
    int softversion = 0;
    int bus = 1;
    int index = 1;
    robot.FT_SetConfig(company, device, softversion, bus);
    robot.Sleep(1000);
    robot.FT_GetConfig(&company, &device, &softversion, &bus);
    printf("FT config:%d,%d,%d,%d\n", company, device, softversion, bus);
    robot.Sleep(1000);
    robot.FT_Activate(0);
    robot.Sleep(1000);
    robot.FT_Activate(1);
    robot.Sleep(1000);
    robot.Sleep(1000);
    robot.FT_SetZero(0);
    robot.Sleep(1000);
    ForceTorque ft;
    memset(&ft, 0, sizeof(ForceTorque));
    robot.FT_GetForceTorqueOrigin(0, &ft);
    printf("ft origin:%f,%f,%f,%f,%f,%f\n", ft.fx, ft.fy, ft.fz, ft.tx, ft.ty, ft.tz);
    //robot.FT_SetZero(1);
    //robot.Sleep(1000);
    DescPose ftCoord = {};
    robot.FT_SetRCS(0, ftCoord);
    robot.SetForceSensorPayload(0.824);
    robot.SetForceSensorPayloadCog(0.778, 2.554, 48.765);
    double weight = 0;
    double x = 0, y = 0, z = 0;
    robot.GetForceSensorPayload(weight);
    robot.GetForceSensorPayloadCog(x, y, z);
    printf("the FT load is %lf, %lf %lf %lf\n", weight, x, y, z);
    robot.SetForceSensorPayload(0);
    robot.SetForceSensorPayloadCog(0, 0, 0);
    double computeWeight = 0;
    DescTran tran = {};
    robot.ForceSensorAutoComputeLoad(weight, tran);
    cout << "the result is weight " << weight << " pos is " << tran.x << " " << tran.y << " " << tran.z << endl;
    robot.CloseRPC();
    return 0;
}

int TestFTLoadCompute(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    int company = 22;
    int device = 0;
    int softversion = 0;
    int bus = 1;
    int index = 1;
    robot.FT_SetConfig(company, device, softversion, bus);
    robot.Sleep(1000);
    robot.FT_GetConfig(&company, &device, &softversion, &bus);
    printf("FT config:%d,%d,%d,%d\n", company, device, softversion, bus);
    robot.Sleep(1000);
    robot.FT_Activate(0);
    robot.Sleep(1000);
    robot.FT_Activate(1);
    robot.Sleep(1000);
    robot.Sleep(1000);
    robot.FT_SetZero(0);
    robot.Sleep(1000);
    ForceTorque ft;
    memset(&ft, 0, sizeof(ForceTorque));
    robot.FT_GetForceTorqueOrigin(0, &ft);
    printf("ft origin:%f,%f,%f,%f,%f,%f\n", ft.fx, ft.fy, ft.fz, ft.tx, ft.ty, ft.tz);
    robot.FT_SetZero(1);
    robot.Sleep(1000);
    DescPose tcoord = {};
    tcoord.tran.z = 35.0;
    robot.SetToolCoord(11, &tcoord, 1, 0, 0, 0);
    robot.FT_PdIdenRecord(11);
    robot.Sleep(1000);
    float weight = 0.0;
    robot.FT_PdIdenCompute(&weight);
    printf("payload weight:%f\n", weight);
    DescPose desc_p1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
    DescPose desc_p2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
    DescPose desc_p3(-327.622, 402.230, 320.402, -178.067, 2.127, -46.207);
    robot.MoveCart(&desc_p1, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
    robot.Sleep(1000);
    robot.FT_PdCogIdenRecord(11, 1);
    robot.MoveCart(&desc_p2, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
    robot.Sleep(1000);
    robot.FT_PdCogIdenRecord(11, 2);
    robot.MoveCart(&desc_p3, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
    robot.Sleep(1000);
    robot.FT_PdCogIdenRecord(11, 3);
    robot.Sleep(1000);
    DescTran cog;
    memset(&cog, 0, sizeof(DescTran));
    robot.FT_PdCogIdenCompute(&cog);
    printf("cog:%f,%f,%f\n", cog.x, cog.y, cog.z);
    robot.CloseRPC();
    return 0;
}

int TestFTGuard(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    int company = 22;
    int device = 0;
    int softversion = 0;
    int bus = 1;
    int index = 1;
    robot.FT_SetConfig(company, device, softversion, bus);
    robot.Sleep(1000);
    robot.FT_GetConfig(&company, &device, &softversion, &bus);
    printf("FT config:%d,%d,%d,%d\n", company, device, softversion, bus);
    robot.Sleep(1000);
    robot.FT_Activate(0);
    robot.Sleep(1000);
    robot.FT_Activate(1);
    robot.Sleep(1000);
    robot.Sleep(1000);
    robot.FT_SetZero(0);
    robot.Sleep(1000);
    robot.FT_SetZero(1);
    uint8_t sensor_id = 1;
    uint8_t select[6] = { 1,1,1,1,1,1 };
    float max_threshold[6] = { 20.0,20.0,20.0,20.0,20.0,20.0 };
    float min_threshold[6] = { 5.0,5.0,5.0,5.0,5.0,5.0 };
    ForceTorque ft = {0.0};
    DescPose desc_p1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
    DescPose desc_p2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);
    DescPose desc_p3(-327.622, 402.230, 320.402, -178.067, 2.127, -46.207);
    robot.FT_Guard(1, sensor_id, select, &ft, max_threshold, min_threshold);
    robot.MoveCart(&desc_p1, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
    robot.MoveCart(&desc_p2, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
    robot.MoveCart(&desc_p3, 0, 0, 100.0, 100.0, 100.0, -1.0, -1);
    robot.FT_Guard(0, sensor_id, select, &ft, max_threshold, min_threshold);
    robot.CloseRPC();
    return 0;
}

int TestFTControlWithAdjustCoeff(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    uint8_t sensor_id = 1;
    uint8_t select[6] = { 0,0,1,0,0,0 };
    float ft_pid[6] = { 0.0008, 0.0, 0.0, 0.0, 0.0, 0.0 };
    uint8_t adj_sign = 0;
    uint8_t ILC_sign = 0;
    float max_dis = 1000.0;
    float max_ang = 20;
    ForceTorque ft = { 0.0 };
    ExaxisPos epos(0, 0, 0, 0);
    JointPos j1(80.765, -98.795, 106.548, -97.734, -89.999, 94.842);
    JointPos j2(43.067, -84.429, 92.620, -98.175, -90.011, 57.144);
    DescPose desc_p1(5.009, -547.463, 262.053, -179.999, -0.019, 75.923);
    DescPose desc_p2(-347.966, -547.463, 262.048, -180.000, -0.019, 75.923);
    DescPose offset_pos(0, 0, 0, 0, 0, 0);
    double M[2] = { 2.0, 2.0 };
    double B[2] = { 15.0, 15.0 };
    double threshold[2] = { 1.0, 1.0 };
    double adjustCoeff[2] = { 1.0, 0.8 };
    double polishRadio = 0.0;
    int filter_Sign = 0;
    int posAdapt_sign = 1;
    int isNoBlock;
    ft.fz = -10.0;
    robot.MoveL(&desc_p1, 1, 0, 100, 100, 100, -1, 0, &epos, 0, 0, &offset_pos);
    robot.MoveL(&desc_p2, 1, 0, 100, 100, 100, -1, 0, &epos, 0, 0, &offset_pos);
    while (true)
    {
        rtn = robot.FT_Control(1, sensor_id, select, &ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, M, B, threshold, adjustCoeff, 0, 0, 1, 0);
        printf("FT_Control start rtn is %d\n", rtn);
        robot.MoveL(&desc_p1, 1, 0, 100, 100, 100, -1, 0, &epos, 0, 0, &offset_pos);
        robot.MoveL(&desc_p2, 1, 0, 100, 100, 100, -1, 0, &epos, 0, 0, &offset_pos);
        rtn = robot.FT_Control(0, sensor_id, select, &ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, M, B, threshold, adjustCoeff, 0, 0, 1, 0);
        printf("FT_Control end rtn is %d\n", rtn);
        robot.Sleep(1000);
    }
    robot.CloseRPC();
    return 0;
}

int TestFTSearch(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;

    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);

    int company = 24;
    int device = 0;
    int softversion = 0;
    int bus = 1;
    int index = 1;

    robot.FT_SetConfig(company, device, softversion, bus);
    robot.Sleep(1000);
    robot.FT_GetConfig(&company, &device, &softversion, &bus);
    printf("FT config:%d,%d,%d,%d\n", company, device, softversion, bus);
    robot.Sleep(1000);

    robot.FT_Activate(0);
    robot.Sleep(1000);
    robot.FT_Activate(1);
    robot.Sleep(1000);

    robot.Sleep(1000);
    robot.FT_SetZero(0);
    robot.Sleep(1000);

    //恒力参数
    uint8_t status = 1;  //恒力控制开启标志，0-关，1-开
    int sensor_num = 1; //力传感器编号
    float gain[6] = { 0.0001,0.0,0.0,0.0,0.0,0.0 };  //最大阈值
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
    float vmax = 3.0; //线速度最大值，单位mm/s

    //直线插入参数
    float force_goal = 20.0;  //力或力矩阈值（0~100），单位N或Nm
    float lin_v = 0.0; //直线速度，单位mm/s
    float lin_a = 0.0; //直线加速度，单位mm/s^2,暂不使用
    float disMax = 100.0; //最大插入距离，单位mm
    uint8_t linorn = 1; //插入方向，1-正方向，2-负方向

    //旋转插入参数
    float angVelRot = 2.0;  //旋转角速度，单位°/s
    float forceInsertion = 1.0; //力或力矩阈值（0~100），单位N或Nm
    int angleMax = 45; //最大旋转角度，单位°
    uint8_t orn = 1; //力的方向，1-fz,2-mz
    float angAccmax = 0.0; //最大旋转角加速度，单位°/s^2,暂不使用
    uint8_t rotorn = 1; //旋转方向，1-顺时针，2-逆时针

    uint8_t select1[6] = { 0,0,1,1,1,0 }; //六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
    ft.fz = -10.0;
    robot.FT_Control(status, sensor_num, select1, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
    rtn = robot.FT_SpiralSearch(rcs, dr, fFinish, t, vmax);
    printf("FT_SpiralSearch rtn is %d\n", rtn);
    status = 0;
    robot.FT_Control(status, sensor_num, select1, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);

    uint8_t select2[6] = { 1,1,1,0,0,0 };  //六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
    gain[0] = 0.00005;
    ft.fz = -30.0;
    status = 1;
    robot.FT_Control(status, sensor_num, select2, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
    rtn = robot.FT_LinInsertion(rcs, force_goal, lin_v, lin_a, disMax, linorn);
    printf("FT_LinInsertion rtn is %d\n", rtn);
    status = 0;
    robot.FT_Control(status, sensor_num, select2, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);

    uint8_t select4[6] = { 1,1,1,0,0,0 };  //六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
    ft.fz = -30.0;
    status = 1;
    robot.FT_Control(status, sensor_num, select4, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
    rtn = robot.FT_LinInsertion(rcs, force_goal, lin_v, lin_a, disMax, linorn);
    printf("FT_LinInsertion rtn is %d\n", rtn);
    status = 0;
    robot.FT_Control(status, sensor_num, select4, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);

    robot.CloseRPC();
    return 0;
}

int TestRotInsert()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;

    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);

    float forceInsertion = 5.0; //力或力矩阈值（0~100），单位N或Nm
    int angleMax = 300; //最大旋转角度，单位°
    uint8_t orn = 1; //力的方向，1-fz,2-mz
    float angAccmax = 0; //最大旋转角加速度，单位°/s^2,暂不使用
    uint8_t status = 1;  //恒力控制开启标志，0-关，1-开
    int sensor_num = 11; //力传感器编号
    float gain[6] = { 0.0001,0.0,0.0,0.0,0.0,0.0 };  //最大阈值
    uint8_t adj_sign = 0;  //自适应启停状态，0-关闭，1-开启
    uint8_t ILC_sign = 0;  //ILC控制启停状态，0-停止，1-训练，2-实操
    float max_dis = 1000.0;  //最大调整距离
    float max_ang = 20.0;  //最大调整角度
    ForceTorque ft;
    memset(&ft, 0, sizeof(ForceTorque));
    int rcs = 0;  //参考坐标系，0-工具坐标系，1-基坐标系
    float angVelRot = 1.0;  //旋转角速度，单位°/s
    uint8_t rotorn = 1; //旋转方向，1-顺时针，2-逆时针

    JointPos j1(113.254, -101.309, 123.007, -111.046, -90.836, 76.24);
    //JointPos j2();
    DescPose desc_p1(244.184, -313.59, 321.577, -179.338, 0.005, 127.009);
   // DescPose desc_p2(328.795, 339.109, 373.605, 179.993, 0.005, 0.079);
    ExaxisPos epos = { 0.0, 0.0, 0.0, 0.0 };
    DescPose offset_pos(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    robot.MoveL(&j1, &desc_p1, 2, 0, 100.0, 180.0, 100.0, -1.0, &epos, 0, 1, &offset_pos);
    uint8_t select3[6] = { 0,0,1,0,0,0 };  //六个自由度选择[fx,fy,fz,mx,my,mz]，0-不生效，1-生效
    ft.fz = -5.0;
    gain[0] = 0.0001;
    status = 1;
    robot.FT_Control(1, sensor_num, select3, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
    rtn = robot.FT_LinInsertion(rcs, 10, 1, 1, 100, 1);
    printf("FT_LinInsertion rtn is %d\n", rtn);
    robot.FT_Control(0, sensor_num, select3, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);

    ft.fz = -30.0;
    robot.FT_Control(1, sensor_num, select3, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
    rtn = robot.FT_RotInsertion(rcs, angVelRot, forceInsertion, angleMax, orn, angAccmax, rotorn, 0);
    printf("FT_RotInsertion rtn is %d\n", rtn);
    robot.FT_Control(0, sensor_num, select3, &ft, gain, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);

    rtn = robot.FT_LinInsertion(0, 20, 3, 0, 20, 1);

    //robot.MoveL(&j2, &desc_p2, 0, 0, 100.0, 180.0, 100.0, -1.0, &epos, 0, 0, &offset_pos);
    robot.Sleep(1000);
    robot.GetRobotRealTimeState(&pkg);
    printf("robot errcode %d  %d\n", pkg.main_code, pkg.sub_code);
    robot.CloseRPC();
    return 0;
}

int TestSurface(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;

    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);

    int company = 22;
    int device = 0;
    int softversion = 0;
    int bus = 1;
    int index = 1;

    robot.FT_SetConfig(company, device, softversion, bus);
    robot.Sleep(1000);
    robot.FT_GetConfig(&company, &device, &softversion, &bus);
    printf("FT config:%d,%d,%d,%d\n", company, device, softversion, bus);
    robot.Sleep(1000);

    robot.FT_Activate(0);
    robot.Sleep(1000);
    robot.FT_Activate(1);
    robot.Sleep(1000);

    robot.Sleep(1000);
    robot.FT_SetZero(0);
    robot.Sleep(1000);

    int rcs = 0;
    uint8_t dir = 1;
    uint8_t axis = 1;
    float lin_v = 15.0;
    float lin_a = 0.0;
    float maxdis = 500.0;
    float ft_goal = 2.0;
    DescPose desc_pos(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
    DescPose xcenter(0, 0, 0, 0, 0, 0);
    DescPose ycenter(0, 0, 0, 0, 0, 0);

    ForceTorque ft;
    memset(&ft, 0, sizeof(ForceTorque));

    ft.fx = -2.0;

    robot.MoveCart(&desc_pos, 9, 0, 100.0, 100.0, 100.0, -1.0, -1);

    robot.FT_CalCenterStart();
    robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);
    robot.MoveCart(&desc_pos, 9, 0, 100.0, 100.0, 100.0, -1.0, -1);
    robot.WaitMs(1000);

    dir = 2;
    robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);
    robot.FT_CalCenterEnd(&xcenter);
    printf("xcenter:%f,%f,%f,%f,%f,%f\n", xcenter.tran.x, xcenter.tran.y, xcenter.tran.z, xcenter.rpy.rx, xcenter.rpy.ry, xcenter.rpy.rz);
    robot.MoveCart(&xcenter, 9, 0, 60.0, 50.0, 50.0, -1.0, -1);

    robot.FT_CalCenterStart();
    dir = 1;
    axis = 2;
    lin_v = 6.0;
    maxdis = 150.0;
    robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);
    robot.MoveCart(&desc_pos, 9, 0, 100.0, 100.0, 100.0, -1.0, -1);
    robot.WaitMs(1000);

    dir = 2;
    robot.FT_FindSurface(rcs, dir, axis, lin_v, lin_a, maxdis, ft_goal);
    robot.FT_CalCenterEnd(&ycenter);
    printf("ycenter:%f,%f,%f,%f,%f,%f\n", ycenter.tran.x, ycenter.tran.y, ycenter.tran.z, ycenter.rpy.rx, ycenter.rpy.ry, ycenter.rpy.rz);
    robot.MoveCart(&ycenter, 9, 0, 60.0, 50.0, 50.0, 0.0, -1);

    robot.CloseRPC();
    return 0;
}

int TestCompliance(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;

    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);

    int company = 22;
    int device = 0;
    int softversion = 0;
    int bus = 1;
    int index = 1;

    robot.FT_SetConfig(company, device, softversion, bus);
    robot.Sleep(1000);
    robot.FT_GetConfig(&company, &device, &softversion, &bus);
    printf("FT config:%d,%d,%d,%d\n", company, device, softversion, bus);
    robot.Sleep(1000);

    robot.FT_Activate(0);
    robot.Sleep(1000);
    robot.FT_Activate(1);
    robot.Sleep(1000);

    robot.Sleep(1000);
    robot.FT_SetZero(0);
    robot.Sleep(1000);

    uint8_t flag = 1;
    int sensor_id = 1;
    uint8_t select[6] = { 0,0,1,0,0,0 };
    float ft_pid[6] = { 0.0005,0.0,0.0,0.0,0.0,0.0 };
    uint8_t adj_sign = 0;
    uint8_t ILC_sign = 0;
    float max_dis = 100.0;
    float max_ang = 0.0;

    ForceTorque ft;
    DescPose  offset_pos(0, 0, 0, 0, 0, 0);
    ExaxisPos epos(0, 0, 0, 0);

    memset(&ft, 0, sizeof(ForceTorque));

    JointPos j1(-11.904, -99.669, 117.473, -108.616, -91.726, 74.256);
    JointPos j2(-45.615, -106.172, 124.296, -107.151, -91.282, 74.255);
    DescPose desc_p1(-419.524, -13.000, 351.569, -178.118, 0.314, 3.833);
    DescPose desc_p2(-321.222, 185.189, 335.520, -179.030, -1.284, -29.869);

    ft.fx = -10.0;
    ft.fy = -10.0;
    ft.fz = -10.0;
    robot.FT_Control(flag, sensor_id, select, &ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);
    float p = 0.00005;
    float force = 30.0;
    rtn = robot.FT_ComplianceStart(p, force);
    printf("FT_ComplianceStart rtn is %d\n", rtn);
    int count = 15;
    while (count)
    {
        robot.MoveL(&j1, &desc_p1, 0, 0, 100.0, 180.0, 100.0, -1.0, &epos, 0, 1, &offset_pos);
        robot.MoveL(&j2, &desc_p2, 0, 0, 100.0, 180.0, 100.0, -1.0, &epos, 0, 0, &offset_pos);
        count -= 1;
    }
    robot.FT_ComplianceStop();
    printf("FT_ComplianceStop rtn is %d\n", rtn);
    flag = 0;
    robot.FT_Control(flag, sensor_id, select, &ft, ft_pid, adj_sign, ILC_sign, max_dis, max_ang, 0, 0, 0);

    robot.CloseRPC();
    return 0;
}

int TestEndForceDragCtrl(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;

    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);

    robot.SetForceSensorDragAutoFlag(1);

    vector <double> M = { 15.0, 15.0, 15.0, 0.5, 0.5, 0.1 };
    vector <double> B = { 150.0, 150.0, 150.0, 5.0, 5.0, 1.0 };
    vector <double> K = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    vector <double> F = { 10.0, 10.0, 10.0, 1.0, 1.0, 1.0 };
    robot.EndForceDragControl(1, 0, 0, 0, 1, M, B, K, F, 50, 100);
    printf("EndForceDragControl start\n");

    robot.Sleep(5000);

    int dragState = 0;
    int sixDimensionalDragState = 0;
    robot.GetForceAndTorqueDragState(dragState, sixDimensionalDragState);
    printf("the drag state is %d %d \n", dragState, sixDimensionalDragState);

    robot.EndForceDragControl(0, 0, 0, 0, 1, M, B, K, F, 50, 100);
    printf("EndForceDragControl end\n");
    robot.CloseRPC();
    return 0;
}

int TestForceAndJointImpedance(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;

    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);

    robot.DragTeachSwitch(1);
    vector <double> lamdeDain = { 3.0, 2.0, 2.0, 2.0, 2.0, 3.0 };
    vector <double> KGain = { 0, 0, 0, 0, 0, 0 };
    vector <double> BGain = { 150, 150, 150, 5.0, 5.0, 1.0 };
    rtn = robot.ForceAndJointImpedanceStartStop(1, 0, lamdeDain, KGain, BGain, 1000, 180);
    printf("ForceAndJointImpedanceStartStop rtn is %d\n", rtn);

    robot.Sleep(5000);

    robot.DragTeachSwitch(0);
    rtn = robot.ForceAndJointImpedanceStartStop(0, 0, lamdeDain, KGain, BGain, 1000, 180);
    printf("ForceAndJointImpedanceStartStop rtn is %d\n", rtn);

    robot.CloseRPC();
    return 0;
}

int TestIdentify(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    int retval = 0;
    retval = robot.LoadIdentifyDynFilterInit();
    printf("LoadIdentifyDynFilterInit retval is: %d \n", retval);
    retval = robot.LoadIdentifyDynVarInit();
    printf("LoadIdentifyDynVarInit retval is: %d \n", retval);
    JointPos posJ = {};
    DescPose posDec = {};
    float joint_toq[6] = { 0.0 };
    robot.GetActualJointPosDegree(0, &posJ);
    posJ.jPos[1] = posJ.jPos[1] + 10;
    robot.GetJointTorques(0, joint_toq);
    joint_toq[1] = joint_toq[1] + 2;
    double tmpTorque[6] = { 0.0 };
    for (int i = 0; i < 6; i++)
    {
        tmpTorque[i] = joint_toq[i];
    }
    retval = robot.LoadIdentifyMain(tmpTorque, posJ.jPos, 1);
    printf("LoadIdentifyMain retval is: %d \n", retval);
    double gain[12] = { 0,0.05,0,0,0,0,0,0.02,0,0,0,0 };
    double weight = 0;
    DescTran load_pos;
    memset(&load_pos, 0, sizeof(DescTran));
    retval = robot.LoadIdentifyGetResult(gain, &weight, &load_pos);
    printf("LoadIdentifyGetResult retval is: %d ; weight is %f cog is %f %f %f \n", retval, weight, load_pos.x, load_pos.y, load_pos.z);
    robot.CloseRPC();
    return 0;
}

int TestImpedanceControl()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    uint8_t ctrl[20];
    uint8_t state;
    int pressVlaue;
    int error;
    robot.CloseRPC();
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return 0;
    }
    robot.SetReConnectParam(true, 30000, 500);
    JointPos j1(102.622, -135.990, 120.769, -73.950, -90.848, 35.507);
    JointPos j2(93.674, -80.062, 82.947, -92.199, -90.967, 26.559);
    DescPose desc_pos1(136.552, -149.799, 449.532, 179.817, -1.172, 157.123);
    DescPose desc_pos2(136.540, -561.048, 449.542, 179.819, -1.172, 157.122);
    DescPose offset_pos(0, 0, 0, 0, 0, 0);
    ExaxisPos epos(0, 0, 0, 0);
    int tool = 0;
    int user = 0;
    float vel = 100.0;
    float acc = 200.0;
    float ovl = 100.0;
    float blendT = -1.0;
    float blendR = -1.0;
    uint8_t flag = 0;
    uint8_t search = 0;
    robot.SetSpeed(20);
    int company = 22;
    int device = 0;
    int softversion = 0;
    int bus = 1;
    robot.FT_SetConfig(company, device, softversion, bus);
    robot.Sleep(1000);
    robot.FT_GetConfig(&company, &device, &softversion, &bus);
    printf("FT config:%d,%d,%d,%d\n", company, device, softversion, bus);
    robot.Sleep(1000);
    robot.FT_Activate(0);
    robot.Sleep(1000);
    robot.FT_Activate(1);
    robot.Sleep(1000);
    robot.Sleep(1000);
    robot.FT_SetZero(0);
    robot.Sleep(1000);
    robot.FT_SetZero(1);
    robot.Sleep(1000);
    double forceThreshold[6] = { 30,30,30,5,5,5 };
    double m[6] = { 0.1,0.1,0.1,0.02,0.02,0.02 };
    double b[6] = { 1,1,1,0.08,0.08,0.08 };
    double k[6] = { 0,0,0,0,0,0 };
    rtn = robot.ImpedanceControlStartStop(1, 1, forceThreshold, m, b, k, 1000, 500, 100, 100);
    printf("ImpedanceControlStartStop errcode:%d\n", rtn);
    rtn = robot.MoveL(&desc_pos1, tool, user, vel, acc, ovl, blendR, 0, &epos, search, flag, &offset_pos, -1, 1);
    rtn = robot.MoveL(&desc_pos2, tool, user, vel, acc, ovl, blendR, 0, &epos, search, flag, &offset_pos, -1, 1);
    rtn = robot.MoveL(&desc_pos1, tool, user, vel, acc, ovl, blendR, 0, &epos, search, flag, &offset_pos, -1, 1);
    rtn = robot.MoveL(&desc_pos2, tool, user, vel, acc, ovl, blendR, 0, &epos, search, flag, &offset_pos, -1, 1);
    printf("movel errcode:%d\n", rtn);
    robot.ImpedanceControlStartStop(0, 1, forceThreshold, m, b, k, 1000, 500, 100, 100);
    robot.CloseRPC();
    return 0;
}


int TestFTStrategy()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    robot.SetReConnectParam(true, 30000, 500);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }

    //========== FT_SpiralSearch: strategy 0 / 1 ==========
    printf("=== FT_SpiralSearch strategy=0 ===\n");
    rtn = robot.FT_SpiralSearch(0, 0.7f, 5.0f, 3000.0f, 3.0f, 0);
    printf("FT_SpiralSearch(0) rtn is %d\n", rtn);
    robot.Sleep(5000);
    robot.ResetAllError();
    robot.Sleep(2000);

    printf("=== FT_SpiralSearch strategy=1 ===\n");
    rtn = robot.FT_SpiralSearch(0, 0.7f, 1.0f, 3000.0f, 3.0f, 1);
    printf("FT_SpiralSearch(1) rtn is %d\n", rtn);
    robot.Sleep(5000);
    robot.ResetAllError();
    robot.Sleep(2000);

    // ========== FT_LinInsertion: strategy 0/1 ==========
    printf("=== FT_LinInsertion strategy=0 ===\n");
    rtn = robot.FT_LinInsertion(0, 20.0f, 15.0f, 1.0f, 10.0f, 1, 0);
    printf("FT_LinInsertion(0) rtn is %d\n", rtn);
    robot.Sleep(5000);
    robot.ResetAllError();
    robot.Sleep(2000);

    printf("=== FT_LinInsertion strategy=1 ===\n");
    rtn = robot.FT_LinInsertion(0, 20.0f, 15.0f, 1.0f, 10.0f, 1, 1);
    printf("FT_LinInsertion(1) rtn is %d\n", rtn);
    robot.Sleep(5000);
    robot.ResetAllError();
    robot.Sleep(2000);

    // ========== FT_FindSurface: strategy 0/1 ==========
    printf("=== FT_FindSurface strategy=0 ===\n");
    rtn = robot.FT_FindSurface(0, 1, 1, 15.0f, 0.0f, 50.0f, 20.0f, 0);
    printf("FT_FindSurface(0) rtn is %d\n", rtn);
    robot.Sleep(10000);
    robot.ResetAllError();
    robot.Sleep(5000);

    printf("=== FT_FindSurface strategy=1 ===\n");
    rtn = robot.FT_FindSurface(0, 1, 1, 15.0f, 0.0f, 50.0f, 20.0f, 1);
    printf("FT_FindSurface(1) rtn is %d\n", rtn);
    robot.Sleep(10000);
    robot.ResetAllError();

    printf("finish\n");

    robot.Sleep(1000);
    robot.CloseRPC();
    robot.Sleep(1000);
    return 0;
}

#pragma endregion
#pragma region 扩展轴

int Test485Auxservo(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    int retval = robot.AuxServoSetParam(1, 1, 1, 1, 131072, 15.45);
    std::cout << "AuxServoSetParam is: " << retval << std::endl;
    int servoCompany;
    int servoModel;
    int servoSoftVersion;
    int servoResolution;
    double axisMechTransRatio;
    retval = robot.AuxServoGetParam(1, &servoCompany, &servoModel, &servoSoftVersion, &servoResolution, &axisMechTransRatio);
    std::cout << "servoCompany " << servoCompany << "\n"
        << "servoModel " << servoModel << "\n"
        << "servoSoftVersion " << servoSoftVersion << "\n"
        << "servoResolution " << servoResolution << "\n"
        << "axisMechTransRatio " << axisMechTransRatio << "\n"
        << std::endl;
    retval = robot.AuxServoSetParam(1, 10, 11, 12, 13, 14);
    std::cout << "AuxServoSetParam is: " << retval << std::endl;
    retval = robot.AuxServoGetParam(1, &servoCompany, &servoModel, &servoSoftVersion, &servoResolution, &axisMechTransRatio);
    std::cout << "servoCompany " << servoCompany << "\n"
        << "servoModel " << servoModel << "\n"
        << "servoSoftVersion " << servoSoftVersion << "\n"
        << "servoResolution " << servoResolution << "\n"
        << "axisMechTransRatio " << axisMechTransRatio << "\n"
        << std::endl;
    retval = robot.AuxServoSetParam(1, 1, 1, 1, 131072, 36);
    std::cout << "AuxServoSetParam is: " << retval << std::endl;
    robot.Sleep(3000);
    robot.AuxServoSetAcc(3000, 3000);
    robot.AuxServoSetEmergencyStopAcc(5000, 5000);
    robot.Sleep(1000);
    double emagacc = 0, acc = 0;
    double emagdec = 0, dec = 0;
    robot.AuxServoGetEmergencyStopAcc(emagacc, emagdec);
    printf("emergency acc is %f dec is %f \n", emagacc, emagdec);
    robot.AuxServoGetAcc(acc, dec);
    printf("acc is %f dec is %f \n", acc, dec);
    robot.AuxServoSetControlMode(1, 0);
    robot.Sleep(2000);
    retval = robot.AuxServoEnable(1, 0);
    std::cout << "AuxServoEnable disenable " << retval << std::endl;
    robot.Sleep(1000);
    int servoerrcode = 0;
    int servoErrCode;
    int servoState;
    double servoPos;
    double servoSpeed;
    double servoTorque;
    retval = robot.AuxServoGetStatus(1, &servoErrCode, &servoState, &servoPos, &servoSpeed, &servoTorque);
    std::cout << "AuxServoGetStatus servoState " << std::bitset<16>(servoState) << std::endl;
    robot.Sleep(1000);;
    retval = robot.AuxServoEnable(1, 1);
    std::cout << "AuxServoEnable enable " << retval << std::endl;
    robot.Sleep(1000);
    retval = robot.AuxServoGetStatus(1, &servoErrCode, &servoState, &servoPos, &servoSpeed, &servoTorque);
    std::cout << "AuxServoGetStatus servoState " << std::bitset<16>(servoState) << std::endl;
    robot.Sleep(1000);
    retval = robot.AuxServoHoming(1, 1, 5, 1);
    std::cout << "AuxServoHoming " << retval << std::endl;
    robot.Sleep(3000);
    retval = robot.AuxServoSetTargetPos(1, 200, 30);
    std::cout << "AuxServoSetTargetPos " << retval << std::endl;
    robot.Sleep(1000);
    retval = robot.AuxServoGetStatus(1, &servoErrCode, &servoState, &servoPos, &servoSpeed, &servoTorque);
    std::cout << "AuxServoGetStatus servoSpeed " << servoSpeed << std::endl;
    robot.Sleep(8000);
    robot.AuxServoSetControlMode(1, 1);
    robot.Sleep(2000);
    robot.AuxServoEnable(1, 0);
    robot.Sleep(1000);
    robot.AuxServoEnable(1, 1);
    robot.Sleep(1000);
    robot.AuxServoSetTargetSpeed(1, 100, 80);
    robot.Sleep(5000);
    robot.AuxServoSetTargetSpeed(1, 0, 80);
    robot.CloseRPC();
    return 0;
}

int TestUDPAxis(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
   /* rtn = robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 200, 1, 100, 5, 1);
    cout << "ExtDevSetUDPComParam rtn is " << rtn << endl;
    string ip = ""; int port = 0; int period = 0; int lossPkgTime = 0; int lossPkgNum = 0; int disconnectTime = 0; int reconnectEnable = 0; int reconnectPeriod = 0; int reconnectNum = 0; int selfConnect = 0;
    rtn = robot.ExtDevGetUDPComParam(ip, port, period, lossPkgTime, lossPkgNum, disconnectTime, reconnectEnable, reconnectPeriod, reconnectNum, selfConnect);
    string patam = "\nip " + ip + "\nport " + to_string(port) + "\nperiod  " + to_string(period) + "\nlossPkgTime " + to_string(lossPkgTime) + "\nlossPkgNum  " + to_string(lossPkgNum) + "\ndisConntime  " +
        to_string(disconnectTime) + "\nreconnecable  " + to_string(reconnectEnable) + "\nreconnperiod  " + to_string(reconnectPeriod) + "\nreconnnun  " + to_string(reconnectNum) + "\nselfConnect  " + to_string(selfConnect);
    cout << "ExtDevGetUDPComParam rtn is " << rtn << patam << endl;
    robot.ExtDevLoadUDPDriver();
    rtn = robot.SetExAxisCmdDoneTime(5000.0);
    cout << "SetExAxisCmdDoneTime rtn is " << rtn << endl;
    rtn = robot.ExtAxisServoOn(1, 1);
    cout << "ExtAxisServoOn axis id 1 rtn is " << rtn << endl;
    rtn = robot.ExtAxisServoOn(2, 1);
    cout << "ExtAxisServoOn axis id 2 rtn is " << rtn << endl;
    robot.Sleep(2000);
    robot.ExtAxisSetHoming(1, 0, 10, 2);
    robot.Sleep(2000);
    rtn = robot.ExtAxisSetHoming(2, 0, 10, 2);
    cout << "ExtAxisSetHoming rtnn is  " << rtn << endl;
    robot.Sleep(4000);
    rtn = robot.SetRobotPosToAxis(1);
    cout << "SetRobotPosToAxis rtn is " << rtn << endl;
    rtn = robot.SetAxisDHParaConfig(10, 20, 0, 0, 0, 0, 0, 0, 0);
    cout << "SetAxisDHParaConfig rtn is " << rtn << endl;
    int axisType = -1;
    int axisDirection = -1;
    double axisMax = -1;
    double axisMin = -1;
    double axisVel = -1;
    double axisAcc = -1;
    double axisLead = -1;
    int encResolution = -1;
    double axisOffect = -1;
    int axisCompany = -1;
    int axisModel = -1;
    int axisEncType = -1;
    rtn = robot.ExtAxisParamConfig(1, 1, 1, 1000, -1000, 1000, 1000, 1.905, 262144, 200, 1, 0, 0);
    cout << "ExtAxisParamConfig axis 1 rtn is " << rtn << endl;
    rtn = robot.ExtAxisGetParamConfig(1, axisType, axisDirection, axisMax, axisMin, axisVel, axisAcc, axisLead, encResolution, axisOffect, axisCompany, axisModel, axisEncType);
    printf("axis id 1 ExtAxisGetParamConfig : axisType %d, axisDirection %d, axisMax %lf, axisMin %lf, axisVel %lf, axisAcc %lf, axisLead%lf, encResolution %d, axisOffect %f, axisCompany %d, axisModel %d, axisEncType %d\n",
        axisType, axisDirection, axisMax, axisMin, axisVel, axisAcc, axisLead, encResolution, axisOffect, axisCompany, axisModel, axisEncType);
    rtn = robot.ExtAxisParamConfig(2, 1, 1, 1000, -1000, 1000, 1000, 4.444, 262144, 200, 1, 0, 0);
    cout << "ExtAxisParamConfig axis 2 rtn is " << rtn << endl;
    rtn = robot.ExtAxisGetParamConfig(2, axisType, axisDirection, axisMax, axisMin, axisVel, axisAcc, axisLead, encResolution, axisOffect, axisCompany, axisModel, axisEncType);
    printf("axis id 2 ExtAxisGetParamConfig : axisType %d, axisDirection %d, axisMax %lf, axisMin %lf, axisVel %lf, axisAcc %lf, axisLead%lf, encResolution %d, axisOffect %f, axisCompany %d, axisModel %d, axisEncType %d\n",
        axisType, axisDirection, axisMax, axisMin, axisVel, axisAcc, axisLead, encResolution, axisOffect, axisCompany, axisModel, axisEncType);*/
    robot.Sleep(1000 * 3);
    robot.ExtAxisStartJog(1, 0, 10, 10, 30);
    robot.Sleep(1000 * 1);
    robot.ExtAxisStopJog(1);
    robot.Sleep(1000 * 3);
    robot.ExtAxisServoOn(1, 0);
    robot.Sleep(1000 * 3);
    robot.ExtAxisStartJog(2, 0, 10, 10, 30);
    robot.Sleep(1000 * 1);
    robot.ExtAxisStopJog(2);
    robot.Sleep(1000 * 3);
    robot.ExtAxisServoOn(2, 0);
    robot.Sleep(1000 * 1);
    //robot.ExtDevUnloadUDPDriver();
    robot.CloseRPC();
    return 0;
}

int TestUDPAxisMove(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    ExaxisPos axisPos1;
    axisPos1.ePos[0] = 20;
    axisPos1.ePos[1] = 0;
    axisPos1.ePos[2] = 0;
    axisPos1.ePos[3] = 0;

    ExaxisPos axisPos2;
    axisPos2.ePos[0] = 0;
    axisPos2.ePos[1] = 20;
    axisPos2.ePos[2] = 0;
    axisPos2.ePos[3] = 0;

    for (int i = 0; i< 5; i++)
    {
        robot.ExtAxisMove(axisPos1, 50);
        robot.Sleep(2000);
        robot.ExtAxisMove(axisPos2, 50);
        robot.Sleep(2000);
    }
    robot.CloseRPC();
    return 0;
}

int testSyncMoveJ()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    //1.标定并应用机器人工具坐标系，您可以使用四点法或六点法进行工具坐标系的标定和应用，涉及工具坐标系标定的接口如下：
    //  int SetToolPoint(int point_num); //设置工具参考点-六点法
    //  int ComputeTool(ref DescPose tcp_pose); //计算工具坐标系
    //  int SetTcp4RefPoint(int point_num);  //设置工具参考点-四点法
    //  int ComputeTcp4(ref DescPose tcp_pose);  //计算工具坐标系-四点法
    //  int SetToolCoord(int id, DescPose coord, int type, int install); //设置应用工具坐标系
    //  int SetToolList(int id, DescPose coord, int type, int install);  //设置应用工具坐标系列表
    //2.设置UDP通信参数，并加载UDP通信
    //robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
    //robot.ExtDevLoadUDPDriver();
    ////3.设置扩展轴参数，包括扩展轴类型、扩展轴驱动器参数、扩展轴DH参数
    //robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0); //单轴变位机及DH参数
    //robot.SetRobotPosToAxis(1); //扩展轴安装位置
    //robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0); //伺服驱动器参数，本示例为单轴变位机，因此只需要设置一个驱动器参数，若您选择包含多个轴的扩展轴类型，需要每一个轴设置驱动器参数
    ////4.设置所选的轴使能、回零
    //robot.ExtAxisServoOn(1, 0);
    //robot.ExtAxisSetHoming(1, 0, 20, 3);
    ////5.进行扩展轴坐标系标定及应用
    //DescPose pos = {/* 输入您的标定点坐标 */ };
    //robot.SetRefPointInExAxisEnd(pos);
    //robot.PositionorSetRefPoint(1); /*您需要通过四个不同位置的点来标定扩展轴，因此需要调用此接口4次才能完成标定 */
    //DescPose coord = {};
    //robot.PositionorComputeECoordSys(coord); //计算扩展轴标定结果
    //robot.ExtAxisActiveECoordSys(1, 1, coord, 1); //将标定结果应用到扩展轴坐标系
    //6.在扩展轴上标定工件坐标系，您需要用到以下接口
    //int SetWObjCoordPoint(int point_num);
    //int ComputeWObjCoord(int method, ref DescPose wobj_pose);
    //int SetWObjCoord(int id, DescPose coord);
    //int SetWObjList(int id, DescPose coord);
    //7.记录您的同步关节运动起始点
    DescPose startdescPose = { 409.950, 35.714, 272.466, -142.158, -1.209, -134.392 };
    JointPos startjointPos = { 60.164, -104.046, -20.299, -157.828, 53.871, 108.125 };
    ExaxisPos startexaxisPos = { 35.00, 25.00, 0.000, 0.000 };
    //8.记录您的同步关节运动终点坐标
    DescPose enddescPose = { 485.838, 25.316, 313.259, -137.616, 17.480, -138.072 };
    JointPos endjointPos = { 55.266, -89.767, -46.349, -128.985, 45.001, 108.13 };
    ExaxisPos endexaxisPos = { 35.00, -60.000, 0.000, 0.000 };
    //9.编写同步运动程序
    //运动到起始点，假设应用的工具坐标系、工件坐标系都是1
    robot.ExtAxisMove(startexaxisPos, 20);
    DescPose offdese = { 0, 0, 0, 0, 0, 0 };
    robot.MoveJ(&startjointPos, &startdescPose, 1, 0, 100, 100, 100, &startexaxisPos, 0, 0, &offdese);
    //开始同步运动
    robot.ExtAxisSyncMoveJ(endjointPos, enddescPose, 1, 0, 100, 100, 20, endexaxisPos, -1, 0, offdese);


    robot.MoveJ(&startjointPos, 1, 0, 100, 100, 100, &startexaxisPos, 0, 0, &offdese);
    robot.ExtAxisMove(startexaxisPos, 20);
    //开始同步运动
    robot.ExtAxisSyncMoveJ(endjointPos, 1, 0, 100, 100, 20, endexaxisPos, -1, 0, offdese);
    robot.CloseRPC();
}

int testSyncMoveL()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    //1.标定并应用机器人工具坐标系，您可以使用四点法或六点法进行工具坐标系的标定和应用，涉及工具坐标系标定的接口如下：
    //  int SetToolPoint(int point_num); //设置工具参考点-六点法
    //  int ComputeTool(ref DescPose tcp_pose); //计算工具坐标系
    //  int SetTcp4RefPoint(int point_num);  //设置工具参考点-四点法
    //  int ComputeTcp4(ref DescPose tcp_pose);  //计算工具坐标系-四点法
    //  int SetToolCoord(int id, DescPose coord, int type, int install); //设置应用工具坐标系
    //  int SetToolList(int id, DescPose coord, int type, int install);  //设置应用工具坐标系列表
    //2.设置UDP通信参数，并加载UDP通信
    //robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
    //robot.ExtDevLoadUDPDriver();
    ////3.设置扩展轴参数，包括扩展轴类型、扩展轴驱动器参数、扩展轴DH参数
    //robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0); //单轴变位机及DH参数
    //robot.SetRobotPosToAxis(1); //扩展轴安装位置
    //robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0); //伺服驱动器参数，本示例为单轴变位机，因此只需要设置一个驱动器参数，若您选择包含多个轴的扩展轴类型，需要每一个轴设置驱动器参数
    ////4.设置所选的轴使能、回零
    //robot.ExtAxisServoOn(1, 0);
    //robot.ExtAxisSetHoming(1, 0, 20, 3);
    ////5.进行扩展轴坐标系标定及应用
    //DescPose pos = {/* 输入您的标定点坐标 */ };
    //robot.SetRefPointInExAxisEnd(pos);
    //robot.PositionorSetRefPoint(1); /*您需要通过四个不同位置的点来标定扩展轴，因此需要调用此接口4次才能完成标定 */
    //DescPose coord = {};
    //robot.PositionorComputeECoordSys(coord); //计算扩展轴标定结果
    //robot.ExtAxisActiveECoordSys(1, 1, coord, 1); //将标定结果应用到扩展轴坐标系
    //6.在扩展轴上标定工件坐标系，您需要用到以下接口
    //int SetWObjCoordPoint(int point_num);
    //int ComputeWObjCoord(int method, ref DescPose wobj_pose);
    //int SetWObjCoord(int id, DescPose coord);
    //int SetWObjList(int id, DescPose coord);
    //7.记录您的同步关节运动起始点
    DescPose startdescPose = { 409.950, 35.714, 272.466, -142.158, -1.209, -134.392 };
    JointPos startjointPos = { 60.164, -104.046, -20.299, -157.828, 53.871, 108.125 };
    ExaxisPos startexaxisPos = { 35.00, 25.00, 0.000, 0.000 };
    //8.记录您的同步关节运动终点坐标
    DescPose enddescPose = { 485.838, 25.316, 313.259, -137.616, 17.480, -138.072 };
    JointPos endjointPos = { 55.266, -89.767, -46.349, -128.985, 45.001, 108.13 };
    ExaxisPos endexaxisPos = { 35.00, -60.000, 0.000, 0.000 };
    //9.编写同步运动程序
    //运动到起始点，假设应用的工具坐标系、工件坐标系都是1
    rtn = robot.ExtAxisMove(startexaxisPos, 20);
    printf("ExtAxisMove rtn is %d\n", rtn);
    DescPose offdese = { 0, 0, 0, 0, 0, 0 };
    robot.MoveJ(&startjointPos, &startdescPose, 1, 0, 100, 100, 100, &startexaxisPos, -1, 0, &offdese);
    //开始同步运动
    rtn = robot.ExtAxisSyncMoveL(endjointPos, enddescPose, 1, 0, 20, 100, 20, -1, endexaxisPos, 0, offdese);
    printf("ExtAxisSyncMoveL rtn is %d\n", rtn);

    robot.ExtAxisMove(startexaxisPos, 20);
    printf("ExtAxisMove rtn is %d\n", rtn);
    robot.MoveJ(&startjointPos, 1, 0, 100, 100, 100, &startexaxisPos, -1, 0, &offdese);
    //开始同步运动
    rtn = robot.ExtAxisSyncMoveL(enddescPose, 1, 0, 20, 100, 20, -1, endexaxisPos, 0, offdese);
    printf("ExtAxisSyncMoveL rtn is %d\n", rtn);
    robot.CloseRPC();
}


int testSyncMoveC()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    //1.标定并应用机器人工具坐标系，您可以使用四点法或六点法进行工具坐标系的标定和应用，涉及工具坐标系标定的接口如下：
    //  int SetToolPoint(int point_num); //设置工具参考点-六点法
    //  int ComputeTool(ref DescPose tcp_pose); //计算工具坐标系
    //  int SetTcp4RefPoint(int point_num);  //设置工具参考点-四点法
    //  int ComputeTcp4(ref DescPose tcp_pose);  //计算工具坐标系-四点法
    //  int SetToolCoord(int id, DescPose coord, int type, int install); //设置应用工具坐标系
    //  int SetToolList(int id, DescPose coord, int type, int install);  //设置应用工具坐标系列表
    //2.设置UDP通信参数，并加载UDP通信
    //robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 100, 1, 100, 10);
    //robot.ExtDevLoadUDPDriver();
    ////3.设置扩展轴参数，包括扩展轴类型、扩展轴驱动器参数、扩展轴DH参数
    //robot.SetAxisDHParaConfig(4, 200, 200, 0, 0, 0, 0, 0, 0); //单轴变位机及DH参数
    //robot.SetRobotPosToAxis(1); //扩展轴安装位置
    //robot.ExtAxisParamConfig(1, 0, 1, 100, -100, 10, 10, 12, 131072, 0, 1, 0, 0); //伺服驱动器参数，本示例为单轴变位机，因此只需要设置一个驱动器参数，若您选择包含多个轴的扩展轴类型，需要每一个轴设置驱动器参数
    ////4.设置所选的轴使能、回零
    //robot.ExtAxisServoOn(1, 0);
    //robot.ExtAxisSetHoming(1, 0, 20, 3);
    ////5.进行扩展轴坐标系标定及应用
    //DescPose pos = {/* 输入您的标定点坐标 */ };
    //robot.SetRefPointInExAxisEnd(pos);
    //robot.PositionorSetRefPoint(1); /*您需要通过四个不同位置的点来标定扩展轴，因此需要调用此接口4次才能完成标定 */
    //DescPose coord = {};
    //robot.PositionorComputeECoordSys(coord); //计算扩展轴标定结果
    //robot.ExtAxisActiveECoordSys(1, 1, coord, 1); //将标定结果应用到扩展轴坐标系
    //6.在扩展轴上标定工件坐标系，您需要用到以下接口
    //int SetWObjCoordPoint(int point_num);
    //int ComputeWObjCoord(int method, ref DescPose wobj_pose);
    //int SetWObjCoord(int id, DescPose coord);
    //int SetWObjList(int id, DescPose coord);
    //7.记录您的同步圆弧运动起始点
    DescPose startdescPose = { 409.950, 35.714, 272.466, -142.158, -1.209, -134.392 };
    JointPos startjointPos = { 60.164, -104.046, -20.299, -157.828, 53.871, 108.125 };
    ExaxisPos startexaxisPos = { 35.00, 25.00, 0.000, 0.000 };
    //8.记录您的同步关节运动终点坐标
    DescPose enddescPose = { 485.838, 25.316, 313.259, -137.616, 17.480, -138.072 };
    JointPos endjointPos = { 55.266, -89.767, -46.349, -128.985, 45.001, 108.13 };
    ExaxisPos endexaxisPos = { 35.00, -60.000, 0.000, 0.000 };
    //9.记录您的同步圆弧运动中间点坐标
    DescPose middescPose = { 456.062, 47.663, 291.916, -139.201, 4.688, -135.673 };
    JointPos midjointPos = { 58.054, -107.816, -15.798, -153.559, 49.501, 108.121 };
    ExaxisPos midexaxisPos = { 35.00, -25.000, 0.000, 0.000 };
    //10.编写同步运动程序
    //运动到起始点，假设应用的工具坐标系、工件坐标系都是1
    robot.ExtAxisMove(startexaxisPos, 20);
    DescPose offdese = { 0, 0, 0, 0, 0, 0 };
    robot.MoveJ(&startjointPos, &startdescPose, 1, 0, 100, 100, 100, &startexaxisPos, -1, 0, &offdese);
    //开始同步运动
    robot.ExtAxisSyncMoveC(midjointPos, middescPose, 1, 0, 20, 100, midexaxisPos, 0, offdese, endjointPos, enddescPose, 1, 0, 20, 100, endexaxisPos, 0, offdese, 100, -1);
    robot.ExtAxisMove(startexaxisPos, 20);
    robot.MoveJ(&startjointPos, 1, 0, 100, 100, 100, &startexaxisPos, -1, 0, &offdese);
    //开始同步运动
    robot.ExtAxisSyncMoveC(middescPose, 1, 0, 20, 100, midexaxisPos, 0, offdese, enddescPose, 1, 0, 20, 100, endexaxisPos, 0, offdese, 100, -1);
    robot.CloseRPC();
}

int TestUDPAxisCalib(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
   /* rtn = robot.ExtDevSetUDPComParam("192.168.58.88", 2021, 2, 100, 3, 200, 1, 100, 5, 1);
    cout << "ExtDevSetUDPComParam rtn is " << rtn << endl;
    string ip = ""; int port = 0; int period = 0; int lossPkgTime = 0; int lossPkgNum = 0; int disconnectTime = 0; int reconnectEnable = 0; int reconnectPeriod = 0; int reconnectNum = 0;
    rtn = robot.ExtDevGetUDPComParam(ip, port, period, lossPkgTime, lossPkgNum, disconnectTime, reconnectEnable, reconnectPeriod, reconnectNum);
    string patam = "\nip " + ip + "\nport " + to_string(port) + "\nperiod " + to_string(period) + "\nlossPkgTime " + to_string(lossPkgTime) + "\nlossPkgNum " + to_string(lossPkgNum) + "\ndisConntime " + to_string(disconnectTime) + "\nreconnecable " + to_string(reconnectEnable) + "\nreconnperiod " + to_string(reconnectPeriod) + "\nreconnnun " + to_string(reconnectNum);
    cout << "ExtDevGetUDPComParam rtn is " << rtn << patam << endl;
    robot.ExtDevLoadUDPDriver();
    rtn = robot.ExtAxisServoOn(1, 1);
    cout << "ExtAxisServoOn axis id 1 rtn is " << rtn << endl;
    rtn = robot.ExtAxisServoOn(2, 1);
    cout << "ExtAxisServoOn axis id 2 rtn is " << rtn << endl;
    robot.Sleep(2000);
    robot.ExtAxisSetHoming(1, 0, 10, 2);
    robot.Sleep(2000);
    rtn = robot.ExtAxisSetHoming(2, 0, 10, 2);
    cout << "ExtAxisSetHoming rtnn is " << rtn << endl;*/
    robot.Sleep(4000);
    rtn = robot.SetRobotPosToAxis(1);
    cout << "SetRobotPosToAxis rtn is " << rtn << endl;
    rtn = robot.SetAxisDHParaConfig(1, 128.5, 206.4, 0, 0, 0, 0, 0, 0);
    cout << "SetAxisDHParaConfig rtn is " << rtn << endl;
    //rtn = robot.ExtAxisParamConfig(1, 1, 1, 1000, -1000, 1000, 1000, 1.905, 262144, 200, 1, 0, 0);
    //cout << "ExtAxisParamConfig axis 1 rtn is " << rtn << endl;
    //rtn = robot.ExtAxisParamConfig(2, 1, 1, 1000, -1000, 1000, 1000, 4.444, 262144, 200, 1, 0, 0);
    //cout << "ExtAxisParamConfig axis 1 rtn is " << rtn << endl;
    DescPose toolCoord(0, 0, 300, 0, 0, 0);
    robot.SetToolCoord(1, &toolCoord, 0, 0, 1, 0);
    JointPos jSafe(47.434, -74.061, -46.445, -140.394, 52.175, 108.040);
    JointPos j1(46.778, -75.370, -45.376, -140.058, 51.582, 108.038);
    JointPos j2(26.821, -79.971, -41.801, -124.459, 65.051, 108.036);
    JointPos j3(26.709, -82.025, -39.224, -124.958, 64.560, 108.035);
    JointPos j4(27.177, -82.909, -38.352, -124.937, 63.591, 108.035);


    DescPose descSafe = {};
    DescPose desc1 = {};
    DescPose desc2 = {};
    DescPose desc3 = {};
    DescPose desc4 = {};
    ExaxisPos exaxisPos = { 0, 0, 0, 0 };
    DescPose offdese = { 0, 0, 0, 0, 0, 0 };
    robot.GetForwardKin(&jSafe, &descSafe);
    robot.MoveJ(&jSafe, &descSafe, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);

    ExaxisPos axisPos1(35, 22, 0, 0);
    robot.ExtAxisMove(axisPos1, 50);

    robot.Sleep(2000);
    robot.GetForwardKin(&j1, &desc1);
    robot.MoveJ(&j1, &desc1, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.Sleep(2000);
    DescPose actualTCPPos = {};
    robot.GetActualTCPPose(0, &actualTCPPos);
    robot.SetRefPointInExAxisEnd(actualTCPPos);
    rtn = robot.PositionorSetRefPoint(1);
    cout << "PositionorSetRefPoint 1 rtn is " << rtn << endl;
    robot.Sleep(2000);
    //robot.MoveJ(&jSafe, &descSafe, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    //robot.ExtAxisStartJog(1, 0, 50, 50, 10);
    //robot.Sleep(1000);
    robot.ExtAxisStartJog(2, 1, 50, 50, 5);
    robot.Sleep(1000);
    robot.GetForwardKin(&j2, &desc2);
    rtn = robot.MoveJ(&j2, &desc2, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    rtn = robot.PositionorSetRefPoint(2);
    cout << "PositionorSetRefPoint 2 rtn is " << rtn << endl;
    robot.Sleep(2000);
    //robot.MoveJ(&jSafe, &descSafe, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    //robot.ExtAxisStartJog(1, 0, 50, 50, 10);
    //robot.Sleep(1000);
    robot.ExtAxisStartJog(2, 1, 50, 50, 5);
    robot.Sleep(1000);
    robot.GetForwardKin(&j3, &desc3);
    robot.MoveJ(&j3, &desc3, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    rtn = robot.PositionorSetRefPoint(3);
    cout << "PositionorSetRefPoint 3 rtn is " << rtn << endl;
    robot.Sleep(2000);
    //robot.MoveJ(&jSafe, &descSafe, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    //robot.ExtAxisStartJog(1, 0, 50, 50, 10);
    //robot.Sleep(1000);
    robot.ExtAxisStartJog(2, 1, 50, 50, 5);
    robot.Sleep(1000);
    robot.GetForwardKin(&j4, &desc4);
    robot.MoveJ(&j4, &desc4, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    rtn = robot.PositionorSetRefPoint(4);
    cout << "PositionorSetRefPoint 4 rtn is " << rtn << endl;
    robot.Sleep(2000);
    DescPose axisCoord = {};
    robot.PositionorComputeECoordSys(axisCoord);
    robot.MoveJ(&jSafe, &descSafe, 1, 0, 100, 100, 100, &exaxisPos, -1, 0, &offdese);
    printf("PositionorComputeECoordSys rtn is %f %f %f %f %f %f\n", axisCoord.tran.x, axisCoord.tran.y, axisCoord.tran.z, axisCoord.rpy.rx, axisCoord.rpy.ry, axisCoord.rpy.rz);
    rtn = robot.ExtAxisActiveECoordSys(3, 3, axisCoord, 1);
    cout << "ExtAxisActiveECoordSys rtn is " << rtn << endl;
    DescPose getCoord(0, 0, 0, 0, 0, 0);
    rtn = robot.ExtAxisGetCoord(getCoord);
    printf("ExtAxisGetCoord rtn is %f %f %f %f %f %f\n", getCoord.tran.x, getCoord.tran.y, getCoord.tran.z, getCoord.rpy.rx, getCoord.rpy.ry, getCoord.rpy.rz);
    robot.CloseRPC();
    return 0;
}

int TestAuxDOAO(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    for (int i = 0; i < 128; i++)
    {
        robot.SetAuxDO(i, true, false, true);
        Sleep(100);
    }
    for (int i = 0; i < 128; i++)
    {
        robot.SetAuxDO(i, false, false, true);
        Sleep(100);
    }
    for (int i = 0; i < 409; i++)
    {
        robot.SetAuxAO(0, i * 10, true);
        robot.SetAuxAO(1, 4095 - i * 10, true);
        robot.SetAuxAO(2, i * 10, true);
        robot.SetAuxAO(3, 4095 - i * 10, true);
        Sleep(10);
    }
    robot.SetAuxDIFilterTime(10);
    robot.SetAuxAIFilterTime(0, 10);
    for (int i = 0; i < 20; i++)
    {
        bool curValue = false;
        int rtn = robot.GetAuxDI(i, false, curValue);
        cout << "DI" << i << "  " << curValue << endl;
    }
    int curValue = -1;
    for (int i = 0; i < 4; i++)
    {
        rtn = robot.GetAuxAI(i, true, curValue);
    }
    robot.WaitAuxDI(1, false, 1000, false);
    robot.WaitAuxAI(1, 1, 132, 1000, false);
    robot.CloseRPC();
    return 0;
}

#pragma endregion
#pragma region 机器人焊接

int TestWireSearch(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    DescPose toolCoord(0, 0, 200, 0, 0, 0);
    robot.SetToolCoord(1, &toolCoord, 0, 0, 1, 0);
    DescPose wobjCoord(0, 0, 0, 0, 0, 0);
    robot.SetWObjCoord(1, &wobjCoord, 0);
    int rtn0, rtn1, rtn2 = 0;
    ExaxisPos exaxisPos = { 0, 0, 0, 0 };
    DescPose offdese = { 0, 0, 0, 0, 0, 0 };
    DescPose descStart = { 441.901, 416.508, -51.979, -179.234, 0.718, -115.305 };
    JointPos jointStart = { -146.22, -60.551, 104.859, -135.317, -90.289, 59.088};
    DescPose descEnd = { 441.901, 615.317, -51.979, -179.234, 0.718, -115.305 };
    JointPos jointEnd = { -133.22, -44.193, 74.934, -121.661, -90.509, 72.087 };
    robot.MoveL(&jointStart, &descStart, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
    robot.MoveL(&jointEnd, &descEnd, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
    DescPose descREF0A = { 406.638, 347.992, -51.925, -179.229, 0.725, -115.305 };
    JointPos jointREF0A = { -150.307, -67.812, 117.086, -140.31, -90.216,  55 };
    DescPose descREF0B = { 406.638, 403.815, -51.925, -179.229, 0.725, -115.305 };
    JointPos jointREF0B = { -145.285, -63.993, 110.76, -137.771, -90.305,  60.021 };
    DescPose descREF1A = { 361.731, 357.024, -51.985, -179.235, 0.717, -115.304 };
    JointPos jointREF1A = { -146.785, -70.516, 121.407, -141.902, -90.278, 58.521 };
    DescPose descREF1B = { 361.731, 399.681, -51.985, -179.235, 0.717, -115.304 };
    JointPos jointREF1B = { -142.858, -67.39, 116.395, -139.995, -90.347, 62.449 };
    rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
    robot.MoveL(&jointREF0A, &descREF0A, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese); //起点
    robot.MoveL(&jointREF0B, &descREF0B, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese); //方向点
    rtn1 = robot.WireSearchWait("REF0");
    rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);
    rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
    robot.MoveL(&jointREF1A, &descREF1A, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese); //起点
    robot.MoveL(&jointREF1B, &descREF1B, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese); //方向点
    rtn1 = robot.WireSearchWait("REF1");
    rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);

    robot.Sleep(5000);

    rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
    robot.MoveL(&jointREF0A, &descREF0A, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese); //起点
    robot.MoveL(&jointREF0B, &descREF0B, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese); //方向点
    rtn1 = robot.WireSearchWait("RES0");
    rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);
    rtn0 = robot.WireSearchStart(0, 10, 100, 0, 10, 100, 0);
    robot.MoveL(&jointREF1A, &descREF1A, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese); //起点
    robot.MoveL(&jointREF1B, &descREF1B, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese); //方向点
    rtn1 = robot.WireSearchWait("RES1");
    rtn2 = robot.WireSearchEnd(0, 10, 100, 0, 10, 100, 0);
    vector <string> varNameRef = { "REF0", "REF1", "#", "#", "#", "#" };
    vector <string> varNameRes = { "RES0", "RES1", "#", "#", "#", "#" };
    int offectFlag = 0;
    DescPose offectPos = { 0, 0, 0, 0, 0, 0 };
    rtn0 = robot.GetWireSearchOffset(0, 0, varNameRef, varNameRes, offectFlag, offectPos);
    printf("offset is %f %f %f\n", offectPos.tran.x, offectPos.tran.y, offectPos.tran.z);
    robot.PointsOffsetEnable(0, &offectPos);
    robot.MoveL(&jointStart, &descStart, 1, 1, 100, 100, 100, -1, &exaxisPos, 0, 0, &offdese);
    robot.MoveL(&jointEnd, &descEnd, 1, 1, 100, 100, 100, -1, &exaxisPos, 1, 0, &offdese);
    robot.PointsOffsetDisable();
    robot.CloseRPC();
    return 0;
}

int TestSetWeldParam(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    robot.WeldingSetProcessParam(1, 177, 27, 1000, 178, 28, 176, 26, 1000);
    robot.WeldingSetProcessParam(2, 188, 28, 555, 199, 29, 133, 23, 333);
    double startCurrent = 0;
    double startVoltage = 0;
    double startTime = 0;
    double weldCurrent = 0;
    double weldVoltage = 0;
    double endCurrent = 0;
    double endVoltage = 0;
    double endTime = 0;
    robot.WeldingGetProcessParam(1, startCurrent, startVoltage, startTime, weldCurrent, weldVoltage, endCurrent, endVoltage, endTime);
    cout << "the Num 1 process param is " << startCurrent << " " << startVoltage << " " << startTime << " " << weldCurrent << " " << weldVoltage << " " << endCurrent << " " << endVoltage << " " << endTime << endl;
    robot.WeldingGetProcessParam(2, startCurrent, startVoltage, startTime, weldCurrent, weldVoltage, endCurrent, endVoltage, endTime);
    cout << "the Num 2 process param is " << startCurrent << " " << startVoltage << " " << startTime << " " << weldCurrent << " " << weldVoltage << " " << endCurrent << " " << endVoltage << " " << endTime << endl;
    rtn = robot.WeldingSetCurrentRelation(0, 400, 0, 10, 0);
    cout << "WeldingSetCurrentRelation rtn is: " << rtn << endl;
    rtn = robot.WeldingSetVoltageRelation(0, 40, 0, 10, 1);
    cout << "WeldingSetVoltageRelation rtn is: " << rtn << endl;
    double current_min = 0;
    double current_max = 0;
    double vol_min = 0;
    double vol_max = 0;
    double output_vmin = 0;
    double output_vmax = 0;
    int curIndex = 0;
    int volIndex = 0;
    rtn = robot.WeldingGetCurrentRelation(&current_min, &current_max, &output_vmin, &output_vmax, &curIndex);
    cout << "WeldingGetCurrentRelation rtn is: " << rtn << endl;
    cout << "current min " << current_min << " current max " << current_max << " output vol min " << output_vmin << " output vol max " << output_vmax << endl;
    rtn = robot.WeldingGetVoltageRelation(&vol_min, &vol_max, &output_vmin, &output_vmax, &volIndex);
    cout << "WeldingGetVoltageRelation rtn is: " << rtn << endl;
    cout << "vol min " << vol_min << " vol max " << vol_max << " output vol min " << output_vmin << " output vol max " << output_vmax << endl;
    rtn = robot.WeldingSetCurrent(0, 100, 0, 0);
    cout << "WeldingSetCurrent rtn is: " << rtn << endl;
    this_thread::sleep_for(chrono::seconds(3));
    rtn = robot.WeldingSetVoltage(0, 10, 0, 0);
    cout << "WeldingSetVoltage rtn is: " << rtn << endl;
    rtn = robot.WeaveSetPara(0, 0, 2.000000, 0, 10.000000, 0.000000, 0.000000, 0, 0, 0, 0, 0, 60.000000);
    cout << "rtn is: " << rtn << endl;
    robot.WeaveOnlineSetPara(0, 0, 1, 0, 20, 0, 0, 0, 0);
    rtn = robot.WeldingSetCheckArcInterruptionParam(1, 200);
    printf("WeldingSetCheckArcInterruptionParam  %d\n", rtn);
    rtn = robot.WeldingSetReWeldAfterBreakOffParam(1, 5.7, 98.2, 0);
    printf("WeldingSetReWeldAfterBreakOffParam  %d\n", rtn);
    int enable = 0;
    double length = 0;
    double velocity = 0;
    int moveType = 0;
    int checkEnable = 0;
    int arcInterruptTimeLength = 0;
    rtn = robot.WeldingGetCheckArcInterruptionParam(&checkEnable, &arcInterruptTimeLength);
    printf("WeldingGetCheckArcInterruptionParam checkEnable %d  arcInterruptTimeLength %d\n", checkEnable, arcInterruptTimeLength);
    rtn = robot.WeldingGetReWeldAfterBreakOffParam(&enable, &length, &velocity, &moveType);
    printf("WeldingGetReWeldAfterBreakOffParam enable = %d, length = %lf, velocity = %lf, moveType = %d\n", enable, length, velocity, moveType);
    robot.SetWeldMachineCtrlModeExtDoNum(17);
    for (int i = 0; i < 5; i++)
    {
        int getCtrlMode = -1;
        robot.SetWeldMachineCtrlMode(0);
        robot.GetWeldMachineCtrlMode(getCtrlMode);
        printf("GetWeldMachineCtrlMode %d\n", getCtrlMode);
        robot.Sleep(1000);
        robot.SetWeldMachineCtrlMode(1);
        robot.GetWeldMachineCtrlMode(getCtrlMode);
        printf("GetWeldMachineCtrlMode %d\n", getCtrlMode);
        robot.Sleep(1000);
    }
    robot.CloseRPC();
    return 0;
}

int TestWelding(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    robot.SetForwardWireFeed(0, 1);
    robot.Sleep(1000);
    robot.SetForwardWireFeed(0, 0);
    robot.SetReverseWireFeed(0, 1);
    robot.Sleep(1000);
    robot.SetReverseWireFeed(0, 0);
    robot.SetAspirated(0, 1);
    robot.Sleep(1000);
    robot.SetAspirated(0, 0);
    robot.WeldingSetCurrent(1, 230, 0, 0);
    robot.WeldingSetVoltage(1, 24, 0, 1);
    DescPose p1Desc(441.901, 416.508, -51.979, -179.234, 0.718, -115.305);
    JointPos p1Joint(-146.22, -60.551, 104.859, -135.317, -90.289, 59.088);

    DescPose p2Desc(441.901, 615.317, -51.979, -179.234, 0.718, -115.305);
    JointPos p2Joint(-133.22, -44.193, 74.934, -121.661, -90.509, 72.087);
    ExaxisPos exaxisPos(0, 0, 0, 0);
    DescPose offdese(0, 0, 0, 0, 0, 0);
    robot.MoveJ(&p1Joint, &p1Desc, 1, 0, 20, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.ARCStart(0, 0, 10000);
    robot.WeaveStart(0);
    robot.MoveL(&p2Joint, &p2Desc, 1, 0, 20, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);
    robot.ARCEnd(0, 0, 10000);
    robot.WeaveEnd(0);
    robot.WeldingStartReWeldAfterBreakOff();
    robot.WeldingAbortWeldAfterBreakOff();
    robot.CloseRPC();
    return 0;
}

int TestSegWeld(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    robot.WeldingSetCurrent(1, 230, 0, 0);
    robot.WeldingSetVoltage(1, 24, 0, 1);



    DescPose p1Desc(441.901, 416.508, -51.979, -179.234, 0.718, -115.305);
    JointPos p1Joint(-146.22, -60.551, 104.859, -135.317, -90.289, 59.088);

    DescPose p2Desc(441.901, 615.317, -51.979, -179.234, 0.718, -115.305);
    JointPos p2Joint(-133.22, -44.193, 74.934, -121.661, -90.509, 72.087);
    ExaxisPos exaxisPos(0, 0, 0, 0);
    DescPose offdese(0, 0, 0, 0, 0, 0);
    rtn = robot.SegmentWeldStart(&p1Desc, &p2Desc, &p1Joint, &p2Joint, 20, 20, 0, 0, 5000, true, 2, 1, 0, 100, 100, 10, -1, &exaxisPos, 0, 0, &offdese);
    printf("SegmentWeldStart rtn is %d\n", rtn);
    robot.CloseRPC();
    return 0;
}


int TestWeave(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    DescPose p1Desc(441.901, 416.508, -51.979, -179.234, 0.718, -115.305);
    JointPos p1Joint(-146.22, -60.551, 104.859, -135.317, -90.289, 59.088);

    DescPose p2Desc(441.901, 615.317, -51.979, -179.234, 0.718, -115.305);
    JointPos p2Joint(-133.22, -44.193, 74.934, -121.661, -90.509, 72.087);
    ExaxisPos exaxisPos(0, 0, 0, 0);
    DescPose offdese(0, 0, 0, 0, 0, 0);
    robot.MoveJ(&p1Joint, &p1Desc, 1, 0, 20, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.WeaveStartSim(0);
    robot.MoveL(&p2Joint, &p2Desc, 1, 0, 20, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);
    robot.WeaveEndSim(0);
    robot.MoveJ(&p1Joint, &p1Desc, 1, 0, 20, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot.WeaveInspectStart(0);
    robot.MoveL(&p2Joint, &p2Desc, 1, 0, 20, 100, 100, -1, 0, &exaxisPos, 0, 0, &offdese);
    robot.WeaveInspectEnd(0);
    robot.WeldingSetVoltage(0, 19, 0, 0);
    robot.WeldingSetCurrent(0, 190, 0, 0);
    robot.MoveL(&p1Joint, &p1Desc, 1, 0, 100, 100, 50, -1, &exaxisPos, 0, 0, &offdese);
    robot.ARCStart(0, 0, 10000);
    robot.ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0);
    robot.WeaveStart(0);
    robot.WeaveChangeStart(1, 1, 50, 30);
    robot.MoveL(&p2Joint, &p2Desc, 1, 0, 100, 100, 10, -1, &exaxisPos, 0, 0, &offdese);
    robot.WeaveChangeEnd();
    robot.WeaveEnd(0);
    robot.ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 60, 1, 0.06, 5, 5, 80, 0, 0, 4, 1, 10, 0, 0);
    robot.ARCEnd(0, 0, 10000);
    robot.CloseRPC();
    return 0;
}

int TestExtDIConfig(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    robot.SetArcStartExtDoNum(10);
    robot.SetAirControlExtDoNum(20);
    robot.SetWireForwardFeedExtDoNum(30);
    robot.SetWireReverseFeedExtDoNum(40);
    robot.SetWeldReadyExtDiNum(50);
    robot.SetArcDoneExtDiNum(60);
    robot.SetExtDIWeldBreakOffRecover(70, 80);
    robot.SetWireSearchExtDIONum(0, 1);

    int DIConfig[16] = { 0 };
    int DOConfig[16] = { 0 };
    rtn = robot.GetExtDIConfig(DIConfig);
    printf("GetExtDIConfig rtn is %d\n welder ready %d\narc done %d\nreweld start %d\nabort reweld %d\nwiresearch done %d\nLaser welding State %d\nlaser welding error state %d\n",
        rtn, DIConfig[0], DIConfig[1], DIConfig[2], DIConfig[3], DIConfig[4], DIConfig[5], DIConfig[6]);

    rtn = robot.GetExtDOConfig(DOConfig);
    printf("GetExtDOConfig rtn is %d\n Arc Start %d\nAir Test %d\nWire forward %d\nWire Inverse %d\nwiresearch %d\nWeld Mode %d\nlaser Enable %d\nLaser On %d\nLaser Reset Error %d\n",
        rtn, DOConfig[0], DOConfig[1], DOConfig[2], DOConfig[3], DOConfig[4], DOConfig[5], DOConfig[6], DOConfig[7], DOConfig[8]);
    robot.CloseRPC();
    return 0;
}

int TestArcWeldTrace(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }

    robot.SetReConnectParam(true, 30000, 500);
    JointPos mulitilineorigin1_joint(-146.22, -60.551, 104.859, -135.317, -90.289, 59.088);
    DescPose mulitilineorigin1_desc(441.901, 416.508, -51.979, -179.234, 0.718, -115.305);
    DescTran mulitilineX1_desc;
    mulitilineX1_desc.x = 541.901;
    mulitilineX1_desc.y = 416.508;
    mulitilineX1_desc.z = -51.979;
    DescTran mulitilineZ1_desc;
    mulitilineZ1_desc.x = 441.901;
    mulitilineZ1_desc.y = 416.508;
    mulitilineZ1_desc.z = 10.979;
    JointPos mulitilinesafe_joint(-138.179, -55.975, 88.096, -123.081, -90.426, 67.129);
    DescPose mulitilinesafe_desc(439.754, 527.356, -4.026, -179.234, 0.719, -115.306);
    JointPos mulitilineorigin2_joint(-133.22, -44.193, 74.934, -121.661, -90.509, 72.087);
    DescPose mulitilineorigin2_desc(441.901, 615.317, -51.979, -179.234, 0.718, -115.305);
    DescTran mulitilineX2_desc;
    mulitilineX2_desc.x = 541.901;
    mulitilineX2_desc.y = 615.317;
    mulitilineX2_desc.z = -51.979;
    DescTran mulitilineZ2_desc;
    mulitilineZ2_desc.x = 441.901;
    mulitilineZ2_desc.y = 615.317;
    mulitilineZ2_desc.z = 10.979;
    ExaxisPos epos(0, 0, 0, 0);
    DescPose offset(0, 0, 0, 0, 0, 0);
    robot.Sleep(10);
    int error = robot.MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 1, 0, 10, 100, 100, &epos, -1, 0, &offset);
    printf("MoveJ return: %d\n", error);
    error = robot.MoveL(&mulitilineorigin1_joint, &mulitilineorigin1_desc, 1, 0, 10, 100, 100, -1, &epos, 0, 0, &offset, 0, 100);
    printf("MoveL return: %d\n", error);
    error = robot.MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 1, 0, 10, 100, 100, &epos, -1, 0, &offset);
    printf("MoveJ return: %d\n", error);
    error = robot.MoveL(&mulitilineorigin2_joint, &mulitilineorigin2_desc, 1, 0, 10, 100, 100, -1, &epos, 0, 0, &offset, 0, 100);
    printf("MoveL return: %d\n", error);
    error = robot.MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 1, 0, 10, 100, 100, &epos, -1, 0, &offset);
    printf("MoveJ return: %d\n", error);
    error = robot.MoveL(&mulitilineorigin1_joint, &mulitilineorigin1_desc, 1, 0, 10, 100, 100, -1, &epos, 0, 0, &offset, 0, 100);
    printf("MoveL return: %d\n", error);
    error = robot.ARCStart(1, 0, 3000);
    printf("ARCStart return: %d\n", error);
    error = robot.WeaveStart(2);
    printf("WeaveStart return: %d\n", error);
    error = robot.ArcWeldTraceControl(1, 0, 1, 0.06, 5, 5, 50, 1, 0.06, 5, 5, 55, 0, 0, 4, 1, 10);
    printf("ArcWeldTraceControl return: %d\n", error);
    error = robot.MoveL(&mulitilineorigin2_joint, &mulitilineorigin2_desc, 1, 0, 10, 100, 100, -1, &epos, 0, 0, &offset, 0, 100);
    printf("MoveL return: %d\n", error);
    error = robot.ArcWeldTraceControl(0, 0, 1, 0.06, 5, 5, 50, 1, 0.06, 5, 5, 55, 0, 0, 4, 1, 10);
    printf("ArcWeldTraceControl return: %d\n", error);
    error = robot.WeaveEnd(2);
    printf("WeaveEnd return: %d\n", error);
    error = robot.ARCEnd(1, 0, 10000);
    printf("ARCEnd return: %d\n", error);
    error = robot.MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 1, 0, 10, 100, 100, &epos, -1, 0, &offset);
    printf("MoveJ return: %d\n", error);
    error = robot.MultilayerOffsetTrsfToBase(mulitilineorigin1_desc.tran, mulitilineX1_desc, mulitilineZ1_desc, 10.0, 0.0, 0.0, offset);
    printf("MultilayerOffsetTrsfToBase return: %d offect is %f %f %f \n", error, offset.tran.x, offset.tran.y, offset.tran.z);
    error = robot.MoveL(&mulitilineorigin1_joint, &mulitilineorigin1_desc, 1, 0, 10, 100, 100, -1, &epos, 0, 1, &offset, 0, 100);
    printf("MoveL return: %d\n", error);
    error = robot.ARCStart(1, 0, 3000);
    error = robot.WeaveStart(2);
    printf("ARCStart return: %d\n", error);
    error = robot.MultilayerOffsetTrsfToBase(mulitilineorigin2_desc.tran, mulitilineX2_desc, mulitilineZ2_desc, 10, 0, 0, offset);
    printf("MultilayerOffsetTrsfToBase return: %d offect is %f %f %f \n", error, offset.tran.x, offset.tran.y, offset.tran.z);
    error = robot.ArcWeldTraceReplayStart();
    printf("ArcWeldTraceReplayStart return: %d\n", error);
    error = robot.MoveL(&mulitilineorigin2_joint, &mulitilineorigin2_desc, 1, 0, 10, 100, 100, -1, &epos, 0, 1, &offset, 0, 100);
    printf("MoveL return: %d\n", error);
    error = robot.ArcWeldTraceReplayEnd();
    printf("ArcWeldTraceReplayEnd return: %d\n", error);
    error = robot.ARCEnd(1, 0, 10000);
    error = robot.WeaveEnd(2);
    printf("ARCEnd return: %d\n", error);
    error = robot.MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 1, 0, 10, 100, 100, &epos, -1, 0, &offset);
    printf("MoveJ return: %d\n", error);
    error = robot.MultilayerOffsetTrsfToBase(mulitilineorigin1_desc.tran, mulitilineX1_desc, mulitilineZ1_desc, 0, 10, 0, offset);
    printf("MultilayerOffsetTrsfToBase return: %d offect is %f %f %f \n", error, offset.tran.x, offset.tran.y, offset.tran.z);
    error = robot.MoveL(&mulitilineorigin1_joint, &mulitilineorigin1_desc, 1, 0, 10, 100, 100, -1, &epos, 0, 1, &offset, 0, 100);
    printf("MoveL return: %d\n", error);
    error = robot.ARCStart(1, 0, 3000);
    printf("ARCStart return: %d\n", error);
    error = robot.WeaveStart(2);
    error = robot.MultilayerOffsetTrsfToBase(mulitilineorigin2_desc.tran, mulitilineX2_desc, mulitilineZ2_desc, 0, 10, 0, offset);
    printf("MultilayerOffsetTrsfToBase return: %d offect is %f %f %f \n", error, offset.tran.x, offset.tran.y, offset.tran.z);
    error = robot.ArcWeldTraceReplayStart();
    printf("MoveJ return: %d\n", error);
    error = robot.MoveL(&mulitilineorigin2_joint, &mulitilineorigin2_desc, 1, 0, 10, 100, 100, -1, &epos, 0, 1, &offset, 0, 100);
    printf("MoveL return: %d\n", error);
    error = robot.ArcWeldTraceReplayEnd();
    printf("ArcWeldTraceReplayEnd return: %d\n", error);
    error = robot.ARCEnd(1, 0, 3000);
    error = robot.WeaveEnd(2);
    printf("ARCEnd return: %d\n", error);
    error = robot.MoveJ(&mulitilinesafe_joint, &mulitilinesafe_desc, 1, 0, 10, 100, 100, &epos, -1, 0, &offset);
    printf("MoveJ return: %d\n", error);
    robot.CloseRPC();
    return 0;
}

int WeldTraceControlWithCtrlBoxAI(FRRobot* robot)
{
    DescPose startdescPose = { 441.901, 416.508, -51.979, -179.234, 0.718, -115.305 };
    JointPos startjointPos = { -146.22, -60.551, 104.859, -135.317, -90.289, 59.088 };

    DescPose enddescPose = { 441.901, 615.317, -51.979, -179.234, 0.718, -115.305 };
    JointPos endjointPos = { -133.22, -44.193, 74.934, -121.661, -90.509, 72.087 };

    DescPose safedescPose = { 439.754, 527.356, -4.026, -179.234, 0.719, -115.306 };
    JointPos safejointPos = { -138.179, -55.975, 88.096, -123.081, -90.426, 67.129 };

    ExaxisPos exaxisPos = { 0, 0, 0, 0 };
    DescPose offdese = { 0, 0, 0, 0, 0, 0 };

    robot->WeldingSetCurrentRelation(0, 495, 1, 10, 0);
    robot->WeldingSetVoltageRelation(10, 45, 1, 10, 1);

    robot->WeldingSetVoltage(0, 25, 1, 0);// ----设置电压
    robot->WeldingSetCurrent(0, 260, 0, 0);// ----设置电流

    int rtn = robot->ArcWeldTraceAIChannelCurrent(4);
    cout << "ArcWeldTraceAIChannelCurrent rtn is " << rtn << endl;
    rtn = robot->ArcWeldTraceAIChannelVoltage(5);
    cout << "ArcWeldTraceAIChannelVoltage rtn is " << rtn << endl;
    rtn = robot->ArcWeldTraceCurrentPara(0, 5, 0, 500);
    cout << "ArcWeldTraceCurrentPara rtn is " << rtn << endl;
    rtn = robot->ArcWeldTraceVoltagePara(1.018, 10, 0, 50);
    cout << "ArcWeldTraceVoltagePara rtn is " << rtn << endl;
    robot->MoveJ(&safejointPos, &safedescPose, 1, 0, 10, 100, 100, &exaxisPos, -1, 0, &offdese);
    robot->MoveJ(&startjointPos, &startdescPose, 1, 0, 10, 100, 100, &exaxisPos, -1, 0, &offdese);
    rtn = robot->ArcWeldTraceControl(1, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0);
    cout << "ArcWeldTraceControl start rtn is " << rtn << endl;
    robot->ARCStart(0, 0, 10000);
    robot->WeaveStart(2);
    robot->MoveL(&endjointPos, &enddescPose, 1, 0, 100, 100, 10, -1, &exaxisPos, 0, 0, &offdese);
    robot->ARCEnd(0, 0, 10000);
    robot->WeaveEnd(2);
    rtn = robot->ArcWeldTraceControl(0, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0);
    cout << "ArcWeldTraceControl end rtn is " << rtn << endl;
    return 0;
}

int WeldparamChange(FRRobot* robot)
{
    DescPose startdescPose = { 441.901, 416.508, -51.979, -179.234, 0.718, -115.305 };
    JointPos startjointPos = { -146.22, -60.551, 104.859, -135.317, -90.289, 59.088 };

    DescPose enddescPose = { 441.901, 615.317, -51.979, -179.234, 0.718, -115.305 };
    JointPos endjointPos = { -133.22, -44.193, 74.934, -121.661, -90.509, 72.087 };

    DescPose safedescPose = { 439.754, 527.356, -4.026, -179.234, 0.719, -115.306 };
    JointPos safejointPos = { -138.179, -55.975, 88.096, -123.081, -90.426, 67.129 };

    ExaxisPos exaxisPos = { 0, 0, 0, 0 };
    DescPose offdese = { 0, 0, 0, 0, 0, 0 };

    robot->WeldingSetCurrentRelation(0, 495, 1, 10, 0);
    robot->WeldingSetVoltageRelation(10, 45, 1, 10, 1);
    robot->MoveJ(&safejointPos, &safedescPose, 1, 0, 5, 100, 100, &exaxisPos, -1, 0, &offdese);
    int rtn = robot->WeldingSetCurrentGradualChangeStart(0, 260, 220, 0, 0);
    cout << "WeldingSetCurrentGradualChangeStart rtn is " << rtn << endl;
    rtn = robot->WeldingSetVoltageGradualChangeStart(0, 25, 22, 1, 0);
    cout << "WeldingSetVoltageGradualChangeStart rtn is " << rtn << endl;
    rtn = robot->ArcWeldTraceControl(1, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0);
    cout << "ArcWeldTraceControl rtn is " << rtn << endl;
    robot->MoveJ(&startjointPos, &startdescPose, 1, 0, 5, 100, 100, &exaxisPos, -1, 0, &offdese);

    robot->ARCStart(0, 0, 10000);
    robot->WeaveStart(0);
    robot->WeaveChangeStart(2, 1, 24, 36);
    robot->MoveL(&endjointPos, &enddescPose, 1, 0, 100, 100, 2, -1, &exaxisPos, 0, 0, &offdese);
    robot->ARCEnd(0, 0, 10000);
    robot->WeaveChangeEnd();
    robot->WeaveEnd(0);
    robot->ArcWeldTraceControl(0, 0, 1, 0.08, 5, 5, 300, 1, 0.06, 4, 4, 300, 1, 0, 4, 1, 10, 0, 0);
    robot->WeldingSetCurrentGradualChangeEnd();
    robot->WeldingSetVoltageGradualChangeEnd();
    return 0;
}

int TestCustomWeaveSetPara()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return 0;
    }
    robot.SetReConnectParam(true, 30000, 500);
    DescTran point[10] = {};
    point[0].x = -3;
    point[0].y = -3;
    point[0].z = 0;
    point[1].x = -6;
    point[1].y = 0;
    point[1].z = 0;
    point[2].x = -3;
    point[2].y = 3;
    point[2].z = 0;
    point[3].x = 0;
    point[3].y = 0;
    point[3].z = 0;
    double stayTime[10] = { 0,0,0,0,0,0,0,0,0,0 };
    rtn = robot.CustomWeaveSetPara(2, 4, point, stayTime, 1.000, 0, 0);
    printf("CustomWeaveSetPara rtn is %d\n", rtn);
    robot.Sleep(1000);
    int pointNum = 0;
    double frequency;
    int incStayType;
    int stationary;
    robot.CustomWeaveGetPara(2, pointNum, point, stayTime, frequency, incStayType, stationary);
    printf("pointNum is %d\n", pointNum);
    for (int i = 0; i < pointNum; i++)
    {
        printf("point %d, point x y z %f %f %f\n", i, point[i].x, point[i].y, point[i].z);
    }
    printf("fre is %f, stay is %d %d \n", frequency, incStayType, stationary);
    robot.WeaveSetPara(0, 9, 1.000000, 1, 5.000000, 6.000000, 5.000000, 50, 100, 100, 0, 1, 0.000000, 0.000000);



    DescPose desc_p1(441.901, 416.508, -51.979, -179.234, 0.718, -115.305);
    JointPos j1(-146.22, -60.551, 104.859, -135.317, -90.289, 59.088);
    DescPose desc_p2(441.901, 615.317, -51.979, -179.234, 0.718, -115.305);
    JointPos j2(-133.22, -44.193, 74.934, -121.661, -90.509, 72.087);
    DescPose desc_p3(291.781, 682.326, -51.976, -179.234, 0.718, -115.305);
    JointPos j3(-120.770, -45.957, 78.232, -123.063, -90.694, 84.535);

    ExaxisPos epos = {};
    DescPose offset_pos = {};
    robot.MoveJ(&j1, &desc_p1, 1, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.WeaveStart(0);
    robot.Circle(&j3, &desc_p3, 1, 0, 100, 100, &epos, &j2, &desc_p2, 1, 0, 100, 100, &epos, 10, -1, &offset_pos);
    robot.WeaveEnd(0);
    robot.MoveJ(&j1, &desc_p1, 1, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.WeaveStart(0);
    robot.MoveC(&j3, &desc_p3, 1, 0, 100, 100, &epos, 0, &offset_pos, &j2, &desc_p2, 1, 0, 100, 100, &epos, 0, &offset_pos, 10, -1);
    robot.WeaveEnd(0);
    robot.MoveJ(&j1, &desc_p1, 1, 0, 100, 100, 100, &epos, -1, 0, &offset_pos);
    robot.WeaveStart(0);
    robot.MoveL(&j2, &desc_p2, 1, 0, 100, 100, 10, -1, &epos, 0, 0, &offset_pos, 0, 100);
    robot.WeaveEnd(0);
    robot.CloseRPC();
}

int TestLaserWeld()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    robot.SetReConnectParam(true, 300000, 500);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    rtn = robot.ExtDevLoadUDPDriver();
    if (rtn != 0)
    {
        std::cout << "Failed to load UDP driver, error code: " << rtn << std::endl;
    }
    robot.Sleep(1000);
    rtn = robot.SetLaserWeldingParam(1, 3, 2000, 3, 1500, 100, 1000);
    if (rtn != 0)
    {
        std::cout << "SetLaserWeldingParam failed, error code: " << rtn << std::endl;
    }
    else
    {
        std::cout << "SetLaserWeldingParam success" << std::endl;
    }
    rtn = robot.SetLaserWeldingStartExtDoNum(1);
    if (rtn != 0)
    {
        std::cout << "SetLaserWeldingStartExtDoNum failed, error code: " << rtn << std::endl;
    }
    rtn = robot.Mode(0);
    if (rtn != 0)
    {
        std::cout << "Set mode 0 failed, error code: " << rtn << std::endl;
    }
    robot.Sleep(1000);


    DescPose desc_pos1 = { 441.901, 416.508, -51.979, -179.234, 0.718, -115.305 };
    JointPos jointPos1 = { -146.22, -60.551, 104.859, -135.317, -90.289, 59.088 };

    DescPose desc_pos2 = { 441.901, 615.317, -51.979, -179.234, 0.718, -115.305 };
    JointPos jointPos2 = { -133.22, -44.193, 74.934, -121.661, -90.509, 72.087 };

    DescPose desc_safe = { 439.754, 527.356, -4.026, -179.234, 0.719, -115.306 };
    JointPos jointSafe = { -138.179, -55.975, 88.096, -123.081, -90.426, 67.129 };

    ExaxisPos exaxis(0.0, 0.0, 0.0, 0.0);
    DescPose offset(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    int error = robot.MoveL(&desc_pos1, 1, 0, 100, 100, 100, -1, 0, &exaxis, 0, 0, &offset, -1, 0);
    std::cout << "MoveL to pos1 return: " << error << std::endl;
    rtn = robot.SetLaserWeldingStartEnd(1, 1, 10000);
    if (rtn != 0)
    {
        std::cout << "SetLaserWeldingStartEnd (start) failed, error code: " << rtn << std::endl;
    }
    else
    {
        std::cout << "Laser started" << std::endl;
    }
    rtn = robot.MoveL(&desc_pos2, 1, 0, 30, 100, 100, -1, 0, &exaxis, 0, 0, &offset, -1, 0);
    std::cout << "MoveL to pos2 return: " << rtn << std::endl;
    rtn = robot.SetLaserWeldingStartEnd(1, 0, 10000);
    if (rtn != 0)
    {
        std::cout << "SetLaserWeldingStartEnd (stop) failed, error code: " << rtn << std::endl;
    }
    else
    {
        std::cout << "Laser stopped" << std::endl;
    }
    robot.Sleep(500);
    rtn = robot.MoveL(&desc_safe, 1, 0, 100, 100, 100, -1, 0, &exaxis, 0, 0, &offset, -1, 0);
    std::cout << "MoveL to safe_pos return: " << rtn << std::endl;
    rtn = robot.Mode(1);
    if (rtn != 0)
    {
        std::cout << "Set mode 1 failed, error code: " << rtn << std::endl;
    }
    robot.Sleep(1000);
    robot.CloseRPC();
    robot.Sleep(1000);
    std::cout << "Test completed" << std::endl;
    return 0;
}

#pragma endregion
#pragma region 其他接口

int TestSSHMd5(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    char file_path[256] = "/fruser/test.lua";
    char md5[256] = { 0 };
    uint8_t emerg_state = 0;
    uint8_t si0_state = 0;
    uint8_t si1_state = 0;
    int sdk_com_state = 0;
    char ssh_keygen[1024] = { 0 };
    int retval = robot.GetSSHKeygen(ssh_keygen);
    printf("GetSSHKeygen retval is: %d\n", retval);
    printf("ssh key is: %s \n", ssh_keygen);
    char ssh_name[32] = "fr";
    char ssh_ip[32] = "192.168.58.45";
    char ssh_route[128] = "D://zDOWN/";
    char ssh_robot_url[128] = "/usr/local/etc/controller/dhpara.config";
    retval = robot.SetSSHScpCmd(1, ssh_name, ssh_ip, ssh_route, ssh_robot_url);
    printf("SetSSHScpCmd retval is: %d\n", retval);
    printf("robot url is: %s\n", ssh_robot_url);
    robot.ComputeFileMD5(file_path, md5);
    printf("md5 is: %s \n", md5);
    robot.CloseRPC();
    return 0;
}


int TestRealtimePeriod(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    robot.SetRobotRealtimeStateSamplePeriod(20);
    int getPeriod = 0;
    robot.GetRobotRealtimeStateSamplePeriod(getPeriod);
    cout << "period is " << getPeriod << endl;
    robot.Sleep(1000);
    robot.CloseRPC();
    return 0;
}


int TestUpgrade(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(3);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    robot.SoftwareUpgrade("C://Users/fr/Desktop/software.tar.gz", false);
    while (true)
    {
        int curState = -1;
        robot.GetSoftwareUpgradeState(curState);
        printf("upgrade state is %d\n", curState);
        robot.Sleep(300);
    }
    robot.CloseRPC();
    return 0;
}

int TestPointTable(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    string save_path = "D://zDOWN/";
    string point_table_name = "point_table_FR5.db";
    rtn = robot.PointTableDownLoad(point_table_name, save_path);
    cout << "download : " << point_table_name << " fail: " << rtn << endl;
    string upload_path = "D://zUP/point_table_FR5.db";
    rtn = robot.PointTableUpLoad(upload_path);
    cout << "PointTableUpLoad retval is: " << rtn << endl;
    string point_tablename = "point_table_FR5.db";
    string lua_name = "airlab.lua";
    rtn = robot.PointTableUpdateLua(point_tablename, lua_name);
    cout << "retval is: " << rtn << endl;
    robot.CloseRPC();
    return 0;
}

int TestDownLoadRobotData(void)
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    rtn = robot.RbLogDownload("D://zDOWN/");
    cout << "RbLogDownload rtn is " << rtn << endl;
    rtn = robot.AllDataSourceDownload("D://zDOWN/");
    cout << "AllDataSourceDownload rtn is " << rtn << endl;
    rtn = robot.DataPackageDownload("D://zDOWN/");
    cout << "DataPackageDownload rtn is " << rtn << endl;
    robot.CloseRPC();
    return 0;
}

int TestFirmWareUpgrade()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    robot.RobotEnable(0);
    robot.Sleep(200);
    rtn = robot.JointAllParamUpgrade("D://zUP/upgrade/jointallparameters.db");
    printf("robot JointAllParamUpgrade rtn is %d\n", rtn);
    rtn = robot.SetCtrlFirmwareUpgrade(2, "D://zUP/upgrade/FAIR_Cobot_Cbd_Asix_V2.0.bin");
    printf("robot SetCtrlFirmwareUpgrade config param rtn is %d\n", rtn);
    rtn = robot.SetEndFirmwareUpgrade(2, "D://zUP/upgrade/FAIR_Cobot_Axle_Asix_V2.4.bin");
    printf("robot SetEndFirmwareUpgrade config param rtn is %d\n", rtn);
    robot.SetSysServoBootMode();
    rtn = robot.SetCtrlFirmwareUpgrade(1, "D://zUP/upgrade/FR_CTRL_PRIMCU_FV201013_MAIN_U4_T01_20260424.bin");
    printf("robot SetCtrlFirmwareUpgrade rtn is %d\n", rtn);
    rtn = robot.SetEndFirmwareUpgrade(1, "D://zUP/upgrade/FR_END_FV201013_MAIN_U1_T01_20260407.bin");
    printf("robot SetEndFirmwareUpgrade rtn is %d\n", rtn);
    rtn = robot.SetJointFirmwareUpgrade(1, "D://zUP/upgrade/FR_SERVO_FV504316_MAIN_U7_T07_20250715.bin");
    printf("robot SetJointFirmwareUpgrade rtn is %d\n", rtn);
    robot.CloseRPC();
    return 0;
}

int TestRobotStopOnComDisc()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    bool enable = false;
    int confirmTime = 0;
    rtn = robot.SetRobotStopOnComDisc(0, true, 330);
    rtn = robot.SetRobotStopOnComDisc(1, true, 550);
    rtn = robot.SetRobotStopOnComDisc(2, true, 110);
    rtn = robot.SetRobotStopOnComDisc(3, true, 220);
    printf("SetRobotStopOnComDisc %d\n", rtn);
    robot.GetRobotStopOnComDisc(0, enable, confirmTime);
    printf("GetRobotStopOnComDisc 8080 rtn %d; enable is %d; confirm time is %d\n", rtn, enable, confirmTime);
    robot.GetRobotStopOnComDisc(1, enable, confirmTime);
    printf("GetRobotStopOnComDisc 80803 rtn %d; enable is %d; confirm time is %d\n", rtn, enable, confirmTime);
    robot.GetRobotStopOnComDisc(2, enable, confirmTime);
    printf("GetRobotStopOnComDisc 20002 rtn %d; enable is %d; confirm time is %d\n", rtn, enable, confirmTime);
    robot.GetRobotStopOnComDisc(3, enable, confirmTime);
    printf("GetRobotStopOnComDisc 20004 rtn %d; enable is %d; confirm time is %d\n", rtn, enable, confirmTime);
    robot.CloseRPC();
    robot.Sleep(1000);
    return 0;
}

int TestSendUDPFrame()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.SetCmdRpyCallback(UDPFrameCallBack);
    printf("SetCmdRpyCallback rtn is %d\n", rtn);
    rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }
    robot.SetReConnectParam(true, 30000, 500);
    rtn = robot.SendUDPFrame("/f/bIII20III303III7IIIMode(0)III/b/f");
    printf("SendUDPFrame Mode(0) rtn is %d\n", rtn);
    robot.Sleep(1000);
    rtn = robot.SendUDPFrame("/f/bIII21III303III7IIIMode(1)III/b/f");
    printf("SendUDPFrame Mode(1) rtn is %d\n", rtn);
    robot.Sleep(1000);
    rtn = robot.SendUDPFrame("/f/bIII49III201III184IIIMoveJ(-15.625, -82.680, 101.654, -110.950, -88.290, 0.017, -383.012, -2.325, 242.655, -178.024, 1.710, 74.416, 0, 0, 100, 100, 100, 0.000, 0.000, 0.000, 0.000, -1, 0, 0, 0, 0, 0, 0, 0)III/b/f");
    printf("SendUDPFrame MoveJ(-15.625 rtn is %d\n", rtn);
    robot.Sleep(1000);
    rtn = robot.SendUDPFrame("/f/bIII48III203III199IIIMoveL(-75.622, -82.680, 101.654, -110.950, -88.290, 0.017, -193.537, 330.525, 242.657, -178.024, 1.710, 14.420, 0, 0, 100, 100, 100, -1, 0, 0.000, 0.000, 0.000, 0.000, 0, 0, 0, 0, 0, 0, 0, 0, 100, 0)III/b/f");
    printf("SendUDPFrame MoveL(-75.622 rtn is %d\n", rtn);
    robot.Sleep(1000);
    rtn = robot.SendUDPFrame("/f/bIII4III905III20IIIGetSoftwareVersion()III/b/f");
    printf("SendUDPFrame GetSoftwareVersion() rtn is %d\n", rtn);
    robot.Sleep(1000);
    rtn = robot.SendUDPFrame("/f/bIII20III303III7IIIMode(0)III/b/f");
    printf("SendUDPFrame rtn is %d\n", rtn);
    rtn = robot.SendUDPFrame("III20III303III7IIIMode(0)III/b/f");
    printf("SendUDPFrame rtn is %d\n", rtn);
    rtn = robot.SendUDPFrame("/f/bIII20III303III7IIIMode(0)");
    printf("SendUDPFrame rtn is %d\n", rtn);
    rtn = robot.SendUDPFrame("/f/bIII20III303III6IIIMode(0)III/b/f");
    printf("SendUDPFrame rtn is %d\n", rtn);
    rtn = robot.SendUDPFrame("/f/b|||20|||303|||7|||Mode(0)|||/b/f");
    printf("SendUDPFrame rtn is %d\n", rtn);
    rtn = robot.SendUDPFrame("/f/bII20II303II7IIMode(0)II/b/f");
    printf("SendUDPFrame rtn is %d\n", rtn);
    robot.Sleep(5000);
    robot.CloseRPC();
    robot.Sleep(1000);
    return 0;
}

int TestUserLedColor()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return 0;
    }
    robot.SetReConnectParam(true, 30000, 500);
    robot.SetUserLEDColor(true, true, true);
    robot.Sleep(1000);
    robot.SetUserLEDColor(false, false, false);
    robot.Sleep(1000);
    robot.SetUserLEDColor(true, false, false);
    robot.Sleep(1000);
    robot.SetUserLEDColor(false, true, false);
    robot.Sleep(1000);
    robot.SetUserLEDColor(false, false, true);
    robot.Sleep(1000);
    robot.CloseRPC();
    robot.Sleep(1000);
    return 0;
}

#pragma endregion

int TestStable()
{
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    robot.SetReConnectParam(true, 30000, 500);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }

    

    while (1)
    {
        

        robot.Sleep(2000);

       
    }

    robot.Sleep(2000);

    robot.CloseRPC();
    robot.Sleep(1000000);
}

using JointPose = std::vector<double>;

int main() 
{
    TestSetAndGetRobotTime();
    return 0;
    ROBOT_STATE_PKG pkg = {};
    FRRobot robot;
    robot.LoggerInit();
    robot.SetLoggerLevel(1);
    robot.SetReConnectParam(true, 30000, 500);
    int rtn = robot.RPC("192.168.58.2");
    if (rtn != 0)
    {
        return -1;
    }

    

    robot.Sleep(1000);
    robot.CloseRPC();
    robot.Sleep(1000);
    return 0;
}
