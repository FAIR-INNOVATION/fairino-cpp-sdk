#include "robot.h"
#include "robot_types.h"
#include "robot_error.h"

#include "logger.h"
#include "XmlRpc.h"
#include "md5.hpp"
#include "network.h"
#include <stdio.h>
#include <string.h>
#include <cstdlib>
#include <iostream>
#include <chrono>

#ifdef __MINGW32__
    #define TCP_MAXRT 5
    #include <mingw.thread.h>
#else
    #include <thread>
#endif

#include <fstream>
#include <sstream>
#include <iomanip>
#include <FRTcpClient.h>



#ifdef WIN32
    #include <fileapi.h>
    #include <handleapi.h>
    #include <Shlwapi.h>
    #include <WS2tcpip.h>

    #define SDK_VERSION "SDK V2.1"
    #pragma comment(lib, "Shlwapi.lib")
#else
    #include <stdlib.h>
    #include <unistd.h>
    #include <libgen.h>
    // #include <filesystem>
    // SDK版本号
    #define SDK_VERSION_MAJOR "2"
    #define SDK_VERSION_MINOR "2"
    #define SDK_VERSION_RELEASE "2"
    #define SDK_VERSION_RELEASE_NUM "0"
    #define SDK_VERSION "SDK V" SDK_VERSION_MAJOR "." SDK_VERSION_MINOR
#endif
#define SDK_RELEASE "SDK V2.2.3.0-robot v3.8.3"

#define ROBOT_REALTIME_PORT 20004
#define ROBOT_CMD_PORT 8080

/* 文件上传下载都是走的这个接口 */
#define DOWNLOAD_POINT_TABLE_PORT 20011
#define UPLOAD_POINT_TABLE_PORT 20010

#define BUFFER_SIZE 1024 * 4
#define MAX_CHECK_CNT_COM 25
#define POINT_TABLE_MAX_SIZE (1024 * 1024 * 2)
#define POINT_TABLE_HEAD "/f/b"
#define POINT_TABLE_TAIL "/b/f"

#define DOWNLOAD_FILE_MAX_SIZE             (1024 * 1024 * 500)
#define UPLOAD_FILE_MAX_SIZE               (1024 * 1024 * 500)
#define DOWNLOAD_FILE_TIME_LIMIT_MS 15000
#define FILE_HEAD   "/f/b"
#define FILE_TAIL   "/b/f"

#define MAX_POINT_TABLE_PATH_LENGTH 512+1
#define MAX_FILE_PATH_LENGTH 512+1

using namespace std;
using namespace XmlRpc;

/**
 * @brief  机器人接口类构造函数
 */
FRRobot::FRRobot(void)
{
    char url[64] = "192.168.58.2";
    memset(serverUrl, 0, 64);
    strncpy(serverUrl, url, strlen(url));

#ifdef LINUX_OPTION
    char default_ip[64] = "192.168.58.2";
    memset(robot_ip, 0, 64);
    strncpy(robot_ip, default_ip, strlen(default_ip));
#endif

    robot_state_pkg = std::make_shared<ROBOT_STATE_PKG>();
    memset(robot_state_pkg.get(), 0, sizeof(ROBOT_STATE_PKG));
    robot_instcmd_recv_exit = 0;
    robot_instcmd_send_exit = 0;
    robot_realstate_exit = 0;
    robot_task_exit = 0;
    g_sock_com_err = ERR_SUCCESS;
    rtClient = std::make_shared<FRTcpClient>(robot_ip, ROBOT_REALTIME_PORT);
    cmdClient = std::make_shared<FRTcpClient>(robot_ip, ROBOT_CMD_PORT);

}

/**
 * @brief 机器人状态反馈处理线程
 */
void FRRobot::RobotStateRoutineThread()
{
    int rtn = rtClient->Connect();
    if (rtn != 0)
    {
        logger_error("RobotStateRoutineThread connect fail");
        g_sock_com_err = ERR_SOCKET_COM_FAILED;
        return;
    }

    uint8_t pkgBuf[1024] = {};
    int pkgSize = sizeof(ROBOT_STATE_PKG);
    while (!robot_realstate_exit)
    {
        int rtn = rtClient->RecvPkg((char*)pkgBuf, pkgSize);
        if (rtn != 0)
        {   // -1 网络断开；-2 和校验失败；-3 机器人版本不对应
            g_sock_com_err = ERR_SOCKET_COM_FAILED;
            return;
        }
        else
        {
            memcpy(robot_state_pkg.get(), pkgBuf, sizeof(ROBOT_STATE_PKG));
            memset(pkgBuf, 0, 1024);
        }
    }
    rtClient->Close();
    return;
}

/**
 * @brief 即时指令发送处理线程
 */
void FRRobot::RobotInstCmdSendRoutineThread()
{
    int sendbyte = 0;
    int rtn = cmdClient->Connect();
    if (rtn != 0)
    {
        logger_error("RobotInstCmdSendRoutineThread connect fail");
        g_sock_com_err = ERR_SOCKET_COM_FAILED;
        return;
    }

    while (!robot_instcmd_send_exit)
    {
        if (is_sendcmd && strlen(g_sendbuf) > 0)
        {
            sendbyte = cmdClient->Send(g_sendbuf, strlen(g_sendbuf));
            logger_info("cmd send:%s.", g_sendbuf);
            if (sendbyte < 0)
            {
                cmdClient->Close();
                g_sock_com_err = ERR_SOCKET_COM_FAILED;
                memset(robot_state_pkg.get(), 0, sizeof(ROBOT_STATE_PKG));
                logger_error("cmd send %s error", g_sendbuf);
                return;
            }
            is_sendcmd = false;
        }
        else
        {
            Sleep(1);
        }
    }

    cmdClient->Close();
    g_sock_com_err = ERR_SOCKET_COM_FAILED;
    memset(robot_state_pkg.get(), 0, sizeof(ROBOT_STATE_PKG));
    
    return;
}

/**
 * @brief 即时指令接收处理线程
 */
void FRRobot::RobotInstCmdRecvRoutineThread()
{
    int recvbyte;

    while (!robot_instcmd_recv_exit)
    {
        memset(g_recvbuf, 0, BUFFER_SIZE * sizeof(char));
        recvbyte = cmdClient->Recv(g_recvbuf, BUFFER_SIZE * sizeof(char));
        if (recvbyte > 0)
        {
            logger_info("recv cmd is %s.", (char*)g_recvbuf);
        }
        
        //if (recvbyte < 0)
        //{
        //    cmdClient->Close();
        //    g_sock_com_err = ERR_SOCKET_COM_FAILED;
        //    memset(robot_state_pkg.get(), 0, sizeof(ROBOT_STATE_PKG));
        //    logger_error("cmd recv fail.");
        //    return;
        //}
    }

    cmdClient->Close();
    g_sock_com_err = ERR_SOCKET_COM_FAILED;
    memset(robot_state_pkg.get(), 0, sizeof(ROBOT_STATE_PKG));
   
    return;
}

/**
 * @brief 任务处理线程
 */
void FRRobot::RobotTaskRoutineThread()
{
    uint8_t s_last_frame_cnt;
    uint8_t s_isFirst = 0;
    uint8_t s_check_cnt = 0;
    long long s_last_time = 0;

    logger_info("task routine.");

    while (!robot_task_exit)
    {
        if (g_sock_com_err == ERR_SUCCESS)
        {
            auto now = std::chrono::system_clock::now();                         // 标准时间年-月-日-...-纳秒;
            auto ns = now.time_since_epoch();                                    // 转成纳秒;
            auto ms = std::chrono::duration_cast<std::chrono::microseconds>(ns); // ns -> ms;
            long long curtime = ms.count();                                      //// 提取微秒数量,打时间戳;

            if (!s_isFirst)
            {
                s_last_frame_cnt = robot_state_pkg->frame_cnt;
                s_last_time = curtime;
                s_isFirst = 1;
            }
            else
            {
                if (((robot_state_pkg->frame_cnt - s_last_frame_cnt) == 0) && ((curtime - s_last_time) < 2 * 8000))
                {
                    s_check_cnt++;
                    if (s_check_cnt >= MAX_CHECK_CNT_COM && !rtClient->GetReConnectEnable())
                    {
                        logger_error("the robot RobotTaskRoutineThread is error");
                        g_sock_com_err = ERR_SOCKET_COM_FAILED;
                        memset(robot_state_pkg.get(), 0, sizeof(ROBOT_STATE_PKG));
                        s_check_cnt = 0;
                    }
                }
                else
                {
                    s_check_cnt = 0;
                }

                s_last_frame_cnt = robot_state_pkg->frame_cnt;
                s_last_time = curtime;
            }
        }

        Sleep(8);
    }

    s_isFirst = 0;

    return;
}

/**
 * @brief  与机器人控制器建立通信
 * @param  [in] ip  控制器IP地址，出场默认为192.168.58.2
 * @return 错误码
 */
errno_t FRRobot::RPC(const char *ip)
{
    robot_instcmd_recv_exit = 0;
    robot_instcmd_send_exit = 0;
    robot_realstate_exit = 0;
    robot_task_exit = 0;
    g_sock_com_err = ERR_SUCCESS;

    memset(serverUrl, 0, 64);
    sprintf(serverUrl, "%s", ip);
    logger_info("serverUrl:%s", serverUrl);

    memset(robot_ip, 0, 64);
    strncpy(robot_ip, ip, strlen(ip));

    rtClient->SetIpConfig(robot_ip);
    cmdClient->SetIpConfig(robot_ip);

    thread stateThread(&FRRobot::RobotStateRoutineThread, this);
    stateThread.detach();

    thread cmdsendThread(&FRRobot::RobotInstCmdSendRoutineThread, this);
    cmdsendThread.detach();

    Sleep(2000);
    if (IsSockError())
    {
        logger_info("RPC Fail.");
        return g_sock_com_err;
    }

    thread cmdrecvThread(&FRRobot::RobotInstCmdRecvRoutineThread, this);
    cmdrecvThread.detach();

    thread taskRoutineThread(&FRRobot::RobotTaskRoutineThread, this);
    taskRoutineThread.detach();

    Sleep(1000);
    logger_info("RPC SUCCESS.");
    rpc_done = true;
    return g_sock_com_err;
}

/**
 * @brief  与机器人控制器关闭通讯
 * @return 错误码
 */
errno_t FRRobot::CloseRPC()
{
    robot_instcmd_send_exit = 1;
    robot_instcmd_recv_exit = 1;
    robot_realstate_exit = 1;
    robot_task_exit = 1;

    Sleep(500);

    if (rtClient != nullptr)
    {
        rtClient->Close();
    }

    if (cmdClient != nullptr)
    {
        cmdClient->Close();
    }
    
    g_sock_com_err = ERR_SOCKET_COM_FAILED;
    memset(robot_state_pkg.get(), 0, sizeof(ROBOT_STATE_PKG));

    return 0;
}

/**
 * @brief  查询SDK版本号
 * @param  [out] version   SDK版本号
 * @return  错误码
 */
errno_t FRRobot::GetSDKVersion(char *version)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    strncpy(version, SDK_RELEASE, strlen(SDK_RELEASE));
    return errcode;
}

/**
 * @brief  获取控制器IP
 * @param  [out] ip  控制器IP
 * @return  错误码
 */
errno_t FRRobot::GetControllerIP(char *ip)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetControllerIP", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            std::string s(result[1]);
            strcpy(ip, s.c_str());
        }
        else
        {
            const char *res = "";
            strcpy(ip, res);
            logger_error("GetControllerIP fail. %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  控制机器人进入或退出拖动示教模式
 * @param  [in] state 0-退出拖动示教模式，1-进入拖动示教模式
 * @return  错误码
 */
errno_t FRRobot::DragTeachSwitch(uint8_t state)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = state;

    if (c.execute("DragTeachSwitch", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  查询机器人是否处于拖动示教模式
 * @param  [out] state 0-非拖动示教模式，1-拖动示教模式
 * @return  错误码
 */
errno_t FRRobot::IsInDragTeach(uint8_t *state)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        if (robot_state_pkg->robot_state == 4)
        {
            *state = 1;
        }
        else
        {
            *state = 0;
        }
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d, return value is: %u", errcode, robot_state_pkg->robot_state);
    return errcode;
}

/**
 * @brief  控制机器人上使能或下使能，机器人上电后默认自动上使能
 * @param  [in] state  0-下使能，1-上使能
 * @return  错误码
 */
errno_t FRRobot::RobotEnable(uint8_t state)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = state;

    if (c.execute("RobotEnable", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 控制机器人手自动模式切换
 * @param [in] mode 0-自动模式，1-手动模式
 * @return 错误码
 */
errno_t FRRobot::Mode(int mode)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = mode;

    if (c.execute("Mode", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  jog点动
 * @param  [in]  ref 0-关节点动，2-基坐标系下点动，4-工具坐标系下点动，8-工件坐标系下点动
 * @param  [in]  nb 1-关节1(或x轴)，2-关节2(或y轴)，3-关节3(或z轴)，4-关节4(或绕x轴旋转)，5-关节5(或绕y轴旋转)，6-关节6(或绕z轴旋转)
 * @param  [in]  dir 0-负方向，1-正方向
 * @param  [in]  vel 速度百分比，[0~100]
 * @param  [in]  acc 加速度百分比， [0~100]
 * @param  [in]  max_dis 单次点动最大角度，单位[°]或距离，单位[mm]
 * @return  错误码
 */
errno_t FRRobot::StartJOG(uint8_t ref, uint8_t nb, uint8_t dir, float vel, float acc, float max_dis)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = ref;
    param[1] = nb;
    param[2] = dir;
    param[3] = vel;
    param[4] = acc;
    param[5] = max_dis;

    if (c.execute("StartJOG", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  jog点动减速停止
 * @param  [in]  ref  1-关节点动停止，3-基坐标系下点动停止，5-工具坐标系下点动停止，9-工件坐标系下点动停止
 * @return  错误码
 */
errno_t FRRobot::StopJOG(uint8_t ref)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = ref;

    if (c.execute("StopJOG", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief jog点动立即停止
 * @return  错误码
 */
errno_t FRRobot::ImmStopJOG()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ImmStopJOG", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  关节空间运动
 * @param  [in] joint_pos  目标关节位置,单位deg
 * @param  [in] desc_pos   目标笛卡尔位姿
 * @param  [in] tool  工具坐标号，范围[1~15]
 * @param  [in] user  工件坐标号，范围[1~15]
 * @param  [in] vel  速度百分比，范围[0~100]
 * @param  [in] acc  加速度百分比，范围[0~100],暂不开放
 * @param  [in] ovl  速度缩放因子，范围[0~100]
 * @param  [in] epos  扩展轴位置，单位mm
 * @param  [in] blendT [-1.0]-运动到位(阻塞)，[0~500.0]-平滑时间(非阻塞)，单位ms
 * @param  [in] offset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
 * @param  [in] offset_pos  位姿偏移量
 * @return  错误码
 */
errno_t FRRobot::MoveJ(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos *epos, float blendT, uint8_t offset_flag, DescPose *offset_pos)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0][0] = joint_pos->jPos[0];
    param[0][1] = joint_pos->jPos[1];
    param[0][2] = joint_pos->jPos[2];
    param[0][3] = joint_pos->jPos[3];
    param[0][4] = joint_pos->jPos[4];
    param[0][5] = joint_pos->jPos[5];
    param[1][0] = desc_pos->tran.x;
    param[1][1] = desc_pos->tran.y;
    param[1][2] = desc_pos->tran.z;
    param[1][3] = desc_pos->rpy.rx;
    param[1][4] = desc_pos->rpy.ry;
    param[1][5] = desc_pos->rpy.rz;
    param[2] = tool;
    param[3] = user;
    param[4] = vel;
    param[5] = acc;
    param[6] = ovl;
    param[7][0] = epos->ePos[0];
    param[7][1] = epos->ePos[1];
    param[7][2] = epos->ePos[2];
    param[7][3] = epos->ePos[3];
    param[8] = blendT;
    param[9] = offset_flag;
    param[10][0] = offset_pos->tran.x;
    param[10][1] = offset_pos->tran.y;
    param[10][2] = offset_pos->tran.z;
    param[10][3] = offset_pos->rpy.rx;
    param[10][4] = offset_pos->rpy.ry;
    param[10][5] = offset_pos->rpy.rz;

    if (c.execute("MoveJ", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    if ((robot_state_pkg->main_code != 0 || robot_state_pkg->sub_code != 0) && errcode == 0)
    {
        errcode = 14;
    }

    return errcode;
}

/**
 * @brief  笛卡尔空间直线运动
 * @param  [in] joint_pos  目标关节位置,单位deg
 * @param  [in] desc_pos   目标笛卡尔位姿
 * @param  [in] tool  工具坐标号，范围[1~15]
 * @param  [in] user  工件坐标号，范围[1~15]
 * @param  [in] vel  速度百分比，范围[0~100]
 * @param  [in] acc  加速度百分比，范围[0~100],暂不开放
 * @param  [in] ovl  速度缩放因子，范围[0~100]
 * @param  [in] blendR [-1.0]-运动到位(阻塞)，[0~1000.0]-平滑半径(非阻塞)，单位mm
 * @param  [in] blendMode 过渡方式；0-内切过渡；1-角点过渡
 * @param  [in] epos  扩展轴位置，单位mm
 * @param  [in] search  0-不焊丝寻位，1-焊丝寻位
 * @param  [in] offset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
 * @param  [in] offset_pos  位姿偏移量
 * @param  [in] overSpeedStrategy  超速处理策略，1-标准；2-超速时报错停止；3-自适应降速，默认为0
 * @param  [in] speedPercent  允许降速阈值百分比[0-100]，默认10%
 * @return  错误码
 */
errno_t FRRobot::MoveL(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, int blendMode, ExaxisPos *epos, uint8_t search, uint8_t offset_flag, DescPose *offset_pos, int overSpeedStrategy, int speedPercent)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }

    int errcode = 0;

    XmlRpcClient c(serverUrl, 20003);

    if (overSpeedStrategy > 1)
    {
        XmlRpcValue overSpeedProtectParam, overSpeedProtectResult;
        overSpeedProtectParam[0] = overSpeedStrategy;
        overSpeedProtectParam[1] = speedPercent;
        if (c.execute("JointOverSpeedProtectStart", overSpeedProtectParam, overSpeedProtectResult))
        {
            errcode = int(overSpeedProtectResult);
        }
        else
        {
            c.close();
            return ERR_XMLRPC_CMD_FAILED;
        }
       
        if (errcode != 0)
        {
            c.close();
            return errcode;
        }
    }
    
    XmlRpcValue param, result;

    param[0][0] = joint_pos->jPos[0];
    param[0][1] = joint_pos->jPos[1];
    param[0][2] = joint_pos->jPos[2];
    param[0][3] = joint_pos->jPos[3];
    param[0][4] = joint_pos->jPos[4];
    param[0][5] = joint_pos->jPos[5];
    param[1][0] = desc_pos->tran.x;
    param[1][1] = desc_pos->tran.y;
    param[1][2] = desc_pos->tran.z;
    param[1][3] = desc_pos->rpy.rx;
    param[1][4] = desc_pos->rpy.ry;
    param[1][5] = desc_pos->rpy.rz;
    param[2] = tool;
    param[3] = user;
    param[4] = vel;
    param[5] = acc;
    param[6] = ovl;
    param[7] = blendR;
    param[8] = blendMode;
    param[9][0] = epos->ePos[0];
    param[9][1] = epos->ePos[1];
    param[9][2] = epos->ePos[2];
    param[9][3] = epos->ePos[3];
    param[10] = search;
    param[11] = offset_flag;
    param[12][0] = offset_pos->tran.x;
    param[12][1] = offset_pos->tran.y;
    param[12][2] = offset_pos->tran.z;
    param[12][3] = offset_pos->rpy.rx;
    param[12][4] = offset_pos->rpy.ry;
    param[12][5] = offset_pos->rpy.rz;

    if (c.execute("MoveL", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    if (overSpeedStrategy > 1)
    {
        XmlRpcValue overSpeedProtectParam, overSpeedProtectResult;
        if (c.execute("JointOverSpeedProtectEnd", overSpeedProtectParam, overSpeedProtectResult))
        {
            errcode = int(overSpeedProtectResult);
        }
        else
        {
            c.close();
            return ERR_XMLRPC_CMD_FAILED;
        }

        if (errcode != 0)
        {
            return errcode;
        }
    }

    c.close();

    if ((robot_state_pkg->main_code != 0 || robot_state_pkg->sub_code != 0) && errcode == 0)
    {
        errcode = 14;
    }

    return errcode;
}

/**
 * @brief  笛卡尔空间直线运动
 * @param  [in] joint_pos  目标关节位置,单位deg
 * @param  [in] desc_pos   目标笛卡尔位姿
 * @param  [in] tool  工具坐标号，范围[1~15]
 * @param  [in] user  工件坐标号，范围[1~15]
 * @param  [in] vel  速度百分比，范围[0~100]
 * @param  [in] acc  加速度百分比，范围[0~100],暂不开放
 * @param  [in] ovl  速度缩放因子，范围[0~100]
 * @param  [in] blendR [-1.0]-运动到位(阻塞)，[0~1000.0]-平滑半径(非阻塞)，单位mm
 * @param  [in] epos  扩展轴位置，单位mm
 * @param  [in] search  0-不焊丝寻位，1-焊丝寻位
 * @param  [in] offset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
 * @param  [in] offset_pos  位姿偏移量
 * @param  [in] overSpeedStrategy  超速处理策略，1-标准；2-超速时报错停止；3-自适应降速，默认为0
 * @param  [in] speedPercent  允许降速阈值百分比[0-100]，默认10%
 * @return  错误码
 */
errno_t FRRobot::MoveL(JointPos* joint_pos, DescPose* desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos* epos, uint8_t search, uint8_t offset_flag, DescPose* offset_pos, int overSpeedStrategy, int speedPercent)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }

    int errcode = 0;

    XmlRpcClient c(serverUrl, 20003);

    if (overSpeedStrategy > 1)
    {
        XmlRpcValue overSpeedProtectParam, overSpeedProtectResult;
        overSpeedProtectParam[0] = overSpeedStrategy;
        overSpeedProtectParam[1] = speedPercent;
        if (c.execute("JointOverSpeedProtectStart", overSpeedProtectParam, overSpeedProtectResult))
        {
            errcode = int(overSpeedProtectResult);
        }
        else
        {
            c.close();
            return ERR_XMLRPC_CMD_FAILED;
        }

        if (errcode != 0)
        {
            c.close();
            return errcode;
        }
    }

    XmlRpcValue param, result;

    param[0][0] = joint_pos->jPos[0];
    param[0][1] = joint_pos->jPos[1];
    param[0][2] = joint_pos->jPos[2];
    param[0][3] = joint_pos->jPos[3];
    param[0][4] = joint_pos->jPos[4];
    param[0][5] = joint_pos->jPos[5];
    param[1][0] = desc_pos->tran.x;
    param[1][1] = desc_pos->tran.y;
    param[1][2] = desc_pos->tran.z;
    param[1][3] = desc_pos->rpy.rx;
    param[1][4] = desc_pos->rpy.ry;
    param[1][5] = desc_pos->rpy.rz;
    param[2] = tool;
    param[3] = user;
    param[4] = vel;
    param[5] = acc;
    param[6] = ovl;
    param[7] = blendR;
    param[8] = 0;
    param[9][0] = epos->ePos[0];
    param[9][1] = epos->ePos[1];
    param[9][2] = epos->ePos[2];
    param[9][3] = epos->ePos[3];
    param[10] = search;
    param[11] = offset_flag;
    param[12][0] = offset_pos->tran.x;
    param[12][1] = offset_pos->tran.y;
    param[12][2] = offset_pos->tran.z;
    param[12][3] = offset_pos->rpy.rx;
    param[12][4] = offset_pos->rpy.ry;
    param[12][5] = offset_pos->rpy.rz;

    if (c.execute("MoveL", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    if (overSpeedStrategy > 1)
    {
        XmlRpcValue overSpeedProtectParam, overSpeedProtectResult;
        if (c.execute("JointOverSpeedProtectEnd", overSpeedProtectParam, overSpeedProtectResult))
        {
            errcode = int(overSpeedProtectResult);
        }
        else
        {
            c.close();
            return ERR_XMLRPC_CMD_FAILED;
        }

        if (errcode != 0)
        {
            return errcode;
        }
    }

    c.close();

    if ((robot_state_pkg->main_code != 0 || robot_state_pkg->sub_code != 0) && errcode == 0)
    {
        errcode = 14;
    }

    return errcode;
}

/**
 * @brief  笛卡尔空间圆弧运动
 * @param  [in] joint_pos_p  路径点关节位置,单位deg
 * @param  [in] desc_pos_p   路径点笛卡尔位姿
 * @param  [in] ptool  工具坐标号，范围[1~15]
 * @param  [in] puser  工件坐标号，范围[1~15]
 * @param  [in] pvel  速度百分比，范围[0~100]
 * @param  [in] pacc  加速度百分比，范围[0~100],暂不开放
 * @param  [in] epos_p  扩展轴位置，单位mm
 * @param  [in] poffset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
 * @param  [in] offset_pos_p  位姿偏移量
 * @param  [in] joint_pos_t  目标点关节位置,单位deg
 * @param  [in] desc_pos_t   目标点笛卡尔位姿
 * @param  [in] ttool  工具坐标号，范围[1~15]
 * @param  [in] tuser  工件坐标号，范围[1~15]
 * @param  [in] tvel  速度百分比，范围[0~100]
 * @param  [in] tacc  加速度百分比，范围[0~100],暂不开放
 * @param  [in] epos_t  扩展轴位置，单位mm
 * @param  [in] toffset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
 * @param  [in] offset_pos_t  位姿偏移量
 * @param  [in] ovl  速度缩放因子，范围[0~100]
 * @param  [in] blendR [-1.0]-运动到位(阻塞)，[0~1000.0]-平滑半径(非阻塞)，单位mm
 * @return  错误码
 */
errno_t FRRobot::MoveC(JointPos *joint_pos_p, DescPose *desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos *epos_p, uint8_t poffset_flag, DescPose *offset_pos_p, JointPos *joint_pos_t, DescPose *desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos *epos_t, uint8_t toffset_flag, DescPose *offset_pos_t, float ovl, float blendR)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0][0] = joint_pos_p->jPos[0];
    param[0][1] = joint_pos_p->jPos[1];
    param[0][2] = joint_pos_p->jPos[2];
    param[0][3] = joint_pos_p->jPos[3];
    param[0][4] = joint_pos_p->jPos[4];
    param[0][5] = joint_pos_p->jPos[5];
    param[1][0] = desc_pos_p->tran.x;
    param[1][1] = desc_pos_p->tran.y;
    param[1][2] = desc_pos_p->tran.z;
    param[1][3] = desc_pos_p->rpy.rx;
    param[1][4] = desc_pos_p->rpy.ry;
    param[1][5] = desc_pos_p->rpy.rz;
    param[2][0] = (double)ptool;
    param[2][1] = (double)puser;
    param[2][2] = pvel;
    param[2][3] = pacc;
    param[3][0] = epos_p->ePos[0];
    param[3][1] = epos_p->ePos[1];
    param[3][2] = epos_p->ePos[2];
    param[3][3] = epos_p->ePos[3];
    param[4] = poffset_flag;
    param[5][0] = offset_pos_p->tran.x;
    param[5][1] = offset_pos_p->tran.y;
    param[5][2] = offset_pos_p->tran.z;
    param[5][3] = offset_pos_p->rpy.rx;
    param[5][4] = offset_pos_p->rpy.ry;
    param[5][5] = offset_pos_p->rpy.rz;
    param[6][0] = joint_pos_t->jPos[0];
    param[6][1] = joint_pos_t->jPos[1];
    param[6][2] = joint_pos_t->jPos[2];
    param[6][3] = joint_pos_t->jPos[3];
    param[6][4] = joint_pos_t->jPos[4];
    param[6][5] = joint_pos_t->jPos[5];
    param[7][0] = desc_pos_t->tran.x;
    param[7][1] = desc_pos_t->tran.y;
    param[7][2] = desc_pos_t->tran.z;
    param[7][3] = desc_pos_t->rpy.rx;
    param[7][4] = desc_pos_t->rpy.ry;
    param[7][5] = desc_pos_t->rpy.rz;
    param[8][0] = (double)ttool;
    param[8][1] = (double)tuser;
    param[8][2] = tvel;
    param[8][3] = tacc;
    param[9][0] = epos_t->ePos[0];
    param[9][1] = epos_t->ePos[1];
    param[9][2] = epos_t->ePos[2];
    param[9][3] = epos_t->ePos[3];
    param[10] = toffset_flag;
    param[11][0] = offset_pos_t->tran.x;
    param[11][1] = offset_pos_t->tran.y;
    param[11][2] = offset_pos_t->tran.z;
    param[11][3] = offset_pos_t->rpy.rx;
    param[11][4] = offset_pos_t->rpy.ry;
    param[11][5] = offset_pos_t->rpy.rz;
    param[12] = ovl;
    param[13] = blendR;

    logger_info("movec 1\n");

    if (c.execute("MoveC", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    logger_info("errcode:%d.", errcode);

    c.close();

    return errcode;
}

/**
 *@brief  笛卡尔空间整圆运动
 *@param  [in] joint_pos_p  路径点1关节位置,单位deg
 *@param  [in] desc_pos_p   路径点1笛卡尔位姿
 *@param  [in] ptool  工具坐标号，范围[1~15]
 *@param  [in] puser  工件坐标号，范围[1~15]
 *@param  [in] pvel  速度百分比，范围[0~100]
 *@param  [in] pacc  加速度百分比，范围[0~100],暂不开放
 *@param  [in] epos_p  扩展轴位置，单位mm
 *@param  [in] joint_pos_t  路径点2关节位置,单位deg
 *@param  [in] desc_pos_t   路径点2笛卡尔位姿
 *@param  [in] ttool  工具坐标号，范围[1~15]
 *@param  [in] tuser  工件坐标号，范围[1~15]
 *@param  [in] tvel  速度百分比，范围[0~100]
 *@param  [in] tacc  加速度百分比，范围[0~100],暂不开放
 *@param  [in] epos_t  扩展轴位置，单位mm
 *@param  [in] ovl  速度缩放因子，范围[0~100]
 *@param  [in] offset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
 *@param  [in] offset_pos  位姿偏移量
 *@param  [in] oacc 加速度百分比
 *@param  [in] blendR -1：阻塞；0~1000：平滑半径
 *@return  错误码
 */
errno_t FRRobot::Circle(JointPos *joint_pos_p, DescPose *desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos *epos_p, JointPos *joint_pos_t, DescPose *desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos *epos_t, float ovl, uint8_t offset_flag, DescPose *offset_pos, double oacc, double blendR)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0][0] = joint_pos_p->jPos[0];
    param[0][1] = joint_pos_p->jPos[1];
    param[0][2] = joint_pos_p->jPos[2];
    param[0][3] = joint_pos_p->jPos[3];
    param[0][4] = joint_pos_p->jPos[4];
    param[0][5] = joint_pos_p->jPos[5];
    param[1][0] = desc_pos_p->tran.x;
    param[1][1] = desc_pos_p->tran.y;
    param[1][2] = desc_pos_p->tran.z;
    param[1][3] = desc_pos_p->rpy.rx;
    param[1][4] = desc_pos_p->rpy.ry;
    param[1][5] = desc_pos_p->rpy.rz;
    param[2][0] = (double)ptool;
    param[2][1] = (double)puser;
    param[2][2] = pvel;
    param[2][3] = pacc;
    param[3][0] = epos_p->ePos[0];
    param[3][1] = epos_p->ePos[1];
    param[3][2] = epos_p->ePos[2];
    param[3][3] = epos_p->ePos[3];
    param[4][0] = joint_pos_t->jPos[0];
    param[4][1] = joint_pos_t->jPos[1];
    param[4][2] = joint_pos_t->jPos[2];
    param[4][3] = joint_pos_t->jPos[3];
    param[4][4] = joint_pos_t->jPos[4];
    param[4][5] = joint_pos_t->jPos[5];
    param[5][0] = desc_pos_t->tran.x;
    param[5][1] = desc_pos_t->tran.y;
    param[5][2] = desc_pos_t->tran.z;
    param[5][3] = desc_pos_t->rpy.rx;
    param[5][4] = desc_pos_t->rpy.ry;
    param[5][5] = desc_pos_t->rpy.rz;
    param[6][0] = (double)ttool;
    param[6][1] = (double)tuser;
    param[6][2] = tvel;
    param[6][3] = tacc;
    param[7][0] = epos_t->ePos[0];
    param[7][1] = epos_t->ePos[1];
    param[7][2] = epos_t->ePos[2];
    param[7][3] = epos_t->ePos[3];
    param[8][0] = ovl;
    param[8][1] = (double)offset_flag;
    param[9][0] = offset_pos->tran.x;
    param[9][1] = offset_pos->tran.y;
    param[9][2] = offset_pos->tran.z;
    param[9][3] = offset_pos->rpy.rx;
    param[9][4] = offset_pos->rpy.ry;
    param[9][5] = offset_pos->rpy.rz;
    param[10][0] = oacc;
    param[10][1] = blendR;

    if (c.execute("Circle", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 *@brief  笛卡尔空间螺旋线运动
 *@param  [in] joint_pos  目标关节位置,单位deg
 *@param  [in] desc_pos   目标笛卡尔位姿
 *@param  [in] tool  工具坐标号，范围[1~15]
 *@param  [in] user  工件坐标号，范围[1~15]
 *@param  [in] vel  速度百分比，范围[0~100]
 *@param  [in] acc  加速度百分比，范围[0~100],暂不开放
 *@param  [in] epos  扩展轴位置，单位mm
 *@param  [in] ovl  速度缩放因子，范围[0~100]
 *@param  [in] offset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
 *@param  [in] offset_pos  位姿偏移量
 *@param  [in] spiral_param  螺旋参数
 *@return  错误码
 */
errno_t FRRobot::NewSpiral(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, ExaxisPos *epos, float ovl, uint8_t offset_flag, DescPose *offset_pos, SpiralParam spiral_param)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0][0] = joint_pos->jPos[0];
    param[0][1] = joint_pos->jPos[1];
    param[0][2] = joint_pos->jPos[2];
    param[0][3] = joint_pos->jPos[3];
    param[0][4] = joint_pos->jPos[4];
    param[0][5] = joint_pos->jPos[5];
    param[1][0] = desc_pos->tran.x;
    param[1][1] = desc_pos->tran.y;
    param[1][2] = desc_pos->tran.z;
    param[1][3] = desc_pos->rpy.rx;
    param[1][4] = desc_pos->rpy.ry;
    param[1][5] = desc_pos->rpy.rz;
    param[2] = tool;
    param[3] = user;
    param[4] = vel;
    param[5] = acc;
    param[6][0] = epos->ePos[0];
    param[6][1] = epos->ePos[1];
    param[6][2] = epos->ePos[2];
    param[6][3] = epos->ePos[3];
    param[7] = ovl;
    param[8] = offset_flag;
    param[9][0] = offset_pos->tran.x;
    param[9][1] = offset_pos->tran.y;
    param[9][2] = offset_pos->tran.z;
    param[9][3] = offset_pos->rpy.rx;
    param[9][4] = offset_pos->rpy.ry;
    param[9][5] = offset_pos->rpy.rz;
    param[10][0] = (double)spiral_param.circle_num;
    param[10][1] = spiral_param.circle_angle;
    param[10][2] = spiral_param.rad_init;
    param[10][3] = spiral_param.rad_add;
    param[10][4] = spiral_param.rotaxis_add;
    param[10][5] = (double)spiral_param.rot_direction;

    if (c.execute("NewSpiral", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 伺服运动开始，配合ServoJ、ServoCart指令使用
 * @return  错误码
 */
errno_t FRRobot::ServoMoveStart()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ServoMoveStart", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 伺服运动结束，配合ServoJ、ServoCart指令使用
 * @return  错误码
 */
errno_t FRRobot::ServoMoveEnd()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ServoMoveEnd", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 *@brief  关节空间伺服模式运动
 *@param  [in] joint_pos  目标关节位置,单位deg
 *@param  [in] axisPos  外部轴位置,单位mm
 *@param  [in] acc  加速度百分比，范围[0~100],暂不开放，默认为0
 *@param  [in] vel  速度百分比，范围[0~100]，暂不开放，默认为0
 *@param  [in] cmdT  指令下发周期，单位s，建议范围[0.001~0.0016]
 *@param  [in] filterT 滤波时间，单位s，暂不开放，默认为0
 *@param  [in] gain  目标位置的比例放大器，暂不开放，默认为0
 *@param  [in] id servoJ指令ID,默认为0
 *@return  错误码
 */
errno_t FRRobot::ServoJ(JointPos *joint_pos, ExaxisPos* axisPos, float acc, float vel, float cmdT, float filterT, float gain, int id)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0][0] = joint_pos->jPos[0];
    param[0][1] = joint_pos->jPos[1];
    param[0][2] = joint_pos->jPos[2];
    param[0][3] = joint_pos->jPos[3];
    param[0][4] = joint_pos->jPos[4];
    param[0][5] = joint_pos->jPos[5];
    param[1][0] = axisPos->ePos[0];
    param[1][1] = axisPos->ePos[1];
    param[1][2] = axisPos->ePos[2];
    param[1][3] = axisPos->ePos[3];
    param[2] = acc;
    param[3] = vel;
    param[4] = cmdT;
    param[5] = filterT;
    param[6] = gain;
    param[7] = id;

    if (c.execute("ServoJ", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 *@brief  笛卡尔空间伺服模式运动
 *@param  [in]  mode  0-绝对运动(基坐标系)，1-增量运动(基坐标系)，2-增量运动(工具坐标系)
 *@param  [in]  desc_pos  目标笛卡尔位姿或位姿增量
 *@param  [in]  pos_gain  位姿增量比例系数，仅在增量运动下生效，范围[0~1]
 *@param  [in] acc  加速度百分比，范围[0~100],暂不开放，默认为0
 *@param  [in] vel  速度百分比，范围[0~100]，暂不开放，默认为0
 *@param  [in] cmdT  指令下发周期，单位s，建议范围[0.001~0.0016]
 *@param  [in] filterT 滤波时间，单位s，暂不开放，默认为0
 *@param  [in] gain  目标位置的比例放大器，暂不开放，默认为0
 *@return  错误码
 */
errno_t FRRobot::ServoCart(int mode, DescPose *desc_pose, float pos_gain[6], float acc, float vel, float cmdT, float filterT, float gain)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = mode;
    param[1][0] = desc_pose->tran.x;
    param[1][1] = desc_pose->tran.y;
    param[1][2] = desc_pose->tran.z;
    param[1][3] = desc_pose->rpy.rx;
    param[1][4] = desc_pose->rpy.ry;
    param[1][5] = desc_pose->rpy.rz;
    param[2][0] = pos_gain[0];
    param[2][1] = pos_gain[1];
    param[2][2] = pos_gain[2];
    param[2][3] = pos_gain[3];
    param[2][4] = pos_gain[4];
    param[2][5] = pos_gain[5];
    param[3] = acc;
    param[4] = vel;
    param[5] = cmdT;
    param[6] = filterT;
    param[7] = gain;

    if (c.execute("ServoCart", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 *@brief  笛卡尔空间点到点运动
 *@param  [in]  desc_pos  目标笛卡尔位姿或位姿增量
 *@param  [in] tool  工具坐标号，范围[1~15]
 *@param  [in] user  工件坐标号，范围[1~15]
 *@param  [in] vel  速度百分比，范围[0~100]
 *@param  [in] acc  加速度百分比，范围[0~100],暂不开放
 *@param  [in] ovl  速度缩放因子，范围[0~100]
 *@param  [in] blendT [-1.0]-运动到位(阻塞)，[0~500.0]-平滑时间(非阻塞)，单位ms
 *@param  [in] config  关节空间配置，[-1]-参考当前关节位置解算，[0~7]-参考特定关节空间配置解算，默认为-1
 *@return  错误码
 */
errno_t FRRobot::MoveCart(DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendT, int config)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0][0] = desc_pos->tran.x;
    param[0][1] = desc_pos->tran.y;
    param[0][2] = desc_pos->tran.z;
    param[0][3] = desc_pos->rpy.rx;
    param[0][4] = desc_pos->rpy.ry;
    param[0][5] = desc_pos->rpy.rz;
    param[1] = tool;
    param[2] = user;
    param[3] = vel;
    param[4] = acc;
    param[5] = ovl;
    param[6] = blendT;
    param[7] = config;

    if (c.execute("MoveCart", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  样条运动开始
 * @return  错误码
 */
errno_t FRRobot::SplineStart()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("SplineStart", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 *@brief  关节空间样条运动
 *@param  [in] joint_pos  目标关节位置,单位deg
 *@param  [in] desc_pos   目标笛卡尔位姿
 *@param  [in] tool  工具坐标号，范围[1~15]
 *@param  [in] user  工件坐标号，范围[1~15]
 *@param  [in] vel  速度百分比，范围[0~100]
 *@param  [in] acc  加速度百分比，范围[0~100],暂不开放
 *@param  [in] ovl  速度缩放因子，范围[0~100]
 *@return  错误码
 */
errno_t FRRobot::SplinePTP(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0][0] = joint_pos->jPos[0];
    param[0][1] = joint_pos->jPos[1];
    param[0][2] = joint_pos->jPos[2];
    param[0][3] = joint_pos->jPos[3];
    param[0][4] = joint_pos->jPos[4];
    param[0][5] = joint_pos->jPos[5];
    param[1][0] = desc_pos->tran.x;
    param[1][1] = desc_pos->tran.y;
    param[1][2] = desc_pos->tran.z;
    param[1][3] = desc_pos->rpy.rx;
    param[1][4] = desc_pos->rpy.ry;
    param[1][5] = desc_pos->rpy.rz;
    param[2] = tool;
    param[3] = user;
    param[4] = vel;
    param[5] = acc;
    param[6] = ovl;

    if (c.execute("SplinePTP", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  样条运动结束
 * @return  错误码
 */
errno_t FRRobot::SplineEnd()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("SplineEnd", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 新样条运动开始
 * @param  [in] type   0-圆弧过渡，1-给定点位为路径点
 * @return  错误码
 */
errno_t FRRobot::NewSplineStart(int type,  int averageTime)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = type;
    param[1] = averageTime;

    if (c.execute("NewSplineStart", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 新样条指令点
 * @param  [in] joint_pos  目标关节位置,单位deg
 * @param  [in] desc_pos   目标笛卡尔位姿
 * @param  [in] tool  工具坐标号，范围[1~15]
 * @param  [in] user  工件坐标号，范围[1~15]
 * @param  [in] vel  速度百分比，范围[0~100]
 * @param  [in] acc  加速度百分比，范围[0~100],暂不开放
 * @param  [in] ovl  速度缩放因子，范围[0~100]
 * @param  [in] blendR [-1.0]-运动到位(阻塞)，[0~1000.0]-平滑半径(非阻塞)，单位mm
 * @param  [in] lastFlag 是否为最后一个点，0-否，1-是
 * @return  错误码
 */
errno_t FRRobot::NewSplinePoint(JointPos *joint_pos, DescPose *desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, int lastFlag)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0][0] = joint_pos->jPos[0];
    param[0][1] = joint_pos->jPos[1];
    param[0][2] = joint_pos->jPos[2];
    param[0][3] = joint_pos->jPos[3];
    param[0][4] = joint_pos->jPos[4];
    param[0][5] = joint_pos->jPos[5];
    param[1][0] = desc_pos->tran.x;
    param[1][1] = desc_pos->tran.y;
    param[1][2] = desc_pos->tran.z;
    param[1][3] = desc_pos->rpy.rx;
    param[1][4] = desc_pos->rpy.ry;
    param[1][5] = desc_pos->rpy.rz;
    param[2] = tool;
    param[3] = user;
    param[4] = vel;
    param[5] = acc;
    param[6] = ovl;
    param[7] = blendR;
    param[8] = lastFlag;

    if (c.execute("NewSplinePoint", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 新样条运动结束
 * @return  错误码
 */
errno_t FRRobot::NewSplineEnd()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("NewSplineEnd", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 终止运动
 * @return  错误码
 */
errno_t FRRobot::StopMotion()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    static int cnt = 0;

    memset(g_sendbuf, 0, BUFFER_SIZE * sizeof(char));
    sprintf(g_sendbuf, "/f/bIII44III102III4IIISTOPIII/b/f", cnt);
    cnt++;
    is_sendcmd = true;

    logger_info("StopMotion.");
    return errcode;
}

/**
 * @brief 暂停运动
 * @return  错误码
 */
errno_t FRRobot::PauseMotion()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    static int cnt = 0;

    memset(g_sendbuf, 0, BUFFER_SIZE * sizeof(char));
    sprintf(g_sendbuf, "/f/bIII%dIII103III5IIIPAUSEIII/b/f", cnt);
    cnt++;
    is_sendcmd = true;

    logger_info("PauseMotion.");
    return errcode;
}

/**
 * @brief 恢复运动
 * @return  错误码
 */
errno_t FRRobot::ResumeMotion()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }

    int errcode = 0;
    static int cnt = 0;

    memset(g_sendbuf, 0, BUFFER_SIZE * sizeof(char));
    sprintf(g_sendbuf, "/f/bIII%dIII104III6IIIRESUMEIII/b/f", cnt);
    cnt++;
    is_sendcmd = true;

    logger_info("ResumeMotion.");
    return errcode;
}

/**
 * @brief  点位整体偏移开始
 * @param  [in]  flag  0-基坐标系下/工件坐标系下偏移，2-工具坐标系下偏移
 * @param  [in] offset_pos  位姿偏移量
 * @return  错误码
 */
errno_t FRRobot::PointsOffsetEnable(int flag, DescPose *offset_pos)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = flag;
    param[1][0] = offset_pos->tran.x;
    param[1][1] = offset_pos->tran.y;
    param[1][2] = offset_pos->tran.z;
    param[1][3] = offset_pos->rpy.rx;
    param[1][4] = offset_pos->rpy.ry;
    param[1][5] = offset_pos->rpy.rz;

    if (c.execute("PointsOffsetEnable", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  点位整体偏移结束
 * @return  错误码
 */
errno_t FRRobot::PointsOffsetDisable()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("PointsOffsetDisable", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置控制箱数字量输出
 * @param  [in] id  io编号，范围[0~15]
 * @param  [in] status 0-关，1-开
 * @param  [in] smooth 0-不平滑， 1-平滑
 * @param  [in] block  0-阻塞，1-非阻塞
 * @return  错误码
 */
errno_t FRRobot::SetDO(int id, uint8_t status, uint8_t smooth, uint8_t block)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    param[1] = status;
    param[2] = smooth;
    param[3] = block;

    if (c.execute("SetDO", param, result))
    {
        errcode = int(result);
        if(0 != errcode)
        {
            logger_error("fail, errorcode: %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置工具数字量输出
 * @param  [in] id  io编号，范围[0~1]
 * @param  [in] status 0-关，1-开
 * @param  [in] smooth 0-不平滑， 1-平滑
 * @param  [in] block  0-阻塞，1-非阻塞
 * @return  错误码
 */
errno_t FRRobot::SetToolDO(int id, uint8_t status, uint8_t smooth, uint8_t block)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    param[1] = status;
    param[2] = smooth;
    param[3] = block;

    if (c.execute("SetToolDO", param, result))
    {
        errcode = int(result);
        if(0 != errcode)
        {
            logger_error("fail, errorcode: %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置控制箱模拟量输出
 * @param  [in] id  io编号，范围[0~1]
 * @param  [in] value 电流或电压值百分比，范围[0~100]对应电流值[0~20mA]或电压[0~10V]
 * @param  [in] block  0-阻塞，1-非阻塞
 * @return  错误码
 */
errno_t FRRobot::SetAO(int id, float value, uint8_t block)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    param[1] = value;
    param[2] = block;

    if (c.execute("SetAO", param, result))
    {
        errcode = int(result);
        if(0 != errcode)
        {
            logger_error("fail, errorcode: %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置工具模拟量输出
 * @param  [in] id  io编号，范围[0]
 * @param  [in] value 电流或电压值百分比，范围[0~100]对应电流值[0~20mA]或电压[0~10V]
 * @param  [in] block  0-阻塞，1-非阻塞
 * @return  错误码
 */
errno_t FRRobot::SetToolAO(int id, float value, uint8_t block)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    param[1] = value;
    param[2] = block;

    if (c.execute("SetToolAO", param, result))
    {
        errcode = int(result);
        if(0 != errcode)
        {
            logger_error("fail, errorcode: %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  获取控制箱数字量输入
 * @param  [in] id  io编号，范围[0~15]
 * @param  [in] block  0-阻塞，1-非阻塞
 * @param  [out] result  0-低电平，1-高电平
 * @return  错误码
 */
errno_t FRRobot::GetDI(int id, uint8_t block, uint8_t *result)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        if (id >= 0 && id < 8)
        {
            *result = (robot_state_pkg->cl_dgt_input_l & (0x01 << id)) >> id;
        }
        else if (id >= 8 && id < 16)
        {
            id -= 8;
            *result = (robot_state_pkg->cl_dgt_input_h & (0x01 << id)) >> id;
        }
        else
        {
            *result = 0;
            errcode = -1;
        }
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d", errcode);
    logger_info("return value is: %u", *result);
    return errcode;
}

/**
 * @brief  获取工具数字量输入
 * @param  [in] id  io编号，范围[0~1]
 * @param  [in] block  0-阻塞，1-非阻塞
 * @param  [out] result  0-低电平，1-高电平
 * @return  错误码
 */
errno_t FRRobot::GetToolDI(int id, uint8_t block, uint8_t *result)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        if (id >= 0 && id < 2)
        {
            id += 1;
            *result = (robot_state_pkg->tl_dgt_input_l & (0x01 << id)) >> id;
        }
        else
        {
            *result = 0;
            errcode = -1;
        }
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d", errcode);
    logger_info("return value is: %u", *result);
    return errcode;
}

/**
 * @brief 等待控制箱数字量输入
 * @param  [in]  id  io编号，范围[0~15]
 * @param  [in]  status 0-关，1-开
 * @param  [in]  max_time  最大等待时间，单位ms
 * @param  [in]  opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
 * @return  错误码
 */
errno_t FRRobot::WaitDI(int id, uint8_t status, int max_time, int opt)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    param[1] = status;
    param[2] = max_time;
    param[3] = opt;

    if (c.execute("WaitDI", param, result))
    {
        errcode = int(result);
        if(0 != errcode)
        {
            logger_error("fail, errorcode: %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 等待控制箱多路数字量输入
 * @param  [in] mode 0-多路与，1-多路或
 * @param  [in] id  io编号，bit0~bit7对应DI0~DI7，bit8~bit15对应CI0~CI7
 * @param  [in] status 0-关，1-开
 * @param  [in] max_time  最大等待时间，单位ms
 * @param  [in] opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
 * @return  错误码
 */
errno_t FRRobot::WaitMultiDI(int mode, int id, uint8_t status, int max_time, int opt)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = mode;
    param[1] = id;
    param[2] = status;
    param[3] = max_time;
    param[4] = opt;

    if (c.execute("WaitMultiDI", param, result))
    {
        errcode = int(result);
        if(0 != errcode)
        {
            logger_error("fail, errorcode: %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 等待工具数字量输入
 * @param  [in] id  io编号，范围[0~1]
 * @param  [in] status 0-关，1-开
 * @param  [in] max_time  最大等待时间，单位ms
 * @param  [in] opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
 * @return  错误码
 */
errno_t FRRobot::WaitToolDI(int id, uint8_t status, int max_time, int opt)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id + 1;
    param[1] = status;
    param[2] = max_time;
    param[3] = opt;

    if (c.execute("WaitToolDI", param, result))
    {
        errcode = int(result);
        if(0 != errcode)
        {
            logger_error("fail, errorcode: %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  获取控制箱模拟量输入
 * @param  [in] id  io编号，范围[0~1]
 * @param  [in] block  0-阻塞，1-非阻塞
 * @param  [out] result  输入电流或电压值百分比，范围[0~100]对应电流值[0~20mS]或电压[0~10V]
 * @return  错误码
 */
errno_t FRRobot::GetAI(int id, uint8_t block, float *result)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        if (id >= 0 && id < 2)
        {
            *result = (float)(robot_state_pkg->cl_analog_input[id] / 40.95);
        }
        else
        {
            *result = 0;
            errcode = -1;
        }
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d", errcode);
    for (int i = 0; i < 2; i++)
    {
        logger_info("return value %d is: %u", i, robot_state_pkg->cl_analog_input[i]);
    }
    return errcode;
}

/**
 * @brief  获取工具模拟量输入
 * @param  [in] id  io编号，范围[0]
 * @param  [in] block  0-阻塞，1-非阻塞
 * @param  [out] result  输入电流或电压值百分比，范围[0~100]对应电流值[0~20mS]或电压[0~10V]
 * @return  错误码
 */
errno_t FRRobot::GetToolAI(int id, uint8_t block, float *result)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *result = (float)(robot_state_pkg->tl_anglog_input / 40.95);
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d", errcode);
    logger_info("return value is: %u", robot_state_pkg->tl_anglog_input);
    return errcode;
}
/**
 * @brief 获取机器人末端点记录按钮状态
 * @param [out] state 按钮状态，0-按下，1-松开
 * @return 错误码
 */
errno_t FRRobot::GetAxlePointRecordBtnState(uint8_t *state)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *state = (robot_state_pkg->tl_dgt_input_l & 0x10) >> 4;
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d", errcode);
    logger_info("return value is: %u", *state );
    return errcode;
}
/**
 * @brief 获取机器人末端DO输出状态
 * @param [out] do_state DO输出状态，do0~do1对应bit1~bit2,从bit0开始
 * @return 错误码
 */
errno_t FRRobot::GetToolDO(uint8_t *do_state)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *do_state = robot_state_pkg->tl_dgt_output_l;
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d", errcode);
    logger_info("return value is: %u", robot_state_pkg->tl_dgt_output_l);
    return errcode;
}

/**
 * @brief 获取机器人控制器DO输出状态
 * @param [out] do_state_h DO输出状态，co0~co7对应bit0~bit7
 * @param [out] do_state_l DO输出状态，do0~do7对应bit0~bit7
 * @return 错误码
 */
errno_t FRRobot::GetDO(uint8_t *do_state_h, uint8_t *do_state_l)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *do_state_h = robot_state_pkg->cl_dgt_output_h;
        *do_state_l = robot_state_pkg->cl_dgt_output_l;
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d", errcode);
    logger_info("return value is: %u - %u", robot_state_pkg->cl_dgt_output_h, robot_state_pkg->cl_dgt_output_l);
    return errcode;
}

/**
 * @brief 等待控制箱模拟量输入
 * @param  [in]  id  io编号，范围[0~1]
 * @param  [in]  sign 0-大于，1-小于
 * @param  [in]  value 输入电流或电压值百分比，范围[0~100]对应电流值[0~20mS]或电压[0~10V]
 * @param  [in]  max_time  最大等待时间，单位ms
 * @param  [in]  opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
 * @return  错误码
 */
errno_t FRRobot::WaitAI(int id, int sign, float value, int max_time, int opt)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    param[1] = sign;
    param[2] = value;
    param[3] = max_time;
    param[4] = opt;

    if (c.execute("WaitAI", param, result))
    {
        errcode = int(result);
        if(0 != errcode)
        {
            logger_error("fail, errorcode: %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 等待工具模拟量输入
 * @param  [in]  id  io编号，范围[0]
 * @param  [in]  sign 0-大于，1-小于
 * @param  [in]  value 输入电流或电压值百分比，范围[0~100]对应电流值[0~20mS]或电压[0~10V]
 * @param  [in]  max_time  最大等待时间，单位ms
 * @param  [in]  opt  超时后策略，0-程序停止并提示超时，1-忽略超时提示程序继续执行，2-一直等待
 * @return  错误码
 */
errno_t FRRobot::WaitToolAI(int id, int sign, float value, int max_time, int opt)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    param[1] = sign;
    param[2] = value;
    param[3] = max_time;
    param[4] = opt;

    if (c.execute("WaitToolAI", param, result))
    {
        errcode = int(result);
        if(0 != errcode)
        {
            logger_error("fail, errorcode: %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置全局速度
 * @param  [in]  vel  速度百分比，范围[0~100]
 * @return  错误码
 */
errno_t FRRobot::SetSpeed(int vel)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = vel;

    if((vel <= 10) || (vel>=90))
    {
        logger_warn("robot speed is: %d.", vel);
    }
    if (c.execute("SetSpeed", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置系统变量值
 * @param  [in]  id  变量编号，范围[1~20]
 * @param  [in]  value 变量值
 * @return  错误码
 */
errno_t FRRobot::SetSysVarValue(int id, float value)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    param[1] = value;

    if (c.execute("SetSysVarValue", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 设置工具参考点-六点法
 * @param [in] point_num 点编号,范围[1~6]
 * @return 错误码
 */
errno_t FRRobot::SetToolPoint(int point_num)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = point_num;

    if (c.execute("SetToolPoint", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  计算工具坐标系
 * @param [out] tcp_pose 工具坐标系
 * @return 错误码
 */
errno_t FRRobot::ComputeTool(DescPose *tcp_pose)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ComputeTool", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            tcp_pose->tran.x = double(result[1]);
            tcp_pose->tran.y = double(result[2]);
            tcp_pose->tran.z = double(result[3]);
            tcp_pose->rpy.rx = double(result[4]);
            tcp_pose->rpy.ry = double(result[5]);
            tcp_pose->rpy.rz = double(result[6]);
        }
        else{
            logger_error("execute ComputeTool fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 设置工具参考点-四点法
 * @param [in] point_num 点编号,范围[1~4]
 * @return 错误码
 */
errno_t FRRobot::SetTcp4RefPoint(int point_num)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = point_num;

    if (c.execute("SetTcp4RefPoint", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  计算工具坐标系
 * @param [out] tcp_pose 工具坐标系
 * @return 错误码
 */
errno_t FRRobot::ComputeTcp4(DescPose *tcp_pose)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ComputeTcp4", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            tcp_pose->tran.x = double(result[1]);
            tcp_pose->tran.y = double(result[2]);
            tcp_pose->tran.z = double(result[3]);
            tcp_pose->rpy.rx = double(result[4]);
            tcp_pose->rpy.ry = double(result[5]);
            tcp_pose->rpy.rz = double(result[6]);
        }
        else{
            logger_error("execute ComputeTcp4 fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置工具坐标系
 * @param  [in] id 坐标系编号，范围[1~15]
 * @param  [in] coord  工具中心点相对于末端法兰中心位姿
 * @param  [in] type  0-工具坐标系，1-传感器坐标系
 * @param  [in] install 安装位置，0-机器人末端，1-机器人外部
 * @return  错误码
 */
errno_t FRRobot::SetToolCoord(int id, DescPose *coord, int type, int install, int toolID, int loadNum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    param[1][0] = coord->tran.x;
    param[1][1] = coord->tran.y;
    param[1][2] = coord->tran.z;
    param[1][3] = coord->rpy.rx;
    param[1][4] = coord->rpy.ry;
    param[1][5] = coord->rpy.rz;
    param[2] = type;
    param[3] = install;
    param[4] = toolID;
    param[5] = loadNum;

    if (c.execute("SetToolCoord", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置工具坐标系列表
 * @param  [in] id 坐标系编号，范围[1~15]
 * @param  [in] coord  工具中心点相对于末端法兰中心位姿
 * @param  [in] type  0-工具坐标系，1-传感器坐标系
 * @param  [in] install 安装位置，0-机器人末端，1-机器人外部
 * @param  [in] loadNum 负载编号
 * @return  错误码
 */
errno_t FRRobot::SetToolList(int id, DescPose *coord, int type, int install, int loadNum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    param[1][0] = coord->tran.x;
    param[1][1] = coord->tran.y;
    param[1][2] = coord->tran.z;
    param[1][3] = coord->rpy.rx;
    param[1][4] = coord->rpy.ry;
    param[1][5] = coord->rpy.rz;
    param[2] = type;
    param[3] = install;
    param[4] = loadNum;

    if (c.execute("SetToolList", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 设置外部工具参考点-六点法
 * @param [in] point_num 点编号,范围[1~6]
 * @return 错误码
 */
errno_t FRRobot::SetExTCPPoint(int point_num)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = point_num;

    if (c.execute("SetExTCPPoint", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  计算外部工具坐标系
 * @param [out] tcp_pose 工具坐标系
 * @return 错误码
 */
errno_t FRRobot::ComputeExTCF(DescPose *tcp_pose)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ComputeExTCF", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            tcp_pose->tran.x = double(result[1]);
            tcp_pose->tran.y = double(result[2]);
            tcp_pose->tran.z = double(result[3]);
            tcp_pose->rpy.rx = double(result[4]);
            tcp_pose->rpy.ry = double(result[5]);
            tcp_pose->rpy.rz = double(result[6]);
        }
        else{
            logger_error("execute ComputeExTCF fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置外部工具坐标系
 * @param  [in] id 坐标系编号，范围[1~15]
 * @param  [in] etcp  外部工具坐标系
 * @param  [in] etool  末端工具坐标系
 * @return  错误码
 */
errno_t FRRobot::SetExToolCoord(int id, DescPose *etcp, DescPose *etool)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    param[1][0] = etcp->tran.x;
    param[1][1] = etcp->tran.y;
    param[1][2] = etcp->tran.z;
    param[1][3] = etcp->rpy.rx;
    param[1][4] = etcp->rpy.ry;
    param[1][5] = etcp->rpy.rz;
    param[2][0] = etool->tran.x;
    param[2][1] = etool->tran.y;
    param[2][2] = etool->tran.z;
    param[2][3] = etool->rpy.rx;
    param[2][4] = etool->rpy.ry;
    param[2][5] = etool->rpy.rz;

    if (c.execute("SetExToolCoord", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置外部工具坐标系列表
 * @param  [in] id 坐标系编号，范围[1~15]
 * @param  [in] etcp  工具中心点相对末端法兰中心位姿
 * @param  [in] etool  待定
 * @return  错误码
 */
errno_t FRRobot::SetExToolList(int id, DescPose *etcp, DescPose *etool)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    param[1][0] = etcp->tran.x;
    param[1][1] = etcp->tran.y;
    param[1][2] = etcp->tran.z;
    param[1][3] = etcp->rpy.rx;
    param[1][4] = etcp->rpy.ry;
    param[1][5] = etcp->rpy.rz;
    param[2][0] = etool->tran.x;
    param[2][1] = etool->tran.y;
    param[2][2] = etool->tran.z;
    param[2][3] = etool->rpy.rx;
    param[2][4] = etool->rpy.ry;
    param[2][5] = etool->rpy.rz;

    if (c.execute("SetExToolList", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 设置工件参考点-三点法
 * @param [in] point_num 点编号,范围[1~3]
 * @return 错误码
 */
errno_t FRRobot::SetWObjCoordPoint(int point_num)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = point_num;

    if (c.execute("SetWObjCoordPoint", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  计算工件坐标系
 * @param [in] method 计算方法 0：原点-x轴-z轴  1：原点-x轴-xy平面
 * @param [in] refFrame 参考坐标系
 * @param [out] wobj_pose 工件坐标系
 * @return 错误码
 */
errno_t FRRobot::ComputeWObjCoord(int method, int refFrame, DescPose *wobj_pose)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = method;
    param[1] = refFrame;

    if (c.execute("ComputeWObjCoord", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            wobj_pose->tran.x = double(result[1]);
            wobj_pose->tran.y = double(result[2]);
            wobj_pose->tran.z = double(result[3]);
            wobj_pose->rpy.rx = double(result[4]);
            wobj_pose->rpy.ry = double(result[5]);
            wobj_pose->rpy.rz = double(result[6]);
        }
        else{
            logger_error("execute ComputeWObjCoord fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置工件坐标系
 * @param  [in] id 坐标系编号，范围[1~15]
 * @param  [in] coord  工件坐标系相对于末端法兰中心位姿
 * @param  [in] refFrame 参考坐标系
 * @return  错误码
 */
errno_t FRRobot::SetWObjCoord(int id, DescPose *coord, int refFrame)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    param[1][0] = coord->tran.x;
    param[1][1] = coord->tran.y;
    param[1][2] = coord->tran.z;
    param[1][3] = coord->rpy.rx;
    param[1][4] = coord->rpy.ry;
    param[1][5] = coord->rpy.rz;
    param[2] = refFrame;

    if (c.execute("SetWObjCoord", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置工件坐标系列表
 * @param  [in] id 坐标系编号，范围[1~15]
 * @param  [in] coord  工件坐标系相对于末端法兰中心位姿
 * @param  [in] refFrame 参考坐标系
 * @return  错误码
 */
errno_t FRRobot::SetWObjList(int id, DescPose *coord, int refFrame)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    param[1][0] = coord->tran.x;
    param[1][1] = coord->tran.y;
    param[1][2] = coord->tran.z;
    param[1][3] = coord->rpy.rx;
    param[1][4] = coord->rpy.ry;
    param[1][5] = coord->rpy.rz;
    param[2] = refFrame;

    if (c.execute("SetWObjList", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置末端负载重量
 * @param  [in] loadNum 负载编号
 * @param  [in] weight  负载重量，单位kg
 * @return  错误码
 */
errno_t FRRobot::SetLoadWeight(int loadNum, float weight)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = loadNum;
    param[1] = weight;

    if (c.execute("SetLoadWeight", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置末端负载质心坐标
 * @param  [in] coord 质心坐标，单位mm
 * @return  错误码
 */
errno_t FRRobot::SetLoadCoord(DescTran *coord)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = coord->x;
    param[1] = coord->y;
    param[2] = coord->z;

    if (c.execute("SetLoadCoord", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置机器人安装方式
 * @param  [in] install  安装方式，0-正装，1-侧装，2-倒装
 * @return  错误码
 */
errno_t FRRobot::SetRobotInstallPos(uint8_t install)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = install;

    if (c.execute("SetRobotInstallPos", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置机器人安装角度，自由安装
 * @param  [in] yangle  倾斜角
 * @param  [in] zangle  旋转角
 * @return  错误码
 */
errno_t FRRobot::SetRobotInstallAngle(double yangle, double zangle)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = yangle;
    param[1] = zangle;

    if (c.execute("SetRobotInstallAngle", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  等待指定时间
 * @param  [in]  t_ms  单位ms
 * @return  错误码
 */
errno_t FRRobot::WaitMs(int t_ms)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = t_ms;

    if (c.execute("WaitMs", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 设置碰撞等级
 * @param  [in]  mode  0-等级，1-百分比
 * @param  [in]  level 碰撞阈值，等级对应范围[],百分比对应范围[0~1]
 * @param  [in]  config 0-不更新配置文件，1-更新配置文件
 * @return  错误码
 */
errno_t FRRobot::SetAnticollision(int mode, float level[6], int config)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = mode;
    param[1][0] = level[0];
    param[1][1] = level[1];
    param[1][2] = level[2];
    param[1][3] = level[3];
    param[1][4] = level[4];
    param[1][5] = level[5];
    param[2] = config;

    if (c.execute("SetAnticollision", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置碰撞后策略
 * @param  [in] strategy  0-报错停止，1-继续运行
 * @param  [in] safeTime  安全停止时间[1000 - 2000]ms
 * @param  [in] safeDistance  安全停止距离[1-150]mm
 * @param  [in] safeVel 安全速度[50-250] mm/s
 * @param  [in] safetyMargin  j1-j6安全系数[1-10]
 * @return  错误码
 */
errno_t FRRobot::SetCollisionStrategy(int strategy, int safeTime, int safeDistance, int safeVel, int safetyMargin[])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = strategy;
    param[1] = safeTime;
    param[2] = safeDistance;
    param[3] = safeVel;
    param[4][0] = safetyMargin[0];
    param[4][1] = safetyMargin[1];
    param[4][2] = safetyMargin[2];
    param[4][3] = safetyMargin[3];
    param[4][4] = safetyMargin[4];
    param[4][5] = safetyMargin[5];

    if (c.execute("SetCollisionStrategy", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置正限位
 * @param  [in] limit 六个关节位置，单位deg
 * @return  错误码
 */
errno_t FRRobot::SetLimitPositive(float limit[6])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0][0] = limit[0];
    param[0][1] = limit[1];
    param[0][2] = limit[2];
    param[0][3] = limit[3];
    param[0][4] = limit[4];
    param[0][5] = limit[5];

    if (c.execute("SetLimitPositive", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置负限位
 * @param  [in] limit 六个关节位置，单位deg
 * @return  错误码
 */
errno_t FRRobot::SetLimitNegative(float limit[6])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0][0] = limit[0];
    param[0][1] = limit[1];
    param[0][2] = limit[2];
    param[0][3] = limit[3];
    param[0][4] = limit[4];
    param[0][5] = limit[5];

    if (c.execute("SetLimitNegative", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  错误状态清除
 * @return  错误码
 */
errno_t FRRobot::ResetAllError()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ResetAllError", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  关节摩擦力补偿开关
 * @param  [in]  state  0-关，1-开
 * @return  错误码
 */
errno_t FRRobot::FrictionCompensationOnOff(uint8_t state)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = state;

    if (c.execute("FrictionCompensationOnOff", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置关节摩擦力补偿系数-正装
 * @param  [in]  coeff 六个关节补偿系数，范围[0~1]
 * @return  错误码
 */
errno_t FRRobot::SetFrictionValue_level(float coeff[6])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0][0] = coeff[0];
    param[0][1] = coeff[1];
    param[0][2] = coeff[2];
    param[0][3] = coeff[3];
    param[0][4] = coeff[4];
    param[0][5] = coeff[5];

    if (c.execute("SetFrictionValue_level", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置关节摩擦力补偿系数-侧装
 * @param  [in]  coeff 六个关节补偿系数，范围[0~1]
 * @return  错误码
 */
errno_t FRRobot::SetFrictionValue_wall(float coeff[6])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0][0] = coeff[0];
    param[0][1] = coeff[1];
    param[0][2] = coeff[2];
    param[0][3] = coeff[3];
    param[0][4] = coeff[4];
    param[0][5] = coeff[5];

    if (c.execute("SetFrictionValue_wall", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置关节摩擦力补偿系数-倒装
 * @param  [in]  coeff 六个关节补偿系数，范围[0~1]
 * @return  错误码
 */
errno_t FRRobot::SetFrictionValue_ceiling(float coeff[6])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0][0] = coeff[0];
    param[0][1] = coeff[1];
    param[0][2] = coeff[2];
    param[0][3] = coeff[3];
    param[0][4] = coeff[4];
    param[0][5] = coeff[5];

    if (c.execute("SetFrictionValue_ceiling", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置关节摩擦力补偿系数-自由安装
 * @param  [in]  coeff 六个关节补偿系数，范围[0~1]
 * @return  错误码
 */
errno_t FRRobot::SetFrictionValue_freedom(float coeff[6])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0][0] = coeff[0];
    param[0][1] = coeff[1];
    param[0][2] = coeff[2];
    param[0][3] = coeff[3];
    param[0][4] = coeff[4];
    param[0][5] = coeff[5];

    if (c.execute("SetFrictionValue_freedom", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  获取机器人安装角度
 * @param  [out] yangle 倾斜角
 * @param  [out] zangle 旋转角
 * @return  错误码
 */
errno_t FRRobot::GetRobotInstallAngle(float *yangle, float *zangle)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetRobotInstallAngle", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            *yangle = double(result[1]);
            *zangle = double(result[2]);
        }
        else{
            logger_error("execute GetRobotInstallAngle fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  获取系统变量值
 * @param  [in] id 系统变量编号，范围[1~20]
 * @param  [out] value  系统变量值
 * @return  错误码
 */
errno_t FRRobot::GetSysVarValue(int id, float *value)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = id;

    if (c.execute("GetSysVarValue", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            *value = double(result[1]);
        }
        else{
            logger_error("execute GetSysVarValue fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  获取当前关节位置(角度)
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] jPos 六个关节位置，单位deg
 * @return  错误码
 */
errno_t FRRobot::GetActualJointPosDegree(uint8_t flag, JointPos *jPos)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int i;
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        for (i = 0; i < 6; i++)
        {
            jPos->jPos[i] = robot_state_pkg->jt_cur_pos[i];
        }
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d, return value is: %f-%f-%f-%f-%f-%f", errcode, robot_state_pkg->jt_cur_pos[0],robot_state_pkg->jt_cur_pos[1],robot_state_pkg->jt_cur_pos[2],
                robot_state_pkg->jt_cur_pos[3],robot_state_pkg->jt_cur_pos[4],robot_state_pkg->jt_cur_pos[5]);
    return errcode;
}

/**
 * @brief  获取关节反馈速度-deg/s
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] speed [x,y,z,rx,ry,rz]速度
 * @return  错误码
 */
errno_t FRRobot::GetActualJointSpeedsDegree(uint8_t flag, float speed[6])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    int i;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        for (i = 0; i < 6; i++)
        {
            speed[i] = robot_state_pkg->actual_qd[i];
        }
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d", errcode);
    for (i = 0; i < 6; i++)
    {
        logger_info("return value %d is: %f", i, robot_state_pkg->actual_qd[i]);
    }
    return errcode;
}

/**
 * @brief  获取关节反馈加速度-deg/s^2
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] acc 六个关节加速度
 * @return  错误码
 */
errno_t FRRobot::GetActualJointAccDegree(uint8_t flag, float acc[6])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    int i;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        for (i = 0; i < 6; i++)
        {
            acc[i] = robot_state_pkg->actual_qdd[i];
        }
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d", errcode);
    for (i = 0; i < 6; i++)
    {
        logger_info("return value %d is: %f", i, robot_state_pkg->actual_qdd[i]);
    }
    return errcode;
}

/**
 * @brief  获取TCP指令速度
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] tcp_speed 线性速度
 * @param  [out] ori_speed 姿态速度
 * @return  错误码
 */
errno_t FRRobot::GetTargetTCPCompositeSpeed(uint8_t flag, float *tcp_speed, float *ori_speed)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *tcp_speed = robot_state_pkg->target_TCP_CmpSpeed[0];
        *ori_speed = robot_state_pkg->target_TCP_CmpSpeed[1];
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d", errcode);
    for (int i = 0; i < 2; i++)
    {
        logger_info("return value %d is: %f", i, robot_state_pkg->target_TCP_CmpSpeed[i]);
    }
    return errcode;
}

/**
 * @brief  获取TCP反馈速度
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] tcp_speed 线性速度
 * @param  [out] ori_speed 姿态速度
 * @return  错误码
 */
errno_t FRRobot::GetActualTCPCompositeSpeed(uint8_t flag, float *tcp_speed, float *ori_speed)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *tcp_speed = robot_state_pkg->actual_TCP_CmpSpeed[0];
        *ori_speed = robot_state_pkg->actual_TCP_CmpSpeed[1];
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d", errcode);
    for (int i = 0; i < 2; i++)
    {
        logger_info("return value %d is: %f", i, robot_state_pkg->actual_TCP_CmpSpeed[i]);
    }
    return errcode;
}

/**
 * @brief  获取TCP指令速度
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] speed [x,y,z,rx,ry,rz]速度
 * @return  错误码
 */
errno_t FRRobot::GetTargetTCPSpeed(uint8_t flag, float speed[6])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    int i;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        for (i = 0; i < 6; i++)
        {
            speed[i] = robot_state_pkg->target_TCP_Speed[i];
        }
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d", errcode);
    for (i = 0; i < 6; i++)
    {
        logger_info("return value %d is: %f", i, robot_state_pkg->target_TCP_Speed[i]);
    }
    return errcode;
}

/**
 * @brief  获取TCP反馈速度
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] speed [x,y,z,rx,ry,rz]速度
 * @return  错误码
 */
errno_t FRRobot::GetActualTCPSpeed(uint8_t flag, float speed[6])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    int i;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        for (i = 0; i < 6; i++)
        {
            speed[i] = robot_state_pkg->actual_TCP_Speed[i];
        }
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d", errcode);
    for (i = 0; i < 6; i++)
    {
        logger_info("return value %d is: %f", i, robot_state_pkg->actual_TCP_Speed[i]);
    }
    return errcode;
}

/**
 * @brief  获取当前工具位姿
 * @param  [in] flag  0-阻塞，1-非阻塞
 * @param  [out] desc_pos  工具位姿
 * @return  错误码
 */
errno_t FRRobot::GetActualTCPPose(uint8_t flag, DescPose *desc_pos)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        desc_pos->tran.x = robot_state_pkg->tl_cur_pos[0];
        desc_pos->tran.y = robot_state_pkg->tl_cur_pos[1];
        desc_pos->tran.z = robot_state_pkg->tl_cur_pos[2];
        desc_pos->rpy.rx = robot_state_pkg->tl_cur_pos[3];
        desc_pos->rpy.ry = robot_state_pkg->tl_cur_pos[4];
        desc_pos->rpy.rz = robot_state_pkg->tl_cur_pos[5];
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d, return value is: %f-%f-%f-%f-%f-%f", errcode, robot_state_pkg->tl_cur_pos[0],robot_state_pkg->tl_cur_pos[1],robot_state_pkg->tl_cur_pos[2],
                robot_state_pkg->tl_cur_pos[3],robot_state_pkg->tl_cur_pos[4],robot_state_pkg->tl_cur_pos[5]);
    return errcode;
}

/**
 * @brief  获取当前工具坐标系编号
 * @param  [in] flag  0-阻塞，1-非阻塞
 * @param  [out] id  工具坐标系编号
 * @return  错误码
 */
errno_t FRRobot::GetActualTCPNum(uint8_t flag, int *id)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *id = robot_state_pkg->tool;
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d", errcode);
    logger_info("return value is: %d", robot_state_pkg->tool);
    return errcode;
}

/**
 * @brief  获取当前工件坐标系编号
 * @param  [in] flag  0-阻塞，1-非阻塞
 * @param  [out] id  工件坐标系编号
 * @return  错误码
 */
errno_t FRRobot::GetActualWObjNum(uint8_t flag, int *id)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *id = robot_state_pkg->user;
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d", errcode);
    logger_info("return value is: %d", robot_state_pkg->user);
    return errcode;
}

/**
 * @brief  获取当前末端法兰位姿
 * @param  [in] flag  0-阻塞，1-非阻塞
 * @param  [out] desc_pos  法兰位姿
 * @return  错误码
 */
errno_t FRRobot::GetActualToolFlangePose(uint8_t flag, DescPose *desc_pos)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        desc_pos->tran.x = robot_state_pkg->flange_cur_pos[0];
        desc_pos->tran.y = robot_state_pkg->flange_cur_pos[1];
        desc_pos->tran.z = robot_state_pkg->flange_cur_pos[2];
        desc_pos->rpy.rx = robot_state_pkg->flange_cur_pos[3];
        desc_pos->rpy.ry = robot_state_pkg->flange_cur_pos[4];
        desc_pos->rpy.rz = robot_state_pkg->flange_cur_pos[5];
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d, return value is: %f-%f-%f-%f-%f-%f", errcode, robot_state_pkg->flange_cur_pos[0],robot_state_pkg->flange_cur_pos[1],robot_state_pkg->flange_cur_pos[2],
                robot_state_pkg->flange_cur_pos[3],robot_state_pkg->flange_cur_pos[4],robot_state_pkg->flange_cur_pos[5]);
    return errcode;
}

/**
 * @brief  逆运动学求解
 * @param  [in] type 0-绝对位姿(基坐标系)，1-增量位姿(基坐标系)，2-增量位姿(工具坐标系)
 * @param  [in] desc_pos 笛卡尔位姿
 * @param  [in] config 关节空间配置，[-1]-参考当前关节位置解算，[0~7]-依据特定关节空间配置求解
 * @param  [out] joint_pos 关节位置
 * @return  错误码
 */
errno_t FRRobot::GetInverseKin(int type, DescPose *desc_pos, int config, JointPos *joint_pos)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = type;
    param[1][0] = desc_pos->tran.x;
    param[1][1] = desc_pos->tran.y;
    param[1][2] = desc_pos->tran.z;
    param[1][3] = desc_pos->rpy.rx;
    param[1][4] = desc_pos->rpy.ry;
    param[1][5] = desc_pos->rpy.rz;
    param[2] = config;

    if (c.execute("GetInverseKin", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            joint_pos->jPos[0] = double(result[1]);
            joint_pos->jPos[1] = double(result[2]);
            joint_pos->jPos[2] = double(result[3]);
            joint_pos->jPos[3] = double(result[4]);
            joint_pos->jPos[4] = double(result[5]);
            joint_pos->jPos[5] = double(result[6]);
        }
        else{
            logger_error("execute GetInverseKin fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  逆运动学求解，参考指定关节位置求解
 * @param  [in] type 0-绝对位姿(基坐标系)，1-增量位姿(基坐标系)，2-增量位姿(工具坐标系)
 * @param  [in] desc_pos 笛卡尔位姿
 * @param  [in] joint_pos_ref 参考关节位置
 * @param  [out] joint_pos 关节位置
 * @return  错误码
 */
errno_t FRRobot::GetInverseKinRef(int type, DescPose *desc_pos, JointPos *joint_pos_ref, JointPos *joint_pos)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = type;
    param[1][0] = desc_pos->tran.x;
    param[1][1] = desc_pos->tran.y;
    param[1][2] = desc_pos->tran.z;
    param[1][3] = desc_pos->rpy.rx;
    param[1][4] = desc_pos->rpy.ry;
    param[1][5] = desc_pos->rpy.rz;
    param[2][0] = joint_pos_ref->jPos[0];
    param[2][1] = joint_pos_ref->jPos[1];
    param[2][2] = joint_pos_ref->jPos[2];
    param[2][3] = joint_pos_ref->jPos[3];
    param[2][4] = joint_pos_ref->jPos[4];
    param[2][5] = joint_pos_ref->jPos[5];

    if (c.execute("GetInverseKinRef", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            joint_pos->jPos[0] = double(result[1]);
            joint_pos->jPos[1] = double(result[2]);
            joint_pos->jPos[2] = double(result[3]);
            joint_pos->jPos[3] = double(result[4]);
            joint_pos->jPos[4] = double(result[5]);
            joint_pos->jPos[5] = double(result[6]);
        }
        else{
            logger_error("execute GetInverseKinRef fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  逆运动学求解，参考指定关节位置判断是否有解
 * @param  [in] posMode 0 绝对位姿，1 相对位姿-基坐标系，2 相对位姿-工具坐标系
 * @param  [in] desc_pos 笛卡尔位姿
 * @param  [in] joint_pos_ref 参考关节位置
 * @param  [out] result 0-无解，1-有解
 * @return  错误码
 */
errno_t FRRobot::GetInverseKinHasSolution(int type, DescPose *desc_pos, JointPos *joint_pos_ref, uint8_t *result)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, res;

    param[0] = type;
    param[1][0] = desc_pos->tran.x;
    param[1][1] = desc_pos->tran.y;
    param[1][2] = desc_pos->tran.z;
    param[1][3] = desc_pos->rpy.rx;
    param[1][4] = desc_pos->rpy.ry;
    param[1][5] = desc_pos->rpy.rz;

    param[2][0] = joint_pos_ref->jPos[0];
    param[2][1] = joint_pos_ref->jPos[1];
    param[2][2] = joint_pos_ref->jPos[2];
    param[2][3] = joint_pos_ref->jPos[3];
    param[2][4] = joint_pos_ref->jPos[4];
    param[2][5] = joint_pos_ref->jPos[5];

    if (c.execute("GetInverseKinHasSolution", param, res))
    {
        errcode = int(res[0]);
        if (errcode == 0)
        {
            *result = bool(res[1]);
        }
        else{
            logger_error("execute GetInverseKinHasSolution fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }
    c.close();

    return errcode;
}

/**
 * @brief  正运动学求解
 * @param  [in] joint_pos 关节位置
 * @param  [out] desc_pos 笛卡尔位姿
 * @return  错误码
 */
errno_t FRRobot::GetForwardKin(JointPos *joint_pos, DescPose *desc_pos)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0][0] = joint_pos->jPos[0];
    param[0][1] = joint_pos->jPos[1];
    param[0][2] = joint_pos->jPos[2];
    param[0][3] = joint_pos->jPos[3];
    param[0][4] = joint_pos->jPos[4];
    param[0][5] = joint_pos->jPos[5];

    if (c.execute("GetForwardKin", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            desc_pos->tran.x = double(result[1]);
            desc_pos->tran.y = double(result[2]);
            desc_pos->tran.z = double(result[3]);
            desc_pos->rpy.rx = double(result[4]);
            desc_pos->rpy.ry = double(result[5]);
            desc_pos->rpy.rz = double(result[6]);
        }
        else{
            logger_error("execute GetForwardKin fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 获取当前关节转矩
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] torques 关节转矩
 * @return  错误码
 */
errno_t FRRobot::GetJointTorques(uint8_t flag, float torques[6])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    int i;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        for (i = 0; i < 6; i++)
        {
            torques[i] = robot_state_pkg->jt_cur_tor[i];
        }
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d", errcode);
    for (i = 0; i < 6; i++)
    {
        logger_info("return value %d is: %f", i, robot_state_pkg->jt_cur_tor[i]);
    }
    return errcode;
}

/**
 * @brief  获取当前负载的重量
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] weight 负载重量，单位kg
 * @return  错误码
 */
errno_t FRRobot::GetTargetPayload(uint8_t flag, float *weight)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = flag;

    if (c.execute("GetTargetPayload", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            *weight = double(result[1]);
        }
        else{
            logger_error("execute GetTargetPayload fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  获取当前负载的质心
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] cog 负载质心，单位mm
 * @return  错误码
 */
errno_t FRRobot::GetTargetPayloadCog(uint8_t flag, DescTran *cog)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = flag;

    if (c.execute("GetTargetPayloadCog", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            cog->x = double(result[1]);
            cog->y = double(result[2]);
            cog->z = double(result[3]);
        }
        else{
            logger_error("execute GetTargetPayloadCog fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  获取当前工具坐标系
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] desc_pos 工具坐标系位姿
 * @return  错误码
 */
errno_t FRRobot::GetTCPOffset(uint8_t flag, DescPose *desc_pos)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = flag;

    if (c.execute("GetTCPOffset", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            desc_pos->tran.x = double(result[1]);
            desc_pos->tran.y = double(result[2]);
            desc_pos->tran.z = double(result[3]);
            desc_pos->rpy.rx = double(result[4]);
            desc_pos->rpy.ry = double(result[5]);
            desc_pos->rpy.rz = double(result[6]);
        }
        else{
            logger_error("execute GetTCPOffset fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  获取当前工件坐标系
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] desc_pos 工件坐标系位姿
 * @return  错误码
 */
errno_t FRRobot::GetWObjOffset(uint8_t flag, DescPose *desc_pos)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = flag;

    if (c.execute("GetWObjOffset", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            desc_pos->tran.x = double(result[1]);
            desc_pos->tran.y = double(result[2]);
            desc_pos->tran.z = double(result[3]);
            desc_pos->rpy.rx = double(result[4]);
            desc_pos->rpy.ry = double(result[5]);
            desc_pos->rpy.rz = double(result[6]);
        }
        else{
            logger_error("execute GetWObjOffset fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  获取关节软限位角度
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] negative  负限位角度，单位deg
 * @param  [out] positive  正限位角度，单位deg
 * @return  错误码
 */
errno_t FRRobot::GetJointSoftLimitDeg(uint8_t flag, float negative[6], float positive[6])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = flag;

    if (c.execute("GetJointSoftLimitDeg", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            negative[0] = double(result[1]);
            positive[0] = double(result[2]);
            negative[1] = double(result[3]);
            positive[1] = double(result[4]);
            negative[2] = double(result[5]);
            positive[2] = double(result[6]);
            negative[3] = double(result[7]);
            positive[3] = double(result[8]);
            negative[4] = double(result[9]);
            positive[4] = double(result[10]);
            negative[5] = double(result[11]);
            positive[5] = double(result[12]);
        }
        else{
            logger_error("execute GetJointSoftLimitDeg fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  获取系统时间
 * @param  [out] t_ms 单位ms
 * @return  错误码
 */
errno_t FRRobot::GetSystemClock(float *t_ms)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetSystemClock", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            *t_ms = double(result[1]);
        }
        else{
            logger_error("execute GetSystemClock fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  获取机器人当前关节位置
 * @param  [out]  config  关节空间配置，范围[0~7]
 * @return  错误码
 */
errno_t FRRobot::GetRobotCurJointsConfig(int *config)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetRobotCurJointsConfig", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            *config = int(result[1]);
        }
        else{
            logger_error("execute GetRobotCurJointsConfig fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  获取机器人默认速度
 * @param  [out]  vel  速度，单位mm/s
 * @return  错误码
 */
errno_t FRRobot::GetDefaultTransVel(float *vel)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetDefaultTransVel", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            *vel = double(result[1]);
        }
        else{
            logger_error("execute GetDefaultTransVel fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  查询机器人运动是否完成
 * @param  [out]  state  0-未完成，1-完成
 * @return  错误码
 */
errno_t FRRobot::GetRobotMotionDone(uint8_t *state)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *state = robot_state_pkg->motion_done;
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d, return value is: %d.", errcode, robot_state_pkg->motion_done);
    return errcode;
}

/**
 * @brief  查询机器人错误码
 * @param  [out]  maincode  主错误码
 * @param  [out]  subcode   子错误码
 * @return  错误码
 */
errno_t FRRobot::GetRobotErrorCode(int *maincode, int *subcode)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *maincode = robot_state_pkg->main_code;
        *subcode = robot_state_pkg->sub_code;
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d, return value is: %d-%d", errcode, robot_state_pkg->main_code, robot_state_pkg->sub_code);
    return errcode;
}

/**
 * @brief  查询机器人示教管理点位数据
 * @param  [in]  name  点位名
 * @param  [out]  data   点位数据
 * @return  错误码
 */
errno_t FRRobot::GetRobotTeachingPoint(char name[64], float data[20])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = name;

    if (c.execute("GetRobotTeachingPoint", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            string paramStr = (string)result[1];
            std::vector<std::string> parS = split(paramStr, ',');
            if (parS.size() != 20)
            {
                logger_error("get Teaching Point size fail");
                return -1;
            }
            for (int i = 0; i < 20; i++)
            {
                data[i] = stod(parS[i]);
            }
        }
        else{
            logger_error("execute GetRobotTeachingPoint fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  查询机器人运动队列缓存长度
 * @param  [out]  len  缓存长度
 * @return  错误码
 */
errno_t FRRobot::GetMotionQueueLength(int *len)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *len = robot_state_pkg->mc_queue_len;
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d, return value is: %u.", errcode, robot_state_pkg->mc_queue_len);
    return errcode;
}

/**
 * @brief  设置轨迹记录参数
 * @param  [in] type  记录数据类型，1-关节位置
 * @param  [in] name  轨迹文件名
 * @param  [in] period_ms  数据采样周期，固定值2ms或4ms或8ms
 * @param  [in] di_choose  DI选择,bit0~bit7对应控制箱DI0~DI7，bit8~bit9对应末端DI0~DI1，0-不选择，1-选择
 * @param  [in] do_choose  DO选择,bit0~bit7对应控制箱DO0~DO7，bit8~bit9对应末端DO0~DO1，0-不选择，1-选择
 * @return  错误码
 */
errno_t FRRobot::SetTPDParam(int type, char name[30], int period_ms, uint16_t di_choose, uint16_t do_choose)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = type;
    param[1] = name;
    param[2] = period_ms;
    param[3] = di_choose;
    param[4] = do_choose;

    if (c.execute("SetTPDParam", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  开始轨迹记录
 * @param  [in] type  记录数据类型，1-关节位置
 * @param  [in] name  轨迹文件名
 * @param  [in] period_ms  数据采样周期，固定值2ms或4ms或8ms
 * @param  [in] di_choose  DI选择,bit0~bit7对应控制箱DI0~DI7，bit8~bit9对应末端DI0~DI1，0-不选择，1-选择
 * @param  [in] do_choose  DO选择,bit0~bit7对应控制箱DO0~DO7，bit8~bit9对应末端DO0~DO1，0-不选择，1-选择
 * @return  错误码
 */
errno_t FRRobot::SetTPDStart(int type, char name[30], int period_ms, uint16_t di_choose, uint16_t do_choose)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = type;
    param[1] = name;
    param[2] = period_ms;
    param[3] = di_choose;
    param[4] = do_choose;

    if (c.execute("SetTPDStart", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  停止轨迹记录
 * @return  错误码
 */
errno_t FRRobot::SetWebTPDStop()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("SetWebTPDStop", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  删除轨迹记录
 * @param  [in] name  轨迹文件名
 * @return  错误码
 */
errno_t FRRobot::SetTPDDelete(char name[30])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = name;

    if (c.execute("SetTPDDelete", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  轨迹预加载
 * @param  [in] name  轨迹文件名
 * @return  错误码
 */
errno_t FRRobot::LoadTPD(char name[30])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = name;

    if (c.execute("LoadTPD", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  获取轨迹起始位姿
 * @param  [in] name 轨迹文件名
 * @return  错误码
 */
errno_t FRRobot::GetTPDStartPose(char name[30], DescPose *desc_pose)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = name;

    if (c.execute("GetTPDStartPose", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            desc_pose->tran.x = double(result[1]);
            desc_pose->tran.y = double(result[2]);
            desc_pose->tran.z = double(result[3]);
            desc_pose->rpy.rx = double(result[4]);
            desc_pose->rpy.ry = double(result[5]);
            desc_pose->rpy.rz = double(result[6]);
        }
        else{
            logger_error("execute GetTPDStartPose fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  轨迹复现
 * @param  [in] name  轨迹文件名
 * @param  [in] blend 0-不平滑，1-平滑
 * @param  [in] ovl  速度缩放百分比，范围[0~100]
 * @return  错误码
 */
errno_t FRRobot::MoveTPD(char name[30], uint8_t blend, float ovl)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = name;
    param[1] = blend;
    param[2] = ovl;

    if (c.execute("MoveTPD", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  轨迹预处理
 * @param  [in] name  轨迹文件名
 * @param  [in] ovl 速度缩放百分比，范围[0~100]
 * @param  [in] opt 1-控制点，默认为1
 * @return  错误码
 */
errno_t FRRobot::LoadTrajectoryJ(char name[30], float ovl, int opt)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = name;
    param[1] = ovl;
    param[2] = opt;

    if (c.execute("LoadTrajectoryJ", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  轨迹复现
 * @return  错误码
 */
errno_t FRRobot::MoveTrajectoryJ()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("MoveTrajectoryJ", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  获取轨迹起始位姿
 * @param  [in] name 轨迹文件名
 * @return  错误码
 */
errno_t FRRobot::GetTrajectoryStartPose(char name[30], DescPose *desc_pose)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = name;

    if (c.execute("GetTrajectoryStartPose", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            desc_pose->tran.x = double(result[1]);
            desc_pose->tran.y = double(result[2]);
            desc_pose->tran.z = double(result[3]);
            desc_pose->rpy.rx = double(result[4]);
            desc_pose->rpy.ry = double(result[5]);
            desc_pose->rpy.rz = double(result[6]);
        }
        else{
            logger_error("execute GetTrajectoryStartPose fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  获取轨迹点编号
 * @return  错误码
 */
errno_t FRRobot::GetTrajectoryPointNum(int *pnum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *pnum = robot_state_pkg->trajectory_pnum;
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d, return value is: %d.", errcode, robot_state_pkg->trajectory_pnum);
    return errcode;
}

/**
 * @brief  设置轨迹运行中的速度
 * @param  [in] ovl 速度百分比
 * @return  错误码
 */
errno_t FRRobot::SetTrajectoryJSpeed(float ovl)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = ovl;

    if (c.execute("SetTrajectoryJSpeed", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置轨迹运行中的力和扭矩
 * @param  [in] ft 三个方向的力和扭矩，单位N和Nm
 * @return  错误码
 */
errno_t FRRobot::SetTrajectoryJForceTorque(ForceTorque *ft)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0][0] = ft->fx;
    param[0][1] = ft->fy;
    param[0][2] = ft->fz;
    param[0][3] = ft->tx;
    param[0][4] = ft->ty;
    param[0][5] = ft->tz;

    if (c.execute("SetTrajectoryJForceTorque", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置轨迹运行中的沿x方向的力
 * @param  [in] fx 沿x方向的力，单位N
 * @return  错误码
 */
errno_t FRRobot::SetTrajectoryJForceFx(double fx)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = fx;

    if (c.execute("SetTrajectoryJForceFx", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置轨迹运行中的沿y方向的力
 * @param  [in] fy 沿y方向的力，单位N
 * @return  错误码
 */
errno_t FRRobot::SetTrajectoryJForceFy(double fy)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = fy;

    if (c.execute("SetTrajectoryJForceFy", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置轨迹运行中的沿z方向的力
 * @param  [in] fz 沿x方向的力，单位N
 * @return  错误码
 */
errno_t FRRobot::SetTrajectoryJForceFz(double fz)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = fz;

    if (c.execute("SetTrajectoryJForceFz", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置轨迹运行中的绕x轴的扭矩
 * @param  [in] tx 绕x轴的扭矩，单位Nm
 * @return  错误码
 */
errno_t FRRobot::SetTrajectoryJTorqueTx(double tx)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = tx;

    if (c.execute("SetTrajectoryJTorqueTx", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置轨迹运行中的绕x轴的扭矩
 * @param  [in] ty 绕y轴的扭矩，单位Nm
 * @return  错误码
 */
errno_t FRRobot::SetTrajectoryJTorqueTy(double ty)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = ty;

    if (c.execute("SetTrajectoryJTorqueTy", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置轨迹运行中的绕x轴的扭矩
 * @param  [in] tz 绕z轴的扭矩，单位Nm
 * @return  错误码
 */
errno_t FRRobot::SetTrajectoryJTorqueTz(double tz)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = tz;

    if (c.execute("SetTrajectoryJTorqueTz", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置开机自动加载默认的作业程序
 * @param  [in] flag  0-开机不自动加载默认程序，1-开机自动加载默认程序
 * @param  [in] program_name 作业程序名及路径，如"/fruser/movej.lua"，其中"/fruser/"为固定路径
 * @return  错误码
 */
errno_t FRRobot::LoadDefaultProgConfig(uint8_t flag, char program_name[64])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = flag;
    param[1] = program_name;

    if (c.execute("LoadDefaultProgConfig", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  加载指定的作业程序
 * @param  [in] program_name 作业程序名及路径，如"/fruser/movej.lua"，其中"/fruser/"为固定路径
 * @return  错误码
 */
errno_t FRRobot::ProgramLoad(char program_name[64])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = program_name;

    if (c.execute("ProgramLoad", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  获取已加载的作业程序名
 * @param  [out] program_name 作业程序名及路径，如"/fruser/movej.lua"，其中"/fruser/"为固定路径
 * @return  错误码
 */
errno_t FRRobot::GetLoadedProgram(char program_name[64])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetLoadedProgram", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            std::string s(result[1]);
            strcpy(program_name, s.c_str());
        }
        else
        {
            const char *res = "";
            strcpy(program_name, res);
            logger_error("execute GetLoadedProgram fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  获取当前机器人作业程序执行的行号
 * @param  [out] line  行号
 * @return  错误码
 */
errno_t FRRobot::GetCurrentLine(int *line)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetCurrentLine", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            *line = int(result[1]);
        }
        else{
            logger_error("execute GetCurrentLine fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  运行当前加载的作业程序
 * @return  错误码
 */
errno_t FRRobot::ProgramRun()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ProgramRun", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  暂停当前运行的作业程序
 * @return  错误码
 */
errno_t FRRobot::ProgramPause()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ProgramPause", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  恢复当前暂停的作业程序
 * @return  错误码
 */
errno_t FRRobot::ProgramResume()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ProgramResume", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  终止当前运行的作业程序
 * @return  错误码
 */
errno_t FRRobot::ProgramStop()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ProgramStop", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  获取机器人作业程序执行状态
 * @param  [out]  state 1-程序停止或无程序运行，2-程序运行中，3-程序暂停
 * @return  错误码
 */
errno_t FRRobot::GetProgramState(uint8_t *state)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *state = robot_state_pkg->robot_state;
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d, return value is: %u", errcode, robot_state_pkg->robot_state);
    return errcode;
}

/**
 * @brief  配置夹爪
 * @param  [in] company  夹爪厂商，待定
 * @param  [in] device  设备号，暂不使用，默认为0
 * @param  [in] softvesion  软件版本号，暂不使用，默认为0
 * @param  [in] bus 设备挂在末端总线位置，暂不使用，默认为0
 * @return  错误码
 */
errno_t FRRobot::SetGripperConfig(int company, int device, int softvesion, int bus)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = company;
    param[1] = device;
    param[2] = softvesion;
    param[3] = bus;

    if (c.execute("SetGripperConfig", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  获取夹爪配置
 * @param  [out] company  夹爪厂商，待定
 * @param  [out] device  设备号，暂不使用，默认为0
 * @param  [out] softvesion  软件版本号，暂不使用，默认为0
 * @param  [out] bus 设备挂在末端总线位置，暂不使用，默认为0
 * @return  错误码
 */
errno_t FRRobot::GetGripperConfig(int *company, int *device, int *softvesion, int *bus)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetGripperConfig", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            *company = int(result[1]);
            *device = int(result[2]);
            *softvesion = int(result[3]);
            *bus = int(result[4]);
        }
        else{
            logger_error("execute GetGripperConfig fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  激活夹爪
 * @param  [in] index  夹爪编号
 * @param  [in] act  0-复位，1-激活
 * @return  错误码
 */
errno_t FRRobot::ActGripper(int index, uint8_t act)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = index;
    param[1] = act;

    if (c.execute("ActGripper", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  控制夹爪
 * @param  [in] index  夹爪编号
 * @param  [in] pos  位置百分比，范围[0~100]
 * @param  [in] vel  速度百分比，范围[0~100]
 * @param  [in] force  力矩百分比，范围[0~100]
 * @param  [in] max_time  最大等待时间，范围[0~30000]，单位ms
 * @param  [in] block  0-阻塞，1-非阻塞
 * @param  [in] type 夹爪类型，0-平行夹爪；1-旋转夹爪
 * @param  [in] rotNum 旋转圈数
 * @param  [in] rotVel 旋转速度百分比[0-100]
 * @param  [in] rotTorque 旋转力矩百分比[0-100]
 * @return  错误码
 */
errno_t FRRobot::MoveGripper(int index, int pos, int vel, int force, int max_time, uint8_t block, int type, double rotNum, int rotVel, int rotTorque)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = index;
    param[1] = pos;
    param[2] = vel;
    param[3] = force;
    param[4] = max_time;
    param[5] = block;
    param[6] = type;
    param[7] = rotNum;
    param[8] = rotVel;
    param[9] = rotTorque;

    if (c.execute("MoveGripper", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  获取夹爪运动状态
 * @param  [out] fault  0-无错误，1-有错误
 * @param  [out] staus  0-运动未完成，1-运动完成
 * @return  错误码
 */
errno_t FRRobot::GetGripperMotionDone(uint16_t *fault, uint8_t *status)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *fault = robot_state_pkg->gripper_fault;
        *status = robot_state_pkg->gripper_motiondone;
    }
    else
    {
        errcode = g_sock_com_err;
    }

    //logger_info("state com: %d, return value is: %u - %u.", errcode, robot_state_pkg->gripper_fault, robot_state_pkg->gripper_motiondone);
    return errcode;
}

/**
 * @brief  获取夹爪激活状态
 * @param  [out] fault  0-无错误，1-有错误
 * @param  [out] status  bit0~bit15对应夹爪编号0~15，bit=0为未激活，bit=1为激活
 * @return  错误码
 */
errno_t FRRobot::GetGripperActivateStatus(uint16_t *fault, uint16_t *status)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *fault = robot_state_pkg->gripper_fault;
        *status = robot_state_pkg->gripper_active;
    }
    else
    {
        errcode = g_sock_com_err;
    }

    return errcode;
}

/**
 * @brief  获取夹爪位置
 * @param  [out] fault  0-无错误，1-有错误
 * @param  [out] position  位置百分比，范围0~100%
 * @return  错误码
 */
errno_t FRRobot::GetGripperCurPosition(uint16_t *fault, uint8_t *position)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *fault = robot_state_pkg->gripper_fault;
        *position = robot_state_pkg->gripper_position;
    }
    else
    {
        errcode = g_sock_com_err;
    }

    return errcode;
}

/**
 * @brief  获取夹爪速度
 * @param  [out] fault  0-无错误，1-有错误
 * @param  [out] speed  速度百分比，范围0~100%
 * @return  错误码
 */
errno_t FRRobot::GetGripperCurSpeed(uint16_t *fault, int8_t *speed)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *fault = robot_state_pkg->gripper_fault;
        *speed = robot_state_pkg->gripper_speed;
    }
    else
    {
        errcode = g_sock_com_err;
    }

    return errcode;
}

/**
 * @brief  获取夹爪电流
 * @param  [out] fault  0-无错误，1-有错误
 * @param  [out] current  电流百分比，范围0~100%
 * @return  错误码
 */
errno_t FRRobot::GetGripperCurCurrent(uint16_t *fault, int8_t *current)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *fault = robot_state_pkg->gripper_fault;
        *current = robot_state_pkg->gripper_current;
    }
    else
    {
        errcode = g_sock_com_err;
    }

    return errcode;
}

/**
 * @brief  获取夹爪电压
 * @param  [out] fault  0-无错误，1-有错误
 * @param  [out] voltage  电压,单位0.1V
 * @return  错误码
 */
errno_t FRRobot::GetGripperVoltage(uint16_t *fault, int *voltage)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *fault = robot_state_pkg->gripper_fault;
        *voltage = robot_state_pkg->gripper_voltage;
    }
    else
    {
        errcode = g_sock_com_err;
    }

    return errcode;
}

/**
 * @brief  获取夹爪温度
 * @param  [out] fault  0-无错误，1-有错误
 * @param  [out] temp  温度，单位℃
 * @return  错误码
 */
errno_t FRRobot::GetGripperTemp(uint16_t *fault, int *temp)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *fault = robot_state_pkg->gripper_fault;
        *temp = robot_state_pkg->gripper_temp;
    }
    else
    {
        errcode = g_sock_com_err;
    }

    return errcode;
}

/**
 * @brief  获取旋转夹爪的旋转圈数
 * @param  [out] fault  0-无错误，1-有错误
 * @param  [out] num  旋转圈数
 * @return  错误码
 */
errno_t FRRobot::GetGripperRotNum(uint16_t* fault, double* num)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *fault = robot_state_pkg->gripper_fault;
        *num = robot_state_pkg->gripperRotNum;
    }
    else
    {
        errcode = g_sock_com_err;
    }

    
    return errcode;
}

/**
 * @brief  获取旋转夹爪的旋转速度百分比
 * @param  [out] fault  0-无错误，1-有错误
 * @param  [out] speed  旋转速度百分比
 * @return  错误码
 */
errno_t FRRobot::GetGripperRotSpeed(uint16_t* fault, int* speed)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *fault = robot_state_pkg->gripper_fault;
        *speed = robot_state_pkg->gripperRotSpeed;
    }
    else
    {
        errcode = g_sock_com_err;
    }

    return errcode;
}

/**
 * @brief  获取旋转夹爪的旋转力矩百分比
 * @param  [out] fault  0-无错误，1-有错误
 * @param  [out] torque  旋转力矩百分比
 * @return  错误码
 */
errno_t FRRobot::GetGripperRotTorque(uint16_t* fault, int* torque)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *fault = robot_state_pkg->gripper_fault;
        *torque = robot_state_pkg->gripperRotTorque;
    }
    else
    {
        errcode = g_sock_com_err;
    }

    return errcode;
}

/**
 * @brief  计算预抓取点-视觉
 * @param  [in] desc_pos  抓取点笛卡尔位姿
 * @param  [in] zlength   z轴偏移量
 * @param  [in] zangle    绕z轴旋转偏移量
 * @param  [out] pre_pos  预抓取点
 * @return  错误码
 */
errno_t FRRobot::ComputePrePick(DescPose *desc_pos, double zlength, double zangle, DescPose *pre_pos)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0][0] = desc_pos->tran.x;
    param[0][1] = desc_pos->tran.y;
    param[0][2] = desc_pos->tran.z;
    param[0][3] = desc_pos->rpy.rx;
    param[0][4] = desc_pos->rpy.ry;
    param[0][5] = desc_pos->rpy.rz;
    param[1] = zlength;
    param[2] = zangle;

    if (c.execute("ComputePrePick", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            pre_pos->tran.x = double(result[1]);
            pre_pos->tran.y = double(result[2]);
            pre_pos->tran.z = double(result[3]);
            pre_pos->rpy.rx = double(result[4]);
            pre_pos->rpy.ry = double(result[5]);
            pre_pos->rpy.rz = double(result[6]);
        }
        else{
            logger_error("execute ComputePrePick fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  计算撤退点-视觉
 * @param  [in] desc_pos  抓取点笛卡尔位姿
 * @param  [in] zlength   z轴偏移量
 * @param  [in] zangle    绕z轴旋转偏移量
 * @param  [out] post_pos 撤退点
 * @return  错误码
 */
errno_t FRRobot::ComputePostPick(DescPose *desc_pos, double zlength, double zangle, DescPose *post_pos)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0][0] = desc_pos->tran.x;
    param[0][1] = desc_pos->tran.y;
    param[0][2] = desc_pos->tran.z;
    param[0][3] = desc_pos->rpy.rx;
    param[0][4] = desc_pos->rpy.ry;
    param[0][5] = desc_pos->rpy.rz;
    param[1] = zlength;
    param[2] = zangle;

    if (c.execute("ComputePostPick", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            post_pos->tran.x = double(result[1]);
            post_pos->tran.y = double(result[2]);
            post_pos->tran.z = double(result[3]);
            post_pos->rpy.rx = double(result[4]);
            post_pos->rpy.ry = double(result[5]);
            post_pos->rpy.rz = double(result[6]);
        }
        else{
            logger_error("execute ComputePostPick fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  配置力传感器
 * @param  [in] company  力传感器厂商，17-坤维科技
 * @param  [in] device  设备号，暂不使用，默认为0
 * @param  [in] softvesion  软件版本号，暂不使用，默认为0
 * @param  [in] bus 设备挂在末端总线位置，暂不使用，默认为0
 * @return  错误码
 */
errno_t FRRobot::FT_SetConfig(int company, int device, int softvesion, int bus)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = company;
    param[1] = device;
    param[2] = softvesion;
    param[3] = bus;

    if (c.execute("FT_SetConfig", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  获取力传感器配置
 * @param  [out] company  力传感器厂商，待定
 * @param  [out] device  设备号，暂不使用，默认为0
 * @param  [out] softvesion  软件版本号，暂不使用，默认为0
 * @param  [out] bus 设备挂在末端总线位置，暂不使用，默认为0
 * @return  错误码
 */
errno_t FRRobot::FT_GetConfig(int *company, int *device, int *softvesion, int *bus)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("FT_GetConfig", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            *company = int(result[1]);
            *device = int(result[2]);
            *softvesion = int(result[3]);
            *bus = int(result[4]);
        }
        else{
            logger_error("execute FT_GetConfig fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  力传感器激活
 * @param  [in] act  0-复位，1-激活
 * @return  错误码
 */
errno_t FRRobot::FT_Activate(uint8_t act)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = act;

    if (c.execute("FT_Activate", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  力传感器校零
 * @param  [in] act  0-去除零点，1-零点矫正
 * @return  错误码
 */
errno_t FRRobot::FT_SetZero(uint8_t act)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = act;

    if (c.execute("FT_SetZero", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  设置力传感器参考坐标系
 * @param  [in] ref  0-工具坐标系，1-基坐标系， 2-自定义坐标系
 * @param  [in] coord  自定义坐标系值
 * @return  错误码
 */
errno_t FRRobot::FT_SetRCS(uint8_t ref, DescPose coord)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = ref;
    param[1][0] = coord.tran.x;
    param[1][1] = coord.tran.y;
    param[1][2] = coord.tran.z;
    param[1][3] = coord.rpy.rx;
    param[1][4] = coord.rpy.ry;
    param[1][5] = coord.rpy.rz;

    if (c.execute("FT_SetRCS", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  负载重量辨识记录
 * @param  [in] id  传感器坐标系编号，范围[1~14]
 * @return  错误码
 */
errno_t FRRobot::FT_PdIdenRecord(int id)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = id;

    if (c.execute("FT_PdIdenRecord", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  负载重量辨识计算
 * @param  [out] weight  负载重量，单位kg
 * @return  错误码
 */
errno_t FRRobot::FT_PdIdenCompute(float *weight)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("FT_PdIdenCompute", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            *weight = double(result[1]);
        }
        else{
            logger_error("execute FT_PdIdenCompute fail %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  负载质心辨识记录
 * @param  [in] id  传感器坐标系编号，范围[1~14]
 * @param  [in] index 点编号，范围[1~3]
 * @return  错误码
 */
errno_t FRRobot::FT_PdCogIdenRecord(int id, int index)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    param[1] = index;

    if (c.execute("FT_PdCogIdenRecord", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  负载质心辨识计算
 * @param  [out] cog  负载质心，单位mm
 * @return  错误码
 */
errno_t FRRobot::FT_PdCogIdenCompute(DescTran *cog)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("FT_PdCogIdenCompute", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            cog->x = double(result[1]);
            cog->y = double(result[2]);
            cog->z = double(result[3]);
        }
        else{
            logger_error("execute FT_PdCogIdenCompute fail %d", errcode);
        }
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  获取参考坐标系下力/扭矩数据
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] ft  力/扭矩，fx,fy,fz,tx,ty,tz
 * @return  错误码
 */
errno_t FRRobot::FT_GetForceTorqueRCS(uint8_t flag, ForceTorque *ft)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        ft->fx = robot_state_pkg->ft_sensor_data[0];
        ft->fy = robot_state_pkg->ft_sensor_data[1];
        ft->fz = robot_state_pkg->ft_sensor_data[2];
        ft->tx = robot_state_pkg->ft_sensor_data[3];
        ft->ty = robot_state_pkg->ft_sensor_data[4];
        ft->tz = robot_state_pkg->ft_sensor_data[5];
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d", errcode);
    for (int i = 0; i < 6; i++)
    {
        logger_info("return value %d is: %f", i, robot_state_pkg->ft_sensor_data[i]);
    }
    return errcode;
}

/**
 * @brief  获取力传感器原始力/扭矩数据
 * @param  [in] flag 0-阻塞，1-非阻塞
 * @param  [out] ft  力/扭矩，fx,fy,fz,tx,ty,tz
 * @return  错误码
 */
errno_t FRRobot::FT_GetForceTorqueOrigin(uint8_t flag, ForceTorque *ft)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        ft->fx = robot_state_pkg->ft_sensor_raw_data[0];
        ft->fy = robot_state_pkg->ft_sensor_raw_data[1];
        ft->fz = robot_state_pkg->ft_sensor_raw_data[2];
        ft->tx = robot_state_pkg->ft_sensor_raw_data[3];
        ft->ty = robot_state_pkg->ft_sensor_raw_data[4];
        ft->tz = robot_state_pkg->ft_sensor_raw_data[5];
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d", errcode);
    for (int i = 0; i < 6; i++)
    {
        logger_info("return value %d is: %f", i, robot_state_pkg->ft_sensor_raw_data[i]);
    }
    return errcode;
}

/**
 * @brief  碰撞守护
 * @param  [in] flag 0-关闭碰撞守护，1-开启碰撞守护
 * @param  [in] sensor_id 力传感器编号
 * @param  [in] select  选择六个自由度是否检测碰撞，0-不检测，1-检测
 * @param  [in] ft  碰撞力/扭矩，fx,fy,fz,tx,ty,tz
 * @param  [in] max_threshold 最大阈值
 * @param  [in] min_threshold 最小阈值
 * @note   力/扭矩检测范围：(ft-min_threshold, ft+max_threshold)
 * @return  错误码
 */
errno_t FRRobot::FT_Guard(uint8_t flag, int sensor_id, uint8_t select[6], ForceTorque *ft, float max_threshold[6], float min_threshold[6])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = flag;
    param[1] = sensor_id;
    param[2][0] = select[0];
    param[2][1] = select[1];
    param[2][2] = select[2];
    param[2][3] = select[3];
    param[2][4] = select[4];
    param[2][5] = select[5];
    param[3][0] = ft->fx;
    param[3][1] = ft->fy;
    param[3][2] = ft->fz;
    param[3][3] = ft->tx;
    param[3][4] = ft->ty;
    param[3][5] = ft->tz;
    param[4][0] = max_threshold[0];
    param[4][1] = max_threshold[1];
    param[4][2] = max_threshold[2];
    param[4][3] = max_threshold[3];
    param[4][4] = max_threshold[4];
    param[4][5] = max_threshold[5];
    param[5][0] = min_threshold[0];
    param[5][1] = min_threshold[1];
    param[5][2] = min_threshold[2];
    param[5][3] = min_threshold[3];
    param[5][4] = min_threshold[4];
    param[5][5] = min_threshold[5];

    if (c.execute("FT_Guard", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  恒力控制
 * @param  [in] flag 0-关闭恒力控制，1-开启恒力控制
 * @param  [in] sensor_id 力传感器编号
 * @param  [in] select  选择六个自由度是否检测碰撞，0-不检测，1-检测
 * @param  [in] ft  碰撞力/扭矩，fx,fy,fz,tx,ty,tz
 * @param  [in] ft_pid 力pid参数，力矩pid参数
 * @param  [in] adj_sign 自适应启停控制，0-关闭，1-开启
 * @param  [in] ILC_sign ILC启停控制， 0-停止，1-训练，2-实操
 * @param  [in] max_dis 最大调整距离，单位mm
 * @param  [in] max_ang 最大调整角度，单位deg
 * @param  [in] filter_Sign 滤波开启标志 0-关；1-开，默认关闭
 * @param  [in] posAdapt_sign 姿态顺应开启标志 0-关；1-开，默认关闭
 * @param  [in] isNoBlock 阻塞标志，0-阻塞；1-非阻塞
 * @return  错误码
 */
errno_t FRRobot::FT_Control(uint8_t flag, int sensor_id, uint8_t select[6], ForceTorque *ft, float ft_pid[6], uint8_t adj_sign, uint8_t ILC_sign, float max_dis, float max_ang, int filter_Sign, int posAdapt_sign, int isNoBlock)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = flag;
    param[1] = sensor_id;
    param[2][0] = select[0];
    param[2][1] = select[1];
    param[2][2] = select[2];
    param[2][3] = select[3];
    param[2][4] = select[4];
    param[2][5] = select[5];
    param[3][0] = ft->fx;
    param[3][1] = ft->fy;
    param[3][2] = ft->fz;
    param[3][3] = ft->tx;
    param[3][4] = ft->ty;
    param[3][5] = ft->tz;
    param[4][0] = ft_pid[0];
    param[4][1] = ft_pid[1];
    param[4][2] = ft_pid[2];
    param[4][3] = ft_pid[3];
    param[4][4] = ft_pid[4];
    param[4][5] = ft_pid[5];
    param[5] = adj_sign;
    param[6] = ILC_sign;
    param[7] = max_dis;
    param[8] = max_ang;
    param[9] = filter_Sign;
    param[10] = posAdapt_sign;
    param[11] = isNoBlock;

    if (c.execute("FT_Control", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  螺旋线探索
 * @param  [in] rcs 参考坐标系，0-工具坐标系，1-基坐标系
 * @param  [in] dr 每圈半径进给量
 * @param  [in] ft 力/扭矩阈值，fx,fy,fz,tx,ty,tz，范围[0~100]
 * @param  [in] max_t_ms 最大探索时间，单位ms
 * @param  [in] max_vel 最大线速度，单位mm/s
 * @return  错误码
 */
errno_t FRRobot::FT_SpiralSearch(int rcs, float dr, float ft, float max_t_ms, float max_vel)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = rcs;
    param[1] = dr;
    param[2] = ft;
    param[3] = max_t_ms;
    param[4] = max_vel;

    if (c.execute("FT_SpiralSearch", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  旋转插入
 * @param  [in] rcs 参考坐标系，0-工具坐标系，1-基坐标系
 * @param  [in] angVelRot 旋转角速度，单位deg/s
 * @param  [in] ft  力/扭矩阈值，fx,fy,fz,tx,ty,tz，范围[0~100]
 * @param  [in] max_angle 最大旋转角度，单位deg
 * @param  [in] orn 力/扭矩方向，1-沿z轴方向，2-绕z轴方向
 * @param  [in] max_angAcc 最大旋转加速度，单位deg/s^2，暂不使用，默认为0
 * @param  [in] rotorn  旋转方向，1-顺时针，2-逆时针
 * @return  错误码
 */
errno_t FRRobot::FT_RotInsertion(int rcs, float angVelRot, float ft, float max_angle, uint8_t orn, float max_angAcc, uint8_t rotorn)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = rcs;
    param[1] = angVelRot;
    param[2] = ft;
    param[3] = max_angle;
    param[4] = orn;
    param[5] = max_angAcc;
    param[6] = rotorn;

    if (c.execute("FT_RotInsertion", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  直线插入
 * @param  [in] rcs 参考坐标系，0-工具坐标系，1-基坐标系
 * @param  [in] ft  力/扭矩阈值，fx,fy,fz,tx,ty,tz，范围[0~100]
 * @param  [in] lin_v 直线速度，单位mm/s
 * @param  [in] lin_a 直线加速度，单位mm/s^2，暂不使用
 * @param  [in] max_dis 最大插入距离，单位mm
 * @param  [in] linorn  插入方向，0-负方向，1-正方向
 * @return  错误码
 */
errno_t FRRobot::FT_LinInsertion(int rcs, float ft, float lin_v, float lin_a, float max_dis, uint8_t linorn)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = rcs;
    param[1] = ft;
    param[2] = lin_v;
    param[3] = lin_a;
    param[4] = max_dis;
    param[5] = linorn;

    if (c.execute("FT_LinInsertion", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  表面定位
 * @param  [in] rcs 参考坐标系，0-工具坐标系，1-基坐标系
 * @param  [in] dir  移动方向，1-正方向，2-负方向
 * @param  [in] axis 移动轴，1-x轴，2-y轴，3-z轴
 * @param  [in] lin_v 探索直线速度，单位mm/s
 * @param  [in] lin_a 探索直线加速度，单位mm/s^2，暂不使用，默认为0
 * @param  [in] max_dis 最大探索距离，单位mm
 * @param  [in] ft  动作终止力/扭矩阈值，fx,fy,fz,tx,ty,tz
 * @return  错误码
 */
errno_t FRRobot::FT_FindSurface(int rcs, uint8_t dir, uint8_t axis, float lin_v, float lin_a, float max_dis, float ft)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = rcs;
    param[1] = dir;
    param[2] = axis;
    param[3] = lin_v;
    param[4] = lin_a;
    param[5] = max_dis;
    param[6] = ft;

    if (c.execute("FT_FindSurface", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  计算中间平面位置开始
 * @return  错误码
 */
errno_t FRRobot::FT_CalCenterStart()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("FT_CalCenterStart", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  计算中间平面位置结束
 * @param  [out] pos 中间平面位姿
 * @return  错误码
 */
errno_t FRRobot::FT_CalCenterEnd(DescPose *pos)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("FT_CalCenterEnd", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            pos->tran.x = double(result[1]);
            pos->tran.y = double(result[2]);
            pos->tran.z = double(result[3]);
            pos->rpy.rx = double(result[4]);
            pos->rpy.ry = double(result[5]);
            pos->rpy.rz = double(result[6]);
        }
        else{
            logger_error("execute FT_CalCenterEnd fail %d", errcode);
        }
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  柔顺控制开启
 * @param  [in] p 位置调节系数或柔顺系数
 * @param  [in] force 柔顺开启力阈值，单位N
 * @return  错误码
 */
errno_t FRRobot::FT_ComplianceStart(float p, float force)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = p;
    param[1] = force;

    if (c.execute("FT_ComplianceStart", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  柔顺控制关闭
 * @return  错误码
 */
errno_t FRRobot::FT_ComplianceStop()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("FT_ComplianceStop", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 负载辨识初始化
 * @return 错误码
 */
errno_t FRRobot::LoadIdentifyDynFilterInit()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("LoadIdentifyDynFilterInit", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 负载辨识初始化
 * @return 错误码
 */
errno_t FRRobot::LoadIdentifyDynVarInit()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("LoadIdentifyDynVarInit", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 负载辨识主程序
 * @param [in] joint_torque 关节扭矩
 * @param [in] joint_pos 关节位置
 * @param [in] t 采样周期
 * @return 错误码
 */
errno_t FRRobot::LoadIdentifyMain(double joint_torque[6], double joint_pos[6], double t)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;
    int i;

    for (i = 0; i < 6; i++)
    {
        param[0][i] = joint_torque[i];
        param[1][i] = joint_pos[i];
    }

    param[2] = t;

    if (c.execute("LoadIdentifyMain", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 获取负载辨识结果
 * @param [in] gain
 * @param [out] weight 负载重量
 * @param [out] cog 负载质心
 * @return 错误码
 */
errno_t FRRobot::LoadIdentifyGetResult(double gain[12], double *weight, DescTran *cog)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;
    int i;

    for (i = 0; i < 12; i++)
    {
        param[0][i] = gain[i];
    }

    if (c.execute("LoadIdentifyGetResult", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            *weight = double(result[1]);
            cog->x = double(result[2]);
            cog->y = double(result[3]);
            cog->z = double(result[4]);
        }
        else{
            logger_error("execute LoadIdentifyGetResult fail %d", errcode);
        }
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 传动带启动、停止
 * @param [in] status 状态，1-启动，0-停止
 * @return 错误码
 */
errno_t FRRobot::ConveyorStartEnd(uint8_t status)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = status;

    if (c.execute("ConveyorStartEnd", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 记录IO检测点
 * @return 错误码
 */
errno_t FRRobot::ConveyorPointIORecord()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ConveyorPointIORecord", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 记录A点
 * @return 错误码
 */
errno_t FRRobot::ConveyorPointARecord()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ConveyorPointARecord", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 记录参考点
 * @return 错误码
 */
errno_t FRRobot::ConveyorRefPointRecord()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ConveyorRefPointRecord", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 记录B点
 * @return 错误码
 */
errno_t FRRobot::ConveyorPointBRecord()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ConveyorPointBRecord", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 传送带工件IO检测
 * @param [in] max_t 最大检测时间，单位ms
 * @return 错误码
 */
errno_t FRRobot::ConveyorIODetect(int max_t)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = max_t;

    if (c.execute("ConveyorIODetect", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 获取物体当前位置
 * @param [in] mode
 * @return 错误码
 */
errno_t FRRobot::ConveyorGetTrackData(int mode)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = mode;

    if (c.execute("ConveyorGetTrackData", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 传动带跟踪开始
 * @param [in] status 状态，1-启动，0-停止
 * @return 错误码
 */
errno_t FRRobot::ConveyorTrackStart(uint8_t status)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = status;

    if (c.execute("ConveyorTrackStart", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 传动带跟踪停止
 * @return 错误码
 */
errno_t FRRobot::ConveyorTrackEnd()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ConveyorTrackEnd", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 传动带参数配置
 * @param [in] para[0] 编码器通道 1~2
 * @param [in] para[1] 编码器转一圈的脉冲数
 * @param [in] para[2] 编码器转一圈传送带行走距离
 * @param [in] para[3] 工件坐标系编号 针对跟踪运动功能选择工件坐标系编号，跟踪抓取、TPD跟踪设为0
 * @param [in] para[4] 是否配视觉  0 不配  1 配
 * @param [in] para[5] 速度比  针对传送带跟踪抓取选项（1-100）  其他选项默认为1 
 * @param [in] followType 跟踪运动类型，0-跟踪运动；1-追检运动
 * @param [in] startDis 追检抓取需要设置， 跟踪起始距离， -1：自动计算(工件到达机器人下方后自动追检)，单位mm， 默认值0
 * @param [in] endDis 追检抓取需要设置，跟踪终止距离， 单位mm， 默认值100
 * @return 错误码
 */
errno_t FRRobot::ConveyorSetParam(float para[6], int followType, int startDis, int endDis)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;
    int i;

    for (i = 0; i < 6; i++)
    {
        param[0][i] = para[i];
    }
    param[1] = followType;
    param[2] = startDis;
    param[3] = endDis;

    if (c.execute("ConveyorSetParam", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 传动带抓取点补偿
 * @param [in] cmp 补偿位置
 * @return 错误码
 */
errno_t FRRobot::ConveyorCatchPointComp(double cmp[3])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;
    int i;

    for (i = 0; i < 3; i++)
    {
        param[0][i] = cmp[i];
    }

    if (c.execute("ConveyorCatchPointComp", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 直线运动
 * @param [in] status 状态，1-启动，0-停止
 * @return 错误码
 */
errno_t FRRobot::TrackMoveL(char name[32], int tool, int wobj, float vel, float acc, float ovl, float blendR, uint8_t flag, uint8_t type)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = name;
    param[1] = tool;
    param[2] = wobj;
    param[3] = vel;
    param[4] = acc;
    param[5] = ovl;
    param[6] = blendR;
    param[7] = flag;
    param[8] = type;

    if (c.execute("ConveyorTrackMoveL", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 获取SSH公钥
 * @param [out] keygen 公钥
 * @return 错误码
 */
errno_t FRRobot::GetSSHKeygen(char keygen[1024])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetSSHKeygen", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            std::string s(result[1]);
            strcpy(keygen, s.c_str());
        }
        else
        {   
            const char *res = "";
            strcpy(keygen, res);
            logger_error("execute GetSSHKeygen fail %d", errcode);
        }
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 下发SCP指令
 * @param [in] mode 0-上传（上位机->控制器），1-下载（控制器->上位机）
 * @param [in] sshname 上位机用户名
 * @param [in] sship 上位机ip地址
 * @param [in] usr_file_url 上位机文件路径
 * @param [in] robot_file_url 机器人控制器文件路径
 * @return 错误码
 */
errno_t FRRobot::SetSSHScpCmd(int mode, char sshname[32], char sship[32], char usr_file_url[128], char robot_file_url[128])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = mode;
    param[1] = sshname;
    param[2] = sship;
    param[3] = usr_file_url;
    param[4] = robot_file_url;

    if (c.execute("SetSSHScpCmd", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 计算指定路径下文件的MD5值
 * @param [in] file_path 文件路径包含文件名，默认Traj文件夹路径为:"/fruser/traj/",如"/fruser/traj/trajHelix_aima_1.txt"
 * @param [out] md5 文件MD5值
 * @return 错误码
 */
errno_t FRRobot::ComputeFileMD5(char file_path[256], char md5[256])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = file_path;

    if (c.execute("ComputeFileMD5", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            std::string s(result[1]);
            strcpy(md5, s.c_str());
        }
        else
        {
            const char *res = "";
            strcpy(md5, res);
            logger_error("execute ComputeFileMD5 fail %d", errcode);
        }
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 获取机器人急停状态
 * @param [out] state 急停状态，0-非急停，1-急停
 * @return 错误码
 */
errno_t FRRobot::GetRobotEmergencyStopState(uint8_t *state)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *state = robot_state_pkg->EmergencyStop;
        logger_info("emergency = %u.", robot_state_pkg->EmergencyStop);
    }
    else
    {
        errcode = g_sock_com_err;
    }

    return errcode;
}

/**
 * @brief 获取SDK与机器人的通讯状态
 * @param [out]  state 通讯状态，0-通讯正常，1-通讯异常
 */
errno_t FRRobot::GetSDKComState(int *state)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *state = 0;
    }
    else if (g_sock_com_err == ERR_SOCKET_COM_FAILED)
    {
        *state = 1;
    }

    int tmp_state = *state;

    logger_info("com state = %d\n", tmp_state);

    return errcode;
}

/**
 * @brief 获取安全停止信号
 * @param [out]  si0_state 安全停止信号SI0
 * @param [out]  si1_state 安全停止信号SI1
 */
errno_t FRRobot::GetSafetyStopState(uint8_t *si0_state, uint8_t *si1_state)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    if (g_sock_com_err == ERR_SUCCESS)
    {
        *si0_state = robot_state_pkg->safety_stop0_state;
        *si1_state = robot_state_pkg->safety_stop1_state;
    }
    else
    {
        errcode = g_sock_com_err;
    }

    logger_info("state com: %d", errcode);
    logger_info("return value is: %u - %u", robot_state_pkg->safety_stop0_state, robot_state_pkg->safety_stop1_state);
    return errcode;
}
/**
 * @brief 获取机器人软件版本
 * @param[out]	robotModel 机器人型号
 * @param[out]	webversion web版本
 * @param[out]	controllerVersion 控制器版本
 * @return 错误码
 */
errno_t FRRobot::GetSoftwareVersion(char robotModel[64], char webVersion[64], char controllerVersion[64])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetSoftwareVersion", param, result))
    {
        errcode = int(result[0]);
        if (0 == errcode)
        {
            std::string s1(result[1]);
            std::string s2(result[2]);
            std::string s3(result[3]);
            strcpy(robotModel, s1.c_str());
            strcpy(webVersion, s2.c_str());
            strcpy(controllerVersion, s3.c_str());
        }
        else
        {
            logger_error("GetSoftwareVersion fail, errcode is: %d\n", errcode);
        }
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 获取机器人硬件版本
 * @param[out] ctrlBoxBoardversion 控制箱载板硬件版本
 * @param[out] driver1version 驱动器1硬件版本
 * @param[out] driver2version 驱动器2硬件版本
 * @param[out] driver3version 驱动器3硬件版本
 * @param[out] driver4version 驱动器4硬件版本
 * @param[out] driver5version 驱动器5硬件版本
 * @param[out] driver6version 驱动器6硬件版本
 * @param[out] endBoardversion 未端版硬件版本
 */
errno_t FRRobot::GetHardwareVersion(char ctrlBoxBoardversion[128], char driver1version[128], char driver2version[128],
                                    char driver3version[128], char driver4version[128], char driver5version[128],
                                    char driver6version[128], char endBoardversion[128])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetSlaveHardVersion", param, result))
    {
        errcode = int(result[0]);
        if (0 == errcode)
        {
            std::string s1(result[1]);
            std::string s2(result[2]);
            std::string s3(result[3]);
            std::string s4(result[4]);
            std::string s5(result[5]);
            std::string s6(result[6]);
            std::string s7(result[7]);
            std::string s8(result[8]);

            strcpy(ctrlBoxBoardversion, s1.c_str());
            strcpy(driver1version, s2.c_str());
            strcpy(driver2version, s3.c_str());
            strcpy(driver3version, s4.c_str());
            strcpy(driver4version, s5.c_str());
            strcpy(driver5version, s6.c_str());
            strcpy(driver6version, s7.c_str());
            strcpy(endBoardversion, s8.c_str());
        }
        else
        {
            logger_error("GetHardwareVersion fail, errcode is: %d\n", errcode);
        }
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 获取机器人固件版本
 * @param[out] ctrlBoxBoardversion 控制箱载板固件版本
 * @param[out] driver1version 驱动器1固件版本
 * @param[out] driver2version 驱动器2固件版本
 * @param[out] driver3version 驱动器3固件版本
 * @param[out] driver4version 驱动器4固件版本
 * @param[out] driver5version 驱动器5固件版本
 * @param[out] driver6version 驱动器6固件版本
 * @param[out] endBoardversion 未端版固件版本
 */
errno_t FRRobot::GetFirmwareVersion(char ctrlBoxBoardversion[128], char driver1version[128], char driver2version[128],
                                    char driver3version[128], char driver4version[128], char driver5version[128],
                                    char driver6version[128], char endBoardversion[128])

{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetSlaveFirmVersion", param, result))
    {
        errcode = int(result[0]);
        if (0 == errcode)
        {
            std::string s1(result[1]);
            std::string s2(result[2]);
            std::string s3(result[3]);
            std::string s4(result[4]);
            std::string s5(result[5]);
            std::string s6(result[6]);
            std::string s7(result[7]);
            std::string s8(result[8]);

            strcpy(ctrlBoxBoardversion, s1.c_str());
            strcpy(driver1version, s2.c_str());
            strcpy(driver2version, s3.c_str());
            strcpy(driver3version, s4.c_str());
            strcpy(driver4version, s5.c_str());
            strcpy(driver5version, s6.c_str());
            strcpy(driver6version, s7.c_str());
            strcpy(endBoardversion, s8.c_str());
        }
        else
        {
            logger_error("GetSlaveFirmVersion fail, errcode is: %d\n", errcode);
        }
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}
/**
 * @brief 获取机器人DH参数补偿值
 * @param [out] dhCompensation 机器人DH参数补偿值(mm) [cmpstD1,cmpstA2,cmpstA3,cmpstD4,cmpstD5,cmpstD6]
 * @return 错误码
 */
errno_t FRRobot::GetDHCompensation(double dhCompensation[6])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetDHCompensation", param, result))
    {
        int retval = int(result[0]);
        if (0 == retval)
        {
            for (int i = 0; i < 6; i++)
            {
                dhCompensation[i] = double(result[i + 1]);
            }
        }
        else
        {
            logger_error("execute GetDHCompensation fail, retval is: %d", retval);
        }
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 点位表切换
 * @param [in] pointTableName 要切换的点位表名称    pointTable1.db
 * @return 错误码
 */
errno_t FRRobot::PointTableSwitch(const std::string pointTableName)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = pointTableName;
    if (c.execute("PointTableSwitch", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute PointTableSwitch fail: %d", errcode);
        }
        return errcode;
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
        return errcode;
    }
}

/**
 * @brief 下载点位表数据库
 * @param [in] pointTableName 要下载的点位表名称    pointTable1.db
 * @param [in] saveFilePath 下载点位表的存储路径   C://test/
 * @return 错误码
 */
errno_t FRRobot::PointTableDownLoad(const std::string &pointTableName, const std::string &saveFilePath)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    /* 入参检查 */
    if((pointTableName.size() == 0) || (saveFilePath.size() == 0))
    {
        logger_error("point table name and file path can not be empty.");
        return 4;
    }
// 检查文件路径;
#ifdef WIN32
    std::wstring save_path_wide(saveFilePath.begin(), saveFilePath.end());
    if (GetFileAttributesA(saveFilePath.c_str()) == INVALID_FILE_ATTRIBUTES)
    {
        logger_error("path %s do not exist.", saveFilePath.c_str());
        c.close();
        return ERR_SAVE_FILE_PATH_NOT_FOUND;
    }
#else
    char save_path[MAX_POINT_TABLE_PATH_LENGTH];
    memset(save_path, 0, MAX_POINT_TABLE_PATH_LENGTH);
    snprintf(save_path, MAX_POINT_TABLE_PATH_LENGTH, "%s", saveFilePath.c_str());
    if(access(save_path, F_OK) != 0)
    {
        logger_error("path %s do not exist.", save_path);
        c.close();
        return ERR_SAVE_FILE_PATH_NOT_FOUND;
    }
#endif

    logger_info("path %s do exist.", saveFilePath.c_str());

    // 发起远程调用;
    param = pointTableName;
    if (c.execute("PointTableDownload", param, result))
    {
        errcode = int(result);
        if (-1 == errcode)
        {
            logger_error("do not find point table. %d ", ERR_POINTTABLE_NOTFOUND);
            c.close();
            return ERR_POINTTABLE_NOTFOUND;
        }
        if (0 != errcode)
        {
            logger_error("execute PointTableDownload fail, %d " , errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    Sleep(2000);
    // 发起网络连接，客户端;
    fr_network::socket_fd fd = fr_network::get_socket_fd();

#ifdef WIN32
    int syncnt = 10;
    if (setsockopt(fd, IPPROTO_TCP, TCP_MAXRT, (char*)&syncnt, sizeof(syncnt)) == SOCKET_ERROR)
    {
        logger_error("Failed to set TCP_MAXRT option.\n");
        c.close();
        fr_network::close_fd(fd);
        return ERR_OTHER;
}
    int timeout = 5000;
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout, sizeof(int));
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(int));
#else
    int syncnt = 3;
    setsockopt(fd, IPPROTO_TCP, TCP_SYNCNT, &syncnt, sizeof(syncnt));
    struct timeval tv;
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (const char *)&tv, sizeof tv);
#endif

    errcode = fr_network::connect(fd, robot_ip, DOWNLOAD_POINT_TABLE_PORT);
    logger_info("connect error code is %d ", errcode);
    if ((errcode < 0))
    {
        logger_error("connect fail, %s.", strerror(errno));
        fr_network::close_fd(fd);
        c.close();
        return ERR_SOCKET_COM_FAILED;
    }
    Sleep(2000);

    // 超时时间10s；内容上限2M;
    char recv_buf[1024 * 8] = {0};
    string total_buf; // 头+长度+md5+点位表+尾;
    total_buf.reserve(POINT_TABLE_MAX_SIZE);

    string point_table; // 点位表;
    point_table.reserve(POINT_TABLE_MAX_SIZE);

    string except_md5;
    except_md5.reserve(40);
    string compute_md5;
    compute_md5.reserve(40);
    string file_path;

    int recv_bytes = 0;
    int total_bytes = 0; // 总共收到的字节数;
    int except_bytes = 0;
    bool find_head = false;
    bool find_size_md5 = false;
    bool download_success_finish = false;

    errcode = 0;
    do
    {
        double last_time = 0;
        double cur_time = 0;
        auto now_ns = std::chrono::system_clock::now().time_since_epoch();
        auto now_ms = chrono::duration<double, std::milli>(now_ns);
        cur_time = now_ms.count();
        last_time = cur_time;

        // 进入文件接收循环;
        while (1)
        {
            auto now_ns = std::chrono::system_clock::now().time_since_epoch();
            auto now_ms = chrono::duration<double, std::milli>(now_ns);
            cur_time = now_ms.count();
            if (cur_time - last_time > 10000)
            {
                logger_error("download over time 10s");
                logger_error("total bytes %d, %s.", total_bytes, total_buf.c_str());
                download_success_finish = false;
                break;
            }

            recv_bytes = 0;
            recv_bytes = recv(fd, (char *)&recv_buf, sizeof(recv_buf), 0);
            if (recv_bytes < 0)
            {
                logger_error(" recv bytes is: %d, %s.", recv_bytes, strerror(errno));
                download_success_finish = false;
                break;
                ;
            }

            total_buf.append(recv_buf, recv_bytes);
            total_bytes += recv_bytes;
            recv_bytes = 0;
            logger_debug("size is %d, buf is: %s.", total_bytes, total_buf.c_str());

            // 找头;
            if ((!find_head) && (total_bytes > 4))
            {
                size_t res = total_buf.find(POINT_TABLE_HEAD);
                if ((string::npos == res))
                {
                    logger_error("do not find point table head in: %s", total_buf.c_str());
                }
                if (0 != res)
                {
                    logger_error("point table head is wrong: %s.", total_buf.c_str());
                }
                else
                {
                    find_head = true;
                }
            }

            // 找md5值 和 数据长度;
            if (find_head && (!find_size_md5) && (total_bytes > 12 + 32))
            {
                string str_tmp = total_buf.substr(4, 8);
                except_bytes = stoi(str_tmp);

                except_md5 = total_buf.substr(12, 32);
                find_size_md5 = true;

                logger_info("except bytes is: %d.", except_bytes);
            }

            // 检查完整性;
            if (find_head && find_size_md5 && (except_bytes == total_bytes))
            {
                logger_info("have recv all bytes: %d.", total_bytes);
                download_success_finish = true;
                break;
                ;
            }
        }
        // 检查下载，提取点位表;
        if (download_success_finish == true)
        {
            point_table = total_buf.substr(12 + 32, total_bytes - 12 - 32 - 4);
            errcode = ERR_SUCCESS;
        }
        else
        {
            logger_error("download fail.");
            errcode = ERR_DOWN_LOAD_FILE_FAILED;
            break;
            ;
        }

        compute_md5 = fr_md5::md5_hash_hex(point_table);
        if (0 == except_md5.compare(compute_md5))
        {
            logger_info("md5 is correct.");
            errcode = ERR_SUCCESS;
        }
        else
        {
            logger_error("except is: %s, compute is: %s.", except_md5.c_str(), compute_md5.c_str());
            errcode = ERR_DOWN_LOAD_FILE_CHECK_FAILED;
            break;
        }

        // file_path = saveFilePath + pointTableName;
        /*注意路径末尾是否为字符/， 这点影响拼接结果*/
        if('/' == saveFilePath.back())
        {
            file_path = saveFilePath + pointTableName;
        }else{
            file_path = saveFilePath + '/' + pointTableName;
        }
        ofstream input_file(file_path, ios::binary);
        if (input_file.is_open())
        {
            input_file << point_table;
            input_file.flush();
            input_file.close();
            errcode = ERR_SUCCESS;
        }
        else
        {
            logger_error("open file fail.");
            errcode = ERR_DOWN_LOAD_FILE_WRITE_FAILED;
            input_file.close();
            break;
            ;
        }
    } while (0);

    // 给服务器返回文件传输结果;
    int send_bytes = 0;
    char _success[] = "SUCCESS";
    char _fail[] = "FAIL";
    if (errcode == ERR_SUCCESS)
    {
        send_bytes = send(fd, _success, sizeof("SUCCESS"), 0);
        if (send_bytes < 0)
        {
            logger_error("send success faild %d.", send_bytes);
        }
    }
    else
    {
        send_bytes = send(fd, _fail, sizeof("FAIL"), 0);
        if (send_bytes < 0)
        {
            logger_error("send fail faild %d.", send_bytes);
        }
    }

    // 释放资源，函数返回;
    c.close();
    fr_network::close_fd(fd);
    return errcode;
}

/**
 * @brief 上传点位表数据库
 * @param [in] pointTableFilePath 上传点位表的全路径名   C://test/pointTable1.db
 * @return 错误码
 */
errno_t FRRobot::PointTableUpLoad(const std::string &pointTableFilePath)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (0 == pointTableFilePath.size())
    {
        logger_error("point table name and file path can not be empty.");
        return 4;
    }
// 检查文件路径;
#ifdef WIN32
    std::wstring save_path_wide(pointTableFilePath.begin(), pointTableFilePath.end());
    if (GetFileAttributesA(pointTableFilePath.c_str()) == INVALID_FILE_ATTRIBUTES)
    {
        logger_error("path %s do not exist.", pointTableFilePath.c_str());
        c.close();
        return ERR_SAVE_FILE_PATH_NOT_FOUND;
    }
#else
    char save_path[MAX_POINT_TABLE_PATH_LENGTH];
    memset(save_path, 0, MAX_POINT_TABLE_PATH_LENGTH);
    snprintf(save_path, MAX_POINT_TABLE_PATH_LENGTH, "%s", pointTableFilePath.c_str());
    if(access(save_path, F_OK) != 0)
    {
        logger_error("path %s do not exist.", save_path);
        c.close();
        return ERR_SAVE_FILE_PATH_NOT_FOUND;
    }
#endif

    logger_info("path %s do exist.", pointTableFilePath.c_str());

    long file_size = 0;
// 检查文件大小;
#ifdef WIN32
    HANDLE hFile = CreateFile(pointTableFilePath.c_str(), GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (INVALID_HANDLE_VALUE != hFile)
    {
        DWORD dwHigh = 0;
        DWORD dwSize = ::GetFileSize(hFile, &dwHigh);
        CloseHandle(hFile);
        file_size = dwSize;
        if (file_size < 0)
        {
            logger_error("get file size fail, size is: %l.", file_size);
            c.close();
            return ERR_OTHER;
}
    }
    else {
        logger_error("open file fail, error is: %lu.", GetLastError());
        c.close();
        return ERR_OTHER;
    }
#else
    FILE *file_p = fopen(save_path, "rb");
    if(NULL == file_p)
    {
        logger_error("open file %s fail.", save_path);
        c.close();
        return ERR_OTHER;
    }
    fseek(file_p, 0, SEEK_END);
    file_size = ftell(file_p);
    fclose(file_p);
#endif

    long total_size = file_size + 16 + 32;
    logger_info("file size is: %l, total size is: %l.", file_size, total_size);
    if (total_size > POINT_TABLE_MAX_SIZE)
    {
        logger_error("file size have over max limit: %d", POINT_TABLE_MAX_SIZE);
        c.close();
        return ERR_FILE_TOO_LARGE;
    }

    // 点位表名称;
    string point_table_name;

#ifdef WIN32
    point_table_name = PathFindFileNameA(pointTableFilePath.c_str());
#else
    // point_table_name = save_path.filename();
    char path_dup[MAX_POINT_TABLE_PATH_LENGTH];
    memset(path_dup, 0, MAX_POINT_TABLE_PATH_LENGTH);
    memcpy(path_dup, save_path, MAX_POINT_TABLE_PATH_LENGTH-1);
    point_table_name.append(basename(path_dup));
    logger_info("point table name is: %s.", point_table_name.c_str());
#endif

    // 发起远程调用;
    param = point_table_name;
    if (c.execute("PointTableUpload", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute PointTableUpload fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }
    Sleep(2000);

    // 发起网络连接;
    fr_network::socket_fd fd = fr_network::get_socket_fd();

    int syncnt = 3;
#ifdef WIN32
    if (setsockopt(fd, IPPROTO_TCP, TCP_MAXRT, (char*)&syncnt, sizeof(syncnt)) == SOCKET_ERROR)
    {
        logger_error("Failed to set TCP_MAXRT option.\n");
        c.close();
        fr_network::close_fd(fd);
        return ERR_OTHER;
    }
    int timeout = 5000;
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout, sizeof(int));
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(int));
#else
    setsockopt(fd, IPPROTO_TCP, TCP_SYNCNT, &syncnt, sizeof(syncnt));
    struct timeval tv;
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (const char *)&tv, sizeof tv);
#endif

    errcode = fr_network::connect(fd, robot_ip, UPLOAD_POINT_TABLE_PORT);
    logger_info("connect error code is: %d.", errcode);
    if (errcode < 0)
    {
        logger_error("connect fail.");
        fr_network::close_fd(fd);
        c.close();
        return ERR_SOCKET_COM_FAILED;
    }
    Sleep(2000);

    // 发送 send_buf_first, send_buf, send_buf_last;
    do
    {
        string compute_md5;
        compute_md5.reserve(40);
        string send_buf; // 点位表内容;
        send_buf.reserve(POINT_TABLE_MAX_SIZE);
        // 读取文件，计算md5;
        ifstream input_file(pointTableFilePath, std::ios::binary);
        if (!input_file.is_open())
        {
            logger_error("point table file path is not open.");
            errcode = ERR_FILE_OPEN_FAILED;
            break;
        }
        // 获取文件大小;
        stringstream buffer;
        buffer << input_file.rdbuf();
        send_buf = buffer.str();
        input_file.close();

        compute_md5 = fr_md5::md5_hash_hex(send_buf);
        logger_info("send buf size is: %d. ", send_buf.length());

        // 发送头+size+md5;
        std::ostringstream headerStream;
        headerStream << POINT_TABLE_HEAD << std::setw(8) << std::setfill('0') << total_size << compute_md5;
        std::string send_buf_first = headerStream.str();
        int send_bytes = send(fd, const_cast<char *>(send_buf_first.c_str()), send_buf_first.length(), 0);
        if ((send_bytes < 0) || (static_cast<unsigned int>(send_bytes) != send_buf_first.length()))
        {
            logger_error("send head %s fail.", send_buf_first.c_str());
            errcode = ERR_SOCKET_SEND_FAILED;
            break;
        }
        logger_info("send head end");

        // 发送点位表;
        send_bytes = 0;
        unsigned int total_send_bytes = 0;
        bool send_point_table_success = true;
        while (total_send_bytes < send_buf.length())
        {
            send_bytes = 0;
            send_bytes = send(fd, const_cast<char *>(send_buf.c_str() + total_send_bytes), send_buf.length() - total_send_bytes, 0);
            if (send_bytes < 0)
            {
                logger_error("has send bytes %d", total_send_bytes);
                logger_error("send buf is: %s.", send_buf.c_str()+total_send_bytes);
                logger_error("send bytes is %d", send_buf.length() - total_send_bytes);
                errcode = ERR_SOCKET_SEND_FAILED;
                send_point_table_success = false;
                break;
            }
            total_send_bytes += send_bytes;
        }
        if (false == send_point_table_success)
        {
            logger_error("send point table fail \n");
            break;
        }
        logger_info("except %d, total send bytes: %d", send_buf.length(), total_send_bytes);

        // 发送文件尾;
        send_bytes = 0;
        string send_buf_last = POINT_TABLE_TAIL;
        send_bytes = send(fd, const_cast<char *>(send_buf_last.c_str()), send_buf_last.length(), 0);
        if ((send_bytes < 0) || (static_cast<unsigned int>(send_bytes) != send_buf_last.length()))
        {
            logger_error("send last buf : %s fail.", send_buf_last.c_str());
            errcode = ERR_SOCKET_SEND_FAILED;
            break;
        }
        logger_info("send buf tail end");

        // 接收、校验客户端返回;
        int recv_bytes = 0;
        char recv_buf[1024];
        memset(recv_buf, 0, sizeof(recv_buf));
        recv_bytes = recv(fd, recv_buf, sizeof(recv_buf), 0);
        if (recv_bytes < 0)
        {
            logger_error("recv fail");
            errcode = ERR_SOCKET_RECV_FAILED;
            break;
            ;
        }
        logger_info("recv %d, %s", recv_bytes, recv_buf);
        string recv_str(recv_buf);
        if (0 != recv_str.compare("SUCCESS"))
        {
            logger_error("recv %s fail. upload file fail", recv_str.c_str());
            errcode = ERR_OTHER;
            break;
            ;
        }

        errcode = ERR_SUCCESS;

    } while (0);

    // 释放资源，函数返回;
    c.close();
    fr_network::close_fd(fd);
    return errcode;
}
/**
 * @brief 点位表更新lua文件
 * @param [in] pointTableName 要切换的点位表名称   "pointTable1.db",当点位表为空，即""时，表示将lua程序更新为未应用点位表的初始程序
 * @param [in] luaFileName 要更新的lua文件名称   "testPointTable.lua"
 * @param [out] errorStr 切换点位表错误信息
 * @return 错误码
 */
errno_t FRRobot::PointTableUpdateLua(const std::string &pointTableName, const std::string &luaFileName)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    do
    {
        // 先切换;
        param = pointTableName;
        if (c.execute("PointTableSwitch", param, result))
        {
            errcode = int(result);
            if (0 != errcode)
            {
                logger_error("execute PointTableSwitch fail: %d", errcode);
                break;
                ;
            }
        }
        else
        {
            errcode = ERR_XMLRPC_CMD_FAILED;
            break;
        }

        Sleep(2000);
        // 再更新;
        param.clear();
        result.clear();
        param = luaFileName;
        if (c.execute("PointTableUpdateLua", param, result))
        {
            errcode = int(result[0]);
            if (0 != errcode)
            {
                string str(result[1]);
                logger_error("execute PointTableUpdateLua fail, errcode: %d, %s.", errcode, str.c_str());
                break;
                ;
            }
        }
        else
        {
            errcode = ERR_XMLRPC_CMD_FAILED;
            break;
            ;
        }

    } while (0);

    c.close();
    return errcode;
}

/**
 * @brief 焊接开始
 * @param [in] ioType io类型 0-控制器IO； 1-扩展IO
 * @param [in] arcNum 焊机配置文件编号
 * @param [in] timeout 起弧超时时间
 * @return 错误码
 */
errno_t FRRobot::ARCStart(int ioType, int arcNum, int timeout)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = ioType;
    param[1] = arcNum;
    param[2] = timeout;

    if (c.execute("ARCStart", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 焊接结束
 * @param [in] ioType io类型 0-控制器IO； 1-扩展IO
 * @param [in] arcNum 焊机配置文件编号
 * @param [in] timeout 熄弧超时时间
 * @return 错误码
 */
errno_t FRRobot::ARCEnd(int ioType, int arcNum, int timeout)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = ioType;
    param[1] = arcNum;
    param[2] = timeout;

    if (c.execute("ARCEnd", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置焊接电流与输出模拟量对应关系
 * @param [in] currentMin 焊接电流-模拟量输出线性关系左侧点电流值(A)
 * @param [in] currentMax 焊接电流-模拟量输出线性关系右侧点电流值(A)
 * @param [in] outputVoltageMin 焊接电流-模拟量输出线性关系左侧点模拟量输出电压值(V)
 * @param [in] outputVoltageMax 焊接电流-模拟量输出线性关系右侧点模拟量输出电压值(V)
 * @param [in] AOIndex 焊接电流模拟量输出端口
 * @return 错误码
 */
errno_t FRRobot::WeldingSetCurrentRelation(double currentMin, double currentMax, double outputVoltageMin, double outputVoltageMax, int AOIndex)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = currentMin;
    param[1] = currentMax;
    param[2] = outputVoltageMin;
    param[3] = outputVoltageMax;
    param[4] = AOIndex;

    if (c.execute("WeldingSetCurrentRelation", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}
/**
 * @brief 设置焊接电压与输出模拟量对应关系
 * @param [in] weldVoltageMin 焊接电压-模拟量输出线性关系左侧点焊接电压值(A)
 * @param [in] weldVoltageMax 焊接电压-模拟量输出线性关系右侧点焊接电压值(A)
 * @param [in] outputVoltageMin 焊接电压-模拟量输出线性关系左侧点模拟量输出电压值(V)
 * @param [in] outputVoltageMax 焊接电压-模拟量输出线性关系右侧点模拟量输出电压值(V)
 * @param [in] AOIndex 焊接电压模拟量输出端口
 * @return 错误码
 */
errno_t FRRobot::WeldingSetVoltageRelation(double weldVoltageMin, double weldVoltageMax, double outputVoltageMin, double outputVoltageMax, int AOIndex)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = weldVoltageMin;
    param[1] = weldVoltageMax;
    param[2] = outputVoltageMin;
    param[3] = outputVoltageMax;
    param[4] = AOIndex;

    if (c.execute("WeldingSetVoltageRelation", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}
/**
 * @brief 获取焊接电流与输出模拟量对应关系
 * @param [out] currentMin 焊接电流-模拟量输出线性关系左侧点电流值(A)
 * @param [out] currentMax 焊接电流-模拟量输出线性关系右侧点电流值(A)
 * @param [out] outputVoltageMin 焊接电流-模拟量输出线性关系左侧点模拟量输出电压值(V)
 * @param [out] outputVoltageMax 焊接电流-模拟量输出线性关系右侧点模拟量输出电压值(V)
 * @param [out] AOIndex 焊接电流模拟量输出端口
 * @return 错误码
 */
errno_t FRRobot::WeldingGetCurrentRelation(double *currentMin, double *currentMax, double *outputVoltageMin, double *outputVoltageMax, int* AOIndex)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("WeldingGetCurrentRelation", param, result))
    {
        errcode = int(result[0]);
        if (0 == errcode)
        {
            *currentMin = result[1];
            *currentMax = result[2];
            *outputVoltageMin = result[3];
            *outputVoltageMax = result[4];
            *AOIndex = result[5];
        }
        else
        {
            logger_error("execute WeldingGetCurrentRelation fail %d ", errcode);
        }
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}
/**
 * @brief 获取焊接电压与输出模拟量对应关系
 * @param [out] weldVoltageMin 焊接电压-模拟量输出线性关系左侧点焊接电压值(A)
 * @param [out] weldVoltageMax 焊接电压-模拟量输出线性关系右侧点焊接电压值(A)
 * @param [out] outputVoltageMin 焊接电压-模拟量输出线性关系左侧点模拟量输出电压值(V)
 * @param [out] outputVoltageMax 焊接电压-模拟量输出线性关系右侧点模拟量输出电压值(V)
 * @param [out] AOIndex 焊接电压模拟量输出端口
 * @return 错误码
 */
errno_t FRRobot::WeldingGetVoltageRelation(double *weldVoltageMin, double *weldVoltageMax, double *outputVoltageMin, double *outputVoltageMax, int* AOIndex)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("WeldingGetVoltageRelation", param, result))
    {
        errcode = int(result[0]);
        if (0 == errcode)
        {
            *weldVoltageMin = result[1];
            *weldVoltageMax = result[2];
            *outputVoltageMin = result[3];
            *outputVoltageMax = result[4];
            *AOIndex = result[5];
        }
        else
        {
            logger_error("execute WeldingGetVoltageRelation fail %d ", errcode);
        }
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置焊接电流
 * @param [in] ioType 控制IO类型 0-控制箱IO；1-扩展IO
 * @param [in] current 焊接电流值(A)
 * @param [in] AOIndex 焊接电流控制箱模拟量输出端口(0-1)
 * @param [in] blend 是否平滑 0-不平滑；1-平滑
 * @return 错误码
 */
errno_t FRRobot::WeldingSetCurrent(int ioType, double current, int AOIndex, int blend)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = ioType;
    param[1] = current;
    param[2] = AOIndex;
    param[3] = blend;

    if (c.execute("WeldingSetCurrent", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置焊接电压
 * @param [in] ioType 控制IO类型 0-控制箱IO；1-扩展IO
 * @param [in] voltage 焊接电压值(A)
 * @param [in] AOIndex 焊接电压控制箱模拟量输出端口(0-1)
 * @param [in] blend 是否平滑 0-不平滑；1-平滑
 * @return 错误码
 */
errno_t FRRobot::WeldingSetVoltage(int ioType, double voltage, int AOIndex, int blend)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = ioType;
    param[1] = voltage;
    param[2] = AOIndex;
    param[3] = blend;

    if (c.execute("WeldingSetVoltage", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置摆动参数
 * @param [in] weaveNum 摆焊参数配置编号
 * @param [in] weaveType 摆动类型 0-平面三角波摆动；1-垂直L型三角波摆动；2-顺时针圆形摆动；3-逆时针圆形摆动；4-平面正弦波摆动；5-垂直L型正弦波摆动；6-垂直三角波摆动；7-垂直正弦波摆动
 * @param [in] weaveFrequency 摆动频率(Hz)
 * @param [in] weaveIncStayTime 等待模式 0-周期不包含等待时间；1-周期包含等待时间
 * @param [in] weaveRange 摆动幅度(mm)
 * @param [in] weaveLeftStayTime 摆动左停留时间(ms)
 * @param [in] weaveRightStayTime 摆动右停留时间(ms)
 * @param [in] weaveCircleRadio 圆形摆动-回调比率(0-100%)
 * @param [in] weaveStationary 摆动位置等待，0-等待时间内位置继续移动；1-等待时间内位置静止
 * @param [in] weaveYawAngle 摆动方向方位角(绕摆动Z轴旋转)，单位°
 * @return 错误码
 */
errno_t FRRobot::WeaveSetPara(int weaveNum, int weaveType, double weaveFrequency, 
                            int weaveIncStayTime, double weaveRange, double weaveLeftRange, 
                            double weaveRightRange, int additionalStayTime, int weaveLeftStayTime, 
                            int weaveRightStayTime, int weaveCircleRadio, int weaveStationary, double weaveYawAngle, double weaveRotAngle)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = weaveNum;
    param[1] = weaveType;
    param[2] = weaveFrequency;
    param[3] = weaveIncStayTime;
    param[4] = weaveRange;
    param[5] = weaveLeftRange;
    param[6] = weaveRightRange;
    param[7] = additionalStayTime;
    param[8] = weaveLeftStayTime;
    param[9] = weaveRightStayTime;
    param[10] = weaveCircleRadio;
    param[11] = weaveStationary;
    param[12] = weaveYawAngle;
    param[13] = weaveRotAngle;

    if (c.execute("WeaveSetPara", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 即时设置摆动参数
 * @param [in] weaveNum 摆焊参数配置编号
 * @param [in] weaveType 摆动类型 0-平面三角波摆动；1-垂直L型三角波摆动；2-顺时针圆形摆动；3-逆时针圆形摆动；4-平面正弦波摆动；5-垂直L型正弦波摆动；6-垂直三角波摆动；7-垂直正弦波摆动
 * @param [in] weaveFrequency 摆动频率(Hz)
 * @param [in] weaveIncStayTime 等待模式 0-周期不包含等待时间；1-周期包含等待时间
 * @param [in] weaveRange 摆动幅度(mm)
 * @param [in] weaveLeftStayTime 摆动左停留时间(ms)
 * @param [in] weaveRightStayTime 摆动右停留时间(ms)
 * @param [in] weaveCircleRadio 圆形摆动-回调比率(0-100%)
 * @param [in] weaveStationary 摆动位置等待，0-等待时间内位置继续移动；1-等待时间内位置静止
 * @return 错误码
 */
errno_t FRRobot::WeaveOnlineSetPara(int weaveNum, int weaveType, double weaveFrequency, int weaveIncStayTime, double weaveRange, int weaveLeftStayTime, int weaveRightStayTime, int weaveCircleRadio, int weaveStationary)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = weaveNum;
    param[1] = weaveType;
    param[2] = weaveFrequency;
    param[3] = weaveIncStayTime;
    param[4] = weaveRange;
    param[5] = weaveLeftStayTime;
    param[6] = weaveRightStayTime;
    param[7] = weaveCircleRadio;
    param[8] = weaveStationary;

    if (c.execute("WeaveOnlineSetPara", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 摆动开始
 * @param [in] weaveNum 摆焊参数配置编号
 * @return 错误码
 */
errno_t FRRobot::WeaveStart(int weaveNum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = weaveNum;

    if (c.execute("WeaveStart", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 摆动结束
 * @param [in] weaveNum 摆焊参数配置编号
 * @return 错误码
 */
errno_t FRRobot::WeaveEnd(int weaveNum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = weaveNum;

    if (c.execute("WeaveEnd", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 正向送丝
 * @param [in] ioType io类型  0-控制器IO；1-扩展IO
 * @param [in] wireFeed 送丝控制  0-停止送丝；1-送丝
 * @return 错误码
 */
errno_t FRRobot::SetForwardWireFeed(int ioType, int wireFeed)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = ioType;
    param[1] = wireFeed;

    if (c.execute("SetForwardWireFeed", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 反向送丝
 * @param [in] ioType io类型  0-控制器IO；1-扩展IO
 * @param [in] wireFeed 送丝控制  0-停止送丝；1-送丝
 * @return 错误码
 */
errno_t FRRobot::SetReverseWireFeed(int ioType, int wireFeed)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = ioType;
    param[1] = wireFeed;

    if (c.execute("SetReverseWireFeed", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 送气
 * @param [in] ioType io类型  0-控制器IO；1-扩展IO
 * @param [in] airControl 送气控制  0-停止送气；1-送气
 * @return 错误码
 */
errno_t FRRobot::SetAspirated(int ioType, int airControl)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = ioType;
    param[1] = airControl;

    if (c.execute("SetAspirated", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

errno_t FRRobot::SegmentWeldStart(DescPose* startDesePos, DescPose* endDesePos, JointPos* startJPos, JointPos* endJPos,
                                  double weldLength, double noWeldLength, int weldIOType, int arcNum, int weldTimeout,
                                  bool isWeave, int weaveNum, int tool, int user, float vel, float acc, float ovl, float blendR,
                                  ExaxisPos* epos, uint8_t search, uint8_t offset_flag, DescPose* offset_pos)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    do
    {
        double distance = 0.0;
        double directionX = 0.0;
        double directionY = 0.0;
        double directionZ = 0.0;

        int weldNum = 0;
        int noWeldNum = 0;
        int i=0;

        param[0] = startDesePos->tran.x;
        param[1] = startDesePos->tran.y;
        param[2] = startDesePos->tran.z;
        param[3] = endDesePos->tran.x;
        param[4] = endDesePos->tran.y;
        param[5] = endDesePos->tran.z;

        /* 获取焊接的方向，距离 */
        if (c.execute("GetSegWeldDisDir", param, result))
        {
            errcode = int(result[0]);
            if(0 != errcode)
            {
                logger_error("execute GetSegWeldDisDir fail %d.", errcode);
                break;
            }
            else{
                distance = double(result[1]);
                directionX = double(result[2]);
                directionY = double(result[3]);
                directionZ = double(result[4]);
            }
        }
        else
        {
            errcode = ERR_XMLRPC_CMD_FAILED;
            break;
        }

        /* 移动到起点 */
        errcode = MoveJ(startJPos, startDesePos, tool, user, vel, acc, ovl, epos, blendR, offset_flag, offset_pos);
        if(0 != errcode)
        {
            logger_error("execute moveJ fail %d.", errcode);
            break;
        }

        /* 进入段焊流程 */
        errcode = ERR_SUCCESS;
        DescPose endOffPose;
        while (i < (int)(distance / (weldLength + noWeldLength)) * 2 + 2)
        {
            memset(&endOffPose, 0, sizeof(DescPose));
            if(i % 2 == 0)
            {
                weldNum += 1;
                /* 确定焊接终点 - 启动焊枪 - 焊接进行 */
                if((weldNum * weldLength + noWeldNum * noWeldLength) > distance)
                {
                    errcode = ARCStart(weldIOType, arcNum, weldTimeout);
                    if(0 != errcode)
                    {
                        logger_error("execute arcstart fail %d.", errcode);
                        break;;
                    }

                    if(isWeave)
                    {
                        errcode = WeaveStart(weaveNum);
                        if(0 != errcode)
                        {
                            logger_error("execute WeaveStart fail %d.", errcode);
                            break;;
                        }
                    }

                    /* 开启焊枪后，边焊边移动 */
                    DescPose tmpWeldDesc = {};
                    JointPos tmpJoint = {};
                    int tmpTool = 0;
                    int tmpUser = 0;
                    GetSegmentWeldPoint(*startDesePos, *endDesePos, distance, tmpWeldDesc, tmpJoint, tmpTool, tmpUser);
                    errcode = MoveL(&tmpJoint, &tmpWeldDesc, tmpTool, tmpUser, vel, acc, ovl, blendR, 0, epos, search, 0, &endOffPose);
                    if(0 != errcode)
                    {
                        ARCEnd(weldIOType, arcNum, weldTimeout);
                        if(isWeave)
                        {
                            WeaveEnd(weaveNum);
                        }
                        logger_error("execute moveL fail %d.", errcode);
                        break;;
                    }

                    /* 焊枪移动到位，停止 */
                    errcode = ARCEnd(weldIOType, arcNum, weldTimeout);
                    if (errcode != 0)
                    {
                        logger_error("execute arcend fail %d.", errcode);
                        break;;
                    }
                    if (isWeave)
                    {
                        errcode = WeaveEnd(weaveNum);
                        if (errcode != 0)
                        {
                            logger_error("execute WeaveEnd fail %d.", errcode);
                            break;;
                        }
                    }
                    break;
                }
                else
                {
                    errcode = ARCStart(weldIOType, arcNum, weldTimeout);
                    if(0 != errcode)
                    {
                        logger_error("execute ARCStart fail %d.", errcode);
                        break;;
                    }

                    if(isWeave)
                    {
                        errcode = WeaveStart(weaveNum);
                        if(0 != errcode)
                        {
                            logger_error("execute WeaveStart fail %d.", errcode);
                            break;;
                        }
                    }

                     /* 开启焊枪后，边焊边移动 */
                    DescPose tmpWeldDesc = {};
                    JointPos tmpJoint = {};
                    int tmpTool = 0;
                    int tmpUser = 0;
                    GetSegmentWeldPoint(*startDesePos, *endDesePos, weldNum* weldLength + noWeldNum * noWeldLength, tmpWeldDesc, tmpJoint, tmpTool, tmpUser);
                    errcode = MoveL(&tmpJoint, &tmpWeldDesc, tmpTool, tmpUser, vel, acc, ovl, blendR, 0, epos, search, 0, &endOffPose);
                    if(0 != errcode)
                    {
                        ARCEnd(weldIOType, arcNum, weldTimeout);
                        if(isWeave)
                        {
                            WeaveEnd(weaveNum);
                        }
                        logger_error("execute moveL fail %d.", errcode);
                        break;;
                    }

                    /* 焊枪移动到位，停止 */
                    errcode = ARCEnd(weldIOType, arcNum, weldTimeout);
                    if (errcode != 0)
                    {
                        logger_error("execute arcend fail %d.", errcode);
                        break;;
                    }
                    if (isWeave)
                    {
                        errcode = WeaveEnd(weaveNum);
                        if (errcode != 0)
                        {
                            logger_error("execute Weaveend fail %d.", errcode);
                            break;;
                        }
                    }
                }
            }
            else
            {
                /* 不需要焊接，不停走就行 */
                noWeldNum += 1;
                if((weldNum * weldLength + noWeldNum * noWeldLength) > distance)
                {
                    DescPose tmpWeldDesc = {};
                    JointPos tmpJoint = {};
                    int tmpTool = 0;
                    int tmpUser = 0;
                    GetSegmentWeldPoint(*startDesePos, *endDesePos, distance, tmpWeldDesc, tmpJoint, tmpTool, tmpUser);
                    errcode = MoveL(&tmpJoint, &tmpWeldDesc, tmpTool, tmpUser, vel, acc, ovl, blendR, 0, epos, search, 0, &endOffPose);
                    if(0 != errcode)
                    {
                        logger_error("execute moveL fail %d.", errcode);
                        break;;
                    }
                    break;;
                }
                else
                {
                    DescPose tmpWeldDesc = {};
                    JointPos tmpJoint = {};
                    int tmpTool = 0;
                    int tmpUser = 0;
                    GetSegmentWeldPoint(*startDesePos, *endDesePos, weldNum* weldLength + noWeldNum * noWeldLength, tmpWeldDesc, tmpJoint, tmpTool, tmpUser);
                    errcode = MoveL(&tmpJoint, &tmpWeldDesc, tmpTool, tmpUser, vel, acc, ovl, blendR, 0, epos, search, 0, &endOffPose);
                    if(0 != errcode)
                    {
                        logger_error("execute movel fail %d.", errcode);
                        break;;
                    }
                }
            }
            i++;
        }
        
        if(ERR_SUCCESS != errcode)
        {
            logger_error("execute weld fail %d.", errcode);
        }

    } while (0);

    c.close();
    return errcode;
}

/**
 * @brief 初始化日志参数;
 * @param output_model：输出模式，0-直接输出；1-缓冲输出；2-异步输出;
 * @param file_path: 文件保存路径+名称，,长度上限256，名称必须是xxx.log的形式，比如/home/fr/linux/fairino.log;
 * @param file_num：滚动存储的文件数量，1~20个.单个文件上限50M;
 * @return errno_t 错误码;
 */
errno_t FRRobot::LoggerInit(int output_model, std::string file_path, int file_num)
{
    int retval = ERR_SUCCESS;
    bool enable_async_output = false;
    bool enable_buf_output = false;

    /* RPC调用后不允许初始化 */
    if(rpc_done)
    {
        std::cout << "init logger fail, please do LoggerInit before RPC. \n";
        return ERR_OTHER;
    }
    /* 设置输出模式 */
    if (1 == output_model)
    {
        enable_async_output = false;
        enable_buf_output = true;
    }
    else if (2 == output_model)
    {
        enable_async_output = true;
        enable_buf_output = false;
    }
    else
    {
        enable_async_output = false;
        enable_buf_output = false;
    }

    /* 设置日志文件参数 */
    retval = fr_logger::set_file_param(file_path, file_num);
    if(0 != retval)
    {
        std::cout << "set log file parameters fail, check file path and file number. \n";
        return ERR_OTHER;
    }

    /* 启动日志功能 */
    fr_logger::logger_init(enable_async_output, enable_buf_output, 1);
    return ERR_SUCCESS;
}

/**
 * @brief 设置日志过滤等级;
 * @param lvl: 过滤等级值，值越小输出日志越少，默认值是1. 1-error, 2-warnning, 3-inform, 4-debug;
 */
void FRRobot::SetLoggerLevel(int lvl)
{
    fr_logger::set_logger_level(lvl);
}

/**
 * @brief 下载文件
 * @param [in] fileType 文件类型    0-lua文件,1-控制器日志，2-所有数据源，3-数据备份包
 * @param [in] fileName 文件名称    “test.lua”
 * @param [in] saveFilePath 保存文件路径    “C：//test/”
 * @return 错误码
 */
errno_t FRRobot::FileDownLoad(int fileType, std::string fileName, std::string saveFilePath)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    /* 检查文件名称，文件路径*/
    if(fileName.length() == 0)
    {
        logger_error("file name can not be empty.");
        return ERR_FILE_NAME;
    }
    if(saveFilePath.length() == 0)
    {
        logger_error("file path can not be empty.");
        return ERR_SAVE_FILE_PATH_NOT_FOUND;
    }

#ifdef WIN32
    std::wstring save_path_wide(saveFilePath.begin(), saveFilePath.end());
    if (GetFileAttributesA(saveFilePath.c_str()) == INVALID_FILE_ATTRIBUTES)
    {
        logger_error("path %s do not exist.", saveFilePath.c_str());
        return ERR_SAVE_FILE_PATH_NOT_FOUND;
    }
#else
    char save_path[MAX_FILE_PATH_LENGTH];
    memset(save_path, 0, MAX_FILE_PATH_LENGTH);
    snprintf(save_path, MAX_FILE_PATH_LENGTH, "%s", saveFilePath.c_str());
    if(access(save_path, F_OK) != 0)
    {
        logger_error("path %s do not exist.", saveFilePath.c_str());
        return ERR_SAVE_FILE_PATH_NOT_FOUND;
    }
#endif

    logger_info("path %s do exist.", saveFilePath.c_str());

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    /* 发起远程调用 */
    param[0] = fileType;
    param[1] = fileName;
    if (c.execute("FileDownload", param, result))
    {
        errcode = int(result);
        if (-1 == errcode)
        {
            logger_error("do not find file. %d ", ERR_POINTTABLE_NOTFOUND);
            c.close();
            return ERR_POINTTABLE_NOTFOUND;
        }
        if (0 != errcode)
        {
            logger_error("execute FileDownload fail, %d " , errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    // 发起网络连接，客户端;
    fr_network::socket_fd fd = fr_network::get_socket_fd();
    logger_info("get socker fd %d", fd);

#ifdef WIN32
    int syncnt = 10;
    if (setsockopt(fd, IPPROTO_TCP, TCP_MAXRT, (char *)&syncnt, sizeof(syncnt)) == SOCKET_ERROR)
    {
        logger_error("Failed to set TCP_MAXRT option.\n");
        c.close();
        fr_network::close_fd(fd);
        return ERR_OTHER;
    }
    int timeout = 5000;
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(int));
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(int));
#else
    int syncnt = 3;
    setsockopt(fd, IPPROTO_TCP, TCP_SYNCNT, &syncnt, sizeof(syncnt));
    struct timeval tv;
    tv.tv_sec = 5;
    tv.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (const char *)&tv, sizeof tv);
#endif

    errcode = fr_network::connect(fd, robot_ip, DOWNLOAD_POINT_TABLE_PORT);
    logger_info("connect error code is %d ", errcode);
    if ((errcode < 0))
    {
        logger_error("connect fail, %s. error code is: %d", strerror(errno), errcode);
        fr_network::close_fd(fd);
        c.close();
        return ERR_SOCKET_COM_FAILED;
    }

    // 超时时间15s；内容上限50M;
    char recv_buf[1024 * 8] = {0};
    string total_buf; // 头+长度+md5+点位表+尾;
    total_buf.reserve(DOWNLOAD_FILE_MAX_SIZE);

    string point_table; // 点位表;
    point_table.reserve(DOWNLOAD_FILE_MAX_SIZE);

    string except_md5;
    except_md5.reserve(40);
    string compute_md5;
    compute_md5.reserve(40);
    string file_path;

    int recv_bytes = 0;
    int total_bytes = 0; // 总共收到的字节数;
    int except_bytes = 0;
    bool find_head = false;
    bool find_size_md5 = false;
    bool download_success_finish = false;

    errcode = 0;
    do
    {
        double last_time = 0;
        double cur_time = 0;
        auto now_ns = std::chrono::system_clock::now().time_since_epoch();
        auto now_ms = chrono::duration<double, std::milli>(now_ns);
        cur_time = now_ms.count();
        last_time = cur_time;

        // 进入文件接收循环;
        while (1)
        {
            auto now_ns = std::chrono::system_clock::now().time_since_epoch();
            auto now_ms = chrono::duration<double, std::milli>(now_ns);
            cur_time = now_ms.count();
            if (cur_time - last_time > DOWNLOAD_FILE_TIME_LIMIT_MS)
            {
                logger_error("download over time %d ms", DOWNLOAD_FILE_TIME_LIMIT_MS);
                logger_error("total bytes %d, %s.", total_bytes, total_buf.c_str());
                download_success_finish = false;
                break;
            }

            recv_bytes = 0;
            recv_bytes = recv(fd, (char *)&recv_buf, sizeof(recv_buf), 0);
            if (recv_bytes < 0)
            {
                logger_error(" recv bytes is: %d, %s.", recv_bytes, strerror(errno));
                download_success_finish = false;
                break;
            }

            total_buf.append(recv_buf, recv_bytes);
            total_bytes += recv_bytes;
            recv_bytes = 0;
            logger_info("size is %d, buf is: %s.", total_bytes, total_buf.c_str());

            // 找头;
            if ((!find_head) && (total_bytes > 4))
            {
                size_t res = total_buf.find(FILE_HEAD);
                if ((string::npos == res))
                {
                    logger_error("do not find file head in: %s", total_buf.c_str());
                }
                if (0 != res)
                {
                    logger_error("file head is wrong: %s.", total_buf.c_str());
                }
                else
                {
                    find_head = true;
                }
            }

            // 找md5值 和 数据长度;
            if (find_head && (!find_size_md5) && (total_bytes > 14 + 32))
            {
                string str_tmp = total_buf.substr(4, 10);
                except_bytes = stoi(str_tmp);

                except_md5 = total_buf.substr(14, 32);
                find_size_md5 = true;

                logger_info("except bytes is: %d.", except_bytes);
            }

            // 检查完整性;
            if (find_head && find_size_md5 && (except_bytes == total_bytes))
            {
                logger_info("have recv all bytes: %d.", total_bytes);
                download_success_finish = true;
                break;
            }
        }
        // 检查下载，提取点位表;
        if (download_success_finish == true)
        {
            point_table = total_buf.substr(14 + 32, total_bytes - 14 - 32 - 4);
            errcode = ERR_SUCCESS;
        }
        else
        {
            logger_error("download fail.");
            errcode = ERR_DOWN_LOAD_FILE_FAILED;
            break;
        }

        /* 校验md5值 */
        compute_md5 = fr_md5::md5_hash_hex(point_table);
        if (0 == except_md5.compare(compute_md5))
        {
            logger_info("md5 is correct.");
            errcode = ERR_SUCCESS;
        }
        else
        {
            logger_error("except is: %s, compute is: %s.", except_md5.c_str(), compute_md5.c_str());
            errcode = ERR_DOWN_LOAD_FILE_CHECK_FAILED;
            break;
        }

        /* 将内容写入文件 */
        file_path = saveFilePath + fileName;
        ofstream input_file(file_path, ios::binary);
        if (input_file.is_open())
        {
            input_file << point_table;
            input_file.flush();
            input_file.close();
            errcode = ERR_SUCCESS;
        }
        else
        {
            logger_error("open file fail.");
            errcode = ERR_DOWN_LOAD_FILE_WRITE_FAILED;
            input_file.close();
            break;
        }
    } while (0);

    // 给服务器返回文件传输结果;
    int send_bytes = 0;
    char _success[] = "SUCCESS";
    char _fail[] = "FAIL";
    if (errcode == ERR_SUCCESS)
    {
        send_bytes = send(fd, _success, sizeof("SUCCESS"), 0);
        if (send_bytes < 0)
        {
            logger_error("send success faild %d.", send_bytes);
        }
    }
    else
    {
        send_bytes = send(fd, _fail, sizeof("FAIL"), 0);
        if (send_bytes < 0)
        {
            logger_error("send fail faild %d.", send_bytes);
        }
    }

    // 释放资源，函数返回;
    c.close();
    fr_network::close_fd(fd);
    return errcode;

}

/**
 * @brief 下载Lua文件
 * @param [in] fileName 要下载的lua文件名“test.lua”
 * @param [in] savePath 保存文件本地路径“D://Down/”
 * @return 错误码
 */
errno_t FRRobot::LuaDownLoad(std::string fileName, std::string savePath)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    /* 先检查，lua包和tar.gz压缩包的逻辑不同 */
    if(fileName.size() == 0)
    {
        logger_error("file name can not be empty.");
        return ERR_UPLOAD_FILE_NOT_FOUND;
    }
    std::vector<std::string> _file_name = split(fileName, '.');
    /*upload lua*/
    if ((_file_name.size() == 2) && (_file_name[1] == "lua"))
    {
        logger_info("download lua.");
    }
    /* upload .tar.gz */
    else if((_file_name.size() == 3) && (_file_name[1] == "tar") && (_file_name[2] == "gz"))
    {
        logger_info("download tar.gz.");
        XmlRpcClient c(serverUrl, 20003);
        XmlRpcValue param, result;

        param = fileName;
        if(c.execute("LuaDownLoadPrepare", param, result))
        {
            errcode = int(result);
            if(0 != errcode){
                logger_error("LuaDownLoadPrepare fail.");
                c.close();
                return errcode;
            }
            logger_info("LuaDownLoadPrepare success.");
        }
        else{
            logger_error("execute LuaDownLoadPrepare fail.");
            c.close();
            return ERR_XMLRPC_CMD_FAILED;
        }
    }
    else{
        logger_error("you can not download %s, file name should be xxx.lua or xxx.tar.gz", fileName.c_str());
        errcode = ERR_FILE_NAME;
    }

    errcode = FileDownLoad(0, fileName, savePath);
    return errcode;
}

/**
 * @brief 上传文件
 * @param [in] fileType 文件类型    0-lua文件,1-机器人软件升级包
 * @param [in] filePath 保存文件路径    “C：//test/test.lua”
 * @return 错误码
 */
errno_t FRRobot::FileUpLoad(int fileType, std::string filePath)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    /* 检查文件名称，文件路径*/
    if (filePath.length() == 0)
    {
        logger_error("file path can not be empty.");
        return ERR_FILE_NAME;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

// 检查文件路径;
#ifdef WIN32
    std::wstring save_path_wide(filePath.begin(), filePath.end());
    if (GetFileAttributesA(filePath.c_str()) == INVALID_FILE_ATTRIBUTES)
    {
        logger_error("path %s do not exist.", filePath.c_str());
        c.close();
        return ERR_UPLOAD_FILE_NOT_FOUND;
    }
#else
    char save_path[MAX_FILE_PATH_LENGTH];
    memset(save_path, 0, MAX_FILE_PATH_LENGTH);
    snprintf(save_path, MAX_FILE_PATH_LENGTH, "%s", filePath.c_str());
    if(access(save_path, F_OK) != 0)
    {
        logger_error("path %s do not exist.", save_path);
        c.close();
        return ERR_UPLOAD_FILE_NOT_FOUND;
    }
#endif
    long file_size = 0;
    // 检查文件大小;
#ifdef WIN32
    HANDLE hFile = CreateFile(filePath.c_str(), GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (INVALID_HANDLE_VALUE != hFile)
    {
        DWORD dwHigh = 0;
        DWORD dwSize = ::GetFileSize(hFile, &dwHigh);
        CloseHandle(hFile);
        file_size = dwSize;
        if (file_size < 0)
        {
            logger_error("get file size fail, size is: %l.",  file_size);
            c.close();
            return ERR_UPLOAD_FILE_ERROR;
        }
    }else{
        logger_error("open file fail, error is: %lu.", GetLastError());
        c.close();
        return ERR_UPLOAD_FILE_ERROR;
    }
#else
    FILE *file_p = fopen(save_path, "rb");
    if(NULL == file_p)
    {
        logger_error("open file %s fail.", save_path);
        c.close();
        return ERR_UPLOAD_FILE_ERROR;
    }
    fseek(file_p, 0, SEEK_END);
    file_size = ftell(file_p);
    fclose(file_p);
#endif

    long total_size = file_size + 16 + 32 +2;
    logger_info("file size is: %ld, total size is: %ld.", file_size, total_size);
    if (total_size > UPLOAD_FILE_MAX_SIZE)
    {
        logger_error("file size have over max limit: %d", UPLOAD_FILE_MAX_SIZE);
        c.close();
        return ERR_FILE_TOO_LARGE;
    }

    /* 获取点位表名称 */
    string point_table_name;
#ifdef WIN32
    point_table_name = PathFindFileNameA(filePath.c_str());
#else
    char path_dup[MAX_FILE_PATH_LENGTH];
    memset(path_dup, 0, MAX_FILE_PATH_LENGTH);
    memcpy(path_dup, save_path, MAX_FILE_PATH_LENGTH-1);
    point_table_name.append(basename(path_dup));
    logger_info("name is: %s.", point_table_name.c_str());
#endif


    // 发起远程调用;
    param[0] = fileType;
    param[1] = point_table_name;
    if (c.execute("FileUpload", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute FileUpload fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }
    
    /* 创建，设置套接字 */
    fr_network::socket_fd fd = fr_network::get_socket_fd();

    /* 设置连接超时 */
    int syncnt = 4;
#ifdef WIN32
    if (setsockopt(fd, IPPROTO_TCP, TCP_MAXRT, (char *)&syncnt, sizeof(syncnt)) == SOCKET_ERROR)
    {
        logger_error("Failed to set TCP_MAXRT option.\n");
        c.close();
        fr_network::close_fd(fd);
        return ERR_OTHER;
    }
    int timeout = 400000;
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (char *)&timeout, sizeof(int));
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(int));
#else
    setsockopt(fd, IPPROTO_TCP, TCP_SYNCNT, &syncnt, sizeof(syncnt));
    struct timeval tv;
    tv.tv_sec = 4;
    tv.tv_usec = 0;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (const char *)&tv, sizeof tv);
#endif

    Sleep(30);
    for (int i = 0; i < 100; i++)
    {
        errcode = fr_network::connect(fd, robot_ip, UPLOAD_POINT_TABLE_PORT);
        if (errcode < 0)
        {
            Sleep(30);
        }
        else
        {
            break;
        }
    }
    if (errcode < 0)
    {
        logger_error("connect fail.");
        fr_network::close_fd(fd);
        c.close();
        return ERR_SOCKET_COM_FAILED;
    }
 
    /* 发送 头+内容+尾，send_buf_first, send_buf, send_buf_last */
    do
    {
        string compute_md5;
        compute_md5.reserve(40);
        string send_buf; // 点位表内容;
        /* 检查，分配内存 */
        if(total_size > send_buf.max_size())
        {
            logger_error("memory is not enough. max size is: %u, total size is: %l", send_buf.max_size(), total_size);
            errcode = ERR_FILE_TOO_LARGE;
            break;
        }
        send_buf.resize(file_size, '\0');

        /* 读取文件, 计算md5 */
        ifstream input_file(filePath, std::ios::binary);
        if (!input_file.is_open())
        {
            logger_error("file path is not open.");
            errcode = ERR_FILE_OPEN_FAILED;
            break;
        }

        input_file.seekg(0, std::ios::end);
        std::streampos fileSize = input_file.tellg();
        input_file.seekg(0, std::ios::beg);

        input_file.read(&send_buf[0], fileSize);
        input_file.close();

        // stringstream buffer;
        // buffer << input_file.rdbuf();
        // send_buf = buffer.str();
        // buffer.str("");
        // input_file.close();

        compute_md5 = fr_md5::md5_hash_hex(send_buf);
        logger_info("send buf size is: %d. md5 is: %s.", send_buf.length(), compute_md5.c_str());

        // 发送头+size+md5;
        std::ostringstream headerStream;
        headerStream << FILE_HEAD << std::setw(10) << std::setfill('0') << total_size << compute_md5;

        std::string send_buf_first = headerStream.str();
        int send_bytes = send(fd, const_cast<char *>(send_buf_first.c_str()), send_buf_first.length(), 0);
        if ((send_bytes < 0) || (static_cast<unsigned int>(send_bytes) != send_buf_first.length()))
        {
            logger_error("send head %s fail.", send_buf_first.c_str());
            errcode = ERR_SOCKET_SEND_FAILED;
            break;
        }
        logger_info("send head end");

        /* 发送文件本体 */
        send_bytes = 0;
        unsigned int total_send_bytes = 0;
        bool send_point_table_success = true;
        while (total_send_bytes < send_buf.length())
        {
            send_bytes = 0;
            send_bytes = send(fd, const_cast<char *>(send_buf.c_str() + total_send_bytes), send_buf.length() - total_send_bytes, 0);
            if (send_bytes < 0)
            {
                logger_error("has send bytes %d", total_send_bytes);
                logger_error("send buf is: %s.", send_buf.c_str()+total_send_bytes);
                logger_error("send bytes is %d", send_buf.length() - total_send_bytes);
                errcode = ERR_SOCKET_SEND_FAILED;
                send_point_table_success = false;
                break;
            }

            total_send_bytes += send_bytes;
            fileUploadPercent = total_send_bytes * 100.0 / send_buf.length();
        }
        if (false == send_point_table_success)
        {
            logger_error("send file fail \n");
            break;
        }
        logger_info("except %d, total send bytes: %d", send_buf.length(), total_send_bytes);

        // 发送文件尾;
        send_bytes = 0;
        string send_buf_last = POINT_TABLE_TAIL;
        send_bytes = send(fd, const_cast<char *>(send_buf_last.c_str()), send_buf_last.length(), 0);
        if ((send_bytes < 0 && errno != 0) || (static_cast<unsigned int>(send_bytes) != send_buf_last.length()))
        {
            logger_error("send last buf : %s fail, send bytes %d   errno is %s", send_buf_last.c_str(), send_bytes, strerror(errno));
            errcode = ERR_SOCKET_SEND_FAILED;
            break;
        }
        logger_info("send buf tail end");

        // 接收、校验客户端返回;
        int recv_bytes = 0;
        char recv_buf[1024];
        memset(recv_buf, 0, sizeof(recv_buf));
        recv_bytes = recv(fd, recv_buf, sizeof(recv_buf), 0);
        if (recv_bytes < 0)
        {
            logger_error("recv fail");
            errcode = ERR_SOCKET_RECV_FAILED;
            break;
        }
        logger_info("recv %d, %s", recv_bytes, recv_buf);
        string recv_str(recv_buf);
        if (0 != recv_str.compare("SUCCESS"))
        {
            logger_error("recv %s fail. upload file fail", recv_str.c_str());
            errcode = ERR_OTHER;
            break;
            ;
        }
        else
        {
            logger_error("send file success");
        }

        errcode = ERR_SUCCESS;

    } while (0);

    // 释放资源，函数返回;
    c.close();
    fr_network::close_fd(fd);
    return errcode;
}

/**
 * @brief 上传Lua文件
 * @param [in] filePath 本地lua文件路径名
 * @return 错误码
 */
errno_t FRRobot::LuaUpload(std::string filePath)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    /* 先上传，再发起rpc通知做远程检查 */
    int errcode = FileUpLoad(0, filePath);
    if(0 == errcode)
    {
        logger_info("upload file success. try to check lua format.");
        XmlRpcClient c(serverUrl, 20003);
        XmlRpcValue param, result;

        /* 提取文件名称 */
        size_t pos = filePath.find_last_of("/\\");
        if(std::string::npos == pos)
        {
            c.close();
            logger_error("format of path is wrong, should be like /home/fd/xxx.tar.gz");
            return ERR_FILE_NAME;
        }
        std::string filename = filePath.substr(pos + 1);

        // std::string filename = "2222.tar.gz";
        param[0] = filename;
        logger_info("file name is: [%s]", filename.c_str());
        if(c.execute("LuaUpLoadUpdate", param, result))
        {
            errcode = int(result[0]);
            std::string res_str = std::string(result[1]);
            logger_info("lua format, error code is: %d, %s", errcode, res_str.c_str());
            if(0 != errcode)
            {
                logger_error("lua format error.,error code is: %d, %s", errcode, res_str.c_str());
            }
            c.close();
            return errcode;
        }else{
            logger_error("execute LuaUpLoadUpdate fail.");
            c.close(); 
            return ERR_XMLRPC_CMD_FAILED;
        }
    }else{
        logger_error("upload file fail. errcode is: %d.", errcode);
    }

    return errcode;
}

/**
 * @brief 上传文件
 * @param [in] fileType 文件类型    0-lua文件
 * @param [in] fileName 文件名称    “test.lua”
 * @return 错误码
 */
errno_t FRRobot::FileDelete(int fileType, std::string fileName)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    /* 检查文件名称，文件路径*/
    if (fileName.length() == 0)
    {
        logger_error("file path can not be empty.");
        return ERR_FILE_NAME;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = fileType;
    param[1] = fileName;
    if (c.execute("FileDelete", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute FileDelete fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 下载Lua文件
 * @param [in] fileName 要下载的lua文件名“test.lua”
 * @return 错误码
 */
errno_t FRRobot::LuaDelete(std::string fileName)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = FileDelete(0, fileName);
    return errcode;
}

/**
 * @brief 获取当前所有lua文件名称
 * @param [out] luaNames lua文件名列表  形如 abd;def;
 * @return 错误码
 */
errno_t FRRobot::GetLuaList(std::list<std::string>* luaNames)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;
    int lua_num = 0;
    std::string lua_name_str;

    if (c.execute("GetLuaList", param, result))
    {
        errcode = int(result[0]);
        if(0 == errcode)
        {
            lua_num = int(result[1]);
            logger_info("there %d luas.", lua_num);
            lua_name_str = string(result[2]);
            logger_info("have lua : [%s]", lua_name_str.c_str());

            /* 分割字符串，拿出名称 */
            size_t pos = 0;
            size_t found = 0;
            std::string name_tmp;
            while (std::string::npos != pos)
            {
                name_tmp.clear();
                found = lua_name_str.find(';', pos);

                if(found != std::string::npos)
                {
                    name_tmp = lua_name_str.substr(pos, found - pos);
                    luaNames->push_back(name_tmp);
                    pos = found + 1;
                }
                else{
                    // name_tmp = lua_name_str.substr(pos);
                    // luaNames.push_back(name_tmp);
                    break;
                }
            }

            if(lua_num != luaNames->size())
            {
                logger_error("except number is: %d, recv number is: %d.", lua_num, luaNames->size());
                for(auto it = luaNames->begin(); it != luaNames->end(); it++)
                {
                    logger_error("name is: %s.", (*it).c_str());
                }
                c.close();
                luaNames->clear();
                return ERR_OTHER;
            }            
        }else{
            c.close();
            logger_error("GetLuaList fail, error code: %d.", errcode);
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

errno_t FRRobot::AuxServoSetParam(int servoId, int servoCompany, int servoModel,
                         int servoSoftVersion, int servoResolution, double axisMechTransRatio)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = servoId;
    param[1] = servoCompany;
    param[2] = servoModel;
    param[3] = servoSoftVersion;
    param[4] = servoResolution;
    param[5] = axisMechTransRatio;

    if (c.execute("AuxServoSetParam", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute AuxServoSetParam fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 获取485扩展轴配置参数
 * @param [in] servoId 伺服驱动器ID，范围[1-16],对应从站ID
 * @param [out] servoCompany 伺服驱动器厂商，1-戴纳泰克
 * @param [out] servoModel 伺服驱动器型号，1-FD100-750C
 * @param [out] servoSoftVersion 伺服驱动器软件版本，1-V1.0
 * @param [out] servoResolution 编码器分辨率
 * @param [out] axisMechTransRatio 机械传动比
 * @return 错误码
 */
errno_t FRRobot::AuxServoGetParam(int servoId, int* servoCompany, int* servoModel,
                                  int* servoSoftVersion, int* servoResolution, double* axisMechTransRatio)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = servoId;

    if (c.execute("AuxServoGetParam", param, result))
    {
        errcode = int(result[0]);
        if(0 == errcode)
        {
            *servoCompany = (int)result[1];
            *servoModel = (int)result[2];
            *servoSoftVersion = (int)result[3];
            *servoResolution = (int)result[4];
            *axisMechTransRatio = (double)result[5];
        }
        if (0 != errcode)
        {
            logger_error("execute AuxServoGetParam fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置485扩展轴使能/去使能
 * @param [in] servoId 伺服驱动器ID，范围[1-16],对应从站ID
 * @param [in] status 使能状态，0-去使能， 1-使能
 * @return 错误码
 */
errno_t FRRobot::AuxServoEnable(int servoId, int status)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = servoId;
    param[1] = status;

    if (c.execute("AuxServoEnable", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute AuxServoEnable fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置485扩展轴控制模式
 * @param [in] servoId 伺服驱动器ID，范围[1-16],对应从站ID
 * @param [in] mode 控制模式，0-位置模式，1-速度模式
 * @return 错误码
 */
errno_t FRRobot::AuxServoSetControlMode(int servoId, int mode)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = servoId;
    param[1] = mode;

    if (c.execute("AuxServoSetControlMode", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute AuxServoSetControlMode fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置485扩展轴目标位置(位置模式)
 * @param [in] servoId 伺服驱动器ID，范围[1-16],对应从站ID
 * @param [in] pos 目标位置，mm或°
 * @param [in] speed 目标速度，mm/s或°/s
 * @param [in] acc 加速度百分比[0-100]
 * @return 错误码
 */
errno_t FRRobot::AuxServoSetTargetPos(int servoId, double pos, double speed, double acc)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = servoId;
    param[1] = pos;
    param[2] = speed;
    param[3] = acc;

    if (c.execute("AuxServoSetTargetPos", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute AuxServoSetTargetPos fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置485扩展轴目标速度(速度模式)
 * @param [in] servoId 伺服驱动器ID，范围[1-16],对应从站ID
 * @param [in] speed 目标速度，mm/s或°/s
 * @param [in] acc 加速度百分比[0-100]
 * @return 错误码
 */
errno_t FRRobot::AuxServoSetTargetSpeed(int servoId, double speed, double acc)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = servoId;
    param[1] = speed;
    param[2] = acc;

    if (c.execute("AuxServoSetTargetSpeed", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute AuxServoSetTargetSpeed fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置485扩展轴目标转矩(力矩模式)
 * @param [in] servoId 伺服驱动器ID，范围[1-16],对应从站ID
 * @param [in] torque 目标力矩，Nm
 * @return 错误码
 */
errno_t FRRobot::AuxServoSetTargetTorque(int servoId, double torque)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = servoId;
    param[1] = torque;

    if (c.execute("AuxServoSetTargetTorque", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute AuxServoSetTargetTorque fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置485扩展轴回零
 * @param [in] servoId 伺服驱动器ID，范围[1-16],对应从站ID
 * @param [in] mode 回零模式，0-当前位置回零；1-限位回零
 * @param [in] searchVel 回零速度，mm/s或°/s
 * @param [in] latchVel 箍位速度，mm/s或°/s
 * @param [in] acc 加速度百分比[0-100]
 * @return 错误码
 */
errno_t FRRobot::AuxServoHoming(int servoId, int mode, double searchVel, double latchVel, double acc)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = servoId;
    param[1] = mode;
    param[2] = searchVel;
    param[3] = latchVel;
    param[4] = acc;

    if (c.execute("AuxServoHoming", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute AuxServoHoming fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 清除485扩展轴错误信息
 * @param [in] servoId 伺服驱动器ID，范围[1-16],对应从站ID
 * @return 错误码
 */
errno_t FRRobot::AuxServoClearError(int servoId)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = servoId;

    if (c.execute("AuxServoClearError", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute AuxServoClearError fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 获取485扩展轴伺服状态
 * @param [in] servoId 伺服驱动器ID，范围[1-16],对应从站ID
 * @param [out] servoErrCode 伺服驱动器故障码
 * @param [out] servoState 伺服驱动器状态
 * @param [out] servoPos 伺服当前位置 mm或°
 * @param [out] servoSpeed 伺服当前速度 mm/s或°/s
 * @param [out] servoTorque 伺服当前转矩Nm
 * @return 错误码
 */
errno_t FRRobot::AuxServoGetStatus(int servoId, int* servoErrCode, int* servoState, double* servoPos,
                                   double* servoSpeed, double* servoTorque)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = servoId;

    if (c.execute("AuxServoGetStatus", param, result))
    {
        errcode = int(result[0]);
        if (0 == errcode)
        {
            *servoErrCode = (int)result[1];
            *servoState = (int)result[2];
            *servoPos = (double)result[3];
            *servoSpeed = (double)result[4];
            *servoTorque = (double)result[5];
        }
        if (0 != errcode)
        {
            logger_error("execute AuxServoGetStatus fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置状态反馈中485扩展轴数据轴号
 * @param [in] servoId 伺服驱动器ID，范围[1-16],对应从站ID
 * @return 错误码
 */
errno_t FRRobot::AuxServosetStatusID(int servoId)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = servoId;

    if (c.execute("AuxServosetStatusID", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute AuxServosetStatusID fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 获取机器人实时状态结构体
 * @param [out] pkg 机器人实时状态结构体
 * @return 错误码
 */
errno_t FRRobot::GetRobotRealTimeState(ROBOT_STATE_PKG *pkg)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if(nullptr == pkg)
    {
        logger_error("GetRobotRealTimeState fail, input can not be a nullptr");
        return -1;
    }

    memcpy(pkg, robot_state_pkg.get(), sizeof(ROBOT_STATE_PKG));
    return 0;
}
/**
 * @brief 获取机器人外设协议
 * @param [out] protocol 机器人外设协议号 4096-扩展轴控制卡；4097-ModbusSlave；4098-ModbusMaster
 * @return 错误码
 */
errno_t FRRobot::GetExDevProtocol(int *protocol)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetExDevProtocol", param, result))
    {
        errcode = int(result[0]);
        if (0 != errcode)
        {
            logger_error("execute GetExDevProtocol fail: %d.", errcode);
            c.close();
            return errcode;
        }
        if(0 != errcode)
        {
            *protocol = (int)(result[1]);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 获取机器人外设协议
 * @param [in] protocol 机器人外设协议号 4096-扩展轴控制卡；4097-ModbusSlave；4098-ModbusMaster
 * @return 错误码
 */
errno_t FRRobot::SetExDevProtocol(int protocol)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = protocol;

    if (c.execute("SetExDevProtocol", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetExDevProtocol fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置机器人加速度
 * @param [in] acc 机器人加速度百分比
 * @return 错误码
 */
errno_t FRRobot::SetOaccScale(double acc)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param = acc;

    if (c.execute("SetOaccScale", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetOaccScale fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 控制箱AO飞拍开始
 * @param [in] AONum 控制箱AO编号
 * @param [in] maxTCPSpeed 最大TCP速度值[1-5000mm/s]，默认1000
 * @param [in] maxAOPercent 最大TCP速度值对应的AO百分比，默认100%
 * @param [in] zeroZoneCmp 死区补偿值AO百分比，整形，默认为20%，范围[0-100]
 * @return 错误码
 */

errno_t FRRobot::MoveAOStart(int AONum, int maxTCPSpeed, int maxAOPercent, int zeroZoneCmp)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = AONum;
    param[1] = maxTCPSpeed;
    param[2] = maxAOPercent;
    param[3] = zeroZoneCmp;

    if (c.execute("MoveAOStart", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute MoveAOStart fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 控制箱AO飞拍停止
 * @return 错误码
 */
errno_t FRRobot::MoveAOStop()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("MoveAOStop", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 末端AO飞拍开始
 * @param [in] AONum 末端AO编号
 * @param [in] maxTCPSpeed 最大TCP速度值[1-5000mm/s]，默认1000
 * @param [in] maxAOPercent 最大TCP速度值对应的AO百分比，默认100%
 * @param [in] zeroZoneCmp 死区补偿值AO百分比，整形，默认为20%，范围[0-100]
 * @return 错误码
 */
errno_t FRRobot::MoveToolAOStart(int AONum, int maxTCPSpeed, int maxAOPercent, int zeroZoneCmp)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = AONum;
    param[1] = maxTCPSpeed;
    param[2] = maxAOPercent;
    param[3] = zeroZoneCmp;

    if (c.execute("MoveToolAOStart", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute MoveToolAOStart fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 末端AO飞拍停止
 * @return 错误码
 */
errno_t FRRobot::MoveToolAOStop()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("MoveToolAOStop", param, result))
    {
        errcode = int(result);
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
* @brief UDP扩展轴通讯参数配置
* @param [in] ip PLC IP地址
* @param [in] port	端口号
* @param [in] period	通讯周期(ms，默认为2，请勿修改此参数)
* @param [in] lossPkgTime	丢包检测时间(ms)
* @param [in] lossPkgNum	丢包次数
* @param [in] disconnectTime	通讯断开确认时长
* @param [in] reconnectEnable	通讯断开自动重连使能 0-不使能 1-使能
* @param [in] reconnectPeriod	重连周期间隔(ms)
* @param [in] reconnectNum	重连次数
* @param [in] selfConnect 断电重启是否自动建立连接；0-不建立连接；1-建立连接
* @return 错误码
*/
errno_t FRRobot::ExtDevSetUDPComParam(string ip, int port, int period, int lossPkgTime, int lossPkgNum, int disconnectTime, int reconnectEnable, int reconnectPeriod, int reconnectNum, int selfConnect)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = ip;
    param[1] = port;
    param[2] = period;
    param[3] = lossPkgTime;
    param[4] = lossPkgNum;
    param[5] = disconnectTime;
    param[6] = reconnectEnable;
    param[7] = reconnectPeriod;
    param[8] = reconnectNum;
    param[9] = selfConnect;

    if (c.execute("ExtDevSetUDPComParam", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute ExtDevSetUDPComParam fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 获取UDP扩展轴通讯参数
 * @param [out] ip PLC IP地址
 * @param [out] port	端口号
 * @param [out] period	通讯周期(ms，默认为2，请勿修改此参数)
 * @param [out] lossPkgTime	丢包检测时间(ms)
 * @param [out] lossPkgNum	丢包次数
 * @param [out] disconnectTime	通讯断开确认时长
 * @param [out] reconnectEnable	通讯断开自动重连使能 0-不使能 1-使能
 * @param [out] reconnectPeriod	重连周期间隔(ms)
 * @param [out] reconnectNum	重连次数
 * @return 错误码
 */
errno_t FRRobot::ExtDevGetUDPComParam(string& ip, int& port, int& period, int& lossPkgTime, int& lossPkgNum, int& disconnectTime, int& reconnectEnable, int& reconnectPeriod, int& reconnectNum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ExtDevGetUDPComParam", param, result))
    {
        errcode = int(result[0]);
        if (0 != errcode)
        {
            logger_error("execute ExtDevGetUDPComParam fail: %d.", errcode);
            c.close();
            return errcode;
        }
        else
        {
            ip = (string)(result[1]);
            port = (int)(result[2]);
            period = (int)(result[3]);
            lossPkgTime = (int)(result[4]);
            lossPkgNum = (int)(result[5]);
            disconnectTime = (int)(result[6]);
            reconnectEnable = (int)(result[7]);
            reconnectPeriod = (int)(result[8]);
            reconnectNum = (int)(result[9]);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 加载UDP通信
 * @return 错误码
 */
errno_t FRRobot::ExtDevLoadUDPDriver()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ExtDevLoadUDPDriver", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute ExtDevLoadUDPDriver fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 卸载UDP通信
 * @return 错误码
 */
errno_t FRRobot::ExtDevUnloadUDPDriver()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ExtDevUnloadUDPDriver", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute ExDevUnloadUDPDriver fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief UDP扩展轴回零
 * @param [in] axisID 轴号[1-4]
 * @param [in] mode 回零方式
 * @param [in] searchVel 寻零速度(mm/s)
 * @param [in] latchVel 寻零箍位速度(mm/s)
 * @return 错误码
 */
errno_t FRRobot::ExtAxisSetHoming(int axisID, int mode, double searchVel, double latchVel)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = axisID;
    param[1] = mode;
    param[2] = searchVel;
    param[3] = latchVel;

    if (c.execute("ExtAxisSetHoming", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute ExtAxisSetHoming fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief UDP扩展轴点动开始
 * @param [in] axisID 轴号[1-4]
 * @param [in] direction 转动方向 0-反向；1-正向
 * @param [in] vel 速度(mm/s)
 * @param [in] acc (加速度 mm/s2)
 * @param [in] maxDistance 最大点动距离
 * @return 错误码
 */
errno_t FRRobot::ExtAxisStartJog(int axisID, int direction, double vel, double acc, double maxDistance)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = 6;
    param[1] = axisID;
    param[2] = direction;
    param[3] = vel;
    param[4] = acc;
    param[5] = maxDistance;

    if (c.execute("ExtAxisStartJog", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute ExtAxisStartJog fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief UDP扩展轴点动停止
 * @param [in] axisID 轴号[1-4]
 * @return 错误码
 */
errno_t FRRobot::ExtAxisStopJog(int axisID)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    static int cnt = 0;

    memset(g_sendbuf, 0, BUFFER_SIZE * sizeof(char));
    sprintf(g_sendbuf, "/f/bIII%dIII240III14IIIStopExtAxisJogIII/b/f", cnt);
    cnt++;
    is_sendcmd = true;

    logger_info("ExtAxisStopJog.");
    return errcode;
}

/**
 * @brief UDP扩展轴使能
 * @param [in] axisID 轴号[1-4]
 * @param [in] status 0-去使能；1-使能
 * @return 错误码
 */
errno_t FRRobot::ExtAxisServoOn(int axisID, int status)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = axisID;
    param[1] = status;

    if (c.execute("ExtAxisServoOn", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute ExtAxisServoOn fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief UDP扩展轴运动
 * @param [in] pos 目标位置
 * @param [in] ovl 速度百分比
 * @return 错误码
 */
errno_t FRRobot::ExtAxisMove(ExaxisPos pos, double ovl)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = 0;  //异步
    param[1] = pos.ePos[0];
    param[2] = pos.ePos[1];
    param[3] = pos.ePos[2];
    param[4] = pos.ePos[3];
    param[5] = ovl;

    if (c.execute("ExtAxisMoveJ", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute ExtAxisMove fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置扩展DO
 * @param [in] DONum DO编号
 * @param [in] bOpen 开关 true-开；false-关
 * @param [in] smooth 是否平滑
 * @param [in] block 是否阻塞
 * @return 错误码
 */
errno_t FRRobot::SetAuxDO(int DONum, bool bOpen, bool smooth, bool block)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = DONum;
    param[1] = bOpen ? 1 : 0;
    param[2] = smooth ? 1:0;
    param[3] = block? 0:1;

    if (c.execute("SetAuxDO", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetAuxDO fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置扩展AO
 * @param [in] AONum AO编号
 * @param [in] value 模拟量值百分比
 * @param [in] block 是否阻塞
 * @return 错误码
 */
errno_t FRRobot::SetAuxAO(int AONum, double value, bool block)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = AONum;
    param[1] = value;
    param[2] = block;

    if (c.execute("SetAuxAO", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetAuxAO fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置扩展DI输入滤波时间
 * @param [in] filterTime 滤波时间(ms)
 * @return 错误码
 */
errno_t FRRobot::SetAuxDIFilterTime(int filterTime)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = filterTime;

    if (c.execute("SetAuxDIFilterTime", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetAuxDIFilterTime fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置扩展AI输入滤波时间
 * @param [in] AONum AO编号
 * @param [in] filterTime 滤波时间(ms)
 * @return 错误码
 */
errno_t FRRobot::SetAuxAIFilterTime(int AONum, int filterTime)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = AONum;
    param[1] = filterTime;

    if (c.execute("SetAuxAIFilterTime", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetAuxAIFilterTime fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 等待扩展DI输入
 * @param [in] DINum DI编号
 * @param [in] bOpen 开关 0-关；1-开
 * @param [in] time 最大等待时间(ms)
 * @param [in] errorAlarm 是否继续运动
 * @return 错误码
 */
errno_t FRRobot::WaitAuxDI(int DINum, bool bOpen, int time, bool errorAlarm)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = DINum;
    param[1] = bOpen;
    param[2] = time;
    param[3] = errorAlarm;

    if (c.execute("WaitAuxDI", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute WaitAuxDI fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 等待扩展AI输入
 * @param [in] AINum AI编号
 * @param [in] sign 0-大于；1-小于
 * @param [in] value AI值
 * @param [in] time 最大等待时间(ms)
 * @param [in] errorAlarm 是否继续运动
 * @return 错误码
 */
errno_t FRRobot::WaitAuxAI(int AINum, int sign, int value, int time, bool errorAlarm)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = AINum;
    param[1] = sign;
    param[2] = value;
    param[3] = time;
    param[4] = errorAlarm;

    if (c.execute("WaitAuxAI", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute WaitAuxAI fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 获取扩展DI值
 * @param [in] DINum DI编号
 * @param [in] isNoBlock 是否阻塞
 * @param [out] isOpen 0-关；1-开
 * @return 错误码
 */
errno_t FRRobot::GetAuxDI(int DINum, bool isNoBlock, bool& isOpen)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = DINum;
    param[1] = isNoBlock;

    if (c.execute("GetAuxDI", param, result))
    {
        errcode = int(result[0]);
        if (0 != errcode)
        {
            logger_error("execute GetAuxDI fail: %d.", errcode);
            c.close();
            return errcode;
        }
        else
        {
            isOpen = ((int)result[1] == 1) ? true:false;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 获取扩展AI值
 * @param [in] AINum AI编号
 * @param [in] isNoBlock 是否阻塞
 * @param [in] value 输入值
 * @return 错误码
 */
errno_t FRRobot::GetAuxAI(int AINum, bool isNoBlock, int& value)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = AINum;
    param[1] = isNoBlock;

    if (c.execute("GetAuxDI", param, result))
    {
        errcode = int(result[0]);
        if (0 != errcode)
        {
            logger_error("execute GetAuxDI fail: %d.", errcode);
            c.close();
            return errcode;
        }
        else
        {
            value = (int)(result[1]);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief UDP扩展轴通信异常断开后恢复连接
 * @return 错误码
 */
errno_t FRRobot::ExtDevUDPClientComReset()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ExtDevUDPClientComReset", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute ExtDevUDPClientComReset fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief UDP扩展轴通信异常断开后关闭通讯
 * @return 错误码
 */
errno_t FRRobot::ExtDevUDPClientComClose()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ExtDevUDPClientComClose", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute ExtDevUDPClientComClose fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}


/**
 * @brief UDP扩展轴参数配置
 * @param [in] axisType 扩展轴类型 0-平移；1-旋转
 * @param [in] axisDirection 扩展轴方向 0-正向；1-方向
 * @param [in] axisMax 扩展轴最大位置 mm
 * @param [in] axisMin 扩展轴最小位置 mm
 * @param [in] axisVel 速度mm/s
 * @param [in] axisAcc 加速度mm/s2
 * @param [in] axisLead 导程mm
 * @param [in] encResolution 编码器分辨率
 * @param [in] axisOffect焊缝起始点扩展轴偏移量
 * @param [in] axisCompany 驱动器厂家 1-禾川；2-汇川；3-松下
 * @param [in] axisModel 驱动器型号 1-禾川-SV-XD3EA040L-E，2-禾川-SV-X2EA150A-A，1-汇川-SV620PT5R4I，1-松下-MADLN15SG，2-松下-MSDLN25SG，3-松下-MCDLN35SG
 * @param [in] axisEncType 编码器类型  0-增量；1-绝对值
 * @return 错误码
 */
errno_t FRRobot::ExtAxisParamConfig(int axisID, int axisType, int axisDirection, double axisMax, double axisMin, double axisVel, double axisAcc, double axisLead, int encResolution, double axisOffect, int axisCompany, int axisModel, int axisEncType)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = axisID;
    param[1] = axisType;
    param[2] = axisDirection;
    param[3] = axisMax;
    param[4] = axisMin;
    param[5] = axisVel;
    param[6] = axisAcc;
    param[7] = axisLead;
    param[8] = encResolution;
    param[9] = axisOffect;
    param[10] = axisCompany;
    param[11] = axisModel;
    param[12] = axisEncType;

    if (c.execute("ExtAxisParamConfig", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute ExtAxisParamConfig fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 获取扩展轴驱动器配置信息
 * @param [in] axisId 轴号[1-4]
 * @param [out] axisCompany 驱动器厂家 1-禾川；2-汇川；3-松下
 * @param [out] axisModel 驱动器型号 1-禾川-SV-XD3EA040L-E，2-禾川-SV-X2EA150A-A，1-汇川-SV620PT5R4I，1-松下-MADLN15SG，2-松下-MSDLN25SG，3-松下-MCDLN35SG
 * @param [out] axisEncType 编码器类型  0-增量；1-绝对值
 * @return 错误码
 */
//errno_t FRRobot::GetExAxisDriverConfig(int axisId, int& axisCompany, int& axisModel, int& axisEncType)
//{
//    int errcode = 0;
//    XmlRpcClient c(serverUrl, 20003);
//    XmlRpcValue param, result;
//
//    param[0] = axisId;
//
//    if (c.execute("GetExAxisDriverConfig", param, result))
//    {
//        errcode = int(result[0]);
//        if (0 != errcode)
//        {
//            logger_error("execute GetExAxisDriverConfig fail: %d.", errcode);
//            c.close();
//            return errcode;
//        }
//        else
//        {
//            axisCompany = (int)(result[1]);
//            axisModel = (int)(result[2]);
//            axisEncType = (int)(result[3]);
//        }
//    }
//    else
//    {
//        c.close();
//        return ERR_XMLRPC_CMD_FAILED;
//    }
//
//    c.close();
//    return errcode;
//}

/**
 * @brief 设置扩展机器人相对扩展轴位置
 * @param [in] installType 0-机器人安装在外部轴上，1-机器人安装在外部轴外
 * @return 错误码
 */
errno_t FRRobot::SetRobotPosToAxis(int installType)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = installType;

    if (c.execute("SetRobotPosToAxis", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetRobotPosToAxis fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置扩展轴系统DH参数配置
 * @param [in]  axisConfig 外部轴构型，0-单自由度直线滑轨，1-两自由度L型变位机，2-三自由度，3-四自由度，4-单自由度变位机
 * @param [in]  axisDHd1 外部轴DH参数d1 mm
 * @param [in]  axisDHd2 外部轴DH参数d2 mm
 * @param [in]  axisDHd3 外部轴DH参数d3 mm
 * @param [in]  axisDHd4 外部轴DH参数d4 mm
 * @param [in]  axisDHa1 外部轴DH参数11 mm
 * @param [in]  axisDHa2 外部轴DH参数a2 mm
 * @param [in]  axisDHa3 外部轴DH参数a3 mm
 * @param [in]  axisDHa4 外部轴DH参数a4 mm
 * @return 错误码
 */
errno_t FRRobot::SetAxisDHParaConfig(int axisConfig, double axisDHd1, double axisDHd2, double axisDHd3, double axisDHd4, double axisDHa1, double axisDHa2, double axisDHa3, double axisDHa4)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = axisConfig;
    param[1] = axisDHd1;
    param[2] = axisDHd2;
    param[3] = axisDHd3;
    param[4] = axisDHd4;
    param[5] = axisDHa1;
    param[6] = axisDHa2;
    param[7] = axisDHa3;
    param[8] = axisDHa4;

    if (c.execute("SetAxisDHParaConfig", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetAxisDHParaConfig fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置扩展轴坐标系参考点-四点法
 * @param [in]  pointNum 点编号[1-4]
 * @return 错误码
 */
errno_t FRRobot::ExtAxisSetRefPoint(int pointNum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = pointNum;

    if (c.execute("ExtAxisSetRefPoint", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute ExtAxisSetRefPoint fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 计算扩展轴坐标系-四点法
 * @param [out]  coord 坐标系值
 * @return 错误码
 */
errno_t FRRobot::ExtAxisComputeECoordSys(DescPose& coord)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ExtAxisComputeECoordSys", param, result))
    {
        errcode = int(result[0]);
        if (0 != errcode)
        {
            logger_error("execute ExtAxisComputeECoordSys fail: %d.", errcode);
            c.close();
            return errcode;
        }
        else
        {
            coord.tran.x = (double)(result[1]);
            coord.tran.y = (double)(result[2]);
            coord.tran.z = (double)(result[3]);
            coord.rpy.rx = (double)(result[4]);
            coord.rpy.ry = (double)(result[5]);
            coord.rpy.rz = (double)(result[6]);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 应用扩展轴坐标系
 * @param [in]  axisCoordNum 坐标系编号
 * @param [in]  toolNum 工具号
 * @param [in]  coord 坐标系值
 * @param [in]  calibFlag 标定标志 0-否，1-是
 * @return 错误码
 */
errno_t FRRobot::ExtAxisActiveECoordSys(int axisCoordNum, int toolNum, DescPose coord, int calibFlag)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = axisCoordNum;
    param[1] = toolNum;
    param[2] = coord.tran.x;
    param[3] = coord.tran.y;
    param[4] = coord.tran.z;
    param[5] = coord.rpy.rx;
    param[6] = coord.rpy.ry;
    param[7] = coord.rpy.rz;
    param[8] = calibFlag;

    if (c.execute("ExtAxisActiveECoordSys", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute ExtAxisActiveECoordSys fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置标定参考点在变位机末端坐标系下位姿
 * @param [in] pos 位姿值
 * @return 错误码
 */
errno_t FRRobot::SetRefPointInExAxisEnd(DescPose pos)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = pos.tran.x;
    param[1] = pos.tran.y;
    param[2] = pos.tran.z;
    param[3] = pos.rpy.rx;
    param[4] = pos.rpy.ry;
    param[5] = pos.rpy.rz;

    if (c.execute("SetRefPointInExAxisEnd", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetRefPointInExAxisEnd fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 变位机坐标系参考点设置
 * @param [in]  pointNum 点编号[1-4]
 * @return 错误码
 */
errno_t FRRobot::PositionorSetRefPoint(int pointNum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = pointNum;

    if (c.execute("PositionorSetRefPoint", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute PositionorSetRefPoint fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 变位机坐标系计算-四点法
 * @param [out]  coord 坐标系值
 * @return 错误码
 */
errno_t FRRobot::PositionorComputeECoordSys(DescPose& coord)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("PositionorComputeECoordSys", param, result))
    {
        errcode = int(result[0]);
        if (0 != errcode)
        {
            logger_error("execute PositionorComputeECoordSys fail: %d.", errcode);
            c.close();
            return errcode;
        }
        else
        {
            coord.tran.x = (double)(result[1]);
            coord.tran.y = (double)(result[2]);
            coord.tran.z = (double)(result[3]);
            coord.rpy.rx = (double)(result[4]);
            coord.rpy.ry = (double)(result[5]);
            coord.rpy.rz = (double)(result[6]);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  UDP扩展轴与机器人关节运动同步运动
 * @param  [in] jopublic int_pos  目标关节位置,单位deg
 * @param  [in] desc_pos   目标笛卡尔位姿
 * @param  [in] tool  工具坐标号，范围[0~14]
 * @param  [in] user  工件坐标号，范围[0~14]
 * @param  [in] vel  速度百分比，范围[0~100]
 * @param  [in] acc  加速度百分比，范围[0~100],暂不开放
 * @param  [in] ovl  速度缩放因子，范围[0~100]
 * @param  [in] epos  扩展轴位置，单位mm
 * @param  [in] blendT [-1.0]-运动到位(阻塞)，[0~500.0]-平滑时间(非阻塞)，单位ms
 * @param  [in] offset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
 * @param  [in] offset_pos  位姿偏移量
 * @return  错误码
 */
errno_t FRRobot::ExtAxisSyncMoveJ(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, ExaxisPos epos, float blendT, uint8_t offset_flag, DescPose offset_pos)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = 1;  //同步
    param[1] = epos.ePos[0];
    param[2] = epos.ePos[1];
    param[3] = epos.ePos[2];
    param[4] = epos.ePos[3];
    param[5] = ovl;

    if (c.execute("ExtAxisMoveJ", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute ExtAxisMoveJ fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    errcode = MoveJ(&joint_pos, &desc_pos, tool, user, vel, acc, ovl, &epos, blendT, offset_flag, &offset_pos);
    return errcode;
}

/**
 * @brief  UDP扩展轴与机器人直线运动同步运动
 * @param  [in] joint_pos  目标关节位置,单位deg
 * @param  [in] desc_pos   目标笛卡尔位姿
 * @param  [in] tool  工具坐标号，范围[0~14]
 * @param  [in] user  工件坐标号，范围[0~14]
 * @param  [in] vel  速度百分比，范围[0~100]
 * @param  [in] acc  加速度百分比，范围[0~100],暂不开放
 * @param  [in] ovl  速度缩放因子，范围[0~100]
 * @param  [in] blendR [-1.0]-运动到位(阻塞)，[0~1000.0]-平滑半径(非阻塞)，单位mm
 * @param  [in] epos  扩展轴位置，单位mm
 * @param  [in] offset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
 * @param  [in] offset_pos  位姿偏移量
 * @return  错误码
 */
errno_t FRRobot::ExtAxisSyncMoveL(JointPos joint_pos, DescPose desc_pos, int tool, int user, float vel, float acc, float ovl, float blendR, ExaxisPos epos, uint8_t offset_flag, DescPose offset_pos)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = 1;  //同步
    param[1] = epos.ePos[0];
    param[2] = epos.ePos[1];
    param[3] = epos.ePos[2];
    param[4] = epos.ePos[3];
    param[5] = ovl;

    if (c.execute("ExtAxisMoveJ", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute ExtAxisMove fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    errcode = MoveL(&joint_pos, &desc_pos, tool, user, vel, acc, ovl, blendR, 0, &epos, 0, offset_flag, &offset_pos);
    return errcode;
}

/**
 * @brief  UDP扩展轴与机器人圆弧运动同步运动
 * @param  [in] joint_pos_p  路径点关节位置,单位deg
 * @param  [in] desc_pos_p   路径点笛卡尔位姿
 * @param  [in] ptool  工具坐标号，范围[0~14]
 * @param  [in] puser  工件坐标号，范围[0~14]
 * @param  [in] pvel  速度百分比，范围[0~100]
 * @param  [in] pacc  加速度百分比，范围[0~100],暂不开放
 * @param  [in] epos_p  扩展轴位置，单位mm
 * @param  [in] poffset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
 * @param  [in] offset_pos_p  位姿偏移量
 * @param  [in] joint_pos_t  目标点关节位置,单位deg
 * @param  [in] desc_pos_t   目标点笛卡尔位姿
 * @param  [in] ttool  工具坐标号，范围[0~14]
 * @param  [in] tuser  工件坐标号，范围[0~14]
 * @param  [in] tvel  速度百分比，范围[0~100]
 * @param  [in] tacc  加速度百分比，范围[0~100],暂不开放
 * @param  [in] epos_t  扩展轴位置，单位mm
 * @param  [in] toffset_flag  0-不偏移，1-基坐标系/工件坐标系下偏移，2-工具坐标系下偏移
 * @param  [in] offset_pos_t  位姿偏移量
 * @param  [in] ovl  速度缩放因子，范围[0~100]
 * @param  [in] blendR [-1.0]-运动到位(阻塞)，[0~1000.0]-平滑半径(非阻塞)，单位mm
 * @return  错误码
 */
errno_t FRRobot::ExtAxisSyncMoveC(JointPos joint_pos_p, DescPose desc_pos_p, int ptool, int puser, float pvel, float pacc, ExaxisPos epos_p, uint8_t poffset_flag, DescPose offset_pos_p, JointPos joint_pos_t, DescPose desc_pos_t, int ttool, int tuser, float tvel, float tacc, ExaxisPos epos_t, uint8_t toffset_flag, DescPose offset_pos_t, float ovl, float blendR)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = 1;  //同步
    param[1] = epos_t.ePos[0];
    param[2] = epos_t.ePos[1];
    param[3] = epos_t.ePos[2];
    param[4] = epos_t.ePos[3];
    param[5] = ovl;

    if (c.execute("ExtAxisMoveJ", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute ExtAxisMove fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    errcode = MoveC(&joint_pos_p, &desc_pos_p, ptool, puser, pvel, pacc, &epos_p, poffset_flag, &offset_pos_p, &joint_pos_t, &desc_pos_t, ttool, tuser, tvel, tacc, &epos_t, toffset_flag, &offset_pos_t, ovl, blendR);
    return errcode;
}

/**
* @brief  焊丝寻位开始
* @param  [in] refPos  1-基准点 2-接触点
* @param  [in] searchVel   寻位速度 %
* @param  [in] searchDis  寻位距离 mm
* @param  [in] autoBackFlag 自动返回标志，0-不自动；-自动
* @param  [in] autoBackVel  自动返回速度 %
* @param  [in] autoBackDis  自动返回距离 mm
* @param  [in] offectFlag  1-带偏移量寻位；2-示教点寻位
* @return  错误码
*/
errno_t FRRobot::WireSearchStart(int refPos, float searchVel, int searchDis, int autoBackFlag, float autoBackVel, int autoBackDis, int offectFlag)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = refPos;
    param[1] = searchVel;
    param[2] = searchDis;
    param[3] = autoBackFlag;
    param[4] = autoBackVel;
    param[5] = autoBackDis;
    param[6] = offectFlag;


    if (c.execute("WireSearchStart", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute WireSearchStart fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  焊丝寻位结束
 * @param  [in] refPos  1-基准点 2-接触点
 * @param  [in] searchVel   寻位速度 %
 * @param  [in] searchDis  寻位距离 mm
 * @param  [in] autoBackFlag 自动返回标志，0-不自动；-自动
 * @param  [in] autoBackVel  自动返回速度 %
 * @param  [in] autoBackDis  自动返回距离 mm
 * @param  [in] offectFlag  1-带偏移量寻位；2-示教点寻位
 * @return  错误码
 */
errno_t FRRobot::WireSearchEnd(int refPos, float searchVel, int searchDis, int autoBackFlag, float autoBackVel, int autoBackDis, int offectFlag)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = refPos;
    param[1] = searchVel;
    param[2] = searchDis;
    param[3] = autoBackFlag;
    param[4] = autoBackVel;
    param[5] = autoBackDis;
    param[6] = offectFlag;


    if (c.execute("WireSearchEnd", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute WireSearchEnd fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  计算焊丝寻位偏移量
 * @param  [in] seamType  焊缝类型
 * @param  [in] method   计算方法
 * @param  [in] varNameRef 基准点1-6，“#”表示无点变量
 * @param  [in] varNameRes 接触点1-6，“#”表示无点变量
 * @param  [out] offectFlag 0-偏移量直接叠加到指令点；1-偏移量需要对指令点进行坐标变换
 * @param  [out] offect 偏移位姿[x, y, z, a, b, c]
 * @return  错误码
 */
errno_t FRRobot::GetWireSearchOffset(int seamType, int method, vector<string> varNameRef, vector<string> varNameRes, int& offectFlag, DescPose& offect)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;
    param[0] = seamType;
    param[1] = method;
    for (int i = 0; i < 6; i++)
    {
        param[2 + i] = varNameRef.at(i);
        param[8 + i] = varNameRes.at(i);
    }

    if (c.execute("GetWireSearchOffset", param, result))
    {
        errcode = int(result[0]);
        if (0 != errcode)
        {
            logger_error("execute GetWireSearchOffset fail: %d.", errcode);
            c.close();
            return errcode;
        }
        else
        {
            offectFlag = (int)result[1];
            offect.tran.x = (double)result[2];
            offect.tran.y = (double)result[3];
            offect.tran.z = (double)result[4];
            offect.rpy.rx = (double)result[5];
            offect.rpy.ry = (double)result[6];
            offect.rpy.rz = (double)result[7];
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  等待焊丝寻位完成
 * @return  错误码
 */
errno_t FRRobot::WireSearchWait(string varName)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;
    param[0] = varName;

    if (c.execute("WireSearchWait", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute WireSearchWait fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  焊丝寻位接触点写入数据库
 * @param  [in] varName  接触点名称 “RES0” ~ “RES99”
 * @param  [in] pos  接触点数据[x, y, x, a, b, c]
 * @return  错误码
 */
errno_t FRRobot::SetPointToDatabase(string varName, DescPose pos)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = varName;
    param[1][0] = pos.tran.x;
    param[1][1] = pos.tran.y;
    param[1][2] = pos.tran.z;
    param[1][3] = pos.rpy.rx;
    param[1][4] = pos.rpy.ry;
    param[1][5] = pos.rpy.rz;

    if (c.execute("SetPointToDatabase", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetPointToDatabase fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  电弧跟踪控制
 * @param  [in] flag 开关，0-关；1-开
 * @param  [in] dalayTime 滞后时间，单位ms
 * @param  [in] isLeftRight 左右偏差补偿
 * @param  [in] klr 左右调节系数(灵敏度)
 * @param  [in] tStartLr 左右开始补偿时间cyc
 * @param  [in] stepMaxLr 左右每次最大补偿量 mm
 * @param  [in] sumMaxLr 左右总计最大补偿量 mm
 * @param  [in] isUpLow 上下偏差补偿
 * @param  [in] kud 上下调节系数(灵敏度)
 * @param  [in] tStartUd 上下开始补偿时间cyc
 * @param  [in] stepMaxUd 上下每次最大补偿量 mm
 * @param  [in] sumMaxUd 上下总计最大补偿量
 * @param  [in] axisSelect 上下坐标系选择，0-摆动；1-工具；2-基座
 * @param  [in] referenceType 上下基准电流设定方式，0-反馈；1-常数
 * @param  [in] referSampleStartUd 上下基准电流采样开始计数(反馈)，cyc
 * @param  [in] referSampleCountUd 上下基准电流采样循环计数(反馈)，cyc
 * @param  [in] referenceCurrent 上下基准电流mA
 * @param  [in] offsetType 偏置跟踪类型，0-不偏置；1-采样；2-百分比
 * @param  [in] offsetParameter 偏置参数；采样(偏置采样开始时间，默认采一周期)；百分比(偏置百分比(-100 ~ 100))
 * @return  错误码
 */
errno_t FRRobot::ArcWeldTraceControl(int flag, double delaytime, int isLeftRight, double klr, double tStartLr, double stepMaxLr, double sumMaxLr, int isUpLow, double kud, double tStartUd, double stepMaxUd, double sumMaxUd, int axisSelect, int referenceType, double referSampleStartUd, double referSampleCountUd, double referenceCurrent, int offsetType, int offsetParameter)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = flag;
    param[1] = delaytime;
    param[2] = isLeftRight;
    param[3][0] = klr;
    param[3][1] = tStartLr;
    param[3][2] = stepMaxLr;
    param[3][3] = sumMaxLr;
    param[4] = isUpLow;
    param[5][0] = kud;
    param[5][1] = tStartUd;
    param[5][2] = stepMaxUd;
    param[5][3] = sumMaxUd;
    param[6] = axisSelect;
    param[7] = referenceType;
    param[8] = referSampleStartUd;
    param[9] = referSampleCountUd;
    param[10] = referenceCurrent;
    param[11] = offsetType;
    param[12] = offsetParameter;


    if (c.execute("ArcWeldTraceControl", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute ArcWeldTraceControl fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  焊丝寻位接触点写入数据库
 * @param  [in] channel 电弧跟踪AI通带选择,[0-3]
 * @return  错误码
 */
errno_t FRRobot::ArcWeldTraceExtAIChannelConfig(int channel)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = channel;

    if (c.execute("ArcWeldTraceExtAIChannelConfig", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute ArcWeldTraceExtAIChannelConfig fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  力传感器辅助拖动
 * @param  [in] status 控制状态，0-关闭；1-开启
 * @param  [in] asaptiveFlag 自适应开启标志，0-关闭；1-开启
 * @param  [in] interfereDragFlag 干涉区拖动标志，0-关闭；1-开启
 * @param  [in] ingularityConstraintsFlag 奇异点策略，0-规避；1-穿越
 * @param  [in] M 惯性系数
 * @param  [in] B 阻尼系数
 * @param  [in] K 刚度系数
 * @param  [in] F 拖动六维力阈值
 * @param  [in] Fmax 最大拖动力限制
 * @param  [in] Vmax 最大关节速度限制
 * @return  错误码
 */
errno_t FRRobot::EndForceDragControl(int status, int asaptiveFlag, int interfereDragFlag, int ingularityConstraintsFlag, vector<double> M, vector<double> B, vector<double> K, vector<double> F, double Fmax, double Vmax)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = status;
    param[1] = asaptiveFlag;
    param[2] = interfereDragFlag;
    param[3] = ingularityConstraintsFlag;
    for (int i = 0; i < 6; i++)
    {
        param[4][i] = M.at(i);
        param[5][i] = B.at(i);
        param[6][i] = K.at(i);
        param[7][i] = F.at(i);
    }
    param[8] = Fmax;
    param[9] = Vmax;

    if (c.execute("EndForceDragControl", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute EndForceDragControl fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  力传感器辅助拖动
 * @param  [in] status 控制状态，0-关闭；1-开启
 * @param  [in] asaptiveFlag 自适应开启标志，0-关闭；1-开启
 * @param  [in] interfereDragFlag 干涉区拖动标志，0-关闭；1-开启
 * @param  [in] ingularityConstraintsFlag 奇异点策略，0-规避；1-穿越
 * @param  [in] forceCollisionFlag 辅助拖动时机器人碰撞检测标志；0-关闭；1-开启
 * @param  [in] M 惯性系数
 * @param  [in] B 阻尼系数
 * @param  [in] K 刚度系数
 * @param  [in] F 拖动六维力阈值
 * @param  [in] Fmax 最大拖动力限制
 * @param  [in] Vmax 最大关节速度限制
 * @return  错误码
 */
errno_t FRRobot::EndForceDragControl(int status, int asaptiveFlag, int interfereDragFlag, int ingularityConstraintsFlag, int forceCollisionFlag, vector<double> M, vector<double> B, vector<double> K, vector<double> F, double Fmax, double Vmax)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = status;
    param[1] = asaptiveFlag;
    param[2] = interfereDragFlag;
    param[3] = ingularityConstraintsFlag;
    param[4] = forceCollisionFlag;
    for (int i = 0; i < 6; i++)
    {
        param[5][i] = M.at(i);
        param[6][i] = B.at(i);
        param[7][i] = K.at(i);
        param[8][i] = F.at(i);
    }
    param[9] = Fmax;
    param[10] = Vmax;

    if (c.execute("EndForceDragControl", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute EndForceDragControl fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  报错清除后力传感器自动开启
 * @param  [in] status 控制状态，0-关闭；1-开启
 * @return  错误码
 */
errno_t FRRobot::SetForceSensorDragAutoFlag(int status)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = status;

    if (c.execute("SetForceSensorDragAutoFlag", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetForceSensorDragAutoFlag fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  设置六维力和关节阻抗混合拖动开关及参数
 * @param  [in] status 控制状态，0-关闭；1-开启
 * @param  [in] impedanceFlag 阻抗开启标志，0-关闭；1-开启
 * @param  [in] lamdeDain 拖动增益
 * @param  [in] KGain 刚度增益
 * @param  [in] BGain 阻尼增益
 * @param  [in] dragMaxTcpVel 拖动末端最大线速度限制
 * @param  [in] dragMaxTcpOriVel 拖动末端最大角速度限制
 * @return  错误码
 */
errno_t FRRobot::ForceAndJointImpedanceStartStop(int status, int impedanceFlag, vector<double> lamdeDain, vector<double> KGain, vector<double> BGain, double dragMaxTcpVel, double dragMaxTcpOriVel)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = status;
    param[1] = impedanceFlag;
    for (int i = 0; i < 6; i++)
    {
        param[2][i] = lamdeDain.at(i);
        param[3][i] = KGain.at(i);
        param[4][i] = BGain.at(i);
    }
    param[5] = dragMaxTcpVel;
    param[6] = dragMaxTcpOriVel;

    if (c.execute("ForceAndJointImpedanceStartStop", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute ForceAndJointImpedanceStartStop fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  获取力传感器拖动开关状态
 * @param  [out] dragState 力传感器辅助拖动控制状态，0-关闭；1-开启
 * @param  [out] sixDimensionalDragState 六维力辅助拖动控制状态，0-关闭；1-开启
 * @return  错误码
 */
errno_t FRRobot::GetForceAndTorqueDragState(int& dragState, int& sixDimensionalDragState)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetForceAndTorqueDragState", param, result))
    {
        errcode = int(result[0]);
        if (0 != errcode)
        {
            logger_error("execute GetForceAndTorqueDragState fail: %d.", errcode);
            c.close();
            return errcode;
        }
        else
        {
            dragState = (int)result[1];
            sixDimensionalDragState = (int)result[2];
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  设置力传感器下负载重量
 * @param  [in] weight 负载重量 kg
 * @return  错误码
 */
errno_t FRRobot::SetForceSensorPayload(double weight)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = weight;

    if (c.execute("SetForceSensorPayload", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetForceSensorPayload fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  设置力传感器下负载质心
 * @param  [in] x 负载质心x mm
 * @param  [in] y 负载质心y mm
 * @param  [in] z 负载质心z mm
 * @return  错误码
 */
errno_t FRRobot::SetForceSensorPayloadCog(double x, double y, double z)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = x;
    param[1] = y;
    param[2] = z;

    if (c.execute("SetForceSensorPayloadCog", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetForceSensorPayloadCog fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  获取力传感器下负载重量
 * @param  [in] weight 负载重量 kg
 * @return  错误码
 */
errno_t FRRobot::GetForceSensorPayload(double& weight)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetForceSensorPayload", param, result))
    {
        errcode = int(result[0]);
        if (0 != errcode)
        {
            logger_error("execute GetForceSensorPayload fail: %d.", errcode);
            c.close();
            return errcode;
        }
        else
        {
            weight = (double)result[1];
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  获取力传感器下负载质心
 * @param  [out] x 负载质心x mm
 * @param  [out] y 负载质心y mm
 * @param  [out] z 负载质心z mm
 * @return  错误码
 */
errno_t FRRobot::GetForceSensorPayloadCog(double& x, double& y, double& z)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetForceSensorPayloadCog", param, result))
    {
        errcode = int(result[0]);
        if (0 != errcode)
        {
            logger_error("execute GetForceSensorPayloadCog fail: %d.", errcode);
            c.close();
            return errcode;
        }
        else
        {
            x = (double)result[1];
            y = (double)result[2];
            z = (double)result[3];
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  力传感器自动校零
 * @param  [out] weight 传感器质量 kg
 * @param  [out] pos 传感器质心 mm
 * @return  错误码
 */
errno_t FRRobot::ForceSensorAutoComputeLoad(double& weight, DescTran& pos)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    JointPos startJ = { };
    DescPose startDesc = {};
    GetActualJointPosDegree(1, &startJ);
    GetActualTCPPose(1, &startDesc);

    JointPos tmpJPos = {};
    DescPose tmpDescPos = {};
    DescPose offectPos = {};
    ExaxisPos tmpExaxisPos = {};

    ForceSensorSetSaveDataFlag(1);

    GetActualJointPosDegree(1, &tmpJPos);
    if (tmpJPos.jPos[2] < 0)
    {
        tmpJPos.jPos[3] += 90;
        GetForwardKin(&tmpJPos, &tmpDescPos);
    }
    else
    {
        tmpJPos.jPos[3] -= 90;
        GetForwardKin(&tmpJPos, &tmpDescPos);
    }
    MoveJ(&tmpJPos, &tmpDescPos, 0, 0, 100, 100, 100, &tmpExaxisPos, -1, 0, &offectPos);

    ForceSensorSetSaveDataFlag(2);

    GetActualJointPosDegree(1, &tmpJPos);
    if (tmpJPos.jPos[5] < 0)
    {
        tmpJPos.jPos[5] += 90;
        GetForwardKin(&tmpJPos, &tmpDescPos);
    }
    else
    {
        tmpJPos.jPos[5] -= 90;
        GetForwardKin(&tmpJPos, &tmpDescPos);
    }
    MoveJ(&tmpJPos, &tmpDescPos, 0, 0, 100, 100, 100, &tmpExaxisPos, -1, 0, &offectPos);

    ForceSensorSetSaveDataFlag(3);

    ForceSensorComputeLoad(weight, pos);
    WaitMs(100);
    MoveJ(&startJ, &startDesc, 0, 0, 100, 100, 100, &tmpExaxisPos, -1, 0, &offectPos);
    return 0;
}

/**
 * @brief  传感器自动校零数据记录
 * @param  [in] recordCount 记录数据个数 1-3
 * @return  错误码
 */
errno_t FRRobot::ForceSensorSetSaveDataFlag(int recordCount)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = recordCount;

    if (c.execute("ForceSensorSetSaveDataFlag", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute ForceSensorSetSaveDataFlag fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  传感器自动校零计算
 * @param  [out] weight 传感器质量 kg
 * @param  [out] pos 传感器质心 [x, y, z]
 * @return  错误码
 */
errno_t FRRobot::ForceSensorComputeLoad(double& weight, DescTran& pos)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ForceSensorComputeLoad", param, result))
    {
        errcode = int(result[0]);
        if (0 != errcode)
        {
            logger_error("execute ForceSensorComputeLoad fail: %d.", errcode);
            c.close();
            return errcode;
        }
        else
        {
            weight = (double)result[1];
            printf("the wejght is %f\n", weight);
            pos.x = (double)result[2];
            pos.y = (double)result[3];
            pos.z = (double)result[4];
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  段焊获取位置和姿态
 * @param  [in] startPos 起始点坐标
 * @param  [in] endPos 终止点坐标
 * @param  [in] startDistance 焊接点至起点的长度
 * @param  [out] weldPointDesc 焊接点的笛卡尔坐标信息
 * @param  [out] weldPointJoint 焊接点的笛卡尔坐标信息
 * @param  [out] tool 工具号
 * @param  [out] user 工件号
 * @return  错误码
 */
errno_t FRRobot::GetSegmentWeldPoint(DescPose startPos, DescPose endPos, double startDistance, DescPose& weldPointDesc, JointPos& weldPointJoint, int& tool, int& user)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0][0] = startPos.tran.x;
    param[0][1] = startPos.tran.y;
    param[0][2] = startPos.tran.z;
    param[0][3] = startPos.rpy.rx;
    param[0][4] = startPos.rpy.ry;
    param[0][5] = startPos.rpy.rz;

    param[1][0] = endPos.tran.x;
    param[1][1] = endPos.tran.y;
    param[1][2] = endPos.tran.z;
    param[1][3] = endPos.rpy.rx;
    param[1][4] = endPos.rpy.ry;
    param[1][5] = endPos.rpy.rz;
    param[2] = startDistance;

    if (c.execute("GetSegmentWeldPoint", param, result))
    {
        errcode = int(result[0]);
        if (0 != errcode)
        {
            logger_error("execute GetSegmentWeldPoint fail: %d.", errcode);
            c.close();
            return errcode;
        }
        else
        {
            string paramStr = (string)result[1];
            std::vector<std::string> parS = split(paramStr, ',');
            if (parS.size() != 14)
            {
                logger_error("get segment weld point fail");
                return -1;
            }

            weldPointJoint.jPos[0] = stod(parS[0]);
            weldPointJoint.jPos[1] = stod(parS[1]);
            weldPointJoint.jPos[2] = stod(parS[2]);
            weldPointJoint.jPos[3] = stod(parS[3]);
            weldPointJoint.jPos[4] = stod(parS[4]);
            weldPointJoint.jPos[5] = stod(parS[5]);

            weldPointDesc.tran.x = stod(parS[6]);
            weldPointDesc.tran.y = stod(parS[7]);
            weldPointDesc.tran.z = stod(parS[8]);
            weldPointDesc.rpy.rx = stod(parS[9]);
            weldPointDesc.rpy.ry = stod(parS[10]);
            weldPointDesc.rpy.rz = stod(parS[11]);

            tool = stoi(parS[12]);
            user = stoi(parS[13]);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  设置焊接工艺曲线参数
 * @param  [in] id 焊接工艺编号(1-99)
 * @param  [in] startCurrent 起弧电流(A)
 * @param  [in] startVoltage 起弧电压(V)
 * @param  [in] startTime 起弧时间(ms)
 * @param  [in] weldCurrent 焊接电流(A)
 * @param  [in] weldVoltage 焊接电压(V)
 * @param  [in] endCurrent 收弧电流(A)
 * @param  [in] endVoltage 收弧电压(V)
 * @param  [in] endTime 收弧时间(ms)
 * @return  错误码
 */
errno_t FRRobot::WeldingSetProcessParam(int id, double startCurrent, double startVoltage, double startTime, double weldCurrent, double weldVoltage, double endCurrent, double endVoltage, double endTime)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    param[1] = startCurrent;
    param[2] = startVoltage;
    param[3] = startTime;
    param[4] = weldCurrent;
    param[5] = weldVoltage;
    param[6] = endCurrent;
    param[7] = endVoltage;
    param[8] = endTime;

    if (c.execute("WeldingSetProcessParam", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute WeldingSetProcessParam fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  获取焊接工艺曲线参数
 * @param  [in] id 焊接工艺编号(1-99)
 * @param  [out] startCurrent 起弧电流(A)
 * @param  [out] startVoltage 起弧电压(V)
 * @param  [out] startTime 起弧时间(ms)
 * @param  [out] weldCurrent 焊接电流(A)
 * @param  [out] weldVoltage 焊接电压(V)
 * @param  [out] endCurrent 收弧电流(A)
 * @param  [out] endVoltage 收弧电压(V)
 * @param  [out] endTime 收弧时间(ms)
 * @return  错误码
 */
errno_t FRRobot::WeldingGetProcessParam(int id, double& startCurrent, double& startVoltage, double& startTime, double& weldCurrent, double& weldVoltage, double& endCurrent, double& endVoltage, double& endTime)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;

    if (c.execute("WeldingGetProcessParam", param, result))
    {
        errcode = int(result[0]);
        if (0 != errcode)
        {
            logger_error("execute WeldingGetProcessParam fail: %d.", errcode);
            c.close();
            return errcode;
        }
        else
        {
            startCurrent = (double)result[1];
            startVoltage = (double)result[2];
            startTime = (double)result[3];
            weldCurrent = (double)result[4];
            weldVoltage = (double)result[5];
            endCurrent = (double)result[6];
            endVoltage = (double)result[7];
            endTime = (double)result[8];
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  末端传感器配置
 * @param  [in] idCompany 厂商，18-JUNKONG；25-HUIDE
 * @param  [in] idDevice 类型，0-JUNKONG/RYR6T.V1.0
 * @param  [in] idSoftware 软件版本，0-J1.0/HuiDe1.0(暂未开放)
 * @param  [in] idBus 挂载位置，1-末端1号口；2-末端2号口...8-末端8号口(暂未开放)
 * @return  错误码
 */
errno_t FRRobot::AxleSensorConfig(int idCompany, int idDevice, int idSoftware, int idBus)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = idCompany;
    param[1] = idDevice;
    param[2] = idSoftware;
    param[3] = idBus;

    if (c.execute("AxleSensorConfig", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute AxleSensorConfig fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  获取末端传感器配置
 * @param  [out] idCompany 厂商，18-JUNKONG；25-HUIDE
 * @param  [out] idDevice 类型，0-JUNKONG/RYR6T.V1.0
 * @return  错误码
 */
errno_t FRRobot::AxleSensorConfigGet(int& idCompany, int& idDevice)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("AxleSensorConfigGet", param, result))
    {
        errcode = int(result[0]);
        if (0 != errcode)
        {
            logger_error("execute AxleSensorConfigGet fail: %d.", errcode);
            c.close();
            return errcode;
        }
        else
        {
            idCompany = (int)result[1];
            idDevice = (int)result[2];
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  末端传感器激活
 * @param  [in] actFlag 0-复位；1-激活
 * @return  错误码
 */
errno_t FRRobot::AxleSensorActivate(int actFlag)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = actFlag;

    if (c.execute("AxleSensorActivate", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute AxleSensorActivate fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  末端传感器寄存器写入
 * @param  [in] devAddr  设备地址编号 0-255
 * @param  [in] regHAddr 寄存器地址高8位
 * @param  [in] regLAddr 寄存器地址低8位
 * @param  [in] regNum  寄存器个数 0-255
 * @param  [in] data1 写入寄存器数值1
 * @param  [in] data2 写入寄存器数值2
 * @param  [in] isNoBlock 0-阻塞；1-非阻塞
 * @return  错误码
 */
errno_t FRRobot::AxleSensorRegWrite(int devAddr, int regHAddr, int regLAddr, int regNum, int data1, int data2, int isNoBlock)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = devAddr;
    param[1] = regHAddr;
    param[2] = regLAddr;
    param[3] = regNum;
    param[4] = data1;
    param[5] = data2;
    param[6] = isNoBlock;

    if (c.execute("AxleSensorRegWrite", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute AxleSensorRegWrite fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  设置控制箱DO停止/暂停后输出是否复位
 * @param  [in] resetFlag  0-不复位；1-复位
 * @return  错误码
 */
errno_t FRRobot::SetOutputResetCtlBoxDO(int resetFlag)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = resetFlag;

    if (c.execute("SetOutputResetCtlBoxDO", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetOutputResetCtlBoxDO fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  设置控制箱AO停止/暂停后输出是否复位
 * @param  [in] resetFlag  0-不复位；1-复位
 * @return  错误码
 */
errno_t FRRobot::SetOutputResetCtlBoxAO(int resetFlag)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = resetFlag;

    if (c.execute("SetOutputResetCtlBoxAO", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetOutputResetCtlBoxAO fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  设置末端工具DO停止/暂停后输出是否复位
 * @param  [in] resetFlag  0-不复位；1-复位
 * @return  错误码
 */
errno_t FRRobot::SetOutputResetAxleDO(int resetFlag)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = resetFlag;

    if (c.execute("SetOutputResetAxleDO", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetOutputResetAxleDO fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  设置末端工具AO停止/暂停后输出是否复位
 * @param  [in] resetFlag  0-不复位；1-复位
 * @return  错误码
 */
errno_t FRRobot::SetOutputResetAxleAO(int resetFlag)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = resetFlag;

    if (c.execute("SetOutputResetAxleAO", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetOutputResetAxleAO fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  设置扩展DO停止/暂停后输出是否复位
 * @param  [in] resetFlag  0-不复位；1-复位
 * @return  错误码
 */
errno_t FRRobot::SetOutputResetExtDO(int resetFlag)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = resetFlag;

    if (c.execute("SetOutputResetExtDO", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetOutputResetExtDO fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}


/**
 * @brief  设置扩展AO停止/暂停后输出是否复位
 * @param  [in] resetFlag  0-不复位；1-复位
 * @return  错误码
 */
errno_t FRRobot::SetOutputResetExtAO(int resetFlag)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = resetFlag;

    if (c.execute("SetOutputResetExtAO", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetOutputResetExtAO fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  设置SmartTool停止/暂停后输出是否复位
 * @param  [in] resetFlag  0-不复位；1-复位
 * @return  错误码
 */
errno_t FRRobot::SetOutputResetSmartToolDO(int resetFlag)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = resetFlag;

    if (c.execute("SetOutputResetSmartToolDO", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetOutputResetSmartToolDO fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  仿真摆动开始
 * @param  [in] weaveNum  摆动参数编号
 * @return  错误码
 */
errno_t FRRobot::WeaveStartSim(int weaveNum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = weaveNum;

    if (c.execute("WeaveStartSim", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute WeaveStartSim fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  仿真摆动结束
 * @param  [in] weaveNum  摆动参数编号
 * @return  错误码
 */
errno_t FRRobot::WeaveEndSim(int weaveNum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = weaveNum;

    if (c.execute("WeaveEndSim", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute WeaveEndSim fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  开始轨迹检测预警(不运动)
 * @param  [in] weaveNum   摆动参数编号
 * @return  错误码
 */
errno_t FRRobot::WeaveInspectStart(int weaveNum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = weaveNum;

    if (c.execute("WeaveInspectStart", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute WeaveInspectStart fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 结束轨迹检测预警(不运动)
 * @param  [in] weaveNum   摆动参数编号
 * @return  错误码
 */
errno_t FRRobot::WeaveInspectEnd(int weaveNum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = weaveNum;

    if (c.execute("WeaveInspectEnd", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute WeaveInspectEnd fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 扩展IO-配置焊机气体检测信号
 * @param  [in] DONum  气体检测信号扩展DO编号
 * @return  错误码
 */
errno_t FRRobot::SetAirControlExtDoNum(int DONum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = DONum;

    if (c.execute("SetAirControlExtDoNum", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetAirControlExtDoNum fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 扩展IO-配置焊机起弧信号
 * @param  [in] DONum  焊机起弧信号扩展DO编号
 * @return  错误码
 */
errno_t FRRobot::SetArcStartExtDoNum(int DONum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = DONum;

    if (c.execute("SetArcStartExtDoNum", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetArcStartExtDoNum fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 扩展IO-配置焊机反向送丝信号
 * @param  [in] DONum  反向送丝信号扩展DO编号
 * @return  错误码
 */
errno_t FRRobot::SetWireReverseFeedExtDoNum(int DONum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = DONum;

    if (c.execute("SetWireReverseFeedExtDoNum", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetWireReverseFeedExtDoNum fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 扩展IO-配置焊机正向送丝信号
 * @param  [in] DONum  正向送丝信号扩展DO编号
 * @return  错误码
 */
errno_t FRRobot::SetWireForwardFeedExtDoNum(int DONum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = DONum;

    if (c.execute("SetWireForwardFeedExtDoNum", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetWireForwardFeedExtDoNum fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 扩展IO-配置焊机起弧成功信号
 * @param  [in] DINum  起弧成功信号扩展DI编号
 * @return  错误码
 */
errno_t FRRobot::SetArcDoneExtDiNum(int DINum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = DINum;

    if (c.execute("SetArcDoneExtDiNum", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetArcDoneExtDiNum fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 扩展IO-配置焊机准备信号
 * @param  [in] DINum  焊机准备信号扩展DI编号
 * @return  错误码
 */
errno_t FRRobot::SetWeldReadyExtDiNum(int DINum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = DINum;

    if (c.execute("SetWeldReadyExtDiNum", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetWeldReadyExtDiNum fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 扩展IO-配置焊接中断恢复信号
 * @param  [in] reWeldDINum  焊接中断后恢复焊接信号扩展DI编号
 * @param  [in] abortWeldDINum  焊接中断后退出焊接信号扩展DI编号
 * @return  错误码
 */
errno_t FRRobot::SetExtDIWeldBreakOffRecover(int reWeldDINum, int abortWeldDINum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = reWeldDINum;
    param[1] = abortWeldDINum;

    if (c.execute("SetExtDIWeldBreakOffRecover", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetExtDIWeldBreakOffRecover fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置机器人碰撞检测方法
 * @param [in] method 碰撞检测方法：0-电流模式；1-双编码器；2-电流和双编码器同时开启
 * @param [in] thresholdMode 碰撞等级阈值方式；0-碰撞等级固定阈值方式；1-自定义碰撞检测阈值
 * @return 错误码
 */
errno_t FRRobot::SetCollisionDetectionMethod(int method, int thresholdMode)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = method;
    param[1] = thresholdMode;

    if (c.execute("SetCollisionDetectionMethod", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetCollisionDetectionMethod fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置静态下碰撞检测开始关闭
 * @param  [in] status 0-关闭；1-开启
 * @return  错误码
 */
errno_t FRRobot::SetStaticCollisionOnOff(int status)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = status;

    if (c.execute("SetStaticCollisionOnOff", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetStaticCollisionOnOff fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 关节扭矩功率检测
 * @param  [in] status 0-关闭；1-开启
 * @param  [in] power 设定最大功率(W)
 * @return  错误码
 */
errno_t FRRobot::SetPowerLimit(int status, double power)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = status;
    param[1] = power;

    if (c.execute("SetPowerLimit", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetPowerLimit fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
* @brief 关节扭矩控制开始
* @return  错误码
*/
errno_t FRRobot::ServoJTStart()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ServoJTStart", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute ServoJTStart fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 关节扭矩控制
 * @param  [in] torque j1~j6关节扭矩，单位Nm
 * @param  [in] interval 指令周期，单位s，范围[0.001~0.008]
 * @return  错误码
 */
errno_t FRRobot::ServoJT(float torque[], double interval)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0][0] = torque[0];
    param[0][1] = torque[1];
    param[0][2] = torque[2];
    param[0][3] = torque[3];
    param[0][4] = torque[4];
    param[0][5] = torque[5];
    param[1] = interval;

    if (c.execute("ServoJT", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute ServoJT fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 关节扭矩控制结束
 * @return  错误码
 */
errno_t FRRobot::ServoJTEnd()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ServoJTEnd", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute ServoJTEnd fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
* @brief 设置机器人 20004 端口反馈周期
* @param period 机器人 20004 端口反馈周期(ms)
* @return  错误码
*/
errno_t FRRobot::SetRobotRealtimeStateSamplePeriod(int period)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = period;

    if (c.execute("SetRobotRealtimeStateSamplePeriod", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetRobotRealtimeStateSamplePeriod fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  获取机器人 20004 端口反馈周期
 * @param [out] period 机器人 20004 端口反馈周期(ms)
 * @return  错误码
 */
errno_t FRRobot::GetRobotRealtimeStateSamplePeriod(int& period)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetRobotRealtimeStateSamplePeriod", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            period = int(result[1]);
        }
        else
        {
            logger_error("GetRobotRealtimeStateSamplePeriod fail. %d", errcode);
        }
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
* @brief 获取机器人关节驱动器温度(℃)
* @return 错误码
*/
errno_t FRRobot::GetJointDriverTemperature(double temperature[])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    
    for (int i = 0; i < 6; i++)
    {
        temperature[i] = robot_state_pkg->jointDriverTemperature[i];
    }
    return g_sock_com_err;
}

/**
 * @brief 获取机器人关节驱动器扭矩(Nm)
 * @return 错误码
 */
errno_t FRRobot::GetJointDriverTorque(double torque[])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    for (int i = 0; i < 6; i++)
    {
        torque[i] = robot_state_pkg->jointDriverTorque[i];
    }
    return g_sock_com_err;
}

/**
 * @brief 电弧追踪 + 多层多道补偿开启
 * @return 错误码
 */
errno_t FRRobot::ArcWeldTraceReplayStart()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ArcWeldTraceReplayStart", param, result))
    {
        return int(result);
    }
    else
    {
        logger_error("execute ArcWeldTraceReplayStart fail: %d.", errcode);
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 电弧追踪 + 多层多道补偿关闭
 * @return 错误码
 */
errno_t FRRobot::ArcWeldTraceReplayEnd()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ArcWeldTraceReplayEnd", param, result))
    {
        return int(result);
    }
    else
    {
        logger_error("execute ArcWeldTraceReplayEnd fail: %d.", errcode);
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
* @brief 偏移量坐标变化-多层多道焊
* @param[in] pointO
* @param[in] pointX
* @param[in] pointZ
* @param[in] dx
* @param[in] dy
* @param[in] db
* @param[out] offset
* @return 错误码
*/
errno_t FRRobot::MultilayerOffsetTrsfToBase(DescTran pointO, DescTran pointX, DescTran pointZ, double dx, double dy, double db, DescPose& offset)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = pointO.x;
    param[1] = pointO.y;
    param[2] = pointO.z;
    param[3] = pointX.x;
    param[4] = pointX.y;
    param[5] = pointX.z;
    param[6] = pointZ.x;
    param[7] = pointZ.y;
    param[8] = pointZ.z;
    param[9] = dx;
    param[10] = dy;
    param[11] = db;

    if (c.execute("MultilayerOffsetTrsfToBase", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            offset.tran.x = double(result[1]);
            offset.tran.y = double(result[2]);
            offset.tran.z = double(result[3]);
            offset.rpy.rx = double(result[4]);
            offset.rpy.ry = double(result[5]);
            offset.rpy.rz = double(result[6]);
        }
        else
        {
            logger_error("MultilayerOffsetTrsfToBase fail. %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
* @brief 指定姿态速度开启
* @param [in] ratio 姿态速度百分比[0-300]
* @return  错误码
*/
errno_t FRRobot::AngularSpeedStart(int ratio)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = ratio;

    if (c.execute("AngularSpeedStart", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute AngularSpeedStart fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;

}

/**
 * @brief 指定姿态速度关闭
 * @return  错误码
 */
errno_t FRRobot::AngularSpeedEnd()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("AngularSpeedEnd", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute AngularSpeedEnd fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
* @brief 机器人软件升级
* @param [in] filePath 软件升级包全路径
* @param [in] block 是否阻塞至升级完成 true:阻塞；false:非阻塞
* @return  错误码
*/
errno_t FRRobot::SoftwareUpgrade(std::string filePath, bool block)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = FileUpLoad(1, filePath);
    if (0 == errcode)
    {
        logger_info("Software Upload success!");
        XmlRpcClient c(serverUrl, 20003);
        XmlRpcValue param, result;

        if (c.execute("SoftwareUpgrade", param, result))
        {
            errcode = int(result);
            logger_info("software Upgrade start!");
            if (0 != errcode)
            {
                logger_error("software Upgrade fail");
            }
            c.close();

            if (block)
            {
                int upgradeState = -1;
                Sleep(3000);
                GetSoftwareUpgradeState(upgradeState);
                if (upgradeState == 0)
                {
                    logger_error("software upgrade not start");
                    return -1;
                }
                while (upgradeState > 0 && upgradeState < 100)
                {
                    Sleep(1000);
                    GetSoftwareUpgradeState(upgradeState);
                    logger_info("software Upgrade state is %d!", upgradeState);
                }

                if (upgradeState == 100)
                {
                    errcode = 0;
                }
                else
                {
                    errcode = upgradeState;
                }
            }

            return errcode;
        }
        else {
            logger_error("execute SoftwareUpgrade fail.");
            c.close();
            return ERR_XMLRPC_CMD_FAILED;
        }
    }
    else {
        logger_error("SoftwareUpgrade fail. errcode is: %d.", errcode);
    }

    return errcode;
}


/**
* @brief  获取机器人软件升级状态
* @param [out] state 机器人软件包升级状态 0-空闲中或上传升级包中；1~100：升级完成百分比；-1:升级软件失败；-2：校验失败；-3：版本校验失败；-4：解压失败；-5：用户配置升级失败；-6：外设配置升级失败；-7：扩展轴配置升级失败；-8：机器人配置升级失败；-9：DH参数配置升级失败
* @return  错误码
*/
errno_t FRRobot::GetSoftwareUpgradeState(int& state)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    state = robot_state_pkg->softwareUpgradeState;
    
    return g_sock_com_err;
}

/**
* @brief 设置485扩展轴运动加减速度
* @param [in] acc 485扩展轴运动加速度
* @param [in] dec 485扩展轴运动减速度
* @return  错误码
*/
errno_t FRRobot::AuxServoSetAcc(double acc, double dec)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = acc;
    param[1] = dec;

    if (c.execute("AuxServoSetAcc", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute AuxServoSetAcc fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置485扩展轴急停加减速度
 * @param [in] acc 485扩展轴急停加速度
 * @param [in] dec 485扩展轴急停减速度
 * @return  错误码
 */
errno_t FRRobot::AuxServoSetEmergencyStopAcc(double acc, double dec)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = acc;
    param[1] = dec;

    if (c.execute("AuxServoSetEmergencyStopAcc", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute AuxServoSetEmergencyStopAcc fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 获取485扩展轴运动加减速度
 * @param [out] acc 485扩展轴运动加速度
 * @param [out] dec 485扩展轴运动减速度
 * @return  错误码
 */
errno_t FRRobot::AuxServoGetAcc(double& acc, double& dec)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("AuxServoGetAcc", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            acc = double(result[1]);
            dec = double(result[2]);
        }
        else
        {
            logger_error("AuxServoGetAcc fail. %d", errcode);
        }
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 获取485扩展轴急停加减速度
 * @param [out] acc 485扩展轴急停加速度
 * @param [out] dec 485扩展轴急停减速度
 * @return  错误码
 */
errno_t FRRobot::AuxServoGetEmergencyStopAcc(double& acc, double& dec)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("AuxServoGetEmergencyStopAcc", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            acc = double(result[1]);
            dec = double(result[2]);
        }
        else
        {
            logger_error("AuxServoGetEmergencyStopAcc fail. %d", errcode);
        }
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 获取末端通讯参数
 * @param param 末端通讯参数
 * @return  错误码
 */
errno_t FRRobot::GetAxleCommunicationParam(AxleComParam* comParam)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetAxleCommunicationParam", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            comParam->baudRate = (int)result[1];
            comParam->dataBit = (int)result[2];
            comParam->stopBit = (int)result[3];
            comParam->verify = (int)result[4];
            comParam->timeout = (int)result[5];
            comParam->timeoutTimes = (int)result[6];
            comParam->period = (int)result[7];
        }
        else
        {
            logger_error("GetAxleCommunicationParam fail. %d", errcode);
        }
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
    
}

/**
 * @brief 设置末端通讯参数
 * @param param  末端通讯参数
 * @return  错误码
 */
errno_t FRRobot::SetAxleCommunicationParam(AxleComParam comParam)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = comParam.baudRate;
    param[1] = comParam.dataBit;
    param[2] = comParam.stopBit;
    param[3] = comParam.verify;
    param[4] = comParam.timeout;
    param[5] = comParam.timeoutTimes;
    param[6] = comParam.period;

    if (c.execute("SetAxleCommunicationParam", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetAxleCommunicationParam fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置末端文件传输类型
 * @param type 1-MCU升级文件；2-LUA文件
 * @return  错误码
 */
errno_t FRRobot::SetAxleFileType(int type)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = type;

    if (c.execute("SetAxleFileType", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetAxleFileType fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置启用末端LUA执行
 * @param enable 0-不启用；1-启用
 * @return  错误码
 */
errno_t FRRobot::SetAxleLuaEnable(int enable)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = enable;

    if (c.execute("SetAxleLuaEnable", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetAxleLuaEnable fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 末端LUA文件异常错误恢复
 * @param status 0-不恢复；1-恢复
 * @return  错误码
 */
errno_t FRRobot::SetRecoverAxleLuaErr(int status)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = status;

    if (c.execute("SetRecoverAxleLuaErr", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetRecoverAxleLuaErr fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 获取末端LUA执行使能状态
 * @param status status[0]: 0-未使能；1-已使能
 * @return  错误码
 */
errno_t FRRobot::GetAxleLuaEnableStatus(int* status)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetAxleLuaEnableStatus", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            *status = (int)result[1];
        }
        else
        {
            logger_error("GetAxleLuaEnableStatus fail. %d", errcode);
        }
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 设置末端LUA末端设备启用类型
 * @param forceSensorEnable 力传感器启用状态，0-不启用；1-启用
 * @param gripperEnable 夹爪启用状态，0-不启用；1-启用
 * @param IOEnable IO设备启用状态，0-不启用；1-启用
 * @return  错误码
 */
errno_t FRRobot::SetAxleLuaEnableDeviceType(int forceSensorEnable, int gripperEnable, int IOEnable)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = forceSensorEnable;
    param[1] = gripperEnable;
    param[2] = IOEnable;

    if (c.execute("SetAxleLuaEnableDeviceType", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetAxleLuaEnableDeviceType fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 获取末端LUA末端设备启用类型
 * @param forceSensorEnable 力传感器启用状态，0-不启用；1-启用
 * @param gripperEnable 夹爪启用状态，0-不启用；1-启用
 * @param IOEnable IO设备启用状态，0-不启用；1-启用
 * @return  错误码
 */
errno_t FRRobot::GetAxleLuaEnableDeviceType(int* forceSensorEnable, int* gripperEnable, int* IOEnable)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetAxleLuaEnableDeviceType", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            *forceSensorEnable = (int)result[1];
            *gripperEnable = (int)result[2];
            *IOEnable = (int)result[3];
        }
        else
        {
            logger_error("GetAxleLuaEnableDeviceType fail. %d", errcode);
        }
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 获取当前配置的末端设备
 * @param forceSensorEnable 力传感器启用设备编号 0-未启用；1-启用
 * @param gripperEnable 夹爪启用设备编号，0-不启用；1-启用
 * @param IODeviceEnable IO设备启用设备编号，0-不启用；1-启用
 * @return  错误码
 */
errno_t FRRobot::GetAxleLuaEnableDevice(int forceSensorEnable[], int gripperEnable[], int IODeviceEnable[])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetAxleLuaEnableDevice", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            string paramStr = (string)result[1];
            std::vector<std::string> parS = split(paramStr, ',');
            if (parS.size() != 24)
            {
                logger_error("GetAxleLuaEnableDevice fail");
                return -1;
            }
            forceSensorEnable[0] = stod(parS[0]);
            forceSensorEnable[1] = stod(parS[1]);
            forceSensorEnable[2] = stod(parS[2]);
            forceSensorEnable[3] = stod(parS[3]);
            forceSensorEnable[4] = stod(parS[4]);
            forceSensorEnable[5] = stod(parS[5]);
            forceSensorEnable[6] = stod(parS[6]);
            forceSensorEnable[7] = stod(parS[7]);

            gripperEnable[0] = stod(parS[8]);
            gripperEnable[1] = stod(parS[9]);
            gripperEnable[2] = stod(parS[10]);
            gripperEnable[3] = stod(parS[11]);
            gripperEnable[4] = stod(parS[12]);
            gripperEnable[5] = stod(parS[13]);
            gripperEnable[6] = stod(parS[14]);
            gripperEnable[7] = stod(parS[15]);

            IODeviceEnable[0] = stod(parS[16]);
            IODeviceEnable[1] = stod(parS[17]);
            IODeviceEnable[2] = stod(parS[18]);
            IODeviceEnable[3] = stod(parS[19]);
            IODeviceEnable[4] = stod(parS[20]);
            IODeviceEnable[5] = stod(parS[21]);
            IODeviceEnable[6] = stod(parS[22]);
            IODeviceEnable[7] = stod(parS[23]);
        }
        else {
            logger_error("execute GetRobotTeachingPoint fail %d", errcode);
        }
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 设置启用夹爪动作控制功能
 * @param id 夹爪设备编号
 * @param func func[0]-夹爪使能；func[1]-夹爪初始化；2-位置设置；3-速度设置；4-力矩设置；6-读夹爪状态；7-读初始化状态；8-读故障码；9-读位置；10-读速度；11-读力矩; 12-15预留
 * @return  错误码
 */
errno_t FRRobot::SetAxleLuaGripperFunc(int id, int func[])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    for (int i = 0; i < 16; i++)
    {
        param[1][i] = func[i];
    }


    if (c.execute("SetAxleLuaGripperFunc", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetAxleLuaGripperFunc fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 获取启用夹爪动作控制功能
 * @param id 夹爪设备编号
 * @param func func[0]-夹爪使能；func[1]-夹爪初始化；2-位置设置；3-速度设置；4-力矩设置；6-读夹爪状态；7-读初始化状态；8-读故障码；9-读位置；10-读速度；11-读力矩
 * @return  错误码
 */
errno_t FRRobot::GetAxleLuaGripperFunc(int id, int func[])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;
    param[0] = id;

    if (c.execute("GetAxleLuaGripperFunc", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            string paramStr = (string)result[1];
            std::vector<std::string> parS = split(paramStr, ',');
            if (parS.size() != 16)
            {
                logger_error("GetAxleLuaGripperFunc fail");
                return -1;
            }
            func[0] = stod(parS[0]);
            func[1] = stod(parS[1]);
            func[2] = stod(parS[2]);
            func[3] = stod(parS[3]);
            func[4] = stod(parS[4]);
            func[5] = stod(parS[5]);
            func[6] = stod(parS[6]);
            func[7] = stod(parS[7]);
            func[8] = stod(parS[8]);
            func[9] = stod(parS[9]);
            func[10] = stod(parS[10]);
            func[11] = stod(parS[11]);
            func[12] = stod(parS[12]);
            func[13] = stod(parS[13]);
            func[14] = stod(parS[14]);
            func[15] = stod(parS[15]);
        }
        else {
            logger_error("execute GetAxleLuaGripperFunc fail %d", errcode);
        }
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 设置控制器外设协议LUA文件名
 * @param id 协议编号
 * @param name lua文件名称 “CTRL_LUA_test.lua”
 * @return  错误码
 */
errno_t FRRobot::SetCtrlOpenLUAName(int id, std::string name)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    param[1] = name;

    if (c.execute("SetCtrlOpenLUAName", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetCtrlOpenLUAName fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 获取当前配置的控制器外设协议LUA文件名
 * @param name 4个lua文件名称 “CTRL_LUA_test.lua”
 * @return  错误码
 */
errno_t FRRobot::GetCtrlOpenLUAName(std::string name[])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetCtrlOpenLUAName", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            string paramStr = (string)result[1];
            std::vector<std::string> parS = split(paramStr, ',');
            if (parS.size() != 24)
            {
                logger_error("GetCtrlOpenLUAName fail");
                return -1;
            }
            name[0] = parS[0];
            name[1] = parS[1];
            name[2] = parS[2];
            name[3] = parS[3];
        }
        else {
            logger_error("execute GetCtrlOpenLUAName fail %d", errcode);
        }
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 加载控制器LUA协议
 * @param id 控制器LUA协议编号
 * @return  错误码
 */
errno_t FRRobot::LoadCtrlOpenLUA(int id)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;

    if (c.execute("LoadCtrlOpenLUA", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute LoadCtrlOpenLUA fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 卸载控制器LUA协议
 * @param id 控制器LUA协议编号
 * @return  错误码
 */
errno_t FRRobot::UnloadCtrlOpenLUA(int id)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;

    if (c.execute("UnloadCtrlOpenLUA", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute UnloadCtrlOpenLUA fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置控制器LUA协议错误码
 * @param id 控制器LUA协议编号
 * @return  错误码
 */
errno_t FRRobot::SetCtrlOpenLuaErrCode(int id, int code)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    param[1] = code;

    if (c.execute("SetCtrlOpenLuaErrCode", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetCtrlOpenLuaErrCode fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 机器人Ethercat从站文件写入
 * @param type 从站文件类型，1-升级从站文件；2-升级从站配置文件
 * @param slaveID 从站号
 * @param fileName 上传文件名
 * @return  错误码
 */
errno_t FRRobot::SlaveFileWrite(int type, int slaveID, std::string fileName)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = type;
    param[1] = slaveID;
    param[2] = fileName;

    if (c.execute("SlaveFileWrite", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SlaveFileWrite fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 机器人Ethercat从站进入boot模式
 * @return  错误码
 */
errno_t FRRobot::SetSysServoBootMode()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("SetSysServoBootMode", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetSysServoBootMode fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 可移动装置使能
 * @param enable false-去使能；true-使能
 * @return 错误码
 */
errno_t FRRobot::TractorEnable(bool enable)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = enable? 1:0;

    if (c.execute("TractorEnable", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute TractorEnable fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 可移动装置回零
 * @return 错误码
 */
errno_t FRRobot::TractorHoming()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("TractorHoming", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute TractorHoming fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 可移动装置直线运动
 * @param distance 直线运动距离（mm）
 * @param vel 直线运动速度百分比（0-100）
 * @return 错误码
 */
errno_t FRRobot::TractorMoveL(double distance, double vel)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = distance;
    param[1] = vel;

    if (c.execute("TractorMoveL", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute TractorMoveL fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 可移动装置圆弧运动
 * @param radio 圆弧运动半径（mm）
 * @param angle 圆弧运动角度（°）
 * @param vel 直线运动速度百分比（0-100）
 * @return 错误码
 */
errno_t FRRobot::TractorMoveC(double radio, double angle, double vel)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = radio;
    param[1] = angle;
    param[2] = vel;

    if (c.execute("TractorMoveC", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute TractorMoveC fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 可移动装置停止运动
 * @return 错误码
 */
errno_t FRRobot::TractorStop()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int rtn = ProgramStop();
    return rtn;
}

/**
 * @brief 设置焊丝寻位扩展IO端口
 * @param searchDoneDINum 焊丝寻位成功DO端口(0-127)
 * @param searchStartDONum 焊丝寻位启停控制DO端口(0-127)
 * @return 错误码
 */
errno_t FRRobot::SetWireSearchExtDIONum(int searchDoneDINum, int searchStartDONum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = searchDoneDINum;
    param[1] = searchStartDONum;

    if (c.execute("SetWireSearchExtDIONum", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetWireSearchExtDIONum fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置焊机控制模式扩展DO端口
 * @param DONum 焊机控制模式DO端口(0-127)
 * @return 错误码
 */
errno_t FRRobot::SetWeldMachineCtrlModeExtDoNum(int DONum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = DONum;

    if (c.execute("SetWeldMachineCtrlModeExtDoNum", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetWeldMachineCtrlModeExtDoNum fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置焊机控制模式
 * @param mode 焊机控制模式;0-一元化
 * @return 错误码
 */
errno_t FRRobot::SetWeldMachineCtrlMode(int mode)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = mode;

    if (c.execute("SetWeldMachineCtrlMode", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetWeldMachineCtrlMode fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
* @brief 上传末端Lua开放协议文件
* @param filePath 本地lua文件路径名 ".../AXLE_LUA_End_DaHuan.lua"
* @return 错误码
*/
errno_t FRRobot::AxleLuaUpload(std::string filePath)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = FileUpLoad(10, filePath);
    if (0 == errcode)
    {
  
        /* 提取文件名称 */
        size_t pos = filePath.find_last_of("/\\");
        if (std::string::npos == pos)
        {
            logger_error("format of path is wrong, should be like /home/fd/xxx.tar.gz");
            return ERR_UPLOAD_FILE_NOT_FOUND;
        }
        std::string filename = filePath.substr(pos + 1);
        std::string fileFullName = "/tmp/" + filename;
        cout << "file full path is << " << fileFullName << endl;
        int rtn = SetAxleFileType(2);
        if (rtn != 0)
        {
            logger_error("SetAxleFileType failed");
            return -1;
        }
        rtn = SetSysServoBootMode();
        if (rtn != 0)
        {
            logger_error("SetSysServoBootMode failed");
            return -1;
        }
        Sleep(1000);
        rtn = SlaveFileWrite(1, 7, fileFullName);
        if (rtn != 0)
        {
            logger_error("SlaveFileWrite failed %s" ,fileFullName);
            return -1;
        }

    }
    else {
        logger_error("upload file fail. errcode is: %d.", errcode);
    }

    return errcode;
       
}

/**
* @brief 开始奇异位姿保护
* @param [in] protectMode 奇异保护模式，0：关节模式；1-笛卡尔模式
* @param [in] minShoulderPos 肩奇异调整范围(mm), 默认100
* @param [in] minElbowPos 肘奇异调整范围(mm), 默认50
* @param [in] minWristPos 腕奇异调整范围(°), 默认10
* @return 错误码
*/
errno_t FRRobot::SingularAvoidStart(int protectMode, double minShoulderPos, double minElbowPos, double minWristPos)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = protectMode;
    param[1] = minShoulderPos;
    param[2] = minElbowPos;
    param[3] = minWristPos;

    if (c.execute("SingularAvoidStart", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SingularAvoidStart fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
* @brief 停止奇异位姿保护
* @return 错误码
*/
errno_t FRRobot::SingularAvoidEnd()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("SingularAvoidEnd", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SingularAvoidEnd fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
* @brief 开始Ptp运动FIR滤波
* @param [in] maxAcc 最大加速度极值(deg/s2)
* @param [in] maxJek 统一关节急动度极值(deg/s3)
* @return 错误码
*/
errno_t FRRobot::PtpFIRPlanningStart(double maxAcc, double maxJek)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = maxAcc;
    param[1] = maxJek;

    if (c.execute("PtpFIRPlanningStart", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute PtpFIRPlanningStart fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
* @brief 关闭Ptp运动FIR滤波
* @return 错误码
*/
errno_t FRRobot::PtpFIRPlanningEnd()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("PtpFIRPlanningEnd", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute PtpFIRPlanningEnd fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
* @brief 开始LIN、ARC运动FIR滤波
* @param [in] maxAccLin 线加速度极值(mm/s2)
* @param [in] maxAccDeg 角加速度极值(deg/s2)
* @param [in] maxJerkLin 线加加速度极值(mm/s3)
* @param [in] maxJerkDeg 角加加速度极值(deg/s3)
* @return 错误码
*/
errno_t FRRobot::LinArcFIRPlanningStart(double maxAccLin, double maxAccDeg, double maxJerkLin, double maxJerkDeg)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = maxAccLin;
    param[1] = maxAccDeg;
    param[2] = maxJerkLin;
    param[3] = maxJerkDeg;

    if (c.execute("LinArcFIRPlanningStart", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute LinArcFIRPlanningStart fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
* @brief 关闭LIN、ARC运动FIR滤波
* @return 错误码
*/
errno_t FRRobot::LinArcFIRPlanningEnd()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("LinArcFIRPlanningEnd", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute LinArcFIRPlanningEnd fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 上传轨迹J文件
 * @param [in] filePath 上传轨迹文件的全路径名   C://test/testJ.txt
 * @return 错误码
 */
errno_t FRRobot::TrajectoryJUpLoad(const std::string& filePath)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    return FileUpLoad(20, filePath);
}

/**
 * @brief 删除轨迹J文件
 * @param [in] fileName 文件名称 testJ.txt
 * @return 错误码
 */
errno_t FRRobot::TrajectoryJDelete(const std::string& fileName)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    return FileDelete(20, fileName);
}

/**
 * @brief 根据点位信息计算工具坐标系
 * @param [in] method 计算方法；0-四点法；1-六点法
 * @param [in] pos 关节位置组，四点法时数组长度为4个，六点法时数组长度为6个
 * @param [out] coord 工具坐标系结果
 * @return 错误码
 */
errno_t FRRobot::ComputeToolCoordWithPoints(int method, JointPos pos[], DescPose& coord)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = method;
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 6; j++)
        {
            param[i+1][j] = pos[i].jPos[j];
        }
    }

    for (int i = 4; i < 6; i++)
    {
        if (method == 0)
        {
            for (int j = 0; j < 6; j++)
            {
                param[i + 1][j] = 0.0;
            }
        }
        else if (method == 1)
        {
            for (int j = 0; j < 6; j++)
            {
                param[i + 1][j] = pos[i].jPos[j];
            }
        }
        
    }

    if (c.execute("ComputeToolCoordWithPoints", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            coord.tran.x = double(result[1]);
            coord.tran.y = double(result[2]);
            coord.tran.z = double(result[3]);
            coord.rpy.rx = double(result[4]);
            coord.rpy.ry = double(result[5]);
            coord.rpy.rz = double(result[6]);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 根据点位信息计算工件坐标系
 * @param [in] method 计算方法；0：原点-x轴-z轴  1：原点-x轴-xy平面
 * @param [in] pos 三个TCP位置组
 * @param [in] refFrame 参考坐标系
 * @param [out] coord 工具坐标系结果
 * @return 错误码
 */
errno_t FRRobot::ComputeWObjCoordWithPoints(int method, DescPose pos[], int refFrame, DescPose& coord)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = method;
    for (int i = 0; i < 3; i++)
    {
        param[i + 1][0] = pos[i].tran.x;
        param[i + 1][1] = pos[i].tran.y;
        param[i + 1][2] = pos[i].tran.z;
        param[i + 1][3] = pos[i].rpy.rx;
        param[i + 1][4] = pos[i].rpy.ry;
        param[i + 1][5] = pos[i].rpy.rz;
    }
    param[4] = refFrame;

    if (c.execute("ComputeWObjCoordWithPoints", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            coord.tran.x = double(result[1]);
            coord.tran.y = double(result[2]);
            coord.tran.z = double(result[3]);
            coord.rpy.rx = double(result[4]);
            coord.rpy.ry = double(result[5]);
            coord.rpy.rz = double(result[6]);
        }

    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 设置机器人焊接电弧意外中断检测参数
 * @param [in] checkEnable 是否使能检测；0-不使能；1-使能
 * @param [in] arcInterruptTimeLength 电弧中断确认时长(ms)
 * @return 错误码
 */
errno_t FRRobot::WeldingSetCheckArcInterruptionParam(int checkEnable, int arcInterruptTimeLength)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = checkEnable;
    param[1] = arcInterruptTimeLength;

    if (c.execute("WeldingSetCheckArcInterruptionParam", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute WeldingSetCheckArcInterruptionParam fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 获取机器人焊接电弧意外中断检测参数
 * @param [out] checkEnable 是否使能检测；0-不使能；1-使能
 * @param [out] arcInterruptTimeLength 电弧中断确认时长(ms)
 * @return 错误码
 */
errno_t FRRobot::WeldingGetCheckArcInterruptionParam(int* checkEnable, int* arcInterruptTimeLength)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("WeldingGetCheckArcInterruptionParam", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            *checkEnable = (int)result[1];
            *arcInterruptTimeLength = (int)result[2];
        }
        else
        {
            logger_error("WeldingGetCheckArcInterruptionParam fail. %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置机器人焊接中断恢复参数
 * @param [in] enable 是否使能焊接中断恢复
 * @param [in] length 焊缝重叠距离(mm)
 * @param [in] velocity 机器人回到再起弧点速度百分比(0-100)
 * @param [in] moveType 机器人运动到再起弧点方式；0-LIN；1-PTP
 * @return 错误码
 */
errno_t FRRobot::WeldingSetReWeldAfterBreakOffParam(int enable, double length, double velocity, int moveType)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = enable;
    param[1] = length;
    param[2] = velocity;
    param[3] = moveType;

    if (c.execute("WeldingSetReWeldAfterBreakOffParam", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute WeldingSetReWeldAfterBreakOffParam fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 获取机器人焊接中断恢复参数
 * @param [out] enable 是否使能焊接中断恢复
 * @param [out] length 焊缝重叠距离(mm)
 * @param [out] velocity 机器人回到再起弧点速度百分比(0-100)
 * @param [out] moveType 机器人运动到再起弧点方式；0-LIN；1-PTP
 * @return 错误码
 */
errno_t FRRobot::WeldingGetReWeldAfterBreakOffParam(int* enable, double* length, double* velocity, int* moveType)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("WeldingGetReWeldAfterBreakOffParam", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            *enable = (int)result[1];
            *length = (double)result[2];
            *velocity = (double)result[3];
            *moveType = (int)result[4];
        }
        else
        {
            logger_error("WeldingGetReWeldAfterBreakOffParam fail. %d", errcode);
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置机器人焊接中断后恢复焊接
 * @return 错误码
 */
errno_t FRRobot::WeldingStartReWeldAfterBreakOff()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("WeldingStartReWeldAfterBreakOff", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置机器人焊接中断后退出焊接
 * @return 错误码
 */
errno_t FRRobot::WeldingAbortWeldAfterBreakOff()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("WeldingAbortWeldAfterBreakOff", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

errno_t FRRobot::LaserSensorRecord(int status, int delayMode, int delayTime, int delayDisExAxisNum, double delayDis, double sensitivePara, double speed)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = status;
    param[1] = delayMode;
    param[2] = delayTime;
    param[3] = delayDisExAxisNum;
    param[4] = delayDis;
    param[5] = sensitivePara;
    param[6] = speed;

    if (c.execute("LaserSensorRecord", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute LaserSensorRecord fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

errno_t FRRobot::LaserTrackingLaserOn(int weldId)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = weldId;

    if (c.execute("LaserTrackingLaserOn", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute LaserTrackingLaserOn fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

errno_t FRRobot::LaserTrackingLaserOff()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("LaserTrackingLaserOff", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute LaserTrackingLaserOff fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

errno_t FRRobot::LaserTrackingTrackOn(int coordId)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = coordId;

    if (c.execute("LaserTrackingTrackOn", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute LaserTrackingTrackOn fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

errno_t FRRobot::LaserTrackingTrackOff()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("LaserTrackingTrackOff", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute LaserTrackingTrackOff fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

errno_t FRRobot::LaserTrackingSearchStart(int direction, DescTran directionPoint, int vel, int distance, int timeout, int posSensorNum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = direction;
    param[1] = directionPoint.x;
    param[2] = directionPoint.y;
    param[3] = directionPoint.z;
    param[4] = vel;
    param[5] = distance;
    param[6] = timeout;
    param[7] = posSensorNum;

    if (c.execute("LaserTrackingSearchStart", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute LaserTrackingSearchStart fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

errno_t FRRobot::LaserTrackingSearchStop()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("LaserTrackingSearchStop", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute LaserTrackingSearchStop fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  摆动渐变开始
 * @param [in] weaveChangeFlag 1-变摆动参数；2-变摆动参数+焊接速度
 * @param [in] weaveNum 摆动编号 
 * @param [in] velStart 焊接开始速度，(cm/min)
 * @param [in] velEnd 焊接结束速度，(cm/min)
 * @return  错误码
 */
errno_t FRRobot::WeaveChangeStart(int weaveChangeFlag, int weaveNum, double velStart, double velEnd)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = weaveChangeFlag;
    param[1] = weaveNum;
    param[2] = velStart;
    param[3] = velEnd;

    if (c.execute("WeaveChangeStart", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute WeaveChangeStart fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  摆动渐变结束
 * @param  [in] weaveNum 摆动编号
 * @return  错误码
 */
errno_t FRRobot::WeaveChangeEnd()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("WeaveChangeEnd", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute WeaveChangeEnd fail: %d.", errcode);
            c.close();
            return errcode;
        }
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief  轨迹预处理(轨迹前瞻)
 * @param  [in] name  轨迹文件名
 * @param  [in] mode 采样模式，0-不进行采样；1-等数据间隔采样；2-等误差限制采样
 * @param  [in] errorLim 误差限制，使用直线拟合生效
 * @param  [in] type 平滑方式，0-贝塞尔平滑
 * @param  [in] precision 平滑精度，使用贝塞尔平滑时生效
 * @param  [in] vamx 设定的最大速度，mm/s
 * @param  [in] amax 设定的最大加速度，mm/s2
 * @param  [in] jmax 设定的最大加加速度，mm/s3
 * @return  错误码
 */
errno_t FRRobot::LoadTrajectoryLA(char name[30], int mode, double errorLim, int type, double precision, double vamx, double amax, double jmax)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = name;
    param[1] = mode;
    param[2] = errorLim * 1.0;
    param[3] = type;
    param[4] = precision * 1.0;
    param[5] = vamx * 1.0;
    param[6] = amax * 1.0;
    param[7] = jmax * 1.0;

    if (c.execute("LoadTrajectoryLA", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  轨迹复现(轨迹前瞻)
 * @return  错误码
 */
errno_t FRRobot::MoveTrajectoryLA()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("MoveTrajectoryLA", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  自定义碰撞检测阈值功能开始，设置关节端和TCP端的碰撞检测阈值
 * @param  [in] flag 1-仅关节检测开启；2-仅TCP检测开启；3-关节和TCP检测同时开启
 * @param  [in] jointDetectionThreshould 关节碰撞检测阈值 j1-j6
 * @param  [in] tcpDetectionThreshould TCP碰撞检测阈值，xyzabc
 * @param  [in] block 0-非阻塞；1-阻塞
 * @return  错误码
 */
errno_t FRRobot::CustomCollisionDetectionStart(int flag, double jointDetectionThreshould[6], double tcpDetectionThreshould[6], int block)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = flag;
    param[1][0] = jointDetectionThreshould[0];
    param[1][1] = jointDetectionThreshould[1];
    param[1][2] = jointDetectionThreshould[2];
    param[1][3] = jointDetectionThreshould[3];
    param[1][4] = jointDetectionThreshould[4];
    param[1][5] = jointDetectionThreshould[5];
    param[2][0] = tcpDetectionThreshould[0];
    param[2][1] = tcpDetectionThreshould[1];
    param[2][2] = tcpDetectionThreshould[2];
    param[2][3] = tcpDetectionThreshould[3];
    param[2][4] = tcpDetectionThreshould[4];
    param[2][5] = tcpDetectionThreshould[5];
    param[3] = block;

    if (c.execute("CustomCollisionDetectionStart", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    if ((robot_state_pkg->main_code != 0 || robot_state_pkg->sub_code != 0) && errcode == 0)
    {
        errcode = 14;
    }

    return errcode;
}

/**
 * @brief  自定义碰撞检测阈值功能关闭
 * @return  错误码
 */
errno_t FRRobot::CustomCollisionDetectionEnd()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("CustomCollisionDetectionEnd", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}


/**
 * @brief 加速度平滑开启
 * @param  [in] saveFlag 是否断电保存
 * @return  错误码
 */
errno_t FRRobot::AccSmoothStart(bool saveFlag)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = saveFlag? 1 : 0;

    if (c.execute("AccSmoothStart", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    if ((robot_state_pkg->main_code != 0 || robot_state_pkg->sub_code != 0) && errcode == 0)
    {
        errcode = 14;
    }

    return errcode;
}

/**
 * @brief 加速度平滑关闭
 * @param  [in] saveFlag 是否断电保存
 * @return  错误码
 */
errno_t FRRobot::AccSmoothEnd(bool saveFlag)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = saveFlag ? 1 : 0;

    if (c.execute("AccSmoothEnd", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief  控制器日志下载
 * @param [in] savePath 保存文件路径"D://zDown/"
 * @return  错误码
 */
errno_t FRRobot::RbLogDownload(std::string savePath)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("RbLogDownloadPrepare", param, result))
    {
        errcode = int(result);
        if (0 != errcode) {
            logger_error("RbLogDownloadPrepare fail.");
            c.close();
            return errcode;
        }
        logger_info("RbLogDownloadPrepare success.");
    }
    else 
    {
        logger_error("execute RbLogDownloadPrepare fail.");
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    string fileName = "rblog.tar.gz";
    errcode = FileDownLoad(1, fileName, savePath);
    if (errcode != 0)
    {
        remove((savePath + "rblog.tar.gz").c_str());
    }
    return errcode;
}

/**
 * @brief 所有数据源下载
 * @param [in] savePath 保存文件路径"D://zDown/"
 * @return  错误码
 */
errno_t FRRobot::AllDataSourceDownload(std::string savePath)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("AllDataSourceDownloadPrepare", param, result))
    {
        errcode = int(result);
        if (0 != errcode) {
            logger_error("AllDataSourceDownloadPrepare fail.");
            c.close();
            return errcode;
        }
        logger_info("AllDataSourceDownloadPrepare success.");
    }
    else
    {
        logger_error("execute AllDataSourceDownloadPrepare fail.");
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    string fileName = "alldatasource.tar.gz";
    errcode = FileDownLoad(2, fileName, savePath);
    if (errcode != 0)
    {
        remove((savePath + "alldatasource.tar.gz").c_str());
    }
    return errcode;
}

/**
 * @brief 数据备份包下载
 * @param [in] savePath 保存文件路径"D://zDown/"
 * @return  错误码
 */
errno_t FRRobot::DataPackageDownload(std::string savePath)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;

    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("DataPackageDownloadPrepare", param, result))
    {
        errcode = int(result);
        if (0 != errcode) {
            logger_error("DataPackageDownloadPrepare fail.");
            c.close();
            return errcode;
        }
        logger_info("DataPackageDownloadPrepare success.");
    }
    else
    {
        logger_error("execute DataPackageDownloadPrepare fail.");
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    string fileName = "fr_user_data.tar.gz";
    errcode = FileDownLoad(3, fileName, savePath);
    if (errcode != 0)
    {
        remove((savePath + "fr_user_data.tar.gz").c_str());
    }
    return errcode;
}

/**
 * @brief 获取控制箱SN码
 * @param [out] SNCode 控制箱SN码
 * @return 错误码
 */
errno_t FRRobot::GetRobotSN(std::string& SNCode)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetRobotSN", param, result))
    {
        errcode = int(result[0]);
        if (0 == errcode)
        {
            SNCode = (string)result[1];
        }
        else
        {
            logger_error("GetRobotSN fail, errcode is: %d\n", errcode);
        }
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 关闭机器人操作系统
 * @return 错误码
 */
errno_t FRRobot::ShutDownRobotOS()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ShutDownRobotOS", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 传送带通讯输入检测
 * @param [in] timeout 等待超时时间ms
 * @return 错误码
 */
errno_t FRRobot::ConveyorComDetect(int timeout)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = timeout;

    if (c.execute("ConveyorComDetect", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 传送带通讯输入检测触发
 * @return 错误码
 */
errno_t FRRobot::ConveyorComDetectTrigger()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    static int cnt = 0;

    memset(g_sendbuf, 0, BUFFER_SIZE * sizeof(char));
    sprintf(g_sendbuf, "/f/bIII%dIII1149III26IIIConveyorComDetectTrigger()III/b/f", cnt);
    cnt++;
    is_sendcmd = true;

    logger_info("ConveryComDetectTrigger().");
    return errcode;
}

/**
 * @brief 电弧跟踪焊机电流反馈AI通道选择
 * @param [in]  channel 通道；0-扩展AI0；1-扩展AI1；2-扩展AI2；3-扩展AI3；4-控制箱AI0；5-控制箱AI1
 * @return 错误码
 */
errno_t FRRobot::ArcWeldTraceAIChannelCurrent(int channel)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = channel;

    if (c.execute("ArcWeldTraceAIChannelCurrent", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 电弧跟踪焊机电压反馈AI通道选择
 * @param [in]  channel 通道；0-扩展AI0；1-扩展AI1；2-扩展AI2；3-扩展AI3；4-控制箱AI0；5-控制箱AI1
 * @return 错误码
 */
errno_t FRRobot::ArcWeldTraceAIChannelVoltage(int channel)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = channel;

    if (c.execute("ArcWeldTraceAIChannelVoltage", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 电弧跟踪焊机电流反馈转换参数
 * @param [in] AILow AI通道下限，默认值0V，范围[0-10V]
 * @param [in] AIHigh AI通道上限，默认值10V，范围[0-10V]
 * @param [in] currentLow AI通道下限对应焊机电流值，默认值0V，范围[0-200V]
 * @param [in] currentHigh AI通道上限对应焊机电流值，默认值100V，范围[0-200V]
 * @return 错误码
 */
errno_t FRRobot::ArcWeldTraceCurrentPara(float AILow, float AIHigh, float currentLow, float currentHigh)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = AILow;
    param[1] = AIHigh;
    param[2] = currentLow;
    param[3] = currentHigh;

    if (c.execute("ArcWeldTraceCurrentPara", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 电弧跟踪焊机电压反馈转换参数
 * @param [in] AILow AI通道下限，默认值0V，范围[0-10V]
 * @param [in] AIHigh AI通道上限，默认值10V，范围[0-10V]
 * @param [in] voltageLow AI通道下限对应焊机电压值，默认值0V，范围[0-200V]
 * @param [in] voltageHigh AI通道上限对应焊机电压值，默认值100V，范围[0-200V]
 * @return 错误码
 */
errno_t FRRobot::ArcWeldTraceVoltagePara(float AILow, float AIHigh, float voltageLow, float voltageHigh)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = AILow;
    param[1] = AIHigh;
    param[2] = voltageLow;
    param[3] = voltageHigh;

    if (c.execute("ArcWeldTraceVoltagePara", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 设置焊接电压渐变开始
 * @param [in] IOType 控制类型；0-控制箱IO；1-数字通信协议(UDP);2-数字通信协议(ModbusTCP)
 * @param [in] voltageStart 起始焊接电压(V)
 * @param [in] voltageEnd 终止焊接电压(V)
 * @param [in] AOIndex 控制箱AO端口号(0-1)
 * @param [in] blend 是否平滑 0-不平滑；1-平滑
 * @return 错误码
 */
errno_t FRRobot::WeldingSetVoltageGradualChangeStart(int IOType, double voltageStart, double voltageEnd, int AOIndex, int blend)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = IOType;
    param[1] = (double)voltageStart;
    param[2] = (double)voltageEnd;
    param[3] = AOIndex;
    param[4] = blend;

    if (c.execute("WeldingSetVoltageGradualChangeStart", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 设置焊接电压渐变结束
 * @return 错误码
 */
errno_t FRRobot::WeldingSetVoltageGradualChangeEnd()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("WeldingSetVoltageGradualChangeEnd", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 设置焊接电流渐变开始
 * @param [in] IOType 控制类型；0-控制箱IO；1-数字通信协议(UDP);2-数字通信协议(ModbusTCP)
 * @param [in] voltageStart 起始焊接电流(A)
 * @param [in] voltageEnd 终止焊接电流(A)
 * @param [in] AOIndex 控制箱AO端口号(0-1)
 * @param [in] blend 是否平滑 0-不平滑；1-平滑
 * @return 错误码
 */
errno_t FRRobot::WeldingSetCurrentGradualChangeStart(int IOType, double currentStart, double currentEnd, int AOIndex, int blend)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = IOType;
    param[1] = currentStart;
    param[2] = currentEnd;
    param[3] = AOIndex;
    param[4] = blend;

    if (c.execute("WeldingSetCurrentGradualChangeStart", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 设置焊接电流渐变结束
 * @return 错误码
 */
errno_t FRRobot::WeldingSetCurrentGradualChangeEnd()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("WeldingSetCurrentGradualChangeEnd", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
* @brief 获取SmartTool按钮状态
* @param [out] state SmartTool手柄按钮状态;(bit0:0-通信正常；1-通信掉线；bit1-撤销操作；bit2-清空程序；
bit3-A键；bit4-B键；bit5-C键；bit6-D键；bit7-E键；bit8-IO键；bit9-手自动；bit10开始)
* @return 错误码
*/
errno_t FRRobot::GetSmarttoolBtnState(int& state)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    state = robot_state_pkg->smartToolState;

    return 0;
}

/**
 * @brief 获取扩展轴坐标系
 * @param [out] coord 扩展轴坐标系
 * @return 错误码
 */
errno_t FRRobot::ExtAxisGetCoord(DescPose& coord)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ExtAxisGetCoord", param, result))
    {
        errcode = int(result[0]);
        if (0 == errcode)
        {
            coord.tran.x = (double)result[1];
            coord.tran.y = (double)result[2];
            coord.tran.z = (double)result[3];
            coord.rpy.rx = (double)result[4];
            coord.rpy.ry = (double)result[5];
            coord.rpy.rz = (double)result[6];
        }
        else
        {
            logger_error("ExtAxisGetCoord fail, errcode is: %d\n", errcode);
        }
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 设置宽电压控制箱温度及风扇电流监控参数
 * @param [in] enable 0-不使能监测；1-使能监测
 * @param [in] period 监测周期(s),范围1-100
 * @return 错误码
 */
errno_t FRRobot::SetWideBoxTempFanMonitorParam(int enable, int period)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = enable;
    param[1] = period;

    if (c.execute("SetWideBoxTempFanMonitorParam", param, result))
    {
        errcode = int(result);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}

/**
 * @brief 获取宽电压控制箱温度及风扇电流监控参数
 * @param [out] enable 0-不使能监测；1-使能监测
 * @param [out] period 监测周期(s),范围1-100
 * @return 错误码
 */
errno_t FRRobot::GetWideBoxTempFanMonitorParam(int& enable, int& period)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetWideBoxTempFanMonitorParam", param, result))
    {
        errcode = int(result[0]);
        if (0 == errcode)
        {
            enable = (int)result[1];
            period = (int)result[2];
        }
        else
        {
            logger_error("GetWideBoxTempFanMonitorParam fail, errcode is: %d\n", errcode);
        }
    }
    else
    {
        errcode = ERR_XMLRPC_CMD_FAILED;
    }

    c.close();

    return errcode;
}



/* 根据字符分割字符串 */
std::vector<std::string> FRRobot::split(const std::string &s, char delim)
{
    std::vector<std::string> result;
    std::stringstream ss(s);
    std::string item;

    while (getline(ss, item, delim))
    {
        result.push_back(item);
    }

    return result;
}

/* 根据字符串分割字符串 */
std::vector<std::string> FRRobot::split(std::string s, std::string delimiter)
{
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    std::string token;
    std::vector<std::string> res;

    while ((pos_end = s.find(delimiter, pos_start)) != std::string::npos) {
        token = s.substr (pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back (token);
    }

    res.push_back (s.substr (pos_start));
    return res;
}

bool FRRobot::IsSockError()
{
    while (rtClient->GetReConnState())
    {
        //如果正在重连，就等待重连结果
        Sleep(100);
    }

    if (g_sock_com_err != ERR_SUCCESS)
    {
        return true;
    }
    else
    {
        return false;
    }
}

//判断当前安全状态，安全停止、主子故障等
int FRRobot::GetSafetyCode()
{
    if (robot_state_pkg->safety_stop0_state == 1 || robot_state_pkg->safety_stop1_state == 1)
    {
        return 99;
    }

    return 0;
}

/**
 * @brief  设置与机器人通讯重连参数
 * @param  [in] enable  网络故障时使能重连 true-使能 false-不使能
 * @param  [in] reconnectTimes 重连时间，若超出此时间仍未连接成功，则报通信故障， 单位ms
 * @param  [in] period 重连周期，单位ms
 * @return  错误码
 */
errno_t FRRobot::SetReConnectParam(bool enable, int reconnectTime, int period)
{
    rtClient->SetReConnectParam(enable, reconnectTime, period);
    cmdClient->SetReConnectParam(enable, reconnectTime, period);
    return 0;
}

errno_t FRRobot::Sleep(int ms)
{
#ifdef WIN32
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
#else
    usleep(ms * 1000);
#endif
    return 0;
}

/**
 * @brief  机器人接口类析构函数
 */
FRRobot::~FRRobot(void)
{
    fr_logger::logger_deinit();

}
