#include "FRCNDEClient.h"
#include "CNDEFrameHandle.h"
#include "robot_error.h"
#include "Utility.h"
#include <stdio.h>
#include <chrono>
#include <tuple>
#include <algorithm>
#include <vector>

#ifdef __MINGW32__
#define TCP_MAXRT 5
#include <mingw.thread.h>
#else
#include <thread>
#endif
#include <memory>

using namespace std;

FRCNDEClient::FRCNDEClient(std::shared_ptr<ROBOT_STATE_PKG> pkg, int* comErr)
{
	robotStatePkg = pkg;
	rtClient = std::make_shared<FRTcpClient>("192.168.58.2", 20005);
    sockComErr = comErr;
    InitAllStates();
}

FRCNDEClient::~FRCNDEClient()
{

}

int FRCNDEClient::Connect(std::string IP, int port)
{
    rtClient->SetIpConfig(IP);
    rtClient->SetPortConfig(port);

	int rtn = rtClient->Connect();
    if (rtn != 0)
    {
        *sockComErr = ERR_SOCKET_COM_FAILED;
        return rtn;
    }
    else
    {
        robotStateRunFlag = true;
        rtn = SendCNDEOutputConfig();
        if (rtn != 0)
        {
            return rtn;
        }

        rtn = SetCNDEStart();
        if (rtn != 0)
        {
            return rtn;
        }
        
        thread stateThread(&FRCNDEClient::RecvRobotStateThread, this);
        stateThread.detach();
        return 0;
    }
}

int FRCNDEClient::SetReConnectParam(bool enable, int reconnectTime, int period)
{
    return rtClient->SetReConnectParam(enable, reconnectTime, period);
}

bool FRCNDEClient::GetReConnectEnable()
{
    return rtClient->GetReConnectEnable();
}

bool FRCNDEClient::GetReConnState()
{
    return rtClient->GetReConnState();
}

int FRCNDEClient::Close()
{
    robotStateRunFlag = false;
    
    rtClient->Close();
	return 0;
}



void FRCNDEClient::RecvRobotStateThread()
{
    char pkgBuf[CNDE_MAX_PKG_SIZE] = {};
    CNDE_PKG pkg = {};

    while (robotStateRunFlag)
    {
        std::lock_guard<std::mutex> lock(recvCNDEPkgMutex);

        int recvLen = rtClient->RecvCNDEPkg(pkgBuf);
        //std::cout << "recv pkg time is " << std::chrono::duration_cast<std::chrono::milliseconds>(
        //    std::chrono::system_clock::now().time_since_epoch()
        //).count() << std::endl;

        if (recvLen < 0)
        {   
            *sockComErr = ERR_SOCKET_COM_FAILED;
            return;
        }
        else if (recvLen == 0)  //发生重连，需要再次发送开始指令帧
        {
            SendCNDEOutputConfig();
            SetCNDEStart();
        }
        else
        {
            pkg.Clear();
            std::vector<char> frame(pkgBuf, pkgBuf + recvLen);
            int rtn = FrameToCNDEPkg(frame, pkg);
            //printf("pkg length is %d\n", pkg.len);
            memset(pkgBuf, 0, CNDE_MAX_PKG_SIZE);
            int statePtrIndex = 0;
            switch (pkg.type)
            {
                case CNDE_FRAME_TYPE_OUTPUT_STATE:
                {
                    //printf("type CNDE_FRAME_TYPE_OUTPUT_STATE\n");
                    for (auto type : configStates)
                    {
                        if (get<3>(allStates[type]) == get<2>(allStates[type]))//数据类型相同，直接对应
                        {
                            memcpy(reinterpret_cast<char*>(robotStatePkg.get()) + get<1>(allStates[type]), pkg.data.data() + statePtrIndex, GetConfigTypeSize(get<3>(allStates[type])));
                            statePtrIndex += GetConfigTypeSize(get<3>(allStates[type]));
                        }
                        else if (GetConfigTypeSize(get<3>(allStates[type])) == GetConfigTypeSize(get<2>(allStates[type])))//数据长度相同，直接对应
                        {
                            memcpy(reinterpret_cast<char*>(robotStatePkg.get()) + get<1>(allStates[type]), pkg.data.data() + statePtrIndex, GetConfigTypeSize(get<3>(allStates[type])));
                            statePtrIndex += GetConfigTypeSize(get<3>(allStates[type]));
                        }
                        else if(get<3>(allStates[type]) == "UINT8")
                        {
                            if (get<2>(allStates[type]) == "INT32")
                            {
                                uint8_t cndeValue = 0;
                                memcpy(&cndeValue, pkg.data.data() + statePtrIndex, GetConfigTypeSize(get<3>(allStates[type])));

                                int tmpValue = cndeValue;
                                memcpy(reinterpret_cast<char*>(robotStatePkg.get()) + get<1>(allStates[type]), &tmpValue, sizeof(int));
                            }
                            statePtrIndex += GetConfigTypeSize(get<3>(allStates[type]));
                        }
                        else if (get<3>(allStates[type]) == "INT32")
                        {
                            if (get<2>(allStates[type]) == "UINT8")
                            {
                                int cndeValue = 0;
                                memcpy(&cndeValue, pkg.data.data() + statePtrIndex, GetConfigTypeSize(get<3>(allStates[type])));

                                uint8_t tmpValue = cndeValue;
                                memcpy(reinterpret_cast<char*>(robotStatePkg.get()) + get<1>(allStates[type]), &tmpValue, sizeof(uint8_t));
                            }
                            else if (get<2>(allStates[type]) == "UINT16")
                            {
                                int cndeValue = 0;
                                memcpy(&cndeValue, pkg.data.data() + statePtrIndex, GetConfigTypeSize(get<3>(allStates[type])));

                                uint16_t tmpValue = cndeValue;
                                memcpy(reinterpret_cast<char*>(robotStatePkg.get()) + get<1>(allStates[type]), &tmpValue, sizeof(uint16_t));
                            }
                            else if (get<2>(allStates[type]) == "UINT32")
                            {
                                int cndeValue = 0;
                                memcpy(&cndeValue, pkg.data.data() + statePtrIndex, GetConfigTypeSize(get<3>(allStates[type])));

                                uint32_t tmpValue = cndeValue;
                                memcpy(reinterpret_cast<char*>(robotStatePkg.get()) + get<1>(allStates[type]), &tmpValue, sizeof(uint32_t));
                            }
                            else if (get<2>(allStates[type]) == "INT8")
                            {
                                int cndeValue = 0;
                                memcpy(&cndeValue, pkg.data.data() + statePtrIndex, GetConfigTypeSize(get<3>(allStates[type])));

                                int8_t tmpValue = cndeValue;
                                memcpy(reinterpret_cast<char*>(robotStatePkg.get()) + get<1>(allStates[type]), &tmpValue, sizeof(uint32_t));
                            }

                            statePtrIndex += GetConfigTypeSize(get<3>(allStates[type]));
                        }
                        else if (get<3>(allStates[type]) == "UINT32")
                        {
                            if (get<2>(allStates[type]) == "INT32")
                            {
                                uint32_t cndeValue = 0;
                                memcpy(&cndeValue, pkg.data.data() + statePtrIndex, GetConfigTypeSize(get<3>(allStates[type])));

                                int tmpValue = cndeValue;
                                memcpy(reinterpret_cast<char*>(robotStatePkg.get()) + get<1>(allStates[type]), &tmpValue, sizeof(int));  
                            }
                            statePtrIndex += GetConfigTypeSize(get<3>(allStates[type]));
                        }
                        else if (get<3>(allStates[type]) == "DOUBLE")
                        {
                            if (get<2>(allStates[type]) == "FLOAT")
                            {
                                double cndeValue = 0;
                                memcpy(&cndeValue, pkg.data.data() + statePtrIndex, GetConfigTypeSize(get<3>(allStates[type])));

                                float tmpValue = cndeValue;
                                memcpy(reinterpret_cast<char*>(robotStatePkg.get()) + get<1>(allStates[type]), &tmpValue, sizeof(float));
                            }
                            else if (get<2>(allStates[type]) == "UINT16") //AI、AO。0-100.0   0-4095
                            {
                                double cndeValue = 0;
                                memcpy(&cndeValue, pkg.data.data() + statePtrIndex, GetConfigTypeSize(get<3>(allStates[type])));

                                uint16_t tmpValue = cndeValue;
                                memcpy(reinterpret_cast<char*>(robotStatePkg.get()) + get<1>(allStates[type]), &tmpValue, sizeof(uint16_t));  
                            }
                            statePtrIndex += GetConfigTypeSize(get<3>(allStates[type]));
                        }
                        else if (get<3>(allStates[type]).find("UINT8_") != std::string::npos)
                        {
                            statePtrIndex += GetConfigTypeSize(get<3>(allStates[type]));
                        }
                        else if (get<3>(allStates[type]).find("DOUBLE_") != std::string::npos)
                        {
                            statePtrIndex += GetConfigTypeSize(get<3>(allStates[type]));
                        }
                        else if (get<3>(allStates[type]).find("INT32_") != std::string::npos)
                        {
                            if (get<2>(allStates[type]).find("UINT16_") != std::string::npos)
                            {
                                int cndeValue[4] = { 0 };
                                memcpy(cndeValue, pkg.data.data() + statePtrIndex, GetConfigTypeSize(get<3>(allStates[type])));

                                int cndeNum = GetConfigTypeSize(get<3>(allStates[type])) / 4;
                                int structNum = GetConfigTypeSize(get<2>(allStates[type])) / 2;

                                uint16_t tmpValue[4] = { 0 };
                                if (cndeNum == structNum)
                                {
                                    for (int i = 0; i < cndeNum; i++)
                                    {
                                        tmpValue[i] = cndeValue[i];    
                                    }
                                }

                                memcpy(reinterpret_cast<char*>(robotStatePkg.get()) + get<1>(allStates[type]), &tmpValue, GetConfigTypeSize(get<2>(allStates[type])));
                            }
                            else if (get<2>(allStates[type]).find("UINT16,UINT8,UINT8,UINT8,UINT8,UINT8,UINT6") != std::string::npos)  //机器人时间戳，特殊处理一下
                            {                                      
                                int robotTime[7] = { 0 };
                                memcpy(robotTime, pkg.data.data() + statePtrIndex, GetConfigTypeSize(get<3>(allStates[type])));
                                robotStatePkg->robotTime.year = robotTime[0];
                                robotStatePkg->robotTime.mouth = robotTime[1];
                                robotStatePkg->robotTime.day = robotTime[2];
                                robotStatePkg->robotTime.hour = robotTime[3];
                                robotStatePkg->robotTime.minute = robotTime[4];
                                robotStatePkg->robotTime.second = robotTime[5];
                                robotStatePkg->robotTime.millisecond = robotTime[6];
                            }

                            statePtrIndex += GetConfigTypeSize(get<3>(allStates[type]));   
                        }
                        else if (get<3>(allStates[type]).find("DOUBLE,DOUBLE") != std::string::npos)  //AIAO
                        {
                            if (get<2>(allStates[type]).find("UINT16_2") != std::string::npos)
                            {
                                double cndeValue[2] = { 0.0 };
                                memcpy(cndeValue, pkg.data.data() + statePtrIndex, GetConfigTypeSize(get<3>(allStates[type])));

                                uint16_t tmpValue[2] = { 0 };
                                tmpValue[0] = cndeValue[0];
                                tmpValue[1] = cndeValue[1];
                                memcpy(reinterpret_cast<char*>(robotStatePkg.get()) + get<1>(allStates[type]), &tmpValue, GetConfigTypeSize(get<2>(allStates[type])));
                            }
                            statePtrIndex += GetConfigTypeSize(get<3>(allStates[type]));
                        }
                        else
                        {
                            statePtrIndex += GetConfigTypeSize(get<3>(allStates[type]));
                            logger_error("Robot cnde state type error. %s %s", get<1>(allStates[type]), get<3>(allStates[type]));
                        }
                    }
                    break;
                }

                default:
                    break;
            }
        }
    }
    rtClient->Close();
}


int FRCNDEClient::SendCNDEOutputConfig()
{
    CNDE_PKG startPkg = {};
    startPkg.count = sendCount;
    sendCount++;
    startPkg.type = CNDE_FRAME_TYPE_OUTPUT_CONFIG;
    char periodByte[2] = { 0 };
    Int16ToByte(robotStatePeriod, periodByte);
    startPkg.data.insert(startPkg.data.end(), periodByte, periodByte + sizeof(periodByte) / sizeof(periodByte[0]));

    std::string configNameListStr;

    for (int i = 0; i < configStates.size(); i++) 
    {
        auto it = allStates.find(configStates[i]);
        if (it != allStates.end()) 
        {
            std::string names = std::get<0>(it->second); 
            if (i > 0)
            {
                configNameListStr += ",";
            }
            configNameListStr += names;
        }
    }

    //cout << "config list is " << configNameListStr << endl;

    startPkg.data.insert(startPkg.data.end(), configNameListStr.begin(), configNameListStr.end());
    startPkg.len = configNameListStr.length() + 2;
    std::vector<char> startFrame = CNDEPkgToFrame(startPkg);

    int rtn = rtClient->Send(startFrame.data(), startFrame.size());
    if (rtn <= 0)
    {
        logger_error("CNDE Send output config pkg Failed;  %d", rtn);
        return ERR_SOCKET_SEND_FAILED;  //发送失败
    }

    char pkgBuf[CNDE_MAX_PKG_SIZE] = {};
    CNDE_PKG pkg = {};

    while (robotStateRunFlag)
    {
        int recvLen = rtClient->RecvCNDEPkg(pkgBuf);
        if (recvLen < 0)
        {
            logger_error("CNDE recv outputconfig rpy pkg Failed; %d", recvLen);
            *sockComErr = ERR_SOCKET_COM_FAILED;
            return ERR_SOCKET_RECV_FAILED;
        }
        else if (recvLen == 0)  //发生重连，需要再次发送开始指令帧
        {
            rtClient->Send(startFrame.data(), startFrame.size());
        }
        else
        {
            pkg.Clear();
            std::vector<char> frame(pkgBuf, pkgBuf + recvLen);
            int rtn = FrameToCNDEPkg(frame, pkg);
            memset(pkgBuf, 0, CNDE_MAX_PKG_SIZE);
            switch (pkg.type)
            {
            case CNDE_FRAME_TYPE_MESSAGE:
            {
                if (pkg.data[0] == 0x00) //消息类型为“成功”
                {
                    return 0;
                }
                else 
                {
                    logger_error("CNDE states config Failed; %s", (char*)(pkg.data.data() + 1));
                    if (pkg.data.size() > 1) 
                    {
                        std::string resultStr(pkg.data.data() + 1, pkg.data.size() - 1);
                        if (resultStr.find("NOT_FOUND") != std::string::npos) 
                        {
                            return ERR_STATE_INVALID;
                        }
                    }

                    return ERR_EXECUTION_FAILED;
                }
                break;
            }

            default:   //收到其它类型的消息，直接抛弃，并继续接受。通常不会有其他类型的消息
                break;
            }
        }
    }

    return 0;
}

int FRCNDEClient::SetCNDEStart()
{
    /*std::lock_guard<std::mutex> lock(recvCNDEPkgMutex);*/
    CNDE_PKG startPkg = {};
    startPkg.count = sendCount;
    sendCount++;
    startPkg.type = CNDE_FRAME_TYPE_START;
    startPkg.len = 0;

    std::vector<char> startFrame = CNDEPkgToFrame(startPkg);

    //for (int i = 0; i < startFrame.size(); i++)
    //{
    //    printf(" %x", (unsigned char)startFrame[i]);
    //}
    int rtn = rtClient->Send(startFrame.data(), startFrame.size());
    if (rtn <= 0)
    {
        logger_error("CNDE Send Start pkg Failed;  %d", rtn);
        return ERR_SOCKET_SEND_FAILED;  //发送失败
    }

    char pkgBuf[CNDE_MAX_PKG_SIZE] = {};
    CNDE_PKG pkg = {};

    while (robotStateRunFlag)
    {
        int recvLen = rtClient->RecvCNDEPkg(pkgBuf);
        if (recvLen < 0)
        {
            logger_error("CNDE recv stop rpy pkg Failed;");
            *sockComErr = ERR_SOCKET_COM_FAILED;
            return ERR_SOCKET_RECV_FAILED;
        }
        else if (recvLen == 0)  //发生重连，需要再次发送开始指令帧
        {
            rtClient->Send(startFrame.data(), startFrame.size());
            logger_error("send another start after reconnect;");
        }
        else
        {
            pkg.Clear();
            std::vector<char> frame(pkgBuf, pkgBuf + recvLen);
            int rtn = FrameToCNDEPkg(frame, pkg);
            memset(pkgBuf, 0, CNDE_MAX_PKG_SIZE);
            switch (pkg.type)
            {
                case CNDE_FRAME_TYPE_MESSAGE:
                {
                    if (pkg.data[0] == 0x00)
                    {
                        return 0;
                    }
                    else
                    {
                        return ERR_CNDE_STATES_START_FAILED;
                    }
                    break;
                }

            default:   //收到其它类型的消息，直接抛弃，并继续接受。通常不会有其他类型的消息
                break;
            }
        }
    }

    return 0;
}

int FRCNDEClient::SetCNDEStop()
{
    std::lock_guard<std::mutex> lock(recvCNDEPkgMutex);

    CNDE_PKG startPkg = {};
    startPkg.count = sendCount;
    sendCount++;
    startPkg.type = CNDE_FRAME_TYPE_STOP;
    startPkg.len = 0;

    std::vector<char> startFrame = CNDEPkgToFrame(startPkg);
    int rtn = rtClient->Send(startFrame.data(), startFrame.size());
    if (rtn != 0)
    {
        logger_error("CNDE Send Stop pkg Failed;");
        return -1;  //发送失败
    }

    char pkgBuf[CNDE_MAX_PKG_SIZE] = {};
    CNDE_PKG pkg = {};

    while (robotStateRunFlag)
    {
        int recvLen = rtClient->RecvCNDEPkg(pkgBuf);
        if (recvLen < 0)
        {
            logger_error("CNDE recv stop rpy pkg Failed;");
            *sockComErr = ERR_SOCKET_COM_FAILED;
            return -1;
        }
        else if (recvLen == 0)  //发生重连，需要再次发送停止指令帧
        {
            rtClient->Send(startFrame.data(), startFrame.size());
        }
        else
        {
            pkg.Clear();
            std::vector<char> frame(pkgBuf, pkgBuf + recvLen);
            int rtn = FrameToCNDEPkg(frame, pkg);
            memset(pkgBuf, 0, CNDE_MAX_PKG_SIZE);
            switch (pkg.type)
            {
                case CNDE_FRAME_TYPE_MESSAGE:
                {
                    if (pkg.data[0] == 0x00)
                    {
                        return 0;
                    }
                    else
                    {
                        logger_error("CNDE Stop Failed;");
                        return -2;
                    }
                    break;
                }

                default:   //这里大概率会收到几帧状态数据，直接抛弃
                    break;
            }
        }
    }

    return 0;
}

int FRCNDEClient::SetCNDEStateConfig(vector<RobotState> states, int period)
{
    if (states.size() == 0)
    {
        return ERR_NEED_AT_LEAST_ONE_STATE;
    }

    for (auto state : states)
    {
        if (allStates.find(state) == allStates.end())
        {
            return ERR_STATE_INVALID;
        }
    }

    if (period < 8 || period > 1000)
    {
        return ERR_PARAM_VALUE;
    }

    configStates.clear();
    configStates = states;
    robotStatePeriod = period;
    return 0;
}

int FRCNDEClient::AddCNDEState(RobotState state)
{
    if (allStates.find(state) == allStates.end()) 
    {
        return ERR_STATE_INVALID;
    }
   
    auto it = find(configStates.begin(), configStates.end(), state);
    if (it != configStates.end())
    {
        return ERR_STATE_ALREADY_EXISTS;
    }

    configStates.push_back(state);
    return 0;
}

int FRCNDEClient::DeleteCNDEState(RobotState state)
{
    auto it = std::find(configStates.begin(), configStates.end(), state);
    if (it == configStates.end())
    {
        return ERR_STATE_INVALID;
    }

    if (configStates.size() == 1)
    {
        return  ERR_NEED_AT_LEAST_ONE_STATE;
    }

    configStates.erase(std::remove(configStates.begin(), configStates.end(), state), configStates.end());
    return 0;
}

int FRCNDEClient::SetCNDEStatePeriod(int period)
{
    if (period < 8 || period > 1000)
    {
        return ERR_PARAM_VALUE;
    }
    robotStatePeriod = period;

    return 0;
}

int FRCNDEClient::GetCNDEStateConfig(std::vector<RobotState>& states,  int& period)
{
    states.clear();
    states = configStates;
    period = robotStatePeriod;
    return 0;
}

void FRCNDEClient::InitAllStates()
{
    /*allStates.insert({ RobotState::FrameHead,  std::make_tuple("frame_head", offsetof(ROBOT_STATE_PKG, frame_head), "UINT16", "UINT16") });
    allStates.insert({ RobotState::FrameCnt,  std::make_tuple("frame_cnt", offsetof(ROBOT_STATE_PKG, frame_cnt), "UINT8", "UINT8") });
    allStates.insert({ RobotState::DataLen,  std::make_tuple("data_len", offsetof(ROBOT_STATE_PKG, data_len), "UINT16", "UINT16") });*/
    allStates.insert({ RobotState::ProgramState,  std::make_tuple("program_state", offsetof(ROBOT_STATE_PKG, program_state), "UINT8", "UINT8") });
    allStates.insert({ RobotState::RobotState,  std::make_tuple("robot_state", offsetof(ROBOT_STATE_PKG, robot_state), "UINT8", "UINT8") });
    allStates.insert({ RobotState::MainCode,  std::make_tuple("main_code", offsetof(ROBOT_STATE_PKG, main_code), "INT32", "INT32") });
    allStates.insert({ RobotState::SubCode,  std::make_tuple("sub_code", offsetof(ROBOT_STATE_PKG, sub_code), "INT32", "INT32") });
    allStates.insert({ RobotState::RobotMode,  std::make_tuple("robot_mode", offsetof(ROBOT_STATE_PKG, robot_mode), "UINT8", "UINT8") });
    allStates.insert({ RobotState::JointCurPos,  std::make_tuple("actual_joint_pos", offsetof(ROBOT_STATE_PKG, jt_cur_pos), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::ToolCurPos,  std::make_tuple("actual_TCP_pos", offsetof(ROBOT_STATE_PKG, tl_cur_pos), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::FlangeCurPos,  std::make_tuple("actual_flange_pos", offsetof(ROBOT_STATE_PKG, flange_cur_pos), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::ActualJointVel,  std::make_tuple("actual_joint_vel", offsetof(ROBOT_STATE_PKG, actual_qd), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::ActualJointAcc,  std::make_tuple("actual_joint_acc", offsetof(ROBOT_STATE_PKG, actual_qdd), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::TargetTCPCmpSpeed,  std::make_tuple("target_TCP_cmpvel", offsetof(ROBOT_STATE_PKG, target_TCP_CmpSpeed), "DOUBLE_2", "DOUBLE_2") });
    allStates.insert({ RobotState::TargetTCPSpeed,  std::make_tuple("target_TCP_vel", offsetof(ROBOT_STATE_PKG, target_TCP_Speed), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::ActualTCPCmpSpeed,  std::make_tuple("actual_TCP_cmpvel", offsetof(ROBOT_STATE_PKG, actual_TCP_CmpSpeed), "DOUBLE_2", "DOUBLE_2") });
    allStates.insert({ RobotState::ActualTCPSpeed,  std::make_tuple("actual_TCP_vel", offsetof(ROBOT_STATE_PKG, actual_TCP_Speed), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::ActualJointTorque,  std::make_tuple("actual_joint_torque", offsetof(ROBOT_STATE_PKG, jt_cur_tor), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::Tool,  std::make_tuple("tool_id", offsetof(ROBOT_STATE_PKG, tool), "INT32", "INT32") });
    allStates.insert({ RobotState::User,  std::make_tuple("wobj_id", offsetof(ROBOT_STATE_PKG, user), "INT32", "INT32") });
    allStates.insert({ RobotState::ClDgtOutputH,  std::make_tuple("cfg_DO_box", offsetof(ROBOT_STATE_PKG, cl_dgt_output_h), "UINT8", "UINT8") });
    allStates.insert({ RobotState::ClDgtOutputL,  std::make_tuple("std_DO_box", offsetof(ROBOT_STATE_PKG, cl_dgt_output_l), "UINT8", "UINT8") });
    allStates.insert({ RobotState::TlDgtOutputL,  std::make_tuple("cfg_DO_tool", offsetof(ROBOT_STATE_PKG, tl_dgt_output_l), "UINT8", "UINT8") });
    allStates.insert({ RobotState::ClDgtInputH,  std::make_tuple("cfg_DI_box", offsetof(ROBOT_STATE_PKG, cl_dgt_input_h), "UINT8", "UINT8") });
    allStates.insert({ RobotState::ClDgtInputL,  std::make_tuple("std_DI_box", offsetof(ROBOT_STATE_PKG, cl_dgt_input_l), "UINT8", "UINT8") });
    allStates.insert({ RobotState::TlDgtInputL,  std::make_tuple("cfg_DI_tool", offsetof(ROBOT_STATE_PKG, tl_dgt_input_l), "UINT8", "UINT8") });
    allStates.insert({ RobotState::ClAnalogInput,  std::make_tuple("std_AI0_box,std_AI1_box", offsetof(ROBOT_STATE_PKG, cl_analog_input), "UINT16_2", "DOUBLE,DOUBLE") });  // 结构体是UINT16[2]，输出是DOUBLE
    allStates.insert({ RobotState::TlAnalogInput,  std::make_tuple("std_AI_tool", offsetof(ROBOT_STATE_PKG, tl_anglog_input), "UINT16", "DOUBLE") });
    allStates.insert({ RobotState::FtSensorRawData,  std::make_tuple("ft_sensor_raw_data", offsetof(ROBOT_STATE_PKG, ft_sensor_raw_data), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::FtSensorData,  std::make_tuple("ft_sensor_data", offsetof(ROBOT_STATE_PKG, ft_sensor_data), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::FtSensorActive,  std::make_tuple("ft_sensor_active", offsetof(ROBOT_STATE_PKG, ft_sensor_active), "UINT8", "UINT8") });
    allStates.insert({ RobotState::EmergencyStop,  std::make_tuple("emergency_stop", offsetof(ROBOT_STATE_PKG, EmergencyStop), "UINT8", "UINT8") });
    allStates.insert({ RobotState::MotionDone,  std::make_tuple("motion_done", offsetof(ROBOT_STATE_PKG, motion_done), "INT32", "INT32") });
    allStates.insert({ RobotState::GripperMotiondone,  std::make_tuple("gripper_motion_done", offsetof(ROBOT_STATE_PKG, gripper_motiondone), "UINT8", "UINT8") });
    allStates.insert({ RobotState::McQueueLen,  std::make_tuple("motion_queue_len", offsetof(ROBOT_STATE_PKG, mc_queue_len), "INT32", "INT32") });
    allStates.insert({ RobotState::CollisionState,  std::make_tuple("collision_state", offsetof(ROBOT_STATE_PKG, collisionState), "UINT8", "UINT8") });
    allStates.insert({ RobotState::TrajectoryPnum,  std::make_tuple("trajectory_pnum", offsetof(ROBOT_STATE_PKG, trajectory_pnum), "INT32", "INT32") });
    allStates.insert({ RobotState::SafetyStop0State,  std::make_tuple("safety_stop0_state", offsetof(ROBOT_STATE_PKG, safety_stop0_state), "UINT8", "UINT8") });
    allStates.insert({ RobotState::SafetyStop1State,  std::make_tuple("safety_stop1_state", offsetof(ROBOT_STATE_PKG, safety_stop1_state), "UINT8", "UINT8") });
    allStates.insert({ RobotState::GripperFaultId,  std::make_tuple("gripper_fault_id", offsetof(ROBOT_STATE_PKG, gripper_fault_id), "UINT8", "UINT8") });
    allStates.insert({ RobotState::GripperFault,  std::make_tuple("gripper_fault", offsetof(ROBOT_STATE_PKG, gripper_fault), "UINT16", "INT32") });
    allStates.insert({ RobotState::GripperActive,  std::make_tuple("gripper_active", offsetof(ROBOT_STATE_PKG, gripper_active), "UINT16", "INT32") });
    allStates.insert({ RobotState::GripperPosition,  std::make_tuple("gripper_position", offsetof(ROBOT_STATE_PKG, gripper_position), "UINT8", "UINT8") });
    allStates.insert({ RobotState::GripperSpeed,  std::make_tuple("gripper_speed", offsetof(ROBOT_STATE_PKG, gripper_speed), "INT8", "INT32") });
    allStates.insert({ RobotState::GripperCurrent,  std::make_tuple("gripper_current", offsetof(ROBOT_STATE_PKG, gripper_current), "INT8", "INT32") });
    allStates.insert({ RobotState::GripperTemp,  std::make_tuple("gripper_temp", offsetof(ROBOT_STATE_PKG, gripper_temp), "INT32", "INT32") });
    allStates.insert({ RobotState::GripperVoltage,  std::make_tuple("gripper_voltage", offsetof(ROBOT_STATE_PKG, gripper_voltage), "INT32", "INT32") });
    allStates.insert({ RobotState::AuxState,  std::make_tuple("aux_axis_state", offsetof(ROBOT_STATE_PKG, aux_state), "UINT8_25", "UINT8_25") });
    allStates.insert({ RobotState::ExtAxisStatus,  std::make_tuple("exaxis_status", offsetof(ROBOT_STATE_PKG, extAxisStatus), "UINT8_116", "UINT8_116") });
    allStates.insert({ RobotState::ExtDIState,  std::make_tuple("ext_DI_state", offsetof(ROBOT_STATE_PKG, extDIState), "UINT16_8", "UINT8_16") });
    allStates.insert({ RobotState::ExtDOState,  std::make_tuple("ext_DO_state", offsetof(ROBOT_STATE_PKG, extDOState), "UINT16_8", "UINT8_16") });
    allStates.insert({ RobotState::ExtAIState,  std::make_tuple("ext_AI_state", offsetof(ROBOT_STATE_PKG, extAIState), "UINT16_4", "INT32_4") });
    allStates.insert({ RobotState::ExtAOState,  std::make_tuple("ext_AO_state", offsetof(ROBOT_STATE_PKG, extAOState), "UINT16_4", "INT32_4") });
    allStates.insert({ RobotState::RbtEnableState,  std::make_tuple("rbt_enable_state", offsetof(ROBOT_STATE_PKG, rbtEnableState), "INT32", "INT32") });
    allStates.insert({ RobotState::JointDriverTorque,  std::make_tuple("joint_driver_torque", offsetof(ROBOT_STATE_PKG, jointDriverTorque), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::JointDriverTemperature,  std::make_tuple("actual_joint_temp", offsetof(ROBOT_STATE_PKG, jointDriverTemperature), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::RobotTime,  std::make_tuple("robot_time", offsetof(ROBOT_STATE_PKG, robotTime), "UINT16,UINT8,UINT8,UINT8,UINT8,UINT8,UINT6", "INT32_7") });
    allStates.insert({ RobotState::SoftwareUpgradeState,  std::make_tuple("software_upgrade_state", offsetof(ROBOT_STATE_PKG, softwareUpgradeState), "INT32", "INT32") });
    allStates.insert({ RobotState::EndLuaErrCode,  std::make_tuple("end_lua_err_code", offsetof(ROBOT_STATE_PKG, endLuaErrCode), "UINT16", "INT32") });
    allStates.insert({ RobotState::ClAnalogOutput,  std::make_tuple("std_AO0_box,std_AO1_box", offsetof(ROBOT_STATE_PKG, cl_analog_output), "UINT16_2", "DOUBLE,DOUBLE") });
    allStates.insert({ RobotState::TlAnalogOutput,  std::make_tuple("std_AO_tool", offsetof(ROBOT_STATE_PKG, tl_analog_output), "UINT16", "DOUBLE") });
    allStates.insert({ RobotState::GripperRotNum,  std::make_tuple("rotating_gripper_num", offsetof(ROBOT_STATE_PKG, gripperRotNum), "FLOAT", "DOUBLE") });
    allStates.insert({ RobotState::GripperRotSpeed,  std::make_tuple("rotating_gripper_speed", offsetof(ROBOT_STATE_PKG, gripperRotSpeed), "UINT8", "UINT8") });
    allStates.insert({ RobotState::GripperRotTorque,  std::make_tuple("rotating_gripper_tor", offsetof(ROBOT_STATE_PKG, gripperRotTorque), "UINT8", "UINT8") });
    allStates.insert({ RobotState::WeldingBreakOffState,  std::make_tuple("weld_break_off_state,weld_arc_state", offsetof(ROBOT_STATE_PKG, weldingBreakOffState), "UINT8,UINT8", "UINT8,UINT8") });
    allStates.insert({ RobotState::TargetJointTorque,  std::make_tuple("target_joint_torque", offsetof(ROBOT_STATE_PKG, jt_tgt_tor), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::SmartToolState,  std::make_tuple("smarttool_state", offsetof(ROBOT_STATE_PKG, smartToolState), "INT32", "UINT32") });
    allStates.insert({ RobotState::WideVoltageCtrlBoxTemp,  std::make_tuple("wide_voltage_ctrl_box_temp", offsetof(ROBOT_STATE_PKG, wideVoltageCtrlBoxTemp), "FLOAT", "DOUBLE") });
    allStates.insert({ RobotState::WideVoltageCtrlBoxFanCurrent,  std::make_tuple("wide_voltage_ctrl_box_fan_current", offsetof(ROBOT_STATE_PKG, wideVoltageCtrlBoxFanCurrent), "UINT16", "INT32") });
    allStates.insert({ RobotState::ToolCoord,  std::make_tuple("tool_coord", offsetof(ROBOT_STATE_PKG, toolCoord), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::WobjCoord,  std::make_tuple("wobj_coord", offsetof(ROBOT_STATE_PKG, wobjCoord), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::ExtoolCoord,  std::make_tuple("exTool_coord", offsetof(ROBOT_STATE_PKG, extoolCoord), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::ExAxisCoord,  std::make_tuple("exAxis_coord", offsetof(ROBOT_STATE_PKG, exAxisCoord), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::Load,  std::make_tuple("payload", offsetof(ROBOT_STATE_PKG, load), "DOUBLE", "DOUBLE") });
    allStates.insert({ RobotState::LoadCog,  std::make_tuple("pay_cog", offsetof(ROBOT_STATE_PKG, loadCog), "DOUBLE_3", "DOUBLE_3") });
    allStates.insert({ RobotState::LastServoTarget,  std::make_tuple("last_servoJ_target", offsetof(ROBOT_STATE_PKG, lastServoTarget), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::ServoJCmdNum,  std::make_tuple("servoJ_cmd_num", offsetof(ROBOT_STATE_PKG, servoJCmdNum), "INT32", "INT32") });
    allStates.insert({ RobotState::TargetJointPos,  std::make_tuple("target_joint_pos", offsetof(ROBOT_STATE_PKG, targetJointPos), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::TargetJointVel,  std::make_tuple("target_joint_vel", offsetof(ROBOT_STATE_PKG, targetJointVel), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::TargetJointAcc,  std::make_tuple("target_joint_acc", offsetof(ROBOT_STATE_PKG, targetJointAcc), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::TargetJointCurrent,  std::make_tuple("target_joint_current", offsetof(ROBOT_STATE_PKG, targetJointCurrent), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::ActualJointCurrent,  std::make_tuple("actual_joint_current", offsetof(ROBOT_STATE_PKG, actualJointCurrent), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::ActualTCPForce,  std::make_tuple("actual_TCP_force", offsetof(ROBOT_STATE_PKG, actualTCPForce), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::TargetTCPPos,  std::make_tuple("target_TCP_pos", offsetof(ROBOT_STATE_PKG, targetTCPPos), "DOUBLE_6", "DOUBLE_6") });
    allStates.insert({ RobotState::CollisionLevel,  std::make_tuple("collision_level", offsetof(ROBOT_STATE_PKG, collisionLevel), "UINT8_6", "UINT8_6") });
    allStates.insert({ RobotState::SpeedScaleManual,  std::make_tuple("speed_scaling_man", offsetof(ROBOT_STATE_PKG, speedScaleManual), "DOUBLE", "DOUBLE") });
    allStates.insert({ RobotState::SpeedScaleAuto,  std::make_tuple("speed_scaling_auto", offsetof(ROBOT_STATE_PKG, speedScaleAuto), "DOUBLE", "DOUBLE") });
    allStates.insert({ RobotState::LuaLineNum,  std::make_tuple("line_number", offsetof(ROBOT_STATE_PKG, luaLineNum), "INT32", "INT32") });
    allStates.insert({ RobotState::AbnomalStop,  std::make_tuple("abnormal_stop", offsetof(ROBOT_STATE_PKG, abnomalStop), "UINT8", "UINT8") });
    allStates.insert({ RobotState::CurrentLuaFileName,  std::make_tuple("cur_lua_file_name", offsetof(ROBOT_STATE_PKG, currentLuaFileName), "UINT8_256", "UINT8_256") });
    allStates.insert({ RobotState::ProgramTotalLine,  std::make_tuple("prog_total_line", offsetof(ROBOT_STATE_PKG, programTotalLine), "UINT8", "UINT8") });
    allStates.insert({ RobotState::SafetyBoxSingal,  std::make_tuple("safety_box_signal", offsetof(ROBOT_STATE_PKG, safetyBoxSingal), "UINT8_6", "UINT8_6") });
    allStates.insert({ RobotState::WeldVoltage,  std::make_tuple("welding_voltage", offsetof(ROBOT_STATE_PKG, weldVoltage), "DOUBLE", "DOUBLE") });
    allStates.insert({ RobotState::WeldCurrent,  std::make_tuple("welding_current", offsetof(ROBOT_STATE_PKG, weldCurrent), "DOUBLE", "DOUBLE") });
    allStates.insert({ RobotState::WeldTrackVel,  std::make_tuple("welding_track_speed", offsetof(ROBOT_STATE_PKG, weldTrackVel), "DOUBLE", "DOUBLE") });
    allStates.insert({ RobotState::TpdException,  std::make_tuple("tpd_exception", offsetof(ROBOT_STATE_PKG, tpdException), "UINT8", "UINT8") });
    allStates.insert({ RobotState::AlarmRebootRobot,  std::make_tuple("alarm_reboot_robot", offsetof(ROBOT_STATE_PKG, alarmRebootRobot), "UINT8", "UINT8") });
    allStates.insert({ RobotState::ModbusMasterConnect,  std::make_tuple("modbus_master_connect", offsetof(ROBOT_STATE_PKG, modbusMasterConnect), "UINT8", "UINT8") });
    allStates.insert({ RobotState::ModbusSlaveConnect,  std::make_tuple("modbus_slave_connect", offsetof(ROBOT_STATE_PKG, modbusSlaveConnect), "UINT8", "UINT8") });
    allStates.insert({ RobotState::BtnBoxStopSignal,  std::make_tuple("btn_box_stop_signal", offsetof(ROBOT_STATE_PKG, btnBoxStopSignal), "UINT8", "UINT8") });
    allStates.insert({ RobotState::DragAlarm,  std::make_tuple("drag_alarm", offsetof(ROBOT_STATE_PKG, dragAlarm), "UINT8", "UINT8") });
    allStates.insert({ RobotState::SafetyDoorAlarm,  std::make_tuple("safety_door_alarm", offsetof(ROBOT_STATE_PKG, safetyDoorAlarm), "UINT8", "UINT8") });
    allStates.insert({ RobotState::SafetyPlaneAlarm,  std::make_tuple("safety_plane_alarm", offsetof(ROBOT_STATE_PKG, safetyPlaneAlarm), "UINT8", "UINT8") });
    allStates.insert({ RobotState::MotonAlarm,  std::make_tuple("motion_alarm", offsetof(ROBOT_STATE_PKG, motonAlarm), "UINT8", "UINT8") });
    allStates.insert({ RobotState::InterfaceAlarm,  std::make_tuple("interfere_alarm", offsetof(ROBOT_STATE_PKG, interfaceAlarm), "UINT8", "UINT8") });
    allStates.insert({ RobotState::UdpCmdState,  std::make_tuple("udp_cmd_state", offsetof(ROBOT_STATE_PKG, udpCmdState), "INT32", "INT32") });
    allStates.insert({ RobotState::WeldReadyState,  std::make_tuple("weld_ready_state", offsetof(ROBOT_STATE_PKG, weldReadyState), "UINT8", "UINT8") });
    allStates.insert({ RobotState::AlarmCheckEmergStopBtn,  std::make_tuple("alarm_check_emerg_stop_btn", offsetof(ROBOT_STATE_PKG, alarmCheckEmergStopBtn), "UINT8", "UINT8") });
    allStates.insert({ RobotState::TsTmCmdComError,  std::make_tuple("ts_tm_cmd_com_error", offsetof(ROBOT_STATE_PKG, tsTmCmdComError), "UINT8", "UINT8") });
    allStates.insert({ RobotState::TsTmStateComError,  std::make_tuple("ts_tm_state_com_error", offsetof(ROBOT_STATE_PKG, tsTmStateComError), "UINT8", "UINT8") });
    allStates.insert({ RobotState::CtrlBoxError,  std::make_tuple("ctrl_box_error", offsetof(ROBOT_STATE_PKG, ctrlBoxError), "INT32", "INT32") });
    allStates.insert({ RobotState::SafetyDataState,  std::make_tuple("safety_data_state", offsetof(ROBOT_STATE_PKG, safetyDataState), "UINT8", "UINT8") });
    allStates.insert({ RobotState::ForceSensorErrState,  std::make_tuple("force_sensor_err_state", offsetof(ROBOT_STATE_PKG, forceSensorErrState), "UINT8", "UINT8") });
    allStates.insert({ RobotState::CtrlOpenLuaErrCode,  std::make_tuple("ctrl_open_lua_errcode", offsetof(ROBOT_STATE_PKG, ctrlOpenLuaErrCode), "UINT8_4", "UINT8_4") });
    allStates.insert({ RobotState::StrangePosFlag,  std::make_tuple("strange_pos_flag", offsetof(ROBOT_STATE_PKG, strangePosFlag), "UINT8", "UINT8") });
    allStates.insert({ RobotState::Alarm,  std::make_tuple("alarm", offsetof(ROBOT_STATE_PKG, alarm), "UINT8", "UINT8") });
    allStates.insert({ RobotState::DriverAlarm,  std::make_tuple("dr_alarm", offsetof(ROBOT_STATE_PKG, driverAlarm), "UINT8", "UINT8") });
    allStates.insert({ RobotState::AliveSlaveNumError,  std::make_tuple("alive_slave_num_error", offsetof(ROBOT_STATE_PKG, aliveSlaveNumError), "UINT8", "UINT8") });
    allStates.insert({ RobotState::SlaveComError,  std::make_tuple("slave_com_error", offsetof(ROBOT_STATE_PKG, slaveComError), "UINT8_8", "UINT8_8") });
    allStates.insert({ RobotState::CmdPointError,  std::make_tuple("cmd_point_error", offsetof(ROBOT_STATE_PKG, cmdPointError), "UINT8", "UINT8") });
    allStates.insert({ RobotState::IOError,  std::make_tuple("IO_error", offsetof(ROBOT_STATE_PKG, IOError), "UINT8", "UINT8") });
    allStates.insert({ RobotState::GripperError,  std::make_tuple("gripper_error", offsetof(ROBOT_STATE_PKG, gripperError), "UINT8", "UINT8") });
    allStates.insert({ RobotState::FileError,  std::make_tuple("file_error", offsetof(ROBOT_STATE_PKG, fileError), "UINT8", "UINT8") });
    allStates.insert({ RobotState::ParaError,  std::make_tuple("para_error", offsetof(ROBOT_STATE_PKG, paraError), "UINT8", "UINT8") });
    allStates.insert({ RobotState::ExaxisOutLimitError,  std::make_tuple("exaxis_out_slimit_error", offsetof(ROBOT_STATE_PKG, exaxisOutLimitError), "UINT8", "UINT8") });
    allStates.insert({ RobotState::DriverComError,  std::make_tuple("dr_com_err", offsetof(ROBOT_STATE_PKG, driverComError), "UINT8_6", "UINT8_6") });
    allStates.insert({ RobotState::DriverError,  std::make_tuple("dr_err", offsetof(ROBOT_STATE_PKG, driverError), "UINT8", "UINT8") });
    allStates.insert({ RobotState::OutSoftLimitError,  std::make_tuple("out_sflimit_err", offsetof(ROBOT_STATE_PKG, outSoftLimitError), "UINT8", "UINT8") });
    allStates.insert({ RobotState::AxleGenComData,  std::make_tuple("axle_gen_com_data", offsetof(ROBOT_STATE_PKG, axleGenComData), "UINT8_130", "UINT8_130") });
    //allStates.insert({ RobotState::CheckSum,  std::make_tuple("check_sum", offsetof(ROBOT_STATE_PKG, check_sum), "UINT16", "UINT16") });
    allStates.insert({ RobotState::SocketConnTimeout,  std::make_tuple("socket_conn_timeout", offsetof(ROBOT_STATE_PKG, socketConnTimeout), "UINT8", "UINT8") });
    allStates.insert({ RobotState::SocketReadTimeout,  std::make_tuple("socket_read_timeout", offsetof(ROBOT_STATE_PKG, socketReadTimeout), "UINT8", "UINT8") });
    allStates.insert({ RobotState::TsWebStateComErr,  std::make_tuple("ts_web_state_com_err", offsetof(ROBOT_STATE_PKG, tsWebStateComErr), "UINT8", "UINT8") });
    
    configStates = {
    RobotState::ProgramState,
    RobotState::RobotState,
    RobotState::MainCode,
    RobotState::SubCode,
    RobotState::RobotMode,
    RobotState::JointCurPos,
    RobotState::ToolCurPos,
    RobotState::FlangeCurPos,
    RobotState::ActualJointVel,
    RobotState::ActualJointAcc,
    RobotState::TargetTCPCmpSpeed,
    RobotState::TargetTCPSpeed,
    RobotState::ActualTCPCmpSpeed,
    RobotState::ActualTCPSpeed,
    RobotState::ActualJointTorque,
    RobotState::Tool,
    RobotState::User,
    RobotState::ClDgtOutputH,
    RobotState::ClDgtOutputL,
    RobotState::TlDgtOutputL,
    RobotState::ClDgtInputH,
    RobotState::ClDgtInputL,
    RobotState::TlDgtInputL,
    RobotState::ClAnalogInput,
    RobotState::TlAnalogInput,
    RobotState::FtSensorRawData,
    RobotState::FtSensorData,
    RobotState::FtSensorActive,
    RobotState::EmergencyStop,
    RobotState::MotionDone,
    RobotState::GripperMotiondone,
    RobotState::McQueueLen,
    RobotState::CollisionState,
    RobotState::TrajectoryPnum,
    RobotState::SafetyStop0State,
    RobotState::SafetyStop1State,
    RobotState::GripperFaultId,
    RobotState::GripperFault,
    RobotState::GripperActive,
    RobotState::GripperPosition,
    RobotState::GripperSpeed,
    RobotState::GripperCurrent,
    RobotState::GripperTemp,
    RobotState::GripperVoltage,
    RobotState::AuxState,
    RobotState::ExtAxisStatus,
    RobotState::ExtDIState,
    RobotState::ExtDOState,
    RobotState::ExtAIState,
    RobotState::ExtAOState,
    RobotState::RbtEnableState,
    RobotState::JointDriverTorque,
    RobotState::JointDriverTemperature,
    RobotState::RobotTime,
    RobotState::SoftwareUpgradeState,
    RobotState::EndLuaErrCode,
    RobotState::ClAnalogOutput,
    RobotState::TlAnalogOutput,
    RobotState::GripperRotNum,
    RobotState::GripperRotSpeed,
    RobotState::GripperRotTorque,
    RobotState::WeldingBreakOffState,
    RobotState::TargetJointTorque,
    RobotState::SmartToolState,
    RobotState::WideVoltageCtrlBoxTemp,
    RobotState::WideVoltageCtrlBoxFanCurrent,
    RobotState::ToolCoord,
    RobotState::WobjCoord,
    RobotState::ExtoolCoord,
    RobotState::ExAxisCoord,
    RobotState::Load,
    RobotState::LoadCog,
    RobotState::LastServoTarget
    };
}

int FRCNDEClient::GetConfigTypeSize(string typeStr)
{
    if (typeStr.empty())
    {
        return -1;
    }

    int totalSize = 0;
    stringstream ss(typeStr);
    string item;

    while (getline(ss, item, ',')) 
    {
        // 去除空格
        size_t start = item.find_first_not_of(" \t");
        size_t end = item.find_last_not_of(" \t");

        if (start != string::npos) {
            string type = item.substr(start, end - start + 1);
            int typeSize = GetSingleConfigTypeSize(type);
            if (typeSize == -1) 
            {
                return -1;
            }
            totalSize += typeSize;
        }
    }

    return totalSize;
}

int FRCNDEClient::GetSingleConfigTypeSize(string typeStr)
{
    if (typeStr == "UINT8")
    {
        return 1;
    }
    else if (typeStr == "UINT16")
    {
        return 2;
    }
    else if (typeStr == "INT32")
    {
        return 4;
    }
    else if (typeStr == "UINT32")
    {
        return 4;
    }
    else if (typeStr == "DOUBLE")
    {
        return 8;
    }
    else if (typeStr.find("UINT8_") != std::string::npos)
    {
        size_t pos = typeStr.find('_');
        if (pos != std::string::npos)
        {
            std::string numStr = typeStr.substr(pos + 1);
            return std::stoi(numStr);
        }
        else
        {
            return -1;
        }
    }
    else if (typeStr.find("UINT16_") != std::string::npos)
    {
        size_t pos = typeStr.find('_');
        if (pos != std::string::npos)
        {
            std::string numStr = typeStr.substr(pos + 1);
            return 2 * std::stoi(numStr);
        }
        else
        {
            return -1;
        }
    }
    else if (typeStr.find("DOUBLE_") != std::string::npos)
    {
        size_t pos = typeStr.find('_');
        if (pos != std::string::npos)
        {
            std::string numStr = typeStr.substr(pos + 1);
            return 8 * std::stoi(numStr);
        }
        else
        {
            return -1;
        }
    }
    else if (typeStr.find("INT32_") != std::string::npos)
    {
        size_t pos = typeStr.find('_');
        if (pos != std::string::npos)
        {
            std::string numStr = typeStr.substr(pos + 1);
            return 4 * std::stoi(numStr);
        }
        else
        {
            return -1;
        }
    }
    else
    {
        return -1;
    }
}