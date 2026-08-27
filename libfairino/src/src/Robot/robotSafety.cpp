#include "robot.h"
#include "robot_types.h"
#include "robot_error.h"
#include "logger.h"
#include "XmlRpc.h"
#include <stdio.h>
#include <string.h>
#include <cstdlib>
#include <iostream>
#include <chrono>
#include <fstream>
#include <sstream>
#include <iomanip>

using namespace std;
using namespace XmlRpc;

/**
 * @brief 获取安全配置参数校验和
 * @param [out] status 校验状态，0-有效，1-校验中，2-校验失败
 * @param [out] checksum 校验和 8位16进制
 * @return  错误码
 */
errno_t FRRobot::GetSafetyParamsCheckSum(int& status, uint32_t& checksum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetSafetyParamsCheckSum", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            status = (double)result[1];
            double tmpSum = (double)result[2];
            checksum = tmpSum;
        }
        else
        {
            logger_error("execute GetSafetyParamsCheckSum fail %d", errcode);
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
* @brief 安全操作密码校验
* @param [in] status 校验，0-开启，1-关闭
* @param [in] password 密码
* @return 错误码
*/
errno_t FRRobot::SafetyOPPasswordCheck(int status, std::string password)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = status;
    param[1] = password;

    if (c.execute("SafetyOPPasswordCheck", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SafetyOPPasswordCheck fail: %d.", errcode);
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
 * @brief 安全双通道CI功能配置
 * @param [in] ID 双通道ID; [0-3]
 * @param [in] config 功能配置; 0-无配置; 201-外部急停输入信号1; 202-外部急停输入信号2; 203-一级缩减模式; 204-二级缩减模式; 205-三级缩减模式;
                               206-常规停止; 207-安全墙1; 208-安全墙2; 209-安全墙3; 210-安全墙4; 211-安全墙5; 212-安全墙6; 213-安全墙7;
                               214-安全墙8; 215-安全停止重置;
 * @return 错误码
 */
errno_t FRRobot::SetSafetyDIConfig(int ID, int config)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = ID;
    param[1] = config;

    if (c.execute("SetSafetyDIConfig", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetSafetyDIConfig fail: %d.", errcode);
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
 * @brief 安全双通道CO功能配置
 * @param [in] ID 双通道ID; [0-3]
 * @param [in] config 功能配置; 0-无配置; 201-急停输出信号1; 202-急停输出信号2; 203-安全状态输出; 204-保护性停止状态输出; 205-机器人运动中;
                               206-机器人缩减模式; 207-机器人非缩减模式;
 * @return 错误码
 */
errno_t FRRobot::SetSafetyDOConfig(int ID, int config)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = ID;
    param[1] = config;

    if (c.execute("SetSafetyDOConfig", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetSafetyDOConfig fail: %d.", errcode);
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