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
* @brief 开启末端通用透传功能
* @param [in] 使能，0-关闭，1-开启
* @return  错误码
*/
errno_t FRRobot::SetAxleGenComEnable(int mode)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;
    param[0] = mode;

    if (c.execute("SetAxleGenComEnable", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetAxleGenComEnable fail: %d.", errcode);
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
* @brief 按长度获取周期数据
* @param [in] len，返回的长度
* @return  错误码
*/
errno_t FRRobot::GetAxleGenComCycleData(int len, int cycleData[130])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = len;

    if (c.execute("GetAxleGenComCycleData", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            for (int i = 0; i < len; i++)
            {
                cycleData[i] = int(result[i + 1]);
            }
        }
        else {
            logger_error("execute GetAxleGenComCycleData fail %d", errcode);
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
* @brief 末端发送非周期数据并等待应答
* @param [in] len_snd，发送的长度
* @param [in] sndBuff[]，发送数据
* @param [in] len_rcv，选择接受的长度
* @param [out] rcvBuff[]，应答的数据
* @return  错误码
*/
errno_t FRRobot::SndRcvAxleGenComCmdData(int lenSnd, int sndBuff[130], int lenRcv, int rcvData[130])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = lenSnd;
    for (int i = 0; i < lenSnd; i++)
    {
        param[1][i] = sndBuff[i];
    }
    param[2] = lenRcv;


    if (c.execute("SndRcvAxleGenComCmdData", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            for (int i = 0; i < lenRcv; i++)
            {
                rcvData[i] = int(result[i + 1]);
            }
        }
        else {
            logger_error("execute SndRcvAxleGenComCmdData fail %d", errcode);
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
 * @brief UDP扩展轴定位完成时间设置
 * @param [in] time 定位完成时间[ms]
 * @return 错误码
 */
errno_t FRRobot::SetExAxisCmdDoneTime(double time)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0][0] = time;
    

    if (c.execute("SetExAxisCmdDoneTime", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetExAxisCmdDoneTime fail: %d.", errcode);
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