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
 * @brief 定点摆动开始
 * @param [in] weaveNum 摆动编号[0-7]
 * @param [in] mode 0-工具坐标系；1-参考点
 * @param [in] refPoint 参考点笛卡尔坐标[x,y,z,a,b,c]
 * @param [in] weaveTime 摆动时间[s]
 * @return 错误码
 */
errno_t FRRobot::OriginPointWeaveStart(int weaveNum, int mode, DescPose refPoint, double weaveTime)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0][0] = weaveNum;
    param[0][1] = mode;
    param[0][2] = refPoint.tran.x;
    param[0][3] = refPoint.tran.y;
    param[0][4] = refPoint.tran.z;
    param[0][5] = refPoint.rpy.rx;
    param[0][6] = refPoint.rpy.ry;
    param[0][7] = refPoint.rpy.rz;
    param[0][8] = weaveTime;

    if (c.execute("OriginPointWeaveStart", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute OriginPointWeaveStart fail: %d.", errcode);
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
 * @brief 定点摆动结束
 * @return 错误码
 */
errno_t FRRobot::OriginPointWeaveEnd()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("OriginPointWeaveEnd", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute OriginPointWeaveEnd fail: %d.", errcode);
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