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
 * @brief 设置可配置CI端口功能
 * @param [in] config CI0-CI7功能编码；
 * 0-无;1-起弧成功;2-焊机准备;3-传送带检测;4-暂停;5-恢复;6-启动;7-停止;
 8-暂停/恢复;9-启动/停止;10-脚踏拖动;11-移至作业原点;12-手自动切换;
 13-焊丝寻位成功;14-运动中断;15-启动主程序;16-启动倒带;17-启动确认;
 18-光电检测信号X;19-光电检测信号Y;20-外部急停输入信号1;21-外部急停输入信号2;
 22-一级缩减模式;23-二级缩减模式;24-三级缩减模式(停止);25-恢复焊接;26-终止焊接;
 27-辅助拖动开启;28-辅助拖动关闭;29-辅助拖动开启/关闭;30-清除所有错误;
 31-手自动切换(高低电平);32-使能;33-去使能;34-使能/去使能(上升下降沿);35-定点跟踪开始/结束
 * @return 错误码
 */
errno_t FRRobot::SetDIConfig(int config[8])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;
    for (int i = 0; i < 8; i++)
    {
        param[0][i] = config[i];
    }

    if (c.execute("SetDIConfig", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetDIConfig fail: %d.", errcode);
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
 * @brief 获取控制箱可配置CI端口功能
 * @param [in] config CI0-CI7功能编码；
 * 0-无;1-起弧成功;2-焊机准备;3-传送带检测;4-暂停;5-恢复;6-启动;7-停止;
 8-暂停/恢复;9-启动/停止;10-脚踏拖动;11-移至作业原点;12-手自动切换;
 13-焊丝寻位成功;14-运动中断;15-启动主程序;16-启动倒带;17-启动确认;
 18-光电检测信号X;19-光电检测信号Y;20-外部急停输入信号1;21-外部急停输入信号2;
 22-一级缩减模式;23-二级缩减模式;24-三级缩减模式(停止);25-恢复焊接;26-终止焊接;
 27-辅助拖动开启;28-辅助拖动关闭;29-辅助拖动开启/关闭;30-清除所有错误;
 31-手自动切换(高低电平);32-使能;33-去使能;34-使能/去使能(上升下降沿);35-定点跟踪开始/结束
 * @return 错误码
 */
errno_t FRRobot::GetDIConfig(int config[8])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetDIConfig", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            for (int i = 0; i < 8; i++)
            {
                config[i] = int(result[i + 1]);
            }
        }
        else 
        {
            logger_error("execute GetDIConfig fail %d", errcode);
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
 * @brief 设置可配置CO端口功能
 * @param [out] config CI0-CI7功能编码；
 * 0-无;1-机器人报错;2-机器人运动中;3-喷涂启停;4-喷涂清枪;5-送气信号;6-起弧信号;7-点动送丝;
    8-反向送丝;9-JOB输入口1;10-JOB输入口2;11-JOB输入口3;12-传送带启停控制;13-机器人暂停中;14-到达作业原点;
    15-到达干涉区;16-焊丝寻位启停控制;17-机器人启动完成;18-程序启动停止;19-自动手动模式;20-急停输出信号1-安全;
    21-急停输出信号2-安全;22-LUA脚本程序运行停止;23-安全状态输出-安全;24-保护性停止状态输出-安全;
    25-机器人运动中-安全;26-机器人缩减模式-安全;27-机器人非缩减模式-安全;28-机器人非停止;29-机器人报错-指令点错误;
    30-机器人报错-驱动器错误;31-机器人报错-超出软限位错误;32-机器人报错-碰撞错误;33-机器人报错-活动从站数量错误;
    34-机器人报错-从站错误;35-机器人报错-IO错误;36-机器人报错-夹爪错误;37-机器人报错-文件错误;38-机器人报错-奇异位姿错误;
    39-机器人报错-驱动器通信错误;40-机器人报错-参数错误;41-机器人报错-外部轴超出软限位错误;42-机器人警告-警告;
    43-机器人警告-安全门警告;44-机器人警告-运动警告;45-机器人警告-干涉区警告;46-机器人警告-安全墙警告;
    47-使能状态;48-断线自动抬升中;49-立方体1干涉警告;50-立方体2干涉警告;51-立方体3干涉警告;52-立方体4干涉警告;
 * @return 错误码
 */
errno_t FRRobot::SetDOConfig(int config[8])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;
    for (int i = 0; i < 8; i++)
    {
        param[0][i] = config[i];
    }

    if (c.execute("SetDOConfig", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetDOConfig fail: %d.", errcode);
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
* @brief 获取可配置CO端口功能
* @param [out] config CO0-CO7功能编码；
* 0-无;1-机器人报错;2-机器人运动中;3-喷涂启停;4-喷涂清枪;5-送气信号;6-起弧信号;7-点动送丝;
8-反向送丝;9-JOB输入口1;10-JOB输入口2;11-JOB输入口3;12-传送带启停控制;13-机器人暂停中;14-到达作业原点;
15-到达干涉区;16-焊丝寻位启停控制;17-机器人启动完成;18-程序启动停止;19-自动手动模式;20-急停输出信号1-安全;
21-急停输出信号2-安全;22-LUA脚本程序运行停止;23-安全状态输出-安全;24-保护性停止状态输出-安全;
25-机器人运动中-安全;26-机器人缩减模式-安全;27-机器人非缩减模式-安全;28-机器人非停止;29-机器人报错-指令点错误;
30-机器人报错-驱动器错误;31-机器人报错-超出软限位错误;32-机器人报错-碰撞错误;33-机器人报错-活动从站数量错误;
34-机器人报错-从站错误;35-机器人报错-IO错误;36-机器人报错-夹爪错误;37-机器人报错-文件错误;38-机器人报错-奇异位姿错误;
39-机器人报错-驱动器通信错误;40-机器人报错-参数错误;41-机器人报错-外部轴超出软限位错误;42-机器人警告-警告;
43-机器人警告-安全门警告;44-机器人警告-运动警告;45-机器人警告-干涉区警告;46-机器人警告-安全墙警告;
47-使能状态;48-断线自动抬升中;49-立方体1干涉警告;50-立方体2干涉警告;51-立方体3干涉警告;52-立方体4干涉警告;
* @return 错误码
*/
errno_t FRRobot::GetDOConfig(int config[8])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetDOConfig", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            for (int i = 0; i < 8; i++)
            {
                config[i] = int(result[i + 1]);
            }
        }
        else
        {
            logger_error("execute GetDOConfig fail %d", errcode);
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
 * @brief 设置末端可配置End-CI端口功能
 * @param [in] config End CI0-CI1功能编码；
 * 0-无;1-拖动示教工具开关;2-点记录信号;3-手自动切换（脉冲信号）;4-TPD记录启动/停止;5-暂停运动;
	6-恢复运动;7-启动;8-停止;9-暂停/恢复;10-启动/停止;11-力传感器辅助拖动开启;12-力传感器辅助拖动关闭;
	13-力传感器辅助拖动开启/关闭;14-激光检测信号X;15-激光检测信号Y;16-PTP运动至作业原点;17-运动中断，根据信号停止当前运动;
	18-启动主程序;19-启动倒带;20-启动确认;21-恢复焊接;22-终止焊接;23-清除错误;24-手自动切换（高低电平）
	25-使能;26-去使能;27-使能/去使能;28-激光伺服跟踪启停信号;
 * @return 错误码
 */
errno_t FRRobot::SetToolDIConfig(int config[2])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;
    for (int i = 0; i < 2; i++)
    {
        param[0][i] = config[i];
    }

    if (c.execute("SetToolDIConfig", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetToolDIConfig fail: %d.", errcode);
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
 * @brief 获取末端可配置End-CI端口功能
 * @param [out] config End CI0-CI1功能编码；
 * 0-无;1-拖动示教工具开关;2-点记录信号;3-手自动切换（脉冲信号）;4-TPD记录启动/停止;5-暂停运动;
	6-恢复运动;7-启动;8-停止;9-暂停/恢复;10-启动/停止;11-力传感器辅助拖动开启;12-力传感器辅助拖动关闭;
	13-力传感器辅助拖动开启/关闭;14-激光检测信号X;15-激光检测信号Y;16-PTP运动至作业原点;17-运动中断，根据信号停止当前运动;
	18-启动主程序;19-启动倒带;20-启动确认;21-恢复焊接;22-终止焊接;23-清除错误;24-手自动切换（高低电平）
	25-使能;26-去使能;27-使能/去使能;28-激光伺服跟踪启停信号;
 * @return 错误码
 */
errno_t FRRobot::GetToolDIConfig(int config[2])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetToolDIConfig", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            for (int i = 0; i < 2; i++)
            {
                config[i] = int(result[i + 1]);
            }
        }
        else
        {
            logger_error("execute GetToolDIConfig fail %d", errcode);
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

///**
// * @brief 设置末端可配置End-DO端口功能
// * @param [in] config End DO0-DO1功能编码；
// * @return 错误码
// */
//errno_t FRRobot::SetToolDOConfig(int config[2])
//{
//    if (IsSockError())
//    {
//        return g_sock_com_err;
//    }
//
//    int errcode = 0;
//    XmlRpcClient c(serverUrl, 20003);
//    XmlRpcValue param, result;
//    for (int i = 0; i < 2; i++)
//    {
//        param[0][i] = config[i];
//    }
//
//    if (c.execute("SetToolDOConfig", param, result))
//    {
//        errcode = int(result);
//        if (0 != errcode)
//        {
//            logger_error("execute SetToolDOConfig fail: %d.", errcode);
//            c.close();
//            return errcode;
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
//
///**
// * @brief 获取末端可配置End-DO端口功能
// * @param [out] config End DO0-DO1功能编码；
// * @return 错误码
// */
//errno_t FRRobot::GetToolDOConfig(int config[2])
//{
//    if (IsSockError())
//    {
//        return g_sock_com_err;
//    }
//    int errcode = 0;
//    XmlRpcClient c(serverUrl, 20003);
//    XmlRpcValue param, result;
//
//    if (c.execute("GetToolDOConfig", param, result))
//    {
//        errcode = int(result[0]);
//        if (errcode == 0)
//        {
//            for (int i = 0; i < 2; i++)
//            {
//                config[i] = int(result[i + 1]);
//            }
//        }
//        else
//        {
//            logger_error("execute GetToolDOConfig fail %d", errcode);
//        }
//    }
//    else
//    {
//        c.close();
//        return ERR_XMLRPC_CMD_FAILED;
//    }
//
//    c.close();
//
//    return errcode;
//}

/**
 * @brief 设置控制箱可配置CI有效状态
 * @param [in] config CI0-CI7端口有效状态；0-高电平有效；1-低电平有效
 * @return 错误码
 */
errno_t FRRobot::SetDIConfigLevel(int config[8])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;
    for (int i = 0; i < 8; i++)
    {
        param[0][i] = config[i];
    }

    if (c.execute("SetDIConfigLevel", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetDIConfigLevel fail: %d.", errcode);
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
 * @brief 获取控制箱可配置CI有效状态
 * @param [out] config CI0-CI7端口有效状态；0-高电平有效；1-低电平有效
 * @return 错误码
 */
errno_t FRRobot::GetDIConfigLevel(int config[8])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetDIConfigLevel", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            for (int i = 0; i < 8; i++)
            {
                config[i] = int(result[i + 1]);
            }
        }
        else
        {
            logger_error("execute GetDIConfigLevel fail %d", errcode);
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
 * @brief 设置控制箱可配置CO有效状态
 * @param [in] config CO0-CO7端口有效状态；0-高电平有效；1-低电平有效
 * @return 错误码
 */
errno_t FRRobot::SetDOConfigLevel(int config[8])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;
    for (int i = 0; i < 8; i++)
    {
        param[0][i] = config[i];
    }

    if (c.execute("SetDOConfigLevel", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetDOConfigLevel fail: %d.", errcode);
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
 * @brief 获取控制箱可配置CO有效状态
 * @param [out] config CO0-CO7端口有效状态；0-高电平有效；1-低电平有效
 * @return 错误码
 */
errno_t FRRobot::GetDOConfigLevel(int config[8])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetDOConfigLevel", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            for (int i = 0; i < 8; i++)
            {
                config[i] = int(result[i + 1]);
            }
        }
        else
        {
            logger_error("execute GetDOConfigLevel fail %d", errcode);
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
 * @brief 设置末端可配置CI有效状态
 * @param [in] config CI0-CI1端口有效状态；0-高电平有效；1-低电平有效
 * @return 错误码
 */
errno_t FRRobot::SetToolDIConfigLevel(int config[2])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;
    for (int i = 0; i < 2; i++)
    {
        param[0][i] = config[i];
    }

    if (c.execute("SetToolDIConfigLevel", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetToolDIConfigLevel fail: %d.", errcode);
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
 * @brief 获取末端可配置CI有效状态
 * @param [out] config CI0-CI1端口有效状态；0-高电平有效；1-低电平有效
 * @return 错误码
 */
errno_t FRRobot::GetToolDIConfigLevel(int config[2])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetToolDIConfigLevel", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            for (int i = 0; i < 2; i++)
            {
                config[i] = int(result[i + 1]);
            }
        }
        else
        {
            logger_error("execute GetToolDIConfigLevel fail %d", errcode);
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
//
///**
// * @brief 设置末端可配置CO有效状态
// * @param [in] config CO0-CO1端口有效状态；0-高电平有效；1-低电平有效
// * @return 错误码
// */
//errno_t FRRobot::SetToolDOConfigLevel(int config[2])
//{
//    if (IsSockError())
//    {
//        return g_sock_com_err;
//    }
//
//    int errcode = 0;
//    XmlRpcClient c(serverUrl, 20003);
//    XmlRpcValue param, result;
//    for (int i = 0; i < 2; i++)
//    {
//        param[0][i] = config[i];
//    }
//
//    if (c.execute("SetToolDOConfigLevel", param, result))
//    {
//        errcode = int(result);
//        if (0 != errcode)
//        {
//            logger_error("execute SetToolDOConfigLevel fail: %d.", errcode);
//            c.close();
//            return errcode;
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
//
///**
// * @brief 获取末端可配置CO有效状态
// * @param [out] config CO0-CO7端口有效状态；0-高电平有效；1-低电平有效
// * @return 错误码
// */
//errno_t FRRobot::GetToolDOConfigLevel(int config[2])
//{
//    if (IsSockError())
//    {
//        return g_sock_com_err;
//    }
//    int errcode = 0;
//    XmlRpcClient c(serverUrl, 20003);
//    XmlRpcValue param, result;
//
//    if (c.execute("GetToolDOConfigLevel", param, result))
//    {
//        errcode = int(result[0]);
//        if (errcode == 0)
//        {
//            for (int i = 0; i < 2; i++)
//            {
//                config[i] = int(result[i + 1]);
//            }
//        }
//        else
//        {
//            logger_error("execute GetToolDOConfigLevel fail %d", errcode);
//        }
//    }
//    else
//    {
//        c.close();
//        return ERR_XMLRPC_CMD_FAILED;
//    }
//
//    c.close();
//
//    return errcode;
//}

/**
 * @brief 设置控制箱标准DI有效状态
 * @param [in] config DI0-DI7端口有效状态；0-高电平有效；1-低电平有效
 * @return 错误码
 */
errno_t FRRobot::SetStandardDILevel(int config[8])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;
    for (int i = 0; i < 8; i++)
    {
        param[0][i] = config[i];
    }

    if (c.execute("SetStandardDILevel", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetStandardDILevel fail: %d.", errcode);
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
 * @brief 获取控制箱标准DI有效状态
 * @param [out] config DI0-DI7端口有效状态；0-高电平有效；1-低电平有效
 * @return 错误码
 */
errno_t FRRobot::GetStandardDILevel(int config[8])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetStandardDILevel", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            for (int i = 0; i < 8; i++)
            {
                config[i] = int(result[i + 1]);
            }
        }
        else
        {
            logger_error("execute GetStandardDILevel fail %d", errcode);
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
 * @brief 设置控制箱标准DO有效状态
 * @param [in] config DO0-DO7端口有效状态；0-高电平有效；1-低电平有效
 * @return 错误码
 */
errno_t FRRobot::SetStandardDOLevel(int config[8])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;
    for (int i = 0; i < 8; i++)
    {
        param[0][i] = config[i];
    }

    if (c.execute("SetStandardDOLevel", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetStandardDOLevel fail: %d.", errcode);
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
 * @brief 获取控制箱标准DO有效状态
 * @param [out] config DO0-DO7端口有效状态；0-高电平有效；1-低电平有效
 * @return 错误码
 */
errno_t FRRobot::GetStandardDOLevel(int config[8])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetStandardDOLevel", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            for (int i = 0; i < 8; i++)
            {
                config[i] = int(result[i + 1]);
            }
        }
        else
        {
            logger_error("execute GetStandardDOLevel fail %d", errcode);
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