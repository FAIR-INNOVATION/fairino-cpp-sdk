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


/**
 * @brief 写入激光焊机10个工艺组中某一个的配置参数并配置给焊机
 * @param[in] io_type 通信类型 0-IO 1-UDP
 * @param[in] num 需要设置的组号（1~10）
 * @param[in] scanSpeed 扫描速度
 * @param[in] scanWidth 扫描宽度
 * @param[in] peakPower 峰值功率
 * @param[in] dutyCycle 占空比
 * @param[in] freq 频率
 * @return 错误码
 */
errno_t FRRobot::SetLaserWeldingParam(int io_type, int num, int scanSpeed, int scanWidth, int peakPower, int dutyCycle, int freq)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = io_type;
    param[1] = num;
    param[2] = scanSpeed;
    param[3] = scanWidth;
    param[4] = peakPower;
    param[5] = dutyCycle;
    param[6] = freq;

    if (c.execute("SetLaserWeldingParam", param, result))
    {
        errcode = int(result);
        if (errcode != 0)
        {
            logger_error("execute SetLaserWeldingParam fail %d", errcode);
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
 * @brief 设置激光焊机开启关闭
 * @param[in] io_type 通信类型 0-IO 1-UDP
 * @param[in] status 控制字 0-收光 1-出光
 * @param[in] max_waittime 最大等待时间
 * @return 错误码
 */
errno_t FRRobot::SetLaserWeldingStartEnd(int io_type, int status, int max_waittime)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = io_type;
    param[1] = status;
    param[2] = max_waittime;

    if (c.execute("SetLaserWeldingStartEnd", param, result))
    {
        errcode = int(result);
        if (errcode != 0)
        {
            logger_error("execute SetLaserWeldingStartEnd fail %d", errcode);
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
 * @brief 激光焊机使能去使能
 * @param[in] io_type 通信类型 0-IO 1-UDP
 * @param[in] status 0-去使能 1-使能
 * @return 错误码
 */
errno_t FRRobot::SetLaserWeldingEnable(int io_type, int status)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = io_type;
    param[1] = status;

    if (c.execute("SetLaserWeldingEnable", param, result))
    {
        errcode = int(result);
        if (errcode != 0)
        {
            logger_error("execute SetLaserWeldingEnable fail %d", errcode);
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
 * @brief 激光焊机故障复位
 * @param[in] io_type 通信类型 0-IO 1-UDP
 * @param[in] status 控制字 0-无效 1-故障复位
 * @return 错误码
 */
errno_t FRRobot::ResetLaserWeldingErr(int io_type, int status)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = io_type;
    param[1] = status;

    if (c.execute("ResetLaserWeldingErr", param, result))
    {
        errcode = int(result);
        if (errcode != 0)
        {
            logger_error("execute ResetLaserWeldingErr fail %d", errcode);
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
 * @brief 获取激光焊机运行状态
 * @param[in] io_type 通信类型 0-IO 1-UDP
 * @param[out] status 控制字 0-停机 1-运行
 * @return 错误码
 */
errno_t FRRobot::GetLaserWeldingRunningState(int io_type, int& status)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = io_type;

    if (c.execute("GetLaserWeldingRunningState", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            status = int(result[1]);
        }
        else
        {
            logger_error("execute GetLaserWeldingRunningState fail %d", errcode);
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
 * @brief 获取激光焊机故障状态
 * @param[in] io_type 通信类型 0-IO 1-UDP
 * @param[out] status 0-无故障 1-存在故障
 * @return 错误码
 */
errno_t FRRobot::GetLaserWeldingErrState(int io_type, int& status)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = io_type;

    if (c.execute("GetLaserWeldingErrState", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            status = int(result[1]);
        }
        else
        {
            logger_error("execute GetLaserWeldingErrState fail %d", errcode);
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
 * @brief 获取激光焊机10个工艺组中某一个的配置参数
 * @param[in] num 需要设置的组号（1~10）
 * @param[out] scanSpeed 扫描速度
 * @param[out] scanWidth 扫描宽度
 * @param[out] peakPower 峰值功率
 * @param[out] dutyCycle 占空比
 * @param[out] freq 频率
 * @return 错误码
 */
errno_t FRRobot::GetLaserWeldingParamTarget(int num, int& scanSpeed, int& scanWidth, int& peakPower, int& dutyCycle, int& freq)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = num;

    if (c.execute("GetLaserWeldingParamTarget", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            scanSpeed = int(result[1]);
            scanWidth = int(result[2]);
            peakPower = int(result[3]);
            dutyCycle = int(result[4]);
            freq = int(result[5]);
        }
        else
        {
            logger_error("execute GetLaserWeldingParamTarget fail %d", errcode);
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
 * @brief 获取当前激光焊机生效的配置参数
 * @param[in] io_type 通信类型 0-IO 1-UDP
 * @param[out] scanSpeed 扫描速度
 * @param[out] scanWidth 扫描宽度
 * @param[out] peakPower 峰值功率
 * @param[out] dutyCycle 占空比
 * @param[out] freq 频率
 * @return 错误码
 */
errno_t FRRobot::GetLaserWeldingParamActual(int io_type, int& scanSpeed, int& scanWidth, int& peakPower, int& dutyCycle, int& freq)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = io_type;

    if (c.execute("GetLaserWeldingParamActual", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            scanSpeed = int(result[1]);
            scanWidth = int(result[2]);
            peakPower = int(result[3]);
            dutyCycle = int(result[4]);
            freq = int(result[5]);
        }
        else
        {
            logger_error("execute GetLaserWeldingParamActual fail %d", errcode);
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
 * @brief 激光焊机设置扩展IO，使能的DO端口
 * @param[in] ctrlModeDONum 激光焊机使能的扩展DO端口号
 * @return 错误码
 */
errno_t FRRobot::SetLaserWeldingEnableExtDoNum(int ctrlModeDONum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = ctrlModeDONum;

    if (c.execute("SetLaserWeldingEnableExtDoNum", param, result))
    {
        errcode = int(result);
        if (errcode != 0)
        {
            logger_error("execute SetLaserWeldingEnableExtDoNum fail %d", errcode);
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
 * @brief 激光焊机设置扩展IO，启动的DO端口
 * @param[in] ctrlModeDONum 激光焊机启动（出光收光）的扩展DO端口号
 * @return 错误码
 */
errno_t FRRobot::SetLaserWeldingStartExtDoNum(int ctrlModeDONum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = ctrlModeDONum;

    if (c.execute("SetLaserWeldingStartExtDoNum", param, result))
    {
        errcode = int(result);
        if (errcode != 0)
        {
            logger_error("execute SetLaserWeldingStartExtDoNum fail %d", errcode);
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
 * @brief 激光焊机设置扩展IO，故障复位的DO端口
 * @param[in] ctrlModeDONum 激光焊机故障复位的扩展DO端口号
 * @return 错误码
 */
errno_t FRRobot::SetLaserWeldingErrResetExtDoNum(int ctrlModeDONum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = ctrlModeDONum;

    if (c.execute("SetLaserWeldingErrResetExtDoNum", param, result))
    {
        errcode = int(result);
        if (errcode != 0)
        {
            logger_error("execute SetLaserWeldingErrResetExtDoNum fail %d", errcode);
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
 * @brief 配置激光焊机运行状态（出光状态）扩展DI
 * @param[in] diNum 配置激光焊机运行状态（出光状态）扩展DI端口
 * @return 错误码
 */
errno_t FRRobot::SetLaserWeldingRunningStateExtDiNum(int diNum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = diNum;

    if (c.execute("SetLaserWeldingRunningStateExtDiNum", param, result))
    {
        errcode = int(result);
        if (errcode != 0)
        {
            logger_error("execute SetLaserWeldingRunningStateExtDiNum fail %d", errcode);
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
 * @brief 配置激光焊机故障状态扩展DI
 * @param[in] diNum 配置激光焊机故障状态 扩展DI端口
 * @return 错误码
 */
errno_t FRRobot::SetLaserWeldingErrStateExtDiNum(int diNum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = diNum;

    if (c.execute("SetLaserWeldingErrStateExtDiNum", param, result))
    {
        errcode = int(result);
        if (errcode != 0)
        {
            logger_error("execute SetLaserWeldingErrStateExtDiNum fail %d", errcode);
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
 * @brief 控制灵巧手运动
 * @param [in] idstart  起始从站号
 * @param [in] slaveNum  数量
 * @param [in] pos[16]  位置(-360~360)
 * @param [in] speed[16]  速度百分比，范围[0~100]
 * @param [in] force[16]  力矩百分比，范围[0~100]
 * @param [in] max_time  最大等待时间，范围[0~30000]，单位ms
 * @return 错误码
 */
errno_t FRRobot::SetDexterousHandsMove(int idstart, int slaveNum, double pos[16], int speed[16], int force[16], int max_time)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = idstart;
    param[1] = slaveNum;
    for (int i = 0; i < 16; i++)
    {
        param[2][i] = pos[i];
    }
    for (int i = 0; i < 16; i++)
    {
        param[3][i] = speed[i];
    }
    for (int i = 0; i < 16; i++)
    {
        param[4][i] = force[i];
    }
    param[5] = max_time;

    if (c.execute("SetDexterousHandsMove", param, result))
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
 * @brief  控制灵巧手复位激活
 * @param  [in] id  从站号
 * @param  [in] act  0-复位 1-激活
 * @return  错误码
 */
errno_t FRRobot::SetDexterousHandsAct(int id, int act)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    param[1] = act;

    if (c.execute("SetDexterousHandsAct", param, result))
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
 * @brief  清除灵巧手错误
 * @return  错误码
 */
errno_t FRRobot::ClearDexterousHandsError()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("ClearDexterousHandsError", param, result))
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
 * @brief 设置启用夹爪动作控制功能
 * @param [in] id 夹爪设备编号
 * @param [in] func func[0]-夹爪使能；func[1]-夹爪初始化；2-位置设置；3-速度设置；4-力矩设置；6-读夹爪状态；7-读初始化状态；8-读故障码；9-读位置；10-读速度；11-读力矩
 * @return  错误码
 */
errno_t FRRobot::SetDexterousHandsFunc(int id, int func[32])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    for (int i = 0; i < 32; i++)
    {
        param[1][i] = func[i];
    }

    if (c.execute("SetDexterousHandsFunc", param, result))
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
 * @brief 获取启用夹爪动作控制功能
 * @param [in] id 夹爪设备编号
 * @param [out] func func[0]-夹爪使能；func[1]-夹爪初始化；2-位置设置；3-速度设置；4-力矩设置；6-读夹爪状态；7-读初始化状态；8-读故障码；9-读位置；10-读速度；11-读力矩
 * @return  错误码
 */
errno_t FRRobot::GetDexterousHandsFunc(int id, int func[32])
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;

    if (c.execute("GetDexterousHandsFunc", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            std::string resultStr = std::string(result[1]);
            // 分割字符串
            std::vector<std::string> parS;
            std::stringstream ss(resultStr);
            std::string item;
            while (std::getline(ss, item, ','))
            {
                parS.push_back(item);
            }

            if (parS.size() != 32)
            {
                logger_error("GetDexterousHandsFunc fail");
                c.close();
                return -1;
            }

            for (int i = 0; i < 32; i++)
            {
                func[i] = std::stoi(parS[i]);
            }
        }
        else
        {
            logger_error("execute GetDexterousHandsFunc fail %d", errcode);
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
* @brief 传送带原地跟踪参数配置
* @param [in] trackMode 0-时间；1-距离；2-时间和距离任意满足一个
* @param [in] trackTime 跟踪时间，单位s
* @param [in] trackDis 跟踪距离
* @return 错误码
*/
errno_t FRRobot::SetStationaryTrackPara(int trackMode, double trackTime, int trackDis)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = trackMode;
    param[1] = trackTime;
    param[2] = trackDis;

    if (c.execute("SetStationaryTrackPara", param, result))
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
 * @brief 等待夹爪运动状态
 * @param [in] status 0-运动未完成，1-运动完成未检测到物体，2-运动完成检测到物体，3-运动完成检测到物体掉落
 * @param [in] timeout 超时时间（ms）-1永久等待
 * @param [in] strategy 0-停止报错，1-继续运行
 * @param [in] type 0-平行夹爪，1-旋转夹爪
 * @return 错误码
 */
errno_t FRRobot::GripperWaitMotionDone(int status, int timeout, int strategy, int type)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = status;
    param[1] = timeout;
    param[2] = strategy;
    param[3] = type;

    if (c.execute("GripperWaitMotionDone", param, result))
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
 * @brief 设置焊接类型
 * @param [in] type 0-电弧焊；1-激光焊
 * @return 错误码
 */
errno_t FRRobot::SetWeldType(int type)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = type;

    if (c.execute("SetWeldType", param, result))
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
 * @brief 设置焊机厂商
 * @param [in] value 101-麦格米特；102-OTC；103-松下；104-米勒；1001-everlast激光焊
 * @return 错误码
 */
errno_t FRRobot::SetWeldCompany(int value)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = value;

    if (c.execute("SetWeldCompany", param, result))
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
 * @brief 设置送丝速度
 * @param [in] io_type 0-IO；1-UDP；2-modbusTCP
 * @param [in] status 送丝速度值
 * @return 错误码
 */
errno_t FRRobot::SetWeldWFS(int io_type, double status)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = io_type;
    param[1] = status;

    if (c.execute("SetWeldWFS", param, result))
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
 * @brief 设置焊接弧长
 * @param [in] io_type 0-IO；1-UDP；2-modbusTCP
 * @param [in] status 弧长
 * @return 错误码
 */
errno_t FRRobot::SetWeldArcLen(int io_type, int status)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = io_type;
    param[1] = status;

    if (c.execute("SetWeldArcLen", param, result))
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
 * @brief 设置电弧控制值
 * @param [in] io_type 0-IO；1-UDP；2-modbusTCP
 * @param [in] status 电弧控制值
 * @return 错误码
 */
errno_t FRRobot::SetWeldArcCtrl(int io_type, int status)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = io_type;
    param[1] = status;

    if (c.execute("SetWeldArcCtrl", param, result))
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
 * @brief 设置程序槽号
 * @param [in] io_type 0-IO；1-UDP；2-modbusTCP
 * @param [in] status 程序槽号值
 * @return 错误码
 */
errno_t FRRobot::SetWeldSlot(int io_type, int status)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = io_type;
    param[1] = status;

    if (c.execute("SetWeldSlot", param, result))
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
 * @brief 设置"禁止焊接"DO
 * @param [in] io_type 0-IO；1-UDP；2-modbusTCP
 * @param [in] status 0-关闭"禁止焊接"；1-开启"禁止焊接"
 * @return 错误码
 */
errno_t FRRobot::SetWeldDisable(int io_type, int status)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = io_type;
    param[1] = status;

    if (c.execute("SetWeldDisable", param, result))
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
 * @brief 设置送丝速度UDP自定义控制字序号
 * @param [in] udpNum UDP自定义控制字序号，默认44
 * @return 错误码
 */
errno_t FRRobot::SetweldWFSUdpNum(int udpNum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = udpNum;

    if (c.execute("SetweldWFSUdpNum", param, result))
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
 * @brief 设置弧长UDP自定义控制字序号
 * @param [in] udpNum UDP自定义控制字序号，默认45
 * @return 错误码
 */
errno_t FRRobot::SetWeldArcLenUdpNum(int udpNum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = udpNum;

    if (c.execute("SetWeldArcLenUdpNum", param, result))
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
 * @brief 设置电弧控制UDP自定义控制字序号
 * @param [in] udpNum UDP自定义控制字序号，默认46
 * @return 错误码
 */
errno_t FRRobot::SetWeldArcCtrlUdpNum(int udpNum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = udpNum;

    if (c.execute("SetWeldArcCtrlUdpNum", param, result))
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
 * @brief 设置程序槽号UDP自定义控制字序号
 * @param [in] udpNum UDP自定义控制字序号，默认47
 * @return 错误码
 */
errno_t FRRobot::SetWeldSlotUdpNum(int udpNum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = udpNum;

    if (c.execute("SetWeldSlotUdpNum", param, result))
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
 * @brief 设置"禁止焊接"UDP扩展DO序号
 * @param [in] DONum UDP扩展DO序号，默认18
 * @return 错误码
 */
errno_t FRRobot::SetWeldDisableExtDoNum(int DONum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = DONum;

    if (c.execute("SetWeldDisableExtDoNum", param, result))
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
 * @brief 设置"协同状态"UDP扩展DI序号
 * @param [in] DINum UDP扩展DI序号，默认-1不启用
 * @return 错误码
 */
errno_t FRRobot::SetWeldSynStateExtDINum(int DINum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = DINum;

    if (c.execute("SetWeldSynStateExtDINum", param, result))
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
 * @brief 获取"协同状态"
 * @param [in] io_type 0-IO；1-UDP；2-modbusTCP
 * @param [out] synState 0-非协同；1-协同
 * @return 错误码
 */
errno_t FRRobot::GetWeldSynState(int io_type, int& synState)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = io_type;

    if (c.execute("GetWeldSynState", param, result))
    {
        errcode = int(result[0]);
        synState = int(result[1]);
    }
    else
    {
        c.close();
        return ERR_XMLRPC_CMD_FAILED;
    }

    c.close();
    return errcode;
}

/**
 * @brief 设置"送丝速度"UDP自定义状态字序号
 * @param [in] udpNum UDP自定义状态字序号，默认47
 * @return 错误码
 */
errno_t FRRobot::SetWeldWfsFeedbackUdpNum(int udpNum)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = udpNum;

    if (c.execute("SetWeldWfsFeedbackUdpNum", param, result))
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
* @brief 获取"送丝速度"当前值
* @param [in] io_type 0-IO；1-UDP；2-modbusTCP
* @param [out] weldWfsFdbk 当前送丝速度
* @return 错误码
*/
errno_t FRRobot::GetWeldWfsFeedback(int io_type, double& weldWfsFdbk)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = io_type;

    if (c.execute("GetWeldWfsFeedback", param, result))
    {
        if (result.size() >= 2)
        {
            errcode = int(result[0]);
            if (errcode == 0)
            {
                weldWfsFdbk = double(result[1]);
            }
        }
        else
        {
            weldWfsFdbk = double(result[0]);
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
 * @brief 设置启用末端吸盘动作控制功能码
 * @param [in] id 站号
 * @param [in] func 功能码数组，长度16，下标0~15对应Bit0~Bit15：
 * 0-吸盘初始化；1-设置通道一最大真空度；2-设置通道一最小真空度；
 * 3-设置通道二最大真空度；4-设置通道二最小真空度；5-读通道一状态；6-读通道二状态；
 * 7-读初始化状态；8-读故障码；9-读通道一实际真空度；10-读通道二实际真空度；
 * 11-通道一控制使能；12-通道二控制使能；每项 0-不启用；1-启用
 * @return 错误码
 */
errno_t FRRobot::SetAxleSuckerFunc(int id, const std::vector<int>& func)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    if (func.empty() || func.size() > 16)
    {
        return ERR_PARAM_NUM;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    for (int i = 0; i < func.size(); i++)
    {
        param[1][i] = func[i];
    }

    if (c.execute("SetAxleSuckerFunc", param, result))
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
 * @brief 获取末端吸盘动作控制功能码
 * @param [in] id 站号
 * @param [out] func 功能码数组，长度16，下标0~15对应Bit0~Bit15，含义同 SetAxleSuckerFunc
 * @return 错误码
 */
errno_t FRRobot::GetAxleSuckerFunc(int id, std::vector<int>& func)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    if (func.size() < 16)
    {
        return ERR_PARAM_NUM;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;

    if (c.execute("GetAxleSuckerFunc", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            std::string s = std::string(result[1]);
            size_t pos = 0, next;
            for (int i = 0; i < 16; i++)
            {
                next = s.find(',', pos);
                func[i] = std::stoi(s.substr(pos, next - pos));
                pos = next + 1;
            }
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
 * @brief 设置末端吸盘控制参数
 * @param [in] id 站号
 * @param [in] channel 通道使能，长度4：channel[0]=1-使能，0-不使能（下标0-3对应通道1-通道4）
 * @param [in] minPress 最小真空度，长度4
 * @param [in] maxPress 最大真空度（吸盘错误码），长度4
 * @param [in] time 超时时间(ms)
 * @return 错误码
 */
errno_t FRRobot::SetAxleSuckerCtrl(int id, const std::vector<int>& channel, const std::vector<int>& minPress, const std::vector<int>& maxPress, int time)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    if (channel.size() < 4 || minPress.size() < 4 || maxPress.size() < 4)
    {
        return ERR_PARAM_NUM;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    for (int i = 0; i < 4; i++)
    {
        param[1][i] = channel[i];
    }
    for (int i = 0; i < 4; i++)
    {
        param[2][i] = minPress[i];
    }
    for (int i = 0; i < 4; i++)
    {
        param[3][i] = maxPress[i];
    }
    param[4] = time;

    if (c.execute("SetAxleSuckerCtrl", param, result))
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
 * @brief 激活/复位吸盘
 * @param [in] id 站号
 * @param [in] act 1-激活；0-复位
 * @return 错误码
 */
errno_t FRRobot::SetAxleSuckerAct(int id, int act)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = id;
    param[1] = act;

    if (c.execute("SetAxleSuckerAct", param, result))
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
 * @brief 获取末端吸盘状态
 * @param [in] channel 吸盘通道；0-3（对应通道1-通道4）
 * @param [out] enable 激活状态
 * @param [out] done 吸附完成状态
 * @param [out] vacuumValue 真空度
 * @param [out] error 吸盘错误码
 * @return 错误码
 */
errno_t FRRobot::GetAxleSuckerState(int channel, int& enable, int& done, int& vacuumValue, int& error)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = channel;

    if (c.execute("GetAxleSuckerState", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            enable = int(result[1]);
            done = int(result[2]);
            vacuumValue = int(result[3]);
            error = int(result[4]);
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
 * @brief 等待末端吸盘吸附完成信号
 * @param [in] channel 吸盘通道；0-3（对应通道1-通道4）
 * @param [in] timeout 超时时间(ms)，-1 表示永远等待
 * @param [in] state 状态：0-未吸附物体；1-吸附物体；2-物体脱落
 * @param [in] strategy 超时策略：0-报错停止；1-继续运行
 * @return 错误码
 */
errno_t FRRobot::WaitAxleSuckerState(int channel, int timeout, int state, int strategy)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = channel;
    param[1] = timeout;
    param[2] = state;
    param[3] = strategy;

    if (c.execute("WaitAxleSuckerState", param, result))
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
 * @brief 设置末端LUA末端设备启用类型
 * @param [in] forceSensorEnable 力传感器启用状态，0-不启用；1-启用
 * @param [in] gripperEnable 夹爪启用状态，0-不启用；1-启用
 * @param [in] IOEnable IO设备启用状态，0-不启用；1-启用
 * @param [in] dexhandEnable 灵巧手设备启用状态，0-不启用；1-启用
 * @param [in] suckerEnable 末端吸盘设备启用状态，0-不启用；1-启用；默认-1：不传或传-1时按原四参数下发，兼容未升级控制器
 * @return 错误码
 */
errno_t FRRobot::SetAxleLuaEnableDeviceType(int forceSensorEnable, int gripperEnable, int IOEnable, int dexhandEnable, int suckerEnable)
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
    param[3] = dexhandEnable;

    if (suckerEnable >= 0)
    {
        param[4] = suckerEnable;
        if (c.execute("SetAxleLuaEnableDeviceType", param, result))
        {
            errcode = int(result);
        }
        else
        {
            c.close();
            return ERR_XMLRPC_CMD_FAILED;
        }
    }
    else
    {
        if (c.execute("SetAxleLuaEnableDeviceType", param, result))
        {
            errcode = int(result);
        }
        else
        {
            c.close();
            return ERR_XMLRPC_CMD_FAILED;
        }
    }

    c.close();
    return errcode;
}

/**
 * @brief 获取末端LUA末端设备启用类型（含末端吸盘）
 * @param [out] forceSensorEnable 力传感器启用状态，0-不启用；1-启用
 * @param [out] gripperEnable 夹爪启用状态，0-不启用；1-启用
 * @param [out] IOEnable IO设备启用状态，0-不启用；1-启用
 * @param [out] dexhandEnable 灵巧手设备启用状态，0-不启用；1-启用
 * @param [out] suckerEnable 末端吸盘设备启用状态，0-不启用；1-启用
 * @return 错误码
 */
errno_t FRRobot::GetAxleLuaEnableDeviceType(int& forceSensorEnable, int& gripperEnable, int& IOEnable, int& dexhandEnable, int& suckerEnable)
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
            forceSensorEnable = int(result[1]);
            gripperEnable = int(result[2]);
            IOEnable = int(result[3]);
            dexhandEnable = int(result[4]);
            if (result.size() > 5)
            {
                suckerEnable = int(result[5]);
            }
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
 * @brief 获取当前配置的末端设备（含末端吸盘）
 * @param [out] forceSensorEnable 力传感器启用设备编号 0-未启用；1-启用
 * @param [out] gripperEnable 夹爪启用设备编号，0-不启用；1-启用
 * @param [out] IODeviceEnable IO设备启用设备编号，0-不启用；1-启用
 * @param [out] decHandEnable 灵巧手启用设备编号，0-不启用；1-启用
 * @param [out] suckerEnable 末端吸盘启用设备编号，0-不启用；1-启用（逗号串第40~47项）
 * @return 错误码
 */
errno_t FRRobot::GetAxleLuaEnableDevice(std::vector<int>& forceSensorEnable, std::vector<int>& gripperEnable, std::vector<int>& IODeviceEnable, std::vector<int>& decHandEnable, std::vector<int>& suckerEnable)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    if (forceSensorEnable.size() < 8 || gripperEnable.size() < 8 || IODeviceEnable.size() < 8 || decHandEnable.size() < 8 || suckerEnable.size() < 8)
    {
        return ERR_PARAM_NUM;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetAxleLuaEnableDevice", param, result))
    {
        errcode = int(result[0]);
        if (errcode == 0)
        {
            std::string s = std::string(result[1]);
            std::vector<std::string> items;
            size_t pos = 0, next;
            for (int i = 0; i < 48; i++)
            {
                next = s.find(',', pos);
                items.push_back(s.substr(pos, next - pos));
                pos = next + 1;
            }
            for (int i = 0; i < 8; i++)
            {
                forceSensorEnable[i] = std::stoi(items[i]);
                gripperEnable[i] = std::stoi(items[i + 8]);
                IODeviceEnable[i] = std::stoi(items[i + 16]);
                decHandEnable[i] = std::stoi(items[i + 24]);
                suckerEnable[i] = std::stoi(items[i + 40]);
            }
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

