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
 * @param [in] status 0-运动未完成，1-运动完成未检测到物体 2-运动完成检测到物体
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