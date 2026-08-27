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
#include <FrameHandle.h>
#include "FRUdpClient.h"

using namespace std;
using namespace XmlRpc;


/**
 *@brief 关节空间伺服模式运动
 *@param [in] joint_pos 目标关节位置,单位deg
 *@param [in] axisPos 外部轴位置,单位mm
 *@param [in] acc 加速度百分比，范围[0~100],暂不开放，默认为0
 *@param [in] vel 速度百分比，范围[0~100]，暂不开放，默认为0
 *@param [in] cmdT 指令下发周期，单位s，建议范围[0.001~0.0016]
 *@param [in] filterT 滤波时间，单位s，暂不开放，默认为0
 *@param [in] gain 目标位置的比例放大器，暂不开放，默认为0
 *@param [in] id servoJ指令ID,默认为0
 *@param [in] comType 指令下发类型；0-xmlrpc；1-UDP(对应机器人20007端口)
 *@return  错误码
 */
errno_t FRRobot::ServoJ(JointPos* joint_pos, ExaxisPos* axisPos, float acc, float vel, float cmdT, float filterT, float gain, int id, int comType)
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
    if (comType == 0)
    {
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
            if (result.getType() == XmlRpc::XmlRpcValue::Type::TypeArray)
            {
                errcode = int(result[0]);
            }
            else
            {
                errcode = int(result);
            }
        }
        else
        {
            c.close();
            return ERR_XMLRPC_CMD_FAILED;
        }

        c.close();
    }
    else if (comType == 1)
    {
        char jointStr[128] = { 0 };
        snprintf(jointStr, 128, "{%.3f,%.3f,%.3f,%.3f,%.3f,%.3f}", joint_pos->jPos[0], joint_pos->jPos[1], joint_pos->jPos[2], joint_pos->jPos[3], joint_pos->jPos[4], joint_pos->jPos[5]);
        char axisStr[128] = { 0 };
        snprintf(axisStr, 128, "{%.3f,%.3f,%.3f,%.3f}", axisPos->ePos[0], axisPos->ePos[1], axisPos->ePos[2], axisPos->ePos[3]);

        string cmdStr = string("ServoJ(") + jointStr + "," + axisStr + "," + to_string(acc) + "," + to_string(vel) + "," + to_string(cmdT) + "," + to_string(filterT) + "," + to_string(gain) + "," + to_string(id) + ")";
        FRAME frame(cmdFrameCnt, 376, cmdStr);
        int rtn = udpCmdClient->SendFrame(PackFrame(frame));
        if (rtn != 0)
        {
            return ERR_SOCKET_SEND_FAILED;
        }
        cmdFrameCnt++;
    }
    else
    {
        return ERR_PARAM_VALUE;
    }

    return errcode;
}

/**
* @brief 关节空间伺服模式运动(支持多点位一次输入)
* @param [in] joint_pos 目标关节位置集合(最多支持10组),单位deg
* @param [in] axisPos 外部轴位置,单位mm
* @param [in] acc 加速度百分比，范围[0~100],暂不开放，默认为0
* @param [in] vel 速度百分比，范围[0~100]，暂不开放，默认为0
* @param [in] cmdT 指令下发周期，单位s，建议范围[0.001~0.0016]
* @param [in] filterT 滤波时间，单位s，暂不开放，默认为0
* @param [in] gain 目标位置的比例放大器，暂不开放，默认为0
* @param [out] servoJCmdCount ServoJ指令点位计数[0-10000]
* @param [in] id servoJ指令ID,默认为0
* @param [in] comType 指令下发类型；0-xmlrpc；1-UDP(对应机器人20007端口)
* @return  错误码
*/
errno_t FRRobot::ServoJ(std::vector<JointPos> joint_pos, ExaxisPos* axisPos, float acc, float vel, float cmdT, float filterT, float gain, int& servoJCmdCount, int id, int comType)
{
    if (joint_pos.size() > 10 || joint_pos.size() == 0)
    {
        return ERR_PARAM_NUM;
    }

    if (IsSockError())
    {
        return g_sock_com_err;
    }

    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }

    int pointNum = joint_pos.size();

    int errcode = 0;
    if (comType == 0)
    {
        XmlRpcClient c(serverUrl, 20003);
        XmlRpcValue param, result;

        for (int i = 0; i < pointNum; i++)
        {
            param[0][i * 6 + 0] = joint_pos[i].jPos[0];
            param[0][i * 6 + 1] = joint_pos[i].jPos[1];
            param[0][i * 6 + 2] = joint_pos[i].jPos[2];
            param[0][i * 6 + 3] = joint_pos[i].jPos[3];
            param[0][i * 6 + 4] = joint_pos[i].jPos[4];
            param[0][i * 6 + 5] = joint_pos[i].jPos[5];
        }

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
            if (result.getType() == XmlRpc::XmlRpcValue::Type::TypeArray)
            {
                errcode = int(result[0]);
                servoJCmdCount = int(result[1]);
            }
            else
            {
                errcode = int(result);
            }
        }
        else
        {
            c.close();
            return ERR_XMLRPC_CMD_FAILED;
        }

        c.close();
    }
    else if (comType == 1)
    {
        char jointStr[2048] = { 0 };
        for (int i = 0; i < pointNum; i++)
        {
            if (i != 0)
            {
                strcat(jointStr, ",");
            }
            char tmpJointStr[128] = { 0 };
            snprintf(tmpJointStr, 128, "{%.3f,%.3f,%.3f,%.3f,%.3f,%.3f}", joint_pos[i].jPos[0], joint_pos[i].jPos[1], joint_pos[i].jPos[2], joint_pos[i].jPos[3], joint_pos[i].jPos[4], joint_pos[i].jPos[5]);

            strcat(jointStr, tmpJointStr);
        }

        char axisStr[128] = { 0 };
        snprintf(axisStr, 128, "{%.3f,%.3f,%.3f,%.3f}", axisPos->ePos[0], axisPos->ePos[1], axisPos->ePos[2], axisPos->ePos[3]);

        string cmdStr = string("ServoJ(") + jointStr + "," + axisStr + "," + to_string(acc) + "," + to_string(vel) + "," + to_string(cmdT) + "," + to_string(filterT) + "," + to_string(gain) + "," + to_string(id) + ")";
        FRAME frame(cmdFrameCnt, 376, cmdStr);
        int rtn = udpCmdClient->SendFrame(PackFrame(frame));
        if (rtn != 0)
        {
            return ERR_SOCKET_SEND_FAILED;
        }
        cmdFrameCnt++;
    }
    else
    {
        return ERR_PARAM_VALUE;
    }

    return errcode;
}


/**
 *@brief 笛卡尔空间伺服模式运动
 *@param [in] mode 0-绝对运动(基坐标系)，1-增量运动(基坐标系)，2-增量运动(工具坐标系)
 *@param [in] desc_pos 目标笛卡尔位姿或位姿增量
 *@param [in] exaxis 扩展轴位置
 *@param [in] pos_gain 位姿增量比例系数，仅在增量运动下生效，范围[0~1]
 *@param [in] acc 加速度百分比，范围[0~100],暂不开放，默认为0
 *@param [in] vel 速度百分比，范围[0~100]，暂不开放，默认为0
 *@param [in] cmdT 指令下发周期，单位s，建议范围[0.001~0.016]
 *@param [in] filterT 滤波时间，单位s，暂不开放，默认为0
 *@param [in] gain 目标位置的比例放大器，暂不开放，默认为0
 *@return 错误码
 */
errno_t FRRobot::ServoCart(int mode, DescPose* desc_pose, ExaxisPos exaxis, float pos_gain[6], float acc, float vel, float cmdT, float filterT, float gain)
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
    param[3][0] = exaxis.ePos[0];
    param[3][1] = exaxis.ePos[1];
    param[3][2] = exaxis.ePos[2];
    param[3][3] = exaxis.ePos[3];
    param[4] = acc;
    param[5] = vel;
    param[6] = cmdT;
    param[7] = filterT;
    param[8] = gain;

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

/**
 * @brief 关节空间速度伺服模式运动
 * @param [in] joint_pos 6个目标关节速度,单位deg/s
 * @param [in] axisPos 4个外部轴速度,单位deg/s
 * @param [in] acc 加速度百分比，范围[0~100],暂不开放，默认为0
 * @param [in] vel 速度百分比，范围[0~100]，暂不开放，默认为0
 * @param [in] cmdT 指令下发周期，单位s，建议范围[0.001~0.0016]
 * @param [in] filterT 滤波时间，单位s，暂不开放，默认为0
 * @param [in] gain 目标位置的比例放大器，暂不开放，默认为0
 * @param [in] id servoJ指令ID,默认为0
 * @param [in] comType 指令下发类型；0-xmlrpc；1-UDP(对应机器人20007端口)
 * @return 错误码
 */
errno_t FRRobot::ServoJV(double jointVel[6], double exisVel[4], float acc, float vel, float cmdT, float filterT, float gain, int id, int comType)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }

    if (comType == 0)
    {
        int errcode = 0;
        XmlRpcClient c(serverUrl, 20003);
        XmlRpcValue param, result;

        param[0][0] = jointVel[0];
        param[0][1] = jointVel[1];
        param[0][2] = jointVel[2];
        param[0][3] = jointVel[3];
        param[0][4] = jointVel[4];
        param[0][5] = jointVel[5];
        param[1][0] = exisVel[0];
        param[1][1] = exisVel[1];
        param[1][2] = exisVel[2];
        param[1][3] = exisVel[3];
        param[2] = acc;
        param[3] = vel;
        param[4] = cmdT;
        param[5] = filterT;
        param[6] = gain;
        param[7] = id;

        if (c.execute("ServoJV", param, result))
        {
            errcode = int(result);
            if (0 != errcode)
            {
                logger_error("execute ServoJV fail: %d.", errcode);
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
    else if (comType == 1)
    {
        char jointVelStr[128] = { 0 };
        snprintf(jointVelStr, 128, "{%.3f,%.3f,%.3f,%.3f,%.3f,%.3f}", jointVel[0], jointVel[1], jointVel[2], jointVel[3], jointVel[4], jointVel[5]);
        char exaxisVelStr[128] = { 0 };
        snprintf(exaxisVelStr, 128, "{%.3f,%.3f,%.3f,%.3f}", exisVel[0], exisVel[1], exisVel[2], exisVel[3]);
                                                                             
        string cmdStr = string("ServoJV(") + jointVelStr + "," + exaxisVelStr + "," + 
        to_string(acc) + "," + to_string(vel) + "," + to_string(cmdT) + "," + to_string(filterT) + "," + to_string(gain) + "," + to_string(id) + ")";
        FRAME frame(cmdFrameCnt, 1337, cmdStr);
        int rtn = udpCmdClient->SendFrame(PackFrame(frame));
        if (rtn != 0)
        {
            return ERR_SOCKET_SEND_FAILED;
        }
        cmdFrameCnt++;

        return ERR_SUCCESS;
    }
    else
    {
        return ERR_PARAM_VALUE;
    }

}

/**
 * @brief 关节MIT控制开始
 * @param [in]  comType 指令下发类型；0-xmlrpc；1-UDP(对应机器人20007端口)
 * @return  错误码
 */
errno_t FRRobot::ServoMITStart(int comType)
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

    if (comType == 0)
    {
        XmlRpcClient c(serverUrl, 20003);
        XmlRpcValue param, result;

        if (c.execute("ServoMITStart", param, result))
        {
            errcode = int(result);
            if (0 != errcode)
            {
                logger_error("execute ServoMITStart fail: %d.", errcode);
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
    }
    else if (comType == 1)
    {
        string cmdStr = string("ServoMITStart()");
        FRAME frame(cmdFrameCnt, 1334, cmdStr);
        int rtn = udpCmdClient->SendFrame(PackFrame(frame));
        if (rtn != 0)
        {
            return ERR_SOCKET_SEND_FAILED;
        }
        cmdFrameCnt++;
    }
    else
    {
        return ERR_PARAM_VALUE;
    }

    return errcode;
}


/**
 * @brief 关节MIT控制结束
 * @param [in]  comType 指令下发类型；0-xmlrpc；1-UDP(对应机器人20007端口)
 * @return  错误码
 */
errno_t FRRobot::ServoMITEnd(int comType)
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

    if (comType == 0)
    {
        XmlRpcClient c(serverUrl, 20003);
        XmlRpcValue param, result;

        if (c.execute("ServoMITEnd", param, result))
        {
            errcode = int(result);
            if (0 != errcode)
            {
                logger_error("execute ServoMITEnd fail: %d.", errcode);
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
    }
    else if (comType == 1)
    {
        string cmdStr = string("ServoMITEnd()");
        FRAME frame(cmdFrameCnt, 1335, cmdStr);
        int rtn = udpCmdClient->SendFrame(PackFrame(frame));
        if (rtn != 0)
        {
            return ERR_SOCKET_SEND_FAILED;
        }
        cmdFrameCnt++;
    }
    else
    {
        return ERR_PARAM_VALUE;
    }

    return errcode;
}


/**
* @brief 关节MIT控制
* @param [in] posGain j1~j6关节位置增益
* @param [in] desPos j1~j6关节期望位置 单位:deg
* @param [in] velGain j1~j6关节速度增益
* @param [in] desVel j1~j6关节期望速度 单位:deg/s
* @param [in] torque_ff j1~j6前馈力矩 单位:Nm
* @param [in] interval 指令周期，单位s，范围[0.001~0.008]
* @param [in]  comType 指令下发类型；0-xmlrpc；1-UDP(对应机器人20007端口)
* @return 错误码
*/
errno_t FRRobot::ServoMIT(double posGain[6], double desPos[6], double velGain[6], double desVel[6], double torque_ff[6], double interval, int comType)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    if (GetSafetyCode() != 0)
    {
        return GetSafetyCode();
    }

    if (comType == 0)
    {
        int errcode = 0;
        XmlRpcClient c(serverUrl, 20003);
        XmlRpcValue param, result;

        param[0][0] = posGain[0];
        param[0][1] = posGain[1];
        param[0][2] = posGain[2];
        param[0][3] = posGain[3];
        param[0][4] = posGain[4];
        param[0][5] = posGain[5];
        param[1][0] = desPos[0];
        param[1][1] = desPos[1];
        param[1][2] = desPos[2];
        param[1][3] = desPos[3];
        param[1][4] = desPos[4];
        param[1][5] = desPos[5];
        param[2][0] = velGain[0];
        param[2][1] = velGain[1];
        param[2][2] = velGain[2];
        param[2][3] = velGain[3];
        param[2][4] = velGain[4];
        param[2][5] = velGain[5];
        param[3][0] = desVel[0];
        param[3][1] = desVel[1];
        param[3][2] = desVel[2];
        param[3][3] = desVel[3];
        param[3][4] = desVel[4];
        param[3][5] = desVel[5];
        param[4][0] = torque_ff[0];
        param[4][1] = torque_ff[1];
        param[4][2] = torque_ff[2];
        param[4][3] = torque_ff[3];
        param[4][4] = torque_ff[4];
        param[4][5] = torque_ff[5];
        param[5] = interval;

        if (c.execute("ServoMIT", param, result))
        {
            errcode = int(result);
            if (0 != errcode)
            {
                logger_error("execute ServoMIT fail: %d.", errcode);
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
    else if (comType == 1)
    {
        char posGainStr[128] = { 0 };
        snprintf(posGainStr, 128, "{%.3f,%.3f,%.3f,%.3f,%.3f,%.3f}", posGain[0], posGain[1], posGain[2], posGain[3], posGain[4], posGain[5]);
        char desPosStr[128] = { 0 };
        snprintf(desPosStr, 128, "{%.3f,%.3f,%.3f,%.3f,%.3f,%.3f}", desPos[0], desPos[1], desPos[2], desPos[3], desPos[4], desPos[5]);
        char velGainStr[128] = { 0 };
        snprintf(velGainStr, 128, "{%.3f,%.3f,%.3f,%.3f,%.3f,%.3f}", velGain[0], velGain[1], velGain[2], velGain[3], velGain[4], velGain[5]);
        char desVelStr[128] = { 0 };
        snprintf(desVelStr, 128, "{%.3f,%.3f,%.3f,%.3f,%.3f,%.3f}", desVel[0], desVel[1], desVel[2], desVel[3], desVel[4], desVel[5]);
        char torque_ffStr[128] = { 0 };
        snprintf(torque_ffStr, 128, "{%.3f,%.3f,%.3f,%.3f,%.3f,%.3f}", torque_ff[0], torque_ff[1], torque_ff[2], torque_ff[3], torque_ff[4], torque_ff[5]);


        string cmdStr = string("ServoMIT(") + posGainStr + "," + desPosStr + "," + velGainStr + "," + desVelStr + "," + torque_ffStr + "," + to_string(interval) + ")";
        FRAME frame(cmdFrameCnt, 1336, cmdStr);
        int rtn = udpCmdClient->SendFrame(PackFrame(frame));
        if (rtn != 0)
        {
            return ERR_SOCKET_SEND_FAILED;
        }
        cmdFrameCnt++;

        return ERR_SUCCESS;
    }
    else
    {
        return ERR_PARAM_VALUE;
    }

}


/**
 * @brief 设置摆动结束回周期零点
 * @param [in] flag 摆动结束是否回周期零点；0-不回周期零点；1-回周期零点
 * @return 错误码
 */
errno_t FRRobot::SetWeaveBackCenterConfig(int flag)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = flag;

    if (c.execute("SetWeaveBackCenterConfig", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute SetWeaveBackCenterConfig fail: %d.", errcode);
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
 * @brief 设置摆动实时偏移
 * @param [in] offset 实时偏移量[mm，°]
 * @return 错误码
 */
errno_t FRRobot::SetWeaveOffsetRT(DescPose offset)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    //XmlRpcClient c(serverUrl, 20003);
    //XmlRpcValue param, result;
    //param[0] = offset.tran.x;
    //param[1] = offset.tran.y;
    //param[2] = offset.tran.z;
    //param[3] = offset.rpy.rx;
    //param[4] = offset.rpy.ry;
    //param[5] = offset.rpy.rz;
    //if (c.execute("SetWeaveOffsetRT", param, result))
    //{
    //    errcode = int(result);
    //    if (0 != errcode)
    //    {
    //        logger_error("execute SetWeaveOffsetRT fail: %d.", errcode);
    //        c.close();
    //        return errcode;
    //    }
    //}
    //else
    //{
    //    c.close();
    //    return ERR_XMLRPC_CMD_FAILED;
    //}
    //c.close();

    const int BUFFER_SIZE = 1024 * 4;

    char paramStr[128] = "";
    sprintf(paramStr, "%f,%f,%f,%f,%f,%f", offset.tran.x, offset.tran.y, offset.tran.z, offset.rpy.rx, offset.rpy.ry, offset.rpy.rz);

    string cmdStr = string("SetWeaveOffsetRT(") + paramStr + ")";
    FRAME frame(cmdFrameCnt, 1368, cmdStr);
    string sendFrame = PackFrame(frame);

    memset(g_sendbuf, 0, BUFFER_SIZE * sizeof(char));
    strcpy(g_sendbuf, sendFrame.c_str());

    is_sendcmd = true;

    logger_info("SetWeaveOffsetRT(%s).", sendFrame);

    return 0;
}

/**
 * @brief 获取摆动结束回周期零点参数
 * @param [out] flag 摆动结束是否回周期零点；0-不回周期零点；1-回周期零点
 * @return 错误码
 */
errno_t FRRobot::GetWeaveBackCenterConfig(int& flag)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("GetWeaveBackCenterConfig", param, result))
    {
        errcode = int(result[0]);
        if (0 == errcode)
        {
            flag = (int)result[1];
        }
        else
        {
            logger_error("execute GetWeaveBackCenterConfig fail %d ", errcode);
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
 * @brief 工件坐标系点位转换开始
 * @param [in] workpieceID 工件号[0-14]
 * @return 错误码，成功返回0
 */
errno_t FRRobot::WorkPieceTrsfStart(int workpieceID)
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    param[0] = workpieceID;

    if (c.execute("WorkPieceTrsfStart", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute WorkPieceTrsfStart fail: %d.", errcode);
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
 * @brief 工件坐标系点位转换结束
 * @return 错误码，成功返回0
 */
errno_t FRRobot::WorkPieceTrsfEnd()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }

    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("WorkPieceTrsfEnd", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute WorkPieceTrsfEnd fail: %d.", errcode);
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
 * @brief 原地空运动
 * @return 错误码
 */
errno_t FRRobot::MoveStationary()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("MoveStationary", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute MoveStationary fail: %d.", errcode);
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
 * @brief 等待原地空运动完成
 * @return 错误码
 */
errno_t FRRobot::WaitStationaryMotionDone()
{
    if (IsSockError())
    {
        return g_sock_com_err;
    }
    int errcode = 0;
    XmlRpcClient c(serverUrl, 20003);
    XmlRpcValue param, result;

    if (c.execute("WaitStationaryMotionDone", param, result))
    {
        errcode = int(result);
        if (0 != errcode)
        {
            logger_error("execute WaitStationaryMotionDone fail: %d.", errcode);
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