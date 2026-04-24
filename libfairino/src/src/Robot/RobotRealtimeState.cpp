#include "robot.h"
#include "robot_types.h"
#include "FRCNDEClient.h"

/**
 * @brief 配置近期人状态反馈
 * @param [in] states 可配置状态列表
 * @param [in] period 状态反馈周期
 * @return 错误码
 */
errno_t FRRobot::SetRobotRealtimeStateConfig(std::vector<RobotState> states, int period)
{
	if (IsSockError())
	{
		return g_sock_com_err;
	}

	return cndeClient->SetCNDEStateConfig(states, period);
}

/**
 * @brief 添加一个机器人状态
 * @param [in] state 可配置状态
 * @return 错误码
 */
errno_t FRRobot::AddRobotRealtimeState(RobotState state)
{
	if (IsSockError())
	{
		return g_sock_com_err;
	}

	return cndeClient->AddCNDEState(state);
}

/**
 * @brief 删除一个机器人状态
 * @param [in] state 可配置状态
 * @return 错误码
 */
errno_t FRRobot::DeleteRobotRealtimeState(RobotState state)
{
	if (IsSockError())
	{
		return g_sock_com_err;
	}

	return cndeClient->DeleteCNDEState(state);
}


/**
 * @brief 设置机器人可配置状态反馈周期
 * @param [in] period 可配置状态反馈周期 4-1000ms
 * @return 错误码
 */
errno_t FRRobot::SetRobotRealtimeStatePeriod(int period)
{
	if (IsSockError())
	{
		return g_sock_com_err;
	}

	return cndeClient->SetCNDEStatePeriod(period);
}


/**
 * @brief 设置机器人可配置状态反馈周期
 * @param [out] states 可配置状态列表
 * @param [out] period 可配置状态反馈周期
 * @return 错误码
 */
errno_t FRRobot::GetRobotRealtimeStateConfig(std::vector<RobotState>& states, int& period)
{
	if (IsSockError())
	{
		return g_sock_com_err;
	}

	return cndeClient->GetCNDEStateConfig(states, period);
}