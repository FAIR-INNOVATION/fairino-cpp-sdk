#pragma once

#include "robot_types.h"
#include "FRTcpClient.h"
#include <mutex>
#include <memory>

class FRCNDEClient
{
public:
	FRCNDEClient(std::shared_ptr<ROBOT_STATE_PKG> pkg, int* comErr);
	~FRCNDEClient();

	int Connect(std::string IP, int port);
	int SetReConnectParam(bool enable, int reconnectTime, int period);
	bool GetReConnectEnable();
	bool GetReConnState();
	int Close();

	int SendCNDEOutputConfig();
	int SetCNDEStart();
	int SetCNDEStop();

	int SetCNDEStateConfig(std::vector<RobotState> states, int period);
	int AddCNDEState(RobotState state);
	int DeleteCNDEState(RobotState state);
	int SetCNDEStatePeriod(int period);
	int GetCNDEStateConfig(std::vector<RobotState>& states, int& period);



	int GetConfigTypeSize(std::string typeStr);
	int GetSingleConfigTypeSize(std::string typeStr);

private:
	void RecvRobotStateThread();
	void InitAllStates();



private:

	bool robotStateRunFlag = false;
	std::shared_ptr <FRTcpClient> rtClient;
	std::shared_ptr<ROBOT_STATE_PKG> robotStatePkg;

	int robotStatePeriod = 8;
	uint8_t sendCount = 0;
	int* sockComErr = nullptr;

	std::mutex recvCNDEPkgMutex;

	std::map<RobotState, std::tuple<std::string, int, std::string, std::string>> allStates;  //
//           枚举键                 cnde状态名称 结构体索引  结构体中的数据类型  cnde中的数据类型

	std::vector<RobotState> configStates;
};
