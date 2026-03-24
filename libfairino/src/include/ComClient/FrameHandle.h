#pragma once

#include <iostream>
#include <vector>
#include <string>

#define COM_TCP 0
#define COM_UDP 1

typedef struct FRAME
{
	std::string head;
	int count;
	int cmdID;
	int contentLen;
	std::string content;
	std::string tail;

	FRAME()
	{
		head = "";
		count = 0;
		cmdID = 0;
		contentLen = 0;
		content = "";
		tail = "";
	}

	FRAME(int _count, int _cmdID, std::string _content)
	{
		head = "/f/b";
		count = _count;
		cmdID = _cmdID;
		contentLen = _content.length();
		content = _content;
		tail = "/b/f";
	}
}FRAME;

//   /f/b分包
std::vector<std::string> SplitFrame(const std::string& data);

//   |||分包
FRAME UnpacketFrame(std::string frameStr);

//   获取lua程序500错误码
void GetRobotLUAProgram500ErrCode(const std::string& content, int& errLinNum, int& luaErrCode);

//   组数据帧
std::string PackFrame(FRAME frame);

bool VerifyFrame(const std::string& frameStr);