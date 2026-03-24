#include "FRUdpClient.h"
#include "FrameHandle.h"

using namespace std;

FRUdpClient::FRUdpClient()
{
	
}

FRUdpClient::~FRUdpClient()
{
	runFlag = false;
#ifdef WIN32
	closesocket(fd);
	WSACleanup();
#else
    shutdown(fd, 2);
	close(fd);
#endif
}

int FRUdpClient::Connect(std::string IP, int port)
{
	robotIP = IP;
	robotPort = port;
#ifdef WIN32
	WORD wVersionRequested = MAKEWORD(2, 2);
	WSADATA data;
	WSAStartup(wVersionRequested, &data);
#endif
	fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(int));
	setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (char*)&timeout, sizeof(int));


#ifdef WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
    memset(&addrUDPServer, 0, sizeof(addrUDPServer));
    addrUDPServer.sin_family = AF_INET;
    addrUDPServer.sin_addr.S_un.S_addr = inet_addr(robotIP.c_str()); /// ������ip;
    addrUDPServer.sin_port = htons(robotPort);                      /// �������˿�;
#else

    memset(&addrUDPServer, 0, sizeof(addrUDPServer));
    addrUDPServer.sin_family = AF_INET;
    addrUDPServer.sin_port = htons(port);                 /// �������˿�;
    addrUDPServer.sin_addr.s_addr = inet_addr(robotIP.c_str()); /// ������ip;
#endif

	thread recvThread(&FRUdpClient::RobotUDPCmdRecvThread, this);
	recvThread.detach();

	return 0;
}

int FRUdpClient::Close()
{
	runFlag = false;

#ifdef WIN32
	closesocket(fd);
	WSACleanup();
#else
    shutdown(fd, 2);
	close(fd);
#endif

	return 0;
}

void FRUdpClient::RobotUDPCmdRecvThread()
{
	int nRecvNum = 0;
#ifdef WIN32
	int servoSize = sizeof(addrUDPServer);
#else
	socklen_t servoSize = sizeof(addrUDPServer);
#endif
	bool firstData = true;

	char recvBuf[2048] = { 0 };

	while (runFlag)
	{
		memset(recvBuf, 0, sizeof(recvBuf));
		nRecvNum = recvfrom(fd, recvBuf, sizeof(recvBuf), 0, (sockaddr*)&addrUDPServer, &servoSize);
		//printf("recv num is %d, %s\n", nRecvNum, recvBuf);
		if (nRecvNum <= 0)
		{
#ifdef WIN32
		    std::this_thread::sleep_for(std::chrono::milliseconds(300));
#else
    		usleep(300 * 1000);
#endif
			continue;
		}

		std::vector<std::string> allFrames = SplitFrame(std::string(recvBuf));
		for (int i = 0; i < allFrames.size(); i++)
		{
			FRAME recvFrame = UnpacketFrame(allFrames[i]);

			if (UdpRecvFrameCallBack == nullptr)
			{
				continue;
			}

			try 
			{
				UdpRecvFrameCallBack(COM_UDP, recvFrame.count, recvFrame.cmdID, recvFrame.contentLen, recvFrame.content);
			}
			catch(...) 
			{
				logger_error("Catch error in callback! we now set disable callback funtion! ");
				UdpRecvFrameCallBack = nullptr;
			}
		}
	}
}

int FRUdpClient::SetUDPCmdRpyCallback(void (*CallBack)(int, int, int, int, string))
{
	if (CallBack == nullptr)
	{
		return 1;
	}
	UdpRecvFrameCallBack = CallBack;
	return 0;
}

int FRUdpClient::SendFrame(string sendFrame)
{
	int rtn = sendto(fd, sendFrame.c_str(), sendFrame.length(), 0, (sockaddr*)&addrUDPServer, sizeof(addrUDPServer));
	if (rtn != sendFrame.length())
	{
		return 1;
	}
	
	return 0;
}
